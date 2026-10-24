//! Zolt addition, no Jolt file: tests of the SIMD contact solver (Zolt/Physics/Constraints/ContactConstraintSimd.zig,
//! Roadmap Phase 11). Jolt has no SIMD contact solver.
//! Status: complete
//!
//! - The SIMD solver must give the bits of ContactConstraintManager.solveVelocityConstraints. Each test builds a batch of
//!   random contact constraints by hand in a constraint buffer, twice from the same seed, and solves one copy with the
//!   scalar solver and the other with the SIMD solver for several iterations. After every iteration every velocity
//!   and every byte of every constraint (the lambdas, and the members that must not change) must be equal.
//! - The random batches have dynamic vs dynamic contacts with 1 to 4 points (and a few without points), friction on and
//!   off, lambdas of both signs, -0.0, exact zeros, tiny and large values, locked translation and rotation axes, zero
//!   inverse masses and inertias, and contacts against static and kinematic bodies. The members that Jolt leaves
//!   uninitialized (a deactivated part) hold random bits, also NaN, which the solvers must not use.
//! - Physical batches calculate the parts like the manager does. Edge batches write every member directly with signed
//!   zeros, tiny values and small integers, so that the sign of a zero reaches the lambdas (Vec3.dot's `+ 0.0`,
//!   math.max of -0.0) and tiny lambdas meet the FLT_MIN of the friction clamp.
//! - Huge batches are edge batches with lambdas and velocities of up to 3e19. The squares of the friction lambdas
//!   overflow to +inf: the scale-back runs with total_lambda_sq = +inf, or not at all when the square of the limit is
//!   +inf too. They are solved for one iteration only, so that no inf - inf makes a NaN.
//! - The dispatcher: a batch whose contacts share dynamic bodies (like the non parallel split) must take the scalar
//!   path. The test proves that it would notice otherwise: the SIMD solver gives other bits for such a batch. With the
//!   determinism log even a parallel batch must take the scalar path (a test that the determinism log binary runs).
//! - Bite proofs (each one fails these tests): another association in the jacobian sum, a velocity step without the
//!   lane mask, the friction clamp without FLT_MIN, a point mask with `<=`, Vec3.dot without `+ 0.0`, math.max
//!   that returns `b` for equal values, and the friction scale-back test in f64 (only the huge batches see it). `@max`
//!   instead of math.max is not visible on x86-64 with finite inputs (LLVM emits the same maxps there, only NaN
//!   differs), it stays forbidden. Without `!DeterminismLog.enabled` in ContactConstraintSimd.enabled the determinism
//!   log test fails.

const std = @import("std");
const zolt = @import("zolt");

const AllowedDOFs = zolt.AllowedDOFs;
const Body = zolt.Body;
const ContactConstraintManager = zolt.ContactConstraintManager;
const ContactConstraintSimd = zolt.ContactConstraintSimd;
const Core = zolt.Core;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const MotionProperties = zolt.MotionProperties;
const MotionType = zolt.MotionType;
const Mt19937 = zolt.Mt19937;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const Allocator = std.mem.Allocator;
const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;

/// Random numbers for a scene: the same seed gives the same scene
const Random = struct {
    rng: Mt19937,

    /// The values of `edge`: the edge cases, or mostly signed zeros (`Values.zeros`)
    edge_values: []const f32 = &edge_cases,

    /// `edgeOrHuge` gives a value of `huge_values` (`Values.huge`)
    huge: bool = false,

    const edge_cases = [_]f32{ 0.0, -0.0, 1.0e-20, -1.0e-20, 1.0e-30, -1.0e-30, 1.0, -1.0, 2.0, -0.5 };
    const signed_zeros = [_]f32{ 0.0, -0.0, 0.0, -0.0, 0.0, -0.0, 1.0, -1.0 };

    /// Huge values and some small ones. The square of 3e19 overflows to +inf, the square of 1e18 does not.
    const huge_values = [_]f32{ 1.0e18, -1.0e18, 3.0e19, -3.0e19, 0.0, -0.0, 1.0, -0.5 };

    fn int(self: *Random, n: u32) u32 {
        return self.rng.next() % n;
    }

    fn chance(self: *Random, percent: u32) bool {
        return self.int(100) < percent;
    }

    /// Uniform in [lo, hi)
    fn float(self: *Random, lo: f32, hi: f32) f32 {
        const t = @as(f32, @floatFromInt(self.rng.next() >> 8)) / (1 << 24);
        return lo + (hi - lo) * t;
    }

    /// About `scale` with a random sign, sometimes 0, -0.0, a tiny or a large value
    fn value(self: *Random, scale: f32) f32 {
        return switch (self.int(20)) {
            0 => 0.0,
            1 => -0.0,
            2 => self.float(-1.0, 1.0) * scale * 1.0e-30,
            3 => self.float(-1.0, 1.0) * scale * 1.0e3,
            else => self.float(-1.0, 1.0) * scale,
        };
    }

    /// Like `value`, but not negative (a non-penetration lambda, a distance)
    fn positive(self: *Random, scale: f32) f32 {
        return switch (self.int(20)) {
            0 => 0.0,
            1 => -0.0,
            2 => self.float(0.0, 1.0) * scale * 1.0e-30,
            3 => self.float(0.0, 1.0) * scale * 1.0e3,
            else => self.float(0.0, 1.0) * scale,
        };
    }

    /// A value of the edge cases: signed zeros, tiny values whose squares and products are subnormal or 0, small integers
    fn edge(self: *Random) f32 {
        return self.edge_values[self.int(@intCast(self.edge_values.len))];
    }

    fn edgeVec3(self: *Random) Vec3 {
        return Vec3.init(self.edge(), self.edge(), self.edge());
    }

    /// A lambda or a velocity component of an edge batch: an edge value, or with `huge` a value of `huge_values`
    fn edgeOrHuge(self: *Random) f32 {
        if (self.huge) return huge_values[self.int(huge_values.len)];
        return self.edge();
    }

    fn edgeOrHugeVec3(self: *Random) Vec3 {
        return Vec3.init(self.edgeOrHuge(), self.edgeOrHuge(), self.edgeOrHuge());
    }

    fn edgeFloat3(self: *Random) Float3 {
        return .{ .x = self.edge(), .y = self.edge(), .z = self.edge() };
    }

    /// An effective mass for the edge cases (0: an inactive part whose other members are still finite)
    fn edgeMass(self: *Random) f32 {
        const values = [_]f32{ 0.0, 1.0e-20, 1.0, 2.0 };
        return values[self.int(values.len)];
    }

    fn vec3(self: *Random, scale: f32) Vec3 {
        return Vec3.init(self.value(scale), self.value(scale), self.value(scale));
    }

    /// A unit vector, sometimes along an axis with signed zeros in the other components
    fn unitVec3(self: *Random) Vec3 {
        if (self.chance(15)) {
            var c = [3]f32{ if (self.chance(50)) 0.0 else -0.0, if (self.chance(50)) 0.0 else -0.0, if (self.chance(50)) 0.0 else -0.0 };
            c[self.int(3)] = if (self.chance(50)) 1.0 else -1.0;
            return Vec3.init(c[0], c[1], c[2]);
        }
        while (true) {
            const v = Vec3.init(self.float(-1, 1), self.float(-1, 1), self.float(-1, 1));
            if (v.lengthSq() > 0.01) return v.normalized();
        }
    }

    /// An inverse inertia: a random rotation of a diagonal with sometimes zero entries
    fn invInertia(self: *Random) Mat44 {
        const d = Vec3.init(
            if (self.chance(15)) 0.0 else self.float(0.0, 3.0),
            if (self.chance(15)) 0.0 else self.float(0.0, 3.0),
            if (self.chance(15)) 0.0 else self.float(0.0, 3.0),
        );
        const r = Mat44.rotation(self.unitVec3(), self.float(0.0, 6.0));
        return r.mul(Mat44.scaleVec3(d)).mul(r.transposed3x3());
    }

    fn allowedDOFs(self: *Random) AllowedDOFs {
        if (self.chance(70)) return .all;
        var dofs: AllowedDOFs = .all;
        dofs.translation_x = self.chance(60);
        dofs.translation_y = self.chance(60);
        dofs.translation_z = self.chance(60);
        dofs.rotation_x = self.chance(70);
        dofs.rotation_y = self.chance(70);
        dofs.rotation_z = self.chance(70);
        return dofs;
    }
};

/// How the contacts use the dynamic bodies
const Sharing = enum {
    /// Every dynamic body is in at most one contact (a batch of a parallel split)
    parallel,
    /// The contacts share dynamic bodies (the non parallel split)
    shared,
};

/// The values in the constraints
const Values = enum {
    /// The parts are calculated like ContactConstraintManager does, from random masses, inertias and lever arms
    physical,
    /// Every member is written directly with edge values: signed zeros, tiny values, small integers. The solvers must
    /// give the same bits for any finite input, so these need not be physical.
    edge,
    /// Like `edge` with mostly +0.0 and -0.0: the signs of zero reach the lambdas through every operation
    zeros,
    /// Like `edge` with huge lambdas and velocities (`Random.huge_values`): the squares of the friction lambdas and of
    /// their limit overflow to +inf. Solve these for one iteration only: a second one could make inf - inf, a NaN.
    huge,
};

/// A batch of contact constraints in a constraint buffer, with the bodies it uses
const Scene = struct {
    allocator: Allocator,
    bodies: []Body,
    motion_properties: []MotionProperties,
    buffer: []align(16) u8,
    offsets: []u32,
    sizes: []u32,
    manager: ContactConstraintManager,

    /// Build a scene: `num_constraints` contacts, `seed` decides everything else
    fn create(allocator: Allocator, seed: u32, num_constraints: u32, sharing: Sharing, values: Values) !*Scene {
        var random: Random = .{ .rng = Mt19937.init(seed), .edge_values = if (values == .zeros) &Random.signed_zeros else &Random.edge_cases, .huge = values == .huge };

        const self = try allocator.create(Scene);
        errdefer allocator.destroy(self);
        self.allocator = allocator;

        // Bodies: a dynamic pair for every contact, a few static and kinematic ones
        const num_dynamic = 2 * num_constraints + 2;
        const num_other = 6;
        const num_bodies = num_dynamic + num_other;
        self.bodies = try allocator.alloc(Body, num_bodies);
        errdefer allocator.free(self.bodies);
        self.motion_properties = try allocator.alloc(MotionProperties, num_bodies);
        errdefer allocator.free(self.motion_properties);
        for (self.bodies, self.motion_properties, 0..) |*body, *mp, i| {
            const motion_type: MotionType = if (i < num_dynamic) .dynamic else if (i < num_dynamic + num_other / 2) .kinematic else .static;
            mp.* = .{
                .linear_velocity = if (values == .physical) random.vec3(10.0) else random.edgeOrHugeVec3(),
                .angular_velocity = if (values == .physical) random.vec3(10.0) else random.edgeOrHugeVec3(),
                .allowed_dofs = random.allowedDOFs(),
            };
            if (Core.enable_asserts) mp.cached_motion_type = motion_type;
            body.* = .{ .motion_type = motion_type, .motion_properties = if (motion_type == .static) null else mp };
        }

        // The constraint buffer, random bytes where Jolt leaves memory uninitialized
        self.buffer = try allocator.alignedAlloc(u8, .@"16", num_constraints * ContactConstraintManager.max_constraint_size);
        errdefer allocator.free(self.buffer);
        for (self.buffer) |*b| b.* = @truncate(random.rng.next());
        self.offsets = try allocator.alloc(u32, num_constraints);
        errdefer allocator.free(self.offsets);
        self.sizes = try allocator.alloc(u32, num_constraints);
        errdefer allocator.free(self.sizes);

        var next_dynamic: u32 = 0;
        var offset: u32 = 0;
        for (self.offsets, self.sizes) |*constraint_offset, *constraint_size| {
            // The motion types of the bodies, mostly dynamic vs dynamic
            const pair: [2]MotionType = switch (random.int(20)) {
                0 => .{ .dynamic, .static },
                1 => .{ .static, .dynamic },
                2 => .{ .dynamic, .kinematic },
                3 => .{ .kinematic, .dynamic },
                else => .{ .dynamic, .dynamic },
            };
            var body_indices: [2]u32 = undefined;
            for (pair, &body_indices) |motion_type, *index| {
                index.* = switch (motion_type) {
                    .dynamic => switch (sharing) {
                        .parallel => blk: {
                            next_dynamic += 1;
                            break :blk next_dynamic - 1;
                        },
                        .shared => random.int(4),
                    },
                    .kinematic => num_dynamic + random.int(num_other / 2),
                    .static => num_dynamic + num_other / 2 + random.int(num_other / 2),
                };
            }
            if (body_indices[0] == body_indices[1]) body_indices[1] = (body_indices[1] + 1) % 4;

            // Mostly 1 to 4 contact points, sometimes none
            const num_points: u32 = if (random.chance(2)) 0 else 1 + random.int(ContactConstraintManager.max_contact_points);

            constraint_offset.* = offset;
            constraint_size.* = switch (pair[0]) {
                inline else => |type1| switch (pair[1]) {
                    inline else => |type2| if (comptime type1 == .dynamic or type2 == .dynamic)
                        self.buildConstraint(type1, type2, offset, &self.bodies[body_indices[0]], &self.bodies[body_indices[1]], num_points, values, &random)
                    else
                        unreachable,
                },
            };
            offset += constraint_size.*;
        }

        self.manager = .empty;
        self.manager.constraints = .{ .items = self.buffer };
        return self;
    }

    fn destroy(self: *Scene) void {
        self.manager.constraints = .empty;
        self.allocator.free(self.sizes);
        self.allocator.free(self.offsets);
        self.allocator.free(self.buffer);
        self.allocator.free(self.motion_properties);
        self.allocator.free(self.bodies);
        self.allocator.destroy(self);
    }

    /// Write a ContactConstraint(type1, type2) at `offset` like ContactConstraintManager does (calculateConstraintProperties
    /// of the parts, deactivated parts keep their random bytes), returns its size
    fn buildConstraint(self: *Scene, comptime type1: MotionType, comptime type2: MotionType, offset: u32, body1: *Body, body2: *Body, num_points: u32, values: Values, random: *Random) u32 {
        const Constraint = ContactConstraintManager.ContactConstraint(type1, type2);
        const size: u32 = @intCast(std.mem.alignForward(usize, @sizeOf(Constraint) + @as(usize, num_points) * @sizeOf(Constraint.ContactPoint) - @sizeOf(Constraint.ContactPoint), @alignOf(Constraint)));
        const constraint: *Constraint = @ptrCast(@alignCast(self.buffer.ptr + offset));
        if (values != .physical) {
            buildEdgeConstraint(Constraint, constraint, body1, body2, num_points, random);
            return size;
        }

        const normal = random.unitVec3();
        const tangent1 = normal.getNormalizedPerpendicular();
        const tangent2 = normal.cross(tangent1);
        const friction: f32 = if (random.chance(25)) 0.0 else random.float(0.0, 1.5);
        const inv_mass1: f32 = if (type1 != .dynamic or random.chance(10)) 0.0 else random.float(0.0, 2.0);
        const inv_mass2: f32 = if (type2 != .dynamic or random.chance(10)) 0.0 else random.float(0.0, 2.0);
        const inv_i1 = if (type1 == .dynamic) random.invInertia() else Mat44.zero();
        const inv_i2 = if (type2 == .dynamic) random.invInertia() else Mat44.zero();

        constraint.base.body1 = body1;
        constraint.base.body2 = body2;
        constraint.base.sort_key = random.rng.next();
        normal.storeFloat3(&constraint.base.world_space_normal);
        tangent1.storeFloat3(&constraint.base.world_space_tangent1);
        tangent2.storeFloat3(&constraint.base.world_space_tangent2);
        constraint.base.combined_friction = friction;
        constraint.base.inv_mass1 = inv_mass1;
        constraint.base.inv_inertia_scale1 = 1.0;
        constraint.base.inv_mass2 = inv_mass2;
        constraint.base.inv_inertia_scale2 = 1.0;
        constraint.base.num_contact_points = num_points;

        // Friction (CalculateFrictionConstraintProperties)
        if (friction > 0.0) {
            const r1 = random.vec3(1.0);
            const r2 = random.vec3(1.0);
            constraint.friction_constraint1.calculateConstraintProperties(inv_mass1, inv_i1, r1, inv_mass2, inv_i2, r2, tangent1, .{ .bias = random.value(2.0) });
            constraint.friction_constraint2.calculateConstraintProperties(inv_mass1, inv_i1, r1, inv_mass2, inv_i2, r2, tangent2, .{ .bias = random.value(2.0) });
            if (constraint.friction_constraint1.isActive()) constraint.friction_constraint1.setTotalLambda(random.value(5.0));
            if (constraint.friction_constraint2.isActive()) constraint.friction_constraint2.setTotalLambda(random.value(5.0));
            if (num_points > 1) {
                constraint.angular_friction_constraint.calculateConstraintProperties(inv_i1, inv_i2, normal, .{ .bias = random.value(1.0) });
                if (constraint.angular_friction_constraint.isActive()) constraint.angular_friction_constraint.setTotalLambda(random.value(5.0));
            } else constraint.angular_friction_constraint.deactivate();
        } else {
            constraint.friction_constraint1.deactivate();
            constraint.friction_constraint2.deactivate();
            constraint.angular_friction_constraint.deactivate();
        }

        // Non-penetration (CalculateNonPenetrationConstraintProperties)
        for (constraint.getContactPoints()[0..num_points]) |*wcp| {
            wcp.non_penetration_constraint.calculateConstraintProperties(inv_mass1, inv_i1, random.vec3(1.0), inv_mass2, inv_i2, random.vec3(1.0), normal, .{ .bias = random.value(5.0) });
            if (wcp.non_penetration_constraint.isActive()) wcp.non_penetration_constraint.setTotalLambda(random.positive(5.0));
            wcp.distance_to_friction_center = random.positive(1.0);
        }

        return size;
    }

    /// Write a constraint with edge values in every member (see `Values.edge`), deactivated parts keep their random bytes
    fn buildEdgeConstraint(comptime Constraint: type, constraint: *Constraint, body1: *Body, body2: *Body, num_points: u32, random: *Random) void {
        const type1 = Constraint.motion_type1;
        const type2 = Constraint.motion_type2;
        constraint.base.body1 = body1;
        constraint.base.body2 = body2;
        constraint.base.sort_key = random.rng.next();
        const normal = if (random.chance(50)) random.edgeVec3() else random.unitVec3();
        normal.storeFloat3(&constraint.base.world_space_normal);
        constraint.base.world_space_tangent1 = random.edgeFloat3();
        constraint.base.world_space_tangent2 = random.edgeFloat3();
        const frictions = [_]f32{ 0.0, 1.0e-20, 1.0, 2.0 };
        const friction = frictions[random.int(frictions.len)];
        constraint.base.combined_friction = friction;
        const masses = [_]f32{ 0.0, 1.0e-20, 1.0, 2.0 };
        constraint.base.inv_mass1 = if (type1 == .dynamic) masses[random.int(masses.len)] else 0.0;
        constraint.base.inv_inertia_scale1 = 1.0;
        constraint.base.inv_mass2 = if (type2 == .dynamic) masses[random.int(masses.len)] else 0.0;
        constraint.base.inv_inertia_scale2 = 1.0;
        constraint.base.num_contact_points = num_points;

        if (friction > 0.0) {
            writeEdgePart(&constraint.friction_constraint1, random);
            writeEdgePart(&constraint.friction_constraint2, random);
            if (num_points > 1) {
                const part = &constraint.angular_friction_constraint;
                part.base1.base.effective_mass = random.edgeMass();
                part.base1.base.total_lambda = random.edgeOrHuge();
                if (comptime type1 == .dynamic) part.base1.inv_i1_axis = random.edgeFloat3();
                if (comptime type2 == .dynamic) part.base2.inv_i2_axis = random.edgeFloat3();
                part.bias = random.edge();
            } else constraint.angular_friction_constraint.deactivate();
        } else {
            constraint.friction_constraint1.deactivate();
            constraint.friction_constraint2.deactivate();
            constraint.angular_friction_constraint.deactivate();
        }

        for (constraint.getContactPoints()[0..num_points]) |*wcp| {
            writeEdgePart(&wcp.non_penetration_constraint, random);
            const distances = [_]f32{ 0.0, 1.0e-20, 1.0 };
            wcp.distance_to_friction_center = distances[random.int(distances.len)];
        }
    }

    /// Write every member of a ContactConstraintPart with edge values
    fn writeEdgePart(part: anytype, random: *Random) void {
        const Part = @TypeOf(part.*);
        part.base1.effective_mass = random.edgeMass();
        part.base1.total_lambda = random.edgeOrHuge();
        if (comptime Part.motion_type1 != .static) part.base1.r1_plus_u_x_axis = random.edgeFloat3();
        if (comptime Part.motion_type1 == .dynamic) part.base1.inv_i1_r1_plus_u_x_axis = random.edgeFloat3();
        if (comptime Part.motion_type2 != .static) part.base2.r2_x_axis = random.edgeFloat3();
        if (comptime Part.motion_type2 == .dynamic) part.base2.inv_i2_r2_x_axis = random.edgeFloat3();
        part.bias = random.edge();
    }

    fn constraintOffsets(self: *const Scene) []const u32 {
        return self.offsets;
    }

    /// A velocity of a body is NaN
    fn hasNaNVelocity(self: *const Scene) bool {
        for (self.motion_properties) |*mp|
            if (mp.linear_velocity.isNaN() or mp.angular_velocity.isNaN()) return true;
        return false;
    }
};

/// How often the friction lambdas of the dynamic vs dynamic contacts of `Values.huge` batches overflow
const FrictionOverflow = struct {
    /// total_lambda_sq is +inf and the square of the limit is not: the scale-back gives scale = max / sqrt(+inf) = 0
    scaled_to_zero: u32 = 0,
    /// total_lambda_sq and the square of the limit are +inf: no scale-back, the huge lambdas stay
    both_infinite: u32 = 0,

    /// Count the contacts of a scene before it is solved, with the operations of the scalar solver. In a parallel batch
    /// every contact sees the velocities of the scene.
    fn add(self: *FrictionOverflow, scene: *const Scene) void {
        const Constraint = ContactConstraintManager.ContactConstraint(.dynamic, .dynamic);
        const inf = std.math.inf(f32);
        for (scene.offsets) |offset| {
            const constraint: *const Constraint = @ptrCast(@alignCast(scene.buffer.ptr + offset));
            const body1 = constraint.base.body1;
            const body2 = constraint.base.body2;
            if (!body1.isDynamic() or !body2.isDynamic()) continue;
            if (!constraint.friction_constraint1.isActive() and !constraint.friction_constraint2.isActive()) continue;

            const mp1 = body1.motion_properties.?;
            const mp2 = body2.motion_properties.?;
            const t = constraint.base.getTangents();
            const lambda1 = constraint.friction_constraint1.solveVelocityConstraintGetTotalLambda(mp1.linear_velocity, mp1.angular_velocity, mp2.linear_velocity, mp2.angular_velocity, t.tangent1);
            const lambda2 = constraint.friction_constraint2.solveVelocityConstraintGetTotalLambda(mp1.linear_velocity, mp1.angular_velocity, mp2.linear_velocity, mp2.angular_velocity, t.tangent2);
            var max_linear_lambda: f32 = 0.0;
            for (constraint.getContactPointsConst()[0..constraint.base.num_contact_points]) |*wcp|
                max_linear_lambda += wcp.non_penetration_constraint.getTotalLambda();
            max_linear_lambda *= constraint.base.combined_friction;

            if (math.square(lambda1) + math.square(lambda2) != inf) continue;
            if (math.square(max_linear_lambda) + math.flt_min == inf)
                self.both_infinite += 1
            else
                self.scaled_to_zero += 1;
        }
    }
};

/// Fail unless the two copies of a scene have the same bits: every velocity, every byte of every constraint after the
/// body pointers. `report`: print the first difference.
fn expectSameBits(a: *const Scene, b: *const Scene, seed: u32, iteration: usize, report: bool) !void {
    for (a.motion_properties, b.motion_properties, 0..) |*mpa, *mpb, i| {
        const lva: [4]u32 = @bitCast(mpa.linear_velocity.value);
        const lvb: [4]u32 = @bitCast(mpb.linear_velocity.value);
        const ava: [4]u32 = @bitCast(mpa.angular_velocity.value);
        const avb: [4]u32 = @bitCast(mpb.angular_velocity.value);
        if (!std.mem.eql(u32, &lva, &lvb) or !std.mem.eql(u32, &ava, &avb)) {
            if (report) std.debug.print("seed {d} iteration {d}: velocity of body {d} differs: {any} {any} vs {any} {any}\n", .{ seed, iteration, i, lva, ava, lvb, avb });
            return error.TestExpectedEqual;
        }
    }
    const body_pointers = 2 * @sizeOf(*Body);
    for (a.offsets, a.sizes, 0..) |offset, size, i| {
        const bytes_a = a.buffer[offset + body_pointers .. offset + size];
        const bytes_b = b.buffer[offset + body_pointers .. offset + size];
        if (std.mem.indexOfDiff(u8, bytes_a, bytes_b)) |byte| {
            if (report) std.debug.print("seed {d} iteration {d}: constraint {d} differs at byte {d}\n", .{ seed, iteration, i, body_pointers + byte });
            return error.TestExpectedEqual;
        }
    }
}

/// Solve `scalar` with ContactConstraintManager.solveVelocityConstraints and `simd` with `solve` for some iterations
/// and compare the bits after every iteration. Returns false at the first difference when `allow_difference`.
fn solveAndCompare(scalar: *Scene, simd: *Scene, comptime solve: anytype, seed: u32, num_iterations: usize, allow_difference: bool) !bool {
    for (0..num_iterations) |iteration| {
        const applied_scalar = scalar.manager.solveVelocityConstraints(scalar.constraintOffsets());
        const applied_simd = solve(&simd.manager, simd.constraintOffsets());
        if (allow_difference) {
            expectSameBits(scalar, simd, seed, iteration, false) catch return false;
            if (applied_scalar != applied_simd) return false;
        } else {
            try expectSameBits(scalar, simd, seed, iteration, true);
            try expectEqual(applied_scalar, applied_simd);
        }
    }
    return true;
}

test "ContactConstraintSimd: the SIMD solver gives the bits of the scalar solver (random parallel batches)" {
    if (!ContactConstraintSimd.supported) return error.SkipZigTest;
    const allocator = std.testing.allocator;

    // Many small batches (a batch of the splitter has 16 contacts) and some larger than the 64 the solver sorts at once
    var seed: u32 = 1;
    while (seed <= 400) : (seed += 1) {
        const num_constraints: u32 = if (seed % 50 == 0) 150 else 1 + seed % 40;
        const scalar = try Scene.create(allocator, seed, num_constraints, .parallel, .physical);
        defer scalar.destroy();
        const simd = try Scene.create(allocator, seed, num_constraints, .parallel, .physical);
        defer simd.destroy();
        try expectSameBits(scalar, simd, seed, 0, true);
        _ = try solveAndCompare(scalar, simd, ContactConstraintSimd.solveVelocityConstraints, seed, 6, false);
    }
}

test "ContactConstraintSimd: the SIMD solver gives the bits of the scalar solver (signed zeros and tiny values)" {
    if (!ContactConstraintSimd.supported) return error.SkipZigTest;
    const allocator = std.testing.allocator;

    // Edge values in every member: -0.0 must stay -0.0 through Vec3.dot's `+ 0.0`, math.max and the masks, tiny
    // lambdas must meet the FLT_MIN of the friction clamp
    for ([_]Values{ .edge, .zeros }) |values| {
        var seed: u32 = 1;
        while (seed <= 400) : (seed += 1) {
            const num_constraints: u32 = 1 + seed % 24;
            const scalar = try Scene.create(allocator, seed, num_constraints, .parallel, values);
            defer scalar.destroy();
            const simd = try Scene.create(allocator, seed, num_constraints, .parallel, values);
            defer simd.destroy();
            try expectSameBits(scalar, simd, seed, 0, true);
            _ = try solveAndCompare(scalar, simd, ContactConstraintSimd.solveVelocityConstraints, seed, 4, false);
        }
    }
}

test "ContactConstraintSimd: the SIMD solver gives the bits of the scalar solver (huge values)" {
    if (!ContactConstraintSimd.supported) return error.SkipZigTest;
    const allocator = std.testing.allocator;

    // Lambdas and velocities of up to 3e19 (Values.huge): both overflow cases of the friction scale-back must happen
    var overflow: FrictionOverflow = .{};
    var seed: u32 = 1;
    while (seed <= 400) : (seed += 1) {
        const num_constraints: u32 = 1 + seed % 24;
        const scalar = try Scene.create(allocator, seed, num_constraints, .parallel, .huge);
        defer scalar.destroy();
        const simd = try Scene.create(allocator, seed, num_constraints, .parallel, .huge);
        defer simd.destroy();
        overflow.add(scalar);
        try expectSameBits(scalar, simd, seed, 0, true);
        _ = try solveAndCompare(scalar, simd, ContactConstraintSimd.solveVelocityConstraints, seed, 1, false);

        // One iteration makes no inf - inf: the bits of a NaN could depend on the order of the operands
        try expect(!scalar.hasNaNVelocity());
    }
    try expect(overflow.scaled_to_zero > 0);
    try expect(overflow.both_infinite > 0);
}

/// Solve batches whose contacts share dynamic bodies with solveBatch(`parallel`) and with the scalar solver, and fail
/// unless the bits are equal. Also fail unless the SIMD solver gives other bits for some of these batches, so that
/// the comparison shows which solver solveBatch used.
fn expectScalarSolveBatch(comptime parallel: bool) !void {
    const allocator = std.testing.allocator;

    // The contacts share 4 dynamic bodies. The scalar solver solves them one after the other, the SIMD solver would
    // read the same old velocities in several lanes: the bits differ for some seeds.
    const solveBatch = struct {
        fn solve(manager: *ContactConstraintManager, constraint_offsets: []const u32) bool {
            return ContactConstraintSimd.solveBatch(manager, constraint_offsets, parallel);
        }
    }.solve;
    var num_simd_differences: u32 = 0;
    var seed: u32 = 1;
    while (seed <= 100) : (seed += 1) {
        const num_constraints = 8 + seed % 9;
        {
            const scalar = try Scene.create(allocator, seed, num_constraints, .shared, .physical);
            defer scalar.destroy();
            const batch = try Scene.create(allocator, seed, num_constraints, .shared, .physical);
            defer batch.destroy();
            _ = try solveAndCompare(scalar, batch, solveBatch, seed, 4, false);
        }

        if (ContactConstraintSimd.supported) {
            const scalar = try Scene.create(allocator, seed, num_constraints, .shared, .physical);
            defer scalar.destroy();
            const simd = try Scene.create(allocator, seed, num_constraints, .shared, .physical);
            defer simd.destroy();
            if (!try solveAndCompare(scalar, simd, ContactConstraintSimd.solveVelocityConstraints, seed, 4, true))
                num_simd_differences += 1;
        }
    }
    if (ContactConstraintSimd.supported)
        try expect(num_simd_differences > 0);
}

test "ContactConstraintSimd: solveBatch solves a non parallel batch with the scalar solver" {
    try expectScalarSolveBatch(false);
}

test "DeterminismLog: solveBatch solves a parallel batch with the scalar solver when the log is on" {
    // The SIMD solver solves the contacts of a batch in another order (the others first, then the dynamic vs dynamic
    // ones by their number of points). So the ApplyLinearVelocityStep and ApplyAngularVelocityStep lines of the log
    // would not be in Jolt's order. "DeterminismLog" in the name: the unit test binary with the log runs this test
    // without a filter (build.zig).
    if (!Core.determinism_log) return error.SkipZigTest; // Needs -Ddeterminism_log=true (build.zig runs it)
    try expect(!ContactConstraintSimd.enabled);
    try expectScalarSolveBatch(true);
}

test "ContactConstraintSimd: solveBatch uses the SIMD solver for a parallel batch when it is enabled" {
    const allocator = std.testing.allocator;

    // A parallel batch gives the scalar bits either way, so compare with a batch that shares bodies: solveBatch with
    // parallel = true gives the bits of the SIMD solver when it is enabled and those of the scalar solver otherwise.
    const solveParallel = struct {
        fn solve(manager: *ContactConstraintManager, constraint_offsets: []const u32) bool {
            return ContactConstraintSimd.solveBatch(manager, constraint_offsets, true);
        }
    }.solve;
    var seed: u32 = 1;
    while (seed <= 20) : (seed += 1) {
        const num_constraints = 8 + seed % 9;
        const reference = try Scene.create(allocator, seed, num_constraints, .shared, .physical);
        defer reference.destroy();
        const batch = try Scene.create(allocator, seed, num_constraints, .shared, .physical);
        defer batch.destroy();
        for (0..4) |iteration| {
            const applied_reference = if (ContactConstraintSimd.enabled)
                ContactConstraintSimd.solveVelocityConstraints(&reference.manager, reference.constraintOffsets())
            else
                reference.manager.solveVelocityConstraints(reference.constraintOffsets());
            const applied_batch = solveParallel(&batch.manager, batch.constraintOffsets());
            try expectSameBits(reference, batch, seed, iteration, true);
            try expectEqual(applied_reference, applied_batch);
        }
    }
}

test "ContactConstraintSimd: the gates" {
    // The SIMD path needs the LLVM backend, is off with the determinism log and with -Dsimd_contact_solver=false
    try expectEqual(@import("builtin").zig_backend == .stage2_llvm, ContactConstraintSimd.supported);
    try expectEqual(ContactConstraintSimd.supported and Core.simd_contact_solver and !Core.determinism_log, ContactConstraintSimd.enabled);
    try expectEqual(@as(usize, 8), ContactConstraintSimd.lanes);
}
