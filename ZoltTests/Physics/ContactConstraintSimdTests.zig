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
//!
//! The plan of a batch (Phase 13, ContactConstraintSimd.planBatch / solvePlanned, "The plan of a batch" in its header):
//! - Random batches of 4 to 64 contacts of a parallel split (every size that PhysicsSystem plans: full groups, tails
//!   under 4 and padded tails, contacts with static and kinematic bodies, 0 to 4 contact points; physical, edge, signed
//!   zero and huge values) are solved twice: in Jolt's order with ContactConstraintManager's scalar solver, and like
//!   PhysicsSystem solves a planned batch (warm start, planBatch, solvePlanned in every velocity iteration,
//!   storeAppliedImpulses and the position iterations in the planned order). The scenes have body transforms and
//!   inertias and a write cache with a manifold per constraint. After every phase every velocity, constraint byte, body
//!   position and rotation and cached manifold byte must be equal. The plan itself is checked (the order, the marker,
//!   the count) and the words around the batch must stay untouched.
//! - plannedCount finds a plan only behind the marker; solvePlanned with a word without a plan (split indices, a count
//!   above the batch, other high bits) on a batch that starts with a kinematic contact gives the scalar bits.
//! - PhysicsSystemTests ("the SIMD plans ...") runs a pyramid and a pile on a kinematic platform (with constraints,
//!   step overrides and 7 workers) with and without plans and checks that the planned path ran: the test-only counters
//!   PhysicsSystem.simd_plans count the velocity iterations of the planned batches by their plan word, like
//!   solvePlanned decides, so an iteration without its plan shows also without asserts.
//! - Bite proofs (each one fails these tests): planBatch writing count + 1 (the plan check fails; the pyramid tests
//!   crash: a contact with the static floor goes into a SIMD lane); plannedCount without the marker check (a split index
//!   taken as a plan: the plannedCount and plan tests fail, and without its plannedCount check the fallback test hits
//!   the MotionProperties assert of a kinematic body that a SIMD lane writes); planBatch placing the others one slot
//!   further (across the batch end: the index check panics, without safety checks the guard words change); `plans`
//!   without the `parallel` condition (the gates test and the pyramid and pile comparisons fail: the non parallel split
//!   shares bodies); a warm start that does not plan (the assert in PhysicsSystem fires with asserts on, the counters
//!   fail in every build mode, ReleaseFast too).

const std = @import("std");
const zolt = @import("zolt");

const AllowedDOFs = zolt.AllowedDOFs;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const ContactConstraintManager = zolt.ContactConstraintManager;
const ContactConstraintSimd = zolt.ContactConstraintSimd;
const Core = zolt.Core;
const DummyCalculateSolverSteps = zolt.DummyCalculateSolverSteps;
const Float3 = zolt.Float3;
const LargeIslandSplitter = zolt.LargeIslandSplitter;
const Mat44 = zolt.Mat44;
const MotionProperties = zolt.MotionProperties;
const MotionType = zolt.MotionType;
const Mt19937 = zolt.Mt19937;
const PhysicsSettings = zolt.PhysicsSettings;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDPair = zolt.SubShapeIDPair;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const ManifoldCache = ContactConstraintManager.ManifoldCache;
const CachedManifold = ContactConstraintManager.CachedManifold;

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

    /// A rotation (a unit quaternion)
    fn rotation(self: *Random) Quat {
        return Quat.rotation(self.unitVec3(), self.float(0.0, 6.0));
    }

    /// A Float3 with components in [-scale, scale)
    fn float3(self: *Random, scale: f32) Float3 {
        return .{ .x = self.float(-scale, scale), .y = self.float(-scale, scale), .z = self.float(-scale, scale) };
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
    /// What the position solve reads (Jolt's defaults)
    settings: PhysicsSettings,
    /// The write cache of the manager: a manifold per constraint, which storeAppliedImpulses writes and the position
    /// solve reads
    cache: ManifoldCache,

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

        // What the warm start, storeAppliedImpulses and the position solve read besides the velocities and the
        // constraints, from a random stream of its own (the velocities and constraints above stay those of the seed)
        var extra: Random = .{ .rng = Mt19937.init(seed ^ 0x5eed_c0de) };
        for (self.bodies, self.motion_properties) |*body, *mp| {
            body.body_type = .rigid_body;
            body.position = RVec3.init(extra.float(-2.0, 2.0), extra.float(-2.0, 2.0), extra.float(-2.0, 2.0));
            body.rotation = extra.rotation();
            mp.inv_inertia_diagonal = Vec3.init(
                if (extra.chance(15)) 0.0 else extra.float(0.0, 3.0),
                if (extra.chance(15)) 0.0 else extra.float(0.0, 3.0),
                if (extra.chance(15)) 0.0 else extra.float(0.0, 3.0),
            );
            mp.inertia_rotation = extra.rotation();
        }
        self.settings = .{};
        self.cache = .{};
        const manifold_size: u32 = @sizeOf(ContactConstraintManager.MKeyValue) + @as(u32, @intCast(CachedManifold.getRequiredExtraSize(ContactConstraintManager.max_contact_points)));
        const max_constraints = @max(num_constraints, 4); // Like ContactConstraintManager.init: the map needs 4 buckets
        try self.cache.init(allocator, 4, max_constraints, max_constraints * manifold_size + ManifoldCache.allocator_block_size);
        errdefer self.cache.deinit(allocator);
        var contact_allocator = self.cache.getContactAllocator();
        for (self.offsets, 0..) |constraint_offset, i| {
            const base: *ContactConstraintManager.ContactConstraintBase = @ptrCast(@alignCast(self.buffer.ptr + constraint_offset));
            var sub_shape_id: SubShapeID = .empty;
            sub_shape_id.setValue(@intCast(i));
            const key: SubShapeIDPair = .init(BodyID.init(0), sub_shape_id, BodyID.init(1), .empty);
            const kv = self.cache.createManifold(&contact_allocator, key, key.getHash(), @intCast(base.num_contact_points)) orelse return error.OutOfMemory;
            base.cached_manifold_handle = self.cache.toHandle(kv);
            const manifold = kv.getValue();
            manifold.contact_normal = extra.float3(1.0);
            manifold.friction_lambda.f32s = .{ extra.value(5.0), extra.value(5.0) };
            manifold.angular_friction_lambda = extra.value(5.0);
            for (manifold.getContactPoints()[0..base.num_contact_points]) |*point| {
                point.position1 = extra.float3(1.0);
                point.position2 = extra.float3(1.0);
                point.non_penetration_lambda = extra.value(5.0);
            }
        }

        self.manager = .empty;
        self.manager.constraints = .{ .items = self.buffer };
        self.manager.physics_settings = &self.settings;
        self.manager.write_cache = &self.cache;
        return self;
    }

    fn destroy(self: *Scene) void {
        self.cache.deinit(self.allocator);
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
    try expect(!ContactConstraintSimd.plans(true, 16)); // No batch is planned (permuted) either
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

    // The plans (Phase 13): only the batches of a parallel split that can fill a group, only with the SIMD solver.
    // Without it nothing is ever permuted.
    for (0..ContactConstraintSimd.max_chunk + 2) |n| {
        try expect(!ContactConstraintSimd.plans(false, n));
        try expectEqual(ContactConstraintSimd.enabled and n >= ContactConstraintSimd.min_lanes and n <= ContactConstraintSimd.max_chunk, ContactConstraintSimd.plans(true, n));
    }
    try expectEqual(@as(usize, 64), ContactConstraintSimd.max_chunk);
}

// ---------------------------------------------------------------------------------------------------------------------
// The plan of a batch (Phase 13): planBatch at the warm start, solvePlanned in the velocity iterations

/// Fail unless the two copies of a scene have the same bits in everything that the solve phases write: the velocities
/// and the constraints (expectSameBits), the positions and rotations of the bodies and the manifolds of the write cache
fn expectSameState(a: *const Scene, b: *const Scene, seed: u32, phase: []const u8, iteration: usize) !void {
    expectSameBits(a, b, seed, iteration, true) catch |err| {
        std.debug.print("  (after {s})\n", .{phase});
        return err;
    };
    for (a.bodies, b.bodies, 0..) |*body_a, *body_b, i| {
        if (!std.mem.eql(u8, std.mem.asBytes(&body_a.position), std.mem.asBytes(&body_b.position)) or
            !std.mem.eql(u8, std.mem.asBytes(&body_a.rotation), std.mem.asBytes(&body_b.rotation)))
        {
            std.debug.print("seed {d} {s} {d}: position or rotation of body {d} differs\n", .{ seed, phase, iteration, i });
            return error.TestExpectedEqual;
        }
    }
    for (a.offsets, 0..) |offset, i| {
        const base_a = a.manager.getConstraintBase(offset);
        const base_b = b.manager.getConstraintBase(offset);
        const size: usize = @intCast(CachedManifold.getRequiredTotalSize(@intCast(base_a.num_contact_points)));
        const manifold_a: [*]const u8 = @ptrCast(a.cache.fromHandle(base_a.cached_manifold_handle).getValueConst());
        const manifold_b: [*]const u8 = @ptrCast(b.cache.fromHandle(base_b.cached_manifold_handle).getValueConst());
        if (!std.mem.eql(u8, manifold_a[0..size], manifold_b[0..size])) {
            std.debug.print("seed {d} {s} {d}: cached manifold of constraint {d} differs\n", .{ seed, phase, iteration, i });
            return error.TestExpectedEqual;
        }
    }
}

/// The number of contact points of the constraint at `offset` when it goes into a SIMD lane (dynamic vs dynamic, 1
/// to 4 points), null otherwise
fn laneContactPoints(scene: *const Scene, offset: u32) ?u32 {
    const base = scene.manager.getConstraintBase(offset);
    if (!base.body1.isDynamic() or !base.body2.isDynamic())
        return null;
    if (base.num_contact_points < 1 or base.num_contact_points > ContactConstraintManager.max_contact_points)
        return null;
    return base.num_contact_points;
}

/// Fail unless planBatch planned `original` into `batch` and `plan_word`: [the lane constraints, most contact points
/// first, then in their original order][the others in their original order], the marker and the number of lane
/// constraints. Returns that number.
fn expectPlan(scene: *const Scene, original: []const u32, batch: []const u32, plan_word: u32) !u32 {
    var expected: [ContactConstraintSimd.max_chunk]u32 = undefined;
    var num: u32 = 0;
    var points: u32 = ContactConstraintManager.max_contact_points;
    while (points >= 1) : (points -= 1) {
        for (original) |offset| {
            if (laneContactPoints(scene, offset) == points) {
                expected[num] = offset;
                num += 1;
            }
        }
    }
    const num_lanes = num;
    for (original) |offset| {
        if (laneContactPoints(scene, offset) == null) {
            expected[num] = offset;
            num += 1;
        }
    }
    try std.testing.expectEqualSlices(u32, expected[0..num], batch);
    try expectEqual(ContactConstraintSimd.plan_marker | num_lanes, plan_word);
    try expectEqual(@as(?u32, num_lanes), ContactConstraintSimd.plannedCount(plan_word, batch.len));
    return num_lanes;
}

/// The phases of the solve of a batch of a parallel split that runPlannedBatch compares
const Phases = struct {
    /// The warm start (before planBatch and in Jolt's order, like PhysicsSystem)
    warm_start: bool = true,
    velocity_iterations: usize,
    /// storeAppliedImpulses after the velocity iterations
    store: bool = true,
    /// The position iterations (they need normalized normals: physical scenes)
    position_iterations: usize = 0,
    /// (Phase 14) Solve the velocity iterations from LaneGroups (buildLanes, solvePlannedLanes) and write their lambdas
    /// back after every iteration (writeBackLanes), so that every iteration can be compared
    lanes: bool = false,
};

/// The kinds of plans that the random batches made
const PlanCoverage = struct {
    batches: u32 = 0,
    /// The plan has a full group of 8
    full_groups: u32 = 0,
    /// The lane constraints end with 1 to 3 that the scalar solver solves
    tails_under_min_lanes: u32 = 0,
    /// The lane constraints end with 4 to 7 in a padded group
    tails_padded: u32 = 0,
    /// Contacts with a static or kinematic body, or without 1 to 4 points, after the lane constraints
    others: u32 = 0,

    fn add(self: *PlanCoverage, num_lanes: u32, num_contacts: usize) void {
        self.batches += 1;
        if (num_lanes >= ContactConstraintSimd.lanes) self.full_groups += 1;
        const tail = num_lanes % ContactConstraintSimd.lanes;
        if (tail > 0 and tail < ContactConstraintSimd.min_lanes) self.tails_under_min_lanes += 1;
        if (tail >= ContactConstraintSimd.min_lanes) self.tails_padded += 1;
        if (num_lanes < num_contacts) self.others += 1;
    }

    fn expectAll(self: *const PlanCoverage) !void {
        try expect(self.full_groups > 0 and self.tails_under_min_lanes > 0 and self.tails_padded > 0 and self.others > 0);
    }
};

/// Solve the batch of `jolt` in Jolt's order with the scalar solver (ContactConstraintManager) and the same batch of
/// `planned` like PhysicsSystem solves a planned batch of a parallel split: the warm start, planBatch, solvePlanned in
/// every velocity iteration, storeAppliedImpulses and the position iterations in the planned order. Compare the bits
/// after every phase, the plan, and that planBatch writes nothing outside the batch.
fn runPlannedBatch(jolt: *Scene, planned: *Scene, seed: u32, phases: Phases, coverage: *PlanCoverage) !void {
    const original = jolt.constraintOffsets();
    std.debug.assert(original.len >= ContactConstraintSimd.min_lanes and original.len <= ContactConstraintSimd.max_chunk); // A size that PhysicsSystem plans

    // The batch of the planned scene, a slice of a larger buffer like a batch of the splitter
    const guard = 4;
    const sentinel: u32 = 0xdead_beef;
    var buffer: [ContactConstraintSimd.max_chunk + 2 * guard]u32 = @splat(sentinel);
    const batch = buffer[guard..][0..original.len];
    @memcpy(batch, original);
    var dummy: DummyCalculateSolverSteps = .{};

    if (phases.warm_start) {
        const warm_start_impulse_ratio = 0.5 + @as(f32, @floatFromInt(seed % 5)) * 0.25;
        jolt.manager.warmStartVelocityConstraints(original, warm_start_impulse_ratio, &dummy);
        planned.manager.warmStartVelocityConstraints(batch, warm_start_impulse_ratio, &dummy);
        try expectSameState(jolt, planned, seed, "warm start", 0);
    }

    // The word holds a split index before, like the entry of contact_and_constraints_split_idx that holds the plan
    var plan_word: u32 = seed % LargeIslandSplitter.num_splits;
    try expectEqual(@as(?u32, null), ContactConstraintSimd.plannedCount(plan_word, batch.len));
    ContactConstraintSimd.planBatch(&planned.manager, batch, &plan_word);
    for (buffer[0..guard]) |word| try expectEqual(sentinel, word);
    for (buffer[guard + batch.len ..]) |word| try expectEqual(sentinel, word);
    coverage.add(try expectPlan(planned, original, batch, plan_word), batch.len);

    // (Phase 14) The LaneGroups of the batch in the manager's block, between groups of other batches (`before` and
    // `after` of them, filled with a pattern that must stay), named by a lane word like PhysicsSystem's claim
    const num_dynamic = ContactConstraintSimd.plannedCount(plan_word, batch.len).?;
    const num_groups = ContactConstraintSimd.numLaneGroups(num_dynamic);
    const before: u32 = seed % 3;
    const after = 1;
    const simd_lanes = &planned.manager.simd_lanes;
    try simd_lanes.replace(std.testing.allocator, before + num_groups + after);
    defer simd_lanes.free(std.testing.allocator);
    const all_groups = simd_lanes.groups();
    try expectEqual(@as(usize, before + num_groups + after), all_groups.len);
    @memset(std.mem.sliceAsBytes(all_groups), 0x5a);
    var lane_word = ContactConstraintSimd.no_lanes;
    if (phases.lanes and num_groups > 0) { // (PhysicsSystem builds no LaneGroups for a batch that fills no group)
        lane_word = ContactConstraintSimd.laneWord(before);
        const batch_lanes = ContactConstraintSimd.laneGroups(lane_word, plan_word, batch.len, all_groups).?;
        try expectEqual(num_dynamic, batch_lanes.num_dynamic);
        try expectEqual(@as([*]ContactConstraintSimd.LaneGroup, all_groups[before..].ptr), batch_lanes.groups.ptr);
        try expectEqual(@as(usize, num_groups), batch_lanes.groups.len);
        ContactConstraintSimd.buildLanes(&planned.manager, batch, num_dynamic, batch_lanes.groups);
        try expectSameState(jolt, planned, seed, "buildLanes", 0);
    }
    const lanes = ContactConstraintSimd.hasLanes(lane_word);
    try expectEqual(phases.lanes and num_groups > 0, lanes);

    for (0..phases.velocity_iterations) |iteration| {
        const applied_jolt = jolt.manager.solveVelocityConstraints(original);
        const applied_planned = if (lanes) ContactConstraintSimd.solvePlannedLanes(&planned.manager, batch, plan_word, lane_word) else ContactConstraintSimd.solvePlanned(&planned.manager, batch, plan_word);
        if (lanes) {
            ContactConstraintSimd.writeBackLanes(&planned.manager, batch, plan_word, lane_word);
            for (buffer[0..guard]) |word| try expectEqual(sentinel, word);
            for (buffer[guard + batch.len ..]) |word| try expectEqual(sentinel, word);
        }
        try expectSameState(jolt, planned, seed, "velocity iteration", iteration);
        try expectEqual(applied_jolt, applied_planned);
    }

    // The groups of the other batches are untouched
    for (std.mem.sliceAsBytes(all_groups[0..before])) |byte| try expectEqual(@as(u8, 0x5a), byte);
    for (std.mem.sliceAsBytes(all_groups[before + num_groups ..])) |byte| try expectEqual(@as(u8, 0x5a), byte);

    if (phases.store) {
        jolt.manager.storeAppliedImpulses(original);
        planned.manager.storeAppliedImpulses(batch);
        try expectSameState(jolt, planned, seed, "storeAppliedImpulses", 0);
    }

    for (0..phases.position_iterations) |iteration| {
        const applied_jolt = jolt.manager.solvePositionConstraints(original);
        const applied_planned = planned.manager.solvePositionConstraints(batch);
        try expectSameState(jolt, planned, seed, "position iteration", iteration);
        try expectEqual(applied_jolt, applied_planned);
    }
}

/// runPlannedBatch for batches of 4 to 64 contacts (every size that PhysicsSystem plans) built from `seeds`
fn runPlannedBatches(values: Values, num_seeds: u32, phases: Phases) !void {
    const allocator = std.testing.allocator;
    var coverage: PlanCoverage = .{};
    var seed: u32 = 1;
    while (seed <= num_seeds) : (seed += 1) {
        const num_constraints = ContactConstraintSimd.min_lanes + seed % (ContactConstraintSimd.max_chunk - ContactConstraintSimd.min_lanes + 1);
        const jolt = try Scene.create(allocator, seed, num_constraints, .parallel, values);
        defer jolt.destroy();
        const planned = try Scene.create(allocator, seed, num_constraints, .parallel, values);
        defer planned.destroy();
        try expectSameState(jolt, planned, seed, "creation", 0);
        try runPlannedBatch(jolt, planned, seed, phases, &coverage);
    }
    try coverage.expectAll();
}

test "ContactConstraintSimd: a planned batch gives the bits of Jolt's order with the scalar solver (warm start, velocity and position iterations, stored impulses)" {
    if (!ContactConstraintSimd.supported) return error.SkipZigTest;
    try runPlannedBatches(.physical, 300, .{ .velocity_iterations = 6, .position_iterations = 3 });
}

test "ContactConstraintSimd: a planned batch gives the bits of Jolt's order with the scalar solver (signed zeros, tiny and huge values)" {
    if (!ContactConstraintSimd.supported) return error.SkipZigTest;

    // The edge values have normals that are not normalized: no position iterations (calculateConstraintProperties
    // asserts a normalized axis). The huge values are solved for one velocity iteration without a warm start (see
    // Values.huge).
    try runPlannedBatches(.edge, 200, .{ .velocity_iterations = 4 });
    try runPlannedBatches(.zeros, 200, .{ .velocity_iterations = 4 });
    try runPlannedBatches(.huge, 200, .{ .warm_start = false, .velocity_iterations = 1 });
}

test "ContactConstraintSimd: a planned batch with LaneGroups gives the bits of Jolt's order with the scalar solver" {
    // Phase 14: buildLanes after the warm start and planBatch (into groups between the groups of other batches, named by
    // a lane word), solvePlannedLanes in every velocity iteration and writeBackLanes after it: the bits of Jolt's order
    // after every phase (also the lambdas: the write back covers every lane and point that the SIMD solver changed, and
    // only those), nothing written outside the batch's groups
    if (!ContactConstraintSimd.supported) return error.SkipZigTest;
    try runPlannedBatches(.physical, 300, .{ .velocity_iterations = 6, .position_iterations = 3, .lanes = true });
    try runPlannedBatches(.edge, 200, .{ .velocity_iterations = 4, .lanes = true });
    try runPlannedBatches(.zeros, 200, .{ .velocity_iterations = 4, .lanes = true });
    try runPlannedBatches(.huge, 200, .{ .warm_start = false, .velocity_iterations = 1, .lanes = true });
}

test "ContactConstraintSimd: a lane word names LaneGroups only behind its marker and only when they fit" {
    // Phase 14: the word after a batch's plan word holds its lane word. splitIsland writes split indices there and a plan
    // word is never there in a correct run; neither is taken for a lane word.
    const laneGroups = ContactConstraintSimd.laneGroups;
    const LaneGroup = ContactConstraintSimd.LaneGroup;
    const allocator = std.testing.allocator;
    const groups = try allocator.alignedAlloc(LaneGroup, .fromByteUnits(64), 10);
    defer allocator.free(groups);
    const plan = ContactConstraintSimd.plan_marker | 12; // 12 planned contacts: 2 groups

    // No marker: the batch gathers (also the split indices and no_lanes)
    for (0..LargeIslandSplitter.num_splits) |split|
        try expectEqual(@as(?ContactConstraintSimd.BatchLanes, null), laneGroups(@intCast(split), plan, 16, groups));
    try expectEqual(@as(?ContactConstraintSimd.BatchLanes, null), laneGroups(ContactConstraintSimd.no_lanes, plan, 16, groups));
    try expectEqual(@as(?ContactConstraintSimd.BatchLanes, null), laneGroups(plan, plan, 16, groups));
    try expectEqual(@as(?ContactConstraintSimd.BatchLanes, null), laneGroups(0x4d00_0003, plan, 16, groups));

    // The marker: the groups from the word's index on
    for ([_]u32{ 0, 3, 8 }) |first| {
        const lanes = laneGroups(ContactConstraintSimd.laneWord(first), plan, 16, groups).?;
        try expectEqual(@as(u32, 12), lanes.num_dynamic);
        try expectEqual(@as([*]LaneGroup, groups[first..].ptr), lanes.groups.ptr);
        try expectEqual(@as(usize, 2), lanes.groups.len);
    }
    try expectEqual(ContactConstraintSimd.lane_marker | 5, ContactConstraintSimd.laneWord(5));
    try expectEqual(ContactConstraintSimd.lane_marker | (ContactConstraintSimd.max_lane_groups - 1), ContactConstraintSimd.laneWord(ContactConstraintSimd.max_lane_groups - 1));

    // Not reachable in a correct run (they assert with asserts on): the groups do not fit, or no plan in the plan word.
    // Without asserts the batch gathers.
    if (!std.debug.runtime_safety) {
        try expectEqual(@as(?ContactConstraintSimd.BatchLanes, null), laneGroups(ContactConstraintSimd.laneWord(9), plan, 16, groups));
        try expectEqual(@as(?ContactConstraintSimd.BatchLanes, null), laneGroups(ContactConstraintSimd.laneWord(11), plan, 16, groups));
        try expectEqual(@as(?ContactConstraintSimd.BatchLanes, null), laneGroups(ContactConstraintSimd.laneWord(0), 7, 16, groups));
        try expectEqual(@as(?ContactConstraintSimd.BatchLanes, null), laneGroups(ContactConstraintSimd.laneWord(0), ContactConstraintSimd.plan_marker | 3, 16, groups));
    }
}

test "ContactConstraintSimd: laneGroupBound is an upper bound on the LaneGroups of the batches of an island" {
    // Phase 14: the bound that PhysicsSystem sizes its LaneGroups by. Batches of a parallel split: chunks of 16 items
    // (contacts and constraints), planned with 4 to 16 contacts, numLaneGroups of their planned contacts. Random
    // islands: the split sizes, the contacts per split and the planned share of the contacts vary.
    const numLaneGroups = ContactConstraintSimd.numLaneGroups;
    var prng = std.Random.DefaultPrng.init(0x7a17);
    const random = prng.random();
    for (0..2000) |_| {
        var num_contacts: u32 = 0;
        var num_items: u32 = 0;
        var num_groups: u64 = 0;
        const num_parallel = random.intRangeAtMost(u32, 0, LargeIslandSplitter.non_parallel_split_idx);
        for (0..num_parallel) |_| {
            const split_contacts = random.intRangeAtMost(u32, 0, 200);
            const split_constraints = if (random.boolean()) random.intRangeAtMost(u32, 0, 40) else 0;
            num_contacts += split_contacts;
            num_items += split_contacts + split_constraints;

            // A batch takes the constraints first, then the contacts (fetchNextBatch)
            var item: u32 = 0;
            while (item < split_contacts + split_constraints) : (item += 16) {
                const end = @min(item + 16, split_contacts + split_constraints);
                const contacts = end -| @max(item, split_constraints);
                if (contacts < ContactConstraintSimd.min_lanes)
                    continue; // Not planned
                const planned = if (random.boolean()) contacts else random.intRangeAtMost(u32, 0, contacts);
                num_groups += numLaneGroups(planned);
            }
        }

        // The non parallel split: contacts and constraints that are never planned
        const non_parallel_contacts = random.intRangeAtMost(u32, 0, 100);
        num_contacts += non_parallel_contacts;
        num_items += non_parallel_contacts + random.intRangeAtMost(u32, 0, 20);

        try expect(num_groups <= ContactConstraintSimd.laneGroupBound(num_contacts, LargeIslandSplitter.maxParallelBatches(num_items)));
    }

    // Reached: 16 contacts in 4 splits of 4 (4 padded groups), 4 contacts, 31 splits of 4 contacts; fewer than 4
    // contacts plan nothing
    try expectEqual(@as(u64, 4), ContactConstraintSimd.laneGroupBound(16, LargeIslandSplitter.maxParallelBatches(16)));
    try expectEqual(@as(u64, 1), ContactConstraintSimd.laneGroupBound(4, LargeIslandSplitter.maxParallelBatches(4)));
    try expectEqual(@as(u64, 31), ContactConstraintSimd.laneGroupBound(31 * 4, LargeIslandSplitter.maxParallelBatches(31 * 4)));
    try expectEqual(@as(u64, 0), ContactConstraintSimd.laneGroupBound(3, LargeIslandSplitter.maxParallelBatches(200)));
}

test "ContactConstraintSimd: plannedCount finds a plan only behind the marker" {
    const plannedCount = ContactConstraintSimd.plannedCount;
    const marker = ContactConstraintSimd.plan_marker;

    // splitIsland writes split indices into the words: none of them is a plan
    for (0..LargeIslandSplitter.num_splits) |split|
        try expectEqual(@as(?u32, null), plannedCount(@intCast(split), 16));
    try expectEqual(@as(?u32, 0), plannedCount(marker, 16));
    try expectEqual(@as(?u32, 9), plannedCount(marker | 9, 16));
    try expectEqual(@as(?u32, 16), plannedCount(marker | 16, 16));
    try expectEqual(@as(?u32, null), plannedCount(marker | 17, 16)); // More planned contacts than the batch has
    try expectEqual(@as(?u32, null), plannedCount(0x4000_0000 | 8, 16)); // Other bits in the marker half
    try expectEqual(@as(?u32, null), plannedCount(0xc000_0000 | 8, 16));
    try expectEqual(@as(?u32, null), plannedCount(0x8001_0008, 16));
    try expectEqual(@as(?u32, null), plannedCount(0x0001_0008, 16));
}

test "ContactConstraintSimd: solvePlanned without a plan in its word solves like solveVelocityConstraints" {
    if (!ContactConstraintSimd.supported) return error.SkipZigTest;
    const allocator = std.testing.allocator;

    // Batches in an order that planBatch never makes: a kinematic vs dynamic contact first, then the lane constraints,
    // then the others. A word without a plan must give the bits of the scalar solver; taken as a plan of 4 or 8 lane
    // constraints, the kinematic contact would go into a SIMD lane (another layout) and give other bits.
    const words = [_]u32{ 0, 4, 8, LargeIslandSplitter.non_parallel_split_idx, ContactConstraintSimd.plan_marker | 65, 0x4000_0008, 0xffff_0008, 0x0001_0008 };
    var num_batches: u32 = 0;
    var seed: u32 = 1;
    while (seed <= 60) : (seed += 1) {
        const num_constraints = 8 + seed % 24;
        for (words) |word| {
            const jolt = try Scene.create(allocator, seed, num_constraints, .parallel, .physical);
            defer jolt.destroy();
            const fallback = try Scene.create(allocator, seed, num_constraints, .parallel, .physical);
            defer fallback.destroy();

            // The order: the first kinematic vs dynamic contact, the lane constraints, the others
            var batch: [ContactConstraintSimd.max_chunk]u32 = undefined;
            var num: usize = 0;
            for (jolt.offsets) |offset| {
                const base = jolt.manager.getConstraintBase(offset);
                if (base.body1.isKinematic() != base.body2.isKinematic()) {
                    batch[0] = offset;
                    num = 1;
                    break;
                }
            }
            if (num == 0)
                continue; // No kinematic contact in this scene
            for (jolt.offsets) |offset| {
                if (offset != batch[0] and laneContactPoints(jolt, offset) != null) {
                    batch[num] = offset;
                    num += 1;
                }
            }
            if (num < 1 + ContactConstraintSimd.lanes)
                continue; // Too few lane constraints behind it for a group
            for (jolt.offsets) |offset| {
                if (offset != batch[0] and laneContactPoints(jolt, offset) == null) {
                    batch[num] = offset;
                    num += 1;
                }
            }
            try expectEqual(jolt.offsets.len, num);
            try expectEqual(@as(?u32, null), ContactConstraintSimd.plannedCount(word, num));
            num_batches += 1;

            for (0..4) |iteration| {
                const applied_jolt = jolt.manager.solveVelocityConstraints(batch[0..num]);
                const applied_fallback = ContactConstraintSimd.solvePlanned(&fallback.manager, batch[0..num], word);
                try expectSameState(jolt, fallback, seed, "velocity iteration without a plan", iteration);
                try expectEqual(applied_jolt, applied_fallback);
            }
        }
    }
    try expect(num_batches >= 100);
}
