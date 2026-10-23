//! Port of: Jolt/Physics/Constraints/ConstraintPart/ContactConstraintPart.h
//! Status: complete
//!
//! Jolt's class templates over the motion types (`template <EMotionType Type1>`, with specializations that add members)
//! are functions of comptime `MotionType`s that return extern structs (SimulationArchitecture D8). The contact
//! constraint manager selects them with `inline else` over the motion types of the two bodies.
//! - `ContactConstraintPart1(type1)` is the specialization chain `ContactConstraintPart1<Static>` (effective mass,
//!   total lambda) <- `<Kinematic>` (+ `r1_plus_u_x_axis`) <- `<Dynamic>` (+ `inv_i1_r1_plus_u_x_axis`) in one struct:
//!   a member that the motion type does not have is a `void` field, so the layout is Jolt's (comptime checked with
//!   Jolt's static_asserts).
//! - `ContactConstraintPart(type1, type2) : ContactConstraintPart1<Type1>, ContactConstraintPart2<Type2>` embeds its two
//!   base classes as `base1` and `base2` (in Jolt's order, followed by `bias`) and forwards the public functions of
//!   `ContactConstraintPart1` (`deactivate`, `isActive`, `setTotalLambda`, `getTotalLambda`).
//! - "Constructor will not be called": every member defaults to `undefined`.
//! - `Vec3 &ioLinearVelocity1` parameters are `*Vec3`, the defaulted `inBias = 0.0f` is an options struct.
//! - `JPH_ASSERT(false); // Static vs static is nonsensical!` leaves `jv` undefined, like Jolt's release build.

const std = @import("std");
const Core = @import("../../../Core/Core.zig");
const math = @import("../../../Math/Math.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Body = @import("../../Body/Body.zig").Body;
const MotionType = @import("../../Body/MotionType.zig").MotionType;

/// Decide which members this constraint part needs based on motion type
pub fn ContactConstraintPart1(comptime type1: MotionType) type {
    return extern struct {
        const Self = @This();

        // Note: Constructor will not be called
        effective_mass: f32 = undefined,
        total_lambda: f32 = undefined,

        /// ContactConstraintPart1<EMotionType::Kinematic> (also inherited by <EMotionType::Dynamic>)
        // Note: Constructor will not be called
        r1_plus_u_x_axis: if (type1 != .static) Float3 else void = undefined,

        /// ContactConstraintPart1<EMotionType::Dynamic>
        // Note: Constructor will not be called
        inv_i1_r1_plus_u_x_axis: if (type1 == .dynamic) Float3 else void = undefined,

        /// Deactivate this constraint
        pub fn deactivate(self: *Self) void {
            self.effective_mass = 0.0;
            self.total_lambda = 0.0;
        }

        /// Check if constraint is active
        pub fn isActive(self: *const Self) bool {
            return self.effective_mass != 0.0;
        }

        /// Override total lagrange multiplier, can be used to set the initial value for warm starting
        pub fn setTotalLambda(self: *Self, lambda: f32) void {
            self.total_lambda = lambda;
        }

        /// Return lagrange multiplier
        pub fn getTotalLambda(self: *const Self) f32 {
            return self.total_lambda;
        }
    };
}

/// Decide which members this constraint part needs based on motion type (empty for EMotionType::Static)
pub fn ContactConstraintPart2(comptime type2: MotionType) type {
    return extern struct {
        /// ContactConstraintPart2<EMotionType::Kinematic> (also inherited by <EMotionType::Dynamic>)
        // Note: Constructor will not be called
        r2_x_axis: if (type2 != .static) Float3 else void = undefined,

        /// ContactConstraintPart2<EMotionType::Dynamic>
        // Note: Constructor will not be called
        inv_i2_r2_x_axis: if (type2 == .dynamic) Float3 else void = undefined,
    };
}

/// This is a copy of AxisConstraintPart, specialized to handle contact constraints. See the documentation of AxisConstraintPart for more documentation behind the math.
pub fn ContactConstraintPart(comptime type1: MotionType, comptime type2: MotionType) type {
    return extern struct {
        const Self = @This();

        /// The motion types this part was specialized for (the template arguments)
        pub const motion_type1 = type1;
        pub const motion_type2 = type2;

        /// Base class ContactConstraintPart1<Type1>
        base1: ContactConstraintPart1(type1) = .{},
        /// Base class ContactConstraintPart2<Type2>
        base2: ContactConstraintPart2(type2) = .{},
        // Note: Constructor will not be called. This serves as 1 extra float so we can read the previous member using Vec3::sLoadFloat3Unsafe
        bias: f32 = undefined,

        /// See AxisConstraintPart::ApplyVelocityStep
        fn applyVelocityStep(self: *const Self, linear_velocity1: *Vec3, angular_velocity1: *Vec3, linear_velocity2: *Vec3, angular_velocity2: *Vec3, inv_mass1: f32, inv_mass2: f32, world_space_axis: Vec3, lambda: f32) bool {
            if (lambda != 0.0) {
                if (comptime type1 == .dynamic) {
                    linear_velocity1.* = linear_velocity1.sub(world_space_axis.mulScalar(lambda * inv_mass1));
                    angular_velocity1.* = angular_velocity1.sub(Vec3.loadFloat3Unsafe(&self.base1.inv_i1_r1_plus_u_x_axis).mulScalar(lambda));
                }
                if (comptime type2 == .dynamic) {
                    linear_velocity2.* = linear_velocity2.add(world_space_axis.mulScalar(lambda * inv_mass2));
                    angular_velocity2.* = angular_velocity2.add(Vec3.loadFloat3Unsafe(&self.base2.inv_i2_r2_x_axis).mulScalar(lambda));
                }
                return true;
            }

            return false;
        }

        /// See AxisConstraintPart::CalculateConstraintProperties
        pub fn calculateConstraintProperties(self: *Self, inv_mass1: f32, inv_i1: Mat44, r1_plus_u: Vec3, inv_mass2: f32, inv_i2: Mat44, r2: Vec3, world_space_axis: Vec3, opts: struct { bias: f32 = 0.0 }) void {
            if (Core.enable_asserts) std.debug.assert(world_space_axis.isNormalized(.{ .tolerance = 1.0e-5 }));

            // Store bias
            self.bias = opts.bias;

            // Calculate inverse effective mass: K = J M^-1 J^T
            var inv_effective_mass: f32 = undefined;

            if (comptime type1 != .static) {
                const r1_plus_u_x_axis = r1_plus_u.cross(world_space_axis);
                r1_plus_u_x_axis.storeFloat3(&self.base1.r1_plus_u_x_axis);

                if (comptime type1 == .dynamic) {
                    const invi1_r1_plus_u_x_axis = inv_i1.multiply3x3(r1_plus_u_x_axis);
                    invi1_r1_plus_u_x_axis.storeFloat3(&self.base1.inv_i1_r1_plus_u_x_axis);

                    inv_effective_mass = inv_mass1 + invi1_r1_plus_u_x_axis.dot(r1_plus_u_x_axis);
                } else inv_effective_mass = 0.0;
            } else inv_effective_mass = 0.0;

            if (comptime type2 != .static) {
                const r2_x_axis = r2.cross(world_space_axis);
                r2_x_axis.storeFloat3(&self.base2.r2_x_axis);

                if (comptime type2 == .dynamic) {
                    const invi2_r2_x_axis = inv_i2.multiply3x3(r2_x_axis);
                    invi2_r2_x_axis.storeFloat3(&self.base2.inv_i2_r2_x_axis);

                    inv_effective_mass += inv_mass2 + invi2_r2_x_axis.dot(r2_x_axis);
                }
            }

            if (inv_effective_mass < math.flt_min)
                self.base1.deactivate()
            else
                self.base1.effective_mass = 1.0 / inv_effective_mass;
        }

        /// See AxisConstraintPart::WarmStart
        pub fn warmStart(self: *Self, linear_velocity1: *Vec3, angular_velocity1: *Vec3, linear_velocity2: *Vec3, angular_velocity2: *Vec3, inv_mass1: f32, inv_mass2: f32, world_space_axis: Vec3, warm_start_impulse_ratio: f32) bool {
            self.base1.total_lambda *= warm_start_impulse_ratio;

            return self.applyVelocityStep(linear_velocity1, angular_velocity1, linear_velocity2, angular_velocity2, inv_mass1, inv_mass2, world_space_axis, self.base1.total_lambda);
        }

        /// Part 1 of AxisConstraint::SolveVelocityConstraint: get the total lambda
        pub fn solveVelocityConstraintGetTotalLambda(self: *const Self, linear_velocity1: Vec3, angular_velocity1: Vec3, linear_velocity2: Vec3, angular_velocity2: Vec3, world_space_axis: Vec3) f32 {
            // Calculate jacobian multiplied by linear velocity
            var jv: f32 = undefined;
            if (comptime type1 != .static and type2 != .static)
                jv = world_space_axis.dot(linear_velocity1.sub(linear_velocity2))
            else if (comptime type1 != .static)
                jv = world_space_axis.dot(linear_velocity1)
            else if (comptime type2 != .static)
                jv = world_space_axis.dot(linear_velocity2.negate())
            else if (Core.enable_asserts)
                std.debug.assert(false); // Static vs static is nonsensical!

            // Calculate jacobian multiplied by angular velocity
            if (comptime type1 != .static)
                jv += Vec3.loadFloat3Unsafe(&self.base1.r1_plus_u_x_axis).dot(angular_velocity1);
            if (comptime type2 != .static)
                jv -= Vec3.loadFloat3Unsafe(&self.base2.r2_x_axis).dot(angular_velocity2);

            // Lagrange multiplier is:
            //
            // lambda = -K^-1 (J v + b)
            const lambda = self.base1.effective_mass * (jv - self.bias);

            // Return the total accumulated lambda
            return self.base1.total_lambda + lambda;
        }

        /// Part 2 of AxisConstraint::SolveVelocityConstraint: apply new lambda
        pub fn solveVelocityConstraintApplyLambda(self: *Self, linear_velocity1: *Vec3, angular_velocity1: *Vec3, linear_velocity2: *Vec3, angular_velocity2: *Vec3, inv_mass1: f32, inv_mass2: f32, world_space_axis: Vec3, total_lambda: f32) bool {
            const delta_lambda = total_lambda - self.base1.total_lambda; // Calculate change in lambda
            self.base1.total_lambda = total_lambda; // Store accumulated impulse

            return self.applyVelocityStep(linear_velocity1, angular_velocity1, linear_velocity2, angular_velocity2, inv_mass1, inv_mass2, world_space_axis, delta_lambda);
        }

        /// See: AxisConstraintPart::SolveVelocityConstraint
        pub fn solveVelocityConstraint(self: *Self, linear_velocity1: *Vec3, angular_velocity1: *Vec3, linear_velocity2: *Vec3, angular_velocity2: *Vec3, inv_mass1: f32, inv_mass2: f32, world_space_axis: Vec3, min_lambda: f32, max_lambda: f32) bool {
            var total_lambda = self.solveVelocityConstraintGetTotalLambda(linear_velocity1.*, angular_velocity1.*, linear_velocity2.*, angular_velocity2.*, world_space_axis);

            // Clamp impulse to specified range
            total_lambda = math.clamp(total_lambda, min_lambda, max_lambda);

            return self.solveVelocityConstraintApplyLambda(linear_velocity1, angular_velocity1, linear_velocity2, angular_velocity2, inv_mass1, inv_mass2, world_space_axis, total_lambda);
        }

        /// See: AxisConstraintPart::SolvePositionConstraint
        pub fn solvePositionConstraint(self: *const Self, body1: *Body, inv_mass1: f32, body2: *Body, inv_mass2: f32, world_space_axis: Vec3, c: f32, baumgarte: f32) bool {
            if (c != 0.0) {
                const lambda = -self.base1.effective_mass * baumgarte * c;
                if (comptime type1 == .dynamic) {
                    body1.subPositionStep(world_space_axis.mulScalar(lambda * inv_mass1));
                    body1.subRotationStep(Vec3.loadFloat3Unsafe(&self.base1.inv_i1_r1_plus_u_x_axis).mulScalar(lambda));
                }
                if (comptime type2 == .dynamic) {
                    body2.addPositionStep(world_space_axis.mulScalar(lambda * inv_mass2));
                    body2.addRotationStep(Vec3.loadFloat3Unsafe(&self.base2.inv_i2_r2_x_axis).mulScalar(lambda));
                }
                return true;
            }

            return false;
        }

        // The public functions of the base class ContactConstraintPart1<Type1>

        /// Deactivate this constraint
        pub fn deactivate(self: *Self) void {
            self.base1.deactivate();
        }

        /// Check if constraint is active
        pub fn isActive(self: *const Self) bool {
            return self.base1.isActive();
        }

        /// Override total lagrange multiplier, can be used to set the initial value for warm starting
        pub fn setTotalLambda(self: *Self, lambda: f32) void {
            self.base1.setTotalLambda(lambda);
        }

        /// Return lagrange multiplier
        pub fn getTotalLambda(self: *const Self) f32 {
            return self.base1.getTotalLambda();
        }
    };
}

comptime {
    std.debug.assert(@sizeOf(ContactConstraintPart(.dynamic, .dynamic)) == 3 * @sizeOf(f32) + 4 * @sizeOf(Float3));
    std.debug.assert(@sizeOf(ContactConstraintPart(.dynamic, .kinematic)) == 3 * @sizeOf(f32) + 3 * @sizeOf(Float3));
    std.debug.assert(@sizeOf(ContactConstraintPart(.dynamic, .static)) == 3 * @sizeOf(f32) + 2 * @sizeOf(Float3));
    std.debug.assert(@sizeOf(ContactConstraintPart(.kinematic, .dynamic)) == 3 * @sizeOf(f32) + 3 * @sizeOf(Float3));
    std.debug.assert(@sizeOf(ContactConstraintPart(.kinematic, .kinematic)) == 3 * @sizeOf(f32) + 2 * @sizeOf(Float3));
    std.debug.assert(@sizeOf(ContactConstraintPart(.kinematic, .static)) == 3 * @sizeOf(f32) + @sizeOf(Float3));
    std.debug.assert(@sizeOf(ContactConstraintPart(.static, .dynamic)) == 3 * @sizeOf(f32) + 2 * @sizeOf(Float3));
    std.debug.assert(@sizeOf(ContactConstraintPart(.static, .kinematic)) == 3 * @sizeOf(f32) + @sizeOf(Float3));
    std.debug.assert(@sizeOf(ContactConstraintPart(.static, .static)) == 3 * @sizeOf(f32));
}

// Zolt: Vec3.loadFloat3Unsafe reads 4 bytes past each Float3 member. `bias` makes sure that they are part of the struct.
comptime {
    for ([_]MotionType{ .static, .kinematic, .dynamic }) |type1| {
        for ([_]MotionType{ .static, .kinematic, .dynamic }) |type2| {
            const T = ContactConstraintPart(type1, type2);
            if (type1 != .static) std.debug.assert(Vec3.canLoadFloat3Unsafe(T, "base1.r1_plus_u_x_axis"));
            if (type1 == .dynamic) std.debug.assert(Vec3.canLoadFloat3Unsafe(T, "base1.inv_i1_r1_plus_u_x_axis"));
            if (type2 != .static) std.debug.assert(Vec3.canLoadFloat3Unsafe(T, "base2.r2_x_axis"));
            if (type2 == .dynamic) std.debug.assert(Vec3.canLoadFloat3Unsafe(T, "base2.inv_i2_r2_x_axis"));
        }
    }
}

const TestContext = @import("ConstraintPartTestContext.zig");

/// The velocities and mass properties that the contact constraint manager reads from the motion properties of a body
const TestBodyState = struct {
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    inv_mass: f32,
    inv_inertia: Mat44,

    fn init(body: *const Body) TestBodyState {
        return .{
            .linear_velocity = body.getLinearVelocity(),
            .angular_velocity = body.getAngularVelocity(),
            .inv_mass = if (body.isDynamic()) body.getMotionProperties().getInverseMass() else 0.0,
            .inv_inertia = if (body.isDynamic()) body.getInverseInertia() else Mat44.zero(),
        };
    }
};

fn testContactConstraintPart(comptime type1: MotionType, comptime type2: MotionType, context: *TestContext.ConstraintPartTestContext) !void {
    const testing = std.testing;
    const Part = ContactConstraintPart(type1, type2);
    try testing.expectEqual(type1, Part.motion_type1);
    try testing.expectEqual(type2, Part.motion_type2);

    const bodies = try context.createBodyPair(.{ type1, type2 });
    defer context.destroyBodies(&bodies);
    const body1 = bodies[0];
    const body2 = bodies[1];
    var s1 = TestBodyState.init(body1);
    var s2 = TestBodyState.init(body2);
    const r1_plus_u = Vec3.init(0.3, -0.5, 0.8);
    const r2 = Vec3.init(-0.4, 0.6, 0.2);
    const axis = Vec3.init(1, 2, -0.5).normalized();

    var part: Part = .{};
    part.calculateConstraintProperties(s1.inv_mass, s1.inv_inertia, r1_plus_u, s2.inv_mass, s2.inv_inertia, r2, axis, .{});
    const any_dynamic = type1 == .dynamic or type2 == .dynamic;
    try testing.expectEqual(any_dynamic, part.isActive());
    if (!any_dynamic) {
        try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
        return;
    }

    // Same effective mass as the AxisConstraintPart that works on the bodies
    const AxisConstraintPart = @import("AxisConstraintPart.zig").AxisConstraintPart;
    var axis_part: AxisConstraintPart = .{};
    axis_part.calculateConstraintProperties(body1, r1_plus_u, body2, r2, axis, .{});
    try testing.expectApproxEqRel(axis_part.effective_mass, part.base1.effective_mass, 1.0e-5);

    // Solving without limits removes the relative velocity, the static / kinematic velocities are not touched
    part.setTotalLambda(0.0);
    const lv1_before = s1.linear_velocity;
    const lv2_before = s2.linear_velocity;
    const total_lambda = part.solveVelocityConstraintGetTotalLambda(s1.linear_velocity, s1.angular_velocity, s2.linear_velocity, s2.angular_velocity, axis);
    try testing.expect(part.solveVelocityConstraint(&s1.linear_velocity, &s1.angular_velocity, &s2.linear_velocity, &s2.angular_velocity, s1.inv_mass, s2.inv_mass, axis, -math.flt_max, math.flt_max));
    try testing.expectEqual(total_lambda, part.getTotalLambda());
    try testing.expectApproxEqAbs(part.getTotalLambda(), part.solveVelocityConstraintGetTotalLambda(s1.linear_velocity, s1.angular_velocity, s2.linear_velocity, s2.angular_velocity, axis), 1.0e-5 * @abs(total_lambda));
    if (type1 != .dynamic) try testing.expect(s1.linear_velocity.eql(lv1_before));
    if (type2 != .dynamic) try testing.expect(s2.linear_velocity.eql(lv2_before));

    // The two step version with a clamp equals the one step version
    var clamped: Part = .{};
    clamped.calculateConstraintProperties(s1.inv_mass, s1.inv_inertia, r1_plus_u, s2.inv_mass, s2.inv_inertia, r2, axis, .{ .bias = 2.0 });
    clamped.setTotalLambda(0.0);
    var v = [4]Vec3{ s1.linear_velocity, s1.angular_velocity, s2.linear_velocity, s2.angular_velocity };
    var w = v;
    _ = clamped.solveVelocityConstraint(&v[0], &v[1], &v[2], &v[3], s1.inv_mass, s2.inv_mass, axis, 0.0, 0.5);
    try testing.expect(clamped.getTotalLambda() >= 0.0 and clamped.getTotalLambda() <= 0.5);
    var clamped2: Part = .{};
    clamped2.calculateConstraintProperties(s1.inv_mass, s1.inv_inertia, r1_plus_u, s2.inv_mass, s2.inv_inertia, r2, axis, .{ .bias = 2.0 });
    clamped2.setTotalLambda(0.0);
    const new_lambda = math.clamp(clamped2.solveVelocityConstraintGetTotalLambda(w[0], w[1], w[2], w[3], axis), 0.0, 0.5);
    _ = clamped2.solveVelocityConstraintApplyLambda(&w[0], &w[1], &w[2], &w[3], s1.inv_mass, s2.inv_mass, axis, new_lambda);
    try testing.expectEqual(clamped.getTotalLambda(), clamped2.getTotalLambda());
    for (v, w) |a, b|
        try testing.expect(a.eql(b));

    // A zero range applies nothing once the accumulated impulse is at the limit
    _ = clamped.solveVelocityConstraint(&v[0], &v[1], &v[2], &v[3], s1.inv_mass, s2.inv_mass, axis, 0.5, 0.5);
    try testing.expectEqual(@as(f32, 0.5), clamped.getTotalLambda());
    w = v;
    try testing.expect(!clamped.solveVelocityConstraint(&v[0], &v[1], &v[2], &v[3], s1.inv_mass, s2.inv_mass, axis, 0.5, 0.5));
    for (v, w) |a, b|
        try testing.expect(a.eql(b));

    // Warm start
    const lambda = part.getTotalLambda();
    const av_before = if (type1 == .dynamic) s1.angular_velocity else s2.angular_velocity;
    try testing.expect(part.warmStart(&s1.linear_velocity, &s1.angular_velocity, &s2.linear_velocity, &s2.angular_velocity, s1.inv_mass, s2.inv_mass, axis, 0.5));
    try testing.expectEqual(lambda * 0.5, part.getTotalLambda());
    try testing.expect(!av_before.eql(if (type1 == .dynamic) s1.angular_velocity else s2.angular_velocity));
    part.setTotalLambda(0.0);
    try testing.expect(!part.warmStart(&s1.linear_velocity, &s1.angular_velocity, &s2.linear_velocity, &s2.angular_velocity, s1.inv_mass, s2.inv_mass, axis, 0.5));

    // Position constraint
    const before1 = TestContext.BodySnapshot.take(body1);
    const before2 = TestContext.BodySnapshot.take(body2);
    try testing.expect(!part.solvePositionConstraint(body1, s1.inv_mass, body2, s2.inv_mass, axis, 0.0, 0.2));
    try testing.expect(part.solvePositionConstraint(body1, s1.inv_mass, body2, s2.inv_mass, axis, -0.05, 0.2));
    try TestContext.checkUnmoved(body1, before1);
    try TestContext.checkUnmoved(body2, before2);
    const moved = if (type1 == .dynamic) body1 else body2;
    const moved_before = if (type1 == .dynamic) before1 else before2;
    try testing.expect(!moved.getCenterOfMassPosition().eql(moved_before.position));

    // Deactivate
    part.deactivate();
    try testing.expect(!part.isActive());
    try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
}

test "ContactConstraintPart for every combination of motion types" {
    const testing = std.testing;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    inline for (TestContext.motion_type_pairs) |pair|
        try testContactConstraintPart(pair[0], pair[1], &context);
}

test "ContactConstraintPart deactivates without effective mass" {
    const Part = ContactConstraintPart(.dynamic, .static);
    var part: Part = .{};
    part.setTotalLambda(3.0);
    // Infinite mass and zero inertia
    part.calculateConstraintProperties(0.0, Mat44.zero(), Vec3.init(1, 2, 3), 0.0, Mat44.zero(), Vec3.zero(), Vec3.axisY(), .{});
    try std.testing.expect(!part.isActive());
    try std.testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
}
