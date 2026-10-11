//! Port of: Jolt/Physics/Constraints/ConstraintPart/AngularFrictionConstraintPart.h
//! Status: complete
//!
//! Like ContactConstraintPart.zig (SimulationArchitecture D8): the class templates over the motion types are functions
//! of comptime `MotionType`s that return extern structs with Jolt's layout (comptime checked with Jolt's static_asserts).
//! - `AngularFrictionConstraintPart1<Type1> : ContactConstraintPart1<EMotionType::Static>` embeds its base as `base`
//!   and adds `inv_i1_axis` for EMotionType::Dynamic (a `void` field otherwise); it forwards the public functions of
//!   the base (`deactivate`, `isActive`, `setTotalLambda`, `getTotalLambda`).
//! - `AngularFrictionConstraintPart(type1, type2)` embeds its two base classes as `base1` and `base2`, followed by
//!   `bias`, and forwards the same public functions.
//! - `Vec3 &ioAngularVelocity1` parameters are `*Vec3`, the defaulted `inBias = 0.0f` is an options struct.
//! - `JPH_ASSERT(false); // Static vs static is nonsensical!` is `if (Core.enable_asserts) std.debug.assert(false)`:
//!   Jolt's release build continues (in CalculateConstraintProperties with an inverse effective mass of 0, which
//!   deactivates the part; in SolveVelocityConstraint with an undefined `jv`).

const std = @import("std");
const Core = @import("../../../Core/Core.zig");
const math = @import("../../../Math/Math.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Float3 = @import("../../../Math/Float3.zig").Float3;
const MotionType = @import("../../Body/MotionType.zig").MotionType;
const ContactConstraintPart1 = @import("ContactConstraintPart.zig").ContactConstraintPart1;

/// Decide which members this constraint part needs based on motion type
pub fn AngularFrictionConstraintPart1(comptime type1: MotionType) type {
    return extern struct {
        const Self = @This();

        /// Base class ContactConstraintPart1<EMotionType::Static>
        base: ContactConstraintPart1(.static) = .{},

        /// AngularFrictionConstraintPart1<EMotionType::Dynamic>
        // Note: Constructor will not be called
        inv_i1_axis: if (type1 == .dynamic) Float3 else void = undefined,

        /// Deactivate this constraint
        pub fn deactivate(self: *Self) void {
            self.base.deactivate();
        }

        /// Check if constraint is active
        pub fn isActive(self: *const Self) bool {
            return self.base.isActive();
        }

        /// Override total lagrange multiplier, can be used to set the initial value for warm starting
        pub fn setTotalLambda(self: *Self, lambda: f32) void {
            self.base.setTotalLambda(lambda);
        }

        /// Return lagrange multiplier
        pub fn getTotalLambda(self: *const Self) f32 {
            return self.base.getTotalLambda();
        }
    };
}

/// Decide which members this constraint part needs based on motion type (empty unless EMotionType::Dynamic)
pub fn AngularFrictionConstraintPart2(comptime type2: MotionType) type {
    return extern struct {
        /// AngularFrictionConstraintPart2<EMotionType::Dynamic>
        // Note: Constructor will not be called
        inv_i2_axis: if (type2 == .dynamic) Float3 else void = undefined,
    };
}

/// This is a copy of AngleConstraintPart, specialized to handle contact constraints. See the documentation of AngleConstraintPart for more documentation behind the math.
pub fn AngularFrictionConstraintPart(comptime type1: MotionType, comptime type2: MotionType) type {
    return extern struct {
        const Self = @This();

        /// The motion types this part was specialized for (the template arguments)
        pub const motion_type1 = type1;
        pub const motion_type2 = type2;

        /// Base class AngularFrictionConstraintPart1<Type1>
        base1: AngularFrictionConstraintPart1(type1) = .{},
        /// Base class AngularFrictionConstraintPart2<Type2>
        base2: AngularFrictionConstraintPart2(type2) = .{},
        // Note: Constructor will not be called. This serves as 1 extra float so we can read the previous member using Vec3::sLoadFloat3Unsafe
        bias: f32 = undefined,

        /// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
        fn applyVelocityStep(self: *const Self, angular_velocity1: *Vec3, angular_velocity2: *Vec3, lambda: f32) bool {
            // Apply impulse if delta is not zero
            if (lambda != 0.0) {
                if (comptime type1 == .dynamic)
                    angular_velocity1.* = angular_velocity1.sub(Vec3.loadFloat3Unsafe(&self.base1.inv_i1_axis).mulScalar(lambda));
                if (comptime type2 == .dynamic)
                    angular_velocity2.* = angular_velocity2.add(Vec3.loadFloat3Unsafe(&self.base2.inv_i2_axis).mulScalar(lambda));
                return true;
            }

            return false;
        }

        /// See: AngleConstraintPart::CalculateConstraintProperties
        pub fn calculateConstraintProperties(self: *Self, inv_i1: Mat44, inv_i2: Mat44, world_space_axis: Vec3, opts: struct { bias: f32 = 0.0 }) void {
            if (Core.enable_asserts) std.debug.assert(world_space_axis.isNormalized(.{ .tolerance = 1.0e-4 }));

            // Store bias
            self.bias = opts.bias;

            var invi1_axis: Vec3 = undefined;
            var invi2_axis: Vec3 = undefined;
            if (comptime type1 == .dynamic) {
                invi1_axis = inv_i1.multiply3x3(world_space_axis);
                invi1_axis.storeFloat3(&self.base1.inv_i1_axis);
            }
            if (comptime type2 == .dynamic) {
                invi2_axis = inv_i2.multiply3x3(world_space_axis);
                invi2_axis.storeFloat3(&self.base2.inv_i2_axis);
            }

            var inv_effective_mass: f32 = 0.0;
            if (comptime type1 == .dynamic and type2 == .dynamic)
                inv_effective_mass = world_space_axis.dot(invi1_axis.add(invi2_axis))
            else if (comptime type1 == .dynamic)
                inv_effective_mass = world_space_axis.dot(invi1_axis)
            else if (comptime type2 == .dynamic)
                inv_effective_mass = world_space_axis.dot(invi2_axis)
            else if (Core.enable_asserts)
                std.debug.assert(false); // Static vs static is nonsensical!

            if (inv_effective_mass < math.flt_min)
                self.base1.deactivate()
            else
                self.base1.base.effective_mass = 1.0 / inv_effective_mass;
        }

        /// See: AngleConstraintPart::WarmStart
        pub fn warmStart(self: *Self, angular_velocity1: *Vec3, angular_velocity2: *Vec3, warm_start_impulse_ratio: f32) bool {
            self.base1.base.total_lambda *= warm_start_impulse_ratio;
            return self.applyVelocityStep(angular_velocity1, angular_velocity2, self.base1.base.total_lambda);
        }

        /// See: AngleConstraintPart::SolveVelocityConstraint
        pub fn solveVelocityConstraint(self: *Self, angular_velocity1: *Vec3, angular_velocity2: *Vec3, world_space_axis: Vec3, min_lambda: f32, max_lambda: f32) bool {
            var jv: f32 = undefined;
            if (comptime type1 != .static and type2 != .static)
                jv = world_space_axis.dot(angular_velocity1.sub(angular_velocity2.*))
            else if (comptime type1 != .static)
                jv = world_space_axis.dot(angular_velocity1.*)
            else if (comptime type2 != .static)
                jv = -world_space_axis.dot(angular_velocity2.*)
            else if (Core.enable_asserts)
                std.debug.assert(false); // Static vs static is nonsensical!

            const total_lambda = &self.base1.base.total_lambda;
            var lambda = self.base1.base.effective_mass * (jv - self.bias);
            const new_lambda = math.clamp(total_lambda.* + lambda, min_lambda, max_lambda); // Clamp impulse
            lambda = new_lambda - total_lambda.*; // Lambda potentially got clamped, calculate the new impulse to apply
            total_lambda.* = new_lambda; // Store accumulated impulse

            return self.applyVelocityStep(angular_velocity1, angular_velocity2, lambda);
        }

        // The public functions of the base class ContactConstraintPart1<EMotionType::Static>

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
    std.debug.assert(@sizeOf(AngularFrictionConstraintPart(.dynamic, .dynamic)) == 3 * @sizeOf(f32) + 2 * @sizeOf(Float3));
    std.debug.assert(@sizeOf(AngularFrictionConstraintPart(.dynamic, .kinematic)) == 3 * @sizeOf(f32) + @sizeOf(Float3));
    std.debug.assert(@sizeOf(AngularFrictionConstraintPart(.dynamic, .static)) == 3 * @sizeOf(f32) + @sizeOf(Float3));
    std.debug.assert(@sizeOf(AngularFrictionConstraintPart(.kinematic, .dynamic)) == 3 * @sizeOf(f32) + @sizeOf(Float3));
    std.debug.assert(@sizeOf(AngularFrictionConstraintPart(.kinematic, .kinematic)) == 3 * @sizeOf(f32));
    std.debug.assert(@sizeOf(AngularFrictionConstraintPart(.kinematic, .static)) == 3 * @sizeOf(f32));
    std.debug.assert(@sizeOf(AngularFrictionConstraintPart(.static, .dynamic)) == 3 * @sizeOf(f32) + @sizeOf(Float3));
    std.debug.assert(@sizeOf(AngularFrictionConstraintPart(.static, .kinematic)) == 3 * @sizeOf(f32));
    std.debug.assert(@sizeOf(AngularFrictionConstraintPart(.static, .static)) == 3 * @sizeOf(f32));
}

const TestContext = @import("ConstraintPartTestContext.zig");

fn testAngularFrictionConstraintPart(comptime type1: MotionType, comptime type2: MotionType, context: *TestContext.ConstraintPartTestContext) !void {
    const testing = std.testing;
    const Part = AngularFrictionConstraintPart(type1, type2);
    try testing.expectEqual(type1, Part.motion_type1);

    // Jolt asserts when neither body is dynamic
    const any_dynamic = type1 == .dynamic or type2 == .dynamic;
    if (!any_dynamic and Core.enable_asserts)
        return;

    const bodies = try context.createBodyPair(.{ type1, type2 });
    defer context.destroyBodies(&bodies);
    const body1 = bodies[0];
    const body2 = bodies[1];
    const inv_i1 = if (body1.isDynamic()) body1.getInverseInertia() else Mat44.zero();
    const inv_i2 = if (body2.isDynamic()) body2.getInverseInertia() else Mat44.zero();
    var w1 = body1.getAngularVelocity();
    var w2 = body2.getAngularVelocity();
    const w1_before = w1;
    const w2_before = w2;
    const axis = Vec3.init(-0.5, 2, 1).normalized();

    var part: Part = .{};
    part.calculateConstraintProperties(inv_i1, inv_i2, axis, .{});
    try testing.expectEqual(any_dynamic, part.isActive());
    if (!any_dynamic) {
        try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
        return;
    }
    part.setTotalLambda(0.0);

    // Solving without limits removes the relative angular velocity around the axis
    const jv_before = axis.dot(w1.sub(w2));
    try testing.expect(@abs(jv_before) > 0.1);
    try testing.expect(part.solveVelocityConstraint(&w1, &w2, axis, -math.flt_max, math.flt_max));
    try testing.expectApproxEqAbs(@as(f32, 0.0), axis.dot(w1.sub(w2)), 1.0e-5);
    if (type1 != .dynamic) try testing.expect(w1.eql(w1_before));
    if (type2 != .dynamic) try testing.expect(w2.eql(w2_before));

    // Clamped
    var clamped: Part = .{};
    clamped.calculateConstraintProperties(inv_i1, inv_i2, axis, .{ .bias = 1.0 });
    clamped.setTotalLambda(0.0);
    var c1 = w1_before;
    var c2 = w2_before;
    _ = clamped.solveVelocityConstraint(&c1, &c2, axis, -0.01, 0.01);
    try testing.expect(@abs(clamped.getTotalLambda()) <= 0.01);
    const c1_limit = c1;
    try testing.expect(!clamped.solveVelocityConstraint(&c1, &c2, axis, clamped.getTotalLambda(), clamped.getTotalLambda()));
    try testing.expect(c1.eql(c1_limit));

    // Warm start
    const lambda = part.getTotalLambda();
    const ws_before = if (type1 == .dynamic) w1 else w2;
    try testing.expect(part.warmStart(&w1, &w2, 0.25));
    try testing.expectEqual(lambda * 0.25, part.getTotalLambda());
    try testing.expect(!ws_before.eql(if (type1 == .dynamic) w1 else w2));

    part.deactivate();
    try testing.expect(!part.isActive());
    try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
    try testing.expect(!part.warmStart(&w1, &w2, 1.0));
}

test "AngularFrictionConstraintPart for every combination of motion types" {
    const testing = std.testing;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    inline for (TestContext.motion_type_pairs) |pair|
        try testAngularFrictionConstraintPart(pair[0], pair[1], &context);
}

test "AngularFrictionConstraintPart deactivates without inertia" {
    var part: AngularFrictionConstraintPart(.dynamic, .dynamic) = .{};
    part.setTotalLambda(2.0);
    part.calculateConstraintProperties(Mat44.zero(), Mat44.zero(), Vec3.axisZ(), .{});
    try std.testing.expect(!part.isActive());
    try std.testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
}
