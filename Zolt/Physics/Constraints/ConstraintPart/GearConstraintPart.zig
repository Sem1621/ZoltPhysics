//! Port of: Jolt/Physics/Constraints/ConstraintPart/GearConstraintPart.h
//! Status: complete
//!
//! Like Jolt, CalculateConstraintProperties and the velocity steps use the motion properties of both bodies without
//! checking the motion type: both bodies must be dynamic (with asserts on, the dynamic body asserts of MotionProperties
//! fire otherwise, like Jolt's JPH_ASSERTs).

const std = @import("std");
const Core = @import("../../../Core/Core.zig");
const math = @import("../../../Math/Math.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Body = @import("../../Body/Body.zig").Body;
const StateRecorder = @import("../../StateRecorder.zig").StateRecorder;

/// Constraint that constrains two rotations using a gear (rotating in opposite direction)
///
/// Constraint equation:
///
/// C = Rotation1(t) + r Rotation2(t)
///
/// Derivative:
///
/// d/dt C = 0
/// <=> w1 . a + r w2 . b = 0
///
/// Jacobian:
///
/// \f[J = \begin{bmatrix}0 & a^T & 0 & r b^T\end{bmatrix}\f]
///
/// Used terms (here and below, everything in world space):\n
/// a = axis around which body 1 rotates (normalized).\n
/// b = axis along which body 2 slides (normalized).\n
/// Rotation1(t) = rotation around a of body 1.\n
/// Rotation2(t) = rotation around b of body 2.\n
/// r = ratio between rotation for body 1 and 2.\n
/// v = [v1, w1, v2, w2].\n
/// v1, v2 = linear velocity of body 1 and 2.\n
/// w1, w2 = angular velocity of body 1 and 2.\n
/// M = mass matrix, a diagonal matrix of the mass and inertia with diagonal [m1, I1, m2, I2].\n
/// \f$K^{-1} = \left( J M^{-1} J^T \right)^{-1}\f$ = effective mass.\n
/// \f$\beta\f$ = baumgarte constant.
pub const GearConstraintPart = struct {
    inv_i1_a: Vec3 = undefined,
    inv_i2_b: Vec3 = undefined,
    effective_mass: f32 = 0.0,
    total_lambda: f32 = 0.0,

    /// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
    fn applyVelocityStep(self: *const GearConstraintPart, body1: *Body, body2: *Body, lambda: f32) bool {
        // Apply impulse if delta is not zero
        if (lambda != 0.0) {
            // Calculate velocity change due to constraint
            //
            // Impulse:
            // P = J^T lambda
            //
            // Euler velocity integration:
            // v' = v + M^-1 P
            body1.getMotionPropertiesMut().addAngularVelocityStep(self.inv_i1_a.mulScalar(lambda));
            body2.getMotionPropertiesMut().addAngularVelocityStep(self.inv_i2_b.mulScalar(lambda));
            return true;
        }

        return false;
    }

    /// Calculate properties used during the functions below
    /// @param inBody1 The first body that this constraint is attached to
    /// @param inBody2 The second body that this constraint is attached to
    /// @param inWorldSpaceHingeAxis1 The axis around which body 1 rotates
    /// @param inWorldSpaceHingeAxis2 The axis around which body 2 rotates
    /// @param inRatio The ratio between rotation and translation
    pub fn calculateConstraintProperties(self: *GearConstraintPart, body1: *const Body, world_space_hinge_axis1: Vec3, body2: *const Body, world_space_hinge_axis2: Vec3, ratio: f32) void {
        if (Core.enable_asserts) std.debug.assert(world_space_hinge_axis1.isNormalized(.{ .tolerance = 1.0e-4 }));
        if (Core.enable_asserts) std.debug.assert(world_space_hinge_axis2.isNormalized(.{ .tolerance = 1.0e-4 }));

        // Calculate: I1^-1 a
        self.inv_i1_a = body1.getMotionProperties().multiplyWorldSpaceInverseInertiaByVector(body1.getRotation(), world_space_hinge_axis1);

        // Calculate: I2^-1 b
        self.inv_i2_b = body2.getMotionProperties().multiplyWorldSpaceInverseInertiaByVector(body2.getRotation(), world_space_hinge_axis2);

        // K^-1 = 1 / (J M^-1 J^T) = 1 / (a^T I1^-1 a + r^2 * b^T I2^-1 b)
        const inv_effective_mass = (world_space_hinge_axis1.dot(self.inv_i1_a) + world_space_hinge_axis2.dot(self.inv_i2_b) * math.square(ratio));
        if (inv_effective_mass < math.flt_min)
            self.deactivate()
        else
            self.effective_mass = 1.0 / inv_effective_mass;
    }

    /// Deactivate this constraint
    pub fn deactivate(self: *GearConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    /// Check if constraint is active
    pub fn isActive(self: *const GearConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    /// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
    pub fn warmStart(self: *GearConstraintPart, body1: *Body, body2: *Body, warm_start_impulse_ratio: f32) void {
        self.total_lambda *= warm_start_impulse_ratio;
        _ = self.applyVelocityStep(body1, body2, self.total_lambda);
    }

    /// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inWorldSpaceHingeAxis1 The axis around which body 1 rotates
    /// @param inWorldSpaceHingeAxis2 The axis around which body 2 rotates
    /// @param inRatio The ratio between rotation and translation
    pub fn solveVelocityConstraint(self: *GearConstraintPart, body1: *Body, world_space_hinge_axis1: Vec3, body2: *Body, world_space_hinge_axis2: Vec3, ratio: f32) bool {
        // Lagrange multiplier is:
        //
        // lambda = -K^-1 (J v + b)
        const lambda = -self.effective_mass * (world_space_hinge_axis1.dot(body1.getAngularVelocity()) + ratio * world_space_hinge_axis2.dot(body2.getAngularVelocity()));
        self.total_lambda += lambda; // Store accumulated impulse

        return self.applyVelocityStep(body1, body2, lambda);
    }

    /// Return lagrange multiplier
    pub fn getTotalLambda(self: *const GearConstraintPart) f32 {
        return self.total_lambda;
    }

    /// Iteratively update the position constraint. Makes sure C(...) == 0.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inC Value of the constraint equation (C)
    /// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
    pub fn solvePositionConstraint(self: *const GearConstraintPart, body1: *Body, body2: *Body, c: f32, baumgarte: f32) bool {
        // Only apply position constraint when the constraint is hard, otherwise the velocity bias will fix the constraint
        if (c != 0.0) {
            // Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
            //
            // lambda = -K^-1 * beta / dt * C
            //
            // We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
            const lambda = -self.effective_mass * baumgarte * c;

            // Directly integrate velocity change for one time step
            //
            // Euler velocity integration:
            // dv = M^-1 P
            //
            // Impulse:
            // P = J^T lambda
            //
            // Euler position integration:
            // x' = x + dv * dt
            //
            // Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
            // Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
            // stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
            // integrate + a position integrate and then discard the velocity change.
            if (body1.isDynamic())
                body1.addRotationStep(self.inv_i1_a.mulScalar(lambda));
            if (body2.isDynamic())
                body2.addRotationStep(self.inv_i2_b.mulScalar(lambda));
            return true;
        }

        return false;
    }

    /// Save state of this constraint part
    pub fn saveState(self: *const GearConstraintPart, stream: *StateRecorder) void {
        stream.write(self.total_lambda);
    }

    /// Restore state of this constraint part
    pub fn restoreState(self: *GearConstraintPart, stream: *StateRecorder) void {
        stream.read(&self.total_lambda);
    }
};

const TestContext = @import("ConstraintPartTestContext.zig");

test "GearConstraintPart with two dynamic bodies" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../../StateRecorderImpl.zig").StateRecorderImpl;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    const body1 = bodies[0];
    const body2 = bodies[1];
    const a = Vec3.init(0.3, 1, -0.2).normalized();
    const b = Vec3.init(-1, 0.2, 0.4).normalized();
    const ratio: f32 = 2.0;

    // With a ratio of 1 one iteration makes w1 . a + r w2 . b zero. Jolt applies lambda I2^-1 b to body 2 while the
    // effective mass uses r^2 b^T I2^-1 b, so for other ratios an iteration only reduces the error.
    var unit_ratio: GearConstraintPart = .{};
    unit_ratio.calculateConstraintProperties(body1, a, body2, b, 1.0);
    const jv_unit = a.dot(body1.getAngularVelocity()) + b.dot(body2.getAngularVelocity());
    try testing.expect(@abs(jv_unit) > 0.1);
    try testing.expect(unit_ratio.solveVelocityConstraint(body1, a, body2, b, 1.0));
    try testing.expectApproxEqAbs(@as(f32, 0.0), a.dot(body1.getAngularVelocity()) + b.dot(body2.getAngularVelocity()), 1.0e-5);
    body2.getMotionPropertiesMut().setAngularVelocity(Vec3.init(0.4, -0.6, 0.1));

    var part: GearConstraintPart = .{};
    try testing.expect(!part.isActive());
    part.calculateConstraintProperties(body1, a, body2, b, ratio);
    try testing.expect(part.isActive());

    const jv_before = a.dot(body1.getAngularVelocity()) + ratio * b.dot(body2.getAngularVelocity());
    try testing.expect(@abs(jv_before) > 0.1);
    try testing.expect(part.solveVelocityConstraint(body1, a, body2, b, ratio));
    try testing.expect(@abs(a.dot(body1.getAngularVelocity()) + ratio * b.dot(body2.getAngularVelocity())) < 0.75 * @abs(jv_before));
    try testing.expectEqual(-part.effective_mass * jv_before, part.getTotalLambda());

    const lambda = part.getTotalLambda();
    part.warmStart(body1, body2, 0.5);
    try testing.expectEqual(lambda * 0.5, part.getTotalLambda());

    try testing.expect(!part.solvePositionConstraint(body1, body2, 0.0, 0.2));
    const rotation_before = body2.getRotation();
    try testing.expect(part.solvePositionConstraint(body1, body2, 0.2, 0.2));
    try testing.expect(!body2.getRotation().eql(rotation_before));

    var recorder = StateRecorderImpl.init(testing.allocator);
    defer recorder.deinit();
    part.saveState(recorder.recorder());
    var restored: GearConstraintPart = .{};
    restored.restoreState(recorder.recorder());
    try testing.expectEqual(part.getTotalLambda(), restored.getTotalLambda());

    part.deactivate();
    try testing.expect(!part.isActive());
    try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());

    // A body that cannot rotate around its axis: no effective mass
    const locked1 = try context.createBody(.dynamic, .{ .allowed_dofs = .{ .translation_y = true } });
    const locked2 = try context.createBody(.dynamic, .{ .allowed_dofs = .{ .translation_x = true } });
    var inactive: GearConstraintPart = .{ .total_lambda = 1.0 };
    inactive.calculateConstraintProperties(locked1, a, locked2, b, ratio);
    try testing.expect(!inactive.isActive());
    try testing.expectEqual(@as(f32, 0.0), inactive.getTotalLambda());
}
