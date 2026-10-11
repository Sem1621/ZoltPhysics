//! Port of: Jolt/Physics/Constraints/ConstraintPart/RackAndPinionConstraintPart.h
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

/// Constraint that constrains a rotation to a translation
///
/// Constraint equation:
///
/// C = Theta(t) - r d(t)
///
/// Derivative:
///
/// d/dt C = 0
/// <=> w1 . a - r v2 . b = 0
///
/// Jacobian:
///
/// \f[J = \begin{bmatrix}0 & a^T & -r b^T & 0\end{bmatrix}\f]
///
/// Used terms (here and below, everything in world space):\n
/// a = axis around which body 1 rotates (normalized).\n
/// b = axis along which body 2 slides (normalized).\n
/// Theta(t) = rotation around a of body 1.\n
/// d(t) = distance body 2 slides.\n
/// r = ratio between rotation and translation.\n
/// v = [v1, w1, v2, w2].\n
/// v1, v2 = linear velocity of body 1 and 2.\n
/// w1, w2 = angular velocity of body 1 and 2.\n
/// M = mass matrix, a diagonal matrix of the mass and inertia with diagonal [m1, I1, m2, I2].\n
/// \f$K^{-1} = \left( J M^{-1} J^T \right)^{-1}\f$ = effective mass.\n
/// \f$\beta\f$ = baumgarte constant.
pub const RackAndPinionConstraintPart = struct {
    inv_i1_a: Vec3 = undefined,
    ratio_inv_m2_b: Vec3 = undefined,
    effective_mass: f32 = 0.0,
    total_lambda: f32 = 0.0,

    /// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
    fn applyVelocityStep(self: *const RackAndPinionConstraintPart, body1: *Body, body2: *Body, lambda: f32) bool {
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
            body2.getMotionPropertiesMut().subLinearVelocityStep(self.ratio_inv_m2_b.mulScalar(lambda));
            return true;
        }

        return false;
    }

    /// Calculate properties used during the functions below
    /// @param inBody1 The first body that this constraint is attached to
    /// @param inBody2 The second body that this constraint is attached to
    /// @param inWorldSpaceHingeAxis The axis around which body 1 rotates
    /// @param inWorldSpaceSliderAxis The axis along which body 2 slides
    /// @param inRatio The ratio between rotation and translation
    pub fn calculateConstraintProperties(self: *RackAndPinionConstraintPart, body1: *const Body, world_space_hinge_axis: Vec3, body2: *const Body, world_space_slider_axis: Vec3, ratio: f32) void {
        if (Core.enable_asserts) std.debug.assert(world_space_hinge_axis.isNormalized(.{ .tolerance = 1.0e-4 }));
        if (Core.enable_asserts) std.debug.assert(world_space_slider_axis.isNormalized(.{ .tolerance = 1.0e-4 }));

        // Calculate: I1^-1 a
        self.inv_i1_a = body1.getMotionProperties().multiplyWorldSpaceInverseInertiaByVector(body1.getRotation(), world_space_hinge_axis);

        // Calculate: r/m2 b
        const inv_m2 = body2.getMotionProperties().getInverseMass();
        self.ratio_inv_m2_b = world_space_slider_axis.mulScalar(ratio * inv_m2);

        // K^-1 = 1 / (J M^-1 J^T) = 1 / (a^T I1^-1 a + 1/m2 * r^2 * b . b)
        const inv_effective_mass = (world_space_hinge_axis.dot(self.inv_i1_a) + inv_m2 * math.square(ratio));
        if (inv_effective_mass < math.flt_min)
            self.deactivate()
        else
            self.effective_mass = 1.0 / inv_effective_mass;
    }

    /// Deactivate this constraint
    pub fn deactivate(self: *RackAndPinionConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    /// Check if constraint is active
    pub fn isActive(self: *const RackAndPinionConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    /// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
    pub fn warmStart(self: *RackAndPinionConstraintPart, body1: *Body, body2: *Body, warm_start_impulse_ratio: f32) void {
        self.total_lambda *= warm_start_impulse_ratio;
        _ = self.applyVelocityStep(body1, body2, self.total_lambda);
    }

    /// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inWorldSpaceHingeAxis The axis around which body 1 rotates
    /// @param inWorldSpaceSliderAxis The axis along which body 2 slides
    /// @param inRatio The ratio between rotation and translation
    pub fn solveVelocityConstraint(self: *RackAndPinionConstraintPart, body1: *Body, world_space_hinge_axis: Vec3, body2: *Body, world_space_slider_axis: Vec3, ratio: f32) bool {
        // Lagrange multiplier is:
        //
        // lambda = -K^-1 (J v + b)
        const lambda = self.effective_mass * (ratio * world_space_slider_axis.dot(body2.getLinearVelocity()) - world_space_hinge_axis.dot(body1.getAngularVelocity()));
        self.total_lambda += lambda; // Store accumulated impulse

        return self.applyVelocityStep(body1, body2, lambda);
    }

    /// Return lagrange multiplier
    pub fn getTotalLambda(self: *const RackAndPinionConstraintPart) f32 {
        return self.total_lambda;
    }

    /// Iteratively update the position constraint. Makes sure C(...) == 0.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inC Value of the constraint equation (C)
    /// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
    pub fn solvePositionConstraint(self: *const RackAndPinionConstraintPart, body1: *Body, body2: *Body, c: f32, baumgarte: f32) bool {
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
                body2.subPositionStep(self.ratio_inv_m2_b.mulScalar(lambda));
            return true;
        }

        return false;
    }

    /// Save state of this constraint part
    pub fn saveState(self: *const RackAndPinionConstraintPart, stream: *StateRecorder) void {
        stream.write(self.total_lambda);
    }

    /// Restore state of this constraint part
    pub fn restoreState(self: *RackAndPinionConstraintPart, stream: *StateRecorder) void {
        stream.read(&self.total_lambda);
    }
};

const TestContext = @import("ConstraintPartTestContext.zig");

test "RackAndPinionConstraintPart with two dynamic bodies" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../../StateRecorderImpl.zig").StateRecorderImpl;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    const body1 = bodies[0];
    const body2 = bodies[1];
    const hinge = Vec3.init(0.3, 1, -0.2).normalized();
    const slider = Vec3.init(-1, 0.2, 0.4).normalized();
    const ratio: f32 = 3.0;

    var part: RackAndPinionConstraintPart = .{};
    part.calculateConstraintProperties(body1, hinge, body2, slider, ratio);
    try testing.expect(part.isActive());

    // One iteration makes w1 . a - r v2 . b zero
    const jv_before = hinge.dot(body1.getAngularVelocity()) - ratio * slider.dot(body2.getLinearVelocity());
    try testing.expect(@abs(jv_before) > 0.1);
    try testing.expect(part.solveVelocityConstraint(body1, hinge, body2, slider, ratio));
    try testing.expectApproxEqAbs(@as(f32, 0.0), hinge.dot(body1.getAngularVelocity()) - ratio * slider.dot(body2.getLinearVelocity()), 1.0e-5);

    const lambda = part.getTotalLambda();
    part.warmStart(body1, body2, 0.5);
    try testing.expectEqual(lambda * 0.5, part.getTotalLambda());

    try testing.expect(!part.solvePositionConstraint(body1, body2, 0.0, 0.2));
    const position_before = body2.getCenterOfMassPosition();
    const rotation_before = body1.getRotation();
    try testing.expect(part.solvePositionConstraint(body1, body2, 0.2, 0.2));
    try testing.expect(!body2.getCenterOfMassPosition().eql(position_before));
    try testing.expect(!body1.getRotation().eql(rotation_before));

    var recorder = StateRecorderImpl.init(testing.allocator);
    defer recorder.deinit();
    part.saveState(recorder.recorder());
    var restored: RackAndPinionConstraintPart = .{};
    restored.restoreState(recorder.recorder());
    try testing.expectEqual(part.getTotalLambda(), restored.getTotalLambda());

    part.deactivate();
    try testing.expect(!part.isActive());
    try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());

    // Neither rotation of body 1 nor translation of body 2: no effective mass
    const locked1 = try context.createBody(.dynamic, .{ .allowed_dofs = .{ .translation_y = true } });
    const locked2 = try context.createBody(.dynamic, .{ .allowed_dofs = .{ .rotation_x = true } });
    var inactive: RackAndPinionConstraintPart = .{ .total_lambda = 1.0 };
    inactive.calculateConstraintProperties(locked1, hinge, locked2, slider, ratio);
    try testing.expect(!inactive.isActive());
    try testing.expectEqual(@as(f32, 0.0), inactive.getTotalLambda());
}
