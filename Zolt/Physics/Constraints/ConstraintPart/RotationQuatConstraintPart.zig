//! Port of: Jolt/Physics/Constraints/ConstraintPart/RotationQuatConstraintPart.h
//! Status: complete
//!
//! `mEffectiveMass(3, 3)` is `effective_mass.get(3, 3)`, the static sGetInvInitialOrientation is
//! `getInvInitialOrientation`.

const std = @import("std");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Quat = @import("../../../Math/Quat.zig").Quat;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Body = @import("../../Body/Body.zig").Body;
const StateRecorder = @import("../../StateRecorder.zig").StateRecorder;

/// Quaternion based constraint that constrains rotation around all axis so that only translation is allowed.
///
/// NOTE: This constraint part is more expensive than the RotationEulerConstraintPart and slightly more correct since
/// RotationEulerConstraintPart::SolvePositionConstraint contains an approximation. In practice the difference
/// is small, so the RotationEulerConstraintPart is probably the better choice.
///
/// Rotation is fixed between bodies like this:
///
/// q2 = q1 r0
///
/// Where:
/// q1, q2 = world space quaternions representing rotation of body 1 and 2.
/// r0 = initial rotation between bodies in local space of body 1, this can be calculated by:
///
/// q20 = q10 r0
/// <=> r0 = q10^* q20
///
/// Where:
/// q10, q20 = initial world space rotations of body 1 and 2.
/// q10^* = conjugate of quaternion q10 (which is the same as the inverse for a unit quaternion)
///
/// We exclusively use the conjugate below:
///
/// r0^* = q20^* q10
///
/// The error in the rotation is (in local space of body 1):
///
/// q2 = q1 error r0
/// <=> error = q1^* q2 r0^*
///
/// The imaginary part of the quaternion represents the rotation axis * sin(angle / 2). The real part of the quaternion
/// does not add any additional information (we know the quaternion in normalized) and we're removing 3 degrees of freedom
/// so we want 3 parameters. Therefore we define the constraint equation like:
///
/// C = A q1^* q2 r0^* = 0
///
/// Where (if you write a quaternion as [real-part, i-part, j-part, k-part]):
///
///         [0, 1, 0, 0]
///     A = [0, 0, 1, 0]
///         [0, 0, 0, 1]
///
/// or in our case since we store a quaternion like [i-part, j-part, k-part, real-part]:
///
///         [1, 0, 0, 0]
///     A = [0, 1, 0, 0]
///         [0, 0, 1, 0]
///
/// Time derivative:
///
/// d/dt C = A (q1^* d/dt(q2) + d/dt(q1^*) q2) r0^*
/// = A (q1^* (1/2 W2 q2) + (1/2 W1 q1)^* q2) r0^*
/// = 1/2 A (q1^* W2 q2 + q1^* W1^* q2) r0^*
/// = 1/2 A (q1^* W2 q2 - q1^* W1 * q2) r0^*
/// = 1/2 A ML(q1^*) MR(q2 r0^*) (W2 - W1)
/// = 1/2 A ML(q1^*) MR(q2 r0^*) A^T (w2 - w1)
///
/// Where:
/// W1 = [0, w1], W2 = [0, w2] (converting angular velocity to imaginary part of quaternion).
/// w1, w2 = angular velocity of body 1 and 2.
/// d/dt(q) = 1/2 W q (time derivative of a quaternion).
/// W^* = -W (conjugate negates angular velocity as quaternion).
/// ML(q): 4x4 matrix so that q * p = ML(q) * p, where q and p are quaternions.
/// MR(p): 4x4 matrix so that q * p = MR(p) * q, where q and p are quaternions.
/// A^T: Transpose of A.
///
/// Jacobian:
///
/// J = [0, -1/2 A ML(q1^*) MR(q2 r0^*) A^T, 0, 1/2 A ML(q1^*) MR(q2 r0^*) A^T]
/// = [0, -JP, 0, JP]
///
/// Suggested reading:
/// - 3D Constraint Derivations for Impulse Solvers - Marijn Tamis
/// - Game Physics Pearls - Section 9 - Quaternion Based Constraints - Claude Lacoursiere
pub const RotationQuatConstraintPart = struct {
    inv_i1_jpt: Mat44 = undefined,
    inv_i2_jpt: Mat44 = undefined,
    effective_mass: Mat44 = undefined,
    effective_mass_jp: Mat44 = undefined,
    total_lambda: Vec3 = Vec3.zero(),

    /// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
    fn applyVelocityStep(self: *const RotationQuatConstraintPart, body1: *Body, body2: *Body, lambda: Vec3) bool {
        // Apply impulse if delta is not zero
        if (!lambda.eql(Vec3.zero())) {
            // Calculate velocity change due to constraint
            //
            // Impulse:
            // P = J^T lambda
            //
            // Euler velocity integration:
            // v' = v + M^-1 P
            if (body1.isDynamic())
                body1.getMotionPropertiesMut().subAngularVelocityStep(self.inv_i1_jpt.multiply3x3(lambda));
            if (body2.isDynamic())
                body2.getMotionPropertiesMut().addAngularVelocityStep(self.inv_i2_jpt.multiply3x3(lambda));
            return true;
        }

        return false;
    }

    /// Return inverse of initial rotation from body 1 to body 2 in body 1 space
    pub fn getInvInitialOrientation(body1: *const Body, body2: *const Body) Quat {
        // q20 = q10 r0
        // <=> r0 = q10^-1 q20
        // <=> r0^-1 = q20^-1 q10
        //
        // where:
        //
        // q20 = initial orientation of body 2
        // q10 = initial orientation of body 1
        // r0 = initial rotation from body 1 to body 2
        return body2.getRotation().conjugated().mul(body1.getRotation());
    }

    /// Calculate properties used during the functions below
    pub fn calculateConstraintProperties(self: *RotationQuatConstraintPart, body1: *const Body, rotation1: Mat44, body2: *const Body, rotation2: Mat44, inv_initial_orientation: Quat) void {
        // Calculate: JP = 1/2 A ML(q1^*) MR(q2 r0^*) A^T
        const jp = Mat44.quatLeftMultiply(body1.getRotation().conjugated().mulScalar(0.5)).mul(Mat44.quatRightMultiply(body2.getRotation().mul(inv_initial_orientation))).getRotationSafe();

        // Calculate properties used during constraint solving
        const inv_i1 = if (body1.isDynamic()) body1.getMotionProperties().getInverseInertiaForRotation(rotation1) else Mat44.zero();
        const inv_i2 = if (body2.isDynamic()) body2.getMotionProperties().getInverseInertiaForRotation(rotation2) else Mat44.zero();
        self.inv_i1_jpt = inv_i1.multiply3x3RightTransposed(jp);
        self.inv_i2_jpt = inv_i2.multiply3x3RightTransposed(jp);

        // Calculate effective mass: K^-1 = (J M^-1 J^T)^-1
        // = (JP * I1^-1 * JP^T + JP * I2^-1 * JP^T)^-1
        // = (JP * (I1^-1 + I2^-1) * JP^T)^-1
        if (!self.effective_mass.setInversed3x3(jp.multiply3x3Mat44(inv_i1.add(inv_i2)).multiply3x3RightTransposed(jp)))
            self.deactivate()
        else
            self.effective_mass_jp = self.effective_mass.multiply3x3Mat44(jp);
    }

    /// Deactivate this constraint
    pub fn deactivate(self: *RotationQuatConstraintPart) void {
        self.effective_mass = Mat44.zero();
        self.effective_mass_jp = Mat44.zero();
        self.total_lambda = Vec3.zero();
    }

    /// Check if constraint is active
    pub fn isActive(self: *const RotationQuatConstraintPart) bool {
        return self.effective_mass.get(3, 3) != 0.0;
    }

    /// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
    pub fn warmStart(self: *RotationQuatConstraintPart, body1: *Body, body2: *Body, warm_start_impulse_ratio: f32) void {
        self.total_lambda = self.total_lambda.mulScalar(warm_start_impulse_ratio);
        _ = self.applyVelocityStep(body1, body2, self.total_lambda);
    }

    /// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
    pub fn solveVelocityConstraint(self: *RotationQuatConstraintPart, body1: *Body, body2: *Body) bool {
        // Calculate lagrange multiplier:
        //
        // lambda = -K^-1 (J v + b)
        const lambda = self.effective_mass_jp.multiply3x3(body1.getAngularVelocity().sub(body2.getAngularVelocity()));
        self.total_lambda = self.total_lambda.add(lambda);
        return self.applyVelocityStep(body1, body2, lambda);
    }

    /// Iteratively update the position constraint. Makes sure C(...) = 0.
    pub fn solvePositionConstraint(self: *const RotationQuatConstraintPart, body1: *Body, body2: *Body, inv_initial_orientation: Quat, baumgarte: f32) bool {
        // Calculate constraint equation
        const c = body1.getRotation().conjugated().mul(body2.getRotation()).mul(inv_initial_orientation).getXYZ();
        if (!c.eql(Vec3.zero())) {
            // Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
            //
            // lambda = -K^-1 * beta / dt * C
            //
            // We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
            const lambda = self.effective_mass.mulScalar(-baumgarte).mulVec3(c);

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
                body1.subRotationStep(self.inv_i1_jpt.multiply3x3(lambda));
            if (body2.isDynamic())
                body2.addRotationStep(self.inv_i2_jpt.multiply3x3(lambda));
            return true;
        }

        return false;
    }

    /// Return lagrange multiplier
    pub fn getTotalLambda(self: *const RotationQuatConstraintPart) Vec3 {
        return self.total_lambda;
    }

    /// Save state of this constraint part
    pub fn saveState(self: *const RotationQuatConstraintPart, stream: *StateRecorder) void {
        stream.write(self.total_lambda);
    }

    /// Restore state of this constraint part
    pub fn restoreState(self: *RotationQuatConstraintPart, stream: *StateRecorder) void {
        stream.read(&self.total_lambda);
    }
};

const TestContext = @import("ConstraintPartTestContext.zig");

test "RotationQuatConstraintPart for every combination of motion types" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../../StateRecorderImpl.zig").StateRecorderImpl;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);

        const inv_initial_orientation = RotationQuatConstraintPart.getInvInitialOrientation(body1, body2).mul(Quat.rotation(Vec3.init(1, 0, 1).normalized(), 0.2));

        var part: RotationQuatConstraintPart = .{};
        part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), body2, Mat44.rotationQuat(body2.getRotation()), inv_initial_orientation);
        const any_dynamic = body1.isDynamic() or body2.isDynamic();
        try testing.expectEqual(any_dynamic, part.isActive());
        if (!any_dynamic) {
            try testing.expect(part.getTotalLambda().eql(Vec3.zero()));
            try testing.expect(part.effective_mass_jp.eql(Mat44.zero()));
            continue;
        }

        // Velocity: the relative angular velocity is removed
        try testing.expect(part.solveVelocityConstraint(body1, body2));
        try testing.expect(body1.getAngularVelocity().sub(body2.getAngularVelocity()).isNearZero(.{ .max_dist_sq = 1.0e-10 }));
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Warm start
        const lambda = part.getTotalLambda();
        part.warmStart(body1, body2, 0.5);
        try testing.expect(part.getTotalLambda().eql(lambda.mulScalar(0.5)));

        // Position: the constraint error decreases
        const errorLength = struct {
            fn get(b1: *const Body, b2: *const Body, q: Quat) f32 {
                return b1.getRotation().conjugated().mul(b2.getRotation()).mul(q).getXYZ().length();
            }
        }.get;
        const error_before = errorLength(body1, body2, inv_initial_orientation);
        try testing.expect(error_before > 0.01);
        try testing.expect(part.solvePositionConstraint(body1, body2, inv_initial_orientation, 0.5));
        try testing.expect(errorLength(body1, body2, inv_initial_orientation) < error_before);
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // State
        var recorder = StateRecorderImpl.init(testing.allocator);
        defer recorder.deinit();
        part.saveState(recorder.recorder());
        var restored: RotationQuatConstraintPart = .{};
        restored.restoreState(recorder.recorder());
        try testing.expect(restored.getTotalLambda().eql(part.getTotalLambda()));

        part.deactivate();
        try testing.expect(!part.isActive());
        try testing.expect(part.getTotalLambda().eql(Vec3.zero()));
    }
}

test "RotationQuatConstraintPart deactivates without rotational freedom" {
    const testing = std.testing;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const body1 = try context.createBody(.dynamic, .{ .allowed_dofs = .{ .translation_x = true, .translation_z = true } });
    const body2 = try context.createBody(.kinematic, .{});
    var part: RotationQuatConstraintPart = .{ .total_lambda = Vec3.init(1, 1, 1) };
    part.calculateConstraintProperties(body1, Mat44.identity(), body2, Mat44.identity(), Quat.identity());
    try testing.expect(!part.isActive());
    try testing.expect(part.getTotalLambda().eql(Vec3.zero()));
}
