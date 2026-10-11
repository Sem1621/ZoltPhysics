//! Port of: Jolt/Physics/Constraints/ConstraintPart/RotationEulerConstraintPart.h
//! Status: complete
//!
//! `mEffectiveMass(3, 3)` is `effective_mass.get(3, 3)`. The static functions sGetInvInitialOrientation,
//! sGetInvInitialOrientationXY and sGetInvInitialOrientationXZ are `getInvInitialOrientation`,
//! `getInvInitialOrientationXY` and `getInvInitialOrientationXZ`.

const std = @import("std");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../../Math/Vec4.zig").Vec4;
const Quat = @import("../../../Math/Quat.zig").Quat;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Body = @import("../../Body/Body.zig").Body;
const StateRecorder = @import("../../StateRecorder.zig").StateRecorder;

/// Constrains rotation around all axis so that only translation is allowed
///
/// Based on: "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.5.1
///
/// Constraint equation (eq 129):
///
/// \f[C = \begin{bmatrix}\Delta\theta_x, \Delta\theta_y, \Delta\theta_z\end{bmatrix}\f]
///
/// Jacobian (eq 131):
///
/// \f[J = \begin{bmatrix}0 & -E & 0 & E\end{bmatrix}\f]
///
/// Used terms (here and below, everything in world space):\n
/// delta_theta_* = difference in rotation between initial rotation of bodies 1 and 2.\n
/// x1, x2 = center of mass for the bodies.\n
/// v = [v1, w1, v2, w2].\n
/// v1, v2 = linear velocity of body 1 and 2.\n
/// w1, w2 = angular velocity of body 1 and 2.\n
/// M = mass matrix, a diagonal matrix of the mass and inertia with diagonal [m1, I1, m2, I2].\n
/// \f$K^{-1} = \left( J M^{-1} J^T \right)^{-1}\f$ = effective mass.\n
/// b = velocity bias.\n
/// \f$\beta\f$ = baumgarte constant.\n
/// E = identity matrix.\n
pub const RotationEulerConstraintPart = struct {
    inv_i1: Mat44 = undefined,
    inv_i2: Mat44 = undefined,
    effective_mass: Mat44 = undefined,
    total_lambda: Vec3 = Vec3.zero(),

    /// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
    fn applyVelocityStep(self: *const RotationEulerConstraintPart, body1: *Body, body2: *Body, lambda: Vec3) bool {
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
                body1.getMotionPropertiesMut().subAngularVelocityStep(self.inv_i1.multiply3x3(lambda));
            if (body2.isDynamic())
                body2.getMotionPropertiesMut().addAngularVelocityStep(self.inv_i2.multiply3x3(lambda));
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

    /// @brief Return inverse of initial rotation from body 1 to body 2 in body 1 space
    /// @param inAxisX1 Reference axis X for body 1
    /// @param inAxisY1 Reference axis Y for body 1
    /// @param inAxisX2 Reference axis X for body 2
    /// @param inAxisY2 Reference axis Y for body 2
    pub fn getInvInitialOrientationXY(axis_x1: Vec3, axis_y1: Vec3, axis_x2: Vec3, axis_y2: Vec3) Quat {
        // Store inverse of initial rotation from body 1 to body 2 in body 1 space:
        //
        // q20 = q10 r0
        // <=> r0 = q10^-1 q20
        // <=> r0^-1 = q20^-1 q10
        //
        // where:
        //
        // q10, q20 = world space initial orientation of body 1 and 2
        // r0 = initial rotation from body 1 to body 2 in local space of body 1
        //
        // We can also write this in terms of the constraint matrices:
        //
        // q20 c2 = q10 c1
        // <=> q20 = q10 c1 c2^-1
        // => r0 = c1 c2^-1
        // <=> r0^-1 = c2 c1^-1
        //
        // where:
        //
        // c1, c2 = matrix that takes us from body 1 and 2 COM to constraint space 1 and 2
        if (axis_x1.eql(axis_x2) and axis_y1.eql(axis_y2)) {
            // Axis are the same -> identity transform
            return Quat.identity();
        } else {
            const constraint1 = Mat44.init(Vec4.fromVec3W(axis_x1, 0), Vec4.fromVec3W(axis_y1, 0), Vec4.fromVec3W(axis_x1.cross(axis_y1), 0), Vec4.init(0, 0, 0, 1));
            const constraint2 = Mat44.init(Vec4.fromVec3W(axis_x2, 0), Vec4.fromVec3W(axis_y2, 0), Vec4.fromVec3W(axis_x2.cross(axis_y2), 0), Vec4.init(0, 0, 0, 1));
            return constraint2.getQuaternion().mul(constraint1.getQuaternion().conjugated());
        }
    }

    /// @brief Return inverse of initial rotation from body 1 to body 2 in body 1 space
    /// @param inAxisX1 Reference axis X for body 1
    /// @param inAxisZ1 Reference axis Z for body 1
    /// @param inAxisX2 Reference axis X for body 2
    /// @param inAxisZ2 Reference axis Z for body 2
    pub fn getInvInitialOrientationXZ(axis_x1: Vec3, axis_z1: Vec3, axis_x2: Vec3, axis_z2: Vec3) Quat {
        // See comment at sGetInvInitialOrientationXY
        if (axis_x1.eql(axis_x2) and axis_z1.eql(axis_z2)) {
            return Quat.identity();
        } else {
            const constraint1 = Mat44.init(Vec4.fromVec3W(axis_x1, 0), Vec4.fromVec3W(axis_z1.cross(axis_x1), 0), Vec4.fromVec3W(axis_z1, 0), Vec4.init(0, 0, 0, 1));
            const constraint2 = Mat44.init(Vec4.fromVec3W(axis_x2, 0), Vec4.fromVec3W(axis_z2.cross(axis_x2), 0), Vec4.fromVec3W(axis_z2, 0), Vec4.init(0, 0, 0, 1));
            return constraint2.getQuaternion().mul(constraint1.getQuaternion().conjugated());
        }
    }

    /// Calculate properties used during the functions below
    pub fn calculateConstraintProperties(self: *RotationEulerConstraintPart, body1: *const Body, rotation1: Mat44, body2: *const Body, rotation2: Mat44) void {
        // Calculate properties used during constraint solving
        self.inv_i1 = if (body1.isDynamic()) body1.getMotionProperties().getInverseInertiaForRotation(rotation1) else Mat44.zero();
        self.inv_i2 = if (body2.isDynamic()) body2.getMotionProperties().getInverseInertiaForRotation(rotation2) else Mat44.zero();

        // Calculate effective mass: K^-1 = (J M^-1 J^T)^-1
        var inertia_sum = self.inv_i1.add(self.inv_i2);
        if (!self.effective_mass.setInversed3x3(inertia_sum)) {
            // If a column is zero, the axis is locked and we set the column to identity.
            // This does not matter because any impulse will always be multiplied with mInvI1 or mInvI2 which will result in zero for the locked coordinate.
            const zero = Vec4.zero();
            if (inertia_sum.getColumn4(0).eql(zero))
                inertia_sum.setColumn4(0, Vec4.init(1, 0, 0, 0));
            if (inertia_sum.getColumn4(1).eql(zero))
                inertia_sum.setColumn4(1, Vec4.init(0, 1, 0, 0));
            if (inertia_sum.getColumn4(2).eql(zero))
                inertia_sum.setColumn4(2, Vec4.init(0, 0, 1, 0));
            if (!self.effective_mass.setInversed3x3(inertia_sum))
                self.deactivate();
        }
    }

    /// Deactivate this constraint
    pub fn deactivate(self: *RotationEulerConstraintPart) void {
        self.effective_mass = Mat44.zero();
        self.total_lambda = Vec3.zero();
    }

    /// Check if constraint is active
    pub fn isActive(self: *const RotationEulerConstraintPart) bool {
        return self.effective_mass.get(3, 3) != 0.0;
    }

    /// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
    pub fn warmStart(self: *RotationEulerConstraintPart, body1: *Body, body2: *Body, warm_start_impulse_ratio: f32) void {
        self.total_lambda = self.total_lambda.mulScalar(warm_start_impulse_ratio);
        _ = self.applyVelocityStep(body1, body2, self.total_lambda);
    }

    /// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
    pub fn solveVelocityConstraint(self: *RotationEulerConstraintPart, body1: *Body, body2: *Body) bool {
        // Calculate lagrange multiplier:
        //
        // lambda = -K^-1 (J v + b)
        const lambda = self.effective_mass.multiply3x3(body1.getAngularVelocity().sub(body2.getAngularVelocity()));
        self.total_lambda = self.total_lambda.add(lambda);
        return self.applyVelocityStep(body1, body2, lambda);
    }

    /// Iteratively update the position constraint. Makes sure C(...) = 0.
    pub fn solvePositionConstraint(self: *const RotationEulerConstraintPart, body1: *Body, body2: *Body, inv_initial_orientation: Quat, baumgarte: f32) bool {
        // Calculate difference in rotation
        //
        // The rotation should be:
        //
        // q2 = q1 r0
        //
        // But because of drift the actual rotation is
        //
        // q2 = diff q1 r0
        // <=> diff = q2 r0^-1 q1^-1
        //
        // Where:
        // q1 = current rotation of body 1
        // q2 = current rotation of body 2
        // diff = error that needs to be reduced to zero
        const diff = body2.getRotation().mul(inv_initial_orientation).mul(body1.getRotation().conjugated());

        // A quaternion can be seen as:
        //
        // q = [sin(theta / 2) * v, cos(theta/2)]
        //
        // Where:
        // v = rotation vector
        // theta = rotation angle
        //
        // If we assume theta is small (error is small) then sin(x) = x so an approximation of the error angles is:
        const err = diff.ensureWPositive().getXYZ().mulScalar(2.0);
        if (!err.eql(Vec3.zero())) {
            // Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
            //
            // lambda = -K^-1 * beta / dt * C
            //
            // We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
            const lambda = self.effective_mass.mulScalar(-baumgarte).mulVec3(err);

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
                body1.subRotationStep(self.inv_i1.multiply3x3(lambda));
            if (body2.isDynamic())
                body2.addRotationStep(self.inv_i2.multiply3x3(lambda));
            return true;
        }

        return false;
    }

    /// Return lagrange multiplier
    pub fn getTotalLambda(self: *const RotationEulerConstraintPart) Vec3 {
        return self.total_lambda;
    }

    /// Save state of this constraint part
    pub fn saveState(self: *const RotationEulerConstraintPart, stream: *StateRecorder) void {
        stream.write(self.total_lambda);
    }

    /// Restore state of this constraint part
    pub fn restoreState(self: *RotationEulerConstraintPart, stream: *StateRecorder) void {
        stream.read(&self.total_lambda);
    }
};

const TestContext = @import("ConstraintPartTestContext.zig");

test "RotationEulerConstraintPart for every combination of motion types" {
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

        // The bodies were created with different rotations, keep a slightly different relative rotation
        const inv_initial_orientation = RotationEulerConstraintPart.getInvInitialOrientation(body1, body2).mul(Quat.rotation(Vec3.axisY(), 0.1));

        var part: RotationEulerConstraintPart = .{};
        part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), body2, Mat44.rotationQuat(body2.getRotation()));
        // Without a dynamic body the inertia sum is zero: every column is replaced by an identity column, so the part
        // stays active (Jolt's behavior), but the solver cannot move a body
        try testing.expect(part.isActive());
        if (!body1.isDynamic() and !body2.isDynamic()) {
            try testing.expect(part.effective_mass.eql(Mat44.identity()));
            _ = part.solveVelocityConstraint(body1, body2);
            _ = part.solvePositionConstraint(body1, body2, inv_initial_orientation, 0.5);
            try TestContext.checkUnmoved(body1, before1);
            try TestContext.checkUnmoved(body2, before2);
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
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Position: the rotation error decreases
        const errorAngle = struct {
            fn get(b1: *const Body, b2: *const Body, q: Quat) f32 {
                return b2.getRotation().mul(q).mul(b1.getRotation().conjugated()).ensureWPositive().getXYZ().length();
            }
        }.get;
        const error_before = errorAngle(body1, body2, inv_initial_orientation);
        try testing.expect(error_before > 0.01);
        try testing.expect(part.solvePositionConstraint(body1, body2, inv_initial_orientation, 0.5));
        try testing.expect(errorAngle(body1, body2, inv_initial_orientation) < error_before);
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // State
        var recorder = StateRecorderImpl.init(testing.allocator);
        defer recorder.deinit();
        part.saveState(recorder.recorder());
        var restored: RotationEulerConstraintPart = .{};
        restored.restoreState(recorder.recorder());
        try testing.expect(restored.getTotalLambda().eql(part.getTotalLambda()));

        part.deactivate();
        try testing.expect(!part.isActive());
        try testing.expect(part.getTotalLambda().eql(Vec3.zero()));
    }
}

test "RotationEulerConstraintPart locked axes and initial orientations" {
    const testing = std.testing;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    // A 2D body only rotates around Z: the zero columns of the inertia sum are replaced by identity columns
    const body1 = try context.createBody(.dynamic, .{ .allowed_dofs = .plane_2d });
    const body2 = try context.createBody(.static, .{});
    var part: RotationEulerConstraintPart = .{};
    part.calculateConstraintProperties(body1, Mat44.identity(), body2, Mat44.identity());
    try testing.expect(part.isActive());
    try testing.expectEqual(@as(f32, 1.0), part.effective_mass.get(0, 0));
    try testing.expectEqual(@as(f32, 1.0), part.effective_mass.get(1, 1));

    // A body that cannot rotate at all: the inverse of the identity
    const body3 = try context.createBody(.dynamic, .{ .allowed_dofs = .{ .translation_x = true } });
    var locked: RotationEulerConstraintPart = .{};
    locked.calculateConstraintProperties(body3, Mat44.identity(), body2, Mat44.identity());
    try testing.expect(locked.isActive());
    try testing.expect(locked.effective_mass.eql(Mat44.identity()));

    // Identical axes give the identity
    try testing.expect(RotationEulerConstraintPart.getInvInitialOrientationXY(Vec3.axisX(), Vec3.axisY(), Vec3.axisX(), Vec3.axisY()).eql(Quat.identity()));
    try testing.expect(RotationEulerConstraintPart.getInvInitialOrientationXZ(Vec3.axisX(), Vec3.axisZ(), Vec3.axisX(), Vec3.axisZ()).eql(Quat.identity()));

    // Axes rotated by q: the inverse initial orientation is q
    const q = Quat.rotation(Vec3.init(1, 1, 0).normalized(), 0.7);
    const xy = RotationEulerConstraintPart.getInvInitialOrientationXY(Vec3.axisX(), Vec3.axisY(), q.mulVec3(Vec3.axisX()), q.mulVec3(Vec3.axisY()));
    try testing.expect(xy.isClose(q, .{ .max_dist_sq = 1.0e-10 }) or xy.isClose(q.negate(), .{ .max_dist_sq = 1.0e-10 }));
    const xz = RotationEulerConstraintPart.getInvInitialOrientationXZ(Vec3.axisX(), Vec3.axisZ(), q.mulVec3(Vec3.axisX()), q.mulVec3(Vec3.axisZ()));
    try testing.expect(xz.isClose(q, .{ .max_dist_sq = 1.0e-10 }) or xz.isClose(q.negate(), .{ .max_dist_sq = 1.0e-10 }));
}
