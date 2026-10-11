//! Port of: Jolt/Physics/Constraints/ConstraintPart/PointConstraintPart.h
//! Status: complete
//!
//! `mEffectiveMass(3, 3)` is `effective_mass.get(3, 3)`; `JPH_IF_DEBUG(mInvI1_R1X = Mat44::sNaN();)` is
//! `if (builtin.mode == .Debug)`.

const std = @import("std");
const builtin = @import("builtin");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Body = @import("../../Body/Body.zig").Body;
const StateRecorder = @import("../../StateRecorder.zig").StateRecorder;

/// Constrains movement along 3 axis
///
/// @see "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.2.1
///
/// Constraint equation (eq 45):
///
/// \f[C = p_2 - p_1\f]
///
/// Jacobian (transposed) (eq 47):
///
/// \f[J^T = \begin{bmatrix}-E & r1x & E & -r2x^T\end{bmatrix}
/// = \begin{bmatrix}-E^T \\ r1x^T \\ E^T \\ -r2x^T\end{bmatrix}
/// = \begin{bmatrix}-E \\ -r1x \\ E \\ r2x\end{bmatrix}\f]
///
/// Used terms (here and below, everything in world space):\n
/// p1, p2 = constraint points.\n
/// r1 = p1 - x1.\n
/// r2 = p2 - x2.\n
/// r1x = 3x3 matrix for which r1x v = r1 x v (cross product).\n
/// x1, x2 = center of mass for the bodies.\n
/// v = [v1, w1, v2, w2].\n
/// v1, v2 = linear velocity of body 1 and 2.\n
/// w1, w2 = angular velocity of body 1 and 2.\n
/// M = mass matrix, a diagonal matrix of the mass and inertia with diagonal [m1, I1, m2, I2].\n
/// \f$K^{-1} = \left( J M^{-1} J^T \right)^{-1}\f$ = effective mass.\n
/// b = velocity bias.\n
/// \f$\beta\f$ = baumgarte constant.\n
/// E = identity matrix.
pub const PointConstraintPart = struct {
    r1: Vec3 = undefined,
    r2: Vec3 = undefined,
    inv_i1_r1x: Mat44 = undefined,
    inv_i2_r2x: Mat44 = undefined,
    effective_mass: Mat44 = undefined,
    total_lambda: Vec3 = Vec3.zero(),

    fn applyVelocityStep(self: *const PointConstraintPart, body1: *Body, body2: *Body, lambda: Vec3) bool {
        // Apply impulse if delta is not zero
        if (!lambda.eql(Vec3.zero())) {
            // Calculate velocity change due to constraint
            //
            // Impulse:
            // P = J^T lambda
            //
            // Euler velocity integration:
            // v' = v + M^-1 P
            if (body1.isDynamic()) {
                const mp1 = body1.getMotionPropertiesMut();
                mp1.subLinearVelocityStep(lambda.mulScalar(mp1.getInverseMass()));
                mp1.subAngularVelocityStep(self.inv_i1_r1x.mulVec3(lambda));
            }
            if (body2.isDynamic()) {
                const mp2 = body2.getMotionPropertiesMut();
                mp2.addLinearVelocityStep(lambda.mulScalar(mp2.getInverseMass()));
                mp2.addAngularVelocityStep(self.inv_i2_r2x.mulVec3(lambda));
            }
            return true;
        }

        return false;
    }

    /// Calculate properties used during the functions below
    /// @param inBody1 The first body that this constraint is attached to
    /// @param inBody2 The second body that this constraint is attached to
    /// @param inRotation1 The 3x3 rotation matrix for body 1 (translation part is ignored)
    /// @param inRotation2 The 3x3 rotation matrix for body 2 (translation part is ignored)
    /// @param inR1 Local space vector from center of mass to constraint point for body 1
    /// @param inR2 Local space vector from center of mass to constraint point for body 2
    pub fn calculateConstraintProperties(self: *PointConstraintPart, body1: *const Body, rotation1: Mat44, r1: Vec3, body2: *const Body, rotation2: Mat44, r2: Vec3) void {
        // Positions where the point constraint acts on (middle point between center of masses) in world space
        self.r1 = rotation1.multiply3x3(r1);
        self.r2 = rotation2.multiply3x3(r2);

        // Calculate effective mass: K^-1 = (J M^-1 J^T)^-1
        // Using: I^-1 = R * Ibody^-1 * R^T
        var summed_inv_mass: f32 = undefined;
        var inv_effective_mass: Mat44 = undefined;
        if (body1.isDynamic()) {
            const mp1 = body1.getMotionProperties();
            const inv_i1 = mp1.getInverseInertiaForRotation(rotation1);
            summed_inv_mass = mp1.getInverseMass();

            const r1x = Mat44.crossProduct(self.r1);
            self.inv_i1_r1x = inv_i1.multiply3x3Mat44(r1x);
            inv_effective_mass = r1x.multiply3x3Mat44(inv_i1).multiply3x3RightTransposed(r1x);
        } else {
            if (builtin.mode == .Debug) self.inv_i1_r1x = Mat44.nan();

            summed_inv_mass = 0.0;
            inv_effective_mass = Mat44.zero();
        }

        if (body2.isDynamic()) {
            const mp2 = body2.getMotionProperties();
            const inv_i2 = mp2.getInverseInertiaForRotation(rotation2);
            summed_inv_mass += mp2.getInverseMass();

            const r2x = Mat44.crossProduct(self.r2);
            self.inv_i2_r2x = inv_i2.multiply3x3Mat44(r2x);
            inv_effective_mass = inv_effective_mass.add(r2x.multiply3x3Mat44(inv_i2).multiply3x3RightTransposed(r2x));
        } else {
            if (builtin.mode == .Debug) self.inv_i2_r2x = Mat44.nan();
        }

        inv_effective_mass = inv_effective_mass.add(Mat44.scale(summed_inv_mass));
        if (!self.effective_mass.setInversed3x3(inv_effective_mass))
            self.deactivate();
    }

    /// Deactivate this constraint
    pub fn deactivate(self: *PointConstraintPart) void {
        self.effective_mass = Mat44.zero();
        self.total_lambda = Vec3.zero();
    }

    /// Check if constraint is active
    pub fn isActive(self: *const PointConstraintPart) bool {
        return self.effective_mass.get(3, 3) != 0.0;
    }

    /// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
    pub fn warmStart(self: *PointConstraintPart, body1: *Body, body2: *Body, warm_start_impulse_ratio: f32) void {
        self.total_lambda = self.total_lambda.mulScalar(warm_start_impulse_ratio);
        _ = self.applyVelocityStep(body1, body2, self.total_lambda);
    }

    /// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    pub fn solveVelocityConstraint(self: *PointConstraintPart, body1: *Body, body2: *Body) bool {
        // Calculate lagrange multiplier:
        //
        // lambda = -K^-1 (J v + b)
        const lambda = self.effective_mass.mulVec3(body1.getLinearVelocity().sub(self.r1.cross(body1.getAngularVelocity())).sub(body2.getLinearVelocity()).add(self.r2.cross(body2.getAngularVelocity())));
        self.total_lambda = self.total_lambda.add(lambda); // Store accumulated lambda
        return self.applyVelocityStep(body1, body2, lambda);
    }

    /// Iteratively update the position constraint. Makes sure C(...) = 0.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
    pub fn solvePositionConstraint(self: *const PointConstraintPart, body1: *Body, body2: *Body, baumgarte: f32) bool {
        const separation = body2.getCenterOfMassPosition().sub(body1.getCenterOfMassPosition()).toVec3().add(self.r2).sub(self.r1);
        if (!separation.eql(Vec3.zero())) {
            // Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
            //
            // lambda = -K^-1 * beta / dt * C
            //
            // We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
            const lambda = self.effective_mass.mulScalar(-baumgarte).mulVec3(separation);

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
            if (body1.isDynamic()) {
                body1.subPositionStep(lambda.mulScalar(body1.getMotionProperties().getInverseMass()));
                body1.subRotationStep(self.inv_i1_r1x.mulVec3(lambda));
            }
            if (body2.isDynamic()) {
                body2.addPositionStep(lambda.mulScalar(body2.getMotionProperties().getInverseMass()));
                body2.addRotationStep(self.inv_i2_r2x.mulVec3(lambda));
            }

            return true;
        }

        return false;
    }

    /// Return lagrange multiplier
    pub fn getTotalLambda(self: *const PointConstraintPart) Vec3 {
        return self.total_lambda;
    }

    /// Save state of this constraint part
    pub fn saveState(self: *const PointConstraintPart, stream: *StateRecorder) void {
        stream.write(self.total_lambda);
    }

    /// Restore state of this constraint part
    pub fn restoreState(self: *PointConstraintPart, stream: *StateRecorder) void {
        stream.read(&self.total_lambda);
    }
};

const TestContext = @import("ConstraintPartTestContext.zig");

/// Relative velocity of the two constraint points (J v)
fn testPointVelocity(body1: *const Body, r1: Vec3, body2: *const Body, r2: Vec3) Vec3 {
    return body1.getLinearVelocity().add(body1.getAngularVelocity().cross(r1)).sub(body2.getLinearVelocity().add(body2.getAngularVelocity().cross(r2)));
}

test "PointConstraintPart for every combination of motion types" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../../StateRecorderImpl.zig").StateRecorderImpl;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const local_r1 = Vec3.init(0.5, 0.25, -0.5);
    const local_r2 = Vec3.init(-0.25, 0.5, 0.75);

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);
        const rotation1 = Mat44.rotationQuat(body1.getRotation());
        const rotation2 = Mat44.rotationQuat(body2.getRotation());

        var part: PointConstraintPart = .{};
        part.calculateConstraintProperties(body1, rotation1, local_r1, body2, rotation2, local_r2);
        const any_dynamic = body1.isDynamic() or body2.isDynamic();
        try testing.expectEqual(any_dynamic, part.isActive());
        if (!any_dynamic) {
            try testing.expect(part.getTotalLambda().eql(Vec3.zero()));
            continue;
        }
        const r1 = rotation1.multiply3x3(local_r1);
        const r2 = rotation2.multiply3x3(local_r2);

        // One iteration removes the relative velocity of the constraint points
        try testing.expect(testPointVelocity(body1, r1, body2, r2).length() > 0.1);
        try testing.expect(part.solveVelocityConstraint(body1, body2));
        try testing.expect(testPointVelocity(body1, r1, body2, r2).isNearZero(.{ .max_dist_sq = 1.0e-9 }));
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);
        try testing.expect(!part.getTotalLambda().eql(Vec3.zero()));

        // Warm start
        const lambda = part.getTotalLambda();
        part.warmStart(body1, body2, 0.5);
        try testing.expect(part.getTotalLambda().eql(lambda.mulScalar(0.5)));
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Position: the bodies are apart, so the separation is not zero and the dynamic bodies move closer
        const p1 = body1.getCenterOfMassPosition().addVec3(r1);
        const p2 = body2.getCenterOfMassPosition().addVec3(r2);
        const separation_before = p2.sub(p1).toVec3().length();
        try testing.expect(part.solvePositionConstraint(body1, body2, 0.5));
        const r1_after = Mat44.rotationQuat(body1.getRotation()).multiply3x3(local_r1);
        const r2_after = Mat44.rotationQuat(body2.getRotation()).multiply3x3(local_r2);
        const separation_after = body2.getCenterOfMassPosition().addVec3(r2_after).sub(body1.getCenterOfMassPosition().addVec3(r1_after)).toVec3().length();
        try testing.expect(separation_after < separation_before);
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // State
        var recorder = StateRecorderImpl.init(testing.allocator);
        defer recorder.deinit();
        part.saveState(recorder.recorder());
        try testing.expectEqual(@as(usize, 12), recorder.getData().len);
        var restored: PointConstraintPart = .{};
        restored.restoreState(recorder.recorder());
        try testing.expect(restored.getTotalLambda().eql(part.getTotalLambda()));

        part.deactivate();
        try testing.expect(!part.isActive());
        try testing.expect(part.getTotalLambda().eql(Vec3.zero()));
    }
}

test "PointConstraintPart deactivates on a singular effective mass" {
    const testing = std.testing;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    // Body 1 can only rotate around Z and the constraint point is its center of mass: K is singular
    const body1 = try context.createBody(.dynamic, .{ .allowed_dofs = .{ .rotation_z = true } });
    const body2 = try context.createBody(.static, .{});
    var part: PointConstraintPart = .{ .total_lambda = Vec3.init(1, 2, 3) };
    part.calculateConstraintProperties(body1, Mat44.identity(), Vec3.zero(), body2, Mat44.identity(), Vec3.zero());
    try testing.expect(!part.isActive());
    try testing.expect(part.getTotalLambda().eql(Vec3.zero()));
}
