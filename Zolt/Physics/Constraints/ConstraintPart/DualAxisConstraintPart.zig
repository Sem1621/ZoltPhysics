//! Port of: Jolt/Physics/Constraints/ConstraintPart/DualAxisConstraintPart.h
//! Status: complete
//!
//! - The out parameter of the private CalculateLagrangeMultiplier is its return value.
//! - `Vec2 = Vector<2>` and `Mat22 = Matrix<2, 2>` are `Vector(2)` and `Matrix(2, 2)`; `mat(r, c) = x` is `set(r, c, x)`.
//! - `JPH_IF_DEBUG(mInvI1_R1PlusUxN1 = Vec3::sNaN();)` is `if (builtin.mode == .Debug)`.

const std = @import("std");
const builtin = @import("builtin");
const Core = @import("../../../Core/Core.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Vector = @import("../../../Math/Vector.zig").Vector;
const Matrix = @import("../../../Math/Matrix.zig").Matrix;
const Body = @import("../../Body/Body.zig").Body;
const StateRecorder = @import("../../StateRecorder.zig").StateRecorder;

/// Constrains movement on 2 axis
///
/// @see "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.3.1
///
/// Constraint equation (eq 51):
///
/// \f[C = \begin{bmatrix} (p_2 - p_1) \cdot n_1 \\ (p_2 - p_1) \cdot n_2\end{bmatrix}\f]
///
/// Jacobian (transposed) (eq 55):
///
/// \f[J^T = \begin{bmatrix}
/// -n_1                    & -n_2                  \\
/// -(r_1 + u) \times n_1   & -(r_1 + u) \times n_2 \\
/// n_1                     & n_2                   \\
/// r_2 \times n_1          & r_2 \times n_2
/// \end{bmatrix}\f]
///
/// Used terms (here and below, everything in world space):\n
/// n1, n2 = constraint axis (normalized).\n
/// p1, p2 = constraint points.\n
/// r1 = p1 - x1.\n
/// r2 = p2 - x2.\n
/// u = x2 + r2 - x1 - r1 = p2 - p1.\n
/// x1, x2 = center of mass for the bodies.\n
/// v = [v1, w1, v2, w2].\n
/// v1, v2 = linear velocity of body 1 and 2.\n
/// w1, w2 = angular velocity of body 1 and 2.\n
/// M = mass matrix, a diagonal matrix of the mass and inertia with diagonal [m1, I1, m2, I2].\n
/// \f$K^{-1} = \left( J M^{-1} J^T \right)^{-1}\f$ = effective mass.\n
/// b = velocity bias.\n
/// \f$\beta\f$ = baumgarte constant.
pub const DualAxisConstraintPart = struct {
    pub const Vec2 = Vector(2);
    pub const Mat22 = Matrix(2, 2);

    r1_plus_u_x_n1: Vec3 = undefined,
    r1_plus_u_x_n2: Vec3 = undefined,
    r2_x_n1: Vec3 = undefined,
    r2_x_n2: Vec3 = undefined,
    inv_i1_r1_plus_u_x_n1: Vec3 = undefined,
    inv_i1_r1_plus_u_x_n2: Vec3 = undefined,
    inv_i2_r2_x_n1: Vec3 = undefined,
    inv_i2_r2_x_n2: Vec3 = undefined,
    effective_mass: Mat22 = undefined,
    total_lambda: Vec2 = Vec2.zero(),

    /// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
    fn applyVelocityStep(self: *const DualAxisConstraintPart, body1: *Body, body2: *Body, n1: Vec3, n2: Vec3, lambda: Vec2) bool {
        // Apply impulse if delta is not zero
        if (!lambda.isZero()) {
            // Calculate velocity change due to constraint
            //
            // Impulse:
            // P = J^T lambda
            //
            // Euler velocity integration:
            // v' = v + M^-1 P
            const impulse = n1.mulScalar(lambda.getComponent(0)).add(n2.mulScalar(lambda.getComponent(1)));
            if (body1.isDynamic()) {
                const mp1 = body1.getMotionPropertiesMut();
                mp1.subLinearVelocityStep(impulse.mulScalar(mp1.getInverseMass()));
                mp1.subAngularVelocityStep(self.inv_i1_r1_plus_u_x_n1.mulScalar(lambda.getComponent(0)).add(self.inv_i1_r1_plus_u_x_n2.mulScalar(lambda.getComponent(1))));
            }
            if (body2.isDynamic()) {
                const mp2 = body2.getMotionPropertiesMut();
                mp2.addLinearVelocityStep(impulse.mulScalar(mp2.getInverseMass()));
                mp2.addAngularVelocityStep(self.inv_i2_r2_x_n1.mulScalar(lambda.getComponent(0)).add(self.inv_i2_r2_x_n2.mulScalar(lambda.getComponent(1))));
            }
            return true;
        }

        return false;
    }

    /// Internal helper function to calculate the lagrange multiplier
    fn calculateLagrangeMultiplier(self: *const DualAxisConstraintPart, body1: *const Body, body2: *const Body, n1: Vec3, n2: Vec3) Vec2 {
        // Calculate lagrange multiplier:
        //
        // lambda = -K^-1 (J v + b)
        const delta_lin = body1.getLinearVelocity().sub(body2.getLinearVelocity());
        var jv: Vec2 = undefined;
        jv.setComponent(0, n1.dot(delta_lin) + self.r1_plus_u_x_n1.dot(body1.getAngularVelocity()) - self.r2_x_n1.dot(body2.getAngularVelocity()));
        jv.setComponent(1, n2.dot(delta_lin) + self.r1_plus_u_x_n2.dot(body1.getAngularVelocity()) - self.r2_x_n2.dot(body2.getAngularVelocity()));
        return self.effective_mass.mulVector(jv);
    }

    /// Calculate properties used during the functions below
    /// All input vectors are in world space
    pub fn calculateConstraintProperties(self: *DualAxisConstraintPart, body1: *const Body, rotation1: Mat44, r1_plus_u: Vec3, body2: *const Body, rotation2: Mat44, r2: Vec3, n1: Vec3, n2: Vec3) void {
        if (Core.enable_asserts) std.debug.assert(n1.isNormalized(.{ .tolerance = 1.0e-5 }));
        if (Core.enable_asserts) std.debug.assert(n2.isNormalized(.{ .tolerance = 1.0e-5 }));

        // Calculate properties used during constraint solving
        self.r1_plus_u_x_n1 = r1_plus_u.cross(n1);
        self.r1_plus_u_x_n2 = r1_plus_u.cross(n2);
        self.r2_x_n1 = r2.cross(n1);
        self.r2_x_n2 = r2.cross(n2);

        // Calculate effective mass: K^-1 = (J M^-1 J^T)^-1, eq 59
        var inv_effective_mass: Mat22 = undefined;
        if (body1.isDynamic()) {
            const mp1 = body1.getMotionProperties();
            const inv_i1 = mp1.getInverseInertiaForRotation(rotation1);
            self.inv_i1_r1_plus_u_x_n1 = inv_i1.multiply3x3(self.r1_plus_u_x_n1);
            self.inv_i1_r1_plus_u_x_n2 = inv_i1.multiply3x3(self.r1_plus_u_x_n2);

            inv_effective_mass.set(0, 0, mp1.getInverseMass() + self.r1_plus_u_x_n1.dot(self.inv_i1_r1_plus_u_x_n1));
            inv_effective_mass.set(0, 1, self.r1_plus_u_x_n1.dot(self.inv_i1_r1_plus_u_x_n2));
            inv_effective_mass.set(1, 0, self.r1_plus_u_x_n2.dot(self.inv_i1_r1_plus_u_x_n1));
            inv_effective_mass.set(1, 1, mp1.getInverseMass() + self.r1_plus_u_x_n2.dot(self.inv_i1_r1_plus_u_x_n2));
        } else {
            if (builtin.mode == .Debug) self.inv_i1_r1_plus_u_x_n1 = Vec3.nan();
            if (builtin.mode == .Debug) self.inv_i1_r1_plus_u_x_n2 = Vec3.nan();

            inv_effective_mass = Mat22.zero();
        }

        if (body2.isDynamic()) {
            const mp2 = body2.getMotionProperties();
            const inv_i2 = mp2.getInverseInertiaForRotation(rotation2);
            self.inv_i2_r2_x_n1 = inv_i2.multiply3x3(self.r2_x_n1);
            self.inv_i2_r2_x_n2 = inv_i2.multiply3x3(self.r2_x_n2);

            inv_effective_mass.set(0, 0, inv_effective_mass.get(0, 0) + (mp2.getInverseMass() + self.r2_x_n1.dot(self.inv_i2_r2_x_n1)));
            inv_effective_mass.set(0, 1, inv_effective_mass.get(0, 1) + self.r2_x_n1.dot(self.inv_i2_r2_x_n2));
            inv_effective_mass.set(1, 0, inv_effective_mass.get(1, 0) + self.r2_x_n2.dot(self.inv_i2_r2_x_n1));
            inv_effective_mass.set(1, 1, inv_effective_mass.get(1, 1) + (mp2.getInverseMass() + self.r2_x_n2.dot(self.inv_i2_r2_x_n2)));
        } else {
            if (builtin.mode == .Debug) self.inv_i2_r2_x_n1 = Vec3.nan();
            if (builtin.mode == .Debug) self.inv_i2_r2_x_n2 = Vec3.nan();
        }

        if (!self.effective_mass.setInversed(inv_effective_mass))
            self.deactivate();
    }

    /// Deactivate this constraint
    pub fn deactivate(self: *DualAxisConstraintPart) void {
        self.effective_mass.setZero();
        self.total_lambda.setZero();
    }

    /// Check if constraint is active
    pub fn isActive(self: *const DualAxisConstraintPart) bool {
        return !self.effective_mass.isZero();
    }

    /// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
    /// All input vectors are in world space
    pub fn warmStart(self: *DualAxisConstraintPart, body1: *Body, body2: *Body, n1: Vec3, n2: Vec3, warm_start_impulse_ratio: f32) void {
        self.total_lambda = self.total_lambda.mulScalar(warm_start_impulse_ratio);
        _ = self.applyVelocityStep(body1, body2, n1, n2, self.total_lambda);
    }

    /// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
    /// All input vectors are in world space
    pub fn solveVelocityConstraint(self: *DualAxisConstraintPart, body1: *Body, body2: *Body, n1: Vec3, n2: Vec3) bool {
        const lambda = self.calculateLagrangeMultiplier(body1, body2, n1, n2);

        // Store accumulated lambda
        self.total_lambda = self.total_lambda.add(lambda);

        return self.applyVelocityStep(body1, body2, n1, n2, lambda);
    }

    /// Iteratively update the position constraint. Makes sure C(...) = 0.
    /// All input vectors are in world space
    pub fn solvePositionConstraint(self: *const DualAxisConstraintPart, body1: *Body, body2: *Body, u: Vec3, n1: Vec3, n2: Vec3, baumgarte: f32) bool {
        var c: Vec2 = undefined;
        c.setComponent(0, u.dot(n1));
        c.setComponent(1, u.dot(n2));
        if (!c.isZero()) {
            // Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
            //
            // lambda = -K^-1 * beta / dt * C
            //
            // We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
            const lambda = self.effective_mass.mulVector(c).mulScalar(-baumgarte);

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
            const impulse = n1.mulScalar(lambda.getComponent(0)).add(n2.mulScalar(lambda.getComponent(1)));
            if (body1.isDynamic()) {
                body1.subPositionStep(impulse.mulScalar(body1.getMotionProperties().getInverseMass()));
                body1.subRotationStep(self.inv_i1_r1_plus_u_x_n1.mulScalar(lambda.getComponent(0)).add(self.inv_i1_r1_plus_u_x_n2.mulScalar(lambda.getComponent(1))));
            }
            if (body2.isDynamic()) {
                body2.addPositionStep(impulse.mulScalar(body2.getMotionProperties().getInverseMass()));
                body2.addRotationStep(self.inv_i2_r2_x_n1.mulScalar(lambda.getComponent(0)).add(self.inv_i2_r2_x_n2.mulScalar(lambda.getComponent(1))));
            }
            return true;
        }

        return false;
    }

    /// Override total lagrange multiplier, can be used to set the initial value for warm starting
    pub fn setTotalLambda(self: *DualAxisConstraintPart, lambda: Vec2) void {
        self.total_lambda = lambda;
    }

    /// Return lagrange multiplier
    pub fn getTotalLambda(self: *const DualAxisConstraintPart) *const Vec2 {
        return &self.total_lambda;
    }

    /// Save state of this constraint part
    pub fn saveState(self: *const DualAxisConstraintPart, stream: *StateRecorder) void {
        stream.write(self.total_lambda);
    }

    /// Restore state of this constraint part
    pub fn restoreState(self: *DualAxisConstraintPart, stream: *StateRecorder) void {
        stream.read(&self.total_lambda);
    }
};

const TestContext = @import("ConstraintPartTestContext.zig");

fn testJacobianTimesVelocity(body1: *const Body, r1_plus_u: Vec3, body2: *const Body, r2: Vec3, n: Vec3) f32 {
    return n.dot(body1.getLinearVelocity().sub(body2.getLinearVelocity())) + r1_plus_u.cross(n).dot(body1.getAngularVelocity()) - r2.cross(n).dot(body2.getAngularVelocity());
}

test "DualAxisConstraintPart for every combination of motion types" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../../StateRecorderImpl.zig").StateRecorderImpl;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const r1_plus_u = Vec3.init(0.3, -0.5, 0.8);
    const r2 = Vec3.init(-0.4, 0.6, 0.2);
    const n1 = Vec3.init(1, 2, -0.5).normalized();
    const n2 = n1.getNormalizedPerpendicular();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);

        var part: DualAxisConstraintPart = .{};
        part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), r1_plus_u, body2, Mat44.rotationQuat(body2.getRotation()), r2, n1, n2);
        const any_dynamic = body1.isDynamic() or body2.isDynamic();
        try testing.expectEqual(any_dynamic, part.isActive());
        if (!any_dynamic) {
            try testing.expect(part.getTotalLambda().isZero());
            continue;
        }

        // Solving removes the relative velocity along both axis in one iteration
        try testing.expect(@abs(testJacobianTimesVelocity(body1, r1_plus_u, body2, r2, n1)) > 0.01);
        try testing.expect(part.solveVelocityConstraint(body1, body2, n1, n2));
        try testing.expectApproxEqAbs(@as(f32, 0.0), testJacobianTimesVelocity(body1, r1_plus_u, body2, r2, n1), 1.0e-5);
        try testing.expectApproxEqAbs(@as(f32, 0.0), testJacobianTimesVelocity(body1, r1_plus_u, body2, r2, n2), 1.0e-5);
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Warm start
        const lambda = part.getTotalLambda().*;
        part.warmStart(body1, body2, n1, n2, 0.5);
        try testing.expect(part.getTotalLambda().eql(lambda.mulScalar(0.5)));
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Position
        try testing.expect(!part.solvePositionConstraint(body1, body2, Vec3.zero(), n1, n2, 0.2));
        const dynamic_body = if (body1.isDynamic()) body1 else body2;
        const position_before = dynamic_body.getCenterOfMassPosition();
        try testing.expect(part.solvePositionConstraint(body1, body2, n1.mulScalar(0.1), n1, n2, 0.2));
        try testing.expect(!dynamic_body.getCenterOfMassPosition().eql(position_before));
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // State
        var recorder = StateRecorderImpl.init(testing.allocator);
        defer recorder.deinit();
        part.saveState(recorder.recorder());
        try testing.expectEqual(@as(usize, 8), recorder.getData().len);
        var restored: DualAxisConstraintPart = .{};
        restored.restoreState(recorder.recorder());
        try testing.expect(restored.getTotalLambda().eql(part.getTotalLambda().*));

        restored.setTotalLambda(DualAxisConstraintPart.Vec2.zero());
        try testing.expect(restored.getTotalLambda().isZero());

        part.deactivate();
        try testing.expect(!part.isActive());
        try testing.expect(part.getTotalLambda().isZero());
    }
}

test "DualAxisConstraintPart deactivates on a singular effective mass" {
    const testing = std.testing;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    // Body 1 can only rotate around X (infinite mass) and the constraint acts at its center of mass: K is zero
    const body1 = try context.createBody(.dynamic, .{ .allowed_dofs = .{ .rotation_x = true } });
    const body2 = try context.createBody(.static, .{});
    var part: DualAxisConstraintPart = .{};
    part.setTotalLambda(DualAxisConstraintPart.Vec2{ .f32s = .{ 1, 2 } });
    part.calculateConstraintProperties(body1, Mat44.identity(), Vec3.zero(), body2, Mat44.identity(), Vec3.zero(), Vec3.axisY(), Vec3.axisZ());
    try testing.expect(!part.isActive());
    try testing.expect(part.getTotalLambda().isZero());
}
