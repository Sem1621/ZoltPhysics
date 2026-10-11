//! Port of: Jolt/Physics/Constraints/ConstraintPart/HingeRotationConstraintPart.h
//! Status: complete
//!
//! `Vec2 = Vector<2>` and `Mat22 = Matrix<2, 2>` are `Vector(2)` and `Matrix(2, 2)`; `mat(r, c) = x` is `set(r, c, x)`.

const std = @import("std");
const Core = @import("../../../Core/Core.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Vector = @import("../../../Math/Vector.zig").Vector;
const Matrix = @import("../../../Math/Matrix.zig").Matrix;
const Body = @import("../../Body/Body.zig").Body;
const StateRecorder = @import("../../StateRecorder.zig").StateRecorder;

/// Constrains rotation around 2 axis so that it only allows rotation around 1 axis
///
/// Based on: "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.4.1
///
/// Constraint equation (eq 87):
///
/// \f[C = \begin{bmatrix}a_1 \cdot b_2 \\ a_1 \cdot c_2\end{bmatrix}\f]
///
/// Jacobian (eq 90):
///
/// \f[J = \begin{bmatrix}
/// 0   & -b_2 \times a_1   & 0     & b_2 \times a_1    \\
/// 0   & -c_2 \times a_1   & 0     & c2 \times a_1
/// \end{bmatrix}\f]
///
/// Used terms (here and below, everything in world space):\n
/// a1 = hinge axis on body 1.\n
/// b2, c2 = axis perpendicular to hinge axis on body 2.\n
/// x1, x2 = center of mass for the bodies.\n
/// v = [v1, w1, v2, w2].\n
/// v1, v2 = linear velocity of body 1 and 2.\n
/// w1, w2 = angular velocity of body 1 and 2.\n
/// M = mass matrix, a diagonal matrix of the mass and inertia with diagonal [m1, I1, m2, I2].\n
/// \f$K^{-1} = \left( J M^{-1} J^T \right)^{-1}\f$ = effective mass.\n
/// b = velocity bias.\n
/// \f$\beta\f$ = baumgarte constant.\n
/// E = identity matrix.
pub const HingeRotationConstraintPart = struct {
    pub const Vec2 = Vector(2);
    pub const Mat22 = Matrix(2, 2);

    /// World space hinge axis for body 1
    a1: Vec3 = undefined,
    /// World space perpendiculars of hinge axis for body 2
    b2: Vec3 = undefined,
    c2: Vec3 = undefined,
    inv_i1: Mat44 = undefined,
    inv_i2: Mat44 = undefined,
    b2_x_a1: Vec3 = undefined,
    c2_x_a1: Vec3 = undefined,
    effective_mass: Mat22 = undefined,
    total_lambda: Vec2 = Vec2.zero(),

    /// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
    fn applyVelocityStep(self: *const HingeRotationConstraintPart, body1: *Body, body2: *Body, lambda: Vec2) bool {
        // Apply impulse if delta is not zero
        if (!lambda.isZero()) {
            // Calculate velocity change due to constraint
            //
            // Impulse:
            // P = J^T lambda
            //
            // Euler velocity integration:
            // v' = v + M^-1 P
            const impulse = self.b2_x_a1.mulScalar(lambda.getComponent(0)).add(self.c2_x_a1.mulScalar(lambda.getComponent(1)));
            if (body1.isDynamic())
                body1.getMotionPropertiesMut().subAngularVelocityStep(self.inv_i1.multiply3x3(impulse));
            if (body2.isDynamic())
                body2.getMotionPropertiesMut().addAngularVelocityStep(self.inv_i2.multiply3x3(impulse));
            return true;
        }

        return false;
    }

    /// Calculate properties used during the functions below
    pub fn calculateConstraintProperties(self: *HingeRotationConstraintPart, body1: *const Body, rotation1: Mat44, world_space_hinge_axis1: Vec3, body2: *const Body, rotation2: Mat44, world_space_hinge_axis2: Vec3) void {
        if (Core.enable_asserts) std.debug.assert(world_space_hinge_axis1.isNormalized(.{ .tolerance = 1.0e-5 }));
        if (Core.enable_asserts) std.debug.assert(world_space_hinge_axis2.isNormalized(.{ .tolerance = 1.0e-5 }));

        // Calculate hinge axis in world space
        self.a1 = world_space_hinge_axis1;
        var a2 = world_space_hinge_axis2;
        const dot = self.a1.dot(a2);
        if (dot <= 1.0e-3) {
            // World space axes are more than 90 degrees apart, get a perpendicular vector in the plane formed by mA1 and a2 as hinge axis until the rotation is less than 90 degrees
            var perp = a2.sub(self.a1.mulScalar(dot));
            if (perp.lengthSq() < 1.0e-6) {
                // mA1 ~ -a2, take random perpendicular
                perp = self.a1.getNormalizedPerpendicular();
            }

            // Blend in a little bit from mA1 so we're less than 90 degrees apart
            a2 = perp.normalized().mulScalar(0.99).add(self.a1.mulScalar(0.01)).normalized();
        }
        self.b2 = a2.getNormalizedPerpendicular();
        self.c2 = a2.cross(self.b2);

        // Calculate properties used during constraint solving
        self.inv_i1 = if (body1.isDynamic()) body1.getMotionProperties().getInverseInertiaForRotation(rotation1) else Mat44.zero();
        self.inv_i2 = if (body2.isDynamic()) body2.getMotionProperties().getInverseInertiaForRotation(rotation2) else Mat44.zero();
        self.b2_x_a1 = self.b2.cross(self.a1);
        self.c2_x_a1 = self.c2.cross(self.a1);

        // Calculate effective mass: K^-1 = (J M^-1 J^T)^-1
        const summed_inv_inertia = self.inv_i1.add(self.inv_i2);
        var inv_effective_mass: Mat22 = undefined;
        inv_effective_mass.set(0, 0, self.b2_x_a1.dot(summed_inv_inertia.multiply3x3(self.b2_x_a1)));
        inv_effective_mass.set(0, 1, self.b2_x_a1.dot(summed_inv_inertia.multiply3x3(self.c2_x_a1)));
        inv_effective_mass.set(1, 0, self.c2_x_a1.dot(summed_inv_inertia.multiply3x3(self.b2_x_a1)));
        inv_effective_mass.set(1, 1, self.c2_x_a1.dot(summed_inv_inertia.multiply3x3(self.c2_x_a1)));
        if (!self.effective_mass.setInversed(inv_effective_mass))
            self.deactivate();
    }

    /// Deactivate this constraint
    pub fn deactivate(self: *HingeRotationConstraintPart) void {
        self.effective_mass.setZero();
        self.total_lambda.setZero();
    }

    /// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
    pub fn warmStart(self: *HingeRotationConstraintPart, body1: *Body, body2: *Body, warm_start_impulse_ratio: f32) void {
        self.total_lambda = self.total_lambda.mulScalar(warm_start_impulse_ratio);
        _ = self.applyVelocityStep(body1, body2, self.total_lambda);
    }

    /// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
    pub fn solveVelocityConstraint(self: *HingeRotationConstraintPart, body1: *Body, body2: *Body) bool {
        // Calculate lagrange multiplier:
        //
        // lambda = -K^-1 (J v + b)
        const delta_ang = body1.getAngularVelocity().sub(body2.getAngularVelocity());
        var jv: Vec2 = undefined;
        jv.setComponent(0, self.b2_x_a1.dot(delta_ang));
        jv.setComponent(1, self.c2_x_a1.dot(delta_ang));
        const lambda = self.effective_mass.mulVector(jv);

        // Store accumulated lambda
        self.total_lambda = self.total_lambda.add(lambda);

        return self.applyVelocityStep(body1, body2, lambda);
    }

    /// Iteratively update the position constraint. Makes sure C(...) = 0.
    pub fn solvePositionConstraint(self: *const HingeRotationConstraintPart, body1: *Body, body2: *Body, baumgarte: f32) bool {
        // Constraint needs Axis of body 1 perpendicular to both B and C from body 2 (which are both perpendicular to the Axis of body 2)
        var c: Vec2 = undefined;
        c.setComponent(0, self.a1.dot(self.b2));
        c.setComponent(1, self.a1.dot(self.c2));
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
            const impulse = self.b2_x_a1.mulScalar(lambda.getComponent(0)).add(self.c2_x_a1.mulScalar(lambda.getComponent(1)));
            if (body1.isDynamic())
                body1.subRotationStep(self.inv_i1.multiply3x3(impulse));
            if (body2.isDynamic())
                body2.addRotationStep(self.inv_i2.multiply3x3(impulse));
            return true;
        }

        return false;
    }

    /// Return lagrange multiplier
    pub fn getTotalLambda(self: *const HingeRotationConstraintPart) *const Vec2 {
        return &self.total_lambda;
    }

    /// Save state of this constraint part
    pub fn saveState(self: *const HingeRotationConstraintPart, stream: *StateRecorder) void {
        stream.write(self.total_lambda);
    }

    /// Restore state of this constraint part
    pub fn restoreState(self: *HingeRotationConstraintPart, stream: *StateRecorder) void {
        stream.read(&self.total_lambda);
    }
};

const TestContext = @import("ConstraintPartTestContext.zig");

test "HingeRotationConstraintPart for every combination of motion types" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../../StateRecorderImpl.zig").StateRecorderImpl;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const hinge_axis1 = Vec3.init(0.2, 1, -0.3).normalized();
    const hinge_axis2 = Vec3.init(0.3, 1, -0.2).normalized();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);

        var part: HingeRotationConstraintPart = .{};
        part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), hinge_axis1, body2, Mat44.rotationQuat(body2.getRotation()), hinge_axis2);
        const any_dynamic = body1.isDynamic() or body2.isDynamic();
        try testing.expectEqual(any_dynamic, !part.effective_mass.isZero());
        if (!any_dynamic) {
            try testing.expect(part.getTotalLambda().isZero());
            continue;
        }

        // The relative angular velocity is removed except around the hinge axis
        try testing.expect(part.solveVelocityConstraint(body1, body2));
        const delta_ang = body1.getAngularVelocity().sub(body2.getAngularVelocity());
        try testing.expectApproxEqAbs(@as(f32, 0.0), part.b2_x_a1.dot(delta_ang), 1.0e-5);
        try testing.expectApproxEqAbs(@as(f32, 0.0), part.c2_x_a1.dot(delta_ang), 1.0e-5);
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Warm start
        const lambda = part.getTotalLambda().*;
        part.warmStart(body1, body2, 2.0);
        try testing.expect(part.getTotalLambda().eql(lambda.mulScalar(2.0)));
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Position: the axes are not aligned
        const dynamic_body = if (body1.isDynamic()) body1 else body2;
        const rotation_before = dynamic_body.getRotation();
        try testing.expect(part.solvePositionConstraint(body1, body2, 0.2));
        try testing.expect(!dynamic_body.getRotation().eql(rotation_before));
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // State
        var recorder = StateRecorderImpl.init(testing.allocator);
        defer recorder.deinit();
        part.saveState(recorder.recorder());
        var restored: HingeRotationConstraintPart = .{};
        restored.restoreState(recorder.recorder());
        try testing.expect(restored.getTotalLambda().eql(part.getTotalLambda().*));

        part.deactivate();
        try testing.expect(part.effective_mass.isZero());
        try testing.expect(part.getTotalLambda().isZero());
    }
}

test "HingeRotationConstraintPart opposing and perpendicular axes" {
    const testing = std.testing;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });

    // Aligned axes: no position error
    var aligned: HingeRotationConstraintPart = .{};
    aligned.calculateConstraintProperties(bodies[0], Mat44.identity(), Vec3.axisX(), bodies[1], Mat44.identity(), Vec3.axisX());
    try testing.expect(!aligned.solvePositionConstraint(bodies[0], bodies[1], 0.2));

    // Opposing axes (mA1 ~ -a2): a2 is blended towards a1, so b2 and c2 are not perpendicular to a1
    var opposing: HingeRotationConstraintPart = .{};
    opposing.calculateConstraintProperties(bodies[0], Mat44.identity(), Vec3.axisY(), bodies[1], Mat44.identity(), Vec3.axisY().negate());
    try testing.expect(@abs(opposing.b2.dot(opposing.c2)) < 1.0e-6);
    try testing.expect(!opposing.effective_mass.isZero());
    try testing.expect(opposing.solvePositionConstraint(bodies[0], bodies[1], 0.2));

    // Perpendicular axes
    var perpendicular: HingeRotationConstraintPart = .{};
    perpendicular.calculateConstraintProperties(bodies[0], Mat44.identity(), Vec3.axisZ(), bodies[1], Mat44.identity(), Vec3.axisX());
    try testing.expect(!perpendicular.effective_mass.isZero());
    try testing.expect(perpendicular.solvePositionConstraint(bodies[0], bodies[1], 0.2));
}
