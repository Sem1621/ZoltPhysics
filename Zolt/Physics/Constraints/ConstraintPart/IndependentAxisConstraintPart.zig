//! Port of: Jolt/Physics/Constraints/ConstraintPart/IndependentAxisConstraintPart.h
//! Status: complete
//!
//! Like Jolt, CalculateConstraintProperties uses the motion properties of every body that is not static, so a kinematic
//! body trips the dynamic body assert of `getInverseMass` when asserts are on (Jolt's JPH_ASSERT in GetInverseMass);
//! without asserts its inverse mass and inertia count like Jolt's release build.
//!
//! Zolt (defined where Jolt is undefined): CalculateConstraintProperties does not write mR1xN1 when body 1 is static and
//! mRatioR2xN2 when body 2 is static, but SolveVelocityConstraint multiplies them with the angular velocity of their
//! body (zero for a static body). Jolt leaves the members uninitialized, so a NaN or an infinity in that memory makes the
//! velocity of the other body NaN. Zolt's members start at zero: the product is 0, like Jolt's with the zero-filled
//! allocations of the parity reference. Pinned by "IndependentAxisConstraintPart: the members of a static body are zero".

const std = @import("std");
const Core = @import("../../../Core/Core.zig");
const math = @import("../../../Math/Math.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Body = @import("../../Body/Body.zig").Body;
const StateRecorder = @import("../../StateRecorder.zig").StateRecorder;

/// Constraint part to an AxisConstraintPart but both bodies have an independent axis on which the force is applied.
///
/// Constraint equation:
///
/// \f[C = (x_1 + r_1 - f_1) . n_1 + r (x_2 + r_2 - f_2) \cdot n_2\f]
///
/// Calculating the Jacobian:
///
/// \f[dC/dt = (v_1 + w_1 \times r_1) \cdot n_1 + (x_1 + r_1 - f_1) \cdot d n_1/dt + r (v_2 + w_2 \times r_2) \cdot n_2 + r (x_2 + r_2 - f_2) \cdot d n_2/dt\f]
///
/// Assuming that d n1/dt and d n2/dt are small this becomes:
///
/// \f[(v_1 + w_1 \times r_1) \cdot n_1 + r (v_2 + w_2 \times r_2) \cdot n_2\f]
/// \f[= v_1 \cdot n_1 + r_1 \times n_1 \cdot w_1 + r v_2 \cdot n_2 + r r_2 \times n_2 \cdot w_2\f]
///
/// Jacobian:
///
/// \f[J = \begin{bmatrix}n_1 & r_1 \times n_1 & r n_2 & r r_2 \times n_2\end{bmatrix}\f]
///
/// Effective mass:
///
/// \f[K = m_1^{-1} + r_1 \times n_1 I_1^{-1} r_1 \times n_1 + r^2 m_2^{-1} + r^2 r_2 \times n_2 I_2^{-1} r_2 \times n_2\f]
///
/// Used terms (here and below, everything in world space):\n
/// n1 = (x1 + r1 - f1) / |x1 + r1 - f1|, axis along which the force is applied for body 1\n
/// n2 = (x2 + r2 - f2) / |x2 + r2 - f2|, axis along which the force is applied for body 2\n
/// r = ratio how forces are applied between bodies.\n
/// x1, x2 = center of mass for the bodies.\n
/// v = [v1, w1, v2, w2].\n
/// v1, v2 = linear velocity of body 1 and 2.\n
/// w1, w2 = angular velocity of body 1 and 2.\n
/// M = mass matrix, a diagonal matrix of the mass and inertia with diagonal [m1, I1, m2, I2].\n
/// \f$K^{-1} = \left( J M^{-1} J^T \right)^{-1}\f$ = effective mass.\n
/// b = velocity bias.\n
/// \f$\beta\f$ = baumgarte constant.
pub const IndependentAxisConstraintPart = struct {
    // Zolt: zero, Jolt leaves them uninitialized (see the file comment)
    r1_x_n1: Vec3 = .zero(),
    inv_i1_r1_x_n1: Vec3 = .zero(),
    ratio_r2_x_n2: Vec3 = .zero(),
    inv_i2_ratio_r2_x_n2: Vec3 = .zero(),
    effective_mass: f32 = 0.0,
    total_lambda: f32 = 0.0,

    /// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
    fn applyVelocityStep(self: *const IndependentAxisConstraintPart, body1: *Body, body2: *Body, n1: Vec3, n2: Vec3, ratio: f32, lambda: f32) bool {
        // Apply impulse if delta is not zero
        if (lambda != 0.0) {
            // Calculate velocity change due to constraint
            //
            // Impulse:
            // P = J^T lambda
            //
            // Euler velocity integration:
            // v' = v + M^-1 P
            if (body1.isDynamic()) {
                const mp1 = body1.getMotionPropertiesMut();
                mp1.addLinearVelocityStep(n1.mulScalar(mp1.getInverseMass() * lambda));
                mp1.addAngularVelocityStep(self.inv_i1_r1_x_n1.mulScalar(lambda));
            }
            if (body2.isDynamic()) {
                const mp2 = body2.getMotionPropertiesMut();
                mp2.addLinearVelocityStep(n2.mulScalar(ratio * mp2.getInverseMass() * lambda));
                mp2.addAngularVelocityStep(self.inv_i2_ratio_r2_x_n2.mulScalar(lambda));
            }
            return true;
        }

        return false;
    }

    /// Calculate properties used during the functions below
    /// @param inBody1 The first body that this constraint is attached to
    /// @param inBody2 The second body that this constraint is attached to
    /// @param inR1 The position on which the constraint operates on body 1 relative to COM
    /// @param inN1 The world space normal in which the constraint operates for body 1
    /// @param inR2 The position on which the constraint operates on body 1 relative to COM
    /// @param inN2 The world space normal in which the constraint operates for body 2
    /// @param inRatio The ratio how forces are applied between bodies
    pub fn calculateConstraintProperties(self: *IndependentAxisConstraintPart, body1: *const Body, body2: *const Body, r1: Vec3, n1: Vec3, r2: Vec3, n2: Vec3, ratio: f32) void {
        if (Core.enable_asserts) std.debug.assert(n1.isNormalized(.{ .tolerance = 1.0e-4 }) and n2.isNormalized(.{ .tolerance = 1.0e-4 }));

        var inv_effective_mass: f32 = 0.0;

        if (!body1.isStatic()) {
            const mp1 = body1.getMotionProperties();

            self.r1_x_n1 = r1.cross(n1);
            self.inv_i1_r1_x_n1 = mp1.multiplyWorldSpaceInverseInertiaByVector(body1.getRotation(), self.r1_x_n1);

            inv_effective_mass += mp1.getInverseMass() + self.inv_i1_r1_x_n1.dot(self.r1_x_n1);
        }

        if (!body2.isStatic()) {
            const mp2 = body2.getMotionProperties();

            self.ratio_r2_x_n2 = r2.cross(n2).mulScalar(ratio);
            self.inv_i2_ratio_r2_x_n2 = mp2.multiplyWorldSpaceInverseInertiaByVector(body2.getRotation(), self.ratio_r2_x_n2);

            inv_effective_mass += math.square(ratio) * mp2.getInverseMass() + self.inv_i2_ratio_r2_x_n2.dot(self.ratio_r2_x_n2);
        }

        // Calculate inverse effective mass: K = J M^-1 J^T
        if (inv_effective_mass < math.flt_min)
            self.deactivate()
        else
            self.effective_mass = 1.0 / inv_effective_mass;
    }

    /// Deactivate this constraint
    pub fn deactivate(self: *IndependentAxisConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    /// Check if constraint is active
    pub fn isActive(self: *const IndependentAxisConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    /// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inN1 The world space normal in which the constraint operates for body 1
    /// @param inN2 The world space normal in which the constraint operates for body 2
    /// @param inRatio The ratio how forces are applied between bodies
    /// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
    pub fn warmStart(self: *IndependentAxisConstraintPart, body1: *Body, body2: *Body, n1: Vec3, n2: Vec3, ratio: f32, warm_start_impulse_ratio: f32) void {
        self.total_lambda *= warm_start_impulse_ratio;
        _ = self.applyVelocityStep(body1, body2, n1, n2, ratio, self.total_lambda);
    }

    /// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inN1 The world space normal in which the constraint operates for body 1
    /// @param inN2 The world space normal in which the constraint operates for body 2
    /// @param inRatio The ratio how forces are applied between bodies
    /// @param inMinLambda Minimum angular impulse to apply (N m s)
    /// @param inMaxLambda Maximum angular impulse to apply (N m s)
    pub fn solveVelocityConstraint(self: *IndependentAxisConstraintPart, body1: *Body, body2: *Body, n1: Vec3, n2: Vec3, ratio: f32, min_lambda: f32, max_lambda: f32) bool {
        // Lagrange multiplier is:
        //
        // lambda = -K^-1 (J v + b)
        var lambda = -self.effective_mass * (n1.dot(body1.getLinearVelocity()) + self.r1_x_n1.dot(body1.getAngularVelocity()) + ratio * n2.dot(body2.getLinearVelocity()) + self.ratio_r2_x_n2.dot(body2.getAngularVelocity()));
        const new_lambda = math.clamp(self.total_lambda + lambda, min_lambda, max_lambda); // Clamp impulse
        lambda = new_lambda - self.total_lambda; // Lambda potentially got clamped, calculate the new impulse to apply
        self.total_lambda = new_lambda; // Store accumulated impulse

        return self.applyVelocityStep(body1, body2, n1, n2, ratio, lambda);
    }

    /// Return lagrange multiplier
    pub fn getTotalLambda(self: *const IndependentAxisConstraintPart) f32 {
        return self.total_lambda;
    }

    /// Iteratively update the position constraint. Makes sure C(...) == 0.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inN1 The world space normal in which the constraint operates for body 1
    /// @param inN2 The world space normal in which the constraint operates for body 2
    /// @param inRatio The ratio how forces are applied between bodies
    /// @param inC Value of the constraint equation (C)
    /// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
    pub fn solvePositionConstraint(self: *const IndependentAxisConstraintPart, body1: *Body, body2: *Body, n1: Vec3, n2: Vec3, ratio: f32, c: f32, baumgarte: f32) bool {
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
            if (body1.isDynamic()) {
                body1.addPositionStep(n1.mulScalar(lambda * body1.getMotionPropertiesUnchecked().?.getInverseMass()));
                body1.addRotationStep(self.inv_i1_r1_x_n1.mulScalar(lambda));
            }
            if (body2.isDynamic()) {
                body2.addPositionStep(n2.mulScalar(lambda * ratio * body2.getMotionPropertiesUnchecked().?.getInverseMass()));
                body2.addRotationStep(self.inv_i2_ratio_r2_x_n2.mulScalar(lambda));
            }
            return true;
        }

        return false;
    }

    /// Save state of this constraint part
    pub fn saveState(self: *const IndependentAxisConstraintPart, stream: *StateRecorder) void {
        stream.write(self.total_lambda);
    }

    /// Restore state of this constraint part
    pub fn restoreState(self: *IndependentAxisConstraintPart, stream: *StateRecorder) void {
        stream.read(&self.total_lambda);
    }
};

const TestContext = @import("ConstraintPartTestContext.zig");

test "IndependentAxisConstraintPart for every combination of motion types" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../../StateRecorderImpl.zig").StateRecorderImpl;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const r1 = Vec3.init(0.3, -0.5, 0.8);
    const r2 = Vec3.init(-0.4, 0.6, 0.2);
    const n1 = Vec3.init(1, 2, -0.5).normalized();
    const n2 = Vec3.init(-0.5, 1, 1).normalized();
    const ratio: f32 = 1.5;

    for (TestContext.motion_type_pairs) |pair| {
        // A kinematic body trips Jolt's dynamic body assert in GetInverseMass (without asserts its mass counts in the
        // effective mass while the body does not move, so one iteration does not remove J v)
        if (pair[0] == .kinematic or pair[1] == .kinematic)
            continue;

        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);

        // The members that a static body leaves unset start at zero: they are multiplied with its zero velocity
        var part: IndependentAxisConstraintPart = .{};
        part.calculateConstraintProperties(body1, body2, r1, n1, r2, n2, ratio);
        const any_moving = !body1.isStatic() or !body2.isStatic();
        try testing.expectEqual(any_moving, part.isActive());
        if (!any_moving)
            continue;

        // One iteration without limits removes J v
        const jvFn = struct {
            fn get(b1: *const Body, b2: *const Body, p: *const IndependentAxisConstraintPart, a1: Vec3, a2: Vec3, r: f32) f32 {
                return a1.dot(b1.getLinearVelocity()) + p.r1_x_n1.dot(b1.getAngularVelocity()) + r * a2.dot(b2.getLinearVelocity()) + p.ratio_r2_x_n2.dot(b2.getAngularVelocity());
            }
        }.get;
        try testing.expect(@abs(jvFn(body1, body2, &part, n1, n2, ratio)) > 0.1);
        try testing.expect(part.solveVelocityConstraint(body1, body2, n1, n2, ratio, -math.flt_max, math.flt_max));
        try testing.expectApproxEqAbs(@as(f32, 0.0), jvFn(body1, body2, &part, n1, n2, ratio), 1.0e-5);
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Limits
        const total = part.getTotalLambda();
        try testing.expect(!part.solveVelocityConstraint(body1, body2, n1, n2, ratio, total, total));

        // Warm start
        part.warmStart(body1, body2, n1, n2, ratio, 0.5);
        try testing.expectEqual(total * 0.5, part.getTotalLambda());

        // Position
        try testing.expect(!part.solvePositionConstraint(body1, body2, n1, n2, ratio, 0.0, 0.2));
        const dynamic_body = if (body1.isDynamic()) body1 else body2;
        const position_before = dynamic_body.getCenterOfMassPosition();
        try testing.expect(part.solvePositionConstraint(body1, body2, n1, n2, ratio, 0.3, 0.2));
        try testing.expect(!dynamic_body.getCenterOfMassPosition().eql(position_before));
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // State
        var recorder = StateRecorderImpl.init(testing.allocator);
        defer recorder.deinit();
        part.saveState(recorder.recorder());
        var restored: IndependentAxisConstraintPart = .{};
        restored.restoreState(recorder.recorder());
        try testing.expectEqual(part.getTotalLambda(), restored.getTotalLambda());

        part.deactivate();
        try testing.expect(!part.isActive());
        try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
    }
}

test "IndependentAxisConstraintPart: the members of a static body are zero" {
    const testing = std.testing;

    // Jolt leaves them uninitialized. Checked at compile time: an undefined default does not compile here.
    comptime {
        const fresh: IndependentAxisConstraintPart = .{};
        for ([_]Vec3{ fresh.r1_x_n1, fresh.inv_i1_r1_x_n1, fresh.ratio_r2_x_n2, fresh.inv_i2_ratio_r2_x_n2 }) |v| {
            if (!v.eql(Vec3.zero())) @compileError("IndependentAxisConstraintPart: a member of a static body does not start at zero");
        }
    }

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const r1 = Vec3.init(0.3, -0.5, 0.8);
    const r2 = Vec3.init(-0.4, 0.6, 0.2);
    const n1 = Vec3.init(1, 2, -0.5).normalized();
    const n2 = Vec3.init(-0.5, 1, 1).normalized();
    for ([_]TestContext.MotionTypePair{ .{ .dynamic, .static }, .{ .static, .dynamic } }) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const dynamic_body = if (bodies[0].isDynamic()) bodies[0] else bodies[1];

        // A pulley between a dynamic and a static body: the solve reads the unset members of the static body
        var part: IndependentAxisConstraintPart = .{};
        part.calculateConstraintProperties(bodies[0], bodies[1], r1, n1, r2, n2, 1.5);
        try testing.expect(part.isActive());
        if (pair[1] == .static) {
            try testing.expect(part.ratio_r2_x_n2.eql(Vec3.zero()) and part.inv_i2_ratio_r2_x_n2.eql(Vec3.zero()));
        } else {
            try testing.expect(part.r1_x_n1.eql(Vec3.zero()) and part.inv_i1_r1_x_n1.eql(Vec3.zero()));
        }
        try testing.expect(part.solveVelocityConstraint(bodies[0], bodies[1], n1, n2, 1.5, -math.flt_max, math.flt_max));
        try testing.expect(!dynamic_body.getLinearVelocity().isNaN() and !dynamic_body.getAngularVelocity().isNaN());
    }
}
