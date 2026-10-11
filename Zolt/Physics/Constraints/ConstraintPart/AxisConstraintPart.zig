//! Port of: Jolt/Physics/Constraints/ConstraintPart/AxisConstraintPart.h
//! Status: complete
//!
//! - The defaulted `inBias = 0.0f` of CalculateConstraintProperties is an options struct (`.{ .bias = b }`).
//! - `const Body &` parameters are `*const Body`, `Body &ioBody` parameters are `*Body`: the solver only writes a body
//!   after checking `isDynamic()`, so a static body (also `Body.fixedToWorld()`) is never written.
//! - `JPH_IF_DEBUG(Vec3::sNaN().StoreFloat3(...))` is `if (builtin.mode == .Debug)`.

const std = @import("std");
const builtin = @import("builtin");
const Core = @import("../../../Core/Core.zig");
const math = @import("../../../Math/Math.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Body = @import("../../Body/Body.zig").Body;
const SpringPart = @import("SpringPart.zig").SpringPart;
const SpringSettings = @import("../SpringSettings.zig").SpringSettings;
const StateRecorder = @import("../../StateRecorder.zig").StateRecorder;

/// Constraint that constrains motion along 1 axis
///
/// @see "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, section 2.1.1
/// (we're not using the approximation of eq 27 but instead add the U term as in eq 55)
///
/// Constraint equation (eq 25):
///
/// \f[C = (p_2 - p_1) \cdot n\f]
///
/// Jacobian (eq 28):
///
/// \f[J = \begin{bmatrix} -n^T & (-(r_1 + u) \times n)^T & n^T & (r_2 \times n)^T \end{bmatrix}\f]
///
/// Used terms (here and below, everything in world space):\n
/// n = constraint axis (normalized).\n
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
pub const AxisConstraintPart = struct {
    r1_plus_u_x_axis: Float3 = undefined,
    r2_x_axis: Float3 = undefined,
    inv_i1_r1_plus_u_x_axis: Float3 = undefined,
    inv_i2_r2_x_axis: Float3 = undefined,
    effective_mass: f32 = 0.0,
    spring_part: SpringPart = .{},
    total_lambda: f32 = 0.0,

    /// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
    fn applyVelocityStep(self: *const AxisConstraintPart, body1: *Body, body2: *Body, world_space_axis: Vec3, lambda: f32) bool {
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
                const mp1 = body1.getMotionPropertiesUncheckedMut().?;
                mp1.subLinearVelocityStep(world_space_axis.mulScalar(lambda * mp1.getInverseMass()));
                mp1.subAngularVelocityStep(Vec3.loadFloat3Unsafe(&self.inv_i1_r1_plus_u_x_axis).mulScalar(lambda));
            }
            if (body2.isDynamic()) {
                const mp2 = body2.getMotionPropertiesUncheckedMut().?;
                mp2.addLinearVelocityStep(world_space_axis.mulScalar(lambda * mp2.getInverseMass()));
                mp2.addAngularVelocityStep(Vec3.loadFloat3Unsafe(&self.inv_i2_r2_x_axis).mulScalar(lambda));
            }
            return true;
        }

        return false;
    }

    /// Internal helper function to calculate the inverse effective mass
    fn calculateInverseEffectiveMass(self: *AxisConstraintPart, body1: *const Body, r1_plus_u: Vec3, body2: *const Body, r2: Vec3, world_space_axis: Vec3) f32 {
        if (Core.enable_asserts) std.debug.assert(world_space_axis.isNormalized(.{ .tolerance = 1.0e-5 }));

        // Calculate inverse effective mass: K = J M^-1 J^T
        var inv_effective_mass: f32 = undefined;

        if (!body1.isStatic()) {
            const r1_plus_u_x_axis = r1_plus_u.cross(world_space_axis);
            r1_plus_u_x_axis.storeFloat3(&self.r1_plus_u_x_axis);

            if (body1.isDynamic()) {
                const mp1 = body1.getMotionPropertiesUnchecked().?;
                const invi1_r1_plus_u_x_axis = mp1.multiplyWorldSpaceInverseInertiaByVector(body1.getRotation(), r1_plus_u_x_axis);
                invi1_r1_plus_u_x_axis.storeFloat3(&self.inv_i1_r1_plus_u_x_axis);

                inv_effective_mass = mp1.getInverseMass() + invi1_r1_plus_u_x_axis.dot(r1_plus_u_x_axis);
            } else {
                if (builtin.mode == .Debug) Vec3.nan().storeFloat3(&self.inv_i1_r1_plus_u_x_axis);

                inv_effective_mass = 0.0;
            }
        } else {
            if (builtin.mode == .Debug) Vec3.nan().storeFloat3(&self.r1_plus_u_x_axis);
            if (builtin.mode == .Debug) Vec3.nan().storeFloat3(&self.inv_i1_r1_plus_u_x_axis);

            inv_effective_mass = 0.0;
        }

        if (!body2.isStatic()) {
            const r2_x_axis = r2.cross(world_space_axis);
            r2_x_axis.storeFloat3(&self.r2_x_axis);

            if (body2.isDynamic()) {
                const mp2 = body2.getMotionPropertiesUnchecked().?;
                const invi2_r2_x_axis = mp2.multiplyWorldSpaceInverseInertiaByVector(body2.getRotation(), r2_x_axis);
                invi2_r2_x_axis.storeFloat3(&self.inv_i2_r2_x_axis);

                inv_effective_mass += mp2.getInverseMass() + invi2_r2_x_axis.dot(r2_x_axis);
            } else {
                if (builtin.mode == .Debug) Vec3.nan().storeFloat3(&self.inv_i2_r2_x_axis);
            }
        } else {
            if (builtin.mode == .Debug) Vec3.nan().storeFloat3(&self.r2_x_axis);
            if (builtin.mode == .Debug) Vec3.nan().storeFloat3(&self.inv_i2_r2_x_axis);
        }

        return inv_effective_mass;
    }

    /// Calculate properties used during the functions below. Creates a constraint without spring.
    /// @param inBody1 The first body that this constraint is attached to
    /// @param inBody2 The second body that this constraint is attached to
    /// @param inR1PlusU See equations above (r1 + u)
    /// @param inR2 See equations above (r2)
    /// @param inWorldSpaceAxis Axis along which the constraint acts (normalized, pointing from body 1 to 2)
    /// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
    pub fn calculateConstraintProperties(self: *AxisConstraintPart, body1: *const Body, r1_plus_u: Vec3, body2: *const Body, r2: Vec3, world_space_axis: Vec3, opts: struct { bias: f32 = 0.0 }) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, r1_plus_u, body2, r2, world_space_axis);

        if (inv_effective_mass < math.flt_min)
            self.deactivate()
        else {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring_part.calculateSpringPropertiesWithBias(opts.bias);
        }
    }

    /// Calculate properties used during the functions below. Set inFrequency to zero if you don't want to drive using a spring.
    /// @param inDeltaTime Time step
    /// @param inBody1 The first body that this constraint is attached to
    /// @param inBody2 The second body that this constraint is attached to
    /// @param inR1PlusU See equations above (r1 + u)
    /// @param inR2 See equations above (r2)
    /// @param inWorldSpaceAxis Axis along which the constraint acts (normalized, pointing from body 1 to 2)
    /// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
    /// @param inC Value of the constraint equation (C).
    /// @param inFrequency Oscillation frequency (Hz).
    /// @param inDamping Damping factor (0 = no damping, 1 = critical damping).
    pub fn calculateConstraintPropertiesWithFrequencyAndDamping(self: *AxisConstraintPart, delta_time: f32, body1: *const Body, r1_plus_u: Vec3, body2: *const Body, r2: Vec3, world_space_axis: Vec3, bias: f32, c: f32, frequency: f32, damping: f32) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, r1_plus_u, body2, r2, world_space_axis);

        if (inv_effective_mass < math.flt_min)
            self.deactivate()
        else if (frequency > 0.0)
            self.effective_mass = self.spring_part.calculateSpringPropertiesWithFrequencyAndDamping(delta_time, inv_effective_mass, bias, c, frequency, damping)
        else {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring_part.calculateSpringPropertiesWithBias(bias);
        }
    }

    /// Calculate properties used during the functions below. Set inFrequency and inDamping to zero if you don't want to drive using a spring.
    /// @param inDeltaTime Time step
    /// @param inBody1 The first body that this constraint is attached to
    /// @param inBody2 The second body that this constraint is attached to
    /// @param inR1PlusU See equations above (r1 + u)
    /// @param inR2 See equations above (r2)
    /// @param inWorldSpaceAxis Axis along which the constraint acts (normalized, pointing from body 1 to 2)
    /// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
    /// @param inC Value of the constraint equation (C).
    /// @param inStiffness Spring stiffness k.
    /// @param inDamping Spring damping coefficient c.
    pub fn calculateConstraintPropertiesWithStiffnessAndDamping(self: *AxisConstraintPart, delta_time: f32, body1: *const Body, r1_plus_u: Vec3, body2: *const Body, r2: Vec3, world_space_axis: Vec3, bias: f32, c: f32, stiffness: f32, damping: f32) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, r1_plus_u, body2, r2, world_space_axis);

        if (inv_effective_mass < math.flt_min)
            self.deactivate()
        else if (stiffness > 0.0 or damping > 0.0)
            self.effective_mass = self.spring_part.calculateSpringPropertiesWithStiffnessAndDamping(delta_time, inv_effective_mass, bias, c, stiffness, damping)
        else {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring_part.calculateSpringPropertiesWithBias(bias);
        }
    }

    /// Calculate properties used during the functions below based on inSpringSettings.
    /// Turns to a hard limit when inSpringSettings has stiffness / frequency = 0
    pub fn calculateConstraintPropertiesWithSettingsForLimit(self: *AxisConstraintPart, delta_time: f32, body1: *const Body, r1_plus_u: Vec3, body2: *const Body, r2: Vec3, world_space_axis: Vec3, bias: f32, c: f32, spring_settings: *const SpringSettings) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, r1_plus_u, body2, r2, world_space_axis);

        if (inv_effective_mass < math.flt_min)
            self.deactivate()
        else if (!spring_settings.hasStiffness()) {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring_part.calculateSpringPropertiesWithBias(bias);
        } else self.effective_mass = self.spring_part.calculateSpringPropertiesWithSettings(delta_time, inv_effective_mass, bias, c, spring_settings);
    }

    /// Calculate properties used during the functions below based on inSpringSettings.
    /// Assumes the spring has either stiffness or damping.
    pub fn calculateConstraintPropertiesWithSettingsForMotor(self: *AxisConstraintPart, delta_time: f32, body1: *const Body, r1_plus_u: Vec3, body2: *const Body, r2: Vec3, world_space_axis: Vec3, bias: f32, c: f32, spring_settings: *const SpringSettings) void {
        if (Core.enable_asserts) std.debug.assert(spring_settings.hasStiffnessOrDamping());

        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, r1_plus_u, body2, r2, world_space_axis);

        if (inv_effective_mass < math.flt_min)
            self.deactivate()
        else
            self.effective_mass = self.spring_part.calculateSpringPropertiesWithSettings(delta_time, inv_effective_mass, bias, c, spring_settings);
    }

    /// Deactivate this constraint
    pub fn deactivate(self: *AxisConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    /// Check if constraint is active
    pub fn isActive(self: *const AxisConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    /// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inWorldSpaceAxis Axis along which the constraint acts (normalized)
    /// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
    pub fn warmStart(self: *AxisConstraintPart, body1: *Body, body2: *Body, world_space_axis: Vec3, warm_start_impulse_ratio: f32) void {
        self.total_lambda *= warm_start_impulse_ratio;

        _ = self.applyVelocityStep(body1, body2, world_space_axis, self.total_lambda);
    }

    /// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inWorldSpaceAxis Axis along which the constraint acts (normalized)
    /// @param inMinLambda Minimum value of constraint impulse to apply (N s)
    /// @param inMaxLambda Maximum value of constraint impulse to apply (N s)
    pub fn solveVelocityConstraint(self: *AxisConstraintPart, body1: *Body, body2: *Body, world_space_axis: Vec3, min_lambda: f32, max_lambda: f32) bool {
        const mp1 = body1.getMotionPropertiesUnchecked();
        const mp2 = body2.getMotionPropertiesUnchecked();

        // Calculate jacobian multiplied by linear velocity
        var jv: f32 = undefined;
        if (!body1.isStatic()) {
            if (!body2.isStatic())
                jv = world_space_axis.dot(mp1.?.getLinearVelocity().sub(mp2.?.getLinearVelocity()))
            else
                jv = world_space_axis.dot(mp1.?.getLinearVelocity());
        } else {
            std.debug.assert(!body2.isStatic());
            jv = world_space_axis.dot(mp2.?.getLinearVelocity().negate());
        }

        // Calculate jacobian multiplied by angular velocity
        if (!body1.isStatic())
            jv += Vec3.loadFloat3Unsafe(&self.r1_plus_u_x_axis).dot(mp1.?.getAngularVelocity());
        if (!body2.isStatic())
            jv -= Vec3.loadFloat3Unsafe(&self.r2_x_axis).dot(mp2.?.getAngularVelocity());

        // Lagrange multiplier is:
        //
        // lambda = -K^-1 (J v + b)
        var lambda = self.effective_mass * (jv - self.spring_part.getBias(self.total_lambda));
        const new_lambda = math.clamp(self.total_lambda + lambda, min_lambda, max_lambda); // Clamp impulse
        lambda = new_lambda - self.total_lambda; // Lambda potentially got clamped, calculate the new impulse to apply
        self.total_lambda = new_lambda; // Store accumulated impulse

        return self.applyVelocityStep(body1, body2, world_space_axis, lambda);
    }

    /// Return lagrange multiplier
    pub fn getTotalLambda(self: *const AxisConstraintPart) f32 {
        return self.total_lambda;
    }

    /// Iteratively update the position constraint. Makes sure C(...) = 0.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inWorldSpaceAxis Axis along which the constraint acts (normalized)
    /// @param inC Value of the constraint equation (C)
    /// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
    pub fn solvePositionConstraint(self: *const AxisConstraintPart, body1: *Body, body2: *Body, world_space_axis: Vec3, c: f32, baumgarte: f32) bool {
        // Only apply position constraint when the constraint is hard, otherwise the velocity bias will fix the constraint
        if (c != 0.0 and !self.spring_part.isActive()) {
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
                body1.subPositionStep(world_space_axis.mulScalar(lambda * body1.getMotionProperties().getInverseMass()));
                body1.subRotationStep(Vec3.loadFloat3Unsafe(&self.inv_i1_r1_plus_u_x_axis).mulScalar(lambda));
            }
            if (body2.isDynamic()) {
                body2.addPositionStep(world_space_axis.mulScalar(lambda * body2.getMotionProperties().getInverseMass()));
                body2.addRotationStep(Vec3.loadFloat3Unsafe(&self.inv_i2_r2_x_axis).mulScalar(lambda));
            }
            return true;
        }

        return false;
    }

    /// Save state of this constraint part
    pub fn saveState(self: *const AxisConstraintPart, stream: *StateRecorder) void {
        stream.write(self.total_lambda);
    }

    /// Restore state of this constraint part
    pub fn restoreState(self: *AxisConstraintPart, stream: *StateRecorder) void {
        stream.read(&self.total_lambda);
    }
};

const TestContext = @import("ConstraintPartTestContext.zig");

/// J v of the axis constraint (the relative velocity of the constraint points along the axis)
fn testJacobianTimesVelocity(body1: *const Body, r1_plus_u: Vec3, body2: *const Body, r2: Vec3, axis: Vec3) f32 {
    return axis.dot(body1.getLinearVelocity().sub(body2.getLinearVelocity())) + r1_plus_u.cross(axis).dot(body1.getAngularVelocity()) - r2.cross(axis).dot(body2.getAngularVelocity());
}

test "AxisConstraintPart for every combination of motion types" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../../StateRecorderImpl.zig").StateRecorderImpl;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const r1_plus_u = Vec3.init(0.3, -0.5, 0.8);
    const r2 = Vec3.init(-0.4, 0.6, 0.2);
    const axis = Vec3.init(1, 2, -0.5).normalized();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);

        var part: AxisConstraintPart = .{};
        try testing.expect(!part.isActive());
        part.calculateConstraintProperties(body1, r1_plus_u, body2, r2, axis, .{});
        const any_dynamic = body1.isDynamic() or body2.isDynamic();
        try testing.expectEqual(any_dynamic, part.isActive());
        if (!any_dynamic) {
            try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
            continue;
        }
        try testing.expect(!part.spring_part.isActive());

        // A hard constraint without bias removes the relative velocity in one iteration
        const jv_before = testJacobianTimesVelocity(body1, r1_plus_u, body2, r2, axis);
        try testing.expect(@abs(jv_before) > 0.1);
        try testing.expect(part.solveVelocityConstraint(body1, body2, axis, -math.flt_max, math.flt_max));
        const jv_after = testJacobianTimesVelocity(body1, r1_plus_u, body2, r2, axis);
        try testing.expect(@abs(jv_after) < 1.0e-5);
        try testing.expectEqual(part.effective_mass * jv_before, part.getTotalLambda());
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Nothing left to do
        const lambda = part.getTotalLambda();
        _ = part.solveVelocityConstraint(body1, body2, axis, -math.flt_max, math.flt_max);
        try testing.expectApproxEqAbs(lambda, part.getTotalLambda(), 1.0e-5 * @abs(lambda));

        // Lambda limits clamp the accumulated impulse, a zero range applies nothing
        var limited: AxisConstraintPart = .{};
        limited.calculateConstraintProperties(body1, r1_plus_u, body2, r2, axis, .{ .bias = 1.0 });
        const snapshot1 = TestContext.BodySnapshot.take(body1);
        const snapshot2 = TestContext.BodySnapshot.take(body2);
        try testing.expect(!limited.solveVelocityConstraint(body1, body2, axis, 0.0, 0.0));
        try testing.expect(TestContext.BodySnapshot.take(body1).eql(snapshot1));
        try testing.expect(TestContext.BodySnapshot.take(body2).eql(snapshot2));
        _ = limited.solveVelocityConstraint(body1, body2, axis, -0.01, 0.02);
        try testing.expect(limited.getTotalLambda() >= -0.01 and limited.getTotalLambda() <= 0.02);

        // Warm start scales the accumulated impulse and applies it
        const total_lambda = part.getTotalLambda();
        const snapshot_ws = TestContext.BodySnapshot.take(if (body1.isDynamic()) body1 else body2);
        part.warmStart(body1, body2, axis, 0.5);
        try testing.expectEqual(total_lambda * 0.5, part.getTotalLambda());
        try testing.expect(!TestContext.BodySnapshot.take(if (body1.isDynamic()) body1 else body2).eql(snapshot_ws));
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Position constraint: only when there is an error and the constraint is hard
        try testing.expect(!part.solvePositionConstraint(body1, body2, axis, 0.0, 0.2));
        const snapshot_pos = TestContext.BodySnapshot.take(if (body1.isDynamic()) body1 else body2);
        try testing.expect(part.solvePositionConstraint(body1, body2, axis, 0.1, 0.2));
        const after_pos = TestContext.BodySnapshot.take(if (body1.isDynamic()) body1 else body2);
        try testing.expect(!after_pos.position.eql(snapshot_pos.position) or !after_pos.rotation.eql(snapshot_pos.rotation));
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Save / restore round trip
        var recorder = StateRecorderImpl.init(testing.allocator);
        defer recorder.deinit();
        part.saveState(recorder.recorder());
        try testing.expectEqualSlices(u8, std.mem.asBytes(&part.total_lambda), recorder.getData());
        var restored: AxisConstraintPart = .{};
        restored.restoreState(recorder.recorder());
        try testing.expectEqual(part.getTotalLambda(), restored.getTotalLambda());

        // Deactivate
        part.deactivate();
        try testing.expect(!part.isActive());
        try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
    }
}

test "AxisConstraintPart springs" {
    const testing = std.testing;
    const SpringMode = @import("../SpringSettings.zig").SpringMode;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    const body1 = bodies[0];
    const body2 = bodies[1];
    const r1_plus_u = Vec3.init(0.3, -0.5, 0.8);
    const r2 = Vec3.init(-0.4, 0.6, 0.2);
    const axis = Vec3.init(-1, 0.5, 2).normalized();
    const dt: f32 = 1.0 / 60.0;

    var hard: AxisConstraintPart = .{};
    hard.calculateConstraintProperties(body1, r1_plus_u, body2, r2, axis, .{});
    const inv_effective_mass = 1.0 / hard.effective_mass;

    // Frequency and damping: a soft constraint, the position constraint does nothing
    var soft: AxisConstraintPart = .{};
    soft.calculateConstraintPropertiesWithFrequencyAndDamping(dt, body1, r1_plus_u, body2, r2, axis, 0.0, 0.1, 2.0, 0.5);
    try testing.expect(soft.spring_part.isActive());
    try testing.expect(soft.effective_mass < hard.effective_mass);
    try testing.expect(!soft.solvePositionConstraint(body1, body2, axis, 0.1, 0.2));

    // Frequency 0 is a hard constraint with the bias
    var no_frequency: AxisConstraintPart = .{};
    no_frequency.calculateConstraintPropertiesWithFrequencyAndDamping(dt, body1, r1_plus_u, body2, r2, axis, 0.25, 0.1, 0.0, 0.5);
    try testing.expect(!no_frequency.spring_part.isActive());
    try testing.expectEqual(hard.effective_mass, no_frequency.effective_mass);
    try testing.expectEqual(@as(f32, 0.25), no_frequency.spring_part.getBias(0.0));

    // Stiffness and damping, also damping only
    var stiff: AxisConstraintPart = .{};
    stiff.calculateConstraintPropertiesWithStiffnessAndDamping(dt, body1, r1_plus_u, body2, r2, axis, 0.0, 0.1, 1000.0, 10.0);
    try testing.expect(stiff.spring_part.isActive());
    var damper: AxisConstraintPart = .{};
    damper.calculateConstraintPropertiesWithStiffnessAndDamping(dt, body1, r1_plus_u, body2, r2, axis, 0.0, 0.1, 0.0, 10.0);
    try testing.expect(damper.spring_part.isActive());
    var no_spring: AxisConstraintPart = .{};
    no_spring.calculateConstraintPropertiesWithStiffnessAndDamping(dt, body1, r1_plus_u, body2, r2, axis, 0.0, 0.1, 0.0, 0.0);
    try testing.expect(!no_spring.spring_part.isActive());

    // Settings for a limit: frequency 0 is hard, the modes match the direct calls
    const damping_only = SpringSettings.init(.stiffness_and_damping, 0.0, 10.0);
    var limit_hard: AxisConstraintPart = .{};
    limit_hard.calculateConstraintPropertiesWithSettingsForLimit(dt, body1, r1_plus_u, body2, r2, axis, 0.0, 0.1, &damping_only);
    try testing.expect(!limit_hard.spring_part.isActive());
    const modes = [_]SpringMode{ .frequency_and_damping, .stiffness_and_damping, .mass_normalized_stiffness_and_damping };
    for (modes) |mode| {
        const settings = SpringSettings.init(mode, 3.0, 0.5);
        var limit: AxisConstraintPart = .{};
        limit.calculateConstraintPropertiesWithSettingsForLimit(dt, body1, r1_plus_u, body2, r2, axis, 0.0, 0.1, &settings);
        var motor: AxisConstraintPart = .{};
        motor.calculateConstraintPropertiesWithSettingsForMotor(dt, body1, r1_plus_u, body2, r2, axis, 0.0, 0.1, &settings);
        var spring: SpringPart = .{};
        const effective_mass = spring.calculateSpringPropertiesWithSettings(dt, inv_effective_mass, 0.0, 0.1, &settings);
        try testing.expectEqual(effective_mass, limit.effective_mass);
        try testing.expectEqual(effective_mass, motor.effective_mass);
        try testing.expectEqual(spring.softness, limit.spring_part.softness);
        try testing.expectEqual(spring.bias, motor.spring_part.bias);
    }

    // A motor with damping only (frequency 0) is still a spring
    var motor_damping: AxisConstraintPart = .{};
    motor_damping.calculateConstraintPropertiesWithSettingsForMotor(dt, body1, r1_plus_u, body2, r2, axis, 0.0, 0.1, &damping_only);
    try testing.expect(motor_damping.spring_part.isActive());

    // The spring bias feeds back the accumulated impulse
    const total_before = soft.getTotalLambda();
    _ = soft.solveVelocityConstraint(body1, body2, axis, -math.flt_max, math.flt_max);
    try testing.expect(soft.getTotalLambda() != total_before);
}

test "AxisConstraintPart deactivates without effective mass" {
    const testing = std.testing;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    // A body that can only rotate around the constraint axis through its center of mass has no effective mass along it
    const body1 = try context.createBody(.dynamic, .{ .allowed_dofs = .{ .rotation_x = true } });
    const body2 = try context.createBody(.static, .{});
    var part: AxisConstraintPart = .{ .total_lambda = 5.0 };
    part.calculateConstraintPropertiesWithStiffnessAndDamping(1.0 / 60.0, body1, Vec3.zero(), body2, Vec3.zero(), Vec3.axisX(), 0.0, 0.0, 100.0, 1.0);
    try testing.expect(!part.isActive());
    try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
}
