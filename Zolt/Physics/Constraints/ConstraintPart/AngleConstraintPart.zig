//! Port of: Jolt/Physics/Constraints/ConstraintPart/AngleConstraintPart.h
//! Status: complete
//!
//! The defaulted `inBias = 0.0f` of CalculateConstraintProperties is an options struct (`.{ .bias = b }`).

const std = @import("std");
const Core = @import("../../../Core/Core.zig");
const math = @import("../../../Math/Math.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Body = @import("../../Body/Body.zig").Body;
const SpringPart = @import("SpringPart.zig").SpringPart;
const SpringSettings = @import("../SpringSettings.zig").SpringSettings;
const StateRecorder = @import("../../StateRecorder.zig").StateRecorder;

/// Constraint that constrains rotation along 1 axis
///
/// Based on: "Constraints Derivation for Rigid Body Simulation in 3D" - Daniel Chappuis, see section 2.4.5
///
/// Constraint equation (eq 108):
///
/// \f[C = \theta(t) - \theta_{min}\f]
///
/// Jacobian (eq 109):
///
/// \f[J = \begin{bmatrix}0 & -a^T & 0 & a^T\end{bmatrix}\f]
///
/// Used terms (here and below, everything in world space):\n
/// a = axis around which rotation is constrained (normalized).\n
/// x1, x2 = center of mass for the bodies.\n
/// v = [v1, w1, v2, w2].\n
/// v1, v2 = linear velocity of body 1 and 2.\n
/// w1, w2 = angular velocity of body 1 and 2.\n
/// M = mass matrix, a diagonal matrix of the mass and inertia with diagonal [m1, I1, m2, I2].\n
/// \f$K^{-1} = \left( J M^{-1} J^T \right)^{-1}\f$ = effective mass.\n
/// b = velocity bias.\n
/// \f$\beta\f$ = baumgarte constant.
pub const AngleConstraintPart = struct {
    inv_i1_axis: Vec3 = undefined,
    inv_i2_axis: Vec3 = undefined,
    effective_mass: f32 = 0.0,
    spring_part: SpringPart = .{},
    total_lambda: f32 = 0.0,

    /// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
    fn applyVelocityStep(self: *const AngleConstraintPart, body1: *Body, body2: *Body, lambda: f32) bool {
        // Apply impulse if delta is not zero
        if (lambda != 0.0) {
            // Calculate velocity change due to constraint
            //
            // Impulse:
            // P = J^T lambda
            //
            // Euler velocity integration:
            // v' = v + M^-1 P
            if (body1.isDynamic())
                body1.getMotionPropertiesMut().subAngularVelocityStep(self.inv_i1_axis.mulScalar(lambda));
            if (body2.isDynamic())
                body2.getMotionPropertiesMut().addAngularVelocityStep(self.inv_i2_axis.mulScalar(lambda));
            return true;
        }

        return false;
    }

    /// Internal helper function to calculate the inverse effective mass
    fn calculateInverseEffectiveMass(self: *AngleConstraintPart, body1: *const Body, body2: *const Body, world_space_axis: Vec3) f32 {
        if (Core.enable_asserts) std.debug.assert(world_space_axis.isNormalized(.{ .tolerance = 1.0e-4 }));

        // Calculate properties used below
        self.inv_i1_axis = if (body1.isDynamic()) body1.getMotionProperties().multiplyWorldSpaceInverseInertiaByVector(body1.getRotation(), world_space_axis) else Vec3.zero();
        self.inv_i2_axis = if (body2.isDynamic()) body2.getMotionProperties().multiplyWorldSpaceInverseInertiaByVector(body2.getRotation(), world_space_axis) else Vec3.zero();

        // Calculate inverse effective mass: K = J M^-1 J^T
        return world_space_axis.dot(self.inv_i1_axis.add(self.inv_i2_axis));
    }

    /// Calculate properties used during the functions below. Creates a constraint without spring.
    /// @param inBody1 The first body that this constraint is attached to
    /// @param inBody2 The second body that this constraint is attached to
    /// @param inWorldSpaceAxis The axis of rotation along which the constraint acts (normalized)
    /// Set the following terms to zero if you don't want to drive the constraint to zero with a spring:
    /// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
    pub fn calculateConstraintProperties(self: *AngleConstraintPart, body1: *const Body, body2: *const Body, world_space_axis: Vec3, opts: struct { bias: f32 = 0.0 }) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, body2, world_space_axis);

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
    /// @param inWorldSpaceAxis The axis of rotation along which the constraint acts (normalized)
    /// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
    /// @param inC Value of the constraint equation (C)
    /// @param inFrequency Oscillation frequency (Hz)
    /// @param inDamping Damping factor (0 = no damping, 1 = critical damping)
    pub fn calculateConstraintPropertiesWithFrequencyAndDamping(self: *AngleConstraintPart, delta_time: f32, body1: *const Body, body2: *const Body, world_space_axis: Vec3, bias: f32, c: f32, frequency: f32, damping: f32) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, body2, world_space_axis);

        if (inv_effective_mass < math.flt_min)
            self.deactivate()
        else if (frequency > 0.0)
            self.effective_mass = self.spring_part.calculateSpringPropertiesWithFrequencyAndDamping(delta_time, inv_effective_mass, bias, c, frequency, damping)
        else {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring_part.calculateSpringPropertiesWithBias(bias);
        }
    }

    /// Calculate properties used during the functions below. Set inStiffness and inDamping to zero if you don't want to drive using a spring.
    /// @param inDeltaTime Time step
    /// @param inBody1 The first body that this constraint is attached to
    /// @param inBody2 The second body that this constraint is attached to
    /// @param inWorldSpaceAxis The axis of rotation along which the constraint acts (normalized)
    /// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
    /// @param inC Value of the constraint equation (C)
    /// @param inStiffness Spring stiffness k.
    /// @param inDamping Spring damping coefficient c.
    pub fn calculateConstraintPropertiesWithStiffnessAndDamping(self: *AngleConstraintPart, delta_time: f32, body1: *const Body, body2: *const Body, world_space_axis: Vec3, bias: f32, c: f32, stiffness: f32, damping: f32) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, body2, world_space_axis);

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
    pub fn calculateConstraintPropertiesWithSettingsForLimit(self: *AngleConstraintPart, delta_time: f32, body1: *const Body, body2: *const Body, world_space_axis: Vec3, bias: f32, c: f32, spring_settings: *const SpringSettings) void {
        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, body2, world_space_axis);

        if (inv_effective_mass < math.flt_min)
            self.deactivate()
        else if (!spring_settings.hasStiffness()) {
            self.effective_mass = 1.0 / inv_effective_mass;
            self.spring_part.calculateSpringPropertiesWithBias(bias);
        } else self.effective_mass = self.spring_part.calculateSpringPropertiesWithSettings(delta_time, inv_effective_mass, bias, c, spring_settings);
    }

    /// Calculate properties used during the functions below based on inSpringSettings.
    /// Assumes the spring has either stiffness or damping.
    pub fn calculateConstraintPropertiesWithSettingsForMotor(self: *AngleConstraintPart, delta_time: f32, body1: *const Body, body2: *const Body, world_space_axis: Vec3, bias: f32, c: f32, spring_settings: *const SpringSettings) void {
        if (Core.enable_asserts) std.debug.assert(spring_settings.hasStiffnessOrDamping());

        const inv_effective_mass = self.calculateInverseEffectiveMass(body1, body2, world_space_axis);

        if (inv_effective_mass < math.flt_min)
            self.deactivate()
        else
            self.effective_mass = self.spring_part.calculateSpringPropertiesWithSettings(delta_time, inv_effective_mass, bias, c, spring_settings);
    }

    /// Deactivate this constraint
    pub fn deactivate(self: *AngleConstraintPart) void {
        self.effective_mass = 0.0;
        self.total_lambda = 0.0;
    }

    /// Check if constraint is active
    pub fn isActive(self: *const AngleConstraintPart) bool {
        return self.effective_mass != 0.0;
    }

    /// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
    pub fn warmStart(self: *AngleConstraintPart, body1: *Body, body2: *Body, warm_start_impulse_ratio: f32) void {
        self.total_lambda *= warm_start_impulse_ratio;
        _ = self.applyVelocityStep(body1, body2, self.total_lambda);
    }

    /// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inWorldSpaceAxis The axis of rotation along which the constraint acts (normalized)
    /// @param inMinLambda Minimum angular impulse to apply (N m s)
    /// @param inMaxLambda Maximum angular impulse to apply (N m s)
    pub fn solveVelocityConstraint(self: *AngleConstraintPart, body1: *Body, body2: *Body, world_space_axis: Vec3, min_lambda: f32, max_lambda: f32) bool {
        // Lagrange multiplier is:
        //
        // lambda = -K^-1 (J v + b)
        var lambda = self.effective_mass * (world_space_axis.dot(body1.getAngularVelocity().sub(body2.getAngularVelocity())) - self.spring_part.getBias(self.total_lambda));
        const new_lambda = math.clamp(self.total_lambda + lambda, min_lambda, max_lambda); // Clamp impulse
        lambda = new_lambda - self.total_lambda; // Lambda potentially got clamped, calculate the new impulse to apply
        self.total_lambda = new_lambda; // Store accumulated impulse

        return self.applyVelocityStep(body1, body2, lambda);
    }

    /// Return lagrange multiplier
    pub fn getTotalLambda(self: *const AngleConstraintPart) f32 {
        return self.total_lambda;
    }

    /// Iteratively update the position constraint. Makes sure C(...) == 0.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inC Value of the constraint equation (C)
    /// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
    pub fn solvePositionConstraint(self: *const AngleConstraintPart, body1: *Body, body2: *Body, c: f32, baumgarte: f32) bool {
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
            if (body1.isDynamic())
                body1.subRotationStep(self.inv_i1_axis.mulScalar(lambda));
            if (body2.isDynamic())
                body2.addRotationStep(self.inv_i2_axis.mulScalar(lambda));
            return true;
        }

        return false;
    }

    /// Save state of this constraint part
    pub fn saveState(self: *const AngleConstraintPart, stream: *StateRecorder) void {
        stream.write(self.total_lambda);
    }

    /// Restore state of this constraint part
    pub fn restoreState(self: *AngleConstraintPart, stream: *StateRecorder) void {
        stream.read(&self.total_lambda);
    }
};

const TestContext = @import("ConstraintPartTestContext.zig");

test "AngleConstraintPart for every combination of motion types" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../../StateRecorderImpl.zig").StateRecorderImpl;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const axis = Vec3.init(-1, 1, 1).normalized();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);

        var part: AngleConstraintPart = .{};
        part.calculateConstraintProperties(body1, body2, axis, .{});
        const any_dynamic = body1.isDynamic() or body2.isDynamic();
        try testing.expectEqual(any_dynamic, part.isActive());
        if (!any_dynamic)
            continue;

        // A hard constraint removes the relative angular velocity around the axis
        const jv_before = axis.dot(body1.getAngularVelocity().sub(body2.getAngularVelocity()));
        try testing.expect(@abs(jv_before) > 0.1);
        try testing.expect(part.solveVelocityConstraint(body1, body2, axis, -math.flt_max, math.flt_max));
        try testing.expectApproxEqAbs(@as(f32, 0.0), axis.dot(body1.getAngularVelocity().sub(body2.getAngularVelocity())), 1.0e-5);
        try testing.expectEqual(part.effective_mass * jv_before, part.getTotalLambda());
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Limits
        var limited: AngleConstraintPart = .{};
        limited.calculateConstraintProperties(body1, body2, axis, .{ .bias = 0.5 });
        const snapshot = TestContext.BodySnapshot.take(if (body1.isDynamic()) body1 else body2);
        try testing.expect(!limited.solveVelocityConstraint(body1, body2, axis, 0.0, 0.0));
        try testing.expect(TestContext.BodySnapshot.take(if (body1.isDynamic()) body1 else body2).eql(snapshot));
        _ = limited.solveVelocityConstraint(body1, body2, axis, -0.001, 0.001);
        try testing.expect(@abs(limited.getTotalLambda()) <= 0.001);

        // Warm start
        const lambda = part.getTotalLambda();
        part.warmStart(body1, body2, 0.5);
        try testing.expectEqual(lambda * 0.5, part.getTotalLambda());
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // Position
        try testing.expect(!part.solvePositionConstraint(body1, body2, 0.0, 0.2));
        const dynamic_body = if (body1.isDynamic()) body1 else body2;
        const rotation_before = dynamic_body.getRotation();
        try testing.expect(part.solvePositionConstraint(body1, body2, 0.1, 0.2));
        try testing.expect(!dynamic_body.getRotation().eql(rotation_before));
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);

        // State
        var recorder = StateRecorderImpl.init(testing.allocator);
        defer recorder.deinit();
        part.saveState(recorder.recorder());
        var restored: AngleConstraintPart = .{};
        restored.restoreState(recorder.recorder());
        try testing.expectEqual(part.getTotalLambda(), restored.getTotalLambda());

        part.deactivate();
        try testing.expect(!part.isActive());
        try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
    }
}

test "AngleConstraintPart springs" {
    const testing = std.testing;
    const SpringMode = @import("../SpringSettings.zig").SpringMode;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const bodies = try context.createBodyPair(.{ .kinematic, .dynamic });
    const body1 = bodies[0];
    const body2 = bodies[1];
    const axis = Vec3.init(0.2, -1, 0.4).normalized();
    const dt: f32 = 1.0 / 60.0;

    var hard: AngleConstraintPart = .{};
    hard.calculateConstraintProperties(body1, body2, axis, .{});
    const inv_effective_mass = 1.0 / hard.effective_mass;

    var soft: AngleConstraintPart = .{};
    soft.calculateConstraintPropertiesWithFrequencyAndDamping(dt, body1, body2, axis, 0.0, 0.2, 1.0, 0.1);
    try testing.expect(soft.spring_part.isActive());
    try testing.expect(!soft.solvePositionConstraint(body1, body2, 0.2, 0.2));
    var no_frequency: AngleConstraintPart = .{};
    no_frequency.calculateConstraintPropertiesWithFrequencyAndDamping(dt, body1, body2, axis, 0.0, 0.2, 0.0, 0.1);
    try testing.expect(!no_frequency.spring_part.isActive());

    var stiff: AngleConstraintPart = .{};
    stiff.calculateConstraintPropertiesWithStiffnessAndDamping(dt, body1, body2, axis, 0.0, 0.2, 50.0, 0.0);
    try testing.expect(stiff.spring_part.isActive());
    var no_stiffness: AngleConstraintPart = .{};
    no_stiffness.calculateConstraintPropertiesWithStiffnessAndDamping(dt, body1, body2, axis, 0.0, 0.2, 0.0, 0.0);
    try testing.expect(!no_stiffness.spring_part.isActive());

    const hard_settings = SpringSettings.init(.frequency_and_damping, 0.0, 1.0);
    var limit_hard: AngleConstraintPart = .{};
    limit_hard.calculateConstraintPropertiesWithSettingsForLimit(dt, body1, body2, axis, 0.0, 0.2, &hard_settings);
    try testing.expect(!limit_hard.spring_part.isActive());
    try testing.expectEqual(hard.effective_mass, limit_hard.effective_mass);

    const modes = [_]SpringMode{ .frequency_and_damping, .stiffness_and_damping, .mass_normalized_stiffness_and_damping };
    for (modes) |mode| {
        const settings = SpringSettings.init(mode, 4.0, 0.25);
        var limit: AngleConstraintPart = .{};
        limit.calculateConstraintPropertiesWithSettingsForLimit(dt, body1, body2, axis, 0.0, 0.2, &settings);
        var motor: AngleConstraintPart = .{};
        motor.calculateConstraintPropertiesWithSettingsForMotor(dt, body1, body2, axis, 0.0, 0.2, &settings);
        var spring: SpringPart = .{};
        const effective_mass = spring.calculateSpringPropertiesWithSettings(dt, inv_effective_mass, 0.0, 0.2, &settings);
        try testing.expectEqual(effective_mass, limit.effective_mass);
        try testing.expectEqual(effective_mass, motor.effective_mass);
        try testing.expectEqual(spring.softness, motor.spring_part.softness);
    }
}

test "AngleConstraintPart deactivates without inertia around the axis" {
    const testing = std.testing;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    // Body 1 can only translate: no inverse inertia
    const body1 = try context.createBody(.dynamic, .{ .allowed_dofs = .{ .translation_x = true, .translation_y = true } });
    const body2 = try context.createBody(.kinematic, .{});
    var part: AngleConstraintPart = .{ .total_lambda = 1.0 };
    part.calculateConstraintPropertiesWithFrequencyAndDamping(1.0 / 60.0, body1, body2, Vec3.axisZ(), 0.0, 0.0, 2.0, 1.0);
    try testing.expect(!part.isActive());
    try testing.expectEqual(@as(f32, 0.0), part.getTotalLambda());
}
