//! Port of: UnitTests/Physics/HingeConstraintTests.cpp
//! Status: complete
//!
//! The class itself (binary state, conversions, setters, solver calls, allocation failures) is tested inline in
//! Zolt/Physics/Constraints/HingeConstraint.zig, bit exactness in ZoltParity/Physics/ConstraintsParity.zig and the
//! hinge scenes of ZoltParity/Physics/SimulationParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const HingeConstraint = zolt.HingeConstraint;
const HingeConstraintSettings = zolt.HingeConstraintSettings;
const math = zolt.math;

const checkApproxEqual = fw.checkApproxEqual;

// Test if the hinge constraint can be used to create a spring
test "TestHingeSpring" {
    const allocator = std.testing.allocator;

    // Configuration of the spring
    const initial_angle = math.degreesToRadians(100.0);
    const frequency: f32 = 2.0;
    const damping: f32 = 0.1;

    for (0..2) |mode| {
        // Create a sphere
        var context: PhysicsTestContext = undefined;
        try context.init(allocator, .{});
        defer context.deinit();
        const body = try context.createSphere(RVec3.zero(), 0.5, .dynamic, .discrete, Layers.moving, .{ .activation = .activate });
        body.getMotionPropertiesMut().setAngularDamping(0.0);
        body.setAllowSleeping(false);

        // Calculate stiffness and damping of spring
        const inertia = body.getMotionProperties().getInverseInertiaForRotation(Mat44.identity()).inversed3x3().getAxisY().length();
        const omega = 2.0 * math.pi * frequency;
        const k = inertia * math.square(omega);
        const c = 2.0 * inertia * damping * omega;

        // Create spring
        var constraint = HingeConstraintSettings.init(allocator);
        defer constraint.asSettings().deinit();
        if (mode == 0) {
            // First iteration use stiffness and damping
            constraint.limits_spring_settings.mode = .stiffness_and_damping;
            constraint.limits_spring_settings.setStiffness(k);
            constraint.limits_spring_settings.damping = c;
        } else {
            // Second iteration use frequency and damping
            constraint.limits_spring_settings.mode = .frequency_and_damping;
            constraint.limits_spring_settings.frequency = frequency;
            constraint.limits_spring_settings.damping = damping;
        }
        constraint.limits_min = 0.0;
        constraint.limits_max = 0.0;
        _ = try context.createConstraint(HingeConstraint, Body.fixedToWorld(), body, constraint.asTwoBodySettingsConst());

        // Rotate the body to the initial angle
        context.getBodyInterface().setRotation(body.getID(), Quat.rotation(Vec3.axisY(), initial_angle), .activate);

        // Simulate angular spring
        var angle = initial_angle;
        var angular_v: f32 = 0.0;
        const dt = context.getDeltaTime();
        for (0..120) |_| {
            // Using the equations from page 32 of Soft Constraints: Reinventing The Spring - Erin Catto - GDC 2011 for an implicit euler spring damper
            angular_v = (angular_v - dt * k / inertia * angle) / (1.0 + dt * c / inertia + math.square(dt) * k / inertia);
            angle += angular_v * dt;

            // Run physics simulation
            _ = try context.simulateSingleStep();

            // Decompose body rotation
            const axis_angle = body.getRotation().getAxisAngle();
            const actual_axis = axis_angle.axis;
            var actual_angle = axis_angle.angle;
            if (actual_axis.getY() < 0.0)
                actual_angle = -actual_angle;

            // Test if simulation matches prediction
            try checkApproxEqual(angle, actual_angle, .{ .tolerance = math.degreesToRadians(0.1) });
            try checkApproxEqual(actual_axis.getX(), 0, .{});
            try checkApproxEqual(actual_axis.getZ(), 0, .{});
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Zolt additions (Jolt has only TestHingeSpring): the motor, the limits and the friction of the hinge in a
// PhysicsSystem

/// A box at `position` hinged to the world around `axis` at `pivot` (world space, the same frame on both sides)
fn createHingedBox(context: *PhysicsTestContext, settings: *HingeConstraintSettings, position: RVec3, pivot: RVec3, axis: Vec3, normal: Vec3) !struct { *Body, *HingeConstraint } {
    const box = try context.createBox(position, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.5, 0.25, 0.25), .{});
    box.setAllowSleeping(false);
    settings.point1 = pivot;
    settings.point2 = pivot;
    settings.hinge_axis1 = axis;
    settings.hinge_axis2 = axis;
    settings.normal_axis1 = normal;
    settings.normal_axis2 = normal;
    const hinge = try context.createConstraint(HingeConstraint, Body.fixedToWorld(), box, settings.asTwoBodySettingsConst());
    return .{ box, hinge };
}

test "TestHingeMotorVelocity" {
    const allocator = std.testing.allocator;
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    context.zeroGravity();

    // A box spinning around the Y axis through its center of mass, driven by a velocity motor
    var settings = HingeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    const box, const hinge = try createHingedBox(&context, &settings, RVec3.init(0, 2, 0), RVec3.init(0, 2, 0), Vec3.axisY(), Vec3.axisX());
    hinge.setMotorState(.velocity);
    hinge.setTargetAngularVelocity(2.0);
    _ = try context.simulate(0.5, {});

    // The box turns with the target velocity around the hinge axis only, the motor applies no impulse once it is there
    try checkApproxEqual(box.getAngularVelocity(), Vec3.init(0, 2.0, 0), .{ .tolerance = 1.0e-3 });
    try std.testing.expect(@abs(hinge.getTotalLambdaMotor()) < 1.0e-4);
    const angle_before = hinge.getCurrentAngle();
    _ = try context.simulate(0.25, {});
    try checkApproxEqual(math.centerAngleAroundZero(hinge.getCurrentAngle() - angle_before), 0.5, .{ .tolerance = 1.0e-2 });

    // A weak motor (torque limit) accelerates the box no faster than its torque allows: dw = T / I * t
    hinge.getMotorSettingsMut().setTorqueLimit(0.05);
    hinge.setTargetAngularVelocity(-2.0);
    const inertia = 1.0 / box.getMotionProperties().getInverseInertiaForRotation(Mat44.identity()).getAxisY().getY();
    _ = try context.simulate(0.5, {});
    try checkApproxEqual(box.getAngularVelocity().getY(), 2.0 - 0.05 / inertia * 0.5, .{ .tolerance = 1.0e-2 });
}

test "TestHingeMotorPosition" {
    const allocator = std.testing.allocator;
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    context.zeroGravity();

    // A position motor turns the box to the target angle (a target orientation in body space works the same)
    var settings = HingeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.motor_settings = .init(2.0, 1.0);
    const box, const hinge = try createHingedBox(&context, &settings, RVec3.init(1, 2, 0), RVec3.init(0, 2, 0), Vec3.axisZ(), Vec3.axisX());
    hinge.setMotorState(.position);
    hinge.setTargetAngle(0.8);
    _ = try context.simulate(6.0, {}); // The spring acts on the inertia around the center of mass, the box swings longer
    try checkApproxEqual(hinge.getCurrentAngle(), 0.8, .{ .tolerance = 1.0e-2 });
    try checkApproxEqual(box.getAngularVelocity(), Vec3.zero(), .{ .tolerance = 1.0e-2 });

    hinge.setTargetOrientationBS(Quat.rotation(Vec3.axisZ(), -0.4));
    try checkApproxEqual(hinge.getTargetAngle(), -0.4, .{ .tolerance = 1.0e-5 });
    _ = try context.simulate(6.0, {});
    try checkApproxEqual(hinge.getCurrentAngle(), -0.4, .{ .tolerance = 1.0e-2 });

    // The pivot stays where it is
    const pivot = box.getCenterOfMassTransform().mulRVec3(RVec3.fromVec3(hinge.getLocalSpacePoint2()));
    try std.testing.expect(pivot.sub(RVec3.init(0, 2, 0)).length() < 1.0e-3);
}

test "TestHingeLimits" {
    const allocator = std.testing.allocator;
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();

    // A pendulum around the Z axis that falls from horizontal: the limits stop it at -0.5
    var settings = HingeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.limits_min = -0.5;
    settings.limits_max = 0.25;
    _, const hinge = try createHingedBox(&context, &settings, RVec3.init(1, 5, 0), RVec3.init(0, 5, 0), Vec3.axisZ(), Vec3.axisX());
    try std.testing.expect(hinge.hasLimits());
    var min_angle: f32 = 0.0;
    for (0..120) |_| {
        _ = try context.simulateSingleStep();
        min_angle = @min(min_angle, hinge.getCurrentAngle());
    }
    try std.testing.expect(min_angle > -0.5 - 0.02);
    try checkApproxEqual(hinge.getCurrentAngle(), -0.5, .{ .tolerance = 1.0e-2 });
    try std.testing.expect(hinge.getTotalLambdaRotationLimits() > 0.0); // Pushes towards the inside

    // Without limits it swings through
    hinge.setLimits(-math.pi, math.pi);
    try std.testing.expect(!hinge.hasLimits());
    _ = try context.simulate(0.5, {});
    try std.testing.expect(hinge.getCurrentAngle() < -0.6);
    try std.testing.expectEqual(@as(f32, 0.0), hinge.getTotalLambdaRotationLimits());
}

test "TestHingeFriction" {
    const allocator = std.testing.allocator;
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    context.zeroGravity();

    // A spinning box hinged at its center of mass: the friction torque decelerates it linearly until it stops
    var settings = HingeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.max_friction_torque = 0.02;
    const box, const hinge = try createHingedBox(&context, &settings, RVec3.init(0, 2, 0), RVec3.init(0, 2, 0), Vec3.axisY(), Vec3.axisX());
    context.getBodyInterface().setAngularVelocity(box.getID(), Vec3.init(0, 3.0, 0));
    const inertia = 1.0 / box.getMotionProperties().getInverseInertiaForRotation(Mat44.identity()).getAxisY().getY();
    _ = try context.simulate(0.5, {});
    try checkApproxEqual(box.getAngularVelocity().getY(), 3.0 - 0.02 / inertia * 0.5, .{ .tolerance = 2.0e-2 });
    try checkApproxEqual(hinge.getTotalLambdaMotor(), -0.02 * context.getDeltaTime(), .{ .tolerance = 1.0e-6 });

    // Strong friction stops it
    hinge.setMaxFrictionTorque(1000.0);
    _ = try context.simulate(0.25, {});
    try std.testing.expect(@abs(box.getAngularVelocity().getY()) < 1.0e-3);
}
