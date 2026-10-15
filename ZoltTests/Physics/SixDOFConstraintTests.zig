//! Port of: UnitTests/Physics/SixDOFConstraintTests.cpp
//! Status: complete
//!
//! Jolt's TEST_CASEs with the same names and values, plus Zolt simulations of the limits, the friction and the
//! position motors in a PhysicsSystem. The class itself (binary state, conversions, the limits bookkeeping, setters,
//! solver calls, allocation failures) is tested inline in Zolt/Physics/Constraints/SixDOFConstraint.zig, bit exactness
//! in ZoltParity/Physics/ConstraintsParity.zig and the six_dof scenes of ZoltParity/Physics/SimulationParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Core = zolt.Core;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Real = zolt.Real;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const RefConst = zolt.RefConst;
const Ref = zolt.Ref;
const Shape = zolt.Shape;
const SpringSettings = zolt.SpringSettings;
const SixDOFConstraint = zolt.SixDOFConstraint;
const SixDOFConstraintSettings = zolt.SixDOFConstraintSettings;
const math = zolt.math;

const expect = std.testing.expect;
const checkApproxEqual = fw.checkApproxEqual;

/// x += v * dt for a world space position
fn addVelocity(x: RVec3, v: Vec3, dt: f32) RVec3 {
    return if (Core.double_precision) x.addVec3(v.mulScalar(dt)) else x.add(v.mulScalar(dt));
}

// Test if the 6DOF constraint can be used to create a spring
test "TestSixDOFSpring" {
    const allocator = std.testing.allocator;

    // Configuration of the spring
    const frequency: f32 = 2.0;
    const damping: f32 = 0.1;

    // Test all permutations of axis
    var spring_axis: u32 = 0b001;
    while (spring_axis <= 0b111) : (spring_axis += 1) {
        // Test all spring modes
        for (0..2) |mode| {
            const initial_position = RVec3.init(10.0 * @as(f32, @floatFromInt(spring_axis & 1)), 8.0 * @as(f32, @floatFromInt(spring_axis & 2)), 6.0 * @as(f32, @floatFromInt(spring_axis & 4)));

            // Create a sphere
            var context: PhysicsTestContext = undefined;
            try context.init(allocator, .{});
            defer context.deinit();
            context.zeroGravity();
            const body = try context.createSphere(initial_position, 0.5, .dynamic, .discrete, Layers.moving, .{});
            body.getMotionPropertiesMut().setLinearDamping(0.0);

            // Calculate stiffness and damping of spring
            const m = 1.0 / body.getMotionProperties().getInverseMass();
            const omega = 2.0 * math.pi * frequency;
            const k = m * math.square(omega);
            const c = 2.0 * m * damping * omega;

            // Create spring
            var constraint = SixDOFConstraintSettings.init(allocator);
            defer constraint.asSettings().deinit();
            constraint.position2 = initial_position;
            for (0..3) |axis| {
                // Check if this axis is supposed to be a spring
                if (((@as(u32, 1) << @intCast(axis)) & spring_axis) != 0) {
                    if (mode == 0) {
                        // First iteration use stiffness and damping
                        constraint.limits_spring_settings[axis].mode = .stiffness_and_damping;
                        constraint.limits_spring_settings[axis].setStiffness(k);
                        constraint.limits_spring_settings[axis].damping = c;
                    } else {
                        // Second iteration use frequency and damping
                        constraint.limits_spring_settings[axis].mode = .frequency_and_damping;
                        constraint.limits_spring_settings[axis].frequency = frequency;
                        constraint.limits_spring_settings[axis].damping = damping;
                    }
                    constraint.limit_max[axis] = 0.0;
                    constraint.limit_min[axis] = 0.0;
                }
            }
            _ = try context.createConstraint(SixDOFConstraint, Body.fixedToWorld(), body, constraint.asTwoBodySettingsConst());

            // Simulate spring
            var x = initial_position;
            var v = Vec3.zero();
            const dt = context.getDeltaTime();
            for (0..120) |_| {
                // Using the equations from page 32 of Soft Constraints: Reinventing The Spring - Erin Catto - GDC 2011 for an implicit euler spring damper
                for (0..3) |axis| {
                    if (((@as(u32, 1) << @intCast(axis)) & spring_axis) != 0) // Only update velocity for axis where there is a spring
                        v.setComponent(@intCast(axis), (v.getComponent(@intCast(axis)) - dt * k / m * @as(f32, @floatCast(x.getComponent(@intCast(axis))))) / (1.0 + dt * c / m + math.square(dt) * k / m));
                }
                x = addVelocity(x, v, dt);

                // Run physics simulation
                _ = try context.simulateSingleStep();

                // Test if simulation matches prediction
                try checkApproxEqual(x, body.getPosition(), .{ .tolerance = 1.0e-5 });
            }
        }
    }
}

// Test that a velocity motor with damping-only SpringSettings acts as a soft, mass-normalized
// acceleration-mode drive: a = -damping * v_err. Velocity follows v_new = (v_old + dt * c * v_target) / (1 + dt * c)
// independently of body mass/inertia. Exercises both translational and rotational motor paths and both spring modes
test "TestSixDOFMotorVelocityAcceleration" {
    const allocator = std.testing.allocator;

    const damping: f32 = 4.0;
    const target_velocity: f32 = 5.0;

    // Cover all translation and rotation motor axes
    for (0..6) |axis| {
        const is_rotation = axis >= 3;

        // Run against two sphere sizes; identical motion curves prove mass/inertia independence
        for ([_]f32{ 0.5, 1.0 }) |radius| {
            var context: PhysicsTestContext = undefined;
            try context.init(allocator, .{});
            defer context.deinit();
            context.zeroGravity();
            const body = try context.createSphere(RVec3.zero(), radius, .dynamic, .discrete, Layers.moving, .{});
            body.getMotionPropertiesMut().setLinearDamping(0.0);
            body.getMotionPropertiesMut().setAngularDamping(0.0);

            var settings = SixDOFConstraintSettings.init(allocator);
            defer settings.asSettings().deinit();
            settings.position1 = RVec3.zero();
            settings.position2 = RVec3.zero();
            settings.motor_settings[axis].spring_settings = SpringSettings.init(.mass_normalized_stiffness_and_damping, 0.0, damping);
            const constraint = try context.createConstraint(SixDOFConstraint, Body.fixedToWorld(), body, settings.asTwoBodySettingsConst());
            constraint.setMotorState(@enumFromInt(axis), .position_and_velocity);
            var target = Vec3.zero();
            target.setComponent(@intCast(axis % 3), target_velocity);
            if (is_rotation)
                constraint.setTargetAngularVelocityCS(target)
            else
                constraint.setTargetVelocityCS(target);

            // Predicted velocity. Implicit Euler soft-constraint form from 'Soft Constraints: Reinventing
            // The Spring' - Erin Catto - GDC 2011 (page 32), k = 0 limit, mass-normalized damping c:
            // v_new = (v_old + dt * c * v_target) / (1 + dt * c)
            var v: f32 = 0.0;
            const dt = context.getDeltaTime();
            for (0..120) |_| {
                v = (v + dt * damping * target_velocity) / (1.0 + dt * damping);
                _ = try context.simulateSingleStep();
                var expected = Vec3.zero();
                expected.setComponent(@intCast(axis % 3), v);
                const actual = if (is_rotation) body.getAngularVelocity() else body.getLinearVelocity();
                try checkApproxEqual(expected, actual, .{ .tolerance = 1.0e-5 });
            }
        }
    }
}

// Test combination of locked rotation axis with a 6DOF constraint
test "TestSixDOFLockedRotation" {
    const allocator = std.testing.allocator;
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    const bi = context.getBodyInterface();
    const system = context.getSystem();

    var box_shape = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(1.0), .{})).asShape());
    defer box_shape.deinit();

    // Static 'anchor' body
    var settings1 = BodyCreationSettings.initWithShape(box_shape.get(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer settings1.deinit();
    const body1 = (try bi.createBody(&settings1)).?;
    try bi.addBody(body1.getID(), .activate);

    // Dynamic body that cannot rotate around X and Y
    const position2 = RVec3.init(3, 0, 0);
    const rotation2 = Quat.identity();
    var settings2 = BodyCreationSettings.initWithShape(box_shape.get(), position2, rotation2, .dynamic, Layers.moving);
    defer settings2.deinit();
    settings2.allowed_dofs = .{ .rotation_z = true, .translation_x = true, .translation_y = true, .translation_z = true };
    const body2 = (try bi.createBody(&settings2)).?;
    try bi.addBody(body2.getID(), .activate);

    // Lock all 6 axis with a 6DOF constraint
    var six_dof = SixDOFConstraintSettings.init(allocator);
    defer six_dof.asSettings().deinit();
    six_dof.makeFixedAxis(.translation_x);
    six_dof.makeFixedAxis(.translation_y);
    six_dof.makeFixedAxis(.translation_z);
    six_dof.makeFixedAxis(.rotation_x);
    six_dof.makeFixedAxis(.rotation_y);
    six_dof.makeFixedAxis(.rotation_z);
    {
        var constraint = Ref(zolt.Constraint).init(&(try six_dof.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer constraint.deinit(); // AddConstraint holds its own reference
        try system.addConstraint(constraint.get().?);
    }

    _ = try context.simulate(1.0, {});

    // Check that body didn't rotate
    try checkApproxEqual(body2.getPosition(), position2, .{ .tolerance = 5.0e-3 });
    try checkApproxEqual(body2.getRotation(), rotation2, .{ .tolerance = 5.0e-3 });
}

// ---------------------------------------------------------------------------------------------------------------------
// Zolt additions: the limits, the friction and the position motors of the SixDOF in a PhysicsSystem

test "TestSixDOFTranslationAndRotationLimits" {
    const allocator = std.testing.allocator;
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();

    // A box hanging from the world: it can drop 0.5 m along Y, the other translations are fixed; it can twist around Y
    // (the X axis of the constraint frame) by at most 0.3 rad, the swing is fixed
    const position = RVec3.init(0, 5, 0);
    const box = try context.createBox(position, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.5, 0.25, 0.25), .{});
    box.setAngularVelocity(Vec3.init(0, 2, 0));
    var settings = SixDOFConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.position1 = position;
    settings.position2 = position;
    settings.axis_x1 = Vec3.axisY();
    settings.axis_y1 = Vec3.axisX().negate();
    settings.axis_x2 = Vec3.axisY();
    settings.axis_y2 = Vec3.axisX().negate();
    settings.setLimitedAxis(.translation_x, -0.5, 0); // Constraint X is world Y
    settings.makeFixedAxis(.translation_y);
    settings.makeFixedAxis(.translation_z);
    settings.setLimitedAxis(.rotation_x, -0.3, 0.3);
    settings.makeFixedAxis(.rotation_y);
    settings.makeFixedAxis(.rotation_z);
    const constraint = try context.createConstraint(SixDOFConstraint, Body.fixedToWorld(), box, settings.asTwoBodySettingsConst());

    _ = try context.simulate(2.0, {});

    // The box rests at the lower translation limit, twisted to the rotation limit
    try checkApproxEqual(@as(f32, @floatCast(box.getPosition().getY())), 4.5, .{ .tolerance = 2.0e-3 });
    try checkApproxEqual(@as(f32, @floatCast(box.getPosition().getX())), 0, .{ .tolerance = 1.0e-3 });
    try checkApproxEqual(@as(f32, @floatCast(box.getPosition().getZ())), 0, .{ .tolerance = 1.0e-3 });
    try checkApproxEqual(box.getRotation().getRotationAngle(Vec3.axisY()), 0.3, .{ .tolerance = 1.0e-2 });
    try expect(constraint.getTotalLambdaPosition().getX() > 0.0); // The limit carries the weight
}

test "TestSixDOFFriction" {
    const allocator = std.testing.allocator;

    // A box sliding along a free axis with friction decelerates with friction / mass, until it stops
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    context.zeroGravity();
    const box = try context.createBox(RVec3.zero(), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});
    box.getMotionPropertiesMut().setLinearDamping(0.0);
    box.setLinearVelocity(Vec3.init(2, 0, 0));
    const mass = 1.0 / box.getMotionProperties().getInverseMass();
    var settings = SixDOFConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    const friction: f32 = 0.5 * mass; // 0.5 m/s^2
    settings.max_friction[0] = friction;
    _ = try context.createConstraint(SixDOFConstraint, Body.fixedToWorld(), box, settings.asTwoBodySettingsConst());

    _ = try context.simulate(1.0, {});
    try checkApproxEqual(box.getLinearVelocity().getX(), 1.5, .{ .tolerance = 1.0e-3 });
    _ = try context.simulate(4.0, {});
    try checkApproxEqual(box.getLinearVelocity().getX(), 0.0, .{ .tolerance = 1.0e-4 });
}

test "TestSixDOFPositionMotors" {
    const allocator = std.testing.allocator;

    // Position motors drive the box to a target position and orientation (rotation motors on every combination of axes
    // drive the rotation around those axes only)
    var mask: u32 = 1;
    while (mask <= 0b111) : (mask += 1) {
        var context: PhysicsTestContext = undefined;
        try context.init(allocator, .{});
        defer context.deinit();
        context.zeroGravity();
        const box = try context.createBox(RVec3.zero(), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        for (&settings.motor_settings) |*m| m.spring_settings = .init(.frequency_and_damping, 4.0, 1.0);
        const constraint = try context.createConstraint(SixDOFConstraint, Body.fixedToWorld(), box, settings.asTwoBodySettingsConst());
        constraint.setMotorState(.translation_x, .position);
        constraint.setTargetPositionCS(Vec3.init(0.5, 0, 0));
        const target = Quat.eulerAngles(Vec3.init(0.3, -0.2, 0.25));
        constraint.setTargetOrientationCS(target);
        for (0..3) |i| {
            if ((mask & (@as(u32, 1) << @intCast(i))) != 0)
                constraint.setMotorState(@enumFromInt(3 + i), .position);
        }

        _ = try context.simulate(3.0, {});
        try checkApproxEqual(@as(f32, @floatCast(box.getPosition().getX())), 0.5, .{ .tolerance = 1.0e-3 });
        if (mask == 0b111) {
            try checkApproxEqual(box.getRotation(), target, .{ .tolerance = 1.0e-3 });
        } else {
            // Rotation around the axes without a motor is not driven: the box stays close to the projection
            try expect(!box.getRotation().isClose(target, .{ .max_dist_sq = 1.0e-6 }));
            try expect(!box.getRotation().isClose(Quat.identity(), .{ .max_dist_sq = 1.0e-6 }));
        }
    }
}
