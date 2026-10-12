//! Port of: UnitTests/Physics/SliderConstraintTests.cpp
//! Status: complete
//!
//! Every TEST_CASE of Jolt's SliderConstraintTests (same names, inputs and expected values). `c.CreateConstraint<T>` is
//! `c.createConstraint(T, ...)`, `GetSystem()->AddConstraint(s.Create(b1, b2))` is the same call (Create + AddConstraint,
//! the system owns the constraint). The class itself (binary state, conversions, solver calls, allocation failures) is
//! tested inline in Zolt/Physics/Constraints/SliderConstraint.zig, bit exactness in
//! ZoltParity/Physics/ConstraintsParity.zig and the slider scenes of ZoltParity/Physics/SimulationParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;
const LoggingContactListener = @import("../LoggingContactListener.zig").LoggingContactListener;

const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const MotionType = zolt.MotionType;
const MotionProperties = zolt.MotionProperties;
const MotorSettings = zolt.MotorSettings;
const CollisionGroup = zolt.CollisionGroup;
const GroupFilter = zolt.GroupFilter;
const GroupFilterTable = zolt.GroupFilterTable;
const Ref = zolt.Ref;
const SliderConstraint = zolt.SliderConstraint;
const SliderConstraintSettings = zolt.SliderConstraintSettings;
const math = zolt.math;

const expect = std.testing.expect;
const checkApproxEqual = fw.checkApproxEqual;

/// RVec3 of float values (widened to Real in double precision)
fn rvec3f(x: f32, y: f32, z: f32) RVec3 {
    return RVec3.init(x, y, z);
}

/// body.SetCollisionGroup(CollisionGroup(inFilter, inGroupID, inSubGroupID)) (the temporary releases its reference)
fn setCollisionGroup(body: *Body, filter: *GroupFilterTable, group_id: u32, sub_group_id: u32) void {
    var group = CollisionGroup.init(filter.asGroupFilter(), group_id, sub_group_id);
    defer group.deinit();
    body.setCollisionGroup(&group);
}

// Test a box attached to a slider constraint, test that the body doesn't move beyond the min limit
test "TestSliderConstraintLimitMin" {
    const allocator = std.testing.allocator;
    const initial_pos = rvec3f(3.0, 0, 0);
    const limit_min: f32 = -7.0;

    // Create group filter
    const group_filter = try GroupFilterTable.create(allocator, .{});
    var group_filter_ref = Ref(GroupFilter).init(group_filter.asGroupFilterMut());
    defer group_filter_ref.deinit();

    // Create two boxes
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const body1 = try c.createBox(RVec3.zero(), Quat.identity(), .static, .discrete, Layers.non_moving, Vec3.init(1, 1, 1), .{});
    const body2 = try c.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});

    // Give body 2 velocity towards min limit (and ensure that it arrives well before 1 second)
    body2.setLinearVelocity(Vec3.init(10.0, 0, 0).negate());

    // Bodies will go through each other, make sure they don't collide
    setCollisionGroup(body1, group_filter, 0, 0);
    setCollisionGroup(body2, group_filter, 0, 0);

    // Create slider constraint
    var s = SliderConstraintSettings.init(allocator);
    defer s.asSettings().deinit();
    s.auto_detect_point = true;
    s.setSliderAxis(Vec3.axisX());
    s.limits_min = limit_min;
    s.limits_max = 0.0;
    _ = try c.createConstraint(SliderConstraint, body1, body2, s.asTwoBodySettingsConst());

    // Simulate
    _ = try c.simulate(1.0, {});

    // Test resulting velocity
    try checkApproxEqual(Vec3.zero(), body2.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Test resulting position
    try checkApproxEqual(initial_pos.addVec3(s.slider_axis1.mulScalar(limit_min)), body2.getPosition(), .{ .tolerance = 1.0e-4 });
}

// Test a box attached to a slider constraint, test that the body doesn't move beyond the max limit
test "TestSliderConstraintLimitMax" {
    const allocator = std.testing.allocator;
    const initial_pos = rvec3f(3.0, 0, 0);
    const limit_max: f32 = 7.0;

    // Create two boxes
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const body1 = try c.createBox(RVec3.zero(), Quat.identity(), .static, .discrete, Layers.non_moving, Vec3.init(1, 1, 1), .{});
    const body2 = try c.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});

    // Give body 2 velocity towards max limit (and ensure that it arrives well before 1 second)
    body2.setLinearVelocity(Vec3.init(10.0, 0, 0));

    // Create slider constraint
    var s = SliderConstraintSettings.init(allocator);
    defer s.asSettings().deinit();
    s.auto_detect_point = true;
    s.setSliderAxis(Vec3.axisX());
    s.limits_min = 0.0;
    s.limits_max = limit_max;
    _ = try c.createConstraint(SliderConstraint, body1, body2, s.asTwoBodySettingsConst());

    // Simulate
    _ = try c.simulate(1.0, {});

    // Test resulting velocity
    try checkApproxEqual(Vec3.zero(), body2.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Test resulting position
    try checkApproxEqual(initial_pos.addVec3(s.slider_axis1.mulScalar(limit_max)), body2.getPosition(), .{ .tolerance = 1.0e-4 });
}

// Test a box attached to a slider constraint, test that a motor can drive it to a specific velocity
test "TestSliderConstraintDriveVelocityStaticVsDynamic" {
    const allocator = std.testing.allocator;
    const initial_pos = rvec3f(3.0, 0, 0);
    const motor_acceleration: f32 = 2.0;

    // Create two boxes
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const body1 = try c.createBox(RVec3.zero(), Quat.identity(), .static, .discrete, Layers.non_moving, Vec3.init(1, 1, 1), .{});
    const body2 = try c.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});

    // Create slider constraint
    var s = SliderConstraintSettings.init(allocator);
    defer s.asSettings().deinit();
    s.auto_detect_point = true;
    s.setSliderAxis(Vec3.axisX());
    const mass = comptime math.cubed(@as(f32, 2.0)) * 1000.0; // Density * Volume
    s.motor_settings = MotorSettings.initLimits(0.0, 0.0, mass * motor_acceleration, 0.0);
    const constraint = try c.createConstraint(SliderConstraint, body1, body2, s.asTwoBodySettingsConst());
    constraint.setMotorState(.velocity);
    constraint.setTargetVelocity(1.5 * motor_acceleration);

    // Simulate
    _ = try c.simulate(1.0, {});

    // Test resulting velocity
    var expected_vel = s.slider_axis1.mulScalar(motor_acceleration);
    try checkApproxEqual(expected_vel, body2.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Simulate (after 0.5 seconds it should reach the target velocity)
    _ = try c.simulate(1.0, {});

    // Test resulting velocity
    expected_vel = s.slider_axis1.mulScalar(1.5 * motor_acceleration);
    try checkApproxEqual(expected_vel, body2.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Test resulting position (1.5s of acceleration + 0.5s of constant speed)
    const expected_pos = c.predictPosition(initial_pos, Vec3.zero(), s.slider_axis1.mulScalar(motor_acceleration), 1.5).addVec3(expected_vel.mulScalar(0.5));
    try checkApproxEqual(expected_pos, body2.getPosition(), .{ .tolerance = 1.0e-4 });
}

// Test 2 dynamic boxes attached to a slider constraint, test that a motor can drive it to a specific velocity
test "TestSliderConstraintDriveVelocityDynamicVsDynamic" {
    const allocator = std.testing.allocator;
    const initial_pos = rvec3f(3.0, 0, 0);
    const motor_acceleration: f32 = 2.0;

    // Create two boxes
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();
    const body1 = try c.createBox(RVec3.zero(), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});
    const body2 = try c.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});

    // Create slider constraint
    var s = SliderConstraintSettings.init(allocator);
    defer s.asSettings().deinit();
    s.auto_detect_point = true;
    s.setSliderAxis(Vec3.axisX());
    const mass = comptime math.cubed(@as(f32, 2.0)) * 1000.0; // Density * Volume
    s.motor_settings = MotorSettings.initLimits(0.0, 0.0, mass * motor_acceleration, 0.0);
    const constraint = try c.createConstraint(SliderConstraint, body1, body2, s.asTwoBodySettingsConst());
    constraint.setMotorState(.velocity);
    constraint.setTargetVelocity(3.0 * motor_acceleration);

    // Simulate
    _ = try c.simulate(1.0, {});

    // Test resulting velocity (both boxes move in opposite directions with the same force, so the resulting velocity difference is 2x as big as the previous test)
    var expected_vel = s.slider_axis1.mulScalar(motor_acceleration);
    try checkApproxEqual(expected_vel.negate(), body1.getLinearVelocity(), .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(expected_vel, body2.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Simulate (after 0.5 seconds it should reach the target velocity)
    _ = try c.simulate(1.0, {});

    // Test resulting velocity
    expected_vel = s.slider_axis1.mulScalar(1.5 * motor_acceleration);
    try checkApproxEqual(expected_vel.negate(), body1.getLinearVelocity(), .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(expected_vel, body2.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Test resulting position (1.5s of acceleration + 0.5s of constant speed)
    const expected_pos1 = c.predictPosition(RVec3.zero(), Vec3.zero(), s.slider_axis1.mulScalar(-motor_acceleration), 1.5).subVec3(expected_vel.mulScalar(0.5));
    const expected_pos2 = c.predictPosition(initial_pos, Vec3.zero(), s.slider_axis1.mulScalar(motor_acceleration), 1.5).addVec3(expected_vel.mulScalar(0.5));
    try checkApproxEqual(expected_pos1, body1.getPosition(), .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(expected_pos2, body2.getPosition(), .{ .tolerance = 1.0e-4 });
}

// Test a box attached to a slider constraint, test that a motor can drive it to a specific position
test "TestSliderConstraintDrivePosition" {
    const allocator = std.testing.allocator;
    const initial_pos = rvec3f(3.0, 0, 0);
    const motor_pos = rvec3f(10.0, 0, 0);

    // Create two boxes
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const body1 = try c.createBox(RVec3.zero(), Quat.identity(), .static, .discrete, Layers.non_moving, Vec3.init(1, 1, 1), .{});
    const body2 = try c.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});

    // Create slider constraint
    var s = SliderConstraintSettings.init(allocator);
    defer s.asSettings().deinit();
    s.auto_detect_point = true;
    s.setSliderAxis(Vec3.axisX());
    const constraint = try c.createConstraint(SliderConstraint, body1, body2, s.asTwoBodySettingsConst());
    constraint.setMotorState(.position);
    constraint.setTargetPosition(motor_pos.sub(initial_pos).toVec3().dot(s.slider_axis1));

    // Simulate
    _ = try c.simulate(2.0, {});

    // Test resulting velocity
    try checkApproxEqual(Vec3.zero(), body2.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Test resulting position
    try checkApproxEqual(motor_pos, body2.getPosition(), .{ .tolerance = 1.0e-4 });
}

// Test a box attached to a slider constraint, give it initial velocity and test that the friction provides the correct deceleration
test "TestSliderConstraintFriction" {
    const allocator = std.testing.allocator;
    const initial_pos = rvec3f(3.0, 0, 0);
    const initial_velocity = Vec3.init(10.0, 0, 0);
    const friction_acceleration: f32 = 2.0;
    const simulation_time: f32 = 2.0;

    // Create two boxes
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const body1 = try c.createBox(RVec3.zero(), Quat.identity(), .static, .discrete, Layers.non_moving, Vec3.init(1, 1, 1), .{});
    const body2 = try c.createBox(initial_pos, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{});
    body2.setLinearVelocity(initial_velocity);

    // Create slider constraint
    var s = SliderConstraintSettings.init(allocator);
    defer s.asSettings().deinit();
    s.auto_detect_point = true;
    s.setSliderAxis(Vec3.axisX());
    const mass = comptime math.cubed(@as(f32, 2.0)) * 1000.0; // Density * Volume
    s.max_friction_force = mass * friction_acceleration;
    _ = try c.createConstraint(SliderConstraint, body1, body2, s.asTwoBodySettingsConst());

    // Simulate while applying friction
    _ = try c.simulate(simulation_time, {});

    // Test resulting velocity
    const expected_vel = initial_velocity.sub(s.slider_axis1.mulScalar(friction_acceleration * simulation_time));
    try checkApproxEqual(expected_vel, body2.getLinearVelocity(), .{ .tolerance = 1.0e-4 });

    // Test resulting position
    const expected_pos = c.predictPosition(initial_pos, initial_velocity, s.slider_axis1.mulScalar(-friction_acceleration), simulation_time);
    try checkApproxEqual(expected_pos, body2.getPosition(), .{ .tolerance = 1.0e-4 });
}

/// The bodies of the activation tests: two boxes far away enough so they are not touching, not activated, connected
/// with a slider constraint
fn createSliderPair(c: *PhysicsTestContext, allocator: std.mem.Allocator, motion_type1: MotionType, layer1: zolt.ObjectLayer, motion_type2: MotionType) ![2]*Body {
    // Create two boxes far away enough so they are not touching
    const body1 = try c.createBox(RVec3.zero(), Quat.identity(), motion_type1, .discrete, layer1, Vec3.init(1, 1, 1), .{ .activation = .dont_activate });
    const body2 = try c.createBox(rvec3f(10, 0, 0), Quat.identity(), motion_type2, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{ .activation = .dont_activate });

    // Create slider constraint
    var s = SliderConstraintSettings.init(allocator);
    defer s.asSettings().deinit();
    s.auto_detect_point = true;
    s.setSliderAxis(Vec3.axisX());
    _ = try c.createConstraint(SliderConstraint, body1, body2, s.asTwoBodySettingsConst());
    return .{ body1, body2 };
}

// Test if a slider constraint wakes up connected bodies
test "TestSliderStaticVsKinematic" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bodies = try createSliderPair(&c, allocator, .static, Layers.non_moving, .kinematic);
    const body1 = bodies[0];
    const body2 = bodies[1];

    // Verify they're not active
    try expect(!body1.isActive());
    try expect(!body2.isActive());

    // After a physics step, the bodies should still not be active
    _ = try c.simulateSingleStep();
    try expect(!body1.isActive());
    try expect(!body2.isActive());

    // Activate the kinematic body
    c.getSystem().getBodyInterfaceMut().activateBody(body2.getID());
    try expect(!body1.isActive());
    try expect(body2.isActive());

    // The static body should not become active (it can't)
    _ = try c.simulateSingleStep();
    try expect(!body1.isActive());
    try expect(body2.isActive());
}

// Test if a slider constraint wakes up connected bodies
test "TestSliderStaticVsDynamic" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bodies = try createSliderPair(&c, allocator, .static, Layers.non_moving, .dynamic);
    const body1 = bodies[0];
    const body2 = bodies[1];

    // Verify they're not active
    try expect(!body1.isActive());
    try expect(!body2.isActive());

    // After a physics step, the bodies should still not be active
    _ = try c.simulateSingleStep();
    try expect(!body1.isActive());
    try expect(!body2.isActive());

    // Activate the dynamic body
    c.getSystem().getBodyInterfaceMut().activateBody(body2.getID());
    try expect(!body1.isActive());
    try expect(body2.isActive());

    // The static body should not become active (it can't)
    _ = try c.simulateSingleStep();
    try expect(!body1.isActive());
    try expect(body2.isActive());
}

// Test if a slider constraint wakes up connected bodies
test "TestSliderKinematicVsDynamic" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bodies = try createSliderPair(&c, allocator, .kinematic, Layers.moving, .dynamic);
    const body1 = bodies[0];
    const body2 = bodies[1];

    // Verify they're not active
    try expect(!body1.isActive());
    try expect(!body2.isActive());

    // After a physics step, the bodies should still not be active
    _ = try c.simulateSingleStep();
    try expect(!body1.isActive());
    try expect(!body2.isActive());

    // Activate the keyframed body
    c.getSystem().getBodyInterfaceMut().activateBody(body1.getID());
    try expect(body1.isActive());
    try expect(!body2.isActive());

    // After a physics step, both bodies should be active now
    _ = try c.simulateSingleStep();
    try expect(body1.isActive());
    try expect(body2.isActive());
}

// Test if a slider constraint wakes up connected bodies
test "TestSliderKinematicVsKinematic" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bodies = try createSliderPair(&c, allocator, .kinematic, Layers.moving, .kinematic);
    const body1 = bodies[0];
    const body2 = bodies[1];

    // Verify they're not active
    try expect(!body1.isActive());
    try expect(!body2.isActive());

    // After a physics step, the bodies should still not be active
    _ = try c.simulateSingleStep();
    try expect(!body1.isActive());
    try expect(!body2.isActive());

    // Activate the first keyframed body
    c.getSystem().getBodyInterfaceMut().activateBody(body1.getID());
    try expect(body1.isActive());
    try expect(!body2.isActive());

    // After a physics step, the second keyframed body should not be woken up
    _ = try c.simulateSingleStep();
    try expect(body1.isActive());
    try expect(!body2.isActive());
}

// Test if a slider constraint wakes up connected bodies
test "TestSliderDynamicVsDynamic" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bodies = try createSliderPair(&c, allocator, .dynamic, Layers.moving, .dynamic);
    const body1 = bodies[0];
    const body2 = bodies[1];

    // Verify they're not active
    try expect(!body1.isActive());
    try expect(!body2.isActive());

    // After a physics step, the bodies should still not be active
    _ = try c.simulateSingleStep();
    try expect(!body1.isActive());
    try expect(!body2.isActive());

    // Activate the first dynamic body
    c.getSystem().getBodyInterfaceMut().activateBody(body1.getID());
    try expect(body1.isActive());
    try expect(!body2.isActive());

    // After a physics step, both bodies should be active now
    _ = try c.simulateSingleStep();
    try expect(body1.isActive());
    try expect(body2.isActive());
}

// Test that when a reference frame is provided, the slider constraint is correctly constructed
test "TestSliderReferenceFrame" {
    const allocator = std.testing.allocator;

    // Create two boxes in semi random position/orientation
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const body1 = try c.createBox(rvec3f(1, 2, 3), Quat.rotation(Vec3.init(1, 1, 1).normalized(), 0.1 * math.pi), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{ .activation = .activate });
    const body2 = try c.createBox(rvec3f(-3, -2, -1), Quat.rotation(Vec3.init(1, 0, 1).normalized(), 0.2 * math.pi), .dynamic, .discrete, Layers.moving, Vec3.init(1, 1, 1), .{ .activation = .activate });

    // Disable collision between the boxes
    const group_filter = try GroupFilterTable.create(allocator, .{ .num_sub_groups = 2 });
    var group_filter_ref = Ref(GroupFilter).init(group_filter.asGroupFilterMut()); // Jolt: the bodies' collision groups own it
    defer group_filter_ref.deinit();
    group_filter.disableCollision(0, 1);
    setCollisionGroup(body1, group_filter, 0, 0);
    setCollisionGroup(body2, group_filter, 0, 1);

    // Get their transforms
    var t1 = body1.getCenterOfMassTransform();
    var t2 = body2.getCenterOfMassTransform();

    // Create slider constraint so that slider connects the bodies at their center of mass and rotated XY -> YZ
    var s = SliderConstraintSettings.init(allocator);
    defer s.asSettings().deinit();
    s.point1 = t1.getTranslation();
    s.slider_axis1 = t1.getColumn3(0);
    s.normal_axis1 = t1.getColumn3(1);
    s.point2 = t2.getTranslation();
    s.slider_axis2 = t2.getColumn3(1);
    s.normal_axis2 = t2.getColumn3(2);
    const constraint = try c.createConstraint(SliderConstraint, body1, body2, s.asTwoBodySettingsConst());

    // Activate the motor to drive to 0
    constraint.setMotorState(.position);
    constraint.setTargetPosition(0);

    // Simulate for a second
    _ = try c.simulate(1.0, {});

    // Now the bodies should have aligned so their COM is at the same position and they're rotated XY -> YZ
    t1 = body1.getCenterOfMassTransform();
    t2 = body2.getCenterOfMassTransform();
    try checkApproxEqual(t1.getColumn3(0), t2.getColumn3(1), .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(t1.getColumn3(1), t2.getColumn3(2), .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(t1.getColumn3(2), t2.getColumn3(0), .{ .tolerance = 1.0e-4 });
    try checkApproxEqual(t1.getTranslation(), t2.getTranslation(), .{ .tolerance = 1.0e-2 });
}

// Test if the slider constraint can be used to create a spring
test "TestSliderSpring" {
    const allocator = std.testing.allocator;

    // Configuration of the spring
    const initial_position = rvec3f(10, 0, 0);
    const frequency: f32 = 2.0;
    const damping: f32 = 0.1;

    for (0..2) |mode| {
        // Create a sphere
        var context: PhysicsTestContext = undefined;
        try context.init(allocator, .{});
        defer context.deinit();
        const body = try context.createSphere(initial_position, 0.5, .dynamic, .discrete, Layers.moving, .{});
        body.getMotionPropertiesMut().setLinearDamping(0.0);

        // Calculate stiffness and damping of spring
        const m = 1.0 / body.getMotionProperties().getInverseMass();
        const omega = 2.0 * math.pi * frequency;
        const k = m * math.square(omega);
        const c = 2.0 * m * damping * omega;

        // Create spring
        var constraint = SliderConstraintSettings.init(allocator);
        defer constraint.asSettings().deinit();
        constraint.point2 = initial_position;
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
        _ = try context.createConstraint(SliderConstraint, Body.fixedToWorld(), body, constraint.asTwoBodySettingsConst());

        // Simulate spring
        var x: Real = initial_position.getX();
        var v: f32 = 0.0;
        const dt = context.getDeltaTime();
        for (0..120) |_| {
            // Using the equations from page 32 of Soft Constraints: Reinventing The Spring - Erin Catto - GDC 2011 for an implicit euler spring damper
            v = (v - dt * k / m * @as(f32, @floatCast(x))) / (1.0 + dt * c / m + math.square(dt) * k / m);
            x += v * dt;

            // Run physics simulation
            _ = try context.simulateSingleStep();

            // Test if simulation matches prediction
            try checkApproxEqual(x, body.getPosition().getX(), .{ .tolerance = 5.0e-6 });
            try checkApproxEqual(body.getPosition().getY(), 0, .{});
            try checkApproxEqual(body.getPosition().getZ(), 0, .{});
        }
    }
}

// Tests that constrained dynamic bodies are put in the same simulation island and that kinematic and static bodies are not
test "TestConstraintsVsSimulationIslands" {
    const allocator = std.testing.allocator;
    const motion_types = [_]MotionType{ .static, .kinematic, .dynamic };
    for (motion_types) |m1| {
        for (motion_types) |m2| {
            for (motion_types) |m3| {
                // Create 3 boxes with different motion types that are apart
                var c: PhysicsTestContext = undefined;
                try c.init(allocator, .{});
                defer c.deinit();
                const b1 = try c.createBox(rvec3f(0, 0, 0), Quat.identity(), m1, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });
                const b2 = try c.createBox(rvec3f(5, 0, 0), Quat.identity(), m2, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });
                const b3 = try c.createBox(rvec3f(10, 0, 0), Quat.identity(), m3, .discrete, Layers.moving, Vec3.one(), .{ .activation = .activate });

                // Constrain b1 to b2 and b2 to b3
                var s = SliderConstraintSettings.init(allocator);
                defer s.asSettings().deinit();
                s.auto_detect_point = true;
                _ = try c.createConstraint(SliderConstraint, b1, b2, s.asTwoBodySettingsConst()); // AddConstraint(s.Create(b1, b2))
                _ = try c.createConstraint(SliderConstraint, b2, b3, s.asTwoBodySettingsConst()); // AddConstraint(s.Create(b2, b3))

                var listener: LoggingContactListener = .init(allocator);
                defer listener.deinit();
                c.getSystem().setContactListener(.init(&listener));

                // Simulate a step to make sure we properly create simulation islands with different motion types in them
                _ = try c.simulateSingleStep();
                try listener.checkError();

                // Nothing should have collided
                try expect(listener.getEntryCount() == 0);

                const getIslandIndex = struct {
                    fn f(body: *const Body) u32 {
                        return if (body.isStatic()) MotionProperties.inactive_index else body.getMotionProperties().getIslandIndexInternal();
                    }
                }.f;

                const island_index1 = getIslandIndex(b1);
                const island_index2 = getIslandIndex(b2);
                const island_index3 = getIslandIndex(b3);

                if (m1 == .dynamic and m2 == .dynamic) {
                    // Two dynamic bodies should always be in the same island
                    try expect(island_index1 != MotionProperties.inactive_index);
                    try expect(island_index1 == island_index2);
                } else if (m1 == .dynamic or m1 == .kinematic) {
                    // A kinematic body or a dynamic body that is touching a kinematic/static body should be in an island of its own
                    try expect(island_index1 != MotionProperties.inactive_index);
                    try expect(island_index1 != island_index2);
                    try expect(island_index1 != island_index3);
                } else {
                    // Static bodies should not be in an island
                    try expect(island_index1 == MotionProperties.inactive_index);
                }

                if (m3 == .dynamic and m2 == .dynamic) {
                    // Two dynamic bodies should always be in the same island
                    try expect(island_index3 != MotionProperties.inactive_index);
                    try expect(island_index2 == island_index3);
                } else if (m3 == .dynamic or m3 == .kinematic) {
                    // A kinematic body or a dynamic body that is touching a kinematic/static body should be in an island of its own
                    try expect(island_index3 != MotionProperties.inactive_index);
                    try expect(island_index3 != island_index2);
                    try expect(island_index3 != island_index1);
                } else {
                    // Static bodies should not be in an island
                    try expect(island_index3 == MotionProperties.inactive_index);
                }
            }
        }
    }
}
