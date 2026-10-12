//! Port of: UnitTests/Physics/DistanceConstraintTests.cpp
//! Status: complete
//!
//! TestDistanceSpring is Jolt's test (same name, inputs and expected values). The Zolt additions below simulate a rope
//! of fixed length, a rope with a range (slack until the max distance is reached) and a soft range in a PhysicsSystem.
//! The class itself (binary state, conversions, setters, solver branches, allocation failures) is tested inline in
//! Zolt/Physics/Constraints/DistanceConstraint.zig, bit exactness in ZoltParity/Physics/ConstraintsParity.zig and the
//! distance scenes of ZoltParity/Physics/SimulationParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const DistanceConstraint = zolt.DistanceConstraint;
const DistanceConstraintSettings = zolt.DistanceConstraintSettings;
const math = zolt.math;

const checkApproxEqual = fw.checkApproxEqual;
const expect = std.testing.expect;

// Test if the distance constraint can be used to create a spring
test "TestDistanceSpring" {
    const allocator = std.testing.allocator;

    // Configuration of the spring
    const initial_position = RVec3.init(10, 0, 0);
    const frequency: f32 = 2.0;
    const damping: f32 = 0.1;

    for (0..2) |mode| {
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
        var constraint = DistanceConstraintSettings.init(allocator);
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
        constraint.min_distance = 0.0;
        constraint.max_distance = 0.0;
        _ = try context.createConstraint(DistanceConstraint, Body.fixedToWorld(), body, constraint.asTwoBodySettingsConst());

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
            try expect(body.getPosition().getY() == 0);
            try expect(body.getPosition().getZ() == 0);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Zolt additions (Jolt has only TestDistanceSpring): ropes in a PhysicsSystem

/// The distance between two world space positions (computed in Real, returned as float)
fn distance(a: RVec3, b: RVec3) f32 {
    return @floatCast(a.sub(b).length());
}

test "TestDistanceRope" {
    // A sphere on a rope of fixed length (min = max = -1: the distance at creation) from the world swings and keeps its
    // distance to the pivot
    const allocator = std.testing.allocator;
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    const pivot = RVec3.init(0, 10, 0);
    const start = RVec3.init(2, 8, 1);
    const sphere = try context.createSphere(start, 0.25, .dynamic, .discrete, Layers.moving, .{});

    var settings = DistanceConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = pivot;
    settings.point2 = start;
    const rope = try context.createConstraint(DistanceConstraint, Body.fixedToWorld(), sphere, settings.asTwoBodySettingsConst());
    const length = distance(start, pivot);
    try expect(rope.getMinDistance() == rope.getMaxDistance());
    try checkApproxEqual(length, rope.getMinDistance(), .{});

    var max_x: f32 = 0.0;
    var min_x: f32 = 0.0;
    for (0..180) |_| {
        _ = try context.simulateSingleStep();
        try checkApproxEqual(length, distance(sphere.getCenterOfMassPosition(), pivot), .{ .tolerance = 2.0e-3 });
        const x: f32 = @floatCast(sphere.getCenterOfMassPosition().getX());
        max_x = @max(max_x, x);
        min_x = @min(min_x, x);
    }

    // It swung to the other side, the rope pulls (negative lambda: towards body 1)
    try expect(min_x < -1.5 and max_x <= 2.0 + 1.0e-3);
    try expect(rope.getTotalLambdaPosition() < 0.0);
}

test "TestDistanceRopeRange" {
    // A sphere 1 below the pivot on a rope with a range [0.5, 3] falls freely until the rope is taut at 3, then the rope
    // holds it; SetDistance shortens the rope and pulls it up
    const allocator = std.testing.allocator;
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    const pivot = RVec3.init(0, 10, 0);
    const start = RVec3.init(0, 9, 0);
    const sphere = try context.createSphere(start, 0.25, .dynamic, .discrete, Layers.moving, .{});
    sphere.setAllowSleeping(false);

    var settings = DistanceConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = pivot;
    settings.point2 = start;
    settings.min_distance = 0.5;
    settings.max_distance = 3.0;
    const rope = try context.createConstraint(DistanceConstraint, Body.fixedToWorld(), sphere, settings.asTwoBodySettingsConst());

    // Free fall while the rope is slack (the constraint applies no impulse)
    const gravity = context.getSystem().getGravity();
    var time: f32 = 0.0;
    while (distance(sphere.getCenterOfMassPosition(), pivot) < 2.5) {
        _ = try context.simulateSingleStep();
        time += context.getDeltaTime();
        try expect(rope.getTotalLambdaPosition() == 0.0);
    }
    try checkApproxEqual(context.predictPosition(start, Vec3.zero(), gravity, time), sphere.getCenterOfMassPosition(), .{ .tolerance = 1.0e-4 });

    // Taut: held at the max distance
    _ = try context.simulate(1.0, {});
    try checkApproxEqual(@as(f32, 3.0), distance(sphere.getCenterOfMassPosition(), pivot), .{ .tolerance = 1.0e-2 });
    try expect(rope.getTotalLambdaPosition() < 0.0);

    // A shorter rope pulls it up
    rope.setDistance(0.5, 1.5);
    _ = try context.simulate(2.0, {});
    try checkApproxEqual(@as(f32, 1.5), distance(sphere.getCenterOfMassPosition(), pivot), .{ .tolerance = 1.0e-2 });
}

test "TestDistanceSoftRange" {
    // A soft max distance: the spring lets the sphere sink below the max distance (a hard limit would hold it there) and
    // the stiffer the spring the less it sinks
    const allocator = std.testing.allocator;
    var sinks: [2]f32 = undefined;
    for ([_]f32{ 1.0, 4.0 }, &sinks) |frequency, *sink| {
        var context: PhysicsTestContext = undefined;
        try context.init(allocator, .{});
        defer context.deinit();
        const pivot = RVec3.init(0, 10, 0);
        const start = RVec3.init(0, 8, 0);
        const sphere = try context.createSphere(start, 0.25, .dynamic, .discrete, Layers.moving, .{});
        sphere.setAllowSleeping(false);

        var settings = DistanceConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.point1 = pivot;
        settings.point2 = start;
        settings.min_distance = 0.0;
        settings.max_distance = 2.0;
        settings.limits_spring_settings = .init(.frequency_and_damping, frequency, 1.0);
        const rope = try context.createConstraint(DistanceConstraint, Body.fixedToWorld(), sphere, settings.asTwoBodySettingsConst());
        try expect(rope.getLimitsSpringSettings().frequency == frequency);

        _ = try context.simulate(3.0, {});
        sink.* = distance(sphere.getCenterOfMassPosition(), pivot) - 2.0;

        // The static deflection of a spring: x = g / omega^2
        const omega = 2.0 * math.pi * frequency;
        try checkApproxEqual(9.81 / math.square(omega), sink.*, .{ .tolerance = 0.1 * 9.81 / math.square(omega) });
    }
    try expect(sinks[0] > sinks[1] and sinks[1] > 0.0);
}
