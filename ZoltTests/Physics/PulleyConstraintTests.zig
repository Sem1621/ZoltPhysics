//! Zolt addition, no Jolt file: simulations of PulleyConstraint through PhysicsTestContext
//! Status: complete
//!
//! Jolt has no PulleyConstraintTests.cpp. These tests check the physics of the constraint in a PhysicsSystem: two
//! bodies on a rope over two fixed points that are in balance when the masses match the ratio, the accelerations of an
//! unbalanced block and tackle (ratio 2), a rope that goes slack (min length 0) and SetLength. The class itself
//! (binary state, conversions, setters, solver branches, allocation failures) is tested inline in
//! Zolt/Physics/Constraints/PulleyConstraint.zig, bit exactness in ZoltParity/Physics/ConstraintsParity.zig and the
//! pulley scenes of ZoltParity/Physics/SimulationParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Vec3 = zolt.Vec3;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const PulleyConstraint = zolt.PulleyConstraint;
const PulleyConstraintSettings = zolt.PulleyConstraintSettings;

const checkApproxEqual = fw.checkApproxEqual;
const expect = std.testing.expect;

/// The distance between two world space positions (computed in Real, returned as float)
fn distance(a: RVec3, b: RVec3) f32 {
    return @floatCast(a.sub(b).length());
}

/// Two spheres hanging below two fixed points on a pulley (the attachment points are the centers of mass)
const Pulley = struct {
    body1: *Body,
    body2: *Body,
    pulley: *PulleyConstraint,
    fixed1: RVec3,
    fixed2: RVec3,

    fn create(context: *PhysicsTestContext, settings: *PulleyConstraintSettings, radius1: f32, radius2: f32, ratio: f32) !Pulley {
        const fixed1 = RVec3.init(-2, 10, 0);
        const fixed2 = RVec3.init(2, 10, 0);
        const body1 = try context.createSphere(RVec3.init(-2, 7, 0), radius1, .dynamic, .discrete, Layers.moving, .{});
        const body2 = try context.createSphere(RVec3.init(2, 6, 0), radius2, .dynamic, .discrete, Layers.moving, .{});
        body1.setAllowSleeping(false);
        body2.setAllowSleeping(false);
        settings.body_point1 = body1.getCenterOfMassPosition();
        settings.fixed_point1 = fixed1;
        settings.body_point2 = body2.getCenterOfMassPosition();
        settings.fixed_point2 = fixed2;
        settings.ratio = ratio;
        const pulley = try context.createConstraint(PulleyConstraint, body1, body2, settings.asTwoBodySettingsConst());
        return .{ .body1 = body1, .body2 = body2, .pulley = pulley, .fixed1 = fixed1, .fixed2 = fixed2 };
    }

    fn length1(self: *const Pulley) f32 {
        return distance(self.body1.getCenterOfMassPosition(), self.fixed1);
    }

    fn length2(self: *const Pulley) f32 {
        return distance(self.body2.getCenterOfMassPosition(), self.fixed2);
    }
};

test "TestPulleyBalance" {
    // Equal masses with ratio 1 on a rope of fixed length: the tensions cancel gravity, nothing moves
    const allocator = std.testing.allocator;
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    var settings = PulleyConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.min_length = -1.0;
    settings.max_length = -1.0;
    const p = try Pulley.create(&context, &settings, 0.5, 0.5, 1.0);
    try checkApproxEqual(@as(f32, 3.0 + 4.0), p.pulley.getCurrentLength(), .{});
    try expect(p.pulley.getMinLength() == p.pulley.getMaxLength());

    _ = try context.simulate(1.0, {});
    try checkApproxEqual(@as(f32, 3.0), p.length1(), .{ .tolerance = 1.0e-3 });
    try checkApproxEqual(@as(f32, 4.0), p.length2(), .{ .tolerance = 1.0e-3 });

    // The tension holds the weight of one body: lambda = -m g dt per step (the impulse pulls the bodies up)
    const m = 1.0 / p.body1.getMotionProperties().getInverseMass();
    try checkApproxEqual(-m * 9.81 * context.getDeltaTime(), p.pulley.getTotalLambdaPosition(), .{ .tolerance = 0.01 * m * 9.81 * context.getDeltaTime() });
}

test "TestPulleyBlockAndTackle" {
    // Equal masses with ratio 2 (a block and tackle): L1 + 2 L2 stays the same, so body 1 sinks with 2 g / 5 and body 2
    // rises with g / 5 (Lagrange: 5 m x2'' = -m g with x1 = C - 2 x2)
    const allocator = std.testing.allocator;
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    var settings = PulleyConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.min_length = -1.0;
    settings.max_length = -1.0;
    const p = try Pulley.create(&context, &settings, 0.5, 0.5, 2.0);
    const length = p.pulley.getCurrentLength();
    try checkApproxEqual(@as(f32, 3.0 + 2.0 * 4.0), length, .{});

    const time: f32 = 0.5;
    _ = try context.simulate(time, {});
    try checkApproxEqual(length, p.length1() + 2.0 * p.length2(), .{ .tolerance = 1.0e-3 });
    const g: f32 = 9.81;
    const expected_drop = 0.5 * (2.0 * g / 5.0) * time * time;
    try checkApproxEqual(expected_drop, p.length1() - 3.0, .{ .tolerance = 0.05 * expected_drop });
    try checkApproxEqual(-0.5 * expected_drop, p.length2() - 4.0, .{ .tolerance = 0.05 * expected_drop });
}

test "TestPulleySlackRope" {
    // Jolt's default lengths (min 0, max -1: the length at creation): a rope that can go slack. Body 2 is thrown up: the
    // rope goes slack and it flies freely while body 1 falls freely as well, until the rope is taut again
    const allocator = std.testing.allocator;
    var context: PhysicsTestContext = undefined;
    try context.init(allocator, .{});
    defer context.deinit();
    var settings = PulleyConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    const p = try Pulley.create(&context, &settings, 0.5, 0.5, 1.0);
    try expect(p.pulley.getMinLength() == 0.0);
    try checkApproxEqual(@as(f32, 7.0), p.pulley.getMaxLength(), .{});

    // Throw body 2 up while body 1 hangs still: the rope is slack (shorter than the max length)
    context.getBodyInterface().setLinearVelocity(p.body2.getID(), Vec3.init(0, 5, 0));
    _ = try context.simulateSingleStep();
    _ = try context.simulateSingleStep();
    try expect(p.pulley.getCurrentLength() < p.pulley.getMaxLength());
    try expect(p.pulley.getTotalLambdaPosition() == 0.0);

    // Until the rope is taut again: never longer than the max length (+ what one step at the speed of the jerk adds
    // before the next update sees the violation, which the position solver then removes)
    var was_taut = false;
    for (0..120) |_| {
        _ = try context.simulateSingleStep();
        try expect(p.length1() + p.length2() < 7.0 + 5.0 * context.getDeltaTime());
        if (p.pulley.getTotalLambdaPosition() < 0.0) was_taut = true;
    }
    try expect(was_taut);
    try checkApproxEqual(@as(f32, 7.0), p.length1() + p.length2(), .{ .tolerance = 1.0e-2 });

    // SetLength: a shorter rope lifts both bodies
    p.pulley.setLength(0.0, 5.0);
    _ = try context.simulate(2.0, {});
    try checkApproxEqual(@as(f32, 5.0), p.length1() + p.length2(), .{ .tolerance = 2.0e-2 });
}
