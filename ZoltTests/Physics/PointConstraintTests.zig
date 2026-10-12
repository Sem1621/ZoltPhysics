//! Zolt addition, no Jolt file: simulations of PointConstraint through PhysicsTestContext
//! Status: complete
//!
//! Jolt has no PointConstraintTests.cpp (its only unit test with a PointConstraint is TestPhysicsConstraintUserData in
//! PhysicsTests.zig). These tests check the physics of the constraint in a PhysicsSystem: a pendulum on
//! Body.fixedToWorld() (Jolt's Body::sFixedToWorld) that keeps its length and swings with the period of a physical
//! pendulum, momentum conservation between two dynamic bodies, a dynamic body dragged by a kinematic one and moving
//! the pivot with setPoint1. The class itself (binary state, conversions, solver calls, allocation failures) is tested
//! inline in Zolt/Physics/Constraints/PointConstraint.zig, bit exactness in ZoltParity/Physics/ConstraintsParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const PointConstraint = zolt.PointConstraint;
const PointConstraintSettings = zolt.PointConstraintSettings;
const math = zolt.math;
const trigonometry = zolt.trigonometry;

const expect = std.testing.expect;
const expectApproxEqAbs = std.testing.expectApproxEqAbs;

/// RVec3 of float values (widened to Real in double precision)
fn rvec3f(x: f32, y: f32, z: f32) RVec3 {
    return RVec3.init(x, y, z);
}

/// The distance between two world space positions (computed in Real, returned as float)
fn distance(a: RVec3, b: RVec3) f32 {
    return @floatCast(a.sub(b).length());
}

/// The world space position of a point in the local space of the center of mass of a body
fn worldPoint(body: *const Body, local_point: Vec3) RVec3 {
    return body.getCenterOfMassTransform().mulVec3(local_point);
}

/// Measures the period of an oscillating value from the times at which it changes sign
const PeriodMeter = struct {
    prev_value: f32,
    prev_time: f32 = 0.0,
    crossings: [3]f32 = undefined,
    num_crossings: u32 = 0,

    fn add(self: *PeriodMeter, time: f32, value: f32) void {
        if ((self.prev_value > 0.0) != (value > 0.0) and self.num_crossings < self.crossings.len) {
            // Linear interpolation of the time where the value is zero
            self.crossings[self.num_crossings] = self.prev_time + (time - self.prev_time) * self.prev_value / (self.prev_value - value);
            self.num_crossings += 1;
        }
        self.prev_value = value;
        self.prev_time = time;
    }

    /// Time between the first and the third zero crossing
    fn period(self: *const PeriodMeter) ?f32 {
        return if (self.num_crossings == self.crossings.len) self.crossings[2] - self.crossings[0] else null;
    }
};

test "TestPointConstraintPendulum" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // A sphere at distance cLength below the pivot, displaced by a small angle
    const length: f32 = 2.0;
    const radius: f32 = 0.5;
    const angle: f32 = 0.1;
    const pivot = rvec3f(1.0, 10.0, -2.0);
    const start = pivot.add(rvec3f(trigonometry.sin(angle) * length, -trigonometry.cos(angle) * length, 0.0));
    const sphere = try c.createSphere(start, radius, .dynamic, .discrete, Layers.moving, .{});

    // Attach it to the world at the pivot
    var settings = PointConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = pivot;
    settings.point2 = pivot;
    const constraint = try c.createConstraint(PointConstraint, Body.fixedToWorld(), sphere, settings.asTwoBodySettingsConst());
    try expect(constraint.getLocalSpacePoint1().eql(pivot.toVec3())); // The world body has an identity transform
    try expect(constraint.asConstraint().isActive());

    // The period of a physical pendulum: 2 pi sqrt(I / (m g L)), I = 2/5 m r^2 + m L^2 the inertia around the pivot
    const g = -c.getSystem().getGravity().getY();
    const expected_period = 2.0 * math.pi * @sqrt((0.4 * radius * radius + length * length) / (g * length));

    var meter: PeriodMeter = .{ .prev_value = @floatCast(sphere.getCenterOfMassPosition().getX() - pivot.getX()) };
    var max_amplitude: f32 = 0.0;
    var time: f32 = 0.0;
    for (0..300) |_| {
        _ = try c.simulateSingleStep();
        time += c.getDeltaTime();

        // The attachment point stays at the pivot, the sphere at its distance
        try expect(distance(worldPoint(sphere, constraint.getLocalSpacePoint2()), pivot) < 1.0e-3);
        try expectApproxEqAbs(length, distance(sphere.getCenterOfMassPosition(), pivot), 1.0e-3);

        const x: f32 = @floatCast(sphere.getCenterOfMassPosition().getX() - pivot.getX());
        meter.add(time, x);
        if (@abs(x) > max_amplitude) max_amplitude = @abs(x);

        // The sphere swings in the XY plane
        try expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatCast(sphere.getCenterOfMassPosition().getZ() - pivot.getZ())), 1.0e-5);
    }

    // The energy does not grow and the period is the one of the physical pendulum
    try expect(max_amplitude <= trigonometry.sin(angle) * length + 1.0e-3);
    const period = meter.period() orelse return error.TestUnexpectedResult;
    try expectApproxEqAbs(expected_period, period, 0.005 * expected_period);
}

test "TestPointConstraintMomentum" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    // Two boxes of different mass connected at a point between them (body 2 is in a layer that does not collide with
    // body 1, the bodies touch at the constraint point)
    const box1 = try c.createBox(rvec3f(-1.0, 5.0, 0.0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1.0, 0.5, 0.5), .{});
    const box2 = try c.createBox(rvec3f(0.5, 5.0, 0.0), Quat.rotation(Vec3.axisY(), 0.3), .dynamic, .discrete, Layers.moving2, Vec3.init(0.5, 0.25, 0.25), .{});
    var settings = PointConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = rvec3f(0.0, 5.0, 0.0);
    settings.point2 = rvec3f(0.0, 5.0, 0.0);
    const constraint = try c.createConstraint(PointConstraint, box1, box2, settings.asTwoBodySettingsConst());

    // Throw box 1
    const bi = c.getBodyInterface();
    bi.setLinearAndAngularVelocity(box1.getID(), Vec3.init(2.0, 1.0, -0.5), Vec3.init(0.0, 1.0, 3.0));
    const m1 = 1.0 / box1.getMotionProperties().getInverseMass();
    const m2 = 1.0 / box2.getMotionProperties().getInverseMass();
    const momentum = box1.getLinearVelocity().mulScalar(m1);

    for (0..120) |_| {
        _ = try c.simulateSingleStep();

        // The impulses of the constraint are equal and opposite: the linear momentum does not change
        const p = box1.getLinearVelocity().mulScalar(m1).add(box2.getLinearVelocity().mulScalar(m2));
        try expect(p.isClose(momentum, .{ .max_dist_sq = 1.0e-6 * momentum.lengthSq() }));

        // The points stay together
        try expect(distance(worldPoint(box1, constraint.getLocalSpacePoint1()), worldPoint(box2, constraint.getLocalSpacePoint2())) < 1.0e-2);
    }

    // Both bodies move: the constraint transferred velocity to box 2
    try expect(!box2.getLinearVelocity().isNearZero(.{}));
    try expect(!constraint.getTotalLambdaPosition().isNearZero(.{}));
}

test "TestPointConstraintKinematicBody" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // A sphere hanging below a kinematic box that moves sideways
    const box = try c.createBox(rvec3f(0.0, 10.0, 0.0), Quat.identity(), .kinematic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});
    const sphere = try c.createSphere(rvec3f(0.0, 8.0, 0.0), 0.5, .dynamic, .discrete, Layers.moving, .{});
    var settings = PointConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.space = .local_to_body_com;
    settings.point1 = rvec3f(0.0, -0.5, 0.0);
    settings.point2 = rvec3f(0.0, 1.5, 0.0);
    const constraint = try c.createConstraint(PointConstraint, box, sphere, settings.asTwoBodySettingsConst());
    try expect(constraint.getLocalSpacePoint1().eql(Vec3.init(0.0, -0.5, 0.0)));

    const velocity = Vec3.init(1.0, 0.0, 0.5);
    c.getBodyInterface().setLinearVelocity(box.getID(), velocity);
    for (0..120) |_| {
        _ = try c.simulateSingleStep();

        // The kinematic body is not affected by the constraint, the sphere follows it
        try expect(box.getLinearVelocity().eql(velocity));
        try expect(box.getAngularVelocity().eql(Vec3.zero()));
        try expect(distance(worldPoint(box, constraint.getLocalSpacePoint1()), worldPoint(sphere, constraint.getLocalSpacePoint2())) < 1.0e-2);
    }
    try expect(box.getPosition().isClose(rvec3f(2.0, 10.0, 1.0), .{ .max_dist_sq = 1.0e-6 }));
    try expect(sphere.getPosition().getX() > 1.0);
}

test "TestPointConstraintSetPoint" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // A sphere hanging at rest below a pivot
    const sphere = try c.createSphere(rvec3f(0.0, 8.0, 0.0), 0.5, .dynamic, .discrete, Layers.moving, .{});
    var settings = PointConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = rvec3f(0.0, 10.0, 0.0);
    settings.point2 = rvec3f(0.0, 10.0, 0.0);
    const constraint = try c.createConstraint(PointConstraint, Body.fixedToWorld(), sphere, settings.asTwoBodySettingsConst());
    _ = try c.simulate(0.25, {});
    try expect(sphere.getPosition().isClose(rvec3f(0.0, 8.0, 0.0), .{ .max_dist_sq = 1.0e-6 }));

    // Move the pivot up: the position solver pulls the sphere along
    constraint.setPoint1(.world_space, rvec3f(0.0, 10.5, 0.0));
    try expect(constraint.getLocalSpacePoint1().eql(Vec3.init(0.0, 10.5, 0.0)));
    c.getBodyInterface().activateBody(sphere.getID()); // Changing a point does not wake up the body (Jolt)
    _ = try c.simulate(1.0, {});
    try expect(distance(worldPoint(sphere, constraint.getLocalSpacePoint2()), rvec3f(0.0, 10.5, 0.0)) < 1.0e-3);
    try expectApproxEqAbs(@as(f32, 0.0), @as(f32, @floatCast(sphere.getPosition().getX())), 1.0e-4);
    try expect(sphere.getPosition().getY() > 8.4);

    // Move the attachment point on the sphere in its local space (the sphere is not rotated)
    constraint.setPoint2(.local_to_body_com, rvec3f(0.0, 1.0, 0.0));
    c.getBodyInterface().activateBody(sphere.getID());
    _ = try c.simulate(1.0, {});
    try expect(distance(worldPoint(sphere, constraint.getLocalSpacePoint2()), rvec3f(0.0, 10.5, 0.0)) < 1.0e-3);
    try expect(sphere.getPosition().getY() > 9.4);
}
