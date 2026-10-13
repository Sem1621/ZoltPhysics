//! Zolt addition, no Jolt file: simulations of ConeConstraint through PhysicsTestContext
//! Status: complete
//!
//! Jolt has no ConeConstraintTests.cpp. These tests check the physics of the constraint in a PhysicsSystem: a box that
//! swings into the cone limit and stays inside it, rotation around the twist axis that the cone does not limit, a cone
//! that narrows (SetHalfConeAngle) and pushes the body back inside, and two dynamic bodies whose twist axes are pulled
//! into the cone while the linear momentum is conserved. The class itself (binary state, conversions, solver calls,
//! allocation failures) is tested inline in Zolt/Physics/Constraints/ConeConstraint.zig, bit exactness in
//! ZoltParity/Physics/ConstraintsParity.zig and SimulationParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const ConeConstraint = zolt.ConeConstraint;
const ConeConstraintSettings = zolt.ConeConstraintSettings;
const trigonometry = zolt.trigonometry;
const math = zolt.math;

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

/// The angle between two unit vectors
fn angleBetween(a: Vec3, b: Vec3) f32 {
    return trigonometry.acos(math.clamp(a.dot(b), -1.0, 1.0));
}

test "TestConeConstraintLimit" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // A box hanging one meter below a pivot, thrown sideways: without the cone it would swing up far beyond 45 degrees
    const pivot = rvec3f(2.0, 10.0, -1.0);
    const box = try c.createBox(pivot.sub(rvec3f(0.0, 1.0, 0.0)), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.1, 0.5, 0.1), .{});
    c.getBodyInterface().setLinearVelocity(box.getID(), Vec3.init(5.0, 0.0, 0.0));
    var settings = ConeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = pivot;
    settings.point2 = pivot;
    settings.twist_axis1 = Vec3.axisY().negate();
    settings.twist_axis2 = Vec3.axisY().negate();
    settings.half_cone_angle = 0.3;
    const constraint = try c.createConstraint(ConeConstraint, Body.fixedToWorld(), box, settings.asTwoBodySettingsConst());
    try expect(constraint.asConstraint().isActive());

    var max_angle: f32 = 0.0;
    var limit_reached = false;
    for (0..120) |_| {
        _ = try c.simulateSingleStep();

        // The box stays attached to the pivot and its twist axis inside the cone (up to the softness of the solver: the
        // limit only acts once it is violated, the position steps take a few updates to correct a violation)
        try expect(distance(worldPoint(box, constraint.local_space_position2), pivot) < 5.0e-2);
        const angle = angleBetween(box.getRotation().mulVec3(constraint.local_space_twist_axis2), Vec3.axisY().negate());
        try expect(angle < 0.3 + 0.05);
        max_angle = math.max(max_angle, angle);
        if (constraint.getTotalLambdaRotation() > 0.0) limit_reached = true;
    }
    try expect(limit_reached);
    try expect(max_angle > 0.29);
}

test "TestConeConstraintTwistIsFree" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    // A box attached at its center of mass that spins around the twist axis: the cone does not limit the twist
    const position = rvec3f(0.0, 5.0, 0.0);
    const box = try c.createBox(position, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.5, 0.2, 0.3), .{});
    c.getBodyInterface().setAngularVelocity(box.getID(), Vec3.init(0.0, 3.0, 0.0));
    var settings = ConeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = position;
    settings.point2 = position;
    settings.twist_axis1 = Vec3.axisY();
    settings.twist_axis2 = Vec3.axisY();
    settings.half_cone_angle = 0.1;
    const constraint = try c.createConstraint(ConeConstraint, Body.fixedToWorld(), box, settings.asTwoBodySettingsConst());

    _ = try c.simulate(1.0, {});
    try expect(box.getAngularVelocity().isClose(Vec3.init(0.0, 3.0, 0.0), .{ .max_dist_sq = 1.0e-6 }));
    try expectApproxEqAbs(@as(f32, 0.0), constraint.getTotalLambdaRotation(), 1.0e-6);
    try expect(distance(box.getCenterOfMassPosition(), position) < 1.0e-3);
}

test "TestConeConstraintSetHalfConeAngle" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    // A box tilted by 0.5 radians inside a cone of 0.6: nothing happens; the cone narrows to 0.1: the box turns back
    const position = rvec3f(-3.0, 2.0, 4.0);
    const tilt = Quat.rotation(Vec3.axisZ(), 0.5);
    const box = try c.createBox(position, tilt, .dynamic, .discrete, Layers.moving, Vec3.init(0.2, 0.5, 0.2), .{});
    var settings = ConeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = position;
    settings.point2 = position;
    settings.twist_axis1 = Vec3.axisY();
    settings.twist_axis2 = tilt.mulVec3(Vec3.axisY());
    settings.half_cone_angle = 0.6;
    const constraint = try c.createConstraint(ConeConstraint, Body.fixedToWorld(), box, settings.asTwoBodySettingsConst());

    _ = try c.simulate(0.5, {});
    try expect(box.getRotation().isClose(tilt, .{ .max_dist_sq = 1.0e-10 }));
    try expectApproxEqAbs(@as(f32, 0.0), constraint.getTotalLambdaRotation(), 1.0e-6);

    try expect(!box.isActive()); // At rest: asleep

    // SetHalfConeAngle does not wake the body up
    constraint.setHalfConeAngle(0.1);
    try expectApproxEqAbs(trigonometry.cos(0.1), constraint.getCosHalfConeAngle(), 1.0e-7);
    _ = try c.simulate(0.1, {});
    try expect(box.getRotation().isClose(tilt, .{ .max_dist_sq = 1.0e-10 }));
    // Activated, the position steps turn it back (the correction is the Baumgarte fraction of the error in cos(angle),
    // which converges slowly near the edge of a narrow cone, and the body has no velocity: it falls asleep again once
    // the corrections no longer move it enough, just outside the cone)
    c.getBodyInterface().activateBody(box.getID());
    var previous: f32 = 0.5;
    for (0..120) |_| {
        _ = try c.simulateSingleStep();
        const angle = angleBetween(box.getRotation().mulVec3(constraint.local_space_twist_axis2), Vec3.axisY());
        try expect(angle <= previous + 1.0e-6 and angle > 0.1 - 0.005); // Monotonic, to the edge of the cone, not further
        previous = angle;
    }
    try expect(previous < 0.1 + 0.015);
    try expect(!box.isActive());
}

test "TestConeConstraintBetweenBodies" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    // Two boxes of different mass with twist axes one radian apart and a cone of 0.2 (body 2 in a layer that does not
    // collide with body 1)
    const box1 = try c.createBox(rvec3f(-1.0, 5.0, 0.0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1.0, 0.25, 0.25), .{});
    const rotation2 = Quat.rotation(Vec3.axisY(), 1.0);
    const box2 = try c.createBox(rvec3f(0.5, 5.0, 0.0), rotation2, .dynamic, .discrete, Layers.moving2, Vec3.init(0.5, 0.2, 0.2), .{});
    var settings = ConeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = rvec3f(0.0, 5.0, 0.0);
    settings.point2 = rvec3f(0.0, 5.0, 0.0);
    settings.twist_axis1 = Vec3.axisX();
    settings.twist_axis2 = rotation2.mulVec3(Vec3.axisX());
    settings.half_cone_angle = 0.2;
    const constraint = try c.createConstraint(ConeConstraint, box1, box2, settings.asTwoBodySettingsConst());
    const bi = c.getBodyInterface();
    bi.setLinearVelocity(box1.getID(), Vec3.init(0.5, 1.0, 0.0));
    const m1 = 1.0 / box1.getMotionProperties().getInverseMass();
    const m2 = 1.0 / box2.getMotionProperties().getInverseMass();
    const momentum = box1.getLinearVelocity().mulScalar(m1);

    for (0..90) |i| {
        _ = try c.simulateSingleStep();

        // The impulses of the constraint are equal and opposite: the linear momentum does not change
        const p = box1.getLinearVelocity().mulScalar(m1).add(box2.getLinearVelocity().mulScalar(m2));
        try expect(p.isClose(momentum, .{ .max_dist_sq = 1.0e-6 * momentum.lengthSq() }));

        // The attachment points separate while the cone turns the bodies by almost a radian in the first updates, then
        // come together again
        const separation = distance(worldPoint(box1, constraint.local_space_position1), worldPoint(box2, constraint.local_space_position2));
        try expect(separation < if (i < 30) @as(f32, 0.1) else 1.0e-2);
    }

    // The twist axes are (almost) inside the cone, both bodies turned
    const twist1 = box1.getRotation().mulVec3(constraint.local_space_twist_axis1);
    const twist2 = box2.getRotation().mulVec3(constraint.local_space_twist_axis2);
    try expect(angleBetween(twist1, twist2) < 0.2 + 0.02);
    try expect(!box1.getRotation().isClose(Quat.identity(), .{}));
    try expect(!box2.getRotation().isClose(rotation2, .{}));
}
