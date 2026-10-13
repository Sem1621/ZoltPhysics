//! Port of: UnitTests/Physics/PathConstraintTests.cpp
//! Status: complete
//!
//! TestPathConstraintPathHermite is Jolt's test with its names and values. The other tests are Zolt additions that
//! check the physics of PathConstraint in a PhysicsSystem (Jolt has no simulation test of the class): a box driven
//! around a looping circle by a velocity motor that stays on the path with its rotation following it, a box sliding
//! down an open slope under gravity until the limit at the end of the path stops it, friction that holds a box on the
//! slope, a position motor that drives a box to a target fraction, and the momentum of two dynamic bodies. The class
//! itself (binary state, the path frames, solver calls, allocation failures) is tested inline in
//! Zolt/Physics/Constraints/PathConstraint.zig and PathConstraintPathHermite.zig, bit exactness in
//! ZoltParity/Physics/ConstraintsParity.zig and SimulationParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const Core = zolt.Core;
const PhysicsScene = zolt.PhysicsScene;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const PathConstraint = zolt.PathConstraint;
const PathConstraintSettings = zolt.PathConstraintSettings;
const PathConstraintPath = zolt.PathConstraintPath;
const PathConstraintPathHermite = zolt.PathConstraintPathHermite;
const math = zolt.math;
const trigonometry = zolt.trigonometry;

const expect = std.testing.expect;
const expectApproxEqAbs = std.testing.expectApproxEqAbs;
const checkApproxEqual = fw.checkApproxEqual;

// Test a straight line using a hermite spline.
test "TestPathConstraintPathHermite" {
    const allocator = std.testing.allocator;

    // A straight spline
    // This has e.g. for t = 0.1 a local minimum at 0.7 which breaks the Newton Raphson root finding if not doing the bisection algorithm first.
    const p1 = Vec3.init(1424.96313, 468.565399, 483.655975);
    const t1 = Vec3.init(61.4222832, 42.8926392, -1.70530257e-13);
    const n1 = Vec3.init(0, 0, 1);
    const p2 = Vec3.init(1445.20105, 482.364319, 483.655975);
    const t2 = Vec3.init(20.2380009, 13.7989082, -5.68434189e-14);
    const n2 = Vec3.init(0, 0, 1);

    // Construct path
    const hermite = try PathConstraintPathHermite.create(allocator);
    var path = Ref(PathConstraintPath).init(hermite.asPath());
    defer path.deinit();
    try hermite.addPoint(p1, t1, n1);
    try hermite.addPoint(p2, t2, n2);

    // Test that positions before and after the line return 0 and 1
    const before_start = path.get().?.getClosestPoint(p1.sub(t1.mulScalar(0.01)), 0.0);
    try expect(before_start == 0.0);
    const after_end = path.get().?.getClosestPoint(p2.add(t2.mulScalar(0.01)), 0.0);
    try expect(after_end == 1.0);

    for (0..11) |i| {
        // Get point on the curve
        const fraction = 0.1 * @as(f32, @floatFromInt(i));
        const point = path.get().?.getPointOnPath(fraction);

        // Let the path determine the fraction of the closest point
        const closest_fraction = path.get().?.getClosestPoint(point.position, 0.0);

        // Validate that it is equal to what we put in
        try checkApproxEqual(fraction, closest_fraction, .{ .tolerance = 1.0e-4 });
    }
}

/// A Hermite path in the XZ plane with normal Y: a circle of `num` points around the origin (looping) or an arc of a
/// quarter circle
fn makeArc(allocator: std.mem.Allocator, num: u32, radius: f32, looping: bool) !RefConst(PathConstraintPath) {
    const hermite = try PathConstraintPathHermite.create(allocator);
    var path = RefConst(PathConstraintPath).init(hermite.asPath());
    errdefer path.deinit();
    hermite.base.setIsLooping(looping);
    const arc: f32 = if (looping) 2.0 * math.pi else 0.5 * math.pi;
    const segments: f32 = @floatFromInt(if (looping) num else num - 1);
    for (0..num) |p| {
        const a = arc * @as(f32, @floatFromInt(p)) / segments;
        const sin_cos = Vec4.replicate(a).sinCos();
        const s = sin_cos.sin.getX();
        const c = sin_cos.cos.getX();
        try hermite.addPoint(Vec3.init(radius * c, 0, radius * s), Vec3.init(-s, 0, c).mulScalar(radius * arc / segments), Vec3.axisY());
    }
    return path;
}

/// A straight path from `start` to `end` with `num` points (normal: `normal`)
fn makeLine(allocator: std.mem.Allocator, start: Vec3, end: Vec3, num: u32, normal: Vec3) !RefConst(PathConstraintPath) {
    const hermite = try PathConstraintPathHermite.create(allocator);
    var path = RefConst(PathConstraintPath).init(hermite.asPath());
    errdefer path.deinit();
    const segments: f32 = @floatFromInt(num - 1);
    const tangent = end.sub(start).divScalar(segments);
    for (0..num) |p|
        try hermite.addPoint(start.add(tangent.mulScalar(@floatFromInt(p))), tangent, normal);
    return path;
}

/// The world space position of the attachment point of body 2 (the origin of its constraint frame)
fn attachment(c: *const PathConstraint) RVec3 {
    const tb = c.asTwoBodyConstraintConst();
    return tb.getBody2().getCenterOfMassTransform().mulVec3(tb.getConstraintToBody2Matrix().getTranslation());
}

/// The distance of the attachment point of body 2 from the path (body 1 is the world)
fn distanceFromPath(c: *const PathConstraint) f32 {
    const path = c.getPath().?;
    const p = attachment(c).toVec3();
    return p.sub(path.getPointOnPath(path.getClosestPoint(p, 0.0)).position).length();
}

test "TestPathConstraintVelocityMotorOnLoopingPath" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    // A box on a circle of radius 5 (16 points), driven by a velocity motor with its rotation constrained to the path
    var path = try makeArc(allocator, 16, 5.0, true);
    defer path.deinit();
    const box = try c.createBox(RVec3.init(5, 0, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.2, 0.2, 0.2), .{});
    var settings = PathConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.path.set(path.get());
    settings.rotation_constraint_type = .constrain_to_path;
    const constraint = try c.createConstraint(PathConstraint, Body.fixedToWorld(), box, settings.asTwoBodySettingsConst());
    constraint.setPositionMotorState(.velocity);
    constraint.setTargetVelocity(4.0); // Along the tangent in m/s
    try expect(constraint.asConstraint().isActive());

    var prev_fraction = constraint.getPathFraction();
    var total_fraction: f32 = 0.0;
    for (0..120) |_| {
        _ = try c.simulateSingleStep();

        // The box stays on the path and its X axis follows the tangent (ConstrainToPath: the frame of body 2 is the
        // tangent, binormal, normal frame of the path)
        try expect(distanceFromPath(constraint) < 2.0e-2);
        const point = path.get().?.getPointOnPath(constraint.getPathFraction());
        const frame = box.getCenterOfMassTransform().getRotation().mul(constraint.asTwoBodyConstraintConst().getConstraintToBody2Matrix());
        try expect(frame.getAxisX().dot(point.tangent) > 0.99);
        try expect(frame.getAxisZ().dot(point.normal) > 0.99);

        // The fraction moves forward around the loop
        var step = constraint.getPathFraction() - prev_fraction;
        if (step < -8.0) step += 16.0;
        try expect(step >= 0.0);
        total_fraction += step;
        prev_fraction = constraint.getPathFraction();
    }

    // The speed along the path reached the target
    try expectApproxEqAbs(@as(f32, 4.0), box.getLinearVelocity().length(), 0.05);
    const expected_fraction = 2.0 * 4.0 / (2.0 * math.pi * 5.0) * 16.0; // Two seconds at 4 m/s on a circle of 16 points
    try expect(total_fraction > 0.9 * expected_fraction and total_fraction < 1.05 * expected_fraction);
}

test "TestPathConstraintLimitsAndFriction" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // A slope of 30 degrees down along X: 10 m long, 6 points
    const start = Vec3.init(0, 10, 0);
    const end = start.add(Vec3.init(10.0 * trigonometry.cos(math.degreesToRadians(30.0)), -10.0 * trigonometry.sin(math.degreesToRadians(30.0)), 0));
    var path = try makeLine(allocator, start, end, 6, Vec3.axisZ());
    defer path.deinit();
    const max_fraction = path.get().?.getPathMaxFraction();
    try std.testing.expectEqual(@as(f32, 5.0), max_fraction);

    // Box 1 slides without friction and stops at the end of the path; box 2 has enough friction to stay
    var settings = PathConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.path.set(path.get());
    settings.path_fraction = 1.0;
    settings.rotation_constraint_type = .fully_constrained;
    const box1 = try c.createBox(RVec3.fromVec3(path.get().?.getPointOnPath(1.0).position), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.2, 0.2, 0.2), .{});
    const slider = try c.createConstraint(PathConstraint, Body.fixedToWorld(), box1, settings.asTwoBodySettingsConst());
    const mass = 1.0 / box1.getMotionProperties().getInverseMass();
    settings.path_position = Vec3.init(0, 0, 2);
    settings.max_friction_force = 2.0 * mass * 9.81 * trigonometry.sin(math.degreesToRadians(30.0)); // More than gravity along the slope
    const box2 = try c.createBox(RVec3.fromVec3(path.get().?.getPointOnPath(1.0).position.add(Vec3.init(0, 0, 2))), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.2, 0.2, 0.2), .{});
    const stuck = try c.createConstraint(PathConstraint, Body.fixedToWorld(), box2, settings.asTwoBodySettingsConst());

    _ = try c.simulate(3.0, {});

    // Box 1 is at the end of the path, held by the limit (the limit lambda pushes it back up the slope)
    try std.testing.expectEqual(max_fraction, slider.getPathFraction());
    try expect(slider.getTotalLambdaPositionLimits() < 0.0);
    try expect(distanceFromPath(slider) < 1.0e-2);
    try expect(box1.getLinearVelocity().length() < 0.1);
    try checkApproxEqual(Quat.identity(), box1.getRotation(), .{ .tolerance = 1.0e-3 }); // Fully constrained

    // Box 2 did not move along the slope, friction balances gravity (the impulse points up the slope, against the path)
    try expectApproxEqAbs(@as(f32, 1.0), stuck.getPathFraction(), 1.0e-3);
    try expectApproxEqAbs(mass * 9.81 * trigonometry.sin(math.degreesToRadians(30.0)) * c.getDeltaTime(), -stuck.getTotalLambdaMotor(), 0.05 * mass * c.getDeltaTime());
}

test "TestPathConstraintPositionMotor" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    // A quarter circle arc, a stiff position motor drives the box from fraction 0.5 to 2.5
    var path = try makeArc(allocator, 4, 3.0, false);
    defer path.deinit();
    var settings = PathConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.path.set(path.get());
    settings.path_fraction = 0.5;
    settings.rotation_constraint_type = .constrain_around_normal;
    settings.position_motor_settings = .init(2.0, 1.0);
    const box = try c.createBox(RVec3.fromVec3(path.get().?.getPointOnPath(0.5).position), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.2, 0.2, 0.2), .{});
    const constraint = try c.createConstraint(PathConstraint, Body.fixedToWorld(), box, settings.asTwoBodySettingsConst());
    constraint.setPositionMotorState(.position);
    constraint.setTargetPathFraction(2.5);

    _ = try c.simulate(5.0, {});
    try expectApproxEqAbs(@as(f32, 2.5), constraint.getPathFraction(), 1.0e-2);
    try expect(distanceFromPath(constraint) < 1.0e-2);
    try expect(box.getLinearVelocity().length() < 0.05);
}

test "TestPathConstraintMomentum" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    // Two dynamic boxes: the path is attached to box 1, box 2 starts moving along and across it; the constraint forces
    // are internal, so the total linear momentum stays the same
    var path = try makeLine(allocator, Vec3.init(-5, 0, 0), Vec3.init(5, 0, 0), 3, Vec3.axisY());
    defer path.deinit();
    const box1 = try c.createBox(RVec3.init(0, 0, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.5, 0.5, 0.5), .{});
    const box2 = try c.createBox(RVec3.init(0, 0, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.2, 0.2, 0.2), .{});
    box2.setLinearVelocity(Vec3.init(1, 2, 0));
    var settings = PathConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.path.set(path.get());
    settings.path_fraction = 1.0;
    _ = try c.createConstraint(PathConstraint, box1, box2, settings.asTwoBodySettingsConst());

    const m1 = 1.0 / box1.getMotionProperties().getInverseMass();
    const m2 = 1.0 / box2.getMotionProperties().getInverseMass();
    const momentum = box1.getLinearVelocity().mulScalar(m1).add(box2.getLinearVelocity().mulScalar(m2));
    _ = try c.simulate(1.0, {});
    const after = box1.getLinearVelocity().mulScalar(m1).add(box2.getLinearVelocity().mulScalar(m2));
    try expect(after.isClose(momentum, .{ .max_dist_sq = 1.0e-4 * momentum.lengthSq() }));

    // Box 2 stays on the path of box 1 (Y and Z in the space of box 1 are 0) and moved along it (X)
    const local = box1.getInverseCenterOfMassTransform().mulVec3(box2.getCenterOfMassPosition().toVec3()).toVec3();
    try expect(@abs(local.getY()) < 1.0e-2 and @abs(local.getZ()) < 1.0e-2);
    try expect(local.getX() > 0.1);
}

test "TestPathConstraintFromPhysicsSystem (Jolt's release behavior)" {
    // PathConstraint::GetConstraintSettings is not implemented in Jolt: it asserts and returns nullptr. Jolt's release
    // build continues, PhysicsScene::FromPhysicsSystem adds the constraint with null settings (AddConstraint(nullptr,
    // ...)) and the body indices. Zolt's ReleaseFast build does the same instead of unwrapping the empty Ref.
    if (Core.enable_asserts) return error.SkipZigTest;

    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    var path = try makeLine(allocator, Vec3.init(-5, 0, 0), Vec3.init(5, 0, 0), 3, Vec3.axisY());
    defer path.deinit();
    const box1 = try c.createBox(RVec3.init(0, 0, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.5, 0.5, 0.5), .{});
    const box2 = try c.createBox(RVec3.init(1, 0, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.2, 0.2, 0.2), .{});
    var settings = PathConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.path.set(path.get());
    _ = try c.createConstraint(PathConstraint, box1, box2, settings.asTwoBodySettingsConst());
    _ = try c.createConstraint(PathConstraint, Body.fixedToWorld(), box1, settings.asTwoBodySettingsConst());

    var scene: PhysicsScene = .init(allocator);
    defer scene.deinit();
    try scene.fromPhysicsSystem(&c.system);
    try std.testing.expectEqual(@as(usize, 2), scene.getNumBodies());
    try std.testing.expectEqual(@as(usize, 2), scene.getNumConstraints());
    const constraints = scene.getConstraints();
    try expect(constraints[0].getSettings() == null);
    try std.testing.expectEqual(@as(u32, 0), constraints[0].body1);
    try std.testing.expectEqual(@as(u32, 1), constraints[0].body2);
    try expect(constraints[1].getSettings() == null);
    try std.testing.expectEqual(PhysicsScene.fixed_to_world, constraints[1].body1);
    try std.testing.expectEqual(@as(u32, 0), constraints[1].body2);

    // A copy of the scene keeps the constraints without settings
    var copy = scene.getConstraints()[0].clone();
    defer copy.deinit();
    try expect(copy.getSettings() == null);
}
