//! Zolt addition, no Jolt file: simulations of FixedConstraint through PhysicsTestContext
//! Status: complete
//!
//! Jolt has no FixedConstraintTests.cpp. These tests check the physics of the constraint in a PhysicsSystem: a body
//! fixed to Body.fixedToWorld() (Jolt's Body::sFixedToWorld) carries its weight (the total lambda is the impulse of
//! gravity), two welded bodies move as one rigid body, a compound pendulum (a point constraint to the world and a
//! welded second body) swings with the period of the combined rigid body, a dynamic body welded to a rotating kinematic
//! body follows it, and a disabled constraint lets the body fall until it is enabled again. The class itself (binary
//! state, conversions, solver calls, allocation failures) is tested inline in
//! Zolt/Physics/Constraints/FixedConstraint.zig, bit exactness in ZoltParity/Physics/ConstraintsParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const FixedConstraint = zolt.FixedConstraint;
const FixedConstraintSettings = zolt.FixedConstraintSettings;
const PointConstraint = zolt.PointConstraint;
const PointConstraintSettings = zolt.PointConstraintSettings;
const math = zolt.math;
const trigonometry = zolt.trigonometry;

const expect = std.testing.expect;
const expectApproxEqAbs = std.testing.expectApproxEqAbs;
const checkApproxEqual = fw.checkApproxEqual;

/// RVec3 of float values (widened to Real in double precision)
fn rvec3f(x: f32, y: f32, z: f32) RVec3 {
    return RVec3.init(x, y, z);
}

/// The distance between two world space positions (computed in Real, returned as float)
fn distance(a: RVec3, b: RVec3) f32 {
    return @floatCast(a.sub(b).length());
}

/// The angle of the rotation from `a` to `b`
fn angleBetween(a: Quat, b: Quat) f32 {
    var q = a.conjugated().mul(b);
    if (q.getW() < 0.0) q = q.negate();
    return 2.0 * trigonometry.acos(if (q.getW() > 1.0) 1.0 else q.getW());
}

/// The pose of body 2 in the center of mass space of body 1
const RelativePose = struct {
    position: Vec3,
    rotation: Quat,

    fn of(body1: *const Body, body2: *const Body) RelativePose {
        return .{
            .position = body1.getInverseCenterOfMassTransform().mulRVec3(body2.getCenterOfMassPosition()).toVec3(),
            .rotation = body1.getRotation().conjugated().mul(body2.getRotation()),
        };
    }

    fn expectClose(self: RelativePose, other: RelativePose, max_distance: f32, max_angle: f32) !void {
        try expect(self.position.sub(other.position).length() < max_distance);
        try expect(angleBetween(self.rotation, other.rotation) < max_angle);
    }
};

test "TestFixedConstraintToWorld" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // A rotated box in the air, fixed to the world at its center of mass (auto detect point)
    const position = rvec3f(1.0, 5.0, 2.0);
    const rotation = Quat.rotation(Vec3.init(1.0, 2.0, 3.0).normalized(), 0.7);
    const box = try c.createBox(position, rotation, .dynamic, .discrete, Layers.moving, Vec3.init(0.5, 1.0, 1.5), .{});
    var settings = FixedConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.auto_detect_point = true;
    const constraint = try c.createConstraint(FixedConstraint, Body.fixedToWorld(), box, settings.asTwoBodySettingsConst());
    try expect(constraint.asConstraint().isActive());

    // The constraint carries the weight of the box: the impulse per step is the impulse of gravity
    const mass = 1.0 / box.getMotionProperties().getInverseMass();
    const gravity = c.getSystem().getGravity();
    for (0..10) |_| _ = try c.simulateSingleStep();
    const weight_impulse = gravity.mulScalar(-mass * c.getDeltaTime());
    try expect(constraint.getTotalLambdaPosition().isClose(weight_impulse, .{ .max_dist_sq = 1.0e-6 * weight_impulse.lengthSq() }));
    try expect(constraint.getTotalLambdaRotation().length() < 1.0e-3 * weight_impulse.length());

    // The box does not move and goes to sleep
    _ = try c.simulate(2.0, {});
    try expect(distance(box.getPosition(), position) < 1.0e-3);
    try expect(angleBetween(box.getRotation(), rotation) < 1.0e-3);
    try expect(!box.isActive());
}

test "TestFixedConstraintRigidPair" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    // Two boxes of different size side by side (body 2 is in a layer that does not collide with body 1), welded
    const box1 = try c.createBox(rvec3f(0.0, 5.0, 0.0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1.0, 0.5, 0.5), .{});
    const box2 = try c.createBox(rvec3f(1.5, 5.5, 0.0), Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), .dynamic, .discrete, Layers.moving2, Vec3.replicate(0.5), .{});
    var settings = FixedConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.auto_detect_point = true;
    _ = try c.createConstraint(FixedConstraint, box1, box2, settings.asTwoBodySettingsConst());
    const initial_pose = RelativePose.of(box1, box2);

    // Spin and throw box 1
    c.getBodyInterface().setLinearAndAngularVelocity(box1.getID(), Vec3.init(1.0, -0.5, 0.25), Vec3.init(0.5, 0.0, 2.0));
    const m1 = 1.0 / box1.getMotionProperties().getInverseMass();
    const m2 = 1.0 / box2.getMotionProperties().getInverseMass();
    const momentum = box1.getLinearVelocity().mulScalar(m1);
    for (0..120) |_| {
        _ = try c.simulateSingleStep();

        // The pair moves as one rigid body: the pose of box 2 relative to box 1 does not change, the linear momentum is
        // conserved and both boxes rotate with the same angular velocity
        try RelativePose.of(box1, box2).expectClose(initial_pose, 1.0e-2, 1.0e-2);
        const p = box1.getLinearVelocity().mulScalar(m1).add(box2.getLinearVelocity().mulScalar(m2));
        try expect(p.isClose(momentum, .{ .max_dist_sq = 1.0e-6 * momentum.lengthSq() }));
        try expect(box1.getAngularVelocity().isClose(box2.getAngularVelocity(), .{ .max_dist_sq = 1.0e-4 }));
    }
    try expect(!box1.getAngularVelocity().isNearZero(.{}));
}

test "TestFixedConstraintCompoundPendulum" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Two rods below a pivot, displaced by a small angle: rod A hangs from the pivot on a point constraint, rod B is
    // welded below rod A (B is in a layer that does not collide with A)
    const half_extent = Vec3.init(0.1, 0.5, 0.1);
    const angle: f32 = 0.1;
    const pivot = rvec3f(0.0, 10.0, 0.0);
    const rotation = Quat.rotation(Vec3.axisZ(), angle);
    const center_a = pivot.add(rvec3fFromVec3(rotation.mulVec3(Vec3.init(0.0, -0.5, 0.0))));
    const center_b = pivot.add(rvec3fFromVec3(rotation.mulVec3(Vec3.init(0.0, -1.5, 0.0))));
    const rod_a = try c.createBox(center_a, rotation, .dynamic, .discrete, Layers.moving, half_extent, .{});
    const rod_b = try c.createBox(center_b, rotation, .dynamic, .discrete, Layers.moving2, half_extent, .{});

    var point_settings = PointConstraintSettings.init(allocator);
    defer point_settings.asSettings().deinit();
    point_settings.point1 = pivot;
    point_settings.point2 = pivot;
    _ = try c.createConstraint(PointConstraint, Body.fixedToWorld(), rod_a, point_settings.asTwoBodySettingsConst());
    var fixed_settings = FixedConstraintSettings.init(allocator);
    defer fixed_settings.asSettings().deinit();
    fixed_settings.auto_detect_point = true;
    const weld = try c.createConstraint(FixedConstraint, rod_a, rod_b, fixed_settings.asTwoBodySettingsConst());
    const initial_pose = RelativePose.of(rod_a, rod_b);

    // The period of the combined rigid body: 2 pi sqrt(I / (M g d)), I the inertia around the pivot (a box rotating around
    // Z: m / 3 (hx^2 + hy^2) around its center, plus m r^2 for its distance r to the pivot), d the distance of the center
    // of mass to the pivot
    const m = 1.0 / rod_a.getMotionProperties().getInverseMass();
    const box_inertia = m / 3.0 * (half_extent.getX() * half_extent.getX() + half_extent.getY() * half_extent.getY());
    const inertia = 2.0 * box_inertia + m * 0.5 * 0.5 + m * 1.5 * 1.5;
    const g = -c.getSystem().getGravity().getY();
    const expected_period = 2.0 * math.pi * @sqrt(inertia / (2.0 * m * g * 1.0));

    var prev_x: f32 = @floatCast(center_a.getX() + center_b.getX());
    var crossings: [3]f32 = undefined;
    var num_crossings: usize = 0;
    var time: f32 = 0.0;
    for (0..300) |_| {
        _ = try c.simulateSingleStep();
        time += c.getDeltaTime();

        // The weld holds and rod A stays on the pivot
        try RelativePose.of(rod_a, rod_b).expectClose(initial_pose, 1.0e-3, 1.0e-3);
        try expectApproxEqAbs(@as(f32, 0.5), distance(rod_a.getCenterOfMassPosition(), pivot), 1.0e-3);

        // Zero crossings of the combined center of mass (interpolated)
        const x: f32 = @floatCast(rod_a.getCenterOfMassPosition().getX() + rod_b.getCenterOfMassPosition().getX() - 2.0 * pivot.getX());
        if ((prev_x > 0.0) != (x > 0.0) and num_crossings < crossings.len) {
            crossings[num_crossings] = time - c.getDeltaTime() + c.getDeltaTime() * prev_x / (prev_x - x);
            num_crossings += 1;
        }
        prev_x = x;
    }
    try expect(num_crossings == crossings.len);
    try expectApproxEqAbs(expected_period, crossings[2] - crossings[0], 0.005 * expected_period);

    // The weld carries rod B
    try expect(!weld.getTotalLambdaPosition().isNearZero(.{}));
}

/// RVec3 of a Vec3 (Jolt's `RVec3(Vec3)`)
fn rvec3fFromVec3(v: Vec3) RVec3 {
    return rvec3f(v.getX(), v.getY(), v.getZ());
}

test "TestFixedConstraintKinematicBody" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // A dynamic box welded to the side of a rotating kinematic box
    const kinematic = try c.createBox(rvec3f(0.0, 10.0, 0.0), Quat.identity(), .kinematic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});
    const box = try c.createBox(rvec3f(2.0, 10.0, 0.0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(1.0, 0.25, 0.25), .{});
    var settings = FixedConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.space = .local_to_body_com;
    settings.point1 = rvec3f(0.5, 0.0, 0.0);
    settings.point2 = rvec3f(-1.5, 0.0, 0.0);
    const constraint = try c.createConstraint(FixedConstraint, kinematic, box, settings.asTwoBodySettingsConst());
    const initial_pose = RelativePose.of(kinematic, box);

    const angular_velocity = Vec3.init(0.0, 1.0, 0.0);
    c.getBodyInterface().setAngularVelocity(kinematic.getID(), angular_velocity);
    for (0..120) |_| {
        _ = try c.simulateSingleStep();

        // The kinematic body is not affected by the constraint, the dynamic body rotates with it
        try expect(kinematic.getAngularVelocity().eql(angular_velocity));
        try expect(kinematic.getLinearVelocity().eql(Vec3.zero()));
        try RelativePose.of(kinematic, box).expectClose(initial_pose, 1.0e-2, 1.0e-2);
    }
    try expect(box.getPosition().getZ() < -1.0); // Rotated by 2 radians around Y
    try expect(constraint.getTotalLambdaPosition().getY() > 0.0); // The weight of the box
}

test "TestFixedConstraintDisabled" {
    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // A box fixed to the world
    const position = rvec3f(0.0, 10.0, 0.0);
    const box = try c.createBox(position, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});
    var settings = FixedConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = position;
    settings.point2 = position;
    const constraint = try c.createConstraint(FixedConstraint, Body.fixedToWorld(), box, settings.asTwoBodySettingsConst());
    _ = try c.simulate(0.25, {});
    try expect(distance(box.getPosition(), position) < 1.0e-3);

    // Disabled: the box falls like a free body
    constraint.asConstraint().setEnabled(false);
    try expect(!constraint.asConstraint().isActive());
    const fall_time: f32 = 0.5;
    const expected_position = c.predictPosition(box.getPosition(), box.getLinearVelocity(), c.getSystem().getGravity(), fall_time);
    _ = try c.simulate(fall_time, {});
    try checkApproxEqual(expected_position, box.getPosition(), .{});

    // Enabled again: the constraint stops the box and the position solver moves it back
    constraint.asConstraint().setEnabled(true);
    _ = try c.simulate(2.0, {});
    try expect(distance(box.getPosition(), position) < 1.0e-3);
}
