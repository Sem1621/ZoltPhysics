//! Zolt addition, no Jolt file: simulations of SwingTwistConstraint through PhysicsTestContext
//! Status: complete
//!
//! Jolt has no SwingTwistConstraintTests.cpp. These tests check the physics of the constraint in a PhysicsSystem
//! (without gravity, the bodies are attached at their center of mass so that only their rotation matters): the twist
//! limits, the swing limits of both swing types, a velocity motor, position motors that reach a target orientation (also
//! one outside the limits, which is clamped) and the friction torque. The class itself (binary state, conversions,
//! solver calls, allocation failures) is tested inline in Zolt/Physics/Constraints/SwingTwistConstraint.zig, bit
//! exactness in ZoltParity/Physics/ConstraintsParity.zig and SimulationParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const SwingTwistConstraint = zolt.SwingTwistConstraint;
const SwingTwistConstraintSettings = zolt.SwingTwistConstraintSettings;
const SwingType = zolt.SwingType;
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

/// A world without gravity and a box attached to the world at its center of mass with a swing twist constraint (twist
/// axis X, plane axis Z, so the normal axis is Y)
const Rig = struct {
    context: PhysicsTestContext,
    settings: SwingTwistConstraintSettings,
    box: *Body,
    constraint: *SwingTwistConstraint,

    const position = rvec3f(1.0, 3.0, -2.0);

    /// Initialize in place; `limits` are the normal half cone angle, the plane half cone angle, the twist min / max angle
    fn init(self: *Rig, allocator: std.mem.Allocator, swing_type: SwingType, limits: [4]f32, friction: f32) !void {
        try self.context.init(allocator, .{});
        errdefer self.context.deinit();
        self.context.zeroGravity();
        self.box = try self.context.createBox(position, Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.init(0.5, 0.25, 0.25), .{});
        self.settings = SwingTwistConstraintSettings.init(allocator);
        errdefer self.settings.asSettings().deinit();
        self.settings.position1 = position;
        self.settings.position2 = position;
        self.settings.twist_axis1 = Vec3.axisX();
        self.settings.plane_axis1 = Vec3.axisZ();
        self.settings.twist_axis2 = Vec3.axisX();
        self.settings.plane_axis2 = Vec3.axisZ();
        self.settings.swing_type = swing_type;
        self.settings.normal_half_cone_angle = limits[0];
        self.settings.plane_half_cone_angle = limits[1];
        self.settings.twist_min_angle = limits[2];
        self.settings.twist_max_angle = limits[3];
        self.settings.max_friction_torque = friction;
        self.constraint = try self.context.createConstraint(SwingTwistConstraint, Body.fixedToWorld(), self.box, self.settings.asTwoBodySettingsConst());
    }

    fn deinit(self: *Rig) void {
        self.settings.asSettings().deinit();
        self.context.deinit();
    }

    fn setAngularVelocity(self: *Rig, w: Vec3) void {
        self.context.getBodyInterface().setAngularVelocity(self.box.getID(), w);
    }

    /// The twist angle of the constraint (around X)
    fn twistAngle(self: *const Rig) f32 {
        const twist = self.constraint.getRotationInConstraintSpace().getSwingTwist().twist;
        return 2.0 * trigonometry.atan2(twist.getX(), twist.getW());
    }

    /// The swing angles of the constraint around the normal axis Y (limited by the plane half cone angle) and the plane
    /// axis Z (limited by the normal half cone angle), for a swing around one of them
    fn swingAngles(self: *const Rig) [2]f32 {
        const swing = self.constraint.getRotationInConstraintSpace().getSwingTwist().swing;
        return .{ 2.0 * trigonometry.atan2(swing.getY(), swing.getW()), 2.0 * trigonometry.atan2(swing.getZ(), swing.getW()) };
    }
};

test "TestSwingTwistConstraintTwistLimits" {
    var rig: Rig = undefined;
    try rig.init(std.testing.allocator, .cone, .{ 0.2, 0.2, -0.3, 0.5 }, 0.0);
    defer rig.deinit();

    // The box spins around the twist axis into the max twist angle, then the other way into the min twist angle (the
    // limit acts once it is violated: the box overshoots by up to one step, the position steps turn it back)
    for ([_]f32{ 4.0, -4.0 }) |w| {
        rig.setAngularVelocity(Vec3.init(w, 0.0, 0.0));
        var extreme: f32 = 0.0;
        var limit_impulse = false;
        for (0..60) |_| {
            _ = try rig.context.simulateSingleStep();
            if (rig.constraint.getTotalLambdaTwist() != 0.0) limit_impulse = true;
            const angle = rig.twistAngle();
            try expect(angle < 0.5 + 4.0 / 60.0 and angle > -0.3 - 4.0 / 60.0);
            extreme = if (w > 0.0) math.max(extreme, angle) else math.min(extreme, angle);
            try expect(distance(rig.box.getCenterOfMassPosition(), Rig.position) < 1.0e-3);
        }
        if (w > 0.0) try expect(extreme > 0.48) else try expect(extreme < -0.28);
        try expect(limit_impulse); // The limit stopped the box
    }
}

test "TestSwingTwistConstraintSwingLimits" {
    // Cone and pyramid swing: a swing around the normal axis Y is limited by the plane half cone angle, a swing around the
    // plane axis Z by the normal half cone angle
    for ([_]SwingType{ .cone, .pyramid }) |swing_type| {
        for (0..2) |axis| {
            var rig: Rig = undefined;
            try rig.init(std.testing.allocator, swing_type, .{ 0.6, 0.4, -0.1, 0.1 }, 0.0);
            defer rig.deinit();
            const limit: f32 = if (axis == 0) 0.4 else 0.6;
            rig.setAngularVelocity(if (axis == 0) Vec3.init(0.0, 3.0, 0.0) else Vec3.init(0.0, 0.0, 3.0));
            var extreme: f32 = 0.0;
            var limit_impulse = false;
            for (0..60) |_| {
                _ = try rig.context.simulateSingleStep();
                if (rig.constraint.getTotalLambdaSwingY() != 0.0 or rig.constraint.getTotalLambdaSwingZ() != 0.0) limit_impulse = true;
                const angle = rig.swingAngles()[axis];
                try expect(@abs(angle) < limit + 3.0 / 60.0); // Up to one step of overshoot
                extreme = math.max(extreme, angle);
            }
            try expect(extreme > limit - 0.02);
            try expect(limit_impulse); // The limit stopped the box
        }
    }
}

test "TestSwingTwistConstraintVelocityMotor" {
    var rig: Rig = undefined;
    try rig.init(std.testing.allocator, .cone, .{ 0.5, 0.5, -std.math.pi, std.math.pi }, 0.0);
    defer rig.deinit();

    // The twist is free: a twist velocity motor spins the box up to the target angular velocity (in the constraint space
    // of body 2, which is the world space here)
    rig.constraint.setTwistMotorState(.velocity);
    rig.constraint.setTargetAngularVelocityCS(Vec3.init(2.0, 0.0, 0.0));
    _ = try rig.context.simulate(0.5, {});
    try expect(rig.box.getAngularVelocity().isClose(Vec3.init(2.0, 0.0, 0.0), .{ .max_dist_sq = 1.0e-6 }));
    try expectApproxEqAbs(@as(f32, 0.0), rig.constraint.getTotalLambdaMotor().getX(), 1.0e-4); // Nothing to correct anymore

    // A torque limit: the motor can only slowly brake the box
    rig.constraint.getTwistMotorSettingsMut().setTorqueLimit(1.0);
    rig.constraint.setTargetAngularVelocityCS(Vec3.zero());
    _ = try rig.context.simulateSingleStep();
    try expectApproxEqAbs(@as(f32, 1.0 / 60.0), @abs(rig.constraint.getTotalLambdaMotor().getX()), 1.0e-6);
    try expect(rig.box.getAngularVelocity().getX() > 1.9);
}

test "TestSwingTwistConstraintPositionMotor" {
    for ([_]SwingType{ .cone, .pyramid }) |swing_type| {
        var rig: Rig = undefined;
        try rig.init(std.testing.allocator, swing_type, .{ 0.6, 0.5, -0.4, 0.6 }, 0.0);
        defer rig.deinit();

        // Swing and twist position motors drive the constraint to a target orientation inside the limits
        rig.constraint.setSwingMotorState(.position);
        rig.constraint.setTwistMotorState(.position);
        const target = Quat.rotation(Vec3.axisY(), 0.3).mul(Quat.rotation(Vec3.axisX(), 0.4));
        rig.constraint.setTargetOrientationCS(target);
        try expect(rig.constraint.getTargetOrientationCS().eql(target)); // Inside the limits: not clamped
        _ = try rig.context.simulate(4.0, {});
        try expect(rig.constraint.getRotationInConstraintSpace().isClose(target, .{ .max_dist_sq = 1.0e-5 }));
        try expect(rig.box.getAngularVelocity().isNearZero(.{ .max_dist_sq = 1.0e-4 }));

        // A target outside the twist limit is clamped to the limit, the box stops there. The box is at rest and asleep: a
        // new target does not wake it up
        try expect(!rig.box.isActive());
        rig.constraint.setTargetOrientationCS(Quat.rotation(Vec3.axisX(), 1.5));
        try expect(!rig.constraint.getTargetOrientationCS().isClose(Quat.rotation(Vec3.axisX(), 1.5), .{}));
        rig.context.getBodyInterface().activateBody(rig.box.getID());
        _ = try rig.context.simulate(4.0, {});
        try expectApproxEqAbs(@as(f32, 0.6), rig.twistAngle(), 0.02);

        // A target in body space: R2 = R1 * orientation (body 1 is the world)
        const body_space = Quat.rotation(Vec3.axisZ(), -0.3);
        rig.constraint.setTargetOrientationBS(body_space);
        rig.context.getBodyInterface().activateBody(rig.box.getID());
        _ = try rig.context.simulate(4.0, {});
        try expect(rig.box.getRotation().isClose(body_space, .{ .max_dist_sq = 1.0e-5 }));
    }
}

test "TestSwingTwistConstraintFriction" {
    var rig: Rig = undefined;
    try rig.init(std.testing.allocator, .cone, .{ 0.5, 0.5, -std.math.pi, std.math.pi }, 10.0);
    defer rig.deinit();

    // The box spins around the free twist axis, the friction torque brakes it by dt * friction / inertia per step
    rig.setAngularVelocity(Vec3.init(2.0, 0.0, 0.0));
    const inv_inertia = rig.box.getMotionProperties().getInverseInertiaForRotation(Mat44.identity()).get(0, 0);
    _ = try rig.context.simulate(0.5, {});
    const expected = 2.0 - 30.0 * (10.0 / 60.0) * inv_inertia;
    try expect(expected > 0.5); // Still spinning
    try expectApproxEqAbs(expected, rig.box.getAngularVelocity().getX(), 1.0e-3);
    try expectApproxEqAbs(@as(f32, -10.0 / 60.0), rig.constraint.getTotalLambdaMotor().getX(), 1.0e-6);

    // Friction stops it in the end and holds it
    _ = try rig.context.simulate(5.0, {});
    try expect(rig.box.getAngularVelocity().isNearZero(.{ .max_dist_sq = 1.0e-8 }));
}
