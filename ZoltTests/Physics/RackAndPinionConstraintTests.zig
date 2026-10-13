//! Zolt addition, no Jolt file: simulations of RackAndPinionConstraint through PhysicsTestContext
//! Status: complete
//!
//! Jolt has no RackAndPinionConstraintTests.cpp. These tests check the physics of the constraint in a PhysicsSystem: a
//! pinion on a hinge and a rack on a slider from the world, driven by a slider motor or by a hinge motor, move with the
//! ratio of SetRatio (PinionRotation = ratio * RackTranslation), with SetConstraints the position correction keeps the
//! angle and the position in that relation also when the pinion angle wraps around, and the limit of the slider stops
//! the pinion. The class itself (binary state, conversions, solver calls, allocation failures) is tested inline in
//! Zolt/Physics/Constraints/RackAndPinionConstraint.zig, bit exactness in ZoltParity/Physics/ConstraintsParity.zig and
//! the rack and pinion scenes of ZoltParity/Physics/SimulationParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const RackAndPinionConstraint = zolt.RackAndPinionConstraint;
const RackAndPinionConstraintSettings = zolt.RackAndPinionConstraintSettings;
const HingeConstraint = zolt.HingeConstraint;
const HingeConstraintSettings = zolt.HingeConstraintSettings;
const SliderConstraint = zolt.SliderConstraint;
const SliderConstraintSettings = zolt.SliderConstraintSettings;
const math = zolt.math;

const expect = std.testing.expect;

/// A pinion (a sphere) on a hinge around Z and a rack (a box) on a slider along X, both from the world, coupled by a
/// rack and pinion constraint with SetRatio(20 teeth on the rack, rack length 2, 10 teeth on the pinion): one turn of the
/// pinion moves the rack by 1, the ratio is 2 pi
const RackAndPinion = struct {
    pinion_body: *Body,
    rack_body: *Body,
    pinion: *HingeConstraint,
    rack: *SliderConstraint,
    constraint: *RackAndPinionConstraint,

    fn init(context: *PhysicsTestContext, limits: ?[2]f32) !RackAndPinion {
        const allocator = std.testing.allocator;
        context.zeroGravity();
        var self: RackAndPinion = undefined;
        self.pinion_body = try context.createSphere(RVec3.init(0, 5, 0), 0.3, .dynamic, .discrete, Layers.moving, .{});
        self.rack_body = try context.createBox(RVec3.init(0, 4.6, 0), Quat.identity(), .dynamic, .discrete, Layers.moving2, Vec3.init(1.0, 0.05, 0.05), .{});
        self.pinion_body.setAllowSleeping(false);
        self.rack_body.setAllowSleeping(false);

        var hinge_settings = HingeConstraintSettings.init(allocator);
        defer hinge_settings.asSettings().deinit();
        hinge_settings.point1 = self.pinion_body.getCenterOfMassPosition();
        hinge_settings.point2 = hinge_settings.point1;
        hinge_settings.hinge_axis1 = Vec3.axisZ();
        hinge_settings.hinge_axis2 = Vec3.axisZ();
        hinge_settings.normal_axis1 = Vec3.axisX();
        hinge_settings.normal_axis2 = Vec3.axisX();
        self.pinion = try context.createConstraint(HingeConstraint, Body.fixedToWorld(), self.pinion_body, hinge_settings.asTwoBodySettingsConst());

        var slider_settings = SliderConstraintSettings.init(allocator);
        defer slider_settings.asSettings().deinit();
        slider_settings.auto_detect_point = true;
        slider_settings.setSliderAxis(Vec3.axisX());
        if (limits) |l| {
            slider_settings.limits_min = l[0];
            slider_settings.limits_max = l[1];
        }
        self.rack = try context.createConstraint(SliderConstraint, Body.fixedToWorld(), self.rack_body, slider_settings.asTwoBodySettingsConst());

        var settings = RackAndPinionConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.hinge_axis = Vec3.axisZ();
        settings.slider_axis = Vec3.axisX();
        settings.setRatio(20, 2.0, 10);
        self.constraint = try context.createConstraint(RackAndPinionConstraint, self.pinion_body, self.rack_body, settings.asTwoBodySettingsConst());
        self.constraint.setConstraints(self.pinion.asConstraintConst(), self.rack.asConstraintConst());
        return self;
    }

    /// The error of the relation: rotation - ratio * translation, modulo 2 pi and centered around zero
    fn positionError(self: *const RackAndPinion) f32 {
        return math.centerAngleAroundZero(@rem(self.pinion.getCurrentAngle() - self.constraint.ratio * self.rack.getCurrentPosition(), 2.0 * math.pi));
    }
};

test "TestRackAndPinionSliderMotor" {
    var context: PhysicsTestContext = undefined;
    try context.init(std.testing.allocator, .{});
    defer context.deinit();

    // A slider motor moves the rack at 0.5 m/s: the pinion turns at ratio * 0.5 = pi rad/s
    var rp = try RackAndPinion.init(&context, null);
    try expect(@abs(rp.constraint.ratio - 2.0 * math.pi) < 1.0e-6);
    rp.rack.setMotorState(.velocity);
    rp.rack.setTargetVelocity(0.5);
    _ = try context.simulate(1.5, {});
    try expect(@abs(rp.rack_body.getLinearVelocity().getX() - 0.5) < 1.0e-3);
    try expect(@abs(rp.pinion_body.getAngularVelocity().getZ() - math.pi) < 1.0e-2);
    try expect(rp.constraint.getTotalLambda() != 0.0);

    // The pinion turned more than pi (its hinge angle wrapped around): the relation still holds
    try expect(@abs(rp.positionError()) < 1.0e-3);
}

test "TestRackAndPinionHingeMotor" {
    var context: PhysicsTestContext = undefined;
    try context.init(std.testing.allocator, .{});
    defer context.deinit();

    // A hinge motor turns the pinion at -pi rad/s: the rack moves at -0.5 m/s
    var rp = try RackAndPinion.init(&context, null);
    rp.pinion.setMotorState(.velocity);
    rp.pinion.setTargetAngularVelocity(-math.pi);
    _ = try context.simulate(1.0, {});
    try expect(@abs(rp.pinion_body.getAngularVelocity().getZ() + math.pi) < 1.0e-2);
    try expect(@abs(rp.rack_body.getLinearVelocity().getX() + 0.5) < 1.0e-3);
    try expect(@abs(rp.positionError()) < 1.0e-3);

    // A position motor on the rack takes the pinion along to the angle of the position
    rp.pinion.setMotorState(.off);
    rp.rack.setMotorState(.position);
    rp.rack.setTargetPosition(0.1);
    _ = try context.simulate(4.0, {});
    try expect(@abs(rp.rack.getCurrentPosition() - 0.1) < 1.0e-2);
    try expect(@abs(rp.pinion.getCurrentAngle() - 0.1 * rp.constraint.ratio) < 5.0e-2);
}

test "TestRackAndPinionSliderLimit" {
    var context: PhysicsTestContext = undefined;
    try context.init(std.testing.allocator, .{});
    defer context.deinit();

    // A hinge motor drives the rack into the limit of its slider: the pinion stops at ratio * limit
    var rp = try RackAndPinion.init(&context, .{ -0.2, 0.2 });
    rp.pinion.setMotorState(.velocity);
    rp.pinion.setTargetAngularVelocity(2.0);
    rp.pinion.getMotorSettingsMut().setTorqueLimit(5.0);
    _ = try context.simulate(1.5, {});
    try expect(@abs(rp.rack.getCurrentPosition() - 0.2) < 1.0e-2);
    try expect(@abs(rp.pinion.getCurrentAngle() - 0.2 * rp.constraint.ratio) < 5.0e-2);

    // It stays there (the motor, the rack and pinion and the limit of the slider fight in the velocity solver, the
    // position solver keeps the rack at its limit and the pinion at the angle of the rack)
    _ = try context.simulate(0.5, {});
    try expect(@abs(rp.rack.getCurrentPosition() - 0.2) < 1.0e-2);
    try expect(@abs(rp.pinion.getCurrentAngle() - 0.2 * rp.constraint.ratio) < 5.0e-2);
}
