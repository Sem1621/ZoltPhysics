//! Zolt addition, no Jolt file: simulations of GearConstraint through PhysicsTestContext
//! Status: complete
//!
//! Jolt has no GearConstraintTests.cpp. These tests check the physics of the constraint in a PhysicsSystem: two gears on
//! hinges from the world, the first one driven by a hinge motor (velocity and position), turn with the ratio of SetRatio
//! (Gear1Rotation = -ratio * Gear2Rotation), with SetConstraints the position correction keeps the angles in that
//! relation also when they wrap around (for an integer ratio), and a disabled gear constraint uncouples them. Jolt's
//! position correction works on the hinge angles, which wrap around at +-pi: with a ratio whose multiple of 2 pi is not a
//! multiple of 2 pi, a wrap of the angle of gear 2 shifts the relation that the correction restores (reproduced, see
//! "TestGearConstraintAngleWrap"). The class itself (binary state, conversions, solver calls, allocation failures) is
//! tested inline in Zolt/Physics/Constraints/GearConstraint.zig, bit exactness in ZoltParity/Physics/ConstraintsParity.zig
//! and the gear scenes of ZoltParity/Physics/SimulationParity.zig.

const std = @import("std");
const zolt = @import("zolt");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Vec3 = zolt.Vec3;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const GearConstraint = zolt.GearConstraint;
const GearConstraintSettings = zolt.GearConstraintSettings;
const HingeConstraint = zolt.HingeConstraint;
const HingeConstraintSettings = zolt.HingeConstraintSettings;
const math = zolt.math;

const expect = std.testing.expect;

/// Two gears (spheres of equal size: with Jolt's GearConstraintPart, which applies the impulse to body 2 without the
/// ratio, the iterations converge for these ratios) on hinges around Z from the world, coupled by a gear constraint
const Gears = struct {
    gear1: *Body,
    gear2: *Body,
    hinge1: *HingeConstraint,
    hinge2: *HingeConstraint,
    gear: *GearConstraint,

    fn init(context: *PhysicsTestContext, num_teeth1: i32, num_teeth2: i32, set_constraints: bool) !Gears {
        const allocator = std.testing.allocator;
        context.zeroGravity();
        var self: Gears = undefined;
        self.gear1 = try context.createSphere(RVec3.init(0, 5, 0), 0.5, .dynamic, .discrete, Layers.moving, .{});
        self.gear2 = try context.createSphere(RVec3.init(1.1, 5, 0), 0.5, .dynamic, .discrete, Layers.moving, .{});
        self.gear1.setAllowSleeping(false);
        self.gear2.setAllowSleeping(false);
        self.hinge1 = try createHinge(context, self.gear1);
        self.hinge2 = try createHinge(context, self.gear2);

        var settings = GearConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.hinge_axis1 = Vec3.axisZ();
        settings.hinge_axis2 = Vec3.axisZ();
        settings.setRatio(num_teeth1, num_teeth2);
        self.gear = try context.createConstraint(GearConstraint, self.gear1, self.gear2, settings.asTwoBodySettingsConst());
        if (set_constraints) self.gear.setConstraints(self.hinge1.asConstraintConst(), self.hinge2.asConstraintConst());
        return self;
    }

    fn createHinge(context: *PhysicsTestContext, body: *Body) !*HingeConstraint {
        var settings = HingeConstraintSettings.init(std.testing.allocator);
        defer settings.asSettings().deinit();
        settings.point1 = body.getCenterOfMassPosition();
        settings.point2 = settings.point1;
        settings.hinge_axis1 = Vec3.axisZ();
        settings.hinge_axis2 = Vec3.axisZ();
        settings.normal_axis1 = Vec3.axisX();
        settings.normal_axis2 = Vec3.axisX();
        return context.createConstraint(HingeConstraint, Body.fixedToWorld(), body, settings.asTwoBodySettingsConst());
    }

    /// The error of the gear relation: gear1rot + ratio * gear2rot, modulo 2 pi and centered around zero
    fn angleError(self: *const Gears) f32 {
        return math.centerAngleAroundZero(@rem(self.hinge1.getCurrentAngle() + self.gear.ratio * self.hinge2.getCurrentAngle(), 2.0 * math.pi));
    }
};

test "TestGearConstraintVelocityMotor" {
    var context: PhysicsTestContext = undefined;
    try context.init(std.testing.allocator, .{});
    defer context.deinit();

    // Gear 1 has 10 teeth, gear 2 has 20: gear 2 turns at half the speed in the opposite direction
    var gears = try Gears.init(&context, 10, 20, true);
    try expect(gears.gear.ratio == 2.0);
    gears.hinge1.setMotorState(.velocity);
    gears.hinge1.setTargetAngularVelocity(2.0);
    _ = try context.simulate(2.0, {});
    try expect(@abs(gears.gear1.getAngularVelocity().getZ() - 2.0) < 1.0e-2);
    try expect(@abs(gears.gear2.getAngularVelocity().getZ() + 1.0) < 1.0e-2);
    try expect(gears.gear.getTotalLambda() != 0.0);

    // After 2 s gear 1 turned 4 rad (its hinge angle wrapped around): the relation still holds
    try expect(@abs(gears.angleError()) < 1.0e-3);

    // The other direction
    gears.hinge1.setTargetAngularVelocity(-1.0);
    _ = try context.simulate(1.0, {});
    try expect(@abs(gears.gear1.getAngularVelocity().getZ() + 1.0) < 1.0e-2);
    try expect(@abs(gears.gear2.getAngularVelocity().getZ() - 0.5) < 1.0e-2);
    try expect(@abs(gears.angleError()) < 1.0e-3);
}

test "TestGearConstraintRatios" {
    // Gear 2 turns at -num_teeth1 / num_teeth2 times the speed of gear 1 (the angles stay in the relation as long as
    // the angle of gear 2 does not wrap around)
    const teeth = [_][2]i32{ .{ 10, 10 }, .{ 12, 18 }, .{ 30, 20 }, .{ 16, 40 } };
    for (teeth) |t| {
        var context: PhysicsTestContext = undefined;
        try context.init(std.testing.allocator, .{});
        defer context.deinit();
        var gears = try Gears.init(&context, t[0], t[1], true);
        gears.hinge1.setMotorState(.velocity);
        gears.hinge1.setTargetAngularVelocity(1.5);
        _ = try context.simulate(1.0, {});
        const expected = -1.5 * @as(f32, @floatFromInt(t[0])) / @as(f32, @floatFromInt(t[1]));
        try expect(@abs(gears.gear2.getAngularVelocity().getZ() - expected) < 1.0e-2);
        try expect(@abs(gears.hinge2.getCurrentAngle()) < 3.0);
        try expect(@abs(gears.angleError()) < 1.0e-3);
    }
}

test "TestGearConstraintAngleWrap" {
    var context: PhysicsTestContext = undefined;
    try context.init(std.testing.allocator, .{});
    defer context.deinit();

    // Gear 1 has 20 teeth, gear 2 has 10 (ratio 0.5): gear 2 turns twice as fast in the opposite direction
    var gears = try Gears.init(&context, 20, 10, true);
    try expect(gears.gear.ratio == 0.5);
    gears.hinge1.setMotorState(.velocity);
    gears.hinge1.setTargetAngularVelocity(1.0);

    // Until the angle of gear 2 reaches -pi (after about 94 steps) the angles are in the relation; when it wraps around
    // to +pi, the error that Jolt corrects, CenterAngleAroundZero(fmod(gear1rot + ratio * gear2rot, 2 pi)) with the
    // wrapped angles, jumps by ratio * 2 pi = pi and the position correction turns the gears away from the relation
    // (gear 2 then hovers around the wrap point while the correction and the velocity coupling fight)
    var max_error_after_wrap: f32 = 0.0;
    for (0..150) |step| {
        _ = try context.simulateSingleStep();
        const err = gears.angleError();
        try expect(std.math.isFinite(err));
        if (step < 90) {
            try expect(@abs(err) < 1.0e-3);
        } else {
            max_error_after_wrap = @max(max_error_after_wrap, @abs(err));
        }
    }
    try expect(max_error_after_wrap > 2.0);
}

test "TestGearConstraintPositionMotor" {
    var context: PhysicsTestContext = undefined;
    try context.init(std.testing.allocator, .{});
    defer context.deinit();

    // A position motor turns gear 1 to 1 rad, gear 2 follows to -2 rad
    var gears = try Gears.init(&context, 20, 10, true);
    gears.hinge1.setMotorState(.position);
    gears.hinge1.setTargetAngle(1.0);
    _ = try context.simulate(4.0, {});
    try expect(@abs(gears.hinge1.getCurrentAngle() - 1.0) < 1.0e-2);
    try expect(@abs(gears.hinge2.getCurrentAngle() + 2.0) < 2.0e-2);
    try expect(gears.gear2.getAngularVelocity().length() < 1.0e-2);
}

test "TestGearConstraintSetConstraints" {
    var context: PhysicsTestContext = undefined;
    try context.init(std.testing.allocator, .{});
    defer context.deinit();

    // Without the hinges the gear only couples the velocities: an angle error stays
    var gears = try Gears.init(&context, 10, 10, false);
    context.getBodyInterface().setRotation(gears.gear2.getID(), zolt.Quat.rotation(Vec3.axisZ(), 0.5), .activate);
    gears.hinge1.setMotorState(.velocity);
    gears.hinge1.setTargetAngularVelocity(1.0);
    _ = try context.simulate(0.5, {});
    try expect(@abs(gears.angleError() - 0.5) < 1.0e-2);
    try expect(@abs(gears.gear2.getAngularVelocity().getZ() + 1.0) < 1.0e-2);

    // SetConstraints: the position correction removes the error
    gears.gear.setConstraints(gears.hinge1.asConstraintConst(), gears.hinge2.asConstraintConst());
    _ = try context.simulate(1.0, {});
    try expect(@abs(gears.angleError()) < 1.0e-3);

    // A disabled gear uncouples the gears: gear 2 keeps its velocity while gear 1 stops
    gears.gear.asConstraint().setEnabled(false);
    gears.hinge1.setTargetAngularVelocity(0.0);
    _ = try context.simulate(0.5, {});
    try expect(@abs(gears.gear1.getAngularVelocity().getZ()) < 1.0e-3);
    try expect(@abs(gears.gear2.getAngularVelocity().getZ() + 1.0) < 1.0e-2);
}
