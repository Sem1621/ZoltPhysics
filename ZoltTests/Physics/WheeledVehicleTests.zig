//! Port of: UnitTests/Physics/WheeledVehicleTests.cpp
//! Status: complete
//!
//! `AddVehicle` builds the settings like Jolt (the wheel and controller settings on the heap, the constraint settings on
//! the stack) and creates the VehicleConstraint with `VehicleConstraint.create`; the physics system takes the
//! constraint (AddConstraint) and its step listener (`stepListener()`). The lambda that gives the wheels extra grip is a
//! `TireMaxImpulseCallback` without context.
//!
//! Zolt tests after the ported ones: the three collision testers, the settings and controller settings that the
//! constraint gives back, SaveState / RestoreState in the middle of a drive, the thread count independence of the
//! simulation, and allocation failure sweeps of the creation and of GetConstraintSettings.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const Allocator = std.mem.Allocator;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const Real = zolt.Real;
const RVec3 = zolt.RVec3;
const math = zolt.math;
const virtual = zolt.virtual;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const OffsetCenterOfMassShapeSettings = zolt.OffsetCenterOfMassShapeSettings;
const Constraint = zolt.Constraint;
const ConstraintSettings = zolt.ConstraintSettings;
const StateRecorderImpl = zolt.StateRecorderImpl;
const VehicleConstraint = zolt.VehicleConstraint;
const VehicleConstraintSettings = zolt.VehicleConstraintSettings;
const VehicleCollisionTester = zolt.VehicleCollisionTester;
const VehicleCollisionTesterRay = zolt.VehicleCollisionTesterRay;
const VehicleCollisionTesterCastSphere = zolt.VehicleCollisionTesterCastSphere;
const VehicleCollisionTesterCastCylinder = zolt.VehicleCollisionTesterCastCylinder;
const WheelSettings = zolt.WheelSettings;
const WheelSettingsWV = zolt.WheelSettingsWV;
const WheeledVehicleController = zolt.WheeledVehicleController;
const WheeledVehicleControllerSettings = zolt.WheeledVehicleControllerSettings;
const VehicleControllerSettings = zolt.VehicleControllerSettings;

const expect = std.testing.expect;
const checkApproxEqual = fw.checkApproxEqual;
const testing_allocator = std.testing.allocator;

const fl_wheel = 0;
const fr_wheel = 1;
const bl_wheel = 2;
const br_wheel = 3;

/// The collision tester of a test vehicle (Jolt's mUseCastSphere, plus the cylinder for the Zolt tests)
const TesterKind = enum { ray, cast_sphere, cast_cylinder };

/// Simplified vehicle settings
const VehicleSettings = struct {
    position: RVec3 = RVec3.init(0, 2, 0),
    use_cast_sphere: bool = true,
    wheel_radius: f32 = 0.3,
    wheel_width: f32 = 0.1,
    half_vehicle_length: f32 = 2.0,
    half_vehicle_width: f32 = 0.9,
    half_vehicle_height: f32 = 0.2,
    wheel_offset_horizontal: f32 = 1.4,
    wheel_offset_vertical: f32 = 0.18,
    suspension_min_length: f32 = 0.3,
    suspension_max_length: f32 = 0.5,
    max_steering_angle: f32 = math.degreesToRadians(30),
    four_wheel_drive: bool = false,
    front_back_limited_slip_ratio: f32 = 1.4,
    left_right_limited_slip_ratio: f32 = 1.4,
    anti_rollbar: bool = true,

    /// Zolt: the cylinder tester (overrides use_cast_sphere)
    use_cast_cylinder: bool = false,
};

/// Create the VehicleConstraintSettings of a test vehicle (the wheels and the controller are heap settings referenced by
/// the constraint settings). The caller calls `asSettings().deinit()`.
fn makeVehicleSettings(allocator: Allocator, settings: *const VehicleSettings) !VehicleConstraintSettings {
    var vehicle = VehicleConstraintSettings.init(allocator);
    errdefer vehicle.asSettings().deinit();
    vehicle.base.draw_constraint_size = 0.1;
    vehicle.max_pitch_roll_angle = math.degreesToRadians(60.0);

    // Wheels
    const positions = [4]Vec3{
        Vec3.init(settings.half_vehicle_width, -settings.wheel_offset_vertical, settings.wheel_offset_horizontal),
        Vec3.init(-settings.half_vehicle_width, -settings.wheel_offset_vertical, settings.wheel_offset_horizontal),
        Vec3.init(settings.half_vehicle_width, -settings.wheel_offset_vertical, -settings.wheel_offset_horizontal),
        Vec3.init(-settings.half_vehicle_width, -settings.wheel_offset_vertical, -settings.wheel_offset_horizontal),
    };
    for (positions, 0..) |position, i| {
        const w = try WheelSettingsWV.create(allocator);
        var w_ref = Ref(WheelSettings).init(&w.base);
        defer w_ref.deinit();
        w.base.position = position;
        if (i == fl_wheel or i == fr_wheel) {
            w.max_steer_angle = settings.max_steering_angle;
            w.max_hand_brake_torque = 0.0; // Front wheel doesn't have hand brake
        } else {
            w.max_steer_angle = 0.0;
        }
        try vehicle.addWheel(&w.base);
    }

    for (vehicle.wheels.items) |w_ref| {
        const w = w_ref.ptr.?;
        w.radius = settings.wheel_radius;
        w.width = settings.wheel_width;
        w.suspension_min_length = settings.suspension_min_length;
        w.suspension_max_length = settings.suspension_max_length;
    }

    const controller = try WheeledVehicleControllerSettings.create(allocator);
    vehicle.setController(&controller.base);

    // Differential
    try controller.differentials.appendNTimes(allocator, .{}, if (settings.four_wheel_drive) 2 else 1);
    controller.differentials.items[0].left_wheel = fl_wheel;
    controller.differentials.items[0].right_wheel = fr_wheel;
    controller.differentials.items[0].limited_slip_ratio = settings.left_right_limited_slip_ratio;
    controller.differential_limited_slip_ratio = settings.front_back_limited_slip_ratio;
    if (settings.four_wheel_drive) {
        controller.differentials.items[1].left_wheel = bl_wheel;
        controller.differentials.items[1].right_wheel = br_wheel;
        controller.differentials.items[1].limited_slip_ratio = settings.left_right_limited_slip_ratio;

        // Split engine torque
        controller.differentials.items[0].engine_torque_ratio = 0.5;
        controller.differentials.items[1].engine_torque_ratio = 0.5;
    }

    // Anti rollbars
    if (settings.anti_rollbar) {
        try vehicle.addAntiRollBar(.{ .left_wheel = fl_wheel, .right_wheel = fr_wheel });
        try vehicle.addAntiRollBar(.{ .left_wheel = bl_wheel, .right_wheel = br_wheel });
    }

    return vehicle;
}

/// Create the body of a test vehicle and add it to the world
fn addVehicleBody(c: *PhysicsTestContext, settings: *const VehicleSettings) !*Body {
    const allocator = c.allocator;

    // Create vehicle body
    const box = try BoxShape.create(allocator, Vec3.init(settings.half_vehicle_width, settings.half_vehicle_height, settings.half_vehicle_length), .{});
    var com = OffsetCenterOfMassShapeSettings.initPtr(allocator, Vec3.init(0, -settings.half_vehicle_height, 0), box.asShape());
    com.asShapeSettings().setEmbedded();
    defer com.deinit();
    var shape_result = try com.createShape(allocator);
    defer shape_result.deinit();
    const car_shape = shape_result.getPtr().?;
    var car_body_settings = BodyCreationSettings.initWithShape(car_shape, settings.position, Quat.identity(), .dynamic, Layers.moving);
    defer car_body_settings.deinit();
    car_body_settings.override_mass_properties = .calculate_inertia;
    car_body_settings.mass_properties_override.mass = 1500.0;
    return c.createBody(&car_body_settings, .activate);
}

/// Create a collision tester for a test vehicle (reference count 0)
fn createTester(allocator: Allocator, settings: *const VehicleSettings) !*VehicleCollisionTester {
    if (settings.use_cast_cylinder)
        return &(try VehicleCollisionTesterCastCylinder.create(allocator, Layers.moving, .{})).base;
    if (settings.use_cast_sphere)
        return &(try VehicleCollisionTesterCastSphere.create(allocator, Layers.moving, 0.5 * settings.wheel_width, .{})).base;
    return &(try VehicleCollisionTesterRay.create(allocator, Layers.moving, .{})).base;
}

/// Helper function to create a vehicle
fn addVehicle(c: *PhysicsTestContext, settings: *const VehicleSettings) !*VehicleConstraint {
    const allocator = c.allocator;
    const car_body = try addVehicleBody(c, settings);

    // Create vehicle constraint
    var vehicle = try makeVehicleSettings(allocator, settings);
    defer vehicle.asSettings().deinit();

    // Create the constraint (a temporary reference, so that it is freed when adding it to the world fails)
    const constraint = try VehicleConstraint.create(allocator, car_body, &vehicle);
    var constraint_ref = Ref(Constraint).init(constraint.asConstraint());
    defer constraint_ref.deinit();

    // Create collision tester
    var tester = RefConst(VehicleCollisionTester).init(try createTester(allocator, settings));
    defer tester.deinit();
    constraint.setVehicleCollisionTester(tester.ptr);

    // Add to the world
    try c.getSystem().addConstraint(constraint.asConstraint());
    try c.getSystem().addStepListener(constraint.stepListener());
    return constraint;
}

fn checkOnGround(constraint: *const VehicleConstraint, settings: *const VehicleSettings, ground_id: BodyID) !void {
    // Between min and max suspension length
    const pos = constraint.getVehicleBody().getPosition();
    try expect(pos.getY() > settings.suspension_min_length + settings.wheel_offset_vertical + settings.half_vehicle_height);
    try expect(pos.getY() < settings.suspension_max_length + settings.wheel_offset_vertical + settings.half_vehicle_height);

    // Wheels touching ground
    for (constraint.getWheels()) |w|
        try expect(w.getContactBodyID().eql(ground_id));
}

fn wheeledController(constraint: *VehicleConstraint) *WheeledVehicleController {
    return virtual.downcast(WheeledVehicleController, constraint.getControllerMut());
}

fn simulate(c: *PhysicsTestContext, time: f32) !void {
    _ = try c.simulate(time, {});
}

test "TestBasicWheeledVehicle" {
    var c: PhysicsTestContext = undefined;
    try c.init(testing_allocator, .{});
    defer c.deinit();
    const floor_id = (try c.createFloor()).getID();

    const settings: VehicleSettings = .{};
    const constraint = try addVehicle(&c, &settings);
    const body = constraint.getVehicleBody();
    const controller = wheeledController(constraint);

    // Should start at specified position
    try checkApproxEqual(body.getPosition(), settings.position, .{});

    // After 1 step we should not be at ground yet
    _ = try c.simulateSingleStep();
    for (constraint.getWheels()) |w|
        try expect(w.getContactBodyID().isInvalid());
    try expect(controller.getTransmission().getCurrentGear() == 0);

    // After 1 second we should be on ground but not moving horizontally
    try simulate(&c, 1.0);
    try checkOnGround(constraint, &settings, floor_id);
    const pos1 = body.getPosition();
    try checkApproxEqual(pos1.getX(), @as(Real, 0), .{}); // Not moving horizontally
    try checkApproxEqual(pos1.getZ(), @as(Real, 0), .{});
    try expect(controller.getTransmission().getCurrentGear() == 0);

    // Start driving forward
    controller.setDriverInput(1.0, 0.0, 0.0, 0.0);
    c.getBodyInterface().activateBody(body.getID());
    try simulate(&c, 2.0);
    try checkOnGround(constraint, &settings, floor_id);
    const pos2 = body.getPosition();
    try checkApproxEqual(pos2.getX(), @as(Real, 0), .{ .tolerance = 1.0e-2 }); // Not moving left/right
    try expect(pos2.getZ() > pos1.getZ() + 1.0); // Moving in Z direction
    var vel = body.getLinearVelocity();
    try checkApproxEqual(vel.getX(), 0, .{ .tolerance = 2.0e-2 }); // Not moving left/right
    try expect(vel.getZ() > 1.0); // Moving in Z direction
    try expect(controller.getTransmission().getCurrentGear() > 0);

    // Brake
    controller.setDriverInput(0.0, 0.0, 1.0, 0.0);
    c.getBodyInterface().activateBody(body.getID());
    try simulate(&c, 5.0);
    try checkOnGround(constraint, &settings, floor_id);
    try expect(!body.isActive()); // Car should have gone to sleep
    const pos3 = body.getPosition();
    try checkApproxEqual(pos3.getX(), @as(Real, 0), .{ .tolerance = 2.0e-2 }); // Not moving left/right
    try expect(pos3.getZ() > pos2.getZ() + 1.0); // Moving in Z direction while braking
    vel = body.getLinearVelocity();
    try checkApproxEqual(vel, Vec3.zero(), .{ .tolerance = 1.0e-3 }); // Not moving

    // Start driving backwards
    controller.setDriverInput(-1.0, 0.0, 0.0, 0.0);
    c.getBodyInterface().activateBody(body.getID());
    try simulate(&c, 2.0);
    try checkOnGround(constraint, &settings, floor_id);
    const pos4 = body.getPosition();
    try checkApproxEqual(pos4.getX(), @as(Real, 0), .{ .tolerance = 3.0e-2 }); // Not moving left/right
    try expect(pos4.getZ() < pos3.getZ() - 1.0); // Moving in -Z direction
    vel = body.getLinearVelocity();
    try checkApproxEqual(vel.getX(), 0, .{ .tolerance = 5.0e-2 }); // Not moving left/right
    try expect(vel.getZ() < -1.0); // Moving in -Z direction
    try expect(controller.getTransmission().getCurrentGear() < 0);

    // Brake
    controller.setDriverInput(0.0, 0.0, 1.0, 0.0);
    c.getBodyInterface().activateBody(body.getID());
    try simulate(&c, 5.0);
    try checkOnGround(constraint, &settings, floor_id);
    try expect(!body.isActive()); // Car should have gone to sleep
    const pos5 = body.getPosition();
    try checkApproxEqual(pos5.getX(), @as(Real, 0), .{ .tolerance = 7.0e-2 }); // Not moving left/right
    try expect(pos5.getZ() < pos4.getZ() - 1.0); // Moving in -Z direction while braking
    vel = body.getLinearVelocity();
    try checkApproxEqual(vel, Vec3.zero(), .{ .tolerance = 1.0e-3 }); // Not moving

    // Turn right
    controller.setDriverInput(1.0, 1.0, 0.0, 0.0);
    c.getBodyInterface().activateBody(body.getID());
    try simulate(&c, 2.0);
    try checkOnGround(constraint, &settings, floor_id);
    var omega = body.getAngularVelocity();
    try expect(omega.getY() < -0.4); // Rotating right
    try expect(controller.getTransmission().getCurrentGear() > 0);

    // Hand brake
    controller.setDriverInput(0.0, 0.0, 0.0, 1.0);
    c.getBodyInterface().activateBody(body.getID());
    try simulate(&c, 7.0);
    try checkOnGround(constraint, &settings, floor_id);
    try expect(!body.isActive()); // Car should have gone to sleep
    vel = body.getLinearVelocity();
    try checkApproxEqual(vel, Vec3.zero(), .{ .tolerance = 1.0e-3 }); // Not moving

    // Turn left
    controller.setDriverInput(1.0, -1.0, 0.0, 0.0);
    c.getBodyInterface().activateBody(body.getID());
    try simulate(&c, 2.0);
    try checkOnGround(constraint, &settings, floor_id);
    omega = body.getAngularVelocity();
    try expect(omega.getY() > 0.4); // Rotating left
    try expect(controller.getTransmission().getCurrentGear() > 0);
}

test "TestLSDifferential" {
    const Test = struct {
        block_position: RVec3, // Location of the box under the vehicle
        four_wheel_drive: bool, // 4WD or not
        fb_ls_ratio: f32, // Limited slip ratio front-back
        lr_ls_ratio: f32, // Limited slip ratio left-right
        fl_has_contact_pre: bool, // Which wheels should be in contact with the ground prior to the test
        fr_has_contact_pre: bool,
        bl_has_contact_pre: bool,
        br_has_contact_pre: bool,
        should_move: bool, // If the vehicle should be able to drive off the block
    };

    const flt_max = math.flt_max;
    const tests = [_]Test{
        // Block Position,                4WD,   FBSlip,  LRSlip   FLPre, FRPre, BLPre, BRPre, ShouldMove
        .{ .block_position = RVec3.init(1, 0.5, 0), .four_wheel_drive = true, .fb_ls_ratio = flt_max, .lr_ls_ratio = flt_max, .fl_has_contact_pre = false, .fr_has_contact_pre = true, .bl_has_contact_pre = false, .br_has_contact_pre = true, .should_move = false }, // Block left, no limited slip -> vehicle can't move
        .{ .block_position = RVec3.init(1, 0.5, 0), .four_wheel_drive = true, .fb_ls_ratio = 1.4, .lr_ls_ratio = flt_max, .fl_has_contact_pre = false, .fr_has_contact_pre = true, .bl_has_contact_pre = false, .br_has_contact_pre = true, .should_move = false }, // Block left, only FB limited slip -> vehicle can't move
        .{ .block_position = RVec3.init(1, 0.5, 0), .four_wheel_drive = true, .fb_ls_ratio = 1.4, .lr_ls_ratio = 1.4, .fl_has_contact_pre = false, .fr_has_contact_pre = true, .bl_has_contact_pre = false, .br_has_contact_pre = true, .should_move = true }, // Block left, limited slip -> vehicle drives off
        .{ .block_position = RVec3.init(-1, 0.5, 0), .four_wheel_drive = true, .fb_ls_ratio = flt_max, .lr_ls_ratio = flt_max, .fl_has_contact_pre = true, .fr_has_contact_pre = false, .bl_has_contact_pre = true, .br_has_contact_pre = false, .should_move = false }, // Block right, no limited slip -> vehicle can't move
        .{ .block_position = RVec3.init(-1, 0.5, 0), .four_wheel_drive = true, .fb_ls_ratio = 1.4, .lr_ls_ratio = flt_max, .fl_has_contact_pre = true, .fr_has_contact_pre = false, .bl_has_contact_pre = true, .br_has_contact_pre = false, .should_move = false }, // Block right, only FB limited slip -> vehicle can't move
        .{ .block_position = RVec3.init(-1, 0.5, 0), .four_wheel_drive = true, .fb_ls_ratio = 1.4, .lr_ls_ratio = 1.4, .fl_has_contact_pre = true, .fr_has_contact_pre = false, .bl_has_contact_pre = true, .br_has_contact_pre = false, .should_move = true }, // Block right, limited slip -> vehicle drives off
        .{ .block_position = RVec3.init(0, 0.5, 1.5), .four_wheel_drive = true, .fb_ls_ratio = flt_max, .lr_ls_ratio = flt_max, .fl_has_contact_pre = false, .fr_has_contact_pre = false, .bl_has_contact_pre = true, .br_has_contact_pre = true, .should_move = false }, // Block front, no limited slip -> vehicle can't move
        .{ .block_position = RVec3.init(0, 0.5, 1.5), .four_wheel_drive = true, .fb_ls_ratio = 1.4, .lr_ls_ratio = flt_max, .fl_has_contact_pre = false, .fr_has_contact_pre = false, .bl_has_contact_pre = true, .br_has_contact_pre = true, .should_move = true }, // Block front, only FB limited slip -> vehicle drives off
        .{ .block_position = RVec3.init(0, 0.5, 1.5), .four_wheel_drive = true, .fb_ls_ratio = 1.4, .lr_ls_ratio = 1.4, .fl_has_contact_pre = false, .fr_has_contact_pre = false, .bl_has_contact_pre = true, .br_has_contact_pre = true, .should_move = true }, // Block front, limited slip -> vehicle drives off
        .{ .block_position = RVec3.init(0, 0.5, 1.5), .four_wheel_drive = false, .fb_ls_ratio = 1.4, .lr_ls_ratio = 1.4, .fl_has_contact_pre = false, .fr_has_contact_pre = false, .bl_has_contact_pre = true, .br_has_contact_pre = true, .should_move = false }, // Block front, limited slip, 2WD -> vehicle can't move
        .{ .block_position = RVec3.init(0, 0.5, -1.5), .four_wheel_drive = true, .fb_ls_ratio = flt_max, .lr_ls_ratio = flt_max, .fl_has_contact_pre = true, .fr_has_contact_pre = true, .bl_has_contact_pre = false, .br_has_contact_pre = false, .should_move = false }, // Block back, no limited slip -> vehicle can't move
        .{ .block_position = RVec3.init(0, 0.5, -1.5), .four_wheel_drive = true, .fb_ls_ratio = 1.4, .lr_ls_ratio = flt_max, .fl_has_contact_pre = true, .fr_has_contact_pre = true, .bl_has_contact_pre = false, .br_has_contact_pre = false, .should_move = true }, // Block back, only FB limited slip -> vehicle drives off
        .{ .block_position = RVec3.init(0, 0.5, -1.5), .four_wheel_drive = true, .fb_ls_ratio = 1.4, .lr_ls_ratio = 1.4, .fl_has_contact_pre = true, .fr_has_contact_pre = true, .bl_has_contact_pre = false, .br_has_contact_pre = false, .should_move = true }, // Block back, limited slip -> vehicle drives off
        .{ .block_position = RVec3.init(0, 0.5, -1.5), .four_wheel_drive = false, .fb_ls_ratio = 1.4, .lr_ls_ratio = 1.4, .fl_has_contact_pre = true, .fr_has_contact_pre = true, .bl_has_contact_pre = false, .br_has_contact_pre = false, .should_move = true }, // Block back, limited slip, 2WD -> vehicle drives off
    };

    // Give the wheels extra grip
    const ExtraGrip = struct {
        fn call(context: ?*anyopaque, wheel_index: u32, longitudinal_impulse: *f32, lateral_impulse: *f32, suspension_impulse: f32, longitudinal_friction: f32, lateral_friction: f32, longitudinal_slip: f32, lateral_slip: f32, delta_time: f32) void {
            _ = .{ context, wheel_index, longitudinal_slip, lateral_slip, delta_time };
            longitudinal_impulse.* = 10.0 * longitudinal_friction * suspension_impulse;
            lateral_impulse.* = lateral_friction * suspension_impulse;
        }
    };

    for (tests) |t| {
        var c: PhysicsTestContext = undefined;
        try c.init(testing_allocator, .{});
        defer c.deinit();
        const floor_id = (try c.createFloor()).getID();

        // Box under left side of the vehicle, left wheels won't be touching the ground
        const box = try c.createBox(t.block_position, Quat.identity(), .static, .discrete, Layers.non_moving, Vec3.replicate(0.5), .{});
        box.setFriction(1.0);

        // Create vehicle
        var settings: VehicleSettings = .{};
        settings.four_wheel_drive = t.four_wheel_drive;
        settings.front_back_limited_slip_ratio = t.fb_ls_ratio;
        settings.left_right_limited_slip_ratio = t.lr_ls_ratio;

        const constraint = try addVehicle(&c, &settings);
        const body = constraint.getVehicleBody();
        const controller = wheeledController(constraint);

        controller.setTireMaxImpulseCallback(.{ .function = ExtraGrip.call });

        // Simulate till vehicle rests on block
        var vehicle_on_floor = false;
        var time: f32 = 0;
        while (time < 2.0) : (time += c.getDeltaTime()) {
            _ = try c.simulateSingleStep();

            // Check pre condition
            if ((constraint.getWheel(fl_wheel).getContactBodyID().eql(if (t.fl_has_contact_pre) floor_id else BodyID.invalid)) and
                (constraint.getWheel(fr_wheel).getContactBodyID().eql(if (t.fr_has_contact_pre) floor_id else BodyID.invalid)) and
                (constraint.getWheel(bl_wheel).getContactBodyID().eql(if (t.bl_has_contact_pre) floor_id else BodyID.invalid)) and
                (constraint.getWheel(br_wheel).getContactBodyID().eql(if (t.br_has_contact_pre) floor_id else BodyID.invalid)))
            {
                vehicle_on_floor = true;
                break;
            }
        }
        try expect(vehicle_on_floor);
        try checkApproxEqual(body.getPosition().getZ(), @as(Real, 0), .{ .tolerance = 0.03 });

        // Start driving
        controller.setDriverInput(1.0, 0, 0, 0);
        c.getBodyInterface().activateBody(body.getID());
        try simulate(&c, 2.0);

        // Check if vehicle had traction
        if (t.should_move)
            try expect(body.getPosition().getZ() > 0.5)
        else
            try checkApproxEqual(body.getPosition().getZ(), @as(Real, 0), .{ .tolerance = 0.06 });
    }
}

test "WheeledVehicle: the ray, cast sphere and cast cylinder testers put the wheels on the floor" {
    for ([_]TesterKind{ .ray, .cast_sphere, .cast_cylinder }) |kind| {
        var c: PhysicsTestContext = undefined;
        try c.init(testing_allocator, .{});
        defer c.deinit();
        const floor_id = (try c.createFloor()).getID();

        var settings: VehicleSettings = .{};
        settings.use_cast_sphere = kind == .cast_sphere;
        settings.use_cast_cylinder = kind == .cast_cylinder;
        const constraint = try addVehicle(&c, &settings);
        try simulate(&c, 1.0);
        try checkOnGround(constraint, &settings, floor_id);

        // The wheel transforms follow the suspension: the wheel center is the suspension length below the attachment
        for (constraint.getWheels(), 0..) |w, i| {
            const local = constraint.getWheelLocalTransform(@intCast(i), Vec3.axisX(), Vec3.axisY());
            const expected = w.getSettings().position.add(w.getSettings().suspension_direction.mulScalar(w.getSuspensionLength()));
            try checkApproxEqual(local.getTranslation(), expected, .{ .tolerance = 1.0e-5 });
            try expect(w.getContactNormal().dot(Vec3.axisY()) > 0.99);
            try expect(w.hasContact() and !w.hasHitHardPoint());
        }

        // Driving forward moves the car forward
        const controller = wheeledController(constraint);
        controller.setDriverInput(1.0, 0.0, 0.0, 0.0);
        c.getBodyInterface().activateBody(constraint.getVehicleBody().getID());
        try simulate(&c, 1.0);
        try expect(constraint.getVehicleBody().getPosition().getZ() > 0.5);
        try expect(controller.getEngine().getCurrentRPM() > controller.getEngine().base.min_rpm);
        try expect(controller.getWheelSpeedAtClutch() > 0.0);
    }
}

test "WheeledVehicle: constraint and controller settings, binary state of the settings" {
    var c: PhysicsTestContext = undefined;
    try c.init(testing_allocator, .{});
    defer c.deinit();
    const settings: VehicleSettings = .{ .four_wheel_drive = true };
    const constraint = try addVehicle(&c, &settings);
    const base = constraint.asConstraint();
    try expect(base.getType() == .constraint);
    try expect(base.getSubType() == .vehicle);
    try checkApproxEqual(constraint.getMaxPitchRollAngle(), math.degreesToRadians(60.0), .{ .tolerance = 1.0e-6 });
    try expect(constraint.getVehicleCollisionTester() != null);
    try expect(constraint.getWheels().len == 4);
    try expect(constraint.getAntiRollBars().len == 2);

    var copy = try base.getConstraintSettings();
    defer copy.deinit();
    const s = virtual.downcast(VehicleConstraintSettings, copy.get().?);
    try expect(s.wheels.items.len == 4);
    for (s.wheels.items, constraint.getWheels()) |ws, w|
        try expect(ws.ptr.? == @constCast(w.getSettings())); // The wheels share their settings
    try expect(s.anti_roll_bars.items.len == 2 and s.anti_roll_bars.items[1].left_wheel == bl_wheel);
    try checkApproxEqual(s.max_pitch_roll_angle, math.degreesToRadians(60.0), .{ .tolerance = 1.0e-6 });
    const cs = virtual.downcast(WheeledVehicleControllerSettings, s.controller.ptr.?);
    try expect(cs.differentials.items.len == 2);
    try expect(cs.differentials.items[1].engine_torque_ratio == 0.5);
    try expect(cs.engine.normalized_torque.points.items.len == 3);

    // The binary state of the settings (without the wheels: Jolt cannot restore settings with wheels, E6)
    for (s.wheels.items) |*w| w.deinit();
    s.wheels.clearRetainingCapacity();
    var out: std.Io.Writer.Allocating = .init(testing_allocator);
    defer out.deinit();
    var out_wrapper: zolt.StreamOutWrapper = .init(&out.writer);
    s.saveBinaryState(out_wrapper.streamOut());

    var restored = VehicleConstraintSettings.init(testing_allocator);
    defer restored.asSettings().deinit();
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper: zolt.StreamInWrapper = .init(&reader);
    var hash: u32 = 0;
    in_wrapper.streamIn().read(&hash); // Read by sRestoreFromBinaryState
    try expect(hash == ConstraintSettings.rttiHash("VehicleConstraintSettings"));
    try restored.restoreBinaryState(in_wrapper.streamIn());
    try expect(!in_wrapper.isEOF() and !in_wrapper.isFailed());
    try expect(restored.anti_roll_bars.items.len == 2);
    const rcs = virtual.downcast(WheeledVehicleControllerSettings, restored.controller.ptr.?);
    try expect(rcs.differentials.items.len == 2 and rcs.differentials.items[1].left_wheel == bl_wheel);

    // Saved again: the same bytes
    var out2: std.Io.Writer.Allocating = .init(testing_allocator);
    defer out2.deinit();
    var out_wrapper2: zolt.StreamOutWrapper = .init(&out2.writer);
    restored.saveBinaryState(out_wrapper2.streamOut());
    try std.testing.expectEqualSlices(u8, out.written(), out2.written());
}

/// Drive a vehicle and record the state of the physics system after every update
fn driveAndRecord(allocator: Allocator, worker_threads: i32, num_updates: usize, states: *std.ArrayList(u8)) !void {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{ .worker_threads = worker_threads });
    defer c.deinit();
    _ = try c.createFloor();

    // A box that the vehicle drives into
    _ = try c.createBox(RVec3.init(0.3, 1.0, 6.0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});

    const settings: VehicleSettings = .{};
    const constraint = try addVehicle(&c, &settings);
    const controller = wheeledController(constraint);

    for (0..num_updates) |i| {
        if (i == 30) controller.setDriverInput(1.0, 0.3, 0.0, 0.0);
        if (i == 120) controller.setDriverInput(0.0, 0.0, 1.0, 0.5);
        _ = try c.simulateSingleStep();
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        try c.getSystem().saveState(recorder.recorder(), .{});
        try states.appendSlice(allocator, recorder.getData());
    }
}

test "WheeledVehicle: the simulation does not depend on the number of threads" {
    var reference: std.ArrayList(u8) = .empty;
    defer reference.deinit(testing_allocator);
    try driveAndRecord(testing_allocator, 0, 160, &reference);
    for ([_]i32{ 1, 3 }) |threads| {
        var states: std.ArrayList(u8) = .empty;
        defer states.deinit(testing_allocator);
        try driveAndRecord(testing_allocator, threads, 160, &states);
        try std.testing.expectEqualSlices(u8, reference.items, states.items);
    }
}

test "WheeledVehicle: SaveState / RestoreState in the middle of a drive" {
    var c: PhysicsTestContext = undefined;
    try c.init(testing_allocator, .{});
    defer c.deinit();
    _ = try c.createFloor();
    const settings: VehicleSettings = .{};
    const constraint = try addVehicle(&c, &settings);
    const controller = wheeledController(constraint);
    try simulate(&c, 1.0);
    controller.setDriverInput(1.0, -0.5, 0.0, 0.0);
    c.getBodyInterface().activateBody(constraint.getVehicleBody().getID());
    try simulate(&c, 0.5);

    // Save, simulate, restore and simulate again: the same states
    var snapshot = StateRecorderImpl.init(testing_allocator);
    defer snapshot.deinit();
    try c.getSystem().saveState(snapshot.recorder(), .{});

    var first: std.ArrayList(u8) = .empty;
    defer first.deinit(testing_allocator);
    for (0..30) |_| {
        _ = try c.simulateSingleStep();
        var recorder = StateRecorderImpl.init(testing_allocator);
        defer recorder.deinit();
        try c.getSystem().saveState(recorder.recorder(), .{});
        try first.appendSlice(testing_allocator, recorder.getData());
    }
    const gear = controller.getTransmission().getCurrentGear();

    snapshot.rewind();
    try expect(try c.getSystem().restoreState(snapshot.recorder(), .{}));
    var second: std.ArrayList(u8) = .empty;
    defer second.deinit(testing_allocator);
    for (0..30) |_| {
        _ = try c.simulateSingleStep();
        var recorder = StateRecorderImpl.init(testing_allocator);
        defer recorder.deinit();
        try c.getSystem().saveState(recorder.recorder(), .{});
        try second.appendSlice(testing_allocator, recorder.getData());
    }
    try std.testing.expectEqualSlices(u8, first.items, second.items);
    try expect(gear == controller.getTransmission().getCurrentGear());
}

test "WheeledVehicle: creation, settings and controller settings out of memory" {
    var c: PhysicsTestContext = undefined;
    try c.init(testing_allocator, .{});
    defer c.deinit();
    const settings: VehicleSettings = .{ .four_wheel_drive = true };
    const body = try addVehicleBody(&c, &settings);

    const S = struct {
        fn run(allocator: Allocator, car_body: *Body, vehicle_settings: *const VehicleSettings) !void {
            var vehicle = try makeVehicleSettings(allocator, vehicle_settings);
            defer vehicle.asSettings().deinit();

            const constraint = try VehicleConstraint.create(allocator, car_body, &vehicle);
            var constraint_ref = Ref(Constraint).init(constraint.asConstraint());
            defer constraint_ref.deinit();

            var tester = RefConst(VehicleCollisionTester).init(try createTester(allocator, vehicle_settings));
            defer tester.deinit();
            constraint.setVehicleCollisionTester(tester.ptr);

            var copy = try constraint.asConstraint().getConstraintSettings();
            defer copy.deinit();
            var controller_settings = try constraint.getController().getSettings();
            defer controller_settings.deinit();
        }
    };
    try std.testing.checkAllAllocationFailures(testing_allocator, S.run, .{ body, &settings });
}
