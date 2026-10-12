//! Port of: UnitTests/Physics/SensorTests.cpp
//! Status: complete
//!
//! - The callbacks cannot fail a test: Jolt's CHECK (and JPH_ASSERT) inside a callback is remembered by the listener
//!   and reported after the update (`LoggingContactListener.checkError`).
//! - `SensorOverridingListener : public LoggingContactListener` embeds a LoggingContactListener (`base`), forwards every
//!   callback to it and overrides OnContactAdded / OnContactPersisted like Jolt's derived class.
//! - The `sCollideBodyVsBody*` functions are `PhysicsSystem.SimCollideBodyVsBody` function pointers (Jolt: a
//!   std::function).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;
const LoggingContactListener = @import("../LoggingContactListener.zig").LoggingContactListener;

const Allocator = std.mem.Allocator;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const BoxShape = zolt.BoxShape;
const BoxShapeSettings = zolt.BoxShapeSettings;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollisionDispatch = zolt.CollisionDispatch;
const ContactManifold = zolt.ContactManifold;
const ContactSettings = zolt.ContactSettings;
const Float3 = zolt.Float3;
const IndexedTriangle = zolt.IndexedTriangle;
const Mat44 = zolt.Mat44;
const MeshShapeSettings = zolt.MeshShapeSettings;
const ObjectLayer = zolt.ObjectLayer;
const PhysicsSystem = zolt.PhysicsSystem;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Ref = zolt.Ref;
const ShapeFilter = zolt.ShapeFilter;
const ShapeSettings = zolt.ShapeSettings;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const SubShapeIDPair = zolt.SubShapeIDPair;
const ValidateResult = zolt.ValidateResult;
const Vec3 = zolt.Vec3;

const expect = std.testing.expect;
const checkApproxEqual = fw.checkApproxEqual;
const allocator = std.testing.allocator;

const EType = LoggingContactListener.Type;

/// c.Simulate(time) with the CHECKs that the listener made inside its callbacks
fn simulate(c: *PhysicsTestContext, listener: *const LoggingContactListener, total_time: f32) !void {
    _ = try c.simulate(total_time, {});
    try listener.checkError();
}

/// c.SimulateSingleStep() with the CHECKs that the listener made inside its callbacks
fn simulateSingleStep(c: *PhysicsTestContext, listener: *const LoggingContactListener) !void {
    _ = try c.simulateSingleStep();
    try listener.checkError();
}

/// BodyCreationSettings(new BoxShape(Vec3::sReplicate(1)), position, Quat::sIdentity(), motion type, layer) of the sensors
fn sensorSettings(position: RVec3, motion_type: zolt.MotionType, layer: ObjectLayer) !BodyCreationSettings {
    return BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(1), .{})).asShape(), position, Quat.identity(), motion_type, layer);
}

test "TestDynamicVsSensor" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    // Register listener
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Sensor
    var sensor_settings = try sensorSettings(RVec3.zero(), .static, Layers.sensor);
    defer sensor_settings.deinit();
    sensor_settings.is_sensor = true;
    const sensor_id = try c.getBodyInterface().createAndAddBody(&sensor_settings, .dont_activate);

    // Dynamic body moving downwards
    const dynamic = try c.createBox(RVec3.init(0, 2, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});
    dynamic.setLinearVelocity(Vec3.init(0, -1, 0));

    // After a single step the dynamic object should not have touched the sensor yet
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 0);

    // After half a second and one step we should be touching the sensor
    try simulate(&c, &listener, 0.5 + c.getStepDeltaTime());
    try expect(listener.contains(.add, dynamic.getID(), sensor_id));
    listener.clear();

    // The next step we require that the contact persists
    try simulateSingleStep(&c, &listener);
    try expect(listener.contains(.persist, dynamic.getID(), sensor_id));
    try expect(!listener.contains(.remove, dynamic.getID(), sensor_id));
    listener.clear();

    // After 3 more seconds we should have left the sensor at the bottom side
    try simulate(&c, &listener, 3.0);
    try expect(listener.contains(.remove, dynamic.getID(), sensor_id));
    try checkApproxEqual(dynamic.getPosition(), RVec3.init(0, -1.5 - 3.0 * c.getDeltaTime(), 0), .{ .tolerance = 1.0e-4 });
}

test "TestKinematicVsSensor" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Register listener
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Sensor
    var sensor_settings = try sensorSettings(RVec3.zero(), .static, Layers.sensor);
    defer sensor_settings.deinit();
    sensor_settings.is_sensor = true;
    const sensor_id = try c.getBodyInterface().createAndAddBody(&sensor_settings, .dont_activate);

    // Kinematic body moving downwards
    const kinematic = try c.createBox(RVec3.init(0, 2, 0), Quat.identity(), .kinematic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});
    kinematic.setLinearVelocity(Vec3.init(0, -1, 0));

    // After a single step the kinematic object should not have touched the sensor yet
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 0);

    // After half a second and one step we should be touching the sensor
    try simulate(&c, &listener, 0.5 + c.getStepDeltaTime());
    try expect(listener.contains(.add, kinematic.getID(), sensor_id));
    listener.clear();

    // The next step we require that the contact persists
    try simulateSingleStep(&c, &listener);
    try expect(listener.contains(.persist, kinematic.getID(), sensor_id));
    try expect(!listener.contains(.remove, kinematic.getID(), sensor_id));
    listener.clear();

    // After 3 more seconds we should have left the sensor at the bottom side
    try simulate(&c, &listener, 3.0);
    try expect(listener.contains(.remove, kinematic.getID(), sensor_id));
    try checkApproxEqual(kinematic.getPosition(), RVec3.init(0, -1.5 - 3.0 * c.getDeltaTime(), 0), .{ .tolerance = 1.0e-4 });
}

test "TestKinematicVsKinematicSensor" {
    // Same as TestKinematicVsSensor but with the sensor being an active kinematic body

    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Register listener
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Kinematic sensor
    var sensor_settings = try sensorSettings(RVec3.zero(), .kinematic, Layers.sensor);
    defer sensor_settings.deinit();
    sensor_settings.is_sensor = true;
    const sensor_id = try c.getBodyInterface().createAndAddBody(&sensor_settings, .activate);

    // Kinematic body moving downwards
    const kinematic = try c.createBox(RVec3.init(0, 2, 0), Quat.identity(), .kinematic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});
    kinematic.setLinearVelocity(Vec3.init(0, -1, 0));

    // After a single step the kinematic object should not have touched the sensor yet
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 0);

    // After half a second and one step we should be touching the sensor
    try simulate(&c, &listener, 0.5 + c.getStepDeltaTime());
    try expect(listener.contains(.add, kinematic.getID(), sensor_id));
    listener.clear();

    // The next step we require that the contact persists
    try simulateSingleStep(&c, &listener);
    try expect(listener.contains(.persist, kinematic.getID(), sensor_id));
    try expect(!listener.contains(.remove, kinematic.getID(), sensor_id));
    listener.clear();

    // After 3 more seconds we should have left the sensor at the bottom side
    try simulate(&c, &listener, 3.0);
    try expect(listener.contains(.remove, kinematic.getID(), sensor_id));
    try checkApproxEqual(kinematic.getPosition(), RVec3.init(0, -1.5 - 3.0 * c.getDeltaTime(), 0), .{ .tolerance = 1.0e-4 });
}

test "TestKinematicVsKinematicSensorReversed" {
    // Same as TestKinematicVsKinematicSensor but with bodies created in reverse order (this matters for Body::sFindCollidingPairsCanCollide because MotionProperties::mIndexInActiveBodies is swapped between the bodies)

    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Register listener
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Kinematic body moving downwards
    const kinematic = try c.createBox(RVec3.init(0, 2, 0), Quat.identity(), .kinematic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});
    kinematic.setLinearVelocity(Vec3.init(0, -1, 0));

    // Kinematic sensor
    var sensor_settings = try sensorSettings(RVec3.zero(), .kinematic, Layers.sensor);
    defer sensor_settings.deinit();
    sensor_settings.is_sensor = true;
    const sensor_id = try c.getBodyInterface().createAndAddBody(&sensor_settings, .activate);

    // After a single step the kinematic object should not have touched the sensor yet
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 0);

    // After half a second and one step we should be touching the sensor
    try simulate(&c, &listener, 0.5 + c.getStepDeltaTime());
    try expect(listener.contains(.add, kinematic.getID(), sensor_id));
    listener.clear();

    // The next step we require that the contact persists
    try simulateSingleStep(&c, &listener);
    try expect(listener.contains(.persist, kinematic.getID(), sensor_id));
    try expect(!listener.contains(.remove, kinematic.getID(), sensor_id));
    listener.clear();

    // After 3 more seconds we should have left the sensor at the bottom side
    try simulate(&c, &listener, 3.0);
    try expect(listener.contains(.remove, kinematic.getID(), sensor_id));
    try checkApproxEqual(kinematic.getPosition(), RVec3.init(0, -1.5 - 3.0 * c.getDeltaTime(), 0), .{ .tolerance = 1.0e-4 });
}

test "TestDynamicSleepingVsStaticSensor" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Register listener
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Sensor
    var sensor_settings = try sensorSettings(RVec3.zero(), .static, Layers.sensor);
    defer sensor_settings.deinit();
    sensor_settings.is_sensor = true;
    const sensor = (try c.getBodyInterface().createBody(&sensor_settings)) orelse return error.OutOfBodies;
    try c.getBodyInterface().addBody(sensor.getID(), .dont_activate);

    // Floor
    const floor = try c.createFloor();

    // Dynamic body on floor (make them penetrate)
    const dynamic = try c.createBox(RVec3.init(0, 0.5 - c.getSystem().getPhysicsSettings().max_penetration_distance, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{ .activation = .dont_activate });

    // After a single step (because the object is sleeping) there should not be a contact
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 0);

    // The dynamic object should not be part of an island
    try expect(!sensor.isActive());
    try expect(dynamic.getMotionProperties().getIslandIndexInternal() == Body.inactive_index);

    // Activate the body
    c.getBodyInterface().activateBody(dynamic.getID());

    // After a single step we should have detected the collision with the floor and the sensor
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 4);
    try expect(listener.contains(.validate, floor.getID(), dynamic.getID()));
    try expect(listener.contains(.add, floor.getID(), dynamic.getID()));
    try expect(listener.contains(.validate, sensor.getID(), dynamic.getID()));
    try expect(listener.contains(.add, sensor.getID(), dynamic.getID()));
    listener.clear();

    // The dynamic object should be part of an island now
    try expect(!sensor.isActive());
    try expect(dynamic.getMotionProperties().getIslandIndexInternal() != Body.inactive_index);

    // After a second the body should have gone to sleep and the contacts should have been removed
    try simulate(&c, &listener, 1.0);
    try expect(!dynamic.isActive());
    try expect(listener.contains(.remove, floor.getID(), dynamic.getID()));
    try expect(listener.contains(.remove, sensor.getID(), dynamic.getID()));

    // The dynamic object should not be part of an island
    try expect(!sensor.isActive());
    try expect(dynamic.getMotionProperties().getIslandIndexInternal() == Body.inactive_index);
}

test "TestDynamicSleepingVsKinematicSensor" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Register listener
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Kinematic sensor that is active (so will keep detecting contacts with sleeping bodies)
    var sensor_settings = try sensorSettings(RVec3.zero(), .kinematic, Layers.sensor);
    defer sensor_settings.deinit();
    sensor_settings.is_sensor = true;
    const sensor = (try c.getBodyInterface().createBody(&sensor_settings)) orelse return error.OutOfBodies;
    try c.getBodyInterface().addBody(sensor.getID(), .activate);

    // Floor
    const floor = try c.createFloor();

    // Dynamic body on floor (make them penetrate)
    const dynamic = try c.createBox(RVec3.init(0, 0.5 - c.getSystem().getPhysicsSettings().max_penetration_distance, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{ .activation = .dont_activate });

    // After a single step, there should be a contact with the sensor only (the sensor is active)
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.validate, sensor.getID(), dynamic.getID()));
    try expect(listener.contains(.add, sensor.getID(), dynamic.getID()));
    listener.clear();

    // The sensor should be in its own island
    try expect(sensor.getMotionProperties().getIslandIndexInternal() != Body.inactive_index);
    try expect(dynamic.getMotionProperties().getIslandIndexInternal() == Body.inactive_index);

    // The second step, the contact with the sensor should have persisted
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 1);
    try expect(listener.contains(.persist, sensor.getID(), dynamic.getID()));
    listener.clear();

    // The sensor should still be in its own island
    try expect(sensor.getMotionProperties().getIslandIndexInternal() != Body.inactive_index);
    try expect(dynamic.getMotionProperties().getIslandIndexInternal() == Body.inactive_index);

    // Activate the body
    c.getBodyInterface().activateBody(dynamic.getID());

    // After a single step we should have detected collision with the floor and the collision with the sensor should have persisted
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 3);
    try expect(listener.contains(.validate, floor.getID(), dynamic.getID()));
    try expect(listener.contains(.add, floor.getID(), dynamic.getID()));
    try expect(listener.contains(.persist, sensor.getID(), dynamic.getID()));
    listener.clear();

    // The sensor should not be part of the same island as the dynamic body (they won't interact, so this is not needed)
    try expect(sensor.getMotionProperties().getIslandIndexInternal() != Body.inactive_index);
    try expect(dynamic.getMotionProperties().getIslandIndexInternal() != Body.inactive_index);
    try expect(sensor.getMotionProperties().getIslandIndexInternal() != dynamic.getMotionProperties().getIslandIndexInternal());

    // After another step we should have persisted the collision with the floor and sensor
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() >= 2); // Depending on if we used the contact cache or not there will be validate callbacks too
    try expect(listener.contains(.persist, floor.getID(), dynamic.getID()));
    try expect(!listener.contains(.remove, floor.getID(), dynamic.getID()));
    try expect(listener.contains(.persist, sensor.getID(), dynamic.getID()));
    try expect(!listener.contains(.remove, sensor.getID(), dynamic.getID()));
    listener.clear();

    // The same islands as the previous step should have been created
    try expect(sensor.getMotionProperties().getIslandIndexInternal() != Body.inactive_index);
    try expect(dynamic.getMotionProperties().getIslandIndexInternal() != Body.inactive_index);
    try expect(sensor.getMotionProperties().getIslandIndexInternal() != dynamic.getMotionProperties().getIslandIndexInternal());

    // After a second the body should have gone to sleep and the contacts with the floor should have been removed, but not with the sensor
    try simulate(&c, &listener, 1.0);
    try expect(!dynamic.isActive());
    try expect(listener.contains(.remove, floor.getID(), dynamic.getID()));
    try expect(!listener.contains(.remove, sensor.getID(), dynamic.getID()));
}

test "TestSensorVsSensor" {
    for (0..2) |test_index| {
        const sensor_detects_sensor = test_index == 1;

        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();

        // Register listener
        var listener: LoggingContactListener = .init(allocator);
        defer listener.deinit();
        c.getSystem().setContactListener(.init(&listener));

        // Depending on the iteration we either place the sensor in the moving layer which means it will collide with other sensors
        // or we put it in the sensor layer which means it won't collide with other sensors
        const layer: ObjectLayer = if (sensor_detects_sensor) Layers.moving else Layers.sensor;

        // Sensor 1
        var sensor_settings = try sensorSettings(RVec3.zero(), .static, layer);
        defer sensor_settings.deinit();
        sensor_settings.is_sensor = true;
        const sensor_id1 = try c.getBodyInterface().createAndAddBody(&sensor_settings, .dont_activate);

        // Sensor 2 moving downwards
        sensor_settings.motion_type = .kinematic;
        sensor_settings.position = RVec3.init(0, 3, 0);
        sensor_settings.is_sensor = true;
        sensor_settings.linear_velocity = Vec3.init(0, -2, 0);
        const sensor_id2 = try c.getBodyInterface().createAndAddBody(&sensor_settings, .activate);

        // After a single step the sensors should not touch yet
        try simulateSingleStep(&c, &listener);
        try expect(listener.getEntryCount() == 0);

        // After half a second and one step the sensors should be touching
        try simulate(&c, &listener, 0.5 + c.getDeltaTime());
        if (sensor_detects_sensor)
            try expect(listener.contains(.add, sensor_id1, sensor_id2))
        else
            try expect(listener.getEntryCount() == 0);
        listener.clear();

        // The next step we require that the contact persists
        try simulateSingleStep(&c, &listener);
        if (sensor_detects_sensor) {
            try expect(listener.contains(.persist, sensor_id1, sensor_id2));
            try expect(!listener.contains(.remove, sensor_id1, sensor_id2));
        } else try expect(listener.getEntryCount() == 0);
        listener.clear();

        // After 2 more seconds we should have left the sensor at the bottom side
        try simulate(&c, &listener, 2.0);
        if (sensor_detects_sensor)
            try expect(listener.contains(.remove, sensor_id1, sensor_id2))
        else
            try expect(listener.getEntryCount() == 0);
        try checkApproxEqual(c.getBodyInterface().getPosition(sensor_id2), sensor_settings.position.addVec3(sensor_settings.linear_velocity.mulScalar(2.5 + 3.0 * c.getDeltaTime())), .{ .tolerance = 1.0e-4 });
    }
}

/// `class SensorOverridingListener : public LoggingContactListener` of TestContactListenerMakesSensor(CCD): contacts with
/// `body_id` become sensor contacts
const SensorOverridingListener = struct {
    base: LoggingContactListener,
    body_id: BodyID = .invalid,
    /// Zolt: the JPH_ASSERT inside a callback failed
    assert_failed: bool = false,

    fn init() SensorOverridingListener {
        return .{ .base = .init(allocator) };
    }

    fn deinit(self: *SensorOverridingListener) void {
        self.base.deinit();
    }

    /// The CHECKs of the base class and the JPH_ASSERTs of the overrides
    fn checkError(self: *const SensorOverridingListener) !void {
        try self.base.checkError();
        if (self.assert_failed) return error.TestUnexpectedResult;
    }

    pub fn onContactValidate(self: *SensorOverridingListener, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult {
        return self.base.onContactValidate(body1, body2, base_offset, collision_result);
    }

    pub fn onContactAdded(self: *SensorOverridingListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        self.base.onContactAdded(body1, body2, manifold, settings);

        if (settings.is_sensor != false) self.assert_failed = true;
        if (body1.getID().eql(self.body_id) or body2.getID().eql(self.body_id))
            settings.is_sensor = true;
    }

    pub fn onContactPersisted(self: *SensorOverridingListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        self.base.onContactPersisted(body1, body2, manifold, settings);

        if (settings.is_sensor != false) self.assert_failed = true;
        if (body1.getID().eql(self.body_id) or body2.getID().eql(self.body_id))
            settings.is_sensor = true;
    }

    pub fn onContactRemoved(self: *SensorOverridingListener, sub_shape_pair: *const SubShapeIDPair) void {
        self.base.onContactRemoved(sub_shape_pair);
    }
};

test "TestContactListenerMakesSensor" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    // Register listener (mBodyThatSeesSensorID)
    var listener: SensorOverridingListener = .init();
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Body that will appear as a sensor to one object and as static to another
    var static_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.init(5, 1, 5), .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer static_settings.deinit();
    const static_id = try c.getBodyInterface().createAndAddBody(&static_settings, .dont_activate);

    // Dynamic body moving down that will do a normal collision
    const dynamic1 = try c.createBox(RVec3.init(-2, 2, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});
    dynamic1.setAllowSleeping(false);
    dynamic1.setLinearVelocity(Vec3.init(0, -1, 0));

    // Dynamic body moving down that will only see the static object as a sensor
    const dynamic2 = try c.createBox(RVec3.init(2, 2, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{});
    dynamic2.setAllowSleeping(false);
    dynamic2.setLinearVelocity(Vec3.init(0, -1, 0));
    listener.body_id = dynamic2.getID();

    // After a single step the dynamic object should not have touched the sensor yet
    _ = try c.simulateSingleStep();
    try listener.checkError();
    try expect(listener.base.getEntryCount() == 0);

    // After half a second both bodies should be touching the sensor
    _ = try c.simulate(0.5, {});
    try listener.checkError();
    try expect(listener.base.contains(.add, dynamic1.getID(), static_id));
    try expect(listener.base.contains(.add, dynamic2.getID(), static_id));
    listener.base.clear();

    // The next step we require that the contact persists
    _ = try c.simulateSingleStep();
    try listener.checkError();
    try expect(listener.base.contains(.persist, dynamic1.getID(), static_id));
    try expect(!listener.base.contains(.remove, dynamic1.getID(), static_id));
    try expect(listener.base.contains(.persist, dynamic2.getID(), static_id));
    try expect(!listener.base.contains(.remove, dynamic2.getID(), static_id));
    listener.base.clear();

    // After 3 more seconds one body should be resting on the static body, the other should have fallen through
    _ = try c.simulate(3.0 + c.getDeltaTime(), {});
    try listener.checkError();
    try expect(listener.base.contains(.persist, dynamic1.getID(), static_id));
    try expect(!listener.base.contains(.remove, dynamic1.getID(), static_id));
    try expect(listener.base.contains(.remove, dynamic2.getID(), static_id));
    try checkApproxEqual(dynamic1.getPosition(), RVec3.init(-2, 1.5, 0), .{ .tolerance = 5.0e-3 });
    try checkApproxEqual(dynamic2.getPosition(), RVec3.init(2, -1.5 - 3.0 * c.getDeltaTime(), 0), .{ .tolerance = 1.0e-4 });
}

test "TestContactListenerMakesSensorCCD" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    const penetration_slop = c.getSystem().getPhysicsSettings().penetration_slop;

    // Register listener (mBodyThatBecomesSensor)
    var listener: SensorOverridingListener = .init();
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Body that blocks the path
    var static_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.init(0.1, 10, 10), .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer static_settings.deinit();
    const static_id = try c.getBodyInterface().createAndAddBody(&static_settings, .dont_activate);

    // Dynamic body moving to the static object that will do a normal CCD collision
    const dynamic1_pos = RVec3.init(-0.5, 2, 0);
    const initial_velocity = Vec3.init(500, 0, 0);
    const dynamic1 = try c.createBox(dynamic1_pos, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(0.1), .{});
    dynamic1.setAllowSleeping(false);
    dynamic1.setLinearVelocity(initial_velocity);
    dynamic1.setRestitution(1.0);

    // Dynamic body moving through the static object that will become a sensor an thus pass through
    const dynamic2_pos = RVec3.init(-0.5, -2, 0);
    const dynamic2 = try c.createBox(dynamic2_pos, Quat.identity(), .dynamic, .linear_cast, Layers.moving, Vec3.replicate(0.1), .{});
    dynamic2.setAllowSleeping(false);
    dynamic2.setLinearVelocity(initial_velocity);
    dynamic2.setRestitution(1.0);
    listener.body_id = dynamic2.getID();

    // After a single step the we should have contact added callbacks for both bodies
    _ = try c.simulateSingleStep();
    try listener.checkError();
    try expect(listener.base.contains(.add, dynamic1.getID(), static_id));
    try expect(listener.base.contains(.add, dynamic2.getID(), static_id));
    listener.base.clear();
    try checkApproxEqual(dynamic1.getPosition(), dynamic1_pos.add(RVec3.init(0.3 + penetration_slop, 0, 0)), .{ .tolerance = 1.0e-4 }); // Dynamic 1 should have moved to the surface of the static body
    try checkApproxEqual(dynamic2.getPosition(), dynamic2_pos.addVec3(initial_velocity.mulScalar(c.getDeltaTime())), .{ .tolerance = 1.0e-4 }); // Dynamic 2 should have passed through the static body because it became a sensor

    // The next step the sensor should have its contact removed and the CCD body should have its contact persisted because it starts penetrating
    _ = try c.simulateSingleStep();
    try listener.checkError();
    try expect(listener.base.contains(.persist, dynamic1.getID(), static_id));
    try expect(listener.base.contains(.remove, dynamic2.getID(), static_id));
    listener.base.clear();

    // The next step all contacts have been removed
    _ = try c.simulateSingleStep();
    try listener.checkError();
    try expect(listener.base.contains(.remove, dynamic1.getID(), static_id));
    listener.base.clear();
}

test "TestSensorVsSubShapes" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    const bi = c.getBodyInterface();

    // Register listener
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Create sensor
    var sensor_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(5.0), .{})).asShape(), RVec3.init(0, 10, 0), Quat.identity(), .static, Layers.sensor);
    defer sensor_settings.deinit();
    sensor_settings.is_sensor = true;
    const sensor_id = try bi.createAndAddBody(&sensor_settings, .dont_activate);

    // We will be testing if we receive callbacks from the individual sub shapes
    const UserData = enum(u64) {
        bottom,
        middle,
        top,
    };

    // Create compound with 3 sub shapes
    const compound_settings = try StaticCompoundShapeSettings.create(allocator);
    var shape_settings = Ref(ShapeSettings).init(compound_settings.asShapeSettings());
    defer shape_settings.deinit();
    var shape1 = Ref(ShapeSettings).init((try BoxShapeSettings.create(allocator, Vec3.replicate(0.4), .{})).asShapeSettings());
    defer shape1.deinit();
    shape1.get().?.user_data = @intFromEnum(UserData.bottom);
    var shape2 = Ref(ShapeSettings).init((try BoxShapeSettings.create(allocator, Vec3.replicate(0.4), .{})).asShapeSettings());
    defer shape2.deinit();
    shape2.get().?.user_data = @intFromEnum(UserData.middle);
    var shape3 = Ref(ShapeSettings).init((try BoxShapeSettings.create(allocator, Vec3.replicate(0.4), .{})).asShapeSettings());
    defer shape3.deinit();
    shape3.get().?.user_data = @intFromEnum(UserData.top);
    try compound_settings.base.addShape(Vec3.init(0, -1.0, 0), Quat.identity(), shape1.get(), .{});
    try compound_settings.base.addShape(Vec3.init(0, 0.0, 0), Quat.identity(), shape2.get(), .{});
    try compound_settings.base.addShape(Vec3.init(0, 1.0, 0), Quat.identity(), shape3.get(), .{});
    var compound_body_settings = BodyCreationSettings.initWithSettings(shape_settings.get(), RVec3.init(0, 20, 0), Quat.identity(), .dynamic, Layers.moving);
    defer compound_body_settings.deinit();
    compound_body_settings.use_manifold_reduction = false; // Turn off manifold reduction for this body so that we can get proper callbacks for individual sub shapes
    const compound_body = try bi.createAndAddBody(&compound_body_settings, .activate);

    // Simulate until the body passes the origin
    while (bi.getPosition(compound_body).getY() > 0.0)
        try simulateSingleStep(&c, &listener);

    // The expected events
    const Expected = struct {
        type: EType,
        user_data: UserData,
    };
    const expected = [_]Expected{
        .{ .type = .add, .user_data = .bottom },
        .{ .type = .add, .user_data = .middle },
        .{ .type = .add, .user_data = .top },
        .{ .type = .remove, .user_data = .bottom },
        .{ .type = .remove, .user_data = .middle },
        .{ .type = .remove, .user_data = .top },
    };
    var next: usize = 0;
    const end = expected.len;

    var shape = bi.getShape(compound_body);
    defer shape.deinit();

    // Loop over events that we received
    for (0..listener.getEntryCount()) |e| {
        const entry = listener.getEntry(e);

        // Only interested in adds/removes
        if (entry.type != .add and entry.type != .remove)
            continue;

        // Check if we have more expected events
        if (next >= end) {
            try expect(false);
            break;
        }

        // Check if it is of expected type
        try expect(entry.type == expected[next].type);
        try expect(entry.body1.eql(sensor_id));
        try expect(entry.manifold.sub_shape_id1.eql(SubShapeID.empty));
        try expect(entry.body2.eql(compound_body));
        const user_data: UserData = @enumFromInt(shape.get().?.getSubShapeUserData(entry.manifold.sub_shape_id2));
        try expect(user_data == expected[next].user_data);

        // Next expected event
        next += 1;
    }

    // Check all expected events received
    try expect(next == end);
}

test "TestSensorVsStatic" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Register listener
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    c.getSystem().setContactListener(.init(&listener));

    // Static body 1
    const static1 = try c.createSphere(RVec3.zero(), 1.0, .static, .discrete, Layers.non_moving, .{ .activation = .dont_activate });

    // Sensor
    var sensor_settings = try sensorSettings(RVec3.zero(), .kinematic, Layers.moving); // Put in layer that collides with static
    defer sensor_settings.deinit();
    sensor_settings.is_sensor = true;
    const sensor = (try c.getBodyInterface().createBody(&sensor_settings)) orelse return error.OutOfBodies;
    const sensor_id = sensor.getID();
    try c.getBodyInterface().addBody(sensor_id, .activate);

    // Static body 2 (created after sensor to force higher body ID)
    const static2 = try c.createSphere(RVec3.zero(), 1.0, .static, .discrete, Layers.non_moving, .{ .activation = .dont_activate });

    // After a step we should not detect the static bodies
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 0);
    listener.clear();

    // Start detecting static
    sensor.setCollideKinematicVsNonDynamic(true);

    // After a single step we should detect both static bodies
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 4); // Should also contain validates
    try expect(listener.contains(.add, static1.getID(), sensor_id));
    try expect(listener.contains(.add, static2.getID(), sensor_id));
    listener.clear();

    // Stop detecting static
    sensor.setCollideKinematicVsNonDynamic(false);

    // After a single step we should stop detecting both static bodies
    try simulateSingleStep(&c, &listener);
    try expect(listener.getEntryCount() == 2);
    try expect(listener.contains(.remove, static1.getID(), sensor_id));
    try expect(listener.contains(.remove, static2.getID(), sensor_id));
    listener.clear();
}

fn collideBodyVsBodyAll(context: ?*anyopaque, alloc: Allocator, body1: *const Body, body2: *const Body, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, collide_shape_settings: *CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) Allocator.Error!void {
    _ = context;

    // Override the back face mode so we get hits with all triangles
    collide_shape_settings.back_face_mode = .collide_with_back_faces;
    try PhysicsSystem.defaultSimCollideBodyVsBody(alloc, body1, body2, center_of_mass_transform1, center_of_mass_transform2, collide_shape_settings, collector, shape_filter);
}

fn collideBodyVsBodyPerBody(context: ?*anyopaque, alloc: Allocator, body1: *const Body, body2: *const Body, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, collide_shape_settings: *CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) Allocator.Error!void {
    _ = .{ context, alloc, shape_filter };

    // Max 1 hit per body pair
    var any_hit_collector = AnyHitCollisionCollector(CollideShapeCollector).init();
    defer any_hit_collector.deinit();
    const part1: SubShapeIDCreator = .{};
    const part2: SubShapeIDCreator = .{};
    CollisionDispatch.collideShapeVsShape(body1.getShape(), body2.getShape(), Vec3.one(), Vec3.one(), center_of_mass_transform1, center_of_mass_transform2, part1, part2, collide_shape_settings, &any_hit_collector.base, &.{});
    if (any_hit_collector.hadHit())
        collector.addHit(&any_hit_collector.hit);
}

fn collideBodyVsBodyPerLeaf(context: ?*anyopaque, alloc: Allocator, body1: *const Body, body2: *const Body, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, collide_shape_settings: *CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) Allocator.Error!void {
    _ = context;

    // Max 1 hit per leaf shape pair
    const part1: SubShapeIDCreator = .{};
    const part2: SubShapeIDCreator = .{};
    try zolt.collideShapeVsShapePerLeaf(AnyHitCollisionCollector(CollideShapeCollector), alloc, body1.getShape(), body2.getShape(), Vec3.one(), Vec3.one(), center_of_mass_transform1, center_of_mass_transform2, part1, part2, collide_shape_settings, collector, shape_filter);
}

test "TestSimCollideBodyVsBody" {
    // Create pyramid with flat top
    var pyramid = MeshShapeSettings.initDefault(allocator);
    defer pyramid.deinit();
    try pyramid.triangle_vertices.appendSlice(allocator, &.{ Float3.init(1, 0, 1), Float3.init(1, 0, -1), Float3.init(-1, 0, -1), Float3.init(-1, 0, 1), Float3.init(0.1, 1, 0.1), Float3.init(0.1, 1, -0.1), Float3.init(-0.1, 1, -0.1), Float3.init(-0.1, 1, 0.1) });
    try pyramid.indexed_triangles.appendSlice(allocator, &.{ IndexedTriangle.init(0, 1, 4, .{}), IndexedTriangle.init(4, 1, 5, .{}), IndexedTriangle.init(1, 2, 5, .{}), IndexedTriangle.init(2, 6, 5, .{}), IndexedTriangle.init(2, 3, 6, .{}), IndexedTriangle.init(3, 7, 6, .{}), IndexedTriangle.init(3, 0, 7, .{}), IndexedTriangle.init(0, 4, 7, .{}), IndexedTriangle.init(4, 5, 6, .{}), IndexedTriangle.init(4, 6, 7, .{}) });
    pyramid.asShapeSettings().setEmbedded();

    // Create floor of many pyramids
    var compound = StaticCompoundShapeSettings.init(allocator);
    defer compound.deinit();
    var x: i32 = -10;
    while (x <= 10) : (x += 1) {
        var z: i32 = -10;
        while (z <= 10) : (z += 1)
            try compound.base.addShape(Vec3.init(@as(f32, @floatFromInt(x)) * 2.0, 0, @as(f32, @floatFromInt(z)) * 2.0), Quat.identity(), pyramid.asShapeSettings(), .{});
    }
    compound.asShapeSettings().setEmbedded();

    for (0..3) |test_type| {
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();

        // Register listener
        var listener: LoggingContactListener = .init(allocator);
        defer listener.deinit();
        c.getSystem().setContactListener(.init(&listener));

        // Create floor
        var floor_settings = BodyCreationSettings.initWithSettings(compound.asShapeSettings(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
        defer floor_settings.deinit();
        const floor_id = try c.getBodyInterface().createAndAddBody(&floor_settings, .dont_activate);

        // A kinematic sensor that also detects static bodies
        // Note that the size has been picked so that it is slightly smaller than 11 x 11 pyramids and incorporates all triangles in those pyramids
        var sensor_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(10.99), .{})).asShape(), RVec3.init(0, 5, 0), Quat.identity(), .kinematic, Layers.moving); // Put in a layer that collides with static
        defer sensor_settings.deinit();
        sensor_settings.is_sensor = true;
        sensor_settings.collide_kinematic_vs_non_dynamic = true;
        sensor_settings.use_manifold_reduction = false;
        const sensor_id = try c.getBodyInterface().createAndAddBody(&sensor_settings, .activate);

        // Select the body vs body collision function
        switch (test_type) {
            0 => c.getSystem().setSimCollideBodyVsBody(.{ .function = collideBodyVsBodyAll }),
            1 => c.getSystem().setSimCollideBodyVsBody(.{ .function = collideBodyVsBodyPerLeaf }),
            2 => c.getSystem().setSimCollideBodyVsBody(.{ .function = collideBodyVsBodyPerBody }),
            else => unreachable,
        }

        // Trigger collision callbacks
        try simulateSingleStep(&c, &listener);

        // Count the number of hits that were detected
        var count: usize = 0;
        for (0..listener.getEntryCount()) |i| {
            const entry = listener.getEntry(i);
            if (entry.type == .add and entry.body1.eql(floor_id) and entry.body2.eql(sensor_id))
                count += 1;
        }

        // Check that we received the number of hits that we expected
        switch (test_type) {
            // All hits
            0 => try expect(count == 11 * 11 * pyramid.indexed_triangles.items.len),
            // Only 1 per sub shape
            1 => try expect(count == 11 * 11),
            // Only 1 per body pair
            2 => try expect(count == 1),
            else => unreachable,
        }
    }
}
