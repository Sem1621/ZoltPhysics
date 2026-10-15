//! Port of: UnitTests/Physics/ObjectLayerPairFilterMaskTests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const LoggingContactListener = @import("../LoggingContactListener.zig").LoggingContactListener;

const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const BoxShape = zolt.BoxShape;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterfaceMask = zolt.BroadPhaseLayerInterfaceMask;
const ObjectLayer = zolt.ObjectLayer;
const ObjectLayerPairFilterMask = zolt.ObjectLayerPairFilterMask;
const ObjectVsBroadPhaseLayerFilterMask = zolt.ObjectVsBroadPhaseLayerFilterMask;
const JobSystemSingleThreaded = zolt.JobSystemSingleThreaded;
const MotionType = zolt.MotionType;
const PhysicsSystem = zolt.PhysicsSystem;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const Vec3 = zolt.Vec3;

const allocator = std.testing.allocator;
const io = std.testing.io;

const expect = std.testing.expect;

fn getObjectLayer(group: u32, mask: u32) ObjectLayer {
    return ObjectLayerPairFilterMask.getObjectLayer(group, .{ .mask = mask });
}

test "ObjectLayerPairFilterMaskTest" {
    // Some example layers
    const filter_default: u32 = 1;
    const filter_static: u32 = 2;
    const filter_debris: u32 = 4;
    const filter_sensor: u32 = 8;
    const filter_all: u32 = filter_default | filter_static | filter_debris | filter_sensor;

    const pair_filter: ObjectLayerPairFilterMask = .{};

    var layer1 = getObjectLayer(filter_default, filter_all);
    var layer2 = getObjectLayer(filter_static, filter_all);
    try expect(pair_filter.shouldCollide(layer1, layer2));
    try expect(pair_filter.shouldCollide(layer2, layer1));

    layer1 = getObjectLayer(filter_default, filter_static);
    layer2 = getObjectLayer(filter_static, filter_default);
    try expect(pair_filter.shouldCollide(layer1, layer2));
    try expect(pair_filter.shouldCollide(layer2, layer1));

    layer1 = getObjectLayer(filter_default, filter_default);
    layer2 = getObjectLayer(filter_static, filter_default);
    try expect(!pair_filter.shouldCollide(layer1, layer2));
    try expect(!pair_filter.shouldCollide(layer2, layer1));

    layer1 = getObjectLayer(filter_default, filter_static);
    layer2 = getObjectLayer(filter_static, filter_static);
    try expect(!pair_filter.shouldCollide(layer1, layer2));
    try expect(!pair_filter.shouldCollide(layer2, layer1));

    layer1 = getObjectLayer(filter_default | filter_debris, filter_all);
    layer2 = getObjectLayer(filter_static, filter_static);
    try expect(!pair_filter.shouldCollide(layer1, layer2));
    try expect(!pair_filter.shouldCollide(layer2, layer1));

    var bp_interface = try BroadPhaseLayerInterfaceMask.init(std.testing.allocator, 4);
    defer bp_interface.deinit();
    bp_interface.configureLayer(.init(0), filter_default, 0); // Default goes to 0
    bp_interface.configureLayer(.init(1), filter_static, filter_sensor); // Static but not sensor goes to 1
    bp_interface.configureLayer(.init(2), filter_static, 0); // Everything else static goes to 2
    // Last layer is for everything else

    try expect(bp_interface.getBroadPhaseLayer(ObjectLayerPairFilterMask.getObjectLayer(filter_default, .{})).eql(.init(0)));
    try expect(bp_interface.getBroadPhaseLayer(ObjectLayerPairFilterMask.getObjectLayer(filter_all, .{})).eql(.init(0)));
    try expect(bp_interface.getBroadPhaseLayer(ObjectLayerPairFilterMask.getObjectLayer(filter_static, .{})).eql(.init(1)));
    try expect(bp_interface.getBroadPhaseLayer(ObjectLayerPairFilterMask.getObjectLayer(filter_static | filter_sensor, .{})).eql(.init(2)));
    try expect(bp_interface.getBroadPhaseLayer(ObjectLayerPairFilterMask.getObjectLayer(filter_debris, .{})).eql(.init(3)));

    const bp_filter = ObjectVsBroadPhaseLayerFilterMask.init(&bp_interface);

    try expect(bp_filter.shouldCollide(getObjectLayer(filter_all, filter_default), .init(0)));
    try expect(!bp_filter.shouldCollide(getObjectLayer(filter_all, filter_default), .init(1)));
    try expect(!bp_filter.shouldCollide(getObjectLayer(filter_all, filter_default), .init(2)));
    try expect(bp_filter.shouldCollide(getObjectLayer(filter_all, filter_default), .init(3)));

    try expect(!bp_filter.shouldCollide(getObjectLayer(filter_all, filter_static), .init(0)));
    try expect(bp_filter.shouldCollide(getObjectLayer(filter_all, filter_static), .init(1)));
    try expect(bp_filter.shouldCollide(getObjectLayer(filter_all, filter_static), .init(2)));
    try expect(bp_filter.shouldCollide(getObjectLayer(filter_all, filter_static), .init(3)));

    try expect(!bp_filter.shouldCollide(getObjectLayer(filter_all, filter_sensor), .init(0)));
    try expect(!bp_filter.shouldCollide(getObjectLayer(filter_all, filter_sensor), .init(1)));
    try expect(!bp_filter.shouldCollide(getObjectLayer(filter_all, filter_sensor), .init(2)));
    try expect(bp_filter.shouldCollide(getObjectLayer(filter_all, filter_sensor), .init(3)));
}

/// body_interface.CreateAndAddBody(BodyCreationSettings(shape, position, Quat::sIdentity(), motion type, layer), activation)
fn createAndAddBody(body_interface: *zolt.BodyInterface, shape: *const Shape, position: RVec3, motion_type: MotionType, layer: ObjectLayer, activation: zolt.Activation) !BodyID {
    var settings = BodyCreationSettings.initWithShape(shape, position, Quat.identity(), motion_type, layer);
    defer settings.deinit();
    return body_interface.createAndAddBody(&settings, activation);
}

test "ThreeFloorTest" {
    // Define the group bits
    const group_static: u32 = 1;
    const group_floor1: u32 = 2;
    const group_floor2: u32 = 4;
    const group_floor3: u32 = 8;
    const group_all: u32 = group_static | group_floor1 | group_floor2 | group_floor3;

    const pair_filter: ObjectLayerPairFilterMask = .{};

    const num_broad_phase_layers: u32 = 2;
    const bp_layer_static: BroadPhaseLayer = .init(0);
    const bp_layer_dynamic: BroadPhaseLayer = .init(1);
    var bp_interface = try BroadPhaseLayerInterfaceMask.init(allocator, num_broad_phase_layers);
    defer bp_interface.deinit();
    bp_interface.configureLayer(bp_layer_static, group_static, 0); // Anything that has the static bit set goes into the static broadphase layer
    bp_interface.configureLayer(bp_layer_dynamic, group_floor1 | group_floor2 | group_floor3, 0); // Anything that has one of the floor bits set goes into the dynamic broadphase layer

    const bp_filter = ObjectVsBroadPhaseLayerFilterMask.init(&bp_interface);

    var system: PhysicsSystem = .empty;
    try system.init(allocator, io, 1024, 0, 1024, 1024, &bp_interface.base, &bp_filter.base, &pair_filter.base, .{});
    defer system.deinit();
    const body_interface = system.getBodyInterfaceMut();

    // Create 3 floors, each colliding with a different group
    var floor_shape = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(10, 0.1, 10), .{})).asShape());
    defer floor_shape.deinit();
    const ground = try createAndAddBody(body_interface, (try BoxShape.create(allocator, Vec3.init(20, 0.1, 20), .{})).asShape(), RVec3.zero(), .static, getObjectLayer(group_static, group_floor1), .dont_activate);
    const floor1 = try createAndAddBody(body_interface, floor_shape.get().?, RVec3.init(0, 2, 0), .static, getObjectLayer(group_static, group_floor1), .dont_activate);
    const floor2 = try createAndAddBody(body_interface, floor_shape.get().?, RVec3.init(0, 4, 0), .static, getObjectLayer(group_static, group_floor2), .dont_activate);
    const floor3 = try createAndAddBody(body_interface, floor_shape.get().?, RVec3.init(0, 6, 0), .static, getObjectLayer(group_static, group_floor3), .dont_activate);

    // Create dynamic bodies, each colliding with a different floor
    var box_shape = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape());
    defer box_shape.deinit();
    const dynamic_floor1 = try createAndAddBody(body_interface, box_shape.get().?, RVec3.init(0, 8, 0), .dynamic, getObjectLayer(group_floor1, group_all), .activate);
    const dynamic_floor2 = try createAndAddBody(body_interface, box_shape.get().?, RVec3.init(0, 9, 0), .dynamic, getObjectLayer(group_floor2, group_all), .activate);
    const dynamic_floor3 = try createAndAddBody(body_interface, box_shape.get().?, RVec3.init(0, 10, 0), .dynamic, getObjectLayer(group_floor3, group_all), .activate);
    const dynamic_ground = try createAndAddBody(body_interface, box_shape.get().?, RVec3.init(15, 8, 0), .dynamic, getObjectLayer(group_floor1, group_all), .activate);

    // Start listening to collision events
    var listener: LoggingContactListener = .init(allocator);
    defer listener.deinit();
    system.setContactListener(.init(&listener));

    // Simulate long enough for all objects to fall on the ground
    var temp_allocator = try TempAllocatorImpl.init(allocator, 4 * 1024 * 1024);
    defer temp_allocator.deinit();
    var job_system = try JobSystemSingleThreaded.init(allocator, io, @intCast(zolt.physics_settings.max_physics_jobs));
    defer job_system.deinit();
    for (0..100) |_| {
        _ = try system.update(1.0 / 60.0, 1, temp_allocator.tempAllocator(), job_system.jobSystem());
        try listener.checkError();
    }

    // Dynamic 1 should rest on floor 1
    try expect(listener.contains(.add, dynamic_floor1, floor1));
    try expect(!listener.contains(.add, dynamic_floor1, floor2));
    try expect(!listener.contains(.add, dynamic_floor1, floor3));
    try expect(!listener.contains(.add, dynamic_floor1, ground));
    const tolerance = 1.1 * system.getPhysicsSettings().penetration_slop;
    try fw.checkApproxEqual(body_interface.getPosition(dynamic_floor1), RVec3.init(0, 2.6, 0), .{ .tolerance = tolerance });

    // Dynamic 2 should rest on floor 2
    try expect(!listener.contains(.add, dynamic_floor2, floor1));
    try expect(listener.contains(.add, dynamic_floor2, floor2));
    try expect(!listener.contains(.add, dynamic_floor2, floor3));
    try expect(!listener.contains(.add, dynamic_floor2, ground));
    try fw.checkApproxEqual(body_interface.getPosition(dynamic_floor2), RVec3.init(0, 4.6, 0), .{ .tolerance = tolerance });

    // Dynamic 3 should rest on floor 3
    try expect(!listener.contains(.add, dynamic_floor3, floor1));
    try expect(!listener.contains(.add, dynamic_floor3, floor2));
    try expect(listener.contains(.add, dynamic_floor3, floor3));
    try expect(!listener.contains(.add, dynamic_floor3, ground));
    try fw.checkApproxEqual(body_interface.getPosition(dynamic_floor3), RVec3.init(0, 6.6, 0), .{ .tolerance = tolerance });

    // Dynamic 4 should rest on the ground floor
    try expect(!listener.contains(.add, dynamic_ground, floor1));
    try expect(!listener.contains(.add, dynamic_ground, floor2));
    try expect(!listener.contains(.add, dynamic_ground, floor3));
    try expect(listener.contains(.add, dynamic_ground, ground));
    try fw.checkApproxEqual(body_interface.getPosition(dynamic_ground), RVec3.init(15, 0.6, 0), .{ .tolerance = tolerance });
}
