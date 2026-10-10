//! Port of: UnitTests/Physics/ObjectLayerPairFilterMaskTests.cpp
//! Status: partial
//! Missing: ThreeFloorTest (needs PhysicsSystem / LoggingContactListener, Phase 5)

const std = @import("std");
const zolt = @import("zolt");

const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterfaceMask = zolt.BroadPhaseLayerInterfaceMask;
const ObjectLayer = zolt.ObjectLayer;
const ObjectLayerPairFilterMask = zolt.ObjectLayerPairFilterMask;
const ObjectVsBroadPhaseLayerFilterMask = zolt.ObjectVsBroadPhaseLayerFilterMask;

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

// Not ported: ThreeFloorTest (needs PhysicsSystem / LoggingContactListener, Phase 5)
