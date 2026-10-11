//! Port of: UnitTests/Physics/ObjectLayerPairFilterTableTests.cpp
//! Not ported: JPH_EXTERNAL_PROFILE / JPH_PROFILE_ENABLED (the broadphase layer name checks, Zolt has no profiler)

const std = @import("std");
const zolt = @import("zolt");
const layers = @import("../Layers.zig");

const ObjectLayer = zolt.ObjectLayer;
const ObjectLayerPairFilterTable = zolt.ObjectLayerPairFilterTable;
const BroadPhaseLayerInterfaceTable = zolt.BroadPhaseLayerInterfaceTable;
const ObjectVsBroadPhaseLayerFilterTable = zolt.ObjectVsBroadPhaseLayerFilterTable;
const Layers = layers.Layers;
const BroadPhaseLayers = layers.BroadPhaseLayers;

const expect = std.testing.expect;

test "ObjectLayerPairFilterTableTest" {
    const allocator = std.testing.allocator;

    // Init object layers
    var obj_vs_obj_filter = try ObjectLayerPairFilterTable.init(allocator, Layers.num_layers);
    defer obj_vs_obj_filter.deinit();
    obj_vs_obj_filter.enableCollision(Layers.moving, Layers.non_moving);
    obj_vs_obj_filter.enableCollision(Layers.moving, Layers.moving);
    obj_vs_obj_filter.enableCollision(Layers.moving, Layers.sensor);
    obj_vs_obj_filter.enableCollision(Layers.lq_debris, Layers.non_moving);
    obj_vs_obj_filter.enableCollision(Layers.hq_debris, Layers.non_moving);
    obj_vs_obj_filter.enableCollision(Layers.hq_debris, Layers.moving);

    // Check collision pairs
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.non_moving, Layers.non_moving));
    try expect(obj_vs_obj_filter.shouldCollide(Layers.non_moving, Layers.moving));
    try expect(obj_vs_obj_filter.shouldCollide(Layers.non_moving, Layers.hq_debris));
    try expect(obj_vs_obj_filter.shouldCollide(Layers.non_moving, Layers.lq_debris));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.non_moving, Layers.sensor));

    try expect(obj_vs_obj_filter.shouldCollide(Layers.moving, Layers.non_moving));
    try expect(obj_vs_obj_filter.shouldCollide(Layers.moving, Layers.moving));
    try expect(obj_vs_obj_filter.shouldCollide(Layers.moving, Layers.hq_debris));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.moving, Layers.lq_debris));
    try expect(obj_vs_obj_filter.shouldCollide(Layers.moving, Layers.sensor));

    try expect(obj_vs_obj_filter.shouldCollide(Layers.hq_debris, Layers.non_moving));
    try expect(obj_vs_obj_filter.shouldCollide(Layers.hq_debris, Layers.moving));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.hq_debris, Layers.hq_debris));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.hq_debris, Layers.lq_debris));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.hq_debris, Layers.sensor));

    try expect(obj_vs_obj_filter.shouldCollide(Layers.lq_debris, Layers.non_moving));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.lq_debris, Layers.moving));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.lq_debris, Layers.hq_debris));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.lq_debris, Layers.lq_debris));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.lq_debris, Layers.sensor));

    try expect(!obj_vs_obj_filter.shouldCollide(Layers.sensor, Layers.non_moving));
    try expect(obj_vs_obj_filter.shouldCollide(Layers.sensor, Layers.moving));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.sensor, Layers.hq_debris));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.sensor, Layers.lq_debris));
    try expect(!obj_vs_obj_filter.shouldCollide(Layers.sensor, Layers.sensor));

    // Init broad phase layers
    var bp_layer_interface = try BroadPhaseLayerInterfaceTable.init(allocator, Layers.num_layers, BroadPhaseLayers.num_layers);
    defer bp_layer_interface.deinit();
    bp_layer_interface.mapObjectToBroadPhaseLayer(Layers.non_moving, BroadPhaseLayers.non_moving);
    bp_layer_interface.mapObjectToBroadPhaseLayer(Layers.moving, BroadPhaseLayers.moving);
    bp_layer_interface.mapObjectToBroadPhaseLayer(Layers.hq_debris, BroadPhaseLayers.moving);
    bp_layer_interface.mapObjectToBroadPhaseLayer(Layers.lq_debris, BroadPhaseLayers.lq_debris);
    bp_layer_interface.mapObjectToBroadPhaseLayer(Layers.sensor, BroadPhaseLayers.sensor);

    // Not ported: the layer name checks (JPH_EXTERNAL_PROFILE || JPH_PROFILE_ENABLED)

    // Init object vs broad phase layer filter
    var obj_vs_bp_filter = try ObjectVsBroadPhaseLayerFilterTable.init(allocator, &bp_layer_interface.base, BroadPhaseLayers.num_layers, &obj_vs_obj_filter.base, Layers.num_layers);
    defer obj_vs_bp_filter.deinit();

    // Check collision pairs
    try expect(!obj_vs_bp_filter.shouldCollide(Layers.non_moving, BroadPhaseLayers.non_moving));
    try expect(obj_vs_bp_filter.shouldCollide(Layers.non_moving, BroadPhaseLayers.moving));
    try expect(obj_vs_bp_filter.shouldCollide(Layers.non_moving, BroadPhaseLayers.lq_debris));
    try expect(!obj_vs_bp_filter.shouldCollide(Layers.non_moving, BroadPhaseLayers.sensor));

    try expect(obj_vs_bp_filter.shouldCollide(Layers.moving, BroadPhaseLayers.non_moving));
    try expect(obj_vs_bp_filter.shouldCollide(Layers.moving, BroadPhaseLayers.moving));
    try expect(!obj_vs_bp_filter.shouldCollide(Layers.moving, BroadPhaseLayers.lq_debris));
    try expect(obj_vs_bp_filter.shouldCollide(Layers.moving, BroadPhaseLayers.sensor));

    try expect(obj_vs_bp_filter.shouldCollide(Layers.hq_debris, BroadPhaseLayers.non_moving));
    try expect(obj_vs_bp_filter.shouldCollide(Layers.hq_debris, BroadPhaseLayers.moving));
    try expect(!obj_vs_bp_filter.shouldCollide(Layers.hq_debris, BroadPhaseLayers.lq_debris));
    try expect(!obj_vs_bp_filter.shouldCollide(Layers.hq_debris, BroadPhaseLayers.sensor));

    try expect(obj_vs_bp_filter.shouldCollide(Layers.lq_debris, BroadPhaseLayers.non_moving));
    try expect(!obj_vs_bp_filter.shouldCollide(Layers.lq_debris, BroadPhaseLayers.moving));
    try expect(!obj_vs_bp_filter.shouldCollide(Layers.lq_debris, BroadPhaseLayers.lq_debris));
    try expect(!obj_vs_bp_filter.shouldCollide(Layers.lq_debris, BroadPhaseLayers.sensor));

    try expect(!obj_vs_bp_filter.shouldCollide(Layers.sensor, BroadPhaseLayers.non_moving));
    try expect(obj_vs_bp_filter.shouldCollide(Layers.sensor, BroadPhaseLayers.moving));
    try expect(!obj_vs_bp_filter.shouldCollide(Layers.sensor, BroadPhaseLayers.lq_debris));
    try expect(!obj_vs_bp_filter.shouldCollide(Layers.sensor, BroadPhaseLayers.sensor));
}

test "ObjectLayerPairFilterTableTest2" {
    const n = 10;

    const Pair = struct { first: ObjectLayer, second: ObjectLayer };
    const pairs = [_]Pair{
        .{ .first = 0, .second = 0 },
        .{ .first = 9, .second = 9 },
        .{ .first = 1, .second = 3 },
        .{ .first = 3, .second = 1 },
        .{ .first = 5, .second = 7 },
        .{ .first = 7, .second = 5 },
    };

    for (pairs) |p| {
        var obj_vs_obj_filter = try ObjectLayerPairFilterTable.init(std.testing.allocator, n);
        defer obj_vs_obj_filter.deinit();
        obj_vs_obj_filter.enableCollision(p.first, p.second);

        var i: ObjectLayer = 0;
        while (i < n) : (i += 1) {
            var j: ObjectLayer = 0;
            while (j < n) : (j += 1) {
                const should_collide = (i == p.first and j == p.second) or (i == p.second and j == p.first);
                try std.testing.expectEqual(should_collide, obj_vs_obj_filter.shouldCollide(i, j));
            }
        }
    }
}
