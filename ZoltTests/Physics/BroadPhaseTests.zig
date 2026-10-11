//! Port of: UnitTests/Physics/BroadPhaseTests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const checkApproxEqual = @import("../UnitTestFramework.zig").checkApproxEqual;

const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const BodyID = zolt.BodyID;
const BodyManager = zolt.BodyManager;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const BroadPhaseQuadTree = zolt.BroadPhaseQuadTree;
const RayCastBodyCollector = zolt.RayCastBodyCollector;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const RayCast = zolt.RayCast;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;

test "TestBroadPhaseOptimize" {
    const allocator = std.testing.allocator;
    const broad_phase_layer_interface: LayersFile.BPLayerInterfaceImpl = .init();

    // Create body manager
    var body_manager: BodyManager = .empty;
    try body_manager.init(allocator, std.testing.io, 1, 0, &broad_phase_layer_interface.base);
    defer body_manager.deinit();

    // Create quad tree
    const broadphase = try BroadPhaseQuadTree.create(allocator);
    defer broadphase.base.base.destroy();
    try broadphase.asBroadPhase().initBroadPhase(&body_manager, &broad_phase_layer_interface.base);
    const query = broadphase.asBroadPhase().asQuery();

    // Create a box
    var settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.one(), .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer settings.deinit();
    const body = try body_manager.allocateBody(&settings);
    try expect(body_manager.addBody(body));

    // Add it to the broadphase
    var id = [_]BodyID{body.getID()};
    const add_state = try broadphase.asBroadPhase().addBodiesPrepare(&id);
    broadphase.asBroadPhase().addBodiesFinalize(&id, add_state);

    // Test that we hit the box at its current location and not where we're going to move it to
    var collector = AllHitCollisionCollector(RayCastBodyCollector).init(allocator);
    defer collector.deinit();
    query.castRay(&RayCast.init(Vec3.init(0, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expectEqual(@as(usize, 1), collector.hits.items.len);
    try expect(collector.hits.items[0].body_id.eql(id[0]));
    try checkApproxEqual(collector.hits.items[0].fraction, 0.5, .{});
    collector.reset();
    query.castRay(&RayCast.init(Vec3.init(2, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expect(collector.hits.items.len == 0);
    query.castRay(&RayCast.init(Vec3.init(4, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expect(collector.hits.items.len == 0);

    // Move the body
    body.setPositionAndRotationInternal(RVec3.init(2, 0, 0), Quat.identity(), .{});
    broadphase.asBroadPhase().notifyBodiesAABBChanged(&id, .{ .take_lock = true });

    // Test that we hit the box at its previous and current location
    query.castRay(&RayCast.init(Vec3.init(0, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expectEqual(@as(usize, 1), collector.hits.items.len);
    try expect(collector.hits.items[0].body_id.eql(id[0]));
    try checkApproxEqual(collector.hits.items[0].fraction, 0.5, .{});
    collector.reset();
    query.castRay(&RayCast.init(Vec3.init(2, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expectEqual(@as(usize, 1), collector.hits.items.len);
    try expect(collector.hits.items[0].body_id.eql(id[0]));
    try checkApproxEqual(collector.hits.items[0].fraction, 0.5, .{});
    collector.reset();
    query.castRay(&RayCast.init(Vec3.init(4, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expect(collector.hits.items.len == 0);

    // Optimize the broadphase
    try broadphase.asBroadPhase().optimize();

    // Test that we hit the box only at the new location
    query.castRay(&RayCast.init(Vec3.init(0, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expect(collector.hits.items.len == 0);
    query.castRay(&RayCast.init(Vec3.init(2, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expectEqual(@as(usize, 1), collector.hits.items.len);
    try expect(collector.hits.items[0].body_id.eql(id[0]));
    try checkApproxEqual(collector.hits.items[0].fraction, 0.5, .{});
    collector.reset();
    query.castRay(&RayCast.init(Vec3.init(4, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expect(collector.hits.items.len == 0);

    // Move the body again (so that for the next optimize we'll have to discard a tree)
    body.setPositionAndRotationInternal(RVec3.init(4, 0, 0), Quat.identity(), .{});
    broadphase.asBroadPhase().notifyBodiesAABBChanged(&id, .{ .take_lock = true });

    // Test that we hit the box at its previous and current location
    query.castRay(&RayCast.init(Vec3.init(0, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expect(collector.hits.items.len == 0);
    query.castRay(&RayCast.init(Vec3.init(2, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expectEqual(@as(usize, 1), collector.hits.items.len);
    try expect(collector.hits.items[0].body_id.eql(id[0]));
    try checkApproxEqual(collector.hits.items[0].fraction, 0.5, .{});
    collector.reset();
    query.castRay(&RayCast.init(Vec3.init(4, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expectEqual(@as(usize, 1), collector.hits.items.len);
    try expect(collector.hits.items[0].body_id.eql(id[0]));
    try checkApproxEqual(collector.hits.items[0].fraction, 0.5, .{});
    collector.reset();

    // Optimize the broadphase (this will internally have to discard a tree)
    try broadphase.asBroadPhase().optimize();

    // Test that we hit the box only at the new location
    query.castRay(&RayCast.init(Vec3.init(0, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expect(collector.hits.items.len == 0);
    query.castRay(&RayCast.init(Vec3.init(2, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expect(collector.hits.items.len == 0);
    query.castRay(&RayCast.init(Vec3.init(4, 2, 0), Vec3.init(0, -2, 0)), &collector.base, .{});
    try expectEqual(@as(usize, 1), collector.hits.items.len);
    try expect(collector.hits.items[0].body_id.eql(id[0]));
    try checkApproxEqual(collector.hits.items[0].fraction, 0.5, .{});
    collector.reset();
}
