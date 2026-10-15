//! Zolt addition, no Jolt file: tests of BroadPhaseBruteForce (SimulationArchitecture D9)
//! Status: complete
//!
//! Jolt has no unit test for the brute force broad phase (BroadPhaseTests.cpp tests the BroadPhaseQuadTree of A1).
//! These tests drive it through the virtual BroadPhase / BroadPhaseQuery interfaces on a BodyManager: adding (prepare /
//! finalize / abort) and removing bodies, every query with the object layer filter and early out, FindCollidingPairs
//! with the layer pair filter and Body::sFindCollidingPairsCanCollide, queries from several threads while bodies are
//! added and removed, and allocation failures in create / initBroadPhase.

const std = @import("std");
const zolt = @import("zolt");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;

const Allocator = std.mem.Allocator;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Mat44 = zolt.Mat44;
const AABox = zolt.AABox;
const OrientedBox = zolt.OrientedBox;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyPair = zolt.BodyPair;
const BodyManager = zolt.BodyManager;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const SphereShape = zolt.SphereShape;
const BroadPhase = zolt.BroadPhase;
const BroadPhaseBruteForce = zolt.BroadPhaseBruteForce;
const BroadPhaseCastResult = zolt.BroadPhaseCastResult;
const BodyPairCollector = zolt.BodyPairCollector;
const RayCastBodyCollector = zolt.RayCastBodyCollector;
const CastShapeBodyCollector = zolt.CastShapeBodyCollector;
const CollideShapeBodyCollector = zolt.CollideShapeBodyCollector;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const ObjectLayer = zolt.ObjectLayer;
const ObjectLayerFilter = zolt.ObjectLayerFilter;
const RayCast = zolt.RayCast;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;

const io = std.testing.io;

/// A body manager with a brute force broad phase (in place, neither may move)
const Context = struct {
    layer_interface: LayersFile.BPLayerInterfaceImpl = .init(),
    object_vs_broad_phase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_layer_pair_filter: LayersFile.ObjectLayerPairFilterImpl = .{},
    body_manager: BodyManager = .empty,
    broad_phase: *BroadPhase = undefined,

    fn init(self: *Context, allocator: Allocator, max_bodies: u32) !void {
        self.* = .{};
        try self.body_manager.init(allocator, io, max_bodies, 0, &self.layer_interface.base);
        errdefer self.body_manager.deinit();
        const brute_force = try BroadPhaseBruteForce.create(allocator);
        self.broad_phase = brute_force.asBroadPhase();
        errdefer self.broad_phase.base.destroy();
        try self.broad_phase.initBroadPhase(&self.body_manager, &self.layer_interface.base);
    }

    fn deinit(self: *Context) void {
        self.broad_phase.base.destroy();
        self.body_manager.deinit();
    }

    /// Creates a body in the body manager (not yet in the broad phase), optionally active
    fn createBody(self: *Context, settings: *const BodyCreationSettings, activate: bool) !BodyID {
        const body = try self.body_manager.allocateBody(settings);
        if (!self.body_manager.addBody(body)) {
            self.body_manager.freeBody(body);
            return error.OutOfBodies;
        }
        const id = body.getID();
        if (activate) {
            // BodyManager::ActivateBodies only activates bodies that are in the broad phase
            body.setInBroadPhaseInternal(true);
            self.body_manager.activateBodies(&.{id});
            body.setInBroadPhaseInternal(false);
        }
        return id;
    }

    /// Adds bodies to the broad phase (BodyInterface::AddBodiesPrepare / AddBodiesFinalize)
    fn add(self: *Context, ids: []BodyID) !void {
        const state = try self.broad_phase.addBodiesPrepare(ids);
        self.broad_phase.addBodiesFinalize(ids, state);
    }

    fn query(self: *const Context) *const zolt.BroadPhaseQuery {
        return self.broad_phase.asQuery();
    }
};

/// An object layer filter that rejects one layer
const RejectLayer = struct {
    pub const overrides = .{.shouldCollide};
    base: ObjectLayerFilter = .init(RejectLayer),
    layer: ObjectLayer,
    pub fn shouldCollide(self: *const RejectLayer, layer: ObjectLayer) bool {
        return layer != self.layer;
    }
};

fn sortedIDs(allocator: Allocator, hits: []const BodyID) ![]BodyID {
    const ids = try allocator.dupe(BodyID, hits);
    std.mem.sort(BodyID, ids, {}, struct {
        fn lessThan(_: void, a: BodyID, b: BodyID) bool {
            return a.lessThan(b);
        }
    }.lessThan);
    return ids;
}

fn expectIDs(allocator: Allocator, hits: []const BodyID, expected: []const BodyID) !void {
    const ids = try sortedIDs(allocator, hits);
    defer allocator.free(ids);
    try std.testing.expectEqualSlices(BodyID, expected, ids);
}

/// The scene of the query tests: a static box at the origin, a dynamic box overlapping it, a sphere far away and a box
/// in another layer
const Scene = struct {
    floor: BodyID,
    box: BodyID,
    sphere: BodyID,
    debris: BodyID,

    fn create(c: *Context, allocator: Allocator) !Scene {
        var box_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(1), .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
        defer box_settings.deinit();
        var sphere_settings = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 0.5, .{})).asShape(), RVec3.init(10, 0, 0), Quat.identity(), .dynamic, Layers.moving);
        defer sphere_settings.deinit();

        var scene: Scene = undefined;
        scene.floor = try c.createBody(&box_settings, false); // [-1, 1]^3
        box_settings.motion_type = .dynamic;
        box_settings.object_layer = Layers.moving;
        box_settings.position = RVec3.init(1.5, 0, 0);
        scene.box = try c.createBody(&box_settings, true); // [0.5, 2.5] x [-1, 1] x [-1, 1]
        scene.sphere = try c.createBody(&sphere_settings, true); // [9.5, 10.5] x [-0.5, 0.5] x [-0.5, 0.5]
        box_settings.object_layer = Layers.lq_debris;
        box_settings.position = RVec3.init(0, 0, 1.5);
        scene.debris = try c.createBody(&box_settings, false); // [-1, 1] x [-1, 1] x [0.5, 2.5]

        var ids = [_]BodyID{ scene.debris, scene.sphere, scene.floor, scene.box };
        try c.add(&ids);
        return scene;
    }
};

test "BroadPhaseBruteForce add, abort and remove bodies" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator, 8);
    defer c.deinit();
    const scene = try Scene.create(&c, allocator);

    for ([_]BodyID{ scene.floor, scene.box, scene.sphere, scene.debris }) |id|
        try expect(c.body_manager.getBody(id).isInBroadPhase());

    // Everything is found by a big box
    var collector = AllHitCollisionCollector(CollideShapeBodyCollector).init(allocator);
    defer collector.deinit();
    c.query().collideAABox(&AABox.init(Vec3.replicate(-100), Vec3.replicate(100)), &collector.base, .{});
    try collector.checkError();
    try expectIDs(allocator, collector.hits.items, &.{ scene.floor, scene.box, scene.sphere, scene.debris });

    // AddBodiesAbort: the body does not enter the broad phase
    var box_settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(1), .{})).asShape(), RVec3.init(0, 20, 0), Quat.identity(), .static, Layers.non_moving);
    defer box_settings.deinit();
    var aborted = [_]BodyID{try c.createBody(&box_settings, false)};
    const state = try c.broad_phase.addBodiesPrepare(&aborted);
    c.broad_phase.addBodiesAbort(&aborted, state);
    try expect(!c.body_manager.getBody(aborted[0]).isInBroadPhase());
    collector.reset();
    c.query().collideAABox(&AABox.init(Vec3.replicate(-100), Vec3.replicate(100)), &collector.base, .{});
    try expectEqual(@as(usize, 4), collector.hits.items.len);

    // Remove two bodies
    var removed = [_]BodyID{ scene.sphere, scene.floor };
    c.broad_phase.removeBodies(&removed);
    try expect(!c.body_manager.getBody(scene.sphere).isInBroadPhase());
    try expect(!c.body_manager.getBody(scene.floor).isInBroadPhase());
    collector.reset();
    c.query().collideAABox(&AABox.init(Vec3.replicate(-100), Vec3.replicate(100)), &collector.base, .{});
    try expectIDs(allocator, collector.hits.items, &.{ scene.box, scene.debris });

    // And add them again together with the aborted body
    var again = [_]BodyID{ aborted[0], scene.floor, scene.sphere };
    try c.add(&again);
    collector.reset();
    c.query().collideAABox(&AABox.init(Vec3.replicate(-100), Vec3.replicate(100)), &collector.base, .{});
    try expectIDs(allocator, collector.hits.items, &.{ scene.floor, scene.box, scene.sphere, scene.debris, aborted[0] });

    // The bounds of all bodies
    const bounds = c.query().getBounds();
    try expect(bounds.min.eql(Vec3.init(-1, -1, -1)));
    try expect(bounds.max.eql(Vec3.init(10.5, 21, 2.5)));

    // Moving a body needs no notification: the bodies are referenced directly
    var moved = [_]BodyID{scene.sphere};
    c.body_manager.getBodyMut(scene.sphere).setPositionAndRotationInternal(RVec3.init(-10, 0, 0), Quat.identity(), .{});
    c.broad_phase.notifyBodiesAABBChanged(&moved, .{});
    try expect(c.query().getBounds().min.eql(Vec3.init(-10.5, -1, -1)));
}

test "BroadPhaseBruteForce queries" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator, 8);
    defer c.deinit();
    const scene = try Scene.create(&c, allocator);
    const reject_debris: RejectLayer = .{ .layer = Layers.lq_debris };

    // Ray cast: hits the top of the floor box at half the ray
    {
        var collector = AllHitCollisionCollector(RayCastBodyCollector).init(allocator);
        defer collector.deinit();
        c.query().castRay(&RayCast.init(Vec3.init(0, 2, 0.75), Vec3.init(0, -2, 0)), &collector.base, .{});
        try collector.checkError();
        collector.sort();
        try expectEqual(@as(usize, 2), collector.hits.items.len); // The floor and the debris box, both with their top at y = 1
        try expectEqual(@as(f32, 0.5), collector.hits.items[0].fraction);
        try expectEqual(@as(f32, 0.5), collector.hits.items[1].fraction);

        // The object layer filter removes the debris
        collector.reset();
        c.query().castRay(&RayCast.init(Vec3.init(0, 2, 0.75), Vec3.init(0, -2, 0)), &collector.base, .{ .object_layer_filter = &reject_debris.base });
        try expectEqual(@as(usize, 1), collector.hits.items.len);
        try expect(collector.hits.items[0].body_id.eql(scene.floor));

        // The sphere: its bounds start at y = 0.5
        collector.reset();
        c.query().castRay(&RayCast.init(Vec3.init(10, 2, 0), Vec3.init(0, -4, 0)), &collector.base, .{});
        try expectEqual(@as(usize, 1), collector.hits.items.len);
        try expect(collector.hits.items[0].body_id.eql(scene.sphere));
        try expectEqual(@as(f32, 0.375), collector.hits.items[0].fraction);

        // Too short
        collector.reset();
        c.query().castRay(&RayCast.init(Vec3.init(10, 2, 0), Vec3.init(0, -1, 0)), &collector.base, .{});
        try expect(!collector.hadHit());
    }

    // Collide with a box, a sphere, a point and an oriented box
    {
        var collector = AllHitCollisionCollector(CollideShapeBodyCollector).init(allocator);
        defer collector.deinit();
        c.query().collideAABox(&AABox.init(Vec3.init(2, -0.1, -0.1), Vec3.init(9.6, 0.1, 0.1)), &collector.base, .{});
        try expectIDs(allocator, collector.hits.items, &.{ scene.box, scene.sphere });

        collector.reset();
        c.query().collideSphere(Vec3.init(0, 0, 3), 0.6, &collector.base, .{}); // Touches the debris box at z = 2.5
        try expectIDs(allocator, collector.hits.items, &.{scene.debris});
        collector.reset();
        c.query().collideSphere(Vec3.init(0, 0, 3), 0.6, &collector.base, .{ .object_layer_filter = &reject_debris.base });
        try expect(!collector.hadHit());

        collector.reset();
        c.query().collidePoint(Vec3.init(0.75, 0, 0), &collector.base, .{}); // Inside the floor and the dynamic box
        try expectIDs(allocator, collector.hits.items, &.{ scene.floor, scene.box });

        collector.reset();
        c.query().collideOrientedBox(&OrientedBox.init(Mat44.rotationTranslation(Quat.rotation(Vec3.init(0, 1, 0), 0.25 * std.math.pi), Vec3.init(10, 0, 0)), Vec3.replicate(0.2)), &collector.base, .{});
        try expectIDs(allocator, collector.hits.items, &.{scene.sphere});
    }

    // Cast a box: the fraction at which the expanded bounds are hit
    {
        var collector = AllHitCollisionCollector(CastShapeBodyCollector).init(allocator);
        defer collector.deinit();
        c.query().castAABox(&.{ .box = .init(Vec3.init(9.5, 3.5, -0.5), Vec3.init(10.5, 4.5, 0.5)), .direction = Vec3.init(0, -8, 0) }, &collector.base, .{});
        try collector.checkError();
        try expectEqual(@as(usize, 1), collector.hits.items.len);
        try expect(collector.hits.items[0].body_id.eql(scene.sphere));
        try expectEqual(@as(f32, 0.375), collector.hits.items[0].fraction); // Center at y = 4 hits y = 1 (0.5 + 0.5)

        // castAABoxNoLock is the same query without the lock
        collector.reset();
        try c.broad_phase.castAABoxNoLock(&.{ .box = .init(Vec3.init(9.5, 3.5, -0.5), Vec3.init(10.5, 4.5, 0.5)), .direction = Vec3.init(0, -8, 0) }, &collector.base, &.{}, &.{});
        try expectEqual(@as(usize, 1), collector.hits.items.len);
        try expectEqual(@as(f32, 0.375), collector.hits.items[0].fraction);
    }

    // Early out: the any hit collector stops after the first hit
    {
        var collector = AnyHitCollisionCollector(CollideShapeBodyCollector).init();
        defer collector.deinit();
        c.query().collideAABox(&AABox.init(Vec3.replicate(-100), Vec3.replicate(100)), &collector.base, .{});
        try expect(collector.had_hit);
    }
}

test "BroadPhaseBruteForce findCollidingPairs" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator, 8);
    defer c.deinit();
    const scene = try Scene.create(&c, allocator);

    // A second active box touching the first one and the sphere
    var settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(1), .{})).asShape(), RVec3.init(3.0, 0, 0), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    var box2 = [_]BodyID{try c.createBody(&settings, true)};
    try c.add(&box2);

    const Collector = AllHitCollisionCollector(BodyPairCollector);
    var collector = Collector.init(allocator);
    defer collector.deinit();
    var active = [_]BodyID{ scene.box, scene.sphere, box2[0] };
    try c.broad_phase.findCollidingPairs(&active, 0.0, &c.object_vs_broad_phase_layer_filter.base, &c.object_layer_pair_filter.base, &collector.base);
    try collector.checkError();

    // box vs floor, box vs box2 once (the pair of two active bodies is reported by the body with the lower index); the
    // debris box overlaps the floor and the box but lq_debris only collides with non_moving and is not active
    const expected = [_]BodyPair{ .init(scene.box, scene.floor), .init(scene.box, box2[0]) };
    try expectEqual(expected.len, collector.hits.items.len);
    for (expected) |pair| {
        var found = false;
        for (collector.hits.items) |hit| found = found or hit.eql(pair);
        try expect(found);
    }

    // The speculative contact distance expands the bounds of the active bodies: the sphere ([9.5, 10.5] in x) reaches
    // box2 ([2, 4]) with 5.6, and reports the pair because it has the lower index in the active bodies
    var sphere = [_]BodyID{scene.sphere};
    collector.reset();
    try c.broad_phase.findCollidingPairs(&sphere, 0.0, &c.object_vs_broad_phase_layer_filter.base, &c.object_layer_pair_filter.base, &collector.base);
    try expect(!collector.hadHit());
    try c.broad_phase.findCollidingPairs(&sphere, 5.6, &c.object_vs_broad_phase_layer_filter.base, &c.object_layer_pair_filter.base, &collector.base);
    try expectEqual(@as(usize, 1), collector.hits.items.len);
    try expect(collector.hits.items[0].eql(.init(scene.sphere, box2[0])));

    // From box2 the pair is skipped (Body::sFindCollidingPairsCanCollide), only the box overlaps
    collector.reset();
    try c.broad_phase.findCollidingPairs(&box2, 5.6, &c.object_vs_broad_phase_layer_filter.base, &c.object_layer_pair_filter.base, &collector.base);
    for (collector.hits.items) |hit| try expect(!hit.eql(.init(box2[0], scene.sphere)) and !hit.eql(.init(box2[0], scene.box)));
}

test "BroadPhaseBruteForce queries from several threads while bodies are added and removed" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator, 64);
    defer c.deinit();

    var settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer settings.deinit();
    var fixed: [16]BodyID = undefined;
    for (&fixed, 0..) |*id, i| {
        settings.position = RVec3.init(@floatFromInt(i), 0, 0);
        id.* = try c.createBody(&settings, false);
    }
    try c.add(&fixed);
    var changing: [16]BodyID = undefined;
    for (&changing, 0..) |*id, i| {
        settings.position = RVec3.init(@floatFromInt(i), 5, 0);
        id.* = try c.createBody(&settings, false);
    }

    const Reader = struct {
        fn run(context: *const Context, stop: *const std.atomic.Value(bool), failures: *std.atomic.Value(u32)) void {
            var collector = AllHitCollisionCollector(CollideShapeBodyCollector).init(std.testing.allocator);
            defer collector.deinit();
            while (!stop.load(.acquire)) {
                collector.reset();
                context.query().collideAABox(&AABox.init(Vec3.replicate(-100), Vec3.replicate(100)), &collector.base, .{});
                // The fixed bodies are always there, the changing ones all or none (added and removed as one batch)
                const n = collector.hits.items.len;
                const ok = if (collector.checkError()) |_| n == 16 or n == 32 else |_| false;
                if (!ok) _ = failures.fetchAdd(1, .monotonic);
            }
        }
    };
    var stop: std.atomic.Value(bool) = .init(false);
    var failures: std.atomic.Value(u32) = .init(0);
    var threads: [4]std.Thread = undefined;
    for (&threads) |*t| t.* = try std.Thread.spawn(.{}, Reader.run, .{ &c, &stop, &failures });
    for (0..200) |_| {
        try c.add(&changing);
        c.broad_phase.removeBodies(&changing);
    }
    stop.store(true, .release);
    for (threads) |t| t.join();
    try expectEqual(@as(u32, 0), failures.load(.monotonic));
}

test "BroadPhaseBruteForce out of memory in create and initBroadPhase" {
    const Ctx = struct {
        fn run(allocator: Allocator) !void {
            var c: Context = undefined;
            try c.init(allocator, 1024);
            c.deinit();
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, Ctx.run, .{});
}
