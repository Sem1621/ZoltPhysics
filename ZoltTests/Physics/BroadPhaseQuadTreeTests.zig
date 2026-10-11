//! Zolt addition, no Jolt file: tests of BroadPhaseQuadTree and QuadTree (SimulationArchitecture D9)
//! Status: complete
//!
//! Jolt's only test of the QuadTree is TestBroadPhaseOptimize (BroadPhaseTests.zig). These tests drive the broad phase
//! through the virtual BroadPhase / BroadPhaseQuery interfaces on a BodyManager:
//! - a random sequence of batch adds (finalize and abort), single adds, removes, moves (NotifyBodiesAABBChanged),
//!   layer changes, updates (the PhysicsSystem protocol: FrameSync, LockModifications, UpdatePrepare, UpdateFinalize,
//!   UnlockModifications) and Optimize, checking the tree invariants after every step (Jolt's ValidateTree plus the
//!   tracking) and every query against a brute force computation: the hits are a superset while bounds are widened
//!   and exactly the brute force set after Optimize; FindCollidingPairs always gives exactly the brute force pairs;
//! - every query with filters and early outs, CastAABoxNoLock and GetBounds;
//! - queries from several threads while one thread moves bodies and rebuilds the trees and another adds and removes
//!   bodies (run repeatedly);
//! - the out of memory paths: checkAllAllocationFailures on init and on a whole sequence, and a failing heap in
//!   addBodiesPrepare, updatePrepare, optimize and notifyBodiesLayerChanged, which must leave the broad phase
//!   unchanged (a dump of every tree and of the tracking is compared);
//! - a ray along bodies that were added one by one: the walk stack grows on the heap (Jolt's "Stack full!").
//! The PhysicsSystem tests with the QuadTree (the default broad phase) are in PhysicsSystemTests.zig, the walk stacks of
//! a hand built deep tree (FindCollidingPairs out of memory) in QuadTree.zig.

const std = @import("std");
const zolt = @import("zolt");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const BroadPhaseLayers = LayersFile.BroadPhaseLayers;

const Allocator = std.mem.Allocator;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const Mat44 = zolt.Mat44;
const AABox = zolt.AABox;
const OrientedBox = zolt.OrientedBox;
const RayInvDirection = zolt.RayInvDirection;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyPair = zolt.BodyPair;
const BodyManager = zolt.BodyManager;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const SphereShape = zolt.SphereShape;
const Shape = zolt.Shape;
const RefConst = zolt.RefConst;
const BroadPhase = zolt.BroadPhase;
const BroadPhaseQuery = zolt.BroadPhaseQuery;
const BroadPhaseQuadTree = zolt.BroadPhaseQuadTree;
const QuadTree = zolt.QuadTree;
const BroadPhaseCastResult = zolt.BroadPhaseCastResult;
const BodyPairCollector = zolt.BodyPairCollector;
const RayCastBodyCollector = zolt.RayCastBodyCollector;
const CastShapeBodyCollector = zolt.CastShapeBodyCollector;
const CollideShapeBodyCollector = zolt.CollideShapeBodyCollector;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const ObjectLayer = zolt.ObjectLayer;
const ObjectLayerFilter = zolt.ObjectLayerFilter;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerFilter = zolt.BroadPhaseLayerFilter;
const RayCast = zolt.RayCast;
const AABoxCast = zolt.AABoxCast;
const rayAABox = zolt.rayAABox;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;
const checkApproxEqual = @import("../UnitTestFramework.zig").checkApproxEqual;

const io = std.testing.io;

const invalid_node_index: u32 = 0xffffffff;

/// A body manager with a quad tree broad phase (in place, neither may move)
const Context = struct {
    layer_interface: LayersFile.BPLayerInterfaceImpl = .init(),
    object_vs_broad_phase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_layer_pair_filter: LayersFile.ObjectLayerPairFilterImpl = .{},
    body_manager: BodyManager = .empty,
    quad_tree: *BroadPhaseQuadTree = undefined,

    /// `heap` is the allocator of the broad phase
    fn init(self: *Context, allocator: Allocator, heap: Allocator, max_bodies: u32) !void {
        self.* = .{};
        try self.body_manager.init(allocator, io, max_bodies, 0, &self.layer_interface.base);
        errdefer self.body_manager.deinit();
        self.quad_tree = try BroadPhaseQuadTree.create(heap);
        errdefer self.quad_tree.base.base.destroy();
        try self.quad_tree.asBroadPhase().initBroadPhase(&self.body_manager, &self.layer_interface.base);
    }

    fn deinit(self: *Context) void {
        self.quad_tree.base.base.destroy();
        self.body_manager.deinit();
    }

    fn bp(self: *Context) *BroadPhase {
        return self.quad_tree.asBroadPhase();
    }

    fn query(self: *const Context) *const BroadPhaseQuery {
        return self.quad_tree.base.asQuery();
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
        const state = try self.bp().addBodiesPrepare(ids);
        self.bp().addBodiesFinalize(ids, state);
    }

    /// The broad phase part of PhysicsSystem::Update
    fn update(self: *Context) !void {
        self.bp().frameSync();
        self.bp().lockModifications();
        defer self.bp().unlockModifications();
        const state = try self.bp().updatePrepare();
        self.bp().updateFinalize(&state);
    }

    fn bodyMut(self: *Context, id: BodyID) *Body {
        return self.body_manager.getBodyMut(id);
    }
};

/// Every tree (its state and the nodes of the current and the old tree in walk order) and the tracking, as u32s
fn dumpTrees(allocator: Allocator, qt: *const BroadPhaseQuadTree) ![]u32 {
    var out: std.ArrayList(u32) = .empty;
    errdefer out.deinit(allocator);
    var stack: std.ArrayList(u32) = .empty;
    defer stack.deinit(allocator);
    try out.appendSlice(allocator, &.{ qt.next_layer_to_update, qt.query_lock_idx.load(.monotonic) });
    for (qt.layers) |*tree| {
        const ts = &tree.tree_state;
        try out.appendSlice(allocator, &.{ ts.num_bodies.load(.monotonic), @intFromBool(ts.is_dirty.load(.monotonic)), ts.root_node_index.load(.monotonic), ts.root_node[0].index.load(.monotonic), ts.root_node[1].index.load(.monotonic), tree.free_node_batch.num_objects, tree.free_node_batch.first_object_index, tree.free_node_batch.last_object_index });
        for (&ts.root_node) |*root| {
            const root_idx = root.index.load(.monotonic);
            if (root_idx == invalid_node_index) continue;
            try stack.append(allocator, root_idx);
            while (stack.pop()) |node_idx| {
                const node = tree.node_allocator.?.get(node_idx);
                try out.appendSlice(allocator, &.{ node_idx, node.parent_node_index.load(.monotonic), node.is_changed.load(.monotonic) });
                for (0..4) |i| {
                    const child = node.child_node_id[i].get();
                    try out.append(allocator, child.id);
                    for ([_]*const [4]std.atomic.Value(f32){ &node.bounds_min_x, &node.bounds_min_y, &node.bounds_min_z, &node.bounds_max_x, &node.bounds_max_y, &node.bounds_max_z }) |b|
                        try out.append(allocator, @bitCast(b[i].load(.monotonic)));
                    if (child.isValid() and child.isNode())
                        try stack.append(allocator, child.getNodeIndex());
                }
            }
        }
    }
    for (qt.tracking) |*t|
        try out.appendSlice(allocator, &.{ t.broad_phase_layer.load(.monotonic), t.object_layer.load(.monotonic), t.body_location.load(.monotonic) });
    return out.toOwnedSlice(allocator);
}

fn lessThanID(_: void, a: BodyID, b: BodyID) bool {
    return a.lessThan(b);
}

/// Checks the invariants of every tree (Jolt's ValidateTree) and of the tracking, returns the number of bodies in the
/// broad phase
fn validate(allocator: Allocator, c: *Context) !u32 {
    const qt = c.quad_tree;
    var stack: std.ArrayList(u32) = .empty;
    defer stack.deinit(allocator);
    var seen: std.AutoHashMapUnmanaged(u32, void) = .empty;
    defer seen.deinit(allocator);

    for (qt.layers, 0..) |*tree, layer| {
        const ts = &tree.tree_state;
        const root_idx = ts.root_node[ts.root_node_index.load(.monotonic)].index.load(.monotonic);
        try expect(root_idx != invalid_node_index);
        try expect(tree.node_allocator.?.get(root_idx).parent_node_index.load(.monotonic) == invalid_node_index);
        var num_bodies: u32 = 0;
        try stack.append(allocator, root_idx);
        while (stack.pop()) |node_idx| {
            const node = tree.node_allocator.?.get(node_idx);
            for (0..4) |i| {
                const child = node.child_node_id[i].get();
                if (!child.isValid()) {
                    try expect(!node.getChildBounds(i).isValid()); // Removed or never set: no query can enter
                    continue;
                }
                if (child.isNode()) {
                    const child_node = tree.node_allocator.?.get(child.getNodeIndex());
                    try expect(child_node.parent_node_index.load(.monotonic) == node_idx);
                    try expect(child_node.is_changed.load(.monotonic) == 0 or node.is_changed.load(.monotonic) != 0);
                    const real = child_node.getNodeBounds();
                    try expect(!real.isValid() or node.getChildBounds(i).contains(real));
                    try stack.append(allocator, child.getNodeIndex());
                } else {
                    num_bodies += 1;
                    const id = child.getBodyID();
                    try expect(!(try seen.getOrPut(allocator, id.getIndexAndSequenceNumber())).found_existing);
                    const b = c.body_manager.getBody(id);
                    try expect(b.getID().eql(id));
                    try expect(b.isInBroadPhase());
                    const t = &qt.tracking[id.getIndex()];
                    try expectEqual(@as(u32, @intCast(node_idx)) | (@as(u32, @intCast(i)) << 30), t.body_location.load(.monotonic));
                    try expectEqual(@as(u8, @intCast(layer)), t.broad_phase_layer.load(.monotonic));
                    try expectEqual(b.getBroadPhaseLayer().getValue(), t.broad_phase_layer.load(.monotonic));
                    try expectEqual(b.getObjectLayer(), t.object_layer.load(.monotonic));
                    try expect(node.getChildBounds(i).contains(b.getWorldSpaceBounds().*));
                }
            }
        }
        try expectEqual(ts.num_bodies.load(.monotonic), num_bodies);
    }

    // Every body in the broad phase is in a tree, the others have no tracking
    for (c.body_manager.getBodies()) |slot| {
        if (!BodyManager.isValidBodyPointer(slot)) continue;
        const b = slot.getBody();
        const t = &qt.tracking[b.getID().getIndex()];
        if (b.isInBroadPhase()) {
            try expect(seen.contains(b.getID().getIndexAndSequenceNumber()));
        } else {
            try expectEqual(zolt.broad_phase_layer_invalid.value, t.broad_phase_layer.load(.monotonic));
            try expectEqual(zolt.object_layer_invalid, t.object_layer.load(.monotonic));
            try expectEqual(@as(u32, 0xffffffff), t.body_location.load(.monotonic));
        }
    }
    return seen.count();
}

/// A broad phase layer filter that accepts the layers of a mask
const MaskBroadPhaseLayerFilter = struct {
    pub const overrides = .{.shouldCollide};
    base: BroadPhaseLayerFilter = .init(MaskBroadPhaseLayerFilter),
    mask: u32,
    pub fn shouldCollide(self: *const MaskBroadPhaseLayerFilter, layer: BroadPhaseLayer) bool {
        return (self.mask >> @intCast(layer.getValue())) & 1 != 0;
    }
};

/// An object layer filter that accepts the layers of a mask
const MaskObjectLayerFilter = struct {
    pub const overrides = .{.shouldCollide};
    base: ObjectLayerFilter = .init(MaskObjectLayerFilter),
    mask: u32,
    pub fn shouldCollide(self: *const MaskObjectLayerFilter, layer: ObjectLayer) bool {
        return (self.mask >> @intCast(layer)) & 1 != 0;
    }
};

const object_layers = [_]ObjectLayer{ Layers.non_moving, Layers.moving, Layers.moving2, Layers.hq_debris, Layers.lq_debris, Layers.sensor };

fn randomVec3(random: std.Random, min: f32, max: f32) Vec3 {
    return Vec3.init(min + (max - min) * random.float(f32), min + (max - min) * random.float(f32), min + (max - min) * random.float(f32));
}

/// The bodies of the random tests: shapes and settings
const Shapes = struct {
    box: RefConst(Shape),
    sphere: RefConst(Shape),
    small: RefConst(Shape),

    fn init(allocator: Allocator) !Shapes {
        var box = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(2, 1, 3), .{})).asShape());
        errdefer box.deinit();
        var sphere = RefConst(Shape).init((try SphereShape.create(allocator, 1.5, .{})).asShape());
        errdefer sphere.deinit();
        const small = RefConst(Shape).init((try SphereShape.create(allocator, 0.25, .{})).asShape());
        return .{ .box = box, .sphere = sphere, .small = small };
    }

    fn deinit(self: *Shapes) void {
        self.box.deinit();
        self.sphere.deinit();
        self.small.deinit();
    }

    /// Creates a body with a random shape, position, rotation and layer (static in the non moving layer, otherwise
    /// dynamic and active)
    fn createRandomBody(self: *const Shapes, c: *Context, random: std.Random, extent: f32) !BodyID {
        const layer = object_layers[random.uintLessThan(usize, object_layers.len)];
        const shape = switch (random.uintLessThan(u32, 3)) {
            0 => self.box.get().?,
            1 => self.sphere.get().?,
            else => self.small.get().?,
        };
        const position = randomVec3(random, -extent, extent);
        const rotation = Quat.rotation(randomVec3(random, -1, 1).normalizedOr(Vec3.axisX()), 6.0 * random.float(f32));
        const motion_type: zolt.MotionType = if (layer == Layers.non_moving) .static else .dynamic;
        var settings = BodyCreationSettings.initWithShape(shape, RVec3.init(position.getX(), position.getY(), position.getZ()), rotation, motion_type, layer);
        defer settings.deinit();
        return c.createBody(&settings, motion_type == .dynamic);
    }
};

fn sortIDs(ids: []BodyID) void {
    std.mem.sort(BodyID, ids, {}, lessThanID);
}

/// Every query against a brute force computation: the hits must contain the brute force hits (all of them when
/// `exact`, the bounds in the tree are then tight), no body twice, and only bodies in the broad phase that pass the
/// filters
fn checkQueries(allocator: Allocator, c: *Context, random: std.Random, exact: bool) !void {
    const bp_filter: MaskBroadPhaseLayerFilter = .{ .mask = if (random.boolean()) 0xffffffff else random.int(u32) };
    const ol_filter: MaskObjectLayerFilter = .{ .mask = if (random.boolean()) 0xffffffff else random.int(u32) };
    const filters: zolt.BroadPhaseQueryFilters = .{ .broad_phase_layer_filter = &bp_filter.base, .object_layer_filter = &ol_filter.base };

    // The bodies a query can find
    var candidates: std.ArrayList(*const Body) = .empty;
    defer candidates.deinit(allocator);
    for (c.body_manager.getBodies()) |slot| {
        if (!BodyManager.isValidBodyPointer(slot)) continue;
        const b = slot.getBody();
        if (b.isInBroadPhase() and bp_filter.shouldCollide(b.getBroadPhaseLayer()) and ol_filter.shouldCollide(b.getObjectLayer()))
            try candidates.append(allocator, b);
    }

    var expected: std.ArrayList(BodyID) = .empty;
    defer expected.deinit(allocator);
    var found: std.ArrayList(BodyID) = .empty;
    defer found.deinit(allocator);

    for (0..7) |kind| {
        expected.clearRetainingCapacity();
        found.clearRetainingCapacity();
        const center = randomVec3(random, -60, 60);
        const half = randomVec3(random, 0, 30);
        switch (kind) {
            0 => {
                const box = AABox.init(center.sub(half), center.add(half));
                for (candidates.items) |b| if (b.getWorldSpaceBounds().overlaps(box)) try expected.append(allocator, b.getID());
                var collector = AllHitCollisionCollector(CollideShapeBodyCollector).init(allocator);
                defer collector.deinit();
                c.query().collideAABox(&box, &collector.base, filters);
                try collector.checkError();
                try found.appendSlice(allocator, collector.hits.items);
            },
            1 => {
                const radius = half.getX();
                for (candidates.items) |b| if (b.getWorldSpaceBounds().getSqDistanceTo(center) <= radius * radius) try expected.append(allocator, b.getID());
                var collector = AllHitCollisionCollector(CollideShapeBodyCollector).init(allocator);
                defer collector.deinit();
                c.query().collideSphere(center, radius, &collector.base, filters);
                try collector.checkError();
                try found.appendSlice(allocator, collector.hits.items);
            },
            2 => {
                for (candidates.items) |b| if (b.getWorldSpaceBounds().containsVec3(center)) try expected.append(allocator, b.getID());
                var collector = AllHitCollisionCollector(CollideShapeBodyCollector).init(allocator);
                defer collector.deinit();
                c.query().collidePoint(center, &collector.base, filters);
                try collector.checkError();
                try found.appendSlice(allocator, collector.hits.items);
            },
            3 => {
                const box = OrientedBox.init(Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 3.0 * random.float(f32)), center), half);
                for (candidates.items) |b| if (box.overlapsAABox(b.getWorldSpaceBounds().*, .{})) try expected.append(allocator, b.getID());
                var collector = AllHitCollisionCollector(CollideShapeBodyCollector).init(allocator);
                defer collector.deinit();
                c.query().collideOrientedBox(&box, &collector.base, filters);
                try collector.checkError();
                try found.appendSlice(allocator, collector.hits.items);
            },
            4 => {
                const ray = RayCast.init(center, randomVec3(random, -80, 80));
                const inv = RayInvDirection.init(ray.direction);
                for (candidates.items) |b| {
                    const bounds = b.getWorldSpaceBounds();
                    if (rayAABox(ray.origin, inv, bounds.min, bounds.max) < 1.0 + zolt.math.flt_epsilon) try expected.append(allocator, b.getID());
                }
                var collector = AllHitCollisionCollector(RayCastBodyCollector).init(allocator);
                defer collector.deinit();
                c.query().castRay(&ray, &collector.base, filters);
                try collector.checkError();
                for (collector.hits.items) |hit| try found.append(allocator, hit.body_id);
            },
            5, 6 => {
                const cast: AABoxCast = .{ .box = .init(center.sub(half.mulScalar(0.2)), center.add(half.mulScalar(0.2))), .direction = randomVec3(random, -80, 80) };
                const inv = RayInvDirection.init(cast.direction);
                const extent = cast.box.getExtent();
                for (candidates.items) |b| {
                    const bounds = b.getWorldSpaceBounds();
                    if (rayAABox(cast.box.getCenter(), inv, bounds.min.sub(extent), bounds.max.add(extent)) < 1.0 + zolt.math.flt_epsilon) try expected.append(allocator, b.getID());
                }
                var collector = AllHitCollisionCollector(CastShapeBodyCollector).init(allocator);
                defer collector.deinit();
                if (kind == 5)
                    c.query().castAABox(&cast, &collector.base, filters)
                else
                    c.bp().castAABoxNoLock(&cast, &collector.base, &bp_filter.base, &ol_filter.base);
                try collector.checkError();
                for (collector.hits.items) |hit| try found.append(allocator, hit.body_id);
            },
            else => unreachable,
        }

        sortIDs(expected.items);
        sortIDs(found.items);
        for (found.items[0..found.items.len -| 1], found.items[@min(1, found.items.len)..]) |a, b| try expect(!a.eql(b)); // No body twice
        for (found.items) |id| {
            var ok = false;
            for (candidates.items) |b| ok = ok or b.getID().eql(id);
            try expect(ok);
        }
        if (exact) {
            try std.testing.expectEqualSlices(BodyID, expected.items, found.items);
        } else {
            var i: usize = 0;
            for (found.items) |id| {
                if (i < expected.items.len and expected.items[i].eql(id)) i += 1;
            }
            try expectEqual(expected.items.len, i); // Superset
        }
    }

    // The bounds of everything in the broad phase
    const bounds = c.query().getBounds();
    for (c.body_manager.getBodies()) |slot| {
        if (!BodyManager.isValidBodyPointer(slot)) continue;
        if (slot.getBody().isInBroadPhase()) try expect(bounds.contains(slot.getBody().getWorldSpaceBounds().*));
    }
}

/// FindCollidingPairs against a brute force computation (always exact: the final test uses the bounds of the bodies)
fn checkPairs(allocator: Allocator, c: *Context, random: std.Random) !void {
    var active: std.ArrayList(BodyID) = .empty;
    defer active.deinit(allocator);
    for (c.body_manager.getBodies()) |slot| {
        if (!BodyManager.isValidBodyPointer(slot)) continue;
        const b = slot.getBody();
        if (b.isInBroadPhase() and b.isActive() and random.boolean())
            try active.append(allocator, b.getID());
    }
    if (active.items.len == 0) return;
    const distance = if (random.boolean()) 0.0 else 2.0 * random.float(f32);

    var expected: std.ArrayList(u64) = .empty;
    defer expected.deinit(allocator);
    for (active.items) |id1| {
        const b1 = c.body_manager.getBody(id1);
        var bounds1 = b1.getWorldSpaceBounds().*;
        bounds1.expandBy(Vec3.replicate(distance));
        for (c.body_manager.getBodies()) |slot| {
            if (!BodyManager.isValidBodyPointer(slot)) continue;
            const b2 = slot.getBody();
            if (!b2.isInBroadPhase() or b1 == b2) continue;
            if (c.object_vs_broad_phase_layer_filter.base.shouldCollide(b1.getObjectLayer(), b2.getBroadPhaseLayer()) and
                c.object_layer_pair_filter.base.shouldCollide(b1.getObjectLayer(), b2.getObjectLayer()) and
                Body.findCollidingPairsCanCollide(b1, b2) and bounds1.overlaps(b2.getWorldSpaceBounds().*))
                try expected.append(allocator, @as(u64, id1.getIndexAndSequenceNumber()) << 32 | b2.getID().getIndexAndSequenceNumber());
        }
    }

    var collector = AllHitCollisionCollector(BodyPairCollector).init(allocator);
    defer collector.deinit();
    try c.bp().findCollidingPairs(active.items, distance, &c.object_vs_broad_phase_layer_filter.base, &c.object_layer_pair_filter.base, &collector.base);
    try collector.checkError();
    var found: std.ArrayList(u64) = .empty;
    defer found.deinit(allocator);
    for (collector.hits.items) |pair| try found.append(allocator, @as(u64, pair.body_a.getIndexAndSequenceNumber()) << 32 | pair.body_b.getIndexAndSequenceNumber());
    std.mem.sort(u64, expected.items, {}, std.sort.asc(u64));
    std.mem.sort(u64, found.items, {}, std.sort.asc(u64));
    try std.testing.expectEqualSlices(u64, expected.items, found.items);
}

/// Picks up to `max` random bodies with `in_broad_phase` as the state
fn pickBodies(allocator: Allocator, c: *Context, all: []const BodyID, random: std.Random, in_broad_phase: bool, max: usize) !std.ArrayList(BodyID) {
    var result: std.ArrayList(BodyID) = .empty;
    errdefer result.deinit(allocator);
    const n = 1 + random.uintLessThan(usize, max);
    const start = random.uintLessThan(usize, all.len);
    for (0..all.len) |i| {
        const id = all[(start + i * 7) % all.len];
        if (c.body_manager.getBody(id).isInBroadPhase() != in_broad_phase) continue;
        var dup = false;
        for (result.items) |r| dup = dup or r.eql(id);
        if (dup) continue;
        try result.append(allocator, id);
        if (result.items.len == n) break;
    }
    return result;
}

/// The random sequence of operations (see the file comment), `num_ops` operations on `num_bodies` bodies
fn randomSequence(allocator: Allocator, seed: u64, num_bodies: u32, num_ops: u32) !void {
    var prng: std.Random.DefaultPrng = .init(seed);
    const random = prng.random();

    var c: Context = undefined;
    try c.init(allocator, allocator, num_bodies + 16);
    defer c.deinit();
    var shapes = try Shapes.init(allocator);
    defer shapes.deinit();

    const all = try allocator.alloc(BodyID, num_bodies);
    defer allocator.free(all);
    for (all) |*id| id.* = try shapes.createRandomBody(&c, random, 50);

    var exact = true; // The trees are tight (only after Optimize)
    var counts: [9]u32 = @splat(0);
    for (0..num_ops) |_| {
        const op = random.uintLessThan(u32, 9);
        counts[op] += 1;
        switch (op) {
            0, 1 => {
                // Add a batch, or abort it
                var ids = try pickBodies(allocator, &c, all, random, false, 60);
                defer ids.deinit(allocator);
                if (ids.items.len == 0) continue;
                const state = try c.bp().addBodiesPrepare(ids.items);
                if (op == 0) {
                    c.bp().addBodiesFinalize(ids.items, state);
                    exact = false;
                } else c.bp().addBodiesAbort(ids.items, state);
            },
            2 => {
                // Add bodies one by one (TryInsertLeaf / TryCreateNewRoot)
                var ids = try pickBodies(allocator, &c, all, random, false, 12);
                defer ids.deinit(allocator);
                for (ids.items) |*id| try c.add(id[0..1]);
                if (ids.items.len > 0) exact = false;
            },
            3 => {
                // Remove
                var ids = try pickBodies(allocator, &c, all, random, true, 40);
                defer ids.deinit(allocator);
                if (ids.items.len > 0) c.bp().removeBodies(ids.items);
            },
            4 => {
                // Move bodies and notify the broad phase
                var ids = try pickBodies(allocator, &c, all, random, true, 40);
                defer ids.deinit(allocator);
                for (ids.items) |id| {
                    const b = c.bodyMut(id);
                    const p = b.getPosition();
                    const delta = randomVec3(random, -3, 3);
                    b.setPositionAndRotationInternal(RVec3.init(p.getX() + delta.getX(), p.getY() + delta.getY(), p.getZ() + delta.getZ()), b.getRotation(), .{});
                }
                if (ids.items.len > 0) {
                    if (random.boolean()) {
                        c.bp().notifyBodiesAABBChanged(ids.items, .{});
                    } else {
                        // Between LockModifications / UnlockModifications (like PhysicsSystem's jobs): no lock taken
                        c.bp().lockModifications();
                        c.bp().notifyBodiesAABBChanged(ids.items, .{ .take_lock = false });
                        c.bp().unlockModifications();
                    }
                    exact = false;
                }
            },
            5 => try c.update(),
            6 => {
                try c.bp().optimize();
                exact = true;
            },
            7 => {
                // Change the object layer of bodies (sometimes the broad phase layer stays the same)
                var ids = try pickBodies(allocator, &c, all, random, true, 8);
                defer ids.deinit(allocator);
                for (ids.items) |id| {
                    const b = c.bodyMut(id);
                    var layer = object_layers[random.uintLessThan(usize, object_layers.len)];
                    if (b.getObjectLayer() == Layers.moving and random.boolean()) layer = Layers.hq_debris; // Same broad phase layer
                    b.setObjectLayerInternal(layer, &c.layer_interface.base);
                }
                if (ids.items.len > 0) {
                    try c.bp().notifyBodiesLayerChanged(ids.items);
                    exact = false;
                }
            },
            8 => try checkPairs(allocator, &c, random),
            else => unreachable,
        }
        _ = try validate(allocator, &c);
        try checkQueries(allocator, &c, random, exact);
    }
    for (counts) |n| try expect(n > 0);

    // In the end the optimized trees give exactly the brute force hits
    try c.bp().optimize();
    _ = try validate(allocator, &c);
    for (0..20) |_| try checkQueries(allocator, &c, random, true);
    try checkPairs(allocator, &c, random);
}

test "BroadPhaseQuadTree random operations against brute force" {
    try randomSequence(std.testing.allocator, 1, 300, 400);
    try randomSequence(std.testing.allocator, 2, 40, 200); // Few bodies: empty trees and trees with 1 body
}

test "BroadPhaseQuadTree queries with filters and early outs" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator, allocator, 64);
    defer c.deinit();

    // A row of boxes along x in the moving layer and one static box in the non moving layer
    var settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    var ids: [10]BodyID = undefined;
    for (&ids, 0..) |*id, i| {
        settings.position = RVec3.init(2.0 * @as(f32, @floatFromInt(i)), 0, 0);
        id.* = try c.createBody(&settings, true);
    }
    try c.add(&ids);
    settings.motion_type = .static;
    settings.object_layer = Layers.non_moving;
    settings.position = RVec3.init(5, 0, 0);
    var static = [_]BodyID{try c.createBody(&settings, false)};
    try c.add(&static);
    try c.bp().optimize();

    // The closest hit along the row: the ray starts before the first box, the closest hit collector prunes the rest
    {
        var collector = ClosestHitCollisionCollector(RayCastBodyCollector).init();
        defer collector.deinit();
        c.query().castRay(&RayCast.init(Vec3.init(-2, 0, 0), Vec3.init(30, 0, 0)), &collector.base, .{});
        try expect(collector.had_hit);
        try expect(collector.hit.body_id.eql(ids[0]));
        try checkApproxEqual(collector.hit.fraction, 1.5 / 30.0, .{});

        // From the other side
        var collector2 = ClosestHitCollisionCollector(RayCastBodyCollector).init();
        defer collector2.deinit();
        c.query().castRay(&RayCast.init(Vec3.init(30, 0, 0), Vec3.init(-40, 0, 0)), &collector2.base, .{});
        try expect(collector2.hit.body_id.eql(ids[9]));

        // The broad phase layer filter removes the moving layer: only the static box remains
        const only_static: zolt.SpecifiedBroadPhaseLayerFilter = .init(BroadPhaseLayers.non_moving);
        var collector3 = ClosestHitCollisionCollector(RayCastBodyCollector).init();
        defer collector3.deinit();
        c.query().castRay(&RayCast.init(Vec3.init(-2, 0, 0), Vec3.init(30, 0, 0)), &collector3.base, .{ .broad_phase_layer_filter = &only_static.base });
        try expect(collector3.hit.body_id.eql(static[0]));
    }

    // Cast a box along the row: the closest hit
    {
        var collector = ClosestHitCollisionCollector(CastShapeBodyCollector).init();
        defer collector.deinit();
        const cast: AABoxCast = .{ .box = .init(Vec3.init(-4.5, -0.5, -0.5), Vec3.init(-3.5, 0.5, 0.5)), .direction = Vec3.init(40, 0, 0) };
        c.query().castAABox(&cast, &collector.base, .{});
        try expect(collector.hit.body_id.eql(ids[0]));
        try checkApproxEqual(collector.hit.fraction, 3.0 / 40.0, .{});
        var collector2 = ClosestHitCollisionCollector(CastShapeBodyCollector).init();
        defer collector2.deinit();
        c.bp().castAABoxNoLock(&cast, &collector2.base, &.{}, &.{});
        try expect(collector2.hit.body_id.eql(ids[0]));
    }

    // The any hit collector stops after one hit (ShouldAbort)
    {
        var collector = AnyHitCollisionCollector(CollideShapeBodyCollector).init();
        defer collector.deinit();
        c.query().collideAABox(&AABox.init(Vec3.replicate(-100), Vec3.replicate(100)), &collector.base, .{});
        try expect(collector.had_hit);
        var collector2 = AnyHitCollisionCollector(CollideShapeBodyCollector).init();
        defer collector2.deinit();
        c.query().collideSphere(Vec3.init(5, 0, 0), 0.1, &collector2.base, .{ .object_layer_filter = &(MaskObjectLayerFilter{ .mask = @as(u32, 1) << Layers.moving }).base });
        try expect(!collector2.had_hit); // Only the static box is there
        c.query().collidePoint(Vec3.init(4, 0, 0), &collector2.base, .{});
        try expect(collector2.had_hit);
    }

    // The bounds of everything
    const bounds = c.query().getBounds();
    try expect(bounds.min.eql(Vec3.init(-0.5, -0.5, -0.5)));
    try expect(bounds.max.eql(Vec3.init(18.5, 0.5, 0.5)));

    // Remove everything: the trees are empty, the queries find nothing and the bounds are invalid
    c.bp().removeBodies(&ids);
    c.bp().removeBodies(&static);
    var collector = AllHitCollisionCollector(CollideShapeBodyCollector).init(allocator);
    defer collector.deinit();
    c.query().collideAABox(&AABox.init(Vec3.replicate(-100), Vec3.replicate(100)), &collector.base, .{});
    try expect(!collector.hadHit());
    try c.bp().optimize();
    try expect(!c.query().getBounds().isValid());
    try expectEqual(@as(u32, 0), try validate(allocator, &c));
}

test "BroadPhaseQuadTree updates rebuild one dirty layer at a time" {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator, allocator, 64);
    defer c.deinit();
    var settings = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 0.5, .{})).asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    var ids: [8]BodyID = undefined;
    for (&ids, 0..) |*id, i| {
        settings.object_layer = if (i % 2 == 0) Layers.moving else Layers.moving2;
        settings.position = RVec3.init(@floatFromInt(i), 0, 0);
        id.* = try c.createBody(&settings, true);
        try c.add(id[0..1]); // One by one: changed nodes
    }
    const moving = &c.quad_tree.layers[BroadPhaseLayers.moving.getValue()];
    const moving2 = &c.quad_tree.layers[BroadPhaseLayers.moving2.getValue()];
    try expect(moving.isDirty() and moving2.isDirty());

    // The first update rebuilds the first dirty layer, the second one the next
    try c.update();
    try expect(!moving.isDirty() and moving2.isDirty());
    try expect(!moving.canBeUpdated()); // The old tree is discarded by the next frameSync
    try c.update();
    try expect(!moving.isDirty() and !moving2.isDirty());
    try expect(moving.canBeUpdated());
    try c.update(); // Nothing to update: a zeroed state
    _ = try validate(allocator, &c);

    // A zeroed update state does nothing
    c.bp().lockModifications();
    c.bp().updateFinalize(&.{});
    c.bp().unlockModifications();

    // The query lock alternates with every swap of a tree
    try expectEqual(@as(u32, 0), c.quad_tree.query_lock_idx.load(.monotonic));
}

test "BroadPhaseQuadTree queries from several threads while bodies move, are added, removed and the trees are rebuilt" {
    for (0..3) |round| try multiThreaded(round);
}

fn multiThreaded(round: usize) !void {
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator, allocator, 256);
    defer c.deinit();

    var settings = BodyCreationSettings.initWithShape((try BoxShape.create(allocator, Vec3.replicate(0.5), .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer settings.deinit();

    // Fixed bodies at y = 0 (found by every query), moving bodies at y = 10, changing bodies at y = 20
    var fixed: [32]BodyID = undefined;
    for (&fixed, 0..) |*id, i| {
        settings.position = RVec3.init(2.0 * @as(f32, @floatFromInt(i)), 0, 0);
        id.* = try c.createBody(&settings, false);
    }
    try c.add(&fixed);
    settings.motion_type = .dynamic;
    settings.object_layer = Layers.moving;
    var moving: [32]BodyID = undefined;
    for (&moving, 0..) |*id, i| {
        settings.position = RVec3.init(2.0 * @as(f32, @floatFromInt(i)), 10, 0);
        id.* = try c.createBody(&settings, true);
        try c.add(id[0..1]);
    }
    var changing: [32]BodyID = undefined;
    for (&changing, 0..) |*id, i| {
        settings.position = RVec3.init(2.0 * @as(f32, @floatFromInt(i)), 20, 0);
        id.* = try c.createBody(&settings, true);
    }

    const Shared = struct {
        c: *Context,
        stop: std.atomic.Value(bool) = .init(false),
        failures: std.atomic.Value(u32) = .init(0),
        queries: std.atomic.Value(u32) = .init(0),
        /// The number of updates of the main thread: the adder does one add / remove per update, the removed bodies
        /// leave empty nodes until their tree is rebuilt (Jolt runs out of nodes otherwise too)
        updates: std.atomic.Value(u32) = .init(0),
        fixed: *const [32]BodyID,

        fn reader(s: *@This(), seed: u64) void {
            var prng: std.Random.DefaultPrng = .init(seed);
            const random = prng.random();
            var collector = AllHitCollisionCollector(CollideShapeBodyCollector).init(std.testing.allocator);
            defer collector.deinit();
            var ray_collector = AllHitCollisionCollector(RayCastBodyCollector).init(std.testing.allocator);
            defer ray_collector.deinit();
            while (!s.stop.load(.acquire)) {
                // The fixed bodies are always there, the other bodies at most once
                collector.reset();
                s.c.query().collideAABox(&AABox.init(Vec3.replicate(-1000), Vec3.replicate(1000)), &collector.base, .{});
                var ok = if (collector.checkError()) |_| true else |_| false;
                var num_fixed: u32 = 0;
                for (collector.hits.items, 0..) |a, i| {
                    for (s.fixed) |f| num_fixed += @intFromBool(f.eql(a));
                    for (collector.hits.items[i + 1 ..]) |b| ok = ok and !a.eql(b);
                }
                ok = ok and num_fixed == 32 and collector.hits.items.len <= 96;

                // A ray along the fixed bodies hits all of them
                ray_collector.reset();
                const y = 0.4 * random.float(f32) - 0.2;
                s.c.query().castRay(&RayCast.init(Vec3.init(-1, y, 0), Vec3.init(70, 0, 0)), &ray_collector.base, .{});
                var num_ray_fixed: u32 = 0;
                for (ray_collector.hits.items) |hit| {
                    for (s.fixed) |f| num_ray_fixed += @intFromBool(f.eql(hit.body_id));
                }
                ok = ok and num_ray_fixed == 32;
                if (!ok) _ = s.failures.fetchAdd(1, .monotonic);
                _ = s.queries.fetchAdd(1, .monotonic);
            }
        }

        fn adder(s: *@This(), ids: *[32]BodyID) void {
            var prng: std.Random.DefaultPrng = .init(77);
            const random = prng.random();
            var last_update: u32 = 0;
            while (!s.stop.load(.acquire)) {
                const update = s.updates.load(.acquire);
                if (update == last_update) {
                    std.Thread.yield() catch {};
                    continue;
                }
                last_update = update;

                // Prepare on this thread (no lock), finalize, sometimes abort; remove again
                const n = 1 + random.uintLessThan(usize, 32);
                const state = s.c.bp().addBodiesPrepare(ids[0..n]) catch {
                    _ = s.failures.fetchAdd(1, .monotonic);
                    return;
                };
                if (random.uintLessThan(u32, 4) == 0) {
                    s.c.bp().addBodiesAbort(ids[0..n], state);
                } else {
                    s.c.bp().addBodiesFinalize(ids[0..n], state);
                    s.c.bp().removeBodies(ids[0..n]);
                }
            }
        }
    };

    var shared: Shared = .{ .c = &c, .fixed = &fixed };
    var readers: [3]std.Thread = undefined;
    for (&readers, 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Shared.reader, .{ &shared, @as(u64, round * 10 + i) });
    const adder = try std.Thread.spawn(.{}, Shared.adder, .{ &shared, &changing });

    // This thread moves bodies, notifies the broad phase and rebuilds the trees (the PhysicsSystem protocol)
    var prng: std.Random.DefaultPrng = .init(round);
    const random = prng.random();
    for (0..300) |iteration| {
        for (&moving) |id| {
            const b = c.bodyMut(id);
            const p = b.getPosition();
            b.setPositionAndRotationInternal(RVec3.init(p.getX() + random.float(f32) - 0.5, p.getY(), p.getZ() + random.float(f32) - 0.5), Quat.identity(), .{});
        }
        c.bp().notifyBodiesAABBChanged(&moving, .{});
        if (iteration % 3 == 0) {
            try c.update();
            _ = shared.updates.fetchAdd(1, .release);
        }
        // Not Optimize: it swaps trees without switching the query lock (QuadTree.updateFinalize directly), so its
        // second FrameSync frees the old trees while queries may still walk them. Jolt's Optimize is meant for the
        // start (PhysicsSystem::OptimizeBroadPhase: not while bodies are modified from another thread).
    }
    shared.stop.store(true, .release);
    for (readers) |t| t.join();
    adder.join();
    try expectEqual(@as(u32, 0), shared.failures.load(.monotonic));
    try expect(shared.queries.load(.monotonic) > 0);
    try c.update();
    _ = try validate(allocator, &c);
}

test "BroadPhaseQuadTree a ray along bodies that were added one by one grows the walk stack" {
    // Every single body creates a new root on top of the old one: a deep tree. The old root (child 0) has huge bounds,
    // so the ray enters it first (its fraction is negative) and the leaves of every level stay on the stack.
    const allocator = std.testing.allocator;
    var c: Context = undefined;
    try c.init(allocator, allocator, 512);
    defer c.deinit();
    var settings = BodyCreationSettings.initWithShape((try SphereShape.create(allocator, 0.4, .{})).asShape(), RVec3.zero(), Quat.identity(), .static, Layers.non_moving);
    defer settings.deinit();
    var ids: [400]BodyID = undefined;
    for (&ids, 0..) |*id, i| {
        settings.position = RVec3.init(@floatFromInt(i), 0, 0);
        id.* = try c.createBody(&settings, false);
        try c.add(id[0..1]);
    }
    const tree = &c.quad_tree.layers[BroadPhaseLayers.non_moving.getValue()];
    const ts = &tree.tree_state;
    try expect(tree.getMaxTreeDepth(ts.root_node[ts.root_node_index.load(.monotonic)].getNodeID()) > 100);

    var collector = AllHitCollisionCollector(RayCastBodyCollector).init(allocator);
    defer collector.deinit();
    c.query().castRay(&RayCast.init(Vec3.init(-1, 0, 0), Vec3.init(500, 0, 0)), &collector.base, .{});
    try collector.checkError();
    try expectEqual(@as(usize, 400), collector.hits.items.len);
    var cast_collector = AllHitCollisionCollector(CastShapeBodyCollector).init(allocator);
    defer cast_collector.deinit();
    c.query().castAABox(&.{ .box = .init(Vec3.init(-1.1, -0.1, -0.1), Vec3.init(-0.9, 0.1, 0.1)), .direction = Vec3.init(500, 0, 0) }, &cast_collector.base, .{});
    try expectEqual(@as(usize, 400), cast_collector.hits.items.len);

    // After the rebuild the tree is shallow
    try c.bp().optimize();
    try expect(tree.getMaxTreeDepth(ts.root_node[ts.root_node_index.load(.monotonic)].getNodeID()) < 10);
    collector.reset();
    c.query().castRay(&RayCast.init(Vec3.init(-1, 0, 0), Vec3.init(500, 0, 0)), &collector.base, .{});
    try expectEqual(@as(usize, 400), collector.hits.items.len);
}

test "BroadPhaseQuadTree out of memory in create and initBroadPhase" {
    const Ctx = struct {
        fn run(allocator: Allocator) !void {
            var c: Context = undefined;
            try c.init(allocator, allocator, 1024);
            c.deinit();
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, Ctx.run, .{});
}

test "BroadPhaseQuadTree out of memory in a sequence of operations" {
    // Every allocation fails in turn: the error arrives and nothing leaks
    const Ctx = struct {
        fn run(allocator: Allocator) !void {
            var prng: std.Random.DefaultPrng = .init(5);
            const random = prng.random();
            var c: Context = undefined;
            try c.init(allocator, allocator, 64);
            defer c.deinit();
            var shapes = try Shapes.init(allocator);
            defer shapes.deinit();
            var ids: [40]BodyID = undefined;
            for (&ids) |*id| id.* = try shapes.createRandomBody(&c, random, 20);
            try c.add(ids[0..30]);
            for (ids[30..]) |*id| try c.add(id[0..1]);
            for (ids[0..10]) |id| c.bodyMut(id).setPositionAndRotationInternal(RVec3.init(1, 2, 3), Quat.identity(), .{});
            c.bp().notifyBodiesAABBChanged(ids[0..10], .{});
            for (0..3) |_| try c.update();
            try c.bp().optimize();
            for (ids[0..5]) |id| c.bodyMut(id).setObjectLayerInternal(Layers.lq_debris, &c.layer_interface.base);
            try c.bp().notifyBodiesLayerChanged(ids[0..5]);
            var collector = AllHitCollisionCollector(BodyPairCollector).init(allocator);
            defer collector.deinit();
            var active: [10]BodyID = undefined;
            var n: usize = 0;
            for (ids) |id| {
                if (n < active.len and c.bodyMut(id).isActive()) {
                    active[n] = id;
                    n += 1;
                }
            }
            if (n > 0) try c.bp().findCollidingPairs(active[0..n], 0.1, &c.object_vs_broad_phase_layer_filter.base, &c.object_layer_pair_filter.base, &collector.base);
            try collector.checkError();
            c.bp().removeBodies(ids[10..]);
            try c.update();
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, Ctx.run, .{});
}

/// A context whose broad phase allocates from a FailingAllocator (fails nothing until `failAfter`)
const FailingContext = struct {
    failing: std.testing.FailingAllocator,
    c: Context,
    shapes: Shapes,
    ids: [60]BodyID,

    fn init(self: *FailingContext, allocator: Allocator) !void {
        self.failing = .init(allocator, .{});
        try self.c.init(allocator, self.failing.allocator(), 128);
        errdefer self.c.deinit();
        self.shapes = try Shapes.init(allocator);
        errdefer self.shapes.deinit();
        var prng: std.Random.DefaultPrng = .init(11);
        const random = prng.random();
        for (&self.ids) |*id| id.* = try self.shapes.createRandomBody(&self.c, random, 30);
        try self.c.add(self.ids[0..30]);
        for (self.ids[30..40]) |*id| try self.c.add(id[0..1]);
        try self.c.update();
    }

    fn deinit(self: *FailingContext) void {
        self.shapes.deinit();
        self.c.deinit();
    }

    /// The n-th allocation from now fails (n = 0: the next one)
    fn failAfter(self: *FailingContext, n: usize) void {
        self.failing.fail_index = self.failing.alloc_index + n;
    }

    fn noFailures(self: *FailingContext) void {
        self.failing.fail_index = std.math.maxInt(usize);
    }
};

fn expectSameDump(allocator: Allocator, before: []const u32, qt: *const BroadPhaseQuadTree) !void {
    const after = try dumpTrees(allocator, qt);
    defer allocator.free(after);
    try std.testing.expectEqualSlices(u32, before, after);
}

test "BroadPhaseQuadTree addBodiesPrepare out of heap memory leaves the broad phase unchanged" {
    const allocator = std.testing.allocator;
    var f: FailingContext = undefined;
    try f.init(allocator);
    defer f.deinit();

    var num_failures: usize = 0;
    for (0..10) |k| {
        const before = try dumpTrees(allocator, f.c.quad_tree);
        defer allocator.free(before);
        var ids = f.ids[40..60].*; // Several layers
        f.failAfter(k);
        const result = f.c.bp().addBodiesPrepare(&ids);
        f.noFailures();
        if (result) |state| {
            f.c.bp().addBodiesAbort(&ids, state);
            try expectSameDump(allocator, before, f.c.quad_tree);
            break;
        } else |err| {
            try expectEqual(error.OutOfMemory, err);
            num_failures += 1;
            try expectSameDump(allocator, before, f.c.quad_tree);
            for (ids) |id| try expect(!f.c.bodyMut(id).isInBroadPhase());
        }
    } else return error.TestUnexpectedResult;
    try expect(num_failures >= 3); // The layer states, BuildTree of the first and of a later layer

    // And now it works
    var ids = f.ids[40..60].*;
    try f.c.add(&ids);
    try expectEqual(@as(u32, 60), try validate(allocator, &f.c));
}

test "BroadPhaseQuadTree updatePrepare and optimize out of heap memory leave the trees unchanged" {
    const allocator = std.testing.allocator;
    var f: FailingContext = undefined;
    try f.init(allocator);
    defer f.deinit();

    // Make every layer dirty
    var ids = f.ids[40..60].*;
    try f.c.add(&ids);
    for (f.ids[0..40]) |id| {
        const b = f.c.bodyMut(id);
        b.setPositionAndRotationInternal(b.getPosition().addVec3(Vec3.init(0.5, 0, 0)), b.getRotation(), .{});
    }
    f.c.bp().notifyBodiesAABBChanged(f.ids[0..40], .{});

    // updatePrepare: the node IDs, BuildTree's centers
    var num_failures: usize = 0;
    for (0..20) |k| {
        f.c.bp().frameSync();
        const before = try dumpTrees(allocator, f.c.quad_tree);
        defer allocator.free(before);
        f.c.bp().lockModifications();
        f.failAfter(k);
        const result = f.c.bp().updatePrepare();
        f.noFailures();
        if (result) |state| {
            f.c.bp().updateFinalize(&state);
            f.c.bp().unlockModifications();
            if (state.data[0] == null) break; // Nothing was dirty anymore
        } else |err| {
            f.c.bp().unlockModifications();
            try expectEqual(error.OutOfMemory, err);
            num_failures += 1;
            try expectSameDump(allocator, before, f.c.quad_tree);
            _ = try validate(allocator, &f.c);
        }
    } else return error.TestUnexpectedResult;
    try expect(num_failures >= 2);

    // optimize: the layers before the failing one are optimized, the broad phase stays valid
    num_failures = 0;
    for (0..40) |k| {
        f.failAfter(k);
        const result = f.c.bp().optimize();
        f.noFailures();
        _ = try validate(allocator, &f.c);
        if (result) |_| break else |err| {
            try expectEqual(error.OutOfMemory, err);
            num_failures += 1;
        }
    } else return error.TestUnexpectedResult;
    try expect(num_failures >= 4);
    try expectEqual(@as(u32, 60), try validate(allocator, &f.c));
}

test "BroadPhaseQuadTree notifyBodiesLayerChanged out of heap memory leaves everything unchanged" {
    const allocator = std.testing.allocator;
    var f: FailingContext = undefined;
    try f.init(allocator);
    defer f.deinit();

    var num_failures: usize = 0;
    for (0..10) |k| {
        const before = try dumpTrees(allocator, f.c.quad_tree);
        defer allocator.free(before);

        // Change the layer of a few bodies (one keeps its broad phase layer: moving -> hq_debris)
        var ids = [_]BodyID{ f.ids[0], f.ids[1], f.ids[2], f.ids[3] };
        var old_layers: [ids.len]ObjectLayer = undefined;
        for (&ids, &old_layers, 0..) |id, *old, i| {
            const b = f.c.bodyMut(id);
            old.* = b.getObjectLayer();
            b.setObjectLayerInternal(if (i == 0 and old.* == Layers.moving) Layers.hq_debris else if (old.* == Layers.lq_debris) Layers.sensor else Layers.lq_debris, &f.c.layer_interface.base);
        }
        f.failAfter(k);
        const result = f.c.bp().notifyBodiesLayerChanged(&ids);
        f.noFailures();
        if (result) |_| {
            _ = try validate(allocator, &f.c);
            break;
        } else |err| {
            try expectEqual(error.OutOfMemory, err);
            num_failures += 1;
            // Like BodyInterface.setObjectLayer: restore the layers, the broad phase is unchanged
            for (ids, old_layers) |id, old| f.c.bodyMut(id).setObjectLayerInternal(old, &f.c.layer_interface.base);
            try expectSameDump(allocator, before, f.c.quad_tree);
            _ = try validate(allocator, &f.c);
        }
    } else return error.TestUnexpectedResult;
    try expect(num_failures >= 2);
}
