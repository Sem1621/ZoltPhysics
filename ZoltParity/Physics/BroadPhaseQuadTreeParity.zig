//! Parity tests for the QuadTree broad phase (Phase 5 A1, SimulationArchitecture D9): Jolt's BroadPhaseQuadTree and
//! Zolt's are driven with the same BodyManager contents through the same random sequences of operations, and after
//! every operation everything is compared bit for bit:
//! - the state of the broad phase (`dumpTrees`, the C++ side reads the private members): the next layer to update, the
//!   query lock index, per layer the body count, the dirty flag, the root indices of both trees, the batch of nodes to
//!   free, the nodes of the current and the old tree in walk order (node index, parent, changed flag, the 4 child IDs
//!   and the 24 bounds), the nodes of the new tree of a pending update, and the tracking of every body;
//! - the orders that the operations write to ioBodies (AddBodiesPrepare, RemoveBodies, NotifyBodiesAABBChanged,
//!   NotifyBodiesLayerChanged, FindCollidingPairs sort them);
//! - every query's hits in collector order with their fractions (CastRay, CollideAABox, CollideSphere, CollidePoint,
//!   CollideOrientedBox, CastAABox, CastAABoxNoLock) with 3 collectors (all hits, closest hit, force early out after k
//!   hits) and broad phase / object layer filters, GetBounds;
//! - FindCollidingPairs called single threaded: the pairs in order.
//! Operations: batch adds (finalize, abort), single adds (TryInsertLeaf / TryCreateNewRoot), removes, moves with and
//! without the lock, layer changes, FrameSync, updates (also left pending while queries, FindCollidingPairs and aborted
//! adds run on the old tree), Optimize. Scenes: uniform, coincident, collinear, clustered, exponentially spaced
//! (degenerate partitions and deep trees), large coordinates, 1 to 6 broad phase layers, random layer filter masks.
//! The node indices in the dumps prove that both sides allocate and free nodes in the same order.
//! A scene of bodies added one by one without rebuilds makes the trees deeper than 50 levels (the walk stacks grow on
//! the heap). Every operation kind must run in every scene.
//! Proof that it bites: QuadTree.partition with `<= split` instead of `< split` is reported as "dump differs after
//! operation 2" (scene "coincident"); the collide visitors without CountAndSortTrues as "query hits differs".
//! C ABI wrappers: ZoltParity/Physics/BroadPhaseQuadTreeReference.cpp.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Allocator = std.mem.Allocator;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const RVec3 = zolt.RVec3;
const AABox = zolt.AABox;
const OrientedBox = zolt.OrientedBox;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyPair = zolt.BodyPair;
const BodyManager = zolt.BodyManager;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShape = zolt.BoxShape;
const SphereShape = zolt.SphereShape;
const MotionType = zolt.MotionType;
const ObjectLayer = zolt.ObjectLayer;
const BroadPhase = zolt.BroadPhase;
const BroadPhaseQuadTree = zolt.BroadPhaseQuadTree;
const QuadTree = zolt.QuadTree;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const BroadPhaseLayerFilter = zolt.BroadPhaseLayerFilter;
const ObjectLayerFilter = zolt.ObjectLayerFilter;
const ObjectLayerPairFilter = zolt.ObjectLayerPairFilter;
const ObjectVsBroadPhaseLayerFilter = zolt.ObjectVsBroadPhaseLayerFilter;
const BroadPhaseCastResult = zolt.BroadPhaseCastResult;
const RayCastBodyCollector = zolt.RayCastBodyCollector;
const CastShapeBodyCollector = zolt.CastShapeBodyCollector;
const CollideShapeBodyCollector = zolt.CollideShapeBodyCollector;
const BodyPairCollector = zolt.BodyPairCollector;
const RayCast = zolt.RayCast;
const AABoxCast = zolt.AABoxCast;

const io = std.testing.io;

const invalid_index: u32 = 0xffffffff;
const num_object_layers = 8;

/// Must match BQWorldDesc in BroadPhaseQuadTreeReference.cpp
const WorldDesc = extern struct {
    max_bodies: u32,
    num_broad_phase_layers: u32,
    object_vs_broad_phase_mask: [num_object_layers]u32,
    object_pair_mask: [num_object_layers]u32,
};

/// Must match BQBodyDesc in BroadPhaseQuadTreeReference.cpp
const BodyDesc = extern struct {
    position: [3]f32,
    rotation: [4]f32,
    size: [3]f32,
    shape: u32,
    motion_type: u32,
    object_layer: u32,
    padding: u32 = 0,
};

/// Must match BQQuery in BroadPhaseQuadTreeReference.cpp
const Query = extern struct {
    f: [16]f32 = @splat(0),
    kind: u32,
    collector_mode: u32,
    max_hits: u32,
    broad_phase_layer_mask: u32,
    object_layer_mask: u32,
    padding: [3]u32 = @splat(0),
};

/// Must match BQHit in BroadPhaseQuadTreeReference.cpp
const Hit = extern struct {
    body_id: u32,
    fraction: f32,
};

const jolt = struct {
    extern fn jolt_bpqt_create(desc: *const WorldDesc) *anyopaque;
    extern fn jolt_bpqt_destroy(world: *anyopaque) void;
    extern fn jolt_bpqt_create_body(world: *anyopaque, desc: *const BodyDesc) u32;
    extern fn jolt_bpqt_add(world: *anyopaque, ids: [*]const u32, count: u32, abort: u32, activate: u32, out_order: [*]u32) void;
    extern fn jolt_bpqt_remove(world: *anyopaque, ids: [*]const u32, count: u32, out_order: [*]u32) void;
    extern fn jolt_bpqt_move(world: *anyopaque, ids: [*]const u32, count: u32, transforms: [*]const f32, take_lock: u32, out_order: [*]u32) void;
    extern fn jolt_bpqt_set_layers(world: *anyopaque, ids: [*]const u32, count: u32, layers: [*]const u32, out_order: [*]u32) void;
    extern fn jolt_bpqt_frame_sync(world: *anyopaque) void;
    extern fn jolt_bpqt_optimize(world: *anyopaque) void;
    extern fn jolt_bpqt_update_prepare(world: *anyopaque, out_state: *[2]u32) void;
    extern fn jolt_bpqt_update_finalize(world: *anyopaque) void;
    extern fn jolt_bpqt_query(world: *anyopaque, query: *const Query, out_hits: [*]Hit, capacity: u32) u32;
    extern fn jolt_bpqt_get_bounds(world: *anyopaque, out_bounds: *[6]f32) void;
    extern fn jolt_bpqt_find_colliding_pairs(world: *anyopaque, ids: [*]const u32, count: u32, speculative_contact_distance: f32, out_order: [*]u32, out_pairs: [*]u32, capacity: u32) u32;
    extern fn jolt_bpqt_dump(world: *anyopaque, out: [*]u32, capacity: u32) u32;
};

// -------------------------------------------------------------------------------------------------------------------
// The Zolt side

/// Broad phase layer = object layer % number of layers
const LayerInterface = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };
    base: BroadPhaseLayerInterface = .init(LayerInterface),
    num_layers: u32,
    pub fn getNumBroadPhaseLayers(self: *const LayerInterface) u32 {
        return self.num_layers;
    }
    pub fn getBroadPhaseLayer(self: *const LayerInterface, layer: ObjectLayer) BroadPhaseLayer {
        return .init(@intCast(layer % self.num_layers));
    }
};

const ObjectVsBroadPhaseFilter = struct {
    pub const overrides = .{.shouldCollide};
    base: ObjectVsBroadPhaseLayerFilter = .init(ObjectVsBroadPhaseFilter),
    mask: [num_object_layers]u32,
    pub fn shouldCollide(self: *const ObjectVsBroadPhaseFilter, layer1: ObjectLayer, layer2: BroadPhaseLayer) bool {
        return (self.mask[layer1 % num_object_layers] >> @intCast(layer2.getValue())) & 1 != 0;
    }
};

const ObjectPairFilter = struct {
    pub const overrides = .{.shouldCollide};
    base: ObjectLayerPairFilter = .init(ObjectPairFilter),
    mask: [num_object_layers]u32,
    pub fn shouldCollide(self: *const ObjectPairFilter, layer1: ObjectLayer, layer2: ObjectLayer) bool {
        return (self.mask[layer1 % num_object_layers] >> @intCast(layer2 % 32)) & 1 != 0;
    }
};

const MaskBroadPhaseLayerFilter = struct {
    pub const overrides = .{.shouldCollide};
    base: BroadPhaseLayerFilter = .init(MaskBroadPhaseLayerFilter),
    mask: u32,
    pub fn shouldCollide(self: *const MaskBroadPhaseLayerFilter, layer: BroadPhaseLayer) bool {
        return (self.mask >> @intCast(layer.getValue())) & 1 != 0;
    }
};

const MaskObjectLayerFilter = struct {
    pub const overrides = .{.shouldCollide};
    base: ObjectLayerFilter = .init(MaskObjectLayerFilter),
    mask: u32,
    pub fn shouldCollide(self: *const MaskObjectLayerFilter, layer: ObjectLayer) bool {
        return (self.mask >> @intCast(layer % 32)) & 1 != 0;
    }
};

/// The collector of the 3 modes (see BQRecordingCollector in the C++ file)
fn RecordingCollector(comptime Base: type) type {
    return struct {
        const Self = @This();
        pub const overrides = .{.addHit};

        base: Base = .init(Self),
        mode: u32,
        max_hits: u32,
        hits: *std.ArrayList(Hit),
        allocator: Allocator,
        failed: bool = false,

        pub fn addHit(self: *Self, result: *const Base.ResultType) void {
            var hit: Hit = undefined;
            if (Base.ResultType == BodyID) {
                hit = .{ .body_id = result.getIndexAndSequenceNumber(), .fraction = 0.0 };
            } else {
                hit = .{ .body_id = result.body_id.getIndexAndSequenceNumber(), .fraction = result.fraction };
                if (self.mode == 1 and result.fraction < self.base.getEarlyOutFraction())
                    self.base.updateEarlyOutFraction(result.fraction);
            }
            self.hits.append(self.allocator, hit) catch {
                self.failed = true;
            };
            if (self.mode == 2 and self.hits.items.len >= self.max_hits)
                self.base.forceEarlyOut();
        }
    };
}

const PairCollector = struct {
    pub const overrides = .{.addHit};
    base: BodyPairCollector = .init(PairCollector),
    pairs: *std.ArrayList(u32),
    allocator: Allocator,
    failed: bool = false,
    pub fn addHit(self: *PairCollector, pair: *const BodyPair) void {
        self.pairs.appendSlice(self.allocator, &.{ pair.body_a.getIndexAndSequenceNumber(), pair.body_b.getIndexAndSequenceNumber() }) catch {
            self.failed = true;
        };
    }
};

fn loadVec3(v: []const f32) Vec3 {
    return Vec3.init(v[0], v[1], v[2]);
}

fn toIDs(ids: []BodyID, raw: []const u32) void {
    for (ids, raw) |*id, r| id.* = .init(r);
}

fn fromIDs(raw: []u32, ids: []const BodyID) void {
    for (raw, ids) |*r, id| r.* = id.getIndexAndSequenceNumber();
}

/// The Zolt world, initialized in place (the broad phase points to the body manager)
const ZoltWorld = struct {
    allocator: Allocator,
    layer_interface: LayerInterface,
    object_vs_broad_phase_filter: ObjectVsBroadPhaseFilter,
    pair_filter: ObjectPairFilter,
    body_manager: BodyManager,
    quad_tree: *BroadPhaseQuadTree,
    pending_state: BroadPhase.UpdateState,
    pending: bool,
    scratch: std.ArrayList(BodyID),

    fn init(self: *ZoltWorld, allocator: Allocator, desc: *const WorldDesc) !void {
        self.* = .{
            .allocator = allocator,
            .layer_interface = .{ .num_layers = desc.num_broad_phase_layers },
            .object_vs_broad_phase_filter = .{ .mask = desc.object_vs_broad_phase_mask },
            .pair_filter = .{ .mask = desc.object_pair_mask },
            .body_manager = .empty,
            .quad_tree = undefined,
            .pending_state = .{},
            .pending = false,
            .scratch = .empty,
        };
        try self.body_manager.init(allocator, io, desc.max_bodies, 0, &self.layer_interface.base);
        errdefer self.body_manager.deinit();
        self.quad_tree = try BroadPhaseQuadTree.create(allocator);
        errdefer self.quad_tree.base.base.destroy();
        try self.quad_tree.asBroadPhase().initBroadPhase(&self.body_manager, &self.layer_interface.base);
    }

    fn deinit(self: *ZoltWorld) void {
        self.scratch.deinit(self.allocator);
        self.quad_tree.base.base.destroy();
        self.body_manager.deinit();
    }

    fn bp(self: *ZoltWorld) *BroadPhase {
        return self.quad_tree.asBroadPhase();
    }

    fn ids(self: *ZoltWorld, raw: []const u32) ![]BodyID {
        self.scratch.clearRetainingCapacity();
        try self.scratch.resize(self.allocator, raw.len);
        toIDs(self.scratch.items, raw);
        return self.scratch.items;
    }

    fn createBody(self: *ZoltWorld, desc: *const BodyDesc) !u32 {
        const shape = if (desc.shape == 0)
            (try BoxShape.create(self.allocator, loadVec3(&desc.size), .{ .convex_radius = 0.0 })).asShape()
        else
            (try SphereShape.create(self.allocator, desc.size[0], .{})).asShape();
        var settings = BodyCreationSettings.initWithShape(shape, RVec3.init(desc.position[0], desc.position[1], desc.position[2]), Quat.init(desc.rotation[0], desc.rotation[1], desc.rotation[2], desc.rotation[3]), @enumFromInt(desc.motion_type), @intCast(desc.object_layer));
        defer settings.deinit();
        const body = try self.body_manager.allocateBody(&settings);
        if (!self.body_manager.addBody(body)) {
            self.body_manager.freeBody(body);
            return BodyID.invalid_body_id;
        }
        return body.getID().getIndexAndSequenceNumber();
    }

    fn add(self: *ZoltWorld, raw: []const u32, abort: bool, activate: bool, out_order: []u32) !void {
        const body_ids = try self.ids(raw);
        const state = try self.bp().addBodiesPrepare(body_ids);
        fromIDs(out_order, body_ids);
        if (abort) {
            self.bp().addBodiesAbort(body_ids, state);
            return;
        }
        self.bp().addBodiesFinalize(body_ids, state);
        if (activate) {
            for (body_ids) |id| {
                if (self.body_manager.getBody(id).isDynamic())
                    self.body_manager.activateBodies(&.{id});
            }
        }
    }

    fn remove(self: *ZoltWorld, raw: []const u32, out_order: []u32) !void {
        const body_ids = try self.ids(raw);
        self.body_manager.deactivateBodies(body_ids);
        self.bp().removeBodies(body_ids);
        fromIDs(out_order, body_ids);
    }

    fn move(self: *ZoltWorld, raw: []const u32, transforms: []const f32, take_lock: bool, out_order: []u32) !void {
        const body_ids = try self.ids(raw);
        for (body_ids, 0..) |id, i| {
            const t = transforms[7 * i ..];
            self.body_manager.getBodyMut(id).setPositionAndRotationInternal(RVec3.init(t[0], t[1], t[2]), Quat.init(t[3], t[4], t[5], t[6]), .{});
        }
        if (take_lock) {
            self.bp().notifyBodiesAABBChanged(body_ids, .{ .take_lock = true });
        } else {
            self.bp().lockModifications();
            self.bp().notifyBodiesAABBChanged(body_ids, .{ .take_lock = false });
            self.bp().unlockModifications();
        }
        fromIDs(out_order, body_ids);
    }

    fn setLayers(self: *ZoltWorld, raw: []const u32, layers: []const u32, out_order: []u32) !void {
        const body_ids = try self.ids(raw);
        for (body_ids, layers) |id, layer|
            self.body_manager.getBodyMut(id).setObjectLayerInternal(@intCast(layer), &self.layer_interface.base);
        try self.bp().notifyBodiesLayerChanged(body_ids);
        fromIDs(out_order, body_ids);
    }

    fn updatePrepare(self: *ZoltWorld) ![2]u32 {
        self.bp().lockModifications();
        self.pending_state = self.bp().updatePrepare() catch |err| {
            self.bp().unlockModifications();
            return err;
        };
        self.pending = true;
        const impl: *const BroadPhaseQuadTree.UpdateStateImpl = @ptrCast(&self.pending_state);
        const tree = impl.tree orelse return .{ invalid_index, invalid_index };
        return .{ @intCast((@intFromPtr(tree) - @intFromPtr(self.quad_tree.layers.ptr)) / @sizeOf(QuadTree)), impl.update_state.root_node_id.id };
    }

    fn updateFinalize(self: *ZoltWorld) void {
        self.bp().updateFinalize(&self.pending_state);
        self.bp().unlockModifications();
        self.pending = false;
    }

    fn query(self: *ZoltWorld, q: *const Query, hits: *std.ArrayList(Hit)) !void {
        const f = &q.f;
        const bp_filter: MaskBroadPhaseLayerFilter = .{ .mask = q.broad_phase_layer_mask };
        const ol_filter: MaskObjectLayerFilter = .{ .mask = q.object_layer_mask };
        const filters: zolt.BroadPhaseQueryFilters = .{ .broad_phase_layer_filter = &bp_filter.base, .object_layer_filter = &ol_filter.base };
        const query_interface = self.quad_tree.base.asQuery();
        hits.clearRetainingCapacity();
        switch (q.kind) {
            0 => {
                var collector: RecordingCollector(RayCastBodyCollector) = .{ .mode = q.collector_mode, .max_hits = q.max_hits, .hits = hits, .allocator = self.allocator };
                query_interface.castRay(&RayCast.init(loadVec3(f[0..3]), loadVec3(f[3..6])), &collector.base, filters);
                if (collector.failed) return error.OutOfMemory;
            },
            1, 2, 3, 4 => {
                var collector: RecordingCollector(CollideShapeBodyCollector) = .{ .mode = q.collector_mode, .max_hits = q.max_hits, .hits = hits, .allocator = self.allocator };
                switch (q.kind) {
                    1 => query_interface.collideAABox(&AABox.init(loadVec3(f[0..3]), loadVec3(f[3..6])), &collector.base, filters),
                    2 => query_interface.collideSphere(loadVec3(f[0..3]), f[3], &collector.base, filters),
                    3 => query_interface.collidePoint(loadVec3(f[0..3]), &collector.base, filters),
                    else => query_interface.collideOrientedBox(&OrientedBox.init(Mat44.rotationTranslation(Quat.init(f[0], f[1], f[2], f[3]), loadVec3(f[4..7])), loadVec3(f[7..10])), &collector.base, filters),
                }
                if (collector.failed) return error.OutOfMemory;
            },
            else => {
                var collector: RecordingCollector(CastShapeBodyCollector) = .{ .mode = q.collector_mode, .max_hits = q.max_hits, .hits = hits, .allocator = self.allocator };
                const cast: AABoxCast = .{ .box = .init(loadVec3(f[0..3]), loadVec3(f[3..6])), .direction = loadVec3(f[6..9]) };
                if (q.kind == 5)
                    query_interface.castAABox(&cast, &collector.base, filters)
                else
                    try self.bp().castAABoxNoLock(&cast, &collector.base, &bp_filter.base, &ol_filter.base);
                if (collector.failed) return error.OutOfMemory;
            },
        }
    }

    fn findCollidingPairs(self: *ZoltWorld, raw: []const u32, distance: f32, out_order: []u32, pairs: *std.ArrayList(u32)) !void {
        const body_ids = try self.ids(raw);
        pairs.clearRetainingCapacity();
        var collector: PairCollector = .{ .pairs = pairs, .allocator = self.allocator };
        try self.bp().findCollidingPairs(body_ids, distance, &self.object_vs_broad_phase_filter.base, &self.pair_filter.base, &collector.base);
        if (collector.failed) return error.OutOfMemory;
        fromIDs(out_order, body_ids);
    }

    /// The nodes of the tree with root node `root_index` in walk order (see BQDumpTree in the C++ file)
    fn dumpTree(self: *ZoltWorld, tree: *const QuadTree, root_index: u32, out: *std.ArrayList(u32)) !void {
        var stack: std.ArrayList(u32) = .empty;
        defer stack.deinit(self.allocator);
        try stack.append(self.allocator, root_index);
        while (stack.pop()) |node_idx| {
            const node = tree.node_allocator.?.get(node_idx);
            try out.appendSlice(self.allocator, &.{ node_idx, node.parent_node_index.load(.monotonic), node.is_changed.load(.monotonic) });
            for (0..4) |i| {
                const child = node.child_node_id[i].id.load(.monotonic);
                try out.append(self.allocator, child);
                for ([_]*const [4]std.atomic.Value(f32){ &node.bounds_min_x, &node.bounds_min_y, &node.bounds_min_z, &node.bounds_max_x, &node.bounds_max_y, &node.bounds_max_z }) |b|
                    try out.append(self.allocator, @bitCast(b[i].load(.monotonic)));
                if (child != invalid_index and (child & BodyID.broad_phase_bit) != 0)
                    try stack.append(self.allocator, child & ~BodyID.broad_phase_bit);
            }
        }
    }

    /// The state of the broad phase (see jolt_bpqt_dump)
    fn dump(self: *ZoltWorld, out: *std.ArrayList(u32)) !void {
        const qt = self.quad_tree;
        out.clearRetainingCapacity();
        try out.appendSlice(self.allocator, &.{ qt.next_layer_to_update, qt.query_lock_idx.load(.monotonic), qt.num_layers });
        for (qt.layers) |*tree| {
            const ts = &tree.tree_state;
            try out.appendSlice(self.allocator, &.{ ts.num_bodies.load(.monotonic), @intFromBool(ts.is_dirty.load(.monotonic)), ts.root_node_index.load(.monotonic), ts.root_node[0].index.load(.monotonic), ts.root_node[1].index.load(.monotonic), tree.free_node_batch.num_objects, tree.free_node_batch.first_object_index, tree.free_node_batch.last_object_index });
            for (&ts.root_node) |*root| {
                const root_idx = root.index.load(.monotonic);
                if (root_idx != invalid_index)
                    try self.dumpTree(tree, root_idx, out);
            }
        }

        // The new tree of a pending update
        if (self.pending) {
            const impl: *const BroadPhaseQuadTree.UpdateStateImpl = @ptrCast(&self.pending_state);
            if (impl.tree) |tree|
                try self.dumpTree(tree, impl.update_state.root_node_id.id & ~BodyID.broad_phase_bit, out);
        }

        // The tracking
        for (qt.tracking) |*t|
            try out.appendSlice(self.allocator, &.{ t.broad_phase_layer.load(.monotonic), t.object_layer.load(.monotonic), t.body_location.load(.monotonic) });
    }
};

// -------------------------------------------------------------------------------------------------------------------
// The driver

const Scene = struct {
    name: []const u8,
    seed: u32,
    num_bodies: u32,
    num_ops: u32,
    num_broad_phase_layers: u32,
    distribution: enum { uniform, coincident, collinear, clustered, exponential, large },
    extent: f32,
    /// Relative weight of single adds (TryInsertLeaf / TryCreateNewRoot chains)
    single_add_weight: u32 = 2,
    /// Relative weight of the operations that rebuild trees (updates and Optimize)
    rebuild_weight: u32 = 4,
};

const scenes = [_]Scene{
    .{ .name = "uniform, 1 layer", .seed = 1, .num_bodies = 300, .num_ops = 1500, .num_broad_phase_layers = 1, .distribution = .uniform, .extent = 50 },
    .{ .name = "uniform, 4 layers", .seed = 2, .num_bodies = 400, .num_ops = 1500, .num_broad_phase_layers = 4, .distribution = .uniform, .extent = 100 },
    .{ .name = "uniform, 6 layers, single adds", .seed = 3, .num_bodies = 300, .num_ops = 1500, .num_broad_phase_layers = 6, .distribution = .uniform, .extent = 20, .single_add_weight = 8 },
    .{ .name = "coincident", .seed = 4, .num_bodies = 120, .num_ops = 800, .num_broad_phase_layers = 2, .distribution = .coincident, .extent = 1 },
    .{ .name = "collinear", .seed = 5, .num_bodies = 200, .num_ops = 800, .num_broad_phase_layers = 3, .distribution = .collinear, .extent = 100 },
    .{ .name = "clustered", .seed = 6, .num_bodies = 300, .num_ops = 1000, .num_broad_phase_layers = 2, .distribution = .clustered, .extent = 200 },
    .{ .name = "exponential", .seed = 7, .num_bodies = 40, .num_ops = 600, .num_broad_phase_layers = 1, .distribution = .exponential, .extent = 1 },
    .{ .name = "large coordinates", .seed = 8, .num_bodies = 200, .num_ops = 800, .num_broad_phase_layers = 2, .distribution = .large, .extent = 1.0e5 },
    .{ .name = "few bodies", .seed = 9, .num_bodies = 6, .num_ops = 600, .num_broad_phase_layers = 2, .distribution = .uniform, .extent = 5 },
    // Bodies added one by one and hardly ever rebuilt: deep chains of roots, the walk stacks grow on the heap (the rays
    // along the line keep 3 leaves per level on the stack)
    .{ .name = "deep chains", .seed = 10, .num_bodies = 400, .num_ops = 500, .num_broad_phase_layers = 1, .distribution = .collinear, .extent = 200, .single_add_weight = 30, .rebuild_weight = 0 },
};

const OpKind = enum { add, add_abort, add_single, remove, move, set_layers, update, update_prepare, update_finalize, frame_sync, optimize, query, find_pairs, get_bounds };

const Driver = struct {
    allocator: Allocator,
    scene: *const Scene,
    rng: fw.Rng,
    cpp: *anyopaque,
    zolt_world: *ZoltWorld,
    ids: []u32,
    is_dynamic: []bool,
    counts: [@typeInfo(OpKind).@"enum".fields.len]u32 = @splat(0),
    num_hits: u64 = 0,
    num_pairs: u64 = 0,
    max_depth: u32 = 0,

    // Buffers
    zolt_dump: std.ArrayList(u32) = .empty,
    cpp_dump: std.ArrayList(u32) = .empty,
    raw: std.ArrayList(u32) = .empty,
    zolt_order: std.ArrayList(u32) = .empty,
    cpp_order: std.ArrayList(u32) = .empty,
    floats: std.ArrayList(f32) = .empty,
    extra: std.ArrayList(u32) = .empty,
    zolt_hits: std.ArrayList(Hit) = .empty,
    cpp_hits: std.ArrayList(Hit) = .empty,
    zolt_pairs: std.ArrayList(u32) = .empty,
    cpp_pairs: std.ArrayList(u32) = .empty,

    fn deinit(self: *Driver) void {
        inline for (.{ &self.zolt_dump, &self.cpp_dump, &self.raw, &self.zolt_order, &self.cpp_order, &self.extra, &self.zolt_pairs, &self.cpp_pairs }) |list| list.deinit(self.allocator);
        self.floats.deinit(self.allocator);
        self.zolt_hits.deinit(self.allocator);
        self.cpp_hits.deinit(self.allocator);
    }

    fn float(self: *Driver, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn chance(self: *Driver, percent: u32) bool {
        return self.rng.intRange(u32, 0, 99) < percent;
    }

    fn randomRotation(self: *Driver) [4]f32 {
        if (self.chance(30)) return .{ 0, 0, 0, 1 };
        const q = Quat.init(self.float(-1, 1), self.float(-1, 1), self.float(-1, 1), self.float(-1, 1)).normalized();
        return .{ q.getX(), q.getY(), q.getZ(), q.getW() };
    }

    fn randomPosition(self: *Driver, index: u32) [3]f32 {
        const e = self.scene.extent;
        return switch (self.scene.distribution) {
            .uniform, .large => .{ self.float(-e, e), self.float(-e, e), self.float(-e, e) },
            .coincident => if (self.chance(80)) .{ 1, 2, 3 } else .{ 1, 2, 3 + self.float(0, e) },
            .collinear => .{ self.float(-e, e), 0.5, -0.25 },
            .clustered => blk: {
                const cluster: f32 = @floatFromInt(self.rng.intRange(u32, 0, 3));
                break :blk .{ cluster * e + self.float(-1, 1), self.float(-1, 1), -cluster * e + self.float(-1, 1) };
            },
            // Every body further out than all before: partitions that split off one body, deep trees
            .exponential => .{ -std.math.pow(f32, 2.0, @floatFromInt(index % 30)) - self.float(0, 0.1), self.float(0, 0.01), 0 },
        };
    }

    fn sizeFor(self: *Driver) [3]f32 {
        const scale: f32 = switch (self.scene.distribution) {
            .large => 50,
            .exponential => 0.01,
            else => 1,
        };
        return .{ scale * self.float(0.01, 2), scale * self.float(0.01, 2), scale * self.float(0.01, 2) };
    }

    fn compareSlices(self: *Driver, comptime what: []const u8, zolt_values: []const u32, cpp_values: []const u32, op: usize) !void {
        _ = self;
        if (std.mem.eql(u32, zolt_values, cpp_values)) return;
        const n = @min(zolt_values.len, cpp_values.len);
        var first: usize = n;
        for (0..n) |i| {
            if (zolt_values[i] != cpp_values[i]) {
                first = i;
                break;
            }
        }
        std.debug.print(what ++ " differs after operation {d}: length zolt {d} jolt {d}, first difference at {d}\n", .{ op, zolt_values.len, cpp_values.len, first });
        const start = first -| 8;
        std.debug.print("  zolt: {any}\n  jolt: {any}\n", .{ zolt_values[start..@min(first + 8, zolt_values.len)], cpp_values[start..@min(first + 8, cpp_values.len)] });
        return error.ParityMismatch;
    }

    fn compareDumps(self: *Driver, op: usize) !void {
        try self.zolt_world.dump(&self.zolt_dump);
        while (true) {
            const size = jolt.jolt_bpqt_dump(self.cpp, self.cpp_dump.items.ptr, @intCast(self.cpp_dump.items.len));
            if (size <= self.cpp_dump.items.len) {
                self.cpp_dump.items.len = size;
                break;
            }
            try self.cpp_dump.resize(self.allocator, size);
        }
        try self.compareSlices("dump", self.zolt_dump.items, self.cpp_dump.items, op);
        self.cpp_dump.items.len = self.cpp_dump.capacity;
    }

    /// Picks up to `max` distinct bodies with the given broad phase state into self.raw
    fn pick(self: *Driver, in_broad_phase: bool, max: u32, only_active: bool) !void {
        self.raw.clearRetainingCapacity();
        const n = self.rng.intRange(u32, 1, max);
        const start = self.rng.intRange(u32, 0, @intCast(self.ids.len - 1));
        const step = self.rng.intRange(u32, 1, 7);
        for (0..self.ids.len) |i| {
            const index = (start + i * step) % self.ids.len;
            const body = self.zolt_world.body_manager.getBody(.init(self.ids[index]));
            if (body.isInBroadPhase() != in_broad_phase) continue;
            if (only_active and !body.isActive()) continue;
            if (std.mem.indexOfScalar(u32, self.raw.items, self.ids[index]) != null) continue;
            try self.raw.append(self.allocator, self.ids[index]);
            if (self.raw.items.len == n) break;
        }
    }

    fn prepareOrders(self: *Driver) !void {
        try self.zolt_order.resize(self.allocator, self.raw.items.len);
        try self.cpp_order.resize(self.allocator, self.raw.items.len);
    }

    fn randomQuery(self: *Driver) Query {
        const e = self.scene.extent;
        var q: Query = .{
            .kind = self.rng.intRange(u32, 0, 6),
            .collector_mode = self.rng.intRange(u32, 0, 2),
            .max_hits = self.rng.intRange(u32, 1, 5),
            .broad_phase_layer_mask = if (self.chance(70)) 0xffffffff else self.rng.next(),
            .object_layer_mask = if (self.chance(70)) 0xffffffff else self.rng.next(),
        };
        // Around a body (so that the queries hit something) or anywhere
        var center: [3]f32 = .{ self.float(-e, e), self.float(-e, e), self.float(-e, e) };
        if (self.chance(60)) {
            const body = self.zolt_world.body_manager.getBody(.init(self.ids[self.rng.intRange(u32, 0, @intCast(self.ids.len - 1))]));
            const c = body.getWorldSpaceBounds().getCenter();
            center = .{ c.getX(), c.getY(), c.getZ() };
        }
        const half = [3]f32{ self.float(0, e * 0.3), self.float(0, e * 0.3), self.float(0, e * 0.3) };
        const dir = [3]f32{ self.float(-2 * e, 2 * e), self.float(-2 * e, 2 * e), self.float(-2 * e, 2 * e) };
        switch (q.kind) {
            0 => q.f[0..6].* = .{ center[0] - dir[0] * 0.5, center[1] - dir[1] * 0.5, center[2] - dir[2] * 0.5, dir[0], dir[1], dir[2] },
            1 => q.f[0..6].* = .{ center[0] - half[0], center[1] - half[1], center[2] - half[2], center[0] + half[0], center[1] + half[1], center[2] + half[2] },
            2 => q.f[0..4].* = .{ center[0], center[1], center[2], half[0] },
            3 => q.f[0..3].* = center,
            4 => {
                const r = self.randomRotation();
                q.f[0..10].* = .{ r[0], r[1], r[2], r[3], center[0], center[1], center[2], half[0], half[1], half[2] };
            },
            else => q.f[0..9].* = .{ center[0] - dir[0] * 0.5 - half[0] * 0.1, center[1] - dir[1] * 0.5 - half[1] * 0.1, center[2] - dir[2] * 0.5 - half[2] * 0.1, center[0] - dir[0] * 0.5 + half[0] * 0.1, center[1] - dir[1] * 0.5 + half[1] * 0.1, center[2] - dir[2] * 0.5 + half[2] * 0.1, dir[0], dir[1], dir[2] },
        }
        // A ray along the line of the collinear scenes (through every body)
        if (self.scene.distribution == .collinear and q.kind == 0 and self.chance(30))
            q.f[0..6].* = .{ -2 * e, 0.5, -0.25, 4 * e, 0, 0 };

        // Edge cases: a zero length ray / cast, a ray along an axis
        if (self.chance(5)) {
            if (q.kind == 0) q.f[3..6].* = .{ 0, 0, 0 };
            if (q.kind >= 5) q.f[6..9].* = .{ 0, 0, 0 };
        }
        if (self.chance(5) and q.kind == 0) q.f[4..6].* = .{ 0, 0 };
        return q;
    }

    fn runQuery(self: *Driver, op: usize) !void {
        const q = self.randomQuery();
        try self.zolt_world.query(&q, &self.zolt_hits);
        try self.cpp_hits.resize(self.allocator, @max(self.ids.len, 1));
        const n = jolt.jolt_bpqt_query(self.cpp, &q, self.cpp_hits.items.ptr, @intCast(self.cpp_hits.items.len));
        if (n > self.cpp_hits.items.len) return error.TooManyHits;
        self.cpp_hits.items.len = n;
        self.num_hits += n;
        try self.compareSlices("query hits", std.mem.bytesAsSlice(u32, std.mem.sliceAsBytes(self.zolt_hits.items)), std.mem.bytesAsSlice(u32, std.mem.sliceAsBytes(self.cpp_hits.items)), op);
    }

    fn runOp(self: *Driver, op: usize) !void {
        const pending = self.zolt_world.pending;
        const kind: OpKind = if (pending) switch (self.rng.intRange(u32, 0, 9)) {
            0, 1, 2, 3 => .query,
            4, 5 => .find_pairs,
            6, 7 => .add_abort,
            else => .update_finalize,
        } else blk: {
            const rebuild = self.scene.rebuild_weight;
            const weights = [_]u32{ 6, 3, self.scene.single_add_weight, 5, 8, 2, rebuild, (rebuild * 3 + 3) / 4, 0, 4, @intFromBool(rebuild > 0), 12, 6, 1 };
            var total: u32 = 0;
            for (weights) |w| total += w;
            var r = self.rng.intRange(u32, 0, total - 1);
            for (weights, 0..) |w, i| {
                if (r < w) break :blk @enumFromInt(i);
                r -= w;
            }
            unreachable;
        };
        self.counts[@intFromEnum(kind)] += 1;

        switch (kind) {
            .add, .add_abort, .add_single => {
                try self.pick(false, if (kind == .add_single) 4 else 64, false);
                if (self.raw.items.len == 0) return;
                try self.prepareOrders();
                if (kind == .add_single) {
                    // One body at a time
                    for (self.raw.items, 0..) |id, i| {
                        try self.zolt_world.add(&.{id}, false, true, self.zolt_order.items[i .. i + 1]);
                        jolt.jolt_bpqt_add(self.cpp, &.{id}, 1, 0, 1, self.cpp_order.items[i .. i + 1].ptr);
                    }
                } else {
                    const abort = kind == .add_abort;
                    try self.zolt_world.add(self.raw.items, abort, true, self.zolt_order.items);
                    jolt.jolt_bpqt_add(self.cpp, self.raw.items.ptr, @intCast(self.raw.items.len), @intFromBool(abort), 1, self.cpp_order.items.ptr);
                }
                try self.compareSlices("add order", self.zolt_order.items, self.cpp_order.items, op);
            },
            .remove => {
                try self.pick(true, 40, false);
                if (self.raw.items.len == 0) return;
                try self.prepareOrders();
                try self.zolt_world.remove(self.raw.items, self.zolt_order.items);
                jolt.jolt_bpqt_remove(self.cpp, self.raw.items.ptr, @intCast(self.raw.items.len), self.cpp_order.items.ptr);
                try self.compareSlices("remove order", self.zolt_order.items, self.cpp_order.items, op);
            },
            .move => {
                try self.pick(true, 40, false);
                if (self.raw.items.len == 0) return;
                try self.prepareOrders();
                self.floats.clearRetainingCapacity();
                for (self.raw.items) |id| {
                    const body = self.zolt_world.body_manager.getBody(.init(id));
                    const p = body.getPosition();
                    const step = 0.1 * self.scene.extent;
                    var position: [3]f32 = .{ @floatCast(p.getX()), @floatCast(p.getY()), @floatCast(p.getZ()) };
                    if (self.chance(10)) {
                        position = self.randomPosition(id); // A teleport
                    } else if (self.scene.distribution != .exponential and self.scene.distribution != .coincident) {
                        for (&position) |*c| c.* += self.float(-step, step);
                    }
                    const rotation = if (self.chance(50)) self.randomRotation() else [4]f32{ body.getRotation().getX(), body.getRotation().getY(), body.getRotation().getZ(), body.getRotation().getW() };
                    try self.floats.appendSlice(self.allocator, &position);
                    try self.floats.appendSlice(self.allocator, &rotation);
                }
                const take_lock = self.chance(50);
                try self.zolt_world.move(self.raw.items, self.floats.items, take_lock, self.zolt_order.items);
                jolt.jolt_bpqt_move(self.cpp, self.raw.items.ptr, @intCast(self.raw.items.len), self.floats.items.ptr, @intFromBool(take_lock), self.cpp_order.items.ptr);
                try self.compareSlices("move order", self.zolt_order.items, self.cpp_order.items, op);
            },
            .set_layers => {
                try self.pick(true, 10, false);
                if (self.raw.items.len == 0) return;
                try self.prepareOrders();
                self.extra.clearRetainingCapacity();
                for (self.raw.items) |_| try self.extra.append(self.allocator, self.rng.intRange(u32, 0, num_object_layers - 1));
                try self.zolt_world.setLayers(self.raw.items, self.extra.items, self.zolt_order.items);
                jolt.jolt_bpqt_set_layers(self.cpp, self.raw.items.ptr, @intCast(self.raw.items.len), self.extra.items.ptr, self.cpp_order.items.ptr);
                try self.compareSlices("layer change order", self.zolt_order.items, self.cpp_order.items, op);
            },
            .update, .update_prepare => {
                const zolt_state = try self.zolt_world.updatePrepare();
                var cpp_state: [2]u32 = undefined;
                jolt.jolt_bpqt_update_prepare(self.cpp, &cpp_state);
                try self.compareSlices("update state", &zolt_state, &cpp_state, op);
                if (kind == .update) {
                    self.zolt_world.updateFinalize();
                    jolt.jolt_bpqt_update_finalize(self.cpp);
                } // Otherwise the update stays pending: the next operations run on the old tree
            },
            .update_finalize => {
                self.zolt_world.updateFinalize();
                jolt.jolt_bpqt_update_finalize(self.cpp);
            },
            .frame_sync => {
                self.zolt_world.bp().frameSync();
                jolt.jolt_bpqt_frame_sync(self.cpp);
            },
            .optimize => {
                try self.zolt_world.bp().optimize();
                jolt.jolt_bpqt_optimize(self.cpp);
            },
            .query => for (0..4) |_| try self.runQuery(op),
            .find_pairs => {
                try self.pick(true, 50, true);
                if (self.raw.items.len == 0) return;
                try self.prepareOrders();
                const distance = if (self.chance(50)) 0.0 else self.float(0, 0.05 * self.scene.extent);
                try self.zolt_world.findCollidingPairs(self.raw.items, distance, self.zolt_order.items, &self.zolt_pairs);
                try self.cpp_pairs.resize(self.allocator, 2 * self.ids.len * self.raw.items.len);
                const n = jolt.jolt_bpqt_find_colliding_pairs(self.cpp, self.raw.items.ptr, @intCast(self.raw.items.len), distance, self.cpp_order.items.ptr, self.cpp_pairs.items.ptr, @intCast(self.cpp_pairs.items.len / 2));
                self.cpp_pairs.items.len = 2 * n;
                self.num_pairs += n;
                try self.compareSlices("pairs", self.zolt_pairs.items, self.cpp_pairs.items, op);
                try self.compareSlices("active body order", self.zolt_order.items, self.cpp_order.items, op);
            },
            .get_bounds => {
                const zolt_bounds = self.zolt_world.quad_tree.base.asQuery().getBounds();
                const zolt_values: [6]f32 = .{ zolt_bounds.min.getX(), zolt_bounds.min.getY(), zolt_bounds.min.getZ(), zolt_bounds.max.getX(), zolt_bounds.max.getY(), zolt_bounds.max.getZ() };
                var cpp_values: [6]f32 = undefined;
                jolt.jolt_bpqt_get_bounds(self.cpp, &cpp_values);
                try self.compareSlices("bounds", std.mem.bytesAsSlice(u32, std.mem.asBytes(&zolt_values)), std.mem.bytesAsSlice(u32, std.mem.asBytes(&cpp_values)), op);
            },
        }
    }
};

/// Runs a scene, returns the number of pairs that FindCollidingPairs found
fn runScene(allocator: Allocator, scene: *const Scene) !u64 {
    var rng: fw.Rng = .{ .state = 0x9e3779b9 ^ scene.seed };

    // The layer filters: mostly colliding
    var desc: WorldDesc = .{
        .max_bodies = 4 * scene.num_bodies + 16,
        .num_broad_phase_layers = scene.num_broad_phase_layers,
        .object_vs_broad_phase_mask = undefined,
        .object_pair_mask = undefined,
    };
    for (&desc.object_vs_broad_phase_mask, &desc.object_pair_mask) |*a, *b| {
        a.* = rng.next() | rng.next();
        b.* = rng.next() | rng.next() | rng.next();
    }

    const cpp = jolt.jolt_bpqt_create(&desc);
    defer jolt.jolt_bpqt_destroy(cpp);
    const zolt_world = try allocator.create(ZoltWorld);
    defer allocator.destroy(zolt_world);
    try zolt_world.init(allocator, &desc);
    defer zolt_world.deinit();

    const ids = try allocator.alloc(u32, scene.num_bodies);
    defer allocator.free(ids);
    const is_dynamic = try allocator.alloc(bool, scene.num_bodies);
    defer allocator.free(is_dynamic);

    var driver: Driver = .{ .allocator = allocator, .scene = scene, .rng = rng, .cpp = cpp, .zolt_world = zolt_world, .ids = ids, .is_dynamic = is_dynamic };
    defer driver.deinit();
    try driver.cpp_dump.resize(allocator, 1024);

    // The bodies
    for (ids, is_dynamic, 0..) |*id, *dynamic, i| {
        dynamic.* = driver.chance(60);
        const body_desc: BodyDesc = .{
            .position = driver.randomPosition(@intCast(i)),
            .rotation = driver.randomRotation(),
            .size = driver.sizeFor(),
            .shape = if (dynamic.* or driver.chance(50)) 1 else 0,
            .motion_type = if (dynamic.*) @intFromEnum(MotionType.dynamic) else @intFromEnum(MotionType.static),
            .object_layer = driver.rng.intRange(u32, 0, num_object_layers - 1),
        };
        const zolt_id = try zolt_world.createBody(&body_desc);
        const cpp_id = jolt.jolt_bpqt_create_body(cpp, &body_desc);
        if (zolt_id != cpp_id) return error.ParityMismatch;
        id.* = zolt_id;
    }
    try driver.compareDumps(0);

    for (0..scene.num_ops) |op| {
        try driver.runOp(op + 1);
        try driver.compareDumps(op + 1);
        for (zolt_world.quad_tree.layers) |*tree| {
            const ts = &tree.tree_state;
            driver.max_depth = @max(driver.max_depth, tree.getMaxTreeDepth(ts.root_node[ts.root_node_index.load(.monotonic)].getNodeID()));
        }
    }

    // Finish a pending update, then everything is compared once more after an optimize
    if (zolt_world.pending) {
        zolt_world.updateFinalize();
        jolt.jolt_bpqt_update_finalize(cpp);
    }
    try zolt_world.bp().optimize();
    jolt.jolt_bpqt_optimize(cpp);
    try driver.compareDumps(scene.num_ops + 1);
    for (0..50) |_| try driver.runQuery(scene.num_ops + 1);

    // Every operation ran, the queries found something
    for (driver.counts, 0..) |n, i| {
        const kind: OpKind = @enumFromInt(i);
        const rebuilds = kind == .update or kind == .update_prepare or kind == .update_finalize or kind == .optimize;
        if (n == 0 and !(rebuilds and scene.rebuild_weight == 0) and kind != .update_finalize) {
            std.debug.print("{s}: operation {t} never ran\n", .{ scene.name, @as(OpKind, @enumFromInt(i)) });
            return error.CoverageMissing;
        }
    }
    if (driver.num_hits == 0) return error.CoverageMissing;
    if (scene.rebuild_weight == 0 and driver.max_depth < 50) return error.CoverageMissing; // A deep tree
    return driver.num_pairs;
}

test "BroadPhaseQuadTree parity: random operation sequences" {
    var num_pairs: u64 = 0;
    for (&scenes) |*scene| {
        num_pairs += runScene(std.testing.allocator, scene) catch |err| {
            std.debug.print("scene \"{s}\" failed: {t}\n", .{ scene.name, err });
            return err;
        };
    }
    try std.testing.expect(num_pairs > 0);
}
