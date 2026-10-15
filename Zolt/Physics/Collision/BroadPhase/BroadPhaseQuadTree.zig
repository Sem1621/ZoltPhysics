//! Port of: Jolt/Physics/Collision/BroadPhase/BroadPhaseQuadTree.h, Jolt/Physics/Collision/BroadPhase/BroadPhaseQuadTree.cpp
//! Status: complete
//! Not ported: JPH_TRACK_BROADPHASE_STATS (ReportStats, off in Jolt's default build, like BroadPhase.zig), JPH_EXTERNAL_PROFILE / JPH_PROFILE_ENABLED (naming the trees after their layers)
//!
//! SimulationArchitecture D9: the `final` concrete class of BroadPhaseQuery -> BroadPhase, one QuadTree per broad
//! phase layer, the default broad phase of PhysicsSystem (Jolt's BROAD_PHASE define). It is created on the heap like
//! Jolt's `new BroadPhaseQuadTree` (`create(allocator)`, the allocator is also the heap of the trees) and freed through
//! the virtual destructor (`broad_phase.base.destroy()`). It must not move after initBroadPhase: the trees point to its
//! node allocator.
//! - `mutable SharedMutex mQueryLocks[2]` live in the heap block `mutable_fields` (Rule M), so the queries keep
//!   `*const` receivers. Like Jolt, the queries lock them directly (`shared_lock`), FrameSync with PhysicsLock's lock
//!   order bookkeeping. `mLockContext` (asserts only) is the body manager of `BroadPhase.body_manager`; `io` is the
//!   body manager's.
//! - initBroadPhase commits every node page (FixedSizeFreeList.reserveAllPages, D9): about 170 bytes per max body
//!   (2 * (n / 2 + n / 6) nodes of 128 bytes, the node estimate of Jolt's Init, which Jolt itself uses once the tree is
//!   full: the allocator holds twice the estimated node count for the rebuild in Update), plus 8 bytes of tracking per
//!   max body. AddBodiesFinalize / AddBodiesAbort / UpdateFinalize then never allocate; running out of nodes is Jolt's
//!   abort (`@panic("QuadTree: Out of nodes!")`).
//! - Errors (D9): `addBodiesPrepare` (`new LayerState[]` and BuildTree's `new Vec3[]`) and `updatePrepare`
//!   (`new NodeID[]`, BuildTree, the node stack) return Allocator.Error and leave the broad phase unchanged: a failing
//!   AddBodiesPrepare aborts the layers it prepared (ioBodies stays sorted), a failing UpdatePrepare restores
//!   `next_layer_to_update` and the tree. A zeroed UpdateState is a no-op in updateFinalize. Four Jolt functions that
//!   allocate return Allocator.Error too (BroadPhase.zig): `optimize` (UpdatePrepare with a full rebuild: the layers
//!   before the failing one are optimized, the others unchanged, the broad phase stays valid), `notifyBodiesLayerChanged`
//!   (it allocates what AddBodiesPrepare needs before it changes anything, so nothing changed on error),
//!   `findCollidingPairs` (its node stack can grow on the heap, the FindCollisions job latches the error, D5) and
//!   `castAABoxNoLock` (the same, inside the FindCCDContacts job). The locking BroadPhaseQuery queries have no error
//!   channel (CollisionArchitecture rule 4): their node stack growing on the heap is the one heap allocation of a
//!   query (from the broad phase's heap, only in a tree deeper than about 40 levels) and a failure there panics, where
//!   Jolt's allocator aborts (see QuadTree.zig).
//! - `LayerState`'s `mBodyStart` / `mBodyEnd` are a slice of ioBodies (null: the layer has no bodies). `UpdateStateImpl`
//!   is an extern struct that is reinterpreted from BroadPhase.UpdateState, like Jolt's reinterpret_cast.
//! - `std::upper_bound` on the sorted bodies is `upperBound` (libstdc++'s binary search, the result is unique).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../../Core/Core.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const OrientedBox = @import("../../../Geometry/OrientedBox.zig").OrientedBox;
const SharedMutex = @import("../../../Core/Mutex.zig").SharedMutex;
const quickSort = @import("../../../Core/QuickSort.zig").quickSort;
const BodyID = @import("../../Body/BodyID.zig").BodyID;
const BodyManagerFile = @import("../../Body/BodyManager.zig");
const BodyManager = BodyManagerFile.BodyManager;
const BodySlot = BodyManager.BodySlot;
const PhysicsLockFile = @import("../../PhysicsLock.zig");
const PhysicsLock = PhysicsLockFile.PhysicsLock;
const UniqueLock = PhysicsLockFile.UniqueLock;
const SharedLock = PhysicsLockFile.SharedLock;
const RayCast = @import("../RayCast.zig").RayCast;
const AABoxCast = @import("../AABoxCast.zig").AABoxCast;
const ObjectLayerFile = @import("../ObjectLayer.zig");
const ObjectLayer = ObjectLayerFile.ObjectLayer;
const object_layer_invalid = ObjectLayerFile.object_layer_invalid;
const ObjectLayerFilter = ObjectLayerFile.ObjectLayerFilter;
const ObjectLayerPairFilter = ObjectLayerFile.ObjectLayerPairFilter;
const BroadPhaseLayerFile = @import("BroadPhaseLayer.zig");
const BroadPhaseLayer = BroadPhaseLayerFile.BroadPhaseLayer;
const broad_phase_layer_invalid = BroadPhaseLayerFile.broad_phase_layer_invalid;
const BroadPhaseLayerInterface = BroadPhaseLayerFile.BroadPhaseLayerInterface;
const BroadPhaseLayerFilter = BroadPhaseLayerFile.BroadPhaseLayerFilter;
const ObjectVsBroadPhaseLayerFilter = BroadPhaseLayerFile.ObjectVsBroadPhaseLayerFilter;
const BroadPhaseQueryFile = @import("BroadPhaseQuery.zig");
const RayCastBodyCollector = BroadPhaseQueryFile.RayCastBodyCollector;
const CastShapeBodyCollector = BroadPhaseQueryFile.CastShapeBodyCollector;
const CollideShapeBodyCollector = BroadPhaseQueryFile.CollideShapeBodyCollector;
const BroadPhaseFile = @import("BroadPhase.zig");
const BroadPhase = BroadPhaseFile.BroadPhase;
const BodyPairCollector = BroadPhaseFile.BodyPairCollector;
const QuadTree = @import("QuadTree.zig").QuadTree;

const Tracking = QuadTree.Tracking;

/// Fast SIMD based quad tree BroadPhase that is multithreading aware and tries to do a minimal amount of locking.
pub const BroadPhaseQuadTree = struct {
    // Implementing interface of BroadPhase (see BroadPhase for documentation)
    pub const overrides = .{ .initBroadPhase, .optimize, .frameSync, .lockModifications, .updatePrepare, .updateFinalize, .unlockModifications, .addBodiesPrepare, .addBodiesFinalize, .addBodiesAbort, .removeBodies, .notifyBodiesAABBChanged, .notifyBodiesLayerChanged, .castRay, .collideAABox, .collideSphere, .collidePoint, .collideOrientedBox, .castAABoxNoLock, .castAABox, .findCollidingPairs, .getBounds };

    /// Helper struct for AddBodies handle
    pub const LayerState = struct {
        /// mBodyStart / mBodyEnd: the bodies of this layer in ioBodies (null: none)
        bodies: ?[]BodyID = null,
        add_state: QuadTree.AddState = .{},
    };

    /// UpdateState implementation for this tree used during UpdatePrepare/Finalize()
    pub const UpdateStateImpl = extern struct {
        tree: ?*QuadTree,
        update_state: QuadTree.UpdateState,
    };

    comptime {
        std.debug.assert(@sizeOf(UpdateStateImpl) <= @sizeOf(BroadPhase.UpdateState));
        std.debug.assert(@alignOf(UpdateStateImpl) <= @alignOf(BroadPhase.UpdateState));
    }

    /// The C++ `mutable` members, locked by the const queries (Rule M: behind a pointer, never through @constCast)
    pub const MutableFields = struct {
        /// We double buffer all trees so that we can query while building the next one and we destroy the old tree the next physics update.
        /// This structure ensures that we wait for queries that are still using the old tree.
        query_locks: [2]SharedMutex = .{ .{}, .{} },

        /// Zolt: the latch of QuadTree's performance warning, shared by the trees (Jolt: a function `static`, only with
        /// asserts), so that nothing mutable is global
        triggered_report: if (Core.enable_asserts) std.atomic.Value(bool) else void = if (Core.enable_asserts) .init(false) else {},
    };

    base: BroadPhase,

    /// Max amount of bodies we support
    max_bodies: usize = 0,

    /// Array that for each BodyID keeps track of where it is located in which tree
    tracking: []Tracking = &.{},

    /// Node allocator for all trees (mAllocator)
    node_allocator: QuadTree.NodeAllocator = .empty,

    /// Information about broad phase layers
    broad_phase_layer_interface: ?*const BroadPhaseLayerInterface = null,

    /// One tree per object layer
    layers: []QuadTree = &.{},
    num_layers: u32 = 0,

    /// Mutex that prevents object modification during UpdatePrepare/Finalize()
    update_mutex: SharedMutex = .{},

    /// mQueryLocks (mutable)
    mutable_fields: *MutableFields,

    /// This index indicates which lock is currently active, it alternates between 0 and 1
    query_lock_idx: std.atomic.Value(u32) = .init(0),

    /// This is the next tree to update in UpdatePrepare()
    next_layer_to_update: u32 = 0,

    /// new BroadPhaseQuadTree (`allocator` frees it and is the heap of the trees)
    pub fn create(allocator: Allocator) Allocator.Error!*BroadPhaseQuadTree {
        const mutable_fields = try allocator.create(MutableFields);
        errdefer allocator.destroy(mutable_fields);
        mutable_fields.* = .{};
        const self = try allocator.create(BroadPhaseQuadTree);
        self.* = .{ .base = .init(BroadPhaseQuadTree, allocator), .mutable_fields = mutable_fields };
        return self;
    }

    /// ~BroadPhaseQuadTree (run by the generated destructor chain)
    pub fn destruct(self: *BroadPhaseQuadTree) void {
        const allocator = self.base.base.allocator;

        // delete [] mLayers (the trees are destroyed in reverse order)
        var l = self.layers.len;
        while (l > 0) {
            l -= 1;
            self.layers[l].deinit();
        }
        allocator.free(self.layers);

        // The members
        self.node_allocator.deinit();
        allocator.free(self.tracking);
        allocator.destroy(self.mutable_fields);
    }

    /// The BroadPhase base class
    pub fn asBroadPhase(self: *BroadPhaseQuadTree) *BroadPhase {
        return &self.base;
    }

    fn bodyManager(self: *const BroadPhaseQuadTree) *BodyManager {
        return self.base.getBodyManager();
    }

    fn io(self: *const BroadPhaseQuadTree) std.Io {
        return self.bodyManager().io;
    }

    /// Context used to lock a physics lock (mLockContext)
    fn lockContext(self: *const BroadPhaseQuadTree) PhysicsLockFile.PhysicsLockContext {
        return self.bodyManager();
    }

    /// Initialize the broadphase (see BroadPhase.initBroadPhase).
    /// Commits every node page: about 170 bytes per max body of the body manager, see the file comment.
    pub fn initBroadPhase(self: *BroadPhaseQuadTree, body_manager: *BodyManager, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void {
        try BroadPhase.impl.initBroadPhase(&self.base, body_manager, layer_interface);
        const allocator = self.base.base.allocator;

        // Store input parameters
        self.broad_phase_layer_interface = layer_interface;
        self.num_layers = layer_interface.getNumBroadPhaseLayers();
        std.debug.assert(self.num_layers < broad_phase_layer_invalid.value);

        // Store max bodies
        self.max_bodies = body_manager.getMaxBodies();

        // Initialize tracking data
        const tracking = try allocator.alloc(Tracking, self.max_bodies);
        @memset(tracking, .{});
        self.tracking = tracking;

        // Init allocator
        // Estimate the amount of nodes we're going to need
        const num_leaves: u32 = @as(u32, @intCast(self.max_bodies + 1)) / 2; // Assume 50% fill
        const num_leaves_plus_internal_nodes = num_leaves + (num_leaves + 2) / 3; // = Sum(num_leaves * 4^-i) with i = [0, Inf].
        self.node_allocator = try .init(allocator, body_manager.io, 2 * num_leaves_plus_internal_nodes, 256); // We use double the amount of nodes while rebuilding the tree during Update()

        // Zolt (D9): commit every node page now, so that adding bodies and swapping trees never allocate
        try self.node_allocator.reserveAllPages();

        // Init sub trees
        const layers = try allocator.alloc(QuadTree, self.num_layers);
        for (layers) |*layer| {
            layer.* = .empty;
            layer.init(&self.node_allocator, allocator);
            if (Core.enable_asserts) layer.triggered_report = &self.mutable_fields.triggered_report;
        }
        self.layers = layers;
    }

    /// Must be called just before updating the broadphase when none of the body mutexes are locked
    pub fn frameSync(self: *BroadPhaseQuadTree) void {
        // Take a unique lock on the old query lock so that we know no one is using the old nodes anymore.
        // Note that nothing should be locked at this point to avoid risking a lock inversion deadlock.
        // Note that in other places where we lock this mutex we don't use SharedLock to detect lock inversions. As long as
        // nothing else is locked this is safe. This is why BroadPhaseQuery should be the highest priority lock.
        var root_lock: UniqueLock(SharedMutex) = .init(&self.mutable_fields.query_locks[self.query_lock_idx.load(.seq_cst) ^ 1], self.io(), self.lockContext(), .broad_phase_query);
        defer root_lock.deinit();

        for (self.layers) |*layer|
            layer.discardOldTree();
    }

    /// Should be called after many objects have been inserted to make the broadphase more efficient, usually done on startup only
    /// (Zolt: Allocator.Error from the rebuild of a layer: the layers before it were optimized, the others are unchanged)
    pub fn optimize(self: *BroadPhaseQuadTree) Allocator.Error!void {
        // Free the previous tree so we can create a new optimized tree
        self.frameSync();

        self.lockModifications();

        var result: Allocator.Error!void = {};
        for (self.layers) |*tree| {
            if (tree.hasBodies() or tree.isDirty()) {
                const update_state = tree.updatePrepare(self.bodyManager().getBodies(), self.tracking, true) catch |err| {
                    result = err; // Zolt: this tree is unchanged
                    break;
                };
                tree.updateFinalize(self.bodyManager().getBodies(), self.tracking, update_state);
            }
        }

        self.unlockModifications();

        // Free the tree from before we created a new optimized tree
        self.frameSync();

        try result;

        self.next_layer_to_update = 0;
    }

    /// Must be called before UpdatePrepare to prevent modifications from being made to the tree
    pub fn lockModifications(self: *BroadPhaseQuadTree) void {
        // From this point on we prevent modifications to the tree
        PhysicsLock.lock(&self.update_mutex, self.io(), self.lockContext(), .broad_phase_update);
    }

    /// Update the broadphase (see BroadPhase.updatePrepare). Zolt: on error nothing changed.
    pub fn updatePrepare(self: *BroadPhaseQuadTree) Allocator.Error!BroadPhase.UpdateState {
        // LockModifications should have been called
        if (Core.enable_asserts) std.debug.assert(self.update_mutex.isLocked());

        // Create update state
        var update_state: BroadPhase.UpdateState = .{};
        const update_state_impl: *UpdateStateImpl = @ptrCast(&update_state);

        // Zolt: the broad phase is unchanged on error
        const next_layer_to_update = self.next_layer_to_update;
        errdefer self.next_layer_to_update = next_layer_to_update;

        // Loop until we've seen all layers
        for (0..self.num_layers) |_| {
            // Get the layer
            const tree = &self.layers[self.next_layer_to_update];
            self.next_layer_to_update = (self.next_layer_to_update + 1) % self.num_layers;

            // If it is dirty we update this one
            if (tree.hasBodies() and tree.isDirty() and tree.canBeUpdated()) {
                update_state_impl.tree = tree;
                update_state_impl.update_state = try tree.updatePrepare(self.bodyManager().getBodies(), self.tracking, false);
                return update_state;
            }
        }

        // Nothing to update
        update_state_impl.tree = null;
        return update_state;
    }

    /// Finalizing the update will quickly apply the changes (a zeroed state does nothing)
    pub fn updateFinalize(self: *BroadPhaseQuadTree, update_state: *const BroadPhase.UpdateState) void {
        // LockModifications should have been called
        if (Core.enable_asserts) std.debug.assert(self.update_mutex.isLocked());

        // Test if a tree was updated
        const update_state_impl: *const UpdateStateImpl = @ptrCast(update_state);
        const tree = update_state_impl.tree orelse return;

        tree.updateFinalize(self.bodyManager().getBodies(), self.tracking, update_state_impl.update_state);

        // Make all queries from now on use the new lock
        self.query_lock_idx.store(self.query_lock_idx.load(.seq_cst) ^ 1, .seq_cst);
    }

    /// Must be called after UpdateFinalize to allow modifications to the broadphase
    pub fn unlockModifications(self: *BroadPhaseQuadTree) void {
        // From this point on we allow modifications to the tree again
        PhysicsLock.unlock(&self.update_mutex, self.io(), self.lockContext(), .broad_phase_update);
    }

    fn lessThanBroadPhaseLayer(bodies: []const BodySlot, lhs: BodyID, rhs: BodyID) bool {
        return bodies[lhs.getIndex()].getBody().getBroadPhaseLayer().lessThan(bodies[rhs.getIndex()].getBody().getBroadPhaseLayer());
    }

    fn layerLessThanBody(bodies: []const BodySlot, layer: BroadPhaseLayer.Type, body_id: BodyID) bool {
        return layer < bodies[body_id.getIndex()].getBody().getBroadPhaseLayer().getValue();
    }

    fn lessThanTrackedBroadPhaseLayer(tracking: []const Tracking, lhs: BodyID, rhs: BodyID) bool {
        return tracking[lhs.getIndex()].broad_phase_layer.load(.seq_cst) < tracking[rhs.getIndex()].broad_phase_layer.load(.seq_cst);
    }

    fn layerLessThanTrackedBody(tracking: []const Tracking, layer: BroadPhaseLayer.Type, body_id: BodyID) bool {
        return layer < tracking[body_id.getIndex()].broad_phase_layer.load(.seq_cst);
    }

    fn lessThanTrackedObjectLayer(tracking: []const Tracking, lhs: BodyID, rhs: BodyID) bool {
        return tracking[lhs.getIndex()].object_layer.load(.seq_cst) < tracking[rhs.getIndex()].object_layer.load(.seq_cst);
    }

    fn objectLayerLessThanTrackedBody(tracking: []const Tracking, layer: ObjectLayer, body_id: BodyID) bool {
        return layer < tracking[body_id.getIndex()].object_layer.load(.seq_cst);
    }

    /// std::upper_bound(begin, end, value, comp): the first body for which `comp(value, body)` is true (the bodies are
    /// partitioned by it)
    fn upperBound(comptime T: type, body_ids: []const BodyID, value: T, context: anytype, comptime comp: fn (@TypeOf(context), T, BodyID) bool) usize {
        var first: usize = 0;
        var count = body_ids.len;
        while (count > 0) {
            const step = count / 2;
            const it = first + step;
            if (!comp(context, value, body_ids[it])) {
                first = it + 1;
                count -= step + 1;
            } else {
                count = step;
            }
        }
        return first;
    }

    /// Prepare adding bodies to the broadphase (see BroadPhase.addBodiesPrepare). Zolt: on error nothing changed.
    pub fn addBodiesPrepare(self: *BroadPhaseQuadTree, body_ids: []BodyID) Allocator.Error!BroadPhase.AddState {
        if (body_ids.len <= 0)
            return null;

        const allocator = self.base.base.allocator;
        const state = try allocator.alloc(LayerState, self.num_layers);
        @memset(state, .{});
        try self.addBodiesPrepareInto(body_ids, state, null); // Zolt: frees the state on error
        return state.ptr;
    }

    /// The body of AddBodiesPrepare with the state allocated by the caller. Zolt: `centers` (null: BuildTree allocates)
    /// is a buffer for BuildTree with at least body_ids.len entries. On error the layers that were prepared are aborted
    /// and the state is freed (AddBodiesAbort).
    fn addBodiesPrepareInto(self: *BroadPhaseQuadTree, body_ids: []BodyID, state: []LayerState, centers: ?[]Vec3) Allocator.Error!void {
        const bodies = self.bodyManager().getBodies();
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        // Zolt: undo the layers that were prepared (their bodies are set below after a successful prepare) and free the state
        errdefer self.addBodiesAbort(body_ids, state.ptr);

        // Sort bodies on layer
        quickSort(BodyID, body_ids, bodies, lessThanBroadPhaseLayer);

        var b_start: usize = 0;
        const b_end = body_ids.len;
        while (b_start < b_end) {
            // Get broadphase layer
            const broadphase_layer = bodies[body_ids[b_start].getIndex()].getBody().getBroadPhaseLayer().getValue();
            std.debug.assert(broadphase_layer < self.num_layers);

            // Find first body with different layer
            const b_mid = b_start + upperBound(BroadPhaseLayer.Type, body_ids[b_start..b_end], broadphase_layer, bodies, layerLessThanBody);
            const layer_bodies = body_ids[b_start..b_mid];

            // Keep track of state for this layer
            const layer_state = &state[broadphase_layer];

            // Insert all bodies of the same layer (Zolt: the bodies of the layer state are set after a successful prepare)
            layer_state.add_state = try self.layers[broadphase_layer].addBodiesPrepareWithCenters(bodies, self.tracking, layer_bodies, centers);
            layer_state.bodies = layer_bodies;

            // Keep track in which tree we placed the object
            for (layer_bodies) |b| {
                const index = b.getIndex();
                std.debug.assert(bodies[index].getBody().getID().eql(b)); // Provided BodyID doesn't match BodyID in body manager
                std.debug.assert(!bodies[index].getBody().isInBroadPhase());
                const t = &self.tracking[index];
                std.debug.assert(t.broad_phase_layer.load(.seq_cst) == broad_phase_layer_invalid.value);
                t.broad_phase_layer.store(broadphase_layer, .seq_cst);
                std.debug.assert(t.object_layer.load(.seq_cst) == object_layer_invalid);
                t.object_layer.store(bodies[index].getBody().getObjectLayer(), .seq_cst);
            }

            // Repeat
            b_start = b_mid;
        }
    }

    /// Finalize adding bodies to the broadphase (see BroadPhase.addBodiesFinalize)
    pub fn addBodiesFinalize(self: *BroadPhaseQuadTree, body_ids: []BodyID, add_state: BroadPhase.AddState) void {
        if (body_ids.len <= 0) {
            std.debug.assert(add_state == null);
            return;
        }

        // This cannot run concurrently with UpdatePrepare()/UpdateFinalize()
        var lock: SharedLock(SharedMutex) = .init(&self.update_mutex, self.io(), self.lockContext(), .broad_phase_update);
        defer lock.deinit();

        const bodies = self.bodyManager().getBodies();
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        const state: [*]LayerState = @ptrCast(@alignCast(add_state.?));

        for (0..self.num_layers) |broadphase_layer| {
            const l = &state[broadphase_layer];
            if (l.bodies) |layer_bodies| {
                // Insert all bodies of the same layer
                self.layers[broadphase_layer].addBodiesFinalize(self.tracking, @intCast(layer_bodies.len), &l.add_state);

                // Mark added to broadphase
                for (layer_bodies) |b| {
                    const index = b.getIndex();
                    const body = bodies[index].getBody();
                    std.debug.assert(body.getID().eql(b)); // Provided BodyID doesn't match BodyID in body manager
                    std.debug.assert(self.tracking[index].broad_phase_layer.load(.seq_cst) == broadphase_layer);
                    std.debug.assert(self.tracking[index].object_layer.load(.seq_cst) == body.getObjectLayer());
                    std.debug.assert(!body.isInBroadPhase());
                    body.setInBroadPhaseInternal(true);
                }
            }
        }

        self.base.base.allocator.free(state[0..self.num_layers]);
    }

    /// Abort adding bodies to the broadphase (see BroadPhase.addBodiesAbort)
    pub fn addBodiesAbort(self: *BroadPhaseQuadTree, body_ids: []BodyID, add_state: BroadPhase.AddState) void {
        if (body_ids.len <= 0) {
            std.debug.assert(add_state == null);
            return;
        }

        const bodies = self.bodyManager().getBodies();
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        const state: [*]LayerState = @ptrCast(@alignCast(add_state.?));

        for (0..self.num_layers) |broadphase_layer| {
            const l = &state[broadphase_layer];
            if (l.bodies) |layer_bodies| {
                // Insert all bodies of the same layer
                self.layers[broadphase_layer].addBodiesAbort(self.tracking, &l.add_state);

                // Reset bookkeeping
                for (layer_bodies) |b| {
                    const index = b.getIndex();
                    std.debug.assert(bodies[index].getBody().getID().eql(b)); // Provided BodyID doesn't match BodyID in body manager
                    std.debug.assert(!bodies[index].getBody().isInBroadPhase());
                    const t = &self.tracking[index];
                    std.debug.assert(t.broad_phase_layer.load(.seq_cst) == broadphase_layer);
                    t.broad_phase_layer.store(broad_phase_layer_invalid.value, .seq_cst);
                    t.object_layer.store(object_layer_invalid, .seq_cst);
                }
            }
        }

        self.base.base.allocator.free(state[0..self.num_layers]);
    }

    /// Remove bodies from the broadphase (see BroadPhase.removeBodies)
    pub fn removeBodies(self: *BroadPhaseQuadTree, body_ids: []BodyID) void {
        if (body_ids.len <= 0)
            return;

        // This cannot run concurrently with UpdatePrepare()/UpdateFinalize()
        var lock: SharedLock(SharedMutex) = .init(&self.update_mutex, self.io(), self.lockContext(), .broad_phase_update);
        defer lock.deinit();

        const bodies = self.bodyManager().getBodies();
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        // Sort bodies on layer
        const tracking = self.tracking; // C pointer or else sort is incredibly slow in debug mode
        quickSort(BodyID, body_ids, @as([]const Tracking, tracking), lessThanTrackedBroadPhaseLayer);

        var b_start: usize = 0;
        const b_end = body_ids.len;
        while (b_start < b_end) {
            // Get broad phase layer
            const broadphase_layer = self.tracking[body_ids[b_start].getIndex()].broad_phase_layer.load(.seq_cst);
            std.debug.assert(broadphase_layer != broad_phase_layer_invalid.value);

            // Find first body with different layer
            const b_mid = b_start + upperBound(BroadPhaseLayer.Type, body_ids[b_start..b_end], broadphase_layer, @as([]const Tracking, tracking), layerLessThanTrackedBody);

            // Remove all bodies of the same layer
            self.layers[broadphase_layer].removeBodies(bodies, self.tracking, body_ids[b_start..b_mid]);

            for (body_ids[b_start..b_mid]) |b| {
                // Reset bookkeeping
                const index = b.getIndex();
                const t = &tracking[index];
                t.broad_phase_layer.store(broad_phase_layer_invalid.value, .seq_cst);
                t.object_layer.store(object_layer_invalid, .seq_cst);

                // Mark removed from broadphase
                std.debug.assert(bodies[index].getBody().isInBroadPhase());
                bodies[index].getBody().setInBroadPhaseInternal(false);
            }

            // Repeat
            b_start = b_mid;
        }
    }

    /// Call whenever the aabb of a body changes (see BroadPhase.notifyBodiesAABBChanged)
    pub fn notifyBodiesAABBChanged(self: *BroadPhaseQuadTree, body_ids: []BodyID, take_lock: bool) void {
        if (body_ids.len <= 0)
            return;

        // This cannot run concurrently with UpdatePrepare()/UpdateFinalize()
        if (take_lock)
            PhysicsLock.lockShared(&self.update_mutex, self.io(), self.lockContext(), .broad_phase_update)
        else if (Core.enable_asserts)
            std.debug.assert(self.update_mutex.isLocked());

        const bodies = self.bodyManager().getBodies();
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        // Sort bodies on layer
        const tracking: []const Tracking = self.tracking; // C pointer or else sort is incredibly slow in debug mode
        quickSort(BodyID, body_ids, tracking, lessThanTrackedBroadPhaseLayer);

        var b_start: usize = 0;
        const b_end = body_ids.len;
        while (b_start < b_end) {
            // Get broadphase layer
            const broadphase_layer = tracking[body_ids[b_start].getIndex()].broad_phase_layer.load(.seq_cst);
            std.debug.assert(broadphase_layer != broad_phase_layer_invalid.value);

            // Find first body with different layer
            const b_mid = b_start + upperBound(BroadPhaseLayer.Type, body_ids[b_start..b_end], broadphase_layer, tracking, layerLessThanTrackedBody);

            // Notify all bodies of the same layer changed
            self.layers[broadphase_layer].notifyBodiesAABBChanged(bodies, self.tracking, body_ids[b_start..b_mid]);

            // Repeat
            b_start = b_mid;
        }

        if (take_lock)
            PhysicsLock.unlockShared(&self.update_mutex, self.io(), self.lockContext(), .broad_phase_update);
    }

    /// Call whenever the layer (and optionally the aabb as well) of a body changes (can change order of ioBodies array)
    /// (Zolt: Allocator.Error, nothing changed then: what AddBodiesPrepare needs is allocated before anything changes)
    pub fn notifyBodiesLayerChanged(self: *BroadPhaseQuadTree, body_ids: []BodyID) Allocator.Error!void {
        if (body_ids.len <= 0)
            return;

        // First sort the bodies that actually changed layer to beginning of the array
        const bodies = self.bodyManager().getBodies();
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        // Zolt: the memory of AddBodiesPrepare for the bodies that change broad phase layer, allocated before anything
        // changes (RemoveBodies below cannot be undone)
        var num_changed: usize = 0;
        for (body_ids) |b| {
            if (self.tracking[b.getIndex()].broad_phase_layer.load(.seq_cst) != bodies[b.getIndex()].getBody().getBroadPhaseLayer().getValue())
                num_changed += 1;
        }
        const allocator = self.base.base.allocator;
        const state: []LayerState = if (num_changed > 0) try allocator.alloc(LayerState, self.num_layers) else &.{};
        var state_owned = state.len > 0;
        defer if (state_owned) allocator.free(state);
        const centers: []Vec3 = if (num_changed > 0) try allocator.alloc(Vec3, num_changed) else &.{};
        defer allocator.free(centers);

        var number = body_ids.len;
        var i = body_ids.len;
        while (i > 0) {
            i -= 1;
            const body_id = &body_ids[i];
            const index = body_id.getIndex();
            const body = bodies[index].getBody();
            std.debug.assert(body.getID().eql(body_id.*)); // Provided BodyID doesn't match BodyID in body manager
            const broadphase_layer = body.getBroadPhaseLayer().getValue();
            std.debug.assert(broadphase_layer < self.num_layers);
            if (self.tracking[index].broad_phase_layer.load(.seq_cst) == broadphase_layer) {
                // Update tracking information
                self.tracking[index].object_layer.store(body.getObjectLayer(), .seq_cst);

                // Move the body to the end, layer didn't change
                std.mem.swap(BodyID, body_id, &body_ids[number - 1]);
                number -= 1;
            }
        }
        std.debug.assert(number == num_changed);

        if (number > 0) {
            // Changing layer requires us to remove from one tree and add to another, so this is equivalent to removing all bodies first and then adding them again
            const changed = body_ids[0..number];
            self.removeBodies(changed);
            @memset(state, .{});
            state_owned = false; // AddBodiesFinalize (or the abort on error) frees it
            self.addBodiesPrepareInto(changed, state, centers) catch unreachable; // Cannot fail: BuildTree uses `centers`
            self.addBodiesFinalize(changed, state.ptr);
        }
    }

    /// Zolt: the BroadPhaseQuery queries have no error channel (CollisionArchitecture rule 4): a node stack that cannot
    /// grow on the heap (only in a tree deeper than about 40 levels) panics, where Jolt's allocator aborts
    fn queryOutOfMemory() noreturn {
        @panic("BroadPhaseQuadTree: Out of memory while growing the node stack of a query");
    }

    /// Cast a ray and add any hits to ioCollector
    pub fn castRay(self: *const BroadPhaseQuadTree, ray: *const RayCast, collector: *RayCastBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        // Prevent this from running in parallel with node deletion in FrameSync(), see notes there
        const query_lock = &self.mutable_fields.query_locks[self.query_lock_idx.load(.seq_cst)];
        query_lock.lockShared(self.io());
        defer query_lock.unlockShared(self.io());

        // Loop over all layers and test the ones that could hit
        for (self.layers, 0..) |*tree, l| {
            if (tree.hasBodies() and broad_phase_layer_filter.shouldCollide(.init(@intCast(l)))) {
                tree.castRay(ray, collector, object_layer_filter, self.tracking) catch queryOutOfMemory();
                if (collector.shouldEarlyOut())
                    break;
            }
        }
    }

    /// Get bodies intersecting with inBox and any hits to ioCollector
    pub fn collideAABox(self: *const BroadPhaseQuadTree, box: *const AABox, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        // Prevent this from running in parallel with node deletion in FrameSync(), see notes there
        const query_lock = &self.mutable_fields.query_locks[self.query_lock_idx.load(.seq_cst)];
        query_lock.lockShared(self.io());
        defer query_lock.unlockShared(self.io());

        // Loop over all layers and test the ones that could hit
        for (self.layers, 0..) |*tree, l| {
            if (tree.hasBodies() and broad_phase_layer_filter.shouldCollide(.init(@intCast(l)))) {
                tree.collideAABox(box, collector, object_layer_filter, self.tracking) catch queryOutOfMemory();
                if (collector.shouldEarlyOut())
                    break;
            }
        }
    }

    /// Get bodies intersecting with a sphere and any hits to ioCollector
    pub fn collideSphere(self: *const BroadPhaseQuadTree, center: Vec3, radius: f32, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        // Prevent this from running in parallel with node deletion in FrameSync(), see notes there
        const query_lock = &self.mutable_fields.query_locks[self.query_lock_idx.load(.seq_cst)];
        query_lock.lockShared(self.io());
        defer query_lock.unlockShared(self.io());

        // Loop over all layers and test the ones that could hit
        for (self.layers, 0..) |*tree, l| {
            if (tree.hasBodies() and broad_phase_layer_filter.shouldCollide(.init(@intCast(l)))) {
                tree.collideSphere(center, radius, collector, object_layer_filter, self.tracking) catch queryOutOfMemory();
                if (collector.shouldEarlyOut())
                    break;
            }
        }
    }

    /// Get bodies intersecting with a point and any hits to ioCollector
    pub fn collidePoint(self: *const BroadPhaseQuadTree, point: Vec3, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        // Prevent this from running in parallel with node deletion in FrameSync(), see notes there
        const query_lock = &self.mutable_fields.query_locks[self.query_lock_idx.load(.seq_cst)];
        query_lock.lockShared(self.io());
        defer query_lock.unlockShared(self.io());

        // Loop over all layers and test the ones that could hit
        for (self.layers, 0..) |*tree, l| {
            if (tree.hasBodies() and broad_phase_layer_filter.shouldCollide(.init(@intCast(l)))) {
                tree.collidePoint(point, collector, object_layer_filter, self.tracking) catch queryOutOfMemory();
                if (collector.shouldEarlyOut())
                    break;
            }
        }
    }

    /// Get bodies intersecting with an oriented box and any hits to ioCollector
    pub fn collideOrientedBox(self: *const BroadPhaseQuadTree, box: *const OrientedBox, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        // Prevent this from running in parallel with node deletion in FrameSync(), see notes there
        const query_lock = &self.mutable_fields.query_locks[self.query_lock_idx.load(.seq_cst)];
        query_lock.lockShared(self.io());
        defer query_lock.unlockShared(self.io());

        // Loop over all layers and test the ones that could hit
        for (self.layers, 0..) |*tree, l| {
            if (tree.hasBodies() and broad_phase_layer_filter.shouldCollide(.init(@intCast(l)))) {
                tree.collideOrientedBox(box, collector, object_layer_filter, self.tracking) catch queryOutOfMemory();
                if (collector.shouldEarlyOut())
                    break;
            }
        }
    }

    /// Same as BroadPhaseQuery::CastAABox but can be implemented in a way to take no broad phase locks.
    /// (Zolt: Allocator.Error when a node stack cannot grow on the heap, the hits found so far were reported)
    pub fn castAABoxNoLock(self: *const BroadPhaseQuadTree, box: *const AABoxCast, collector: *CastShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) Allocator.Error!void {
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        // Loop over all layers and test the ones that could hit
        for (self.layers, 0..) |*tree, l| {
            if (tree.hasBodies() and broad_phase_layer_filter.shouldCollide(.init(@intCast(l)))) {
                try tree.castAABox(box, collector, object_layer_filter, self.tracking);
                if (collector.shouldEarlyOut())
                    break;
            }
        }
    }

    /// Cast a box and add any hits to ioCollector
    pub fn castAABox(self: *const BroadPhaseQuadTree, box: *const AABoxCast, collector: *CastShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        // Prevent this from running in parallel with node deletion in FrameSync(), see notes there
        const query_lock = &self.mutable_fields.query_locks[self.query_lock_idx.load(.seq_cst)];
        query_lock.lockShared(self.io());
        defer query_lock.unlockShared(self.io());

        self.castAABoxNoLock(box, collector, broad_phase_layer_filter, object_layer_filter) catch queryOutOfMemory();
    }

    /// Find all colliding pairs between dynamic bodies (see BroadPhase.findCollidingPairs)
    /// (Zolt: Allocator.Error when a node stack cannot grow on the heap, see the file comment)
    pub fn findCollidingPairs(self: *const BroadPhaseQuadTree, active_bodies: []BodyID, speculative_contact_distance: f32, object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, object_layer_pair_filter: *const ObjectLayerPairFilter, pair_collector: *BodyPairCollector) Allocator.Error!void {
        const bodies = self.bodyManager().getBodies();
        std.debug.assert(self.max_bodies == self.bodyManager().getMaxBodies());

        // Note that we don't take any locks at this point. We know that the tree is not going to be swapped or deleted while finding collision pairs due to the way the jobs are scheduled in the PhysicsSystem::Update.

        // Sort bodies on layer
        const tracking: []const Tracking = self.tracking; // C pointer or else sort is incredibly slow in debug mode
        quickSort(BodyID, active_bodies, tracking, lessThanTrackedObjectLayer);

        var b_start: usize = 0;
        const b_end = active_bodies.len;
        while (b_start < b_end) {
            // Get broadphase layer
            const object_layer = tracking[active_bodies[b_start].getIndex()].object_layer.load(.seq_cst);
            std.debug.assert(object_layer != object_layer_invalid);

            // Find first body with different layer
            const b_mid = b_start + upperBound(ObjectLayer, active_bodies[b_start..b_end], object_layer, tracking, objectLayerLessThanTrackedBody);

            // Loop over all layers and test the ones that could hit
            for (self.layers, 0..) |*tree, l| {
                if (tree.hasBodies() and object_vs_broad_phase_layer_filter.shouldCollide(object_layer, .init(@intCast(l))))
                    try tree.findCollidingPairs(bodies, active_bodies[b_start..b_mid], speculative_contact_distance, pair_collector, object_layer_pair_filter);
            }

            // Repeat
            b_start = b_mid;
        }
    }

    /// Get the bounding box of all objects in the broadphase
    pub fn getBounds(self: *const BroadPhaseQuadTree) AABox {
        // Prevent this from running in parallel with node deletion in FrameSync(), see notes there
        const query_lock = &self.mutable_fields.query_locks[self.query_lock_idx.load(.seq_cst)];
        query_lock.lockShared(self.io());
        defer query_lock.unlockShared(self.io());

        var bounds: AABox = .empty;
        for (self.layers) |*layer|
            bounds.encapsulate(layer.getBounds());
        return bounds;
    }
};

test "BroadPhaseQuadTree upperBound is std::upper_bound" {
    const Ctx = struct {
        fn less(_: void, value: u32, id: BodyID) bool {
            return value < id.getIndex();
        }
    };
    const ids = [_]BodyID{ .init(1), .init(3), .init(3), .init(7) };
    try std.testing.expectEqual(@as(usize, 0), BroadPhaseQuadTree.upperBound(u32, &ids, 0, {}, Ctx.less));
    try std.testing.expectEqual(@as(usize, 1), BroadPhaseQuadTree.upperBound(u32, &ids, 1, {}, Ctx.less));
    try std.testing.expectEqual(@as(usize, 1), BroadPhaseQuadTree.upperBound(u32, &ids, 2, {}, Ctx.less));
    try std.testing.expectEqual(@as(usize, 3), BroadPhaseQuadTree.upperBound(u32, &ids, 3, {}, Ctx.less));
    try std.testing.expectEqual(@as(usize, 4), BroadPhaseQuadTree.upperBound(u32, &ids, 7, {}, Ctx.less));
    try std.testing.expectEqual(@as(usize, 0), BroadPhaseQuadTree.upperBound(u32, &.{}, 7, {}, Ctx.less));
}
