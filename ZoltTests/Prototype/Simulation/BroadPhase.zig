//! Phase 5 architecture prototype: reduced Jolt/Physics/Collision/BroadPhase/{BroadPhaseQuery, BroadPhase}.h and
//! Jolt/Physics/Body/BodyPair.h, plus `PrototypeBroadPhase`: Jolt's BroadPhaseBruteForce extended with the allocation
//! and locking protocol of BroadPhaseQuadTree (Docs/Zolt/SimulationArchitecture.md D9).
//!
//! - A three level pattern A hierarchy BroadPhaseQuery -> BroadPhase -> PrototypeBroadPhase: BroadPhase adds virtual
//!   functions (VTable prefix), keeps Jolt's default bodies in `impl`, and owns a virtual destructor that is not
//!   reference counted (`destroy` is `delete mBroadPhase`).
//! - The functions that allocate in BroadPhaseQuadTree return Allocator.Error: `addBodiesPrepare` (`new LayerState[]`)
//!   and `updatePrepare` (`new NodeID[]` / `new Vec3[]`, run inside the BroadPhasePrepare job). Finalize / Abort never
//!   fail; a zeroed UpdateState makes UpdateFinalize a no-op.
//! - Rule M: queries are `*const` and lock the C++ `mutable` query locks in `mutable_fields`.

const std = @import("std");
const zolt = @import("zolt");
const virtual = zolt.virtual;
const Vec3 = zolt.Vec3;
const AABox = zolt.AABox;
const BodyID = zolt.BodyID;
const SharedMutex = zolt.SharedMutex;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const BroadPhaseLayerFilter = zolt.BroadPhaseLayerFilter;
const ObjectLayerFilter = zolt.ObjectLayerFilter;
const ObjectLayerPairFilter = zolt.ObjectLayerPairFilter;
const ObjectVsBroadPhaseLayerFilter = zolt.ObjectVsBroadPhaseLayerFilter;
const Allocator = std.mem.Allocator;
const Body = @import("Body.zig").Body;
const BodyManagerFile = @import("BodyManager.zig");
const BodyManager = BodyManagerFile.BodyManager;
const PhysicsLock = BodyManagerFile.PhysicsLock;

/// Structure that holds a body pair (extern: a LockFreeHashMap key in the ContactConstraintManager)
pub const BodyPair = extern struct {
    body_a: BodyID,
    body_b: BodyID,

    pub fn init(a: BodyID, b: BodyID) BodyPair {
        return .{ .body_a = a, .body_b = b };
    }

    comptime {
        std.debug.assert(@sizeOf(BodyPair) == 8);
    }
};

/// Collector for the broad phase pairs (Jolt: CollisionCollector<BodyPair, CollisionCollectorTraitsCollideShape>)
pub const BodyPairCollector = zolt.CollisionCollector(BodyPair, zolt.CollisionCollectorTraitsCollideShape);

/// Collector for the broad phase collide queries (Jolt: CollisionCollector<BodyID, CollisionCollectorTraitsCollideShape>)
pub const CollideShapeBodyCollector = zolt.CollisionCollector(BodyID, zolt.CollisionCollectorTraitsCollideShape);

/// Interface to the broadphase that can perform collision queries (reduced: CollideAABox and GetBounds; CastRay,
/// CollideSphere, CollidePoint, CollideOrientedBox and CastAABox follow the same pattern)
pub const BroadPhaseQuery = struct {
    pub const VTable = struct {
        deinit: *const fn (self: *BroadPhaseQuery) void,
        destroy: *const fn (self: *BroadPhaseQuery) void,
        /// Get bodies intersecting with inBox and any hits to ioCollector
        collideAABox: *const fn (self: *const BroadPhaseQuery, box: *const AABox, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void,
        /// Get the bounding box of all objects in the broadphase
        getBounds: *const fn (self: *const BroadPhaseQuery) AABox,
    };

    vtable: *const VTable,

    /// Allocator that frees the object (Jolt: global delete) and its arrays
    allocator: Allocator,

    /// Get bodies intersecting with inBox and any hits to ioCollector (Jolt's defaulted filter arguments: options)
    pub fn collideAABox(self: *const BroadPhaseQuery, box: *const AABox, collector: *CollideShapeBodyCollector, opts: struct { broad_phase_layer_filter: *const BroadPhaseLayerFilter = &.{}, object_layer_filter: *const ObjectLayerFilter = &.{} }) void {
        self.vtable.collideAABox(self, box, collector, opts.broad_phase_layer_filter, opts.object_layer_filter);
    }

    pub fn getBounds(self: *const BroadPhaseQuery) AABox {
        return self.vtable.getBounds(self);
    }

    /// delete through the virtual destructor (no reference count)
    pub fn destroy(self: *BroadPhaseQuery) void {
        self.vtable.destroy(self);
    }

    pub const impl = struct {};
};

/// Used to do hard work in a broadphase
pub const BroadPhase = struct {
    /// Context used during broadphase update (Jolt: struct UpdateState { void *mData[4]; }). Zeroed: nothing to finalize.
    pub const UpdateState = extern struct {
        data: [4]?*anyopaque = @splat(null),
    };

    /// Handle used during adding bodies to the broadphase
    pub const AddState = ?*anyopaque;

    pub const VTable = struct {
        base: BroadPhaseQuery.VTable,
        /// Initialize the broadphase (Init, renamed: `init` is the Zig constructor). Zolt: allocates, so Allocator.Error.
        initBroadPhase: *const fn (self: *BroadPhase, body_manager: *BodyManager, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void,
        /// Should be called after many objects have been inserted to make the broadphase more efficient, usually done on startup only
        optimize: *const fn (self: *BroadPhase) void,
        /// Must be called just before updating the broadphase when none of the body mutexes are locked
        frameSync: *const fn (self: *BroadPhase) void,
        /// Must be called before UpdatePrepare to prevent modifications from being made to the tree
        lockModifications: *const fn (self: *BroadPhase) void,
        /// Update the broadphase, needs to be called frequently to update the internal state when bodies have been
        /// modified. Zolt: BroadPhaseQuadTree builds the new tree here (heap allocations), so Allocator.Error; on error
        /// nothing changed.
        updatePrepare: *const fn (self: *BroadPhase) Allocator.Error!UpdateState,
        /// Finalizing the update will quickly apply the changes (never fails, a zeroed state does nothing)
        updateFinalize: *const fn (self: *BroadPhase, update_state: *const UpdateState) void,
        /// Must be called after UpdateFinalize to allow modifications to the broadphase
        unlockModifications: *const fn (self: *BroadPhase) void,
        /// Prepare adding inNumber bodies at ioBodies to the broadphase, returns a handle that should be used in
        /// AddBodiesFinalize/Abort. Zolt: BroadPhaseQuadTree allocates the layer states and builds trees here.
        addBodiesPrepare: *const fn (self: *BroadPhase, bodies: []BodyID) Allocator.Error!AddState,
        /// Finalize adding bodies to the broadphase, supply the return value of AddBodiesPrepare in inAddState.
        addBodiesFinalize: *const fn (self: *BroadPhase, bodies: []BodyID, add_state: AddState) void,
        /// Abort adding bodies to the broadphase, supply the return value of AddBodiesPrepare in inAddState.
        addBodiesAbort: *const fn (self: *BroadPhase, bodies: []BodyID, add_state: AddState) void,
        /// Remove inNumber bodies in ioBodies from the broadphase.
        removeBodies: *const fn (self: *BroadPhase, bodies: []BodyID) void,
        /// Call whenever the aabb of a body changes (can change order of ioBodies array)
        notifyBodiesAABBChanged: *const fn (self: *BroadPhase, bodies: []BodyID, take_lock: bool) void,
        /// Find all colliding pairs between dynamic bodies
        findCollidingPairs: *const fn (self: *const BroadPhase, active_bodies: []BodyID, speculative_contact_distance: f32, object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, object_layer_pair_filter: *const ObjectLayerPairFilter, pair_collector: *BodyPairCollector) void,
    };

    base: BroadPhaseQuery,

    /// Body manager (Jolt: BodyManager *mBodyManager = nullptr)
    body_manager: ?*BodyManager = null,

    /// Constructor of the introducing class
    pub fn init(comptime T: type, allocator: Allocator) BroadPhase {
        return .{ .base = .{ .vtable = &virtual.vtablePtr(VTable, T).base, .allocator = allocator } };
    }

    fn getVTable(self: *const BroadPhase) *const VTable {
        return virtual.downcast(VTable, self.base.vtable);
    }

    // Dispatchers
    pub fn initBroadPhase(self: *BroadPhase, body_manager: *BodyManager, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void {
        return self.getVTable().initBroadPhase(self, body_manager, layer_interface);
    }
    pub fn optimize(self: *BroadPhase) void {
        self.getVTable().optimize(self);
    }
    pub fn frameSync(self: *BroadPhase) void {
        self.getVTable().frameSync(self);
    }
    pub fn lockModifications(self: *BroadPhase) void {
        self.getVTable().lockModifications(self);
    }
    pub fn updatePrepare(self: *BroadPhase) Allocator.Error!UpdateState {
        return self.getVTable().updatePrepare(self);
    }
    pub fn updateFinalize(self: *BroadPhase, update_state: *const UpdateState) void {
        self.getVTable().updateFinalize(self, update_state);
    }
    pub fn unlockModifications(self: *BroadPhase) void {
        self.getVTable().unlockModifications(self);
    }
    pub fn addBodiesPrepare(self: *BroadPhase, bodies: []BodyID) Allocator.Error!AddState {
        return self.getVTable().addBodiesPrepare(self, bodies);
    }
    pub fn addBodiesFinalize(self: *BroadPhase, bodies: []BodyID, add_state: AddState) void {
        self.getVTable().addBodiesFinalize(self, bodies, add_state);
    }
    pub fn addBodiesAbort(self: *BroadPhase, bodies: []BodyID, add_state: AddState) void {
        self.getVTable().addBodiesAbort(self, bodies, add_state);
    }
    pub fn removeBodies(self: *BroadPhase, bodies: []BodyID) void {
        self.getVTable().removeBodies(self, bodies);
    }
    pub fn notifyBodiesAABBChanged(self: *BroadPhase, bodies: []BodyID, opts: struct { take_lock: bool = true }) void {
        self.getVTable().notifyBodiesAABBChanged(self, bodies, opts.take_lock);
    }
    pub fn findCollidingPairs(self: *const BroadPhase, active_bodies: []BodyID, speculative_contact_distance: f32, object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, object_layer_pair_filter: *const ObjectLayerPairFilter, pair_collector: *BodyPairCollector) void {
        self.getVTable().findCollidingPairs(self, active_bodies, speculative_contact_distance, object_vs_broad_phase_layer_filter, object_layer_pair_filter, pair_collector);
    }

    /// Jolt's default implementations of the optional virtual functions
    pub const impl = struct {
        pub fn initBroadPhase(self: *BroadPhase, body_manager: *BodyManager, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void {
            _ = layer_interface;
            self.body_manager = body_manager;
        }
        pub fn optimize(self: *BroadPhase) void {
            _ = self; // Optionally overridden by implementation
        }
        pub fn frameSync(self: *BroadPhase) void {
            _ = self; // Optionally overridden by implementation
        }
        pub fn lockModifications(self: *BroadPhase) void {
            _ = self; // Optionally overridden by implementation
        }
        pub fn updatePrepare(self: *BroadPhase) Allocator.Error!UpdateState {
            _ = self;
            return .{};
        }
        pub fn updateFinalize(self: *BroadPhase, update_state: *const UpdateState) void {
            _ = self; // Optionally overridden by implementation
            _ = update_state;
        }
        pub fn unlockModifications(self: *BroadPhase) void {
            _ = self; // Optionally overridden by implementation
        }
        pub fn addBodiesPrepare(self: *BroadPhase, bodies: []BodyID) Allocator.Error!AddState {
            _ = self; // By default the broadphase doesn't support this
            _ = bodies;
            return null;
        }
        pub fn addBodiesAbort(self: *BroadPhase, bodies: []BodyID, add_state: AddState) void {
            _ = self; // By default nothing needs to be done
            _ = bodies;
            _ = add_state;
        }
    };
};

/// BroadPhaseBruteForce (Jolt's test broad phase) extended with the parts of BroadPhaseQuadTree that matter for the
/// architecture: the update mutex, the double buffered query locks, a "tree" built in UpdatePrepare (heap) that
/// UpdateFinalize swaps in and FrameSync discards, and an AddState allocated in AddBodiesPrepare.
pub const PrototypeBroadPhase = struct {
    pub const overrides = .{ .initBroadPhase, .frameSync, .lockModifications, .updatePrepare, .updateFinalize, .unlockModifications, .addBodiesPrepare, .addBodiesFinalize, .addBodiesAbort, .removeBodies, .notifyBodiesAABBChanged, .findCollidingPairs, .collideAABox, .getBounds };

    /// The C++ `mutable` members (Rule M: behind a pointer, locked by const queries)
    const MutableFields = struct {
        /// BroadPhaseBruteForce::mMutex (protects body_ids)
        mutex: SharedMutex = .{},
        /// BroadPhaseQuadTree::mQueryLocks: we double buffer the query locks so that FrameSync can wait for the old tree's readers
        query_locks: [2]SharedMutex = @splat(.{}),
    };

    /// BroadPhaseQuadTree::UpdateStateImpl, stored in UpdateState's `void *mData[4]`
    const UpdateStateImpl = extern struct {
        tree: ?[*]BodyID = null,
        len: usize = 0,
    };

    comptime {
        std.debug.assert(@sizeOf(UpdateStateImpl) <= @sizeOf(BroadPhase.UpdateState));
        std.debug.assert(@alignOf(UpdateStateImpl) <= @alignOf(BroadPhase.UpdateState));
    }

    /// The state that AddBodiesPrepare allocates (BroadPhaseQuadTree: `new LayerState[mNumLayers]`)
    const AddStateImpl = struct {
        num_bodies: usize,
    };

    base: BroadPhase,

    /// Sorted list of body IDs in the broad phase
    body_ids: std.ArrayList(BodyID) = .empty,

    mutable_fields: ?*MutableFields = null,

    /// BroadPhaseQuadTree::mUpdateMutex: lock to ensure no one is modifying the tree while UpdatePrepare / Finalize run
    update_mutex: SharedMutex = .{},

    /// Which query lock is currently active
    query_lock_idx: std.atomic.Value(u32) = .init(0),

    /// The current "tree" (a snapshot of the body list) and the one that FrameSync discards
    tree: []BodyID = &.{},
    old_tree: ?[]BodyID = null,

    /// new PrototypeBroadPhase
    pub fn create(allocator: Allocator) Allocator.Error!*PrototypeBroadPhase {
        const self = try allocator.create(PrototypeBroadPhase);
        self.* = .{ .base = .init(PrototypeBroadPhase, allocator) };
        return self;
    }

    pub fn asBroadPhase(self: *PrototypeBroadPhase) *BroadPhase {
        return &self.base;
    }

    /// ~BroadPhase
    pub fn destruct(self: *PrototypeBroadPhase) void {
        const allocator = self.base.base.allocator;
        self.body_ids.deinit(allocator);
        allocator.free(self.tree);
        if (self.old_tree) |t| allocator.free(t);
        if (self.mutable_fields) |m| allocator.destroy(m);
    }

    fn bodyManager(self: *const PrototypeBroadPhase) *BodyManager {
        return self.base.body_manager.?;
    }

    fn io(self: *const PrototypeBroadPhase) std.Io {
        return self.bodyManager().io;
    }

    /// Zolt override of Init: Jolt's push_back in AddBodiesFinalize can't fail, so the list is reserved here
    pub fn initBroadPhase(self: *PrototypeBroadPhase, body_manager: *BodyManager, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void {
        try BroadPhase.impl.initBroadPhase(&self.base, body_manager, layer_interface);
        const allocator = self.base.base.allocator;
        self.mutable_fields = try allocator.create(MutableFields);
        self.mutable_fields.?.* = .{};
        try self.body_ids.ensureTotalCapacityPrecise(allocator, body_manager.getMaxBodies());
    }

    /// Discard the old tree once no query can use it anymore (QuadTree::DiscardOldTree under the old query lock)
    pub fn frameSync(self: *PrototypeBroadPhase) void {
        // Take a unique lock on the old query lock so that we know no one is using the old nodes anymore.
        // Note that nothing should be locked at this point to avoid risking a lock inversion deadlock.
        const old_lock = &self.mutable_fields.?.query_locks[self.query_lock_idx.load(.monotonic) ^ 1];
        PhysicsLock.lock(old_lock, self.io(), self.bodyManager(), .broad_phase_query);
        defer PhysicsLock.unlock(old_lock, self.io(), self.bodyManager(), .broad_phase_query);

        if (self.old_tree) |t| {
            self.base.base.allocator.free(t);
            self.old_tree = null;
        }
    }

    pub fn lockModifications(self: *PrototypeBroadPhase) void {
        // From this point on we prevent modifications to the tree
        PhysicsLock.lock(&self.update_mutex, self.io(), self.bodyManager(), .broad_phase_update);
    }

    /// Builds the new "tree" (QuadTree: allocates NodeID / Vec3 arrays and nodes). On error nothing changed.
    pub fn updatePrepare(self: *PrototypeBroadPhase) Allocator.Error!BroadPhase.UpdateState {
        // LockModifications should have been called
        std.debug.assert(self.update_mutex.isLocked());

        // Create update state
        var update_state: BroadPhase.UpdateState = .{};
        const update_state_impl: *UpdateStateImpl = @ptrCast(&update_state);

        // The previous tree has not been discarded yet (QuadTree::CanBeUpdated): nothing to update
        if (self.old_tree != null or self.body_ids.items.len == 0)
            return update_state;

        const new_tree = try self.base.base.allocator.dupe(BodyID, self.body_ids.items);
        update_state_impl.* = .{ .tree = new_tree.ptr, .len = new_tree.len };
        return update_state;
    }

    pub fn updateFinalize(self: *PrototypeBroadPhase, update_state: *const BroadPhase.UpdateState) void {
        // LockModifications should have been called
        std.debug.assert(self.update_mutex.isLocked());

        // Test if a tree was updated
        const update_state_impl: *const UpdateStateImpl = @ptrCast(update_state);
        const new_tree = update_state_impl.tree orelse return;

        // Swap in the new tree, the old one is freed by FrameSync
        std.debug.assert(self.old_tree == null);
        self.old_tree = self.tree;
        self.tree = new_tree[0..update_state_impl.len];

        // Make all queries from now on use the new lock
        _ = self.query_lock_idx.fetchXor(1, .monotonic);
    }

    pub fn unlockModifications(self: *PrototypeBroadPhase) void {
        // From this point on we allow modifications to the tree again
        PhysicsLock.unlock(&self.update_mutex, self.io(), self.bodyManager(), .broad_phase_update);
    }

    pub fn addBodiesPrepare(self: *PrototypeBroadPhase, bodies: []BodyID) Allocator.Error!BroadPhase.AddState {
        if (bodies.len == 0)
            return null;

        const state = try self.base.base.allocator.create(AddStateImpl);
        state.* = .{ .num_bodies = bodies.len };
        return state;
    }

    pub fn addBodiesFinalize(self: *PrototypeBroadPhase, bodies: []BodyID, add_state: BroadPhase.AddState) void {
        if (bodies.len == 0) {
            std.debug.assert(add_state == null);
            return;
        }

        // This cannot run concurrently with UpdatePrepare()/UpdateFinalize()
        PhysicsLock.lockShared(&self.update_mutex, self.io(), self.bodyManager(), .broad_phase_update);
        defer PhysicsLock.unlockShared(&self.update_mutex, self.io(), self.bodyManager(), .broad_phase_update);

        const mutex = &self.mutable_fields.?.mutex;
        mutex.lock(self.io());
        defer mutex.unlock(self.io());

        const state: *AddStateImpl = @ptrCast(@alignCast(add_state.?));
        std.debug.assert(state.num_bodies == bodies.len);
        self.base.base.allocator.destroy(state);

        // Add bodies
        const body_manager = self.bodyManager();
        for (bodies) |b| {
            const body = body_manager.getBody(b);

            // Validate that body ID is consistent with array index
            std.debug.assert(body.getID().eql(b));
            std.debug.assert(!body.isInBroadPhase());

            // Add it to the list
            self.body_ids.appendAssumeCapacity(body.getID());

            // Indicate body is in the broadphase
            body.setInBroadPhaseInternal(true);
        }

        // Resort
        zolt.quickSort(BodyID, self.body_ids.items, {}, bodyIDLessThan);
    }

    pub fn addBodiesAbort(self: *PrototypeBroadPhase, bodies: []BodyID, add_state: BroadPhase.AddState) void {
        _ = bodies;
        if (add_state) |s| self.base.base.allocator.destroy(@as(*AddStateImpl, @ptrCast(@alignCast(s))));
    }

    fn bodyIDLessThan(_: void, a: BodyID, b: BodyID) bool {
        return a.lessThan(b);
    }

    pub fn removeBodies(self: *PrototypeBroadPhase, bodies: []BodyID) void {
        // This cannot run concurrently with UpdatePrepare()/UpdateFinalize()
        PhysicsLock.lockShared(&self.update_mutex, self.io(), self.bodyManager(), .broad_phase_update);
        defer PhysicsLock.unlockShared(&self.update_mutex, self.io(), self.bodyManager(), .broad_phase_update);

        const mutex = &self.mutable_fields.?.mutex;
        mutex.lock(self.io());
        defer mutex.unlock(self.io());

        const body_manager = self.bodyManager();
        std.debug.assert(self.body_ids.items.len >= bodies.len);

        // Remove bodies
        for (bodies) |b| {
            const body = body_manager.getBody(b);

            // Validate that body ID is consistent with array index
            std.debug.assert(body.getID().eql(b));
            std.debug.assert(body.isInBroadPhase());

            // Find body id (std::lower_bound)
            const index = std.sort.lowerBound(BodyID, self.body_ids.items, body.getID(), struct {
                fn order(key: BodyID, item: BodyID) std.math.Order {
                    return std.math.order(key.getIndexAndSequenceNumber(), item.getIndexAndSequenceNumber());
                }
            }.order);
            std.debug.assert(index < self.body_ids.items.len and self.body_ids.items[index].eql(body.getID()));

            // Remove element
            _ = self.body_ids.orderedRemove(index);

            // Indicate body is no longer in the broadphase
            body.setInBroadPhaseInternal(false);
        }
    }

    pub fn notifyBodiesAABBChanged(self: *PrototypeBroadPhase, bodies: []BodyID, take_lock: bool) void {
        // This cannot run concurrently with UpdatePrepare()/UpdateFinalize() (QuadTree marks the tree dirty here)
        if (take_lock)
            PhysicsLock.lockShared(&self.update_mutex, self.io(), self.bodyManager(), .broad_phase_update)
        else
            std.debug.assert(self.update_mutex.isLocked());
        defer if (take_lock) PhysicsLock.unlockShared(&self.update_mutex, self.io(), self.bodyManager(), .broad_phase_update);

        // Brute force: do nothing, we directly reference the body
        _ = bodies;
    }

    pub fn findCollidingPairs(self: *const PrototypeBroadPhase, active_bodies: []BodyID, speculative_contact_distance: f32, object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, object_layer_pair_filter: *const ObjectLayerPairFilter, pair_collector: *BodyPairCollector) void {
        _ = object_vs_broad_phase_layer_filter; // BroadPhaseBruteForce does not use it
        const mutex = &self.mutable_fields.?.mutex;
        mutex.lockShared(self.io());
        defer mutex.unlockShared(self.io());

        const body_manager = self.bodyManager();

        // Loop through all active bodies
        for (active_bodies) |b1_id| {
            const body1 = body_manager.getBody(b1_id);
            const layer1 = body1.getObjectLayer();

            // Expand the bounding box by the speculative contact distance
            var bounds1 = body1.getWorldSpaceBounds().*;
            bounds1.expandBy(Vec3.replicate(speculative_contact_distance));

            // For all other bodies
            for (self.body_ids.items) |b2_id| {
                // Check if bodies can collide
                const body2 = body_manager.getBody(b2_id);
                if (!Body.findCollidingPairsCanCollide(body1, body2))
                    continue;

                // Check if layers can collide
                const layer2 = body2.getObjectLayer();
                if (!object_layer_pair_filter.shouldCollide(layer1, layer2))
                    continue;

                // Check if bounds overlap
                if (!bounds1.overlaps(body2.getWorldSpaceBounds().*))
                    continue;

                // Store overlapping pair
                pair_collector.addHit(&BodyPair.init(b1_id, b2_id));
            }
        }
    }

    /// A const query: locks the `mutable` query lock and list mutex through `mutable_fields` (Rule M)
    pub fn collideAABox(self: *const PrototypeBroadPhase, box: *const AABox, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        const mutable = self.mutable_fields.?;
        const query_lock = &mutable.query_locks[self.query_lock_idx.load(.monotonic)];
        query_lock.lockShared(self.io());
        defer query_lock.unlockShared(self.io());
        mutable.mutex.lockShared(self.io());
        defer mutable.mutex.unlockShared(self.io());

        const body_manager = self.bodyManager();
        for (self.body_ids.items) |b| {
            const body = body_manager.getBody(b);
            if (!broad_phase_layer_filter.shouldCollide(body.broad_phase_layer) or !object_layer_filter.shouldCollide(body.getObjectLayer()))
                continue;
            if (box.overlaps(body.getWorldSpaceBounds().*)) {
                collector.addHit(&b);
                if (collector.shouldEarlyOut())
                    break;
            }
        }
    }

    pub fn getBounds(self: *const PrototypeBroadPhase) AABox {
        const mutex = &self.mutable_fields.?.mutex;
        mutex.lockShared(self.io());
        defer mutex.unlockShared(self.io());

        var bounds: AABox = .empty;
        for (self.body_ids.items) |b|
            bounds.encapsulate(self.bodyManager().getBody(b).getWorldSpaceBounds().*);
        return bounds;
    }
};
