//! Port of: Jolt/Physics/Collision/BroadPhase/BroadPhase.h, Jolt/Physics/Collision/BroadPhase/BroadPhase.cpp
//! Status: complete
//! Not ported: JPH_TRACK_BROADPHASE_STATS (`JPH_IF_TRACK_BROADPHASE_STATS`, the virtual `ReportStats`), off in Jolt's default build
//!
//! SimulationArchitecture D9: pattern A, `BroadPhase.VTable` starts with `base: BroadPhaseQuery.VTable` and the bodies
//! of Jolt's optional virtual functions live in `impl`. Implementations embed `base: BroadPhase` and construct it
//! with `.init(T, allocator)` (the constructor of the introducing class builds the vtable of the most derived class).
//! - Jolt's `Init` is `initBroadPhase`, because `init` is the constructor. It returns Allocator.Error: the QuadTree
//!   allocates its trees and commits its node pages there.
//! - The two functions that allocate in BroadPhaseQuadTree return errors even where an implementation cannot fail:
//!   `addBodiesPrepare(bodies) Allocator.Error!AddState` and `updatePrepare() Allocator.Error!UpdateState` (the latter
//!   runs inside the BroadPhasePrepare job, which latches the error, D5). On error nothing changed. Finalize and Abort
//!   never fail, and a zeroed `UpdateState` makes `updateFinalize` a no-op.
//! - Pointer + count parameters are slices (`ioBodies` may be shuffled by the implementation, so they are mutable).
//!   Jolt's defaulted `inTakeLock = true` is an option: `notifyBodiesAABBChanged(bodies, .{ .take_lock = false })`.
//! - `mBodyManager` is `body_manager: ?*BodyManager` (a non-const pointer that const queries read through, Rule M).

const std = @import("std");
const Allocator = std.mem.Allocator;
const virtual = @import("../../../Core/Virtual.zig");
const BodyID = @import("../../Body/BodyID.zig").BodyID;
const BodyPair = @import("../../Body/BodyPair.zig").BodyPair;
const BodyManager = @import("../../Body/BodyManager.zig").BodyManager;
const ObjectLayerPairFilter = @import("../ObjectLayer.zig").ObjectLayerPairFilter;
const ObjectLayerFilter = @import("../ObjectLayer.zig").ObjectLayerFilter;
const BroadPhaseLayerFile = @import("BroadPhaseLayer.zig");
const BroadPhaseLayerInterface = BroadPhaseLayerFile.BroadPhaseLayerInterface;
const BroadPhaseLayerFilter = BroadPhaseLayerFile.BroadPhaseLayerFilter;
const ObjectVsBroadPhaseLayerFilter = BroadPhaseLayerFile.ObjectVsBroadPhaseLayerFilter;
const CollisionCollectorFile = @import("../CollisionCollector.zig");
const AABoxCast = @import("../AABoxCast.zig").AABoxCast;
const BroadPhaseQueryFile = @import("BroadPhaseQuery.zig");
const BroadPhaseQuery = BroadPhaseQueryFile.BroadPhaseQuery;
const CastShapeBodyCollector = BroadPhaseQueryFile.CastShapeBodyCollector;

/// Collector for the body pairs of FindCollidingPairs
pub const BodyPairCollector = CollisionCollectorFile.CollisionCollector(BodyPair, CollisionCollectorFile.CollisionCollectorTraitsCollideShape);

/// Used to do coarse collision detection operations to quickly prune out bodies that will not collide.
pub const BroadPhase = struct {
    /// Context used during broadphase update (`struct UpdateState { void *mData[4]; }`). Zeroed (Jolt's value
    /// initialized `UpdateState()`): nothing to finalize.
    pub const UpdateState = extern struct {
        data: [4]?*anyopaque = @splat(null),
    };

    /// Handle used during adding bodies to the broadphase
    pub const AddState = ?*anyopaque;

    /// The virtual functions that BroadPhase adds, in Jolt's declaration order
    pub const VTable = struct {
        base: BroadPhaseQuery.VTable,

        /// Initialize the broadphase.
        /// @param body_manager The body manager singleton
        /// @param layer_interface Interface that maps object layers to broadphase layers.
        /// Note that the broadphase takes a pointer to the data inside inObjectToBroadPhaseLayer so this object should remain static.
        /// (Init, renamed: `init` is the constructor. Zolt: allocates, Allocator.Error)
        initBroadPhase: *const fn (self: *BroadPhase, body_manager: *BodyManager, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void,

        /// Should be called after many objects have been inserted to make the broadphase more efficient, usually done on startup only
        optimize: *const fn (self: *BroadPhase) void,

        /// Must be called just before updating the broadphase when none of the body mutexes are locked
        frameSync: *const fn (self: *BroadPhase) void,

        /// Must be called before UpdatePrepare to prevent modifications from being made to the tree
        lockModifications: *const fn (self: *BroadPhase) void,

        /// Update the broadphase, needs to be called frequently to update the internal state when bodies have been modified.
        /// The UpdatePrepare() function can run in a background thread without influencing the broadphase
        /// (Zolt: Allocator.Error, nothing changed on error)
        updatePrepare: *const fn (self: *BroadPhase) Allocator.Error!UpdateState,

        /// Finalizing the update will quickly apply the changes
        updateFinalize: *const fn (self: *BroadPhase, update_state: *const UpdateState) void,

        /// Must be called after UpdateFinalize to allow modifications to the broadphase
        unlockModifications: *const fn (self: *BroadPhase) void,

        /// Prepare adding inNumber bodies at ioBodies to the broadphase, returns a handle that should be used in AddBodiesFinalize/Abort.
        /// This can be done on a background thread without influencing the broadphase.
        /// ioBodies may be shuffled around by this function and should be kept that way until AddBodiesFinalize/Abort is called.
        /// (Zolt: Allocator.Error, nothing to abort on error)
        addBodiesPrepare: *const fn (self: *BroadPhase, bodies: []BodyID) Allocator.Error!AddState,

        /// Finalize adding bodies to the broadphase, supply the return value of AddBodiesPrepare in inAddState.
        /// Please ensure that the ioBodies array passed to AddBodiesPrepare is unmodified and passed again to this function.
        addBodiesFinalize: *const fn (self: *BroadPhase, bodies: []BodyID, add_state: AddState) void,

        /// Abort adding bodies to the broadphase, supply the return value of AddBodiesPrepare in inAddState.
        /// This can be done on a background thread without influencing the broadphase.
        /// Please ensure that the ioBodies array passed to AddBodiesPrepare is unmodified and passed again to this function.
        addBodiesAbort: *const fn (self: *BroadPhase, bodies: []BodyID, add_state: AddState) void,

        /// Remove inNumber bodies in ioBodies from the broadphase.
        /// ioBodies may be shuffled around by this function.
        removeBodies: *const fn (self: *BroadPhase, bodies: []BodyID) void,

        /// Call whenever the aabb of a body changes (can change order of ioBodies array)
        /// inTakeLock should be false if we're between LockModifications/UnlockModifications, in which case care needs to be taken to not call this between UpdatePrepare/UpdateFinalize
        notifyBodiesAABBChanged: *const fn (self: *BroadPhase, bodies: []BodyID, take_lock: bool) void,

        /// Call whenever the layer (and optionally the aabb as well) of a body changes (can change order of ioBodies array)
        notifyBodiesLayerChanged: *const fn (self: *BroadPhase, bodies: []BodyID) void,

        /// Find all colliding pairs between dynamic bodies
        /// Note that this function is very specifically tailored for the PhysicsSystem::Update function, hence it is not part of the BroadPhaseQuery interface.
        /// One of the assumptions it can make is that no locking is needed during the query as it will only be called during a very particular part of the update.
        /// @param active_bodies is a list of bodies for which we need to find colliding pairs (this function can change the order of the ioActiveBodies array). This can be a subset of the set of active bodies in the system.
        /// @param speculative_contact_distance Distance at which speculative contact points will be created.
        /// @param object_vs_broad_phase_layer_filter is the filter that determines if an object can collide with a broadphase layer.
        /// @param object_layer_pair_filter is the filter that determines if two objects can collide.
        /// @param pair_collector receives callbacks for every body pair found.
        findCollidingPairs: *const fn (self: *const BroadPhase, active_bodies: []BodyID, speculative_contact_distance: f32, object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, object_layer_pair_filter: *const ObjectLayerPairFilter, pair_collector: *BodyPairCollector) void,

        /// Same as BroadPhaseQuery::CastAABox but can be implemented in a way to take no broad phase locks.
        castAABoxNoLock: *const fn (self: *const BroadPhase, box: *const AABoxCast, collector: *CastShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void,
    };

    base: BroadPhaseQuery,

    /// Link to the body manager that manages the bodies in this broadphase
    body_manager: ?*BodyManager = null,

    /// Constructor of the introducing class: builds the vtable of the most derived class T (`allocator` frees it)
    pub fn init(comptime T: type, allocator: Allocator) BroadPhase {
        return .{ .base = .init(&vtableFor(T).base, allocator) };
    }

    /// The vtable of broad phase class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    fn getVTable(self: *const BroadPhase) *const VTable {
        return virtual.downcast(VTable, self.base.vtable);
    }

    /// The BroadPhaseQuery base class
    pub fn asQuery(self: *const BroadPhase) *const BroadPhaseQuery {
        return &self.base;
    }

    /// The body manager that this broad phase was initialized with (initBroadPhase must have been called)
    pub fn getBodyManager(self: *const BroadPhase) *BodyManager {
        return self.body_manager.?;
    }

    /// Initialize the broadphase (virtual dispatcher, see VTable.initBroadPhase)
    pub fn initBroadPhase(self: *BroadPhase, body_manager: *BodyManager, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void {
        return self.getVTable().initBroadPhase(self, body_manager, layer_interface);
    }

    /// Should be called after many objects have been inserted to make the broadphase more efficient, usually done on startup only
    pub fn optimize(self: *BroadPhase) void {
        self.getVTable().optimize(self);
    }

    /// Must be called just before updating the broadphase when none of the body mutexes are locked
    pub fn frameSync(self: *BroadPhase) void {
        self.getVTable().frameSync(self);
    }

    /// Must be called before UpdatePrepare to prevent modifications from being made to the tree
    pub fn lockModifications(self: *BroadPhase) void {
        self.getVTable().lockModifications(self);
    }

    /// Update the broadphase (see VTable.updatePrepare)
    pub fn updatePrepare(self: *BroadPhase) Allocator.Error!UpdateState {
        return self.getVTable().updatePrepare(self);
    }

    /// Finalizing the update will quickly apply the changes
    pub fn updateFinalize(self: *BroadPhase, update_state: *const UpdateState) void {
        self.getVTable().updateFinalize(self, update_state);
    }

    /// Must be called after UpdateFinalize to allow modifications to the broadphase
    pub fn unlockModifications(self: *BroadPhase) void {
        self.getVTable().unlockModifications(self);
    }

    /// Prepare adding bodies to the broadphase (see VTable.addBodiesPrepare)
    pub fn addBodiesPrepare(self: *BroadPhase, bodies: []BodyID) Allocator.Error!AddState {
        return self.getVTable().addBodiesPrepare(self, bodies);
    }

    /// Finalize adding bodies to the broadphase (see VTable.addBodiesFinalize)
    pub fn addBodiesFinalize(self: *BroadPhase, bodies: []BodyID, add_state: AddState) void {
        self.getVTable().addBodiesFinalize(self, bodies, add_state);
    }

    /// Abort adding bodies to the broadphase (see VTable.addBodiesAbort)
    pub fn addBodiesAbort(self: *BroadPhase, bodies: []BodyID, add_state: AddState) void {
        self.getVTable().addBodiesAbort(self, bodies, add_state);
    }

    /// Remove bodies from the broadphase (see VTable.removeBodies)
    pub fn removeBodies(self: *BroadPhase, bodies: []BodyID) void {
        self.getVTable().removeBodies(self, bodies);
    }

    /// Call whenever the aabb of a body changes (see VTable.notifyBodiesAABBChanged)
    pub fn notifyBodiesAABBChanged(self: *BroadPhase, bodies: []BodyID, opts: struct { take_lock: bool = true }) void {
        self.getVTable().notifyBodiesAABBChanged(self, bodies, opts.take_lock);
    }

    /// Call whenever the layer (and optionally the aabb as well) of a body changes (can change order of ioBodies array)
    pub fn notifyBodiesLayerChanged(self: *BroadPhase, bodies: []BodyID) void {
        self.getVTable().notifyBodiesLayerChanged(self, bodies);
    }

    /// Find all colliding pairs between dynamic bodies (see VTable.findCollidingPairs)
    pub fn findCollidingPairs(self: *const BroadPhase, active_bodies: []BodyID, speculative_contact_distance: f32, object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, object_layer_pair_filter: *const ObjectLayerPairFilter, pair_collector: *BodyPairCollector) void {
        self.getVTable().findCollidingPairs(self, active_bodies, speculative_contact_distance, object_vs_broad_phase_layer_filter, object_layer_pair_filter, pair_collector);
    }

    /// Same as BroadPhaseQuery::CastAABox but can be implemented in a way to take no broad phase locks.
    pub fn castAABoxNoLock(self: *const BroadPhase, box: *const AABoxCast, collector: *CastShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        self.getVTable().castAABoxNoLock(self, box, collector, broad_phase_layer_filter, object_layer_filter);
    }

    /// Jolt's bodies of the optional virtual functions (BroadPhase is abstract)
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
            _ = .{ self, update_state }; // Optionally overridden by implementation
        }

        pub fn unlockModifications(self: *BroadPhase) void {
            _ = self; // Optionally overridden by implementation
        }

        pub fn addBodiesPrepare(self: *BroadPhase, bodies: []BodyID) Allocator.Error!AddState {
            _ = .{ self, bodies };
            return null; // By default the broadphase doesn't support this
        }

        pub fn addBodiesAbort(self: *BroadPhase, bodies: []BodyID, add_state: AddState) void {
            _ = .{ self, bodies, add_state }; // By default nothing needs to be done
        }
    };
};
