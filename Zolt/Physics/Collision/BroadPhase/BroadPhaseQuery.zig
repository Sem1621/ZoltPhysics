//! Port of: Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h
//! Status: complete
//!
//! SimulationArchitecture D9: the root of the broad phase hierarchy BroadPhaseQuery -> BroadPhase ->
//! BroadPhaseBruteForce / BroadPhaseQuadTree (porting guide pattern A, `virtual.make`):
//! - Every query is pure virtual and const. Implementations lock their C++ `mutable` mutexes through a
//!   `mutable_fields` heap block (Rule M), so the queries keep `*const` receivers.
//! - Jolt's defaulted filter arguments (`= { }`) are an options struct:
//!   `query.collideAABox(&box, &collector.base, .{ .object_layer_filter = &filter.base })`.
//! - The root is not reference counted. It holds the `allocator` that frees the object: PhysicsSystem owns its
//!   broad phase through a plain pointer and `delete mBroadPhase` is `broad_phase.base.destroy()` (the generated
//!   `destroy` runs the destructor chain and frees the most derived object).
//! - NonCopyable in Jolt: a broad phase is only used through pointers.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const AABox = @import("../../../Geometry/AABox.zig").AABox;
const OrientedBox = @import("../../../Geometry/OrientedBox.zig").OrientedBox;
const BodyID = @import("../../Body/BodyID.zig").BodyID;
const RayCast = @import("../RayCast.zig").RayCast;
const AABoxCast = @import("../AABoxCast.zig").AABoxCast;
const BroadPhaseCastResult = @import("../CastResult.zig").BroadPhaseCastResult;
const CollisionCollectorFile = @import("../CollisionCollector.zig");
const CollisionCollector = CollisionCollectorFile.CollisionCollector;
const ObjectLayerFilter = @import("../ObjectLayer.zig").ObjectLayerFilter;
const BroadPhaseLayerFilter = @import("BroadPhaseLayer.zig").BroadPhaseLayerFilter;

// Various collector configurations
pub const RayCastBodyCollector = CollisionCollector(BroadPhaseCastResult, CollisionCollectorFile.CollisionCollectorTraitsCastRay);
pub const CastShapeBodyCollector = CollisionCollector(BroadPhaseCastResult, CollisionCollectorFile.CollisionCollectorTraitsCastShape);
pub const CollideShapeBodyCollector = CollisionCollector(BodyID, CollisionCollectorFile.CollisionCollectorTraitsCollideShape);

/// The defaulted filter arguments of the queries (`inBroadPhaseLayerFilter = { }`, `inObjectLayerFilter = { }`)
pub const QueryFilters = struct {
    broad_phase_layer_filter: *const BroadPhaseLayerFilter = &.{},
    object_layer_filter: *const ObjectLayerFilter = &.{},
};

/// Interface to the broadphase that can perform collision queries. These queries will only test the bounding box of the body to quickly determine a potential set of colliding bodies.
/// The shapes of the bodies are not tested, if you want this then you should use the NarrowPhaseQuery interface.
pub const BroadPhaseQuery = struct {
    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *BroadPhaseQuery) void,
        /// delete this (generated, frees the object with `allocator`)
        destroy: *const fn (self: *BroadPhaseQuery) void,

        /// Cast a ray and add any hits to ioCollector
        castRay: *const fn (self: *const BroadPhaseQuery, ray: *const RayCast, collector: *RayCastBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void,

        /// Get bodies intersecting with inBox and any hits to ioCollector
        collideAABox: *const fn (self: *const BroadPhaseQuery, box: *const AABox, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void,

        /// Get bodies intersecting with a sphere and any hits to ioCollector
        collideSphere: *const fn (self: *const BroadPhaseQuery, center: Vec3, radius: f32, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void,

        /// Get bodies intersecting with a point and any hits to ioCollector
        collidePoint: *const fn (self: *const BroadPhaseQuery, point: Vec3, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void,

        /// Get bodies intersecting with an oriented box and any hits to ioCollector
        collideOrientedBox: *const fn (self: *const BroadPhaseQuery, box: *const OrientedBox, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void,

        /// Cast a box and add any hits to ioCollector
        castAABox: *const fn (self: *const BroadPhaseQuery, box: *const AABoxCast, collector: *CastShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void,

        /// Get the bounding box of all objects in the broadphase
        getBounds: *const fn (self: *const BroadPhaseQuery) AABox,
    };

    vtable: *const VTable,

    /// Zolt: frees the most derived object (Jolt's global delete) and the memory it owns
    allocator: Allocator,

    /// Constructor of the root, called by BroadPhase.init with the vtable of the most derived class
    pub fn init(vtable: *const VTable, allocator: Allocator) BroadPhaseQuery {
        return .{ .vtable = vtable, .allocator = allocator };
    }

    /// Virtual destructor of an object that is not on the heap
    pub fn deinit(self: *BroadPhaseQuery) void {
        self.vtable.deinit(self);
    }

    /// delete this: runs the destructor chain and frees the object
    pub fn destroy(self: *BroadPhaseQuery) void {
        self.vtable.destroy(self);
    }

    /// Cast a ray and add any hits to ioCollector
    pub fn castRay(self: *const BroadPhaseQuery, ray: *const RayCast, collector: *RayCastBodyCollector, opts: QueryFilters) void {
        self.vtable.castRay(self, ray, collector, opts.broad_phase_layer_filter, opts.object_layer_filter);
    }

    /// Get bodies intersecting with inBox and any hits to ioCollector
    pub fn collideAABox(self: *const BroadPhaseQuery, box: *const AABox, collector: *CollideShapeBodyCollector, opts: QueryFilters) void {
        self.vtable.collideAABox(self, box, collector, opts.broad_phase_layer_filter, opts.object_layer_filter);
    }

    /// Get bodies intersecting with a sphere and any hits to ioCollector
    pub fn collideSphere(self: *const BroadPhaseQuery, center: Vec3, radius: f32, collector: *CollideShapeBodyCollector, opts: QueryFilters) void {
        self.vtable.collideSphere(self, center, radius, collector, opts.broad_phase_layer_filter, opts.object_layer_filter);
    }

    /// Get bodies intersecting with a point and any hits to ioCollector
    pub fn collidePoint(self: *const BroadPhaseQuery, point: Vec3, collector: *CollideShapeBodyCollector, opts: QueryFilters) void {
        self.vtable.collidePoint(self, point, collector, opts.broad_phase_layer_filter, opts.object_layer_filter);
    }

    /// Get bodies intersecting with an oriented box and any hits to ioCollector
    pub fn collideOrientedBox(self: *const BroadPhaseQuery, box: *const OrientedBox, collector: *CollideShapeBodyCollector, opts: QueryFilters) void {
        self.vtable.collideOrientedBox(self, box, collector, opts.broad_phase_layer_filter, opts.object_layer_filter);
    }

    /// Cast a box and add any hits to ioCollector
    pub fn castAABox(self: *const BroadPhaseQuery, box: *const AABoxCast, collector: *CastShapeBodyCollector, opts: QueryFilters) void {
        self.vtable.castAABox(self, box, collector, opts.broad_phase_layer_filter, opts.object_layer_filter);
    }

    /// Get the bounding box of all objects in the broadphase
    pub fn getBounds(self: *const BroadPhaseQuery) AABox {
        return self.vtable.getBounds(self);
    }

    /// Every query is pure virtual
    pub const impl = struct {};
};
