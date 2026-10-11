//! Port of: Jolt/Physics/Collision/NarrowPhaseQuery.h, Jolt/Physics/Collision/NarrowPhaseQuery.cpp
//! Status: complete
//!
//! - PhysicsSystem holds a locking and a non locking NarrowPhaseQuery (Jolt: mNarrowPhaseQueryLocking /
//!   mNarrowPhaseQueryNoLock). Jolt's `Init(BodyLockInterface &, BroadPhaseQuery &)` sets the two pointers of a default
//!   constructed object: Zolt's `init` returns the initialized value (`.empty` is the default constructed one).
//! - Every query is const (Rule M): the body lock interface and the broad phase query are `*const`, the broad phase
//!   query locks its C++ `mutable` mutexes through its `mutable_fields`.
//! - The queries pass a body level collector (`RayCastBodyCollector`, `CollideShapeBodyCollector`,
//!   `CastShapeBodyCollector`) to the broad phase. Like Jolt's local `MyCollector` classes, they are local structs that
//!   embed the base collector (`overrides = .{.addHit}`, CollisionArchitecture D5). For every body that the broad phase
//!   finds they check the body filter, lock the body (it may have been removed from the broad phase since it was found),
//!   check the body filter again with the locked body, take the TransformedShape, release the lock and run the narrow
//!   phase query on the TransformedShape. The wrapping collectors start from the early out fraction and the context of
//!   the user's collector (`initFrom`, Jolt's `RayCastBodyCollector(ioCollector)`), call `onBody` / `onBodyEnd` of the
//!   user's collector around the narrow phase query and follow its early out fraction (`updateEarlyOutFraction`). The
//!   single hit CastRay starts its collector at the fraction of the hit (`resetEarlyOutFraction`).
//! - Jolt's defaulted filter arguments (`= { }`) are options structs. Jolt writes `mBodyID2` (a `mutable` member) of the
//!   shape filter through its `const ShapeFilter &` in the TransformedShape queries, so the queries that forward the
//!   shape filter to a TransformedShape query that writes it take `shape_filter: ?*ShapeFilter = null` and use a local
//!   default filter when none is given (Rule M, CollisionArchitecture D7). CollectTransformedShapes does not write it and
//!   takes `*const ShapeFilter`.
//! - CollideShapeWithInternalEdgeRemoval wraps the collector in an InternalEdgeRemovingCollector, whose arrays fall back
//!   to the heap (Jolt: an STLLocalAllocator over its global heap). It takes the `allocator` of that fallback and
//!   returns error.OutOfMemory when a hit could not be stored, like `InternalEdgeRemovingCollector.collideShapeVsShape`
//!   (CollisionArchitecture section 9, the exception to D13). The other queries never allocate: results go to the
//!   user's collector, which owns its allocator and reports its own allocation failures (D6).
//! - Jolt's `JPH_ASSERT`s on the hits that the broad phase passes on (`This hit should not have been passed on to the
//!   collector`) only run with asserts enabled: degenerate input (NaN rays) can violate them and Jolt's release build
//!   continues.
//!
//! The body level collectors are called from the thread that runs the query. A query can run concurrently with other
//! queries and with bodies being added / removed / modified through the locking BodyInterface (the broad phase query
//! takes its query lock, the bodies are locked one at a time through the BodyLockInterface).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const RealFile = @import("../../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const BodyFilter = @import("../Body/BodyFilter.zig").BodyFilter;
const BodyLockFile = @import("../Body/BodyLock.zig");
const BodyLockRead = BodyLockFile.BodyLockRead;
const BodyLockInterface = @import("../Body/BodyLockInterface.zig").BodyLockInterface;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const ShapeFilter = @import("ShapeFilter.zig").ShapeFilter;
const BroadPhaseQueryFile = @import("BroadPhase/BroadPhaseQuery.zig");
const BroadPhaseQuery = BroadPhaseQueryFile.BroadPhaseQuery;
const RayCastBodyCollector = BroadPhaseQueryFile.RayCastBodyCollector;
const CastShapeBodyCollector = BroadPhaseQueryFile.CastShapeBodyCollector;
const CollideShapeBodyCollector = BroadPhaseQueryFile.CollideShapeBodyCollector;
const BroadPhaseLayerFilter = @import("BroadPhase/BroadPhaseLayer.zig").BroadPhaseLayerFilter;
const ObjectLayerFilter = @import("ObjectLayer.zig").ObjectLayerFilter;
const ShapeFile = @import("Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const CastRayCollector = ShapeFile.CastRayCollector;
const CastShapeCollector = ShapeFile.CastShapeCollector;
const CollidePointCollector = ShapeFile.CollidePointCollector;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const TransformedShapeCollector = ShapeFile.TransformedShapeCollector;
const RayCastFile = @import("RayCast.zig");
const RRayCast = RayCastFile.RRayCast;
const RayCastSettings = RayCastFile.RayCastSettings;
const CastResultFile = @import("CastResult.zig");
const RayCastResult = CastResultFile.RayCastResult;
const BroadPhaseCastResult = CastResultFile.BroadPhaseCastResult;
const ShapeCastFile = @import("ShapeCast.zig");
const RShapeCast = ShapeCastFile.RShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const CollideShapeSettings = @import("CollideShape.zig").CollideShapeSettings;
const InternalEdgeRemovingCollector = @import("InternalEdgeRemovingCollector.zig").InternalEdgeRemovingCollector;

/// Class that provides an interface for doing precise collision detection against the broad and then the narrow phase.
/// Unlike a BroadPhaseQuery, the NarrowPhaseQuery will test against shapes and will return collision information against triangles, spheres etc.
pub const NarrowPhaseQuery = struct {
    body_lock_interface: ?*const BodyLockInterface = null,
    broad_phase_query: ?*const BroadPhaseQuery = null,

    /// Not initialized (Jolt's constructor)
    pub const empty: NarrowPhaseQuery = .{};

    /// The defaulted filter arguments of the single hit CastRay (`= { }` in Jolt: accept everything)
    pub const RayFilters = struct {
        /// Filter that filters at broadphase level
        broad_phase_layer_filter: *const BroadPhaseLayerFilter = &.{},
        /// Filter that filters at layer level
        object_layer_filter: *const ObjectLayerFilter = &.{},
        /// Filter that filters at body level
        body_filter: *const BodyFilter = &.{},
    };

    /// The defaulted filter arguments of the queries that pass the shape filter on to a TransformedShape query, which
    /// sets the body ID of the filter before calling it (`= { }` in Jolt: accept everything)
    pub const QueryFilters = struct {
        /// Filter that filters at broadphase level
        broad_phase_layer_filter: *const BroadPhaseLayerFilter = &.{},
        /// Filter that filters at layer level
        object_layer_filter: *const ObjectLayerFilter = &.{},
        /// Filter that filters at body level
        body_filter: *const BodyFilter = &.{},
        /// Filter that filters at shape level (Jolt's `const ShapeFilter &` whose `mutable mBodyID2` is set per body:
        /// mutable here, Rule M; null is Jolt's default filter)
        shape_filter: ?*ShapeFilter = null,
    };

    /// The defaulted filter arguments of CollectTransformedShapes (the shape filter is only read)
    pub const CollectFilters = struct {
        /// Filter that filters at broadphase level
        broad_phase_layer_filter: *const BroadPhaseLayerFilter = &.{},
        /// Filter that filters at layer level
        object_layer_filter: *const ObjectLayerFilter = &.{},
        /// Filter that filters at body level
        body_filter: *const BodyFilter = &.{},
        /// Filter that filters at shape level
        shape_filter: *const ShapeFilter = &.{},
    };

    /// Initialize the interface (should only be called by PhysicsSystem)
    pub fn init(body_lock_interface: *const BodyLockInterface, broad_phase_query: *const BroadPhaseQuery) NarrowPhaseQuery {
        return .{ .body_lock_interface = body_lock_interface, .broad_phase_query = broad_phase_query };
    }

    /// Cast a ray and find the closest hit. Returns true if it finds a hit. Hits further than hit.fraction will not be considered and in this case hit will remain unmodified (and the function will return false).
    /// Convex objects will be treated as solid (meaning if the ray starts inside, you'll get a hit fraction of 0) and back face hits against triangles are returned.
    /// If you want the surface normal of the hit use Body::GetWorldSpaceSurfaceNormal(hit.sub_shape_id2, ray.getPointOnRay(hit.fraction)) on body with ID hit.body_id.
    pub fn castRay(self: *const NarrowPhaseQuery, ray: RRayCast, hit: *RayCastResult, opts: RayFilters) bool {
        const MyCollector = struct {
            pub const overrides = .{.addHit};

            base: RayCastBodyCollector,
            ray: RRayCast,
            hit: *RayCastResult,
            body_lock_interface: *const BodyLockInterface,
            body_filter: *const BodyFilter,

            fn init(r: RRayCast, h: *RayCastResult, body_lock_interface: *const BodyLockInterface, body_filter: *const BodyFilter) @This() {
                var c: @This() = .{ .base = .init(@This()), .ray = r, .hit = h, .body_lock_interface = body_lock_interface, .body_filter = body_filter };
                c.base.resetEarlyOutFraction(.{ .fraction = h.fraction });
                return c;
            }

            pub fn addHit(c: *@This(), result: *const BroadPhaseCastResult) void {
                if (Core.enable_asserts) std.debug.assert(result.fraction < c.hit.fraction); // This hit should not have been passed on to the collector

                // Only test shape if it passes the body filter
                if (c.body_filter.shouldCollide(result.body_id)) {
                    // Lock the body
                    var lock: BodyLockRead = .init(c.body_lock_interface, result.body_id);
                    defer lock.deinit();
                    if (lock.succeededAndIsInBroadPhase()) { // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
                        const body = lock.getBody();

                        // Check body filter again now that we've locked the body
                        if (c.body_filter.shouldCollideLocked(body)) {
                            // Collect the transformed shape
                            var ts = body.getTransformedShape();
                            defer ts.deinit();

                            // Release the lock now, we have all the info we need in the transformed shape
                            lock.releaseLock();

                            // Do narrow phase collision check
                            if (ts.castRay(c.ray, c.hit)) {
                                // Test that we didn't find a further hit by accident
                                if (Core.enable_asserts) std.debug.assert(c.hit.fraction >= 0.0 and c.hit.fraction < c.base.getEarlyOutFraction());

                                // Update early out fraction based on narrow phase collector
                                c.base.updateEarlyOutFraction(c.hit.fraction);
                            }
                        }
                    }
                }
            }
        };

        // Do broadphase test, note that the broadphase uses floats so we drop precision here
        var collector: MyCollector = .init(ray, hit, self.body_lock_interface.?, opts.body_filter);
        self.broad_phase_query.?.castRay(&ray.toRayCast(), &collector.base, .{ .broad_phase_layer_filter = opts.broad_phase_layer_filter, .object_layer_filter = opts.object_layer_filter });
        return hit.fraction <= 1.0;
    }

    /// Cast a ray, allows collecting multiple hits. Note that this version is more flexible but also slightly slower than the CastRay function that returns only a single hit.
    /// If you want the surface normal of the hit use Body::GetWorldSpaceSurfaceNormal(collected sub shape ID, ray.getPointOnRay(collected fraction)) on body with collected body ID.
    /// (Jolt's CastRay overload with a collector)
    pub fn castRayCollector(self: *const NarrowPhaseQuery, ray: RRayCast, ray_cast_settings: *const RayCastSettings, collector: *CastRayCollector, opts: QueryFilters) void {
        const MyCollector = struct {
            pub const overrides = .{.addHit};

            base: RayCastBodyCollector,
            ray: RRayCast,
            ray_cast_settings: RayCastSettings,
            collector: *CastRayCollector,
            body_lock_interface: *const BodyLockInterface,
            body_filter: *const BodyFilter,
            shape_filter: *ShapeFilter,

            pub fn addHit(c: *@This(), result: *const BroadPhaseCastResult) void {
                if (Core.enable_asserts) std.debug.assert(result.fraction < c.collector.getEarlyOutFraction()); // This hit should not have been passed on to the collector

                // Only test shape if it passes the body filter
                if (c.body_filter.shouldCollide(result.body_id)) {
                    // Lock the body
                    var lock: BodyLockRead = .init(c.body_lock_interface, result.body_id);
                    defer lock.deinit();
                    if (lock.succeededAndIsInBroadPhase()) { // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
                        const body = lock.getBody();

                        // Check body filter again now that we've locked the body
                        if (c.body_filter.shouldCollideLocked(body)) {
                            // Collect the transformed shape
                            var ts = body.getTransformedShape();
                            defer ts.deinit();

                            // Notify collector of new body
                            c.collector.onBody(body);

                            // Release the lock now, we have all the info we need in the transformed shape
                            lock.releaseLock();

                            // Do narrow phase collision check
                            ts.castRayCollector(c.ray, &c.ray_cast_settings, c.collector, .{ .shape_filter = c.shape_filter });

                            // Notify collector of the end of this body
                            // We do this before updating the early out fraction so that the collector can still modify it
                            c.collector.onBodyEnd();

                            // Update early out fraction based on narrow phase collector
                            c.base.updateEarlyOutFraction(c.collector.getEarlyOutFraction());
                        }
                    }
                }
            }
        };

        var default_shape_filter: ShapeFilter = .{};
        const shape_filter = opts.shape_filter orelse &default_shape_filter;

        // Do broadphase test, note that the broadphase uses floats so we drop precision here
        var my_collector: MyCollector = .{ .base = .initFrom(MyCollector, collector), .ray = ray, .ray_cast_settings = ray_cast_settings.*, .collector = collector, .body_lock_interface = self.body_lock_interface.?, .body_filter = opts.body_filter, .shape_filter = shape_filter };
        self.broad_phase_query.?.castRay(&ray.toRayCast(), &my_collector.base, .{ .broad_phase_layer_filter = opts.broad_phase_layer_filter, .object_layer_filter = opts.object_layer_filter });
    }

    /// Check if point is inside any shapes. For this tests all shapes are treated as if they were solid.
    /// For a mesh shape, this test will only provide sensible information if the mesh is a closed manifold.
    /// For each shape that collides, collector will receive a hit
    pub fn collidePoint(self: *const NarrowPhaseQuery, point: RVec3, collector: *CollidePointCollector, opts: QueryFilters) void {
        const MyCollector = struct {
            pub const overrides = .{.addHit};

            base: CollideShapeBodyCollector,
            point: RVec3,
            collector: *CollidePointCollector,
            body_lock_interface: *const BodyLockInterface,
            body_filter: *const BodyFilter,
            shape_filter: *ShapeFilter,

            pub fn addHit(c: *@This(), result: *const BodyID) void {
                // Only test shape if it passes the body filter
                if (c.body_filter.shouldCollide(result.*)) {
                    // Lock the body
                    var lock: BodyLockRead = .init(c.body_lock_interface, result.*);
                    defer lock.deinit();
                    if (lock.succeededAndIsInBroadPhase()) { // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
                        const body = lock.getBody();

                        // Check body filter again now that we've locked the body
                        if (c.body_filter.shouldCollideLocked(body)) {
                            // Collect the transformed shape
                            var ts = body.getTransformedShape();
                            defer ts.deinit();

                            // Notify collector of new body
                            c.collector.onBody(body);

                            // Release the lock now, we have all the info we need in the transformed shape
                            lock.releaseLock();

                            // Do narrow phase collision check
                            ts.collidePoint(c.point, c.collector, .{ .shape_filter = c.shape_filter });

                            // Notify collector of the end of this body
                            // We do this before updating the early out fraction so that the collector can still modify it
                            c.collector.onBodyEnd();

                            // Update early out fraction based on narrow phase collector
                            c.base.updateEarlyOutFraction(c.collector.getEarlyOutFraction());
                        }
                    }
                }
            }
        };

        var default_shape_filter: ShapeFilter = .{};
        const shape_filter = opts.shape_filter orelse &default_shape_filter;

        // Do broadphase test (note: truncates double to single precision since the broadphase uses single precision)
        var my_collector: MyCollector = .{ .base = .initFrom(MyCollector, collector), .point = point, .collector = collector, .body_lock_interface = self.body_lock_interface.?, .body_filter = opts.body_filter, .shape_filter = shape_filter };
        self.broad_phase_query.?.collidePoint(point.toVec3(), &my_collector.base, .{ .broad_phase_layer_filter = opts.broad_phase_layer_filter, .object_layer_filter = opts.object_layer_filter });
    }

    /// Collide a shape with the system
    /// @param shape Shape to test
    /// @param shape_scale Scale in local space of shape
    /// @param center_of_mass_transform Center of mass transform for the shape
    /// @param collide_shape_settings Settings
    /// @param base_offset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. center_of_mass_transform.getTranslation() since floats are most accurate near the origin
    /// @param collector Collector that receives the hits
    /// @param opts.broad_phase_layer_filter Filter that filters at broadphase level
    /// @param opts.object_layer_filter Filter that filters at layer level
    /// @param opts.body_filter Filter that filters at body level
    /// @param opts.shape_filter Filter that filters at shape level
    pub fn collideShape(self: *const NarrowPhaseQuery, shape: *const Shape, shape_scale: Vec3, center_of_mass_transform: RMat44, collide_shape_settings: *const CollideShapeSettings, base_offset: RVec3, collector: *CollideShapeCollector, opts: QueryFilters) void {
        const MyCollector = struct {
            pub const overrides = .{.addHit};

            base: CollideShapeBodyCollector,
            shape: *const Shape,
            shape_scale: Vec3,
            center_of_mass_transform: RMat44,
            collide_shape_settings: *const CollideShapeSettings,
            base_offset: RVec3,
            collector: *CollideShapeCollector,
            body_lock_interface: *const BodyLockInterface,
            body_filter: *const BodyFilter,
            shape_filter: *ShapeFilter,

            pub fn addHit(c: *@This(), result: *const BodyID) void {
                // Only test shape if it passes the body filter
                if (c.body_filter.shouldCollide(result.*)) {
                    // Lock the body
                    var lock: BodyLockRead = .init(c.body_lock_interface, result.*);
                    defer lock.deinit();
                    if (lock.succeededAndIsInBroadPhase()) { // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
                        const body = lock.getBody();

                        // Check body filter again now that we've locked the body
                        if (c.body_filter.shouldCollideLocked(body)) {
                            // Collect the transformed shape
                            var ts = body.getTransformedShape();
                            defer ts.deinit();

                            // Notify collector of new body
                            c.collector.onBody(body);

                            // Release the lock now, we have all the info we need in the transformed shape
                            lock.releaseLock();

                            // Do narrow phase collision check
                            ts.collideShape(c.shape, c.shape_scale, c.center_of_mass_transform, c.collide_shape_settings, c.base_offset, c.collector, .{ .shape_filter = c.shape_filter });

                            // Notify collector of the end of this body
                            // We do this before updating the early out fraction so that the collector can still modify it
                            c.collector.onBodyEnd();

                            // Update early out fraction based on narrow phase collector
                            c.base.updateEarlyOutFraction(c.collector.getEarlyOutFraction());
                        }
                    }
                }
            }
        };

        // Calculate bounds for shape and expand by max separation distance
        var bounds = shape.getWorldSpaceBoundsRMat44(center_of_mass_transform, shape_scale);
        bounds.expandBy(Vec3.replicate(collide_shape_settings.max_separation_distance));

        var default_shape_filter: ShapeFilter = .{};
        const shape_filter = opts.shape_filter orelse &default_shape_filter;

        // Do broadphase test
        var my_collector: MyCollector = .{
            .base = .initFrom(MyCollector, collector),
            .shape = shape,
            .shape_scale = shape_scale,
            .center_of_mass_transform = center_of_mass_transform,
            .collide_shape_settings = collide_shape_settings,
            .base_offset = base_offset,
            .collector = collector,
            .body_lock_interface = self.body_lock_interface.?,
            .body_filter = opts.body_filter,
            .shape_filter = shape_filter,
        };
        self.broad_phase_query.?.collideAABox(&bounds, &my_collector.base, .{ .broad_phase_layer_filter = opts.broad_phase_layer_filter, .object_layer_filter = opts.object_layer_filter });
    }

    /// Same as CollideShape, but uses InternalEdgeRemovingCollector to remove internal edges from the collision results (a.k.a. ghost collisions)
    /// `allocator` is the heap that the InternalEdgeRemovingCollector falls back to when its local buffers are full. Returns
    /// error.OutOfMemory when a result could not be stored (the query stopped early, `collector` has a partial result).
    pub fn collideShapeWithInternalEdgeRemoval(self: *const NarrowPhaseQuery, allocator: Allocator, shape: *const Shape, shape_scale: Vec3, center_of_mass_transform: RMat44, collide_shape_settings: *const CollideShapeSettings, base_offset: RVec3, collector: *CollideShapeCollector, opts: QueryFilters) Allocator.Error!void {
        // We require these settings for internal edge removal to work
        var settings = collide_shape_settings.*;
        settings.active_edge_mode = .collide_with_all;
        settings.collect_faces_mode = .collect_faces;

        var wrapper: InternalEdgeRemovingCollector = undefined;
        wrapper.init(collector, settings.internal_edge_removal_vertex_tolerance_sq, allocator);
        defer wrapper.deinit();
        self.collideShape(shape, shape_scale, center_of_mass_transform, &settings, base_offset, &wrapper.base, opts);
        try wrapper.checkError();
    }

    /// Cast a shape and report any hits to collector
    /// @param shape_cast The shape cast and its position and direction
    /// @param shape_cast_settings Settings for the shape cast
    /// @param base_offset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. shape_cast.center_of_mass_start.getTranslation() since floats are most accurate near the origin
    /// @param collector Collector that receives the hits
    /// @param opts.broad_phase_layer_filter Filter that filters at broadphase level
    /// @param opts.object_layer_filter Filter that filters at layer level
    /// @param opts.body_filter Filter that filters at body level
    /// @param opts.shape_filter Filter that filters at shape level
    pub fn castShape(self: *const NarrowPhaseQuery, shape_cast: *const RShapeCast, shape_cast_settings: *const ShapeCastSettings, base_offset: RVec3, collector: *CastShapeCollector, opts: QueryFilters) void {
        const MyCollector = struct {
            pub const overrides = .{.addHit};

            base: CastShapeBodyCollector,
            shape_cast: RShapeCast,
            shape_cast_settings: *const ShapeCastSettings,
            base_offset: RVec3,
            collector: *CastShapeCollector,
            body_lock_interface: *const BodyLockInterface,
            body_filter: *const BodyFilter,
            shape_filter: *ShapeFilter,

            pub fn addHit(c: *@This(), result: *const BroadPhaseCastResult) void {
                if (Core.enable_asserts) std.debug.assert(result.fraction <= math.max(@as(f32, 0.0), c.collector.getEarlyOutFraction())); // This hit should not have been passed on to the collector

                // Only test shape if it passes the body filter
                if (c.body_filter.shouldCollide(result.body_id)) {
                    // Lock the body
                    var lock: BodyLockRead = .init(c.body_lock_interface, result.body_id);
                    defer lock.deinit();
                    if (lock.succeededAndIsInBroadPhase()) { // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
                        const body = lock.getBody();

                        // Check body filter again now that we've locked the body
                        if (c.body_filter.shouldCollideLocked(body)) {
                            // Collect the transformed shape
                            var ts = body.getTransformedShape();
                            defer ts.deinit();

                            // Notify collector of new body
                            c.collector.onBody(body);

                            // Release the lock now, we have all the info we need in the transformed shape
                            lock.releaseLock();

                            // Do narrow phase collision check
                            ts.castShape(&c.shape_cast, c.shape_cast_settings, c.base_offset, c.collector, .{ .shape_filter = c.shape_filter });

                            // Notify collector of the end of this body
                            // We do this before updating the early out fraction so that the collector can still modify it
                            c.collector.onBodyEnd();

                            // Update early out fraction based on narrow phase collector
                            c.base.updateEarlyOutFraction(c.collector.getEarlyOutFraction());
                        }
                    }
                }
            }
        };

        // Calculate bounds for shape and expand by extra convex radius
        var bounds = shape_cast.shape_world_bounds;
        bounds.expandBy(Vec3.replicate(shape_cast_settings.extra_convex_radius));

        var default_shape_filter: ShapeFilter = .{};
        const shape_filter = opts.shape_filter orelse &default_shape_filter;

        // Do broadphase test
        var my_collector: MyCollector = .{
            .base = .initFrom(MyCollector, collector),
            .shape_cast = shape_cast.*,
            .shape_cast_settings = shape_cast_settings,
            .base_offset = base_offset,
            .collector = collector,
            .body_lock_interface = self.body_lock_interface.?,
            .body_filter = opts.body_filter,
            .shape_filter = shape_filter,
        };
        self.broad_phase_query.?.castAABox(&.{ .box = bounds, .direction = shape_cast.direction }, &my_collector.base, .{ .broad_phase_layer_filter = opts.broad_phase_layer_filter, .object_layer_filter = opts.object_layer_filter });
    }

    /// Collect all leaf transformed shapes that fall inside world space box `box`
    pub fn collectTransformedShapes(self: *const NarrowPhaseQuery, box: AABox, collector: *TransformedShapeCollector, opts: CollectFilters) void {
        const MyCollector = struct {
            pub const overrides = .{.addHit};

            base: CollideShapeBodyCollector,
            box: *const AABox,
            collector: *TransformedShapeCollector,
            body_lock_interface: *const BodyLockInterface,
            body_filter: *const BodyFilter,
            shape_filter: *const ShapeFilter,

            pub fn addHit(c: *@This(), result: *const BodyID) void {
                // Only test shape if it passes the body filter
                if (c.body_filter.shouldCollide(result.*)) {
                    // Lock the body
                    var lock: BodyLockRead = .init(c.body_lock_interface, result.*);
                    defer lock.deinit();
                    if (lock.succeededAndIsInBroadPhase()) { // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
                        const body = lock.getBody();

                        // Check body filter again now that we've locked the body
                        if (c.body_filter.shouldCollideLocked(body)) {
                            // Collect the transformed shape
                            var ts = body.getTransformedShape();
                            defer ts.deinit();

                            // Notify collector of new body
                            c.collector.onBody(body);

                            // Release the lock now, we have all the info we need in the transformed shape
                            lock.releaseLock();

                            // Do narrow phase collision check
                            ts.collectTransformedShapes(c.box.*, c.collector, .{ .shape_filter = c.shape_filter });

                            // Notify collector of the end of this body
                            // We do this before updating the early out fraction so that the collector can still modify it
                            c.collector.onBodyEnd();

                            // Update early out fraction based on narrow phase collector
                            c.base.updateEarlyOutFraction(c.collector.getEarlyOutFraction());
                        }
                    }
                }
            }
        };

        // Do broadphase test
        var my_collector: MyCollector = .{ .base = .initFrom(MyCollector, collector), .box = &box, .collector = collector, .body_lock_interface = self.body_lock_interface.?, .body_filter = opts.body_filter, .shape_filter = opts.shape_filter };
        self.broad_phase_query.?.collideAABox(&box, &my_collector.base, .{ .broad_phase_layer_filter = opts.broad_phase_layer_filter, .object_layer_filter = opts.object_layer_filter });
    }
};
