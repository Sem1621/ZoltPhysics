//! Port of: Jolt/Physics/Collision/InternalEdgeRemovingCollector.h
//! Status: complete
//! Not ported: JPH_INTERNAL_EDGE_REMOVING_COLLECTOR_DEBUG (the drawing of the results and the `inBaseOffset`
//!   parameters that only exist for it)
//!
//! A wrapping collector (Docs/Zolt/CollisionArchitecture.md, D5): it embeds `base: CollideShapeCollector`, built with
//! `initFrom` (Jolt's `CollideShapeCollector(inChainedCollector)` copies the early out fraction and the context), and
//! declares the overrides of `Reset`, `OnBody`, `AddHit` and `OnBodyEnd`.
//!
//! Allocation (D6 / D13): Jolt keeps the voided features and the delayed results in `Array`s with an
//! `STLLocalAllocator` (128 and 32 elements in the collector itself, the heap beyond that), and sorts the delayed
//! results through another local array (32 elements). Zolt does the same with `STLLocalAllocator`s whose heap is the
//! `allocator` passed to `init` (Jolt uses its global heap). The local buffers are taken with exact reservations
//! (`ensureTotalCapacityPrecise`, like Jolt's `reserve` and `resize`; `std.ArrayList.resize` over-allocates and would
//! miss the local buffer), so up to 128 voided features, 32 delayed results and their 32 sort indices never touch the
//! heap. The collector contains these buffers, so it is
//! constructed in place (`var c: InternalEdgeRemovingCollector = undefined; c.init(...)`) and must not be moved; it
//! is about 37 KB, like the C++ object. Like the collectors of CollisionCollectorImpl.zig, an allocation failure is
//! recorded in an `AllocationErrorLatch`, forces an early out, and is reported by `checkError()`; the hits that are
//! added after it are dropped and `flush` only clears the arrays (no more hits reach the chained collector, whose
//! early out fraction the forced early out no longer follows). `deinit` frees the heap memory.
//!
//! `sCollideShapeVsShape` is `collideShapeVsShape(allocator, ...)`, which returns error.OutOfMemory when the wrapper
//! could not store a hit (Jolt's default argument `inShapeFilter = { }` is a required parameter, pass `&.{}`).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const quickSort = @import("../../Core/QuickSort.zig").quickSort;
const STLLocalAllocator = @import("../../Core/STLLocalAllocator.zig").STLLocalAllocator;
const math = @import("../../Math/Math.zig");
const Float3 = @import("../../Math/Float3.zig").Float3;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Body = @import("../Body/Body.zig").Body;
const ShapeFile = @import("Shape/Shape.zig");
const Shape = ShapeFile.Shape;
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const SubShapeIDFile = @import("Shape/SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const CollideShapeFile = @import("CollideShape.zig");
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const CollisionDispatch = @import("CollisionDispatch.zig");
const AllocationErrorLatch = @import("CollisionCollectorImpl.zig").AllocationErrorLatch;
const ShapeFilter = @import("ShapeFilter.zig").ShapeFilter;

/// Removes internal edges from collision results. Can be used to filter out 'ghost collisions'.
/// Based on: Contact generation for meshes - Pierre Terdiman (https://www.codercorner.com/MeshContacts.pdf)
///
/// Note that this class requires that CollideSettingsBase::mActiveEdgeMode == EActiveEdgeMode::CollideWithAll
/// and CollideSettingsBase::mCollectFacesMode == ECollectFacesMode::CollectFaces.
pub const InternalEdgeRemovingCollector = struct {
    /// The virtual functions this class overrides (C++ `override`, header order)
    pub const overrides = .{ .reset, .onBody, .addHit, .onBodyEnd };

    const max_local_delayed_results = 32;
    const max_local_voided_features = 128;

    // This algorithm tests a convex shape (shape 1) against a set of polygons (shape 2).
    // This assumption doesn't hold if the shape we're testing is a compound shape, so we must also
    // store the sub shape ID and ignore voided features that belong to another sub shape ID.
    const Voided = struct {
        /// Feature that is voided (of shape 2). Read with Vec3::sLoadFloat3Unsafe so must not be the last member (in Jolt; Zolt's loadFloat3Unsafe reads 3 floats).
        feature: Float3,
        /// Sub shape ID of the shape that is colliding against the feature (of shape 1).
        sub_shape_id: SubShapeID,
    };

    base: CollideShapeCollector,
    chained_collector: *CollideShapeCollector,
    /// The local buffer of `voided_features` (STLLocalAllocator<Voided, cMaxLocalVoidedFeatures>), falls back to `allocator`
    voided_features_allocator: STLLocalAllocator(Voided, max_local_voided_features),
    voided_features: std.ArrayList(Voided) = .empty,
    /// The local buffer of `delayed_results` (STLLocalAllocator<CollideShapeResult, cMaxLocalDelayedResults>), falls back to `allocator`
    delayed_results_allocator: STLLocalAllocator(CollideShapeResult, max_local_delayed_results),
    delayed_results: std.ArrayList(CollideShapeResult) = .empty,
    /// Max squared distance to consider a vertex to be the same as another vertex, used to determine if a feature is voided
    vertex_tolerance_sq: f32,
    /// The heap that the arrays fall back to (Jolt: its global heap)
    allocator: Allocator,
    /// Allocation failure (D6)
    alloc_error: AllocationErrorLatch = .{},

    /// Check if a vertex is voided
    fn isVoided(self: *const InternalEdgeRemovingCollector, sub_shape_id: SubShapeID, v: Vec3) bool {
        for (self.voided_features.items) |*vf| {
            if (vf.sub_shape_id.eql(sub_shape_id) and v.isClose(Vec3.loadFloat3Unsafe(&vf.feature), .{ .max_dist_sq = self.vertex_tolerance_sq }))
                return true;
        }
        return false;
    }

    /// Add all vertices of a face to the voided features
    fn voidFeatures(self: *InternalEdgeRemovingCollector, result: *const CollideShapeResult) void {
        for (result.shape2_face.constSlice()) |v| {
            if (!self.isVoided(result.sub_shape_id1, v)) {
                var vf: Voided = undefined;
                v.storeFloat3(&vf.feature);
                vf.sub_shape_id = result.sub_shape_id1;
                self.voided_features.append(self.voided_features_allocator.allocator(), vf) catch |err| {
                    self.setAllocationError(err);
                    return;
                };
            }
        }
    }

    /// Call the chained collector
    fn chain(self: *InternalEdgeRemovingCollector, result: *const CollideShapeResult) void {
        // Make sure the chained collector has the same context as we do
        self.chained_collector.setContext(self.base.getContext());

        // Forward the hit
        self.chained_collector.addHit(result);

        // If our chained collector updated its early out fraction, we need to follow
        self.base.updateEarlyOutFraction(self.chained_collector.getEarlyOutFraction());
    }

    /// Call the chained collector and void all features of result
    fn chainAndVoid(self: *InternalEdgeRemovingCollector, result: *const CollideShapeResult) void {
        self.chain(result);
        self.voidFeatures(result);
    }

    /// Record an allocation failure and stop the query (D6)
    fn setAllocationError(self: *InternalEdgeRemovingCollector, err: Allocator.Error) void {
        self.alloc_error.set(err);
        self.base.forceEarlyOut();
    }

    /// Constructor, configures a collector to be called with all the results that do not hit internal edges.
    /// In place (the arrays use buffers inside the collector): `var c: InternalEdgeRemovingCollector = undefined; c.init(...)`.
    /// `allocator` is the heap the arrays fall back to when they outgrow their local buffers.
    pub fn init(self: *InternalEdgeRemovingCollector, chained_collector: *CollideShapeCollector, vertex_tolerance_sq: f32, allocator: Allocator) void {
        self.* = .{
            .base = .initFrom(InternalEdgeRemovingCollector, chained_collector),
            .chained_collector = chained_collector,
            .voided_features_allocator = .init(allocator),
            .delayed_results_allocator = .init(allocator),
            .vertex_tolerance_sq = vertex_tolerance_sq,
            .allocator = allocator,
        };

        // Initialize arrays to full capacity to avoid needless reallocation calls
        // (this takes the local buffers, which cannot fail; if it did, the arrays would grow on demand)
        self.voided_features.ensureTotalCapacityPrecise(self.voided_features_allocator.allocator(), max_local_voided_features) catch {};
        self.delayed_results.ensureTotalCapacityPrecise(self.delayed_results_allocator.allocator(), max_local_delayed_results) catch {};
    }

    /// Destructor: frees the heap memory of the arrays (a recorded allocation failure must have been observed with checkError)
    pub fn deinit(self: *InternalEdgeRemovingCollector) void {
        self.alloc_error.clear();
        self.voided_features.deinit(self.voided_features_allocator.allocator());
        self.delayed_results.deinit(self.delayed_results_allocator.allocator());
    }

    /// Returns error.OutOfMemory if a hit or a voided feature could not be stored (the query was stopped early)
    pub fn checkError(self: *InternalEdgeRemovingCollector) Allocator.Error!void {
        return self.alloc_error.check();
    }

    // See: CollideShapeCollector::Reset
    pub fn reset(self: *InternalEdgeRemovingCollector) void {
        CollideShapeCollector.impl.reset(&self.base);

        self.chained_collector.reset();

        self.voided_features.clearRetainingCapacity();
        self.delayed_results.clearRetainingCapacity();
        self.alloc_error.clear();
    }

    // See: CollideShapeCollector::OnBody
    pub fn onBody(self: *InternalEdgeRemovingCollector, body: *const Body) void {
        // Just forward the call to our chained collector
        self.chained_collector.onBody(body);
    }

    // See: CollideShapeCollector::AddHit
    pub fn addHit(self: *InternalEdgeRemovingCollector, result: *const CollideShapeResult) void {
        // Hits after an allocation failure are dropped, the query is stopping (D6)
        if (self.alloc_error.err != null)
            return;

        // We only support welding when the shape is a triangle or has more vertices so that we can calculate a normal
        if (result.shape2_face.len < 3)
            return self.chainAndVoid(result);

        // Get the triangle normal of shape 2 face
        const triangle_normal = result.shape2_face.get(1).sub(result.shape2_face.get(0)).cross(result.shape2_face.get(2).sub(result.shape2_face.get(0)));
        const triangle_normal_len = triangle_normal.length();
        if (triangle_normal_len < 1e-6)
            return self.chainAndVoid(result);

        // If the triangle normal matches the contact normal within 1 degree, we can process the contact immediately
        // We make the assumption here that if the contact normal and the triangle normal align that the we're dealing with a 'face contact'
        const contact_normal = result.penetration_axis.negate();
        const contact_normal_len = result.penetration_axis.length();
        if (triangle_normal.dot(contact_normal) > 0.999848 * contact_normal_len * triangle_normal_len) // cos(1 degree)
            return self.chainAndVoid(result);

        // Delayed processing
        self.delayed_results.append(self.delayed_results_allocator.allocator(), result.*) catch |err| self.setAllocationError(err);
    }

    /// After all hits have been added, call this function to process the delayed results
    pub fn flush(self: *InternalEdgeRemovingCollector) void {
        defer {
            // All delayed results have been processed
            self.voided_features.clearRetainingCapacity();
            self.delayed_results.clearRetainingCapacity();
        }

        // After an allocation failure the results are incomplete and the early out fraction no longer follows the chained collector
        if (self.alloc_error.err != null)
            return;

        // Sort on biggest penetration depth first
        var sorted_indices_allocator = STLLocalAllocator(u32, max_local_delayed_results).init(self.allocator);
        const sorted_allocator = sorted_indices_allocator.allocator();
        var sorted_indices: std.ArrayList(u32) = .empty;
        defer sorted_indices.deinit(sorted_allocator);
        // Jolt's Array::resize reserves exactly the new size, so up to 32 indices stay in the local buffer
        // (std.ArrayList.resize would ask for more through growCapacity and always fall back to the heap)
        sorted_indices.ensureTotalCapacityPrecise(sorted_allocator, self.delayed_results.items.len) catch |err| return self.setAllocationError(err);
        sorted_indices.items.len = self.delayed_results.items.len;
        for (sorted_indices.items, 0..) |*index, i|
            index.* = @intCast(i);
        quickSort(u32, sorted_indices.items, self, struct {
            fn greaterPenetrationDepth(collector: *InternalEdgeRemovingCollector, lhs: u32, rhs: u32) bool {
                return collector.delayed_results.items[lhs].penetration_depth > collector.delayed_results.items[rhs].penetration_depth;
            }
        }.greaterPenetrationDepth);

        // Loop over all results
        for (0..self.delayed_results.items.len) |i| {
            const r = &self.delayed_results.items[sorted_indices.items[i]];

            // Determine which vertex or which edge is the closest to the contact point
            var best_dist_sq = math.flt_max;
            var best_v1_idx: u32 = 0;
            var best_v2_idx: u32 = 0;
            const num_v: u32 = r.shape2_face.len;
            var v1_idx: u32 = num_v - 1;
            var v1 = r.shape2_face.get(v1_idx).sub(r.contact_point_on2);
            for (0..num_v) |v2_idx_usize| {
                const v2_idx: u32 = @intCast(v2_idx_usize);
                const v2 = r.shape2_face.get(v2_idx).sub(r.contact_point_on2);
                const v1_v2 = v2.sub(v1);
                const denominator = v1_v2.lengthSq();
                if (denominator < math.square(math.flt_epsilon)) {
                    // Degenerate, assume v1 is closest, v2 will be tested in a later iteration
                    const v1_len_sq = v1.lengthSq();
                    if (v1_len_sq < best_dist_sq) {
                        best_dist_sq = v1_len_sq;
                        best_v1_idx = v1_idx;
                        best_v2_idx = v1_idx;
                    }
                } else {
                    // Taken from ClosestPoint::GetBaryCentricCoordinates
                    const fraction = -v1.dot(v1_v2) / denominator;
                    if (fraction < 1.0e-6) {
                        // Closest lies on v1
                        const v1_len_sq = v1.lengthSq();
                        if (v1_len_sq < best_dist_sq) {
                            best_dist_sq = v1_len_sq;
                            best_v1_idx = v1_idx;
                            best_v2_idx = v1_idx;
                        }
                    } else if (fraction < 1.0 - 1.0e-6) {
                        // Closest lies on the line segment v1, v2
                        const closest = v1.add(v1_v2.mulScalar(fraction));
                        const closest_len_sq = closest.lengthSq();
                        if (closest_len_sq < best_dist_sq) {
                            best_dist_sq = closest_len_sq;
                            best_v1_idx = v1_idx;
                            best_v2_idx = v2_idx;
                        }
                    }
                    // else closest is v2, but v2 will be tested in a later iteration
                }

                v1_idx = v2_idx;
                v1 = v2;
            }

            // Check if this vertex/edge is voided
            const voided = self.isVoided(r.sub_shape_id1, r.shape2_face.get(best_v1_idx)) and
                (best_v1_idx == best_v2_idx or self.isVoided(r.sub_shape_id1, r.shape2_face.get(best_v2_idx)));

            // No voided features, accept the contact
            if (!voided)
                self.chain(r);

            // Void the features of this face
            self.voidFeatures(r);
            if (self.alloc_error.err != null)
                return; // Out of memory: stop, the remaining results cannot be judged
        }
    }

    // See: CollideShapeCollector::OnBodyEnd
    pub fn onBodyEnd(self: *InternalEdgeRemovingCollector) void {
        self.flush();
        self.chained_collector.onBodyEnd();
    }

    /// Version of CollisionDispatch::sCollideShapeVsShape that removes internal edges.
    /// `allocator` is the heap of the wrapper's arrays (Zolt addition), returns error.OutOfMemory when it was exhausted.
    pub fn collideShapeVsShape(allocator: Allocator, shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) Allocator.Error!void {
        std.debug.assert(collide_shape_settings.active_edge_mode == .collide_with_all); // Won't work without colliding with all edges
        std.debug.assert(collide_shape_settings.collect_faces_mode == .collect_faces); // Won't work without collecting faces

        var wrapper: InternalEdgeRemovingCollector = undefined;
        wrapper.init(collector, collide_shape_settings.internal_edge_removal_vertex_tolerance_sq, allocator);
        defer wrapper.deinit();
        CollisionDispatch.collideShapeVsShape(shape1, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, &wrapper.base, shape_filter);
        wrapper.flush();
        try wrapper.checkError();
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the bit exact comparison with Jolt is in ZoltParity/Physics/TrianglesParity.zig)

const testing = std.testing;
const Quat = @import("../../Math/Quat.zig").Quat;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const CollisionCollectorImpl = @import("CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const ClosestHitPerBodyCollisionCollector = CollisionCollectorImpl.ClosestHitPerBodyCollisionCollector;
const TransformedShape = @import("TransformedShape.zig").TransformedShape;
const SphereShape = @import("Shape/SphereShape.zig").SphereShape;
const BoxShape = @import("Shape/BoxShape.zig").BoxShape;
const CollideConvexVsTriangles = @import("CollideConvexVsTriangles.zig").CollideConvexVsTriangles;

/// A hit against a triangle of shape 2
fn triangleHit(face: [3]Vec3, contact_point: Vec3, penetration_axis: Vec3, penetration_depth: f32, sub_shape_id1: u32, sub_shape_id2: u32) CollideShapeResult {
    var r = CollideShapeResult.init(contact_point, contact_point, penetration_axis, penetration_depth, .{ .value = sub_shape_id1 }, .{ .value = sub_shape_id2 }, .invalid);
    for (face) |v| r.shape2_face.append(v);
    return r;
}

test "InternalEdgeRemovingCollector: face contacts pass, contacts on voided edges and vertices are removed" {
    const allocator = testing.allocator;

    var chained = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer chained.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(5), .{});
    chained.base.setContext(&context);

    var collector: InternalEdgeRemovingCollector = undefined;
    collector.init(&chained.base, 1.0e-6, allocator);
    defer collector.deinit();
    try testing.expect(collector.base.getContext() == &context); // initFrom copies the context

    // Two coplanar triangles sharing the edge (0, 0, 0) - (0, 0, 1), normals +Y
    const t1 = [3]Vec3{ Vec3.init(0, 0, 0), Vec3.init(0, 0, 1), Vec3.init(1, 0, 0) };
    const t2 = [3]Vec3{ Vec3.init(0, 0, 0), Vec3.init(-1, 0, 0), Vec3.init(0, 0, 1) };

    // Face contact on t1 (contact normal = triangle normal): chained immediately, its vertices are voided
    const face_hit = triangleHit(t1, Vec3.init(0.25, 0, 0.25), Vec3.init(0, -1, 0), 0.1, 1, 10);
    collector.base.addHit(&face_hit);
    try testing.expectEqual(@as(usize, 1), chained.hits.items.len);
    try testing.expectEqual(@as(usize, 3), collector.voided_features.items.len);

    // Edge contact on t2 at the shared edge (ghost collision): delayed and then removed because both edge vertices are voided
    const edge_hit = triangleHit(t2, Vec3.init(0, 0, 0.5), Vec3.init(1, -0.2, 0), 0.05, 1, 11);
    collector.base.addHit(&edge_hit);
    try testing.expectEqual(@as(usize, 1), collector.delayed_results.items.len);

    // Vertex contact on t2 at (-1, 0, 0), which is not shared: delayed and accepted (it is deeper, so it is processed
    // before the edge contact voids the vertices of t2)
    const vertex_hit = triangleHit(t2, Vec3.init(-1, 0, 0), Vec3.init(1, -1, 0), 0.08, 1, 12);
    collector.base.addHit(&vertex_hit);

    // A contact of another sub shape of shape 1 on the shared edge is not affected by the voided features of sub shape 1
    const other_hit = triangleHit(t2, Vec3.init(0, 0, 0.5), Vec3.init(1, -0.2, 0), 0.01, 2, 11);
    collector.base.addHit(&other_hit);

    // The chained collector gets our context when a hit is passed on
    const context2 = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(6), .{});
    collector.base.setContext(&context2);
    collector.flush();
    try collector.checkError();
    try chained.checkError();
    try testing.expect(chained.base.getContext() == &context2);
    try testing.expectEqual(@as(usize, 3), chained.hits.items.len);
    try testing.expectEqual(@as(u32, 10), chained.hits.items[0].sub_shape_id2.getValue());
    try testing.expectEqual(@as(u32, 12), chained.hits.items[1].sub_shape_id2.getValue()); // Deepest first: 0.08, 0.05 (removed), 0.01
    try testing.expectEqual(@as(u32, 2), chained.hits.items[2].sub_shape_id1.getValue());
    try testing.expectEqual(@as(usize, 0), collector.voided_features.items.len); // Cleared by flush
    try testing.expectEqual(@as(usize, 0), collector.delayed_results.items.len);

    // Hits without a face or with a degenerate face are chained immediately
    var no_face = triangleHit(t1, Vec3.zero(), Vec3.init(1, 0, 0), 0.1, 1, 13);
    no_face.shape2_face.resize(2);
    collector.base.addHit(&no_face);
    const degenerate = triangleHit(.{ Vec3.zero(), Vec3.zero(), Vec3.init(1, 0, 0) }, Vec3.zero(), Vec3.init(1, 0, 0), 0.1, 1, 14);
    collector.base.addHit(&degenerate);
    try testing.expectEqual(@as(usize, 5), chained.hits.items.len);

    // Reset forwards to the chained collector
    collector.base.reset();
    try testing.expectEqual(@as(usize, 0), chained.hits.items.len);
    try testing.expectEqual(@as(usize, 0), collector.voided_features.items.len);
}

test "InternalEdgeRemovingCollector: follows the early out fraction of the chained collector, OnBody / OnBodyEnd" {
    const allocator = testing.allocator;

    var chained = ClosestHitPerBodyCollisionCollector(CollideShapeCollector).init(allocator);
    defer chained.deinit();
    var collector: InternalEdgeRemovingCollector = undefined;
    collector.init(&chained.base, 1.0e-6, allocator);
    defer collector.deinit();

    const t = [3]Vec3{ Vec3.init(0, 0, 0), Vec3.init(0, 0, 1), Vec3.init(1, 0, 0) };
    const body: Body = .{};
    collector.base.onBody(&body);
    const delayed = triangleHit(t, Vec3.init(0.5, 0, 0), Vec3.init(0, -1, 1), 0.3, 1, 1);
    collector.base.addHit(&delayed);
    const face_hit = triangleHit(t, Vec3.init(0.25, 0, 0.25), Vec3.init(0, -1, 0), 0.1, 1, 2);
    collector.base.addHit(&face_hit);
    try testing.expectEqual(@as(f32, -0.1), collector.base.getEarlyOutFraction()); // Follows the chained collector
    collector.base.onBodyEnd(); // Flush: the delayed hit is on the edge v2..v0, whose vertices the face contact voided: removed
    try chained.checkError();
    try testing.expectEqual(@as(usize, 1), chained.hits.items.len);
    try testing.expectEqual(@as(f32, 0.1), chained.hits.items[0].penetration_depth);
}

test "InternalEdgeRemovingCollector: many delayed results and voided features (heap fallback), out of memory" {
    const allocator = testing.allocator;

    // 100 separate triangles (300 voided features > 128) with 2 edge contacts each (200 delayed results > 32): the deeper
    // contact of a triangle is accepted, the other one is removed because the first one voided the vertices of its edge
    const num_triangles = 100;
    var hits: std.ArrayList(CollideShapeResult) = .empty;
    defer hits.deinit(allocator);
    for (0..num_triangles) |i| {
        const p = Vec3.init(3.0 * @as(f32, @floatFromInt(i)), 0, 0);
        const t = [3]Vec3{ p, p.add(Vec3.init(0, 0, 1)), p.add(Vec3.init(1, 0, 0)) };
        const sub_shape_id1: u32 = @intCast(1 + i % 2); // Two sub shapes of shape 1
        const depth = 0.001 * @as(f32, @floatFromInt(i));
        try hits.append(allocator, triangleHit(t, p.add(Vec3.init(0.5, 0, 0)), Vec3.init(0, -1, 1), 1.0 + depth, sub_shape_id1, @intCast(i)));
        try hits.append(allocator, triangleHit(t, p.add(Vec3.init(0.25, 0, 0)), Vec3.init(0, -1, 1), 0.5 + depth, sub_shape_id1, @intCast(1000 + i)));
    }

    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var chained = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer chained.deinit();
        var collector: InternalEdgeRemovingCollector = undefined;
        collector.init(&chained.base, 1.0e-6, failing.allocator());
        defer collector.deinit();
        for (hits.items) |*h| collector.base.addHit(h);
        collector.flush();
        collector.checkError() catch |err| {
            try testing.expectEqual(error.OutOfMemory, err);
            try testing.expect(collector.base.shouldEarlyOut());
            continue;
        };
        try chained.checkError();
        try testing.expect(fail_index >= 3); // The delayed results, the sorted indices and the voided features went to the heap

        // The deeper contact of every triangle, deepest first
        try testing.expectEqual(@as(usize, num_triangles), chained.hits.items.len);
        for (chained.hits.items, 0..) |hit, i|
            try testing.expectEqual(@as(u32, @intCast(num_triangles - 1 - i)), hit.sub_shape_id2.getValue());
        break;
    }
}

test "InternalEdgeRemovingCollector: 32 delayed results and 128 voided features stay in the local buffers" {
    const allocator = testing.allocator;

    // Like Jolt, which does not touch the heap here: every heap allocation fails
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    var chained = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer chained.deinit();
    var collector: InternalEdgeRemovingCollector = undefined;
    collector.init(&chained.base, 1.0e-6, failing.allocator());
    defer collector.deinit();

    // 10 face contacts on separate triangles and a hit with a 2 vertex face: chained immediately, 32 voided features
    for (0..10) |i| {
        const p = Vec3.init(3.0 * @as(f32, @floatFromInt(i)), 0, 0);
        const face_hit = triangleHit(.{ p, p.add(Vec3.init(0, 0, 1)), p.add(Vec3.init(1, 0, 0)) }, p.add(Vec3.init(0.25, 0, 0.25)), Vec3.init(0, -1, 0), 0.1, 1, @intCast(i));
        collector.base.addHit(&face_hit);
    }
    var two_vertices = triangleHit(.{ Vec3.init(-3, 0, 0), Vec3.init(-3, 0, 1), Vec3.zero() }, Vec3.init(-3, 0, 0.5), Vec3.init(1, 0, 0), 0.1, 1, 10);
    two_vertices.shape2_face.resize(2);
    collector.base.addHit(&two_vertices);
    try testing.expectEqual(@as(usize, 32), collector.voided_features.items.len);

    // 32 edge contacts on 32 other separate triangles: delayed, then accepted by flush, which voids 96 more features
    // (128 in total) and sorts 32 indices
    for (0..InternalEdgeRemovingCollector.max_local_delayed_results) |i| {
        const p = Vec3.init(3.0 * @as(f32, @floatFromInt(i)), 0, 5);
        const edge_hit = triangleHit(.{ p, p.add(Vec3.init(0, 0, 1)), p.add(Vec3.init(1, 0, 0)) }, p.add(Vec3.init(0.5, 0, 0)), Vec3.init(0, -1, 1), 0.5 + 0.001 * @as(f32, @floatFromInt(i)), 1, @intCast(100 + i));
        collector.base.addHit(&edge_hit);
    }
    try testing.expectEqual(@as(usize, InternalEdgeRemovingCollector.max_local_delayed_results), collector.delayed_results.items.len);
    collector.flush();
    try collector.checkError();
    try chained.checkError();
    try testing.expectEqual(@as(usize, 11 + InternalEdgeRemovingCollector.max_local_delayed_results), chained.hits.items.len);
    try testing.expectEqual(@as(usize, 0), failing.allocations);
    try testing.expectEqual(@as(usize, InternalEdgeRemovingCollector.max_local_voided_features), collector.voided_features.capacity);
    try testing.expectEqual(@as(usize, InternalEdgeRemovingCollector.max_local_delayed_results), collector.delayed_results.capacity);

    // One more delayed result needs the heap
    collector.base.reset();
    for (0..InternalEdgeRemovingCollector.max_local_delayed_results + 1) |i| {
        const p = Vec3.init(3.0 * @as(f32, @floatFromInt(i)), 0, 5);
        const edge_hit = triangleHit(.{ p, p.add(Vec3.init(0, 0, 1)), p.add(Vec3.init(1, 0, 0)) }, p.add(Vec3.init(0.5, 0, 0)), Vec3.init(0, -1, 1), 0.5, 1, @intCast(100 + i));
        collector.base.addHit(&edge_hit);
    }
    collector.flush();
    try testing.expectError(error.OutOfMemory, collector.checkError());
    try testing.expectEqual(@as(usize, 0), chained.hits.items.len);
}

test "InternalEdgeRemovingCollector.collideShapeVsShape: a box on two coplanar triangles" {
    const allocator = testing.allocator;

    var box = BoxShape.init(allocator, Vec3.replicate(0.5), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    var sphere = SphereShape.init(allocator, 0.5, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();

    // collideShapeVsShape dispatches box vs sphere through CollisionDispatch and flushes
    var settings: CollideShapeSettings = .{};
    settings.active_edge_mode = .collide_with_all;
    settings.collect_faces_mode = .collect_faces;
    var chained = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer chained.deinit();
    try InternalEdgeRemovingCollector.collideShapeVsShape(allocator, box.asShape(), sphere.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.translation(Vec3.init(0, 0.9, 0)), .{}, .{}, &settings, &chained.base, &.{});
    try chained.checkError();
    try testing.expectEqual(@as(usize, 1), chained.hits.items.len);

    // An edge of a box rotated 45 degrees around Z pushed into the top face of the box: the hit is delayed (the contact
    // normal is not the normal of a face of shape 2), and like in Jolt flushing it does not need the heap
    chained.reset();
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 0 });
    try InternalEdgeRemovingCollector.collideShapeVsShape(failing.allocator(), box.asShape(), box.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.25 * math.pi), Vec3.init(0, 1.1, 0)), .{}, .{}, &settings, &chained.base, &.{});
    try chained.checkError();
    try testing.expectEqual(@as(usize, 1), chained.hits.items.len);
    try testing.expectEqual(@as(usize, 0), failing.allocations);

    // The wrapper with triangles fed by CollideConvexVsTriangles: a box resting on 2 triangles that form a quad, sliding
    // over the shared diagonal does not report the edge normal
    chained.reset();
    var collector: InternalEdgeRemovingCollector = undefined;
    collector.init(&chained.base, settings.internal_edge_removal_vertex_tolerance_sq, allocator);
    defer collector.deinit();
    var collider = CollideConvexVsTriangles.init(&box.base, Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(0, 0.47, 0)), Mat44.identity(), .{}, &settings, &collector.base);
    collider.collide(Vec3.init(-2, 0, -2), Vec3.init(-2, 0, 2), Vec3.init(2, 0, 2), 0b111, .{ .value = 0 });
    collider.collide(Vec3.init(-2, 0, -2), Vec3.init(2, 0, 2), Vec3.init(2, 0, -2), 0b111, .{ .value = 1 });
    collector.flush();
    try collector.checkError();
    try chained.checkError();
    try testing.expect(chained.hits.items.len >= 1);
    for (chained.hits.items) |hit|
        try testing.expect(hit.penetration_axis.normalized().isClose(Vec3.init(0, -1, 0), .{ .max_dist_sq = 1.0e-6 }));
}
