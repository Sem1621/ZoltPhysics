//! Port of: Jolt/Physics/Collision/CollideConvexVsTriangles.h, Jolt/Physics/Collision/CollideConvexVsTriangles.cpp
//! Status: complete
//!
//! A helper class that the triangle shapes (TriangleShape, MeshShape, HeightFieldShape, SoftBodyShape) construct on the
//! stack and feed with triangles (`collide`). Jolt's `struct Visitor : public CollideConvexVsTriangles` becomes a struct
//! that embeds it as `base` and calls `self.base.collide(...)`.
//! - The references `const CollideShapeSettings &` / `CollideShapeCollector &` are pointers (`*const` / `*`), the
//!   `Mat44Arg` / `Vec3Arg` members are copies.
//! - The support functions are constructed lazily in the two SupportBuffers inside this struct and the struct keeps
//!   pointers into them (D9): after the first `collide` it must not be moved (C++ objects never move either).
//! - `collide` is non-const in Jolt (it creates the support functions): `self: *CollideConvexVsTriangles`.

const std = @import("std");
const AABox = @import("../../Geometry/AABox.zig").AABox;
const ConvexSupport = @import("../../Geometry/ConvexSupport.zig");
const AddConvexRadius = ConvexSupport.AddConvexRadius;
const TriangleConvexSupport = ConvexSupport.TriangleConvexSupport;
const EPAPenetrationDepth = @import("../../Geometry/EPAPenetrationDepth.zig").EPAPenetrationDepth;
const math = @import("../../Math/Math.zig");
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const ShapeFile = @import("Shape/Shape.zig");
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const ConvexShape = @import("Shape/ConvexShape.zig").ConvexShape;
const ScaleHelpers = @import("Shape/ScaleHelpers.zig");
const SubShapeID = @import("Shape/SubShapeID.zig").SubShapeID;
const CollideShapeFile = @import("CollideShape.zig");
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const TransformedShape = @import("TransformedShape.zig").TransformedShape;
const ActiveEdges = @import("ActiveEdges.zig");
const NarrowPhaseStats = @import("NarrowPhaseStats.zig");
const TrackNarrowPhaseCollector = NarrowPhaseStats.TrackNarrowPhaseCollector;
const track_narrowphase_stats = NarrowPhaseStats.track_narrowphase_stats;

/// Collision detection helper that collides a convex object vs one or more triangles
pub const CollideConvexVsTriangles = struct {
    /// Settings for this collision operation
    collide_shape_settings: *const CollideShapeSettings,
    /// The collector that will receive the results
    collector: *CollideShapeCollector,
    /// The shape that we're colliding with
    shape1: *const ConvexShape,
    /// The scale of the shape (in shape local space) of the shape we're colliding with
    scale1: Vec3,
    /// The scale of the shape (in shape local space) of the shape we're colliding against
    scale2: Vec3,
    /// Transform of the shape we're colliding with
    transform1: Mat44,
    /// Transform that takes a point in space of the colliding shape to the shape we're colliding with
    transform2_to1: Mat44,
    /// Bounds of the colliding shape in local space
    bounds_of1: AABox,
    /// Bounds of the colliding shape in space of shape we're colliding with
    bounds_of1_in_space_of2: AABox,
    /// Sub shape ID of colliding shape
    sub_shape_id1: SubShapeID,
    /// Sign of the scale of object 2, -1 if object is inside out, 1 if not
    scale_sign2: f32,
    /// Buffer that holds the support function data excluding convex radius
    buffer_ex_cvx_radius: ConvexShape.SupportBuffer = .{},
    /// Buffer that holds the support function data including convex radius
    buffer_inc_cvx_radius: ConvexShape.SupportBuffer = .{},
    /// Actual support function object excluding convex radius
    shape1_ex_cvx_radius: ?*const ConvexShape.Support = null,
    /// Actual support function object including convex radius
    shape1_inc_cvx_radius: ?*const ConvexShape.Support = null,

    /// Constructor
    /// @param shape1 The convex shape to collide against triangles
    /// @param scale1 Local space scale for the convex object (scales relative to its center of mass)
    /// @param scale2 Local space scale for the triangles
    /// @param center_of_mass_transform1 Transform that takes the center of mass of 1 into world space
    /// @param center_of_mass_transform2 Transform that takes the center of mass of 2 into world space
    /// @param sub_shape_id1 Sub shape ID of the convex object
    /// @param collide_shape_settings Settings for the collide shape query
    /// @param collector The collector that will receive the results
    pub fn init(shape1: *const ConvexShape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id1: SubShapeID, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector) CollideConvexVsTriangles {
        // Get transforms
        const inverse_transform2 = center_of_mass_transform2.inversedRotationTranslation();
        const transform1_to_2 = inverse_transform2.mul(center_of_mass_transform1);
        const transform2_to1 = transform1_to_2.inversedRotationTranslation();

        // Calculate bounds
        var bounds_of1 = shape1.base.getLocalBounds().scaled(scale1);
        bounds_of1.expandBy(Vec3.replicate(collide_shape_settings.max_separation_distance));
        const bounds_of1_in_space_of2 = bounds_of1.transformed(transform1_to_2); // Convert bounding box of 1 into space of 2

        // Determine if shape 2 is inside out or not
        const scale_sign2: f32 = if (ScaleHelpers.isInsideOut(scale2)) -1.0 else 1.0;

        return .{
            .collide_shape_settings = collide_shape_settings,
            .collector = collector,
            .shape1 = shape1,
            .scale1 = scale1,
            .scale2 = scale2,
            .transform1 = center_of_mass_transform1,
            .transform2_to1 = transform2_to1,
            .bounds_of1 = bounds_of1,
            .bounds_of1_in_space_of2 = bounds_of1_in_space_of2,
            .sub_shape_id1 = sub_shape_id1,
            .scale_sign2 = scale_sign2,
        };
    }

    /// Collide convex object with a single triangle
    /// @param v0_in , v1_in , v2_in: CCW triangle vertices
    /// @param active_edges bit 0 = edge v0..v1 is active, bit 1 = edge v1..v2 is active, bit 2 = edge v2..v0 is active
    /// An active edge is an edge that is not connected to another triangle in such a way that it is impossible to collide with the edge
    /// @param sub_shape_id2 The sub shape ID for the triangle
    pub fn collide(self: *CollideConvexVsTriangles, v0_in: Vec3, v1_in: Vec3, v2_in: Vec3, active_edges: u8, sub_shape_id2: SubShapeID) void {
        const settings = self.collide_shape_settings;

        // Scale triangle and transform it to the space of 1
        const v0 = self.transform2_to1.mulVec3(self.scale2.mul(v0_in));
        const v1 = self.transform2_to1.mulVec3(self.scale2.mul(v1_in));
        const v2 = self.transform2_to1.mulVec3(self.scale2.mul(v2_in));

        // Calculate triangle normal
        const triangle_normal = v1.sub(v0).cross(v2.sub(v0)).mulScalar(self.scale_sign2);

        // Backface check
        const back_facing = triangle_normal.dot(v0) > 0.0;
        if (settings.back_face_mode == .ignore_back_faces and back_facing)
            return;

        // Get bounding box for triangle
        var triangle_bbox = AABox.fromTwoPoints(v0, v1);
        triangle_bbox.encapsulateVec3(v2);

        // Get intersection between triangle and shape box, if there is none, we're done
        if (!triangle_bbox.overlaps(self.bounds_of1))
            return;

        // Create triangle support function
        const triangle = TriangleConvexSupport.init(v0, v1, v2);

        // Perform collision detection
        // Note: As we don't remember the penetration axis from the last iteration, and it is likely that the shape (A) we're colliding the triangle (B) against is in front of the triangle,
        // and the penetration axis is the shortest distance along to push B out of collision, we use the inverse of the triangle normal as an initial penetration axis. This has been seen
        // to improve performance by approx. 5% over using a fixed axis like (1, 0, 0).
        var penetration_axis = triangle_normal.negate();
        var point1: Vec3 = undefined;
        var point2: Vec3 = undefined;
        var pen_depth: EPAPenetrationDepth = .{};
        var status: EPAPenetrationDepth.Status = undefined;

        // Get the support function
        if (self.shape1_ex_cvx_radius == null)
            self.shape1_ex_cvx_radius = self.shape1.getSupportFunction(.exclude_convex_radius, &self.buffer_ex_cvx_radius, self.scale1);
        const shape1_ex_cvx_radius = self.shape1_ex_cvx_radius.?;

        // Perform GJK step
        var max_separation_distance = settings.max_separation_distance;
        status = pen_depth.getPenetrationDepthStepGJK(shape1_ex_cvx_radius, shape1_ex_cvx_radius.getConvexRadius() + max_separation_distance, &triangle, 0.0, settings.collision_tolerance, &penetration_axis, &point1, &point2);

        // Check result of collision detection
        if (status == .not_colliding) {
            return;
        } else if (status == .indeterminate) {
            // Need to run expensive EPA algorithm

            // We know we're overlapping at this point, so we can set the max separation distance to 0.
            // Numerically it is possible that GJK finds that the shapes are overlapping but EPA finds that they're separated.
            // In order to avoid this, we clamp the max separation distance to 1 so that we don't excessively inflate the shape,
            // but we still inflate it enough to avoid the case where EPA misses the collision.
            max_separation_distance = math.min(max_separation_distance, @as(f32, 1.0));

            // Get the support function
            if (self.shape1_inc_cvx_radius == null)
                self.shape1_inc_cvx_radius = self.shape1.getSupportFunction(.include_convex_radius, &self.buffer_inc_cvx_radius, self.scale1);

            // Add convex radius
            const shape1_add_max_separation_distance = AddConvexRadius(ConvexShape.Support).init(self.shape1_inc_cvx_radius.?, max_separation_distance);

            // Perform EPA step
            if (!pen_depth.getPenetrationDepthStepEPA(&shape1_add_max_separation_distance, &triangle, settings.penetration_tolerance, &penetration_axis, &point1, &point2))
                return;
        }

        // Check if the penetration is bigger than the early out fraction
        const penetration_depth = point2.sub(point1).length() - max_separation_distance;
        if (-penetration_depth >= self.collector.getEarlyOutFraction())
            return;

        // Correct point1 for the added separation distance
        const penetration_axis_len = penetration_axis.length();
        if (penetration_axis_len > 0.0)
            point1 = point1.sub(penetration_axis.mulScalar(max_separation_distance / penetration_axis_len));

        // Check if we have enabled active edge detection
        if (settings.active_edge_mode == .collide_only_with_active and active_edges != 0b111) {
            // Convert the active edge velocity hint to local space
            const active_edge_movement_direction = self.transform1.multiply3x3Transposed(settings.active_edge_movement_direction);

            // Update the penetration axis to account for active edges
            // Note that we flip the triangle normal as the penetration axis is pointing towards the triangle instead of away
            penetration_axis = ActiveEdges.fixNormal(v0, v1, v2, if (back_facing) triangle_normal else triangle_normal.negate(), active_edges, point2, penetration_axis, active_edge_movement_direction);
        }

        // Convert to world space
        point1 = self.transform1.mulVec3(point1);
        point2 = self.transform1.mulVec3(point2);
        const penetration_axis_world = self.transform1.multiply3x3(penetration_axis);

        // Create collision result
        var result = CollideShapeResult.init(point1, point2, penetration_axis_world, penetration_depth, self.sub_shape_id1, sub_shape_id2, TransformedShape.getBodyID(self.collector.getContext()));

        // Gather faces
        if (settings.collect_faces_mode == .collect_faces) {
            // Get supporting face of shape 1
            self.shape1.base.getSupportingFace(.empty, penetration_axis.negate(), self.scale1, self.transform1, &result.shape1_face);

            // Get face of the triangle
            result.shape2_face.resize(3);
            result.shape2_face.buffer[0] = self.transform1.mulVec3(v0);
            result.shape2_face.buffer[1] = self.transform1.mulVec3(v1);
            result.shape2_face.buffer[2] = self.transform1.mulVec3(v2);

            // When inside out, we need to swap the triangle winding
            if (self.scale_sign2 < 0.0)
                std.mem.swap(Vec3, &result.shape2_face.buffer[1], &result.shape2_face.buffer[2]);
        }

        // Notify the collector
        var track: if (track_narrowphase_stats) TrackNarrowPhaseCollector else void = undefined;
        if (track_narrowphase_stats) track = .init();
        defer if (track_narrowphase_stats) track.deinit();
        self.collector.addHit(&result);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt's TestConvexVsTriangles is in ZoltTests/Physics/ConvexVsTrianglesTest.zig, the bit exact comparison with
// Jolt in ZoltParity/Physics/TrianglesParity.zig)

const testing = std.testing;
const Quat = @import("../../Math/Quat.zig").Quat;
const BoxShape = @import("Shape/BoxShape.zig").BoxShape;
const SphereShape = @import("Shape/SphereShape.zig").SphereShape;
const CollisionCollectorImpl = @import("CollisionCollectorImpl.zig");
const AllHitCollisionCollector = CollisionCollectorImpl.AllHitCollisionCollector;
const ClosestHitCollisionCollector = CollisionCollectorImpl.ClosestHitCollisionCollector;
const RVec3 = @import("../../Math/Real.zig").RVec3;

test "CollideConvexVsTriangles: box vs triangles, back faces, max separation distance, faces, inside out scale" {
    const allocator = testing.allocator;

    var box = BoxShape.init(allocator, Vec3.replicate(0.5), .{ .convex_radius = 0.05 });
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    // A big triangle in the XZ plane (CCW seen from above: normal +Y), the box 0.47 above it (penetrating 0.03, less than
    // the convex radius: GJK finds the penetration)
    const v0 = Vec3.init(-5, 0, -5);
    const v1 = Vec3.init(0, 0, 5);
    const v2 = Vec3.init(5, 0, -5);
    const transform2 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisX(), 0.3), Vec3.init(1, 2, 3));
    const transform1 = transform2.mul(Mat44.translation(Vec3.init(0, 0.47, 0)));
    var settings: CollideShapeSettings = .{};
    settings.collect_faces_mode = .collect_faces;
    var hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(7), .{});
    hits.base.setContext(&context);

    var collider = CollideConvexVsTriangles.init(&box.base, Vec3.one(), Vec3.one(), transform1, transform2, .{ .value = 3 }, &settings, &hits.base);
    collider.collide(v0, v1, v2, 0b111, .{ .value = 5 });
    try hits.checkError();
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    const hit = hits.hits.items[0];
    try testing.expectApproxEqAbs(@as(f32, 0.03), hit.penetration_depth, 1.0e-5);
    try testing.expect(collider.shape1_inc_cvx_radius == null); // No EPA
    try testing.expect(transform2.multiply3x3Transposed(hit.penetration_axis).normalized().isClose(Vec3.init(0, -1, 0), .{ .max_dist_sq = 1.0e-8 }));
    try testing.expectEqual(@as(u32, 3), hit.sub_shape_id1.getValue());
    try testing.expectEqual(@as(u32, 5), hit.sub_shape_id2.getValue());
    try testing.expect(hit.body_id2.eql(.init(7)));
    try testing.expectEqual(@as(u32, 4), hit.shape1_face.len); // Bottom face of the box
    try testing.expectEqual(@as(u32, 3), hit.shape2_face.len);
    try testing.expect(hit.shape2_face.get(0).isClose(transform2.mulVec3(v0), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(hit.shape2_face.get(1).isClose(transform2.mulVec3(v1), .{ .max_dist_sq = 1.0e-10 }));

    // The support functions are created once and reused for the next triangle
    const support = collider.shape1_ex_cvx_radius;
    try testing.expect(support != null);
    collider.collide(v0, v1, v2, 0b111, .{ .value = 6 });
    try testing.expect(collider.shape1_ex_cvx_radius == support);
    try testing.expectEqual(@as(usize, 2), hits.hits.items.len);

    // Back facing triangle (reversed winding): ignored by default, reported with CollideWithBackFaces
    hits.reset();
    collider.collide(v0, v2, v1, 0b111, .{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
    settings.back_face_mode = .collide_with_back_faces;
    collider.collide(v0, v2, v1, 0b111, .{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);

    // Inside out scale: the back facing triangle is front facing again and the face winding is swapped back
    hits.reset();
    settings.back_face_mode = .ignore_back_faces;
    var inside_out = CollideConvexVsTriangles.init(&box.base, Vec3.one(), Vec3.init(-1, 1, 1), transform1, transform2, .{}, &settings, &hits.base);
    const m = Vec3.init(-1, 1, 1);
    inside_out.collide(v0.mul(m), v2.mul(m), v1.mul(m), 0b111, .{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expect(hits.hits.items[0].shape2_face.get(1).isClose(transform2.mulVec3(v1), .{ .max_dist_sq = 1.0e-10 }));

    // Separated by 0.5: no hit, unless the max separation distance is bigger (negative penetration depth)
    hits.reset();
    const far = transform2.mul(Mat44.translation(Vec3.init(0, 1.0, 0)));
    var separated = CollideConvexVsTriangles.init(&box.base, Vec3.one(), Vec3.one(), far, transform2, .{}, &settings, &hits.base);
    separated.collide(v0, v1, v2, 0b111, .{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
    settings.max_separation_distance = 1.0;
    var separated2 = CollideConvexVsTriangles.init(&box.base, Vec3.one(), Vec3.one(), far, transform2, .{}, &settings, &hits.base);
    separated2.collide(v0, v1, v2, 0b111, .{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expectApproxEqAbs(@as(f32, -0.5), hits.hits.items[0].penetration_depth, 1.0e-5);

    // A triangle outside of the bounds of the box: rejected before GJK
    hits.reset();
    separated2.collide(v0.add(Vec3.init(20, 0, 0)), v1.add(Vec3.init(20, 0, 0)), v2.add(Vec3.init(20, 0, 0)), 0b111, .{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
}

test "CollideConvexVsTriangles: deep penetration (EPA), early out fraction and active edges" {
    const allocator = testing.allocator;

    var sphere = SphereShape.init(allocator, 1.0, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();

    const v0 = Vec3.init(0, 0, 0);
    const v1 = Vec3.init(0, 0, 4);
    const v2 = Vec3.init(4, 0, 0);
    var settings: CollideShapeSettings = .{};

    // The sphere center is below the triangle plane but in front of its interior: EPA is needed (convex radius is fully penetrated)
    var closest = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer closest.deinit();
    var collider = CollideConvexVsTriangles.init(&sphere.base, Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(1, 0.0, 1)), Mat44.identity(), .{}, &settings, &closest.base);
    collider.collide(v0, v1, v2, 0b111, .{});
    try testing.expect(closest.hadHit());
    try testing.expect(collider.shape1_inc_cvx_radius != null); // EPA ran
    try testing.expectApproxEqAbs(@as(f32, 1.0), closest.hit.penetration_depth, 1.0e-4);

    // Early out fraction: a shallower hit is rejected
    var shallow = CollideConvexVsTriangles.init(&sphere.base, Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(1, 0.5, 1)), Mat44.identity(), .{}, &settings, &closest.base);
    shallow.collide(v0, v1, v2, 0b111, .{});
    try testing.expectApproxEqAbs(@as(f32, 1.0), closest.hit.penetration_depth, 1.0e-4);

    // Hitting the edge v0..v1 from the side: an inactive edge gives the triangle normal, an active edge the edge normal
    var hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer hits.deinit();
    var edge_collider = CollideConvexVsTriangles.init(&sphere.base, Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(-0.9, 0.001, 2)), Mat44.identity(), .{}, &settings, &hits.base);
    edge_collider.collide(v0, v1, v2, 0b110, .{});
    edge_collider.collide(v0, v1, v2, 0b001, .{});
    settings.active_edge_mode = .collide_with_all;
    edge_collider.collide(v0, v1, v2, 0b110, .{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 3), hits.hits.items.len);
    try testing.expect(hits.hits.items[0].penetration_axis.normalized().isClose(Vec3.init(0, -1, 0), .{ .max_dist_sq = 1.0e-6 }));
    try testing.expect(hits.hits.items[1].penetration_axis.normalized().isClose(Vec3.init(1, 0, 0), .{ .max_dist_sq = 1.0e-4 }));
    try testing.expect(hits.hits.items[2].penetration_axis.normalized().isClose(Vec3.init(1, 0, 0), .{ .max_dist_sq = 1.0e-4 }));
}
