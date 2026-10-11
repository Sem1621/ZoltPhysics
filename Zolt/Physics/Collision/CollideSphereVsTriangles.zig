//! Port of: Jolt/Physics/Collision/CollideSphereVsTriangles.h, Jolt/Physics/Collision/CollideSphereVsTriangles.cpp
//! Status: complete
//!
//! A helper class that the triangle shapes construct on the stack and feed with triangles (`collide`), see
//! CollideConvexVsTriangles.zig. The references `const CollideShapeSettings &` / `CollideShapeCollector &` are pointers,
//! the `Mat44Arg` / `Vec3Arg` members are copies. Unlike CollideConvexVsTriangles it holds no pointers into itself.

const std = @import("std");
const ClosestPoint = @import("../../Geometry/ClosestPoint.zig");
const math = @import("../../Math/Math.zig");
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const ShapeFile = @import("Shape/Shape.zig");
const CollideShapeCollector = ShapeFile.CollideShapeCollector;
const SphereShape = @import("Shape/SphereShape.zig").SphereShape;
const ScaleHelpers = @import("Shape/ScaleHelpers.zig");
const SubShapeID = @import("Shape/SubShapeID.zig").SubShapeID;
const CollideShapeFile = @import("CollideShape.zig");
const CollideShapeResult = CollideShapeFile.CollideShapeResult;
const CollideShapeSettings = CollideShapeFile.CollideShapeSettings;
const TransformedShape = @import("TransformedShape.zig").TransformedShape;
const NarrowPhaseStats = @import("NarrowPhaseStats.zig");
const TrackNarrowPhaseCollector = NarrowPhaseStats.TrackNarrowPhaseCollector;
const track_narrowphase_stats = NarrowPhaseStats.track_narrowphase_stats;

/// The edges that need to be active for the closest feature of GetClosestPointOnTriangle (sClosestFeatureToActiveEdgesMask)
const closest_feature_to_active_edges_mask = [_]u8{
    0b000, // 0b000: Invalid, guarded by an assert
    0b101, // 0b001: Vertex 1 -> edge 1 or 3
    0b011, // 0b010: Vertex 2 -> edge 1 or 2
    0b001, // 0b011: Vertex 1 & 2 -> edge 1
    0b110, // 0b100: Vertex 3 -> edge 2 or 3
    0b100, // 0b101: Vertex 1 & 3 -> edge 3
    0b010, // 0b110: Vertex 2 & 3 -> edge 2
    // 0b111: Vertex 1, 2 & 3 -> interior, guarded by an if
};

/// Collision detection helper that collides a sphere vs one or more triangles
pub const CollideSphereVsTriangles = struct {
    /// Settings for this collision operation
    collide_shape_settings: *const CollideShapeSettings,
    /// The collector that will receive the results
    collector: *CollideShapeCollector,
    /// The shape that we're colliding with
    shape1: *const SphereShape,
    /// The scale of the shape (in shape local space) of the shape we're colliding against
    scale2: Vec3,
    /// Transform of the shape we're colliding against
    transform2: Mat44,
    /// The center of the sphere in the space of 2
    sphere_center_in2: Vec3,
    /// Sub shape ID of colliding shape
    sub_shape_id1: SubShapeID,
    /// Sign of the scale of object 2, -1 if object is inside out, 1 if not
    scale_sign2: f32,
    /// Radius of the sphere
    radius: f32,
    /// (Radius + Max SeparationDistance)^2
    radius_plus_max_separation_sq: f32,

    /// Constructor
    /// @param shape1 The sphere to collide against triangles
    /// @param scale1 Local space scale for the sphere (scales relative to its center of mass)
    /// @param scale2 Local space scale for the triangles
    /// @param center_of_mass_transform1 Transform that takes the center of mass of 1 into world space
    /// @param center_of_mass_transform2 Transform that takes the center of mass of 2 into world space
    /// @param sub_shape_id1 Sub shape ID of the convex object
    /// @param collide_shape_settings Settings for the collide shape query
    /// @param collector The collector that will receive the results
    pub fn init(shape1: *const SphereShape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id1: SubShapeID, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector) CollideSphereVsTriangles {
        // Calculate the center of the sphere in the space of 2
        const sphere_center_in2 = center_of_mass_transform2.multiply3x3Transposed(center_of_mass_transform1.getTranslation().sub(center_of_mass_transform2.getTranslation()));

        // Determine if shape 2 is inside out or not
        const scale_sign2: f32 = if (ScaleHelpers.isInsideOut(scale2)) -1.0 else 1.0;

        // Check that the sphere is uniformly scaled
        std.debug.assert(ScaleHelpers.isUniformScale(scale1.abs()));
        const radius = @abs(scale1.getX()) * shape1.getRadius();
        const radius_plus_max_separation_sq = math.square(radius + collide_shape_settings.max_separation_distance);

        return .{
            .collide_shape_settings = collide_shape_settings,
            .collector = collector,
            .shape1 = shape1,
            .scale2 = scale2,
            .transform2 = center_of_mass_transform2,
            .sphere_center_in2 = sphere_center_in2,
            .sub_shape_id1 = sub_shape_id1,
            .scale_sign2 = scale_sign2,
            .radius = radius,
            .radius_plus_max_separation_sq = radius_plus_max_separation_sq,
        };
    }

    /// Collide sphere with a single triangle
    /// @param v0_in , v1_in , v2_in: CCW triangle vertices
    /// @param active_edges bit 0 = edge v0..v1 is active, bit 1 = edge v1..v2 is active, bit 2 = edge v2..v0 is active
    /// An active edge is an edge that is not connected to another triangle in such a way that it is impossible to collide with the edge
    /// @param sub_shape_id2 The sub shape ID for the triangle
    pub fn collide(self: *CollideSphereVsTriangles, v0_in: Vec3, v1_in: Vec3, v2_in: Vec3, active_edges: u8, sub_shape_id2: SubShapeID) void {
        const settings = self.collide_shape_settings;

        // Scale triangle and make it relative to the center of the sphere
        const v0 = self.scale2.mul(v0_in).sub(self.sphere_center_in2);
        const v1 = self.scale2.mul(v1_in).sub(self.sphere_center_in2);
        const v2 = self.scale2.mul(v2_in).sub(self.sphere_center_in2);

        // Calculate triangle normal
        const triangle_normal = v1.sub(v0).cross(v2.sub(v0)).mulScalar(self.scale_sign2);

        // Backface check
        const back_facing = triangle_normal.dot(v0) > 0.0;
        if (settings.back_face_mode == .ignore_back_faces and back_facing)
            return;

        // Check if we collide with the sphere
        const closest = ClosestPoint.getClosestPointOnTriangle(v0, v1, v2, .{});
        var point2 = closest.point;
        const closest_feature = closest.set;
        const point2_len_sq = point2.lengthSq();
        if (point2_len_sq > self.radius_plus_max_separation_sq)
            return;

        // Calculate penetration depth
        const penetration_depth = self.radius - @sqrt(point2_len_sq);
        if (-penetration_depth >= self.collector.getEarlyOutFraction())
            return;

        // Calculate penetration axis, direction along which to push 2 to move it out of collision (this is always away from the sphere center)
        var penetration_axis = point2.normalizedOr(Vec3.axisY());

        // Calculate the point on the sphere
        var point1 = penetration_axis.mulScalar(self.radius);

        // Check if we have enabled active edge detection
        std.debug.assert(closest_feature != 0);
        if (settings.active_edge_mode == .collide_only_with_active and closest_feature != 0b111 // For an interior hit we should already have the right normal
        and (active_edges & closest_feature_to_active_edges_mask[closest_feature]) == 0) // If we didn't hit an active edge we should take the triangle normal
        {
            // Convert the active edge velocity hint to local space
            const active_edge_movement_direction = self.transform2.multiply3x3Transposed(settings.active_edge_movement_direction);

            // See ActiveEdges::FixNormal. If penetration_axis affects the movement less than the triangle normal we keep penetration_axis.
            const new_penetration_axis = if (back_facing) triangle_normal else triangle_normal.negate();
            if (active_edge_movement_direction.dot(penetration_axis) * new_penetration_axis.length() >= active_edge_movement_direction.dot(new_penetration_axis))
                penetration_axis = new_penetration_axis;
        }

        // Convert to world space
        point1 = self.transform2.mulVec3(self.sphere_center_in2.add(point1));
        point2 = self.transform2.mulVec3(self.sphere_center_in2.add(point2));
        const penetration_axis_world = self.transform2.multiply3x3(penetration_axis);

        // Create collision result
        var result = CollideShapeResult.init(point1, point2, penetration_axis_world, penetration_depth, self.sub_shape_id1, sub_shape_id2, TransformedShape.getBodyID(self.collector.getContext()));

        // Gather faces
        if (settings.collect_faces_mode == .collect_faces) {
            // The sphere doesn't have a supporting face

            // Get face of triangle 2
            result.shape2_face.resize(3);
            result.shape2_face.buffer[0] = self.transform2.mulVec3(self.sphere_center_in2.add(v0));
            result.shape2_face.buffer[1] = self.transform2.mulVec3(self.sphere_center_in2.add(v1));
            result.shape2_face.buffer[2] = self.transform2.mulVec3(self.sphere_center_in2.add(v2));

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
// Tests (Jolt's TestSphereVsTriangles is in ZoltTests/Physics/ConvexVsTrianglesTest.zig, the bit exact comparison with
// Jolt in ZoltParity/Physics/TrianglesParity.zig)

const testing = std.testing;
const Quat = @import("../../Math/Quat.zig").Quat;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const AllHitCollisionCollector = @import("CollisionCollectorImpl.zig").AllHitCollisionCollector;

test "CollideSphereVsTriangles: interior, edges, vertices, active edges, scales, faces" {
    const allocator = testing.allocator;

    var sphere = SphereShape.init(allocator, 0.5, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();

    const v0 = Vec3.init(0, 0, 0);
    const v1 = Vec3.init(0, 0, 4);
    const v2 = Vec3.init(4, 0, 0);
    const transform2 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.2), Vec3.init(-1, 2, 0.5));
    var settings: CollideShapeSettings = .{};
    settings.collect_faces_mode = .collect_faces;
    var hits = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(9), .{});
    hits.base.setContext(&context);

    // Interior hit, the sphere (scale 2: radius 1) 0.25 above the triangle
    var interior = CollideSphereVsTriangles.init(&sphere, Vec3.replicate(-2), Vec3.one(), transform2.mul(Mat44.translation(Vec3.init(1, 0.25, 1))), transform2, .{ .value = 1 }, &settings, &hits.base);
    interior.collide(v0, v1, v2, 0b000, .{ .value = 2 });
    try hits.checkError();
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    var hit = hits.hits.items[0];
    try testing.expectApproxEqAbs(@as(f32, 0.75), hit.penetration_depth, 1.0e-6);
    try testing.expect(transform2.multiply3x3Transposed(hit.penetration_axis).isClose(Vec3.init(0, -1, 0), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(hit.body_id2.eql(.init(9)));
    try testing.expectEqual(@as(u32, 0), hit.shape1_face.len); // The sphere has no supporting face
    try testing.expectEqual(@as(u32, 3), hit.shape2_face.len);
    try testing.expect(hit.shape2_face.get(1).isClose(transform2.mulVec3(v1), .{ .max_dist_sq = 1.0e-10 }));

    // Back face: ignored by default
    hits.reset();
    interior.collide(v0, v2, v1, 0b111, .{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);

    // Inside out scale on the triangles: the reversed triangle is front facing, the face winding is swapped
    var inside_out = CollideSphereVsTriangles.init(&sphere, Vec3.replicate(2), Vec3.init(1, -1, 1), transform2.mul(Mat44.translation(Vec3.init(1, 0.25, 1))), transform2, .{}, &settings, &hits.base);
    inside_out.collide(v0, v2, v1, 0b111, .{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expect(hits.hits.items[0].shape2_face.get(1).isClose(transform2.mulVec3(v1), .{ .max_dist_sq = 1.0e-10 }));

    // Vertex v0 hit from the side: the triangle normal when edges 0 and 2 are inactive, unless the movement direction (into
    // the triangle) prefers the vertex normal
    hits.reset();
    var vertex = CollideSphereVsTriangles.init(&sphere, Vec3.one(), Vec3.one(), transform2.mul(Mat44.translation(Vec3.init(-0.3, 0.001, -0.3))), transform2, .{}, &settings, &hits.base);
    vertex.collide(v0, v1, v2, 0b010, .{});
    vertex.collide(v0, v1, v2, 0b001, .{});
    settings.active_edge_movement_direction = transform2.multiply3x3(Vec3.init(0, -1, 0));
    vertex.collide(v0, v1, v2, 0b010, .{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 3), hits.hits.items.len);
    try testing.expect(transform2.multiply3x3Transposed(hits.hits.items[0].penetration_axis).normalized().isClose(Vec3.init(0, -1, 0), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(transform2.multiply3x3Transposed(hits.hits.items[1].penetration_axis).normalized().isClose(Vec3.init(1, 0, 1).normalized(), .{ .max_dist_sq = 1.0e-4 }));
    try testing.expect(transform2.multiply3x3Transposed(hits.hits.items[2].penetration_axis).normalized().isClose(Vec3.init(1, 0, 1).normalized(), .{ .max_dist_sq = 1.0e-4 }));

    // Too far away, max separation distance and early out fraction
    hits.reset();
    settings.active_edge_movement_direction = Vec3.zero();
    var far = CollideSphereVsTriangles.init(&sphere, Vec3.one(), Vec3.one(), transform2.mul(Mat44.translation(Vec3.init(1, 1, 1))), transform2, .{}, &settings, &hits.base);
    far.collide(v0, v1, v2, 0b111, .{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
    settings.max_separation_distance = 0.6;
    var far2 = CollideSphereVsTriangles.init(&sphere, Vec3.one(), Vec3.one(), transform2.mul(Mat44.translation(Vec3.init(1, 1, 1))), transform2, .{}, &settings, &hits.base);
    far2.collide(v0, v1, v2, 0b111, .{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    hit = hits.hits.items[0];
    try testing.expectApproxEqAbs(@as(f32, -0.5), hit.penetration_depth, 1.0e-6);
    hits.base.updateEarlyOutFraction(0.4); // Only hits with a penetration depth > -0.4
    far2.collide(v0, v1, v2, 0b111, .{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);

    // Sphere center exactly on the triangle: the penetration axis falls back to Y
    hits.reset();
    var on = CollideSphereVsTriangles.init(&sphere, Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(1, 0, 1)), Mat44.identity(), .{}, &settings, &hits.base);
    on.collide(v0, v1, v2, 0b111, .{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expect(hits.hits.items[0].penetration_axis.eql(Vec3.axisY()));
    try testing.expectEqual(@as(f32, 0.5), hits.hits.items[0].penetration_depth);
}
