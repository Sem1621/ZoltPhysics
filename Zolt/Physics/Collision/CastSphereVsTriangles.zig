//! Port of: Jolt/Physics/Collision/CastSphereVsTriangles.h, Jolt/Physics/Collision/CastSphereVsTriangles.cpp
//! Status: complete
//!
//! A helper class that the triangle shapes construct on the stack and feed with triangles (`cast`), see
//! CastConvexVsTriangles.zig. The references `const ShapeCastSettings &` / `CastShapeCollector &` are pointers, Jolt's
//! `const Mat44 &mCenterOfMassTransform2` (a reference to the constructor argument) is a copy with the same value.
//! The private member functions `AddHit`, `AddHitWithActiveEdgeDetection` and `RayCylinder` are private functions.

const std = @import("std");
const ClosestPoint = @import("../../Geometry/ClosestPoint.zig");
const RaySphere = @import("../../Geometry/RaySphere.zig");
const math = @import("../../Math/Math.zig");
const Core = @import("../../Core/Core.zig");
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const ShapeFile = @import("Shape/Shape.zig");
const CastShapeCollector = ShapeFile.CastShapeCollector;
const SphereShape = @import("Shape/SphereShape.zig").SphereShape;
const ScaleHelpers = @import("Shape/ScaleHelpers.zig");
const SubShapeIDFile = @import("Shape/SubShapeID.zig");
const SubShapeID = SubShapeIDFile.SubShapeID;
const SubShapeIDCreator = SubShapeIDFile.SubShapeIDCreator;
const ShapeCastFile = @import("ShapeCast.zig");
const ShapeCast = ShapeCastFile.ShapeCast;
const ShapeCastSettings = ShapeCastFile.ShapeCastSettings;
const ShapeCastResult = ShapeCastFile.ShapeCastResult;
const TransformedShape = @import("TransformedShape.zig").TransformedShape;
const ActiveEdges = @import("ActiveEdges.zig");
const NarrowPhaseStats = @import("NarrowPhaseStats.zig");
const TrackNarrowPhaseCollector = NarrowPhaseStats.TrackNarrowPhaseCollector;
const track_narrowphase_stats = NarrowPhaseStats.track_narrowphase_stats;

/// Collision detection helper that casts a sphere vs one or more triangles
pub const CastSphereVsTriangles = struct {
    /// Starting location of the sphere
    start: Vec3,
    /// Direction and length of movement of sphere
    direction: Vec3,
    /// Scaled radius of sphere
    radius: f32,
    shape_cast_settings: *const ShapeCastSettings,
    center_of_mass_transform2: Mat44,
    scale: Vec3,
    sub_shape_id_creator1: SubShapeIDCreator,
    collector: *CastShapeCollector,

    /// Sign of the scale, -1 if object is inside out, 1 if not
    scale_sign: f32,

    /// Constructor
    /// @param shape_cast The sphere to cast against the triangles and its start and direction
    /// @param shape_cast_settings Settings for performing the cast
    /// @param scale Local space scale for the shape to cast against (scales relative to its center of mass).
    /// @param center_of_mass_transform2 Is the center of mass transform of shape 2 (excluding scale), this is used to provide a transform to the shape cast result so that local quantities can be transformed into world space.
    /// @param sub_shape_id_creator1 Class that tracks the current sub shape ID for the casting shape
    /// @param collector The collector that receives the results.
    pub fn init(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, scale: Vec3, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, collector: *CastShapeCollector) CastSphereVsTriangles {
        // Cast to sphere shape
        std.debug.assert(shape_cast.shape.getSubType() == .sphere);
        const sphere = shape_cast.shape.cast(SphereShape);

        return .{
            .start = shape_cast.center_of_mass_start.getTranslation(),
            .direction = shape_cast.direction,
            // Scale the radius
            .radius = sphere.getRadius() * @abs(shape_cast.scale.getX()) + shape_cast_settings.extra_convex_radius,
            .shape_cast_settings = shape_cast_settings,
            .center_of_mass_transform2 = center_of_mass_transform2,
            .scale = scale,
            .sub_shape_id_creator1 = sub_shape_id_creator1,
            .collector = collector,
            // Determine if shape is inside out or not
            .scale_sign = if (ScaleHelpers.isInsideOut(scale)) -1.0 else 1.0,
        };
    }

    fn addHit(self: *CastSphereVsTriangles, back_facing: bool, sub_shape_id2: SubShapeID, fraction: f32, contact_point_a_in: Vec3, contact_point_b_in: Vec3, contact_normal: Vec3) void {
        // Convert to world space
        const contact_point_a = self.center_of_mass_transform2.mulVec3(self.start.add(contact_point_a_in));
        const contact_point_b = self.center_of_mass_transform2.mulVec3(self.start.add(contact_point_b_in));
        const contact_normal_world = self.center_of_mass_transform2.multiply3x3(contact_normal);

        // Its a hit, store the sub shape id's
        const result = ShapeCastResult.init(fraction, contact_point_a, contact_point_b, contact_normal_world, back_facing, self.sub_shape_id_creator1.getID(), sub_shape_id2, TransformedShape.getBodyID(self.collector.getContext()));

        // Note: We don't gather faces here because that's only useful if both shapes have a face. Since the sphere always has only 1 contact point, the manifold is always a point.

        var track: if (track_narrowphase_stats) TrackNarrowPhaseCollector else void = undefined;
        if (track_narrowphase_stats) track = .init();
        defer if (track_narrowphase_stats) track.deinit();
        self.collector.addHit(&result);
    }

    fn addHitWithActiveEdgeDetection(self: *CastSphereVsTriangles, v0: Vec3, v1: Vec3, v2: Vec3, back_facing: bool, triangle_normal: Vec3, active_edges: u8, sub_shape_id2: SubShapeID, fraction: f32, contact_point_a: Vec3, contact_point_b: Vec3, contact_normal_in: Vec3) void {
        // Check if we have enabled active edge detection
        var contact_normal = contact_normal_in;
        if (self.shape_cast_settings.active_edge_mode == .collide_only_with_active and active_edges != 0b111) {
            // Convert the active edge velocity hint to local space
            const active_edge_movement_direction = self.center_of_mass_transform2.multiply3x3Transposed(self.shape_cast_settings.active_edge_movement_direction);

            // Update the contact normal to account for active edges
            // Note that we flip the triangle normal as the penetration axis is pointing towards the triangle instead of away
            contact_normal = ActiveEdges.fixNormal(v0, v1, v2, if (back_facing) triangle_normal else triangle_normal.negate(), active_edges, contact_point_b, contact_normal_in, active_edge_movement_direction);
        }

        self.addHit(back_facing, sub_shape_id2, fraction, contact_point_a, contact_point_b, contact_normal);
    }

    // This is a simplified version of the ray cylinder test from: Real Time Collision Detection - Christer Ericson
    // Chapter 5.3.7, page 194-197. Some conditions have been removed as we're not interested in hitting the caps of the cylinder.
    // Note that the ray origin is assumed to be the origin here.
    fn rayCylinder(self: *const CastSphereVsTriangles, ray_direction: Vec3, cylinder_a: Vec3, cylinder_b: Vec3, radius: f32) f32 {
        _ = self;

        // Calculate cylinder axis
        const axis = cylinder_b.sub(cylinder_a);

        // Make ray start relative to cylinder side A (moving cylinder A to the origin)
        const start = cylinder_a.negate();

        // Test if segment is fully on the A side of the cylinder
        const start_dot_axis = start.dot(axis);
        const direction_dot_axis = ray_direction.dot(axis);
        const end_dot_axis = start_dot_axis + direction_dot_axis;
        if (start_dot_axis < 0.0 and end_dot_axis < 0.0)
            return math.flt_max;

        // Test if segment is fully on the B side of the cylinder
        const axis_len_sq = axis.lengthSq();
        if (start_dot_axis > axis_len_sq and end_dot_axis > axis_len_sq)
            return math.flt_max;

        // Calculate a, b and c, the factors for quadratic equation
        // We're basically solving the ray: x = start + direction * t
        // The closest point to x on the segment A B is: w = (x . axis) * axis / (axis . axis)
        // The distance between x and w should be radius: (x - w) . (x - w) = radius^2
        // Solving this gives the following:
        const a = axis_len_sq * ray_direction.lengthSq() - math.square(direction_dot_axis);
        if (@abs(a) < 1.0e-6)
            return math.flt_max; // Segment runs parallel to cylinder axis, stop processing, we will either hit at fraction = 0 or we'll hit a vertex
        const b = axis_len_sq * start.dot(ray_direction) - direction_dot_axis * start_dot_axis; // should be multiplied by 2, instead we'll divide a and c by 2 when we solve the quadratic equation
        const c = axis_len_sq * (start.lengthSq() - math.square(radius)) - math.square(start_dot_axis);
        const det = math.differenceOfProducts(b, b, a, c); // normally 4 * a * c but since both a and c need to be divided by 2 we lose the 4
        if (det < 0.0)
            return math.flt_max; // No solution to quadratic equation

        // Solve fraction t where the ray hits the cylinder
        const t = -(b + @sqrt(det)) / a; // normally divided by 2 * a but since a should be divided by 2 we lose the 2
        if (t < 0.0 or t > 1.0)
            return math.flt_max; // Intersection lies outside segment
        if (start_dot_axis + t * direction_dot_axis < 0.0 or start_dot_axis + t * direction_dot_axis > axis_len_sq)
            return math.flt_max; // Intersection outside the end point of the cylinder, stop processing, we will possibly hit a vertex
        return t;
    }

    /// Cast sphere with a single triangle
    /// @param v0_in , v1_in , v2_in: CCW triangle vertices
    /// @param active_edges bit 0 = edge v0..v1 is active, bit 1 = edge v1..v2 is active, bit 2 = edge v2..v0 is active
    /// An active edge is an edge that is not connected to another triangle in such a way that it is impossible to collide with the edge
    /// @param sub_shape_id2 The sub shape ID for the triangle
    pub fn cast(self: *CastSphereVsTriangles, v0_in: Vec3, v1_in: Vec3, v2_in: Vec3, active_edges: u8, sub_shape_id2: SubShapeID) void {
        // Scale triangle and make it relative to the start of the cast
        const v0 = self.scale.mul(v0_in).sub(self.start);
        const v1 = self.scale.mul(v1_in).sub(self.start);
        const v2 = self.scale.mul(v2_in).sub(self.start);

        // Calculate triangle normal
        var triangle_normal = v1.sub(v0).cross(v2.sub(v0)).mulScalar(self.scale_sign);
        const triangle_normal_len = triangle_normal.length();
        if (triangle_normal_len == 0.0)
            return; // Degenerate triangle
        triangle_normal = triangle_normal.divScalar(triangle_normal_len);

        // Backface check
        const normal_dot_direction = triangle_normal.dot(self.direction);
        const back_facing = normal_dot_direction > 0.0;
        if (self.shape_cast_settings.back_face_mode_triangles == .ignore_back_faces and back_facing)
            return;

        // Test if distance between the sphere and plane of triangle is smaller or equal than the radius
        if (@abs(v0.dot(triangle_normal)) <= self.radius) {
            // Check if the sphere intersects at the start of the cast
            const q = ClosestPoint.getClosestPointOnTriangle(v0, v1, v2, .{}).point;
            const q_len_sq = q.lengthSq();
            if (q_len_sq <= math.square(self.radius)) {
                // Early out if this hit is deeper than the collector's early out value
                const q_len = @sqrt(q_len_sq);
                const penetration_depth = self.radius - q_len;
                if (-penetration_depth >= self.collector.getEarlyOutFraction())
                    return;

                // Generate contact point
                const contact_normal = if (q_len > 0.0) q.divScalar(q_len) else Vec3.axisY();
                const contact_point_a = q.add(contact_normal.mulScalar(penetration_depth));
                const contact_point_b = q;
                self.addHitWithActiveEdgeDetection(v0, v1, v2, back_facing, triangle_normal, active_edges, sub_shape_id2, 0.0, contact_point_a, contact_point_b, contact_normal);
                return;
            }
        } else {
            // Check if cast is not parallel to the plane of the triangle
            const abs_normal_dot_direction = @abs(normal_dot_direction);
            if (abs_normal_dot_direction > 1.0e-6) {
                // Calculate the point on the sphere that will hit the triangle's plane first and calculate a fraction where it will do so
                const d = triangle_normal.mulScalar(math.sign(normal_dot_direction) * self.radius);
                const plane_intersection = v0.sub(d).dot(triangle_normal) / normal_dot_direction;

                // Check if sphere will hit in the interval that we're interested in
                if (plane_intersection * abs_normal_dot_direction < -self.radius // Sphere hits the plane before the sweep, cannot intersect
                or plane_intersection >= self.collector.getEarlyOutFraction()) // Sphere hits the plane after the sweep / early out fraction, cannot intersect
                    return;

                // We can only report an interior hit if we're hitting the plane during our sweep and not before
                if (plane_intersection >= 0.0) {
                    // Calculate the point of contact on the plane
                    const p = d.add(self.direction.mulScalar(plane_intersection));

                    // Check if this is an interior point
                    const bary = ClosestPoint.getBaryCentricCoordinatesTriangle(v0.sub(p), v1.sub(p), v2.sub(p));
                    if (bary.valid and bary.u >= 0.0 and bary.v >= 0.0 and bary.w >= 0.0) {
                        // Interior point, we found the collision point. We don't need to check active edges.
                        self.addHit(back_facing, sub_shape_id2, plane_intersection, p, p, if (back_facing) triangle_normal else triangle_normal.negate());
                        return;
                    }
                }
            }
        }

        // Test 3 edges
        var fraction = self.rayCylinder(self.direction, v0, v1, self.radius);
        fraction = math.min(fraction, self.rayCylinder(self.direction, v1, v2, self.radius));
        fraction = math.min(fraction, self.rayCylinder(self.direction, v2, v0, self.radius));

        // Test 3 vertices
        fraction = math.min(fraction, RaySphere.raySphere(Vec3.zero(), self.direction, v0, self.radius));
        fraction = math.min(fraction, RaySphere.raySphere(Vec3.zero(), self.direction, v1, self.radius));
        fraction = math.min(fraction, RaySphere.raySphere(Vec3.zero(), self.direction, v2, self.radius));

        // Check if we have a collision
        if (Core.enable_asserts) std.debug.assert(fraction >= 0.0); // Large finite input can make RayCylinder return NaN; Jolt's release build then skips the hit
        if (fraction < self.collector.getEarlyOutFraction()) {
            // Calculate the center of the sphere at the point of contact
            const p = self.direction.mulScalar(fraction);

            // Get contact point and normal
            const q = ClosestPoint.getClosestPointOnTriangle(v0.sub(p), v1.sub(p), v2.sub(p), .{}).point;
            // The distance between p and the triangle should be mRadius, but for very long casts,
            // floating point accuracy can become low enough so that p is on the plane and q is zero
            const contact_normal = q.normalizedOr(if (back_facing) triangle_normal else triangle_normal.negate());
            const contact_point_ab = p.add(q);
            self.addHitWithActiveEdgeDetection(v0, v1, v2, back_facing, triangle_normal, active_edges, sub_shape_id2, fraction, contact_point_ab, contact_point_ab, contact_normal);
        }
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt's TestCastSphereTriangle / TestCastSphereVsDegenerateTriangle are in ZoltTests/Physics/CastShapeTests.zig,
// the bit exact comparison with Jolt is in ZoltParity/Physics/TrianglesParity.zig)

const testing = std.testing;
const Quat = @import("../../Math/Quat.zig").Quat;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const AllHitCollisionCollector = @import("CollisionCollectorImpl.zig").AllHitCollisionCollector;

test "CastSphereVsTriangles: interior, edge and vertex hits, initial overlap, back faces, degenerate triangles" {
    const allocator = testing.allocator;

    var sphere = SphereShape.init(allocator, 0.5, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();

    const v0 = Vec3.init(0, 0, 0);
    const v1 = Vec3.init(0, 0, 4);
    const v2 = Vec3.init(4, 0, 0);
    const transform2 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisX(), -0.4), Vec3.init(0.5, -1, 2));
    var settings: ShapeCastSettings = .{};
    var hits = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(3), .{});
    hits.base.setContext(&context);
    const creator = SubShapeIDCreator.pushID(.{}, 2, 2);

    // Interior hit: the sphere (scale -2, radius 1) starts 3 above the interior, moving 4 down: fraction (3 - 1) / 4
    const down = ShapeCast.init(sphere.asShape(), Vec3.replicate(-2), Mat44.translation(Vec3.init(1, 3, 1)), Vec3.init(0, -4, 0));
    var caster = CastSphereVsTriangles.init(&down, &settings, Vec3.one(), transform2, creator, &hits.base);
    caster.cast(v0, v1, v2, 0b111, .{ .value = 1 });
    try hits.checkError();
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    var hit = hits.hits.items[0];
    try testing.expectEqual(@as(f32, 0.5), hit.fraction);
    try testing.expect(!hit.is_back_face_hit);
    try testing.expect(transform2.multiply3x3Transposed(hit.base.penetration_axis).isClose(Vec3.init(0, -1, 0), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(hit.base.contact_point_on2.isClose(transform2.mulVec3(Vec3.init(1, 0, 1)), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectEqual(creator.getID().getValue(), hit.base.sub_shape_id1.getValue());
    try testing.expectEqual(@as(u32, 1), hit.base.sub_shape_id2.getValue());
    try testing.expect(hit.base.body_id2.eql(.init(3)));
    try testing.expectEqual(@as(u32, 0), hit.base.shape1_face.len); // No faces for spheres

    // Back face (reversed winding): ignored by default
    hits.reset();
    caster.cast(v0, v2, v1, 0b111, .{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
    settings.back_face_mode_triangles = .collide_with_back_faces;
    caster.cast(v0, v2, v1, 0b111, .{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expect(hits.hits.items[0].is_back_face_hit);

    // Degenerate triangle: no hit
    hits.reset();
    caster.cast(v0, v1, v1, 0b111, .{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);

    // Edge hit: moving along -x towards the edge v0..v1 (x = 0) from the side, in the plane of the triangle
    const side = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(-2, 0.75, 2)), Vec3.init(4, -1, 0));
    var side_caster = CastSphereVsTriangles.init(&side, &settings, Vec3.one(), transform2, creator, &hits.base);
    side_caster.cast(v0, v1, v2, 0b111, .{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    hit = hits.hits.items[0];
    try testing.expect(hit.fraction > 0.0 and hit.fraction < 1.0);
    try testing.expect(transform2.multiply3x3Transposed(hit.base.contact_point_on2).sub(transform2.multiply3x3Transposed(transform2.getTranslation())).isClose(Vec3.init(0, 0, 2), .{ .max_dist_sq = 1.0e-8 }));

    // Vertex hit: aiming at v2 from beyond it, edges inactive: the triangle normal is used
    hits.reset();
    const vertex = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(6, 0.1, -2)), Vec3.init(-4, 0, 4));
    var vertex_caster = CastSphereVsTriangles.init(&vertex, &settings, Vec3.one(), transform2, creator, &hits.base);
    vertex_caster.cast(v0, v1, v2, 0b000, .{});
    settings.active_edge_mode = .collide_with_all;
    vertex_caster.cast(v0, v1, v2, 0b000, .{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
    try testing.expectEqual(hits.hits.items[0].fraction, hits.hits.items[1].fraction);
    try testing.expect(transform2.multiply3x3Transposed(hits.hits.items[0].base.penetration_axis).isClose(Vec3.init(0, -1, 0), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(!transform2.multiply3x3Transposed(hits.hits.items[1].base.penetration_axis).isClose(Vec3.init(0, -1, 0), .{ .max_dist_sq = 1.0e-2 }));

    // Initially overlapping: fraction 0 with the penetration depth, rejected by an early out fraction that is deeper
    hits.reset();
    const overlap = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(1, 0.25, 1)), Vec3.init(0, 1, 0));
    var overlap_caster = CastSphereVsTriangles.init(&overlap, &settings, Vec3.one(), transform2, creator, &hits.base);
    overlap_caster.cast(v0, v1, v2, 0b111, .{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.0), hits.hits.items[0].fraction);
    try testing.expectApproxEqAbs(@as(f32, 0.25), hits.hits.items[0].base.penetration_depth, 1.0e-6);
    hits.base.updateEarlyOutFraction(-0.3);
    overlap_caster.cast(v0, v1, v2, 0b111, .{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);

    // Missing: moving parallel to the triangle above it, or away from it
    hits.reset();
    const parallel = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(-2, 1, 1)), Vec3.init(8, 0, 0));
    var parallel_caster = CastSphereVsTriangles.init(&parallel, &settings, Vec3.one(), transform2, creator, &hits.base);
    parallel_caster.cast(v0, v1, v2, 0b111, .{});
    const away = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(1, 1, 1)), Vec3.init(0, 2, 0));
    var away_caster = CastSphereVsTriangles.init(&away, &settings, Vec3.one(), transform2, creator, &hits.base);
    away_caster.cast(v0, v1, v2, 0b111, .{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
}
