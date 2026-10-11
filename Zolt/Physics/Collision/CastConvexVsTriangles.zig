//! Port of: Jolt/Physics/Collision/CastConvexVsTriangles.h, Jolt/Physics/Collision/CastConvexVsTriangles.cpp
//! Status: complete
//!
//! A helper class that the triangle shapes construct on the stack and feed with triangles (`cast`). Jolt's
//! `struct Visitor : public CastConvexVsTriangles` becomes a struct that embeds it as `base`.
//! - The references `const ShapeCast &`, `const ShapeCastSettings &` and `CastShapeCollector &` are pointers. Jolt's
//!   `const Mat44 &mCenterOfMassTransform2` refers to the constructor argument (a `Mat44Arg`, which lives in the caller
//!   during the whole cast), here it is a copy with the same value.
//! - The support function of the cast shape is constructed lazily in the SupportBuffer inside this struct and the
//!   struct keeps a pointer into it (D9): after the first `cast` it must not be moved.
//! - `cast` is non-const in Jolt (it creates the support function): `self: *CastConvexVsTriangles`.

const std = @import("std");
const ConvexSupport = @import("../../Geometry/ConvexSupport.zig");
const TriangleConvexSupport = ConvexSupport.TriangleConvexSupport;
const EPAPenetrationDepth = @import("../../Geometry/EPAPenetrationDepth.zig").EPAPenetrationDepth;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const ShapeFile = @import("Shape/Shape.zig");
const CastShapeCollector = ShapeFile.CastShapeCollector;
const ConvexShape = @import("Shape/ConvexShape.zig").ConvexShape;
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

/// Collision detection helper that casts a convex object vs one or more triangles
pub const CastConvexVsTriangles = struct {
    shape_cast: *const ShapeCast,
    shape_cast_settings: *const ShapeCastSettings,
    center_of_mass_transform2: Mat44,
    scale: Vec3,
    sub_shape_id_creator1: SubShapeIDCreator,
    collector: *CastShapeCollector,

    /// Buffer that holds the support function of the cast shape
    support_buffer: ConvexShape.SupportBuffer = .{},
    /// Support function of the cast shape
    support: ?*const ConvexShape.Support = null,
    /// Sign of the scale, -1 if object is inside out, 1 if not
    scale_sign: f32,

    /// Constructor
    /// @param shape_cast The shape to cast against the triangles and its start and direction
    /// @param shape_cast_settings Settings for performing the cast
    /// @param scale Local space scale for the shape to cast against (scales relative to its center of mass).
    /// @param center_of_mass_transform2 Is the center of mass transform of shape 2 (excluding scale), this is used to provide a transform to the shape cast result so that local quantities can be transformed into world space.
    /// @param sub_shape_id_creator1 Class that tracks the current sub shape ID for the casting shape
    /// @param collector The collector that receives the results.
    pub fn init(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, scale: Vec3, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, collector: *CastShapeCollector) CastConvexVsTriangles {
        std.debug.assert(shape_cast.shape.getType() == .convex);

        return .{
            .shape_cast = shape_cast,
            .shape_cast_settings = shape_cast_settings,
            .center_of_mass_transform2 = center_of_mass_transform2,
            .scale = scale,
            .sub_shape_id_creator1 = sub_shape_id_creator1,
            .collector = collector,
            // Determine if shape is inside out or not
            .scale_sign = if (ScaleHelpers.isInsideOut(scale)) -1.0 else 1.0,
        };
    }

    /// Cast convex object with a single triangle
    /// @param v0_in , v1_in , v2_in: CCW triangle vertices
    /// @param active_edges bit 0 = edge v0..v1 is active, bit 1 = edge v1..v2 is active, bit 2 = edge v2..v0 is active
    /// An active edge is an edge that is not connected to another triangle in such a way that it is impossible to collide with the edge
    /// @param sub_shape_id2 The sub shape ID for the triangle
    pub fn cast(self: *CastConvexVsTriangles, v0_in: Vec3, v1_in: Vec3, v2_in: Vec3, active_edges: u8, sub_shape_id2: SubShapeID) void {
        const settings = self.shape_cast_settings;
        const shape_cast = self.shape_cast;

        // Scale triangle
        const v0 = self.scale.mul(v0_in);
        const v1 = self.scale.mul(v1_in);
        const v2 = self.scale.mul(v2_in);

        // Calculate triangle normal
        const triangle_normal = v1.sub(v0).cross(v2.sub(v0)).mulScalar(self.scale_sign);

        // Backface check
        const back_facing = triangle_normal.dot(shape_cast.direction) > 0.0;
        if (settings.back_face_mode_triangles == .ignore_back_faces and back_facing)
            return;

        // Create triangle support function
        const triangle = TriangleConvexSupport.init(v0, v1, v2);

        // Check if we already created the cast shape support function
        if (self.support == null) {
            // Determine if we want to use the actual shape or a shrunken shape with convex radius
            const support_mode: ConvexShape.SupportMode = if (settings.use_shrunken_shape_and_convex_radius) .exclude_convex_radius else .default;

            // Create support function
            self.support = shape_cast.shape.cast(ConvexShape).getSupportFunction(support_mode, &self.support_buffer, shape_cast.scale);
        }
        const support = self.support.?;

        var epa: EPAPenetrationDepth = .{};
        var fraction = self.collector.getEarlyOutFraction();
        var contact_point_a: Vec3 = undefined;
        var contact_point_b: Vec3 = undefined;
        var contact_normal: Vec3 = undefined;
        if (epa.castShape(shape_cast.center_of_mass_start, shape_cast.direction, settings.collision_tolerance, settings.penetration_tolerance, support, &triangle, support.getConvexRadius() + settings.extra_convex_radius, 0.0, settings.return_deepest_point, &fraction, &contact_point_a, &contact_point_b, &contact_normal)) {
            // Check if we have enabled active edge detection
            if (settings.active_edge_mode == .collide_only_with_active and active_edges != 0b111) {
                // Convert the active edge velocity hint to local space
                const active_edge_movement_direction = self.center_of_mass_transform2.multiply3x3Transposed(settings.active_edge_movement_direction);

                // Update the contact normal to account for active edges
                // Note that we flip the triangle normal as the penetration axis is pointing towards the triangle instead of away
                contact_normal = ActiveEdges.fixNormal(v0, v1, v2, if (back_facing) triangle_normal else triangle_normal.negate(), active_edges, contact_point_b, contact_normal, active_edge_movement_direction);
            }

            // Convert to world space
            contact_point_a = self.center_of_mass_transform2.mulVec3(contact_point_a);
            contact_point_b = self.center_of_mass_transform2.mulVec3(contact_point_b);
            const contact_normal_world = self.center_of_mass_transform2.multiply3x3(contact_normal);

            // Its a hit, store the sub shape id's
            var result = ShapeCastResult.init(fraction, contact_point_a, contact_point_b, contact_normal_world, back_facing, self.sub_shape_id_creator1.getID(), sub_shape_id2, TransformedShape.getBodyID(self.collector.getContext()));

            // Early out if this hit is deeper than the collector's early out value
            if (fraction == 0.0 and -result.base.penetration_depth >= self.collector.getEarlyOutFraction())
                return;

            // Gather faces
            if (settings.collect_faces_mode == .collect_faces) {
                // Get supporting face of shape 1
                var transform_1_to_2 = shape_cast.center_of_mass_start;
                transform_1_to_2.setTranslation(transform_1_to_2.getTranslation().add(shape_cast.direction.mulScalar(fraction)));
                shape_cast.shape.cast(ConvexShape).base.getSupportingFace(.empty, transform_1_to_2.multiply3x3Transposed(contact_normal.negate()), shape_cast.scale, self.center_of_mass_transform2.mul(transform_1_to_2), &result.base.shape1_face);

                // Get face of the triangle
                result.base.shape2_face.resize(3);
                result.base.shape2_face.buffer[0] = self.center_of_mass_transform2.mulVec3(v0);
                result.base.shape2_face.buffer[1] = self.center_of_mass_transform2.mulVec3(v1);
                result.base.shape2_face.buffer[2] = self.center_of_mass_transform2.mulVec3(v2);

                // When inside out, we need to swap the triangle winding
                if (self.scale_sign < 0.0)
                    std.mem.swap(Vec3, &result.base.shape2_face.buffer[1], &result.base.shape2_face.buffer[2]);
            }

            var track: if (track_narrowphase_stats) TrackNarrowPhaseCollector else void = undefined;
            if (track_narrowphase_stats) track = .init();
            defer if (track_narrowphase_stats) track.deinit();
            self.collector.addHit(&result);
        }
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the bit exact comparison with Jolt is in ZoltParity/Physics/TrianglesParity.zig)

const testing = std.testing;
const Quat = @import("../../Math/Quat.zig").Quat;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const BoxShape = @import("Shape/BoxShape.zig").BoxShape;
const AllHitCollisionCollector = @import("CollisionCollectorImpl.zig").AllHitCollisionCollector;

test "CastConvexVsTriangles: box cast onto a triangle, back faces, faces, initial overlap, active edges" {
    const allocator = testing.allocator;

    var box = BoxShape.init(allocator, Vec3.replicate(0.5), .{});
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();

    const v0 = Vec3.init(-5, 0, -5);
    const v1 = Vec3.init(0, 0, 5);
    const v2 = Vec3.init(5, 0, -5);
    const transform2 = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.7), Vec3.init(3, 2, 1));
    var settings: ShapeCastSettings = .{};
    settings.collect_faces_mode = .collect_faces;
    var hits = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer hits.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(4), .{});
    hits.base.setContext(&context);

    // Box 2 above the triangle (local space of the triangles), moving 4 down: hits at fraction (2 - 0.5) / 4
    const shape_cast = ShapeCast.init(box.asShape(), Vec3.one(), Mat44.translation(Vec3.init(0, 2, 0)), Vec3.init(0, -4, 0));
    const creator = SubShapeIDCreator.pushID(.{}, 1, 2);
    var caster = CastConvexVsTriangles.init(&shape_cast, &settings, Vec3.one(), transform2, creator, &hits.base);
    caster.cast(v0, v1, v2, 0b111, .{ .value = 8 });
    try hits.checkError();
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    const hit = hits.hits.items[0];
    try testing.expectApproxEqAbs(@as(f32, 0.375), hit.fraction, 1.0e-4);
    try testing.expect(!hit.is_back_face_hit);
    try testing.expectEqual(creator.getID().getValue(), hit.base.sub_shape_id1.getValue());
    try testing.expectEqual(@as(u32, 8), hit.base.sub_shape_id2.getValue());
    try testing.expect(hit.base.body_id2.eql(.init(4)));
    try testing.expectEqual(@as(u32, 4), hit.base.shape1_face.len);
    try testing.expectEqual(@as(u32, 3), hit.base.shape2_face.len);
    try testing.expect(hit.base.shape2_face.get(2).isClose(transform2.mulVec3(v2), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(transform2.multiply3x3Transposed(hit.base.penetration_axis).normalized().isClose(Vec3.init(0, -1, 0), .{ .max_dist_sq = 1.0e-6 }));

    // The support function was created once
    const support = caster.support;
    caster.cast(v0, v1, v2, 0b111, .{});
    try testing.expect(caster.support == support);

    // Moving away from the back side of a reversed triangle: back facing, ignored by default
    hits.reset();
    caster.cast(v0, v2, v1, 0b111, .{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);
    settings.back_face_mode_triangles = .collide_with_back_faces;
    caster.cast(v0, v2, v1, 0b111, .{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expect(hits.hits.items[0].is_back_face_hit);

    // Inside out scale: the reversed triangle is front facing, the face winding is swapped back
    hits.reset();
    settings.back_face_mode_triangles = .ignore_back_faces;
    var inside_out = CastConvexVsTriangles.init(&shape_cast, &settings, Vec3.init(1, 1, -1), transform2, creator, &hits.base);
    const m = Vec3.init(1, 1, -1);
    inside_out.cast(v0.mul(m), v2.mul(m), v1.mul(m), 0b111, .{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expect(hits.hits.items[0].base.shape2_face.get(1).isClose(transform2.mulVec3(v1), .{ .max_dist_sq = 1.0e-10 }));

    // Initially overlapping (fraction 0) with the deepest point, and the shrunken shape
    hits.reset();
    settings.return_deepest_point = true;
    settings.use_shrunken_shape_and_convex_radius = true;
    const overlapping = ShapeCast.init(box.asShape(), Vec3.one(), Mat44.translation(Vec3.init(0, 0.25, 0)), Vec3.init(0, -1, 0));
    var overlap_caster = CastConvexVsTriangles.init(&overlapping, &settings, Vec3.one(), transform2, creator, &hits.base);
    overlap_caster.cast(v0, v1, v2, 0b111, .{});
    try testing.expectEqual(@as(usize, 1), hits.hits.items.len);
    try testing.expectEqual(@as(f32, 0.0), hits.hits.items[0].fraction);
    try testing.expectApproxEqAbs(@as(f32, 0.25), hits.hits.items[0].base.penetration_depth, 1.0e-4);

    // Missing the triangle
    hits.reset();
    const miss = ShapeCast.init(box.asShape(), Vec3.one(), Mat44.translation(Vec3.init(20, 2, 0)), Vec3.init(0, -4, 0));
    var miss_caster = CastConvexVsTriangles.init(&miss, &settings, Vec3.one(), transform2, creator, &hits.base);
    miss_caster.cast(v0, v1, v2, 0b111, .{});
    try testing.expectEqual(@as(usize, 0), hits.hits.items.len);

    // Sliding sideways into the edge v1..v2 (inactive): the triangle normal is used
    hits.reset();
    settings.return_deepest_point = false;
    const side = ShapeCast.init(box.asShape(), Vec3.one(), Mat44.translation(Vec3.init(5, -0.25, 3)), Vec3.init(-4, 0, 0));
    var side_caster = CastConvexVsTriangles.init(&side, &settings, Vec3.one(), transform2, creator, &hits.base);
    side_caster.cast(v0, v1, v2, 0b000, .{});
    settings.active_edge_mode = .collide_with_all;
    side_caster.cast(v0, v1, v2, 0b000, .{});
    try hits.checkError();
    try testing.expectEqual(@as(usize, 2), hits.hits.items.len);
    try testing.expect(transform2.multiply3x3Transposed(hits.hits.items[0].base.penetration_axis).normalized().isClose(Vec3.init(0, -1, 0), .{ .max_dist_sq = 1.0e-6 }));
    try testing.expect(!transform2.multiply3x3Transposed(hits.hits.items[1].base.penetration_axis).normalized().isClose(Vec3.init(0, -1, 0), .{ .max_dist_sq = 1.0e-2 }));
}
