//! Port of: UnitTests/Physics/TransformedShapeTests.cpp
//! Status: partial
//! Missing: creating the body in a PhysicsTestContext and collecting its leaf shape with
//!   NarrowPhaseQuery::CollectTransformedShapes in TestTransformedShape (need PhysicsTestContext / NarrowPhaseQuery,
//!   Phase 5). The port starts from the TransformedShape the body would have (Body::GetTransformedShape) and collects the
//!   leaf shape with TransformedShape::CollectTransformedShapes, which is what NarrowPhaseQuery does per body.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const BodyID = zolt.BodyID;
const BoxShapeSettings = zolt.BoxShapeSettings;
const Color = zolt.Color;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Mat44 = zolt.Mat44;
const PhysicsMaterialSimple = zolt.PhysicsMaterialSimple;
const Quat = zolt.Quat;
const RayCastResult = zolt.RayCastResult;
const RMat44 = zolt.RMat44;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const RRayCast = zolt.RRayCast;
const RVec3 = zolt.RVec3;
const ScaledShapeSettings = zolt.ScaledShapeSettings;
const TransformedShape = zolt.TransformedShape;
const TransformedShapeCollector = zolt.TransformedShapeCollector;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const allocator = std.testing.allocator;

/// JPH_DOUBLE_PRECISION
const double_precision = RVec3 == DVec3;

test "TestTransformedShape" {
    const half_extents = Vec3.init(0.5, 1.0, 1.5);
    const scale = Vec3.init(-2, 3, 4);
    const rtshape_translation = Vec3.init(1, 3, 5);
    const rtshape_rotation = Quat.rotation(Vec3.init(1, 2, 3).normalized(), 0.25 * math.pi);
    const translation = RVec3.init(13, 9, 7);
    const rotation = Quat.rotation(Vec3.axisY(), 0.5 * math.pi); // A rotation of 90 degrees in order to not shear the shape

    const material = try PhysicsMaterialSimple.create(allocator, "Test Material", Color.red);

    // Create a scaled, rotated and translated box
    var box_settings = BoxShapeSettings.init(allocator, half_extents, .{ .convex_radius = 0.0, .material = material.material() });
    box_settings.asShapeSettings().setEmbedded();
    defer box_settings.deinit();
    var scale_settings = ScaledShapeSettings.init(allocator, box_settings.asShapeSettings(), scale);
    scale_settings.asShapeSettings().setEmbedded();
    defer scale_settings.deinit();
    var rtshape_settings = RotatedTranslatedShapeSettings.init(allocator, rtshape_translation, rtshape_rotation, scale_settings.asShapeSettings());
    rtshape_settings.asShapeSettings().setEmbedded();
    defer rtshape_settings.deinit();

    // Create a body with this shape
    // Not ported: PhysicsTestContext::CreateBody (Phase 5). Body::GetTransformedShape is the shape at the center of mass
    // position of the body (Body::SetPositionAndRotationInternal: inPosition + inRotation * mShape->GetCenterOfMass())
    var rtshape = try rtshape_settings.asShapeSettings().createShape(allocator);
    defer rtshape.deinit();
    const body_id = BodyID.init(1); // Stands in for body.GetID()
    var body_transformed_shape = TransformedShape.init(translation.addVec3(rotation.mulVec3(rtshape.getPtr().?.getCenterOfMass())), rotation, rtshape.getPtr().?, body_id, .{});
    defer body_transformed_shape.deinit();

    // Collect the leaf shape transform
    var collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer collector.deinit();
    // Not ported: c.GetSystem()->GetNarrowPhaseQuery().CollectTransformedShapes(AABox::sBiggest(), collector) (Phase 5),
    // NarrowPhaseQuery calls this for each body with OnBody / OnBodyEnd around it
    body_transformed_shape.collectTransformedShapes(AABox.biggest(), &collector.base, .{});
    try collector.checkError();

    // Check that there is exactly 1 shape
    try fw.expect(collector.hits.items.len == 1);
    const ts = &collector.hits.items[0];

    // Check that we got the leaf shape: box
    var box_result = try box_settings.asShapeSettings().createShape(allocator);
    defer box_result.deinit();
    try fw.expect(ts.shape.get() == box_result.getPtr().?);

    // Check that its transform matches the transform that we provided
    const calc_transform = if (double_precision)
        DMat44.rotationTranslation(rotation, translation).mulMat44(Mat44.rotationTranslation(rtshape_rotation, rtshape_translation)).mul(DMat44.scaleVec3(scale))
    else
        Mat44.rotationTranslation(rotation, translation).mul(Mat44.rotationTranslation(rtshape_rotation, rtshape_translation)).mul(Mat44.scaleVec3(scale));
    try fw.checkApproxEqual(calc_transform, ts.getWorldTransform(), .{});

    // Check that all corner points are in the bounding box
    const aabox = ts.getWorldSpaceBounds();
    const corners = [_]Vec3{
        Vec3.init(-0.99, -0.99, -0.99).mul(half_extents),
        Vec3.init(0.99, -0.99, -0.99).mul(half_extents),
        Vec3.init(-0.99, 0.99, -0.99).mul(half_extents),
        Vec3.init(0.99, 0.99, -0.99).mul(half_extents),
        Vec3.init(-0.99, -0.99, 0.99).mul(half_extents),
        Vec3.init(0.99, -0.99, 0.99).mul(half_extents),
        Vec3.init(-0.99, 0.99, 0.99).mul(half_extents),
        Vec3.init(0.99, 0.99, 0.99).mul(half_extents),
    };
    for (corners) |corner| {
        try fw.expect(aabox.containsRVec3(calc_transform.mulVec3(corner)));
        try fw.expect(!aabox.containsRVec3(calc_transform.mulVec3(corner.mulScalar(2)))); // Check that points twice as far away are not in the box
    }

    // Now pick a point on the box near the edge in local space, determine a raycast that hits it
    const point_on_box = Vec3.init(half_extents.getX() - 0.01, half_extents.getY() - 0.01, half_extents.getZ());
    const normal_on_box = Vec3.init(0, 0, 1);
    const ray_direction_local = Vec3.init(1, 1, -1);

    // Transform to world space and do the raycast
    const ray_start_local = point_on_box.sub(ray_direction_local);
    const ray_end_local = point_on_box.add(ray_direction_local);
    const ray_start_world = calc_transform.mulVec3(ray_start_local);
    const ray_end_world = calc_transform.mulVec3(ray_end_local);
    const ray_direction_world = ray_end_world.sub(ray_start_world).toVec3();
    const ray_in_world = RRayCast.init(ray_start_world, ray_direction_world);
    var hit: RayCastResult = .{};
    _ = ts.castRay(ray_in_world, &hit);

    // Check the hit result
    try fw.checkApproxEqual(hit.fraction, 0.5, .{});
    try fw.expect(hit.body_id.eql(body_id));
    try fw.expect(ts.getMaterial(hit.sub_shape_id2) == material.material());
    const world_space_normal = ts.getWorldSpaceSurfaceNormal(hit.sub_shape_id2, ray_in_world.getPointOnRay(hit.fraction));
    const expected_normal = calc_transform.getDirectionPreservingMatrix().mulVec3(normal_on_box).normalized();
    try fw.checkApproxEqual(world_space_normal, expected_normal, .{});

    // Reset the transform to identity and check that it worked
    ts.setWorldTransformRMat44(RMat44.identity());
    try fw.checkApproxEqual(ts.getWorldTransform(), RMat44.identity(), .{});

    // Set the calculated world transform again to see if getting/setting a transform is symmetric
    ts.setWorldTransformRMat44(calc_transform);
    try fw.checkApproxEqual(calc_transform, ts.getWorldTransform(), .{});
}
