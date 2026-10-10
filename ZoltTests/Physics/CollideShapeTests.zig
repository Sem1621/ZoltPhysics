//! Port of: UnitTests/Physics/CollideShapeTests.cpp
//! Status: partial
//! Missing: TestCollideShapeSphere, TestCollideShapeSphereVsBox (entirely NarrowPhaseQuery::CollideShape against bodies in
//!   a PhysicsTestContext, need PhysicsTestContext / NarrowPhaseQuery, Phase 5; sCompareCollideShapeResultSphere is
//!   only used by TestCollideShapeSphere)
//!
//! One check of TestTriangleVsBoxLargeSeparationDistance is not portable (not a Phase 5 item): in the no hit branch Jolt
//! reads the uninitialized mHit of a collector that had no hit (see the comment there).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const BoxShape = zolt.BoxShape;
const BoxShapeSettings = zolt.BoxShapeSettings;
const CapsuleShape = zolt.CapsuleShape;
const CapsuleShapeSettings = zolt.CapsuleShapeSettings;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const CollideConvexVsTriangles = zolt.CollideConvexVsTriangles;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollisionDispatch = zolt.CollisionDispatch;
const ConvexHullShapeSettings = zolt.ConvexHullShapeSettings;
const ConvexShape = zolt.ConvexShape;
const CylinderShape = zolt.CylinderShape;
const EPAPenetrationDepth = zolt.EPAPenetrationDepth;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const SphereShape = zolt.SphereShape;
const SubShapeID = zolt.SubShapeID;
const TriangleConvexSupport = zolt.TriangleConvexSupport;
const TriangleShape = zolt.TriangleShape;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;
const physics_settings = zolt.physics_settings;

const allocator = std.testing.allocator;

/// `settings.Create().Get()`: the shape of a newly created result (the caller releases the reference)
fn createShape(settings: *ShapeSettings) !RefConst(Shape) {
    var result = try settings.createShape(allocator);
    defer result.deinit();
    return RefConst(Shape).init(result.getPtr());
}

// Not ported: sCompareCollideShapeResultSphere, TestCollideShapeSphere and TestCollideShapeSphereVsBox (they collide
// with bodies through NarrowPhaseQuery::CollideShape, need PhysicsTestContext / NarrowPhaseQuery, Phase 5)

// Test colliding a very long capsule vs a box that is intersecting with the line segment inside the capsule
// This particular config reported the wrong penetration due to accuracy problems before
test "TestCollideShapeLongCapsuleVsEmbeddedBox" {
    // Create box
    const box_min = Vec3.init(-1.0, -2.0, 0.5);
    const box_max = Vec3.init(2.0, -0.5, 3.0);
    const box_inner = try BoxShapeSettings.create(allocator, box_max.sub(box_min).mulScalar(0.5), .{});
    var box_settings = Ref(ShapeSettings).init((try RotatedTranslatedShapeSettings.create(allocator, box_min.add(box_max).mulScalar(0.5), Quat.identity(), box_inner.asShapeSettings())).asShapeSettings());
    defer box_settings.deinit();
    var box_shape_ref = try createShape(box_settings.get().?);
    defer box_shape_ref.deinit();
    const box_shape = box_shape_ref.get().?;
    const box_transform = Mat44.init(Vec4.init(0.516170502, -0.803887904, -0.295520246, 0.0), Vec4.init(0.815010250, 0.354940295, 0.458012700, 0.0), Vec4.init(-0.263298869, -0.477264702, 0.838386655, 0.0), Vec4.init(-10.2214508, -18.6808319, 40.7468987, 1.0));

    // Create capsule
    const capsule_half_height: f32 = 75.0;
    const capsule_radius: f32 = 1.5;
    const capsule_inner = try CapsuleShapeSettings.create(allocator, capsule_half_height, capsule_radius, .{});
    var capsule_settings = Ref(ShapeSettings).init((try RotatedTranslatedShapeSettings.create(allocator, Vec3.init(0, 0, 75), Quat.init(0.499999970, -0.499999970, -0.499999970, 0.499999970), capsule_inner.asShapeSettings())).asShapeSettings());
    defer capsule_settings.deinit();
    var capsule_shape_ref = try createShape(capsule_settings.get().?);
    defer capsule_shape_ref.deinit();
    const capsule_shape = capsule_shape_ref.get().?;
    const capsule_transform = Mat44.translation(Vec3.init(-9.68538570, -18.0328083, 41.3212280));

    // Collision settings
    var settings: CollideShapeSettings = .{};
    settings.active_edge_mode = .collide_with_all;
    settings.back_face_mode = .collide_with_back_faces;
    settings.collect_faces_mode = .no_faces;

    // Collide the two shapes
    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collector.deinit();
    CollisionDispatch.collideShapeVsShape(capsule_shape, box_shape, Vec3.one(), Vec3.one(), capsule_transform, box_transform, .{}, .{}, &settings, &collector.base, &.{});
    try collector.checkError();

    // Check that there was a hit
    try fw.expect(collector.hits.items.len == 1);
    const result = collector.hits.items[0];

    // Now move the box 1% further than the returned penetration depth and check that it is no longer in collision
    const distance_to_move_box = result.penetration_axis.normalized().mulScalar(result.penetration_depth);
    collector.reset();
    try fw.expect(!collector.hadHit());
    CollisionDispatch.collideShapeVsShape(capsule_shape, box_shape, Vec3.one(), Vec3.one(), capsule_transform, Mat44.translation(distance_to_move_box.mulScalar(1.01)).mul(box_transform), .{}, .{}, &settings, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(!collector.hadHit());

    // Now check that moving 1% less than the penetration distance makes the shapes still overlap
    CollisionDispatch.collideShapeVsShape(capsule_shape, box_shape, Vec3.one(), Vec3.one(), capsule_transform, Mat44.translation(distance_to_move_box.mulScalar(0.99)).mul(box_transform), .{}, .{}, &settings, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 1);
}

// Another test case found in practice of a very large oriented box (convex hull) vs a small triangle outside the hull. This should not report a collision
test "TestCollideShapeSmallTriangleVsLargeBox" {
    // Triangle vertices
    const v0 = Vec3.init(-81.5637589, -126.987244, -146.771729);
    const v1 = Vec3.init(-81.8749924, -127.270691, -146.544403);
    const v2 = Vec3.init(-81.6972275, -127.383545, -146.773254);

    // Oriented box vertices
    const obox_points = [_]Vec3{
        Vec3.init(125.932892, -374.712250, 364.192169),
        Vec3.init(319.492218, -73.2614441, 475.009613),
        Vec3.init(-122.277550, -152.200287, 192.441437),
        Vec3.init(71.2817841, 149.250519, 303.258881),
        Vec3.init(-77.8921967, -359.410797, 678.579712),
        Vec3.init(115.667137, -57.9600067, 789.397095),
        Vec3.init(-326.102631, -136.898834, 506.828949),
        Vec3.init(-132.543304, 164.551971, 617.646362),
    };
    var hull_settings = try ConvexHullShapeSettings.init(allocator, &obox_points, .{ .max_convex_radius = 0.0 });
    defer hull_settings.deinit();
    var convex_hull_ref = try createShape(hull_settings.asShapeSettings()); // StaticCast<ConvexShape>: a RefConst(Shape), cast on use
    defer convex_hull_ref.deinit();
    const convex_hull = convex_hull_ref.get().?.cast(ConvexShape);

    // Create triangle support function
    const triangle = TriangleConvexSupport.init(v0, v1, v2);

    // Create the convex hull support function
    var buffer: ConvexShape.SupportBuffer = .{};
    const support = convex_hull.getSupportFunction(.include_convex_radius, &buffer, Vec3.one());

    // Triangle is close enough to make GJK report indeterminate
    var penetration_axis = Vec3.axisX();
    var point1: Vec3 = undefined;
    var point2: Vec3 = undefined;
    var pen_depth: EPAPenetrationDepth = .{};
    const status = pen_depth.getPenetrationDepthStepGJK(support, support.getConvexRadius(), &triangle, 0.0, physics_settings.default_collision_tolerance, &penetration_axis, &point1, &point2);
    try fw.expect(status == .indeterminate);

    // But there should not be an actual collision
    try fw.expect(!pen_depth.getPenetrationDepthStepEPA(support, &triangle, physics_settings.default_penetration_tolerance, &penetration_axis, &point1, &point2));
}

// A test case of a triangle that's nearly parallel to a capsule and penetrating it. This one was causing numerical issues.
test "TestCollideParallelTriangleVsCapsule" {
    const v1 = Vec3.init(-0.479988575, -1.36185002, 0.269966960);
    const v2 = Vec3.init(-0.104996204, 0.388152480, 0.269967079);
    const v3 = Vec3.init(-0.104996204, -1.36185002, 0.269966960);
    var triangle = TriangleShape.init(allocator, v1, v2, v3, .{});
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();

    const capsule_radius: f32 = 0.37;
    const capsule_half_height: f32 = 0.5;
    var capsule = CapsuleShape.init(allocator, capsule_half_height, capsule_radius, .{});
    capsule.asShape().setEmbedded();
    defer capsule.asShapeMut().deinit();

    const settings: CollideShapeSettings = .{};
    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collector.deinit();
    CollisionDispatch.collideShapeVsShape(triangle.asShape(), capsule.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.identity(), .{}, .{}, &settings, &collector.base, &.{});
    try collector.checkError();

    // The capsule's center is closest to the triangle's edge v2 v3
    var capsule_center_to_triangle_v2_v3 = v3;
    capsule_center_to_triangle_v2_v3.setY(0); // The penetration axis will be in x, z only because the triangle is parallel to the capsule axis
    const capsule_center_to_triangle_v2_v3_len = capsule_center_to_triangle_v2_v3.length();
    const expected_penetration_axis = capsule_center_to_triangle_v2_v3.negate().divScalar(capsule_center_to_triangle_v2_v3_len);
    const expected_penetration_depth = capsule_radius - capsule_center_to_triangle_v2_v3_len;

    try fw.expect(collector.hits.items.len == 1);
    const hit = &collector.hits.items[0];
    const actual_penetration_axis = hit.penetration_axis.normalized();
    const actual_penetration_depth = hit.penetration_depth;

    try fw.checkApproxEqual(actual_penetration_axis, expected_penetration_axis, .{});
    try fw.checkApproxEqual(actual_penetration_depth, expected_penetration_depth, .{});
}

// A test case of a triangle that's nearly parallel to a capsule and penetrating it. This one was causing numerical issues.
test "TestCollideParallelTriangleVsCapsule2" {
    const v1 = Vec3.init(-0.0904417038, -4.72410202, 0.307858467);
    const v2 = Vec3.init(-0.0904417038, 5.27589798, 0.307857513);
    const v3 = Vec3.init(9.90955830, 5.27589798, 0.307864189);
    var triangle = TriangleShape.init(allocator, v1, v2, v3, .{});
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();

    const capsule_radius: f32 = 0.42;
    const capsule_half_height: f32 = 0.675;
    var capsule = CapsuleShape.init(allocator, capsule_half_height, capsule_radius, .{});
    capsule.asShape().setEmbedded();
    defer capsule.asShapeMut().deinit();

    const settings: CollideShapeSettings = .{};
    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collector.deinit();
    CollisionDispatch.collideShapeVsShape(triangle.asShape(), capsule.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.identity(), .{}, .{}, &settings, &collector.base, &.{});
    try collector.checkError();

    // The capsule intersects with the triangle and the closest point is in the interior of the triangle
    const expected_penetration_axis = Vec3.init(0, 0, -1); // Triangle is in the XY plane so the normal is Z
    const expected_penetration_depth = capsule_radius - v1.getZ();

    try fw.expect(collector.hits.items.len == 1);
    const hit = &collector.hits.items[0];
    const actual_penetration_axis = hit.penetration_axis.normalized();
    const actual_penetration_depth = hit.penetration_depth;

    try fw.checkApproxEqual(actual_penetration_axis, expected_penetration_axis, .{});
    try fw.checkApproxEqual(actual_penetration_depth, expected_penetration_depth, .{});
}

// A test case of a triangle that's nearly parallel to a capsule and almost penetrating it. This one was causing numerical issues.
test "TestCollideParallelTriangleVsCapsule3" {
    const v1 = Vec3.init(-0.474807739, 17.2921791, 0.212532043);
    const v2 = Vec3.init(-0.474807739, -2.70782185, 0.212535858);
    const v3 = Vec3.init(-0.857490540, -2.70782185, -0.711341858);
    var triangle = TriangleShape.init(allocator, v1, v2, v3, .{});
    triangle.asShape().setEmbedded();
    defer triangle.asShapeMut().deinit();

    const capsule_radius: f32 = 0.5;
    const capsule_half_height: f32 = 0.649999976;
    var capsule = CapsuleShape.init(allocator, capsule_half_height, capsule_radius, .{});
    capsule.asShape().setEmbedded();
    defer capsule.asShapeMut().deinit();

    var settings: CollideShapeSettings = .{};
    settings.max_separation_distance = 0.120000005;
    var collector = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer collector.deinit();
    CollisionDispatch.collideShapeVsShape(capsule.asShape(), triangle.asShape(), Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.identity(), .{}, .{}, &settings, &collector.base, &.{});

    try fw.expect(collector.hadHit());
    const expected_normal = v2.sub(v1).cross(v3.sub(v1)).normalized();
    const actual_normal = collector.hit.penetration_axis.normalized().negate();
    try fw.checkApproxEqual(actual_normal, expected_normal, .{ .tolerance = 1.0e-6 });
    const expected_penetration_depth = capsule.getRadius() + v1.dot(expected_normal);
    try fw.checkApproxEqual(collector.hit.penetration_depth, expected_penetration_depth, .{ .tolerance = 1.0e-6 });
}

// A test case of a triangle that's nearly parallel to a cylinder and is just penetrating it. This one was causing numerical issues. See issue #1008.
test "TestCollideParallelTriangleVsCylinder" {
    var cylinder = CylinderShape.init(allocator, 0.85, 0.25, .{ .convex_radius = 0.02 });
    cylinder.asShape().setEmbedded();
    defer cylinder.asShapeMut().deinit();

    const cylinder_transform = Mat44.translation(Vec3.init(-42.8155518, -4.32299995, 12.1734285));

    var settings: CollideShapeSettings = .{};
    settings.max_separation_distance = 0.001;
    var collector = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer collector.deinit();
    var c = CollideConvexVsTriangles.init(&cylinder.base, Vec3.one(), Vec3.one(), cylinder_transform, Mat44.identity(), .empty, &settings, &collector.base);

    const v0 = Vec3.init(-42.7954292, -0.647318780, 12.4227943);
    const v1 = Vec3.init(-29.9111290, -0.647318780, 12.4227943);
    const v2 = Vec3.init(-42.7954292, -4.86970234, 12.4227943);
    c.collide(v0, v1, v2, 0, .empty);

    // Check there was a hit
    try fw.expect(collector.hadHit());
    try fw.expect(collector.hit.penetration_depth < 1.0e-4);
    try fw.expect(collector.hit.penetration_axis.normalized().isClose(Vec3.axisZ(), .{}));
}

// A test case of a box and a convex hull that are nearly touching and that should return a contact with correct normal because the collision settings specify a max separation distance. This was producing the wrong normal.
test "BoxVsConvexHullNoConvexRadius" {
    const separation_distance: f32 = 0.001;
    const box_separation_from_hull: f32 = 0.5 * separation_distance;
    const hull_height: f32 = 0.25;

    // Box with no convex radius
    var box_settings = Ref(ShapeSettings).init((try BoxShapeSettings.create(allocator, Vec3.init(0.25, 0.75, 0.375), .{ .convex_radius = 0.0 })).asShapeSettings());
    defer box_settings.deinit();
    var box_shape_ref = try createShape(box_settings.get().?);
    defer box_shape_ref.deinit();
    const box_shape = box_shape_ref.get().?;

    // Convex hull (also a box) with no convex radius
    const hull_points = [_]Vec3{
        Vec3.init(-2.5, -hull_height, -1.5),
        Vec3.init(-2.5, hull_height, -1.5),
        Vec3.init(2.5, -hull_height, -1.5),
        Vec3.init(-2.5, -hull_height, 1.5),
        Vec3.init(-2.5, hull_height, 1.5),
        Vec3.init(2.5, hull_height, -1.5),
        Vec3.init(2.5, -hull_height, 1.5),
        Vec3.init(2.5, hull_height, 1.5),
    };
    var hull_settings = Ref(ShapeSettings).init((try ConvexHullShapeSettings.create(allocator, &hull_points, .{ .max_convex_radius = 0.0 })).asShapeSettings());
    defer hull_settings.deinit();
    var hull_shape_ref = try createShape(hull_settings.get().?);
    defer hull_shape_ref.deinit();
    const hull_shape = hull_shape_ref.get().?;

    var angle: f32 = 0.0;
    for (0..481) |_| {
        // Slowly rotate both box and convex hull
        angle += math.degreesToRadians(45.0) / 60.0;
        const hull_transform = Mat44.rotationY(angle);
        const box_local_translation = Mat44.translation(Vec3.init(0.1, 1.0 + box_separation_from_hull, -0.5));
        const box_local_rotation = Mat44.rotationY(math.degreesToRadians(-45.0));
        const box_local_transform = box_local_translation.mul(box_local_rotation);
        const box_transform = hull_transform.mul(box_local_transform);

        var settings: CollideShapeSettings = .{};
        settings.max_separation_distance = separation_distance;
        var collector = ClosestHitCollisionCollector(CollideShapeCollector).init();
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(box_shape, hull_shape, Vec3.one(), Vec3.one(), box_transform, hull_transform, .{}, .{}, &settings, &collector.base, &.{});

        // Check that there was a hit and that the contact normal is correct
        try fw.expect(collector.hadHit());
        const hit = &collector.hit;
        try fw.checkApproxEqual(hit.contact_point_on1.getY(), hull_height + box_separation_from_hull, .{ .tolerance = 1.0e-3 });
        try fw.checkApproxEqual(hit.contact_point_on2.getY(), hull_height, .{});
        try fw.checkApproxEqual(hit.penetration_axis.normalizedOr(Vec3.zero()), Vec3.axisY().negate(), .{ .tolerance = 1.0e-3 });
    }

    try fw.expect(angle >= 2.0 * math.pi);
}

// This test checks extreme values of the max separation distance and how it affects ConvexShape::sCollideConvexVsConvex
// See: https://github.com/jrouwe/JoltPhysics/discussions/1379
test "TestBoxVsSphereLargeSeparationDistance" {
    const radius: f32 = 1.0;
    const half_extent: f32 = 10.0;
    var sphere_shape = RefConst(Shape).init((try SphereShape.create(allocator, radius, .{})).asShape());
    defer sphere_shape.deinit();
    var box_shape = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(half_extent), .{})).asShape());
    defer box_shape.deinit();
    const distances = [_]f32{ 0.0, 0.5, 1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0, 5000.0, 10000.0 };
    for (distances) |x| {
        for (distances) |max_separation| {
            var collide_settings: CollideShapeSettings = .{};
            collide_settings.max_separation_distance = max_separation;
            var collector = ClosestHitCollisionCollector(CollideShapeCollector).init();
            defer collector.deinit();
            CollisionDispatch.collideShapeVsShape(box_shape.get().?, sphere_shape.get().?, Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.translation(Vec3.init(x, 0, 0)), .{}, .{}, &collide_settings, &collector.base, &.{});

            const expected_penetration = half_extent - (x - radius);
            if (collector.hadHit())
                try fw.checkApproxEqual(expected_penetration, collector.hit.penetration_depth, .{ .tolerance = 1.0e-3 })
            else
                try fw.expect(expected_penetration < -max_separation);
        }
    }
}

// This test case checks extreme values of the max separation distance and how it affects CollideConvexVsTriangles::Collide
// See: https://github.com/jrouwe/JoltPhysics/discussions/1379
test "TestTriangleVsBoxLargeSeparationDistance" {
    const triangle_x: f32 = -0.1;
    const half_extent: f32 = 10.0;
    var triangle_shape = RefConst(Shape).init((try TriangleShape.create(allocator, Vec3.init(triangle_x, -10, 10), Vec3.init(triangle_x, -10, -10), Vec3.init(triangle_x, 10, 0), .{})).asShape());
    defer triangle_shape.deinit();
    var box_shape = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.replicate(half_extent), .{})).asShape());
    defer box_shape.deinit();
    const distances = [_]f32{ 0.0, 0.5, 1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0, 5000.0, 10000.0 };
    for (distances) |x| {
        for (distances) |max_separation| {
            var collide_settings: CollideShapeSettings = .{};
            collide_settings.max_separation_distance = max_separation;
            var collector = ClosestHitCollisionCollector(CollideShapeCollector).init();
            defer collector.deinit();
            CollisionDispatch.collideShapeVsShape(triangle_shape.get().?, box_shape.get().?, Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.translation(Vec3.init(x, 0, 0)), .{}, .{}, &collide_settings, &collector.base, &.{});

            const expected_penetration = triangle_x - (x - half_extent);
            if (collector.hadHit()) {
                try fw.checkApproxEqual(expected_penetration, collector.hit.penetration_depth, .{ .tolerance = 1.0e-3 });
            } else {
                try fw.expect(expected_penetration < -max_separation);
                // Not ported: CHECK_APPROX_EQUAL(collector.mHit.mPenetrationAxis.NormalizedOr(Vec3::sZero()), Vec3::sAxisX(), 1.0e-5f).
                // The collector had no hit, so Jolt reads the default constructed (uninitialized) mHit: undefined behavior
                // that only passes because each iteration's collector reuses the stack slot of the previous one (whose
                // hit has axis X). Zolt's ClosestHitCollisionCollector has no hit to read either.
            }
        }
    }
}

test "TestCollideTriangleVsTriangle" {
    const penetration: f32 = 0.01;

    // A triangle centered around the origin in the XZ plane
    var t1 = RefConst(Shape).init((try TriangleShape.create(allocator, Vec3.init(-1, 0, 1), Vec3.init(1, 0, 1), Vec3.init(0, 0, -1), .{})).asShape());
    defer t1.deinit();

    // A triangle in the XY plane with its tip just pointing in the origin
    var t2 = RefConst(Shape).init((try TriangleShape.create(allocator, Vec3.init(-1, 1, 0), Vec3.init(1, 1, 0), Vec3.init(0, -penetration, 0), .{})).asShape());
    defer t2.deinit();

    const collide_settings: CollideShapeSettings = .{};
    var collector = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer collector.deinit();
    CollisionDispatch.collideShapeVsShape(t1.get().?, t2.get().?, Vec3.one(), Vec3.one(), Mat44.identity(), Mat44.identity(), .{}, .{}, &collide_settings, &collector.base, &.{});

    try fw.expect(collector.hadHit());
    try fw.checkApproxEqual(collector.hit.contact_point_on1, Vec3.zero(), .{});
    try fw.checkApproxEqual(collector.hit.contact_point_on2, Vec3.init(0, -penetration, 0), .{});
    try fw.checkApproxEqual(collector.hit.penetration_depth, penetration, .{});
    try fw.checkApproxEqual(collector.hit.penetration_axis.normalized(), Vec3.init(0, 1, 0), .{});
}
