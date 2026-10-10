//! Port of: UnitTests/Physics/CastShapeTests.cpp
//! Status: partial
//! Missing: TestCastShapeSphereVsBox, TestCastShapeCylinderVsCapsule, TestCastShapePenetrationDepthOrdering,
//!   TestDeepestPenetrationAtFraction0, TestClosestHitPerBodyCollisionCollector, TestCastShapeExtraConvexRadius,
//!   TestCastShapeInitiallyIntersecting (entirely NarrowPhaseQuery::CastShape against bodies in a PhysicsTestContext,
//!   need PhysicsTestContext / NarrowPhaseQuery, Phase 5)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const CastShapeCollector = zolt.CastShapeCollector;
const CastSphereVsTriangles = zolt.CastSphereVsTriangles;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const CollisionDispatch = zolt.CollisionDispatch;
const ConvexHullShapeSettings = zolt.ConvexHullShapeSettings;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const MeshShapeSettings = zolt.MeshShapeSettings;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeSettings = zolt.ShapeSettings;
const SphereShape = zolt.SphereShape;
const SphereShapeSettings = zolt.SphereShapeSettings;
const Triangle = zolt.Triangle;
const TriangleShapeSettings = zolt.TriangleShapeSettings;
const Vec3 = zolt.Vec3;

const allocator = std.testing.allocator;

/// `settings.Create().Get()`: the shape of a newly created result (the caller releases the reference)
fn createShape(settings: *ShapeSettings) !RefConst(Shape) {
    var result = try settings.createShape(allocator);
    defer result.deinit();
    return RefConst(Shape).init(result.getPtr());
}

/// Helper function that tests a sphere against a triangle
fn testCastSphereVertexOrEdge(sphere: *const Shape, position: Vec3, direction: Vec3, triangle: *const Shape) !void {
    const shape_cast = ShapeCast.init(sphere, Vec3.one(), Mat44.translation(position.sub(direction)), direction);
    var cast_settings: ShapeCastSettings = .{};
    cast_settings.back_face_mode_triangles = .collide_with_back_faces;
    cast_settings.back_face_mode_convex = .collide_with_back_faces;
    var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer collector.deinit();
    CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 1);
    const result = &collector.hits.items[collector.hits.items.len - 1];
    try fw.checkApproxEqual(result.fraction, 1.0 - 0.2 / direction.length(), .{ .tolerance = 1.0e-4 });
    try fw.checkApproxEqual(result.base.penetration_axis.normalized(), direction.normalized(), .{ .tolerance = 1.0e-3 });
    try fw.checkApproxEqual(result.base.penetration_depth, 0.0, .{ .tolerance = 1.0e-3 });
    try fw.checkApproxEqual(result.base.contact_point_on1, position, .{ .tolerance = 1.0e-3 });
    try fw.checkApproxEqual(result.base.contact_point_on2, position, .{ .tolerance = 1.0e-3 });
}

/// Helper function that tests a sphere against a triangle centered on the origin with normal Z
fn testCastSphereTriangle(triangle: *const Shape) !void {
    // Create sphere
    var sphere_settings = SphereShapeSettings.init(allocator, 0.2, .{});
    defer sphere_settings.deinit();
    var sphere_ref = try createShape(sphere_settings.asShapeSettings());
    defer sphere_ref.deinit();
    const sphere = sphere_ref.get().?;

    {
        // Hit front face
        const shape_cast = ShapeCast.init(sphere, Vec3.one(), Mat44.translation(Vec3.init(0, 0, 15)), Vec3.init(0, 0, -30));
        var cast_settings: ShapeCastSettings = .{};
        cast_settings.back_face_mode_triangles = .ignore_back_faces;
        cast_settings.back_face_mode_convex = .ignore_back_faces;
        cast_settings.return_deepest_point = false;
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 1);
        const result = &collector.hits.items[collector.hits.items.len - 1];
        try fw.checkApproxEqual(result.fraction, (15.0 - 0.2) / 30.0, .{ .tolerance = 1.0e-4 });
        try fw.checkApproxEqual(result.base.penetration_axis.normalized(), Vec3.init(0, 0, -1), .{ .tolerance = 1.0e-3 });
        try fw.expect(result.base.penetration_depth == 0.0);
        try fw.checkApproxEqual(result.base.contact_point_on1, Vec3.zero(), .{ .tolerance = 1.0e-3 });
        try fw.checkApproxEqual(result.base.contact_point_on2, Vec3.zero(), .{ .tolerance = 1.0e-3 });
        try fw.expect(!result.is_back_face_hit);
    }

    {
        // Hit back face -> ignored
        const shape_cast = ShapeCast.init(sphere, Vec3.one(), Mat44.translation(Vec3.init(0, 0, -15)), Vec3.init(0, 0, 30));
        var cast_settings: ShapeCastSettings = .{};
        cast_settings.back_face_mode_triangles = .ignore_back_faces;
        cast_settings.back_face_mode_convex = .ignore_back_faces;
        cast_settings.return_deepest_point = false;
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 0);

        // Hit back face -> collision
        cast_settings.back_face_mode_triangles = .collide_with_back_faces;
        cast_settings.back_face_mode_convex = .collide_with_back_faces;
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 1);
        const result = &collector.hits.items[collector.hits.items.len - 1];
        try fw.checkApproxEqual(result.fraction, (15.0 - 0.2) / 30.0, .{ .tolerance = 1.0e-4 });
        try fw.checkApproxEqual(result.base.penetration_axis.normalized(), Vec3.init(0, 0, 1), .{ .tolerance = 1.0e-3 });
        try fw.expect(result.base.penetration_depth == 0.0);
        try fw.checkApproxEqual(result.base.contact_point_on1, Vec3.zero(), .{ .tolerance = 1.0e-3 });
        try fw.checkApproxEqual(result.base.contact_point_on2, Vec3.zero(), .{ .tolerance = 1.0e-3 });
        try fw.expect(result.is_back_face_hit);
    }

    {
        // Hit back face while starting in collision -> ignored
        const shape_cast = ShapeCast.init(sphere, Vec3.one(), Mat44.translation(Vec3.init(0, 0, -0.1)), Vec3.init(0, 0, 15));
        var cast_settings: ShapeCastSettings = .{};
        cast_settings.back_face_mode_triangles = .ignore_back_faces;
        cast_settings.back_face_mode_convex = .ignore_back_faces;
        cast_settings.return_deepest_point = true;
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 0);

        // Hit back face while starting in collision -> collision
        cast_settings.back_face_mode_triangles = .collide_with_back_faces;
        cast_settings.back_face_mode_convex = .collide_with_back_faces;
        CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &cast_settings, triangle, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &collector.base);
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 1);
        const result = &collector.hits.items[collector.hits.items.len - 1];
        try fw.checkApproxEqual(result.fraction, 0.0, .{});
        try fw.checkApproxEqual(result.base.penetration_axis.normalized(), Vec3.init(0, 0, 1), .{ .tolerance = 1.0e-3 });
        try fw.checkApproxEqual(result.base.penetration_depth, 0.1, .{ .tolerance = 1.0e-3 });
        try fw.checkApproxEqual(result.base.contact_point_on1, Vec3.init(0, 0, 0.1), .{ .tolerance = 1.0e-3 });
        try fw.checkApproxEqual(result.base.contact_point_on2, Vec3.zero(), .{ .tolerance = 1.0e-3 });
        try fw.expect(result.is_back_face_hit);
    }

    // Hit vertex 1, 2 and 3
    try testCastSphereVertexOrEdge(sphere, Vec3.init(50, 25, 0), Vec3.init(-10, -10, 0), triangle);
    try testCastSphereVertexOrEdge(sphere, Vec3.init(-50, 25, 0), Vec3.init(10, -10, 0), triangle);
    try testCastSphereVertexOrEdge(sphere, Vec3.init(0, -25, 0), Vec3.init(0, 10, 0), triangle);

    // Hit edge 1, 2 and 3
    try testCastSphereVertexOrEdge(sphere, Vec3.init(0, 25, 0), Vec3.init(0, -10, 0), triangle); // Edge: Vec3(50, 25, 0), Vec3(-50, 25, 0)
    try testCastSphereVertexOrEdge(sphere, Vec3.init(-25, 0, 0), Vec3.init(10, 10, 0), triangle); // Edge: Vec3(-50, 25, 0), Vec3(0,-25, 0)
    try testCastSphereVertexOrEdge(sphere, Vec3.init(25, 0, 0), Vec3.init(-10, 10, 0), triangle); // Edge: Float3(0,-25, 0), Float3(50, 25, 0)
}

test "TestCastSphereTriangle" {
    // Create triangle
    var triangle_settings = TriangleShapeSettings.init(allocator, Vec3.init(50, 25, 0), Vec3.init(-50, 25, 0), Vec3.init(0, -25, 0), .{});
    defer triangle_settings.deinit();
    var triangle = try createShape(triangle_settings.asShapeSettings());
    defer triangle.deinit();
    try testCastSphereTriangle(triangle.get().?);

    // Create a triangle mesh shape
    var mesh_settings = try MeshShapeSettings.init(allocator, &.{Triangle.fromFloat3(Float3.init(50, 25, 0), Float3.init(-50, 25, 0), Float3.init(0, -25, 0), .{})}, .{});
    defer mesh_settings.deinit();
    var triangle_mesh = try createShape(mesh_settings.asShapeSettings());
    defer triangle_mesh.deinit();
    try testCastSphereTriangle(triangle_mesh.get().?);
}

// Not ported: TestCastShapeSphereVsBox, TestCastShapeCylinderVsCapsule, TestCastShapePenetrationDepthOrdering and
// TestDeepestPenetrationAtFraction0 (they cast against bodies through NarrowPhaseQuery::CastShape, need
// PhysicsTestContext / NarrowPhaseQuery, Phase 5)

// Test a problem case where a sphere cast would incorrectly hit a degenerate triangle (see: https://github.com/jrouwe/JoltPhysics/issues/886)
test "TestCastSphereVsDegenerateTriangle" {
    var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer collector.deinit();
    var sphere = SphereShape.init(allocator, 0.2, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();
    const cast = ShapeCast.init(sphere.asShape(), Vec3.one(), Mat44.translation(Vec3.init(14.8314590, 8.19055080, -4.30825043)), Vec3.init(-0.0988006592, 5.96046448e-08, 0.000732421875));
    const settings: ShapeCastSettings = .{};
    var caster = CastSphereVsTriangles.init(&cast, &settings, Vec3.one(), Mat44.identity(), .{}, &collector.base);
    caster.cast(Vec3.init(14.5536213, 10.5973721, -0.00600051880), Vec3.init(14.5536213, 10.5969315, -3.18638134), Vec3.init(14.5536213, 10.5969315, -5.18637228), 0b111, .empty);
    try collector.checkError();
    try fw.expect(!collector.hadHit());
}

// Not ported: TestClosestHitPerBodyCollisionCollector (casts against bodies through NarrowPhaseQuery::CastShape, needs
// PhysicsTestContext / NarrowPhaseQuery, Phase 5)

// Test 2D shape cast against a box
test "TestCast2DBoxVsBox" {
    var box_shape: RefConst(Shape) = .empty;
    defer box_shape.deinit();
    {
        const size: f32 = 5.0;
        const thickness: f32 = 1.0;
        const points = [_]Vec3{
            Vec3.init(-size, -size, thickness),
            Vec3.init(size, -size, thickness),
            Vec3.init(size, size, thickness),
            Vec3.init(-size, size, thickness),
            Vec3.init(-size, -size, -thickness),
            Vec3.init(size, -size, -thickness),
            Vec3.init(size, size, -thickness),
            Vec3.init(-size, size, -thickness),
        };
        var box_shape_settings = try ConvexHullShapeSettings.init(allocator, &points, .{});
        box_shape_settings.asShapeSettings().setEmbedded();
        defer box_shape_settings.deinit();
        box_shape_settings.max_convex_radius = 0.0;
        var box = try createShape(box_shape_settings.asShapeSettings());
        defer box.deinit();
        box_shape.set(box.get());
    }

    var cast_shape: RefConst(Shape) = .empty;
    defer cast_shape.deinit();
    {
        const size: f32 = 1.0;
        const points = [_]Vec3{
            Vec3.init(-size, -size, 0),
            Vec3.init(size, -size, 0),
            Vec3.init(size, size, 0),
            Vec3.init(-size, size, 0),
        };
        var cast_shape_settings = try ConvexHullShapeSettings.init(allocator, &points, .{});
        cast_shape_settings.asShapeSettings().setEmbedded();
        defer cast_shape_settings.deinit();
        cast_shape_settings.max_convex_radius = 0.0;
        var cast = try createShape(cast_shape_settings.asShapeSettings());
        defer cast.deinit();
        cast_shape.set(cast.get());
    }

    // The 2d box cast touches the surface of the box at the start and moves into it
    var settings: ShapeCastSettings = .{};
    settings.return_deepest_point = true;
    const shape_cast = ShapeCast.init(cast_shape.get().?, Vec3.one(), Mat44.translation(Vec3.init(0, 0, 1)), Vec3.init(0, 0, -10));
    var cast_shape_collector = ClosestHitCollisionCollector(CastShapeCollector).init();
    defer cast_shape_collector.deinit();
    CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, &settings, box_shape.get().?, Vec3.one(), &.{}, Mat44.identity(), .{}, .{}, &cast_shape_collector.base);

    try fw.expect(cast_shape_collector.hadHit());
    try fw.expect(cast_shape_collector.hit.fraction == 0.0);
    try fw.checkApproxEqual(cast_shape_collector.hit.base.penetration_axis.normalized(), Vec3.init(0, 0, -1), .{});
    try fw.checkApproxEqual(cast_shape_collector.hit.base.penetration_depth, 0.0, .{});
    try fw.checkApproxEqual(cast_shape_collector.hit.base.contact_point_on1, Vec3.init(0, 0, 1), .{ .tolerance = 1.0e-4 });
    try fw.checkApproxEqual(cast_shape_collector.hit.base.contact_point_on2, Vec3.init(0, 0, 1), .{ .tolerance = 1.0e-4 });
}

// Not ported: TestCastShapeExtraConvexRadius and TestCastShapeInitiallyIntersecting (they cast against bodies through
// NarrowPhaseQuery::CastShape, need PhysicsTestContext / NarrowPhaseQuery, Phase 5)
