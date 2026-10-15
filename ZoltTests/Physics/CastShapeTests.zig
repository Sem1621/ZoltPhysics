//! Port of: UnitTests/Physics/CastShapeTests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");
const Layers = @import("../Layers.zig").Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const Body = zolt.Body;
const BoxShape = zolt.BoxShape;
const BoxShapeSettings = zolt.BoxShapeSettings;
const CapsuleShape = zolt.CapsuleShape;
const CapsuleShapeSettings = zolt.CapsuleShapeSettings;
const ClosestHitPerBodyCollisionCollector = zolt.ClosestHitPerBodyCollisionCollector;
const CylinderShape = zolt.CylinderShape;
const Quat = zolt.Quat;
const Ref = zolt.Ref;
const RMat44 = zolt.RMat44;
const RShapeCast = zolt.RShapeCast;
const RVec3 = zolt.RVec3;
const ScaledShape = zolt.ScaledShape;
const ScaledShapeSettings = zolt.ScaledShapeSettings;
const ShapeCastResult = zolt.ShapeCastResult;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const math = zolt.math;
const trigonometry = zolt.trigonometry;
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

/// `new ScaledShape(new SphereShape(radius), scale)`
fn createScaledSphere(radius: f32, scale: Vec3) !Ref(Shape) {
    var sphere = RefConst(Shape).init((try SphereShape.create(allocator, radius, .{})).asShape());
    defer sphere.deinit();
    return Ref(Shape).init((try ScaledShape.create(allocator, sphere.get().?, scale)).asShapeMut());
}

test "TestCastShapeSphereVsBox" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Create box to collide against (shape 2)
    // The box is scaled up by a factor 10 in the X axis and then rotated so that the X axis is up
    var box = BoxShapeSettings.init(allocator, Vec3.one(), .{});
    box.asShapeSettings().setEmbedded();
    defer box.deinit();
    var scaled_box = ScaledShapeSettings.init(allocator, box.asShapeSettings(), Vec3.init(10, 1, 1));
    scaled_box.asShapeSettings().setEmbedded();
    defer scaled_box.deinit();
    const body2 = try c.createBodyWithShapeSettings(scaled_box.asShapeSettings(), RVec3.init(0, 1, 0), Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), .static, .discrete, Layers.non_moving, .dont_activate);

    // Set settings
    var settings: ShapeCastSettings = .{};
    settings.return_deepest_point = true;
    settings.back_face_mode_triangles = .collide_with_back_faces;
    settings.back_face_mode_convex = .collide_with_back_faces;

    {
        // Create shape cast
        var normal_sphere = Ref(Shape).init((try SphereShape.create(allocator, 1.0, .{})).asShapeMut());
        defer normal_sphere.deinit();
        const shape_cast = RShapeCast.init(normal_sphere.get().?, Vec3.one(), RMat44.translation(RVec3.init(0, 11, 0)), Vec3.init(0, 1, 0));

        // Shape is intersecting at the start
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &settings, RVec3.zero(), &collector.base, .{});
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 1);
        const result = &collector.hits.items[0];
        try fw.expect(result.base.body_id2.eql(body2.getID()));
        try fw.checkApproxEqual(result.fraction, 0.0, .{});
        try fw.checkApproxEqual(result.base.penetration_axis.normalized(), Vec3.init(0, -1, 0), .{ .tolerance = 1.0e-3 });
        try fw.checkApproxEqual(result.base.penetration_depth, 1.0, .{ .tolerance = 1.0e-5 });
        try fw.checkApproxEqual(result.base.contact_point_on1, Vec3.init(0, 10, 0), .{ .tolerance = 1.0e-3 });
        try fw.checkApproxEqual(result.base.contact_point_on2, Vec3.init(0, 11, 0), .{ .tolerance = 1.0e-3 });
        try fw.expect(!result.is_back_face_hit);
    }

    {
        // This repeats the same test as above but uses scaling at all levels and validate that the penetration depth is still correct
        var scaled_sphere = try createScaledSphere(0.1, Vec3.replicate(5.0));
        defer scaled_sphere.deinit();
        const shape_cast = RShapeCast.init(scaled_sphere.get().?, Vec3.replicate(2.0), RMat44.translation(RVec3.init(0, 11, 0)), Vec3.init(0, 1, 0));

        // Shape is intersecting at the start
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &settings, RVec3.zero(), &collector.base, .{});
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 1);
        const result = &collector.hits.items[0];
        try fw.expect(result.base.body_id2.eql(body2.getID()));
        try fw.checkApproxEqual(result.fraction, 0.0, .{});
        try fw.checkApproxEqual(result.base.penetration_axis.normalized(), Vec3.init(0, -1, 0), .{ .tolerance = 1.0e-3 });
        try fw.checkApproxEqual(result.base.penetration_depth, 1.0, .{ .tolerance = 1.0e-5 });
        try fw.checkApproxEqual(result.base.contact_point_on1, Vec3.init(0, 10, 0), .{ .tolerance = 1.0e-3 });
        try fw.checkApproxEqual(result.base.contact_point_on2, Vec3.init(0, 11, 0), .{ .tolerance = 1.0e-3 });
        try fw.expect(!result.is_back_face_hit);
    }
}

// Tests casting a cylinder that is touching a capsule. The expectation is that it finds an intersection at fraction 0 and with very little penetration.
test "TestCastShapeCylinderVsCapsule" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    var capsule_shape_settings = CapsuleShapeSettings.init(allocator, 2.03114343, 7.19732189, .{});
    capsule_shape_settings.asShapeSettings().setEmbedded();
    defer capsule_shape_settings.deinit();
    _ = try c.createBodyWithShapeSettings(capsule_shape_settings.asShapeSettings(), RVec3.zero(), Quat.identity(), .static, .discrete, Layers.non_moving, .dont_activate);

    var settings: ShapeCastSettings = .{};
    settings.use_shrunken_shape_and_convex_radius = true;

    var cylinder_shape = Ref(Shape).init((try CylinderShape.create(allocator, 0.3, 0.4975, .{})).asShapeMut());
    defer cylinder_shape.deinit();
    // (Jolt passes float literals to RVec3)
    const shape_cast = RShapeCast.init(cylinder_shape.get().?, Vec3.one(), RMat44.translation(RVec3.init(@as(f32, -0.951660156), @as(f32, -2.09155273), @as(f32, -7.63574218))), Vec3.init(0.00244140625, -0.0068359375, 0.0029296875));

    var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer collector.deinit();
    c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &settings, RVec3.zero(), &collector.base, .{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 1);
    const result = &collector.hits.items[0];
    try fw.expect(result.fraction == 0.0);
    try fw.expect(result.base.penetration_depth < 1.0e-4);
}

// Test CastShape ordering according to penetration depth
test "TestCastShapePenetrationDepthOrdering" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Create box to collide against (shape 2)
    var box = BoxShapeSettings.init(allocator, Vec3.init(0.1, 2.0, 2.0), .{});
    box.asShapeSettings().setEmbedded();
    defer box.deinit();

    // Create 10 boxes that are 0.2 thick in the X axis and 4 in Y and Z, put them all next to each other on the X axis starting from X = 0 going to X = 2
    var bodies: [10]*Body = undefined;
    for (&bodies, 0..) |*body, i|
        body.* = try c.createBodyWithShapeSettings(box.asShapeSettings(), RVec3.init(0.1 + 0.2 * @as(f32, @floatFromInt(i)), 0, 0), Quat.identity(), .static, .discrete, Layers.non_moving, .dont_activate);

    // Set settings
    var settings: ShapeCastSettings = .{};
    settings.return_deepest_point = true;
    settings.back_face_mode_triangles = .collide_with_back_faces;
    settings.back_face_mode_convex = .collide_with_back_faces;
    settings.collision_tolerance = 1.0e-5; // Increased precision
    settings.penetration_tolerance = 1.0e-5;

    var sphere = RefConst(Shape).init((try SphereShape.create(allocator, 1.0, .{})).asShape());
    defer sphere.deinit();

    {
        // Create shape cast in X from -5 to 5
        const shape_cast = RShapeCast.init(sphere.get().?, Vec3.one(), RMat44.translation(RVec3.init(-5, 0, 0)), Vec3.init(10, 0, 0));

        // We should hit the first body
        var collector = ClosestHitCollisionCollector(CastShapeCollector).init();
        defer collector.deinit();
        c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &settings, RVec3.zero(), &collector.base, .{});
        try fw.expect(collector.hadHit());
        try fw.expect(collector.hit.base.body_id2.eql(bodies[0].getID()));
        try fw.checkApproxEqual(collector.hit.fraction, 4.0 / 10.0, .{});
        try fw.expect(collector.hit.base.penetration_axis.normalized().dot(Vec3.init(1, 0, 0)) > trigonometry.cos(math.degreesToRadians(1.0)));
        try fw.checkApproxEqual(collector.hit.base.penetration_depth, 0.0, .{});
        try fw.checkApproxEqual(collector.hit.base.contact_point_on1, Vec3.init(0, 0, 0), .{ .tolerance = 2.0e-3 });
        try fw.checkApproxEqual(collector.hit.base.contact_point_on2, Vec3.init(0, 0, 0), .{ .tolerance = 2.0e-3 });
        try fw.expect(!collector.hit.is_back_face_hit);
    }

    {
        // Create shape cast in X from 5 to -5
        const shape_cast = RShapeCast.init(sphere.get().?, Vec3.one(), RMat44.translation(RVec3.init(5, 0, 0)), Vec3.init(-10, 0, 0));

        // We should hit the last body
        var collector = ClosestHitCollisionCollector(CastShapeCollector).init();
        defer collector.deinit();
        c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &settings, RVec3.zero(), &collector.base, .{});
        try fw.expect(collector.hadHit());
        try fw.expect(collector.hit.base.body_id2.eql(bodies[bodies.len - 1].getID()));
        try fw.checkApproxEqual(collector.hit.fraction, 2.0 / 10.0, .{ .tolerance = 1.0e-4 });
        try fw.expect(collector.hit.base.penetration_axis.normalized().dot(Vec3.init(-1, 0, 0)) > trigonometry.cos(math.degreesToRadians(1.0)));
        try fw.checkApproxEqual(collector.hit.base.penetration_depth, 0.0, .{});
        try fw.checkApproxEqual(collector.hit.base.contact_point_on1, Vec3.init(2, 0, 0), .{ .tolerance = 4.0e-4 });
        try fw.checkApproxEqual(collector.hit.base.contact_point_on2, Vec3.init(2, 0, 0), .{ .tolerance = 4.0e-4 });
        try fw.expect(!collector.hit.is_back_face_hit);
    }

    {
        // Create shape cast in X from 1.05 to 11, this should intersect with all bodies and have deepest penetration in bodies[5]
        const shape_cast = RShapeCast.init(sphere.get().?, Vec3.one(), RMat44.translation(RVec3.init(1.05, 0, 0)), Vec3.init(10, 0, 0));

        // We should hit bodies[5]
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &settings, RVec3.zero(), &collector.base, .{});
        try collector.checkError();
        collector.sort();
        try fw.expect(collector.hits.items.len == 10);
        const result = &collector.hits.items[0];
        try fw.expect(result.base.body_id2.eql(bodies[5].getID()));
        try fw.checkApproxEqual(result.fraction, 0.0, .{});
        try fw.expect(result.base.penetration_axis.normalized().dot(Vec3.init(1, 0, 0)) > trigonometry.cos(math.degreesToRadians(1.0)));
        try fw.checkApproxEqual(result.base.penetration_depth, 1.05, .{});
        try fw.checkApproxEqual(result.base.contact_point_on1, Vec3.init(2.05, 0, 0), .{ .tolerance = 2.0e-5 }); // Box starts at 1.0, center of sphere adds 0.05, radius of sphere is 1
        try fw.checkApproxEqual(result.base.contact_point_on2, Vec3.init(1.0, 0, 0), .{ .tolerance = 2.0e-5 }); // Box starts at 1.0
        try fw.expect(!result.is_back_face_hit);
    }
}

// Test casting a capsule against a mesh that is intersecting at fraction 0 and test that it returns the deepest penetration
test "TestDeepestPenetrationAtFraction0" {
    // Create an n x n grid of triangles
    const n = 10;
    const s: f32 = 0.1;
    var triangles: std.ArrayList(Triangle) = .empty;
    defer triangles.deinit(allocator);
    for (0..n) |z_index|
        for (0..n) |x_index| {
            const x: f32 = @floatFromInt(x_index);
            const z: f32 = @floatFromInt(z_index);
            const fx = s * x - s * @as(f32, n) / 2;
            const fz = s * z - s * @as(f32, n) / 2;
            try triangles.append(allocator, Triangle.init(Vec3.init(fx, 0, fz), Vec3.init(fx, 0, fz + s), Vec3.init(fx + s, 0, fz + s), .{}));
            try triangles.append(allocator, Triangle.init(Vec3.init(fx, 0, fz), Vec3.init(fx + s, 0, fz + s), Vec3.init(fx + s, 0, fz), .{}));
        };
    var mesh_settings = try MeshShapeSettings.init(allocator, triangles.items, .{});
    mesh_settings.asShapeSettings().setEmbedded();
    defer mesh_settings.deinit();

    // Create a compound shape with two copies of the mesh
    var compound_settings = StaticCompoundShapeSettings.init(allocator);
    defer compound_settings.deinit();
    try compound_settings.base.addShape(Vec3.zero(), Quat.identity(), mesh_settings.asShapeSettings(), .{});
    try compound_settings.base.addShape(Vec3.init(0, -0.01, 0), Quat.identity(), mesh_settings.asShapeSettings(), .{}); // This will not result in the deepest penetration
    compound_settings.asShapeSettings().setEmbedded();

    // Add it to the scene
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    _ = try c.createBodyWithShapeSettings(compound_settings.asShapeSettings(), RVec3.zero(), Quat.identity(), .static, .discrete, Layers.non_moving, .dont_activate);

    // Add the same compound a little bit lower (this will not result in the deepest penetration)
    _ = try c.createBodyWithShapeSettings(compound_settings.asShapeSettings(), RVec3.init(0, -0.1, 0), Quat.identity(), .static, .discrete, Layers.non_moving, .dont_activate);

    // We want the deepest hit
    var cast_settings: ShapeCastSettings = .{};
    cast_settings.return_deepest_point = true;

    // Create capsule to test
    const capsule_half_height: f32 = 2.0;
    const capsule_radius: f32 = 1.0;
    var cast_shape = RefConst(Shape).init((try CapsuleShape.create(allocator, capsule_half_height, capsule_radius, .{})).asShape());
    defer cast_shape.deinit();

    // Cast the shape starting inside the mesh with a long distance so that internally in the mesh shape the RayAABox4 test will return a low negative fraction.
    // This used to be confused with the penetration depth and would cause an early out and return the wrong result.
    const capsule_offset: f32 = 0.1;
    const shape_cast = RShapeCast.init(cast_shape.get().?, Vec3.one(), RMat44.translation(RVec3.init(0, capsule_half_height + capsule_offset, 0)), Vec3.init(0, -100, 0));

    // Cast first using the closest hit collector
    var collector = ClosestHitCollisionCollector(CastShapeCollector).init();
    defer collector.deinit();
    c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &cast_settings, RVec3.zero(), &collector.base, .{});

    // Check that it indeed found a hit at fraction 0 with the deepest penetration of all triangles
    try fw.expect(collector.hadHit());
    try fw.expect(collector.hit.fraction == 0.0);
    try fw.checkApproxEqual(collector.hit.base.penetration_depth, capsule_radius - capsule_offset, .{ .tolerance = 1.0e-4 });
    try fw.checkApproxEqual(collector.hit.base.penetration_axis.normalized(), Vec3.init(0, -1, 0), .{});
    try fw.checkApproxEqual(collector.hit.base.contact_point_on2, Vec3.zero(), .{});

    // Cast again while triggering a force early out after the first hit
    const MyCollector = struct {
        pub const overrides = .{.addHit};

        base: CastShapeCollector = .init(@This()),
        num_hits: i32 = 0,

        pub fn addHit(self: *@This(), result: *const ShapeCastResult) void {
            _ = result;
            self.num_hits += 1;
            self.base.forceEarlyOut();
        }
    };
    var collector2: MyCollector = .{};
    c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &cast_settings, RVec3.zero(), &collector2.base, .{});

    // Ensure that we indeed stopped after the first hit
    try fw.expect(collector2.num_hits == 1);
}

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

// Test ClosestHitPerBodyCollisionCollector
test "TestClosestHitPerBodyCollisionCollector" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();

    // Create a 1 by 1 by 1 box consisting of 10 slabs
    var compound_settings = StaticCompoundShapeSettings.init(allocator);
    compound_settings.asShapeSettings().setEmbedded();
    defer compound_settings.deinit();
    for (0..10) |i| {
        var slab = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(0.05, 0.5, 0.5), .{})).asShape());
        defer slab.deinit();
        try compound_settings.base.addShapePtr(Vec3.init(0.1 * @as(f32, @floatFromInt(i)) - 0.45, 0, 0), Quat.identity(), slab.get(), .{});
    }

    // Create 2 instances
    const body1 = try c.createBodyWithShapeSettings(compound_settings.asShapeSettings(), RVec3.zero(), Quat.identity(), .static, .discrete, Layers.non_moving, .dont_activate);
    const body2 = try c.createBodyWithShapeSettings(compound_settings.asShapeSettings(), RVec3.init(1.0, 0, 0), Quat.identity(), .static, .discrete, Layers.non_moving, .dont_activate);

    const cast_settings: ShapeCastSettings = .{};

    var sphere = SphereShape.init(allocator, 0.1, .{});
    sphere.asShape().setEmbedded();
    defer sphere.asShapeMut().deinit();

    // Override ClosestHitPerBodyCollisionCollector so that we can count the number of calls to AddHit
    const MyClosestHitPerBodyCollisionCollector = struct {
        pub const overrides = .{.addHit};

        base: ClosestHitPerBodyCollisionCollector(CastShapeCollector),
        num_calls: i32 = 0,

        fn init() @This() {
            return .{ .base = .initDerived(@This(), allocator) };
        }

        pub fn addHit(self: *@This(), result: *const ShapeCastResult) void {
            self.base.addHit(result); // ClosestHitPerBodyCollisionCollector<CastShapeCollector>::AddHit(inResult)

            self.num_calls += 1;
        }
    };

    {
        const shape_cast = RShapeCast.init(sphere.asShape(), Vec3.one(), RMat44.translation(RVec3.init(-1, 0, 0)), Vec3.init(3, 0, 0));

        // Check that the all hit collector finds 20 hits (2 x 10 slabs)
        var all_collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer all_collector.deinit();
        c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &cast_settings, RVec3.zero(), &all_collector.base, .{});
        try all_collector.checkError();
        all_collector.sort();
        try fw.expect(all_collector.hits.items.len == 20);
        for (0..10) |i| {
            try fw.expect(all_collector.hits.items[i].base.body_id2.eql(body1.getID()));
            try fw.checkApproxEqual(all_collector.hits.items[i].base.contact_point_on1, Vec3.init(-0.5 + 0.1 * @as(f32, @floatFromInt(i)), 0, 0), .{});
        }
        for (0..10) |i| {
            try fw.expect(all_collector.hits.items[10 + i].base.body_id2.eql(body2.getID()));
            try fw.checkApproxEqual(all_collector.hits.items[10 + i].base.contact_point_on1, Vec3.init(0.5 + 0.1 * @as(f32, @floatFromInt(i)), 0, 0), .{});
        }

        // Check that the closest hit per body collector only finds 2
        var closest_collector = MyClosestHitPerBodyCollisionCollector.init();
        defer closest_collector.base.deinit();
        c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &cast_settings, RVec3.zero(), &closest_collector.base.base, .{});
        try closest_collector.base.checkError();
        try fw.expect(closest_collector.num_calls == 2); // Spatial ordering by the broad phase and compound shape and the early out value should have resulted in only 2 calls to AddHit
        closest_collector.base.sort();
        try fw.expect(closest_collector.base.hits.items.len == 2);
        try fw.expect(closest_collector.base.hits.items[0].base.body_id2.eql(body1.getID()));
        try fw.checkApproxEqual(closest_collector.base.hits.items[0].base.contact_point_on1, Vec3.init(-0.5, 0, 0), .{});
        try fw.expect(closest_collector.base.hits.items[1].base.body_id2.eql(body2.getID()));
        try fw.checkApproxEqual(closest_collector.base.hits.items[1].base.contact_point_on1, Vec3.init(0.5, 0, 0), .{});
    }

    {
        // Cast in reverse direction
        const shape_cast = RShapeCast.init(sphere.asShape(), Vec3.one(), RMat44.translation(RVec3.init(2, 0, 0)), Vec3.init(-3, 0, 0));

        // Check that the all hit collector finds 20 hits (2 x 10 slabs)
        var all_collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer all_collector.deinit();
        c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &cast_settings, RVec3.zero(), &all_collector.base, .{});
        try all_collector.checkError();
        all_collector.sort();
        try fw.expect(all_collector.hits.items.len == 20);
        for (0..10) |i| {
            try fw.expect(all_collector.hits.items[i].base.body_id2.eql(body2.getID()));
            try fw.checkApproxEqual(all_collector.hits.items[i].base.contact_point_on1, Vec3.init(1.5 - 0.1 * @as(f32, @floatFromInt(i)), 0, 0), .{});
        }
        for (0..10) |i| {
            try fw.expect(all_collector.hits.items[10 + i].base.body_id2.eql(body1.getID()));
            try fw.checkApproxEqual(all_collector.hits.items[10 + i].base.contact_point_on1, Vec3.init(0.5 - 0.1 * @as(f32, @floatFromInt(i)), 0, 0), .{});
        }

        // Check that the closest hit per body collector only finds 2
        var closest_collector = MyClosestHitPerBodyCollisionCollector.init();
        defer closest_collector.base.deinit();
        c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &cast_settings, RVec3.zero(), &closest_collector.base.base, .{});
        try closest_collector.base.checkError();
        try fw.expect(closest_collector.num_calls == 2); // Spatial ordering by the broad phase and compound shape and the early out value should have resulted in only 2 calls to AddHit
        closest_collector.base.sort();
        try fw.expect(closest_collector.base.hits.items.len == 2);
        try fw.expect(closest_collector.base.hits.items[0].base.body_id2.eql(body2.getID()));
        try fw.checkApproxEqual(closest_collector.base.hits.items[0].base.contact_point_on1, Vec3.init(1.5, 0, 0), .{});
        try fw.expect(closest_collector.base.hits.items[1].base.body_id2.eql(body1.getID()));
        try fw.checkApproxEqual(closest_collector.base.hits.items[1].base.contact_point_on1, Vec3.init(0.5, 0, 0), .{});
    }
}

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

// Test CastShape with extra convex radius
test "TestCastShapeExtraConvexRadius" {
    // Set settings
    var settings: ShapeCastSettings = .{};
    settings.extra_convex_radius = 0.5; // Add 0.5 extra radius
    settings.return_deepest_point = true;
    settings.back_face_mode_triangles = .collide_with_back_faces;
    settings.back_face_mode_convex = .collide_with_back_faces;

    // Create a scaled sphere: effective radius=0.5
    var scaled_sphere = try createScaledSphere(0.1, Vec3.replicate(5.0));
    defer scaled_sphere.deinit();

    // Scale the sphere during the cast: effective radius=2
    // We want to hit an object at y=11, we start it at y=14 so it should hit after moving by 0.5
    // Cast length is set low enough so that when the bounding box of the swept object is not enlarged by 0.5, it will hit the target object
    const shape_cast = RShapeCast.init(scaled_sphere.get().?, Vec3.replicate(4.0), RMat44.translation(RVec3.init(0, 14, 0)), Vec3.init(0, -0.8, 0));

    {
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();

        // Create box to collide against, effective top of box is at y=11
        var box = BoxShapeSettings.init(allocator, Vec3.one(), .{});
        box.asShapeSettings().setEmbedded();
        defer box.deinit();
        var scaled_box = ScaledShapeSettings.init(allocator, box.asShapeSettings(), Vec3.init(10, 1, 1));
        scaled_box.asShapeSettings().setEmbedded();
        defer scaled_box.deinit();
        const body2 = try c.createBodyWithShapeSettings(scaled_box.asShapeSettings(), RVec3.init(0, 1, 0), Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), .static, .discrete, Layers.non_moving, .dont_activate);

        // Shape is intersecting at the start
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &settings, RVec3.zero(), &collector.base, .{});
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 1);
        const result = &collector.hits.items[0];
        try fw.expect(result.base.body_id2.eql(body2.getID()));
        try fw.checkApproxEqual(result.fraction, 0.5 / 0.8, .{});
        try fw.checkApproxEqual(result.base.penetration_axis.normalized(), Vec3.init(0, -1, 0), .{});
        try fw.expect(result.base.penetration_depth == 0);
        try fw.checkApproxEqual(result.base.contact_point_on1, Vec3.init(0, 11, 0), .{});
        try fw.checkApproxEqual(result.base.contact_point_on2, Vec3.init(0, 11, 0), .{});
        try fw.expect(!result.is_back_face_hit);
    }

    {
        var c: PhysicsTestContext = undefined;
        try c.init(allocator, .{});
        defer c.deinit();

        // Create triangle to collide against, triangle is at y=11
        var triangle = TriangleShapeSettings.init(allocator, Vec3.init(50, 10, 0), Vec3.init(-50, 10, 0), Vec3.init(0, 10, 50), .{});
        triangle.asShapeSettings().setEmbedded();
        defer triangle.deinit();
        const body2 = try c.createBodyWithShapeSettings(triangle.asShapeSettings(), RVec3.init(0, 1, 0), Quat.identity(), .static, .discrete, Layers.non_moving, .dont_activate);

        // Shape is intersecting at the start
        var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
        defer collector.deinit();
        c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &settings, RVec3.zero(), &collector.base, .{});
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 1);
        const result = &collector.hits.items[0];
        try fw.expect(result.base.body_id2.eql(body2.getID()));
        try fw.checkApproxEqual(result.fraction, 0.5 / 0.8, .{});
        try fw.checkApproxEqual(result.base.penetration_axis.normalized(), Vec3.init(0, -1, 0), .{});
        try fw.expect(result.base.penetration_depth == 0);
        try fw.checkApproxEqual(result.base.contact_point_on1, Vec3.init(0, 11, 0), .{});
        try fw.checkApproxEqual(result.base.contact_point_on2, Vec3.init(0, 11, 0), .{});
        try fw.expect(!result.is_back_face_hit);
    }
}

// Test CastShape between two boxes that are initially intersecting and off center
test "TestCastShapeInitiallyIntersecting" {
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{});
    defer c.deinit();
    _ = try c.createBox(RVec3.zero(), Quat.identity(), .static, .discrete, Layers.non_moving, Vec3.init(1.5, 0.5, 1.5), .{ .activation = .dont_activate });

    var box_shape = RefConst(Shape).init((try BoxShape.create(allocator, Vec3.init(0.5, 0.5, 0.5), .{})).asShape());
    defer box_shape.deinit();
    const shape_cast = RShapeCast.init(box_shape.get().?, Vec3.replicate(1.0), RMat44.rotationTranslation(Quat.identity(), RVec3.init(0.01, 0.95, 0)), Vec3.init(0, -0.5, 0));

    const settings: ShapeCastSettings = .{};
    var collector = ClosestHitCollisionCollector(CastShapeCollector).init();
    defer collector.deinit();
    c.getSystem().getNarrowPhaseQuery().castShape(&shape_cast, &settings, RVec3.zero(), &collector.base, .{});

    try fw.expect(collector.hadHit());
    try fw.expect(collector.hit.fraction == 0.0);
    try fw.expect(!collector.hit.is_back_face_hit);
}
