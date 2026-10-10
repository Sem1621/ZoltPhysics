//! Port of: UnitTests/Physics/ActiveEdgesTests.cpp
//! Status: partial
//! Missing: DiscreteCubeSlideMesh, DiscreteCubeSlideHeightField, LinearCastCubeSlideMesh, LinearCastCubeSlideHeightField
//!   (sDiscreteCubeSlide / sLinearCastCubeSlide simulate a box sliding over the shape, need PhysicsTestContext, Phase 5)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const CapsuleShapeSettings = zolt.CapsuleShapeSettings;
const CastShapeCollector = zolt.CastShapeCollector;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollisionDispatch = zolt.CollisionDispatch;
const Float3 = zolt.Float3;
const HeightFieldShapeSettings = zolt.HeightFieldShapeSettings;
const Mat44 = zolt.Mat44;
const MeshShapeSettings = zolt.MeshShapeSettings;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeSettings = zolt.ShapeSettings;
const SphereShape = zolt.SphereShape;
const Triangle = zolt.Triangle;
const TriangleList = zolt.TriangleList;
const Vec3 = zolt.Vec3;

const allocator = std.testing.allocator;

const capsule_probe_offset: f32 = 0.1; // How much to offset the probe from y = 0 in order to avoid hitting a back instead of a front face
const capsule_radius: f32 = 0.1;

/// `settings.Create().Get()`: the shape of a newly created result (the caller releases the reference)
fn createShape(settings: *ShapeSettings) !RefConst(Shape) {
    var result = try settings.createShape(allocator);
    defer result.deinit();
    return RefConst(Shape).init(result.getPtr());
}

// Create a capsule as our probe
fn createProbeCapsule() !RefConst(Shape) {
    // Ensure capsule is long enough so that when active edges mode is on, we will always get a horizontal penetration axis rather than a vertical one
    var capsule = CapsuleShapeSettings.init(allocator, 1.0, capsule_radius, .{});
    capsule.asShapeSettings().setEmbedded();
    defer capsule.deinit();
    return createShape(capsule.asShapeSettings());
}

// Create a flat mesh shape consisting of 7 x 7 quads, we know that only the outer edges of this shape are active
fn createMeshShape() !Ref(ShapeSettings) {
    var triangles: TriangleList = .empty;
    defer triangles.deinit(allocator);
    for (0..7) |z| {
        for (0..7) |x| {
            const fx = @as(f32, @floatFromInt(x)) - 3.5;
            const fz = @as(f32, @floatFromInt(z)) - 3.5;
            try triangles.append(allocator, .init(Vec3.init(fx, 0, fz), Vec3.init(fx, 0, fz + 1), Vec3.init(fx + 1, 0, fz + 1), .{}));
            try triangles.append(allocator, .init(Vec3.init(fx, 0, fz), Vec3.init(fx + 1, 0, fz + 1), Vec3.init(fx + 1, 0, fz), .{}));
        }
    }

    return .init((try MeshShapeSettings.create(allocator, triangles.items, .{})).asShapeSettings());
}

// Create a flat height field shape that has the same properties as the mesh shape
fn createHeightFieldShape() !Ref(ShapeSettings) {
    const samples = [_]f32{0} ** (8 * 8);
    return .init((try HeightFieldShapeSettings.create(allocator, &samples, Vec3.init(-3.5, 0, -3.5), Vec3.one(), 8, .{})).asShapeSettings());
}

// This struct indicates what we hope to find as hit
const ExpectedHit = struct {
    position: Vec3,
    penetration_axis: Vec3,
};

// Compare expected hits with returned hits
fn checkMatch(comptime ResultType: type, results: []const ResultType, expected_hits: []const ExpectedHit, accuracy_sq: f32) !void {
    try fw.expect(results.len == expected_hits.len);

    for (expected_hits) |hit| {
        var found = false;
        for (results) |*result| {
            const r = if (ResultType == zolt.ShapeCastResult) &result.base else result;
            if (r.contact_point_on2.isClose(hit.position, .{ .max_dist_sq = accuracy_sq }) and r.penetration_axis.normalized().isClose(hit.penetration_axis, .{ .max_dist_sq = accuracy_sq })) {
                found = true;
                break;
            }
        }
        try fw.expect(found);
    }
}

// Collide our probe against the test shape and validate the hit results
fn testCollideShapeProbe(probe_shape: *const Shape, test_shape: *const Shape, test_shape_scale: Vec3, settings: *const CollideShapeSettings, probe_shape_pos: Vec3, expected_hits: []const ExpectedHit) !void {
    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collector.deinit();
    CollisionDispatch.collideShapeVsShape(probe_shape, test_shape, Vec3.one(), test_shape_scale, Mat44.translation(probe_shape_pos), Mat44.identity(), .{}, .{}, settings, &collector.base, &.{});
    try collector.checkError();

    try checkMatch(zolt.CollideShapeResult, collector.hits.items, expected_hits, 1.0e-8);
}

// Collide a probe shape against our test shape in various locations to verify active edge behavior
fn testCollideShape(test_shape_settings: *ShapeSettings, test_shape_scale: Vec3, active_edges_only: bool) !void {
    var settings: CollideShapeSettings = .{};
    settings.active_edge_mode = if (active_edges_only) .collide_only_with_active else .collide_with_all;

    var test_shape_ref = try createShape(test_shape_settings);
    defer test_shape_ref.deinit();
    const test_shape = test_shape_ref.get().?;
    var capsule_ref = try createProbeCapsule();
    defer capsule_ref.deinit();
    const capsule = capsule_ref.get().?;

    // Test hitting all active edges
    try testCollideShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(-3.5, capsule_probe_offset, 0), &.{.{ .position = Vec3.init(-3.5, 0, 0), .penetration_axis = Vec3.init(1, 0, 0) }});
    try testCollideShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(3.5, capsule_probe_offset, 0), &.{.{ .position = Vec3.init(3.5, 0, 0), .penetration_axis = Vec3.init(-1, 0, 0) }});
    try testCollideShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(0, capsule_probe_offset, -3.5), &.{.{ .position = Vec3.init(0, 0, -3.5), .penetration_axis = Vec3.init(0, 0, 1) }});
    try testCollideShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(0, capsule_probe_offset, 3.5), &.{.{ .position = Vec3.init(0, 0, 3.5), .penetration_axis = Vec3.init(0, 0, -1) }});

    // Test hitting internal edges, this should return two hits
    try testCollideShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(-2.5, capsule_probe_offset, 0), &.{
        .{ .position = Vec3.init(-2.5, 0, 0), .penetration_axis = if (active_edges_only) Vec3.init(0, -1, 0) else Vec3.init(-1, 0, 0) },
        .{ .position = Vec3.init(-2.5, 0, 0), .penetration_axis = if (active_edges_only) Vec3.init(0, -1, 0) else Vec3.init(1, 0, 0) },
    });
    try testCollideShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(0, capsule_probe_offset, -2.5), &.{
        .{ .position = Vec3.init(0, 0, -2.5), .penetration_axis = if (active_edges_only) Vec3.init(0, -1, 0) else Vec3.init(0, 0, -1) },
        .{ .position = Vec3.init(0, 0, -2.5), .penetration_axis = if (active_edges_only) Vec3.init(0, -1, 0) else Vec3.init(0, 0, -1) },
    });

    // Test hitting an interior diagonal, this should return two hits
    try testCollideShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(-3.0, capsule_probe_offset, 0), &.{
        .{ .position = Vec3.init(-3.0, 0, 0), .penetration_axis = if (active_edges_only) Vec3.init(0, -1, 0) else test_shape_scale.mul(Vec3.init(1, 0, -1)).normalized() },
        .{ .position = Vec3.init(-3.0, 0, 0), .penetration_axis = if (active_edges_only) Vec3.init(0, -1, 0) else test_shape_scale.mul(Vec3.init(-1, 0, 1)).normalized() },
    });
}

test "CollideShapeMesh" {
    var shape = try createMeshShape();
    defer shape.deinit();

    try testCollideShape(shape.get().?, Vec3.one(), false);

    try testCollideShape(shape.get().?, Vec3.one(), true);

    try testCollideShape(shape.get().?, Vec3.init(-1, 1, 1), false);

    try testCollideShape(shape.get().?, Vec3.init(-1, 1, 1), true);
}

test "CollideShapeHeightField" {
    var shape = try createHeightFieldShape();
    defer shape.deinit();

    try testCollideShape(shape.get().?, Vec3.one(), false);

    try testCollideShape(shape.get().?, Vec3.one(), true);

    try testCollideShape(shape.get().?, Vec3.init(-1, 1, 1), false);

    try testCollideShape(shape.get().?, Vec3.init(-1, 1, 1), true);
}

// Cast our probe against the test shape and validate the hit results
fn testCastShapeProbe(probe_shape: *const Shape, test_shape: *const Shape, test_shape_scale: Vec3, settings: *const ShapeCastSettings, probe_shape_pos: Vec3, probe_shape_direction: Vec3, expected_hits: []const ExpectedHit) !void {
    var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer collector.deinit();
    const shape_cast = ShapeCast.init(probe_shape, Vec3.one(), Mat44.translation(probe_shape_pos), probe_shape_direction);
    CollisionDispatch.castShapeVsShapeLocalSpace(&shape_cast, settings, test_shape, test_shape_scale, &.{}, Mat44.identity(), .{}, .{}, &collector.base);
    try collector.checkError();

    try checkMatch(zolt.ShapeCastResult, collector.hits.items, expected_hits, 1.0e-6);
}

// Cast a probe shape against our test shape in various locations to verify active edge behavior
fn testCastShape(test_shape_settings: *ShapeSettings, test_shape_scale: Vec3, active_edges_only: bool) !void {
    var settings: ShapeCastSettings = .{};
    settings.active_edge_mode = if (active_edges_only) .collide_only_with_active else .collide_with_all;
    settings.return_deepest_point = true;

    var test_shape_ref = try createShape(test_shape_settings);
    defer test_shape_ref.deinit();
    const test_shape = test_shape_ref.get().?;
    var capsule_ref = try createProbeCapsule();
    defer capsule_ref.deinit();
    const capsule = capsule_ref.get().?;

    // Test hitting all active edges
    try testCastShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(-4, capsule_probe_offset, 0), Vec3.init(0.5, 0, 0), &.{.{ .position = Vec3.init(-3.5, 0, 0), .penetration_axis = Vec3.init(1, 0, 0) }});
    try testCastShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(4, capsule_probe_offset, 0), Vec3.init(-0.5, 0, 0), &.{.{ .position = Vec3.init(3.5, 0, 0), .penetration_axis = Vec3.init(-1, 0, 0) }});
    try testCastShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(0, capsule_probe_offset, -4), Vec3.init(0, 0, 0.5), &.{.{ .position = Vec3.init(0, 0, -3.5), .penetration_axis = Vec3.init(0, 0, 1) }});
    try testCastShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(0, capsule_probe_offset, 4), Vec3.init(0, 0, -0.5), &.{.{ .position = Vec3.init(0, 0, 3.5), .penetration_axis = Vec3.init(0, 0, -1) }});

    // Test hitting internal edges, this should return two hits
    try testCastShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(-2.5 - 1.1 * capsule_radius, capsule_probe_offset, 0), Vec3.init(0.2 * capsule_radius, 0, 0), &.{
        .{ .position = Vec3.init(-2.5, 0, 0), .penetration_axis = if (active_edges_only) Vec3.init(0, -1, 0) else Vec3.init(-1, 0, 0) },
        .{ .position = Vec3.init(-2.5, 0, 0), .penetration_axis = if (active_edges_only) Vec3.init(0, -1, 0) else Vec3.init(1, 0, 0) },
    });
    try testCastShapeProbe(capsule, test_shape, test_shape_scale, &settings, Vec3.init(0, capsule_probe_offset, -2.5 - 1.1 * capsule_radius), Vec3.init(0, 0, 0.2 * capsule_radius), &.{
        .{ .position = Vec3.init(0, 0, -2.5), .penetration_axis = if (active_edges_only) Vec3.init(0, -1, 0) else Vec3.init(0, 0, -1) },
        .{ .position = Vec3.init(0, 0, -2.5), .penetration_axis = if (active_edges_only) Vec3.init(0, -1, 0) else Vec3.init(0, 0, -1) },
    });
}

test "CastShapeMesh" {
    var shape = try createMeshShape();
    defer shape.deinit();

    try testCastShape(shape.get().?, Vec3.one(), false);

    try testCastShape(shape.get().?, Vec3.one(), true);

    try testCastShape(shape.get().?, Vec3.init(-1, 1, 1), false);

    try testCastShape(shape.get().?, Vec3.init(-1, 1, 1), true);
}

test "CastShapeHeightField" {
    var shape = try createHeightFieldShape();
    defer shape.deinit();

    try testCastShape(shape.get().?, Vec3.one(), false);

    try testCastShape(shape.get().?, Vec3.one(), true);

    try testCastShape(shape.get().?, Vec3.init(-1, 1, 1), false);

    try testCastShape(shape.get().?, Vec3.init(-1, 1, 1), true);
}

// Not ported: sDiscreteCubeSlide, DiscreteCubeSlideMesh, DiscreteCubeSlideHeightField, sLinearCastCubeSlide,
// LinearCastCubeSlideMesh and LinearCastCubeSlideHeightField (they simulate a box sliding over the shape, need
// PhysicsTestContext, Phase 5)

test "TestNonManifoldMesh" {
    // Test 3 triangles in a plane that all share the same edge
    // Normally the shared edge would not be active, but since the mesh is non-manifold we expect all of them to be active
    const triangles = [_]Triangle{
        .fromFloat3(Float3.init(0, 0, -1), Float3.init(0, 0, 1), Float3.init(1, 0, 0), .{ .material_index = 0 }),
        .fromFloat3(Float3.init(0, 0, 1), Float3.init(0, 0, -1), Float3.init(-1, 0, 0), .{ .material_index = 0 }),
        .fromFloat3(Float3.init(0, 0, 1), Float3.init(0, 0, -1), Float3.init(-0.5, 0, 0), .{ .material_index = 0 }),
    };

    var mesh_settings = try MeshShapeSettings.init(allocator, &triangles, .{});
    defer mesh_settings.deinit();
    var shape_ref = try createShape(mesh_settings.asShapeSettings());
    defer shape_ref.deinit();
    const shape = shape_ref.get().?;

    var sphere_ref = RefConst(Shape).init((try SphereShape.create(allocator, 0.1, .{})).asShape());
    defer sphere_ref.deinit();
    const sphere = sphere_ref.get().?;

    var settings: CollideShapeSettings = .{};
    settings.active_edge_mode = .collide_only_with_active;

    // Collide a sphere on both sides of the active edge so that a 45 degree normal will be found then the edge is active.
    // An inactive edge will return a normal that is perpendicular to the plane.
    {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(sphere, shape, Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(0.05, 0.05, 0)), Mat44.identity(), .{}, .{}, &settings, &collector.base, &.{});
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 3);

        // We expect one interior hit because the sphere is above the triangle and 2 active edge hits that provide a normal pointing towards the sphere
        var num_interior: i32 = 0;
        var num_on_shared_edge: i32 = 0;
        for (collector.hits.items) |*r| {
            if (r.contact_point_on2.isClose(Vec3.init(0.05, 0.0, 0.0), .{})) {
                try fw.checkApproxEqual(r.penetration_axis.normalized(), Vec3.init(0, -1, 0), .{});
                num_interior += 1;
            } else if (r.contact_point_on2.isNearZero(.{})) {
                try fw.checkApproxEqual(r.penetration_axis.normalized(), Vec3.init(-1, -1, 0).normalized(), .{ .tolerance = 1.0e-5 });
                num_on_shared_edge += 1;
            }
        }
        try fw.expect(num_interior == 1);
        try fw.expect(num_on_shared_edge == 2);
    }

    {
        var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
        defer collector.deinit();
        CollisionDispatch.collideShapeVsShape(sphere, shape, Vec3.one(), Vec3.one(), Mat44.translation(Vec3.init(-0.05, 0.05, 0)), Mat44.identity(), .{}, .{}, &settings, &collector.base, &.{});
        try collector.checkError();
        try fw.expect(collector.hits.items.len == 3);

        // We expect 2 interior hits because the sphere is above the triangle and 1 active edge hit that provide a normal pointing towards the sphere
        var num_interior: i32 = 0;
        var num_on_shared_edge: i32 = 0;
        for (collector.hits.items) |*r| {
            if (r.contact_point_on2.isClose(Vec3.init(-0.05, 0.0, 0.0), .{})) {
                try fw.checkApproxEqual(r.penetration_axis.normalized(), Vec3.init(0, -1, 0), .{});
                num_interior += 1;
            } else if (r.contact_point_on2.isNearZero(.{})) {
                try fw.checkApproxEqual(r.penetration_axis.normalized(), Vec3.init(1, -1, 0).normalized(), .{ .tolerance = 1.0e-5 });
                num_on_shared_edge += 1;
            }
        }
        try fw.expect(num_interior == 2);
        try fw.expect(num_on_shared_edge == 1);
    }
}
