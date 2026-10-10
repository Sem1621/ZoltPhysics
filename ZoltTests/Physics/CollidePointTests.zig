//! Port of: UnitTests/Physics/CollidePointTests.cpp
//! Status: partial
//! Missing: the NarrowPhaseQuery overloads of sTestHit / sTestMiss and the body part of TestCollidePointVsMesh
//!   (creating the mesh body with a random position and rotation and testing the points through NarrowPhaseQuery;
//!   need PhysicsTestContext / NarrowPhaseQuery, Phase 5)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const BoxShape = zolt.BoxShape;
const CapsuleShape = zolt.CapsuleShape;
const CollidePointCollector = zolt.CollidePointCollector;
const ConvexHullShapeSettings = zolt.ConvexHullShapeSettings;
const CylinderShape = zolt.CylinderShape;
const IndexedTriangle = zolt.IndexedTriangle;
const Mat44 = zolt.Mat44;
const MeshShapeSettings = zolt.MeshShapeSettings;
const MutableCompoundShapeSettings = zolt.MutableCompoundShapeSettings;
const OffsetCenterOfMassShapeSettings = zolt.OffsetCenterOfMassShapeSettings;
const Quat = zolt.Quat;
const RefConst = zolt.RefConst;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const ScaledShape = zolt.ScaledShape;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const SphereShape = zolt.SphereShape;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const TaperedCapsuleShapeSettings = zolt.TaperedCapsuleShapeSettings;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const allocator = std.testing.allocator;

// Probe directions in the direction of the faces
const cube_probes = [_]Vec3{
    Vec3.init(-1.0, 0, 0),
    Vec3.init(1.0, 0, 0),
    Vec3.init(0, -1.0, 0),
    Vec3.init(0, 1.0, 0),
    Vec3.init(0, 0, -1.0),
    Vec3.init(0, 0, 1.0),
};

// Probe directions in the direction of the faces
const cube_and_zero_probes = [_]Vec3{
    Vec3.init(0, 0, 0),
    Vec3.init(-1.0, 0, 0),
    Vec3.init(1.0, 0, 0),
    Vec3.init(0, -1.0, 0),
    Vec3.init(0, 1.0, 0),
    Vec3.init(0, 0, -1.0),
    Vec3.init(0, 0, 1.0),
};

// Probes in the xy-plane
const xy_probes = [_]Vec3{
    Vec3.init(-1.0, 0, 0),
    Vec3.init(1.0, 0, 0),
    Vec3.init(0, 0, -1.0),
    Vec3.init(0, 0, 1.0),
};

// Probes in the xy-plane and zero
const xy_and_zero_probes = [_]Vec3{
    Vec3.init(0, 0, 0),
    Vec3.init(-1.0, 0, 0),
    Vec3.init(1.0, 0, 0),
    Vec3.init(0, 0, -1.0),
    Vec3.init(0, 0, 1.0),
};

// Vertices of a cube
const cube_vertices = [_]Vec3{
    Vec3.init(-1.0, -1.0, -1.0),
    Vec3.init(1.0, -1.0, -1.0),
    Vec3.init(-1.0, -1.0, 1.0),
    Vec3.init(1.0, -1.0, 1.0),
    Vec3.init(-1.0, 1.0, -1.0),
    Vec3.init(1.0, 1.0, -1.0),
    Vec3.init(-1.0, 1.0, 1.0),
    Vec3.init(1.0, 1.0, 1.0),
};

fn testHit(shape: *const Shape, position: Vec3) !void {
    var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer collector.deinit();
    shape.collidePoint(position.sub(shape.getCenterOfMass()), .{}, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 1);
}

// Not ported: sTestHit(const NarrowPhaseQuery &, RVec3Arg, const BodyID &) (needs NarrowPhaseQuery, Phase 5)

fn testMiss(shape: *const Shape, position: Vec3) !void {
    var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer collector.deinit();
    shape.collidePoint(position.sub(shape.getCenterOfMass()), .{}, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 0);
}

// Not ported: sTestMiss(const NarrowPhaseQuery &, RVec3Arg) (needs NarrowPhaseQuery, Phase 5)

/// `settings.Create().Get()`: the shape of a newly created result (the caller releases the reference)
fn createShape(settings: *ShapeSettings) !RefConst(Shape) {
    var result = try settings.createShape(allocator);
    defer result.deinit();
    return RefConst(Shape).init(result.getPtr());
}

test "TestCollidePointVsBox" {
    const half_box_size = Vec3.init(0.1, 0.2, 0.3);
    var shape = RefConst(Shape).init((try BoxShape.create(allocator, half_box_size, .{})).asShape());
    defer shape.deinit();

    // Hits
    for (cube_and_zero_probes) |probe|
        try testHit(shape.get().?, half_box_size.mulScalar(0.99).mul(probe));

    // Misses
    for (cube_probes) |probe|
        try testMiss(shape.get().?, half_box_size.mulScalar(1.01).mul(probe));
}

test "TestCollidePointVsSphere" {
    const radius: f32 = 0.1;
    var shape = RefConst(Shape).init((try SphereShape.create(allocator, radius, .{})).asShape());
    defer shape.deinit();

    // Hits
    for (cube_and_zero_probes) |probe|
        try testHit(shape.get().?, Vec3.replicate(radius).mulScalar(0.99).mul(probe));

    // Misses
    for (cube_probes) |probe|
        try testMiss(shape.get().?, Vec3.replicate(radius).mulScalar(1.01).mul(probe));
}

test "TestCollidePointVsCapsule" {
    const half_height: f32 = 0.2;
    const radius: f32 = 0.1;
    var shape = RefConst(Shape).init((try CapsuleShape.create(allocator, half_height, radius, .{})).asShape());
    defer shape.deinit();

    // Top hits
    for (xy_and_zero_probes) |probe|
        try testHit(shape.get().?, probe.mulScalar(0.99 * radius).add(Vec3.init(0, half_height, 0)));

    // Center hit
    try testHit(shape.get().?, Vec3.zero());

    // Bottom hits
    for (xy_and_zero_probes) |probe|
        try testHit(shape.get().?, probe.mulScalar(0.99 * radius).add(Vec3.init(0, -half_height, 0)));

    // Misses
    for (cube_probes) |probe|
        try testMiss(shape.get().?, Vec3.init(radius, half_height + radius, radius).mulScalar(1.01).mul(probe));
}

test "TestCollidePointVsTaperedCapsule" {
    const half_height: f32 = 0.4;
    const top_radius: f32 = 0.1;
    const bottom_radius: f32 = 0.2;
    var settings = TaperedCapsuleShapeSettings.init(allocator, half_height, top_radius, bottom_radius, .{});
    defer settings.deinit();
    var shape = try createShape(settings.asShapeSettings());
    defer shape.deinit();

    // Top hits
    for (xy_and_zero_probes) |probe|
        try testHit(shape.get().?, probe.mulScalar(0.99 * top_radius).add(Vec3.init(0, half_height, 0)));

    // Center hit
    try testHit(shape.get().?, Vec3.zero());

    // Bottom hits
    for (xy_and_zero_probes) |probe|
        try testHit(shape.get().?, probe.mulScalar(0.99 * bottom_radius).add(Vec3.init(0, -half_height, 0)));

    // Top misses
    try testMiss(shape.get().?, Vec3.init(0, half_height + top_radius + 0.01, 0));
    for (xy_probes) |probe|
        try testMiss(shape.get().?, probe.mulScalar(1.01 * top_radius).add(Vec3.init(0, half_height, 0)));

    // Bottom misses
    try testMiss(shape.get().?, Vec3.init(0, -half_height - bottom_radius - 0.01, 0));
    for (xy_probes) |probe|
        try testMiss(shape.get().?, probe.mulScalar(1.01 * bottom_radius).add(Vec3.init(0, -half_height, 0)));
}

test "TestCollidePointVsCylinder" {
    const half_height: f32 = 0.2;
    const radius: f32 = 0.1;
    var shape = RefConst(Shape).init((try CylinderShape.create(allocator, half_height, radius, .{})).asShape());
    defer shape.deinit();

    // Top hits
    for (xy_and_zero_probes) |probe|
        try testHit(shape.get().?, probe.mulScalar(radius).add(Vec3.init(0, half_height, 0)).mulScalar(0.99));

    // Center hit
    try testHit(shape.get().?, Vec3.zero());

    // Bottom hits
    for (xy_and_zero_probes) |probe|
        try testHit(shape.get().?, probe.mulScalar(radius).add(Vec3.init(0, -half_height, 0)).mulScalar(0.99));

    // Misses
    for (cube_probes) |probe|
        try testMiss(shape.get().?, Vec3.init(radius, half_height, radius).mulScalar(1.01).mul(probe));
}

test "TestCollidePointVsConvexHull" {
    const half_box_size = Vec3.init(0.1, 0.2, 0.3);
    const offset = Vec3.init(10.0, 11.0, 12.0);

    var settings = ConvexHullShapeSettings.initDefault(allocator);
    defer settings.deinit();
    for (cube_vertices) |v|
        try settings.points.append(allocator, offset.add(v.mul(half_box_size)));
    var shape = try createShape(settings.asShapeSettings());
    defer shape.deinit();

    // Hits
    for (cube_and_zero_probes) |probe|
        try testHit(shape.get().?, offset.add(half_box_size.mulScalar(0.99).mul(probe)));

    // Misses
    for (cube_probes) |probe|
        try testMiss(shape.get().?, offset.add(half_box_size.mulScalar(1.01).mul(probe)));
}

test "TestCollidePointVsRotatedTranslated" {
    const translation = Vec3.init(10.0, 11.0, 12.0);
    const rotation = Quat.rotation(Vec3.init(1, 2, 3).normalized(), 0.3 * math.pi);
    const transform = Mat44.rotationTranslation(rotation, translation);

    const half_box_size = Vec3.init(0.1, 0.2, 0.3);
    var box = RefConst(Shape).init((try BoxShape.create(allocator, half_box_size, .{})).asShape());
    defer box.deinit();
    var settings = RotatedTranslatedShapeSettings.initPtr(allocator, translation, rotation, box.get());
    defer settings.deinit();
    var shape = try createShape(settings.asShapeSettings());
    defer shape.deinit();

    // Hits
    for (cube_and_zero_probes) |probe|
        try testHit(shape.get().?, transform.mulVec3(half_box_size.mulScalar(0.99).mul(probe)));

    // Misses
    for (cube_probes) |probe|
        try testMiss(shape.get().?, transform.mulVec3(half_box_size.mulScalar(1.01).mul(probe)));
}

test "TestCollidePointVsScaled" {
    const scale = Vec3.init(2.0, 3.0, -4.0);
    const half_box_size = Vec3.init(0.1, 0.2, 0.3);
    var box = RefConst(Shape).init((try BoxShape.create(allocator, half_box_size, .{})).asShape());
    defer box.deinit();
    var shape = RefConst(Shape).init((try ScaledShape.create(allocator, box.get().?, scale)).asShape());
    defer shape.deinit();

    // Hits
    for (cube_and_zero_probes) |probe|
        try testHit(shape.get().?, scale.mul(half_box_size.mulScalar(0.99).mul(probe)));

    // Misses
    for (cube_probes) |probe|
        try testMiss(shape.get().?, scale.mul(half_box_size.mulScalar(1.01).mul(probe)));
}

test "TestCollidePointVsOffsetCenterOfMass" {
    const offset = Vec3.init(10.0, 11.0, 12.0);
    const half_box_size = Vec3.init(0.1, 0.2, 0.3);
    var box = RefConst(Shape).init((try BoxShape.create(allocator, half_box_size, .{})).asShape());
    defer box.deinit();
    var settings = OffsetCenterOfMassShapeSettings.initPtr(allocator, offset, box.get());
    defer settings.deinit();
    var shape = try createShape(settings.asShapeSettings());
    defer shape.deinit();

    // Hits
    for (cube_and_zero_probes) |probe|
        try testHit(shape.get().?, half_box_size.mulScalar(0.99).mul(probe));

    // Misses
    for (cube_probes) |probe|
        try testMiss(shape.get().?, half_box_size.mulScalar(1.01).mul(probe));
}

/// The body of TestCollidePointVsStaticCompound and TestCollidePointVsMutableCompound (`Settings` is the compound
/// settings type)
fn testCollidePointVsCompound(comptime Settings: type) !void {
    const translation1 = Vec3.init(10.0, 11.0, 12.0);
    const rotation1 = Quat.rotation(Vec3.init(1, 2, 3).normalized(), 0.3 * math.pi);
    const transform1 = Mat44.rotationTranslation(rotation1, translation1);

    const translation2 = Vec3.init(-1.0, -2.0, -3.0);
    const rotation2 = Quat.rotation(Vec3.init(4, 5, 6).normalized(), 0.2 * math.pi);
    const transform2 = Mat44.rotationTranslation(rotation2, translation2);

    const half_box_size = Vec3.init(0.1, 0.2, 0.3);
    var box = RefConst(Shape).init((try BoxShape.create(allocator, half_box_size, .{})).asShape());
    defer box.deinit();

    var settings = Settings.init(allocator);
    defer settings.deinit();
    try settings.base.addShapePtr(translation1, rotation1, box.get(), .{});
    try settings.base.addShapePtr(translation2, rotation2, box.get(), .{});
    var shape = try createShape(settings.asShapeSettings());
    defer shape.deinit();

    // Hits
    for (cube_and_zero_probes) |probe| {
        const point = half_box_size.mulScalar(0.99).mul(probe);
        try testHit(shape.get().?, transform1.mulVec3(point));
        try testHit(shape.get().?, transform2.mulVec3(point));
    }

    // Misses
    for (cube_probes) |probe| {
        const point = half_box_size.mulScalar(1.01).mul(probe);
        try testMiss(shape.get().?, transform1.mulVec3(point));
        try testMiss(shape.get().?, transform2.mulVec3(point));
    }
}

test "TestCollidePointVsStaticCompound" {
    try testCollidePointVsCompound(StaticCompoundShapeSettings);
}

test "TestCollidePointVsMutableCompound" {
    try testCollidePointVsCompound(MutableCompoundShapeSettings);
}

test "TestCollidePointVsMesh" {
    // Face indices of a cube
    const indices = [_][3]u32{
        .{ 0, 1, 3 },
        .{ 0, 3, 2 },
        .{ 4, 7, 5 },
        .{ 4, 6, 7 },
        .{ 2, 3, 6 },
        .{ 3, 7, 6 },
        .{ 1, 0, 4 },
        .{ 1, 4, 5 },
        .{ 1, 7, 3 },
        .{ 1, 5, 7 },
        .{ 0, 2, 6 },
        .{ 0, 6, 4 },
    };

    const grid_size = 2;

    var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);
    const range = fw.UniformFloatDistribution.init(0.1, 0.3);

    // Create a grid of closed shapes
    var settings = MeshShapeSettings.initDefault(allocator);
    settings.asShapeSettings().setEmbedded();
    defer settings.deinit();
    const num_cubes = math.cubed(2 * grid_size + 1);
    try settings.triangle_vertices.ensureTotalCapacity(allocator, num_cubes * cube_vertices.len);
    try settings.indexed_triangles.ensureTotalCapacity(allocator, num_cubes * indices.len);
    var x: i32 = -grid_size;
    while (x <= grid_size) : (x += 1) {
        var y: i32 = -grid_size;
        while (y <= grid_size) : (y += 1) {
            var z: i32 = -grid_size;
            while (z <= grid_size) : (z += 1) {
                const center = Vec3.init(@floatFromInt(x), @floatFromInt(y), @floatFromInt(z));

                // Create vertices with randomness
                // (C++ leaves the evaluation order of constructor arguments unspecified, Zolt draws them left to right)
                const vtx: u32 = @intCast(settings.triangle_vertices.items.len);
                for (cube_vertices) |v| {
                    const rx = range.next(&random);
                    const ry = range.next(&random);
                    const rz = range.next(&random);
                    const vertex = center.add(v.mul(Vec3.init(rx, ry, rz)));
                    vertex.storeFloat3(settings.triangle_vertices.addOneAssumeCapacity());
                }

                // Flip inside out? (inside out shapes should act the same as normal shapes for CollidePoint)
                const flip = (y & 1) == 0;

                // Create face indices
                for (indices) |i|
                    settings.indexed_triangles.appendAssumeCapacity(IndexedTriangle.init(vtx + i[0], vtx + i[if (flip) 2 else 1], vtx + i[if (flip) 1 else 2], .{}));
            }
        }
    }

    // Get the shape (the body with a random position and rotation is not ported, see the header)
    var mesh_shape = try createShape(settings.asShapeSettings());
    defer mesh_shape.deinit();

    // Test points
    x = -grid_size;
    while (x <= grid_size) : (x += 1) {
        var y: i32 = -grid_size;
        while (y <= grid_size) : (y += 1) {
            var z: i32 = -grid_size;
            while (z <= grid_size) : (z += 1) {
                const center = Vec3.init(@floatFromInt(x), @floatFromInt(y), @floatFromInt(z));

                // The center point should hit
                try testHit(mesh_shape.get().?, center);
                // Not ported: sTestHit(narrow_phase, body_transform * center, mesh_body.GetID()) (Phase 5)

                // Points outside the hull should not hit
                for (cube_probes) |probe| {
                    const point = center.add(probe.mulScalar(0.4));
                    try testMiss(mesh_shape.get().?, point);
                    // Not ported: sTestMiss(narrow_phase, body_transform * point) (Phase 5)
                }
            }
        }
    }
}
