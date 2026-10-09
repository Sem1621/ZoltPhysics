//! Port of: UnitTests/Physics/CollidePointTests.cpp
//! Status: partial
//! Missing: TestCollidePointVsCapsule, TestCollidePointVsTaperedCapsule, TestCollidePointVsCylinder,
//!   TestCollidePointVsConvexHull, TestCollidePointVsRotatedTranslated, TestCollidePointVsScaled,
//!   TestCollidePointVsOffsetCenterOfMass, TestCollidePointVsStaticCompound, TestCollidePointVsMutableCompound (need
//!   their shapes, Phase 4 Wave A / B), TestCollidePointVsMesh (needs MeshShape and PhysicsTestContext, Phase 5), the
//!   NarrowPhaseQuery versions of sTestHit / sTestMiss (Phase 5)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const BoxShape = zolt.BoxShape;
const CollidePointCollector = zolt.CollidePointCollector;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const SphereShape = zolt.SphereShape;
const Vec3 = zolt.Vec3;

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

fn testHit(shape: *const Shape, position: Vec3) !void {
    var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer collector.deinit();
    shape.collidePoint(position.sub(shape.getCenterOfMass()), .{}, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 1);
}

fn testMiss(shape: *const Shape, position: Vec3) !void {
    var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer collector.deinit();
    shape.collidePoint(position.sub(shape.getCenterOfMass()), .{}, &collector.base, &.{});
    try collector.checkError();
    try fw.expect(collector.hits.items.len == 0);
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
