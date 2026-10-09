//! Port of: UnitTests/Physics/CollideShapeTests.cpp
//! Status: partial
//! Missing: TestCollideShapeSphere, TestCollideShapeSphereVsBox (need PhysicsTestContext, Phase 5),
//!   TestCollideShapeLongCapsuleVsEmbeddedBox, TestCollideShapeSmallTriangleVsLargeBox,
//!   TestCollideParallelTriangleVsCapsule, TestCollideParallelTriangleVsCapsule2, TestCollideParallelTriangleVsCapsule3,
//!   TestCollideParallelTriangleVsCylinder, BoxVsConvexHullNoConvexRadius, TestTriangleVsBoxLargeSeparationDistance,
//!   TestCollideTriangleVsTriangle (need CapsuleShape, CylinderShape, TriangleShape, ConvexHullShape and the triangle
//!   collision algorithms, Phase 4 Wave A / B)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const BoxShape = zolt.BoxShape;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollisionDispatch = zolt.CollisionDispatch;
const Mat44 = zolt.Mat44;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const SphereShape = zolt.SphereShape;
const Vec3 = zolt.Vec3;

const allocator = std.testing.allocator;

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
