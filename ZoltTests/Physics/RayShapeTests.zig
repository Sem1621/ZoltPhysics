//! Port of: UnitTests/Physics/RayShapeTests.cpp
//! Status: partial
//! Missing: the PhysicsSystem part of TestRayHelper (TestSystemRay, TestSystemRayMultiHitIgnoreBackFace,
//!   TestSystemRayMultiHitWithBackFace: need PhysicsSystem / NarrowPhaseQuery, Phase 5), TestRayMiss (only used by
//!   shapes that are not ported yet), TestConvexHullShapeRay, TestCapsuleShapeRay, TestTaperedCapsuleShapeRay,
//!   TestCylinderShapeRay, TestTaperedCylinderShapeRay, TestScaledShapeRay, TestStaticCompoundShapeRay,
//!   TestMutableCompoundShapeRay (need their shapes, Phase 4 Wave A / B)

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const BoxShape = zolt.BoxShape;
const CastRayCollector = zolt.CastRayCollector;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const Ref = zolt.Ref;
const Shape = zolt.Shape;
const SphereShape = zolt.SphereShape;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const allocator = std.testing.allocator;
const flt_max = math.flt_max;

/// The kind of test that TestRayHelper runs (the TestFunction lambdas that test directly against a shape)
const TestKind = enum {
    shape_ray,
    shape_ray_multi_hit_ignore_back_face,
    shape_ray_multi_hit_with_back_face,
    shape_ray_multi_hit_ignore_back_face_non_solid,
    shape_ray_multi_hit_with_back_face_non_solid,
};

// Function that does the actual ray cast test, expected_fraction1/2 should be FLT_MAX if no hit expected
fn testFunction(kind: TestKind, shape: *const Shape, in_ray: RayCast, expected_fraction1_in: f32, expected_fraction2_in: f32) !void {
    var expected_fraction1 = expected_fraction1_in;
    var expected_fraction2 = expected_fraction2_in;

    // CastRay works relative to center of mass, so transform the ray
    var ray = in_ray;
    ray.origin = ray.origin.sub(shape.getCenterOfMass());

    switch (kind) {
        // Test function that directly tests against a shape
        .shape_ray => {
            var hit: RayCastResult = .{};
            if (expected_fraction1 != flt_max) {
                try fw.expect(shape.castRay(ray, .{}, &hit));
                try fw.checkApproxEqual(hit.fraction, expected_fraction1, .{ .tolerance = 1.0e-5 });
            } else {
                try fw.expect(!shape.castRay(ray, .{}, &hit));
            }
        },

        // Test function that directly tests against a shape allowing multiple hits but no back facing hits, treating convex objects as solids
        .shape_ray_multi_hit_ignore_back_face => {
            // Ray cast settings
            var settings: RayCastSettings = .{};
            settings.setBackFaceMode(.ignore_back_faces);
            settings.treat_convex_as_solid = true;

            var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer collector.deinit();
            shape.castRayCollector(ray, &settings, .{}, &collector.base, &.{});
            try collector.checkError();

            if (expected_fraction1 != flt_max) {
                try fw.expect(collector.hits.items.len == 1);
                try fw.checkApproxEqual(collector.hits.items[0].fraction, expected_fraction1, .{ .tolerance = 1.0e-5 });
            } else {
                try fw.expect(collector.hits.items.len == 0);
            }
        },

        // Test function that directly tests against a shape allowing multiple hits and back facing hits, treating convex objects as solids
        .shape_ray_multi_hit_with_back_face => {
            // Ray cast settings
            var settings: RayCastSettings = .{};
            settings.setBackFaceMode(.collide_with_back_faces);
            settings.treat_convex_as_solid = true;

            var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer collector.deinit();
            shape.castRayCollector(ray, &settings, .{}, &collector.base, &.{});
            try collector.checkError();

            if (expected_fraction1 != flt_max) {
                try fw.expect(collector.hits.items.len >= 1);
                try fw.checkApproxEqual(collector.hits.items[0].fraction, expected_fraction1, .{ .tolerance = 1.0e-5 });
            } else {
                std.debug.assert(expected_fraction2 == flt_max);
                try fw.expect(collector.hits.items.len == 0);
            }

            if (expected_fraction2 != flt_max) {
                try fw.expect(collector.hits.items.len >= 2);
                try fw.checkApproxEqual(collector.hits.items[1].fraction, expected_fraction2, .{ .tolerance = 1.0e-5 });
            } else {
                try fw.expect(collector.hits.items.len < 2);
            }
        },

        // Test function that directly tests against a shape allowing multiple hits but no back facing hits, treating convex object as non-solids
        .shape_ray_multi_hit_ignore_back_face_non_solid => {
            // Ray cast settings
            var settings: RayCastSettings = .{};
            settings.setBackFaceMode(.ignore_back_faces);
            settings.treat_convex_as_solid = false;

            var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer collector.deinit();
            shape.castRayCollector(ray, &settings, .{}, &collector.base, &.{});
            try collector.checkError();

            // A fraction of 0 means that the ray starts in solid, we treat this as a non-hit
            if (expected_fraction1 != 0.0 and expected_fraction1 != flt_max) {
                try fw.expect(collector.hits.items.len == 1);
                try fw.checkApproxEqual(collector.hits.items[0].fraction, expected_fraction1, .{ .tolerance = 1.0e-5 });
            } else {
                try fw.expect(collector.hits.items.len == 0);
            }
        },

        // Test function that directly tests against a shape allowing multiple hits and back facing hits, treating convex object as non-solids
        .shape_ray_multi_hit_with_back_face_non_solid => {
            // Ray cast settings
            var settings: RayCastSettings = .{};
            settings.setBackFaceMode(.collide_with_back_faces);
            settings.treat_convex_as_solid = false;

            var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer collector.deinit();
            shape.castRayCollector(ray, &settings, .{}, &collector.base, &.{});
            try collector.checkError();

            // A fraction of 0 means that the ray starts in solid, we treat this as a non-hit
            if (expected_fraction1 == 0.0) {
                expected_fraction1 = expected_fraction2;
                expected_fraction2 = flt_max;
            }

            if (expected_fraction1 != flt_max) {
                try fw.expect(collector.hits.items.len >= 1);
                try fw.checkApproxEqual(collector.hits.items[0].fraction, expected_fraction1, .{ .tolerance = 1.0e-5 });
            } else {
                std.debug.assert(expected_fraction2 == flt_max);
                try fw.expect(collector.hits.items.len == 0);
            }

            if (expected_fraction2 != flt_max) {
                try fw.expect(collector.hits.items.len >= 2);
                try fw.checkApproxEqual(collector.hits.items[1].fraction, expected_fraction2, .{ .tolerance = 1.0e-5 });
            } else {
                try fw.expect(collector.hits.items.len < 2);
            }
        },
    }
}

// Test ray against shape with lines going through hit_a and hit_b (which should be surface positions of the shape)
fn testRayHelperInternal(hit_a: Vec3, hit_b: Vec3, kind: TestKind, shape: *const Shape) !void {
    // Determine points before and after the surface on both sides (i1 / i2 are inner1 / inner2: i1 is a Zig type)
    const delta = hit_b.sub(hit_a);
    const l1 = hit_a.sub(delta.mulScalar(2.0));
    const l2 = hit_a.sub(delta.mulScalar(0.1));
    const inner1 = hit_a.add(delta.mulScalar(0.1));
    const inner2 = hit_b.sub(delta.mulScalar(0.1));
    const r1 = hit_b.add(delta.mulScalar(0.1));
    const r2 = hit_b.add(delta.mulScalar(2.0));

    // -O---->-|--------|--------
    try testFunction(kind, shape, .init(l1, l2.sub(l1)), flt_max, flt_max);

    // -----O>-|--------|--------
    try testFunction(kind, shape, .init(l2, Vec3.zero()), flt_max, flt_max);

    // ------O-|->------|--------
    try testFunction(kind, shape, .init(l2, inner1.sub(l2)), 0.5, flt_max);

    // ------O-|--------|->------
    try testFunction(kind, shape, .init(l2, r1.sub(l2)), @as(f32, 0.1) / 1.2, @as(f32, 1.1) / 1.2);

    // --------|-----O>-|--------
    try testFunction(kind, shape, .init(inner2, Vec3.zero()), 0.0, flt_max);

    // --------|------O-|->------
    try testFunction(kind, shape, .init(inner2, r1.sub(inner2)), 0.0, 0.5);

    // --------|--------|-O---->-
    try testFunction(kind, shape, .init(r1, r2.sub(l1)), flt_max, flt_max);
}

fn testRayHelper(shape: *const Shape, hit_a: Vec3, hit_b: Vec3) !void {
    for ([_]TestKind{ .shape_ray, .shape_ray_multi_hit_ignore_back_face, .shape_ray_multi_hit_with_back_face, .shape_ray_multi_hit_ignore_back_face_non_solid, .shape_ray_multi_hit_with_back_face_non_solid }) |kind| {
        // Test normal ray
        try testRayHelperInternal(hit_a, hit_b, kind, shape);

        // Test inverse ray
        try testRayHelperInternal(hit_b, hit_a, kind, shape);
    }

    // Not ported yet: inserting the shape into a PhysicsSystem and testing the rays through its NarrowPhaseQuery
    // (TestSystemRay, TestSystemRayMultiHitIgnoreBackFace, TestSystemRayMultiHitWithBackFace), Phase 5
}

test "TestBoxShapeRay" {
    // Create box shape
    var box = BoxShape.init(allocator, Vec3.init(2, 3, 4), .{}); // Allocate on the stack to test embedded refcounted structs
    box.asShape().setEmbedded();
    defer box.asShapeMut().deinit();
    var shape = Ref(Shape).init(box.asShapeMut()); // Add a reference to see if we don't hit free() of a stack allocated struct
    defer shape.deinit();

    try testRayHelper(shape.get().?, Vec3.init(-2, 0, 0), Vec3.init(2, 0, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, -3, 0), Vec3.init(0, 3, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, 0, -4), Vec3.init(0, 0, 4));
}

test "TestSphereShapeRay" {
    // Create sphere shape
    var shape = Ref(Shape).init((try SphereShape.create(allocator, 2, .{})).asShapeMut());
    defer shape.deinit();

    try testRayHelper(shape.get().?, Vec3.init(-2, 0, 0), Vec3.init(2, 0, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, -2, 0), Vec3.init(0, 2, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, 0, -2), Vec3.init(0, 0, 2));
}
