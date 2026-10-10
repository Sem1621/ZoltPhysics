//! Port of: UnitTests/Physics/RayShapeTests.cpp
//! Status: partial
//! Missing: the PhysicsSystem part of TestRayHelper (TestSystemRay, TestSystemRayMultiHitIgnoreBackFace,
//!   TestSystemRayMultiHitWithBackFace: insert the shape into a PhysicsSystem and cast through its NarrowPhaseQuery;
//!   need PhysicsSystem / NarrowPhaseQuery, Phase 5), in every TEST_CASE that uses TestRayHelper

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const BoxShape = zolt.BoxShape;
const CapsuleShape = zolt.CapsuleShape;
const CastRayCollector = zolt.CastRayCollector;
const ConvexHullShapeSettings = zolt.ConvexHullShapeSettings;
const CylinderShape = zolt.CylinderShape;
const MutableCompoundShapeSettings = zolt.MutableCompoundShapeSettings;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const ScaledShape = zolt.ScaledShape;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const SphereShape = zolt.SphereShape;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const TaperedCapsuleShapeSettings = zolt.TaperedCapsuleShapeSettings;
const TaperedCylinderShapeSettings = zolt.TaperedCylinderShapeSettings;
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

    // Not ported: inserting the shape into a PhysicsSystem and testing the rays through its NarrowPhaseQuery
    // (TestSystemRay, TestSystemRayMultiHitIgnoreBackFace, TestSystemRayMultiHitWithBackFace: need PhysicsSystem /
    // NarrowPhaseQuery, Phase 5)
}

/// Helper function to check that a ray misses a shape
fn testRayMiss(shape: *const Shape, origin: Vec3, direction: Vec3) !void {
    var hit: RayCastResult = .{};
    try fw.expect(!shape.castRay(.init(origin.sub(shape.getCenterOfMass()), direction), .{}, &hit));
}

/// `settings.Create().Get()`: the shape of a newly created result (the caller releases the reference)
fn createShape(settings: *ShapeSettings) !RefConst(Shape) {
    var result = try settings.createShape(allocator);
    defer result.deinit();
    return RefConst(Shape).init(result.getPtr());
}

/// Convex hull shape of a box (off center so the center of mass is not zero)
const off_center_box = [_]Vec3{
    Vec3.init(-2, -4, -6),
    Vec3.init(-2, -4, 7),
    Vec3.init(-2, 5, -6),
    Vec3.init(-2, 5, 7),
    Vec3.init(3, -4, -6),
    Vec3.init(3, -4, 7),
    Vec3.init(3, 5, -6),
    Vec3.init(3, 5, 7),
};

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

test "TestConvexHullShapeRay" {
    // Create convex hull shape of a box (off center so the center of mass is not zero)
    var settings = try ConvexHullShapeSettings.init(allocator, &off_center_box, .{});
    defer settings.deinit();
    var shape = try createShape(settings.asShapeSettings());
    defer shape.deinit();

    try testRayHelper(shape.get().?, Vec3.init(-2, 0, 0), Vec3.init(3, 0, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, -4, 0), Vec3.init(0, 5, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, 0, -6), Vec3.init(0, 0, 7));

    try testRayMiss(shape.get().?, Vec3.init(-3, -5, 0), Vec3.init(0, 1, 0));
    try testRayMiss(shape.get().?, Vec3.init(-3, 0, 0), Vec3.init(0, 1, 0));
    try testRayMiss(shape.get().?, Vec3.init(-3, 6, 0), Vec3.init(0, 1, 0));
}

test "TestCapsuleShapeRay" {
    // Create capsule shape
    var shape = Ref(Shape).init((try CapsuleShape.create(allocator, 4, 2, .{})).asShapeMut());
    defer shape.deinit();

    try testRayHelper(shape.get().?, Vec3.init(-2, 0, 0), Vec3.init(2, 0, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, -6, 0), Vec3.init(0, 6, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, 0, -2), Vec3.init(0, 0, 2));
}

test "TestTaperedCapsuleShapeRay" {
    // Create tapered capsule shape
    var settings = TaperedCapsuleShapeSettings.init(allocator, 3, 4, 2, .{});
    defer settings.deinit();
    var shape = try createShape(settings.asShapeSettings());
    defer shape.deinit();

    try testRayHelper(shape.get().?, Vec3.init(0, 7, 0), Vec3.init(0, -5, 0)); // Top to bottom
    try testRayHelper(shape.get().?, Vec3.init(-4, 3, 0), Vec3.init(4, 3, 0)); // Top sphere
    try testRayHelper(shape.get().?, Vec3.init(0, 3, -4), Vec3.init(0, 3, 4)); // Top sphere
}

test "TestCylinderShapeRay" {
    // Create cylinder shape
    var shape = Ref(Shape).init((try CylinderShape.create(allocator, 4, 2, .{})).asShapeMut());
    defer shape.deinit();

    try testRayHelper(shape.get().?, Vec3.init(-2, 0, 0), Vec3.init(2, 0, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, -4, 0), Vec3.init(0, 4, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, 0, -2), Vec3.init(0, 0, 2));
}

test "TestTaperedCylinderShapeRay" {
    // Create tapered cylinder shape
    var settings = TaperedCylinderShapeSettings.init(allocator, 4, 1, 3, .{});
    defer settings.deinit();
    var shape = try createShape(settings.asShapeSettings());
    defer shape.deinit();

    // Ray through origin
    try testRayHelper(shape.get().?, Vec3.init(-2, 0, 0), Vec3.init(2, 0, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, -4, 0), Vec3.init(0, 4, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, 0, -2), Vec3.init(0, 0, 2));

    // Ray halfway to the top
    try testRayHelper(shape.get().?, Vec3.init(-1.5, 2, 0), Vec3.init(1.5, 2, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, 2, -1.5), Vec3.init(0, 2, 1.5));

    // Ray halfway to the bottom
    try testRayHelper(shape.get().?, Vec3.init(-2.5, -2, 0), Vec3.init(2.5, -2, 0));
    try testRayHelper(shape.get().?, Vec3.init(0, -2, -2.5), Vec3.init(0, -2, 2.5));
}

test "TestScaledShapeRay" {
    // Create convex hull shape of a box (off center so the center of mass is not zero)
    var settings = try ConvexHullShapeSettings.init(allocator, &off_center_box, .{});
    defer settings.deinit();
    var hull = try createShape(settings.asShapeSettings());
    defer hull.deinit();

    // Scale the hull
    var shape1 = Ref(Shape).init((try ScaledShape.create(allocator, hull.get().?, Vec3.init(2, 3, 4))).asShapeMut());
    defer shape1.deinit();

    try testRayHelper(shape1.get().?, Vec3.init(-4, 0, 0), Vec3.init(6, 0, 0));
    try testRayHelper(shape1.get().?, Vec3.init(0, -12, 0), Vec3.init(0, 15, 0));
    try testRayHelper(shape1.get().?, Vec3.init(0, 0, -24), Vec3.init(0, 0, 28));

    // Scale the hull (and flip it inside out)
    var shape2 = Ref(Shape).init((try ScaledShape.create(allocator, hull.get().?, Vec3.init(-2, 3, 4))).asShapeMut());
    defer shape2.deinit();

    try testRayHelper(shape2.get().?, Vec3.init(-6, 0, 0), Vec3.init(4, 0, 0));
    try testRayHelper(shape2.get().?, Vec3.init(0, -12, 0), Vec3.init(0, 15, 0));
    try testRayHelper(shape2.get().?, Vec3.init(0, 0, -24), Vec3.init(0, 0, 28));
}

/// The body of TestStaticCompoundShapeRay and TestMutableCompoundShapeRay (`Settings` is the compound settings type)
fn testCompoundShapeRay(comptime Settings: type) !void {
    // Create convex hull shape of a box (off center so the center of mass is not zero)
    var hull = Ref(ShapeSettings).init((try ConvexHullShapeSettings.create(allocator, &off_center_box, .{})).asShapeSettings());
    defer hull.deinit();

    // Translate/rotate the shape through a compound (off center to force center of mass not zero)
    const shape1_position = Vec3.init(10, 20, 30);
    const shape1_rotation = Quat.rotation(Vec3.axisX(), 0.1 * math.pi).mul(Quat.rotation(Vec3.axisY(), 0.2 * math.pi));
    const shape2_position = Vec3.init(40, 50, 60);
    const shape2_rotation = Quat.rotation(Vec3.axisZ(), 0.3 * math.pi);

    var compound_settings = Settings.init(allocator);
    defer compound_settings.deinit();
    try compound_settings.base.addShape(shape1_position, shape1_rotation, hull.get(), .{}); // Shape 1
    try compound_settings.base.addShape(shape2_position, shape2_rotation, hull.get(), .{}); // Shape 2
    var compound_ref = try createShape(compound_settings.asShapeSettings());
    defer compound_ref.deinit();
    const compound = compound_ref.get().?;

    // Hitting shape 1
    try testRayHelper(compound, shape1_position.add(shape1_rotation.mulVec3(Vec3.init(-2, 0, 0))), shape1_position.add(shape1_rotation.mulVec3(Vec3.init(3, 0, 0))));
    try testRayHelper(compound, shape1_position.add(shape1_rotation.mulVec3(Vec3.init(0, -4, 0))), shape1_position.add(shape1_rotation.mulVec3(Vec3.init(0, 5, 0))));
    try testRayHelper(compound, shape1_position.add(shape1_rotation.mulVec3(Vec3.init(0, 0, -6))), shape1_position.add(shape1_rotation.mulVec3(Vec3.init(0, 0, 7))));

    // Hitting shape 2
    try testRayHelper(compound, shape2_position.add(shape2_rotation.mulVec3(Vec3.init(-2, 0, 0))), shape2_position.add(shape2_rotation.mulVec3(Vec3.init(3, 0, 0))));
    try testRayHelper(compound, shape2_position.add(shape2_rotation.mulVec3(Vec3.init(0, -4, 0))), shape2_position.add(shape2_rotation.mulVec3(Vec3.init(0, 5, 0))));
    try testRayHelper(compound, shape2_position.add(shape2_rotation.mulVec3(Vec3.init(0, 0, -6))), shape2_position.add(shape2_rotation.mulVec3(Vec3.init(0, 0, 7))));
}

test "TestStaticCompoundShapeRay" {
    try testCompoundShapeRay(StaticCompoundShapeSettings);
}

test "TestMutableCompoundShapeRay" {
    try testCompoundShapeRay(MutableCompoundShapeSettings);
}
