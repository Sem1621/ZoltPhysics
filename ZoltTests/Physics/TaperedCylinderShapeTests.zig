//! Port of: UnitTests/Physics/TaperedCylinderShapeTests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const CollidePointCollector = zolt.CollidePointCollector;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TaperedCylinderShape = zolt.TaperedCylinderShape;
const TaperedCylinderShapeSettings = zolt.TaperedCylinderShapeSettings;
const Vec3 = zolt.Vec3;
const math = zolt.math;

const allocator = std.testing.allocator;

/// `settings.Create().Get()` as a reference
fn createShape(settings: *TaperedCylinderShapeSettings) !RefConst(Shape) {
    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    return .init(result.getPtr().?);
}

test "TestMassAndInertia" {
    const density: f32 = 3.0;
    const radius: f32 = 5.0;
    const height: f32 = 7.0;

    var settings1 = TaperedCylinderShapeSettings.init(allocator, 0.5 * height, radius, 0.0, .{ .convex_radius = 0.0 });
    defer settings1.deinit();
    settings1.base.setDensity(density);

    var settings2 = TaperedCylinderShapeSettings.init(allocator, 0.5 * height, 0.0, radius, .{ .convex_radius = 0.0 });
    defer settings2.deinit();
    settings2.base.setDensity(density);

    var cylinder1_ref = try createShape(&settings1); // StaticCast<TaperedCylinderShape>: a RefConst(Shape), cast on use
    defer cylinder1_ref.deinit();
    var cylinder2_ref = try createShape(&settings2);
    defer cylinder2_ref.deinit();
    const cylinder1 = cylinder1_ref.get().?.cast(TaperedCylinderShape);
    const cylinder2 = cylinder2_ref.get().?.cast(TaperedCylinderShape);

    // Check accessors
    try fw.expect(cylinder1.getTopRadius() == radius);
    try fw.expect(cylinder1.getBottomRadius() == 0.0);
    try fw.expect(cylinder1.getConvexRadius() == 0.0);
    try fw.checkApproxEqual(cylinder1.getHalfHeight(), 0.5 * height, .{});

    const m1 = cylinder1.asShape().getMassProperties();
    const m2 = cylinder2.asShape().getMassProperties();

    // Mass/inertia is the same for both shapes because they are mirrored versions (inertia is calculated from COM)
    try fw.checkApproxEqual(m1.mass, m2.mass, .{});
    try fw.checkApproxEqual(m1.inertia, m2.inertia, .{});

    // Center of mass for a cone is at 1/4 h (if cone runs from -h/2 to h/2)
    // See: https://www.miniphysics.com/uy1-centre-of-mass-of-a-cone.html
    const expected_com1 = Vec3.init(0, height / 4.0, 0);
    const expected_com2 = expected_com1.negate();
    try fw.checkApproxEqual(cylinder1.asShape().getCenterOfMass(), expected_com1, .{});
    try fw.checkApproxEqual(cylinder2.asShape().getCenterOfMass(), expected_com2, .{});

    // Mass of cone
    const expected_mass = density * math.pi * math.square(radius) * height / 3.0;
    try fw.checkApproxEqual(expected_mass, m1.mass, .{});

    // Inertia of cone (according to https://en.wikipedia.org/wiki/List_of_moments_of_inertia)
    const expected_inertia_xx = expected_mass * (3.0 / 20.0 * math.square(radius) + 3.0 / 80.0 * math.square(height));
    const expected_inertia_yy = expected_mass * (3.0 / 10.0 * math.square(radius));
    try fw.checkApproxEqual(expected_inertia_xx, m1.inertia.get(0, 0), .{ .tolerance = 1.0e-3 });
    try fw.checkApproxEqual(expected_inertia_yy, m1.inertia.get(1, 1), .{ .tolerance = 1.0e-3 });
    try fw.checkApproxEqual(expected_inertia_xx, m1.inertia.get(2, 2), .{ .tolerance = 1.0e-3 });
}

test "TestCollidePoint" {
    const top_radius: f32 = 3.0;
    const bottom_radius: f32 = 5.0;
    const half_height: f32 = 3.5;

    var settings = TaperedCylinderShapeSettings.init(allocator, half_height, top_radius, bottom_radius, .{});
    defer settings.deinit();
    var shape_ref = try createShape(&settings);
    defer shape_ref.deinit();
    const shape = shape_ref.get().?;

    const helpers = struct {
        fn testInside(s: *const Shape, point: Vec3) !void {
            var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
            defer collector.deinit();
            s.collidePoint(point.sub(s.getCenterOfMass()), .{}, &collector.base, &.{});
            try collector.checkError();
            try fw.expect(collector.hits.items.len == 1);
        }

        fn testOutside(s: *const Shape, point: Vec3) !void {
            var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
            defer collector.deinit();
            s.collidePoint(point.sub(s.getCenterOfMass()), .{}, &collector.base, &.{});
            try collector.checkError();
            try fw.expect(collector.hits.items.len == 0);
        }
    };
    const test_inside = helpers.testInside;
    const test_outside = helpers.testOutside;

    const epsilon: f32 = 1.0e-3;

    try test_inside(shape, Vec3.zero());

    // Top plane
    try test_inside(shape, Vec3.init(0, half_height - epsilon, 0));
    try test_outside(shape, Vec3.init(0, half_height + epsilon, 0));

    // Bottom plane
    try test_inside(shape, Vec3.init(0, -half_height + epsilon, 0));
    try test_outside(shape, Vec3.init(0, -half_height - epsilon, 0));

    // COM plane
    try test_inside(shape, Vec3.init(0.5 * (top_radius + bottom_radius) - epsilon, 0, 0));
    try test_outside(shape, Vec3.init(0.5 * (top_radius + bottom_radius) + epsilon, 0, 0));

    // At quarter h above COM plane
    const h = 0.5 * half_height;
    const r = bottom_radius + (top_radius - bottom_radius) * (h + half_height) / (2.0 * half_height);
    try test_inside(shape, Vec3.init(0, h, r - epsilon));
    try test_outside(shape, Vec3.init(0, h, r + epsilon));
}
