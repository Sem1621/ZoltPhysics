//! Port of: UnitTests/Geometry/GJKTests.cpp
//! Status: partial
//! Missing: TestGJKRay*Shape (need Phase 4 convex shapes)

const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const checkApproxEqual = fw.checkApproxEqual;
const UnitTestRandom = fw.UnitTestRandom;
const UniformFloatDistribution = fw.UniformFloatDistribution;

const math = zolt.math;
const AABox = zolt.AABox;
const GJKClosestPoint = zolt.GJKClosestPoint;
const RayInvDirection = zolt.RayInvDirection;
const Sphere = zolt.Sphere;
const TriangleConvexSupport = zolt.TriangleConvexSupport;
const Vec3 = zolt.Vec3;

test "TestGJKIntersectSphere" {
    var gjk: GJKClosestPoint = .{};

    // Sphere 1 is centered around the origin
    const s1 = Sphere.init(Vec3.zero(), 1.0);

    // Shere 2 is far away from s1
    const c2 = Vec3.init(10.0, 10.0, 10.0);
    const s2 = Sphere.init(c2, 1.0);

    // Sphere 3 is exactly 2 away from s1
    const l: f32 = 2.0 / @sqrt(@as(f32, 3.0));
    const c3 = Vec3.init(l, l, l);
    const s3 = Sphere.init(c3, 1.0);

    {
        // Test sphere s1 and s2, they should not collide
        var v = Vec3.zero();
        try expect(!gjk.intersects(&s1, &s2, 1.0e-4, &v));
    }

    {
        // Test sphere s1 and s3, they should touch exactly
        var v = Vec3.zero();
        try expect(gjk.intersects(&s1, &s3, 1.0e-4, &v));
    }

    {
        // Test sphere s1 and s2, they should not collide, verify their closest points
        var pa: Vec3 = undefined;
        var pb: Vec3 = undefined;
        var v = Vec3.zero();
        const d = @sqrt(gjk.getClosestPoints(&s1, &s2, 1.0e-4, math.large_float, &v, &pa, &pb));
        try checkApproxEqual(c2.length() - 2.0, d, .{ .tolerance = 1.0e-4 });
        try checkApproxEqual(c2.normalized(), pa, .{ .tolerance = 1.0e-4 });
        try checkApproxEqual(c2.sub(c2.normalized()), pb, .{ .tolerance = 1.0e-4 });
    }

    {
        // Test sphere s1 and s3, they should touch exactly, verify their closest points
        var pa: Vec3 = undefined;
        var pb: Vec3 = undefined;
        var v = Vec3.zero();
        const d = @sqrt(gjk.getClosestPoints(&s1, &s3, 1.0e-4, math.large_float, &v, &pa, &pb));
        try checkApproxEqual(@as(f32, 0.0), d, .{ .tolerance = 1.0e-4 });
        try checkApproxEqual(c2.normalized(), pa, .{ .tolerance = 1.0e-4 });
        try checkApproxEqual(c2.normalized(), pb, .{ .tolerance = 1.0e-4 });
    }
}

fn testIntersect(
    comptime A: type,
    comptime B: type,
    createFuncA: fn (*UnitTestRandom) A,
    createFuncB: fn (*UnitTestRandom) B,
    compareFunc: fn (a: A, b: B, is_intersecting: bool, tolerance: f32) bool,
) !void {
    var random = UnitTestRandom.init(12345);

    const count = 10000;

    var hits: u32 = 0;

    var gjk: GJKClosestPoint = .{};

    for (0..count) |_| {
        const shape1 = createFuncA(&random);
        const shape2 = createFuncB(&random);

        // Use GJK to test for intersection
        var v = Vec3.zero();
        const tolerance: f32 = 1.0e-4;
        const result_gjk = gjk.intersects(&shape1, &shape2, tolerance, &v);

        // Compare with reference function and increase tolerance a bit to account for floating point imprecision
        try expect(compareFunc(shape1, shape2, result_gjk, 2.0 * tolerance));

        if (result_gjk)
            hits += 1;
    }

    // Check that there were enough hits so that the test is representative
    const hit_rate = 100.0 * @as(f32, @floatFromInt(hits)) / @as(f32, count);
    try expect(hit_rate > 30.0);
    try expect(hit_rate < 70.0);
}

// (C++ leaves the evaluation order of constructor / function arguments unspecified, Zolt draws them left to right)
fn sphereCreator(random: *UnitTestRandom) Sphere {
    const pos = UniformFloatDistribution.init(-2.0, 2.0);
    const rad = UniformFloatDistribution.init(0.5, 2.0);
    const x = pos.next(random);
    const y = pos.next(random);
    const z = pos.next(random);
    return Sphere.init(Vec3.init(x, y, z), rad.next(random));
}

fn boxCreator(random: *UnitTestRandom) AABox {
    const pos = UniformFloatDistribution.init(-2.0, 2.0);
    const x1 = pos.next(random);
    const y1 = pos.next(random);
    const z1 = pos.next(random);
    const p1 = Vec3.init(x1, y1, z1);
    const x2 = pos.next(random);
    const y2 = pos.next(random);
    const z2 = pos.next(random);
    const p2 = Vec3.init(x2, y2, z2);
    return AABox.fromTwoPoints(p1, p2);
}

test "TestGJKSphereVsSphereIntersect" {
    try testIntersect(Sphere, Sphere, sphereCreator, sphereCreator, struct {
        fn compare(sphere_a: Sphere, sphere_b: Sphere, is_intersecting: bool, tolerance: f32) bool {
            // Test without and with tolerance if the results are equal
            return sphere_a.overlaps(sphere_b) == is_intersecting or
                Sphere.init(sphere_a.getCenter(), sphere_a.getRadius() + tolerance).overlaps(sphere_b) == is_intersecting;
        }
    }.compare);
}

test "TestGJKSphereVsBoxIntersect" {
    try testIntersect(Sphere, AABox, sphereCreator, boxCreator, struct {
        fn compare(sphere_a: Sphere, box_b: AABox, is_intersecting: bool, tolerance: f32) bool {
            // Test without and with tolerance if the results are equal
            return sphere_a.overlapsAABox(box_b) == is_intersecting or
                Sphere.init(sphere_a.getCenter(), sphere_a.getRadius() + tolerance).overlapsAABox(box_b) == is_intersecting;
        }
    }.compare);
}

/// `a` is a pointer to the convex object to cast against, `context` is passed to compareFunc
fn testRay(a: anytype, context: anytype, compareFunc: fn (@TypeOf(context), Vec3, Vec3) f32) !void {
    var random = UnitTestRandom.init(12345);
    const random_scale = UniformFloatDistribution.init(-2.0, 2.0);

    const count = 1000;

    for (0..count) |_| {
        // (C++ leaves the evaluation order of constructor arguments unspecified, Zolt draws them left to right)
        const from_x = random_scale.next(&random);
        const from_y = random_scale.next(&random);
        const from_z = random_scale.next(&random);
        const from = Vec3.init(from_x, from_y, from_z);
        const to_x = random_scale.next(&random);
        const to_y = random_scale.next(&random);
        const to_z = random_scale.next(&random);
        const to = Vec3.init(to_x, to_y, to_z);
        const direction = to.sub(from);

        // Use GJK to cast a ray
        var fraction1: f32 = 1.0 + math.flt_epsilon;
        var gjk: GJKClosestPoint = .{};
        if (!gjk.castRay(from, direction, 1.0e-4, a, &fraction1))
            fraction1 = math.flt_max;

        // Use the comparison function
        var fraction2 = compareFunc(context, from, direction);

        // The comparison functions work with infinite rays, so a fraction > 1 means a miss
        if (fraction2 > 1.0)
            fraction2 = math.flt_max;

        try checkApproxEqual(fraction1, fraction2, .{ .tolerance = 0.01 });
    }
}

test "TestGJKRaySphere" {
    const sphere = Sphere.init(Vec3.init(0.1, 0.2, 0.3), 1.1);
    try testRay(&sphere, sphere, struct {
        fn compare(s: Sphere, ray_origin: Vec3, ray_direction: Vec3) f32 {
            return zolt.raySphere(ray_origin, ray_direction, s.getCenter(), s.getRadius());
        }
    }.compare);
}

// Not ported: TestGJKRaySphereShape, needs SphereShape (Phase 4 convex shapes)

test "TestGJKRayBox" {
    const box = AABox.init(Vec3.init(-0.9, -1.0, -1.1), Vec3.init(0.8, 0.9, 1.0));
    try testRay(&box, box, struct {
        fn compare(b: AABox, ray_origin: Vec3, ray_direction: Vec3) f32 {
            const fraction = zolt.rayAABox(ray_origin, RayInvDirection.init(ray_direction), b.min, b.max);
            return math.max(fraction, 0.0);
        }
    }.compare);
}

// Not ported: TestGJKRayBoxShape, needs BoxShape (Phase 4 convex shapes)

// Not ported: TestGJKRayCapsuleShape, needs CapsuleShape (Phase 4 convex shapes)

// Not ported: TestGJKRayCylinderShape, needs CylinderShape (Phase 4 convex shapes)

test "TestGJKRayTriangle" {
    const triangle = TriangleConvexSupport.init(Vec3.init(0.1, 0.9, 0.3), Vec3.init(-0.9, -0.5, 0.2), Vec3.init(0.7, -0.3, -0.1));
    try testRay(&triangle, triangle, struct {
        fn compare(t: TriangleConvexSupport, ray_origin: Vec3, ray_direction: Vec3) f32 {
            return zolt.rayTriangle(ray_origin, ray_direction, t.v1, t.v2, t.v3);
        }
    }.compare);
}
