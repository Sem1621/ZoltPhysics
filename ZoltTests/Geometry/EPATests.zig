//! Port of: UnitTests/Geometry/EPATests.cpp
//! Status: complete

const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const checkApproxEqual = fw.checkApproxEqual;
const UnitTestRandom = fw.UnitTestRandom;

const math = zolt.math;
const AABox = zolt.AABox;
const EPAPenetrationDepth = zolt.EPAPenetrationDepth;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const Sphere = zolt.Sphere;
const TransformedConvexObject = zolt.TransformedConvexObject;
const Vec3 = zolt.Vec3;

// Enable to trace accuracy of EPA algorithm (EPA_TESTS_TRACE, not ported)

/// Helper function to return the angle between two vectors in degrees
fn angleBetweenVectors(v1: Vec3, v2: Vec3) f32 {
    const dot = v1.dot(v2);
    const len = v1.length() * v2.length();
    return math.radiansToDegrees(zolt.trigonometry.acos(dot / len));
}

/// Test box versus sphere and compare analytical solution with that of the EPA algorithm
/// @return If a collision was detected
fn collideBoxSphere(matrix: Mat44, box: AABox, sphere: Sphere) !bool {
    const transformed_box = TransformedConvexObject(AABox).init(matrix, &box);
    const transformed_sphere = TransformedConvexObject(Sphere).init(matrix, &sphere);

    // Use EPA algorithm. Don't use convex radius to avoid EPA being skipped because the inner hulls are not touching.
    var epa: EPAPenetrationDepth = .{};
    var v1 = Vec3.axisX();
    var pa1: Vec3 = undefined;
    var pb1: Vec3 = undefined;
    const intersect1 = epa.getPenetrationDepth(&transformed_box, &transformed_box, 0.0, &transformed_sphere, &transformed_sphere, 0.0, 1.0e-2, math.flt_epsilon, &v1, &pa1, &pb1);

    // Analytical solution
    var pa2 = box.getClosestPoint(sphere.getCenter());
    var v2 = sphere.getCenter().sub(pa2);
    const intersect2 = v2.lengthSq() <= math.square(sphere.getRadius());

    try expect(intersect1 == intersect2);
    if (intersect1 and intersect2) {
        // Analytical solution of contact on B
        var pb2 = sphere.getCenter().sub(v2.normalizedOr(Vec3.zero()).mulScalar(sphere.getRadius()));

        // Transform analytical solution
        v2 = matrix.multiply3x3(v2);
        pa2 = matrix.mulVec3(pa2);
        pb2 = matrix.mulVec3(pb2);

        // Check angle between v1 and v2
        const angle = angleBetweenVectors(v1, v2);
        try expect(angle < 0.1);

        // Check delta between contact on A
        const dpa = pa2.sub(pa1);
        try expect(dpa.length() < 9.0e-4);

        // Check delta between contact on B
        const dpb = pb2.sub(pb1);
        try expect(dpb.length() < 9.0e-4);
    }

    return intersect1;
}

/// Test multiple boxes against spheres and transform both with matrix
fn collideBoxesWithSpheres(matrix: Mat44) !void {
    {
        // Sphere just missing face of box
        const box = AABox.init(Vec3.init(-2, -3, -4), Vec3.init(2, 3, 4));
        const sphere = Sphere.init(Vec3.init(4, 0, 0), 1.99);
        try expect(!try collideBoxSphere(matrix, box, sphere));
    }

    {
        // Sphere just touching face of box
        const box = AABox.init(Vec3.init(-2, -3, -4), Vec3.init(2, 3, 4));
        const sphere = Sphere.init(Vec3.init(4, 0, 0), 2.01);
        try expect(try collideBoxSphere(matrix, box, sphere));
    }

    {
        // Sphere deeply penetrating box on face
        const box = AABox.init(Vec3.init(-2, -3, -4), Vec3.init(2, 3, 4));
        const sphere = Sphere.init(Vec3.init(3, 0, 0), 2);
        try expect(try collideBoxSphere(matrix, box, sphere));
    }

    {
        // Sphere just missing box on edge
        const box = AABox.init(Vec3.init(1, 1, -2), Vec3.init(2, 2, 2));
        const sphere = Sphere.init(Vec3.init(4, 4, 0), @sqrt(@as(f32, 8.0)) - 0.01);
        try expect(!try collideBoxSphere(matrix, box, sphere));
    }

    {
        // Sphere just penetrating box on edge
        const box = AABox.init(Vec3.init(1, 1, -2), Vec3.init(2, 2, 2));
        const sphere = Sphere.init(Vec3.init(4, 4, 0), @sqrt(@as(f32, 8.0)) + 0.01);
        try expect(try collideBoxSphere(matrix, box, sphere));
    }

    {
        // Sphere just missing box on vertex
        const box = AABox.init(Vec3.init(1, 1, 1), Vec3.init(2, 2, 2));
        const sphere = Sphere.init(Vec3.init(4, 4, 4), @sqrt(@as(f32, 12.0)) - 0.01);
        try expect(!try collideBoxSphere(matrix, box, sphere));
    }

    {
        // Sphere just penetrating box on vertex
        const box = AABox.init(Vec3.init(1, 1, 1), Vec3.init(2, 2, 2));
        const sphere = Sphere.init(Vec3.init(4, 4, 4), @sqrt(@as(f32, 12.0)) + 0.01);
        try expect(try collideBoxSphere(matrix, box, sphere));
    }
}

test "TestEPASphereBox" {
    // Test identity transform
    try collideBoxesWithSpheres(Mat44.identity());

    // Test some random rotations/translations
    // (C++ leaves the evaluation order of function arguments unspecified, Zolt draws them left to right)
    var random = UnitTestRandom.init(UnitTestRandom.default_seed);
    for (0..10) |_| {
        const rotation = Quat.random(&random);
        const translation = Vec3.random(&random);
        try collideBoxesWithSpheres(Mat44.rotationTranslation(rotation, translation));
    }
}

test "TestEPASphereSphereOverlapping" {
    // Worst case: Two spheres exactly overlapping
    // In this case the Minkowski sum is a sphere which means the EPA algorithm will be building a convex hull of a full sphere and run out of triangles resulting in a pretty bad approximation
    const sphere = Sphere.init(Vec3.init(1, 2, 3), 2.0);
    var epa: EPAPenetrationDepth = .{};
    var v = Vec3.axisX();
    var pa: Vec3 = undefined;
    var pb: Vec3 = undefined;
    try expect(epa.getPenetrationDepth(&sphere, &sphere, 0.0, &sphere, &sphere, 0.0, 1.0e-4, math.flt_epsilon, &v, &pa, &pb));
    const delta_a = pa.sub(sphere.getCenter()).length() - sphere.getRadius();
    try expect(@abs(delta_a) < 0.07);
    const delta_b = pb.sub(sphere.getCenter()).length() - sphere.getRadius();
    try expect(@abs(delta_b) < 0.07);
    const delta_penetration = pa.sub(pb).length() - 2.0 * sphere.getRadius();
    try expect(@abs(delta_penetration) < 0.14);
    const angle = angleBetweenVectors(v, pa.sub(pb));
    try expect(angle < 0.02);
}

test "TestEPASphereSphereNearOverlapping" {
    // Near worst case: Two spheres almost exactly overlapping
    // Still limited by amount of triangles in the hull but more precise
    const sphere1 = Sphere.init(Vec3.init(1, 2, 3), 2.0);
    const sphere2 = Sphere.init(Vec3.init(1.1, 2, 3), 1.8);
    var epa: EPAPenetrationDepth = .{};
    var v = Vec3.axisX();
    var pa: Vec3 = undefined;
    var pb: Vec3 = undefined;
    try expect(epa.getPenetrationDepth(&sphere1, &sphere1, 0.0, &sphere2, &sphere2, 0.0, 1.0e-4, math.flt_epsilon, &v, &pa, &pb));
    const delta_a = pa.sub(sphere1.getCenter()).length() - sphere1.getRadius();
    try expect(@abs(delta_a) < 0.05);
    const delta_b = pb.sub(sphere2.getCenter()).length() - sphere2.getRadius();
    try expect(@abs(delta_b) < 0.05);
    const delta_penetration = pa.sub(pb).length() - (sphere1.getRadius() + sphere2.getRadius() - sphere1.getCenter().sub(sphere2.getCenter()).length());
    try expect(@abs(delta_penetration) < 0.06);
    const angle = angleBetweenVectors(v, pa.sub(pb));
    try expect(angle < 0.02);
}

test "TestEPACastSphereSphereMiss" {
    const sphere = Sphere.init(Vec3.init(0, 0, 0), 1.0);
    var epa: EPAPenetrationDepth = .{};
    var lambda: f32 = 1.0 + math.flt_epsilon;
    const invalid = Vec3.init(-999, -999, -999);
    var pa = invalid;
    var pb = invalid;
    var normal = invalid;
    try expect(!epa.castShape(Mat44.translation(Vec3.init(-10, 2.1, 0)), Vec3.init(20, 0, 0), 1.0e-4, 1.0e-4, &sphere, &sphere, 0.0, 0.0, true, &lambda, &pa, &pb, &normal));
    try expect(lambda == 1.0 + math.flt_epsilon); // Check input values didn't change
    try expect(pa.eql(invalid));
    try expect(pb.eql(invalid));
    try expect(normal.eql(invalid));
}

test "TestEPACastSphereSphereInitialOverlap" {
    const sphere = Sphere.init(Vec3.init(0, 0, 0), 1.0);
    var epa: EPAPenetrationDepth = .{};
    var lambda: f32 = 1.0 + math.flt_epsilon;
    const invalid = Vec3.init(-999, -999, -999);
    var pa = invalid;
    var pb = invalid;
    var normal = invalid;
    try expect(epa.castShape(Mat44.translation(Vec3.init(-1, 0, 0)), Vec3.init(10, 0, 0), 1.0e-4, 1.0e-4, &sphere, &sphere, 0.0, 0.0, true, &lambda, &pa, &pb, &normal));
    try expect(lambda == 0.0);
    try checkApproxEqual(pa, Vec3.zero(), .{ .tolerance = 5.0e-3 });
    try checkApproxEqual(pb, Vec3.init(-1, 0, 0), .{ .tolerance = 5.0e-3 });
    try checkApproxEqual(normal.normalizedOr(Vec3.zero()), Vec3.init(1, 0, 0), .{ .tolerance = 1.0e-2 });
}

test "TestEPACastSphereSphereHit" {
    const sphere = Sphere.init(Vec3.init(0, 0, 0), 1.0);
    var epa: EPAPenetrationDepth = .{};
    var lambda: f32 = 1.0 + math.flt_epsilon;
    const invalid = Vec3.init(-999, -999, -999);
    var pa = invalid;
    var pb = invalid;
    var normal = invalid;
    try expect(epa.castShape(Mat44.translation(Vec3.init(-10, 0, 0)), Vec3.init(20, 0, 0), 1.0e-4, 1.0e-4, &sphere, &sphere, 0.0, 0.0, true, &lambda, &pa, &pb, &normal));
    try checkApproxEqual(lambda, 8.0 / 20.0, .{});
    try checkApproxEqual(pa, Vec3.init(-1, 0, 0), .{});
    try checkApproxEqual(pb, Vec3.init(-1, 0, 0), .{});
    try checkApproxEqual(normal.normalizedOr(Vec3.zero()), Vec3.init(1, 0, 0), .{});
}
