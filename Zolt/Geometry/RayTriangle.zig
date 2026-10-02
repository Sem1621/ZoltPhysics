//! Port of: Jolt/Geometry/RayTriangle.h
//! Status: complete

const std = @import("std");
const math = @import("../Math/Math.zig");
const UVec4 = @import("../Math/UVec4.zig").UVec4;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;

/// Intersect ray with triangle, returns closest point or FLT_MAX if no hit (branch less version)
/// Adapted from: http://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
pub fn rayTriangle(origin: Vec3, direction: Vec3, v0: Vec3, v1: Vec3, v2: Vec3) f32 {
    // Epsilon
    const epsilon = Vec3.replicate(1.0e-12);

    // Zero & one
    const zero = Vec3.zero();
    const one = Vec3.one();

    // Find vectors for two edges sharing v0
    const e1 = v1.sub(v0);
    const e2 = v2.sub(v0);

    // Begin calculating determinant - also used to calculate u parameter
    const p = direction.crossPrecise(e2);

    // if determinant is near zero, ray lies in plane of triangle
    var det = Vec3.replicate(e1.dot(p));

    // Check if determinant is near zero
    const det_near_zero = Vec3.less(det.abs(), epsilon);

    // When the determinant is near zero, set it to one to avoid dividing by zero
    det = Vec3.select(det, Vec3.one(), det_near_zero);

    // Calculate distance from v0 to ray origin
    const s = origin.sub(v0);

    // Calculate u parameter
    const u = Vec3.replicate(s.dot(p)).div(det);

    // Prepare to test v parameter
    const q = s.crossPrecise(e1);

    // Calculate v parameter
    const v = Vec3.replicate(direction.dot(q)).div(det);

    // Get intersection point
    const t = Vec3.replicate(e2.dot(q)).div(det);

    // Check if there is an intersection
    const no_intersection = UVec4.bitOr(
        UVec4.bitOr(
            UVec4.bitOr(
                det_near_zero,
                Vec3.less(u, zero),
            ),
            UVec4.bitOr(
                Vec3.less(v, zero),
                Vec3.greater(u.add(v), one),
            ),
        ),
        Vec3.less(t, zero),
    );

    // Select intersection point or FLT_MAX based on if there is an intersection or not
    return Vec3.select(t, Vec3.replicate(math.flt_max), no_intersection).getX();
}

/// Intersect ray with 4 triangles in SOA format, returns 4 vector of closest points or FLT_MAX if no hit (uses bit tricks to do less divisions)
pub fn rayTriangle4(origin: Vec3, direction: Vec3, v0x: Vec4, v0y: Vec4, v0z: Vec4, v1x: Vec4, v1y: Vec4, v1z: Vec4, v2x: Vec4, v2y: Vec4, v2z: Vec4) Vec4 {
    // Epsilon
    const epsilon = Vec4.replicate(1.0e-12);

    // Zero
    const zero = Vec4.zero();

    // Find vectors for two edges sharing v0
    const e1x = v1x.sub(v0x);
    const e1y = v1y.sub(v0y);
    const e1z = v1z.sub(v0z);
    const e2x = v2x.sub(v0x);
    const e2y = v2y.sub(v0y);
    const e2z = v2z.sub(v0z);

    // Get direction vector components
    const dx = direction.splatX();
    const dy = direction.splatY();
    const dz = direction.splatZ();

    // Begin calculating determinant - also used to calculate u parameter
    const px = Vec4.differenceOfProducts(dy, e2z, dz, e2y);
    const py = Vec4.differenceOfProducts(dz, e2x, dx, e2z);
    const pz = Vec4.differenceOfProducts(dx, e2y, dy, e2x);

    // if determinant is near zero, ray lies in plane of triangle
    var det = e1x.mul(px).add(e1y.mul(py)).add(e1z.mul(pz));

    // Get sign bit for determinant and make positive
    const det_sign = Vec4.bitAnd(det, UVec4.replicate(0x80000000).reinterpretAsFloat());
    det = Vec4.bitXor(det, det_sign);

    // Check which determinants are near zero
    const det_near_zero = Vec4.less(det, epsilon);

    // Set components of the determinant to 1 that are near zero to avoid dividing by zero
    det = Vec4.select(det, Vec4.one(), det_near_zero);

    // Calculate distance from v0 to ray origin
    const sx = origin.splatX().sub(v0x);
    const sy = origin.splatY().sub(v0y);
    const sz = origin.splatZ().sub(v0z);

    // Calculate u parameter and flip sign if determinant was negative
    const u = Vec4.bitXor(sx.mul(px).add(sy.mul(py)).add(sz.mul(pz)), det_sign);

    // Prepare to test v parameter
    const qx = Vec4.differenceOfProducts(sy, e1z, sz, e1y);
    const qy = Vec4.differenceOfProducts(sz, e1x, sx, e1z);
    const qz = Vec4.differenceOfProducts(sx, e1y, sy, e1x);

    // Calculate v parameter and flip sign if determinant was negative
    const v = Vec4.bitXor(dx.mul(qx).add(dy.mul(qy)).add(dz.mul(qz)), det_sign);

    // Get intersection point and flip sign if determinant was negative
    const t = Vec4.bitXor(e2x.mul(qx).add(e2y.mul(qy)).add(e2z.mul(qz)), det_sign);

    // Check if there is an intersection
    const no_intersection = UVec4.bitOr(
        UVec4.bitOr(
            UVec4.bitOr(
                det_near_zero,
                Vec4.less(u, zero),
            ),
            UVec4.bitOr(
                Vec4.less(v, zero),
                Vec4.greater(u.add(v), det),
            ),
        ),
        Vec4.less(t, zero),
    );

    // Select intersection point or FLT_MAX based on if there is an intersection or not
    return Vec4.select(t.div(det), Vec4.replicate(math.flt_max), no_intersection);
}

test "rayTriangle" {
    const v0 = Vec3.init(-1, -1, 0);
    const v1 = Vec3.init(1, -1, 0);
    const v2 = Vec3.init(0, 1, 0);

    // Hit from both sides
    try std.testing.expectEqual(@as(f32, 2), rayTriangle(Vec3.init(0, 0, 2), Vec3.init(0, 0, -1), v0, v1, v2));
    try std.testing.expectEqual(@as(f32, 0.5), rayTriangle(Vec3.init(0, 0, -2), Vec3.init(0, 0, 4), v0, v1, v2));

    // Miss: outside the triangle, pointing away and parallel
    try std.testing.expectEqual(math.flt_max, rayTriangle(Vec3.init(2, 0, 2), Vec3.init(0, 0, -1), v0, v1, v2));
    try std.testing.expectEqual(math.flt_max, rayTriangle(Vec3.init(0, 0, 2), Vec3.init(0, 0, 1), v0, v1, v2));
    try std.testing.expectEqual(math.flt_max, rayTriangle(Vec3.init(-5, 0, 0), Vec3.init(1, 0, 0), v0, v1, v2));
}

test "rayTriangle4" {
    // Triangle 0: hit from the front, 1: hit from the back (reversed winding), 2: miss, 3: degenerate
    const origin = Vec3.init(0, 0, 2);
    const direction = Vec3.init(0, 0, -1);
    const result = rayTriangle4(origin, direction, Vec4.init(-1, 1, 4, 0), Vec4.init(-1, -1, -1, 0), Vec4.init(0, -1, 0, 0), Vec4.init(1, -1, 6, 1), Vec4.init(-1, -1, -1, 1), Vec4.init(0, -1, 0, 0), Vec4.init(0, 0, 5, 2), Vec4.init(1, 1, 1, 2), Vec4.init(0, -1, 0, 0));
    try std.testing.expectEqual(@as(f32, 2), result.getX());
    try std.testing.expectEqual(@as(f32, 3), result.getY());
    try std.testing.expectEqual(math.flt_max, result.getZ());
    try std.testing.expectEqual(math.flt_max, result.getW());
}
