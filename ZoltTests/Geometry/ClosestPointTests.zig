//! Port of: UnitTests/Geometry/ClosestPointTests.cpp
//! Status: complete

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const ClosestPoint = zolt.ClosestPoint;
const Vec3 = zolt.Vec3;

/// Test closest point from point to triangle (a, b, c)
fn testClosestPointToTriangle(in_a: Vec3, in_b: Vec3, in_c: Vec3, point: Vec3, expected_closest_point: Vec3, expected_set: u32) !void {
    // Make triangle relative to point so we can get the closest point to the origin
    const a = in_a.sub(point);
    const b = in_b.sub(point);
    const c = in_c.sub(point);

    // Extract bits for A, B and C
    const expected_a = expected_set & 1;
    const expected_b = (expected_set & 2) >> 1;
    const expected_c = (expected_set & 4) >> 2;

    // Test all permutations of ABC
    var r = ClosestPoint.getClosestPointOnTriangle(a, b, c, .{});
    try fw.expectEqual(expected_set, r.set);
    try fw.checkApproxEqual(point.add(r.point), expected_closest_point, .{ .tolerance = 2.0e-5 });

    r = ClosestPoint.getClosestPointOnTriangle(a, c, b, .{});
    try fw.expectEqual((expected_b << 2) | (expected_c << 1) | expected_a, r.set);
    try fw.checkApproxEqual(point.add(r.point), expected_closest_point, .{ .tolerance = 2.0e-5 });

    r = ClosestPoint.getClosestPointOnTriangle(b, a, c, .{});
    try fw.expectEqual((expected_c << 2) | (expected_a << 1) | expected_b, r.set);
    try fw.checkApproxEqual(point.add(r.point), expected_closest_point, .{ .tolerance = 2.0e-5 });

    r = ClosestPoint.getClosestPointOnTriangle(b, c, a, .{});
    try fw.expectEqual((expected_a << 2) | (expected_c << 1) | expected_b, r.set);
    try fw.checkApproxEqual(point.add(r.point), expected_closest_point, .{ .tolerance = 2.0e-5 });

    r = ClosestPoint.getClosestPointOnTriangle(c, a, b, .{});
    try fw.expectEqual((expected_b << 2) | (expected_a << 1) | expected_c, r.set);
    try fw.checkApproxEqual(point.add(r.point), expected_closest_point, .{ .tolerance = 2.0e-5 });

    r = ClosestPoint.getClosestPointOnTriangle(c, b, a, .{});
    try fw.expectEqual((expected_a << 2) | (expected_b << 1) | expected_c, r.set);
    try fw.checkApproxEqual(point.add(r.point), expected_closest_point, .{ .tolerance = 2.0e-5 });
}

test "TestLongTriangle" {
    const a = Vec3.init(100, 1, 0);
    const b = Vec3.init(100, 1, 1);
    const c = Vec3.init(-100, 1, 0);

    // Test interior
    try testClosestPointToTriangle(a, b, c, Vec3.init(0, 0, 0.1), Vec3.init(0, 1, 0.1), 0b0111);

    // Edge AB
    try testClosestPointToTriangle(a, b, c, Vec3.init(101, 0, 0.5), Vec3.init(100, 1, 0.5), 0b0011);

    // Edge AC
    try testClosestPointToTriangle(a, b, c, Vec3.init(0, 0, -0.1), Vec3.init(0, 1, 0), 0b0101);

    // Edge BC
    const point_bc = Vec3.init(0, 0, 1);
    const bc = c.sub(b);
    const closest_bc = b.add(bc.mulScalar(point_bc.sub(b).dot(bc) / bc.lengthSq()));
    try testClosestPointToTriangle(a, b, c, point_bc, closest_bc, 0b0110);

    // Vertex A
    try testClosestPointToTriangle(a, b, c, Vec3.init(101, 0, -1), a, 0b0001);

    // Vertex B
    try testClosestPointToTriangle(a, b, c, Vec3.init(101, 0, 2), b, 0b0010);

    // Vertex C
    try testClosestPointToTriangle(a, b, c, Vec3.init(-101, 0, 0), c, 0b0100);
}

test "TestNearColinearTriangle" {
    // A very long triangle that is nearly colinear
    const a = Vec3.init(99.9999847, 0.946687222, 99.9999847);
    const b = Vec3.init(-100.010002, 0.977360725, -100.010002);
    const c = Vec3.init(-100.000137, 0.977310658, -100.000137);

    // Closest point is on edge AC
    const ac = c.sub(a);
    const expected_closest = a.add(ac.mulScalar(-a.dot(ac) / ac.lengthSq()));

    try testClosestPointToTriangle(a, b, c, Vec3.zero(), expected_closest, 0b0101);
}

test "TestSmallTriangleWithPlaneGoingThroughOrigin" {
    // A small but non-degenerate triangle whose plane almost goes through the origin
    const a = Vec3.init(-0.132395342, -0.294095188, -0.164812326);
    const b = Vec3.init(-0.126054004, -0.283950001, -0.159065604);
    const c = Vec3.init(-0.154956535, -0.284792334, -0.160523415);

    const bary = ClosestPoint.getBaryCentricCoordinatesTriangle(a, b, c);
    const u = bary.u;
    const v = bary.v;
    const w = bary.w;

    // Closest point should be close to origin
    const p = a.mulScalar(u).add(b.mulScalar(v)).add(c.mulScalar(w));
    try fw.checkApproxEqual(p, Vec3.zero(), .{});

    // Closest point should be outside triangle
    try fw.expect((u < 0.0 or v > 0.0 or w < 0.0));
}
