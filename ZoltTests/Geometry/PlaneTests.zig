//! Port of: UnitTests/Geometry/PlaneTests.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const checkApproxEqual = fw.checkApproxEqual;
const math = zolt.math;
const Mat44 = zolt.Mat44;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const Vec3 = zolt.Vec3;

test "TestPlaneSignedDistance" {
    const p = Plane.fromPointAndNormal(Vec3.init(0, 2, 0), Vec3.init(0, 1, 0));
    try expect(p.signedDistance(Vec3.init(5, 7, 0)) == 5.0);
    try expect(p.signedDistance(Vec3.init(5, -3, 0)) == -5.0);
}

test "TestPlaneGetTransformed" {
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.init(1.0, 2.0, 3.0).normalized(), 0.1 * math.pi), Vec3.init(5.0, -7.0, 9.0));

    const point = Vec3.init(11.0, 13.0, 15.0);
    const normal = Vec3.init(-3.0, 5.0, -7.0).normalized();

    const p1 = Plane.fromPointAndNormal(point, normal).getTransformed(transform);
    const p2 = Plane.fromPointAndNormal(transform.mulVec3(point), transform.multiply3x3(normal));

    try checkApproxEqual(p1.getNormal(), p2.getNormal(), .{});
    try checkApproxEqual(p1.getConstant(), p2.getConstant(), .{ .tolerance = 2.0e-6 });
}

test "TestPlaneIntersectPlanes" {
    const p1 = Plane.fromPointAndNormal(Vec3.init(0, 2, 0), Vec3.init(0, 1, 0));
    const p2 = Plane.fromPointAndNormal(Vec3.init(3, 0, 0), Vec3.init(1, 0, 0));
    const p3 = Plane.fromPointAndNormal(Vec3.init(0, 0, 4), Vec3.init(0, 0, 1));

    {
        const point = Plane.intersectPlanes(p1, p2, p3);
        try expect(point != null);
        try expect(point.?.eql(Vec3.init(3, 2, 4)));
    }

    {
        const p4 = Plane.fromPointAndNormal(Vec3.init(0, 3, 0), Vec3.init(0, 1, 0));
        const point = Plane.intersectPlanes(p1, p2, p4);
        try expect(point == null);
    }
}
