//! Port of: UnitTests/Geometry/EllipseTest.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const checkApproxEqual = fw.checkApproxEqual;
const Ellipse = zolt.Ellipse;
const Float2 = zolt.Float2;

test "TestEllipseIsInside" {
    const e = Ellipse.init(1.0, 2.0);

    try expect(e.isInside(Float2.init(0.1, 0.1)));

    try expect(!e.isInside(Float2.init(2.0, 0.0)));
}

test "TestEllipseClosestPoint" {
    const e = Ellipse.init(1.0, 2.0);

    var c = e.getClosestPoint(Float2.init(2.0, 0.0));
    try expect(c.eql(Float2.init(1.0, 0.0)));

    c = e.getClosestPoint(Float2.init(-2.0, 0.0));
    try expect(c.eql(Float2.init(-1.0, 0.0)));

    c = e.getClosestPoint(Float2.init(0.0, 4.0));
    try expect(c.eql(Float2.init(0.0, 2.0)));

    c = e.getClosestPoint(Float2.init(0.0, -4.0));
    try expect(c.eql(Float2.init(0.0, -2.0)));

    const e2 = Ellipse.init(2.0, 2.0);

    c = e2.getClosestPoint(Float2.init(4.0, 4.0));
    try checkApproxEqual(c, Float2.init(@sqrt(@as(f32, 2.0)), @sqrt(@as(f32, 2.0))), .{});
}
