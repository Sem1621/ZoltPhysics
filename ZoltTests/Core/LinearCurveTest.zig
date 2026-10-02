//! Port of: UnitTests/Core/LinearCurveTest.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const LinearCurve = zolt.LinearCurve;

test "Test0PointCurve" {
    var curve: LinearCurve = .{};

    try fw.expect(curve.getValue(1.0) == 0.0);
}

test "Test1PointCurve" {
    const allocator = std.testing.allocator;
    var curve: LinearCurve = .{};
    defer curve.deinit(allocator);
    try curve.addPoint(allocator, 1.0, 20.0);

    try fw.expect(curve.getValue(0.9) == 20.0);
    try fw.expect(curve.getValue(1.0) == 20.0);
    try fw.expect(curve.getValue(1.1) == 20.0);
}

test "Test2PointCurve" {
    const allocator = std.testing.allocator;
    var curve: LinearCurve = .{};
    defer curve.deinit(allocator);
    try curve.addPoint(allocator, -1.0, 40.0);
    try curve.addPoint(allocator, -3.0, 20.0);
    curve.sort();

    try fw.checkApproxEqual(curve.getValue(-3.1), 20.0, .{});
    try fw.checkApproxEqual(curve.getValue(-3.0), 20.0, .{});
    try fw.checkApproxEqual(curve.getValue(-2.0), 30.0, .{});
    try fw.checkApproxEqual(curve.getValue(-1.0), 40.0, .{});
    try fw.checkApproxEqual(curve.getValue(-0.9), 40.0, .{});
}

test "Test3PointCurve" {
    const allocator = std.testing.allocator;
    var curve: LinearCurve = .{};
    defer curve.deinit(allocator);
    try curve.addPoint(allocator, 1.0, 20.0);
    try curve.addPoint(allocator, 5.0, 60.0);
    try curve.addPoint(allocator, 3.0, 40.0);
    curve.sort();

    try fw.checkApproxEqual(curve.getValue(0.9), 20.0, .{});
    try fw.checkApproxEqual(curve.getValue(1.0), 20.0, .{});
    try fw.checkApproxEqual(curve.getValue(2.0), 30.0, .{});
    try fw.checkApproxEqual(curve.getValue(3.0), 40.0, .{});
    try fw.checkApproxEqual(curve.getValue(4.0), 50.0, .{});
    try fw.checkApproxEqual(curve.getValue(5.0), 60.0, .{});
    try fw.checkApproxEqual(curve.getValue(5.1), 60.0, .{});
}
