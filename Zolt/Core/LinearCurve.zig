//! Port of: Jolt/Core/LinearCurve.h, Jolt/Core/LinearCurve.cpp
//! Status: partial
//! Missing: JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL (ObjectStream, Phase 8)
//!
//! `mPoints` is a `std.ArrayList(Point)`: the functions that allocate take the allocator, `deinit` frees the points
//! and `clone` replaces the C++ copy constructor.

const std = @import("std");
const Allocator = std.mem.Allocator;
const quickSort = @import("QuickSort.zig").quickSort;
const StreamIn = @import("StreamIn.zig").StreamIn;
const StreamOut = @import("StreamOut.zig").StreamOut;

// A set of points (x, y) that form a linear curve
pub const LinearCurve = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, LinearCurve)

    /// A point on the curve
    pub const Point = extern struct {
        // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, Point)

        x: f32 = 0.0,
        y: f32 = 0.0,
    };

    /// The points on the curve, should be sorted ascending by x
    pub const Points = std.ArrayList(Point);

    /// The points on the curve, should be sorted ascending by x
    points: Points = .empty,

    /// Free the points
    pub fn deinit(self: *LinearCurve, allocator: Allocator) void {
        self.points.deinit(allocator);
    }

    /// Copy of this curve (copy constructor)
    pub fn clone(self: *const LinearCurve, allocator: Allocator) Allocator.Error!LinearCurve {
        return .{ .points = try self.points.clone(allocator) };
    }

    /// Remove all points
    pub fn clear(self: *LinearCurve) void {
        self.points.clearRetainingCapacity();
    }

    /// Reserve memory for inNumPoints points
    pub fn reserve(self: *LinearCurve, allocator: Allocator, num_points: u32) Allocator.Error!void {
        try self.points.ensureTotalCapacityPrecise(allocator, num_points);
    }

    /// Add a point to the curve. Points must be inserted in ascending X or Sort() needs to be called when all points have been added.
    /// @param inX X value
    /// @param inY Y value
    pub fn addPoint(self: *LinearCurve, allocator: Allocator, x: f32, y: f32) Allocator.Error!void {
        try self.points.append(allocator, .{ .x = x, .y = y });
    }

    /// Sort the points on X ascending
    pub fn sort(self: *LinearCurve) void {
        quickSort(Point, self.points.items, {}, lessThanX);
    }

    fn lessThanX(_: void, lhs: Point, rhs: Point) bool {
        return lhs.x < rhs.x;
    }

    /// Get the lowest X value
    pub fn getMinX(self: *const LinearCurve) f32 {
        return if (self.points.items.len == 0) 0.0 else self.points.items[0].x;
    }

    /// Get the highest X value
    pub fn getMaxX(self: *const LinearCurve) f32 {
        return if (self.points.items.len == 0) 0.0 else self.points.getLast().x;
    }

    /// Sample value on the curve
    /// @param inX X value to sample at
    /// @return Interpolated Y value
    pub fn getValue(self: *const LinearCurve, x: f32) f32 {
        const points = self.points.items;
        if (points.len == 0)
            return 0.0;

        // Iterators i1 and i2 become the points p1 and p2 (`i2` is a reserved name in Zig)
        const index2 = lowerBound(points, x);

        if (index2 == 0)
            return points[0].y
        else if (index2 == points.len)
            return points[points.len - 1].y;

        const p1 = points[index2 - 1];
        const p2 = points[index2];
        return p1.y + (x - p1.x) * (p2.y - p1.y) / (p2.x - p1.x);
    }

    /// std::lower_bound(points.begin(), points.end(), x, [](const Point &inPoint, float inValue) { return inPoint.mX < inValue; }):
    /// the index of the first point that is not less than x. Uses the same bisection as the standard libraries, so that
    /// the result is also the same for points that are not sorted.
    fn lowerBound(points: []const Point, x: f32) usize {
        var first: usize = 0;
        var len = points.len;
        while (len != 0) {
            const half = len >> 1;
            const middle = first + half;
            if (points[middle].x < x) {
                first = middle + 1;
                len -= half + 1;
            } else {
                len = half;
            }
        }
        return first;
    }

    /// Saves the state of this object in binary form to inStream.
    pub fn saveBinaryState(self: *const LinearCurve, stream: StreamOut) void {
        stream.writeArray(Point, self.points.items);
    }

    /// Restore the state of this object from inStream.
    pub fn restoreBinaryState(self: *LinearCurve, allocator: Allocator, stream: StreamIn) Allocator.Error!void {
        try stream.readArray(Point, allocator, &self.points);
    }
};

test "LinearCurve save / restore and min / max" {
    const allocator = std.testing.allocator;
    const StreamWrapper = @import("StreamWrapper.zig");

    var curve: LinearCurve = .{};
    defer curve.deinit(allocator);
    try std.testing.expectEqual(@as(f32, 0.0), curve.getMinX());
    try std.testing.expectEqual(@as(f32, 0.0), curve.getMaxX());
    try curve.reserve(allocator, 3);
    try curve.addPoint(allocator, 3.0, 30.0);
    try curve.addPoint(allocator, 1.0, 10.0);
    try curve.addPoint(allocator, 2.0, 20.0);
    curve.sort();
    try std.testing.expectEqual(@as(f32, 1.0), curve.getMinX());
    try std.testing.expectEqual(@as(f32, 3.0), curve.getMaxX());

    // Save: number of points followed by the points
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    curve.saveBinaryState(out_wrapper.streamOut());
    const expected = std.mem.toBytes(@as(u32, 3)) ++ std.mem.toBytes([6]f32{ 1, 10, 2, 20, 3, 30 });
    try std.testing.expectEqualSlices(u8, &expected, out.written());

    // Restore
    var copy = try curve.clone(allocator);
    defer copy.deinit(allocator);
    copy.clear();
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
    try copy.restoreBinaryState(allocator, in_wrapper.streamIn());
    try std.testing.expectEqualSlices(LinearCurve.Point, curve.points.items, copy.points.items);
    try std.testing.expectEqual(@as(f32, 25.0), copy.getValue(2.5));
}
