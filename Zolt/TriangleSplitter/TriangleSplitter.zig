//! Port of: Jolt/TriangleSplitter/TriangleSplitter.h, Jolt/TriangleSplitter/TriangleSplitter.cpp
//! Status: complete
//!
//! `TriangleSplitter` is an abstract class with data in the base class, so it uses polymorphism pattern A of the
//! porting guide: the base struct holds `vtable` plus the shared data, the splitters (`TriangleSplitterBinning`,
//! `TriangleSplitterMean`) embed it as their first field `base` and downcast with `@fieldParentPtr`. Code that
//! works with any splitter (AABBTreeBuilder) takes a `*TriangleSplitter`.
//!
//! - The constructor allocates, so it takes an allocator: `init(allocator, vertices, triangles, vtable)` (only called
//!   by the derived splitters) and the virtual destructor is `deinit(allocator)`.
//! - `Split(inTriangles, outLeft, outRight) -> bool` is `split(triangles) ?SplitResult`: null when no split could be
//!   made (Jolt's out parameters are unspecified in that case), otherwise the left and right ranges.
//! - `GetStats(Stats &outStats)` only writes the splitter name, so it keeps the out pointer: `getStats(&stats)`.
//! - The vertex and triangle lists are borrowed (Jolt stores references), they must outlive the splitter.

const std = @import("std");
const Core = @import("../Core/Core.zig");
const Float3 = @import("../Math/Float3.zig").Float3;
const IndexedTriangle = @import("../Geometry/IndexedTriangle.zig").IndexedTriangle;

/// A class that splits a triangle list into two parts for building a tree
pub const TriangleSplitter = struct {
    /// Virtual functions, implemented by the derived splitters
    pub const VTable = struct {
        /// Virtual destructor, frees the memory of the derived splitter and the base
        deinit: *const fn (self: *TriangleSplitter, allocator: std.mem.Allocator) void,

        /// Get stats of splitter
        getStats: *const fn (self: *const TriangleSplitter, stats: *Stats) void,

        /// Split triangles into two groups left and right, returns null if no split could be made
        split: *const fn (self: *TriangleSplitter, triangles: Range) ?SplitResult,
    };

    pub const Stats = struct {
        splitter_name: ?[]const u8 = null,
        leaf_size: i32 = 0,
    };

    /// Helper struct to indicate triangle range before and after the split
    pub const Range = struct {
        /// Start and end index (end = 1 beyond end)
        begin: u32,
        end: u32,

        /// Constructor (Jolt's default constructor leaves the members uninitialized: `var r: Range = undefined;`)
        pub fn init(begin: u32, end: u32) Range {
            return .{ .begin = begin, .end = end };
        }

        /// Get number of triangles in range
        pub fn count(self: Range) u32 {
            return self.end - self.begin;
        }
    };

    /// Result of a successful split: the ranges of the left and right subparts (Jolt's outLeft / outRight)
    pub const SplitResult = struct {
        left: Range,
        right: Range,
    };

    vtable: *const VTable,

    /// Vertices of the indexed triangles
    vertices: []const Float3,

    /// Unsorted triangles
    triangles: []const IndexedTriangle,

    /// Unsorted centroids of triangles
    centroids: std.ArrayList(Float3) = .empty,

    /// Indices to sort triangles
    sorted_triangle_idx: std.ArrayList(u32) = .empty,

    /// Constructor, called by the derived splitters with their vtable
    pub fn init(allocator: std.mem.Allocator, vertices: []const Float3, triangles: []const IndexedTriangle, vtable: *const VTable) std.mem.Allocator.Error!TriangleSplitter {
        var self: TriangleSplitter = .{ .vtable = vtable, .vertices = vertices, .triangles = triangles };
        errdefer self.deinitBase(allocator);

        try self.sorted_triangle_idx.resize(allocator, triangles.len);
        try self.centroids.resize(allocator, triangles.len + 1); // Add 1 so we can load with Vec3::sLoadFloat3Unsafe

        for (0..triangles.len) |t| {
            // Initially triangles start unsorted
            self.sorted_triangle_idx.items[t] = @intCast(t);

            // Calculate centroid
            triangles[t].getCentroid(vertices).storeFloat3(&self.centroids.items[t]);
        }

        // Make sure Vec3::sLoatFloat3Unsafe doesn't read uninitialized data
        self.centroids.items[self.centroids.items.len - 1] = Float3.init(0, 0, 0);

        return self;
    }

    /// Virtual destructor: frees the splitter, whatever its type
    pub fn deinit(self: *TriangleSplitter, allocator: std.mem.Allocator) void {
        self.vtable.deinit(self, allocator);
    }

    /// Free the memory owned by the base class, called by the destructors of the derived splitters
    pub fn deinitBase(self: *TriangleSplitter, allocator: std.mem.Allocator) void {
        self.centroids.deinit(allocator);
        self.sorted_triangle_idx.deinit(allocator);
    }

    /// Get stats of splitter
    pub fn getStats(self: *const TriangleSplitter, stats: *Stats) void {
        self.vtable.getStats(self, stats);
    }

    /// Range of triangles to start with
    pub fn getInitialRange(self: *const TriangleSplitter) Range {
        return Range.init(0, @intCast(self.sorted_triangle_idx.items.len));
    }

    /// Split triangles into two groups left and right, returns null if no split could be made
    /// @param triangles The range of triangles (in sorted_triangle_idx) to process
    /// @return The ranges for the left and right subparts. sorted_triangle_idx may have been shuffled.
    pub fn split(self: *TriangleSplitter, triangles: Range) ?SplitResult {
        return self.vtable.split(self, triangles);
    }

    /// Get the list of vertices
    pub fn getVertices(self: *const TriangleSplitter) []const Float3 {
        return self.vertices;
    }

    /// Get triangle by index
    pub fn getTriangle(self: *const TriangleSplitter, idx: u32) *const IndexedTriangle {
        return &self.triangles[self.sorted_triangle_idx.items[idx]];
    }

    /// Helper function to split triangles based on dimension and split value (protected in Jolt).
    /// Returns null when one of the two ranges would be empty.
    pub fn splitInternal(self: *TriangleSplitter, triangles: Range, dimension: u32, split_value: f32) ?SplitResult {
        const sorted_idx = self.sorted_triangle_idx.items;
        const centroids = self.centroids.items;

        // Divide triangles
        var start: u32 = triangles.begin;
        var end: u32 = triangles.end;
        while (start < end) {
            // Search for first element that is on the right hand side of the split plane
            while (start < end and centroids[sorted_idx[start]].getComponent(dimension) < split_value)
                start += 1;

            // Search for the first element that is on the left hand side of the split plane
            while (start < end and centroids[sorted_idx[end - 1]].getComponent(dimension) >= split_value)
                end -= 1;

            if (start < end) {
                // Swap the two elements
                end -= 1;
                std.mem.swap(u32, &sorted_idx[start], &sorted_idx[end]);
                start += 1;
            }
        }
        std.debug.assert(start == end);

        const start_idx = start;

        if (Core.enable_asserts) {
            // Validate division algorithm
            std.debug.assert(triangles.begin <= start_idx);
            std.debug.assert(start_idx <= triangles.end);
            for (triangles.begin..start_idx) |i|
                std.debug.assert(centroids[sorted_idx[i]].getComponent(dimension) < split_value);
            for (start_idx..triangles.end) |i|
                std.debug.assert(centroids[sorted_idx[i]].getComponent(dimension) >= split_value);
        }

        const result: SplitResult = .{ .left = Range.init(triangles.begin, start_idx), .right = Range.init(start_idx, triangles.end) };
        return if (result.left.count() > 0 and result.right.count() > 0) result else null;
    }
};

/// Test splitter that splits at a fixed dimension and value (exercises the base class and its vtable)
const TestSplitter = struct {
    base: TriangleSplitter,
    dimension: u32,
    value: f32,

    const vtable: TriangleSplitter.VTable = .{ .deinit = deinitImpl, .getStats = getStatsImpl, .split = splitImpl };

    fn init(allocator: std.mem.Allocator, vertices: []const Float3, triangles: []const IndexedTriangle, dimension: u32, value: f32) !TestSplitter {
        return .{ .base = try TriangleSplitter.init(allocator, vertices, triangles, &vtable), .dimension = dimension, .value = value };
    }

    fn fromSplitter(splitter: *TriangleSplitter) *TestSplitter {
        return @alignCast(@fieldParentPtr("base", splitter));
    }

    fn deinitImpl(splitter: *TriangleSplitter, allocator: std.mem.Allocator) void {
        splitter.deinitBase(allocator);
    }

    fn getStatsImpl(_: *const TriangleSplitter, stats: *TriangleSplitter.Stats) void {
        stats.splitter_name = "TestSplitter";
    }

    fn splitImpl(splitter: *TriangleSplitter, triangles: TriangleSplitter.Range) ?TriangleSplitter.SplitResult {
        const self = fromSplitter(splitter);
        return splitter.splitInternal(triangles, self.dimension, self.value);
    }
};

test "TriangleSplitter" {
    const allocator = std.testing.allocator;

    // A row of triangles along X with centroids at x = 1, 4, 7, 10, 13, ...
    var vertices: std.ArrayList(Float3) = .empty;
    defer vertices.deinit(allocator);
    var triangles: std.ArrayList(IndexedTriangle) = .empty;
    defer triangles.deinit(allocator);
    const order = [_]u32{ 4, 1, 3, 0, 2, 5 };
    for (order) |i| {
        const x: f32 = @floatFromInt(3 * i);
        const v: u32 = @intCast(vertices.items.len);
        try vertices.appendSlice(allocator, &.{ .init(x, 0, 0), .init(x + 3, 0, 0), .init(x, 1, 1) });
        try triangles.append(allocator, .init(v, v + 1, v + 2, .{ .material_index = i }));
    }

    var splitter = try TestSplitter.init(allocator, vertices.items, triangles.items, 0, 8.0);
    defer splitter.base.deinit(allocator);
    const base = &splitter.base;

    // Centroids, the extra one is zero
    try std.testing.expectEqual(@as(usize, 7), base.centroids.items.len);
    try std.testing.expect(base.centroids.items[0].eql(.init(13, 1.0 / 3.0, 1.0 / 3.0)));
    try std.testing.expect(base.centroids.items[6].eql(.init(0, 0, 0)));

    // Initial state
    const initial = base.getInitialRange();
    try std.testing.expectEqual(@as(u32, 0), initial.begin);
    try std.testing.expectEqual(@as(u32, 6), initial.count());
    for (0..6) |i|
        try std.testing.expectEqual(&triangles.items[i], base.getTriangle(@intCast(i)));
    try std.testing.expectEqual(vertices.items.ptr, base.getVertices().ptr);

    var stats: TriangleSplitter.Stats = .{ .leaf_size = 3 };
    base.getStats(&stats);
    try std.testing.expectEqualStrings("TestSplitter", stats.splitter_name.?);
    try std.testing.expectEqual(@as(i32, 3), stats.leaf_size); // Untouched

    // Split at x = 8: centroids 1, 4, 7 go left
    const result = base.split(initial).?;
    try std.testing.expectEqual(TriangleSplitter.Range.init(0, 3), result.left);
    try std.testing.expectEqual(TriangleSplitter.Range.init(3, 6), result.right);
    for (0..3) |i|
        try std.testing.expect(base.getTriangle(@intCast(i)).material_index < 3);
    for (3..6) |i|
        try std.testing.expect(base.getTriangle(@intCast(i)).material_index >= 3);

    // Same order as Jolt's partitioning: elements on the wrong side are swapped from both ends inwards
    const expected = [_]u32{ 4, 1, 3, 2, 0, 5 };
    for (expected, 0..) |e, i|
        try std.testing.expectEqual(e, base.sorted_triangle_idx.items[i]);

    // No split when everything is on one side
    try std.testing.expectEqual(null, base.splitInternal(initial, 0, -100.0));
    try std.testing.expectEqual(null, base.splitInternal(initial, 0, 100.0));
    try std.testing.expectEqual(null, base.splitInternal(.init(2, 2), 0, 8.0));

    // Sub range in another dimension (all z centroids are 1/3)
    try std.testing.expectEqual(null, base.splitInternal(.init(1, 5), 2, 0.5));
    const sub = base.splitInternal(.init(0, 3), 0, 5.0).?;
    try std.testing.expectEqual(TriangleSplitter.Range.init(0, 2), sub.left);
    try std.testing.expectEqual(TriangleSplitter.Range.init(2, 3), sub.right);
}

test "TriangleSplitter empty and out of memory" {
    const allocator = std.testing.allocator;

    // No triangles
    var splitter = try TestSplitter.init(allocator, &.{}, &.{}, 0, 0.0);
    defer splitter.base.deinit(allocator);
    try std.testing.expectEqual(@as(u32, 0), splitter.base.getInitialRange().count());
    try std.testing.expectEqual(@as(usize, 1), splitter.base.centroids.items.len);

    // Allocation failures don't leak
    const vertices = [_]Float3{ .init(0, 0, 0), .init(1, 0, 0), .init(0, 1, 0) };
    const triangles = [_]IndexedTriangle{ .init(0, 1, 2, .{}), .init(2, 1, 0, .{}) };
    for (0..2) |fail_index| {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        try std.testing.expectError(error.OutOfMemory, TestSplitter.init(failing.allocator(), &vertices, &triangles, 0, 0.0));
    }
}
