//! Port of: Jolt/TriangleSplitter/TriangleSplitterMean.h, Jolt/TriangleSplitter/TriangleSplitterMean.cpp
//! Status: complete
//!
//! Derived splitter of `TriangleSplitter` (polymorphism pattern A): pass `&splitter.base` (or `splitter.splitter()`)
//! where Jolt passes a `TriangleSplitter &`.

const std = @import("std");
const Float3 = @import("../Math/Float3.zig").Float3;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const IndexedTriangle = @import("../Geometry/IndexedTriangle.zig").IndexedTriangle;
const TriangleSplitter = @import("TriangleSplitter.zig").TriangleSplitter;

/// Splitter using mean of axis with biggest centroid deviation
pub const TriangleSplitterMean = struct {
    base: TriangleSplitter,

    const vtable: TriangleSplitter.VTable = .{ .deinit = deinitImpl, .getStats = getStatsImpl, .split = splitImpl };

    /// Constructor
    pub fn init(allocator: std.mem.Allocator, vertices: []const Float3, triangles: []const IndexedTriangle) std.mem.Allocator.Error!TriangleSplitterMean {
        return .{ .base = try TriangleSplitter.init(allocator, vertices, triangles, &vtable) };
    }

    /// Destructor
    pub fn deinit(self: *TriangleSplitterMean, allocator: std.mem.Allocator) void {
        self.base.deinitBase(allocator);
    }

    /// The base class (to pass to AABBTreeBuilder)
    pub fn splitter(self: *TriangleSplitterMean) *TriangleSplitter {
        return &self.base;
    }

    /// Downcast (static_cast<TriangleSplitterMean *>(splitter) in C++)
    pub fn fromSplitter(base: *TriangleSplitter) *TriangleSplitterMean {
        return @alignCast(@fieldParentPtr("base", base));
    }

    /// Downcast, const version
    pub fn fromSplitterConst(base: *const TriangleSplitter) *const TriangleSplitterMean {
        return @alignCast(@fieldParentPtr("base", base));
    }

    // See TriangleSplitter::GetStats
    pub fn getStats(self: *const TriangleSplitterMean, stats: *TriangleSplitter.Stats) void {
        _ = self;
        stats.splitter_name = "TriangleSplitterMean";
    }

    // See TriangleSplitter::Split
    pub fn split(self: *TriangleSplitterMean, triangles: TriangleSplitter.Range) ?TriangleSplitter.SplitResult {
        const sorted_idx = self.base.sorted_triangle_idx.items[triangles.begin..triangles.end];
        const centroids = self.base.centroids.items;

        // Calculate mean value for these triangles
        var mean = Vec3.zero();
        for (sorted_idx) |t|
            mean = mean.add(Vec3.loadFloat3Unsafe(&centroids[t]));
        mean = mean.mulScalar(1.0 / @as(f32, @floatFromInt(triangles.count())));

        // Calculate deviation
        var deviation = Vec3.zero();
        for (sorted_idx) |t| {
            const delta = Vec3.loadFloat3Unsafe(&centroids[t]).sub(mean);
            deviation = deviation.add(delta.mul(delta));
        }
        deviation = deviation.mulScalar(1.0 / @as(f32, @floatFromInt(triangles.count())));

        // Calculate split plane
        const dimension = deviation.getHighestComponentIndex();
        const split_value = mean.getComponent(dimension);

        return self.base.splitInternal(triangles, dimension, split_value);
    }

    fn deinitImpl(base: *TriangleSplitter, allocator: std.mem.Allocator) void {
        fromSplitter(base).deinit(allocator);
    }

    fn getStatsImpl(base: *const TriangleSplitter, stats: *TriangleSplitter.Stats) void {
        fromSplitterConst(base).getStats(stats);
    }

    fn splitImpl(base: *TriangleSplitter, triangles: TriangleSplitter.Range) ?TriangleSplitter.SplitResult {
        return fromSplitter(base).split(triangles);
    }
};

test "TriangleSplitterMean" {
    const allocator = std.testing.allocator;

    // Triangles along X with centroids at x = 1, 2, 3, 10 (mean 4) and spread less in Y and Z
    const vertices = [_]Float3{
        .init(0, 0, 0), .init(2, 0, 0),  .init(1, 1, 1),
        .init(1, 0, 0), .init(3, 0, 0),  .init(2, 1, 1),
        .init(2, 0, 0), .init(4, 0, 0),  .init(3, 1, 1),
        .init(9, 0, 0), .init(11, 0, 0), .init(10, 1, 1),
    };
    const triangles = [_]IndexedTriangle{ .init(9, 10, 11, .{}), .init(0, 1, 2, .{}), .init(6, 7, 8, .{}), .init(3, 4, 5, .{}) };

    var mean = try TriangleSplitterMean.init(allocator, &vertices, &triangles);
    defer mean.deinit(allocator);

    const base = mean.splitter();
    var stats: TriangleSplitter.Stats = .{};
    base.getStats(&stats);
    try std.testing.expectEqualStrings("TriangleSplitterMean", stats.splitter_name.?);

    // Split at the mean (x = 4): the triangle at x = 10 goes right
    const result = base.split(base.getInitialRange()).?;
    try std.testing.expectEqual(TriangleSplitter.Range.init(0, 3), result.left);
    try std.testing.expectEqual(TriangleSplitter.Range.init(3, 4), result.right);
    try std.testing.expectEqual(&triangles[0], base.getTriangle(3));

    // Split the left part at x = 2: centroid x = 1 left, 2 and 3 right
    const left = mean.split(result.left).?;
    try std.testing.expectEqual(TriangleSplitter.Range.init(0, 1), left.left);
    try std.testing.expectEqual(TriangleSplitter.Range.init(1, 3), left.right);
    try std.testing.expectEqual(&triangles[1], base.getTriangle(0));

    // A single triangle can't be split
    try std.testing.expectEqual(null, mean.split(.init(0, 1)));

    // Identical triangles can't be split
    const same = [_]IndexedTriangle{ .init(0, 1, 2, .{}), .init(0, 1, 2, .{}), .init(2, 1, 0, .{}) };
    var mean2 = try TriangleSplitterMean.init(allocator, &vertices, &same);
    defer mean2.base.deinit(allocator); // Virtual destructor
    try std.testing.expectEqual(null, mean2.base.split(mean2.base.getInitialRange()));
}
