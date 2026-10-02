//! Port of: Jolt/TriangleSplitter/TriangleSplitterBinning.h, Jolt/TriangleSplitter/TriangleSplitterBinning.cpp
//! Status: complete
//!
//! Derived splitter of `TriangleSplitter` (polymorphism pattern A): pass `&splitter.base` (or `splitter.splitter()`)
//! where Jolt passes a `TriangleSplitter &`. The constructor's default arguments are the options struct of `init`.
//! Zolt's `Vec3.toInt` requires the bin numbers to be in the range of an i32 (safety checked), so the centroids
//! must be finite, like the meshes Jolt supports.

const std = @import("std");
const math = @import("../Math/Math.zig");
const Float3 = @import("../Math/Float3.zig").Float3;
const UVec4 = @import("../Math/UVec4.zig").UVec4;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const AABox = @import("../Geometry/AABox.zig").AABox;
const IndexedTriangle = @import("../Geometry/IndexedTriangle.zig").IndexedTriangle;
const TriangleSplitter = @import("TriangleSplitter.zig").TriangleSplitter;

/// Binning splitter approach taken from: Realtime Ray Tracing on GPU with BVH-based Packet Traversal by Johannes Gunther et al.
pub const TriangleSplitterBinning = struct {
    base: TriangleSplitter,

    // Configuration
    min_num_bins: u32,
    max_num_bins: u32,
    num_triangles_per_bin: u32,

    /// Scratch area to store the bins
    bins: std.ArrayList(Bin) = .empty,

    const Bin = struct {
        // Properties of this bin
        bounds: AABox = .empty,
        min_centroid: f32 = undefined,
        num_triangles: u32 = undefined,

        // Accumulated data from left most / right most bin to current (including this bin)
        bounds_accumulated_left: AABox = .empty,
        bounds_accumulated_right: AABox = .empty,
        num_triangles_accumulated_left: u32 = undefined,
        num_triangles_accumulated_right: u32 = undefined,
    };

    /// Optional arguments of `init`
    pub const Options = struct {
        min_num_bins: u32 = 8,
        max_num_bins: u32 = 128,
        num_triangles_per_bin: u32 = 6,
    };

    const vtable: TriangleSplitter.VTable = .{ .deinit = deinitImpl, .getStats = getStatsImpl, .split = splitImpl };

    /// Constructor
    pub fn init(allocator: std.mem.Allocator, vertices: []const Float3, triangles: []const IndexedTriangle, opts: Options) std.mem.Allocator.Error!TriangleSplitterBinning {
        var self: TriangleSplitterBinning = .{
            .base = try TriangleSplitter.init(allocator, vertices, triangles, &vtable),
            .min_num_bins = opts.min_num_bins,
            .max_num_bins = opts.max_num_bins,
            .num_triangles_per_bin = opts.num_triangles_per_bin,
        };
        errdefer self.base.deinitBase(allocator);

        try self.bins.appendNTimes(allocator, .{}, self.max_num_bins * 3); // mMaxNumBins per dimension

        return self;
    }

    /// Destructor
    pub fn deinit(self: *TriangleSplitterBinning, allocator: std.mem.Allocator) void {
        self.bins.deinit(allocator);
        self.base.deinitBase(allocator);
    }

    /// The base class (to pass to AABBTreeBuilder)
    pub fn splitter(self: *TriangleSplitterBinning) *TriangleSplitter {
        return &self.base;
    }

    /// Downcast (static_cast<TriangleSplitterBinning *>(splitter) in C++)
    pub fn fromSplitter(base: *TriangleSplitter) *TriangleSplitterBinning {
        return @alignCast(@fieldParentPtr("base", base));
    }

    /// Downcast, const version
    pub fn fromSplitterConst(base: *const TriangleSplitter) *const TriangleSplitterBinning {
        return @alignCast(@fieldParentPtr("base", base));
    }

    // See TriangleSplitter::GetStats
    pub fn getStats(self: *const TriangleSplitterBinning, stats: *TriangleSplitter.Stats) void {
        _ = self;
        stats.splitter_name = "TriangleSplitterBinning";
    }

    // See TriangleSplitter::Split
    pub fn split(self: *TriangleSplitterBinning, triangles: TriangleSplitter.Range) ?TriangleSplitter.SplitResult {
        const sorted_idx = self.base.sorted_triangle_idx.items[triangles.begin..triangles.end];
        const centroids = self.base.centroids.items;
        const bins = self.bins.items;

        // Calculate bounds for this range
        var centroid_bounds: AABox = .empty;
        for (sorted_idx) |t|
            centroid_bounds.encapsulateVec3(Vec3.loadFloat3Unsafe(&centroids[t]));

        // Convert bounds to min coordinate and size
        // Prevent division by zero if one of the dimensions is zero
        const min_size: f32 = 1.0e-5;
        const bounds_min = centroid_bounds.min;
        const bounds_size = Vec3.max(centroid_bounds.max.sub(bounds_min), Vec3.replicate(min_size));

        var best_cp: f32 = math.flt_max;
        var best_dim: u32 = 0xffffffff;
        var best_split: f32 = 0;

        // Bin in all dimensions
        const num_bins: u32 = math.clamp(triangles.count() / self.num_triangles_per_bin, self.min_num_bins, self.max_num_bins);
        const num_bins_f: f32 = @floatFromInt(num_bins);

        // Initialize bins
        for (0..3) |dim| {
            // Get bounding box size for this dimension
            const bounds_min_dim = bounds_min.getComponent(@intCast(dim));
            const bounds_size_dim = bounds_size.getComponent(@intCast(dim));

            // Get the bins for this dimension
            const bins_dim = bins[num_bins * dim ..][0..num_bins];

            for (bins_dim, 0..) |*bin, b| {
                bin.bounds.setEmpty();
                bin.min_centroid = bounds_min_dim + bounds_size_dim * @as(f32, @floatFromInt(b + 1)) / num_bins_f;
                bin.num_triangles = 0;
            }
        }

        // Bin all triangles in all dimensions at once
        for (sorted_idx) |t| {
            const centroid_pos = Vec3.loadFloat3Unsafe(&centroids[t]);

            const triangle_bounds = AABox.fromTriangle(self.base.vertices, self.base.triangles[t]);

            const bin_no_f = centroid_pos.sub(bounds_min).div(bounds_size).mulScalar(num_bins_f);
            const bin_no = UVec4.min(bin_no_f.toInt(), UVec4.replicate(num_bins - 1));

            for (0..3) |dim| {
                // Select bin
                const bin = &bins[num_bins * dim + bin_no.getComponent(@intCast(dim))];

                // Accumulate triangle in bin
                bin.bounds.encapsulate(triangle_bounds);
                bin.min_centroid = math.min(bin.min_centroid, centroid_pos.getComponent(@intCast(dim)));
                bin.num_triangles += 1;
            }
        }

        for (0..3) |dim| {
            // Skip axis if too small
            if (bounds_size.getComponent(@intCast(dim)) <= min_size)
                continue;

            // Get the bins for this dimension
            const bins_dim = bins[num_bins * dim ..][0..num_bins];

            // Calculate totals left to right
            var prev_bounds: AABox = .empty;
            var prev_triangles: i32 = 0;
            for (bins_dim) |*bin| {
                bin.bounds_accumulated_left = prev_bounds; // Don't include this node as we'll take a split on the left side of the bin
                bin.num_triangles_accumulated_left = @intCast(prev_triangles);
                prev_bounds.encapsulate(bin.bounds);
                prev_triangles += @intCast(bin.num_triangles);
            }

            // Calculate totals right to left
            prev_bounds.setEmpty();
            prev_triangles = 0;
            var b: usize = num_bins;
            while (b > 0) {
                b -= 1;
                const bin = &bins_dim[b];
                prev_bounds.encapsulate(bin.bounds);
                prev_triangles += @intCast(bin.num_triangles);
                bin.bounds_accumulated_right = prev_bounds;
                bin.num_triangles_accumulated_right = @intCast(prev_triangles);
            }

            // Get best splitting plane
            for (bins_dim[1..]) |*bin| { // Start at 1 since selecting bin 0 would result in everything ending up on the right side
                // Calculate surface area heuristic and see if it is better than the current best
                const cp = bin.bounds_accumulated_left.getSurfaceArea() * @as(f32, @floatFromInt(bin.num_triangles_accumulated_left)) + bin.bounds_accumulated_right.getSurfaceArea() * @as(f32, @floatFromInt(bin.num_triangles_accumulated_right));
                if (cp < best_cp) {
                    best_cp = cp;
                    best_dim = @intCast(dim);
                    best_split = bin.min_centroid;
                }
            }
        }

        // No split found?
        if (best_dim == 0xffffffff)
            return null;

        return self.base.splitInternal(triangles, best_dim, best_split);
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

/// Grid of 2 * n * n triangles in the XZ plane, cell size 1, starting at the origin
fn createGrid(allocator: std.mem.Allocator, n: u32, vertices: *std.ArrayList(Float3), triangles: *std.ArrayList(IndexedTriangle)) !void {
    for (0..n + 1) |z|
        for (0..n + 1) |x|
            try vertices.append(allocator, .init(@floatFromInt(x), 0, @floatFromInt(z)));
    for (0..n) |z|
        for (0..n) |x| {
            const v: u32 = @intCast(z * (n + 1) + x);
            try triangles.append(allocator, .init(v, v + n + 1, v + 1, .{}));
            try triangles.append(allocator, .init(v + 1, v + n + 1, v + n + 2, .{}));
        };
}

test "TriangleSplitterBinning" {
    const allocator = std.testing.allocator;

    var vertices: std.ArrayList(Float3) = .empty;
    defer vertices.deinit(allocator);
    var triangles: std.ArrayList(IndexedTriangle) = .empty;
    defer triangles.deinit(allocator);
    try createGrid(allocator, 8, &vertices, &triangles);

    var binning = try TriangleSplitterBinning.init(allocator, vertices.items, triangles.items, .{});
    defer binning.deinit(allocator);
    try std.testing.expectEqual(@as(usize, 3 * 128), binning.bins.items.len);

    const base = binning.splitter();
    var stats: TriangleSplitter.Stats = .{};
    base.getStats(&stats);
    try std.testing.expectEqualStrings("TriangleSplitterBinning", stats.splitter_name.?);
    try std.testing.expectEqual(@as(i32, 0), stats.leaf_size);

    // The grid is split in two halves of equal size (through the virtual function)
    const initial = base.getInitialRange();
    try std.testing.expectEqual(@as(u32, 128), initial.count());
    const result = base.split(initial).?;
    try std.testing.expectEqual(@as(u32, 0), result.left.begin);
    try std.testing.expectEqual(@as(u32, 64), result.left.count());
    try std.testing.expectEqual(@as(u32, 128), result.right.end);

    // All triangles on the left are on one side of the split plane, which is along X or Z
    var left_bounds: AABox = .empty;
    for (result.left.begin..result.left.end) |i|
        left_bounds.encapsulateIndexedTriangle(vertices.items, base.getTriangle(@intCast(i)).*);
    var right_bounds: AABox = .empty;
    for (result.right.begin..result.right.end) |i|
        right_bounds.encapsulateIndexedTriangle(vertices.items, base.getTriangle(@intCast(i)).*);
    try std.testing.expect(left_bounds.max.getX() <= right_bounds.min.getX() or left_bounds.max.getZ() <= right_bounds.min.getZ());

    // Keep splitting until single triangles
    var stack: std.ArrayList(TriangleSplitter.Range) = .empty;
    defer stack.deinit(allocator);
    try stack.append(allocator, result.left);
    try stack.append(allocator, result.right);
    var num_leaves: u32 = 0;
    while (stack.pop()) |range| {
        if (range.count() <= 1) {
            num_leaves += 1;
            continue;
        }
        // Two triangles of a cell share a centroid in neither dimension, so a split is always possible
        const r = binning.split(range).?;
        try std.testing.expectEqual(range.begin, r.left.begin);
        try std.testing.expectEqual(r.left.end, r.right.begin);
        try std.testing.expectEqual(range.end, r.right.end);
        try stack.append(allocator, r.left);
        try stack.append(allocator, r.right);
    }
    try std.testing.expectEqual(@as(u32, 128), num_leaves);

    // Every triangle is still there exactly once
    var seen = [_]bool{false} ** 128;
    for (binning.base.sorted_triangle_idx.items) |idx| {
        try std.testing.expect(!seen[idx]);
        seen[idx] = true;
    }
}

test "TriangleSplitterBinning degenerate" {
    const allocator = std.testing.allocator;

    // All triangles have the same centroid: no split possible
    const vertices = [_]Float3{ .init(0, 0, 0), .init(1, 0, 0), .init(0, 1, 0), .init(1, 1, 0) };
    var triangles: [20]IndexedTriangle = undefined;
    for (&triangles) |*t| t.* = .init(0, 1, 2, .{});

    var binning = try TriangleSplitterBinning.init(allocator, &vertices, &triangles, .{ .min_num_bins = 2, .max_num_bins = 4, .num_triangles_per_bin = 1 });
    defer binning.base.deinit(allocator); // Virtual destructor
    try std.testing.expectEqual(@as(usize, 12), binning.bins.items.len);
    try std.testing.expectEqual(null, binning.base.split(binning.base.getInitialRange()));

    // One triangle with a different centroid in X and Y
    triangles[3] = .init(1, 3, 2, .{});
    var binning2 = try TriangleSplitterBinning.init(allocator, &vertices, &triangles, .{ .min_num_bins = 2, .max_num_bins = 4, .num_triangles_per_bin = 1 });
    defer binning2.base.deinit(allocator);
    const r = binning2.base.split(binning2.base.getInitialRange()).?;
    try std.testing.expectEqual(@as(u32, 19), r.left.count());
    try std.testing.expectEqual(@as(u32, 1), r.right.count());
    try std.testing.expectEqual(&triangles[3], binning2.base.getTriangle(19));
}
