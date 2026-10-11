//! Port of: Jolt/AABBTree/AABBTreeBuilder.h, Jolt/AABBTree/AABBTreeBuilder.cpp
//! Status: complete
//!
//! - The builder borrows the splitter (`*TriangleSplitter`, see TriangleSplitter.zig) and owns its node and triangle
//!   arrays: `init(splitter, .{ .max_triangles_per_leaf = 16 })`, `build(allocator, &stats)` and `deinit(allocator)`.
//! - `Build(AABBTreeBuilderStats &outStats)` keeps the out pointer: like in Jolt the splitter only fills in its name in
//!   `stats.splitter_stats`, the other splitter stats keep the value the caller gave them.
//! - `Node::GetTriangleCountPerNode(inNodes, outAverage, outMin, outMax)` returns
//!   `TriangleCountPerNode{ .average, .min, .max }`; `GetNChildren` takes the allocator for the output array.
//! - The `uint` -> `int` conversions of the stats are `@bitCast`, which is what the C++ conversion does.

const std = @import("std");
const builtin = @import("builtin");
const math = @import("../Math/Math.zig");
const Float3 = @import("../Math/Float3.zig").Float3;
const AABox = @import("../Geometry/AABox.zig").AABox;
const IndexedTriangle = @import("../Geometry/IndexedTriangle.zig").IndexedTriangle;
const TriangleSplitter = @import("../TriangleSplitter/TriangleSplitter.zig").TriangleSplitter;

const log = std.log.scoped(.zolt);

pub const AABBTreeBuilderStats = struct {
    // Splitter stats

    /// Stats returned by the triangle splitter algorithm
    splitter_stats: TriangleSplitter.Stats = .{},

    // Tree structure

    /// Surface Area Heuristic cost of this tree
    sah_cost: f32 = 0.0,
    /// Minimal depth of tree (number of nodes)
    min_depth: i32 = 0,
    /// Maximum depth of tree (number of nodes)
    max_depth: i32 = 0,
    /// Number of nodes in the tree
    node_count: i32 = 0,
    /// Number of leaf nodes (that contain triangles)
    leaf_node_count: i32 = 0,

    // Configured stats

    /// Configured max triangles per leaf
    max_triangles_per_leaf: i32 = 0,

    // Actual stats

    /// Minimal amount of triangles in a leaf
    tree_min_triangles_per_leaf: i32 = 0,
    /// Maximal amount of triangles in a leaf
    tree_max_triangles_per_leaf: i32 = 0,
    /// Average amount of triangles in leaf nodes
    tree_avg_triangles_per_leaf: f32 = 0.0,
};

/// Helper class to build an AABB tree
pub const AABBTreeBuilder = struct {
    /// A node in the tree, contains the AABox for the tree and any child nodes or triangles
    pub const Node = struct {
        /// Indicates that there is no child
        pub const invalid_node_index: u32 = ~@as(u32, 0);

        /// Bounding box
        bounds: AABox = .empty,

        /// Triangles (if no child nodes)
        triangles_begin: u32 = undefined, // Index into triangles
        num_triangles: u32 = 0,

        /// Child node indices (if no triangles)
        child: [2]u32 = .{ invalid_node_index, invalid_node_index },

        /// Result of `getTriangleCountPerNode`
        pub const TriangleCountPerNode = struct {
            average: f32,
            min: u32,
            max: u32,
        };

        /// Get number of triangles in this node
        pub fn getTriangleCount(self: *const Node) u32 {
            return self.num_triangles;
        }

        /// Check if this node has any children
        pub fn hasChildren(self: *const Node) bool {
            return self.child[0] != invalid_node_index or self.child[1] != invalid_node_index;
        }

        /// Get child node
        pub fn getChild(self: *const Node, idx: u32, nodes: []const Node) ?*const Node {
            return if (self.child[idx] != invalid_node_index) &nodes[self.child[idx]] else null;
        }

        /// Min depth of tree
        pub fn getMinDepth(self: *const Node, nodes: []const Node) u32 {
            if (self.hasChildren()) {
                const left = nodes[self.child[0]].getMinDepth(nodes);
                const right = nodes[self.child[1]].getMinDepth(nodes);
                return @min(left, right) + 1;
            } else return 1;
        }

        /// Max depth of tree
        pub fn getMaxDepth(self: *const Node, nodes: []const Node) u32 {
            if (self.hasChildren()) {
                const left = nodes[self.child[0]].getMaxDepth(nodes);
                const right = nodes[self.child[1]].getMaxDepth(nodes);
                return @max(left, right) + 1;
            } else return 1;
        }

        /// Number of nodes in tree
        pub fn getNodeCount(self: *const Node, nodes: []const Node) u32 {
            if (self.hasChildren())
                return nodes[self.child[0]].getNodeCount(nodes) + nodes[self.child[1]].getNodeCount(nodes) + 1
            else
                return 1;
        }

        /// Number of leaf nodes in tree
        pub fn getLeafNodeCount(self: *const Node, nodes: []const Node) u32 {
            if (self.hasChildren())
                return nodes[self.child[0]].getLeafNodeCount(nodes) + nodes[self.child[1]].getLeafNodeCount(nodes)
            else
                return 1;
        }

        /// Get triangle count in tree
        pub fn getTriangleCountInTree(self: *const Node, nodes: []const Node) u32 {
            if (self.hasChildren())
                return nodes[self.child[0]].getTriangleCountInTree(nodes) + nodes[self.child[1]].getTriangleCountInTree(nodes)
            else
                return self.getTriangleCount();
        }

        /// Calculate min and max triangles per node
        pub fn getTriangleCountPerNode(self: *const Node, nodes: []const Node) TriangleCountPerNode {
            var result: TriangleCountPerNode = .{ .average = 0, .min = std.math.maxInt(i32), .max = 0 };
            var avg_divisor: u32 = 0;
            self.getTriangleCountPerNodeInternal(nodes, &result.average, &avg_divisor, &result.min, &result.max);
            if (avg_divisor > 0)
                result.average /= @as(f32, @floatFromInt(avg_divisor));
            return result;
        }

        /// Calculate the total cost of the tree using the surface area heuristic
        pub fn calculateSAHCost(self: *const Node, nodes: []const Node, cost_traversal: f32, cost_leaf: f32) f32 {
            const surface_area = self.bounds.getSurfaceArea();
            return if (surface_area > 0.0) self.calculateSAHCostInternal(nodes, cost_traversal / surface_area, cost_leaf / surface_area) else 0.0;
        }

        /// Recursively get children (breadth first) to get in total n children (or less if there are no more)
        pub fn getNChildren(self: *const Node, allocator: std.mem.Allocator, nodes: []const Node, n: u32, children: *std.ArrayList(*const Node)) std.mem.Allocator.Error!void {
            std.debug.assert(children.items.len == 0);

            // Check if there is anything to expand
            if (!self.hasChildren())
                return;

            // Start with the children of this node
            try children.append(allocator, &nodes[self.child[0]]);
            try children.append(allocator, &nodes[self.child[1]]);

            var next: usize = 0;
            var all_triangles = true;
            while (children.items.len < n) {
                // If we have looped over all nodes, start over with the first node again
                if (next >= children.items.len) {
                    // If there only triangle nodes left, we have to terminate
                    if (all_triangles)
                        return;
                    next = 0;
                    all_triangles = true;
                }

                // Try to expand this node into its two children
                const to_expand = children.items[next];
                if (to_expand.hasChildren()) {
                    _ = children.orderedRemove(next);
                    try children.append(allocator, &nodes[to_expand.child[0]]);
                    try children.append(allocator, &nodes[to_expand.child[1]]);
                    all_triangles = false;
                } else {
                    next += 1;
                }
            }
        }

        /// Recursive helper function to calculate cost of the tree
        fn calculateSAHCostInternal(self: *const Node, nodes: []const Node, cost_traversal_div_surface_area: f32, cost_leaf_div_surface_area: f32) f32 {
            if (self.hasChildren())
                return cost_traversal_div_surface_area * self.bounds.getSurfaceArea() +
                    nodes[self.child[0]].calculateSAHCostInternal(nodes, cost_traversal_div_surface_area, cost_leaf_div_surface_area) +
                    nodes[self.child[1]].calculateSAHCostInternal(nodes, cost_traversal_div_surface_area, cost_leaf_div_surface_area)
            else
                return cost_leaf_div_surface_area * self.bounds.getSurfaceArea() * @as(f32, @floatFromInt(self.getTriangleCount()));
        }

        /// Recursive helper function to calculate min and max triangles per node
        fn getTriangleCountPerNodeInternal(self: *const Node, nodes: []const Node, average: *f32, average_divisor: *u32, min: *u32, max: *u32) void {
            if (self.hasChildren()) {
                nodes[self.child[0]].getTriangleCountPerNodeInternal(nodes, average, average_divisor, min, max);
                nodes[self.child[1]].getTriangleCountPerNodeInternal(nodes, average, average_divisor, min, max);
            } else {
                average.* += @as(f32, @floatFromInt(self.getTriangleCount()));
                average_divisor.* += 1;
                min.* = @min(min.*, self.getTriangleCount());
                max.* = @max(max.*, self.getTriangleCount());
            }
        }
    };

    triangle_splitter: *TriangleSplitter,
    max_triangles_per_leaf: u32,
    nodes: std.ArrayList(Node) = .empty,
    triangles: std.ArrayList(IndexedTriangle) = .empty,

    /// Constructor
    pub fn init(splitter: *TriangleSplitter, opts: struct { max_triangles_per_leaf: u32 = 16 }) AABBTreeBuilder {
        return .{ .triangle_splitter = splitter, .max_triangles_per_leaf = opts.max_triangles_per_leaf };
    }

    /// Destructor
    pub fn deinit(self: *AABBTreeBuilder, allocator: std.mem.Allocator) void {
        self.nodes.deinit(allocator);
        self.triangles.deinit(allocator);
    }

    /// Recursively build tree, returns the root node of the tree
    pub fn build(self: *AABBTreeBuilder, allocator: std.mem.Allocator, stats: *AABBTreeBuilderStats) std.mem.Allocator.Error!*Node {
        const initial = self.triangle_splitter.getInitialRange();

        // Worst case for number of nodes: 1 leaf node per triangle. At each level above, the number of nodes is half that of the level below.
        // This means that at most we'll be allocating 2x the number of triangles in nodes.
        try self.nodes.ensureTotalCapacityPrecise(allocator, 2 * @as(usize, initial.count()));
        try self.triangles.ensureTotalCapacityPrecise(allocator, initial.count());

        // Build the tree
        const root = &self.nodes.items[try self.buildInternal(allocator, initial)];

        // Collect stats
        const triangles_per_leaf = root.getTriangleCountPerNode(self.nodes.items);

        self.triangle_splitter.getStats(&stats.splitter_stats);

        stats.sah_cost = root.calculateSAHCost(self.nodes.items, 1.0, 1.0);
        stats.min_depth = @bitCast(root.getMinDepth(self.nodes.items));
        stats.max_depth = @bitCast(root.getMaxDepth(self.nodes.items));
        stats.node_count = @bitCast(root.getNodeCount(self.nodes.items));
        stats.leaf_node_count = @bitCast(root.getLeafNodeCount(self.nodes.items));
        stats.max_triangles_per_leaf = @bitCast(self.max_triangles_per_leaf);
        stats.tree_min_triangles_per_leaf = @bitCast(triangles_per_leaf.min);
        stats.tree_max_triangles_per_leaf = @bitCast(triangles_per_leaf.max);
        stats.tree_avg_triangles_per_leaf = triangles_per_leaf.average;

        return root;
    }

    /// Get all nodes
    pub fn getNodes(self: *const AABBTreeBuilder) []const Node {
        return self.nodes.items;
    }

    /// Get all triangles
    pub fn getTriangles(self: *const AABBTreeBuilder) []const IndexedTriangle {
        return self.triangles.items;
    }

    fn buildInternal(self: *AABBTreeBuilder, allocator: std.mem.Allocator, triangles: TriangleSplitter.Range) std.mem.Allocator.Error!u32 {
        // Check if there are too many triangles left
        if (triangles.count() > self.max_triangles_per_leaf) {
            // Split triangles in two batches
            const split = self.triangle_splitter.split(triangles) orelse blk: {
                // When the trace below triggers:
                //
                // This code builds a tree structure to accelerate collision detection.
                // At top level it will start with all triangles in a mesh and then divides the triangles into two batches.
                // This process repeats until until the batch size is smaller than mMaxTrianglePerLeaf.
                //
                // It uses a TriangleSplitter to find a good split. When this warning triggers, the splitter was not able
                // to create a reasonable split for the triangles. This usually happens when the triangles in a batch are
                // intersecting. They could also be overlapping when projected on the 3 coordinate axis.
                //
                // To solve this issue, you could try to pass your mesh through a mesh cleaning / optimization algorithm.
                // You could also inspect the triangles that cause this issue and see if that part of the mesh can be fixed manually.
                //
                // When you do not fix this warning, the tree will be less efficient for collision detection, but it will still work.
                if (builtin.mode == .Debug)
                    log.info("AABBTreeBuilder: Doing random split for {d} triangles (max per node: {d})!", .{ triangles.count(), self.max_triangles_per_leaf });
                const half = triangles.count() / 2;
                std.debug.assert(half > 0);
                break :blk TriangleSplitter.SplitResult{
                    .left = .init(triangles.begin, triangles.begin + half),
                    .right = .init(triangles.begin + half, triangles.end),
                };
            };

            // Recursively build
            const node_index: u32 = @intCast(self.nodes.items.len);
            try self.nodes.append(allocator, .{});
            const left_index = try self.buildInternal(allocator, split.left);
            const right_index = try self.buildInternal(allocator, split.right);
            const node = &self.nodes.items[node_index];
            node.child[0] = left_index;
            node.child[1] = right_index;
            node.bounds = self.nodes.items[node.child[0]].bounds;
            node.bounds.encapsulate(self.nodes.items[node.child[1]].bounds);
            return node_index;
        }

        // Create leaf node
        const node_index: u32 = @intCast(self.nodes.items.len);
        try self.nodes.append(allocator, .{});
        const node = &self.nodes.items[self.nodes.items.len - 1];
        node.triangles_begin = @intCast(self.triangles.items.len);
        node.num_triangles = triangles.end - triangles.begin;
        const v = self.triangle_splitter.getVertices();
        for (triangles.begin..triangles.end) |i| {
            const t = self.triangle_splitter.getTriangle(@intCast(i));
            try self.triangles.append(allocator, t.*);
            node.bounds.encapsulateIndexedTriangle(v, t.*);
        }

        return node_index;
    }
};

const TriangleSplitterBinning = @import("../TriangleSplitter/TriangleSplitterBinning.zig").TriangleSplitterBinning;
const TriangleSplitterMean = @import("../TriangleSplitter/TriangleSplitterMean.zig").TriangleSplitterMean;
const Vec3 = @import("../Math/Vec3.zig").Vec3;

/// Test mesh: grid of 2 * n * n triangles in the XZ plane, cell size 1, with the material index set to the triangle index
const TestMesh = struct {
    vertices: std.ArrayList(Float3) = .empty,
    triangles: std.ArrayList(IndexedTriangle) = .empty,

    fn grid(allocator: std.mem.Allocator, n: u32) !TestMesh {
        var mesh: TestMesh = .{};
        errdefer mesh.deinit(allocator);
        for (0..n + 1) |z|
            for (0..n + 1) |x|
                try mesh.vertices.append(allocator, .init(@floatFromInt(x), 0, @floatFromInt(z)));
        for (0..n) |z|
            for (0..n) |x| {
                const v: u32 = @intCast(z * (n + 1) + x);
                const t: u32 = @intCast(mesh.triangles.items.len);
                try mesh.triangles.append(allocator, .init(v, v + n + 1, v + 1, .{ .material_index = t }));
                try mesh.triangles.append(allocator, .init(v + 1, v + n + 1, v + n + 2, .{ .material_index = t + 1 }));
            };
        return mesh;
    }

    fn deinit(self: *TestMesh, allocator: std.mem.Allocator) void {
        self.vertices.deinit(allocator);
        self.triangles.deinit(allocator);
    }
};

/// Check the invariants of a tree built by AABBTreeBuilder
fn validateTree(builder: *const AABBTreeBuilder, root: *const AABBTreeBuilder.Node, mesh: *const TestMesh, stats: AABBTreeBuilderStats) !void {
    const nodes = builder.getNodes();
    try std.testing.expectEqual(&nodes[0], root);

    // Every triangle appears exactly once
    const triangles = builder.getTriangles();
    try std.testing.expectEqual(mesh.triangles.items.len, triangles.len);
    const seen = try std.testing.allocator.alloc(bool, triangles.len);
    defer std.testing.allocator.free(seen);
    @memset(seen, false);
    for (triangles) |t| {
        try std.testing.expect(!seen[t.material_index]);
        seen[t.material_index] = true;
        try std.testing.expect(t.eql(mesh.triangles.items[t.material_index]));
    }

    // Leaves contain their triangles, parents their children, and leaves cover the triangle array in order
    var next_triangle: u32 = 0;
    var num_leaves: u32 = 0;
    var stack: std.ArrayList(u32) = .empty;
    defer stack.deinit(std.testing.allocator);
    try stack.append(std.testing.allocator, 0);
    while (stack.pop()) |index| {
        const node = &nodes[index];
        if (node.hasChildren()) {
            try std.testing.expectEqual(@as(u32, 0), node.getTriangleCount());
            for (0..2) |i| {
                const child = node.getChild(@intCast(i), nodes).?;
                try std.testing.expect(node.bounds.contains(child.bounds));
                try std.testing.expect(node.child[i] > index); // Children are added after their parent
            }
            try stack.append(std.testing.allocator, node.child[1]);
            try stack.append(std.testing.allocator, node.child[0]);
        } else {
            try std.testing.expectEqual(null, node.getChild(0, nodes));
            try std.testing.expectEqual(null, node.getChild(1, nodes));
            try std.testing.expect(node.getTriangleCount() <= builder.max_triangles_per_leaf or node.getTriangleCount() == 1);
            try std.testing.expectEqual(next_triangle, node.triangles_begin);
            var bounds: AABox = .empty;
            for (triangles[node.triangles_begin..][0..node.num_triangles]) |t|
                bounds.encapsulateIndexedTriangle(mesh.vertices.items, t);
            try std.testing.expect(bounds.eql(node.bounds));
            next_triangle += node.num_triangles;
            num_leaves += 1;
        }
    }
    try std.testing.expectEqual(@as(u32, @intCast(triangles.len)), next_triangle);

    // Stats
    try std.testing.expectEqual(@as(i32, @intCast(nodes.len)), stats.node_count);
    try std.testing.expectEqual(@as(i32, @intCast(num_leaves)), stats.leaf_node_count);
    try std.testing.expectEqual(@as(i32, @intCast(builder.max_triangles_per_leaf)), stats.max_triangles_per_leaf);
    try std.testing.expect(stats.min_depth <= stats.max_depth);
    try std.testing.expect(stats.tree_min_triangles_per_leaf <= stats.tree_max_triangles_per_leaf);
    try std.testing.expectApproxEqRel(@as(f32, @floatFromInt(triangles.len)) / @as(f32, @floatFromInt(num_leaves)), stats.tree_avg_triangles_per_leaf, 1.0e-6);
    try std.testing.expectEqual(@as(u32, @intCast(triangles.len)), root.getTriangleCountInTree(nodes));
    try std.testing.expect(stats.sah_cost > 0.0);
}

test "AABBTreeBuilder" {
    const allocator = std.testing.allocator;

    var mesh = try TestMesh.grid(allocator, 16);
    defer mesh.deinit(allocator);

    for ([_]u32{ 1, 2, 4, 8, 16 }) |max_triangles_per_leaf| {
        for (0..2) |splitter_type| {
            var binning = try TriangleSplitterBinning.init(allocator, mesh.vertices.items, mesh.triangles.items, .{});
            defer binning.deinit(allocator);
            var mean = try TriangleSplitterMean.init(allocator, mesh.vertices.items, mesh.triangles.items);
            defer mean.deinit(allocator);
            const splitter = if (splitter_type == 0) binning.splitter() else mean.splitter();

            var builder: AABBTreeBuilder = .init(splitter, .{ .max_triangles_per_leaf = max_triangles_per_leaf });
            defer builder.deinit(allocator);
            var stats: AABBTreeBuilderStats = .{};
            const root = try builder.build(allocator, &stats);
            try validateTree(&builder, root, &mesh, stats);

            try std.testing.expectEqualStrings(if (splitter_type == 0) "TriangleSplitterBinning" else "TriangleSplitterMean", stats.splitter_stats.splitter_name.?);
            try std.testing.expect(root.bounds.eql(.init(.init(0, 0, 0), .init(16, 0, 16))));

            // A regular grid splits perfectly
            try std.testing.expectEqual(@as(i32, @intCast(max_triangles_per_leaf)), stats.tree_max_triangles_per_leaf);
            try std.testing.expectEqual(@as(i32, @intCast(max_triangles_per_leaf)), stats.tree_min_triangles_per_leaf);
            try std.testing.expectEqual(stats.min_depth, stats.max_depth);
        }
    }
}

test "AABBTreeBuilder single leaf" {
    const allocator = std.testing.allocator;

    var mesh = try TestMesh.grid(allocator, 2);
    defer mesh.deinit(allocator);

    var mean = try TriangleSplitterMean.init(allocator, mesh.vertices.items, mesh.triangles.items);
    defer mean.deinit(allocator);
    var builder: AABBTreeBuilder = .init(mean.splitter(), .{});
    defer builder.deinit(allocator);
    var stats: AABBTreeBuilderStats = .{ .splitter_stats = .{ .leaf_size = 5 } };
    const root = try builder.build(allocator, &stats);
    try validateTree(&builder, root, &mesh, stats);

    try std.testing.expect(!root.hasChildren());
    try std.testing.expectEqual(@as(usize, 1), builder.getNodes().len);
    try std.testing.expectEqual(@as(u32, 8), root.getTriangleCount());
    try std.testing.expectEqual(@as(i32, 5), stats.splitter_stats.leaf_size); // Not touched by the splitter
    try std.testing.expectEqual(@as(i32, 1), stats.min_depth);
    try std.testing.expectEqual(@as(i32, 1), stats.max_depth);
    try std.testing.expectEqual(@as(i32, 8), stats.tree_min_triangles_per_leaf);
    try std.testing.expectEqual(@as(f32, 8.0), stats.tree_avg_triangles_per_leaf);
    try std.testing.expectEqual(@as(f32, 8.0), stats.sah_cost); // Leaf cost 1 * area / area * 8 triangles

    // Leaf nodes have no children to expand
    var children: std.ArrayList(*const AABBTreeBuilder.Node) = .empty;
    defer children.deinit(allocator);
    try root.getNChildren(allocator, builder.getNodes(), 4, &children);
    try std.testing.expectEqual(@as(usize, 0), children.items.len);
}

test "AABBTreeBuilder random split and GetNChildren" {
    const allocator = std.testing.allocator;

    // Overlapping identical triangles can't be split, the builder splits them in half
    const vertices = [_]Float3{ .init(0, 0, 0), .init(1, 0, 0), .init(0, 1, 0), .init(5, 5, 5), .init(6, 5, 5), .init(5, 6, 5) };
    var triangles: [24]IndexedTriangle = undefined;
    for (&triangles, 0..) |*t, i| t.* = if (i < 20) .init(0, 1, 2, .{ .material_index = @intCast(i) }) else .init(3, 4, 5, .{ .material_index = @intCast(i) });
    var mesh: TestMesh = .{};
    defer mesh.deinit(allocator);
    try mesh.vertices.appendSlice(allocator, &vertices);
    try mesh.triangles.appendSlice(allocator, &triangles);

    var binning = try TriangleSplitterBinning.init(allocator, &vertices, &triangles, .{});
    defer binning.deinit(allocator);
    var builder: AABBTreeBuilder = .init(binning.splitter(), .{ .max_triangles_per_leaf = 2 });
    defer builder.deinit(allocator);
    var stats: AABBTreeBuilderStats = .{};
    const root = try builder.build(allocator, &stats);
    try validateTree(&builder, root, &mesh, stats);
    const nodes = builder.getNodes();

    // The first split separates the two groups, the 20 identical triangles are split in halves: 20 -> 10 -> 5 -> 2 + 3 -> ...
    try std.testing.expectEqual(@as(u32, 20), nodes[root.child[0]].getTriangleCountInTree(nodes));
    try std.testing.expectEqual(@as(u32, 4), nodes[root.child[1]].getTriangleCountInTree(nodes));
    try std.testing.expectEqual(@as(i32, 2), stats.tree_max_triangles_per_leaf);
    try std.testing.expectEqual(@as(i32, 1), stats.tree_min_triangles_per_leaf);

    // GetNChildren expands breadth first
    var children: std.ArrayList(*const AABBTreeBuilder.Node) = .empty;
    defer children.deinit(allocator);
    try root.getNChildren(allocator, nodes, 2, &children);
    try std.testing.expectEqual(@as(usize, 2), children.items.len);
    try std.testing.expectEqual(&nodes[root.child[0]], children.items[0]);
    try std.testing.expectEqual(&nodes[root.child[1]], children.items[1]);

    children.clearRetainingCapacity();
    try root.getNChildren(allocator, nodes, 4, &children);
    try std.testing.expectEqual(@as(usize, 4), children.items.len);
    const left = &nodes[root.child[0]];
    const right = &nodes[root.child[1]];
    try std.testing.expectEqual(&nodes[left.child[0]], children.items[0]);
    try std.testing.expectEqual(&nodes[left.child[1]], children.items[1]);
    try std.testing.expectEqual(&nodes[right.child[0]], children.items[2]);
    try std.testing.expectEqual(&nodes[right.child[1]], children.items[3]);

    // Asking for more children than leaves returns all leaves
    children.clearRetainingCapacity();
    try root.getNChildren(allocator, nodes, 1000, &children);
    try std.testing.expectEqual(@as(usize, @intCast(stats.leaf_node_count)), children.items.len);
    for (children.items) |c| try std.testing.expect(!c.hasChildren());

    // Root bounds
    try std.testing.expect(root.bounds.eql(.init(Vec3.zero(), Vec3.init(6, 6, 5))));
}

test "AABBTreeBuilder empty and out of memory" {
    const allocator = std.testing.allocator;

    // No triangles: a single empty leaf
    var mean = try TriangleSplitterMean.init(allocator, &.{}, &.{});
    defer mean.deinit(allocator);
    var builder: AABBTreeBuilder = .init(mean.splitter(), .{});
    defer builder.deinit(allocator);
    var stats: AABBTreeBuilderStats = .{};
    const root = try builder.build(allocator, &stats);
    try std.testing.expect(!root.hasChildren());
    try std.testing.expectEqual(@as(u32, 0), root.getTriangleCount());
    try std.testing.expect(!root.bounds.isValid());
    try std.testing.expect(std.math.isNan(stats.sah_cost)); // Empty box: infinite surface area * 0 triangles
    try std.testing.expectEqual(@as(f32, 0.0), stats.tree_avg_triangles_per_leaf);

    // Allocation failures don't leak
    var mesh = try TestMesh.grid(allocator, 4);
    defer mesh.deinit(allocator);
    var binning = try TriangleSplitterBinning.init(allocator, mesh.vertices.items, mesh.triangles.items, .{});
    defer binning.deinit(allocator);
    for (0..2) |fail_index| {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var b: AABBTreeBuilder = .init(binning.splitter(), .{ .max_triangles_per_leaf = 1 });
        defer b.deinit(failing.allocator());
        var s: AABBTreeBuilderStats = .{};
        try std.testing.expectError(error.OutOfMemory, b.build(failing.allocator(), &s));
    }
}
