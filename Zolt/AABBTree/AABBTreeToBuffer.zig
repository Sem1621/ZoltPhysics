//! Port of: Jolt/AABBTree/AABBTreeToBuffer.h
//! Status: complete
//!
//! `template <class TriangleCodec, class NodeCodec> class AABBTreeToBuffer` is the type function
//! `AABBTreeToBuffer(TriangleCodec, NodeCodec)`. The resulting buffer is owned: `deinit(allocator)` frees it.
//!
//! `Convert(..., const char *&outError) -> bool` is `convert(allocator, ...) Error!void`; `errorMessage(err)` gives
//! Jolt's message for an error (the codecs provide the messages of their own errors). The `size_t(total_size) !=
//! total_size` check ("AABBTreeToBuffer: Out of memory!") and allocation failures are `error.OutOfMemory`.
//!
//! A node codec (see NodeCodec/NodeCodecQuadTreeHalfFloat.zig) declares: `Header`, `header_size`,
//! `num_children_per_node`, `Error` + `errorMessage(Error)`, and `EncodingContext` (default value `.{}`) with
//! `prepareNodeAllocate`, `nodeAllocate`, `nodeFinalize` and `finalize`.
//! A triangle codec (see TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.zig) declares: `TriangleHeader`,
//! `triangle_header_size`, `Error` + `errorMessage(Error)`, and `EncodingContext` with `init(allocator, vertices)`,
//! `deinit`, `preparePack`, `finalizePreparePack`, `pack` and `finalize`.
//!
//! Zolt guards the pointers that Convert keeps into its node list: when the node estimate would be exceeded (which
//! would reallocate the list and leave dangling pointers in the C++, undefined behavior) it returns
//! `error.NodeMemoryEstimateIncorrect`, the error Jolt reports for a wrong estimate.

const std = @import("std");
const Float3 = @import("../Math/Float3.zig").Float3;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const ByteBuffer = @import("../Core/ByteBuffer.zig").ByteBuffer;
const AABox = @import("../Geometry/AABox.zig").AABox;
const IndexedTriangle = @import("../Geometry/IndexedTriangle.zig").IndexedTriangle;
const AABBTreeBuilder = @import("AABBTreeBuilder.zig").AABBTreeBuilder;

/// Conversion algorithm that converts an AABB tree to an optimized binary buffer
pub fn AABBTreeToBuffer(comptime TriangleCodec: type, comptime NodeCodec: type) type {
    return struct {
        const Self = @This();
        const Node = AABBTreeBuilder.Node;

        /// Header for the tree
        pub const NodeHeader = NodeCodec.Header;

        /// Size in bytes of the header of the tree
        pub const header_size: comptime_int = NodeCodec.header_size;

        /// Maximum number of children per node in the tree
        pub const num_children_per_node: comptime_int = NodeCodec.num_children_per_node;

        /// Header for the triangles
        pub const TriangleHeader = TriangleCodec.TriangleHeader;

        /// Size in bytes of the header for the triangles
        pub const triangle_header_size: comptime_int = TriangleCodec.triangle_header_size;

        /// Errors of `convert`
        pub const Error = error{
            /// "Internal Error: Node memory estimate was incorrect, memory corruption!"
            NodeMemoryEstimateIncorrect,
            /// "Internal Error: Tree memory estimate was incorrect, memory corruption!"
            TreeMemoryEstimateIncorrect,
        } || NodeCodec.Error || TriangleCodec.Error || std.mem.Allocator.Error;

        /// Resulting tree structure
        tree: ByteBuffer = .empty,

        pub const empty: Self = .{};

        /// Destructor
        pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
            self.tree.deinit(allocator);
        }

        /// Jolt's error message (outError) for an error returned by `convert`
        pub fn errorMessage(err: Error) []const u8 {
            switch (err) {
                error.OutOfMemory => return "AABBTreeToBuffer: Out of memory!",
                error.NodeMemoryEstimateIncorrect => return "Internal Error: Node memory estimate was incorrect, memory corruption!",
                error.TreeMemoryEstimateIncorrect => return "Internal Error: Tree memory estimate was incorrect, memory corruption!",
                else => {
                    inline for (@typeInfo(NodeCodec.Error).error_set.?) |e|
                        if (err == @field(anyerror, e.name)) return NodeCodec.errorMessage(@field(NodeCodec.Error, e.name));
                    inline for (@typeInfo(TriangleCodec.Error).error_set.?) |e|
                        if (err == @field(anyerror, e.name)) return TriangleCodec.errorMessage(@field(TriangleCodec.Error, e.name));
                    unreachable;
                },
            }
        }

        /// Data of a node during the conversion
        const NodeData = struct {
            /// Node that this entry belongs to
            node: *const Node,
            /// Quantized node bounds
            node_bounds_min: Vec3,
            node_bounds_max: Vec3,
            /// Start of node in tree
            node_start: usize = std.math.maxInt(usize),
            /// Start of the triangle data in tree
            triangle_start: usize = std.math.maxInt(usize),
            /// Start of the children of the node in tree
            child_node_start: [num_children_per_node]usize = undefined,
            /// Start of the triangle data in tree
            child_triangles_start: [num_children_per_node]usize = undefined,
            /// Where to store node_start (to patch child_node_start of my parent)
            parent_child_node_start: ?*usize = null,
            /// Where to store triangle_start (to patch child_triangles_start of my parent)
            parent_triangles_start: ?*usize = null,
            /// Number of children
            num_children: u32 = 0,
        };

        /// Convert AABB tree. Returns an error if failed (see `errorMessage`).
        pub fn convert(self: *Self, allocator: std.mem.Allocator, triangles: []const IndexedTriangle, nodes: []const Node, vertices: []const Float3, root: *const Node, store_user_data: bool) Error!void {
            var node_ctx: NodeCodec.EncodingContext = .{};
            var tri_ctx = try TriangleCodec.EncodingContext.init(allocator, vertices);
            defer tri_ctx.deinit(allocator);

            // Child nodes out of loop so we don't constantly realloc it
            var child_nodes: std.ArrayList(*const Node) = .empty;
            defer child_nodes.deinit(allocator);
            try child_nodes.ensureTotalCapacityPrecise(allocator, num_children_per_node);

            // First calculate how big the tree is going to be.
            // Since the tree can be huge for very large meshes, we don't want
            // to reallocate the buffer as it may cause out of memory situations.
            // This loop mimics the construction loop below.
            var total_size: u64 = header_size + triangle_header_size;
            var node_count: usize = 1; // Start with root node
            var to_process_max_size: usize = 1; // Track size of queues so we can do a single reserve below
            var to_process_triangles_max_size: usize = 0;
            { // A scope to free the memory associated with to_estimate and to_estimate_triangles
                var to_estimate: std.ArrayList(*const Node) = .empty;
                defer to_estimate.deinit(allocator);
                var to_estimate_triangles: std.ArrayList(*const Node) = .empty;
                defer to_estimate_triangles.deinit(allocator);
                try to_estimate.append(allocator, root);
                while (true) {
                    while (to_estimate.pop()) |node| {
                        // Update total size
                        node_ctx.prepareNodeAllocate(node, &total_size);

                        if (node.hasChildren()) {
                            // Collect the first NumChildrenPerNode sub-nodes in the tree
                            child_nodes.clearRetainingCapacity(); // Won't free the memory
                            try node.getNChildren(allocator, nodes, num_children_per_node, &child_nodes);

                            // Increment the number of nodes we're going to store
                            node_count += child_nodes.items.len;

                            // Insert in reverse order so we estimate left child first when taking nodes from the back
                            var idx = child_nodes.items.len;
                            while (idx > 0) {
                                idx -= 1;

                                // Store triangles in separate list so we process them last
                                const child = child_nodes.items[idx];
                                if (child.hasChildren()) {
                                    try to_estimate.append(allocator, child);
                                    to_process_max_size = @max(to_estimate.items.len, to_process_max_size);
                                } else {
                                    try to_estimate_triangles.append(allocator, child);
                                    to_process_triangles_max_size = @max(to_estimate_triangles.items.len, to_process_triangles_max_size);
                                }
                            }
                        } else {
                            // Update total size
                            tri_ctx.preparePack(triangles[node.triangles_begin..][0..node.num_triangles], store_user_data, &total_size);
                        }
                    }

                    // If we've got triangles to estimate, loop again with just the triangles
                    if (to_estimate_triangles.items.len == 0)
                        break
                    else
                        std.mem.swap(std.ArrayList(*const Node), &to_estimate, &to_estimate_triangles);
                }
            }

            // Finalize the prepare stage for the triangle context
            try tri_ctx.finalizePreparePack(allocator, &total_size);

            // Reserve the buffer
            const total_size_usize = std.math.cast(usize, total_size) orelse return error.OutOfMemory; // "AABBTreeToBuffer: Out of memory!"
            try self.tree.vector.ensureTotalCapacityPrecise(allocator, total_size_usize);

            // Add headers
            const header_start = self.tree.vector.items.len;
            if (header_size > 0)
                _ = try self.tree.allocate(allocator, NodeHeader, .{});
            const triangle_header_start = self.tree.vector.items.len;
            if (triangle_header_size > 0)
                _ = try self.tree.allocate(allocator, TriangleHeader, .{});

            var to_process: std.ArrayList(*NodeData) = .empty;
            defer to_process.deinit(allocator);
            try to_process.ensureTotalCapacityPrecise(allocator, to_process_max_size);
            var to_process_triangles: std.ArrayList(*NodeData) = .empty;
            defer to_process_triangles.deinit(allocator);
            try to_process_triangles.ensureTotalCapacityPrecise(allocator, to_process_triangles_max_size);
            var node_list: std.ArrayList(NodeData) = .empty;
            defer node_list.deinit(allocator);
            try node_list.ensureTotalCapacityPrecise(allocator, node_count); // Needed to ensure that array is not reallocated, so we can keep pointers in the array

            node_list.appendAssumeCapacity(.{ .node = root, .node_bounds_min = root.bounds.min, .node_bounds_max = root.bounds.max });
            try to_process.append(allocator, &node_list.items[node_list.items.len - 1]);

            while (true) {
                while (to_process.pop()) |node_data| {
                    // Due to quantization box could have become bigger, not smaller
                    std.debug.assert(AABox.init(node_data.node_bounds_min, node_data.node_bounds_max).contains(node_data.node.bounds)); // AABBTreeToBuffer: Bounding box became smaller!

                    // Collect the first NumChildrenPerNode sub-nodes in the tree
                    child_nodes.clearRetainingCapacity(); // Won't free the memory
                    try node_data.node.getNChildren(allocator, nodes, num_children_per_node, &child_nodes);
                    node_data.num_children = @intCast(child_nodes.items.len);

                    // Fill in default child bounds
                    var child_bounds_min: [num_children_per_node]Vec3 = undefined;
                    var child_bounds_max: [num_children_per_node]Vec3 = undefined;
                    for (0..num_children_per_node) |i| {
                        if (i < child_nodes.items.len) {
                            child_bounds_min[i] = child_nodes.items[i].bounds.min;
                            child_bounds_max[i] = child_nodes.items[i].bounds.max;
                        } else {
                            child_bounds_min[i] = Vec3.zero();
                            child_bounds_max[i] = Vec3.zero();
                        }
                    }

                    // Start a new node
                    node_data.node_start = try node_ctx.nodeAllocate(allocator, node_data.node, node_data.node_bounds_min, node_data.node_bounds_max, child_nodes.items, &child_bounds_min, &child_bounds_max, &self.tree);

                    if (node_data.node.hasChildren()) {
                        // Insert in reverse order so we process left child first when taking nodes from the back
                        var idx = child_nodes.items.len;
                        while (idx > 0) {
                            idx -= 1;
                            const child_node = child_nodes.items[idx];

                            // Due to quantization box could have become bigger, not smaller
                            std.debug.assert(AABox.init(child_bounds_min[idx], child_bounds_max[idx]).contains(child_node.bounds)); // AABBTreeToBuffer: Bounding box became smaller!

                            // Add child to list of nodes to be processed (the list must not reallocate, see the file comment)
                            if (node_list.items.len >= node_count)
                                return error.NodeMemoryEstimateIncorrect;
                            node_list.appendAssumeCapacity(.{
                                .node = child_node,
                                .node_bounds_min = child_bounds_min[idx],
                                .node_bounds_max = child_bounds_max[idx],
                                .parent_child_node_start = &node_data.child_node_start[idx],
                                .parent_triangles_start = &node_data.child_triangles_start[idx],
                            });
                            const child = &node_list.items[node_list.items.len - 1];

                            // Store triangles in separate list so we process them last
                            if (child_node.hasChildren())
                                try to_process.append(allocator, child)
                            else
                                try to_process_triangles.append(allocator, child);
                        }
                    } else {
                        // Add triangles
                        node_data.triangle_start = try tri_ctx.pack(allocator, triangles[node_data.node.triangles_begin..][0..node_data.node.num_triangles], store_user_data, &self.tree);
                    }

                    // Patch offset into parent
                    if (node_data.parent_child_node_start) |parent_child_node_start| {
                        parent_child_node_start.* = node_data.node_start;
                        node_data.parent_triangles_start.?.* = node_data.triangle_start;
                    }
                }

                // If we've got triangles to process, loop again with just the triangles
                if (to_process_triangles.items.len == 0)
                    break
                else
                    std.mem.swap(std.ArrayList(*NodeData), &to_process, &to_process_triangles);
            }

            // Assert that our reservation was correct (we don't know if we swapped the arrays or not)
            std.debug.assert(to_process_max_size == to_process.capacity or to_process_triangles_max_size == to_process.capacity);
            std.debug.assert(to_process_max_size == to_process_triangles.capacity or to_process_triangles_max_size == to_process_triangles.capacity);

            // Finalize all nodes
            for (node_list.items) |*n|
                try node_ctx.nodeFinalize(n.node, n.node_start, n.num_children, &n.child_node_start, &n.child_triangles_start, &self.tree);

            // Finalize the triangles
            const triangle_header = if (triangle_header_size > 0) self.tree.getMut(TriangleHeader, triangle_header_start) else null;
            try tri_ctx.finalize(allocator, vertices, triangle_header, &self.tree);

            // Validate that our reservations were correct
            if (node_count != node_list.items.len)
                return error.NodeMemoryEstimateIncorrect;
            if (total_size != self.tree.vector.items.len)
                return error.TreeMemoryEstimateIncorrect;

            // Finalize the nodes
            const header = if (header_size > 0) self.tree.getMut(NodeHeader, header_start) else null;
            try node_ctx.finalize(header, root, node_list.items[0].node_start, node_list.items[0].triangle_start);
        }

        /// Get resulting data
        pub fn getBuffer(self: *const Self) *const ByteBuffer {
            return &self.tree;
        }

        /// Get resulting data (non-const version, e.g. to take ownership of the buffer)
        pub fn getBufferMut(self: *Self) *ByteBuffer {
            return &self.tree;
        }

        /// Get header for tree
        pub fn getNodeHeader(self: *const Self) *const NodeHeader {
            return self.tree.get(NodeHeader, 0);
        }

        /// Get header for triangles
        pub fn getTriangleHeader(self: *const Self) *const TriangleHeader {
            return self.tree.get(TriangleHeader, header_size);
        }

        /// Get root of resulting tree
        pub fn getRoot(self: *const Self) *const anyopaque {
            return self.tree.get(anyopaque, header_size + triangle_header_size);
        }
    };
}

const math = @import("../Math/Math.zig");
const UVec4 = @import("../Math/UVec4.zig").UVec4;
const Vec4 = @import("../Math/Vec4.zig").Vec4;
const sortReverseAndStore = @import("../Physics/Collision/SortReverseAndStore.zig").sortReverseAndStore;
const RayInvDirection = @import("../Geometry/RayAABox.zig").RayInvDirection;
const rayAABox4 = @import("../Geometry/RayAABox.zig").rayAABox4;
const rayTriangle = @import("../Geometry/RayTriangle.zig").rayTriangle;
const AABBTreeBuilderStats = @import("AABBTreeBuilder.zig").AABBTreeBuilderStats;
const TriangleSplitter = @import("../TriangleSplitter/TriangleSplitter.zig").TriangleSplitter;
const TriangleSplitterBinning = @import("../TriangleSplitter/TriangleSplitterBinning.zig").TriangleSplitterBinning;
const TriangleSplitterMean = @import("../TriangleSplitter/TriangleSplitterMean.zig").TriangleSplitterMean;
const NodeCodecQuadTreeHalfFloat = @import("NodeCodec/NodeCodecQuadTreeHalfFloat.zig").NodeCodecQuadTreeHalfFloat;
const TriangleCodecIndexed8BitPackSOA4Flags = @import("TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.zig").TriangleCodecIndexed8BitPackSOA4Flags;

const TestNodeCodec = NodeCodecQuadTreeHalfFloat;
const TestTriangleCodec = TriangleCodecIndexed8BitPackSOA4Flags;
const TestBuffer = AABBTreeToBuffer(TestTriangleCodec, TestNodeCodec);

/// Test mesh: a height field like grid of n * n cells (2 triangles each) with material index = triangle index % 256
/// and user data = triangle index
const TestMesh = struct {
    vertices: std.ArrayList(Float3) = .empty,
    triangles: std.ArrayList(IndexedTriangle) = .empty,

    fn init(allocator: std.mem.Allocator, n: u32) !TestMesh {
        var mesh: TestMesh = .{};
        errdefer mesh.deinit(allocator);
        for (0..n + 1) |z|
            for (0..n + 1) |x| {
                const fx: f32 = @floatFromInt(x);
                const fz: f32 = @floatFromInt(z);
                try mesh.vertices.append(allocator, .init(fx, @as(f32, @floatFromInt((x * 7 + z * 13) % 5)) * 0.25, fz));
            };
        for (0..n) |z|
            for (0..n) |x| {
                const v: u32 = @intCast(z * (n + 1) + x);
                for ([_][3]u32{ .{ v, v + n + 1, v + 1 }, .{ v + 1, v + n + 1, v + n + 2 } }) |idx| {
                    const t: u32 = @intCast(mesh.triangles.items.len);
                    try mesh.triangles.append(allocator, .init(idx[0], idx[1], idx[2], .{ .material_index = t % 256, .user_data = t }));
                }
            };
        return mesh;
    }

    fn deinit(self: *TestMesh, allocator: std.mem.Allocator) void {
        self.vertices.deinit(allocator);
        self.triangles.deinit(allocator);
    }
};

/// Visitor that visits every node and collects the triangles (like MeshShape::GetTrianglesNext)
const CollectVisitor = struct {
    allocator: std.mem.Allocator,
    triangles: std.ArrayList([3]Vec3) = .empty,
    flags: std.ArrayList(u8) = .empty,
    user_data: std.ArrayList(u32) = .empty,
    block_ids: std.ArrayList(u32) = .empty,
    num_nodes: u32 = 0,
    decode_mismatch: bool = false,

    fn deinit(self: *CollectVisitor) void {
        self.triangles.deinit(self.allocator);
        self.flags.deinit(self.allocator);
        self.user_data.deinit(self.allocator);
        self.block_ids.deinit(self.allocator);
    }

    pub fn shouldAbort(_: *const CollectVisitor) bool {
        return false;
    }

    pub fn shouldVisitNode(_: *const CollectVisitor, _: i32) bool {
        return true;
    }

    pub fn visitNodes(self: *CollectVisitor, _: Vec4, _: Vec4, _: Vec4, _: Vec4, _: Vec4, _: Vec4, _: *UVec4, _: i32) i32 {
        self.num_nodes += 1;
        return 4;
    }

    pub fn visitTriangles(self: *CollectVisitor, ctx: *const TestTriangleCodec.DecodingContext, triangles: *const anyopaque, num_triangles: u32, triangle_block_id: u32) void {
        var vertices: [3 * 16]Vec3 = undefined;
        var flags: [16]u8 = undefined;
        ctx.unpackWithFlags(triangles, num_triangles, &vertices, &flags);
        for (0..num_triangles) |i| {
            const t = ctx.getTriangle(triangles, @intCast(i));
            if (!t.v1.eql(vertices[3 * i]) or !t.v2.eql(vertices[3 * i + 1]) or !t.v3.eql(vertices[3 * i + 2]) or flags[i] != TestTriangleCodec.DecodingContext.getTriangleFlags(triangles, @intCast(i)))
                self.decode_mismatch = true;
            self.triangles.append(self.allocator, .{ vertices[3 * i], vertices[3 * i + 1], vertices[3 * i + 2] }) catch @panic("OOM");
            self.flags.append(self.allocator, flags[i]) catch @panic("OOM");
            self.user_data.append(self.allocator, ctx.getUserData(triangles, @intCast(i))) catch @panic("OOM");
        }
        self.block_ids.append(self.allocator, triangle_block_id) catch @panic("OOM");
    }
};

/// Visitor that casts a ray (like MeshShape::CastRay)
const RayCastVisitor = struct {
    ray_origin: Vec3,
    ray_direction: Vec3,
    ray_inv_direction: RayInvDirection,
    fraction: f32 = math.flt_max,
    triangle_block_id: u32 = 0,
    triangle_idx: u32 = 0,
    distance_stack: [TestNodeCodec.stack_size]f32 = undefined,

    pub fn shouldAbort(self: *const RayCastVisitor) bool {
        return self.fraction <= 0.0;
    }

    pub fn shouldVisitNode(self: *const RayCastVisitor, stack_top: i32) bool {
        return self.distance_stack[@intCast(stack_top)] < self.fraction;
    }

    pub fn visitNodes(self: *RayCastVisitor, min_x: Vec4, min_y: Vec4, min_z: Vec4, max_x: Vec4, max_y: Vec4, max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
        // Test bounds of 4 children
        const distance = rayAABox4(self.ray_origin, self.ray_inv_direction, min_x, min_y, min_z, max_x, max_y, max_z);

        // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
        return sortReverseAndStore(distance, self.fraction, properties, self.distance_stack[@intCast(stack_top)..][0..4]);
    }

    pub fn visitTriangles(self: *RayCastVisitor, ctx: *const TestTriangleCodec.DecodingContext, triangles: *const anyopaque, num_triangles: u32, triangle_block_id: u32) void {
        const hit = ctx.testRay(self.ray_origin, self.ray_direction, triangles, num_triangles, self.fraction);
        if (hit.closest < self.fraction) {
            self.fraction = hit.closest;
            self.triangle_block_id = triangle_block_id;
            self.triangle_idx = hit.closest_triangle_index;
        }
    }
};

/// Build a tree with the splitter, convert it and check the result
fn buildAndConvert(allocator: std.mem.Allocator, mesh: *const TestMesh, splitter: *TriangleSplitter, max_triangles_per_leaf: u32, store_user_data: bool) !TestBuffer {
    var builder: AABBTreeBuilder = .init(splitter, .{ .max_triangles_per_leaf = max_triangles_per_leaf });
    defer builder.deinit(allocator);
    var stats: AABBTreeBuilderStats = .{};
    const root = try builder.build(allocator, &stats);

    var buffer: TestBuffer = .empty;
    errdefer buffer.deinit(allocator);
    try buffer.convert(allocator, builder.getTriangles(), builder.getNodes(), mesh.vertices.items, root, store_user_data);

    // The buffer was allocated in one go
    try std.testing.expectEqual(buffer.tree.vector.capacity, buffer.tree.vector.items.len);

    // Header
    const header = buffer.getNodeHeader();
    try std.testing.expect(Vec3.fromFloat3(header.root_bounds_min).eql(root.bounds.min));
    try std.testing.expect(Vec3.fromFloat3(header.root_bounds_max).eql(root.bounds.max));
    try std.testing.expectEqual([3]u8{ 0, 0, 0 }, header.padding);
    try std.testing.expectEqual(@intFromPtr(buffer.tree.vector.items.ptr) + TestBuffer.header_size + TestBuffer.triangle_header_size, @intFromPtr(buffer.getRoot()));
    if (root.hasChildren())
        try std.testing.expectEqual(@as(u32, (TestBuffer.header_size + TestBuffer.triangle_header_size) >> 2), header.root_properties);

    return buffer;
}

/// Walk the tree and check that it contains every triangle of the mesh once, cast rays and compare with brute force
fn checkBuffer(allocator: std.mem.Allocator, buffer: *const TestBuffer, mesh: *const TestMesh, store_user_data: bool) !void {
    const buffer_start = buffer.tree.vector.items.ptr;
    const triangle_ctx: TestTriangleCodec.DecodingContext = .init(buffer.getTriangleHeader());

    // Collect all triangles
    var collect: CollectVisitor = .{ .allocator = allocator };
    defer collect.deinit();
    var node_ctx: TestNodeCodec.DecodingContext = .init(buffer.getNodeHeader());
    node_ctx.walkTree(buffer_start, &triangle_ctx, &collect);
    try std.testing.expect(node_ctx.isDoneWalking());
    try std.testing.expectEqual(mesh.triangles.items.len, collect.triangles.items.len);
    try std.testing.expect(!collect.decode_mismatch); // getTriangle / getTriangleFlags give the same as unpack / getFlags
    try std.testing.expect(collect.num_nodes > 0 or mesh.triangles.items.len <= 8);

    // Block IDs fit in the number of bits in the header
    const block_id_bits = TestNodeCodec.DecodingContext.triangleBlockIDBits(buffer.getNodeHeader());
    for (collect.block_ids.items) |id|
        try std.testing.expect(block_id_bits == 32 or id < (@as(u32, 1) << @intCast(block_id_bits)));

    // Every triangle was stored once with its flags and (optional) user data, the vertices survive compression
    const seen = try allocator.alloc(bool, mesh.triangles.items.len);
    defer allocator.free(seen);
    @memset(seen, false);
    for (collect.triangles.items, collect.flags.items, collect.user_data.items, 0..) |t, flags, user_data, i| {
        const index = if (store_user_data) user_data else blk: {
            try std.testing.expectEqual(@as(u32, 0), user_data);
            // Find the triangle by its vertices
            for (mesh.triangles.items, 0..) |mt, j| {
                if (!seen[j] and mt.material_index == flags and Vec3.fromFloat3(mesh.vertices.items[mt.idx[0]]).isClose(t[0], .{ .max_dist_sq = 1.0e-8 }) and
                    Vec3.fromFloat3(mesh.vertices.items[mt.idx[1]]).isClose(t[1], .{ .max_dist_sq = 1.0e-8 }) and Vec3.fromFloat3(mesh.vertices.items[mt.idx[2]]).isClose(t[2], .{ .max_dist_sq = 1.0e-8 }))
                    break :blk @as(u32, @intCast(j));
            }
            std.debug.print("triangle {d} not found\n", .{i});
            return error.TestUnexpectedResult;
        };
        try std.testing.expect(!seen[index]);
        seen[index] = true;
        const mt = mesh.triangles.items[index];
        try std.testing.expectEqual(@as(u8, @intCast(mt.material_index)), flags);
        for (0..3) |v|
            try std.testing.expect(Vec3.fromFloat3(mesh.vertices.items[mt.idx[v]]).isClose(t[v], .{ .max_dist_sq = 1.0e-8 }));
    }

    // Cast rays from above, compare with testing all triangles
    var rng: u32 = 12345;
    for (0..100) |_| {
        rng = rng *% 1664525 +% 1013904223;
        const x = @as(f32, @floatFromInt(rng >> 8)) / 16777216.0 * 20.0 - 2.0;
        rng = rng *% 1664525 +% 1013904223;
        const z = @as(f32, @floatFromInt(rng >> 8)) / 16777216.0 * 20.0 - 2.0;
        const origin = Vec3.init(x, 5, z);
        const direction = Vec3.init(0.1, -10, -0.2);

        var ray: RayCastVisitor = .{ .ray_origin = origin, .ray_direction = direction, .ray_inv_direction = .init(direction) };
        var ray_ctx: TestNodeCodec.DecodingContext = .init(buffer.getNodeHeader());
        ray_ctx.walkTree(buffer_start, &triangle_ctx, &ray);

        var brute_force: f32 = math.flt_max;
        for (collect.triangles.items) |t|
            brute_force = math.min(brute_force, rayTriangle(origin, direction, t[0], t[1], t[2]));
        if (brute_force == math.flt_max) {
            try std.testing.expectEqual(math.flt_max, ray.fraction);
        } else {
            try std.testing.expectApproxEqAbs(brute_force, ray.fraction, 1.0e-5);

            // The reported triangle is hit at that fraction
            const hit_triangles = TestNodeCodec.DecodingContext.getTriangleBlockStart(buffer_start, ray.triangle_block_id);
            const t = triangle_ctx.getTriangle(hit_triangles, ray.triangle_idx);
            try std.testing.expectApproxEqAbs(ray.fraction, rayTriangle(origin, direction, t.v1, t.v2, t.v3), 1.0e-5);
        }
    }
}

test "AABBTreeToBuffer" {
    const allocator = std.testing.allocator;

    for ([_]u32{ 1, 4, 10 }) |n| {
        var mesh = try TestMesh.init(allocator, n);
        defer mesh.deinit(allocator);

        for ([_]u32{ 1, 4, 8, 14 }) |max_triangles_per_leaf| {
            for ([_]bool{ false, true }) |store_user_data| {
                for (0..2) |splitter_type| {
                    var binning = try TriangleSplitterBinning.init(allocator, mesh.vertices.items, mesh.triangles.items, .{});
                    defer binning.deinit(allocator);
                    var mean = try TriangleSplitterMean.init(allocator, mesh.vertices.items, mesh.triangles.items);
                    defer mean.deinit(allocator);
                    const splitter = if (splitter_type == 0) binning.splitter() else mean.splitter();

                    var buffer = try buildAndConvert(allocator, &mesh, splitter, max_triangles_per_leaf, store_user_data);
                    defer buffer.deinit(allocator);
                    try checkBuffer(allocator, &buffer, &mesh, store_user_data);
                }
            }
        }
    }
}

test "AABBTreeToBuffer root leaf and abort" {
    const allocator = std.testing.allocator;

    var mesh = try TestMesh.init(allocator, 2); // 8 triangles
    defer mesh.deinit(allocator);
    var mean = try TriangleSplitterMean.init(allocator, mesh.vertices.items, mesh.triangles.items);
    defer mean.deinit(allocator);

    // The whole mesh in the root
    var buffer = try buildAndConvert(allocator, &mesh, mean.splitter(), 8, true);
    defer buffer.deinit(allocator);
    try checkBuffer(allocator, &buffer, &mesh, true);
    const header = buffer.getNodeHeader();
    try std.testing.expectEqual(@as(u32, 8), header.root_properties >> TestNodeCodec.triangle_count_shift);
    try std.testing.expectEqual(@as(u32, (TestBuffer.header_size + TestBuffer.triangle_header_size) >> 2), header.root_properties & TestNodeCodec.offset_mask);

    // Take ownership of the resulting data (like MeshShape does with mTree.swap(buffer.GetBuffer()))
    const size = buffer.getBuffer().vector.items.len;
    var tree: ByteBuffer = .empty;
    defer tree.deinit(allocator);
    std.mem.swap(ByteBuffer, &tree, buffer.getBufferMut());
    try std.testing.expectEqual(size, tree.vector.items.len);
    try std.testing.expectEqual(@as(usize, 0), buffer.getBuffer().vector.items.len);

    // A visitor that collects at most `budget` triangles per walk and aborts when a triangle block doesn't fit, the
    // next walk continues with that block (like MeshShape::GetTrianglesNext)
    const BudgetVisitor = struct {
        budget: u32,
        num_found: u32 = 0,
        total_found: u32 = 0,
        should_abort: bool = false,

        pub fn shouldAbort(self: *const @This()) bool {
            return self.should_abort;
        }

        pub fn shouldVisitNode(_: *const @This(), _: i32) bool {
            return true;
        }

        pub fn visitNodes(_: *@This(), _: Vec4, _: Vec4, _: Vec4, _: Vec4, _: Vec4, _: Vec4, _: *UVec4, _: i32) i32 {
            return 4;
        }

        pub fn visitTriangles(self: *@This(), _: *const TestTriangleCodec.DecodingContext, _: *const anyopaque, num_triangles: u32, _: u32) void {
            if (self.num_found + num_triangles > self.budget) {
                self.should_abort = true;
                return;
            }
            self.num_found += num_triangles;
            self.total_found += num_triangles;
        }
    };

    var mesh2 = try TestMesh.init(allocator, 4); // 32 triangles
    defer mesh2.deinit(allocator);
    var binning = try TriangleSplitterBinning.init(allocator, mesh2.vertices.items, mesh2.triangles.items, .{});
    defer binning.deinit(allocator);
    var buffer2 = try buildAndConvert(allocator, &mesh2, binning.splitter(), 2, false);
    defer buffer2.deinit(allocator);
    const triangle_ctx: TestTriangleCodec.DecodingContext = .init(buffer2.getTriangleHeader());

    // A budget of 3 triangles: the walk aborts every time it finds a second block of 2 triangles
    var visitor: BudgetVisitor = .{ .budget = 3 };
    var ctx: TestNodeCodec.DecodingContext = .init(buffer2.getNodeHeader());
    var num_walks: u32 = 0;
    while (true) {
        visitor.num_found = 0;
        visitor.should_abort = false;
        ctx.walkTree(buffer2.tree.vector.items.ptr, &triangle_ctx, &visitor);
        num_walks += 1;
        try std.testing.expect(visitor.num_found > 0 and visitor.num_found <= 3);
        if (ctx.isDoneWalking())
            break;
        try std.testing.expect(visitor.should_abort);
    }
    try std.testing.expectEqual(@as(u32, 32), visitor.total_found);
    try std.testing.expectEqual(@as(u32, 16), num_walks);
}

test "AABBTreeToBuffer errors" {
    const allocator = std.testing.allocator;

    var mesh = try TestMesh.init(allocator, 4); // 32 triangles
    defer mesh.deinit(allocator);
    var mean = try TriangleSplitterMean.init(allocator, mesh.vertices.items, mesh.triangles.items);
    defer mean.deinit(allocator);

    // Too many triangles in a leaf (with a parent node)
    {
        var builder: AABBTreeBuilder = .init(mean.splitter(), .{ .max_triangles_per_leaf = 16 });
        defer builder.deinit(allocator);
        var stats: AABBTreeBuilderStats = .{};
        const root = try builder.build(allocator, &stats);
        var buffer: TestBuffer = .empty;
        defer buffer.deinit(allocator);
        const result = buffer.convert(allocator, builder.getTriangles(), builder.getNodes(), mesh.vertices.items, root, false);
        try std.testing.expectError(error.NodeCodecTooManyTriangles, result);
        try std.testing.expectEqualStrings("NodeCodecQuadTreeHalfFloat: Too many triangles", TestBuffer.errorMessage(error.NodeCodecTooManyTriangles));
    }

    // Too many triangles in the root
    {
        var builder: AABBTreeBuilder = .init(mean.splitter(), .{ .max_triangles_per_leaf = 32 });
        defer builder.deinit(allocator);
        var stats: AABBTreeBuilderStats = .{};
        const root = try builder.build(allocator, &stats);
        var buffer: TestBuffer = .empty;
        defer buffer.deinit(allocator);
        try std.testing.expectError(error.NodeCodecTooManyTriangles, buffer.convert(allocator, builder.getTriangles(), builder.getNodes(), mesh.vertices.items, root, false));
    }

    // Material index that doesn't fit in the flags
    {
        mesh.triangles.items[5].material_index = 1000;
        defer mesh.triangles.items[5].material_index = 5;
        var builder: AABBTreeBuilder = .init(mean.splitter(), .{ .max_triangles_per_leaf = 4 });
        defer builder.deinit(allocator);
        var stats: AABBTreeBuilderStats = .{};
        const root = try builder.build(allocator, &stats);
        var buffer: TestBuffer = .empty;
        defer buffer.deinit(allocator);
        try std.testing.expectError(error.TriangleCodecMaterialIndexTooLarge, buffer.convert(allocator, builder.getTriangles(), builder.getNodes(), mesh.vertices.items, root, false));
        try std.testing.expectEqualStrings("TriangleCodecIndexed8BitPackSOA4Flags: Material index doesn't fit in 8 bit", TestBuffer.errorMessage(error.TriangleCodecMaterialIndexTooLarge));
    }

    // Messages of the other errors
    try std.testing.expectEqualStrings("AABBTreeToBuffer: Out of memory!", TestBuffer.errorMessage(error.OutOfMemory));
    try std.testing.expectEqualStrings("Internal Error: Node memory estimate was incorrect, memory corruption!", TestBuffer.errorMessage(error.NodeMemoryEstimateIncorrect));
    try std.testing.expectEqualStrings("Internal Error: Tree memory estimate was incorrect, memory corruption!", TestBuffer.errorMessage(error.TreeMemoryEstimateIncorrect));
    try std.testing.expectEqualStrings("NodeCodecQuadTreeHalfFloat: Offset too large. Too much data.", TestBuffer.errorMessage(error.NodeCodecOffsetTooLarge));
    try std.testing.expectEqualStrings("TriangleCodecIndexed8BitPackSOA4Flags: Offset doesn't fit in 8 bit", TestBuffer.errorMessage(error.TriangleCodecVertexOffsetTooLarge));

    // Allocation failures don't leak
    var builder: AABBTreeBuilder = .init(mean.splitter(), .{ .max_triangles_per_leaf = 4 });
    defer builder.deinit(allocator);
    var stats: AABBTreeBuilderStats = .{};
    const root = try builder.build(allocator, &stats);
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var buffer: TestBuffer = .empty;
        defer buffer.deinit(failing.allocator());
        buffer.convert(failing.allocator(), builder.getTriangles(), builder.getNodes(), mesh.vertices.items, root, true) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        break;
    }
    try std.testing.expect(fail_index > 5);
}
