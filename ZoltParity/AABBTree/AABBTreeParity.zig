//! Parity tests for the triangle splitters and the AABB tree (TriangleSplitterBinning, TriangleSplitterMean,
//! AABBTreeBuilder, AABBTreeToBuffer with NodeCodecQuadTreeHalfFloat and TriangleCodecIndexed8BitPackSOA4Flags):
//! run Zolt and the C++ Jolt library on the same meshes and require identical bits. C ABI wrappers:
//! ZoltParity/AABBTree/AABBTreeReference.cpp. See ZoltParity/parity.zig for how parity tests work.
//!
//! Both sides write their results into a stream of u32 values (floats as bits) and the streams are compared:
//! - splitter: every Split call of a depth first run like AABBTreeBuilder's (ranges, success) and the final
//!   triangle order;
//! - builder: stats, all nodes (bounds bits, triangle ranges, children), the triangle order, and for the first nodes
//!   the results of the Node query functions (depths, counts, SAH cost, GetNChildren for N = 1..8);
//! - convert: ValidationContext::IsDegenerate for every triangle, the result / error message of Convert and every
//!   byte of the buffer;
//! - walk: the order of the nodes and triangle blocks that WalkTree visits with three visitors (visit everything and
//!   decode all triangles, flags and user data; ray casts like MeshShape::CastRay; box queries with a triangle budget
//!   that abort and resume the walk like MeshShape::GetTrianglesNext) and what they compute (bounds, properties,
//!   distances, TestRay hits).
//!
//! The meshes are random soups, height field grids, planar meshes, meshes with degenerate and duplicate triangles,
//! integer coordinates (exact ties), far apart clusters, long strips, tiny meshes (smaller than the binning
//! splitter's minimum size), huge coordinates (beyond the half float range of the node bounds), unshared vertices,
//! and meshes that make Convert fail (too many triangles per leaf, material indices that don't fit in 8 bits, too
//! many vertices in the root leaf). Both splitters, random binning options, leaf sizes and with / without user data.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const AABBTreeBuilder = zolt.AABBTreeBuilder;
const AABBTreeBuilderStats = zolt.AABBTreeBuilderStats;
const AABox = zolt.AABox;
const Float3 = zolt.Float3;
const IndexedTriangle = zolt.IndexedTriangle;
const RayInvDirection = zolt.RayInvDirection;
const TriangleSplitter = zolt.TriangleSplitter;
const TriangleSplitterBinning = zolt.TriangleSplitterBinning;
const TriangleSplitterMean = zolt.TriangleSplitterMean;
const UVec4 = zolt.UVec4;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

const NodeCodec = zolt.NodeCodecQuadTreeHalfFloat;
const TriangleCodec = zolt.TriangleCodecIndexed8BitPackSOA4Flags;
const Buffer = zolt.AABBTreeToBuffer(TriangleCodec, NodeCodec);
const Range = TriangleSplitter.Range;

/// The C++ reference functions, see AABBTreeReference.cpp
const jolt = struct {
    extern fn jolt_aabb_splitter_run(input: *const Input, leaf_size: u32, out_stream: [*]u32, capacity: u32) u32;
    extern fn jolt_aabb_tree_build(input: *const Input, num_query_nodes: u32, cost_traversal: f32, cost_leaf: f32, out_stream: [*]u32, capacity: u32) u32;
    extern fn jolt_aabb_tree_convert(input: *const Input, out_stream: [*]u32, capacity: u32) u32;
    extern fn jolt_aabb_tree_walk(input: *const Input, rays: [*]const f32, num_rays: u32, boxes: [*]const f32, max_triangles: [*]const u32, num_boxes: u32, out_stream: [*]u32, capacity: u32) u32;
};

/// Input of the C++ functions, must match ParityInput in AABBTreeReference.cpp
const Input = extern struct {
    vertices: [*]const Float3,
    num_vertices: u32,
    triangles: [*]const IndexedTriangle,
    num_triangles: u32,
    /// 0 = TriangleSplitterBinning, 1 = TriangleSplitterMean
    splitter_type: u32,
    min_num_bins: u32,
    max_num_bins: u32,
    num_triangles_per_bin: u32,
    max_triangles_per_leaf: u32,
    store_user_data: u32,
};

// Section markers in the streams, must match AABBTreeReference.cpp
const marker_node: u32 = 0xA0000001;
const marker_triangles: u32 = 0xA0000002;
const marker_walk_done: u32 = 0xA0000003;
const marker_abort: u32 = 0xA0000004;
const marker_all_visitor: u32 = 0xA0000010;
const marker_ray_visitor: u32 = 0xA0000011;
const marker_box_visitor: u32 = 0xA0000012;
const marker_no_progress: u32 = 0xA0000013;

/// Number of nodes for which the builder test compares the Node query functions
const num_query_nodes = 40;

/// Output stream of u32 values (see AABBTreeReference.cpp)
const Stream = struct {
    allocator: std.mem.Allocator,
    values: std.ArrayList(u32) = .empty,

    fn deinit(self: *Stream) void {
        self.values.deinit(self.allocator);
    }

    fn add(self: *Stream, value: u32) void {
        self.values.append(self.allocator, value) catch @panic("OOM");
    }

    fn addFloat(self: *Stream, value: f32) void {
        self.add(@bitCast(value));
    }

    fn addVec3(self: *Stream, value: Vec3) void {
        self.addFloat(value.getX());
        self.addFloat(value.getY());
        self.addFloat(value.getZ());
    }

    fn addVec4(self: *Stream, value: Vec4) void {
        self.addFloat(value.getX());
        self.addFloat(value.getY());
        self.addFloat(value.getZ());
        self.addFloat(value.getW());
    }

    fn addUVec4(self: *Stream, value: UVec4) void {
        self.add(value.getX());
        self.add(value.getY());
        self.add(value.getZ());
        self.add(value.getW());
    }

    fn addString(self: *Stream, string: ?[]const u8) void {
        const s = string orelse {
            self.add(0xffffffff);
            return;
        };
        self.add(@intCast(s.len));
        for (s) |c| self.add(c);
    }
};

/// Run a C++ reference function that writes a stream, retrying with a bigger buffer when needed
fn joltStream(allocator: std.mem.Allocator, context: anytype, comptime function: fn (@TypeOf(context), [*]u32, u32) u32) ![]u32 {
    var capacity: u32 = 1 << 16;
    while (true) {
        const values = try allocator.alloc(u32, capacity);
        const size = function(context, values.ptr, capacity);
        if (size <= capacity)
            return try allocator.realloc(values, size);
        allocator.free(values);
        capacity = size;
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Zolt side, mirrors AABBTreeReference.cpp

/// Holds the splitter selected by the input
const SplitterHolder = union(enum) {
    binning: TriangleSplitterBinning,
    mean: TriangleSplitterMean,

    fn init(allocator: std.mem.Allocator, input: *const Input) !SplitterHolder {
        const vertices = input.vertices[0..input.num_vertices];
        const triangles = input.triangles[0..input.num_triangles];
        if (input.splitter_type == 0)
            return .{ .binning = try .init(allocator, vertices, triangles, .{ .min_num_bins = input.min_num_bins, .max_num_bins = input.max_num_bins, .num_triangles_per_bin = input.num_triangles_per_bin }) }
        else
            return .{ .mean = try .init(allocator, vertices, triangles) };
    }

    fn splitter(self: *SplitterHolder) *TriangleSplitter {
        return switch (self.*) {
            .binning => |*b| b.splitter(),
            .mean => |*m| m.splitter(),
        };
    }

    fn deinit(self: *SplitterHolder, allocator: std.mem.Allocator) void {
        self.splitter().deinit(allocator); // Virtual destructor
    }
};

fn zoltSplitterRun(allocator: std.mem.Allocator, input: *const Input, leaf_size: u32, s: *Stream) !void {
    var holder = try SplitterHolder.init(allocator, input);
    defer holder.deinit(allocator);
    const splitter = holder.splitter();

    var stats: TriangleSplitter.Stats = .{};
    splitter.getStats(&stats);
    s.addString(stats.splitter_name);
    s.add(@bitCast(stats.leaf_size));

    const initial = splitter.getInitialRange();
    s.add(initial.begin);
    s.add(initial.end);

    var stack: std.ArrayList(Range) = .empty;
    defer stack.deinit(allocator);
    try stack.append(allocator, initial);
    while (stack.pop()) |range| {
        if (range.count() <= leaf_size)
            continue;

        var left: Range = undefined;
        var right: Range = undefined;
        var success = false;
        if (splitter.split(range)) |result| {
            left = result.left;
            right = result.right;
            success = true;
        } else {
            const half = range.count() / 2;
            left = .init(range.begin, range.begin + half);
            right = .init(range.begin + half, range.end);
        }
        s.add(range.begin);
        s.add(range.end);
        s.add(@intFromBool(success));
        s.add(left.begin);
        s.add(left.end);
        s.add(right.begin);
        s.add(right.end);

        try stack.append(allocator, right);
        try stack.append(allocator, left);
    }

    for (0..input.num_triangles) |i|
        s.add(@intCast((@intFromPtr(splitter.getTriangle(@intCast(i))) - @intFromPtr(input.triangles)) / @sizeOf(IndexedTriangle)));
}

fn nodeIndex(node: *const AABBTreeBuilder.Node, nodes: []const AABBTreeBuilder.Node) u32 {
    return @intCast((@intFromPtr(node) - @intFromPtr(nodes.ptr)) / @sizeOf(AABBTreeBuilder.Node));
}

fn zoltBuild(allocator: std.mem.Allocator, input: *const Input, cost_traversal: f32, cost_leaf: f32, s: *Stream) !void {
    var holder = try SplitterHolder.init(allocator, input);
    defer holder.deinit(allocator);
    var builder: AABBTreeBuilder = .init(holder.splitter(), .{ .max_triangles_per_leaf = input.max_triangles_per_leaf });
    defer builder.deinit(allocator);
    var stats: AABBTreeBuilderStats = .{};
    const root = try builder.build(allocator, &stats);
    const nodes = builder.getNodes();

    // Stats
    s.addString(stats.splitter_stats.splitter_name);
    s.add(@bitCast(stats.splitter_stats.leaf_size));
    s.addFloat(stats.sah_cost);
    s.add(@bitCast(stats.min_depth));
    s.add(@bitCast(stats.max_depth));
    s.add(@bitCast(stats.node_count));
    s.add(@bitCast(stats.leaf_node_count));
    s.add(@bitCast(stats.max_triangles_per_leaf));
    s.add(@bitCast(stats.tree_min_triangles_per_leaf));
    s.add(@bitCast(stats.tree_max_triangles_per_leaf));
    s.addFloat(stats.tree_avg_triangles_per_leaf);

    // Nodes
    s.add(nodeIndex(root, nodes));
    s.add(@intCast(nodes.len));
    for (nodes) |*n| {
        s.addVec3(n.bounds.min);
        s.addVec3(n.bounds.max);
        s.add(if (n.hasChildren()) 0xffffffff else n.triangles_begin); // Not initialized for nodes with children
        s.add(n.num_triangles);
        s.add(n.child[0]);
        s.add(n.child[1]);
        s.add(n.getTriangleCount());
        s.add(@intFromBool(n.hasChildren()));
    }

    // Triangles
    const triangles = builder.getTriangles();
    s.add(@intCast(triangles.len));
    for (triangles) |t| {
        s.add(t.idx[0]);
        s.add(t.idx[1]);
        s.add(t.idx[2]);
        s.add(t.material_index);
        s.add(t.user_data);
    }

    // Node queries
    var children: std.ArrayList(*const AABBTreeBuilder.Node) = .empty;
    defer children.deinit(allocator);
    for (nodes[0..@min(num_query_nodes, nodes.len)]) |*n| {
        s.add(n.getMinDepth(nodes));
        s.add(n.getMaxDepth(nodes));
        s.add(n.getNodeCount(nodes));
        s.add(n.getLeafNodeCount(nodes));
        s.add(n.getTriangleCountInTree(nodes));
        const per_node = n.getTriangleCountPerNode(nodes);
        s.addFloat(per_node.average);
        s.add(per_node.min);
        s.add(per_node.max);
        s.addFloat(n.calculateSAHCost(nodes, 1.0, 1.0));
        s.addFloat(n.calculateSAHCost(nodes, cost_traversal, cost_leaf));
        for (0..2) |c| {
            const child = n.getChild(@intCast(c), nodes);
            s.add(if (child) |ch| nodeIndex(ch, nodes) else 0xffffffff);
        }
        for (1..9) |num| {
            children.clearRetainingCapacity();
            try n.getNChildren(allocator, nodes, @intCast(num), &children);
            s.add(@intCast(children.items.len));
            for (children.items) |c|
                s.add(nodeIndex(c, nodes));
        }
    }
}

/// Builds and converts the tree of the input
const Tree = struct {
    holder: SplitterHolder,
    builder: AABBTreeBuilder,
    root: *const AABBTreeBuilder.Node,
    buffer: Buffer = .empty,
    result: Buffer.Error!void,

    /// Initialize in place (the builder keeps a pointer to the splitter)
    fn init(self: *Tree, allocator: std.mem.Allocator, input: *const Input) !void {
        self.holder = try SplitterHolder.init(allocator, input);
        errdefer self.holder.deinit(allocator);
        self.builder = .init(self.holder.splitter(), .{ .max_triangles_per_leaf = input.max_triangles_per_leaf });
        errdefer self.builder.deinit(allocator);
        var stats: AABBTreeBuilderStats = .{};
        self.root = try self.builder.build(allocator, &stats);
        self.buffer = .empty;
        self.result = self.buffer.convert(allocator, self.builder.getTriangles(), self.builder.getNodes(), input.vertices[0..input.num_vertices], self.root, input.store_user_data != 0);
        if (self.result) |_| {} else |err| if (err == error.OutOfMemory) return error.OutOfMemory;
    }

    fn deinit(self: *Tree, allocator: std.mem.Allocator) void {
        self.buffer.deinit(allocator);
        self.builder.deinit(allocator);
        self.holder.deinit(allocator);
    }
};

fn zoltConvert(allocator: std.mem.Allocator, input: *const Input, s: *Stream) !void {
    var tree: Tree = undefined;
    try tree.init(allocator, input);
    defer tree.deinit(allocator);

    const vertices = input.vertices[0..input.num_vertices];
    const triangles = input.triangles[0..input.num_triangles];
    const validation: TriangleCodec.ValidationContext = .init(triangles, vertices);
    for (triangles) |t|
        s.add(@intFromBool(validation.isDegenerate(t)));

    tree.result catch |err| {
        s.add(0);
        s.addString(Buffer.errorMessage(err));
        return;
    };
    s.add(1);

    const bytes = tree.buffer.getBuffer().vector.items;
    s.add(@intCast(bytes.len));
    var i: usize = 0;
    while (i < bytes.len) : (i += 4) {
        var value: u32 = 0;
        var b: usize = 0;
        while (b < 4 and i + b < bytes.len) : (b += 1)
            value |= @as(u32, bytes[i + b]) << @intCast(8 * b);
        s.add(value);
    }

    // Offsets of the headers and the root
    const start = @intFromPtr(bytes.ptr);
    s.add(@intCast(@intFromPtr(tree.buffer.getNodeHeader()) - start));
    s.add(@intCast(@intFromPtr(tree.buffer.getTriangleHeader()) - start));
    s.add(@intCast(@intFromPtr(tree.buffer.getRoot()) - start));
}

/// Visits all nodes and decodes all triangles
const AllVisitor = struct {
    stream: *Stream,

    pub fn shouldAbort(_: *const AllVisitor) bool {
        return false;
    }

    pub fn shouldVisitNode(_: *const AllVisitor, _: i32) bool {
        return true;
    }

    pub fn visitNodes(self: *AllVisitor, min_x: Vec4, min_y: Vec4, min_z: Vec4, max_x: Vec4, max_y: Vec4, max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
        const s = self.stream;
        s.add(marker_node);
        s.add(@bitCast(stack_top));
        s.addVec4(min_x);
        s.addVec4(min_y);
        s.addVec4(min_z);
        s.addVec4(max_x);
        s.addVec4(max_y);
        s.addVec4(max_z);
        s.addUVec4(properties.*);
        return 4;
    }

    pub fn visitTriangles(self: *AllVisitor, ctx: *const TriangleCodec.DecodingContext, triangles: *const anyopaque, num_triangles: u32, triangle_block_id: u32) void {
        const s = self.stream;
        s.add(marker_triangles);
        s.add(triangle_block_id);
        s.add(num_triangles);

        var vertices: [16 * 3]Vec3 = undefined;
        var flags: [16]u8 = undefined;
        ctx.unpackWithFlags(triangles, num_triangles, &vertices, &flags);
        for (vertices[0 .. 3 * num_triangles]) |v|
            s.addVec3(v);
        for (flags[0..num_triangles]) |f|
            s.add(f);

        for (0..num_triangles) |i| {
            const t = ctx.getTriangle(triangles, @intCast(i));
            s.addVec3(t.v1);
            s.addVec3(t.v2);
            s.addVec3(t.v3);
            s.add(ctx.getUserData(triangles, @intCast(i)));
            s.add(TriangleCodec.DecodingContext.getTriangleFlags(triangles, @intCast(i)));
        }
    }
};

/// Port of SortReverseAndStore (Physics/Collision/SortReverseAndStore.h, not ported yet)
fn sortReverseAndStore(values_in: Vec4, max_value: f32, identifiers: *UVec4, out_values: []f32) i32 {
    // Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
    var values = values_in;
    Vec4.sort4Reverse(&values, identifiers);

    // Count how many results are less than the max value
    const closer = Vec4.less(values, Vec4.replicate(max_value));
    const num_results = closer.countTrues();

    // Shift the values so that only the ones that are less than max are kept
    values = values.reinterpretAsInt().shiftComponents4Minus(num_results).reinterpretAsFloat();
    identifiers.* = identifiers.shiftComponents4Minus(num_results);

    // Store the values
    out_values[0] = values.getX();
    out_values[1] = values.getY();
    out_values[2] = values.getZ();
    out_values[3] = values.getW();

    return @intCast(num_results);
}

/// Port of CountAndSortTrues (Physics/Collision/SortReverseAndStore.h, not ported yet)
fn countAndSortTrues(value: UVec4, identifiers: *UVec4) i32 {
    // Sort the hits
    identifiers.* = UVec4.sort4True(value, identifiers.*);

    // Return the amount of hits
    return @intCast(value.countTrues());
}

/// Casts a ray (like MeshShape::CastRay)
const RayVisitor = struct {
    stream: *Stream,
    ray_origin: Vec3,
    ray_direction: Vec3,
    ray_inv_direction: RayInvDirection,
    fraction: f32,
    triangle_block_id: u32 = 0xffffffff,
    triangle_idx: u32 = 0xffffffff,
    distance_stack: [NodeCodec.stack_size]f32 = undefined,

    pub fn shouldAbort(self: *const RayVisitor) bool {
        return self.fraction <= 0.0;
    }

    pub fn shouldVisitNode(self: *const RayVisitor, stack_top: i32) bool {
        return self.distance_stack[@intCast(stack_top)] < self.fraction;
    }

    pub fn visitNodes(self: *RayVisitor, min_x: Vec4, min_y: Vec4, min_z: Vec4, max_x: Vec4, max_y: Vec4, max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
        const distance = zolt.rayAABox4(self.ray_origin, self.ray_inv_direction, min_x, min_y, min_z, max_x, max_y, max_z);
        const top: usize = @intCast(stack_top);
        const num_results = sortReverseAndStore(distance, self.fraction, properties, self.distance_stack[top..][0..4]);

        const s = self.stream;
        s.add(marker_node);
        s.add(@bitCast(stack_top));
        s.addVec4(distance);
        s.addUVec4(properties.*);
        s.add(@bitCast(num_results));
        for (self.distance_stack[top..][0..4]) |d|
            s.addFloat(d);
        return num_results;
    }

    pub fn visitTriangles(self: *RayVisitor, ctx: *const TriangleCodec.DecodingContext, triangles: *const anyopaque, num_triangles: u32, triangle_block_id: u32) void {
        const hit = ctx.testRay(self.ray_origin, self.ray_direction, triangles, num_triangles, self.fraction);

        const s = self.stream;
        s.add(marker_triangles);
        s.add(triangle_block_id);
        s.add(num_triangles);
        s.addFloat(hit.closest);
        s.add(hit.closest_triangle_index);

        if (hit.closest < self.fraction) {
            self.fraction = hit.closest;
            self.triangle_block_id = triangle_block_id;
            self.triangle_idx = hit.closest_triangle_index;
        }
    }
};

/// Collects the triangles that overlap with a box, at most max_triangles per walk (like MeshShape::GetTrianglesNext)
const BoxVisitor = struct {
    stream: *Stream,
    box: AABox,
    max_triangles: i32,
    num_triangles_found: i32 = 0,
    should_abort: bool = false,

    pub fn shouldAbort(self: *const BoxVisitor) bool {
        return self.should_abort;
    }

    pub fn shouldVisitNode(_: *const BoxVisitor, _: i32) bool {
        return true;
    }

    pub fn visitNodes(self: *BoxVisitor, min_x: Vec4, min_y: Vec4, min_z: Vec4, max_x: Vec4, max_y: Vec4, max_z: Vec4, properties: *UVec4, stack_top: i32) i32 {
        const collides = zolt.aabox4VsBox(self.box, min_x, min_y, min_z, max_x, max_y, max_z);
        const num_results = countAndSortTrues(collides, properties);

        const s = self.stream;
        s.add(marker_node);
        s.add(@bitCast(stack_top));
        s.addUVec4(collides);
        s.addUVec4(properties.*);
        s.add(@bitCast(num_results));
        return num_results;
    }

    pub fn visitTriangles(self: *BoxVisitor, ctx: *const TriangleCodec.DecodingContext, triangles: *const anyopaque, num_triangles: u32, triangle_block_id: u32) void {
        const s = self.stream;

        // When the buffer is full abort the walk, the next walk continues here
        if (self.num_triangles_found + @as(i32, @intCast(num_triangles)) > self.max_triangles) {
            s.add(marker_abort);
            self.should_abort = true;
            return;
        }

        var vertices: [16 * 3]Vec3 = undefined;
        ctx.unpack(triangles, num_triangles, &vertices);

        s.add(marker_triangles);
        s.add(triangle_block_id);
        s.add(num_triangles);
        for (vertices[0 .. 3 * num_triangles]) |v|
            s.addVec3(v);

        self.num_triangles_found += @intCast(num_triangles);
    }
};

fn zoltWalk(allocator: std.mem.Allocator, input: *const Input, rays: []const [7]f32, boxes: []const [6]f32, max_triangles: []const u32, s: *Stream) !void {
    var tree: Tree = undefined;
    try tree.init(allocator, input);
    defer tree.deinit(allocator);
    tree.result catch return;

    const header = tree.buffer.getNodeHeader();
    const buffer_start = tree.buffer.getBuffer().vector.items.ptr;
    const triangle_ctx: TriangleCodec.DecodingContext = .init(tree.buffer.getTriangleHeader());
    s.add(NodeCodec.DecodingContext.triangleBlockIDBits(header));

    // Visit everything
    {
        s.add(marker_all_visitor);
        var visitor: AllVisitor = .{ .stream = s };
        var ctx: NodeCodec.DecodingContext = .init(header);
        ctx.walkTree(buffer_start, &triangle_ctx, &visitor);
        s.add(marker_walk_done);
        s.add(@intFromBool(ctx.isDoneWalking()));
    }

    // Ray casts
    for (rays) |ray| {
        s.add(marker_ray_visitor);
        const direction = Vec3.init(ray[3], ray[4], ray[5]);
        var visitor: RayVisitor = .{ .stream = s, .ray_origin = .init(ray[0], ray[1], ray[2]), .ray_direction = direction, .ray_inv_direction = .init(direction), .fraction = ray[6] };
        var ctx: NodeCodec.DecodingContext = .init(header);
        ctx.walkTree(buffer_start, &triangle_ctx, &visitor);
        s.add(marker_walk_done);
        s.add(@intFromBool(ctx.isDoneWalking()));
        s.addFloat(visitor.fraction);
        s.add(visitor.triangle_block_id);
        s.add(visitor.triangle_idx);
        if (visitor.triangle_block_id != 0xffffffff) {
            // Decode the triangle that was hit
            const t = triangle_ctx.getTriangle(NodeCodec.DecodingContext.getTriangleBlockStart(buffer_start, visitor.triangle_block_id), visitor.triangle_idx);
            s.addVec3(t.v1);
            s.addVec3(t.v2);
            s.addVec3(t.v3);
        }
    }

    // Box queries
    for (boxes, max_triangles) |box, max| {
        s.add(marker_box_visitor);
        var visitor: BoxVisitor = .{ .stream = s, .box = .init(.init(box[0], box[1], box[2]), .init(box[3], box[4], box[5])), .max_triangles = @intCast(max) };
        var ctx: NodeCodec.DecodingContext = .init(header);
        while (true) {
            visitor.num_triangles_found = 0;
            visitor.should_abort = false;
            ctx.walkTree(buffer_start, &triangle_ctx, &visitor);
            s.add(marker_walk_done);
            s.add(@bitCast(visitor.num_triangles_found));
            s.add(@intFromBool(ctx.isDoneWalking()));
            if (ctx.isDoneWalking())
                break;
            if (visitor.num_triangles_found == 0) {
                // The budget is smaller than a triangle block
                s.add(marker_no_progress);
                break;
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Input generation

/// Input generator: xorshift32 with helpers
const Gen = struct {
    rng: fw.Rng = .{},

    fn next(self: *Gen) u32 {
        return self.rng.next();
    }

    /// Random integer in [0, n)
    fn index(self: *Gen, n: usize) u32 {
        return @intCast(self.next() % n);
    }

    /// Random integer in [min, max]
    fn range(self: *Gen, min: u32, max: u32) u32 {
        return min + self.index(max - min + 1);
    }

    /// True one in n times
    fn oneIn(self: *Gen, n: u32) bool {
        return self.next() % n == 0;
    }

    fn float(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn vec(self: *Gen, min: f32, max: f32) [3]f32 {
        return .{ self.float(min, max), self.float(min, max), self.float(min, max) };
    }

    fn pick(self: *Gen, comptime T: type, values: []const T) T {
        return values[self.index(values.len)];
    }
};

/// A generated test mesh
const Mesh = struct {
    kind: []const u8 = "",
    vertices: std.ArrayList(Float3) = .empty,
    triangles: std.ArrayList(IndexedTriangle) = .empty,

    fn deinit(self: *Mesh, allocator: std.mem.Allocator) void {
        self.vertices.deinit(allocator);
        self.triangles.deinit(allocator);
    }

    fn addVertex(self: *Mesh, allocator: std.mem.Allocator, v: [3]f32) !void {
        try self.vertices.append(allocator, .init(v[0], v[1], v[2]));
    }

    fn addTriangle(self: *Mesh, allocator: std.mem.Allocator, gen: *Gen, a: u32, b: u32, c: u32) !void {
        try self.triangles.append(allocator, .init(a, b, c, .{ .material_index = gen.index(256), .user_data = gen.next() }));
    }

    /// Triangles with random indices
    fn addRandomTriangles(self: *Mesh, allocator: std.mem.Allocator, gen: *Gen, count: u32) !void {
        const nv: u32 = @intCast(self.vertices.items.len);
        for (0..count) |_|
            try self.addTriangle(allocator, gen, gen.index(nv), gen.index(nv), gen.index(nv));
    }

    /// Height field grid of w x h cells
    fn addGrid(self: *Mesh, allocator: std.mem.Allocator, gen: *Gen, w: u32, h: u32, cell: f32, height: f32, offset: [3]f32) !void {
        const first: u32 = @intCast(self.vertices.items.len);
        for (0..h + 1) |z|
            for (0..w + 1) |x|
                try self.addVertex(allocator, .{ offset[0] + @as(f32, @floatFromInt(x)) * cell, offset[1] + gen.float(0, height), offset[2] + @as(f32, @floatFromInt(z)) * cell });
        for (0..h) |z|
            for (0..w) |x| {
                const v: u32 = first + @as(u32, @intCast(z * (w + 1) + x));
                try self.addTriangle(allocator, gen, v, v + w + 1, v + 1);
                try self.addTriangle(allocator, gen, v + 1, v + w + 1, v + w + 2);
            };
    }

    /// Shuffle the triangles (Fisher-Yates)
    fn shuffle(self: *Mesh, gen: *Gen) void {
        const t = self.triangles.items;
        var i = t.len;
        while (i > 1) {
            i -= 1;
            std.mem.swap(IndexedTriangle, &t[i], &t[gen.index(i + 1)]);
        }
    }
};

const num_mesh_kinds = 14;

fn generateMesh(allocator: std.mem.Allocator, gen: *Gen, kind: u32, mesh: *Mesh) !void {
    switch (kind) {
        0 => {
            mesh.kind = "random soup";
            const s = gen.pick(f32, &.{ 1.0e-3, 1.0, 10.0, 1000.0 });
            for (0..gen.range(3, 300)) |_| try mesh.addVertex(allocator, gen.vec(-s, s));
            try mesh.addRandomTriangles(allocator, gen, gen.range(1, 600));
        },
        1 => {
            mesh.kind = "height field";
            try mesh.addGrid(allocator, gen, gen.range(1, 30), gen.range(1, 30), gen.pick(f32, &.{ 0.25, 1.0, 3.0 }), gen.pick(f32, &.{ 0.0, 0.5, 5.0 }), gen.vec(-100, 100));
            if (gen.oneIn(2)) mesh.shuffle(gen);
        },
        2 => {
            mesh.kind = "planar";
            const o = gen.vec(-10, 10);
            const axis_aligned = gen.oneIn(2);
            const u: [3]f32 = if (axis_aligned) .{ 1, 0, 0 } else gen.vec(-1, 1);
            const w: [3]f32 = if (axis_aligned) .{ 0, 0, 1 } else gen.vec(-1, 1);
            for (0..gen.range(3, 200)) |_| {
                const a = gen.float(-20, 20);
                const b = gen.float(-20, 20);
                try mesh.addVertex(allocator, .{ o[0] + a * u[0] + b * w[0], o[1] + a * u[1] + b * w[1], o[2] + a * u[2] + b * w[2] });
            }
            try mesh.addRandomTriangles(allocator, gen, gen.range(1, 400));
        },
        3 => {
            mesh.kind = "degenerate";
            for (0..gen.range(3, 100)) |_| try mesh.addVertex(allocator, gen.vec(-5, 5));
            const nv: u32 = @intCast(mesh.vertices.items.len);
            for (0..gen.range(1, 300)) |_| {
                const a = gen.index(nv);
                switch (gen.index(4)) {
                    0 => try mesh.addTriangle(allocator, gen, a, a, a), // A point
                    1 => try mesh.addTriangle(allocator, gen, a, a, gen.index(nv)), // A line
                    2 => {
                        // Colinear vertices
                        const p = mesh.vertices.items[a];
                        const d = gen.vec(-1, 1);
                        const first: u32 = @intCast(mesh.vertices.items.len);
                        try mesh.addVertex(allocator, .{ p.x + d[0], p.y + d[1], p.z + d[2] });
                        try mesh.addVertex(allocator, .{ p.x + 2 * d[0], p.y + 2 * d[1], p.z + 2 * d[2] });
                        try mesh.addTriangle(allocator, gen, a, first, first + 1);
                    },
                    else => try mesh.addTriangle(allocator, gen, a, gen.index(nv), gen.index(nv)),
                }
            }
        },
        4 => {
            mesh.kind = "duplicates";
            for (0..gen.range(3, 50)) |_| try mesh.addVertex(allocator, gen.vec(-5, 5));
            const nv: u32 = @intCast(mesh.vertices.items.len);
            for (0..gen.range(1, 10)) |_| {
                const t = [3]u32{ gen.index(nv), gen.index(nv), gen.index(nv) };
                for (0..gen.range(1, 60)) |_| {
                    if (gen.oneIn(3)) {
                        // Same positions, different vertex indices
                        const first: u32 = @intCast(mesh.vertices.items.len);
                        for (t) |i| {
                            const v = mesh.vertices.items[i];
                            try mesh.addVertex(allocator, .{ v.x, v.y, v.z });
                        }
                        try mesh.addTriangle(allocator, gen, first, first + 1, first + 2);
                    } else try mesh.addTriangle(allocator, gen, t[0], t[1], t[2]);
                }
            }
            mesh.shuffle(gen);
        },
        5 => {
            mesh.kind = "integer coordinates";
            const n: i32 = @intCast(gen.range(1, 6));
            for (0..gen.range(3, 150)) |_| {
                const v = [3]f32{ @floatFromInt(gen.rng.intRange(i32, -n, n)), @floatFromInt(gen.rng.intRange(i32, -n, n)), @floatFromInt(gen.rng.intRange(i32, -n, n)) };
                try mesh.addVertex(allocator, v);
            }
            try mesh.addRandomTriangles(allocator, gen, gen.range(1, 400));
        },
        6 => {
            mesh.kind = "clusters";
            for (0..gen.range(2, 6)) |_| {
                const center = gen.vec(-1.0e4, 1.0e4);
                const size = gen.pick(f32, &.{ 1.0e-3, 0.1, 1.0 });
                const first: u32 = @intCast(mesh.vertices.items.len);
                for (0..gen.range(3, 40)) |_| {
                    const d = gen.vec(-size, size);
                    try mesh.addVertex(allocator, .{ center[0] + d[0], center[1] + d[1], center[2] + d[2] });
                }
                const nv: u32 = @as(u32, @intCast(mesh.vertices.items.len)) - first;
                for (0..gen.range(1, 80)) |_|
                    try mesh.addTriangle(allocator, gen, first + gen.index(nv), first + gen.index(nv), first + gen.index(nv));
            }
            if (gen.oneIn(2)) mesh.shuffle(gen);
        },
        7 => {
            mesh.kind = "large";
            if (gen.oneIn(2)) {
                try mesh.addGrid(allocator, gen, gen.range(30, 45), gen.range(30, 45), 1.0, 2.0, .{ 0, 0, 0 });
                mesh.shuffle(gen);
            } else {
                for (0..gen.range(500, 2000)) |_| try mesh.addVertex(allocator, gen.vec(-50, 50));
                try mesh.addRandomTriangles(allocator, gen, gen.range(1000, 4000));
            }
        },
        8 => {
            mesh.kind = "unshared vertices";
            const s = gen.pick(f32, &.{ 1.0, 100.0 });
            for (0..gen.range(1, 500)) |_| {
                const c = gen.vec(-s, s);
                const first: u32 = @intCast(mesh.vertices.items.len);
                for (0..3) |_| {
                    const d = gen.vec(-1, 1);
                    try mesh.addVertex(allocator, .{ c[0] + d[0], c[1] + d[1], c[2] + d[2] });
                }
                try mesh.addTriangle(allocator, gen, first, first + 1, first + 2);
            }
        },
        9 => {
            mesh.kind = "tiny";
            const s = gen.pick(f32, &.{ 1.0e-7, 1.0e-6, 2.0e-6 });
            const o = gen.vec(-1, 1);
            for (0..gen.range(3, 60)) |_| {
                const d = gen.vec(-s, s);
                try mesh.addVertex(allocator, .{ o[0] + d[0], o[1] + d[1], o[2] + d[2] });
            }
            try mesh.addRandomTriangles(allocator, gen, gen.range(1, 120));
        },
        10 => {
            mesh.kind = "few triangles";
            for (0..gen.range(3, 12)) |_| try mesh.addVertex(allocator, gen.vec(-3, 3));
            try mesh.addRandomTriangles(allocator, gen, gen.range(1, 20));
        },
        11 => {
            mesh.kind = "strip";
            const n = gen.range(2, 300);
            const step = gen.pick(f32, &.{ 0.01, 1.0, 7.0 });
            for (0..n + 1) |i| {
                const x = @as(f32, @floatFromInt(i)) * step;
                try mesh.addVertex(allocator, .{ x, 0, 0 });
                try mesh.addVertex(allocator, .{ x, 1, gen.float(-0.1, 0.1) });
            }
            for (0..n) |i| {
                const v: u32 = @intCast(2 * i);
                try mesh.addTriangle(allocator, gen, v, v + 1, v + 2);
                try mesh.addTriangle(allocator, gen, v + 1, v + 3, v + 2);
            }
            if (gen.oneIn(2)) mesh.shuffle(gen);
        },
        12 => {
            mesh.kind = "huge coordinates";
            const o = gen.vec(-1.0e6, 1.0e6);
            const s = gen.pick(f32, &.{ 1.0, 1000.0, 1.0e5 });
            for (0..gen.range(3, 100)) |_| {
                const d = gen.vec(-s, s);
                try mesh.addVertex(allocator, .{ o[0] + d[0], o[1] + d[1], o[2] + d[2] });
            }
            try mesh.addRandomTriangles(allocator, gen, gen.range(1, 200));
        },
        else => {
            mesh.kind = "shared fan";
            // Many triangles around a few vertices with the same centroid along one axis
            const n = gen.range(3, 200);
            try mesh.addVertex(allocator, .{ 0, 0, 0 });
            for (0..n) |i| {
                const a = @as(f32, @floatFromInt(i)) * 0.1;
                try mesh.addVertex(allocator, .{ a, 1, -a });
            }
            for (1..n) |i|
                try mesh.addTriangle(allocator, gen, 0, @intCast(i), @intCast(i + 1));
        },
    }
}

/// Rays (origin, direction, initial fraction) for a mesh
fn generateRays(allocator: std.mem.Allocator, gen: *Gen, mesh: *const Mesh, rays: *std.ArrayList([7]f32)) !void {
    var bounds: AABox = .empty;
    for (mesh.vertices.items) |v| bounds.encapsulateVec3(.fromFloat3(v));
    const center = bounds.getCenter();
    const extent = Vec3.max(bounds.getExtent(), Vec3.replicate(1.0e-3)).mulScalar(1.5);
    const fractions = [_]f32{ 1.0, 1.0 + 1.0e-4, zolt.math.flt_max, 0.5, 0.0 };

    for (0..12) |_| {
        const origin = center.add(Vec3.init(gen.float(-1, 1), gen.float(-1, 1), gen.float(-1, 1)).mul(extent));
        var direction: Vec3 = undefined;
        switch (gen.index(6)) {
            0, 1, 2 => {
                // Towards a random triangle
                const t = mesh.triangles.items[gen.index(mesh.triangles.items.len)];
                const target = t.getCentroid(mesh.vertices.items);
                direction = target.sub(origin).mulScalar(gen.pick(f32, &.{ 1.0, 2.0, 0.5 }));
            },
            3 => {
                // Axis aligned
                const sign: f32 = if (gen.oneIn(2)) 1 else -1;
                direction = Vec3.zero();
                direction.setComponent(gen.index(3), sign * 4 * extent.getComponent(gen.index(3)));
            },
            4 => direction = Vec3.init(gen.float(-1, 1), gen.float(-1, 1), gen.float(-1, 1)).mul(extent).mulScalar(4),
            else => direction = Vec3.init(gen.float(-1, 1), 0, gen.float(-1, 1)).mul(extent).mulScalar(3),
        }
        try rays.append(allocator, .{ origin.getX(), origin.getY(), origin.getZ(), direction.getX(), direction.getY(), direction.getZ(), gen.pick(f32, &fractions) });
    }
}

/// Boxes (min, max) and the triangle budget per walk for a mesh
fn generateBoxes(allocator: std.mem.Allocator, gen: *Gen, mesh: *const Mesh, boxes: *std.ArrayList([6]f32), budgets: *std.ArrayList(u32)) !void {
    var bounds: AABox = .empty;
    for (mesh.vertices.items) |v| bounds.encapsulateVec3(.fromFloat3(v));
    const center = bounds.getCenter();
    const extent = Vec3.max(bounds.getExtent(), Vec3.replicate(1.0e-3));

    for (0..6) |i| {
        var box: AABox = undefined;
        if (i == 0) {
            // Everything, including the padding nodes with HALF_FLT_MAX bounds
            box = .init(Vec3.replicate(-1.0e30), Vec3.replicate(1.0e30));
        } else {
            const c = center.add(Vec3.init(gen.float(-1, 1), gen.float(-1, 1), gen.float(-1, 1)).mul(extent));
            const e = extent.mul(Vec3.init(gen.float(0, 0.5), gen.float(0, 0.5), gen.float(0, 0.5)));
            box = .init(c.sub(e), c.add(e));
        }
        try boxes.append(allocator, .{ box.min.getX(), box.min.getY(), box.min.getZ(), box.max.getX(), box.max.getY(), box.max.getZ() });
        try budgets.append(allocator, gen.pick(u32, &.{ 1, 4, 8, 16, 64, 100000 }));
    }
}

// ---------------------------------------------------------------------------------------------------------------
// Comparison

/// Counts the mismatches of one kind of stream and reports the first few
const StreamChecker = struct {
    name: []const u8,
    num_cases: usize = 0,
    num_values: usize = 0,
    mismatches: usize = 0,

    fn check(self: *StreamChecker, description: []const u8, case_index: usize, zolt_stream: []const u32, jolt_stream: []const u32) void {
        self.num_cases += 1;
        self.num_values += jolt_stream.len;
        const len = @min(zolt_stream.len, jolt_stream.len);
        const first_diff: ?usize = for (0..len) |i| {
            if (zolt_stream[i] != jolt_stream[i]) break i;
        } else if (zolt_stream.len != jolt_stream.len) len else null;
        const diff = first_diff orelse return;

        if (self.mismatches < 5) {
            // Find the last marker before the mismatch to report where it happened
            var marker: u32 = 0;
            var marker_index: usize = 0;
            for (jolt_stream[0..@min(diff, jolt_stream.len)], 0..) |v, i|
                if (v >= 0xA0000001 and v <= 0xA0000013) {
                    marker = v;
                    marker_index = i;
                };
            std.debug.print("{s}: mismatch in case {d} ({s}) at value {d} of {d} / {d} (last marker 0x{x} at {d})\n", .{ self.name, case_index, description, diff, zolt_stream.len, jolt_stream.len, marker, marker_index });
            const from = diff -| 4;
            std.debug.print("  zolt: {any}\n  jolt: {any}\n", .{ zolt_stream[from..@min(diff + 8, zolt_stream.len)], jolt_stream[from..@min(diff + 8, jolt_stream.len)] });
        }
        self.mismatches += 1;
    }

    fn finish(self: *const StreamChecker) !void {
        if (self.mismatches > 0) {
            std.debug.print("{s}: {d} of {d} cases mismatch\n", .{ self.name, self.mismatches, self.num_cases });
            return error.ParityMismatch;
        }
    }
};

fn finishAll(checkers: []const *const StreamChecker) !void {
    var failed = false;
    for (checkers) |checker|
        checker.finish() catch {
            failed = true;
        };
    if (failed) return error.ParityMismatch;
}

/// Per test case statistics, to make sure that the interesting paths are taken
const Coverage = struct {
    cases: usize = 0,
    convert_ok: usize = 0,
    convert_errors: [8]usize = .{0} ** 8,
    failed_splits: usize = 0,
    ray_hits: usize = 0,
    rays: usize = 0,
    box_aborts: usize = 0,
    root_leaf: usize = 0,
};

fn countValue(stream: []const u32, value: u32) usize {
    var count: usize = 0;
    for (stream) |v| count += @intFromBool(v == value);
    return count;
}

const ConvertError = enum { too_many_triangles, material_index, vertex_offset, other };

test "AABBTree / TriangleSplitter" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};

    var splitter_checker: StreamChecker = .{ .name = "TriangleSplitter split" };
    var build_checker: StreamChecker = .{ .name = "AABBTreeBuilder build" };
    var convert_checker: StreamChecker = .{ .name = "AABBTreeToBuffer convert" };
    var walk_checker: StreamChecker = .{ .name = "NodeCodec WalkTree / TriangleCodec decode" };
    var coverage: Coverage = .{};

    const num_meshes = 420;
    for (0..num_meshes) |mesh_index| {
        var mesh: Mesh = .{};
        defer mesh.deinit(allocator);
        const kind: u32 = @intCast(mesh_index % num_mesh_kinds);
        try generateMesh(allocator, &gen, kind, &mesh);
        const nt: u32 = @intCast(mesh.triangles.items.len);

        // Errors: material indices that don't fit in 8 bits
        if (gen.oneIn(25)) mesh.triangles.items[gen.index(nt)].material_index = gen.range(256, 1000);

        // Leaf size (15 and up make Convert fail when a leaf has that many triangles)
        var max_triangles_per_leaf = gen.pick(u32, &.{ 1, 2, 3, 4, 4, 5, 8, 8, 8, 10, 14, 15, 16, 40 });
        var store_user_data = gen.oneIn(2);
        if (gen.oneIn(30)) {
            // The whole mesh in the root leaf (with more than 85 unshared vertices the codec can't address them)
            max_triangles_per_leaf = 1000000;
            store_user_data = false;
        }
        if (nt <= max_triangles_per_leaf and nt > 28)
            store_user_data = false; // Zolt asserts (like Jolt) when the user data offset doesn't fit, the reference has no asserts

        // Rays and boxes
        var rays: std.ArrayList([7]f32) = .empty;
        defer rays.deinit(allocator);
        try generateRays(allocator, &gen, &mesh, &rays);
        var boxes: std.ArrayList([6]f32) = .empty;
        defer boxes.deinit(allocator);
        var budgets: std.ArrayList(u32) = .empty;
        defer budgets.deinit(allocator);
        try generateBoxes(allocator, &gen, &mesh, &boxes, &budgets);

        const custom_bins = gen.oneIn(3);
        for (0..2) |splitter_type| {
            const min_bins = if (custom_bins) gen.range(1, 16) else 8;
            const input: Input = .{
                .vertices = mesh.vertices.items.ptr,
                .num_vertices = @intCast(mesh.vertices.items.len),
                .triangles = mesh.triangles.items.ptr,
                .num_triangles = nt,
                .splitter_type = @intCast(splitter_type),
                .min_num_bins = min_bins,
                .max_num_bins = if (custom_bins) gen.range(min_bins, 64) else 128,
                .num_triangles_per_bin = if (custom_bins) gen.range(1, 10) else 6,
                .max_triangles_per_leaf = max_triangles_per_leaf,
                .store_user_data = @intFromBool(store_user_data),
            };
            const case_index = 2 * mesh_index + splitter_type;
            var description_buffer: [256]u8 = undefined;
            const description = try std.fmt.bufPrint(&description_buffer, "{s}, {d} triangles, {s}, bins {d}/{d}/{d}, leaf {d}, user data {}", .{ mesh.kind, nt, if (splitter_type == 0) "binning" else "mean", input.min_num_bins, input.max_num_bins, input.num_triangles_per_bin, max_triangles_per_leaf, store_user_data });
            coverage.cases += 1;

            // Splitter
            {
                const leaf_size = @min(max_triangles_per_leaf, 8);
                var s: Stream = .{ .allocator = allocator };
                defer s.deinit();
                try zoltSplitterRun(allocator, &input, leaf_size, &s);
                const Ctx = struct { input: *const Input, leaf_size: u32 };
                const j = try joltStream(allocator, Ctx{ .input = &input, .leaf_size = leaf_size }, struct {
                    fn f(c: Ctx, out: [*]u32, capacity: u32) u32 {
                        return jolt.jolt_aabb_splitter_run(c.input, c.leaf_size, out, capacity);
                    }
                }.f);
                defer allocator.free(j);
                splitter_checker.check(description, case_index, s.values.items, j);

                // Count failed splits (index 2 + name length + 3 is the first success flag, stride 7)
                const name_len = j[0];
                var i: usize = name_len + 4;
                const end = j.len - nt;
                while (i + 7 <= end) : (i += 7) coverage.failed_splits += @intFromBool(j[i + 2] == 0);
            }

            // Builder
            {
                const cost_traversal = gen.float(0.1, 3.0);
                const cost_leaf = gen.float(0.1, 3.0);
                var s: Stream = .{ .allocator = allocator };
                defer s.deinit();
                try zoltBuild(allocator, &input, cost_traversal, cost_leaf, &s);
                const Ctx = struct { input: *const Input, cost_traversal: f32, cost_leaf: f32 };
                const j = try joltStream(allocator, Ctx{ .input = &input, .cost_traversal = cost_traversal, .cost_leaf = cost_leaf }, struct {
                    fn f(c: Ctx, out: [*]u32, capacity: u32) u32 {
                        return jolt.jolt_aabb_tree_build(c.input, num_query_nodes, c.cost_traversal, c.cost_leaf, out, capacity);
                    }
                }.f);
                defer allocator.free(j);
                build_checker.check(description, case_index, s.values.items, j);
                if (nt <= max_triangles_per_leaf) coverage.root_leaf += 1;
            }

            // Convert
            {
                var s: Stream = .{ .allocator = allocator };
                defer s.deinit();
                try zoltConvert(allocator, &input, &s);
                const j = try joltStream(allocator, &input, struct {
                    fn f(c: *const Input, out: [*]u32, capacity: u32) u32 {
                        return jolt.jolt_aabb_tree_convert(c, out, capacity);
                    }
                }.f);
                defer allocator.free(j);
                convert_checker.check(description, case_index, s.values.items, j);

                if (j[nt] == 1) {
                    coverage.convert_ok += 1;
                } else {
                    const message = j[nt + 2 ..][0..j[nt + 1]];
                    var text: [256]u8 = undefined;
                    for (message, 0..) |c, i| text[i] = @intCast(c);
                    const msg = text[0..message.len];
                    const err: ConvertError = if (std.mem.indexOf(u8, msg, "Too many triangles") != null) .too_many_triangles else if (std.mem.indexOf(u8, msg, "Material index") != null) .material_index else if (std.mem.indexOf(u8, msg, "Offset doesn't fit") != null) .vertex_offset else .other;
                    coverage.convert_errors[@intFromEnum(err)] += 1;
                }
            }

            // Walk
            {
                var s: Stream = .{ .allocator = allocator };
                defer s.deinit();
                try zoltWalk(allocator, &input, rays.items, boxes.items, budgets.items, &s);
                const Ctx = struct { input: *const Input, rays: []const [7]f32, boxes: []const [6]f32, budgets: []const u32 };
                const j = try joltStream(allocator, Ctx{ .input = &input, .rays = rays.items, .boxes = boxes.items, .budgets = budgets.items }, struct {
                    fn f(c: Ctx, out: [*]u32, capacity: u32) u32 {
                        return jolt.jolt_aabb_tree_walk(c.input, @ptrCast(c.rays.ptr), @intCast(c.rays.len), @ptrCast(c.boxes.ptr), c.budgets.ptr, @intCast(c.boxes.len), out, capacity);
                    }
                }.f);
                defer allocator.free(j);
                walk_checker.check(description, case_index, s.values.items, j);

                if (j.len > 0) {
                    coverage.rays += rays.items.len;
                    coverage.box_aborts += countValue(j, marker_abort);
                    // A ray hit is a walk done marker followed by isDone, fraction, block id != -1
                    var i: usize = 0;
                    while (i + 3 < j.len) : (i += 1) {
                        if (j[i] == marker_ray_visitor) {
                            var k = i + 1;
                            while (j[k] != marker_walk_done) k += 1;
                            coverage.ray_hits += @intFromBool(j[k + 3] != 0xffffffff);
                        }
                    }
                }
            }
        }
    }

    // Make sure the interesting paths were taken
    const covered = coverage.convert_ok > coverage.cases / 2 and coverage.convert_errors[0] > 0 and coverage.convert_errors[1] > 0 and
        coverage.convert_errors[2] > 0 and coverage.failed_splits > 0 and coverage.ray_hits > coverage.rays / 4 and coverage.box_aborts > 0;

    const result = finishAll(&.{ &splitter_checker, &build_checker, &convert_checker, &walk_checker });
    if (!covered or std.meta.isError(result)) {
        std.debug.print("AABBTree parity: {d} cases ({d} values split, {d} build, {d} convert, {d} walk), {d} converted, errors: {d} too many triangles, {d} material index, {d} vertex offset, {d} other; {d} failed splits, {d} root leaves, {d} / {d} rays hit, {d} box walk aborts\n", .{
            coverage.cases,
            splitter_checker.num_values,
            build_checker.num_values,
            convert_checker.num_values,
            walk_checker.num_values,
            coverage.convert_ok,
            coverage.convert_errors[0],
            coverage.convert_errors[1],
            coverage.convert_errors[2],
            coverage.convert_errors[3],
            coverage.failed_splits,
            coverage.root_leaf,
            coverage.ray_hits,
            coverage.rays,
            coverage.box_aborts,
        });
    }
    try result;
    try std.testing.expect(covered);
}

test "AABBTree / TriangleSplitter empty mesh" {
    const allocator = std.testing.allocator;
    const vertices = [_]Float3{.init(0, 0, 0)};
    const triangles = [_]IndexedTriangle{.init(0, 0, 0, .{})};
    for (0..2) |splitter_type| {
        // No triangles (pointers to valid memory, the count is zero)
        const input: Input = .{ .vertices = &vertices, .num_vertices = 1, .triangles = &triangles, .num_triangles = 0, .splitter_type = @intCast(splitter_type), .min_num_bins = 8, .max_num_bins = 128, .num_triangles_per_bin = 6, .max_triangles_per_leaf = 4, .store_user_data = 0 };
        var checker: StreamChecker = .{ .name = "AABBTreeBuilder build (empty)" };
        var s: Stream = .{ .allocator = allocator };
        defer s.deinit();
        try zoltBuild(allocator, &input, 1.0, 1.0, &s);
        const j = try joltStream(allocator, &input, struct {
            fn f(c: *const Input, out: [*]u32, capacity: u32) u32 {
                return jolt.jolt_aabb_tree_build(c, num_query_nodes, 1.0, 1.0, out, capacity);
            }
        }.f);
        defer allocator.free(j);
        checker.check("empty", splitter_type, s.values.items, j);

        var split_checker: StreamChecker = .{ .name = "TriangleSplitter split (empty)" };
        var s2: Stream = .{ .allocator = allocator };
        defer s2.deinit();
        try zoltSplitterRun(allocator, &input, 1, &s2);
        const j2 = try joltStream(allocator, &input, struct {
            fn f(c: *const Input, out: [*]u32, capacity: u32) u32 {
                return jolt.jolt_aabb_splitter_run(c, 1, out, capacity);
            }
        }.f);
        defer allocator.free(j2);
        split_checker.check("empty", splitter_type, s2.values.items, j2);
        try finishAll(&.{ &checker, &split_checker });
    }
}
