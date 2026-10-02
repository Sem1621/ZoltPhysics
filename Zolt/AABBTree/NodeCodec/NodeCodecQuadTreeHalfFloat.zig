//! Port of: Jolt/AABBTree/NodeCodec/NodeCodecQuadTreeHalfFloat.h
//! Status: complete
//!
//! A node codec for `AABBTreeToBuffer` (see there for the functions a node codec declares).
//! - `Header` and `Node` are written into the tree's ByteBuffer and are part of MeshShape's binary state, so they are
//!   extern structs with Jolt's layout (sizes checked at compile time).
//! - The anonymous node properties enum (`TRIANGLE_COUNT_BITS`, ...) becomes constants (`triangle_count_bits`, ...).
//! - Errors: where Jolt returns `size_t(-1)` / `false` and sets `outError`, Zolt returns an error from `Error`;
//!   `errorMessage(err)` gives Jolt's message.
//! - `NodeAllocate`'s `ioChildren` (which the codec may reorder) is a mutable slice, the arrays of child offsets
//!   passed to `NodeFinalize` are slices.
//! - `DecodingContext.walkTree` takes the visitor as `anytype` (a pointer to a struct that declares
//!   `visitNodes(min_x, min_y, min_z, max_x, max_y, max_z: Vec4, properties: *UVec4, stack_top: i32) i32`,
//!   `visitTriangles(triangle_context, triangles: *const anyopaque, num_triangles: u32, triangle_block_id: u32) void`,
//!   `shouldAbort() bool` and `shouldVisitNode(stack_top: i32) bool`), like the C++ template.
//! - JPH_CPU_BIG_ENDIAN: both ways of unpacking the bounds are ported (`unpackBoundsLittleEndian` /
//!   `unpackBoundsBigEndian`), `walkTree` picks the one for the target at compile time.

const std = @import("std");
const builtin = @import("builtin");
const math = @import("../../Math/Math.zig");
const half_float = @import("../../Math/HalfFloat.zig");
const HalfFloat = half_float.HalfFloat;
const Float3 = @import("../../Math/Float3.zig").Float3;
const UVec4 = @import("../../Math/UVec4.zig").UVec4;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const ByteBuffer = @import("../../Core/ByteBuffer.zig").ByteBuffer;
const AABBTreeBuilder = @import("../AABBTreeBuilder.zig").AABBTreeBuilder;

pub const NodeCodecQuadTreeHalfFloat = struct {
    /// Number of child nodes of this node
    pub const num_children_per_node = 4;

    /// Header for the tree
    pub const Header = extern struct {
        root_bounds_min: Float3,
        root_bounds_max: Float3,
        root_properties: u32,
        /// Number of bits to address a triangle block
        block_id_bits: u8,
        padding: [3]u8 = .{ 0, 0, 0 },

        comptime {
            std.debug.assert(@sizeOf(Header) == 32);
        }
    };

    /// Size of the header (an empty struct is always > 0 bytes so this needs a separate variable)
    pub const header_size = @sizeOf(Header);

    /// Stack size to use during DecodingContext::sWalkTree
    pub const stack_size = 128;

    // Node properties
    pub const triangle_count_bits = 4;
    pub const triangle_count_shift = 28;
    pub const triangle_count_mask = (1 << triangle_count_bits) - 1;
    pub const offset_bits = 28;
    pub const offset_mask = (1 << offset_bits) - 1;
    pub const offset_non_significant_bits = 2;
    pub const offset_non_significant_mask = (1 << offset_non_significant_bits) - 1;

    /// Node structure
    pub const Node = extern struct {
        /// 4 child bounding boxes
        bounds_min_x: [4]HalfFloat,
        bounds_min_y: [4]HalfFloat,
        bounds_min_z: [4]HalfFloat,
        bounds_max_x: [4]HalfFloat,
        bounds_max_y: [4]HalfFloat,
        bounds_max_z: [4]HalfFloat,
        /// 4 child node properties
        node_properties: [4]u32,

        comptime {
            std.debug.assert(@sizeOf(Node) == 64); // Node should be 64 bytes
        }
    };

    /// Errors of the encoder (Jolt's outError strings, see `errorMessage`)
    pub const Error = error{
        /// "NodeCodecQuadTreeHalfFloat: Too many triangles"
        NodeCodecTooManyTriangles,
        /// "NodeCodecQuadTreeHalfFloat: Internal Error: Offset has non-significant bits set"
        NodeCodecOffsetNonSignificantBitsSet,
        /// "NodeCodecQuadTreeHalfFloat: Offset too large. Too much data."
        NodeCodecOffsetTooLarge,
    };

    /// Jolt's error message for an error of the encoder
    pub fn errorMessage(err: Error) []const u8 {
        return switch (err) {
            error.NodeCodecTooManyTriangles => "NodeCodecQuadTreeHalfFloat: Too many triangles",
            error.NodeCodecOffsetNonSignificantBitsSet => "NodeCodecQuadTreeHalfFloat: Internal Error: Offset has non-significant bits set",
            error.NodeCodecOffsetTooLarge => "NodeCodecQuadTreeHalfFloat: Offset too large. Too much data.",
        };
    }

    /// This class encodes and compresses quad tree nodes
    pub const EncodingContext = struct {
        highest_triangle_block: usize = 0,

        /// Mimics the size a call to NodeAllocate() would add to the buffer
        pub fn prepareNodeAllocate(self: *const EncodingContext, node: *const AABBTreeBuilder.Node, buffer_size: *u64) void {
            _ = self;

            // We don't emit nodes for leafs
            if (!node.hasChildren())
                return;

            // Add size of node
            buffer_size.* += @sizeOf(Node);
        }

        /// Allocate a new node for node.
        /// Algorithm can modify the order of children to indicate in which order children should be compressed
        /// Algorithm can enlarge the bounding boxes of the children during compression and returns these in child_bounds_min, child_bounds_max
        /// node_bounds_min, node_bounds_max is the bounding box if node possibly widened by compressing the parent node
        /// Returns the start of the node in the buffer, or an error
        pub fn nodeAllocate(self: *const EncodingContext, allocator: std.mem.Allocator, node: *const AABBTreeBuilder.Node, node_bounds_min: Vec3, node_bounds_max: Vec3, children: []*const AABBTreeBuilder.Node, child_bounds_min: *[num_children_per_node]Vec3, child_bounds_max: *[num_children_per_node]Vec3, buffer: *ByteBuffer) (Error || std.mem.Allocator.Error)!usize {
            _ = self;

            // We don't emit nodes for leafs
            if (!node.hasChildren())
                return buffer.vector.items.len;

            // Remember the start of the node
            const node_start = buffer.vector.items.len;

            // Fill in bounds
            const out = &(try buffer.allocate(allocator, Node, .{}))[0];

            for (0..4) |i| {
                if (i < children.len) {
                    const this_node = children[i];

                    // Copy bounding box
                    out.bounds_min_x[i] = half_float.fromFloat(.round_to_neg_inf, this_node.bounds.min.getX());
                    out.bounds_min_y[i] = half_float.fromFloat(.round_to_neg_inf, this_node.bounds.min.getY());
                    out.bounds_min_z[i] = half_float.fromFloat(.round_to_neg_inf, this_node.bounds.min.getZ());
                    out.bounds_max_x[i] = half_float.fromFloat(.round_to_pos_inf, this_node.bounds.max.getX());
                    out.bounds_max_y[i] = half_float.fromFloat(.round_to_pos_inf, this_node.bounds.max.getY());
                    out.bounds_max_z[i] = half_float.fromFloat(.round_to_pos_inf, this_node.bounds.max.getZ());

                    // Store triangle count
                    out.node_properties[i] = this_node.getTriangleCount() << triangle_count_shift;
                    if (this_node.getTriangleCount() >= triangle_count_mask)
                        return error.NodeCodecTooManyTriangles;
                } else {
                    // Make this an invalid triangle node
                    out.node_properties[i] = @as(u32, triangle_count_mask) << triangle_count_shift;

                    // Make bounding box invalid
                    out.bounds_min_x[i] = half_float.half_flt_max;
                    out.bounds_min_y[i] = half_float.half_flt_max;
                    out.bounds_min_z[i] = half_float.half_flt_max;
                    out.bounds_max_x[i] = half_float.half_flt_max;
                    out.bounds_max_y[i] = half_float.half_flt_max;
                    out.bounds_max_z[i] = half_float.half_flt_max;
                }
            }

            // Since we don't keep track of the bounding box while descending the tree, we keep the root bounds at all levels for triangle compression
            for (0..num_children_per_node) |i| {
                child_bounds_min[i] = node_bounds_min;
                child_bounds_max[i] = node_bounds_max;
            }

            return node_start;
        }

        /// Once all nodes have been added, this call finalizes all nodes by patching in the offsets of the child nodes (that were added after the node itself was added)
        pub fn nodeFinalize(self: *EncodingContext, node: *const AABBTreeBuilder.Node, node_start: usize, num_children: u32, children_node_start: []const usize, children_triangles_start: []const usize, buffer: *ByteBuffer) Error!void {
            if (!node.hasChildren())
                return;

            const out = buffer.getMut(Node, node_start);
            for (0..num_children) |i| {
                var offset: usize = undefined;
                if (out.node_properties[i] != 0) {
                    // This is a triangle block
                    offset = children_triangles_start[i];

                    // Store highest block with triangles so we can count the number of bits we need
                    self.highest_triangle_block = @max(self.highest_triangle_block, offset);
                } else {
                    // This is a node block
                    offset = children_node_start[i];
                }

                // Store offset of next node / triangles
                if (offset & offset_non_significant_mask != 0)
                    return error.NodeCodecOffsetNonSignificantBitsSet;
                offset >>= offset_non_significant_bits;
                if (offset > offset_mask)
                    return error.NodeCodecOffsetTooLarge;
                out.node_properties[i] |= @as(u32, @intCast(offset));
            }
        }

        /// Once all nodes have been finalized, this will finalize the header of the nodes
        pub fn finalize(self: *const EncodingContext, header: *Header, root: *const AABBTreeBuilder.Node, root_node_start: usize, root_triangles_start: usize) Error!void {
            // Check if we can address the root node
            var offset = if (root.hasChildren()) root_node_start else root_triangles_start;
            if (offset & offset_non_significant_mask != 0)
                return error.NodeCodecOffsetNonSignificantBitsSet;
            offset >>= offset_non_significant_bits;
            if (offset > offset_mask)
                return error.NodeCodecOffsetTooLarge;

            // If the root has triangles, we need to take that offset instead since the mHighestTriangleBlock will be zero
            var highest_triangle_block = if (root_triangles_start != std.math.maxInt(usize)) root_triangles_start else self.highest_triangle_block;
            highest_triangle_block >>= offset_non_significant_bits;

            root.bounds.min.storeFloat3(&header.root_bounds_min);
            root.bounds.max.storeFloat3(&header.root_bounds_max);
            header.root_properties = @as(u32, @intCast(offset)) +% (root.getTriangleCount() << triangle_count_shift);
            header.block_id_bits = @intCast(32 - math.countLeadingZeros(@truncate(highest_triangle_block)));
            if (root.getTriangleCount() >= triangle_count_mask)
                return error.NodeCodecTooManyTriangles;
        }
    };

    /// The unpacked bounds of the 4 children of a node
    pub const NodeBounds = struct {
        min_x: Vec4,
        min_y: Vec4,
        min_z: Vec4,
        max_x: Vec4,
        max_y: Vec4,
        max_z: Vec4,
    };

    /// Unpack the bounds of a node like Jolt does on little endian CPUs (loads 2 arrays of 4 half floats at a time)
    pub fn unpackBoundsLittleEndian(node: *const Node) NodeBounds {
        const words: *const [16]u32 = @ptrCast(node);

        const bounds_minxy = UVec4.loadInt4(words[0..4]);
        const bounds_minx = half_float.toFloat(bounds_minxy);
        const bounds_miny = half_float.toFloat(bounds_minxy.swizzle(.z, .w, .unused, .unused));

        const bounds_minzmaxx = UVec4.loadInt4(words[4..8]);
        const bounds_minz = half_float.toFloat(bounds_minzmaxx);
        const bounds_maxx = half_float.toFloat(bounds_minzmaxx.swizzle(.z, .w, .unused, .unused));

        const bounds_maxyz = UVec4.loadInt4(words[8..12]);
        const bounds_maxy = half_float.toFloat(bounds_maxyz);
        const bounds_maxz = half_float.toFloat(bounds_maxyz.swizzle(.z, .w, .unused, .unused));

        return .{ .min_x = bounds_minx, .min_y = bounds_miny, .min_z = bounds_minz, .max_x = bounds_maxx, .max_y = bounds_maxy, .max_z = bounds_maxz };
    }

    /// Unpack the bounds of a node like Jolt does on big endian CPUs (JPH_CPU_BIG_ENDIAN, builds the vectors from the
    /// half float values, so it gives the same result on any CPU)
    pub fn unpackBoundsBigEndian(node: *const Node) NodeBounds {
        return .{
            .min_x = half_float.toFloat(packHalfFloats(node.bounds_min_x)),
            .min_y = half_float.toFloat(packHalfFloats(node.bounds_min_y)),
            .min_z = half_float.toFloat(packHalfFloats(node.bounds_min_z)),
            .max_x = half_float.toFloat(packHalfFloats(node.bounds_max_x)),
            .max_y = half_float.toFloat(packHalfFloats(node.bounds_max_y)),
            .max_z = half_float.toFloat(packHalfFloats(node.bounds_max_z)),
        };
    }

    /// UVec4(v[0] + (v[1] << 16), v[2] + (v[3] << 16), 0, 0) (the big endian path of WalkTree)
    fn packHalfFloats(v: [4]HalfFloat) UVec4 {
        return UVec4.init(@as(u32, v[0]) +% (@as(u32, v[1]) << 16), @as(u32, v[2]) +% (@as(u32, v[3]) << 16), 0, 0);
    }

    /// This class decodes and decompresses quad tree nodes
    pub const DecodingContext = struct {
        node_stack: [stack_size]u32 = undefined,
        top: i32 = 0,

        /// Get the amount of bits needed to store an ID to a triangle block
        pub fn triangleBlockIDBits(header: *const Header) u32 {
            return header.block_id_bits;
        }

        /// Convert a triangle block ID to the start of the triangle buffer
        pub fn getTriangleBlockStart(buffer_start: [*]const u8, triangle_block_id: u32) *const anyopaque {
            return buffer_start + (triangle_block_id << offset_non_significant_bits);
        }

        /// Constructor
        pub fn init(header: *const Header) DecodingContext {
            var result: DecodingContext = .{};

            // Start with the root node on the stack
            result.node_stack[0] = header.root_properties;

            return result;
        }

        /// Walk the node tree calling the Visitor::VisitNodes for each node encountered and Visitor::VisitTriangles for each triangle encountered
        pub fn walkTree(self: *DecodingContext, buffer_start: [*]const u8, triangle_context: anytype, visitor: anytype) void {
            while (true) {
                // Test if node contains triangles
                const node_properties = self.node_stack[@intCast(self.top)];
                const tri_count = node_properties >> triangle_count_shift;
                if (tri_count == 0) {
                    const node: *const Node = @ptrCast(@alignCast(buffer_start + (node_properties << offset_non_significant_bits)));

                    // Unpack bounds
                    const bounds = if (comptime builtin.cpu.arch.endian() == .big) unpackBoundsBigEndian(node) else unpackBoundsLittleEndian(node);

                    // Load properties for 4 children
                    var properties = UVec4.loadInt4(&node.node_properties);

                    // Check which sub nodes to visit
                    const num_results = visitor.visitNodes(bounds.min_x, bounds.min_y, bounds.min_z, bounds.max_x, bounds.max_y, bounds.max_z, &properties, self.top);

                    // Push them onto the stack
                    std.debug.assert(self.top + 4 < stack_size);
                    properties.storeInt4(self.node_stack[@intCast(self.top)..][0..4]);
                    self.top += @intCast(num_results);
                } else if (tri_count != triangle_count_mask) { // TRIANGLE_COUNT_MASK indicates a padding node, normally we shouldn't visit these nodes but when querying with a big enough box you could touch HALF_FLT_MAX (about 65K)
                    // Node contains triangles, do individual tests
                    const triangle_block_id = node_properties & offset_mask;
                    const triangles = getTriangleBlockStart(buffer_start, triangle_block_id);

                    visitor.visitTriangles(triangle_context, triangles, tri_count, triangle_block_id);
                }

                // Check if we're done
                if (visitor.shouldAbort())
                    break;

                // Fetch next node until we find one that the visitor wants to see
                while (true) {
                    self.top -= 1;
                    if (!(self.top >= 0 and !visitor.shouldVisitNode(self.top)))
                        break;
                }

                if (self.top < 0)
                    break;
            }
        }

        /// This can be used to have the visitor early out (ioVisitor.ShouldAbort() returns true) and later continue again (call WalkTree() again)
        pub fn isDoneWalking(self: *const DecodingContext) bool {
            return self.top < 0;
        }
    };
};

test "NodeCodecQuadTreeHalfFloat unpack bounds" {
    const Codec = NodeCodecQuadTreeHalfFloat;
    var node: Codec.Node = undefined;
    const values = [_]f32{ -1.5, 0.25, 3.0, 65504.0, -0.0, 5.9604645e-08, 100.0, -2.0, 7.5, 8.0, -9.0, 10.0, 0.5, 0.75, 1.25, 2.5, 1000.0, -1000.0, 0.125, 6.0, -7.0, 12.0, 13.0, 14.0 };
    const arrays = [_]*[4]HalfFloat{ &node.bounds_min_x, &node.bounds_min_y, &node.bounds_min_z, &node.bounds_max_x, &node.bounds_max_y, &node.bounds_max_z };
    for (arrays, 0..) |a, j|
        for (0..4) |i| {
            a[i] = half_float.fromFloat(.round_to_nearest, values[j * 4 + i]);
        };
    node.node_properties = .{ 1, 2, 3, 4 };

    // Both unpack paths give the bounds that were stored (all values are exact half floats)
    const big = Codec.unpackBoundsBigEndian(&node);
    const results = [_]Vec4{ big.min_x, big.min_y, big.min_z, big.max_x, big.max_y, big.max_z };
    for (results, 0..) |r, j|
        try std.testing.expect(r.eql(Vec4.init(values[j * 4], values[j * 4 + 1], values[j * 4 + 2], values[j * 4 + 3])));
    if (builtin.cpu.arch.endian() == .little) {
        const little = Codec.unpackBoundsLittleEndian(&node);
        try std.testing.expect(std.meta.eql(big, little));
    }

    // Decoding helpers
    const header: Codec.Header = .{ .root_bounds_min = .init(0, 0, 0), .root_bounds_max = .init(1, 1, 1), .root_properties = 0x12345678, .block_id_bits = 7 };
    try std.testing.expectEqual(@as(u32, 7), Codec.DecodingContext.triangleBlockIDBits(&header));
    const ctx: Codec.DecodingContext = .init(&header);
    try std.testing.expectEqual(@as(u32, 0x12345678), ctx.node_stack[0]);
    try std.testing.expect(!ctx.isDoneWalking());
    const buffer = [_]u8{0} ** 64;
    try std.testing.expectEqual(@as(*const anyopaque, &buffer[12]), Codec.DecodingContext.getTriangleBlockStart(&buffer, 3));
    try std.testing.expectEqualStrings("NodeCodecQuadTreeHalfFloat: Too many triangles", Codec.errorMessage(error.NodeCodecTooManyTriangles));
}
