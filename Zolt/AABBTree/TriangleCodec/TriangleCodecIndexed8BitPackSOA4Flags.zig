//! Port of: Jolt/AABBTree/TriangleCodec/TriangleCodecIndexed8BitPackSOA4Flags.h
//! Status: complete
//!
//! A triangle codec for `AABBTreeToBuffer` (see there for the functions a triangle codec declares).
//! - `TriangleHeader`, `TriangleBlockHeader`, `TriangleBlock` and `VertexData` are written into the tree's ByteBuffer
//!   and are part of MeshShape's binary state, so they are extern structs with Jolt's layout (sizes checked at
//!   compile time).
//! - The unscoped enums `EComponentData`, `EVertexXY`, `EVertexZY` and `ETriangleBlockHeaderFlags` become constants
//!   (`COMPONENT_BITS` -> `component_bits`, ...).
//! - Pointer + count arguments (`const IndexedTriangle *inTriangles, uint inNumTriangles`) become slices, output
//!   arrays (`Vec3 *outTriangles`, `uint8 *outTriangleFlags`) become slices that must be big enough.
//! - Errors: where Jolt returns `size_t(-1)` and sets `outError`, Zolt returns an error from `Error`;
//!   `errorMessage(err)` gives Jolt's message.
//! - The encoding context owns memory: `EncodingContext.init(allocator, vertices)` / `deinit(allocator)`.
//! - DecodingContext overloads: the private `Unpack` helpers are `unpackVertex` / `unpackBlock`,
//!   `Unpack(start, count, outTriangles, outFlags)` is `unpackWithFlags`, `sGetFlags(start, count, outFlags)` is
//!   `getFlags` and `sGetFlags(start, index)` is `getTriangleFlags`. `TestRay(..., outClosestTriangleIndex)` returns
//!   `TestRayResult{ .closest, .closest_triangle_index }` and `GetTriangle(..., outV1, outV2, outV3)` returns
//!   `TriangleVertices{ .v1, .v2, .v3 }`.
//! - Zolt's `Vec3.toInt` requires the quantized coordinates to be in the range of an i32 (safety checked), so the
//!   vertices must be finite and the bounds of the mesh must not overflow, like the meshes Jolt supports.

const std = @import("std");
const builtin = @import("builtin");
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const Float3 = @import("../../Math/Float3.zig").Float3;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const UVec4 = @import("../../Math/UVec4.zig").UVec4;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const ByteBuffer = @import("../../Core/ByteBuffer.zig").ByteBuffer;
const AABox = @import("../../Geometry/AABox.zig").AABox;
const IndexedTriangle = @import("../../Geometry/IndexedTriangle.zig").IndexedTriangle;
const rayTriangle4 = @import("../../Geometry/RayTriangle.zig").rayTriangle4;

/// Store vertices in 64 bits and indices in 8 bits + 8 bit of flags per triangle like this:
///
/// TriangleBlockHeader,
/// TriangleBlock (4 triangles and their flags in 16 bytes),
/// TriangleBlock...
/// [Optional] UserData (4 bytes per triangle)
///
/// Vertices are stored:
///
/// VertexData (1 vertex in 64 bits),
/// VertexData...
///
/// They're compressed relative to the bounding box as provided by the node codec.
pub const TriangleCodecIndexed8BitPackSOA4Flags = struct {
    pub const TriangleHeader = extern struct {
        /// Offset of all vertices
        offset: Float3,
        /// Scale of all vertices, vertex_position = mOffset + mScale * compressed_vertex_position
        scale: Float3,

        comptime {
            std.debug.assert(@sizeOf(TriangleHeader) == 24);
        }
    };

    /// Size of the header (an empty struct is always > 0 bytes so this needs a separate variable)
    pub const triangle_header_size = @sizeOf(TriangleHeader);

    /// If this codec could return a different offset than the current buffer size when calling Pack()
    pub const changes_offset_on_pack = false;

    // Amount of bits per component (EComponentData)
    pub const component_bits = 21;
    pub const component_mask = (1 << component_bits) - 1;

    // Packed X and Y coordinate (EVertexXY)
    pub const component_x = 0;
    pub const component_y1 = component_bits;
    pub const component_y1_bits = 32 - component_bits;

    // Packed Z and Y coordinate (EVertexZY)
    pub const component_z = 0;
    pub const component_y2 = component_bits;
    pub const component_y2_bits = 31 - component_bits;

    /// A single packed vertex
    pub const VertexData = extern struct {
        vertex_xy: u32,
        vertex_zy: u32,

        comptime {
            std.debug.assert(@sizeOf(VertexData) == 8); // Compiler added padding
        }
    };

    /// A block of 4 triangles
    pub const TriangleBlock = extern struct {
        /// 8 bit indices to triangle vertices for 4 triangles in the form mIndices[vertex][triangle] where vertex in [0, 2] and triangle in [0, 3]
        indices: [3][4]u8,
        /// Triangle flags (could contain material and active edges)
        flags: [4]u8,

        comptime {
            std.debug.assert(@sizeOf(TriangleBlock) == 16); // Compiler added padding
        }
    };

    // ETriangleBlockHeaderFlags

    /// Offset from current block to start of vertices in bytes
    pub const offset_to_vertices_bits = 29;
    pub const offset_to_vertices_mask = (1 << offset_to_vertices_bits) - 1;
    /// The offset from the current block to the start of the vertices must be a multiple of 4 bytes
    pub const offset_non_significant_bits = 2;
    pub const offset_non_significant_mask = (1 << offset_non_significant_bits) - 1;
    /// When user data is stored, this is the number of blocks to skip to get to the user data (0 = no user data)
    pub const offset_to_userdata_bits = 3;
    pub const offset_to_userdata_mask = (1 << offset_to_userdata_bits) - 1;

    /// A triangle header, will be followed by one or more TriangleBlocks
    pub const TriangleBlockHeader = extern struct {
        flags: u32,

        comptime {
            std.debug.assert(@sizeOf(TriangleBlockHeader) == 4); // Compiler added padding
        }

        pub fn getVertexData(self: *const TriangleBlockHeader) [*]const VertexData {
            const bytes: [*]const u8 = @ptrCast(self);
            return @ptrCast(@alignCast(bytes + ((self.flags & offset_to_vertices_mask) << offset_non_significant_bits)));
        }

        pub fn getTriangleBlock(self: *const TriangleBlockHeader) [*]const TriangleBlock {
            const bytes: [*]const u8 = @ptrCast(self);
            return @ptrCast(bytes + @sizeOf(TriangleBlockHeader));
        }

        pub fn getUserData(self: *const TriangleBlockHeader) ?[*]const u32 {
            const offset = self.flags >> offset_to_vertices_bits;
            return if (offset == 0) null else @ptrCast(@alignCast(self.getTriangleBlock() + offset));
        }
    };

    /// Errors of the encoder (Jolt's outError strings, see `errorMessage`)
    pub const Error = error{
        /// "TriangleCodecIndexed8BitPackSOA4Flags: Internal Error: Offset has non-significant bits set"
        TriangleCodecOffsetNonSignificantBitsSet,
        /// "TriangleCodecIndexed8BitPackSOA4Flags: Offset to vertices doesn't fit. Too much data."
        TriangleCodecOffsetToVerticesTooLarge,
        /// "TriangleCodecIndexed8BitPackSOA4Flags: Offset doesn't fit in 8 bit"
        TriangleCodecVertexOffsetTooLarge,
        /// "TriangleCodecIndexed8BitPackSOA4Flags: Material index doesn't fit in 8 bit"
        TriangleCodecMaterialIndexTooLarge,
    };

    /// Jolt's error message for an error of the encoder
    pub fn errorMessage(err: Error) []const u8 {
        return switch (err) {
            error.TriangleCodecOffsetNonSignificantBitsSet => "TriangleCodecIndexed8BitPackSOA4Flags: Internal Error: Offset has non-significant bits set",
            error.TriangleCodecOffsetToVerticesTooLarge => "TriangleCodecIndexed8BitPackSOA4Flags: Offset to vertices doesn't fit. Too much data.",
            error.TriangleCodecVertexOffsetTooLarge => "TriangleCodecIndexed8BitPackSOA4Flags: Offset doesn't fit in 8 bit",
            error.TriangleCodecMaterialIndexTooLarge => "TriangleCodecIndexed8BitPackSOA4Flags: Material index doesn't fit in 8 bit",
        };
    }

    /// This class is used to validate that the triangle data will not be degenerate after compression
    pub const ValidationContext = struct {
        vertices: []const Float3,
        bounds: AABox = .empty,

        /// Constructor
        pub fn init(triangles: []const IndexedTriangle, vertices: []const Float3) ValidationContext {
            var self: ValidationContext = .{ .vertices = vertices };

            // Only used the referenced triangles, just like EncodingContext::Finalize does
            for (triangles) |i|
                for (i.idx) |idx|
                    self.bounds.encapsulateVec3(Vec3.fromFloat3(vertices[idx]));

            return self;
        }

        /// Test if a triangle will be degenerate after quantization
        pub fn isDegenerate(self: *const ValidationContext, triangle: IndexedTriangle) bool {
            // Quantize the triangle in the same way as EncodingContext::Finalize does
            var quantized_vertex: [3]UVec4 = undefined;
            const compress_scale = Vec3.replicate(component_mask).div(Vec3.max(self.bounds.getSize(), Vec3.replicate(1.0e-20)));
            for (0..3) |i|
                quantized_vertex[i] = Vec3.fromFloat3(self.vertices[triangle.idx[i]]).sub(self.bounds.min).mul(compress_scale).add(Vec3.replicate(0.5)).toInt();
            return quantized_vertex[0].eql(quantized_vertex[1]) or quantized_vertex[1].eql(quantized_vertex[2]) or quantized_vertex[0].eql(quantized_vertex[2]);
        }
    };

    /// This class is used to encode and compress triangle data into a byte buffer
    pub const EncodingContext = struct {
        /// Indicates a vertex hasn't been seen yet in the triangle list
        pub const not_found: u32 = 0xffffffff;

        /// Number of vertices calculated during PreparePack
        vertex_count: u32 = 0,
        /// Start of the vertices in the output buffer, calculated during PreparePack
        vertices_start_idx: usize = 0,
        /// Output vertices as an index into the original vertex list (vertices), sorted according to occurrence
        vertices: std.ArrayList(u32) = .empty,
        /// Maps from the original mesh vertex index (vertices) to the index in our output vertices (self.vertices)
        vertex_map: VertexMap = .empty,

        const VertexMap = std.ArrayList(u32);

        /// Construct the encoding context
        pub fn init(allocator: std.mem.Allocator, vertices: []const Float3) std.mem.Allocator.Error!EncodingContext {
            var self: EncodingContext = .{};
            try self.vertex_map.appendNTimes(allocator, not_found, vertices.len);
            return self;
        }

        /// Destructor
        pub fn deinit(self: *EncodingContext, allocator: std.mem.Allocator) void {
            self.vertices.deinit(allocator);
            self.vertex_map.deinit(allocator);
        }

        /// Mimics the size a call to Pack() would add to the buffer
        pub fn preparePack(self: *EncodingContext, triangles: []const IndexedTriangle, store_user_data: bool, buffer_size: *u64) void {
            const num_triangles: u32 = @intCast(triangles.len);

            // Add triangle block header
            buffer_size.* += @sizeOf(TriangleBlockHeader);

            // Compute first vertex that this batch will use (ensuring there's enough room if none of the vertices are shared)
            const start_vertex: u32 = startVertex(self.vertex_count, num_triangles);

            // Pack vertices
            const padded_triangle_count = math.alignUp(num_triangles, 4);
            var t: u32 = 0;
            while (t < padded_triangle_count) : (t += 4) {
                // Add triangle block header
                buffer_size.* += @sizeOf(TriangleBlock);

                for (0..3) |vertex_nr|
                    for (0..4) |block_tri_idx| {
                        // Fetch vertex index. Create degenerate triangles for padding triangles.
                        const triangle_available = t + block_tri_idx < num_triangles;
                        const src_vertex_index = if (triangle_available) triangles[t + block_tri_idx].idx[vertex_nr] else triangles[num_triangles - 1].idx[0];

                        // Check if we've seen this vertex before and if it is in the range that we can encode
                        const vertex_index = &self.vertex_map.items[src_vertex_index];
                        if (vertex_index.* == not_found or vertex_index.* < start_vertex) {
                            // Add vertex
                            vertex_index.* = self.vertex_count;
                            self.vertex_count += 1;
                        }
                    };
            }

            // Add user data
            if (store_user_data)
                buffer_size.* += @as(usize, num_triangles) * @sizeOf(u32);
        }

        /// Mimics the size the Finalize() call would add to buffer_size
        pub fn finalizePreparePack(self: *EncodingContext, allocator: std.mem.Allocator, buffer_size: *u64) std.mem.Allocator.Error!void {
            // Remember where the vertices are going to start in the output buffer
            std.debug.assert(math.isAligned(buffer_size.*, 4));
            self.vertices_start_idx = @intCast(buffer_size.*);

            // Add vertices to buffer
            buffer_size.* += @as(u64, self.vertex_count) * @sizeOf(VertexData);

            // Reserve the amount of memory we need for the vertices
            try self.vertices.ensureTotalCapacityPrecise(allocator, self.vertex_count);

            // Set vertex map back to 'not found'
            for (self.vertex_map.items) |*v|
                v.* = not_found;
        }

        /// Pack the triangles in triangles to buffer. This stores the mMaterialIndex of a triangle in the 8 bit flags.
        /// Returns the start of the triangle block in the buffer, or an error.
        pub fn pack(self: *EncodingContext, allocator: std.mem.Allocator, triangles: []const IndexedTriangle, store_user_data: bool, buffer: *ByteBuffer) (Error || std.mem.Allocator.Error)!usize {
            const num_triangles: u32 = @intCast(triangles.len);
            std.debug.assert(num_triangles > 0);

            // Determine position of triangles start
            const triangle_block_start = buffer.vector.items.len;

            // Allocate triangle block header
            const header = &(try buffer.allocate(allocator, TriangleBlockHeader, .{}))[0];

            // Compute first vertex that this batch will use (ensuring there's enough room if none of the vertices are shared)
            const start_vertex: u32 = startVertex(@intCast(self.vertices.items.len), num_triangles);

            // Store the start vertex offset relative to TriangleBlockHeader
            var offset_to_vertices: usize = self.vertices_start_idx -% triangle_block_start +% @as(usize, start_vertex) *% @sizeOf(VertexData);
            if (offset_to_vertices & offset_non_significant_mask != 0)
                return error.TriangleCodecOffsetNonSignificantBitsSet;
            offset_to_vertices >>= offset_non_significant_bits;
            if (offset_to_vertices > offset_to_vertices_mask)
                return error.TriangleCodecOffsetToVerticesTooLarge;
            header.flags = @intCast(offset_to_vertices);

            // When we store user data we need to store the offset to the user data in TriangleBlocks
            const padded_triangle_count = math.alignUp(num_triangles, 4);
            if (store_user_data) {
                const num_blocks: u32 = padded_triangle_count >> 2;
                if (Core.enable_asserts) // Reachable with more than 28 triangles, release builds continue like Jolt does (the offset is lost)
                    std.debug.assert(num_blocks <= offset_to_userdata_mask);
                header.flags |= num_blocks << offset_to_vertices_bits;
            }

            // Pack vertices
            var t: u32 = 0;
            while (t < padded_triangle_count) : (t += 4) {
                const block = &(try buffer.allocate(allocator, TriangleBlock, .{}))[0];
                for (0..3) |vertex_nr|
                    for (0..4) |block_tri_idx| {
                        // Fetch vertex index. Create degenerate triangles for padding triangles.
                        const triangle_available = t + block_tri_idx < num_triangles;
                        const src_vertex_index = if (triangle_available) triangles[t + block_tri_idx].idx[vertex_nr] else triangles[num_triangles - 1].idx[0];

                        // Check if we've seen this vertex before and if it is in the range that we can encode
                        const vertex_index = &self.vertex_map.items[src_vertex_index];
                        if (vertex_index.* == not_found or vertex_index.* < start_vertex) {
                            // Add vertex
                            vertex_index.* = @intCast(self.vertices.items.len);
                            try self.vertices.append(allocator, src_vertex_index);
                        }

                        // Store vertex index
                        const vertex_offset: u32 = vertex_index.* - start_vertex;
                        if (vertex_offset > 0xff)
                            return error.TriangleCodecVertexOffsetTooLarge;
                        block.indices[vertex_nr][block_tri_idx] = @intCast(vertex_offset);

                        // Store flags
                        const flags: u32 = if (triangle_available) triangles[t + block_tri_idx].material_index else 0;
                        if (flags > 0xff)
                            return error.TriangleCodecMaterialIndexTooLarge;
                        block.flags[block_tri_idx] = @intCast(flags);
                    };
            }

            // Store user data
            if (store_user_data) {
                const user_data = try buffer.allocate(allocator, u32, .{ .size = num_triangles });
                for (0..num_triangles) |i|
                    user_data[i] = triangles[i].user_data;
            }

            return triangle_block_start;
        }

        /// After all triangles have been packed, this finalizes the header and triangle buffer.
        /// `header` points into `buffer`: the buffer must have been reserved for the final size so that it doesn't move.
        pub fn finalize(self: *const EncodingContext, allocator: std.mem.Allocator, vertices: []const Float3, header: *TriangleHeader, buffer: *ByteBuffer) std.mem.Allocator.Error!void {
            // Assert that our reservations were correct
            std.debug.assert(self.vertices.items.len == self.vertex_count);
            std.debug.assert(buffer.vector.items.len == self.vertices_start_idx);

            // Check if anything to do
            if (self.vertices.items.len == 0)
                return;

            // Calculate bounding box
            var bounds: AABox = .empty;
            for (self.vertices.items) |v|
                bounds.encapsulateVec3(Vec3.fromFloat3(vertices[v]));

            // Compress vertices
            const out_vertices = try buffer.allocate(allocator, VertexData, .{ .size = self.vertices.items.len });
            const compress_scale = Vec3.replicate(component_mask).div(Vec3.max(bounds.getSize(), Vec3.replicate(1.0e-20)));
            for (self.vertices.items, out_vertices) |v, *out| {
                const c = Vec3.fromFloat3(vertices[v]).sub(bounds.min).mul(compress_scale).add(Vec3.replicate(0.5)).toInt();
                std.debug.assert(c.getX() <= component_mask);
                std.debug.assert(c.getY() <= component_mask);
                std.debug.assert(c.getZ() <= component_mask);
                out.vertex_xy = c.getX() +% (c.getY() << component_y1);
                out.vertex_zy = c.getZ() +% ((c.getY() >> component_y1_bits) << component_y2);
            }

            // Store decompression information
            bounds.min.storeFloat3(&header.offset);
            bounds.getSize().div(Vec3.replicate(component_mask)).storeFloat3(&header.scale);
        }

        /// Clamp((int)vertex_count - 256 + (int)num_triangles * 3, 0, (int)vertex_count)
        fn startVertex(vertex_count: u32, num_triangles: u32) u32 {
            const count: i32 = @bitCast(vertex_count);
            return @intCast(math.clamp(count - 256 + @as(i32, @bitCast(num_triangles)) * 3, 0, count));
        }
    };

    /// Result of `DecodingContext.testRay`
    pub const TestRayResult = struct {
        /// Fraction of the closest hit (the input `closest` when there is no closer hit)
        closest: f32,
        /// Index of the closest triangle in the block (Jolt's outClosestTriangleIndex)
        closest_triangle_index: u32,
    };

    /// Result of `DecodingContext.getTriangle`
    pub const TriangleVertices = struct {
        v1: Vec3,
        v2: Vec3,
        v3: Vec3,
    };

    /// The unpacked vertices of 4 triangles: x, y and z coordinates of the first, second and third vertex
    pub const UnpackedBlock = struct {
        v1x: Vec4,
        v1y: Vec4,
        v1z: Vec4,
        v2x: Vec4,
        v2y: Vec4,
        v2z: Vec4,
        v3x: Vec4,
        v3y: Vec4,
        v3z: Vec4,
    };

    /// This class is used to decode and decompress triangle data packed by the EncodingContext
    pub const DecodingContext = struct {
        offset_x: Vec4,
        offset_y: Vec4,
        offset_z: Vec4,
        scale_x: Vec4,
        scale_y: Vec4,
        scale_z: Vec4,

        /// The x, y and z coordinates of 1 vertex of 4 triangles
        const UnpackedVertex = struct { x: Vec4, y: Vec4, z: Vec4 };

        /// Private helper function to unpack the 1 vertex of 4 triangles (x contains the x coordinate of triangle 0 .. 3 etc.)
        fn unpackVertex(self: *const DecodingContext, vertices: [*]const VertexData, index: UVec4) UnpackedVertex {
            // Get compressed data
            const c1 = UVec4.gatherInt4(8, @ptrCast(&vertices[0].vertex_xy), index);
            const c2 = UVec4.gatherInt4(8, @ptrCast(&vertices[0].vertex_zy), index);

            // Unpack the x y and z component
            const xc = UVec4.bitAnd(c1, UVec4.replicate(component_mask));
            const yc = UVec4.bitOr(c1.logicalShiftRight(component_y1), c2.logicalShiftRight(component_y2).logicalShiftLeft(component_y1_bits));
            const zc = UVec4.bitAnd(c2, UVec4.replicate(component_mask));

            // Convert to float
            return .{
                .x = Vec4.fusedMultiplyAdd(xc.toFloat(), self.scale_x, self.offset_x),
                .y = Vec4.fusedMultiplyAdd(yc.toFloat(), self.scale_y, self.offset_y),
                .z = Vec4.fusedMultiplyAdd(zc.toFloat(), self.scale_z, self.offset_z),
            };
        }

        /// Private helper function to unpack 4 triangles from a triangle block
        fn unpackBlock(self: *const DecodingContext, block: *const TriangleBlock, vertices: [*]const VertexData) UnpackedBlock {
            // Get the indices for the three vertices (reads 4 bytes extra, but these are the flags so that's ok)
            const words: [4]u32 = @as(*align(1) const [4]u32, @ptrCast(block)).*;
            const indices = UVec4.loadInt4(&words);
            var iv1 = indices.expand4Byte0();
            var iv2 = indices.expand4Byte4();
            var iv3 = indices.expand4Byte8();

            if (comptime builtin.cpu.arch.endian() == .big) {
                // On big endian systems we need to reverse the bytes
                iv1 = iv1.swizzle(.w, .z, .y, .x);
                iv2 = iv2.swizzle(.w, .z, .y, .x);
                iv3 = iv3.swizzle(.w, .z, .y, .x);
            }

            // Decompress the triangle data
            const v1 = self.unpackVertex(vertices, iv1);
            const v2 = self.unpackVertex(vertices, iv2);
            const v3 = self.unpackVertex(vertices, iv3);
            return .{ .v1x = v1.x, .v1y = v1.y, .v1z = v1.z, .v2x = v2.x, .v2y = v2.y, .v2z = v2.z, .v3x = v3.x, .v3y = v3.y, .v3z = v3.z };
        }

        /// Constructor
        pub fn init(header: *const TriangleHeader) DecodingContext {
            return .{
                .offset_x = Vec4.replicate(header.offset.x),
                .offset_y = Vec4.replicate(header.offset.y),
                .offset_z = Vec4.replicate(header.offset.z),
                .scale_x = Vec4.replicate(header.scale.x),
                .scale_y = Vec4.replicate(header.scale.y),
                .scale_z = Vec4.replicate(header.scale.z),
            };
        }

        /// Unpacks triangles in the format t1v1,t1v2,t1v3, t2v1,t2v2,t2v3, ... (out_triangles needs room for 3 * num_triangles vertices)
        pub fn unpack(self: *const DecodingContext, triangle_start: *const anyopaque, num_triangles: u32, out_triangles: []Vec3) void {
            std.debug.assert(num_triangles > 0);
            std.debug.assert(out_triangles.len >= 3 * @as(usize, num_triangles));
            const header: *const TriangleBlockHeader = @ptrCast(@alignCast(triangle_start));
            const vertices = header.getVertexData();
            var t = header.getTriangleBlock();
            const end = t + ((num_triangles +% 3) >> 2);

            var triangles_left: i32 = @bitCast(num_triangles);
            var out: usize = 0;

            while (true) {
                // Unpack the vertices for 4 triangles
                const v = self.unpackBlock(&t[0], vertices);

                // Transpose it so we get normal vectors
                const v1 = Mat44.init(v.v1x, v.v1y, v.v1z, Vec4.zero()).transposed();
                const v2 = Mat44.init(v.v2x, v.v2y, v.v2z, Vec4.zero()).transposed();
                const v3 = Mat44.init(v.v3x, v.v3y, v.v3z, Vec4.zero()).transposed();

                // Store triangle data
                var i: u32 = 0;
                while (i < 4 and triangles_left > 0) : ({
                    i += 1;
                    triangles_left -= 1;
                }) {
                    out_triangles[out] = v1.getColumn3(i);
                    out_triangles[out + 1] = v2.getColumn3(i);
                    out_triangles[out + 2] = v3.getColumn3(i);
                    out += 3;
                }

                t += 1;
                if (@intFromPtr(t) >= @intFromPtr(end))
                    break;
            }
        }

        /// Tests a ray against the packed triangles
        pub fn testRay(self: *const DecodingContext, ray_origin: Vec3, ray_direction: Vec3, triangle_start: *const anyopaque, num_triangles: u32, closest_in: f32) TestRayResult {
            std.debug.assert(num_triangles > 0);
            const header: *const TriangleBlockHeader = @ptrCast(@alignCast(triangle_start));
            const vertices = header.getVertexData();
            var t = header.getTriangleBlock();
            const end = t + ((num_triangles +% 3) >> 2);

            var closest = Vec4.replicate(closest_in);
            var closest_triangle_idx = UVec4.zero();

            var start_triangle_idx = UVec4.zero();
            while (true) {
                // Unpack the vertices for 4 triangles
                const v = self.unpackBlock(&t[0], vertices);

                // Perform ray vs triangle test
                const distance = rayTriangle4(ray_origin, ray_direction, v.v1x, v.v1y, v.v1z, v.v2x, v.v2y, v.v2z, v.v3x, v.v3y, v.v3z);

                // Update closest with the smaller values
                const smaller = Vec4.less(distance, closest);
                closest = Vec4.select(closest, distance, smaller);

                // Update triangle index with the smallest values
                const triangle_idx = start_triangle_idx.add(UVec4.init(0, 1, 2, 3));
                closest_triangle_idx = UVec4.select(closest_triangle_idx, triangle_idx, smaller);

                // Next block
                t += 1;
                start_triangle_idx = start_triangle_idx.add(UVec4.replicate(4));
                if (@intFromPtr(t) >= @intFromPtr(end))
                    break;
            }

            // Get the smallest component
            Vec4.sort4(&closest, &closest_triangle_idx);
            return .{ .closest = closest.getX(), .closest_triangle_index = closest_triangle_idx.getX() };
        }

        /// Decode a single triangle
        pub fn getTriangle(self: *const DecodingContext, triangle_start: *const anyopaque, triangle_idx: u32) TriangleVertices {
            const header: *const TriangleBlockHeader = @ptrCast(@alignCast(triangle_start));
            const vertices = header.getVertexData();
            const block = &header.getTriangleBlock()[triangle_idx >> 2];
            const block_triangle_idx = triangle_idx & 0b11;

            // Get the 3 vertices
            const v1 = &vertices[block.indices[0][block_triangle_idx]];
            const v2 = &vertices[block.indices[1][block_triangle_idx]];
            const v3 = &vertices[block.indices[2][block_triangle_idx]];

            // Pack the vertices
            const c1 = UVec4.init(v1.vertex_xy, v2.vertex_xy, v3.vertex_xy, 0);
            const c2 = UVec4.init(v1.vertex_zy, v2.vertex_zy, v3.vertex_zy, 0);

            // Unpack the x y and z component
            const xc = UVec4.bitAnd(c1, UVec4.replicate(component_mask));
            const yc = UVec4.bitOr(c1.logicalShiftRight(component_y1), c2.logicalShiftRight(component_y2).logicalShiftLeft(component_y1_bits));
            const zc = UVec4.bitAnd(c2, UVec4.replicate(component_mask));

            // Convert to float
            const vx = Vec4.fusedMultiplyAdd(xc.toFloat(), self.scale_x, self.offset_x);
            const vy = Vec4.fusedMultiplyAdd(yc.toFloat(), self.scale_y, self.offset_y);
            const vz = Vec4.fusedMultiplyAdd(zc.toFloat(), self.scale_z, self.offset_z);

            // Transpose it so we get normal vectors
            const trans = Mat44.init(vx, vy, vz, Vec4.zero()).transposed();
            return .{ .v1 = trans.getAxisX(), .v2 = trans.getAxisY(), .v3 = trans.getAxisZ() };
        }

        /// Get user data for a triangle
        pub fn getUserData(self: *const DecodingContext, triangle_start: *const anyopaque, triangle_idx: u32) u32 {
            _ = self;
            const header: *const TriangleBlockHeader = @ptrCast(@alignCast(triangle_start));
            const user_data = header.getUserData();
            return if (user_data) |u| u[triangle_idx] else 0;
        }

        /// Get flags for entire triangle block (out_triangle_flags needs room for num_triangles flags)
        pub fn getFlags(triangle_start: *const anyopaque, num_triangles: u32, out_triangle_flags: []u8) void {
            std.debug.assert(num_triangles > 0);
            std.debug.assert(out_triangle_flags.len >= num_triangles);
            const header: *const TriangleBlockHeader = @ptrCast(@alignCast(triangle_start));
            var t = header.getTriangleBlock();
            const end = t + ((num_triangles +% 3) >> 2);

            var triangles_left: i32 = @bitCast(num_triangles);
            var out: usize = 0;
            while (true) {
                var i: usize = 0;
                while (i < 4 and triangles_left > 0) : ({
                    i += 1;
                    triangles_left -= 1;
                }) {
                    out_triangle_flags[out] = t[0].flags[i];
                    out += 1;
                }

                t += 1;
                if (@intFromPtr(t) >= @intFromPtr(end))
                    break;
            }
        }

        /// Get flags for a particular triangle (sGetFlags(inTriangleStart, inTriangleIndex))
        pub fn getTriangleFlags(triangle_start: *const anyopaque, triangle_index: i32) u8 {
            const header: *const TriangleBlockHeader = @ptrCast(@alignCast(triangle_start));
            const first_block = header.getTriangleBlock();
            return first_block[@intCast(triangle_index >> 2)].flags[@intCast(triangle_index & 0b11)];
        }

        /// Unpacks triangles and flags, convenience function
        pub fn unpackWithFlags(self: *const DecodingContext, triangle_start: *const anyopaque, num_triangles: u32, out_triangles: []Vec3, out_triangle_flags: []u8) void {
            self.unpack(triangle_start, num_triangles, out_triangles);
            getFlags(triangle_start, num_triangles, out_triangle_flags);
        }
    };
};

const Codec = TriangleCodecIndexed8BitPackSOA4Flags;

test "TriangleCodecIndexed8BitPackSOA4Flags pack and unpack" {
    const allocator = std.testing.allocator;

    // A small fan of triangles with user data and material indices, coordinates that quantize exactly
    const vertices = [_]Float3{ .init(0, 0, 0), .init(4, 0, 0), .init(4, 2, 0), .init(0, 2, 8), .init(2, 1, 4), .init(1, 1, 1) };
    const triangles = [_]IndexedTriangle{
        .init(0, 1, 2, .{ .material_index = 3, .user_data = 100 }),
        .init(0, 2, 3, .{ .material_index = 255, .user_data = 101 }),
        .init(4, 1, 3, .{ .material_index = 0, .user_data = 102 }),
        .init(3, 2, 4, .{ .material_index = 7, .user_data = 103 }),
        .init(4, 0, 1, .{ .material_index = 1, .user_data = 104 }),
    };

    for ([_]bool{ false, true }) |store_user_data| {
        var ctx = try Codec.EncodingContext.init(allocator, &vertices);
        defer ctx.deinit(allocator);

        // Prepare
        var size: u64 = Codec.triangle_header_size;
        ctx.preparePack(&triangles, store_user_data, &size);
        const expected_blocks_size: u64 = 4 + 2 * 16 + @as(u64, if (store_user_data) 5 * 4 else 0);
        try std.testing.expectEqual(Codec.triangle_header_size + expected_blocks_size, size);
        try ctx.finalizePreparePack(allocator, &size);
        try std.testing.expectEqual(@as(u32, 5), ctx.vertex_count); // Vertex 5 is not used
        try std.testing.expectEqual(Codec.triangle_header_size + expected_blocks_size + 5 * 8, size);

        // Pack
        var buffer: ByteBuffer = .empty;
        defer buffer.deinit(allocator);
        try buffer.vector.ensureTotalCapacityPrecise(allocator, @intCast(size));
        _ = try buffer.allocate(allocator, Codec.TriangleHeader, .{});
        const start = try ctx.pack(allocator, &triangles, store_user_data, &buffer);
        try std.testing.expectEqual(@as(usize, Codec.triangle_header_size), start);
        try ctx.finalize(allocator, &vertices, buffer.getMut(Codec.TriangleHeader, 0), &buffer);
        try std.testing.expectEqual(@as(usize, @intCast(size)), buffer.vector.items.len);

        // Vertices are stored in order of first use: first vertex of the 4 triangles of a block, then the second...
        const expected_order = [_]u32{ 0, 4, 3, 1, 2 };
        try std.testing.expectEqualSlices(u32, &expected_order, ctx.vertices.items);

        // Header
        const header = buffer.get(Codec.TriangleHeader, 0);
        try std.testing.expect(header.offset.eql(.init(0, 0, 0)));
        try std.testing.expect(header.scale.eql(.init(4.0 / @as(f32, Codec.component_mask), 2.0 / @as(f32, Codec.component_mask), 8.0 / @as(f32, Codec.component_mask))));
        const block_header = buffer.get(Codec.TriangleBlockHeader, start);
        try std.testing.expectEqual(store_user_data, block_header.getUserData() != null);

        // Decode
        const dec: Codec.DecodingContext = .init(header);
        const triangle_start: *const anyopaque = block_header;
        var unpacked: [15]Vec3 = undefined;
        var flags: [5]u8 = undefined;
        dec.unpackWithFlags(triangle_start, 5, &unpacked, &flags);
        for (triangles, 0..) |t, i| {
            const tv = dec.getTriangle(triangle_start, @intCast(i));
            const v = [_]Vec3{ tv.v1, tv.v2, tv.v3 };
            for (0..3) |j| {
                const expected = Vec3.fromFloat3(vertices[t.idx[j]]);
                try std.testing.expect(v[j].isClose(expected, .{ .max_dist_sq = 1.0e-10 }));
                try std.testing.expect(unpacked[3 * i + j].isClose(expected, .{ .max_dist_sq = 1.0e-10 }));
            }
            try std.testing.expectEqual(@as(u8, @intCast(t.material_index)), flags[i]);
            try std.testing.expectEqual(flags[i], Codec.DecodingContext.getTriangleFlags(triangle_start, @intCast(i)));
            try std.testing.expectEqual(if (store_user_data) t.user_data else 0, dec.getUserData(triangle_start, @intCast(i)));
        }

        // Ray test: hits triangle 0 at z = 0 (and triangle 4 further away)
        const hit = dec.testRay(Vec3.init(3, 0.5, -1), Vec3.init(0, 0, 2), triangle_start, 5, math.flt_max);
        try std.testing.expectApproxEqAbs(@as(f32, 0.5), hit.closest, 1.0e-6);
        try std.testing.expectEqual(@as(u32, 0), hit.closest_triangle_index);

        // Ray that misses
        const miss = dec.testRay(Vec3.init(10, 10, -1), Vec3.init(0, 0, 2), triangle_start, 5, 0.75);
        try std.testing.expectEqual(@as(f32, 0.75), miss.closest);

        // A closer limit than the hit
        const limited = dec.testRay(Vec3.init(3, 0.5, -1), Vec3.init(0, 0, 2), triangle_start, 5, 0.25);
        try std.testing.expectEqual(@as(f32, 0.25), limited.closest);
    }
}

test "TriangleCodecIndexed8BitPackSOA4Flags errors" {
    const allocator = std.testing.allocator;
    const vertices = [_]Float3{ .init(0, 0, 0), .init(1, 0, 0), .init(0, 1, 0) };

    // Material index doesn't fit in 8 bits
    {
        const triangles = [_]IndexedTriangle{.init(0, 1, 2, .{ .material_index = 256 })};
        var ctx = try Codec.EncodingContext.init(allocator, &vertices);
        defer ctx.deinit(allocator);
        var size: u64 = 0;
        ctx.preparePack(&triangles, false, &size);
        try ctx.finalizePreparePack(allocator, &size);
        var buffer: ByteBuffer = .empty;
        defer buffer.deinit(allocator);
        try std.testing.expectError(error.TriangleCodecMaterialIndexTooLarge, ctx.pack(allocator, &triangles, false, &buffer));
        try std.testing.expectEqualStrings("TriangleCodecIndexed8BitPackSOA4Flags: Material index doesn't fit in 8 bit", Codec.errorMessage(error.TriangleCodecMaterialIndexTooLarge));
    }

    // Too many vertices in one block: 100 triangles that don't share vertices
    {
        var many_vertices: [300]Float3 = undefined;
        for (&many_vertices, 0..) |*v, i| v.* = .init(@floatFromInt(i), @floatFromInt(i % 7), 0);
        var triangles: [100]IndexedTriangle = undefined;
        for (&triangles, 0..) |*t, i| t.* = .init(@intCast(3 * i), @intCast(3 * i + 1), @intCast(3 * i + 2), .{});
        var ctx = try Codec.EncodingContext.init(allocator, &many_vertices);
        defer ctx.deinit(allocator);
        var size: u64 = 0;
        ctx.preparePack(&triangles, false, &size);
        try ctx.finalizePreparePack(allocator, &size);
        var buffer: ByteBuffer = .empty;
        defer buffer.deinit(allocator);
        try std.testing.expectError(error.TriangleCodecVertexOffsetTooLarge, ctx.pack(allocator, &triangles, false, &buffer));
    }

    // Offset to the vertices must be a multiple of 4
    {
        const triangles = [_]IndexedTriangle{.init(0, 1, 2, .{})};
        var ctx = try Codec.EncodingContext.init(allocator, &vertices);
        defer ctx.deinit(allocator);
        var buffer: ByteBuffer = .empty;
        defer buffer.deinit(allocator);
        ctx.vertices_start_idx = 22;
        try std.testing.expectError(error.TriangleCodecOffsetNonSignificantBitsSet, ctx.pack(allocator, &triangles, false, &buffer));
    }
}

test "TriangleCodecIndexed8BitPackSOA4Flags validation" {
    // Vertices 0 and 1 quantize to the same value
    const vertices = [_]Float3{ .init(0, 0, 0), .init(1.0e-7, 0, 0), .init(0, 1, 0), .init(1, 1, 1) };
    const triangles = [_]IndexedTriangle{ .init(0, 1, 2, .{}), .init(0, 2, 3, .{}) };
    const ctx: Codec.ValidationContext = .init(&triangles, &vertices);
    try std.testing.expect(ctx.bounds.eql(.init(Vec3.zero(), Vec3.one())));
    try std.testing.expect(ctx.isDegenerate(triangles[0]));
    try std.testing.expect(!ctx.isDegenerate(triangles[1]));
    try std.testing.expect(ctx.isDegenerate(.init(3, 2, 3, .{})));
}
