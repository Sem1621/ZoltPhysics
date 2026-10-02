//! Port of: Jolt/Geometry/Indexify.h, Jolt/Geometry/Indexify.cpp
//! Status: complete
//!
//! The pointer ranges of the C++ implementation (`uint32 *ioVertexIndices` + count) are slices.

const std = @import("std");
const math = @import("../Math/Math.zig");
const Float3 = @import("../Math/Float3.zig").Float3;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const VertexList = @import("../Math/Float3.zig").VertexList;
const AABox = @import("AABox.zig").AABox;
const IndexedTriangle = @import("IndexedTriangle.zig").IndexedTriangle;
const IndexedTriangleList = @import("IndexedTriangle.zig").IndexedTriangleList;
const Triangle = @import("Triangle.zig").Triangle;
const TriangleList = @import("Triangle.zig").TriangleList;

fn indexifyGetFloat3(triangles: []const Triangle, vertex_index: u32) Float3 {
    return triangles[vertex_index / 3].v[vertex_index % 3];
}

fn indexifyGetVec3(triangles: []const Triangle, vertex_index: u32) Vec3 {
    return Vec3.loadFloat3Unsafe(&triangles[vertex_index / 3].v[vertex_index % 3]);
}

fn indexifyVerticesBruteForce(triangles: []const Triangle, vertex_indices: []const u32, welded_vertices: []u32, vertex_weld_distance: f32) void {
    const weld_dist_sq = math.square(vertex_weld_distance);

    // Compare every vertex
    for (vertex_indices, 0..) |v1_idx, i| {
        const v1 = indexifyGetVec3(triangles, v1_idx);

        // with every other vertex...
        for (vertex_indices[i + 1 ..]) |v2_idx| {
            const v2 = indexifyGetVec3(triangles, v2_idx);

            // If they're weldable
            if (v2.sub(v1).lengthSq() <= weld_dist_sq) {
                // Find the lowest indices both indices link to
                var idx1 = v1_idx;
                while (true) {
                    const new_idx1 = welded_vertices[idx1];
                    if (new_idx1 >= idx1)
                        break;
                    idx1 = new_idx1;
                }
                var idx2 = v2_idx;
                while (true) {
                    const new_idx2 = welded_vertices[idx2];
                    if (new_idx2 >= idx2)
                        break;
                    idx2 = new_idx2;
                }

                // Order the vertices
                const lowest = @min(idx1, idx2);
                const highest = @max(idx1, idx2);

                // Link highest to lowest
                welded_vertices[highest] = lowest;

                // Also update the vertices we started from to avoid creating long chains
                welded_vertices[v1_idx] = lowest;
                welded_vertices[v2_idx] = lowest;
                break;
            }
        }
    }
}

fn indexifyVerticesRecursively(triangles: []const Triangle, vertex_indices: []u32, scratch_buffer: []u32, welded_vertices: []u32, vertex_weld_distance: f32, max_recursion: u32) void {
    const num_vertices = vertex_indices.len;

    // Check if we have few enough vertices to do a brute force search
    // Or if we've recursed too deep (this means we chipped off a few vertices each iteration because all points are very close)
    if (num_vertices <= 8 or max_recursion == 0) {
        indexifyVerticesBruteForce(triangles, vertex_indices, welded_vertices, vertex_weld_distance);
        return;
    }

    // Calculate bounds
    var bounds: AABox = .empty;
    for (vertex_indices) |v|
        bounds.encapsulateVec3(indexifyGetVec3(triangles, v));

    // Determine split plane
    const split_axis = bounds.getExtent().getHighestComponentIndex();
    const split_value = bounds.getCenter().getComponent(split_axis);

    // Partition vertices
    var v_read: usize = 0;
    var v_write: usize = 0;
    var v_end: usize = num_vertices;
    var scratch: usize = 0;
    while (v_read < v_end) {
        // Calculate distance to plane
        const distance_to_split_plane = indexifyGetFloat3(triangles, vertex_indices[v_read]).getComponent(split_axis) - split_value;
        if (distance_to_split_plane < -vertex_weld_distance) {
            // Vertex is on the right side
            vertex_indices[v_write] = vertex_indices[v_read];
            v_read += 1;
            v_write += 1;
        } else if (distance_to_split_plane > vertex_weld_distance) {
            // Vertex is on the wrong side, swap with the last vertex
            v_end -= 1;
            std.mem.swap(u32, &vertex_indices[v_read], &vertex_indices[v_end]);
        } else {
            // Vertex is too close to the split plane, it goes on both sides
            scratch_buffer[scratch] = vertex_indices[v_read];
            scratch += 1;
            v_read += 1;
        }
    }

    // Check if we made any progress
    const num_vertices_on_both_sides = scratch;
    if (num_vertices_on_both_sides == num_vertices) {
        indexifyVerticesBruteForce(triangles, vertex_indices, welded_vertices, vertex_weld_distance);
        return;
    }

    // Calculate how we classified the vertices
    const num_vertices_left = v_write;
    const num_vertices_right = num_vertices - v_end;
    std.debug.assert(num_vertices_left + num_vertices_right + num_vertices_on_both_sides == num_vertices);
    @memcpy(vertex_indices[v_write .. v_write + num_vertices_on_both_sides], scratch_buffer[0..num_vertices_on_both_sides]);

    // Recurse
    const next_max_recursion = max_recursion - 1;
    indexifyVerticesRecursively(triangles, vertex_indices[0 .. num_vertices_left + num_vertices_on_both_sides], scratch_buffer, welded_vertices, vertex_weld_distance, next_max_recursion);
    indexifyVerticesRecursively(triangles, vertex_indices[num_vertices_left .. num_vertices_left + num_vertices_right + num_vertices_on_both_sides], scratch_buffer, welded_vertices, vertex_weld_distance, next_max_recursion);
}

/// Optional arguments of `indexify`
pub const IndexifyOptions = struct { vertex_weld_distance: f32 = 1.0e-4 };

/// Take a list of triangles and get the unique set of vertices and use them to create indexed triangles.
/// Vertices that are less than vertex_weld_distance apart will be combined to a single vertex.
/// `out_vertices` and `out_triangles` are cleared first and allocated with `allocator`.
pub fn indexify(allocator: std.mem.Allocator, triangles: []const Triangle, out_vertices: *VertexList, out_triangles: *IndexedTriangleList, opts: IndexifyOptions) std.mem.Allocator.Error!void {
    const num_triangles: u32 = @intCast(triangles.len);
    const num_vertices = num_triangles * 3;

    // Create a list of all vertex indices
    const vertex_indices = try allocator.alloc(u32, num_vertices);
    defer allocator.free(vertex_indices);
    for (vertex_indices, 0..) |*v, i|
        v.* = @intCast(i);

    // Link each vertex to itself
    const welded_vertices = try allocator.alloc(u32, num_vertices);
    defer allocator.free(welded_vertices);
    for (welded_vertices, 0..) |*v, i|
        v.* = @intCast(i);

    // A scope to free memory used by the scratch array
    {
        // Some scratch memory, used for the vertices that fall in both partitions
        const scratch = try allocator.alloc(u32, num_vertices);
        defer allocator.free(scratch);

        // Recursively split the vertices
        indexifyVerticesRecursively(triangles, vertex_indices, scratch, welded_vertices, opts.vertex_weld_distance, 32);
    }

    // Do a pass to complete the welding, linking each vertex to the vertex it is welded to
    // (and since we're going from 0 to N we can be sure that the vertex we're linking to is already linked to the lowest vertex)
    var num_resulting_vertices: u32 = 0;
    for (0..num_vertices) |i| {
        std.debug.assert(welded_vertices[welded_vertices[i]] <= welded_vertices[i]);
        welded_vertices[i] = welded_vertices[welded_vertices[i]];
        if (welded_vertices[i] == i)
            num_resulting_vertices += 1;
    }

    // Collect the vertices
    out_vertices.clearRetainingCapacity();
    try out_vertices.ensureTotalCapacity(allocator, num_resulting_vertices);
    for (0..num_vertices) |i| {
        if (welded_vertices[i] == i) {
            // New vertex
            welded_vertices[i] = @intCast(out_vertices.items.len);
            out_vertices.appendAssumeCapacity(indexifyGetFloat3(triangles, @intCast(i)));
        } else {
            // Reused vertex, remap index
            welded_vertices[i] = welded_vertices[welded_vertices[i]];
        }
    }

    // Create indexed triangles
    out_triangles.clearRetainingCapacity();
    try out_triangles.ensureTotalCapacity(allocator, num_triangles);
    for (0..num_triangles) |t| {
        var it: IndexedTriangle = .{ .idx = undefined };
        it.material_index = triangles[t].material_index;
        it.user_data = triangles[t].user_data;
        for (0..3) |v|
            it.idx[v] = welded_vertices[t * 3 + v];
        if (!it.isDegenerate(out_vertices.items))
            out_triangles.appendAssumeCapacity(it);
    }
}

/// Take a list of indexed triangles and unpack them. `out_triangles` is resized to the number of triangles.
pub fn deindexify(allocator: std.mem.Allocator, vertices: []const Float3, triangles: []const IndexedTriangle, out_triangles: *TriangleList) std.mem.Allocator.Error!void {
    try out_triangles.resize(allocator, triangles.len);
    for (triangles, out_triangles.items) |in, *out| {
        out.material_index = in.material_index;
        out.user_data = in.user_data;
        for (0..3) |v|
            out.v[v] = vertices[in.idx[v]];
    }
}

test "indexify / deindexify" {
    const allocator = std.testing.allocator;

    // Two triangles forming a quad, with slightly perturbed shared vertices, and a degenerate triangle
    const triangles = [_]Triangle{
        .init(Vec3.init(0, 0, 0), Vec3.init(1, 0, 0), Vec3.init(1, 1, 0), .{ .material_index = 1, .user_data = 10 }),
        .init(Vec3.init(0, 0, 0.00001), Vec3.init(1, 1.00001, 0), Vec3.init(0, 1, 0), .{ .material_index = 2, .user_data = 11 }),
        .init(Vec3.init(0, 0, 0), Vec3.init(0.00001, 0, 0), Vec3.init(1, 0, 0), .{ .material_index = 3, .user_data = 12 }),
    };

    var vertices: VertexList = .empty;
    defer vertices.deinit(allocator);
    var indexed: IndexedTriangleList = .empty;
    defer indexed.deinit(allocator);
    try indexify(allocator, &triangles, &vertices, &indexed, .{});

    try std.testing.expectEqualSlices(Float3, &.{ .init(0, 0, 0), .init(1, 0, 0), .init(1, 1, 0), .init(0, 1, 0) }, vertices.items);
    try std.testing.expectEqual(@as(usize, 2), indexed.items.len); // The degenerate triangle is removed
    try std.testing.expect(indexed.items[0].eql(.init(0, 1, 2, .{ .material_index = 1, .user_data = 10 })));
    try std.testing.expect(indexed.items[1].eql(.init(0, 2, 3, .{ .material_index = 2, .user_data = 11 })));

    // Without welding the perturbed vertices stay separate (only the exact duplicates are merged), the last triangle is still degenerate
    try indexify(allocator, &triangles, &vertices, &indexed, .{ .vertex_weld_distance = 0 });
    try std.testing.expectEqual(@as(usize, 7), vertices.items.len);
    try std.testing.expectEqual(@as(usize, 2), indexed.items.len);

    var out: TriangleList = .empty;
    defer out.deinit(allocator);
    try deindexify(allocator, vertices.items, indexed.items, &out);
    try std.testing.expectEqual(@as(usize, 2), out.items.len);
    for (triangles[0..2], out.items) |a, b| {
        try std.testing.expectEqual(a.material_index, b.material_index);
        try std.testing.expectEqual(a.user_data, b.user_data);
        for (a.v, b.v) |va, vb|
            try std.testing.expect(va.eql(vb));
    }
}

test "indexify many vertices" {
    // Enough vertices to take the recursive path: a grid of quads, each quad has its own copy of the vertices
    const allocator = std.testing.allocator;
    var triangles: TriangleList = .empty;
    defer triangles.deinit(allocator);
    const n = 20;
    for (0..n) |x| {
        for (0..n) |y| {
            const fx: f32 = @floatFromInt(x);
            const fy: f32 = @floatFromInt(y);
            try triangles.append(allocator, .init(Vec3.init(fx, fy, 0), Vec3.init(fx + 1, fy, 0), Vec3.init(fx + 1, fy + 1, 0), .{}));
            try triangles.append(allocator, .init(Vec3.init(fx, fy, 0), Vec3.init(fx + 1, fy + 1, 0), Vec3.init(fx, fy + 1, 0), .{}));
        }
    }

    var vertices: VertexList = .empty;
    defer vertices.deinit(allocator);
    var indexed: IndexedTriangleList = .empty;
    defer indexed.deinit(allocator);
    try indexify(allocator, triangles.items, &vertices, &indexed, .{});
    try std.testing.expectEqual(@as(usize, (n + 1) * (n + 1)), vertices.items.len);
    try std.testing.expectEqual(@as(usize, 2 * n * n), indexed.items.len);

    // Unpacking gives the original triangles
    var out: TriangleList = .empty;
    defer out.deinit(allocator);
    try deindexify(allocator, vertices.items, indexed.items, &out);
    for (triangles.items, out.items) |a, b|
        for (a.v, b.v) |va, vb|
            try std.testing.expect(va.eql(vb));
}
