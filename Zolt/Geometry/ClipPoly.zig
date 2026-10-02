//! Port of: Jolt/Geometry/ClipPoly.h
//! Status: complete
//!
//! Jolt's `template <class VERTEX_ARRAY>` functions: the input polygons (`const VERTEX_ARRAY &`) are slices, the
//! output polygon is a vertex array (`*StaticArray(Vec3, N)` or `VertexArrayList`, see VertexArray.zig). Temporary
//! polygons have the same type as the output polygon. Like in Jolt, the polygon to clip may be the output polygon in
//! `clipPolyVsPoly` / `clipPolyVsAABox` (they only write the output after the last read of the input), but the
//! clipping polygon must not be.
//! The functions return `VertexArray.Error(@TypeOf(out))!void`, which is an empty error set for a StaticArray, so
//! `try` never fails for it (and also compiles in a function that does not return an error union).

const std = @import("std");
const StaticArray = @import("../Core/StaticArray.zig").StaticArray;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const AABox = @import("AABox.zig").AABox;
const VertexArray = @import("VertexArray.zig");

/// Clip polygon_to_clip against the positive halfspace of plane defined by plane_origin and plane_normal.
/// plane_normal does not need to be normalized.
pub fn clipPolyVsPlane(polygon_to_clip: []const Vec3, plane_origin: Vec3, plane_normal: Vec3, clipped_polygon: anytype) VertexArray.Error(@TypeOf(clipped_polygon))!void {
    std.debug.assert(polygon_to_clip.len >= 2);
    std.debug.assert(VertexArray.len(clipped_polygon) == 0);

    // Determine state of last point
    var e1 = polygon_to_clip[polygon_to_clip.len - 1];
    var prev_num = plane_origin.sub(e1).dot(plane_normal);
    var prev_inside = prev_num < 0.0;

    // Loop through all vertices
    for (polygon_to_clip) |e2| {
        // Check if second point is inside
        const num = plane_origin.sub(e2).dot(plane_normal);
        var cur_inside = num < 0.0;

        // In -> Out or Out -> In: Add point on clipping plane
        if (cur_inside != prev_inside) {
            // Solve: (X - plane_origin) . plane_normal = 0 and X = e1 + t * (e2 - e1) for X
            const e12 = e2.sub(e1);
            const denom = e12.dot(plane_normal);
            if (denom != 0.0)
                try VertexArray.append(clipped_polygon, e1.add(e12.mulScalar(prev_num / denom)))
            else
                cur_inside = prev_inside; // Edge is parallel to plane, treat point as if it were on the same side as the last point
        }

        // Point inside, add it
        if (cur_inside)
            try VertexArray.append(clipped_polygon, e2);

        // Update previous state
        prev_num = num;
        prev_inside = cur_inside;
        e1 = e2;
    }
}

/// Clip polygon versus polygon.
/// Both polygons are assumed to be in counter clockwise order.
/// @param clipping_polygon_normal is used to create planes of all edges in clipping_polygon against which polygon_to_clip is clipped, clipping_polygon_normal does not need to be normalized
/// @param clipping_polygon is the polygon which polygon_to_clip is clipped against
/// @param polygon_to_clip is the polygon that is clipped
/// @param clipped_polygon will contain clipped polygon when function returns
pub fn clipPolyVsPoly(polygon_to_clip: []const Vec3, clipping_polygon: []const Vec3, clipping_polygon_normal: Vec3, clipped_polygon: anytype) VertexArray.Error(@TypeOf(clipped_polygon))!void {
    std.debug.assert(polygon_to_clip.len >= 2);
    std.debug.assert(clipping_polygon.len >= 3);

    var tmp_vertices: [2]VertexArray.Temporary(@TypeOf(clipped_polygon)) = .{ .empty, .empty };
    defer for (&tmp_vertices) |*tmp| VertexArray.deinitTemporary(clipped_polygon, tmp);
    var tmp_vertices_idx: usize = 0;

    for (0..clipping_polygon.len) |i| {
        // Get edge to clip against
        const clip_e1 = clipping_polygon[i];
        const clip_e2 = clipping_polygon[(i + 1) % clipping_polygon.len];
        const clip_normal = clipping_polygon_normal.cross(clip_e2.sub(clip_e1)); // Pointing inward to the clipping polygon

        // Get source and target polygon
        const src_polygon: []const Vec3 = if (i == 0) polygon_to_clip else VertexArray.items(VertexArray.temporary(clipped_polygon, &tmp_vertices[tmp_vertices_idx]));
        tmp_vertices_idx ^= 1;
        const tgt_polygon = if (i == clipping_polygon.len - 1) clipped_polygon else VertexArray.temporary(clipped_polygon, &tmp_vertices[tmp_vertices_idx]);
        VertexArray.clear(tgt_polygon);

        // Clip against the edge
        try clipPolyVsPlane(src_polygon, clip_e1, clip_normal, tgt_polygon);

        // Break out if no polygon left
        if (VertexArray.len(tgt_polygon) < 3) {
            VertexArray.clear(clipped_polygon);
            break;
        }
    }
}

/// Clip polygon_to_clip against an edge, the edge is projected on polygon_to_clip using clipping_edge_normal.
/// The positive half space (the side on the edge in the direction of clipping_edge_normal) is cut away.
pub fn clipPolyVsEdge(polygon_to_clip: []const Vec3, edge_vertex1: Vec3, edge_vertex2: Vec3, clipping_edge_normal: Vec3, clipped_polygon: anytype) VertexArray.Error(@TypeOf(clipped_polygon))!void {
    std.debug.assert(polygon_to_clip.len >= 3);
    std.debug.assert(VertexArray.len(clipped_polygon) == 0);

    // Get normal that is perpendicular to the edge and the clipping edge normal
    const edge = edge_vertex2.sub(edge_vertex1);
    const edge_normal = clipping_edge_normal.cross(edge);

    // Project vertices of edge on polygon_to_clip
    const polygon_normal = polygon_to_clip[2].sub(polygon_to_clip[0]).cross(polygon_to_clip[1].sub(polygon_to_clip[0]));
    const polygon_normal_len_sq = polygon_normal.lengthSq();
    const v1 = edge_vertex1.add(polygon_normal.mulScalar(polygon_normal.dot(polygon_to_clip[0].sub(edge_vertex1))).divScalar(polygon_normal_len_sq));
    const v2 = edge_vertex2.add(polygon_normal.mulScalar(polygon_normal.dot(polygon_to_clip[0].sub(edge_vertex2))).divScalar(polygon_normal_len_sq));
    const v12 = v2.sub(v1);
    const v12_len_sq = v12.lengthSq();

    // Determine state of last point
    var e1 = polygon_to_clip[polygon_to_clip.len - 1];
    var prev_num = edge_vertex1.sub(e1).dot(edge_normal);
    var prev_inside = prev_num < 0.0;

    // Loop through all vertices
    for (polygon_to_clip) |e2| {
        // Check if second point is inside
        const num = edge_vertex1.sub(e2).dot(edge_normal);
        const cur_inside = num < 0.0;

        // In -> Out or Out -> In: Add point on clipping plane
        if (cur_inside != prev_inside) {
            // Solve: (edge_vertex1 - X) . edge_normal = 0 and X = e1 + t * (e2 - e1) for X
            const e12 = e2.sub(e1);
            const denom = e12.dot(edge_normal);
            const clipped_point = if (denom != 0.0) e1.add(e12.mulScalar(prev_num / denom)) else e1;

            // Project point on line segment v1, v2 so see if it falls outside if the edge
            const projection = clipped_point.sub(v1).dot(v12);
            if (projection < 0.0)
                try VertexArray.append(clipped_polygon, v1)
            else if (projection > v12_len_sq)
                try VertexArray.append(clipped_polygon, v2)
            else
                try VertexArray.append(clipped_polygon, clipped_point);
        }

        // Update previous state
        prev_num = num;
        prev_inside = cur_inside;
        e1 = e2;
    }
}

/// Clip polygon vs axis aligned box, polygon_to_clip is assume to be in counter clockwise order.
/// Output will be stored in clipped_polygon. Everything inside aabox will be kept.
pub fn clipPolyVsAABox(polygon_to_clip: []const Vec3, aabox: AABox, clipped_polygon: anytype) VertexArray.Error(@TypeOf(clipped_polygon))!void {
    std.debug.assert(polygon_to_clip.len >= 2);

    var tmp_vertices: [2]VertexArray.Temporary(@TypeOf(clipped_polygon)) = .{ .empty, .empty };
    defer for (&tmp_vertices) |*tmp| VertexArray.deinitTemporary(clipped_polygon, tmp);
    var tmp_vertices_idx: usize = 0;

    for (0..3) |coord_index| {
        const coord: u32 = @intCast(coord_index);
        for (0..2) |side| {
            // Get plane to clip against
            var origin = Vec3.zero();
            var normal = Vec3.zero();
            if (side == 0) {
                normal.setComponent(coord, 1.0);
                origin.setComponent(coord, aabox.min.getComponent(coord));
            } else {
                normal.setComponent(coord, -1.0);
                origin.setComponent(coord, aabox.max.getComponent(coord));
            }

            // Get source and target polygon
            const src_polygon: []const Vec3 = if (tmp_vertices_idx == 0) polygon_to_clip else VertexArray.items(VertexArray.temporary(clipped_polygon, &tmp_vertices[tmp_vertices_idx & 1]));
            tmp_vertices_idx += 1;
            const tgt_polygon = if (tmp_vertices_idx == 6) clipped_polygon else VertexArray.temporary(clipped_polygon, &tmp_vertices[tmp_vertices_idx & 1]);
            VertexArray.clear(tgt_polygon);

            // Clip against the edge
            try clipPolyVsPlane(src_polygon, origin, normal, tgt_polygon);

            // Break out if no polygon left
            if (VertexArray.len(tgt_polygon) < 3) {
                VertexArray.clear(clipped_polygon);
                return;
            }

            // Flip normal (Jolt does this too, the value is not used: normal is recalculated in the next iteration)
            normal = normal.negate();
        }
    }
}

test "clipPolyVsPlane" {
    const square = [_]Vec3{ .init(-1, -1, 0), .init(1, -1, 0), .init(1, 1, 0), .init(-1, 1, 0) };

    // Keep x > 0.5
    var out: StaticArray(Vec3, 8) = .empty;
    try clipPolyVsPlane(&square, Vec3.init(0.5, 0, 0), Vec3.init(1, 0, 0), &out);
    try std.testing.expectEqualSlices(Vec3, &.{ .init(0.5, -1, 0), .init(1, -1, 0), .init(1, 1, 0), .init(0.5, 1, 0) }, out.constSlice());

    // Everything is clipped away
    out.clear();
    try clipPolyVsPlane(&square, Vec3.init(2, 0, 0), Vec3.init(1, 0, 0), &out);
    try std.testing.expectEqual(@as(u32, 0), out.len);

    // Same with a std.ArrayList
    const allocator = std.testing.allocator;
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(allocator);
    try clipPolyVsPlane(&square, Vec3.init(0.5, 0, 0), Vec3.init(1, 0, 0), VertexArray.VertexArrayList.init(allocator, &list));
    try std.testing.expectEqualSlices(Vec3, &.{ .init(0.5, -1, 0), .init(1, -1, 0), .init(1, 1, 0), .init(0.5, 1, 0) }, list.items);
}

test "clipPolyVsPoly" {
    const square = [_]Vec3{ .init(-1, -1, 0), .init(1, -1, 0), .init(1, 1, 0), .init(-1, 1, 0) };
    const triangle = [_]Vec3{ .init(0, -1.5, 0), .init(1.5, 0, 0), .init(0, 1.5, 0) };

    // The square cuts off the 3 corners of the triangle
    var out: StaticArray(Vec3, 16) = .empty;
    try clipPolyVsPoly(&triangle, &square, Vec3.init(0, 0, 1), &out);
    try std.testing.expectEqual(@as(u32, 6), out.len);
    for (out.constSlice()) |v| {
        try std.testing.expect(v.getX() >= 0 and v.getX() <= 1 and v.getY() >= -1 and v.getY() <= 1);
        try std.testing.expect(v.getX() + @abs(v.getY()) <= 1.5);
    }

    const allocator = std.testing.allocator;
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(allocator);
    try clipPolyVsPoly(&triangle, &square, Vec3.init(0, 0, 1), VertexArray.VertexArrayList.init(allocator, &list));
    try std.testing.expectEqualSlices(Vec3, out.constSlice(), list.items);

    // The output can be the polygon to clip (like in Jolt)
    var in_place: StaticArray(Vec3, 16) = .fromSlice(&triangle);
    try clipPolyVsPoly(in_place.constSlice(), &square, Vec3.init(0, 0, 1), &in_place);
    try std.testing.expectEqualSlices(Vec3, out.constSlice(), in_place.constSlice());
    var in_place_list: std.ArrayList(Vec3) = .empty;
    defer in_place_list.deinit(allocator);
    try in_place_list.appendSlice(allocator, &triangle);
    try clipPolyVsPoly(in_place_list.items, &square, Vec3.init(0, 0, 1), VertexArray.VertexArrayList.init(allocator, &in_place_list));
    try std.testing.expectEqualSlices(Vec3, out.constSlice(), in_place_list.items);

    // No overlap: empty result
    const far_triangle = [_]Vec3{ .init(10, 10, 0), .init(11, 10, 0), .init(10, 11, 0) };
    try clipPolyVsPoly(&far_triangle, &square, Vec3.init(0, 0, 1), &out);
    try std.testing.expectEqual(@as(u32, 0), out.len);
    try clipPolyVsPoly(&far_triangle, &square, Vec3.init(0, 0, 1), VertexArray.VertexArrayList.init(allocator, &list));
    try std.testing.expectEqual(@as(usize, 0), list.items.len);
}

test "clipPolyVsEdge" {
    const square = [_]Vec3{ .init(-1, -1, 0), .init(1, -1, 0), .init(1, 1, 0), .init(-1, 1, 0) };

    // Edge along Y at x = 0.5, cut away the +X side
    var out: StaticArray(Vec3, 8) = .empty;
    try clipPolyVsEdge(&square, Vec3.init(0.5, -0.5, 1), Vec3.init(0.5, 0.5, 1), Vec3.init(0, 0, 1), &out);
    try std.testing.expectEqualSlices(Vec3, &.{ .init(0.5, -0.5, 0), .init(0.5, 0.5, 0) }, out.constSlice());

    const allocator = std.testing.allocator;
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(allocator);
    try clipPolyVsEdge(&square, Vec3.init(0.5, -0.5, 1), Vec3.init(0.5, 0.5, 1), Vec3.init(0, 0, 1), VertexArray.VertexArrayList.init(allocator, &list));
    try std.testing.expectEqualSlices(Vec3, out.constSlice(), list.items);
}

test "clipPolyVsAABox" {
    const triangle = [_]Vec3{ .init(-2, 0, 0.5), .init(2, 0, 0.5), .init(0, 2, 0.5) };
    const box = AABox.init(Vec3.init(-1, -1, 0), Vec3.init(1, 1, 1));

    var out: StaticArray(Vec3, 16) = .empty;
    try clipPolyVsAABox(&triangle, box, &out);
    try std.testing.expect(out.len >= 3);
    for (out.constSlice()) |v|
        try std.testing.expect(box.containsVec3(v));

    const allocator = std.testing.allocator;
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(allocator);
    try clipPolyVsAABox(&triangle, box, VertexArray.VertexArrayList.init(allocator, &list));
    try std.testing.expectEqualSlices(Vec3, out.constSlice(), list.items);

    // The output can be the polygon to clip (like in Jolt)
    var in_place: StaticArray(Vec3, 16) = .fromSlice(&triangle);
    try clipPolyVsAABox(in_place.constSlice(), box, &in_place);
    try std.testing.expectEqualSlices(Vec3, out.constSlice(), in_place.constSlice());

    // Outside the box: empty result
    try clipPolyVsAABox(&triangle, AABox.init(Vec3.init(5, 5, 5), Vec3.init(6, 6, 6)), &out);
    try std.testing.expectEqual(@as(u32, 0), out.len);
    try clipPolyVsAABox(&triangle, AABox.init(Vec3.init(5, 5, 5), Vec3.init(6, 6, 6)), VertexArray.VertexArrayList.init(allocator, &list));
    try std.testing.expectEqual(@as(usize, 0), list.items.len);
}
