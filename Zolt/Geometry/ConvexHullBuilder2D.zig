//! Port of: Jolt/Geometry/ConvexHullBuilder2D.h, Jolt/Geometry/ConvexHullBuilder2D.cpp
//! Status: complete
//! Not ported: JPH_CONVEX_BUILDER_2D_DEBUG (DrawState, cDrawScale, mOffset, mDelta)
//!
//! Jolt's constructor + `Initialize` become `init(allocator, positions)` + `initialize(...)`, the destructor is
//! `deinit()`. The allocator is used for the edges, their conflict lists and `out_edges`. Allocation failures
//! (which abort in Jolt) are returned as `error.OutOfMemory`, without leaking memory.

const std = @import("std");
const Core = @import("../Core/Core.zig");
const math = @import("../Math/Math.zig");
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;

/// A convex hull builder that tries to create 2D hulls as accurately as possible. Used for offline processing.
/// NonCopyable in Jolt: the builder owns its edges, don't copy it.
pub const ConvexHullBuilder2D = struct {
    pub const Positions = std.ArrayList(Vec3);
    pub const Edges = std.ArrayList(i32);

    /// Result enum that indicates how the hull got created
    pub const Result = enum {
        /// Hull building finished successfully
        success,
        /// Hull building finished successfully, but the desired accuracy was not reached because the max vertices limit was reached
        max_vertices_reached,
    };

    const ConflictList = std.ArrayList(i32);

    /// Linked list of edges
    const Edge = struct {
        /// Normal of the edge (not normalized)
        normal: Vec3 = undefined,
        /// Center of the edge
        center: Vec3 = undefined,
        /// Positions associated with this edge (that are closest to this edge). Last entry is the one furthest away from the edge, remainder is unsorted.
        conflict_list: ConflictList = .empty,
        /// Previous edge in circular list
        prev_edge: ?*Edge = null,
        /// Next edge in circular list
        next_edge: ?*Edge = null,
        /// Position index of start of this edge
        start_idx: i32,
        /// Squared distance of furthest point from the conflict list to the edge
        furthest_point_distance_sq: f32 = 0.0,

        /// Calculate the center of the edge and the edge normal
        fn calculateNormalAndCenter(self: *Edge, positions: []const Vec3) void {
            const p1 = positions[@intCast(self.start_idx)];
            const p2 = positions[@intCast(self.next_edge.?.start_idx)];

            // Center of edge
            self.center = p1.add(p2).mulScalar(0.5);

            // Create outward pointing normal.
            // We have two choices for the normal (which satisfies normal . edge = 0):
            // normal1 = (-edge.y, edge.x, 0)
            // normal2 = (edge.y, -edge.x, 0)
            // We want (normal x edge).z > 0 so that the normal points out of the polygon. Only normal2 satisfies this condition.
            const edge = p2.sub(p1);
            self.normal = Vec3.init(edge.getY(), -edge.getX(), 0);
        }

        /// Check if this edge is facing position
        fn isFacing(self: *const Edge, position: Vec3) bool {
            return self.normal.dot(position.sub(self.center)) > 0.0;
        }
    };

    allocator: std.mem.Allocator,
    /// List of positions (some of them are part of the hull)
    positions: []const Vec3,
    /// First edge of the hull
    first_edge: ?*Edge = null,
    /// Number of edges in hull
    num_edges: i32 = 0,

    /// Constructor
    /// @param positions Positions used to make the hull. Uses X and Y component of Vec3 only! Must outlive the builder.
    pub fn init(allocator: std.mem.Allocator, positions: []const Vec3) ConvexHullBuilder2D {
        return .{ .allocator = allocator, .positions = positions };
    }

    /// Destructor
    pub fn deinit(self: *ConvexHullBuilder2D) void {
        self.freeEdges();
    }

    /// Takes all positions as provided by the constructor and use them to build a hull
    /// Any points that are closer to the hull than tolerance will be discarded
    /// @param idx1 , idx2 , idx3 The indices to use as initial hull (in any order)
    /// @param max_vertices Max vertices to allow in the hull. Specify std.math.maxInt(i32) (INT_MAX) if there is no limit.
    /// @param tolerance Max distance that a point is allowed to be outside of the hull
    /// @param out_edges On success this will contain the list of indices that form the hull (counter clockwise). Allocated with the builder's allocator.
    /// @return Status code that reports if the hull was created or not
    pub fn initialize(self: *ConvexHullBuilder2D, idx1_in: i32, idx2_in: i32, idx3: i32, max_vertices: i32, tolerance: f32, out_edges: *Edges) error{OutOfMemory}!Result {
        var idx1 = idx1_in;
        var idx2 = idx2_in;

        // Clear any leftovers
        self.freeEdges();
        out_edges.clearRetainingCapacity();

        // Reset flag
        var result: Result = .success;

        // Determine a suitable tolerance for detecting that points are colinear
        // Formula as per: Implementing Quickhull - Dirk Gregorius.
        var vmax = Vec3.zero();
        for (self.positions) |v|
            vmax = Vec3.max(vmax, v.abs());
        const colinear_tolerance_sq = math.square(2.0 * math.flt_epsilon * (vmax.getX() + vmax.getY()));

        // Increase desired tolerance if accuracy doesn't allow it
        const tolerance_sq = math.max(colinear_tolerance_sq, math.square(tolerance));

        // Start with the initial indices in counter clockwise order
        const z = self.positionAt(idx2).sub(self.positionAt(idx1)).cross(self.positionAt(idx3).sub(self.positionAt(idx1))).getZ();
        if (z < 0.0)
            std.mem.swap(i32, &idx1, &idx2);

        // Create and link edges
        const edges = try self.createInitialEdges(idx1, idx2, idx3);

        // Build the initial conflict lists
        for (edges) |edge|
            edge.calculateNormalAndCenter(self.positions);
        var idx: i32 = 0;
        while (idx < @as(i32, @intCast(self.positions.len))) : (idx += 1) {
            if (idx != idx1 and idx != idx2 and idx != idx3)
                try self.assignPointToEdge(idx, &edges);
        }

        if (Core.enable_asserts) self.validateEdges();

        // Add the remaining points to the hull
        while (true) {
            // Check if we've reached the max amount of vertices that are allowed
            if (self.num_edges >= max_vertices) {
                result = .max_vertices_reached;
                break;
            }

            // Find the edge with the furthest point on it
            var edge_with_furthest_point: ?*Edge = null;
            var furthest_dist_sq: f32 = 0.0;
            var edge = self.first_edge.?;
            while (true) {
                if (edge.furthest_point_distance_sq > furthest_dist_sq) {
                    furthest_dist_sq = edge.furthest_point_distance_sq;
                    edge_with_furthest_point = edge;
                }
                edge = edge.next_edge.?;
                if (edge == self.first_edge.?) break;
            }

            // If there is none closer than our tolerance value, we're done
            if (edge_with_furthest_point == null or furthest_dist_sq < tolerance_sq)
                break;
            const furthest_edge = edge_with_furthest_point.?;

            // Take the furthest point
            const furthest_point_idx = furthest_edge.conflict_list.pop().?;
            const furthest_point = self.positionAt(furthest_point_idx);

            // Find the horizon of edges that need to be removed
            var first_edge = furthest_edge;
            while (true) {
                const prev = first_edge.prev_edge.?;
                if (!prev.isFacing(furthest_point))
                    break;
                first_edge = prev;
                if (first_edge == furthest_edge) break;
            }

            var last_edge = furthest_edge;
            while (true) {
                const next = last_edge.next_edge.?;
                if (!next.isFacing(furthest_point))
                    break;
                last_edge = next;
                if (last_edge == furthest_edge) break;
            }

            // Create new edges
            const e1 = try self.createEdge(first_edge.start_idx);
            const e2 = self.createEdge(furthest_point_idx) catch |err| {
                self.destroyEdge(e1);
                return err;
            };
            e1.next_edge = e2;
            e1.prev_edge = first_edge.prev_edge;
            e2.prev_edge = e1;
            e2.next_edge = last_edge.next_edge;
            e1.prev_edge.?.next_edge = e1;
            e2.next_edge.?.prev_edge = e2;
            self.first_edge = e1; // We could delete mFirstEdge so just update it to the newly created edge
            self.num_edges += 2;

            // Calculate normals
            const new_edges = [_]*Edge{ e1, e2 };
            for (new_edges) |new_edge|
                new_edge.calculateNormalAndCenter(self.positions);

            // Delete the old edges
            while (true) {
                const next = first_edge.next_edge.?;

                // Redistribute points in conflict list
                for (first_edge.conflict_list.items) |point_idx| {
                    self.assignPointToEdge(point_idx, &new_edges) catch |err| {
                        // The old edges are no longer part of the hull, free the ones that are left (Zolt only, Jolt aborts)
                        self.destroyEdgeRange(first_edge, last_edge);
                        return err;
                    };
                }

                // Delete the old edge
                self.destroyEdge(first_edge);
                self.num_edges -= 1;

                if (first_edge == last_edge)
                    break;
                first_edge = next;
            }

            if (Core.enable_asserts) self.validateEdges();
        }

        // Convert the edge list to a list of indices
        try out_edges.ensureTotalCapacity(self.allocator, @intCast(self.num_edges));
        var edge = self.first_edge.?;
        while (true) {
            try out_edges.append(self.allocator, edge.start_idx);
            edge = edge.next_edge.?;
            if (edge == self.first_edge.?) break;
        }

        return result;
    }

    /// Get the position with index idx
    fn positionAt(self: *const ConvexHullBuilder2D, idx: i32) Vec3 {
        return self.positions[@intCast(idx)];
    }

    /// Allocate a new edge (new Edge(inStartIdx))
    fn createEdge(self: *ConvexHullBuilder2D, start_idx: i32) error{OutOfMemory}!*Edge {
        const edge = try self.allocator.create(Edge);
        edge.* = .{ .start_idx = start_idx };
        return edge;
    }

    /// Free an edge (delete edge)
    fn destroyEdge(self: *ConvexHullBuilder2D, edge: *Edge) void {
        edge.conflict_list.deinit(self.allocator);
        self.allocator.destroy(edge);
    }

    /// Free the edges first .. last (following next_edge), used when an allocation fails while the old edges are being deleted
    fn destroyEdgeRange(self: *ConvexHullBuilder2D, first: *Edge, last: *Edge) void {
        var edge = first;
        while (true) {
            const next = edge.next_edge.?;
            self.destroyEdge(edge);
            if (edge == last)
                break;
            edge = next;
        }
    }

    /// Create the initial triangle of edges e1 -> e2 -> e3 and make it the hull
    fn createInitialEdges(self: *ConvexHullBuilder2D, idx1: i32, idx2: i32, idx3: i32) error{OutOfMemory}![3]*Edge {
        const e1 = try self.createEdge(idx1);
        errdefer self.destroyEdge(e1);
        const e2 = try self.createEdge(idx2);
        errdefer self.destroyEdge(e2);
        const e3 = try self.createEdge(idx3);
        e1.next_edge = e2;
        e1.prev_edge = e3;
        e2.next_edge = e3;
        e2.prev_edge = e1;
        e3.next_edge = e1;
        e3.prev_edge = e2;
        self.first_edge = e1;
        self.num_edges = 3;
        return .{ e1, e2, e3 };
    }

    /// Frees all edges
    fn freeEdges(self: *ConvexHullBuilder2D) void {
        const first = self.first_edge orelse return;

        var edge = first;
        while (true) {
            const next = edge.next_edge.?;
            self.destroyEdge(edge);
            edge = next;
            if (edge == first) break;
        }

        self.first_edge = null;
        self.num_edges = 0;
    }

    /// Validate that the edge structure is intact (only with asserts enabled)
    fn validateEdges(self: *const ConvexHullBuilder2D) void {
        const first = self.first_edge orelse {
            std.debug.assert(self.num_edges == 0);
            return;
        };

        var count: i32 = 0;

        var edge = first;
        while (true) {
            // Validate connectivity
            std.debug.assert(edge.next_edge.?.prev_edge.? == edge);
            std.debug.assert(edge.prev_edge.?.next_edge.? == edge);

            count += 1;
            edge = edge.next_edge.?;
            if (edge == first) break;
        }

        // Validate that count matches
        std.debug.assert(count == self.num_edges);
    }

    /// Assigns a position to one of the supplied edges based on which edge is closest.
    /// @param position_idx Index of the position to add
    /// @param edges List of edges to consider
    fn assignPointToEdge(self: *const ConvexHullBuilder2D, position_idx: i32, edges: []const *Edge) error{OutOfMemory}!void {
        const point = self.positionAt(position_idx);

        var best_edge: ?*Edge = null;
        var best_dist_sq: f32 = 0.0;

        // Test against all edges
        for (edges) |edge| {
            // Determine distance to edge
            const dot = edge.normal.dot(point.sub(edge.center));
            if (dot > 0.0) {
                const dist_sq = dot * dot / edge.normal.lengthSq();
                if (dist_sq > best_dist_sq) {
                    best_edge = edge;
                    best_dist_sq = dist_sq;
                }
            }
        }

        // If this point is in front of the edge, add it to the conflict list
        if (best_edge) |edge| {
            if (best_dist_sq > edge.furthest_point_distance_sq) {
                // This point is further away than any others, update the distance and add point as last point
                edge.furthest_point_distance_sq = best_dist_sq;
                try edge.conflict_list.append(self.allocator, position_idx);
            } else {
                // Not the furthest point, add it as the before last point
                try edge.conflict_list.insert(self.allocator, edge.conflict_list.items.len - 1, position_idx);
            }
        }
    }
};

/// Build a hull of `positions` and compare the result with `expected_edges` (test helper)
fn testHull(allocator: std.mem.Allocator, positions: []const Vec3, idx1: i32, idx2: i32, idx3: i32, max_vertices: i32, tolerance: f32, expected_result: ConvexHullBuilder2D.Result, expected_edges: []const i32) !void {
    var builder = ConvexHullBuilder2D.init(allocator, positions);
    defer builder.deinit();
    var edges: ConvexHullBuilder2D.Edges = .empty;
    defer edges.deinit(allocator);
    const result = try builder.initialize(idx1, idx2, idx3, max_vertices, tolerance, &edges);
    try std.testing.expectEqual(expected_result, result);
    try std.testing.expectEqualSlices(i32, expected_edges, edges.items);
}

test "ConvexHullBuilder2D square" {
    // Square with interior points, a point on an edge and duplicates
    const positions = [_]Vec3{
        Vec3.init(0, 0, 0), // 0: interior
        Vec3.init(-1, -1, 0), // 1
        Vec3.init(1, -1, 0), // 2
        Vec3.init(1, 1, 0), // 3
        Vec3.init(-1, 1, 0), // 4
        Vec3.init(0.5, 0.5, 0), // 5: interior
        Vec3.init(1, 0, 0), // 6: on an edge
        Vec3.init(1, 1, 0), // 7: duplicate of 3
        Vec3.init(-0.5, 0.2, 7), // 8: interior, Z is ignored
    };

    // Start with a clockwise triangle, it gets swapped to counter clockwise
    try testHull(std.testing.allocator, &positions, 4, 2, 1, std.math.maxInt(i32), 1.0e-3, .success, &.{ 2, 3, 4, 1 });

    // Counter clockwise start
    try testHull(std.testing.allocator, &positions, 0, 2, 3, std.math.maxInt(i32), 1.0e-3, .success, &.{ 3, 4, 1, 2 });

    // Limit the number of vertices
    try testHull(std.testing.allocator, &positions, 0, 2, 3, 4, 1.0e-3, .max_vertices_reached, &.{ 0, 1, 2, 3 });
    try testHull(std.testing.allocator, &positions, 0, 2, 3, 3, 1.0e-3, .max_vertices_reached, &.{ 0, 2, 3 });

    // Large tolerance: no points get added
    try testHull(std.testing.allocator, &positions, 1, 2, 3, std.math.maxInt(i32), 10, .success, &.{ 1, 2, 3 });
}

test "ConvexHullBuilder2D reuse and colinear" {
    // Colinear points: the hull degenerates into a line
    const positions = [_]Vec3{ Vec3.init(0, 0, 0), Vec3.init(1, 0, 0), Vec3.init(2, 0, 0), Vec3.init(3, 0, 0) };
    var builder = ConvexHullBuilder2D.init(std.testing.allocator, &positions);
    defer builder.deinit();
    var edges: ConvexHullBuilder2D.Edges = .empty;
    defer edges.deinit(std.testing.allocator);
    try std.testing.expectEqual(ConvexHullBuilder2D.Result.success, try builder.initialize(0, 1, 2, std.math.maxInt(i32), 0, &edges));
    try std.testing.expectEqualSlices(i32, &.{ 0, 1, 2 }, edges.items);

    // A second initialize frees the previous hull
    try std.testing.expectEqual(ConvexHullBuilder2D.Result.success, try builder.initialize(3, 0, 1, std.math.maxInt(i32), 0, &edges));
    try std.testing.expectEqualSlices(i32, &.{ 3, 0, 1 }, edges.items);
}

fn testHullAllocations(allocator: std.mem.Allocator) !void {
    // Circle of points with some interior points, needs many conflict list allocations and edge replacements
    var positions: [40]Vec3 = undefined;
    for (&positions, 0..) |*p, i| {
        const angle: f32 = @as(f32, @floatFromInt(i)) * 0.7;
        const radius: f32 = if (i % 3 == 0) 0.5 else 1.0;
        const sc = Vec4.replicate(angle).sinCos();
        p.* = Vec3.init(radius * sc.cos.getX(), radius * sc.sin.getX(), 0);
    }
    var builder = ConvexHullBuilder2D.init(allocator, &positions);
    defer builder.deinit();
    var edges: ConvexHullBuilder2D.Edges = .empty;
    defer edges.deinit(allocator);
    _ = try builder.initialize(1, 2, 4, std.math.maxInt(i32), 1.0e-4, &edges);
    try std.testing.expect(edges.items.len >= 3);
}

test "ConvexHullBuilder2D allocation failures" {
    // Every allocation failure must be reported without leaking memory
    try std.testing.checkAllAllocationFailures(std.testing.allocator, testHullAllocations, .{});
}
