//! Port of: Jolt/Geometry/EPAConvexHullBuilder.h
//! Status: complete
//! Not ported: JPH_EPA_CONVEX_BUILDER_VALIDATE (ValidateTriangle, ValidateTriangles, mTriangles), JPH_EPA_CONVEX_BUILDER_DRAW (cDrawScale, DrawState, DrawLabel, DrawGeometry, DrawWireTriangle, DrawMarker, DrawArrow, Triangle::mIteration, mIteration, mOffset)
//!
//! The triangles live in the fixed size buffer of the `TriangleFactory` inside the builder and point to each other, so
//! the builder must not be moved after `initialize` (NonCopyable in Jolt). Jolt's `Points` class (a `StaticArray` with
//! `GetSizeRef()`) is the `StaticArray` itself in Zolt: its `len` field is public, `GetSizeRef()` is `&points.len`.

const std = @import("std");
const builtin = @import("builtin");
const Core = @import("../Core/Core.zig");
const math = @import("../Math/Math.zig");
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const StaticArray = @import("../Core/StaticArray.zig").StaticArray;
const BinaryHeap = @import("../Core/BinaryHeap.zig");

/// A convex hull builder specifically made for the EPA penetration depth calculation. It trades accuracy for speed and will simply abort of the hull forms defects due to numerical precision problems.
/// NonCopyable in Jolt: don't copy or move it after `initialize`.
pub const EPAConvexHullBuilder = struct {
    // Due to the Euler characteristic (https://en.wikipedia.org/wiki/Euler_characteristic) we know that Vertices - Edges + Faces = 2
    // In our case we only have triangles and they are always fully connected, so each edge is shared exactly between 2 faces: Edges = Faces * 3 / 2
    // Substituting: Vertices = Faces / 2 + 2 which is approximately Faces / 2.
    /// Max triangles in hull
    pub const max_triangles: u32 = 256;
    /// Max number of points in hull
    pub const max_points: u32 = max_triangles / 2;

    // Constants
    /// Max number of edges in FindEdge
    pub const max_edge_length: u32 = 128;
    /// Minimum area of a triangle before, if smaller than this it will not be added to the priority queue
    pub const min_triangle_area: f32 = 1.0e-10;
    /// Epsilon value used to determine if a point is in the interior of a triangle
    pub const barycentric_epsilon: f32 = 1.0e-3;

    /// Class that holds the information of an edge
    pub const Edge = extern struct {
        /// Information about neighbouring triangle
        /// Triangle that neighbours this triangle
        neighbour_triangle: ?*Triangle,
        /// Index in edge that specifies edge that this Edge is connected to
        neighbour_edge: u32,

        /// Vertex index in positions that indicates the start vertex of this edge
        start_idx: u32,
    };

    pub const Edges = StaticArray(Edge, max_edge_length);
    pub const NewTriangles = StaticArray(*Triangle, max_edge_length);

    /// Class that holds the information of one triangle
    /// NonCopyable in Jolt: triangles are created in place by the TriangleFactory and are referenced by pointer.
    pub const Triangle = extern struct {
        /// 3 edges of this triangle
        edge: [3]Edge,
        /// Normal of this triangle, length is 2 times area of triangle
        normal: Vec3,
        /// Center of the triangle
        centroid: Vec3,
        /// Closest distance^2 from origin to triangle
        closest_len_sq: f32 = math.flt_max,
        /// Barycentric coordinates of closest point to origin on triangle
        lambda: [2]f32,
        /// How to calculate the closest point, true: y0 + l0 * (y1 - y0) + l1 * (y2 - y0), false: y1 + l0 * (y0 - y1) + l1 * (y2 - y1)
        lambda_relative_to_0: bool,
        /// Flag that indicates that the closest point from this triangle to the origin is an interior point
        closest_point_interior: bool = false,
        /// Flag that indicates that triangle has been removed
        removed: bool = false,
        /// Flag that indicates that this triangle was placed in the sorted heap (stays true after it is popped because the triangle is freed by the main EPA algorithm loop)
        in_queue: bool = false,

        /// Constructor. Constructs the triangle in place, like the placement new in Jolt's TriangleFactory::CreateTriangle:
        /// `lambda` and `lambda_relative_to_0` are only written when they are calculated (Jolt leaves them uninitialized
        /// otherwise, they are only used when closest_point_interior is set).
        pub fn init(self: *Triangle, idx0: u32, idx1: u32, idx2: u32, positions: []const Vec3) void {
            // Default member initializers
            self.closest_len_sq = math.flt_max;
            self.closest_point_interior = false;
            self.removed = false;
            self.in_queue = false;

            // Fill in indexes
            std.debug.assert(idx0 != idx1 and idx0 != idx2 and idx1 != idx2);
            self.edge[0].start_idx = idx0;
            self.edge[1].start_idx = idx1;
            self.edge[2].start_idx = idx2;

            // Clear links
            self.edge[0].neighbour_triangle = null;
            self.edge[1].neighbour_triangle = null;
            self.edge[2].neighbour_triangle = null;

            // Get vertex positions
            const y0 = positions[idx0];
            const y1 = positions[idx1];
            const y2 = positions[idx2];

            // Calculate centroid
            self.centroid = y0.add(y1).add(y2).divScalar(3.0);

            // Calculate edges
            const y10 = y1.sub(y0);
            const y20 = y2.sub(y0);
            const y21 = y2.sub(y1);

            // The most accurate normal is calculated by using the two shortest edges
            // See: https://box2d.org/posts/2014/01/troublesome-triangle/
            // The difference in normals is most pronounced when one edge is much smaller than the others (in which case the other 2 must have roughly the same length).
            // Therefore we can suffice by just picking the shortest from 2 edges and use that with the 3rd edge to calculate the normal.
            // We first check which of the edges is shorter.
            const y20_dot_y20 = y20.dot(y20);
            const y21_dot_y21 = y21.dot(y21);
            if (y20_dot_y20 < y21_dot_y21) {
                // We select the edges y10 and y20
                self.normal = y10.crossPrecise(y20);

                // Check if triangle is degenerate
                const normal_len_sq = self.normal.lengthSq();
                if (normal_len_sq > min_triangle_area) {
                    // Determine distance between triangle and origin: distance = (centroid - origin) . normal / |normal|
                    // Note that this way of calculating the closest point is much more accurate than first calculating barycentric coordinates and then calculating the closest
                    // point based on those coordinates. Note that we preserve the sign of the distance to check on which side the origin is.
                    const c_dot_n = self.centroid.dot(self.normal);
                    self.closest_len_sq = @abs(c_dot_n) * c_dot_n / normal_len_sq;

                    // Calculate closest point to origin using barycentric coordinates:
                    //
                    // v = y0 + l0 * (y1 - y0) + l1 * (y2 - y0)
                    // v . (y1 - y0) = 0
                    // v . (y2 - y0) = 0
                    //
                    // Written in matrix form:
                    //
                    // | y10.y10  y20.y10 | | l0 | = | -y0.y10 |
                    // | y10.y20  y20.y20 | | l1 |   | -y0.y20 |
                    //
                    // (y10 = y1 - y0 etc.)
                    //
                    // Cramers rule to invert matrix:
                    const y10_dot_y10 = y10.lengthSq();
                    const y10_dot_y20 = y10.dot(y20);
                    const determinant = math.differenceOfProducts(y10_dot_y10, y20_dot_y20, y10_dot_y20, y10_dot_y20);
                    if (determinant > 0.0) // If determinant == 0 then the system is linearly dependent and the triangle is degenerate, since y10.10 * y20.y20 > y10.y20^2 it should also be > 0
                    {
                        const y0_dot_y10 = y0.dot(y10);
                        const y0_dot_y20 = y0.dot(y20);
                        const l0 = math.differenceOfProducts(y10_dot_y20, y0_dot_y20, y20_dot_y20, y0_dot_y10) / determinant;
                        const l1 = math.differenceOfProducts(y10_dot_y20, y0_dot_y10, y10_dot_y10, y0_dot_y20) / determinant;
                        self.lambda[0] = l0;
                        self.lambda[1] = l1;
                        self.lambda_relative_to_0 = true;

                        // Check if closest point is interior to the triangle. For a convex hull which contains the origin each face must contain the origin, but because
                        // our faces are triangles, we can have multiple coplanar triangles and only 1 will have the origin as an interior point. We want to use this triangle
                        // to calculate the contact points because it gives the most accurate results, so we will only add these triangles to the priority queue.
                        if (l0 > -barycentric_epsilon and l1 > -barycentric_epsilon and l0 + l1 < 1.0 + barycentric_epsilon)
                            self.closest_point_interior = true;
                    }
                }
            } else {
                // We select the edges y10 and y21
                self.normal = y10.crossPrecise(y21);

                // Check if triangle is degenerate
                const normal_len_sq = self.normal.lengthSq();
                if (normal_len_sq > min_triangle_area) {
                    // Again calculate distance between triangle and origin
                    const c_dot_n = self.centroid.dot(self.normal);
                    self.closest_len_sq = @abs(c_dot_n) * c_dot_n / normal_len_sq;

                    // Calculate closest point to origin using barycentric coordinates but this time using y1 as the reference vertex
                    //
                    // v = y1 + l0 * (y0 - y1) + l1 * (y2 - y1)
                    // v . (y0 - y1) = 0
                    // v . (y2 - y1) = 0
                    //
                    // Written in matrix form:
                    //
                    // | y10.y10  -y21.y10 | | l0 | = | y1.y10 |
                    // | -y10.y21  y21.y21 | | l1 |   | -y1.y21 |
                    //
                    // Cramers rule to invert matrix:
                    const y10_dot_y10 = y10.lengthSq();
                    const y10_dot_y21 = y10.dot(y21);
                    const determinant = math.differenceOfProducts(y10_dot_y10, y21_dot_y21, y10_dot_y21, y10_dot_y21);
                    if (determinant > 0.0) {
                        const y1_dot_y10 = y1.dot(y10);
                        const y1_dot_y21 = y1.dot(y21);
                        const l0 = math.differenceOfProducts(y21_dot_y21, y1_dot_y10, y10_dot_y21, y1_dot_y21) / determinant;
                        const l1 = math.differenceOfProducts(y10_dot_y21, y1_dot_y10, y10_dot_y10, y1_dot_y21) / determinant;
                        self.lambda[0] = l0;
                        self.lambda[1] = l1;
                        self.lambda_relative_to_0 = false;

                        // Again check if the closest point is inside the triangle
                        if (l0 > -barycentric_epsilon and l1 > -barycentric_epsilon and l0 + l1 < 1.0 + barycentric_epsilon)
                            self.closest_point_interior = true;
                    }
                }
            }
        }

        /// Check if triangle is facing position
        pub fn isFacing(self: *const Triangle, position: Vec3) bool {
            std.debug.assert(!self.removed);
            return self.normal.dot(position.sub(self.centroid)) > 0.0;
        }

        /// Check if triangle is facing the origin
        pub fn isFacingOrigin(self: *const Triangle) bool {
            std.debug.assert(!self.removed);
            return self.normal.dot(self.centroid) < 0.0;
        }

        /// Get the next edge of edge index
        pub fn getNextEdge(self: *const Triangle, index: u32) *const Edge {
            return &self.edge[(index + 1) % 3];
        }
    };

    /// Factory that creates triangles in a fixed size buffer
    /// NonCopyable in Jolt: the free list points into the buffer.
    pub const TriangleFactory = struct {
        /// Struct that stores both a triangle or a next pointer in case the triangle is unused
        const Block = extern union {
            triangle: Triangle,
            next_free: ?*Block,
        };

        /// Storage for triangles
        triangles: [max_triangles]Block = undefined,
        /// List of free triangles
        next_free: ?*Block = null,
        /// High water mark for used triangles (if next_free == null we can take one from here)
        high_watermark: u32 = 0,

        /// Return all triangles to the free pool
        pub fn clear(self: *TriangleFactory) void {
            self.next_free = null;
            self.high_watermark = 0;
        }

        /// Allocate a new triangle with 3 indexes
        pub fn createTriangle(self: *TriangleFactory, idx0: u32, idx1: u32, idx2: u32, positions: []const Vec3) ?*Triangle {
            var t: *Triangle = undefined;
            if (self.next_free) |next_free| {
                // Entry available from the free list
                t = &next_free.triangle;
                self.next_free = next_free.next_free;
            } else {
                // Allocate from never used before triangle store
                if (self.high_watermark >= max_triangles)
                    return null; // Buffer full
                t = &self.triangles[self.high_watermark].triangle;
                self.high_watermark += 1;
            }

            // Call constructor
            t.init(idx0, idx1, idx2, positions);

            return t;
        }

        /// Free a triangle
        pub fn freeTriangle(self: *TriangleFactory, t: *Triangle) void {
            // Destruct triangle (trivial)
            if (builtin.mode == .Debug)
                @memset(std.mem.asBytes(t), 0xcd);

            // Add triangle to the free list
            const tu: *Block = @ptrCast(t);
            tu.next_free = self.next_free;
            self.next_free = tu;
        }
    };

    // Typedefs
    pub const PointsBase = StaticArray(Vec3, max_points);
    pub const Triangles = StaticArray(*Triangle, max_triangles);

    /// Specialized points list that allows direct access to the size (GetSizeRef() is `&points.len`)
    pub const Points = PointsBase;

    /// Specialized triangles list that keeps them sorted on closest distance to origin
    pub const TriangleQueue = struct {
        /// The triangles (Jolt's base class `Triangles`), a binary heap
        triangles: Triangles = .empty,

        /// Function to sort triangles on closest distance to origin
        pub fn triangleSorter(context: void, t1: *Triangle, t2: *Triangle) bool {
            _ = context;
            return t1.closest_len_sq > t2.closest_len_sq;
        }

        /// Add triangle to the list (push_back)
        pub fn append(self: *TriangleQueue, t: *Triangle) void {
            // Add to base
            self.triangles.append(t);

            // Mark in queue
            t.in_queue = true;

            // Resort heap
            BinaryHeap.binaryHeapPush(*Triangle, self.triangles.slice(), {}, triangleSorter);
        }

        /// Returns true if there are no triangles in the queue (empty)
        pub fn isEmpty(self: *const TriangleQueue) bool {
            return self.triangles.isEmpty();
        }

        /// Peek the next closest triangle without removing it
        pub fn peekClosest(self: *TriangleQueue) *Triangle {
            return self.triangles.front().*;
        }

        /// Get next closest triangle
        pub fn popClosest(self: *TriangleQueue) *Triangle {
            // Move closest to end
            BinaryHeap.binaryHeapPop(*Triangle, self.triangles.slice(), {}, triangleSorter);

            // Remove last triangle
            return self.triangles.pop();
        }
    };

    /// Result of `findFacingTriangle`
    pub const FacingTriangle = struct {
        /// The triangle on which the position is the furthest to the front, null if no triangle faces the position
        triangle: ?*Triangle,
        /// The squared distance of the position to the plane of that triangle (0 if there is none)
        best_dist_sq: f32,
    };

    /// Factory to create new triangles and remove old ones
    factory: TriangleFactory = .{},
    /// List of positions (some of them are part of the hull)
    positions: *const Points,
    /// List of triangles that are part of the hull that still need to be checked (if !removed)
    triangle_queue: TriangleQueue = .{},

    /// Constructor. `positions` must outlive the builder (Jolt keeps a `const Points &`); points can be added to it
    /// while the hull is built.
    pub fn init(positions: *const Points) EPAConvexHullBuilder {
        return .{ .positions = positions };
    }

    /// Initialize the hull with 3 points
    pub fn initialize(self: *EPAConvexHullBuilder, idx1: u32, idx2: u32, idx3: u32) void {
        // Release triangles
        self.factory.clear();

        // Create triangles (back to back)
        const t1 = self.createTriangle(idx1, idx2, idx3).?;
        const t2 = self.createTriangle(idx1, idx3, idx2).?;

        // Link triangles edges
        linkTriangle(t1, 0, t2, 2);
        linkTriangle(t1, 1, t2, 1);
        linkTriangle(t1, 2, t2, 0);

        // Always add both triangles to the priority queue
        self.triangle_queue.append(t1);
        self.triangle_queue.append(t2);
    }

    /// Check if there's another triangle to process from the queue
    pub fn hasNextTriangle(self: *const EPAConvexHullBuilder) bool {
        return !self.triangle_queue.isEmpty();
    }

    /// Access to the next closest triangle to the origin (won't remove it from the queue).
    pub fn peekClosestTriangleInQueue(self: *EPAConvexHullBuilder) *Triangle {
        return self.triangle_queue.peekClosest();
    }

    /// Access to the next closest triangle to the origin and remove it from the queue.
    pub fn popClosestTriangleFromQueue(self: *EPAConvexHullBuilder) *Triangle {
        return self.triangle_queue.popClosest();
    }

    /// Find the triangle on which position is the furthest to the front
    /// Note this function works as long as all points added have been added with addPoint(..., FLT_MAX).
    pub fn findFacingTriangle(self: *EPAConvexHullBuilder, position: Vec3) FacingTriangle {
        var best: ?*Triangle = null;
        var best_dist_sq: f32 = 0.0;

        for (self.triangle_queue.triangles.constSlice()) |t| {
            if (!t.removed) {
                const dot = t.normal.dot(position.sub(t.centroid));
                if (dot > 0.0) {
                    const dist_sq = dot * dot / t.normal.lengthSq();
                    if (dist_sq > best_dist_sq) {
                        best = t;
                        best_dist_sq = dist_sq;
                    }
                }
            }
        }

        return .{ .triangle = best, .best_dist_sq = best_dist_sq };
    }

    /// Add a new point to the convex hull
    pub fn addPoint(self: *EPAConvexHullBuilder, facing_triangle: *Triangle, idx: u32, closest_dist_sq: f32, out_triangles: *NewTriangles) bool {
        // Get position
        const pos = self.positions.get(idx);

        // Find edge of convex hull of triangles that are not facing the new vertex w
        var edges: Edges = .empty;
        if (!self.findEdge(facing_triangle, pos, &edges))
            return false;

        // Create new triangles
        const edge_items = edges.constSlice();
        const num_edges = edge_items.len;
        for (0..num_edges) |i| {
            // Create new triangle
            const nt = self.createTriangle(edge_items[i].start_idx, edge_items[(i + 1) % num_edges].start_idx, idx) orelse return false;
            out_triangles.append(nt);

            // Check if we need to put this triangle in the priority queue
            if ((nt.closest_point_interior and nt.closest_len_sq < closest_dist_sq) or // For the main algorithm
                nt.closest_len_sq < 0.0) // For when the origin is not inside the hull yet
                self.triangle_queue.append(nt);
        }

        // Link edges
        const new_triangles = out_triangles.constSlice();
        for (0..num_edges) |i| {
            linkTriangle(new_triangles[i], 0, edge_items[i].neighbour_triangle.?, edge_items[i].neighbour_edge);
            linkTriangle(new_triangles[i], 1, new_triangles[(i + 1) % num_edges], 2);
        }

        return true;
    }

    /// Free a triangle
    pub fn freeTriangle(self: *EPAConvexHullBuilder, t: *Triangle) void {
        if (Core.enable_asserts) {
            // Make sure that this triangle is not connected
            std.debug.assert(t.removed);
            for (t.edge) |e|
                std.debug.assert(e.neighbour_triangle == null);
        }

        self.factory.freeTriangle(t);
    }

    /// Create a new triangle
    fn createTriangle(self: *EPAConvexHullBuilder, idx1: u32, idx2: u32, idx3: u32) ?*Triangle {
        // Call provider to create triangle
        const t = self.factory.createTriangle(idx1, idx2, idx3, self.positions.constSlice()) orelse return null;

        return t;
    }

    /// Link triangle edge to other triangle edge
    fn linkTriangle(t1: *Triangle, edge1: u32, t2: *Triangle, edge2: u32) void {
        std.debug.assert(edge1 < 3);
        std.debug.assert(edge2 < 3);
        const e1 = &t1.edge[edge1];
        const e2 = &t2.edge[edge2];

        // Check not connected yet
        std.debug.assert(e1.neighbour_triangle == null);
        std.debug.assert(e2.neighbour_triangle == null);

        // Check vertices match
        std.debug.assert(e1.start_idx == t2.getNextEdge(edge2).start_idx);
        std.debug.assert(e2.start_idx == t1.getNextEdge(edge1).start_idx);

        // Link up
        e1.neighbour_triangle = t2;
        e1.neighbour_edge = edge2;
        e2.neighbour_triangle = t1;
        e2.neighbour_edge = edge1;
    }

    /// Unlink this triangle
    fn unlinkTriangle(self: *EPAConvexHullBuilder, t: *Triangle) void {
        // Unlink from neighbours
        for (0..3) |i| {
            const edge = &t.edge[i];
            if (edge.neighbour_triangle) |neighbour_triangle| {
                const neighbour_edge = &neighbour_triangle.edge[edge.neighbour_edge];

                // Validate that neighbour points to us
                std.debug.assert(neighbour_edge.neighbour_triangle == t);
                std.debug.assert(neighbour_edge.neighbour_edge == i);

                // Unlink
                neighbour_edge.neighbour_triangle = null;
                edge.neighbour_triangle = null;
            }
        }

        // If this triangle is not in the priority queue, we can delete it now
        if (!t.in_queue)
            self.freeTriangle(t);
    }

    /// Given one triangle that faces vertex, find the edges of the triangles that are not facing vertex.
    /// Will flag all those triangles for removal.
    fn findEdge(self: *EPAConvexHullBuilder, facing_triangle: *Triangle, vertex: Vec3, out_edges: *Edges) bool {
        // Assert that we were given an empty array
        std.debug.assert(out_edges.isEmpty());

        // Should start with a facing triangle
        std.debug.assert(facing_triangle.isFacing(vertex));

        // Flag as removed
        facing_triangle.removed = true;

        // Instead of recursing, we build our own stack with the information we need
        const StackEntry = struct {
            triangle: *Triangle,
            edge: u32,
            iter: i32,
        };
        var stack: [max_edge_length]StackEntry = undefined;
        var cur_stack_pos: u32 = 0;

        // Start with the triangle / edge provided
        stack[0].triangle = facing_triangle;
        stack[0].edge = 0;
        stack[0].iter = -1; // Start with edge 0 (is incremented below before use)

        // Next index that we expect to find, if we don't then there are 'islands' (Jolt uses -1 for none)
        var next_expected_start_idx: ?u32 = null;

        while (true) {
            const cur_entry = &stack[cur_stack_pos];

            // Next iteration
            cur_entry.iter += 1;
            if (cur_entry.iter >= 3) {
                // This triangle needs to be removed, unlink it now
                self.unlinkTriangle(cur_entry.triangle);

                // Pop from stack
                if (cur_stack_pos == 0)
                    break;
                cur_stack_pos -= 1;
            } else {
                // Visit neighbour
                const e = &cur_entry.triangle.edge[(cur_entry.edge + @as(u32, @intCast(cur_entry.iter))) % 3];
                if (e.neighbour_triangle) |n| {
                    if (!n.removed) {
                        // Check if vertex is on the front side of this triangle
                        if (n.isFacing(vertex)) {
                            // Vertex on front, this triangle needs to be removed
                            n.removed = true;

                            // Add element to the stack of elements to visit
                            cur_stack_pos += 1;
                            std.debug.assert(cur_stack_pos < max_edge_length);
                            const new_entry = &stack[cur_stack_pos];
                            new_entry.triangle = n;
                            new_entry.edge = e.neighbour_edge;
                            new_entry.iter = 0; // Is incremented before use, we don't need to test this edge again since we came from it
                        } else {
                            // Detect if edge doesn't connect to previous edge, if this happens we have found and 'island' which means
                            // the newly added point is so close to the triangles of the hull that we classified some (nearly) coplanar
                            // triangles as before and some behind the point. At this point we just abort adding the point because
                            // we've reached numerical precision.
                            // Note that we do not need to test if the first and last edge connect, since when there are islands
                            // there should be at least 2 disconnects.
                            if (next_expected_start_idx) |expected| {
                                if (e.start_idx != expected)
                                    return false;
                            }

                            // Next expected index is the start index of our neighbour's edge
                            next_expected_start_idx = n.edge[e.neighbour_edge].start_idx;

                            // Vertex behind, keep edge
                            out_edges.append(e.*);
                        }
                    }
                }
            }
        }

        // Assert that we have a fully connected loop
        // (only evaluated when asserts are enabled, Jolt continues in release builds if the loop is not closed)
        if (Core.enable_asserts)
            std.debug.assert(out_edges.isEmpty() or out_edges.get(0).start_idx == next_expected_start_idx.?);

        // When we start with two triangles facing away from each other and adding a point that is on the plane,
        // sometimes we consider the point in front of both causing both triangles to be removed resulting in an empty edge list.
        // In this case we fail to add the point which will result in no collision reported (the shapes are contacting in 1 point so there's 0 penetration)
        return out_edges.len >= 3;
    }
};
