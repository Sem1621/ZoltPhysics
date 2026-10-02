//! Port of: Jolt/Geometry/ConvexHullBuilder.h, Jolt/Geometry/ConvexHullBuilder.cpp
//! Status: complete
//! Not ported: JPH_CONVEX_BUILDER_DEBUG (DrawState, DrawWireFace, DrawEdge, cDrawScale, mIteration, mOffset, mDelta), JPH_CONVEX_BUILDER_DUMP_SHAPE (DumpShape)
//!
//! Jolt's constructor + `Initialize` become `init(allocator, positions)` + `initialize(max_vertices, tolerance)`, the
//! destructor is `deinit()`. The allocator is used for the faces and edges (`new` / `delete` in Jolt), the conflict
//! lists and all temporary arrays. Allocation failures (which abort in Jolt) are returned as `error.OutOfMemory`
//! without leaking memory; after such an error the hull is incomplete and the builder can only be re-initialized or
//! deinitialized.
//!
//! The out parameters of Jolt's functions are returned as structs: `initialize` returns
//! `InitializeResult{ .result, .error_message }` (Jolt's return value and `outError`), `getCenterOfMassAndVolume`
//! returns `CenterOfMassAndVolume{ .center_of_mass, .volume }` and `determineMaxError` returns
//! `MaxError{ .face_with_max_error, .max_error, .max_error_position_idx, .coplanar_distance }`.
//! `getNumVerticesUsed` allocates (an UnorderedSet like Jolt) and can therefore fail.

const std = @import("std");
const Core = @import("../Core/Core.zig");
const UnorderedSet = @import("../Core/UnorderedSet.zig").UnorderedSet;
const math = @import("../Math/Math.zig");
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;
const ClosestPoint = @import("ClosestPoint.zig");
const ConvexHullBuilder2D = @import("ConvexHullBuilder2D.zig").ConvexHullBuilder2D;

const assert = std.debug.assert;
const log = std.log.scoped(.zolt);

/// A convex hull builder that tries to create hulls as accurately as possible. Used for offline processing.
/// NonCopyable in Jolt: the builder owns its faces and edges, don't copy it.
pub const ConvexHullBuilder = struct {
    /// Class that holds the information of an edge
    /// NonCopyable in Jolt: edges are linked through pointers, don't copy them.
    pub const Edge = struct {
        /// Face that this edge belongs to
        face: *Face,
        /// Next edge of this face
        next_edge: ?*Edge = null,
        /// Edge that this edge is connected to
        neighbour_edge: ?*Edge = null,
        /// Vertex index in positions that indicates the start vertex of this edge
        start_idx: i32,

        /// Constructor
        pub fn init(face: *Face, start_idx: i32) Edge {
            return .{ .face = face, .start_idx = start_idx };
        }

        /// Get the previous edge
        pub fn getPreviousEdge(self: *Edge) *Edge {
            var prev_edge = self;
            while (prev_edge.next_edge.? != self)
                prev_edge = prev_edge.next_edge.?;
            return prev_edge;
        }
    };

    pub const ConflictList = std.ArrayList(i32);

    /// Class that holds the information of one face
    /// NonCopyable in Jolt: the face owns its edges, don't copy it.
    pub const Face = struct {
        /// Normal of this face, length is 2 times area of face
        normal: Vec3 = undefined,
        /// Center of the face
        centroid: Vec3 = undefined,
        /// Positions associated with this edge (that are closest to this edge). The last position in the list is the point that is furthest away from the face.
        conflict_list: ConflictList = .empty,
        /// First edge of this face
        first_edge: ?*Edge = null,
        /// Squared distance of furthest point from the conflict list to the face
        furthest_point_distance_sq: f32 = 0.0,
        /// Flag that indicates that face has been removed (face will be freed later)
        removed: bool = false,

        /// Destructor: frees the edges and the conflict list (`allocator` is the one that created them)
        pub fn deinit(self: *Face, allocator: std.mem.Allocator) void {
            // Free all edges
            var e = self.first_edge;
            if (e != null) {
                while (true) {
                    const next = e.?.next_edge;
                    allocator.destroy(e.?);
                    e = next;
                    if (e == self.first_edge) break;
                }
            }

            self.conflict_list.deinit(allocator);
        }

        /// Initialize a face with three indices. The edges are allocated with `allocator`.
        pub fn initialize(self: *Face, allocator: std.mem.Allocator, idx0: i32, idx1: i32, idx2: i32, positions: []const Vec3) error{OutOfMemory}!void {
            assert(self.first_edge == null);
            assert(idx0 != idx1 and idx0 != idx2 and idx1 != idx2);

            // Create 3 edges
            const e0 = try createEdge(allocator, self, idx0);
            errdefer allocator.destroy(e0);
            const e1 = try createEdge(allocator, self, idx1);
            errdefer allocator.destroy(e1);
            const e2 = try createEdge(allocator, self, idx2);

            // Link edges
            e0.next_edge = e1;
            e1.next_edge = e2;
            e2.next_edge = e0;
            self.first_edge = e0;

            self.calculateNormalAndCentroid(positions);
        }

        /// Calculates the centroid and normal for this face
        pub fn calculateNormalAndCentroid(self: *Face, positions: []const Vec3) void {
            // Get point that we use to construct a triangle fan
            var e = self.first_edge.?;
            const y0 = positions[@intCast(e.start_idx)];

            // Get the 2nd point
            e = e.next_edge.?;
            var y1 = positions[@intCast(e.start_idx)];

            // Start accumulating the centroid
            self.centroid = y0.add(y1);
            var n: i32 = 2;

            // Start accumulating the normal
            self.normal = Vec3.zero();

            // Loop over remaining edges accumulating normals in a triangle fan fashion
            e = e.next_edge.?;
            while (e != self.first_edge.?) : (e = e.next_edge.?) {
                // Get the 3rd point
                const y2 = positions[@intCast(e.start_idx)];

                // Calculate edges (counter clockwise)
                const e0 = y1.sub(y0);
                const e1 = y2.sub(y1);
                const e2 = y0.sub(y2);

                // The best normal is calculated by using the two shortest edges
                // See: https://box2d.org/posts/2014/01/troublesome-triangle/
                // The difference in normals is most pronounced when one edge is much smaller than the others (in which case the others must have roughly the same length).
                // Therefore we can suffice by just picking the shortest from 2 edges and use that with the 3rd edge to calculate the normal.
                // We first check which of the edges is shorter: e1 or e2
                const e1_shorter_than_e2 = Vec4.less(e1.dotV4(e1), e2.dotV4(e2));

                // We calculate both normals and then select the one that had the shortest edge for our normal (this avoids branching)
                const normal_e01 = e0.cross(e1);
                const normal_e02 = e2.cross(e0);
                self.normal = self.normal.add(Vec3.select(normal_e02, normal_e01, e1_shorter_than_e2));

                // Accumulate centroid
                self.centroid = self.centroid.add(y2);
                n += 1;

                // Update y1 for next triangle
                y1 = y2;
            }

            // Finalize centroid
            self.centroid = self.centroid.divScalar(@floatFromInt(n));
        }

        /// Check if face is facing position
        pub fn isFacing(self: *const Face, position: Vec3) bool {
            assert(!self.removed);
            return self.normal.dot(position.sub(self.centroid)) > 0.0;
        }
    };

    // Typedefs
    pub const Positions = std.ArrayList(Vec3);
    pub const Faces = std.ArrayList(*Face);

    /// Result enum that indicates how the hull got created
    pub const Result = enum {
        /// Hull building finished successfully
        success,
        /// Hull building finished successfully, but the desired accuracy was not reached because the max vertices limit was reached
        max_vertices_reached,
        /// Too few points to create a hull
        too_few_points,
        /// Too few faces in the created hull (signifies precision errors during building)
        too_few_faces,
        /// Degenerate hull detected
        degenerate,
    };

    /// Return value of `initialize` (Jolt's return value and the outError out parameter)
    pub const InitializeResult = struct {
        /// Status code that reports if the hull was created or not
        result: Result,
        /// Error message when building fails (Jolt leaves outError untouched when there is no error, this is null then)
        error_message: ?[]const u8 = null,
    };

    /// Return value of `getCenterOfMassAndVolume` (the outCenterOfMass and outVolume out parameters)
    pub const CenterOfMassAndVolume = struct {
        center_of_mass: Vec3,
        volume: f32,
    };

    /// Return value of `determineMaxError` (the out parameters of Jolt's DetermineMaxError)
    pub const MaxError = struct {
        /// The face that caused the error
        face_with_max_error: ?*Face,
        /// The maximum distance of a point to the hull
        max_error: f32,
        /// The index of the point that had this distance
        max_error_position_idx: i32,
        /// Points that are less than this distance from the hull are considered on the hull. This should be used as a lowerbound for the allowed error.
        coplanar_distance: f32,
    };

    /// Minimal square area of a triangle (used for merging and checking if a triangle is degenerate)
    const min_triangle_area_sq: f32 = 1.0e-12;

    /// Extra slop used to determine if the hull is coplanar / a point is on a face.
    const coplanar_slop_factor: f32 = 6.0;

    /// Class that holds an edge including start and end index
    const FullEdge = struct {
        /// Edge that this edge is connected to
        neighbour_edge: *Edge,
        /// Vertex index in positions that indicates the start vertex of this edge
        start_idx: i32,
        /// Vertex index in positions that indicates the end vertex of this edge
        end_idx: i32,
    };

    // Private typedefs
    const FullEdges = std.ArrayList(FullEdge);

    const Coplanar = struct {
        /// Index in positions
        position_idx: i32,
        /// Distance to the edge of closest face (should be > 0)
        distance_sq: f32,
    };
    const CoplanarList = std.ArrayList(Coplanar);

    /// The outFace and outDistSq out parameters of GetFaceForPoint
    const FaceForPoint = struct {
        /// The best face
        face: ?*Face,
        /// The squared distance how much the point is in front of the plane of the face
        dist_sq: f32,
    };

    allocator: std.mem.Allocator,
    /// List of positions (some of them are part of the hull)
    positions: []const Vec3,
    /// List of faces that are part of the hull (if !removed)
    faces: Faces = .empty,
    /// List of positions that are coplanar to a face but outside of the face, these are added to the hull at the end
    coplanar_list: CoplanarList = .empty,

    /// Constructor
    /// `positions` must outlive the builder (Jolt keeps a `const Positions &`, pass `list.items` for a `Positions`
    /// list and assign `builder.positions` again when the list changed).
    pub fn init(allocator: std.mem.Allocator, positions: []const Vec3) ConvexHullBuilder {
        return .{ .allocator = allocator, .positions = positions };
    }

    /// Destructor
    pub fn deinit(self: *ConvexHullBuilder) void {
        self.freeFaces();
        self.faces.deinit(self.allocator);
        self.coplanar_list.deinit(self.allocator);
    }

    /// Takes all positions as provided by the constructor and use them to build a hull
    /// Any points that are closer to the hull than tolerance will be discarded
    /// @param max_vertices Max vertices to allow in the hull. Specify std.math.maxInt(i32) (INT_MAX) if there is no limit.
    /// @param tolerance Max distance that a point is allowed to be outside of the hull
    /// @return Status code that reports if the hull was created or not and the error message when building fails
    pub fn initialize(self: *ConvexHullBuilder, max_vertices: i32, tolerance: f32) error{OutOfMemory}!InitializeResult {
        // Free the faces possibly left over from an earlier hull
        self.freeFaces();

        // Test that we have at least 3 points
        if (self.positions.len < 3)
            return .{ .result = .too_few_points, .error_message = "Need at least 3 points to make a hull" };

        const num_positions: i32 = @intCast(self.positions.len);

        // Determine a suitable tolerance for detecting that points are coplanar
        const coplanar_tolerance_sq = math.square(self.determineCoplanarDistance());

        // Increase desired tolerance if accuracy doesn't allow it
        const tolerance_sq = math.max(coplanar_tolerance_sq, math.square(tolerance));

        // Find point furthest from the origin
        var idx1: i32 = -1;
        var max_dist_sq: f32 = -1.0;
        var i: i32 = 0;
        while (i < num_positions) : (i += 1) {
            const dist_sq = self.positionAt(i).lengthSq();
            if (dist_sq > max_dist_sq) {
                max_dist_sq = dist_sq;
                idx1 = i;
            }
        }
        assert(idx1 >= 0);

        // Find point that is furthest away from this point
        var idx2: i32 = -1;
        max_dist_sq = -1.0;
        i = 0;
        while (i < num_positions) : (i += 1) {
            if (i != idx1) {
                const dist_sq = self.positionAt(i).sub(self.positionAt(idx1)).lengthSq();
                if (dist_sq > max_dist_sq) {
                    max_dist_sq = dist_sq;
                    idx2 = i;
                }
            }
        }
        assert(idx2 >= 0);

        // Find point that forms the biggest triangle
        var idx3: i32 = -1;
        var best_triangle_area_sq: f32 = -1.0;
        i = 0;
        while (i < num_positions) : (i += 1) {
            if (i != idx1 and i != idx2) {
                const triangle_area_sq = self.positionAt(idx1).sub(self.positionAt(i)).cross(self.positionAt(idx2).sub(self.positionAt(i))).lengthSq();
                if (triangle_area_sq > best_triangle_area_sq) {
                    best_triangle_area_sq = triangle_area_sq;
                    idx3 = i;
                }
            }
        }
        assert(idx3 >= 0);
        if (best_triangle_area_sq < min_triangle_area_sq)
            return .{ .result = .degenerate, .error_message = "Could not find a suitable initial triangle because its area was too small" };

        // Check if we have only 3 vertices
        if (self.positions.len == 3) {
            // Create two triangles (back to back)
            const t1 = try self.createTriangle(idx1, idx2, idx3);
            const t2 = try self.createTriangle(idx1, idx3, idx2);

            // Link faces edges
            linkFace(t1.first_edge.?, t2.first_edge.?.next_edge.?.next_edge.?);
            linkFace(t1.first_edge.?.next_edge.?, t2.first_edge.?.next_edge.?);
            linkFace(t1.first_edge.?.next_edge.?.next_edge.?, t2.first_edge.?);

            return .{ .result = .success };
        }

        // Find point that forms the biggest tetrahedron
        const initial_plane_normal = self.positionAt(idx2).sub(self.positionAt(idx1)).cross(self.positionAt(idx3).sub(self.positionAt(idx1))).normalized();
        const initial_plane_centroid = self.positionAt(idx1).add(self.positionAt(idx2)).add(self.positionAt(idx3)).divScalar(3.0);
        var idx4: i32 = -1;
        var max_dist: f32 = 0.0;
        i = 0;
        while (i < num_positions) : (i += 1) {
            if (i != idx1 and i != idx2 and i != idx3) {
                const dist = self.positionAt(i).sub(initial_plane_centroid).dot(initial_plane_normal);
                if (@abs(dist) > @abs(max_dist)) {
                    max_dist = dist;
                    idx4 = i;
                }
            }
        }

        // Check if the hull is coplanar
        if (math.square(max_dist) <= math.square(coplanar_slop_factor) * coplanar_tolerance_sq)
            return self.initialize2D(idx1, idx2, idx3, initial_plane_normal, max_vertices, tolerance);

        // Ensure the planes are facing outwards
        if (max_dist < 0.0)
            std.mem.swap(i32, &idx2, &idx3);

        // Create tetrahedron
        const t1 = try self.createTriangle(idx1, idx2, idx4);
        const t2 = try self.createTriangle(idx2, idx3, idx4);
        const t3 = try self.createTriangle(idx3, idx1, idx4);
        const t4 = try self.createTriangle(idx1, idx3, idx2);

        // Link face edges
        linkFace(t1.first_edge.?, t4.first_edge.?.next_edge.?.next_edge.?);
        linkFace(t1.first_edge.?.next_edge.?, t2.first_edge.?.next_edge.?.next_edge.?);
        linkFace(t1.first_edge.?.next_edge.?.next_edge.?, t3.first_edge.?.next_edge.?);
        linkFace(t2.first_edge.?, t4.first_edge.?.next_edge.?);
        linkFace(t2.first_edge.?.next_edge.?, t3.first_edge.?.next_edge.?.next_edge.?);
        linkFace(t3.first_edge.?, t4.first_edge.?);

        // Build the initial conflict lists
        const faces = [_]*Face{ t1, t2, t3, t4 };
        var idx: i32 = 0;
        while (idx < num_positions) : (idx += 1) {
            if (idx != idx1 and idx != idx2 and idx != idx3 and idx != idx4)
                _ = try self.assignPointToFace(idx, &faces, tolerance_sq);
        }

        // Overestimate of the actual amount of vertices we use, for limiting the amount of vertices in the hull
        var num_vertices_used: i32 = 4;

        // Loop through the remainder of the points and add them
        main_loop: while (true) {
            // Find the face with the furthest point on it
            var face_with_furthest_point: ?*Face = null;
            var furthest_dist_sq: f32 = 0.0;
            for (self.faces.items) |f| {
                if (f.furthest_point_distance_sq > furthest_dist_sq) {
                    furthest_dist_sq = f.furthest_point_distance_sq;
                    face_with_furthest_point = f;
                }
            }

            var furthest_point_idx: i32 = undefined;
            if (face_with_furthest_point) |f| {
                // Take the furthest point
                furthest_point_idx = f.conflict_list.pop().?;
            } else if (self.coplanar_list.items.len != 0) {
                // Try to assign points to faces (this also recalculates the distance to the hull for the coplanar vertices)
                var coplanar = self.coplanar_list;
                self.coplanar_list = .empty;
                defer coplanar.deinit(self.allocator);
                var added = false;
                for (coplanar.items) |c| {
                    // Note: Jolt's `added |= ...` always calls AssignPointToFace
                    if (try self.assignPointToFace(c.position_idx, self.faces.items, tolerance_sq))
                        added = true;
                }

                // If we were able to assign a point, loop again to pick it up
                if (added)
                    continue :main_loop;

                // If the coplanar list is empty, there are no points left and we're done
                if (self.coplanar_list.items.len == 0)
                    break :main_loop;

                while (true) {
                    // Find the vertex that is furthest from the hull
                    var best_idx: usize = 0;
                    var best_dist_sq = self.coplanar_list.items[0].distance_sq;
                    for (1..self.coplanar_list.items.len) |c_idx| {
                        const c = &self.coplanar_list.items[c_idx];
                        if (c.distance_sq > best_dist_sq) {
                            best_idx = c_idx;
                            best_dist_sq = c.distance_sq;
                        }
                    }

                    // Swap it to the end
                    std.mem.swap(Coplanar, &self.coplanar_list.items[best_idx], &self.coplanar_list.items[self.coplanar_list.items.len - 1]);

                    // Remove it
                    furthest_point_idx = self.coplanar_list.pop().?.position_idx;

                    // Find the face for which the point is furthest away
                    face_with_furthest_point = self.getFaceForPoint(self.positionAt(furthest_point_idx), self.faces.items).face;

                    if (!(self.coplanar_list.items.len != 0 and face_with_furthest_point == null))
                        break;
                }

                if (face_with_furthest_point == null)
                    break :main_loop;
            } else {
                // If there are no more vertices, we're done
                break :main_loop;
            }

            // Check if we have a limit on the max vertices that we should produce
            if (num_vertices_used >= max_vertices) {
                // Count the actual amount of used vertices (we did not take the removal of any vertices into account)
                num_vertices_used = try self.getNumVerticesUsed();

                // Check if there are too many
                if (num_vertices_used >= max_vertices)
                    return .{ .result = .max_vertices_reached };
            }

            // We're about to add another vertex
            num_vertices_used += 1;

            // Add the point to the hull
            var new_faces: Faces = .empty;
            defer new_faces.deinit(self.allocator);
            try self.addPoint(face_with_furthest_point.?, furthest_point_idx, coplanar_tolerance_sq, &new_faces);

            // Redistribute points on conflict lists belonging to removed faces
            for (self.faces.items) |face| {
                if (face.removed) {
                    for (face.conflict_list.items) |point_idx|
                        _ = try self.assignPointToFace(point_idx, new_faces.items, tolerance_sq);
                }
            }

            // Permanently delete faces that we removed in addPoint()
            self.garbageCollectFaces();
        }

        // Check if we are left with a hull. It is possible that hull building fails if the points are nearly coplanar.
        if (self.faces.items.len < 2)
            return .{ .result = .too_few_faces, .error_message = "Too few faces in hull" };

        return .{ .result = .success };
    }

    /// The coplanar case of `initialize` (inline in Jolt): build a 2D hull and create two back to back faces from it
    fn initialize2D(self: *ConvexHullBuilder, idx1: i32, idx2: i32, idx3: i32, initial_plane_normal: Vec3, max_vertices: i32, tolerance: f32) error{OutOfMemory}!InitializeResult {
        const allocator = self.allocator;

        // First project all points in 2D space
        const base1 = initial_plane_normal.getNormalizedPerpendicular();
        const base2 = initial_plane_normal.cross(base1);
        var positions_2d: std.ArrayList(Vec3) = .empty;
        defer positions_2d.deinit(allocator);
        try positions_2d.ensureTotalCapacity(allocator, self.positions.len);
        for (self.positions) |v|
            positions_2d.appendAssumeCapacity(Vec3.init(base1.dot(v), base2.dot(v), 0.0));

        // Build hull
        var edges_2d: ConvexHullBuilder2D.Edges = .empty;
        defer edges_2d.deinit(allocator);
        var builder_2d = ConvexHullBuilder2D.init(allocator, positions_2d.items);
        defer builder_2d.deinit();
        const result = try builder_2d.initialize(idx1, idx2, idx3, max_vertices, tolerance, &edges_2d);

        // Create faces (back to back)
        const f1 = try self.createFace();
        const f2 = try self.createFace();

        // Create edges for face 1
        var edges_f1: std.ArrayList(*Edge) = .empty;
        defer edges_f1.deinit(allocator);
        try edges_f1.ensureTotalCapacity(allocator, edges_2d.items.len);
        for (edges_2d.items) |start_idx| {
            const edge = createEdge(allocator, f1, start_idx) catch |err| {
                // The edge loop is not closed yet, free the edges that were created (Zolt only, Jolt aborts)
                destroyOpenEdgeLoop(allocator, f1, edges_f1.items);
                return err;
            };
            if (edges_f1.items.len == 0)
                f1.first_edge = edge
            else
                edges_f1.items[edges_f1.items.len - 1].next_edge = edge;
            edges_f1.appendAssumeCapacity(edge);
        }
        edges_f1.items[edges_f1.items.len - 1].next_edge = f1.first_edge;

        // Create edges for face 2
        var edges_f2: std.ArrayList(*Edge) = .empty;
        defer edges_f2.deinit(allocator);
        try edges_f2.ensureTotalCapacity(allocator, edges_2d.items.len);
        var i: i32 = @as(i32, @intCast(edges_2d.items.len)) - 1;
        while (i >= 0) : (i -= 1) {
            const edge = createEdge(allocator, f2, edges_2d.items[@intCast(i)]) catch |err| {
                // The edge loop is not closed yet, free the edges that were created (Zolt only, Jolt aborts)
                destroyOpenEdgeLoop(allocator, f2, edges_f2.items);
                return err;
            };
            if (edges_f2.items.len == 0)
                f2.first_edge = edge
            else
                edges_f2.items[edges_f2.items.len - 1].next_edge = edge;
            edges_f2.appendAssumeCapacity(edge);
        }
        edges_f2.items[edges_f2.items.len - 1].next_edge = f2.first_edge;

        // Link edges
        const num_edges = edges_2d.items.len;
        for (0..num_edges) |e|
            linkFace(edges_f1.items[e], edges_f2.items[(2 * num_edges - 2 - e) % num_edges]);

        // Calculate the plane for both faces
        f1.calculateNormalAndCentroid(self.positions);
        f2.normal = f1.normal.negate();
        f2.centroid = f1.centroid;

        return .{ .result = if (result == .max_vertices_reached) .max_vertices_reached else .success };
    }

    /// Returns the amount of vertices that are currently used by the hull
    pub fn getNumVerticesUsed(self: *const ConvexHullBuilder) error{OutOfMemory}!i32 {
        var used_verts: UnorderedSet(i32, .{}) = .empty;
        defer used_verts.deinit(self.allocator);
        try used_verts.ensureTotalCapacity(self.allocator, @intCast(self.positions.len));
        for (self.faces.items) |f| {
            var e = f.first_edge.?;
            while (true) {
                _ = try used_verts.insert(self.allocator, e.start_idx);
                e = e.next_edge.?;
                if (e == f.first_edge.?) break;
            }
        }
        return @intCast(used_verts.count());
    }

    /// Returns true if the hull contains a polygon with indices (counter clockwise indices in positions)
    pub fn containsFace(self: *const ConvexHullBuilder, indices: []const i32) bool {
        for (self.faces.items) |f| {
            var e = f.first_edge.?;
            if (std.mem.indexOfScalar(i32, indices, e.start_idx)) |found| {
                var index = found;
                var matches: usize = 0;

                while (true) {
                    // Check if index matches
                    if (indices[index] != e.start_idx)
                        break;

                    // Increment number of matches
                    matches += 1;

                    // Next index in list of indices
                    index += 1;
                    if (index == indices.len)
                        index = 0;

                    // Next edge
                    e = e.next_edge.?;
                    if (e == f.first_edge.?) break;
                }

                if (matches == indices.len)
                    return true;
            }
        }

        return false;
    }

    /// Calculate the center of mass and the volume of the current convex hull
    pub fn getCenterOfMassAndVolume(self: *const ConvexHullBuilder) CenterOfMassAndVolume {
        // Fourth point is the average of all face centroids
        var v4 = Vec3.zero();
        for (self.faces.items) |f|
            v4 = v4.add(f.centroid);
        v4 = v4.divScalar(@floatFromInt(self.faces.items.len));

        // Calculate mass and center of mass of this convex hull by summing all tetrahedrons
        var volume: f32 = 0.0;
        var center_of_mass = Vec3.zero();
        for (self.faces.items) |f| {
            // Get the first vertex that we'll use to create a triangle fan
            var e = f.first_edge.?;
            const v1 = self.positionAt(e.start_idx);

            // Get the second vertex
            e = e.next_edge.?;
            var v2 = self.positionAt(e.start_idx);

            e = e.next_edge.?;
            while (e != f.first_edge.?) : (e = e.next_edge.?) {
                // Fetch the last point of the triangle
                const v3 = self.positionAt(e.start_idx);

                // Calculate center of mass and mass of this tetrahedron,
                // see: https://en.wikipedia.org/wiki/Tetrahedron#Volume
                const volume_tetrahedron = v1.sub(v4).dot(v2.sub(v4).cross(v3.sub(v4))); // Needs to be divided by 6, postpone this until the end of the loop
                const center_of_mass_tetrahedron = v1.add(v2).add(v3).add(v4); // Needs to be divided by 4, postpone this until the end of the loop

                // Accumulate results
                volume += volume_tetrahedron;
                center_of_mass = center_of_mass.add(center_of_mass_tetrahedron.mulScalar(volume_tetrahedron));

                // Update v2 for next triangle
                v2 = v3;
            }
        }

        // Calculate center of mass, fall back to average point in case there is no volume (everything is on a plane in this case)
        if (volume > math.flt_epsilon)
            center_of_mass = center_of_mass.divScalar(4.0 * volume)
        else
            center_of_mass = v4;

        volume /= 6.0;

        return .{ .center_of_mass = center_of_mass, .volume = volume };
    }

    /// Determines the point that is furthest outside of the hull and reports how far it is outside of the hull (which indicates a failure during hull building)
    /// Returns the face that caused the error, the maximum distance of a point to the hull, the index of the point that had
    /// this distance and the coplanar distance (points that are less than this distance from the hull are considered on the hull,
    /// this should be used as a lowerbound for the allowed error).
    pub fn determineMaxError(self: *const ConvexHullBuilder) MaxError {
        const coplanar_distance = self.determineCoplanarDistance();

        // This measures the distance from a polygon to the furthest point outside of the hull
        var max_error: f32 = 0.0;
        var max_error_face: ?*Face = null;
        var max_error_point: i32 = -1;

        for (self.positions, 0..) |v, i| {
            // This measures the closest edge from all faces to point v
            // Note that we take the min of all faces since there may be multiple near coplanar faces so if we were to test this per face
            // we may find that a point is outside of a polygon and mark it as an error, while it is actually inside a nearly coplanar
            // polygon.
            var min_edge_dist_sq: f32 = math.flt_max;
            var min_edge_dist_face: ?*Face = null;

            for (self.faces.items) |f| {
                // Check if point is on or in front of plane
                const normal_len = f.normal.length();
                assert(normal_len > 0.0);
                const plane_dist = f.normal.dot(v.sub(f.centroid)) / normal_len;
                if (plane_dist > -coplanar_slop_factor * coplanar_distance) {
                    // Check distance to the edges of this face
                    const edge_dist_sq = self.getDistanceToEdgeSq(v, f);
                    if (edge_dist_sq < min_edge_dist_sq) {
                        min_edge_dist_sq = edge_dist_sq;
                        min_edge_dist_face = f;
                    }

                    // If the point is inside the polygon and the point is in front of the plane, measure the distance
                    if (edge_dist_sq == 0.0 and plane_dist > max_error) {
                        max_error = plane_dist;
                        max_error_face = f;
                        max_error_point = @intCast(i);
                    }
                }
            }

            // If the minimum distance to an edge is further than our current max error, we use that as max error
            const min_edge_dist = math.sqrt(min_edge_dist_sq);
            if (min_edge_dist_face != null and min_edge_dist > max_error) {
                max_error = min_edge_dist;
                max_error_face = min_edge_dist_face;
                max_error_point = @intCast(i);
            }
        }

        return .{
            .face_with_max_error = max_error_face,
            .max_error = max_error,
            .max_error_position_idx = max_error_point,
            .coplanar_distance = coplanar_distance,
        };
    }

    /// Access to the created faces. Memory is owned by the convex hull builder.
    pub fn getFaces(self: *const ConvexHullBuilder) []const *Face {
        return self.faces.items;
    }

    /// Get the position with index idx
    fn positionAt(self: *const ConvexHullBuilder, idx: i32) Vec3 {
        return self.positions[@intCast(idx)];
    }

    /// Determine a suitable tolerance for detecting that points are coplanar
    fn determineCoplanarDistance(self: *const ConvexHullBuilder) f32 {
        // Formula as per: Implementing Quickhull - Dirk Gregorius.
        var vmax = Vec3.zero();
        for (self.positions) |v|
            vmax = Vec3.max(vmax, v.abs());
        return 3.0 * math.flt_epsilon * (vmax.getX() + vmax.getY() + vmax.getZ());
    }

    /// Find the face for which point is furthest to the front
    /// @param point Point to test
    /// @param faces List of faces to test
    /// @return The best face and the squared distance how much point is in front of the plane of the face
    fn getFaceForPoint(self: *const ConvexHullBuilder, point: Vec3, faces: []const *Face) FaceForPoint {
        _ = self;
        var out_face: ?*Face = null;
        var out_dist_sq: f32 = 0.0;

        for (faces) |f| {
            if (!f.removed) {
                // Determine distance to face
                const dot = f.normal.dot(point.sub(f.centroid));
                if (dot > 0.0) {
                    const dist_sq = dot * dot / f.normal.lengthSq();
                    if (dist_sq > out_dist_sq) {
                        out_face = f;
                        out_dist_sq = dist_sq;
                    }
                }
            }
        }

        return .{ .face = out_face, .dist_sq = out_dist_sq };
    }

    /// Calculates the distance between point and face
    /// @param point Point to test
    /// @param face Face to test
    /// @return If the projection of the point on the plane is interior to the face 0, otherwise the squared distance to the closest edge
    fn getDistanceToEdgeSq(self: *const ConvexHullBuilder, point: Vec3, face: *const Face) f32 {
        var all_inside = true;
        var edge_dist_sq: f32 = math.flt_max;

        // Test if it is inside the edges of the polygon
        var edge = face.first_edge.?;
        var p1 = self.positionAt(edge.getPreviousEdge().start_idx);
        while (true) {
            const p2 = self.positionAt(edge.start_idx);
            if (p2.sub(p1).cross(point.sub(p1)).dot(face.normal) < 0.0) {
                // It is outside
                all_inside = false;

                // Measure distance to this edge
                edge_dist_sq = math.min(edge_dist_sq, ClosestPoint.getClosestPointOnLine(p1.sub(point), p2.sub(point)).point.lengthSq());
            }
            p1 = p2;
            edge = edge.next_edge.?;
            if (edge == face.first_edge.?) break;
        }

        return if (all_inside) 0.0 else edge_dist_sq;
    }

    /// Assigns a position to one of the supplied faces based on which face is closest.
    /// @param position_idx Index of the position to add
    /// @param faces List of faces to consider
    /// @param tolerance_sq Tolerance of the hull, if the point is closer to the face than this, we ignore it
    /// @return True if point was assigned, false if it was discarded or added to the coplanar list
    fn assignPointToFace(self: *ConvexHullBuilder, position_idx: i32, faces: []const *Face, tolerance_sq: f32) error{OutOfMemory}!bool {
        const point = self.positionAt(position_idx);

        // Find the face for which the point is furthest away
        const best = self.getFaceForPoint(point, faces);
        const best_dist_sq = best.dist_sq;

        if (best.face) |best_face| {
            // Check if this point is within the tolerance margin to the plane
            if (best_dist_sq <= tolerance_sq) {
                // Check distance to edges
                const dist_to_edge_sq = self.getDistanceToEdgeSq(point, best_face);
                if (dist_to_edge_sq > tolerance_sq) {
                    // Point is outside of the face and too far away to discard
                    try self.coplanar_list.append(self.allocator, .{ .position_idx = position_idx, .distance_sq = dist_to_edge_sq });
                }
            } else {
                // This point is in front of the face, add it to the conflict list
                if (best_dist_sq > best_face.furthest_point_distance_sq) {
                    // This point is further away than any others, update the distance and add point as last point
                    best_face.furthest_point_distance_sq = best_dist_sq;
                    try best_face.conflict_list.append(self.allocator, position_idx);
                } else {
                    // Not the furthest point, add it as the before last point
                    try best_face.conflict_list.insert(self.allocator, best_face.conflict_list.items.len - 1, position_idx);
                }

                return true;
            }
        }

        return false;
    }

    /// Add a new point to the convex hull
    fn addPoint(self: *ConvexHullBuilder, facing_face: *Face, idx: i32, coplanar_tolerance_sq: f32, out_new_faces: *Faces) error{OutOfMemory}!void {
        // Get position
        const pos = self.positionAt(idx);

        // Check if structure is intact
        if (Core.enable_asserts) self.validateFaces();

        // Find edge of convex hull of faces that are not facing the new vertex
        var edges: FullEdges = .empty;
        defer edges.deinit(self.allocator);
        try self.findEdge(facing_face, pos, &edges);
        assert(edges.items.len >= 3);

        // Create new faces
        try out_new_faces.ensureTotalCapacity(self.allocator, edges.items.len);
        for (edges.items) |e| {
            assert(e.start_idx != e.end_idx);
            const f = try self.createTriangle(e.start_idx, e.end_idx, idx);
            try out_new_faces.append(self.allocator, f);
        }

        // Link edges
        const new_faces = out_new_faces.items;
        for (0..new_faces.len) |i| {
            linkFace(new_faces[i].first_edge.?, edges.items[i].neighbour_edge);
            linkFace(new_faces[i].first_edge.?.next_edge.?, new_faces[(i + 1) % new_faces.len].first_edge.?.next_edge.?.next_edge.?);
        }

        // Loop on faces that were modified until nothing needs to be checked anymore
        var affected_faces = try out_new_faces.clone(self.allocator);
        defer affected_faces.deinit(self.allocator);
        while (affected_faces.items.len != 0) {
            // Take the next face
            const face = affected_faces.pop().?;

            if (!face.removed) {
                // Merge with neighbour if this is a degenerate face
                try self.mergeDegenerateFace(face, &affected_faces);

                // Merge with coplanar neighbours (or when the neighbour forms a concave edge)
                if (!face.removed)
                    try self.mergeCoplanarOrConcaveFaces(face, coplanar_tolerance_sq, &affected_faces);
            }
        }

        // Check if structure is intact
        if (Core.enable_asserts) self.validateFaces();
    }

    /// Remove all faces that have been marked 'removed' from faces list
    fn garbageCollectFaces(self: *ConvexHullBuilder) void {
        var i = self.faces.items.len;
        while (i > 0) {
            i -= 1;
            const f = self.faces.items[i];
            if (f.removed) {
                self.freeFace(f);
                _ = self.faces.orderedRemove(i);
            }
        }
    }

    /// Create a new face
    fn createFace(self: *ConvexHullBuilder) error{OutOfMemory}!*Face {
        // Make room in the list first so that the face can't leak when the list can't grow (Zolt only, Jolt aborts)
        try self.faces.ensureUnusedCapacity(self.allocator, 1);

        // Call provider to create face
        const f = try self.allocator.create(Face);
        f.* = .{};

        // Add to list
        self.faces.appendAssumeCapacity(f);
        return f;
    }

    /// Create a new triangle
    fn createTriangle(self: *ConvexHullBuilder, idx1: i32, idx2: i32, idx3: i32) error{OutOfMemory}!*Face {
        const f = try self.createFace();
        try f.initialize(self.allocator, idx1, idx2, idx3, self.positions);
        return f;
    }

    /// Delete a face (checking that it is not connected to any other faces)
    fn freeFace(self: *ConvexHullBuilder, face: *Face) void {
        assert(face.removed);

        if (Core.enable_asserts) {
            // Make sure that this face is not connected
            if (face.first_edge) |first| {
                var e = first;
                while (true) {
                    assert(e.neighbour_edge == null);
                    e = e.next_edge.?;
                    if (e == first) break;
                }
            }
        }

        // Free the face
        destroyFace(self.allocator, face);
    }

    /// Release all faces and edges
    fn freeFaces(self: *ConvexHullBuilder) void {
        for (self.faces.items) |f|
            destroyFace(self.allocator, f);
        self.faces.clearRetainingCapacity();
    }

    /// Allocate a new edge (new Edge(face, start_idx))
    fn createEdge(allocator: std.mem.Allocator, face: *Face, start_idx: i32) error{OutOfMemory}!*Edge {
        const edge = try allocator.create(Edge);
        edge.* = .init(face, start_idx);
        return edge;
    }

    /// Free a face and its edges (delete face)
    fn destroyFace(allocator: std.mem.Allocator, face: *Face) void {
        face.deinit(allocator);
        allocator.destroy(face);
    }

    /// Free the edges of a face whose edge loop is not closed yet, used when an allocation fails while creating it (Zolt only)
    fn destroyOpenEdgeLoop(allocator: std.mem.Allocator, face: *Face, edges: []const *Edge) void {
        for (edges) |edge|
            allocator.destroy(edge);
        face.first_edge = null;
    }

    /// Link face edge to other face edge
    fn linkFace(edge1: *Edge, edge2: *Edge) void {
        // Check not connected yet
        assert(edge1.neighbour_edge == null);
        assert(edge2.neighbour_edge == null);
        assert(edge1.face != edge2.face);

        // Check vertices match
        assert(edge1.start_idx == edge2.next_edge.?.start_idx);
        assert(edge2.start_idx == edge1.next_edge.?.start_idx);

        // Link up
        edge1.neighbour_edge = edge2;
        edge2.neighbour_edge = edge1;
    }

    /// Unlink this face from all of its neighbours
    fn unlinkFace(face: *Face) void {
        // Unlink from neighbours
        var e = face.first_edge.?;
        while (true) {
            if (e.neighbour_edge) |neighbour_edge| {
                // Validate that neighbour points to us
                assert(neighbour_edge.neighbour_edge == e);

                // Unlink
                neighbour_edge.neighbour_edge = null;
                e.neighbour_edge = null;
            }
            e = e.next_edge.?;
            if (e == face.first_edge.?) break;
        }
    }

    /// Given one face that faces vertex, find the edges of the faces that are not facing vertex.
    /// Will flag all those faces for removal.
    fn findEdge(self: *const ConvexHullBuilder, facing_face: *Face, vertex: Vec3, out_edges: *FullEdges) error{OutOfMemory}!void {
        // Assert that we were given an empty array
        assert(out_edges.items.len == 0);

        // Should start with a facing face
        assert(facing_face.isFacing(vertex));

        // Flag as removed
        facing_face.removed = true;

        // Instead of recursing, we build our own stack with the information we need
        const StackEntry = struct {
            first_edge: *Edge,
            /// Pointer to the current edge, the lowest bit is set for the first edge of the first face
            current_edge: usize,
        };
        const max_edge_length = 128;
        var stack: [max_edge_length]StackEntry = undefined;
        var cur_stack_pos: i32 = 0;

        comptime assert(@alignOf(Edge) >= 2); // Need lowest bit to indicate to tell if we completed the loop

        // Start with the face / edge provided
        stack[0].first_edge = facing_face.first_edge.?;
        stack[0].current_edge = @intFromPtr(facing_face.first_edge.?) | 1; // Set lowest bit of pointer to make it different from the first edge

        while (true) {
            const cur_entry = &stack[@intCast(cur_stack_pos)];

            // Next edge
            const raw_e = cur_entry.current_edge;
            const e: *Edge = @ptrFromInt(raw_e & ~@as(usize, 1)); // Remove the lowest bit which was used to indicate that this is the first edge we're testing
            cur_entry.current_edge = @intFromPtr(e.next_edge.?);

            // If we're back at the first edge we've completed the face and we're done
            if (raw_e == @intFromPtr(cur_entry.first_edge)) {
                // This face needs to be removed, unlink it now, caller will free
                unlinkFace(e.face);

                // Pop from stack
                cur_stack_pos -= 1;
                if (cur_stack_pos < 0)
                    break;
            } else {
                // Visit neighbour face
                if (e.neighbour_edge) |ne| {
                    const n = ne.face;
                    if (!n.removed) {
                        // Check if vertex is on the front side of this face
                        if (n.isFacing(vertex)) {
                            // Vertex on front, this face needs to be removed
                            n.removed = true;

                            // Add element to the stack of elements to visit
                            cur_stack_pos += 1;
                            assert(cur_stack_pos < max_edge_length);
                            const new_entry = &stack[@intCast(cur_stack_pos)];
                            new_entry.first_edge = ne;
                            new_entry.current_edge = @intFromPtr(ne.next_edge.?); // We don't need to test this edge again since we came from it
                        } else {
                            // Vertex behind, keep edge
                            try out_edges.append(self.allocator, .{
                                .neighbour_edge = ne,
                                .start_idx = e.start_idx,
                                .end_idx = ne.start_idx,
                            });
                        }
                    }
                }
            }
        }

        // Assert that we have a fully connected loop
        if (Core.enable_asserts) {
            const items = out_edges.items;
            for (0..items.len) |i|
                assert(items[i].end_idx == items[(i + 1) % items.len].start_idx);
        }
    }

    /// Merges the two faces that share edge into the face edge.face
    fn mergeFaces(self: *ConvexHullBuilder, edge_to_remove: *Edge) error{OutOfMemory}!void {
        // Get the face
        const face = edge_to_remove.face;

        // Find the previous and next edge
        const next_edge = edge_to_remove.next_edge.?;
        const prev_edge = edge_to_remove.getPreviousEdge();

        // Get the other face
        const other_edge = edge_to_remove.neighbour_edge.?;
        const other_face = other_edge.face;

        // Check if attempting to merge with self
        assert(face != other_face);

        // Loop over the edges of the other face and make them belong to face
        var edge = other_edge.next_edge.?;
        prev_edge.next_edge = edge;
        while (true) {
            edge.face = face;
            if (edge.next_edge.? == other_edge) {
                // Terminate when we are back at other_edge
                edge.next_edge = next_edge;
                break;
            }
            edge = edge.next_edge.?;
        }

        // If the first edge happens to be edge_to_remove we need to fix it because this edge is no longer part of the face.
        // Note that we replace it with the first edge of the merged face so that if the mergeFaces function is called
        // from a loop that loops around the face that it will still terminate after visiting all edges once.
        if (face.first_edge.? == edge_to_remove)
            face.first_edge = prev_edge.next_edge;

        // Free the edges
        self.allocator.destroy(edge_to_remove);
        self.allocator.destroy(other_edge);

        // Mark the other face as removed
        other_face.first_edge = null;
        other_face.removed = true;

        // Recalculate plane
        face.calculateNormalAndCentroid(self.positions);

        // Merge conflict lists
        if (face.furthest_point_distance_sq > other_face.furthest_point_distance_sq) {
            // This face has a point that's further away, make sure it remains the last one as we add the other points to this faces list
            try face.conflict_list.insertSlice(self.allocator, face.conflict_list.items.len - 1, other_face.conflict_list.items);
        } else {
            // The other face has a point that's furthest away, add that list at the end.
            try face.conflict_list.appendSlice(self.allocator, other_face.conflict_list.items);
            face.furthest_point_distance_sq = other_face.furthest_point_distance_sq;
        }
        other_face.conflict_list.clearRetainingCapacity();
    }

    /// Merges face with a neighbour if it is degenerate (a sliver)
    fn mergeDegenerateFace(self: *ConvexHullBuilder, face: *Face, affected_faces: *Faces) error{OutOfMemory}!void {
        // Check area of face
        if (face.normal.lengthSq() < min_triangle_area_sq) {
            // Find longest edge, since this face is a sliver this should keep the face convex
            var max_length_sq: f32 = 0.0;
            var longest_edge: ?*Edge = null;
            var e = face.first_edge.?;
            var p1 = self.positionAt(e.start_idx);
            while (true) {
                const next = e.next_edge.?;
                const p2 = self.positionAt(next.start_idx);
                const length_sq = p2.sub(p1).lengthSq();
                if (length_sq >= max_length_sq) {
                    max_length_sq = length_sq;
                    longest_edge = e;
                }
                p1 = p2;
                e = next;
                if (e == face.first_edge.?) break;
            }

            // Merge with face on longest edge
            try self.mergeFaces(longest_edge.?);

            // Remove any invalid edges
            try self.removeInvalidEdges(face, affected_faces);
        }
    }

    /// Merges any coplanar as well as neighbours that form a non-convex edge into face.
    /// Faces are considered coplanar if the distance^2 of the other face's centroid is smaller than coplanar_tolerance_sq.
    fn mergeCoplanarOrConcaveFaces(self: *ConvexHullBuilder, face: *Face, coplanar_tolerance_sq: f32, affected_faces: *Faces) error{OutOfMemory}!void {
        var merged = false;

        var edge = face.first_edge.?;
        while (true) {
            // Store next edge since this edge can be removed
            const next_edge = edge.next_edge.?;

            // Test if centroid of one face is above plane of the other face by coplanar_tolerance_sq.
            // If so we need to merge other face into face.
            const other_face = edge.neighbour_edge.?.face;
            const delta_centroid = other_face.centroid.sub(face.centroid);
            const dist_other_face_centroid = face.normal.dot(delta_centroid);
            const signed_dist_other_face_centroid_sq = @abs(dist_other_face_centroid) * dist_other_face_centroid;
            const dist_face_centroid = -other_face.normal.dot(delta_centroid);
            const signed_dist_face_centroid_sq = @abs(dist_face_centroid) * dist_face_centroid;
            const face_normal_len_sq = face.normal.lengthSq();
            const other_face_normal_len_sq = other_face.normal.lengthSq();
            if ((signed_dist_other_face_centroid_sq > -coplanar_tolerance_sq * face_normal_len_sq or
                signed_dist_face_centroid_sq > -coplanar_tolerance_sq * other_face_normal_len_sq) and
                face.normal.dot(other_face.normal) > 0.0) // Never merge faces that are back to back
            {
                try self.mergeFaces(edge);
                merged = true;
            }

            edge = next_edge;
            if (edge == face.first_edge.?) break;
        }

        if (merged)
            try self.removeInvalidEdges(face, affected_faces);
    }

    /// Mark face as affected if it is not already in the list
    fn markAffected(allocator: std.mem.Allocator, face: *Face, affected_faces: *Faces) error{OutOfMemory}!void {
        if (std.mem.indexOfScalar(*Face, affected_faces.items, face) == null)
            try affected_faces.append(allocator, face);
    }

    /// Removes all invalid edges.
    /// 1. Merges face with faces that share two edges with it since this means face or the other face cannot be convex or the edge is colinear.
    /// 2. Removes edges that are interior to face (that have face on both sides)
    /// Any faces that need to be checked for validity will be added to affected_faces.
    fn removeInvalidEdges(self: *ConvexHullBuilder, face: *Face, affected_faces: *Faces) error{OutOfMemory}!void {
        // This marks that the plane needs to be recalculated (we delay this until the end of the
        // function since we don't use the plane and we want to avoid calculating it multiple times)
        var recalculate_plane = false;

        // We keep going through this loop until no more edges were removed
        while (true) {
            var removed = false;

            // Loop over all edges in this face
            var edge = face.first_edge.?;
            var neighbour_face = edge.neighbour_edge.?.face;
            while (true) {
                const next_edge = edge.next_edge.?;
                const next_neighbour_face = next_edge.neighbour_edge.?.face;

                if (neighbour_face == face) {
                    // We only remove 1 edge at a time, check if this edge's next edge is our neighbour.
                    // If this check fails, we will continue to scan along the edge until we find an edge where this is the case.
                    if (edge.neighbour_edge.? == next_edge) {
                        // This edge leads back to the starting point, this means the edge is interior and needs to be removed

                        // Remove edge
                        const prev_edge = edge.getPreviousEdge();
                        prev_edge.next_edge = next_edge.next_edge;
                        if (face.first_edge.? == edge or face.first_edge.? == next_edge)
                            face.first_edge = prev_edge;
                        self.allocator.destroy(edge);
                        self.allocator.destroy(next_edge);

                        // Check if face now has only 2 edges left
                        if (try self.removeTwoEdgeFace(face, affected_faces))
                            return; // Bail if face no longer exists

                        // Restart the loop
                        recalculate_plane = true;
                        removed = true;
                        break;
                    }
                } else if (neighbour_face == next_neighbour_face) {
                    // There are two edges that connect to the same face, we will remove the second one

                    // First merge the neighbours edges
                    const neighbour_edge = next_edge.neighbour_edge.?;
                    const next_neighbour_edge = neighbour_edge.next_edge.?;
                    if (neighbour_face.first_edge.? == next_neighbour_edge)
                        neighbour_face.first_edge = neighbour_edge;
                    neighbour_edge.next_edge = next_neighbour_edge.next_edge;
                    neighbour_edge.neighbour_edge = edge;
                    self.allocator.destroy(next_neighbour_edge);

                    // Then merge my own edges
                    if (face.first_edge.? == next_edge)
                        face.first_edge = edge;
                    edge.next_edge = next_edge.next_edge;
                    edge.neighbour_edge = neighbour_edge;
                    self.allocator.destroy(next_edge);

                    // Check if neighbour has only 2 edges left
                    if (!try self.removeTwoEdgeFace(neighbour_face, affected_faces)) {
                        // No, we need to recalculate its plane
                        neighbour_face.calculateNormalAndCentroid(self.positions);

                        // Mark neighbour face as affected
                        try markAffected(self.allocator, neighbour_face, affected_faces);
                    }

                    // Check if face now has only 2 edges left
                    if (try self.removeTwoEdgeFace(face, affected_faces))
                        return; // Bail if face no longer exists

                    // Restart loop
                    recalculate_plane = true;
                    removed = true;
                    break;
                }

                // This edge is ok, go to the next edge
                edge = next_edge;
                neighbour_face = next_neighbour_face;

                if (edge == face.first_edge.?) break;
            }

            if (!removed) break;
        }

        // Recalculate plane?
        if (recalculate_plane)
            face.calculateNormalAndCentroid(self.positions);
    }

    /// Removes face if it consists of only 2 edges, linking its neighbouring faces together
    /// Any faces that need to be checked for validity will be added to affected_faces.
    /// @return True if face was removed.
    fn removeTwoEdgeFace(self: *const ConvexHullBuilder, face: *Face, affected_faces: *Faces) error{OutOfMemory}!bool {
        // Check if this face contains only 2 edges
        const edge = face.first_edge.?;
        const next_edge = edge.next_edge.?;
        assert(edge != next_edge); // 1 edge faces should not exist
        if (next_edge.next_edge.? == edge) {
            // Schedule both neighbours for re-checking
            const neighbour_edge = edge.neighbour_edge.?;
            const neighbour_face = neighbour_edge.face;
            const next_neighbour_edge = next_edge.neighbour_edge.?;
            const next_neighbour_face = next_neighbour_edge.face;
            try markAffected(self.allocator, neighbour_face, affected_faces);
            try markAffected(self.allocator, next_neighbour_face, affected_faces);

            // Link my neighbours to each other
            neighbour_edge.neighbour_edge = next_neighbour_edge;
            next_neighbour_edge.neighbour_edge = neighbour_edge;

            // Unlink my edges
            edge.neighbour_edge = null;
            next_edge.neighbour_edge = null;

            // Mark this face as removed
            face.removed = true;

            return true;
        }

        return false;
    }

    /// Dumps the text representation of a face to the log (only with asserts enabled in Jolt)
    fn dumpFace(self: *const ConvexHullBuilder, face: *const Face) void {
        _ = self;
        log.info("f:0x{x}", .{@intFromPtr(face)});

        var e = face.first_edge.?;
        while (true) {
            log.info("\te:0x{x} {{ i:{d} e:0x{x} f:0x{x} }}", .{ @intFromPtr(e), e.start_idx, @intFromPtr(e.neighbour_edge.?), @intFromPtr(e.neighbour_edge.?.face) });
            e = e.next_edge.?;
            if (e == face.first_edge.?) break;
        }
    }

    /// Dumps the text representation of all faces to the log (only with asserts enabled in Jolt)
    fn dumpFaces(self: *const ConvexHullBuilder) void {
        log.info("Dump Faces:", .{});

        for (self.faces.items) |f| {
            if (!f.removed)
                self.dumpFace(f);
        }
    }

    /// Check consistency of 1 face (only called with asserts enabled)
    fn validateFace(self: *const ConvexHullBuilder, face: *const Face) void {
        if (face.removed) {
            if (face.first_edge) |first| {
                var e = first;
                while (true) {
                    assert(e.neighbour_edge == null);
                    e = e.next_edge.?;
                    if (e == first) break;
                }
            }
        } else {
            var edge_count: i32 = 0;

            var e = face.first_edge.?;
            while (true) {
                // Count edge
                edge_count += 1;

                // Validate that adjacent faces are all different
                if (self.faces.items.len > 2) {
                    var other_edge = e.next_edge.?;
                    while (other_edge != face.first_edge.?) : (other_edge = other_edge.next_edge.?)
                        assert(e.neighbour_edge.?.face != other_edge.neighbour_edge.?.face);
                }

                // Assert that the face is correct
                assert(e.face == face);

                // Assert that we have a neighbour
                const nb_edge = e.neighbour_edge;
                assert(nb_edge != null);
                if (nb_edge) |nb| {
                    // Assert that our neighbours edge points to us
                    assert(nb.neighbour_edge == e);

                    // Assert that it belongs to a different face
                    assert(nb.face != face);

                    // Assert that the next edge of the neighbour points to the same vertex as this edge's vertex
                    assert(nb.next_edge.?.start_idx == e.start_idx);

                    // Assert that my next edge points to the same vertex as my neighbours vertex
                    assert(e.next_edge.?.start_idx == nb.start_idx);
                }
                e = e.next_edge.?;
                if (e == face.first_edge.?) break;
            }

            // Assert that we have 3 or more edges
            assert(edge_count >= 3);
        }
    }

    /// Check consistency of all faces (only called with asserts enabled)
    fn validateFaces(self: *const ConvexHullBuilder) void {
        for (self.faces.items) |f|
            self.validateFace(f);
    }
};

/// Collect the vertex indices of a face in edge order (test helper)
fn faceIndices(face: *const ConvexHullBuilder.Face, buffer: []i32) []i32 {
    var count: usize = 0;
    var e = face.first_edge.?;
    while (true) {
        buffer[count] = e.start_idx;
        count += 1;
        e = e.next_edge.?;
        if (e == face.first_edge.?) break;
    }
    return buffer[0..count];
}

test "ConvexHullBuilder tetrahedron and reuse" {
    const allocator = std.testing.allocator;
    const no_limit = std.math.maxInt(i32);

    // Tetrahedron with interior points, a point on a face and a duplicate
    const positions = [_]Vec3{
        Vec3.init(0, 0, 0),
        Vec3.init(1, 0, 0),
        Vec3.init(0, 1, 0),
        Vec3.init(0, 0, 1),
        Vec3.init(0.1, 0.1, 0.1), // interior
        Vec3.init(0.25, 0.25, 0), // on the face z = 0
        Vec3.init(1, 0, 0), // duplicate of 1
    };
    var builder = ConvexHullBuilder.init(allocator, &positions);
    defer builder.deinit();

    const result = try builder.initialize(no_limit, 1.0e-3);
    try std.testing.expectEqual(ConvexHullBuilder.Result.success, result.result);
    try std.testing.expect(result.error_message == null);
    try std.testing.expectEqual(4, builder.getFaces().len);
    try std.testing.expectEqual(4, try builder.getNumVerticesUsed());
    try std.testing.expect(builder.containsFace(&.{ 0, 2, 1 }));
    try std.testing.expect(builder.containsFace(&.{ 2, 1, 0 }));
    try std.testing.expect(!builder.containsFace(&.{ 0, 1, 2 }));
    try std.testing.expect(builder.containsFace(&.{ 1, 2, 3 }));

    // Volume of the tetrahedron is 1/6, center of mass at 1/4
    const mass = builder.getCenterOfMassAndVolume();
    try std.testing.expectApproxEqAbs(@as(f32, 1.0 / 6.0), mass.volume, 1.0e-6);
    try std.testing.expect(mass.center_of_mass.isClose(Vec3.replicate(0.25), .{ .max_dist_sq = 1.0e-12 }));

    // All points are inside the hull
    const max_error = builder.determineMaxError();
    try std.testing.expect(max_error.max_error <= max_error.coplanar_distance);

    // The debug helpers
    builder.dumpFaces();
    builder.validateFaces();

    // A second initialize frees the previous hull, the vertex limit is reached immediately
    const limited = try builder.initialize(4, 1.0e-3);
    try std.testing.expectEqual(ConvexHullBuilder.Result.success, limited.result);
    try std.testing.expectEqual(4, builder.getFaces().len);

    // A cube with the vertex limit 4 is a tetrahedron
    const cube = [_]Vec3{
        Vec3.init(-1, -1, -1), Vec3.init(1, -1, -1), Vec3.init(-1, 1, -1), Vec3.init(1, 1, -1),
        Vec3.init(-1, -1, 1),  Vec3.init(1, -1, 1),  Vec3.init(-1, 1, 1),  Vec3.init(1, 1, 1),
    };
    builder.positions = &cube;
    const cube_limited = try builder.initialize(4, 1.0e-3);
    try std.testing.expectEqual(ConvexHullBuilder.Result.max_vertices_reached, cube_limited.result);
    try std.testing.expectEqual(4, try builder.getNumVerticesUsed());

    const cube_result = try builder.initialize(no_limit, 1.0e-3);
    try std.testing.expectEqual(ConvexHullBuilder.Result.success, cube_result.result);
    try std.testing.expectEqual(6, builder.getFaces().len);
    try std.testing.expectEqual(8, try builder.getNumVerticesUsed());
    var buffer: [8]i32 = undefined;
    for (builder.getFaces()) |f|
        try std.testing.expectEqual(4, faceIndices(f, &buffer).len);
    const cube_mass = builder.getCenterOfMassAndVolume();
    try std.testing.expectApproxEqAbs(@as(f32, 8.0), cube_mass.volume, 1.0e-5);
    try std.testing.expect(cube_mass.center_of_mass.isNearZero(.{ .max_dist_sq = 1.0e-10 }));
}

test "ConvexHullBuilder result codes" {
    const allocator = std.testing.allocator;
    const no_limit = std.math.maxInt(i32);

    // Too few points
    const two = [_]Vec3{ Vec3.init(0, 0, 0), Vec3.init(1, 0, 0) };
    var builder = ConvexHullBuilder.init(allocator, &two);
    defer builder.deinit();
    const too_few = try builder.initialize(no_limit, 1.0e-3);
    try std.testing.expectEqual(ConvexHullBuilder.Result.too_few_points, too_few.result);
    try std.testing.expectEqualStrings("Need at least 3 points to make a hull", too_few.error_message.?);

    // Colinear points
    const line = [_]Vec3{ Vec3.init(0, 0, 0), Vec3.init(1, 0, 0), Vec3.init(2, 0, 0), Vec3.init(3, 0, 0) };
    builder.positions = &line;
    const degenerate = try builder.initialize(no_limit, 1.0e-3);
    try std.testing.expectEqual(ConvexHullBuilder.Result.degenerate, degenerate.result);
    try std.testing.expectEqualStrings("Could not find a suitable initial triangle because its area was too small", degenerate.error_message.?);

    // Flat square with a vertex limit: 2D fallback
    const square = [_]Vec3{ Vec3.init(-1, 0, -1), Vec3.init(1, 0, -1), Vec3.init(1, 0, 1), Vec3.init(-1, 0, 1), Vec3.init(0, 0, 0) };
    builder.positions = &square;
    const flat = try builder.initialize(3, 1.0e-3);
    try std.testing.expectEqual(ConvexHullBuilder.Result.max_vertices_reached, flat.result);
    try std.testing.expectEqual(2, builder.getFaces().len);
    try std.testing.expectEqual(3, try builder.getNumVerticesUsed());
    const flat_full = try builder.initialize(no_limit, 1.0e-3);
    try std.testing.expectEqual(ConvexHullBuilder.Result.success, flat_full.result);
    try std.testing.expectEqual(4, try builder.getNumVerticesUsed());
    const flat_mass = builder.getCenterOfMassAndVolume();
    try std.testing.expectEqual(@as(f32, 0.0), flat_mass.volume);
}

fn buildHullAllocations(allocator: std.mem.Allocator, positions: []const Vec3, max_vertices: i32) !void {
    var builder = ConvexHullBuilder.init(allocator, positions);
    defer builder.deinit();
    _ = try builder.initialize(max_vertices, 1.0e-3);
    _ = try builder.getNumVerticesUsed();
    _ = try builder.initialize(max_vertices, 1.0e-2);
}

test "ConvexHullBuilder allocation failures" {
    // Points on a sphere with interior and nearly coplanar points: needs face merges, coplanar points and conflict lists
    var positions: [60]Vec3 = undefined;
    for (&positions, 0..) |*p, i| {
        const f: f32 = @floatFromInt(i);
        const sc = Vec4.init(0.7 * f, 1.3 * f, 0, 0).sinCos();
        const dir = Vec3.init(sc.sin.getX() * sc.cos.getY(), sc.sin.getX() * sc.sin.getY(), sc.cos.getX());
        const radius: f32 = if (i % 5 == 0) 0.5 else 1.0;
        p.* = dir.mulScalar(radius);
        if (i % 7 == 0) p.setZ(1.0); // Flat top
    }

    // Every allocation failure must be reported without leaking memory
    try std.testing.checkAllAllocationFailures(std.testing.allocator, buildHullAllocations, .{ positions[0..], std.math.maxInt(i32) });
    try std.testing.checkAllAllocationFailures(std.testing.allocator, buildHullAllocations, .{ positions[0..], 10 });

    // The 2D fallback
    var flat: [20]Vec3 = undefined;
    for (&flat, 0..) |*p, i| {
        const sc = Vec4.replicate(@as(f32, @floatFromInt(i)) * 0.9).sinCos();
        p.* = Vec3.init(sc.cos.getX(), 2.0, sc.sin.getX());
    }
    try std.testing.checkAllAllocationFailures(std.testing.allocator, buildHullAllocations, .{ flat[0..], std.math.maxInt(i32) });
}
