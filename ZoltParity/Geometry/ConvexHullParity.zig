//! Parity tests for the 3D convex hull builder (ConvexHullBuilder): run Zolt and the C++ Jolt library on the same
//! point clouds and require identical results. C ABI wrappers: ZoltParity/Geometry/ConvexHullReference.cpp. See
//! ZoltParity/parity.zig for how parity tests work.
//!
//! Every hull is compared completely: the result code and error string of `initialize`, the face list in order (for
//! each face the vertex indices in edge order, the neighbour face / edge of every edge, the normal, the centroid, the
//! conflict list in order, the furthest point distance and the removed flag), `getNumVerticesUsed`,
//! `getCenterOfMassAndVolume`, `determineMaxError` and `containsFace` queries. A builder is often initialized again
//! with other parameters, which tests the cleanup of the previous hull (and Jolt's coplanar list that survives an
//! early `max_vertices_reached` return).
//!
//! The point clouds are uniform, on / in a sphere, flat (2D fallback), nearly flat, colinear, on integer grids (exact
//! ties, coplanar faces that need merging), boxes with points on their faces, clusters, cylinders, Jolt's
//! TestRandomHull distribution (duplicates and near duplicates), tiny / huge / translated clouds and clouds with
//! fewer than 4 points, with small and large max vertex counts and several tolerances.
//!
//! Some nearly degenerate clouds violate Jolt's own asserts (see `ConvexHullBuilder.initialize`). A Jolt build without
//! asserts (the reference) continues, Jolt and Zolt builds with asserts abort. With `Core.enable_asserts` (Debug,
//! ReleaseSafe) these clouds are skipped (`assert_clouds`) and `determineMaxError` is not called for hulls with a zero
//! face normal. In ReleaseFast / ReleaseSmall everything is compared (`zig build parity -Doptimize=ReleaseFast`).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const ConvexHullBuilder = zolt.ConvexHullBuilder;
const Vec3 = zolt.Vec3;

/// The C++ reference functions, see ConvexHullReference.cpp
const jolt = struct {
    extern fn jolt_chb_create(positions: [*]const f32, count: c_int) *anyopaque;
    extern fn jolt_chb_destroy(builder: *anyopaque) void;
    extern fn jolt_chb_initialize(builder: *anyopaque, max_vertices: c_int, tolerance: f32, io_error: *?[*:0]const u8) c_int;
    extern fn jolt_chb_get_num_vertices_used(builder: *const anyopaque) c_int;
    extern fn jolt_chb_contains_face(builder: *const anyopaque, indices: [*]const c_int, count: c_int) bool;
    extern fn jolt_chb_get_center_of_mass_and_volume(builder: *const anyopaque, out_center_of_mass: *[3]f32, out_volume: *f32) void;
    extern fn jolt_chb_determine_max_error(builder: *const anyopaque, out_face_idx: *c_int, out_max_error: *f32, out_max_error_position_idx: *c_int, out_coplanar_distance: *f32) void;
    extern fn jolt_chb_get_num_faces(builder: *const anyopaque) c_int;
    extern fn jolt_chb_get_face(builder: *const anyopaque, face_idx: c_int, capacity: c_int, out_num_edges: *c_int, out_start_indices: [*]c_int, out_neighbour_faces: [*]c_int, out_neighbour_edges: [*]c_int, out_normal: *[3]f32, out_centroid: *[3]f32, out_num_conflicts: *c_int, out_conflict_list: [*]c_int, out_furthest_point_distance_sq: *f32, out_removed: *bool) void;
};

/// Maximum number of positions in a point cloud (also the capacity of the per face arrays)
const max_points = 1000;

/// Number of random point clouds
const num_clouds = 4000;

const no_limit = std.math.maxInt(i32);

/// The random clouds that violate a Jolt assert in `initialize` (`edges.size() >= 3` in AddPoint or `IsFacing` in
/// FindEdge). Found by printing the inputs (`dump_inputs`) and building them with a Jolt build with JPH_ENABLE_ASSERTS
/// (Jolt/Geometry/ConvexHullBuilder*.cpp + Jolt/Core/{Memory,IssueReporting,StringTools}.cpp and a small main that
/// reads the dump, initializes a builder per PARAMS line and installs an AssertFailed hook that records the assert
/// and returns false). Update this list when the generator changes.
const assert_clouds = [_]usize{ 391, 1461, 1694, 2052, 2083, 3380, 3556 };

/// Print the inputs instead of testing them (CLOUD <index>, P <x> <y> <z> as hex float bits, PARAMS <max vertices> <tolerance bits>)
const dump_inputs = false;

/// Everything that is compared for a hull except the faces
const Summary = struct {
    result: i32,
    /// The error string (Jolt's outError, which keeps its value when building succeeds), zero padded
    error_message: [96]u8,
    num_faces: i32,
    num_vertices_used: i32,
    center_of_mass: [3]f32,
    volume: f32,
    max_error_face: i32,
    max_error: f32,
    max_error_position_idx: i32,
    coplanar_distance: f32,
};

/// Everything stored in a face, unused array entries are zero
const FaceData = struct {
    num_edges: i32,
    start_indices: [max_points]i32,
    neighbour_faces: [max_points]i32,
    neighbour_edges: [max_points]i32,
    normal: [3]f32,
    centroid: [3]f32,
    num_conflicts: i32,
    conflict_list: [max_points]i32,
    furthest_point_distance_sq: f32,
    removed: bool,
};

/// Parameters of one call to initialize
const Params = struct {
    max_vertices: i32,
    tolerance: f32,
};

fn copyErrorMessage(message: ?[]const u8) [96]u8 {
    var result: [96]u8 = @splat(0);
    if (message) |m| {
        std.debug.assert(m.len < result.len);
        @memcpy(result[0..m.len], m);
    }
    return result;
}

/// Index of face in the face list, -1 if not found / null
fn zoltFaceIndex(builder: *const ConvexHullBuilder, face: ?*const ConvexHullBuilder.Face) i32 {
    const f = face orelse return -1;
    for (builder.getFaces(), 0..) |other, i|
        if (other == f) return @intCast(i);
    return -1;
}

/// Position of edge in the edge loop of its face, -1 if not found / null
fn zoltEdgeIndex(edge: ?*const ConvexHullBuilder.Edge) i32 {
    const e = edge orelse return -1;
    const first = e.face.first_edge orelse return -1;
    var idx: i32 = 0;
    var cur = first;
    while (true) {
        if (cur == e) return idx;
        idx += 1;
        cur = cur.next_edge.?;
        if (cur == first) break;
    }
    return -1;
}

fn zoltFace(builder: *const ConvexHullBuilder, face_idx: usize, out: *FaceData) void {
    out.* = std.mem.zeroes(FaceData);
    const face = builder.getFaces()[face_idx];
    var num_edges: usize = 0;
    if (face.first_edge) |first| {
        var e = first;
        while (true) {
            if (num_edges < max_points) {
                out.start_indices[num_edges] = e.start_idx;
                out.neighbour_faces[num_edges] = if (e.neighbour_edge) |ne| zoltFaceIndex(builder, ne.face) else -1;
                out.neighbour_edges[num_edges] = zoltEdgeIndex(e.neighbour_edge);
            }
            num_edges += 1;
            e = e.next_edge.?;
            if (e == first) break;
        }
    }
    out.num_edges = @intCast(num_edges);
    out.normal = .{ face.normal.getX(), face.normal.getY(), face.normal.getZ() };
    out.centroid = .{ face.centroid.getX(), face.centroid.getY(), face.centroid.getZ() };
    out.num_conflicts = @intCast(face.conflict_list.items.len);
    for (face.conflict_list.items, 0..) |idx, i| {
        if (i < max_points) out.conflict_list[i] = idx;
    }
    out.furthest_point_distance_sq = face.furthest_point_distance_sq;
    out.removed = face.removed;
}

fn joltFace(builder: *const anyopaque, face_idx: usize, out: *FaceData) void {
    out.* = std.mem.zeroes(FaceData);
    var num_edges: c_int = 0;
    var num_conflicts: c_int = 0;
    jolt.jolt_chb_get_face(builder, @intCast(face_idx), max_points, &num_edges, &out.start_indices, &out.neighbour_faces, &out.neighbour_edges, &out.normal, &out.centroid, &num_conflicts, &out.conflict_list, &out.furthest_point_distance_sq, &out.removed);
    out.num_edges = num_edges;
    out.num_conflicts = num_conflicts;
}

/// Compare one face, prints a compact description of the first mismatches
fn checkFace(checker: *Checker, cloud: usize, face_idx: usize, zolt_face: *const FaceData, jolt_face: *const FaceData) void {
    if (fw.sameValue(zolt_face.*, jolt_face.*))
        return;
    if (checker.mismatches < 5) {
        const ze: usize = @intCast(@min(zolt_face.num_edges, max_points));
        const je: usize = @intCast(@min(jolt_face.num_edges, max_points));
        const zc: usize = @intCast(@min(zolt_face.num_conflicts, max_points));
        const jc: usize = @intCast(@min(jolt_face.num_conflicts, max_points));
        std.debug.print("{s}: face {d} of cloud {d} differs\n  zolt: edges {any} neighbours {any} / {any} normal {any} centroid {any} conflicts {any} furthest {} removed {}\n  jolt: edges {any} neighbours {any} / {any} normal {any} centroid {any} conflicts {any} furthest {} removed {}\n", .{
            checker.name,                         cloud,                            face_idx,
            zolt_face.start_indices[0..ze],       zolt_face.neighbour_faces[0..ze], zolt_face.neighbour_edges[0..ze],
            zolt_face.normal,                     zolt_face.centroid,               zolt_face.conflict_list[0..zc],
            zolt_face.furthest_point_distance_sq, zolt_face.removed,                jolt_face.start_indices[0..je],
            jolt_face.neighbour_faces[0..je],     jolt_face.neighbour_edges[0..je], jolt_face.normal,
            jolt_face.centroid,                   jolt_face.conflict_list[0..jc],   jolt_face.furthest_point_distance_sq,
            jolt_face.removed,
        });
    }
    checker.mismatches += 1;
}

/// Statistics of the generated inputs, to check that the interesting paths are covered
const Stats = struct {
    clouds: usize = 0,
    results: [5]usize = @splat(0),
    two_d: usize = 0,
    faces: usize = 0,
    max_faces: usize = 0,
    leftover_conflicts: usize = 0,
    zero_normals: usize = 0,
    skipped_clouds: usize = 0,
    /// Initializations that started with a coplanar list left over from an earlier max_vertices_reached return
    stale_coplanar: usize = 0,
};

/// True if no face of the hull has a zero normal (precondition of determineMaxError)
fn allNormalsNonZero(builder: *const ConvexHullBuilder) bool {
    for (builder.getFaces()) |f|
        if (!(f.normal.length() > 0.0)) return false;
    return true;
}

/// Build hulls of positions with Zolt and Jolt (one builder each, initialized once per entry of params) and compare everything
fn checkHull(ctx: *Context, cloud: usize, positions: []const Vec3, params: []const Params) !void {
    std.debug.assert(positions.len <= max_points);
    if (dump_inputs) {
        std.debug.print("CLOUD {d}\n", .{cloud});
        for (positions) |q| std.debug.print("P {x} {x} {x}\n", .{ @as(u32, @bitCast(q.getX())), @as(u32, @bitCast(q.getY())), @as(u32, @bitCast(q.getZ())) });
        for (params) |p| std.debug.print("PARAMS {d} {x}\n", .{ p.max_vertices, @as(u32, @bitCast(p.tolerance)) });
        return;
    }
    var flat: [3 * max_points]f32 = undefined;
    for (positions, 0..) |p, i| {
        flat[3 * i + 0] = p.getX();
        flat[3 * i + 1] = p.getY();
        flat[3 * i + 2] = p.getZ();
    }
    const jolt_builder = jolt.jolt_chb_create(&flat, @intCast(positions.len));
    defer jolt.jolt_chb_destroy(jolt_builder);

    var zolt_builder = ConvexHullBuilder.init(std.testing.allocator, positions);
    defer zolt_builder.deinit();

    // Jolt's outError keeps its value when there is no error, emulate that for Zolt
    var jolt_error: ?[*:0]const u8 = null;
    var zolt_error: ?[]const u8 = null;

    for (params) |p| {
        ctx.stats.clouds += 1;
        if (zolt_builder.coplanar_list.items.len > 0) ctx.stats.stale_coplanar += 1;

        // Initialize
        var expected = std.mem.zeroes(Summary);
        expected.result = jolt.jolt_chb_initialize(jolt_builder, p.max_vertices, p.tolerance, &jolt_error);
        expected.error_message = copyErrorMessage(if (jolt_error) |e| std.mem.span(e) else null);

        var actual = std.mem.zeroes(Summary);
        const zolt_result = try zolt_builder.initialize(p.max_vertices, p.tolerance);
        actual.result = @intFromEnum(zolt_result.result);
        if (zolt_result.error_message) |e| zolt_error = e;
        actual.error_message = copyErrorMessage(zolt_error);

        // Queries
        expected.num_faces = jolt.jolt_chb_get_num_faces(jolt_builder);
        actual.num_faces = @intCast(zolt_builder.getFaces().len);
        expected.num_vertices_used = jolt.jolt_chb_get_num_vertices_used(jolt_builder);
        actual.num_vertices_used = try zolt_builder.getNumVerticesUsed();
        jolt.jolt_chb_get_center_of_mass_and_volume(jolt_builder, &expected.center_of_mass, &expected.volume);
        const mass = zolt_builder.getCenterOfMassAndVolume();
        actual.center_of_mass = .{ mass.center_of_mass.getX(), mass.center_of_mass.getY(), mass.center_of_mass.getZ() };
        actual.volume = mass.volume;
        // determineMaxError asserts that every face has a normal (JPH_ASSERT(normal_len > 0.0f)). Jolt only calls it for
        // a successfully built hull, but even then a nearly degenerate cloud can leave faces with a zero normal (Jolt
        // without asserts skips them because the plane distance is NaN, a build with asserts aborts).
        if (!zolt.Core.enable_asserts or allNormalsNonZero(&zolt_builder)) {
            jolt.jolt_chb_determine_max_error(jolt_builder, &expected.max_error_face, &expected.max_error, &expected.max_error_position_idx, &expected.coplanar_distance);
            const max_error = zolt_builder.determineMaxError();
            actual.max_error_face = zoltFaceIndex(&zolt_builder, max_error.face_with_max_error);
            actual.max_error = max_error.max_error;
            actual.max_error_position_idx = max_error.max_error_position_idx;
            actual.coplanar_distance = max_error.coplanar_distance;
        } else {
            ctx.stats.zero_normals += 1;
        }

        ctx.summary.check(.{ cloud, positions.len, p.max_vertices, p.tolerance }, actual, expected);

        ctx.stats.results[@intFromEnum(zolt_result.result)] += 1;
        if (actual.num_faces == 2 and positions.len > 3) ctx.stats.two_d += 1; // The 2D fallback
        ctx.stats.faces += @intCast(actual.num_faces);
        ctx.stats.max_faces = @max(ctx.stats.max_faces, @as(usize, @intCast(actual.num_faces)));

        // Faces
        if (actual.num_faces != expected.num_faces)
            continue;
        var zolt_face: FaceData = undefined;
        var jolt_face: FaceData = undefined;
        for (0..@intCast(actual.num_faces)) |f| {
            zoltFace(&zolt_builder, f, &zolt_face);
            joltFace(jolt_builder, f, &jolt_face);
            checkFace(ctx.faces, cloud, f, &zolt_face, &jolt_face);
            ctx.stats.leftover_conflicts += @intCast(zolt_face.num_conflicts);

            // ContainsFace with the vertices of this face: rotated (found), reversed and a prefix
            const n: usize = @intCast(@min(zolt_face.num_edges, max_points));
            if (n == 0) continue;
            var indices: [max_points]i32 = undefined;
            const offset = ctx.query_gen.index(n);
            for (0..n) |i| indices[i] = zolt_face.start_indices[(i + offset) % n];
            checkContainsFace(ctx.contains_face, cloud, &zolt_builder, jolt_builder, indices[0..n]);
            std.mem.reverse(i32, indices[0..n]);
            checkContainsFace(ctx.contains_face, cloud, &zolt_builder, jolt_builder, indices[0..n]);
            checkContainsFace(ctx.contains_face, cloud, &zolt_builder, jolt_builder, indices[0 .. 1 + ctx.query_gen.index(n)]);
        }

        // ContainsFace with random indices
        if (positions.len > 0) {
            var indices: [8]i32 = undefined;
            const n = 1 + ctx.query_gen.index(indices.len);
            for (indices[0..n]) |*i| i.* = @intCast(ctx.query_gen.index(positions.len));
            checkContainsFace(ctx.contains_face, cloud, &zolt_builder, jolt_builder, indices[0..n]);
        }
        checkContainsFace(ctx.contains_face, cloud, &zolt_builder, jolt_builder, &.{});
    }
}

fn checkContainsFace(checker: *Checker, cloud: usize, zolt_builder: *const ConvexHullBuilder, jolt_builder: *const anyopaque, indices: []const i32) void {
    const expected = jolt.jolt_chb_contains_face(jolt_builder, indices.ptr, @intCast(indices.len));
    const actual = zolt_builder.containsFace(indices);
    checker.check(.{ cloud, indices.len }, actual, expected);
}

/// Checkers and state shared by all hulls
const Context = struct {
    summary: *Checker,
    faces: *Checker,
    contains_face: *Checker,
    stats: *Stats,
    /// Generator for the containsFace queries (separate from the input generator so that the inputs don't depend on the results)
    query_gen: Gen = .{ .rng = .{ .state = 0x9e3779b9 } },
};

/// Input generator: xorshift32 with helpers for point clouds
const Gen = struct {
    rng: fw.Rng = .{},

    fn next(self: *Gen) u32 {
        return self.rng.next();
    }

    /// Random index in [0, n)
    fn index(self: *Gen, n: usize) usize {
        return self.next() % n;
    }

    /// True one in n times
    fn oneIn(self: *Gen, n: u32) bool {
        return self.next() % n == 0;
    }

    fn float(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn vec(self: *Gen, min: f32, max: f32) Vec3 {
        return Vec3.init(self.float(min, max), self.float(min, max), self.float(min, max));
    }

    /// Integer in [-n, n] as float
    fn grid(self: *Gen, n: i32) f32 {
        return @floatFromInt(self.rng.intRange(i32, -n, n));
    }

    /// Random unit vector
    fn unit(self: *Gen) Vec3 {
        while (true) {
            const v = self.vec(-1, 1);
            const len_sq = v.lengthSq();
            if (len_sq > 1.0e-4 and len_sq <= 1.0) return v.mulScalar(1.0 / @sqrt(len_sq));
        }
    }

    /// Random power of 10
    fn scale(self: *Gen) f32 {
        const scales = [_]f32{ 1.0e-5, 1.0e-4, 1.0e-3, 1.0e-2, 0.1, 1.0, 10.0, 100.0, 1.0e3, 1.0e4, 1.0e5 };
        return scales[self.index(scales.len)];
    }

    /// Number of points in a cloud
    fn count(self: *Gen) usize {
        return switch (self.next() % 20) {
            0 => self.index(6), // 0 .. 5
            1, 2, 3, 4 => 6 + self.index(10),
            5, 6, 7, 8, 9, 10, 11, 12, 13 => 16 + self.index(64),
            14, 15, 16, 17, 18 => 80 + self.index(220),
            else => 300 + self.index(max_points - 300 + 1),
        };
    }

    /// Fill points with a random point cloud
    fn cloud(self: *Gen, points: []Vec3) void {
        switch (self.next() % 14) {
            // Uniform in a box
            0 => for (points) |*p| {
                p.* = self.vec(-1, 1);
            },
            // On a sphere (many hull vertices)
            1 => for (points) |*p| {
                p.* = self.unit();
            },
            // In and on a sphere
            2 => for (points) |*p| {
                p.* = self.unit().mulScalar(if (self.oneIn(2)) 1.0 else self.float(0, 1));
            },
            // Flat: exactly on an axis aligned plane (2D fallback)
            3 => {
                const axis = self.index(3);
                const c = self.float(-1, 1);
                for (points) |*p| {
                    p.* = self.vec(-1, 1);
                    p.setComponent(@intCast(axis), c);
                }
            },
            // Flat or nearly flat: on a random plane (rounding makes it nearly coplanar) plus noise
            4 => {
                const n = self.unit();
                const u = n.getNormalizedPerpendicular();
                const v = n.cross(u);
                const o = self.vec(-1, 1);
                const noises = [_]f32{ 0, 0, 1.0e-8, 1.0e-7, 1.0e-6, 3.0e-6, 1.0e-5, 1.0e-4, 1.0e-3 };
                const noise = noises[self.index(noises.len)];
                const circle = self.oneIn(3);
                for (points) |*p| {
                    var a = self.float(-1, 1);
                    var b = self.float(-1, 1);
                    if (circle) {
                        const len = @sqrt(a * a + b * b);
                        if (len > 0) {
                            a /= len;
                            b /= len;
                        }
                    }
                    p.* = o.add(u.mulScalar(a)).add(v.mulScalar(b)).add(n.mulScalar(self.float(-noise, noise)));
                }
            },
            // Colinear or nearly colinear
            5 => {
                const d = self.unit();
                const o = self.vec(-1, 1);
                const noise: f32 = if (self.oneIn(2)) 0 else 1.0e-6;
                for (points) |*p| p.* = o.add(d.mulScalar(self.float(-2, 2))).add(self.vec(-noise, noise));
            },
            // Integer grid (exact ties, duplicates, coplanar faces)
            6 => {
                const n: i32 = @intCast(1 + self.index(3));
                for (points) |*p| p.* = Vec3.init(self.grid(n), self.grid(n), self.grid(n));
            },
            // Box with points on its faces, edges and corners
            7 => {
                const half = self.vec(0.1, 2);
                for (points) |*p| {
                    var q = self.vec(-1, 1);
                    const fixed = self.index(4); // 0..2: one coordinate on the surface, 3: corner
                    for (0..3) |c| {
                        if (fixed == 3 or c == fixed) q.setComponent(@intCast(c), if (self.oneIn(2)) 1.0 else -1.0);
                    }
                    if (self.oneIn(5)) q = q.mulScalar(self.float(0, 1)); // interior
                    p.* = q.mul(half);
                }
            },
            // Jolt's TestRandomHull: random points, points close to them and points on lines between them
            8 => {
                const s = Vec3.init(self.float(0.1, 1), self.float(0.1, 1), self.float(0.1, 1));
                for (points, 0..) |*p, i| {
                    p.* = switch (i % 4) {
                        0 => self.unit().mulScalar(self.float(0.1, 2.5)).mul(s),
                        1, 3 => points[i - 1].add(self.unit().mulScalar(1.0e-3 * self.float(0, 2))),
                        else => blk: {
                            const fraction = self.float(0, 1);
                            break :blk points[i - 2].mulScalar(fraction).add(points[self.index(i)].mulScalar(1.0 - fraction));
                        },
                    };
                }
            },
            // Clusters
            9 => {
                var centers: [4]Vec3 = undefined;
                for (&centers) |*c| c.* = self.vec(-1, 1);
                const num_centers = 1 + self.index(centers.len);
                const spreads = [_]f32{ 1.0e-4, 1.0e-3, 0.01, 0.1 };
                const spread = spreads[self.index(spreads.len)];
                for (points) |*p| p.* = centers[self.index(num_centers)].add(self.vec(-spread, spread));
            },
            // Cylinder: two circles (many coplanar points on the caps)
            10 => {
                const segments = 3 + self.index(30);
                const height = self.float(0.01, 2);
                for (points) |*p| {
                    const angle = 2.0 * zolt.math.pi * @as(f32, @floatFromInt(self.index(segments))) / @as(f32, @floatFromInt(segments));
                    const sc = zolt.Vec4.replicate(angle).sinCos();
                    const r: f32 = if (self.oneIn(4)) self.float(0, 1) else 1;
                    p.* = Vec3.init(r * sc.cos.getX(), if (self.oneIn(2)) height else -height, r * sc.sin.getX());
                }
            },
            // Few distinct points, repeated
            11 => {
                var distinct: [5]Vec3 = undefined;
                for (&distinct) |*d| d.* = self.vec(-1, 1);
                const num_distinct = 1 + self.index(distinct.len);
                for (points) |*p| p.* = distinct[self.index(num_distinct)];
            },
            // Thin slab: a flat cloud with a few points slightly above it
            12 => {
                const heights = [_]f32{ 1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3, 0.01 };
                const height = heights[self.index(heights.len)];
                for (points) |*p| p.* = Vec3.init(self.float(-1, 1), if (self.oneIn(8)) height else 0, self.float(-1, 1));
            },
            // Octahedron / box corners with interior points and duplicates
            else => {
                const corners = [_]Vec3{ Vec3.init(1, 0, 0), Vec3.init(-1, 0, 0), Vec3.init(0, 1, 0), Vec3.init(0, -1, 0), Vec3.init(0, 0, 1), Vec3.init(0, 0, -1), Vec3.init(1, 1, 1), Vec3.init(-1, -1, -1) };
                for (points) |*p| p.* = if (self.oneIn(2)) corners[self.index(corners.len)] else self.vec(-0.3, 0.3);
            },
        }

        // Exact duplicates
        if (points.len > 0 and self.oneIn(5)) {
            for (0..1 + self.index(points.len)) |_| points[self.index(points.len)] = points[self.index(points.len)];
        }

        // Scale and translate
        if (self.oneIn(4)) {
            const s = self.scale();
            for (points) |*p| p.* = p.mulScalar(s);
        }
        if (self.oneIn(8)) {
            const distances = [_]f32{ 1, 10, 100, 1000 };
            const t = self.vec(-1, 1).mulScalar(distances[self.index(distances.len)]);
            for (points) |*p| p.* = p.add(t);
        }
    }

    fn params(self: *Gen, points: []const Vec3) Params {
        // Size of the cloud, to scale the tolerance
        var extent: f32 = 0;
        for (points) |p| extent = zolt.math.max(extent, p.abs().reduceMax());
        const tolerances = [_]f32{ 0, 1.0e-6, 1.0e-4, 1.0e-3, 1.0e-3, 1.0e-2, 0.1, 0.5 };
        var tolerance = tolerances[self.index(tolerances.len)];
        if (self.oneIn(2) and extent > 0) tolerance *= extent;
        const max_vertices: i32 = switch (self.next() % 10) {
            0 => @intCast(self.index(4)), // 0 .. 3
            1, 2, 3 => @intCast(4 + self.index(40)),
            else => no_limit,
        };
        return .{ .max_vertices = max_vertices, .tolerance = tolerance };
    }
};

/// The inputs of Jolt's ConvexHullBuilder unit tests plus hand picked shapes
fn handPicked(ctx: *Context) !void {
    const unlimited = [_]Params{ .{ .max_vertices = no_limit, .tolerance = 1.0e-3 }, .{ .max_vertices = 6, .tolerance = 1.0e-3 }, .{ .max_vertices = no_limit, .tolerance = 0 }, .{ .max_vertices = 4, .tolerance = 0.1 } };
    var points: [max_points]Vec3 = undefined;

    // Fewer than 3 points, coinciding points, a line
    try checkHull(ctx, 0, &.{}, &unlimited);
    try checkHull(ctx, 0, &.{Vec3.init(1, 2, 3)}, &unlimited);
    try checkHull(ctx, 0, &.{ Vec3.init(1, 2, 3), Vec3.init(1.0005, 2, 3) }, &unlimited);
    try checkHull(ctx, 0, &.{ Vec3.init(1, 2, 3), Vec3.init(1.0005, 2, 3), Vec3.init(1, 2.0005, 3) }, &unlimited);
    try checkHull(ctx, 0, &.{ Vec3.init(1, 2, 3), Vec3.init(1.0005, 2, 3), Vec3.init(1, 2.0005, 3), Vec3.init(1, 2, 3.0005) }, &unlimited);
    {
        var n: usize = 0;
        var v: f32 = 0.0;
        while (v < 1.01) : (v += 0.1) {
            points[n] = Vec3.init(v, 0, 0);
            n += 1;
        }
        try checkHull(ctx, 0, points[0..n], &unlimited);
    }

    // Triangles, tetrahedra, colinear and coplanar quads
    try checkHull(ctx, 0, &.{ Vec3.init(-1, 0, -1), Vec3.init(1, 0, -1), Vec3.init(-1, 0, 1) }, &unlimited);
    try checkHull(ctx, 0, &.{ Vec3.init(0, 0, 0), Vec3.init(1, 0, 0), Vec3.init(2, 0, 0) }, &unlimited);
    try checkHull(ctx, 0, &.{ Vec3.init(0, 0, 0), Vec3.init(1, 0, 0), Vec3.init(0, 1, 0), Vec3.init(0, 0, 1) }, &unlimited);
    try checkHull(ctx, 0, &.{ Vec3.init(0, 0, 0), Vec3.init(1, 0, 0), Vec3.init(0, 1, 0), Vec3.init(1, 1, 0) }, &unlimited);
    try checkHull(ctx, 0, &.{ Vec3.init(0, 0, 0), Vec3.init(1, 0, 0), Vec3.init(0, 1, 0), Vec3.init(0.25, 0.25, 0) }, &unlimited);

    // Unit cube corners, octahedron
    {
        var n: usize = 0;
        for (0..8) |i| {
            points[n] = Vec3.init(@floatFromInt(i & 1), @floatFromInt((i >> 1) & 1), @floatFromInt((i >> 2) & 1));
            n += 1;
        }
        try checkHull(ctx, 0, points[0..n], &unlimited);
        const octahedron = [_]Vec3{ Vec3.init(1, 0, 0), Vec3.init(-1, 0, 0), Vec3.init(0, 1, 0), Vec3.init(0, -1, 0), Vec3.init(0, 0, 1), Vec3.init(0, 0, -1) };
        try checkHull(ctx, 0, &octahedron, &unlimited);
    }

    // Test2DHull: a quad and a disc with many interior points
    {
        var n: usize = 0;
        for (0..10) |x| {
            for (0..10) |z| {
                const one: f32 = 1.0;
                points[n] = Vec3.init(0.1 * @as(f32, @floatFromInt(x)), 0, one * 0.2 * @as(f32, @floatFromInt(z)));
                n += 1;
            }
        }
        try checkHull(ctx, 0, points[0..n], &unlimited);

        n = 0;
        for (0..10) |r| {
            for (0..10) |phi| {
                const f_r = 2.0 * @as(f32, @floatFromInt(r));
                const f_phi = 2.0 * zolt.math.pi * @as(f32, @floatFromInt(phi)) / 10;
                points[n] = Vec3.init(f_r * zolt.trigonometry.cos(f_phi), f_r * zolt.trigonometry.sin(f_phi), 0);
                n += 1;
            }
        }
        try checkHull(ctx, 0, points[0..n], &unlimited);
    }

    // Test3DHull: a cube and a sphere with many interior points
    {
        var n: usize = 0;
        for (0..10) |x| {
            for (0..10) |y| {
                for (0..10) |z| {
                    const two: f32 = 2.0;
                    points[n] = Vec3.init(0.1 * @as(f32, @floatFromInt(x)), 1.0 + 0.2 * @as(f32, @floatFromInt(y)), two * 0.3 * @as(f32, @floatFromInt(z)));
                    n += 1;
                }
            }
        }
        try checkHull(ctx, 0, points[0..n], &unlimited);

        n = 0;
        for (0..10) |r| {
            for (0..10) |phi| {
                for (0..10) |theta| {
                    const f_r = 2.0 * @as(f32, @floatFromInt(r));
                    const f_phi = 2.0 * zolt.math.pi * @as(f32, @floatFromInt(phi)) / 10;
                    const f_theta = zolt.math.pi * @as(f32, @floatFromInt(theta)) / 9;
                    points[n] = Vec3.unitSpherical(f_theta, f_phi).mulScalar(f_r);
                    n += 1;
                }
            }
        }
        try checkHull(ctx, 0, points[0..n], &unlimited);
    }

    // TestHullEdgeCases: nearly coplanar faces
    const edge_case1 = [_]Vec3{
        Vec3.init(-0.020472288, -0.195635557, 0.308015466), Vec3.init(0.136248738, 0.633286834, 0.135366619),
        Vec3.init(0.286418647, -0.228475571, 0.308084548),  Vec3.init(-0.267285109, 1.024676085, 0.308042824),
        Vec3.init(0.396568149, -0.971658647, 0.308055162),  Vec3.init(0.321081549, -1.024676085, 0.308036327),
        Vec3.init(0.034643859, -0.404506862, 0.308015764),  Vec3.init(0.189224690, -0.252762139, 0.308060408),
    };
    try checkHull(ctx, 0, &edge_case1, &unlimited);
    const edge_case2 = [_]Vec3{
        Vec3.init(0.917345762, 0.157111734, 1.650970459),   Vec3.init(-0.098074198, 0.157116055, 0.664742708),
        Vec3.init(1.777100325, 0.157112047, 1.238879442),   Vec3.init(2.114324570, 0.157112464, 0.780688763),
        Vec3.init(1.926570415, 0.157114446, 0.240761161),   Vec3.init(-1.045998096, 0.157108605, 1.548911095),
        Vec3.init(-1.820045233, 0.157106474, 1.050360918),  Vec3.init(-1.918573976, 0.157108605, 0.039246202),
        Vec3.init(0.042619467, 0.157113969, -1.405336142),  Vec3.init(0.575986624, 0.157114401, -1.370834589),
        Vec3.init(1.402592659, 0.157115221, -0.834864557),  Vec3.init(1.110557318, 0.157113969, -1.336267948),
        Vec3.init(1.689781666, 0.157115355, -0.308773756),  Vec3.init(2.205337524, 0.157113209, -0.281754494),
        Vec3.init(-1.346967936, 0.157110974, -0.978962541), Vec3.init(-1.346967936, 0.157110974, -0.978962541),
        Vec3.init(-2.085033417, 0.157106936, -0.506602883), Vec3.init(-0.981224537, 0.157110706, -1.445893764),
        Vec3.init(-0.481085658, 0.157112658, -1.426232934), Vec3.init(-0.981224537, 0.157110706, -1.445893764),
    };
    try checkHull(ctx, 0, &edge_case2, &unlimited);
}

test "ConvexHullBuilder" {
    var summary: Checker = .{ .name = "ConvexHullBuilder" };
    var faces: Checker = .{ .name = "ConvexHullBuilder faces" };
    var contains_face: Checker = .{ .name = "ConvexHullBuilder.containsFace" };
    var stats: Stats = .{};
    var ctx: Context = .{ .summary = &summary, .faces = &faces, .contains_face = &contains_face, .stats = &stats };
    var gen: Gen = .{};

    try handPicked(&ctx);

    var points: [max_points]Vec3 = undefined;
    for (1..num_clouds + 1) |cloud| {
        const pos = points[0..gen.count()];
        gen.cloud(pos);

        // Initialize once, sometimes again with other parameters on the same builder
        var params: [3]Params = undefined;
        const num_params = if (gen.oneIn(4)) 2 + gen.index(2) else 1;
        for (params[0..num_params]) |*p| p.* = gen.params(pos);

        // Skip the clouds that make Jolt's (and Zolt's) asserts fail
        if (zolt.Core.enable_asserts and !dump_inputs and std.mem.indexOfScalar(usize, &assert_clouds, cloud) != null) {
            stats.skipped_clouds += 1;
            continue;
        }

        try checkHull(&ctx, cloud, pos, params[0..num_params]);
    }
    if (dump_inputs)
        return;

    // The inputs must reach every result code and the interesting paths
    // (Debug: 5566 hulls, results { 3889, 814, 129, 69, 665 }, 800 2D hulls, 98407 faces (max 1794), 49661 leftover
    // conflict points, 9 hulls with zero normals, 7 skipped clouds)
    var covered = stats.two_d > 0 and stats.leftover_conflicts > 0 and stats.max_faces > 1000 and stats.stale_coplanar > 0;
    for (stats.results) |r|
        covered = covered and r > 0;
    if (!covered) {
        std.debug.print("ConvexHullBuilder parity: the inputs don't cover everything: {d} hulls, results {any}, {d} 2D hulls, {d} faces (max {d}), {d} leftover conflict points, {d} hulls with zero normals, {d} skipped clouds, {d} stale coplanar lists\n", .{ stats.clouds, stats.results, stats.two_d, stats.faces, stats.max_faces, stats.leftover_conflicts, stats.zero_normals, stats.skipped_clouds, stats.stale_coplanar });
        return error.TestUnexpectedResult;
    }

    try fw.finishAll(&.{ &summary, &faces, &contains_face });
}
