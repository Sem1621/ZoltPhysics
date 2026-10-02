//! Parity tests for the Jolt/Geometry primitives (Triangle, IndexedTriangle, Plane, AABox, Sphere, OrientedBox,
//! AABox4, ClipPoly, MortonCode, Indexify, Ellipse): run Zolt and the C++ Jolt library on the same inputs and require
//! identical bits. C ABI wrappers: ZoltParity/Geometry/PrimitivesReference.cpp. See ZoltParity/parity.zig for how
//! parity tests work.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const AABox = zolt.AABox;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Ellipse = zolt.Ellipse;
const Float2 = zolt.Float2;
const Float3 = zolt.Float3;
const IndexedTriangle = zolt.IndexedTriangle;
const IndexedTriangleList = zolt.IndexedTriangleList;
const IndexedTriangleNoMaterial = zolt.IndexedTriangleNoMaterial;
const Mat44 = zolt.Mat44;
const MortonCode = zolt.MortonCode;
const OrientedBox = zolt.OrientedBox;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const Sphere = zolt.Sphere;
const StaticArray = zolt.StaticArray;
const Triangle = zolt.Triangle;
const TriangleList = zolt.TriangleList;
const UVec4 = zolt.UVec4;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const VertexArrayList = zolt.VertexArrayList;
const VertexList = zolt.VertexList;

/// The C++ reference functions, see PrimitivesReference.cpp
const jolt = struct {
    extern fn jolt_triangle_get_centroid(t: *const Triangle, out: *[3]f32) void;

    extern fn jolt_indexed_triangle_no_material_eq(a: *const IndexedTriangleNoMaterial, b: *const IndexedTriangleNoMaterial) bool;
    extern fn jolt_indexed_triangle_no_material_is_equivalent(a: *const IndexedTriangleNoMaterial, b: *const IndexedTriangleNoMaterial) bool;
    extern fn jolt_indexed_triangle_no_material_is_opposite(a: *const IndexedTriangleNoMaterial, b: *const IndexedTriangleNoMaterial) bool;
    extern fn jolt_indexed_triangle_no_material_is_degenerate(t: *const IndexedTriangleNoMaterial, vertices: [*]const Float3, num_vertices: u32) bool;
    extern fn jolt_indexed_triangle_no_material_rotate(t: *const IndexedTriangleNoMaterial, out: *IndexedTriangleNoMaterial) void;
    extern fn jolt_indexed_triangle_no_material_get_centroid(t: *const IndexedTriangleNoMaterial, vertices: [*]const Float3, num_vertices: u32, out: *[3]f32) void;
    extern fn jolt_indexed_triangle_no_material_get_hash(t: *const IndexedTriangleNoMaterial) u64;
    extern fn jolt_indexed_triangle_no_material_hash(t: *const IndexedTriangleNoMaterial) u64;
    extern fn jolt_indexed_triangle_no_material_std_hash(t: *const IndexedTriangleNoMaterial) u64;
    extern fn jolt_indexed_triangle_eq(a: *const IndexedTriangle, b: *const IndexedTriangle) bool;
    extern fn jolt_indexed_triangle_is_equivalent(a: *const IndexedTriangle, b: *const IndexedTriangle) bool;
    extern fn jolt_indexed_triangle_is_opposite(a: *const IndexedTriangle, b: *const IndexedTriangle) bool;
    extern fn jolt_indexed_triangle_is_degenerate(t: *const IndexedTriangle, vertices: [*]const Float3, num_vertices: u32) bool;
    extern fn jolt_indexed_triangle_rotate(t: *const IndexedTriangle, out: *IndexedTriangle) void;
    extern fn jolt_indexed_triangle_get_centroid(t: *const IndexedTriangle, vertices: [*]const Float3, num_vertices: u32, out: *[3]f32) void;
    extern fn jolt_indexed_triangle_get_lowest_index_first(t: *const IndexedTriangle, out: *IndexedTriangle) void;
    extern fn jolt_indexed_triangle_get_hash(t: *const IndexedTriangle) u64;
    extern fn jolt_indexed_triangle_hash(t: *const IndexedTriangle) u64;
    extern fn jolt_indexed_triangle_std_hash(t: *const IndexedTriangle) u64;

    extern fn jolt_plane_init(normal: *const [3]f32, constant: f32, out: *[4]f32) void;
    extern fn jolt_plane_from_point_and_normal(point: *const [3]f32, normal: *const [3]f32, out: *[4]f32) void;
    extern fn jolt_plane_from_point_and_normal_dvec3(point: *const [3]f64, normal: *const [3]f32, out: *[4]f32) void;
    extern fn jolt_plane_from_points_ccw(v1: *const [3]f32, v2: *const [3]f32, v3: *const [3]f32, out: *[4]f32) void;
    extern fn jolt_plane_set_normal_constant(p: *const [4]f32, normal: *const [3]f32, constant: f32, out_normal: *[3]f32, out_constant: *f32, out: *[4]f32) void;
    extern fn jolt_plane_offset(p: *const [4]f32, distance: f32, out: *[4]f32) void;
    extern fn jolt_plane_get_transformed(p: *const [4]f32, m: *const [16]f32, out: *[4]f32) void;
    extern fn jolt_plane_scaled(p: *const [4]f32, scale: *const [3]f32, out: *[4]f32) void;
    extern fn jolt_plane_signed_distance(p: *const [4]f32, point: *const [3]f32) f32;
    extern fn jolt_plane_project_point_on_plane(p: *const [4]f32, point: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_plane_intersect_planes(p1: *const [4]f32, p2: *const [4]f32, p3: *const [4]f32, out: *[3]f32) bool;

    extern fn jolt_aabox_constants(out_empty: *[6]f32, out_biggest: *[6]f32) void;
    extern fn jolt_aabox_from_dvec3(min: *const [3]f64, max: *const [3]f64, out: *[6]f32) void;
    extern fn jolt_aabox_from_center_and_radius(center: *const [3]f32, radius: f32, out: *[6]f32) void;
    extern fn jolt_aabox_from_two_points(p1: *const [3]f32, p2: *const [3]f32, out: *[6]f32) void;
    extern fn jolt_aabox_from_triangle(vertices: [*]const Float3, num_vertices: u32, t: *const IndexedTriangle, out: *[6]f32) void;
    extern fn jolt_aabox_compare(a: *const [6]f32, b: *const [6]f32, out_equal: *bool, out_not_equal: *bool, out_is_valid: *bool) void;
    extern fn jolt_aabox_encapsulate(a: *const [6]f32, b: *const [6]f32, point: *const [3]f32, triangle: *const Triangle, vertices: [*]const Float3, num_vertices: u32, indexed_triangle: *const IndexedTriangle, out_vec3: *[6]f32, out_aabox: *[6]f32, out_triangle: *[6]f32, out_indexed_triangle: *[6]f32) void;
    extern fn jolt_aabox_intersect(a: *const [6]f32, b: *const [6]f32, out: *[6]f32) void;
    extern fn jolt_aabox_ensure_minimal_edge_length(a: *const [6]f32, length: f32, out: *[6]f32) void;
    extern fn jolt_aabox_expand_by(a: *const [6]f32, v: *const [3]f32, out: *[6]f32) void;
    extern fn jolt_aabox_properties(a: *const [6]f32, out_center: *[3]f32, out_extent: *[3]f32, out_size: *[3]f32, out_surface_area: *f32, out_volume: *f32) void;
    extern fn jolt_aabox_contains_overlaps(a: *const [6]f32, b: *const [6]f32, point: *const [3]f32, dpoint: *const [3]f64, plane: *const [4]f32, out_contains: *bool, out_contains_vec3: *bool, out_contains_dvec3: *bool, out_overlaps: *bool, out_overlaps_plane: *bool) void;
    extern fn jolt_aabox_translate(a: *const [6]f32, translation: *const [3]f32, out: *[6]f32) void;
    extern fn jolt_aabox_translate_dvec3(a: *const [6]f32, translation: *const [3]f64, out: *[6]f32) void;
    extern fn jolt_aabox_transformed(a: *const [6]f32, m: *const [16]f32, out: *[6]f32) void;
    extern fn jolt_aabox_transformed_dmat44(a: *const [6]f32, cols: *const [12]f32, t: *const [3]f64, out: *[6]f32) void;
    extern fn jolt_aabox_scaled(a: *const [6]f32, scale: *const [3]f32, out: *[6]f32) void;
    extern fn jolt_aabox_get_support(a: *const [6]f32, direction: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_aabox_get_supporting_face(a: *const [6]f32, direction: *const [3]f32, out: *[12]f32) void;
    extern fn jolt_aabox_get_closest_point(a: *const [6]f32, point: *const [3]f32, out: *[3]f32, out_sq_distance: *f32) void;

    extern fn jolt_sphere_get_support(s: *const [4]f32, direction: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_sphere_overlaps(a: *const [4]f32, b: *const [4]f32) bool;
    extern fn jolt_sphere_overlaps_aabox(s: *const [4]f32, b: *const [6]f32) bool;
    extern fn jolt_sphere_encapsulate_point(s: *const [4]f32, point: *const [3]f32, out: *[4]f32) void;

    extern fn jolt_oriented_box_from_aabox(m: *const [16]f32, b: *const [6]f32, out_m: *[16]f32, out_half_extents: *[3]f32) void;
    extern fn jolt_oriented_box_overlaps_aabox(m: *const [16]f32, half_extents: *const [3]f32, b: *const [6]f32, epsilon: f32) bool;
    extern fn jolt_oriented_box_overlaps(m1: *const [16]f32, half_extents1: *const [3]f32, m2: *const [16]f32, half_extents2: *const [3]f32, epsilon: f32) bool;

    extern fn jolt_aabox4_vs_box(b: *const [6]f32, bounds: *const [24]f32, out: *[4]u32) void;
    extern fn jolt_aabox4_scale(scale: *const [3]f32, bounds: *const [24]f32, out: *[24]f32) void;
    extern fn jolt_aabox4_enlarge_with_extent(extent: *const [3]f32, bounds: *const [24]f32, out: *[24]f32) void;
    extern fn jolt_aabox4_vs_point(point: *const [3]f32, bounds: *const [24]f32, out: *[4]u32) void;
    extern fn jolt_aabox4_vs_oriented_box_mat44(m: *const [16]f32, half_extents: *const [3]f32, bounds: *const [24]f32, epsilon: f32, out: *[4]u32) void;
    extern fn jolt_aabox4_vs_oriented_box(m: *const [16]f32, half_extents: *const [3]f32, bounds: *const [24]f32, epsilon: f32, out: *[4]u32) void;
    extern fn jolt_aabox4_distance_sq_to_point(point: *const [12]f32, bounds: *const [24]f32, out: *[4]f32) void;
    extern fn jolt_aabox4_distance_sq_to_point_vec3(point: *const [3]f32, bounds: *const [24]f32, out: *[4]f32) void;
    extern fn jolt_aabox4_vs_sphere(center: *const [12]f32, radius_sq: *const [4]f32, bounds: *const [24]f32, out: *[4]u32) void;
    extern fn jolt_aabox4_vs_sphere_vec3(center: *const [3]f32, radius_sq: f32, bounds: *const [24]f32, out: *[4]u32) void;

    extern fn jolt_clip_poly_vs_plane(polygon: [*]const [3]f32, num_vertices: u32, plane_origin: *const [3]f32, plane_normal: *const [3]f32, out: [*][3]f32, max_vertices: u32) u32;
    extern fn jolt_clip_poly_vs_poly(polygon: [*]const [3]f32, num_vertices: u32, clipping_polygon: [*]const [3]f32, num_clipping_vertices: u32, clipping_polygon_normal: *const [3]f32, out: [*][3]f32, max_vertices: u32) u32;
    extern fn jolt_clip_poly_vs_edge(polygon: [*]const [3]f32, num_vertices: u32, edge_vertex1: *const [3]f32, edge_vertex2: *const [3]f32, clipping_edge_normal: *const [3]f32, out: [*][3]f32, max_vertices: u32) u32;
    extern fn jolt_clip_poly_vs_aabox(polygon: [*]const [3]f32, num_vertices: u32, box: *const [6]f32, out: [*][3]f32, max_vertices: u32) u32;

    extern fn jolt_morton_code_expand_bits(v: f32) u32;
    extern fn jolt_morton_code_get_morton_code(vector: *const [3]f32, bounds: *const [6]f32) u32;

    extern fn jolt_indexify(triangles: [*]const Triangle, num_triangles: u32, vertex_weld_distance: f32, out_vertices: [*]Float3, out_num_vertices: *u32, out_triangles: [*]IndexedTriangle, out_num_triangles: *u32) void;
    extern fn jolt_deindexify(vertices: [*]const Float3, num_vertices: u32, triangles: [*]const IndexedTriangle, num_triangles: u32, out_triangles: [*]Triangle) void;

    extern fn jolt_ellipse_is_inside(a: f32, b: f32, point: *const [2]f32) bool;
    extern fn jolt_ellipse_get_closest_point(a: f32, b: f32, point: *const [2]f32, out: *[2]f32) void;
    extern fn jolt_ellipse_get_normal(a: f32, b: f32, point: *const [2]f32, out: *[2]f32) void;
};

/// Number of random inputs per test
const iterations = fw.iterations;

const flt_max = zolt.math.flt_max;
const inf = std.math.inf(f32);

/// Values that tend to expose differences, mixed into the random inputs
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, 1.0e-30, -1.0e-30, 1.0e-6, -1.0e-6, 100.0, -100.0 };

/// Huge, tiny, denormal and infinite values, for the functions that only compare, select, add or multiply
const extreme_values = [_]f32{ flt_max, -flt_max, 0.5 * flt_max, -0.5 * flt_max, 1.0e30, -1.0e30, 1.0e-38, -1.0e-38, 1.0e-45, -1.0e-45, inf, -inf };

/// Deterministic input generator (xorshift32)
const Rng = struct {
    state: u32 = 0x12345678,

    fn next(self: *Rng) u32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        return self.state;
    }

    /// True one out of n times
    fn chance(self: *Rng, n: u32) bool {
        return self.next() % n == 0;
    }

    /// Random integer in [0, n)
    fn index(self: *Rng, n: usize) usize {
        return self.next() % n;
    }

    /// Random float in [min, max), or one of the special values (10% of the time)
    fn float(self: *Rng, min: f32, max: f32) f32 {
        if (self.chance(10)) {
            const v = special_values[self.index(special_values.len)];
            if (v >= min and v <= max) return v;
        }
        const t = @as(f32, @floatFromInt(self.next() >> 8)) / 16777216.0;
        return min + (max - min) * t;
    }

    fn array(self: *Rng, comptime n: usize, min: f32, max: f32) [n]f32 {
        var result: [n]f32 = undefined;
        for (&result) |*v| v.* = self.float(min, max);
        return result;
    }

    /// One of the extreme values
    fn extreme(self: *Rng) f32 {
        return extreme_values[self.index(extreme_values.len)];
    }

    /// Random float in [min, max), sometimes an extreme value
    fn floatOrExtreme(self: *Rng, min: f32, max: f32) f32 {
        return if (self.chance(20)) self.extreme() else self.float(min, max);
    }

    fn arrayOrExtreme(self: *Rng, comptime n: usize, min: f32, max: f32) [n]f32 {
        var result: [n]f32 = undefined;
        for (&result) |*v| v.* = self.floatOrExtreme(min, max);
        return result;
    }

    /// Random float in [1, 2) times a random power of 2 in [2^-20, 2^20] with a random sign
    fn anyMagnitude(self: *Rng) f32 {
        const sign: f32 = if (self.chance(2)) 1 else -1;
        const exponent: i32 = @as(i32, @intCast(self.next() % 41)) - 20;
        return sign * std.math.ldexp(self.float(1, 2), exponent);
    }

    /// Random double in [min, max) with a full mantissa
    fn double(self: *Rng, min: f64, max: f64) f64 {
        const bits = (@as(u64, self.next()) << 21) ^ @as(u64, self.next() >> 11);
        const t = @as(f64, @floatFromInt(bits & ((1 << 53) - 1))) / 9007199254740992.0;
        return min + (max - min) * t;
    }

    fn doubleArray(self: *Rng, comptime n: usize, min: f64, max: f64) [n]f64 {
        var result: [n]f64 = undefined;
        for (&result) |*v| v.* = self.double(min, max);
        return result;
    }

    /// Random vector that is not too close to zero
    fn nonZero(self: *Rng, comptime n: usize, min: f32, max: f32) [n]f32 {
        while (true) {
            const v = self.array(n, min, max);
            var len_sq: f32 = 0;
            for (v) |c| len_sq += c * c;
            if (len_sq > 1.0e-6) return v;
        }
    }

    fn unitVector(self: *Rng, comptime n: usize) [n]f32 {
        return normalize(n, self.nonZero(n, -1, 1));
    }

    /// Random unit quaternion, sometimes an exact axis aligned rotation
    fn unitQuat(self: *Rng) Quat {
        if (self.chance(8)) {
            const axis_quats = [_][4]f32{ .{ 0, 0, 0, 1 }, .{ 1, 0, 0, 0 }, .{ 0, 1, 0, 0 }, .{ 0, 0, 1, 0 }, .{ 0.70710677, 0, 0, 0.70710677 }, .{ 0, 0.70710677, 0, 0.70710677 }, .{ 0, 0, 0.70710677, -0.70710677 }, .{ 0.5, 0.5, 0.5, 0.5 } };
            const q = axis_quats[self.index(axis_quats.len)];
            return Quat.init(q[0], q[1], q[2], q[3]);
        }
        const q = self.unitVector(4);
        return Quat.init(q[0], q[1], q[2], q[3]);
    }

    /// Rotation + translation matrix, sometimes with an exact axis permutation as rotation
    fn rotationTranslation(self: *Rng, range: f32) Mat44 {
        var m = if (self.chance(6)) self.axisPermutation() else Mat44.rotationQuat(self.unitQuat());
        m.setTranslation(vec3(self.array(3, -range, range)));
        return m;
    }

    /// Rotation matrix that maps axes to (negated) axes, all elements are exactly 0, 1 or -1
    fn axisPermutation(self: *Rng) Mat44 {
        const permutations = [_][3]u32{ .{ 0, 1, 2 }, .{ 0, 2, 1 }, .{ 1, 0, 2 }, .{ 1, 2, 0 }, .{ 2, 0, 1 }, .{ 2, 1, 0 } };
        const p = permutations[self.index(permutations.len)];
        var m = Mat44.identity();
        for (0..3) |c| {
            var col = Vec3.zero();
            col.setComponent(p[c], if (self.chance(2)) 1 else -1);
            m.col[c] = Vec4.fromVec3W(col, 0);
        }
        return m;
    }

    /// Any matrix: rotation + translation, rotation * scale + translation (scale may be negative), random 3x3 + translation or fully random
    fn matrix(self: *Rng) Mat44 {
        switch (self.next() % 4) {
            0 => return self.rotationTranslation(100),
            1 => {
                const m = self.rotationTranslation(100);
                return m.preScaled(vec3(self.array(3, -10, 10)));
            },
            2 => {
                const a = self.array(16, -10, 10);
                var m = mat44(a);
                m.col[0].setW(0);
                m.col[1].setW(0);
                m.col[2].setW(0);
                m.col[3].setW(1);
                return m;
            },
            else => return mat44(self.array(16, -10, 10)),
        }
    }

    /// Valid random box
    fn validBox(self: *Rng, range: f32) [6]f32 {
        const a = vec3(self.array(3, -range, range));
        const b = vec3(self.array(3, -range, range));
        return arrBox(AABox.init(Vec3.min(a, b), Vec3.max(a, b)));
    }

    /// Random box: valid, empty, biggest, a point, flat, invalid (min > max) or with extreme values
    fn box(self: *Rng) [6]f32 {
        switch (self.next() % 12) {
            0 => return arrBox(AABox.empty),
            1 => return arrBox(AABox.biggest()),
            2 => {
                const p = self.array(3, -100, 100);
                return p ++ p;
            },
            3 => {
                var b = self.validBox(100);
                const axis = self.index(3);
                b[3 + axis] = b[axis];
                return b;
            },
            4 => return self.array(6, -100, 100),
            5 => {
                var b = self.validBox(100);
                b[self.index(6)] = self.extreme();
                return b;
            },
            else => return self.validBox(100),
        }
    }

    /// Box related to `a`: random, equal, touching a face or a corner, inside, slightly separated or overlapping
    fn relatedBox(self: *Rng, a: [6]f32) [6]f32 {
        switch (self.next() % 8) {
            0 => return a,
            1 => {
                // Touching: share a face
                var b = self.validBox(100);
                const axis = self.index(3);
                const size = b[3 + axis] - b[axis];
                if (self.chance(2)) {
                    b[axis] = a[3 + axis];
                    b[3 + axis] = a[3 + axis] + size;
                } else {
                    b[3 + axis] = a[axis];
                    b[axis] = a[axis] - size;
                }
                return b;
            },
            2 => {
                // Slightly separated or overlapping along an axis
                var b = a;
                const axis = self.index(3);
                const dir: f32 = if (self.chance(2)) 1 else -1;
                const offset = (a[3 + axis] - a[axis]) * dir;
                b[axis] = nextFloat(a[axis] + offset, @intCast(self.next() % 5), self.chance(2));
                b[3 + axis] = b[axis] + (a[3 + axis] - a[axis]);
                return b;
            },
            3 => {
                // Inside
                const ab = aabox(a);
                const p1 = ab.getClosestPoint(vec3(self.array(3, -100, 100)));
                const p2 = ab.getClosestPoint(vec3(self.array(3, -100, 100)));
                return arrBox(AABox.fromTwoPoints(p1, p2));
            },
            else => return self.box(),
        }
    }

    /// Point related to box a: random, on a face, a corner, inside, or with extreme values
    fn relatedPoint(self: *Rng, a: [6]f32) [3]f32 {
        switch (self.next() % 6) {
            0 => {
                var p = self.array(3, -100, 100);
                const axis = self.index(3);
                p[axis] = a[axis + 3 * self.index(2)];
                return p;
            },
            1 => return .{ a[3 * self.index(2)], a[1 + 3 * self.index(2)], a[2 + 3 * self.index(2)] },
            2 => return arr3(aabox(a).getClosestPoint(vec3(self.array(3, -100, 100)))),
            3 => return self.arrayOrExtreme(3, -100, 100),
            else => return self.array(3, -150, 150),
        }
    }

    /// Random direction, sometimes zero, axis aligned, tiny or with ties between the components
    fn direction(self: *Rng) [3]f32 {
        return switch (self.next() % 10) {
            0 => .{ 0, 0, 0 },
            1 => .{ -0.0, 0, -0.0 },
            2 => blk: {
                var d: [3]f32 = .{ 0, 0, 0 };
                d[self.index(3)] = if (self.chance(2)) 1 else -1;
                break :blk d;
            },
            3 => blk: {
                // Ties between components (equal absolute values)
                const s = self.float(-10, 10);
                break :blk .{ s, if (self.chance(2)) s else -s, if (self.chance(2)) s else self.float(-10, 10) };
            },
            4 => .{ self.float(-1.0e-25, 1.0e-25), self.float(-1.0e-25, 1.0e-25), self.float(-1.0e-25, 1.0e-25) },
            else => self.array(3, -10, 10),
        };
    }

    /// 4 boxes in SOA layout (min x, min y, min z, max x, max y, max z), each a random `box()` or related to `related`
    fn bounds(self: *Rng, related: ?[6]f32) [24]f32 {
        var result: [24]f32 = undefined;
        for (0..4) |i| {
            const b = if (related) |r| self.relatedBox(r) else self.box();
            for (0..6) |c| result[4 * c + i] = b[c];
        }
        return result;
    }
};

/// Move v by `ulps` floats up or down
fn nextFloat(v: f32, ulps: u32, up: bool) f32 {
    var result = v;
    for (0..ulps) |_| result = std.math.nextAfter(f32, result, if (up) inf else -inf);
    return result;
}

fn normalize(comptime n: usize, v: [n]f32) [n]f32 {
    var len_sq: f32 = 0;
    for (v) |c| len_sq += c * c;
    var result: [n]f32 = undefined;
    for (&result, v) |*r, c| r.* = c / @sqrt(len_sq);
    return result;
}

fn vec3(a: [3]f32) Vec3 {
    return Vec3.init(a[0], a[1], a[2]);
}

fn arr3(v: Vec3) [3]f32 {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn vec4(a: [4]f32) Vec4 {
    return Vec4.init(a[0], a[1], a[2], a[3]);
}

fn arr4(v: Vec4) [4]f32 {
    return v.value;
}

fn arrU4(v: UVec4) [4]u32 {
    return v.value;
}

fn mat44(a: [16]f32) Mat44 {
    return Mat44.init(vec4(a[0..4].*), vec4(a[4..8].*), vec4(a[8..12].*), vec4(a[12..16].*));
}

fn arr16(m: Mat44) [16]f32 {
    return arr4(m.col[0]) ++ arr4(m.col[1]) ++ arr4(m.col[2]) ++ arr4(m.col[3]);
}

fn dvec3(a: [3]f64) DVec3 {
    return DVec3.init(a[0], a[1], a[2]);
}

fn aabox(a: [6]f32) AABox {
    return AABox.init(vec3(a[0..3].*), vec3(a[3..6].*));
}

fn arrBox(b: AABox) [6]f32 {
    return arr3(b.min) ++ arr3(b.max);
}

fn plane(a: [4]f32) Plane {
    return Plane.fromVec4(vec4(a));
}

fn arrPlane(p: Plane) [4]f32 {
    return arr4(p.normal_and_constant);
}

fn sphere(a: [4]f32) Sphere {
    return Sphere.init(vec3(a[0..3].*), a[3]);
}

fn arrSphere(s: Sphere) [4]f32 {
    return arr3(s.getCenter()) ++ [1]f32{s.getRadius()};
}

/// 4 boxes in SOA layout
const Bounds = struct {
    min_x: Vec4,
    min_y: Vec4,
    min_z: Vec4,
    max_x: Vec4,
    max_y: Vec4,
    max_z: Vec4,

    fn load(a: [24]f32) Bounds {
        return .{ .min_x = vec4(a[0..4].*), .min_y = vec4(a[4..8].*), .min_z = vec4(a[8..12].*), .max_x = vec4(a[12..16].*), .max_y = vec4(a[16..20].*), .max_z = vec4(a[20..24].*) };
    }

    fn store(self: Bounds) [24]f32 {
        return arr4(self.min_x) ++ arr4(self.min_y) ++ arr4(self.min_z) ++ arr4(self.max_x) ++ arr4(self.max_y) ++ arr4(self.max_z);
    }
};

/// A random list of vertices with duplicates and collinear points, for the IndexedTriangle helpers
fn randomVertices(rng: *Rng, out: []Float3) void {
    for (out, 0..) |*v, i| {
        v.* = switch (rng.next() % 6) {
            0 => if (i > 0) out[rng.index(i)] else .init(0, 0, 0), // Duplicate
            1 => if (i > 1) blk: {
                // On the line through two earlier vertices
                const a = Vec3.fromFloat3(out[rng.index(i)]);
                const b = Vec3.fromFloat3(out[rng.index(i)]);
                var f: Float3 = undefined;
                a.add(b.sub(a).mulScalar(rng.float(-2, 2))).storeFloat3(&f);
                break :blk f;
            } else .init(1, 0, 0),
            2 => blk: {
                // Close to an earlier vertex
                if (i == 0) break :blk .init(0, 1, 0);
                const a = out[rng.index(i)];
                break :blk .init(a.x + rng.float(-1.0e-6, 1.0e-6), a.y, a.z + rng.float(-1.0e-6, 1.0e-6));
            },
            else => blk: {
                const a = rng.array(3, -10, 10);
                break :blk .init(a[0], a[1], a[2]);
            },
        };
    }
}

test "Triangle / IndexedTriangle" {
    var rng: Rng = .{};
    var triangle_centroid: Checker = .{ .name = "Triangle.getCentroid" };
    var nm_eq: Checker = .{ .name = "IndexedTriangleNoMaterial.eql" };
    var nm_equivalent: Checker = .{ .name = "IndexedTriangleNoMaterial.isEquivalent" };
    var nm_opposite: Checker = .{ .name = "IndexedTriangleNoMaterial.isOpposite" };
    var nm_degenerate: Checker = .{ .name = "IndexedTriangleNoMaterial.isDegenerate" };
    var nm_rotate: Checker = .{ .name = "IndexedTriangleNoMaterial.rotate" };
    var nm_centroid: Checker = .{ .name = "IndexedTriangleNoMaterial.getCentroid" };
    var nm_hash: Checker = .{ .name = "IndexedTriangleNoMaterial.getHash / Hash<> / std::hash" };
    var it_eq: Checker = .{ .name = "IndexedTriangle.eql" };
    var it_equivalent: Checker = .{ .name = "IndexedTriangle.isEquivalent" };
    var it_opposite: Checker = .{ .name = "IndexedTriangle.isOpposite" };
    var it_degenerate: Checker = .{ .name = "IndexedTriangle.isDegenerate" };
    var it_rotate: Checker = .{ .name = "IndexedTriangle.rotate" };
    var it_centroid: Checker = .{ .name = "IndexedTriangle.getCentroid" };
    var it_lowest: Checker = .{ .name = "IndexedTriangle.getLowestIndexFirst" };
    var it_hash: Checker = .{ .name = "IndexedTriangle.getHash / Hash<> / std::hash" };
    var vertices: [6]Float3 = undefined;
    for (0..iterations) |_| {
        var expected3: [3]f32 = undefined;

        // Triangle
        const tv = [3][3]f32{ rng.arrayOrExtreme(3, -100, 100), rng.arrayOrExtreme(3, -100, 100), rng.arrayOrExtreme(3, -100, 100) };
        const tri = Triangle.init(vec3(tv[0]), vec3(tv[1]), vec3(tv[2]), .{ .material_index = rng.next(), .user_data = rng.next() });
        jolt.jolt_triangle_get_centroid(&tri, &expected3);
        triangle_centroid.check(tv, arr3(tri.getCentroid()), expected3);

        // Indices from a small range so that equal, equivalent and opposite triangles are common
        const small = rng.chance(4) == false;
        const max_index: u32 = if (small) 3 else 0xffffffff;
        var idx: [6]u32 = undefined;
        for (&idx) |*i| i.* = if (small) rng.next() % max_index else rng.next();
        if (rng.chance(4)) {
            // Make b a rotation or reversal of a
            const r = rng.index(3);
            const reverse = rng.chance(2);
            for (0..3) |i| idx[3 + i] = idx[if (reverse) (r + 3 - i) % 3 else (r + i) % 3];
        }
        const a = IndexedTriangleNoMaterial.init(idx[0], idx[1], idx[2]);
        const b = IndexedTriangleNoMaterial.init(idx[3], idx[4], idx[5]);
        nm_eq.check(.{ a, b }, a.eql(b), jolt.jolt_indexed_triangle_no_material_eq(&a, &b));
        nm_equivalent.check(.{ a, b }, a.isEquivalent(b), jolt.jolt_indexed_triangle_no_material_is_equivalent(&a, &b));
        nm_opposite.check(.{ a, b }, a.isOpposite(b), jolt.jolt_indexed_triangle_no_material_is_opposite(&a, &b));
        var rotated_nm = a;
        rotated_nm.rotate();
        var expected_nm: IndexedTriangleNoMaterial = undefined;
        jolt.jolt_indexed_triangle_no_material_rotate(&a, &expected_nm);
        nm_rotate.check(a, rotated_nm, expected_nm);
        const nm_hashes = [3]u64{ a.getHash(), zolt.HashCombine.hash(a), zolt.HashCombine.hash(a) };
        nm_hash.check(a, nm_hashes, .{ jolt.jolt_indexed_triangle_no_material_get_hash(&a), jolt.jolt_indexed_triangle_no_material_hash(&a), jolt.jolt_indexed_triangle_no_material_std_hash(&a) });

        const ta = IndexedTriangle.init(idx[0], idx[1], idx[2], .{ .material_index = if (small) rng.next() % 2 else rng.next(), .user_data = if (small) rng.next() % 2 else rng.next() });
        const tb = if (rng.chance(8)) ta else IndexedTriangle.init(idx[3], idx[4], idx[5], .{ .material_index = if (small) rng.next() % 2 else rng.next(), .user_data = if (small) rng.next() % 2 else rng.next() });
        it_eq.check(.{ ta, tb }, ta.eql(tb), jolt.jolt_indexed_triangle_eq(&ta, &tb));
        it_equivalent.check(.{ ta, tb }, ta.isEquivalent(tb.toNoMaterial()), jolt.jolt_indexed_triangle_is_equivalent(&ta, &tb));
        it_opposite.check(.{ ta, tb }, ta.isOpposite(tb.toNoMaterial()), jolt.jolt_indexed_triangle_is_opposite(&ta, &tb));
        var rotated = ta;
        rotated.rotate();
        var expected_it: IndexedTriangle = undefined;
        jolt.jolt_indexed_triangle_rotate(&ta, &expected_it);
        it_rotate.check(ta, rotated, expected_it);
        jolt.jolt_indexed_triangle_get_lowest_index_first(&ta, &expected_it);
        it_lowest.check(ta, ta.getLowestIndexFirst(), expected_it);
        const it_hashes = [3]u64{ ta.getHash(), zolt.HashCombine.hash(ta), zolt.HashCombine.hash(ta) };
        it_hash.check(ta, it_hashes, .{ jolt.jolt_indexed_triangle_get_hash(&ta), jolt.jolt_indexed_triangle_hash(&ta), jolt.jolt_indexed_triangle_std_hash(&ta) });

        // Vertex based helpers
        randomVertices(&rng, &vertices);
        const n: u32 = vertices.len;
        const va = IndexedTriangleNoMaterial.init(rng.next() % n, rng.next() % n, rng.next() % n);
        nm_degenerate.check(.{ va, vertices }, va.isDegenerate(&vertices), jolt.jolt_indexed_triangle_no_material_is_degenerate(&va, &vertices, n));
        jolt.jolt_indexed_triangle_no_material_get_centroid(&va, &vertices, n, &expected3);
        nm_centroid.check(.{ va, vertices }, arr3(va.getCentroid(&vertices)), expected3);
        const vt = IndexedTriangle.init(va.idx[0], va.idx[1], va.idx[2], .{ .material_index = rng.next() });
        it_degenerate.check(.{ vt, vertices }, vt.isDegenerate(&vertices), jolt.jolt_indexed_triangle_is_degenerate(&vt, &vertices, n));
        jolt.jolt_indexed_triangle_get_centroid(&vt, &vertices, n, &expected3);
        it_centroid.check(.{ vt, vertices }, arr3(vt.getCentroid(&vertices)), expected3);
    }
    try finishAll(&.{ &triangle_centroid, &nm_eq, &nm_equivalent, &nm_opposite, &nm_degenerate, &nm_rotate, &nm_centroid, &nm_hash, &it_eq, &it_equivalent, &it_opposite, &it_degenerate, &it_rotate, &it_centroid, &it_lowest, &it_hash });
}

test "Plane" {
    var rng: Rng = .{};
    var init_checker: Checker = .{ .name = "Plane.init" };
    var from_point_and_normal: Checker = .{ .name = "Plane.fromPointAndNormal" };
    var from_point_and_normal_dvec3: Checker = .{ .name = "Plane.fromPointAndNormalDVec3" };
    var from_points_ccw: Checker = .{ .name = "Plane.fromPointsCCW" };
    var properties: Checker = .{ .name = "Plane.getNormal / getConstant / setNormal / setConstant" };
    var offset: Checker = .{ .name = "Plane.offset" };
    var get_transformed: Checker = .{ .name = "Plane.getTransformed" };
    var scaled: Checker = .{ .name = "Plane.scaled" };
    var signed_distance: Checker = .{ .name = "Plane.signedDistance" };
    var project_point: Checker = .{ .name = "Plane.projectPointOnPlane" };
    var intersect_planes: Checker = .{ .name = "Plane.intersectPlanes" };
    for (0..iterations) |_| {
        var expected4: [4]f32 = undefined;
        var expected3: [3]f32 = undefined;

        const normal = if (rng.chance(10)) rng.array(3, -10, 10) else rng.unitVector(3);
        const constant = rng.float(-100, 100);
        jolt.jolt_plane_init(&normal, constant, &expected4);
        init_checker.check(.{ normal, constant }, arrPlane(Plane.init(vec3(normal), constant)), expected4);

        const point = rng.array(3, -100, 100);
        jolt.jolt_plane_from_point_and_normal(&point, &normal, &expected4);
        from_point_and_normal.check(.{ point, normal }, arrPlane(Plane.fromPointAndNormal(vec3(point), vec3(normal))), expected4);

        const range: f64 = if (rng.chance(2)) 100.0 else 1.0e7;
        const dpoint = rng.doubleArray(3, -range, range);
        jolt.jolt_plane_from_point_and_normal_dvec3(&dpoint, &normal, &expected4);
        from_point_and_normal_dvec3.check(.{ dpoint, normal }, arrPlane(Plane.fromPointAndNormalDVec3(dvec3(dpoint), vec3(normal))), expected4);

        const v1 = rng.array(3, -100, 100);
        const v2 = rng.array(3, -100, 100);
        const v3 = rng.array(3, -100, 100);
        if (vec3(v2).sub(vec3(v1)).cross(vec3(v3).sub(vec3(v1))).lengthSq() > 1.0e-6) {
            jolt.jolt_plane_from_points_ccw(&v1, &v2, &v3, &expected4);
            from_points_ccw.check(.{ v1, v2, v3 }, arrPlane(Plane.fromPointsCCW(vec3(v1), vec3(v2), vec3(v3))), expected4);
        }

        const p = rng.array(4, -10, 10);
        var expected_normal: [3]f32 = undefined;
        var expected_constant: f32 = undefined;
        jolt.jolt_plane_set_normal_constant(&p, &normal, constant, &expected_normal, &expected_constant, &expected4);
        var modified = plane(p);
        const before = .{ arr3(modified.getNormal()), modified.getConstant() };
        modified.setNormal(vec3(normal));
        modified.setConstant(constant);
        properties.check(.{ p, normal, constant }, .{ before[0], before[1], arrPlane(modified) }, .{ expected_normal, expected_constant, expected4 });

        const distance = rng.float(-100, 100);
        jolt.jolt_plane_offset(&p, distance, &expected4);
        offset.check(.{ p, distance }, arrPlane(plane(p).offset(distance)), expected4);

        const m = rng.matrix();
        const ma = arr16(m);
        jolt.jolt_plane_get_transformed(&p, &ma, &expected4);
        get_transformed.check(.{ p, ma }, arrPlane(plane(p).getTransformed(m)), expected4);

        const scale: [3]f32 = if (rng.chance(4)) .{ rng.anyMagnitude(), rng.anyMagnitude(), rng.anyMagnitude() } else rng.nonZero(3, -10, 10);
        if (scale[0] != 0 and scale[1] != 0 and scale[2] != 0) {
            jolt.jolt_plane_scaled(&p, &scale, &expected4);
            scaled.check(.{ p, scale }, arrPlane(plane(p).scaled(vec3(scale))), expected4);
        }

        signed_distance.check(.{ p, point }, plane(p).signedDistance(vec3(point)), jolt.jolt_plane_signed_distance(&p, &point));
        jolt.jolt_plane_project_point_on_plane(&p, &point, &expected3);
        project_point.check(.{ p, point }, arr3(plane(p).projectPointOnPlane(vec3(point))), expected3);

        // 3 planes: random, axis aligned or with parallel normals (no intersection)
        var planes: [3][4]f32 = .{ rng.array(4, -10, 10), rng.array(4, -10, 10), rng.array(4, -10, 10) };
        switch (rng.next() % 4) {
            0 => for (&planes, 0..) |*pl, i| {
                pl.* = .{ 0, 0, 0, rng.float(-10, 10) };
                pl[i] = if (rng.chance(2)) 1 else -1;
            },
            1 => {
                const s = rng.float(-2, 2);
                planes[2] = .{ planes[0][0] * s, planes[0][1] * s, planes[0][2] * s, rng.float(-10, 10) };
            },
            2 => planes[1] = .{ 0, 0, 0, 1 },
            else => {},
        }
        const found = jolt.jolt_plane_intersect_planes(&planes[0], &planes[1], &planes[2], &expected3);
        const result = Plane.intersectPlanes(plane(planes[0]), plane(planes[1]), plane(planes[2]));
        intersect_planes.check(planes, .{ result != null, if (result) |r| arr3(r) else [3]f32{ 0, 0, 0 } }, .{ found, if (found) expected3 else [3]f32{ 0, 0, 0 } });
    }
    try finishAll(&.{ &init_checker, &from_point_and_normal, &from_point_and_normal_dvec3, &from_points_ccw, &properties, &offset, &get_transformed, &scaled, &signed_distance, &project_point, &intersect_planes });
}

test "AABox constructors / compare / encapsulate" {
    var rng: Rng = .{};
    var constants: Checker = .{ .name = "AABox.empty / biggest" };
    var from_dvec3: Checker = .{ .name = "AABox.fromDVec3" };
    var from_center_and_radius: Checker = .{ .name = "AABox.fromCenterAndRadius" };
    var from_two_points: Checker = .{ .name = "AABox.fromTwoPoints" };
    var from_triangle: Checker = .{ .name = "AABox.fromTriangle" };
    var compare: Checker = .{ .name = "AABox.eql / != / isValid" };
    var encapsulate_vec3: Checker = .{ .name = "AABox.encapsulateVec3" };
    var encapsulate: Checker = .{ .name = "AABox.encapsulate" };
    var encapsulate_triangle: Checker = .{ .name = "AABox.encapsulateTriangle" };
    var encapsulate_indexed_triangle: Checker = .{ .name = "AABox.encapsulateIndexedTriangle" };

    var expected_empty: [6]f32 = undefined;
    var expected_biggest: [6]f32 = undefined;
    jolt.jolt_aabox_constants(&expected_empty, &expected_biggest);
    constants.check({}, .{ arrBox(AABox.empty), arrBox(AABox.biggest()) }, .{ expected_empty, expected_biggest });

    var vertices: [5]Float3 = undefined;
    for (0..iterations) |_| {
        var expected: [6]f32 = undefined;

        // Doubles of any magnitude, and doubles that are exactly representable as floats
        const range: f64 = switch (rng.next() % 3) {
            0 => 1.0,
            1 => 1.0e5,
            else => 1.0e30,
        };
        var dmin = rng.doubleArray(3, -range, range);
        var dmax = rng.doubleArray(3, -range, range);
        if (rng.chance(5)) {
            for (&dmin, &dmax) |*a, *b| {
                a.* = @as(f32, @floatCast(a.*));
                b.* = @as(f32, @floatCast(b.*));
            }
        }
        if (rng.chance(10)) dmax[rng.index(3)] = 1.0e300; // Out of float range
        jolt.jolt_aabox_from_dvec3(&dmin, &dmax, &expected);
        from_dvec3.check(.{ dmin, dmax }, arrBox(AABox.fromDVec3(dvec3(dmin), dvec3(dmax))), expected);

        const center = rng.arrayOrExtreme(3, -100, 100);
        const radius = rng.floatOrExtreme(-10, 100);
        jolt.jolt_aabox_from_center_and_radius(&center, radius, &expected);
        from_center_and_radius.check(.{ center, radius }, arrBox(AABox.fromCenterAndRadius(vec3(center), radius)), expected);

        const p1 = rng.arrayOrExtreme(3, -100, 100);
        const p2 = if (rng.chance(10)) p1 else rng.arrayOrExtreme(3, -100, 100);
        jolt.jolt_aabox_from_two_points(&p1, &p2, &expected);
        from_two_points.check(.{ p1, p2 }, arrBox(AABox.fromTwoPoints(vec3(p1), vec3(p2))), expected);

        randomVertices(&rng, &vertices);
        if (rng.chance(10)) vertices[rng.index(vertices.len)] = .init(rng.extreme(), 0, rng.extreme());
        const n: u32 = vertices.len;
        const it = IndexedTriangle.init(rng.next() % n, rng.next() % n, rng.next() % n, .{});
        jolt.jolt_aabox_from_triangle(&vertices, n, &it, &expected);
        from_triangle.check(.{ vertices, it }, arrBox(AABox.fromTriangle(&vertices, it)), expected);

        const a = rng.box();
        const b = rng.relatedBox(a);
        var expected_equal: bool = undefined;
        var expected_not_equal: bool = undefined;
        var expected_valid: bool = undefined;
        jolt.jolt_aabox_compare(&a, &b, &expected_equal, &expected_not_equal, &expected_valid);
        compare.check(.{ a, b }, .{ aabox(a).eql(aabox(b)), !aabox(a).eql(aabox(b)), aabox(a).isValid() }, .{ expected_equal, expected_not_equal, expected_valid });

        const point = rng.relatedPoint(a);
        const tri = Triangle.fromFloat3(vertices[0], vertices[1], vertices[2], .{});
        var expected_vec3: [6]f32 = undefined;
        var expected_aabox: [6]f32 = undefined;
        var expected_triangle: [6]f32 = undefined;
        var expected_indexed_triangle: [6]f32 = undefined;
        jolt.jolt_aabox_encapsulate(&a, &b, &point, &tri, &vertices, n, &it, &expected_vec3, &expected_aabox, &expected_triangle, &expected_indexed_triangle);
        var box = aabox(a);
        box.encapsulateVec3(vec3(point));
        encapsulate_vec3.check(.{ a, point }, arrBox(box), expected_vec3);
        box = aabox(a);
        box.encapsulate(aabox(b));
        encapsulate.check(.{ a, b }, arrBox(box), expected_aabox);
        box = aabox(a);
        box.encapsulateTriangle(tri);
        encapsulate_triangle.check(.{ a, tri }, arrBox(box), expected_triangle);
        box = aabox(a);
        box.encapsulateIndexedTriangle(&vertices, it);
        encapsulate_indexed_triangle.check(.{ a, vertices, it }, arrBox(box), expected_indexed_triangle);
    }
    try finishAll(&.{ &constants, &from_dvec3, &from_center_and_radius, &from_two_points, &from_triangle, &compare, &encapsulate_vec3, &encapsulate, &encapsulate_triangle, &encapsulate_indexed_triangle });
}

test "AABox properties / intersect / ensureMinimalEdgeLength / expandBy" {
    var rng: Rng = .{};
    var properties: Checker = .{ .name = "AABox.getCenter / getExtent / getSize / getSurfaceArea / getVolume" };
    var intersect: Checker = .{ .name = "AABox.intersect" };
    var ensure_minimal_edge_length: Checker = .{ .name = "AABox.ensureMinimalEdgeLength" };
    var expand_by: Checker = .{ .name = "AABox.expandBy" };
    for (0..iterations) |_| {
        var expected: [6]f32 = undefined;

        const a = rng.box();
        var center: [3]f32 = undefined;
        var extent: [3]f32 = undefined;
        var size: [3]f32 = undefined;
        var area: f32 = undefined;
        var volume: f32 = undefined;
        jolt.jolt_aabox_properties(&a, &center, &extent, &size, &area, &volume);
        const box = aabox(a);
        properties.check(a, .{ arr3(box.getCenter()), arr3(box.getExtent()), arr3(box.getSize()), box.getSurfaceArea(), box.getVolume() }, .{ center, extent, size, area, volume });

        const b = rng.relatedBox(a);
        jolt.jolt_aabox_intersect(&a, &b, &expected);
        intersect.check(.{ a, b }, arrBox(box.intersect(aabox(b))), expected);

        const length = if (rng.chance(4)) a[3 + rng.index(3)] - a[rng.index(3)] else rng.floatOrExtreme(0, 10);
        jolt.jolt_aabox_ensure_minimal_edge_length(&a, length, &expected);
        var e = box;
        e.ensureMinimalEdgeLength(length);
        ensure_minimal_edge_length.check(.{ a, length }, arrBox(e), expected);

        const v = rng.arrayOrExtreme(3, -10, 10);
        jolt.jolt_aabox_expand_by(&a, &v, &expected);
        var x = box;
        x.expandBy(vec3(v));
        expand_by.check(.{ a, v }, arrBox(x), expected);
    }
    try finishAll(&.{ &properties, &intersect, &ensure_minimal_edge_length, &expand_by });
}

test "AABox contains / overlaps" {
    var rng: Rng = .{};
    var contains: Checker = .{ .name = "AABox.contains" };
    var contains_vec3: Checker = .{ .name = "AABox.containsVec3" };
    var contains_dvec3: Checker = .{ .name = "AABox.containsDVec3" };
    var overlaps: Checker = .{ .name = "AABox.overlaps" };
    var overlaps_plane: Checker = .{ .name = "AABox.overlapsPlane" };
    for (0..iterations) |_| {
        const a = rng.box();
        const b = rng.relatedBox(a);
        const point = rng.relatedPoint(a);

        // Double precision point: close to the point (rounding decides), or far away
        var dpoint: [3]f64 = .{ point[0], point[1], point[2] };
        if (rng.chance(2)) {
            for (&dpoint) |*d| d.* += rng.double(-1.0e-5, 1.0e-5);
        } else if (rng.chance(4)) {
            dpoint = rng.doubleArray(3, -1.0e40, 1.0e40);
        }

        // Plane: random, through a corner of the box or touching a face
        var p = rng.array(4, -10, 10);
        switch (rng.next() % 3) {
            0 => {
                const n = rng.unitVector(3);
                const corner = [3]f32{ a[3 * rng.index(2)], a[1 + 3 * rng.index(2)], a[2 + 3 * rng.index(2)] };
                p = arrPlane(Plane.fromPointAndNormal(vec3(corner), vec3(n)));
            },
            1 => {
                const axis = rng.index(3);
                var n: [3]f32 = .{ 0, 0, 0 };
                n[axis] = if (rng.chance(2)) 1 else -1;
                p = .{ n[0], n[1], n[2], -n[axis] * a[axis + 3 * rng.index(2)] };
            },
            else => {},
        }

        var expected_contains: bool = undefined;
        var expected_contains_vec3: bool = undefined;
        var expected_contains_dvec3: bool = undefined;
        var expected_overlaps: bool = undefined;
        var expected_overlaps_plane: bool = undefined;
        jolt.jolt_aabox_contains_overlaps(&a, &b, &point, &dpoint, &p, &expected_contains, &expected_contains_vec3, &expected_contains_dvec3, &expected_overlaps, &expected_overlaps_plane);
        const box = aabox(a);
        contains.check(.{ a, b }, box.contains(aabox(b)), expected_contains);
        contains_vec3.check(.{ a, point }, box.containsVec3(vec3(point)), expected_contains_vec3);
        contains_dvec3.check(.{ a, dpoint }, box.containsDVec3(dvec3(dpoint)), expected_contains_dvec3);
        overlaps.check(.{ a, b }, box.overlaps(aabox(b)), expected_overlaps);
        overlaps_plane.check(.{ a, p }, box.overlapsPlane(plane(p)), expected_overlaps_plane);
    }
    try finishAll(&.{ &contains, &contains_vec3, &contains_dvec3, &overlaps, &overlaps_plane });
}

test "AABox translate / transformed / scaled" {
    var rng: Rng = .{};
    var translate: Checker = .{ .name = "AABox.translate" };
    var translate_dvec3: Checker = .{ .name = "AABox.translateDVec3" };
    var transformed: Checker = .{ .name = "AABox.transformed" };
    var transformed_dmat44: Checker = .{ .name = "AABox.transformedDMat44" };
    var scaled: Checker = .{ .name = "AABox.scaled" };
    for (0..iterations) |_| {
        var expected: [6]f32 = undefined;

        const a = if (rng.chance(4)) rng.box() else rng.validBox(100);
        const box = aabox(a);

        const t = rng.arrayOrExtreme(3, -100, 100);
        jolt.jolt_aabox_translate(&a, &t, &expected);
        var tb = box;
        tb.translate(vec3(t));
        translate.check(.{ a, t }, arrBox(tb), expected);

        const drange: f64 = if (rng.chance(2)) 100.0 else 1.0e9;
        const dt = rng.doubleArray(3, -drange, drange);
        jolt.jolt_aabox_translate_dvec3(&a, &dt, &expected);
        tb = box;
        tb.translateDVec3(dvec3(dt));
        translate_dvec3.check(.{ a, dt }, arrBox(tb), expected);

        const m = rng.matrix();
        const ma = arr16(m);
        jolt.jolt_aabox_transformed(&a, &ma, &expected);
        transformed.check(.{ a, ma }, arrBox(box.transformed(m)), expected);

        const cols: [12]f32 = ma[0..12].*;
        const dm = DMat44.init(vec4(cols[0..4].*), vec4(cols[4..8].*), vec4(cols[8..12].*), dvec3(dt));
        jolt.jolt_aabox_transformed_dmat44(&a, &cols, &dt, &expected);
        transformed_dmat44.check(.{ a, cols, dt }, arrBox(box.transformedDMat44(dm)), expected);

        const s = rng.arrayOrExtreme(3, -10, 10);
        jolt.jolt_aabox_scaled(&a, &s, &expected);
        scaled.check(.{ a, s }, arrBox(box.scaled(vec3(s))), expected);
    }
    try finishAll(&.{ &translate, &translate_dvec3, &transformed, &transformed_dmat44, &scaled });
}

test "AABox getSupport / getSupportingFace / getClosestPoint / getSqDistanceTo" {
    var rng: Rng = .{};
    var get_support: Checker = .{ .name = "AABox.getSupport" };
    var supporting_face_static: Checker = .{ .name = "AABox.getSupportingFace (StaticArray)" };
    var supporting_face_list: Checker = .{ .name = "AABox.getSupportingFace (std.ArrayList)" };
    var closest_point: Checker = .{ .name = "AABox.getClosestPoint / getSqDistanceTo" };
    const allocator = std.testing.allocator;
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(allocator);
    for (0..iterations) |_| {
        var expected3: [3]f32 = undefined;

        const a = rng.box();
        const box = aabox(a);
        const d = rng.direction();
        jolt.jolt_aabox_get_support(&a, &d, &expected3);
        get_support.check(.{ a, d }, arr3(box.getSupport(vec3(d))), expected3);

        var expected_face: [12]f32 = undefined;
        jolt.jolt_aabox_get_supporting_face(&a, &d, &expected_face);
        var face: StaticArray(Vec3, 8) = .empty;
        try box.getSupportingFace(vec3(d), &face);
        var face_result: [12]f32 = @splat(0);
        for (face.constSlice(), 0..) |v, i| face_result[3 * i ..][0..3].* = arr3(v);
        supporting_face_static.check(.{ a, d }, .{ face.len, face_result }, .{ 4, expected_face });
        try box.getSupportingFace(vec3(d), VertexArrayList.init(allocator, &list));
        face_result = @splat(0);
        for (list.items, 0..) |v, i| face_result[3 * i ..][0..3].* = arr3(v);
        supporting_face_list.check(.{ a, d }, .{ @as(u32, @intCast(list.items.len)), face_result }, .{ 4, expected_face });

        const point = rng.relatedPoint(a);
        var expected_sq_distance: f32 = undefined;
        jolt.jolt_aabox_get_closest_point(&a, &point, &expected3, &expected_sq_distance);
        closest_point.check(.{ a, point }, .{ arr3(box.getClosestPoint(vec3(point))), box.getSqDistanceTo(vec3(point)) }, .{ expected3, expected_sq_distance });
    }
    try finishAll(&.{ &get_support, &supporting_face_static, &supporting_face_list, &closest_point });
}

test "Sphere" {
    var rng: Rng = .{};
    var get_support: Checker = .{ .name = "Sphere.getSupport" };
    var overlaps: Checker = .{ .name = "Sphere.overlaps" };
    var overlaps_aabox: Checker = .{ .name = "Sphere.overlapsAABox" };
    var encapsulate_point: Checker = .{ .name = "Sphere.encapsulatePoint" };
    for (0..iterations) |_| {
        var expected3: [3]f32 = undefined;
        var expected4: [4]f32 = undefined;

        const s = rng.array(3, -100, 100) ++ [1]f32{if (rng.chance(10)) 0 else rng.float(0, 50)};
        const d = rng.direction();
        jolt.jolt_sphere_get_support(&s, &d, &expected3);
        get_support.check(.{ s, d }, arr3(sphere(s).getSupport(vec3(d))), expected3);

        // Second sphere: random or exactly touching along an axis
        var s2 = rng.array(3, -100, 100) ++ [1]f32{rng.float(0, 50)};
        if (rng.chance(4)) {
            s2 = s;
            s2[3] = rng.float(0, 50);
            const axis = rng.index(3);
            s2[axis] = s[axis] + s[3] + s2[3];
            s2[axis] = nextFloat(s2[axis], @intCast(rng.next() % 3), rng.chance(2));
        }
        overlaps.check(.{ s, s2 }, sphere(s).overlaps(sphere(s2)), jolt.jolt_sphere_overlaps(&s, &s2));

        // Box: random or touching the sphere
        var b = rng.box();
        if (rng.chance(4)) {
            b = rng.validBox(20);
            const axis = rng.index(3);
            const size = b[3 + axis] - b[axis];
            b[axis] = nextFloat(s[axis] + s[3], @intCast(rng.next() % 3), rng.chance(2));
            b[3 + axis] = b[axis] + size;
            for (0..3) |i| if (i != axis) {
                b[i] = s[i] - 1;
                b[3 + i] = s[i] + 1;
            };
        }
        overlaps_aabox.check(.{ s, b }, sphere(s).overlapsAABox(aabox(b)), jolt.jolt_sphere_overlaps_aabox(&s, &b));

        // Point: random, inside, on the surface
        var p = rng.array(3, -150, 150);
        if (rng.chance(4)) {
            p = arr3(vec3(s[0..3].*).add(vec3(rng.unitVector(3)).mulScalar(s[3])));
        } else if (rng.chance(4)) {
            p = s[0..3].*;
        }
        jolt.jolt_sphere_encapsulate_point(&s, &p, &expected4);
        var e = sphere(s);
        e.encapsulatePoint(vec3(p));
        encapsulate_point.check(.{ s, p }, arrSphere(e), expected4);
    }
    try finishAll(&.{ &get_support, &overlaps, &overlaps_aabox, &encapsulate_point });
}

/// Find the distance along `dir` at which an oriented box (orientation, half extents) moving away from `start` stops
/// overlapping, using Zolt's overlap test (bisection). Used to generate inputs on the boundary.
fn findOverlapBoundary(overlap_fn: anytype, context: anytype, orientation: Mat44, dir: Vec3) f32 {
    var lo: f32 = 0;
    var hi: f32 = 1000;
    if (!overlap_fn(context, orientation) or overlap_fn(context, orientation.postTranslated(dir.mulScalar(hi))))
        return 0;
    for (0..60) |_| {
        const mid = 0.5 * (lo + hi);
        if (mid == lo or mid == hi) break;
        if (overlap_fn(context, orientation.postTranslated(dir.mulScalar(mid))))
            lo = mid
        else
            hi = mid;
    }
    return lo;
}

/// Random epsilon for the oriented box tests: the default, zero or bigger
fn randomEpsilon(rng: *Rng) f32 {
    return switch (rng.next() % 4) {
        0 => 0,
        1 => 1.0e-3,
        2 => rng.float(0, 0.1),
        else => 1.0e-6,
    };
}

test "OrientedBox" {
    var rng: Rng = .{};
    var from_aabox: Checker = .{ .name = "OrientedBox.fromAABox" };
    var overlaps_aabox: Checker = .{ .name = "OrientedBox.overlapsAABox" };
    var overlaps: Checker = .{ .name = "OrientedBox.overlaps" };

    const AABoxContext = struct {
        box: AABox,
        half_extents: Vec3,
        epsilon: f32,
        fn overlap(self: @This(), m: Mat44) bool {
            return OrientedBox.init(m, self.half_extents).overlapsAABox(self.box, .{ .epsilon = self.epsilon });
        }
    };
    const OBBContext = struct {
        a: OrientedBox,
        half_extents: Vec3,
        epsilon: f32,
        fn overlap(self: @This(), m: Mat44) bool {
            return self.a.overlaps(OrientedBox.init(m, self.half_extents), .{ .epsilon = self.epsilon });
        }
    };

    for (0..iterations) |_| {
        const b = if (rng.chance(10)) rng.box() else rng.validBox(20);
        const m = rng.rotationTranslation(20);
        const ma = arr16(m);
        var expected_m: [16]f32 = undefined;
        var expected_he: [3]f32 = undefined;
        jolt.jolt_oriented_box_from_aabox(&ma, &b, &expected_m, &expected_he);
        const ob = OrientedBox.fromAABox(m, aabox(b));
        from_aabox.check(.{ ma, b }, .{ arr16(ob.orientation), arr3(ob.half_extents) }, .{ expected_m, expected_he });

        // Oriented box vs axis aligned box: random placement, or on the boundary of overlapping
        const box = rng.validBox(10);
        const half_extents: [3]f32 = if (rng.chance(10)) .{ 0, rng.float(0, 5), rng.float(0, 5) } else rng.array(3, 0, 5);
        const epsilon = randomEpsilon(&rng);
        var orientation = rng.rotationTranslation(20);
        if (rng.chance(2)) {
            const context: AABoxContext = .{ .box = aabox(box), .half_extents = vec3(half_extents), .epsilon = epsilon };
            orientation.setTranslation(aabox(box).getCenter());
            const dir = vec3(if (rng.chance(3)) blk: {
                var d: [3]f32 = .{ 0, 0, 0 };
                d[rng.index(3)] = if (rng.chance(2)) 1 else -1;
                break :blk d;
            } else rng.unitVector(3));
            const boundary = findOverlapBoundary(AABoxContext.overlap, context, orientation, dir);
            orientation = orientation.postTranslated(dir.mulScalar(nextFloat(boundary, @intCast(rng.next() % 3), rng.chance(2))));
        }
        const oa = arr16(orientation);
        overlaps_aabox.check(.{ oa, half_extents, box, epsilon }, OrientedBox.init(orientation, vec3(half_extents)).overlapsAABox(aabox(box), .{ .epsilon = epsilon }), jolt.jolt_oriented_box_overlaps_aabox(&oa, &half_extents, &box, epsilon));

        // Oriented box vs oriented box
        const m1 = rng.rotationTranslation(10);
        const he1 = rng.array(3, 0, 5);
        var m2 = rng.rotationTranslation(20);
        const he2 = rng.array(3, 0, 5);
        if (rng.chance(2)) {
            const context: OBBContext = .{ .a = OrientedBox.init(m1, vec3(he1)), .half_extents = vec3(he2), .epsilon = epsilon };
            m2.setTranslation(m1.getTranslation());
            const dir = vec3(rng.unitVector(3));
            const boundary = findOverlapBoundary(OBBContext.overlap, context, m2, dir);
            m2 = m2.postTranslated(dir.mulScalar(nextFloat(boundary, @intCast(rng.next() % 3), rng.chance(2))));
        }
        const m1a = arr16(m1);
        const m2a = arr16(m2);
        overlaps.check(.{ m1a, he1, m2a, he2, epsilon }, OrientedBox.init(m1, vec3(he1)).overlaps(OrientedBox.init(m2, vec3(he2)), .{ .epsilon = epsilon }), jolt.jolt_oriented_box_overlaps(&m1a, &he1, &m2a, &he2, epsilon));
    }
    try finishAll(&.{ &from_aabox, &overlaps_aabox, &overlaps });
}

test "AABox4" {
    var rng: Rng = .{};
    var vs_box: Checker = .{ .name = "aabox4VsBox" };
    var scale: Checker = .{ .name = "aabox4Scale" };
    var enlarge: Checker = .{ .name = "aabox4EnlargeWithExtent" };
    var vs_point: Checker = .{ .name = "aabox4VsPoint" };
    var vs_oriented_box_mat44: Checker = .{ .name = "aabox4VsOrientedBoxMat44" };
    var vs_oriented_box: Checker = .{ .name = "aabox4VsOrientedBox" };
    var distance_sq: Checker = .{ .name = "aabox4DistanceSqToPoint" };
    var distance_sq_vec3: Checker = .{ .name = "aabox4DistanceSqToPointVec3" };
    var vs_sphere: Checker = .{ .name = "aabox4VsSphere" };
    var vs_sphere_vec3: Checker = .{ .name = "aabox4VsSphereVec3" };

    const BoundaryContext = struct {
        box: AABox,
        half_extents: Vec3,
        epsilon: f32,
        fn overlap(self: @This(), m: Mat44) bool {
            const b = self.box;
            const result = zolt.aabox4VsOrientedBoxMat44(m, self.half_extents, b.min.splatX(), b.min.splatY(), b.min.splatZ(), b.max.splatX(), b.max.splatY(), b.max.splatZ(), .{ .epsilon = self.epsilon });
            return result.getX() != 0;
        }
    };

    for (0..iterations) |_| {
        var expected_u: [4]u32 = undefined;
        var expected_f: [4]f32 = undefined;
        var expected_b: [24]f32 = undefined;

        const box1 = rng.box();
        const ba = rng.bounds(if (rng.chance(2)) box1 else null);
        const b = Bounds.load(ba);
        jolt.jolt_aabox4_vs_box(&box1, &ba, &expected_u);
        vs_box.check(.{ box1, ba }, arrU4(zolt.aabox4VsBox(aabox(box1), b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z)), expected_u);

        const s = rng.arrayOrExtreme(3, -10, 10);
        jolt.jolt_aabox4_scale(&s, &ba, &expected_b);
        const scaled = zolt.aabox4Scale(vec3(s), b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z);
        const scaled_bounds: Bounds = .{ .min_x = scaled.min_x, .min_y = scaled.min_y, .min_z = scaled.min_z, .max_x = scaled.max_x, .max_y = scaled.max_y, .max_z = scaled.max_z };
        scale.check(.{ s, ba }, scaled_bounds.store(), expected_b);

        const extent = rng.arrayOrExtreme(3, -10, 10);
        jolt.jolt_aabox4_enlarge_with_extent(&extent, &ba, &expected_b);
        var e = b;
        zolt.aabox4EnlargeWithExtent(vec3(extent), &e.min_x, &e.min_y, &e.min_z, &e.max_x, &e.max_y, &e.max_z);
        enlarge.check(.{ extent, ba }, e.store(), expected_b);

        const point = rng.relatedPoint(box1);
        jolt.jolt_aabox4_vs_point(&point, &ba, &expected_u);
        vs_point.check(.{ point, ba }, arrU4(zolt.aabox4VsPoint(vec3(point), b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z)), expected_u);

        // Oriented box: random or on the overlap boundary of the first box
        const vb = rng.bounds(null);
        const v = Bounds.load(vb);
        const half_extents = rng.array(3, 0, 5);
        const epsilon = randomEpsilon(&rng);
        var orientation = rng.rotationTranslation(20);
        if (rng.chance(2)) {
            const first = AABox.init(Vec3.init(v.min_x.getX(), v.min_y.getX(), v.min_z.getX()), Vec3.init(v.max_x.getX(), v.max_y.getX(), v.max_z.getX()));
            if (first.isValid()) {
                const context: BoundaryContext = .{ .box = first, .half_extents = vec3(half_extents), .epsilon = epsilon };
                orientation.setTranslation(first.getCenter());
                const dir = vec3(rng.unitVector(3));
                const boundary = findOverlapBoundary(BoundaryContext.overlap, context, orientation, dir);
                orientation = orientation.postTranslated(dir.mulScalar(nextFloat(boundary, @intCast(rng.next() % 3), rng.chance(2))));
            }
        }
        const oa = arr16(orientation);
        jolt.jolt_aabox4_vs_oriented_box_mat44(&oa, &half_extents, &vb, epsilon, &expected_u);
        vs_oriented_box_mat44.check(.{ oa, half_extents, vb, epsilon }, arrU4(zolt.aabox4VsOrientedBoxMat44(orientation, vec3(half_extents), v.min_x, v.min_y, v.min_z, v.max_x, v.max_y, v.max_z, .{ .epsilon = epsilon })), expected_u);
        jolt.jolt_aabox4_vs_oriented_box(&oa, &half_extents, &vb, epsilon, &expected_u);
        vs_oriented_box.check(.{ oa, half_extents, vb, epsilon }, arrU4(zolt.aabox4VsOrientedBox(.init(orientation, vec3(half_extents)), v.min_x, v.min_y, v.min_z, v.max_x, v.max_y, v.max_z, .{ .epsilon = epsilon })), expected_u);

        // Distance to point / sphere (point per lane or splatted)
        const p4 = rng.arrayOrExtreme(12, -100, 100);
        jolt.jolt_aabox4_distance_sq_to_point(&p4, &ba, &expected_f);
        distance_sq.check(.{ p4, ba }, arr4(zolt.aabox4DistanceSqToPoint(vec4(p4[0..4].*), vec4(p4[4..8].*), vec4(p4[8..12].*), b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z)), expected_f);
        jolt.jolt_aabox4_distance_sq_to_point_vec3(&point, &ba, &expected_f);
        distance_sq_vec3.check(.{ point, ba }, arr4(zolt.aabox4DistanceSqToPointVec3(vec3(point), b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z)), expected_f);

        var radius_sq: [4]f32 = rng.array(4, 0, 1000);
        if (rng.chance(4)) {
            // Exactly on the boundary
            const d = zolt.aabox4DistanceSqToPoint(vec4(p4[0..4].*), vec4(p4[4..8].*), vec4(p4[8..12].*), b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z);
            radius_sq = arr4(d);
        }
        jolt.jolt_aabox4_vs_sphere(&p4, &radius_sq, &ba, &expected_u);
        vs_sphere.check(.{ p4, radius_sq, ba }, arrU4(zolt.aabox4VsSphere(vec4(p4[0..4].*), vec4(p4[4..8].*), vec4(p4[8..12].*), vec4(radius_sq), b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z)), expected_u);
        jolt.jolt_aabox4_vs_sphere_vec3(&point, radius_sq[0], &ba, &expected_u);
        vs_sphere_vec3.check(.{ point, radius_sq[0], ba }, arrU4(zolt.aabox4VsSphereVec3(vec3(point), radius_sq[0], b.min_x, b.min_y, b.min_z, b.max_x, b.max_y, b.max_z)), expected_u);
    }
    try finishAll(&.{ &vs_box, &scale, &enlarge, &vs_point, &vs_oriented_box_mat44, &vs_oriented_box, &distance_sq, &distance_sq_vec3, &vs_sphere, &vs_sphere_vec3 });
}

/// Maximum number of vertices of a clipped polygon in the ClipPoly tests
const max_clip_vertices = 256;

/// Result of a ClipPoly function: the vertex count and the vertices (unused entries are 0)
const ClipResult = struct {
    count: u32,
    vertices: [max_clip_vertices][3]f32,

    fn fromSlice(vertices: []const Vec3) ClipResult {
        var result: ClipResult = .{ .count = @intCast(vertices.len), .vertices = @splat(.{ 0, 0, 0 }) };
        for (vertices, 0..) |v, i| result.vertices[i] = arr3(v);
        return result;
    }
};

/// Random polygon in a plane: (non) convex, with duplicate or collinear vertices, axis aligned
fn randomPolygon(rng: *Rng, out: [][3]f32, min_vertices: usize) [][3]f32 {
    const n = min_vertices + rng.index(out.len - min_vertices + 1);
    const center = vec3(rng.array(3, -2, 2));
    var u = vec3(rng.unitVector(3));
    var v = u.getNormalizedPerpendicular();
    if (rng.chance(4)) {
        // Axis aligned plane
        const axis = rng.index(3);
        u = Vec3.zero();
        u.setComponent(@intCast((axis + 1) % 3), 1);
        v = Vec3.zero();
        v.setComponent(@intCast((axis + 2) % 3), 1);
    }
    const convex = !rng.chance(4);
    const base_radius = rng.float(0.1, 3);
    var angle: f32 = rng.float(0, 1);
    for (out[0..n], 0..) |*p, i| {
        angle += 2.0 * zolt.math.pi / @as(f32, @floatFromInt(n)) * (if (convex) 1.0 else rng.float(0.2, 1.8));
        const radius = if (convex) base_radius else base_radius * rng.float(0.2, 1.5);
        const sc = Vec4.replicate(angle).sinCos();
        var point = center.add(u.mulScalar(radius * sc.cos.getX())).add(v.mulScalar(radius * sc.sin.getX()));
        if (i > 0 and rng.chance(20))
            point = vec3(out[i - 1]); // Duplicate vertex
        p.* = arr3(point);
    }
    if (rng.chance(10)) {
        // Collinear: move a vertex onto the line between its neighbors
        const i = rng.index(n);
        const a = vec3(out[(i + n - 1) % n]);
        const b = vec3(out[(i + 1) % n]);
        out[i] = arr3(a.add(b.sub(a).mulScalar(0.5)));
    }
    return out[0..n];
}

test "ClipPoly" {
    var rng: Rng = .{};
    var vs_plane: Checker = .{ .name = "clipPolyVsPlane (StaticArray)" };
    var vs_plane_list: Checker = .{ .name = "clipPolyVsPlane (std.ArrayList)" };
    var vs_poly: Checker = .{ .name = "clipPolyVsPoly (StaticArray)" };
    var vs_poly_list: Checker = .{ .name = "clipPolyVsPoly (std.ArrayList)" };
    var vs_edge: Checker = .{ .name = "clipPolyVsEdge (StaticArray)" };
    var vs_edge_list: Checker = .{ .name = "clipPolyVsEdge (std.ArrayList)" };
    var vs_aabox: Checker = .{ .name = "clipPolyVsAABox (StaticArray)" };
    var vs_aabox_list: Checker = .{ .name = "clipPolyVsAABox (std.ArrayList)" };

    const allocator = std.testing.allocator;
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(allocator);
    const list_out = VertexArrayList.init(allocator, &list);

    var poly_buffer: [8][3]f32 = undefined;
    var clip_buffer: [8][3]f32 = undefined;
    var expected: ClipResult = undefined;
    var out: StaticArray(Vec3, max_clip_vertices) = .empty;
    for (0..iterations) |_| {
        const poly_arr = randomPolygon(&rng, &poly_buffer, 2);
        var poly: [8]Vec3 = undefined;
        for (poly_arr, 0..) |p, i| poly[i] = vec3(p);
        const polygon = poly[0..poly_arr.len];
        const n: u32 = @intCast(polygon.len);

        // Plane: random, through a vertex of the polygon, or the plane of the polygon itself
        var origin = rng.array(3, -2, 2);
        var normal = rng.array(3, -1, 1);
        switch (rng.next() % 5) {
            0 => origin = poly_arr[rng.index(n)],
            1 => {
                origin = poly_arr[0];
                if (n >= 3) normal = arr3(polygon[1].sub(polygon[0]).cross(polygon[2].sub(polygon[0])));
            },
            2 => {
                normal = .{ 0, 0, 0 };
                normal[rng.index(3)] = if (rng.chance(2)) 1 else -1;
                origin = poly_arr[rng.index(n)];
            },
            else => {},
        }
        expected.vertices = @splat(.{ 0, 0, 0 });
        expected.count = jolt.jolt_clip_poly_vs_plane(poly_arr.ptr, n, &origin, &normal, &expected.vertices, max_clip_vertices);
        out.clear();
        try zolt.clipPolyVsPlane(polygon, vec3(origin), vec3(normal), &out);
        vs_plane.check(.{ poly_arr, origin, normal }, ClipResult.fromSlice(out.constSlice()), expected);
        list.clearRetainingCapacity();
        try zolt.clipPolyVsPlane(polygon, vec3(origin), vec3(normal), list_out);
        vs_plane_list.check(.{ poly_arr, origin, normal }, ClipResult.fromSlice(list.items), expected);

        // Clipping polygon: random, the same polygon, a box face or a big polygon around it
        var clip_arr = randomPolygon(&rng, &clip_buffer, 3);
        if (rng.chance(8) and n >= 3) {
            @memcpy(clip_buffer[0..n], poly_arr);
            clip_arr = clip_buffer[0..n];
        }
        var clip: [8]Vec3 = undefined;
        for (clip_arr, 0..) |p, i| clip[i] = vec3(p);
        const clipping = clip[0..clip_arr.len];
        const m: u32 = @intCast(clipping.len);
        const clip_normal = if (rng.chance(4)) rng.array(3, -1, 1) else arr3(clipping[1].sub(clipping[0]).cross(clipping[2].sub(clipping[0])));
        expected.vertices = @splat(.{ 0, 0, 0 });
        expected.count = jolt.jolt_clip_poly_vs_poly(poly_arr.ptr, n, clip_arr.ptr, m, &clip_normal, &expected.vertices, max_clip_vertices);
        out.clear();
        try zolt.clipPolyVsPoly(polygon, clipping, vec3(clip_normal), &out);
        vs_poly.check(.{ poly_arr, clip_arr[0..m], clip_normal }, ClipResult.fromSlice(out.constSlice()), expected);
        list.clearRetainingCapacity();
        try zolt.clipPolyVsPoly(polygon, clipping, vec3(clip_normal), list_out);
        vs_poly_list.check(.{ poly_arr, clip_arr[0..m], clip_normal }, ClipResult.fromSlice(list.items), expected);

        // Edge: random or an edge of the clipping polygon
        if (n >= 3) {
            const e = rng.index(m);
            const edge1 = if (rng.chance(2)) clip_arr[e] else rng.array(3, -2, 2);
            const edge2 = if (rng.chance(2)) clip_arr[(e + 1) % m] else rng.array(3, -2, 2);
            const edge_normal = if (rng.chance(4)) rng.array(3, -1, 1) else clip_normal;
            expected.vertices = @splat(.{ 0, 0, 0 });
            expected.count = jolt.jolt_clip_poly_vs_edge(poly_arr.ptr, n, &edge1, &edge2, &edge_normal, &expected.vertices, max_clip_vertices);
            out.clear();
            try zolt.clipPolyVsEdge(polygon, vec3(edge1), vec3(edge2), vec3(edge_normal), &out);
            vs_edge.check(.{ poly_arr, edge1, edge2, edge_normal }, ClipResult.fromSlice(out.constSlice()), expected);
            list.clearRetainingCapacity();
            try zolt.clipPolyVsEdge(polygon, vec3(edge1), vec3(edge2), vec3(edge_normal), list_out);
            vs_edge_list.check(.{ poly_arr, edge1, edge2, edge_normal }, ClipResult.fromSlice(list.items), expected);
        }

        // Box: random, around the polygon, touching a vertex
        var box = rng.validBox(2);
        switch (rng.next() % 4) {
            0 => {
                var bounds: AABox = .empty;
                for (polygon) |p| bounds.encapsulateVec3(p);
                box = arrBox(bounds);
            },
            1 => {
                const p = poly_arr[rng.index(n)];
                const axis = rng.index(3);
                box[axis + 3 * rng.index(2)] = p[axis];
                box = arrBox(AABox.fromTwoPoints(vec3(box[0..3].*), vec3(box[3..6].*)));
            },
            else => {},
        }
        expected.vertices = @splat(.{ 0, 0, 0 });
        expected.count = jolt.jolt_clip_poly_vs_aabox(poly_arr.ptr, n, &box, &expected.vertices, max_clip_vertices);
        out.clear();
        try zolt.clipPolyVsAABox(polygon, aabox(box), &out);
        vs_aabox.check(.{ poly_arr, box }, ClipResult.fromSlice(out.constSlice()), expected);
        list.clearRetainingCapacity();
        try zolt.clipPolyVsAABox(polygon, aabox(box), list_out);
        vs_aabox_list.check(.{ poly_arr, box }, ClipResult.fromSlice(list.items), expected);
    }
    try finishAll(&.{ &vs_plane, &vs_plane_list, &vs_poly, &vs_poly_list, &vs_edge, &vs_edge_list, &vs_aabox, &vs_aabox_list });
}

test "MortonCode" {
    var rng: Rng = .{};
    var expand_bits: Checker = .{ .name = "MortonCode.expandBits" };
    var get_morton_code: Checker = .{ .name = "MortonCode.getMortonCode" };
    for (0..iterations) |_| {
        // Values in [0, 1], including the rounding boundaries (k + 0.5) / 1023
        var v = rng.float(0, 1);
        switch (rng.next() % 4) {
            0 => {
                const k: f32 = @floatFromInt(rng.next() % 1023);
                v = nextFloat((k + 0.5) / 1023.0, @intCast(rng.next() % 3), rng.chance(2));
            },
            1 => v = if (rng.chance(2)) 0 else 1,
            else => {},
        }
        v = std.math.clamp(v, 0, 1);
        expand_bits.check(v, MortonCode.expandBits(v), jolt.jolt_morton_code_expand_bits(v));

        // Point inside the bounds (bounds with a non zero size)
        var bounds = rng.validBox(if (rng.chance(2)) 1 else 1000);
        for (0..3) |i| if (bounds[3 + i] - bounds[i] <= 0) {
            bounds[3 + i] = bounds[i] + 1;
        };
        const b = aabox(bounds);
        var point = arr3(b.getClosestPoint(vec3(rng.array(3, -1100, 1100))));
        if (rng.chance(4)) point = arr3(b.min.add(b.getSize().mulScalar(rng.float(0, 1))));
        point = arr3(b.getClosestPoint(vec3(point)));
        get_morton_code.check(.{ point, bounds }, MortonCode.getMortonCode(vec3(point), b), jolt.jolt_morton_code_get_morton_code(&point, &bounds));
    }
    try finishAll(&.{ &expand_bits, &get_morton_code });
}

/// Compare the elements of two slices, every difference counts as a mismatch
fn checkSlices(checker: *Checker, input: anytype, comptime T: type, zolt_items: []const T, jolt_items: []const T) void {
    checker.check(.{ input, "count" }, zolt_items.len, jolt_items.len);
    for (zolt_items[0..@min(zolt_items.len, jolt_items.len)], jolt_items[0..@min(zolt_items.len, jolt_items.len)], 0..) |z, j, i|
        checker.check(.{ input, i }, z, j);
}

test "Indexify / Deindexify" {
    var rng: Rng = .{};
    var indexify_vertices: Checker = .{ .name = "indexify (vertices)" };
    var indexify_triangles: Checker = .{ .name = "indexify (triangles)" };
    var deindexify: Checker = .{ .name = "deindexify" };
    const allocator = std.testing.allocator;

    var triangles: TriangleList = .empty;
    defer triangles.deinit(allocator);
    var vertices: VertexList = .empty;
    defer vertices.deinit(allocator);
    var indexed: IndexedTriangleList = .empty;
    defer indexed.deinit(allocator);
    var expected_vertices: VertexList = .empty;
    defer expected_vertices.deinit(allocator);
    var expected_triangles: IndexedTriangleList = .empty;
    defer expected_triangles.deinit(allocator);
    var deindexified: TriangleList = .empty;
    defer deindexified.deinit(allocator);
    var expected_deindexified: TriangleList = .empty;
    defer expected_deindexified.deinit(allocator);
    var pool: [64]Vec3 = undefined;

    // Note: Jolt's recursive welding duplicates the vertices that are within the weld distance of the split plane into both
    // halves, so a weld distance that is large compared to the spacing of the vertices makes it exponentially slow. The
    // weld distances below are chosen per kind of mesh to avoid that.
    for (0..iterations / 50) |_| {
        // Pool of base vertices
        var pool_size = 3 + rng.index(pool.len - 2);
        var weld: f32 = undefined;
        switch (rng.next() % 8) {
            0, 1 => {
                // On a grid with spacing 0.5: exact duplicates, vertices exactly on the split planes
                for (pool[0..pool_size]) |*p|
                    p.* = Vec3.init(@floatFromInt(rng.next() % 5), @floatFromInt(rng.next() % 5), @floatFromInt(rng.next() % 5)).mulScalar(0.5);
                weld = ([_]f32{ 0, 1.0e-5, 1.0e-4, 1.0e-3, 1.0e-2, 0.1 })[rng.index(6)];
            },
            2 => {
                // Clustered around a point
                for (pool[0..pool_size]) |*p|
                    p.* = Vec3.init(1, 2, 3).add(vec3(rng.array(3, -0.1, 0.1)));
                weld = ([_]f32{ 0, 1.0e-5, 1.0e-4, 1.0e-3 })[rng.index(4)];
            },
            3 => {
                // A cluster plus a chain of outliers at doubling distances: every split only chips off the farthest
                // outlier, so the recursion reaches its maximum depth
                weld = ([_]f32{ 1.0e-4, 1.0e-3 })[rng.index(2)];
                pool_size = pool.len;
                const dir = vec3(rng.unitVector(3));
                for (pool[0..pool_size], 0..) |*p, i| {
                    if (i < 16)
                        p.* = Vec3.init(1, 2, 3).add(vec3(rng.array(3, -0.1 * weld, 0.1 * weld)))
                    else
                        p.* = Vec3.init(1, 2, 3).add(dir.mulScalar(weld * std.math.ldexp(@as(f32, 3), @intCast(i - 16))));
                }
            },
            else => {
                // Random
                for (pool[0..pool_size]) |*p|
                    p.* = vec3(rng.array(3, -10, 10));
                weld = ([_]f32{ 0, 1.0e-5, 1.0e-4, 1.0e-3, 1.0e-2, 0.1 })[rng.index(6)];
            },
        }

        // Triangles: 3 vertices from the pool, each copy perturbed a little (near duplicates), some degenerate
        const num_triangles = 1 + rng.index(if (rng.chance(4)) 300 else 30);
        const perturbation: f32 = switch (rng.next() % 5) {
            0 => 0,
            1 => 1.0e-6,
            2 => 1.0e-5,
            3 => 0.5 * weld,
            else => 2.0 * weld,
        };
        triangles.clearRetainingCapacity();
        for (0..num_triangles) |_| {
            var v: [3]Vec3 = undefined;
            for (&v) |*p| p.* = pool[rng.index(pool_size)].add(vec3(rng.array(3, -perturbation, perturbation)));
            try triangles.append(allocator, .init(v[0], v[1], v[2], .{ .material_index = rng.next() % 4, .user_data = rng.next() }));
        }

        try zolt.indexify(allocator, triangles.items, &vertices, &indexed, .{ .vertex_weld_distance = weld });

        try expected_vertices.resize(allocator, 3 * num_triangles);
        try expected_triangles.resize(allocator, num_triangles);
        var num_expected_vertices: u32 = undefined;
        var num_expected_triangles: u32 = undefined;
        jolt.jolt_indexify(triangles.items.ptr, @intCast(num_triangles), weld, expected_vertices.items.ptr, &num_expected_vertices, expected_triangles.items.ptr, &num_expected_triangles);
        const input = .{ triangles.items, weld };
        checkSlices(&indexify_vertices, input, Float3, vertices.items, expected_vertices.items[0..num_expected_vertices]);
        checkSlices(&indexify_triangles, input, IndexedTriangle, indexed.items, expected_triangles.items[0..num_expected_triangles]);

        // Deindexify the result and random indexed triangles
        if (rng.chance(2) and vertices.items.len > 0) {
            for (indexed.items) |*t| {
                if (rng.chance(4)) t.idx[rng.index(3)] = rng.next() % @as(u32, @intCast(vertices.items.len));
            }
        }
        try zolt.deindexify(allocator, vertices.items, indexed.items, &deindexified);
        try expected_deindexified.resize(allocator, indexed.items.len);
        jolt.jolt_deindexify(vertices.items.ptr, @intCast(vertices.items.len), indexed.items.ptr, @intCast(indexed.items.len), expected_deindexified.items.ptr);
        checkSlices(&deindexify, .{ vertices.items, indexed.items }, Triangle, deindexified.items, expected_deindexified.items);
    }
    try finishAll(&.{ &indexify_vertices, &indexify_triangles, &deindexify });
}

test "Ellipse" {
    var rng: Rng = .{};
    var is_inside: Checker = .{ .name = "Ellipse.isInside" };
    var get_closest_point: Checker = .{ .name = "Ellipse.getClosestPoint" };
    var get_normal: Checker = .{ .name = "Ellipse.getNormal" };
    for (0..iterations) |_| {
        var expected: [2]f32 = undefined;

        const a = rng.float(0.1, 10);
        const b = if (rng.chance(5)) a else rng.float(0.1, 10);
        if (a <= 0 or b <= 0) continue;
        const e = Ellipse.init(a, b);

        // Point: random, on an axis, on the ellipse
        var p = rng.array(2, -20, 20);
        switch (rng.next() % 5) {
            0 => p[rng.index(2)] = 0,
            1 => {
                const angle = rng.float(0, 2 * zolt.math.pi);
                const sc = Vec4.replicate(angle).sinCos();
                p = .{ a * sc.cos.getX(), b * sc.sin.getX() };
            },
            else => {},
        }
        const point = Float2.init(p[0], p[1]);
        is_inside.check(.{ a, b, p }, e.isInside(point), jolt.jolt_ellipse_is_inside(a, b, &p));

        jolt.jolt_ellipse_get_normal(a, b, &p, &expected);
        const n = e.getNormal(point);
        get_normal.check(.{ a, b, p }, [2]f32{ n.x, n.y }, expected);

        // The closest point needs a point outside the ellipse
        if (!e.isInside(point)) {
            jolt.jolt_ellipse_get_closest_point(a, b, &p, &expected);
            const c = e.getClosestPoint(point);
            get_closest_point.check(.{ a, b, p }, [2]f32{ c.x, c.y }, expected);
        }
    }
    try finishAll(&.{ &is_inside, &get_closest_point, &get_normal });
}
