//! Parity tests for the Geometry queries (ClosestPoint, RayAABox, RayTriangle, RaySphere, RayCylinder, RayCapsule,
//! ConvexSupport, ConvexHullBuilder2D): run Zolt and the C++ Jolt library on the same inputs and require identical
//! bits. C ABI wrappers: ZoltParity/Geometry/QueriesReference.cpp. See ZoltParity/parity.zig for how parity tests work.
//!
//! The inputs are random values mixed with constructed edge cases: the origin exactly on a vertex / edge / face,
//! degenerate (coincident, colinear, flat) triangles and tetrahedra, small integer coordinates (exact ties), rays
//! with zero / tiny / huge direction components, rays parallel to faces, starting inside or on the surface, grazing
//! rays and invalid boxes.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const ClosestPoint = zolt.ClosestPoint;
const ConvexHullBuilder2D = zolt.ConvexHullBuilder2D;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const RayInvDirection = zolt.RayInvDirection;
const StaticArray = zolt.StaticArray;
const UVec4 = zolt.UVec4;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

/// The C++ reference functions, see QueriesReference.cpp
const jolt = struct {
    extern fn jolt_closest_point_bary_line(a: *const P, b: *const P, out_uv: *[2]f32) bool;
    extern fn jolt_closest_point_bary_triangle(a: *const P, b: *const P, c: *const P, out_uvw: *[3]f32) bool;
    extern fn jolt_closest_point_on_line(a: *const P, b: *const P, out_point: *P) u32;
    extern fn jolt_closest_point_on_triangle(a: *const P, b: *const P, c: *const P, must_include_c: bool, out_point: *P) u32;
    extern fn jolt_origin_outside_of_plane(a: *const P, b: *const P, c: *const P, d: *const P) bool;
    extern fn jolt_origin_outside_of_tetrahedron_planes(a: *const P, b: *const P, c: *const P, d: *const P, out_mask: *[4]u32) void;
    extern fn jolt_closest_point_on_tetrahedron(a: *const P, b: *const P, c: *const P, d: *const P, must_include_d: bool, out_point: *P) u32;

    extern fn jolt_ray_inv_direction(direction: *const P, out_inv_direction: *[4]f32, out_is_parallel: *[4]u32) void;
    extern fn jolt_ray_aabox(origin: *const P, direction: *const P, min: *const P, max: *const P) f32;
    extern fn jolt_ray_aabox4(origin: *const P, direction: *const P, bounds: *const [24]f32, out_fractions: *[4]f32) void;
    extern fn jolt_ray_aabox_min_max(origin: *const P, direction: *const P, min: *const P, max: *const P, out_min_max: *[2]f32) void;
    extern fn jolt_ray_aabox_hits(origin: *const P, direction: *const P, min: *const P, max: *const P, closest: f32) bool;
    extern fn jolt_ray_aabox_hits_direction(origin: *const P, direction: *const P, min: *const P, max: *const P) bool;
    extern fn jolt_ray_triangle(origin: *const P, direction: *const P, v0: *const P, v1: *const P, v2: *const P) f32;
    extern fn jolt_ray_triangle4(origin: *const P, direction: *const P, v: *const [36]f32, out_fractions: *[4]f32) void;
    extern fn jolt_ray_sphere(origin: *const P, direction: *const P, center: *const P, radius: f32) f32;
    extern fn jolt_ray_sphere_min_max(origin: *const P, direction: *const P, center: *const P, radius: f32, out_min_max: *[2]f32) c_int;
    extern fn jolt_ray_infinite_cylinder(origin: *const P, direction: *const P, radius: f32) f32;
    extern fn jolt_ray_cylinder(origin: *const P, direction: *const P, half_height: f32, radius: f32) f32;
    extern fn jolt_ray_capsule(origin: *const P, direction: *const P, half_height: f32, radius: f32) f32;

    extern fn jolt_convex_support(transform: *const [16]f32, point: *const P, triangle: *const [9]f32, polygon: [*]const f32, polygon_count: c_int, radius: f32, direction: *const P, out_supports: *[num_supports][3]f32, out_faces: *[num_faces][32][3]f32, out_face_counts: *[num_faces]c_int) void;

    extern fn jolt_convex_hull_builder_2d(positions: [*]const f32, count: c_int, idx1: c_int, idx2: c_int, idx3: c_int, max_vertices: c_int, tolerance: f32, out_edges: [*]c_int, out_num_edges: *c_int) c_int;
};

/// Number of random inputs per test
const iterations = fw.iterations;

/// A point / vector as passed to the C ABI
const P = [3]f32;

fn vec3(a: P) Vec3 {
    return Vec3.init(a[0], a[1], a[2]);
}

fn arr3(v: Vec3) P {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn vec4(a: [4]f32) Vec4 {
    return Vec4.init(a[0], a[1], a[2], a[3]);
}

fn add(a: P, b: P) P {
    return .{ a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}

fn sub(a: P, b: P) P {
    return .{ a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}

fn mulS(a: P, s: f32) P {
    return .{ a[0] * s, a[1] * s, a[2] * s };
}

fn lerp(a: P, b: P, t: f32) P {
    return add(a, mulS(sub(b, a), t));
}

fn cross(a: P, b: P) P {
    return .{ a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0] };
}

/// Values that tend to expose differences, mixed into the random inputs
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 1.0e-6, -1.0e-6, 1.0e-20, -1.0e-20, 1.0e-30, 100.0, -100.0 };

/// Components for ray directions around the "parallel" threshold of RayInvDirection (1.0e-20) and other tiny values
const tiny_components = [_]f32{ 0.0, -0.0, 1.0e-20, -1.0e-20, 1.0000001e-20, -9.999999e-21, 1.0e-21, -1.0e-21, 1.0e-19, -1.0e-19, 1.0e-40, -1.0e-40, 1.0e-10, -1.0e-7 };

/// Input generator: xorshift32 with helpers for the geometric edge cases
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

    /// Random float in [min, max), or one of the special values (10% of the time)
    fn float(self: *Gen, min: f32, max: f32) f32 {
        if (self.oneIn(10)) {
            const v = special_values[self.index(special_values.len)];
            if (v >= min and v <= max) return v;
        }
        return self.rng.float(min, max);
    }

    fn vec(self: *Gen, min: f32, max: f32) P {
        return .{ self.float(min, max), self.float(min, max), self.float(min, max) };
    }

    /// Integer in [-n, n] as float
    fn grid(self: *Gen, n: i32) f32 {
        return @floatFromInt(self.rng.intRange(i32, -n, n));
    }

    fn gridVec(self: *Gen, n: i32) P {
        return .{ self.grid(n), self.grid(n), self.grid(n) };
    }

    /// Random power of 10 in [1e-6, 1e6]
    fn scale(self: *Gen) f32 {
        const scales = [_]f32{ 1.0e-6, 1.0e-5, 1.0e-4, 1.0e-3, 1.0e-2, 0.1, 1.0, 10.0, 100.0, 1.0e3, 1.0e4, 1.0e5, 1.0e6 };
        return scales[self.index(scales.len)];
    }

    /// Random barycentric coordinates (sum to 1)
    fn barycentric(self: *Gen) [3]f32 {
        const u = self.rng.float(0, 1);
        const v = self.rng.float(0, 1 - u);
        return .{ u, v, 1 - u - v };
    }

    /// Random unit vector
    fn unit(self: *Gen) P {
        while (true) {
            const v = self.rng.floatArray(3, -1, 1);
            const len_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            if (len_sq > 1.0e-4 and len_sq <= 1.0) return mulS(v, 1.0 / @sqrt(len_sq));
        }
    }

    /// Triangle relative to the origin (the closest point queries look for the point closest to the origin)
    fn triangle(self: *Gen) [3]P {
        var t: [3]P = .{ self.vec(-1, 1), self.vec(-1, 1), self.vec(-1, 1) };
        switch (self.next() % 14) {
            0, 1 => {},
            // Origin exactly on a vertex
            2 => {
                const o = t[self.index(3)];
                for (&t) |*v| v.* = sub(v.*, o);
            },
            // Origin (approximately) on an edge
            3 => {
                const i = self.index(3);
                const tt: f32 = if (self.oneIn(2)) 0.5 else self.rng.float(0, 1);
                const o = lerp(t[i], t[(i + 1) % 3], tt);
                for (&t) |*v| v.* = sub(v.*, o);
            },
            // Origin (approximately) in the interior
            4 => {
                const b = self.barycentric();
                const o = add(add(mulS(t[0], b[0]), mulS(t[1], b[1])), mulS(t[2], b[2]));
                for (&t) |*v| v.* = sub(v.*, o);
            },
            // Colinear
            5 => t[self.index(3)] = lerp(t[0], t[1], self.float(-1, 2)),
            // Coincident vertices
            6 => {
                const i = self.index(3);
                t[(i + 1) % 3] = t[i];
                if (self.oneIn(3)) t[(i + 2) % 3] = t[i];
            },
            // Nearly colinear
            7 => {
                const i = self.index(3);
                const eps = [_]f32{ 1.0e-3, 1.0e-4, 1.0e-5, 1.0e-6, 1.0e-7 };
                t[i] = add(lerp(t[(i + 1) % 3], t[(i + 2) % 3], self.rng.float(-0.5, 1.5)), mulS(self.unit(), eps[self.index(eps.len)]));
            },
            // Small integer coordinates (exact ties)
            8 => t = .{ self.gridVec(2), self.gridVec(2), self.gridVec(2) },
            // Scaled
            9 => {
                const s = self.scale();
                for (&t) |*v| v.* = mulS(v.*, s);
            },
            // Far away
            10 => {
                const o = mulS(self.unit(), self.rng.float(10, 1000));
                for (&t) |*v| v.* = add(v.*, o);
            },
            // Axis aligned triangle in a plane through or near the origin, integer coordinates
            11 => {
                const axis = self.index(3);
                const offset = self.grid(1);
                for (&t) |*v| {
                    v.* = self.gridVec(3);
                    v[axis] = offset;
                }
            },
            // Long thin triangle
            12 => {
                const d = mulS(self.unit(), self.rng.float(10, 200));
                const o = self.vec(-1, 1);
                t = .{ add(o, d), sub(o, d), add(add(o, d), mulS(self.unit(), self.rng.float(1.0e-3, 1))) };
            },
            // Tiny triangle near the origin
            else => {
                for (&t) |*v| v.* = add(mulS(v.*, 1.0e-4), mulS(self.unit(), 1.0e-3));
            },
        }
        return t;
    }

    /// Tetrahedron relative to the origin
    fn tetrahedron(self: *Gen) [4]P {
        var t: [4]P = .{ self.vec(-1, 1), self.vec(-1, 1), self.vec(-1, 1), self.vec(-1, 1) };
        switch (self.next() % 13) {
            0, 1 => {},
            // Origin exactly on a vertex
            2 => {
                const o = t[self.index(4)];
                for (&t) |*v| v.* = sub(v.*, o);
            },
            // Origin (approximately) on an edge
            3 => {
                const i = self.index(4);
                const o = lerp(t[i], t[(i + 1 + self.index(3)) % 4], self.rng.float(0, 1));
                for (&t) |*v| v.* = sub(v.*, o);
            },
            // Origin (approximately) on a face
            4 => {
                const skip = self.index(4);
                const b = self.barycentric();
                var o: P = .{ 0, 0, 0 };
                var k: usize = 0;
                for (t, 0..) |v, i| {
                    if (i == skip) continue;
                    o = add(o, mulS(v, b[k]));
                    k += 1;
                }
                for (&t) |*v| v.* = sub(v.*, o);
            },
            // Origin inside
            5 => {
                const b = self.barycentric();
                const w = self.rng.float(0, 1);
                const o = add(mulS(add(add(mulS(t[0], b[0]), mulS(t[1], b[1])), mulS(t[2], b[2])), 1 - w), mulS(t[3], w));
                for (&t) |*v| v.* = sub(v.*, o);
            },
            // Flat: d in the plane of abc
            6 => {
                const i = self.index(4);
                t[i] = add(t[(i + 1) % 4], add(mulS(sub(t[(i + 2) % 4], t[(i + 1) % 4]), self.float(-1, 2)), mulS(sub(t[(i + 3) % 4], t[(i + 1) % 4]), self.float(-1, 2))));
            },
            // Colinear / coincident points
            7 => {
                const i = self.index(4);
                t[i] = lerp(t[(i + 1) % 4], t[(i + 2) % 4], self.float(-1, 2));
                if (self.oneIn(2)) t[(i + 3) % 4] = t[(i + 1) % 4];
            },
            // Small integer coordinates
            8 => t = .{ self.gridVec(2), self.gridVec(2), self.gridVec(2), self.gridVec(2) },
            // Scaled
            9 => {
                const s = self.scale();
                for (&t) |*v| v.* = mulS(v.*, s);
            },
            // GJK like: small triangle abc close to the origin and a new support point d further away
            10 => {
                const n = self.unit();
                for (t[0..3]) |*v| v.* = add(mulS(v.*, 0.1), mulS(n, self.rng.float(-0.05, 0.2)));
                t[3] = mulS(n, -self.rng.float(0.1, 2));
            },
            // Far away
            11 => {
                const o = mulS(self.unit(), self.rng.float(5, 500));
                for (&t) |*v| v.* = add(v.*, o);
            },
            // Regular tetrahedron with random orientation around a point near the origin
            else => {
                const c = self.vec(-0.5, 0.5);
                const e1 = self.unit();
                const e2 = cross(e1, self.unit());
                const e3 = cross(e1, e2);
                t = .{ add(c, e1), add(c, mulS(e1, -1)), add(c, e2), add(c, e3) };
            },
        }
        return t;
    }

    /// Ray direction
    fn direction(self: *Gen) P {
        switch (self.next() % 12) {
            0, 1, 2, 3 => return self.vec(-1, 1),
            // Axis aligned
            4 => {
                var d: P = .{ if (self.oneIn(2)) 0.0 else -0.0, if (self.oneIn(2)) 0.0 else -0.0, if (self.oneIn(2)) 0.0 else -0.0 };
                d[self.index(3)] = self.float(-2, 2);
                return d;
            },
            // One component zero
            5 => {
                var d = self.vec(-1, 1);
                d[self.index(3)] = if (self.oneIn(2)) 0.0 else -0.0;
                return d;
            },
            // Tiny components
            6 => {
                var d = self.vec(-1, 1);
                for (&d) |*c| {
                    if (!self.oneIn(3)) c.* = tiny_components[self.index(tiny_components.len)];
                }
                return d;
            },
            // Huge
            7 => return mulS(self.vec(-1, 1), if (self.oneIn(2)) 1.0e20 else 1.0e30),
            // Zero
            8 => return .{ if (self.oneIn(2)) 0.0 else -0.0, 0.0, if (self.oneIn(2)) 0.0 else -0.0 },
            // Small integers
            9 => return self.gridVec(2),
            // Scaled
            10 => return mulS(self.vec(-1, 1), self.scale()),
            else => return self.vec(-1.0e-3, 1.0e-3),
        }
    }

    const Box = struct { min: P, max: P };

    fn box(self: *Gen) Box {
        switch (self.next() % 12) {
            // Integer coordinates (zero extents possible)
            0 => {
                const min = self.gridVec(2);
                return .{ .min = min, .max = add(min, .{ @abs(self.grid(2)), @abs(self.grid(2)), @abs(self.grid(2)) }) };
            },
            // Flat box
            1 => {
                const min = self.vec(-2, 2);
                var max = add(min, self.vec(0, 3));
                const axis = self.index(3);
                max[axis] = min[axis];
                return .{ .min = min, .max = max };
            },
            // Invalid on some axes
            2 => {
                var min = self.vec(-2, 2);
                var max = add(min, self.vec(0, 3));
                const axis = self.index(3);
                std.mem.swap(f32, &min[axis], &max[axis]);
                return .{ .min = min, .max = max };
            },
            // Default constructed (invalid) AABox
            3 => return .{ .min = @splat(std.math.floatMax(f32)), .max = @splat(-std.math.floatMax(f32)) },
            // Huge
            4 => return .{ .min = mulS(self.vec(0.5, 1), -1.0e30), .max = mulS(self.vec(0.5, 1), 1.0e30) },
            // Scaled
            5 => {
                const s = self.scale();
                const min = mulS(self.vec(-2, 2), s);
                return .{ .min = min, .max = add(min, mulS(self.vec(0, 3), s)) };
            },
            else => {
                const min = self.vec(-2, 2);
                return .{ .min = min, .max = add(min, self.vec(0, 3)) };
            },
        }
    }

    /// Point on the surface of the box (on a face, an edge or a corner)
    fn pointOnBox(self: *Gen, b: Box) P {
        var p: P = undefined;
        for (&p, b.min, b.max) |*c, min, max| c.* = if (self.oneIn(3)) min else if (self.oneIn(2)) max else min + (max - min) * self.rng.float(0, 1);
        const axis = self.index(3);
        p[axis] = if (self.oneIn(2)) b.min[axis] else b.max[axis];
        return p;
    }

    /// Ray origin relative to a box
    fn originForBox(self: *Gen, b: Box) P {
        switch (self.next() % 10) {
            // Inside
            0 => {
                var p: P = undefined;
                for (&p, b.min, b.max) |*c, min, max| c.* = min + (max - min) * self.rng.float(0, 1);
                return p;
            },
            // On the surface
            1, 2 => return self.pointOnBox(b),
            3 => return self.gridVec(3),
            4 => return mulS(self.vec(-1, 1), 100),
            else => return self.vec(-4, 4),
        }
    }

    /// Ray direction for a ray starting at origin towards a box
    fn directionForBox(self: *Gen, b: Box, origin: P) P {
        if (self.oneIn(5))
            return sub(self.pointOnBox(b), origin); // Through a face, an edge or a corner
        return self.direction();
    }
};

/// Result of the closest point functions in a form that `sameValue` can compare
const PointAndSet = struct {
    point: P,
    set: u32,

    fn fromZolt(r: ClosestPoint.PointAndSet) PointAndSet {
        return .{ .point = arr3(r.point), .set = r.set };
    }
};

fn checkTriangle(t: [3]P, bary_line: *Checker, bary_triangle: *Checker, on_line: *Checker, on_triangle: *Checker, on_triangle_c: *Checker) void {
    const a = vec3(t[0]);
    const b = vec3(t[1]);
    const c = vec3(t[2]);

    // GetBaryCentricCoordinates (line)
    var uv: [2]f32 = undefined;
    const line_valid = jolt.jolt_closest_point_bary_line(&t[0], &t[1], &uv);
    bary_line.check(.{ t[0], t[1] }, ClosestPoint.getBaryCentricCoordinates(a, b), .{ .u = uv[0], .v = uv[1], .valid = line_valid });

    // GetBaryCentricCoordinates (triangle)
    var uvw: [3]f32 = undefined;
    const triangle_valid = jolt.jolt_closest_point_bary_triangle(&t[0], &t[1], &t[2], &uvw);
    bary_triangle.check(t, ClosestPoint.getBaryCentricCoordinatesTriangle(a, b, c), .{ .u = uvw[0], .v = uvw[1], .w = uvw[2], .valid = triangle_valid });

    // GetClosestPointOnLine
    var point: P = undefined;
    var set = jolt.jolt_closest_point_on_line(&t[0], &t[1], &point);
    on_line.check(.{ t[0], t[1] }, PointAndSet.fromZolt(ClosestPoint.getClosestPointOnLine(a, b)), .{ .point = point, .set = set });

    // GetClosestPointOnTriangle<false / true>
    set = jolt.jolt_closest_point_on_triangle(&t[0], &t[1], &t[2], false, &point);
    on_triangle.check(t, PointAndSet.fromZolt(ClosestPoint.getClosestPointOnTriangle(a, b, c, .{})), .{ .point = point, .set = set });
    set = jolt.jolt_closest_point_on_triangle(&t[0], &t[1], &t[2], true, &point);
    on_triangle_c.check(t, PointAndSet.fromZolt(ClosestPoint.getClosestPointOnTriangle(a, b, c, .{ .must_include_c = true })), .{ .point = point, .set = set });
}

test "ClosestPoint line / triangle" {
    var gen: Gen = .{};
    var bary_line: Checker = .{ .name = "ClosestPoint.getBaryCentricCoordinates" };
    var bary_triangle: Checker = .{ .name = "ClosestPoint.getBaryCentricCoordinatesTriangle" };
    var on_line: Checker = .{ .name = "ClosestPoint.getClosestPointOnLine" };
    var on_triangle: Checker = .{ .name = "ClosestPoint.getClosestPointOnTriangle" };
    var on_triangle_c: Checker = .{ .name = "ClosestPoint.getClosestPointOnTriangle(must_include_c)" };

    // Hand picked cases: the unit test triangles, exact zero / tiny edges and degenerate triangles
    const hand_picked = [_][3]P{
        .{ .{ 100, 1, 0 }, .{ 100, 1, 1 }, .{ -100, 1, 0 } },
        .{ .{ 99.9999847, 0.946687222, 99.9999847 }, .{ -100.010002, 0.977360725, -100.010002 }, .{ -100.000137, 0.977310658, -100.000137 } },
        .{ .{ -0.132395342, -0.294095188, -0.164812326 }, .{ -0.126054004, -0.283950001, -0.159065604 }, .{ -0.154956535, -0.284792334, -0.160523415 } },
        .{ .{ 0, 0, 0 }, .{ 0, 0, 0 }, .{ 0, 0, 0 } },
        .{ .{ 1, 2, 3 }, .{ 1, 2, 3 }, .{ 1, 2, 3 } },
        .{ .{ 1, 0, 0 }, .{ 1, 1.0e-8, 0 }, .{ 1, 0, 1.0e-8 } },
        .{ .{ 1, 1, 0 }, .{ 2, 1, 0 }, .{ 3, 1, 0 } },
        .{ .{ 1, 0, 0 }, .{ 0, 0, 0 }, .{ 1.5, 0, 0 } },
        .{ .{ 1, 0, 0 }, .{ 3, 0, 0 }, .{ 2, 0, 0 } },
        .{ .{ 2, 0, 0 }, .{ 5, 0, 0 }, .{ 4, 0, 0 } },
        .{ .{ -1, -1, 1 }, .{ 2, -1, 1 }, .{ -1, 2, 1 } },
        .{ .{ -1, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 } },
        .{ .{ -1, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 0, 0 } },
        .{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 } },
        .{ .{ 1.0e-20, 0, 0 }, .{ 0, 1.0e-20, 0 }, .{ 0, 0, 1.0e-20 } },
        .{ .{ 1.0e19, 0, 0 }, .{ 0, 1.0e19, 0 }, .{ 0, 0, 1.0e19 } },
    };
    for (hand_picked) |t| {
        // All permutations
        const permutations = [_][3]usize{ .{ 0, 1, 2 }, .{ 0, 2, 1 }, .{ 1, 0, 2 }, .{ 1, 2, 0 }, .{ 2, 0, 1 }, .{ 2, 1, 0 } };
        for (permutations) |p|
            checkTriangle(.{ t[p[0]], t[p[1]], t[p[2]] }, &bary_line, &bary_triangle, &on_line, &on_triangle, &on_triangle_c);
    }

    for (0..iterations) |_|
        checkTriangle(gen.triangle(), &bary_line, &bary_triangle, &on_line, &on_triangle, &on_triangle_c);

    try finishAll(&.{ &bary_line, &bary_triangle, &on_line, &on_triangle, &on_triangle_c });
}

fn checkTetrahedron(t: [4]P, outside_plane: *Checker, outside_planes: *Checker, on_tetrahedron: *Checker, on_tetrahedron_d: *Checker) void {
    const a = vec3(t[0]);
    const b = vec3(t[1]);
    const c = vec3(t[2]);
    const d = vec3(t[3]);

    // OriginOutsideOfPlane
    outside_plane.check(t, ClosestPoint.originOutsideOfPlane(a, b, c, d), jolt.jolt_origin_outside_of_plane(&t[0], &t[1], &t[2], &t[3]));

    // OriginOutsideOfTetrahedronPlanes
    var mask: [4]u32 = undefined;
    jolt.jolt_origin_outside_of_tetrahedron_planes(&t[0], &t[1], &t[2], &t[3], &mask);
    const zolt_mask: [4]u32 = ClosestPoint.originOutsideOfTetrahedronPlanes(a, b, c, d).value;
    outside_planes.check(t, zolt_mask, mask);

    // GetClosestPointOnTetrahedron<false / true>
    var point: P = undefined;
    var set = jolt.jolt_closest_point_on_tetrahedron(&t[0], &t[1], &t[2], &t[3], false, &point);
    on_tetrahedron.check(t, PointAndSet.fromZolt(ClosestPoint.getClosestPointOnTetrahedron(a, b, c, d, .{})), .{ .point = point, .set = set });
    set = jolt.jolt_closest_point_on_tetrahedron(&t[0], &t[1], &t[2], &t[3], true, &point);
    on_tetrahedron_d.check(t, PointAndSet.fromZolt(ClosestPoint.getClosestPointOnTetrahedron(a, b, c, d, .{ .must_include_d = true })), .{ .point = point, .set = set });
}

test "ClosestPoint tetrahedron" {
    var gen: Gen = .{};
    var outside_plane: Checker = .{ .name = "ClosestPoint.originOutsideOfPlane" };
    var outside_planes: Checker = .{ .name = "ClosestPoint.originOutsideOfTetrahedronPlanes" };
    var on_tetrahedron: Checker = .{ .name = "ClosestPoint.getClosestPointOnTetrahedron" };
    var on_tetrahedron_d: Checker = .{ .name = "ClosestPoint.getClosestPointOnTetrahedron(must_include_d)" };

    const hand_picked = [_][4]P{
        .{ .{ -1, -1, -1 }, .{ 3, -1, -1 }, .{ -1, 3, -1 }, .{ -1, -1, 3 } },
        .{ .{ -1, -1, 1 }, .{ 3, -1, 1 }, .{ -1, 3, 1 }, .{ -1, -1, 5 } },
        .{ .{ 0, 0, -5 }, .{ 4, 0, -5 }, .{ 0, 4, -5 }, .{ 0, 0, -1 } },
        .{ .{ -1, -1, -1 }, .{ 3, -1, -1 }, .{ -1, 3, -1 }, .{ 1, 1, -1 } },
        .{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 } },
        .{ .{ 0, 0, 0 }, .{ 0, 0, 0 }, .{ 0, 0, 0 }, .{ 0, 0, 0 } },
        .{ .{ 1, 1, 1 }, .{ 1, 1, 1 }, .{ 2, 1, 1 }, .{ 1, 2, 1 } },
        .{ .{ 1, 0, 0 }, .{ 2, 0, 0 }, .{ 3, 0, 0 }, .{ 4, 0, 0 } },
        .{ .{ -1, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 } },
    };
    for (hand_picked) |t| {
        // All even permutations and the odd ones (inverted winding)
        const permutations = [_][4]usize{ .{ 0, 1, 2, 3 }, .{ 1, 0, 2, 3 }, .{ 3, 1, 2, 0 }, .{ 0, 3, 2, 1 }, .{ 2, 3, 0, 1 }, .{ 1, 2, 3, 0 } };
        for (permutations) |p|
            checkTetrahedron(.{ t[p[0]], t[p[1]], t[p[2]], t[p[3]] }, &outside_plane, &outside_planes, &on_tetrahedron, &on_tetrahedron_d);
    }

    for (0..iterations) |_|
        checkTetrahedron(gen.tetrahedron(), &outside_plane, &outside_planes, &on_tetrahedron, &on_tetrahedron_d);

    try finishAll(&.{ &outside_plane, &outside_planes, &on_tetrahedron, &on_tetrahedron_d });
}

const RayInvDirectionResult = struct { inv_direction: [4]f32, is_parallel: [4]u32 };

test "RayInvDirection / RayAABox" {
    var gen: Gen = .{};
    var inv_direction: Checker = .{ .name = "RayInvDirection" };
    var ray_aabox: Checker = .{ .name = "rayAABox" };
    var ray_aabox_min_max: Checker = .{ .name = "rayAABoxMinMax" };
    var ray_aabox_hits: Checker = .{ .name = "rayAABoxHits" };
    var ray_aabox_hits_direction: Checker = .{ .name = "rayAABoxHitsDirection" };
    var ray_aabox4: Checker = .{ .name = "rayAABox4" };

    for (0..iterations) |_| {
        const box = gen.box();
        const origin = gen.originForBox(box);
        const direction = gen.directionForBox(box, origin);
        const inv = RayInvDirection.init(vec3(direction));
        const input = .{ origin, direction, box.min, box.max };

        // RayInvDirection
        var expected_inv: RayInvDirectionResult = undefined;
        jolt.jolt_ray_inv_direction(&direction, &expected_inv.inv_direction, &expected_inv.is_parallel);
        inv_direction.check(direction, RayInvDirectionResult{ .inv_direction = inv.inv_direction.value, .is_parallel = inv.is_parallel.value }, expected_inv);

        // RayAABox
        ray_aabox.check(input, zolt.rayAABox(vec3(origin), inv, vec3(box.min), vec3(box.max)), jolt.jolt_ray_aabox(&origin, &direction, &box.min, &box.max));

        // RayAABox (min / max)
        var min_max: [2]f32 = undefined;
        jolt.jolt_ray_aabox_min_max(&origin, &direction, &box.min, &box.max, &min_max);
        const zolt_min_max = zolt.rayAABoxMinMax(vec3(origin), inv, vec3(box.min), vec3(box.max));
        ray_aabox_min_max.check(input, [2]f32{ zolt_min_max.min, zolt_min_max.max }, min_max);

        // RayAABoxHits
        const closests = [_]f32{ std.math.floatMax(f32), 0, min_max[0], gen.float(-1, 10) };
        const closest = closests[gen.index(closests.len)];
        ray_aabox_hits.check(.{ input, closest }, zolt.rayAABoxHits(vec3(origin), inv, vec3(box.min), vec3(box.max), closest), jolt.jolt_ray_aabox_hits(&origin, &direction, &box.min, &box.max, closest));

        // RayAABoxHits (direction)
        ray_aabox_hits_direction.check(input, zolt.rayAABoxHitsDirection(vec3(origin), vec3(direction), vec3(box.min), vec3(box.max)), jolt.jolt_ray_aabox_hits_direction(&origin, &direction, &box.min, &box.max));

        // RayAABox4: this box and 3 more
        var bounds: [24]f32 = undefined;
        for (0..4) |i| {
            const b = if (i == 0) box else gen.box();
            for (0..3) |axis| {
                bounds[4 * axis + i] = b.min[axis];
                bounds[12 + 4 * axis + i] = b.max[axis];
            }
        }
        var fractions: [4]f32 = undefined;
        jolt.jolt_ray_aabox4(&origin, &direction, &bounds, &fractions);
        const zolt_fractions: [4]f32 = zolt.rayAABox4(vec3(origin), inv, vec4(bounds[0..4].*), vec4(bounds[4..8].*), vec4(bounds[8..12].*), vec4(bounds[12..16].*), vec4(bounds[16..20].*), vec4(bounds[20..24].*)).value;
        ray_aabox4.check(.{ origin, direction, bounds }, zolt_fractions, fractions);
    }

    try finishAll(&.{ &inv_direction, &ray_aabox, &ray_aabox_min_max, &ray_aabox_hits, &ray_aabox_hits_direction, &ray_aabox4 });
}

/// A triangle somewhere in space and a ray origin and direction for it
const RayTriangleInput = struct { origin: P, direction: P, v: [3]P };

fn rayTriangleInput(gen: *Gen) RayTriangleInput {
    var v = gen.triangle();
    const offset = gen.vec(-2, 2);
    for (&v) |*p| p.* = add(p.*, offset);
    var origin = gen.vec(-3, 3);
    if (gen.oneIn(10)) {
        // Origin on the triangle
        const b = gen.barycentric();
        origin = add(add(mulS(v[0], b[0]), mulS(v[1], b[1])), mulS(v[2], b[2]));
    } else if (gen.oneIn(10)) {
        origin = gen.gridVec(3);
    }
    const target = switch (gen.next() % 8) {
        // Through a vertex
        0 => v[gen.index(3)],
        // Through an edge
        1 => lerp(v[gen.index(3)], v[gen.index(3)], gen.rng.float(0, 1)),
        // Through the interior
        2, 3 => blk: {
            const b = gen.barycentric();
            break :blk add(add(mulS(v[0], b[0]), mulS(v[1], b[1])), mulS(v[2], b[2]));
        },
        // Parallel to the plane of the triangle
        4 => add(origin, cross(cross(sub(v[1], v[0]), sub(v[2], v[0])), gen.vec(-1, 1))),
        else => add(origin, gen.direction()),
    };
    var direction = sub(target, origin);
    if (gen.oneIn(10)) direction = mulS(direction, -1); // Pointing away
    return .{ .origin = origin, .direction = direction, .v = v };
}

test "RayTriangle / RayTriangle4" {
    var gen: Gen = .{};
    var ray_triangle: Checker = .{ .name = "rayTriangle" };
    var ray_triangle4: Checker = .{ .name = "rayTriangle4" };

    for (0..iterations) |_| {
        const in = rayTriangleInput(&gen);
        ray_triangle.check(in, zolt.rayTriangle(vec3(in.origin), vec3(in.direction), vec3(in.v[0]), vec3(in.v[1]), vec3(in.v[2])), jolt.jolt_ray_triangle(&in.origin, &in.direction, &in.v[0], &in.v[1], &in.v[2]));

        // 4 triangles for the same ray: this one and 3 more
        var v: [36]f32 = undefined;
        for (0..4) |i| {
            const t = if (i == 0) in.v else rayTriangleInput(&gen).v;
            for (0..3) |vertex| {
                for (0..3) |axis|
                    v[12 * vertex + 4 * axis + i] = t[vertex][axis];
            }
        }
        var fractions: [4]f32 = undefined;
        jolt.jolt_ray_triangle4(&in.origin, &in.direction, &v, &fractions);
        const zolt_fractions: [4]f32 = zolt.rayTriangle4(vec3(in.origin), vec3(in.direction), vec4(v[0..4].*), vec4(v[4..8].*), vec4(v[8..12].*), vec4(v[12..16].*), vec4(v[16..20].*), vec4(v[20..24].*), vec4(v[24..28].*), vec4(v[28..32].*), vec4(v[32..36].*)).value;
        ray_triangle4.check(.{ in.origin, in.direction, v }, zolt_fractions, fractions);
    }

    try finishAll(&.{ &ray_triangle, &ray_triangle4 });
}

/// Ray for a shape of `radius` (and `half_height` along Y) centered around `center`
const RayShapeInput = struct { origin: P, direction: P, center: P, radius: f32, half_height: f32 };

fn rayShapeInput(gen: *Gen) RayShapeInput {
    const radii = [_]f32{ 0, 1.0e-6, 0.5, 1, 2 };
    const radius = if (gen.oneIn(4)) radii[gen.index(radii.len)] else gen.rng.float(0, 2);
    const half_height = if (gen.oneIn(4)) radii[gen.index(radii.len)] else gen.rng.float(0, 2);
    const center = if (gen.oneIn(3)) P{ 0, 0, 0 } else gen.vec(-1, 1);
    var origin = gen.vec(-3, 3);
    var direction = gen.direction();
    switch (gen.next() % 10) {
        // Origin on the sphere / cylinder surface
        0 => origin = add(center, mulS(gen.unit(), radius)),
        // Origin on the cylinder side
        1 => {
            const n = gen.unit();
            const len = @sqrt(n[0] * n[0] + n[2] * n[2]);
            if (len > 0) origin = .{ n[0] / len * radius, gen.float(-3, 3), n[2] / len * radius };
        },
        // Tangent ray (grazing the sphere)
        2 => {
            const n = gen.unit();
            const t = cross(n, gen.unit());
            origin = sub(add(center, mulS(n, radius)), mulS(t, gen.rng.float(0.5, 3)));
            direction = t;
        },
        // Tangent ray along the cylinder side
        3 => {
            origin = .{ radius, gen.float(-3, 3), -gen.rng.float(0.5, 3) };
            direction = .{ 0, gen.float(-1, 1), 1 };
        },
        // Origin on the cap plane, direction along the axis
        4 => {
            origin = .{ gen.float(-1, 1) * radius, if (gen.oneIn(2)) half_height else -half_height, gen.float(-1, 1) * radius };
            direction = .{ 0, gen.float(-2, 2), 0 };
        },
        // Origin on the axis
        5 => origin = .{ 0, gen.float(-3, 3), 0 },
        6 => origin = gen.gridVec(3),
        else => {},
    }
    return .{ .origin = origin, .direction = direction, .center = center, .radius = radius, .half_height = half_height };
}

test "RaySphere / RayCylinder / RayCapsule" {
    var gen: Gen = .{};
    var ray_sphere: Checker = .{ .name = "raySphere" };
    var ray_sphere_min_max: Checker = .{ .name = "raySphereMinMax" };
    var ray_infinite_cylinder: Checker = .{ .name = "rayInfiniteCylinder" };
    var ray_cylinder: Checker = .{ .name = "rayCylinder" };
    var ray_capsule: Checker = .{ .name = "rayCapsule" };

    for (0..iterations) |_| {
        const in = rayShapeInput(&gen);
        const origin = vec3(in.origin);
        const direction = vec3(in.direction);

        ray_sphere.check(in, zolt.raySphere(origin, direction, vec3(in.center), in.radius), jolt.jolt_ray_sphere(&in.origin, &in.direction, &in.center, in.radius));

        // The fractions are only written by Jolt when there is an intersection
        var min_max: [2]f32 = undefined;
        const num = jolt.jolt_ray_sphere_min_max(&in.origin, &in.direction, &in.center, in.radius, &min_max);
        const r = zolt.raySphereMinMax(origin, direction, vec3(in.center), in.radius);
        if (num == 0)
            ray_sphere_min_max.check(in, r.num_intersections, @as(i32, num))
        else
            ray_sphere_min_max.check(in, [3]f32{ @floatFromInt(r.num_intersections), r.min_fraction, r.max_fraction }, [3]f32{ @floatFromInt(num), min_max[0], min_max[1] });

        ray_infinite_cylinder.check(in, zolt.rayInfiniteCylinder(origin, direction, in.radius), jolt.jolt_ray_infinite_cylinder(&in.origin, &in.direction, in.radius));
        ray_cylinder.check(in, zolt.rayCylinder(origin, direction, in.half_height, in.radius), jolt.jolt_ray_cylinder(&in.origin, &in.direction, in.half_height, in.radius));
        ray_capsule.check(in, zolt.rayCapsule(origin, direction, in.half_height, in.radius), jolt.jolt_ray_capsule(&in.origin, &in.direction, in.half_height, in.radius));
    }

    try finishAll(&.{ &ray_sphere, &ray_sphere_min_max, &ray_infinite_cylinder, &ray_cylinder, &ray_capsule });
}

/// Number of support points and faces that jolt_convex_support computes
const num_supports = 13;
const num_faces = 5;

const ConvexSupportResult = struct {
    supports: [num_supports][3]f32,
    face_counts: [num_faces]i32,
    faces: [num_faces][32][3]f32,
};

/// Store a face in the result (unused entries stay zero)
fn storeFace(face: *const StaticArray(Vec3, 32), out_vertices: *[32][3]f32, out_count: *i32) void {
    out_count.* = @intCast(face.len);
    for (face.constSlice(), 0..) |v, i|
        out_vertices[i] = arr3(v);
}

/// The Zolt version of jolt_convex_support
fn convexSupport(transform: Mat44, point_in: P, triangle_in: [9]f32, polygon_in: []const P, radius: f32, direction: Vec3) !ConvexSupportResult {
    const Polygon = zolt.PolygonConvexSupport(std.ArrayList(Vec3));
    const PolygonFace = zolt.PolygonConvexSupport(StaticArray(Vec3, 32));
    const Triangle = zolt.TriangleConvexSupport;
    const Point = zolt.PointConvexSupport;

    const point: Point = .{ .point = vec3(point_in) };
    const triangle = Triangle.init(vec3(triangle_in[0..3].*), vec3(triangle_in[3..6].*), vec3(triangle_in[6..9].*));
    var polygon_vertices: std.ArrayList(Vec3) = .empty;
    defer polygon_vertices.deinit(std.testing.allocator);
    for (polygon_in) |v|
        try polygon_vertices.append(std.testing.allocator, vec3(v));
    const polygon = Polygon.init(&polygon_vertices);
    var polygon_face_vertices: StaticArray(Vec3, 32) = .empty;
    for (polygon_in) |v|
        polygon_face_vertices.append(vec3(v));
    const polygon_face = PolygonFace.init(&polygon_face_vertices);

    const add_radius_triangle = zolt.AddConvexRadius(Triangle).init(&triangle, radius);
    const add_radius_polygon = zolt.AddConvexRadius(Polygon).init(&polygon, radius);
    const triangle_minus_polygon = zolt.MinkowskiDifference(Triangle, Polygon).init(&triangle, &polygon);
    const rounded_polygon_minus_point = zolt.MinkowskiDifference(zolt.AddConvexRadius(Polygon), Point).init(&add_radius_polygon, &point);
    const transformed_triangle = zolt.TransformedConvexObject(Triangle).init(transform, &triangle);
    const transformed_rounded_polygon = zolt.TransformedConvexObject(zolt.AddConvexRadius(Polygon)).init(transform, &add_radius_polygon);
    const transformed_rounded_triangle = zolt.TransformedConvexObject(zolt.AddConvexRadius(Triangle)).init(transform, &add_radius_triangle);
    const epa_like = zolt.MinkowskiDifference(@TypeOf(transformed_rounded_triangle), zolt.AddConvexRadius(Polygon)).init(&transformed_rounded_triangle, &add_radius_polygon);
    const transformed_difference = zolt.TransformedConvexObject(@TypeOf(triangle_minus_polygon)).init(transform, &triangle_minus_polygon);
    const transformed_point = zolt.TransformedConvexObject(Point).init(transform, &point);
    const gjk_cast_like = zolt.MinkowskiDifference(Polygon, @TypeOf(transformed_triangle)).init(&polygon, &transformed_triangle);
    const transformed_polygon = zolt.TransformedConvexObject(Polygon).init(transform, &polygon);
    const transformed_polygon_face = zolt.TransformedConvexObject(PolygonFace).init(transform, &polygon_face);

    const supports = [num_supports]Vec3{
        point.getSupport(direction),
        triangle.getSupport(direction),
        polygon.getSupport(direction),
        add_radius_triangle.getSupport(direction),
        add_radius_polygon.getSupport(direction),
        triangle_minus_polygon.getSupport(direction),
        rounded_polygon_minus_point.getSupport(direction),
        transformed_triangle.getSupport(direction),
        transformed_rounded_polygon.getSupport(direction),
        epa_like.getSupport(direction),
        transformed_difference.getSupport(direction),
        transformed_point.getSupport(direction),
        gjk_cast_like.getSupport(direction),
    };

    var result = std.mem.zeroes(ConvexSupportResult);
    for (supports, &result.supports) |s, *out|
        out.* = arr3(s);

    var faces = [_]StaticArray(Vec3, 32){.empty} ** num_faces;
    triangle.getSupportingFace(direction, &faces[0]);
    polygon.getSupportingFace(direction, &faces[1]);
    transformed_triangle.getSupportingFace(direction, &faces[2]);
    faces[3].append(vec3(point_in)); // Vertices that are already in the array get transformed too
    transformed_polygon.getSupportingFace(direction, &faces[3]);
    transformed_polygon_face.getSupportingFace(direction, &faces[4]);
    for (&faces, 0..) |*face, i|
        storeFace(face, &result.faces[i], &result.face_counts[i]);
    return result;
}

test "ConvexSupport" {
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "ConvexSupport" };

    for (0..iterations / 4) |_| {
        // Transform: rotation, uniform scale and translation, or a random matrix
        var transform: [16]f32 = undefined;
        if (gen.oneIn(5)) {
            transform = gen.rng.floatArray(16, -2, 2);
        } else {
            const axis = gen.unit();
            const m = Mat44.rotationTranslation(Quat.rotation(vec3(axis), gen.rng.float(-4, 4)), vec3(gen.vec(-10, 10))).mul(Mat44.scale(if (gen.oneIn(4)) 1 else gen.rng.float(0.1, 3)));
            for (0..4) |c| {
                const col: [4]f32 = m.col[c].value;
                @memcpy(transform[4 * c .. 4 * c + 4], &col);
            }
        }

        const point = gen.vec(-2, 2);
        const t = gen.triangle();
        const triangle = t[0] ++ t[1] ++ t[2];

        // Polygon: 1 .. 31 vertices (one face also holds an extra vertex, a face has room for 32), sometimes integer coordinates (ties) or duplicates
        var polygon: [31]P = undefined;
        const polygon_count = 1 + gen.index(31);
        const grid = gen.oneIn(3);
        for (polygon[0..polygon_count]) |*v|
            v.* = if (grid) gen.gridVec(2) else gen.vec(-2, 2);
        if (gen.oneIn(4)) polygon[gen.index(polygon_count)] = polygon[gen.index(polygon_count)];

        const radii = [_]f32{ 0, 0.05, 1 };
        const radius = if (gen.oneIn(3)) radii[gen.index(radii.len)] else gen.rng.float(0, 1);

        var direction = gen.direction();
        if (gen.oneIn(10)) direction = cross(sub(t[1], t[0]), gen.vec(-1, 1)); // Ties for the triangle
        if (gen.oneIn(10)) direction = .{ 1.0e-30, -1.0e-30, 0 }; // Length underflows to 0

        var expected = std.mem.zeroes(ConvexSupportResult);
        var face_counts: [num_faces]c_int = undefined;
        jolt.jolt_convex_support(&transform, &point, &triangle, @ptrCast(&polygon), @intCast(polygon_count), radius, &direction, &expected.supports, &expected.faces, &face_counts);
        for (face_counts, &expected.face_counts) |c, *out|
            out.* = c;

        const m = Mat44.init(vec4(transform[0..4].*), vec4(transform[4..8].*), vec4(transform[8..12].*), vec4(transform[12..16].*));
        const actual = try convexSupport(m, point, triangle, polygon[0..polygon_count], radius, vec3(direction));
        checker.check(.{ transform, point, triangle, polygon[0..polygon_count].len, radius, direction }, actual, expected);
    }

    try checker.finish();
}

/// Maximum number of positions in a hull test
const max_hull_positions = 300;

const HullResult = struct {
    result: i32,
    num_edges: i32,
    edges: [max_hull_positions]i32,
};

fn checkHull(checker: *Checker, positions: []const P, idx1: i32, idx2: i32, idx3: i32, max_vertices: i32, tolerance: f32) !void {
    // Jolt
    var expected = std.mem.zeroes(HullResult);
    var edges: [max_hull_positions]c_int = @splat(0);
    var num_edges: c_int = 0;
    expected.result = jolt.jolt_convex_hull_builder_2d(@ptrCast(positions.ptr), @intCast(positions.len), idx1, idx2, idx3, max_vertices, tolerance, &edges, &num_edges);
    expected.num_edges = num_edges;
    for (edges, &expected.edges) |e, *out|
        out.* = e;

    // Zolt
    var zolt_positions: [max_hull_positions]Vec3 = undefined;
    for (positions, 0..) |p, i|
        zolt_positions[i] = vec3(p);
    var builder = ConvexHullBuilder2D.init(std.testing.allocator, zolt_positions[0..positions.len]);
    defer builder.deinit();
    var zolt_edges: ConvexHullBuilder2D.Edges = .empty;
    defer zolt_edges.deinit(std.testing.allocator);
    var actual = std.mem.zeroes(HullResult);
    actual.result = @intFromEnum(try builder.initialize(idx1, idx2, idx3, max_vertices, tolerance, &zolt_edges));
    actual.num_edges = @intCast(zolt_edges.items.len);
    @memcpy(actual.edges[0..zolt_edges.items.len], zolt_edges.items);

    checker.check(.{ positions.len, idx1, idx2, idx3, max_vertices, tolerance }, actual, expected);
}

test "ConvexHullBuilder2D" {
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "ConvexHullBuilder2D" };
    const no_limit = std.math.maxInt(i32);

    // Hand picked: the inline test cases
    const square = [_]P{ .{ 0, 0, 0 }, .{ -1, -1, 0 }, .{ 1, -1, 0 }, .{ 1, 1, 0 }, .{ -1, 1, 0 }, .{ 0.5, 0.5, 0 }, .{ 1, 0, 0 }, .{ 1, 1, 0 }, .{ -0.5, 0.2, 7 } };
    try checkHull(&checker, &square, 4, 2, 1, no_limit, 1.0e-3);
    try checkHull(&checker, &square, 0, 2, 3, no_limit, 1.0e-3);
    try checkHull(&checker, &square, 0, 2, 3, 4, 1.0e-3);
    try checkHull(&checker, &square, 0, 2, 3, 3, 1.0e-3);
    try checkHull(&checker, &square, 1, 2, 3, no_limit, 10);
    const line = [_]P{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 2, 0, 0 }, .{ 3, 0, 0 } };
    try checkHull(&checker, &line, 0, 1, 2, no_limit, 0);
    try checkHull(&checker, &line, 3, 0, 1, no_limit, 0);

    var positions: [max_hull_positions]P = undefined;
    for (0..iterations / 10) |_| {
        const count = if (gen.oneIn(10)) 3 + gen.index(max_hull_positions - 2) else 3 + gen.index(62);
        const pos = positions[0..count];
        const s = if (gen.oneIn(5)) gen.scale() else 1;
        switch (gen.next() % 8) {
            // Disk
            0 => for (pos) |*p| {
                const u = gen.unit();
                const r = gen.rng.float(0, 1);
                p.* = .{ u[0] * r, u[1] * r, gen.float(-1, 1) };
            },
            // Circle (many hull vertices)
            1 => for (pos) |*p| {
                const u = gen.unit();
                const len = @sqrt(u[0] * u[0] + u[1] * u[1]);
                p.* = if (len > 0) .{ u[0] / len, u[1] / len, 0 } else .{ 1, 0, 0 };
            },
            // Integer grid (colinear points and duplicates)
            2 => for (pos) |*p| {
                p.* = .{ gen.grid(3), gen.grid(3), gen.grid(1) };
            },
            // All colinear
            3 => {
                const d = gen.unit();
                for (pos) |*p| p.* = mulS(d, gen.float(-2, 2));
            },
            // Nearly colinear
            4 => {
                const d = gen.unit();
                for (pos) |*p| p.* = add(mulS(d, gen.float(-2, 2)), .{ gen.rng.float(-1.0e-5, 1.0e-5), gen.rng.float(-1.0e-5, 1.0e-5), 0 });
            },
            // Clusters
            5 => {
                const c1 = gen.vec(-1, 1);
                const c2 = gen.vec(-1, 1);
                for (pos) |*p| p.* = add(if (gen.oneIn(2)) c1 else c2, mulS(gen.vec(-1, 1), 0.01));
            },
            else => for (pos) |*p| {
                p.* = gen.vec(-1, 1);
            },
        }
        for (pos) |*p| p.* = mulS(p.*, s);

        // Duplicates
        if (gen.oneIn(4)) {
            for (0..1 + gen.index(count)) |_| pos[gen.index(count)] = pos[gen.index(count)];
        }

        // Initial triangle: 3 different indices, sometimes the extreme points (like ConvexHullBuilder)
        var idx: [3]i32 = .{ 0, 0, 0 };
        if (gen.oneIn(3)) {
            var min_x: usize = 0;
            var max_x: usize = 0;
            for (pos, 0..) |p, i| {
                if (p[0] < pos[min_x][0]) min_x = i;
                if (p[0] > pos[max_x][0]) max_x = i;
            }
            var furthest: usize = 0;
            var furthest_dist: f32 = -1;
            for (pos, 0..) |p, i| {
                const dist = @abs(cross(sub(pos[max_x], pos[min_x]), sub(p, pos[min_x]))[2]);
                if (dist > furthest_dist) {
                    furthest_dist = dist;
                    furthest = i;
                }
            }
            idx = .{ @intCast(min_x), @intCast(max_x), @intCast(furthest) };
        }
        if (idx[0] == idx[1] or idx[0] == idx[2] or idx[1] == idx[2]) {
            idx[0] = @intCast(gen.index(count));
            idx[1] = @intCast((@as(usize, @intCast(idx[0])) + 1 + gen.index(count - 1)) % count);
            while (true) {
                idx[2] = @intCast(gen.index(count));
                if (idx[2] != idx[0] and idx[2] != idx[1]) break;
            }
        }

        const max_vertices: i32 = if (gen.oneIn(3)) @intCast(3 + gen.index(8)) else no_limit;
        const tolerances = [_]f32{ 0, 1.0e-6, 1.0e-4, 1.0e-3, 0.01, 0.1 };
        const tolerance = tolerances[gen.index(tolerances.len)] * s;

        try checkHull(&checker, pos, idx[0], idx[1], idx[2], max_vertices, tolerance);
    }

    try checker.finish();
}
