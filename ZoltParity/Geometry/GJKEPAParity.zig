//! Parity tests for GJK and EPA (GJKClosestPoint, EPAConvexHullBuilder, EPAPenetrationDepth): run Zolt and the C++ Jolt
//! library on the same inputs and require identical bits. C ABI wrappers: ZoltParity/Geometry/GJKEPAReference.cpp. See
//! ZoltParity/parity.zig for how parity tests work.
//!
//! The convex objects are points, spheres, boxes, triangles and polygons, optionally wrapped in AddConvexRadius and / or
//! TransformedConvexObject (a ShapeDesc describes one, both sides build it from their own ConvexSupport wrappers). The
//! pairs are random (separated or overlapping), identical, touching exactly (spheres, boxes sharing a face / edge /
//! vertex, points and triangles on box faces), slightly separated / overlapping, deeply penetrating, far apart, tiny and
//! huge, with degenerate members (zero radius spheres, flat boxes, colinear / coincident triangles, single vertex and
//! planar polygons). All outputs are compared: return values, every in/out and out parameter (filled with a sentinel
//! first, so outputs that are not written are compared too) and the GJK simplex (y, p, q, number of points). The hull
//! builder test compares every triangle it creates, the facing triangles and the order in which triangles leave the queue.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const AABox = zolt.AABox;
const AddConvexRadius = zolt.AddConvexRadius;
const EPAConvexHullBuilder = zolt.EPAConvexHullBuilder;
const EPAPenetrationDepth = zolt.EPAPenetrationDepth;
const GJKClosestPoint = zolt.GJKClosestPoint;
const Mat44 = zolt.Mat44;
const PointConvexSupport = zolt.PointConvexSupport;
const PolygonConvexSupport = zolt.PolygonConvexSupport;
const Quat = zolt.Quat;
const Sphere = zolt.Sphere;
const StaticArray = zolt.StaticArray;
const TransformedConvexObject = zolt.TransformedConvexObject;
const TriangleConvexSupport = zolt.TriangleConvexSupport;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

/// The C++ reference functions, see GJKEPAReference.cpp
const jolt = struct {
    extern fn jolt_gjk_intersects(a: *const ShapeDesc, b: *const ShapeDesc, tolerance: f32, io_v: *P, out_simplex: *Simplex) bool;
    extern fn jolt_gjk_get_closest_points(a: *const ShapeDesc, b: *const ShapeDesc, tolerance: f32, max_dist_sq: f32, io_v: *P, io_point_a: *P, io_point_b: *P, out_simplex: *Simplex) f32;
    extern fn jolt_gjk_cast_ray(origin: *const P, direction: *const P, tolerance: f32, a: *const ShapeDesc, io_lambda: *f32, out_simplex: *Simplex) bool;
    extern fn jolt_gjk_cast_shape(start: *const [16]f32, direction: *const P, tolerance: f32, a: *const ShapeDesc, b: *const ShapeDesc, io_lambda: *f32, out_simplex: *Simplex) bool;
    extern fn jolt_gjk_cast_shape_radius(start: *const [16]f32, direction: *const P, tolerance: f32, a: *const ShapeDesc, b: *const ShapeDesc, convex_radius_a: f32, convex_radius_b: f32, io_lambda: *f32, io_point_a: *P, io_point_b: *P, io_separating_axis: *P, out_simplex: *Simplex) bool;
    extern fn jolt_epa_penetration_depth_steps(a: *const ShapeDesc, convex_radius_a: f32, b: *const ShapeDesc, convex_radius_b: f32, include_mode: c_int, collision_tolerance: f32, penetration_tolerance: f32, io_v: *P, io_point_a: *P, io_point_b: *P, out_gjk_v: *P, out_gjk_point_a: *P, out_gjk_point_b: *P, out_epa_result: *c_int) c_int;
    extern fn jolt_epa_get_penetration_depth(a: *const ShapeDesc, convex_radius_a: f32, b: *const ShapeDesc, convex_radius_b: f32, include_mode: c_int, collision_tolerance_sq: f32, penetration_tolerance: f32, io_v: *P, io_point_a: *P, io_point_b: *P) bool;
    extern fn jolt_epa_cast_shape(start: *const [16]f32, direction: *const P, collision_tolerance: f32, penetration_tolerance: f32, a: *const ShapeDesc, b: *const ShapeDesc, convex_radius_a: f32, convex_radius_b: f32, return_deepest_point: c_int, io_lambda: *f32, io_point_a: *P, io_point_b: *P, io_contact_normal: *P) bool;
    extern fn jolt_epa_hull(points: [*]const P, num_points: c_int, closest_dist_sq: [*]const f32, pop_every: c_int, out_records: [*]HullRecord) c_int;
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

fn loadMat44(m: [16]f32) Mat44 {
    return Mat44.init(vec4(m[0..4].*), vec4(m[4..8].*), vec4(m[8..12].*), vec4(m[12..16].*));
}

fn storeMat44(m: Mat44) [16]f32 {
    var result: [16]f32 = undefined;
    for (0..4) |c| {
        const col: [4]f32 = m.col[c].value;
        @memcpy(result[4 * c .. 4 * c + 4], &col);
    }
    return result;
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

/// Filled into the in/out parameters before a call. Zolt fills the points that Jolt documents as invalid with 0xcd bytes
/// in Debug builds (JPH_DEBUG), Jolt (compiled without JPH_DEBUG) leaves them untouched: with this sentinel both agree.
const sentinel: f32 = @bitCast(@as(u32, 0xcdcdcdcd));
const sentinel_p: P = .{ sentinel, sentinel, sentinel };

const identity_transform: [16]f32 = .{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

/// Description of a convex object, must match ShapeDesc in GJKEPAReference.cpp
const ShapeDesc = extern struct {
    /// 0 = point, 1 = sphere, 2 = box, 3 = triangle, 4 = polygon
    kind: c_int = 0,
    /// 0 = plain, 1 = AddConvexRadius, 2 = TransformedConvexObject, 3 = TransformedConvexObject(AddConvexRadius), 4 = AddConvexRadius(TransformedConvexObject)
    mode: c_int = 0,
    /// Radius of the AddConvexRadius wrapper
    radius: f32 = 0,
    /// Number of polygon vertices
    count: c_int = 0,
    /// Transform of the TransformedConvexObject wrapper
    transform: [16]f32 = identity_transform,
    /// point: position; sphere: center, radius; box: min, max; triangle: 3 vertices; polygon: vertices
    data: [3 * 32]f32 = @splat(0),

    fn point(p: P) ShapeDesc {
        var d: ShapeDesc = .{ .kind = 0 };
        d.data[0..3].* = p;
        return d;
    }

    fn sphere(center: P, radius: f32) ShapeDesc {
        var d: ShapeDesc = .{ .kind = 1 };
        d.data[0..3].* = center;
        d.data[3] = radius;
        return d;
    }

    fn box(min: P, max: P) ShapeDesc {
        var d: ShapeDesc = .{ .kind = 2 };
        d.data[0..3].* = min;
        d.data[3..6].* = max;
        return d;
    }

    fn triangle(v1: P, v2: P, v3: P) ShapeDesc {
        var d: ShapeDesc = .{ .kind = 3 };
        d.data[0..3].* = v1;
        d.data[3..6].* = v2;
        d.data[6..9].* = v3;
        return d;
    }

    fn polygon(vertices: []const P) ShapeDesc {
        std.debug.assert(vertices.len >= 1 and vertices.len <= 32);
        var d: ShapeDesc = .{ .kind = 4, .count = @intCast(vertices.len) };
        for (vertices, 0..) |v, i|
            d.data[3 * i ..][0..3].* = v;
        return d;
    }

    /// Translate the object (not its convex radius wrapper)
    fn translated(self: ShapeDesc, t: P) ShapeDesc {
        var d = self;
        const n: usize = switch (d.kind) {
            0, 1 => 1,
            2 => 2,
            3 => 3,
            else => @intCast(d.count),
        };
        for (0..n) |i|
            d.data[3 * i ..][0..3].* = add(d.data[3 * i ..][0..3].*, t);
        return d;
    }
};

/// The Zolt version of BaseShape in GJKEPAReference.cpp: a point, sphere, box, triangle or polygon
const BaseShape = struct {
    kind: c_int,
    point: PointConvexSupport = .{ .point = Vec3.zero() },
    sphere: Sphere = Sphere.init(Vec3.zero(), 0),
    box: AABox = AABox.init(Vec3.zero(), Vec3.zero()),
    triangle: TriangleConvexSupport = TriangleConvexSupport.init(Vec3.zero(), Vec3.zero(), Vec3.zero()),
    vertices: StaticArray(Vec3, 32) = .empty,

    fn init(desc: *const ShapeDesc) BaseShape {
        var result: BaseShape = .{ .kind = desc.kind };
        const d = &desc.data;
        switch (desc.kind) {
            0 => result.point.point = vec3(d[0..3].*),
            1 => result.sphere = Sphere.init(vec3(d[0..3].*), d[3]),
            2 => result.box = AABox.init(vec3(d[0..3].*), vec3(d[3..6].*)),
            3 => result.triangle = TriangleConvexSupport.init(vec3(d[0..3].*), vec3(d[3..6].*), vec3(d[6..9].*)),
            else => for (0..@intCast(desc.count)) |i| result.vertices.append(vec3(d[3 * i ..][0..3].*)),
        }
        return result;
    }

    pub fn getSupport(self: *const BaseShape, direction: Vec3) Vec3 {
        return switch (self.kind) {
            0 => self.point.getSupport(direction),
            1 => self.sphere.getSupport(direction),
            2 => self.box.getSupport(direction),
            3 => self.triangle.getSupport(direction),
            else => PolygonConvexSupport.init(self.vertices.constSlice()).getSupport(direction),
        };
    }
};

/// The Zolt version of Shape in GJKEPAReference.cpp: a BaseShape, optionally wrapped in AddConvexRadius and / or TransformedConvexObject
const Shape = struct {
    base: BaseShape,
    mode: c_int,
    radius: f32,
    transform: Mat44,

    fn init(desc: *const ShapeDesc) Shape {
        return .{ .base = BaseShape.init(desc), .mode = desc.mode, .radius = desc.radius, .transform = loadMat44(desc.transform) };
    }

    pub fn getSupport(self: *const Shape, direction: Vec3) Vec3 {
        switch (self.mode) {
            0 => return self.base.getSupport(direction),
            1 => return AddConvexRadius(BaseShape).init(&self.base, self.radius).getSupport(direction),
            2 => return TransformedConvexObject(BaseShape).init(self.transform, &self.base).getSupport(direction),
            3 => {
                const rounded = AddConvexRadius(BaseShape).init(&self.base, self.radius);
                return TransformedConvexObject(AddConvexRadius(BaseShape)).init(self.transform, &rounded).getSupport(direction);
            },
            else => {
                const transformed = TransformedConvexObject(BaseShape).init(self.transform, &self.base);
                return AddConvexRadius(TransformedConvexObject(BaseShape)).init(&transformed, self.radius).getSupport(direction);
            },
        }
    }
};

/// Simplex of the GJK algorithm, must match Simplex in GJKEPAReference.cpp
const Simplex = extern struct {
    y: [4]P,
    p: [4]P,
    q: [4]P,
    num_points: u32,
};

/// Which parts of the simplex a function keeps valid (the others are stale or uninitialized in Jolt)
const SimplexParts = enum { y, yp, ypq };

/// Zero the parts of the simplex that are not compared
fn masked(simplex: Simplex, parts: SimplexParts) Simplex {
    var result = simplex;
    for (0..4) |i| {
        if (i >= simplex.num_points) {
            result.y[i] = .{ 0, 0, 0 };
            result.p[i] = .{ 0, 0, 0 };
            result.q[i] = .{ 0, 0, 0 };
        }
        if (parts == .y) result.p[i] = .{ 0, 0, 0 };
        if (parts != .ypq) result.q[i] = .{ 0, 0, 0 };
    }
    return result;
}

fn storeSimplex(gjk: *const GJKClosestPoint, parts: SimplexParts) Simplex {
    var y: [4]Vec3 = @splat(Vec3.zero());
    var p: [4]Vec3 = @splat(Vec3.zero());
    var q: [4]Vec3 = @splat(Vec3.zero());
    const num_points = gjk.getClosestPointsSimplex(&y, &p, &q);
    var result = std.mem.zeroes(Simplex);
    for (0..num_points) |i| {
        result.y[i] = arr3(y[i]);
        result.p[i] = arr3(p[i]);
        result.q[i] = arr3(q[i]);
    }
    result.num_points = num_points;
    return masked(result, parts);
}

/// Values that tend to expose differences, mixed into the random inputs
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 1.0e-6, -1.0e-6, 1.0e-20, -1.0e-20 };

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

    /// Pick one of the values
    fn pick(self: *Gen, comptime T: type, values: []const T) T {
        return values[self.index(values.len)];
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

    /// Random power of 10
    fn scale(self: *Gen) f32 {
        return self.pick(f32, &.{ 1.0e-5, 1.0e-4, 1.0e-3, 1.0e-2, 0.1, 1.0, 10.0, 100.0, 1.0e3, 1.0e4, 1.0e5 });
    }

    /// Random unit vector
    fn unit(self: *Gen) P {
        while (true) {
            const v = self.rng.floatArray(3, -1, 1);
            const len_sq = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            if (len_sq > 1.0e-4 and len_sq <= 1.0) return mulS(v, 1.0 / @sqrt(len_sq));
        }
    }

    /// Unit vector along a (signed) coordinate axis
    fn axis(self: *Gen) P {
        var a: P = .{ 0, 0, 0 };
        a[self.index(3)] = if (self.oneIn(2)) 1 else -1;
        return a;
    }

    /// Random rigid transform (rotation + translation), sometimes with uniform scale, sometimes identity / translation only
    fn transform(self: *Gen, s: f32) [16]f32 {
        return switch (self.next() % 8) {
            0 => identity_transform,
            1 => storeMat44(Mat44.translation(vec3(mulS(self.vec(-1, 1), s)))),
            2 => storeMat44(Mat44.rotationTranslation(Quat.rotation(vec3(self.axis()), 0.5 * math.pi), vec3(mulS(self.gridVec(2), s)))),
            3 => storeMat44(Mat44.rotationTranslation(Quat.rotation(vec3(self.unit()), self.rng.float(-4, 4)), vec3(mulS(self.vec(-1, 1), s))).mul(Mat44.scale(self.rng.float(0.25, 3)))),
            else => storeMat44(Mat44.rotationTranslation(Quat.rotation(vec3(self.unit()), self.rng.float(-4, 4)), vec3(mulS(self.vec(-1, 1), s)))),
        };
    }

    /// Random base object of size ~s around the origin
    fn baseShape(self: *Gen, s: f32) ShapeDesc {
        switch (self.next() % 5) {
            0 => return ShapeDesc.point(if (self.oneIn(4)) mulS(self.gridVec(1), s) else mulS(self.vec(-1, 1), s)),
            1 => {
                const radius: f32 = if (self.oneIn(10)) 0 else if (self.oneIn(4)) self.pick(f32, &.{ 0.5, 1, 2 }) * s else self.rng.float(0.05, 1.5) * s;
                return ShapeDesc.sphere(if (self.oneIn(4)) mulS(self.gridVec(1), s) else mulS(self.vec(-1, 1), s), radius);
            },
            2 => {
                const p1 = if (self.oneIn(4)) self.gridVec(2) else self.vec(-1.5, 1.5);
                var p2 = if (self.oneIn(4)) self.gridVec(2) else self.vec(-1.5, 1.5);
                if (self.oneIn(6)) p2[self.index(3)] = p1[self.index(3)]; // Sometimes flat
                if (self.oneIn(10)) p2 = p1; // Sometimes a point
                var min: P = undefined;
                var max: P = undefined;
                for (0..3) |i| {
                    min[i] = @min(p1[i], p2[i]) * s;
                    max[i] = @max(p1[i], p2[i]) * s;
                }
                return ShapeDesc.box(min, max);
            },
            3 => {
                var t: [3]P = .{ self.vec(-1.5, 1.5), self.vec(-1.5, 1.5), self.vec(-1.5, 1.5) };
                switch (self.next() % 8) {
                    0 => t = .{ self.gridVec(2), self.gridVec(2), self.gridVec(2) },
                    1 => t[self.index(3)] = lerp(t[0], t[1], self.float(-1, 2)), // Colinear
                    2 => t[1] = t[0], // Coincident vertices
                    else => {},
                }
                return ShapeDesc.triangle(mulS(t[0], s), mulS(t[1], s), mulS(t[2], s));
            },
            else => {
                var vertices: [32]P = undefined;
                const count = 1 + self.index(32);
                switch (self.next() % 6) {
                    // Points on a circle in a random plane (a planar polygon)
                    0 => {
                        const n = self.unit();
                        const c = self.vec(-0.5, 0.5);
                        const e1 = self.unit();
                        const e2: P = .{ n[1] * e1[2] - n[2] * e1[1], n[2] * e1[0] - n[0] * e1[2], n[0] * e1[1] - n[1] * e1[0] };
                        for (vertices[0..count], 0..) |*v, i| {
                            const angle = 2.0 * math.pi * @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(count));
                            v.* = add(c, add(mulS(e1, zolt.trigonometry.cos(angle)), mulS(e2, zolt.trigonometry.sin(angle))));
                        }
                    },
                    // Points on a sphere
                    1 => for (vertices[0..count]) |*v| {
                        v.* = self.unit();
                    },
                    // Integer coordinates (ties)
                    2 => for (vertices[0..count]) |*v| {
                        v.* = self.gridVec(1);
                    },
                    else => for (vertices[0..count]) |*v| {
                        v.* = self.vec(-1.5, 1.5);
                    },
                }
                if (self.oneIn(5)) vertices[self.index(count)] = vertices[self.index(count)]; // Duplicates
                for (vertices[0..count]) |*v| v.* = mulS(v.*, s);
                return ShapeDesc.polygon(vertices[0..count]);
            },
        }
    }

    /// Random wrappers around a base object
    fn wrap(self: *Gen, base: ShapeDesc, s: f32) ShapeDesc {
        var d = base;
        d.mode = if (self.oneIn(2)) 0 else @intCast(1 + self.index(4));
        d.radius = if (self.oneIn(4)) 0 else self.rng.float(0, 0.5) * s;
        d.transform = self.transform(s);
        return d;
    }

    fn shape(self: *Gen, s: f32) ShapeDesc {
        return self.wrap(self.baseShape(s), s);
    }

    /// A pair of convex objects in one of many configurations (separated / touching / overlapping / identical, ...)
    fn pair(self: *Gen) [2]ShapeDesc {
        const s: f32 = if (self.oneIn(6)) self.scale() else 1;
        switch (self.next() % 16) {
            // Identical objects (deeply penetrating, symmetric Minkowski difference)
            0 => {
                const a = self.shape(s);
                return .{ a, a };
            },
            // Same object, slightly moved
            1 => {
                const a = self.shape(s);
                return .{ a, a.translated(mulS(self.vec(-1, 1), s * self.pick(f32, &.{ 1.0e-6, 1.0e-3, 0.1 }))) };
            },
            // Spheres touching exactly along an axis
            2 => {
                const c = self.gridVec(2);
                const r1: f32 = 0.5 * @as(f32, @floatFromInt(1 + self.index(4)));
                const r2: f32 = 0.5 * @as(f32, @floatFromInt(self.index(4)));
                const gap = self.pick(f32, &.{ 0, 0, 1.0e-6, -1.0e-6, 1.0e-3, -1.0e-3 });
                return .{ ShapeDesc.sphere(mulS(c, s), r1 * s), ShapeDesc.sphere(mulS(add(c, mulS(self.axis(), r1 + r2 + gap)), s), r2 * s) };
            },
            // Integer boxes sharing a face, edge or vertex (or slightly separated / overlapping)
            3 => {
                const min = self.gridVec(2);
                const size: P = .{ @floatFromInt(1 + self.index(3)), @floatFromInt(1 + self.index(3)), @floatFromInt(1 + self.index(3)) };
                const max = add(min, size);
                var offset: P = .{ 0, 0, 0 };
                const num_axis = 1 + self.index(3);
                for (0..num_axis) |_| {
                    const i = self.index(3);
                    offset[i] = if (self.oneIn(2)) size[i] else -size[i];
                }
                const gap = self.pick(f32, &.{ 0, 0, 0, 1.0e-5, -1.0e-5, 0.01, -0.01 });
                for (&offset) |*o| o.* += if (o.* > 0) gap else if (o.* < 0) -gap else 0;
                return .{ ShapeDesc.box(mulS(min, s), mulS(max, s)), ShapeDesc.box(mulS(add(min, offset), s), mulS(add(max, offset), s)) };
            },
            // Point on / in / near a box
            4 => {
                const b = ShapeDesc.box(mulS(.{ -1, -2, -1 }, s), mulS(.{ 2, 1, 1 }, s));
                var p: P = self.gridVec(2);
                if (self.oneIn(2)) p[self.index(3)] = self.rng.float(-2, 2);
                const a = ShapeDesc.point(mulS(p, s));
                return if (self.oneIn(2)) .{ a, b } else .{ b, a };
            },
            // Triangle in the plane of a box face (or crossing it)
            5 => {
                const z = self.pick(f32, &.{ 1, 1, 1.0001, 0.9999, 0 });
                const t = ShapeDesc.triangle(.{ self.grid(2), self.grid(2), z }, .{ self.grid(2), self.grid(2), z }, .{ self.grid(2), self.grid(2), if (self.oneIn(3)) self.grid(2) else z });
                return .{ ShapeDesc.box(mulS(.{ -1, -1, -1 }, s), mulS(.{ 1, 1, 1 }, s)), t };
            },
            // Far apart
            6 => {
                const a = self.shape(s);
                const b = self.shape(s);
                return .{ a, b.translated(mulS(self.unit(), s * self.pick(f32, &.{ 10, 1.0e3, 1.0e5 }))) };
            },
            // Small object inside a big one
            7 => {
                const a = self.shape(s * 3);
                const b = self.shape(s * 0.1);
                return if (self.oneIn(2)) .{ a, b } else .{ b, a };
            },
            // Plain objects without wrappers
            8 => return .{ self.baseShape(s), self.baseShape(s) },
            // Curved objects (spheres, rounded points / boxes) nearly touching along a random direction
            9 => {
                const u = self.unit();
                const r1 = self.rng.float(0.1, 1.5);
                const r2 = self.rng.float(0.1, 1.5);
                const gap = self.pick(f32, &.{ 0, 1.0e-6, -1.0e-6, 1.0e-4, -1.0e-4, 1.0e-2 });
                const c = self.vec(-1, 1);
                var a = ShapeDesc.sphere(mulS(c, s), r1 * s);
                var b = ShapeDesc.sphere(mulS(add(c, mulS(u, r1 + r2 + gap)), s), r2 * s);
                if (self.oneIn(2)) {
                    // Rounded point / box instead of a sphere
                    a = if (self.oneIn(2)) ShapeDesc.point(mulS(c, s)) else ShapeDesc.box(mulS(c, s), mulS(c, s));
                    a.mode = 1;
                    a.radius = r1 * s;
                }
                if (self.oneIn(3)) b.transform = storeMat44(Mat44.rotationTranslation(Quat.rotation(vec3(self.unit()), self.rng.float(-4, 4)), Vec3.zero()));
                if (self.oneIn(3)) b.mode = 2;
                return .{ a, b };
            },
            // Random
            else => return .{ self.shape(s), self.shape(s) },
        }
    }

    /// Initial separating axis for GJK
    fn initialV(self: *Gen, a: *const ShapeDesc, b: *const ShapeDesc) P {
        return switch (self.next() % 8) {
            0 => .{ 0, 0, 0 },
            1 => .{ 1, 0, 0 },
            2 => self.unit(),
            3 => sub(b.data[0..3].*, a.data[0..3].*), // Roughly from A to B
            4 => mulS(self.unit(), 1.0e-20),
            5 => mulS(self.unit(), 1.0e10),
            else => self.vec(-2, 2),
        };
    }

    /// Initial separating axis for EPA (must not be near zero)
    fn initialVNonZero(self: *Gen, a: *const ShapeDesc, b: *const ShapeDesc) P {
        while (true) {
            const v = self.initialV(a, b);
            if (vec3(v).lengthSq() > 1.0e-6) return v;
        }
    }

    /// Ray / cast direction
    fn direction(self: *Gen, s: f32) P {
        return switch (self.next() % 10) {
            0 => .{ 0, 0, 0 },
            1 => mulS(self.axis(), s * self.rng.float(0.5, 5)),
            2 => mulS(self.unit(), 1.0e-10),
            3 => mulS(self.gridVec(3), s),
            else => mulS(self.vec(-4, 4), s),
        };
    }

    /// Max fraction for casts
    fn lambda(self: *Gen) f32 {
        return self.pick(f32, &.{ 1.0 + math.flt_epsilon, 1.0 + math.flt_epsilon, 1.0, 0.5, math.flt_max, std.math.inf(f32), 0.0 });
    }

    fn tolerance(self: *Gen) f32 {
        return self.pick(f32, &.{ 0, 1.0e-6, 1.0e-4, 1.0e-4, 1.0e-3, 1.0e-2 });
    }

    fn penetrationTolerance(self: *Gen) f32 {
        return self.pick(f32, &.{ math.flt_epsilon, 1.0e-4, 1.0e-3, 1.0e-2 });
    }

    fn convexRadius(self: *Gen) f32 {
        return if (self.oneIn(3)) 0 else self.pick(f32, &.{ 0.01, 0.05, 0.1, 0.5, 1 });
    }

    /// Start transform for casts: rotation and translation
    fn start(self: *Gen) [16]f32 {
        return switch (self.next() % 4) {
            0 => identity_transform,
            1 => storeMat44(Mat44.translation(vec3(self.vec(-4, 4)))),
            else => storeMat44(Mat44.rotationTranslation(Quat.rotation(vec3(self.unit()), self.rng.float(-4, 4)), vec3(self.vec(-4, 4)))),
        };
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Hand picked inputs

/// A hand picked pair of convex objects. `huge` pairs overflow to infinity inside GJK: Jolt handles that in GJK, but
/// the EPA cast would continue with an empty simplex (which Jolt asserts on), so they are not used for it.
const EdgePair = struct {
    a: ShapeDesc,
    b: ShapeDesc,
    huge: bool = false,
};

fn withRadius(desc: ShapeDesc, radius: f32) ShapeDesc {
    var d = desc;
    d.mode = 1;
    d.radius = radius;
    return d;
}

fn withTransform(desc: ShapeDesc, m: Mat44) ShapeDesc {
    var d = desc;
    d.mode = 2;
    d.transform = storeMat44(m);
    return d;
}

/// Identical / coincident / exactly touching objects, degenerate objects, huge and tiny coordinates
const edge_pairs = blk: {
    @setEvalBranchQuota(100_000);
    const point = ShapeDesc.point;
    const sphere = ShapeDesc.sphere;
    const box = ShapeDesc.box;
    const triangle = ShapeDesc.triangle;
    const polygon = ShapeDesc.polygon;
    const unit_box = box(.{ 0, 0, 0 }, .{ 1, 1, 1 });
    const square = [_]P{ .{ -1, -1, 0 }, .{ 1, -1, 0 }, .{ 1, 1, 0 }, .{ -1, 1, 0 } };
    break :blk [_]EdgePair{
        // Coincident points
        .{ .a = point(.{ 0, 0, 0 }), .b = point(.{ 0, 0, 0 }) },
        .{ .a = point(.{ 1, 2, 3 }), .b = point(.{ 1, 2, 3 }) },
        .{ .a = point(.{ 0, 0, 0 }), .b = point(.{ 1.0e-30, 0, 0 }) },
        .{ .a = point(.{ 0, 0, 0 }), .b = point(.{ 1, 0, 0 }) },
        // Zero radius spheres
        .{ .a = sphere(.{ 0, 0, 0 }, 0), .b = sphere(.{ 0, 0, 0 }, 0) },
        .{ .a = sphere(.{ 0, 0, 0 }, 0), .b = sphere(.{ 0, 1, 0 }, 0) },
        // Spheres: touching exactly, overlapping, identical, concentric
        .{ .a = sphere(.{ 0, 0, 0 }, 1), .b = sphere(.{ 2, 0, 0 }, 1) },
        .{ .a = sphere(.{ 0, 0, 0 }, 1), .b = sphere(.{ 0, 0, 3 }, 2) },
        .{ .a = sphere(.{ 0, 0, 0 }, 1), .b = sphere(.{ 1, 0, 0 }, 1) },
        .{ .a = sphere(.{ 1, 2, 3 }, 2), .b = sphere(.{ 1, 2, 3 }, 2) },
        .{ .a = sphere(.{ 1, 2, 3 }, 2), .b = sphere(.{ 1, 2, 3 }, 0.5) },
        // Boxes sharing a face, an edge, a vertex; identical; nested; flat and point boxes
        .{ .a = unit_box, .b = box(.{ 1, 0, 0 }, .{ 2, 1, 1 }) },
        .{ .a = unit_box, .b = box(.{ 1, 1, 0 }, .{ 2, 2, 1 }) },
        .{ .a = unit_box, .b = box(.{ 1, 1, 1 }, .{ 2, 2, 2 }) },
        .{ .a = unit_box, .b = box(.{ 0, 0, 0 }, .{ 1, 1, 1 }) },
        .{ .a = box(.{ -2, -2, -2 }, .{ 2, 2, 2 }), .b = unit_box },
        .{ .a = box(.{ 0, 0, 0 }, .{ 1, 1, 0 }), .b = box(.{ 0, 0, 0 }, .{ 1, 1, 0 }) },
        .{ .a = box(.{ 0, 0, 0 }, .{ 0, 0, 0 }), .b = box(.{ 0, 0, 0 }, .{ 0, 0, 0 }) },
        .{ .a = unit_box, .b = point(.{ 1, 0.5, 0.5 }) },
        .{ .a = unit_box, .b = point(.{ 1, 1, 1 }) },
        .{ .a = unit_box, .b = point(.{ 0.5, 0.5, 0.5 }) },
        // Triangles: in the plane of a box face, identical coplanar, colinear, coincident vertices
        .{ .a = box(.{ -1, -1, -1 }, .{ 1, 1, 1 }), .b = triangle(.{ -2, -2, 1 }, .{ 2, -2, 1 }, .{ 0, 2, 1 }) },
        .{ .a = triangle(.{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }), .b = triangle(.{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }) },
        .{ .a = triangle(.{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 2, 0, 0 }), .b = point(.{ 1, 0, 0 }) },
        .{ .a = triangle(.{ 0, 0, 0 }, .{ 0, 0, 0 }, .{ 0, 0, 0 }), .b = sphere(.{ 0, 0, 0 }, 1) },
        // Polygons: single vertex, planar square against a box face, duplicated vertices
        .{ .a = polygon(&.{.{ 0, 0, 0 }}), .b = point(.{ 0, 0, 0 }) },
        .{ .a = polygon(&square), .b = box(.{ -1, -1, -2 }, .{ 1, 1, 0 }) },
        .{ .a = polygon(&square), .b = polygon(&square) },
        .{ .a = polygon(&(square ++ square)), .b = sphere(.{ 0, 0, 1 }, 1) },
        // Rounded and transformed objects
        .{ .a = withRadius(point(.{ 0, 0, 0 }), 1), .b = withRadius(point(.{ 2, 0, 0 }), 1) },
        .{ .a = withRadius(unit_box, 0.5), .b = withRadius(box(.{ 2, 0, 0 }, .{ 3, 1, 1 }), 0.5) },
        .{ .a = withTransform(unit_box, Mat44.rotationZ(0.25 * math.pi)), .b = withTransform(unit_box, Mat44.translation(Vec3.init(1.5, 0, 0))) },
        // Tiny
        .{ .a = sphere(.{ 0, 0, 0 }, 1.0e-20), .b = sphere(.{ 1.0e-20, 0, 0 }, 1.0e-20) },
        .{ .a = box(.{ 0, 0, 0 }, .{ 1.0e-20, 1.0e-20, 1.0e-20 }), .b = point(.{ 1.0e-20, 0, 0 }) },
        // Huge
        .{ .a = sphere(.{ 0, 0, 0 }, 1), .b = sphere(.{ 1.0e20, 0, 0 }, 1), .huge = true },
        .{ .a = point(.{ 1.0e20, 0, 0 }), .b = point(.{ -1.0e20, 0, 0 }), .huge = true },
        .{ .a = sphere(.{ 1.0e20, 0, 0 }, 1), .b = sphere(.{ 1.0e20, 0, 0 }, 1), .huge = true },
        .{ .a = sphere(.{ 0, 0, 0 }, 1.0e18), .b = sphere(.{ 0, 0, 1.0e18 }, 1.0e18), .huge = true },
        .{ .a = box(.{ -1.0e19, -1.0e19, -1.0e19 }, .{ 1.0e19, 1.0e19, 1.0e19 }), .b = point(.{ 0, 0, 0 }), .huge = true },
    };
};

/// Initial separating axes for the hand picked inputs
const edge_vs = [_]P{ .{ 0, 0, 0 }, .{ 1, 0, 0 }, .{ 0, -1, 0 }, .{ 1, 1, 1 } };

// ---------------------------------------------------------------------------------------------------------------------
// GJKClosestPoint

const IntersectsResult = struct {
    result: bool,
    v: P,
    simplex: Simplex,
};

fn zoltIntersects(a_desc: *const ShapeDesc, b_desc: *const ShapeDesc, tolerance: f32, v_in: P) IntersectsResult {
    const a = Shape.init(a_desc);
    const b = Shape.init(b_desc);
    var gjk: GJKClosestPoint = .{};
    var v = vec3(v_in);
    const result = gjk.intersects(&a, &b, tolerance, &v);
    return .{ .result = result, .v = arr3(v), .simplex = storeSimplex(&gjk, .y) };
}

fn checkIntersects(checker: *Checker, a: *const ShapeDesc, b: *const ShapeDesc, tolerance: f32, v: P) IntersectsResult {
    var expected = std.mem.zeroes(IntersectsResult);
    expected.v = v;
    expected.result = jolt.jolt_gjk_intersects(a, b, tolerance, &expected.v, &expected.simplex);
    expected.simplex = masked(expected.simplex, .y);

    const actual = zoltIntersects(a, b, tolerance, v);
    checker.check(.{ a.*, b.*, tolerance, v }, actual, expected);
    return actual;
}

test "GJKClosestPoint.intersects" {
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "GJKClosestPoint.intersects" };
    var hits: [2]u32 = .{ 0, 0 };

    for (edge_pairs) |pair| {
        for (edge_vs) |v| {
            for ([_]f32{ 0, 1.0e-4 }) |tolerance| {
                _ = checkIntersects(&checker, &pair.a, &pair.b, tolerance, v);
                _ = checkIntersects(&checker, &pair.b, &pair.a, tolerance, v);
            }
        }
    }

    for (0..iterations) |_| {
        const pair = gen.pair();
        const tolerance = gen.tolerance();
        const v = gen.initialV(&pair[0], &pair[1]);
        const actual = checkIntersects(&checker, &pair[0], &pair[1], tolerance, v);
        hits[@intFromBool(actual.result)] += 1;
    }

    errdefer std.debug.print("GJKClosestPoint.intersects: {d} separated, {d} intersecting\n", .{ hits[0], hits[1] });
    try std.testing.expect(hits[0] > iterations / 10 and hits[1] > iterations / 10);
    try checker.finish();
}

const ClosestPointsResult = struct {
    dist_sq: f32,
    v: P,
    point_a: P,
    point_b: P,
    simplex: Simplex,
};

fn zoltGetClosestPoints(a_desc: *const ShapeDesc, b_desc: *const ShapeDesc, tolerance: f32, max_dist_sq: f32, v_in: P) ClosestPointsResult {
    const a = Shape.init(a_desc);
    const b = Shape.init(b_desc);
    var gjk: GJKClosestPoint = .{};
    var v = vec3(v_in);
    var point_a = vec3(sentinel_p);
    var point_b = vec3(sentinel_p);
    const dist_sq = gjk.getClosestPoints(&a, &b, tolerance, max_dist_sq, &v, &point_a, &point_b);
    return .{ .dist_sq = dist_sq, .v = arr3(v), .point_a = arr3(point_a), .point_b = arr3(point_b), .simplex = storeSimplex(&gjk, .ypq) };
}

fn checkGetClosestPoints(checker: *Checker, a: *const ShapeDesc, b: *const ShapeDesc, tolerance: f32, max_dist_sq: f32, v: P) ClosestPointsResult {
    var expected = std.mem.zeroes(ClosestPointsResult);
    expected.v = v;
    expected.point_a = sentinel_p;
    expected.point_b = sentinel_p;
    expected.dist_sq = jolt.jolt_gjk_get_closest_points(a, b, tolerance, max_dist_sq, &expected.v, &expected.point_a, &expected.point_b, &expected.simplex);
    expected.simplex = masked(expected.simplex, .ypq);

    const actual = zoltGetClosestPoints(a, b, tolerance, max_dist_sq, v);
    checker.check(.{ a.*, b.*, tolerance, max_dist_sq, v }, actual, expected);
    return actual;
}

test "GJKClosestPoint.getClosestPoints" {
    var gen: Gen = .{ .rng = .{ .state = 0x9e3779b9 } };
    var checker: Checker = .{ .name = "GJKClosestPoint.getClosestPoints" };
    // Coverage: FLT_MAX (further than max distance), 0 with a full simplex, 0 with a smaller simplex, positive distance
    var counts: [4]u32 = @splat(0);

    for (edge_pairs) |pair| {
        for (edge_vs) |v| {
            for ([_]f32{ 0, 1.0e-4 }) |tolerance| {
                for ([_]f32{ math.flt_max, std.math.inf(f32), 1 }) |max_dist_sq| {
                    _ = checkGetClosestPoints(&checker, &pair.a, &pair.b, tolerance, max_dist_sq, v);
                    _ = checkGetClosestPoints(&checker, &pair.b, &pair.a, tolerance, max_dist_sq, v);
                }
            }
        }
    }

    for (0..iterations) |_| {
        const pair = gen.pair();
        const tolerance = gen.tolerance();
        const max_dist_sq = gen.pick(f32, &.{ math.flt_max, math.large_float, std.math.inf(f32), 0, 1.0e-4, 0.25, 1, 4 });
        const v = gen.initialV(&pair[0], &pair[1]);
        const actual = checkGetClosestPoints(&checker, &pair[0], &pair[1], tolerance, max_dist_sq, v);

        if (actual.dist_sq == math.flt_max) {
            counts[0] += 1;
        } else if (actual.dist_sq == 0) {
            counts[if (actual.simplex.num_points == 4) 1 else 2] += 1;
        } else counts[3] += 1;
    }

    errdefer std.debug.print("GJKClosestPoint.getClosestPoints: {d} beyond max distance, {d} full simplex, {d} within tolerance, {d} separated\n", .{ counts[0], counts[1], counts[2], counts[3] });
    for (counts) |c| try std.testing.expect(c > iterations / 50);
    try checker.finish();
}

const CastResult = struct {
    result: bool,
    lambda: f32,
    simplex: Simplex,
};

fn zoltCastRay(origin: P, direction: P, tolerance: f32, a_desc: *const ShapeDesc, lambda_in: f32) CastResult {
    const a = Shape.init(a_desc);
    var gjk: GJKClosestPoint = .{};
    var lambda = lambda_in;
    const result = gjk.castRay(vec3(origin), vec3(direction), tolerance, &a, &lambda);
    return .{ .result = result, .lambda = lambda, .simplex = storeSimplex(&gjk, .yp) };
}

fn checkCastRay(checker: *Checker, origin: P, direction: P, tolerance: f32, a: *const ShapeDesc, lambda: f32) CastResult {
    var expected = std.mem.zeroes(CastResult);
    expected.lambda = lambda;
    expected.result = jolt.jolt_gjk_cast_ray(&origin, &direction, tolerance, a, &expected.lambda, &expected.simplex);
    expected.simplex = masked(expected.simplex, .yp);

    const actual = zoltCastRay(origin, direction, tolerance, a, lambda);
    checker.check(.{ origin, direction, tolerance, a.*, lambda }, actual, expected);
    return actual;
}

/// Origins and directions of the hand picked rays: towards the objects, along them, zero length, from far away
const edge_rays = [_][2]P{
    .{ .{ -3, 0, 0 }, .{ 6, 0, 0 } },
    .{ .{ -3, 0.5, 0.5 }, .{ 6, 0, 0 } },
    .{ .{ 0, 0, 0 }, .{ 0, 0, 0 } },
    .{ .{ 0, 0, 0 }, .{ 1, 1, 1 } },
    .{ .{ 3, 3, 3 }, .{ -1, -1, -1 } },
    .{ .{ -3, 1, 1 }, .{ 6, 0, 0 } },
    .{ .{ 1.0e20, 0, 0 }, .{ -2.0e20, 0, 0 } },
    .{ .{ 0, 0, -1.0e-20 }, .{ 0, 0, 1.0e-20 } },
};

test "GJKClosestPoint.castRay" {
    var gen: Gen = .{ .rng = .{ .state = 0x2545f491 } };
    var checker: Checker = .{ .name = "GJKClosestPoint.castRay" };
    // Coverage: miss, hit at lambda 0 (start inside), hit at lambda > 0
    var counts: [3]u32 = @splat(0);

    for (edge_pairs) |pair| {
        for (edge_rays) |ray| {
            for ([_]f32{ 1.0 + math.flt_epsilon, std.math.inf(f32) }) |lambda| {
                for ([_]f32{ 0, 1.0e-4 }) |tolerance| {
                    _ = checkCastRay(&checker, ray[0], ray[1], tolerance, &pair.a, lambda);
                    _ = checkCastRay(&checker, ray[0], ray[1], tolerance, &pair.b, lambda);
                }
            }
        }
    }

    for (0..iterations) |_| {
        const s: f32 = if (gen.oneIn(6)) gen.scale() else 1;
        const a = gen.shape(s);
        const tolerance = gen.tolerance() * (if (gen.oneIn(2)) s else 1);
        const lambda = gen.lambda();

        // Rays from a random origin towards a random target, through the object or grazing it (towards a support point)
        const origin = if (gen.oneIn(4)) blk: {
            // Inside the object (or on its surface for flat objects): a convex combination of two support points
            const shape = Shape.init(&a);
            const d = gen.unit();
            const s1 = arr3(shape.getSupport(vec3(d)));
            const s2 = arr3(shape.getSupport(vec3(mulS(d, -1))));
            break :blk lerp(s1, s2, gen.pick(f32, &.{ 0.5, 0, 1, 0.25 }));
        } else mulS(gen.vec(-4, 4), s);
        const direction = switch (gen.next() % 4) {
            0 => gen.direction(s),
            1 => blk: {
                const shape = Shape.init(&a);
                const support = arr3(shape.getSupport(vec3(gen.unit())));
                break :blk mulS(sub(support, origin), gen.pick(f32, &.{ 1, 1.5, 0.5 }));
            },
            else => sub(mulS(gen.vec(-1, 1), s), origin),
        };

        const actual = checkCastRay(&checker, origin, direction, tolerance, &a, lambda);
        counts[if (!actual.result) 0 else if (actual.lambda == 0) 1 else 2] += 1;
    }

    errdefer std.debug.print("GJKClosestPoint.castRay: {d} misses, {d} hits at 0, {d} hits\n", .{ counts[0], counts[1], counts[2] });
    for (counts) |c| try std.testing.expect(c > iterations / 50);
    try checker.finish();
}

fn zoltCastShape(start: [16]f32, direction: P, tolerance: f32, a_desc: *const ShapeDesc, b_desc: *const ShapeDesc, lambda_in: f32) CastResult {
    const a = Shape.init(a_desc);
    const b = Shape.init(b_desc);
    var gjk: GJKClosestPoint = .{};
    var lambda = lambda_in;
    const result = gjk.castShape(loadMat44(start), vec3(direction), tolerance, &a, &b, &lambda);
    return .{ .result = result, .lambda = lambda, .simplex = storeSimplex(&gjk, .yp) };
}

fn checkCastShape(checker: *Checker, start: [16]f32, direction: P, tolerance: f32, a: *const ShapeDesc, b: *const ShapeDesc, lambda: f32) CastResult {
    var expected = std.mem.zeroes(CastResult);
    expected.lambda = lambda;
    expected.result = jolt.jolt_gjk_cast_shape(&start, &direction, tolerance, a, b, &expected.lambda, &expected.simplex);
    expected.simplex = masked(expected.simplex, .yp);

    const actual = zoltCastShape(start, direction, tolerance, a, b, lambda);
    checker.check(.{ a.*, b.*, start, direction, tolerance, lambda }, actual, expected);
    return actual;
}

/// Cast direction for a shape cast from start towards B
fn castDirection(gen: *Gen, start: [16]f32, b: *const ShapeDesc) P {
    const translation: P = start[12..15].*;
    return switch (gen.next() % 4) {
        0 => gen.direction(1),
        else => add(sub(b.data[0..3].*, translation), gen.vec(-1, 1)), // Roughly towards B
    };
}

/// Start transforms and directions of the hand picked shape casts
const edge_casts = blk: {
    @setEvalBranchQuota(100_000);
    break :blk [_]struct { start: [16]f32, direction: P }{
        .{ .start = identity_transform, .direction = .{ 0, 0, 0 } },
        .{ .start = identity_transform, .direction = .{ 1, 0, 0 } },
        .{ .start = storeMat44(Mat44.translation(Vec3.init(-4, 0, 0))), .direction = .{ 8, 0, 0 } },
        .{ .start = storeMat44(Mat44.translation(Vec3.init(-4, 0.5, 0.5))), .direction = .{ 8, 0, 0 } },
        .{ .start = storeMat44(Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.5 * math.pi), Vec3.init(0, 4, 0))), .direction = .{ 0, -8, 0 } },
        .{ .start = storeMat44(Mat44.translation(Vec3.init(-1.0e20, 0, 0))), .direction = .{ 2.0e20, 0, 0 } },
    };
};

test "GJKClosestPoint.castShape" {
    var gen: Gen = .{ .rng = .{ .state = 0x6a09e667 } };
    var checker: Checker = .{ .name = "GJKClosestPoint.castShape" };
    var counts: [3]u32 = @splat(0);

    for (edge_pairs) |pair| {
        for (edge_casts) |cast| {
            for ([_]f32{ 1.0 + math.flt_epsilon, std.math.inf(f32) }) |lambda| {
                _ = checkCastShape(&checker, cast.start, cast.direction, 1.0e-4, &pair.a, &pair.b, lambda);
                _ = checkCastShape(&checker, cast.start, cast.direction, 0, &pair.b, &pair.a, lambda);
            }
        }
    }

    for (0..iterations / 2) |_| {
        const pair = gen.pair();
        const start = gen.start();
        const direction = castDirection(&gen, start, &pair[1]);
        const tolerance = gen.tolerance();
        const lambda = gen.lambda();
        const actual = checkCastShape(&checker, start, direction, tolerance, &pair[0], &pair[1], lambda);
        counts[if (!actual.result) 0 else if (actual.lambda == 0) 1 else 2] += 1;
    }

    errdefer std.debug.print("GJKClosestPoint.castShape: {d} misses, {d} hits at 0, {d} hits\n", .{ counts[0], counts[1], counts[2] });
    for (counts) |c| try std.testing.expect(c > iterations / 100);
    try checker.finish();
}

const CastRadiusResult = struct {
    result: bool,
    lambda: f32,
    point_a: P,
    point_b: P,
    separating_axis: P,
    simplex: Simplex,
};

fn zoltCastShapeWithConvexRadius(start: [16]f32, direction: P, tolerance: f32, a_desc: *const ShapeDesc, b_desc: *const ShapeDesc, convex_radius_a: f32, convex_radius_b: f32, lambda_in: f32) CastRadiusResult {
    const a = Shape.init(a_desc);
    const b = Shape.init(b_desc);
    var gjk: GJKClosestPoint = .{};
    var lambda = lambda_in;
    var point_a = vec3(sentinel_p);
    var point_b = vec3(sentinel_p);
    var separating_axis = vec3(sentinel_p);
    const result = gjk.castShapeWithConvexRadius(loadMat44(start), vec3(direction), tolerance, &a, &b, convex_radius_a, convex_radius_b, &lambda, &point_a, &point_b, &separating_axis);
    return .{ .result = result, .lambda = lambda, .point_a = arr3(point_a), .point_b = arr3(point_b), .separating_axis = arr3(separating_axis), .simplex = storeSimplex(&gjk, .ypq) };
}

fn checkCastShapeWithConvexRadius(checker: *Checker, start: [16]f32, direction: P, tolerance: f32, a: *const ShapeDesc, b: *const ShapeDesc, convex_radius_a: f32, convex_radius_b: f32, lambda: f32) CastRadiusResult {
    var expected = std.mem.zeroes(CastRadiusResult);
    expected.lambda = lambda;
    expected.point_a = sentinel_p;
    expected.point_b = sentinel_p;
    expected.separating_axis = sentinel_p;
    expected.result = jolt.jolt_gjk_cast_shape_radius(&start, &direction, tolerance, a, b, convex_radius_a, convex_radius_b, &expected.lambda, &expected.point_a, &expected.point_b, &expected.separating_axis, &expected.simplex);
    expected.simplex = masked(expected.simplex, .ypq);

    const actual = zoltCastShapeWithConvexRadius(start, direction, tolerance, a, b, convex_radius_a, convex_radius_b, lambda);
    checker.check(.{ a.*, b.*, start, direction, tolerance, lambda, convex_radius_a, convex_radius_b }, actual, expected);
    return actual;
}

test "GJKClosestPoint.castShapeWithConvexRadius" {
    var gen: Gen = .{ .rng = .{ .state = 0xbb67ae85 } };
    var checker: Checker = .{ .name = "GJKClosestPoint.castShapeWithConvexRadius" };
    var counts: [3]u32 = @splat(0);

    for (edge_pairs) |pair| {
        for (edge_casts) |cast| {
            for ([_]f32{ 1.0 + math.flt_epsilon, std.math.inf(f32) }) |lambda| {
                for ([_][2]f32{ .{ 0, 0 }, .{ 0.1, 0 }, .{ 0.05, 0.5 } }) |radius| {
                    _ = checkCastShapeWithConvexRadius(&checker, cast.start, cast.direction, 1.0e-4, &pair.a, &pair.b, radius[0], radius[1], lambda);
                    _ = checkCastShapeWithConvexRadius(&checker, cast.start, cast.direction, 0, &pair.b, &pair.a, radius[0], radius[1], lambda);
                }
            }
        }
    }

    for (0..iterations / 2) |_| {
        const pair = gen.pair();
        const start = gen.start();
        const direction = castDirection(&gen, start, &pair[1]);
        const tolerance = gen.tolerance();
        const lambda = gen.lambda();
        const convex_radius_a = gen.convexRadius();
        const convex_radius_b = gen.convexRadius();
        const actual = checkCastShapeWithConvexRadius(&checker, start, direction, tolerance, &pair[0], &pair[1], convex_radius_a, convex_radius_b, lambda);
        counts[if (!actual.result) 0 else if (actual.lambda == 0) 1 else 2] += 1;
    }

    errdefer std.debug.print("GJKClosestPoint.castShapeWithConvexRadius: {d} misses, {d} hits at 0, {d} hits\n", .{ counts[0], counts[1], counts[2] });
    for (counts) |c| try std.testing.expect(c > iterations / 100);
    try checker.finish();
}

// ---------------------------------------------------------------------------------------------------------------------
// EPAPenetrationDepth

const EPAStepsResult = struct {
    status: i32,
    gjk_v: P,
    gjk_point_a: P,
    gjk_point_b: P,
    epa_result: i32,
    v: P,
    point_a: P,
    point_b: P,
};

/// Zolt version of jolt_epa_penetration_depth_steps, also returns the size of the GJK simplex (for the coverage statistics)
fn zoltEPASteps(a_desc: *const ShapeDesc, convex_radius_a: f32, b_desc: *const ShapeDesc, convex_radius_b: f32, include_mode: c_int, collision_tolerance: f32, penetration_tolerance: f32, v_in: P, out_simplex_size: *u32) EPAStepsResult {
    const a = Shape.init(a_desc);
    const b = Shape.init(b_desc);
    const a_incl = AddConvexRadius(Shape).init(&a, convex_radius_a);
    const b_incl = AddConvexRadius(Shape).init(&b, convex_radius_b);
    var epa: EPAPenetrationDepth = .{};
    var v = vec3(v_in);
    var point_a = vec3(sentinel_p);
    var point_b = vec3(sentinel_p);
    const status = epa.getPenetrationDepthStepGJK(&a, convex_radius_a, &b, convex_radius_b, collision_tolerance, &v, &point_a, &point_b);
    out_simplex_size.* = epa.gjk.num_points;
    var result: EPAStepsResult = undefined;
    result.status = @intFromEnum(status);
    result.gjk_v = arr3(v);
    result.gjk_point_a = arr3(point_a);
    result.gjk_point_b = arr3(point_b);
    result.epa_result = -1;
    if (status == .indeterminate) {
        const epa_result = if (include_mode == 0)
            epa.getPenetrationDepthStepEPA(&a_incl, &b_incl, penetration_tolerance, &v, &point_a, &point_b)
        else
            epa.getPenetrationDepthStepEPA(&a, &b, penetration_tolerance, &v, &point_a, &point_b);
        result.epa_result = @intFromBool(epa_result);
    }
    result.v = arr3(v);
    result.point_a = arr3(point_a);
    result.point_b = arr3(point_b);
    return result;
}

fn checkEPASteps(checker: *Checker, a: *const ShapeDesc, convex_radius_a: f32, b: *const ShapeDesc, convex_radius_b: f32, include_mode: c_int, collision_tolerance: f32, penetration_tolerance: f32, v: P, out_simplex_size: *u32) EPAStepsResult {
    var expected: EPAStepsResult = undefined;
    expected.v = v;
    expected.point_a = sentinel_p;
    expected.point_b = sentinel_p;
    var epa_result: c_int = undefined;
    expected.status = jolt.jolt_epa_penetration_depth_steps(a, convex_radius_a, b, convex_radius_b, include_mode, collision_tolerance, penetration_tolerance, &expected.v, &expected.point_a, &expected.point_b, &expected.gjk_v, &expected.gjk_point_a, &expected.gjk_point_b, &epa_result);
    expected.epa_result = epa_result;

    const actual = zoltEPASteps(a, convex_radius_a, b, convex_radius_b, include_mode, collision_tolerance, penetration_tolerance, v, out_simplex_size);
    checker.check(.{ a.*, b.*, convex_radius_a, convex_radius_b, include_mode, collision_tolerance, penetration_tolerance, v }, actual, expected);
    return actual;
}

test "EPAPenetrationDepth steps" {
    var gen: Gen = .{ .rng = .{ .state = 0x3c6ef372 } };
    var checker: Checker = .{ .name = "EPAPenetrationDepth steps" };
    // Coverage: GJK status (not colliding, colliding, indeterminate), EPA result per simplex size of the GJK step (1 .. 4)
    var status_counts: [3]u32 = @splat(0);
    var epa_counts: [5][2]u32 = @splat(.{ 0, 0 });

    var simplex_size: u32 = 0;
    for (edge_pairs) |pair| {
        for ([_]P{ .{ 1, 0, 0 }, .{ 0, -1, 0 }, .{ 1, 1, 1 } }) |v| {
            for ([_][2]f32{ .{ 0, 0 }, .{ 0.1, 0 }, .{ 0.05, 0.5 } }) |radius| {
                for ([_]c_int{ 0, 1 }) |include_mode| {
                    _ = checkEPASteps(&checker, &pair.a, radius[0], &pair.b, radius[1], include_mode, 1.0e-4, math.flt_epsilon, v, &simplex_size);
                    _ = checkEPASteps(&checker, &pair.b, radius[0], &pair.a, radius[1], include_mode, 1.0e-3, 1.0e-3, v, &simplex_size);
                }
            }
        }
    }

    for (0..iterations / 2) |_| {
        const pair = gen.pair();
        const convex_radius_a = gen.convexRadius();
        const convex_radius_b = gen.convexRadius();
        const include_mode: c_int = if (gen.oneIn(4)) 1 else 0;
        const collision_tolerance = gen.pick(f32, &.{ 1.0e-4, 1.0e-4, 1.0e-3, 1.0e-2 });
        const penetration_tolerance = gen.penetrationTolerance();
        const v = gen.initialVNonZero(&pair[0], &pair[1]);
        const actual = checkEPASteps(&checker, &pair[0], convex_radius_a, &pair[1], convex_radius_b, include_mode, collision_tolerance, penetration_tolerance, v, &simplex_size);

        status_counts[@intCast(actual.status)] += 1;
        if (actual.epa_result >= 0)
            epa_counts[simplex_size][@intCast(actual.epa_result)] += 1;
    }

    errdefer std.debug.print("EPAPenetrationDepth steps: status {any}, EPA result (false, true) per GJK simplex size {any}\n", .{ status_counts, epa_counts });
    for (status_counts) |c| try std.testing.expect(c > iterations / 100);
    for (epa_counts[1..]) |c| try std.testing.expect(c[0] + c[1] > 0);
    try checker.finish();
}

const PenetrationDepthResult = struct {
    result: bool,
    v: P,
    point_a: P,
    point_b: P,
};

fn zoltGetPenetrationDepth(a_desc: *const ShapeDesc, convex_radius_a: f32, b_desc: *const ShapeDesc, convex_radius_b: f32, include_mode: c_int, collision_tolerance_sq: f32, penetration_tolerance: f32, v_in: P) PenetrationDepthResult {
    const a = Shape.init(a_desc);
    const b = Shape.init(b_desc);
    const a_incl = AddConvexRadius(Shape).init(&a, convex_radius_a);
    const b_incl = AddConvexRadius(Shape).init(&b, convex_radius_b);
    var epa: EPAPenetrationDepth = .{};
    var v = vec3(v_in);
    var point_a = vec3(sentinel_p);
    var point_b = vec3(sentinel_p);
    const result = if (include_mode == 0)
        epa.getPenetrationDepth(&a, &a_incl, convex_radius_a, &b, &b_incl, convex_radius_b, collision_tolerance_sq, penetration_tolerance, &v, &point_a, &point_b)
    else
        epa.getPenetrationDepth(&a, &a, convex_radius_a, &b, &b, convex_radius_b, collision_tolerance_sq, penetration_tolerance, &v, &point_a, &point_b);
    return .{ .result = result, .v = arr3(v), .point_a = arr3(point_a), .point_b = arr3(point_b) };
}

fn checkGetPenetrationDepth(checker: *Checker, a: *const ShapeDesc, convex_radius_a: f32, b: *const ShapeDesc, convex_radius_b: f32, include_mode: c_int, collision_tolerance_sq: f32, penetration_tolerance: f32, v: P) PenetrationDepthResult {
    var expected: PenetrationDepthResult = undefined;
    expected.v = v;
    expected.point_a = sentinel_p;
    expected.point_b = sentinel_p;
    expected.result = jolt.jolt_epa_get_penetration_depth(a, convex_radius_a, b, convex_radius_b, include_mode, collision_tolerance_sq, penetration_tolerance, &expected.v, &expected.point_a, &expected.point_b);

    const actual = zoltGetPenetrationDepth(a, convex_radius_a, b, convex_radius_b, include_mode, collision_tolerance_sq, penetration_tolerance, v);
    checker.check(.{ a.*, b.*, convex_radius_a, convex_radius_b, include_mode, collision_tolerance_sq, penetration_tolerance, v }, actual, expected);
    return actual;
}

test "EPAPenetrationDepth.getPenetrationDepth" {
    var gen: Gen = .{ .rng = .{ .state = 0xa54ff53a } };
    var checker: Checker = .{ .name = "EPAPenetrationDepth.getPenetrationDepth" };
    var counts: [2]u32 = @splat(0);

    for (edge_pairs) |pair| {
        for ([_][2]f32{ .{ 0, 0 }, .{ 0.1, 0 }, .{ 0.05, 0.5 } }) |radius| {
            for ([_]c_int{ 0, 1 }) |include_mode| {
                _ = checkGetPenetrationDepth(&checker, &pair.a, radius[0], &pair.b, radius[1], include_mode, 1.0e-8, math.flt_epsilon, .{ 1, 0, 0 });
                _ = checkGetPenetrationDepth(&checker, &pair.b, radius[0], &pair.a, radius[1], include_mode, 1.0e-4, 1.0e-3, .{ 0, 1, 0 });
            }
        }
    }

    for (0..iterations / 4) |_| {
        const pair = gen.pair();
        const convex_radius_a = gen.convexRadius();
        const convex_radius_b = gen.convexRadius();
        const include_mode: c_int = if (gen.oneIn(4)) 1 else 0;
        const collision_tolerance_sq = gen.pick(f32, &.{ 1.0e-8, 1.0e-4, 1.0e-2 });
        const penetration_tolerance = gen.penetrationTolerance();
        const v = gen.initialVNonZero(&pair[0], &pair[1]);
        const actual = checkGetPenetrationDepth(&checker, &pair[0], convex_radius_a, &pair[1], convex_radius_b, include_mode, collision_tolerance_sq, penetration_tolerance, v);
        counts[@intFromBool(actual.result)] += 1;
    }

    errdefer std.debug.print("EPAPenetrationDepth.getPenetrationDepth: {d} not colliding, {d} colliding\n", .{ counts[0], counts[1] });
    for (counts) |c| try std.testing.expect(c > iterations / 100);
    try checker.finish();
}

const EPACastResult = struct {
    result: bool,
    lambda: f32,
    point_a: P,
    point_b: P,
    contact_normal: P,
};

fn zoltEPACastShape(start: [16]f32, direction: P, collision_tolerance: f32, penetration_tolerance: f32, a_desc: *const ShapeDesc, b_desc: *const ShapeDesc, convex_radius_a: f32, convex_radius_b: f32, return_deepest_point: bool, lambda_in: f32) EPACastResult {
    const a = Shape.init(a_desc);
    const b = Shape.init(b_desc);
    var epa: EPAPenetrationDepth = .{};
    var lambda = lambda_in;
    var point_a = vec3(sentinel_p);
    var point_b = vec3(sentinel_p);
    var contact_normal = vec3(sentinel_p);
    const result = epa.castShape(loadMat44(start), vec3(direction), collision_tolerance, penetration_tolerance, &a, &b, convex_radius_a, convex_radius_b, return_deepest_point, &lambda, &point_a, &point_b, &contact_normal);
    return .{ .result = result, .lambda = lambda, .point_a = arr3(point_a), .point_b = arr3(point_b), .contact_normal = arr3(contact_normal) };
}

fn checkEPACastShape(checker: *Checker, start: [16]f32, direction: P, collision_tolerance: f32, penetration_tolerance: f32, a: *const ShapeDesc, b: *const ShapeDesc, convex_radius_a: f32, convex_radius_b: f32, return_deepest_point: bool, lambda: f32) EPACastResult {
    var expected: EPACastResult = undefined;
    expected.lambda = lambda;
    expected.point_a = sentinel_p;
    expected.point_b = sentinel_p;
    expected.contact_normal = sentinel_p;
    // (bool parameters are passed as int: a Zig bool argument was not received correctly by the C++ code)
    expected.result = jolt.jolt_epa_cast_shape(&start, &direction, collision_tolerance, penetration_tolerance, a, b, convex_radius_a, convex_radius_b, @intFromBool(return_deepest_point), &expected.lambda, &expected.point_a, &expected.point_b, &expected.contact_normal);

    const actual = zoltEPACastShape(start, direction, collision_tolerance, penetration_tolerance, a, b, convex_radius_a, convex_radius_b, return_deepest_point, lambda);
    checker.check(.{ a.*, b.*, start, direction, collision_tolerance, penetration_tolerance, lambda, convex_radius_a, convex_radius_b, return_deepest_point }, actual, expected);
    return actual;
}

test "EPAPenetrationDepth.castShape" {
    var gen: Gen = .{ .rng = .{ .state = 0x510e527f } };
    var checker: Checker = .{ .name = "EPAPenetrationDepth.castShape" };
    var counts: [3]u32 = @splat(0);

    for (edge_pairs) |pair| {
        if (pair.huge) continue;
        for (edge_casts) |cast| {
            if (cast.start[12] < -1.0e10) continue; // Huge
            for ([_][2]f32{ .{ 0, 0 }, .{ 0.1, 0 }, .{ 0.05, 0.5 } }) |radius| {
                for ([_]bool{ false, true }) |return_deepest_point| {
                    _ = checkEPACastShape(&checker, cast.start, cast.direction, 1.0e-4, math.flt_epsilon, &pair.a, &pair.b, radius[0], radius[1], return_deepest_point, 1.0 + math.flt_epsilon);
                    _ = checkEPACastShape(&checker, cast.start, cast.direction, 1.0e-3, 1.0e-3, &pair.b, &pair.a, radius[0], radius[1], return_deepest_point, std.math.inf(f32));
                }
            }
        }
    }

    for (0..iterations / 2) |_| {
        const pair = gen.pair();
        const start = if (gen.oneIn(3)) identity_transform else gen.start();
        const direction = castDirection(&gen, start, &pair[1]);
        const collision_tolerance = gen.pick(f32, &.{ 1.0e-4, 1.0e-4, 1.0e-3, 1.0e-2 });
        const penetration_tolerance = gen.penetrationTolerance();
        const lambda = gen.lambda();
        const convex_radius_a = gen.convexRadius();
        const convex_radius_b = gen.convexRadius();
        const return_deepest_point = !gen.oneIn(4);
        const actual = checkEPACastShape(&checker, start, direction, collision_tolerance, penetration_tolerance, &pair[0], &pair[1], convex_radius_a, convex_radius_b, return_deepest_point, lambda);
        counts[if (!actual.result) 0 else if (actual.lambda == 0) 1 else 2] += 1;
    }

    errdefer std.debug.print("EPAPenetrationDepth.castShape: {d} misses, {d} hits at 0, {d} hits\n", .{ counts[0], counts[1], counts[2] });
    for (counts) |c| try std.testing.expect(c > iterations / 100);
    try checker.finish();
}

// ---------------------------------------------------------------------------------------------------------------------
// EPAConvexHullBuilder

/// Record of the hull builder test, must match HullRecord in GJKEPAReference.cpp
const HullRecord = extern struct {
    /// 0 = findFacingTriangle, 1 = addPoint, 2 = new triangle, 3 = peeked triangle, 4 = popped triangle
    type: c_int,
    /// findFacingTriangle: 1 if a triangle was found; addPoint: the result
    value_int: c_int,
    /// findFacingTriangle: the best distance^2; addPoint: number of new triangles
    value: f32,
    start_idx: [3]c_int,
    /// -1 if there is no neighbour
    neighbour_edge: [3]c_int,
    neighbour_start_idx: [3][3]c_int,
    normal: P,
    centroid: P,
    closest_len_sq: f32,
    /// Only when the closest point is interior
    lambda: [2]f32,
    /// 1 = lambda relative to 0 (only when interior), 2 = closest point interior, 4 = removed, 8 = in queue
    flags: c_int,
};

const Triangle = EPAConvexHullBuilder.Triangle;

/// Upper bound of the number of records of one hull test
const max_hull_records = (EPAConvexHullBuilder.max_points - 3) * (4 + EPAConvexHullBuilder.max_edge_length) + EPAConvexHullBuilder.max_triangles;

fn storeTriangle(record_type: c_int, t: *const Triangle) HullRecord {
    var r = std.mem.zeroes(HullRecord);
    r.type = record_type;
    for (0..3) |e| {
        const edge = &t.edge[e];
        r.start_idx[e] = @intCast(edge.start_idx);
        if (edge.neighbour_triangle) |n| {
            r.neighbour_edge[e] = @intCast(edge.neighbour_edge);
            for (0..3) |k|
                r.neighbour_start_idx[e][k] = @intCast(n.edge[k].start_idx);
        } else {
            r.neighbour_edge[e] = -1;
            r.neighbour_start_idx[e] = .{ -1, -1, -1 };
        }
    }
    r.normal = arr3(t.normal);
    r.centroid = arr3(t.centroid);
    r.closest_len_sq = t.closest_len_sq;
    if (t.closest_point_interior) {
        r.lambda = t.lambda;
        r.flags |= if (t.lambda_relative_to_0) 1 else 0;
        r.flags |= 2;
    }
    r.flags |= if (t.removed) 4 else 0;
    r.flags |= if (t.in_queue) 8 else 0;
    return r;
}

/// Zolt version of jolt_epa_hull
fn zoltHull(points_in: []const P, closest_dist_sq: []const f32, pop_every: u32, out_records: []HullRecord) usize {
    var points: EPAConvexHullBuilder.Points = .empty;
    for (points_in) |p|
        points.append(vec3(p));

    var num_records: usize = 0;
    var hull = EPAConvexHullBuilder.init(&points);
    hull.initialize(0, 1, 2);
    var i: u32 = 3;
    while (i < points.len) : (i += 1) {
        const facing = hull.findFacingTriangle(points.get(i));
        var record = if (facing.triangle) |t| storeTriangle(0, t) else std.mem.zeroes(HullRecord);
        record.value_int = @intFromBool(facing.triangle != null);
        record.value = facing.best_dist_sq;
        out_records[num_records] = record;
        num_records += 1;

        if (facing.triangle) |t| {
            var new_triangles: EPAConvexHullBuilder.NewTriangles = .empty;
            const ok = hull.addPoint(t, i, closest_dist_sq[i], &new_triangles);
            var add_record = std.mem.zeroes(HullRecord);
            add_record.type = 1;
            add_record.value_int = @intFromBool(ok);
            add_record.value = @floatFromInt(new_triangles.len);
            out_records[num_records] = add_record;
            num_records += 1;
            for (new_triangles.constSlice()) |nt| {
                out_records[num_records] = storeTriangle(2, nt);
                num_records += 1;
            }
            if (!ok)
                return num_records; // The hull is broken after a failed addPoint
        }

        if (pop_every > 0 and i % pop_every == 0 and hull.hasNextTriangle()) {
            out_records[num_records] = storeTriangle(3, hull.peekClosestTriangleInQueue());
            num_records += 1;
            const pt = hull.popClosestTriangleFromQueue();
            out_records[num_records] = storeTriangle(4, pt);
            num_records += 1;
            if (pt.removed)
                hull.freeTriangle(pt);
        }
    }

    while (hull.hasNextTriangle()) {
        const pt = hull.popClosestTriangleFromQueue();
        out_records[num_records] = storeTriangle(4, pt);
        num_records += 1;
        if (pt.removed)
            hull.freeTriangle(pt);
    }

    return num_records;
}

test "EPAConvexHullBuilder" {
    var gen: Gen = .{ .rng = .{ .state = 0x1f83d9ab } };
    var checker: Checker = .{ .name = "EPAConvexHullBuilder" };
    const expected_records = try std.testing.allocator.alloc(HullRecord, max_hull_records);
    defer std.testing.allocator.free(expected_records);
    const actual_records = try std.testing.allocator.alloc(HullRecord, max_hull_records);
    defer std.testing.allocator.free(actual_records);
    var add_point_failures: u32 = 0;
    var total_records: usize = 0;

    var points: [EPAConvexHullBuilder.max_points]P = undefined;
    var closest_dist_sq: [EPAConvexHullBuilder.max_points]f32 = undefined;
    for (0..iterations / 20) |_| {
        const count = 4 + gen.index(if (gen.oneIn(4)) EPAConvexHullBuilder.max_points - 3 else 40);
        const pts = points[0..count];
        switch (gen.next() % 10) {
            // Points on a sphere around the origin (like EPA on spheres)
            0, 1 => for (pts) |*p| {
                p.* = gen.unit();
            },
            // Integer coordinates (ties in the queue order, coplanar triangles, duplicates)
            2 => for (pts) |*p| {
                p.* = gen.gridVec(2);
            },
            // A box around the origin and points inside / outside it
            3 => for (pts, 0..) |*p, i| {
                p.* = if (i < 8) .{ if (i & 1 != 0) 1 else -1, if (i & 2 != 0) 1 else -1, if (i & 4 != 0) 1 else -1 } else mulS(gen.vec(-1, 1), 1.5);
            },
            // Origin outside of the hull
            4 => {
                const offset = mulS(gen.unit(), gen.rng.float(1, 3));
                for (pts) |*p| p.* = add(gen.vec(-1, 1), offset);
            },
            // Nearly flat
            5 => for (pts) |*p| {
                p.* = .{ gen.float(-1, 1), gen.float(-1, 1), gen.rng.float(-1.0e-4, 1.0e-4) };
            },
            // A tetrahedron, then points (nearly) in the plane of one of its faces (coplanar triangles, islands in findEdge)
            6 => {
                const noise = gen.pick(f32, &.{ 0, 1.0e-7, 1.0e-6, 1.0e-5 });
                for (pts, 0..) |*p, i| {
                    p.* = switch (i) {
                        0 => .{ -2, -2, 0 },
                        1 => .{ 2, -2, 0 },
                        2 => .{ 0, 2, 0 },
                        3 => .{ 0, 0, if (gen.oneIn(2)) 1 else -1 },
                        else => .{ if (gen.oneIn(2)) gen.grid(3) * 0.5 else gen.rng.float(-1.5, 1.5), if (gen.oneIn(2)) gen.grid(3) * 0.5 else gen.rng.float(-1.5, 1.5), gen.rng.float(-noise, noise) },
                    };
                }
            },
            // Like the previous case but in a tilted plane: the vertices are rounded off the plane, so the (nearly) coplanar
            // triangles disagree on which side of them a new point in the plane is (islands in findEdge)
            7 => {
                const n = gen.unit();
                const e1 = gen.unit();
                const e2: P = .{ n[1] * e1[2] - n[2] * e1[1], n[2] * e1[0] - n[0] * e1[2], n[0] * e1[1] - n[1] * e1[0] };
                const noise = gen.pick(f32, &.{ 0, 1.0e-7 });
                for (pts, 0..) |*p, i| {
                    const uv: [2]f32 = switch (i) {
                        0 => .{ -2, -2 },
                        1 => .{ 2, -2 },
                        2 => .{ 0, 2 },
                        3 => .{ 0, 0 },
                        else => .{ gen.rng.float(-1.5, 1.5), gen.rng.float(-1.5, 1.5) },
                    };
                    p.* = add(add(mulS(e1, uv[0]), mulS(e2, uv[1])), mulS(n, if (i == 3) 1 else gen.rng.float(-noise, noise)));
                }
            },
            else => for (pts) |*p| {
                p.* = gen.vec(-1, 1);
            },
        }
        if (gen.oneIn(5)) pts[gen.index(count)] = pts[gen.index(count)]; // Duplicates
        const s: f32 = if (gen.oneIn(5)) gen.scale() else 1;
        for (pts) |*p| p.* = mulS(p.*, s);
        for (closest_dist_sq[0..count]) |*d|
            d.* = if (gen.oneIn(3)) gen.rng.float(-0.1, 1) * s * s else math.flt_max;
        const pop_every: u32 = if (gen.oneIn(2)) 0 else @intCast(1 + gen.index(4));

        const num_expected: usize = @intCast(jolt.jolt_epa_hull(pts.ptr, @intCast(count), &closest_dist_sq, @intCast(pop_every), expected_records.ptr));
        const num_actual = zoltHull(pts, closest_dist_sq[0..count], pop_every, actual_records);
        checker.check(.{ count, pop_every, pts[0] }, num_actual, num_expected);
        for (expected_records[0..@min(num_expected, num_actual)], actual_records[0..@min(num_expected, num_actual)], 0..) |e, a, i| {
            if (!fw.sameValue(a, e)) {
                checker.check(.{ count, pop_every, i }, a, e);
                break;
            }
        }
        total_records += num_actual;
        for (actual_records[0..num_actual]) |r| {
            if (r.type == 1 and r.value_int == 0) add_point_failures += 1;
        }
    }

    errdefer std.debug.print("EPAConvexHullBuilder: {d} records, {d} failed addPoint\n", .{ total_records, add_point_failures });
    try std.testing.expect(add_point_failures > 0);
    try checker.finish();
}
