//! Parity tests for the shape core (Phase 4 foundation, part 2): ScaleHelpers, GetTrianglesContextVertexList /
//! MultiVertexList and the vertex list helpers, ShapeCast / RShapeCast / ShapeCastResult, TransformedShape and the
//! default implementations of Shape (with a test shape that derives from Shape directly, the same class on both sides),
//! the collision collectors on synthetic hit sequences (early out fractions, the sort order with ties, ClosestHitPerBody),
//! the binary state of a shape graph (Shape::SaveWithChildren), CollisionDispatch (collide / cast in world and local
//! space, the reversed functions) and the TransformedShape queries that go through it (CollideShape, CastShape,
//! GetTrianglesStart, GetSupportingFace, CollectTransformedShapes, CollidePoint through Shape::sCollidePointUsingRayCast)
//! and the contents of the CollisionDispatch / ShapeFunctions tables. Zolt and the C++ Jolt library run on the same
//! inputs and must produce identical bits. C ABI wrappers: ZoltParity/Physics/ShapeCoreReference.cpp. See
//! ZoltParity/parity.zig for how parity tests work.
//!
//! The test shape and the collide / cast functions registered for it (User1 / User2, with reversed entries) are in
//! ShapeCoreUserTypes.zig, the `zolt_user_types` module of the parity build; the C++ reference registers the same
//! functions. They record what they receive, so the transforms, shape casts and sub shape IDs computed by the entry
//! points are compared, not only the results.
//!
//! The dispatch table test registers, on the C++ side, the classes of Jolt's RegisterTypes order whose Zolt port
//! registers something (a stub registers nothing) and then the parity user registrations, so it compares Jolt's final
//! table restricted to the ported classes and covers more of the table with every shape that is ported.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const Allocator = std.mem.Allocator;
const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const Body = zolt.Body;
const CastRayCollector = zolt.CastRayCollector;
const CastShapeCollector = zolt.CastShapeCollector;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const ClosestHitPerBodyCollisionCollector = zolt.ClosestHitPerBodyCollisionCollector;
const CollidePointCollector = zolt.CollidePointCollector;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollisionDispatch = zolt.CollisionDispatch;
const Color = zolt.Color;
const Core = zolt.Core;
const DMat44 = zolt.DMat44;
const Float3 = zolt.Float3;
const GetTrianglesContextMultiVertexList = zolt.GetTrianglesContextMultiVertexList;
const GetTrianglesContextVertexList = zolt.GetTrianglesContextVertexList;
const Mat44 = zolt.Mat44;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsMaterialSimple = zolt.PhysicsMaterialSimple;
const Quat = zolt.Quat;
const RayCastResult = zolt.RayCastResult;
const Real = zolt.Real;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RegisterTypes = zolt.RegisterTypes;
const RMat44 = zolt.RMat44;
const RShapeCast = zolt.RShapeCast;
const RVec3 = zolt.RVec3;
const ScaleHelpers = zolt.ScaleHelpers;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastResult = zolt.ShapeCastResult;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeFilter = zolt.ShapeFilter;
const ShapeSubType = zolt.ShapeSubType;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TransformedShape = zolt.TransformedShape;
const TransformedShapeCollector = zolt.TransformedShapeCollector;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const VertexArrayList = zolt.VertexArrayList;

const num_sub_shape_types = zolt.num_sub_shape_types;

/// The C++ reference functions, see ShapeCoreReference.cpp
const jolt = struct {
    extern fn jolt_scale_helpers(scale: *const P, convex_radius: f32, rotation: *const [4]f32, out_bools: *[6]c_int, out_vectors: *[12]f32) f32;
    extern fn jolt_triangles_vertex_list_helper(which: c_int, detail_level: c_int, out_vertices: [*]f32, capacity: u32) u32;
    extern fn jolt_triangles_vertex_list(position_com: *const P, rotation: *const [4]f32, scale: *const P, local_transform: *const [16]f32, vertices: [*]const f32, num_vertices: u32, max_triangles_requested: c_int, out_counts: *[64]c_int, out_vertices: [*]f32) c_int;
    extern fn jolt_triangles_multi_vertex_list(inside_out: c_int, num_parts: c_int, transforms: *const [48]f32, vertices: [*]const f32, part_sizes: *const [3]u32, max_triangles_requested: c_int, out_counts: *[64]c_int, out_vertices: [*]f32) c_int;
    extern fn jolt_shape_cast(half_extent: *const P, center_of_mass: *const P, scale: *const P, start: *const [16]f32, direction: *const P, transform: *const [16]f32, translation: *const P, fraction: f32, out_casts: *[100]f32, out_point: *P) void;
    extern fn jolt_r_shape_cast(half_extent: *const P, center_of_mass: *const P, scale: *const P, start_columns: *const [12]f32, start_translation: *const R3, direction: *const P, translation: *const R3, fraction: f32, out_columns: *[36]f32, out_translations: *[9]Real, out_rest: *[27]f32, out_point: *R3, out_shape_cast: *[25]f32) void;
    extern fn jolt_shape_cast_result(fraction: f32, contact1: *const P, contact2: *const P, axis: *const P, back_face: c_int, id1: u32, id2: u32, body_id: u32, face1: [*]const f32, face1_count: u32, face2: [*]const f32, face2_count: u32, direction: *const P, out_values: *[14]f32, out_ints: *[6]u32, out_faces: *[192]f32) void;
    extern fn jolt_transformed_shape(half_extent: *const P, center_of_mass: *const P, uniform_scale: c_int, position: *const R3, rotation: *const [4]f32, scale: *const P, sub_shape_id: u32, sub_shape_id_bits: u32, world_columns: *const [12]f32, world_translation: *const R3, shape_transform: *const [16]f32, ray_origin: *const R3, ray_direction: *const P, point: *const R3, out_matrix_columns: *[36]f32, out_matrix_translations: *[9]Real, out_ts: *[5]TS, out_vectors: *[19]f32) void;
    extern fn jolt_save_with_children(num_shapes: c_int, half_extents: [*]const f32, user_data: [*]const u64, children: [*]const c_int, materials: [*]const c_int, num_materials: c_int, material_colors: [*]const u32, roots: [*]const c_int, num_roots: c_int, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_collector(kind: c_int, result_type: c_int, num_hits: c_int, bodies: [*]const u32, fractions: [*]const f32, depths: [*]const f32, out_early_out: [*]f32, out_hits: [*]Hit) c_int;
    extern fn jolt_dispatch_queries(input: *const DispatchInput, output: *DispatchOutput) void;
    extern fn jolt_dispatch_tables(mask: u32, out_collide: *[num_sub_shape_types * num_sub_shape_types]c_int, out_cast: *[num_sub_shape_types * num_sub_shape_types]c_int, out_construct: *[num_sub_shape_types]c_int, out_color: *[num_sub_shape_types]u32) void;
};

/// Number of random inputs per test
const iterations = fw.iterations;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// An RVec3 as passed to the C ABI
const R3 = [3]Real;

/// TransformedShape in the format of the C ABI, must match TS in ShapeCoreReference.cpp
const TS = extern struct {
    position_com: R3,
    rotation: [4]f32,
    scale: P,
    body_id: u32,
    sub_shape_id: u32,
    sub_shape_id_bits: u32,

    fn fromTransformedShape(ts: *const TransformedShape) TS {
        return .{
            .position_com = arrR3(ts.shape_position_com),
            .rotation = arr4(ts.shape_rotation.getXYZW()),
            .scale = arr3(ts.getShapeScale()),
            .body_id = ts.body_id.getIndexAndSequenceNumber(),
            .sub_shape_id = ts.sub_shape_id_creator.getID().getValue(),
            .sub_shape_id_bits = ts.sub_shape_id_creator.getNumBitsWritten(),
        };
    }
};

/// Inputs of the dispatch / TransformedShape query test, must match DispatchInput in ShapeCoreReference.cpp
const DispatchInput = extern struct {
    /// Shape A and B: half extent, center of mass, sub type (0: User1, 1: User2)
    half_extents: [2]P,
    centers_of_mass: [2]P,
    sub_types: [2]u32,
    /// Penetration depths (collide) / fractions (cast) of the 2 hits that the registered functions add
    hit_values: [2]f32,
    /// CollisionDispatch: scales and center of mass transforms of A and B (cast: A starts at transforms[0]), sub shape ID
    /// creators (ID, bits), cast direction
    scales: [2]P,
    transforms: [2][16]f32,
    creators: [2][2]u32,
    direction: P,
    /// TransformedShape of B (body 7, sub shape ID creator creators[1])
    position: R3,
    rotation: [4]f32,
    ts_scale: P,
    /// RMat44 of A for TransformedShape::CollideShape, start of the RShapeCast of A for TransformedShape::CastShape
    query_columns: [12]f32,
    query_translation: R3,
    base_offset: R3,
    /// World space box of CollectTransformedShapes / GetTrianglesStart
    box: [6]f32,
    /// GetSupportingFace: the sub shape ID pushed on the creator of the TransformedShape (value, bits), the direction
    face_id: [2]u32,
    face_direction: P,
    /// CollidePoint through Shape::sCollidePointUsingRayCast: world space point, number of hits of the ray
    point: R3,
    num_ray_hits: u32,
};

/// A collide / cast hit of a ClosestHitCollisionCollector, must match HitOut in ShapeCoreReference.cpp
const HitOut = extern struct {
    had_hit: u32,
    contact1: P,
    contact2: P,
    axis: P,
    depth: f32,
    fraction: f32,
    back_face: u32,
    ids: [2]u32,
    body_id: u32,
    face_counts: [2]u32,
    faces: [2][3]P,
    early_out: f32,
};

/// Results of the dispatch / TransformedShape query test, must match DispatchOutput in ShapeCoreReference.cpp
const DispatchOutput = extern struct {
    /// 0: CollisionDispatch::sCollideShapeVsShape(A, B), 1: TransformedShape(B)::CollideShape(A)
    collide: [2]CollideRecord,
    collide_hits: [2]HitOut,
    /// 0: sCastShapeVsShapeWorldSpace(A, B), 1: sCastShapeVsShapeLocalSpace(A, B), 2: TransformedShape(B)::CastShape(A)
    cast: [3]CastRecord,
    cast_hits: [3]HitOut,
    /// GetTrianglesStart, GetSupportingFace, CollectTransformedShapes and the ray of sCollidePointUsingRayCast
    queries: QueryRecord,
    face_count: u32,
    face: [3]P,
    collect_count: u32,
    /// CollidePoint: local point, ray, number of hits, sub shape ID and body ID of the first hit
    point: P,
    ray: [2]P,
    point_hits: u32,
    point_ids: [2]u32,
};

/// A stored collector hit in the format of the C ABI, must match Hit in ShapeCoreReference.cpp
const Hit = extern struct {
    body_id: u32,
    fraction: f32,
    penetration_depth: f32,
};

fn vec3(a: P) Vec3 {
    return Vec3.init(a[0], a[1], a[2]);
}

fn arr3(v: Vec3) P {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn vec4(a: [4]f32) Vec4 {
    return Vec4.init(a[0], a[1], a[2], a[3]);
}

fn arr4(v: Vec4) [4]f32 {
    return .{ v.getX(), v.getY(), v.getZ(), v.getW() };
}

fn quat(a: [4]f32) Quat {
    return Quat.init(a[0], a[1], a[2], a[3]);
}

fn mat44(a: [16]f32) Mat44 {
    return Mat44.init(vec4(a[0..4].*), vec4(a[4..8].*), vec4(a[8..12].*), vec4(a[12..16].*));
}

fn arr16(m: Mat44) [16]f32 {
    return arr4(m.getColumn4(0)) ++ arr4(m.getColumn4(1)) ++ arr4(m.getColumn4(2)) ++ arr4(m.getColumn4(3));
}

fn rvec3(a: R3) RVec3 {
    return RVec3.init(a[0], a[1], a[2]);
}

fn arrR3(v: RVec3) R3 {
    return .{ v.getX(), v.getY(), v.getZ() };
}

/// RMat44(Vec4, Vec4, Vec4, RVec3): 3 columns and a translation
fn rmat44(columns: [12]f32, translation: R3) RMat44 {
    const c0 = vec4(columns[0..4].*);
    const c1 = vec4(columns[4..8].*);
    const c2 = vec4(columns[8..12].*);
    return if (Core.double_precision) DMat44.init(c0, c1, c2, rvec3(translation)) else Mat44.fromColumnsTranslation(c0, c1, c2, rvec3(translation));
}

fn columns12(m: RMat44) [12]f32 {
    return arr4(m.getColumn4(0)) ++ arr4(m.getColumn4(1)) ++ arr4(m.getColumn4(2));
}

fn boxArr(b: AABox) [6]f32 {
    return arr3(b.min) ++ arr3(b.max);
}

/// Values that tend to expose differences, mixed into the random inputs
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 1.0e-6, -1.0e-6, 1.0e-7, 1.0e-20, 100.0, -100.0 };

/// Input generator: xorshift32 with helpers for the edge cases
const Gen = struct {
    rng: fw.Rng = .{},

    fn next(self: *Gen) u32 {
        return self.rng.next();
    }

    fn index(self: *Gen, n: usize) usize {
        return self.next() % n;
    }

    fn oneIn(self: *Gen, n: u32) bool {
        return self.next() % n == 0;
    }

    /// Random float in [min, max), or one of the special values (10% of the time)
    fn float(self: *Gen, min: f32, max: f32) f32 {
        if (self.oneIn(10)) return special_values[self.index(special_values.len)];
        return self.rng.float(min, max);
    }

    fn plain(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn vec(self: *Gen, min: f32, max: f32) P {
        return .{ self.float(min, max), self.float(min, max), self.float(min, max) };
    }

    fn plainVec(self: *Gen, min: f32, max: f32) P {
        return .{ self.plain(min, max), self.plain(min, max), self.plain(min, max) };
    }

    fn realVec(self: *Gen, min: f32, max: f32) R3 {
        const v = self.vec(min, max);
        // Big offsets that only fit in double precision (in single precision they are rounded like any float)
        const big: Real = if (self.oneIn(3)) 1.0e7 else 0.0;
        return .{ @as(Real, v[0]) + big, @as(Real, v[1]), @as(Real, v[2]) - big };
    }

    /// Integer in [-n, n] as float
    fn grid(self: *Gen, n: i32) f32 {
        return @floatFromInt(self.rng.intRange(i32, -n, n));
    }

    /// A random unit quaternion, sometimes a rotation of a multiple of 90 degrees around an axis
    fn rotation(self: *Gen) [4]f32 {
        if (self.oneIn(4)) {
            const axes = [_]Vec3{ Vec3.axisX(), Vec3.axisY(), Vec3.axisZ() };
            const axis = axes[self.index(3)];
            const angle = @as(f32, @floatFromInt(self.index(4))) * 0.5 * zolt.math.pi;
            return arr4(Quat.rotation(axis, angle).getXYZW());
        }
        while (true) {
            const q = self.rng.floatArray(4, -1, 1);
            const len_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
            if (len_sq > 1.0e-2 and len_sq <= 1.0) return arr4(quat(q).normalized().getXYZW());
        }
    }

    /// A scale: random, uniform, uniform in XZ, near one, with tiny / zero / negative components
    fn scale(self: *Gen) P {
        return switch (self.index(6)) {
            0 => self.vec(-3, 3),
            1 => blk: {
                const s = self.float(-3, 3);
                break :blk .{ s, s, s };
            },
            2 => blk: {
                const s = self.float(-3, 3);
                break :blk .{ s, self.float(-3, 3), if (self.oneIn(2)) s else -s };
            },
            3 => .{ 1.0 + self.plain(-1.0e-4, 1.0e-4), 1.0, 1.0 + self.plain(-1.0e-5, 1.0e-5) },
            4 => .{ self.float(-1.0e-5, 1.0e-5), self.plain(0.1, 2), self.plain(-2, -0.1) },
            else => .{ self.grid(2), self.grid(2), self.grid(2) },
        };
    }

    /// A rotation + translation matrix, or random values in the 3x3 part (a scale / shear)
    fn transform(self: *Gen) [16]f32 {
        var m = Mat44.rotationQuat(quat(self.rotation()));
        if (self.oneIn(2)) m = m.mul(Mat44.scaleVec3(vec3(self.plainVec(0.2, 3))));
        m.setTranslation(vec3(self.vec(-10, 10)));
        return arr16(m);
    }
};

/// The parity test shape (User1 / User2) and its registered collide / cast functions (the zolt_user_types module of the
/// parity build, see build.zig)
const parity_user_types = @import("parity_user_types");
const ParityShape = parity_user_types.ParityShape;
const CollideRecord = parity_user_types.CollideRecord;
const CastRecord = parity_user_types.CastRecord;
const QueryRecord = parity_user_types.QueryRecord;

test "ShapeCore parity: ScaleHelpers" {
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "ScaleHelpers" };
    for (0..iterations) |_| {
        const scale = gen.scale();
        const radius = gen.float(0, 1);
        const rotation = gen.rotation();

        var bools: [6]c_int = undefined;
        var vectors: [12]f32 = undefined;
        const jolt_radius = jolt.jolt_scale_helpers(&scale, radius, &rotation, &bools, &vectors);

        const s = vec3(scale);
        const q = quat(rotation);
        const zolt_bools = [6]c_int{ @intFromBool(ScaleHelpers.isNotScaled(s)), @intFromBool(ScaleHelpers.isUniformScale(s)), @intFromBool(ScaleHelpers.isUniformScaleXZ(s)), @intFromBool(ScaleHelpers.isInsideOut(s)), @intFromBool(ScaleHelpers.isZeroScale(s)), @intFromBool(ScaleHelpers.canScaleBeRotated(q, s)) };
        const zolt_vectors = arr3(ScaleHelpers.makeNonZeroScale(s)) ++ arr3(ScaleHelpers.makeUniformScale(s)) ++ arr3(ScaleHelpers.makeUniformScaleXZ(s)) ++ arr3(ScaleHelpers.rotateScale(q, s));
        checker.check(.{ scale, radius, rotation }, .{ zolt_bools, zolt_vectors, ScaleHelpers.scaleConvexRadius(radius, s) }, .{ bools, vectors, jolt_radius });
    }
    try checker.finish();
}

test "ShapeCore parity: GetTrianglesContextVertexList vertex list helpers" {
    const allocator = std.testing.allocator;
    var checker: Checker = .{ .name = "vertex list helpers" };
    var buffer: [3 * 4 * 1024 * 3]f32 = undefined;
    for (0..3) |which| {
        for (0..6) |level| {
            if (which < 2 and level > 4) continue;
            const count = jolt.jolt_triangles_vertex_list_helper(@intCast(which), @intCast(level), &buffer, buffer.len / 3);
            var vertices: std.ArrayList(Vec3) = .empty;
            defer vertices.deinit(allocator);
            const list: VertexArrayList = .{ .allocator = allocator, .list = &vertices };
            switch (which) {
                0 => try GetTrianglesContextVertexList.createHalfUnitSphereTop(list, @intCast(level)),
                1 => try GetTrianglesContextVertexList.createHalfUnitSphereBottom(list, @intCast(level)),
                else => try GetTrianglesContextVertexList.createUnitOpenCylinder(list, @intCast(level)),
            }
            checker.check(.{ which, level }, vertices.items.len, count);
            if (vertices.items.len != count) continue;
            for (vertices.items, 0..) |v, i| checker.check(.{ which, level, i }, arr3(v), buffer[3 * i ..][0..3].*);
        }
    }
    try checker.finish();
}

test "ShapeCore parity: GetTrianglesContextVertexList / GetTrianglesContextMultiVertexList" {
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "GetTrianglesContextVertexList" };
    var multi_checker: Checker = .{ .name = "GetTrianglesContextMultiVertexList" };
    var input: [3 * 3 * 120]f32 = undefined;
    var jolt_vertices: [3 * 3 * 120]f32 = undefined;
    var zolt_vertices: [3 * 3 * 120]f32 = undefined;
    var triangles: [3 * 100]Float3 = undefined;
    var materials: [100]*const PhysicsMaterial = undefined;
    const max_requested = [_]u32{ 32, 33, 47, 100 };

    for (0..iterations / 20) |_| {
        const num_triangles = gen.index(100);
        const num_vertices: u32 = @intCast(3 * num_triangles);
        for (input[0 .. 3 * num_vertices]) |*v| v.* = gen.float(-5, 5);
        const position = gen.vec(-10, 10);
        const rotation = gen.rotation();
        const scale = gen.scale();
        const local = gen.transform();
        const max = max_requested[gen.index(max_requested.len)];

        var jolt_counts: [64]c_int = undefined;
        const jolt_calls = jolt.jolt_triangles_vertex_list(&position, &rotation, &scale, &local, &input, num_vertices, @intCast(max), &jolt_counts, &jolt_vertices);

        var vertices: [3 * 100]Vec3 = undefined;
        for (vertices[0..num_vertices], 0..) |*v, i| v.* = vec3(input[3 * i ..][0..3].*);
        var context: Shape.GetTrianglesContext = .{};
        const list = context.emplace(GetTrianglesContextVertexList);
        list.* = .init(vec3(position), quat(rotation), vec3(scale), mat44(local), vertices[0..num_vertices], PhysicsMaterial.default);
        var zolt_counts: [64]c_int = undefined;
        var calls: c_int = 0;
        var out: usize = 0;
        while (true) {
            const count = list.getTrianglesNext(max, triangles[0 .. 3 * max], materials[0..max]);
            zolt_counts[@intCast(calls)] = @intCast(count);
            calls += 1;
            for (triangles[0 .. 3 * count]) |t| {
                zolt_vertices[out..][0..3].* = .{ t.x, t.y, t.z };
                out += 3;
            }
            if (count == 0 or calls == 64) break;
        }
        checker.check(.{ num_triangles, max }, calls, jolt_calls);
        if (calls != jolt_calls) continue;
        for (0..@intCast(calls)) |i| checker.check(.{ num_triangles, max, i }, zolt_counts[i], jolt_counts[i]);
        for (0..out) |i| checker.check(.{ num_triangles, max, i, position, rotation, scale }, zolt_vertices[i], jolt_vertices[i]);

        // Multi vertex list: up to 3 parts, the vertices of `input` split in parts
        const num_parts = 1 + gen.index(3);
        var part_sizes: [3]u32 = .{ 0, 0, 0 };
        var remaining: u32 = @intCast(num_triangles);
        for (0..num_parts) |p| {
            const n: u32 = if (p == num_parts - 1) remaining else @intCast(gen.index(remaining + 1));
            part_sizes[p] = 3 * n;
            remaining -= n;
        }
        var transforms: [48]f32 = undefined;
        for (0..3) |p| transforms[16 * p ..][0..16].* = gen.transform();
        const inside_out = gen.oneIn(2);
        const jolt_multi_calls = jolt.jolt_triangles_multi_vertex_list(@intFromBool(inside_out), @intCast(num_parts), &transforms, &input, &part_sizes, @intCast(max), &jolt_counts, &jolt_vertices);

        var multi_context: Shape.GetTrianglesContext = .{};
        const multi = multi_context.emplace(GetTrianglesContextMultiVertexList);
        multi.* = .init(inside_out, PhysicsMaterial.default);
        var offset: usize = 0;
        for (0..num_parts) |p| {
            multi.addPart(mat44(transforms[16 * p ..][0..16].*), vertices[offset .. offset + part_sizes[p]]);
            offset += part_sizes[p];
        }
        calls = 0;
        out = 0;
        while (true) {
            const count = multi.getTrianglesNext(max, triangles[0 .. 3 * max], null);
            zolt_counts[@intCast(calls)] = @intCast(count);
            calls += 1;
            for (triangles[0 .. 3 * count]) |t| {
                zolt_vertices[out..][0..3].* = .{ t.x, t.y, t.z };
                out += 3;
            }
            if (count == 0 or calls == 64) break;
        }
        multi_checker.check(.{ part_sizes, max }, calls, jolt_multi_calls);
        if (calls != jolt_multi_calls) continue;
        for (0..@intCast(calls)) |i| multi_checker.check(.{ part_sizes, max, i }, zolt_counts[i], jolt_counts[i]);
        for (0..out) |i| multi_checker.check(.{ part_sizes, max, i }, zolt_vertices[i], jolt_vertices[i]);
    }
    try finishAll(&.{ &checker, &multi_checker });
}

test "ShapeCore parity: ShapeCast / RShapeCast" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "ShapeCast" };
    var r_checker: Checker = .{ .name = "RShapeCast" };
    var record: parity_user_types.Record = .{};
    for (0..iterations / 4) |_| {
        const half_extent = gen.plainVec(0.1, 3);
        const center_of_mass = gen.vec(-2, 2);
        const scale = gen.scale();
        const start = gen.transform();
        const direction = gen.vec(-10, 10);
        const transform = gen.transform();
        const translation = gen.vec(-10, 10);
        const fraction = gen.float(-1, 2);

        var shape = ParityShape.init(allocator, vec3(half_extent), vec3(center_of_mass), false, &record);
        shape.base.setEmbedded();
        defer shape.base.deinit();

        var jolt_casts: [100]f32 = undefined;
        var jolt_point: P = undefined;
        jolt.jolt_shape_cast(&half_extent, &center_of_mass, &scale, &start, &direction, &transform, &translation, fraction, &jolt_casts, &jolt_point);

        const cast = ShapeCast.init(shape.asShape(), vec3(scale), mat44(start), vec3(direction));
        const casts = [_]ShapeCast{ cast, ShapeCast.fromWorldTransform(shape.asShape(), vec3(scale), mat44(start), vec3(direction)), cast.postTransformed(mat44(transform)), cast.postTranslated(vec3(translation)) };
        var zolt_casts: [100]f32 = undefined;
        for (casts, 0..) |c, i| zolt_casts[25 * i ..][0..25].* = arr16(c.center_of_mass_start) ++ arr3(c.direction) ++ boxArr(c.shape_world_bounds);
        checker.check(.{ half_extent, center_of_mass, scale, start, direction, transform }, .{ zolt_casts, arr3(cast.getPointOnRay(fraction)) }, .{ jolt_casts, jolt_point });

        // RShapeCast (RMat44 start, big translations in double precision)
        var start_columns: [12]f32 = undefined;
        @memcpy(&start_columns, start[0..12]);
        const start_translation = gen.realVec(-10, 10);
        const r_translation = gen.realVec(-10, 10);
        var jolt_columns: [36]f32 = undefined;
        var jolt_translations: [9]Real = undefined;
        var jolt_rest: [27]f32 = undefined;
        var jolt_r_point: R3 = undefined;
        var jolt_shape_cast: [25]f32 = undefined;
        jolt.jolt_r_shape_cast(&half_extent, &center_of_mass, &scale, &start_columns, &start_translation, &direction, &r_translation, fraction, &jolt_columns, &jolt_translations, &jolt_rest, &jolt_r_point, &jolt_shape_cast);

        const r_start = rmat44(start_columns, start_translation);
        const r_cast = RShapeCast.init(shape.asShape(), vec3(scale), r_start, vec3(direction));
        const r_casts = [_]RShapeCast{ r_cast, RShapeCast.fromWorldTransform(shape.asShape(), vec3(scale), r_start, vec3(direction)), r_cast.postTranslated(rvec3(r_translation)) };
        var zolt_columns: [36]f32 = undefined;
        var zolt_translations: [9]Real = undefined;
        var zolt_rest: [27]f32 = undefined;
        for (r_casts, 0..) |c, i| {
            zolt_columns[12 * i ..][0..12].* = columns12(c.center_of_mass_start);
            zolt_translations[3 * i ..][0..3].* = arrR3(c.center_of_mass_start.getTranslation());
            zolt_rest[9 * i ..][0..9].* = arr3(c.direction) ++ boxArr(c.shape_world_bounds);
        }
        const single = r_cast.toShapeCast();
        const zolt_shape_cast = arr16(single.center_of_mass_start) ++ arr3(single.direction) ++ boxArr(single.shape_world_bounds);
        r_checker.check(.{ half_extent, center_of_mass, scale, start_columns, start_translation, direction, r_translation }, .{ zolt_columns, zolt_translations, zolt_rest, arrR3(r_cast.getPointOnRay(fraction)), zolt_shape_cast }, .{ jolt_columns, jolt_translations, jolt_rest, jolt_r_point, jolt_shape_cast });
    }
    try finishAll(&.{ &checker, &r_checker });
}

test "ShapeCore parity: ShapeCastResult" {
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "ShapeCastResult" };
    for (0..iterations) |_| {
        const fraction = if (gen.oneIn(4)) 0.0 else gen.float(-1, 1);
        const contact1 = gen.vec(-10, 10);
        const contact2 = gen.vec(-10, 10);
        const axis = gen.vec(-1, 1);
        const back_face = gen.oneIn(2);
        const ids = [3]u32{ gen.next(), gen.next(), gen.next() };
        const direction = gen.vec(-10, 10);
        var faces: [2][32]P = undefined;
        const counts = [2]u32{ @intCast(gen.index(33)), @intCast(gen.index(33)) };
        for (&faces) |*face| for (face) |*v| {
            v.* = gen.vec(-10, 10);
        };

        var jolt_values: [14]f32 = undefined;
        var jolt_ints: [6]u32 = undefined;
        var jolt_faces: [192]f32 = @splat(0);
        jolt.jolt_shape_cast_result(fraction, &contact1, &contact2, &axis, @intFromBool(back_face), ids[0], ids[1], ids[2], @ptrCast(&faces[0]), counts[0], @ptrCast(&faces[1]), counts[1], &direction, &jolt_values, &jolt_ints, &jolt_faces);

        var r = ShapeCastResult.init(fraction, vec3(contact1), vec3(contact2), vec3(axis), back_face, .{ .value = ids[0] }, .{ .value = ids[1] }, .{ .id = ids[2] });
        for (faces[0][0..counts[0]]) |v| r.base.shape1_face.append(vec3(v));
        for (faces[1][0..counts[1]]) |v| r.base.shape2_face.append(vec3(v));
        const rev = r.reversed(vec3(direction));
        const zolt_values = [2]f32{ r.base.penetration_depth, r.getEarlyOutFraction() } ++ arr3(rev.base.contact_point_on1) ++ arr3(rev.base.contact_point_on2) ++ arr3(rev.base.penetration_axis) ++ [3]f32{ rev.base.penetration_depth, rev.fraction, rev.getEarlyOutFraction() };
        const zolt_ints = [6]u32{ rev.base.sub_shape_id1.getValue(), rev.base.sub_shape_id2.getValue(), rev.base.body_id2.getIndexAndSequenceNumber(), @intFromBool(rev.is_back_face_hit), rev.base.shape1_face.len, rev.base.shape2_face.len };
        var zolt_faces: [192]f32 = @splat(0);
        for (rev.base.shape1_face.constSlice(), 0..) |v, i| zolt_faces[3 * i ..][0..3].* = arr3(v);
        for (rev.base.shape2_face.constSlice(), 0..) |v, i| zolt_faces[96 + 3 * i ..][0..3].* = arr3(v);
        checker.check(.{ fraction, contact1, contact2, axis, direction }, .{ zolt_values, zolt_ints, zolt_faces }, .{ jolt_values, jolt_ints, jolt_faces });
    }
    try checker.finish();
}

/// Collects the transformed shapes (copies), like TSCollector in ShapeCoreReference.cpp
fn collectFirst(collector: *AllHitCollisionCollector(TransformedShapeCollector)) !TS {
    try collector.checkError();
    return .fromTransformedShape(&collector.hits.items[0]);
}

test "ShapeCore parity: TransformedShape and the default implementations of Shape" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "TransformedShape" };
    var record: parity_user_types.Record = .{};
    for (0..iterations / 4) |_| {
        const half_extent = gen.plainVec(0.1, 3);
        const center_of_mass = gen.vec(-2, 2);
        const uniform = gen.oneIn(2);
        const position = gen.realVec(-10, 10);
        const rotation = gen.rotation();
        const scale = gen.scale();
        const bits: u32 = @intCast(1 + gen.index(8));
        const sub_shape_id: u32 = gen.next() & ((@as(u32, 1) << @intCast(bits)) - 1);
        const world = gen.transform();
        var world_columns: [12]f32 = undefined;
        @memcpy(&world_columns, world[0..12]);
        const world_translation = gen.realVec(-10, 10);
        const shape_transform = gen.transform();
        const ray_origin = gen.realVec(-10, 10);
        const ray_direction = gen.vec(-10, 10);
        const point = gen.realVec(-10, 10);

        var jolt_columns: [36]f32 = undefined;
        var jolt_translations: [9]Real = undefined;
        var jolt_ts: [5]TS = undefined;
        var jolt_vectors: [19]f32 = undefined;
        jolt.jolt_transformed_shape(&half_extent, &center_of_mass, @intFromBool(uniform), &position, &rotation, &scale, sub_shape_id, bits, &world_columns, &world_translation, &shape_transform, &ray_origin, &ray_direction, &point, &jolt_columns, &jolt_translations, &jolt_ts, &jolt_vectors);

        var shape = ParityShape.init(allocator, vec3(half_extent), vec3(center_of_mass), uniform, &record);
        shape.base.setEmbedded();
        defer shape.base.deinit();
        const creator = SubShapeIDCreator.pushID(.{}, sub_shape_id, bits);
        var ts = TransformedShape.init(rvec3(position), quat(rotation), shape.asShape(), .init(7), .{ .sub_shape_id_creator = creator });
        defer ts.deinit();
        ts.setShapeScale(vec3(scale));

        var zolt_columns: [36]f32 = undefined;
        var zolt_translations: [9]Real = undefined;
        for ([_]RMat44{ ts.getCenterOfMassTransform(), ts.getInverseCenterOfMassTransform(), ts.getWorldTransform() }, 0..) |m, i| {
            zolt_columns[12 * i ..][0..12].* = columns12(m);
            zolt_translations[3 * i ..][0..3].* = arrR3(m.getTranslation());
        }

        var zolt_vectors: [19]f32 = undefined;
        zolt_vectors[0..6].* = boxArr(ts.getWorldSpaceBounds());
        zolt_vectors[6..9].* = arr3(ts.getWorldSpaceSurfaceNormal(creator.getID(), rvec3(point)));
        var hit: RayCastResult = .{};
        _ = ts.castRay(.init(rvec3(ray_origin), vec3(ray_direction)), &hit);
        zolt_vectors[9..12].* = arr3(record.last_ray.origin);
        zolt_vectors[12..15].* = arr3(record.last_ray.direction);
        zolt_vectors[15] = hit.fraction;
        var point_collector = AnyHitCollisionCollector(CollidePointCollector).init();
        defer point_collector.deinit();
        ts.collidePoint(rvec3(point), &point_collector.base, .{});
        zolt_vectors[16..19].* = arr3(record.last_point);

        var zolt_ts: [5]TS = undefined;
        var set1 = ts.clone();
        defer set1.deinit();
        set1.setWorldTransform(rvec3(position), quat(rotation), vec3(scale));
        zolt_ts[0] = .fromTransformedShape(&set1);
        var set2 = ts.clone();
        defer set2.deinit();
        set2.setWorldTransformRMat44(rmat44(world_columns, world_translation));
        zolt_ts[1] = .fromTransformedShape(&set2);
        var sub = ts.getSubShapeTransformedShape(creator.getID());
        defer sub.transformed_shape.deinit();
        zolt_ts[2] = .fromTransformedShape(&sub.transformed_shape);

        var transform_collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer transform_collector.deinit();
        shape.asShape().transformShape(mat44(shape_transform), &transform_collector.base);
        zolt_ts[3] = try collectFirst(&transform_collector);
        var collect_collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer collect_collector.deinit();
        ts.collectTransformedShapes(AABox.biggest(), &collect_collector.base, .{});
        zolt_ts[4] = try collectFirst(&collect_collector);

        checker.check(.{ half_extent, center_of_mass, uniform, position, rotation, scale, world, shape_transform }, .{ zolt_columns, zolt_translations, zolt_vectors, zolt_ts }, .{ jolt_columns, jolt_translations, jolt_vectors, jolt_ts });
    }
    try checker.finish();
}

fn hitOut(had_hit: bool, r: *const CollideShapeResult, fraction: f32, back_face: bool, early_out: f32) HitOut {
    var out = std.mem.zeroes(HitOut);
    out.early_out = early_out;
    if (!had_hit) return out;
    out.had_hit = 1;
    out.contact1 = arr3(r.contact_point_on1);
    out.contact2 = arr3(r.contact_point_on2);
    out.axis = arr3(r.penetration_axis);
    out.depth = r.penetration_depth;
    out.fraction = fraction;
    out.back_face = @intFromBool(back_face);
    out.ids = .{ r.sub_shape_id1.getValue(), r.sub_shape_id2.getValue() };
    out.body_id = r.body_id2.getIndexAndSequenceNumber();
    out.face_counts = .{ r.shape1_face.len, r.shape2_face.len };
    for (r.shape1_face.constSlice()[0..@min(r.shape1_face.len, 3)], 0..) |v, i| out.faces[0][i] = arr3(v);
    for (r.shape2_face.constSlice()[0..@min(r.shape2_face.len, 3)], 0..) |v, i| out.faces[1][i] = arr3(v);
    return out;
}

fn collideHitOut(c: *const ClosestHitCollisionCollector(CollideShapeCollector)) HitOut {
    return hitOut(c.hadHit(), &c.hit, 0.0, false, c.base.getEarlyOutFraction());
}

fn castHitOut(c: *const ClosestHitCollisionCollector(CastShapeCollector)) HitOut {
    return hitOut(c.hadHit(), &c.hit.base, c.hit.fraction, c.hit.is_back_face_hit, c.base.getEarlyOutFraction());
}

/// A rotation for the dispatch test: random, or (half of the time) a rotation of the cube's symmetry group whose matrix
/// is an exact signed permutation (quaternion components 0 / +-0.5 / +-1). Products with exact zeros make the sign of
/// zero results depend on the order of the operations (e.g. -(M * d) versus M * (-d)), which bit for bit checks see.
fn dispatchRotation(gen: *Gen) [4]f32 {
    if (gen.oneIn(2)) return gen.rotation();
    if (gen.oneIn(3)) {
        var q: [4]f32 = .{ 0, 0, 0, 0 };
        q[gen.index(4)] = if (gen.oneIn(2)) 1.0 else -1.0;
        return q;
    }
    var q: [4]f32 = undefined;
    for (&q) |*c| c.* = if (gen.oneIn(2)) 0.5 else -0.5;
    return q;
}

/// A vector for the dispatch test: random, or (a third of the time) components from {0, -0, +-5}
fn dispatchVec(gen: *Gen, min: f32, max: f32) P {
    if (!gen.oneIn(3)) return gen.vec(min, max);
    const values = [_]f32{ 0.0, -0.0, 5.0, -5.0 };
    return .{ values[gen.index(4)], values[gen.index(4)], values[gen.index(4)] };
}

/// A rotation + translation matrix (the transforms that the dispatch functions expect)
fn rotationTranslation(gen: *Gen) [16]f32 {
    return arr16(Mat44.rotationTranslation(quat(dispatchRotation(gen)), vec3(dispatchVec(gen, -10, 10))));
}

/// Run the dispatch / TransformedShape queries of jolt_dispatch_queries in Zolt
fn dispatchQueries(allocator: Allocator, in: *const DispatchInput) !DispatchOutput {
    var out = std.mem.zeroes(DispatchOutput);
    var record: parity_user_types.Record = .{ .hit_values = in.hit_values };
    const sub_types = [2]ShapeSubType{ if (in.sub_types[0] == 0) .user1 else .user2, if (in.sub_types[1] == 0) .user1 else .user2 };
    var a = ParityShape.initSubType(allocator, sub_types[0], vec3(in.half_extents[0]), vec3(in.centers_of_mass[0]), false, &record);
    a.base.setEmbedded();
    defer a.base.deinit();
    var b = ParityShape.initSubType(allocator, sub_types[1], vec3(in.half_extents[1]), vec3(in.centers_of_mass[1]), false, &record);
    b.base.setEmbedded();
    defer b.base.deinit();
    const creator0 = SubShapeIDCreator.pushID(.{}, in.creators[0][0], in.creators[0][1]);
    const creator1 = SubShapeIDCreator.pushID(.{}, in.creators[1][0], in.creators[1][1]);
    const scale0 = vec3(in.scales[0]);
    const scale1 = vec3(in.scales[1]);
    const transform0 = mat44(in.transforms[0]);
    const transform1 = mat44(in.transforms[1]);
    const direction = vec3(in.direction);
    const collide_settings: CollideShapeSettings = .{};
    const cast_settings: ShapeCastSettings = .{};
    const filter: ShapeFilter = .{};

    // CollisionDispatch::sCollideShapeVsShape (User1 vs User2 goes through sReversedCollideShape)
    var collide0 = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer collide0.deinit();
    CollisionDispatch.collideShapeVsShape(a.asShape(), b.asShape(), scale0, scale1, transform0, transform1, creator0, creator1, &collide_settings, &collide0.base, &filter);
    out.collide[0] = record.collide;
    out.collide_hits[0] = collideHitOut(&collide0);

    // TransformedShape of B
    var ts = TransformedShape.init(rvec3(in.position), quat(in.rotation), b.asShape(), .init(7), .{ .sub_shape_id_creator = creator1 });
    defer ts.deinit();
    ts.setShapeScale(vec3(in.ts_scale));
    const query = rmat44(in.query_columns, in.query_translation);
    const base_offset = rvec3(in.base_offset);

    record.collide = std.mem.zeroes(CollideRecord);
    var collide1 = ClosestHitCollisionCollector(CollideShapeCollector).init();
    defer collide1.deinit();
    ts.collideShape(a.asShape(), scale0, query, &collide_settings, base_offset, &collide1.base, .{});
    out.collide[1] = record.collide;
    out.collide_hits[1] = collideHitOut(&collide1);

    // CollisionDispatch::sCastShapeVsShapeWorldSpace / LocalSpace (User1 vs User2 goes through sReversedCastShape)
    const cast = ShapeCast.init(a.asShape(), scale0, transform0, direction);
    var cast0 = ClosestHitCollisionCollector(CastShapeCollector).init();
    defer cast0.deinit();
    CollisionDispatch.castShapeVsShapeWorldSpace(&cast, &cast_settings, b.asShape(), scale1, &filter, transform1, creator0, creator1, &cast0.base);
    out.cast[0] = record.cast;
    out.cast_hits[0] = castHitOut(&cast0);

    record.cast = std.mem.zeroes(CastRecord);
    var cast1 = ClosestHitCollisionCollector(CastShapeCollector).init();
    defer cast1.deinit();
    CollisionDispatch.castShapeVsShapeLocalSpace(&cast, &cast_settings, b.asShape(), scale1, &filter, transform1, creator0, creator1, &cast1.base);
    out.cast[1] = record.cast;
    out.cast_hits[1] = castHitOut(&cast1);

    record.cast = std.mem.zeroes(CastRecord);
    const r_cast = RShapeCast.init(a.asShape(), scale0, query, direction);
    var cast2 = ClosestHitCollisionCollector(CastShapeCollector).init();
    defer cast2.deinit();
    ts.castShape(&r_cast, &cast_settings, base_offset, &cast2.base, .{});
    out.cast[2] = record.cast;
    out.cast_hits[2] = castHitOut(&cast2);

    // TransformedShape::GetTrianglesStart, GetSupportingFace, CollectTransformedShapes
    const box: AABox = .init(vec3(in.box[0..3].*), vec3(in.box[3..6].*));
    var context: Shape.GetTrianglesContext = .{};
    ts.getTrianglesStart(&context, box, base_offset);
    var face: Shape.SupportingFace = .empty;
    ts.getSupportingFace(creator1.pushID(in.face_id[0], in.face_id[1]).getID(), vec3(in.face_direction), base_offset, &face);
    out.face_count = face.len;
    for (face.constSlice()[0..@min(face.len, 3)], 0..) |v, i| out.face[i] = arr3(v);
    var collect_collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer collect_collector.deinit();
    ts.collectTransformedShapes(box, &collect_collector.base, .{});
    try collect_collector.checkError();
    out.collect_count = @intCast(collect_collector.hits.items.len);

    // TransformedShape::CollidePoint through Shape::sCollidePointUsingRayCast
    record.point_using_ray_cast = true;
    record.num_ray_hits = in.num_ray_hits;
    var point_collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer point_collector.deinit();
    ts.collidePoint(rvec3(in.point), &point_collector.base, .{});
    try point_collector.checkError();
    out.point = arr3(record.last_point);
    out.ray = .{ arr3(record.last_ray.origin), arr3(record.last_ray.direction) };
    out.point_hits = @intCast(point_collector.hits.items.len);
    if (point_collector.hits.items.len > 0) {
        const hit = point_collector.hits.items[0];
        out.point_ids = .{ hit.sub_shape_id2.getValue(), hit.body_id.getIndexAndSequenceNumber() };
    }
    out.queries = record.queries;
    return out;
}

test "ShapeCore parity: CollisionDispatch (reversed functions) and the TransformedShape queries through registered functions" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "CollisionDispatch / TransformedShape queries" };
    for (0..iterations / 10) |_| {
        var in = std.mem.zeroes(DispatchInput);
        for (0..2) |i| {
            in.half_extents[i] = gen.plainVec(0.1, 3);
            in.centers_of_mass[i] = gen.vec(-2, 2);
            in.sub_types[i] = @intCast(gen.index(2));
            in.scales[i] = gen.scale();
            in.transforms[i] = rotationTranslation(&gen);
            const bits: u32 = @intCast(1 + gen.index(8));
            in.creators[i] = .{ gen.next() & ((@as(u32, 1) << @intCast(bits)) - 1), bits };
        }
        // Depths / fractions: ties and zeros (penetrating casts) included
        for (&in.hit_values) |*v| v.* = if (gen.oneIn(4)) 0.0 else if (gen.oneIn(3)) @as(f32, @floatFromInt(gen.index(4))) / 4.0 else gen.plain(0, 1);
        in.direction = dispatchVec(&gen, -10, 10);
        in.position = gen.realVec(-10, 10);
        in.rotation = dispatchRotation(&gen);
        in.ts_scale = gen.scale();
        const query = rotationTranslation(&gen);
        @memcpy(&in.query_columns, query[0..12]);
        in.query_translation = gen.realVec(-10, 10);
        in.base_offset = if (gen.oneIn(4)) .{ 0, 0, 0 } else if (gen.oneIn(3)) in.position else gen.realVec(-10, 10);
        const center = gen.vec(-20, 20);
        const extent = gen.plainVec(0, 10);
        in.box = .{ center[0] - extent[0], center[1] - extent[1], center[2] - extent[2], center[0] + extent[0], center[1] + extent[1], center[2] + extent[2] };
        const face_bits: u32 = @intCast(gen.index(4));
        in.face_id = .{ gen.next() & ((@as(u32, 1) << @intCast(face_bits)) - 1), face_bits };
        in.face_direction = dispatchVec(&gen, -1, 1);
        // The point is close to the shape (inside its bounds about half of the time)
        const local_point = vec3(gen.vec(-3, 3));
        var point_ts = TransformedShape.init(rvec3(in.position), quat(in.rotation), null, .invalid, .{});
        point_ts.setShapeScale(vec3(in.ts_scale));
        in.point = arrR3(point_ts.getCenterOfMassTransform().mulVec3(vec3(in.ts_scale).mul(local_point)));
        in.num_ray_hits = @intCast(gen.index(4));

        var jolt_out: DispatchOutput = undefined;
        jolt.jolt_dispatch_queries(&in, &jolt_out);
        const zolt_out = try dispatchQueries(allocator, &in);
        checker.check(.{in}, zolt_out, jolt_out);
    }
    try checker.finish();
}

test "ShapeCore parity: SaveWithChildren of a shape graph" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "SaveWithChildren" };
    var record: parity_user_types.Record = .{};
    var jolt_bytes: [8192]u8 = undefined;
    var zolt_bytes: [8192]u8 = undefined;
    for (0..iterations / 50) |_| {
        const num_shapes = 1 + gen.index(8);
        const num_materials = gen.index(4);
        var half_extents: [8 * 3]f32 = undefined;
        var user_data: [8]u64 = undefined;
        var children: [16]c_int = undefined;
        var material_indices: [8]c_int = undefined;
        var colors: [3]u32 = undefined;
        for (0..num_shapes) |s| {
            half_extents[3 * s ..][0..3].* = gen.plainVec(0.1, 3);
            user_data[s] = @as(u64, gen.next()) << 32 | gen.next();
            for (0..2) |c| {
                // Children have a higher index (no cycles), sometimes the same child twice
                const remaining = num_shapes - s - 1;
                children[2 * s + c] = if (remaining > 0 and !gen.oneIn(3)) @intCast(s + 1 + gen.index(remaining)) else -1;
            }
            material_indices[s] = if (num_materials > 0 and !gen.oneIn(3)) @intCast(gen.index(num_materials)) else -1;
        }
        for (0..num_materials) |m| colors[m] = gen.next();
        var roots: [3]c_int = undefined;
        const num_roots = 1 + gen.index(3);
        for (0..num_roots) |r| roots[r] = @intCast(gen.index(num_shapes));

        const jolt_size = jolt.jolt_save_with_children(@intCast(num_shapes), &half_extents, &user_data, &children, &material_indices, @intCast(num_materials), &colors, &roots, @intCast(num_roots), &jolt_bytes, jolt_bytes.len);

        // The same graph in Zolt (heap shapes and materials, released at the end)
        var materials: [3]RefConst(PhysicsMaterial) = .{ .empty, .empty, .empty };
        defer for (&materials) |*m| m.deinit();
        var names: [3][16]u8 = undefined;
        for (0..num_materials) |m| {
            const name = try std.fmt.bufPrint(&names[m], "Material{d}", .{m});
            materials[m] = .init((try PhysicsMaterialSimple.create(allocator, name, Color.fromUInt32(colors[m]))).material());
        }
        var shapes: [8]Ref(Shape) = @splat(.empty);
        defer for (&shapes) |*s| s.deinit();
        var parity_shapes: [8]*ParityShape = undefined;
        for (0..num_shapes) |s| {
            const p = try allocator.create(ParityShape);
            p.* = .init(allocator, vec3(half_extents[3 * s ..][0..3].*), Vec3.zero(), false, &record);
            p.base.setUserData(user_data[s]);
            if (material_indices[s] >= 0) p.material.set(materials[@intCast(material_indices[s])].get());
            parity_shapes[s] = p;
            shapes[s] = .init(&p.base);
        }
        for (0..num_shapes) |s| for (0..2) |c| {
            if (children[2 * s + c] >= 0) try parity_shapes[s].children.append(allocator, .init(shapes[@intCast(children[2 * s + c])].get()));
        };
        defer for (parity_shapes[0..num_shapes]) |p| {
            for (p.children.items) |*c| c.deinit();
            p.children.clearRetainingCapacity();
        };

        var writer: std.Io.Writer = .fixed(&zolt_bytes);
        var out = StreamOutWrapper.init(&writer);
        var shape_map: Shape.ShapeToIDMap = .empty;
        defer shape_map.deinit(allocator);
        var material_map: Shape.MaterialToIDMap = .empty;
        defer material_map.deinit(allocator);
        for (roots[0..num_roots]) |r| try shapes[@intCast(r)].get().?.saveWithChildren(allocator, out.streamOut(), &shape_map, &material_map);
        const zolt_size = writer.buffered().len;

        checker.check(.{ num_shapes, num_materials, num_roots }, zolt_size, jolt_size);
        if (zolt_size != jolt_size) continue;
        for (0..zolt_size) |i| checker.check(.{ num_shapes, i }, zolt_bytes[i], jolt_bytes[i]);
    }
    try checker.finish();
}

/// Run a collector on a synthetic hit sequence, like sCollect in ShapeCoreReference.cpp
fn collect(allocator: Allocator, kind: usize, comptime Collector: type, num_hits: usize, bodies: []const u32, make: anytype, store: anytype, out_early_out: []f32, out_hits: []Hit) !usize {
    const body: Body = .{};
    switch (kind) {
        0, 2 => {
            var c = if (kind == 0) AllHitCollisionCollector(Collector).init(allocator) else undefined;
            var p = if (kind == 2) ClosestHitPerBodyCollisionCollector(Collector).init(allocator) else undefined;
            const base: *Collector = if (kind == 0) &c.base else &p.base;
            for (0..num_hits) |i| {
                if (i == 0 or bodies[i] != bodies[i - 1]) {
                    if (i > 0) base.onBodyEnd();
                    base.onBody(&body);
                }
                const r = make.at(i);
                base.addHit(&r);
                out_early_out[i] = base.getEarlyOutFraction();
            }
            if (num_hits > 0) base.onBodyEnd();
            out_early_out[num_hits] = base.getEarlyOutFraction();
            if (kind == 0) {
                defer c.deinit();
                try c.checkError();
                c.sort();
                for (c.hits.items, 0..) |*h, i| store(h, &out_hits[i]);
                return c.hits.items.len;
            } else {
                defer p.deinit();
                try p.checkError();
                p.sort();
                for (p.hits.items, 0..) |*h, i| store(h, &out_hits[i]);
                return p.hits.items.len;
            }
        },
        1 => {
            var c = ClosestHitCollisionCollector(Collector).init();
            defer c.deinit();
            for (0..num_hits) |i| {
                if (i == 0 or bodies[i] != bodies[i - 1]) {
                    if (i > 0) c.base.onBodyEnd();
                    c.base.onBody(&body);
                }
                const r = make.at(i);
                c.base.addHit(&r);
                out_early_out[i] = c.base.getEarlyOutFraction();
            }
            if (num_hits > 0) c.base.onBodyEnd();
            out_early_out[num_hits] = c.base.getEarlyOutFraction();
            if (!c.hadHit()) return 0;
            store(&c.hit, &out_hits[0]);
            return 1;
        },
        else => {
            var c = AnyHitCollisionCollector(Collector).init();
            defer c.deinit();
            var i: usize = 0;
            while (i < num_hits and !c.base.shouldEarlyOut()) : (i += 1) {
                const r = make.at(i);
                c.base.addHit(&r);
            }
            out_early_out[0] = c.base.getEarlyOutFraction();
            if (!c.hadHit()) return 0;
            store(&c.hit, &out_hits[0]);
            return 1;
        },
    }
}

const MakeRay = struct {
    bodies: []const u32,
    fractions: []const f32,
    fn at(self: MakeRay, i: usize) RayCastResult {
        return .{ .body_id = .{ .id = self.bodies[i] }, .fraction = self.fractions[i], .sub_shape_id2 = .{ .value = @intCast(i) } };
    }
};

fn storeRay(r: *const RayCastResult, h: *Hit) void {
    h.* = .{ .body_id = r.body_id.getIndexAndSequenceNumber(), .fraction = r.fraction, .penetration_depth = @floatFromInt(r.sub_shape_id2.getValue()) };
}

const MakeCollide = struct {
    bodies: []const u32,
    depths: []const f32,
    fn at(self: MakeCollide, i: usize) CollideShapeResult {
        return .init(Vec3.zero(), Vec3.zero(), Vec3.axisX(), self.depths[i], .{ .value = @intCast(i) }, .empty, .{ .id = self.bodies[i] });
    }
};

fn storeCollide(r: *const CollideShapeResult, h: *Hit) void {
    h.* = .{ .body_id = r.body_id2.getIndexAndSequenceNumber(), .fraction = @floatFromInt(r.sub_shape_id1.getValue()), .penetration_depth = r.penetration_depth };
}

const MakeCast = struct {
    bodies: []const u32,
    fractions: []const f32,
    depths: []const f32,
    fn at(self: MakeCast, i: usize) ShapeCastResult {
        return .init(self.fractions[i], Vec3.zero(), Vec3.init(0, 0, self.depths[i]), Vec3.axisX(), false, .{ .value = @intCast(i) }, .empty, .{ .id = self.bodies[i] });
    }
};

fn storeCast(r: *const ShapeCastResult, h: *Hit) void {
    h.* = .{ .body_id = r.base.body_id2.getIndexAndSequenceNumber(), .fraction = r.fraction, .penetration_depth = r.base.penetration_depth + @as(f32, @floatFromInt(r.base.sub_shape_id1.getValue())) * 1000.0 };
}

test "ShapeCore parity: collectors on synthetic hit sequences (early out fractions, sort order with ties, per body)" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "collectors" };
    for (0..iterations / 10) |_| {
        const num_hits = gen.index(24);
        var bodies: [24]u32 = undefined;
        var fractions: [24]f32 = undefined;
        var depths: [24]f32 = undefined;
        var body: u32 = 1;
        for (0..num_hits) |i| {
            if (gen.oneIn(3)) body += 1 + @as(u32, @intCast(gen.index(2)));
            if (gen.oneIn(8)) body -= @min(body - 1, 2); // Bodies can repeat after another one (a new OnBody)
            bodies[i] = body;
            // Grid values give exact ties (QuickSort's order of equal keys), in [0, 1] (the collectors assume early outs decrease)
            fractions[i] = if (gen.oneIn(2)) @as(f32, @floatFromInt(gen.index(9))) / 8.0 else gen.plain(0, 1);
            depths[i] = if (gen.oneIn(2)) @as(f32, @floatFromInt(gen.index(5))) / 4.0 else gen.plain(0, 2);
        }

        for (0..4) |kind| for (0..3) |result_type| {
            var jolt_early_out: [25]f32 = @splat(0);
            var jolt_hits: [24]Hit = undefined;
            const jolt_count = jolt.jolt_collector(@intCast(kind), @intCast(result_type), @intCast(num_hits), &bodies, &fractions, &depths, &jolt_early_out, &jolt_hits);

            var zolt_early_out: [25]f32 = @splat(0);
            var zolt_hits: [24]Hit = undefined;
            const zolt_count = switch (result_type) {
                0 => try collect(allocator, kind, CastRayCollector, num_hits, &bodies, MakeRay{ .bodies = &bodies, .fractions = &fractions }, storeRay, &zolt_early_out, &zolt_hits),
                1 => try collect(allocator, kind, CollideShapeCollector, num_hits, &bodies, MakeCollide{ .bodies = &bodies, .depths = &depths }, storeCollide, &zolt_early_out, &zolt_hits),
                else => try collect(allocator, kind, CastShapeCollector, num_hits, &bodies, MakeCast{ .bodies = &bodies, .fractions = &fractions, .depths = &depths }, storeCast, &zolt_early_out, &zolt_hits),
            };
            checker.check(.{ kind, result_type, num_hits }, .{ @as(c_int, @intCast(zolt_count)), zolt_early_out }, .{ jolt_count, jolt_early_out });
            if (zolt_count != jolt_count) continue;
            for (0..zolt_count) |i| checker.check(.{ kind, result_type, num_hits, i }, .{ zolt_hits[i].body_id, zolt_hits[i].fraction, zolt_hits[i].penetration_depth }, .{ jolt_hits[i].body_id, jolt_hits[i].fraction, jolt_hits[i].penetration_depth });
        };
    }
    try checker.finish();
}

/// True if registering `T` alone changes the registry (a stub registers nothing)
fn registersSomething(comptime T: type) bool {
    return comptime !std.meta.eql(CollisionDispatch.Registry.build(.{T}), CollisionDispatch.Registry.init());
}

/// Names of the functions in a dispatch table: -1 unsupported, -2 reversed, others the index of their first occurrence
fn tableNames(comptime F: type, table: *const [num_sub_shape_types][num_sub_shape_types]F, unsupported: F, reversed: F) [num_sub_shape_types * num_sub_shape_types]c_int {
    var names: [num_sub_shape_types * num_sub_shape_types]c_int = undefined;
    var seen: [num_sub_shape_types * num_sub_shape_types]F = undefined;
    var num_seen: usize = 0;
    for (0..num_sub_shape_types) |i| for (0..num_sub_shape_types) |j| {
        const f = table[i][j];
        const name = &names[i * num_sub_shape_types + j];
        if (f == unsupported) {
            name.* = -1;
        } else if (f == reversed) {
            name.* = -2;
        } else if (std.mem.indexOfScalar(F, seen[0..num_seen], f)) |k| {
            name.* = @intCast(k);
        } else {
            name.* = @intCast(num_seen);
            seen[num_seen] = f;
            num_seen += 1;
        }
    };
    return names;
}

test "ShapeCore parity: CollisionDispatch and ShapeFunctions tables (Jolt's RegisterTypes order, ported classes)" {
    // The classes whose Zolt port registers something (bit k = k-th class of Jolt's RegisterTypes order)
    var mask: u32 = 0;
    inline for (RegisterTypes.registration_order, 0..) |T, k| {
        if (registersSomething(T)) mask |= @as(u32, 1) << k;
    }
    // The parity build registers the functions of the parity shapes (ShapeCoreUserTypes.zig), jolt_dispatch_tables too
    try std.testing.expectEqual(@as(usize, 1), RegisterTypes.user_registrations.len);
    try std.testing.expect(RegisterTypes.user_registrations[0] == parity_user_types.ParityShapeRegistration);

    var jolt_collide: [num_sub_shape_types * num_sub_shape_types]c_int = undefined;
    var jolt_cast: [num_sub_shape_types * num_sub_shape_types]c_int = undefined;
    var jolt_construct: [num_sub_shape_types]c_int = undefined;
    var jolt_color: [num_sub_shape_types]u32 = undefined;
    jolt.jolt_dispatch_tables(mask, &jolt_collide, &jolt_cast, &jolt_construct, &jolt_color);

    const registry = &RegisterTypes.registry;
    // Named at compile time: the registry is comptime, and comptime function pointer equality is function identity.
    // At runtime an optimized build can fold functions with identical machine code to one address (ReleaseFast did),
    // which would give two different functions the same name.
    const zolt_collide = comptime blk: {
        @setEvalBranchQuota(1_000_000);
        break :blk tableNames(CollisionDispatch.CollideShape, &RegisterTypes.registry.collide_shape, &CollisionDispatch.collideUnsupported, &CollisionDispatch.reversedCollideShape);
    };
    const zolt_cast = comptime blk: {
        @setEvalBranchQuota(1_000_000);
        break :blk tableNames(CollisionDispatch.CastShape, &RegisterTypes.registry.cast_shape, &CollisionDispatch.castUnsupported, &CollisionDispatch.reversedCastShape);
    };
    var zolt_construct: [num_sub_shape_types]c_int = undefined;
    var zolt_color: [num_sub_shape_types]u32 = undefined;
    for (registry.shape_functions, 0..) |f, i| {
        zolt_construct[i] = @intFromBool(f.construct != null);
        zolt_color[i] = f.color.getUInt32();
    }

    var checker: Checker = .{ .name = "dispatch tables" };
    for (0..num_sub_shape_types) |i| for (0..num_sub_shape_types) |j| {
        const k = i * num_sub_shape_types + j;
        checker.check(.{ mask, zolt.sub_shape_type_names[i], zolt.sub_shape_type_names[j] }, .{ zolt_collide[k], zolt_cast[k] }, .{ jolt_collide[k], jolt_cast[k] });
    };
    for (0..num_sub_shape_types) |i| checker.check(.{ mask, zolt.sub_shape_type_names[i] }, .{ zolt_construct[i], zolt_color[i] }, .{ jolt_construct[i], jolt_color[i] });
    try checker.finish();
}
