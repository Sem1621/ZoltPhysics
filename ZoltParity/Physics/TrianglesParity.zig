//! Parity tests for the triangle collision algorithms (Phase 4, Wave A): CollideConvexVsTriangles,
//! CollideSphereVsTriangles, CastConvexVsTriangles, CastSphereVsTriangles, ManifoldBetweenTwoFaces /
//! PruneContactPoints, InternalEdgeRemovingCollector (on hits from the triangle colliders, on recorded hit sequences and
//! through its sCollideShapeVsShape), CollideShapeVsShapePerLeaf and CollideSoftBodyVerticesVsTriangles. The triangle
//! classes are driven directly with triangle sequences (no triangle shape exists yet): random soups, connected meshes
//! (grids) with active edge flags, back facing, degenerate and far away triangles, triangles that exactly touch the
//! shape, with sphere and box shapes (built from their settings on both sides) with scales (negative components / inside
//! out), random transforms, the back face / active edge / collect faces modes, max separation distances, active edge
//! movement directions, tolerances and early out fractions. Every collector hit is compared in order, with the final
//! early out fractions. C ABI wrappers: ZoltParity/Physics/TrianglesReference.cpp.
//!
//! CollideShapeVsShapePerLeaf only gets single leaf shapes (spheres and boxes): the compound shapes that have more
//! leaves are ported in Wave B (the inline tests of CollideShapeVsShapePerLeaf.zig cover compounds with the test shapes).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const Allocator = std.mem.Allocator;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const Body = zolt.Body;
const BoxShapeSettings = zolt.BoxShapeSettings;
const CastConvexVsTriangles = zolt.CastConvexVsTriangles;
const CastShapeCollector = zolt.CastShapeCollector;
const CastSphereVsTriangles = zolt.CastSphereVsTriangles;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const ClosestHitPerBodyCollisionCollector = zolt.ClosestHitPerBodyCollisionCollector;
const CollideConvexVsTriangles = zolt.CollideConvexVsTriangles;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollideSoftBodyVertexIterator = zolt.CollideSoftBodyVertexIterator;
const CollideSoftBodyVerticesVsTriangles = zolt.CollideSoftBodyVerticesVsTriangles;
const CollideSphereVsTriangles = zolt.CollideSphereVsTriangles;
const ContactPoints = zolt.ContactPoints;
const ConvexShape = zolt.ConvexShape;
const InternalEdgeRemovingCollector = zolt.InternalEdgeRemovingCollector;
const Mat44 = zolt.Mat44;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const Ref = zolt.Ref;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastResult = zolt.ShapeCastResult;
const ShapeCastSettings = zolt.ShapeCastSettings;
const SphereShape = zolt.SphereShape;
const SphereShapeSettings = zolt.SphereShapeSettings;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TransformedShape = zolt.TransformedShape;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const collideShapeVsShapePerLeaf = zolt.collideShapeVsShapePerLeaf;
const manifoldBetweenTwoFaces = zolt.manifoldBetweenTwoFaces;
const math = zolt.math;
const pruneContactPoints = zolt.pruneContactPoints;

/// The C++ reference functions, see TrianglesReference.cpp
const jolt = struct {
    extern fn jolt_triangles_collide(input: *const CollideTrianglesInput, output: *HitsOutput) void;
    extern fn jolt_triangles_cast(input: *const CastTrianglesInput, output: *HitsOutput) void;
    extern fn jolt_triangles_manifold(input: *const ManifoldInput, output: *ManifoldOutput) void;
    extern fn jolt_triangles_prune(axis: *const P, count: u32, points1: [*]const f32, points2: [*]const f32, output: *ManifoldOutput) void;
    extern fn jolt_triangles_internal_edges(input: *const RecordedHitsInput, output: *HitsOutput) void;
    extern fn jolt_triangles_shape_pair(input: *const ShapePairInput, output: *HitsOutput) void;
    extern fn jolt_triangles_soft_body(input: *const SoftBodyInput, output: *SoftBodyOutput) void;
};

const max_triangles = 64;
const max_hits = 64;
const max_soft_body_vertices = 16;
const max_soft_body_triangles = 32;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// Shape description, must match TriShapeDesc in TrianglesReference.cpp
const TriShapeDesc = extern struct {
    /// 0: SphereShape, 1: BoxShape
    kind: u32,
    radius: f32 = 0.0,
    half_extent: P = .{ 0, 0, 0 },
    convex_radius: f32 = 0.0,
};

/// Must match TriangleInput in TrianglesReference.cpp
const TriangleInput = extern struct {
    v: [9]f32,
    active_edges: u32,
    sub_shape_id2: u32,
};

/// Must match HitOutput in TrianglesReference.cpp (also the input of the recorded hits)
const HitOutput = extern struct {
    /// Cast only
    fraction: f32,
    /// Cast only
    back_face: c_int,
    point1: P,
    point2: P,
    axis: P,
    depth: f32,
    id1: u32,
    id2: u32,
    body_id: u32,
    face1_count: u32,
    face2_count: u32,
    face1: [32 * 3]f32,
    face2: [32 * 3]f32,
};

/// Must match HitsOutput in TrianglesReference.cpp
const HitsOutput = extern struct {
    num_hits: u32,
    /// Early out fraction of the collector after the query
    early_out: f32,
    /// Early out fraction of the InternalEdgeRemovingCollector (when used)
    wrapper_early_out: f32,
    padding: u32,
    hits: [max_hits]HitOutput,
};

/// Must match CollideTrianglesInput in TrianglesReference.cpp
const CollideTrianglesInput = extern struct {
    shape1: TriShapeDesc,
    scale1: P,
    scale2: P,
    transform1: [16]f32,
    transform2: [16]f32,
    sub_shape_id1: u32,
    /// 1: CollideWithAll
    active_edge_mode: c_int,
    collect_faces: c_int,
    /// 1: CollideWithBackFaces
    back_face_mode: c_int,
    max_separation_distance: f32,
    collision_tolerance: f32,
    penetration_tolerance: f32,
    active_edge_movement_direction: P,
    vertex_tolerance_sq: f32,
    /// 0: AllHit, 1: ClosestHit, 2: AnyHit
    collector: c_int,
    /// 0: CollideConvexVsTriangles, 1: CollideSphereVsTriangles
    use_sphere_collider: c_int,
    /// Wrap the collector in an InternalEdgeRemovingCollector (and flush)
    internal_edge_removal: c_int,
    early_out: f32,
    body_id: u32,
    num_triangles: u32,
    triangles: [max_triangles]TriangleInput,
};

/// Must match CastTrianglesInput in TrianglesReference.cpp
const CastTrianglesInput = extern struct {
    shape1: TriShapeDesc,
    scale1: P,
    start: [16]f32,
    direction: P,
    scale2: P,
    transform2: [16]f32,
    creator1: [2]u32,
    active_edge_mode: c_int,
    collect_faces: c_int,
    back_face_mode_triangles: c_int,
    back_face_mode_convex: c_int,
    collision_tolerance: f32,
    penetration_tolerance: f32,
    extra_convex_radius: f32,
    use_shrunken_shape: c_int,
    return_deepest_point: c_int,
    active_edge_movement_direction: P,
    /// 0: AllHit, 1: ClosestHit
    collector: c_int,
    /// 0: CastConvexVsTriangles, 1: CastSphereVsTriangles
    use_sphere_caster: c_int,
    early_out: f32,
    body_id: u32,
    num_triangles: u32,
    triangles: [max_triangles]TriangleInput,
};

/// Must match ManifoldInput in TrianglesReference.cpp
const ManifoldInput = extern struct {
    contact_point1: P,
    contact_point2: P,
    penetration_axis: P,
    max_contact_distance: f32,
    face1_count: u32,
    face2_count: u32,
    face1: [32 * 3]f32,
    face2: [32 * 3]f32,
    num_existing: u32,
    existing1: [32 * 3]f32,
    existing2: [32 * 3]f32,
    prune: c_int,
};

/// Must match ManifoldOutput in TrianglesReference.cpp
const ManifoldOutput = extern struct {
    count1: u32,
    count2: u32,
    points1: [64 * 3]f32,
    points2: [64 * 3]f32,
};

/// Must match RecordedHitsInput in TrianglesReference.cpp
const RecordedHitsInput = extern struct {
    num_hits: u32,
    vertex_tolerance_sq: f32,
    /// 0: AllHit, 1: ClosestHit, 2: ClosestHitPerBody
    collector: c_int,
    /// Call OnBody / OnBodyEnd around the hits of each body (otherwise Flush at the end)
    use_bodies: c_int,
    early_out: f32,
    body_id: u32,
    hit_body: [max_hits]u32,
    hits: [max_hits]HitOutput,
};

/// Must match ShapePairInput in TrianglesReference.cpp
const ShapePairInput = extern struct {
    shape1: TriShapeDesc,
    shape2: TriShapeDesc,
    scale1: P,
    scale2: P,
    transform1: [16]f32,
    transform2: [16]f32,
    creator1: [2]u32,
    creator2: [2]u32,
    /// 0: InternalEdgeRemovingCollector::sCollideShapeVsShape, 1: CollideShapeVsShapePerLeaf<AnyHit>, 2: CollideShapeVsShapePerLeaf<ClosestHit>
    mode: c_int,
    active_edge_mode: c_int,
    collect_faces: c_int,
    max_separation_distance: f32,
    vertex_tolerance_sq: f32,
    /// 0: AllHit, 1: ClosestHit
    collector: c_int,
    early_out: f32,
    body_id: u32,
};

/// Must match SoftBodyInput in TrianglesReference.cpp
const SoftBodyInput = extern struct {
    transform: [16]f32,
    scale: P,
    triangle_thickness: f32,
    num_vertices: u32,
    colliding_shape_index: c_int,
    positions: [max_soft_body_vertices * 3]f32,
    penetrations: [max_soft_body_vertices]f32,
    planes: [max_soft_body_vertices * 4]f32,
    indices: [max_soft_body_vertices]c_int,
    /// Number of triangles processed for each vertex (a prefix of `triangles`)
    num_triangles: [max_soft_body_vertices]u32,
    triangles: [max_soft_body_triangles * 9]f32,
};

/// Must match SoftBodyOutput in TrianglesReference.cpp
const SoftBodyOutput = extern struct {
    penetrations: [max_soft_body_vertices]f32,
    planes: [max_soft_body_vertices * 4]f32,
    indices: [max_soft_body_vertices]c_int,
};

// ---------------------------------------------------------------------------------------------------------------------
// Conversions

fn vec3(a: P) Vec3 {
    return Vec3.init(a[0], a[1], a[2]);
}

fn vec3At(a: []const f32, i: usize) Vec3 {
    return Vec3.init(a[3 * i], a[3 * i + 1], a[3 * i + 2]);
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

fn mat44(a: [16]f32) Mat44 {
    return Mat44.init(vec4(a[0..4].*), vec4(a[4..8].*), vec4(a[8..12].*), vec4(a[12..16].*));
}

fn arr16(m: Mat44) [16]f32 {
    return arr4(m.getColumn4(0)) ++ arr4(m.getColumn4(1)) ++ arr4(m.getColumn4(2)) ++ arr4(m.getColumn4(3));
}

fn makeCreator(c: [2]u32) SubShapeIDCreator {
    return SubShapeIDCreator.pushID(.{}, c[0], c[1]);
}

fn makeID(value: u32) SubShapeID {
    var id: SubShapeID = .empty;
    id.setValue(value);
    return id;
}

/// Build the shape from its settings (the descriptions are always valid)
fn createShape(allocator: Allocator, desc: TriShapeDesc) !Ref(Shape) {
    var result = if (desc.kind == 0) blk: {
        var settings = SphereShapeSettings.init(allocator, desc.radius, .{});
        defer settings.deinit();
        break :blk try settings.asShapeSettings().createShape(allocator);
    } else blk: {
        var settings = BoxShapeSettings.init(allocator, vec3(desc.half_extent), .{ .convex_radius = desc.convex_radius });
        defer settings.deinit();
        break :blk try settings.asShapeSettings().createShape(allocator);
    };
    defer result.deinit();
    return Ref(Shape).init(result.getPtr().?);
}

fn storeFace(face: *const Shape.SupportingFace, out_count: *u32, out_face: *[32 * 3]f32) void {
    out_count.* = face.len;
    for (face.constSlice(), 0..) |v, i| out_face[3 * i ..][0..3].* = arr3(v);
}

fn storeCollideHit(result: *const CollideShapeResult, out: *HitOutput) void {
    out.point1 = arr3(result.contact_point_on1);
    out.point2 = arr3(result.contact_point_on2);
    out.axis = arr3(result.penetration_axis);
    out.depth = result.penetration_depth;
    out.id1 = result.sub_shape_id1.getValue();
    out.id2 = result.sub_shape_id2.getValue();
    out.body_id = result.body_id2.getIndexAndSequenceNumber();
    storeFace(&result.shape1_face, &out.face1_count, &out.face1);
    storeFace(&result.shape2_face, &out.face2_count, &out.face2);
}

fn addCollideHit(result: *const CollideShapeResult, out: *HitsOutput) void {
    if (out.num_hits < max_hits) {
        const h = &out.hits[out.num_hits];
        h.fraction = 0.0;
        h.back_face = 0;
        storeCollideHit(result, h);
    }
    out.num_hits += 1;
}

fn addCastHit(result: *const ShapeCastResult, out: *HitsOutput) void {
    if (out.num_hits < max_hits) {
        const h = &out.hits[out.num_hits];
        h.fraction = result.fraction;
        h.back_face = @intFromBool(result.is_back_face_hit);
        storeCollideHit(&result.base, h);
    }
    out.num_hits += 1;
}

fn loadHit(h: *const HitOutput) CollideShapeResult {
    var r = CollideShapeResult.init(vec3(h.point1), vec3(h.point2), vec3(h.axis), h.depth, makeID(h.id1), makeID(h.id2), .init(h.body_id));
    for (0..h.face1_count) |i| r.shape1_face.append(vec3At(&h.face1, i));
    for (0..h.face2_count) |i| r.shape2_face.append(vec3At(&h.face2, i));
    return r;
}

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of the wrappers

/// Run `query.run(collector)` with the requested collector type (0: AllHit, 1: ClosestHit, 2: AnyHit), the context and
/// the early out fraction, and store the hits
fn runWithCollector(comptime CollectorBase: type, allocator: Allocator, collector_type: c_int, body_id: u32, early_out: f32, out: *HitsOutput, query: anytype) !void {
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(body_id), .{});
    const store = if (CollectorBase == CastShapeCollector) addCastHit else addCollideHit;
    switch (collector_type) {
        0 => {
            var collector = AllHitCollisionCollector(CollectorBase).init(allocator);
            defer collector.deinit();
            collector.base.setContext(&context);
            if (early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(early_out);
            try query.run(&collector.base);
            try collector.checkError();
            for (collector.hits.items) |*h| store(h, out);
            out.early_out = collector.base.getEarlyOutFraction();
        },
        1 => {
            var collector = ClosestHitCollisionCollector(CollectorBase).init();
            defer collector.deinit();
            collector.base.setContext(&context);
            if (early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(early_out);
            try query.run(&collector.base);
            if (collector.hadHit()) store(&collector.hit, out);
            out.early_out = collector.base.getEarlyOutFraction();
        },
        else => {
            var collector = AnyHitCollisionCollector(CollectorBase).init();
            defer collector.deinit();
            collector.base.setContext(&context);
            if (early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(early_out);
            try query.run(&collector.base);
            if (collector.hadHit()) store(&collector.hit, out);
            out.early_out = collector.base.getEarlyOutFraction();
        },
    }
}

fn collideSettings(input: *const CollideTrianglesInput) CollideShapeSettings {
    var settings: CollideShapeSettings = .{};
    settings.active_edge_mode = if (input.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    settings.back_face_mode = if (input.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.max_separation_distance = input.max_separation_distance;
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.active_edge_movement_direction = vec3(input.active_edge_movement_direction);
    settings.internal_edge_removal_vertex_tolerance_sq = input.vertex_tolerance_sq;
    return settings;
}

fn zoltCollide(allocator: Allocator, input: *const CollideTrianglesInput) !HitsOutput {
    var shape1 = try createShape(allocator, input.shape1);
    defer shape1.deinit();
    const settings = collideSettings(input);
    var out = std.mem.zeroes(HitsOutput);

    const Query = struct {
        allocator: Allocator,
        input: *const CollideTrianglesInput,
        shape1: *const Shape,
        settings: *const CollideShapeSettings,
        out: *HitsOutput,

        fn collide(q: *const @This(), collector: *CollideShapeCollector) void {
            const in = q.input;
            if (in.use_sphere_collider != 0) {
                var collider = CollideSphereVsTriangles.init(q.shape1.cast(SphereShape), vec3(in.scale1), vec3(in.scale2), mat44(in.transform1), mat44(in.transform2), makeID(in.sub_shape_id1), q.settings, collector);
                for (in.triangles[0..in.num_triangles]) |*t|
                    collider.collide(vec3At(&t.v, 0), vec3At(&t.v, 1), vec3At(&t.v, 2), @intCast(t.active_edges), makeID(t.sub_shape_id2));
            } else {
                var collider = CollideConvexVsTriangles.init(q.shape1.cast(ConvexShape), vec3(in.scale1), vec3(in.scale2), mat44(in.transform1), mat44(in.transform2), makeID(in.sub_shape_id1), q.settings, collector);
                for (in.triangles[0..in.num_triangles]) |*t|
                    collider.collide(vec3At(&t.v, 0), vec3At(&t.v, 1), vec3At(&t.v, 2), @intCast(t.active_edges), makeID(t.sub_shape_id2));
            }
        }

        pub fn run(q: *const @This(), collector: *CollideShapeCollector) !void {
            if (q.input.internal_edge_removal != 0) {
                var wrapper: InternalEdgeRemovingCollector = undefined;
                wrapper.init(collector, q.settings.internal_edge_removal_vertex_tolerance_sq, q.allocator);
                defer wrapper.deinit();
                q.collide(&wrapper.base);
                wrapper.flush();
                try wrapper.checkError();
                q.out.wrapper_early_out = wrapper.base.getEarlyOutFraction();
            } else q.collide(collector);
        }
    };
    const query: Query = .{ .allocator = allocator, .input = input, .shape1 = shape1.get().?, .settings = &settings, .out = &out };
    try runWithCollector(CollideShapeCollector, allocator, input.collector, input.body_id, input.early_out, &out, &query);
    return out;
}

fn zoltCast(allocator: Allocator, input: *const CastTrianglesInput) !HitsOutput {
    var shape1 = try createShape(allocator, input.shape1);
    defer shape1.deinit();
    var settings: ShapeCastSettings = .{};
    settings.active_edge_mode = if (input.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    settings.back_face_mode_triangles = if (input.back_face_mode_triangles != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.back_face_mode_convex = if (input.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.extra_convex_radius = input.extra_convex_radius;
    settings.use_shrunken_shape_and_convex_radius = input.use_shrunken_shape != 0;
    settings.return_deepest_point = input.return_deepest_point != 0;
    settings.active_edge_movement_direction = vec3(input.active_edge_movement_direction);
    const shape_cast = ShapeCast.init(shape1.get().?, vec3(input.scale1), mat44(input.start), vec3(input.direction));
    var out = std.mem.zeroes(HitsOutput);

    const Query = struct {
        input: *const CastTrianglesInput,
        shape_cast: *const ShapeCast,
        settings: *const ShapeCastSettings,

        pub fn run(q: *const @This(), collector: *CastShapeCollector) !void {
            const in = q.input;
            if (in.use_sphere_caster != 0) {
                var caster = CastSphereVsTriangles.init(q.shape_cast, q.settings, vec3(in.scale2), mat44(in.transform2), makeCreator(in.creator1), collector);
                for (in.triangles[0..in.num_triangles]) |*t|
                    caster.cast(vec3At(&t.v, 0), vec3At(&t.v, 1), vec3At(&t.v, 2), @intCast(t.active_edges), makeID(t.sub_shape_id2));
            } else {
                var caster = CastConvexVsTriangles.init(q.shape_cast, q.settings, vec3(in.scale2), mat44(in.transform2), makeCreator(in.creator1), collector);
                for (in.triangles[0..in.num_triangles]) |*t|
                    caster.cast(vec3At(&t.v, 0), vec3At(&t.v, 1), vec3At(&t.v, 2), @intCast(t.active_edges), makeID(t.sub_shape_id2));
            }
        }
    };
    const query: Query = .{ .input = input, .shape_cast = &shape_cast, .settings = &settings };
    try runWithCollector(CastShapeCollector, allocator, input.collector, input.body_id, input.early_out, &out, &query);
    return out;
}

fn storePoints(points1: *const ContactPoints, points2: *const ContactPoints) ManifoldOutput {
    var out = std.mem.zeroes(ManifoldOutput);
    out.count1 = points1.len;
    out.count2 = points2.len;
    for (points1.constSlice(), 0..) |p, i| out.points1[3 * i ..][0..3].* = arr3(p);
    for (points2.constSlice(), 0..) |p, i| out.points2[3 * i ..][0..3].* = arr3(p);
    return out;
}

fn zoltManifold(input: *const ManifoldInput) ManifoldOutput {
    var face1: Shape.SupportingFace = .empty;
    var face2: Shape.SupportingFace = .empty;
    for (0..input.face1_count) |i| face1.append(vec3At(&input.face1, i));
    for (0..input.face2_count) |i| face2.append(vec3At(&input.face2, i));
    var points1: ContactPoints = .empty;
    var points2: ContactPoints = .empty;
    for (0..input.num_existing) |i| {
        points1.append(vec3At(&input.existing1, i));
        points2.append(vec3At(&input.existing2, i));
    }
    const axis = vec3(input.penetration_axis);
    manifoldBetweenTwoFaces(vec3(input.contact_point1), vec3(input.contact_point2), axis, input.max_contact_distance, &face1, &face2, &points1, &points2);
    if (input.prune != 0 and points1.len > 4)
        pruneContactPoints(axis.normalized(), &points1, &points2);
    return storePoints(&points1, &points2);
}

fn zoltPrune(axis: P, count: u32, points1_in: []const f32, points2_in: []const f32) ManifoldOutput {
    var points1: ContactPoints = .empty;
    var points2: ContactPoints = .empty;
    for (0..count) |i| {
        points1.append(vec3At(points1_in, i));
        points2.append(vec3At(points2_in, i));
    }
    pruneContactPoints(vec3(axis), &points1, &points2);
    return storePoints(&points1, &points2);
}

fn feedRecorded(allocator: Allocator, input: *const RecordedHitsInput, chained: *CollideShapeCollector, out: *HitsOutput) !void {
    var wrapper: InternalEdgeRemovingCollector = undefined;
    wrapper.init(chained, input.vertex_tolerance_sq, allocator);
    defer wrapper.deinit();
    if (input.use_bodies != 0) {
        const body: Body = .{};
        var current_body: u32 = std.math.maxInt(u32);
        for (0..input.num_hits) |i| {
            if (input.hit_body[i] != current_body) {
                if (current_body != std.math.maxInt(u32))
                    wrapper.base.onBodyEnd();
                wrapper.base.onBody(&body);
                current_body = input.hit_body[i];
            }
            const hit = loadHit(&input.hits[i]);
            wrapper.base.addHit(&hit);
        }
        if (current_body != std.math.maxInt(u32))
            wrapper.base.onBodyEnd();
    } else {
        for (0..input.num_hits) |i| {
            const hit = loadHit(&input.hits[i]);
            wrapper.base.addHit(&hit);
        }
        wrapper.flush();
    }
    try wrapper.checkError();
    out.wrapper_early_out = wrapper.base.getEarlyOutFraction();
}

fn zoltInternalEdges(allocator: Allocator, input: *const RecordedHitsInput) !HitsOutput {
    var out = std.mem.zeroes(HitsOutput);
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    switch (input.collector) {
        0 => {
            var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
            defer collector.deinit();
            collector.base.setContext(&context);
            if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
            try feedRecorded(allocator, input, &collector.base, &out);
            try collector.checkError();
            for (collector.hits.items) |*h| addCollideHit(h, &out);
            out.early_out = collector.base.getEarlyOutFraction();
        },
        1 => {
            var collector = ClosestHitCollisionCollector(CollideShapeCollector).init();
            defer collector.deinit();
            collector.base.setContext(&context);
            if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
            try feedRecorded(allocator, input, &collector.base, &out);
            if (collector.hadHit()) addCollideHit(&collector.hit, &out);
            out.early_out = collector.base.getEarlyOutFraction();
        },
        else => {
            var collector = ClosestHitPerBodyCollisionCollector(CollideShapeCollector).init(allocator);
            defer collector.deinit();
            collector.base.setContext(&context);
            try feedRecorded(allocator, input, &collector.base, &out);
            try collector.checkError();
            for (collector.hits.items) |*h| addCollideHit(h, &out);
            out.early_out = collector.base.getEarlyOutFraction();
        },
    }
    return out;
}

fn zoltShapePair(allocator: Allocator, input: *const ShapePairInput) !HitsOutput {
    var shape1 = try createShape(allocator, input.shape1);
    defer shape1.deinit();
    var shape2 = try createShape(allocator, input.shape2);
    defer shape2.deinit();
    var settings: CollideShapeSettings = .{};
    settings.active_edge_mode = if (input.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    settings.max_separation_distance = input.max_separation_distance;
    settings.internal_edge_removal_vertex_tolerance_sq = input.vertex_tolerance_sq;
    var out = std.mem.zeroes(HitsOutput);

    const Query = struct {
        allocator: Allocator,
        input: *const ShapePairInput,
        shape1: *const Shape,
        shape2: *const Shape,
        settings: *const CollideShapeSettings,

        pub fn run(q: *const @This(), collector: *CollideShapeCollector) !void {
            const in = q.input;
            const s1 = vec3(in.scale1);
            const s2 = vec3(in.scale2);
            const t1 = mat44(in.transform1);
            const t2 = mat44(in.transform2);
            const c1 = makeCreator(in.creator1);
            const c2 = makeCreator(in.creator2);
            switch (in.mode) {
                0 => try InternalEdgeRemovingCollector.collideShapeVsShape(q.allocator, q.shape1, q.shape2, s1, s2, t1, t2, c1, c2, q.settings, collector, &.{}),
                1 => try collideShapeVsShapePerLeaf(AnyHitCollisionCollector(CollideShapeCollector), q.allocator, q.shape1, q.shape2, s1, s2, t1, t2, c1, c2, q.settings, collector, &.{}),
                else => try collideShapeVsShapePerLeaf(ClosestHitCollisionCollector(CollideShapeCollector), q.allocator, q.shape1, q.shape2, s1, s2, t1, t2, c1, c2, q.settings, collector, &.{}),
            }
        }
    };
    const query: Query = .{ .allocator = allocator, .input = input, .shape1 = shape1.get().?, .shape2 = shape2.get().?, .settings = &settings };
    try runWithCollector(CollideShapeCollector, allocator, input.collector, input.body_id, input.early_out, &out, &query);
    return out;
}

fn zoltSoftBody(input: *const SoftBodyInput) SoftBodyOutput {
    const old_thickness = CollideSoftBodyVerticesVsTriangles.triangle_thickness;
    defer CollideSoftBodyVerticesVsTriangles.triangle_thickness = old_thickness;
    CollideSoftBodyVerticesVsTriangles.triangle_thickness = input.triangle_thickness;

    var out = std.mem.zeroes(SoftBodyOutput);
    var positions: [max_soft_body_vertices]Vec3 = undefined;
    var planes: [max_soft_body_vertices]Plane = undefined;
    var inv_masses: [max_soft_body_vertices]f32 = undefined;
    var indices: [max_soft_body_vertices]i32 = undefined;
    for (0..input.num_vertices) |i| {
        positions[i] = vec3At(&input.positions, i);
        planes[i] = .fromVec4(vec4(input.planes[4 * i ..][0..4].*));
        inv_masses[i] = 1.0;
        out.penetrations[i] = input.penetrations[i];
        indices[i] = input.indices[i];
    }

    var collider = CollideSoftBodyVerticesVsTriangles.init(mat44(input.transform), vec3(input.scale));
    for (0..input.num_vertices) |i| {
        const vertex = CollideSoftBodyVertexIterator.init(.init(&positions[i], .{}), .init(&inv_masses[i], .{}), .init(&planes[i], .{}), .init(&out.penetrations[i], .{}), .init(&indices[i], .{}));
        collider.startVertex(&vertex);
        for (0..input.num_triangles[i]) |t|
            collider.processTriangle(vec3At(&input.triangles, 3 * t), vec3At(&input.triangles, 3 * t + 1), vec3At(&input.triangles, 3 * t + 2));
        collider.finishVertex(&vertex, input.colliding_shape_index);
    }

    for (0..input.num_vertices) |i| {
        out.planes[4 * i ..][0..4].* = arr4(planes[i].normal_and_constant);
        out.indices[i] = indices[i];
    }
    return out;
}

// ---------------------------------------------------------------------------------------------------------------------
// Input generation

/// Values that tend to expose differences, mixed into the random inputs
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 1.0e-6, -1.0e-6, 3.0, -3.0 };

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

    fn flag(self: *Gen) c_int {
        return @intFromBool(self.oneIn(2));
    }

    /// Random float in [min, max), or one of the special values (10% of the time)
    fn float(self: *Gen, min: f32, max: f32) f32 {
        if (self.oneIn(10)) return special_values[self.index(special_values.len)];
        return self.rng.float(min, max);
    }

    fn plain(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn plainVec(self: *Gen, min: f32, max: f32) Vec3 {
        return Vec3.init(self.plain(min, max), self.plain(min, max), self.plain(min, max));
    }

    /// Integer in [-n, n] as float
    fn grid(self: *Gen, n: i32) f32 {
        return @floatFromInt(self.rng.intRange(i32, -n, n));
    }

    /// A direction: random, axis aligned, diagonal, zero or tiny
    fn direction(self: *Gen, length: f32) Vec3 {
        return switch (self.index(8)) {
            0 => Vec3.zero(),
            1 => blk: {
                var d = Vec3.zero();
                d.setComponent(@intCast(self.index(3)), if (self.oneIn(2)) length else -length);
                break :blk d;
            },
            2 => Vec3.init(self.grid(1), self.grid(1), self.grid(1)),
            3 => self.plainVec(-1.0e-6, 1.0e-6),
            else => self.plainVec(-length, length),
        };
    }

    /// A unit direction (not zero)
    fn unitDirection(self: *Gen) Vec3 {
        if (self.oneIn(4)) {
            var d = Vec3.zero();
            d.setComponent(@intCast(self.index(3)), if (self.oneIn(2)) 1.0 else -1.0);
            return d;
        }
        while (true) {
            const d = self.plainVec(-1, 1);
            const len_sq = d.lengthSq();
            if (len_sq > 1.0e-2 and len_sq <= 1.0) return d.normalized();
        }
    }

    /// A shape: sphere or box (with or without convex radius)
    fn shape(self: *Gen) TriShapeDesc {
        if (self.oneIn(2))
            return self.sphere();
        var half_extent = self.plainVec(0.05, 2);
        if (self.oneIn(5)) half_extent = Vec3.replicate(0.5);
        const min_extent = half_extent.reduceMin();
        const convex_radius: f32 = switch (self.index(4)) {
            0 => 0.0,
            1 => @min(0.05, min_extent), // cDefaultConvexRadius
            2 => min_extent, // As big as allowed
            else => self.plain(0, min_extent),
        };
        return .{ .kind = 1, .half_extent = arr3(half_extent), .convex_radius = convex_radius };
    }

    fn sphere(self: *Gen) TriShapeDesc {
        return .{ .kind = 0, .radius = if (self.oneIn(5)) 0.5 else self.plain(0.05, 2) };
    }

    /// A valid scale for the shape: uniform (with signs) for a sphere, anything non zero for a box
    fn scale(self: *Gen, desc: TriShapeDesc) Vec3 {
        if (self.oneIn(5)) return Vec3.one();
        const s = if (self.oneIn(4)) self.grid(2) else self.plain(0.2, 2.5);
        const m = if (s == 0.0) 1.0 else @abs(s);
        if (desc.kind == 0)
            return Vec3.init(if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m);
        return self.anyScale();
    }

    /// A scale for the triangles: anything non zero, sometimes inside out
    fn anyScale(self: *Gen) Vec3 {
        if (self.oneIn(4)) return Vec3.one();
        var r = self.plainVec(0.3, 2.5);
        for (0..3) |c| {
            if (self.oneIn(3)) r.setComponent(@intCast(c), -r.getComponent(@intCast(c)));
        }
        return r;
    }

    /// A unit quaternion, sometimes a rotation of a multiple of 90 degrees around an axis or the identity
    fn rotation(self: *Gen) Quat {
        if (self.oneIn(4)) {
            const axes = [_]Vec3{ Vec3.axisX(), Vec3.axisY(), Vec3.axisZ() };
            return Quat.rotation(axes[self.index(3)], @as(f32, @floatFromInt(self.index(4))) * 0.5 * math.pi);
        }
        while (true) {
            const q = self.rng.floatArray(4, -1, 1);
            const len_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
            if (len_sq > 1.0e-2 and len_sq <= 1.0) return Quat.init(q[0], q[1], q[2], q[3]).normalized();
        }
    }

    /// A rotation + translation matrix
    fn transform(self: *Gen, range: f32) Mat44 {
        return Mat44.rotationTranslation(self.rotation(), self.plainVec(-range, range));
    }

    fn creator(self: *Gen) [2]u32 {
        const bits: u32 = @intCast(self.index(9));
        return .{ if (bits == 0) 0 else self.next() & ((@as(u32, 1) << @intCast(bits)) - 1), bits };
    }

    fn activeEdges(self: *Gen) u32 {
        return switch (self.index(4)) {
            0 => 0b111,
            1 => 0b000,
            else => self.next() & 0b111,
        };
    }

    fn earlyOutCollide(self: *Gen) f32 {
        return if (self.oneIn(4)) self.plain(-2, 2) else math.flt_max;
    }

    fn vertexToleranceSq(self: *Gen) f32 {
        return switch (self.index(4)) {
            0 => zolt.physics_settings.default_internal_edge_removal_vertex_tolerance_sq,
            1 => 0.0,
            2 => 1.0e-4,
            else => self.plain(0, 0.05),
        };
    }
};

/// The extent of a shape (for placing triangles near it)
fn extentOf(desc: TriShapeDesc) f32 {
    return if (desc.kind == 0) desc.radius else @max(desc.half_extent[0], @max(desc.half_extent[1], desc.half_extent[2]));
}

/// Builds the triangle list of an input
const TriangleBuilder = struct {
    triangles: *[max_triangles]TriangleInput,
    count: *u32,

    fn add(self: TriangleBuilder, v0: Vec3, v1: Vec3, v2: Vec3, active_edges: u32, sub_shape_id2: u32) void {
        if (self.count.* >= max_triangles) return;
        self.triangles[self.count.*] = .{ .v = arr3(v0) ++ arr3(v1) ++ arr3(v2), .active_edges = active_edges, .sub_shape_id2 = sub_shape_id2 };
        self.count.* += 1;
    }
};

/// Fill `b` with triangles (in the local, unscaled space of shape 2) around `center` (also local, unscaled) with a world
/// space size of about `reach`; `scale2` converts world sizes to the unscaled space. With `allow_degenerate` false,
/// triangles that are (nearly) degenerate after scaling are moved far away (CollideConvexVsTriangles asserts on a zero
/// penetration axis, like Jolt in debug builds).
fn generateTriangles(gen: *Gen, b: TriangleBuilder, center: Vec3, reach: f32, scale2: Vec3, allow_degenerate: bool) void {
    const inv_scale = Vec3.one().div(scale2);
    const num_triangles: usize = switch (gen.index(4)) {
        0 => 1,
        1 => 2 + gen.index(8),
        else => 4 + gen.index(max_triangles - 4),
    };
    const sub_shape_base: u32 = gen.next() & 0xffff;
    switch (gen.index(3)) {
        0 => {
            // Random soup near the shape
            for (0..num_triangles) |i| {
                const v0 = center.add(gen.plainVec(-2 * reach, 2 * reach).mul(inv_scale));
                const v1 = v0.add(gen.plainVec(-2 * reach, 2 * reach).mul(inv_scale));
                const v2 = v0.add(gen.plainVec(-2 * reach, 2 * reach).mul(inv_scale));
                b.add(v0, v1, v2, gen.activeEdges(), sub_shape_base + @as(u32, @intCast(i)));
            }
        },
        1 => {
            // A connected grid of quads in a random plane through (or near) the shape, heights are noise
            const n: usize = 1 + gen.index(5);
            const rot = Mat44.rotationQuat(gen.rotation());
            const u = rot.getAxisX();
            const w = rot.getAxisZ();
            const up = rot.getAxisY();
            const cell = reach * gen.plain(0.3, 1.5);
            const offset = up.mulScalar(if (gen.oneIn(3)) 0.0 else gen.plain(-reach, reach));
            const noise = if (gen.oneIn(2)) 0.0 else cell * gen.plain(0, 0.5);
            var heights: [6][6]f32 = undefined;
            for (&heights) |*row| {
                for (row) |*h| h.* = if (noise == 0.0) 0.0 else gen.plain(-noise, noise);
            }
            const half = 0.5 * @as(f32, @floatFromInt(n)) * cell;
            const flip = gen.oneIn(4); // Back facing mesh
            var id: u32 = sub_shape_base;
            for (0..n) |x| {
                for (0..n) |z| {
                    var corners: [4]Vec3 = undefined;
                    for (0..2) |dx| {
                        for (0..2) |dz| {
                            const px = @as(f32, @floatFromInt(x + dx)) * cell - half;
                            const pz = @as(f32, @floatFromInt(z + dz)) * cell - half;
                            const world = u.mulScalar(px).add(w.mulScalar(pz)).add(up.mulScalar(heights[x + dx][z + dz])).add(offset);
                            corners[2 * dx + dz] = center.add(world.mul(inv_scale));
                        }
                    }
                    // corners: 0 = (x, z), 1 = (x, z + 1), 2 = (x + 1, z), 3 = (x + 1, z + 1)
                    if ((x + z) % 2 == 0) {
                        if (flip) b.add(corners[0], corners[2], corners[1], gen.activeEdges(), id) else b.add(corners[0], corners[1], corners[2], gen.activeEdges(), id);
                        if (flip) b.add(corners[2], corners[3], corners[1], gen.activeEdges(), id + 1) else b.add(corners[2], corners[1], corners[3], gen.activeEdges(), id + 1);
                    } else {
                        if (flip) b.add(corners[0], corners[3], corners[1], gen.activeEdges(), id) else b.add(corners[0], corners[1], corners[3], gen.activeEdges(), id);
                        if (flip) b.add(corners[0], corners[2], corners[3], gen.activeEdges(), id + 1) else b.add(corners[0], corners[3], corners[2], gen.activeEdges(), id + 1);
                    }
                    id += 2;
                }
            }
        },
        else => {
            // Special triangles: through the center, touching at a vertex / edge, degenerate, far away, reversed
            for (0..num_triangles) |i| {
                const id = sub_shape_base + @as(u32, @intCast(i));
                const d1 = gen.plainVec(-reach, reach).mul(inv_scale);
                const d2 = gen.plainVec(-reach, reach).mul(inv_scale);
                switch (gen.index(7)) {
                    0 => b.add(center, center.add(d1), center.add(d2), gen.activeEdges(), id), // Vertex at the center
                    1 => b.add(center.sub(d1), center.add(d1), center.add(d2), gen.activeEdges(), id), // Edge through the center
                    2 => b.add(center.add(d1), center.add(d1), center.add(d2), gen.activeEdges(), id), // Two equal vertices
                    3 => b.add(center.add(d1), center.add(d1.mulScalar(2)), center.add(d1.mulScalar(-0.5)), gen.activeEdges(), id), // Collinear
                    4 => b.add(center.add(d1).add(Vec3.init(1000, 0, 0)), center.add(d2).add(Vec3.init(1000, 0, 0)), center.add(Vec3.init(1000, 1, 0)), gen.activeEdges(), id), // Far away
                    5 => b.add(center.add(d1), center.add(d2), center.add(d1.cross(d2).mul(inv_scale)), gen.activeEdges(), id),
                    else => b.add(center.add(d2), center.add(d1), center.add(d1.add(d2).mulScalar(0.25)), gen.activeEdges(), id), // Small
                }
            }
        },
    }

    // Reverse the winding of some triangles (back faces)
    if (gen.oneIn(3)) {
        for (b.triangles[0..b.count.*]) |*t| {
            if (gen.oneIn(3)) {
                const v1 = t.v[3..6].*;
                t.v[3..6].* = t.v[6..9].*;
                t.v[6..9].* = v1;
            }
        }
    }

    // Remove (nearly) degenerate triangles from the neighborhood when they are not allowed
    if (!allow_degenerate) {
        for (b.triangles[0..b.count.*]) |*t| {
            const v0 = vec3At(&t.v, 0).mul(scale2);
            const v1 = vec3At(&t.v, 1).mul(scale2);
            const v2 = vec3At(&t.v, 2).mul(scale2);
            if (v1.sub(v0).cross(v2.sub(v0)).lengthSq() <= 1.0e-8) {
                for (0..3) |c| t.v[3 * c] += 1000.0;
            }
        }
    }
}

fn genCollide(gen: *Gen) CollideTrianglesInput {
    var input = std.mem.zeroes(CollideTrianglesInput);
    const use_sphere = gen.oneIn(3);
    input.use_sphere_collider = @intFromBool(use_sphere);
    input.shape1 = if (use_sphere) gen.sphere() else gen.shape();
    const scale1 = gen.scale(input.shape1);
    const scale2 = gen.anyScale();
    input.scale1 = arr3(scale1);
    input.scale2 = arr3(scale2);
    const reach = extentOf(input.shape1) * scale1.abs().reduceMax();

    // Shape 1 relative to shape 2 (in the scaled local space of 2)
    const transform2 = gen.transform(5);
    var relative = Mat44.rotationTranslation(gen.rotation(), gen.plainVec(-reach, reach));
    if (gen.oneIn(10)) relative = Mat44.identity();
    input.transform2 = arr16(transform2);
    input.transform1 = arr16(transform2.mul(relative));
    input.sub_shape_id1 = gen.next();

    input.active_edge_mode = @intFromBool(gen.oneIn(3));
    input.collect_faces = gen.flag();
    input.back_face_mode = gen.flag();
    input.max_separation_distance = switch (gen.index(5)) {
        0, 1 => 0.0,
        2 => gen.plain(0, 1),
        3 => gen.plain(1, 3), // Clamped to 1 in the EPA path
        else => 10.0,
    };
    input.collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance;
    input.penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance;
    input.active_edge_movement_direction = arr3(gen.direction(1));
    input.vertex_tolerance_sq = gen.vertexToleranceSq();
    input.collector = @intCast(gen.index(3));
    // Not with AnyHit: Flush can add several hits, AnyHit asserts that it gets one (Jolt's debug build does too)
    input.internal_edge_removal = @intFromBool(input.collector != 2 and gen.oneIn(2));
    if (input.internal_edge_removal != 0) {
        // Required by the InternalEdgeRemovingCollector
        input.active_edge_mode = 1;
        input.collect_faces = 1;
    }
    input.early_out = gen.earlyOutCollide();
    input.body_id = gen.next() & 0x7fffff;

    // Triangles around the center of shape 1 in the unscaled space of 2
    const center = relative.getTranslation().div(scale2);
    generateTriangles(gen, .{ .triangles = &input.triangles, .count = &input.num_triangles }, center, @max(reach, 0.05), scale2, use_sphere);
    return input;
}

fn genCast(gen: *Gen) CastTrianglesInput {
    var input = std.mem.zeroes(CastTrianglesInput);
    const use_sphere = gen.oneIn(2);
    input.use_sphere_caster = @intFromBool(use_sphere);
    input.shape1 = if (use_sphere) gen.sphere() else gen.shape();
    const scale1 = gen.scale(input.shape1);
    const scale2 = gen.anyScale();
    input.scale1 = arr3(scale1);
    input.scale2 = arr3(scale2);
    const reach = @max(extentOf(input.shape1) * scale1.abs().reduceMax(), 0.05);

    // The cast in the scaled local space of shape 2: start somewhere, move a few times the size of the shape
    const start = Mat44.rotationTranslation(gen.rotation(), gen.plainVec(-2 * reach, 2 * reach));
    const direction = switch (gen.index(4)) {
        0 => gen.direction(6 * reach),
        else => gen.unitDirection().mulScalar(gen.plain(0.1, 8) * reach),
    };
    input.start = arr16(start);
    input.direction = arr3(direction);
    input.transform2 = arr16(gen.transform(5));
    input.creator1 = gen.creator();

    input.active_edge_mode = @intFromBool(gen.oneIn(3));
    input.collect_faces = gen.flag();
    input.back_face_mode_triangles = gen.flag();
    input.back_face_mode_convex = gen.flag();
    input.collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance;
    input.penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance;
    input.extra_convex_radius = if (gen.oneIn(4)) gen.plain(0, 0.3) else 0.0;
    input.use_shrunken_shape = gen.flag();
    input.return_deepest_point = gen.flag();
    input.active_edge_movement_direction = arr3(gen.direction(1));
    input.collector = @intCast(gen.index(2));
    input.early_out = if (gen.oneIn(4)) gen.plain(-0.5, 1) else 2.0;
    input.body_id = gen.next() & 0x7fffff;

    // Triangles along the path of the cast (in the unscaled space of 2)
    const along = start.getTranslation().add(direction.mulScalar(gen.plain(-0.2, 1.2)));
    generateTriangles(gen, .{ .triangles = &input.triangles, .count = &input.num_triangles }, along.div(scale2), reach, scale2, true);
    return input;
}

/// A convex polygon with n vertices in the plane through `center` with normal `normal`, counter clockwise around the
/// normal unless `reverse`
fn polygon(gen: *Gen, out: *[32 * 3]f32, n: usize, center: Vec3, normal: Vec3, radius: f32, reverse: bool) void {
    const u = normal.getNormalizedPerpendicular();
    const v = normal.cross(u);
    const start = gen.plain(0, 2 * math.pi);
    for (0..n) |i| {
        const k = if (reverse) n - 1 - i else i;
        const angle = start + 2.0 * math.pi * @as(f32, @floatFromInt(k)) / @as(f32, @floatFromInt(n));
        const r = radius * gen.plain(0.7, 1.0);
        out[3 * i ..][0..3].* = arr3(center.add(u.mulScalar(zolt.trigonometry.cos(angle) * r)).add(v.mulScalar(zolt.trigonometry.sin(angle) * r)));
    }
}

fn genManifold(gen: *Gen) ManifoldInput {
    var input = std.mem.zeroes(ManifoldInput);
    const sizes = [_]u32{ 0, 1, 2, 2, 3, 3, 4, 4, 4, 5, 6, 8, 12 };
    input.face1_count = sizes[gen.index(sizes.len)];
    input.face2_count = sizes[gen.index(sizes.len)];
    const normal1 = gen.unitDirection();
    const center = gen.plainVec(-2, 2);
    const radius1 = gen.plain(0.1, 2);
    polygon(gen, &input.face1, input.face1_count, center, normal1, radius1, gen.oneIn(4));

    // Face 2: parallel (facing the other way or the same way), tilted, perpendicular or random
    const normal2 = switch (gen.index(5)) {
        0, 1 => normal1.negate(),
        2 => normal1.negate().add(gen.plainVec(-0.2, 0.2)).normalized(),
        3 => normal1.getNormalizedPerpendicular(),
        else => gen.unitDirection(),
    };
    const separation = switch (gen.index(4)) {
        0 => 0.0,
        else => gen.plain(-0.5, 0.5),
    };
    const center2 = center.add(normal1.mulScalar(separation)).add(gen.plainVec(-radius1, radius1).mulScalar(0.5));
    polygon(gen, &input.face2, input.face2_count, center2, normal2, gen.plain(0.1, 2), gen.oneIn(4));
    if (gen.oneIn(10) and input.face2_count >= 2) {
        // Degenerate face: all points the same
        for (1..input.face2_count) |i| input.face2[3 * i ..][0..3].* = input.face2[0..3].*;
    }

    // Penetration axis: along the normal of face 1 (either direction), perpendicular, random
    const axis = switch (gen.index(6)) {
        0, 1 => normal1.mulScalar(gen.plain(0.1, 3)),
        2 => normal1.negate().mulScalar(gen.plain(0.1, 3)),
        3 => normal1.getNormalizedPerpendicular(),
        4 => normal1.add(gen.plainVec(-0.3, 0.3)),
        else => gen.unitDirection().mulScalar(gen.plain(0.5, 2)),
    };
    input.penetration_axis = arr3(axis);
    input.contact_point1 = arr3(center.add(gen.plainVec(-0.5, 0.5)));
    input.contact_point2 = arr3(vec3(input.contact_point1).add(gen.plainVec(-0.5, 0.5)));
    input.max_contact_distance = if (gen.oneIn(4)) 0.02 else gen.plain(1.0e-3, 2);
    input.num_existing = if (gen.oneIn(2)) 0 else @intCast(gen.index(33));
    for (0..input.num_existing) |i| {
        input.existing1[3 * i ..][0..3].* = arr3(gen.plainVec(-2, 2));
        input.existing2[3 * i ..][0..3].* = arr3(gen.plainVec(-2, 2));
    }
    input.prune = @intFromBool(!axis.isNearZero(.{}) and gen.oneIn(2));
    return input;
}

/// Random contact point sets for PruneContactPoints: planar, noisy, collinear, coinciding, with or without penetration
fn genPrune(gen: *Gen, axis: *P, points1: *[64 * 3]f32, points2: *[64 * 3]f32) u32 {
    const count: u32 = if (gen.oneIn(5)) 64 else @intCast(5 + gen.index(60));
    const normal = gen.unitDirection();
    axis.* = arr3(normal);
    const u = normal.getNormalizedPerpendicular();
    const v = normal.cross(u);
    const center = gen.plainVec(-1, 1);
    const mode = gen.index(6);
    for (0..count) |i| {
        var p = switch (mode) {
            0 => center.add(u.mulScalar(gen.plain(-1, 1))), // Collinear
            1 => center, // All the same
            2 => center.add(u.mulScalar(gen.grid(2) * 0.5)).add(v.mulScalar(gen.grid(2) * 0.5)), // Grid (duplicates)
            else => center.add(u.mulScalar(gen.plain(-2, 2))).add(v.mulScalar(gen.plain(-2, 2))),
        };
        p = p.add(normal.mulScalar(if (gen.oneIn(2)) 0.0 else gen.plain(-0.1, 0.1)));
        points1[3 * i ..][0..3].* = arr3(p);
        const depth: f32 = switch (gen.index(4)) {
            0 => 0.0,
            1 => 0.05,
            else => gen.plain(-0.1, 0.3),
        };
        points2[3 * i ..][0..3].* = arr3(p.sub(normal.mulScalar(depth)));
    }
    return count;
}

/// Recorded hits against a small mesh (shared vertices): contacts on the interior, on edges, at vertices, with face
/// contact normals or not, equal depths, several sub shapes of shape 1, faces that are not triangles
fn genRecorded(gen: *Gen) RecordedHitsInput {
    var input = std.mem.zeroes(RecordedHitsInput);
    input.num_hits = switch (gen.index(5)) {
        0 => 1 + @as(u32, @intCast(gen.index(4))),
        1 => 32 + @as(u32, @intCast(gen.index(2))), // Around the local buffer size of the delayed results
        2 => max_hits,
        else => 1 + @as(u32, @intCast(gen.index(max_hits))),
    };
    input.vertex_tolerance_sq = gen.vertexToleranceSq();
    input.collector = @intCast(gen.index(3));
    input.use_bodies = if (input.collector == 2) 1 else gen.flag();
    input.early_out = if (input.collector == 0 and gen.oneIn(3)) gen.plain(-1, 1) else math.flt_max; // Other collectors would assert on worse hits
    input.body_id = gen.next() & 0x7fffff;

    // A 4 x 4 grid of vertices (3 x 3 quads, 18 triangles) with a random transform and height noise
    const transform = gen.transform(3);
    const cell = gen.plain(0.2, 2);
    const noise = if (gen.oneIn(2)) 0.0 else gen.plain(0, 0.3) * cell;
    var vertices: [16]Vec3 = undefined;
    for (0..4) |x| {
        for (0..4) |z| {
            const h = if (noise == 0.0) 0.0 else gen.plain(-noise, noise);
            vertices[4 * x + z] = transform.mulVec3(Vec3.init(@as(f32, @floatFromInt(x)) * cell, h, @as(f32, @floatFromInt(z)) * cell));
        }
    }
    var triangles: [18][3]u8 = undefined;
    var t: usize = 0;
    for (0..3) |x| {
        for (0..3) |z| {
            const c0: u8 = @intCast(4 * x + z);
            const c1 = c0 + 1;
            const c2 = c0 + 4;
            const c3 = c0 + 5;
            triangles[t] = .{ c0, c1, c2 };
            triangles[t + 1] = .{ c2, c1, c3 };
            t += 2;
        }
    }

    var body: u32 = 0;
    for (0..input.num_hits) |i| {
        if (input.collector != 2 and gen.oneIn(12)) body += 1;
        input.hit_body[i] = body;
        const h = &input.hits[i];
        const tri_index = gen.index(18);
        const tri = triangles[tri_index];
        var face: [4]Vec3 = .{ vertices[tri[0]], vertices[tri[1]], vertices[tri[2]], Vec3.zero() };
        var face_count: u32 = 3;
        switch (gen.index(20)) {
            0 => face_count = 0,
            1 => face_count = 1,
            2 => face_count = 2,
            3 => face[1] = face[0], // Degenerate
            4 => {
                // A quad (the triangle and the 4th vertex of its quad)
                face_count = 4;
                face[3] = vertices[@min(15, tri[2] + 1)];
            },
            5 => std.mem.swap(Vec3, &face[1], &face[2]), // Reversed winding
            6 => {
                // Not part of the mesh
                face = .{ gen.plainVec(-5, 5), gen.plainVec(-5, 5), gen.plainVec(-5, 5), Vec3.zero() };
            },
            else => {},
        }
        h.face2_count = face_count;
        for (0..face_count) |k| h.face2[3 * k ..][0..3].* = arr3(face[k]);

        // Contact point on the face: interior, on an edge, at a vertex (exactly or nearly) or anywhere
        const a = face[0];
        const b = face[1];
        const c = face[2];
        const point2 = switch (gen.index(6)) {
            0 => blk: {
                const u = gen.plain(0, 1);
                const v = gen.plain(0, 1 - u);
                break :blk a.add(b.sub(a).mulScalar(u)).add(c.sub(a).mulScalar(v));
            },
            1 => switch (gen.index(3)) {
                0 => a.add(b.sub(a).mulScalar(gen.plain(0, 1))),
                1 => b.add(c.sub(b).mulScalar(if (gen.oneIn(3)) 0.5 else gen.plain(0, 1))),
                else => c.add(a.sub(c).mulScalar(gen.plain(0, 1))),
            },
            2 => face[gen.index(3)],
            3 => face[gen.index(3)].add(gen.plainVec(-1.0e-3, 1.0e-3)),
            4 => vertices[gen.index(16)],
            else => gen.plainVec(-5, 5),
        };

        // Penetration axis: against the face normal (a face contact), slightly off, random
        const normal = b.sub(a).cross(c.sub(a));
        const axis = switch (gen.index(5)) {
            0 => normal.negate().mulScalar(gen.plain(0.1, 3)),
            1 => normal.negate().normalizedOr(Vec3.axisY()).add(gen.plainVec(-0.03, 0.03)),
            2 => normal.negate().normalizedOr(Vec3.axisY()).add(gen.plainVec(-0.3, 0.3)),
            3 => Vec3.zero(),
            else => gen.unitDirection().mulScalar(gen.plain(0.1, 2)),
        };
        h.axis = arr3(axis);
        h.point2 = arr3(point2);
        h.point1 = arr3(point2.add(gen.plainVec(-0.1, 0.1)));
        h.depth = switch (gen.index(4)) {
            0 => @as(f32, @floatFromInt(gen.index(4))) * 0.05, // Equal depths
            else => gen.plain(-0.2, 0.5),
        };
        h.id1 = @intCast(gen.index(3));
        h.id2 = @intCast(tri_index);
        h.body_id = gen.next() & 0x7fffff;
        h.face1_count = @intCast(gen.index(5));
        for (0..h.face1_count) |k| h.face1[3 * k ..][0..3].* = arr3(gen.plainVec(-5, 5));
    }
    return input;
}

fn genShapePair(gen: *Gen) ShapePairInput {
    var input = std.mem.zeroes(ShapePairInput);
    input.shape1 = gen.shape();
    input.shape2 = gen.shape();
    const scale1 = gen.scale(input.shape1);
    const scale2 = gen.scale(input.shape2);
    input.scale1 = arr3(scale1);
    input.scale2 = arr3(scale2);
    const reach = extentOf(input.shape1) * scale1.abs().reduceMax() + extentOf(input.shape2) * scale2.abs().reduceMax();
    const transform1 = gen.transform(5);
    var relative = gen.transform(1.2 * reach);
    if (gen.oneIn(10)) relative = Mat44.identity();
    input.transform1 = arr16(transform1);
    input.transform2 = arr16(transform1.mul(relative));
    input.creator1 = gen.creator();
    input.creator2 = gen.creator();
    input.mode = @intCast(gen.index(3));
    input.active_edge_mode = @intFromBool(input.mode == 0 or gen.oneIn(2));
    input.collect_faces = @intFromBool(input.mode == 0 or gen.oneIn(2));
    input.max_separation_distance = if (gen.oneIn(3)) gen.plain(0, 1) else 0.0;
    input.vertex_tolerance_sq = gen.vertexToleranceSq();
    input.collector = @intCast(gen.index(2));
    input.early_out = if (input.collector == 0 and gen.oneIn(4)) gen.plain(-2, 2) else math.flt_max; // ClosestHit would assert on worse leaf hits
    input.body_id = gen.next() & 0x7fffff;
    return input;
}

fn genSoftBody(gen: *Gen) SoftBodyInput {
    var input = std.mem.zeroes(SoftBodyInput);
    const transform = gen.transform(3);
    const scale = gen.anyScale();
    input.transform = arr16(transform);
    input.scale = arr3(scale);
    input.triangle_thickness = if (gen.oneIn(2)) 0.1 else gen.plain(0, 1);
    input.num_vertices = 1 + @as(u32, @intCast(gen.index(max_soft_body_vertices)));
    input.colliding_shape_index = @intCast(gen.index(10));

    // Triangles in the unscaled local space: a grid or a soup
    const num_triangles: u32 = 1 + @as(u32, @intCast(gen.index(max_soft_body_triangles)));
    const grid_mode = gen.oneIn(2);
    const inv_scale = Vec3.one().div(scale);
    for (0..num_triangles) |t| {
        var v: [3]Vec3 = undefined;
        if (grid_mode) {
            const cx: f32 = @floatFromInt(t % 6);
            const cz: f32 = @floatFromInt((t / 2) / 6);
            const o = Vec3.init(cx * 0.5 - 1.5, 0, cz * 0.5 - 1.5);
            v = if (t % 2 == 0) .{ o, o.add(Vec3.init(0, 0, 0.5)), o.add(Vec3.init(0.5, 0, 0)) } else .{ o.add(Vec3.init(0.5, 0, 0)), o.add(Vec3.init(0, 0, 0.5)), o.add(Vec3.init(0.5, 0, 0.5)) };
        } else {
            v = .{ gen.plainVec(-2, 2), gen.plainVec(-2, 2), gen.plainVec(-2, 2) };
            if (gen.oneIn(10)) v[1] = v[0]; // Degenerate
        }
        for (0..3) |k| input.triangles[9 * t + 3 * k ..][0..3].* = arr3(v[k].mul(inv_scale));
    }

    for (0..input.num_vertices) |i| {
        const local = switch (gen.index(4)) {
            0 => gen.plainVec(-0.3, 0.3), // Close to the plane of the grid
            1 => Vec3.init(gen.plain(-2, 2), gen.plain(-0.2, 0.2), gen.plain(-2, 2)),
            2 => vec3At(&input.triangles, gen.index(3 * num_triangles)).mul(scale), // At a vertex
            else => gen.plainVec(-3, 3),
        };
        input.positions[3 * i ..][0..3].* = arr3(transform.mulVec3(local));
        input.penetrations[i] = if (gen.oneIn(2)) -math.flt_max else gen.plain(-1, 0.5);
        input.planes[4 * i ..][0..4].* = arr4(Vec4.init(gen.plain(-1, 1), gen.plain(-1, 1), gen.plain(-1, 1), gen.plain(-1, 1)));
        input.indices[i] = @as(c_int, @intCast(gen.index(5))) - 1;
        input.num_triangles[i] = switch (gen.index(5)) {
            0 => 0, // No triangle: FinishVertex does nothing
            1 => @intCast(gen.index(num_triangles + 1)),
            else => num_triangles,
        };
    }
    return input;
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

/// Number of random inputs of the triangle tests (each input is a sequence of up to 64 triangles)
const triangle_iterations = 20_000;

test "Triangles parity: CollideConvexVsTriangles / CollideSphereVsTriangles (also through InternalEdgeRemovingCollector)" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "collide triangles" };
    var num_hits: usize = 0;
    var num_triangles: usize = 0;
    for (0..triangle_iterations) |_| {
        const input = genCollide(&gen);
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_triangles_collide(&input, &jolt_output);
        const zolt_output = try zoltCollide(allocator, &input);
        num_hits += zolt_output.num_hits;
        num_triangles += input.num_triangles;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > num_triangles / 10 and num_hits < num_triangles); // Hits and misses
}

test "Triangles parity: CastConvexVsTriangles / CastSphereVsTriangles" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "cast triangles" };
    var num_hits: usize = 0;
    var num_triangles: usize = 0;
    for (0..triangle_iterations) |_| {
        const input = genCast(&gen);
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_triangles_cast(&input, &jolt_output);
        const zolt_output = try zoltCast(allocator, &input);
        num_hits += zolt_output.num_hits;
        num_triangles += input.num_triangles;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > num_triangles / 20 and num_hits < num_triangles); // Hits and misses
}

test "Triangles parity: ManifoldBetweenTwoFaces (and PruneContactPoints of its result)" {
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "manifold" };
    var num_clipped: usize = 0;
    for (0..fw.iterations) |_| {
        const input = genManifold(&gen);
        var jolt_output = std.mem.zeroes(ManifoldOutput);
        jolt.jolt_triangles_manifold(&input, &jolt_output);
        const zolt_output = zoltManifold(&input);
        if (zolt_output.count1 > input.num_existing + 1) num_clipped += 1;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_clipped > fw.iterations / 10); // The clipping path is exercised
}

test "Triangles parity: PruneContactPoints" {
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "prune contact points" };
    for (0..fw.iterations) |_| {
        var axis: P = undefined;
        var points1: [64 * 3]f32 = @splat(0);
        var points2: [64 * 3]f32 = @splat(0);
        const count = genPrune(&gen, &axis, &points1, &points2);
        var jolt_output = std.mem.zeroes(ManifoldOutput);
        jolt.jolt_triangles_prune(&axis, count, &points1, &points2, &jolt_output);
        const zolt_output = zoltPrune(axis, count, &points1, &points2);
        checker.check(.{ axis, count, points1, points2 }, zolt_output, jolt_output);
    }
    try checker.finish();
}

test "Triangles parity: InternalEdgeRemovingCollector on recorded hit sequences" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "internal edges" };
    var num_in: usize = 0;
    var num_out: usize = 0;
    for (0..triangle_iterations) |_| {
        const input = genRecorded(&gen);
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_triangles_internal_edges(&input, &jolt_output);
        const zolt_output = try zoltInternalEdges(allocator, &input);
        if (input.collector == 0) {
            num_in += input.num_hits;
            num_out += zolt_output.num_hits;
        }
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_out > num_in / 10 and num_out < num_in); // Hits are removed and kept
}

test "Triangles parity: InternalEdgeRemovingCollector::sCollideShapeVsShape and CollideShapeVsShapePerLeaf" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "shape pair" };
    var num_hits: usize = 0;
    const n = fw.iterations / 2;
    for (0..n) |_| {
        const input = genShapePair(&gen);
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_triangles_shape_pair(&input, &jolt_output);
        const zolt_output = try zoltShapePair(allocator, &input);
        num_hits += zolt_output.num_hits;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > n / 5 and num_hits < 4 * n / 5); // Hits and misses
}

test "Triangles parity: CollideSoftBodyVerticesVsTriangles" {
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "soft body vertices" };
    var num_updated: usize = 0;
    for (0..triangle_iterations) |_| {
        const input = genSoftBody(&gen);
        var jolt_output = std.mem.zeroes(SoftBodyOutput);
        jolt.jolt_triangles_soft_body(&input, &jolt_output);
        const zolt_output = zoltSoftBody(&input);
        for (0..input.num_vertices) |i| {
            if (zolt_output.indices[i] == input.colliding_shape_index and input.indices[i] != input.colliding_shape_index) num_updated += 1;
        }
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_updated > triangle_iterations); // Collisions are found
}
