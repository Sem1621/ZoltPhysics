//! Cross-shape parity sweep (Phase 4): the parity tests of each shape port collide and cast it against spheres and boxes,
//! this file covers every other combination. Both sides build the same catalogue of shapes from the same node
//! descriptions (`NodeDesc`): spheres, boxes, capsules, tapered capsules, cylinders, tapered cylinders (incl. a cone),
//! convex hulls (4 to 150 points), triangles (with and without convex radius), planes, empty shapes, ScaledShape /
//! RotatedTranslatedShape / OffsetCenterOfMassShape around convex shapes and around a mesh, static and mutable compounds
//! (nested, with a mesh, with only unrotated sub shapes so that non uniform scales are valid), meshes (a grid and a
//! closed box) and height fields (with no-collision samples and materials), with convex radius 0 and > 0, materials and
//! user data. Then, for every ordered pair of catalogue shapes and many random relative transforms (separated, touching
//! along an axis, overlapping, deep, rotated, uniformly and non uniformly scaled where IsValidScale allows, inside out):
//! - CollisionDispatch::sCollideShapeVsShape and InternalEdgeRemovingCollector::sCollideShapeVsShape with the AllHit,
//!   ClosestHit and AnyHit collectors and random CollideShapeSettings (max separation distance, tolerances, back face
//!   mode, active edge mode and movement direction, collect faces), compared hit by hit in Jolt's (deterministic) order;
//! - CollisionDispatch::sCastShapeVsShapeWorldSpace with zero and non zero directions and random ShapeCastSettings
//!   (back face modes, shrunken shape and convex radius, deepest point, extra convex radius, collect faces, active edges);
//! every hit field is compared (contact points, penetration axis and depth, sub shape IDs, body ID, faces, fraction, back
//! face flag) as well as the final early out fraction and a hash of every ShapeFilter call (PairwiseFilter rejects a
//! pseudo random subset of the sub shape pairs).
//! Pairs that Jolt does not support (mesh, height field and plane against each other, also inside compounds and
//! decorators) assert in Jolt's debug build and in Zolt's safe builds: they are compared through the dispatch tables (the
//! unsupported / reversed entries must be the same for every pair of sub types) and, in ReleaseFast, through the queries
//! (no hits on both sides). The same holds for the AnyHit collector behind InternalEdgeRemovingCollector (see
//! CollisionArchitecture.md section 9). Every catalogue shape also goes through the TransformedShape queries with random
//! world transforms and scales: both CastRay overloads (with the material, user data, surface normal and supporting face
//! of every hit), CollidePoint, CollectTransformedShapes, GetTrianglesStart / Next, GetSupportingFace and
//! GetWorldSpaceBounds. C ABI wrappers: ZoltParity/Physics/PairwiseReference.cpp.
//!
//! Both sides write their results into a stream of u32 (see `Stream`, the layout of every function is the same as in
//! PairwiseReference.cpp) and the streams must be identical. The number of queries per pair depends on the cost of the
//! pair, so that the sweep stays well below ~3 minutes in Debug.

const std = @import("std");
const builtin = @import("builtin");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Allocator = std.mem.Allocator;
const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const BodyID = zolt.BodyID;
const BoxShapeSettings = zolt.BoxShapeSettings;
const CapsuleShapeSettings = zolt.CapsuleShapeSettings;
const CastRayCollector = zolt.CastRayCollector;
const CastShapeCollector = zolt.CastShapeCollector;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const CollidePointCollector = zolt.CollidePointCollector;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollisionDispatch = zolt.CollisionDispatch;
const Color = zolt.Color;
const CompoundShapeSettings = zolt.CompoundShapeSettings;
const ConvexHullShapeSettings = zolt.ConvexHullShapeSettings;
const CylinderShapeSettings = zolt.CylinderShapeSettings;
const EmptyShapeSettings = zolt.EmptyShapeSettings;
const Float3 = zolt.Float3;
const HeightFieldShapeConstants = zolt.HeightFieldShapeConstants;
const HeightFieldShapeSettings = zolt.HeightFieldShapeSettings;
const IndexedTriangle = zolt.IndexedTriangle;
const InternalEdgeRemovingCollector = zolt.InternalEdgeRemovingCollector;
const Mat44 = zolt.Mat44;
const MeshShapeSettings = zolt.MeshShapeSettings;
const MutableCompoundShapeSettings = zolt.MutableCompoundShapeSettings;
const OffsetCenterOfMassShapeSettings = zolt.OffsetCenterOfMassShapeSettings;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsMaterialSimple = zolt.PhysicsMaterialSimple;
const Plane = zolt.Plane;
const PlaneShapeSettings = zolt.PlaneShapeSettings;
const Quat = zolt.Quat;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const Real = zolt.Real;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RegisterTypes = zolt.RegisterTypes;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const RRayCast = zolt.RRayCast;
const RVec3 = zolt.RVec3;
const ScaledShapeSettings = zolt.ScaledShapeSettings;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastResult = zolt.ShapeCastResult;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeFilter = zolt.ShapeFilter;
const ShapeResult = zolt.ShapeResult;
const ShapeSettings = zolt.ShapeSettings;
const ShapeSubType = zolt.ShapeSubType;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TaperedCapsuleShapeSettings = zolt.TaperedCapsuleShapeSettings;
const TaperedCylinderShapeSettings = zolt.TaperedCylinderShapeSettings;
const TransformedShape = zolt.TransformedShape;
const TransformedShapeCollector = zolt.TransformedShapeCollector;
const TriangleShapeSettings = zolt.TriangleShapeSettings;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

/// The C++ reference functions, see PairwiseReference.cpp
const jolt = struct {
    extern fn pw_catalogue_create(input: *const CatalogueInput, out: [*]u32, capacity: u32, out_size: *u32) ?*anyopaque;
    extern fn pw_catalogue_destroy(catalogue: *anyopaque) void;
    extern fn pw_dispatch(type1: u32, type2: u32, out_collide: *c_int, out_cast: *c_int) void;
    extern fn pw_collide(catalogue: *anyopaque, input: *const CollideInput, out: [*]u32, capacity: u32) u32;
    extern fn pw_cast(catalogue: *anyopaque, input: *const CastInput, out: [*]u32, capacity: u32) u32;
    extern fn pw_transformed_shape(catalogue: *anyopaque, input: *const TransformedShapeInput, out: [*]u32, capacity: u32) u32;
};

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// Number of parity materials, must match cNumMaterials in PairwiseReference.cpp
const num_materials = 8;

/// Capacity of the C++ output stream (in u32)
const stream_capacity = 1 << 20;

/// Print the number of queries and hits of the sweep (passing tests must not print, only for tuning)
const print_statistics = false;

/// Safe builds assert where Jolt's debug build asserts (unsupported shape pairs, AnyHit behind
/// InternalEdgeRemovingCollector): those inputs only run without asserts (ReleaseFast)
const asserts = zolt.Core.enable_asserts;

// Section markers in the streams, must match PairwiseReference.cpp
const marker_properties: u32 = 0xC0000001;
const marker_collide: u32 = 0xC0000002;
const marker_cast: u32 = 0xC0000003;
const marker_hit: u32 = 0xC0000004;
const marker_ray: u32 = 0xC0000005;
const marker_ray_collector: u32 = 0xC0000006;
const marker_point: u32 = 0xC0000007;
const marker_transformed_shapes: u32 = 0xC0000008;
const marker_triangles: u32 = 0xC0000009;
const marker_face: u32 = 0xC000000A;
const marker_bounds: u32 = 0xC000000B;
const marker_filter: u32 = 0xC000000C;

/// Node kinds, must match the enum in PairwiseReference.cpp
const NodeKind = enum(u32) {
    /// f[0]: radius
    sphere,
    /// f[0..3]: half extent, convex_radius
    box,
    /// f[0]: half height of the cylinder, f[1]: radius
    capsule,
    /// f[0]: half height, f[1]: top radius, f[2]: bottom radius
    tapered_capsule,
    /// f[0]: half height, f[1]: radius, convex_radius
    cylinder,
    /// f[0]: half height, f[1]: top radius, f[2]: bottom radius, convex_radius
    tapered_cylinder,
    /// floats[first..][0 .. 3 * count]: the points, convex_radius: the max convex radius
    convex_hull,
    /// f[0..9]: the vertices, convex_radius
    triangle,
    /// f[0..4]: normal and constant, f[4]: half extent
    plane,
    /// f[0..3]: center of mass
    empty,
    /// child, f[0..3]: scale
    scaled,
    /// child, f[0..3]: position, f[3..7]: rotation
    rotated_translated,
    /// child, f[0..3]: offset
    offset_center_of_mass,
    /// subs[first..][0..count]
    static_compound,
    /// subs[first..][0..count]
    mutable_compound,
    /// floats[first..][0 .. 3 * count]: vertices, triangles[first2..][0..count2], num_materials, param0: max triangles
    /// per leaf, param1: per triangle user data, f[0]: active edge cos threshold angle
    mesh,
    /// floats[first..][0 .. count * count]: samples (count = sample count), bytes[first2..][0..count2]: material indices,
    /// num_materials, f[0..3]: offset, f[3..6]: scale, f[6]: active edge cos threshold angle, param0: block size,
    /// param1: bits per sample
    height_field,
};

/// Must match NodeDesc in PairwiseReference.cpp
const NodeDesc = extern struct {
    kind: NodeKind,
    /// 0: no material, i + 1: parity material i
    material: u32 = 0,
    f: [16]f32 = @splat(0),
    convex_radius: f32 = 0,
    child: u32 = 0,
    first: u32 = 0,
    count: u32 = 0,
    first2: u32 = 0,
    count2: u32 = 0,
    num_materials: u32 = 0,
    param0: u32 = 0,
    param1: u32 = 0,
    user_data: u32 = 0,
};

/// Must match SubDesc in PairwiseReference.cpp
const SubDesc = extern struct {
    node: u32,
    user_data: u32,
    position: P,
    rotation: [4]f32,
};

/// Must match CatalogueInput in PairwiseReference.cpp
const CatalogueInput = extern struct {
    nodes: [*]const NodeDesc,
    subs: [*]const SubDesc,
    floats: [*]const f32,
    triangles: [*]const IndexedTriangle,
    bytes: [*]const u8,
    num_nodes: u32,
};

/// Must match CollideInput in PairwiseReference.cpp
const CollideInput = extern struct {
    shape1: u32,
    shape2: u32,
    scale1: P,
    scale2: P,
    transform1: [16]f32,
    transform2: [16]f32,
    creator1: [2]u32,
    creator2: [2]u32,
    max_separation_distance: f32,
    collision_tolerance: f32,
    penetration_tolerance: f32,
    /// 1: CollideWithBackFaces
    back_face_mode: c_int,
    /// 1: CollideWithAll
    active_edge_mode: c_int,
    /// 1: CollectFaces
    collect_faces: c_int,
    active_edge_movement_direction: P,
    /// 0: AllHit, 1: ClosestHit, 2: AnyHit
    collector: c_int,
    /// Early out fraction of the collector (when < FLT_MAX)
    early_out: f32,
    /// Body ID of the collector context
    body_id: u32,
    /// The filter (0: rejects nothing)
    reject_modulus: u32,
    /// Use InternalEdgeRemovingCollector::sCollideShapeVsShape
    internal_edge_removal: c_int,
    vertex_tolerance_sq: f32,
};

/// Must match CastInput in PairwiseReference.cpp
const CastInput = extern struct {
    /// The cast shape
    shape1: u32,
    /// The shape cast against
    shape2: u32,
    scale1: P,
    start: [16]f32,
    direction: P,
    scale2: P,
    transform2: [16]f32,
    creator1: [2]u32,
    creator2: [2]u32,
    collision_tolerance: f32,
    penetration_tolerance: f32,
    extra_convex_radius: f32,
    back_face_mode_triangles: c_int,
    back_face_mode_convex: c_int,
    use_shrunken_shape: c_int,
    return_deepest_point: c_int,
    collect_faces: c_int,
    /// 1: CollideWithAll
    active_edge_mode: c_int,
    active_edge_movement_direction: P,
    /// 0: AllHit, 1: ClosestHit, 2: AnyHit
    collector: c_int,
    /// Early out fraction of the collector (when < 1 + FLT_EPSILON)
    early_out: f32,
    body_id: u32,
    reject_modulus: u32,
};

/// Must match TransformedShapeInput in PairwiseReference.cpp
const TransformedShapeInput = extern struct {
    shape: u32,
    body_id: u32,
    creator: [2]u32,
    /// Center of mass position
    position: [3]f64,
    rotation: [4]f32,
    scale: P,
    ray_origin: [3]f64,
    ray_direction: P,
    /// Initial fraction of the single hit CastRay
    ray_fraction: f32,
    back_face_mode_triangles: c_int,
    back_face_mode_convex: c_int,
    treat_convex_as_solid: c_int,
    /// 0: AllHit, 1: ClosestHit, 2: AnyHit
    ray_collector: c_int,
    /// Early out fraction of the ray collector (when < 1 + FLT_EPSILON)
    ray_early_out: f32,
    /// CollidePoint
    point: [3]f64,
    /// CollectTransformedShapes / GetTrianglesStart (world space)
    box: [6]f32,
    /// GetTrianglesStart / GetSupportingFace
    base_offset: [3]f64,
    max_triangles_requested: c_int,
    /// GetTrianglesNext with materials
    materials: c_int,
    /// GetSupportingFace (world space)
    face_direction: P,
    /// GetSupportingFace with the ID of the root (only for shapes without sub shapes)
    face_of_root: c_int,
    /// Call GetTrianglesStart / Next (Jolt asserts for compound and decorated shapes)
    triangles: c_int,
    reject_modulus: u32,
};

// ---------------------------------------------------------------------------------------------------------------------
// Conversions

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

fn mat44(a: [16]f32) Mat44 {
    return Mat44.init(vec4(a[0..4].*), vec4(a[4..8].*), vec4(a[8..12].*), vec4(a[12..16].*));
}

fn arr16(m: Mat44) [16]f32 {
    return arr4(m.getColumn4(0)) ++ arr4(m.getColumn4(1)) ++ arr4(m.getColumn4(2)) ++ arr4(m.getColumn4(3));
}

fn quat(a: [4]f32) Quat {
    return Quat.init(a[0], a[1], a[2], a[3]);
}

fn rvec3(a: [3]f64) RVec3 {
    return RVec3.init(@floatCast(a[0]), @floatCast(a[1]), @floatCast(a[2]));
}

fn aabox(a: [6]f32) AABox {
    return AABox.init(vec3(a[0..3].*), vec3(a[3..6].*));
}

fn makeCreator(c: [2]u32) SubShapeIDCreator {
    return SubShapeIDCreator.pushID(.{}, c[0], c[1]);
}

// ---------------------------------------------------------------------------------------------------------------------
// The output stream (the same layout as OutStream in PairwiseReference.cpp)

const Stream = struct {
    allocator: Allocator,
    values: std.ArrayList(u32) = .empty,

    fn deinit(s: *Stream) void {
        s.values.deinit(s.allocator);
    }

    fn u(s: *Stream, v: u32) void {
        s.values.append(s.allocator, v) catch @panic("out of memory");
    }

    fn b(s: *Stream, v: bool) void {
        s.u(@intFromBool(v));
    }

    fn f(s: *Stream, v: f32) void {
        s.u(@bitCast(v));
    }

    fn d(s: *Stream, v: f64) void {
        const bits: u64 = @bitCast(v);
        s.u(@truncate(bits));
        s.u(@truncate(bits >> 32));
    }

    fn v3(s: *Stream, v: Vec3) void {
        s.f(v.getX());
        s.f(v.getY());
        s.f(v.getZ());
    }

    fn f3(s: *Stream, v: Float3) void {
        s.f(v.x);
        s.f(v.y);
        s.f(v.z);
    }

    fn r3(s: *Stream, v: RVec3) void {
        s.d(v.getX());
        s.d(v.getY());
        s.d(v.getZ());
    }

    fn q(s: *Stream, v: Quat) void {
        s.f(v.getX());
        s.f(v.getY());
        s.f(v.getZ());
        s.f(v.getW());
    }

    fn box(s: *Stream, v: AABox) void {
        s.v3(v.min);
        s.v3(v.max);
    }

    fn face(s: *Stream, v: *const Shape.SupportingFace) void {
        s.u(v.len);
        for (v.constSlice()) |p| s.v3(p);
    }
};

/// The parity materials ("Pairwise i" with Color.getDistinctColor(i), the same as GetMaterials() in C++)
const Materials = struct {
    refs: [num_materials]RefConst(PhysicsMaterial) = undefined,
    ptrs: [num_materials]*const PhysicsMaterial = undefined,

    fn init(self: *Materials, allocator: Allocator) !void {
        for (0..num_materials) |i| {
            var name_buffer: [32]u8 = undefined;
            const name = try std.fmt.bufPrint(&name_buffer, "Pairwise {d}", .{i});
            const material = try PhysicsMaterialSimple.create(allocator, name, Color.getDistinctColor(@intCast(i)));
            self.ptrs[i] = material.material();
            self.refs[i] = .init(self.ptrs[i]);
        }
    }

    fn deinit(self: *Materials) void {
        for (&self.refs) |*r| r.deinit();
    }

    /// WriteMaterial in C++
    fn write(self: *const Materials, s: *Stream, material: *const PhysicsMaterial) void {
        if (material == PhysicsMaterial.default) {
            s.u(0xffffffff);
            return;
        }
        for (self.ptrs, 0..) |p, i|
            if (p == material) {
                s.u(@intCast(i));
                return;
            };
        s.u(0xfffffffe);
    }
};

fn writeCollideHit(s: *Stream, r: *const CollideShapeResult) void {
    s.u(marker_hit);
    s.v3(r.contact_point_on1);
    s.v3(r.contact_point_on2);
    s.v3(r.penetration_axis);
    s.f(r.penetration_depth);
    s.u(r.sub_shape_id1.getValue());
    s.u(r.sub_shape_id2.getValue());
    s.u(r.body_id2.getIndexAndSequenceNumber());
    s.face(&r.shape1_face);
    s.face(&r.shape2_face);
}

fn writeCastHit(s: *Stream, r: *const ShapeCastResult) void {
    s.f(r.fraction);
    s.b(r.is_back_face_hit);
    writeCollideHit(s, &r.base);
}

// ---------------------------------------------------------------------------------------------------------------------
// The catalogue on the Zolt side (CreateNode in C++)

fn createNode(allocator: Allocator, materials: *const Materials, in: *const CatalogueInput, shapes: []const RefConst(Shape), index: usize) !ShapeResult {
    const n = &in.nodes[index];
    const f = &n.f;
    const material: ?*const PhysicsMaterial = if (n.material == 0) null else materials.ptrs[n.material - 1];
    var material_list: [64]*const PhysicsMaterial = undefined;
    for (0..n.num_materials) |i| material_list[i] = materials.ptrs[i % num_materials];
    const list = material_list[0..n.num_materials];
    const v3 = struct {
        fn at(a: []const f32) Vec3 {
            return Vec3.init(a[0], a[1], a[2]);
        }
    }.at;

    var settings: Ref(ShapeSettings) = .init(switch (n.kind) {
        .sphere => (try SphereShapeSettings.create(allocator, f[0], .{ .material = material })).asShapeSettings(),
        .box => (try BoxShapeSettings.create(allocator, v3(f[0..3]), .{ .convex_radius = n.convex_radius, .material = material })).asShapeSettings(),
        .capsule => (try CapsuleShapeSettings.create(allocator, f[0], f[1], .{ .material = material })).asShapeSettings(),
        .tapered_capsule => (try TaperedCapsuleShapeSettings.create(allocator, f[0], f[1], f[2], .{ .material = material })).asShapeSettings(),
        .cylinder => (try CylinderShapeSettings.create(allocator, f[0], f[1], .{ .convex_radius = n.convex_radius, .material = material })).asShapeSettings(),
        .tapered_cylinder => (try TaperedCylinderShapeSettings.create(allocator, f[0], f[1], f[2], .{ .convex_radius = n.convex_radius, .material = material })).asShapeSettings(),
        .convex_hull => blk: {
            var points: [256]Vec3 = undefined;
            for (0..n.count) |i| points[i] = v3(in.floats[n.first + 3 * i ..][0..3]);
            break :blk (try ConvexHullShapeSettings.create(allocator, points[0..n.count], .{ .max_convex_radius = n.convex_radius, .material = material })).asShapeSettings();
        },
        .triangle => (try TriangleShapeSettings.create(allocator, v3(f[0..3]), v3(f[3..6]), v3(f[6..9]), .{ .convex_radius = n.convex_radius, .material = material })).asShapeSettings(),
        .plane => (try PlaneShapeSettings.create(allocator, Plane.init(v3(f[0..3]), f[3]), .{ .material = material, .half_extent = f[4] })).asShapeSettings(),
        .empty => (try EmptyShapeSettings.create(allocator, v3(f[0..3]))).asShapeSettings(),
        .scaled => (try ScaledShapeSettings.createPtr(allocator, shapes[n.child].get(), v3(f[0..3]))).asShapeSettings(),
        .rotated_translated => (try RotatedTranslatedShapeSettings.createPtr(allocator, v3(f[0..3]), quat(f[3..7].*), shapes[n.child].get())).asShapeSettings(),
        .offset_center_of_mass => (try OffsetCenterOfMassShapeSettings.createPtr(allocator, v3(f[0..3]), shapes[n.child].get())).asShapeSettings(),
        .static_compound, .mutable_compound => blk: {
            const compound: *ShapeSettings = if (n.kind == .static_compound)
                (try StaticCompoundShapeSettings.create(allocator)).asShapeSettings()
            else
                (try MutableCompoundShapeSettings.create(allocator)).asShapeSettings();
            const base: *CompoundShapeSettings = @fieldParentPtr("base", compound);
            for (in.subs[n.first..][0..n.count]) |sub|
                try base.addShapePtr(vec3(sub.position), quat(sub.rotation), shapes[sub.node].get(), .{ .user_data = sub.user_data });
            break :blk compound;
        },
        .mesh => blk: {
            const vertices: [*]const Float3 = @ptrCast(in.floats + n.first);
            const mesh = try MeshShapeSettings.createIndexed(allocator, vertices[0..n.count], in.triangles[n.first2..][0..n.count2], .{ .materials = list });
            mesh.max_triangles_per_leaf = n.param0;
            mesh.active_edge_cos_threshold_angle = f[0];
            mesh.per_triangle_user_data = n.param1 != 0;
            break :blk mesh.asShapeSettings();
        },
        .height_field => blk: {
            const hf = try HeightFieldShapeSettings.create(allocator, in.floats[n.first..][0 .. n.count * n.count], v3(f[0..3]), v3(f[3..6]), n.count, .{ .material_indices = if (n.count2 > 0) in.bytes[n.first2..][0..n.count2] else null, .materials = list });
            hf.block_size = n.param0;
            hf.bits_per_sample = n.param1;
            hf.active_edge_cos_threshold_angle = f[6];
            break :blk hf.asShapeSettings();
        },
    });
    defer settings.deinit();
    settings.get().?.user_data = n.user_data;
    return settings.get().?.createShape(allocator);
}

/// The catalogue shapes of Zolt (one per node) and their properties (jolt: pw_catalogue_create)
fn zoltCatalogue(allocator: Allocator, materials: *const Materials, in: *const CatalogueInput, shapes: *std.ArrayList(RefConst(Shape)), s: *Stream) !bool {
    var valid = true;
    for (0..in.num_nodes) |i| {
        var result = try createNode(allocator, materials, in, shapes.items, i);
        defer result.deinit();
        s.u(marker_properties);
        if (result.hasError()) {
            s.u(0);
            valid = false;
            try shapes.append(allocator, .empty);
            continue;
        }
        s.u(1);
        const shape: *const Shape = result.getPtr().?;
        try shapes.append(allocator, .init(shape));
        s.u(@intFromEnum(shape.getType()));
        s.u(@intFromEnum(shape.getSubType()));
        s.box(shape.getLocalBounds());
        s.v3(shape.getCenterOfMass());
        s.f(shape.getInnerRadius());
        s.u(shape.getSubShapeIDBitsRecursive());
        s.u(@truncate(shape.getUserData()));
        s.u(shape.getStats().num_triangles);
    }
    return valid;
}

// ---------------------------------------------------------------------------------------------------------------------
// The filter, must match PairwiseFilter in PairwiseReference.cpp

const FilterLog = struct {
    calls: u32 = 0,
    hash: u32 = 0x811c9dc5,

    fn add(self: *FilterLog, value: u32) void {
        self.hash = (self.hash ^ value) *% 0x01000193;
        self.calls += 1;
    }

    fn write(self: *const FilterLog, s: *Stream) void {
        s.u(marker_filter);
        s.u(self.calls);
        s.u(self.hash);
    }
};

const PairwiseFilter = struct {
    pub const overrides = .{ .shouldCollide, .shouldCollidePair };

    base: ShapeFilter = .init(@This()),
    reject_modulus: u32,
    /// The calls are logged behind a pointer (Rule M: the filter is const in the queries)
    log: *FilterLog,

    fn reject(self: *const PairwiseFilter, value: u32) bool {
        return self.reject_modulus != 0 and (value >> 16) % self.reject_modulus == 0;
    }

    pub fn shouldCollide(self: *const PairwiseFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        self.log.add(1);
        self.log.add(@intFromEnum(shape2.getSubType()));
        self.log.add(sub_shape_id_of_shape2.getValue());
        self.log.add(self.base.body_id2.getIndexAndSequenceNumber());
        return !self.reject(sub_shape_id_of_shape2.getValue() *% 0x9e3779b1);
    }

    pub fn shouldCollidePair(self: *const PairwiseFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        self.log.add(2);
        self.log.add(@intFromEnum(shape1.getSubType()));
        self.log.add(sub_shape_id_of_shape1.getValue());
        self.log.add(@intFromEnum(shape2.getSubType()));
        self.log.add(sub_shape_id_of_shape2.getValue());
        self.log.add(self.base.body_id2.getIndexAndSequenceNumber());
        return !self.reject(sub_shape_id_of_shape1.getValue() *% 0x9e3779b1 +% sub_shape_id_of_shape2.getValue() *% 0x85ebca6b);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of the queries

/// RunWithCollector in C++: run `query.run(collector_base)` with the collector of `kind` (0: AllHit, 1: ClosestHit,
/// 2: AnyHit) and write the hits (`query.write(stream, hit)`) and the final early out fraction
fn runWithCollector(comptime CollectorType: type, allocator: Allocator, s: *Stream, kind: c_int, early_out: f32, context: ?*const TransformedShape, query: anytype) !void {
    switch (kind) {
        0 => {
            var collector = AllHitCollisionCollector(CollectorType).init(allocator);
            defer collector.deinit();
            if (context) |c| collector.base.setContext(c);
            if (early_out < collector.base.getEarlyOutFraction())
                collector.base.updateEarlyOutFraction(early_out);
            try query.run(&collector.base);
            try collector.checkError();
            s.u(@intCast(collector.hits.items.len));
            for (collector.hits.items) |*hit| query.write(s, hit);
            s.f(collector.base.getEarlyOutFraction());
        },
        1 => {
            var collector = ClosestHitCollisionCollector(CollectorType).init();
            defer collector.deinit();
            if (context) |c| collector.base.setContext(c);
            if (early_out < collector.base.getEarlyOutFraction())
                collector.base.updateEarlyOutFraction(early_out);
            try query.run(&collector.base);
            s.b(collector.hadHit());
            if (collector.hadHit()) query.write(s, &collector.hit);
            s.f(collector.base.getEarlyOutFraction());
        },
        else => {
            var collector = AnyHitCollisionCollector(CollectorType).init();
            defer collector.deinit();
            if (context) |c| collector.base.setContext(c);
            if (early_out < collector.base.getEarlyOutFraction())
                collector.base.updateEarlyOutFraction(early_out);
            try query.run(&collector.base);
            s.b(collector.hadHit());
            if (collector.hadHit()) query.write(s, &collector.hit);
            s.f(collector.base.getEarlyOutFraction());
        },
    }
}

fn collideSettings(in: *const CollideInput) CollideShapeSettings {
    var settings: CollideShapeSettings = .{};
    settings.max_separation_distance = in.max_separation_distance;
    settings.collision_tolerance = in.collision_tolerance;
    settings.penetration_tolerance = in.penetration_tolerance;
    settings.back_face_mode = if (in.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.active_edge_mode = if (in.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.collect_faces_mode = if (in.collect_faces != 0) .collect_faces else .no_faces;
    settings.active_edge_movement_direction = vec3(in.active_edge_movement_direction);
    settings.internal_edge_removal_vertex_tolerance_sq = in.vertex_tolerance_sq;
    return settings;
}

fn castSettings(in: *const CastInput) ShapeCastSettings {
    var settings: ShapeCastSettings = .{};
    settings.collision_tolerance = in.collision_tolerance;
    settings.penetration_tolerance = in.penetration_tolerance;
    settings.extra_convex_radius = in.extra_convex_radius;
    settings.back_face_mode_triangles = if (in.back_face_mode_triangles != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.back_face_mode_convex = if (in.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.use_shrunken_shape_and_convex_radius = in.use_shrunken_shape != 0;
    settings.return_deepest_point = in.return_deepest_point != 0;
    settings.collect_faces_mode = if (in.collect_faces != 0) .collect_faces else .no_faces;
    settings.active_edge_mode = if (in.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.active_edge_movement_direction = vec3(in.active_edge_movement_direction);
    return settings;
}

/// pw_collide
fn zoltCollide(allocator: Allocator, shapes: []const RefConst(Shape), in: *const CollideInput, s: *Stream) !void {
    const shape1 = shapes[in.shape1].get().?;
    const shape2 = shapes[in.shape2].get().?;
    const settings = collideSettings(in);
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(in.body_id), .{});
    var log: FilterLog = .{};
    const filter: PairwiseFilter = .{ .reject_modulus = in.reject_modulus, .log = &log };

    s.u(marker_collide);
    const Query = struct {
        allocator: Allocator,
        in: *const CollideInput,
        shape1: *const Shape,
        shape2: *const Shape,
        settings: *const CollideShapeSettings,
        filter: *const ShapeFilter,

        fn run(c: @This(), collector: *CollideShapeCollector) !void {
            const i = c.in;
            if (i.internal_edge_removal != 0)
                try InternalEdgeRemovingCollector.collideShapeVsShape(c.allocator, c.shape1, c.shape2, vec3(i.scale1), vec3(i.scale2), mat44(i.transform1), mat44(i.transform2), makeCreator(i.creator1), makeCreator(i.creator2), c.settings, collector, c.filter)
            else
                CollisionDispatch.collideShapeVsShape(c.shape1, c.shape2, vec3(i.scale1), vec3(i.scale2), mat44(i.transform1), mat44(i.transform2), makeCreator(i.creator1), makeCreator(i.creator2), c.settings, collector, c.filter);
        }

        fn write(_: @This(), st: *Stream, hit: *const CollideShapeResult) void {
            writeCollideHit(st, hit);
        }
    };
    try runWithCollector(CollideShapeCollector, allocator, s, in.collector, in.early_out, &context, Query{ .allocator = allocator, .in = in, .shape1 = shape1, .shape2 = shape2, .settings = &settings, .filter = &filter.base });
    log.write(s);
}

/// pw_cast
fn zoltCast(allocator: Allocator, shapes: []const RefConst(Shape), in: *const CastInput, s: *Stream) !void {
    const shape1 = shapes[in.shape1].get().?;
    const shape2 = shapes[in.shape2].get().?;
    const settings = castSettings(in);
    const shape_cast = ShapeCast.init(shape1, vec3(in.scale1), mat44(in.start), vec3(in.direction));
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(in.body_id), .{});
    var log: FilterLog = .{};
    const filter: PairwiseFilter = .{ .reject_modulus = in.reject_modulus, .log = &log };

    s.u(marker_cast);
    s.box(shape_cast.shape_world_bounds);
    const Query = struct {
        in: *const CastInput,
        shape_cast: *const ShapeCast,
        shape2: *const Shape,
        settings: *const ShapeCastSettings,
        filter: *const ShapeFilter,

        fn run(c: @This(), collector: *CastShapeCollector) !void {
            const i = c.in;
            CollisionDispatch.castShapeVsShapeWorldSpace(c.shape_cast, c.settings, c.shape2, vec3(i.scale2), c.filter, mat44(i.transform2), makeCreator(i.creator1), makeCreator(i.creator2), collector);
        }

        fn write(_: @This(), st: *Stream, hit: *const ShapeCastResult) void {
            writeCastHit(st, hit);
        }
    };
    try runWithCollector(CastShapeCollector, allocator, s, in.collector, in.early_out, &context, Query{ .in = in, .shape_cast = &shape_cast, .shape2 = shape2, .settings = &settings, .filter = &filter.base });
    log.write(s);
}

/// pw_transformed_shape
fn zoltTransformedShape(allocator: Allocator, materials: *const Materials, shapes: []const RefConst(Shape), in: *const TransformedShapeInput, s: *Stream) !void {
    const shape = shapes[in.shape].get().?;
    var ts = TransformedShape.init(rvec3(in.position), quat(in.rotation), shape, .init(in.body_id), .{ .sub_shape_id_creator = makeCreator(in.creator) });
    defer ts.deinit();
    ts.setShapeScale(vec3(in.scale));
    const base_offset = rvec3(in.base_offset);

    // GetWorldSpaceBounds, IsValidScale, MakeScaleValid
    s.u(marker_bounds);
    s.box(ts.getWorldSpaceBounds());
    s.b(shape.isValidScale(vec3(in.scale)));
    s.v3(shape.makeScaleValid(vec3(in.scale)));

    const SubShapeQueries = struct {
        ts: *const TransformedShape,
        materials: *const Materials,
        in: *const TransformedShapeInput,
        base_offset: RVec3,

        /// The sub shape queries of a hit: material, user data, surface normal, supporting face
        fn run(c: @This(), st: *Stream, id: SubShapeID, position: RVec3) void {
            c.materials.write(st, c.ts.getMaterial(id));
            const user_data = c.ts.getSubShapeUserData(id);
            st.u(@truncate(user_data));
            st.u(@truncate(user_data >> 32));
            st.v3(c.ts.getWorldSpaceSurfaceNormal(id, position));
            var face: Shape.SupportingFace = .empty;
            c.ts.getSupportingFace(id, vec3(c.in.face_direction), c.base_offset, &face);
            st.u(marker_face);
            st.face(&face);
        }
    };
    const sub_shape_queries: SubShapeQueries = .{ .ts = &ts, .materials = materials, .in = in, .base_offset = base_offset };

    // CastRay (single hit)
    const ray = RRayCast.init(rvec3(in.ray_origin), vec3(in.ray_direction));
    {
        var hit: RayCastResult = .{};
        hit.fraction = in.ray_fraction;
        s.u(marker_ray);
        const had_hit = ts.castRay(ray, &hit);
        s.b(had_hit);
        s.f(hit.fraction);
        s.u(hit.sub_shape_id2.getValue());
        s.u(hit.body_id.getIndexAndSequenceNumber());
        if (had_hit)
            sub_shape_queries.run(s, hit.sub_shape_id2, ray.getPointOnRay(hit.fraction));
    }

    // CastRay (collector)
    {
        var settings: RayCastSettings = .{};
        settings.back_face_mode_triangles = if (in.back_face_mode_triangles != 0) .collide_with_back_faces else .ignore_back_faces;
        settings.back_face_mode_convex = if (in.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
        settings.treat_convex_as_solid = in.treat_convex_as_solid != 0;
        var log: FilterLog = .{};
        var filter: PairwiseFilter = .{ .reject_modulus = in.reject_modulus, .log = &log };
        s.u(marker_ray_collector);
        const Query = struct {
            ts: *const TransformedShape,
            ray: RRayCast,
            settings: *const RayCastSettings,
            filter: *ShapeFilter,
            sub_shape_queries: *const SubShapeQueries,

            fn run(c: @This(), collector: *CastRayCollector) !void {
                c.ts.castRayCollector(c.ray, c.settings, collector, .{ .shape_filter = c.filter });
            }

            fn write(c: @This(), st: *Stream, hit: *const RayCastResult) void {
                st.f(hit.fraction);
                st.u(hit.sub_shape_id2.getValue());
                st.u(hit.body_id.getIndexAndSequenceNumber());
                c.sub_shape_queries.run(st, hit.sub_shape_id2, c.ray.getPointOnRay(hit.fraction));
            }
        };
        try runWithCollector(CastRayCollector, allocator, s, in.ray_collector, in.ray_early_out, null, Query{ .ts = &ts, .ray = ray, .settings = &settings, .filter = &filter.base, .sub_shape_queries = &sub_shape_queries });
        log.write(s);
    }

    // CollidePoint
    {
        var log: FilterLog = .{};
        var filter: PairwiseFilter = .{ .reject_modulus = in.reject_modulus, .log = &log };
        var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer collector.deinit();
        ts.collidePoint(rvec3(in.point), &collector.base, .{ .shape_filter = &filter.base });
        try collector.checkError();
        s.u(marker_point);
        s.u(@intCast(collector.hits.items.len));
        for (collector.hits.items) |hit| {
            s.u(hit.body_id.getIndexAndSequenceNumber());
            s.u(hit.sub_shape_id2.getValue());
        }
        log.write(s);
    }

    // CollectTransformedShapes
    {
        var log: FilterLog = .{};
        const filter: PairwiseFilter = .{ .reject_modulus = in.reject_modulus, .log = &log };
        var collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer collector.deinit();
        ts.collectTransformedShapes(aabox(in.box), &collector.base, .{ .shape_filter = &filter.base });
        try collector.checkError();
        s.u(marker_transformed_shapes);
        s.u(@intCast(collector.hits.items.len));
        for (collector.hits.items) |*hit| {
            // The shape as the index of the node shape (0xffffffff when it is not a node shape)
            var index: u32 = 0xffffffff;
            for (shapes, 0..) |node_shape, i|
                if (node_shape.get() == hit.shape.get()) {
                    index = @intCast(i);
                    break;
                };
            s.u(index);
            s.u(@intFromEnum(hit.shape.get().?.getSubType()));
            s.r3(hit.shape_position_com);
            s.q(hit.shape_rotation);
            s.v3(hit.getShapeScale());
            s.u(hit.body_id.getIndexAndSequenceNumber());
            s.u(hit.sub_shape_id_creator.getID().getValue());
            s.u(hit.sub_shape_id_creator.getNumBitsWritten());
            s.box(hit.getWorldSpaceBounds());
        }
        log.write(s);
    }

    // GetTrianglesStart / Next
    if (in.triangles != 0) {
        var context: Shape.GetTrianglesContext = .{};
        ts.getTrianglesStart(&context, aabox(in.box), base_offset);
        const max: u32 = @intCast(in.max_triangles_requested);
        const vertices = try allocator.alloc(Float3, 3 * max);
        defer allocator.free(vertices);
        const out_materials = try allocator.alloc(*const PhysicsMaterial, max);
        defer allocator.free(out_materials);
        s.u(marker_triangles);
        for (0..1000) |_| {
            const count = ts.getTrianglesNext(&context, max, vertices, if (in.materials != 0) out_materials else null);
            s.u(count);
            for (vertices[0 .. 3 * count]) |v| s.f3(v);
            if (in.materials != 0)
                for (out_materials[0..count]) |m| materials.write(s, m);
            if (count == 0)
                break;
        }
    }

    // GetSupportingFace with the ID of the root
    if (in.face_of_root != 0) {
        var face: Shape.SupportingFace = .empty;
        ts.getSupportingFace(makeCreator(in.creator).getID(), vec3(in.face_direction), base_offset, &face);
        s.u(marker_face);
        s.face(&face);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// The catalogue description

const Rng = fw.Rng;

const Gen = struct {
    rng: Rng = .{},

    fn index(self: *Gen, n: u32) u32 {
        return self.rng.next() % n;
    }

    fn chance(self: *Gen, percent: u32) bool {
        return self.index(100) < percent;
    }

    fn float(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn vec(self: *Gen, min: f32, max: f32) Vec3 {
        return Vec3.init(self.float(min, max), self.float(min, max), self.float(min, max));
    }

    fn direction(self: *Gen) Vec3 {
        while (true) {
            const v = self.vec(-1, 1);
            const len_sq = v.lengthSq();
            if (len_sq > 0.01 and len_sq <= 1.0)
                return v.divScalar(@sqrt(len_sq));
        }
    }

    /// A random rotation: identity, a multiple of 90 degrees around an axis or random
    fn rotation(self: *Gen) Quat {
        return switch (self.index(5)) {
            0 => Quat.identity(),
            1 => Quat.rotation(([_]Vec3{ Vec3.axisX(), Vec3.axisY(), Vec3.axisZ() })[self.index(3)], @as(f32, @floatFromInt(self.index(4))) * 0.5 * math.pi),
            else => Quat.rotation(self.direction(), self.float(-math.pi, math.pi)),
        };
    }

    /// A random point in a box
    fn pointIn(self: *Gen, box: AABox) Vec3 {
        return box.min.add(box.max.sub(box.min).mul(self.vec(0, 1)));
    }

    /// A sub shape ID creator that leaves enough bits for a shape that uses `shape_bits`
    fn creator(self: *Gen, shape_bits: u32) [2]u32 {
        const free = 32 - shape_bits;
        return switch (self.index(3)) {
            0 => .{ 0, 0 },
            1 => if (free >= 3) .{ self.index(8), 3 } else .{ 0, 0 },
            else => if (free >= 6) .{ self.index(1 << 6), 6 } else .{ 0, 0 },
        };
    }
};

/// One entry of the sweep: a node of the catalogue
const Entry = struct {
    name: []const u8,
    node: u32,
    /// The pair costs (Debug): 1 cheap (convex), 2 decorated / small composite, 4 mesh / height field / compound
    cost: u32,
};

/// The catalogue description and its arrays
const Catalogue = struct {
    allocator: Allocator,
    nodes: std.ArrayList(NodeDesc) = .empty,
    subs: std.ArrayList(SubDesc) = .empty,
    floats: std.ArrayList(f32) = .empty,
    triangles: std.ArrayList(IndexedTriangle) = .empty,
    bytes: std.ArrayList(u8) = .empty,
    entries: std.ArrayList(Entry) = .empty,

    fn deinit(self: *Catalogue) void {
        self.nodes.deinit(self.allocator);
        self.subs.deinit(self.allocator);
        self.floats.deinit(self.allocator);
        self.triangles.deinit(self.allocator);
        self.bytes.deinit(self.allocator);
        self.entries.deinit(self.allocator);
    }

    fn input(self: *const Catalogue) CatalogueInput {
        const dummy_triangle = &[_]IndexedTriangle{.init(0, 0, 0, .{})};
        const dummy_byte = &[_]u8{0};
        return .{
            .nodes = self.nodes.items.ptr,
            .subs = self.subs.items.ptr,
            .floats = self.floats.items.ptr,
            .triangles = if (self.triangles.items.len > 0) self.triangles.items.ptr else dummy_triangle,
            .bytes = if (self.bytes.items.len > 0) self.bytes.items.ptr else dummy_byte,
            .num_nodes = @intCast(self.nodes.items.len),
        };
    }

    /// Add a node, returns its index
    fn node(self: *Catalogue, desc: NodeDesc) !u32 {
        try self.nodes.append(self.allocator, desc);
        return @intCast(self.nodes.items.len - 1);
    }

    /// Add a node that is part of the sweep
    fn entry(self: *Catalogue, name: []const u8, cost: u32, desc: NodeDesc) !u32 {
        const index = try self.node(desc);
        try self.entries.append(self.allocator, .{ .name = name, .node = index, .cost = cost });
        return index;
    }

    /// Add an existing node to the sweep
    fn addEntry(self: *Catalogue, name: []const u8, cost: u32, index: u32) !void {
        try self.entries.append(self.allocator, .{ .name = name, .node = index, .cost = cost });
    }

    fn params(values: anytype) [16]f32 {
        var result: [16]f32 = @splat(0);
        inline for (values, 0..) |v, i| result[i] = v;
        return result;
    }

    fn points(self: *Catalogue, list: []const Vec3) !u32 {
        const first: u32 = @intCast(self.floats.items.len);
        for (list) |p| try self.floats.appendSlice(self.allocator, &arr3(p));
        return first;
    }

    fn compound(self: *Catalogue, name: []const u8, cost: u32, kind: NodeKind, user_data: u32, list: []const SubDesc) !u32 {
        const first: u32 = @intCast(self.subs.items.len);
        try self.subs.appendSlice(self.allocator, list);
        return self.entry(name, cost, .{ .kind = kind, .first = first, .count = @intCast(list.len), .user_data = user_data });
    }
};

fn subShape(node: u32, user_data: u32, position: Vec3, rotation: Quat) SubDesc {
    return .{ .node = node, .user_data = user_data, .position = arr3(position), .rotation = arr4(rotation.getXYZW()) };
}

/// A grid mesh of nx * nz cells (spacing, random heights, alternating diagonals) or a closed box
fn addMesh(cat: *Catalogue, gen: *Gen, closed: bool) !NodeDesc {
    const allocator = cat.allocator;
    const first: u32 = @intCast(cat.floats.items.len);
    const first2: u32 = @intCast(cat.triangles.items.len);
    var num_vertices: u32 = 0;
    if (!closed) {
        const n = 4;
        for (0..n + 1) |z|
            for (0..n + 1) |x| {
                const fx: f32 = @floatFromInt(x);
                const fz: f32 = @floatFromInt(z);
                try cat.floats.appendSlice(allocator, &.{ (fx - 2.0) * 0.8, gen.float(-0.25, 0.25), (fz - 2.0) * 0.8 });
                num_vertices += 1;
            };
        for (0..n) |z|
            for (0..n) |x| {
                const v0: u32 = @intCast(z * (n + 1) + x);
                const v1 = v0 + 1;
                const v2 = v0 + n + 1;
                const v3 = v2 + 1;
                const m: u32 = @intCast((x + z) % 3);
                if ((x + z) % 2 == 0) {
                    try cat.triangles.append(allocator, .init(v0, v2, v1, .{ .material_index = m, .user_data = @intCast(x) }));
                    try cat.triangles.append(allocator, .init(v1, v2, v3, .{ .material_index = m, .user_data = @intCast(z) }));
                } else {
                    try cat.triangles.append(allocator, .init(v0, v2, v3, .{ .material_index = m, .user_data = @intCast(x) }));
                    try cat.triangles.append(allocator, .init(v0, v3, v1, .{ .material_index = m, .user_data = @intCast(z) }));
                }
            };
    } else {
        const h = Vec3.init(0.8, 0.6, 0.7);
        for (0..8) |i| {
            const v = Vec3.init(if (i & 1 != 0) h.getX() else -h.getX(), if (i & 2 != 0) h.getY() else -h.getY(), if (i & 4 != 0) h.getZ() else -h.getZ());
            try cat.floats.appendSlice(allocator, &arr3(v));
            num_vertices += 1;
        }
        // Outward facing (counter clockwise seen from outside)
        const quads = [_][4]u32{ .{ 0, 4, 6, 2 }, .{ 1, 3, 7, 5 }, .{ 0, 1, 5, 4 }, .{ 2, 6, 7, 3 }, .{ 0, 2, 3, 1 }, .{ 4, 5, 7, 6 } };
        for (quads, 0..) |quad, i| {
            const m: u32 = @intCast(i % 2);
            try cat.triangles.append(allocator, .init(quad[0], quad[1], quad[2], .{ .material_index = m, .user_data = @intCast(2 * i) }));
            try cat.triangles.append(allocator, .init(quad[0], quad[2], quad[3], .{ .material_index = m, .user_data = @intCast(2 * i + 1) }));
        }
    }
    return .{
        .kind = .mesh,
        .first = first,
        .count = num_vertices,
        .first2 = first2,
        .count2 = @as(u32, @intCast(cat.triangles.items.len)) - first2,
        .num_materials = if (closed) 2 else 3,
        .param0 = if (closed) 8 else 4,
        .param1 = @intFromBool(!closed),
        .f = Catalogue.params(.{0.996195}),
        .user_data = if (closed) 21 else 22,
    };
}

/// A height field of sample_count^2 samples with no-collision samples, optionally with materials
fn addHeightField(cat: *Catalogue, gen: *Gen, sample_count: u32, block_size: u32, with_materials: bool) !NodeDesc {
    const allocator = cat.allocator;
    const first: u32 = @intCast(cat.floats.items.len);
    for (0..sample_count) |y|
        for (0..sample_count) |x| {
            const fx: f32 = @floatFromInt(x);
            const fy: f32 = @floatFromInt(y);
            const no_collision = (x == 2 and y == 3) or (x == sample_count - 2 and y == 1) or (x == 4 and y >= 4 and y <= 5);
            try cat.floats.append(allocator, if (no_collision) HeightFieldShapeConstants.no_collision_value else 0.1 * fx - 0.05 * fy + gen.float(-0.2, 0.2));
        };
    const first2: u32 = @intCast(cat.bytes.items.len);
    var count2: u32 = 0;
    if (with_materials) {
        for (0..(sample_count - 1) * (sample_count - 1)) |i| try cat.bytes.append(allocator, @intCast((i / 3) % 4));
        count2 = (sample_count - 1) * (sample_count - 1);
    }
    const extent: f32 = @floatFromInt(sample_count - 1);
    return .{
        .kind = .height_field,
        .first = first,
        .count = sample_count,
        .first2 = first2,
        .count2 = count2,
        .num_materials = if (with_materials) 4 else 0,
        .param0 = block_size,
        .param1 = if (with_materials) 8 else 6,
        .f = Catalogue.params(.{ -0.5 * extent * 0.5, -0.2, -0.5 * extent * 0.45, 0.5, 1.2, 0.45, 0.996195 }),
        .user_data = 30 + sample_count,
    };
}

/// The catalogue of the sweep
fn makeCatalogue(allocator: Allocator) !Catalogue {
    var cat: Catalogue = .{ .allocator = allocator };
    errdefer cat.deinit();
    var gen: Gen = .{ .rng = .{ .state = 0x5eed1234 } };

    // Convex shapes, with and without convex radius and materials
    const sphere = try cat.entry("sphere", 1, .{ .kind = .sphere, .f = Catalogue.params(.{0.5}), .user_data = 1 });
    const sphere_big = try cat.entry("sphere (material)", 1, .{ .kind = .sphere, .material = 2, .f = Catalogue.params(.{1.3}), .user_data = 2 });
    const box = try cat.entry("box", 1, .{ .kind = .box, .f = Catalogue.params(.{ 0.6, 0.4, 0.8 }), .convex_radius = 0, .user_data = 3 });
    const box_round = try cat.entry("box (convex radius, material)", 1, .{ .kind = .box, .material = 3, .f = Catalogue.params(.{ 1.0, 0.3, 0.5 }), .convex_radius = 0.1, .user_data = 4 });
    const capsule = try cat.entry("capsule", 1, .{ .kind = .capsule, .f = Catalogue.params(.{ 0.7, 0.4 }), .user_data = 5 });
    _ = try cat.entry("capsule (short, material)", 1, .{ .kind = .capsule, .material = 4, .f = Catalogue.params(.{ 0.2, 0.9 }), .user_data = 6 });
    const tapered_capsule = try cat.entry("tapered capsule", 1, .{ .kind = .tapered_capsule, .f = Catalogue.params(.{ 0.6, 0.3, 0.6 }), .user_data = 7 });
    _ = try cat.entry("tapered capsule (top bigger, material)", 1, .{ .kind = .tapered_capsule, .material = 5, .f = Catalogue.params(.{ 0.4, 0.7, 0.2 }), .user_data = 8 });
    const cylinder = try cat.entry("cylinder", 1, .{ .kind = .cylinder, .f = Catalogue.params(.{ 0.8, 0.5 }), .convex_radius = 0, .user_data = 9 });
    _ = try cat.entry("cylinder (convex radius, material)", 1, .{ .kind = .cylinder, .material = 6, .f = Catalogue.params(.{ 0.3, 1.0 }), .convex_radius = 0.05, .user_data = 10 });
    _ = try cat.entry("tapered cylinder", 1, .{ .kind = .tapered_cylinder, .f = Catalogue.params(.{ 0.7, 0.2, 0.6 }), .convex_radius = 0, .user_data = 11 });
    _ = try cat.entry("tapered cylinder (convex radius, material)", 1, .{ .kind = .tapered_cylinder, .material = 7, .f = Catalogue.params(.{ 0.5, 0.8, 0.4 }), .convex_radius = 0.05, .user_data = 12 });
    _ = try cat.entry("cone", 1, .{ .kind = .tapered_cylinder, .f = Catalogue.params(.{ 0.6, 0.0, 0.5 }), .convex_radius = 0, .user_data = 13 });

    // Convex hulls: a tetrahedron, 8 random points, 64 points on an ellipsoid, 150 points in a sphere
    const tetrahedron = try cat.points(&.{ Vec3.init(-0.5, -0.4, -0.5), Vec3.init(0.7, -0.4, -0.3), Vec3.init(0.0, -0.4, 0.8), Vec3.init(0.1, 0.7, 0.0) });
    const hull_small = try cat.entry("convex hull (tetrahedron)", 1, .{ .kind = .convex_hull, .first = tetrahedron, .count = 4, .convex_radius = 0, .user_data = 14 });
    var random_points: [8]Vec3 = undefined;
    for (&random_points) |*p| p.* = gen.vec(-0.7, 0.7);
    const hull8 = try cat.entry("convex hull (8 points, convex radius, material)", 1, .{ .kind = .convex_hull, .material = 8, .first = try cat.points(&random_points), .count = 8, .convex_radius = 0.05, .user_data = 15 });
    var ellipsoid: [64]Vec3 = undefined;
    for (&ellipsoid) |*p| p.* = gen.direction().mul(Vec3.init(0.9, 0.5, 0.7));
    _ = try cat.entry("convex hull (64 points)", 1, .{ .kind = .convex_hull, .first = try cat.points(&ellipsoid), .count = 64, .convex_radius = 0.05, .user_data = 16 });
    var cloud: [150]Vec3 = undefined;
    for (&cloud) |*p| p.* = gen.direction().mulScalar(gen.float(0.3, 0.8));
    _ = try cat.entry("convex hull (150 points)", 1, .{ .kind = .convex_hull, .first = try cat.points(&cloud), .count = 150, .convex_radius = 0, .user_data = 17 });

    // Triangles
    const triangle = try cat.entry("triangle", 1, .{ .kind = .triangle, .f = Catalogue.params(.{ -0.8, 0.0, -0.5, 0.9, 0.1, -0.4, 0.0, -0.1, 0.9 }), .user_data = 18 });
    _ = try cat.entry("triangle (convex radius, material)", 1, .{ .kind = .triangle, .material = 1, .f = Catalogue.params(.{ -0.6, 0.2, 0.5, 0.7, -0.2, 0.4, 0.1, 0.0, -0.8 }), .convex_radius = 0.1, .user_data = 19 });

    // Extreme sizes and convex radii
    _ = try cat.entry("sphere (tiny)", 1, .{ .kind = .sphere, .f = Catalogue.params(.{0.05}), .user_data = 50 });
    _ = try cat.entry("box (big, big convex radius)", 1, .{ .kind = .box, .f = Catalogue.params(.{ 3.0, 2.0, 2.5 }), .convex_radius = 0.5, .user_data = 51 });
    var box_points: [8]Vec3 = undefined;
    for (&box_points, 0..) |*p, i| p.* = Vec3.init(if (i & 1 != 0) 0.9 else -0.7, if (i & 2 != 0) 0.6 else -0.5, if (i & 4 != 0) 0.8 else -0.8).add(gen.vec(-0.05, 0.05));
    _ = try cat.entry("convex hull (big convex radius)", 1, .{ .kind = .convex_hull, .first = try cat.points(&box_points), .count = 8, .convex_radius = 0.3, .user_data = 52 });
    _ = try cat.entry("triangle (sliver)", 1, .{ .kind = .triangle, .f = Catalogue.params(.{ -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.03, 0.01 }), .user_data = 53 });
    if (!asserts) {
        // Degenerate triangles assert in CollideConvexVsTriangles (Jolt's debug build and Zolt's safe builds)
        _ = try cat.entry("triangle (degenerate)", 1, .{ .kind = .triangle, .f = Catalogue.params(.{ -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.5, 0.0, 0.0 }), .user_data = 54 });
    }

    // Planes and empty shapes
    _ = try cat.entry("plane", 2, .{ .kind = .plane, .f = Catalogue.params(.{ 0, 1, 0, 0, 4 }), .user_data = 20 });
    const tilted = Vec3.init(0.2, 1.0, -0.3).normalized();
    _ = try cat.entry("plane (tilted, material)", 2, .{ .kind = .plane, .material = 2, .f = Catalogue.params(.{ tilted.getX(), tilted.getY(), tilted.getZ(), 0.3, 3 }), .user_data = 21 });
    _ = try cat.entry("empty", 1, .{ .kind = .empty, .user_data = 22 });
    _ = try cat.entry("empty (center of mass)", 1, .{ .kind = .empty, .f = Catalogue.params(.{ 0.1, 0.2, -0.3 }), .user_data = 23 });

    // Decorators around convex shapes
    _ = try cat.entry("scaled box (uniform)", 2, .{ .kind = .scaled, .child = box, .f = Catalogue.params(.{ 1.5, 1.5, 1.5 }), .user_data = 24 });
    _ = try cat.entry("scaled box (non uniform)", 2, .{ .kind = .scaled, .child = box_round, .f = Catalogue.params(.{ 1.2, 0.5, 2.0 }), .user_data = 25 });
    _ = try cat.entry("scaled hull (inside out)", 2, .{ .kind = .scaled, .child = hull8, .f = Catalogue.params(.{ -1.0, 1.3, 0.8 }), .user_data = 26 });
    _ = try cat.entry("scaled sphere", 2, .{ .kind = .scaled, .child = sphere, .f = Catalogue.params(.{ -2.0, -2.0, -2.0 }), .user_data = 27 });
    const rt_quat = Quat.rotation(Vec3.init(0.3, 1.0, 0.2).normalized(), 0.7);
    const rt_capsule = try cat.entry("rotated translated capsule", 2, .{ .kind = .rotated_translated, .child = capsule, .f = Catalogue.params(.{ 0.3, -0.2, 0.1, rt_quat.getX(), rt_quat.getY(), rt_quat.getZ(), rt_quat.getW() }), .user_data = 28 });
    const rt_quat2 = Quat.rotation(Vec3.axisX(), 0.5 * math.pi);
    _ = try cat.entry("rotated translated hull", 2, .{ .kind = .rotated_translated, .child = hull_small, .f = Catalogue.params(.{ -0.4, 0.5, 0.0, rt_quat2.getX(), rt_quat2.getY(), rt_quat2.getZ(), rt_quat2.getW() }), .user_data = 29 });
    _ = try cat.entry("offset center of mass sphere", 2, .{ .kind = .offset_center_of_mass, .child = sphere_big, .f = Catalogue.params(.{ 0.2, 0.1, -0.3 }), .user_data = 30 });
    _ = try cat.entry("offset center of mass cylinder", 2, .{ .kind = .offset_center_of_mass, .child = cylinder, .f = Catalogue.params(.{ -0.1, 0.4, 0.0 }), .user_data = 31 });

    // Meshes and decorators around them
    const mesh_grid = try cat.entry("mesh (grid)", 4, try addMesh(&cat, &gen, false));
    const mesh_closed = try cat.entry("mesh (closed box)", 4, try addMesh(&cat, &gen, true));
    _ = try cat.entry("scaled mesh", 4, .{ .kind = .scaled, .child = mesh_grid, .f = Catalogue.params(.{ 1.5, -1.0, 0.8 }), .user_data = 32 });
    const rt_quat3 = Quat.rotation(Vec3.init(1.0, 0.2, -0.4).normalized(), -0.9);
    _ = try cat.entry("rotated translated mesh", 4, .{ .kind = .rotated_translated, .child = mesh_closed, .f = Catalogue.params(.{ 0.2, 0.3, -0.5, rt_quat3.getX(), rt_quat3.getY(), rt_quat3.getZ(), rt_quat3.getW() }), .user_data = 33 });
    _ = try cat.entry("offset center of mass mesh", 4, .{ .kind = .offset_center_of_mass, .child = mesh_grid, .f = Catalogue.params(.{ 0.3, -0.2, 0.1 }), .user_data = 34 });

    // Height fields
    _ = try cat.entry("height field", 4, try addHeightField(&cat, &gen, 8, 2, false));
    _ = try cat.entry("height field (materials, block size 4)", 4, try addHeightField(&cat, &gen, 16, 4, true));

    // Compounds
    const static_compound = try cat.compound("static compound", 3, .static_compound, 40, &.{
        subShape(sphere, 100, Vec3.zero(), Quat.identity()),
        subShape(box, 101, Vec3.init(1.0, 0.0, 0.0), Quat.rotation(Vec3.axisY(), 0.4)),
        subShape(capsule, 102, Vec3.init(-1.0, 0.5, 0.0), Quat.rotation(Vec3.axisZ(), 1.1)),
        subShape(hull_small, 103, Vec3.init(0.0, 1.0, 0.2), Quat.identity()),
    });
    const mutable_compound = try cat.compound("mutable compound", 3, .mutable_compound, 41, &.{
        subShape(cylinder, 110, Vec3.init(0.0, 0.0, 0.5), Quat.rotation(Vec3.axisX(), 0.3)),
        subShape(tapered_capsule, 111, Vec3.init(1.2, 0.2, 0.0), Quat.identity()),
        subShape(box_round, 112, Vec3.init(-0.8, -0.3, -0.4), Quat.rotation(Vec3.init(1, 1, 0).normalized(), 0.8)),
        subShape(sphere_big, 113, Vec3.init(0.0, 1.5, 0.0), Quat.identity()),
    });
    _ = try cat.compound("static compound (nested)", 4, .static_compound, 42, &.{
        subShape(mutable_compound, 120, Vec3.init(0.5, 0.0, 0.0), Quat.identity()),
        subShape(sphere, 121, Vec3.init(-1.5, 0.0, 0.0), Quat.identity()),
        subShape(static_compound, 122, Vec3.init(0.0, 1.5, -0.5), Quat.rotation(Vec3.axisY(), -0.6)),
    });
    _ = try cat.compound("mutable compound (nested)", 4, .mutable_compound, 43, &.{
        subShape(static_compound, 130, Vec3.init(0.0, 0.0, 0.0), Quat.rotation(Vec3.axisZ(), 0.25)),
        subShape(rt_capsule, 131, Vec3.init(1.5, 0.0, 0.3), Quat.identity()),
        subShape(triangle, 132, Vec3.init(-1.0, -0.5, 0.0), Quat.rotation(Vec3.axisX(), 0.2)),
    });
    _ = try cat.compound("static compound (mesh)", 4, .static_compound, 44, &.{
        subShape(mesh_closed, 140, Vec3.zero(), Quat.identity()),
        subShape(box, 141, Vec3.init(2.0, 0.0, 0.0), Quat.rotation(Vec3.axisY(), 0.7)),
    });
    const unrotated = try cat.compound("static compound (unrotated)", 3, .static_compound, 45, &.{
        subShape(box, 150, Vec3.zero(), Quat.identity()),
        subShape(box_round, 151, Vec3.init(1.5, 0.0, 0.0), Quat.identity()),
        subShape(hull8, 152, Vec3.init(0.0, 1.2, 0.0), Quat.identity()),
    });

    // Many sub shapes (several levels in the tree of the static compound, several blocks of the mutable compound)
    var many: [20]SubDesc = undefined;
    const leaves = [_]u32{ sphere, box, capsule, hull_small, cylinder, box_round, tapered_capsule };
    for (&many, 0..) |*sd, i| sd.* = subShape(leaves[i % leaves.len], 160 + @as(u32, @intCast(i)), gen.vec(-3, 3), gen.rotation());
    _ = try cat.compound("static compound (20 sub shapes)", 4, .static_compound, 46, &many);
    _ = try cat.compound("mutable compound (11 sub shapes)", 4, .mutable_compound, 47, many[0..11]);

    // Decorators around compounds
    _ = try cat.entry("scaled compound (non uniform)", 4, .{ .kind = .scaled, .child = unrotated, .f = Catalogue.params(.{ 1.2, -0.8, 1.0 }), .user_data = 48 });
    const rt_quat4 = Quat.rotation(Vec3.init(0.0, 0.6, 0.8), 2.1);
    _ = try cat.entry("rotated translated compound", 4, .{ .kind = .rotated_translated, .child = mutable_compound, .f = Catalogue.params(.{ -0.5, 0.2, 0.7, rt_quat4.getX(), rt_quat4.getY(), rt_quat4.getZ(), rt_quat4.getW() }), .user_data = 49 });
    _ = try cat.entry("offset center of mass compound", 4, .{ .kind = .offset_center_of_mass, .child = static_compound, .f = Catalogue.params(.{ 0.3, 0.3, -0.2 }), .user_data = 55 });

    return cat;
}

// ---------------------------------------------------------------------------------------------------------------------
// Comparison of the streams

const Comparison = struct {
    name: []const u8,
    calls: usize = 0,
    mismatches: usize = 0,

    fn check(self: *Comparison, what: []const u8, input: anytype, zolt_values: []const u32, jolt_values: []const u32) void {
        self.calls += 1;
        if (std.mem.eql(u32, zolt_values, jolt_values))
            return;
        if (self.mismatches < 5) {
            // Find the first difference and the last marker before it
            var first: usize = 0;
            while (first < zolt_values.len and first < jolt_values.len and zolt_values[first] == jolt_values[first]) first += 1;
            var marker: u32 = 0;
            for (zolt_values[0..first]) |v|
                if (v >= 0xC0000001 and v <= 0xC000000C) {
                    marker = v;
                };
            std.debug.print("{s}: {s} mismatch at value {d} (after marker 0x{x}), lengths zolt {d} jolt {d}\n", .{ self.name, what, first, marker, zolt_values.len, jolt_values.len });
            const from = first -| 6;
            std.debug.print("  zolt: {any}\n  jolt: {any}\n  input: {any}\n", .{ zolt_values[from..@min(zolt_values.len, first + 8)], jolt_values[from..@min(jolt_values.len, first + 8)], input });
        }
        self.mismatches += 1;
    }

    fn finish(self: *const Comparison) !void {
        if (self.mismatches > 0) {
            std.debug.print("{s}: {d} mismatches in {d} calls\n", .{ self.name, self.mismatches, self.calls });
            return error.ParityMismatch;
        }
    }
};

/// Run a C++ function that writes a stream (`call.run(out, capacity) u32`)
fn joltStream(buffer: []u32, call: anytype) []const u32 {
    const size = call.run(buffer.ptr, @intCast(buffer.len));
    std.debug.assert(size <= buffer.len); // The capacity is big enough for every query of the test
    return buffer[0..size];
}

// ---------------------------------------------------------------------------------------------------------------------
// The shapes on both sides

/// The catalogue built on both sides, with per node information for the input generation
const Built = struct {
    allocator: Allocator,
    cat: Catalogue,
    materials: Materials = .{},
    shapes: std.ArrayList(RefConst(Shape)) = .empty,
    handle: *anyopaque = undefined,
    /// Per node: the sub types of the leaf shapes (bit set) and the number of sub shape ID bits
    leaf_types: std.ArrayList(u64) = .empty,
    /// Per node: contains a MutableCompoundShape with more than one block of 4 sub shapes. Its WalkSubShapes only stops
    /// the walk through the current block when the visitor aborts (CollisionArchitecture.md section 9), so the AnyHit
    /// collector can receive a second hit after forcing an early out, which Jolt asserts (and Zolt in safe builds).
    multi_block: std.ArrayList(bool) = .empty,

    fn init(allocator: Allocator, cat: Catalogue, buffer: []u32) !*Built {
        const self = try allocator.create(Built);
        errdefer allocator.destroy(self);
        self.* = .{ .allocator = allocator, .cat = cat };
        errdefer self.cat.deinit();
        try self.materials.init(allocator);
        errdefer self.materials.deinit();

        // Create on both sides and compare the properties
        const input = self.cat.input();
        var zs: Stream = .{ .allocator = allocator };
        defer zs.deinit();
        const zolt_valid = try zoltCatalogue(allocator, &self.materials, &input, &self.shapes, &zs);
        var jolt_size: u32 = 0;
        const handle = jolt.pw_catalogue_create(&input, buffer.ptr, @intCast(buffer.len), &jolt_size);
        var cmp: Comparison = .{ .name = "Pairwise catalogue (shape properties)" };
        cmp.check("create", input.num_nodes, zs.values.items, buffer[0..jolt_size]);
        if (!zolt_valid or handle == null) {
            std.debug.print("Pairwise catalogue: invalid node (zolt valid {}, jolt valid {})\n", .{ zolt_valid, handle != null });
            if (handle) |h| jolt.pw_catalogue_destroy(h);
            for (self.shapes.items) |*ref| ref.deinit();
            self.shapes.deinit(allocator);
            return error.TestUnexpectedResult;
        }
        self.handle = handle.?;
        cmp.finish() catch |err| {
            self.deinitShapes();
            return err;
        };

        // The leaf sub types of every node
        for (self.cat.nodes.items, 0..) |n, i| {
            const own: u64 = @as(u64, 1) << @intCast(@intFromEnum(self.shapes.items[i].get().?.getSubType()));
            const leaves: u64 = switch (n.kind) {
                .scaled, .rotated_translated, .offset_center_of_mass => self.leaf_types.items[n.child],
                .static_compound, .mutable_compound => blk: {
                    var set: u64 = 0;
                    for (self.cat.subs.items[n.first..][0..n.count]) |sd| set |= self.leaf_types.items[sd.node];
                    break :blk set;
                },
                else => own,
            };
            try self.leaf_types.append(allocator, leaves);
            const multi_block = switch (n.kind) {
                .scaled, .rotated_translated, .offset_center_of_mass => self.multi_block.items[n.child],
                .static_compound, .mutable_compound => blk: {
                    var any = n.kind == .mutable_compound and n.count > 4;
                    for (self.cat.subs.items[n.first..][0..n.count]) |sd| any = any or self.multi_block.items[sd.node];
                    break :blk any;
                },
                else => false,
            };
            try self.multi_block.append(allocator, multi_block);
        }
        return self;
    }

    fn deinitShapes(self: *Built) void {
        jolt.pw_catalogue_destroy(self.handle);
        for (self.shapes.items) |*ref| ref.deinit();
        self.shapes.deinit(self.allocator);
    }

    fn deinit(self: *Built) void {
        self.deinitShapes();
        self.leaf_types.deinit(self.allocator);
        self.multi_block.deinit(self.allocator);
        self.materials.deinit();
        self.cat.deinit();
        self.allocator.destroy(self);
    }

    fn shape(self: *const Built, node: u32) *const Shape {
        return self.shapes.items[node].get().?;
    }

    /// The collide / cast functions of every pair of leaf sub types of the two nodes are supported
    fn supported(self: *const Built, node1: u32, node2: u32, comptime cast: bool) bool {
        const set1 = self.leaf_types.items[node1];
        const set2 = self.leaf_types.items[node2];
        for (0..64) |t1| {
            if (set1 & (@as(u64, 1) << @intCast(t1)) == 0) continue;
            for (0..64) |t2| {
                if (set2 & (@as(u64, 1) << @intCast(t2)) == 0) continue;
                const s1: ShapeSubType = @enumFromInt(t1);
                const s2: ShapeSubType = @enumFromInt(t2);
                if (cast) {
                    if (RegisterTypes.registry.getCastShape(s1, s2) == @as(CollisionDispatch.CastShape, &CollisionDispatch.castUnsupported)) return false;
                } else {
                    if (RegisterTypes.registry.getCollideShape(s1, s2) == @as(CollisionDispatch.CollideShape, &CollisionDispatch.collideUnsupported)) return false;
                }
            }
        }
        return true;
    }

    /// The AnyHit collector can be used with the nodes (see multi_block)
    fn anyHitAllowed(self: *const Built, node1: u32, node2: u32) bool {
        return !asserts or !(self.multi_block.items[node1] or self.multi_block.items[node2]);
    }

    /// Bounds of the node shape with a scale at the origin
    fn bounds(self: *const Built, node: u32, scale: Vec3) AABox {
        return self.shape(node).getWorldSpaceBounds(Mat44.identity(), scale);
    }
};

/// A scale that IsValidScale accepts for the shape: one, uniform, uniform with signs, non uniform (with signs)
fn randomScale(gen: *Gen, shape: *const Shape) Vec3 {
    // Mostly moderate magnitudes, sometimes small or big ones
    const min: f32, const max: f32 = switch (gen.index(20)) {
        0 => .{ 0.2, 0.4 },
        1 => .{ 2.5, 4.0 },
        else => .{ 0.6, 1.6 },
    };
    const m = gen.float(min, max);
    const signs = Vec3.init(if (gen.chance(30)) -1 else 1, if (gen.chance(30)) -1 else 1, if (gen.chance(30)) -1 else 1);
    const candidates = [_]Vec3{ signs.mul(gen.vec(min, max)), signs.mul(Vec3.replicate(m)), Vec3.replicate(if (gen.chance(20)) -m else m), Vec3.one() };
    const start: usize = switch (gen.index(10)) {
        0, 1 => 3,
        2, 3 => 2,
        4, 5 => 1,
        else => 0,
    };
    for (candidates[start..]) |c|
        if (shape.isValidScale(c)) return c;
    return Vec3.one();
}

/// Position of shape 1 (center of mass) relative to shape 2 (world bounds `bounds2`, center of mass `com2`)
fn placement(gen: *Gen, bounds1: AABox, bounds2: AABox, com2: Vec3, axis_touching: bool) Vec3 {
    const extent1 = bounds1.getExtent();
    const r1 = extent1.length();
    const r2 = bounds2.getExtent().length();
    if (axis_touching) {
        // Shape 1 touches the bounds of shape 2 along an axis (bounds1 is at the origin, rotations are the same)
        const axis = gen.index(3);
        const positive = gen.chance(50);
        var p = gen.pointIn(bounds2);
        const touch = if (positive) bounds2.max.getComponent(axis) - bounds1.min.getComponent(axis) else bounds2.min.getComponent(axis) - bounds1.max.getComponent(axis);
        p.setComponent(axis, touch + (if (gen.chance(30)) gen.float(-0.02, 0.02) else 0.0));
        return p;
    }
    return switch (gen.index(4)) {
        // Around a point in the bounds of shape 2
        0, 1 => gen.pointIn(bounds2).add(gen.direction().mulScalar(r1 * gen.float(0, 1.5))),
        // Around the center of mass of shape 2 at a distance relative to the sizes (overlapping to separated)
        2 => com2.add(gen.direction().mulScalar((r1 + r2) * gen.float(0, 1.3))),
        // Deep
        else => com2.add(gen.vec(-0.05, 0.05)),
    };
}

// ---------------------------------------------------------------------------------------------------------------------
// The tests

/// Number of queries per pair: more for cheap pairs (Debug is ~50x slower than ReleaseFast, the sweep must stay fast)
fn queriesPerPair(built: *const Built, e1: Entry, e2: Entry) usize {
    _ = built;
    const cost = e1.cost * e2.cost;
    const base: usize = if (builtin.mode == .Debug) 12 else 48;
    return @max(4, base * 16 / cost);
}

/// An early out fraction set before the query: the ClosestHit collector updates the early out fraction with every hit
/// that is closer than its previous hit, Jolt asserts that this is not more than the current early out fraction (not every
/// shape function checks the early out fraction before it adds a hit). Only without asserts for ClosestHit.
fn earlyOutAllowed(collector: c_int) bool {
    return !asserts or collector != 1;
}

/// User1..8 and UserConvex1..8 (the parity build registers functions for some of them, see ShapeCoreUserTypes.zig)
fn isUserType(t: ShapeSubType) bool {
    return @intFromEnum(t) >= @intFromEnum(ShapeSubType.user1) and @intFromEnum(t) <= @intFromEnum(ShapeSubType.user_convex8);
}

/// SoftBodyShape is ported in Phase 9 (SoftBody): until then its register function is empty in Zolt
fn isSoftBody(t: ShapeSubType) bool {
    return t == .soft_body;
}

test "Pairwise parity: dispatch tables" {
    // Every pair of Jolt's sub types (user types excluded: the parity build registers some of them): unsupported and
    // reversed entries must be the same
    var mismatches: usize = 0;
    for (zolt.all_sub_shape_types) |t1| {
        if (isUserType(t1) or isSoftBody(t1)) continue;
        for (zolt.all_sub_shape_types) |t2| {
            if (isUserType(t2) or isSoftBody(t2)) continue;

            // Without asserts the unsupported functions and EmptyShape's functions all have empty bodies, which LLVM merges
            // into one function, so their addresses are the same: only the safe builds can tell them apart
            if (!asserts and (t1 == .empty or t2 == .empty)) continue;
            const c = RegisterTypes.registry.getCollideShape(t1, t2);
            const zolt_collide: c_int = if (c == @as(CollisionDispatch.CollideShape, &CollisionDispatch.collideUnsupported)) -1 else if (c == @as(CollisionDispatch.CollideShape, &CollisionDispatch.reversedCollideShape)) -2 else 1;
            const s = RegisterTypes.registry.getCastShape(t1, t2);
            const zolt_cast: c_int = if (s == @as(CollisionDispatch.CastShape, &CollisionDispatch.castUnsupported)) -1 else if (s == @as(CollisionDispatch.CastShape, &CollisionDispatch.reversedCastShape)) -2 else 1;
            var jolt_collide: c_int = 0;
            var jolt_cast: c_int = 0;
            jolt.pw_dispatch(@intFromEnum(t1), @intFromEnum(t2), &jolt_collide, &jolt_cast);
            if (zolt_collide != jolt_collide or zolt_cast != jolt_cast) {
                if (mismatches < 10)
                    std.debug.print("dispatch {s} vs {s}: zolt collide {d} cast {d}, jolt collide {d} cast {d}\n", .{ @tagName(t1), @tagName(t2), zolt_collide, zolt_cast, jolt_collide, jolt_cast });
                mismatches += 1;
            }
        }
    }
    if (mismatches > 0) return error.ParityMismatch;
}

test "Pairwise parity: collide and cast" {
    const allocator = std.testing.allocator;
    const buffer = try allocator.alloc(u32, stream_capacity);
    defer allocator.free(buffer);

    const built = try Built.init(allocator, try makeCatalogue(allocator), buffer);
    defer built.deinit();

    var collide_cmp: Comparison = .{ .name = "Pairwise collide (CollisionDispatch / InternalEdgeRemovingCollector)" };
    var cast_cmp: Comparison = .{ .name = "Pairwise cast (CollisionDispatch)" };
    var collide_hits: usize = 0;
    var cast_hits: usize = 0;
    var skipped: usize = 0;

    var gen: Gen = .{};
    var zs: Stream = .{ .allocator = allocator };
    defer zs.deinit();
    const entries = built.cat.entries.items;
    for (entries) |e1| {
        for (entries) |e2| {
            const shape1 = built.shape(e1.node);
            const shape2 = built.shape(e2.node);
            const collide_supported = built.supported(e1.node, e2.node, false);
            const cast_supported = built.supported(e1.node, e2.node, true);
            const n = queriesPerPair(built, e1, e2);
            for (0..n) |i| {
                const scale1 = randomScale(&gen, shape1);
                const scale2 = randomScale(&gen, shape2);
                const bounds1 = built.bounds(e1.node, scale1);
                const axis_touching = gen.chance(15);
                const rotation2 = gen.rotation();
                const rotation1 = if (axis_touching) rotation2 else gen.rotation();
                // Sometimes far from the origin (less precision in the world space transforms)
                const offset = if (gen.chance(10)) Vec3.init(500, -300, 200).add(gen.vec(-50, 50)) else Vec3.zero();
                const transform2 = Mat44.rotationTranslation(rotation2, gen.vec(-2, 2).add(offset));
                const bounds2 = shape2.getWorldSpaceBounds(transform2, scale2);
                const bounds1_rotated = shape1.getWorldSpaceBounds(Mat44.rotationQuat(rotation1), scale1);
                const com1 = placement(&gen, if (axis_touching) bounds1_rotated else bounds1, bounds2, transform2.getTranslation(), axis_touching);
                const transform1 = Mat44.rotationTranslation(rotation1, com1);
                const bits1 = shape1.getSubShapeIDBitsRecursive();
                const bits2 = shape2.getSubShapeIDBitsRecursive();
                const num_collectors: usize = if (built.anyHitAllowed(e1.node, e2.node)) 3 else 2;

                // Collide
                {
                    const collector: c_int = @intCast(i % num_collectors);
                    const internal_edge_removal = gen.chance(15) and !(asserts and collector == 2);
                    const input: CollideInput = .{
                        .shape1 = e1.node,
                        .shape2 = e2.node,
                        .scale1 = arr3(scale1),
                        .scale2 = arr3(scale2),
                        .transform1 = arr16(transform1),
                        .transform2 = arr16(transform2),
                        .creator1 = gen.creator(bits1),
                        .creator2 = gen.creator(bits2),
                        .max_separation_distance = switch (gen.index(10)) {
                            0, 1, 2, 3, 4, 5 => 0.0,
                            6, 7, 8 => gen.float(0, 0.3),
                            else => gen.float(0.5, 2.0),
                        },
                        .collision_tolerance = if (gen.chance(80)) 1.0e-4 else gen.float(1.0e-5, 1.0e-2),
                        .penetration_tolerance = if (gen.chance(80)) 1.0e-4 else gen.float(1.0e-5, 1.0e-2),
                        .back_face_mode = @intFromBool(gen.chance(40)),
                        .active_edge_mode = @intFromBool(internal_edge_removal or gen.chance(30)),
                        .collect_faces = @intFromBool(internal_edge_removal or gen.chance(40)),
                        .active_edge_movement_direction = if (gen.chance(50)) .{ 0, 0, 0 } else arr3(gen.vec(-1, 1)),
                        .collector = collector,
                        .early_out = if (gen.chance(85) or !earlyOutAllowed(collector)) math.flt_max else gen.float(-0.3, 0.3),
                        .body_id = gen.index(1000),
                        .reject_modulus = if (gen.chance(75)) 0 else 2 + gen.index(4),
                        .internal_edge_removal = @intFromBool(internal_edge_removal),
                        .vertex_tolerance_sq = if (gen.chance(50)) 1.0e-8 else gen.float(0, 0.01),
                    };
                    if (collide_supported or !asserts) {
                        zs.values.clearRetainingCapacity();
                        try zoltCollide(allocator, built.shapes.items, &input, &zs);
                        const Call = struct {
                            h: *anyopaque,
                            in: *const CollideInput,
                            fn run(c: @This(), out: [*]u32, cap: u32) u32 {
                                return jolt.pw_collide(c.h, c.in, out, cap);
                            }
                        };
                        const jolt_values = joltStream(buffer, Call{ .h = built.handle, .in = &input });
                        collide_cmp.check(e1.name, .{ e1.name, e2.name, input }, zs.values.items, jolt_values);
                        if (jolt_values.len > 1 and jolt_values[1] != 0) collide_hits += 1;
                    } else skipped += 1;
                }

                // Cast: shape 1 from a start position towards a point of shape 2 (or with a zero direction)
                {
                    const target = if (gen.chance(70)) gen.pointIn(bounds2) else transform2.getTranslation();
                    const size = bounds1.getExtent().length() + bounds2.getExtent().length();
                    var direction = if (axis_touching)
                        ([_]Vec3{ Vec3.axisX(), Vec3.axisY(), Vec3.axisZ() })[gen.index(3)].mulScalar(if (gen.chance(50)) size else -size)
                    else
                        gen.direction().mulScalar(size * gen.float(0.5, 3.0));
                    if (gen.chance(10)) direction = Vec3.zero();
                    const start_position = if (direction.isNearZero(.{})) com1 else target.sub(direction.mulScalar(gen.float(0.2, 1.1)));
                    const cast_collector: c_int = @intCast((i + 1) % num_collectors);
                    const input: CastInput = .{
                        .shape1 = e1.node,
                        .shape2 = e2.node,
                        .scale1 = arr3(scale1),
                        .start = arr16(Mat44.rotationTranslation(rotation1, start_position)),
                        .direction = arr3(direction),
                        .scale2 = arr3(scale2),
                        .transform2 = arr16(transform2),
                        .creator1 = gen.creator(bits1),
                        .creator2 = gen.creator(bits2),
                        .collision_tolerance = if (gen.chance(80)) 1.0e-4 else gen.float(1.0e-5, 1.0e-2),
                        .penetration_tolerance = if (gen.chance(80)) 1.0e-4 else gen.float(1.0e-5, 1.0e-2),
                        .extra_convex_radius = if (gen.chance(70)) 0.0 else gen.float(0, 0.2),
                        .back_face_mode_triangles = @intFromBool(gen.chance(40)),
                        .back_face_mode_convex = @intFromBool(gen.chance(40)),
                        .use_shrunken_shape = @intFromBool(gen.chance(40)),
                        .return_deepest_point = @intFromBool(gen.chance(40)),
                        .collect_faces = @intFromBool(gen.chance(40)),
                        .active_edge_mode = @intFromBool(gen.chance(40)),
                        .active_edge_movement_direction = if (gen.chance(50)) .{ 0, 0, 0 } else arr3(gen.vec(-1, 1)),
                        .collector = cast_collector,
                        .early_out = if (gen.chance(85) or !earlyOutAllowed(cast_collector)) 1.0 + math.flt_epsilon else gen.float(-0.1, 1.0),
                        .body_id = gen.index(1000),
                        .reject_modulus = if (gen.chance(75)) 0 else 2 + gen.index(4),
                    };
                    if (cast_supported or !asserts) {
                        zs.values.clearRetainingCapacity();
                        try zoltCast(allocator, built.shapes.items, &input, &zs);
                        const Call = struct {
                            h: *anyopaque,
                            in: *const CastInput,
                            fn run(c: @This(), out: [*]u32, cap: u32) u32 {
                                return jolt.pw_cast(c.h, c.in, out, cap);
                            }
                        };
                        const jolt_values = joltStream(buffer, Call{ .h = built.handle, .in = &input });
                        cast_cmp.check(e1.name, .{ e1.name, e2.name, input }, zs.values.items, jolt_values);
                        if (jolt_values.len > 7 and jolt_values[7] != 0) cast_hits += 1;
                    } else skipped += 1;
                }
            }
        }
    }

    if (print_statistics)
        std.debug.print("Pairwise parity: {d} collides ({d} with hits), {d} casts ({d} with hits), {d} skipped (unsupported pairs with asserts)\n", .{ collide_cmp.calls, collide_hits, cast_cmp.calls, cast_hits, skipped });
    // Enough of the queries hit something
    if (collide_hits * 4 < collide_cmp.calls or cast_hits * 4 < cast_cmp.calls) {
        std.debug.print("Pairwise parity: {d} of {d} collides and {d} of {d} casts hit something\n", .{ collide_hits, collide_cmp.calls, cast_hits, cast_cmp.calls });
        return error.TestUnexpectedResult;
    }

    var failed = false;
    for ([_]*const Comparison{ &collide_cmp, &cast_cmp }) |c|
        c.finish() catch {
            failed = true;
        };
    if (failed) return error.ParityMismatch;
}

test "Pairwise parity: TransformedShape queries" {
    const allocator = std.testing.allocator;
    const buffer = try allocator.alloc(u32, stream_capacity);
    defer allocator.free(buffer);

    const built = try Built.init(allocator, try makeCatalogue(allocator), buffer);
    defer built.deinit();

    var cmp: Comparison = .{ .name = "Pairwise TransformedShape queries" };
    var gen: Gen = .{ .rng = .{ .state = 0x7a5f00d } };
    var zs: Stream = .{ .allocator = allocator };
    defer zs.deinit();
    for (built.cat.entries.items) |e| {
        const shape = built.shape(e.node);
        const bits = shape.getSubShapeIDBitsRecursive();
        const n: usize = if (builtin.mode == .Debug) 1600 / e.cost else 4000;
        for (0..n) |_| {
            const scale = randomScale(&gen, shape);
            const rotation = gen.rotation();
            const far = gen.chance(10);
            const position = gen.vec(-3, 3);
            const offset: [3]f64 = if (far) .{ 1000.5, -2000.25, 500.0 } else .{ 0, 0, 0 };
            const pos: [3]f64 = .{ position.getX() + offset[0], position.getY() + offset[1], position.getZ() + offset[2] };
            const local_bounds = shape.getWorldSpaceBounds(Mat44.rotationQuat(rotation), scale);
            const world_min = local_bounds.min.sub(Vec3.replicate(0.2));
            const world_max = local_bounds.max.add(Vec3.replicate(0.2));
            const local_box = AABox.init(world_min, world_max);
            const target = gen.pointIn(local_box);
            const size = local_box.getSize().length();
            const ray_origin_local = if (gen.chance(15)) target.add(gen.vec(-0.1, 0.1)) else target.sub(gen.direction().mulScalar(size * gen.float(0.5, 1.5)));
            const ray_direction = if (gen.chance(5)) Vec3.zero() else target.sub(ray_origin_local).mulScalar(gen.float(1.0, 2.5));
            const point_local = gen.pointIn(local_box);
            // A sub box of the bounds or the whole bounds (world space)
            const box_center = gen.pointIn(local_box);
            const box_half = local_box.getExtent().mul(gen.vec(0.1, 1.2));
            const box_local = if (gen.chance(30)) local_box else AABox.init(box_center.sub(box_half), box_center.add(box_half));
            const base_offset: [3]f64 = switch (gen.index(3)) {
                0 => .{ 0, 0, 0 },
                1 => pos,
                else => .{ pos[0] + 0.5, pos[1] - 0.25, pos[2] + 1.0 },
            };
            const ray_collector: c_int = @intCast(gen.index(if (built.anyHitAllowed(e.node, e.node)) 3 else 2));
            const input: TransformedShapeInput = .{
                .shape = e.node,
                .body_id = gen.index(1000),
                .creator = gen.creator(bits),
                .position = pos,
                .rotation = arr4(rotation.getXYZW()),
                .scale = arr3(scale),
                .ray_origin = .{ ray_origin_local.getX() + pos[0], ray_origin_local.getY() + pos[1], ray_origin_local.getZ() + pos[2] },
                .ray_direction = arr3(ray_direction),
                .ray_fraction = if (gen.chance(80)) 1.0 + math.flt_epsilon else gen.float(0, 1),
                .back_face_mode_triangles = @intFromBool(gen.chance(40)),
                .back_face_mode_convex = @intFromBool(gen.chance(40)),
                .treat_convex_as_solid = @intFromBool(gen.chance(70)),
                .ray_collector = ray_collector,
                .ray_early_out = if (gen.chance(85) or !earlyOutAllowed(ray_collector)) 1.0 + math.flt_epsilon else gen.float(0, 1),
                .point = .{ point_local.getX() + pos[0], point_local.getY() + pos[1], point_local.getZ() + pos[2] },
                .box = .{ box_local.min.getX() + @as(f32, @floatCast(pos[0])), box_local.min.getY() + @as(f32, @floatCast(pos[1])), box_local.min.getZ() + @as(f32, @floatCast(pos[2])), box_local.max.getX() + @as(f32, @floatCast(pos[0])), box_local.max.getY() + @as(f32, @floatCast(pos[1])), box_local.max.getZ() + @as(f32, @floatCast(pos[2])) },
                .base_offset = base_offset,
                .max_triangles_requested = @intCast(32 + gen.index(100)),
                .materials = @intFromBool(gen.chance(70)),
                .face_direction = arr3(gen.direction()),
                .face_of_root = @intFromBool(bits == 0),
                .triangles = @intFromBool(!asserts or !(shape.getType() == .compound or shape.getType() == .decorated)),
                .reject_modulus = if (gen.chance(75)) 0 else 2 + gen.index(4),
            };
            zs.values.clearRetainingCapacity();
            try zoltTransformedShape(allocator, &built.materials, built.shapes.items, &input, &zs);
            const Call = struct {
                h: *anyopaque,
                in: *const TransformedShapeInput,
                fn run(c: @This(), out: [*]u32, cap: u32) u32 {
                    return jolt.pw_transformed_shape(c.h, c.in, out, cap);
                }
            };
            cmp.check(e.name, .{ e.name, input }, zs.values.items, joltStream(buffer, Call{ .h = built.handle, .in = &input }));
        }
    }
    try cmp.finish();
}
