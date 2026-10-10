//! Parity tests for TriangleShape (Phase 4, Wave B) and the collision functions it registers in CollisionDispatch
//! (convex vs triangle through CollideConvexVsTriangles / CastConvexVsTriangles, sphere vs triangle through
//! CollideSphereVsTriangles / CastSphereVsTriangles, the reversed triangle vs convex entries and triangle vs triangle).
//! The shapes (triangles, spheres and boxes) are built from their settings on both sides (`ShapeDesc`).
//!
//! Compared bit for bit on random inputs mixed with hand picked edge cases (degenerate triangles: collinear,
//! coincident vertices, a single point, tiny and needle triangles; convex radius or not; materials; scales with
//! negative components / inside out): the error results of invalid settings, GetLocalBounds, GetWorldSpaceBounds (Mat44
//! and DMat44), GetCenterOfMass, GetInnerRadius, GetMassProperties, GetVolume, GetStats, GetSubShapeIDBitsRecursive,
//! MustBeStatic, IsValidScale / MakeScaleValid, GetSurfaceNormal, GetSupportingFace, GetSubmergedVolume, GetLeafShape,
//! GetSubShapeUserData, GetMaterial, the support points of every ESupportMode with scales, CastRay (both overloads, the
//! AllHit / AnyHit / ClosestHit collectors, back face modes, early out fractions; rays through the interior, at the
//! vertices and edges, parallel to and in the plane of the triangle), CollidePoint, CollisionDispatch::sCollideShapeVsShape
//! and sCastShapeVsShapeWorldSpace for sphere / box / triangle vs triangle and triangle vs sphere / box (random
//! transforms and scales, max separation distances, tolerances, back face / active edge / collect faces modes, active
//! edge movement directions, shrunken shapes, deepest points, extra convex radius, early out fractions; every hit in
//! order and the final early out fraction), GetTrianglesStart / Next, CollideSoftBodyVertices and the binary state bytes
//! (and a restore). C ABI wrappers: ZoltParity/Physics/TriangleShapeReference.cpp.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const Allocator = std.mem.Allocator;
const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const BoxShapeSettings = zolt.BoxShapeSettings;
const CastRayCollector = zolt.CastRayCollector;
const CastShapeCollector = zolt.CastShapeCollector;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const Color = zolt.Color;
const CollidePointCollector = zolt.CollidePointCollector;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollideSoftBodyVertexIterator = zolt.CollideSoftBodyVertexIterator;
const CollisionDispatch = zolt.CollisionDispatch;
const ConvexShape = zolt.ConvexShape;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsMaterialSimple = zolt.PhysicsMaterialSimple;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const Ref = zolt.Ref;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastSettings = zolt.ShapeCastSettings;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TransformedShape = zolt.TransformedShape;
const TriangleShape = zolt.TriangleShape;
const TriangleShapeSettings = zolt.TriangleShapeSettings;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

/// The C++ reference functions, see TriangleShapeReference.cpp
const jolt = struct {
    extern fn jolt_triangle_shape_settings(desc: *const ShapeDesc, out_error: *[128]u8) c_int;
    extern fn jolt_triangle_shape_properties(desc: *const ShapeDesc, input: *const PropertiesInput, output: *PropertiesOutput) void;
    extern fn jolt_triangle_shape_support(desc: *const ShapeDesc, mode: c_int, scale: *const P, directions: [*]const f32, num_directions: c_int, out_points: [*]f32) f32;
    extern fn jolt_triangle_shape_cast_ray(desc: *const ShapeDesc, input: *const RayInput, output: *RayOutput) void;
    extern fn jolt_triangle_shape_collide_point(desc: *const ShapeDesc, point: *const P, creator: *const [2]u32) u32;
    extern fn jolt_triangle_shape_collide(input: *const CollideInput, output: *HitsOutput) void;
    extern fn jolt_triangle_shape_cast(input: *const CastInput, output: *HitsOutput) void;
    extern fn jolt_triangle_shape_triangles(desc: *const ShapeDesc, position: *const P, rotation: *const [4]f32, scale: *const P, max_triangles_requested: c_int, with_materials: c_int, out_counts: *[4]c_int, out_vertices: *[4 * 9]f32, out_default_material: *[4]c_int) c_int;
    extern fn jolt_triangle_shape_binary_state(desc: *const ShapeDesc, user_data: u64, out_bytes: [*]u8, capacity: u32, out_restored_bytes: [*]u8, out_restored_size: *u32) u32;
    extern fn jolt_triangle_shape_soft_body(desc: *const ShapeDesc, transform: *const [16]f32, scale: *const P, num_vertices: c_int, positions: [*]const f32, inv_masses: [*]const f32, io_penetrations: [*]f32, io_planes: [*]f32, io_indices: [*]c_int, colliding_shape_index: c_int) void;
};

/// Number of random inputs per test
const iterations = fw.iterations;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// Shape description, must match ShapeDesc in TriangleShapeReference.cpp
const ShapeDesc = extern struct {
    /// 0: TriangleShape, 1: SphereShape, 2: BoxShape
    kind: u32,
    /// TriangleShape
    v1: P = .{ 0, 0, 0 },
    v2: P = .{ 0, 0, 0 },
    v3: P = .{ 0, 0, 0 },
    /// TriangleShape, BoxShape
    convex_radius: f32 = 0.0,
    /// SphereShape
    radius: f32 = 0.0,
    /// BoxShape
    half_extent: P = .{ 0, 0, 0 },
    density: f32 = 1000.0,
    /// TriangleShape: 1 to use a material (otherwise the default material)
    material: c_int = 0,
};

/// Must match PropertiesInput in TriangleShapeReference.cpp
const PropertiesInput = extern struct {
    /// A valid scale (GetWorldSpaceBounds asserts)
    scale: P,
    /// IsValidScale / MakeScaleValid
    any_scale: P,
    transform: [16]f32,
    /// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
    translation: [3]f64,
    /// GetSurfaceNormal
    point: P,
    /// GetSupportingFace
    direction: P,
    /// GetSubmergedVolume
    plane: [4]f32,
    /// GetLeafShape, GetSubShapeUserData
    sub_shape_id: u32,
    user_data: u64,
};

/// Must match PropertiesOutput in TriangleShapeReference.cpp
const PropertiesOutput = extern struct {
    local_bounds: [6]f32,
    world_bounds: [6]f32,
    world_bounds_d: [6]f32,
    center_of_mass: P,
    inner_radius: f32,
    mass: f32,
    inertia: [16]f32,
    volume: f32,
    num_triangles: u32,
    sub_shape_id_bits: u32,
    is_valid_scale: c_int,
    scale_valid: P,
    is_any_scale_valid: c_int,
    any_scale_valid: P,
    surface_normal: P,
    face_count: u32,
    face: [32 * 3]f32,
    /// Total volume, submerged volume, center of buoyancy
    submerged: [5]f32,
    leaf_is_self: c_int,
    leaf_remainder: u32,
    sub_shape_user_data: u64,
    material_is_default: c_int,
    must_be_static: c_int,
};

/// Must match RayInput in TriangleShapeReference.cpp
const RayInput = extern struct {
    origin: P,
    direction: P,
    /// Sub shape ID creator: value pushed, number of bits
    creator: [2]u32,
    /// Initial fraction of the single hit version
    fraction: f32,
    /// 1: collide with back faces
    back_face_mode_triangles: c_int,
    /// 1: collide with back faces
    back_face_mode_convex: c_int,
    treat_convex_as_solid: c_int,
    /// 0: AllHit, 1: AnyHit, 2: ClosestHit
    collector: c_int,
    /// Early out fraction of the collector (when < the initial one)
    early_out: f32,
    /// Body ID of the collector context
    body_id: u32,
};

const RayHit = extern struct {
    fraction: f32,
    body_id: u32,
    sub_shape_id: u32,
};

/// Must match RayOutput in TriangleShapeReference.cpp
const RayOutput = extern struct {
    hit: c_int,
    fraction: f32,
    sub_shape_id: u32,
    num_hits: u32,
    hits: [4]RayHit,
};

/// Must match CollideInput in TriangleShapeReference.cpp
const CollideInput = extern struct {
    shape1: ShapeDesc,
    shape2: ShapeDesc,
    scale1: P,
    scale2: P,
    transform1: [16]f32,
    transform2: [16]f32,
    creator1: [2]u32,
    creator2: [2]u32,
    max_separation_distance: f32,
    collision_tolerance: f32,
    penetration_tolerance: f32,
    active_edge_movement_direction: P,
    /// 1: collide with all edges
    active_edge_mode: c_int,
    /// 1: collide with back faces
    back_face_mode: c_int,
    collect_faces: c_int,
    /// Early out fraction of the collector (when < FLT_MAX)
    early_out: f32,
    body_id: u32,
};

/// Must match HitOutput in TriangleShapeReference.cpp
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

/// Must match HitsOutput in TriangleShapeReference.cpp
const HitsOutput = extern struct {
    num_hits: u32,
    /// The early out fraction of the collector after the query
    early_out: f32,
    hits: [2]HitOutput,
};

/// Must match CastInput in TriangleShapeReference.cpp
const CastInput = extern struct {
    shape1: ShapeDesc,
    shape2: ShapeDesc,
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
    active_edge_movement_direction: P,
    /// 1: collide with all edges
    active_edge_mode: c_int,
    /// 1: collide with back faces
    back_face_mode_triangles: c_int,
    /// 1: collide with back faces
    back_face_mode_convex: c_int,
    use_shrunken_shape: c_int,
    return_deepest_point: c_int,
    collect_faces: c_int,
    /// Early out fraction of the collector (when < 1 + FLT_EPSILON)
    early_out: f32,
    body_id: u32,
};

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of the wrappers

/// Build the shape from its settings, null when the settings are invalid (the error text is copied to `out_error` when
/// given)
fn createShape(allocator: Allocator, desc: ShapeDesc, out_error: ?*[128]u8) !?Ref(Shape) {
    var result = switch (desc.kind) {
        0 => blk: {
            const material: ?*const PhysicsMaterial = if (desc.material != 0) (try PhysicsMaterialSimple.create(allocator, "Parity", Color.red)).material() else null;
            var settings = TriangleShapeSettings.init(allocator, vec3(desc.v1), vec3(desc.v2), vec3(desc.v3), .{ .convex_radius = desc.convex_radius, .material = material });
            defer settings.deinit();
            settings.base.density = desc.density;
            break :blk try settings.asShapeSettings().createShape(allocator);
        },
        1 => blk: {
            var settings = SphereShapeSettings.init(allocator, desc.radius, .{});
            defer settings.deinit();
            settings.base.density = desc.density;
            break :blk try settings.asShapeSettings().createShape(allocator);
        },
        else => blk: {
            var settings = BoxShapeSettings.init(allocator, vec3(desc.half_extent), .{ .convex_radius = desc.convex_radius });
            defer settings.deinit();
            settings.base.density = desc.density;
            break :blk try settings.asShapeSettings().createShape(allocator);
        },
    };
    defer result.deinit();
    if (result.hasError()) {
        if (out_error) |e| {
            @memset(e, 0);
            const text = result.getError();
            @memcpy(e[0..@min(text.len, 127)], text[0..@min(text.len, 127)]);
        }
        return null;
    }
    return Ref(Shape).init(result.getPtr());
}

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

fn boxArr(b: AABox) [6]f32 {
    return arr3(b.min) ++ arr3(b.max);
}

fn makeCreator(c: [2]u32) SubShapeIDCreator {
    return SubShapeIDCreator.pushID(.{}, c[0], c[1]);
}

fn backFaceMode(mode: c_int) zolt.BackFaceMode {
    return if (mode != 0) .collide_with_back_faces else .ignore_back_faces;
}

fn activeEdgeMode(mode: c_int) zolt.ActiveEdgeMode {
    return if (mode != 0) .collide_with_all else .collide_only_with_active;
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

fn zoltProperties(shape: *Shape, input: *const PropertiesInput) PropertiesOutput {
    shape.setUserData(input.user_data);
    var o = std.mem.zeroes(PropertiesOutput);
    const scale = vec3(input.scale);
    const any_scale = vec3(input.any_scale);
    const transform = mat44(input.transform);
    o.local_bounds = boxArr(shape.getLocalBounds());
    o.world_bounds = boxArr(shape.getWorldSpaceBounds(transform, scale));
    o.world_bounds_d = boxArr(shape.getWorldSpaceBoundsDMat44(DMat44.fromMat44Translation(transform, DVec3.init(input.translation[0], input.translation[1], input.translation[2])), scale));
    o.center_of_mass = arr3(shape.getCenterOfMass());
    o.inner_radius = shape.getInnerRadius();
    const p = shape.getMassProperties();
    o.mass = p.mass;
    o.inertia = arr16(p.inertia);
    o.volume = shape.getVolume();
    o.num_triangles = shape.getStats().num_triangles;
    o.sub_shape_id_bits = shape.getSubShapeIDBitsRecursive();
    o.is_valid_scale = @intFromBool(shape.isValidScale(scale));
    o.scale_valid = arr3(shape.makeScaleValid(scale));
    o.is_any_scale_valid = @intFromBool(shape.isValidScale(any_scale));
    o.any_scale_valid = arr3(shape.makeScaleValid(any_scale));
    o.surface_normal = arr3(shape.getSurfaceNormal(.empty, vec3(input.point)));
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, vec3(input.direction), scale, transform, &face);
    storeFace(&face, &o.face_count, &o.face);
    const submerged = shape.getSubmergedVolume(transform, scale, .fromVec4(vec4(input.plane)));
    o.submerged = [_]f32{ submerged.total_volume, submerged.submerged_volume } ++ arr3(submerged.center_of_buoyancy);
    var id: SubShapeID = .empty;
    id.setValue(input.sub_shape_id);
    const leaf = shape.getLeafShape(id);
    o.leaf_is_self = @intFromBool(leaf.shape == shape);
    o.leaf_remainder = leaf.remainder.getValue();
    o.sub_shape_user_data = shape.getSubShapeUserData(id);
    o.material_is_default = @intFromBool(shape.getMaterial(.empty) == PhysicsMaterial.default); // ConvexShape::GetMaterial asserts an empty ID
    o.must_be_static = @intFromBool(shape.mustBeStatic());
    return o;
}

fn zoltCastRay(allocator: Allocator, shape: *const Shape, input: *const RayInput) !RayOutput {
    var o = std.mem.zeroes(RayOutput);
    const ray = RayCast.init(vec3(input.origin), vec3(input.direction));
    const creator = makeCreator(input.creator);

    var hit: RayCastResult = .{};
    hit.fraction = input.fraction;
    o.hit = @intFromBool(shape.castRay(ray, creator, &hit));
    o.fraction = hit.fraction;
    o.sub_shape_id = hit.sub_shape_id2.getValue();

    var settings: RayCastSettings = .{};
    settings.back_face_mode_triangles = backFaceMode(input.back_face_mode_triangles);
    settings.back_face_mode_convex = backFaceMode(input.back_face_mode_convex);
    settings.treat_convex_as_solid = input.treat_convex_as_solid != 0;
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    const store = struct {
        fn f(out: *RayOutput, h: *const RayCastResult) void {
            out.hits[out.num_hits] = .{ .fraction = h.fraction, .body_id = h.body_id.getIndexAndSequenceNumber(), .sub_shape_id = h.sub_shape_id2.getValue() };
            out.num_hits += 1;
        }
    }.f;
    switch (input.collector) {
        0 => {
            var collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
            defer collector.deinit();
            collector.base.setContext(&context);
            if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
            shape.castRayCollector(ray, &settings, creator, &collector.base, &.{});
            try collector.checkError();
            for (collector.hits.items) |*h| store(&o, h);
        },
        1 => {
            var collector = AnyHitCollisionCollector(CastRayCollector).init();
            defer collector.deinit();
            collector.base.setContext(&context);
            if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
            shape.castRayCollector(ray, &settings, creator, &collector.base, &.{});
            if (collector.hadHit()) store(&o, &collector.hit);
        },
        else => {
            var collector = ClosestHitCollisionCollector(CastRayCollector).init();
            defer collector.deinit();
            collector.base.setContext(&context);
            if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
            shape.castRayCollector(ray, &settings, creator, &collector.base, &.{});
            if (collector.hadHit()) store(&o, &collector.hit);
        },
    }
    return o;
}

fn zoltCollide(allocator: Allocator, input: *const CollideInput) !HitsOutput {
    var shape1 = (try createShape(allocator, input.shape1, null)).?;
    defer shape1.deinit();
    var shape2 = (try createShape(allocator, input.shape2, null)).?;
    defer shape2.deinit();
    var settings: CollideShapeSettings = .{};
    settings.max_separation_distance = input.max_separation_distance;
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.active_edge_movement_direction = vec3(input.active_edge_movement_direction);
    settings.active_edge_mode = activeEdgeMode(input.active_edge_mode);
    settings.back_face_mode = backFaceMode(input.back_face_mode);
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    collector.base.setContext(&context);
    if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
    CollisionDispatch.collideShapeVsShape(shape1.get().?, shape2.get().?, vec3(input.scale1), vec3(input.scale2), mat44(input.transform1), mat44(input.transform2), makeCreator(input.creator1), makeCreator(input.creator2), &settings, &collector.base, &.{});
    try collector.checkError();
    var o = std.mem.zeroes(HitsOutput);
    o.num_hits = @intCast(collector.hits.items.len);
    o.early_out = collector.base.getEarlyOutFraction();
    for (collector.hits.items[0..@min(collector.hits.items.len, 2)], 0..) |*r, i| {
        o.hits[i].fraction = 0.0;
        o.hits[i].back_face = 0;
        storeCollideHit(r, &o.hits[i]);
    }
    return o;
}

fn zoltCast(allocator: Allocator, input: *const CastInput) !HitsOutput {
    var shape1 = (try createShape(allocator, input.shape1, null)).?;
    defer shape1.deinit();
    var shape2 = (try createShape(allocator, input.shape2, null)).?;
    defer shape2.deinit();
    var settings: ShapeCastSettings = .{};
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.extra_convex_radius = input.extra_convex_radius;
    settings.active_edge_movement_direction = vec3(input.active_edge_movement_direction);
    settings.active_edge_mode = activeEdgeMode(input.active_edge_mode);
    settings.back_face_mode_triangles = backFaceMode(input.back_face_mode_triangles);
    settings.back_face_mode_convex = backFaceMode(input.back_face_mode_convex);
    settings.use_shrunken_shape_and_convex_radius = input.use_shrunken_shape != 0;
    settings.return_deepest_point = input.return_deepest_point != 0;
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    const shape_cast = ShapeCast.init(shape1.get().?, vec3(input.scale1), mat44(input.start), vec3(input.direction));
    var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    collector.base.setContext(&context);
    if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
    CollisionDispatch.castShapeVsShapeWorldSpace(&shape_cast, &settings, shape2.get().?, vec3(input.scale2), &.{}, mat44(input.transform2), makeCreator(input.creator1), makeCreator(input.creator2), &collector.base);
    try collector.checkError();
    var o = std.mem.zeroes(HitsOutput);
    o.num_hits = @intCast(collector.hits.items.len);
    o.early_out = collector.base.getEarlyOutFraction();
    for (collector.hits.items[0..@min(collector.hits.items.len, 2)], 0..) |*r, i| {
        o.hits[i].fraction = r.fraction;
        o.hits[i].back_face = @intFromBool(r.is_back_face_hit);
        storeCollideHit(&r.base, &o.hits[i]);
    }
    return o;
}

// ---------------------------------------------------------------------------------------------------------------------
// Input generation

/// Values that tend to expose differences, mixed into the random inputs
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 1.0e-6, -1.0e-6, 1.0e-20, 3.0, -3.0 };

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

    /// Integer in [-n, n] as float
    fn grid(self: *Gen, n: i32) f32 {
        return @floatFromInt(self.rng.intRange(i32, -n, n));
    }

    fn gridVec(self: *Gen, n: i32) P {
        return .{ self.grid(n), self.grid(n), self.grid(n) };
    }

    /// A direction: random, axis aligned, zero, tiny
    fn direction(self: *Gen, length: f32) P {
        return switch (self.index(8)) {
            0 => .{ 0, 0, 0 },
            1 => blk: {
                var d: P = .{ 0, 0, 0 };
                d[self.index(3)] = if (self.oneIn(2)) length else -length;
                break :blk d;
            },
            2 => .{ self.grid(1), self.grid(1), self.grid(1) },
            3 => self.plainVec(-1.0e-6, 1.0e-6),
            else => self.vec(-length, length),
        };
    }

    /// The vertices of a triangle of about `size` around the origin: random, on a grid (axis aligned / exactly
    /// representable), degenerate (collinear, coincident vertices, a single point), tiny or a needle
    fn triangleVertices(self: *Gen, size: f32) [3]P {
        switch (self.index(12)) {
            0, 1 => {
                // On a grid, often axis aligned
                const g: i32 = @intFromFloat(@max(1.0, size));
                var v: [3]P = .{ self.gridVec(g), self.gridVec(g), self.gridVec(g) };
                if (self.oneIn(2)) {
                    const axis = self.index(3);
                    const c = self.grid(1);
                    for (&v) |*p| p[axis] = c;
                }
                return v;
            },
            2 => {
                // Collinear: v3 on the line through v1 and v2 (between, beyond or on a vertex)
                const v1 = self.plainVec(-size, size);
                const v2 = self.plainVec(-size, size);
                const t = ([_]f32{ 0.0, 0.5, 1.0, 2.0, -1.0 })[self.index(5)];
                const v3 = arr3(vec3(v1).add(vec3(v2).sub(vec3(v1)).mulScalar(t)));
                return switch (self.index(3)) {
                    0 => .{ v1, v2, v3 },
                    1 => .{ v3, v1, v2 },
                    else => .{ v2, v3, v1 },
                };
            },
            3 => {
                // Coincident vertices
                const a = self.plainVec(-size, size);
                const b = self.plainVec(-size, size);
                return switch (self.index(4)) {
                    0 => .{ a, a, b },
                    1 => .{ a, b, a },
                    2 => .{ b, a, a },
                    else => .{ a, a, a },
                };
            },
            4 => {
                // Tiny
                const c = vec3(self.plainVec(-size, size));
                return .{ arr3(c.add(vec3(self.plainVec(-1.0e-4, 1.0e-4)))), arr3(c.add(vec3(self.plainVec(-1.0e-4, 1.0e-4)))), arr3(c.add(vec3(self.plainVec(-1.0e-4, 1.0e-4)))) };
            },
            5 => {
                // Needle: one very long edge
                const a = self.plainVec(-size, size);
                const d = vec3(self.plainVec(-1, 1)).normalizedOr(Vec3.axisX()).mulScalar(50.0 * size);
                return .{ arr3(vec3(a).sub(d)), arr3(vec3(a).add(d)), arr3(vec3(a).add(vec3(self.plainVec(-0.05, 0.05)))) };
            },
            else => return .{ self.vec(-size, size), self.vec(-size, size), self.vec(-size, size) },
        }
    }

    /// A triangle shape (with or without convex radius / material)
    fn triangle(self: *Gen, size: f32) ShapeDesc {
        const v = self.triangleVertices(size);
        const convex_radius: f32 = switch (self.index(6)) {
            0, 1, 2 => 0.0,
            3 => 0.05,
            else => self.plain(0, 0.3),
        };
        return .{
            .kind = 0,
            .v1 = v[0],
            .v2 = v[1],
            .v3 = v[2],
            .convex_radius = convex_radius,
            .density = if (self.oneIn(4)) 1000.0 else self.plain(1, 3000),
            .material = @intFromBool(self.oneIn(4)),
        };
    }

    /// A sphere or a box (with or without convex radius, sometimes flat)
    fn convex(self: *Gen) ShapeDesc {
        const density = if (self.oneIn(4)) 1000.0 else self.plain(1, 3000);
        if (self.oneIn(2))
            return .{ .kind = 1, .radius = if (self.oneIn(5)) 1.0 else self.plain(0.05, 2), .density = density };
        var half_extent = self.plainVec(0.05, 2);
        if (self.oneIn(10)) half_extent[self.index(3)] = 0.0; // Flat box
        if (self.oneIn(5)) half_extent = .{ 1, 1, 1 };
        const convex_radius: f32 = switch (self.index(4)) {
            0 => 0.0,
            1 => 0.05, // cDefaultConvexRadius
            2 => self.plain(0, 5), // Bigger than the box: clamped
            else => self.plain(0, 0.2),
        };
        return .{ .kind = 2, .half_extent = half_extent, .convex_radius = convex_radius, .density = density };
    }

    /// A shape: a triangle half of the time, otherwise a sphere or a box
    fn shape(self: *Gen) ShapeDesc {
        return if (self.oneIn(2)) self.triangle(1.5) else self.convex();
    }

    /// A valid scale for the shape: uniform (with signs) for a sphere and a triangle with convex radius, anything non
    /// zero for the others
    fn scale(self: *Gen, desc: ShapeDesc) P {
        if (self.oneIn(5)) return .{ 1, 1, 1 };
        const uniform = desc.kind == 1 or (desc.kind == 0 and desc.convex_radius != 0.0);
        if (uniform) {
            const s = if (self.oneIn(4)) self.grid(2) else self.plain(0.2, 2.5);
            const m = if (s == 0.0) 1.0 else @abs(s);
            return .{ if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m };
        }
        var r = self.plainVec(0.2, 2.5);
        for (&r) |*c| {
            if (self.oneIn(3)) c.* = -c.*;
        }
        return r;
    }

    /// Any scale (for IsValidScale / MakeScaleValid): random, uniform, tiny / zero / negative components
    fn anyScale(self: *Gen) P {
        return switch (self.index(5)) {
            0 => self.vec(-3, 3),
            1 => blk: {
                const s = self.float(-3, 3);
                break :blk .{ s, if (self.oneIn(2)) s else -s, s };
            },
            2 => .{ 1.0 + self.plain(-1.0e-4, 1.0e-4), 1.0, 1.0 + self.plain(-1.0e-5, 1.0e-5) },
            3 => .{ self.float(-1.0e-5, 1.0e-5), self.plain(0.1, 2), self.plain(-2, -0.1) },
            else => .{ self.grid(2), self.grid(2), self.grid(2) },
        };
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
            if (len_sq > 1.0e-2 and len_sq <= 1.0) return quat(q).normalized();
        }
    }

    /// A rotation + translation matrix
    fn transform(self: *Gen, range: f32) Mat44 {
        return Mat44.rotationTranslation(self.rotation(), vec3(self.vec(-range, range)));
    }

    fn creator(self: *Gen) [2]u32 {
        const bits: u32 = @intCast(self.index(9));
        return .{ if (bits == 0) 0 else self.next() & ((@as(u32, 1) << @intCast(bits)) - 1), bits };
    }

    /// A point on the triangle: a vertex, on an edge or in the interior
    fn pointOnTriangle(self: *Gen, desc: ShapeDesc) Vec3 {
        const v1 = vec3(desc.v1);
        const v2 = vec3(desc.v2);
        const v3 = vec3(desc.v3);
        return switch (self.index(6)) {
            0 => ([_]Vec3{ v1, v2, v3 })[self.index(3)],
            1 => v1.add(v2.sub(v1).mulScalar(self.plain(0, 1))),
            2 => v2.add(v3.sub(v2).mulScalar(self.plain(0, 1))),
            3 => v3.add(v1.sub(v3).mulScalar(self.plain(0, 1))),
            else => blk: {
                var u = self.plain(0, 1);
                var v = self.plain(0, 1);
                if (u + v > 1.0) {
                    u = 1.0 - u;
                    v = 1.0 - v;
                }
                break :blk v1.add(v2.sub(v1).mulScalar(u)).add(v3.sub(v1).mulScalar(v));
            },
        };
    }

    /// An active edge movement direction: mostly zero, sometimes random or along an axis
    fn movementDirection(self: *Gen) P {
        return switch (self.index(4)) {
            0, 1 => .{ 0, 0, 0 },
            2 => self.direction(1),
            else => self.plainVec(-1, 1),
        };
    }
};

/// The extent of a shape around its center of mass (for placing shapes near each other)
fn extentOf(desc: ShapeDesc) f32 {
    return switch (desc.kind) {
        0 => blk: {
            var m: f32 = 0.0;
            for ([_]P{ desc.v1, desc.v2, desc.v3 }) |v| m = @max(m, vec3(v).length());
            break :blk @min(m, 3.0) + desc.convex_radius; // Needles are long, keep the distances near the center
        },
        1 => desc.radius,
        else => @max(desc.half_extent[0], @max(desc.half_extent[1], desc.half_extent[2])),
    };
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

test "TriangleShape parity: settings and Jolt's error texts" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "settings" };
    for (0..10_000) |i| {
        var desc = gen.triangle(2);
        desc.convex_radius = if (i < 4) ([_]f32{ 0.0, -0.0, -1.0e-30, -1.0 })[i] else gen.float(-0.5, 0.5);
        var jolt_error: [128]u8 = undefined;
        const jolt_valid = jolt.jolt_triangle_shape_settings(&desc, &jolt_error);
        var zolt_error: [128]u8 = @splat(0);
        var shape = try createShape(allocator, desc, &zolt_error);
        const zolt_valid: c_int = @intFromBool(shape != null);
        if (shape) |*s| s.deinit();
        checker.check(.{desc}, .{ zolt_valid, zolt_error }, .{ jolt_valid, jolt_error });
    }
    try checker.finish();
}

test "TriangleShape parity: bounds, mass properties, scales, surface normal, supporting face, submerged volume, leaf shape" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "properties" };
    for (0..iterations) |_| {
        const desc = gen.triangle(3);
        const normal = vec3(gen.direction(1)).normalizedOr(Vec3.axisY());
        const input: PropertiesInput = .{
            .scale = gen.scale(desc),
            .any_scale = gen.anyScale(),
            .transform = arr16(gen.transform(10)),
            .translation = .{ gen.plain(-1.0e5, 1.0e5), gen.plain(-10, 10), gen.plain(-10, 10) },
            .point = gen.vec(-4, 4),
            .direction = gen.direction(3),
            .plane = arr4(Plane.fromPointAndNormal(vec3(gen.vec(-5, 5)), normal).normal_and_constant),
            .sub_shape_id = if (gen.oneIn(2)) 0xffffffff else gen.next(),
            .user_data = (@as(u64, gen.next()) << 32) | gen.next(),
        };

        var jolt_output = std.mem.zeroes(PropertiesOutput);
        jolt.jolt_triangle_shape_properties(&desc, &input, &jolt_output);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const zolt_output = zoltProperties(shape.get().?, &input);
        checker.check(.{ desc, input }, zolt_output, jolt_output);
    }
    try checker.finish();
}

test "TriangleShape parity: support functions of every mode" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "support" };
    for (0..iterations / 4) |_| {
        const desc = gen.triangle(3);
        const scale = if (gen.oneIn(2)) gen.scale(desc) else gen.anyScale();
        var directions: [8 * 3]f32 = undefined;
        for (0..8) |d| directions[3 * d ..][0..3].* = gen.direction(3);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const convex = shape.get().?.cast(ConvexShape);
        for ([_]ConvexShape.SupportMode{ .exclude_convex_radius, .include_convex_radius, .default }) |mode| {
            var jolt_points: [8 * 3]f32 = @splat(0);
            const jolt_radius = jolt.jolt_triangle_shape_support(&desc, @intFromEnum(mode), &scale, &directions, 8, &jolt_points);
            var buffer: ConvexShape.SupportBuffer = .{};
            const support = convex.getSupportFunction(mode, &buffer, vec3(scale));
            var zolt_points: [8 * 3]f32 = @splat(0);
            for (0..8) |d| zolt_points[3 * d ..][0..3].* = arr3(support.getSupport(vec3(directions[3 * d ..][0..3].*)));
            checker.check(.{ desc, mode, scale, directions }, .{ support.getConvexRadius(), zolt_points }, .{ jolt_radius, jolt_points });
        }
    }
    try checker.finish();
}

test "TriangleShape parity: CastRay (single hit and collectors) and CollidePoint" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var rays: Checker = .{ .name = "cast ray" };
    var points: Checker = .{ .name = "collide point" };
    var num_hits: usize = 0;
    for (0..iterations) |i| {
        const desc = if (i < hand_picked_rays.len) xz_triangle else gen.triangle(2);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();

        var input: RayInput = .{
            .origin = gen.vec(-4, 4),
            .direction = gen.direction(8),
            .creator = gen.creator(),
            .fraction = if (gen.oneIn(2)) 1.0 + math.flt_epsilon else gen.plain(0, 1),
            .back_face_mode_triangles = @intFromBool(gen.oneIn(2)),
            .back_face_mode_convex = @intFromBool(gen.oneIn(2)),
            .treat_convex_as_solid = @intFromBool(!gen.oneIn(3)),
            .collector = @intCast(gen.index(3)),
            .early_out = if (gen.oneIn(2)) 2.0 else gen.plain(0, 1),
            .body_id = gen.next() & 0x7fffff,
        };
        const v1 = vec3(desc.v1);
        const normal = vec3(desc.v2).sub(v1).cross(vec3(desc.v3).sub(v1));
        switch (gen.index(8)) {
            0, 1, 2, 3 => {
                // Aim at a point of the triangle (interior, edge or vertex), from either side
                const target = gen.pointOnTriangle(desc);
                input.direction = arr3(target.sub(vec3(input.origin)).mulScalar(if (gen.oneIn(4)) 1.0 else gen.plain(0.5, 3)));
            },
            4 => {
                // Parallel to the plane of the triangle, sometimes in the plane
                const in_plane = normal.cross(vec3(gen.plainVec(-1, 1)));
                input.direction = arr3(in_plane.normalizedOr(Vec3.axisX()).mulScalar(gen.plain(0.5, 6)));
                if (gen.oneIn(2)) input.origin = arr3(gen.pointOnTriangle(desc).sub(vec3(input.direction).mulScalar(gen.plain(0, 1))));
            },
            5 => {
                // Starting on the triangle
                input.origin = arr3(gen.pointOnTriangle(desc));
            },
            else => {},
        }
        if (i < hand_picked_rays.len) {
            input.origin = hand_picked_rays[i][0];
            input.direction = hand_picked_rays[i][1];
        }
        var jolt_output = std.mem.zeroes(RayOutput);
        jolt.jolt_triangle_shape_cast_ray(&desc, &input, &jolt_output);
        const zolt_output = try zoltCastRay(allocator, shape.get().?, &input);
        num_hits += zolt_output.num_hits;
        rays.check(.{ desc, input }, zolt_output, jolt_output);

        // Points on and near the triangle: never inside
        const point = if (gen.oneIn(2)) arr3(gen.pointOnTriangle(desc)) else gen.vec(-3, 3);
        const jolt_count = jolt.jolt_triangle_shape_collide_point(&desc, &point, &input.creator);
        var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer collector.deinit();
        shape.get().?.collidePoint(vec3(point), makeCreator(input.creator), &collector.base, &.{});
        try collector.checkError();
        points.check(.{ desc, point }, @as(u32, @intCast(collector.hits.items.len)), jolt_count);
    }
    try finishAll(&.{ &rays, &points });
    try std.testing.expect(num_hits > iterations / 10 and num_hits < 4 * iterations / 5); // Hits and misses
}

/// True when the scaled triangle is (nearly) degenerate: CollideConvexVsTriangles uses the inverse triangle normal as
/// initial GJK axis and EPAPenetrationDepth::GetPenetrationDepthStepGJK asserts that it is not near zero (Jolt's debug
/// builds assert the same, its release builds continue), so these triangles only go to the sphere paths
fn isDegenerate(desc: ShapeDesc, scale: P) bool {
    const v1 = vec3(desc.v1).mul(vec3(scale));
    const v2 = vec3(desc.v2).mul(vec3(scale));
    const v3 = vec3(desc.v3).mul(vec3(scale));
    return v2.sub(v1).cross(v3.sub(v1)).lengthSq() <= 1.0e-8;
}

/// Random collide input: at least one of the shapes is a triangle, shape 2 is placed near shape 1
fn randomCollideInput(gen: *Gen) CollideInput {
    var shape1 = gen.shape();
    var shape2 = gen.shape();
    if (shape1.kind != 0 and shape2.kind != 0) {
        if (gen.oneIn(2)) shape1 = gen.triangle(1.5) else shape2 = gen.triangle(1.5);
    }
    var scale1 = gen.scale(shape1);
    var scale2 = gen.scale(shape2);

    // The triangle that CollideConvexVsTriangles gets (shape 2, or shape 1 in the reversed triangle vs box) must not be
    // degenerate, sphere vs triangle (also reversed) takes any triangle
    if (shape2.kind == 0 and shape1.kind != 1) {
        while (isDegenerate(shape2, scale2)) {
            shape2 = gen.triangle(1.5);
            scale2 = gen.scale(shape2);
        }
    } else if (shape1.kind == 0 and shape2.kind == 2) {
        while (isDegenerate(shape1, scale1)) {
            shape1 = gen.triangle(1.5);
            scale1 = gen.scale(shape1);
        }
    }

    const transform1 = gen.transform(5);
    const reach = extentOf(shape1) + extentOf(shape2);
    var relative = gen.transform(0.8 * reach);
    if (gen.oneIn(10)) relative.setTranslation(Vec3.zero()); // Same center
    if (gen.oneIn(10)) relative = Mat44.identity();
    return .{
        .shape1 = shape1,
        .shape2 = shape2,
        .scale1 = scale1,
        .scale2 = scale2,
        .transform1 = arr16(transform1),
        .transform2 = arr16(transform1.mul(relative)),
        .creator1 = gen.creator(),
        .creator2 = gen.creator(),
        .max_separation_distance = switch (gen.index(5)) {
            0, 1 => 0.0,
            2 => gen.plain(0, 1),
            3 => gen.plain(1, 3),
            else => 100.0,
        },
        .collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance,
        .penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance,
        .active_edge_movement_direction = gen.movementDirection(),
        .active_edge_mode = @intFromBool(gen.oneIn(2)),
        .back_face_mode = @intFromBool(gen.oneIn(2)),
        .collect_faces = @intFromBool(gen.oneIn(2)),
        .early_out = if (gen.oneIn(4)) gen.plain(-2, 2) else math.flt_max,
        .body_id = gen.next() & 0x7fffff,
    };
}

test "TriangleShape parity: collide sphere / box / triangle vs triangle and triangle vs sphere / box through CollisionDispatch" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "collide" };
    var num_hits: usize = 0;
    var pairs: [3][3]usize = @splat(@splat(0));
    for (0..iterations) |i| {
        var input = randomCollideInput(&gen);
        if (i < 6) {
            // A unit sphere / box exactly touching the triangle in the XZ plane from above, from below and at a vertex
            input.shape1 = if (i % 2 == 0) .{ .kind = 1, .radius = 1.0 } else .{ .kind = 2, .half_extent = .{ 1, 1, 1 }, .convex_radius = 0.0 };
            input.shape2 = .{ .kind = 0, .v1 = .{ -1, 0, -1 }, .v2 = .{ -1, 0, 1 }, .v3 = .{ 1, 0, -1 } };
            input.scale1 = .{ 1, 1, 1 };
            input.scale2 = .{ 1, 1, 1 };
            input.transform1 = arr16(Mat44.translation(Vec3.init(([_]f32{ -0.5, -0.5, -2.0 })[i / 2], ([_]f32{ 1, -1, 0 })[i / 2], -0.5)));
            input.transform2 = arr16(Mat44.identity());
            input.max_separation_distance = 0.0;
            input.early_out = math.flt_max;
        }
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_triangle_shape_collide(&input, &jolt_output);
        const zolt_output = try zoltCollide(allocator, &input);
        num_hits += @min(zolt_output.num_hits, 1);
        pairs[input.shape1.kind][input.shape2.kind] += 1;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > iterations / 5 and num_hits < 4 * iterations / 5); // Hits and misses
    for ([_][2]usize{ .{ 0, 0 }, .{ 0, 1 }, .{ 0, 2 }, .{ 1, 0 }, .{ 2, 0 } }) |pair| try std.testing.expect(pairs[pair[0]][pair[1]] > iterations / 20);
}

/// Hand picked rays (origin, direction) against the triangle in the XZ plane, with the random settings
const hand_picked_rays = [_][2]P{
    .{ .{ -0.5, 1, -0.5 }, .{ 0, -2, 0 } }, // Front face
    .{ .{ -0.5, -1, -0.5 }, .{ 0, 2, 0 } }, // Back face
    .{ .{ -3, 0, -0.5 }, .{ 6, 0, 0 } }, // In the plane
    .{ .{ -3, 1.0e-7, -0.5 }, .{ 6, -2.0e-7, 0 } }, // Grazing
    .{ .{ 1, 1, -1 }, .{ 0, -1, 0 } }, // Exactly at a vertex
    .{ .{ 0, 1, 0 }, .{ 0, -1, 0 } }, // Exactly on the diagonal edge
    .{ .{ -1, 1, 0 }, .{ 0, -1, 0 } }, // Exactly on an axis aligned edge
    .{ .{ -0.5, 0, -0.5 }, .{ 0, 1, 0 } }, // Starting on the triangle
    .{ .{ -0.5, 1, -0.5 }, .{ 0, -1, 0 } }, // Ending on the triangle
    .{ .{ -0.5, 1, -0.5 }, .{ 0, 0, 0 } }, // Zero direction
};

/// A hand picked cast (see the cast test)
const HandPickedCast = struct { shape1: ShapeDesc, shape2: ShapeDesc, start: P, direction: P };

/// The triangle of the hand picked tests, in the XZ plane with its normal along Y
const xz_triangle: ShapeDesc = .{ .kind = 0, .v1 = .{ -1, 0, -1 }, .v2 = .{ -1, 0, 1 }, .v3 = .{ 1, 0, -1 } };
const unit_sphere: ShapeDesc = .{ .kind = 1, .radius = 0.5 };
const unit_box: ShapeDesc = .{ .kind = 2, .half_extent = .{ 0.5, 0.5, 0.5 }, .convex_radius = 0.05 };

const hand_picked_casts = [_]HandPickedCast{
    // Front face, back face, from exactly touching, starting inside
    .{ .shape1 = unit_sphere, .shape2 = xz_triangle, .start = .{ -0.5, 2, -0.5 }, .direction = .{ 0, -4, 0 } },
    .{ .shape1 = unit_sphere, .shape2 = xz_triangle, .start = .{ -0.5, -2, -0.5 }, .direction = .{ 0, 4, 0 } },
    .{ .shape1 = unit_sphere, .shape2 = xz_triangle, .start = .{ -0.5, 0.5, -0.5 }, .direction = .{ 0, -1, 0 } },
    .{ .shape1 = unit_sphere, .shape2 = xz_triangle, .start = .{ -0.5, 0.25, -0.5 }, .direction = .{ 0, 1, 0 } },
    // Grazing: moving parallel to the plane at exactly the radius, in the plane
    .{ .shape1 = unit_sphere, .shape2 = xz_triangle, .start = .{ -3, 0.5, -0.5 }, .direction = .{ 6, 0, 0 } },
    .{ .shape1 = unit_sphere, .shape2 = xz_triangle, .start = .{ -3, 0, -0.5 }, .direction = .{ 6, 0, 0 } },
    // A vertex and an edge, head on
    .{ .shape1 = unit_sphere, .shape2 = xz_triangle, .start = .{ 3, 0, -1 }, .direction = .{ -4, 0, 0 } },
    .{ .shape1 = unit_sphere, .shape2 = xz_triangle, .start = .{ -3, 0, 0 }, .direction = .{ 4, 0, 0 } },
    .{ .shape1 = unit_box, .shape2 = xz_triangle, .start = .{ -0.5, 2, -0.5 }, .direction = .{ 0, -4, 0 } },
    .{ .shape1 = unit_box, .shape2 = xz_triangle, .start = .{ -3, 0, 0 }, .direction = .{ 4, 0, 0 } },
    .{ .shape1 = unit_box, .shape2 = xz_triangle, .start = .{ -0.5, 0.25, -0.5 }, .direction = .{ 0, 1, 0 } },
    // Triangle vs triangle: a vertical triangle with its tip down, the triangle cast into a box and a sphere (reversed)
    .{ .shape1 = .{ .kind = 0, .v1 = .{ -0.5, 1, 0 }, .v2 = .{ 0.5, 1, 0 }, .v3 = .{ 0, 0, 0 } }, .shape2 = xz_triangle, .start = .{ -0.5, 1, -0.5 }, .direction = .{ 0, -2, 0 } },
    .{ .shape1 = xz_triangle, .shape2 = unit_box, .start = .{ 0.25, -2, 0.25 }, .direction = .{ 0, 4, 0 } },
    .{ .shape1 = xz_triangle, .shape2 = unit_sphere, .start = .{ 0.25, -2, 0.25 }, .direction = .{ 0, 4, 0 } },
    // Jolt's TestCastSphereVsDegenerateTriangle (https://github.com/jrouwe/JoltPhysics/issues/886) through a TriangleShape
    .{ .shape1 = .{ .kind = 1, .radius = 0.2 }, .shape2 = .{ .kind = 0, .v1 = .{ 14.5536213, 10.5973721, -0.00600051880 }, .v2 = .{ 14.5536213, 10.5969315, -3.18638134 }, .v3 = .{ 14.5536213, 10.5969315, -5.18637228 } }, .start = .{ 14.8314590, 8.19055080, -4.30825043 }, .direction = .{ -0.0988006592, 5.96046448e-08, 0.000732421875 } },
};

test "TriangleShape parity: cast sphere / box / triangle vs triangle and triangle vs sphere / box through CollisionDispatch (world space)" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "cast" };
    var num_hits: usize = 0;
    for (0..iterations) |i| {
        var shape1 = gen.shape();
        var shape2 = gen.shape();
        if (shape1.kind != 0 and shape2.kind != 0) {
            if (gen.oneIn(2)) shape1 = gen.triangle(1.5) else shape2 = gen.triangle(1.5);
        }
        const scale1 = gen.scale(shape1);
        const scale2 = gen.scale(shape2);
        const transform2 = gen.transform(5);
        const reach = extentOf(shape1) + extentOf(shape2);
        // Start near shape 2 (sometimes overlapping), move towards it, past it or away from it
        var start = transform2.mul(gen.transform(2.0 * reach));
        if (gen.oneIn(8)) start.setTranslation(transform2.getTranslation().add(vec3(gen.plainVec(-0.2, 0.2))));
        var target = transform2.getTranslation();
        if (shape2.kind == 0 and gen.oneIn(2)) target = transform2.mulVec3(gen.pointOnTriangle(shape2).mul(vec3(scale2))); // Aim at a vertex, an edge or the interior
        const to_target = target.sub(start.getTranslation());
        const direction = switch (gen.index(4)) {
            0 => to_target.mulScalar(gen.plain(0.5, 3)),
            1 => to_target.mulScalar(gen.plain(-1, 0.2)),
            2 => vec3(gen.direction(3 * reach)),
            else => to_target.add(vec3(gen.plainVec(-reach, reach))).mulScalar(gen.plain(0.5, 2)),
        };
        var input: CastInput = .{
            .shape1 = shape1,
            .shape2 = shape2,
            .scale1 = scale1,
            .start = arr16(start),
            .direction = arr3(direction),
            .scale2 = scale2,
            .transform2 = arr16(transform2),
            .creator1 = gen.creator(),
            .creator2 = gen.creator(),
            .collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance,
            .penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance,
            .extra_convex_radius = if (gen.oneIn(4)) gen.plain(0, 0.3) else 0.0,
            .active_edge_movement_direction = gen.movementDirection(),
            .active_edge_mode = @intFromBool(gen.oneIn(2)),
            .back_face_mode_triangles = @intFromBool(gen.oneIn(2)),
            .back_face_mode_convex = @intFromBool(gen.oneIn(2)),
            .use_shrunken_shape = @intFromBool(gen.oneIn(2)),
            .return_deepest_point = @intFromBool(gen.oneIn(2)),
            .collect_faces = @intFromBool(gen.oneIn(2)),
            .early_out = if (gen.oneIn(4)) gen.plain(-0.5, 1) else 2.0,
            .body_id = gen.next() & 0x7fffff,
        };
        if (i < hand_picked_casts.len) {
            // Hand picked casts against the triangle in the XZ plane (normal Y), with the random settings
            const c = hand_picked_casts[i];
            input.shape1 = c.shape1;
            input.shape2 = c.shape2;
            input.scale1 = .{ 1, 1, 1 };
            input.scale2 = .{ 1, 1, 1 };
            input.start = arr16(Mat44.translation(vec3(c.start)));
            input.direction = c.direction;
            input.transform2 = arr16(Mat44.identity());
            input.early_out = 2.0;
        }
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_triangle_shape_cast(&input, &jolt_output);
        const zolt_output = try zoltCast(allocator, &input);
        num_hits += @min(zolt_output.num_hits, 1);
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > iterations / 10 and num_hits < 9 * iterations / 10); // Hits and misses
}

test "TriangleShape parity: GetTrianglesStart / Next" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "triangles" };
    for (0..iterations / 10) |_| {
        const desc = gen.triangle(3);
        const scale = if (gen.oneIn(2)) gen.scale(desc) else gen.anyScale();
        const position = gen.vec(-10, 10);
        const rotation = arr4(gen.rotation().getXYZW());
        const max_requested: c_int = if (gen.oneIn(2)) 32 else @intCast(32 + gen.index(100));
        const with_materials: c_int = @intFromBool(!gen.oneIn(4));

        var jolt_counts: [4]c_int = @splat(0);
        var jolt_vertices: [4 * 9]f32 = @splat(0);
        var jolt_default: [4]c_int = @splat(0);
        const jolt_calls = jolt.jolt_triangle_shape_triangles(&desc, &position, &rotation, &scale, max_requested, with_materials, &jolt_counts, &jolt_vertices, &jolt_default);

        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        var zolt_counts: [4]c_int = @splat(0);
        var zolt_vertices: [4 * 9]f32 = @splat(0);
        var zolt_default: [4]c_int = @splat(0);
        var context: Shape.GetTrianglesContext = .{};
        shape.get().?.getTrianglesStart(&context, AABox.biggest(), vec3(position), quat(rotation), vec3(scale));
        var triangles: [3 * 132]Float3 = undefined;
        var materials: [132]*const PhysicsMaterial = undefined;
        const requested: usize = @intCast(max_requested);
        var zolt_calls: c_int = 0;
        var num_triangles: usize = 0;
        while (true) {
            const count = shape.get().?.getTrianglesNext(&context, @intCast(max_requested), triangles[0 .. 3 * requested], if (with_materials != 0) materials[0..requested] else null);
            zolt_counts[@intCast(zolt_calls)] = @intCast(count);
            zolt_calls += 1;
            for (0..count) |t| {
                if (num_triangles < 4) {
                    for (0..3) |v| {
                        const f = triangles[3 * t + v];
                        zolt_vertices[9 * num_triangles + 3 * v ..][0..3].* = .{ f.x, f.y, f.z };
                    }
                    zolt_default[num_triangles] = if (with_materials == 0) 2 else @intFromBool(materials[t] == PhysicsMaterial.default);
                }
                num_triangles += 1;
            }
            if (count == 0 or zolt_calls == 4) break;
        }
        checker.check(.{ desc, scale, position, rotation, max_requested }, .{ zolt_calls, zolt_counts, zolt_vertices, zolt_default }, .{ jolt_calls, jolt_counts, jolt_vertices, jolt_default });
    }
    try checker.finish();
}

test "TriangleShape parity: CollideSoftBodyVertices" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "soft body vertices" };
    const n = 8;
    for (0..iterations / 4) |_| {
        const desc = gen.triangle(2);
        const scale = gen.scale(desc);
        const transform = arr16(gen.transform(3));
        var positions: [n * 3]f32 = undefined;
        var inv_masses: [n]f32 = undefined;
        var penetrations: [n]f32 = undefined;
        const planes: [n * 4]f32 = @splat(0);
        const indices: [n]c_int = @splat(-1);
        for (0..n) |v| {
            // Near the (scaled) triangle: around a point on it, sometimes exactly on it
            var local = gen.pointOnTriangle(desc).mul(vec3(scale));
            if (!gen.oneIn(8)) local = local.add(vec3(gen.vec(-0.5, 0.5)));
            if (gen.oneIn(8)) local = vec3(gen.vec(-5, 5));
            positions[3 * v ..][0..3].* = arr3(mat44(transform).mulVec3(local));
            inv_masses[v] = if (gen.oneIn(5)) 0.0 else 1.0;
            penetrations[v] = if (gen.oneIn(2)) -math.flt_max else gen.plain(-2, 2);
        }
        var jolt_penetrations = penetrations;
        var jolt_planes = planes;
        var jolt_indices = indices;
        jolt.jolt_triangle_shape_soft_body(&desc, &transform, &scale, n, &positions, &inv_masses, &jolt_penetrations, &jolt_planes, &jolt_indices, 3);

        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        var zolt_positions: [n]Vec3 = undefined;
        var zolt_planes: [n]Plane = undefined;
        var zolt_penetrations = penetrations;
        var zolt_indices: [n]i32 = undefined;
        for (0..n) |v| {
            zolt_positions[v] = vec3(positions[3 * v ..][0..3].*);
            zolt_planes[v] = .fromVec4(vec4(planes[4 * v ..][0..4].*));
            zolt_indices[v] = indices[v];
        }
        const vertices = CollideSoftBodyVertexIterator.init(.init(&zolt_positions[0], .{}), .init(&inv_masses[0], .{}), .init(&zolt_planes[0], .{}), .init(&zolt_penetrations[0], .{}), .init(&zolt_indices[0], .{}));
        shape.get().?.collideSoftBodyVertices(mat44(transform), vec3(scale), &vertices, n, 3);
        var zolt_plane_values: [n * 4]f32 = undefined;
        for (0..n) |v| zolt_plane_values[4 * v ..][0..4].* = arr4(zolt_planes[v].normal_and_constant);
        var zolt_index_values: [n]c_int = undefined;
        for (0..n) |v| zolt_index_values[v] = zolt_indices[v];
        checker.check(.{ desc, scale, transform, positions }, .{ zolt_penetrations, zolt_plane_values, zolt_index_values }, .{ jolt_penetrations, jolt_planes, jolt_indices });
    }
    try checker.finish();
}

test "TriangleShape parity: binary state" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "binary state" };
    for (0..iterations / 10) |_| {
        const desc = gen.triangle(3);
        const user_data = (@as(u64, gen.next()) << 32) | gen.next();
        var jolt_bytes: [128]u8 = @splat(0);
        var jolt_restored: [128]u8 = @splat(0);
        var jolt_restored_size: u32 = 0;
        const jolt_size = jolt.jolt_triangle_shape_binary_state(&desc, user_data, &jolt_bytes, 128, &jolt_restored, &jolt_restored_size);

        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        shape.get().?.setUserData(user_data);
        var zolt_bytes: [128]u8 = @splat(0);
        var writer: std.Io.Writer = .fixed(&zolt_bytes);
        var stream_out = StreamOutWrapper.init(&writer);
        shape.get().?.saveBinaryState(stream_out.streamOut());
        const zolt_size: u32 = @intCast(writer.buffered().len);

        // Restore Jolt's bytes and save again
        var zolt_restored: [128]u8 = @splat(0);
        var zolt_restored_size: u32 = 0;
        var reader: std.Io.Reader = .fixed(jolt_bytes[0..jolt_size]);
        var stream_in = StreamInWrapper.init(&reader);
        var result = try Shape.restoreFromBinaryState(allocator, stream_in.streamIn());
        defer result.deinit();
        if (result.isValid()) {
            var restored_writer: std.Io.Writer = .fixed(&zolt_restored);
            var restored_out = StreamOutWrapper.init(&restored_writer);
            result.getPtr().?.saveBinaryState(restored_out.streamOut());
            zolt_restored_size = @intCast(restored_writer.buffered().len);
        }
        checker.check(.{ desc, user_data }, .{ zolt_size, zolt_bytes, zolt_restored_size, zolt_restored }, .{ jolt_size, jolt_bytes, jolt_restored_size, jolt_restored });
    }
    try checker.finish();
}
