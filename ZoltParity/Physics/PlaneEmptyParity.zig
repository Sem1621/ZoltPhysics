//! Parity tests for PlaneShape and EmptyShape (Phase 4, Wave A), with SphereShape and BoxShape as the convex shapes
//! that collide with them. The shapes are built on both sides from a `ShapeDesc` (from their settings, or with the
//! shape constructor that takes the plane / center of mass directly). C ABI wrappers:
//! ZoltParity/Physics/PlaneEmptyReference.cpp.
//!
//! Compared bit for bit on random inputs mixed with edge cases (planes with random, axis aligned and nearly Y normals,
//! random constants and half extents including 0 and negative ones, scales with negative components, convex shapes
//! above / touching / penetrating / behind the plane, rays from both sides, parallel and grazing rays, rays and points
//! on the surface): the error texts of invalid settings (normals that are not normalized, at the tolerance),
//! GetLocalBounds, GetWorldSpaceBounds (Mat44 and DMat44), GetCenterOfMass, GetInnerRadius, GetMassProperties,
//! GetVolume, GetStats (triangles), GetSubShapeIDBitsRecursive, MustBeStatic, IsValidScale / MakeScaleValid,
//! GetSurfaceNormal, GetMaterial, GetSupportingFace, GetLeafShape, GetSubShapeUserData, GetSubShapeTransformedShape,
//! CastRay (both overloads, with the AllHit / AnyHit / ClosestHit collectors, back faces, solid or not, early out
//! fractions), CollidePoint, CollisionDispatch::sCollideShapeVsShape and sCastShapeVsShapeWorldSpace for convex vs
//! plane, plane vs convex (the reversed functions), and the empty shape against every shape (max separation distance,
//! faces, back faces, extra convex radius, early out fractions; all hits in order), GetSubmergedVolume (EmptyShape;
//! PlaneShape asserts "Not supported"), GetTrianglesStart / Next, CollideSoftBodyVertices, the binary state bytes (and
//! a restore) and SaveWithChildren / sRestoreWithChildren with the plane's material.

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
const CollidePointCollector = zolt.CollidePointCollector;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollideSoftBodyVertexIterator = zolt.CollideSoftBodyVertexIterator;
const CollisionDispatch = zolt.CollisionDispatch;
const Color = zolt.Color;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const EmptyShape = zolt.EmptyShape;
const EmptyShapeSettings = zolt.EmptyShapeSettings;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsMaterialSimple = zolt.PhysicsMaterialSimple;
const Plane = zolt.Plane;
const PlaneShape = zolt.PlaneShape;
const PlaneShapeSettings = zolt.PlaneShapeSettings;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const Real = zolt.Real;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
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
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

/// The C++ reference functions, see PlaneEmptyReference.cpp
const jolt = struct {
    extern fn jolt_plane_empty_settings(desc: *const ShapeDesc, out_error: *[128]u8) c_int;
    extern fn jolt_plane_empty_properties(desc: *const ShapeDesc, input: *const PropertiesInput, output: *PropertiesOutput) void;
    extern fn jolt_plane_empty_cast_ray(desc: *const ShapeDesc, input: *const RayInput, output: *RayOutput) void;
    extern fn jolt_plane_empty_collide_point(desc: *const ShapeDesc, point: *const P, creator: *const [2]u32, body_id: u32, out_ids: *[2]u32) u32;
    extern fn jolt_plane_empty_collide(input: *const CollideInput, output: *HitsOutput) void;
    extern fn jolt_plane_empty_cast(input: *const CastInput, output: *HitsOutput) void;
    extern fn jolt_plane_empty_submerged_volume(desc: *const ShapeDesc, transform: *const [16]f32, scale: *const P, plane: *const [4]f32, out_values: *[5]f32) void;
    extern fn jolt_plane_empty_triangles(desc: *const ShapeDesc, position: *const P, rotation: *const [4]f32, scale: *const P, max_triangles_requested: c_int, out_counts: *[4]c_int, out_vertices: *[max_vertices * 3]f32, out_default_material: *[max_vertices / 3]c_int) c_int;
    extern fn jolt_plane_empty_binary_state(desc: *const ShapeDesc, user_data: u64, out_bytes: [*]u8, capacity: u32, out_restored_bytes: [*]u8, out_restored_size: *u32) u32;
    extern fn jolt_plane_empty_save_with_children(desc: *const ShapeDesc, out_bytes: [*]u8, capacity: u32, out_restored_bytes: [*]u8, out_restored_size: *u32, out_material_is_default: *c_int) u32;
    extern fn jolt_plane_empty_soft_body(desc: *const ShapeDesc, transform: *const [16]f32, scale: *const P, num_vertices: c_int, positions: [*]const f32, inv_masses: [*]const f32, io_penetrations: [*]f32, io_planes: [*]f32, io_indices: [*]c_int, colliding_shape_index: c_int) void;
};

/// Number of random inputs per test
const iterations = fw.iterations;

/// The most triangle vertices a plane returns (2 triangles)
const max_vertices = 6;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// Shape kinds of ShapeDesc
const sphere_kind: u32 = 0;
const box_kind: u32 = 1;
const plane_kind: u32 = 2;
const empty_kind: u32 = 3;

/// Shape description, must match ShapeDesc in PlaneEmptyReference.cpp
const ShapeDesc = extern struct {
    /// 0: SphereShape, 1: BoxShape, 2: PlaneShape, 3: EmptyShape
    kind: u32,
    /// SphereShape
    radius: f32 = 0.0,
    /// BoxShape (PlaneShape: half_extent[0] is the half extent)
    half_extent: P = .{ 0, 0, 0 },
    /// BoxShape
    convex_radius: f32 = 0.0,
    /// PlaneShape: normal and constant
    plane: [4]f32 = .{ 0, 1, 0, 0 },
    /// EmptyShape
    center_of_mass: P = .{ 0, 0, 0 },
    /// PlaneShape: 1 = with the material "PlaneMaterial"
    material: c_int = 0,
    /// PlaneShape / EmptyShape: 1 = the shape constructor instead of the settings
    direct: c_int = 0,
};

/// Must match PropertiesInput in PlaneEmptyReference.cpp
const PropertiesInput = extern struct {
    scale: P,
    transform: [16]f32,
    /// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
    translation: [3]f64,
    /// GetSurfaceNormal
    point: P,
    /// GetSupportingFace
    direction: P,
    /// GetLeafShape, GetSubShapeUserData, GetSubShapeTransformedShape
    sub_shape_id: u32,
    /// GetSubShapeTransformedShape
    position: P,
    rotation: [4]f32,
    user_data: u64,
};

/// Must match PropertiesOutput in PlaneEmptyReference.cpp
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
    must_be_static: c_int,
    is_valid_scale: c_int,
    scale_valid: P,
    surface_normal: P,
    material_is_default: c_int,
    face_count: u32,
    face: [32 * 3]f32,
    leaf_is_self: c_int,
    leaf_remainder: u32,
    sub_shape_user_data: u64,
    ts_position: [3]Real,
    ts_rotation: [4]f32,
    ts_scale: P,
    ts_remainder: u32,
    ts_body_id: u32,
    ts_sub_shape_id: u32,
    ts_is_self: c_int,
};

/// Must match RayInput in PlaneEmptyReference.cpp
const RayInput = extern struct {
    origin: P,
    direction: P,
    /// Sub shape ID creator: value pushed, number of bits
    creator: [2]u32,
    /// Initial fraction of the single hit version
    fraction: f32,
    /// 1: collide with back faces (convex)
    back_face_mode: c_int,
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

/// Must match RayOutput in PlaneEmptyReference.cpp
const RayOutput = extern struct {
    hit: c_int,
    fraction: f32,
    sub_shape_id: u32,
    num_hits: u32,
    hits: [4]RayHit,
};

/// Must match CollideInput in PlaneEmptyReference.cpp
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
    collect_faces: c_int,
    /// Early out fraction of the collector (when < FLT_MAX)
    early_out: f32,
    body_id: u32,
};

/// Must match HitOutput in PlaneEmptyReference.cpp
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

/// Must match HitsOutput in PlaneEmptyReference.cpp
const HitsOutput = extern struct {
    num_hits: u32,
    hits: [2]HitOutput,
};

/// Must match CastInput in PlaneEmptyReference.cpp
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
    extra_convex_radius: f32,
    back_face_mode_convex: c_int,
    use_shrunken_shape: c_int,
    collect_faces: c_int,
    /// Early out fraction of the collector (when < 1 + FLT_EPSILON)
    early_out: f32,
    body_id: u32,
};

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of the wrappers

fn vec3(a: P) Vec3 {
    return Vec3.init(a[0], a[1], a[2]);
}

fn arr3(v: Vec3) P {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn arrR3(v: RVec3) [3]Real {
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

/// LoadPlane of the reference: Plane(normal, constant)
fn loadPlane(a: [4]f32) Plane {
    return Plane.init(Vec3.init(a[0], a[1], a[2]), a[3]);
}

fn makeCreator(c: [2]u32) SubShapeIDCreator {
    return SubShapeIDCreator.pushID(.{}, c[0], c[1]);
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

/// Build the shape (from its settings unless `direct`), null when the settings are invalid (the error text is copied
/// to `out_error` when given). A plane with a material gets its own PhysicsMaterialSimple("PlaneMaterial", red), which
/// is identical to the reference's.
fn createShape(allocator: Allocator, desc: ShapeDesc, out_error: ?*[128]u8) !?Ref(Shape) {
    var material: RefConst(PhysicsMaterial) = .empty;
    defer material.deinit();
    if (desc.kind == plane_kind and desc.material != 0)
        material = .init((try PhysicsMaterialSimple.create(allocator, "PlaneMaterial", Color.red)).material());

    var result = switch (desc.kind) {
        sphere_kind => blk: {
            var settings = SphereShapeSettings.init(allocator, desc.radius, .{});
            defer settings.deinit();
            break :blk try settings.asShapeSettings().createShape(allocator);
        },
        box_kind => blk: {
            var settings = BoxShapeSettings.init(allocator, vec3(desc.half_extent), .{ .convex_radius = desc.convex_radius });
            defer settings.deinit();
            break :blk try settings.asShapeSettings().createShape(allocator);
        },
        plane_kind => blk: {
            if (desc.direct != 0) {
                const shape = try PlaneShape.create(allocator, loadPlane(desc.plane), .{ .material = material.get(), .half_extent = desc.half_extent[0] });
                return Ref(Shape).init(shape.asShapeMut());
            }
            var settings = PlaneShapeSettings.init(allocator, loadPlane(desc.plane), .{ .material = material.get(), .half_extent = desc.half_extent[0] });
            defer settings.deinit();
            break :blk try settings.asShapeSettings().createShape(allocator);
        },
        else => blk: {
            if (desc.direct != 0) {
                const shape = try EmptyShape.create(allocator, vec3(desc.center_of_mass));
                return Ref(Shape).init(shape.asShapeMut());
            }
            var settings = EmptyShapeSettings.init(allocator, vec3(desc.center_of_mass));
            defer settings.deinit();
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

fn zoltProperties(shape: *Shape, input: *const PropertiesInput) PropertiesOutput {
    var o = std.mem.zeroes(PropertiesOutput);
    shape.setUserData(input.user_data);
    const scale = vec3(input.scale);
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
    o.must_be_static = @intFromBool(shape.mustBeStatic());
    o.is_valid_scale = @intFromBool(shape.isValidScale(scale));
    o.scale_valid = arr3(shape.makeScaleValid(scale));
    o.surface_normal = arr3(shape.getSurfaceNormal(.empty, vec3(input.point)));
    o.material_is_default = @intFromBool(shape.getMaterial(.empty) == PhysicsMaterial.default);
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, vec3(input.direction), scale, transform, &face);
    storeFace(&face, &o.face_count, &o.face);

    const id: SubShapeID = .{ .value = input.sub_shape_id };
    const leaf = shape.getLeafShape(id);
    o.leaf_is_self = @intFromBool(leaf.shape == shape);
    o.leaf_remainder = leaf.remainder.getValue();
    o.sub_shape_user_data = shape.getSubShapeUserData(id);
    var ts = shape.getSubShapeTransformedShape(id, vec3(input.position), quat(input.rotation), scale);
    defer ts.transformed_shape.deinit();
    o.ts_position = arrR3(ts.transformed_shape.shape_position_com);
    o.ts_rotation = arr4(ts.transformed_shape.shape_rotation.getXYZW());
    o.ts_scale = arr3(ts.transformed_shape.getShapeScale());
    o.ts_remainder = ts.remainder.getValue();
    o.ts_body_id = ts.transformed_shape.body_id.getIndexAndSequenceNumber();
    o.ts_sub_shape_id = ts.transformed_shape.sub_shape_id_creator.getID().getValue();
    o.ts_is_self = @intFromBool(ts.transformed_shape.shape.get() == shape);
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
    settings.back_face_mode_convex = if (input.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
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
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    collector.base.setContext(&context);
    if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
    CollisionDispatch.collideShapeVsShape(shape1.get().?, shape2.get().?, vec3(input.scale1), vec3(input.scale2), mat44(input.transform1), mat44(input.transform2), makeCreator(input.creator1), makeCreator(input.creator2), &settings, &collector.base, &.{});
    try collector.checkError();
    var o = std.mem.zeroes(HitsOutput);
    for (collector.hits.items) |*r| {
        if (o.num_hits < 2) {
            const h = &o.hits[o.num_hits];
            o.num_hits += 1;
            h.fraction = 0.0;
            h.back_face = 0;
            storeCollideHit(r, h);
        }
    }
    return o;
}

fn zoltCast(allocator: Allocator, input: *const CastInput) !HitsOutput {
    var shape1 = (try createShape(allocator, input.shape1, null)).?;
    defer shape1.deinit();
    var shape2 = (try createShape(allocator, input.shape2, null)).?;
    defer shape2.deinit();
    var settings: ShapeCastSettings = .{};
    settings.extra_convex_radius = input.extra_convex_radius;
    settings.back_face_mode_convex = if (input.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.use_shrunken_shape_and_convex_radius = input.use_shrunken_shape != 0;
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
    for (collector.hits.items) |*r| {
        if (o.num_hits < 2) {
            const h = &o.hits[o.num_hits];
            o.num_hits += 1;
            h.fraction = r.fraction;
            h.back_face = @intFromBool(r.is_back_face_hit);
            storeCollideHit(&r.base, h);
        }
    }
    return o;
}

/// SaveBinaryState of the shape into `buffer`, returns the bytes
fn saveState(shape: *const Shape, buffer: []u8) []const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamOutWrapper.init(&writer);
    shape.saveBinaryState(out.streamOut());
    return writer.buffered();
}

/// SaveWithChildren of the shape into `buffer` (fresh maps), returns the bytes
fn saveWithChildren(allocator: Allocator, shape: *const Shape, buffer: []u8) ![]const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamOutWrapper.init(&writer);
    var shape_map: Shape.ShapeToIDMap = .empty;
    defer shape_map.deinit(allocator);
    var material_map: Shape.MaterialToIDMap = .empty;
    defer material_map.deinit(allocator);
    try shape.saveWithChildren(allocator, out.streamOut(), &shape_map, &material_map);
    return writer.buffered();
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

    fn sign(self: *Gen) f32 {
        return if (self.oneIn(2)) 1.0 else -1.0;
    }

    /// A direction: random, axis aligned, zero, tiny
    fn direction(self: *Gen, length: f32) P {
        return switch (self.index(8)) {
            0 => .{ 0, 0, 0 },
            1 => blk: {
                var d: P = .{ 0, 0, 0 };
                d[self.index(3)] = self.sign() * length;
                break :blk d;
            },
            2 => .{ self.grid(1), self.grid(1), self.grid(1) },
            3 => self.plainVec(-1.0e-6, 1.0e-6),
            else => self.vec(-length, length),
        };
    }

    /// A unit plane normal: axis aligned, (nearly) Y (the cross product with Y is (almost) zero), from a grid, random
    fn normal(self: *Gen) Vec3 {
        switch (self.index(8)) {
            0, 1 => {
                var n: P = .{ 0, 0, 0 };
                n[self.index(3)] = self.sign();
                return vec3(n);
            },
            2 => return Vec3.init(self.plain(-1.0e-19, 1.0e-19), self.sign(), self.plain(-1.0e-19, 1.0e-19)).normalized(),
            3 => return Vec3.init(self.plain(-1.0e-3, 1.0e-3), self.sign(), self.plain(-1.0e-3, 1.0e-3)).normalized(),
            4 => return vec3(.{ self.grid(1), self.grid(1), self.grid(1) }).normalizedOr(Vec3.axisY()),
            else => return vec3(self.plainVec(-1, 1)).normalizedOr(Vec3.axisZ()),
        }
    }

    /// A valid plane shape (normal and constant, half extent, material, constructor)
    fn plane(self: *Gen) ShapeDesc {
        const n = self.normal();
        const constant = if (self.oneIn(3)) self.grid(3) else self.float(-5, 5);
        const half_extent: f32 = switch (self.index(6)) {
            0, 1 => PlaneShapeSettings.default_half_extent,
            2 => self.plain(0.1, 10),
            3 => self.plain(10, 1.0e4),
            4 => if (self.oneIn(2)) 0.0 else 1.0e-3,
            else => self.plain(-5, 0), // Not validated by Jolt
        };
        return .{ .kind = plane_kind, .plane = arr3(n) ++ [1]f32{constant}, .half_extent = .{ half_extent, 0, 0 }, .material = @intFromBool(self.oneIn(2)), .direct = @intFromBool(self.oneIn(3)) };
    }

    /// An empty shape
    fn empty(self: *Gen) ShapeDesc {
        return .{ .kind = empty_kind, .center_of_mass = if (self.oneIn(3)) .{ 0, 0, 0 } else self.vec(-5, 5), .direct = @intFromBool(self.oneIn(2)) };
    }

    /// A convex shape: sphere or box (with or without convex radius, sometimes flat)
    fn convex(self: *Gen) ShapeDesc {
        if (self.oneIn(2))
            return .{ .kind = sphere_kind, .radius = if (self.oneIn(5)) 1.0 else self.plain(0.05, 3) };
        var half_extent = self.plainVec(0.05, 3);
        if (self.oneIn(10)) half_extent[self.index(3)] = 0.0; // Flat box
        if (self.oneIn(5)) half_extent = .{ 1, 1, 1 };
        const convex_radius: f32 = switch (self.index(4)) {
            0 => 0.0,
            1 => 0.05, // cDefaultConvexRadius
            2 => self.plain(0, 5), // Bigger than the box: clamped
            else => self.plain(0, 0.2),
        };
        return .{ .kind = box_kind, .half_extent = half_extent, .convex_radius = convex_radius };
    }

    /// Any of the four shapes
    fn anyShape(self: *Gen) ShapeDesc {
        return switch (self.index(4)) {
            0 => self.plane(),
            1 => self.empty(),
            else => self.convex(),
        };
    }

    /// A valid scale for the shape: uniform (with signs) for a sphere, anything non zero for the others
    fn scale(self: *Gen, desc: ShapeDesc) P {
        if (self.oneIn(4)) return .{ 1, 1, 1 };
        if (desc.kind == sphere_kind) {
            const s = self.plain(0.2, 2.5);
            return .{ self.sign() * s, self.sign() * s, self.sign() * s };
        }
        if (self.oneIn(5)) return .{ self.sign(), self.sign(), self.sign() }; // Mirrors (inside out)
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

    /// A rotation + translation matrix (sometimes the identity or a translation only)
    fn transform(self: *Gen, range: f32) Mat44 {
        return switch (self.index(6)) {
            0 => Mat44.identity(),
            1 => Mat44.translation(vec3(self.vec(-range, range))),
            else => Mat44.rotationTranslation(self.rotation(), vec3(self.vec(-range, range))),
        };
    }

    fn creator(self: *Gen) [2]u32 {
        const bits: u32 = @intCast(self.index(9));
        return .{ if (bits == 0) 0 else self.next() & ((@as(u32, 1) << @intCast(bits)) - 1), bits };
    }

    /// A signed distance from a surface for a shape with extent `extent`: behind, penetrating, touching, near, far
    fn distance(self: *Gen, extent: f32) f32 {
        return switch (self.index(8)) {
            0 => extent, // Touching
            1 => 0.0, // Center on the surface
            2 => -extent, // Fully behind
            3 => extent + self.plain(-1.0e-3, 1.0e-3),
            4 => self.plain(-3 * extent - 1, -extent),
            else => self.plain(-extent, 3 * extent + 1),
        };
    }
};

/// The extent of a convex shape (for placing it near a plane), 1 for a plane or an empty shape
fn extentOf(desc: ShapeDesc, scale: P) f32 {
    const max_scale = @max(@abs(scale[0]), @max(@abs(scale[1]), @abs(scale[2])));
    return switch (desc.kind) {
        sphere_kind => desc.radius * max_scale,
        box_kind => @max(desc.half_extent[0], @max(desc.half_extent[1], desc.half_extent[2])) * max_scale,
        else => 1.0,
    };
}

/// The world space plane of a plane shape with scale and transform: a point on it, its normal and two tangents
const WorldPlane = struct {
    point: Vec3,
    normal: Vec3,
    tangent1: Vec3,
    tangent2: Vec3,

    fn init(desc: ShapeDesc, scale: P, transform: Mat44) WorldPlane {
        const p = loadPlane(desc.plane).scaled(vec3(scale)).getTransformed(transform);
        const n = p.getNormal();
        const t1 = n.getNormalizedPerpendicular();
        return .{ .point = n.mulScalar(-p.getConstant()), .normal = n, .tangent1 = t1, .tangent2 = n.cross(t1) };
    }

    /// A point at signed distance `d` from the plane, offset along the plane by (u, v)
    fn at(self: WorldPlane, d: f32, u: f32, v: f32) Vec3 {
        return self.point.add(self.normal.mulScalar(d)).add(self.tangent1.mulScalar(u)).add(self.tangent2.mulScalar(v));
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests

test "PlaneEmpty parity: settings and Jolt's error texts" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "settings" };
    for (0..10_000) |i| {
        var desc = if (gen.oneIn(5)) gen.empty() else gen.plane();
        desc.direct = 0;
        if (desc.kind == plane_kind) {
            // Normals that are not normalized, near the tolerance of IsNormalized and degenerate ones
            const n = vec3(desc.plane[0..3].*);
            const scaled: Vec3 = switch (gen.index(5)) {
                0 => n,
                1 => n.mulScalar(1.0 + @as(f32, @floatFromInt(gen.rng.intRange(i32, -10, 10))) * 1.0e-7),
                2 => vec3(gen.vec(-2, 2)),
                3 => Vec3.zero(),
                else => n.mulScalar(gen.plain(0.5, 2)),
            };
            desc.plane = arr3(scaled) ++ [1]f32{desc.plane[3]};
            if (i < 3) desc.plane = ([_][4]f32{ .{ 0, 0, 0, 0 }, .{ 0, 1.000001, 0, 1 }, .{ 0, 1.0000005, 0, 1 } })[i];
        }
        var jolt_error: [128]u8 = undefined;
        const jolt_valid = jolt.jolt_plane_empty_settings(&desc, &jolt_error);
        var zolt_error: [128]u8 = @splat(0);
        var shape = try createShape(allocator, desc, &zolt_error);
        const zolt_valid: c_int = @intFromBool(shape != null);
        if (shape) |*s| s.deinit();
        checker.check(.{desc}, .{ zolt_valid, zolt_error }, .{ jolt_valid, jolt_error });
    }
    try checker.finish();
}

test "PlaneEmpty parity: bounds, mass properties, scales, normal, material, supporting face, leaf and sub shapes" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "properties" };
    for (0..iterations) |_| {
        const desc = if (gen.oneIn(3)) gen.empty() else gen.plane();
        const input: PropertiesInput = .{
            .scale = if (gen.oneIn(2)) gen.scale(desc) else gen.anyScale(),
            .transform = arr16(gen.transform(10)),
            .translation = .{ gen.plain(-1.0e5, 1.0e5), gen.plain(-10, 10), gen.plain(-10, 10) },
            .point = gen.vec(-4, 4),
            .direction = gen.direction(3),
            .sub_shape_id = if (gen.oneIn(3)) SubShapeID.empty_value else gen.next(),
            .position = gen.vec(-10, 10),
            .rotation = arr4(gen.rotation().getXYZW()),
            .user_data = (@as(u64, gen.next()) << 32) | gen.next(),
        };
        var jolt_output = std.mem.zeroes(PropertiesOutput);
        jolt.jolt_plane_empty_properties(&desc, &input, &jolt_output);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const zolt_output = zoltProperties(shape.get().?, &input);
        checker.check(.{ desc, input }, zolt_output, jolt_output);
    }
    try checker.finish();
}

test "PlaneEmpty parity: CastRay (single hit and collectors) and CollidePoint" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var rays: Checker = .{ .name = "cast ray" };
    var points: Checker = .{ .name = "collide point" };
    var num_hits: usize = 0;
    for (0..iterations) |_| {
        const desc = if (gen.oneIn(6)) gen.empty() else gen.plane();
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const world = WorldPlane.init(if (desc.kind == plane_kind) desc else .{ .kind = plane_kind }, .{ 1, 1, 1 }, Mat44.identity());

        // Rays from both sides of the plane, from the surface, towards / away from / parallel to the plane
        const origin = if (gen.oneIn(8)) world.point else world.at(gen.distance(2), gen.float(-5, 5), gen.float(-5, 5));
        const length = gen.plain(0.1, 10);
        const direction: Vec3 = switch (gen.index(6)) {
            0 => world.normal.mulScalar(-length), // Towards the plane
            1 => world.normal.mulScalar(length), // Away from the plane
            2 => world.tangent1.mulScalar(length).add(world.tangent2.mulScalar(gen.float(-1, 1))), // Parallel (up to rounding)
            3 => world.normal.mulScalar(-gen.plain(1.0e-6, 1.0e-3)).add(world.tangent1.mulScalar(length)), // Grazing
            else => vec3(gen.direction(length)),
        };
        const input: RayInput = .{
            .origin = arr3(origin),
            .direction = arr3(direction),
            .creator = gen.creator(),
            .fraction = if (gen.oneIn(2)) 1.0 + math.flt_epsilon else gen.plain(0, 1),
            .back_face_mode = @intFromBool(gen.oneIn(2)),
            .treat_convex_as_solid = @intFromBool(!gen.oneIn(3)),
            .collector = @intCast(gen.index(3)),
            .early_out = switch (gen.index(4)) {
                0 => 0.0,
                1 => gen.plain(0, 1),
                else => 2.0,
            },
            .body_id = gen.next() & 0x7fffff,
        };
        var jolt_output = std.mem.zeroes(RayOutput);
        jolt.jolt_plane_empty_cast_ray(&desc, &input, &jolt_output);
        const zolt_output = try zoltCastRay(allocator, shape.get().?, &input);
        num_hits += zolt_output.num_hits;
        rays.check(.{ desc, input }, zolt_output, jolt_output);

        // Points on both sides, on the surface
        const point = arr3(if (gen.oneIn(4)) world.point.add(world.tangent1.mulScalar(gen.grid(3))) else world.at(gen.distance(1), gen.float(-5, 5), gen.float(-5, 5)));
        var jolt_ids: [2]u32 = undefined;
        const jolt_count = jolt.jolt_plane_empty_collide_point(&desc, &point, &input.creator, input.body_id, &jolt_ids);
        var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer collector.deinit();
        const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
        collector.base.setContext(&context);
        shape.get().?.collidePoint(vec3(point), makeCreator(input.creator), &collector.base, &.{});
        try collector.checkError();
        var zolt_ids: [2]u32 = .{ 0, 0 };
        for (collector.hits.items) |h| zolt_ids = .{ h.body_id.getIndexAndSequenceNumber(), h.sub_shape_id2.getValue() };
        points.check(.{ desc, point }, .{ @as(u32, @intCast(collector.hits.items.len)), zolt_ids }, .{ jolt_count, jolt_ids });
    }
    try finishAll(&.{ &rays, &points });
    try std.testing.expect(num_hits > iterations / 4 and num_hits < 2 * iterations); // Hits and misses
}

test "PlaneEmpty parity: collide convex vs plane, plane vs convex and empty vs anything through CollisionDispatch" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "collide" };
    var num_hits: usize = 0;
    var num_faces: usize = 0;
    for (0..iterations) |i| {
        // Pick the pair: convex vs plane, plane vs convex, or an empty shape with any shape
        const mode = gen.index(5);
        var plane_desc = gen.plane();
        var other = gen.convex();
        if (mode == 4) {
            plane_desc = gen.anyShape();
            other = gen.empty();
        }
        const plane_scale = gen.scale(plane_desc);
        const other_scale = gen.scale(other);
        const plane_transform = gen.transform(5);

        // Place the convex shape above / touching / penetrating / behind the plane
        const world = WorldPlane.init(if (plane_desc.kind == plane_kind) plane_desc else .{ .kind = plane_kind }, plane_scale, plane_transform);
        const extent = extentOf(other, other_scale);
        var other_transform = Mat44.rotationTranslation(gen.rotation(), world.at(gen.distance(extent), gen.float(-3, 3), gen.float(-3, 3)));
        if (gen.oneIn(10)) other_transform = Mat44.translation(world.at(gen.distance(extent), 0, 0)); // No rotation

        var input: CollideInput = .{
            .shape1 = other,
            .shape2 = plane_desc,
            .scale1 = other_scale,
            .scale2 = plane_scale,
            .transform1 = arr16(other_transform),
            .transform2 = arr16(plane_transform),
            .creator1 = gen.creator(),
            .creator2 = gen.creator(),
            .max_separation_distance = switch (gen.index(5)) {
                0, 1 => 0.0,
                2 => gen.plain(0, 1),
                3 => gen.plain(1, 3),
                else => 100.0,
            },
            .collect_faces = @intFromBool(gen.oneIn(2)),
            .early_out = if (gen.oneIn(4)) gen.plain(-2, 2) else math.flt_max,
            .body_id = gen.next() & 0x7fffff,
        };
        if (mode == 1 or mode == 3 or (mode == 4 and gen.oneIn(2))) {
            // Reversed order
            std.mem.swap(ShapeDesc, &input.shape1, &input.shape2);
            std.mem.swap(P, &input.scale1, &input.scale2);
            std.mem.swap([16]f32, &input.transform1, &input.transform2);
        }
        if (i < 6) {
            // Exactly touching unit spheres on the plane y = 0 (scaled planes and spheres)
            input.shape1 = .{ .kind = sphere_kind, .radius = 1.0 };
            input.shape2 = .{ .kind = plane_kind, .plane = .{ 0, 1, 0, 0 }, .half_extent = .{ 10, 0, 0 } };
            input.scale1 = .{ 1, 1, 1 };
            input.scale2 = if (i < 3) .{ 1, 1, 1 } else .{ 2, -1, 0.5 };
            input.transform1 = arr16(Mat44.translation(Vec3.init(0, @floatFromInt(i % 3), 0)));
            input.transform2 = arr16(Mat44.identity());
        }
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_plane_empty_collide(&input, &jolt_output);
        const zolt_output = try zoltCollide(allocator, &input);
        num_hits += zolt_output.num_hits;
        if (zolt_output.num_hits > 0 and zolt_output.hits[0].face2_count + zolt_output.hits[0].face1_count > 4) num_faces += 1;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > iterations / 4 and num_hits < 3 * iterations / 4); // Both paths are exercised
    try std.testing.expect(num_faces > iterations / 20); // Faces of the box and the plane
}

test "PlaneEmpty parity: cast convex vs plane, plane vs convex and empty vs anything through CollisionDispatch" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "cast" };
    var num_hits: usize = 0;
    var num_start_hits: usize = 0;
    for (0..iterations) |_| {
        const mode = gen.index(5);
        var plane_desc = gen.plane();
        var other = gen.convex();
        if (mode == 4) {
            plane_desc = gen.anyShape();
            other = gen.empty();
        }
        const plane_scale = gen.scale(plane_desc);
        const other_scale = gen.scale(other);
        const plane_transform = gen.transform(5);
        const world = WorldPlane.init(if (plane_desc.kind == plane_kind) plane_desc else .{ .kind = plane_kind }, plane_scale, plane_transform);
        const extent = extentOf(other, other_scale);
        const other_transform = Mat44.rotationTranslation(gen.rotation(), world.at(gen.distance(extent), gen.float(-3, 3), gen.float(-3, 3)));
        const length = gen.plain(0.5, 4) * (extent + 1);
        const direction: Vec3 = switch (gen.index(6)) {
            0, 1 => world.normal.mulScalar(-length).add(world.tangent1.mulScalar(gen.float(-1, 1))), // Towards the plane
            2 => world.normal.mulScalar(length), // Away from the plane
            3 => world.tangent1.mulScalar(length), // Parallel
            else => vec3(gen.direction(length)),
        };
        var input: CastInput = .{
            .shape1 = other,
            .shape2 = plane_desc,
            .scale1 = other_scale,
            .start = arr16(other_transform),
            .direction = arr3(direction),
            .scale2 = plane_scale,
            .transform2 = arr16(plane_transform),
            .creator1 = gen.creator(),
            .creator2 = gen.creator(),
            .extra_convex_radius = if (gen.oneIn(4)) gen.plain(0, 0.3) else 0.0,
            .back_face_mode_convex = @intFromBool(gen.oneIn(2)),
            .use_shrunken_shape = @intFromBool(gen.oneIn(2)),
            .collect_faces = @intFromBool(gen.oneIn(2)),
            .early_out = if (gen.oneIn(4)) gen.plain(-1, 1) else 2.0,
            .body_id = gen.next() & 0x7fffff,
        };
        if (mode == 1 or mode == 3 or (mode == 4 and gen.oneIn(2))) {
            // Cast the plane (or the other shape of the empty pair) against the convex shape: the reversed cast, moving
            // the plane in the opposite direction
            std.mem.swap(ShapeDesc, &input.shape1, &input.shape2);
            std.mem.swap(P, &input.scale1, &input.scale2);
            input.start = arr16(plane_transform);
            input.transform2 = arr16(other_transform);
            input.direction = arr3(direction.negate());
        }
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_plane_empty_cast(&input, &jolt_output);
        const zolt_output = try zoltCast(allocator, &input);
        num_hits += zolt_output.num_hits;
        if (zolt_output.num_hits > 0 and zolt_output.hits[0].fraction == 0.0) num_start_hits += 1;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > iterations / 5 and num_hits < 4 * iterations / 5); // Hits and misses
    try std.testing.expect(num_start_hits > iterations / 20); // Hits at fraction 0
}

test "PlaneEmpty parity: GetSubmergedVolume (EmptyShape) and GetTrianglesStart / Next" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var submerged: Checker = .{ .name = "submerged volume" };
    var triangles: Checker = .{ .name = "triangles" };
    for (0..iterations / 10) |_| {
        // EmptyShape::GetSubmergedVolume (PlaneShape's asserts "Not supported")
        {
            const desc = gen.empty();
            const scale = gen.anyScale();
            const transform = arr16(gen.transform(3));
            const surface = arr4(Plane.fromPointAndNormal(vec3(gen.vec(-5, 5)), gen.normal()).normal_and_constant);
            var jolt_values: [5]f32 = undefined;
            jolt.jolt_plane_empty_submerged_volume(&desc, &transform, &scale, &surface, &jolt_values);
            var shape = (try createShape(allocator, desc, null)).?;
            defer shape.deinit();
            const r = shape.get().?.getSubmergedVolume(mat44(transform), vec3(scale), .fromVec4(vec4(surface)));
            const zolt_values = [_]f32{ r.total_volume, r.submerged_volume } ++ arr3(r.center_of_buoyancy);
            submerged.check(.{ desc, scale, transform, surface }, zolt_values, jolt_values);
        }

        // GetTrianglesStart / Next with scales (inside out ones reverse the vertices)
        const desc = if (gen.oneIn(5)) gen.empty() else gen.plane();
        const scale = if (gen.oneIn(2)) gen.scale(desc) else gen.anyScale();
        const position = gen.vec(-10, 10);
        const rotation = arr4(gen.rotation().getXYZW());
        const max_requested: c_int = if (gen.oneIn(2)) 32 else @intCast(32 + gen.index(100));

        var jolt_counts: [4]c_int = @splat(0);
        var jolt_vertices: [max_vertices * 3]f32 = @splat(0);
        var jolt_default: [max_vertices / 3]c_int = @splat(0);
        const jolt_calls = jolt.jolt_plane_empty_triangles(&desc, &position, &rotation, &scale, max_requested, &jolt_counts, &jolt_vertices, &jolt_default);

        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        var zolt_counts: [4]c_int = @splat(0);
        var zolt_vertices: [max_vertices * 3]f32 = @splat(0);
        var zolt_default: [max_vertices / 3]c_int = @splat(0);
        var context: Shape.GetTrianglesContext = .{};
        shape.get().?.getTrianglesStart(&context, AABox.biggest(), vec3(position), quat(rotation), vec3(scale));
        var vertices: [3 * 132]Float3 = undefined;
        var materials: [132]*const PhysicsMaterial = undefined;
        var zolt_calls: c_int = 0;
        var out: usize = 0;
        var out_material: usize = 0;
        while (true) {
            const count = shape.get().?.getTrianglesNext(&context, @intCast(max_requested), vertices[0 .. 3 * @as(usize, @intCast(max_requested))], materials[0..@intCast(max_requested)]);
            zolt_counts[@intCast(zolt_calls)] = @intCast(count);
            zolt_calls += 1;
            for (vertices[0 .. 3 * count]) |t| {
                zolt_vertices[out..][0..3].* = .{ t.x, t.y, t.z };
                out += 3;
            }
            for (materials[0..count]) |m| {
                zolt_default[out_material] = @intFromBool(m == PhysicsMaterial.default);
                out_material += 1;
            }
            if (count == 0 or zolt_calls == 4) break;
        }
        triangles.check(.{ desc, scale, position, rotation, max_requested }, .{ zolt_calls, zolt_counts, zolt_vertices, zolt_default }, .{ jolt_calls, jolt_counts, jolt_vertices, jolt_default });
    }
    try finishAll(&.{ &submerged, &triangles });
}

test "PlaneEmpty parity: CollideSoftBodyVertices" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "soft body vertices" };
    const n = 8;
    for (0..iterations / 4) |_| {
        const desc = if (gen.oneIn(6)) gen.empty() else gen.plane();
        const scale = gen.scale(desc);
        const transform = gen.transform(3);
        const world = WorldPlane.init(if (desc.kind == plane_kind) desc else .{ .kind = plane_kind }, scale, transform);
        var positions: [n * 3]f32 = undefined;
        var inv_masses: [n]f32 = undefined;
        var penetrations: [n]f32 = undefined;
        const planes: [n * 4]f32 = @splat(0);
        const indices: [n]c_int = @splat(-1);
        for (0..n) |v| {
            positions[3 * v ..][0..3].* = arr3(world.at(gen.distance(1), gen.float(-5, 5), gen.float(-5, 5)));
            inv_masses[v] = if (gen.oneIn(5)) 0.0 else 1.0;
            penetrations[v] = if (gen.oneIn(2)) -math.flt_max else gen.plain(-2, 2);
        }
        const transform_arr = arr16(transform);
        var jolt_penetrations = penetrations;
        var jolt_planes = planes;
        var jolt_indices = indices;
        jolt.jolt_plane_empty_soft_body(&desc, &transform_arr, &scale, n, &positions, &inv_masses, &jolt_penetrations, &jolt_planes, &jolt_indices, 3);

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
        shape.get().?.collideSoftBodyVertices(transform, vec3(scale), &vertices, n, 3);
        var zolt_plane_values: [n * 4]f32 = undefined;
        for (0..n) |v| zolt_plane_values[4 * v ..][0..4].* = arr4(zolt_planes[v].normal_and_constant);
        var zolt_index_values: [n]c_int = undefined;
        for (0..n) |v| zolt_index_values[v] = zolt_indices[v];
        checker.check(.{ desc, scale, transform_arr, positions }, .{ zolt_penetrations, zolt_plane_values, zolt_index_values }, .{ jolt_penetrations, jolt_planes, jolt_indices });
    }
    try checker.finish();
}

test "PlaneEmpty parity: binary state and SaveWithChildren with the plane's material" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var state: Checker = .{ .name = "binary state" };
    var children: Checker = .{ .name = "save with children" };
    const capacity = 256;
    for (0..iterations / 10) |_| {
        const desc = if (gen.oneIn(3)) gen.empty() else gen.plane();
        const user_data = (@as(u64, gen.next()) << 32) | gen.next();

        // SaveBinaryState, sRestoreFromBinaryState of Jolt's bytes, SaveBinaryState of the restored shape
        {
            var jolt_bytes: [capacity]u8 = @splat(0);
            var jolt_restored: [capacity]u8 = @splat(0);
            var jolt_restored_size: u32 = 0;
            const jolt_size = jolt.jolt_plane_empty_binary_state(&desc, user_data, &jolt_bytes, capacity, &jolt_restored, &jolt_restored_size);

            var shape = (try createShape(allocator, desc, null)).?;
            defer shape.deinit();
            shape.get().?.setUserData(user_data);
            var zolt_bytes: [capacity]u8 = @splat(0);
            const zolt_size: u32 = @intCast(saveState(shape.get().?, &zolt_bytes).len);

            var zolt_restored: [capacity]u8 = @splat(0);
            var zolt_restored_size: u32 = 0;
            var reader: std.Io.Reader = .fixed(jolt_bytes[0..jolt_size]);
            var stream_in = StreamInWrapper.init(&reader);
            var result = try Shape.restoreFromBinaryState(allocator, stream_in.streamIn());
            defer result.deinit();
            if (result.isValid())
                zolt_restored_size = @intCast(saveState(result.getPtr().?, &zolt_restored).len);
            state.check(.{ desc, user_data }, .{ zolt_size, zolt_bytes, zolt_restored_size, zolt_restored }, .{ jolt_size, jolt_bytes, jolt_restored_size, jolt_restored });
        }

        // SaveWithChildren (the material of the plane), sRestoreWithChildren of Jolt's bytes, SaveWithChildren again
        {
            var jolt_bytes: [capacity]u8 = @splat(0);
            var jolt_restored: [capacity]u8 = @splat(0);
            var jolt_restored_size: u32 = 0;
            var jolt_material_is_default: c_int = 0;
            const jolt_size = jolt.jolt_plane_empty_save_with_children(&desc, &jolt_bytes, capacity, &jolt_restored, &jolt_restored_size, &jolt_material_is_default);

            var shape = (try createShape(allocator, desc, null)).?;
            defer shape.deinit();
            var zolt_bytes: [capacity]u8 = @splat(0);
            const zolt_size: u32 = @intCast((try saveWithChildren(allocator, shape.get().?, &zolt_bytes)).len);

            var zolt_restored: [capacity]u8 = @splat(0);
            var zolt_restored_size: u32 = 0;
            var zolt_material_is_default: c_int = 0;
            var reader: std.Io.Reader = .fixed(jolt_bytes[0..jolt_size]);
            var stream_in = StreamInWrapper.init(&reader);
            var id_to_shape: Shape.IDToShapeMap = .empty;
            defer {
                for (id_to_shape.items) |*s| s.deinit();
                id_to_shape.deinit(allocator);
            }
            var id_to_material: Shape.IDToMaterialMap = .empty;
            defer {
                for (id_to_material.items) |*m| m.deinit();
                id_to_material.deinit(allocator);
            }
            var result = try Shape.restoreWithChildren(allocator, stream_in.streamIn(), &id_to_shape, &id_to_material);
            defer result.deinit();
            if (result.isValid()) {
                zolt_material_is_default = @intFromBool(result.getPtr().?.getMaterial(.empty) == PhysicsMaterial.default);
                zolt_restored_size = @intCast((try saveWithChildren(allocator, result.getPtr().?, &zolt_restored)).len);
            }
            children.check(.{desc}, .{ zolt_size, zolt_bytes, zolt_restored_size, zolt_restored, zolt_material_is_default }, .{ jolt_size, jolt_bytes, jolt_restored_size, jolt_restored, jolt_material_is_default });
        }
    }
    try finishAll(&.{ &state, &children });
}
