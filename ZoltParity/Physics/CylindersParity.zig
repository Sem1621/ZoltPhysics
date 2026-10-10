//! Parity tests for the cylinders (Phase 4, Wave A): CylinderShape and TaperedCylinderShape. The shapes are built on
//! both sides from a `ShapeDesc`: CylinderShapeSettings, TaperedCylinderShapeSettings (equal radii create a
//! CylinderShape, a zero top or bottom radius is a cone), the CylinderShape constructor, and SphereShape / BoxShape as
//! collision partners. C ABI wrappers: ZoltParity/Physics/CylindersReference.cpp (functions prefixed jolt_cylinders_).
//!
//! Compared bit for bit on random inputs mixed with edge cases (degenerate sizes, convex radii that get clamped, rays
//! parallel to the caps / along the side / starting inside, touching shapes, scales with negative components): the
//! error results of invalid settings and the sub type / getters of valid ones, GetLocalBounds, GetWorldSpaceBounds
//! (Mat44 and DMat44), GetCenterOfMass, GetInnerRadius, GetMassProperties, GetVolume, GetStats (triangles),
//! GetSubShapeIDBitsRecursive, GetMaterial, GetLeafShape, GetSubShapeTransformedShape, IsValidScale / MakeScaleValid,
//! GetSurfaceNormal, GetSupportingFace, the support points of every ESupportMode with scales, CastRay (both overloads,
//! the analytic cylinder and ConvexShape's GJK fallback of the tapered cylinder, with the AllHit / AnyHit / ClosestHit
//! collectors), CollidePoint, CollisionDispatch::sCollideShapeVsShape and sCastShapeVsShapeWorldSpace against spheres,
//! boxes and cylinders (all hits in order, faces, max separation distance, back face / active edge modes, shrunken
//! shapes, deepest point, early out fractions), GetSubmergedVolume, GetTrianglesStart / Next, CollideSoftBodyVertices,
//! the binary state bytes (and a restore of them, also of truncated states: Jolt's error text).

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
const ConvexShape = zolt.ConvexShape;
const CylinderShape = zolt.CylinderShape;
const CylinderShapeSettings = zolt.CylinderShapeSettings;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const PhysicsMaterial = zolt.PhysicsMaterial;
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
const TaperedCylinderShape = zolt.TaperedCylinderShape;
const TaperedCylinderShapeSettings = zolt.TaperedCylinderShapeSettings;
const TransformedShape = zolt.TransformedShape;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

/// The C++ reference functions, see CylindersReference.cpp
const jolt = struct {
    extern fn jolt_cylinders_settings(desc: *const ShapeDesc, out_error: *[128]u8, out_sub_type: *u32, out_getters: *[5]f32) c_int;
    extern fn jolt_cylinders_properties(desc: *const ShapeDesc, input: *const PropertiesInput, scale_only: c_int, out_is_valid_scale: *c_int, out_scale_valid: *P, output: *PropertiesOutput) void;
    extern fn jolt_cylinders_support(desc: *const ShapeDesc, mode: c_int, scale: *const P, directions: [*]const f32, num_directions: c_int, out_points: [*]f32) f32;
    extern fn jolt_cylinders_cast_ray(desc: *const ShapeDesc, input: *const RayInput, output: *RayOutput) void;
    extern fn jolt_cylinders_collide_point(desc: *const ShapeDesc, point: *const P, creator: *const [2]u32, body_id: u32, out_ids: *[2]u32) u32;
    extern fn jolt_cylinders_collide(input: *const CollideInput, output: *HitsOutput) void;
    extern fn jolt_cylinders_cast(input: *const CastInput, output: *HitsOutput) void;
    extern fn jolt_cylinders_submerged_volume(desc: *const ShapeDesc, transform: *const [16]f32, scale: *const P, plane: *const [4]f32, out_values: *[5]f32) void;
    extern fn jolt_cylinders_triangles(desc: *const ShapeDesc, position: *const P, rotation: *const [4]f32, scale: *const P, max_triangles_requested: c_int, out_counts: *[8]c_int, out_vertices: *[max_vertices * 3]f32, out_default_material: *[max_vertices / 3]c_int) c_int;
    extern fn jolt_cylinders_binary_state(desc: *const ShapeDesc, user_data: u64, out_bytes: [*]u8, capacity: u32, out_restored_bytes: [*]u8, out_restored_size: *u32) u32;
    extern fn jolt_cylinders_restore(bytes: [*]const u8, size: u32, out_error: *[128]u8) c_int;
    extern fn jolt_cylinders_soft_body(desc: *const ShapeDesc, transform: *const [16]f32, scale: *const P, num_vertices: c_int, positions: [*]const f32, inv_masses: [*]const f32, io_penetrations: [*]f32, io_planes: [*]f32, io_indices: [*]c_int, colliding_shape_index: c_int) void;
};

/// Number of random inputs per test
const iterations = fw.iterations;

/// The most triangle vertices a shape returns (the cylinder: 32 triangles)
const max_vertices = 96;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// Shape kinds of ShapeDesc
const cylinder_settings_kind = 0;
const tapered_cylinder_settings_kind = 1;
const cylinder_constructor_kind = 2;
const sphere_kind = 3;
const box_kind = 4;

/// Shape description, must match ShapeDesc in CylindersReference.cpp
const ShapeDesc = extern struct {
    /// 0: CylinderShapeSettings, 1: TaperedCylinderShapeSettings, 2: CylinderShape constructor, 3: SphereShapeSettings,
    /// 4: BoxShapeSettings
    kind: u32,
    /// Cylinder, tapered cylinder
    half_height: f32 = 0.0,
    /// Cylinder, sphere
    radius: f32 = 0.0,
    /// Tapered cylinder
    top_radius: f32 = 0.0,
    bottom_radius: f32 = 0.0,
    /// Box
    half_extent: P = .{ 0, 0, 0 },
    /// Cylinder, tapered cylinder, box
    convex_radius: f32 = 0.0,
    density: f32 = 1000.0,

    fn isCylinder(self: ShapeDesc) bool {
        return self.kind <= cylinder_constructor_kind;
    }
};

/// Must match PropertiesInput in CylindersReference.cpp
const PropertiesInput = extern struct {
    scale: P,
    transform: [16]f32,
    /// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
    translation: [3]f64,
    /// GetSurfaceNormal
    point: P,
    /// GetSupportingFace
    direction: P,
    /// GetLeafShape, GetSubShapeTransformedShape
    sub_shape_id: u32,
    /// GetSubShapeTransformedShape
    position: P,
    rotation: [4]f32,
};

/// Must match PropertiesOutput in CylindersReference.cpp
const PropertiesOutput = extern struct {
    sub_type: u32,
    getters: [5]f32,
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
    material_is_default: c_int,
    leaf_is_self: c_int,
    leaf_remainder: u32,
    sub_position: [3]f64,
    sub_rotation: [4]f32,
    sub_scale: P,
    sub_is_self: c_int,
    sub_remainder: u32,
    surface_normal: P,
    face_count: u32,
    face: [32 * 3]f32,
};

/// Must match RayInput in CylindersReference.cpp
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

/// Must match RayOutput in CylindersReference.cpp
const RayOutput = extern struct {
    hit: c_int,
    fraction: f32,
    sub_shape_id: u32,
    num_hits: u32,
    hits: [4]RayHit,
};

/// Must match CollideInput in CylindersReference.cpp
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
    collect_faces: c_int,
    back_face_mode: c_int,
    /// 0: collide only with active, 1: collide with all
    active_edge_mode: c_int,
    active_edge_movement_direction: P,
    /// Early out fraction of the collector (when < FLT_MAX)
    early_out: f32,
    body_id: u32,
};

/// Must match HitOutput in CylindersReference.cpp
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

/// Must match HitsOutput in CylindersReference.cpp
const HitsOutput = extern struct {
    num_hits: u32,
    hits: [2]HitOutput,
};

/// Must match CastInput in CylindersReference.cpp
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
    back_face_mode_triangles: c_int,
    back_face_mode_convex: c_int,
    active_edge_mode: c_int,
    use_shrunken_shape: c_int,
    return_deepest_point: c_int,
    collect_faces: c_int,
    /// Early out fraction of the collector (when < 1 + FLT_EPSILON)
    early_out: f32,
    body_id: u32,
};

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of the wrappers

/// Build the shape, null when the settings are invalid (the error text is copied to `out_error` when given)
fn createShape(allocator: Allocator, desc: ShapeDesc, out_error: ?*[128]u8) !?Ref(Shape) {
    var result = switch (desc.kind) {
        cylinder_settings_kind => blk: {
            var settings = CylinderShapeSettings.init(allocator, desc.half_height, desc.radius, .{ .convex_radius = desc.convex_radius });
            defer settings.deinit();
            settings.base.density = desc.density;
            break :blk try settings.asShapeSettings().createShape(allocator);
        },
        tapered_cylinder_settings_kind => blk: {
            var settings = TaperedCylinderShapeSettings.init(allocator, desc.half_height, desc.top_radius, desc.bottom_radius, .{ .convex_radius = desc.convex_radius });
            defer settings.deinit();
            settings.base.density = desc.density;
            break :blk try settings.asShapeSettings().createShape(allocator);
        },
        cylinder_constructor_kind => {
            const shape = try CylinderShape.create(allocator, desc.half_height, desc.radius, .{ .convex_radius = desc.convex_radius });
            shape.base.setDensity(desc.density);
            return Ref(Shape).init(shape.asShapeMut());
        },
        sphere_kind => blk: {
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
        if (out_error) |e| copyError(result.getError(), e);
        return null;
    }
    return Ref(Shape).init(result.getPtr());
}

fn copyError(text: []const u8, out: *[128]u8) void {
    @memset(out, 0);
    const n = @min(text.len, 127);
    @memcpy(out[0..n], text[0..n]);
}

/// The getters of the shape (StoreGetters in CylindersReference.cpp)
fn getters(shape: *const Shape) [5]f32 {
    var g: [5]f32 = @splat(0);
    switch (shape.getSubType()) {
        .cylinder => {
            const cylinder = shape.cast(CylinderShape);
            g[0] = cylinder.getHalfHeight();
            g[1] = cylinder.getRadius();
            g[2] = cylinder.getConvexRadius();
        },
        .tapered_cylinder => {
            const cylinder = shape.cast(TaperedCylinderShape);
            g[0] = cylinder.getHalfHeight();
            g[1] = cylinder.getTopRadius();
            g[2] = cylinder.getBottomRadius();
            g[3] = cylinder.getConvexRadius();
        },
        else => {},
    }
    g[4] = shape.cast(ConvexShape).getDensity();
    return g;
}

fn vec3(a: P) Vec3 {
    return Vec3.init(a[0], a[1], a[2]);
}

fn arr3(v: Vec3) P {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn arrR3(v: RVec3) [3]f64 {
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

fn zoltProperties(shape: *const Shape, input: *const PropertiesInput) PropertiesOutput {
    var o = std.mem.zeroes(PropertiesOutput);
    const scale = vec3(input.scale);
    const transform = mat44(input.transform);
    o.sub_type = @intFromEnum(shape.getSubType());
    o.getters = getters(shape);
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
    const id: SubShapeID = .{ .value = input.sub_shape_id };
    o.material_is_default = @intFromBool(shape.getMaterial(.empty) == PhysicsMaterial.default); // A convex shape asserts an empty ID
    const leaf = shape.getLeafShape(id);
    o.leaf_is_self = @intFromBool(leaf.shape == shape);
    o.leaf_remainder = leaf.remainder.getValue();
    var sub = shape.getSubShapeTransformedShape(id, vec3(input.position), quat(input.rotation), scale);
    defer sub.transformed_shape.deinit();
    o.sub_position = arrR3(sub.transformed_shape.shape_position_com);
    o.sub_rotation = arr4(sub.transformed_shape.shape_rotation.getXYZW());
    o.sub_scale = arr3(sub.transformed_shape.getShapeScale());
    o.sub_is_self = @intFromBool(sub.transformed_shape.shape.get() == shape);
    o.sub_remainder = sub.remainder.getValue();
    o.surface_normal = arr3(shape.getSurfaceNormal(.empty, vec3(input.point)));
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, vec3(input.direction), scale, transform, &face);
    storeFace(&face, &o.face_count, &o.face);
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
            for (collector.hits.items) |*h| {
                if (o.num_hits < 4) store(&o, h);
            }
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
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    settings.back_face_mode = if (input.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.active_edge_mode = if (input.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.active_edge_movement_direction = vec3(input.active_edge_movement_direction);
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
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.extra_convex_radius = input.extra_convex_radius;
    settings.back_face_mode_triangles = if (input.back_face_mode_triangles != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.back_face_mode_convex = if (input.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.active_edge_mode = if (input.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
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

    fn sign(self: *Gen, v: f32) f32 {
        return if (self.oneIn(2)) v else -v;
    }

    /// A direction: random, axis aligned, zero, tiny, horizontal, nearly vertical (the 5 degree rule of the supporting
    /// faces)
    fn direction(self: *Gen, length: f32) P {
        return switch (self.index(10)) {
            0 => .{ 0, 0, 0 },
            1 => blk: {
                var d: P = .{ 0, 0, 0 };
                d[self.index(3)] = self.sign(length);
                break :blk d;
            },
            2 => .{ self.grid(1), self.grid(1), self.grid(1) },
            3 => self.plainVec(-1.0e-6, 1.0e-6),
            4 => .{ self.plain(-length, length), 0, self.plain(-length, length) },
            5 => .{ self.plain(-0.1, 0.1), self.sign(1), self.plain(-0.1, 0.1) },
            else => self.vec(-length, length),
        };
    }

    /// A convex radius: zero, the default, bigger than the shape (clamped), small
    fn convexRadius(self: *Gen) f32 {
        return switch (self.index(5)) {
            0 => 0.0,
            1 => 0.05, // cDefaultConvexRadius
            2 => self.plain(0, 5), // Bigger than the shape: clamped
            3 => self.plain(0, 0.2),
            else => self.plain(0, 0.02),
        };
    }

    /// A radius: random, sometimes 0, tiny (around the 1e-3 minimum radius of the caps) or 1
    fn radius(self: *Gen) f32 {
        return switch (self.index(10)) {
            0 => 0.0,
            1 => self.plain(0, 2.0e-3),
            2 => 1.0,
            else => self.plain(0.05, 3),
        };
    }

    /// A cylinder: CylinderShapeSettings, TaperedCylinderShapeSettings (cones, equal radii) or the CylinderShape
    /// constructor
    fn cylinder(self: *Gen) ShapeDesc {
        const density = if (self.oneIn(4)) 1000.0 else self.plain(1, 3000);
        var desc: ShapeDesc = .{ .kind = @intCast(self.index(3)), .convex_radius = self.convexRadius(), .density = density };
        switch (desc.kind) {
            tapered_cylinder_settings_kind => {
                desc.half_height = if (self.oneIn(5)) 1.0 else self.plain(0.05, 3);
                desc.top_radius = self.radius();
                desc.bottom_radius = self.radius();
                switch (self.index(8)) {
                    0 => desc.top_radius = 0.0, // Cone with the tip at the top
                    1 => desc.bottom_radius = 0.0, // Cone with the tip at the bottom
                    2 => desc.bottom_radius = desc.top_radius, // Equal radii: a CylinderShape
                    else => {},
                }
                if (desc.top_radius == 0.0 and desc.bottom_radius == 0.0) desc.bottom_radius = 0.5; // Not a line
            },
            else => {
                desc.half_height = switch (self.index(10)) {
                    0 => 0.0, // Disc
                    1 => 1.0,
                    else => self.plain(0.05, 3),
                };
                desc.radius = self.radius();
                if (desc.half_height == 0.0 and desc.radius == 0.0) desc.radius = 0.5; // Not a point
            },
        }
        return desc;
    }

    /// A shape: mostly cylinders, sometimes a sphere or a box
    fn shape(self: *Gen) ShapeDesc {
        switch (self.index(5)) {
            0 => return .{ .kind = sphere_kind, .radius = if (self.oneIn(5)) 1.0 else self.plain(0.05, 3), .density = 1000.0 },
            1 => {
                var half_extent = self.plainVec(0.05, 3);
                if (self.oneIn(10)) half_extent[self.index(3)] = 0.0; // Flat box
                if (self.oneIn(5)) half_extent = .{ 1, 1, 1 };
                return .{ .kind = box_kind, .half_extent = half_extent, .convex_radius = if (self.oneIn(3)) 0.05 else self.plain(0, 0.2), .density = 1000.0 };
            },
            else => return self.cylinder(),
        }
    }

    /// A valid scale for the shape: uniform in XZ (with signs) for a cylinder, uniform for a sphere, anything non zero for
    /// a box
    fn scale(self: *Gen, desc: ShapeDesc) P {
        if (self.oneIn(5)) return .{ 1, 1, 1 };
        const s = if (self.oneIn(4)) self.grid(2) else self.plain(0.2, 2.5);
        const m = if (s == 0.0) 1.0 else @abs(s);
        return switch (desc.kind) {
            sphere_kind => .{ self.sign(m), self.sign(m), self.sign(m) },
            box_kind => blk: {
                var r = self.plainVec(0.2, 2.5);
                for (&r) |*c| c.* = self.sign(c.*);
                break :blk r;
            },
            else => .{ self.sign(m), self.sign(if (self.oneIn(3)) m else self.plain(0.2, 2.5)), self.sign(m) },
        };
    }

    /// Any scale (for IsValidScale / MakeScaleValid): random, uniform in XZ, nearly uniform in XZ, tiny / zero /
    /// negative components
    fn anyScale(self: *Gen) P {
        return switch (self.index(6)) {
            0 => self.vec(-3, 3),
            1 => blk: {
                const s = self.float(-3, 3);
                break :blk .{ s, self.float(-3, 3), self.sign(s) };
            },
            2 => .{ 1.0 + self.plain(-1.0e-4, 1.0e-4), self.plain(-2, 2), self.sign(1.0 + self.plain(-1.0e-5, 1.0e-5)) },
            3 => .{ self.float(-1.0e-5, 1.0e-5), self.plain(0.1, 2), self.plain(-2, -0.1) },
            4 => .{ self.plain(0.1, 2), self.float(-1.0e-5, 1.0e-5), self.plain(0.1, 2) },
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

    fn subShapeID(self: *Gen) u32 {
        return switch (self.index(3)) {
            0 => SubShapeID.empty_value,
            1 => self.next(),
            else => makeCreator(self.creator()).getID().getValue(),
        };
    }
};

/// The extent of a shape around its center of mass (for placing shapes and points near it)
fn extentOf(desc: ShapeDesc) f32 {
    return switch (desc.kind) {
        sphere_kind => desc.radius,
        box_kind => @max(desc.half_extent[0], @max(desc.half_extent[1], desc.half_extent[2])),
        tapered_cylinder_settings_kind => @max(2.0 * desc.half_height, @max(desc.top_radius, desc.bottom_radius)),
        else => @max(desc.half_height, desc.radius),
    };
}

/// A point near the surface of the shape: random, on the axis, on the caps, on the side, on the rim
fn pointNear(gen: *Gen, shape: *const Shape, desc: ShapeDesc) P {
    const extent = extentOf(desc);
    var point = gen.vec(-1.5 * extent, 1.5 * extent);
    const bounds = shape.getLocalBounds();
    switch (gen.index(8)) {
        0 => {
            point[0] = 0;
            point[2] = 0;
        },
        1 => point[1] = bounds.max.getY(),
        2 => point[1] = bounds.min.getY(),
        3 => point[1] = gen.sign(bounds.max.getY() - 1.0e-5), // Around the epsilon of GetSurfaceNormal
        4 => {
            // On the side at the bounding radius
            const r = bounds.max.getX();
            const a = gen.plain(0, 2.0 * math.pi);
            point = .{ r * @cos(a), gen.plain(bounds.min.getY(), bounds.max.getY()), r * @sin(a) };
        },
        5 => point = .{ bounds.max.getX(), if (gen.oneIn(2)) bounds.max.getY() else bounds.min.getY(), 0 }, // Rim
        else => {},
    }
    return point;
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

test "Cylinders parity: settings, Jolt's error texts, sub types and getters" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "settings" };
    const fixed = [_]ShapeDesc{
        // TestCylinderShape / TestTaperedCylinderShape
        .{ .kind = cylinder_settings_kind, .half_height = -1, .radius = 1, .convex_radius = 1 },
        .{ .kind = cylinder_settings_kind, .half_height = 1, .radius = -1, .convex_radius = 1 },
        .{ .kind = cylinder_settings_kind, .half_height = 1, .radius = 1, .convex_radius = -1 },
        .{ .kind = cylinder_settings_kind, .half_height = 0, .radius = 0, .convex_radius = 1 },
        .{ .kind = tapered_cylinder_settings_kind, .half_height = -1, .top_radius = 1, .bottom_radius = 0.1, .convex_radius = 1 },
        .{ .kind = tapered_cylinder_settings_kind, .half_height = 1, .top_radius = -1, .bottom_radius = 0.1, .convex_radius = 1 },
        .{ .kind = tapered_cylinder_settings_kind, .half_height = 1, .top_radius = 1, .bottom_radius = -0.1, .convex_radius = 1 },
        .{ .kind = tapered_cylinder_settings_kind, .half_height = 1, .top_radius = 1, .bottom_radius = 0.1, .convex_radius = -1 },
        .{ .kind = tapered_cylinder_settings_kind, .half_height = 1.0e-12, .top_radius = 0, .bottom_radius = 1.0e-12, .convex_radius = 1 },
        // Equal radii: CylinderShape (a zero height is valid there), -0 == 0
        .{ .kind = tapered_cylinder_settings_kind, .half_height = 0, .top_radius = 1, .bottom_radius = 1, .convex_radius = 0.1 },
        .{ .kind = tapered_cylinder_settings_kind, .half_height = -1, .top_radius = 1, .bottom_radius = 1, .convex_radius = 0.1 },
        .{ .kind = tapered_cylinder_settings_kind, .half_height = 1, .top_radius = -0.0, .bottom_radius = 0.0, .convex_radius = 0.1 },
        .{ .kind = tapered_cylinder_settings_kind, .half_height = 1, .top_radius = -1, .bottom_radius = -1, .convex_radius = 0.1 },
        .{ .kind = tapered_cylinder_settings_kind, .half_height = 0, .top_radius = 1, .bottom_radius = 0.5, .convex_radius = 0.1 },
        .{ .kind = tapered_cylinder_settings_kind, .half_height = 2, .top_radius = 3, .bottom_radius = 0, .convex_radius = 0.1, .density = 0.5 },
    };
    for (0..20_000) |i| {
        var desc: ShapeDesc = undefined;
        if (i < fixed.len) {
            desc = fixed[i];
        } else {
            desc = .{ .kind = @intCast(gen.index(2)), .density = gen.plain(0, 2000) };
            desc.half_height = gen.float(-0.5, 2);
            desc.radius = gen.float(-0.5, 2);
            desc.top_radius = gen.float(-0.5, 2);
            desc.bottom_radius = if (gen.oneIn(5)) desc.top_radius else gen.float(-0.5, 2);
            desc.convex_radius = gen.float(-0.5, 1);
        }
        var jolt_error: [128]u8 = undefined;
        var jolt_sub_type: u32 = undefined;
        var jolt_getters: [5]f32 = undefined;
        const jolt_valid = jolt.jolt_cylinders_settings(&desc, &jolt_error, &jolt_sub_type, &jolt_getters);
        var zolt_error: [128]u8 = @splat(0);
        var zolt_sub_type: u32 = 0;
        var zolt_getters: [5]f32 = @splat(0);
        var shape = try createShape(allocator, desc, &zolt_error);
        const zolt_valid: c_int = @intFromBool(shape != null);
        if (shape) |*s| {
            zolt_sub_type = @intFromEnum(s.get().?.getSubType());
            zolt_getters = getters(s.get().?);
            s.deinit();
        }
        checker.check(.{desc}, .{ zolt_valid, zolt_error, zolt_sub_type, zolt_getters }, .{ jolt_valid, jolt_error, jolt_sub_type, jolt_getters });
    }
    try checker.finish();
}

test "Cylinders parity: bounds, mass properties, volume, scales, leaf shape, surface normal, supporting face" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "properties" };
    var scales: Checker = .{ .name = "scales" };
    for (0..iterations) |_| {
        const desc = gen.cylinder();
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const s = shape.get().?;
        const input: PropertiesInput = .{
            .scale = gen.scale(desc),
            .transform = arr16(gen.transform(10)),
            .translation = .{ gen.plain(-1.0e5, 1.0e5), gen.plain(-10, 10), gen.plain(-10, 10) },
            .point = pointNear(&gen, s, desc),
            .direction = gen.direction(3),
            .sub_shape_id = gen.subShapeID(),
            .position = gen.vec(-10, 10),
            .rotation = arr4(gen.rotation().getXYZW()),
        };

        var jolt_is_valid: c_int = undefined;
        var jolt_scale_valid: P = undefined;
        var jolt_output = std.mem.zeroes(PropertiesOutput);
        jolt.jolt_cylinders_properties(&desc, &input, 0, &jolt_is_valid, &jolt_scale_valid, &jolt_output);
        const zolt_output = zoltProperties(s, &input);
        checker.check(.{ desc, input }, zolt_output, jolt_output);

        // IsValidScale / MakeScaleValid on any scale (the other functions assert a valid scale)
        var scale_input = input;
        scale_input.scale = gen.anyScale();
        jolt.jolt_cylinders_properties(&desc, &scale_input, 1, &jolt_is_valid, &jolt_scale_valid, &jolt_output);
        const zolt_is_valid: c_int = @intFromBool(s.isValidScale(vec3(scale_input.scale)));
        scales.check(.{ desc, scale_input.scale }, .{ zolt_is_valid, arr3(s.makeScaleValid(vec3(scale_input.scale))) }, .{ jolt_is_valid, jolt_scale_valid });
    }
    try finishAll(&.{ &checker, &scales });
}

test "Cylinders parity: support functions of every mode" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "support" };
    for (0..iterations / 4) |_| {
        const desc = gen.cylinder();
        const scale = gen.scale(desc);
        var directions: [8 * 3]f32 = undefined;
        for (0..8) |d| directions[3 * d ..][0..3].* = gen.direction(3);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const convex = shape.get().?.cast(ConvexShape);
        for ([_]ConvexShape.SupportMode{ .exclude_convex_radius, .include_convex_radius, .default }) |mode| {
            var jolt_points: [8 * 3]f32 = @splat(0);
            const jolt_radius = jolt.jolt_cylinders_support(&desc, @intFromEnum(mode), &scale, &directions, 8, &jolt_points);
            var buffer: ConvexShape.SupportBuffer = .{};
            const support = convex.getSupportFunction(mode, &buffer, vec3(scale));
            var zolt_points: [8 * 3]f32 = @splat(0);
            for (0..8) |d| zolt_points[3 * d ..][0..3].* = arr3(support.getSupport(vec3(directions[3 * d ..][0..3].*)));
            checker.check(.{ desc, mode, scale, directions }, .{ support.getConvexRadius(), zolt_points }, .{ jolt_radius, jolt_points });
        }
    }
    try checker.finish();
}

test "Cylinders parity: CastRay (single hit and collectors) and CollidePoint" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var rays: Checker = .{ .name = "cast ray" };
    var points: Checker = .{ .name = "collide point" };
    var num_hits: usize = 0;
    for (0..iterations) |_| {
        const desc = gen.cylinder();
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const s = shape.get().?;
        const extent = extentOf(desc);
        const bounds = s.getLocalBounds();

        // Rays from outside through the shape, from inside, along faces, degenerate directions
        var input: RayInput = .{
            .origin = gen.vec(-2 * extent - 1, 2 * extent + 1),
            .direction = gen.direction(4 * extent + 2),
            .creator = gen.creator(),
            .fraction = if (gen.oneIn(2)) 1.0 + math.flt_epsilon else gen.plain(0, 1),
            .back_face_mode = @intFromBool(gen.oneIn(2)),
            .treat_convex_as_solid = @intFromBool(!gen.oneIn(3)),
            .collector = @intCast(gen.index(3)),
            .early_out = if (gen.oneIn(2)) 2.0 else gen.plain(0, 1),
            .body_id = gen.next() & 0x7fffff,
        };
        switch (gen.index(8)) {
            0 => {
                // Aim at the center
                const target = gen.plainVec(-0.5 * extent, 0.5 * extent);
                input.direction = arr3(vec3(target).sub(vec3(input.origin)).mulScalar(gen.plain(0.5, 3)));
            },
            1 => {
                // Parallel to a cap, in its plane or just above / below it (grazing)
                const y = if (gen.oneIn(2)) bounds.max.getY() else bounds.min.getY();
                input.origin = .{ -2 * extent - 1, y + ([_]f32{ 0, 1.0e-6, -1.0e-6 })[gen.index(3)], gen.plain(-extent, extent) };
                input.direction = .{ 4 * extent + 2, 0, gen.plain(-0.1, 0.1) };
            },
            2 => {
                // Along the side, parallel to the axis
                input.origin = .{ bounds.max.getX(), -2 * extent - 1, 0 };
                input.direction = .{ 0, 4 * extent + 2, 0 };
            },
            3 => input.origin = gen.plainVec(-0.3 * extent, 0.3 * extent), // Starting inside
            else => {},
        }
        var jolt_output = std.mem.zeroes(RayOutput);
        jolt.jolt_cylinders_cast_ray(&desc, &input, &jolt_output);
        const zolt_output = try zoltCastRay(allocator, s, &input);
        num_hits += @intCast(zolt_output.hit);
        rays.check(.{ desc, input }, zolt_output, jolt_output);

        // Points inside, on the surface and outside
        const point = pointNear(&gen, s, desc);
        var jolt_ids: [2]u32 = undefined;
        const jolt_count = jolt.jolt_cylinders_collide_point(&desc, &point, &input.creator, input.body_id, &jolt_ids);
        var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer collector.deinit();
        const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
        collector.base.setContext(&context);
        s.collidePoint(vec3(point), makeCreator(input.creator), &collector.base, &.{});
        try collector.checkError();
        var zolt_ids: [2]u32 = .{ 0, 0 };
        for (collector.hits.items) |h| zolt_ids = .{ h.body_id.getIndexAndSequenceNumber(), h.sub_shape_id2.getValue() };
        points.check(.{ desc, point }, .{ @as(u32, @intCast(collector.hits.items.len)), zolt_ids }, .{ jolt_count, jolt_ids });
    }
    try finishAll(&.{ &rays, &points });
    try std.testing.expect(num_hits > iterations / 5 and num_hits < 4 * iterations / 5); // Hits and misses
}

/// Hand-picked collisions: touching shapes (cylinder on a box, cylinders side by side, cone tip on a box, sphere on the
/// rim, cylinder cap on a cylinder cap)
fn fixedCollideInput(i: usize, input: *CollideInput) void {
    const cylinder: ShapeDesc = .{ .kind = cylinder_settings_kind, .half_height = 1, .radius = 0.5, .convex_radius = 0.05 };
    const cone: ShapeDesc = .{ .kind = tapered_cylinder_settings_kind, .half_height = 1, .top_radius = 0, .bottom_radius = 1, .convex_radius = 0.05 };
    const box: ShapeDesc = .{ .kind = box_kind, .half_extent = .{ 2, 0.5, 2 }, .convex_radius = 0.05 };
    const sphere: ShapeDesc = .{ .kind = sphere_kind, .radius = 0.5 };
    input.scale1 = .{ 1, 1, 1 };
    input.scale2 = .{ 1, 1, 1 };
    input.transform1 = arr16(Mat44.identity());
    switch (i) {
        0 => {
            input.shape1 = cylinder;
            input.shape2 = box;
            input.transform2 = arr16(Mat44.translation(Vec3.init(0, -1.5, 0)));
        },
        1 => {
            input.shape1 = cylinder;
            input.shape2 = cylinder;
            input.transform2 = arr16(Mat44.translation(Vec3.init(1, 0, 0)));
        },
        2 => {
            // The cone's center of mass is not at the origin: put the tip on the box
            input.shape1 = cone;
            input.shape2 = box;
            input.transform1 = arr16(Mat44.rotationTranslation(Quat.rotation(Vec3.axisX(), math.pi), Vec3.zero()));
            input.transform2 = arr16(Mat44.translation(Vec3.init(0, -1.5 - 0.5, 0)));
        },
        3 => {
            input.shape1 = sphere;
            input.shape2 = cylinder;
            input.transform1 = arr16(Mat44.translation(Vec3.init(0.5 + 0.5 * 0.70710677, 1 + 0.5 * 0.70710677, 0)));
        },
        4 => {
            input.shape1 = cylinder;
            input.shape2 = cylinder;
            input.transform2 = arr16(Mat44.translation(Vec3.init(0.3, 2, 0)));
        },
        else => {
            input.shape1 = cone;
            input.shape2 = cylinder;
            input.transform2 = arr16(Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), Vec3.init(0, 0.5, 0)));
        },
    }
}

test "Cylinders parity: collide vs spheres, boxes and cylinders through CollisionDispatch" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "collide" };
    var num_hits: usize = 0;
    for (0..iterations / 2) |i| {
        var shape1 = gen.shape();
        var shape2 = gen.shape();
        if (!shape1.isCylinder() and !shape2.isCylinder()) {
            if (gen.oneIn(2)) shape1 = gen.cylinder() else shape2 = gen.cylinder();
        }
        const transform1 = gen.transform(5);
        // Place shape 2 near shape 1 so that about half of the pairs collide
        const reach = extentOf(shape1) + extentOf(shape2);
        var relative = gen.transform(1.2 * reach);
        if (gen.oneIn(10)) relative.setTranslation(Vec3.zero()); // Same center: near zero penetration axis
        if (gen.oneIn(10)) relative = Mat44.identity();
        var input: CollideInput = .{
            .shape1 = shape1,
            .shape2 = shape2,
            .scale1 = gen.scale(shape1),
            .scale2 = gen.scale(shape2),
            .transform1 = arr16(transform1),
            .transform2 = arr16(transform1.mul(relative)),
            .creator1 = gen.creator(),
            .creator2 = gen.creator(),
            .max_separation_distance = switch (gen.index(5)) {
                0, 1 => 0.0,
                2 => gen.plain(0, 1),
                3 => gen.plain(1, 3), // Clamped to 1 in the EPA path
                else => 100.0,
            },
            .collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance,
            .penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance,
            .collect_faces = @intFromBool(gen.oneIn(2)),
            .back_face_mode = @intFromBool(gen.oneIn(2)),
            .active_edge_mode = @intFromBool(gen.oneIn(2)),
            .active_edge_movement_direction = if (gen.oneIn(2)) .{ 0, 0, 0 } else gen.vec(-1, 1),
            .early_out = if (gen.oneIn(4)) gen.plain(-2, 2) else math.flt_max,
            .body_id = gen.next() & 0x7fffff,
        };
        if (i < 6) fixedCollideInput(i, &input);
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_cylinders_collide(&input, &jolt_output);
        const zolt_output = try zoltCollide(allocator, &input);
        num_hits += zolt_output.num_hits;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > iterations / 8 and num_hits < 3 * iterations / 8); // Both paths are exercised
}

test "Cylinders parity: cast vs spheres, boxes and cylinders through CollisionDispatch (world space)" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "cast" };
    var num_hits: usize = 0;
    for (0..iterations / 2) |_| {
        var shape1 = gen.shape();
        var shape2 = gen.shape();
        if (!shape1.isCylinder() and !shape2.isCylinder()) {
            if (gen.oneIn(2)) shape1 = gen.cylinder() else shape2 = gen.cylinder();
        }
        const transform2 = gen.transform(5);
        const reach = extentOf(shape1) + extentOf(shape2);
        // Start near shape 2 (sometimes overlapping), move towards it, past it or away from it
        var start = transform2.mul(gen.transform(2.0 * reach));
        if (gen.oneIn(8)) start.setTranslation(transform2.getTranslation().add(vec3(gen.plainVec(-0.2, 0.2))));
        const to_target = transform2.getTranslation().sub(start.getTranslation());
        const direction = switch (gen.index(4)) {
            0 => to_target.mulScalar(gen.plain(0.5, 3)),
            1 => to_target.mulScalar(gen.plain(-1, 0.2)),
            2 => vec3(gen.direction(3 * reach)),
            else => to_target.add(vec3(gen.plainVec(-reach, reach))).mulScalar(gen.plain(0.5, 2)),
        };
        const input: CastInput = .{
            .shape1 = shape1,
            .shape2 = shape2,
            .scale1 = gen.scale(shape1),
            .start = arr16(start),
            .direction = arr3(direction),
            .scale2 = gen.scale(shape2),
            .transform2 = arr16(transform2),
            .creator1 = gen.creator(),
            .creator2 = gen.creator(),
            .collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance,
            .penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance,
            .extra_convex_radius = if (gen.oneIn(4)) gen.plain(0, 0.3) else 0.0,
            .back_face_mode_triangles = @intFromBool(gen.oneIn(2)),
            .back_face_mode_convex = @intFromBool(gen.oneIn(2)),
            .active_edge_mode = @intFromBool(gen.oneIn(2)),
            .use_shrunken_shape = @intFromBool(gen.oneIn(2)),
            .return_deepest_point = @intFromBool(gen.oneIn(2)),
            .collect_faces = @intFromBool(gen.oneIn(2)),
            .early_out = if (gen.oneIn(4)) gen.plain(-0.5, 1) else 2.0,
            .body_id = gen.next() & 0x7fffff,
        };
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_cylinders_cast(&input, &jolt_output);
        const zolt_output = try zoltCast(allocator, &input);
        num_hits += zolt_output.num_hits;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > iterations / 10 and num_hits < 4 * iterations / 10); // Hits and misses
}

test "Cylinders parity: GetSubmergedVolume" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "submerged volume" };
    for (0..iterations) |_| {
        const desc = gen.cylinder();
        const scale = gen.scale(desc);
        const transform = arr16(gen.transform(3));
        const normal = vec3(gen.direction(1)).normalizedOr(Vec3.axisY());
        const plane = arr4(Plane.fromPointAndNormal(vec3(gen.vec(-5, 5)), normal).normal_and_constant);
        var jolt_values: [5]f32 = undefined;
        jolt.jolt_cylinders_submerged_volume(&desc, &transform, &scale, &plane, &jolt_values);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const r = shape.get().?.getSubmergedVolume(mat44(transform), vec3(scale), .fromVec4(vec4(plane)));
        const zolt_values = [_]f32{ r.total_volume, r.submerged_volume } ++ arr3(r.center_of_buoyancy);
        checker.check(.{ desc, scale, transform, plane }, zolt_values, jolt_values);
    }
    try checker.finish();
}

test "Cylinders parity: GetTrianglesStart / Next" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "triangles" };
    for (0..iterations / 10) |i| {
        var desc = gen.cylinder();
        if (i == 0) desc = .{ .kind = cylinder_settings_kind, .half_height = 1, .radius = 1 }; // The unit cylinder table (sUnitCylinderTriangles)
        const scale = gen.scale(desc);
        const position = gen.vec(-10, 10);
        const rotation = arr4(gen.rotation().getXYZW());
        const max_requested: c_int = if (gen.oneIn(2)) 32 else @intCast(32 + gen.index(200));

        var jolt_counts: [8]c_int = @splat(0);
        var jolt_vertices: [max_vertices * 3]f32 = @splat(0);
        var jolt_default: [max_vertices / 3]c_int = @splat(0);
        const jolt_calls = jolt.jolt_cylinders_triangles(&desc, &position, &rotation, &scale, max_requested, &jolt_counts, &jolt_vertices, &jolt_default);

        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        var zolt_counts: [8]c_int = @splat(0);
        var zolt_vertices: [max_vertices * 3]f32 = @splat(0);
        var zolt_default: [max_vertices / 3]c_int = @splat(0);
        var context: Shape.GetTrianglesContext = .{};
        shape.get().?.getTrianglesStart(&context, AABox.biggest(), vec3(position), quat(rotation), vec3(scale));
        var triangles: [3 * 232]Float3 = undefined;
        var materials: [232]*const PhysicsMaterial = undefined;
        var zolt_calls: c_int = 0;
        var out: usize = 0;
        var out_material: usize = 0;
        while (true) {
            const count = shape.get().?.getTrianglesNext(&context, @intCast(max_requested), triangles[0 .. 3 * @as(usize, @intCast(max_requested))], materials[0..@intCast(max_requested)]);
            zolt_counts[@intCast(zolt_calls)] = @intCast(count);
            zolt_calls += 1;
            for (triangles[0 .. 3 * count]) |t| {
                zolt_vertices[out..][0..3].* = .{ t.x, t.y, t.z };
                out += 3;
            }
            for (materials[0..count]) |m| {
                zolt_default[out_material] = @intFromBool(m == PhysicsMaterial.default);
                out_material += 1;
            }
            if (count == 0 or zolt_calls == 8) break;
        }
        checker.check(.{ desc, scale, position, rotation, max_requested }, .{ zolt_calls, zolt_counts, zolt_vertices, zolt_default }, .{ jolt_calls, jolt_counts, jolt_vertices, jolt_default });
    }
    try checker.finish();
}

test "Cylinders parity: CollideSoftBodyVertices" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "soft body vertices" };
    const n = 8;
    for (0..iterations / 4) |_| {
        const desc = gen.cylinder();
        const scale = gen.scale(desc);
        const transform = arr16(gen.transform(3));
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const s = shape.get().?;

        var positions: [n * 3]f32 = undefined;
        var inv_masses: [n]f32 = undefined;
        var penetrations: [n]f32 = undefined;
        const planes: [n * 4]f32 = @splat(0);
        const indices: [n]c_int = @splat(-1);
        for (0..n) |v| {
            // Points around the shape in local space (scaled), in every region
            const local = vec3(pointNear(&gen, s, desc)).mul(vec3(scale)).mulScalar(gen.plain(0.5, 2));
            positions[3 * v ..][0..3].* = arr3(mat44(transform).mulVec3(local));
            if (gen.oneIn(8)) positions[3 * v ..][0..3].* = arr3(mat44(transform).getTranslation()); // At the center
            inv_masses[v] = if (gen.oneIn(5)) 0.0 else 1.0;
            penetrations[v] = if (gen.oneIn(2)) -math.flt_max else gen.plain(-2, 2);
        }
        var jolt_penetrations = penetrations;
        var jolt_planes = planes;
        var jolt_indices = indices;
        jolt.jolt_cylinders_soft_body(&desc, &transform, &scale, n, &positions, &inv_masses, &jolt_penetrations, &jolt_planes, &jolt_indices, 3);

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
        s.collideSoftBodyVertices(mat44(transform), vec3(scale), &vertices, n, 3);
        var zolt_plane_values: [n * 4]f32 = undefined;
        for (0..n) |v| zolt_plane_values[4 * v ..][0..4].* = arr4(zolt_planes[v].normal_and_constant);
        var zolt_index_values: [n]c_int = undefined;
        for (0..n) |v| zolt_index_values[v] = zolt_indices[v];
        checker.check(.{ desc, scale, transform, positions }, .{ zolt_penetrations, zolt_plane_values, zolt_index_values }, .{ jolt_penetrations, jolt_planes, jolt_indices });
    }
    try checker.finish();
}

test "Cylinders parity: binary state, restore and truncated states" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "binary state" };
    var truncated: Checker = .{ .name = "truncated binary state" };
    for (0..iterations / 10) |_| {
        const desc = gen.cylinder();
        const user_data = (@as(u64, gen.next()) << 32) | gen.next();
        var jolt_bytes: [64]u8 = @splat(0);
        var jolt_restored: [64]u8 = @splat(0);
        var jolt_restored_size: u32 = 0;
        const jolt_size = jolt.jolt_cylinders_binary_state(&desc, user_data, &jolt_bytes, 64, &jolt_restored, &jolt_restored_size);

        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        shape.get().?.setUserData(user_data);
        var zolt_bytes: [64]u8 = @splat(0);
        var writer: std.Io.Writer = .fixed(&zolt_bytes);
        var stream_out = StreamOutWrapper.init(&writer);
        shape.get().?.saveBinaryState(stream_out.streamOut());
        const zolt_size: u32 = @intCast(writer.buffered().len);

        // Restore Jolt's bytes and save again
        var zolt_restored: [64]u8 = @splat(0);
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

        // Restore a truncated state: Jolt's error text
        const size = gen.index(jolt_size);
        var jolt_error: [128]u8 = undefined;
        const jolt_valid = jolt.jolt_cylinders_restore(&jolt_bytes, @intCast(size), &jolt_error);
        var short_reader: std.Io.Reader = .fixed(jolt_bytes[0..size]);
        var short_in = StreamInWrapper.init(&short_reader);
        var short_result = try Shape.restoreFromBinaryState(allocator, short_in.streamIn());
        defer short_result.deinit();
        var zolt_error: [128]u8 = @splat(0);
        if (short_result.hasError()) copyError(short_result.getError(), &zolt_error);
        truncated.check(.{ desc, size }, .{ @as(c_int, @intFromBool(short_result.isValid())), zolt_error }, .{ jolt_valid, jolt_error });
    }
    try finishAll(&.{ &checker, &truncated });
}
