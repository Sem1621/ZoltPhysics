//! Parity tests for the capsules (Phase 4, Wave A): CapsuleShape and TaperedCapsuleShape. The shapes are built from their
//! settings on both sides (`ShapeDesc`), so Create()'s special cases are compared as well: a capsule without height
//! becomes a SphereShape, a tapered capsule whose spheres contain each other a SphereShape or a RotatedTranslatedShape
//! with a SphereShape. Spheres and boxes are the collision partners. Kinds 4 and 5 call the constructors that take the
//! settings directly (bypassing Create's sphere logic), so that every error text of the constructors is reachable.
//!
//! Compared bit for bit on random inputs mixed with edge cases (zero / tiny heights, equal radii, one sphere containing
//! the other, nearly embedded spheres (steep cones), very different radii, scales with negative components (a negative Y
//! flips a tapered capsule), axis parallel / grazing / inside rays, touching shapes): the settings' IsValid / IsSphere,
//! the created shape (sub type, user data, density, inner shape) and Jolt's error texts, GetLocalBounds,
//! GetWorldSpaceBounds (Mat44 and DMat44), GetCenterOfMass, GetInnerRadius, GetMassProperties, GetVolume, GetStats
//! (triangles), GetSubShapeIDBitsRecursive, IsValidScale / MakeScaleValid, GetSurfaceNormal, GetLeafShape,
//! GetSubShapeUserData, GetMaterial, GetSupportingFace, the support points of every ESupportMode with scales, CastRay
//! (both overloads, with the AllHit / AnyHit / ClosestHit collectors, back faces, solid or not, early out fractions),
//! CollidePoint (with `CapsulesParityFilter`, which rejects a shape sub type and hashes its calls),
//! CollisionDispatch::sCollideShapeVsShape and sCastShapeVsShapeWorldSpace against capsules, tapered
//! capsules, spheres and boxes (GJK, EPA, max separation distance, tolerances, faces, back face / active edge modes,
//! shrunken shapes, deepest point, extra convex radius, early out fractions; all hits in order), GetSubmergedVolume,
//! GetTrianglesStart / Next (CapsuleShape's three vertex lists, ConvexShape's version for the tapered capsule),
//! CollideSoftBodyVertices and the binary state bytes (SaveBinaryState, sRestoreFromBinaryState, SaveWithChildren,
//! sRestoreWithChildren). Some capsules and tapered capsules have a PhysicsMaterialSimple (`ShapeDesc.material`), so
//! GetMaterial, the materials of GetTrianglesNext and the material entries of SaveWithChildren are compared with and
//! without a material. C ABI wrappers: ZoltParity/Physics/CapsulesReference.cpp.

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
const CapsuleShape = zolt.CapsuleShape;
const CapsuleShapeSettings = zolt.CapsuleShapeSettings;
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
const ConvexShape = zolt.ConvexShape;
const DecoratedShape = zolt.DecoratedShape;
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
const RefConst = zolt.RefConst;
const RVec3 = zolt.RVec3;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeFilter = zolt.ShapeFilter;
const ShapeList = zolt.ShapeList;
const ShapeResult = zolt.ShapeResult;
const ShapeSettings = zolt.ShapeSettings;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TaperedCapsuleShape = zolt.TaperedCapsuleShape;
const TaperedCapsuleShapeSettings = zolt.TaperedCapsuleShapeSettings;
const TransformedShape = zolt.TransformedShape;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

/// The C++ reference functions, see CapsulesReference.cpp
const jolt = struct {
    extern fn jolt_capsules_create(desc: *const ShapeDesc, out_error: *[128]u8, out_info: *[6]u32) c_int;
    extern fn jolt_capsules_scale(desc: *const ShapeDesc, scale: *const P, out_scale_valid: *P) c_int;
    extern fn jolt_capsules_properties(desc: *const ShapeDesc, input: *const PropertiesInput, output: *PropertiesOutput) void;
    extern fn jolt_capsules_support(desc: *const ShapeDesc, mode: c_int, scale: *const P, directions: [*]const f32, num_directions: c_int, out_points: [*]f32, out_convex_radius: *f32) c_int;
    extern fn jolt_capsules_cast_ray(desc: *const ShapeDesc, input: *const RayInput, output: *RayOutput) void;
    extern fn jolt_capsules_collide_point(desc: *const ShapeDesc, point: *const P, creator: *const [2]u32, body_id: u32, reject_sub_type: u32, output: *PointOutput) void;
    extern fn jolt_capsules_collide(input: *const CollideInput, output: *HitsOutput) void;
    extern fn jolt_capsules_cast(input: *const CastInput, output: *HitsOutput) void;
    extern fn jolt_capsules_submerged_volume(desc: *const ShapeDesc, transform: *const [16]f32, scale: *const P, plane: *const [4]f32, out_values: *[5]f32) void;
    extern fn jolt_capsules_triangles(desc: *const ShapeDesc, position: *const P, rotation: *const [4]f32, scale: *const P, max_triangles_requested: c_int, out_counts: *[64]c_int, out_vertices: *[max_vertices * 3]f32, out_materials: *[max_vertices / 3]u32) c_int;
    extern fn jolt_capsules_binary_state(desc: *const ShapeDesc, out_bytes: *[binary_capacity]u8, out_restored_bytes: *[binary_capacity]u8, out_children_bytes: *[binary_capacity]u8, out_restored_children_bytes: *[binary_capacity]u8, capacity: u32, out_sizes: *[4]u32) void;
    extern fn jolt_capsules_soft_body(desc: *const ShapeDesc, transform: *const [16]f32, scale: *const P, num_vertices: c_int, positions: [*]const f32, inv_masses: [*]const f32, io_penetrations: [*]f32, io_planes: [*]f32, io_indices: [*]c_int, colliding_shape_index: c_int) void;
};

/// Number of random inputs per test
const iterations = fw.iterations;

/// The most triangle vertices a shape returns (a capsule: 64 + 32 + 64 triangles)
const max_vertices = 480;

/// Capacity of the binary state buffers
const binary_capacity = 256;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// Shape description, must match ShapeDesc in CapsulesReference.cpp
const ShapeDesc = extern struct {
    /// 0: CapsuleShapeSettings::Create, 1: TaperedCapsuleShapeSettings::Create, 2: SphereShapeSettings, 3: BoxShapeSettings,
    /// 4: the CapsuleShape constructor, 5: the TaperedCapsuleShape constructor
    kind: u32,
    /// Capsule: half height of the cylinder, tapered capsule: half height of the tapered cylinder
    half_height: f32 = 0.0,
    /// Capsule, sphere, top radius of the tapered capsule
    radius: f32 = 0.0,
    /// Tapered capsule
    bottom_radius: f32 = 0.0,
    /// Box
    half_extent: P = .{ 0, 0, 0 },
    convex_radius: f32 = 0.0,
    density: f32 = 1000.0,
    user_data: u32 = 0,
    /// Capsule, tapered capsule: 0 for none, otherwise the color of a PhysicsMaterialSimple named `material_name`
    material: u32 = 0,
};

/// Name of the PhysicsMaterialSimple of a ShapeDesc, must match cMaterialName in CapsulesReference.cpp
const material_name = "CapsulesParity";

/// The filter rejects nothing
const no_reject: u32 = ~@as(u32, 0);

/// Must match PropertiesInput in CapsulesReference.cpp
const PropertiesInput = extern struct {
    scale: P,
    transform: [16]f32,
    /// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
    translation: [3]f64,
    /// GetSurfaceNormal
    point: P,
    /// GetSupportingFace
    direction: P,
};

/// Must match PropertiesOutput in CapsulesReference.cpp
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
    surface_normal: P,
    leaf_sub_type: u32,
    leaf_remainder: u32,
    sub_shape_user_data: u32,
    /// materialCode of GetMaterial
    material: u32,
    face_count: u32,
    face: [32 * 3]f32,
};

/// Must match RayInput in CapsulesReference.cpp
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

/// Must match RayOutput in CapsulesReference.cpp
const RayOutput = extern struct {
    hit: c_int,
    fraction: f32,
    sub_shape_id: u32,
    num_hits: u32,
    hits: [4]RayHit,
};

/// Must match PointOutput in CapsulesReference.cpp
const PointOutput = extern struct {
    num_hits: u32,
    /// Body ID and sub shape ID of the last hit
    body_id: u32,
    sub_shape_id: u32,
    filter_calls: u32,
    filter_hash: u32,
};

/// Must match CollideInput in CapsulesReference.cpp
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
    /// 1: collide with back faces
    back_face_mode: c_int,
    /// 1: collide with all
    active_edge_mode: c_int,
    /// Early out fraction of the collector (when < FLT_MAX)
    early_out: f32,
    body_id: u32,
};

/// Must match HitOutput in CapsulesReference.cpp
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

/// Must match HitsOutput in CapsulesReference.cpp
const HitsOutput = extern struct {
    num_hits: u32,
    hits: [2]HitOutput,
};

/// Must match CastInput in CapsulesReference.cpp
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
    back_face_mode_convex: c_int,
    use_shrunken_shape: c_int,
    return_deepest_point: c_int,
    collect_faces: c_int,
    /// Early out fraction of the collector (when < 1 + FLT_EPSILON)
    early_out: f32,
    body_id: u32,
};

// ---------------------------------------------------------------------------------------------------------------------
// The filter, must match CapsulesParityFilter in CapsulesReference.cpp

const FilterLog = struct {
    calls: u32 = 0,
    hash: u32 = 0x811c9dc5,

    fn add(self: *FilterLog, value: u32) void {
        self.hash = (self.hash ^ value) *% 0x01000193;
    }
};

/// Rejects a shape sub type and hashes its calls (CollidePoint only calls the single shape version)
const CapsulesParityFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: ShapeFilter = .init(@This()),
    reject_sub_type: u32,
    /// The calls are logged behind a pointer (Rule M: the filter is const in the queries)
    log: *FilterLog,

    pub fn shouldCollide(self: *const CapsulesParityFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        self.log.calls += 1;
        self.log.add(@intFromEnum(shape2.getSubType()));
        self.log.add(sub_shape_id_of_shape2.getValue());
        return @intFromEnum(shape2.getSubType()) != self.reject_sub_type;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of the wrappers

/// Build the shape from its settings (kinds 4 and 5 call the constructor that takes the settings directly, bypassing the
/// sphere logic of Create), null when the settings are invalid (the error text is copied to `out_error` when given)
fn createShape(allocator: Allocator, desc: ShapeDesc, out_error: ?*[128]u8) !?Ref(Shape) {
    var material: RefConst(PhysicsMaterial) = if (desc.material != 0) .init((try PhysicsMaterialSimple.create(allocator, material_name, Color.fromUInt32(desc.material))).material()) else .empty;
    defer material.deinit();
    var result: ShapeResult = switch (desc.kind) {
        0, 4 => blk: {
            var settings = CapsuleShapeSettings.init(allocator, desc.half_height, desc.radius, .{ .material = material.get() });
            defer settings.deinit();
            settings.base.density = desc.density;
            settings.asShapeSettings().user_data = desc.user_data;
            if (desc.kind == 0)
                break :blk try settings.asShapeSettings().createShape(allocator);
            try ShapeSettings.constructShape(CapsuleShape, &settings, allocator);
            break :blk settings.asShapeSettings().cached_result.clone();
        },
        1, 5 => blk: {
            var settings = TaperedCapsuleShapeSettings.init(allocator, desc.half_height, desc.radius, desc.bottom_radius, .{ .material = material.get() });
            defer settings.deinit();
            settings.base.density = desc.density;
            settings.asShapeSettings().user_data = desc.user_data;
            if (desc.kind == 1)
                break :blk try settings.asShapeSettings().createShape(allocator);
            try ShapeSettings.constructShape(TaperedCapsuleShape, &settings, allocator);
            break :blk settings.asShapeSettings().cached_result.clone();
        },
        2 => blk: {
            var settings = SphereShapeSettings.init(allocator, desc.radius, .{});
            defer settings.deinit();
            settings.base.density = desc.density;
            settings.asShapeSettings().user_data = desc.user_data;
            break :blk try settings.asShapeSettings().createShape(allocator);
        },
        else => blk: {
            var settings = BoxShapeSettings.init(allocator, vec3(desc.half_extent), .{ .convex_radius = desc.convex_radius });
            defer settings.deinit();
            settings.base.density = desc.density;
            settings.asShapeSettings().user_data = desc.user_data;
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
    return result.get().clone();
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

/// 1 for the default material, otherwise the debug color (the colors of ShapeDesc.material have a non zero alpha), must
/// match MaterialCode in CapsulesReference.cpp
fn materialCode(material: *const PhysicsMaterial) u32 {
    return if (material == PhysicsMaterial.default) 1 else material.getDebugColor().getUInt32();
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

/// The info of jolt_capsules_create: IsValid / IsSphere of the settings, sub type, user data, density bits, inner sub type
fn zoltCreateInfo(allocator: Allocator, desc: ShapeDesc, shape: ?*const Shape) [6]u32 {
    var info: [6]u32 = @splat(0);
    switch (desc.kind) {
        0, 4 => {
            var settings = CapsuleShapeSettings.init(allocator, desc.half_height, desc.radius, .{});
            defer settings.deinit();
            info[0] = @intFromBool(settings.isValid());
            info[1] = @intFromBool(settings.isSphere());
        },
        1, 5 => {
            var settings = TaperedCapsuleShapeSettings.init(allocator, desc.half_height, desc.radius, desc.bottom_radius, .{});
            defer settings.deinit();
            info[0] = @intFromBool(settings.isValid());
            info[1] = @intFromBool(settings.isSphere());
        },
        else => {},
    }
    if (shape) |s| {
        info[2] = @intFromEnum(s.getSubType());
        info[3] = @truncate(s.getUserData());
        if (s.getType() == .convex)
            info[4] = @bitCast(s.cast(ConvexShape).getDensity());
        if (s.getType() == .decorated)
            info[5] = @intFromEnum(s.cast(DecoratedShape).getInnerShape().?.getSubType());
    }
    return info;
}

fn zoltProperties(shape: *const Shape, input: *const PropertiesInput) PropertiesOutput {
    var o = std.mem.zeroes(PropertiesOutput);
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
    o.is_valid_scale = @intFromBool(shape.isValidScale(scale));
    o.scale_valid = arr3(shape.makeScaleValid(scale));
    o.surface_normal = arr3(shape.getSurfaceNormal(.empty, vec3(input.point)));
    const leaf = shape.getLeafShape(.empty);
    o.leaf_sub_type = if (leaf.shape) |l| @intFromEnum(l.getSubType()) else 0xffffffff;
    o.leaf_remainder = leaf.remainder.getValue();
    o.sub_shape_user_data = @truncate(shape.getSubShapeUserData(.empty));
    o.material = materialCode(shape.getMaterial(.empty));
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

fn zoltCollidePoint(allocator: Allocator, shape: *const Shape, point: P, creator: [2]u32, body_id: u32, reject_sub_type: u32) !PointOutput {
    var o = std.mem.zeroes(PointOutput);
    var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(body_id), .{});
    collector.base.setContext(&context);
    var log: FilterLog = .{};
    const filter: CapsulesParityFilter = .{ .reject_sub_type = reject_sub_type, .log = &log };
    shape.collidePoint(vec3(point), makeCreator(creator), &collector.base, &filter.base);
    try collector.checkError();
    o.num_hits = @intCast(collector.hits.items.len);
    for (collector.hits.items) |h| {
        o.body_id = h.body_id.getIndexAndSequenceNumber();
        o.sub_shape_id = h.sub_shape_id2.getValue();
    }
    o.filter_calls = log.calls;
    o.filter_hash = log.hash;
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
    settings.back_face_mode_convex = if (input.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
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

fn saveShape(shape: *const Shape, buffer: []u8) u32 {
    var writer: std.Io.Writer = .fixed(buffer);
    var stream_out = StreamOutWrapper.init(&writer);
    shape.saveBinaryState(stream_out.streamOut());
    return @intCast(writer.buffered().len);
}

fn saveShapeWithChildren(allocator: Allocator, shape: *const Shape, buffer: []u8) !u32 {
    var writer: std.Io.Writer = .fixed(buffer);
    var stream_out = StreamOutWrapper.init(&writer);
    var shape_map: Shape.ShapeToIDMap = .empty;
    defer shape_map.deinit(allocator);
    var material_map: Shape.MaterialToIDMap = .empty;
    defer material_map.deinit(allocator);
    try shape.saveWithChildren(allocator, stream_out.streamOut(), &shape_map, &material_map);
    return @intCast(writer.buffered().len);
}

const BinaryOutput = struct {
    sizes: [4]u32 = @splat(0),
    bytes: [4][binary_capacity]u8 = @splat(@splat(0)),
};

fn zoltBinaryState(allocator: Allocator, shape: *const Shape) !BinaryOutput {
    var o: BinaryOutput = .{};
    o.sizes[0] = saveShape(shape, &o.bytes[0]);

    {
        var reader: std.Io.Reader = .fixed(o.bytes[0][0..o.sizes[0]]);
        var stream_in = StreamInWrapper.init(&reader);
        var result = try Shape.restoreFromBinaryState(allocator, stream_in.streamIn());
        defer result.deinit();
        if (result.isValid()) {
            var sub_shapes: ShapeList = .empty;
            defer {
                for (sub_shapes.items) |*s| s.deinit();
                sub_shapes.deinit(allocator);
            }
            try shape.saveSubShapeState(allocator, &sub_shapes);
            result.getPtr().?.restoreSubShapeState(sub_shapes.items);
            o.sizes[1] = saveShape(result.getPtr().?, &o.bytes[1]);
        }
    }

    o.sizes[2] = try saveShapeWithChildren(allocator, shape, &o.bytes[2]);

    {
        var reader: std.Io.Reader = .fixed(o.bytes[2][0..o.sizes[2]]);
        var stream_in = StreamInWrapper.init(&reader);
        var shape_map: Shape.IDToShapeMap = .empty;
        defer {
            for (shape_map.items) |*s| s.deinit();
            shape_map.deinit(allocator);
        }
        var material_map: Shape.IDToMaterialMap = .empty;
        defer {
            for (material_map.items) |*m| m.deinit();
            material_map.deinit(allocator);
        }
        var result = try Shape.restoreWithChildren(allocator, stream_in.streamIn(), &shape_map, &material_map);
        defer result.deinit();
        if (result.isValid())
            o.sizes[3] = try saveShapeWithChildren(allocator, result.getPtr().?, &o.bytes[3]);
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

    /// A direction: random, axis aligned, zero, tiny, horizontal (the supporting face of a capsule)
    fn direction(self: *Gen, length: f32) P {
        return switch (self.index(9)) {
            0 => .{ 0, 0, 0 },
            1 => blk: {
                var d: P = .{ 0, 0, 0 };
                d[self.index(3)] = if (self.oneIn(2)) length else -length;
                break :blk d;
            },
            2 => .{ self.grid(1), self.grid(1), self.grid(1) },
            3 => self.plainVec(-1.0e-6, 1.0e-6),
            4 => .{ self.plain(-length, length), if (self.oneIn(2)) 0.0 else self.plain(-0.03, 0.03) * length, self.plain(-length, length) },
            else => self.vec(-length, length),
        };
    }

    fn density(self: *Gen) f32 {
        return if (self.oneIn(4)) 1000.0 else self.plain(1, 3000);
    }

    /// No material (the default material) or the color of a PhysicsMaterialSimple (a non zero alpha, see materialCode)
    fn material(self: *Gen) u32 {
        return if (self.oneIn(3)) self.next() | 0xff000000 else 0;
    }

    /// CapsuleShapeSettings, sometimes without height (a sphere)
    fn capsule(self: *Gen) ShapeDesc {
        const half_height: f32 = switch (self.index(12)) {
            0 => 0.0, // A sphere
            1 => -0.0,
            2 => 1.0e-6,
            3 => self.plain(2, 10), // Long
            else => self.plain(0.05, 2),
        };
        const radius: f32 = switch (self.index(6)) {
            0 => 1.0,
            1 => self.plain(0.01, 0.1), // Thin
            else => self.plain(0.05, 2),
        };
        return .{ .kind = 0, .half_height = half_height, .radius = radius, .density = self.density(), .user_data = self.next(), .material = self.material() };
    }

    /// TaperedCapsuleShapeSettings: equal radii, one sphere containing the other (a sphere, offset with a
    /// RotatedTranslatedShape), nearly embedded spheres (a steep cone), very different radii
    fn tapered(self: *Gen) ShapeDesc {
        const half_height: f32 = switch (self.index(10)) {
            0 => 0.0,
            1 => 1.0e-7, // The offset of the sphere is too small for a RotatedTranslatedShape
            else => self.plain(0.05, 2),
        };
        var top = self.plain(0.05, 2);
        var bottom = self.plain(0.05, 2);
        switch (self.index(10)) {
            0 => bottom = top, // Equal radii
            1 => bottom = top + 2.0 * half_height + (if (self.oneIn(4)) 0.0 else self.plain(0, 1)), // The bottom sphere contains the top sphere
            2 => top = bottom + 2.0 * half_height + (if (self.oneIn(4)) 0.0 else self.plain(0, 1)), // The top sphere contains the bottom sphere
            3 => {
                // Nearly embedded: a steep cone
                const d = 2.0 * half_height * (1.0 - ([_]f32{ 1.0e-6, 1.0e-4, 1.0e-2, 0.1 })[self.index(4)]);
                if (self.oneIn(2)) bottom = top + d else top = bottom + d;
            },
            4 => {
                // Very different radii, not embedded
                top = self.plain(0.01, 0.05);
                bottom = self.plain(1, 2);
                if (self.oneIn(2)) std.mem.swap(f32, &top, &bottom);
            },
            else => {},
        }
        return .{ .kind = 1, .half_height = half_height, .radius = top, .bottom_radius = bottom, .density = self.density(), .user_data = self.next(), .material = self.material() };
    }

    /// A capsule or a tapered capsule (from settings)
    fn capsuleOrTapered(self: *Gen) ShapeDesc {
        return if (self.oneIn(2)) self.capsule() else self.tapered();
    }

    /// A shape to collide with: capsules, tapered capsules, spheres and boxes
    fn shape(self: *Gen) ShapeDesc {
        switch (self.index(6)) {
            0, 1 => return self.capsule(),
            2, 3 => return self.tapered(),
            4 => return .{ .kind = 2, .radius = if (self.oneIn(5)) 1.0 else self.plain(0.05, 3), .density = self.density() },
            else => {
                var half_extent = self.plainVec(0.05, 3);
                if (self.oneIn(10)) half_extent[self.index(3)] = 0.0; // Flat box
                const convex_radius: f32 = switch (self.index(3)) {
                    0 => 0.0,
                    1 => 0.05, // cDefaultConvexRadius
                    else => self.plain(0, 0.2),
                };
                return .{ .kind = 3, .half_extent = half_extent, .convex_radius = convex_radius, .density = self.density() };
            },
        }
    }

    /// A valid scale for the shape: uniform in absolute value with any signs, anything non zero for a box
    fn scale(self: *Gen, desc: ShapeDesc) P {
        if (self.oneIn(5)) return .{ 1, 1, 1 };
        if (desc.kind == 3) {
            var r = self.plainVec(0.2, 2.5);
            for (&r) |*c| {
                if (self.oneIn(3)) c.* = -c.*;
            }
            return r;
        }
        const s = if (self.oneIn(4)) self.grid(2) else self.plain(0.2, 2.5);
        const m = if (s == 0.0) 1.0 else @abs(s);
        return .{ if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m };
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
};

/// The half height of the shape along Y (for placing shapes near each other)
fn halfHeightOf(desc: ShapeDesc) f32 {
    return switch (desc.kind) {
        0, 4 => @abs(desc.half_height) + desc.radius,
        1, 5 => @abs(desc.half_height) + @max(desc.radius, desc.bottom_radius),
        2 => desc.radius,
        else => @max(desc.half_extent[0], @max(desc.half_extent[1], desc.half_extent[2])),
    };
}

/// The radius of the shape in the XZ plane
fn radiusOf(desc: ShapeDesc) f32 {
    return switch (desc.kind) {
        0, 2, 4 => desc.radius,
        1, 5 => @max(desc.radius, desc.bottom_radius),
        else => @max(desc.half_extent[0], @max(desc.half_extent[1], desc.half_extent[2])),
    };
}

/// The extent of a shape (for placing shapes near each other)
fn extentOf(desc: ShapeDesc) f32 {
    return @max(halfHeightOf(desc), radiusOf(desc));
}

/// A direction for which the supporting face of a tapered capsule is (nearly) a line: minus the normal of its cone
fn taperedFaceDirection(gen: *Gen, desc: ShapeDesc, scale: P) P {
    const height = 2.0 * desc.half_height;
    const sin_alpha = math.clamp((desc.bottom_radius - desc.radius) / height, -1.0, 1.0);
    const cos_alpha = @sqrt(1.0 - sin_alpha * sin_alpha);
    const angle = gen.plain(0, 2.0 * math.pi);
    const sign_y: f32 = if (scale[1] < 0.0) -1.0 else 1.0;
    const noise = if (gen.oneIn(2)) 0.0 else gen.plain(-0.02, 0.02);
    const length = if (gen.oneIn(2)) 1.0 else gen.plain(0.1, 3);
    return .{ -cos_alpha * zolt.trigonometry.cos(angle) * length, (-sin_alpha * sign_y + noise) * length, -cos_alpha * zolt.trigonometry.sin(angle) * length };
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

test "Capsules parity: settings, the created shapes and Jolt's error texts" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "settings" };
    for (0..20_000) |i| {
        var desc = switch (gen.index(3)) {
            0 => gen.capsule(),
            1 => gen.tapered(),
            else => blk: {
                // Invalid values: zero / negative heights and radii, a zero radius at one end
                const values = [_]f32{ 0.0, -0.0, -1.0, 1.0e-30, 0.5, 1.0, 2.0 };
                break :blk ShapeDesc{ .kind = @intCast(gen.index(2)), .half_height = values[gen.index(values.len)], .radius = values[gen.index(values.len)], .bottom_radius = values[gen.index(values.len)], .user_data = gen.next() };
            },
        };
        if (i < 6) {
            // Hand picked: the error of each constructor check
            desc = ([_]ShapeDesc{
                .{ .kind = 4, .half_height = 0.0, .radius = 1.0 }, // Invalid height (a sphere through Create)
                .{ .kind = 4, .half_height = 1.0, .radius = 0.0 }, // Invalid radius
                .{ .kind = 5, .half_height = 1.0, .radius = 0.0, .bottom_radius = 1.0 }, // Invalid top radius
                .{ .kind = 5, .half_height = 1.0, .radius = 1.0, .bottom_radius = -1.0 }, // Invalid bottom radius
                .{ .kind = 5, .half_height = 0.0, .radius = 1.0, .bottom_radius = 1.0 }, // Invalid height (a sphere through Create)
                .{ .kind = 5, .half_height = 1.0, .radius = 3.0, .bottom_radius = 1.0 }, // One sphere embedded in the other (a sphere through Create)
            })[i];
        } else if (gen.oneIn(4)) {
            desc.kind += 4; // The constructor directly
        }
        var jolt_error: [128]u8 = undefined;
        var jolt_info: [6]u32 = undefined;
        const jolt_valid = jolt.jolt_capsules_create(&desc, &jolt_error, &jolt_info);
        var zolt_error: [128]u8 = @splat(0);
        var shape = try createShape(allocator, desc, &zolt_error);
        defer if (shape) |*s| s.deinit();
        const zolt_valid: c_int = @intFromBool(shape != null);
        const zolt_info = zoltCreateInfo(allocator, desc, if (shape) |s| s.get().? else null);
        checker.check(.{desc}, .{ zolt_valid, zolt_error, zolt_info }, .{ jolt_valid, jolt_error, jolt_info });
    }
    try checker.finish();
}

test "Capsules parity: bounds, mass properties, volume, scales, surface normal, leaf shape, supporting face" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "properties" };
    var scales: Checker = .{ .name = "scales" };
    var num_faces: usize = 0;
    for (0..iterations) |_| {
        const desc = gen.capsuleOrTapered();
        const scale = gen.scale(desc);
        const extent = extentOf(desc);
        var input: PropertiesInput = .{
            .scale = scale,
            .transform = arr16(gen.transform(10)),
            .translation = .{ gen.plain(-1.0e5, 1.0e5), gen.plain(-10, 10), gen.plain(-10, 10) },
            .point = gen.vec(-1.5 * extent, 1.5 * extent),
            .direction = gen.direction(3),
        };
        switch (gen.index(6)) {
            // Points on the boundaries between the cylinder and the caps, on the axis
            0 => input.point[1] = if (gen.oneIn(2)) desc.half_height else -desc.half_height,
            1 => {
                input.point[0] = 0.0;
                input.point[2] = 0.0;
            },
            else => {},
        }
        if (desc.kind == 1 and desc.half_height > 0.0 and gen.oneIn(3)) input.direction = taperedFaceDirection(&gen, desc, scale);

        var jolt_output = std.mem.zeroes(PropertiesOutput);
        jolt.jolt_capsules_properties(&desc, &input, &jolt_output);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const zolt_output = zoltProperties(shape.get().?, &input);
        num_faces += @intFromBool(zolt_output.face_count > 0);
        checker.check(.{ desc, input }, zolt_output, jolt_output);

        // IsValidScale / MakeScaleValid on any scale
        const any_scale = gen.anyScale();
        var jolt_scale_valid: P = undefined;
        const jolt_is_valid = jolt.jolt_capsules_scale(&desc, &any_scale, &jolt_scale_valid);
        const s = shape.get().?;
        scales.check(.{ desc, any_scale }, .{ @as(c_int, @intFromBool(s.isValidScale(vec3(any_scale)))), arr3(s.makeScaleValid(vec3(any_scale))) }, .{ jolt_is_valid, jolt_scale_valid });
    }
    try finishAll(&.{ &checker, &scales });
    try std.testing.expect(num_faces > iterations / 20); // The supporting face is a line often enough
}

test "Capsules parity: support functions of every mode" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "support" };
    var num_convex: usize = 0;
    for (0..iterations / 4) |_| {
        const desc = gen.capsuleOrTapered();
        const scale = gen.scale(desc);
        var directions: [8 * 3]f32 = undefined;
        for (0..8) |d| directions[3 * d ..][0..3].* = gen.direction(3);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        for ([_]ConvexShape.SupportMode{ .exclude_convex_radius, .include_convex_radius, .default }) |mode| {
            var jolt_points: [8 * 3]f32 = @splat(0);
            var jolt_radius: f32 = 0.0;
            const jolt_convex = jolt.jolt_capsules_support(&desc, @intFromEnum(mode), &scale, &directions, 8, &jolt_points, &jolt_radius);
            var zolt_points: [8 * 3]f32 = @splat(0);
            var zolt_radius: f32 = 0.0;
            const zolt_convex: c_int = @intFromBool(shape.get().?.getType() == .convex);
            if (zolt_convex != 0) {
                num_convex += 1;
                var buffer: ConvexShape.SupportBuffer = .{};
                const support = shape.get().?.cast(ConvexShape).getSupportFunction(mode, &buffer, vec3(scale));
                for (0..8) |d| zolt_points[3 * d ..][0..3].* = arr3(support.getSupport(vec3(directions[3 * d ..][0..3].*)));
                zolt_radius = support.getConvexRadius();
            }
            checker.check(.{ desc, mode, scale, directions }, .{ zolt_convex, zolt_radius, zolt_points }, .{ jolt_convex, jolt_radius, jolt_points });
        }
    }
    try checker.finish();
    try std.testing.expect(num_convex > iterations / 4); // Most shapes are convex (and support functions are compared)
}

test "Capsules parity: CastRay (single hit and collectors) and CollidePoint" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var rays: Checker = .{ .name = "cast ray" };
    var points: Checker = .{ .name = "collide point" };
    var num_rejected_hits: usize = 0;
    for (0..iterations) |_| {
        const desc = gen.capsuleOrTapered();
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const extent = extentOf(desc);
        const radius = radiusOf(desc);

        // Rays from outside through the shape, from inside, parallel to the axis, grazing, degenerate directions
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
                // Parallel to the axis, inside or outside the cylinder (or on its surface)
                const r = if (gen.oneIn(3)) radius else gen.plain(0, 1.5 * radius);
                const angle = gen.plain(0, 2.0 * math.pi);
                input.origin = .{ r * zolt.trigonometry.cos(angle), if (gen.oneIn(2)) -2 * extent - 1 else 2 * extent + 1, r * zolt.trigonometry.sin(angle) };
                input.direction = .{ 0, if (input.origin[1] < 0.0) 4 * extent + 2 else -4 * extent - 2, 0 };
            },
            2 => {
                // Horizontal, grazing the cylinder or a cap
                input.origin = .{ -2 * extent - 1, gen.plain(-halfHeightOf(desc), halfHeightOf(desc)), if (gen.oneIn(2)) radius else gen.plain(-radius, radius) };
                input.direction = .{ 4 * extent + 2, 0, 0 };
            },
            3 => input.origin = arr3(vec3(gen.plainVec(-0.3, 0.3)).mulScalar(radius)), // From inside
            else => {},
        }
        var jolt_output = std.mem.zeroes(RayOutput);
        jolt.jolt_capsules_cast_ray(&desc, &input, &jolt_output);
        const zolt_output = try zoltCastRay(allocator, shape.get().?, &input);
        rays.check(.{ desc, input }, zolt_output, jolt_output);

        // Points inside, on the surface and outside
        var point = gen.vec(-1.5 * extent, 1.5 * extent);
        switch (gen.index(6)) {
            0 => point = .{ radius, if (desc.kind == 0) gen.plain(-desc.half_height, desc.half_height) else gen.plain(-extent, extent), 0 }, // On the cylinder
            1 => point = .{ 0, if (gen.oneIn(2)) halfHeightOf(desc) else -halfHeightOf(desc), 0 }, // On the caps
            2 => point[1] = if (gen.oneIn(2)) desc.half_height else -desc.half_height,
            else => {},
        }
        // The filter rejects the shape (a capsule, a tapered capsule or the sphere inside a RotatedTranslatedShape), another
        // sub type or nothing
        const sub_type = @intFromEnum(shape.get().?.getLeafShape(.empty).shape.?.getSubType());
        const reject_sub_type: u32 = switch (gen.index(4)) {
            0, 1 => sub_type,
            2 => sub_type ^ 1,
            else => no_reject,
        };
        var jolt_point = std.mem.zeroes(PointOutput);
        jolt.jolt_capsules_collide_point(&desc, &point, &input.creator, input.body_id, reject_sub_type, &jolt_point);
        const zolt_point = try zoltCollidePoint(allocator, shape.get().?, point, input.creator, input.body_id, reject_sub_type);
        points.check(.{ desc, point, reject_sub_type }, zolt_point, jolt_point);
        if (reject_sub_type == sub_type and (try zoltCollidePoint(allocator, shape.get().?, point, input.creator, input.body_id, no_reject)).num_hits > 0)
            num_rejected_hits += 1;
    }
    try finishAll(&.{ &rays, &points });
    try std.testing.expect(num_rejected_hits > iterations / 20); // The filter often rejects a point inside the shape
}

test "Capsules parity: collide with capsules, tapered capsules, spheres and boxes through CollisionDispatch" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "collide" };
    var num_hits: usize = 0;
    var num_faces: usize = 0;
    for (0..iterations) |i| {
        // At least one capsule (or tapered capsule) on either side
        var shape1 = gen.shape();
        var shape2 = gen.shape();
        if (shape1.kind >= 2 and shape2.kind >= 2) {
            if (gen.oneIn(2)) shape1 = gen.capsuleOrTapered() else shape2 = gen.capsuleOrTapered();
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
            .early_out = if (gen.oneIn(4)) gen.plain(-2, 2) else math.flt_max,
            .body_id = gen.next() & 0x7fffff,
        };
        if (i < 8) {
            // Hand picked: exactly touching capsules side by side, end to end, parallel and crossed, a tapered capsule
            // standing on a box
            const capsule: ShapeDesc = .{ .kind = 0, .half_height = 1.0, .radius = 0.5 };
            const tapered: ShapeDesc = .{ .kind = 1, .half_height = 1.0, .radius = 0.5, .bottom_radius = 0.25 };
            const box: ShapeDesc = .{ .kind = 3, .half_extent = .{ 2, 0.5, 2 }, .convex_radius = 0.05 };
            const cases = [_]struct { a: ShapeDesc, b: ShapeDesc, t: Mat44 }{
                .{ .a = capsule, .b = capsule, .t = Mat44.translation(Vec3.init(1, 0, 0)) },
                .{ .a = capsule, .b = capsule, .t = Mat44.translation(Vec3.init(0, 3, 0)) },
                .{ .a = capsule, .b = capsule, .t = Mat44.rotationTranslation(Quat.rotation(Vec3.axisX(), 0.5 * math.pi), Vec3.init(0, 0, 1)) },
                .{ .a = capsule, .b = capsule, .t = Mat44.translation(Vec3.init(0.99, 0.5, 0)) },
                .{ .a = tapered, .b = box, .t = Mat44.translation(Vec3.init(0, -1.75, 0)) },
                .{ .a = tapered, .b = tapered, .t = Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), math.pi), Vec3.init(0.75, 0, 0)) },
                .{ .a = capsule, .b = tapered, .t = Mat44.translation(Vec3.init(0, 2.75, 0)) },
                .{ .a = capsule, .b = .{ .kind = 2, .radius = 0.5 }, .t = Mat44.translation(Vec3.init(1, 1, 0)) },
            };
            input.shape1 = cases[i].a;
            input.shape2 = cases[i].b;
            input.scale1 = .{ 1, 1, 1 };
            input.scale2 = .{ 1, 1, 1 };
            input.transform1 = arr16(Mat44.identity());
            input.transform2 = arr16(cases[i].t);
        }
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_capsules_collide(&input, &jolt_output);
        const zolt_output = try zoltCollide(allocator, &input);
        num_hits += zolt_output.num_hits;
        if (zolt_output.num_hits > 0 and zolt_output.hits[0].face1_count == 2) num_faces += 1;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > iterations / 4 and num_hits < 3 * iterations / 4); // Both paths are exercised
    try std.testing.expect(num_faces > 100); // Capsule edges as supporting faces
}

test "Capsules parity: cast against capsules, tapered capsules, spheres and boxes through CollisionDispatch (world space)" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "cast" };
    var num_hits: usize = 0;
    for (0..iterations) |_| {
        var shape1 = gen.shape();
        var shape2 = gen.shape();
        if (shape1.kind >= 2 and shape2.kind >= 2) {
            if (gen.oneIn(2)) shape1 = gen.capsuleOrTapered() else shape2 = gen.capsuleOrTapered();
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
            .back_face_mode_convex = @intFromBool(gen.oneIn(2)),
            .use_shrunken_shape = @intFromBool(gen.oneIn(2)),
            .return_deepest_point = @intFromBool(gen.oneIn(2)),
            .collect_faces = @intFromBool(gen.oneIn(2)),
            .early_out = if (gen.oneIn(4)) gen.plain(-0.5, 1) else 2.0,
            .body_id = gen.next() & 0x7fffff,
        };
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_capsules_cast(&input, &jolt_output);
        const zolt_output = try zoltCast(allocator, &input);
        num_hits += zolt_output.num_hits;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try std.testing.expect(num_hits > iterations / 5 and num_hits < 4 * iterations / 5); // Hits and misses
}

test "Capsules parity: GetSubmergedVolume" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "submerged volume" };
    for (0..iterations) |_| {
        const desc = gen.capsuleOrTapered();
        const scale = gen.scale(desc);
        const transform = arr16(gen.transform(3));
        const normal = vec3(gen.direction(1)).normalizedOr(Vec3.axisY());
        const plane = arr4(Plane.fromPointAndNormal(vec3(gen.vec(-5, 5)), normal).normal_and_constant);
        var jolt_values: [5]f32 = undefined;
        jolt.jolt_capsules_submerged_volume(&desc, &transform, &scale, &plane, &jolt_values);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const r = shape.get().?.getSubmergedVolume(mat44(transform), vec3(scale), .fromVec4(vec4(plane)));
        const zolt_values = [_]f32{ r.total_volume, r.submerged_volume } ++ arr3(r.center_of_buoyancy);
        checker.check(.{ desc, scale, transform, plane }, zolt_values, jolt_values);
    }
    try checker.finish();
}

test "Capsules parity: GetTrianglesStart / Next" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "triangles" };
    for (0..iterations / 10) |_| {
        const desc = gen.capsuleOrTapered();
        const scale = gen.scale(desc);
        const position = gen.vec(-10, 10);
        const rotation = arr4(gen.rotation().getXYZW());
        const max_requested: c_int = if (gen.oneIn(2)) 32 else @intCast(32 + gen.index(200));

        // Only leaf shapes return triangles (an offset sphere is a RotatedTranslatedShape)
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        if (shape.get().?.getType() == .decorated)
            continue;

        var jolt_counts: [64]c_int = @splat(0);
        var jolt_vertices: [max_vertices * 3]f32 = @splat(0);
        var jolt_materials: [max_vertices / 3]u32 = @splat(0);
        const jolt_calls = jolt.jolt_capsules_triangles(&desc, &position, &rotation, &scale, max_requested, &jolt_counts, &jolt_vertices, &jolt_materials);

        var zolt_counts: [64]c_int = @splat(0);
        var zolt_vertices: [max_vertices * 3]f32 = @splat(0);
        var zolt_materials: [max_vertices / 3]u32 = @splat(0);
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
                zolt_materials[out_material] = materialCode(m);
                out_material += 1;
            }
            if (count == 0 or zolt_calls == 64) break;
        }
        checker.check(.{ desc, scale, position, rotation, max_requested }, .{ zolt_calls, zolt_counts, zolt_vertices, zolt_materials }, .{ jolt_calls, jolt_counts, jolt_vertices, jolt_materials });
    }
    try checker.finish();
}

test "Capsules parity: CollideSoftBodyVertices" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "soft body vertices" };
    const n = 8;
    for (0..iterations / 4) |_| {
        const desc = gen.capsuleOrTapered();
        const scale = gen.scale(desc);
        const transform = arr16(gen.transform(3));
        const extent = extentOf(desc) * @abs(scale[0]) * 1.5;
        var positions: [n * 3]f32 = undefined;
        var inv_masses: [n]f32 = undefined;
        var penetrations: [n]f32 = undefined;
        const planes: [n * 4]f32 = @splat(0);
        const indices: [n]c_int = @splat(-1);
        for (0..n) |v| {
            var local = gen.vec(-extent, extent);
            switch (gen.index(8)) {
                0 => local = .{ 0, 0, 0 }, // At the center
                1 => local = .{ 0, gen.plain(-extent, extent), 0 }, // On the axis
                2 => local[1] = desc.half_height * @abs(scale[0]) * (if (gen.oneIn(2)) @as(f32, 1.0) else -1.0), // At the end of the cylinder
                else => {},
            }
            positions[3 * v ..][0..3].* = arr3(mat44(transform).mulVec3(vec3(local)));
            inv_masses[v] = if (gen.oneIn(5)) 0.0 else 1.0;
            penetrations[v] = if (gen.oneIn(2)) -math.flt_max else gen.plain(-2, 2);
        }
        var jolt_penetrations = penetrations;
        var jolt_planes = planes;
        var jolt_indices = indices;
        jolt.jolt_capsules_soft_body(&desc, &transform, &scale, n, &positions, &inv_masses, &jolt_penetrations, &jolt_planes, &jolt_indices, 3);

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

test "Capsules parity: binary state, SaveWithChildren and the restores" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "binary state" };
    for (0..iterations / 10) |_| {
        const desc = gen.capsuleOrTapered();
        var jolt_output: BinaryOutput = .{};
        jolt.jolt_capsules_binary_state(&desc, &jolt_output.bytes[0], &jolt_output.bytes[1], &jolt_output.bytes[2], &jolt_output.bytes[3], binary_capacity, &jolt_output.sizes);
        var shape = (try createShape(allocator, desc, null)).?;
        defer shape.deinit();
        const zolt_output = try zoltBinaryState(allocator, shape.get().?);
        checker.check(.{desc}, .{ zolt_output.sizes, zolt_output.bytes }, .{ jolt_output.sizes, jolt_output.bytes });
        try std.testing.expect(zolt_output.sizes[1] == zolt_output.sizes[0] and zolt_output.sizes[3] == zolt_output.sizes[2]); // The restores succeed
    }
    try checker.finish();
}
