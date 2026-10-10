//! Parity tests for the decorated shapes (Phase 4, Wave A): ScaledShape, RotatedTranslatedShape and
//! OffsetCenterOfMassShape. A shape is a leaf (SphereShape or BoxShape, created from its settings) wrapped in up to 3
//! decorators (`ShapeDesc`), so decorators of every kind wrap each other (a scaled shape of a rotated translated shape
//! etc.). Each decorator is built with its constructor, from settings that hold the inner shape, from settings that hold
//! the inner settings (nested settings create their children) or from settings without inner shape, the same way on both
//! sides. Scales are uniform and non-uniform, with negative components (mirroring and inside out) where the shapes allow
//! them: the generator makes every scale valid with Zolt's IsValidScale / MakeScaleValid, which are compared with Jolt
//! on any scale as well.
//!
//! Compared bit for bit on random inputs mixed with edge cases: the results and Jolt's error texts of the settings,
//! GetLocalBounds, GetWorldSpaceBounds (Mat44 and DMat44), GetCenterOfMass, GetInnerRadius, GetMassProperties,
//! GetVolume, GetStats / GetStatsRecursive (triangles), GetSubShapeIDBitsRecursive, MustBeStatic, IsValidScale /
//! MakeScaleValid, GetSurfaceNormal, GetSupportingFace, GetLeafShape, GetSubShapeUserData, GetMaterial,
//! SaveSubShapeState, GetSubShapeTransformedShape, ScaledShape::GetScale, RotatedTranslatedShape::GetPosition /
//! GetRotation / TransformScale, OffsetCenterOfMassShape::GetOffset, CastRay (both overloads, with the AllHit / AnyHit /
//! ClosestHit collectors, back faces, solid or not, early out fractions), CollidePoint, CollectTransformedShapes,
//! TransformShape, the triangles of the collected leaves (TransformedShape::GetTrianglesStart / Next),
//! CollisionDispatch::sCollideShapeVsShape and sCastShapeVsShapeWorldSpace with a decorated shape on either side or on
//! both sides and spheres / boxes on the other (all settings, all hits in order), GetSubmergedVolume,
//! CollideSoftBodyVertices and the binary state (SaveBinaryState, sRestoreFromBinaryState + RestoreSubShapeState,
//! SaveWithChildren, sRestoreWithChildren). Every query with a ShapeFilter uses `DecoratedParityFilter`, which rejects
//! a shape sub type and hashes the arguments of every call it receives. C ABI wrappers:
//! ZoltParity/Physics/DecoratedReference.cpp.

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
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const OffsetCenterOfMassShape = zolt.OffsetCenterOfMassShape;
const OffsetCenterOfMassShapeSettings = zolt.OffsetCenterOfMassShapeSettings;
const PhysicsMaterial = zolt.PhysicsMaterial;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const Real = zolt.Real;
const Ref = zolt.Ref;
const RotatedTranslatedShape = zolt.RotatedTranslatedShape;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const RVec3 = zolt.RVec3;
const ScaledShape = zolt.ScaledShape;
const ScaledShapeSettings = zolt.ScaledShapeSettings;
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
const TransformedShape = zolt.TransformedShape;
const TransformedShapeCollector = zolt.TransformedShapeCollector;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

/// The C++ reference functions, see DecoratedReference.cpp
const jolt = struct {
    extern fn jolt_decorated_create(desc: *const ShapeDesc, out_error: *[128]u8, out_info: *[2]u32) c_int;
    extern fn jolt_decorated_properties(desc: *const ShapeDesc, input: *const PropertiesInput, output: *PropertiesOutput) void;
    extern fn jolt_decorated_cast_ray(desc: *const ShapeDesc, input: *const RayInput, output: *RayOutput) void;
    extern fn jolt_decorated_collide_point(desc: *const ShapeDesc, point: *const P, creator: *const [2]u32, body_id: u32, reject_sub_type: u32, output: *PointOutput) void;
    extern fn jolt_decorated_collect(desc: *const ShapeDesc, input: *const CollectInput, output: *CollectOutput) void;
    extern fn jolt_decorated_collide(input: *const CollideInput, output: *HitsOutput) void;
    extern fn jolt_decorated_cast(input: *const CastInput, output: *HitsOutput) void;
    extern fn jolt_decorated_submerged_volume(desc: *const ShapeDesc, transform: *const [16]f32, scale: *const P, plane: *const [4]f32, out_values: *[5]f32) void;
    extern fn jolt_decorated_soft_body(desc: *const ShapeDesc, transform: *const [16]f32, scale: *const P, num_vertices: c_int, positions: [*]const f32, inv_masses: [*]const f32, io_penetrations: [*]f32, io_planes: [*]f32, io_indices: [*]c_int, colliding_shape_index: c_int) void;
    extern fn jolt_decorated_binary_state(desc: *const ShapeDesc, out_bytes: *[binary_capacity]u8, out_restored_bytes: *[binary_capacity]u8, out_children_bytes: *[binary_capacity]u8, out_restored_children_bytes: *[binary_capacity]u8, capacity: u32, out_sizes: *[4]u32) void;
};

/// Number of random inputs per test
const iterations = fw.iterations;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// No sub type is rejected by the filter
const no_reject: u32 = ~@as(u32, 0);

/// Must match cMaxTriangleCalls / cMaxTriangleVertices in DecoratedReference.cpp
const max_triangle_calls = 16;
const max_triangle_vertices = 384;

/// Size of the buffers of the binary state test
const binary_capacity = 512;

// ---------------------------------------------------------------------------------------------------------------------
// Shape descriptions, must match LeafDesc / DecoratorDesc / ShapeDesc in DecoratedReference.cpp

const LeafDesc = extern struct {
    /// 0: SphereShape, 1: BoxShape
    kind: u32 = 0,
    /// SphereShape
    radius: f32 = 1.0,
    /// BoxShape
    half_extent: P = .{ 1, 1, 1 },
    convex_radius: f32 = 0.0,
    density: f32 = 1000.0,
    user_data: u32 = 0,
};

const DecoratorDesc = extern struct {
    /// 0: ScaledShape, 1: RotatedTranslatedShape, 2: OffsetCenterOfMassShape
    kind: u32 = 0,
    /// 0: constructor, 1: settings with the inner shape, 2: settings with the inner settings, 3: settings without inner shape
    mode: u32 = 0,
    /// Scale / position / offset
    vector: P = .{ 1, 1, 1 },
    /// RotatedTranslatedShape
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
    user_data: u32 = 0,
};

const ShapeDesc = extern struct {
    leaf: LeafDesc = .{},
    /// decorators[0] wraps the leaf, decorators[1] wraps that, ...
    num_decorators: u32 = 0,
    decorators: [3]DecoratorDesc = @splat(.{}),
};

// ---------------------------------------------------------------------------------------------------------------------
// Inputs and outputs, must match the structs in DecoratedReference.cpp

const PropertiesInput = extern struct {
    /// A valid scale
    scale: P,
    /// IsValidScale / MakeScaleValid only
    any_scale: P,
    transform: [16]f32,
    /// DMat44 translation for GetWorldSpaceBounds(DMat44Arg, Vec3Arg)
    translation: [3]f64,
    /// GetSurfaceNormal
    point: P,
    /// GetSupportingFace
    direction: P,
    /// GetSubShapeTransformedShape
    position_com: P,
    rotation: [4]f32,
    /// GetLeafShape, GetSubShapeTransformedShape, GetSubShapeUserData
    sub_shape_id: u32,
};

const TSOutput = extern struct {
    position: [3]f64,
    rotation: [4]f32,
    scale: P,
    sub_type: u32,
    user_data: u32,
    body_id: u32,
    creator_id: u32,
    creator_bits: u32,
};

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
    num_triangles_recursive: u32,
    sub_shape_id_bits: u32,
    must_be_static: c_int,
    is_valid_scale: c_int,
    scale_valid: P,
    is_valid_any_scale: c_int,
    any_scale_valid: P,
    surface_normal: P,
    face_count: u32,
    face: [32 * 3]f32,
    /// GetLeafShape
    leaf_sub_type: u32,
    leaf_user_data: u32,
    leaf_remainder: u32,
    /// GetSubShapeUserData (low, high)
    sub_shape_user_data: [2]u32,
    material_is_default: c_int,
    /// SaveSubShapeState
    num_sub_shapes: u32,
    sub_shape_sub_type: u32,
    /// GetSubShapeTransformedShape
    child: TSOutput,
    child_remainder: u32,
    /// The top decorator: ScaledShape::GetScale / RotatedTranslatedShape::GetPosition, GetRotation, TransformScale(scale),
    /// TransformScale(any scale) / OffsetCenterOfMassShape::GetOffset
    decorator: [13]f32,
};

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
    /// Shape sub type that the filter rejects (no_reject: none)
    reject_sub_type: u32,
};

const RayHit = extern struct {
    fraction: f32,
    body_id: u32,
    sub_shape_id: u32,
};

const RayOutput = extern struct {
    hit: c_int,
    fraction: f32,
    sub_shape_id: u32,
    num_hits: u32,
    hits: [4]RayHit,
    filter_calls: u32,
    filter_hash: u32,
};

const PointOutput = extern struct {
    num_hits: u32,
    /// Of the last hit
    body_id: u32,
    sub_shape_id: u32,
    filter_calls: u32,
    filter_hash: u32,
};

const CollectInput = extern struct {
    box: [6]f32,
    position_com: P,
    rotation: [4]f32,
    scale: P,
    creator: [2]u32,
    body_id: u32,
    reject_sub_type: u32,
    /// TransformShape (may contain a scale)
    transform: [16]f32,
    /// TransformedShape::GetTrianglesStart
    base_offset: [3]f64,
    max_triangles_requested: c_int,
};

const CollectOutput = extern struct {
    num_collected: u32,
    filter_calls: u32,
    filter_hash: u32,
    collected: [2]TSOutput,
    num_transformed: u32,
    transformed: [2]TSOutput,
    triangle_calls: c_int,
    triangle_counts: [max_triangle_calls]c_int,
    triangle_vertices: [max_triangle_vertices * 3]f32,
    default_material: [max_triangle_vertices / 3]c_int,
};

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
    /// 1: collide with all edges
    active_edge_mode: c_int,
    active_edge_movement_direction: P,
    /// Early out fraction of the collector (when < FLT_MAX)
    early_out: f32,
    body_id: u32,
    reject_sub_type1: u32,
    reject_sub_type2: u32,
};

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
    reject_sub_type1: u32,
    reject_sub_type2: u32,
};

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

const HitsOutput = extern struct {
    num_hits: u32,
    filter_calls: u32,
    filter_hash: u32,
    hits: [2]HitOutput,
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

fn real(x: f64) Real {
    return if (Real == f64) x else @floatCast(x);
}

fn arrR3(v: RVec3) [3]f64 {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn makeCreator(c: [2]u32) SubShapeIDCreator {
    return SubShapeIDCreator.pushID(.{}, c[0], c[1]);
}

fn makeSubShapeID(value: u32) SubShapeID {
    var id: SubShapeID = .empty;
    id.setValue(value);
    return id;
}

fn storeFace(face: *const Shape.SupportingFace, out_count: *u32, out_face: *[32 * 3]f32) void {
    out_count.* = face.len;
    for (face.constSlice(), 0..) |v, i| out_face[3 * i ..][0..3].* = arr3(v);
}

fn storeTS(ts: *const TransformedShape) TSOutput {
    const shape = ts.shape.get();
    return .{
        .position = arrR3(ts.shape_position_com),
        .rotation = arr4(ts.shape_rotation.getXYZW()),
        .scale = arr3(ts.getShapeScale()),
        .sub_type = if (shape) |s| @intFromEnum(s.getSubType()) else no_reject,
        .user_data = if (shape) |s| @truncate(s.getUserData()) else 0,
        .body_id = ts.body_id.getIndexAndSequenceNumber(),
        .creator_id = ts.sub_shape_id_creator.getID().getValue(),
        .creator_bits = ts.sub_shape_id_creator.getNumBitsWritten(),
    };
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

// ---------------------------------------------------------------------------------------------------------------------
// Building the shapes (the same steps as BuildShape / BuildSettings in DecoratedReference.cpp)

fn userData(value: u32) u64 {
    return @as(u64, value) *% 0x100000001;
}

fn leafSettings(allocator: Allocator, leaf: LeafDesc) Allocator.Error!Ref(ShapeSettings) {
    var settings: *ShapeSettings = undefined;
    if (leaf.kind == 0) {
        const sphere = try SphereShapeSettings.create(allocator, leaf.radius, .{});
        sphere.base.density = leaf.density;
        settings = sphere.asShapeSettings();
    } else {
        const box = try BoxShapeSettings.create(allocator, vec3(leaf.half_extent), .{ .convex_radius = leaf.convex_radius });
        box.base.density = leaf.density;
        settings = box.asShapeSettings();
    }
    settings.user_data = userData(leaf.user_data);
    return .init(settings);
}

/// The inner part of a decorator's settings: the inner shape (const Shape *) or the inner settings (const ShapeSettings *)
const Inner = union(enum) {
    settings: ?*ShapeSettings,
    shape: ?*const Shape,
};

fn decoratorSettings(allocator: Allocator, d: DecoratorDesc, inner: Inner) Allocator.Error!Ref(ShapeSettings) {
    const settings: *ShapeSettings = switch (d.kind) {
        0 => switch (inner) {
            .settings => |s| (try ScaledShapeSettings.create(allocator, s, vec3(d.vector))).asShapeSettings(),
            .shape => |s| (try ScaledShapeSettings.createPtr(allocator, s, vec3(d.vector))).asShapeSettings(),
        },
        1 => switch (inner) {
            .settings => |s| (try RotatedTranslatedShapeSettings.create(allocator, vec3(d.vector), quat(d.rotation), s)).asShapeSettings(),
            .shape => |s| (try RotatedTranslatedShapeSettings.createPtr(allocator, vec3(d.vector), quat(d.rotation), s)).asShapeSettings(),
        },
        else => switch (inner) {
            .settings => |s| (try OffsetCenterOfMassShapeSettings.create(allocator, vec3(d.vector), s)).asShapeSettings(),
            .shape => |s| (try OffsetCenterOfMassShapeSettings.createPtr(allocator, vec3(d.vector), s)).asShapeSettings(),
        },
    };
    settings.user_data = userData(d.user_data);
    return .init(settings);
}

/// The settings of level `level` (-1 is the leaf)
fn buildSettings(allocator: Allocator, desc: *const ShapeDesc, level: i32) Allocator.Error!Ref(ShapeSettings) {
    if (level < 0)
        return leafSettings(allocator, desc.leaf);

    const d = desc.decorators[@intCast(level)];
    switch (d.mode) {
        2 => {
            var inner = try buildSettings(allocator, desc, level - 1);
            defer inner.deinit();
            return decoratorSettings(allocator, d, .{ .settings = inner.get() });
        },
        3 => return decoratorSettings(allocator, d, .{ .settings = null }),
        else => {
            var inner = try buildShape(allocator, desc, level - 1);
            defer inner.deinit();
            return decoratorSettings(allocator, d, .{ .shape = if (inner.isValid()) inner.getPtr() else null });
        },
    }
}

/// The shape of level `level` (-1 is the leaf)
fn buildShape(allocator: Allocator, desc: *const ShapeDesc, level: i32) Allocator.Error!ShapeResult {
    if (level < 0) {
        var settings = try leafSettings(allocator, desc.leaf);
        defer settings.deinit();
        return settings.get().?.createShape(allocator);
    }

    const d = desc.decorators[@intCast(level)];
    switch (d.mode) {
        0 => {
            var inner = try buildShape(allocator, desc, level - 1);
            if (inner.hasError())
                return inner;
            defer inner.deinit();
            const inner_shape = inner.getPtr().?;
            const shape: *Shape = switch (d.kind) {
                0 => (try ScaledShape.create(allocator, inner_shape, vec3(d.vector))).asShapeMut(),
                1 => (try RotatedTranslatedShape.create(allocator, vec3(d.vector), quat(d.rotation), inner_shape)).asShapeMut(),
                else => (try OffsetCenterOfMassShape.create(allocator, inner_shape, vec3(d.vector))).asShapeMut(),
            };
            shape.setUserData(userData(d.user_data));
            var result: ShapeResult = .empty;
            result.set(.init(shape));
            return result;
        },
        1 => {
            var inner = try buildShape(allocator, desc, level - 1);
            if (inner.hasError())
                return inner;
            defer inner.deinit();
            var settings = try decoratorSettings(allocator, d, .{ .shape = inner.getPtr() });
            defer settings.deinit();
            return settings.get().?.createShape(allocator);
        },
        else => {
            var settings = try buildSettings(allocator, desc, level);
            defer settings.deinit();
            return settings.get().?.createShape(allocator);
        },
    }
}

fn createShapeResult(allocator: Allocator, desc: *const ShapeDesc) Allocator.Error!ShapeResult {
    return buildShape(allocator, desc, @as(i32, @intCast(desc.num_decorators)) - 1);
}

/// The shape of a valid description
fn createShape(allocator: Allocator, desc: *const ShapeDesc) Allocator.Error!Ref(Shape) {
    var result = try createShapeResult(allocator, desc);
    defer result.deinit();
    return .init(result.getPtr().?);
}

// ---------------------------------------------------------------------------------------------------------------------
// The filter, must match DecoratedParityFilter in DecoratedReference.cpp

const FilterLog = struct {
    calls: u32 = 0,
    hash: u32 = 0x811c9dc5,

    fn add(self: *FilterLog, value: u32) void {
        self.hash = (self.hash ^ value) *% 0x01000193;
    }
};

const DecoratedParityFilter = struct {
    pub const overrides = .{ .shouldCollide, .shouldCollidePair };

    base: ShapeFilter = .init(@This()),
    reject_sub_type1: u32,
    reject_sub_type2: u32,
    /// The calls are logged behind a pointer (Rule M: the filter is const in the queries)
    log: *FilterLog,

    pub fn shouldCollide(self: *const DecoratedParityFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        self.log.calls += 1;
        self.log.add(@intFromEnum(shape2.getSubType()));
        self.log.add(sub_shape_id_of_shape2.getValue());
        return @intFromEnum(shape2.getSubType()) != self.reject_sub_type2;
    }

    pub fn shouldCollidePair(self: *const DecoratedParityFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        self.log.calls += 1;
        self.log.add(@intFromEnum(shape1.getSubType()));
        self.log.add(sub_shape_id_of_shape1.getValue());
        self.log.add(@intFromEnum(shape2.getSubType()));
        self.log.add(sub_shape_id_of_shape2.getValue());
        return @intFromEnum(shape1.getSubType()) != self.reject_sub_type1 and @intFromEnum(shape2.getSubType()) != self.reject_sub_type2;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of the wrappers

fn zoltProperties(allocator: Allocator, shape: *const Shape, input: *const PropertiesInput) !PropertiesOutput {
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
    var visited: Shape.VisitedShapes = .empty;
    defer visited.deinit(allocator);
    o.num_triangles_recursive = (try shape.getStatsRecursive(allocator, &visited)).num_triangles;
    o.sub_shape_id_bits = shape.getSubShapeIDBitsRecursive();
    o.must_be_static = @intFromBool(shape.mustBeStatic());
    o.is_valid_scale = @intFromBool(shape.isValidScale(scale));
    o.scale_valid = arr3(shape.makeScaleValid(scale));
    o.is_valid_any_scale = @intFromBool(shape.isValidScale(any_scale));
    o.any_scale_valid = arr3(shape.makeScaleValid(any_scale));
    o.surface_normal = arr3(shape.getSurfaceNormal(.empty, vec3(input.point)));
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(.empty, vec3(input.direction), scale, transform, &face);
    storeFace(&face, &o.face_count, &o.face);

    const id = makeSubShapeID(input.sub_shape_id);
    const leaf = shape.getLeafShape(id);
    o.leaf_sub_type = if (leaf.shape) |l| @intFromEnum(l.getSubType()) else no_reject;
    o.leaf_user_data = if (leaf.shape) |l| @truncate(l.getUserData()) else 0;
    o.leaf_remainder = leaf.remainder.getValue();
    const user_data = shape.getSubShapeUserData(id);
    o.sub_shape_user_data = .{ @truncate(user_data), @truncate(user_data >> 32) };
    o.material_is_default = @intFromBool(shape.getMaterial(.empty) == PhysicsMaterial.default);
    var sub_shapes: ShapeList = .empty;
    defer {
        for (sub_shapes.items) |*s| s.deinit();
        sub_shapes.deinit(allocator);
    }
    try shape.saveSubShapeState(allocator, &sub_shapes);
    o.num_sub_shapes = @intCast(sub_shapes.items.len);
    o.sub_shape_sub_type = if (sub_shapes.items.len == 0 or sub_shapes.items[0].get() == null) no_reject else @intFromEnum(sub_shapes.items[0].get().?.getSubType());

    var child = shape.getSubShapeTransformedShape(id, vec3(input.position_com), quat(input.rotation), scale);
    defer child.transformed_shape.deinit();
    o.child = storeTS(&child.transformed_shape);
    o.child_remainder = child.remainder.getValue();

    o.decorator = @splat(0);
    switch (shape.getSubType()) {
        .scaled => o.decorator[0..3].* = arr3(shape.cast(ScaledShape).getScale()),
        .rotated_translated => {
            const rt = shape.cast(RotatedTranslatedShape);
            o.decorator[0..3].* = arr3(rt.getPosition());
            o.decorator[3..7].* = arr4(rt.getRotation().getXYZW());
            o.decorator[7..10].* = arr3(rt.transformScale(scale));
            o.decorator[10..13].* = arr3(rt.transformScale(any_scale));
        },
        .offset_center_of_mass => o.decorator[0..3].* = arr3(shape.cast(OffsetCenterOfMassShape).getOffset()),
        else => {},
    }
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
    var log: FilterLog = .{};
    const filter: DecoratedParityFilter = .{ .reject_sub_type1 = no_reject, .reject_sub_type2 = input.reject_sub_type, .log = &log };
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
            shape.castRayCollector(ray, &settings, creator, &collector.base, &filter.base);
            try collector.checkError();
            for (collector.hits.items) |*h| store(&o, h);
        },
        1 => {
            var collector = AnyHitCollisionCollector(CastRayCollector).init();
            defer collector.deinit();
            collector.base.setContext(&context);
            if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
            shape.castRayCollector(ray, &settings, creator, &collector.base, &filter.base);
            if (collector.hadHit()) store(&o, &collector.hit);
        },
        else => {
            var collector = ClosestHitCollisionCollector(CastRayCollector).init();
            defer collector.deinit();
            collector.base.setContext(&context);
            if (input.early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(input.early_out);
            shape.castRayCollector(ray, &settings, creator, &collector.base, &filter.base);
            if (collector.hadHit()) store(&o, &collector.hit);
        },
    }
    o.filter_calls = log.calls;
    o.filter_hash = log.hash;
    return o;
}

fn zoltCollidePoint(allocator: Allocator, shape: *const Shape, point: P, creator: [2]u32, body_id: u32, reject_sub_type: u32) !PointOutput {
    var o = std.mem.zeroes(PointOutput);
    var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(body_id), .{});
    collector.base.setContext(&context);
    var log: FilterLog = .{};
    const filter: DecoratedParityFilter = .{ .reject_sub_type1 = no_reject, .reject_sub_type2 = reject_sub_type, .log = &log };
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

fn zoltCollect(allocator: Allocator, shape: *const Shape, input: *const CollectInput, o: *CollectOutput) !void {
    var collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer collector.deinit();
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    collector.base.setContext(&context);
    var log: FilterLog = .{};
    const filter: DecoratedParityFilter = .{ .reject_sub_type1 = no_reject, .reject_sub_type2 = input.reject_sub_type, .log = &log };
    shape.collectTransformedShapes(.init(vec3(input.box[0..3].*), vec3(input.box[3..6].*)), vec3(input.position_com), quat(input.rotation), vec3(input.scale), makeCreator(input.creator), &collector.base, &filter.base);
    try collector.checkError();
    o.num_collected = 0;
    for (collector.hits.items) |*ts| {
        if (o.num_collected < 2) {
            o.collected[o.num_collected] = storeTS(ts);
            o.num_collected += 1;
        }
    }
    o.filter_calls = log.calls;
    o.filter_hash = log.hash;

    var transformed = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer transformed.deinit();
    shape.transformShape(mat44(input.transform), &transformed.base);
    try transformed.checkError();
    o.num_transformed = 0;
    for (transformed.hits.items) |*ts| {
        if (o.num_transformed < 2) {
            o.transformed[o.num_transformed] = storeTS(ts);
            o.num_transformed += 1;
        }
    }

    // The triangles of the first collected leaf
    o.triangle_calls = 0;
    if (collector.hits.items.len > 0) {
        const ts = &collector.hits.items[0];
        var triangle_context: Shape.GetTrianglesContext = .{};
        ts.getTrianglesStart(&triangle_context, AABox.biggest(), RVec3.init(real(input.base_offset[0]), real(input.base_offset[1]), real(input.base_offset[2])));
        const max_requested: u32 = @intCast(input.max_triangles_requested);
        const triangles = try allocator.alloc(Float3, 3 * max_requested);
        defer allocator.free(triangles);
        const materials = try allocator.alloc(*const PhysicsMaterial, max_requested);
        defer allocator.free(materials);
        var out: usize = 0;
        var out_material: usize = 0;
        var num_vertices: usize = 0;
        while (true) {
            const count = ts.getTrianglesNext(&triangle_context, max_requested, triangles, materials);
            o.triangle_counts[@intCast(o.triangle_calls)] = @intCast(count);
            o.triangle_calls += 1;
            var i: usize = 0;
            while (i < 3 * count and num_vertices < max_triangle_vertices) : ({
                i += 1;
                num_vertices += 1;
                out += 3;
            }) {
                o.triangle_vertices[out..][0..3].* = .{ triangles[i].x, triangles[i].y, triangles[i].z };
            }
            for (materials[0..count]) |m| {
                if (out_material >= max_triangle_vertices / 3) break;
                o.default_material[out_material] = @intFromBool(m == PhysicsMaterial.default);
                out_material += 1;
            }
            if (count == 0 or o.triangle_calls == max_triangle_calls) break;
        }
    }
}

fn zoltCollide(allocator: Allocator, input: *const CollideInput) !HitsOutput {
    var shape1 = try createShape(allocator, &input.shape1);
    defer shape1.deinit();
    var shape2 = try createShape(allocator, &input.shape2);
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
    var log: FilterLog = .{};
    const filter: DecoratedParityFilter = .{ .reject_sub_type1 = input.reject_sub_type1, .reject_sub_type2 = input.reject_sub_type2, .log = &log };
    CollisionDispatch.collideShapeVsShape(shape1.get().?, shape2.get().?, vec3(input.scale1), vec3(input.scale2), mat44(input.transform1), mat44(input.transform2), makeCreator(input.creator1), makeCreator(input.creator2), &settings, &collector.base, &filter.base);
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
    o.filter_calls = log.calls;
    o.filter_hash = log.hash;
    return o;
}

fn zoltCast(allocator: Allocator, input: *const CastInput) !HitsOutput {
    var shape1 = try createShape(allocator, &input.shape1);
    defer shape1.deinit();
    var shape2 = try createShape(allocator, &input.shape2);
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
    var log: FilterLog = .{};
    const filter: DecoratedParityFilter = .{ .reject_sub_type1 = input.reject_sub_type1, .reject_sub_type2 = input.reject_sub_type2, .log = &log };
    CollisionDispatch.castShapeVsShapeWorldSpace(&shape_cast, &settings, shape2.get().?, vec3(input.scale2), &filter.base, mat44(input.transform2), makeCreator(input.creator1), makeCreator(input.creator2), &collector.base);
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
    o.filter_calls = log.calls;
    o.filter_hash = log.hash;
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

/// The sub types that the filters reject now and then
const reject_candidates = [_]u32{ @intFromEnum(zolt.ShapeSubType.sphere), @intFromEnum(zolt.ShapeSubType.box), @intFromEnum(zolt.ShapeSubType.scaled), @intFromEnum(zolt.ShapeSubType.rotated_translated), @intFromEnum(zolt.ShapeSubType.offset_center_of_mass) };

/// Input generator: xorshift32 with helpers for the edge cases
const Gen = struct {
    rng: fw.Rng = .{},
    allocator: Allocator,

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

    /// A unit quaternion, sometimes the identity or a rotation of a multiple of 90 degrees around an axis
    fn rotation(self: *Gen) Quat {
        switch (self.index(6)) {
            0 => return Quat.identity(),
            1 => {
                const axes = [_]Vec3{ Vec3.axisX(), Vec3.axisY(), Vec3.axisZ() };
                return Quat.rotation(axes[self.index(3)], @as(f32, @floatFromInt(self.index(4))) * 0.5 * math.pi);
            },
            else => while (true) {
                const q = self.rng.floatArray(4, -1, 1);
                const len_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
                if (len_sq > 1.0e-2 and len_sq <= 1.0) return quat(q).normalized();
            },
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

    fn reject(self: *Gen) u32 {
        return if (self.oneIn(4)) reject_candidates[self.index(reject_candidates.len)] else no_reject;
    }

    /// A valid leaf: a sphere or a box (sometimes flat, with or without convex radius)
    fn leaf(self: *Gen) LeafDesc {
        const density = if (self.oneIn(4)) 1000.0 else self.plain(1, 3000);
        const user_data = self.next();
        if (self.oneIn(2))
            return .{ .kind = 0, .radius = if (self.oneIn(5)) 1.0 else self.plain(0.05, 2.5), .density = density, .user_data = user_data };
        var half_extent = self.plainVec(0.05, 2.5);
        if (self.oneIn(5)) half_extent = .{ 1, 1, 1 };
        var convex_radius: f32 = switch (self.index(3)) {
            0 => 0.0,
            1 => 0.05, // cDefaultConvexRadius
            else => self.plain(0, 0.2),
        };
        if (self.oneIn(10)) {
            half_extent[self.index(3)] = 0.0; // Flat box
            convex_radius = 0.0;
        }
        convex_radius = @min(convex_radius, @min(half_extent[0], @min(half_extent[1], half_extent[2])));
        return .{ .kind = 1, .half_extent = half_extent, .convex_radius = convex_radius, .density = density, .user_data = user_data };
    }

    /// A scale: uniform (with or without signs), non-uniform, mirrored, identity
    fn scaleCandidate(self: *Gen) P {
        return switch (self.index(6)) {
            0 => .{ 1, 1, 1 },
            1 => blk: {
                const s = if (self.oneIn(2)) self.grid(3) else self.plain(0.2, 2.5);
                const m = if (s == 0.0) 1.0 else @abs(s);
                break :blk if (self.oneIn(2)) .{ m, m, m } else .{ -m, -m, -m };
            },
            2 => blk: {
                const m = self.plain(0.2, 2.5);
                break :blk .{ if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m, if (self.oneIn(2)) m else -m };
            },
            3 => .{ self.grid(2), self.grid(2), self.grid(2) },
            else => blk: {
                var r = self.plainVec(0.2, 2.5);
                for (&r) |*c| {
                    if (self.oneIn(3)) c.* = -c.*;
                }
                break :blk r;
            },
        };
    }

    /// Any scale (for IsValidScale / MakeScaleValid): random, uniform, tiny / zero / negative components
    fn anyScale(self: *Gen) P {
        return switch (self.index(6)) {
            0 => self.vec(-3, 3),
            1 => blk: {
                const s = self.float(-3, 3);
                break :blk .{ s, if (self.oneIn(2)) s else -s, s };
            },
            2 => .{ 1.0 + self.plain(-1.0e-4, 1.0e-4), 1.0, 1.0 + self.plain(-1.0e-5, 1.0e-5) },
            3 => .{ self.float(-1.0e-5, 1.0e-5), self.plain(0.1, 2), self.plain(-2, -0.1) },
            4 => self.scaleCandidate(),
            else => .{ self.grid(2), self.grid(2), self.grid(2) },
        };
    }

    /// A scale that is valid for `shape` (see makeValid)
    fn validScale(self: *Gen, shape: *const Shape) P {
        return makeValid(shape, self.scaleCandidate());
    }

    /// The parameters of a decorator of kind `kind` (the scale of a scaled shape is made valid later)
    fn decorator(self: *Gen, kind: u32, mode: u32) DecoratorDesc {
        var d: DecoratorDesc = .{ .kind = kind, .mode = mode, .user_data = self.next() };
        switch (kind) {
            0 => d.vector = self.scaleCandidate(),
            1 => {
                d.vector = if (self.oneIn(5)) .{ 0, 0, 0 } else self.vec(-3, 3);
                d.rotation = arr4(self.rotation().getXYZW());
            },
            else => d.vector = if (self.oneIn(5)) .{ 0, 0, 0 } else self.vec(-2, 2),
        }
        return d;
    }

    /// A valid shape description with `min_decorators` to 3 decorators: the scale of every scaled shape is valid for
    /// its inner shape
    fn shapeDesc(self: *Gen, min_decorators: u32) !ShapeDesc {
        var desc: ShapeDesc = .{ .leaf = self.leaf() };
        const n: u32 = min_decorators + @as(u32, @intCast(self.index(4 - min_decorators)));
        for (0..n) |i| {
            const kind: u32 = @intCast(self.index(3));
            var d = self.decorator(kind, @intCast(self.index(3)));
            if (kind == 0) {
                // Make the scale valid for the inner shape (built so far)
                desc.num_decorators = @intCast(i);
                var inner = try createShape(self.allocator, &desc);
                defer inner.deinit();
                d.vector = makeValid(inner.get().?, d.vector);
                if (zolt.ScaleHelpers.isZeroScale(vec3(d.vector)))
                    d.vector = .{ 1, 1, 1 }; // MakeScaleValid of a scaled shape can return a scale below ScaleHelpers::cMinScale, which the scaled shape refuses ("Can't use zero scale!")
            }
            desc.decorators[i] = d;
        }
        desc.num_decorators = n;
        return desc;
    }

    /// A shape description for the settings test: invalid leaves, zero scales, missing inner shapes
    fn anyShapeDesc(self: *Gen) ShapeDesc {
        var desc: ShapeDesc = .{ .leaf = self.leaf() };
        if (self.oneIn(3)) {
            desc.leaf.radius = self.float(-1, 1);
            desc.leaf.half_extent = self.vec(-0.5, 2);
            desc.leaf.convex_radius = self.float(-0.5, 1);
        }
        desc.num_decorators = @intCast(self.index(4));
        for (0..desc.num_decorators) |i| {
            var d = self.decorator(@intCast(self.index(3)), if (self.oneIn(8)) 3 else @intCast(self.index(3)));
            if (d.kind == 0 and self.oneIn(4)) d.vector[self.index(3)] = if (self.oneIn(2)) 0.0 else self.plain(-1.0e-6, 1.0e-6); // Zero scale
            if (d.kind == 0 and d.mode == 0 and zolt.ScaleHelpers.isZeroScale(vec3(d.vector))) d.vector = .{ 1, 1, 1 }; // The constructor asserts
            desc.decorators[i] = d;
        }
        return desc;
    }
};

/// `scale` made valid for `shape` with MakeScaleValid (the identity as last resort)
fn makeValid(shape: *const Shape, scale: P) P {
    var s = vec3(scale);
    if (!shape.isValidScale(s))
        s = shape.makeScaleValid(s);
    if (!shape.isValidScale(s))
        s = Vec3.one();
    return arr3(s);
}

/// The extent of a shape (for placing shapes near each other)
fn extentOf(allocator: Allocator, desc: *const ShapeDesc, scale: P) !f32 {
    var shape = try createShape(allocator, desc);
    defer shape.deinit();
    const bounds = shape.get().?.getLocalBounds().scaled(vec3(scale));
    return @max(@max(bounds.min.abs().reduceMax(), bounds.max.abs().reduceMax()), 0.05);
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

test "Decorated parity: settings and Jolt's error texts" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "settings" };
    var num_valid: usize = 0;
    for (0..iterations / 5) |_| {
        const desc = gen.anyShapeDesc();
        var jolt_error: [128]u8 = undefined;
        var jolt_info: [2]u32 = undefined;
        const jolt_valid = jolt.jolt_decorated_create(&desc, &jolt_error, &jolt_info);

        var zolt_error: [128]u8 = @splat(0);
        var zolt_info: [2]u32 = .{ 0, 0 };
        var result = try createShapeResult(allocator, &desc);
        defer result.deinit();
        const zolt_valid: c_int = @intFromBool(result.isValid());
        if (result.isValid()) {
            zolt_info = .{ @intFromEnum(result.getPtr().?.getSubType()), @truncate(result.getPtr().?.getUserData()) };
            num_valid += 1;
        } else {
            const text = result.getError();
            @memcpy(zolt_error[0..@min(text.len, 127)], text[0..@min(text.len, 127)]);
        }
        checker.check(.{desc}, .{ zolt_valid, zolt_error, zolt_info }, .{ jolt_valid, jolt_error, jolt_info });
    }
    try checker.finish();
    try expectBetween("valid settings", num_valid, iterations / 20, iterations / 5 - iterations / 20);
}

test "Decorated parity: bounds, mass properties, volume, scales, surface normal, supporting face, sub shapes" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "properties" };
    for (0..iterations / 2) |_| {
        const desc = try gen.shapeDesc(1);
        var shape = try createShape(allocator, &desc);
        defer shape.deinit();
        const input: PropertiesInput = .{
            .scale = gen.validScale(shape.get().?),
            .any_scale = gen.anyScale(),
            .transform = arr16(gen.transform(10)),
            .translation = .{ gen.plain(-1.0e5, 1.0e5), gen.plain(-10, 10), gen.plain(-10, 10) },
            .point = gen.vec(-4, 4),
            .direction = gen.direction(3),
            .position_com = gen.vec(-5, 5),
            .rotation = arr4(gen.rotation().getXYZW()),
            .sub_shape_id = if (gen.oneIn(3)) SubShapeID.empty.getValue() else gen.next(),
        };
        var jolt_output = std.mem.zeroes(PropertiesOutput);
        jolt.jolt_decorated_properties(&desc, &input, &jolt_output);
        const zolt_output = try zoltProperties(allocator, shape.get().?, &input);
        checker.check(.{ desc, input }, zolt_output, jolt_output);
    }
    try checker.finish();
}

test "Decorated parity: CastRay (single hit and collectors) and CollidePoint" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var rays: Checker = .{ .name = "cast ray" };
    var points: Checker = .{ .name = "collide point" };
    var num_ray_hits: usize = 0;
    var num_point_hits: usize = 0;
    for (0..iterations / 2) |_| {
        const desc = try gen.shapeDesc(1);
        var shape = try createShape(allocator, &desc);
        defer shape.deinit();
        const extent = try extentOf(allocator, &desc, .{ 1, 1, 1 });
        const center = arr3(shape.get().?.getLocalBounds().getCenter());

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
            .reject_sub_type = gen.reject(),
        };
        for (&input.origin, center) |*o, c| o.* += c;
        if (gen.oneIn(2)) {
            // Aim at the center
            const target = vec3(center).add(vec3(gen.plainVec(-0.5 * extent, 0.5 * extent)));
            input.direction = arr3(target.sub(vec3(input.origin)).mulScalar(gen.plain(0.5, 3)));
        }
        var jolt_output = std.mem.zeroes(RayOutput);
        jolt.jolt_decorated_cast_ray(&desc, &input, &jolt_output);
        const zolt_output = try zoltCastRay(allocator, shape.get().?, &input);
        num_ray_hits += zolt_output.num_hits;
        rays.check(.{ desc, input }, zolt_output, jolt_output);

        // Points inside and outside, often near the surface of the bounding box
        var point = gen.vec(-1.5 * extent, 1.5 * extent);
        if (gen.oneIn(2)) {
            const bounds = shape.get().?.getLocalBounds();
            point = arr3(bounds.getExtent().mul(vec3(gen.plainVec(-1.2, 1.2))));
            const axis = gen.index(3);
            const e = bounds.getExtent().getComponent(@intCast(axis)) * gen.plain(0.9, 1.1);
            point[axis] = if (gen.oneIn(2)) e else -e;
        }
        for (&point, center) |*p, c| p.* += c;
        var jolt_point = std.mem.zeroes(PointOutput);
        jolt.jolt_decorated_collide_point(&desc, &point, &input.creator, input.body_id, input.reject_sub_type, &jolt_point);
        const zolt_point = try zoltCollidePoint(allocator, shape.get().?, point, input.creator, input.body_id, input.reject_sub_type);
        num_point_hits += zolt_point.num_hits;
        points.check(.{ desc, point, input.creator, input.reject_sub_type }, zolt_point, jolt_point);
    }
    try finishAll(&.{ &rays, &points });
    try expectBetween("ray hits", num_ray_hits, iterations / 10, iterations);
    try expectBetween("point hits", num_point_hits, iterations / 40, iterations / 4);
}

test "Decorated parity: CollectTransformedShapes, TransformShape and the triangles of the leaves" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "collect" };
    for (0..iterations / 10) |_| {
        const desc = try gen.shapeDesc(1);
        var shape = try createShape(allocator, &desc);
        defer shape.deinit();
        var transform = gen.transform(5);
        if (gen.oneIn(2)) transform = transform.mul(Mat44.scaleVec3(vec3(gen.scaleCandidate())));
        const input: CollectInput = .{
            .box = if (gen.oneIn(2)) boxArr(AABox.biggest()) else boxArr(.init(vec3(gen.vec(-5, 0)), vec3(gen.vec(0, 5)))),
            .position_com = gen.vec(-5, 5),
            .rotation = arr4(gen.rotation().getXYZW()),
            .scale = gen.validScale(shape.get().?),
            .creator = gen.creator(),
            .body_id = gen.next() & 0x7fffff,
            .reject_sub_type = gen.reject(),
            .transform = arr16(transform),
            .base_offset = if (gen.oneIn(2)) .{ 0, 0, 0 } else .{ gen.plain(-10, 10), gen.plain(-10, 10), gen.plain(-10, 10) },
            .max_triangles_requested = if (gen.oneIn(2)) 32 else @intCast(32 + gen.index(200)),
        };
        var jolt_output = std.mem.zeroes(CollectOutput);
        jolt.jolt_decorated_collect(&desc, &input, &jolt_output);
        var zolt_output = std.mem.zeroes(CollectOutput);
        try zoltCollect(allocator, shape.get().?, &input, &zolt_output);
        checker.check(.{ desc, input }, zolt_output, jolt_output);
    }
    try checker.finish();
}

/// Checks that a test exercised both paths (e.g. hits and misses), prints the count when not
fn expectBetween(name: []const u8, value: usize, min: usize, max: usize) !void {
    if (value <= min or value >= max) {
        std.debug.print("{s}: {d} is not in ({d}, {d})\n", .{ name, value, min, max });
        return error.TestUnexpectedResult;
    }
}

/// The other shape of a collision or cast: a sphere, a box or another decorated shape
fn otherDesc(gen: *Gen) !ShapeDesc {
    return switch (gen.index(3)) {
        0, 1 => .{ .leaf = gen.leaf() },
        else => try gen.shapeDesc(1),
    };
}

/// Decorated shapes that exactly touch a sphere or a box (shape 2 translated by translation2)
const TouchingCase = struct { shape1: ShapeDesc, shape2: ShapeDesc, translation2: P };
const touching_cases = [_]TouchingCase{
    // A sphere of radius 0.5 scaled by 2 touches a unit sphere at distance 2
    .{ .shape1 = .{ .leaf = .{ .kind = 0, .radius = 0.5 }, .num_decorators = 1, .decorators = .{ .{ .kind = 0, .vector = .{ 2, 2, 2 } }, .{}, .{} } }, .shape2 = .{ .leaf = .{ .kind = 0, .radius = 1 } }, .translation2 = .{ 2, 0, 0 } },
    // A unit box (no convex radius) rotated by 90 degrees around Y touches a unit box along X
    .{ .shape1 = .{ .leaf = .{ .kind = 1, .half_extent = .{ 1, 1, 1 } }, .num_decorators = 1, .decorators = .{ .{ .kind = 1, .vector = .{ 3, 0, 0 }, .rotation = .{ 0, 0.70710677, 0, 0.70710677 } }, .{}, .{} } }, .shape2 = .{ .leaf = .{ .kind = 1, .half_extent = .{ 1, 1, 1 } } }, .translation2 = .{ 2, 0, 0 } },
    // A unit sphere with its center of mass moved by (1, 0, 0) is centered at (-1, 0, 0), it touches a unit sphere at (1, 0, 0)
    .{ .shape1 = .{ .leaf = .{ .kind = 0, .radius = 1 }, .num_decorators = 1, .decorators = .{ .{ .kind = 2, .vector = .{ 1, 0, 0 } }, .{}, .{} } }, .shape2 = .{ .leaf = .{ .kind = 0, .radius = 1 } }, .translation2 = .{ 1, 0, 0 } },
    // A unit box scaled by (1, 2, 1) touches a unit box along Y, both decorated
    .{ .shape1 = .{ .leaf = .{ .kind = 1, .half_extent = .{ 1, 1, 1 } }, .num_decorators = 1, .decorators = .{ .{ .kind = 0, .mode = 1, .vector = .{ 1, 2, 1 } }, .{}, .{} } }, .shape2 = .{ .leaf = .{ .kind = 1, .half_extent = .{ 1, 1, 1 } }, .num_decorators = 1, .decorators = .{ .{ .kind = 1, .mode = 2 }, .{}, .{} } }, .translation2 = .{ 0, 3, 0 } },
};

test "Decorated parity: collide with a decorated shape on either side through CollisionDispatch" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "collide" };
    var num_hits: usize = 0;
    for (0..iterations / 2) |i| {
        var shape1 = try gen.shapeDesc(1);
        var shape2 = try otherDesc(&gen);
        if (gen.oneIn(2)) std.mem.swap(ShapeDesc, &shape1, &shape2);
        if (i % 7 == 0) shape2 = shape1; // Against itself
        var s1 = try createShape(allocator, &shape1);
        defer s1.deinit();
        var s2 = try createShape(allocator, &shape2);
        defer s2.deinit();
        const scale1 = gen.validScale(s1.get().?);
        const scale2 = gen.validScale(s2.get().?);
        const transform1 = gen.transform(5);
        // Place shape 2 near shape 1 so that about half of the pairs collide
        const reach = (try extentOf(allocator, &shape1, scale1)) + (try extentOf(allocator, &shape2, scale2));
        var relative = gen.transform(reach);
        if (gen.oneIn(10)) relative.setTranslation(Vec3.zero()); // Same center
        var input: CollideInput = .{
            .shape1 = shape1,
            .shape2 = shape2,
            .scale1 = scale1,
            .scale2 = scale2,
            .transform1 = arr16(transform1),
            .transform2 = arr16(transform1.mul(relative)),
            .creator1 = gen.creator(),
            .creator2 = gen.creator(),
            .max_separation_distance = switch (gen.index(4)) {
                0, 1 => 0.0,
                2 => gen.plain(0, 1),
                else => gen.plain(1, 3),
            },
            .collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance,
            .penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance,
            .collect_faces = @intFromBool(gen.oneIn(2)),
            .back_face_mode = @intFromBool(gen.oneIn(2)),
            .active_edge_mode = @intFromBool(gen.oneIn(2)),
            .active_edge_movement_direction = if (gen.oneIn(2)) .{ 0, 0, 0 } else gen.vec(-1, 1),
            .early_out = if (gen.oneIn(4)) gen.plain(-2, 2) else math.flt_max,
            .body_id = gen.next() & 0x7fffff,
            .reject_sub_type1 = gen.reject(),
            .reject_sub_type2 = gen.reject(),
        };
        if (i < touching_cases.len) {
            // Exactly touching shapes
            input.shape1 = touching_cases[i].shape1;
            input.shape2 = touching_cases[i].shape2;
            input.scale1 = .{ 1, 1, 1 };
            input.scale2 = .{ 1, 1, 1 };
            input.transform1 = arr16(Mat44.identity());
            input.transform2 = arr16(Mat44.translation(vec3(touching_cases[i].translation2)));
            input.max_separation_distance = 0.0;
            input.early_out = math.flt_max;
            input.reject_sub_type1 = no_reject;
            input.reject_sub_type2 = no_reject;
        }
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_decorated_collide(&input, &jolt_output);
        const zolt_output = try zoltCollide(allocator, &input);
        num_hits += zolt_output.num_hits;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try expectBetween("collide hits", num_hits, iterations / 10, 2 * iterations / 5);
}

test "Decorated parity: cast with a decorated shape on either side through CollisionDispatch (world space)" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "cast" };
    var num_hits: usize = 0;
    for (0..iterations / 2) |i| {
        var shape1 = try gen.shapeDesc(1);
        var shape2 = try otherDesc(&gen);
        if (gen.oneIn(2)) std.mem.swap(ShapeDesc, &shape1, &shape2);
        if (i % 7 == 0) shape2 = shape1; // Against itself
        var s1 = try createShape(allocator, &shape1);
        defer s1.deinit();
        var s2 = try createShape(allocator, &shape2);
        defer s2.deinit();
        const scale1 = gen.validScale(s1.get().?);
        const scale2 = gen.validScale(s2.get().?);
        const transform2 = gen.transform(5);
        const reach = (try extentOf(allocator, &shape1, scale1)) + (try extentOf(allocator, &shape2, scale2));
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
            .back_face_mode_triangles = @intFromBool(gen.oneIn(2)),
            .back_face_mode_convex = @intFromBool(gen.oneIn(2)),
            .active_edge_mode = @intFromBool(gen.oneIn(2)),
            .use_shrunken_shape = @intFromBool(gen.oneIn(2)),
            .return_deepest_point = @intFromBool(gen.oneIn(2)),
            .collect_faces = @intFromBool(gen.oneIn(2)),
            .early_out = if (gen.oneIn(4)) gen.plain(-0.5, 1) else 2.0,
            .body_id = gen.next() & 0x7fffff,
            .reject_sub_type1 = gen.reject(),
            .reject_sub_type2 = gen.reject(),
        };
        var jolt_output = std.mem.zeroes(HitsOutput);
        jolt.jolt_decorated_cast(&input, &jolt_output);
        const zolt_output = try zoltCast(allocator, &input);
        num_hits += zolt_output.num_hits;
        checker.check(.{input}, zolt_output, jolt_output);
    }
    try checker.finish();
    try expectBetween("cast hits", num_hits, iterations / 20, 2 * iterations / 5);
}

test "Decorated parity: GetSubmergedVolume" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "submerged volume" };
    for (0..iterations / 2) |_| {
        const desc = try gen.shapeDesc(1);
        var shape = try createShape(allocator, &desc);
        defer shape.deinit();
        const scale = gen.validScale(shape.get().?);
        const transform = arr16(gen.transform(3));
        const normal = vec3(gen.direction(1)).normalizedOr(Vec3.axisY());
        const plane = arr4(Plane.fromPointAndNormal(vec3(gen.vec(-5, 5)), normal).normal_and_constant);
        var jolt_values: [5]f32 = undefined;
        jolt.jolt_decorated_submerged_volume(&desc, &transform, &scale, &plane, &jolt_values);
        const r = shape.get().?.getSubmergedVolume(mat44(transform), vec3(scale), .fromVec4(vec4(plane)));
        const zolt_values = [_]f32{ r.total_volume, r.submerged_volume } ++ arr3(r.center_of_buoyancy);
        checker.check(.{ desc, scale, transform, plane }, zolt_values, jolt_values);
    }
    try checker.finish();
}

test "Decorated parity: CollideSoftBodyVertices" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "soft body vertices" };
    const n = 8;
    for (0..iterations / 4) |_| {
        const desc = try gen.shapeDesc(1);
        var shape = try createShape(allocator, &desc);
        defer shape.deinit();
        const scale = gen.validScale(shape.get().?);
        const transform = arr16(gen.transform(3));
        const extent = (try extentOf(allocator, &desc, scale)) * 1.5 + 1.0;
        var positions: [n * 3]f32 = undefined;
        var inv_masses: [n]f32 = undefined;
        var penetrations: [n]f32 = undefined;
        const planes: [n * 4]f32 = @splat(0);
        const indices: [n]c_int = @splat(-1);
        for (0..n) |v| {
            positions[3 * v ..][0..3].* = arr3(mat44(transform).mulVec3(vec3(gen.vec(-extent, extent))));
            if (gen.oneIn(8)) positions[3 * v ..][0..3].* = arr3(mat44(transform).getTranslation()); // At the center of mass
            inv_masses[v] = if (gen.oneIn(5)) 0.0 else 1.0;
            penetrations[v] = if (gen.oneIn(2)) -math.flt_max else gen.plain(-2, 2);
        }
        var jolt_penetrations = penetrations;
        var jolt_planes = planes;
        var jolt_indices = indices;
        jolt.jolt_decorated_soft_body(&desc, &transform, &scale, n, &positions, &inv_masses, &jolt_penetrations, &jolt_planes, &jolt_indices, 3);

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

test "Decorated parity: binary state, sub shape state, SaveWithChildren and the restores" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "binary state" };
    for (0..iterations / 10) |_| {
        const desc = try gen.shapeDesc(1);
        var jolt_output: BinaryOutput = .{};
        jolt.jolt_decorated_binary_state(&desc, &jolt_output.bytes[0], &jolt_output.bytes[1], &jolt_output.bytes[2], &jolt_output.bytes[3], binary_capacity, &jolt_output.sizes);
        var shape = try createShape(allocator, &desc);
        defer shape.deinit();
        const zolt_output = try zoltBinaryState(allocator, shape.get().?);
        checker.check(.{desc}, .{ zolt_output.sizes, zolt_output.bytes }, .{ jolt_output.sizes, jolt_output.bytes });
        try std.testing.expect(zolt_output.sizes[1] == zolt_output.sizes[0] and zolt_output.sizes[3] == zolt_output.sizes[2]); // The restores succeed
    }
    try checker.finish();
}
