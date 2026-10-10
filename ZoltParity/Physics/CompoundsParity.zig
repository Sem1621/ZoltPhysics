//! Parity tests for the concrete compound shapes (Phase 4, Wave B): StaticCompoundShape, MutableCompoundShape and
//! Shape::ScaleShape. A compound is described by `CompoundDesc`: up to 4 leaves (SphereShape / BoxShape settings shared by
//! the sub shapes), an optional nested compound, 1 to 200 sub shapes (a leaf or the nested compound, as is or wrapped in
//! a RotatedTranslatedShape / ScaledShape, added as a shape or as settings), for a MutableCompoundShape a list of
//! mutations (AddShape at any index, RemoveShape, ModifyShape, ModifyShape with a shape, ModifyShapes with a stride,
//! AdjustCenterOfMass) and Clone, for a StaticCompoundShape the creation with a TempAllocator, and a chain of 2 sub shape
//! compounds around it (to exceed the sub shape ID bits). The same construction is done on both sides
//! (`CompoundBuilder` here and in CompoundsReference.cpp).
//!
//! Compared bit for bit on random inputs mixed with edge cases: the results and Jolt's error texts of the settings
//! (no sub shapes, a single sub shape: the shape itself or a RotatedTranslatedShape, child errors, too deep
//! hierarchies), GetLocalBounds, GetWorldSpaceBounds (Mat44 and DMat44), GetCenterOfMass, GetInnerRadius,
//! GetMassProperties, GetVolume, GetStatsRecursive (triangles), GetSubShapeIDBitsRecursive, MustBeStatic, IsValidScale /
//! MakeScaleValid, GetSurfaceNormal / GetSupportingFace / GetLeafShape / GetSubShapeTransformedShape / GetMaterial of a
//! leaf, GetLeafShape / GetSubShapeUserData / IsSubShapeIDValid of any sub shape ID, GetSubShapeIndexFromID,
//! SaveSubShapeState, every sub shape (position, rotation, user data, IsValidScale / TransformScale), the number of tree
//! nodes / bounds blocks, GetIntersectingSubShapes (AABox and OrientedBox, the indices in visiting order), CastRay (both
//! overloads, all hits in visiting order, AnyHit / ClosestHit collectors, early out fractions), CollidePoint (all hits in
//! order, AnyHit), CollectTransformedShapes (in order), TransformShape, CollisionDispatch::sCollideShapeVsShape and
//! sCastShapeVsShapeWorldSpace with the compound on either side or both sides against spheres, boxes and compounds (all
//! settings, all hits in order and the other collectors), GetSubmergedVolume, CollideSoftBodyVertices, the binary state
//! (SaveBinaryState with the tree nodes / bounds blocks, sRestoreFromBinaryState + RestoreSubShapeState,
//! SaveWithChildren, sRestoreWithChildren) and Shape::ScaleShape (the result's SaveWithChildren). Every query with a
//! ShapeFilter uses `CompoundsParityFilter`, which rejects a shape sub type and a sub shape ID and hashes the arguments
//! of every call it receives. C ABI wrappers: ZoltParity/Physics/CompoundsReference.cpp.

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
const CollidePointResult = zolt.CollidePointResult;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollideSoftBodyVertexIterator = zolt.CollideSoftBodyVertexIterator;
const CollisionDispatch = zolt.CollisionDispatch;
const CompoundShape = zolt.CompoundShape;
const CompoundShapeSettings = zolt.CompoundShapeSettings;
const DecoratedShape = zolt.DecoratedShape;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Mat44 = zolt.Mat44;
const MutableCompoundShape = zolt.MutableCompoundShape;
const MutableCompoundShapeSettings = zolt.MutableCompoundShapeSettings;
const OrientedBox = zolt.OrientedBox;
const PhysicsMaterial = zolt.PhysicsMaterial;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RotatedTranslatedShape = zolt.RotatedTranslatedShape;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const RVec3 = zolt.RVec3;
const ScaledShape = zolt.ScaledShape;
const ScaledShapeSettings = zolt.ScaledShapeSettings;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastResult = zolt.ShapeCastResult;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeFilter = zolt.ShapeFilter;
const ShapeList = zolt.ShapeList;
const ShapeResult = zolt.ShapeResult;
const ShapeSettings = zolt.ShapeSettings;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StaticCompoundShape = zolt.StaticCompoundShape;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const StridedPtrConst = zolt.StridedPtrConst;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const TransformedShape = zolt.TransformedShape;
const TransformedShapeCollector = zolt.TransformedShapeCollector;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

/// The C++ reference functions, see CompoundsReference.cpp
const jolt = struct {
    extern fn jolt_compounds_create(desc: *const CompoundDesc, out_error: *[128]u8, out_info: *[4]u32) c_int;
    extern fn jolt_compounds_properties(desc: *const CompoundDesc, input: *const PropertiesInput, output: *PropertiesOutput) void;
    extern fn jolt_compounds_cast_ray(desc: *const CompoundDesc, input: *const RayInput, output: *RayOutput) void;
    extern fn jolt_compounds_collide_point(desc: *const CompoundDesc, input: *const PointInput, output: *PointOutput) void;
    extern fn jolt_compounds_collect(desc: *const CompoundDesc, input: *const CollectInput, output: *CollectOutput) void;
    extern fn jolt_compounds_collide(desc: *const CompoundDesc, input: *const CollideInput, output: *HitsOutput) void;
    extern fn jolt_compounds_cast(desc: *const CompoundDesc, input: *const CastInput, output: *HitsOutput) void;
    extern fn jolt_compounds_submerged_volume(desc: *const CompoundDesc, transform: *const [16]f32, scale: *const P, plane: *const [4]f32, out_values: *[5]f32) void;
    extern fn jolt_compounds_soft_body(desc: *const CompoundDesc, transform: *const [16]f32, scale: *const P, num_vertices: c_int, positions: [*]const f32, inv_masses: [*]const f32, io_penetrations: [*]f32, io_planes: [*]f32, io_indices: [*]c_int, colliding_shape_index: c_int) void;
    extern fn jolt_compounds_binary_state(desc: *const CompoundDesc, out_bytes: [*]u8, out_restored_bytes: [*]u8, out_children_bytes: [*]u8, out_restored_children_bytes: [*]u8, capacity: u32, out_sizes: *[4]u32) void;
    extern fn jolt_compounds_scale_shape(desc: *const CompoundDesc, scale: *const P, out_error: *[128]u8, out_info: *[2]u32, out_bounds: *[6]f32, out_bytes: [*]u8, capacity: u32, out_size: *u32) c_int;
};

/// Number of random inputs per test (compounds are expensive to build, the tests use a fraction of it)
const iterations = fw.iterations;

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// No sub type / sub shape ID is rejected by the filter
const no_reject: u32 = ~@as(u32, 0);

// Must match the constants in CompoundsReference.cpp
const max_leaves = 4;
const max_nested = 6;
const max_sub_shapes = 200;
const max_mutations = 8;
const max_batch = 4;
const nested_leaf: u32 = 0xffffffff;
const max_hits = 48;
const max_shape_hits = 12;

/// Size of the buffers of the binary state test
const binary_capacity = 64 * 1024;

// ---------------------------------------------------------------------------------------------------------------------
// Compound descriptions, must match LeafDesc / SubDesc / MutationDesc / CompoundDesc in CompoundsReference.cpp

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

const SubDesc = extern struct {
    /// Index in CompoundDesc.leaves, nested_leaf: the nested compound
    leaf: u32 = 0,
    /// 0: none, 1: RotatedTranslatedShape, 2: ScaledShape
    wrap: u32 = 0,
    /// 1: added as settings (the compound creates the child), 0: added as shape
    as_settings: u32 = 0,
    user_data: u32 = 0,
    position: P = .{ 0, 0, 0 },
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
    /// RotatedTranslatedShape position / ScaledShape scale
    wrap_vector: P = .{ 1, 1, 1 },
    /// RotatedTranslatedShape rotation
    wrap_rotation: [4]f32 = .{ 0, 0, 0, 1 },
};

const MutationDesc = extern struct {
    /// 0: AddShape, 1: RemoveShape, 2: ModifyShape, 3: ModifyShape with a shape, 4: ModifyShapes, 5: AdjustCenterOfMass
    op: u32 = 5,
    index: u32 = 0,
    /// ModifyShapes
    count: u32 = 0,
    /// The shape (AddShape, ModifyShape with a shape), position, rotation, user data
    sub: SubDesc = .{},
    /// ModifyShapes
    positions: [max_batch]P = @splat(.{ 0, 0, 0 }),
    rotations: [max_batch][4]f32 = @splat(.{ 0, 0, 0, 1 }),
};

const CompoundDesc = extern struct {
    /// 0: StaticCompoundShape, 1: MutableCompoundShape
    kind: u32 = 0,
    user_data: u32 = 0,
    /// Static: 1 creates with a TempAllocatorImpl. Mutable: 1 clones after the mutations
    create_mode: u32 = 0,
    /// Number of 2 sub shape compounds (of the same kind) wrapped around the compound
    chain_depth: u32 = 0,
    num_leaves: u32 = 0,
    leaves: [max_leaves]LeafDesc = @splat(.{}),
    /// The nested compound (0: StaticCompoundShape, 1: MutableCompoundShape)
    nested_kind: u32 = 0,
    /// 0: there is no nested compound
    num_nested: u32 = 0,
    /// Sub shapes of the nested compound (leaves only)
    nested: [max_nested]SubDesc = @splat(.{}),
    num_sub_shapes: u32 = 0,
    sub_shapes: [max_sub_shapes]SubDesc = @splat(.{}),
    num_mutations: u32 = 0,
    mutations: [max_mutations]MutationDesc = @splat(.{}),
};

/// The stride of ModifyShapes, must match PosRot in CompoundsReference.cpp
const PosRot = extern struct {
    position: Vec3,
    rotation: Quat,

    comptime {
        std.debug.assert(@sizeOf(PosRot) == 32);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Inputs and outputs, must match the extern structs in CompoundsReference.cpp

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

const SubShapeOutput = extern struct {
    position_com: P,
    rotation: [4]f32,
    transform_scale: P,
    user_data: u32,
    sub_type: u32,
    is_valid_scale: c_int,
};

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
    /// A valid sub shape ID: GetSurfaceNormal, GetSupportingFace, GetSubShapeTransformedShape, GetMaterial
    leaf_id: u32,
    /// 0: the shape has no leaf (an empty compound), the functions that need leaf_id are not called
    has_leaf: c_int,
    /// Any sub shape ID: IsSubShapeIDValid (GetLeafShape / GetSubShapeIndexFromID assert on an invalid ID)
    any_id: u32,
    /// GetIntersectingSubShapes
    box: [6]f32,
    /// GetIntersectingSubShapes: orientation, half extents
    oriented_box: [16 + 3]f32,
    max_indices: u32,
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
    /// GetLeafShape(leaf_id)
    leaf_sub_type: u32,
    leaf_user_data: u32,
    leaf_remainder: u32,
    /// GetSubShapeUserData(leaf_id) (low, high)
    sub_shape_user_data: [2]u32,
    material_is_default: c_int,
    /// SaveSubShapeState
    num_sub_shape_state: u32,
    /// GetSubShapeTransformedShape(leaf_id)
    child: TSOutput,
    child_remainder: u32,
    // Compound only
    num_sub_shapes: u32,
    /// GetSubShapeIDBits
    compound_bits: u32,
    /// any_id
    is_sub_shape_id_valid: c_int,
    /// GetSubShapeIndexFromID(leaf_id)
    sub_shape_index: u32,
    sub_shape_index_remainder: u32,
    /// Static: the number of nodes, mutable: the number of bounds blocks (from GetStats)
    num_blocks: u32,
    /// GetIntersectingSubShapes(AABox)
    num_intersecting: u32,
    intersecting: [max_sub_shapes]u32,
    /// GetIntersectingSubShapes(OrientedBox)
    num_intersecting_oriented: u32,
    intersecting_oriented: [max_sub_shapes]u32,
    sub_shapes: [max_sub_shapes]SubShapeOutput,
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
    /// Shape sub type that the filter rejects (~0: none)
    reject_sub_type: u32,
    /// Sub shape ID that the filter rejects
    reject_id: u32,
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
    hits: [max_hits]RayHit,
    filter_calls: u32,
    filter_hash: u32,
};

const PointInput = extern struct {
    point: P,
    creator: [2]u32,
    body_id: u32,
    reject_sub_type: u32,
    reject_id: u32,
    /// 1: AnyHitCollisionCollector
    any_hit: c_int,
};

const PointOutput = extern struct {
    num_hits: u32,
    /// Sub shape IDs in order
    hits: [max_hits]u32,
    /// Of the last hit
    body_id: u32,
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
    reject_id: u32,
    /// 1: AnyHitCollisionCollector
    any_hit: c_int,
    /// TransformShape (may contain a scale)
    transform: [16]f32,
};

const CollectOutput = extern struct {
    num_collected: u32,
    filter_calls: u32,
    filter_hash: u32,
    collected: [max_hits]TSOutput,
    num_transformed: u32,
    transformed: [max_hits]TSOutput,
};

/// The other shape of a collision / cast: a leaf (sphere or box) or a compound
const OtherDesc = extern struct {
    is_compound: u32 = 0,
    leaf: LeafDesc = .{},
};

const CollideInput = extern struct {
    /// 0: the compound is shape 2, 1: shape 1, 2: both (the same compound)
    compound_is_shape1: u32,
    other: OtherDesc,
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
    /// 0: AllHit, 1: AnyHit, 2: ClosestHit
    collector: c_int,
    /// Early out fraction of the collector (when < FLT_MAX)
    early_out: f32,
    body_id: u32,
    reject_sub_type: u32,
    reject_id: u32,
};

const CastInput = extern struct {
    /// 0: the compound is shape 2, 1: shape 1 (the cast shape), 2: both
    compound_is_shape1: u32,
    other: OtherDesc,
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
    /// 0: AllHit, 1: AnyHit, 2: ClosestHit
    collector: c_int,
    /// Early out fraction of the collector (when < 1 + FLT_EPSILON)
    early_out: f32,
    body_id: u32,
    reject_sub_type: u32,
    reject_id: u32,
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
    hits: [max_shape_hits]HitOutput,
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
// Building the shapes (the same steps as CompoundBuilder in CompoundsReference.cpp)

fn userData(value: u32) u64 {
    return @as(u64, value) *% 0x100000001;
}

/// The compound settings base of compound settings created by compoundSettings
fn compoundBase(settings: *ShapeSettings) *CompoundShapeSettings {
    return @fieldParentPtr("base", settings);
}

fn compoundSettings(allocator: Allocator, kind: u32) Allocator.Error!Ref(ShapeSettings) {
    return .init(if (kind == 0) (try StaticCompoundShapeSettings.create(allocator)).asShapeSettings() else (try MutableCompoundShapeSettings.create(allocator)).asShapeSettings());
}

const CompoundBuilder = struct {
    allocator: Allocator,
    desc: *const CompoundDesc,
    leaf_settings: [max_leaves]Ref(ShapeSettings) = @splat(.empty),
    leaf_shapes: [max_leaves]RefConst(Shape) = @splat(.empty),
    nested_settings: Ref(ShapeSettings) = .empty,
    nested_shape: RefConst(Shape) = .empty,

    fn init(allocator: Allocator, desc: *const CompoundDesc) Allocator.Error!CompoundBuilder {
        var self: CompoundBuilder = .{ .allocator = allocator, .desc = desc };
        errdefer self.deinit();
        for (desc.leaves[0..desc.num_leaves], 0..) |leaf, i| {
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
            self.leaf_settings[i] = .init(settings);
            var result = try settings.createShape(allocator);
            defer result.deinit();
            self.leaf_shapes[i] = .init(if (result.isValid()) result.getPtr() else null);
        }

        if (desc.num_nested > 0) {
            self.nested_settings = try compoundSettings(allocator, desc.nested_kind);
            for (desc.nested[0..desc.num_nested]) |*sub|
                try self.addSubShape(compoundBase(self.nested_settings.get().?), sub);
            var result = try self.nested_settings.get().?.createShape(allocator);
            defer result.deinit();
            self.nested_shape = .init(if (result.isValid()) result.getPtr() else null);
        }
        return self;
    }

    fn deinit(self: *CompoundBuilder) void {
        for (&self.leaf_settings) |*s| s.deinit();
        for (&self.leaf_shapes) |*s| s.deinit();
        self.nested_settings.deinit();
        self.nested_shape.deinit();
    }

    /// The compound (the settings of the desc, created, mutated, cloned, wrapped in the chain)
    fn create(self: *CompoundBuilder) Allocator.Error!ShapeResult {
        const allocator = self.allocator;
        const desc = self.desc;
        var settings = try compoundSettings(allocator, desc.kind);
        defer settings.deinit();
        settings.get().?.user_data = userData(desc.user_data);
        for (desc.sub_shapes[0..desc.num_sub_shapes]) |*sub|
            try self.addSubShape(compoundBase(settings.get().?), sub);

        var result: ShapeResult = .empty;
        errdefer result.deinit();
        if (desc.kind == 0 and desc.create_mode == 1) {
            var temp_allocator = try TempAllocatorImpl.init(allocator, 1024 * 1024);
            defer temp_allocator.deinit();
            const static_settings: *StaticCompoundShapeSettings = @fieldParentPtr("base", compoundBase(settings.get().?));
            result = try static_settings.createShapeWithTempAllocator(allocator, temp_allocator.tempAllocator());
        } else result = try settings.get().?.createShape(allocator);
        if (!result.isValid())
            return result;

        if (result.getPtr().?.getSubType() == .mutable_compound) {
            const mutable_shape = result.getPtr().?.castMut(MutableCompoundShape);
            for (desc.mutations[0..desc.num_mutations]) |*mutation|
                try self.mutate(mutable_shape, mutation);
            if (desc.create_mode == 1) {
                var clone = try mutable_shape.clone(allocator);
                defer clone.deinit();
                result.set(clone.clone());
            }
        }

        for (0..desc.chain_depth) |_| {
            var chain = try compoundSettings(allocator, desc.kind);
            defer chain.deinit();
            try compoundBase(chain.get().?).addShapePtr(Vec3.init(-1, 0, 0), Quat.identity(), result.getPtr().?, .{});
            try compoundBase(chain.get().?).addShapePtr(Vec3.init(1, 0, 0), Quat.identity(), self.leaf_shapes[0].get(), .{});
            result.assignMove(try chain.get().?.createShape(allocator));
            if (!result.isValid())
                return result;
        }
        return result;
    }

    /// The shape of a sub shape that is added as a shape (null when the leaf is invalid)
    fn subShape(self: *const CompoundBuilder, sub: *const SubDesc) Allocator.Error!RefConst(Shape) {
        const inner = if (sub.leaf == nested_leaf) self.nested_shape.get() else self.leaf_shapes[sub.leaf].get();
        const inner_shape = inner orelse return .empty;
        return .init(switch (sub.wrap) {
            1 => (try RotatedTranslatedShape.create(self.allocator, vec3(sub.wrap_vector), quat(sub.wrap_rotation), inner_shape)).asShape(),
            2 => (try ScaledShape.create(self.allocator, inner_shape, vec3(sub.wrap_vector))).asShape(),
            else => inner_shape,
        });
    }

    /// The settings of a sub shape that is added as settings
    fn subSettings(self: *const CompoundBuilder, sub: *const SubDesc) Allocator.Error!Ref(ShapeSettings) {
        const inner = if (sub.leaf == nested_leaf) self.nested_settings.get() else self.leaf_settings[sub.leaf].get();
        return .init(switch (sub.wrap) {
            1 => (try RotatedTranslatedShapeSettings.create(self.allocator, vec3(sub.wrap_vector), quat(sub.wrap_rotation), inner)).asShapeSettings(),
            2 => (try ScaledShapeSettings.create(self.allocator, inner, vec3(sub.wrap_vector))).asShapeSettings(),
            else => inner.?,
        });
    }

    fn addSubShape(self: *const CompoundBuilder, settings: *CompoundShapeSettings, sub: *const SubDesc) Allocator.Error!void {
        var shape: RefConst(Shape) = if (sub.as_settings != 0) .empty else try self.subShape(sub);
        defer shape.deinit();
        if (shape.get() != null) {
            try settings.addShapePtr(vec3(sub.position), quat(sub.rotation), shape.get(), .{ .user_data = sub.user_data });
        } else {
            var child = try self.subSettings(sub);
            defer child.deinit();
            try settings.addShape(vec3(sub.position), quat(sub.rotation), child.get(), .{ .user_data = sub.user_data });
        }
    }

    fn mutate(self: *const CompoundBuilder, shape: *MutableCompoundShape, mutation: *const MutationDesc) Allocator.Error!void {
        const num_sub_shapes = shape.base.getNumSubShapes();
        const sub = &mutation.sub;
        switch (mutation.op) {
            0 => {
                var add = try self.subShape(sub);
                defer add.deinit();
                if (add.get()) |s|
                    _ = try shape.addShape(vec3(sub.position), quat(sub.rotation), s, .{ .user_data = sub.user_data, .index = mutation.index });
            },
            1 => if (mutation.index < num_sub_shapes) shape.removeShape(mutation.index),
            2 => if (mutation.index < num_sub_shapes) shape.modifyShape(mutation.index, vec3(sub.position), quat(sub.rotation)),
            3 => {
                var modify = try self.subShape(sub);
                defer modify.deinit();
                if (mutation.index < num_sub_shapes and modify.get() != null)
                    shape.modifyShapeWithShape(mutation.index, vec3(sub.position), quat(sub.rotation), modify.get().?);
            },
            4 => if (@as(u64, mutation.index) + mutation.count <= num_sub_shapes) {
                var pos_rot: [max_batch]PosRot = undefined;
                for (&pos_rot, mutation.positions, mutation.rotations) |*pr, p, r|
                    pr.* = .{ .position = vec3(p), .rotation = quat(r) };
                shape.modifyShapes(mutation.index, mutation.count, .init(&pos_rot[0].position, .{ .stride = @sizeOf(PosRot) }), .init(&pos_rot[0].rotation, .{ .stride = @sizeOf(PosRot) }));
            },
            else => shape.adjustCenterOfMass(),
        }
    }
};

fn createShapeResult(allocator: Allocator, desc: *const CompoundDesc) Allocator.Error!ShapeResult {
    var builder = try CompoundBuilder.init(allocator, desc);
    defer builder.deinit();
    return builder.create();
}

/// The shape of a valid description
fn createShape(allocator: Allocator, desc: *const CompoundDesc) Allocator.Error!RefConst(Shape) {
    var result = try createShapeResult(allocator, desc);
    defer result.deinit();
    return .init(result.getPtr().?);
}

fn createOther(allocator: Allocator, other: *const OtherDesc, compound: *const CompoundDesc) Allocator.Error!RefConst(Shape) {
    if (other.is_compound != 0)
        return createShape(allocator, compound);
    var settings: Ref(ShapeSettings) = .init(if (other.leaf.kind == 0)
        (try SphereShapeSettings.create(allocator, other.leaf.radius, .{})).asShapeSettings()
    else
        (try BoxShapeSettings.create(allocator, vec3(other.leaf.half_extent), .{ .convex_radius = other.leaf.convex_radius })).asShapeSettings());
    defer settings.deinit();
    settings.get().?.user_data = userData(other.leaf.user_data);
    var result = try settings.get().?.createShape(allocator);
    defer result.deinit();
    return .init(result.getPtr().?);
}

// ---------------------------------------------------------------------------------------------------------------------
// The filter, must match CompoundsParityFilter in CompoundsReference.cpp

const FilterLog = struct {
    calls: u32 = 0,
    hash: u32 = 0x811c9dc5,

    fn add(self: *FilterLog, value: u32) void {
        self.hash = (self.hash ^ value) *% 0x01000193;
    }
};

const CompoundsParityFilter = struct {
    pub const overrides = .{ .shouldCollide, .shouldCollidePair };

    base: ShapeFilter = .init(@This()),
    reject_sub_type: u32,
    reject_id: u32,
    /// The calls are logged behind a pointer (Rule M: the filter is const in the queries)
    log: *FilterLog,

    pub fn shouldCollide(self: *const CompoundsParityFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        self.log.calls += 1;
        self.log.add(@intFromEnum(shape2.getSubType()));
        self.log.add(sub_shape_id_of_shape2.getValue());
        return @intFromEnum(shape2.getSubType()) != self.reject_sub_type and sub_shape_id_of_shape2.getValue() != self.reject_id;
    }

    pub fn shouldCollidePair(self: *const CompoundsParityFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        self.log.calls += 1;
        self.log.add(@intFromEnum(shape1.getSubType()));
        self.log.add(sub_shape_id_of_shape1.getValue());
        self.log.add(@intFromEnum(shape2.getSubType()));
        self.log.add(sub_shape_id_of_shape2.getValue());
        return @intFromEnum(shape2.getSubType()) != self.reject_sub_type and sub_shape_id_of_shape1.getValue() != self.reject_id and sub_shape_id_of_shape2.getValue() != self.reject_id;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of the wrappers

/// The number of nodes of a StaticCompoundShape / bounds blocks of a MutableCompoundShape, from GetStats (like the C++
/// side, which cannot read the private arrays)
fn numBlocks(shape: *const Shape) u32 {
    const compound = shape.cast(CompoundShape);
    const size = shape.getStats().size_bytes - compound.getNumSubShapes() * @sizeOf(CompoundShape.SubShape);
    return if (shape.getSubType() == .static_compound)
        @intCast((size - @sizeOf(StaticCompoundShape)) / 64)
    else
        @intCast((size - @sizeOf(MutableCompoundShape)) / (6 * @sizeOf(Vec4)));
}

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
    var visited: Shape.VisitedShapes = .empty;
    defer visited.deinit(allocator);
    o.num_triangles_recursive = (try shape.getStatsRecursive(allocator, &visited)).num_triangles;
    o.sub_shape_id_bits = shape.getSubShapeIDBitsRecursive();
    o.must_be_static = @intFromBool(shape.mustBeStatic());
    o.is_valid_scale = @intFromBool(shape.isValidScale(scale));
    o.scale_valid = arr3(shape.makeScaleValid(scale));
    o.is_valid_any_scale = @intFromBool(shape.isValidScale(any_scale));
    o.any_scale_valid = arr3(shape.makeScaleValid(any_scale));

    const leaf_id = makeSubShapeID(input.leaf_id);
    if (input.has_leaf != 0) {
        o.surface_normal = arr3(shape.getSurfaceNormal(leaf_id, vec3(input.point)));
        var face: Shape.SupportingFace = .empty;
        shape.getSupportingFace(leaf_id, vec3(input.direction), scale, transform, &face);
        storeFace(&face, &o.face_count, &o.face);
    }

    const any_id = makeSubShapeID(input.any_id);
    var sub_shapes: ShapeList = .empty;
    defer {
        for (sub_shapes.items) |*s| s.deinit();
        sub_shapes.deinit(allocator);
    }
    try shape.saveSubShapeState(allocator, &sub_shapes);
    o.num_sub_shape_state = @intCast(sub_shapes.items.len);
    if (input.has_leaf != 0) {
        const leaf = shape.getLeafShape(leaf_id);
        o.leaf_sub_type = if (leaf.shape) |l| @intFromEnum(l.getSubType()) else no_reject;
        o.leaf_user_data = if (leaf.shape) |l| @truncate(l.getUserData()) else 0;
        o.leaf_remainder = leaf.remainder.getValue();
        const user_data = shape.getSubShapeUserData(leaf_id);
        o.sub_shape_user_data = .{ @truncate(user_data), @truncate(user_data >> 32) };
        o.material_is_default = @intFromBool(shape.getMaterial(leaf_id) == PhysicsMaterial.default);

        var child = shape.getSubShapeTransformedShape(leaf_id, vec3(input.position_com), quat(input.rotation), scale);
        defer child.transformed_shape.deinit();
        o.child = storeTS(&child.transformed_shape);
        o.child_remainder = child.remainder.getValue();
    }

    if (shape.getType() == .compound) {
        const compound = shape.cast(CompoundShape);
        o.num_sub_shapes = compound.getNumSubShapes();
        o.compound_bits = compound.getSubShapeIDFromIndex(0, .{}).getNumBitsWritten();
        o.is_sub_shape_id_valid = @intFromBool(compound.isSubShapeIDValid(any_id));
        if (input.has_leaf != 0) {
            const index = compound.getSubShapeIndexFromID(leaf_id);
            o.sub_shape_index = index.index;
            o.sub_shape_index_remainder = index.remainder.getValue();
        }
        o.num_blocks = numBlocks(shape);
        const max_indices = @min(input.max_indices, max_sub_shapes);
        o.num_intersecting = compound.getIntersectingSubShapes(.init(vec3(input.box[0..3].*), vec3(input.box[3..6].*)), o.intersecting[0..max_indices]);
        const oriented_box = OrientedBox.init(mat44(input.oriented_box[0..16].*), vec3(input.oriented_box[16..19].*));
        o.num_intersecting_oriented = compound.getIntersectingSubShapesOrientedBox(oriented_box, o.intersecting_oriented[0..max_indices]);
        const count = @min(o.num_sub_shapes, max_sub_shapes);
        for (compound.getSubShapes()[0..count], o.sub_shapes[0..count]) |*s, *so| {
            so.position_com = arr3(s.getPositionCOM());
            so.rotation = arr4(s.getRotation().getXYZW());
            so.is_valid_scale = @intFromBool(s.isValidScale(scale));
            so.transform_scale = arr3(s.transformScale(scale));
            so.user_data = s.user_data;
            so.sub_type = @intFromEnum(s.shape.get().?.getSubType());
        }
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
    const filter: CompoundsParityFilter = .{ .reject_sub_type = input.reject_sub_type, .reject_id = input.reject_id, .log = &log };
    const store = struct {
        fn f(out: *RayOutput, h: *const RayCastResult) void {
            if (out.num_hits < max_hits)
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

fn zoltCollidePoint(allocator: Allocator, shape: *const Shape, input: *const PointInput) !PointOutput {
    var o = std.mem.zeroes(PointOutput);
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    var log: FilterLog = .{};
    const filter: CompoundsParityFilter = .{ .reject_sub_type = input.reject_sub_type, .reject_id = input.reject_id, .log = &log };
    const store = struct {
        fn f(out: *PointOutput, h: *const CollidePointResult) void {
            if (out.num_hits < max_hits)
                out.hits[out.num_hits] = h.sub_shape_id2.getValue();
            out.num_hits += 1;
            out.body_id = h.body_id.getIndexAndSequenceNumber();
        }
    }.f;
    if (input.any_hit != 0) {
        var collector = AnyHitCollisionCollector(CollidePointCollector).init();
        defer collector.deinit();
        collector.base.setContext(&context);
        shape.collidePoint(vec3(input.point), makeCreator(input.creator), &collector.base, &filter.base);
        if (collector.hadHit()) store(&o, &collector.hit);
    } else {
        var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
        defer collector.deinit();
        collector.base.setContext(&context);
        shape.collidePoint(vec3(input.point), makeCreator(input.creator), &collector.base, &filter.base);
        try collector.checkError();
        for (collector.hits.items) |*h| store(&o, h);
    }
    o.filter_calls = log.calls;
    o.filter_hash = log.hash;
    return o;
}

fn zoltCollect(allocator: Allocator, shape: *const Shape, input: *const CollectInput, o: *CollectOutput) !void {
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    var log: FilterLog = .{};
    const filter: CompoundsParityFilter = .{ .reject_sub_type = input.reject_sub_type, .reject_id = input.reject_id, .log = &log };
    const store = struct {
        fn f(out: *CollectOutput, ts: *const TransformedShape) void {
            if (out.num_collected < max_hits)
                out.collected[out.num_collected] = storeTS(ts);
            out.num_collected += 1;
        }
    }.f;
    const box: AABox = .init(vec3(input.box[0..3].*), vec3(input.box[3..6].*));
    o.num_collected = 0;
    if (input.any_hit != 0) {
        var collector = AnyHitCollisionCollector(TransformedShapeCollector).init();
        defer collector.deinit();
        collector.base.setContext(&context);
        shape.collectTransformedShapes(box, vec3(input.position_com), quat(input.rotation), vec3(input.scale), makeCreator(input.creator), &collector.base, &filter.base);
        if (collector.hadHit()) store(o, &collector.hit);
    } else {
        var collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer collector.deinit();
        collector.base.setContext(&context);
        shape.collectTransformedShapes(box, vec3(input.position_com), quat(input.rotation), vec3(input.scale), makeCreator(input.creator), &collector.base, &filter.base);
        try collector.checkError();
        for (collector.hits.items) |*ts| store(o, ts);
    }
    o.filter_calls = log.calls;
    o.filter_hash = log.hash;

    var transformed = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer transformed.deinit();
    shape.transformShape(mat44(input.transform), &transformed.base);
    try transformed.checkError();
    o.num_transformed = 0;
    for (transformed.hits.items) |*ts| {
        if (o.num_transformed < max_hits)
            o.transformed[o.num_transformed] = storeTS(ts);
        o.num_transformed += 1;
    }
}

/// The shapes of a collide / cast: (shape 1, shape 2)
const Pair = struct {
    shape1: RefConst(Shape),
    shape2: RefConst(Shape),

    fn init(allocator: Allocator, desc: *const CompoundDesc, compound_is_shape1: u32, other_desc: *const OtherDesc) !Pair {
        var compound = try createShape(allocator, desc);
        defer compound.deinit();
        var other = if (compound_is_shape1 == 2) compound.clone() else try createOther(allocator, other_desc, desc);
        defer other.deinit();
        return if (compound_is_shape1 != 0) .{ .shape1 = compound.clone(), .shape2 = other.clone() } else .{ .shape1 = other.clone(), .shape2 = compound.clone() };
    }

    fn deinit(self: *Pair) void {
        self.shape1.deinit();
        self.shape2.deinit();
    }
};

fn storeHit(o: *HitsOutput, r: anytype) void {
    if (o.num_hits < max_shape_hits) {
        const h = &o.hits[o.num_hits];
        if (@TypeOf(r) == *const ShapeCastResult) {
            h.fraction = r.fraction;
            h.back_face = @intFromBool(r.is_back_face_hit);
            storeCollideHit(&r.base, h);
        } else {
            h.fraction = 0.0;
            h.back_face = 0;
            storeCollideHit(r, h);
        }
    }
    o.num_hits += 1;
}

fn zoltCollide(allocator: Allocator, desc: *const CompoundDesc, input: *const CollideInput) !HitsOutput {
    var pair = try Pair.init(allocator, desc, input.compound_is_shape1, &input.other);
    defer pair.deinit();
    var settings: CollideShapeSettings = .{};
    settings.max_separation_distance = input.max_separation_distance;
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    settings.back_face_mode = if (input.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.active_edge_mode = if (input.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.active_edge_movement_direction = vec3(input.active_edge_movement_direction);
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    var log: FilterLog = .{};
    const filter: CompoundsParityFilter = .{ .reject_sub_type = input.reject_sub_type, .reject_id = input.reject_id, .log = &log };
    var o = std.mem.zeroes(HitsOutput);
    const run = struct {
        fn f(c: *CollideShapeCollector, in: *const CollideInput, p: *const Pair, s: *const CollideShapeSettings, ctx: *const TransformedShape, flt: *const ShapeFilter) void {
            c.setContext(ctx);
            if (in.early_out < c.getEarlyOutFraction()) c.updateEarlyOutFraction(in.early_out);
            CollisionDispatch.collideShapeVsShape(p.shape1.get().?, p.shape2.get().?, vec3(in.scale1), vec3(in.scale2), mat44(in.transform1), mat44(in.transform2), makeCreator(in.creator1), makeCreator(in.creator2), s, c, flt);
        }
    }.f;
    switch (input.collector) {
        0 => {
            var collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
            defer collector.deinit();
            run(&collector.base, input, &pair, &settings, &context, &filter.base);
            try collector.checkError();
            for (collector.hits.items) |*r| storeHit(&o, @as(*const CollideShapeResult, r));
        },
        1 => {
            var collector = AnyHitCollisionCollector(CollideShapeCollector).init();
            defer collector.deinit();
            run(&collector.base, input, &pair, &settings, &context, &filter.base);
            if (collector.hadHit()) storeHit(&o, @as(*const CollideShapeResult, &collector.hit));
        },
        else => {
            var collector = ClosestHitCollisionCollector(CollideShapeCollector).init();
            defer collector.deinit();
            run(&collector.base, input, &pair, &settings, &context, &filter.base);
            if (collector.hadHit()) storeHit(&o, @as(*const CollideShapeResult, &collector.hit));
        },
    }
    o.filter_calls = log.calls;
    o.filter_hash = log.hash;
    return o;
}

fn zoltCast(allocator: Allocator, desc: *const CompoundDesc, input: *const CastInput) !HitsOutput {
    var pair = try Pair.init(allocator, desc, input.compound_is_shape1, &input.other);
    defer pair.deinit();
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
    const shape_cast = ShapeCast.init(pair.shape1.get().?, vec3(input.scale1), mat44(input.start), vec3(input.direction));
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    var log: FilterLog = .{};
    const filter: CompoundsParityFilter = .{ .reject_sub_type = input.reject_sub_type, .reject_id = input.reject_id, .log = &log };
    var o = std.mem.zeroes(HitsOutput);
    const run = struct {
        fn f(c: *CastShapeCollector, in: *const CastInput, p: *const Pair, sc: *const ShapeCast, s: *const ShapeCastSettings, ctx: *const TransformedShape, flt: *const ShapeFilter) void {
            c.setContext(ctx);
            if (in.early_out < c.getEarlyOutFraction()) c.updateEarlyOutFraction(in.early_out);
            CollisionDispatch.castShapeVsShapeWorldSpace(sc, s, p.shape2.get().?, vec3(in.scale2), flt, mat44(in.transform2), makeCreator(in.creator1), makeCreator(in.creator2), c);
        }
    }.f;
    switch (input.collector) {
        0 => {
            var collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
            defer collector.deinit();
            run(&collector.base, input, &pair, &shape_cast, &settings, &context, &filter.base);
            try collector.checkError();
            for (collector.hits.items) |*r| storeHit(&o, @as(*const ShapeCastResult, r));
        },
        1 => {
            var collector = AnyHitCollisionCollector(CastShapeCollector).init();
            defer collector.deinit();
            run(&collector.base, input, &pair, &shape_cast, &settings, &context, &filter.base);
            if (collector.hadHit()) storeHit(&o, @as(*const ShapeCastResult, &collector.hit));
        },
        else => {
            var collector = ClosestHitCollisionCollector(CastShapeCollector).init();
            defer collector.deinit();
            run(&collector.base, input, &pair, &shape_cast, &settings, &context, &filter.base);
            if (collector.hadHit()) storeHit(&o, @as(*const ShapeCastResult, &collector.hit));
        },
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

fn zoltBinaryState(allocator: Allocator, shape: *const Shape, o: *BinaryOutput) !void {
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
}

// ---------------------------------------------------------------------------------------------------------------------
// Input generation

/// Values that tend to expose differences, mixed into the random inputs
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 1.0e-6, -1.0e-6, 3.0, -3.0 };

/// The sub types that the filters reject now and then
const reject_candidates = [_]u32{ @intFromEnum(zolt.ShapeSubType.sphere), @intFromEnum(zolt.ShapeSubType.box), @intFromEnum(zolt.ShapeSubType.static_compound), @intFromEnum(zolt.ShapeSubType.mutable_compound), @intFromEnum(zolt.ShapeSubType.rotated_translated), @intFromEnum(zolt.ShapeSubType.scaled) };

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

    /// A unit quaternion, sometimes the identity, -identity, almost the identity or a rotation of a multiple of 90
    /// degrees around an axis
    fn rotation(self: *Gen) Quat {
        switch (self.index(8)) {
            0, 1 => return Quat.identity(),
            2 => return if (self.oneIn(2)) Quat.identity().negate() else Quat.rotation(Vec3.axisX(), self.plain(-1.0e-6, 1.0e-6)),
            3 => {
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
        const bits: u32 = @intCast(self.index(5));
        return .{ if (bits == 0) 0 else self.next() & ((@as(u32, 1) << @intCast(bits)) - 1), bits };
    }

    fn reject(self: *Gen) u32 {
        return if (self.oneIn(5)) reject_candidates[self.index(reject_candidates.len)] else no_reject;
    }

    /// A valid leaf: a sphere or a box (sometimes flat, with or without convex radius)
    fn leaf(self: *Gen) LeafDesc {
        const density = if (self.oneIn(4)) 1000.0 else self.plain(1, 3000);
        const user_data = self.next();
        if (self.oneIn(2))
            return .{ .kind = 0, .radius = if (self.oneIn(5)) 1.0 else self.plain(0.05, 1.5), .density = density, .user_data = user_data };
        var half_extent = self.plainVec(0.05, 1.5);
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

    /// A position of a sub shape: random in a box of `extent`, on a grid, at the origin
    fn position(self: *Gen, extent: f32) P {
        return switch (self.index(10)) {
            0 => .{ 0, 0, 0 },
            1, 2 => .{ self.grid(3), self.grid(3), self.grid(3) },
            else => self.vec(-extent, extent),
        };
    }

    /// A sub shape: a leaf (or the nested compound when `nested`), wrapped or not, as a shape or as settings
    fn subDesc(self: *Gen, num_leaves: u32, nested: bool, extent: f32) SubDesc {
        var sub: SubDesc = .{
            .leaf = if (nested and self.oneIn(4)) nested_leaf else @intCast(self.index(num_leaves)),
            .wrap = if (self.oneIn(3)) @intCast(1 + self.index(2)) else 0,
            .as_settings = @intFromBool(self.oneIn(3)),
            .user_data = self.next(),
            .position = self.position(extent),
            .rotation = arr4(self.rotation().getXYZW()),
        };
        switch (sub.wrap) {
            1 => {
                sub.wrap_vector = self.vec(-1, 1);
                sub.wrap_rotation = arr4(self.rotation().getXYZW());
            },
            2 => {
                // A uniform scale (valid for every child, also when the sub shape is rotated)
                const s = if (self.oneIn(3)) self.grid(2) else self.plain(0.3, 2.0);
                const m = if (s == 0.0) 1.0 else s;
                sub.wrap_vector = .{ m, m, m };
            },
            else => {},
        }
        return sub;
    }

    /// A valid compound description: 1 to 200 sub shapes, sometimes a nested compound, mutations, a clone or the temp
    /// allocator
    fn compoundDesc(self: *Gen, kind: u32) CompoundDesc {
        var desc: CompoundDesc = .{ .kind = kind, .user_data = self.next(), .create_mode = @intFromBool(self.oneIn(3)) };
        desc.num_leaves = @intCast(1 + self.index(max_leaves));
        for (desc.leaves[0..desc.num_leaves]) |*l| l.* = self.leaf();
        if (self.oneIn(4)) {
            desc.nested_kind = @intCast(self.index(2));
            desc.num_nested = @intCast(1 + self.index(max_nested));
            for (desc.nested[0..desc.num_nested]) |*n| n.* = self.subDesc(desc.num_leaves, false, 2.0);
        }
        desc.num_sub_shapes = @intCast(switch (self.index(10)) {
            0...4 => 1 + self.index(8),
            5...7 => 9 + self.index(32),
            else => 41 + self.index(max_sub_shapes - 40 - max_mutations), // Room for the shapes that the mutations add
        });
        const extent: f32 = if (desc.num_sub_shapes < 10) 3.0 else if (desc.num_sub_shapes < 50) 8.0 else 15.0;
        for (desc.sub_shapes[0..desc.num_sub_shapes]) |*s| s.* = self.subDesc(desc.num_leaves, desc.num_nested > 0, extent);
        if (self.oneIn(15)) {
            // All sub shapes at the same position (the tree cannot partition them)
            for (desc.sub_shapes[0..desc.num_sub_shapes]) |*s| s.position = desc.sub_shapes[0].position;
        }
        if (kind == 1 and self.oneIn(2)) {
            desc.num_mutations = @intCast(1 + self.index(max_mutations));
            for (desc.mutations[0..desc.num_mutations]) |*m| {
                m.op = @intCast(self.index(6));
                m.index = switch (self.index(4)) {
                    0 => 0xffffffff,
                    1 => desc.num_sub_shapes -| 1,
                    else => @intCast(self.index(desc.num_sub_shapes + 2)),
                };
                m.count = @intCast(self.index(max_batch + 1));
                m.sub = self.subDesc(desc.num_leaves, desc.num_nested > 0, extent);
                for (&m.positions, &m.rotations) |*p, *r| {
                    p.* = self.position(extent);
                    r.* = arr4(self.rotation().getXYZW());
                }
            }
        }
        return desc;
    }

    /// A scale that is valid for `shape`: uniform (also negative), non-uniform, mirrored (made valid like Jolt's
    /// MakeScaleValid would)
    fn validScale(self: *Gen, shape: *const Shape) P {
        const candidate: P = switch (self.index(5)) {
            0 => .{ 1, 1, 1 },
            1 => blk: {
                const m = if (self.oneIn(2)) self.plain(0.3, 2.0) else 1.0 + self.grid(1) * 0.5;
                break :blk if (self.oneIn(3)) .{ -m, -m, -m } else .{ m, m, m };
            },
            2 => .{ if (self.oneIn(2)) 1 else -1, if (self.oneIn(2)) 1 else -1, if (self.oneIn(2)) 1 else -1 },
            else => self.plainVec(0.3, 2.0),
        };
        return makeValid(shape, candidate);
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

    /// A sub shape ID that leads to a leaf of `shape` (walking the compounds and decorators)
    fn leafID(self: *Gen, shape: *const Shape) ?u32 {
        var id_creator: SubShapeIDCreator = .{};
        var s = shape;
        while (true) {
            switch (s.getType()) {
                .compound => {
                    const c = s.cast(CompoundShape);
                    if (c.getNumSubShapes() == 0) return null;
                    const i: u32 = @intCast(self.index(c.getNumSubShapes()));
                    id_creator = c.getSubShapeIDFromIndex(i, id_creator);
                    s = c.getSubShape(i).shape.get().?;
                },
                .decorated => s = s.cast(DecoratedShape).getInnerShape().?,
                else => return id_creator.getID().getValue(),
            }
        }
    }

    /// Any sub shape ID (for IsSubShapeIDValid): valid, a random value, empty
    fn anyID(self: *Gen, shape: *const Shape) u32 {
        return switch (self.index(3)) {
            0 => self.leafID(shape) orelse SubShapeID.empty.getValue(),
            1 => SubShapeID.empty.getValue(),
            else => self.next(),
        };
    }

    /// A sub shape ID for the filter to reject: one of a leaf, or none
    fn rejectID(self: *Gen, shape: *const Shape, query_creator: [2]u32) u32 {
        if (!self.oneIn(4)) return no_reject;
        // The filter sees the IDs of the sub shapes below the creator of the query
        var id = makeCreator(query_creator);
        if (shape.getType() == .compound) {
            const c = shape.cast(CompoundShape);
            if (c.getNumSubShapes() > 0)
                id = c.getSubShapeIDFromIndex(@intCast(self.index(c.getNumSubShapes())), id);
        }
        return id.getID().getValue();
    }
};

/// True if the queries on the compound of `desc` can use an AnyHitCollisionCollector: MutableCompoundShape::WalkSubShapes
/// only stops the walk through the current block of 4 sub shapes when the collector forces an early out, the next blocks
/// are still tested and can add another hit, which AnyHitCollisionCollector asserts against (Jolt's debug build does the
/// same, its release build keeps the last hit). So no MutableCompoundShape with more than 4 sub shapes may be involved.
fn anyHitAllowed(desc: *const CompoundDesc) bool {
    return desc.kind == 0 and (desc.num_nested == 0 or desc.nested_kind == 0 or desc.num_nested <= 4);
}

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
fn extentOf(shape: *const Shape, scale: P) f32 {
    const bounds = shape.getLocalBounds().scaled(vec3(scale));
    return @max(@max(bounds.min.abs().reduceMax(), bounds.max.abs().reduceMax()), 0.05);
}

/// Hand-picked compounds: 2 unit boxes without convex radius at (-1, 0, 0) and (1, 0, 0) that touch each other
/// (`which` = 0) or 4 unit spheres in a row that touch each other (`which` = 1)
fn handPickedDesc(kind: u32, which: usize) CompoundDesc {
    var desc: CompoundDesc = .{ .kind = kind, .num_leaves = 1 };
    if (which == 0) {
        desc.leaves[0] = .{ .kind = 1, .half_extent = .{ 1, 1, 1 }, .convex_radius = 0 };
        desc.num_sub_shapes = 2;
        desc.sub_shapes[0] = .{ .position = .{ -1, 0, 0 } };
        desc.sub_shapes[1] = .{ .position = .{ 1, 0, 0 } };
    } else {
        desc.leaves[0] = .{ .kind = 0, .radius = 1 };
        desc.num_sub_shapes = 4;
        for (desc.sub_shapes[0..4], 0..) |*sub, i| sub.* = .{ .position = .{ 2 * @as(f32, @floatFromInt(i)) - 3, 0, 0 } };
    }
    return desc;
}

/// Hand-picked rays for the hand-picked compounds (the center of mass is at the origin): along a face, along the face
/// where the boxes touch, along an edge, starting inside, through the point where 2 spheres touch, parallel to the row
const hand_picked_rays = [_]struct { origin: P, direction: P }{
    .{ .origin = .{ -5, 1, 0 }, .direction = .{ 10, 0, 0 } },
    .{ .origin = .{ 0, -5, 0 }, .direction = .{ 0, 10, 0 } },
    .{ .origin = .{ -5, 1, 1 }, .direction = .{ 10, 0, 0 } },
    .{ .origin = .{ -1, 0, 0 }, .direction = .{ 5, 0.5, 0 } },
    .{ .origin = .{ -1, 5, 0 }, .direction = .{ 0, -10, 0 } },
    .{ .origin = .{ -10, 0, 0 }, .direction = .{ 20, 0, 0 } },
};

/// Hand-picked shapes that touch the hand-picked compounds: a unit box at (3, 0, 0) (touches the boxes / the last sphere
/// at x = 2 / 4), a unit sphere at (0, 2, 0) (touches the edge where the boxes meet / 2 spheres)
const hand_picked_others = [_]struct { leaf: LeafDesc, translation: P }{
    .{ .leaf = .{ .kind = 1, .half_extent = .{ 1, 1, 1 }, .convex_radius = 0 }, .translation = .{ 3, 0, 0 } },
    .{ .leaf = .{ .kind = 0, .radius = 1 }, .translation = .{ 0, 2, 0 } },
};

/// Checks that a test exercised both paths (e.g. hits and misses), prints the count when not
fn expectBetween(name: []const u8, value: usize, min: usize, max: usize) !void {
    if (value <= min or value >= max) {
        std.debug.print("{s}: {d} is not in ({d}, {d})\n", .{ name, value, min, max });
        return error.TestUnexpectedResult;
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

test "Compounds parity: settings, Jolt's error texts, the single sub shape shortcuts and too deep hierarchies" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "settings" };
    var num_valid: usize = 0;
    var num_single: usize = 0;
    for (0..iterations / 100) |i| {
        var desc = gen.compoundDesc(@intCast(i % 2));
        switch (gen.index(8)) {
            // No sub shapes
            0 => desc.num_sub_shapes = 0,
            // One sub shape, sometimes at the origin without rotation
            1, 2 => {
                desc.num_sub_shapes = 1;
                if (gen.oneIn(2)) {
                    desc.sub_shapes[0].position = .{ 0, 0, 0 };
                    desc.sub_shapes[0].rotation = .{ 0, 0, 0, 1 };
                }
            },
            // Invalid leaves (child errors), zero scales of the wrapped sub shapes
            3 => {
                desc.leaves[gen.index(desc.num_leaves)].radius = gen.float(-1, 0);
                desc.leaves[gen.index(desc.num_leaves)].kind = 0;
                for (desc.sub_shapes[0..desc.num_sub_shapes]) |*s| s.as_settings = 1; // A shape cannot be added for an invalid leaf
                for (desc.nested[0..desc.num_nested]) |*s| s.as_settings = 1;
                for (desc.mutations[0..desc.num_mutations]) |*m| m.op = 5;
                desc.chain_depth = 0;
            },
            4 => {
                const s = &desc.sub_shapes[gen.index(desc.num_sub_shapes)];
                s.wrap = 2;
                s.as_settings = 1;
                s.wrap_vector[gen.index(3)] = if (gen.oneIn(2)) 0.0 else gen.plain(-1.0e-6, 1.0e-6);
            },
            // A chain that exceeds the sub shape ID bits (or not)
            5, 6 => {
                desc.chain_depth = @intCast(gen.index(40));
                desc.leaves[0] = gen.leaf();
            },
            else => {},
        }
        var jolt_error: [128]u8 = undefined;
        var jolt_info: [4]u32 = undefined;
        const jolt_valid = jolt.jolt_compounds_create(&desc, &jolt_error, &jolt_info);

        var zolt_error: [128]u8 = @splat(0);
        var zolt_info: [4]u32 = @splat(0);
        var result = try createShapeResult(allocator, &desc);
        defer result.deinit();
        const zolt_valid: c_int = @intFromBool(result.isValid());
        if (result.isValid()) {
            const shape = result.getPtr().?;
            zolt_info = .{ @intFromEnum(shape.getSubType()), @truncate(shape.getUserData()), if (shape.getType() == .compound) shape.cast(CompoundShape).getNumSubShapes() else 0, shape.getSubShapeIDBitsRecursive() };
            num_valid += 1;
            if (shape.getType() != .compound) num_single += 1;
        } else {
            const text = result.getError();
            @memcpy(zolt_error[0..@min(text.len, 127)], text[0..@min(text.len, 127)]);
        }
        checker.check(.{desc.kind}, .{ zolt_valid, zolt_error, zolt_info }, .{ jolt_valid, jolt_error, jolt_info });
    }
    try checker.finish();
    try expectBetween("valid settings", num_valid, iterations / 400, iterations / 100 - iterations / 1000);
    try expectBetween("single sub shapes", num_single, 0, iterations / 200);
}

test "Compounds parity: bounds, mass properties, sub shapes, sub shape IDs, the tree / blocks, intersecting sub shapes" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "properties" };
    const jolt_output = try allocator.create(PropertiesOutput);
    defer allocator.destroy(jolt_output);
    const zolt_output = try allocator.create(PropertiesOutput);
    defer allocator.destroy(zolt_output);
    for (0..iterations / 50) |i| {
        const desc = gen.compoundDesc(@intCast(i % 2));
        var shape_ref = try createShape(allocator, &desc);
        defer shape_ref.deinit();
        const shape = shape_ref.get().?;
        const leaf_id = gen.leafID(shape);
        const center = shape.getLocalBounds().getCenter();
        const extent = extentOf(shape, .{ 1, 1, 1 });
        const box_center = center.add(vec3(gen.plainVec(-extent, extent)));
        const box_extent = gen.plain(0.1, extent);
        const num_sub_shapes: u32 = if (shape.getType() == .compound) shape.cast(CompoundShape).getNumSubShapes() else 0;
        const input: PropertiesInput = .{
            .scale = gen.validScale(shape),
            .any_scale = gen.anyScale(),
            .transform = arr16(gen.transform(10)),
            .translation = .{ gen.plain(-1.0e5, 1.0e5), gen.plain(-10, 10), gen.plain(-10, 10) },
            .point = gen.vec(-4, 4),
            .direction = gen.direction(3),
            .position_com = gen.vec(-5, 5),
            .rotation = arr4(gen.rotation().getXYZW()),
            .leaf_id = leaf_id orelse 0,
            .has_leaf = @intFromBool(leaf_id != null),
            .any_id = gen.anyID(shape),
            .box = boxArr(if (gen.oneIn(10)) AABox.biggest() else .fromCenterAndRadius(box_center, box_extent)),
            .oriented_box = arr16(Mat44.rotationTranslation(gen.rotation(), box_center)) ++ gen.plainVec(0.1, extent),
            // A buffer that cannot hold all results stops the walk (MutableCompoundShape overruns the buffer when the
            // results span more than one block, like Jolt, so it always gets the full buffer)
            .max_indices = if (desc.kind == 0 and gen.oneIn(3)) @intCast(gen.index(num_sub_shapes + 1)) else num_sub_shapes,
        };
        jolt_output.* = std.mem.zeroes(PropertiesOutput);
        jolt.jolt_compounds_properties(&desc, &input, jolt_output);
        zolt_output.* = try zoltProperties(allocator, shape, &input);
        checker.check(.{ desc.kind, desc.num_sub_shapes, input }, zolt_output.*, jolt_output.*);
    }
    try checker.finish();
}

test "Compounds parity: CastRay (single hit and collectors) and CollidePoint" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var rays: Checker = .{ .name = "cast ray" };
    var points: Checker = .{ .name = "collide point" };
    var num_ray_hits: usize = 0;
    var num_point_hits: usize = 0;
    for (0..iterations / 40) |i| {
        const hand_picked = i < 4 * hand_picked_rays.len;
        const desc = if (hand_picked) handPickedDesc(@intCast(i % 2), (i / 2) % 2) else gen.compoundDesc(@intCast(i % 2));
        var shape_ref = try createShape(allocator, &desc);
        defer shape_ref.deinit();
        const shape = shape_ref.get().?;
        const extent = extentOf(shape, .{ 1, 1, 1 });
        const center = arr3(shape.getLocalBounds().getCenter());

        // Rays from outside through the compound, from inside, along faces, degenerate directions
        var input: RayInput = .{
            .origin = gen.vec(-2 * extent - 1, 2 * extent + 1),
            .direction = gen.direction(4 * extent + 2),
            .creator = gen.creator(),
            .fraction = if (gen.oneIn(2)) 1.0 + math.flt_epsilon else gen.plain(0, 1),
            .back_face_mode = @intFromBool(gen.oneIn(2)),
            .treat_convex_as_solid = @intFromBool(!gen.oneIn(3)),
            .collector = if (anyHitAllowed(&desc)) @intCast(gen.index(3)) else if (gen.oneIn(2)) 0 else 2,
            .early_out = if (gen.oneIn(2)) 2.0 else gen.plain(0, 1),
            .body_id = gen.next() & 0x7fffff,
            .reject_sub_type = gen.reject(),
            .reject_id = 0,
        };
        input.reject_id = gen.rejectID(shape, input.creator);
        for (&input.origin, center) |*o, c| o.* += c;
        if (gen.oneIn(2)) {
            // Aim at a sub shape (or the center)
            var target = vec3(center).add(vec3(gen.plainVec(-0.5 * extent, 0.5 * extent)));
            if (shape.getType() == .compound and shape.cast(CompoundShape).getNumSubShapes() > 0) {
                const c = shape.cast(CompoundShape);
                target = c.getSubShape(@intCast(gen.index(c.getNumSubShapes()))).getPositionCOM();
                if (gen.oneIn(3)) input.origin = arr3(target.add(vec3(gen.plainVec(-0.1, 0.1)))); // Start inside
            }
            input.direction = arr3(target.sub(vec3(input.origin)).mulScalar(gen.plain(0.5, 3)));
        }
        if (hand_picked) {
            input.origin = hand_picked_rays[i / 4].origin;
            input.direction = hand_picked_rays[i / 4].direction;
            input.fraction = 1.0 + math.flt_epsilon;
            input.early_out = 2.0;
            input.reject_sub_type = no_reject;
            input.reject_id = no_reject;
        }
        var jolt_output = std.mem.zeroes(RayOutput);
        jolt.jolt_compounds_cast_ray(&desc, &input, &jolt_output);
        const zolt_output = try zoltCastRay(allocator, shape, &input);
        num_ray_hits += zolt_output.num_hits;
        rays.check(.{ desc.kind, desc.num_sub_shapes, input }, zolt_output, jolt_output);

        // Points inside and outside, often at a sub shape
        var point_input: PointInput = .{
            .point = gen.vec(-1.5 * extent, 1.5 * extent),
            .creator = input.creator,
            .body_id = input.body_id,
            .reject_sub_type = gen.reject(),
            .reject_id = gen.rejectID(shape, input.creator),
            .any_hit = @intFromBool(anyHitAllowed(&desc) and gen.oneIn(4)),
        };
        for (&point_input.point, center) |*p, c| p.* += c;
        if (gen.oneIn(2) and shape.getType() == .compound and shape.cast(CompoundShape).getNumSubShapes() > 0) {
            const c = shape.cast(CompoundShape);
            point_input.point = arr3(c.getSubShape(@intCast(gen.index(c.getNumSubShapes()))).getPositionCOM().add(vec3(gen.plainVec(-0.5, 0.5))));
        }
        var jolt_point = std.mem.zeroes(PointOutput);
        jolt.jolt_compounds_collide_point(&desc, &point_input, &jolt_point);
        const zolt_point = try zoltCollidePoint(allocator, shape, &point_input);
        num_point_hits += zolt_point.num_hits;
        points.check(.{ desc.kind, desc.num_sub_shapes, point_input }, zolt_point, jolt_point);
    }
    try finishAll(&.{ &rays, &points });
    try expectBetween("ray hits", num_ray_hits, iterations / 200, 100 * iterations);
    try expectBetween("point hits", num_point_hits, iterations / 400, 100 * iterations);
}

test "Compounds parity: CollectTransformedShapes and TransformShape" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "collect" };
    const jolt_output = try allocator.create(CollectOutput);
    defer allocator.destroy(jolt_output);
    const zolt_output = try allocator.create(CollectOutput);
    defer allocator.destroy(zolt_output);
    for (0..iterations / 50) |i| {
        const desc = gen.compoundDesc(@intCast(i % 2));
        var shape_ref = try createShape(allocator, &desc);
        defer shape_ref.deinit();
        const shape = shape_ref.get().?;
        const extent = extentOf(shape, .{ 1, 1, 1 });
        var transform = gen.transform(5);
        if (gen.oneIn(2)) transform = transform.mul(Mat44.scaleVec3(vec3(gen.plainVec(0.3, 2.0))));
        const position_com = gen.vec(-5, 5);
        const creator = gen.creator();
        const input: CollectInput = .{
            .box = if (gen.oneIn(3)) boxArr(AABox.biggest()) else boxArr(.fromCenterAndRadius(vec3(position_com).add(vec3(gen.plainVec(-extent, extent))), gen.plain(0.1, 2.0 * extent))),
            .position_com = position_com,
            .rotation = arr4(gen.rotation().getXYZW()),
            .scale = gen.validScale(shape),
            .creator = creator,
            .body_id = gen.next() & 0x7fffff,
            .reject_sub_type = gen.reject(),
            .reject_id = gen.rejectID(shape, creator),
            .any_hit = @intFromBool(anyHitAllowed(&desc) and gen.oneIn(5)),
            .transform = arr16(transform),
        };
        jolt_output.* = std.mem.zeroes(CollectOutput);
        jolt.jolt_compounds_collect(&desc, &input, jolt_output);
        zolt_output.* = std.mem.zeroes(CollectOutput);
        try zoltCollect(allocator, shape, &input, zolt_output);
        checker.check(.{ desc.kind, desc.num_sub_shapes, input }, zolt_output.*, jolt_output.*);
    }
    try checker.finish();
}

/// The other shape of a collision or cast: a sphere, a box or a compound
fn otherDesc(gen: *Gen) OtherDesc {
    return .{ .is_compound = @intFromBool(gen.oneIn(5)), .leaf = gen.leaf() };
}

/// The collector of a collide / cast: mostly all hits (AnyHit only when anyHitAllowed)
fn collectorKind(gen: *Gen, desc: *const CompoundDesc) c_int {
    return switch (gen.index(6)) {
        0 => if (anyHitAllowed(desc)) 1 else 2,
        1 => 2,
        else => 0,
    };
}

test "Compounds parity: collide with a compound on either side through CollisionDispatch" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "collide" };
    var num_hits: usize = 0;
    const jolt_output = try allocator.create(HitsOutput);
    defer allocator.destroy(jolt_output);
    for (0..iterations / 50) |i| {
        // The first inputs: hand-picked touching shapes (both kinds, both compounds, both other shapes, both sides)
        const hand_picked = i < 16;
        var desc = if (hand_picked) handPickedDesc(@intCast(i % 2), (i / 2) % 2) else gen.compoundDesc(@intCast(i % 2));
        if (desc.num_sub_shapes > 60 and gen.oneIn(2)) desc.num_sub_shapes = 60; // Keep compound vs compound affordable
        const compound_is_shape1: u32 = if (hand_picked) @intCast((i / 8) % 2) else @intCast(gen.index(3));
        const other: OtherDesc = if (hand_picked) .{ .leaf = hand_picked_others[(i / 4) % 2].leaf } else otherDesc(&gen);
        var pair = try Pair.init(allocator, &desc, compound_is_shape1, &other);
        defer pair.deinit();
        const scale1 = gen.validScale(pair.shape1.get().?);
        const scale2 = gen.validScale(pair.shape2.get().?);
        const transform1 = gen.transform(5);
        // Place shape 2 near shape 1 so that many pairs collide
        const reach = extentOf(pair.shape1.get().?, scale1) + extentOf(pair.shape2.get().?, scale2);
        var relative = gen.transform(0.6 * reach);
        if (gen.oneIn(10)) relative.setTranslation(Vec3.zero()); // Same center
        const creator1 = gen.creator();
        const creator2 = gen.creator();
        var input: CollideInput = .{
            .compound_is_shape1 = compound_is_shape1,
            .other = other,
            .scale1 = scale1,
            .scale2 = scale2,
            .transform1 = arr16(transform1),
            .transform2 = arr16(transform1.mul(relative)),
            .creator1 = creator1,
            .creator2 = creator2,
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
            .collector = collectorKind(&gen, &desc),
            .early_out = if (gen.oneIn(4)) gen.plain(-2, 2) else math.flt_max,
            .body_id = gen.next() & 0x7fffff,
            .reject_sub_type = gen.reject(),
            .reject_id = if (gen.oneIn(2)) gen.rejectID(pair.shape1.get().?, creator1) else gen.rejectID(pair.shape2.get().?, creator2),
        };
        if (hand_picked) {
            const t = vec3(hand_picked_others[(i / 4) % 2].translation);
            input.scale1 = .{ 1, 1, 1 };
            input.scale2 = .{ 1, 1, 1 };
            input.transform1 = arr16(if (compound_is_shape1 != 0) Mat44.identity() else Mat44.translation(t));
            input.transform2 = arr16(if (compound_is_shape1 != 0) Mat44.translation(t) else Mat44.identity());
            input.max_separation_distance = 0.0;
            input.collector = 0;
            input.early_out = math.flt_max;
            input.reject_sub_type = no_reject;
            input.reject_id = no_reject;
        }
        jolt_output.* = std.mem.zeroes(HitsOutput);
        jolt.jolt_compounds_collide(&desc, &input, jolt_output);
        const zolt_output = try zoltCollide(allocator, &desc, &input);
        num_hits += zolt_output.num_hits;
        checker.check(.{ desc.kind, desc.num_sub_shapes, input }, zolt_output, jolt_output.*);
    }
    try checker.finish();
    try expectBetween("collide hits", num_hits, iterations / 200, 1000 * iterations);
}

test "Compounds parity: cast with a compound on either side through CollisionDispatch (world space)" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "cast" };
    var num_hits: usize = 0;
    const jolt_output = try allocator.create(HitsOutput);
    defer allocator.destroy(jolt_output);
    for (0..iterations / 50) |i| {
        var desc = gen.compoundDesc(@intCast(i % 2));
        if (desc.num_sub_shapes > 60 and gen.oneIn(2)) desc.num_sub_shapes = 60; // Keep compound vs compound affordable
        const compound_is_shape1: u32 = @intCast(gen.index(3));
        const other = otherDesc(&gen);
        var pair = try Pair.init(allocator, &desc, compound_is_shape1, &other);
        defer pair.deinit();
        const scale1 = gen.validScale(pair.shape1.get().?);
        const scale2 = gen.validScale(pair.shape2.get().?);
        const transform2 = gen.transform(5);
        const reach = extentOf(pair.shape1.get().?, scale1) + extentOf(pair.shape2.get().?, scale2);
        // Start near shape 2 (sometimes overlapping), move towards it, past it or away from it
        var start = transform2.mul(gen.transform(1.5 * reach));
        if (gen.oneIn(8)) start.setTranslation(transform2.getTranslation().add(vec3(gen.plainVec(-0.2, 0.2))));
        const to_target = transform2.getTranslation().sub(start.getTranslation());
        const direction = switch (gen.index(4)) {
            0 => to_target.mulScalar(gen.plain(0.5, 3)),
            1 => to_target.mulScalar(gen.plain(-1, 0.2)),
            2 => vec3(gen.direction(3 * reach)),
            else => to_target.add(vec3(gen.plainVec(-reach, reach))).mulScalar(gen.plain(0.5, 2)),
        };
        const creator1 = gen.creator();
        const creator2 = gen.creator();
        const input: CastInput = .{
            .compound_is_shape1 = compound_is_shape1,
            .other = other,
            .scale1 = scale1,
            .start = arr16(start),
            .direction = arr3(direction),
            .scale2 = scale2,
            .transform2 = arr16(transform2),
            .creator1 = creator1,
            .creator2 = creator2,
            .collision_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_collision_tolerance,
            .penetration_tolerance = if (gen.oneIn(4)) gen.plain(1.0e-5, 1.0e-3) else zolt.physics_settings.default_penetration_tolerance,
            .extra_convex_radius = if (gen.oneIn(4)) gen.plain(0, 0.3) else 0.0,
            .back_face_mode_triangles = @intFromBool(gen.oneIn(2)),
            .back_face_mode_convex = @intFromBool(gen.oneIn(2)),
            .active_edge_mode = @intFromBool(gen.oneIn(2)),
            .use_shrunken_shape = @intFromBool(gen.oneIn(2)),
            .return_deepest_point = @intFromBool(gen.oneIn(2)),
            .collect_faces = @intFromBool(gen.oneIn(2)),
            .collector = collectorKind(&gen, &desc),
            .early_out = if (gen.oneIn(4)) gen.plain(-0.5, 1) else 2.0,
            .body_id = gen.next() & 0x7fffff,
            .reject_sub_type = gen.reject(),
            .reject_id = if (gen.oneIn(2)) gen.rejectID(pair.shape1.get().?, creator1) else gen.rejectID(pair.shape2.get().?, creator2),
        };
        jolt_output.* = std.mem.zeroes(HitsOutput);
        jolt.jolt_compounds_cast(&desc, &input, jolt_output);
        const zolt_output = try zoltCast(allocator, &desc, &input);
        num_hits += zolt_output.num_hits;
        checker.check(.{ desc.kind, desc.num_sub_shapes, input }, zolt_output, jolt_output.*);
    }
    try checker.finish();
    try expectBetween("cast hits", num_hits, iterations / 200, 1000 * iterations);
}

test "Compounds parity: GetSubmergedVolume and CollideSoftBodyVertices" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var volumes: Checker = .{ .name = "submerged volume" };
    var soft_bodies: Checker = .{ .name = "soft body vertices" };
    const n = 8;
    for (0..iterations / 50) |i| {
        const desc = gen.compoundDesc(@intCast(i % 2));
        var shape_ref = try createShape(allocator, &desc);
        defer shape_ref.deinit();
        const shape = shape_ref.get().?;
        const scale = gen.validScale(shape);
        const transform = arr16(gen.transform(3));

        const normal = vec3(gen.direction(1)).normalizedOr(Vec3.axisY());
        const plane = arr4(Plane.fromPointAndNormal(vec3(gen.vec(-5, 5)), normal).normal_and_constant);
        var jolt_values: [5]f32 = undefined;
        jolt.jolt_compounds_submerged_volume(&desc, &transform, &scale, &plane, &jolt_values);
        const r = shape.getSubmergedVolume(mat44(transform), vec3(scale), .fromVec4(vec4(plane)));
        const zolt_values = [_]f32{ r.total_volume, r.submerged_volume } ++ arr3(r.center_of_buoyancy);
        volumes.check(.{ desc.kind, desc.num_sub_shapes, scale, transform, plane }, zolt_values, jolt_values);

        const extent = extentOf(shape, scale) * 1.2 + 0.5;
        var positions: [n * 3]f32 = undefined;
        var inv_masses: [n]f32 = undefined;
        var penetrations: [n]f32 = undefined;
        const planes: [n * 4]f32 = @splat(0);
        const indices: [n]c_int = @splat(-1);
        for (0..n) |v| {
            positions[3 * v ..][0..3].* = arr3(mat44(transform).mulVec3(vec3(gen.vec(-extent, extent))));
            inv_masses[v] = if (gen.oneIn(5)) 0.0 else 1.0;
            penetrations[v] = if (gen.oneIn(2)) -math.flt_max else gen.plain(-2, 2);
        }
        var jolt_penetrations = penetrations;
        var jolt_planes = planes;
        var jolt_indices = indices;
        jolt.jolt_compounds_soft_body(&desc, &transform, &scale, n, &positions, &inv_masses, &jolt_penetrations, &jolt_planes, &jolt_indices, 3);

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
        shape.collideSoftBodyVertices(mat44(transform), vec3(scale), &vertices, n, 3);
        var zolt_plane_values: [n * 4]f32 = undefined;
        for (0..n) |v| zolt_plane_values[4 * v ..][0..4].* = arr4(zolt_planes[v].normal_and_constant);
        var zolt_index_values: [n]c_int = undefined;
        for (0..n) |v| zolt_index_values[v] = zolt_indices[v];
        soft_bodies.check(.{ desc.kind, desc.num_sub_shapes, scale, transform, positions }, .{ zolt_penetrations, zolt_plane_values, zolt_index_values }, .{ jolt_penetrations, jolt_planes, jolt_indices });
    }
    try finishAll(&.{ &volumes, &soft_bodies });
}

test "Compounds parity: binary state (tree nodes, bounds blocks), sub shape state, SaveWithChildren and the restores" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "binary state" };
    const jolt_output = try allocator.create(BinaryOutput);
    defer allocator.destroy(jolt_output);
    const zolt_output = try allocator.create(BinaryOutput);
    defer allocator.destroy(zolt_output);
    for (0..iterations / 100) |i| {
        const desc = gen.compoundDesc(@intCast(i % 2));
        jolt_output.* = .{};
        jolt.jolt_compounds_binary_state(&desc, &jolt_output.bytes[0], &jolt_output.bytes[1], &jolt_output.bytes[2], &jolt_output.bytes[3], binary_capacity, &jolt_output.sizes);
        var shape = try createShape(allocator, &desc);
        defer shape.deinit();
        zolt_output.* = .{};
        try zoltBinaryState(allocator, shape.get().?, zolt_output);
        // Compare the used part of the buffers
        var same = std.mem.eql(u32, &zolt_output.sizes, &jolt_output.sizes);
        for (0..4) |b| same = same and std.mem.eql(u8, zolt_output.bytes[b][0..@min(zolt_output.sizes[b], binary_capacity)], jolt_output.bytes[b][0..@min(jolt_output.sizes[b], binary_capacity)]);
        checker.check(.{ desc.kind, desc.num_sub_shapes }, .{ zolt_output.sizes, same }, .{ jolt_output.sizes, true });
        try std.testing.expect(zolt_output.sizes[0] <= binary_capacity and zolt_output.sizes[2] <= binary_capacity);
        try std.testing.expect(zolt_output.sizes[1] == zolt_output.sizes[0] and zolt_output.sizes[3] == zolt_output.sizes[2]); // The restores succeed
    }
    try checker.finish();
}

test "Compounds parity: Shape::ScaleShape" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .allocator = allocator };
    var checker: Checker = .{ .name = "scale shape" };
    var num_compounds: usize = 0;
    const jolt_bytes = try allocator.alloc(u8, binary_capacity);
    defer allocator.free(jolt_bytes);
    const zolt_bytes = try allocator.alloc(u8, binary_capacity);
    defer allocator.free(zolt_bytes);
    for (0..iterations / 100) |i| {
        var desc = gen.compoundDesc(@intCast(i % 2));
        if (desc.num_sub_shapes > 30) desc.num_sub_shapes = 30;
        // Any scale: uniform, non-uniform (invalid for rotated sub shapes, then the leaves are collected), mirrored, zero
        const scale: P = switch (gen.index(6)) {
            0 => .{ 0, 0, 0 },
            1 => blk: {
                const m = gen.plain(0.3, 2);
                break :blk .{ m, m, m };
            },
            2 => .{ 1, 1, 1 },
            3 => .{ gen.plain(-2, -0.3), gen.plain(0.3, 2), gen.plain(0.3, 2) },
            else => gen.plainVec(0.3, 2),
        };
        var jolt_error: [128]u8 = undefined;
        var jolt_info: [2]u32 = undefined;
        var jolt_bounds: [6]f32 = @splat(0);
        var jolt_size: u32 = 0;
        const jolt_valid = jolt.jolt_compounds_scale_shape(&desc, &scale, &jolt_error, &jolt_info, &jolt_bounds, jolt_bytes.ptr, binary_capacity, &jolt_size);

        var shape = try createShape(allocator, &desc);
        defer shape.deinit();
        var result = try shape.get().?.scaleShape(allocator, vec3(scale));
        defer result.deinit();
        var zolt_error: [128]u8 = @splat(0);
        var zolt_info: [2]u32 = @splat(0);
        var zolt_bounds: [6]f32 = @splat(0);
        var zolt_size: u32 = 0;
        const zolt_valid: c_int = @intFromBool(result.isValid());
        if (result.isValid()) {
            const scaled = result.getPtr().?;
            zolt_info = .{ @intFromEnum(scaled.getSubType()), if (scaled.getType() == .compound) scaled.cast(CompoundShape).getNumSubShapes() else 0 };
            zolt_bounds = boxArr(scaled.getLocalBounds());
            zolt_size = try saveShapeWithChildren(allocator, scaled, zolt_bytes);
            if (scaled.getSubType() == .static_compound) num_compounds += 1;
        } else {
            const text = result.getError();
            @memcpy(zolt_error[0..@min(text.len, 127)], text[0..@min(text.len, 127)]);
        }
        const same = zolt_size == jolt_size and std.mem.eql(u8, zolt_bytes[0..@min(zolt_size, binary_capacity)], jolt_bytes[0..@min(jolt_size, binary_capacity)]);
        checker.check(.{ desc.kind, desc.num_sub_shapes, scale }, .{ zolt_valid, zolt_error, zolt_info, zolt_bounds, zolt_size, same }, .{ jolt_valid, jolt_error, jolt_info, jolt_bounds, jolt_size, true });
    }
    try checker.finish();
    try expectBetween("scaled compounds", num_compounds, 10, iterations / 100);
}
