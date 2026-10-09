//! Parity tests for the abstract bases of the decorated and compound shapes (Phase 4): CompoundShape::SubShape (the
//! transform and the compressed position / rotation), the construction of a compound (center of mass, inner radius,
//! bounds, the sub shapes), every override of CompoundShape and DecoratedShape, the binary and sub shape state, the
//! visitors of CompoundShapeVisitors.h (their TestBounds math and the queries that walk them, including the ones that go
//! through CollisionDispatch) and CompoundShape::sCastCompoundVsShape. Zolt and the C++ Jolt library run on the same
//! inputs and must produce identical bits. C ABI wrappers: ZoltParity/Physics/CompositeReference.cpp. See
//! ZoltParity/parity.zig for how parity tests work.
//!
//! The concrete compounds (StaticCompoundShape / MutableCompoundShape) are ported in Wave B, so these tests use test
//! classes that derive from the abstract bases, the same classes on both sides: ParityCompoundShape (User5, walks its
//! sub shapes linearly with the visitors and tests the bounds of every sub shape with TestBounds), ParityDecoratedShape
//! (User6, passes everything on) and CompositeChild (User3, a box that records the calls it receives in a call log).
//! The end-to-end queries of the concrete compounds (their trees / blocks of 4 bounds) are left to the Wave B parity
//! tests.
//!
//! The visitors that dispatch (CollideCompoundVsShape, CollideShapeVsCompound, CastShape, sCastCompoundVsShape) use the
//! parity shape of the shape core tests (ParityShape of ShapeCoreUserTypes.zig, User1, registered in the parity build
//! with collide / cast functions that record their inputs and compute hits from them); the C++ reference uses a copy of
//! it and of its functions that it installs for (User1, User1) during the call.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const Allocator = std.mem.Allocator;
const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const CastRayCollector = zolt.CastRayCollector;
const CastShapeCollector = zolt.CastShapeCollector;
const ClosestHitCollisionCollector = zolt.ClosestHitCollisionCollector;
const CollidePointCollector = zolt.CollidePointCollector;
const CollidePointResult = zolt.CollidePointResult;
const CollideShapeCollector = zolt.CollideShapeCollector;
const CollideShapeResult = zolt.CollideShapeResult;
const CollideShapeSettings = zolt.CollideShapeSettings;
const CollideSoftBodyVertexIterator = zolt.CollideSoftBodyVertexIterator;
const CompoundShape = zolt.CompoundShape;
const CompoundShapeSettings = zolt.CompoundShapeSettings;
const Core = zolt.Core;
const DecoratedShape = zolt.DecoratedShape;
const DecoratedShapeSettings = zolt.DecoratedShapeSettings;
const Float3 = zolt.Float3;
const MassProperties = zolt.MassProperties;
const Mat44 = zolt.Mat44;
const OrientedBox = zolt.OrientedBox;
const PhysicsMaterial = zolt.PhysicsMaterial;
const Plane = zolt.Plane;
const Quat = zolt.Quat;
const RayCast = zolt.RayCast;
const RayCastResult = zolt.RayCastResult;
const RayCastSettings = zolt.RayCastSettings;
const Real = zolt.Real;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RVec3 = zolt.RVec3;
const ScaleHelpers = zolt.ScaleHelpers;
const Shape = zolt.Shape;
const ShapeCast = zolt.ShapeCast;
const ShapeCastResult = zolt.ShapeCastResult;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeFilter = zolt.ShapeFilter;
const ShapeList = zolt.ShapeList;
const ShapeResult = zolt.ShapeResult;
const ShapeSettings = zolt.ShapeSettings;
const ShapeSubType = zolt.ShapeSubType;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TransformedShape = zolt.TransformedShape;
const TransformedShapeCollector = zolt.TransformedShapeCollector;
const UVec4 = zolt.UVec4;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;
const SubShape = CompoundShape.SubShape;

/// The parity test shape of the shape core tests (User1) and its records (the zolt_user_types module of the parity
/// build, see build.zig)
const parity_user_types = @import("parity_user_types");
const ParityShape = parity_user_types.ParityShape;
const CollideRecord = parity_user_types.CollideRecord;
const CastRecord = parity_user_types.CastRecord;

/// The C++ reference functions, see CompositeReference.cpp
const jolt = struct {
    extern fn jolt_composite_sub_shape(input: *const SubShapeInput, output: *SubShapeOutput) void;
    extern fn jolt_composite_compound(desc: *const CompoundDesc, queries: *const CompoundQueries, output: *CompoundOutput) void;
    extern fn jolt_composite_test_bounds(input: *const BoundsInput, output: *BoundsOutput) void;
    extern fn jolt_composite_visitors(input: *const VisitorInput, output: *VisitorOutput) void;
    extern fn jolt_composite_decorated(input: *const DecoratedInput, output: *DecoratedOutput) void;
};

/// Number of random inputs per test (the compound tests create shapes and run many queries per input)
const iterations = fw.iterations / 10;

/// A point / vector as passed to the C ABI
const P = [3]f32;

// ---------------------------------------------------------------------------------------------------------------------
// The C ABI structures, must match the structs in CompositeReference.cpp

/// The call kinds of the call log (ECall)
const Call = enum(u32) { material, surface_normal, supporting_face, submerged_volume, cast_ray, cast_ray_collector, collide_point, soft_body, collect_transformed_shapes, transform_shape };

/// A call that a CompositeChild received
const CallRecord = extern struct {
    kind: u32,
    child: u32,
    ids: [4]u32,
    values: [24]f32,
};

const max_calls = 256;

/// The calls that the CompositeChild shapes of a test received (all records after `count` are zero)
const CallLog = extern struct {
    count: u32,
    calls: [max_calls]CallRecord,
};

/// A child shape of the composite parity tests
const ChildDesc = extern struct {
    half_extent: P,
    center_of_mass: P,
    density: f32,
    sub_shape_id_bits: u32,
    uniform_scale: u32,
    must_be_static: u32,
    user_data: u32,
};

/// A sub shape of a compound description
const SubShapeDesc = extern struct {
    position: P,
    rotation: [4]f32,
    child: u32,
    user_data: u32,
    from_settings: u32,
};

/// A compound of CompositeChild shapes
const CompoundDesc = extern struct {
    num_children: u32,
    num_sub_shapes: u32,
    children: [4]ChildDesc,
    sub_shapes: [16]SubShapeDesc,
};

/// A TransformedShape
const TSOut = extern struct {
    position_com: [3]Real,
    rotation: [4]f32,
    scale: P,
    body_id: u32,
    sub_shape_id: u32,
    sub_shape_id_bits: u32,
    child: u32,
};

const SubShapeState = extern struct {
    position_com: P,
    rotation: P,
    user_data: u32,
    is_rotation_identity: u32,
    child: u32,
};

const CompoundState = extern struct {
    valid: u32,
    @"error": [128]u8,
    center_of_mass: P,
    local_bounds: [6]f32,
    inner_radius: f32,
    mass: f32,
    inertia: [16]f32,
    volume: f32,
    must_be_static: u32,
    sub_shape_id_bits_recursive: u32,
    sub_shape_id_bits: u32,
    num_triangles: u32,
    num_sub_shapes: u32,
    sub_shapes: [16]SubShapeState,
};

const CompoundQueries = extern struct {
    transform: [16]f32,
    scale: P,
    test_scales: [4]P,
    ids: [4]u32,
    raw_ids: [4]u32,
    position: P,
    rotation: [4]f32,
    local_position: P,
    direction: P,
    surface: [4]f32,
    ray_origin: P,
    ray_direction: P,
    creator: [2]u32,
    hit_fraction: f32,
    back_faces: u32,
    collector_kind: u32,
    point: P,
    box: [6]f32,
    oriented_box: [19]f32,
    max_indices: u32,
};

const CompoundOutput = extern struct {
    state: CompoundState,
    restored: CompoundState,
    num_bytes: u32,
    bytes: [1024]u8,
    world_bounds: [6]f32,
    scale_valid: [4]u32,
    made_valid: [4]P,
    raw_id_valid: [4]u32,
    id_valid: [4]u32,
    index: [4]u32,
    remainder: [4]u32,
    leaf_child: [4]u32,
    leaf_remainder: [4]u32,
    user_data: [4][2]u32,
    material_is_default: [4]u32,
    sub_ts: [4]TSOut,
    sub_ts_remainder: [4]u32,
    normal: [4]P,
    face_count: [4]u32,
    face: [4][2]P,
    submerged: [5]f32,
    num_transformed: u32,
    transformed: [16]TSOut,
    ray_hit: u32,
    ray_fraction: f32,
    ray_id: u32,
    num_ray_hits: u32,
    ray_hit_fractions: [32]f32,
    ray_hit_ids: [32]u32,
    num_point_hits: u32,
    point_hit_ids: [16]u32,
    num_collected: u32,
    collected: [16]TSOut,
    num_intersecting: [2]u32,
    intersecting: [2][16]u32,
    log: CallLog,
};

const BoundsInput = extern struct {
    bounds: [24]f32,
    ray_origin: P,
    ray_direction: P,
    point: P,
    cast_bounds: [6]f32,
    cast_direction: P,
    extra_convex_radius: f32,
    scale: P,
    collect_box: [6]f32,
    position: P,
    rotation: [4]f32,
    other_half_extent: P,
    scale2: P,
    transform1: [16]f32,
    transform2: [16]f32,
    max_separation: f32,
    box: [6]f32,
    oriented_box: [19]f32,
    num_sub_shapes: u32,
};

const BoundsOutput = extern struct {
    ray: [4]f32,
    ray_collector: [4]f32,
    point: [4]u32,
    cast: [4]f32,
    collect: [4]u32,
    compound_vs_shape: [4]u32,
    shape_vs_compound: [4]u32,
    aabox: [4]u32,
    oriented_box: [4]u32,
    sub_shape_bits: u32,
    box_center: P,
    box_extent: P,
    bounds_of2: [6]f32,
    bounds_of1: [6]f32,
    local_box: [19]f32,
};

const SubShapeInput = extern struct {
    child_center_of_mass: P,
    position: P,
    rotation: [4]f32,
    compound_center_of_mass: P,
    scales: [4]P,
    rotation2: [4]f32,
    position2: P,
};

const SubShapeOutput = extern struct {
    stored_position: P,
    stored_rotation: P,
    is_rotation_identity: u32,
    rotation: [4]f32,
    position_com: P,
    valid: [4]u32,
    transform_scale: [4]P,
    local_transform: [4][16]f32,
    round_trip_rotation: [4]f32,
    round_trip_position: P,
    round_trip_stored: [6]f32,
};

const ParityShapeDesc = extern struct {
    half_extent: P,
    center_of_mass: P,
};

const VisitorInput = extern struct {
    num_sub_shapes: u32,
    children: [16]ParityShapeDesc,
    positions: [16]P,
    rotations: [16][4]f32,
    other: ParityShapeDesc,
    hit_values: [2]f32,
    transform1: [16]f32,
    transform2: [16]f32,
    scale1: P,
    scale2: P,
    creators: [2][2]u32,
    max_separation: f32,
    collector_kind: u32,
    cast_start: [16]f32,
    cast_direction: P,
};

const HitOut = extern struct {
    contact1: P,
    contact2: P,
    axis: P,
    value: f32,
    back_face: u32,
    ids: [2]u32,
    body_id: u32,
    face_counts: [2]u32,
    faces: [2][2]P,
};

const VisitorOutput = extern struct {
    num_hits: [4]u32,
    hits: [4][16]HitOut,
    collide: [2]CollideRecord,
    cast: [2]CastRecord,
};

const DecoratedInput = extern struct {
    child: ChildDesc,
    mode: u32,
    id: u32,
    direction: P,
    scale: P,
    transform: [16]f32,
    test_scales: [4]P,
    user_data: u64,
};

const DecoratedOutput = extern struct {
    valid: u32,
    @"error": [128]u8,
    must_be_static: u32,
    center_of_mass: P,
    sub_shape_id_bits: u32,
    leaf_child: u32,
    leaf_remainder: u32,
    material_is_default: u32,
    user_data: [2]u32,
    face_count: u32,
    face: [2]P,
    scale_valid: [4]u32,
    made_valid: [4]P,
    num_triangles: u32,
    num_sub_shapes: u32,
    sub_shape_child: u32,
    num_bytes: u32,
    bytes: [64]u8,
    num_calls: u32,
    calls: [4]CallRecord,
};

// ---------------------------------------------------------------------------------------------------------------------
// Helpers

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

fn aabox(a: [6]f32) AABox {
    return .init(vec3(a[0..3].*), vec3(a[3..6].*));
}

fn orientedBox(a: [19]f32) OrientedBox {
    return .init(mat44(a[0..16].*), vec3(a[16..19].*));
}

fn arrR3(v: RVec3) [3]Real {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn makeSubShapeID(value: u32) SubShapeID {
    return .{ .value = value };
}

/// The sub shape ID creator (ID, bits) of the C ABI (LoadCreator)
fn loadCreator(c: [2]u32) SubShapeIDCreator {
    return if (c[1] == 0) .{} else SubShapeIDCreator.pushID(.{}, c[0], c[1]);
}

fn creatorArr(c: SubShapeIDCreator) [2]u32 {
    return .{ c.getID().getValue(), c.getNumBitsWritten() };
}

/// The record of the next call (an overflow record when the log is full)
var overflow_record: CallRecord = undefined;

fn newCall(log: *CallLog, kind: Call, child: u32) *CallRecord {
    const r = if (log.count < max_calls) &log.calls[log.count] else &overflow_record;
    if (log.count < max_calls)
        log.count += 1;
    r.* = std.mem.zeroes(CallRecord);
    r.kind = @intFromEnum(kind);
    r.child = child;
    return r;
}

fn put3(r: *CallRecord, offset: usize, v: Vec3) void {
    r.values[offset..][0..3].* = arr3(v);
}

fn put4(r: *CallRecord, offset: usize, v: Vec4) void {
    r.values[offset..][0..4].* = arr4(v);
}

fn put16(r: *CallRecord, offset: usize, m: Mat44) void {
    r.values[offset..][0..16].* = arr16(m);
}

fn putCreator(r: *CallRecord, c: SubShapeIDCreator) void {
    r.ids[0] = c.getID().getValue();
    r.ids[1] = c.getNumBitsWritten();
}

// ---------------------------------------------------------------------------------------------------------------------
// The test classes, must match the classes in CompositeReference.cpp

/// A box around its center of mass that records the calls it receives (User3)
const CompositeChild = struct {
    pub const shape_sub_type: ShapeSubType = .user3;
    pub const overrides = .{ .mustBeStatic, .getCenterOfMass, .getLocalBounds, .getSubShapeIDBitsRecursive, .getInnerRadius, .getMassProperties, .getMaterial, .getSurfaceNormal, .getSupportingFace, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .collectTransformedShapes, .transformShape, .getTrianglesStart, .getTrianglesNext, .getStats, .getVolume, .isValidScale, .makeScaleValid };

    base: Shape,
    half_extent: Vec3 = Vec3.one(),
    center_of_mass: Vec3 = Vec3.zero(),
    density: f32 = 1.0,
    sub_shape_id_bits: u32 = 0,
    uniform_scale: bool = false,
    must_be_static: bool = false,
    index: u32 = 0,
    log: ?*CallLog = null,

    pub fn initDefault(allocator: Allocator) CompositeChild {
        return .{ .base = .init(Shape.vtableFor(CompositeChild), allocator, .user1, shape_sub_type) };
    }

    fn init(allocator: Allocator, desc: ChildDesc, index: u32, log: *CallLog) CompositeChild {
        var self = initDefault(allocator);
        self.half_extent = vec3(desc.half_extent);
        self.center_of_mass = vec3(desc.center_of_mass);
        self.density = desc.density;
        self.sub_shape_id_bits = desc.sub_shape_id_bits;
        self.uniform_scale = desc.uniform_scale != 0;
        self.must_be_static = desc.must_be_static != 0;
        self.index = index;
        self.log = log;
        self.base.setUserData(desc.user_data);
        return self;
    }

    fn create(allocator: Allocator, desc: ChildDesc, index: u32, log: *CallLog) Allocator.Error!*CompositeChild {
        const self = try allocator.create(CompositeChild);
        self.* = .init(allocator, desc, index, log);
        return self;
    }

    /// CompositeChildSettings::Create (the error check of the settings)
    pub fn initFromSettings(self: *CompositeChild, settings: *const CompositeChildSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        _ = allocator;
        if (Vec3.lessOrEqual(vec3(settings.desc.half_extent), Vec3.zero()).testAnyXYZTrue()) {
            result.setError("Invalid half extent");
            return;
        }
        // The fields of `init` (the base was constructed in place by initDefault and is referenced already)
        const desc = settings.desc;
        self.half_extent = vec3(desc.half_extent);
        self.center_of_mass = vec3(desc.center_of_mass);
        self.density = desc.density;
        self.sub_shape_id_bits = desc.sub_shape_id_bits;
        self.uniform_scale = desc.uniform_scale != 0;
        self.must_be_static = desc.must_be_static != 0;
        self.index = settings.index;
        self.log = settings.log;
        self.base.setUserData(desc.user_data);
        result.set(.init(&self.base));
    }

    fn asShape(self: *const CompositeChild) *const Shape {
        return &self.base;
    }

    fn newRecord(self: *const CompositeChild, kind: Call) *CallRecord {
        return newCall(self.log.?, kind, self.index);
    }

    pub fn mustBeStatic(self: *const CompositeChild) bool {
        return self.must_be_static;
    }

    pub fn getCenterOfMass(self: *const CompositeChild) Vec3 {
        return self.center_of_mass;
    }

    pub fn getLocalBounds(self: *const CompositeChild) AABox {
        return .init(self.half_extent.negate(), self.half_extent);
    }

    pub fn getSubShapeIDBitsRecursive(self: *const CompositeChild) u32 {
        return self.sub_shape_id_bits;
    }

    pub fn getInnerRadius(self: *const CompositeChild) f32 {
        return self.half_extent.reduceMin();
    }

    pub fn getMassProperties(self: *const CompositeChild) MassProperties {
        var p: MassProperties = .{};
        p.setMassAndInertiaOfSolidBox(self.half_extent.mulScalar(2.0), self.density);
        return p;
    }

    pub fn getStats(self: *const CompositeChild) Shape.Stats {
        _ = self;
        return .init(@sizeOf(CompositeChild), 3);
    }

    pub fn getVolume(self: *const CompositeChild) f32 {
        return 8.0 * self.half_extent.getX() * self.half_extent.getY() * self.half_extent.getZ();
    }

    pub fn getTrianglesNext(self: *const CompositeChild, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        _ = .{ self, context, max_triangles_requested, out_triangle_vertices, out_materials };
        return 0;
    }

    pub fn getTrianglesStart(self: *const CompositeChild, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        _ = .{ self, context, box, position_com, rotation, scale };
    }

    pub fn getMaterial(self: *const CompositeChild, sub_shape_id: SubShapeID) *const PhysicsMaterial {
        const r = self.newRecord(.material);
        r.ids[0] = sub_shape_id.getValue();
        return PhysicsMaterial.default;
    }

    pub fn getSurfaceNormal(self: *const CompositeChild, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        const r = self.newRecord(.surface_normal);
        r.ids[0] = sub_shape_id.getValue();
        put3(r, 0, local_surface_position);
        return local_surface_position.normalizedOr(Vec3.axisY());
    }

    pub fn getSupportingFace(self: *const CompositeChild, sub_shape_id: SubShapeID, direction: Vec3, scale: Vec3, center_of_mass_transform: Mat44, out_vertices: *Shape.SupportingFace) void {
        const r = self.newRecord(.supporting_face);
        r.ids[0] = sub_shape_id.getValue();
        put3(r, 0, direction);
        put3(r, 3, scale);
        put16(r, 6, center_of_mass_transform);
        out_vertices.append(center_of_mass_transform.mulVec3(scale.mul(self.half_extent)));
        out_vertices.append(center_of_mass_transform.mulVec3(direction));
    }

    pub fn getSubmergedVolume(self: *const CompositeChild, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        const r = self.newRecord(.submerged_volume);
        put16(r, 0, center_of_mass_transform);
        put3(r, 16, scale);
        put3(r, 19, surface.getNormal());
        r.values[22] = surface.getConstant();
        const total_volume = self.getVolume() * @abs(scale.getX() * scale.getY() * scale.getZ());
        const center = center_of_mass_transform.getTranslation();
        const distance = surface.signedDistance(center);
        return .{
            .total_volume = total_volume,
            .submerged_volume = if (distance < 0.0) total_volume else 0.0,
            .center_of_buoyancy = if (distance < 0.0) center else Vec3.zero(),
        };
    }

    pub fn castRay(self: *const CompositeChild, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        const r = self.newRecord(.cast_ray);
        putCreator(r, sub_shape_id_creator);
        put3(r, 0, ray.origin);
        put3(r, 3, ray.direction);
        r.values[6] = hit.fraction;
        const fraction = math.max(zolt.rayAABox(ray.origin, .init(ray.direction), self.half_extent.negate(), self.half_extent), 0.0);
        if (fraction < hit.fraction) {
            hit.fraction = fraction;
            hit.sub_shape_id2 = sub_shape_id_creator.getID();
            return true;
        }
        return false;
    }

    pub fn castRayCollector(self: *const CompositeChild, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        const r = self.newRecord(.cast_ray_collector);
        putCreator(r, sub_shape_id_creator);
        r.ids[2] = @intFromBool(ray_cast_settings.back_face_mode_convex == .collide_with_back_faces);
        put3(r, 0, ray.origin);
        put3(r, 3, ray.direction);
        r.values[6] = collector.getEarlyOutFraction();

        const min_max = zolt.rayAABoxMinMax(ray.origin, .init(ray.direction), self.half_extent.negate(), self.half_extent);
        if (min_max.min > min_max.max or min_max.max < 0.0)
            return;
        const body_id = TransformedShape.getBodyID(collector.getContext());
        const front = math.max(min_max.min, 0.0);
        if (front < collector.getEarlyOutFraction()) {
            collector.addHit(&.{ .body_id = body_id, .fraction = front, .sub_shape_id2 = sub_shape_id_creator.getID() });
            if (collector.shouldEarlyOut())
                return;
        }
        if (ray_cast_settings.back_face_mode_convex == .collide_with_back_faces and min_max.max <= 1.0 and min_max.max < collector.getEarlyOutFraction())
            collector.addHit(&.{ .body_id = body_id, .fraction = min_max.max, .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    pub fn collidePoint(self: *const CompositeChild, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        const r = self.newRecord(.collide_point);
        putCreator(r, sub_shape_id_creator);
        put3(r, 0, point);
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;
        if (AABox.init(self.half_extent.negate(), self.half_extent).containsVec3(point))
            collector.addHit(&.{ .body_id = TransformedShape.getBodyID(collector.getContext()), .sub_shape_id2 = sub_shape_id_creator.getID() });
    }

    pub fn collideSoftBodyVertices(self: *const CompositeChild, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        _ = vertices;
        const r = self.newRecord(.soft_body);
        r.ids[0] = num_vertices;
        r.ids[1] = @bitCast(colliding_shape_index);
        put16(r, 0, center_of_mass_transform);
        put3(r, 16, scale);
    }

    pub fn collectTransformedShapes(self: *const CompositeChild, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        const r = self.newRecord(.collect_transformed_shapes);
        putCreator(r, sub_shape_id_creator);
        r.values[0..6].* = boxArr(box);
        put3(r, 6, position_com);
        put4(r, 9, rotation.getXYZW());
        put3(r, 13, scale);
        Shape.impl.collectTransformedShapes(self.asShape(), box, position_com, rotation, scale, sub_shape_id_creator, collector, shape_filter);
    }

    pub fn transformShape(self: *const CompositeChild, center_of_mass_transform: Mat44, collector: *TransformedShapeCollector) void {
        const r = self.newRecord(.transform_shape);
        put16(r, 0, center_of_mass_transform);
        Shape.impl.transformShape(self.asShape(), center_of_mass_transform, collector);
    }

    pub fn isValidScale(self: *const CompositeChild, scale: Vec3) bool {
        return Shape.impl.isValidScale(self.asShape(), scale) and (!self.uniform_scale or ScaleHelpers.isUniformScale(scale));
    }

    pub fn makeScaleValid(self: *const CompositeChild, scale: Vec3) Vec3 {
        const s = Shape.impl.makeScaleValid(self.asShape(), scale);
        return if (self.uniform_scale) ScaleHelpers.makeUniformScale(s) else s;
    }
};

/// Settings of CompositeChild
const CompositeChildSettings = struct {
    pub const overrides = .{.createShape};

    base: ShapeSettings,
    desc: ChildDesc,
    index: u32,
    log: *CallLog,

    fn create(allocator: Allocator, desc: ChildDesc, index: u32, log: *CallLog) Allocator.Error!*CompositeChildSettings {
        const self = try allocator.create(CompositeChildSettings);
        self.* = .{ .base = .init(ShapeSettings.vtableFor(CompositeChildSettings), allocator), .desc = desc, .index = index, .log = log };
        return self;
    }

    pub fn createShape(self: *CompositeChildSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(CompositeChild, self, allocator);
    }
};

/// The index of a CompositeChild (0xffffffff for null, 0xfffffffe for other shapes)
fn childIndex(shape: ?*const Shape) u32 {
    const s = shape orelse return 0xffffffff;
    if (s.getSubType() == .user3) return s.cast(CompositeChild).index;
    return 0xfffffffe;
}

/// Settings of ParityCompoundShape
const ParityCompoundShapeSettings = struct {
    pub const overrides = .{.createShape};

    base: CompoundShapeSettings,

    fn create(allocator: Allocator) Allocator.Error!*ParityCompoundShapeSettings {
        const self = try allocator.create(ParityCompoundShapeSettings);
        self.* = .{ .base = .init(ParityCompoundShapeSettings, allocator) };
        return self;
    }

    pub fn createShape(self: *ParityCompoundShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(ParityCompoundShape, self, allocator);
    }
};

/// A compound (User5) that walks its sub shapes linearly with the visitors of CompoundShapeVisitors.zig. For every sub
/// shape it calls testBounds with the bounds of the sub shape in lane 0 (the other lanes repeat it) and visits the sub
/// shape when the result passes. The constructor is StaticCompoundShape's without the tree (any number of sub shapes).
const ParityCompoundShape = struct {
    pub const shape_sub_type: ShapeSubType = .user5;
    pub const overrides = .{ .castRay, .castRayCollector, .collidePoint, .collectTransformedShapes, .getIntersectingSubShapes, .getIntersectingSubShapesOrientedBox, .getStats };

    base: CompoundShape,

    pub fn initDefault(allocator: Allocator) ParityCompoundShape {
        return .{ .base = .init(ParityCompoundShape, allocator, shape_sub_type) };
    }

    pub fn initFromSettings(self: *ParityCompoundShape, settings: *const ParityCompoundShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        self.base.initFromSettings(&settings.base.base, result);
        const c = &self.base;

        // Keep track of total mass to calculate center of mass
        var mass: f32 = 0.0;

        try c.sub_shapes.appendNTimes(c.base.allocator, .{}, settings.base.sub_shapes.items.len);
        for (settings.base.sub_shapes.items, c.sub_shapes.items) |*shape, *out_shape| {
            // Start constructing the runtime sub shape
            if (!try out_shape.fromSettings(shape, result, allocator))
                return;

            // Calculate mass properties of child
            const child = out_shape.shape.get().?.getMassProperties();

            // Accumulate center of mass
            mass += child.mass;
            c.center_of_mass = c.center_of_mass.add(out_shape.getPositionCOM().mulScalar(child.mass));
        }

        if (mass > 0.0)
            c.center_of_mass = c.center_of_mass.divScalar(mass);

        // Cache the inner radius as it can take a while to recursively iterate over all sub shapes
        c.calculateInnerRadius();

        // Shift all shapes so that the center of mass is now at the origin and calculate bounds
        for (c.sub_shapes.items) |*shape| {
            shape.setPositionCOM(shape.getPositionCOM().sub(c.center_of_mass));
            c.local_bounds.encapsulate(subShapeBounds(shape));
        }

        // Check if we're not exceeding the amount of sub shape id bits
        if (c.asShape().getSubShapeIDBitsRecursive() > SubShapeID.max_bits) {
            result.setError("Compound hierarchy is too deep and exceeds the amount of available sub shape ID bits");
            return;
        }

        result.set(.init(c.asShapeMut()));
    }

    fn asShape(self: *const ParityCompoundShape) *const Shape {
        return self.base.asShape();
    }

    /// The bounds of a sub shape in the space of the compound
    fn subShapeBounds(sub_shape: *const SubShape) AABox {
        return sub_shape.shape.get().?.getWorldSpaceBounds(Mat44.rotationTranslation(sub_shape.getRotation(), sub_shape.getPositionCOM()), Vec3.one());
    }

    fn shouldVisit(visitor: anytype, result: anytype) bool {
        if (@TypeOf(result) == UVec4)
            return result.getX() != 0;
        const fraction = if (@TypeOf(visitor.*) == CompoundShape.CastRayVisitor) visitor.hit.fraction else visitor.collector.getEarlyOutFraction();
        return result.getX() < fraction;
    }

    fn walkSubShapes(self: *const ParityCompoundShape, visitor: anytype) void {
        for (self.base.sub_shapes.items, 0..) |*sub_shape, i| {
            const b = subShapeBounds(sub_shape);
            const result = visitor.testBounds(b.min.splatX(), b.min.splatY(), b.min.splatZ(), b.max.splatX(), b.max.splatY(), b.max.splatZ());
            if (shouldVisit(visitor, result)) {
                visitor.visitShape(sub_shape, @intCast(i));
                if (visitor.shouldAbort())
                    break;
            }
        }
    }

    pub fn castRay(self: *const ParityCompoundShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        var visitor: CompoundShape.CastRayVisitor = .init(&ray, &self.base, sub_shape_id_creator, hit);
        self.walkSubShapes(&visitor);
        return visitor.return_value;
    }

    pub fn castRayCollector(self: *const ParityCompoundShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        var visitor: CompoundShape.CastRayVisitorCollector = .init(&ray, ray_cast_settings, &self.base, sub_shape_id_creator, collector, shape_filter);
        self.walkSubShapes(&visitor);
    }

    pub fn collidePoint(self: *const ParityCompoundShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        var visitor: CompoundShape.CollidePointVisitor = .init(point, &self.base, sub_shape_id_creator, collector, shape_filter);
        self.walkSubShapes(&visitor);
    }

    pub fn collectTransformedShapes(self: *const ParityCompoundShape, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *TransformedShapeCollector, shape_filter: *const ShapeFilter) void {
        // Test shape filter
        if (!shape_filter.shouldCollide(self.asShape(), sub_shape_id_creator.getID()))
            return;

        var visitor: CompoundShape.CollectTransformedShapesVisitor = .init(box, &self.base, position_com, rotation, scale, sub_shape_id_creator, collector, shape_filter);
        self.walkSubShapes(&visitor);
    }

    pub fn getIntersectingSubShapes(self: *const ParityCompoundShape, box: AABox, out_sub_shape_indices: []u32) u32 {
        var visitor: CompoundShape.GetIntersectingSubShapesVisitor(AABox) = .init(box, out_sub_shape_indices);
        if (!visitor.shouldAbort()) self.walkSubShapes(&visitor);
        return visitor.getNumResults();
    }

    pub fn getIntersectingSubShapesOrientedBox(self: *const ParityCompoundShape, box: OrientedBox, out_sub_shape_indices: []u32) u32 {
        var visitor: CompoundShape.GetIntersectingSubShapesVisitor(OrientedBox) = .init(box, out_sub_shape_indices);
        if (!visitor.shouldAbort()) self.walkSubShapes(&visitor);
        return visitor.getNumResults();
    }

    pub fn getStats(self: *const ParityCompoundShape) Shape.Stats {
        return .init(@sizeOf(ParityCompoundShape) + self.base.sub_shapes.items.len * @sizeOf(SubShape), 0);
    }

    // The collision functions that a compound registers in CollisionDispatch (called directly)

    fn collideCompoundVsShape(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const compound = shape1.cast(ParityCompoundShape);
        var visitor: CompoundShape.CollideCompoundVsShapeVisitor = .init(&compound.base, shape2, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
        compound.walkSubShapes(&visitor);
    }

    fn collideShapeVsCompound(shape1: *const Shape, shape2: *const Shape, scale1: Vec3, scale2: Vec3, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collide_shape_settings: *const CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) void {
        const compound = shape2.cast(ParityCompoundShape);
        var visitor: CompoundShape.CollideShapeVsCompoundVisitor = .init(shape1, &compound.base, scale1, scale2, center_of_mass_transform1, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collide_shape_settings, collector, shape_filter);
        compound.walkSubShapes(&visitor);
    }

    fn castShapeVsCompound(shape_cast: *const ShapeCast, shape_cast_settings: *const ShapeCastSettings, shape: *const Shape, scale: Vec3, shape_filter: *const ShapeFilter, center_of_mass_transform2: Mat44, sub_shape_id_creator1: SubShapeIDCreator, sub_shape_id_creator2: SubShapeIDCreator, collector: *CastShapeCollector) void {
        const compound = shape.cast(ParityCompoundShape);
        var visitor: CompoundShape.CastShapeVisitor = .init(shape_cast, shape_cast_settings, &compound.base, scale, shape_filter, center_of_mass_transform2, sub_shape_id_creator1, sub_shape_id_creator2, collector);
        compound.walkSubShapes(&visitor);
    }
};

/// Settings of ParityDecoratedShape
const ParityDecoratedShapeSettings = struct {
    pub const overrides = .{.createShape};

    base: DecoratedShapeSettings,

    pub fn createShape(self: *ParityDecoratedShapeSettings, allocator: Allocator) Allocator.Error!ShapeResult {
        return ShapeSettings.createCached(ParityDecoratedShape, self, allocator);
    }
};

/// A decorator (User6) that implements the pure virtual functions of Shape by passing them on to the inner shape
const ParityDecoratedShape = struct {
    pub const shape_sub_type: ShapeSubType = .user6;
    pub const overrides = .{ .getLocalBounds, .getInnerRadius, .getMassProperties, .getSurfaceNormal, .getSubmergedVolume, .castRay, .castRayCollector, .collidePoint, .collideSoftBodyVertices, .getTrianglesStart, .getTrianglesNext, .getStats, .getVolume };

    base: DecoratedShape,

    pub fn initDefault(allocator: Allocator) ParityDecoratedShape {
        return .{ .base = .init(ParityDecoratedShape, allocator, shape_sub_type, null) };
    }

    pub fn initFromSettings(self: *ParityDecoratedShape, settings: *const ParityDecoratedShapeSettings, result: *ShapeResult, allocator: Allocator) Allocator.Error!void {
        try self.base.initFromSettings(&settings.base, result, allocator);
        if (result.hasError())
            return;

        result.set(.init(self.base.asShapeMut()));
    }

    fn inner(self: *const ParityDecoratedShape) *const Shape {
        return self.base.getInnerShape();
    }

    pub fn getLocalBounds(self: *const ParityDecoratedShape) AABox {
        return self.inner().getLocalBounds();
    }

    pub fn getInnerRadius(self: *const ParityDecoratedShape) f32 {
        return self.inner().getInnerRadius();
    }

    pub fn getMassProperties(self: *const ParityDecoratedShape) MassProperties {
        return self.inner().getMassProperties();
    }

    pub fn getSurfaceNormal(self: *const ParityDecoratedShape, sub_shape_id: SubShapeID, local_surface_position: Vec3) Vec3 {
        return self.inner().getSurfaceNormal(sub_shape_id, local_surface_position);
    }

    pub fn getSubmergedVolume(self: *const ParityDecoratedShape, center_of_mass_transform: Mat44, scale: Vec3, surface: Plane) Shape.SubmergedVolume {
        return self.inner().getSubmergedVolume(center_of_mass_transform, scale, surface);
    }

    pub fn castRay(self: *const ParityDecoratedShape, ray: RayCast, sub_shape_id_creator: SubShapeIDCreator, hit: *RayCastResult) bool {
        return self.inner().castRay(ray, sub_shape_id_creator, hit);
    }

    pub fn castRayCollector(self: *const ParityDecoratedShape, ray: RayCast, ray_cast_settings: *const RayCastSettings, sub_shape_id_creator: SubShapeIDCreator, collector: *CastRayCollector, shape_filter: *const ShapeFilter) void {
        self.inner().castRayCollector(ray, ray_cast_settings, sub_shape_id_creator, collector, shape_filter);
    }

    pub fn collidePoint(self: *const ParityDecoratedShape, point: Vec3, sub_shape_id_creator: SubShapeIDCreator, collector: *CollidePointCollector, shape_filter: *const ShapeFilter) void {
        self.inner().collidePoint(point, sub_shape_id_creator, collector, shape_filter);
    }

    pub fn collideSoftBodyVertices(self: *const ParityDecoratedShape, center_of_mass_transform: Mat44, scale: Vec3, vertices: *const CollideSoftBodyVertexIterator, num_vertices: u32, colliding_shape_index: i32) void {
        self.inner().collideSoftBodyVertices(center_of_mass_transform, scale, vertices, num_vertices, colliding_shape_index);
    }

    pub fn getTrianglesStart(self: *const ParityDecoratedShape, context: *Shape.GetTrianglesContext, box: AABox, position_com: Vec3, rotation: Quat, scale: Vec3) void {
        self.inner().getTrianglesStart(context, box, position_com, rotation, scale);
    }

    pub fn getTrianglesNext(self: *const ParityDecoratedShape, context: *Shape.GetTrianglesContext, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        return self.inner().getTrianglesNext(context, max_triangles_requested, out_triangle_vertices, out_materials);
    }

    pub fn getStats(self: *const ParityDecoratedShape) Shape.Stats {
        _ = self;
        return .init(@sizeOf(ParityDecoratedShape), 1);
    }

    pub fn getVolume(self: *const ParityDecoratedShape) f32 {
        return self.inner().getVolume();
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of the C ABI functions

fn storeTS(ts: *const TransformedShape) TSOut {
    return .{
        .position_com = arrR3(ts.shape_position_com),
        .rotation = arr4(ts.shape_rotation.getXYZW()),
        .scale = arr3(ts.getShapeScale()),
        .body_id = ts.body_id.getIndexAndSequenceNumber(),
        .sub_shape_id = ts.sub_shape_id_creator.getID().getValue(),
        .sub_shape_id_bits = ts.sub_shape_id_creator.getNumBitsWritten(),
        .child = childIndex(ts.shape.get()),
    };
}

fn storeState(compound: *const CompoundShape, out: *CompoundState) void {
    out.* = std.mem.zeroes(CompoundState);
    out.valid = 1;
    out.center_of_mass = arr3(compound.center_of_mass);
    out.local_bounds = boxArr(compound.local_bounds);
    out.inner_radius = compound.inner_radius;
    out.num_sub_shapes = compound.getNumSubShapes();
    out.sub_shape_id_bits = compound.getSubShapeIDBits();
    for (compound.getSubShapes(), 0..) |*s, i| {
        if (i >= 16) break;
        out.sub_shapes[i] = .{
            .position_com = .{ s.position_com.x, s.position_com.y, s.position_com.z },
            .rotation = .{ s.rotation.x, s.rotation.y, s.rotation.z },
            .user_data = s.user_data,
            .is_rotation_identity = @intFromBool(s.is_rotation_identity),
            .child = childIndex(s.shape.get()),
        };
    }
}

/// The state that needs the child shapes
fn storeChildState(allocator: Allocator, compound: *const CompoundShape, out: *CompoundState) !void {
    const shape = compound.asShape();
    const p = shape.getMassProperties();
    out.mass = p.mass;
    out.inertia = arr16(p.inertia);
    out.volume = shape.getVolume();
    out.must_be_static = @intFromBool(shape.mustBeStatic());
    out.sub_shape_id_bits_recursive = shape.getSubShapeIDBitsRecursive();
    var visited: Shape.VisitedShapes = .empty;
    defer visited.deinit(allocator);
    out.num_triangles = (try shape.getStatsRecursive(allocator, &visited)).num_triangles;
}

fn storeError(message: []const u8, out: *CompoundState) void {
    out.* = std.mem.zeroes(CompoundState);
    const n = @min(message.len, out.@"error".len - 1);
    @memcpy(out.@"error"[0..n], message[0..n]);
}

/// The compound described by `desc` with CompositeChild children (shapes or settings, the children of the settings get
/// index 100 + i)
fn createCompound(allocator: Allocator, desc: *const CompoundDesc, log: *CallLog) !ShapeResult {
    const settings = try ParityCompoundShapeSettings.create(allocator);
    var settings_ref = Ref(ShapeSettings).init(&settings.base.base);
    defer settings_ref.deinit();

    var shapes: [4]RefConst(Shape) = @splat(.empty);
    defer for (&shapes) |*s| s.deinit();
    var child_settings: [4]Ref(ShapeSettings) = @splat(.empty);
    defer for (&child_settings) |*s| s.deinit();
    for (0..desc.num_children) |i| {
        shapes[i].set(&(try CompositeChild.create(allocator, desc.children[i], @intCast(i), log)).base);
        child_settings[i].set(&(try CompositeChildSettings.create(allocator, desc.children[i], @intCast(100 + i), log)).base);
    }
    for (desc.sub_shapes[0..desc.num_sub_shapes]) |*s| {
        if (s.from_settings != 0)
            try settings.base.addShape(vec3(s.position), quat(s.rotation), child_settings[s.child].get(), .{ .user_data = s.user_data })
        else
            try settings.base.addShapePtr(vec3(s.position), quat(s.rotation), shapes[s.child].get(), .{ .user_data = s.user_data });
    }
    return settings.base.base.createShape(allocator);
}

/// Collector kinds of the queries
const all_hits = 0;
const closest_hit = 1;
const any_hit = 2;

/// Runs `context.query(collector)` with the collector kind and stores the hits with `context.store(hit, index)`
fn collectHits(comptime C: type, allocator: Allocator, kind: u32, max_hits: u32, context: anytype) !u32 {
    var count: u32 = 0;
    switch (kind) {
        closest_hit => {
            var collector = ClosestHitCollisionCollector(C).init();
            defer collector.deinit();
            context.query(&collector.base);
            if (collector.hadHit()) {
                context.store(&collector.hit, count);
                count += 1;
            }
        },
        any_hit => {
            var collector = AnyHitCollisionCollector(C).init();
            defer collector.deinit();
            context.query(&collector.base);
            if (collector.hadHit()) {
                context.store(&collector.hit, count);
                count += 1;
            }
        },
        else => {
            var collector = AllHitCollisionCollector(C).init(allocator);
            defer collector.deinit();
            context.query(&collector.base);
            try collector.checkError();
            for (collector.hits.items) |*hit| {
                if (count < max_hits) {
                    context.store(hit, count);
                    count += 1;
                }
            }
        },
    }
    return count;
}

fn saveBytes(shape: *const Shape, buffer: []u8) []const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamOutWrapper.init(&writer);
    shape.saveBinaryState(out.streamOut());
    return writer.buffered();
}

/// jolt_composite_compound
fn zoltCompound(allocator: Allocator, desc: *const CompoundDesc, q: *const CompoundQueries, out: *CompoundOutput) !void {
    out.* = std.mem.zeroes(CompoundOutput);
    const log = &out.log;

    var result = try createCompound(allocator, desc, log);
    defer result.deinit();
    if (result.hasError()) {
        storeError(result.getError(), &out.state);
        out.log = std.mem.zeroes(CallLog);
        return;
    }
    const shape = result.getPtr().?;
    const compound = shape.cast(CompoundShape);
    storeState(compound, &out.state);
    try storeChildState(allocator, compound, &out.state);

    // Binary state and restore (the sub shapes are restored from saveSubShapeState)
    {
        const bytes = saveBytes(shape, &out.bytes);
        out.num_bytes = @intCast(bytes.len);

        const restored = try allocator.create(ParityCompoundShape);
        restored.* = .initDefault(allocator);
        var restored_ref = Ref(Shape).init(&restored.base.base);
        defer restored_ref.deinit();
        var reader: std.Io.Reader = .fixed(bytes[1..]); // Shape.restoreFromBinaryState reads the sub type
        var in = StreamInWrapper.init(&reader);
        try restored.base.base.restoreBinaryState(in.streamIn());
        var sub_shapes: ShapeList = .empty;
        defer {
            for (sub_shapes.items) |*s| s.deinit();
            sub_shapes.deinit(allocator);
        }
        try shape.saveSubShapeState(allocator, &sub_shapes);
        restored.base.base.restoreSubShapeState(sub_shapes.items);
        storeState(&restored.base, &out.restored);
        try storeChildState(allocator, &restored.base, &out.restored);
    }

    const transform = mat44(q.transform);
    const scale = vec3(q.scale);
    const position = vec3(q.position);
    const rotation = quat(q.rotation);

    out.world_bounds = boxArr(shape.getWorldSpaceBounds(transform, scale));
    for (q.test_scales, 0..) |s, i| {
        out.scale_valid[i] = @intFromBool(shape.isValidScale(vec3(s)));
        out.made_valid[i] = arr3(shape.makeScaleValid(vec3(s)));
    }

    for (q.raw_ids, 0..) |id, i|
        out.raw_id_valid[i] = @intFromBool(compound.isSubShapeIDValid(makeSubShapeID(id)));

    if (compound.getNumSubShapes() > 0) {
        for (q.ids, 0..) |raw_id, i| {
            const id = makeSubShapeID(raw_id);
            out.id_valid[i] = @intFromBool(compound.isSubShapeIDValid(id));
            const index = compound.getSubShapeIndexFromID(id);
            out.index[i] = index.index;
            out.remainder[i] = index.remainder.getValue();
            const leaf = shape.getLeafShape(id);
            out.leaf_child[i] = childIndex(leaf.shape);
            out.leaf_remainder[i] = leaf.remainder.getValue();
            const user_data = shape.getSubShapeUserData(id);
            out.user_data[i] = .{ @truncate(user_data), @truncate(user_data >> 32) };
            out.material_is_default[i] = @intFromBool(shape.getMaterial(id) == PhysicsMaterial.default);
            var sub = shape.getSubShapeTransformedShape(id, position, rotation, scale);
            defer sub.transformed_shape.deinit();
            out.sub_ts[i] = storeTS(&sub.transformed_shape);
            out.sub_ts_remainder[i] = sub.remainder.getValue();
            out.normal[i] = arr3(shape.getSurfaceNormal(id, vec3(q.local_position)));
            var face: Shape.SupportingFace = .empty;
            shape.getSupportingFace(id, vec3(q.direction), scale, transform, &face);
            out.face_count[i] = face.len;
            for (face.constSlice(), 0..) |v, j| {
                if (j >= 2) break;
                out.face[i][j] = arr3(v);
            }
        }
    }

    const submerged = shape.getSubmergedVolume(transform, scale, .init(vec3(q.surface[0..3].*), q.surface[3]));
    out.submerged = .{ submerged.total_volume, submerged.submerged_volume } ++ arr3(submerged.center_of_buoyancy);

    const vertices: CollideSoftBodyVertexIterator = .{};
    shape.collideSoftBodyVertices(transform, scale, &vertices, 7, 3);

    {
        var collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer collector.deinit();
        shape.transformShape(transform, &collector.base);
        try collector.checkError();
        out.num_transformed = @min(16, @as(u32, @intCast(collector.hits.items.len)));
        for (collector.hits.items[0..out.num_transformed], 0..) |*ts, i| out.transformed[i] = storeTS(ts);
    }

    // Queries through the visitors
    const creator = loadCreator(q.creator);
    const ray = RayCast.init(vec3(q.ray_origin), vec3(q.ray_direction));
    {
        var hit: RayCastResult = .{};
        hit.fraction = q.hit_fraction;
        out.ray_hit = @intFromBool(shape.castRay(ray, creator, &hit));
        out.ray_fraction = hit.fraction;
        out.ray_id = hit.sub_shape_id2.getValue();
    }
    {
        var settings: RayCastSettings = .{};
        if (q.back_faces != 0)
            settings.setBackFaceMode(.collide_with_back_faces);
        out.num_ray_hits = try collectHits(CastRayCollector, allocator, q.collector_kind, 32, struct {
            shape: *const Shape,
            ray: RayCast,
            settings: *const RayCastSettings,
            creator: SubShapeIDCreator,
            out: *CompoundOutput,
            fn query(self: @This(), collector: *CastRayCollector) void {
                self.shape.castRayCollector(self.ray, self.settings, self.creator, collector, &.{});
            }
            fn store(self: @This(), hit: *const RayCastResult, index: u32) void {
                self.out.ray_hit_fractions[index] = hit.fraction;
                self.out.ray_hit_ids[index] = hit.sub_shape_id2.getValue();
            }
        }{ .shape = shape, .ray = ray, .settings = &settings, .creator = creator, .out = out });
    }
    out.num_point_hits = try collectHits(CollidePointCollector, allocator, q.collector_kind, 16, struct {
        shape: *const Shape,
        point: Vec3,
        creator: SubShapeIDCreator,
        out: *CompoundOutput,
        fn query(self: @This(), collector: *CollidePointCollector) void {
            self.shape.collidePoint(self.point, self.creator, collector, &.{});
        }
        fn store(self: @This(), hit: *const CollidePointResult, index: u32) void {
            self.out.point_hit_ids[index] = hit.sub_shape_id2.getValue();
        }
    }{ .shape = shape, .point = vec3(q.point), .creator = creator, .out = out });
    {
        var collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
        defer collector.deinit();
        shape.collectTransformedShapes(aabox(q.box), position, rotation, scale, creator, &collector.base, &.{});
        try collector.checkError();
        out.num_collected = @min(16, @as(u32, @intCast(collector.hits.items.len)));
        for (collector.hits.items[0..out.num_collected], 0..) |*ts, i| out.collected[i] = storeTS(ts);
    }
    out.num_intersecting[0] = compound.getIntersectingSubShapes(aabox(q.box), out.intersecting[0][0..q.max_indices]);
    out.num_intersecting[1] = compound.getIntersectingSubShapesOrientedBox(orientedBox(q.oriented_box), out.intersecting[1][0..q.max_indices]);
}

/// jolt_composite_test_bounds
fn zoltTestBounds(allocator: Allocator, input: *const BoundsInput, out: *BoundsOutput) !void {
    out.* = std.mem.zeroes(BoundsOutput);

    var log = std.mem.zeroes(CallLog);
    var desc = std.mem.zeroes(CompoundDesc);
    desc.num_children = 1;
    desc.children[0] = .{ .half_extent = .{ 1, 1, 1 }, .center_of_mass = .{ 0, 0, 0 }, .density = 1.0, .sub_shape_id_bits = 0, .uniform_scale = 0, .must_be_static = 0, .user_data = 0 };
    desc.num_sub_shapes = input.num_sub_shapes;
    for (desc.sub_shapes[0..desc.num_sub_shapes]) |*s| s.rotation[3] = 1.0;
    var result = try createCompound(allocator, &desc, &log);
    defer result.deinit();
    const c = result.getPtr().?.cast(CompoundShape);

    const b = input.bounds;
    const min_x = vec4(b[0..4].*);
    const min_y = vec4(b[4..8].*);
    const min_z = vec4(b[8..12].*);
    const max_x = vec4(b[12..16].*);
    const max_y = vec4(b[16..20].*);
    const max_z = vec4(b[20..24].*);
    const filter: ShapeFilter = .{};

    const ray = RayCast.init(vec3(input.ray_origin), vec3(input.ray_direction));
    var hit: RayCastResult = .{};
    const ray_visitor = CompoundShape.CastRayVisitor.init(&ray, c, .{}, &hit);
    out.ray = arr4(ray_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z));
    out.sub_shape_bits = ray_visitor.sub_shape_bits;

    var ray_collector = AllHitCollisionCollector(CastRayCollector).init(allocator);
    defer ray_collector.deinit();
    const ray_settings: RayCastSettings = .{};
    const ray_collector_visitor = CompoundShape.CastRayVisitorCollector.init(&ray, &ray_settings, c, .{}, &ray_collector.base, &filter);
    out.ray_collector = arr4(ray_collector_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z));

    var point_collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
    defer point_collector.deinit();
    const point_visitor = CompoundShape.CollidePointVisitor.init(vec3(input.point), c, .{}, &point_collector.base, &filter);
    out.point = point_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).value;

    var other = CompositeChild.init(allocator, .{ .half_extent = input.other_half_extent, .center_of_mass = .{ 0, 0, 0 }, .density = 1.0, .sub_shape_id_bits = 0, .uniform_scale = 0, .must_be_static = 0, .user_data = 0 }, 99, &log);
    other.base.setEmbedded();
    defer other.base.deinit();
    const shape_cast = ShapeCast.initWithBounds(other.asShape(), Vec3.one(), Mat44.identity(), vec3(input.cast_direction), aabox(input.cast_bounds));
    var cast_settings: ShapeCastSettings = .{};
    cast_settings.extra_convex_radius = input.extra_convex_radius;
    var cast_collector = AllHitCollisionCollector(CastShapeCollector).init(allocator);
    defer cast_collector.deinit();
    const cast_visitor = CompoundShape.CastShapeVisitor.init(&shape_cast, &cast_settings, c, vec3(input.scale), &filter, Mat44.identity(), .{}, .{}, &cast_collector.base);
    out.cast = arr4(cast_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z));
    out.box_center = arr3(cast_visitor.box_center);
    out.box_extent = arr3(cast_visitor.box_extent);

    var ts_collector = AllHitCollisionCollector(TransformedShapeCollector).init(allocator);
    defer ts_collector.deinit();
    const collect_visitor = CompoundShape.CollectTransformedShapesVisitor.init(aabox(input.collect_box), c, vec3(input.position), quat(input.rotation), vec3(input.scale), .{}, &ts_collector.base, &filter);
    out.collect = collect_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).value;
    out.local_box = arr16(collect_visitor.local_box.orientation) ++ arr3(collect_visitor.local_box.half_extents);

    var collide_collector = AllHitCollisionCollector(CollideShapeCollector).init(allocator);
    defer collide_collector.deinit();
    var collide_settings: CollideShapeSettings = .{};
    collide_settings.max_separation_distance = input.max_separation;
    const transform1 = mat44(input.transform1);
    const transform2 = mat44(input.transform2);
    const vs_shape = CompoundShape.CollideCompoundVsShapeVisitor.init(c, other.asShape(), vec3(input.scale), vec3(input.scale2), transform1, transform2, .{}, .{}, &collide_settings, &collide_collector.base, &filter);
    out.compound_vs_shape = vs_shape.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).value;
    out.bounds_of2 = boxArr(vs_shape.bounds_of2_in_space_of1);
    const vs_compound = CompoundShape.CollideShapeVsCompoundVisitor.init(other.asShape(), c, vec3(input.scale2), vec3(input.scale), transform2, transform1, .{}, .{}, &collide_settings, &collide_collector.base, &filter);
    out.shape_vs_compound = vs_compound.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).value;
    out.bounds_of1 = boxArr(vs_compound.bounds_of1_in_space_of2);

    var indices: [4]u32 = undefined;
    const aabox_visitor = CompoundShape.GetIntersectingSubShapesVisitor(AABox).init(aabox(input.box), &indices);
    out.aabox = aabox_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).value;
    const obox_visitor = CompoundShape.GetIntersectingSubShapesVisitor(OrientedBox).init(orientedBox(input.oriented_box), &indices);
    out.oriented_box = obox_visitor.testBounds(min_x, min_y, min_z, max_x, max_y, max_z).value;
}

/// jolt_composite_sub_shape
fn zoltSubShape(allocator: Allocator, input: *const SubShapeInput, out: *SubShapeOutput) !void {
    out.* = std.mem.zeroes(SubShapeOutput);
    var log = std.mem.zeroes(CallLog);

    const child = try CompositeChild.create(allocator, .{ .half_extent = .{ 1, 1, 1 }, .center_of_mass = input.child_center_of_mass, .density = 1.0, .sub_shape_id_bits = 0, .uniform_scale = 0, .must_be_static = 0, .user_data = 0 }, 0, &log);
    var s: SubShape = .{ .shape = .init(&child.base) };
    defer s.deinit();
    s.setTransform(vec3(input.position), quat(input.rotation), vec3(input.compound_center_of_mass));
    out.stored_position = .{ s.position_com.x, s.position_com.y, s.position_com.z };
    out.stored_rotation = .{ s.rotation.x, s.rotation.y, s.rotation.z };
    out.is_rotation_identity = @intFromBool(s.is_rotation_identity);
    out.rotation = arr4(s.getRotation().getXYZW());
    out.position_com = arr3(s.getPositionCOM());
    for (input.scales, 0..) |scale_arr, i| {
        const scale = vec3(scale_arr);
        const valid = s.isValidScale(scale);
        out.valid[i] = @intFromBool(valid);
        out.transform_scale[i] = arr3(s.transformScale(scale));
        if (valid)
            out.local_transform[i] = arr16(s.getLocalTransformNoScale(scale));
    }
    s.setRotation(quat(input.rotation2));
    s.setPositionCOM(vec3(input.position2));
    out.round_trip_rotation = arr4(s.getRotation().getXYZW());
    out.round_trip_position = arr3(s.getPositionCOM());
    out.round_trip_stored = .{ s.position_com.x, s.position_com.y, s.position_com.z, s.rotation.x, s.rotation.y, s.rotation.z };
}

fn storeCollideHit(result: *const CollideShapeResult) HitOut {
    var hit = std.mem.zeroes(HitOut);
    hit.contact1 = arr3(result.contact_point_on1);
    hit.contact2 = arr3(result.contact_point_on2);
    hit.axis = arr3(result.penetration_axis);
    hit.value = result.penetration_depth;
    hit.ids = .{ result.sub_shape_id1.getValue(), result.sub_shape_id2.getValue() };
    hit.body_id = result.body_id2.getIndexAndSequenceNumber();
    hit.face_counts = .{ result.shape1_face.len, result.shape2_face.len };
    for (result.shape1_face.constSlice(), 0..) |v, i| {
        if (i < 2) hit.faces[0][i] = arr3(v);
    }
    for (result.shape2_face.constSlice(), 0..) |v, i| {
        if (i < 2) hit.faces[1][i] = arr3(v);
    }
    return hit;
}

fn storeCastHit(result: *const ShapeCastResult) HitOut {
    var hit = storeCollideHit(&result.base);
    hit.value = result.fraction;
    hit.back_face = @intFromBool(result.is_back_face_hit);
    return hit;
}

/// jolt_composite_visitors
fn zoltVisitors(allocator: Allocator, input: *const VisitorInput, out: *VisitorOutput) !void {
    out.* = std.mem.zeroes(VisitorOutput);

    var record: parity_user_types.Record = .{};
    record.hit_values = input.hit_values;

    const settings = try ParityCompoundShapeSettings.create(allocator);
    var settings_ref = Ref(ShapeSettings).init(&settings.base.base);
    defer settings_ref.deinit();
    for (0..input.num_sub_shapes) |i| {
        const c = input.children[i];
        const child = try allocator.create(ParityShape);
        child.* = .init(allocator, vec3(c.half_extent), vec3(c.center_of_mass), false, &record);
        settings.base.addShapePtr(vec3(input.positions[i]), quat(input.rotations[i]), &child.base, .{}) catch |err| {
            child.base.destroy();
            return err;
        };
    }
    var result = try settings.base.base.createShape(allocator);
    defer result.deinit();
    const compound = result.getPtr().?;
    const other_shape = try allocator.create(ParityShape);
    other_shape.* = .init(allocator, vec3(input.other.half_extent), vec3(input.other.center_of_mass), false, &record);
    var other_ref = RefConst(Shape).init(&other_shape.base);
    defer other_ref.deinit();
    const other = &other_shape.base;

    const Context = struct {
        compound: *const Shape,
        other: *const Shape,
        transform1: Mat44,
        transform2: Mat44,
        scale1: Vec3,
        scale2: Vec3,
        creator1: SubShapeIDCreator,
        creator2: SubShapeIDCreator,
        collide_settings: *const CollideShapeSettings,
        cast_settings: *const ShapeCastSettings,
        other_cast: *const ShapeCast,
        compound_cast: *const ShapeCast,
        hits: *[16]HitOut,
    };
    var collide_settings: CollideShapeSettings = .{};
    collide_settings.max_separation_distance = input.max_separation;
    const cast_settings: ShapeCastSettings = .{};
    const other_cast = ShapeCast.init(other, vec3(input.scale2), mat44(input.cast_start), vec3(input.cast_direction));
    const compound_cast = ShapeCast.init(compound, vec3(input.scale1), mat44(input.cast_start), vec3(input.cast_direction));
    var context: Context = .{
        .compound = compound,
        .other = other,
        .transform1 = mat44(input.transform1),
        .transform2 = mat44(input.transform2),
        .scale1 = vec3(input.scale1),
        .scale2 = vec3(input.scale2),
        .creator1 = loadCreator(input.creators[0]),
        .creator2 = loadCreator(input.creators[1]),
        .collide_settings = &collide_settings,
        .cast_settings = &cast_settings,
        .other_cast = &other_cast,
        .compound_cast = &compound_cast,
        .hits = &out.hits[0],
    };

    out.num_hits[0] = try collectHits(CollideShapeCollector, allocator, input.collector_kind, 16, struct {
        c: *const Context,
        fn query(self: @This(), collector: *CollideShapeCollector) void {
            const c = self.c;
            ParityCompoundShape.collideCompoundVsShape(c.compound, c.other, c.scale1, c.scale2, c.transform1, c.transform2, c.creator1, c.creator2, c.collide_settings, collector, &.{});
        }
        fn store(self: @This(), hit: *const CollideShapeResult, index: u32) void {
            self.c.hits[index] = storeCollideHit(hit);
        }
    }{ .c = &context });
    out.collide[0] = record.collide;
    record.collide = std.mem.zeroes(CollideRecord);

    context.hits = &out.hits[1];
    out.num_hits[1] = try collectHits(CollideShapeCollector, allocator, input.collector_kind, 16, struct {
        c: *const Context,
        fn query(self: @This(), collector: *CollideShapeCollector) void {
            const c = self.c;
            ParityCompoundShape.collideShapeVsCompound(c.other, c.compound, c.scale2, c.scale1, c.transform2, c.transform1, c.creator2, c.creator1, c.collide_settings, collector, &.{});
        }
        fn store(self: @This(), hit: *const CollideShapeResult, index: u32) void {
            self.c.hits[index] = storeCollideHit(hit);
        }
    }{ .c = &context });
    out.collide[1] = record.collide;

    context.hits = &out.hits[2];
    out.num_hits[2] = try collectHits(CastShapeCollector, allocator, input.collector_kind, 16, struct {
        c: *const Context,
        fn query(self: @This(), collector: *CastShapeCollector) void {
            const c = self.c;
            ParityCompoundShape.castShapeVsCompound(c.other_cast, c.cast_settings, c.compound, c.scale1, &.{}, c.transform1, c.creator2, c.creator1, collector);
        }
        fn store(self: @This(), hit: *const ShapeCastResult, index: u32) void {
            self.c.hits[index] = storeCastHit(hit);
        }
    }{ .c = &context });
    out.cast[0] = record.cast;
    record.cast = std.mem.zeroes(CastRecord);

    context.hits = &out.hits[3];
    out.num_hits[3] = try collectHits(CastShapeCollector, allocator, input.collector_kind, 16, struct {
        c: *const Context,
        fn query(self: @This(), collector: *CastShapeCollector) void {
            const c = self.c;
            CompoundShape.castCompoundVsShape(c.compound_cast, c.cast_settings, c.other, c.scale2, &.{}, c.transform2, c.creator1, c.creator2, collector);
        }
        fn store(self: @This(), hit: *const ShapeCastResult, index: u32) void {
            self.c.hits[index] = storeCastHit(hit);
        }
    }{ .c = &context });
    out.cast[1] = record.cast;
}

/// jolt_composite_decorated
fn zoltDecorated(allocator: Allocator, input: *const DecoratedInput, out: *DecoratedOutput) !void {
    out.* = std.mem.zeroes(DecoratedOutput);
    var log = std.mem.zeroes(CallLog);

    const child = try CompositeChild.create(allocator, input.child, 0, &log);
    var child_ref = RefConst(Shape).init(&child.base);
    defer child_ref.deinit();
    var bad_desc = input.child;
    bad_desc.half_extent[1] = -1.0;
    const child_settings = try CompositeChildSettings.create(allocator, if (input.mode == 3) bad_desc else input.child, 1, &log);
    var child_settings_ref = Ref(ShapeSettings).init(&child_settings.base);
    defer child_settings_ref.deinit();

    const settings = try allocator.create(ParityDecoratedShapeSettings);
    settings.* = .{ .base = switch (input.mode) {
        0 => .initPtr(ParityDecoratedShapeSettings, allocator, &child.base),
        1, 3 => .init(ParityDecoratedShapeSettings, allocator, &child_settings.base),
        else => .initDefault(ParityDecoratedShapeSettings, allocator),
    } };
    var settings_ref = Ref(ShapeSettings).init(settings.base.asShapeSettings());
    defer settings_ref.deinit();
    settings.base.base.user_data = input.user_data;
    var result = try settings.base.base.createShape(allocator);
    defer result.deinit();
    if (result.hasError()) {
        const message = result.getError();
        const n = @min(message.len, out.@"error".len - 1);
        @memcpy(out.@"error"[0..n], message[0..n]);
        return;
    }
    out.valid = 1;
    const shape = result.getPtr().?;

    out.must_be_static = @intFromBool(shape.mustBeStatic());
    out.center_of_mass = arr3(shape.getCenterOfMass());
    out.sub_shape_id_bits = shape.getSubShapeIDBitsRecursive();
    const id = makeSubShapeID(input.id);
    const leaf = shape.getLeafShape(id);
    out.leaf_child = childIndex(leaf.shape);
    out.leaf_remainder = leaf.remainder.getValue();
    out.material_is_default = @intFromBool(shape.getMaterial(id) == PhysicsMaterial.default);
    const user_data = shape.getSubShapeUserData(id);
    out.user_data = .{ @truncate(user_data), @truncate(user_data >> 32) };
    var face: Shape.SupportingFace = .empty;
    shape.getSupportingFace(id, vec3(input.direction), vec3(input.scale), mat44(input.transform), &face);
    out.face_count = face.len;
    for (face.constSlice(), 0..) |v, j| {
        if (j < 2) out.face[j] = arr3(v);
    }
    for (input.test_scales, 0..) |s, i| {
        out.scale_valid[i] = @intFromBool(shape.isValidScale(vec3(s)));
        out.made_valid[i] = arr3(shape.makeScaleValid(vec3(s)));
    }
    var visited: Shape.VisitedShapes = .empty;
    defer visited.deinit(allocator);
    out.num_triangles = (try shape.getStatsRecursive(allocator, &visited)).num_triangles;
    var sub_shapes: ShapeList = .empty;
    defer {
        for (sub_shapes.items) |*s| s.deinit();
        sub_shapes.deinit(allocator);
    }
    try shape.saveSubShapeState(allocator, &sub_shapes);
    out.num_sub_shapes = @intCast(sub_shapes.items.len);
    out.sub_shape_child = if (sub_shapes.items.len == 0) 0xffffffff else childIndex(sub_shapes.items[0].get());

    const bytes = saveBytes(shape, &out.bytes);
    out.num_bytes = @intCast(bytes.len);

    out.num_calls = log.count;
    for (0..@min(log.count, 4)) |i| out.calls[i] = log.calls[i];
}

// ---------------------------------------------------------------------------------------------------------------------
// Input generation

/// Values that tend to expose differences, mixed into the random inputs
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 1.0e-6, -1.0e-6, 100.0, -100.0 };

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

    fn grid(self: *Gen, n: i32) f32 {
        return @floatFromInt(self.rng.intRange(i32, -n, n));
    }

    /// A random unit quaternion: often identity (or close to it, or -identity), sometimes a multiple of 90 degrees
    /// around an axis
    fn rotation(self: *Gen) [4]f32 {
        switch (self.index(8)) {
            0 => return .{ 0, 0, 0, 1 },
            1 => return .{ 0, 0, 0, -1 },
            2 => return arr4(Quat.rotation(Vec3.axisX(), self.plain(-1.0e-6, 1.0e-6)).getXYZW()),
            3, 4 => {
                const axes = [_]Vec3{ Vec3.axisX(), Vec3.axisY(), Vec3.axisZ() };
                const axis = axes[self.index(3)];
                const angle = @as(f32, @floatFromInt(self.index(4))) * 0.5 * math.pi;
                return arr4(Quat.rotation(axis, angle).getXYZW());
            },
            else => while (true) {
                const q = self.rng.floatArray(4, -1, 1);
                const len_sq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
                if (len_sq > 1.0e-2 and len_sq <= 1.0) return arr4(quat(q).normalized().getXYZW());
            },
        }
    }

    /// A scale: random, uniform (also negative), uniform in XZ, near one, with tiny / zero / negative components
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

    /// A non zero uniform scale (valid for every compound)
    fn uniformScale(self: *Gen) P {
        const s = self.plain(0.2, 3) * (if (self.oneIn(3)) @as(f32, -1.0) else 1.0);
        return .{ s, s, s };
    }

    /// A rotation + translation matrix
    fn rotationTranslation(self: *Gen) [16]f32 {
        var m = Mat44.rotationQuat(quat(self.rotation()));
        m.setTranslation(vec3(self.vec(-10, 10)));
        return arr16(m);
    }

    /// A rotation + translation matrix, sometimes with a scale in the 3x3 part
    fn transform(self: *Gen) [16]f32 {
        var m = Mat44.rotationQuat(quat(self.rotation()));
        if (self.oneIn(3)) m = m.mul(Mat44.scaleVec3(vec3(self.plainVec(0.2, 3))));
        m.setTranslation(vec3(self.vec(-10, 10)));
        return arr16(m);
    }

    fn box(self: *Gen, range: f32) [6]f32 {
        const a = self.vec(-range, range);
        const b = self.vec(-range, range);
        return .{ @min(a[0], b[0]), @min(a[1], b[1]), @min(a[2], b[2]), @max(a[0], b[0]), @max(a[1], b[1]), @max(a[2], b[2]) };
    }

    fn orientedBox(self: *Gen) [19]f32 {
        return self.rotationTranslation() ++ self.plainVec(0.1, 5);
    }

    fn childDesc(self: *Gen) ChildDesc {
        return .{
            .half_extent = self.plainVec(0.1, 2),
            .center_of_mass = if (self.oneIn(3)) .{ 0, 0, 0 } else self.vec(-1, 1),
            .density = if (self.oneIn(10)) 0.0 else self.plain(0.5, 2000),
            .sub_shape_id_bits = if (self.oneIn(20)) 30 else @intCast(self.index(4)),
            .uniform_scale = @intFromBool(self.oneIn(3)),
            .must_be_static = @intFromBool(self.oneIn(5)),
            .user_data = self.next(),
        };
    }

    /// A compound description: 0..16 sub shapes (mostly 1..12) of up to 4 children, some created from settings
    fn compoundDesc(self: *Gen) CompoundDesc {
        var desc = std.mem.zeroes(CompoundDesc);
        desc.num_children = @intCast(1 + self.index(4));
        for (desc.children[0..desc.num_children]) |*c| c.* = self.childDesc();
        desc.num_sub_shapes = if (self.oneIn(20)) 0 else if (self.oneIn(10)) @intCast(13 + self.index(4)) else @intCast(1 + self.index(12));
        for (desc.sub_shapes[0..desc.num_sub_shapes]) |*s| {
            s.* = .{
                .position = self.vec(-5, 5),
                .rotation = self.rotation(),
                .child = @intCast(self.index(desc.num_children)),
                .user_data = self.next(),
                .from_settings = @intFromBool(self.oneIn(4)),
            };
        }
        return desc;
    }

    /// A sub shape ID of a compound with `num_sub_shapes` sub shapes: a valid index and random remainder bits
    fn subShapeID(self: *Gen, num_sub_shapes: u32) u32 {
        const n: u32 = num_sub_shapes -% 1;
        const bits: u32 = 32 - @clz(n);
        const idx: u32 = @intCast(self.index(num_sub_shapes));
        const creator = SubShapeIDCreator.pushID(.{}, idx, bits);
        const remainder_bits: u32 = @min(32 - bits, @as(u32, @intCast(self.index(9))));
        return if (remainder_bits == 0) creator.getID().getValue() else creator.pushID(@intCast(self.next() & ((@as(u32, 1) << @intCast(remainder_bits)) - 1)), remainder_bits).getID().getValue();
    }

    fn randomCreator(self: *Gen) [2]u32 {
        const bits: u32 = @intCast(self.index(9));
        const id: u32 = if (bits == 0) 0 else self.next() & ((@as(u32, 1) << @intCast(bits)) - 1);
        return .{ id, bits };
    }
};

/// A scale that IsValidScale accepts for every sub shape of the compound (so that the queries don't hit Jolt's asserts):
/// the generated scale when it is valid, otherwise a uniform scale
fn validScale(gen: *Gen, desc: *const CompoundDesc) P {
    const s = gen.scale();
    for (desc.sub_shapes[0..desc.num_sub_shapes]) |*sub| {
        const rotation = quat(sub.rotation);
        const identity = rotation.isClose(Quat.identity(), .{}) or rotation.isClose(Quat.identity().negate(), .{});
        var stored: Float3 = undefined; // The rotation as SubShape stores it
        rotation.storeFloat3(&stored);
        if (!identity and !ScaleHelpers.isUniformScale(vec3(s)) and !ScaleHelpers.canScaleBeRotated(Quat.loadFloat3Unsafe(&stored), vec3(s)))
            return gen.uniformScale();
    }
    return s;
}

/// Counts how often the inputs reached each case, so that a test cannot pass by comparing only trivial results
fn Coverage(comptime names: []const []const u8) type {
    return struct {
        counts: [names.len]usize = @splat(0),

        fn hit(self: *@This(), comptime name: []const u8, condition: bool) void {
            inline for (names, 0..) |n, i| {
                if (comptime std.mem.eql(u8, n, name)) {
                    if (condition) self.counts[i] += 1;
                    return;
                }
            }
            @compileError("unknown coverage case " ++ name);
        }

        fn expectAll(self: *const @This()) !void {
            var missing = false;
            for (names, self.counts) |n, c| {
                if (c == 0) {
                    std.debug.print("coverage: no input reached '{s}'\n", .{n});
                    missing = true;
                }
            }
            if (missing) return error.TestCoverage;
        }
    };
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

test "Composite parity: CompoundShape::SubShape (SetTransform, compressed position / rotation, scales)" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "SubShape" };
    var coverage: Coverage(&.{ "identity", "rotated", "valid scale", "invalid scale" }) = .{};
    for (0..fw.iterations / 2) |_| {
        var input: SubShapeInput = .{
            .child_center_of_mass = if (gen.oneIn(4)) .{ 0, 0, 0 } else gen.vec(-2, 2),
            .position = gen.vec(-10, 10),
            .rotation = gen.rotation(),
            .compound_center_of_mass = gen.vec(-3, 3),
            .scales = .{ gen.scale(), gen.scale(), gen.uniformScale(), gen.scale() },
            .rotation2 = gen.rotation(),
            .position2 = gen.vec(-10, 10),
        };
        if (gen.oneIn(5)) input.scales[3] = .{ gen.grid(3), gen.grid(3), gen.grid(3) };

        var jolt_out: SubShapeOutput = undefined;
        jolt.jolt_composite_sub_shape(&input, &jolt_out);
        var zolt_out: SubShapeOutput = undefined;
        try zoltSubShape(allocator, &input, &zolt_out);
        checker.check(input, zolt_out, jolt_out);
        coverage.hit("identity", zolt_out.is_rotation_identity != 0);
        coverage.hit("rotated", zolt_out.is_rotation_identity == 0);
        coverage.hit("valid scale", std.mem.indexOfScalar(u32, &zolt_out.valid, 1) != null);
        coverage.hit("invalid scale", std.mem.indexOfScalar(u32, &zolt_out.valid, 0) != null);
    }
    try checker.finish();
    try coverage.expectAll();
}

test "Composite parity: compound construction and the overrides of CompoundShape, binary state, the queries through the visitors" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var state_checker: Checker = .{ .name = "compound state" };
    var query_checker: Checker = .{ .name = "compound queries" };
    var visitor_checker: Checker = .{ .name = "compound visitor queries" };
    var log_checker: Checker = .{ .name = "compound call log" };
    var coverage: Coverage(&.{ "valid", "empty", "error", "more than 10", "mass", "must be static", "from settings", "identity rotation", "invalid test scale", "invalid raw id", "valid raw id", "submerged", "ray hit", "ray miss", "ray hits", "point hits", "collected some", "intersecting", "intersecting oriented", "log" }) = .{};
    const jolt_out = try allocator.create(CompoundOutput);
    defer allocator.destroy(jolt_out);
    const zolt_out = try allocator.create(CompoundOutput);
    defer allocator.destroy(zolt_out);

    for (0..iterations) |_| {
        const desc = gen.compoundDesc();
        const scale = validScale(&gen, &desc);
        var queries: CompoundQueries = .{
            .transform = gen.rotationTranslation(),
            .scale = scale,
            .test_scales = .{ gen.scale(), gen.scale(), gen.uniformScale(), .{ gen.grid(2), gen.grid(2), gen.grid(2) } },
            .ids = undefined,
            .raw_ids = .{ gen.next(), gen.next() & 0xff, gen.next() & 0xf, 0xffffffff },
            .position = gen.vec(-10, 10),
            .rotation = gen.rotation(),
            .local_position = gen.vec(-5, 5),
            .direction = gen.vec(-1, 1),
            .surface = arr3(Vec3.init(gen.plain(-1, 1), gen.plain(0.1, 1), gen.plain(-1, 1)).normalized()) ++ [1]f32{gen.float(-5, 5)},
            .ray_origin = gen.vec(-10, 10),
            .ray_direction = gen.vec(-20, 20),
            .creator = gen.randomCreator(),
            .hit_fraction = if (gen.oneIn(3)) 1.0 + math.flt_epsilon else gen.plain(0, 1),
            .back_faces = @intFromBool(gen.oneIn(2)),
            .collector_kind = @intCast(gen.index(3)),
            .point = gen.vec(-6, 6),
            .box = gen.box(8),
            .oriented_box = gen.orientedBox(),
            .max_indices = @intCast(gen.index(17)),
        };
        if (gen.oneIn(4)) queries.ray_origin = .{ 0, 0, 0 };
        if (desc.num_sub_shapes > 0) {
            for (&queries.ids) |*id| id.* = gen.subShapeID(desc.num_sub_shapes);
        } else queries.ids = .{ 0, 0, 0, 0 };

        jolt.jolt_composite_compound(&desc, &queries, jolt_out);
        try zoltCompound(allocator, &desc, &queries, zolt_out);

        const input = .{ desc, queries };
        state_checker.check(input, .{ zolt_out.state, zolt_out.restored, zolt_out.num_bytes, zolt_out.bytes }, .{ jolt_out.state, jolt_out.restored, jolt_out.num_bytes, jolt_out.bytes });
        query_checker.check(input, .{ zolt_out.world_bounds, zolt_out.scale_valid, zolt_out.made_valid, zolt_out.raw_id_valid, zolt_out.id_valid, zolt_out.index, zolt_out.remainder, zolt_out.leaf_child, zolt_out.leaf_remainder, zolt_out.user_data, zolt_out.material_is_default, zolt_out.sub_ts, zolt_out.sub_ts_remainder, zolt_out.normal, zolt_out.face_count, zolt_out.face, zolt_out.submerged, zolt_out.num_transformed, zolt_out.transformed }, .{ jolt_out.world_bounds, jolt_out.scale_valid, jolt_out.made_valid, jolt_out.raw_id_valid, jolt_out.id_valid, jolt_out.index, jolt_out.remainder, jolt_out.leaf_child, jolt_out.leaf_remainder, jolt_out.user_data, jolt_out.material_is_default, jolt_out.sub_ts, jolt_out.sub_ts_remainder, jolt_out.normal, jolt_out.face_count, jolt_out.face, jolt_out.submerged, jolt_out.num_transformed, jolt_out.transformed });
        visitor_checker.check(input, .{ zolt_out.ray_hit, zolt_out.ray_fraction, zolt_out.ray_id, zolt_out.num_ray_hits, zolt_out.ray_hit_fractions, zolt_out.ray_hit_ids, zolt_out.num_point_hits, zolt_out.point_hit_ids, zolt_out.num_collected, zolt_out.collected, zolt_out.num_intersecting, zolt_out.intersecting }, .{ jolt_out.ray_hit, jolt_out.ray_fraction, jolt_out.ray_id, jolt_out.num_ray_hits, jolt_out.ray_hit_fractions, jolt_out.ray_hit_ids, jolt_out.num_point_hits, jolt_out.point_hit_ids, jolt_out.num_collected, jolt_out.collected, jolt_out.num_intersecting, jolt_out.intersecting });
        log_checker.check(input, zolt_out.log, jolt_out.log);

        const z = zolt_out;
        coverage.hit("valid", z.state.valid != 0 and z.state.num_sub_shapes > 0);
        coverage.hit("empty", z.state.valid != 0 and z.state.num_sub_shapes == 0);
        coverage.hit("error", z.state.valid == 0);
        coverage.hit("more than 10", z.state.num_sub_shapes > 10);
        coverage.hit("mass", z.state.mass > 0);
        coverage.hit("must be static", z.state.must_be_static != 0);
        coverage.hit("from settings", z.state.sub_shapes[0].child >= 100);
        coverage.hit("identity rotation", z.state.sub_shapes[0].is_rotation_identity != 0);
        coverage.hit("invalid test scale", z.scale_valid[0] == 0 and z.state.valid != 0);
        coverage.hit("invalid raw id", z.raw_id_valid[0] == 0 and z.state.valid != 0);
        coverage.hit("valid raw id", z.raw_id_valid[1] != 0);
        coverage.hit("submerged", z.submerged[1] > 0 and z.submerged[1] < z.submerged[0]);
        coverage.hit("ray hit", z.ray_hit != 0);
        coverage.hit("ray miss", z.ray_hit == 0 and z.state.num_sub_shapes > 0);
        coverage.hit("ray hits", z.num_ray_hits > 1);
        coverage.hit("point hits", z.num_point_hits > 0);
        coverage.hit("collected some", z.num_collected > 0 and z.num_collected < z.state.num_sub_shapes);
        coverage.hit("intersecting", z.num_intersecting[0] > 0 and z.num_intersecting[0] < z.state.num_sub_shapes);
        coverage.hit("intersecting oriented", z.num_intersecting[1] > 0 and z.num_intersecting[1] < z.state.num_sub_shapes);
        coverage.hit("log", z.log.count > 20);
    }
    try finishAll(&.{ &state_checker, &query_checker, &visitor_checker, &log_checker });
    try coverage.expectAll();
}

test "Composite parity: TestBounds of every visitor" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "visitor TestBounds" };
    var coverage: Coverage(&.{ "ray enters", "point inside", "cast enters", "collect overlaps", "collect separate", "compound vs shape overlaps", "shape vs compound separate", "aabox overlaps", "oriented box overlaps", "oriented box separate" }) = .{};
    for (0..fw.iterations / 2) |_| {
        var bounds: [24]f32 = undefined;
        for (0..4) |lane| {
            const b = gen.box(5);
            for (0..6) |k| bounds[4 * k + lane] = b[k];
        }
        if (gen.oneIn(10)) bounds = gen.rng.floatArray(24, -5, 5); // Inverted boxes
        const input: BoundsInput = .{
            .bounds = bounds,
            .ray_origin = gen.vec(-10, 10),
            .ray_direction = gen.vec(-20, 20),
            .point = gen.vec(-5, 5),
            .cast_bounds = gen.box(5),
            .cast_direction = gen.vec(-20, 20),
            .extra_convex_radius = if (gen.oneIn(2)) 0.0 else gen.plain(0, 1),
            .scale = gen.scale(),
            .collect_box = gen.box(8),
            .position = gen.vec(-5, 5),
            .rotation = gen.rotation(),
            .other_half_extent = gen.plainVec(0.1, 3),
            .scale2 = gen.scale(),
            .transform1 = gen.rotationTranslation(),
            .transform2 = gen.transform(),
            .max_separation = if (gen.oneIn(2)) 0.0 else gen.plain(0, 2),
            .box = gen.box(5),
            .oriented_box = gen.orientedBox(),
            .num_sub_shapes = @intCast(1 + gen.index(16)),
        };

        var jolt_out: BoundsOutput = undefined;
        jolt.jolt_composite_test_bounds(&input, &jolt_out);
        var zolt_out: BoundsOutput = undefined;
        try zoltTestBounds(allocator, &input, &zolt_out);
        checker.check(input, zolt_out, jolt_out);
        coverage.hit("ray enters", zolt_out.ray[0] < 1.0);
        coverage.hit("point inside", zolt_out.point[0] != 0);
        coverage.hit("cast enters", zolt_out.cast[0] < 1.0);
        coverage.hit("collect overlaps", zolt_out.collect[0] != 0);
        coverage.hit("collect separate", zolt_out.collect[1] == 0);
        coverage.hit("compound vs shape overlaps", zolt_out.compound_vs_shape[0] != 0);
        coverage.hit("shape vs compound separate", zolt_out.shape_vs_compound[0] == 0);
        coverage.hit("aabox overlaps", zolt_out.aabox[0] != 0);
        coverage.hit("oriented box overlaps", zolt_out.oriented_box[0] != 0);
        coverage.hit("oriented box separate", zolt_out.oriented_box[0] == 0);
    }
    try checker.finish();
    try coverage.expectAll();
}

test "Composite parity: the visitors that dispatch (collide compound vs shape, shape vs compound, cast shape vs compound, CompoundShape::sCastCompoundVsShape)" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "dispatching visitors" };
    var coverage: Coverage(&.{ "compound vs shape", "shape vs compound", "cast shape vs compound", "cast compound vs shape", "compound vs shape rejected", "cast rejected by the bounds", "early out" }) = .{};
    const jolt_out = try allocator.create(VisitorOutput);
    defer allocator.destroy(jolt_out);
    const zolt_out = try allocator.create(VisitorOutput);
    defer allocator.destroy(zolt_out);

    for (0..iterations) |_| {
        var input = std.mem.zeroes(VisitorInput);
        input.num_sub_shapes = @intCast(1 + gen.index(16));
        var desc = std.mem.zeroes(CompoundDesc);
        desc.num_sub_shapes = input.num_sub_shapes;
        for (0..input.num_sub_shapes) |i| {
            input.children[i] = .{ .half_extent = gen.plainVec(0.1, 2), .center_of_mass = gen.vec(-1, 1) };
            input.positions[i] = gen.vec(-5, 5);
            input.rotations[i] = gen.rotation();
            desc.sub_shapes[i].rotation = input.rotations[i];
        }
        input.other = .{ .half_extent = gen.plainVec(0.1, 3), .center_of_mass = gen.vec(-1, 1) };
        input.hit_values = .{ gen.float(-1, 1), gen.float(-1, 1) };
        input.transform1 = gen.rotationTranslation();
        input.transform2 = gen.rotationTranslation();
        input.scale1 = validScale(&gen, &desc);
        input.scale2 = gen.scale();
        input.creators = .{ gen.randomCreator(), gen.randomCreator() };
        input.max_separation = if (gen.oneIn(2)) 0.0 else gen.plain(0, 20);
        input.collector_kind = @intCast(gen.index(3));
        input.cast_start = gen.rotationTranslation();
        input.cast_direction = gen.vec(-20, 20);

        jolt.jolt_composite_visitors(&input, jolt_out);
        try zoltVisitors(allocator, &input, zolt_out);
        checker.check(input, zolt_out.*, jolt_out.*);
        coverage.hit("compound vs shape", zolt_out.num_hits[0] > 1);
        coverage.hit("shape vs compound", zolt_out.num_hits[1] > 1);
        coverage.hit("cast shape vs compound", zolt_out.num_hits[2] > 1);
        coverage.hit("cast compound vs shape", zolt_out.num_hits[3] > 1);
        coverage.hit("compound vs shape rejected", zolt_out.num_hits[0] == 0 and zolt_out.collide[0].calls == 0);
        coverage.hit("cast rejected by the bounds", zolt_out.cast[0].calls < input.num_sub_shapes);
        coverage.hit("early out", input.collector_kind == any_hit and zolt_out.num_hits[3] == 1);
    }
    try checker.finish();
    try coverage.expectAll();
}

test "Composite parity: DecoratedShape (construction errors, the overrides, binary and sub shape state)" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "DecoratedShape" };
    var coverage: Coverage(&.{ "valid", "null inner shape", "child error", "invalid scale", "calls" }) = .{};
    for (0..iterations) |_| {
        const input: DecoratedInput = .{
            .child = gen.childDesc(),
            .mode = @intCast(gen.index(4)),
            .id = if (gen.oneIn(4)) 0xffffffff else gen.next(),
            .direction = gen.vec(-1, 1),
            .scale = gen.scale(),
            .transform = gen.transform(),
            .test_scales = .{ gen.scale(), gen.scale(), gen.uniformScale(), .{ gen.grid(2), gen.grid(2), gen.grid(2) } },
            .user_data = @as(u64, gen.next()) << 32 | gen.next(),
        };

        var jolt_out: DecoratedOutput = undefined;
        jolt.jolt_composite_decorated(&input, &jolt_out);
        var zolt_out: DecoratedOutput = undefined;
        try zoltDecorated(allocator, &input, &zolt_out);
        checker.check(input, zolt_out, jolt_out);
        coverage.hit("valid", zolt_out.valid != 0);
        coverage.hit("null inner shape", std.mem.startsWith(u8, &zolt_out.@"error", "Inner shape is null!"));
        coverage.hit("child error", std.mem.startsWith(u8, &zolt_out.@"error", "Invalid half extent"));
        coverage.hit("invalid scale", zolt_out.valid != 0 and zolt_out.scale_valid[0] == 0);
        coverage.hit("calls", zolt_out.num_calls == 2);
    }
    try checker.finish();
    try coverage.expectAll();
}
