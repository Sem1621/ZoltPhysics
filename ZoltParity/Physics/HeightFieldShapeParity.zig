//! Parity tests for HeightFieldShape (Phase 4, Wave B). The height fields are built from their settings on both sides
//! (`HFInput`: the settings and the sample / material index / material arrays, random height fields of every block
//! size, bits per sample, sample count (also not a multiple of the block size), offset and scale, with holes, flat and
//! extreme heights, materials and material capacities) and kept alive between the queries (an opaque handle on the
//! C++ side). Materials are indices into a table of PhysicsMaterialSimple that both sides build the same way.
//!
//! Compared bit for bit: the creation result (validity and Jolt's error texts, including every invalid setting), the
//! binary state bytes after creation (they contain all internal data: offset and scale after quantization, sample
//! count, block size, bits per sample, min / max sample, the compressed material indices, the range block hierarchy,
//! the quantized height samples and the active edges), Clone, sRestoreFromBinaryState + Save/RestoreMaterialState,
//! SaveWithChildren / sRestoreWithChildren, DetermineMinAndMaxSample, CalculateBitsPerSampleForError, GetLocalBounds,
//! GetWorldSpaceBounds (Mat44 and DMat44), the mass properties and the other simple getters, GetPosition /
//! IsNoCollision / GetMaterial(x, y) of every sample, ProjectOntoSurface, GetHeights and GetMaterials (with positive and
//! negative strides), the sub shape functions of every triangle (GetSurfaceNormal, GetMaterial,
//! GetSubShapeCoordinates, GetSupportingFace with scales, GetLeafShape, GetSubShapeUserData), CastRay (both overloads,
//! AllHit / AnyHit / ClosestHit collectors, back faces, early out fractions, shape filter), CollidePoint, collide
//! sphere / box vs height field and height field vs sphere / box through CollisionDispatch (active edges, back faces,
//! max separation distance, faces, early out, all hits in order), cast sphere / box vs height field and height field
//! vs sphere / box (world space), GetTrianglesStart / Next (boxes, transforms, inside out scales, buffer sizes),
//! CollideSoftBodyVertices, SetHeights (patches with holes, strides, active edge thresholds; then the state, the
//! heights and ray casts) and SetMaterials (new lists, the 256 material limit, null lists; then the state, the
//! material list and the material indices). GetSubmergedVolume is not compared: Jolt asserts and leaves its out
//! parameters untouched. C ABI wrappers: ZoltParity/Physics/HeightFieldShapeReference.cpp.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const Allocator = std.mem.Allocator;
const AABox = zolt.AABox;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const AnyHitCollisionCollector = zolt.AnyHitCollisionCollector;
const BoxShape = zolt.BoxShape;
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
const Float3 = zolt.Float3;
const HeightFieldShape = zolt.HeightFieldShape;
const HeightFieldShapeConstants = zolt.HeightFieldShapeConstants;
const HeightFieldShapeSettings = zolt.HeightFieldShapeSettings;
const Mat44 = zolt.Mat44;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsMaterialList = zolt.PhysicsMaterialList;
const PhysicsMaterialRefC = zolt.PhysicsMaterialRefC;
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
const ShapeCastResult = zolt.ShapeCastResult;
const ShapeCastSettings = zolt.ShapeCastSettings;
const ShapeFilter = zolt.ShapeFilter;
const SphereShape = zolt.SphereShape;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDCreator = zolt.SubShapeIDCreator;
const TempAllocatorMalloc = zolt.TempAllocatorMalloc;
const TransformedShape = zolt.TransformedShape;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const math = zolt.math;

const no_collision_value = HeightFieldShapeConstants.no_collision_value;

/// The C++ reference functions, see HeightFieldShapeReference.cpp
const jolt = struct {
    extern fn jolt_hf_create(desc: *const HFDesc, samples: [*]const f32, material_indices: [*]const u8, materials: [*]const u32, out_shape: *?*anyopaque, out_error: *[128]u8) c_int;
    extern fn jolt_hf_release(shape: *anyopaque) void;
    extern fn jolt_hf_settings_info(desc: *const HFDesc, samples: [*]const f32, max_error: f32, out_min_max_scale: *[3]f32) u32;
    extern fn jolt_hf_binary_state(shape: *const anyopaque, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_hf_clone_state(shape: *const anyopaque, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_hf_restore_state(bytes: [*]const u8, size: u32, materials: [*]const u32, num_materials: u32, out_bytes: [*]u8, capacity: u32, out_materials: [*]u32, out_num_materials: *u32) u32;
    extern fn jolt_hf_save_with_children(shape: *const anyopaque, out_bytes: [*]u8, capacity: u32, out_restored_bytes: [*]u8, out_restored_size: *u32) u32;
    extern fn jolt_hf_properties(shape: *const anyopaque, input: *const PropertiesInput, output: *PropertiesOutput) void;
    extern fn jolt_hf_samples(shape: *const anyopaque, out_positions: [*]f32, out_no_collision: [*]c_int, out_materials: [*]u32) void;
    extern fn jolt_hf_project(shape: *const anyopaque, point: *const P, out_position: *P, out_sub_shape_id: *u32) c_int;
    extern fn jolt_hf_get_heights(shape: *const anyopaque, x: u32, y: u32, size_x: u32, size_y: u32, base: [*]f32, first_row: i64, stride: i64) void;
    extern fn jolt_hf_get_materials(shape: *const anyopaque, x: u32, y: u32, size_x: u32, size_y: u32, base: [*]u8, first_row: i64, stride: i64) void;
    extern fn jolt_hf_set_heights(shape: *anyopaque, x: u32, y: u32, size_x: u32, size_y: u32, base: [*]const f32, first_row: i64, stride: i64, active_edge_cos_threshold_angle: f32) void;
    extern fn jolt_hf_set_materials(shape: *anyopaque, x: u32, y: u32, size_x: u32, size_y: u32, base: [*]const u8, first_row: i64, stride: i64, list: [*]const u32, list_count: c_int) c_int;
    extern fn jolt_hf_sub_shape(shape: *const anyopaque, input: *const SubShapeInput, output: *SubShapeOutput) void;
    extern fn jolt_hf_cast_ray(shape: *const anyopaque, input: *const RayInput, output: *RayOutput) void;
    extern fn jolt_hf_collide_point(shape: *const anyopaque, point: *const P, creator: *const [2]u32) u32;
    extern fn jolt_hf_collide(shape: *const anyopaque, input: *const CollideInput, output: *HitsOutput) void;
    extern fn jolt_hf_cast(shape: *const anyopaque, input: *const CastInput, output: *HitsOutput) void;
    extern fn jolt_hf_triangles(shape: *const anyopaque, box: *const [6]f32, position: *const P, rotation: *const [4]f32, scale: *const P, max_triangles_requested: c_int, with_materials: c_int, max_calls: c_int, max_triangles: c_int, out_counts: [*]c_int, out_vertices: [*]f32, out_materials: [*]u32) c_int;
    extern fn jolt_hf_soft_body(shape: *const anyopaque, transform: *const [16]f32, scale: *const P, num_vertices: c_int, positions: [*]const f32, inv_masses: [*]const f32, io_penetrations: [*]f32, io_planes: [*]f32, io_indices: [*]c_int, colliding_shape_index: c_int) void;
};

/// A point / vector as passed to the C ABI
const P = [3]f32;

/// Number of materials in the material table (must match kNumMaterials in HeightFieldShapeReference.cpp)
const num_table_materials = 300;
/// Material index of PhysicsMaterial::sDefault
const default_material: u32 = 0xfffffffe;
/// Material index of a material that is not in the table
const unknown_material: u32 = 0xffffffff;

/// Must match HFDesc in HeightFieldShapeReference.cpp
const HFDesc = extern struct {
    offset: P = .{ 0, 0, 0 },
    scale: P = .{ 1, 1, 1 },
    sample_count: u32 = 0,
    min_height_value: f32 = math.large_float,
    max_height_value: f32 = -math.large_float,
    materials_capacity: u32 = 0,
    block_size: u32 = 2,
    bits_per_sample: u32 = 8,
    active_edge_cos_threshold_angle: f32 = 0.996195,
    num_samples: u32 = 0,
    num_material_indices: u32 = 0,
    num_materials: u32 = 0,
    user_data: u64 = 0,
};

/// Must match PropertiesInput in HeightFieldShapeReference.cpp
const PropertiesInput = extern struct {
    scale: P,
    transform: [16]f32,
    translation: [3]f64,
};

/// Must match PropertiesOutput in HeightFieldShapeReference.cpp
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
    must_be_static: c_int,
    min_height_value: f32,
    max_height_value: f32,
    sample_count: u32,
    block_size: u32,
    num_materials: u32,
    materials: [260]u32,
};

/// Must match SubShapeInput in HeightFieldShapeReference.cpp
const SubShapeInput = extern struct {
    sub_shape_id: u32,
    point: P,
    direction: P,
    scale: P,
    transform: [16]f32,
};

/// Must match SubShapeOutput in HeightFieldShapeReference.cpp
const SubShapeOutput = extern struct {
    normal: P,
    material: u32,
    x: u32,
    y: u32,
    triangle: u32,
    face_count: u32,
    face: [32 * 3]f32,
    leaf_remainder: u32,
    leaf_is_self: c_int,
    user_data: u64,
};

/// Must match RayInput in HeightFieldShapeReference.cpp
const RayInput = extern struct {
    origin: P,
    direction: P,
    creator: [2]u32,
    fraction: f32,
    back_face_mode: c_int,
    collector: c_int,
    early_out: f32,
    body_id: u32,
    reject_id: u32,
};

const max_ray_hits = 64;

const RayHit = extern struct {
    fraction: f32,
    body_id: u32,
    sub_shape_id: u32,
};

/// Must match RayOutput in HeightFieldShapeReference.cpp
const RayOutput = extern struct {
    hit: c_int,
    fraction: f32,
    sub_shape_id: u32,
    num_hits: u32,
    hits: [max_ray_hits]RayHit,
};

/// Must match ConvexDesc in HeightFieldShapeReference.cpp
const ConvexDesc = extern struct {
    kind: u32,
    radius: f32 = 0.0,
    half_extent: P = .{ 0, 0, 0 },
    convex_radius: f32 = 0.0,
};

/// Must match CollideInput in HeightFieldShapeReference.cpp
const CollideInput = extern struct {
    convex: ConvexDesc,
    height_field_first: c_int,
    scale1: P,
    scale2: P,
    transform1: [16]f32,
    transform2: [16]f32,
    creator1: [2]u32,
    creator2: [2]u32,
    max_separation_distance: f32,
    collision_tolerance: f32,
    penetration_tolerance: f32,
    active_edge_mode: c_int,
    back_face_mode: c_int,
    collect_faces: c_int,
    active_edge_movement_direction: P,
    early_out: f32,
    body_id: u32,
    collector: c_int,
};

const max_hits = 24;

const HitOutput = extern struct {
    fraction: f32,
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

/// Must match HitsOutput in HeightFieldShapeReference.cpp
const HitsOutput = extern struct {
    num_hits: u32,
    hits: [max_hits]HitOutput,
};

/// Must match CastInput in HeightFieldShapeReference.cpp
const CastInput = extern struct {
    convex: ConvexDesc,
    height_field_cast: c_int,
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
    active_edge_mode: c_int,
    back_face_mode_triangles: c_int,
    back_face_mode_convex: c_int,
    use_shrunken_shape: c_int,
    return_deepest_point: c_int,
    collect_faces: c_int,
    active_edge_movement_direction: P,
    early_out: f32,
    body_id: u32,
    collector: c_int,
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

fn makeCreator(c: [2]u32) SubShapeIDCreator {
    return if (c[1] == 0) .{} else SubShapeIDCreator.pushID(.{}, c[0], c[1]);
}

fn storeFace(face: *const Shape.SupportingFace, out_count: *u32, out_face: *[32 * 3]f32) void {
    out_count.* = face.len;
    for (face.constSlice(), 0..) |v, i| out_face[3 * i ..][0..3].* = arr3(v);
}

// ---------------------------------------------------------------------------------------------------------------------
// The material table

const MaterialTable = struct {
    materials: [num_table_materials]PhysicsMaterialRefC,

    fn init(allocator: Allocator) !MaterialTable {
        var self: MaterialTable = undefined;
        for (&self.materials, 0..) |*m, i| {
            const name = try std.fmt.allocPrint(allocator, "Material {d}", .{i});
            defer allocator.free(name);
            const material = try PhysicsMaterialSimple.create(allocator, name, Color.getDistinctColor(@intCast(i)));
            m.* = .init(material.material());
        }
        return self;
    }

    fn deinit(self: *MaterialTable) void {
        for (&self.materials) |*m| m.deinit();
    }

    fn get(self: *const MaterialTable, index: u32) *const PhysicsMaterial {
        return self.materials[index].get().?;
    }

    fn indexOf(self: *const MaterialTable, material: ?*const PhysicsMaterial) u32 {
        if (material == PhysicsMaterial.default)
            return default_material;
        for (self.materials, 0..) |m, i|
            if (m.get() == material) return @intCast(i);
        return unknown_material;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Height field inputs

/// The settings of a height field and its arrays
const HFInput = struct {
    desc: HFDesc = .{},
    samples: std.ArrayList(f32) = .empty,
    material_indices: std.ArrayList(u8) = .empty,
    materials: std.ArrayList(u32) = .empty,

    fn deinit(self: *HFInput, allocator: Allocator) void {
        self.samples.deinit(allocator);
        self.material_indices.deinit(allocator);
        self.materials.deinit(allocator);
    }

    /// Update the array counts of the description
    fn finish(self: *HFInput) void {
        self.desc.num_samples = @intCast(self.samples.items.len);
        self.desc.num_material_indices = @intCast(self.material_indices.items.len);
        self.desc.num_materials = @intCast(self.materials.items.len);
    }
};

/// A height field on both sides
const Pair = struct {
    zolt: Ref(Shape),
    jolt: *anyopaque,

    fn deinit(self: *Pair) void {
        self.zolt.deinit();
        jolt.jolt_hf_release(self.jolt);
    }

    fn hf(self: *const Pair) *const HeightFieldShape {
        return self.zolt.get().?.cast(HeightFieldShape);
    }

    fn hfMut(self: *Pair) *HeightFieldShape {
        return self.zolt.get().?.castMut(HeightFieldShape);
    }

    fn shape(self: *const Pair) *const Shape {
        return self.zolt.get().?;
    }
};

/// Build the Zolt height field (null with the error text when the settings are invalid)
fn zoltCreate(allocator: Allocator, table: *const MaterialTable, input: *const HFInput, out_error: *[128]u8) !?Ref(Shape) {
    var settings = HeightFieldShapeSettings.initDefault(allocator);
    defer settings.deinit();
    const d = input.desc;
    settings.offset = vec3(d.offset);
    settings.scale = vec3(d.scale);
    settings.sample_count = d.sample_count;
    settings.min_height_value = d.min_height_value;
    settings.max_height_value = d.max_height_value;
    settings.materials_capacity = d.materials_capacity;
    settings.block_size = d.block_size;
    settings.bits_per_sample = d.bits_per_sample;
    settings.active_edge_cos_threshold_angle = d.active_edge_cos_threshold_angle;
    try settings.height_samples.appendSlice(allocator, input.samples.items);
    try settings.material_indices.appendSlice(allocator, input.material_indices.items);
    for (input.materials.items) |m| {
        try settings.materials.ensureUnusedCapacity(allocator, 1);
        settings.materials.appendAssumeCapacity(.init(table.get(m)));
    }
    settings.asShapeSettings().user_data = d.user_data;

    var result = try settings.asShapeSettings().createShape(allocator);
    defer result.deinit();
    @memset(out_error, 0);
    if (result.hasError()) {
        const text = result.getError();
        @memcpy(out_error[0..@min(text.len, 127)], text[0..@min(text.len, 127)]);
        return null;
    }
    return Ref(Shape).init(result.getPtr());
}

/// Build the height field on both sides and compare the creation results. Returns null when the settings are invalid.
fn build(allocator: Allocator, table: *const MaterialTable, input: *const HFInput, checker: *Checker) !?Pair {
    var zolt_error: [128]u8 = undefined;
    var zolt_shape = try zoltCreate(allocator, table, input, &zolt_error);
    errdefer if (zolt_shape) |*s| s.deinit();

    var jolt_error: [128]u8 = undefined;
    var jolt_shape: ?*anyopaque = null;
    const jolt_valid = jolt.jolt_hf_create(&input.desc, input.samples.items.ptr, input.material_indices.items.ptr, input.materials.items.ptr, &jolt_shape, &jolt_error);

    checker.check(input.desc, .{ @as(c_int, @intFromBool(zolt_shape != null)), zolt_error }, .{ jolt_valid, jolt_error });
    if (zolt_shape != null and jolt_shape != null)
        return .{ .zolt = zolt_shape.?, .jolt = jolt_shape.? };
    if (zolt_shape) |*s| s.deinit();
    if (jolt_shape) |s| jolt.jolt_hf_release(s);
    return null;
}

/// SaveBinaryState of a Zolt shape (owned by the caller)
fn zoltState(allocator: Allocator, shape: *const Shape) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(allocator);
    errdefer out.deinit();
    var wrapper = StreamOutWrapper.init(&out.writer);
    shape.saveBinaryState(wrapper.streamOut());
    return out.toOwnedSlice();
}

/// SaveBinaryState of a C++ shape (owned by the caller)
fn joltState(allocator: Allocator, shape: *const anyopaque) ![]u8 {
    const size = jolt.jolt_hf_binary_state(shape, undefined, 0);
    const bytes = try allocator.alloc(u8, size);
    _ = jolt.jolt_hf_binary_state(shape, bytes.ptr, size);
    return bytes;
}

/// Compare two byte arrays (the size and the bytes)
fn checkBytes(checker: *Checker, input: anytype, zolt_bytes: []const u8, jolt_bytes: []const u8) void {
    checker.check(input, zolt_bytes.len, jolt_bytes.len);
    if (zolt_bytes.len != jolt_bytes.len) return;
    if (!std.mem.eql(u8, zolt_bytes, jolt_bytes)) {
        const i = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes).?;
        checker.check(.{ input, i }, zolt_bytes[i], jolt_bytes[i]);
    }
}

/// Compare the binary state of both sides
fn checkState(allocator: Allocator, checker: *Checker, input: anytype, pair: *const Pair) !void {
    const zolt_bytes = try zoltState(allocator, pair.shape());
    defer allocator.free(zolt_bytes);
    const jolt_bytes = try joltState(allocator, pair.jolt);
    defer allocator.free(jolt_bytes);
    checkBytes(checker, input, zolt_bytes, jolt_bytes);
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

    fn range(self: *Gen, min: u32, max: u32) u32 {
        return self.rng.intRange(u32, min, max);
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
    fn transform(self: *Gen, range_value: f32) Mat44 {
        return Mat44.rotationTranslation(self.rotation(), vec3(self.vec(-range_value, range_value)));
    }

    fn creator(self: *Gen) [2]u32 {
        const bits: u32 = @intCast(self.index(9));
        return .{ if (bits == 0) 0 else self.next() & ((@as(u32, 1) << @intCast(bits)) - 1), bits };
    }

    /// A scale for a height field: anything non zero (negative components flip it)
    fn hfScale(self: *Gen) P {
        if (self.oneIn(4)) return .{ 1, 1, 1 };
        var s = self.plainVec(0.3, 2.0);
        for (&s) |*c|
            if (self.oneIn(6)) {
                c.* = -c.*;
            };
        return s;
    }

    /// A convex shape: sphere or box (with or without convex radius)
    fn convex(self: *Gen) ConvexDesc {
        if (self.oneIn(2))
            return .{ .kind = 0, .radius = if (self.oneIn(5)) 1.0 else self.plain(0.1, 2.5) };
        var half_extent = self.plainVec(0.1, 2.0);
        if (self.oneIn(5)) half_extent = .{ 1, 1, 1 };
        const convex_radius: f32 = switch (self.index(3)) {
            0 => 0.0,
            1 => 0.05,
            else => self.plain(0, 0.09),
        };
        return .{ .kind = 1, .half_extent = half_extent, .convex_radius = convex_radius };
    }

    /// A valid scale for the convex shape
    fn convexScale(self: *Gen, desc: ConvexDesc) P {
        if (self.oneIn(3)) return .{ 1, 1, 1 };
        const s = self.plain(0.5, 1.5);
        if (desc.kind == 0)
            return .{ if (self.oneIn(2)) s else -s, if (self.oneIn(2)) s else -s, if (self.oneIn(2)) s else -s };
        var r = self.plainVec(0.5, 1.5);
        for (&r) |*c|
            if (self.oneIn(3)) {
                c.* = -c.*;
            };
        return r;
    }

    /// A random valid height field (sample count, block size, bits, offset, scale, samples with holes, materials)
    fn heightField(self: *Gen, allocator: Allocator, opts: struct { max_blocks: u32 = 8, multiple_of_block_size: bool = false }) !HFInput {
        var input: HFInput = .{};
        errdefer input.deinit(allocator);
        var d = &input.desc;
        d.block_size = self.range(2, 8);
        const num_blocks = self.range(2, opts.max_blocks);
        d.sample_count = num_blocks * d.block_size;
        if (!opts.multiple_of_block_size and self.oneIn(3))
            d.sample_count -= self.range(0, d.block_size - 1); // Rounded up to a multiple of the block size by the shape
        d.bits_per_sample = if (self.oneIn(4)) 8 else self.range(1, 16);
        d.offset = if (self.oneIn(4)) .{ 0, 0, 0 } else self.plainVec(-5, 5);
        d.scale = if (self.oneIn(4)) .{ 1, 1, 1 } else .{ self.plain(0.2, 2), self.plain(0.05, 2), self.plain(0.2, 2) };
        if (self.oneIn(15)) d.scale[2 * self.index(2)] *= -1.0; // Mirrored height field (X or Z: a negative or zero Y scale inverts the range blocks, Jolt then walks blocks outside the height field)
        if (self.oneIn(4)) {
            d.min_height_value = self.plain(-20, 0);
            d.max_height_value = self.plain(0, 20);
        }
        if (self.oneIn(4)) d.active_edge_cos_threshold_angle = if (self.oneIn(3)) -1.0 else self.plain(0.5, 1.0);
        d.user_data = (@as(u64, self.next()) << 32) | self.next();

        // Samples
        const n: usize = @as(usize, d.sample_count) * d.sample_count;
        try input.samples.resize(allocator, n);
        const kind = self.index(6);
        const base = self.plain(-10, 10);
        const amplitude: f32 = switch (self.index(4)) {
            0 => 1.0e-4,
            1 => 1000.0,
            else => self.plain(0.1, 5),
        };
        const frequency = self.plain(0.1, 1.5);
        for (input.samples.items, 0..) |*h, i| {
            const x: f32 = @floatFromInt(i % d.sample_count);
            const y: f32 = @floatFromInt(i / d.sample_count);
            h.* = switch (kind) {
                0 => base, // Flat
                1 => base + amplitude * self.plain(-1, 1), // Noise
                2 => base + amplitude * (@abs(@mod(x * frequency, 2.0) - 1.0) + @abs(@mod(y * frequency * 0.7, 2.0) - 1.0)), // Ridges
                3 => base + amplitude * @floor(x * frequency * 0.5) - amplitude * @floor(y * frequency * 0.3), // Terraces
                4 => base + 0.05 * amplitude * (x * x - y * y), // Saddle
                else => if (self.oneIn(2)) base else base + amplitude, // Steps
            };
        }

        // Holes
        const hole_chance: u32 = switch (self.index(5)) {
            0 => 0,
            1 => 2,
            2 => 5,
            else => 20,
        };
        if (hole_chance != 0) {
            for (input.samples.items) |*h|
                if (self.oneIn(hole_chance)) {
                    h.* = no_collision_value;
                };
        }
        if (self.oneIn(40))
            @memset(input.samples.items, no_collision_value); // No collision at all

        // Materials
        switch (self.index(5)) {
            0, 1 => {},
            2 => try input.materials.append(allocator, @intCast(self.index(num_table_materials))),
            else => {
                const num_materials = if (self.oneIn(4)) self.range(2, 255) else self.range(2, 9);
                for (0..num_materials) |_| try input.materials.append(allocator, @intCast(self.index(num_table_materials)));
                const count_min_1: usize = d.sample_count - 1;
                try input.material_indices.resize(allocator, count_min_1 * count_min_1);
                for (input.material_indices.items) |*m| m.* = @intCast(self.index(num_materials));
            },
        }
        if (self.oneIn(4)) d.materials_capacity = self.range(0, 40);

        input.finish();
        return input;
    }
};

/// Hand picked height fields (edge cases)
fn edgeCaseHeightFields(allocator: Allocator, list: *std.ArrayList(HFInput)) !void {
    const Make = struct {
        fn make(a: Allocator, sample_count: u32, block_size: u32, bits: u32, value: anytype) !HFInput {
            var input: HFInput = .{};
            errdefer input.deinit(a);
            input.desc.sample_count = sample_count;
            input.desc.block_size = block_size;
            input.desc.bits_per_sample = bits;
            try input.samples.resize(a, @as(usize, sample_count) * sample_count);
            for (input.samples.items, 0..) |*h, i|
                h.* = value.at(i % sample_count, i / sample_count);
            input.finish();
            return input;
        }
    };
    const Flat = struct {
        v: f32,
        fn at(self: @This(), x: usize, y: usize) f32 {
            _ = .{ x, y };
            return self.v;
        }
    };
    const Checker2 = struct {
        fn at(self: @This(), x: usize, y: usize) f32 {
            _ = self;
            return if ((x + y) % 2 == 0) no_collision_value else @floatFromInt(x);
        }
    };
    const Slope = struct {
        s: f32,
        fn at(self: @This(), x: usize, y: usize) f32 {
            return self.s * @as(f32, @floatFromInt(x)) + 0.5 * self.s * @as(f32, @floatFromInt(y));
        }
    };
    const Huge = struct {
        fn at(self: @This(), x: usize, y: usize) f32 {
            _ = self;
            return if ((x * 7 + y * 3) % 5 == 0) 1.0e6 else -1.0e6 + @as(f32, @floatFromInt(x * y));
        }
    };

    // Flat plane encoded with 1 bit (TestPlane), with offset and scale
    var plane = try Make.make(allocator, 32, 4, 1, Flat{ .v = 1.0 });
    plane.desc.offset = .{ 3, 5, 7 };
    plane.desc.scale = .{ 9, 13, 17 };
    plane.samples.items[17] = no_collision_value;
    plane.samples.items[500] = no_collision_value;
    try list.append(allocator, plane);

    // Flat plane close to the origin (TestPlaneCloseToOrigin)
    try list.append(allocator, try Make.make(allocator, 32, 4, 1, Flat{ .v = 1.0e-6 }));

    // No collision at all (TestEmptyHeightField)
    try list.append(allocator, try Make.make(allocator, 32, 2, 8, Flat{ .v = no_collision_value }));

    // Checkerboard of holes: every quad misses a vertex
    try list.append(allocator, try Make.make(allocator, 16, 4, 6, Checker2{}));

    // Smallest valid height field (2 blocks)
    try list.append(allocator, try Make.make(allocator, 4, 2, 16, Slope{ .s = 0.7 }));
    try list.append(allocator, try Make.make(allocator, 16, 8, 3, Slope{ .s = -2.5 }));

    // Sample count that is not a multiple of the block size
    try list.append(allocator, try Make.make(allocator, 33, 4, 8, Slope{ .s = 0.25 }));
    try list.append(allocator, try Make.make(allocator, 13, 7, 5, Slope{ .s = 1.0 }));

    // A non power of 2 number of blocks and a deep hierarchy
    try list.append(allocator, try Make.make(allocator, 130, 2, 8, Slope{ .s = 0.1 }));
    try list.append(allocator, try Make.make(allocator, 20, 2, 8, Flat{ .v = 0.0 }));

    // Huge height differences
    try list.append(allocator, try Make.make(allocator, 24, 3, 16, Huge{}));

    // Artificial min / max height values
    var min_max = try Make.make(allocator, 16, 4, 8, Slope{ .s = 0.5 });
    min_max.desc.min_height_value = -5.0;
    min_max.desc.max_height_value = 10.0;
    try list.append(allocator, min_max);
}

/// Invalid settings (each must give Jolt's error text)
fn invalidHeightFields(allocator: Allocator, list: *std.ArrayList(HFInput)) !void {
    const Make = struct {
        fn make(a: Allocator, sample_count: u32, block_size: u32, bits: u32, with_samples: bool) !HFInput {
            var input: HFInput = .{};
            errdefer input.deinit(a);
            input.desc.sample_count = sample_count;
            input.desc.block_size = block_size;
            input.desc.bits_per_sample = bits;
            if (with_samples) {
                try input.samples.resize(a, @as(usize, sample_count) * sample_count);
                for (input.samples.items, 0..) |*h, i| h.* = @floatFromInt(i % 7);
            }
            input.finish();
            return input;
        }
    };

    // Block size
    try list.append(allocator, try Make.make(allocator, 16, 1, 8, true));
    try list.append(allocator, try Make.make(allocator, 18, 9, 8, true));
    try list.append(allocator, try Make.make(allocator, 16, 100, 8, true));

    // Bits per sample
    try list.append(allocator, try Make.make(allocator, 16, 2, 0, true));
    try list.append(allocator, try Make.make(allocator, 16, 2, 17, true));

    // Sample count too low (less than 2 blocks)
    try list.append(allocator, try Make.make(allocator, 4, 4, 8, true));
    try list.append(allocator, try Make.make(allocator, 2, 2, 8, true));
    try list.append(allocator, try Make.make(allocator, 0, 2, 8, true));

    // Sample count too high (more than 1 << 14 blocks, the samples are never read)
    try list.append(allocator, try Make.make(allocator, 2 * 16385, 2, 8, false));

    // Too many sub shape ID bits (the samples are never read)
    try list.append(allocator, try Make.make(allocator, 40000, 4, 8, false));

    // More than 256 materials
    var too_many = try Make.make(allocator, 8, 2, 8, true);
    for (0..257) |i| try too_many.materials.append(allocator, @intCast(i));
    too_many.finish();
    try list.append(allocator, too_many);

    // Material index beyond the material list
    var beyond = try Make.make(allocator, 8, 2, 8, true);
    for (0..3) |i| try beyond.materials.append(allocator, @intCast(i + 10));
    try beyond.material_indices.appendNTimes(allocator, 1, 49);
    beyond.material_indices.items[30] = 5;
    beyond.finish();
    try list.append(allocator, beyond);

    // Material indices without materials
    var no_materials = try Make.make(allocator, 8, 2, 8, true);
    try no_materials.material_indices.appendNTimes(allocator, 0, 49);
    no_materials.finish();
    try list.append(allocator, no_materials);
}

/// The height fields of a test: the edge cases and `num_random` random ones
fn heightFields(allocator: Allocator, gen: *Gen, num_random: usize) !std.ArrayList(HFInput) {
    var list: std.ArrayList(HFInput) = .empty;
    errdefer {
        for (list.items) |*i| i.deinit(allocator);
        list.deinit(allocator);
    }
    try edgeCaseHeightFields(allocator, &list);
    for (0..num_random) |_| {
        const input = try gen.heightField(allocator, .{});
        try list.append(allocator, input);
    }
    return list;
}

fn freeHeightFields(allocator: Allocator, list: *std.ArrayList(HFInput)) void {
    for (list.items) |*i| i.deinit(allocator);
    list.deinit(allocator);
}

// ---------------------------------------------------------------------------------------------------------------------
// Zolt side of the queries

fn zoltProperties(table: *const MaterialTable, hf: *const HeightFieldShape, input: *const PropertiesInput) PropertiesOutput {
    var o = std.mem.zeroes(PropertiesOutput);
    const shape = hf.asShape();
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
    o.must_be_static = @intFromBool(shape.mustBeStatic());
    o.min_height_value = hf.getMinHeightValue();
    o.max_height_value = hf.getMaxHeightValue();
    o.sample_count = hf.getSampleCount();
    o.block_size = hf.getBlockSize();
    const materials = hf.getMaterialList();
    o.num_materials = @intCast(materials.len);
    for (materials[0..@min(materials.len, 260)], 0..) |m, i| o.materials[i] = table.indexOf(m.get());
    return o;
}

/// A shape filter that rejects one sub shape ID (RejectFilter in HeightFieldShapeReference.cpp)
const RejectFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: ShapeFilter = .init(@This()),
    reject: u32,

    pub fn shouldCollide(self: *const RejectFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = shape2;
        return sub_shape_id_of_shape2.getValue() != self.reject;
    }
};

/// Run a query with the collector selected by `kind` and store the hits (RunWithCollector in the reference)
fn runWithCollector(allocator: Allocator, comptime CollectorType: type, kind: c_int, early_out: f32, context: *const TransformedShape, query: anytype, store: anytype) !void {
    switch (kind) {
        0 => {
            var collector = AllHitCollisionCollector(CollectorType).init(allocator);
            defer collector.deinit();
            collector.base.setContext(context);
            if (early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(early_out);
            query.run(&collector.base);
            try collector.checkError();
            for (collector.hits.items) |*h| store.add(h);
        },
        1 => {
            var collector = AnyHitCollisionCollector(CollectorType).init();
            defer collector.deinit();
            collector.base.setContext(context);
            if (early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(early_out);
            query.run(&collector.base);
            if (collector.hadHit()) store.add(&collector.hit);
        },
        else => {
            var collector = ClosestHitCollisionCollector(CollectorType).init();
            defer collector.deinit();
            collector.base.setContext(context);
            if (early_out < collector.base.getEarlyOutFraction()) collector.base.updateEarlyOutFraction(early_out);
            query.run(&collector.base);
            if (collector.hadHit()) store.add(&collector.hit);
        },
    }
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
    settings.back_face_mode_triangles = if (input.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    const reject_filter: RejectFilter = .{ .reject = input.reject_id };
    const default_filter: ShapeFilter = .{};
    const filter: *const ShapeFilter = if (input.reject_id != 0xffffffff) &reject_filter.base else &default_filter;
    const Query = struct {
        shape: *const Shape,
        ray: RayCast,
        settings: *const RayCastSettings,
        creator: SubShapeIDCreator,
        filter: *const ShapeFilter,
        fn run(q: @This(), collector: *CastRayCollector) void {
            q.shape.castRayCollector(q.ray, q.settings, q.creator, collector, q.filter);
        }
    };
    const Store = struct {
        out: *RayOutput,
        fn add(s: @This(), h: *const RayCastResult) void {
            if (s.out.num_hits < max_ray_hits)
                s.out.hits[s.out.num_hits] = .{ .fraction = h.fraction, .body_id = h.body_id.getIndexAndSequenceNumber(), .sub_shape_id = h.sub_shape_id2.getValue() };
            s.out.num_hits += 1;
        }
    };
    try runWithCollector(allocator, CastRayCollector, input.collector, input.early_out, &context, Query{ .shape = shape, .ray = ray, .settings = &settings, .creator = creator, .filter = filter }, Store{ .out = &o });
    return o;
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

/// Build the convex shape of a description
fn createConvex(allocator: Allocator, desc: ConvexDesc) !Ref(Shape) {
    if (desc.kind == 0)
        return Ref(Shape).init((try SphereShape.create(allocator, desc.radius, .{})).asShapeMut());
    return Ref(Shape).init((try BoxShape.create(allocator, vec3(desc.half_extent), .{ .convex_radius = desc.convex_radius })).asShapeMut());
}

const HitsStore = struct {
    out: *HitsOutput,

    fn add(s: HitsStore, h: anytype) void {
        if (s.out.num_hits < max_hits) {
            const o = &s.out.hits[s.out.num_hits];
            if (@typeInfo(@TypeOf(h)).pointer.child == ShapeCastResult) {
                o.fraction = h.fraction;
                o.back_face = @intFromBool(h.is_back_face_hit);
                storeCollideHit(&h.base, o);
            } else {
                o.fraction = 0.0;
                o.back_face = 0;
                storeCollideHit(h, o);
            }
        }
        s.out.num_hits += 1;
    }
};

fn zoltCollide(allocator: Allocator, height_field: *const Shape, input: *const CollideInput) !HitsOutput {
    var convex = try createConvex(allocator, input.convex);
    defer convex.deinit();
    const shape1 = if (input.height_field_first != 0) height_field else convex.get().?;
    const shape2 = if (input.height_field_first != 0) convex.get().? else height_field;
    var settings: CollideShapeSettings = .{};
    settings.max_separation_distance = input.max_separation_distance;
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.active_edge_mode = if (input.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.back_face_mode = if (input.back_face_mode != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    settings.active_edge_movement_direction = vec3(input.active_edge_movement_direction);
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    const Query = struct {
        shape1: *const Shape,
        shape2: *const Shape,
        input: *const CollideInput,
        settings: *const CollideShapeSettings,
        fn run(q: @This(), collector: *CollideShapeCollector) void {
            CollisionDispatch.collideShapeVsShape(q.shape1, q.shape2, vec3(q.input.scale1), vec3(q.input.scale2), mat44(q.input.transform1), mat44(q.input.transform2), makeCreator(q.input.creator1), makeCreator(q.input.creator2), q.settings, collector, &.{});
        }
    };
    var o = std.mem.zeroes(HitsOutput);
    try runWithCollector(allocator, CollideShapeCollector, input.collector, input.early_out, &context, Query{ .shape1 = shape1, .shape2 = shape2, .input = input, .settings = &settings }, HitsStore{ .out = &o });
    return o;
}

fn zoltCast(allocator: Allocator, height_field: *const Shape, input: *const CastInput) !HitsOutput {
    var convex = try createConvex(allocator, input.convex);
    defer convex.deinit();
    const cast_shape = if (input.height_field_cast != 0) height_field else convex.get().?;
    const target = if (input.height_field_cast != 0) convex.get().? else height_field;
    var settings: ShapeCastSettings = .{};
    settings.collision_tolerance = input.collision_tolerance;
    settings.penetration_tolerance = input.penetration_tolerance;
    settings.extra_convex_radius = input.extra_convex_radius;
    settings.active_edge_mode = if (input.active_edge_mode != 0) .collide_with_all else .collide_only_with_active;
    settings.back_face_mode_triangles = if (input.back_face_mode_triangles != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.back_face_mode_convex = if (input.back_face_mode_convex != 0) .collide_with_back_faces else .ignore_back_faces;
    settings.use_shrunken_shape_and_convex_radius = input.use_shrunken_shape != 0;
    settings.return_deepest_point = input.return_deepest_point != 0;
    settings.collect_faces_mode = if (input.collect_faces != 0) .collect_faces else .no_faces;
    settings.active_edge_movement_direction = vec3(input.active_edge_movement_direction);
    const shape_cast = ShapeCast.init(cast_shape, vec3(input.scale1), mat44(input.start), vec3(input.direction));
    const context = TransformedShape.init(RVec3.zero(), Quat.identity(), null, .init(input.body_id), .{});
    const Query = struct {
        shape_cast: *const ShapeCast,
        target: *const Shape,
        input: *const CastInput,
        settings: *const ShapeCastSettings,
        fn run(q: @This(), collector: *CastShapeCollector) void {
            CollisionDispatch.castShapeVsShapeWorldSpace(q.shape_cast, q.settings, q.target, vec3(q.input.scale2), &.{}, mat44(q.input.transform2), makeCreator(q.input.creator1), makeCreator(q.input.creator2), collector);
        }
    };
    var o = std.mem.zeroes(HitsOutput);
    try runWithCollector(allocator, CastShapeCollector, input.collector, input.early_out, &context, Query{ .shape_cast = &shape_cast, .target = target, .input = input, .settings = &settings }, HitsStore{ .out = &o });
    return o;
}

/// Jolt walks blocks outside the height field (it reads beyond its range block grid and height samples, undefined
/// behavior; JPH_ASSERT and Zolt's safe builds assert) when the bounds of an empty range block (min 0xffff, max 0)
/// collapse to a valid box: offset.y + scale.y * 65535 == offset.y in float (a flat height field far from the origin)
/// while the number of blocks is not a power of 2 (only then the walker reaches blocks outside the height field).
/// The queries that walk the height field are not compared for these.
fn walksOutside(hf: *const HeightFieldShape) bool {
    const oy = Vec4.replicate(hf.offset.getY());
    const sy = Vec4.replicate(hf.scale.getY());
    const empty_min_y = oy.add(sy.mul(Vec4.replicate(65535.0)));
    const empty_max_y = oy.add(sy.mul(Vec4.replicate(0.0)));
    return empty_min_y.getX() <= empty_max_y.getX() and !math.isPowerOf2(hf.sample_count / hf.block_size);
}

/// A random point on (near) the surface of the height field, in its local space
fn surfacePoint(gen: *Gen, hf: *const HeightFieldShape) Vec3 {
    const count = hf.getSampleCount();
    const x: u32 = gen.range(0, count - 1);
    const y: u32 = gen.range(0, count - 1);
    var p = hf.getPosition(x, y);
    if (hf.isNoCollision(x, y) or hf.getStats().num_triangles == 0 or gen.oneIn(5)) {
        const bounds = hf.asShape().getLocalBounds();
        p = Vec3.init(gen.plain(bounds.min.getX(), bounds.max.getX() + 1.0e-3), gen.plain(bounds.min.getY() - 1, bounds.max.getY() + 1), gen.plain(bounds.min.getZ(), bounds.max.getZ() + 1.0e-3));
    } else if (x + 1 < count and y + 1 < count and gen.oneIn(2)) {
        // Somewhere inside the quad
        const q = hf.getPosition(x + 1, y + 1);
        const t = gen.plain(0, 1);
        p = p.add(q.sub(p).mulScalar(t));
    }
    return p;
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests

test "HeightFieldShape parity: settings, Jolt's error texts, quantized data, clone and binary state" {
    const allocator = std.testing.allocator;
    var table = try MaterialTable.init(allocator);
    defer table.deinit();
    var gen: Gen = .{};
    var create_checker: Checker = .{ .name = "create" };
    var state_checker: Checker = .{ .name = "binary state" };
    var clone_checker: Checker = .{ .name = "clone" };
    var restore_checker: Checker = .{ .name = "restore" };
    var children_checker: Checker = .{ .name = "save with children" };

    var invalid: std.ArrayList(HFInput) = .empty;
    defer freeHeightFields(allocator, &invalid);
    try invalidHeightFields(allocator, &invalid);
    for (invalid.items) |*input| {
        var pair = try build(allocator, &table, input, &create_checker);
        if (pair) |*p| {
            p.deinit();
            create_checker.check(input.desc, @as(u32, 1), @as(u32, 0)); // Must be invalid
        }
    }

    var inputs = try heightFields(allocator, &gen, 300);
    defer freeHeightFields(allocator, &inputs);
    for (inputs.items, 0..) |*input, input_index| {
        var pair = (try build(allocator, &table, input, &create_checker)) orelse continue;
        defer pair.deinit();
        try checkState(allocator, &state_checker, .{ input_index, input.desc }, &pair);

        // Clone (Jolt's Clone of a height field without collision copies from a null buffer, undefined behavior: its
        // clone has uninitialized buffers, Zolt's clone has no collision either)
        if (pair.hf().buffer != null) {
            var clone = Ref(Shape).init((try pair.hf().clone(allocator)).asShapeMut());
            defer clone.deinit();
            const zolt_bytes = try zoltState(allocator, clone.get().?);
            defer allocator.free(zolt_bytes);
            const size = jolt.jolt_hf_clone_state(pair.jolt, undefined, 0);
            const jolt_bytes = try allocator.alloc(u8, size);
            defer allocator.free(jolt_bytes);
            _ = jolt.jolt_hf_clone_state(pair.jolt, jolt_bytes.ptr, size);
            checkBytes(&clone_checker, .{ input_index, "clone" }, zolt_bytes, jolt_bytes);
            const materials = clone.get().?.cast(HeightFieldShape).getMaterialList();
            const original = pair.hf().getMaterialList();
            clone_checker.check(.{ input_index, "materials" }, materials.len, original.len);
            for (materials, original) |a, b| clone_checker.check(.{ input_index, "material" }, @intFromPtr(a.get()), @intFromPtr(b.get()));
        }

        // Restore the bytes of Jolt (with the materials of the table), save again
        {
            const jolt_bytes = try joltState(allocator, pair.jolt);
            defer allocator.free(jolt_bytes);
            var material_ids: [260]u32 = undefined;
            const materials = pair.hf().getMaterialList();
            for (materials, 0..) |m, i| material_ids[i] = table.indexOf(m.get());

            const jolt_restored = try allocator.alloc(u8, jolt_bytes.len);
            defer allocator.free(jolt_restored);
            var jolt_materials: [260]u32 = @splat(0);
            var jolt_num_materials: u32 = 0;
            const jolt_size = jolt.jolt_hf_restore_state(jolt_bytes.ptr, @intCast(jolt_bytes.len), &material_ids, @intCast(materials.len), jolt_restored.ptr, @intCast(jolt_restored.len), &jolt_materials, &jolt_num_materials);

            var reader: std.Io.Reader = .fixed(jolt_bytes);
            var stream_in = StreamInWrapper.init(&reader);
            var result = try Shape.restoreFromBinaryState(allocator, stream_in.streamIn());
            defer result.deinit();
            var zolt_materials: [260]u32 = @splat(0);
            var zolt_num_materials: u32 = 0;
            var zolt_restored: []u8 = &.{};
            defer allocator.free(zolt_restored);
            if (result.isValid()) {
                const restored = result.getPtr().?;
                var list: [260]PhysicsMaterialRefC = undefined;
                for (materials, 0..) |m, i| list[i] = m;
                try restored.restoreMaterialState(list[0..materials.len]);
                var saved: PhysicsMaterialList = .empty;
                defer {
                    for (saved.items) |*m| m.deinit();
                    saved.deinit(allocator);
                }
                try restored.saveMaterialState(allocator, &saved);
                for (saved.items) |m| {
                    zolt_materials[zolt_num_materials] = table.indexOf(m.get());
                    zolt_num_materials += 1;
                }
                zolt_restored = try zoltState(allocator, restored);
            }
            checkBytes(&restore_checker, .{ input_index, "restore" }, zolt_restored, jolt_restored[0..jolt_size]);
            restore_checker.check(.{ input_index, "restored materials" }, .{ zolt_num_materials, zolt_materials }, .{ jolt_num_materials, jolt_materials });
        }

        // SaveWithChildren (writes the materials too) and sRestoreWithChildren
        {
            const capacity: u32 = 1 << 22;
            const jolt_bytes = try allocator.alloc(u8, capacity);
            defer allocator.free(jolt_bytes);
            const jolt_restored = try allocator.alloc(u8, capacity);
            defer allocator.free(jolt_restored);
            var jolt_restored_size: u32 = 0;
            const jolt_size = jolt.jolt_hf_save_with_children(pair.jolt, jolt_bytes.ptr, capacity, jolt_restored.ptr, &jolt_restored_size);

            var out: std.Io.Writer.Allocating = .init(allocator);
            defer out.deinit();
            var wrapper = StreamOutWrapper.init(&out.writer);
            var shape_map: Shape.ShapeToIDMap = .empty;
            defer shape_map.deinit(allocator);
            var material_map: Shape.MaterialToIDMap = .empty;
            defer material_map.deinit(allocator);
            try pair.shape().saveWithChildren(allocator, wrapper.streamOut(), &shape_map, &material_map);
            checkBytes(&children_checker, .{ input_index, "save" }, out.written(), jolt_bytes[0..jolt_size]);

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
            var restored_out: std.Io.Writer.Allocating = .init(allocator);
            defer restored_out.deinit();
            if (result.isValid()) {
                var restored_wrapper = StreamOutWrapper.init(&restored_out.writer);
                var shape_map2: Shape.ShapeToIDMap = .empty;
                defer shape_map2.deinit(allocator);
                var material_map2: Shape.MaterialToIDMap = .empty;
                defer material_map2.deinit(allocator);
                try result.getPtr().?.saveWithChildren(allocator, restored_wrapper.streamOut(), &shape_map2, &material_map2);
            }
            checkBytes(&children_checker, .{ input_index, "restore" }, restored_out.written(), jolt_restored[0..jolt_restored_size]);
        }
    }
    try finishAll(&.{ &create_checker, &state_checker, &clone_checker, &restore_checker, &children_checker });
}

test "HeightFieldShape parity: DetermineMinAndMaxSample and CalculateBitsPerSampleForError" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var checker: Checker = .{ .name = "settings helpers" };
    for (0..300) |i| {
        // CalculateBitsPerSampleForError reads beyond the samples when the sample count is not a multiple of the block size
        var input = try gen.heightField(allocator, .{ .multiple_of_block_size = true });
        defer input.deinit(allocator);
        const max_error: f32 = switch (i % 4) {
            0 => 0.0,
            1 => gen.plain(0, 1.0e-3),
            else => gen.plain(0, 2),
        };

        var settings = HeightFieldShapeSettings.initDefault(allocator);
        defer settings.deinit();
        settings.offset = vec3(input.desc.offset);
        settings.scale = vec3(input.desc.scale);
        settings.sample_count = input.desc.sample_count;
        settings.min_height_value = input.desc.min_height_value;
        settings.max_height_value = input.desc.max_height_value;
        settings.block_size = input.desc.block_size;
        settings.bits_per_sample = input.desc.bits_per_sample;
        try settings.height_samples.appendSlice(allocator, input.samples.items);
        const r = settings.determineMinAndMaxSample();
        const zolt_bits = settings.calculateBitsPerSampleForError(max_error);

        var jolt_values: [3]f32 = undefined;
        const jolt_bits = jolt.jolt_hf_settings_info(&input.desc, input.samples.items.ptr, max_error, &jolt_values);
        checker.check(.{ i, max_error }, .{ zolt_bits, [3]f32{ r.min_value, r.max_value, r.quantization_scale } }, .{ jolt_bits, jolt_values });
    }
    try checker.finish();
}

test "HeightFieldShape parity: properties, positions, materials, projection, GetHeights and GetMaterials" {
    const allocator = std.testing.allocator;
    var table = try MaterialTable.init(allocator);
    defer table.deinit();
    var gen: Gen = .{ .rng = .{ .state = 0x2468ace1 } };
    var create_checker: Checker = .{ .name = "create" };
    var properties_checker: Checker = .{ .name = "properties" };
    var samples_checker: Checker = .{ .name = "samples" };
    var project_checker: Checker = .{ .name = "project onto surface" };
    var heights_checker: Checker = .{ .name = "get heights" };
    var materials_checker: Checker = .{ .name = "get materials" };

    var inputs = try heightFields(allocator, &gen, 150);
    defer freeHeightFields(allocator, &inputs);
    for (inputs.items, 0..) |*input, input_index| {
        var pair = (try build(allocator, &table, input, &create_checker)) orelse continue;
        defer pair.deinit();
        const hf = pair.hf();
        const count = hf.getSampleCount();

        // Properties
        for (0..4) |_| {
            const transform = gen.transform(10);
            const props_input: PropertiesInput = .{
                .scale = if (gen.oneIn(3)) gen.vec(-3, 3) else gen.hfScale(),
                .transform = arr16(transform),
                .translation = .{ gen.plain(-1.0e5, 1.0e5), gen.plain(-10, 10), gen.plain(-1.0e5, 1.0e5) },
            };
            var jolt_props: PropertiesOutput = std.mem.zeroes(PropertiesOutput);
            jolt.jolt_hf_properties(pair.jolt, &props_input, &jolt_props);
            properties_checker.check(.{ input_index, props_input }, zoltProperties(&table, hf, &props_input), jolt_props);
        }

        // GetPosition / IsNoCollision of every sample, GetMaterial(x, y) of every quad
        {
            const n = @as(usize, count) * count;
            const jolt_positions = try allocator.alloc(f32, 3 * n);
            defer allocator.free(jolt_positions);
            const jolt_no_collision = try allocator.alloc(c_int, n);
            defer allocator.free(jolt_no_collision);
            const jolt_materials = try allocator.alloc(u32, (count - 1) * (count - 1));
            defer allocator.free(jolt_materials);
            jolt.jolt_hf_samples(pair.jolt, jolt_positions.ptr, jolt_no_collision.ptr, jolt_materials.ptr);
            for (0..count) |y| for (0..count) |x| {
                const i = y * count + x;
                const zolt_position = arr3(hf.getPosition(@intCast(x), @intCast(y)));
                const zolt_no_collision: c_int = @intFromBool(hf.isNoCollision(@intCast(x), @intCast(y)));
                samples_checker.check(.{ input_index, x, y }, .{ zolt_position, zolt_no_collision }, .{ jolt_positions[3 * i ..][0..3].*, jolt_no_collision[i] });
            };
            for (0..count - 1) |y| for (0..count - 1) |x| {
                samples_checker.check(.{ input_index, x, y, "material" }, table.indexOf(hf.getMaterialAt(@intCast(x), @intCast(y))), jolt_materials[y * (count - 1) + x]);
            };
        }

        // ProjectOntoSurface
        for (0..60) |k| {
            var point = arr3(surfacePoint(&gen, hf));
            if (k % 10 == 0) {
                // Exactly on a sample or on an edge
                const x: f32 = @floatFromInt(gen.range(0, count));
                const z: f32 = @floatFromInt(gen.range(0, count));
                point = arr3(vec3(input.desc.offset).add(hf.scale.mul(Vec3.init(x, 0, z))));
                if (k % 20 == 0) point[0] += hf.scale.getX() * 0.5;
            }
            if (k == 7) point = .{ std.math.nan(f32), 0, 0 };
            var jolt_position: P = undefined;
            var jolt_id: u32 = undefined;
            const jolt_result = jolt.jolt_hf_project(pair.jolt, &point, &jolt_position, &jolt_id);
            var zolt_position: P = .{ -1, -1, -1 };
            var zolt_id: u32 = SubShapeID.empty_value;
            var zolt_result: c_int = 0;
            if (hf.projectOntoSurface(vec3(point))) |s| {
                zolt_result = 1;
                zolt_position = arr3(s.position);
                zolt_id = s.sub_shape_id.getValue();
            }
            project_checker.check(.{ input_index, point }, .{ zolt_result, zolt_position, zolt_id }, .{ jolt_result, jolt_position, jolt_id });
        }

        // GetHeights of block aligned rectangles (and the whole height field), positive and negative strides
        const block_size = hf.getBlockSize();
        const num_blocks = count / block_size;
        for (0..8) |k| {
            var bx: u32 = 0;
            var by: u32 = 0;
            var nbx: u32 = num_blocks;
            var nby: u32 = num_blocks;
            if (k != 0) {
                bx = gen.range(0, num_blocks - 1);
                by = gen.range(0, num_blocks - 1);
                nbx = gen.range(if (k == 1) 0 else 1, num_blocks - bx);
                nby = gen.range(1, num_blocks - by);
            }
            const size_x = nbx * block_size;
            const size_y = nby * block_size;
            const width: u32 = size_x + gen.range(0, 3);
            const stride: i64 = if (gen.oneIn(3)) -@as(i64, width) else width;
            const first_row: i64 = if (stride < 0) @as(i64, size_y -| 1) * width else 0;
            const n = @as(usize, width) * @max(size_y, 1);
            const jolt_heights = try allocator.alloc(f32, n);
            defer allocator.free(jolt_heights);
            @memset(jolt_heights, -123.0);
            jolt.jolt_hf_get_heights(pair.jolt, bx * block_size, by * block_size, size_x, size_y, jolt_heights.ptr, first_row, stride);
            const zolt_heights = try allocator.alloc(f32, n);
            defer allocator.free(zolt_heights);
            @memset(zolt_heights, -123.0);
            hf.getHeights(bx * block_size, by * block_size, size_x, size_y, zolt_heights.ptr + @as(usize, @intCast(first_row)), @intCast(stride));
            for (zolt_heights, jolt_heights, 0..) |z, j, i| heights_checker.check(.{ input_index, bx, by, size_x, size_y, stride, i }, z, j);
        }

        // GetMaterials of random rectangles (x + size_x < sample count), positive and negative strides
        for (0..6) |_| {
            const x = gen.range(0, count - 2);
            const y = gen.range(0, count - 2);
            const size_x = gen.range(0, count - 1 - x - 1);
            const size_y = gen.range(1, count - 1 - y);
            const width: u32 = size_x + gen.range(0, 3);
            const stride: i64 = if (gen.oneIn(3)) -@as(i64, width) else width;
            const first_row: i64 = if (stride < 0) @as(i64, size_y - 1) * width else 0;
            const n = @as(usize, width) * size_y;
            if (n == 0) continue;
            const jolt_out = try allocator.alloc(u8, n);
            defer allocator.free(jolt_out);
            @memset(jolt_out, 0xcd);
            jolt.jolt_hf_get_materials(pair.jolt, x, y, size_x, size_y, jolt_out.ptr, first_row, stride);
            const zolt_out = try allocator.alloc(u8, n);
            defer allocator.free(zolt_out);
            @memset(zolt_out, 0xcd);
            hf.getMaterials(x, y, size_x, size_y, zolt_out.ptr + @as(usize, @intCast(first_row)), @intCast(stride));
            for (zolt_out, jolt_out, 0..) |z, j, i| materials_checker.check(.{ input_index, x, y, size_x, size_y, stride, i }, z, j);
        }
    }
    try finishAll(&.{ &create_checker, &properties_checker, &samples_checker, &project_checker, &heights_checker, &materials_checker });
}

test "HeightFieldShape parity: sub shape functions of every triangle" {
    const allocator = std.testing.allocator;
    var table = try MaterialTable.init(allocator);
    defer table.deinit();
    var gen: Gen = .{ .rng = .{ .state = 0x13572468 } };
    var create_checker: Checker = .{ .name = "create" };
    var checker: Checker = .{ .name = "sub shape" };

    var inputs = try heightFields(allocator, &gen, 80);
    defer freeHeightFields(allocator, &inputs);
    for (inputs.items, 0..) |*input, input_index| {
        var pair = (try build(allocator, &table, input, &create_checker)) orelse continue;
        defer pair.deinit();
        const hf = pair.hf();
        const count = hf.getSampleCount();
        if (count > 40) continue; // Every triangle of the small ones is enough
        const bits = hf.asShape().getSubShapeIDBitsRecursive();
        for (0..count - 1) |y| for (0..count - 1) |x| for (0..2) |t| {
            const id = SubShapeIDCreator.pushID(.{}, @intCast((x + y * count) * 2 + t), bits).getID();
            const sub_input: SubShapeInput = .{
                .sub_shape_id = id.getValue(),
                .point = gen.vec(-5, 5),
                .direction = gen.vec(-1, 1),
                .scale = if (gen.oneIn(2)) .{ 1, 1, 1 } else gen.hfScale(),
                .transform = arr16(gen.transform(5)),
            };
            var jolt_out = std.mem.zeroes(SubShapeOutput);
            jolt.jolt_hf_sub_shape(pair.jolt, &sub_input, &jolt_out);

            var zolt_out = std.mem.zeroes(SubShapeOutput);
            const shape = hf.asShape();
            zolt_out.normal = arr3(shape.getSurfaceNormal(id, vec3(sub_input.point)));
            zolt_out.material = table.indexOf(shape.getMaterial(id));
            const c = hf.getSubShapeCoordinates(id);
            zolt_out.x = c.x;
            zolt_out.y = c.y;
            zolt_out.triangle = c.triangle_index;
            var face: Shape.SupportingFace = .empty;
            shape.getSupportingFace(id, vec3(sub_input.direction), vec3(sub_input.scale), mat44(sub_input.transform), &face);
            storeFace(&face, &zolt_out.face_count, &zolt_out.face);
            const leaf = shape.getLeafShape(id);
            zolt_out.leaf_remainder = leaf.remainder.getValue();
            zolt_out.leaf_is_self = @intFromBool(leaf.shape == shape);
            zolt_out.user_data = shape.getSubShapeUserData(id);
            checker.check(.{ input_index, x, y, t }, zolt_out, jolt_out);
        };
    }
    try finishAll(&.{ &create_checker, &checker });
}

test "HeightFieldShape parity: CastRay (single hit and collectors) and CollidePoint" {
    const allocator = std.testing.allocator;
    var table = try MaterialTable.init(allocator);
    defer table.deinit();
    var gen: Gen = .{ .rng = .{ .state = 0x9e3779b9 } };
    var create_checker: Checker = .{ .name = "create" };
    var checker: Checker = .{ .name = "cast ray" };
    var point_checker: Checker = .{ .name = "collide point" };

    var inputs = try heightFields(allocator, &gen, 100);
    defer freeHeightFields(allocator, &inputs);
    for (inputs.items, 0..) |*input, input_index| {
        var pair = (try build(allocator, &table, input, &create_checker)) orelse continue;
        defer pair.deinit();
        if (walksOutside(pair.hf())) continue;
        const hf = pair.hf();
        const bounds = hf.asShape().getLocalBounds();
        const height = @max(bounds.getSize().getY(), 1.0);
        for (0..150) |k| {
            var origin: Vec3 = undefined;
            var direction: Vec3 = undefined;
            const target = surfacePoint(&gen, hf);
            switch (k % 6) {
                0, 1 => {
                    // From above, straight down (sometimes exactly through a sample) or slanted
                    origin = target.add(Vec3.init(0, height + 1, 0));
                    direction = Vec3.init(0, -2 * (height + 1), 0);
                    if (k % 6 == 1) direction = direction.add(vec3(gen.plainVec(-1, 1)));
                },
                2 => {
                    // Horizontal (parallel to a flat surface)
                    origin = target.add(Vec3.init(-gen.plain(1, 20), gen.plain(-0.01, 0.01), gen.plain(-1, 1)));
                    direction = Vec3.init(gen.plain(1, 40), 0, gen.plain(-1, 1));
                },
                3 => {
                    // From below (back faces) or starting under the surface
                    origin = target.sub(Vec3.init(0, gen.plain(0, height + 1), 0));
                    direction = Vec3.init(gen.plain(-1, 1), 2 * (height + 1), gen.plain(-1, 1));
                },
                4 => {
                    // Long ray through the whole height field
                    origin = bounds.min.sub(vec3(gen.plainVec(0, 5)));
                    direction = bounds.max.sub(origin).add(vec3(gen.plainVec(0, 5))).mulScalar(gen.plain(0.5, 2));
                },
                else => {
                    origin = vec3(gen.vec(-20, 20));
                    direction = vec3(gen.vec(-40, 40));
                    if (gen.oneIn(10)) direction = Vec3.zero();
                },
            }
            const ray_input: RayInput = .{
                .origin = arr3(origin),
                .direction = arr3(direction),
                .creator = gen.creator(),
                .fraction = if (gen.oneIn(4)) gen.plain(0, 1) else 1.0 + math.flt_epsilon,
                .back_face_mode = @intFromBool(gen.oneIn(2)),
                .collector = @intCast(gen.index(3)),
                .early_out = if (gen.oneIn(4)) gen.plain(0, 1) else math.flt_max,
                .body_id = gen.next() & 0x7fffff,
                .reject_id = if (gen.oneIn(10)) makeCreator(gen.creator()).getID().getValue() else 0xffffffff,
            };
            var jolt_out = std.mem.zeroes(RayOutput);
            jolt.jolt_hf_cast_ray(pair.jolt, &ray_input, &jolt_out);
            const zolt_out = try zoltCastRay(allocator, hf.asShape(), &ray_input);
            checker.check(.{ input_index, ray_input }, zolt_out, jolt_out);
        }

        // CollidePoint: a height field has no volume, never a hit
        for (0..5) |_| {
            const point = arr3(surfacePoint(&gen, hf).add(Vec3.init(0, -0.1, 0)));
            const creator = gen.creator();
            const jolt_hits = jolt.jolt_hf_collide_point(pair.jolt, &point, &creator);
            var collector = AllHitCollisionCollector(CollidePointCollector).init(allocator);
            defer collector.deinit();
            hf.asShape().collidePoint(vec3(point), makeCreator(creator), &collector.base, &.{});
            try collector.checkError();
            point_checker.check(.{ input_index, point }, @as(u32, @intCast(collector.hits.items.len)), jolt_hits);
        }
    }
    try finishAll(&.{ &create_checker, &checker, &point_checker });
}

/// Random settings for collide / cast inputs
fn collideSettings(gen: *Gen) struct { max_separation: f32, collision_tolerance: f32, penetration_tolerance: f32, active_edge_mode: c_int, back_face_mode: c_int, collect_faces: c_int, movement: P } {
    return .{
        .max_separation = if (gen.oneIn(2)) 0.0 else gen.plain(0, 0.5),
        .collision_tolerance = if (gen.oneIn(2)) 1.0e-4 else gen.plain(1.0e-5, 1.0e-3),
        .penetration_tolerance = if (gen.oneIn(2)) 1.0e-4 else gen.plain(1.0e-5, 1.0e-3),
        .active_edge_mode = @intFromBool(gen.oneIn(3)),
        .back_face_mode = @intFromBool(gen.oneIn(3)),
        .collect_faces = @intFromBool(gen.oneIn(2)),
        .movement = if (gen.oneIn(2)) .{ 0, 0, 0 } else gen.vec(-1, 1),
    };
}

test "HeightFieldShape parity: collide sphere / box vs height field (and reversed) through CollisionDispatch" {
    const allocator = std.testing.allocator;
    var table = try MaterialTable.init(allocator);
    defer table.deinit();
    var gen: Gen = .{ .rng = .{ .state = 0x7f4a7c15 } };
    var create_checker: Checker = .{ .name = "create" };
    var checker: Checker = .{ .name = "collide" };

    var inputs = try heightFields(allocator, &gen, 60);
    defer freeHeightFields(allocator, &inputs);
    for (inputs.items, 0..) |*input, input_index| {
        var pair = (try build(allocator, &table, input, &create_checker)) orelse continue;
        defer pair.deinit();
        if (walksOutside(pair.hf())) continue;
        const hf = pair.hf();
        for (0..60) |k| {
            const convex = gen.convex();
            const convex_scale = gen.convexScale(convex);
            const hf_scale: P = if (gen.oneIn(2)) .{ 1, 1, 1 } else gen.hfScale();
            const hf_transform = if (gen.oneIn(3)) Mat44.identity() else gen.transform(5);

            // Place the convex shape near the surface of the scaled and transformed height field
            const surface = hf_transform.mulVec3(surfacePoint(&gen, hf).mul(vec3(hf_scale)));
            const extent: f32 = if (convex.kind == 0) convex.radius else convex.half_extent[1];
            const lift = if (k % 4 == 0) extent else extent * gen.plain(-0.5, 1.5); // Touching or penetrating
            const convex_transform = Mat44.rotationTranslation(if (gen.oneIn(3)) Quat.identity() else gen.rotation(), surface.add(Vec3.init(0, lift, 0)));
            const s = collideSettings(&gen);
            const hf_first = gen.oneIn(3);
            const collide_input: CollideInput = .{
                .convex = convex,
                .height_field_first = @intFromBool(hf_first),
                .scale1 = if (hf_first) hf_scale else convex_scale,
                .scale2 = if (hf_first) convex_scale else hf_scale,
                .transform1 = arr16(if (hf_first) hf_transform else convex_transform),
                .transform2 = arr16(if (hf_first) convex_transform else hf_transform),
                .creator1 = gen.creator(),
                .creator2 = gen.creator(),
                .max_separation_distance = s.max_separation,
                .collision_tolerance = s.collision_tolerance,
                .penetration_tolerance = s.penetration_tolerance,
                .active_edge_mode = s.active_edge_mode,
                .back_face_mode = s.back_face_mode,
                .collect_faces = s.collect_faces,
                .active_edge_movement_direction = s.movement,
                .early_out = if (gen.oneIn(5)) gen.plain(-1, 1) else math.flt_max,
                .body_id = gen.next() & 0x7fffff,
                .collector = @intCast(gen.index(3)),
            };
            var jolt_out = std.mem.zeroes(HitsOutput);
            jolt.jolt_hf_collide(pair.jolt, &collide_input, &jolt_out);
            // Only the stored hits are defined
            const zolt_out = try zoltCollide(allocator, hf.asShape(), &collide_input);
            checker.check(.{ input_index, k }, zolt_out.num_hits, jolt_out.num_hits);
            for (0..@min(zolt_out.num_hits, max_hits)) |h| checker.check(.{ input_index, k, h, collide_input }, zolt_out.hits[h], jolt_out.hits[h]);
        }
    }
    try finishAll(&.{ &create_checker, &checker });
}

test "HeightFieldShape parity: cast sphere / box vs height field (and the height field vs sphere / box)" {
    const allocator = std.testing.allocator;
    var table = try MaterialTable.init(allocator);
    defer table.deinit();
    var gen: Gen = .{ .rng = .{ .state = 0x51ed2701 } };
    var create_checker: Checker = .{ .name = "create" };
    var checker: Checker = .{ .name = "cast" };

    var inputs = try heightFields(allocator, &gen, 60);
    defer freeHeightFields(allocator, &inputs);
    for (inputs.items, 0..) |*input, input_index| {
        var pair = (try build(allocator, &table, input, &create_checker)) orelse continue;
        defer pair.deinit();
        if (walksOutside(pair.hf())) continue;
        const hf = pair.hf();
        for (0..50) |k| {
            const convex = gen.convex();
            const convex_scale = gen.convexScale(convex);
            const hf_scale: P = if (gen.oneIn(2)) .{ 1, 1, 1 } else gen.hfScale();
            const hf_transform = if (gen.oneIn(3)) Mat44.identity() else gen.transform(5);
            const surface = hf_transform.mulVec3(surfacePoint(&gen, hf).mul(vec3(hf_scale)));
            const extent: f32 = if (convex.kind == 0) convex.radius else convex.half_extent[1];

            // Cast down onto the surface, sideways, or starting in contact
            var start_position = surface.add(Vec3.init(0, extent + gen.plain(0, 3), 0));
            var direction = Vec3.init(gen.plain(-0.5, 0.5), -gen.plain(0.5, 6), gen.plain(-0.5, 0.5));
            switch (k % 5) {
                0 => start_position = surface.add(Vec3.init(0, extent * gen.plain(-0.5, 1), 0)),
                1 => direction = Vec3.init(gen.plain(-5, 5), 0, gen.plain(-5, 5)),
                2 => direction = Vec3.init(0, -gen.plain(0.5, 6), 0),
                else => {},
            }
            const convex_start = Mat44.rotationTranslation(if (gen.oneIn(3)) Quat.identity() else gen.rotation(), start_position);
            const s = collideSettings(&gen);
            const hf_cast = gen.oneIn(4);
            const cast_input: CastInput = .{
                .convex = convex,
                .height_field_cast = @intFromBool(hf_cast),
                .scale1 = if (hf_cast) hf_scale else convex_scale,
                .start = arr16(if (hf_cast) hf_transform else convex_start),
                .direction = arr3(if (hf_cast) direction.negate() else direction),
                .scale2 = if (hf_cast) convex_scale else hf_scale,
                .transform2 = arr16(if (hf_cast) convex_start else hf_transform),
                .creator1 = gen.creator(),
                .creator2 = gen.creator(),
                .collision_tolerance = s.collision_tolerance,
                .penetration_tolerance = s.penetration_tolerance,
                .extra_convex_radius = if (gen.oneIn(3)) gen.plain(0, 0.3) else 0.0,
                .active_edge_mode = s.active_edge_mode,
                .back_face_mode_triangles = s.back_face_mode,
                .back_face_mode_convex = @intFromBool(gen.oneIn(3)),
                .use_shrunken_shape = @intFromBool(gen.oneIn(3)),
                .return_deepest_point = @intFromBool(gen.oneIn(3)),
                .collect_faces = s.collect_faces,
                .active_edge_movement_direction = s.movement,
                .early_out = if (gen.oneIn(5)) gen.plain(0, 1) else math.flt_max,
                .body_id = gen.next() & 0x7fffff,
                .collector = @intCast(gen.index(3)),
            };
            var jolt_out = std.mem.zeroes(HitsOutput);
            jolt.jolt_hf_cast(pair.jolt, &cast_input, &jolt_out);
            const zolt_out = try zoltCast(allocator, hf.asShape(), &cast_input);
            checker.check(.{ input_index, k }, zolt_out.num_hits, jolt_out.num_hits);
            for (0..@min(zolt_out.num_hits, max_hits)) |h| checker.check(.{ input_index, k, h, cast_input }, zolt_out.hits[h], jolt_out.hits[h]);
        }
    }
    try finishAll(&.{ &create_checker, &checker });
}

test "HeightFieldShape parity: GetTrianglesStart / Next" {
    const allocator = std.testing.allocator;
    var table = try MaterialTable.init(allocator);
    defer table.deinit();
    var gen: Gen = .{ .rng = .{ .state = 0x0badf00d } };
    var create_checker: Checker = .{ .name = "create" };
    var checker: Checker = .{ .name = "triangles" };

    const max_calls = 2048;
    const max_triangles = 1 << 16;
    const jolt_counts = try allocator.alloc(c_int, max_calls);
    defer allocator.free(jolt_counts);
    const jolt_vertices = try allocator.alloc(f32, 9 * max_triangles);
    defer allocator.free(jolt_vertices);
    const jolt_materials = try allocator.alloc(u32, max_triangles);
    defer allocator.free(jolt_materials);
    const zolt_vertices = try allocator.alloc(f32, 9 * max_triangles);
    defer allocator.free(zolt_vertices);
    const zolt_materials = try allocator.alloc(u32, max_triangles);
    defer allocator.free(zolt_materials);

    var inputs = try heightFields(allocator, &gen, 80);
    defer freeHeightFields(allocator, &inputs);
    for (inputs.items, 0..) |*input, input_index| {
        var pair = (try build(allocator, &table, input, &create_checker)) orelse continue;
        defer pair.deinit();
        if (walksOutside(pair.hf())) continue;
        const hf = pair.hf();
        for (0..6) |k| {
            const scale: P = if (gen.oneIn(3)) .{ 1, 1, 1 } else gen.hfScale();
            const position: P = if (gen.oneIn(3)) .{ 0, 0, 0 } else gen.vec(-5, 5);
            const rotation = arr4((if (gen.oneIn(3)) Quat.identity() else gen.rotation()).getXYZW());
            var box: [6]f32 = boxArr(AABox.biggest());
            if (k % 2 == 1) {
                // A box around a point of the surface (in world space)
                const center = Mat44.rotationTranslation(quat(rotation), vec3(position)).mulVec3(surfacePoint(&gen, hf).mul(vec3(scale)));
                const half = vec3(gen.plainVec(0.1, 6));
                box = boxArr(AABox.init(center.sub(half), center.add(half)));
            }
            const max_requested: c_int = switch (gen.index(4)) {
                0 => 32,
                1 => 33,
                2 => @intCast(gen.range(32, 200)),
                else => 2000,
            };
            const with_materials: c_int = @intFromBool(gen.oneIn(2));
            @memset(jolt_counts, -1);
            const jolt_calls = jolt.jolt_hf_triangles(pair.jolt, &box, &position, &rotation, &scale, max_requested, with_materials, max_calls, max_triangles, jolt_counts.ptr, jolt_vertices.ptr, jolt_materials.ptr);

            var context: Shape.GetTrianglesContext = .{};
            hf.asShape().getTrianglesStart(&context, AABox.init(vec3(box[0..3].*), vec3(box[3..6].*)), vec3(position), quat(rotation), vec3(scale));
            const triangles = try allocator.alloc(Float3, 3 * @as(usize, @intCast(max_requested)));
            defer allocator.free(triangles);
            const materials = try allocator.alloc(*const PhysicsMaterial, @intCast(max_requested));
            defer allocator.free(materials);
            var zolt_calls: c_int = 0;
            var total: usize = 0;
            while (true) {
                const count = hf.asShape().getTrianglesNext(&context, @intCast(max_requested), triangles, if (with_materials != 0) materials else null);
                checker.check(.{ input_index, k, zolt_calls }, @as(c_int, @intCast(count)), jolt_counts[@intCast(zolt_calls)]);
                zolt_calls += 1;
                var i: usize = 0;
                while (i < count and total < max_triangles) : ({
                    i += 1;
                    total += 1;
                }) {
                    for (0..3) |v| {
                        const t = triangles[3 * i + v];
                        zolt_vertices[9 * total + 3 * v ..][0..3].* = .{ t.x, t.y, t.z };
                    }
                    zolt_materials[total] = if (with_materials != 0) table.indexOf(materials[i]) else unknown_material;
                }
                if (count == 0 or zolt_calls == max_calls or total >= max_triangles) break;
            }
            checker.check(.{ input_index, k, "calls" }, zolt_calls, jolt_calls);
            for (0..total) |t| checker.check(.{ input_index, k, t, box, position, rotation, scale }, .{ zolt_vertices[9 * t ..][0..9].*, zolt_materials[t] }, .{ jolt_vertices[9 * t ..][0..9].*, jolt_materials[t] });
        }
    }
    try finishAll(&.{ &create_checker, &checker });
}

test "HeightFieldShape parity: CollideSoftBodyVertices" {
    const allocator = std.testing.allocator;
    var table = try MaterialTable.init(allocator);
    defer table.deinit();
    var gen: Gen = .{ .rng = .{ .state = 0x3c6ef372 } };
    var create_checker: Checker = .{ .name = "create" };
    var checker: Checker = .{ .name = "soft body vertices" };
    const n = 16;

    var inputs = try heightFields(allocator, &gen, 60);
    defer freeHeightFields(allocator, &inputs);
    for (inputs.items, 0..) |*input, input_index| {
        var pair = (try build(allocator, &table, input, &create_checker)) orelse continue;
        defer pair.deinit();
        if (walksOutside(pair.hf())) continue;
        const hf = pair.hf();
        for (0..6) |_| {
            const scale: P = if (gen.oneIn(2)) .{ 1, 1, 1 } else gen.hfScale();
            const transform = if (gen.oneIn(3)) Mat44.identity() else gen.transform(3);
            var positions: [n * 3]f32 = undefined;
            var inv_masses: [n]f32 = undefined;
            var penetrations: [n]f32 = undefined;
            const planes: [n * 4]f32 = @splat(0);
            const indices: [n]c_int = @splat(-1);
            for (0..n) |v| {
                const p = transform.mulVec3(surfacePoint(&gen, hf).mul(vec3(scale))).add(vec3(gen.plainVec(-0.3, 0.3)));
                positions[3 * v ..][0..3].* = arr3(p);
                inv_masses[v] = if (gen.oneIn(5)) 0.0 else 1.0;
                penetrations[v] = if (gen.oneIn(2)) -math.flt_max else gen.plain(-0.5, 0.5);
            }
            var jolt_penetrations = penetrations;
            var jolt_planes = planes;
            var jolt_indices = indices;
            jolt.jolt_hf_soft_body(pair.jolt, &arr16(transform), &scale, n, &positions, &inv_masses, &jolt_penetrations, &jolt_planes, &jolt_indices, 5);

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
            hf.asShape().collideSoftBodyVertices(transform, vec3(scale), &vertices, n, 5);
            var zolt_plane_values: [n * 4]f32 = undefined;
            for (0..n) |v| zolt_plane_values[4 * v ..][0..4].* = arr4(zolt_planes[v].normal_and_constant);
            var zolt_index_values: [n]c_int = undefined;
            for (0..n) |v| zolt_index_values[v] = zolt_indices[v];
            checker.check(.{ input_index, scale, positions }, .{ zolt_penetrations, zolt_plane_values, zolt_index_values }, .{ jolt_penetrations, jolt_planes, jolt_indices });
        }
    }
    try finishAll(&.{ &create_checker, &checker });
}

test "HeightFieldShape parity: SetHeights" {
    const allocator = std.testing.allocator;
    var table = try MaterialTable.init(allocator);
    defer table.deinit();
    var gen: Gen = .{ .rng = .{ .state = 0xdeadbeef } };
    var create_checker: Checker = .{ .name = "create" };
    var state_checker: Checker = .{ .name = "state after SetHeights" };
    var heights_checker: Checker = .{ .name = "heights after SetHeights" };
    var ray_checker: Checker = .{ .name = "rays after SetHeights" };

    var inputs = try heightFields(allocator, &gen, 150);
    defer freeHeightFields(allocator, &inputs);
    var temp_allocator = TempAllocatorMalloc.init(allocator);
    for (inputs.items, 0..) |*input, input_index| {
        var pair = (try build(allocator, &table, input, &create_checker)) orelse continue;
        defer pair.deinit();
        if (walksOutside(pair.hf())) continue;
        if (pair.hf().height_samples_size == 0) continue; // SetHeights needs samples
        const count = pair.hf().getSampleCount();
        const block_size = pair.hf().getBlockSize();
        const num_blocks = count / block_size;

        // Several patches on the same height field
        for (0..3) |k| {
            const bx = gen.range(0, num_blocks - 1);
            const by = gen.range(0, num_blocks - 1);
            const nbx = if (k == 2 and gen.oneIn(2)) num_blocks - bx else gen.range(1, num_blocks - bx);
            const nby = gen.range(1, num_blocks - by);
            const x = bx * block_size;
            const y = by * block_size;
            const size_x = nbx * block_size;
            const size_y = nby * block_size;
            const width = size_x + gen.range(0, 2);
            const stride: i64 = if (gen.oneIn(3)) -@as(i64, width) else width;
            const first_row: i64 = if (stride < 0) @as(i64, size_y - 1) * width else 0;
            const heights = try allocator.alloc(f32, @as(usize, width) * size_y);
            defer allocator.free(heights);
            const lo = pair.hf().getMinHeightValue();
            const hi = pair.hf().getMaxHeightValue();
            const base = gen.plain(@min(lo, hi), @max(lo, hi));
            for (heights) |*h| {
                h.* = switch (gen.index(10)) {
                    0 => no_collision_value,
                    1 => lo - 1.0, // Clamped
                    2 => hi + 1.0, // Clamped
                    3 => 1.0e20, // Outside the range of an int while quantizing
                    else => base + gen.plain(-1, 1),
                };
            }
            if (gen.oneIn(5)) @memset(heights, no_collision_value);
            const threshold: f32 = if (gen.oneIn(2)) 0.996195 else gen.plain(0.5, 1);
            jolt.jolt_hf_set_heights(pair.jolt, x, y, size_x, size_y, heights.ptr, first_row, stride, threshold);
            try pair.hfMut().setHeights(x, y, size_x, size_y, heights.ptr + @as(usize, @intCast(first_row)), @intCast(stride), temp_allocator.tempAllocator(), .{ .active_edge_cos_threshold_angle = threshold });
            try checkState(allocator, &state_checker, .{ input_index, k, x, y, size_x, size_y }, &pair);
        }

        // All heights after the update
        const n = @as(usize, count) * count;
        const jolt_heights = try allocator.alloc(f32, n);
        defer allocator.free(jolt_heights);
        jolt.jolt_hf_get_heights(pair.jolt, 0, 0, count, count, jolt_heights.ptr, 0, count);
        const zolt_heights = try allocator.alloc(f32, n);
        defer allocator.free(zolt_heights);
        pair.hf().getHeights(0, 0, count, count, zolt_heights.ptr, count);
        for (zolt_heights, jolt_heights, 0..) |z, j, i| heights_checker.check(.{ input_index, i }, z, j);

        // Queries use the new ranges and samples
        const bounds = pair.hf().asShape().getLocalBounds();
        for (0..30) |_| {
            const target = surfacePoint(&gen, pair.hf());
            const ray_input: RayInput = .{
                .origin = arr3(Vec3.init(target.getX(), bounds.max.getY() + 1, target.getZ())),
                .direction = .{ gen.plain(-0.5, 0.5), -(bounds.getSize().getY() + 2), gen.plain(-0.5, 0.5) },
                .creator = .{ 0, 0 },
                .fraction = 1.0 + math.flt_epsilon,
                .back_face_mode = 1,
                .collector = 0,
                .early_out = math.flt_max,
                .body_id = 1,
                .reject_id = 0xffffffff,
            };
            var jolt_out = std.mem.zeroes(RayOutput);
            jolt.jolt_hf_cast_ray(pair.jolt, &ray_input, &jolt_out);
            ray_checker.check(.{ input_index, ray_input }, try zoltCastRay(allocator, pair.shape(), &ray_input), jolt_out);
        }
    }
    try finishAll(&.{ &create_checker, &state_checker, &heights_checker, &ray_checker });
}

test "HeightFieldShape parity: SetMaterials" {
    const allocator = std.testing.allocator;
    var table = try MaterialTable.init(allocator);
    defer table.deinit();
    var gen: Gen = .{ .rng = .{ .state = 0xfeedface } };
    var create_checker: Checker = .{ .name = "create" };
    var checker: Checker = .{ .name = "set materials" };

    var inputs = try heightFields(allocator, &gen, 150);
    defer freeHeightFields(allocator, &inputs);

    // A height field with 255 materials (adding 3 new ones exceeds 256)
    {
        var full = try gen.heightField(allocator, .{});
        full.materials.clearRetainingCapacity();
        for (0..255) |i| try full.materials.append(allocator, @intCast(i));
        const count_min_1: usize = full.desc.sample_count - 1;
        try full.material_indices.resize(allocator, count_min_1 * count_min_1);
        for (full.material_indices.items) |*m| m.* = @intCast(gen.index(255));
        full.finish();
        try inputs.append(allocator, full);
    }

    var temp_allocator = TempAllocatorMalloc.init(allocator);
    for (inputs.items, 0..) |*input, input_index| {
        var pair = (try build(allocator, &table, input, &create_checker)) orelse continue;
        defer pair.deinit();
        const count = pair.hf().getSampleCount();
        if (pair.hf().getMaterialList().len == 0 and gen.oneIn(2)) continue;

        for (0..4) |k| {
            const x = gen.range(0, count - 2);
            const y = gen.range(0, count - 2);
            const size_x = gen.range(0, count - 1 - x - 1);
            const size_y = gen.range(1, count - 1 - y);

            // The new material list (or null: keep the current list)
            var list: [12]u32 = undefined;
            const current = pair.hf().getMaterialList().len;
            var list_count: c_int = @intCast(gen.range(1, 12));
            if (current > 0 and (k == 3 or gen.oneIn(4))) list_count = -1;
            if (input_index == inputs.items.len - 1) list_count = @intCast(gen.range(3, 12)); // The full height field
            for (list[0..@max(list_count, 0)]) |*m| m.* = @intCast(if (input_index == inputs.items.len - 1) gen.range(255, num_table_materials - 1) else gen.index(20));
            const list_size: usize = if (list_count >= 0) @intCast(list_count) else current;
            if (list_size == 0) continue;

            const width = size_x + gen.range(0, 2);
            const stride: i64 = if (gen.oneIn(3)) -@as(i64, width) else width;
            const first_row: i64 = if (stride < 0) @as(i64, size_y - 1) * width else 0;
            const data = try allocator.alloc(u8, @max(@as(usize, width) * size_y, 1));
            defer allocator.free(data);
            for (data) |*d| d.* = @intCast(gen.index(list_size));

            const jolt_result = jolt.jolt_hf_set_materials(pair.jolt, x, y, size_x, size_y, data.ptr, first_row, stride, &list, list_count);
            var refs: [12]PhysicsMaterialRefC = undefined;
            for (0..@max(list_count, 0)) |i| refs[i] = table.materials[list[i]];
            const zolt_result = try pair.hfMut().setMaterials(x, y, size_x, size_y, data.ptr + @as(usize, @intCast(first_row)), @intCast(stride), if (list_count >= 0) refs[0..@intCast(list_count)] else null, temp_allocator.tempAllocator());
            checker.check(.{ input_index, k, "result" }, @as(c_int, @intFromBool(zolt_result)), jolt_result);

            // The material list and the state
            const props_input: PropertiesInput = .{ .scale = .{ 1, 1, 1 }, .transform = arr16(Mat44.identity()), .translation = .{ 0, 0, 0 } };
            var jolt_props: PropertiesOutput = std.mem.zeroes(PropertiesOutput);
            jolt.jolt_hf_properties(pair.jolt, &props_input, &jolt_props);
            const zolt_props = zoltProperties(&table, pair.hf(), &props_input);
            checker.check(.{ input_index, k, "materials" }, .{ zolt_props.num_materials, zolt_props.materials }, .{ jolt_props.num_materials, jolt_props.materials });
            try checkState(allocator, &checker, .{ input_index, k, x, y, size_x, size_y }, &pair);

            // GetMaterial of every quad
            const quads = @as(usize, count - 1) * (count - 1);
            const jolt_positions = try allocator.alloc(f32, 3 * @as(usize, count) * count);
            defer allocator.free(jolt_positions);
            const jolt_no_collision = try allocator.alloc(c_int, @as(usize, count) * count);
            defer allocator.free(jolt_no_collision);
            const jolt_materials = try allocator.alloc(u32, quads);
            defer allocator.free(jolt_materials);
            jolt.jolt_hf_samples(pair.jolt, jolt_positions.ptr, jolt_no_collision.ptr, jolt_materials.ptr);
            for (0..count - 1) |qy| for (0..count - 1) |qx| {
                checker.check(.{ input_index, k, qx, qy }, table.indexOf(pair.hf().getMaterialAt(@intCast(qx), @intCast(qy))), jolt_materials[qy * (count - 1) + qx]);
            };
        }
    }
    try finishAll(&.{ &create_checker, &checker });
}
