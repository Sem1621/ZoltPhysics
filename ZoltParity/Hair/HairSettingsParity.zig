//! The "HairSettings" layer of the parity (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, R14): Zolt's
//! HairSettings against Jolt's. C ABI wrappers: HairSettingsReference.cpp.
//!
//! The inputs are built here and handed to both sides as bytes: the settings before the groom as their SaveBinaryState
//! bytes (the C++ side restores them), the groom as raw SVertex / SStrand arrays (the layouts are Jolt's).
//! - Scalps: the face of Assets/face.bin (made relative to the neck bind pose like Jolt's HairTest sample, with its
//!   inverse bind pose and 3 skin weights per vertex), or none. The face cases are skipped when the asset is missing.
//! - Procedural grooms: straight and curly strands of 2 to 30 vertices (so the compute buffers have padding vertices)
//!   on the scalp, a few roots off the surface, 3 materials with simulation fractions below 1 (one with a gravity
//!   preload, which runs the Gauss-Seidel solve of Init, one that only uses the global pose), random grooms with random
//!   settings, an empty groom.
//! - Scalps with the skin weights or the inverse bind pose removed: InitCompute then expects the skinned scalp vertices
//!   from outside (no scalp vertex and skin weight buffers). An empty groom on the scalp (no skin points).
//! - Per case: the SaveBinaryState bytes after InitRenderAndSimulationStrands and after Init, Init's result, the bytes of
//!   the 14 compute buffers of InitCompute (with the CPU compute system) and, when the groom is resampled, the
//!   resampled groom. Jolt's quirks are reproduced and compared too: InitRenderAndSimulationStrands appends (called
//!   twice), Init twice without a scalp (the neutral density accumulates), a fraction above 1 on a material that is not
//!   the last (the simulated strands run into the next material).
//! - Per case, the state after Init on the other side: Zolt restores and saves Jolt's bytes, Jolt restores and saves
//!   Zolt's. The skin points, the bounds with their W lanes and the neutral density go through RestoreBinaryState.
//! - Jolt's release build on inputs that Jolt asserts on: simulated strands of 1 vertex after longer strands (Init reads
//!   the x[1] of the last longer strand). The settings hold the simulated strands directly, so the case restores them
//!   and skips InitRenderAndSimulationStrands. Zolt's asserts are recoverable and counted (ParityHooks.ReleaseAsserts).
//! - Resample of strands of 0 and 1 vertices inside the vertex array (literal, without an assert): only the resampled
//!   groom is compared, its strands have rods of zero length.
//! - SkinScalpVertices and PrepareForScalpSkinning with frames of the face animation.
//! - The small functions: Gradient::MakeStepDependent (std::pow, the pin of R13), GradientSampler,
//!   Material::GetBendCompliance, NeedsGrid and GlobalPoseOnly on random inputs.
//!
//! Every comparison is bit for bit. The layer runs in both precisions (HairSettings has no RVec3, so the bytes are the
//! same). One exception is modeled: the W lanes of mSimulationBounds after Init, where Jolt keeps a stale W of the
//! restored padding and Zolt keeps W = Z (joltBoundsW).

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const ParityHooks = @import("../ParityHooks.zig");

const HairSettings = zolt.HairSettings;
const SVertex = HairSettings.SVertex;
const SStrand = HairSettings.SStrand;
const Material = HairSettings.Material;
const Gradient = HairSettings.Gradient;
const Float3 = zolt.Float3;
const Float4 = zolt.Float4;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const UVec4 = zolt.UVec4;
const IndexedTriangleNoMaterial = zolt.IndexedTriangleNoMaterial;
const ComputeBuffer = zolt.ComputeBuffer;
const trigonometry = zolt.trigonometry;

extern fn jolt_hair_result(index: c_int, size: *usize) [*]const u8;
extern fn jolt_hair_settings_run(state: [*]const u8, state_size: usize, vertices: [*]const SVertex, num_vertices: u32, strands: [*]const SStrand, num_strands: u32, resample: u32, flags: u32, max_dist_sq: *f32, buffer_present: *[num_buffers]c_int) c_int;
extern fn jolt_hair_resave(state: [*]const u8, state_size: usize) c_int;
extern fn jolt_hair_skin_scalp(state: [*]const u8, state_size: usize, joint_to_hair: *const [16]f32, joint_matrices: [*]const f32, num_joints: u32) c_int;
extern fn jolt_hair_gradient(gradient: *const [4]f32, time_ratio: f32, fraction: f32, start: u32, end: u32, vertex: u32, multiplier: *const [4]f32, compliance: f32, density_force: f32, enable_collision: c_int, values: *[13]f32) void;

fn cppResult(index: c_int) []const u8 {
    var size: usize = 0;
    const data = jolt_hair_result(index, &size);
    return data[0..size];
}

/// Flags of a run (jolt_hair_settings_run)
const init_render_twice: u32 = 1;
const init_twice: u32 = 2;
const skip_init_render: u32 = 4;
const resample_only: u32 = 8;

/// The compute buffers of InitCompute, in the order of HairSettingsReference.cpp
const buffer_fields = [_][]const u8{
    "scalp_vertices_cb",  "scalp_triangles_cb",          "scalp_skin_weights_cb",   "skin_points_cb",
    "vertices_fixed_cb",  "vertices_position_cb",        "vertices_bishop_cb",      "vertices_omega0_cb",
    "vertices_length_cb", "vertices_strand_fraction_cb", "strand_vertex_counts_cb", "strand_material_index_cb",
    "neutral_density_cb", "s_vertex_influences_cb",
};
const num_buffers = buffer_fields.len;

// The face of Assets/face.bin ----------------------------------------------------------------------------------------

pub const Face = struct {
    vertices: []Float3,
    triangles: []IndexedTriangleNoMaterial,
    inverse_bind_pose: []Mat44,
    skin_weights: []HairSettings.SkinWeight,
    num_skin_weights_per_vertex: u32,
    /// The joint matrices of 3 frames of the animation
    frames: [3][]Mat44,
    /// The inverse of the neck transform of frame 0 (Jolt's HairTest: inv_neck_transform)
    inverse_neck: Mat44,
    /// The index of the neck joint (Jolt's HairTest: mHeadJointIdx)
    head_joint: u32,

    pub fn deinit(self: *Face, allocator: Allocator) void {
        allocator.free(self.vertices);
        allocator.free(self.triangles);
        allocator.free(self.inverse_bind_pose);
        allocator.free(self.skin_weights);
        for (self.frames) |f| allocator.free(f);
    }
};

/// Read Assets/face.bin like Jolt's HairTest sample and make the mesh relative to the neck bind pose. Null when the
/// asset is missing.
pub fn loadFace(allocator: Allocator) !?Face {
    const bytes = std.Io.Dir.cwd().readFileAlloc(std.testing.io, "Assets/face.bin", allocator, .limited(16 << 20)) catch |err| switch (err) {
        error.FileNotFound => return null,
        else => |e| return e,
    };
    defer allocator.free(bytes);
    var reader: std.Io.Reader = .fixed(bytes);
    const head_joint = try reader.takeInt(u32, .little);

    const num_vertices = try reader.takeInt(u32, .little);
    const vertices = try allocator.alloc(Float3, num_vertices);
    errdefer allocator.free(vertices);
    try reader.readSliceAll(std.mem.sliceAsBytes(vertices));

    const num_triangles = try reader.takeInt(u32, .little);
    const triangles = try allocator.alloc(IndexedTriangleNoMaterial, num_triangles);
    errdefer allocator.free(triangles);
    try reader.readSliceAll(std.mem.sliceAsBytes(triangles));

    const num_joints = try reader.takeInt(u32, .little);
    const inverse_bind_pose = try allocator.alloc(Mat44, num_joints);
    errdefer allocator.free(inverse_bind_pose);
    for (inverse_bind_pose) |*m| {
        var columns: [4]Float4 = undefined;
        try reader.readSliceAll(std.mem.asBytes(&columns));
        m.* = Mat44.loadFloat4x4(&columns);
    }

    const num_weights = try reader.takeInt(u32, .little);
    const skin_weights = try allocator.alloc(HairSettings.SkinWeight, num_weights * num_vertices);
    errdefer allocator.free(skin_weights);
    try reader.readSliceAll(std.mem.sliceAsBytes(skin_weights));

    // Animation: 3 frames of the joint matrices
    const num_frames = try reader.takeInt(u32, .little);
    var frames: [3][]Mat44 = .{ &.{}, &.{}, &.{} };
    errdefer for (frames) |f| allocator.free(f);
    const wanted = [3]u32{ 0, num_frames / 3, num_frames - 1 };
    for (0..num_frames) |frame| {
        for (0..num_joints) |joint| {
            var tr: [6]f32 = undefined;
            try reader.readSliceAll(std.mem.asBytes(&tr));
            for (wanted, 0..) |w, slot| {
                if (w != frame) continue;
                if (frames[slot].len == 0) frames[slot] = try allocator.alloc(Mat44, num_joints);
                const xyz = Vec3.init(tr[3], tr[4], tr[5]);
                const rotation = Quat.init(tr[3], tr[4], tr[5], @sqrt(zolt.math.max(@as(f32, 0.0), 1.0 - xyz.lengthSq())));
                frames[slot][joint] = Mat44.rotationTranslation(rotation, Vec3.init(tr[0], tr[1], tr[2]));
            }
        }
    }

    // Make mesh relative to neck bind pose
    const inverse_bind_neck = inverse_bind_pose[head_joint];
    const bind_neck = inverse_bind_neck.inversed();
    for (vertices) |*v| inverse_bind_neck.mulVec3(Vec3.fromFloat3(v.*)).storeFloat3(v);
    for (inverse_bind_pose) |*m| m.* = m.mul(bind_neck);

    return .{
        .vertices = vertices,
        .triangles = triangles,
        .inverse_bind_pose = inverse_bind_pose,
        .skin_weights = skin_weights,
        .num_skin_weights_per_vertex = num_weights,
        .frames = frames,
        .inverse_neck = frames[0][head_joint].inversed(),
        .head_joint = head_joint,
    };
}

// The inputs ---------------------------------------------------------------------------------------------------------

pub const Groom = struct {
    vertices: std.ArrayList(SVertex) = .empty,
    strands: std.ArrayList(SStrand) = .empty,

    pub fn deinit(self: *Groom, allocator: Allocator) void {
        self.vertices.deinit(allocator);
        self.strands.deinit(allocator);
    }
};

pub const Shape = enum { straight, curly };

/// How the strands get their material: random, or in blocks of strands (the first block material 0, ...)
pub const MaterialOrder = union(enum) {
    random: u32,
    blocks: []const u32,

    fn materialOf(self: MaterialOrder, rng: *fw.Rng, strand: usize) u32 {
        switch (self) {
            .random => |num_materials| return rng.next() % num_materials,
            .blocks => |blocks| {
                var end: usize = 0;
                for (blocks, 0..) |count, m| {
                    end += count;
                    if (strand < end) return @intCast(m);
                }
                unreachable;
            },
        }
    }
};

/// Strands on the scalp (or on a sphere without a scalp): `num_strands` strands of `min_vertices` to `max_vertices`
/// vertices
pub fn makeGroom(allocator: Allocator, rng: *fw.Rng, face: ?*const Face, shape: Shape, num_strands: u32, min_vertices: u32, max_vertices: u32, materials: MaterialOrder) !Groom {
    var groom: Groom = .{};
    errdefer groom.deinit(allocator);
    for (0..num_strands) |s| {
        // Root and direction
        var root: Vec3 = undefined;
        var dir: Vec3 = undefined;
        if (face) |f| {
            const t = f.triangles[@as(usize, rng.next()) % f.triangles.len];
            const v0 = Vec3.fromFloat3(f.vertices[t.idx[0]]);
            const v1 = Vec3.fromFloat3(f.vertices[t.idx[1]]);
            const v2 = Vec3.fromFloat3(f.vertices[t.idx[2]]);
            root = v0.add(v1).add(v2).mulScalar(1.0 / 3.0);
            dir = v1.sub(v0).cross(v2.sub(v0)).normalizedOr(Vec3.axisY());
            // A few roots off the surface: Init projects them onto the scalp
            if (s % 7 == 3) root = root.add(dir.mulScalar(rng.float(-0.01, 0.01)));
        } else {
            dir = Vec3.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1)).normalizedOr(Vec3.axisY());
            root = dir.mulScalar(0.1);
        }
        dir = dir.add(Vec3.init(0, -0.5, 0)).normalizedOr(Vec3.axisY());
        const perpendicular = dir.getNormalizedPerpendicular();
        const binormal = dir.cross(perpendicular);

        const num_vertices = min_vertices + rng.next() % (max_vertices - min_vertices + 1);
        const segment = rng.float(0.005, 0.02);
        const radius = rng.float(0.002, 0.01);
        const turn = rng.float(0.5, 1.5);
        const first: u32 = @intCast(groom.vertices.items.len);
        for (0..num_vertices) |i| {
            const fi: f32 = @floatFromInt(i);
            var p = root.add(dir.mulScalar(segment * fi));
            if (shape == .curly)
                p = p.add(perpendicular.mulScalar(radius * (trigonometry.cos(turn * fi) - 1.0))).add(binormal.mulScalar(radius * trigonometry.sin(turn * fi)));
            // Some jitter so that no two rods are parallel
            p = p.add(Vec3.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1)).mulScalar(0.1 * segment));
            var v: SVertex = .{};
            p.storeFloat3(&v.position);
            v.inv_mass = if (i == 0 or (i == 3 and s % 11 == 5)) 0.0 else 1.0;
            try groom.vertices.append(allocator, v);
        }
        try groom.strands.append(allocator, .init(first, first + num_vertices, materials.materialOf(rng, s)));
    }
    return groom;
}

/// Cut strand s of the groom to `keep(s, number of strands)` vertices (null: all of them). The vertices of the cut strands
/// are removed.
fn cutStrands(allocator: Allocator, groom: *Groom, keep: *const fn (strand: usize, num_strands: usize) ?u32) !void {
    var vertices: std.ArrayList(SVertex) = .empty;
    errdefer vertices.deinit(allocator);
    const num_strands = groom.strands.items.len;
    for (groom.strands.items, 0..) |*strand, s| {
        const count = strand.vertexCount();
        const new_count = if (keep(s, num_strands)) |k| @min(k, count) else count;
        const first: u32 = @intCast(vertices.items.len);
        try vertices.appendSlice(allocator, groom.vertices.items[strand.start_vtx..][0..new_count]);
        strand.start_vtx = first;
        strand.end_vtx = first + new_count;
    }
    groom.vertices.deinit(allocator);
    groom.vertices = vertices;
}

/// The 3 materials of the grooms: simulation fractions below 1, a gravity preload, a material that only uses the
/// global pose
fn materials3(fractions: [3]f32) [3]Material {
    var m: [3]Material = .{ .{}, .{}, .{} };
    m[0].simulation_strands_fraction = fractions[0];
    m[1].simulation_strands_fraction = fractions[1];
    m[1].gravity_preload_factor = 1.0;
    m[1].bend_compliance = 1.0e-5;
    m[1].bend_compliance_multiplier = .{ .x = 2.0, .y = 50.0, .z = 0.5, .w = 3.0 };
    m[1].stretch_compliance = 1.0e-7;
    m[1].gravity_factor = .init(0.3, 0.9, .{ .min_fraction = 0.1, .max_fraction = 0.7 });
    m[2].simulation_strands_fraction = fractions[2];
    m[2].enable_collision = false;
    m[2].global_pose = .init(1.0, 1.0, .{});
    m[2].gravity_preload_factor = 0.5;
    return m;
}

/// The settings before the groom: the scalp (if any), the materials and the fields
pub fn baseSettings(allocator: Allocator, face: ?*const Face, materials: []const Material) !HairSettings {
    var settings: HairSettings = .{ .allocator = allocator };
    errdefer settings.deinit();
    if (face) |f| {
        try settings.scalp_vertices.appendSlice(allocator, f.vertices);
        try settings.scalp_triangles.appendSlice(allocator, f.triangles);
        try settings.scalp_inverse_bind_pose.appendSlice(allocator, f.inverse_bind_pose);
        try settings.scalp_skin_weights.appendSlice(allocator, f.skin_weights);
        settings.scalp_num_skin_weights_per_vertex = f.num_skin_weights_per_vertex;
        settings.initial_gravity = f.inverse_bind_pose[0].multiply3x3(Vec3.init(0, -9.81, 0));
    }
    try settings.materials.appendSlice(allocator, materials);
    return settings;
}

pub fn saveState(allocator: Allocator, settings: *const HairSettings) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(allocator);
    errdefer out.deinit();
    var wrapper = zolt.StreamOutWrapper.init(&out.writer);
    settings.saveBinaryState(wrapper.streamOut());
    return out.toOwnedSlice();
}

// Running a case ---------------------------------------------------------------------------------------------------

const Case = struct {
    name: []const u8,
    /// The SaveBinaryState bytes of the settings before the groom
    state: []const u8,
    groom: *const Groom,
    resample: u32 = 0,
    flags: u32 = 0,
};

/// Compare two byte strings, print the first difference
fn expectSameBytes(case: []const u8, what: []const u8, zolt_bytes: []const u8, jolt_bytes: []const u8) !void {
    if (std.mem.eql(u8, zolt_bytes, jolt_bytes)) return;
    const at = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes) orelse 0;
    std.debug.print("HairSettings parity, {s}: {s} differs at byte {d} (zolt {d} bytes, jolt {d} bytes)\n", .{ case, what, at, zolt_bytes.len, jolt_bytes.len });
    const from = at -| 16;
    std.debug.print("  zolt: {x}\n  jolt: {x}\n", .{ zolt_bytes[from..@min(zolt_bytes.len, at + 32)], jolt_bytes[from..@min(jolt_bytes.len, at + 32)] });
    return error.ParityMismatch;
}

/// The W lanes of mSimulationBounds in Jolt's state after Init, written into Zolt's state bytes.
///
/// The reference restores the settings with RestoreBinaryState. StreamIn::Read(Vec3) reads 12 bytes into
/// mSimulationBoundsPadding and calls Vec3::sFixW. sFixW does nothing unless JPH_FLOATING_POINT_EXCEPTIONS_ENABLED is
/// defined, and Jolt's CMake defines it for MSVC only. So the padding keeps the W of the constructor's
/// Vec3::sReplicate(0.1f). Init expands mSimulationBounds by the padding, and SaveBinaryState writes the raw AABox with
/// its W lanes. Zolt's Vec3 keeps W = Z (porting guide section 8, rule 9), like Jolt with floating point exceptions
/// enabled. Jolt's W lanes are the Z of the bounds of the simulated vertices minus / plus 0.1. This writes them, so the
/// rest of the bytes are still compared bit for bit.
fn joltBoundsW(settings: *const HairSettings, state: []u8) void {
    // The state ends with mSimulationBounds (32 bytes), mNeutralDensity (a 32-bit count and the floats), mDensityScale
    // and mMaxVerticesPerStrand
    const offset = state.len - 4 - 4 - (4 + 4 * settings.neutral_density.items.len) - @sizeOf(zolt.AABox);
    std.debug.assert(std.mem.eql(u8, state[offset..][0..@sizeOf(zolt.AABox)], std.mem.asBytes(&settings.simulation_bounds)));
    var bounds: zolt.AABox = .empty;
    for (settings.sim_vertices.items) |v| bounds.encapsulateVec3(Vec3.fromFloat3(v.position));
    const stale_w: f32 = 0.1;
    const min_w: f32 = bounds.min.getZ() - stale_w;
    const max_w: f32 = bounds.max.getZ() + stale_w;
    @memcpy(state[offset + 12 ..][0..4], std.mem.asBytes(&min_w));
    @memcpy(state[offset + 28 ..][0..4], std.mem.asBytes(&max_w));
}

/// Run a case on both sides and compare every result
fn runCase(allocator: Allocator, case: Case) !void {
    // Jolt
    var jolt_max_dist_sq: f32 = 0;
    var present: [num_buffers]c_int = undefined;
    try std.testing.expectEqual(@as(c_int, 0), jolt_hair_settings_run(case.state.ptr, case.state.len, case.groom.vertices.items.ptr, @intCast(case.groom.vertices.items.len), case.groom.strands.items.ptr, @intCast(case.groom.strands.items.len), case.resample, case.flags, &jolt_max_dist_sq, &present));

    // Zolt
    var settings: HairSettings = .{ .allocator = allocator };
    defer settings.deinit();
    var reader: std.Io.Reader = .fixed(case.state);
    var in = zolt.StreamInWrapper.init(&reader);
    try settings.restoreBinaryState(in.streamIn());

    var vertices: std.ArrayList(SVertex) = .empty;
    defer vertices.deinit(allocator);
    try vertices.appendSlice(allocator, case.groom.vertices.items);
    var strands: std.ArrayList(SStrand) = .empty;
    defer strands.deinit(allocator);
    try strands.appendSlice(allocator, case.groom.strands.items);
    if (case.resample > 0) {
        try HairSettings.resample(allocator, &vertices, &strands, case.resample);
        try expectSameBytes(case.name, "the resampled vertices", std.mem.sliceAsBytes(vertices.items), cppResult(16));
        try expectSameBytes(case.name, "the resampled strands", std.mem.sliceAsBytes(strands.items), cppResult(17));
    }
    if (case.flags & resample_only != 0) return;

    if (case.flags & skip_init_render == 0) {
        try settings.initRenderAndSimulationStrands(vertices.items, strands.items);
        if (case.flags & init_render_twice != 0)
            try settings.initRenderAndSimulationStrands(vertices.items, strands.items);
    }
    const state_a = try saveState(allocator, &settings);
    defer allocator.free(state_a);
    try expectSameBytes(case.name, "SaveBinaryState after InitRenderAndSimulationStrands", state_a, cppResult(0));

    var max_dist_sq = try settings.init();
    if (case.flags & init_twice != 0)
        max_dist_sq = try settings.init();
    const zolt_state_b = try saveState(allocator, &settings);
    defer allocator.free(zolt_state_b);
    const state_b = try allocator.dupe(u8, zolt_state_b);
    defer allocator.free(state_b);
    joltBoundsW(&settings, state_b);
    try expectSameBytes(case.name, "SaveBinaryState after Init", state_b, cppResult(1));
    if (!fw.sameBits(max_dist_sq, jolt_max_dist_sq)) {
        std.debug.print("HairSettings parity, {s}: Init gives {e}, Jolt {e}\n", .{ case.name, max_dist_sq, jolt_max_dist_sq });
        return error.ParityMismatch;
    }

    var system = try zolt.ComputeSystemCPU.create(allocator);
    defer system.deinit();
    try settings.initCompute(system.get().?);
    inline for (buffer_fields, 0..) |field, i| {
        const buffer: ?*ComputeBuffer = @field(settings, field).get();
        if ((buffer != null) != (present[i] != 0)) {
            std.debug.print("HairSettings parity, {s}: InitCompute {s} {s} in Zolt, {s} in Jolt\n", .{ case.name, field, if (buffer != null) "exists" else "is missing", if (present[i] != 0) "exists" else "is missing" });
            return error.ParityMismatch;
        }
        if (buffer) |b| {
            const bytes = b.map(.read);
            defer b.unmap();
            try expectSameBytes(case.name, "InitCompute " ++ field, bytes, cppResult(2 + i));
        }
    }

    // The state after Init on the other side: Zolt restores and saves Jolt's bytes, Jolt restores and saves Zolt's.
    // RestoreBinaryState reads the bounds as a raw AABox, so the W lanes go through unchanged.
    {
        var restored: HairSettings = .{ .allocator = allocator };
        defer restored.deinit();
        var jolt_reader: std.Io.Reader = .fixed(cppResult(1));
        var jolt_in = zolt.StreamInWrapper.init(&jolt_reader);
        try restored.restoreBinaryState(jolt_in.streamIn());
        const resaved = try saveState(allocator, &restored);
        defer allocator.free(resaved);
        try expectSameBytes(case.name, "Jolt's state after Init, restored and saved by Zolt", resaved, cppResult(1));
    }
    try std.testing.expectEqual(@as(c_int, 0), jolt_hair_resave(zolt_state_b.ptr, zolt_state_b.len));
    try expectSameBytes(case.name, "Zolt's state after Init, restored and saved by Jolt", cppResult(18), zolt_state_b);
}

// The layer --------------------------------------------------------------------------------------------------------

test "HairSettings parity: grooms on the scalp of Assets/face.bin" {
    const allocator = std.testing.allocator;
    var face = (try loadFace(allocator)) orelse return error.SkipZigTest;
    defer face.deinit(allocator);
    var rng: fw.Rng = .{ .state = 0x3a1f_0c55 };

    // What the scalp keeps: everything, or no skin weights or no inverse bind pose (the skinned scalp vertices then come
    // from outside, InitCompute makes no scalp vertex and skin weight buffers)
    const Scalp = enum { skinned, no_skin_weights, no_inverse_bind_pose };
    const Setup = struct { name: []const u8, shape: Shape, num_strands: u32, min_vertices: u32, max_vertices: u32, fractions: [3]f32, resample: u32 = 0, flags: u32 = 0, blocks: ?[]const u32 = null, scalp: Scalp = .skinned };
    const setups = [_]Setup{
        .{ .name = "face straight", .shape = .straight, .num_strands = 300, .min_vertices = 8, .max_vertices = 24, .fractions = .{ 0.1, 0.5, 0.3 } },
        .{ .name = "face curly", .shape = .curly, .num_strands = 200, .min_vertices = 12, .max_vertices = 30, .fractions = .{ 0.25, 0.05, 0.75 } },
        .{ .name = "face curly resampled", .shape = .curly, .num_strands = 120, .min_vertices = 12, .max_vertices = 30, .fractions = .{ 0.2, 0.4, 0.6 }, .resample = 16 },
        .{ .name = "face short strands", .shape = .straight, .num_strands = 150, .min_vertices = 2, .max_vertices = 5, .fractions = .{ 0.5, 0.0, 0.9 } },
        .{ .name = "face init render twice", .shape = .straight, .num_strands = 80, .min_vertices = 6, .max_vertices = 12, .fractions = .{ 0.3, 0.3, 0.3 }, .flags = init_render_twice },
        // The simulated strands run into the next material twice (a fraction above 1 that is not on the last run, R20 UB8:
        // literal). Material 0 (30 strands) simulates 45 strands. Material 1 (40 strands from position 30) simulates 48.
        // Material 2 (50 strands from position 70) simulates 25.
        .{ .name = "face fraction above 1 before the last material", .shape = .straight, .num_strands = 120, .min_vertices = 6, .max_vertices = 10, .fractions = .{ 1.5, 1.2, 0.5 }, .blocks = &.{ 30, 40, 50 } },
        .{ .name = "face skinned outside (no skin weights)", .shape = .curly, .num_strands = 90, .min_vertices = 4, .max_vertices = 14, .fractions = .{ 0.3, 0.6, 0.2 }, .scalp = .no_skin_weights },
        .{ .name = "face skinned outside (no inverse bind pose)", .shape = .straight, .num_strands = 70, .min_vertices = 3, .max_vertices = 9, .fractions = .{ 0.5, 0.25, 0.4 }, .scalp = .no_inverse_bind_pose },
        .{ .name = "face empty groom", .shape = .straight, .num_strands = 0, .min_vertices = 2, .max_vertices = 2, .fractions = .{ 0.1, 0.1, 0.1 } },
    };
    for (setups) |setup| {
        const materials = materials3(setup.fractions);
        var settings = try baseSettings(allocator, &face, &materials);
        defer settings.deinit();
        switch (setup.scalp) {
            .skinned => {},
            .no_skin_weights => settings.scalp_skin_weights.clearRetainingCapacity(),
            .no_inverse_bind_pose => settings.scalp_inverse_bind_pose.clearRetainingCapacity(),
        }
        settings.grid_size = UVec4.init(16, 24, 20, 0);
        const state = try saveState(allocator, &settings);
        defer allocator.free(state);
        var groom = try makeGroom(allocator, &rng, &face, setup.shape, setup.num_strands, setup.min_vertices, setup.max_vertices, if (setup.blocks) |b| .{ .blocks = b } else .{ .random = 3 });
        defer groom.deinit(allocator);
        try runCase(allocator, .{ .name = setup.name, .state = state, .groom = &groom, .resample = setup.resample, .flags = setup.flags });
    }
}

test "HairSettings parity: grooms without a scalp" {
    const allocator = std.testing.allocator;
    var rng: fw.Rng = .{ .state = 0x77e1_2d09 };

    const Setup = struct { name: []const u8, shape: Shape, num_strands: u32, min_vertices: u32, max_vertices: u32, fractions: [3]f32, resample: u32 = 0, flags: u32 = 0 };
    const setups = [_]Setup{
        .{ .name = "straight", .shape = .straight, .num_strands = 200, .min_vertices = 4, .max_vertices = 20, .fractions = .{ 0.1, 0.5, 0.3 } },
        .{ .name = "curly", .shape = .curly, .num_strands = 150, .min_vertices = 10, .max_vertices = 30, .fractions = .{ 0.3, 0.6, 0.15 } },
        .{ .name = "curly resampled", .shape = .curly, .num_strands = 100, .min_vertices = 10, .max_vertices = 30, .fractions = .{ 0.5, 0.5, 0.5 }, .resample = 7 },
        .{ .name = "init twice", .shape = .straight, .num_strands = 60, .min_vertices = 4, .max_vertices = 9, .fractions = .{ 0.4, 0.4, 0.4 }, .flags = init_twice },
        .{ .name = "init render twice", .shape = .curly, .num_strands = 50, .min_vertices = 5, .max_vertices = 9, .fractions = .{ 0.2, 0.7, 0.9 }, .flags = init_render_twice },
        .{ .name = "fractions 0 and 1", .shape = .straight, .num_strands = 70, .min_vertices = 3, .max_vertices = 8, .fractions = .{ 0.0, 1.0, 0.0 } },
    };
    for (setups) |setup| {
        const materials = materials3(setup.fractions);
        var settings = try baseSettings(allocator, null, &materials);
        defer settings.deinit();
        const state = try saveState(allocator, &settings);
        defer allocator.free(state);
        var groom = try makeGroom(allocator, &rng, null, setup.shape, setup.num_strands, setup.min_vertices, setup.max_vertices, .{ .random = 3 });
        defer groom.deinit(allocator);
        try runCase(allocator, .{ .name = setup.name, .state = state, .groom = &groom, .resample = setup.resample, .flags = setup.flags });
    }

    // An empty groom: no strands, empty compute buffers
    {
        const materials = materials3(.{ 0.1, 0.1, 0.1 });
        var settings = try baseSettings(allocator, null, &materials);
        defer settings.deinit();
        const state = try saveState(allocator, &settings);
        defer allocator.free(state);
        var groom: Groom = .{};
        try runCase(allocator, .{ .name = "empty", .state = state, .groom = &groom });
    }
}

test "HairSettings parity: random grooms and settings" {
    const allocator = std.testing.allocator;
    var face_or_null = try loadFace(allocator);
    defer if (face_or_null) |*f| f.deinit(allocator);
    var rng: fw.Rng = .{ .state = 0x1dea_5eed };
    for (0..24) |round| {
        const with_face = face_or_null != null and round % 2 == 0;
        const face: ?*const Face = if (with_face) &face_or_null.? else null;

        // Random materials
        const num_materials = 1 + rng.next() % 4;
        var materials: [4]Material = undefined;
        for (materials[0..num_materials], 0..) |*m, i| {
            m.* = .{};
            m.simulation_strands_fraction = if (i % 3 == 2) 1.0 else rng.float(0.0, 1.0); // Never above 1: a material can be the last run
            m.gravity_preload_factor = if (rng.next() % 2 == 0) 0.0 else rng.float(0.0, 2.0);
            m.bend_compliance = rng.float(1.0e-9, 1.0e-4);
            m.stretch_compliance = rng.float(0.0, 1.0e-6);
            m.bend_compliance_multiplier = .{ .x = rng.float(0, 10), .y = rng.float(0, 100), .z = rng.float(0, 100), .w = rng.float(0, 10) };
            m.gravity_factor = .init(rng.float(0, 1), rng.float(0, 1), .{ .min_fraction = rng.float(0, 0.5), .max_fraction = rng.float(0.5, 1) });
        }
        var settings = try baseSettings(allocator, face, materials[0..num_materials]);
        defer settings.deinit();
        settings.grid_size = UVec4.init(2 + rng.next() % 30, 2 + rng.next() % 30, 2 + rng.next() % 30, rng.next());
        settings.simulation_bounds_padding = Vec3.init(rng.float(0.0, 0.3), rng.float(0.0, 0.3), rng.float(0.0, 0.3));
        settings.initial_gravity = Vec3.init(rng.float(-10, 10), rng.float(-10, 10), rng.float(-10, 10));
        settings.num_iterations_per_second = 30 + rng.next() % 1000;
        settings.max_delta_time = rng.float(0.001, 0.1);
        const state = try saveState(allocator, &settings);
        defer allocator.free(state);

        const shape: Shape = if (rng.next() % 2 == 0) .straight else .curly;
        const min_vertices = 2 + rng.next() % 6;
        var groom = try makeGroom(allocator, &rng, face, shape, 10 + rng.next() % 150, min_vertices, min_vertices + rng.next() % 25, .{ .random = num_materials });
        defer groom.deinit(allocator);
        const resample: u32 = if (rng.next() % 3 == 0) 2 + rng.next() % 20 else 0;
        const name = try std.fmt.allocPrint(allocator, "random {d}", .{round});
        defer allocator.free(name);
        try runCase(allocator, .{ .name = name, .state = state, .groom = &groom, .resample = resample });
    }
}

test "HairSettings parity: simulated strands of 1 vertex after longer strands (Jolt's release build)" {
    // Init reads x[1] past the size of x for a strand of 1 vertex (an assert of Jolt's Array). Jolt's release build reads
    // the x[1] of the last strand of 2 or more vertices. The settings hold the simulated strands directly: the first one
    // is longer, and no strand of 1 vertex starts at vertex 0 (there Jolt is undefined and Zolt panics). Zolt's asserts
    // are recoverable and counted, with -Dreference_asserts=true Jolt's too.
    const allocator = std.testing.allocator;
    var face_or_null = try loadFace(allocator);
    defer if (face_or_null) |*f| f.deinit(allocator);
    var rng: fw.Rng = .{ .state = 0x51e6_1e00 };
    var asserts: ParityHooks.ReleaseAsserts = undefined;
    asserts.init("HairSettings parity, strands of 1 vertex");
    defer asserts.deinit();
    for (0..4) |round| {
        const face: ?*const Face = if (face_or_null != null and round % 2 == 0) &face_or_null.? else null;
        const materials = materials3(.{ 1.0, 1.0, 1.0 });
        var settings = try baseSettings(allocator, face, &materials);
        defer settings.deinit();
        var groom = try makeGroom(allocator, &rng, face, if (round < 2) .straight else .curly, 60, 2, 12, .{ .random = 3 });
        defer groom.deinit(allocator);
        // Single strands and pairs of strands of 1 vertex, never the first strand
        try cutStrands(allocator, &groom, struct {
            fn keep(s: usize, _: usize) ?u32 {
                return if (s != 0 and (s % 5 == 2 or s % 9 == 4 or s % 9 == 5)) 1 else null;
            }
        }.keep);
        try settings.sim_vertices.appendSlice(allocator, groom.vertices.items);
        try settings.sim_strands.appendSlice(allocator, groom.strands.items);
        const state = try saveState(allocator, &settings);
        defer allocator.free(state);

        const name = try std.fmt.allocPrint(allocator, "strands of 1 vertex {d}", .{round});
        defer allocator.free(name);
        var no_groom: Groom = .{};
        asserts.begin();
        try runCase(allocator, .{ .name = name, .state = state, .groom = &no_groom, .flags = skip_init_render });
        asserts.end(round);
    }
    try asserts.finish();
}

test "HairSettings parity: resample of strands of 0 and 1 vertices inside the vertex array" {
    // Jolt reads the next vertex of the array with a fraction of 0, without an assert. Only the resampled groom is
    // compared: its strands of 0 and 1 vertices become rods of zero length. A strand of 0 or 1 vertices at the end of
    // the array, or of 0 vertices at its start, makes Jolt read past the array (Zolt panics): the first and the last
    // strand keep their vertices.
    const allocator = std.testing.allocator;
    var rng: fw.Rng = .{ .state = 0x0e5a_3b1e };
    for ([_]u32{ 2, 5, 12 }) |num_vertices| {
        const materials = materials3(.{ 0.5, 0.5, 0.5 });
        var settings = try baseSettings(allocator, null, &materials);
        defer settings.deinit();
        const state = try saveState(allocator, &settings);
        defer allocator.free(state);
        var groom = try makeGroom(allocator, &rng, null, .curly, 40, 2, 10, .{ .random = 3 });
        defer groom.deinit(allocator);
        try cutStrands(allocator, &groom, struct {
            fn keep(s: usize, n: usize) ?u32 {
                if (s == 0 or s == n - 1) return null;
                return if (s % 6 == 1 or s % 13 == 2) 0 else if (s % 4 == 3) 1 else null;
            }
        }.keep);
        const name = try std.fmt.allocPrint(allocator, "resample strands of 0 and 1 vertices to {d}", .{num_vertices});
        defer allocator.free(name);
        try runCase(allocator, .{ .name = name, .state = state, .groom = &groom, .resample = num_vertices, .flags = resample_only });
    }
}

test "HairSettings parity: SkinScalpVertices and PrepareForScalpSkinning with the face animation" {
    const allocator = std.testing.allocator;
    var face = (try loadFace(allocator)) orelse return error.SkipZigTest;
    defer face.deinit(allocator);
    const materials = materials3(.{ 0.1, 0.1, 0.1 });
    var settings = try baseSettings(allocator, &face, &materials);
    defer settings.deinit();
    const state = try saveState(allocator, &settings);
    defer allocator.free(state);

    var joint_to_hair: [16]f32 = undefined;
    face.inverse_neck.storeFloat4x4(@ptrCast(&joint_to_hair));
    for (face.frames, 0..) |frame, f| {
        const joints = try allocator.alloc(f32, 16 * frame.len);
        defer allocator.free(joints);
        for (frame, 0..) |m, i| m.storeFloat4x4(@ptrCast(joints[16 * i ..][0..16]));
        try std.testing.expectEqual(@as(c_int, 0), jolt_hair_skin_scalp(state.ptr, state.len, &joint_to_hair, joints.ptr, @intCast(frame.len)));

        var vertices: std.ArrayList(Vec3) = .empty;
        defer vertices.deinit(allocator);
        try settings.skinScalpVertices(allocator, face.inverse_neck, frame, &vertices);
        var skinned: std.ArrayList(u8) = .empty;
        defer skinned.deinit(allocator);
        for (vertices.items) |v| {
            var f3: Float3 = undefined;
            v.storeFloat3(&f3);
            try skinned.appendSlice(allocator, std.mem.asBytes(&f3));
        }
        const name = try std.fmt.allocPrint(allocator, "frame {d}", .{f});
        defer allocator.free(name);
        try expectSameBytes(name, "SkinScalpVertices", skinned.items, cppResult(0));

        const prepared = try allocator.alloc(Mat44, settings.scalp_inverse_bind_pose.items.len);
        defer allocator.free(prepared);
        settings.prepareForScalpSkinning(face.inverse_neck, frame, prepared);
        var prepared_bytes: std.ArrayList(u8) = .empty;
        defer prepared_bytes.deinit(allocator);
        for (prepared) |m| {
            var columns: [4]Float4 = undefined;
            m.storeFloat4x4(&columns);
            try prepared_bytes.appendSlice(allocator, std.mem.asBytes(&columns));
        }
        try expectSameBytes(name, "PrepareForScalpSkinning", prepared_bytes.items, cppResult(1));
    }
}

test "HairSettings parity: Gradient, GradientSampler and Material functions" {
    var checker: fw.Checker = .{ .name = "HairSettings gradients" };
    var rng: fw.Rng = .{ .state = 0x9e37_79b9 };
    for (0..fw.iterations) |i| {
        const g: [4]f32 = switch (i % 4) {
            0 => .{ rng.float(0, 1), rng.float(0, 1), rng.float(0, 0.5), rng.float(0.5, 1) },
            1 => .{ rng.float(-2, 2), rng.float(-2, 2), rng.float(-1, 2), rng.float(-1, 2) },
            2 => .{ rng.float(0, 1), rng.float(0, 1), 0.3, 0.3 }, // Equal fractions: a division by zero
            else => .{ 1.0, 1.0, 0.0, rng.float(0, 1) },
        };
        const ratio = if (i % 3 == 0) rng.float(0, 4) else @as(f32, @floatFromInt(rng.next() % 5)) * 0.5;
        const fraction = if (i % 5 == 0) rng.float(-0.5, 1.5) else rng.float(0, 1);
        const start = rng.next() % 100;
        const end = start + 2 + rng.next() % 30;
        const vertex = start + rng.next() % (end - start);
        const multiplier = [4]f32{ rng.float(0, 100), rng.float(0, 100), rng.float(0, 100), rng.float(0, 100) };
        const compliance = rng.float(0, 1.0e-3);
        const density_force: f32 = if (i % 2 == 0) 0.0 else rng.float(0, 1);
        const enable_collision = rng.next() % 2;

        var jolt: [13]f32 = undefined;
        jolt_hair_gradient(&g, ratio, fraction, start, end, vertex, &multiplier, compliance, density_force, @intCast(enable_collision), &jolt);

        const gradient: Gradient = .init(g[0], g[1], .{ .min_fraction = g[2], .max_fraction = g[3] });
        const step = gradient.makeStepDependent(ratio);
        const sampler: HairSettings.GradientSampler = .init(gradient);
        const f4 = sampler.toFloat4();
        var material: Material = .{};
        material.bend_compliance_multiplier = .{ .x = multiplier[0], .y = multiplier[1], .z = multiplier[2], .w = multiplier[3] };
        material.bend_compliance = compliance;
        // GetBendCompliance asserts a fraction in [0, 1] (Jolt's release build continues): the reference has no asserts
        const bend = if (fraction >= 0.0 and fraction <= 1.0 or !zolt.Core.enable_asserts) material.getBendCompliance(fraction) else jolt[10];
        material.grid_velocity_factor = gradient;
        material.grid_density_force_factor = density_force;
        material.global_pose = gradient;
        material.enable_collision = enable_collision != 0;
        const zolt_values = [13]f32{
            step.min,                                    step.max,                                           step.min_fraction, step.max_fraction,
            f4.x,                                        f4.y,                                               f4.z,              f4.w,
            sampler.sample(fraction),                    sampler.sampleStrand(.init(start, end, 0), vertex), bend,              if (material.needsGrid()) 1.0 else 0.0,
            if (material.globalPoseOnly()) 1.0 else 0.0,
        };
        checker.check(.{ g, ratio, fraction, start, end, vertex }, zolt_values, jolt);
    }
    try checker.finish();
}
