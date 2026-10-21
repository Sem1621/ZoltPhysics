//! Parity of the hair shaders of group C1a with Jolt's CPU backend (Docs/Zolt/RemainingFeaturesArchitecture.md section
//! 4, row "Compute dispatch"): HairTeleport, HairApplyDeltaTransform, HairSkinVertices, HairSkinRoots,
//! HairApplyGlobalPose, HairUpdateRoots, HairCalculateRenderPositions, HairGridClear, HairGridAccumulate and
//! HairGridNormalize. C++ side: ComputeReference.cpp (jolt_compute_dispatch). Both systems have every hair shader:
//! Jolt's HairRegisterShaders and Zolt's registerHairShaders. HairShaderParity2.zig (group C1b) has the layers of the
//! other five hair shaders and reuses the generators of this file.
//!
//! One layer per shader. Each round fills the buffers with random data, dispatches on both sides with
//! `ComputeParity.Dispatcher` and compares the RW buffers: float buffers with `sameBits` (NaNs are equal), integer
//! buffers byte for byte.
//! - The constant blocks are random but valid: every member of `gContext` starts as random bits, then the members that
//!   the shader reads get valid values. So a shader that reads the wrong member fails.
//! - Indices are valid and buffers have at least the elements the shader reads (UB3), at least one element each. Threads
//!   past the end return before they read.
//! - The group size is the shader's batch size, 1 or random. Some dispatches have an extra group (threads that return
//!   early) or a second row in y (the shader runs twice for the same x, also on both sides).
//! - The grid inputs never produce the UB11 grid positions: no NaN positions, a positive and finite grid scale, a grid
//!   of at least 2 cells per axis. Infinite positions are fine: the clamp maps them into the grid.
//! - HairGridAccumulate adds with atomics. Jolt's CPU backend runs the threads one at a time on the calling thread, in
//!   x order (ComputeQueueCPU::Dispatch is a plain loop), and so does Zolt's. The additions are integer additions that
//!   wrap, so the grid does not depend on their order anyway. The grid layer checks this on the C++ side: the reversed
//!   vertex order gives the same bytes.

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const ComputeParity = @import("ComputeParity.zig");

const BindingDesc = ComputeParity.BindingDesc;
const Dispatcher = ComputeParity.Dispatcher;
const Compare = ComputeParity.Compare;
const inputFloat = ComputeParity.inputFloat;
const specialFloat = ComputeParity.specialFloat;
const hlsl = zolt.HLSLToCPP;
const float3 = hlsl.float3;
const float4 = hlsl.float4;
const int4 = hlsl.int4;
const HairStructs = zolt.HairStructs;
const HairUpdateContext = HairStructs.HairUpdateContext;
const HairIterationContext = HairStructs.HairIterationContext;
const HairMaterial = HairStructs.HairMaterial;
const HairPosition = HairStructs.HairPosition;
const HairVelocity = HairStructs.HairVelocity;
const HairGlobalPoseTransform = HairStructs.HairGlobalPoseTransform;
const HairSkinWeight = HairStructs.HairSkinWeight;
const HairSkinPoint = HairStructs.HairSkinPoint;
const HairSVertexInfluence = HairStructs.HairSVertexInfluence;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;

/// The dispatcher of ComputeParity with the hair shaders on the Zolt side (registerHairShaders, like Jolt's system)
pub fn initDispatcher(allocator: Allocator) !Dispatcher {
    var dispatcher: Dispatcher = try .init(allocator);
    errdefer dispatcher.deinit();
    try zolt.registerHairShaders(zolt.virtual.downcast(zolt.ComputeSystemCPU, dispatcher.system.get().?));
    return dispatcher;
}

/// Run `Shader` on both sides for `num_threads` threads in x (see the file header for the group sizes)
pub fn dispatchRandom(dispatcher: *Dispatcher, checker: *fw.Checker, rng: *fw.Rng, comptime Shader: type, batch: u32, num_threads: u32, bindings: []const BindingDesc, compare: Compare) !void {
    const group_size_x: u32 = switch (rng.next() % 4) {
        0, 1 => batch,
        2 => 1,
        else => rng.intRange(u32, 2, 100),
    };
    var num_groups_x = (num_threads + group_size_x - 1) / group_size_x;
    if (rng.next() % 4 == 0) num_groups_x += 1; // Threads past the end return early
    const twice = rng.next() % 8 == 0;
    const group_size_y: u32 = if (twice and rng.next() % 2 == 0) 2 else 1;
    const num_groups_y: u32 = if (twice and group_size_y == 1) 2 else 1;
    const found = try dispatcher.dispatchBoth(checker, Shader.name, .{ group_size_x, group_size_y, 1 }, .{ num_groups_x, num_groups_y, 1 }, bindings, compare);
    try std.testing.expect(found);
}

//////////////////////////////////////////////////////////////////////////////////////////
// Random inputs
//////////////////////////////////////////////////////////////////////////////////////////

/// A float3 with components in [-range, range), one in eight components is special (ComputeParity.inputFloat)
pub fn randomFloat3(rng: *fw.Rng, range: f32) float3 {
    return .init(inputFloat(rng, range), inputFloat(rng, range), inputFloat(rng, range));
}

/// A random unit quaternion (Zolt's math, it is only an input)
pub fn unitQuat(rng: *fw.Rng) Quat {
    const q = Vec4.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1));
    return Quat.fromVec4(if (q.lengthSq() > 1.0e-6) q.normalized() else Vec4.init(0, 0, 0, 1));
}

/// A rotation: mostly a unit quaternion, sometimes any float4 (zero included)
pub fn randomRotation(rng: *fw.Rng) float4 {
    return switch (rng.next() % 8) {
        0 => .init(inputFloat(rng, 2), inputFloat(rng, 2), inputFloat(rng, 2), inputFloat(rng, 2)),
        1 => .init(0, 0, 0, 0),
        else => blk: {
            const q = unitQuat(rng);
            break :blk .init(q.getX(), q.getY(), q.getZ(), q.getW());
        },
    };
}

/// A compressed quaternion (JPH_QuatDecompress): any bit pattern or a compressed unit quaternion
pub fn compressedQuat(rng: *fw.Rng) u32 {
    return if (rng.next() % 2 == 0) rng.next() else unitQuat(rng).compressUnitQuat();
}

/// A matrix: mostly a rotation and translation, sometimes any floats
pub fn randomMat44(rng: *fw.Rng) hlsl.Mat44 {
    if (rng.next() % 4 == 0) {
        var m: hlsl.Mat44 = undefined;
        for (&m.c) |*c| c.* = .init(inputFloat(rng, 3), inputFloat(rng, 3), inputFloat(rng, 3), inputFloat(rng, 3));
        return m;
    }
    const m = Mat44.rotationTranslation(unitQuat(rng), Vec3.init(rng.float(-10, 10), rng.float(-10, 10), rng.float(-10, 10)));
    var result: hlsl.Mat44 = undefined;
    for (&result.c, 0..) |*c, i| {
        const col = m.getColumn4(@intCast(i));
        c.* = .init(col.getX(), col.getY(), col.getZ(), col.getW());
    }
    return result;
}

pub fn randomPosition(rng: *fw.Rng) HairPosition {
    return .{ .position = randomFloat3(rng, 5), .rotation = randomRotation(rng) };
}

pub fn randomVelocity(rng: *fw.Rng) HairVelocity {
    return .{ .velocity = randomFloat3(rng, 5), .angular_velocity = randomFloat3(rng, 5) };
}

pub fn randomGlobalPoseTransform(rng: *fw.Rng) HairGlobalPoseTransform {
    return .{ .position = randomFloat3(rng, 5), .rotation = randomRotation(rng) };
}

/// A gradient sampler (GradientSampler::ToFloat4: multiplier, offset, min, max): mostly a valid range in [0, 1]
pub fn randomGradient(rng: *fw.Rng) float4 {
    if (rng.next() % 4 == 0)
        return .init(inputFloat(rng, 2), inputFloat(rng, 2), inputFloat(rng, 2), inputFloat(rng, 2));
    const a = rng.float(0, 1);
    const b = rng.float(0, 1);
    return .init(rng.float(-2, 2), rng.float(-1, 1), if (a < b) a else b, if (a < b) b else a);
}

/// Random bits, for the members that a shader must not read
pub fn randomBits(comptime T: type, rng: *fw.Rng) T {
    var value: T = undefined;
    const bytes = std.mem.asBytes(&value);
    var i: usize = 0;
    while (i < bytes.len) : (i += 4) {
        const word = rng.next();
        const n = @min(4, bytes.len - i);
        @memcpy(bytes[i..][0..n], std.mem.asBytes(&word)[0..n]);
    }
    return value;
}

/// A material with valid gradients for the members that the shaders of this layer read
pub fn randomMaterial(rng: *fw.Rng) HairMaterial {
    var m = randomBits(HairMaterial, rng);
    m.world_transform_influence = randomGradient(rng);
    m.global_pose = randomGradient(rng);
    m.skin_global_pose = randomGradient(rng);
    return m;
}

/// A context of random bits: the caller sets the members that the shader reads
pub fn randomContext(rng: *fw.Rng) HairUpdateContext {
    return randomBits(HairUpdateContext, rng);
}

/// An iteration fraction like Hair's (1 / (NumIterations - Iteration)), sometimes any value in [0, 1] or a special float
pub fn randomIterationFraction(rng: *fw.Rng) f32 {
    return switch (rng.next() % 4) {
        0 => rng.float(0, 1),
        1 => specialFloat(rng, 2),
        else => blk: {
            const n = rng.intRange(u32, 1, 8);
            const i = rng.intRange(u32, 0, n - 1);
            break :blk 1.0 / @as(f32, @floatFromInt(n - i));
        },
    };
}

/// Allocate `n` elements (at least one) and fill them with `generate(rng)`
pub fn fill(arena: Allocator, comptime T: type, n: usize, rng: *fw.Rng, comptime generate: fn (*fw.Rng) T) ![]T {
    const items = try arena.alloc(T, @max(n, 1));
    for (items) |*item| item.* = generate(rng);
    return items;
}

pub fn garbagePosition(rng: *fw.Rng) HairPosition {
    return randomBits(HairPosition, rng);
}

pub fn garbageGlobalPoseTransform(rng: *fw.Rng) HairGlobalPoseTransform {
    return randomBits(HairGlobalPoseTransform, rng);
}

pub fn garbageFloat3(rng: *fw.Rng) float3 {
    return randomBits(float3, rng);
}

pub fn garbageVelocity(rng: *fw.Rng) HairVelocity {
    return randomBits(HairVelocity, rng);
}

pub fn position5(rng: *fw.Rng) float3 {
    return randomFloat3(rng, 5);
}

/// The strand data of the per vertex shaders that skip the roots. Vertex `v` of strand `s` has index
/// `v * num_strands + s`, like Hair's padded vertex arrays.
pub const Strands = struct {
    num_strands: u32,
    num_vertices: u32,
    /// A bit per vertex
    vertices_fixed: []u32,
    /// A byte per vertex
    strand_fractions: []u32,
    /// A byte per strand, each below the number of materials
    strand_material_index: []u32,
    materials: []HairMaterial,

    pub fn init(arena: Allocator, rng: *fw.Rng) !Strands {
        const num_strands = rng.intRange(u32, 1, 70);
        const vertices_per_strand = rng.intRange(u32, 1, 12);
        const num_vertices = num_strands * vertices_per_strand;
        const num_materials = rng.intRange(u32, 1, 8);

        const vertices_fixed = try arena.alloc(u32, (num_vertices + 31) / 32);
        const fixed_mode = rng.next() % 3;
        for (vertices_fixed) |*f| f.* = switch (fixed_mode) {
            0 => 0,
            1 => rng.next() & rng.next(), // A quarter of the vertices is fixed
            else => rng.next(),
        };

        const strand_fractions = try arena.alloc(u32, (num_vertices + 3) / 4);
        for (strand_fractions) |*f| f.* = rng.next();

        const strand_material_index = try arena.alloc(u32, (num_strands + 3) / 4);
        for (strand_material_index) |*m| {
            var packed_indices: u32 = 0;
            for (0..4) |i| packed_indices |= rng.intRange(u32, 0, num_materials - 1) << @intCast(8 * i);
            m.* = packed_indices;
        }

        const materials = try fill(arena, HairMaterial, num_materials, rng, randomMaterial);

        return .{
            .num_strands = num_strands,
            .num_vertices = num_vertices,
            .vertices_fixed = vertices_fixed,
            .strand_fractions = strand_fractions,
            .strand_material_index = strand_material_index,
            .materials = materials,
        };
    }

    /// Threads of a dispatch that skips the first vertex of each strand
    pub fn numThreadsSkipRoots(self: Strands) u32 {
        return self.num_vertices - self.num_strands;
    }
};

/// The gIterationContext binding: Hair binds it only for the shaders that read it. The other shaders get it in some
/// rounds, and keep the zeroed block of the wrapper otherwise.
pub fn iterationContextBinding(rng: *fw.Rng, iteration_context: []HairIterationContext, always: bool) BindingDesc {
    iteration_context[0] = .{ .accumulated_delta_time = inputFloat(rng, 1), .iteration_fraction = randomIterationFraction(rng) };
    const binding = BindingDesc.of("gIterationContext", .constant_buffer, HairIterationContext, iteration_context);
    return if (always or rng.next() % 2 == 0) binding else binding.withFlags(BindingDesc.flag_null);
}

/// A read only binding: Hair binds RW buffers with setBuffer too (gPositions of HairGridAccumulate)
pub fn readBinding(rng: *fw.Rng, name: [*:0]const u8, comptime T: type, items: []T) BindingDesc {
    const binding = BindingDesc.of(name, .buffer, T, items);
    return switch (rng.next() % 4) {
        0 => binding.withType(.rw_buffer),
        1 => binding.withType(.upload_buffer),
        else => binding,
    };
}

pub const num_rounds = 200;

//////////////////////////////////////////////////////////////////////////////////////////
// The layers
//////////////////////////////////////////////////////////////////////////////////////////

test "HairShaderParity: HairTeleport" {
    const allocator = std.testing.allocator;
    var dispatcher = try initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairTeleport" };
    var rng: fw.Rng = .{ .state = 0x7e1e9047 };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        const num_vertices = rng.intRange(u32, 0, 500);
        var context = [_]HairUpdateContext{randomContext(&rng)};
        context[0].num_vertices = num_vertices;
        var iteration_context: [1]HairIterationContext = undefined;

        const initial_positions = try fill(arena, float3, num_vertices, &rng, position5);
        const initial_bishops = try fill(arena, u32, num_vertices, &rng, compressedQuat);
        const positions = try fill(arena, HairPosition, num_vertices, &rng, garbagePosition);
        const velocities = try fill(arena, HairVelocity, num_vertices, &rng, garbageVelocity);

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            iterationContextBinding(&rng, &iteration_context, false),
            readBinding(&rng, "gInitialPositions", float3, initial_positions),
            readBinding(&rng, "gInitialBishops", u32, initial_bishops),
            .of("gPositions", .rw_buffer, HairPosition, positions),
            .of("gVelocities", .rw_buffer, HairVelocity, velocities),
        };
        try dispatchRandom(&dispatcher, &checker, &rng, zolt.HairTeleport, HairStructs.hair_per_vertex_batch, num_vertices, &bindings, .floats);
    }
    try checker.finish();
}

test "HairShaderParity: HairApplyDeltaTransform" {
    const allocator = std.testing.allocator;
    var dispatcher = try initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairApplyDeltaTransform" };
    var rng: fw.Rng = .{ .state = 0x0de17a7f };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        const strands = try Strands.init(arena, &rng);
        var context = [_]HairUpdateContext{randomContext(&rng)};
        context[0].num_strands = strands.num_strands;
        context[0].num_vertices = strands.num_vertices;
        context[0].delta_transform = randomMat44(&rng);
        context[0].delta_transform_quat = randomRotation(&rng);
        var iteration_context: [1]HairIterationContext = undefined;

        const positions = try fill(arena, HairPosition, strands.num_vertices, &rng, randomPosition);
        const velocities = try fill(arena, HairVelocity, strands.num_vertices, &rng, randomVelocity);

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            iterationContextBinding(&rng, &iteration_context, false),
            readBinding(&rng, "gVerticesFixed", u32, strands.vertices_fixed),
            readBinding(&rng, "gStrandFractions", u32, strands.strand_fractions),
            readBinding(&rng, "gMaterials", HairMaterial, strands.materials),
            readBinding(&rng, "gStrandMaterialIndex", u32, strands.strand_material_index),
            .of("gPositions", .rw_buffer, HairPosition, positions),
            .of("gVelocities", .rw_buffer, HairVelocity, velocities),
        };
        try dispatchRandom(&dispatcher, &checker, &rng, zolt.HairApplyDeltaTransform, HairStructs.hair_per_vertex_batch, strands.numThreadsSkipRoots(), &bindings, .floats);
    }
    try checker.finish();
}

test "HairShaderParity: HairSkinVertices" {
    const allocator = std.testing.allocator;
    var dispatcher = try initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairSkinVertices" };
    var rng: fw.Rng = .{ .state = 0x5c1a7e55 };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        const num_skin_vertices = rng.intRange(u32, 0, 300);
        const num_weights_per_vertex = rng.intRange(u32, 0, 5);
        const num_joints = rng.intRange(u32, 1, 12);
        var context = [_]HairUpdateContext{randomContext(&rng)};
        context[0].num_skin_vertices = num_skin_vertices;
        context[0].num_skin_weights_per_vertex = num_weights_per_vertex;
        var iteration_context: [1]HairIterationContext = undefined;

        const scalp_vertices = try fill(arena, float3, num_skin_vertices, &rng, position5);
        const skin_weights = try arena.alloc(HairSkinWeight, @max(num_skin_vertices * num_weights_per_vertex, 1));
        for (skin_weights) |*w| w.* = .{
            .joint_idx = rng.intRange(u32, 0, num_joints - 1),
            .weight = switch (rng.next() % 8) {
                0, 1 => 0.0,
                2 => -rng.float(0, 1),
                3 => specialFloat(&rng, 1),
                else => rng.float(0, 1),
            },
        };
        const joint_matrices = try arena.alloc(hlsl.Mat44, num_joints);
        for (joint_matrices) |*m| m.* = randomMat44(&rng);
        const scalp_vertices_out = try fill(arena, float3, num_skin_vertices, &rng, garbageFloat3);

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            iterationContextBinding(&rng, &iteration_context, false),
            readBinding(&rng, "gScalpVertices", float3, scalp_vertices),
            readBinding(&rng, "gScalpSkinWeights", HairSkinWeight, skin_weights),
            readBinding(&rng, "gScalpJointMatrices", hlsl.Mat44, joint_matrices),
            .of("gScalpVerticesOut", .rw_buffer, float3, scalp_vertices_out),
        };
        try dispatchRandom(&dispatcher, &checker, &rng, zolt.HairSkinVertices, HairStructs.hair_per_vertex_batch, num_skin_vertices, &bindings, .floats);
    }
    try checker.finish();
}

test "HairShaderParity: HairSkinRoots" {
    const allocator = std.testing.allocator;
    var dispatcher = try initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairSkinRoots" };
    var rng: fw.Rng = .{ .state = 0x2007ab1e };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        const num_strands = rng.intRange(u32, 0, 100);
        const num_scalp_vertices = rng.intRange(u32, 3, 60);
        const num_triangles = rng.intRange(u32, 1, 40);
        var context = [_]HairUpdateContext{randomContext(&rng)};
        context[0].num_strands = num_strands;
        context[0].scalp_to_head = randomMat44(&rng);
        var iteration_context: [1]HairIterationContext = undefined;

        const skin_points = try arena.alloc(HairSkinPoint, @max(num_strands, 1));
        for (skin_points) |*sp| {
            const u = rng.float(0, 1);
            const v = rng.float(0, 1 - u);
            sp.* = .{
                .triangle_index = rng.intRange(u32, 0, num_triangles - 1),
                .u = if (rng.next() % 8 == 0) inputFloat(&rng, 2) else u,
                .v = if (rng.next() % 8 == 0) inputFloat(&rng, 2) else v,
                .to_bishop = compressedQuat(&rng),
            };
        }
        const scalp_vertices = try arena.alloc(float3, num_scalp_vertices);
        for (scalp_vertices) |*v| v.* = randomFloat3(&rng, 1);
        const scalp_triangles = try arena.alloc(u32, 3 * num_triangles);
        for (scalp_triangles, 0..) |*t, i| {
            // Sometimes a degenerate triangle (normalize of a zero vector)
            t.* = if (i % 3 == 2 and rng.next() % 16 == 0) scalp_triangles[i - 1] else rng.intRange(u32, 0, num_scalp_vertices - 1);
        }
        const initial_positions = try fill(arena, float3, num_strands, &rng, position5);
        const initial_bishops = try fill(arena, u32, num_strands, &rng, compressedQuat);
        const positions = try fill(arena, HairPosition, num_strands, &rng, garbagePosition);
        const global_pose_transforms = try fill(arena, HairGlobalPoseTransform, num_strands, &rng, garbageGlobalPoseTransform);

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            iterationContextBinding(&rng, &iteration_context, false),
            readBinding(&rng, "gSkinPoints", HairSkinPoint, skin_points),
            readBinding(&rng, "gScalpVertices", float3, scalp_vertices),
            readBinding(&rng, "gScalpTriangles", u32, scalp_triangles),
            readBinding(&rng, "gInitialPositions", float3, initial_positions),
            readBinding(&rng, "gInitialBishops", u32, initial_bishops),
            .of("gPositions", .rw_buffer, HairPosition, positions),
            .of("gGlobalPoseTransforms", .rw_buffer, HairGlobalPoseTransform, global_pose_transforms),
        };
        try dispatchRandom(&dispatcher, &checker, &rng, zolt.HairSkinRoots, HairStructs.hair_per_strand_batch, num_strands, &bindings, .floats);
    }
    try checker.finish();
}

test "HairShaderParity: HairApplyGlobalPose" {
    const allocator = std.testing.allocator;
    var dispatcher = try initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairApplyGlobalPose" };
    var rng: fw.Rng = .{ .state = 0x91061a1e };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        const strands = try Strands.init(arena, &rng);
        var context = [_]HairUpdateContext{randomContext(&rng)};
        context[0].num_strands = strands.num_strands;
        context[0].num_vertices = strands.num_vertices;
        var iteration_context: [1]HairIterationContext = undefined;

        const initial_positions = try fill(arena, float3, strands.num_vertices, &rng, position5);
        const initial_bishops = try fill(arena, u32, strands.num_vertices, &rng, compressedQuat);
        const global_pose_transforms = try fill(arena, HairGlobalPoseTransform, strands.num_strands, &rng, randomGlobalPoseTransform);
        const positions = try fill(arena, HairPosition, strands.num_vertices, &rng, randomPosition);

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            iterationContextBinding(&rng, &iteration_context, false),
            readBinding(&rng, "gVerticesFixed", u32, strands.vertices_fixed),
            readBinding(&rng, "gStrandFractions", u32, strands.strand_fractions),
            readBinding(&rng, "gInitialPositions", float3, initial_positions),
            readBinding(&rng, "gInitialBishops", u32, initial_bishops),
            readBinding(&rng, "gStrandMaterialIndex", u32, strands.strand_material_index),
            readBinding(&rng, "gMaterials", HairMaterial, strands.materials),
            readBinding(&rng, "gGlobalPoseTransforms", HairGlobalPoseTransform, global_pose_transforms),
            .of("gPositions", .rw_buffer, HairPosition, positions),
        };
        try dispatchRandom(&dispatcher, &checker, &rng, zolt.HairApplyGlobalPose, HairStructs.hair_per_vertex_batch, strands.numThreadsSkipRoots(), &bindings, .floats);
    }
    try checker.finish();
}

test "HairShaderParity: HairUpdateRoots" {
    const allocator = std.testing.allocator;
    var dispatcher = try initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairUpdateRoots" };
    var rng: fw.Rng = .{ .state = 0x4007500f };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        const num_strands = rng.intRange(u32, 0, 150);
        var context = [_]HairUpdateContext{randomContext(&rng)};
        context[0].num_strands = num_strands;
        var iteration_context: [1]HairIterationContext = undefined;

        const target_positions = try fill(arena, HairPosition, num_strands, &rng, randomPosition);
        const target_global_pose_transforms = try fill(arena, HairGlobalPoseTransform, num_strands, &rng, randomGlobalPoseTransform);
        const positions = try fill(arena, HairPosition, num_strands, &rng, randomPosition);
        const global_pose_transforms = try fill(arena, HairGlobalPoseTransform, num_strands, &rng, randomGlobalPoseTransform);

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            iterationContextBinding(&rng, &iteration_context, true),
            readBinding(&rng, "gTargetPositions", HairPosition, target_positions),
            readBinding(&rng, "gTargetGlobalPoseTransforms", HairGlobalPoseTransform, target_global_pose_transforms),
            .of("gPositions", .rw_buffer, HairPosition, positions),
            .of("gGlobalPoseTransforms", .rw_buffer, HairGlobalPoseTransform, global_pose_transforms),
        };
        try dispatchRandom(&dispatcher, &checker, &rng, zolt.HairUpdateRoots, HairStructs.hair_per_strand_batch, num_strands, &bindings, .floats);
    }
    try checker.finish();
}

test "HairShaderParity: HairCalculateRenderPositions" {
    const allocator = std.testing.allocator;
    var dispatcher = try initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairCalculateRenderPositions" };
    var rng: fw.Rng = .{ .state = 0x4e9de405 };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        const num_render_vertices = rng.intRange(u32, 0, 400);
        const num_positions = rng.intRange(u32, 1, 80);
        var context = [_]HairUpdateContext{randomContext(&rng)};
        context[0].num_render_vertices = num_render_vertices;
        var iteration_context: [1]HairIterationContext = undefined;

        const influences = try arena.alloc(HairSVertexInfluence, @max(HairStructs.hair_num_s_vertex_influences * num_render_vertices, 1));
        for (influences) |*inf| inf.* = .{
            .vertex_index = rng.intRange(u32, 0, num_positions - 1),
            .relative_position = randomFloat3(&rng, 0.1),
            .weight = switch (rng.next() % 8) {
                0, 1 => 0.0,
                2 => specialFloat(&rng, 1),
                else => rng.float(0, 1),
            },
        };
        const positions = try fill(arena, HairPosition, num_positions, &rng, randomPosition);
        const render_positions = try fill(arena, float3, num_render_vertices, &rng, garbageFloat3);

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            iterationContextBinding(&rng, &iteration_context, false),
            readBinding(&rng, "gSVertexInfluences", HairSVertexInfluence, influences),
            readBinding(&rng, "gPositions", HairPosition, positions),
            .of("gRenderPositions", .rw_buffer, float3, render_positions),
        };
        try dispatchRandom(&dispatcher, &checker, &rng, zolt.HairCalculateRenderPositions, HairStructs.hair_per_render_vertex_batch, num_render_vertices, &bindings, .floats);
    }
    try checker.finish();
}

/// A grid position input: mostly inside the bounds, some outside (clamped), rarely infinite. Never NaN (UB11).
pub fn gridPosition(rng: *fw.Rng, bounds_min: Vec3, bounds_size: Vec3) float3 {
    var result: [3]f32 = undefined;
    for (&result, 0..) |*r, i| {
        r.* = switch (rng.next() % 16) {
            0 => bounds_min.getComponent(@intCast(i)) + rng.float(-0.5, 0) * bounds_size.getComponent(@intCast(i)),
            1 => bounds_min.getComponent(@intCast(i)) + rng.float(1, 1.5) * bounds_size.getComponent(@intCast(i)),
            2 => if (rng.next() % 2 == 0) std.math.inf(f32) else -std.math.inf(f32),
            3 => bounds_min.getComponent(@intCast(i)) + bounds_size.getComponent(@intCast(i)), // On the upper bound
            else => bounds_min.getComponent(@intCast(i)) + rng.float(0, 1) * bounds_size.getComponent(@intCast(i)),
        };
    }
    return .init(result[0], result[1], result[2]);
}

// The grid layer: HairGridClear, HairGridAccumulate and HairGridNormalize as Hair runs them, each compared
test "HairShaderParity: HairGridClear, HairGridAccumulate, HairGridNormalize" {
    const allocator = std.testing.allocator;
    var dispatcher = try initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairGrid" };
    var order_checker: fw.Checker = .{ .name = "HairShaderParity HairGridAccumulate order (C++ side)" };
    var rng: fw.Rng = .{ .state = 0x51ed2701 };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |round| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        // A grid of at least 2 cells per axis (cGridSizeMin2 = size - 2), the GridSampler of HairSettings
        const gx = rng.intRange(u32, 2, 9);
        const gy = rng.intRange(u32, 2, 9);
        const gz = rng.intRange(u32, 2, 9);
        const num_grid_points = gx * gy * gz;
        const bounds_min = Vec3.init(rng.float(-5, 0), rng.float(-5, 0), rng.float(-5, 0));
        const bounds_size = Vec3.init(rng.float(0.1, 5), rng.float(0.1, 5), rng.float(0.1, 5));

        const strands = try Strands.init(arena, &rng);
        var context = [_]HairUpdateContext{randomContext(&rng)};
        const c = &context[0];
        c.num_strands = strands.num_strands;
        c.num_vertices = strands.num_vertices;
        c.num_grid_points = num_grid_points;
        c.grid_size_min2 = .init(gx - 2, gy - 2, gz - 2);
        c.grid_size_min1 = .init(@floatFromInt(gx - 1), @floatFromInt(gy - 1), @floatFromInt(gz - 1));
        c.grid_offset = .init(bounds_min.getX(), bounds_min.getY(), bounds_min.getZ());
        c.grid_scale = .init(@as(f32, @floatFromInt(gx)) / bounds_size.getX(), @as(f32, @floatFromInt(gy)) / bounds_size.getY(), @as(f32, @floatFromInt(gz)) / bounds_size.getZ());
        // The shaders use stride.y and stride.z only (GridIndexToBufferIndex adds index.x): x keeps its random bits
        c.grid_stride.y = gx;
        c.grid_stride.z = gx * gy;
        if (round % 5 == 0) {
            // Clamp to a smaller grid than the buffer (the indices stay valid: index <= cGridSizeMin2)
            c.grid_size_min2 = .init(rng.intRange(u32, 0, gx - 2), rng.intRange(u32, 0, gy - 2), rng.intRange(u32, 0, gz - 2));
            c.grid_size_min1 = .init(rng.float(0, @floatFromInt(gx - 1)), rng.float(0, @floatFromInt(gy - 1)), rng.float(0, @floatFromInt(gz - 1)));
        }
        var iteration_context: [1]HairIterationContext = undefined;

        // The grid starts with garbage and has cells past cNumGridPoints that HairGridClear must keep
        const grid_cells = num_grid_points + rng.intRange(u32, 0, 40);
        const grid = try arena.alloc(int4, grid_cells);
        for (grid) |*g| g.* = randomBits(int4, &rng);

        const clear = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            iterationContextBinding(&rng, &iteration_context, false),
            .of("gVelocityAndDensity", .rw_buffer, int4, grid),
        };
        try dispatchRandom(&dispatcher, &checker, &rng, zolt.HairGridClear, HairStructs.hair_per_grid_cell_batch, num_grid_points, &clear, .bytes);

        // HairGridAccumulate on the cleared grid (in some rounds on a grid of random values: the additions wrap)
        if (round % 7 == 0) {
            for (grid) |*g| g.* = randomBits(int4, &rng);
        }
        const positions = try arena.alloc(HairPosition, strands.num_vertices);
        for (positions) |*p| p.* = .{ .position = gridPosition(&rng, bounds_min, bounds_size), .rotation = randomRotation(&rng) };
        const velocities = try arena.alloc(HairVelocity, strands.num_vertices);
        for (velocities) |*v| {
            // Large velocities make the fixed point values large, so that one ulp of a weight changes the rounded
            // integer. A special float (NaN, infinite, 1e10) converts to 0x80000000 on both sides.
            const range: f32 = if (rng.next() % 2 == 0) 5 else 3000;
            v.* = .{
                .velocity = .init(inputFloat(&rng, range), inputFloat(&rng, range), inputFloat(&rng, range)),
                .angular_velocity = randomBits(float3, &rng),
            };
        }
        const grid_before = try arena.dupe(int4, grid);

        const accumulate = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            iterationContextBinding(&rng, &iteration_context, false),
            readBinding(&rng, "gVerticesFixed", u32, strands.vertices_fixed),
            readBinding(&rng, "gPositions", HairPosition, positions),
            readBinding(&rng, "gVelocities", HairVelocity, velocities),
            .of("gVelocityAndDensity", .rw_buffer, int4, grid),
        };
        try dispatchRandom(&dispatcher, &checker, &rng, zolt.HairGridAccumulate, HairStructs.hair_per_vertex_batch, strands.numThreadsSkipRoots(), &accumulate, .bytes);

        // The C++ side does not depend on the order of the atomic additions: reverse the order of the vertices that are
        // not roots (with their fixed bits) and accumulate again, once per x (no second row in y)
        if (round % 2 == 0) {
            const n = strands.num_vertices;
            const first = strands.num_strands;
            const reversed_positions = try arena.dupe(HairPosition, positions);
            const reversed_velocities = try arena.dupe(HairVelocity, velocities);
            const reversed_fixed = try arena.alloc(u32, strands.vertices_fixed.len);
            @memset(reversed_fixed, 0);
            for (0..n) |v| {
                const src = if (v < first) v else first + (n - 1 - v);
                reversed_positions[v] = positions[src];
                reversed_velocities[v] = velocities[src];
                if ((strands.vertices_fixed[src >> 5] >> @intCast(src & 31)) & 1 != 0)
                    reversed_fixed[v >> 5] |= @as(u32, 1) << @intCast(v & 31);
            }
            const grid_forward = try arena.dupe(int4, grid_before);
            const grid_reversed = try arena.dupe(int4, grid_before);
            const groups = (strands.numThreadsSkipRoots() + 63) / 64;
            const forward = [_]BindingDesc{
                .of("gContext", .constant_buffer, HairUpdateContext, &context),
                .of("gVerticesFixed", .buffer, u32, strands.vertices_fixed),
                .of("gPositions", .buffer, HairPosition, positions),
                .of("gVelocities", .buffer, HairVelocity, velocities),
                .of("gVelocityAndDensity", .rw_buffer, int4, grid_forward),
            };
            const reversed = [_]BindingDesc{
                .of("gContext", .constant_buffer, HairUpdateContext, &context),
                .of("gVerticesFixed", .buffer, u32, reversed_fixed),
                .of("gPositions", .buffer, HairPosition, reversed_positions),
                .of("gVelocities", .buffer, HairVelocity, reversed_velocities),
                .of("gVelocityAndDensity", .rw_buffer, int4, grid_reversed),
            };
            // dispatchBoth leaves Jolt's results in the RW buffers
            _ = try dispatcher.dispatchBoth(&checker, zolt.HairGridAccumulate.name, .{ 64, 1, 1 }, .{ groups, 1, 1 }, &forward, .bytes);
            _ = try dispatcher.dispatchBoth(&checker, zolt.HairGridAccumulate.name, .{ 64, 1, 1 }, .{ groups, 1, 1 }, &reversed, .bytes);
            order_checker.check(.{ .round = round, .first_mismatching_byte = std.mem.indexOfDiff(u8, std.mem.sliceAsBytes(grid_forward), std.mem.sliceAsBytes(grid_reversed)) }, true, std.mem.eql(u8, std.mem.sliceAsBytes(grid_forward), std.mem.sliceAsBytes(grid_reversed)));
        }

        // HairGridNormalize on the accumulated grid. The cells then hold float bits. They are compared byte for byte:
        // Normalize never makes a NaN, and the cells past cNumGridPoints keep their input bytes.
        const normalize = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            iterationContextBinding(&rng, &iteration_context, false),
            .of("gVelocityAndDensity", .rw_buffer, int4, grid),
        };
        try dispatchRandom(&dispatcher, &checker, &rng, zolt.HairGridNormalize, HairStructs.hair_per_grid_cell_batch, num_grid_points, &normalize, .bytes);
    }
    try fw.finishAll(&.{ &checker, &order_checker });
}

// HairGridNormalize on random fixed point cells: zero, negative, tiny and large densities, any velocity
test "HairShaderParity: HairGridNormalize on random cells" {
    const allocator = std.testing.allocator;
    var dispatcher = try initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairGridNormalize" };
    var rng: fw.Rng = .{ .state = 0x6e0a11ce };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        const num_grid_points = rng.intRange(u32, 0, 700);
        var context = [_]HairUpdateContext{randomContext(&rng)};
        context[0].num_grid_points = num_grid_points;
        var iteration_context: [1]HairIterationContext = undefined;

        const grid = try arena.alloc(int4, @max(num_grid_points + rng.intRange(u32, 0, 10), 1));
        for (grid) |*g| {
            g.* = randomBits(int4, &rng);
            g.w = switch (rng.next() % 8) {
                0 => 0,
                1 => -@as(i32, @intCast(rng.intRange(u32, 0, 5000))),
                // The smallest positive densities (1 to 3 / 1024). Fixed point densities never come near the 1e-12
                // threshold.
                2 => @intCast(rng.intRange(u32, 1, 3)),
                3 => @bitCast(rng.next()),
                else => @intCast(rng.intRange(u32, 1, 1 << 20)),
            };
            if (rng.next() % 2 == 0) {
                // Velocities of the size that HairGridAccumulate produces
                g.x = @as(i32, @intCast(rng.intRange(u32, 0, 1 << 24))) - (1 << 23);
                g.y = @as(i32, @intCast(rng.intRange(u32, 0, 1 << 24))) - (1 << 23);
                g.z = @as(i32, @intCast(rng.intRange(u32, 0, 1 << 24))) - (1 << 23);
            }
        }

        // The cells hold float bits after the dispatch. They are compared byte for byte: Normalize never makes a NaN
        // (a positive density is at least 1 / 1024), and the cells past cNumGridPoints keep their input bytes.
        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            iterationContextBinding(&rng, &iteration_context, false),
            .of("gVelocityAndDensity", .rw_buffer, int4, grid),
        };
        try dispatchRandom(&dispatcher, &checker, &rng, zolt.HairGridNormalize, HairStructs.hair_per_grid_cell_batch, num_grid_points, &bindings, .bytes);
    }
    try checker.finish();
}
