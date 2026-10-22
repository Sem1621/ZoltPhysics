//! Parity of the hair shaders of group C1b with Jolt's CPU backend (Docs/Zolt/RemainingFeaturesArchitecture.md section
//! 4, row "Compute dispatch"): HairCalculateCollisionPlanes, HairIntegrate, HairUpdateStrands, HairUpdateVelocity and
//! HairUpdateVelocityIntegrate, and the registration of the 15 hair shaders (HairWrapper). C++ side:
//! ComputeReference.cpp.
//!
//! The layers work like those of HairShaderParity.zig (group C1a) and reuse its generators. One layer per shader. Each
//! round fills the buffers with random but valid data and dispatches on both sides with several group sizes. Float
//! buffers compare with `sameBits` (NaNs are equal), integer buffers byte for byte.
//! - The collision data has Hair's layout: per shape the number of faces and per face the start and end of its vertex
//!   indices, then a terminating 0, then the vertex indices. The shapes are boxes and random polygons.
//! - HairUpdateStrands writes the first two vertices of a strand even when it has fewer, so every strand has room for
//!   two. The strand fraction of a vertex in the middle of a strand is below 1: the byte 255 reads past the bend
//!   compliance multipliers, which is undefined in Jolt (see Zolt/Shaders/HairUpdateStrands.zig).
//! - `applyGrid` never sees a NaN position (UB11). HairIntegrate reads the positions of its buffer: in the rounds where
//!   a material enables the grid, they come from `gridPosition`. HairUpdateVelocityIntegrate first applies the global
//!   pose and the collision: in those rounds every input of that path is finite and of moderate size. Those rounds have
//!   no second row in y: the second pass would read the positions that the first pass wrote, which are often NaN.
//!   `expectNoNanGridPosition` checks the rule after each dispatch. Rounds without the grid use special floats
//!   everywhere.

const std = @import("std");
const Allocator = std.mem.Allocator;
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const ComputeParity = @import("ComputeParity.zig");
const C1a = @import("HairShaderParity.zig");

const BindingDesc = ComputeParity.BindingDesc;
const inputFloat = ComputeParity.inputFloat;
const specialFloat = ComputeParity.specialFloat;
const hlsl = zolt.HLSLToCPP;
const float3 = hlsl.float3;
const float4 = hlsl.float4;
const HairStructs = zolt.HairStructs;
const HairUpdateContext = HairStructs.HairUpdateContext;
const HairIterationContext = HairStructs.HairIterationContext;
const HairMaterial = HairStructs.HairMaterial;
const HairPosition = HairStructs.HairPosition;
const HairVelocity = HairStructs.HairVelocity;
const HairGlobalPoseTransform = HairStructs.HairGlobalPoseTransform;
const HairCollisionPlane = HairStructs.HairCollisionPlane;
const HairCollisionShape = HairStructs.HairCollisionShape;
const Vec3 = zolt.Vec3;

const randomBits = C1a.randomBits;
const randomGradient = C1a.randomGradient;
const randomRotation = C1a.randomRotation;
const randomFloat3 = C1a.randomFloat3;
const fill = C1a.fill;
const Strands = C1a.Strands;
const num_rounds = C1a.num_rounds;

extern fn jolt_compute_hair_shaders(names: [*]const [*:0]const u8, num_names: u32, out_found: [*]u32, out_init_found: [*]u32) u32;

//////////////////////////////////////////////////////////////////////////////////////////
// Random inputs
//////////////////////////////////////////////////////////////////////////////////////////

/// A float in [min, max). Unless `clean`, one in eight is a special float (ComputeParity.specialFloat).
fn value(rng: *fw.Rng, min: f32, max: f32, clean: bool) f32 {
    if (!clean and rng.next() % 8 == 0)
        return specialFloat(rng, max);
    return rng.float(min, max);
}

/// A float3 with components in [-range, range). Unless `clean`, one in eight components is special.
fn vector(rng: *fw.Rng, range: f32, clean: bool) float3 {
    return .init(value(rng, -range, range, clean), value(rng, -range, range, clean), value(rng, -range, range, clean));
}

/// A gradient sampler (GradientSampler::ToFloat4: multiplier, offset, min, max). `clean`: always a valid range in
/// [0, 1], otherwise sometimes any floats.
fn gradient(rng: *fw.Rng, clean: bool) float4 {
    if (!clean)
        return randomGradient(rng);
    const a = rng.float(0, 1);
    const b = rng.float(0, 1);
    return .init(rng.float(-2, 2), rng.float(-1, 1), if (a < b) a else b, if (a < b) b else a);
}

/// An enable flag of a material: a uint that the shaders test against 0
fn flag(rng: *fw.Rng) u32 {
    return switch (rng.next() % 4) {
        0 => 0,
        1 => rng.next(), // Any bits
        else => 1,
    };
}

/// A material with values for every member that the shaders of this file read. `clean`: the members that move a
/// position before `applyGrid` are finite (global pose, skin global pose, hair radius). `grid`: the grid may be
/// enabled.
fn simulationMaterial(rng: *fw.Rng, clean: bool, grid: bool) HairMaterial {
    var m = randomBits(HairMaterial, rng);
    m.global_pose = gradient(rng, clean);
    m.skin_global_pose = gradient(rng, clean);
    m.gravity_factor = gradient(rng, false);
    m.hair_radius = gradient(rng, clean);
    m.bend_compliance_multiplier = .init(value(rng, 0, 2, false), value(rng, 0, 2, false), value(rng, 0, 2, false), value(rng, 0, 2, false));
    m.grid_velocity_factor = gradient(rng, false);
    m.enable_collision = flag(rng);
    m.enable_lra = flag(rng);
    m.enable_grid = if (grid) flag(rng) else 0;
    m.friction = value(rng, 0, 1, false);
    m.exp_linear_damping_delta_time = value(rng, 0.9, 1, false);
    m.exp_angular_damping_delta_time = value(rng, 0.9, 1, false);
    // Compliances: often 0 (rigid), so that a fixed pair of vertices reaches the denom threshold
    m.bend_compliance_inv_delta_time_sq = if (rng.next() % 4 == 0) 0 else value(rng, 0, 100, false);
    m.stretch_compliance_inv_delta_time_sq = if (rng.next() % 4 == 0) 0 else value(rng, 0, 0.01, false);
    m.grid_density_force_factor = value(rng, 0, 10, false);
    m.inertia_multiplier = value(rng, 0.1, 100, false);
    m.max_linear_velocity_sq = value(rng, 0, 100, false);
    m.max_angular_velocity_sq = value(rng, 0, 1000, false);
    return m;
}

/// The materials of `strands`, replaced with materials for the shaders of this file
fn setMaterials(rng: *fw.Rng, strands: Strands, clean: bool, grid: bool) void {
    for (strands.materials) |*m| m.* = simulationMaterial(rng, clean, grid);
}

/// A delta time: mostly Hair's sub step sizes, sometimes 0, any small value or a special float
fn deltaTime(rng: *fw.Rng) f32 {
    return switch (rng.next() % 8) {
        0 => 0,
        1 => specialFloat(rng, 1),
        2 => rng.float(0, 0.1),
        3 => 1.0 / 90.0,
        else => 1.0 / 60.0,
    };
}

/// A context of random bits with the delta time members: half and two divided by the delta time like Hair, sometimes
/// unrelated values
fn timeContext(rng: *fw.Rng, strands: Strands) HairUpdateContext {
    var c = C1a.randomContext(rng);
    c.num_strands = strands.num_strands;
    c.num_vertices = strands.num_vertices;
    c.delta_time = deltaTime(rng);
    const related = rng.next() % 4 != 0;
    c.half_delta_time = if (related) 0.5 * c.delta_time else value(rng, 0, 0.1, false);
    c.two_div_delta_time = if (related) 2.0 / c.delta_time else value(rng, 0, 200, false);
    c.sub_step_gravity = .init(value(rng, -0.1, 0.1, false), value(rng, -0.5, 0, false), value(rng, -0.1, 0.1, false));
    return c;
}

/// The grid of HairIntegrate and HairUpdateVelocityIntegrate: the normalized velocity and density per cell (floats, the
/// output of HairGridNormalize) and the neutral density per cell
const Grid = struct {
    bounds_min: Vec3,
    bounds_size: Vec3,
    velocity_and_density: []float4,
    neutral_density: []f32,

    /// A grid of at least 2 cells per axis like the grid layer of C1a, with its members of `c`
    fn init(arena: Allocator, rng: *fw.Rng, c: *HairUpdateContext) !Grid {
        const gx = rng.intRange(u32, 2, 9);
        const gy = rng.intRange(u32, 2, 9);
        const gz = rng.intRange(u32, 2, 9);
        const num_grid_points = gx * gy * gz;
        const bounds_min = Vec3.init(rng.float(-5, 0), rng.float(-5, 0), rng.float(-5, 0));
        const bounds_size = Vec3.init(rng.float(0.1, 5), rng.float(0.1, 5), rng.float(0.1, 5));
        c.num_grid_points = num_grid_points;
        c.grid_size_min2 = .init(gx - 2, gy - 2, gz - 2);
        c.grid_size_min1 = .init(@floatFromInt(gx - 1), @floatFromInt(gy - 1), @floatFromInt(gz - 1));
        c.grid_offset = .init(bounds_min.getX(), bounds_min.getY(), bounds_min.getZ());
        c.grid_scale = .init(@as(f32, @floatFromInt(gx)) / bounds_size.getX(), @as(f32, @floatFromInt(gy)) / bounds_size.getY(), @as(f32, @floatFromInt(gz)) / bounds_size.getZ());
        // The shaders use stride.y and stride.z only (GridIndexToBufferIndex adds index.x): x keeps its random bits
        c.grid_stride.y = gx;
        c.grid_stride.z = gx * gy;
        if (rng.next() % 5 == 0) {
            // Clamp to a smaller grid than the buffer (the indices stay valid: index <= cGridSizeMin2)
            c.grid_size_min2 = .init(rng.intRange(u32, 0, gx - 2), rng.intRange(u32, 0, gy - 2), rng.intRange(u32, 0, gz - 2));
            c.grid_size_min1 = .init(rng.float(0, @floatFromInt(gx - 1)), rng.float(0, @floatFromInt(gy - 1)), rng.float(0, @floatFromInt(gz - 1)));
        }

        const velocity_and_density = try arena.alloc(float4, num_grid_points);
        for (velocity_and_density) |*v| v.* = .init(value(rng, -5, 5, false), value(rng, -5, 5, false), value(rng, -5, 5, false), value(rng, 0, 3, false));
        const neutral_density = try arena.alloc(f32, num_grid_points);
        for (neutral_density) |*d| d.* = value(rng, 0, 3, false);
        return .{ .bounds_min = bounds_min, .bounds_size = bounds_size, .velocity_and_density = velocity_and_density, .neutral_density = neutral_density };
    }

    /// A finite position: mostly inside the bounds, some outside (clamped by the grid)
    fn finitePosition(self: Grid, rng: *fw.Rng) float3 {
        var result: [3]f32 = undefined;
        for (&result, 0..) |*r, i|
            r.* = self.bounds_min.getComponent(@intCast(i)) + rng.float(-0.5, 1.5) * self.bounds_size.getComponent(@intCast(i));
        return .init(result[0], result[1], result[2]);
    }
};

/// The convex shapes of HairCalculateCollisionPlanes in Hair's layout, and the collision shapes and planes of the
/// velocity shaders
const Collision = struct {
    planes: []float4,
    vertices: []float3,
    indices: []u32,

    /// Boxes (rotated, translated, faces counter clockwise) and random polygons (any vertex loop, any plane, sometimes
    /// a face without vertices). Every buffer has at least one element.
    fn init(arena: Allocator, rng: *fw.Rng) !Collision {
        var planes: std.ArrayList(float4) = .empty;
        var vertices: std.ArrayList(float3) = .empty;
        var num_faces: std.ArrayList(u32) = .empty; // Per shape
        var faces: std.ArrayList([]const u32) = .empty; // Per face, the vertex indices in gShapeVertices

        const num_shapes = rng.intRange(u32, 0, 5);
        for (0..num_shapes) |_| {
            const first_vertex: u32 = @intCast(vertices.items.len);
            if (rng.next() % 2 == 0) {
                // A box: corner i has the signs of bits 0, 1 and 2 for x, y and z
                const center = Vec3.init(rng.float(-2, 2), rng.float(-2, 2), rng.float(-2, 2));
                const half_extent = Vec3.init(rng.float(0.1, 1.5), rng.float(0.1, 1.5), rng.float(0.1, 1.5));
                const rotation = C1a.unitQuat(rng);
                for (0..8) |i| {
                    const local = Vec3.init(
                        if (i & 1 != 0) half_extent.getX() else -half_extent.getX(),
                        if (i & 2 != 0) half_extent.getY() else -half_extent.getY(),
                        if (i & 4 != 0) half_extent.getZ() else -half_extent.getZ(),
                    );
                    const p = center.add(rotation.mulVec3(local));
                    try vertices.append(arena, .init(p.getX(), p.getY(), p.getZ()));
                }
                const box_faces = [6][4]u32{ .{ 1, 3, 7, 5 }, .{ 0, 4, 6, 2 }, .{ 2, 6, 7, 3 }, .{ 0, 1, 5, 4 }, .{ 4, 5, 7, 6 }, .{ 0, 2, 3, 1 } };
                const normals = [6]Vec3{ .init(1, 0, 0), .init(-1, 0, 0), .init(0, 1, 0), .init(0, -1, 0), .init(0, 0, 1), .init(0, 0, -1) };
                for (box_faces, normals) |face, local_normal| {
                    const indices = try arena.alloc(u32, 4);
                    for (indices, face) |*index, corner| index.* = first_vertex + corner;
                    try faces.append(arena, indices);
                    const n = rotation.mulVec3(local_normal);
                    const v = vertices.items[indices[0]];
                    try planes.append(arena, .init(n.getX(), n.getY(), n.getZ(), -(n.getX() * v.x + n.getY() * v.y + n.getZ() * v.z)));
                }
                try num_faces.append(arena, 6);
            } else {
                // Random polygons on random vertices
                const nv = rng.intRange(u32, 1, 8);
                for (0..nv) |_| try vertices.append(arena, randomFloat3(rng, 3));
                const nf = rng.intRange(u32, 1, 6);
                for (0..nf) |_| {
                    const count: u32 = if (rng.next() % 16 == 0) 0 else rng.intRange(u32, 1, 6);
                    const indices = try arena.alloc(u32, count);
                    for (indices) |*index| index.* = first_vertex + rng.intRange(u32, 0, nv - 1);
                    try faces.append(arena, indices);
                    const n = Vec3.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1));
                    const unit = if (rng.next() % 4 != 0) n.normalizedOr(Vec3.axisX()) else n;
                    try planes.append(arena, .init(unit.getX(), unit.getY(), unit.getZ(), inputFloat(rng, 3)));
                }
                try num_faces.append(arena, nf);
            }
        }

        // The header: per shape the number of faces, per face the start and end of its indices, then the terminator
        var num_header: u32 = 1;
        for (num_faces.items) |nf| num_header += 1 + 2 * nf;
        var indices: std.ArrayList(u32) = .empty;
        var face_index: usize = 0;
        var next_start = num_header;
        for (num_faces.items) |nf| {
            try indices.append(arena, nf);
            for (0..nf) |_| {
                const face = faces.items[face_index];
                face_index += 1;
                try indices.append(arena, next_start);
                next_start += @intCast(face.len);
                try indices.append(arena, next_start);
            }
        }
        try indices.append(arena, 0); // Terminator
        for (faces.items) |face| try indices.appendSlice(arena, face);
        std.debug.assert(indices.items.len == next_start);

        if (planes.items.len == 0) try planes.append(arena, randomBits(float4, rng));
        if (vertices.items.len == 0) try vertices.append(arena, randomFloat3(rng, 3));
        return .{ .planes = planes.items, .vertices = vertices.items, .indices = indices.items };
    }
};

/// The collision shapes of the velocity shaders (at least one). `clean`: finite and moderate values.
fn collisionShapes(arena: Allocator, rng: *fw.Rng, clean: bool) ![]HairCollisionShape {
    const shapes = try arena.alloc(HairCollisionShape, rng.intRange(u32, 1, 4));
    for (shapes) |*s| s.* = .{
        .center_of_mass = vector(rng, 3, clean),
        .linear_velocity = vector(rng, 3, clean),
        .angular_velocity = vector(rng, 3, clean),
    };
    return shapes;
}

/// A collision plane per vertex through a point near the vertex, so that both sides of the plane occur, with a shape
/// index below `num_shapes`. `clean`: finite and moderate values.
fn collisionPlanes(arena: Allocator, rng: *fw.Rng, positions: []const HairPosition, num_shapes: usize, clean: bool) ![]HairCollisionPlane {
    const planes = try arena.alloc(HairCollisionPlane, positions.len);
    for (planes, positions) |*plane, pos| {
        const n = Vec3.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1));
        const normal = if (rng.next() % 8 != 0) n.normalizedOr(Vec3.axisY()) else n;
        const offset = vector(rng, 2, clean);
        const p: float3 = if (clean or rng.next() % 2 == 0) pos.position.add(offset) else offset;
        plane.* = .{
            .plane = .init(normal.getX(), normal.getY(), normal.getZ(), -(normal.getX() * p.x + normal.getY() * p.y + normal.getZ() * p.z)),
            .shape_index = rng.intRange(u32, 0, @intCast(num_shapes - 1)),
        };
    }
    return planes;
}

/// The gIterationContext of the velocity shaders, which read cAccumulatedDeltaTime. `clean`: a finite value.
fn velocityIterationContext(rng: *fw.Rng, iteration_context: []HairIterationContext, clean: bool) BindingDesc {
    iteration_context[0] = .{
        .accumulated_delta_time = if (clean) rng.float(0, 0.05) else inputFloat(rng, 0.05),
        .iteration_fraction = C1a.randomIterationFraction(rng),
    };
    return .of("gIterationContext", .constant_buffer, HairIterationContext, iteration_context);
}

/// A global pose transform with finite and moderate values
fn cleanGlobalPoseTransform(rng: *fw.Rng) HairGlobalPoseTransform {
    const rotation = if (rng.next() % 4 == 0) float4.init(rng.float(-2, 2), rng.float(-2, 2), rng.float(-2, 2), rng.float(-2, 2)) else randomRotation(rng);
    const finite = std.math.isFinite(rotation.x) and std.math.isFinite(rotation.y) and std.math.isFinite(rotation.z) and std.math.isFinite(rotation.w);
    return .{ .position = vector(rng, 5, true), .rotation = if (finite and @abs(rotation.x) < 3 and @abs(rotation.y) < 3 and @abs(rotation.z) < 3 and @abs(rotation.w) < 3) rotation else .init(0, 0, 0, 1) };
}

fn cleanPosition(rng: *fw.Rng) float3 {
    return vector(rng, 5, true);
}

/// Checks the rule of UB11 after a dispatch of HairIntegrate or HairUpdateVelocityIntegrate: `applyGrid` never saw a
/// NaN position. Both shaders write the position to gPreviousPositions just before they call `applyGrid`. So after the
/// dispatch, `previous_positions` (Jolt's results) holds the position that `applyGrid` saw in the last pass. A second
/// row in y runs a second pass on the positions of the first, and a NaN position stays NaN in both shaders.
fn expectNoNanGridPosition(strands: Strands, previous_positions: []const HairPosition) !void {
    var vtx = strands.num_strands; // The roots are not simulated
    while (vtx < strands.num_vertices) : (vtx += 1) {
        if ((strands.vertices_fixed[vtx >> 5] >> @intCast(vtx & 31)) & 1 != 0)
            continue;
        const strand = vtx % strands.num_strands;
        const material = (strands.strand_material_index[strand >> 2] >> @intCast((strand & 3) << 3)) & 0xff;
        if (strands.materials[material].enable_grid == 0)
            continue;
        const p = previous_positions[vtx].position;
        if (std.math.isNan(p.x) or std.math.isNan(p.y) or std.math.isNan(p.z)) {
            std.debug.print("applyGrid saw the NaN position {any} of vertex {d}\n", .{ p, vtx });
            return error.NanGridPosition;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
// The layers
//////////////////////////////////////////////////////////////////////////////////////////

test "HairShaderParity: registerHairShaders registers the hair shaders of Jolt" {
    // The names of HairShaders::Init, then names that are not hair shaders
    const names = [_][*:0]const u8{
        "HairTeleport",        "HairApplyDeltaTransform",      "HairSkinVertices",             "HairSkinRoots",
        "HairApplyGlobalPose", "HairCalculateCollisionPlanes", "HairGridClear",                "HairGridAccumulate",
        "HairGridNormalize",   "HairIntegrate",                "HairUpdateRoots",              "HairUpdateStrands",
        "HairUpdateVelocity",  "HairUpdateVelocityIntegrate",  "HairCalculateRenderPositions", "TestCompute",
        "TestCompute2",        "NonExistingShader",            "hairteleport",
    };
    const num_hair_shaders = 15;

    // Jolt: a system with only HairRegisterShaders, then HairShaders::Init finds every shader
    var jolt_found: [names.len]u32 = undefined;
    var jolt_init_found: [32]u32 = undefined;
    const num_members = jolt_compute_hair_shaders(&names, names.len, &jolt_found, &jolt_init_found);
    try std.testing.expectEqual(num_hair_shaders, num_members);
    for (jolt_init_found[0..num_members]) |found| try std.testing.expectEqual(1, found);

    // Zolt: a system with only registerHairShaders has exactly these 15 shaders
    var system = try zolt.ComputeSystemCPU.create(std.testing.allocator);
    defer system.deinit();
    try zolt.registerHairShaders(zolt.virtual.downcast(zolt.ComputeSystemCPU, system.get().?));
    try std.testing.expectEqual(num_hair_shaders, zolt.virtual.downcast(zolt.ComputeSystemCPU, system.get().?).shader_registry.count());
    for (names, jolt_found, 0..) |name, jolt, i| {
        var zolt_found: u32 = 1;
        if (system.get().?.createComputeShader(std.mem.span(name), HairStructs.hair_per_vertex_batch, .{})) |shader| {
            var s = shader;
            s.deinit();
        } else |err| switch (err) {
            error.ShaderNotFound => zolt_found = 0,
            else => |e| return e,
        }
        try std.testing.expectEqual(@intFromBool(i < num_hair_shaders), jolt);
        try std.testing.expectEqual(jolt, zolt_found);
    }
}

test "HairShaderParity: HairCalculateCollisionPlanes" {
    const allocator = std.testing.allocator;
    var dispatcher = try C1a.initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairCalculateCollisionPlanes" };
    var rng: fw.Rng = .{ .state = 0xc0111de5 };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        const num_strands = rng.intRange(u32, 1, 50);
        const num_vertices = num_strands * rng.intRange(u32, 1, 10);
        var context = [_]HairUpdateContext{C1a.randomContext(&rng)};
        context[0].num_strands = num_strands;
        context[0].num_vertices = num_vertices;
        var iteration_context: [1]HairIterationContext = undefined;

        const collision = try Collision.init(arena, &rng);
        const positions = try arena.alloc(HairPosition, num_vertices);
        for (positions) |*p| p.* = .{ .position = randomFloat3(&rng, 4), .rotation = randomBits(float4, &rng) };
        const collision_planes = try fill(arena, HairCollisionPlane, num_vertices, &rng, garbageCollisionPlane);

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            C1a.iterationContextBinding(&rng, &iteration_context, false),
            C1a.readBinding(&rng, "gPositions", HairPosition, positions),
            C1a.readBinding(&rng, "gShapePlanes", float4, collision.planes),
            C1a.readBinding(&rng, "gShapeVertices", float3, collision.vertices),
            C1a.readBinding(&rng, "gShapeIndices", u32, collision.indices),
            .of("gCollisionPlanes", .rw_buffer, HairCollisionPlane, collision_planes),
        };
        try C1a.dispatchRandom(&dispatcher, &checker, &rng, zolt.HairCalculateCollisionPlanes, HairStructs.hair_per_vertex_batch, num_vertices - num_strands, &bindings, .floats);
    }
    try checker.finish();
}

fn garbageCollisionPlane(rng: *fw.Rng) HairCollisionPlane {
    // The shape index stays below 2^23: compared as float bits it is never a NaN
    var plane = randomBits(HairCollisionPlane, rng);
    plane.shape_index &= 0x7fffff;
    return plane;
}

test "HairShaderParity: HairIntegrate" {
    const allocator = std.testing.allocator;
    var dispatcher = try C1a.initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairIntegrate" };
    var rng: fw.Rng = .{ .state = 0x1a7e9a7e };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        // In a grid round the materials may enable the grid, so the positions are never NaN and the dispatch has a
        // single row in y (UB11)
        const grid_round = rng.next() % 4 != 0;
        const strands = try Strands.init(arena, &rng);
        setMaterials(&rng, strands, false, grid_round);
        var context = [_]HairUpdateContext{timeContext(&rng, strands)};
        const grid = try Grid.init(arena, &rng, &context[0]);
        var iteration_context: [1]HairIterationContext = undefined;

        const positions = try arena.alloc(HairPosition, strands.num_vertices);
        for (positions) |*p| p.* = if (grid_round)
            .{ .position = C1a.gridPosition(&rng, grid.bounds_min, grid.bounds_size), .rotation = randomRotation(&rng) }
        else
            C1a.randomPosition(&rng);
        const velocities = try fill(arena, HairVelocity, strands.num_vertices, &rng, C1a.randomVelocity);
        const previous_positions = try fill(arena, HairPosition, strands.num_vertices, &rng, C1a.garbagePosition);

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            C1a.iterationContextBinding(&rng, &iteration_context, false),
            C1a.readBinding(&rng, "gVerticesFixed", u32, strands.vertices_fixed),
            C1a.readBinding(&rng, "gStrandFractions", u32, strands.strand_fractions),
            C1a.readBinding(&rng, "gNeutralDensity", f32, grid.neutral_density),
            C1a.readBinding(&rng, "gVelocityAndDensity", float4, grid.velocity_and_density),
            C1a.readBinding(&rng, "gStrandMaterialIndex", u32, strands.strand_material_index),
            C1a.readBinding(&rng, "gMaterials", HairMaterial, strands.materials),
            C1a.readBinding(&rng, "gVelocities", HairVelocity, velocities),
            .of("gPositions", .rw_buffer, HairPosition, positions),
            .of("gPreviousPositions", .rw_buffer, HairPosition, previous_positions),
        };
        try C1a.dispatchRandomRows(&dispatcher, &checker, &rng, zolt.HairIntegrate, HairStructs.hair_per_vertex_batch, strands.numThreadsSkipRoots(), &bindings, .floats, !grid_round);
        try expectNoNanGridPosition(strands, previous_positions);
    }
    try checker.finish();
}

test "HairShaderParity: HairUpdateStrands" {
    const allocator = std.testing.allocator;
    var dispatcher = try C1a.initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairUpdateStrands" };
    var rng: fw.Rng = .{ .state = 0x57a2d5e7 };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        // Strands of up to `rows` vertices, vertex `v` of strand `s` has index `v * num_strands + s`
        const num_strands = rng.intRange(u32, 1, 70);
        const rows = rng.intRange(u32, 2, 16);
        const num_vertices = num_strands * rows;
        const num_materials = rng.intRange(u32, 1, 8);

        const counts = try arena.alloc(u32, num_strands);
        for (counts) |*count| count.* = if (rng.next() % 8 == 0) rng.intRange(u32, 0, 1) else rng.intRange(u32, 2, rows);
        const strand_vertex_counts = try arena.alloc(u32, (num_strands + 3) / 4);
        @memset(strand_vertex_counts, 0);
        for (counts, 0..) |count, s| strand_vertex_counts[s >> 2] |= count << @intCast((s & 3) << 3);

        const vertices_fixed = try arena.alloc(u32, (num_vertices + 31) / 32);
        const fixed_mode = rng.next() % 4;
        for (vertices_fixed) |*f| f.* = switch (fixed_mode) {
            0 => 0,
            1 => rng.next() & rng.next(), // A quarter of the vertices is fixed
            else => rng.next(),
        };
        if (fixed_mode == 3) {
            // Hair's usual case: the roots are fixed
            for (0..num_strands) |s| vertices_fixed[s >> 5] |= @as(u32, 1) << @intCast(s & 31);
        }

        // A byte per vertex. A vertex in the middle of a strand (1 to count - 2) is never 255, see the file header.
        const strand_fractions = try arena.alloc(u32, (num_vertices + 3) / 4);
        for (strand_fractions) |*f| f.* = rng.next();
        for (counts, 0..) |count, s| {
            var row: u32 = 1;
            while (row + 1 < count) : (row += 1) {
                const v = row * num_strands + @as(u32, @intCast(s));
                const shift: u5 = @intCast((v & 3) << 3);
                if ((strand_fractions[v >> 2] >> shift) & 0xff == 0xff)
                    strand_fractions[v >> 2] &= ~(@as(u32, 1) << shift); // 254
            }
        }

        const strand_material_index = try arena.alloc(u32, (num_strands + 3) / 4);
        for (strand_material_index) |*m| {
            var packed_indices: u32 = 0;
            for (0..4) |i| packed_indices |= rng.intRange(u32, 0, num_materials - 1) << @intCast(8 * i);
            m.* = packed_indices;
        }
        const materials = try arena.alloc(HairMaterial, num_materials);
        for (materials) |*m| m.* = simulationMaterial(&rng, false, true);

        const initial_positions = try arena.alloc(float3, num_vertices);
        for (initial_positions) |*p| p.* = randomFloat3(&rng, 3);
        const omega0s = try fill(arena, u32, num_vertices, &rng, C1a.compressedQuat);
        const initial_lengths = try arena.alloc(f32, num_vertices);
        for (initial_lengths) |*l| l.* = switch (rng.next() % 16) {
            0 => 0,
            1 => specialFloat(&rng, 1),
            else => rng.float(0.01, 0.5),
        };

        // Positions: along a chain from the root with some noise (the constraints are close to their rest state), or
        // anywhere
        const positions = try arena.alloc(HairPosition, num_vertices);
        const chain = rng.next() % 2 == 0;
        for (0..num_strands) |s| {
            const direction = C1a.unitQuat(&rng).mulVec3(Vec3.axisZ());
            var p = initial_positions[s];
            for (0..rows) |row| {
                const v = row * num_strands + s;
                if (chain) {
                    positions[v] = .{ .position = p.add(randomFloat3(&rng, 0.05)), .rotation = randomRotation(&rng) };
                    const step = direction.mulScalar(rng.float(0.01, 0.5));
                    p = p.add(.init(step.getX(), step.getY(), step.getZ()));
                } else {
                    positions[v] = C1a.randomPosition(&rng);
                }
            }
        }

        var context = [_]HairUpdateContext{C1a.randomContext(&rng)};
        context[0].num_strands = num_strands;
        var iteration_context: [1]HairIterationContext = undefined;

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            C1a.iterationContextBinding(&rng, &iteration_context, false),
            C1a.readBinding(&rng, "gVerticesFixed", u32, vertices_fixed),
            C1a.readBinding(&rng, "gStrandFractions", u32, strand_fractions),
            C1a.readBinding(&rng, "gInitialPositions", float3, initial_positions),
            C1a.readBinding(&rng, "gOmega0s", u32, omega0s),
            C1a.readBinding(&rng, "gInitialLengths", f32, initial_lengths),
            C1a.readBinding(&rng, "gStrandVertexCounts", u32, strand_vertex_counts),
            C1a.readBinding(&rng, "gStrandMaterialIndex", u32, strand_material_index),
            C1a.readBinding(&rng, "gMaterials", HairMaterial, materials),
            .of("gPositions", .rw_buffer, HairPosition, positions),
        };
        try C1a.dispatchRandom(&dispatcher, &checker, &rng, zolt.HairUpdateStrands, HairStructs.hair_per_strand_batch, num_strands, &bindings, .floats);
    }
    try checker.finish();
}

test "HairShaderParity: HairUpdateVelocity" {
    const allocator = std.testing.allocator;
    var dispatcher = try C1a.initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairUpdateVelocity" };
    var rng: fw.Rng = .{ .state = 0x0e10c17e };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        const strands = try Strands.init(arena, &rng);
        setMaterials(&rng, strands, false, true);
        var context = [_]HairUpdateContext{timeContext(&rng, strands)};
        var iteration_context: [1]HairIterationContext = undefined;

        const initial_positions = try fill(arena, float3, strands.num_vertices, &rng, C1a.position5);
        const initial_bishops = try fill(arena, u32, strands.num_vertices, &rng, C1a.compressedQuat);
        const global_pose_transforms = try fill(arena, HairGlobalPoseTransform, strands.num_strands, &rng, C1a.randomGlobalPoseTransform);
        const positions = try fill(arena, HairPosition, strands.num_vertices, &rng, C1a.randomPosition);
        const previous_positions = try arena.alloc(HairPosition, strands.num_vertices);
        for (previous_positions, positions) |*prev, pos| prev.* = if (rng.next() % 4 != 0)
            .{ .position = pos.position.add(randomFloat3(&rng, 0.1)), .rotation = randomRotation(&rng) }
        else
            C1a.randomPosition(&rng);
        const collision_shapes = try collisionShapes(arena, &rng, false);
        const collision_planes = try collisionPlanes(arena, &rng, positions, collision_shapes.len, false);
        const velocities = try fill(arena, HairVelocity, strands.num_vertices, &rng, C1a.garbageVelocity);

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            velocityIterationContext(&rng, &iteration_context, false),
            C1a.readBinding(&rng, "gVerticesFixed", u32, strands.vertices_fixed),
            C1a.readBinding(&rng, "gStrandFractions", u32, strands.strand_fractions),
            C1a.readBinding(&rng, "gInitialPositions", float3, initial_positions),
            C1a.readBinding(&rng, "gInitialBishops", u32, initial_bishops),
            C1a.readBinding(&rng, "gStrandMaterialIndex", u32, strands.strand_material_index),
            C1a.readBinding(&rng, "gMaterials", HairMaterial, strands.materials),
            C1a.readBinding(&rng, "gPreviousPositions", HairPosition, previous_positions),
            C1a.readBinding(&rng, "gGlobalPoseTransforms", HairGlobalPoseTransform, global_pose_transforms),
            C1a.readBinding(&rng, "gCollisionShapes", HairCollisionShape, collision_shapes),
            C1a.readBinding(&rng, "gCollisionPlanes", HairCollisionPlane, collision_planes),
            .of("gPositions", .rw_buffer, HairPosition, positions),
            .of("gVelocities", .rw_buffer, HairVelocity, velocities),
        };
        try C1a.dispatchRandom(&dispatcher, &checker, &rng, zolt.HairUpdateVelocity, HairStructs.hair_per_vertex_batch, strands.numThreadsSkipRoots(), &bindings, .floats);
    }
    try checker.finish();
}

test "HairShaderParity: HairUpdateVelocityIntegrate" {
    const allocator = std.testing.allocator;
    var dispatcher = try C1a.initDispatcher(allocator);
    defer dispatcher.deinit();
    var checker: fw.Checker = .{ .name = "HairShaderParity HairUpdateVelocityIntegrate" };
    var rng: fw.Rng = .{ .state = 0x7e1a7e57 };
    var arena_state = std.heap.ArenaAllocator.init(allocator);
    defer arena_state.deinit();

    for (0..num_rounds) |_| {
        _ = arena_state.reset(.retain_capacity);
        const arena = arena_state.allocator();

        // In a grid round the materials may enable the grid, so every input of the position before applyGrid is
        // finite and the dispatch has a single row in y (UB11). The other rounds use special floats everywhere.
        const grid_round = rng.next() % 4 != 0;
        const clean = grid_round;
        const strands = try Strands.init(arena, &rng);
        setMaterials(&rng, strands, clean, grid_round);
        var context = [_]HairUpdateContext{timeContext(&rng, strands)};
        const grid = try Grid.init(arena, &rng, &context[0]);
        var iteration_context: [1]HairIterationContext = undefined;

        const initial_positions = if (clean)
            try fill(arena, float3, strands.num_vertices, &rng, cleanPosition)
        else
            try fill(arena, float3, strands.num_vertices, &rng, C1a.position5);
        const initial_bishops = try fill(arena, u32, strands.num_vertices, &rng, C1a.compressedQuat);
        const global_pose_transforms = if (clean)
            try fill(arena, HairGlobalPoseTransform, strands.num_strands, &rng, cleanGlobalPoseTransform)
        else
            try fill(arena, HairGlobalPoseTransform, strands.num_strands, &rng, C1a.randomGlobalPoseTransform);
        const positions = try arena.alloc(HairPosition, strands.num_vertices);
        for (positions) |*p| p.* = if (clean)
            .{ .position = grid.finitePosition(&rng), .rotation = randomRotation(&rng) }
        else
            C1a.randomPosition(&rng);
        // The previous positions and the rotations only change the velocity before applyGrid: special floats are fine
        const previous_positions = try arena.alloc(HairPosition, strands.num_vertices);
        for (previous_positions, positions) |*prev, pos| prev.* = if (rng.next() % 4 != 0)
            .{ .position = pos.position.add(randomFloat3(&rng, 0.1)), .rotation = randomRotation(&rng) }
        else
            C1a.randomPosition(&rng);
        const collision_shapes = try collisionShapes(arena, &rng, clean);
        const collision_planes = try collisionPlanes(arena, &rng, positions, collision_shapes.len, clean);

        const bindings = [_]BindingDesc{
            .of("gContext", .constant_buffer, HairUpdateContext, &context),
            velocityIterationContext(&rng, &iteration_context, clean),
            C1a.readBinding(&rng, "gVerticesFixed", u32, strands.vertices_fixed),
            C1a.readBinding(&rng, "gStrandFractions", u32, strands.strand_fractions),
            C1a.readBinding(&rng, "gInitialPositions", float3, initial_positions),
            C1a.readBinding(&rng, "gInitialBishops", u32, initial_bishops),
            C1a.readBinding(&rng, "gNeutralDensity", f32, grid.neutral_density),
            C1a.readBinding(&rng, "gVelocityAndDensity", float4, grid.velocity_and_density),
            C1a.readBinding(&rng, "gStrandMaterialIndex", u32, strands.strand_material_index),
            C1a.readBinding(&rng, "gMaterials", HairMaterial, strands.materials),
            C1a.readBinding(&rng, "gGlobalPoseTransforms", HairGlobalPoseTransform, global_pose_transforms),
            C1a.readBinding(&rng, "gCollisionShapes", HairCollisionShape, collision_shapes),
            C1a.readBinding(&rng, "gCollisionPlanes", HairCollisionPlane, collision_planes),
            .of("gPreviousPositions", .rw_buffer, HairPosition, previous_positions),
            .of("gPositions", .rw_buffer, HairPosition, positions),
        };
        try C1a.dispatchRandomRows(&dispatcher, &checker, &rng, zolt.HairUpdateVelocityIntegrate, HairStructs.hair_per_vertex_batch, strands.numThreadsSkipRoots(), &bindings, .floats, !grid_round);
        try expectNoNanGridPosition(strands, previous_positions);
    }
    try checker.finish();
}
