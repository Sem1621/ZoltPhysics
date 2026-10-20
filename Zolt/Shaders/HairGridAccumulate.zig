//! Port of: Jolt/Shaders/HairGridAccumulate.hlsl, Jolt/Shaders/HairGridAccumulateBindings.h
//! Status: complete
//!
//! The hair shader that adds the velocity and density of every simulated vertex to the 8 grid cells around it, in fixed
//! point (Docs/Zolt/RemainingFeaturesArchitecture.md R12).
//! - The additions are atomic (`hlsl.atomicAdd`), like JPH_AtomicAdd. The CPU backend runs one thread at a time, so the
//!   order of the additions is the order of the threads. Integer addition wraps, so the result does not depend on the
//!   order anyway.
//! - `uint3(grid_pos)` of a NaN grid position is undefined in Jolt (UB11 of R20, see HairCommon.zig).

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const HairStructs = @import("HairStructs.zig");
const HairCommon = @import("HairCommon.zig");
const float3 = hlsl.float3;
const float4 = hlsl.float4;
const int4 = hlsl.int4;
const JPH_int4 = ShaderCore.JPH_int4;
const JPH_uint = ShaderCore.JPH_uint;
const HairPosition = HairStructs.HairPosition;
const HairVelocity = HairStructs.HairVelocity;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairGridAccumulate";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairGridAccumulate): the field names are the
/// binding names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gVerticesFixed: hlsl.Buffer(JPH_uint) = .{},
    gPositions: hlsl.Buffer(HairPosition) = .{},
    gVelocities: hlsl.Buffer(HairVelocity) = .{},
    gVelocityAndDensity: hlsl.RWBuffer(JPH_int4) = .{},
};

/// AtomicAddVelocityAndDensity
fn atomicAddVelocityAndDensity(b: *Bindings, index: u32, value: int4) void {
    const cell = b.gVelocityAndDensity.at(index);
    _ = hlsl.atomicAdd(&cell.x, value.x);
    _ = hlsl.atomicAdd(&cell.y, value.y);
    _ = hlsl.atomicAdd(&cell.z, value.z);
    _ = hlsl.atomicAdd(&cell.w, value.w);
}

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerVertexBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const c = &b.gContext.value;

    // Check that we are processing a valid vertex
    const vtx = tid.x +% c.num_strands; // Skip the root of each strand, it's fixed
    if (vtx >= c.num_vertices)
        return;
    if (HairCommon.isVertexFixed(b.gVerticesFixed, vtx))
        return;

    // Convert position to grid index and fraction
    const index_and_fraction = HairCommon.gridPositionToIndexAndFraction(c, b.gPositions.get(vtx).position);
    const index = index_and_fraction.index;
    const ma = index_and_fraction.fraction;
    const a = float3.init(1, 1, 1).sub(ma);

    // Get velocity
    const velocity_and_density = float4.fromFloat3(b.gVelocities.get(vtx).velocity, 1).mulScalar(HairCommon.float_to_fixed);

    // Calculate contribution of density and velocity for each cell. `(int4)round(w * v)` with a scalar weight w: HLSL's
    // `float * float4` is `float4 * float`.
    const stride = c.grid_stride;
    const adr_000 = HairCommon.gridIndexToBufferIndex(c, index);
    const adr_100 = adr_000 +% 1;
    const adr_010 = adr_000 +% stride.y;
    const adr_110 = adr_010 +% 1;
    atomicAddVelocityAndDensity(b, adr_000, int4.fromFloat4(hlsl.round(velocity_and_density.mulScalar(a.x * a.y * a.z))));
    atomicAddVelocityAndDensity(b, adr_100, int4.fromFloat4(hlsl.round(velocity_and_density.mulScalar(ma.x * a.y * a.z))));
    atomicAddVelocityAndDensity(b, adr_010, int4.fromFloat4(hlsl.round(velocity_and_density.mulScalar(a.x * ma.y * a.z))));
    atomicAddVelocityAndDensity(b, adr_110, int4.fromFloat4(hlsl.round(velocity_and_density.mulScalar(ma.x * ma.y * a.z))));
    atomicAddVelocityAndDensity(b, adr_000 +% stride.z, int4.fromFloat4(hlsl.round(velocity_and_density.mulScalar(a.x * a.y * ma.z))));
    atomicAddVelocityAndDensity(b, adr_100 +% stride.z, int4.fromFloat4(hlsl.round(velocity_and_density.mulScalar(ma.x * a.y * ma.z))));
    atomicAddVelocityAndDensity(b, adr_010 +% stride.z, int4.fromFloat4(hlsl.round(velocity_and_density.mulScalar(a.x * ma.y * ma.z))));
    atomicAddVelocityAndDensity(b, adr_110 +% stride.z, int4.fromFloat4(hlsl.round(velocity_and_density.mulScalar(ma.x * ma.y * ma.z))));
}

test "HairGridAccumulate: a NaN position goes to the cells of index 0 (UB11)" {
    const std = @import("std");

    // A 3 x 3 x 3 grid with scale 1, one strand of 2 vertices: the root is skipped, vertex 1 has a NaN x coordinate
    var context = std.mem.zeroes(HairStructs.HairUpdateContext);
    context.num_strands = 1;
    context.num_vertices = 2;
    context.grid_size_min2 = .init(1, 1, 1);
    context.grid_size_min1 = .init(2, 2, 2);
    context.grid_scale = .init(1, 1, 1);
    context.grid_stride = .init(1, 3, 9);
    const context_bytes: [@sizeOf(HairStructs.HairUpdateContext)]u8 align(16) = @bitCast(context);
    var fixed: [1]u32 align(16) = .{0};
    var positions: [2]HairPosition align(16) = .{
        .{ .position = .init(0, 0, 0), .rotation = .init(0, 0, 0, 1) },
        .{ .position = .init(std.math.nan(f32), 0.5, 0.5), .rotation = .init(0, 0, 0, 1) },
    };
    var velocities: [2]HairVelocity align(16) = @splat(.{ .velocity = .init(1, 2, 3), .angular_velocity = .init(0, 0, 0) });
    var grid: [27]int4 align(16) = @splat(.init(0, 0, 0, 0));

    var b: Bindings = .{};
    b.gContext.bind(&context_bytes);
    b.gVerticesFixed.bind(std.mem.sliceAsBytes(&fixed));
    b.gPositions.bind(std.mem.sliceAsBytes(&positions));
    b.gVelocities.bind(std.mem.sliceAsBytes(&velocities));
    b.gVelocityAndDensity.bind(std.mem.sliceAsBytes(&grid));
    main(&b, .init(0, 0, 0));

    // uint3(NaN) is 0 in Zolt (Jolt's result depends on the call site). The fraction is NaN, so every weight is NaN and
    // int(round(NaN)) adds 0x80000000 to each component of the 8 cells around index (0, 0, 0).
    const min = std.math.minInt(i32);
    for (grid, 0..) |cell, i| {
        const touched = i == 0 or i == 1 or i == 3 or i == 4 or i == 9 or i == 10 or i == 12 or i == 13;
        try std.testing.expect(cell.eql(if (touched) int4.splat(min) else int4.splat(0)));
    }
}
