//! Port of: Jolt/Shaders/HairCommon.h
//! Status: complete
//!
//! Helpers of the hair shaders, a literal port of the HLSL code with the semantics of HLSLToCPP
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R11, R12).
//! - HairCommon.h includes ShaderMath.h, ShaderMat44.h, ShaderQuat.h, ShaderPlane.h and ShaderVec3.h. A hair shader
//!   imports those files itself.
//! - In Jolt the functions are members of the shader's wrapper class, so they read the constants of `gContext`
//!   directly. Zolt passes the block: `gridPositionToIndexAndFraction(&b.gContext.value, position)`.
//! - JPH_SHADER_BUFFER(JPH_uint) parameters are `hlsl.Buffer(u32)` bindings.
//! - GridPositionToIndexAndFraction has two JPH_OUT parameters, so it returns `IndexAndFraction`.
//! - `checkGridPoints` is Zolt's (R20 UB30). HairGridAccumulate and ApplyGrid call it before they index the grid.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const float3 = hlsl.float3;
const float4 = hlsl.float4;
const uint3 = hlsl.uint3;
const HairUpdateContext = @import("HairStructs.zig").HairUpdateContext;

/// JPH_SHADER_CONSTANT(int, cFloatToFixed, 1 << 10): the density and velocity fields are stored in fixed point while
/// accumulating, this constant converts from float to fixed point. A comptime_int like the constants of
/// HairStructs.zig (`float4 * cFloatToFixed` converts it to float).
pub const float_to_fixed = 1 << 10;

/// JPH_SHADER_CONSTANT(float, cFixedToFloat, 1.0f / float(cFloatToFixed))
pub const fixed_to_float: f32 = @as(f32, 1.0) / @as(f32, float_to_fixed);

/// IsVertexFixed
pub fn isVertexFixed(vertex_fixed: hlsl.Buffer(u32), vertex_index: u32) bool {
    return (vertex_fixed.get(vertex_index >> 5) & (@as(u32, 1) << @intCast(vertex_index & 31))) != 0;
}

/// GetVertexInvMass
pub fn getVertexInvMass(vertex_fixed: hlsl.Buffer(u32), vertex_index: u32) f32 {
    return if (isVertexFixed(vertex_fixed, vertex_index)) 0.0 else 1.0;
}

/// GetVertexStrandFraction
pub fn getVertexStrandFraction(strand_fractions: hlsl.Buffer(u32), vertex_index: u32) f32 {
    return @as(f32, @floatFromInt((strand_fractions.get(vertex_index >> 2) >> @intCast((vertex_index & 3) << 3)) & 0xff)) * (@as(f32, 1.0) / 255.0);
}

/// GetStrandVertexCount
pub fn getStrandVertexCount(strand_vertex_counts: hlsl.Buffer(u32), strand_index: u32) u32 {
    return (strand_vertex_counts.get(strand_index >> 2) >> @intCast((strand_index & 3) << 3)) & 0xff;
}

/// GetStrandMaterialIndex
pub fn getStrandMaterialIndex(strand_material_index: hlsl.Buffer(u32), strand_index: u32) u32 {
    return (strand_material_index.get(strand_index >> 2) >> @intCast((strand_index & 3) << 3)) & 0xff;
}

/// GradientSamplerSample
pub fn gradientSamplerSample(sampler: float4, strand_fraction: f32) f32 {
    return hlsl.min(sampler.w, hlsl.max(sampler.z, sampler.y + strand_fraction * sampler.x));
}

/// The out parameters of GridPositionToIndexAndFraction
pub const IndexAndFraction = struct {
    index: uint3,
    fraction: float3,
};

/// GridPositionToIndexAndFraction. `uint3(grid_pos)` is undefined in Jolt for a NaN grid position: a NaN position, or
/// 0 * inf when the simulation bounds have a zero-size axis (see hlsl.uint3.fromFloat3). The parity keeps them out.
pub fn gridPositionToIndexAndFraction(context: *const HairUpdateContext, position: float3) IndexAndFraction {
    // Get position in grid space
    const grid_pos = hlsl.min(hlsl.max(position.sub(context.grid_offset), float3.init(0, 0, 0)).mul(context.grid_scale), context.grid_size_min1);
    const index = hlsl.min(uint3.fromFloat3(grid_pos), context.grid_size_min2);
    const fraction = grid_pos.sub(float3.fromUint3(index));
    return .{ .index = index, .fraction = fraction };
}

/// GridIndexToBufferIndex
pub fn gridIndexToBufferIndex(context: *const HairUpdateContext, index: uint3) u32 {
    return index.x +% index.y *% context.grid_stride.y +% index.z *% context.grid_stride.z;
}

/// Zolt (R20 UB30): panics when one of the 8 grid points around a vertex is past the end of the grid
/// (`cNumGridPoints`), where Jolt reads or writes past the grid buffers. The 8 addresses are `adr_000`, `adr_100`,
/// `adr_010`, `adr_110` and the same 4 plus `cGridStride.z`. With a GridSize of 2 or more on each axis, and fewer
/// than 2^32 points, they are inside. On an axis of size 1 the index is 0, and the point at index 1 is in the next row
/// or slice. That point can be past the end. An axis of size 0 leaves no grid point at all. `shader` names the caller
/// in the message.
/// The check is explicit: the bounds check of the buffer slices panics only in safe builds, and R20 gives an input
/// where Jolt is undefined one behavior in every build mode.
pub fn checkGridPoints(context: *const HairUpdateContext, adr_000: u32, adr_100: u32, adr_010: u32, adr_110: u32, comptime shader: []const u8) void {
    const stride_z = context.grid_stride.z;
    for ([_]u32{ adr_000, adr_100, adr_010, adr_110, adr_000 +% stride_z, adr_100 +% stride_z, adr_010 +% stride_z, adr_110 +% stride_z }) |address| {
        if (address >= context.num_grid_points)
            @panic(shader ++ ": a vertex samples a grid point past the end of the grid");
    }
}
