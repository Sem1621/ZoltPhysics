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
