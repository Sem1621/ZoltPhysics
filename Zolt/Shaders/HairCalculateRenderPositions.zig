//! Port of: Jolt/Shaders/HairCalculateRenderPositions.hlsl, Jolt/Shaders/HairCalculateRenderPositionsBindings.h, Jolt/Shaders/HairCalculateRenderPositions.h
//! Status: complete
//!
//! The hair shader that skins the render vertices to the simulated vertices (Docs/Zolt/RemainingFeaturesArchitecture.md
//! R12).
//! - `skinRenderVertex` ports the helper header HairCalculateRenderPositions.h. In Jolt it is a member of the wrapper
//!   class and reads the buffers directly; Zolt passes the bindings.
//! - The binding override macro JPH_SHADER_BIND_RENDER_POSITIONS is not ported (section 8): `gRenderPositions` has the
//!   default buffer format.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderQuat = @import("ShaderQuat.zig");
const HairStructs = @import("HairStructs.zig");
const float3 = hlsl.float3;
const JPH_float3 = ShaderCore.JPH_float3;
const HairPosition = HairStructs.HairPosition;
const HairSVertexInfluence = HairStructs.HairSVertexInfluence;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairCalculateRenderPositions";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairCalculateRenderPositions): the field names
/// are the binding names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gSVertexInfluences: hlsl.Buffer(HairSVertexInfluence) = .{},
    gPositions: hlsl.Buffer(HairPosition) = .{},
    /// JPH_SHADER_BIND_RENDER_POSITIONS(gRenderPositions), by default a JPH_float3 RW buffer
    gRenderPositions: hlsl.RWBuffer(JPH_float3) = .{},
};

/// SkinRenderVertex (HairCalculateRenderPositions.h)
fn skinRenderVertex(b: *const Bindings, vertex_index: u32) float3 {
    // Calculating resulting render position
    var out_position = float3.init(0, 0, 0);
    var idx = vertex_index *% HairStructs.hair_num_s_vertex_influences;
    const idx_end = idx +% HairStructs.hair_num_s_vertex_influences;
    while (idx < idx_end) : (idx += 1) {
        const inf = b.gSVertexInfluences.get(idx);
        const sim_vtx = b.gPositions.get(inf.vertex_index);
        out_position = out_position.add(sim_vtx.position.add(ShaderQuat.quatMulVec3(sim_vtx.rotation, inf.relative_position)).mulScalar(inf.weight));
    }
    return out_position;
}

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerRenderVertexBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    // Check if this is a valid vertex
    const vtx = tid.x;
    if (vtx >= b.gContext.value.num_render_vertices)
        return;

    const out_position = skinRenderVertex(b, vtx);

    // Copy the vertex position to the output buffer
    b.gRenderPositions.set(vtx, out_position);
}
