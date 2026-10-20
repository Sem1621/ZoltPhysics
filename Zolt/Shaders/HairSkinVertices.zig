//! Port of: Jolt/Shaders/HairSkinVertices.hlsl, Jolt/Shaders/HairSkinVerticesBindings.h
//! Status: complete
//!
//! The hair shader that skins the vertices of the scalp mesh with the joint matrices
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R12).

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderMat44 = @import("ShaderMat44.zig");
const HairStructs = @import("HairStructs.zig");
const float3 = hlsl.float3;
const JPH_float3 = ShaderCore.JPH_float3;
const JPH_Mat44 = ShaderCore.JPH_Mat44;
const HairSkinWeight = HairStructs.HairSkinWeight;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairSkinVertices";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairSkinVertices): the field names are the
/// binding names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gScalpVertices: hlsl.Buffer(JPH_float3) = .{},
    gScalpSkinWeights: hlsl.Buffer(HairSkinWeight) = .{},
    gScalpJointMatrices: hlsl.Buffer(JPH_Mat44) = .{},
    gScalpVerticesOut: hlsl.RWBuffer(JPH_float3) = .{},
};

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerVertexBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const c = &b.gContext.value;

    // Check if this is a valid vertex
    const vtx = tid.x;
    if (vtx >= c.num_skin_vertices)
        return;

    // Skin the vertex
    var v = float3.init(0, 0, 0);
    var w = vtx *% c.num_skin_weights_per_vertex;
    const w_end = w +% c.num_skin_weights_per_vertex;
    while (w < w_end) : (w += 1) {
        const sw = b.gScalpSkinWeights.get(w);
        if (sw.weight > 0.0)
            v = v.add(ShaderMat44.mat44Mul3x4Vec3(b.gScalpJointMatrices.get(sw.joint_idx), b.gScalpVertices.get(vtx)).mulScalar(sw.weight));
    }
    b.gScalpVerticesOut.set(vtx, v);
}
