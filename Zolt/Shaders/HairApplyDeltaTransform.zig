//! Port of: Jolt/Shaders/HairApplyDeltaTransform.hlsl, Jolt/Shaders/HairApplyDeltaTransformBindings.h
//! Status: complete
//!
//! The hair shader that moves the simulated vertices with the change of the hair's transform, as far as the material's
//! world transform influence says (Docs/Zolt/RemainingFeaturesArchitecture.md R12).

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderMat44 = @import("ShaderMat44.zig");
const ShaderQuat = @import("ShaderQuat.zig");
const HairStructs = @import("HairStructs.zig");
const HairCommon = @import("HairCommon.zig");
const float4 = hlsl.float4;
const JPH_uint = ShaderCore.JPH_uint;
const HairMaterial = HairStructs.HairMaterial;
const HairPosition = HairStructs.HairPosition;
const HairVelocity = HairStructs.HairVelocity;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairApplyDeltaTransform";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairApplyDeltaTransform): the field names are
/// the binding names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gVerticesFixed: hlsl.Buffer(JPH_uint) = .{},
    gStrandFractions: hlsl.Buffer(JPH_uint) = .{},
    gStrandMaterialIndex: hlsl.Buffer(JPH_uint) = .{},
    gMaterials: hlsl.Buffer(HairMaterial) = .{},
    gPositions: hlsl.RWBuffer(HairPosition) = .{},
    gVelocities: hlsl.RWBuffer(HairVelocity) = .{},
};

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerVertexBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const c = &b.gContext.value;

    // Check if this is a valid vertex
    const vtx = tid.x +% c.num_strands; // Skip the root of each strand, it's fixed
    if (vtx >= c.num_vertices)
        return;
    if (HairCommon.isVertexFixed(b.gVerticesFixed, vtx))
        return;

    // Load the material
    const strand_idx = vtx % c.num_strands;
    const material = b.gMaterials.get(HairCommon.getStrandMaterialIndex(b.gStrandMaterialIndex, strand_idx));

    // Load the vertex
    const strand_fraction = HairCommon.getVertexStrandFraction(b.gStrandFractions, vtx);
    var pos = b.gPositions.get(vtx);
    var vel = b.gVelocities.get(vtx);

    // Transform the position so that it stays in the same place in world space (if influence is 1)
    const influence = HairCommon.gradientSamplerSample(material.world_transform_influence, strand_fraction);
    pos.position = pos.position.add(ShaderMat44.mat44Mul3x4Vec3(c.delta_transform, pos.position).sub(pos.position).mulScalar(influence));

    // Linear interpolate the rotation based on the influence
    pos.rotation = hlsl.normalize(ShaderQuat.quatMulQuat(c.delta_transform_quat.mulScalar(influence).add(float4.init(0, 0, 0, 1.0 - influence)), pos.rotation));

    // Transform velocities
    vel.velocity = ShaderMat44.mat44Mul3x3Vec3(c.delta_transform, vel.velocity);
    vel.angular_velocity = ShaderMat44.mat44Mul3x3Vec3(c.delta_transform, vel.angular_velocity);

    // Write back vertex
    b.gPositions.set(vtx, pos);
    b.gVelocities.set(vtx, vel);
}
