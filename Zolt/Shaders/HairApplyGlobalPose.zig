//! Port of: Jolt/Shaders/HairApplyGlobalPose.hlsl, Jolt/Shaders/HairApplyGlobalPoseBindings.h, Jolt/Shaders/HairApplyGlobalPose.h
//! Status: complete
//!
//! The hair shader that only applies the global pose to the simulated vertices (Docs/Zolt/RemainingFeaturesArchitecture.md
//! R12). Hair dispatches it for a global pose only update.
//! - `applyGlobalPose` ports the helper header HairApplyGlobalPose.h. HairUpdateVelocity.h includes that header, so the
//!   function is public: Zolt/Shaders/HairUpdateVelocity.zig calls it.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderQuat = @import("ShaderQuat.zig");
const HairStructs = @import("HairStructs.zig");
const HairCommon = @import("HairCommon.zig");
const float3 = hlsl.float3;
const float4 = hlsl.float4;
const JPH_float3 = ShaderCore.JPH_float3;
const JPH_uint = ShaderCore.JPH_uint;
const JPH_Quat = ShaderCore.JPH_Quat;
const HairGlobalPoseTransform = HairStructs.HairGlobalPoseTransform;
const HairMaterial = HairStructs.HairMaterial;
const HairPosition = HairStructs.HairPosition;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairApplyGlobalPose";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairApplyGlobalPose): the field names are the
/// binding names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gVerticesFixed: hlsl.Buffer(JPH_uint) = .{},
    gStrandFractions: hlsl.Buffer(JPH_uint) = .{},
    gInitialPositions: hlsl.Buffer(JPH_float3) = .{},
    gInitialBishops: hlsl.Buffer(JPH_uint) = .{},
    gStrandMaterialIndex: hlsl.Buffer(JPH_uint) = .{},
    gMaterials: hlsl.Buffer(HairMaterial) = .{},
    gGlobalPoseTransforms: hlsl.Buffer(HairGlobalPoseTransform) = .{},
    gPositions: hlsl.RWBuffer(HairPosition) = .{},
};

/// ApplyGlobalPose (HairApplyGlobalPose.h). `pos` is the JPH_IN_OUT parameter `ioPos`.
pub fn applyGlobalPose(pos: *HairPosition, rest_position: float3, rest_orientation: JPH_Quat, global_pose_transform: HairGlobalPoseTransform, material: HairMaterial, strand_fraction: f32) void {
    // LERP between stored global pose and global pose skinned to the scalp
    const skin_factor = HairCommon.gradientSamplerSample(material.skin_global_pose, strand_fraction);
    var in_position = rest_position;
    in_position = in_position.add(global_pose_transform.position.add(ShaderQuat.quatMulVec3(global_pose_transform.rotation, in_position)).sub(in_position).mulScalar(skin_factor));
    var in_rotation = rest_orientation;
    in_rotation = in_rotation.add(ShaderQuat.quatMulQuat(global_pose_transform.rotation, in_rotation).sub(in_rotation).mulScalar(skin_factor));

    // LERP between simulated position and skinned position
    const pose_factor = HairCommon.gradientSamplerSample(material.global_pose, strand_fraction);
    pos.position = pos.position.add(in_position.sub(pos.position).mulScalar(pose_factor));
    pos.rotation = pos.rotation.add(in_rotation.sub(pos.rotation).mulScalar(pose_factor));
    pos.rotation = hlsl.normalize(pos.rotation);
}

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
    const initial_pos = b.gInitialPositions.get(vtx);
    const initial_bishop = ShaderQuat.quatDecompress(b.gInitialBishops.get(vtx));
    const global_pose_transform = b.gGlobalPoseTransforms.get(strand_idx);

    // Only apply global pose
    var pos: HairPosition = .{
        .position = float3.init(0, 0, 0),
        .rotation = float4.init(0, 0, 0, 0),
    };
    applyGlobalPose(&pos, initial_pos, initial_bishop, global_pose_transform, material, strand_fraction);

    // Write back vertex
    b.gPositions.set(vtx, pos);
}
