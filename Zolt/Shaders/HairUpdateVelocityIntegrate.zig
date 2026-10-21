//! Port of: Jolt/Shaders/HairUpdateVelocityIntegrate.hlsl, Jolt/Shaders/HairUpdateVelocityIntegrateBindings.h
//! Status: complete
//!
//! The hair shader that runs HairUpdateVelocity and then HairIntegrate on every simulated vertex in one pass, so the
//! velocity stays a local variable (Docs/Zolt/RemainingFeaturesArchitecture.md R12).
//! - The helpers of HairIntegrate.h and HairUpdateVelocity.h are in Zolt/Shaders/HairIntegrate.zig and
//!   Zolt/Shaders/HairUpdateVelocity.zig. They read the buffers through this shader's bindings.
//! - `uint3(grid_pos)` in `applyGrid` is undefined in Jolt for a NaN grid position (UB11 of R20, see
//!   HairIntegrate.zig). Here the position has the global pose and the collision applied before it reaches the grid.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderQuat = @import("ShaderQuat.zig");
const HairStructs = @import("HairStructs.zig");
const HairCommon = @import("HairCommon.zig");
const HairApplyGlobalPose = @import("HairApplyGlobalPose.zig");
const HairIntegrate = @import("HairIntegrate.zig");
const HairUpdateVelocity = @import("HairUpdateVelocity.zig");
const JPH_float = ShaderCore.JPH_float;
const JPH_float3 = ShaderCore.JPH_float3;
const JPH_float4 = ShaderCore.JPH_float4;
const JPH_uint = ShaderCore.JPH_uint;
const HairCollisionPlane = HairStructs.HairCollisionPlane;
const HairCollisionShape = HairStructs.HairCollisionShape;
const HairGlobalPoseTransform = HairStructs.HairGlobalPoseTransform;
const HairMaterial = HairStructs.HairMaterial;
const HairPosition = HairStructs.HairPosition;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairUpdateVelocityIntegrate";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairUpdateVelocityIntegrate): the field names
/// are the binding names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gVerticesFixed: hlsl.Buffer(JPH_uint) = .{},
    gStrandFractions: hlsl.Buffer(JPH_uint) = .{},
    gInitialPositions: hlsl.Buffer(JPH_float3) = .{},
    gInitialBishops: hlsl.Buffer(JPH_uint) = .{},
    gNeutralDensity: hlsl.Buffer(JPH_float) = .{},
    gVelocityAndDensity: hlsl.Buffer(JPH_float4) = .{},
    gStrandMaterialIndex: hlsl.Buffer(JPH_uint) = .{},
    gMaterials: hlsl.Buffer(HairMaterial) = .{},
    gGlobalPoseTransforms: hlsl.Buffer(HairGlobalPoseTransform) = .{},
    gCollisionShapes: hlsl.Buffer(HairCollisionShape) = .{},
    gCollisionPlanes: hlsl.Buffer(HairCollisionPlane) = .{},
    gPreviousPositions: hlsl.RWBuffer(HairPosition) = .{},
    gPositions: hlsl.RWBuffer(HairPosition) = .{},
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
    const initial_pos = b.gInitialPositions.get(vtx);
    const initial_bishop = ShaderQuat.quatDecompress(b.gInitialBishops.get(vtx));
    const global_pose_transform = b.gGlobalPoseTransforms.get(strand_idx);
    var pos = b.gPositions.get(vtx);
    const prev_pos = b.gPreviousPositions.get(vtx);

    // HairUpdateVelocity shader
    HairApplyGlobalPose.applyGlobalPose(&pos, initial_pos, initial_bishop, global_pose_transform, material, strand_fraction);
    var vel = HairUpdateVelocity.applyCollisionAndUpdateVelocity(b, vtx, &pos, prev_pos, material, strand_fraction); // Keeps velocity as a local variable
    HairUpdateVelocity.limitVelocity(&vel, material);

    // HairIntegrate shader
    b.gPreviousPositions.set(vtx, pos);
    HairIntegrate.applyGrid(b, pos, &vel, material, strand_fraction);
    HairIntegrate.integrate(b, &pos, vel, material, strand_fraction);
    b.gPositions.set(vtx, pos);
}
