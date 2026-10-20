//! Port of: Jolt/Shaders/HairTeleport.hlsl, Jolt/Shaders/HairTeleportBindings.h
//! Status: complete
//!
//! The hair shader that puts every vertex at its initial position and orientation with zero velocity
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R12). Hair dispatches it after a teleport.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const ShaderQuat = @import("ShaderQuat.zig");
const HairStructs = @import("HairStructs.zig");
const float3 = hlsl.float3;
const JPH_float3 = ShaderCore.JPH_float3;
const JPH_uint = ShaderCore.JPH_uint;
const HairPosition = HairStructs.HairPosition;
const HairVelocity = HairStructs.HairVelocity;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairTeleport";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairTeleport): the field names are the binding
/// names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gInitialPositions: hlsl.Buffer(JPH_float3) = .{},
    gInitialBishops: hlsl.Buffer(JPH_uint) = .{},
    gPositions: hlsl.RWBuffer(HairPosition) = .{},
    gVelocities: hlsl.RWBuffer(HairVelocity) = .{},
};

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerVertexBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    // Check if this is a valid vertex
    const vtx = tid.x;
    if (vtx >= b.gContext.value.num_vertices)
        return;

    // Initialize position based on the initial vertex data
    const pos: HairPosition = .{
        .position = b.gInitialPositions.get(vtx),
        .rotation = ShaderQuat.quatDecompress(b.gInitialBishops.get(vtx)),
    };
    b.gPositions.set(vtx, pos);

    // Initialize velocity to zero
    const vel: HairVelocity = .{
        .velocity = float3.init(0, 0, 0),
        .angular_velocity = float3.init(0, 0, 0),
    };
    b.gVelocities.set(vtx, vel);
}
