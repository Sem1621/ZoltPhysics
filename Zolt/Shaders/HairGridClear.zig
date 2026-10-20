//! Port of: Jolt/Shaders/HairGridClear.hlsl, Jolt/Shaders/HairGridClearBindings.h
//! Status: complete
//!
//! The hair shader that clears the velocity and density grid before HairGridAccumulate
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R12).

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const HairStructs = @import("HairStructs.zig");
const int4 = hlsl.int4;
const JPH_int4 = ShaderCore.JPH_int4;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairGridClear";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairGridClear): the field names are the binding
/// names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gVelocityAndDensity: hlsl.RWBuffer(JPH_int4) = .{},
};

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerGridCellBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const index = tid.x;
    if (index >= b.gContext.value.num_grid_points)
        return;

    b.gVelocityAndDensity.set(index, int4.init(0, 0, 0, 0));
}
