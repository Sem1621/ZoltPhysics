//! Port of: Jolt/Shaders/HairGridNormalize.hlsl, Jolt/Shaders/HairGridNormalizeBindings.h
//! Status: complete
//!
//! The hair shader that converts the accumulated grid from fixed point to float and divides the velocity by the density
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R12). The cells keep the float bits in the int4 buffer (`asint`).

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const ShaderCore = @import("ShaderCore.zig");
const HairStructs = @import("HairStructs.zig");
const HairCommon = @import("HairCommon.zig");
const float4 = hlsl.float4;
const JPH_int4 = ShaderCore.JPH_int4;

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairGridNormalize";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairGridNormalize): the field names are the
/// binding names of the HLSL code
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

    // Convert from fixed point back to float and divide velocity by density to get average velocity
    var v = float4.fromInt4(b.gVelocityAndDensity.get(index)).mulScalar(HairCommon.fixed_to_float);
    const density = v.w;
    if (density > @as(f32, 1.0e-12)) {
        v.x /= density;
        v.y /= density;
        v.z /= density;
    }
    b.gVelocityAndDensity.set(index, hlsl.asint(v));
}
