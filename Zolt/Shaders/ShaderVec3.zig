//! Port of: Jolt/Shaders/ShaderVec3.h
//! Status: complete
//!
//! Vector helpers of the shaders, a literal port of the HLSL code with the semantics of HLSLToCPP
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R11, R12). It is not Zolt's Vec3.decompressUnitVector: the operation
//! order differs. `JPH_Foo` is `foo`.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const float2 = hlsl.float2;
const float3 = hlsl.float3;

/// Decompress a unit vector that was compressed with Vec3::CompressUnitVector (JPH_Vec3DecompressUnit)
pub fn vec3DecompressUnit(value: u32) float3 {
    const one_over_sqrt2: f32 = 0.70710678;
    const num_bits: u32 = 14;
    const mask: u32 = (1 << num_bits) - 1;
    const max_value: u32 = mask - 1; // Need odd number of buckets to quantize to or else we can't encode 0
    const half_max_value: i32 = @intCast(max_value >> 1);
    const scale: f32 = 2.0 * one_over_sqrt2 / @as(f32, @floatFromInt(max_value));

    // Restore two components
    const v2 = float2.init(@floatFromInt(@as(i32, @intCast(value & mask)) - half_max_value), @floatFromInt(@as(i32, @intCast((value >> num_bits) & mask)) - half_max_value)).mulScalar(scale);

    // Restore the highest component
    var v = float3.fromFloat2(v2, hlsl.sqrt(hlsl.max(1.0 - hlsl.dot(v2, v2), 0.0)));

    // Extract sign
    if ((value & 0x80000000) != 0)
        v = v.negate();

    // Swizzle the components in place
    const max_element = (value >> 29) & 3;
    v = if (max_element == 0) v.zxy() else (if (max_element == 1) v.xzy() else v);

    return v;
}
