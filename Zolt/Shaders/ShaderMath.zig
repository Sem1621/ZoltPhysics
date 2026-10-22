//! Port of: Jolt/Shaders/ShaderMath.h
//! Status: complete
//!
//! Scalar and vector helpers of the shaders, a literal port of the HLSL code with the semantics of HLSLToCPP
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R11, R12). `JPH_Foo` is `foo`.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const float3 = hlsl.float3;

/// Calculate inValue^2 (JPH_Square)
pub fn square(value: f32) f32 {
    return value * value;
}

/// Get the closest point on a line segment defined by inA + x * inAB for x e [0, 1] to the origin
/// (JPH_GetClosestPointOnLine)
pub fn getClosestPointOnLine(a: float3, ab: float3) float3 {
    const v = hlsl.clamp(-hlsl.dot(a, ab) / hlsl.dot(ab, ab), 0.0, 1.0);
    return a.add(ab.mulScalar(v));
}
