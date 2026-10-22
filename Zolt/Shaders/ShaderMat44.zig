//! Port of: Jolt/Shaders/ShaderMat44.h
//! Status: complete
//!
//! Matrix helpers of the shaders, a literal port of the HLSL code with the semantics of HLSLToCPP
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R11, R12). They are not Zolt's Mat44 functions: the operation order
//! differs. `JPH_Foo` is `foo`.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const float3 = hlsl.float3;
const JPH_Mat44 = @import("ShaderCore.zig").JPH_Mat44;

/// Multiply a vector by a 3x4 matrix (rotation and translation) (JPH_Mat44Mul3x4Vec3)
pub fn mat44Mul3x4Vec3(lhs: JPH_Mat44, rhs: float3) float3 {
    return lhs.c[0].xyz().mulScalar(rhs.x).add(lhs.c[1].xyz().mulScalar(rhs.y)).add(lhs.c[2].xyz().mulScalar(rhs.z)).add(lhs.c[3].xyz());
}

/// Multiply a vector by the 3x3 part of a matrix (rotation only) (JPH_Mat44Mul3x3Vec3)
pub fn mat44Mul3x3Vec3(lhs: JPH_Mat44, rhs: float3) float3 {
    return lhs.c[0].xyz().mulScalar(rhs.x).add(lhs.c[1].xyz().mulScalar(rhs.y)).add(lhs.c[2].xyz().mulScalar(rhs.z));
}
