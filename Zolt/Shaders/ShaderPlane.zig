//! Port of: Jolt/Shaders/ShaderPlane.h
//! Status: complete
//!
//! Plane helpers of the shaders, a literal port of the HLSL code with the semantics of HLSLToCPP
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R11, R12). A plane is a float4: xyz = normal, w = constant. `JPH_Foo` is
//! `foo`.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const float3 = hlsl.float3;
const JPH_Plane = @import("ShaderCore.zig").JPH_Plane;

/// Create a plane from a point on the plane and its normal (JPH_PlaneFromPointAndNormal)
pub fn planeFromPointAndNormal(point: float3, normal: float3) JPH_Plane {
    return .fromFloat3(normal, -hlsl.dot(normal, point));
}

/// The normal of the plane (JPH_PlaneGetNormal)
pub fn planeGetNormal(plane: JPH_Plane) float3 {
    return plane.xyz();
}

/// The signed distance from a point to the plane (JPH_PlaneSignedDistance)
pub fn planeSignedDistance(plane: JPH_Plane, point: float3) f32 {
    return hlsl.dot(point, plane.xyz()) + plane.w;
}
