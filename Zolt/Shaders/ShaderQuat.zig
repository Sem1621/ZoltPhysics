//! Port of: Jolt/Shaders/ShaderQuat.h
//! Status: complete
//!
//! Quaternion helpers of the shaders, a literal port of the HLSL code with the semantics of HLSLToCPP
//! (Docs/Zolt/RemainingFeaturesArchitecture.md R11, R12). They are not Zolt's Quat functions: the operation order
//! differs. A quaternion is a float4: xyz = imaginary part, w = real part. `JPH_Foo` is `foo`.

const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const float3 = hlsl.float3;
const float4 = hlsl.float4;
const JPH_Quat = @import("ShaderCore.zig").JPH_Quat;

/// Rotate a vector by a quaternion (JPH_QuatMulVec3)
pub fn quatMulVec3(lhs: JPH_Quat, rhs: float3) float3 {
    const v_xyz = lhs.xyz();
    const v_yzx = lhs.yzx();
    const q_cross_p = rhs.yzx().mul(v_xyz).sub(v_yzx.mul(rhs)).yzx();
    const q_cross_q_cross_p = q_cross_p.yzx().mul(v_xyz).sub(v_yzx.mul(q_cross_p)).yzx();
    const v = q_cross_p.mulScalar(lhs.w).add(q_cross_q_cross_p);
    return rhs.add(v.add(v));
}

/// Multiply two quaternions (JPH_QuatMulQuat)
pub fn quatMulQuat(lhs: JPH_Quat, rhs: JPH_Quat) JPH_Quat {
    const x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
    const y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
    const z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
    const w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
    return .init(x, y, z, w);
}

/// Multiply a quaternion with real part 0 (given as its imaginary part) by a quaternion (JPH_QuatImaginaryMulQuat)
pub fn quatImaginaryMulQuat(lhs: float3, rhs: JPH_Quat) JPH_Quat {
    const x = lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y; // HLSL: +inLHS.x * inRHS.w + ...
    const y = -lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
    const z = lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w; // HLSL: +inLHS.x * inRHS.y - ...
    const w = -lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
    return .init(x, y, z, w);
}

/// Rotate the Z axis by a quaternion (JPH_QuatRotateAxisZ)
pub fn quatRotateAxisZ(rotation: JPH_Quat) float3 {
    return rotation.xyz().mulScalar(rotation.z + rotation.z).add(float3.init(rotation.y, -rotation.x, rotation.w).mulScalar(rotation.w + rotation.w)).sub(float3.init(0, 0, 1));
}

/// The conjugate of a quaternion (JPH_QuatConjugate)
pub fn quatConjugate(rotation: JPH_Quat) JPH_Quat {
    return .init(-rotation.x, -rotation.y, -rotation.z, rotation.w);
}

/// Decompress a quaternion that was compressed with Quat::CompressUnitQuat (JPH_QuatDecompress)
pub fn quatDecompress(value: u32) JPH_Quat {
    const one_over_sqrt2: f32 = 0.70710678;
    const num_bits: u32 = 9;
    const mask: u32 = (1 << num_bits) - 1;
    const max_value: u32 = mask - 1; // Need odd number of buckets to quantize to or else we can't encode 0
    const half_max_value: i32 = @intCast(max_value >> 1);
    const scale: f32 = 2.0 * one_over_sqrt2 / @as(f32, @floatFromInt(max_value));

    // Restore two components. The third component is `float(int(inValue >> 18) & cMask) - cHalfMaxValue`: the `&` with
    // the unsigned mask gives a uint, the subtraction of the int happens in float.
    const v3 = float3.init(
        @floatFromInt(@as(i32, @intCast(value & mask)) - half_max_value),
        @floatFromInt(@as(i32, @intCast((value >> num_bits) & mask)) - half_max_value),
        @as(f32, @floatFromInt(@as(u32, @bitCast(@as(i32, @intCast(value >> (2 * num_bits))))) & mask)) - @as(f32, @floatFromInt(half_max_value)),
    ).mulScalar(scale);

    // Restore the highest component
    var v = float4.fromFloat3(v3, hlsl.sqrt(hlsl.max(1.0 - hlsl.dot(v3, v3), 0.0)));

    // Extract sign
    if ((value & 0x80000000) != 0)
        v = v.negate();

    // Swizzle the components in place
    const max_element = (value >> 29) & 3;
    v = if (max_element == 0) v.wxyz() else (if (max_element == 1) v.xwyz() else (if (max_element == 2) v.xywz() else v));

    return v;
}

/// Create a quaternion from the three columns of a rotation matrix (JPH_QuatFromMat33)
pub fn quatFromMat33(col0: float3, col1: float3, col2: float3) JPH_Quat {
    const tr = col0.x + col1.y + col2.z;
    if (tr >= 0.0) {
        const s = hlsl.sqrt(tr + 1.0);
        const is = 0.5 / s;
        return .init(
            (col1.z - col2.y) * is,
            (col2.x - col0.z) * is,
            (col0.y - col1.x) * is,
            0.5 * s,
        );
    } else {
        if (col0.x > col1.y and col0.x > col2.z) {
            const s = hlsl.sqrt(col0.x - (col1.y + col2.z) + 1);
            const is = 0.5 / s;
            return .init(
                0.5 * s,
                (col1.x + col0.y) * is,
                (col0.z + col2.x) * is,
                (col1.z - col2.y) * is,
            );
        } else if (col1.y > col2.z) {
            const s = hlsl.sqrt(col1.y - (col2.z + col0.x) + 1);
            const is = 0.5 / s;
            return .init(
                (col1.x + col0.y) * is,
                0.5 * s,
                (col2.y + col1.z) * is,
                (col2.x - col0.z) * is,
            );
        } else {
            const s = hlsl.sqrt(col2.z - (col0.x + col1.y) + 1);
            const is = 0.5 / s;
            return .init(
                (col0.z + col2.x) * is,
                (col2.y + col1.z) * is,
                0.5 * s,
                (col0.y - col1.x) * is,
            );
        }
    }
}
