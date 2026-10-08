//! Port of: Jolt/Physics/Collision/Shape/ScaleHelpers.h (prototype, reduced)
//! Status: partial
//! Missing: IsUniformScaleXZ, MakeUniformScaleXZ
//!
//! `namespace ScaleHelpers` becomes this file, used as a namespace: `ScaleHelpers.isUniformScale(scale)`.

const zolt = @import("zolt");
const math = zolt.math;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const UVec4 = zolt.UVec4;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const PhysicsSettings = @import("../../PhysicsSettings.zig");

/// Minimum valid scale value. This is used to prevent division by zero when scaling a shape with a zero scale.
pub const min_scale: f32 = 1.0e-6;

/// The tolerance used to check if components of the scale vector are the same
pub const scale_tolerance_sq: f32 = 1.0e-8;

/// Test if a scale is identity
pub fn isNotScaled(scale: Vec3) bool {
    return scale.isClose(Vec3.one(), .{ .max_dist_sq = scale_tolerance_sq });
}

/// Test if a scale is uniform
pub fn isUniformScale(scale: Vec3) bool {
    return scale.swizzle(.y, .z, .x).isClose(scale, .{ .max_dist_sq = scale_tolerance_sq });
}

/// Scale the convex radius of an object
pub fn scaleConvexRadius(convex_radius: f32, scale: Vec3) f32 {
    return math.min(convex_radius * scale.abs().reduceMin(), PhysicsSettings.default_convex_radius);
}

/// Test if a scale flips an object inside out (which requires flipping all normals and polygon windings)
pub fn isInsideOut(scale: Vec3) bool {
    return (math.countBits(Vec3.less(scale, Vec3.zero()).getTrues() & 0x7) & 1) != 0;
}

/// Test if any of the components of the scale have a value below min_scale
pub fn isZeroScale(scale: Vec3) bool {
    return Vec3.less(scale.abs(), Vec3.replicate(min_scale)).testAnyXYZTrue();
}

/// Ensure that the scale for each component is at least min_scale
pub fn makeNonZeroScale(scale: Vec3) Vec3 {
    return scale.getSign().mul(Vec3.max(scale.abs(), Vec3.replicate(min_scale)));
}

/// Get the average scale if scale, used to make the scale uniform when a shape doesn't support non-uniform scale
pub fn makeUniformScale(scale: Vec3) Vec3 {
    return Vec3.replicate((scale.getX() + scale.getY() + scale.getZ()) / 3.0);
}

/// Checks in scale can be rotated to child shape
/// @param rotation Rotation of child shape
/// @param scale Scale in local space of parent shape
/// @return True if the scale is valid (no shearing introduced)
pub fn canScaleBeRotated(rotation: Quat, scale: Vec3) bool {
    // scale is a scale in local space of the shape, so the transform for the shape (ignoring translation) is: T = Mat44::sScale(inScale) * mRotation.
    // when we pass the scale to the child it needs to be local to the child, so we want T = mRotation * Mat44::sScale(ChildScale).
    // Solving for ChildScale: ChildScale = mRotation^-1 * Mat44::sScale(inScale) * mRotation = mRotation^T * Mat44::sScale(inScale) * mRotation
    // If any of the off diagonal elements are non-zero, it means the scale / rotation is not compatible.
    const r = Mat44.rotationQuat(rotation);
    const child_scale = r.multiply3x3LeftTransposed(r.postScaled(scale));

    // Get the columns, but zero the diagonal
    const zero = Vec4.zero();
    const c0 = Vec4.select(child_scale.getColumn4(0), zero, UVec4.init(0xffffffff, 0, 0, 0)).abs();
    const c1 = Vec4.select(child_scale.getColumn4(1), zero, UVec4.init(0, 0xffffffff, 0, 0)).abs();
    const c2 = Vec4.select(child_scale.getColumn4(2), zero, UVec4.init(0, 0, 0xffffffff, 0)).abs();

    // Check if all elements are less than epsilon
    const epsilon = Vec4.replicate(1.0e-6);
    return UVec4.bitAnd(UVec4.bitAnd(Vec4.less(c0, epsilon), Vec4.less(c1, epsilon)), Vec4.less(c2, epsilon)).testAllTrue();
}

/// Adjust scale for rotated child shape
/// @param rotation Rotation of child shape
/// @param scale Scale in local space of parent shape
/// @return Rotated scale
pub fn rotateScale(rotation: Quat, scale: Vec3) Vec3 {
    // Get the diagonal of mRotation^T * Mat44::sScale(inScale) * mRotation (see comment at CanScaleBeRotated)
    const r = Mat44.rotationQuat(rotation);
    return r.multiply3x3LeftTransposed(r.postScaled(scale)).getDiagonal3();
}
