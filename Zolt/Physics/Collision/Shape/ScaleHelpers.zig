//! Port of: Jolt/Physics/Collision/Shape/ScaleHelpers.h
//! Status: complete
//!
//! Helper functions to get properties of a scaling vector.
//!
//! `namespace ScaleHelpers` becomes this file, used as a namespace of free functions (D12):
//! `ScaleHelpers.isUniformScale(scale)`.

const std = @import("std");
const math = @import("../../../Math/Math.zig");
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const UVec4 = @import("../../../Math/UVec4.zig").UVec4;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../../Math/Vec4.zig").Vec4;
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

/// Test if a scale is uniform in XZ
pub fn isUniformScaleXZ(scale: Vec3) bool {
    return scale.swizzle(.z, .y, .x).isClose(scale, .{ .max_dist_sq = scale_tolerance_sq });
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

/// Average the scale in XZ, used to make the scale uniform when a shape doesn't support non-uniform scale in the XZ plane
pub fn makeUniformScaleXZ(scale: Vec3) Vec3 {
    return scale.add(scale.swizzle(.z, .y, .x)).mulScalar(0.5);
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

test "ScaleHelpers" {
    const expect = std.testing.expect;

    try expect(isNotScaled(Vec3.one()));
    try expect(isNotScaled(Vec3.init(1.00001, 1, 1)));
    try expect(!isNotScaled(Vec3.init(1.001, 1, 1)));

    try expect(isUniformScale(Vec3.replicate(-2)));
    try expect(!isUniformScale(Vec3.init(2, 2, 3)));
    try expect(isUniformScaleXZ(Vec3.init(2, 5, 2)));
    try expect(!isUniformScaleXZ(Vec3.init(2, 2, 3)));

    try std.testing.expectEqual(@as(f32, 0.02), scaleConvexRadius(0.01, Vec3.init(-2, 3, 4)));
    try std.testing.expectEqual(PhysicsSettings.default_convex_radius, scaleConvexRadius(1.0, Vec3.one()));

    try expect(!isInsideOut(Vec3.one()));
    try expect(isInsideOut(Vec3.init(-1, 1, 1)));
    try expect(!isInsideOut(Vec3.init(-1, -1, 1)));
    try expect(isInsideOut(Vec3.init(-1, -1, -1)));

    try expect(isZeroScale(Vec3.init(1, 0, 1)));
    try expect(isZeroScale(Vec3.init(1, 1, -1.0e-7)));
    try expect(!isZeroScale(Vec3.init(1, -1, 1)));
    try expect(makeNonZeroScale(Vec3.init(0, -1.0e-7, 2)).eql(Vec3.init(min_scale, -min_scale, 2)));

    try expect(makeUniformScale(Vec3.init(1, 2, 3)).eql(Vec3.replicate(2)));
    try expect(makeUniformScaleXZ(Vec3.init(1, 7, 3)).eql(Vec3.init(2, 7, 2)));

    // A rotation around Y keeps a scale that is uniform in XZ valid, but not a non uniform one
    const rot_y = Quat.rotation(Vec3.axisY(), 0.25 * math.pi);
    try expect(canScaleBeRotated(rot_y, Vec3.init(2, 3, 2)));
    try expect(!canScaleBeRotated(rot_y, Vec3.init(2, 3, 4)));
    try expect(canScaleBeRotated(Quat.identity(), Vec3.init(2, 3, 4)));
    try expect(rotateScale(Quat.identity(), Vec3.init(2, 3, 4)).isClose(Vec3.init(2, 3, 4), .{}));
    try expect(rotateScale(Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), Vec3.init(2, 3, 4)).isClose(Vec3.init(3, 2, 4), .{ .max_dist_sq = 1.0e-10 }));
}
