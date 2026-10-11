//! Port of: Jolt/Physics/Body/BodyType.h
//! Status: complete

/// Type of body (EBodyType)
pub const BodyType = enum(u8) {
    /// Rigid body consisting of a rigid shape
    rigid_body,
    /// Soft body consisting of a deformable shape
    soft_body,
};

/// How many types of bodies there are (cBodyTypeCount)
pub const body_type_count: u32 = 2;

test "BodyType values" {
    const std = @import("std");
    try std.testing.expectEqual(@as(u8, 0), @intFromEnum(BodyType.rigid_body));
    try std.testing.expectEqual(@as(u8, 1), @intFromEnum(BodyType.soft_body));
    try std.testing.expectEqual(@as(usize, body_type_count), @typeInfo(BodyType).@"enum".fields.len);
}
