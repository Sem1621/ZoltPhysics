//! Port of: Jolt/Physics/Body/MotionType.h
//! Status: complete

/// Motion type of a physics body (EMotionType). The order matters: Jolt compares motion types (ProcessBodyPair).
pub const MotionType = enum(u8) {
    /// Non movable
    static,
    /// Movable using velocities only, does not respond to forces
    kinematic,
    /// Responds to forces as a normal physics object
    dynamic,
};

test "MotionType values" {
    const std = @import("std");
    try std.testing.expectEqual(@as(u8, 0), @intFromEnum(MotionType.static));
    try std.testing.expectEqual(@as(u8, 1), @intFromEnum(MotionType.kinematic));
    try std.testing.expectEqual(@as(u8, 2), @intFromEnum(MotionType.dynamic));
}
