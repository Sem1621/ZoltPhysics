//! Port of: Jolt/Physics/Constraints/PathConstraint.h, Jolt/Physics/Constraints/PathConstraint.cpp
//! Status: stub
//!
//! Created by the Phase 6 foundation with the final type names, so that RegisterTypes.zig (`constraint_settings_types`)
//! and the parity harnesses (ZoltParity/Physics/ConstraintsParity.zig, SimulationParity.zig) can name the classes
//! before they are ported. The porter of this constraint replaces the stub following the porter template in
//! FixedConstraint.zig (and removes `is_stub`, then fills the harness slots). `PathRotationConstraintType` is final
//! (Zolt/zolt.zig re-exports it).

/// How to constrain the rotation of the body to a PathConstraint
/// (EPathRotationConstraintType: Jolt's enum class has the underlying type int, which the binary state of the settings
/// writes; like MotorState and ConstraintSpace every int value is valid, a restored value outside the named ones is
/// not illegal behavior: a switch on it needs a `_` prong that does what Jolt's switch does when no case matches)
pub const PathRotationConstraintType = enum(i32) {
    /// Do not constrain the rotation of the body at all
    free,
    /// Only allow rotation around the tangent vector (following the path)
    constrain_around_tangent,
    /// Only allow rotation around the normal vector (perpendicular to the path)
    constrain_around_normal,
    /// Only allow rotation around the binormal vector (perpendicular to the path)
    constrain_around_binormal,
    /// Fully constrain the rotation of body 2 to the path (following the tangent and normal of the path)
    constrain_to_path,
    /// Fully constrain the rotation of the body 2 to the rotation of body 1
    fully_constrained,
    /// Any other int value
    _,
};

/// Path constraint settings, used to constrain the degrees of freedom between two bodies to a path
pub const PathConstraintSettings = struct {
    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "PathConstraintSettings";

    /// Zolt marker of a stub class: ConstraintSettings.restoreFromBinaryState does not create it (Jolt's "Failed to
    /// create instance of type") and the parity harnesses skip it, until it is ported (remove with the stub)
    pub const is_stub = true;
};

/// Path constraint, used to constrain the degrees of freedom between two bodies to a path
pub const PathConstraint = struct {};

test "PathRotationConstraintType has Jolt's values" {
    const std = @import("std");
    try std.testing.expectEqual(@as(i32, 0), @intFromEnum(PathRotationConstraintType.free));
    try std.testing.expectEqual(@as(i32, 1), @intFromEnum(PathRotationConstraintType.constrain_around_tangent));
    try std.testing.expectEqual(@as(i32, 2), @intFromEnum(PathRotationConstraintType.constrain_around_normal));
    try std.testing.expectEqual(@as(i32, 3), @intFromEnum(PathRotationConstraintType.constrain_around_binormal));
    try std.testing.expectEqual(@as(i32, 4), @intFromEnum(PathRotationConstraintType.constrain_to_path));
    try std.testing.expectEqual(@as(i32, 5), @intFromEnum(PathRotationConstraintType.fully_constrained));
    try std.testing.expectEqual(@as(usize, 4), @sizeOf(PathRotationConstraintType)); // Written as an int by the binary state
}
