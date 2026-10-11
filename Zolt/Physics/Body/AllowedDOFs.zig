//! Port of: Jolt/Physics/Body/AllowedDOFs.h
//! Status: complete
//!
//! `EAllowedDOFs` is a public bit flag enum with operators: it becomes a `packed struct(u8)` with Jolt's bit order
//! (SimulationArchitecture D1), so streams and the C ABI see Jolt's byte. The single bits are the fields
//! (`EAllowedDOFs::TranslationX` is `.{ .translation_x = true }`), Jolt's named combinations are decls (`none`, `all`,
//! `plane_2d`). The operators `|`, `&`, `^`, `~` are `bitOr`, `bitAnd`, `bitXor`, `bitNot` (rename table: `and` / `or`
//! are keywords); `~` keeps Jolt's `~uint8(x)`, which also sets the two unused high bits (`_padding`).

const std = @import("std");

/// Enum used in BodyCreationSettings and MotionProperties to indicate which degrees of freedom a body has (EAllowedDOFs)
pub const AllowedDOFs = packed struct(u8) {
    /// Body can move in world space X axis
    translation_x: bool = false,
    /// Body can move in world space Y axis
    translation_y: bool = false,
    /// Body can move in world space Z axis
    translation_z: bool = false,
    /// Body can rotate around world space X axis
    rotation_x: bool = false,
    /// Body can rotate around world space Y axis
    rotation_y: bool = false,
    /// Body can rotate around world space Z axis
    rotation_z: bool = false,
    /// The unused bits of Jolt's uint8 (only `bitNot` sets them)
    _padding: u2 = 0,

    /// No degrees of freedom are allowed. Note that this is not valid and will crash. Use a static body instead.
    pub const none: AllowedDOFs = .{};
    /// All degrees of freedom are allowed
    pub const all: AllowedDOFs = .{ .translation_x = true, .translation_y = true, .translation_z = true, .rotation_x = true, .rotation_y = true, .rotation_z = true };
    /// Body can only move in X and Y axis and rotate around Z axis
    pub const plane_2d: AllowedDOFs = .{ .translation_x = true, .translation_y = true, .rotation_z = true };

    comptime {
        std.debug.assert(@as(u8, @bitCast(none)) == 0b000000);
        std.debug.assert(@as(u8, @bitCast(all)) == 0b111111);
        std.debug.assert(@as(u8, @bitCast(AllowedDOFs{ .translation_x = true })) == 0b000001);
        std.debug.assert(@as(u8, @bitCast(AllowedDOFs{ .translation_y = true })) == 0b000010);
        std.debug.assert(@as(u8, @bitCast(AllowedDOFs{ .translation_z = true })) == 0b000100);
        std.debug.assert(@as(u8, @bitCast(AllowedDOFs{ .rotation_x = true })) == 0b001000);
        std.debug.assert(@as(u8, @bitCast(AllowedDOFs{ .rotation_y = true })) == 0b010000);
        std.debug.assert(@as(u8, @bitCast(AllowedDOFs{ .rotation_z = true })) == 0b100000);
        std.debug.assert(@as(u8, @bitCast(plane_2d)) == 0b100011);
    }

    /// The value as Jolt's uint8 (`uint8(inAllowedDOFs)`)
    pub fn toInt(self: AllowedDOFs) u8 {
        return @bitCast(self);
    }

    /// `EAllowedDOFs(inValue)`
    pub fn fromInt(value: u8) AllowedDOFs {
        return @bitCast(value);
    }

    /// Bitwise OR operator for EAllowedDOFs
    pub fn bitOr(lhs: AllowedDOFs, rhs: AllowedDOFs) AllowedDOFs {
        return fromInt(lhs.toInt() | rhs.toInt());
    }

    /// Bitwise AND operator for EAllowedDOFs
    pub fn bitAnd(lhs: AllowedDOFs, rhs: AllowedDOFs) AllowedDOFs {
        return fromInt(lhs.toInt() & rhs.toInt());
    }

    /// Bitwise XOR operator for EAllowedDOFs
    pub fn bitXor(lhs: AllowedDOFs, rhs: AllowedDOFs) AllowedDOFs {
        return fromInt(lhs.toInt() ^ rhs.toInt());
    }

    /// Bitwise NOT operator for EAllowedDOFs
    pub fn bitNot(allowed_dofs: AllowedDOFs) AllowedDOFs {
        return fromInt(~allowed_dofs.toInt());
    }

    /// `==` on the enum (compares the whole byte)
    pub fn eql(lhs: AllowedDOFs, rhs: AllowedDOFs) bool {
        return lhs.toInt() == rhs.toInt();
    }
};

test "AllowedDOFs operators" {
    const tx: AllowedDOFs = .{ .translation_x = true };
    const ty: AllowedDOFs = .{ .translation_y = true };
    const rz: AllowedDOFs = .{ .rotation_z = true };
    try std.testing.expect(tx.bitOr(ty).bitOr(rz).eql(AllowedDOFs.plane_2d));
    try std.testing.expect(AllowedDOFs.plane_2d.bitAnd(ty).eql(ty));
    try std.testing.expect(AllowedDOFs.plane_2d.bitXor(ty).eql(tx.bitOr(rz)));
    try std.testing.expectEqual(@as(u8, 0b11011100), AllowedDOFs.plane_2d.bitNot().toInt());
    try std.testing.expect(AllowedDOFs.all.bitAnd(AllowedDOFs.plane_2d.bitNot()).eql(.{ .translation_z = true, .rotation_x = true, .rotation_y = true }));
    try std.testing.expect(!AllowedDOFs.all.eql(AllowedDOFs.none));
}
