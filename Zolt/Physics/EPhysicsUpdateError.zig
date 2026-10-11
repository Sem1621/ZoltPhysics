//! Port of: Jolt/Physics/EPhysicsUpdateError.h
//! Status: complete
//!
//! `EPhysicsUpdateError` is a public bit flag enum, so it is a `packed struct(u32)` with Jolt's bit order
//! (SimulationArchitecture D1, porting guide section 5): `PhysicsUpdateError.none` is `EPhysicsUpdateError::None`, the
//! operators `|` / `|=` / `&` are `bitOr` / `bitAnd`. These are Jolt's soft errors: the simulation continued but some
//! contacts were ignored. Allocation failure is the Zig error of `PhysicsSystem.update` (D5).

const std = @import("std");

/// Enum used by PhysicsSystem to report error conditions during the PhysicsSystem::Update call. This is a bit field, multiple errors can trigger in the same update.
pub const PhysicsUpdateError = packed struct(u32) {
    /// The manifold cache is full, this means that the total number of contacts between bodies is too high. Some contacts were ignored. Increase inMaxContactConstraints in PhysicsSystem::Init.
    manifold_cache_full: bool = false,
    /// The body pair cache is full, this means that too many bodies contacted. Some contacts were ignored. Increase inMaxBodyPairs in PhysicsSystem::Init.
    body_pair_cache_full: bool = false,
    /// The contact constraints buffer is full. Some contacts were ignored. Increase inMaxContactConstraints in PhysicsSystem::Init.
    contact_constraints_full: bool = false,
    /// The unused bits of Jolt's uint32
    _padding: u29 = 0,

    /// No errors
    pub const none: PhysicsUpdateError = .{};

    comptime {
        std.debug.assert(@as(u32, @bitCast(none)) == 0);
        std.debug.assert(@as(u32, @bitCast(PhysicsUpdateError{ .manifold_cache_full = true })) == 1 << 0);
        std.debug.assert(@as(u32, @bitCast(PhysicsUpdateError{ .body_pair_cache_full = true })) == 1 << 1);
        std.debug.assert(@as(u32, @bitCast(PhysicsUpdateError{ .contact_constraints_full = true })) == 1 << 2);
    }

    /// The value as Jolt's uint32
    pub fn toInt(self: PhysicsUpdateError) u32 {
        return @bitCast(self);
    }

    /// `static_cast<EPhysicsUpdateError>(inValue)`
    pub fn fromInt(value: u32) PhysicsUpdateError {
        return @bitCast(value);
    }

    /// OR operator for EPhysicsUpdateError (also `|=`)
    pub fn bitOr(a: PhysicsUpdateError, b: PhysicsUpdateError) PhysicsUpdateError {
        return fromInt(a.toInt() | b.toInt());
    }

    /// AND operator for EPhysicsUpdateError
    pub fn bitAnd(a: PhysicsUpdateError, b: PhysicsUpdateError) PhysicsUpdateError {
        return fromInt(a.toInt() & b.toInt());
    }

    /// `==` on the enum (compares the whole uint32)
    pub fn eql(a: PhysicsUpdateError, b: PhysicsUpdateError) bool {
        return a.toInt() == b.toInt();
    }

    /// `errors == EPhysicsUpdateError::None`
    pub fn isNone(self: PhysicsUpdateError) bool {
        return self.toInt() == 0;
    }
};

test "PhysicsUpdateError bits and operators" {
    const expect = std.testing.expect;
    const a: PhysicsUpdateError = .{ .manifold_cache_full = true };
    const b: PhysicsUpdateError = .{ .contact_constraints_full = true };
    try expect(PhysicsUpdateError.none.isNone());
    try expect(!a.isNone());
    try std.testing.expectEqual(@as(u32, 5), a.bitOr(b).toInt());
    try expect(a.bitOr(b).bitAnd(b).eql(b));
    try expect(a.bitAnd(b).isNone());
    try expect(PhysicsUpdateError.fromInt(2).body_pair_cache_full);
}
