//! Port of: Jolt/Physics/Body/BodyPair.h
//! Status: complete
//!
//! `operator ==` / `operator <` / `GetHash` reinterpret the pair as one uint64 (body A in the low half on little endian
//! machines), like Jolt: `eql`, `lessThan`, `getHash`. `toU64` is that reinterpretation.

const std = @import("std");
const BodyID = @import("BodyID.zig").BodyID;
const HashCombine = @import("../../Core/HashCombine.zig");

/// Structure that holds a body pair
pub const BodyPair = extern struct {
    body_a: BodyID align(8) = .invalid,
    body_b: BodyID = .invalid,

    comptime {
        std.debug.assert(@sizeOf(BodyPair) == @sizeOf(u64)); // Mismatch in class size
        std.debug.assert(@alignOf(BodyPair) == @alignOf(u64));
    }

    /// Constructor
    pub fn init(a: BodyID, b: BodyID) BodyPair {
        return .{ .body_a = a, .body_b = b };
    }

    /// The pair as Jolt's `*reinterpret_cast<const uint64 *>(this)`
    pub fn toU64(self: BodyPair) u64 {
        return std.mem.bytesToValue(u64, std.mem.asBytes(&self));
    }

    /// Equals operator
    pub fn eql(self: BodyPair, rhs: BodyPair) bool {
        return self.toU64() == rhs.toU64();
    }

    /// Smaller than operator, used for consistently ordering body pairs
    pub fn lessThan(self: BodyPair, rhs: BodyPair) bool {
        return self.toU64() < rhs.toU64();
    }

    /// Get the hash value of this object
    pub fn getHash(self: BodyPair) u64 {
        return HashCombine.hash64(self.toU64());
    }
};

test "BodyPair compares and hashes as one uint64" {
    const a = BodyPair.init(.init(1), .init(2));
    const b = BodyPair.init(.init(2), .init(1));
    try std.testing.expect(a.eql(BodyPair.init(.init(1), .init(2))));
    try std.testing.expect(!a.eql(b));
    // Body B is the high half: (1, 2) > (2, 1)
    try std.testing.expect(b.lessThan(a));
    try std.testing.expect(!a.lessThan(b));
    try std.testing.expectEqual(HashCombine.hash64(0x0000000200000001), a.getHash());
    try std.testing.expectEqual(a.getHash(), HashCombine.hash(a));
    const default_pair: BodyPair = .{};
    try std.testing.expectEqual(~@as(u64, 0), default_pair.toU64());
}
