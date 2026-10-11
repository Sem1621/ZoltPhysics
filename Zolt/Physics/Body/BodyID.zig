//! Port of: Jolt/Physics/Body/BodyID.h
//! Status: complete
//!
//! A value type: `extern struct` (hashed, streamed and used as a key, e.g. in the LockFreeHashMap of the contact
//! cache). `BodyID()` is the field default (`.invalid`), `BodyID(inID)` is `init(id)` and
//! `BodyID(inID, inSequenceNumber)` is `fromIndexAndSequenceNumber`. The comparison operators become `eql`,
//! `lessThan` and `greaterThan` (`!=` is `!a.eql(b)`). JPH_MAKE_HASHABLE becomes `getHash`.

const std = @import("std");
const HashCombine = @import("../../Core/HashCombine.zig");

/// ID of a body. This is a way of reasoning about bodies in a multithreaded simulation while avoiding race conditions.
pub const BodyID = extern struct {
    /// The value for an invalid body ID
    pub const invalid_body_id: u32 = 0xffffffff;
    /// This bit is used by the broadphase
    pub const broad_phase_bit: u32 = 0x80000000;
    /// Maximum value for body index (also the maximum amount of bodies supported - 1)
    pub const max_body_index: u32 = 0x7fffff;
    /// Maximum value for the sequence number
    pub const max_sequence_number: u8 = 0xff;
    /// Number of bits to shift to get the sequence number
    pub const sequence_number_shift: u32 = 23;

    /// Construct invalid body ID (the default constructor)
    pub const invalid: BodyID = .{};

    id: u32 = invalid_body_id,

    /// Construct from index and sequence number combined in a single uint32 (use with care!)
    pub fn init(id: u32) BodyID {
        std.debug.assert((id & broad_phase_bit) == 0 or id == invalid_body_id); // Check bit used by broadphase
        return .{ .id = id };
    }

    /// Construct from index and sequence number
    pub fn fromIndexAndSequenceNumber(id: u32, sequence_number: u8) BodyID {
        std.debug.assert(id <= max_body_index); // Should not overlap with broadphase bit or sequence number
        return .{ .id = (@as(u32, sequence_number) << sequence_number_shift) | id };
    }

    /// Get index in body array
    pub fn getIndex(self: BodyID) u32 {
        return self.id & max_body_index;
    }

    /// Get sequence number of body.
    /// The sequence number can be used to check if a body ID with the same body index has been reused by another body.
    /// It is mainly used in multi threaded situations where a body is removed and its body index is immediately reused by a body created from another thread.
    /// Functions querying the broadphase can (after acquiring a body lock) detect that the body has been removed (we assume that this won't happen more than 128 times in a row).
    pub fn getSequenceNumber(self: BodyID) u8 {
        return @truncate(self.id >> sequence_number_shift);
    }

    /// Returns the index and sequence number combined in an uint32
    pub fn getIndexAndSequenceNumber(self: BodyID) u32 {
        return self.id;
    }

    /// Check if the ID is valid
    pub fn isInvalid(self: BodyID) bool {
        return self.id == invalid_body_id;
    }

    /// Equals check (operator ==, `!=` is `!a.eql(b)`)
    pub fn eql(self: BodyID, other: BodyID) bool {
        return self.id == other.id;
    }

    /// Smaller than operator, can be used for sorting bodies (operator <)
    pub fn lessThan(self: BodyID, other: BodyID) bool {
        return self.id < other.id;
    }

    /// Greater than operator, can be used for sorting bodies (operator >)
    pub fn greaterThan(self: BodyID, other: BodyID) bool {
        return self.id > other.id;
    }

    /// Create a std::hash/JPH::Hash for BodyID (JPH_MAKE_HASHABLE(JPH::BodyID, t.GetIndexAndSequenceNumber()))
    pub fn getHash(self: BodyID) u64 {
        return HashCombine.hashCombineArgs(.{self.getIndexAndSequenceNumber()});
    }

    comptime {
        std.debug.assert(@sizeOf(BodyID) == 4);
    }
};

test "BodyID" {
    const invalid: BodyID = .{};
    try std.testing.expect(invalid.isInvalid());
    try std.testing.expect(invalid.eql(.invalid));
    try std.testing.expect(BodyID.init(BodyID.invalid_body_id).isInvalid());

    const id = BodyID.fromIndexAndSequenceNumber(0x123456, 0xab);
    try std.testing.expectEqual(@as(u32, 0x123456), id.getIndex());
    try std.testing.expectEqual(@as(u8, 0xab), id.getSequenceNumber());
    try std.testing.expectEqual(@as(u32, (0xab << 23) | 0x123456), id.getIndexAndSequenceNumber());
    try std.testing.expect(!id.isInvalid());
    try std.testing.expect(id.eql(BodyID.init(id.getIndexAndSequenceNumber())));
    try std.testing.expect(!id.eql(.invalid));

    // The maximum sequence number fills bits 23..30 and leaves the broadphase bit (bit 31) clear
    const max_sequence = BodyID.fromIndexAndSequenceNumber(BodyID.max_body_index, BodyID.max_sequence_number);
    try std.testing.expectEqual(BodyID.max_sequence_number, max_sequence.getSequenceNumber());
    try std.testing.expectEqual(BodyID.max_body_index, max_sequence.getIndex());
    try std.testing.expectEqual(@as(u32, 0), max_sequence.getIndexAndSequenceNumber() & BodyID.broad_phase_bit);
    try std.testing.expect(!max_sequence.isInvalid());

    try std.testing.expect(BodyID.init(1).lessThan(.init(2)));
    try std.testing.expect(!BodyID.init(2).lessThan(.init(2)));
    try std.testing.expect(BodyID.init(3).greaterThan(.init(2)));
    try std.testing.expect(!BodyID.init(2).greaterThan(.init(2)));

    // Hash<BodyID> hashes the combined value like Hash<uint32>
    try std.testing.expectEqual(HashCombine.hash(@as(u32, 42)), BodyID.init(42).getHash());
    try std.testing.expectEqual(BodyID.init(42).getHash(), HashCombine.hash(BodyID.init(42)));
}
