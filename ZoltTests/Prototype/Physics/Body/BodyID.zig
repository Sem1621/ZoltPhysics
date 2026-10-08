//! Port of: Jolt/Physics/Body/BodyID.h (prototype, reduced)
//! Status: partial
//!
//! A value type: `extern struct` (hashed and used as a key, e.g. in the LockFreeHashMap of the contact cache).

const std = @import("std");

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

    /// Default constructor: an invalid body ID
    pub const invalid: BodyID = .{};

    id: u32 = invalid_body_id,

    /// Construct from index and sequence number combined in a single uint32 (use with care!)
    pub fn init(id: u32) BodyID {
        std.debug.assert((id & broad_phase_bit) == 0 or id == invalid_body_id); // Check bit used by broadphase
        return .{ .id = id };
    }

    /// Get index in body array
    pub fn getIndex(self: BodyID) u32 {
        return self.id & max_body_index;
    }

    /// Returns the index and sequence number combined in an uint32
    pub fn getIndexAndSequenceNumber(self: BodyID) u32 {
        return self.id;
    }

    /// Check if the ID is valid
    pub fn isInvalid(self: BodyID) bool {
        return self.id == invalid_body_id;
    }

    /// Equals check (operator ==)
    pub fn eql(self: BodyID, other: BodyID) bool {
        return self.id == other.id;
    }
};
