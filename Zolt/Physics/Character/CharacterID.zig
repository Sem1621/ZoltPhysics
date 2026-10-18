//! Port of: Jolt/Physics/Character/CharacterID.h
//! Status: complete
//!
//! CharacterVehicleArchitecture E4: the counter of the next character ID (Jolt's private static `sNextID`, an
//! `atomic<uint32>`) is the `pub var` atomic `CharacterID.next_id` (the rename rule for static member variables). It is
//! global like Jolt's, because the IDs are part of the contact ordering and of the M8 determinism hash, and
//! PerformanceTest resets it (`setNextCharacterID`) before every run: a counter per PhysicsSystem would give characters
//! other IDs than Jolt.

const std = @import("std");
const HashCombine = @import("../../Core/HashCombine.zig");

/// ID of a character. Used primarily to identify deleted characters and to sort deterministically.
pub const CharacterID = extern struct {
    /// The value for an invalid character ID
    pub const invalid_character_id: u32 = 0xffffffff;

    /// Construct invalid character ID (the default constructor)
    pub const invalid: CharacterID = .{};

    /// Next character ID to be assigned (Jolt's private static `sNextID`, E4)
    pub var next_id: std.atomic.Value(u32) = .init(1);

    /// ID value
    id: u32 = invalid_character_id,

    /// Construct with specific value, make sure you don't use the same value twice!
    pub fn init(id: u32) CharacterID {
        return .{ .id = id };
    }

    /// Get the numeric value of the ID
    pub fn getValue(self: CharacterID) u32 {
        return self.id;
    }

    /// Check if the ID is valid
    pub fn isInvalid(self: CharacterID) bool {
        return self.id == invalid_character_id;
    }

    /// Equals check
    pub fn eql(self: CharacterID, rhs: CharacterID) bool {
        return self.id == rhs.id;
    }

    /// Smaller than operator, can be used for sorting characters
    pub fn lessThan(self: CharacterID, rhs: CharacterID) bool {
        return self.id < rhs.id;
    }

    /// Greater than operator, can be used for sorting characters
    pub fn greaterThan(self: CharacterID, rhs: CharacterID) bool {
        return self.id > rhs.id;
    }

    /// Get the hash for this character ID
    pub fn getHash(self: CharacterID) u64 {
        return HashCombine.hash(self.id);
    }

    /// Generate the next available character ID
    pub fn nextCharacterID() CharacterID {
        while (true) {
            const next = next_id.fetchAdd(1, .monotonic);
            if (next != invalid_character_id)
                return init(next);
        }
    }

    /// Set the next available character ID, can be used after destroying all character to prepare for a second deterministic run
    pub fn setNextCharacterID(opts: struct { next_value: u32 = 1 }) void {
        next_id.store(opts.next_value, .monotonic);
    }

    comptime {
        std.debug.assert(@sizeOf(CharacterID) == 4); // CharacterContactKey hashes its bytes, Jolt's static_assert there
    }
};

test "CharacterID: invalid, value, comparisons, hash" {
    const expect = std.testing.expect;
    const invalid: CharacterID = .{};
    try expect(invalid.isInvalid());
    try expect(invalid.eql(.invalid));
    try std.testing.expectEqual(@as(u32, 0xffffffff), invalid.getValue());

    const a = CharacterID.init(5);
    const b = CharacterID.init(7);
    try expect(!a.isInvalid());
    try std.testing.expectEqual(@as(u32, 5), a.getValue());
    try expect(a.eql(CharacterID.init(5)) and !a.eql(b));
    try expect(a.lessThan(b) and !b.lessThan(a) and !a.lessThan(a));
    try expect(b.greaterThan(a) and !a.greaterThan(b) and !a.greaterThan(a));

    // Hash<uint32> of the value: HashBytes of its 4 bytes
    const value: u32 = 5;
    try std.testing.expectEqual(HashCombine.hashBytes(std.mem.asBytes(&value)), a.getHash());
    try expect(a.getHash() != b.getHash());
}

test "CharacterID: nextCharacterID, setNextCharacterID, skipping the invalid value" {
    const expectEqual = std.testing.expectEqual;
    const saved = CharacterID.next_id.load(.monotonic);
    defer CharacterID.next_id.store(saved, .monotonic);

    // Jolt's default restarts at 1
    CharacterID.setNextCharacterID(.{});
    try expectEqual(@as(u32, 1), CharacterID.nextCharacterID().getValue());
    try expectEqual(@as(u32, 2), CharacterID.nextCharacterID().getValue());

    CharacterID.setNextCharacterID(.{ .next_value = 100 });
    try expectEqual(@as(u32, 100), CharacterID.nextCharacterID().getValue());

    // The invalid value is never handed out, the counter wraps around to 0 (an atomic fetch_add)
    CharacterID.setNextCharacterID(.{ .next_value = 0xfffffffe });
    try expectEqual(@as(u32, 0xfffffffe), CharacterID.nextCharacterID().getValue());
    try expectEqual(@as(u32, 0), CharacterID.nextCharacterID().getValue());
    try expectEqual(@as(u32, 1), CharacterID.nextCharacterID().getValue());
}

test "CharacterID: concurrent nextCharacterID calls give unique IDs" {
    const saved = CharacterID.next_id.load(.monotonic);
    defer CharacterID.next_id.store(saved, .monotonic);
    CharacterID.setNextCharacterID(.{ .next_value = 1000 });

    const num_threads = 4;
    const per_thread = 250;
    var ids: [num_threads][per_thread]u32 = undefined;
    const Worker = struct {
        fn run(out: *[per_thread]u32) void {
            for (out) |*id| id.* = CharacterID.nextCharacterID().getValue();
        }
    };
    var threads: [num_threads]std.Thread = undefined;
    for (&threads, &ids) |*t, *out| t.* = try std.Thread.spawn(.{}, Worker.run, .{out});
    for (threads) |t| t.join();

    // Every ID in [1000, 2000) is handed out exactly once
    var seen: [num_threads * per_thread]bool = @splat(false);
    for (ids) |list| {
        for (list) |id| {
            try std.testing.expect(id >= 1000 and id < 1000 + num_threads * per_thread);
            try std.testing.expect(!seen[id - 1000]);
            seen[id - 1000] = true;
        }
    }
    try std.testing.expectEqual(@as(u32, 1000 + num_threads * per_thread), CharacterID.next_id.load(.monotonic));
}
