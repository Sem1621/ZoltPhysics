//! Port of: Jolt/Core/UnorderedSet.h, Jolt/Core/UnorderedSetFwd.h
//! Status: complete
//!
//! `UnorderedSet<Key, Hash, KeyEqual>` becomes `UnorderedSet(Key, .{})`, the options struct holds the
//! hash and equality functions (see `HashTableOptions` in HashTable.zig). The C++ class derives from
//! HashTable without adding anything, so the Zig type is the HashTable itself.

const std = @import("std");
const HashTableFile = @import("HashTable.zig");
const HashTable = HashTableFile.HashTable;
const HashTableOptions = HashTableFile.HashTableOptions;

/// Internal helper class to provide context for UnorderedSet
pub fn UnorderedSetDetail(comptime Key: type) type {
    return struct {
        /// The key is the key, just return it
        pub fn getKey(key: *const Key) Key {
            return key.*;
        }
    };
}

/// Hash Set class
/// @tparam Key Key type
/// `options.hash` Hash function (note should be 64-bits)
/// `options.key_equal` Equality comparison function
pub fn UnorderedSet(comptime Key: type, comptime options: HashTableOptions(Key)) type {
    return HashTable(Key, Key, UnorderedSetDetail(Key), options);
}

test "UnorderedSet with custom hash and equality" {
    const allocator = std.testing.allocator;

    // Case insensitive set of strings
    const Funcs = struct {
        fn hash(key: []const u8) u64 {
            var h: u64 = 0;
            for (key) |c| h = h *% 31 +% std.ascii.toLower(c);
            return h;
        }
        fn equal(a: []const u8, b: []const u8) bool {
            return std.ascii.eqlIgnoreCase(a, b);
        }
    };
    var set: UnorderedSet([]const u8, .{ .hash = Funcs.hash, .key_equal = Funcs.equal }) = .empty;
    defer set.deinit(allocator);
    try std.testing.expect((try set.insert(allocator, "Hello")).inserted);
    try std.testing.expect(!(try set.insert(allocator, "HELLO")).inserted);
    try std.testing.expectEqualStrings("Hello", set.find("hello").?.*);
    try std.testing.expect(set.find("world") == null);

    // Default hash and equality on strings compare the contents
    var strings: UnorderedSet([]const u8, .{}) = .empty;
    defer strings.deinit(allocator);
    const buffer = "abcabc";
    try std.testing.expect((try strings.insert(allocator, buffer[0..3])).inserted);
    try std.testing.expect(!(try strings.insert(allocator, buffer[3..6])).inserted);
    try std.testing.expectEqual(1, strings.count());
}
