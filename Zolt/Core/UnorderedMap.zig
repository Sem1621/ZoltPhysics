//! Port of: Jolt/Core/UnorderedMap.h, Jolt/Core/UnorderedMapFwd.h
//! Status: complete
//!
//! `UnorderedMap<Key, Value, Hash, KeyEqual>` becomes `UnorderedMap(Key, Value, .{})`, the options
//! struct holds the hash and equality functions (see `HashTableOptions` in HashTable.zig).
//! The C++ class derives from HashTable; the Zig struct embeds it as `base` and forwards its methods.
//! `std::pair<Key, Value>` becomes `KeyValue{ .key, .value }` (`first` / `second` in C++).

const std = @import("std");
const Allocator = std.mem.Allocator;
const HashTableFile = @import("HashTable.zig");
const HashTable = HashTableFile.HashTable;
const HashTableOptions = HashTableFile.HashTableOptions;

/// Internal helper class to provide context for UnorderedMap
pub fn UnorderedMapDetail(comptime Key: type, comptime Value: type) type {
    return struct {
        /// The key value pair stored in the map (std::pair<Key, Value>)
        pub const KeyValue = struct {
            /// Key (first)
            key: Key,
            /// Value (second)
            value: Value,
        };

        /// Get key from key value pair
        pub fn getKey(key_value: *const KeyValue) Key {
            return key_value.key;
        }
    };
}

/// Hash Map class
/// @tparam Key Key type
/// @tparam Value Value type
/// `options.hash` Hash function (note should be 64-bits)
/// `options.key_equal` Equality comparison function
pub fn UnorderedMap(comptime Key: type, comptime Value: type, comptime options: HashTableOptions(Key)) type {
    return struct {
        const Self = @This();
        const Detail = UnorderedMapDetail(Key, Value);

        /// The hash table this map is built on (the C++ base class)
        pub const Base = HashTable(Key, Detail.KeyValue, Detail, options);

        /// Properties
        pub const size_type = Base.size_type;
        pub const Iterator = Base.Iterator;
        pub const ConstIterator = Base.ConstIterator;
        pub const KeyValue = Detail.KeyValue;
        pub const value_type = KeyValue;
        pub const InsertResult = Base.InsertResult;

        base: Base = .empty,

        /// Empty map (default constructor)
        pub const empty: Self = .{};

        /// Returns the value for `key`, inserting `default_value` first when the key is not in the map (operator []).
        /// C++ inserts `Value()`, pass the Zig equivalent (e.g. `0`, `.empty`, `.{}`).
        pub fn getOrPutValue(self: *Self, allocator: Allocator, key: Key, default_value: Value) Allocator.Error!*Value {
            const result = try self.base.insertKey(allocator, key, false);
            const key_value = self.base.getElement(result.index);
            if (result.inserted)
                key_value.* = .{ .key = key, .value = default_value };
            return &key_value.value;
        }

        /// Insert `value` under `key` if the key is not in the map yet (try_emplace).
        /// Unlike C++ the value is always constructed (by the caller), when it owns memory and the key
        /// already exists (`inserted == false`) the caller must deinit it.
        pub fn tryEmplace(self: *Self, allocator: Allocator, key: Key, value: Value) Allocator.Error!InsertResult {
            const result = try self.base.insertKey(allocator, key, false);
            const key_value = self.base.getElement(result.index);
            if (result.inserted)
                key_value.* = .{ .key = key, .value = value };
            return .{ .ptr = key_value, .inserted = result.inserted };
        }

        /// Const version of find, returns the key value pair or null (end()) if not found
        pub fn find(self: *const Self, key: Key) ?*const KeyValue {
            return self.base.find(key);
        }

        /// Non-const version of find, returns the key value pair or null (end()) if not found
        pub fn findPtr(self: *Self, key: Key) ?*KeyValue {
            const key_value = self.base.find(key) orelse return null;
            return @constCast(key_value);
        }

        // Methods of the HashTable base class

        /// Copy constructor
        pub fn clone(self: *const Self, allocator: Allocator) Allocator.Error!Self {
            return .{ .base = try self.base.clone(allocator) };
        }

        /// Move constructor: returns the contents of this map and leaves it empty
        pub fn move(self: *Self) Self {
            return .{ .base = self.base.move() };
        }

        /// Assignment operator
        pub fn assign(self: *Self, allocator: Allocator, other: *const Self) Allocator.Error!void {
            try self.base.assign(allocator, &other.base);
        }

        /// Move assignment operator
        pub fn assignMove(self: *Self, allocator: Allocator, other: *Self) void {
            self.base.assignMove(allocator, &other.base);
        }

        /// Destructor. Zig has no destructors: deinit values that own memory first.
        pub fn deinit(self: *Self, allocator: Allocator) void {
            self.base.deinit(allocator);
        }

        /// Reserve memory for a certain number of elements (reserve)
        pub fn ensureTotalCapacity(self: *Self, allocator: Allocator, max_size: u32) Allocator.Error!void {
            try self.base.ensureTotalCapacity(allocator, max_size);
        }

        /// Destroy the entire hash table (clear)
        pub fn clearAndFree(self: *Self, allocator: Allocator) void {
            self.base.clearAndFree(allocator);
        }

        /// Destroy the entire hash table but keeps the memory allocated (ClearAndKeepMemory)
        pub fn clearRetainingCapacity(self: *Self) void {
            self.base.clearRetainingCapacity();
        }

        /// Iterate over all key value pairs (begin() / end())
        pub fn iterator(self: *Self) Iterator {
            return self.base.iterator();
        }

        /// Iterate over all key value pairs, const version (begin() const / cbegin() / end() const / cend())
        pub fn constIterator(self: *const Self) ConstIterator {
            return self.base.constIterator();
        }

        /// Number of buckets in the table
        pub fn bucketCount(self: *const Self) u32 {
            return self.base.bucketCount();
        }

        /// Max number of buckets that the table can have
        pub fn maxBucketCount(self: *const Self) u32 {
            return self.base.maxBucketCount();
        }

        /// Check if there are no elements in the table (empty)
        pub fn isEmpty(self: *const Self) bool {
            return self.base.isEmpty();
        }

        /// Number of elements in the table (size)
        pub fn count(self: *const Self) u32 {
            return self.base.count();
        }

        /// Max number of elements that the table can hold
        pub fn maxSize(self: *const Self) u32 {
            return self.base.maxSize();
        }

        /// Get the max load factor for this table (max number of elements / number of buckets)
        pub fn maxLoadFactor(self: *const Self) f32 {
            return self.base.maxLoadFactor();
        }

        /// Insert a new element, returns the element and if the element was inserted
        pub fn insert(self: *Self, allocator: Allocator, key_value: KeyValue) Allocator.Error!InsertResult {
            return self.base.insert(allocator, key_value);
        }

        /// Index of the bucket that holds `key_value` (the mIndex of a C++ iterator)
        pub fn indexOf(self: *const Self, key_value: *const KeyValue) u32 {
            return self.base.indexOf(key_value);
        }

        /// Erase an element by iterator (erase(const_iterator)), `key_value` must point into this map
        pub fn eraseByPtr(self: *Self, key_value: *const KeyValue) void {
            self.base.eraseByPtr(key_value);
        }

        /// Erase an element by key, returns the number of elements erased (0 or 1)
        pub fn erase(self: *Self, key: Key) u32 {
            return self.base.erase(key);
        }

        /// Swap the contents of two hash tables
        pub fn swap(self: *Self, other: *Self) void {
            self.base.swap(&other.base);
        }

        /// In place re-hashing of all elements in the table. Removes all deleted elements
        /// The std version takes a bucket count, but we just re-hash to the same size.
        pub fn rehash(self: *Self, bucket_count: u32) void {
            self.base.rehash(bucket_count);
        }
    };
}

test "UnorderedMap API" {
    const allocator = std.testing.allocator;
    const Map = UnorderedMap(u64, f32, .{});

    var map: Map = .empty;
    defer map.deinit(allocator);
    try map.ensureTotalCapacity(allocator, 100);
    try std.testing.expectEqual(128, map.bucketCount());
    try std.testing.expectEqual(0x80000000, map.maxBucketCount());
    try std.testing.expectEqual(0x70000000, map.maxSize());
    try std.testing.expectEqual(0.875, map.maxLoadFactor());

    for (0..50) |i|
        try std.testing.expect((try map.tryEmplace(allocator, i, @floatFromInt(i))).inserted);
    try std.testing.expectEqual(50, map.count());
    try std.testing.expect(!(try map.insert(allocator, .{ .key = 1, .value = 5.0 })).inserted);
    try std.testing.expectEqual(1.0, map.find(1).?.value);

    // Modify through findPtr / getOrPutValue
    map.findPtr(2).?.value = 20.0;
    try std.testing.expectEqual(20.0, map.find(2).?.value);
    try std.testing.expect(map.findPtr(1000) == null);
    try std.testing.expectEqual(3.0, (try map.getOrPutValue(allocator, 3, 0.0)).*);
    try std.testing.expectEqual(0.0, (try map.getOrPutValue(allocator, 1000, 0.0)).*);
    try std.testing.expectEqual(51, map.count());

    // Iterate and modify values
    var it = map.iterator();
    while (it.next()) |key_value|
        key_value.value += 1.0;
    try std.testing.expectEqual(21.0, map.find(2).?.value);
    try std.testing.expectEqual(map.indexOf(map.find(2).?), map.indexOf(map.findPtr(2).?));

    // Erase
    map.eraseByPtr(map.find(2).?);
    try std.testing.expectEqual(1, map.erase(3));
    try std.testing.expectEqual(0, map.erase(3));
    map.rehash(0);
    try std.testing.expectEqual(49, map.count());

    // Copy / move / swap
    var copy = try map.clone(allocator);
    defer copy.deinit(allocator);
    var assigned: Map = .empty;
    defer assigned.deinit(allocator);
    try assigned.assign(allocator, &copy);
    var it1 = copy.constIterator();
    var it2 = assigned.constIterator();
    while (it1.next()) |kv| {
        const kv2 = it2.next().?;
        try std.testing.expectEqual(kv.key, kv2.key);
        try std.testing.expectEqual(kv.value, kv2.value);
    }
    try std.testing.expect(it2.next() == null);

    var moved = map.move();
    defer moved.deinit(allocator);
    try std.testing.expect(map.isEmpty());
    assigned.assignMove(allocator, &moved);
    try std.testing.expect(moved.isEmpty());
    assigned.swap(&moved);
    try std.testing.expectEqual(49, moved.count());

    moved.clearRetainingCapacity();
    try std.testing.expect(moved.isEmpty());
    try std.testing.expectEqual(128, moved.bucketCount());
    moved.clearAndFree(allocator);
    try std.testing.expectEqual(0, moved.bucketCount());
}

test "UnorderedMap with values that own memory" {
    const allocator = std.testing.allocator;
    var map: UnorderedMap(u32, std.ArrayList(u32), .{}) = .empty;
    defer {
        var it = map.iterator();
        while (it.next()) |key_value|
            key_value.value.deinit(allocator);
        map.deinit(allocator);
    }

    for (0..100) |i| {
        const list = try map.getOrPutValue(allocator, @intCast(i % 10), .empty);
        try list.append(allocator, @intCast(i));
    }
    try std.testing.expectEqual(10, map.count());
    try std.testing.expectEqual(10, map.find(3).?.value.items.len);
}
