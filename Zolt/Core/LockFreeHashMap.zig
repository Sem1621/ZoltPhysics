//! Port of: Jolt/Core/LockFreeHashMap.h, Jolt/Core/LockFreeHashMap.inl
//! Status: complete
//!
//! Differences with the C++ version:
//! - Unmanaged containers: `LFHMAllocator.init` / `LockFreeHashMap.init` take the `std.mem.Allocator` and their `deinit`
//!   takes it again. `LockFreeHashMap.init(allocator, lfhm_allocator, max_buckets)` combines the C++ constructor (which
//!   stores a reference to the LFHMAllocator) and `Init(inMaxBuckets)`. The map keeps a pointer to the LFHMAllocator
//!   (like the C++ reference), so the LFHMAllocator must outlive the map and must not move.
//! - `create` takes the initial value instead of forwarding constructor parameters.
//! - `KeyValue` is an `extern struct` with the C++ layout (key, next offset, value), so that the extra bytes requested
//!   by `create` directly follow the value and handles (offsets) are the same as in Jolt. Key and Value must therefore
//!   be types with a defined layout (integers, floats, enums with an explicit tag type, extern structs, arrays of them).
//!   Keys are compared with `eql` if the type declares it, otherwise with `==`.
//! - The iterator keeps Jolt's begin() / end() / operator++ / operator* (`begin`, `end`, `advance`, `get`) and adds a
//!   Zig style `next()`: `var it = map.begin(); while (it.next()) |kv| { ... }`.
//! - Implicit atomic loads of the C++ code (`uint32 offset = *bucket`) are sequentially consistent, like in C++.

const std = @import("std");
const builtin = @import("builtin");
const Core = @import("Core.zig");
const math = @import("../Math/Math.zig");
const UVec4 = @import("../Math/UVec4.zig").UVec4;

const log = std.log.scoped(.zolt);

/// Allocator for a lock free hash map
pub const LFHMAllocator = struct {
    /// This contains a contiguous list of objects (possibly of varying size)
    object_store: ?[*]align(16) u8 = null,

    /// The size of object_store in bytes
    object_store_size_bytes: u32 = 0,

    /// Next offset to write to in object_store
    write_offset: std.atomic.Value(u32) = .init(0),

    /// Allocator without storage (default constructor), deinit does nothing
    pub const empty: LFHMAllocator = .{};

    /// Initialize the allocator (Init)
    /// `object_store_size_bytes`: Number of bytes to reserve for all key value pairs
    pub fn init(allocator: std.mem.Allocator, object_store_size_bytes: u32) error{OutOfMemory}!LFHMAllocator {
        const object_store = try allocator.alignedAlloc(u8, .fromByteUnits(16), object_store_size_bytes);
        return .{
            .object_store = object_store.ptr,
            .object_store_size_bytes = object_store_size_bytes,
        };
    }

    /// Destructor
    pub fn deinit(self: *LFHMAllocator, allocator: std.mem.Allocator) void {
        if (self.object_store) |object_store|
            allocator.free(object_store[0..self.object_store_size_bytes]);
        self.* = .empty;
    }

    /// Clear all allocations
    pub fn clear(self: *LFHMAllocator) void {
        self.write_offset.store(0, .seq_cst);
    }

    /// Allocate a new block of data
    /// `block_size`: Size of block to allocate (will potentially return a smaller block if memory is full).
    /// `begin`: Should be the start of the first free byte in current memory block on input, will contain the start of the first free byte in allocated block on return.
    /// `end`: Should be the byte beyond the current memory block on input, will contain the byte beyond the allocated block on return.
    pub fn allocate(self: *LFHMAllocator, block_size: u32, begin: *u32, end: *u32) void {
        // If we're already beyond the end of our buffer then don't do an atomic add.
        // It's possible that many keys are inserted after the allocator is full, making it possible
        // for write_offset (u32) to wrap around to zero. When this happens, there will be a memory corruption.
        // This way, we will be able to progress the write offset beyond the size of the buffer
        // worst case by max <CPU count> * block_size.
        if (self.write_offset.load(.monotonic) >= self.object_store_size_bytes)
            return;

        // Atomically fetch a block from the pool
        var block_begin = self.write_offset.fetchAdd(block_size, .monotonic);
        const block_end = @min(block_begin +% block_size, self.object_store_size_bytes);

        if (end.* == block_begin) {
            // Block is allocated straight after our previous block
            block_begin = begin.*;
        } else {
            // Block is a new block
            block_begin = @min(block_begin, self.object_store_size_bytes);
        }

        // Store the begin and end of the resulting block
        begin.* = block_begin;
        end.* = block_end;
    }

    /// Convert a pointer to an offset
    pub fn toOffset(self: *const LFHMAllocator, data: *const anyopaque) u32 {
        const address = @intFromPtr(data);
        const object_store = @intFromPtr(self.object_store.?);
        std.debug.assert(address >= object_store and address < object_store + self.object_store_size_bytes);
        return @intCast(address - object_store);
    }

    /// Convert an offset to a pointer
    pub fn fromOffset(self: *const LFHMAllocator, comptime T: type, offset: u32) *T {
        std.debug.assert(offset < self.object_store_size_bytes);
        return @ptrCast(@alignCast(self.object_store.? + offset));
    }
};

/// Allocator context object for a lock free hash map that allocates a larger memory block at once and hands it out in smaller portions.
/// This avoids contention on the atomic LFHMAllocator.write_offset.
/// Each thread that inserts into a map uses its own context.
pub const LFHMAllocatorContext = struct {
    lfhm_allocator: *LFHMAllocator,
    block_size: u32,
    begin: u32 = 0,
    end: u32 = 0,

    /// Construct a new allocator context
    pub fn init(lfhm_allocator: *LFHMAllocator, block_size: u32) LFHMAllocatorContext {
        return .{ .lfhm_allocator = lfhm_allocator, .block_size = block_size };
    }

    /// Allocate data block
    /// `size`: Size of block to allocate.
    /// `alignment`: Alignment of block to allocate.
    /// Returns the offset in the buffer where the block is located, or null when the allocation failed (C++: returns false).
    pub fn allocate(self: *LFHMAllocatorContext, size: u32, alignment: u32) ?u32 {
        // Calculate needed bytes for alignment
        std.debug.assert(math.isPowerOf2(alignment));
        const alignment_mask = alignment - 1;
        var alignment_bytes = (alignment - (self.begin & alignment_mask)) & alignment_mask;

        // Check if we have space
        if (self.end - self.begin < size +% alignment_bytes) {
            // Allocate a new block
            self.lfhm_allocator.allocate(self.block_size, &self.begin, &self.end);

            // Update alignment
            alignment_bytes = (alignment - (self.begin & alignment_mask)) & alignment_mask;

            // Check if we have space again
            if (self.end - self.begin < size +% alignment_bytes)
                return null;
        }

        // Make the allocation
        self.begin += alignment_bytes;
        const write_offset = self.begin;
        self.begin += size;
        return write_offset;
    }
};

/// Very simple lock free hash map that only allows insertion, retrieval and provides a fixed amount of buckets and fixed storage.
/// Note: This class currently assumes key and value are simple types that need no calls to the destructor.
///
/// Usage:
/// ```zig
/// var lfhm_allocator: LFHMAllocator = try .init(allocator, 1 << 20);
/// defer lfhm_allocator.deinit(allocator);
/// var map: LockFreeHashMap(u32, u32) = try .init(allocator, &lfhm_allocator, 1024);
/// defer map.deinit(allocator);
/// var context: LFHMAllocatorContext = .init(&lfhm_allocator, 4096); // One per thread
/// const kv = map.create(&context, key, HashCombine.hash(key), 0, value) orelse return error.MapFull;
/// const found = map.find(key, HashCombine.hash(key));
/// ```
pub fn LockFreeHashMap(comptime Key: type, comptime Value: type) type {
    return struct {
        const Self = @This();

        pub const MapType = Self;

        /// A key / value pair that is inserted in the map
        pub const KeyValue = extern struct {
            /// Key for this entry
            key: Key,

            /// Offset in the object store of next KeyValue entry with same hash
            next_offset: u32,

            /// Value for this entry + optionally extra bytes
            value: Value,

            pub fn getKey(self: *const KeyValue) *const Key {
                return &self.key;
            }

            pub fn getValue(self: *KeyValue) *Value {
                return &self.value;
            }

            /// Const version of getValue (GetValue() const)
            pub fn getValueConst(self: *const KeyValue) *const Value {
                return &self.value;
            }
        };

        /// Value of an invalid handle
        pub const invalid_handle: u32 = 0xffffffff;

        /// Allocator used to allocate key value pairs (mAllocator)
        lfhm_allocator: *const LFHMAllocator,

        /// Number of key value pairs in the store (only when Core.enable_asserts)
        num_key_values: if (Core.enable_asserts) std.atomic.Value(u32) else void,

        /// This contains the offset in the object store of the first object with a particular hash (max_buckets entries)
        buckets: []align(16) std.atomic.Value(u32),

        /// Current number of buckets
        num_buckets: u32,

        /// Maximum number of buckets
        max_buckets: u32,

        /// Constructor + initialization (LockFreeHashMap(LFHMAllocator &) followed by Init(inMaxBuckets))
        /// `allocator`: Allocates the buckets.
        /// `lfhm_allocator`: Stores the key value pairs, it must outlive the map and must not move.
        /// `max_buckets`: Max amount of buckets to use in the hashmap. Must be power of 2.
        pub fn init(allocator: std.mem.Allocator, lfhm_allocator: *const LFHMAllocator, max_buckets: u32) error{OutOfMemory}!Self {
            std.debug.assert(max_buckets >= 4 and math.isPowerOf2(max_buckets));

            var self: Self = .{
                .lfhm_allocator = lfhm_allocator,
                .num_key_values = if (Core.enable_asserts) .init(0) else {},
                .buckets = try allocator.alignedAlloc(std.atomic.Value(u32), .fromByteUnits(16), max_buckets),
                .num_buckets = max_buckets,
                .max_buckets = max_buckets,
            };
            self.clear();
            return self;
        }

        /// Destructor
        pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
            allocator.free(self.buckets);
            self.* = undefined;
        }

        /// Remove all elements.
        /// Note that this cannot happen simultaneously with adding new elements.
        /// The key value pairs stay in the LFHMAllocator, clear it separately.
        pub fn clear(self: *Self) void {
            // Reset number of key value pairs
            if (Core.enable_asserts) self.num_key_values.store(0, .seq_cst);

            // Reset buckets 4 at a time
            comptime std.debug.assert(@sizeOf(std.atomic.Value(u32)) == @sizeOf(u32));
            const invalid = UVec4.replicate(invalid_handle);
            const start: [*]align(16) u32 = @ptrCast(self.buckets.ptr);
            std.debug.assert(math.isAligned(start, 16));
            var i: u32 = 0;
            while (i < self.num_buckets) : (i += 4)
                invalid.storeInt4Aligned(@alignCast(start[i..][0..4]));
        }

        /// Get the current amount of buckets that the map is using
        pub fn getNumBuckets(self: *const Self) u32 {
            return self.num_buckets;
        }

        /// Get the maximum amount of buckets that this map supports
        pub fn getMaxBuckets(self: *const Self) u32 {
            return self.max_buckets;
        }

        /// Update the number of buckets. This must be done after clearing the map and cannot be done concurrently with any other operations on the map.
        /// Note that the number of buckets can never become bigger than the specified max buckets during initialization and that it must be a power of 2.
        pub fn setNumBuckets(self: *Self, num_buckets: u32) void {
            if (Core.enable_asserts) std.debug.assert(self.num_key_values.load(.seq_cst) == 0);
            std.debug.assert(num_buckets <= self.max_buckets);
            std.debug.assert(num_buckets >= 4 and math.isPowerOf2(num_buckets));

            self.num_buckets = num_buckets;
        }

        /// Insert a new element, returns null if map full.
        /// Multiple threads can be inserting in the map at the same time (each with its own context).
        /// `key_hash` is the hash of `key` (usually HashCombine.hash), `extra_bytes` are allocated directly after the
        /// value and `value` is the initial value (the C++ version forwards constructor parameters).
        pub fn create(self: *Self, context: *LFHMAllocatorContext, key: Key, key_hash: u64, extra_bytes: i32, value: Value) ?*KeyValue {
            // This is not a multi map, test the key hasn't been inserted yet
            if (Core.enable_asserts) std.debug.assert(self.find(key, key_hash) == null);

            // Calculate total size
            const size: u32 = @as(u32, @sizeOf(KeyValue)) +% @as(u32, @bitCast(extra_bytes));

            // Get the write offset for this key value pair
            const write_offset = context.allocate(size, @alignOf(KeyValue)) orelse return null;

            // Increment amount of entries in map
            if (Core.enable_asserts) _ = self.num_key_values.fetchAdd(1, .monotonic);

            // Construct the key/value pair
            const kv = self.lfhm_allocator.fromOffset(KeyValue, write_offset);
            std.debug.assert(@intFromPtr(kv) % @alignOf(KeyValue) == 0);
            if (builtin.mode == .Debug) {
                const bytes: [*]u8 = @ptrCast(kv);
                @memset(bytes[0..size], 0xcd);
            }
            kv.key = key;
            kv.value = value;

            // Get the offset to the first object from the bucket with corresponding hash
            const offset = &self.buckets[@intCast(key_hash & (self.num_buckets - 1))];

            // Add this entry as the first element in the linked list
            var old_offset = offset.load(.monotonic);
            while (true) {
                kv.next_offset = old_offset;
                old_offset = offset.cmpxchgWeak(old_offset, write_offset, .release, .monotonic) orelse break;
            }

            return kv;
        }

        /// Find an element, returns null if not found
        pub fn find(self: *const Self, key: Key, key_hash: u64) ?*const KeyValue {
            // Get the offset to the keyvalue object from the bucket with corresponding hash
            var offset = self.buckets[@intCast(key_hash & (self.num_buckets - 1))].load(.acquire);
            while (offset != invalid_handle) {
                // Loop through linked list of values until the right one is found
                const kv = self.lfhm_allocator.fromOffset(KeyValue, offset);
                if (keyEql(kv.key, key))
                    return kv;
                offset = kv.next_offset;
            }

            // Not found
            return null;
        }

        /// Get convert key value pair to uint32 handle
        pub fn toHandle(self: *const Self, key_value: *const KeyValue) u32 {
            return self.lfhm_allocator.toOffset(key_value);
        }

        /// Convert uint32 handle back to key and value (note that it is illegal to change the key this way).
        /// Covers both the const and the non const FromHandle of the C++ version.
        pub fn fromHandle(self: *const Self, handle: u32) *KeyValue {
            return self.lfhm_allocator.fromOffset(KeyValue, handle);
        }

        /// Get the number of key value pairs that this map currently contains.
        /// Available only when asserts are enabled because adding elements creates contention on this atomic and negatively affects performance.
        pub fn getNumKeyValues(self: *const Self) u32 {
            if (!Core.enable_asserts) @compileError("LockFreeHashMap.getNumKeyValues is only available when Core.enable_asserts (JPH_ENABLE_ASSERTS)");
            return self.num_key_values.load(.seq_cst);
        }

        /// Get all key/value pairs, they are appended to `all`
        pub fn getAllKeyValues(self: *const Self, allocator: std.mem.Allocator, all: *std.ArrayList(*const KeyValue)) error{OutOfMemory}!void {
            for (self.buckets[0..self.num_buckets]) |*bucket| {
                var offset = bucket.load(.seq_cst);
                while (offset != invalid_handle) {
                    const kv = self.lfhm_allocator.fromOffset(KeyValue, offset);
                    try all.append(allocator, kv);
                    offset = kv.next_offset;
                }
            }
        }

        /// Non-const iterator
        pub const Iterator = struct {
            map: *MapType,
            bucket: u32,
            offset: u32,

            /// Comparison (operator ==, `!it.eql(other)` is operator !=)
            pub fn eql(self: Iterator, rhs: Iterator) bool {
                return self.map == rhs.map and self.bucket == rhs.bucket and self.offset == rhs.offset;
            }

            /// Convert to key value pair (operator *)
            pub fn get(self: Iterator) *KeyValue {
                std.debug.assert(self.offset != invalid_handle);

                return self.map.lfhm_allocator.fromOffset(KeyValue, self.offset);
            }

            /// Next item (operator ++)
            pub fn advance(self: *Iterator) void {
                std.debug.assert(self.bucket < self.map.num_buckets);

                // Find the next key value in this bucket
                if (self.offset != invalid_handle) {
                    const kv = self.map.lfhm_allocator.fromOffset(KeyValue, self.offset);
                    self.offset = kv.next_offset;
                    if (self.offset != invalid_handle)
                        return;
                }

                // Loop over next buckets
                while (true) {
                    // Next bucket
                    self.bucket += 1;
                    if (self.bucket >= self.map.num_buckets)
                        return;

                    // Fetch the first entry in the bucket
                    self.offset = self.map.buckets[self.bucket].load(.seq_cst);
                    if (self.offset != invalid_handle)
                        return;
                }
            }

            /// Zig style iteration: returns the current key value pair and moves to the next one, null at the end
            pub fn next(self: *Iterator) ?*KeyValue {
                if (self.bucket >= self.map.num_buckets)
                    return null;
                const kv = self.get();
                self.advance();
                return kv;
            }
        };

        /// Iterate over the map, note that it is not safe to do this in parallel to clear().
        /// It is safe to do this while adding elements to the map, but newly added elements may or may not be returned by the iterator.
        pub fn begin(self: *Self) Iterator {
            // Start with the first bucket
            var it: Iterator = .{ .map = self, .bucket = 0, .offset = self.buckets[0].load(.seq_cst) };

            // If it doesn't contain a valid entry, use the ++ operator to find the first valid entry
            if (it.offset == invalid_handle)
                it.advance();

            return it;
        }

        pub fn end(self: *Self) Iterator {
            return .{ .map = self, .bucket = self.num_buckets, .offset = invalid_handle };
        }

        /// Output stats about this map to the log (Jolt only has this in debug builds, JPH_DEBUG)
        pub fn traceStats(self: *const Self) void {
            const max_per_bucket = 256;

            var max_objects_per_bucket: i32 = 0;
            var num_objects: i32 = 0;
            var histogram: [max_per_bucket]i32 = @splat(0);

            for (self.buckets[0..self.num_buckets]) |*bucket| {
                var objects_in_bucket: i32 = 0;
                var offset = bucket.load(.seq_cst);
                while (offset != invalid_handle) {
                    const kv = self.lfhm_allocator.fromOffset(KeyValue, offset);
                    offset = kv.next_offset;
                    objects_in_bucket += 1;
                    num_objects += 1;
                }
                max_objects_per_bucket = @max(objects_in_bucket, max_objects_per_bucket);
                histogram[@intCast(@min(objects_in_bucket, max_per_bucket - 1))] += 1;
            }

            log.info("max_objects_per_bucket = {d}, num_buckets = {d}, num_objects = {d}", .{ max_objects_per_bucket, self.num_buckets, num_objects });

            for (histogram, 0..) |count, i|
                if (count != 0) log.info("{d}: {d}", .{ i, count });
        }

        /// Compare keys, with `eql` when the key type declares it
        fn keyEql(a: Key, b: Key) bool {
            const has_eql = switch (@typeInfo(Key)) {
                .@"struct", .@"union", .@"enum", .@"opaque" => @hasDecl(Key, "eql"),
                else => false,
            };
            return if (has_eql) a.eql(b) else a == b;
        }
    };
}

const HashCombine = @import("HashCombine.zig");

test "LFHMAllocator and LFHMAllocatorContext" {
    const allocator = std.testing.allocator;

    var empty: LFHMAllocator = .empty;
    empty.deinit(allocator);

    var lfhm_allocator: LFHMAllocator = try .init(allocator, 256);
    defer lfhm_allocator.deinit(allocator);
    try std.testing.expect(math.isAligned(lfhm_allocator.object_store.?, 16));

    var context1: LFHMAllocatorContext = .init(&lfhm_allocator, 64);
    var context2: LFHMAllocatorContext = .init(&lfhm_allocator, 64);

    // Each context fetches a block of 64 bytes on its first allocation
    try std.testing.expectEqual(@as(?u32, 0), context1.allocate(12, 4)); // Block [0, 64)
    try std.testing.expectEqual(@as(?u32, 64), context2.allocate(12, 4)); // Block [64, 128)
    try std.testing.expectEqual(@as(?u32, 12), context1.allocate(12, 4));
    try std.testing.expectEqual(@as(?u32, 32), context1.allocate(1, 16)); // 24 aligned to 16
    try std.testing.expectEqual(@as(?u32, 40), context1.allocate(24, 8)); // 33 aligned to 8, exactly fills the block

    // Not enough space left: fetch a new block, which is not adjacent to the previous block of context1
    try std.testing.expectEqual(@as(?u32, 128), context1.allocate(4, 4)); // Block [128, 192)
    try std.testing.expectEqual(@as(?u32, 192), context2.allocate(60, 4)); // Block [192, 256)

    // The store is full, the allocator no longer hands out blocks but the current block can still be used
    try std.testing.expectEqual(@as(?u32, null), context1.allocate(64, 4));
    try std.testing.expectEqual(@as(u32, 256), lfhm_allocator.write_offset.load(.monotonic));
    try std.testing.expectEqual(@as(?u32, 132), context1.allocate(4, 4));
    try std.testing.expectEqual(@as(?u32, 252), context2.allocate(4, 1));
    try std.testing.expectEqual(@as(?u32, null), context2.allocate(1, 1));

    // A block that directly follows the previous block of the context extends it
    lfhm_allocator.clear();
    var context3: LFHMAllocatorContext = .init(&lfhm_allocator, 32);
    try std.testing.expectEqual(@as(?u32, 0), context3.allocate(20, 4)); // Block [0, 32)
    try std.testing.expectEqual(@as(?u32, 20), context3.allocate(20, 4)); // Block [32, 64) extends to [20, 64)
    try std.testing.expectEqual(@as(u32, 40), context3.begin);
    try std.testing.expectEqual(@as(u32, 64), context3.end);

    // The last block is clamped to the size of the store
    var context4: LFHMAllocatorContext = .init(&lfhm_allocator, 200);
    try std.testing.expectEqual(@as(?u32, 64), context4.allocate(100, 4)); // Block [64, 256)
    try std.testing.expectEqual(@as(u32, 256), context4.end);
    try std.testing.expectEqual(@as(?u32, null), context4.allocate(100, 4));

    // Offsets and pointers
    const ptr = lfhm_allocator.fromOffset(u32, 64);
    try std.testing.expectEqual(@intFromPtr(lfhm_allocator.object_store.?) + 64, @intFromPtr(ptr));
    try std.testing.expectEqual(@as(u32, 64), lfhm_allocator.toOffset(ptr));
}

test "LockFreeHashMap single threaded" {
    const allocator = std.testing.allocator;
    const Map = LockFreeHashMap(u32, u32);
    try std.testing.expectEqual(12, @sizeOf(Map.KeyValue));

    var lfhm_allocator: LFHMAllocator = try .init(allocator, 4096);
    defer lfhm_allocator.deinit(allocator);
    var map: Map = try .init(allocator, &lfhm_allocator, 16);
    defer map.deinit(allocator);
    try std.testing.expectEqual(@as(u32, 16), map.getNumBuckets());
    try std.testing.expectEqual(@as(u32, 16), map.getMaxBuckets());

    // Empty map
    var it = map.begin();
    try std.testing.expect(it.eql(map.end()));
    try std.testing.expectEqual(@as(?*Map.KeyValue, null), it.next());

    for (0..2) |pass| {
        var context: LFHMAllocatorContext = .init(&lfhm_allocator, 128);

        // Insert keys
        const num_keys = 100;
        for (0..num_keys) |i| {
            const key: u32 = @intCast(i * 7 + pass);
            const kv = map.create(&context, key, HashCombine.hash(key), 0, key ^ 0xffff).?;
            try std.testing.expectEqual(key, kv.getKey().*);
            try std.testing.expectEqual(key ^ 0xffff, kv.getValue().*);
            try std.testing.expectEqual(kv, map.fromHandle(map.toHandle(kv)));
        }
        if (Core.enable_asserts) try std.testing.expectEqual(@as(u32, num_keys), map.getNumKeyValues());

        // Find them
        for (0..num_keys) |i| {
            const key: u32 = @intCast(i * 7 + pass);
            const kv = map.find(key, HashCombine.hash(key)).?;
            try std.testing.expectEqual(key, kv.getKey().*);
            try std.testing.expectEqual(key ^ 0xffff, kv.getValueConst().*);
            const missing: u32 = key + 1;
            try std.testing.expectEqual(@as(?*const Map.KeyValue, null), map.find(missing, HashCombine.hash(missing)));
        }

        // All iteration methods return the same elements in the same order: by bucket, newest first within a bucket
        var all: std.ArrayList(*const Map.KeyValue) = .empty;
        defer all.deinit(allocator);
        try map.getAllKeyValues(allocator, &all);
        try std.testing.expectEqual(@as(usize, num_keys), all.items.len);

        var count: usize = 0;
        it = map.begin();
        while (!it.eql(map.end())) : (it.advance()) {
            try std.testing.expectEqual(all.items[count], it.get());
            count += 1;
        }
        try std.testing.expectEqual(@as(usize, num_keys), count);

        count = 0;
        it = map.begin();
        var prev_bucket: u64 = 0;
        var prev_handle: u32 = 0;
        while (it.next()) |kv| {
            try std.testing.expectEqual(all.items[count], kv);
            const bucket = HashCombine.hash(kv.getKey().*) & (map.getNumBuckets() - 1);
            try std.testing.expect(bucket >= prev_bucket);
            if (count > 0 and bucket == prev_bucket)
                try std.testing.expect(map.toHandle(kv) < prev_handle);
            prev_bucket = bucket;
            prev_handle = map.toHandle(kv);
            count += 1;
        }
        try std.testing.expectEqual(@as(usize, num_keys), count);
        map.traceStats();

        // Clear the map
        lfhm_allocator.clear();
        map.clear();
        try std.testing.expect(map.begin().eql(map.end()));
        for (0..num_keys) |i| {
            const key: u32 = @intCast(i * 7 + pass);
            try std.testing.expectEqual(@as(?*const Map.KeyValue, null), map.find(key, HashCombine.hash(key)));
        }
        if (Core.enable_asserts) try std.testing.expectEqual(@as(u32, 0), map.getNumKeyValues());

        // The second pass uses less buckets
        map.setNumBuckets(4);
        try std.testing.expectEqual(@as(u32, 4), map.getNumBuckets());
    }

    // Fill up the map: with a single context the blocks are contiguous, so all 4096 / 12 entries fit
    lfhm_allocator.clear();
    map.clear();
    map.setNumBuckets(16);
    var context: LFHMAllocatorContext = .init(&lfhm_allocator, 128);
    var num_created: u32 = 0;
    while (map.create(&context, num_created, HashCombine.hash(num_created), 0, num_created)) |_|
        num_created += 1;
    try std.testing.expectEqual(@as(u32, 4096 / 12), num_created);
    for (0..num_created) |i| {
        const key: u32 = @intCast(i);
        try std.testing.expectEqual(key, map.find(key, HashCombine.hash(key)).?.getValueConst().*);
    }
}

test "LockFreeHashMap struct keys, collisions and extra bytes" {
    const allocator = std.testing.allocator;

    const Key = extern struct {
        a: u32,
        b: u32,

        pub fn eql(self: @This(), other: @This()) bool {
            return self.a == other.a and self.b == other.b;
        }
    };

    // A value with a variable amount of data, like ContactConstraintManager's CachedManifold
    const Value = extern struct {
        id: u64,
        num_data: u32,
        data: [1]u32,

        fn getData(self: *@This()) [*]u32 {
            return &self.data;
        }
    };

    const Map = LockFreeHashMap(Key, Value);
    try std.testing.expectEqual(8, @alignOf(Map.KeyValue));
    try std.testing.expectEqual(32, @sizeOf(Map.KeyValue));
    try std.testing.expectEqual(16, @offsetOf(Map.KeyValue, "value"));

    var lfhm_allocator: LFHMAllocator = try .init(allocator, 16384);
    defer lfhm_allocator.deinit(allocator);
    var map: Map = try .init(allocator, &lfhm_allocator, 64);
    defer map.deinit(allocator);
    var context: LFHMAllocatorContext = .init(&lfhm_allocator, 256);

    // Every key has the same hash, so they all end up in the same bucket and are told apart by Key.eql
    const hash: u64 = 0x1234_5678_9abc_def0;
    const num_keys = 50;
    for (0..num_keys) |i| {
        const num_data: u32 = @intCast(i % 7 + 1);
        const kv = map.create(&context, .{ .a = @intCast(i), .b = @intCast(i * 3) }, hash, @intCast((num_data - 1) * @sizeOf(u32)), .{ .id = i, .num_data = num_data, .data = .{0} }).?;
        try std.testing.expect(std.mem.isAligned(@intFromPtr(kv), 8));
        const data = kv.getValue().getData();
        for (0..num_data) |j| data[j] = @intCast(i * 100 + j);
    }

    // The extra bytes didn't overwrite other entries
    for (0..num_keys) |i| {
        const kv = map.find(.{ .a = @intCast(i), .b = @intCast(i * 3) }, hash).?;
        const value = map.fromHandle(map.toHandle(kv)).getValue();
        try std.testing.expectEqual(@as(u64, i), value.id);
        try std.testing.expectEqual(@as(u32, @intCast(i % 7 + 1)), value.num_data);
        for (0..value.num_data) |j|
            try std.testing.expectEqual(@as(u32, @intCast(i * 100 + j)), value.getData()[j]);
        try std.testing.expectEqual(@as(?*const Map.KeyValue, null), map.find(.{ .a = @intCast(i), .b = 0xffff }, hash));
    }

    // One bucket, newest entry first
    var it = map.begin();
    var expected_id: u64 = num_keys;
    while (it.next()) |kv| {
        expected_id -= 1;
        try std.testing.expectEqual(expected_id, kv.getValueConst().id);
    }
    try std.testing.expectEqual(@as(u64, 0), expected_id);
}

test "LockFreeHashMap multi threaded create and find" {
    if (builtin.single_threaded) return error.SkipZigTest;
    const allocator = std.testing.allocator;

    const num_threads = 4;
    const keys_per_thread = 25_000;
    const Map = LockFreeHashMap(u32, u32);

    const Context = struct {
        lfhm_allocator: *LFHMAllocator,
        map: *Map,
        done: std.atomic.Value(u32) = .init(0),
        failed: std.atomic.Value(bool) = .init(false),

        fn valueForKey(key: u32) u32 {
            return key ^ 0xdeadbeef;
        }

        fn insert(self: *@This(), thread_index: u32) void {
            defer _ = self.done.fetchAdd(1, .release);
            var rng: std.Random.DefaultPrng = .init(thread_index);
            var context: LFHMAllocatorContext = .init(self.lfhm_allocator, 256);
            for (0..keys_per_thread) |k| {
                const key: u32 = @intCast(k * num_threads + thread_index);
                const kv = self.map.create(&context, key, HashCombine.hash(key), 0, valueForKey(key)) orelse {
                    self.failed.store(true, .monotonic);
                    return;
                };
                if (kv.getKey().* != key)
                    self.failed.store(true, .monotonic);

                // Our own key must be found
                const found = self.map.find(key, HashCombine.hash(key));
                if (found != kv)
                    self.failed.store(true, .monotonic);

                // Keys of other threads may or may not be there yet, but if they are their value must be complete
                const other: u32 = rng.random().uintLessThan(u32, num_threads * keys_per_thread);
                if (self.map.find(other, HashCombine.hash(other))) |other_kv| {
                    if (other_kv.getValueConst().* != valueForKey(other))
                        self.failed.store(true, .monotonic);
                }
            }
        }

        fn iterate(self: *@This()) void {
            // Iterating while elements are added is allowed, every returned element must be complete
            while (self.done.load(.acquire) < num_threads) {
                var it = self.map.begin();
                while (it.next()) |kv| {
                    if (kv.getValueConst().* != valueForKey(kv.getKey().*))
                        self.failed.store(true, .monotonic);
                }
            }
        }
    };

    // Each context wastes at most the remainder of a block when switching blocks
    var lfhm_allocator: LFHMAllocator = try .init(allocator, 2 * num_threads * keys_per_thread * @sizeOf(Map.KeyValue));
    defer lfhm_allocator.deinit(allocator);
    var map: Map = try .init(allocator, &lfhm_allocator, 1024);
    defer map.deinit(allocator);

    var context: Context = .{ .lfhm_allocator = &lfhm_allocator, .map = &map };
    var threads: [num_threads + 1]std.Thread = undefined;
    for (threads[0..num_threads], 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Context.insert, .{ &context, @as(u32, @intCast(i)) });
    threads[num_threads] = try std.Thread.spawn(.{}, Context.iterate, .{&context});
    for (threads) |t| t.join();

    try std.testing.expect(!context.failed.load(.monotonic));
    const total = num_threads * keys_per_thread;
    if (Core.enable_asserts) try std.testing.expectEqual(@as(u32, total), map.getNumKeyValues());

    // Every key is there exactly once
    const seen = try allocator.alloc(bool, total);
    defer allocator.free(seen);
    @memset(seen, false);
    var it = map.begin();
    var count: u32 = 0;
    while (it.next()) |kv| {
        const key = kv.getKey().*;
        try std.testing.expect(key < total);
        try std.testing.expect(!seen[key]);
        seen[key] = true;
        try std.testing.expectEqual(Context.valueForKey(key), kv.getValueConst().*);
        count += 1;
    }
    try std.testing.expectEqual(@as(u32, total), count);
    for (0..total) |i| {
        const key: u32 = @intCast(i);
        try std.testing.expectEqual(Context.valueForKey(key), map.find(key, HashCombine.hash(key)).?.getValueConst().*);
    }
}
