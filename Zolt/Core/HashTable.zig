//! Port of: Jolt/Core/HashTable.h
//! Status: complete
//!
//! The iteration order of this table is observable (it can change simulation results), so it is a
//! literal port of Jolt's open addressing table: same hash, control byte encoding, probing sequence,
//! growth / rehash policy and tombstones.
//!
//! Differences with the C++ class, following the porting guide:
//! - The `Hash` and `KeyEqual` template parameters are the comptime `HashTableOptions(Key)` struct:
//!   `.{}` uses `HashCombine.hash` (Jolt's `Hash<Key>`) and `==` / `eql()` / `std.mem.eql`
//!   (`std::equal_to<Key>`); override them with `.{ .hash = myHash, .key_equal = myEqual }`.
//! - Unmanaged allocation: every function that may allocate takes the allocator, free the table
//!   with `deinit(allocator)`. Zig has no destructors: elements that own memory must be deinitialized
//!   by the caller before `deinit` / `clearAndFree` / `clearRetainingCapacity` / `erase`.
//!   `clone` / `assign` (copy constructor / assignment operator) copy the elements bitwise (shallow): when elements
//!   own memory (e.g. an `std.ArrayList` value) the copy shares it with the original, so the caller must duplicate
//!   those values in the copy (or not copy such tables).
//! - Iterators become pointers to the element (`find` returns `?*const KeyValue`, null is `end()`)
//!   and iterator structs with `next()` (`begin()` / `end()` loops). `next()` combines the C++ iterator
//!   operators (`++`, `*`, `->`, `==`, `IsValid`) and visits the elements in the same (bucket) order.
//!   The bucket index of an element (the iterator's `mIndex`) is `indexOf(ptr)`.
//! - The table is always allocated with the alignment of `KeyValue`, so `cNeedsAlignedAllocate` is not needed.
//! - Container names follow the std hash maps where the semantics are the same: `size()` -> `count()`,
//!   `empty()` -> `isEmpty()`, `reserve` -> `ensureTotalCapacity`, `clear` (frees memory) -> `clearAndFree`,
//!   `ClearAndKeepMemory` -> `clearRetainingCapacity`, copy constructor -> `clone`.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("Core.zig");
const HashCombine = @import("HashCombine.zig");
const BVec16 = @import("../Math/BVec16.zig").BVec16;
const math = @import("../Math/Math.zig");

/// The `Hash` and `KeyEqual` template parameters of HashTable, UnorderedMap and UnorderedSet
pub fn HashTableOptions(comptime Key: type) type {
    return struct {
        /// Hash function (note should be 64-bits), default: Jolt's `Hash<Key>`
        hash: fn (Key) u64 = defaultHash(Key),

        /// Equality comparison function, default: `std::equal_to<Key>`
        key_equal: fn (Key, Key) bool = defaultKeyEqual(Key),
    };
}

/// The default hash function, `Hash<Key>` (see `HashCombine.hash`)
pub fn defaultHash(comptime Key: type) fn (Key) u64 {
    return struct {
        fn hashKey(key: Key) u64 {
            return HashCombine.hash(key);
        }
    }.hashKey;
}

/// The default equality function, `std::equal_to<Key>`: `eql()` for types that declare it,
/// `std.mem.eql` for slices (compares the contents, like `string_view` / `String`) and `==` otherwise.
pub fn defaultKeyEqual(comptime Key: type) fn (Key, Key) bool {
    return struct {
        fn keyEqual(a: Key, b: Key) bool {
            switch (@typeInfo(Key)) {
                .@"struct", .@"union", .@"enum" => if (comptime @hasDecl(Key, "eql")) {
                    // eql can take the other value by value or by pointer
                    const Other = @typeInfo(@TypeOf(Key.eql)).@"fn".params[1].type.?;
                    if (comptime @typeInfo(Other) == .pointer)
                        return a.eql(&b)
                    else
                        return a.eql(b);
                } else if (comptime @typeInfo(Key) != .@"enum") {
                    @compileError(@typeName(Key) ++ " needs an eql() method or a custom key_equal function to be used as a hash table key");
                },
                .pointer => |pointer| if (comptime pointer.size == .slice) return std.mem.eql(pointer.child, a, b),
                else => {},
            }
            return a == b;
        }
    }.keyEqual;
}

/// Helper class for implementing an UnorderedSet or UnorderedMap
/// Based on CppCon 2017: Matt Kulukundis "Designing a Fast, Efficient, Cache-friendly Hash Table, Step by Step"
/// See: https://www.youtube.com/watch?v=ncHmEUmJZf4
///
/// `HashTableDetail` provides `fn getKey(key_value: *const KeyValue) Key` (sGetKey).
pub fn HashTable(comptime Key: type, comptime KeyValue: type, comptime HashTableDetail: type, comptime options: HashTableOptions(Key)) type {
    return struct {
        const Self = @This();

        /// Properties (value_type)
        pub const value_type = KeyValue;

        /// Properties (size_type)
        pub const size_type = u32;

        /// Properties (difference_type)
        pub const difference_type = isize;

        /// Max load factor is max_load_factor_numerator / max_load_factor_denominator
        const max_load_factor_numerator: u64 = 7;
        const max_load_factor_denominator: u64 = 8;

        /// If we can recover this fraction of deleted elements, we'll reshuffle the buckets in place rather than growing the table
        const max_deleted_elements_numerator: u64 = 1;
        const max_deleted_elements_denominator: u64 = 8;

        /// Values that the control bytes can have
        const bucket_empty: u8 = 0;
        const bucket_deleted: u8 = 0x7f;
        const bucket_used: u8 = 0x80; // Lowest 7 bits are lowest 7 bits of the hash value

        /// The buckets, an array of size max_size
        data: ?[*]KeyValue = null,

        /// Control bytes, an array of size max_size + 15
        control: ?[*]u8 = null,

        /// Number of elements in the table
        size: u32 = 0,

        /// Max number of elements that can be stored in the table
        max_size: u32 = 0,

        /// Number of elements we can add to the table before we need to grow
        load_left: u32 = 0,

        /// Empty table (default constructor)
        pub const empty: Self = .{};

        /// Result of `insert` / `UnorderedMap.tryEmplace` (std::pair<iterator, bool>)
        pub const InsertResult = struct {
            /// The inserted element or the element that already existed
            ptr: *KeyValue,
            /// If the element was inserted
            inserted: bool,
        };

        /// Result of `insertKey` (return value + outIndex)
        pub const InsertKeyResult = struct {
            /// True if the element was inserted, false if it already existed
            inserted: bool,
            /// The index at which the element should be constructed / where it is located
            index: u32,
        };

        /// Base class for iterators (IteratorBase). Visits the elements in bucket order.
        fn IteratorBase(comptime is_const: bool) type {
            return struct {
                const It = @This();

                table: if (is_const) *const Self else *Self,

                /// Index of the next bucket to check
                index: u32 = 0,

                /// Get the next element (operator ++ and operator *), null at the end of the table
                pub fn next(it: *It) ?(if (is_const) *const KeyValue else *KeyValue) {
                    const table = it.table;
                    while (it.index < table.max_size) {
                        const index = it.index;
                        it.index += 1;
                        if ((table.control.?[index] & bucket_used) != 0)
                            return &table.data.?[index];
                    }
                    return null;
                }
            };
        }

        /// Non-const iterator
        pub const Iterator = IteratorBase(false);

        /// Const iterator
        pub const ConstIterator = IteratorBase(true);

        /// Split hash into index and control value (out parameters of GetIndexAndControlValue)
        const IndexAndControl = struct {
            index: u32,
            control: u8,
        };

        /// Get the maximum number of elements that we can support given a number of buckets
        fn getMaxLoad(bucket_count: u32) u32 {
            return @intCast((max_load_factor_numerator * bucket_count) / max_load_factor_denominator);
        }

        /// Update the control value for a bucket
        fn setControlValue(self: *Self, index: u32, value: u8) void {
            std.debug.assert(index < self.max_size);
            const control = self.control.?;
            control[index] = value;

            // Mirror the first 15 bytes to the 15 bytes beyond max_size
            // Note that this is equivalent to:
            // if (index < 15)
            //   control[index + max_size] = value
            // else
            //   control[index] = value
            // Which performs a needless write if index >= 15 but at least it is branch-less
            control[((index -% 15) & (self.max_size -% 1)) + 15] = value;
        }

        /// Get the index and control value for a particular key
        fn getIndexAndControlValue(self: *const Self, key: Key) IndexAndControl {
            // Calculate hash
            const hash_value: u64 = options.hash(key);

            // Split hash into index and control value
            return .{
                .index = @as(u32, @truncate(hash_value >> 7)) & (self.max_size -% 1),
                .control = bucket_used | @as(u8, @truncate(hash_value)),
            };
        }

        /// Size in bytes of the memory block for `max_size` buckets
        fn requiredSize(max_size: u32) usize {
            return @as(usize, max_size) * (@sizeOf(KeyValue) + 1) + 15; // Add 15 bytes to mirror the first 15 bytes of the control values
        }

        /// Allocate space for the hash table
        fn allocateTable(self: *Self, allocator: Allocator, max_size: u32) Allocator.Error!void {
            std.debug.assert(self.data == null);

            // Zolt: allocate first so that the table is unchanged when the allocation fails
            const memory = try allocator.alignedAlloc(u8, .of(KeyValue), requiredSize(max_size));

            self.max_size = max_size;
            self.load_left = getMaxLoad(max_size);
            const data: [*]KeyValue = @ptrCast(memory.ptr);
            const control: [*]u8 = @ptrCast(data + max_size);
            self.data = data;
            self.control = control;
        }

        /// Free the memory block returned by allocateTable
        fn freeTable(allocator: Allocator, data: [*]KeyValue, max_size: u32) void {
            const memory: [*]align(@alignOf(KeyValue)) u8 = @ptrCast(data);
            allocator.free(memory[0..requiredSize(max_size)]);
        }

        /// Copy the contents of another hash table
        fn copyTable(self: *Self, allocator: Allocator, other: *const Self) Allocator.Error!void {
            if (other.isEmpty())
                return;

            try self.allocateTable(allocator, other.max_size);

            // Copy control bytes
            const control = self.control.?;
            @memcpy(control[0 .. self.max_size + 15], other.control.?[0 .. self.max_size + 15]);

            // Copy elements
            // Note: like Jolt, load_left is not copied, it keeps the max load that allocateTable set. The copy can
            // therefore take more elements than the original before it grows or rehashes (and the unsigned
            // `max load - size` computations in insertKey / rehash can wrap around).
            const data = self.data.?;
            const other_data = other.data.?;
            for (0..self.max_size) |index|
                if ((control[index] & bucket_used) != 0) {
                    data[index] = other_data[index]; // Zolt: a bitwise copy, Zig has no copy constructors
                };
            self.size = other.size;
        }

        /// Grow the table to a new size
        fn growTable(self: *Self, allocator: Allocator, new_max_size: u32) Allocator.Error!void {
            // Move the old table to a temporary structure
            const old_max_size = self.max_size;
            const old_data = self.data;
            const old_control = self.control;
            const old_size = self.size;
            const old_load_left = self.load_left;
            self.data = null;
            self.control = null;
            self.size = 0;
            self.max_size = 0;
            self.load_left = 0;

            // Allocate new table
            self.allocateTable(allocator, new_max_size) catch |err| {
                // Zolt: restore the old table when the allocation fails
                self.data = old_data;
                self.control = old_control;
                self.size = old_size;
                self.max_size = old_max_size;
                self.load_left = old_load_left;
                return err;
            };

            // Reset all control bytes
            @memset(self.control.?[0 .. self.max_size + 15], bucket_empty);

            if (old_data) |data| {
                // Copy all elements from the old table
                const control = old_control.?;
                for (0..old_max_size) |i|
                    if ((control[i] & bucket_used) != 0) {
                        const element = &data[i];
                        // Inserting after a grow never grows the table, so it cannot fail
                        const result = self.insertKey(allocator, HashTableDetail.getKey(element), true) catch unreachable;
                        std.debug.assert(result.inserted);
                        self.data.?[result.index] = element.*;
                    };

                // Free memory
                freeTable(allocator, data, old_max_size);
            }
        }

        /// Get an element by index (protected in Jolt)
        pub fn getElement(self: *const Self, index: u32) *KeyValue {
            return &self.data.?[index];
        }

        /// Insert a key into the map, returns true if the element was inserted, false if it already existed.
        /// index is the index at which the element should be constructed / where it is located.
        /// (protected in Jolt) The caller must construct the element at index when it was inserted.
        pub fn insertKey(self: *Self, allocator: Allocator, key: Key, comptime insert_after_grow: bool) Allocator.Error!InsertKeyResult {
            // Ensure we have enough space
            if (self.load_left == 0) {
                // Should not be growing if we're already growing!
                std.debug.assert(!insert_after_grow);

                // Decide if we need to clean up all tombstones or if we need to grow the map
                const num_deleted: u32 = getMaxLoad(self.max_size) -% self.size; // Wraps when the table is a copy that was filled beyond its max load, see copyTable
                if (@as(u64, num_deleted) * max_deleted_elements_denominator > @as(u64, self.max_size) * max_deleted_elements_numerator) {
                    self.rehash(0);
                } else {
                    // Grow by a power of 2
                    const new_max_size: u32 = @max(self.max_size << 1, 16);
                    if (new_max_size < self.max_size) {
                        if (Core.enable_asserts) @panic("Overflow in hash table size, can't grow!");
                        return error.OutOfMemory; // Jolt returns false (not inserted) here
                    }
                    try self.growTable(allocator, new_max_size);
                }
            }

            // Split hash into index and control value
            const index_and_control = self.getIndexAndControlValue(key);
            var index = index_and_control.index;
            const control = index_and_control.control;

            // Keeps track of the index of the first deleted bucket we found
            const no_deleted: u32 = ~@as(u32, 0);
            var first_deleted_index = no_deleted;

            // Linear probing
            const control_ptr = self.control.?;
            const bucket_mask = self.max_size - 1;
            const control16 = BVec16.replicate(control);
            const bucket_empty16 = BVec16.zero();
            const bucket_deleted16 = BVec16.replicate(bucket_deleted);
            while (true) {
                // Read 16 control values (note that we added 15 bytes at the end of the control values that mirror the first 15 bytes)
                const control_bytes = BVec16.loadByte16(control_ptr[index..][0..16]);

                // Check if we must find the element before we can insert
                if (!insert_after_grow) {
                    // Check for the control value we're looking for
                    // Note that when deleting we can create empty buckets instead of deleted buckets.
                    // This means we must unconditionally check all buckets in this batch for equality
                    // (also beyond the first empty bucket).
                    var control_equal: u32 = BVec16.equals(control_bytes, control16).getTrues();

                    // Index within the 16 buckets
                    var local_index = index;

                    // Loop while there's still buckets to process
                    while (control_equal != 0) {
                        // Get the first equal bucket
                        const first_equal = math.countTrailingZeros(control_equal);

                        // Skip to the bucket
                        local_index += first_equal;

                        // Make sure that our index is not beyond the end of the table
                        local_index &= bucket_mask;

                        // We found a bucket with same control value
                        if (options.key_equal(HashTableDetail.getKey(&self.data.?[local_index]), key)) {
                            // Element already exists
                            return .{ .inserted = false, .index = local_index };
                        }

                        // Skip past this bucket
                        control_equal >>= @intCast(first_equal + 1);
                        local_index += 1;
                    }

                    // Check if we're still scanning for deleted buckets
                    if (first_deleted_index == no_deleted) {
                        // Check if any buckets have been deleted, if so store the first one
                        const control_deleted: u32 = BVec16.equals(control_bytes, bucket_deleted16).getTrues();
                        if (control_deleted != 0)
                            first_deleted_index = index + math.countTrailingZeros(control_deleted);
                    }
                }

                // Check for empty buckets
                const control_empty: u32 = BVec16.equals(control_bytes, bucket_empty16).getTrues();
                if (control_empty != 0) {
                    // If we found a deleted bucket, use it.
                    // It doesn't matter if it is before or after the first empty bucket we found
                    // since we will always be scanning in batches of 16 buckets.
                    if (first_deleted_index == no_deleted or insert_after_grow) {
                        index += math.countTrailingZeros(control_empty);
                        self.load_left -%= 1; // Using an empty bucket decreases the load left
                    } else {
                        index = first_deleted_index;
                    }

                    // Make sure that our index is not beyond the end of the table
                    index &= bucket_mask;

                    // Update control byte
                    self.setControlValue(index, control);
                    self.size +%= 1; // Wraps like the C++ uint32, see eraseByPtr

                    // Return index to newly allocated bucket
                    return .{ .inserted = true, .index = index };
                }

                // Move to next batch of 16 buckets
                index = (index + 16) & bucket_mask;
            }
        }

        /// Copy constructor. Zolt: elements are copied bitwise, elements that own memory must be duplicated by the caller.
        pub fn clone(self: *const Self, allocator: Allocator) Allocator.Error!Self {
            var result: Self = .empty;
            try result.copyTable(allocator, self);
            return result;
        }

        /// Move constructor: returns the contents of this table and leaves it empty
        pub fn move(self: *Self) Self {
            const result = self.*;
            self.* = .empty;
            return result;
        }

        /// Assignment operator. Zolt: elements are copied bitwise, elements that own memory must be duplicated by the caller.
        pub fn assign(self: *Self, allocator: Allocator, other: *const Self) Allocator.Error!void {
            if (self != other) {
                self.clearAndFree(allocator);

                try self.copyTable(allocator, other);
            }
        }

        /// Move assignment operator
        pub fn assignMove(self: *Self, allocator: Allocator, other: *Self) void {
            if (self != other) {
                self.clearAndFree(allocator);

                self.* = other.*;
                other.* = .empty;
            }
        }

        /// Destructor
        pub fn deinit(self: *Self, allocator: Allocator) void {
            self.clearAndFree(allocator);
        }

        /// Reserve memory for a certain number of elements (reserve)
        pub fn ensureTotalCapacity(self: *Self, allocator: Allocator, max_size: u32) Allocator.Error!void {
            // Calculate max size based on load factor
            const new_max_size = math.getNextPowerOf2(@max(@as(u32, @truncate((max_load_factor_denominator * max_size) / max_load_factor_numerator)), 16));
            if (new_max_size <= self.max_size)
                return;

            try self.growTable(allocator, new_max_size);
        }

        /// Destroy the entire hash table (clear)
        pub fn clearAndFree(self: *Self, allocator: Allocator) void {
            // Zolt: elements are not deinitialized (no destructors), the caller does that when needed

            if (self.data) |data| {
                // Free memory
                freeTable(allocator, data, self.max_size);

                // Reset members
                self.data = null;
                self.control = null;
                self.size = 0;
                self.max_size = 0;
                self.load_left = 0;
            }
        }

        /// Destroy the entire hash table but keeps the memory allocated (ClearAndKeepMemory)
        pub fn clearRetainingCapacity(self: *Self) void {
            // Zolt: elements are not deinitialized (no destructors), the caller does that when needed
            self.size = 0;

            // If there are elements that are not marked bucket_empty, we reset them
            const max_load = getMaxLoad(self.max_size);
            if (self.load_left != max_load) {
                // Reset all control bytes
                @memset(self.control.?[0 .. self.max_size + 15], bucket_empty);
                self.load_left = max_load;
            }
        }

        /// Iterate over all elements (begin() / end())
        pub fn iterator(self: *Self) Iterator {
            return .{ .table = self };
        }

        /// Iterate over all elements, const version (begin() const / cbegin() / end() const / cend())
        pub fn constIterator(self: *const Self) ConstIterator {
            return .{ .table = self };
        }

        /// Number of buckets in the table
        pub fn bucketCount(self: *const Self) u32 {
            return self.max_size;
        }

        /// Max number of buckets that the table can have
        pub fn maxBucketCount(self: *const Self) u32 {
            _ = self;
            return @as(u32, 1) << (@sizeOf(u32) * 8 - 1);
        }

        /// Check if there are no elements in the table (empty)
        pub fn isEmpty(self: *const Self) bool {
            return self.size == 0;
        }

        /// Number of elements in the table (size)
        pub fn count(self: *const Self) u32 {
            return self.size;
        }

        /// Max number of elements that the table can hold
        pub fn maxSize(self: *const Self) u32 {
            return @intCast((@as(u64, self.maxBucketCount()) * max_load_factor_numerator) / max_load_factor_denominator);
        }

        /// Get the max load factor for this table (max number of elements / number of buckets)
        pub fn maxLoadFactor(self: *const Self) f32 {
            _ = self;
            return @as(f32, @floatFromInt(max_load_factor_numerator)) / @as(f32, @floatFromInt(max_load_factor_denominator));
        }

        /// Insert a new element, returns the element and if the element was inserted
        pub fn insert(self: *Self, allocator: Allocator, value: KeyValue) Allocator.Error!InsertResult {
            const result = try self.insertKey(allocator, HashTableDetail.getKey(&value), false);
            const element = &self.data.?[result.index];
            if (result.inserted)
                element.* = value;
            return .{ .ptr = element, .inserted = result.inserted };
        }

        /// Find an element, returns the element or null (end()) if not found
        pub fn find(self: *const Self, key: Key) ?*const KeyValue {
            // Check if we have any data
            if (self.isEmpty())
                return null;

            // Split hash into index and control value
            const index_and_control = self.getIndexAndControlValue(key);
            var index = index_and_control.index;
            const control = index_and_control.control;

            // Linear probing
            const control_ptr = self.control.?;
            const bucket_mask = self.max_size - 1;
            const control16 = BVec16.replicate(control);
            const bucket_empty16 = BVec16.zero();
            while (true) {
                // Read 16 control values
                // (note that we added 15 bytes at the end of the control values that mirror the first 15 bytes)
                const control_bytes = BVec16.loadByte16(control_ptr[index..][0..16]);

                // Check for the control value we're looking for
                // Note that when deleting we can create empty buckets instead of deleted buckets.
                // This means we must unconditionally check all buckets in this batch for equality
                // (also beyond the first empty bucket).
                var control_equal: u32 = BVec16.equals(control_bytes, control16).getTrues();

                // Index within the 16 buckets
                var local_index = index;

                // Loop while there's still buckets to process
                while (control_equal != 0) {
                    // Get the first equal bucket
                    const first_equal = math.countTrailingZeros(control_equal);

                    // Skip to the bucket
                    local_index += first_equal;

                    // Make sure that our index is not beyond the end of the table
                    local_index &= bucket_mask;

                    // We found a bucket with same control value
                    const element = &self.data.?[local_index];
                    if (options.key_equal(HashTableDetail.getKey(element), key)) {
                        // Element found
                        return element;
                    }

                    // Skip past this bucket
                    control_equal >>= @intCast(first_equal + 1);
                    local_index += 1;
                }

                // Check for empty buckets
                const control_empty: u32 = BVec16.equals(control_bytes, bucket_empty16).getTrues();
                if (control_empty != 0) {
                    // An empty bucket was found, we didn't find the element
                    return null;
                }

                // Move to next batch of 16 buckets
                index = (index + 16) & bucket_mask;
            }
        }

        /// Index of the bucket that holds `element` (the mIndex of a C++ iterator)
        pub fn indexOf(self: *const Self, element: *const KeyValue) u32 {
            const index: u32 = @intCast((@intFromPtr(element) - @intFromPtr(self.data.?)) / @sizeOf(KeyValue));
            std.debug.assert(index < self.max_size);
            return index;
        }

        /// Erase an element by iterator (erase(const_iterator)), `element` must point into this table
        pub fn eraseByPtr(self: *Self, element: *const KeyValue) void {
            const index = self.indexOf(element);
            const control = self.control.?;
            std.debug.assert((control[index] & bucket_used) != 0); // IsValid

            // Read 16 control values before and after the current index
            // (note that we added 15 bytes at the end of the control values that mirror the first 15 bytes)
            const control_bytes_before = BVec16.loadByte16(control[((index -% 16) & (self.max_size - 1))..][0..16]);
            const control_bytes_after = BVec16.loadByte16(control[index..][0..16]);
            const bucket_empty16 = BVec16.zero();
            const control_empty_before: u32 = BVec16.equals(control_bytes_before, bucket_empty16).getTrues();
            const control_empty_after: u32 = BVec16.equals(control_bytes_after, bucket_empty16).getTrues();

            // If (this index including) there exist 16 consecutive non-empty slots (represented by a bit being 0) then
            // a probe looking for some element needs to continue probing so we cannot mark the bucket as empty
            // but must mark it as deleted instead.
            // Note that we use: CountLeadingZeros(uint16) = CountLeadingZeros(uint32) - 16.
            const control_value: u8 = if (math.countLeadingZeros(control_empty_before) - 16 + math.countTrailingZeros(control_empty_after) < 16) bucket_empty else bucket_deleted;

            // Mark the bucket as empty/deleted
            self.setControlValue(index, control_value);

            // Zolt: the element is not deinitialized (no destructors), the caller does that when needed

            // If we marked the bucket as empty we can increase the load left
            if (control_value == bucket_empty)
                self.load_left +%= 1;

            // Decrease size
            // Zolt: wraps like the C++ uint32. A copy keeps the max load as load left (see copyTable), so
            // clearRetainingCapacity on a copy sets the size to 0 without resetting the control bytes, the iterator
            // still visits the old elements and erasing one of them makes the size wrap around.
            self.size -%= 1;
        }

        /// Erase an element by key, returns the number of elements erased (0 or 1)
        pub fn erase(self: *Self, key: Key) u32 {
            const element = self.find(key) orelse return 0;

            self.eraseByPtr(element);
            return 1;
        }

        /// Swap the contents of two hash tables
        pub fn swap(self: *Self, other: *Self) void {
            std.mem.swap(?[*]KeyValue, &self.data, &other.data);
            std.mem.swap(?[*]u8, &self.control, &other.control);
            std.mem.swap(u32, &self.size, &other.size);
            std.mem.swap(u32, &self.max_size, &other.max_size);
            std.mem.swap(u32, &self.load_left, &other.load_left);
        }

        /// In place re-hashing of all elements in the table. Removes all bucket_deleted elements
        /// The std version takes a bucket count, but we just re-hash to the same size.
        pub fn rehash(self: *Self, _: u32) void {
            // Zolt: a table without buckets has nothing to rehash (Jolt would dereference a null pointer)
            const control = self.control orelse return;
            const data = self.data.?;

            // Update the control value for all buckets
            for (control[0..self.max_size]) |*c| {
                switch (c.*) {
                    bucket_deleted => {
                        // Deleted buckets become empty
                        c.* = bucket_empty;
                    },
                    bucket_empty => {
                        // Remains empty
                    },
                    else => {
                        // Mark all occupied as deleted, to indicate it needs to move to the correct place
                        c.* = bucket_deleted;
                    },
                }
            }

            // Replicate control values to the last 15 entries
            for (0..15) |i|
                control[self.max_size + i] = control[i];

            // Loop over all elements that have been 'deleted' and move them to their new spot
            const bucket_used16 = BVec16.replicate(bucket_used);
            const bucket_mask = self.max_size - 1;
            const probe_mask: u32 = bucket_mask & ~@as(u32, 0b1111); // Mask out lower 4 bits because we test 16 buckets at a time
            var src: u32 = 0;
            while (src < self.max_size) : (src += 1) {
                if (control[src] == bucket_deleted) {
                    while (true) {
                        // Split hash into index and control value
                        const src_index_and_control = self.getIndexAndControlValue(HashTableDetail.getKey(&data[src]));
                        const src_index = src_index_and_control.index;
                        const src_control = src_index_and_control.control;

                        // Linear probing
                        var dst = src_index;
                        while (true) {
                            // Check if any buckets are free
                            const control_bytes = BVec16.loadByte16(control[dst..][0..16]);
                            const control_free: u32 = BVec16.bitAnd(control_bytes, bucket_used16).getTrues() ^ 0xffff;
                            if (control_free != 0) {
                                // Select this bucket as destination
                                dst += math.countTrailingZeros(control_free);
                                dst &= bucket_mask;
                                break;
                            }

                            // Move to next batch of 16 buckets
                            dst = (dst + 16) & bucket_mask;
                        }

                        // Check if we stay in the same probe group
                        if (((dst -% src_index) & probe_mask) == ((src -% src_index) & probe_mask)) {
                            // We stay in the same group, we can stay where we are
                            self.setControlValue(src, src_control);
                            break;
                        } else if (control[dst] == bucket_empty) {
                            // There's an empty bucket, move us there
                            self.setControlValue(dst, src_control);
                            self.setControlValue(src, bucket_empty);
                            data[dst] = data[src];
                            break;
                        } else {
                            // There's an element in the bucket we want to move to, swap them
                            std.debug.assert(control[dst] == bucket_deleted);
                            self.setControlValue(dst, src_control);
                            std.mem.swap(KeyValue, &data[src], &data[dst]);
                            // Iterate again with the same source bucket
                        }
                    }
                }
            }

            // Reinitialize load left
            self.load_left = getMaxLoad(self.max_size) -% self.size;
        }
    };
}

/// Detail for the tests below: a set of u32
const TestSetDetail = struct {
    fn getKey(key: *const u32) u32 {
        return key.*;
    }
};

test "HashTable API" {
    const allocator = std.testing.allocator;
    const Table = HashTable(u32, u32, TestSetDetail, .{});

    var table: Table = .empty;
    defer table.deinit(allocator);
    try std.testing.expectEqual(0, table.bucketCount());
    try std.testing.expect(table.isEmpty());
    try std.testing.expect(table.find(1) == null);
    try std.testing.expectEqual(0, table.erase(1));
    table.rehash(0); // No buckets: nothing to do
    table.clearRetainingCapacity();

    for (0..100) |i| {
        const result = try table.insert(allocator, @intCast(i));
        try std.testing.expect(result.inserted);
        try std.testing.expectEqual(@as(u32, @intCast(i)), result.ptr.*);
        try std.testing.expectEqual(result.ptr, table.getElement(table.indexOf(result.ptr)));
    }
    try std.testing.expectEqual(100, table.count());
    try std.testing.expectEqual(128, table.bucketCount());
    try std.testing.expectEqual(0.875, table.maxLoadFactor());
    try std.testing.expectEqual(0x80000000, table.maxBucketCount());
    try std.testing.expectEqual(0x70000000, table.maxSize());

    // insertKey reports existing keys
    const existing = try table.insertKey(allocator, 5, false);
    try std.testing.expect(!existing.inserted);
    try std.testing.expectEqual(5, table.getElement(existing.index).*);

    // Clone / assign keep the exact layout
    var copy = try table.clone(allocator);
    defer copy.deinit(allocator);
    var assigned: Table = .empty;
    defer assigned.deinit(allocator);
    _ = try assigned.insert(allocator, 1000);
    try assigned.assign(allocator, &table);
    try assigned.assign(allocator, &assigned); // Self assignment is a no-op
    var it1 = table.constIterator();
    var it2 = copy.iterator();
    var it3 = assigned.constIterator();
    while (it1.next()) |v| {
        try std.testing.expectEqual(v.*, it2.next().?.*);
        try std.testing.expectEqual(v.*, it3.next().?.*);
        try std.testing.expectEqual(table.indexOf(v), copy.indexOf(copy.find(v.*).?));
    }
    try std.testing.expect(it2.next() == null);
    try std.testing.expect(it3.next() == null);

    // Erase by pointer and key
    table.eraseByPtr(table.find(10).?);
    try std.testing.expect(table.find(10) == null);
    try std.testing.expectEqual(1, table.erase(11));
    try std.testing.expectEqual(98, table.count());

    // Rehash keeps all elements
    table.rehash(0);
    for (0..100) |i|
        try std.testing.expectEqual(i != 10 and i != 11, table.find(@intCast(i)) != null);

    // Move / move assign / swap
    var moved = table.move();
    defer moved.deinit(allocator);
    try std.testing.expect(table.isEmpty());
    try std.testing.expectEqual(0, table.bucketCount());
    copy.assignMove(allocator, &moved);
    try std.testing.expect(moved.isEmpty());
    try std.testing.expectEqual(98, copy.count());
    copy.assignMove(allocator, &copy); // Self assignment is a no-op
    copy.swap(&moved);
    try std.testing.expect(copy.isEmpty());
    try std.testing.expectEqual(98, moved.count());

    // Clear but keep memory, then clear and free
    moved.clearRetainingCapacity();
    try std.testing.expect(moved.isEmpty());
    try std.testing.expectEqual(128, moved.bucketCount());
    try std.testing.expect(moved.find(20) == null);
    moved.clearAndFree(allocator);
    try std.testing.expectEqual(0, moved.bucketCount());

    // Cloning an empty table gives a table without buckets
    var empty_clone = try moved.clone(allocator);
    try std.testing.expectEqual(0, empty_clone.bucketCount());
    empty_clone.deinit(allocator);

    // Reserving less than the current size does nothing
    try assigned.ensureTotalCapacity(allocator, 1);
    try std.testing.expectEqual(128, assigned.bucketCount());
}

test "HashTable allocation failure leaves the table unchanged" {
    const Table = HashTable(u32, u32, TestSetDetail, .{});
    var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = 1 });
    const allocator = failing.allocator();

    var table: Table = .empty;
    defer table.deinit(allocator);
    for (0..14) |i|
        _ = try table.insert(allocator, @intCast(i));
    try std.testing.expectEqual(16, table.bucketCount());

    // The next insert needs to grow the table, which fails
    try std.testing.expectError(error.OutOfMemory, table.insert(allocator, 14));
    try std.testing.expectEqual(14, table.count());
    try std.testing.expectEqual(16, table.bucketCount());
    for (0..14) |i|
        try std.testing.expect(table.find(@intCast(i)) != null);
}

test "HashTable size wraps around like the C++ uint32" {
    const allocator = std.testing.allocator;
    const Table = HashTable(u32, u32, TestSetDetail, .{});

    var table: Table = .empty;
    defer table.deinit(allocator);
    for (0..4) |i|
        _ = try table.insert(allocator, @intCast(i));

    // The copy keeps the max load as load left, so clearRetainingCapacity doesn't reset its control bytes
    var copy = try table.clone(allocator);
    defer copy.deinit(allocator);
    copy.clearRetainingCapacity();
    try std.testing.expectEqual(0, copy.count());

    // The iterator still visits the old elements, erasing one makes the size wrap around
    var it = copy.iterator();
    const stale = it.next().?;
    const stale_key = stale.*;
    copy.eraseByPtr(stale);
    try std.testing.expectEqual(0xffffffff, copy.count());
    try std.testing.expect(copy.find(stale_key) == null);

    // Inserting a new element wraps it back
    try std.testing.expect((try copy.insert(allocator, 100)).inserted);
    try std.testing.expectEqual(0, copy.count());
}

test "HashTable keys with getHash / eql and keys that need custom functions" {
    const allocator = std.testing.allocator;

    // A key like MeshShape's Edge: uses the defaults (getHash and eql)
    const Edge = struct {
        idx1: u32,
        idx2: u32,
        pub fn getHash(self: @This()) u64 {
            return HashCombine.hashCombineArgs(.{ self.idx1, self.idx2 });
        }
        pub fn eql(self: @This(), other: @This()) bool {
            return self.idx1 == other.idx1 and self.idx2 == other.idx2;
        }
    };
    const EdgeDetail = struct {
        fn getKey(key: *const Edge) Edge {
            return key.*;
        }
    };
    var edges: HashTable(Edge, Edge, EdgeDetail, .{}) = .empty;
    defer edges.deinit(allocator);
    for (0..100) |i|
        try std.testing.expect((try edges.insert(allocator, .{ .idx1 = @intCast(i), .idx2 = @intCast(i + 1) })).inserted);
    try std.testing.expect(!(try edges.insert(allocator, .{ .idx1 = 5, .idx2 = 6 })).inserted);
    try std.testing.expect(edges.find(.{ .idx1 = 6, .idx2 = 5 }) == null);
    try std.testing.expectEqual(7, edges.find(.{ .idx1 = 6, .idx2 = 7 }).?.idx2);

    // A key without getHash and eql: the default functions must not be analyzed when custom ones are given
    const Plain = struct {
        value: u32,
    };
    const PlainFuncs = struct {
        fn hash(key: Plain) u64 {
            return HashCombine.hash64(key.value);
        }
        fn equal(a: Plain, b: Plain) bool {
            return a.value == b.value;
        }
        fn getKey(key: *const Plain) Plain {
            return key.*;
        }
    };
    var plain: HashTable(Plain, Plain, PlainFuncs, .{ .hash = PlainFuncs.hash, .key_equal = PlainFuncs.equal }) = .empty;
    defer plain.deinit(allocator);
    for (0..100) |i|
        try std.testing.expect((try plain.insert(allocator, .{ .value = @intCast(i) })).inserted);
    try std.testing.expectEqual(100, plain.count());
    try std.testing.expectEqual(42, plain.find(.{ .value = 42 }).?.value);
    try std.testing.expect(plain.find(.{ .value = 100 }) == null);
}

test "defaultKeyEqual" {
    const Point = struct {
        x: i32,
        pub fn eql(self: @This(), other: @This()) bool {
            return self.x == other.x;
        }
    };
    const PointByPtr = struct {
        x: i32,
        pub fn eql(self: *const @This(), other: *const @This()) bool {
            return self.x == other.x;
        }
    };
    const Enum = enum { a, b };
    try std.testing.expect(defaultKeyEqual(u32)(1, 1));
    try std.testing.expect(!defaultKeyEqual(u64)(1, 2));
    try std.testing.expect(defaultKeyEqual(f32)(0.0, -0.0));
    try std.testing.expect(!defaultKeyEqual(f32)(std.math.nan(f32), std.math.nan(f32)));
    try std.testing.expect(defaultKeyEqual(Point)(.{ .x = 1 }, .{ .x = 1 }));
    try std.testing.expect(!defaultKeyEqual(PointByPtr)(.{ .x = 1 }, .{ .x = 2 }));
    try std.testing.expect(defaultKeyEqual(Enum)(.a, .a));
    try std.testing.expect(defaultKeyEqual([]const u8)("abc", "abc"));
    try std.testing.expect(!defaultKeyEqual([]const u8)("abc", "abd"));
    const values = [_]u32{ 1, 2 };
    try std.testing.expect(defaultKeyEqual(*const u32)(&values[0], &values[0]));
    try std.testing.expect(!defaultKeyEqual(*const u32)(&values[0], &values[1]));
    try std.testing.expectEqual(HashCombine.hash(@as(u32, 5)), defaultHash(u32)(5));
    try std.testing.expectEqual(HashCombine.hash(@as([]const u8, "abc")), defaultHash([]const u8)("abc"));
}
