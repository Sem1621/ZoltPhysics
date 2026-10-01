//! Port of: Jolt/Core/StaticArray.h
//! Status: complete
//!
//! Method names follow std.ArrayList (see "Containers" in the porting guide) so that code ported
//! from `Array<T>` and `StaticArray<T, N>` reads the same: push_back -> append, pop_back -> pop,
//! size() -> len, erase -> orderedRemove / orderedRemoveRange, operator[] -> slice()[i] / at(i).

const std = @import("std");
const HashCombine = @import("HashCombine.zig");

/// Simple variable length array backed by a fixed size buffer
pub fn StaticArray(comptime T: type, comptime N: u32) type {
    return struct {
        const Self = @This();

        /// Maximum amount of elements the array can hold (Capacity)
        pub const capacity: u32 = N;

        /// Amount of elements in the array (size())
        len: u32 = 0,

        /// Storage, only the first `len` elements are initialized
        buffer: [N]T = undefined,

        /// Empty array
        pub const empty: Self = .{};

        /// Constructor from initializer list
        pub fn fromSlice(items: []const T) Self {
            std.debug.assert(items.len <= N);
            var result: Self = .{};
            @memcpy(result.buffer[0..items.len], items);
            result.len = @intCast(items.len);
            return result;
        }

        /// Assignment from a static array with a different max length (operator = (const StaticArray<T, M> &))
        pub fn assign(self: *Self, other: anytype) void {
            const items = other.constSlice();
            std.debug.assert(items.len <= N);
            if (@intFromPtr(self) == @intFromPtr(other)) return;
            @memcpy(self.buffer[0..items.len], items);
            self.len = @intCast(items.len);
        }

        /// The elements as a slice (begin() / end() / data())
        pub fn slice(self: *Self) []T {
            return self.buffer[0..self.len];
        }

        /// The elements as a const slice
        pub fn constSlice(self: *const Self) []const T {
            return self.buffer[0..self.len];
        }

        /// Set length to zero
        pub fn clear(self: *Self) void {
            self.len = 0;
        }

        /// Add element to the back of the array (push_back / emplace_back)
        pub fn append(self: *Self, element: T) void {
            std.debug.assert(self.len < N);
            self.buffer[self.len] = element;
            self.len += 1;
        }

        /// Remove element from the back of the array and return it (pop_back)
        pub fn pop(self: *Self) T {
            std.debug.assert(self.len > 0);
            self.len -= 1;
            return self.buffer[self.len];
        }

        /// Returns true if there are no elements in the array
        pub fn isEmpty(self: *const Self) bool {
            return self.len == 0;
        }

        /// Resize array to new length, new elements are undefined
        pub fn resize(self: *Self, new_len: u32) void {
            std.debug.assert(new_len <= N);
            self.len = new_len;
        }

        /// Access element (at / operator [])
        pub fn at(self: *Self, index: u32) *T {
            std.debug.assert(index < self.len);
            return &self.buffer[index];
        }

        /// Get element by value (const at / operator [])
        pub fn get(self: *const Self, index: u32) T {
            std.debug.assert(index < self.len);
            return self.buffer[index];
        }

        /// First element in the array (front)
        pub fn front(self: *Self) *T {
            std.debug.assert(self.len > 0);
            return &self.buffer[0];
        }

        /// Last element in the array (back)
        pub fn back(self: *Self) *T {
            std.debug.assert(self.len > 0);
            return &self.buffer[self.len - 1];
        }

        /// Remove one element from the array, moving the elements after it (erase(iterator))
        pub fn orderedRemove(self: *Self, index: u32) void {
            std.debug.assert(index < self.len);
            self.orderedRemoveRange(index, index + 1);
        }

        /// Remove the elements [begin, end) from the array (erase(begin, end))
        pub fn orderedRemoveRange(self: *Self, begin: u32, end: u32) void {
            std.debug.assert(begin <= end and end <= self.len);
            const n = end - begin;
            if (end < self.len)
                std.mem.copyForwards(T, self.buffer[begin .. self.len - n], self.buffer[end..self.len]);
            self.len -= n;
        }

        /// Comparing arrays (operator ==). Uses `eql` when T has one.
        pub fn eql(self: *const Self, other: *const Self) bool {
            if (self.len != other.len)
                return false;
            for (self.constSlice(), other.constSlice()) |a, b| {
                const equal = if (comptime hasEql()) a.eql(b) else a == b;
                if (!equal)
                    return false;
            }
            return true;
        }

        /// Get hash for this array
        pub fn getHash(self: *const Self) u64 {
            // Hash length first
            var ret = HashCombine.hash(self.len);

            // Then hash elements
            for (self.constSlice()) |element|
                HashCombine.hashCombine(&ret, element);
            return ret;
        }

        fn hasEql() bool {
            return switch (@typeInfo(T)) {
                .@"struct", .@"union", .@"enum", .@"opaque" => @hasDecl(T, "eql"),
                else => false,
            };
        }
    };
}

test "StaticArray" {
    const Array = StaticArray(u32, 8);
    var a = Array.fromSlice(&.{ 1, 2, 3, 4, 5 });
    try std.testing.expectEqual(@as(u32, 5), a.len);
    try std.testing.expectEqual(@as(u32, 8), Array.capacity);

    a.append(6);
    try std.testing.expectEqual(@as(u32, 6), a.back().*);
    try std.testing.expectEqual(@as(u32, 6), a.pop());

    a.orderedRemove(1);
    try std.testing.expectEqualSlices(u32, &.{ 1, 3, 4, 5 }, a.constSlice());
    a.orderedRemoveRange(1, 3);
    try std.testing.expectEqualSlices(u32, &.{ 1, 5 }, a.constSlice());
    a.at(0).* = 7;
    try std.testing.expectEqual(@as(u32, 7), a.get(0));

    var b: StaticArray(u32, 4) = .empty;
    try std.testing.expect(b.isEmpty());
    b.assign(&a);
    try std.testing.expectEqualSlices(u32, &.{ 7, 5 }, b.constSlice());

    var c = Array.fromSlice(&.{ 7, 5 });
    try std.testing.expect(a.eql(&c));
    try std.testing.expectEqual(a.getHash(), c.getHash());
    c.slice()[1] = 6;
    try std.testing.expect(!a.eql(&c));

    a.resize(4);
    try std.testing.expectEqual(@as(u32, 4), a.len);
    a.clear();
    try std.testing.expect(a.isEmpty());
}
