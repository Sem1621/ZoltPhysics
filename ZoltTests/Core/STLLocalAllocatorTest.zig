//! Port of: UnitTests/Core/STLLocalAllocatorTest.cpp
//!
//! Jolt's test uses `Array<T, STLLocalAllocator<T, N>>`. Zolt uses std.ArrayList with the
//! std.mem.Allocator of an STLLocalAllocator (one allocator per array, like the allocator member of
//! the C++ Array). The Array operations of the test are reproduced with Jolt's capacity policy so that
//! the local buffer is used and given up at the same points as in the C++:
//! - `pushBack` is `Array::push_back`: grow to max(size + 1, 2 * capacity) (Array::grow), then move the
//!   value in;
//! - `assign` is `Array::operator =`: clear, reserve exactly the size, copy construct the elements;
//! - `reserve` is `ensureTotalCapacityPrecise`, `clear` + `shrink_to_fit` is `clearAndFree`.
//! Zig has no copy / move constructors. The non trivial types of the C++ test count moves in
//! `mMakeNonTriv` and set it to -999 when copied; here the helpers call `moved()` / `copied()` on the
//! elements where the C++ Array would call the move / copy constructor: when an element is pushed, when
//! the array's memory relocates (the data pointer changes) and when the array is assigned.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const STLLocalAllocator = zolt.STLLocalAllocator;
const expect = fw.expect;
const expectEqual = fw.expectEqual;

/// The number of elements in the local buffer
const N = 20;

/// Force the need for an aligned allocation (struct alignas(64) Aligned)
const Aligned = struct {
    value: i32 align(64),

    fn init(value: i32) Aligned {
        return .{ .value = value };
    }

    /// operator int()
    fn toInt(self: Aligned) i32 {
        return self.value;
    }
};

/// Force non trivial copy constructor
const NonTriv = struct {
    value: i32,
    make_non_triv: i32 = 0,

    fn init(value: i32) NonTriv {
        return .{ .value = value };
    }

    /// NonTriv(const NonTriv &inRHS) : mValue(inRHS.mValue), mMakeNonTriv(-999)
    fn copied(self: NonTriv) NonTriv {
        return .{ .value = self.value, .make_non_triv = -999 };
    }

    /// NonTriv(NonTriv &&inRHS) : mValue(inRHS.mValue), mMakeNonTriv(inRHS.mMakeNonTriv + 1)
    fn moved(self: NonTriv) NonTriv {
        return .{ .value = self.value, .make_non_triv = self.make_non_triv + 1 };
    }

    /// operator int()
    fn toInt(self: NonTriv) i32 {
        return self.value;
    }

    fn getNonTriv(self: NonTriv) i32 {
        return self.make_non_triv;
    }
};

/// Force non trivial copy constructor (struct alignas(64) AlNonTriv)
const AlNonTriv = struct {
    value: i32 align(64),
    make_non_triv: i32 = 0,

    fn init(value: i32) AlNonTriv {
        return .{ .value = value };
    }

    /// AlNonTriv(const AlNonTriv &inRHS) : mValue(inRHS.mValue), mMakeNonTriv(-999)
    fn copied(self: AlNonTriv) AlNonTriv {
        return .{ .value = self.value, .make_non_triv = -999 };
    }

    /// AlNonTriv(AlNonTriv &&inRHS) : mValue(inRHS.mValue), mMakeNonTriv(inRHS.mMakeNonTriv + 1)
    fn moved(self: AlNonTriv) AlNonTriv {
        return .{ .value = self.value, .make_non_triv = self.make_non_triv + 1 };
    }

    /// operator int()
    fn toInt(self: AlNonTriv) i32 {
        return self.value;
    }

    fn getNonTriv(self: AlNonTriv) i32 {
        return self.make_non_triv;
    }
};

/// Element constructed from an int (the implicit conversion in `arr.push_back(i)`)
fn fromInt(comptime T: type, value: i32) T {
    return if (T == i32) value else T.init(value);
}

/// operator int()
fn toInt(comptime T: type, element: T) i32 {
    return if (T == i32) element else element.toInt();
}

/// Move constructor
fn moved(comptime T: type, element: T) T {
    return if (T != i32 and @hasDecl(T, "moved")) element.moved() else element;
}

/// Copy constructor
fn copied(comptime T: type, element: T) T {
    return if (T != i32 and @hasDecl(T, "copied")) element.copied() else element;
}

/// Array::push_back(T &&) with Array::grow()
fn pushBack(comptime T: type, list: *std.ArrayList(T), allocator: std.mem.Allocator, value: i32) !void {
    const min_size = list.items.len + 1;
    if (min_size > list.capacity) {
        const old_data = list.items.ptr;
        try list.ensureTotalCapacityPrecise(allocator, @max(min_size, list.capacity * 2));

        // When the memory relocated, the Array moved the elements to the new memory
        if (list.items.ptr != old_data) {
            for (list.items) |*element|
                element.* = moved(T, element.*);
        }
    }
    list.appendAssumeCapacity(moved(T, fromInt(T, value)));
}

/// Array::operator = (const Array &): assign(begin, end) clears, reserves the size and copy constructs the elements
fn assign(comptime T: type, list: *std.ArrayList(T), allocator: std.mem.Allocator, items: []const T) !void {
    list.clearRetainingCapacity();
    try list.ensureTotalCapacityPrecise(allocator, items.len);
    for (items) |element|
        list.appendAssumeCapacity(copied(T, element));
}

/// The heap of Jolt's STLAllocator<T> for a non trivially copyable T: it has no reallocate, so the elements
/// always move to a new block when a heap array grows. Forwards to std.testing.allocator without resize / remap.
const NoReallocAllocator = struct {
    fn allocator() std.mem.Allocator {
        return .{ .ptr = undefined, .vtable = &.{
            .alloc = alloc,
            .resize = std.mem.Allocator.noResize,
            .remap = std.mem.Allocator.noRemap,
            .free = free,
        } };
    }

    fn alloc(_: *anyopaque, len: usize, alignment: std.mem.Alignment, ret_addr: usize) ?[*]u8 {
        return std.testing.allocator.rawAlloc(len, alignment, ret_addr);
    }

    fn free(_: *anyopaque, memory: []u8, alignment: std.mem.Alignment, ret_addr: usize) void {
        std.testing.allocator.rawFree(memory, alignment, ret_addr);
    }
};

fn testArray(comptime T: type, comptime non_trivial: bool) !void {
    const Allocator = STLLocalAllocator(T, N);
    comptime std.debug.assert(Allocator.has_reallocate);
    const heap = if (non_trivial) NoReallocAllocator.allocator() else std.testing.allocator;

    // Allocate so that we will run out of local memory and reallocate from heap at least once
    var arr_allocator = Allocator.init(heap);
    const arr_gpa = arr_allocator.allocator();
    var arr: std.ArrayList(T) = .empty;
    defer arr.deinit(arr_gpa);
    for (0..64) |i|
        try pushBack(T, &arr, arr_gpa, @intCast(i));
    try expectEqual(@as(usize, 64), arr.items.len);
    for (arr.items, 0..) |element, i| {
        try expectEqual(@as(i32, @intCast(i)), toInt(T, element));
        // We only have to move elements once we run out of the local buffer, this happens as we resize
        // from 16 to 32 elements, we'll reallocate again at 32 and 64
        if (non_trivial)
            try expectEqual(@as(i32, if (i < 16) 3 else if (i < 32) 2 else 1), element.getNonTriv());
    }
    try expect(std.mem.isAligned(@intFromPtr(arr.items.ptr), @alignOf(T)));
    try expect(!arr_allocator.isLocal(arr.items.ptr));

    // Check that we can copy the array to another array
    var arr2_allocator = Allocator.init(heap);
    const arr2_gpa = arr2_allocator.allocator();
    var arr2: std.ArrayList(T) = .empty;
    defer arr2.deinit(arr2_gpa);
    try assign(T, &arr2, arr2_gpa, arr.items);
    for (arr2.items, 0..) |element, i| {
        try expectEqual(@as(i32, @intCast(i)), toInt(T, element));
        if (non_trivial)
            try expectEqual(@as(i32, -999), element.getNonTriv());
    }
    try expect(std.mem.isAligned(@intFromPtr(arr2.items.ptr), @alignOf(T)));
    try expect(!arr2_allocator.isLocal(arr2.items.ptr));

    // Clear the array
    arr.clearAndFree(arr_gpa); // arr.clear(); arr.shrink_to_fit();
    try expectEqual(@as(usize, 0), arr.items.len);
    try expectEqual(@as(usize, 0), arr.capacity);
    // Not ported: CHECK(arr.data() == nullptr), Zig slices are never null

    // Allocate so we stay within the local buffer
    for (0..10) |i|
        try pushBack(T, &arr, arr_gpa, @intCast(i));
    try expectEqual(@as(usize, 10), arr.items.len);
    for (arr.items, 0..) |element, i| {
        try expectEqual(@as(i32, @intCast(i)), toInt(T, element));
        // We never need to move elements as they stay within the local buffer
        if (non_trivial)
            try expectEqual(@as(i32, 1), element.getNonTriv());
    }
    try expect(std.mem.isAligned(@intFromPtr(arr.items.ptr), @alignOf(T)));
    try expect(arr_allocator.isLocal(arr.items.ptr));

    // Check that we can copy the array to the local buffer
    var arr3_allocator = Allocator.init(heap);
    const arr3_gpa = arr3_allocator.allocator();
    var arr3: std.ArrayList(T) = .empty;
    defer arr3.deinit(arr3_gpa);
    try assign(T, &arr3, arr3_gpa, arr.items);
    try expectEqual(@as(usize, 10), arr3.items.len);
    for (arr3.items, 0..) |element, i| {
        try expectEqual(@as(i32, @intCast(i)), toInt(T, element));
        if (non_trivial)
            try expectEqual(@as(i32, -999), element.getNonTriv());
    }
    try expect(std.mem.isAligned(@intFromPtr(arr3.items.ptr), @alignOf(T)));
    try expect(arr3_allocator.isLocal(arr3.items.ptr));

    // Check that if we reserve the memory, that we can fully fill the array in local memory
    var arr4_allocator = Allocator.init(heap);
    const arr4_gpa = arr4_allocator.allocator();
    var arr4: std.ArrayList(T) = .empty;
    defer arr4.deinit(arr4_gpa);
    try arr4.ensureTotalCapacityPrecise(arr4_gpa, N); // arr4.reserve(N)
    for (0..N) |i|
        try pushBack(T, &arr4, arr4_gpa, @intCast(i));
    try expectEqual(@as(usize, N), arr4.items.len);
    try expectEqual(@as(usize, N), arr4.capacity);
    for (arr4.items, 0..) |element, i| {
        try expectEqual(@as(i32, @intCast(i)), toInt(T, element));
        if (non_trivial)
            try expectEqual(@as(i32, 1), element.getNonTriv());
    }
    try expect(std.mem.isAligned(@intFromPtr(arr4.items.ptr), @alignOf(T)));
    try expect(arr4_allocator.isLocal(arr4.items.ptr));
}

test "TestAllocation" {
    try testArray(i32, false);
}

test "TestAllocationAligned" {
    comptime std.debug.assert(@alignOf(Aligned) == 64);
    try testArray(Aligned, false);
}

test "TestAllocationNonTrivial" {
    try testArray(NonTriv, true);
}

test "TestAllocationAlignedNonTrivial" {
    comptime std.debug.assert(@alignOf(AlNonTriv) == 64);
    try testArray(AlNonTriv, true);
}
