//! Port of: UnitTests/Core/QuickSortTest.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const quickSort = zolt.quickSort;
const asc = std.sort.asc;
const desc = std.sort.desc;

test "TestOrderedArray" {
    var array: [100]i32 = undefined;
    for (&array, 0..) |*v, i| v.* = @intCast(i);

    quickSort(i32, &array, {}, asc(i32));

    for (array, 0..) |v, i| try fw.expectEqual(@as(i32, @intCast(i)), v);
}

test "TestOrderedArrayComparator" {
    var array: [100]i32 = undefined;
    for (&array, 0..) |*v, i| v.* = @intCast(i);

    quickSort(i32, &array, {}, desc(i32)); // greater<int>

    for (array, 0..) |v, i| try fw.expectEqual(99 - @as(i32, @intCast(i)), v);
}

test "TestReversedArray" {
    var array: [100]i32 = undefined;
    for (&array, 0..) |*v, i| v.* = 99 - @as(i32, @intCast(i));

    quickSort(i32, &array, {}, asc(i32));

    for (array, 0..) |v, i| try fw.expectEqual(@as(i32, @intCast(i)), v);
}

test "TestRandomArray" {
    var random = fw.UnitTestRandom.init(fw.UnitTestRandom.default_seed);

    var array: std.ArrayList(u32) = .empty;
    defer array.deinit(std.testing.allocator);
    for (0..1000) |_| {
        const value = random.next();

        // Insert value at beginning
        try array.insert(std.testing.allocator, 0, value);

        // Insert value at end
        try array.append(std.testing.allocator, value);
    }

    quickSort(u32, array.items, {}, asc(u32));

    var i: usize = 0;
    while (i < array.items.len - 2) : (i += 2) {
        // We inserted the same value twice so these elements should be the same
        try fw.expect(array.items[i] == array.items[i + 1]);

        // The next element should be bigger or equal
        try fw.expect(array.items[i] <= array.items[i + 2]);
    }
}

test "TestEmptyArray" {
    var array = [_]i32{};
    quickSort(i32, &array, {}, asc(i32));
    try fw.expect(array.len == 0);
}

test "Test1ElementArray" {
    var array = [_]i32{1};
    quickSort(i32, &array, {}, asc(i32));
    try fw.expectEqual(@as(i32, 1), array[0]);
}

test "Test2ElementArray" {
    var array = [_]i32{ 2, 1 };
    quickSort(i32, &array, {}, asc(i32));
    try fw.expectEqual(@as(i32, 1), array[0]);
    try fw.expectEqual(@as(i32, 2), array[1]);
}
