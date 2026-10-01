//! Port of: UnitTests/Core/BinaryHeapTest.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

test "TestBinaryHeap" {
    const allocator = std.testing.allocator;

    // Add some numbers
    var array: std.ArrayList(i32) = .empty;
    defer array.deinit(allocator);
    try array.ensureTotalCapacity(allocator, 1100);
    for (0..1000) |i|
        array.appendAssumeCapacity(@intCast(i));

    // Ensure we have some duplicates
    var d: i32 = 0;
    while (d < 1000) : (d += 10)
        array.appendAssumeCapacity(d);

    // Shuffle the array (Fisher-Yates, std::shuffle's exact sequence is implementation defined)
    var random = fw.UnitTestRandom.init(123);
    var s = array.items.len;
    while (s > 1) : (s -= 1)
        std.mem.swap(i32, &array.items[s - 1], &array.items[random.next() % s]);

    // Add the numbers to the heap
    var heap: std.ArrayList(i32) = .empty;
    defer heap.deinit(allocator);
    for (array.items) |i| {
        try heap.append(allocator, i);
        zolt.binaryHeapPush(i32, heap.items, {}, std.sort.asc(i32));
    }

    // Check that the heap is sorted
    var last: i32 = std.math.maxInt(i32);
    var seen = [_]i32{0} ** 1000;
    while (heap.items.len > 0) {
        zolt.binaryHeapPop(i32, heap.items, {}, std.sort.asc(i32));
        const current = heap.items[heap.items.len - 1];
        const index: usize = @intCast(current);
        seen[index] += 1;
        try fw.expect(seen[index] <= (if (@mod(current, 10) == 0) @as(i32, 2) else 1));
        _ = heap.pop();
        try fw.expect(current <= last);
        last = current;
    }
}
