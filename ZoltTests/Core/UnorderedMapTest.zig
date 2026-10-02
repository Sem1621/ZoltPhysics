//! Port of: UnitTests/Core/UnorderedMapTest.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const expectEqual = fw.expectEqual;

const Map = zolt.UnorderedMap(i32, i32, .{});

test "TestUnorderedMap" {
    const allocator = std.testing.allocator;

    var map: Map = .empty;
    defer map.deinit(allocator);
    try map.ensureTotalCapacity(allocator, 10);

    // Insert some entries
    try expect((try map.insert(allocator, .{ .key = 1, .value = 2 })).ptr.key == 1);
    try expect((try map.insert(allocator, .{ .key = 3, .value = 4 })).inserted);
    try expect(!(try map.insert(allocator, .{ .key = 3, .value = 5 })).inserted);
    try expectEqual(2, map.count());
    try expectEqual(2, map.find(1).?.value);
    try expectEqual(4, map.find(3).?.value);
    try expect(map.find(5) == null);

    // Use operator []
    (try map.getOrPutValue(allocator, 5, 0)).* = 6;
    try expectEqual(3, map.count());
    try expectEqual(6, map.find(5).?.value);
    (try map.getOrPutValue(allocator, 5, 0)).* = 7;
    try expectEqual(3, map.count());
    try expectEqual(7, map.find(5).?.value);

    // Validate all elements are visited by a visitor
    var count: i32 = 0;
    var visited = [_]bool{false} ** 10;
    var const_it = map.constIterator();
    while (const_it.next()) |i| {
        visited[@intCast(i.key)] = true;
        count += 1;
    }
    try expectEqual(3, count);
    try expect(visited[1]);
    try expect(visited[3]);
    try expect(visited[5]);
    var it = map.iterator();
    while (it.next()) |i| {
        visited[@intCast(i.key)] = false;
        count -= 1;
    }
    try expectEqual(0, count);
    try expect(!visited[1]);
    try expect(!visited[3]);
    try expect(!visited[5]);

    // Copy the map
    var map2: Map = .empty;
    defer map2.deinit(allocator);
    try map2.assign(allocator, &map);
    try expectEqual(2, map2.find(1).?.value);
    try expectEqual(4, map2.find(3).?.value);
    try expectEqual(7, map2.find(5).?.value);
    try expect(map2.find(7) == null);

    // Try emplace
    _ = try map.tryEmplace(allocator, 7, 8);
    try expectEqual(4, map.count());
    try expectEqual(8, map.find(7).?.value);

    // Swap
    var map3: Map = .empty;
    defer map3.deinit(allocator);
    map3.swap(&map);
    try expectEqual(2, map3.find(1).?.value);
    try expectEqual(4, map3.find(3).?.value);
    try expectEqual(7, map3.find(5).?.value);
    try expectEqual(8, map3.find(7).?.value);
    try expect(map3.find(9) == null);
    try expect(map.isEmpty());

    // Move construct
    var map4 = map3.move();
    defer map4.deinit(allocator);
    try expectEqual(2, map4.find(1).?.value);
    try expectEqual(4, map4.find(3).?.value);
    try expectEqual(7, map4.find(5).?.value);
    try expectEqual(8, map4.find(7).?.value);
    try expect(map4.find(9) == null);
    try expect(map3.isEmpty());
}

test "TestUnorderedMapGrow" {
    const allocator = std.testing.allocator;

    var map: Map = .empty;
    defer map.deinit(allocator);
    for (0..10000) |n| {
        const i: i32 = @intCast(n);
        try expect((try map.tryEmplace(allocator, i, ~i)).inserted);
    }

    try expectEqual(10000, map.count());

    for (0..10000) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(~i, map.find(i).?.value);
    }

    try expect(map.find(10001) == null);

    for (0..5000) |n|
        try expectEqual(1, map.erase(@intCast(n)));

    try expectEqual(5000, map.count());

    for (0..5000) |n|
        try expect(map.find(@intCast(n)) == null);

    for (5000..10000) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(~i, map.find(i).?.value);
    }

    try expect(map.find(10001) == null);

    for (0..5000) |n| {
        const i: i32 = @intCast(n);
        try expect((try map.tryEmplace(allocator, i, i + 1)).inserted);
    }

    try expect(!(try map.tryEmplace(allocator, 0, 0)).inserted);

    try expectEqual(10000, map.count());

    for (0..5000) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(i + 1, map.find(i).?.value);
    }

    for (5000..10000) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(~i, map.find(i).?.value);
    }

    try expect(map.find(10001) == null);
}
