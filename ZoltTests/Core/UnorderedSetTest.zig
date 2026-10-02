//! Port of: UnitTests/Core/UnorderedSetTest.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const expectEqual = fw.expectEqual;

const Set = zolt.UnorderedSet(i32, .{});

/// Convert an int hash to size_t like the C++ conversion does (sign extension)
fn toSizeT(value: i32) u64 {
    return @bitCast(@as(i64, value));
}

test "TestUnorderedSet" {
    const allocator = std.testing.allocator;

    var set: Set = .empty;
    defer set.deinit(allocator);
    try expectEqual(0, set.bucketCount());
    try set.ensureTotalCapacity(allocator, 10);
    try expectEqual(16, set.bucketCount());

    // Check system limits
    try expectEqual(0x80000000, set.maxBucketCount());
    try expectEqual(@as(u64, 0x80000000) * 7 / 8, set.maxSize());

    // Insert some entries
    try expectEqual(1, (try set.insert(allocator, 1)).ptr.*);
    try expect((try set.insert(allocator, 3)).inserted);
    try expect(!(try set.insert(allocator, 3)).inserted);
    try expectEqual(2, set.count());
    try expectEqual(1, set.find(1).?.*);
    try expectEqual(3, set.find(3).?.*);
    try expect(set.find(5) == null);

    // Validate all elements are visited by a visitor
    var count: i32 = 0;
    var visited = [_]bool{false} ** 10;
    var const_it = set.constIterator();
    while (const_it.next()) |i| {
        visited[@intCast(i.*)] = true;
        count += 1;
    }
    try expectEqual(2, count);
    try expect(visited[1]);
    try expect(visited[3]);
    var it = set.iterator();
    while (it.next()) |i| {
        visited[@intCast(i.*)] = false;
        count -= 1;
    }
    try expectEqual(0, count);
    try expect(!visited[1]);
    try expect(!visited[3]);

    // Copy the set
    var set2: Set = .empty;
    defer set2.deinit(allocator);
    try set2.assign(allocator, &set);
    try expectEqual(1, set2.find(1).?.*);
    try expectEqual(3, set2.find(3).?.*);
    try expect(set2.find(5) == null);

    // Swap
    var set3: Set = .empty;
    defer set3.deinit(allocator);
    set3.swap(&set);
    try expectEqual(1, set3.find(1).?.*);
    try expectEqual(3, set3.find(3).?.*);
    try expect(set3.find(5) == null);
    try expect(set.isEmpty());

    // Move construct
    var set4 = set3.move();
    defer set4.deinit(allocator);
    try expectEqual(1, set4.find(1).?.*);
    try expectEqual(3, set4.find(3).?.*);
    try expect(set4.find(5) == null);
    try expect(set3.isEmpty());

    // Move assign
    var set5: Set = .empty;
    defer set5.deinit(allocator);
    _ = try set5.insert(allocator, 999);
    try expectEqual(999, set5.find(999).?.*);
    set5.assignMove(allocator, &set4);
    try expect(set5.find(999) == null);
    try expectEqual(1, set5.find(1).?.*);
    try expectEqual(3, set5.find(3).?.*);
    try expect(set4.isEmpty());
}

test "TestUnorderedSetGrow" {
    const allocator = std.testing.allocator;

    var set: Set = .empty;
    defer set.deinit(allocator);
    for (0..10000) |i|
        try expect((try set.insert(allocator, @intCast(i))).inserted);

    try expectEqual(10000, set.count());

    for (0..10000) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(i, set.find(i).?.*);
    }

    try expect(set.find(10001) == null);

    for (0..5000) |i|
        try expectEqual(1, set.erase(@intCast(i)));

    try expectEqual(5000, set.count());

    for (0..5000) |i|
        try expect(set.find(@intCast(i)) == null);

    for (5000..10000) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(i, set.find(i).?.*);
    }

    try expect(set.find(10001) == null);

    for (0..5000) |i|
        try expect((try set.insert(allocator, @intCast(i))).inserted);

    try expect(!(try set.insert(allocator, 0)).inserted);

    try expectEqual(10000, set.count());

    for (0..10000) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(i, set.find(i).?.*);
    }

    try expect(set.find(10001) == null);
}

test "TestUnorderedSetHashCollision" {
    const allocator = std.testing.allocator;

    // A hash function that's guaranteed to collide
    const MyBadHash = struct {
        fn hash(value: i32) u64 {
            _ = value;
            return 0;
        }
    };

    var set: zolt.UnorderedSet(i32, .{ .hash = MyBadHash.hash }) = .empty;
    defer set.deinit(allocator);
    for (0..10) |i|
        try expect((try set.insert(allocator, @intCast(i))).inserted);

    try expectEqual(10, set.count());

    for (0..10) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(i, set.find(i).?.*);
    }

    try expect(set.find(11) == null);

    for (0..5) |i|
        try expectEqual(1, set.erase(@intCast(i)));

    try expectEqual(5, set.count());

    for (0..5) |i|
        try expect(set.find(@intCast(i)) == null);

    for (5..10) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(i, set.find(i).?.*);
    }

    try expect(set.find(11) == null);

    for (0..5) |i|
        try expect((try set.insert(allocator, @intCast(i))).inserted);

    try expect(!(try set.insert(allocator, 0)).inserted);

    try expectEqual(10, set.count());

    for (0..10) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(i, set.find(i).?.*);
    }

    try expect(set.find(11) == null);
}

test "TestUnorderedSetAddRemoveCyles" {
    const allocator = std.testing.allocator;

    var set: Set = .empty;
    defer set.deinit(allocator);
    const bucket_count = 64;
    try set.ensureTotalCapacity(allocator, @intFromFloat(set.maxLoadFactor() * bucket_count));
    try expectEqual(bucket_count, set.bucketCount());

    // Repeatedly add and remove elements to see if the set cleans up tombstones
    const num_elements = 64 * 6 / 8; // We make sure that the map is max 6/8 full to ensure that we never grow the map but rehash it instead
    var add_counter: i32 = 0;
    var remove_counter: i32 = 0;
    for (0..100) |_| {
        for (0..num_elements) |_| {
            try expect(set.find(add_counter) == null);
            try expect((try set.insert(allocator, add_counter)).inserted);
            try expect(set.find(add_counter) != null);
            add_counter += 1;
        }

        try expectEqual(num_elements, set.count());

        for (0..num_elements) |_| {
            try expect(set.find(remove_counter) != null);
            try expectEqual(1, set.erase(remove_counter));
            try expectEqual(0, set.erase(remove_counter));
            try expect(set.find(remove_counter) == null);
            remove_counter += 1;
        }

        try expectEqual(0, set.count());
        try expect(set.isEmpty());
    }

    // Test that adding and removing didn't resize the set
    try expectEqual(bucket_count, set.bucketCount());
}

test "TestUnorderedSetManyTombStones" {
    const allocator = std.testing.allocator;

    // A hash function that makes sure that consecutive ints end up in consecutive buckets starting at bucket 63
    const MyBadHash = struct {
        fn hash(value: i32) u64 {
            return toSizeT((value + 63) << 7);
        }
    };

    var set: zolt.UnorderedSet(i32, .{ .hash = MyBadHash.hash }) = .empty;
    defer set.deinit(allocator);
    const bucket_count = 64;
    try set.ensureTotalCapacity(allocator, @intFromFloat(set.maxLoadFactor() * bucket_count));
    try expectEqual(bucket_count, set.bucketCount());

    // Fill 32 buckets
    var add_counter: i32 = 0;
    for (0..32) |_| {
        try expect((try set.insert(allocator, add_counter)).inserted);
        add_counter += 1;
    }

    // Since we control the hash, we know in which order we'll visit the elements
    // The first element was inserted in bucket 63, so we start at 1
    var expected: i32 = 1;
    var it = set.constIterator();
    while (it.next()) |i| {
        try expectEqual(expected, i.*);
        expected = (expected + 1) & 31;
    }
    expected = 1;
    it = set.constIterator();
    while (it.next()) |i| {
        try expectEqual(expected, i.*);
        expected = (expected + 1) & 31;
    }

    // Remove a bucket in the middle with so that the number of occupied slots
    // surrounding the bucket exceed 16 to force creating a tombstone,
    // then add one at the end
    var remove_counter: i32 = 16;
    for (0..100) |_| {
        try expect(set.find(remove_counter) != null);
        try expectEqual(1, set.erase(remove_counter));
        try expect(set.find(remove_counter) == null);

        try expect(set.find(add_counter) == null);
        try expect((try set.insert(allocator, add_counter)).inserted);
        try expect(set.find(add_counter) != null);

        add_counter += 1;
        remove_counter += 1;
    }

    // Check that the elements we inserted are still there
    try expectEqual(32, set.count());
    for (0..16) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(i, set.find(i).?.*);
    }
    for (0..16) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(add_counter - 1 - i, set.find(add_counter - 1 - i).?.*);
    }

    // Test that adding and removing didn't resize the set
    try expectEqual(bucket_count, set.bucketCount());
}

var reversed_hash = false;

test "TestUnorderedSetRehash" {
    const allocator = std.testing.allocator;

    // A hash function for which we can switch the hashing algorithm
    const MyBadHash = struct {
        fn hash(value: i32) u64 {
            return toSizeT((if (reversed_hash) 127 - value else value) << 7);
        }
    };

    const RehashSet = zolt.UnorderedSet(i32, .{ .hash = MyBadHash.hash });
    var set: RehashSet = .empty;
    defer set.deinit(allocator);
    const bucket_count = 128;
    try set.ensureTotalCapacity(allocator, @intFromFloat(set.maxLoadFactor() * bucket_count));
    try expectEqual(bucket_count, set.bucketCount());

    // Fill buckets
    reversed_hash = false;
    const num_elements = 96;
    for (0..num_elements) |i|
        try expect((try set.insert(allocator, @intCast(i))).inserted);

    // Check that we get the elements in the expected order
    var expected: i32 = 0;
    var it = set.constIterator();
    while (it.next()) |i| {
        try expectEqual(expected, i.*);
        expected += 1;
    }

    // Change the hashing algorithm so that a rehash is forced to move elements.
    // The test is designed in such a way that it will both need to move elements to empty slots
    // and to move elements to slots that currently already have another element.
    reversed_hash = true;
    defer reversed_hash = false;
    set.rehash(0);

    // Check that all elements are still there
    for (0..num_elements) |n| {
        const i: i32 = @intCast(n);
        try expectEqual(i, set.find(i).?.*);
    }

    // The hash went from filling buckets 0 .. 95 with values 0 .. 95 to bucket 127 .. 31 with values 0 .. 95
    // However, we don't move elements if they still fall within the same batch, this means that the first 8
    // elements didn't move
    it = set.constIterator();
    for (0..8) |n|
        try expectEqual(@as(i32, @intCast(n)), it.next().?.*);

    // The rest will have been reversed
    var i: i32 = 95;
    while (i > 7) : (i -= 1)
        try expectEqual(i, it.next().?.*);

    // Test that adding and removing didn't resize the set
    try expectEqual(bucket_count, set.bucketCount());
}
