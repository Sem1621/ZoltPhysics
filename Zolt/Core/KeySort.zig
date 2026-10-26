//! Zolt addition, no Jolt file: sort records by a 64-bit key in about linear time, for sorts whose result is unique
//! Status: complete
//!
//! Phase 12 (class A, Docs/Zolt/Roadmap.md): PhysicsSystem sorts the constraints and the contacts of a large island
//! with this instead of Jolt's QuickSort (ConstraintManager.sortConstraintsWithRecords,
//! ContactConstraintManager.constraintIdxToConstraintOffsetAndSort). The result is the same bit for bit, for this
//! reason:
//! - `sortByDistinctKeys` returns true only when it has checked that the keys of the sorted records are strictly
//!   increasing. The records are a permutation of the input (the sort only swaps and moves records), so the keys of
//!   the input were pairwise distinct and the result is the one order of the input with increasing keys.
//! - The callers build the key so that Jolt's comparator, on the elements of the input, compares the keys whenever
//!   they differ. With pairwise distinct keys, Jolt's comparator is then the strict total order `lhs.key < rhs.key` on
//!   these elements, and every correct comparison sort, Jolt's QuickSort too, puts them in that one increasing order.
//! - When it returns false (two equal keys: ties, where Jolt's comparator looks further or where Jolt's QuickSort,
//!   which is not stable, decides the order), the caller sorts the untouched input with Jolt's QuickSort instead.
//!
//! The algorithm: one pass of an MSD radix sort on the top bits of `key - min_key` into about n / 2 buckets (an in
//! place permutation, "American flag sort"), then an insertion sort of each bucket, or Jolt's QuickSort for a bucket of
//! more than 16 records. Keys that are hashes spread evenly over the buckets; dense keys (a priority and an index) too,
//! because the buckets cover the range of the keys, not all 64 bits. Clustered keys only make some buckets larger, and
//! those take QuickSort: no quadratic case. The checks: inside a bucket the insertion sort sees every equal pair (the
//! later one stops at the earlier one), QuickSort's buckets are checked after the sort, and every bucket's first key
//! must be larger than the last key before it.
//!
//! Memory: the records and `bucketMemory(n)` u32 for the buckets, scratch of the caller. Nothing allocates.

const std = @import("std");
const Core = @import("Core.zig");
const quickSort = @import("QuickSort.zig").quickSort;

/// Buckets with more records than this are sorted with QuickSort instead of an insertion sort
const max_insertion_sort = 16;

/// The number of buckets for `num_records` records (a power of two, about num_records / 2)
pub fn numBuckets(num_records: usize) u32 {
    if (num_records < 8)
        return 1;
    const bits: u5 = @intCast(@min(std.math.log2_int(usize, num_records) - 1, 20));
    return @as(u32, 1) << bits;
}

/// The number of u32 that `sortByDistinctKeys` needs in `buckets` for `num_records` records
pub fn bucketMemory(num_records: usize) usize {
    const num_buckets = numBuckets(num_records);
    return if (num_buckets > 1) 2 * @as(usize, num_buckets) else 0;
}

/// A range of keys: `min <= key <= max` for every record
pub const KeyRange = struct {
    min: u64,
    max: u64,

    /// Every u64 (for keys that are hashes: their top bits spread them over the buckets)
    pub const all: KeyRange = .{ .min = 0, .max = std.math.maxInt(u64) };

    /// The empty range, to start `add`
    pub const empty: KeyRange = .{ .min = std.math.maxInt(u64), .max = 0 };

    /// Grow the range to hold `key`
    pub fn add(self: *KeyRange, key: u64) void {
        self.min = @min(self.min, key);
        self.max = @max(self.max, key);
    }
};

/// Sort `records` (a type with a field `key: u64`) by increasing key. `buckets` must hold `bucketMemory(records.len)`
/// u32, and every key must lie in `range` (`KeyRange.all` holds every key; a range that misses a key is a bug of the
/// caller: then a bucket index can be out of bounds). Returns true when the keys are pairwise distinct; then the
/// records are in the one order with increasing keys. Returns false when two keys are equal; then the records are in
/// some order (a permutation of the input).
pub fn sortByDistinctKeys(comptime Record: type, records: []Record, buckets: []u32, range: KeyRange) bool {
    const n = records.len;
    if (n < 2)
        return true;
    std.debug.assert(n <= std.math.maxInt(u32));
    if (Core.enable_asserts) {
        for (records) |r|
            std.debug.assert(r.key >= range.min and r.key <= range.max);
    }

    const num_buckets = numBuckets(n);
    if (num_buckets == 1)
        return sortBucket(Record, records);

    const min_key = range.min;
    const max_key = range.max;
    if (min_key == max_key)
        return false; // All keys equal

    // The bucket of a key is the top `bits` bits of `key - min_key` (in [0, 2^bits)), which does not decrease with the
    // key: the buckets are in key order
    std.debug.assert(buckets.len >= 2 * @as(usize, num_buckets));
    const range_bits: u32 = 64 - @clz(max_key - min_key);
    const bits: u32 = std.math.log2_int(u32, num_buckets);
    const shift: u6 = @intCast(if (range_bits > bits) range_bits - bits else 0);

    // Count the records per bucket (`ends`), then turn the counts into the end of each bucket and its start (`heads`)
    const heads = buckets[0..num_buckets];
    const ends = buckets[num_buckets..][0..num_buckets];
    @memset(ends, 0);
    for (records) |r|
        ends[@intCast((r.key - min_key) >> shift)] += 1;
    var sum: u32 = 0;
    for (heads, ends) |*head, *end| {
        head.* = sum;
        sum += end.*;
        end.* = sum;
    }

    // Move every record into its bucket: take the first record of a bucket that is not in place yet and swap it into
    // the first free place of its bucket until a record of this bucket comes back
    for (heads, ends, 0..) |*head, end, b| {
        while (head.* < end) {
            var r = records[head.*];
            var rb: usize = @intCast((r.key - min_key) >> shift);
            while (rb != b) {
                const dst = heads[rb];
                heads[rb] = dst + 1;
                Core.swap(Record, &r, &records[dst]);
                rb = @intCast((r.key - min_key) >> shift);
            }
            records[head.*] = r;
            head.* += 1;
        }
    }

    // Sort each bucket (bucket b is [ends[b - 1], ends[b]), its keys are in no order yet, there are about 2) and check
    // that its first key is larger than the last key of the bucket before it
    var start: u32 = 0;
    for (ends) |end| {
        if (end > start) {
            if (!sortBucket(Record, records[start..end]))
                return false;
            if (start > 0 and records[start - 1].key >= records[start].key)
                return false;
        }
        start = end;
    }
    return true;
}

/// Sort the records of a bucket by increasing key, false when two keys are equal
fn sortBucket(comptime Record: type, records: []Record) bool {
    if (records.len > max_insertion_sort) {
        quickSort(Record, records, {}, struct {
            fn lessThan(_: void, lhs: Record, rhs: Record) bool {
                return lhs.key < rhs.key;
            }
        }.lessThan);
        for (records[1..], 0..) |r, i| {
            if (records[i].key >= r.key)
                return false;
        }
        return true;
    }

    // Insertion sort: the records before i are sorted. A record stops behind the last record with a key that is not
    // larger than its own, so when the sorted part holds an equal key, it stops right behind an equal key.
    var i: usize = 1;
    while (i < records.len) : (i += 1) {
        const r = records[i];
        var j = i;
        while (j > 0 and records[j - 1].key > r.key) : (j -= 1)
            records[j] = records[j - 1];
        const duplicate = j > 0 and records[j - 1].key == r.key;
        records[j] = r;
        if (duplicate)
            return false;
    }
    return true;
}

const TestRecord = struct {
    key: u64,
    /// The position in the input (unique)
    value: u32,
};

/// The range of the keys of `records`
fn rangeOf(records: []const TestRecord) KeyRange {
    var range: KeyRange = .empty;
    for (records) |r| range.add(r.key);
    return range;
}

/// `records` is a permutation of `input`
fn expectPermutation(records: []const TestRecord, input: []const TestRecord, seen: []bool) !void {
    try std.testing.expectEqual(input.len, records.len);
    @memset(seen[0..input.len], false);
    for (records) |r| {
        try std.testing.expect(r.value < input.len and !seen[r.value]);
        seen[r.value] = true;
        try std.testing.expectEqual(input[r.value].key, r.key);
    }
}

/// `records` is a permutation of `input` with strictly increasing keys
fn expectSortedPermutation(records: []const TestRecord, input: []const TestRecord, seen: []bool) !void {
    try std.testing.expectEqual(input.len, records.len);
    @memset(seen[0..input.len], false);
    for (records, 0..) |r, i| {
        try std.testing.expect(r.value < input.len and !seen[r.value]);
        seen[r.value] = true;
        try std.testing.expectEqual(input[r.value].key, r.key);
        if (i > 0) try std.testing.expect(records[i - 1].key < r.key);
    }
}

test "KeySort: distinct keys give the increasing order, equal keys return false" {
    var prng = std.Random.DefaultPrng.init(0x5eed);
    const random = prng.random();
    var input: [3000]TestRecord = undefined;
    var records: [3000]TestRecord = undefined;
    var seen: [3000]bool = undefined;
    var buckets: [2 * 1024]u32 = undefined;

    // Hash like keys, dense keys (a priority and an index), an arithmetic sequence, single bits plus an index
    const sizes = [_]usize{ 0, 1, 2, 3, 7, 8, 9, 31, 32, 33, 100, 1000, 3000 };
    for (0..4) |kind| {
        for (sizes) |n| {
            for (input[0..n], 0..) |*r, i| {
                r.key = switch (kind) {
                    0 => random.int(u64),
                    1 => (@as(u64, random.uintLessThan(u32, 3)) << 32) | @as(u64, @intCast(i)),
                    2 => 1000 + @as(u64, @intCast(i)) * 3,
                    else => (@as(u64, 1) << @intCast(i % 38)) | (@as(u64, @intCast(i)) << 38),
                };
            }
            random.shuffle(TestRecord, input[0..n]);
            for (input[0..n], 0..) |*r, i| r.value = @intCast(i);
            try std.testing.expect(bucketMemory(n) <= buckets.len);

            // With the range of the keys and with the range of all u64
            for ([_]KeyRange{ rangeOf(input[0..n]), .all }) |range| {
                @memcpy(records[0..n], input[0..n]);
                try std.testing.expect(sortByDistinctKeys(TestRecord, records[0..n], &buckets, range));
                try expectSortedPermutation(records[0..n], input[0..n], &seen);
            }
        }
    }

    // Two equal keys among distinct ones, in any position: false, and the records are still a permutation
    for ([_]usize{ 2, 3, 33, 500, 3000 }) |n| {
        for (0..20) |_| {
            for (input[0..n], 0..) |*r, i| r.* = .{ .key = random.int(u64), .value = @intCast(i) };
            const a = random.uintLessThan(usize, n);
            var b = random.uintLessThan(usize, n);
            if (b == a) b = (a + 1) % n;
            input[a].key = input[b].key;
            for ([_]KeyRange{ rangeOf(input[0..n]), .all }) |range| {
                @memcpy(records[0..n], input[0..n]);
                try std.testing.expect(!sortByDistinctKeys(TestRecord, records[0..n], &buckets, range));
                try expectPermutation(records[0..n], input[0..n], &seen);
            }
        }
    }

    // All keys equal
    for ([_]usize{ 2, 7, 10, 100 }) |n| {
        for (input[0..n], 0..) |*r, i| r.* = .{ .key = 42, .value = @intCast(i) };
        for ([_]KeyRange{ rangeOf(input[0..n]), .all }) |range| {
            @memcpy(records[0..n], input[0..n]);
            try std.testing.expect(!sortByDistinctKeys(TestRecord, records[0..n], &buckets, range));
            try expectPermutation(records[0..n], input[0..n], &seen);
        }
    }
}

test "KeySort: clustered keys (buckets of more than 16 records take QuickSort), with and without duplicates" {
    var prng = std.Random.DefaultPrng.init(0xc105);
    const random = prng.random();
    var input: [3000]TestRecord = undefined;
    var records: [3000]TestRecord = undefined;
    var seen: [3000]bool = undefined;
    var buckets: [2 * 1024]u32 = undefined;
    for ([_]usize{ 40, 300, 3000 }) |n| {
        for (0..10) |round| {
            // A few clusters of consecutive keys far apart: each cluster falls into one or two buckets
            for (input[0..n], 0..) |*r, i|
                r.key = (@as(u64, random.uintLessThan(u32, 4)) << 50) + @as(u64, @intCast(i));
            random.shuffle(TestRecord, input[0..n]);
            for (input[0..n], 0..) |*r, i| r.value = @intCast(i);
            for ([_]KeyRange{ rangeOf(input[0..n]), .all }) |range| {
                @memcpy(records[0..n], input[0..n]);
                try std.testing.expect(sortByDistinctKeys(TestRecord, records[0..n], &buckets, range));
                try expectSortedPermutation(records[0..n], input[0..n], &seen);
            }

            // The same with one key that equals another one of its cluster (the duplicate is inside a large bucket)
            const a = random.uintLessThan(usize, n);
            var b = a;
            for (input[0..n], 0..) |r, i| {
                if (i != a and r.key >> 50 == input[a].key >> 50 and (b == a or round % 2 == 0)) b = i;
            }
            if (b == a) continue;
            input[b].key = input[a].key;
            for ([_]KeyRange{ rangeOf(input[0..n]), .all }) |range| {
                @memcpy(records[0..n], input[0..n]);
                try std.testing.expect(!sortByDistinctKeys(TestRecord, records[0..n], &buckets, range));
                try expectPermutation(records[0..n], input[0..n], &seen);
            }
        }
    }
}

test "KeySort: skewed keys (one bucket gets almost everything) still sort" {
    var prng = std.Random.DefaultPrng.init(7);
    const random = prng.random();
    var input: [2000]TestRecord = undefined;
    var records: [2000]TestRecord = undefined;
    var seen: [2000]bool = undefined;
    var buckets: [2 * 512]u32 = undefined;
    for (&input, 0..) |*r, i| r.key = @as(u64, @intCast(i)) * 7 + 1;
    input[0].key = std.math.maxInt(u64); // The range is 64 bits, all other keys fall into bucket 0
    random.shuffle(TestRecord, &input);
    for (&input, 0..) |*r, i| r.value = @intCast(i);
    for ([_]KeyRange{ rangeOf(&input), .all }) |range| {
        records = input;
        try std.testing.expect(sortByDistinctKeys(TestRecord, &records, &buckets, range));
        try expectSortedPermutation(&records, &input, &seen);
    }
}
