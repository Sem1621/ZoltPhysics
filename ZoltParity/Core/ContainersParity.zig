//! Parity tests for Jolt/Core/HashTable.h, UnorderedMap.h and UnorderedSet.h: run identical operation scripts on the
//! C++ containers and on the Zolt containers and require identical results after every operation, including the
//! bucket index of every element, the bucket count and the full iteration order.
//! C ABI wrappers: ZoltParity/Core/ContainersReference.cpp. See ZoltParity/parity.zig for how parity tests work.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Allocator = std.mem.Allocator;

/// The C++ reference functions, see ContainersReference.cpp
const jolt = struct {
    extern fn jolt_unordered_set_u32_run(ops: [*]const u8, keys: [*]const u64, values: [*]const u64, num_ops: u32, out: [*]u64, max_out: u32) u32;
    extern fn jolt_unordered_set_u64_run(ops: [*]const u8, keys: [*]const u64, values: [*]const u64, num_ops: u32, out: [*]u64, max_out: u32) u32;
    extern fn jolt_unordered_set_u32_clustered_run(ops: [*]const u8, keys: [*]const u64, values: [*]const u64, num_ops: u32, out: [*]u64, max_out: u32) u32;
    extern fn jolt_unordered_map_u32_run(ops: [*]const u8, keys: [*]const u64, values: [*]const u64, num_ops: u32, out: [*]u64, max_out: u32) u32;
    extern fn jolt_unordered_map_u64_run(ops: [*]const u8, keys: [*]const u64, values: [*]const u64, num_ops: u32, out: [*]u64, max_out: u32) u32;
    extern fn jolt_unordered_map_u32_clustered_run(ops: [*]const u8, keys: [*]const u64, values: [*]const u64, num_ops: u32, out: [*]u64, max_out: u32) u32;
};

const RunFn = *const fn (ops: [*]const u8, keys: [*]const u64, values: [*]const u64, num_ops: u32, out: [*]u64, max_out: u32) callconv(.c) u32;

/// Op codes, keep in sync with ContainersReference.cpp
const Op = enum(u8) {
    insert, // insert(key [, value]): inserted, index [, value at index]
    index, // map[key] (set: insert): old value (Value() when new), then assigns value
    try_emplace, // try_emplace(key, value) (set: insert): inserted, index [, value at index]
    find, // find(key): not_found or index [, value]
    erase_key, // erase(key): number of erased elements
    erase_iterator, // erase(find(key)): not_found or index of the erased element
    clear, // clear()
    clear_and_keep_memory, // ClearAndKeepMemory()
    reserve, // reserve(uint32(key))
    rehash, // rehash(0)
    copy_snapshot, // snapshot of a copy of the container (the copy is discarded)
    snapshot, // size, bucket_count, empty, then index, key [, value] per element in iteration order, then snapshot_end
    copy_replace, // replace the container by a copy of itself (copy constructor + swap)
    erase_first, // erase(begin()): not_found (empty iteration) or index of the erased element
};

const not_found: u64 = ~@as(u64, 0);
const snapshot_end: u64 = ~@as(u64, 1);

/// A hash function that puts 4 consecutive keys in the same bucket and only uses 4 different control values
fn clusteredHash(value: u32) u64 {
    return (@as(u64, value >> 2) << 7) | (value & 3);
}

/// With dense keys the clustered hash fills one long run of buckets, which makes every probe linear in the
/// number of elements. One seed is enough to cover that and keeps the parity step fast.
const clustered_num_seeds = 1;

/// An operation script: op codes, keys and values
const Script = struct {
    ops: std.ArrayList(u8) = .empty,
    keys: std.ArrayList(u64) = .empty,
    values: std.ArrayList(u64) = .empty,

    fn deinit(self: *Script, allocator: Allocator) void {
        self.ops.deinit(allocator);
        self.keys.deinit(allocator);
        self.values.deinit(allocator);
    }

    fn add(self: *Script, allocator: Allocator, op: Op, key: u64, value: u64) !void {
        try self.ops.append(allocator, @intFromEnum(op));
        try self.keys.append(allocator, key);
        try self.values.append(allocator, value);
    }

    fn len(self: *const Script) u32 {
        return @intCast(self.ops.items.len);
    }
};

/// Generates operation scripts that grow the containers through many sizes, create tombstones, trigger the in place
/// rehash and exercise all operations
const ScriptGenerator = struct {
    allocator: Allocator,
    script: *Script,
    rng: fw.Rng,

    /// Keys are 32 bit when the containers have uint32 keys (the C++ wrapper truncates, so keep them in range)
    wide_keys: bool,

    /// Map key indices to keys: either the index itself (dense ids) or a multiplicative hash of it (sparse keys)
    sparse_keys: bool,

    fn key(self: *const ScriptGenerator, index: u64) u64 {
        if (!self.sparse_keys)
            return index;
        const k = (index +% 1) *% 0x9e3779b97f4a7c15;
        return if (self.wide_keys) k else k >> 32;
    }

    fn value(self: *ScriptGenerator) u64 {
        const v = @as(u64, self.rng.next()) << 32 | self.rng.next();
        return if (self.wide_keys) v else v & 0xffffffff;
    }

    fn add(self: *ScriptGenerator, op: Op, key_index: u64) !void {
        try self.script.add(self.allocator, op, self.key(key_index), self.value());
    }

    fn randomIndex(self: *ScriptGenerator, max: u64) u64 {
        return self.rng.intRange(u64, 0, max - 1);
    }

    /// One random insert style operation
    fn addInsert(self: *ScriptGenerator, key_index: u64) !void {
        const op: Op = switch (self.rng.next() % 3) {
            0 => .insert,
            1 => .index,
            else => .try_emplace,
        };
        try self.add(op, key_index);
    }

    /// One random operation on keys in [0, key_range)
    fn addRandom(self: *ScriptGenerator, key_range: u64) !void {
        const r = self.rng.next() % 1000;
        const key_index = self.randomIndex(key_range);
        if (r < 350) {
            try self.addInsert(key_index);
        } else if (r < 650) {
            try self.add(.find, key_index);
        } else if (r < 850) {
            try self.add(.erase_key, key_index);
        } else if (r < 995) {
            try self.add(.erase_iterator, key_index);
        } else if (r < 998) {
            try self.add(.rehash, 0);
        } else {
            try self.add(.copy_snapshot, 0);
        }
    }

    fn reserve(self: *ScriptGenerator, count: u64) !void {
        try self.script.add(self.allocator, .reserve, count, 0);
    }

    fn generate(self: *ScriptGenerator) !void {
        // Range of key indices for the large phases
        const key_range = 24_000;

        // Growth through many sizes (up to 16k buckets), with duplicates and finds
        for (0..15_000) |i| {
            const r = self.rng.next() % 4;
            if (r == 0)
                try self.add(.find, self.randomIndex(key_range))
            else
                try self.addInsert(self.randomIndex(key_range));
            if (i % 2500 == 2499)
                try self.add(.snapshot, 0);
        }
        try self.add(.copy_snapshot, 0);

        // Random operations at a stable size
        for (0..24_000) |i| {
            try self.addRandom(key_range);
            if (i % 6000 == 5999)
                try self.add(.snapshot, 0);
        }

        // Erase most elements, this leaves tombstones behind
        for (0..24_000) |_|
            try self.add(if (self.rng.next() % 2 == 0) .erase_key else .erase_iterator, self.randomIndex(key_range));
        try self.add(.snapshot, 0);
        try self.add(.rehash, 0);
        try self.add(.snapshot, 0);

        // Clear and reserve
        try self.add(.clear, 0);
        try self.add(.snapshot, 0);
        try self.reserve(self.randomIndex(5000));
        try self.add(.snapshot, 0);
        for (0..3000) |_|
            try self.addInsert(self.randomIndex(10_000));
        try self.add(.snapshot, 0);
        try self.add(.clear_and_keep_memory, 0);
        try self.add(.snapshot, 0);
        try self.add(.clear_and_keep_memory, 0); // Nothing to reset this time
        for (0..2000) |_|
            try self.addRandom(3000);
        try self.add(.snapshot, 0);
        try self.reserve(20_000); // Grow while there are elements and tombstones
        try self.add(.snapshot, 0);
        try self.reserve(100); // Reserving less than the current size does nothing
        try self.add(.snapshot, 0);

        // Add / remove cycles in a small table: tombstones are cleaned up by an in place rehash instead of growing
        try self.add(.clear, 0);
        try self.reserve(56);
        var add_counter: u64 = 100_000;
        var remove_counter: u64 = add_counter;
        for (0..60) |_| {
            for (0..48) |_| {
                try self.add(.insert, add_counter);
                add_counter += 1;
            }
            try self.add(.snapshot, 0);
            for (0..48) |_| {
                try self.add(.erase_key, remove_counter);
                try self.add(.find, remove_counter);
                remove_counter += 1;
            }
        }
        try self.add(.snapshot, 0);

        // Sliding window at high load: erase the oldest element, insert a new one
        try self.add(.clear, 0);
        try self.reserve(200);
        add_counter = 200_000;
        remove_counter = add_counter;
        for (0..200) |_| {
            try self.add(.insert, add_counter);
            add_counter += 1;
        }
        for (0..20_000) |i| {
            try self.add(.erase_key, remove_counter);
            remove_counter += 1;
            try self.add(.insert, add_counter);
            add_counter += 1;
            if (i % 1000 == 999)
                try self.add(.snapshot, 0);
        }

        // Explicit rehash with many tombstones
        for (0..150) |_| {
            try self.add(.erase_iterator, remove_counter);
            remove_counter += 1;
        }
        try self.add(.snapshot, 0);
        try self.add(.rehash, 0);
        try self.add(.snapshot, 0);

        // A copy does not copy the load left (it is reset to the max load of the table), so the copy can take more
        // elements before it grows. Keep the table small enough that it never gets completely full.
        try self.add(.clear, 0);
        try self.reserve(56);
        for (0..4) |_| {
            try self.add(.insert, add_counter);
            add_counter += 1;
        }
        try self.add(.copy_replace, 0);
        try self.add(.snapshot, 0);
        for (0..58) |i| {
            // The first 56 inserts use up the load left, the 57th triggers an in place rehash because
            // `max load - size` wraps around, which also makes load left wrap around
            try self.add(.insert, add_counter);
            add_counter += 1;
            if (i % 4 == 3 or i >= 55)
                try self.add(.snapshot, 0);
        }

        // Copy of an empty table that has buckets: the copy has no buckets (don't rehash it, Jolt would crash)
        try self.add(.clear_and_keep_memory, 0);
        try self.add(.snapshot, 0);
        try self.add(.copy_replace, 0);
        try self.add(.snapshot, 0);
        for (0..100) |_|
            try self.addInsert(self.randomIndex(1000));
        try self.add(.snapshot, 0);

        // ClearAndKeepMemory on a copy: the load left of the copy is the max load, so the control bytes are not reset
        // and iterating still visits the old elements while the size is 0. Erasing one of them makes the size wrap
        // around (find doesn't see them because the table is empty, so erase the first element in iteration order).
        try self.add(.clear, 0);
        try self.reserve(56);
        const first_stale = add_counter;
        for (0..4) |_| {
            try self.add(.insert, add_counter);
            add_counter += 1;
        }
        try self.add(.copy_replace, 0);
        try self.add(.clear_and_keep_memory, 0);
        try self.add(.snapshot, 0);
        try self.add(.find, first_stale);
        try self.add(.erase_first, 0);
        try self.add(.snapshot, 0);
        for (first_stale..add_counter) |stale|
            try self.add(.find, stale); // The size is no longer 0: finds the remaining old elements
        try self.add(.insert, first_stale + 1); // Wraps the size back to 0 when this is a new element
        try self.add(.snapshot, 0);
        try self.add(.insert, add_counter);
        add_counter += 1;
        try self.add(.snapshot, 0);
        for (0..3) |_|
            try self.add(.erase_first, 0);
        try self.add(.snapshot, 0);
        try self.add(.clear, 0);
        try self.add(.erase_first, 0); // No buckets: nothing to erase
        try self.add(.snapshot, 0);
    }
};

/// Run a script on a Zolt container, mirrors RunScript in ContainersReference.cpp
fn runZolt(comptime Container: type, comptime Key: type, comptime Value: type, comptime is_map: bool, allocator: Allocator, script: *const Script, out: *std.ArrayList(u64)) !void {
    var container: Container = .empty;
    defer container.deinit(allocator);

    for (script.ops.items, script.keys.items, script.values.items) |op_code, key_u64, value_u64| {
        const key: Key = @truncate(key_u64);
        const value: Value = @truncate(value_u64);

        var op: Op = @enumFromInt(op_code);
        if (!is_map and (op == .index or op == .try_emplace))
            op = .insert;

        switch (op) {
            .insert, .try_emplace => {
                if (is_map) {
                    const result = if (op == .insert)
                        try container.insert(allocator, .{ .key = key, .value = value })
                    else
                        try container.tryEmplace(allocator, key, value);
                    try out.append(allocator, @intFromBool(result.inserted));
                    try out.append(allocator, container.indexOf(result.ptr));
                    try out.append(allocator, result.ptr.value);
                } else {
                    const result = try container.insert(allocator, key);
                    try out.append(allocator, @intFromBool(result.inserted));
                    try out.append(allocator, container.indexOf(result.ptr));
                }
            },
            .index => if (is_map) {
                const v = try container.getOrPutValue(allocator, key, 0);
                try out.append(allocator, v.*);
                v.* = value;
            },
            .find => {
                if (container.find(key)) |element| {
                    try out.append(allocator, container.indexOf(element));
                    if (is_map)
                        try out.append(allocator, element.value);
                } else {
                    try out.append(allocator, not_found);
                }
            },
            .erase_key => try out.append(allocator, container.erase(key)),
            .erase_iterator => {
                if (container.find(key)) |element| {
                    try out.append(allocator, container.indexOf(element));
                    container.eraseByPtr(element);
                } else {
                    try out.append(allocator, not_found);
                }
            },
            .clear => container.clearAndFree(allocator),
            .clear_and_keep_memory => container.clearRetainingCapacity(),
            .reserve => try container.ensureTotalCapacity(allocator, @truncate(key_u64)),
            .rehash => container.rehash(0),
            .copy_snapshot => {
                var copy = try container.clone(allocator);
                defer copy.deinit(allocator);
                try snapshot(Container, is_map, allocator, &copy, out);
            },
            .snapshot => try snapshot(Container, is_map, allocator, &container, out),
            .copy_replace => {
                var copy = try container.clone(allocator);
                defer copy.deinit(allocator);
                container.swap(&copy);
            },
            .erase_first => {
                var it = container.constIterator();
                if (it.next()) |element| {
                    try out.append(allocator, container.indexOf(element));
                    container.eraseByPtr(element);
                } else {
                    try out.append(allocator, not_found);
                }
            },
        }
    }
}

fn snapshot(comptime Container: type, comptime is_map: bool, allocator: Allocator, container: *const Container, out: *std.ArrayList(u64)) !void {
    try out.append(allocator, container.count());
    try out.append(allocator, container.bucketCount());
    try out.append(allocator, @intFromBool(container.isEmpty()));
    var it = container.constIterator();
    while (it.next()) |element| {
        try out.append(allocator, container.indexOf(element));
        if (is_map) {
            try out.append(allocator, element.key);
            try out.append(allocator, element.value);
        } else {
            try out.append(allocator, element.*);
        }
    }
    try out.append(allocator, snapshot_end);
}

/// Run all scripts on a Zolt container and its C++ counterpart and compare the results.
/// `num_seeds` scripts are generated for both dense and sparse keys.
fn checkContainer(comptime name: []const u8, comptime Container: type, comptime Key: type, comptime Value: type, comptime is_map: bool, comptime jolt_run: RunFn, num_seeds: u32) !void {
    const allocator = std.testing.allocator;

    var checker: fw.Checker = .{ .name = name };
    for ([_]bool{ false, true }) |sparse_keys| {
        for (0..num_seeds) |seed| {
            var script: Script = .{};
            defer script.deinit(allocator);
            var generator: ScriptGenerator = .{
                .allocator = allocator,
                .script = &script,
                .rng = .{ .state = 0x12345678 +% @as(u32, @intCast(seed)) *% 0x9e3779b9 },
                .wide_keys = Key == u64,
                .sparse_keys = sparse_keys,
            };
            try generator.generate();

            // Zolt
            var zolt_out: std.ArrayList(u64) = .empty;
            defer zolt_out.deinit(allocator);
            try runZolt(Container, Key, Value, is_map, allocator, &script, &zolt_out);

            // Jolt
            const jolt_out = try allocator.alloc(u64, zolt_out.items.len + 1024);
            defer allocator.free(jolt_out);
            const jolt_len = jolt_run(script.ops.items.ptr, script.keys.items.ptr, script.values.items.ptr, script.len(), jolt_out.ptr, @intCast(jolt_out.len));

            // Compare, report the first difference
            const input = .{ .sparse_keys = sparse_keys, .seed = seed, .num_ops = script.len() };
            checker.check(input, zolt_out.items.len, jolt_len);
            const common = @min(zolt_out.items.len, jolt_len);
            if (std.mem.indexOfDiff(u64, zolt_out.items[0..common], jolt_out[0..common])) |first_diff| {
                const begin = first_diff -| 4;
                const end = @min(first_diff + 4, common);
                std.debug.print("{s}: {any}: first difference at result {d}\n  zolt: {any}\n  jolt: {any}\n", .{ name, input, first_diff, zolt_out.items[begin..end], jolt_out[begin..end] });
                checker.mismatches += 1;
            }
        }
    }
    try checker.finish();
}

test "UnorderedSet(u32)" {
    try checkContainer("UnorderedSet(u32)", zolt.UnorderedSet(u32, .{}), u32, u32, false, jolt.jolt_unordered_set_u32_run, 2);
}

test "UnorderedSet(u64)" {
    try checkContainer("UnorderedSet(u64)", zolt.UnorderedSet(u64, .{}), u64, u64, false, jolt.jolt_unordered_set_u64_run, 2);
}

test "UnorderedSet(u32) clustered hash" {
    try checkContainer("UnorderedSet(u32) clustered hash", zolt.UnorderedSet(u32, .{ .hash = clusteredHash }), u32, u32, false, jolt.jolt_unordered_set_u32_clustered_run, clustered_num_seeds);
}

test "UnorderedMap(u32, u32)" {
    try checkContainer("UnorderedMap(u32, u32)", zolt.UnorderedMap(u32, u32, .{}), u32, u32, true, jolt.jolt_unordered_map_u32_run, 2);
}

test "UnorderedMap(u64, u64)" {
    try checkContainer("UnorderedMap(u64, u64)", zolt.UnorderedMap(u64, u64, .{}), u64, u64, true, jolt.jolt_unordered_map_u64_run, 2);
}

test "UnorderedMap(u32, u32) clustered hash" {
    try checkContainer("UnorderedMap(u32, u32) clustered hash", zolt.UnorderedMap(u32, u32, .{ .hash = clusteredHash }), u32, u32, true, jolt.jolt_unordered_map_u32_clustered_run, clustered_num_seeds);
}
