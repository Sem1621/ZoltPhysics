//! Parity tests for the concurrency primitives and lock free containers of Jolt/Core: FixedSizeFreeList,
//! LockFreeHashMap (+ LFHMAllocator / LFHMAllocatorContext), MutexArray and Semaphore.
//!
//! These types are deterministic when used from a single thread, and Jolt relies on that: the object indices that
//! FixedSizeFreeList hands out determine body / constraint / node ordering, the iteration order of LockFreeHashMap
//! determines the order of cached contacts. Each test runs the same random single threaded operation sequence on
//! Zolt and on the C++ library (ZoltParity/Core/ConcurrencyReference.cpp) and requires identical results.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;
const Rng = fw.Rng;
const sameValue = fw.sameValue;

const FixedSizeFreeList = zolt.FixedSizeFreeList;
const HashCombine = zolt.HashCombine;
const LFHMAllocator = zolt.LFHMAllocator;
const LFHMAllocatorContext = zolt.LFHMAllocatorContext;
const LockFreeHashMap = zolt.LockFreeHashMap;
const Mutex = zolt.Mutex;
const MutexArray = zolt.MutexArray;
const Semaphore = zolt.Semaphore;

/// C ABI wrappers around the Jolt implementation (ZoltParity/Core/ConcurrencyReference.cpp)
const jolt = struct {
    extern fn jolt_fsfl_create(max_objects: u32, page_size: u32) *anyopaque;
    extern fn jolt_fsfl_destroy(list: *anyopaque) void;
    extern fn jolt_fsfl_construct(list: *anyopaque, value: u32) u32;
    extern fn jolt_fsfl_destruct(list: *anyopaque, index: u32) void;
    extern fn jolt_fsfl_destruct_ptr(list: *anyopaque, index: u32) void;
    extern fn jolt_fsfl_add_to_batch(list: *anyopaque, index: u32) void;
    extern fn jolt_fsfl_destruct_batch(list: *anyopaque) void;
    extern fn jolt_fsfl_get(list: *anyopaque, index: u32) u32;
    extern fn jolt_fsfl_object_storage_size() c_int;

    extern fn jolt_lfhm_create(variant: c_int, object_store_size: u32, max_buckets: u32, block_sizes: [*]const u32, num_contexts: u32) *anyopaque;
    extern fn jolt_lfhm_destroy(map: *anyopaque) void;
    extern fn jolt_lfhm_set_num_buckets(map: *anyopaque, num_buckets: u32) void;
    extern fn jolt_lfhm_clear(map: *anyopaque, clear_allocator: bool) void;
    extern fn jolt_lfhm_reset_contexts(map: *anyopaque) void;
    extern fn jolt_lfhm_insert(map: *anyopaque, context: u32, key: u64, value: u64, extra_bytes: c_int) u32;
    extern fn jolt_lfhm_find(map: *anyopaque, key: u64, out_value: *u64) u32;
    extern fn jolt_lfhm_iterate(map: *anyopaque, out_keys: [*]u64, out_values: [*]u64, out_handles: [*]u32, max_count: u32) u32;
    extern fn jolt_lfhm_get_all(map: *anyopaque, out_handles: [*]u32, max_count: u32) u32;

    extern fn jolt_mutex_array_create(num_mutexes: u32) *anyopaque;
    extern fn jolt_mutex_array_destroy(array: *anyopaque) void;
    extern fn jolt_mutex_array_get_mutex_index(array: *anyopaque, object_index: u32) u32;

    extern fn jolt_semaphore_create() *anyopaque;
    extern fn jolt_semaphore_destroy(semaphore: *anyopaque) void;
    extern fn jolt_semaphore_release(semaphore: *anyopaque, number: u32) void;
    extern fn jolt_semaphore_acquire(semaphore: *anyopaque, number: u32) void;
    extern fn jolt_semaphore_get_value(semaphore: *anyopaque) c_int;
};

// ---------------------------------------------------------------------------------------------------------------------
// FixedSizeFreeList
// ---------------------------------------------------------------------------------------------------------------------

/// Object stored in the free list, has a destructor like the C++ FreeListObject so that the batch destruct walks the batch
const FreeListObject = struct {
    value: u32,
    check: u32,

    fn init(value: u32) FreeListObject {
        return .{ .value = value, .check = ~value };
    }

    pub fn deinit(self: *FreeListObject) void {
        self.value = 0xdeadbeef;
        self.check = 0xdeadbeef;
    }
};

const FreeList = FixedSizeFreeList(FreeListObject);

const FreeListOperation = enum { construct, get, destruct, destruct_ptr, add_to_batch, destruct_batch, refill };

const FreeListConfig = struct {
    max_objects: u32,
    page_size: u32,
    num_operations: u32,
    seed: u32,
};

/// Runs a random sequence of construct / destruct / batch operations on both free lists and compares every returned
/// object index. Stops at the first difference: after that the two lists hold different objects.
fn runFreeListParity(config: FreeListConfig, checker: *Checker) !void {
    const allocator = std.testing.allocator;
    const io = std.testing.io;

    var list: FreeList = try .init(allocator, io, config.max_objects, config.page_size);
    defer list.deinit();
    const jolt_list = jolt.jolt_fsfl_create(config.max_objects, config.page_size);
    defer jolt.jolt_fsfl_destroy(jolt_list);

    // Objects that are constructed and not in the batch, and objects that are in the batch
    var live: std.ArrayList(u32) = .empty;
    defer live.deinit(allocator);
    var batch_objects: std.ArrayList(u32) = .empty;
    defer batch_objects.deinit(allocator);
    var batch: FreeList.Batch = .{};

    var rng: Rng = .{ .state = config.seed };
    var diverged = false;
    var target_live: u32 = 0;
    var operation: u32 = 0;
    while (operation < config.num_operations and !diverged) : (operation += 1) {
        // Steer towards a fill level that changes now and then, sometimes beyond the capacity so that the list runs full
        if (operation % 500 == 0)
            target_live = rng.intRange(u32, 0, config.max_objects + config.max_objects / 4);

        const r = rng.next() % 100;
        const num_live: u32 = @intCast(live.items.len);
        if (num_live == 0 or (num_live < target_live and r < 70) or (num_live >= target_live and r < 30)) {
            const value = operation;
            const index = try list.constructObject(.init(value));
            const jolt_index = jolt.jolt_fsfl_construct(jolt_list, value);
            checker.check(.{ config, operation, FreeListOperation.construct }, index, jolt_index);
            if (index != FreeList.invalid_object_index)
                try live.append(allocator, index);
            if (index != jolt_index) {
                diverged = true;
                break;
            }
            if (index != FreeList.invalid_object_index)
                checker.check(.{ config, operation, FreeListOperation.get }, list.get(index).value, jolt.jolt_fsfl_get(jolt_list, index));
        } else {
            const index = live.swapRemove(rng.next() % num_live);
            switch (rng.next() % 4) {
                0 => {
                    list.destructObject(index);
                    jolt.jolt_fsfl_destruct(jolt_list, index);
                },
                1 => {
                    list.destructObjectPtr(list.get(index));
                    jolt.jolt_fsfl_destruct_ptr(jolt_list, index);
                },
                else => {
                    list.addObjectToBatch(&batch, index);
                    jolt.jolt_fsfl_add_to_batch(jolt_list, index);
                    try batch_objects.append(allocator, index);
                },
            }
        }

        // Now and then free the batch (sometimes when it is empty)
        if (rng.next() % 16 == 0) {
            list.destructObjectBatch(&batch);
            jolt.jolt_fsfl_destruct_batch(jolt_list);
            batch = .{};
            batch_objects.clearRetainingCapacity();
        }
    }

    // Free everything
    list.destructObjectBatch(&batch);
    batch = .{};
    for (live.items) |index| list.destructObject(index);
    if (diverged) return;
    jolt.jolt_fsfl_destruct_batch(jolt_list);
    for (live.items) |index| jolt.jolt_fsfl_destruct(jolt_list, index);
    live.clearRetainingCapacity();

    // Construct until the list is full, both must hand out all objects in the same order
    // (the capacity is max_objects rounded up to whole pages, the last 2 constructs fail)
    for (0..std.mem.alignForward(u32, config.max_objects, config.page_size) + 2) |_| {
        const index = try list.constructObject(.init(operation));
        const jolt_index = jolt.jolt_fsfl_construct(jolt_list, operation);
        checker.check(.{ config, operation, FreeListOperation.refill }, index, jolt_index);
        if (index != FreeList.invalid_object_index) {
            list.addObjectToBatch(&batch, index);
            if (index != jolt_index) {
                list.destructObjectBatch(&batch);
                return;
            }
            jolt.jolt_fsfl_add_to_batch(jolt_list, index);
        }
        operation += 1;
    }
    list.destructObjectBatch(&batch);
    jolt.jolt_fsfl_destruct_batch(jolt_list);
}

test "FixedSizeFreeList object indices" {
    var checker: Checker = .{ .name = "FixedSizeFreeList object indices" };
    var size_checker: Checker = .{ .name = "FixedSizeFreeList object storage size" };
    size_checker.check({}, @as(c_int, FreeList.object_storage_size), jolt.jolt_fsfl_object_storage_size());

    const configs = [_]FreeListConfig{
        .{ .max_objects = 1000, .page_size = 16, .num_operations = 30_000, .seed = 0x12345678 },
        .{ .max_objects = 100, .page_size = 128, .num_operations = 10_000, .seed = 0x2468ace0 }, // One page, larger than needed
        .{ .max_objects = 4096, .page_size = 64, .num_operations = 30_000, .seed = 0x13579bdf },
        .{ .max_objects = 37, .page_size = 4, .num_operations = 20_000, .seed = 0xcafebabe }, // Runs full often
        .{ .max_objects = 1, .page_size = 1, .num_operations = 1_000, .seed = 0x0badf00d },
        .{ .max_objects = 256, .page_size = 256, .num_operations = 10_000, .seed = 0xfeedface },
    };
    for (configs) |config|
        try runFreeListParity(config, &checker);

    try finishAll(&.{ &checker, &size_checker });
}

// ---------------------------------------------------------------------------------------------------------------------
// LockFreeHashMap
// ---------------------------------------------------------------------------------------------------------------------

const max_contexts = 8;

const HashMapOperation = enum { insert, find, iterate, iterate_count, get_all, get_all_count };

const HashMapConfig = struct {
    object_store_size: u32,
    max_buckets: u32,
    /// Block size of each allocator context, each context acts like a separate thread
    block_sizes: []const u32,
    /// Keys are taken from [0, key_range)
    key_range: u32,
    num_operations: u32,
    /// Chance (per 10000 operations) that the map is cleared
    clear_chance: u32,
    seed: u32,
};

/// Result of a find: handle (invalid_handle when not found) and value (0 when not found)
const FindResult = struct { handle: u32, value: u64 };

/// An entry of the map as returned by the iterator
const Entry = struct { key: u64, value: u64, handle: u32 };

/// Amount of extra bytes requested by `create`, picked at random
const extra_bytes_choices = [_]i32{ 0, 0, 0, 0, 1, 3, 4, 8, 13, 32 };

/// Runs a random sequence of insert / find / clear operations on both maps and compares the returned handles, the
/// found values and the iteration order. Stops at the first difference.
fn runHashMapParity(comptime T: type, config: HashMapConfig, checker: *Checker) !void {
    const allocator = std.testing.allocator;
    const Map = LockFreeHashMap(T, T);
    const variant: c_int = if (T == u32) 0 else 1;

    var lfhm_allocator: LFHMAllocator = try .init(allocator, config.object_store_size);
    defer lfhm_allocator.deinit(allocator);
    var map: Map = try .init(allocator, &lfhm_allocator, config.max_buckets);
    defer map.deinit(allocator);
    var contexts: [max_contexts]LFHMAllocatorContext = undefined;
    for (config.block_sizes, 0..) |block_size, i|
        contexts[i] = .init(&lfhm_allocator, block_size);

    const jolt_map = jolt.jolt_lfhm_create(variant, config.object_store_size, config.max_buckets, config.block_sizes.ptr, @intCast(config.block_sizes.len));
    defer jolt.jolt_lfhm_destroy(jolt_map);

    // Buffers for comparing the full contents
    const max_entries = config.object_store_size / @sizeOf(Map.KeyValue) + 1;
    const keys = try allocator.alloc(u64, max_entries);
    defer allocator.free(keys);
    const values = try allocator.alloc(u64, max_entries);
    defer allocator.free(values);
    const handles = try allocator.alloc(u32, max_entries);
    defer allocator.free(handles);

    var rng: Rng = .{ .state = config.seed };
    var operation: u32 = 0;
    while (operation < config.num_operations) : (operation += 1) {
        const r = rng.next() % 10000;
        if (r < config.clear_chance) {
            // Clear the map, usually together with the allocator (like ContactConstraintManager does every frame)
            const clear_allocator = rng.next() % 3 != 0;
            map.clear();
            if (clear_allocator) {
                lfhm_allocator.clear();
                for (config.block_sizes, 0..) |block_size, i|
                    contexts[i] = .init(&lfhm_allocator, block_size);
            }
            jolt.jolt_lfhm_clear(jolt_map, clear_allocator);

            // Sometimes change the number of buckets (only allowed when the map is empty)
            if (clear_allocator and rng.next() % 2 == 0) {
                const num_buckets = @as(u32, 4) << @intCast(rng.intRange(u32, 0, @ctz(config.max_buckets) - 2));
                map.setNumBuckets(num_buckets);
                jolt.jolt_lfhm_set_num_buckets(jolt_map, num_buckets);
            }
        } else if (r < config.clear_chance + 50) {
            // Contexts start new blocks, as if new jobs start inserting
            for (config.block_sizes, 0..) |block_size, i|
                contexts[i] = .init(&lfhm_allocator, block_size);
            jolt.jolt_lfhm_reset_contexts(jolt_map);
        } else if (r < config.clear_chance + 150) {
            if (!try compareHashMapContents(T, &map, jolt_map, keys, values, handles, .{ config, operation }, checker))
                return;
        } else {
            const key = randomKey(T, &rng, config.key_range);
            const key_hash = HashCombine.hash(key);
            var jolt_value: u64 = 0;
            if (r % 3 == 0 or map.find(key, key_hash) != null) {
                // Find (a map is not a multi map, so keys that already exist are not inserted again)
                const kv = map.find(key, key_hash);
                const handle = if (kv) |found| map.toHandle(found) else Map.invalid_handle;
                const value: u64 = if (kv) |found| found.getValueConst().* else 0;
                const jolt_handle = jolt.jolt_lfhm_find(jolt_map, key, &jolt_value);
                checker.check(.{ config, operation, HashMapOperation.find, key }, FindResult{ .handle = handle, .value = value }, .{ .handle = jolt_handle, .value = jolt_value });
                if (handle != jolt_handle)
                    return;
            } else {
                // Insert using a random context
                const context: u32 = rng.next() % @as(u32, @intCast(config.block_sizes.len));
                const extra_bytes = extra_bytes_choices[rng.next() % extra_bytes_choices.len];
                const value = randomValue(T, &rng);
                const handle = if (map.create(&contexts[context], key, key_hash, extra_bytes, value)) |kv| map.toHandle(kv) else Map.invalid_handle;
                const jolt_handle = jolt.jolt_lfhm_insert(jolt_map, context, key, value, extra_bytes);
                checker.check(.{ config, operation, HashMapOperation.insert, key }, handle, jolt_handle);
                if (handle != jolt_handle)
                    return;
            }
        }
    }

    // Final contents, then clear
    if (!try compareHashMapContents(T, &map, jolt_map, keys, values, handles, .{ config, operation }, checker))
        return;
    map.clear();
    lfhm_allocator.clear();
    jolt.jolt_lfhm_clear(jolt_map, true);
    if (!try compareHashMapContents(T, &map, jolt_map, keys, values, handles, .{ config, operation + 1 }, checker))
        return;
    for (0..config.key_range) |i| {
        const key = keyFromIndex(T, @intCast(i));
        var jolt_value: u64 = 0;
        try std.testing.expectEqual(@as(?*const Map.KeyValue, null), map.find(key, HashCombine.hash(key)));
        try std.testing.expectEqual(Map.invalid_handle, jolt.jolt_lfhm_find(jolt_map, key, &jolt_value));
    }
}

/// Compares the iteration order (begin / operator++) and GetAllKeyValues of both maps, returns false on a difference
fn compareHashMapContents(comptime T: type, map: *LockFreeHashMap(T, T), jolt_map: *anyopaque, keys: []u64, values: []u64, handles: []u32, input: anytype, checker: *Checker) !bool {
    const Map = LockFreeHashMap(T, T);
    const max_count: u32 = @intCast(keys.len);

    // Iterator
    const jolt_count = jolt.jolt_lfhm_iterate(jolt_map, keys.ptr, values.ptr, handles.ptr, max_count);
    var count: u32 = 0;
    var it = map.begin();
    while (!it.eql(map.end())) : (it.advance()) {
        const kv = it.get();
        if (count < @min(jolt_count, max_count)) {
            const expected: Entry = .{ .key = keys[count], .value = values[count], .handle = handles[count] };
            const actual: Entry = .{ .key = kv.getKey().*, .value = kv.getValue().*, .handle = map.toHandle(kv) };
            checker.check(.{ input, HashMapOperation.iterate, count }, actual, expected);
            if (!sameValue(actual, expected))
                return false;
        }
        count += 1;
    }
    checker.check(.{ input, HashMapOperation.iterate_count }, count, jolt_count);
    if (count != jolt_count)
        return false;

    // Zig style iteration visits the same entries
    var it2 = map.begin();
    var index: u32 = 0;
    while (it2.next()) |kv| : (index += 1)
        try std.testing.expectEqual(handles[index], map.toHandle(kv));
    try std.testing.expectEqual(count, index);

    // GetAllKeyValues
    const jolt_all_count = jolt.jolt_lfhm_get_all(jolt_map, handles.ptr, max_count);
    var all: std.ArrayList(*const Map.KeyValue) = .empty;
    defer all.deinit(std.testing.allocator);
    try map.getAllKeyValues(std.testing.allocator, &all);
    checker.check(.{ input, HashMapOperation.get_all_count }, @as(u32, @intCast(all.items.len)), jolt_all_count);
    if (all.items.len != jolt_all_count)
        return false;
    for (all.items, 0..) |kv, i| {
        checker.check(.{ input, HashMapOperation.get_all, i }, map.toHandle(kv), handles[i]);
        if (map.toHandle(kv) != handles[i])
            return false;
    }
    return true;
}

/// The i-th key of the key range: the index itself for 32 bit keys, the index spread over all 64 bits for 64 bit keys
fn keyFromIndex(comptime T: type, i: u32) T {
    return if (T == u32) i else @as(u64, i) *% 0x9e3779b97f4a7c15;
}

fn randomKey(comptime T: type, rng: *Rng, key_range: u32) T {
    return keyFromIndex(T, rng.intRange(u32, 0, key_range - 1));
}

fn randomValue(comptime T: type, rng: *Rng) T {
    return if (T == u32) rng.next() else (@as(u64, rng.next()) << 32) | rng.next();
}

test "LockFreeHashMap<uint32, uint32> handles and iteration order" {
    var checker: Checker = .{ .name = "LockFreeHashMap<uint32, uint32>" };
    const configs = [_]HashMapConfig{
        // Large map that slowly fills up
        .{ .object_store_size = 1 << 16, .max_buckets = 1024, .block_sizes = &.{ 256, 512, 100 }, .key_range = 8000, .num_operations = 40_000, .clear_chance = 1, .seed = 0x12345678 },
        // Small map that runs full all the time, few buckets so that the chains are long
        .{ .object_store_size = 2048, .max_buckets = 16, .block_sizes = &.{ 64, 96, 200 }, .key_range = 400, .num_operations = 20_000, .clear_chance = 30, .seed = 0x87654321 },
        // Store size and block sizes that are not a multiple of the key value alignment, a single context
        .{ .object_store_size = 10_001, .max_buckets = 64, .block_sizes = &.{77}, .key_range = 2000, .num_operations = 20_000, .clear_chance = 10, .seed = 0x0f1e2d3c },
        // Many contexts
        .{ .object_store_size = 1 << 14, .max_buckets = 128, .block_sizes = &.{ 32, 48, 64, 128, 256, 512, 1024, 4096 }, .key_range = 3000, .num_operations = 20_000, .clear_chance = 5, .seed = 0x5a5a5a5a },
    };
    for (configs) |config|
        try runHashMapParity(u32, config, &checker);
    try checker.finish();
}

test "LockFreeHashMap<uint64, uint64> handles and iteration order" {
    var checker: Checker = .{ .name = "LockFreeHashMap<uint64, uint64>" };
    const configs = [_]HashMapConfig{
        .{ .object_store_size = 1 << 15, .max_buckets = 256, .block_sizes = &.{ 128, 333, 1024 }, .key_range = 4000, .num_operations = 30_000, .clear_chance = 2, .seed = 0x31415926 },
        .{ .object_store_size = 1000, .max_buckets = 4, .block_sizes = &.{ 50, 77 }, .key_range = 200, .num_operations = 10_000, .clear_chance = 50, .seed = 0x27182818 },
    };
    for (configs) |config|
        try runHashMapParity(u64, config, &checker);
    try checker.finish();
}

// ---------------------------------------------------------------------------------------------------------------------
// MutexArray
// ---------------------------------------------------------------------------------------------------------------------

test "MutexArray.getMutexIndex" {
    const allocator = std.testing.allocator;
    var checker: Checker = .{ .name = "MutexArray.getMutexIndex" };
    var rng: Rng = .{};

    const special_indices = [_]u32{ 0, 1, 2, 3, 0x7fffffff, 0x80000000, 0xfffffffe, 0xffffffff };
    for ([_]u32{ 1, 2, 4, 16, 64, 1024 }) |num_mutexes| {
        var array: MutexArray(Mutex) = try .init(allocator, num_mutexes);
        defer array.deinit(allocator);
        const jolt_array = jolt.jolt_mutex_array_create(num_mutexes);
        defer jolt.jolt_mutex_array_destroy(jolt_array);

        for (0..fw.iterations / 4) |i| {
            const object_index: u32 = switch (i % 4) {
                0 => @intCast(i), // Consecutive indices, like body indices
                1 => special_indices[rng.next() % special_indices.len],
                else => rng.next(),
            };
            checker.check(.{ num_mutexes, object_index }, array.getMutexIndex(object_index), jolt.jolt_mutex_array_get_mutex_index(jolt_array, object_index));
        }
    }
    try checker.finish();
}

// ---------------------------------------------------------------------------------------------------------------------
// Semaphore
// ---------------------------------------------------------------------------------------------------------------------

test "Semaphore.getValue" {
    // Single threaded release / acquire sequence that never blocks
    const io = std.testing.io;
    var checker: Checker = .{ .name = "Semaphore.getValue" };
    var rng: Rng = .{};

    var semaphore: Semaphore = .{};
    const jolt_semaphore = jolt.jolt_semaphore_create();
    defer jolt.jolt_semaphore_destroy(jolt_semaphore);

    for (0..10_000) |i| {
        const value = semaphore.getValue();
        if (value > 0 and rng.next() % 2 == 0) {
            const number = rng.intRange(u32, 1, @intCast(value));
            semaphore.acquire(io, .{ .number = number });
            jolt.jolt_semaphore_acquire(jolt_semaphore, number);
        } else {
            const number = rng.intRange(u32, 1, 10);
            semaphore.release(io, .{ .number = number });
            jolt.jolt_semaphore_release(jolt_semaphore, number);
        }
        checker.check(i, semaphore.getValue(), jolt.jolt_semaphore_get_value(jolt_semaphore));
    }

    // Take everything that is left
    const value = semaphore.getValue();
    if (value > 0) {
        semaphore.acquire(io, .{ .number = @intCast(value) });
        jolt.jolt_semaphore_acquire(jolt_semaphore, @intCast(value));
    }
    checker.check(@as(usize, 10_000), semaphore.getValue(), jolt.jolt_semaphore_get_value(jolt_semaphore));
    try checker.finish();
}
