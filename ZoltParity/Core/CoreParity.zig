//! Parity tests for Jolt/Core: run Zolt and the C++ Jolt library on the same inputs and require identical results.
//! C ABI wrappers: ZoltParity/Core/CoreReference.cpp. See ZoltParity/parity.zig for how parity tests work.
//!
//! Covers the Core code with observable numeric or ordering results: QuickSort / InsertionSort (order of equal
//! keys), BinaryHeap, HashCombine, Mt19937 (std::mt19937), LinearCurve, the bytes written and read by
//! StreamOut / StreamIn / StreamWrapper, and StringTools.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;
const Rng = fw.Rng;

const HashCombine = zolt.HashCombine;
const LinearCurve = zolt.LinearCurve;
const Point = LinearCurve.Point;
const StreamIn = zolt.StreamIn;
const StreamOut = zolt.StreamOut;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Float3 = zolt.Float3;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

const allocator = std.testing.allocator;

/// Number of random inputs per test
const iterations = fw.iterations;

/// Element with a sort key and its original position (SortElem in CoreReference.cpp)
const SortElem = extern struct {
    key: u32,
    index: u32,
};

/// Types of the values in jolt_hash_combine_sequence (EHashType in CoreReference.cpp)
const HashType = enum(u32) { float, double, int, uint32, uint64, char };

/// Enum that is streamed as raw bytes (EStreamEnum in CoreReference.cpp)
const StreamEnum = enum(u16) { a, b, _ };

/// All types that StreamIn / StreamOut handle (StreamSample in CoreReference.cpp)
const StreamSample = extern struct {
    u8_value: u8,
    u16_value: u16,
    u32_value: u32,
    u64_value: u64,
    i32_value: i32,
    float_value: f32,
    double_value: f64,
    bool_value: bool,
    enum_value: u16,
    vec3: [3]f32,
    vec4: [4]f32,
    quat: [4]f32,
    mat44: [16]f32,
    float3: [3]f32,
    dvec3: [3]f64,
    dmat44_cols: [12]f32,
    dmat44_t: [3]f64,
    float_array_len: u32,
    float_array: [8]f32,
    vec3_array_len: u32,
    vec3_array: [4][3]f32,
    dvec3_array_len: u32,
    dvec3_array: [3][3]f64,
    dmat44_array_len: u32,
    dmat44_array_cols: [2][12]f32,
    dmat44_array_t: [2][3]f64,
    string_len: u32,
    string: [16]u8,
    point_array_len: u32,
    point_array: [4][2]f32,
    curve_len: u32,
    curve: [4][2]f32,
};

/// Fields of StreamSample in the order in which they are streamed (one status entry per field)
const StreamField = enum(u8) { u8_value, u16_value, u32_value, u64_value, i32_value, float_value, double_value, bool_value, enum_value, vec3, vec4, quat, mat44, float3, dvec3, dmat44, float_array, vec3_array, dvec3_array, dmat44_array, string, point_array, curve };
const num_stream_fields = 23; // cNumStreamFields in CoreReference.cpp
comptime {
    std.debug.assert(@typeInfo(StreamField).@"enum".fields.len == num_stream_fields);
}

/// The C++ reference functions, see CoreReference.cpp
const jolt = struct {
    extern fn jolt_quick_sort_elems(items: [*]SortElem, count: u32) void;
    extern fn jolt_insertion_sort_elems(items: [*]SortElem, count: u32) void;
    extern fn jolt_quick_sort_floats(items: [*]f32, count: u32) void;
    extern fn jolt_insertion_sort_floats(items: [*]f32, count: u32) void;

    extern fn jolt_binary_heap_push_less(items: [*]SortElem, count: u32) void;
    extern fn jolt_binary_heap_pop_less(items: [*]SortElem, count: u32) void;
    extern fn jolt_binary_heap_push_less_equal(items: [*]SortElem, count: u32) void;
    extern fn jolt_binary_heap_pop_less_equal(items: [*]SortElem, count: u32) void;

    extern fn jolt_hash_bytes(data: [*]const u8, size: u32, seed: u64) u64;
    extern fn jolt_hash_bytes_default_seed(data: [*]const u8, size: u32) u64;
    extern fn jolt_hash_string(string: [*:0]const u8) u64;
    extern fn jolt_hash_c_string(string: [*:0]const u8) u64;
    extern fn jolt_hash_string_view(data: [*]const u8, size: u32) u64;
    extern fn jolt_hash_string_jolt_string(data: [*]const u8, size: u32) u64;
    extern fn jolt_hash64(value: u64) u64;
    extern fn jolt_hash_float(value: f32) u64;
    extern fn jolt_hash_double(value: f64) u64;
    extern fn jolt_hash_int(value: c_int) u64;
    extern fn jolt_hash_uint32(value: u32) u64;
    extern fn jolt_hash_uint64(value: u64) u64;
    extern fn jolt_hash_char(value: u8) u64;
    extern fn jolt_hash_combine_sequence(seed: u64, types: [*]const HashType, values: [*]const u64, count: u32) u64;
    extern fn jolt_hash_combine_args(a: f32, b: u32, c: c_int, d: u64, e: f64) u64;

    extern fn jolt_mt19937(seed: u32, values: [*]u32, count: u32) void;
    extern fn jolt_mt19937_default(values: [*]u32, count: u32) void;

    extern fn jolt_linear_curve_sort(points: [*]f32, count: u32) void;
    extern fn jolt_linear_curve_get_values(points: [*]const f32, count: u32, x: [*]const f32, y: [*]f32, num_x: u32) void;
    extern fn jolt_linear_curve_min_max(points: [*]const f32, count: u32, min_max: *[2]f32) void;
    extern fn jolt_linear_curve_save(points: [*]const f32, count: u32, bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_linear_curve_restore(bytes: [*]const u8, size: u32, points: [*]f32, capacity: u32, eof: *bool, failed: *bool) u32;

    extern fn jolt_stream_write_sample(sample: *const StreamSample, bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_stream_read_sample(bytes: [*]const u8, size: u32, sample: *StreamSample, status: *[num_stream_fields]u8) void;

    extern fn jolt_convert_to_string_float(value: f32, chars: [*]u8, capacity: u32) u32;
    extern fn jolt_convert_to_string_double(value: f64, chars: [*]u8, capacity: u32) u32;
    extern fn jolt_convert_to_string_int(value: c_int, chars: [*]u8, capacity: u32) u32;
    extern fn jolt_convert_to_string_uint64(value: u64, chars: [*]u8, capacity: u32) u32;
    extern fn jolt_to_lower(chars: [*]const u8, size: u32, out: [*]u8, capacity: u32) u32;
    extern fn jolt_string_replace(chars: [*]const u8, size: u32, search: [*]const u8, search_size: u32, replace: [*]const u8, replace_size: u32, out: [*]u8, capacity: u32) u32;
    extern fn jolt_string_to_vector(chars: [*]const u8, size: u32, delimiter: [*]const u8, delimiter_size: u32, clear_vector: bool, num_initial: u32, lengths: [*]u32, max_strings: u32, out: [*]u8, capacity: u32) u32;
    extern fn jolt_vector_to_string(lengths: [*]const u32, num_strings: u32, chars: [*]const u8, delimiter: [*]const u8, delimiter_size: u32, out: [*]u8, capacity: u32) u32;
};

/// Random float with any bit pattern (NaN, inf, denormals) or one of the special values
fn anyFloat(rng: *Rng) f32 {
    const special = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, std.math.inf(f32), -std.math.inf(f32), std.math.nan(f32), zolt.math.flt_min, zolt.math.flt_max, 1.0e-45 };
    return switch (rng.next() % 4) {
        0 => special[rng.next() % special.len],
        1 => rng.float(-1000, 1000),
        else => @bitCast(rng.next()),
    };
}

/// Random double with any bit pattern or one of the special values
fn anyDouble(rng: *Rng) f64 {
    const special = [_]f64{ 0.0, -0.0, 1.0, -1.0, 0.5, 0.1, std.math.inf(f64), -std.math.inf(f64), std.math.nan(f64), std.math.floatMin(f64), std.math.floatMax(f64), 4.9406564584124654e-324, 1234565.0, 999999.5, 0.15 };
    return switch (rng.next() % 4) {
        0 => special[rng.next() % special.len],
        1 => @as(f64, rng.float(-1000, 1000)),
        else => @bitCast(next64(rng)),
    };
}

/// 64 random bits
fn next64(rng: *Rng) u64 {
    return (@as(u64, rng.next()) << 32) | rng.next();
}

// ---------------------------------------------------------------------------------------------------------------------
// QuickSort / InsertionSort
// ---------------------------------------------------------------------------------------------------------------------

fn lessKey(_: void, a: SortElem, b: SortElem) bool {
    return a.key < b.key;
}

fn lessEqualKey(_: void, a: SortElem, b: SortElem) bool {
    return a.key <= b.key;
}

/// Sizes to sort: all small sizes (insertion sort fallback of QuickSort at <= 32 elements) and sizes around powers of 2
const sort_sizes = blk: {
    var sizes: [101 + 16]u32 = undefined;
    for (0..101) |i| sizes[i] = i;
    const large = [_]u32{ 127, 128, 129, 200, 255, 256, 257, 500, 1000, 1023, 1024, 1025, 2048, 5000, 10000, 65537 };
    for (large, 0..) |s, i| sizes[101 + i] = s;
    break :blk sizes;
};

/// Number of distinct keys (1 = all equal)
const key_ranges = [_]u32{ 1, 2, 3, 10, 1000, std.math.maxInt(u32) };

const SortPattern = enum { random, ascending, descending, organ_pipe, sawtooth };

fn fillSortElems(rng: *Rng, items: []SortElem, pattern: SortPattern, key_range: u32) void {
    const n: u32 = @intCast(items.len);
    for (items, 0..) |*item, i_usize| {
        const i: u32 = @intCast(i_usize);
        const raw: u64 = switch (pattern) {
            .random => rng.next(),
            .ascending => i,
            .descending => n - i,
            .organ_pipe => if (i < n / 2) i else n - i,
            .sawtooth => i % 7,
        };
        item.* = .{ .key = if (key_range == std.math.maxInt(u32)) @truncate(raw) else @intCast(raw % key_range), .index = i };
    }
}

fn checkSortedElems(checker: *Checker, input: anytype, zolt_items: []const SortElem, jolt_items: []const SortElem) void {
    for (zolt_items, jolt_items, 0..) |z, j, i| {
        if (z.key != j.key or z.index != j.index) {
            // Report the first difference only
            checker.check(.{ .input = input, .position = i }, z, j);
            return;
        }
    }
}

test "QuickSort / InsertionSort with equal keys" {
    var rng: Rng = .{};
    var quick_checker: Checker = .{ .name = "QuickSort (key only comparator)" };
    var insertion_checker: Checker = .{ .name = "InsertionSort (key only comparator)" };

    const max_size = sort_sizes[sort_sizes.len - 1];
    const original = try allocator.alloc(SortElem, max_size);
    defer allocator.free(original);
    const zolt_items = try allocator.alloc(SortElem, max_size);
    defer allocator.free(zolt_items);
    const jolt_items = try allocator.alloc(SortElem, max_size);
    defer allocator.free(jolt_items);

    for (sort_sizes) |size| {
        for (std.enums.values(SortPattern)) |pattern| {
            for (key_ranges) |key_range| {
                const repetitions: usize = if (pattern == .random and size <= 2048) 3 else 1;
                for (0..repetitions) |_| {
                    fillSortElems(&rng, original[0..size], pattern, key_range);
                    const input = .{ .size = size, .pattern = @intFromEnum(pattern), .key_range = key_range };

                    @memcpy(zolt_items[0..size], original[0..size]);
                    @memcpy(jolt_items[0..size], original[0..size]);
                    zolt.quickSort(SortElem, zolt_items[0..size], {}, lessKey);
                    jolt.jolt_quick_sort_elems(jolt_items.ptr, size);
                    checkSortedElems(&quick_checker, input, zolt_items[0..size], jolt_items[0..size]);

                    if (size <= 2048) {
                        @memcpy(zolt_items[0..size], original[0..size]);
                        @memcpy(jolt_items[0..size], original[0..size]);
                        zolt.insertionSort(SortElem, zolt_items[0..size], {}, lessKey);
                        jolt.jolt_insertion_sort_elems(jolt_items.ptr, size);
                        checkSortedElems(&insertion_checker, input, zolt_items[0..size], jolt_items[0..size]);
                    }
                }
            }
        }
    }
    try finishAll(&.{ &quick_checker, &insertion_checker });
}

test "QuickSort / InsertionSort floats with -0 and +0 (default comparator)" {
    var rng: Rng = .{};
    var quick_checker: Checker = .{ .name = "QuickSort (std::less<float>)" };
    var insertion_checker: Checker = .{ .name = "InsertionSort (std::less<float>)" };
    const values = [_]f32{ -0.0, 0.0, 1.0, -1.0, 0.5, std.math.inf(f32), -std.math.inf(f32) };

    var original: [300]f32 = undefined;
    var zolt_items: [300]f32 = undefined;
    var jolt_items: [300]f32 = undefined;
    for (0..2000) |iteration| {
        const size: u32 = if (iteration < 300) @intCast(iteration) else rng.intRange(u32, 0, 300);
        const num_values = rng.intRange(usize, 2, values.len);
        for (original[0..size]) |*v| v.* = values[rng.next() % num_values];

        @memcpy(zolt_items[0..size], original[0..size]);
        @memcpy(jolt_items[0..size], original[0..size]);
        zolt.quickSort(f32, zolt_items[0..size], {}, std.sort.asc(f32));
        jolt.jolt_quick_sort_floats(&jolt_items, size);
        for (zolt_items[0..size], jolt_items[0..size], 0..) |z, j, i|
            quick_checker.check(.{ .iteration = iteration, .position = i }, @as(u32, @bitCast(z)), @as(u32, @bitCast(j)));

        @memcpy(zolt_items[0..size], original[0..size]);
        @memcpy(jolt_items[0..size], original[0..size]);
        zolt.insertionSort(f32, zolt_items[0..size], {}, std.sort.asc(f32));
        jolt.jolt_insertion_sort_floats(&jolt_items, size);
        for (zolt_items[0..size], jolt_items[0..size], 0..) |z, j, i|
            insertion_checker.check(.{ .iteration = iteration, .position = i }, @as(u32, @bitCast(z)), @as(u32, @bitCast(j)));
    }
    try finishAll(&.{ &quick_checker, &insertion_checker });
}

// ---------------------------------------------------------------------------------------------------------------------
// BinaryHeap
// ---------------------------------------------------------------------------------------------------------------------

fn checkHeapSequences(
    name: []const u8,
    comptime pred: fn (void, SortElem, SortElem) bool,
    comptime jolt_push: fn ([*]SortElem, u32) callconv(.c) void,
    comptime jolt_pop: fn ([*]SortElem, u32) callconv(.c) void,
) !void {
    var rng: Rng = .{};
    var checker: Checker = .{ .name = name };
    const max_size = 300;
    var zolt_items: [max_size]SortElem = undefined;
    var jolt_items: [max_size]SortElem = undefined;

    const ops_per_sequence = 1000;
    for (0..iterations / ops_per_sequence) |sequence| {
        const key_range = [_]u32{ 1, 2, 4, 16, 1000 }[sequence % 5];
        var len: u32 = 0;
        var next_index: u32 = 0;
        for (0..ops_per_sequence) |op| {
            var compare_len: u32 = undefined;
            if (len == 0 or (len < max_size and rng.next() % 3 != 0)) {
                // Push
                const elem: SortElem = .{ .key = rng.next() % key_range, .index = next_index };
                next_index += 1;
                zolt_items[len] = elem;
                jolt_items[len] = elem;
                len += 1;
                zolt.binaryHeapPush(SortElem, zolt_items[0..len], {}, pred);
                jolt_push(&jolt_items, len);
                compare_len = len;
            } else {
                // Pop, the popped element is moved to the end
                zolt.binaryHeapPop(SortElem, zolt_items[0..len], {}, pred);
                jolt_pop(&jolt_items, len);
                compare_len = len;
                len -= 1;
            }
            checkSortedElems(&checker, .{ .sequence = sequence, .op = op }, zolt_items[0..compare_len], jolt_items[0..compare_len]);
        }
    }
    try checker.finish();
}

test "BinaryHeapPush / BinaryHeapPop sequences (less)" {
    try checkHeapSequences("BinaryHeap (less)", lessKey, jolt.jolt_binary_heap_push_less, jolt.jolt_binary_heap_pop_less);
}

test "BinaryHeapPush / BinaryHeapPop sequences (less or equal)" {
    try checkHeapSequences("BinaryHeap (less or equal)", lessEqualKey, jolt.jolt_binary_heap_push_less_equal, jolt.jolt_binary_heap_pop_less_equal);
}

// ---------------------------------------------------------------------------------------------------------------------
// HashCombine
// ---------------------------------------------------------------------------------------------------------------------

test "HashCombine.hashBytes" {
    var rng: Rng = .{};
    var checker: Checker = .{ .name = "HashBytes" };
    var default_checker: Checker = .{ .name = "HashBytes (default seed)" };
    var bytes: [64]u8 = undefined;
    for (0..iterations) |_| {
        const len = rng.intRange(usize, 0, bytes.len);
        for (bytes[0..len]) |*b| b.* = @truncate(rng.next());
        const seed = if (rng.next() % 4 == 0) HashCombine.fnv1a_seed else next64(&rng);
        checker.check(.{ .len = len, .seed = seed }, HashCombine.hashBytesSeeded(bytes[0..len], seed), jolt.jolt_hash_bytes(&bytes, @intCast(len), seed));
        default_checker.check(.{ .len = len }, HashCombine.hashBytes(bytes[0..len]), jolt.jolt_hash_bytes_default_seed(&bytes, @intCast(len)));
    }
    try finishAll(&.{ &checker, &default_checker });
}

test "HashCombine.hashString" {
    var rng: Rng = .{};
    var string_checker: Checker = .{ .name = "HashString" };
    var c_string_checker: Checker = .{ .name = "Hash<const char *>" };
    var view_checker: Checker = .{ .name = "Hash<string_view>" };
    var jolt_string_checker: Checker = .{ .name = "Hash<String>" };
    var bytes: [33]u8 = undefined;
    for (0..iterations) |_| {
        const len = rng.intRange(usize, 0, bytes.len - 1);

        // HashString / Hash<const char *> stop at the terminating zero and convert `char` to uint64. char is signed on
        // x86 and unsigned on ARM, so Jolt's result for characters >= 0x80 depends on the platform (Zolt follows
        // unsigned char, like HashBytes). Only 7-bit characters are compared.
        for (bytes[0..len]) |*b| b.* = @intCast(rng.intRange(u32, 1, 127));
        bytes[len] = 0;
        const c_string: [*:0]const u8 = @ptrCast(&bytes);
        string_checker.check(len, HashCombine.hashString(bytes[0..len]), jolt.jolt_hash_string(c_string));
        c_string_checker.check(len, HashCombine.hash(bytes[0..len]), jolt.jolt_hash_c_string(c_string));

        // string_view / String hash all bytes, including zeros and bytes >= 0x80
        for (bytes[0..len]) |*b| b.* = @truncate(rng.next());
        view_checker.check(len, HashCombine.hash(@as([]const u8, bytes[0..len])), jolt.jolt_hash_string_view(&bytes, @intCast(len)));
        jolt_string_checker.check(len, HashCombine.hash(@as([]const u8, bytes[0..len])), jolt.jolt_hash_string_jolt_string(&bytes, @intCast(len)));
    }
    try finishAll(&.{ &string_checker, &c_string_checker, &view_checker, &jolt_string_checker });
}

test "HashCombine.hash64 and Hash<T>" {
    var rng: Rng = .{};
    var hash64_checker: Checker = .{ .name = "Hash64" };
    var float_checker: Checker = .{ .name = "Hash<float>" };
    var double_checker: Checker = .{ .name = "Hash<double>" };
    var int_checker: Checker = .{ .name = "Hash<int>" };
    var uint32_checker: Checker = .{ .name = "Hash<uint32>" };
    var uint64_checker: Checker = .{ .name = "Hash<uint64>" };
    var char_checker: Checker = .{ .name = "Hash<char>" };
    for (0..iterations) |_| {
        const v64 = next64(&rng);
        hash64_checker.check(v64, HashCombine.hash64(v64), jolt.jolt_hash64(v64));
        uint64_checker.check(v64, HashCombine.hash(v64), jolt.jolt_hash_uint64(v64));

        const f = anyFloat(&rng);
        float_checker.check(f, HashCombine.hash(f), jolt.jolt_hash_float(f));
        const d = anyDouble(&rng);
        double_checker.check(d, HashCombine.hash(d), jolt.jolt_hash_double(d));

        const v32 = rng.next();
        const i: i32 = @bitCast(v32);
        int_checker.check(i, HashCombine.hash(i), jolt.jolt_hash_int(i));
        uint32_checker.check(v32, HashCombine.hash(v32), jolt.jolt_hash_uint32(v32));
        const c: u8 = @truncate(v32);
        char_checker.check(c, HashCombine.hash(c), jolt.jolt_hash_char(c));
    }

    // -0 and +0 hash the same
    float_checker.check(-0.0, HashCombine.hash(@as(f32, -0.0)), jolt.jolt_hash_float(-0.0));
    float_checker.check(-0.0, HashCombine.hash(@as(f32, -0.0)), jolt.jolt_hash_float(0.0));
    double_checker.check(-0.0, HashCombine.hash(@as(f64, -0.0)), jolt.jolt_hash_double(-0.0));
    double_checker.check(-0.0, HashCombine.hash(@as(f64, -0.0)), jolt.jolt_hash_double(0.0));

    try finishAll(&.{ &hash64_checker, &float_checker, &double_checker, &int_checker, &uint32_checker, &uint64_checker, &char_checker });
}

test "HashCombine.hashCombine sequences and hashCombineArgs" {
    var rng: Rng = .{};
    var sequence_checker: Checker = .{ .name = "HashCombine (sequence)" };
    var args_checker: Checker = .{ .name = "HashCombineArgs" };
    var types: [16]HashType = undefined;
    var values: [16]u64 = undefined;
    for (0..iterations / 4) |iteration| {
        const count = rng.intRange(usize, 0, types.len);
        var seed = next64(&rng);
        const initial_seed = seed;
        for (types[0..count], values[0..count]) |*t, *v| {
            t.* = @enumFromInt(rng.next() % 6);
            switch (t.*) {
                .float => {
                    const f = anyFloat(&rng);
                    v.* = @as(u32, @bitCast(f));
                    HashCombine.hashCombine(&seed, f);
                },
                .double => {
                    const d = anyDouble(&rng);
                    v.* = @bitCast(d);
                    HashCombine.hashCombine(&seed, d);
                },
                .int => {
                    const x = rng.next();
                    v.* = x;
                    HashCombine.hashCombine(&seed, @as(i32, @bitCast(x)));
                },
                .uint32 => {
                    const x = rng.next();
                    v.* = x;
                    HashCombine.hashCombine(&seed, x);
                },
                .uint64 => {
                    const x = next64(&rng);
                    v.* = x;
                    HashCombine.hashCombine(&seed, x);
                },
                .char => {
                    const x: u8 = @truncate(rng.next());
                    v.* = x;
                    HashCombine.hashCombine(&seed, x);
                },
            }
        }
        sequence_checker.check(.{ .iteration = iteration, .count = count }, seed, jolt.jolt_hash_combine_sequence(initial_seed, &types, &values, @intCast(count)));

        const a = anyFloat(&rng);
        const b = rng.next();
        const c: i32 = @bitCast(rng.next());
        const d = next64(&rng);
        const e = anyDouble(&rng);
        args_checker.check(.{ a, b, c, d, e }, HashCombine.hashCombineArgs(.{ a, b, c, d, e }), jolt.jolt_hash_combine_args(a, b, c, d, e));
    }
    try finishAll(&.{ &sequence_checker, &args_checker });
}

// ---------------------------------------------------------------------------------------------------------------------
// Mt19937
// ---------------------------------------------------------------------------------------------------------------------

test "Mt19937 vs std::mt19937" {
    var rng: Rng = .{};
    var checker: Checker = .{ .name = "Mt19937" };
    const count = 5000; // Several regenerations of the 624 word state
    var expected: [count]u32 = undefined;

    var seeds: [40]u32 = undefined;
    const fixed_seeds = [_]u32{ 0, 1, 2, 42, 5489, 0x7fffffff, 0x80000000, 0xfffffffe, 0xffffffff };
    @memcpy(seeds[0..fixed_seeds.len], &fixed_seeds);
    for (seeds[fixed_seeds.len..]) |*s| s.* = rng.next();

    for (seeds) |seed| {
        jolt.jolt_mt19937(seed, &expected, count);
        var random = zolt.Mt19937.init(seed);
        for (expected, 0..) |e, i|
            checker.check(.{ .seed = seed, .index = i }, random.next(), e);
    }

    // Default constructed
    jolt.jolt_mt19937_default(&expected, count);
    var random = zolt.Mt19937.init(zolt.Mt19937.default_seed);
    for (expected, 0..) |e, i|
        checker.check(.{ .seed = zolt.Mt19937.default_seed, .index = i }, random.next(), e);

    try checker.finish();
}

// ---------------------------------------------------------------------------------------------------------------------
// LinearCurve
// ---------------------------------------------------------------------------------------------------------------------

const max_curve_points = 20;

/// Random curve: points with duplicate x values (quantized) or continuous x, in random order
fn randomCurvePoints(rng: *Rng, points: *[max_curve_points][2]f32) u32 {
    const count = rng.intRange(u32, 0, max_curve_points);
    const quantized = rng.next() % 2 == 0;
    for (points[0..count]) |*p| {
        p[0] = if (quantized) @as(f32, @floatFromInt(rng.intRange(i32, -5, 5))) * 0.5 else rng.float(-10, 10);
        p[1] = rng.float(-100, 100);
    }
    return count;
}

fn makeCurve(points: []const [2]f32) !LinearCurve {
    var curve: LinearCurve = .{};
    errdefer curve.deinit(allocator);
    for (points) |p|
        try curve.addPoint(allocator, p[0], p[1]);
    return curve;
}

/// X values to sample: around the curve's range, exactly at the points, and special values
fn sampleX(rng: *Rng, points: []const [2]f32) f32 {
    return switch (rng.next() % 8) {
        0 => if (points.len > 0) points[rng.next() % points.len][0] else 0.0,
        1 => ([_]f32{ 0.0, -0.0, std.math.inf(f32), -std.math.inf(f32), std.math.nan(f32), 1.0e30, -1.0e30 })[rng.next() % 7],
        else => rng.float(-12, 12),
    };
}

test "LinearCurve.sort / getValue / getMinX / getMaxX" {
    var rng: Rng = .{};
    var sort_checker: Checker = .{ .name = "LinearCurve.Sort" };
    var value_checker: Checker = .{ .name = "LinearCurve.GetValue" };
    var unsorted_value_checker: Checker = .{ .name = "LinearCurve.GetValue (unsorted points)" };
    var min_max_checker: Checker = .{ .name = "LinearCurve.GetMinX / GetMaxX" };

    var points: [max_curve_points][2]f32 = undefined;
    const num_x = 64;
    var xs: [num_x]f32 = undefined;
    var expected_y: [num_x]f32 = undefined;
    for (0..iterations / num_x) |iteration| {
        const count = randomCurvePoints(&rng, &points);

        // Sample the unsorted curve (exercises the exact steps of std::lower_bound)
        var curve = try makeCurve(points[0..count]);
        defer curve.deinit(allocator);
        for (&xs) |*x| x.* = sampleX(&rng, points[0..count]);
        jolt.jolt_linear_curve_get_values(@ptrCast(&points), count, &xs, &expected_y, num_x);
        for (xs, expected_y) |x, y|
            unsorted_value_checker.check(.{ .iteration = iteration, .x = x }, curve.getValue(x), y);

        // Sort: the order of points with equal x must be the same
        curve.sort();
        jolt.jolt_linear_curve_sort(@ptrCast(&points), count);
        for (curve.points.items, points[0..count], 0..) |p, expected, i|
            sort_checker.check(.{ .iteration = iteration, .index = i }, [2]f32{ p.x, p.y }, expected);

        // Sample the sorted curve
        for (&xs) |*x| x.* = sampleX(&rng, points[0..count]);
        jolt.jolt_linear_curve_get_values(@ptrCast(&points), count, &xs, &expected_y, num_x);
        for (xs, expected_y) |x, y|
            value_checker.check(.{ .iteration = iteration, .x = x }, curve.getValue(x), y);

        var expected_min_max: [2]f32 = undefined;
        jolt.jolt_linear_curve_min_max(@ptrCast(&points), count, &expected_min_max);
        min_max_checker.check(iteration, [2]f32{ curve.getMinX(), curve.getMaxX() }, expected_min_max);
    }
    try finishAll(&.{ &sort_checker, &value_checker, &unsorted_value_checker, &min_max_checker });
}

test "LinearCurve.saveBinaryState / restoreBinaryState" {
    var rng: Rng = .{};
    var save_checker: Checker = .{ .name = "LinearCurve.SaveBinaryState" };
    var restore_checker: Checker = .{ .name = "LinearCurve.RestoreBinaryState" };

    var points: [max_curve_points][2]f32 = undefined;
    var jolt_bytes: [4 + max_curve_points * 8]u8 = undefined;
    for (0..2000) |iteration| {
        const count = randomCurvePoints(&rng, &points);
        for (points[0..count]) |*p| p.* = .{ anyFloat(&rng), anyFloat(&rng) };

        var curve = try makeCurve(points[0..count]);
        defer curve.deinit(allocator);

        // Save
        var out: std.Io.Writer.Allocating = .init(allocator);
        defer out.deinit();
        var out_wrapper: zolt.StreamOutWrapper = .init(&out.writer);
        curve.saveBinaryState(out_wrapper.streamOut());
        const size = jolt.jolt_linear_curve_save(@ptrCast(&points), count, &jolt_bytes, jolt_bytes.len);
        checkBytes(&save_checker, iteration, out.written(), jolt_bytes[0..size]);

        // Restore, also from truncated data
        for (0..size + 1) |truncated_size| {
            if (truncated_size != size and rng.next() % 4 != 0)
                continue;
            var reader: std.Io.Reader = .fixed(jolt_bytes[0..truncated_size]);
            var in_wrapper: zolt.StreamInWrapper = .init(&reader);
            var restored: LinearCurve = .{};
            defer restored.deinit(allocator);
            try restored.restoreBinaryState(allocator, in_wrapper.streamIn());

            var expected_points: [max_curve_points][2]f32 = undefined;
            var expected_eof: bool = undefined;
            var expected_failed: bool = undefined;
            const expected_count = jolt.jolt_linear_curve_restore(&jolt_bytes, @intCast(truncated_size), @ptrCast(&expected_points), max_curve_points, &expected_eof, &expected_failed);
            const input = .{ .iteration = iteration, .size = truncated_size };
            restore_checker.check(input, .{ restored.points.items.len, in_wrapper.isEOF(), in_wrapper.isFailed() }, .{ expected_count, expected_eof, expected_failed });
            if (restored.points.items.len == expected_count)
                for (restored.points.items, expected_points[0..expected_count]) |p, expected|
                    restore_checker.check(input, [2]f32{ p.x, p.y }, expected);
        }
    }
    try finishAll(&.{ &save_checker, &restore_checker });
}

/// Compare two byte strings, reports the length or the first byte that differs
fn checkBytes(checker: *Checker, input: anytype, zolt_bytes: []const u8, jolt_bytes: []const u8) void {
    if (zolt_bytes.len != jolt_bytes.len) {
        checker.check(.{ .input = input, .what = "length" }, zolt_bytes.len, jolt_bytes.len);
        return;
    }
    if (std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes)) |offset|
        checker.check(.{ .input = input, .offset = offset }, zolt_bytes[offset], jolt_bytes[offset]);
}

// ---------------------------------------------------------------------------------------------------------------------
// StreamIn / StreamOut / StreamWrapper
// ---------------------------------------------------------------------------------------------------------------------

fn randomStreamSample(rng: *Rng) StreamSample {
    var s: StreamSample = std.mem.zeroes(StreamSample);
    s.u8_value = @truncate(rng.next());
    s.u16_value = @truncate(rng.next());
    s.u32_value = rng.next();
    s.u64_value = next64(rng);
    s.i32_value = @bitCast(rng.next());
    s.float_value = anyFloat(rng);
    s.double_value = anyDouble(rng);
    s.bool_value = rng.next() % 2 == 0;
    s.enum_value = @truncate(rng.next() % 2);
    for (&s.vec3) |*v| v.* = anyFloat(rng);
    for (&s.vec4) |*v| v.* = anyFloat(rng);
    for (&s.quat) |*v| v.* = anyFloat(rng);
    for (&s.mat44) |*v| v.* = anyFloat(rng);
    for (&s.float3) |*v| v.* = anyFloat(rng);
    for (&s.dvec3) |*v| v.* = anyDouble(rng);
    for (&s.dmat44_cols) |*v| v.* = anyFloat(rng);
    for (&s.dmat44_t) |*v| v.* = anyDouble(rng);
    s.float_array_len = rng.intRange(u32, 0, 8);
    for (s.float_array[0..s.float_array_len]) |*v| v.* = anyFloat(rng);
    s.vec3_array_len = rng.intRange(u32, 0, 4);
    for (s.vec3_array[0..s.vec3_array_len]) |*a| for (a) |*v| {
        v.* = anyFloat(rng);
    };
    s.dvec3_array_len = rng.intRange(u32, 0, 3);
    for (s.dvec3_array[0..s.dvec3_array_len]) |*a| for (a) |*v| {
        v.* = anyDouble(rng);
    };
    s.dmat44_array_len = rng.intRange(u32, 0, 2);
    for (0..s.dmat44_array_len) |i| {
        for (&s.dmat44_array_cols[i]) |*v| v.* = anyFloat(rng);
        for (&s.dmat44_array_t[i]) |*v| v.* = anyDouble(rng);
    }
    s.string_len = rng.intRange(u32, 0, 16);
    for (s.string[0..s.string_len]) |*c| c.* = @truncate(rng.next());
    s.point_array_len = rng.intRange(u32, 0, 4);
    for (s.point_array[0..s.point_array_len]) |*p| p.* = .{ anyFloat(rng), anyFloat(rng) };
    s.curve_len = rng.intRange(u32, 0, 4);
    for (s.curve[0..s.curve_len]) |*p| p.* = .{ anyFloat(rng), anyFloat(rng) };
    return s;
}

fn vec4(v: [4]f32) Vec4 {
    return Vec4.init(v[0], v[1], v[2], v[3]);
}

fn dmat44(cols: [12]f32, t: [3]f64) DMat44 {
    return DMat44.init(vec4(cols[0..4].*), vec4(cols[4..8].*), vec4(cols[8..12].*), DVec3.init(t[0], t[1], t[2]));
}

fn storeVec4(v: Vec4) [4]f32 {
    return v.value;
}

fn storeDMat44(m: DMat44, cols: *[12]f32, t: *[3]f64) void {
    for (0..3) |c| cols[4 * c ..][0..4].* = storeVec4(m.getColumn4(@intCast(c)));
    const translation = m.getTranslation();
    t.* = .{ translation.getX(), translation.getY(), translation.getZ() };
}

fn writePointYX(_: void, point: *const Point, stream: StreamOut) void {
    stream.write(point.y);
    stream.write(point.x);
}

fn readPointYX(_: void, stream: StreamIn, point: *Point) std.mem.Allocator.Error!void {
    stream.read(&point.y);
    stream.read(&point.x);
}

/// Zolt version of jolt_stream_write_sample, caller owns the returned memory
fn zoltWriteSample(s: *const StreamSample) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var wrapper: zolt.StreamOutWrapper = .init(&out.writer);
    const stream = wrapper.streamOut();

    stream.write(s.u8_value);
    stream.write(s.u16_value);
    stream.write(s.u32_value);
    stream.write(s.u64_value);
    stream.write(s.i32_value);
    stream.write(s.float_value);
    stream.write(s.double_value);
    stream.write(s.bool_value);
    stream.write(@as(StreamEnum, @enumFromInt(s.enum_value)));
    stream.write(Vec3.init(s.vec3[0], s.vec3[1], s.vec3[2]));
    stream.write(vec4(s.vec4));
    stream.write(Quat.fromVec4(vec4(s.quat)));
    stream.write(Mat44.init(vec4(s.mat44[0..4].*), vec4(s.mat44[4..8].*), vec4(s.mat44[8..12].*), vec4(s.mat44[12..16].*)));
    stream.write(Float3{ .x = s.float3[0], .y = s.float3[1], .z = s.float3[2] });
    stream.write(DVec3.init(s.dvec3[0], s.dvec3[1], s.dvec3[2]));
    stream.write(dmat44(s.dmat44_cols, s.dmat44_t));

    stream.writeArray(f32, s.float_array[0..s.float_array_len]);

    var vec3_array: [4]Vec3 = undefined;
    for (vec3_array[0..s.vec3_array_len], s.vec3_array[0..s.vec3_array_len]) |*v, a| v.* = Vec3.init(a[0], a[1], a[2]);
    stream.writeArray(Vec3, vec3_array[0..s.vec3_array_len]);

    var dvec3_array: [3]DVec3 = undefined;
    for (dvec3_array[0..s.dvec3_array_len], s.dvec3_array[0..s.dvec3_array_len]) |*v, a| v.* = DVec3.init(a[0], a[1], a[2]);
    stream.writeArray(DVec3, dvec3_array[0..s.dvec3_array_len]);

    var dmat44_array: [2]DMat44 = undefined;
    for (dmat44_array[0..s.dmat44_array_len], 0..) |*m, i| m.* = dmat44(s.dmat44_array_cols[i], s.dmat44_array_t[i]);
    stream.writeArray(DMat44, dmat44_array[0..s.dmat44_array_len]);

    stream.writeString(s.string[0..s.string_len]);

    var point_array: [4]Point = undefined;
    for (point_array[0..s.point_array_len], s.point_array[0..s.point_array_len]) |*p, a| p.* = .{ .x = a[0], .y = a[1] };
    stream.writeArrayWith(Point, point_array[0..s.point_array_len], {}, writePointYX);

    var curve = try makeCurve(s.curve[0..s.curve_len]);
    defer curve.deinit(allocator);
    curve.saveBinaryState(stream);

    return out.toOwnedSlice();
}

fn streamOk(stream: StreamIn) bool {
    return !stream.isEOF() and !stream.isFailed();
}

/// Zolt version of jolt_stream_read_sample
fn zoltReadSample(bytes: []const u8, s: *StreamSample, status: *[num_stream_fields]u8) !void {
    var reader: std.Io.Reader = .fixed(bytes);
    var wrapper: zolt.StreamInWrapper = .init(&reader);
    const stream = wrapper.streamIn();

    s.* = std.mem.zeroes(StreamSample);
    var field: usize = 0;
    const Status = struct {
        fn record(in: StreamIn, out: *[num_stream_fields]u8, index: *usize) void {
            out[index.*] = @as(u8, @intFromBool(in.isEOF())) | (@as(u8, @intFromBool(in.isFailed())) << 1);
            index.* += 1;
        }
    };

    stream.read(&s.u8_value);
    Status.record(stream, status, &field);
    stream.read(&s.u16_value);
    Status.record(stream, status, &field);
    stream.read(&s.u32_value);
    Status.record(stream, status, &field);
    stream.read(&s.u64_value);
    Status.record(stream, status, &field);
    stream.read(&s.i32_value);
    Status.record(stream, status, &field);
    stream.read(&s.float_value);
    Status.record(stream, status, &field);
    stream.read(&s.double_value);
    Status.record(stream, status, &field);
    stream.read(&s.bool_value);
    Status.record(stream, status, &field);

    var e: StreamEnum = .a;
    stream.read(&e);
    Status.record(stream, status, &field);
    s.enum_value = @intFromEnum(e);

    var v3 = Vec3.zero();
    stream.read(&v3);
    Status.record(stream, status, &field);
    s.vec3 = .{ v3.getX(), v3.getY(), v3.getZ() };

    var v4 = Vec4.zero();
    stream.read(&v4);
    Status.record(stream, status, &field);
    s.vec4 = storeVec4(v4);

    var q = Quat.init(0, 0, 0, 0);
    stream.read(&q);
    Status.record(stream, status, &field);
    s.quat = storeVec4(q.getXYZW());

    var m44 = Mat44.zero();
    stream.read(&m44);
    Status.record(stream, status, &field);
    for (0..4) |c| s.mat44[4 * c ..][0..4].* = storeVec4(m44.getColumn4(@intCast(c)));

    var f3: Float3 = .{ .x = 0, .y = 0, .z = 0 };
    stream.read(&f3);
    Status.record(stream, status, &field);
    s.float3 = .{ f3.x, f3.y, f3.z };

    var dv3 = DVec3.zero();
    stream.read(&dv3);
    Status.record(stream, status, &field);
    s.dvec3 = .{ dv3.getX(), dv3.getY(), dv3.getZ() };

    var dm44 = DMat44.zero();
    stream.read(&dm44);
    Status.record(stream, status, &field);
    storeDMat44(dm44, &s.dmat44_cols, &s.dmat44_t);

    var float_array: std.ArrayList(f32) = .empty;
    defer float_array.deinit(allocator);
    try stream.readArray(f32, allocator, &float_array);
    Status.record(stream, status, &field);
    s.float_array_len = @intCast(float_array.items.len);
    if (streamOk(stream) and float_array.items.len <= 8)
        @memcpy(s.float_array[0..float_array.items.len], float_array.items);

    var vec3_array: std.ArrayList(Vec3) = .empty;
    defer vec3_array.deinit(allocator);
    try stream.readArray(Vec3, allocator, &vec3_array);
    Status.record(stream, status, &field);
    s.vec3_array_len = @intCast(vec3_array.items.len);
    if (streamOk(stream) and vec3_array.items.len <= 4)
        for (vec3_array.items, 0..) |v, i| {
            s.vec3_array[i] = .{ v.getX(), v.getY(), v.getZ() };
        };

    var dvec3_array: std.ArrayList(DVec3) = .empty;
    defer dvec3_array.deinit(allocator);
    try stream.readArray(DVec3, allocator, &dvec3_array);
    Status.record(stream, status, &field);
    s.dvec3_array_len = @intCast(dvec3_array.items.len);
    if (streamOk(stream) and dvec3_array.items.len <= 3)
        for (dvec3_array.items, 0..) |v, i| {
            s.dvec3_array[i] = .{ v.getX(), v.getY(), v.getZ() };
        };

    var dmat44_array: std.ArrayList(DMat44) = .empty;
    defer dmat44_array.deinit(allocator);
    try stream.readArray(DMat44, allocator, &dmat44_array);
    Status.record(stream, status, &field);
    s.dmat44_array_len = @intCast(dmat44_array.items.len);
    if (streamOk(stream) and dmat44_array.items.len <= 2)
        for (dmat44_array.items, 0..) |m, i| storeDMat44(m, &s.dmat44_array_cols[i], &s.dmat44_array_t[i]);

    var string: []u8 = &.{};
    defer allocator.free(string);
    try stream.readString(allocator, &string);
    Status.record(stream, status, &field);
    s.string_len = @intCast(string.len);
    if (string.len <= 16)
        @memcpy(s.string[0..string.len], string);

    var point_array: std.ArrayList(Point) = .empty;
    defer point_array.deinit(allocator);
    try stream.readArrayWith(Point, allocator, &point_array, {}, readPointYX);
    Status.record(stream, status, &field);
    s.point_array_len = @intCast(point_array.items.len);
    if (point_array.items.len <= 4)
        for (point_array.items, 0..) |p, i| {
            s.point_array[i] = .{ p.x, p.y };
        };

    var curve: LinearCurve = .{};
    defer curve.deinit(allocator);
    try curve.restoreBinaryState(allocator, stream);
    Status.record(stream, status, &field);
    s.curve_len = @intCast(curve.points.items.len);
    if (curve.points.items.len <= 4)
        for (curve.points.items, 0..) |p, i| {
            s.curve[i] = .{ p.x, p.y };
        };

    std.debug.assert(field == num_stream_fields);
}

/// Clear the values that are not defined after a failed read: Jolt reads a DMat44 into uninitialized locals and
/// leaves new elements of Array<float / Vec3 / DVec3 / DMat44> uninitialized
fn maskUndefinedValues(s: *StreamSample, status: *const [num_stream_fields]u8) void {
    if (status[@intFromEnum(StreamField.dmat44)] != 0) {
        s.dmat44_cols = @splat(0);
        s.dmat44_t = @splat(0);
    }
    if (status[@intFromEnum(StreamField.float_array)] != 0) s.float_array = @splat(0);
    if (status[@intFromEnum(StreamField.vec3_array)] != 0) s.vec3_array = @splat(@splat(0));
    if (status[@intFromEnum(StreamField.dvec3_array)] != 0) s.dvec3_array = @splat(@splat(0));
    if (status[@intFromEnum(StreamField.dmat44_array)] != 0) {
        s.dmat44_array_cols = @splat(@splat(0));
        s.dmat44_array_t = @splat(@splat(0));
    }
}

test "StreamOut / StreamIn / StreamWrapper bytes" {
    var rng: Rng = .{};
    var write_checker: Checker = .{ .name = "StreamOut (bytes)" };
    var read_checker: Checker = .{ .name = "StreamIn (values)" };
    var status_checker: Checker = .{ .name = "StreamIn (IsEOF / IsFailed after each field)" };

    var jolt_bytes: [2048]u8 = undefined;
    for (0..1000) |iteration| {
        const sample = randomStreamSample(&rng);

        // Write
        const zolt_bytes = try zoltWriteSample(&sample);
        defer allocator.free(zolt_bytes);
        const size = jolt.jolt_stream_write_sample(&sample, &jolt_bytes, jolt_bytes.len);
        try std.testing.expect(size <= jolt_bytes.len);
        checkBytes(&write_checker, iteration, zolt_bytes, jolt_bytes[0..size]);

        // Read the complete data and truncated data (every truncation for the first samples, a few random ones for the rest)
        const num_truncations: usize = if (iteration < 50) size + 1 else 8;
        for (0..num_truncations) |t| {
            const truncated_size: u32 = if (iteration < 50) @intCast(t) else if (t == 0) size else rng.intRange(u32, 0, size);

            var zolt_sample: StreamSample = undefined;
            var zolt_status: [num_stream_fields]u8 = @splat(0);
            try zoltReadSample(jolt_bytes[0..truncated_size], &zolt_sample, &zolt_status);

            var jolt_sample: StreamSample = undefined;
            var jolt_status: [num_stream_fields]u8 = @splat(0);
            jolt.jolt_stream_read_sample(&jolt_bytes, truncated_size, &jolt_sample, &jolt_status);

            const input = .{ .iteration = iteration, .size = truncated_size };
            status_checker.check(input, zolt_status, jolt_status);
            maskUndefinedValues(&zolt_sample, &jolt_status);
            maskUndefinedValues(&jolt_sample, &jolt_status);
            read_checker.check(input, zolt_sample, jolt_sample);
        }
    }
    try finishAll(&.{ &write_checker, &read_checker, &status_checker });
}

// ---------------------------------------------------------------------------------------------------------------------
// StringTools
// ---------------------------------------------------------------------------------------------------------------------

/// Compare a string produced by Zolt with the one from Jolt (`jolt_len` characters in `jolt_chars`)
fn checkString(checker: *Checker, input: anytype, zolt_string: []const u8, jolt_chars: []const u8, jolt_len: u32) void {
    if (jolt_len > jolt_chars.len) {
        checker.check(.{ .input = input, .what = "jolt string too long" }, zolt_string.len, jolt_len);
        return;
    }
    if (!std.mem.eql(u8, zolt_string, jolt_chars[0..jolt_len])) {
        if (checker.mismatches < 5)
            std.debug.print("{s}: mismatch for input {any}\n  zolt: \"{s}\"\n  jolt: \"{s}\"\n", .{ checker.name, input, zolt_string, jolt_chars[0..jolt_len] });
        checker.mismatches += 1;
    }
}

test "ConvertToString" {
    var rng: Rng = .{};
    var float_checker: Checker = .{ .name = "ConvertToString(float)" };
    var double_checker: Checker = .{ .name = "ConvertToString(double)" };
    var int_checker: Checker = .{ .name = "ConvertToString(int)" };
    var uint64_checker: Checker = .{ .name = "ConvertToString(uint64)" };
    var chars: [64]u8 = undefined;
    for (0..iterations / 10) |_| {
        // NaN is skipped: whether a negative NaN is printed as "-nan" depends on the C library
        const f = anyFloat(&rng);
        if (!std.math.isNan(f)) {
            const s = try zolt.convertToString(allocator, f);
            defer allocator.free(s);
            checkString(&float_checker, f, s, &chars, jolt.jolt_convert_to_string_float(f, &chars, chars.len));
        }

        const d = anyDouble(&rng);
        if (!std.math.isNan(d)) {
            const s = try zolt.convertToString(allocator, d);
            defer allocator.free(s);
            checkString(&double_checker, d, s, &chars, jolt.jolt_convert_to_string_double(d, &chars, chars.len));
        }

        const i: i32 = @bitCast(rng.next());
        {
            const s = try zolt.convertToString(allocator, i);
            defer allocator.free(s);
            checkString(&int_checker, i, s, &chars, jolt.jolt_convert_to_string_int(i, &chars, chars.len));
        }

        const u = next64(&rng);
        {
            const s = try zolt.convertToString(allocator, u);
            defer allocator.free(s);
            checkString(&uint64_checker, u, s, &chars, jolt.jolt_convert_to_string_uint64(u, &chars, chars.len));
        }
    }
    try finishAll(&.{ &float_checker, &double_checker, &int_checker, &uint64_checker });
}

test "ToLower" {
    var checker: Checker = .{ .name = "ToLower" };
    var all_chars: [256]u8 = undefined;
    for (&all_chars, 0..) |*c, i| c.* = @intCast(i);
    const s = try zolt.toLower(allocator, &all_chars);
    defer allocator.free(s);
    var chars: [256]u8 = undefined;
    checkString(&checker, "all characters", s, &chars, jolt.jolt_to_lower(&all_chars, all_chars.len, &chars, chars.len));
    try checker.finish();
}

/// Random string over a small alphabet, so that searches and delimiters match often
fn randomString(rng: *Rng, buffer: []u8, min_len: usize) []u8 {
    const alphabet = "ab,.";
    const len = rng.intRange(usize, min_len, buffer.len);
    for (buffer[0..len]) |*c| c.* = alphabet[rng.next() % alphabet.len];
    return buffer[0..len];
}

test "StringReplace" {
    var rng: Rng = .{};
    var checker: Checker = .{ .name = "StringReplace" };
    var string_buffer: [32]u8 = undefined;
    var search_buffer: [3]u8 = undefined;
    var replace_buffer: [4]u8 = undefined;
    var chars: [256]u8 = undefined;
    for (0..iterations / 10) |iteration| {
        const string = randomString(&rng, &string_buffer, 0);
        const search = randomString(&rng, &search_buffer, 1);
        const replace = randomString(&rng, &replace_buffer, 0);

        var s = try allocator.dupe(u8, string);
        defer allocator.free(s);
        try zolt.stringReplace(allocator, &s, search, replace);
        const len = jolt.jolt_string_replace(string.ptr, @intCast(string.len), search.ptr, @intCast(search.len), replace.ptr, @intCast(replace.len), &chars, chars.len);
        checkString(&checker, .{ .iteration = iteration, .string = string, .search = search, .replace = replace }, s, &chars, len);
    }
    try checker.finish();
}

test "StringToVector / VectorToString" {
    var rng: Rng = .{};
    var to_vector_checker: Checker = .{ .name = "StringToVector" };
    var to_string_checker: Checker = .{ .name = "VectorToString" };
    var string_buffer: [32]u8 = undefined;
    var delimiter_buffer: [3]u8 = undefined;
    var lengths: [64]u32 = undefined;
    var chars: [256]u8 = undefined;
    for (0..iterations / 10) |iteration| {
        const string = randomString(&rng, &string_buffer, 0);
        const delimiter = randomString(&rng, &delimiter_buffer, 1);
        const clear_vector = rng.next() % 2 == 0;
        const num_initial = rng.intRange(u32, 0, 2);

        // StringToVector
        var vector: std.ArrayList([]u8) = .empty;
        defer {
            for (vector.items) |item| allocator.free(item);
            vector.deinit(allocator);
        }
        for (0..num_initial) |_| try vector.append(allocator, try allocator.dupe(u8, "x"));
        try zolt.stringToVector(allocator, string, &vector, .{ .delimiter = delimiter, .clear_vector = clear_vector });
        const count = jolt.jolt_string_to_vector(string.ptr, @intCast(string.len), delimiter.ptr, @intCast(delimiter.len), clear_vector, num_initial, &lengths, lengths.len, &chars, chars.len);
        const input = .{ .iteration = iteration, .string = string, .delimiter = delimiter, .clear_vector = clear_vector, .num_initial = num_initial };
        if (vector.items.len != count) {
            to_vector_checker.check(input, vector.items.len, count);
        } else {
            var offset: usize = 0;
            for (vector.items, lengths[0..count]) |item, len| {
                checkString(&to_vector_checker, input, item, chars[offset..], len);
                offset += len;
            }
        }

        // VectorToString on the result (contains empty strings, which take a special path)
        const joined = try zolt.vectorToString(allocator, vector.items, .{ .delimiter = delimiter });
        defer allocator.free(joined);
        var all_chars: std.ArrayList(u8) = .empty;
        defer all_chars.deinit(allocator);
        for (vector.items, 0..) |item, i| {
            try all_chars.appendSlice(allocator, item);
            lengths[i] = @intCast(item.len);
        }
        var joined_chars: [256]u8 = undefined;
        const joined_len = jolt.jolt_vector_to_string(&lengths, @intCast(vector.items.len), all_chars.items.ptr, delimiter.ptr, @intCast(delimiter.len), &joined_chars, joined_chars.len);
        checkString(&to_string_checker, input, joined, &joined_chars, joined_len);
    }
    try finishAll(&.{ &to_vector_checker, &to_string_checker });
}
