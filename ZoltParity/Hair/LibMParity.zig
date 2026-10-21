//! The "LibM and shuffle" layer of the parity (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, R13):
//! Zolt/Math/LibM.zig and Mt19937.shuffleLibcxx / uniformIntLibcxx against the reference. C ABI wrappers:
//! LibMShuffleReference.cpp (std::exp, std::pow, std::shuffle and std::uniform_int_distribution as Jolt calls them) and
//! LibMReference.c (the pin: expf and powf with the generic algorithm, which win over compiler_rt's and glibc's).
//!
//! - The pin probe: the reference's std::exp and std::pow give the generic algorithm's bits on inputs where the other
//!   implementations differ: compiler_rt's older expf (the parity executable's expf without the pin), glibc's FMA
//!   variant of expf (2 inputs of 2^32) and glibc's powf, which returns a NaN for a signaling NaN where musl returns 1.
//! - expf on a 2^20 sample of all floats and on 2^18 floats of the hair domain (-damping * dt, [-64, 0]), bit for bit.
//!   All 2^32 inputs with ZOLT_PARITY_LIBM_ALL=1.
//! - powf on [0, 1] (every 256th float) with the time ratios of Hair::InitializeContext (the exponents of
//!   Gradient::MakeStepDependent), and on 2^20 random pairs. Bit for bit, except that x + y of two NaNs may return
//!   either NaN (clang and Zig order the operands differently): then both results must be NaN.
//! - With ZOLT_PARITY_LIBM_HOST=1: LibM.zig against the host's libm.so.6 (dlopen), listing the differences (glibc's
//!   FMA variant differs on 2 expf inputs; glibc tells signaling NaNs apart in powf). It does not fail on them.
//! - The shuffle: the order and the engine draws of std::shuffle with a default seeded std::mt19937 for 0 to 2000
//!   items, and std::uniform_int_distribution for 4 integer types on random ranges (also the full 64 bit range).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const LibM = zolt.LibM;
const Mt19937 = zolt.Mt19937;

extern fn jolt_libm_linked() c_int;
extern fn jolt_libm_expf(x: [*]const f32, y: [*]f32, count: usize) void;
extern fn jolt_libm_powf(x: [*]const f32, y: [*]const f32, z: [*]f32, count: usize) void;
extern fn jolt_libcxx_shuffle(count: u32, items: [*]u32, draws: *u64, next: *[4]u32) c_int;
extern fn jolt_libcxx_uniform_int(int_type: c_int, a: i64, b: i64, seed: u32, count: u32, values: [*]i64, draws: *u64) void;
extern fn jolt_libcxx_shuffle_hooks_check() u32;

fn bits(f: f32) u32 {
    return @bitCast(f);
}

fn float(b: u32) f32 {
    return @bitCast(b);
}

/// True when the environment variable `name` is set to "1" (opt-in layers)
fn optIn(name: [*:0]const u8) bool {
    const value = std.c.getenv(name) orelse return false;
    return std.mem.eql(u8, std.mem.span(value), "1");
}

/// The reference's std::exp of `inputs`
fn referenceExpf(allocator: std.mem.Allocator, inputs: []const f32) ![]f32 {
    const out = try allocator.alloc(f32, inputs.len);
    jolt_libm_expf(inputs.ptr, out.ptr, inputs.len);
    return out;
}

/// Compare LibM.expf with the reference on `inputs`, bit for bit
fn compareExpf(checker: *fw.Checker, inputs: []const f32) !void {
    const allocator = std.testing.allocator;
    const jolt = try referenceExpf(allocator, inputs);
    defer allocator.free(jolt);
    for (inputs, jolt) |x, j| {
        const z = LibM.expf(x);
        if (bits(z) != bits(j)) {
            if (checker.mismatches < 5)
                std.debug.print("{s}: expf(0x{x:0>8}): zolt 0x{x:0>8}, jolt 0x{x:0>8}\n", .{ checker.name, bits(x), bits(z), bits(j) });
            checker.mismatches += 1;
        }
    }
}

/// Compare LibM.powf with the reference on the pairs, bit for bit (two NaN operands: both results NaN)
fn comparePowf(checker: *fw.Checker, xs: []const f32, ys: []const f32) !void {
    const allocator = std.testing.allocator;
    const jolt = try allocator.alloc(f32, xs.len);
    defer allocator.free(jolt);
    jolt_libm_powf(xs.ptr, ys.ptr, jolt.ptr, xs.len);
    for (xs, ys, jolt) |x, y, j| {
        const z = LibM.powf(x, y);
        const same = if (std.math.isNan(x) and std.math.isNan(y)) std.math.isNan(z) and std.math.isNan(j) else bits(z) == bits(j);
        if (!same) {
            if (checker.mismatches < 5)
                std.debug.print("{s}: powf(0x{x:0>8}, 0x{x:0>8}): zolt 0x{x:0>8}, jolt 0x{x:0>8}\n", .{ checker.name, bits(x), bits(y), bits(z), bits(j) });
            checker.mismatches += 1;
        }
    }
}

/// The inputs where other implementations differ from the generic algorithm, with the generic result and the other one
const ProbeExpf = struct { x: u32, generic: u32, other: u32, what: []const u8 };

const probe_expf = [_]ProbeExpf{
    .{ .x = 0x4202422f, .generic = 0x56fc9f1b, .other = 0x56fc9f1c, .what = "compiler_rt's expf and glibc's FMA variant" },
    .{ .x = 0xc27c65d9, .generic = 0x11fa2992, .other = 0x11fa2993, .what = "glibc's FMA variant" },
    .{ .x = 0x399a522f, .generic = 0x3f8009a5, .other = 0x3f8009a6, .what = "compiler_rt's expf" },
    .{ .x = 0x3b33422f, .generic = 0x3f8059c1, .other = 0x3f8059c0, .what = "compiler_rt's expf" },
    .{ .x = 0x3bade22f, .generic = 0x3f80ae59, .other = 0x3f80ae58, .what = "compiler_rt's expf" },
    .{ .x = 0x3bf5222f, .generic = 0x3f80f60d, .other = 0x3f80f60e, .what = "compiler_rt's expf" },
};

test "LibM pin probe: the reference's std::exp and std::pow are the generic algorithm" {
    // The pin object is linked
    try std.testing.expectEqual(@as(c_int, 1), jolt_libm_linked());

    // expf: the generic result on both sides, the other implementations give other bits
    var inputs: [probe_expf.len]f32 = undefined;
    for (probe_expf, 0..) |p, i| inputs[i] = float(p.x);
    var jolt: [probe_expf.len]f32 = undefined;
    jolt_libm_expf(&inputs, &jolt, inputs.len);
    var failed = false;
    for (probe_expf, jolt) |p, j| {
        try std.testing.expectEqual(p.generic, bits(LibM.expf(float(p.x))));
        if (bits(j) != p.generic) {
            std.debug.print("expf(0x{x:0>8}) of the reference: 0x{x:0>8}, the generic algorithm gives 0x{x:0>8} ({s} gives 0x{x:0>8}): the pin of LibMReference.c did not take\n", .{ p.x, bits(j), p.generic, p.what, p.other });
            failed = true;
        }
    }

    // powf: musl does not tell signaling NaNs apart (x^0 and 1^y are 1), glibc returns a NaN
    const xs = [_]f32{ float(0xff9bc154), 1.0, float(0x7f82bb67) };
    const ys = [_]f32{ 0.0, float(0x7f82bb67), -0.0 };
    var z: [xs.len]f32 = undefined;
    jolt_libm_powf(&xs, &ys, &z, xs.len);
    for (xs, ys, z) |x, y, r| {
        try std.testing.expectEqual(@as(u32, 0x3f800000), bits(LibM.powf(x, y)));
        if (bits(r) != 0x3f800000) {
            std.debug.print("powf(0x{x:0>8}, 0x{x:0>8}) of the reference: 0x{x:0>8}, the generic algorithm gives 1: the pin of LibMReference.c did not take\n", .{ bits(x), bits(y), bits(r) });
            failed = true;
        }
    }
    if (failed) return error.ParityMismatch;
}

test "LibM parity: expf on a 2^20 sample of all floats and on the hair domain" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "expf" };

    // Every 4096th bit pattern with a random offset: every exponent and both signs, NaNs and infinities
    var rng: fw.Rng = .{ .state = 0x6c1b_5e3d };
    const sample = try allocator.alloc(f32, 1 << 20);
    defer allocator.free(sample);
    for (sample, 0..) |*x, i| x.* = float(@as(u32, @intCast(i)) << 12 | (rng.next() & 0xfff));
    try compareExpf(&checker, sample);

    // The hair domain: exp(-damping * dt) with damping and dt >= 0, so [-64, 0], dense
    for (sample[0 .. 1 << 18], 0..) |*x, i| x.* = -64.0 * @as(f32, @floatFromInt(i)) / @as(f32, 1 << 18);
    try compareExpf(&checker, sample[0 .. 1 << 18]);

    // The special values
    const special = [_]f32{ 0.0, -0.0, 88.0, 88.72283, 88.72284, -103.97207, -103.97208, -88.0, std.math.inf(f32), -std.math.inf(f32), std.math.nan(f32), std.math.floatMin(f32), -std.math.floatMin(f32), std.math.floatTrueMin(f32), 1.0e-30, -1.0e-30 };
    try compareExpf(&checker, &special);
    try checker.finish();
}

/// The time ratios of Hair::InitializeContext: dt clamped to mMaxDeltaTime, round(dt * iterations per second)
/// iterations, ratio = (dt / iterations) * cDefaultIterationsPerSecond (float arithmetic like Jolt's)
fn hairTimeRatios(allocator: std.mem.Allocator) ![]f32 {
    var ratios: std.ArrayList(f32) = .empty;
    errdefer ratios.deinit(allocator);
    const max_delta_time: f32 = 1.0 / 30.0;
    const delta_times = [_]f32{ 1.0 / 30.0, 1.0 / 60.0, 1.0 / 90.0, 1.0 / 120.0, 1.0 / 144.0, 1.0 / 240.0, 0.0123, 0.007, 0.02, 0.05, 0.1, 1.0e-4 };
    const iterations_per_second = [_]u32{ 30, 60, 90, 120, 180, 240, 360, 480, 720, 1000, 1440 };
    for (delta_times) |dt| {
        for (iterations_per_second) |ips| {
            const clamped = zolt.math.min(dt, max_delta_time);
            const num_iterations: u32 = @intFromFloat(@round(clamped * @as(f32, @floatFromInt(ips))));
            const delta_time: f32 = if (num_iterations > 0) clamped / @as(f32, @floatFromInt(num_iterations)) else 0.0;
            try ratios.append(allocator, delta_time * @as(f32, @floatFromInt(zolt.HairSettings.default_iterations_per_second)));
        }
    }
    // Plain ratios too
    for ([_]f32{ 0.0, 0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0 }) |r| try ratios.append(allocator, r);
    return ratios.toOwnedSlice(allocator);
}

test "LibM parity: powf on [0, 1] with the time ratios of the hair" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "powf [0, 1] x time ratio" };
    const ratios = try hairTimeRatios(allocator);
    defer allocator.free(ratios);

    // Every 256th float in [0, 1] (1 - v of a gradient value v in [0, 1]) and the 1 - v of Jolt's default gradients
    var xs: std.ArrayList(f32) = .empty;
    defer xs.deinit(allocator);
    var b: u32 = 0;
    while (b <= 0x3f800000) : (b += 256) try xs.append(allocator, float(b));
    for ([_]f32{ 0.1, 1.0, 0.001, 0.0, 0.05, 0.01 }) |v| try xs.append(allocator, 1.0 - v);
    const ys = try allocator.alloc(f32, xs.items.len);
    defer allocator.free(ys);
    for (ratios) |ratio| {
        @memset(ys, ratio);
        try comparePowf(&checker, xs.items, ys);
    }
    try checker.finish();
}

test "LibM parity: powf on random pairs" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "powf random" };
    var rng: fw.Rng = .{ .state = 0x51f1_77a3 };
    const n = 1 << 20;
    const xs = try allocator.alloc(f32, n);
    defer allocator.free(xs);
    const ys = try allocator.alloc(f32, n);
    defer allocator.free(ys);
    for (xs, ys, 0..) |*x, *y, i| {
        // A quarter random bit patterns, the rest values in the usual ranges (both signs, integer exponents)
        x.* = switch (i % 4) {
            0 => float(rng.next()),
            1 => rng.float(0.0, 1.0),
            2 => rng.float(-100.0, 100.0),
            else => rng.float(0.0, 1.0e6),
        };
        y.* = switch ((i / 4) % 4) {
            0 => float(rng.next()),
            1 => rng.float(-8.0, 8.0),
            2 => @floatFromInt(rng.intRange(i32, -40, 40)),
            else => rng.float(-200.0, 200.0),
        };
    }
    try comparePowf(&checker, xs, ys);
    try checker.finish();
}

test "LibM parity: expf on all 2^32 inputs (opt-in: ZOLT_PARITY_LIBM_ALL=1)" {
    if (!optIn("ZOLT_PARITY_LIBM_ALL")) return error.SkipZigTest;
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "expf all" };
    const chunk = try allocator.alloc(f32, 1 << 24);
    defer allocator.free(chunk);
    var base: u64 = 0;
    while (base < (1 << 32)) : (base += chunk.len) {
        for (chunk, 0..) |*x, i| x.* = float(@intCast(base + i));
        try compareExpf(&checker, chunk);
    }
    try checker.finish();
}

test "LibM: LibM.zig against the host's libm.so.6 (opt-in: ZOLT_PARITY_LIBM_HOST=1)" {
    if (!optIn("ZOLT_PARITY_LIBM_HOST")) return error.SkipZigTest;
    const handle = std.c.dlopen("libm.so.6", .{ .NOW = true }) orelse return error.SkipZigTest;
    const host_expf: *const fn (f32) callconv(.c) f32 = @ptrCast(std.c.dlsym(handle, "expf") orelse return error.SkipZigTest);
    const host_powf: *const fn (f32, f32) callconv(.c) f32 = @ptrCast(std.c.dlsym(handle, "powf") orelse return error.SkipZigTest);

    // expf: every input
    var expf_differences: usize = 0;
    var i: u64 = 0;
    while (i < (1 << 32)) : (i += 1) {
        const x = float(@intCast(i));
        const z = LibM.expf(x);
        const h = host_expf(x);
        if (bits(z) != bits(h)) {
            expf_differences += 1;
            if (expf_differences <= 20)
                std.debug.print("host expf(0x{x:0>8}): LibM.zig 0x{x:0>8}, libm.so.6 0x{x:0>8}\n", .{ bits(x), bits(z), bits(h) });
        }
    }

    // powf: 10^8 random pairs (NaN payloads of two NaN operands do not count)
    var rng: fw.Rng = .{ .state = 0x2b7e_1516 };
    var powf_differences: usize = 0;
    for (0..100_000_000) |n| {
        const x = if (n % 2 == 0) float(rng.next()) else rng.float(0.0, 1.0);
        const y = if (n % 3 == 0) float(rng.next()) else rng.float(-16.0, 16.0);
        const z = LibM.powf(x, y);
        const h = host_powf(x, y);
        const same = if (std.math.isNan(x) and std.math.isNan(y)) std.math.isNan(z) and std.math.isNan(h) else bits(z) == bits(h);
        if (!same) {
            powf_differences += 1;
            if (powf_differences <= 20)
                std.debug.print("host powf(0x{x:0>8}, 0x{x:0>8}): LibM.zig 0x{x:0>8}, libm.so.6 0x{x:0>8}\n", .{ bits(x), bits(y), bits(z), bits(h) });
        }
    }
    std.debug.print("LibM.zig against libm.so.6: {d} expf inputs and {d} powf pairs differ\n", .{ expf_differences, powf_differences });
}

test "Shuffle parity: libc++ std::shuffle of 0 to 2000 items, the order and the engine draws" {
    const allocator = std.testing.allocator;
    var checker: fw.Checker = .{ .name = "shuffleLibcxx" };
    const jolt_items = try allocator.alloc(u32, 2000);
    defer allocator.free(jolt_items);
    const zolt_items = try allocator.alloc(u32, 2000);
    defer allocator.free(zolt_items);
    for (0..2001) |n| {
        var draws: u64 = 0;
        var next: [4]u32 = undefined;
        try std.testing.expectEqual(@as(c_int, 0), jolt_libcxx_shuffle(@intCast(n), jolt_items.ptr, &draws, &next));

        var rng = Mt19937.init(Mt19937.default_seed);
        for (zolt_items[0..n], 0..) |*v, i| v.* = @intCast(i);
        rng.shuffleLibcxx(u32, zolt_items[0..n]);
        if (!std.mem.eql(u32, zolt_items[0..n], jolt_items[0..n])) {
            if (checker.mismatches < 5)
                std.debug.print("shuffle of {d} items: the order differs at {d}\n", .{ n, std.mem.indexOfDiff(u32, zolt_items[0..n], jolt_items[0..n]).? });
            checker.mismatches += 1;
        }

        // The engine made as many draws: a fresh engine after `draws` draws continues like the shuffled one
        var zolt_next: [4]u32 = undefined;
        for (&zolt_next) |*v| v.* = rng.next();
        var counted = Mt19937.init(Mt19937.default_seed);
        for (0..draws) |_| _ = counted.next();
        var counted_next: [4]u32 = undefined;
        for (&counted_next) |*v| v.* = counted.next();
        checker.check(n, zolt_next, next);
        checker.check(n, counted_next, next);
    }
    try checker.finish();
}

test "Shuffle parity: the shuffle reference installs the hooks of ParityHooks.h again" {
    // jolt_libcxx_shuffle allocates through Jolt: it calls ParityInstallHooks() first (R1), also after an older
    // reference file installed Jolt's allocator
    try std.testing.expectEqual(@as(u32, 1), jolt_libcxx_shuffle_hooks_check());
}

test "Shuffle parity: libc++ std::uniform_int_distribution of 4 integer types" {
    var checker: fw.Checker = .{ .name = "uniformIntLibcxx" };
    var rng: fw.Rng = .{ .state = 0x0bad_cafe };
    const count = 64;
    var jolt: [count]i64 = undefined;
    for (0..4000) |round| {
        const int_type: c_int = @intCast(round % 4);
        const seed = rng.next();
        // Ranges of every width: small, a power of two, random bounds, the full range
        const width_bits: u6 = @intCast(rng.next() % 64);
        var a: i64 = undefined;
        var b: i64 = undefined;
        switch (int_type) {
            0 => {
                a = if (rng.next() % 2 == 0) 0 else @as(i64, @bitCast((@as(u64, rng.next()) << 32 | rng.next()))) >> 2;
                const span: i64 = @intCast(@min((@as(u64, 1) << width_bits) - 1 + rng.next() % 3, std.math.maxInt(i64) >> 1));
                b = a +% span;
                if (b < a) b = std.math.maxInt(i64);
            },
            1 => {
                const lo = rng.next() >> @intCast(rng.next() % 32);
                const span = (@as(u32, 0xffffffff) >> @intCast(rng.next() % 32)) +% rng.next() % 3;
                a = lo;
                b = @max(lo, lo +% span);
            },
            2 => {
                const lo: i32 = @bitCast(rng.next());
                const span: i32 = @intCast(@as(u32, 0x7fffffff) >> @intCast(rng.next() % 31));
                a = lo;
                b = @max(lo, lo +% span);
            },
            else => {
                if (round % 16 == 3) {
                    a = 0;
                    b = -1; // The full 64 bit range
                } else {
                    a = @intCast(rng.next());
                    b = a + @as(i64, @intCast((@as(u64, 1) << @intCast(width_bits % 62)) + rng.next() % 5));
                }
            },
        }
        var draws: u64 = 0;
        jolt_libcxx_uniform_int(int_type, a, b, seed, count, &jolt, &draws);

        var engine = Mt19937.init(seed);
        var zolt_values: [count]i64 = undefined;
        for (&zolt_values) |*v| v.* = switch (int_type) {
            0 => @as(i64, engine.uniformIntLibcxx(isize, @intCast(a), @intCast(b))),
            1 => @as(i64, engine.uniformIntLibcxx(u32, @intCast(a), @intCast(b))),
            2 => @as(i64, engine.uniformIntLibcxx(i32, @intCast(a), @intCast(b))),
            else => @as(i64, @bitCast(engine.uniformIntLibcxx(u64, @bitCast(a), @bitCast(b)))),
        };
        checker.check(.{ int_type, a, b, seed }, zolt_values, jolt);

        // The same number of draws
        var counted = Mt19937.init(seed);
        for (0..draws) |_| _ = counted.next();
        checker.check(.{ int_type, a, b, seed }, engine.next(), counted.next());
    }
    try checker.finish();
}
