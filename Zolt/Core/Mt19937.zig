//! Zolt addition, no Jolt file: replaces C++ `std::mt19937`.
//! Status: complete
//!
//! Jolt (HairSettings.cpp, the unit tests, PerformanceTest) uses std::mt19937 because, unlike
//! std::default_random_engine and the std distributions, its output is fully specified by the C++
//! standard. Porting it exactly keeps random scenes bit identical to the C++ version.
//!
//! Usage: `var rng = Mt19937.init(Mt19937.default_seed); const v = rng.next();`
//! Satisfies the "random bit generator" convention used by Zolt (e.g. Vec3.random): a `next()`
//! method plus `min_value` / `max_value` declarations, like a C++ UniformRandomBitGenerator.
//!
//! `shuffleLibcxx` and `uniformIntLibcxx` (Docs/Zolt/RemainingFeaturesArchitecture.md R13) are libc++'s `std::shuffle`
//! and `std::uniform_int_distribution` on this engine. The C++ standard does not specify these two: Jolt's result
//! (HairSettings::InitRenderAndSimulationStrands) depends on the standard library. Zolt follows libc++, the library
//! of the parity reference (Zig's C++ compiler), with std::mt19937's result type `uint_fast32_t` of x86-64 Linux (64
//! bits).

const std = @import("std");

const Mt19937 = @This();

const n = 624;
const m = 397;
const matrix_a: u32 = 0x9908b0df;
const upper_mask: u32 = 0x80000000;
const lower_mask: u32 = 0x7fffffff;

/// Seed used by a default constructed std::mt19937
pub const default_seed: u32 = 5489;

/// Smallest value returned by `next` (std::mt19937::min())
pub const min_value: u32 = 0;

/// Largest value returned by `next` (std::mt19937::max())
pub const max_value: u32 = std.math.maxInt(u32);

state: [n]u32,
index: u32,

/// Create a generator, equivalent to `std::mt19937 rng(seed)`
pub fn init(seed: u32) Mt19937 {
    var self: Mt19937 = .{ .state = undefined, .index = n };
    self.state[0] = seed;
    for (1..n) |i| {
        const prev = self.state[i - 1];
        self.state[i] = 1812433253 *% (prev ^ (prev >> 30)) +% @as(u32, @intCast(i));
    }
    return self;
}

/// Generate the next value (std::mt19937::operator())
pub fn next(self: *Mt19937) u32 {
    if (self.index >= n) self.twist();

    var y = self.state[self.index];
    self.index += 1;

    // Tempering
    y ^= y >> 11;
    y ^= (y << 7) & 0x9d2c5680;
    y ^= (y << 15) & 0xefc60000;
    y ^= y >> 18;
    return y;
}

fn twist(self: *Mt19937) void {
    for (0..n) |i| {
        const y = (self.state[i] & upper_mask) | (self.state[(i + 1) % n] & lower_mask);
        var v = self.state[(i + m) % n] ^ (y >> 1);
        if ((y & 1) != 0) v ^= matrix_a;
        self.state[i] = v;
    }
    self.index = 0;
}

// libc++ std::shuffle and std::uniform_int_distribution -----------------------------------------------------------------

/// libc++'s `__independent_bits_engine<std::mt19937, UInt>(engine, w)()`: `w` random bits from as few engine calls as
/// possible, each call rejected while it is at or above `y0` / `y1`. With this engine (32 random bits per call, a 64 bit
/// result type) every `w` up to 32 takes one call per attempt and is never rejected.
fn independentBits(self: *Mt19937, comptime UInt: type, w: usize) UInt {
    // _Engine_result_type is uint_fast32_t (64 bits), so _Working_result_type is 64 bits for every UInt
    const r: u64 = @as(u64, max_value) - min_value + 1; // _Rp
    const log2_r: usize = 32; // __m = __log2<_Working_result_type, _Rp>
    const working_digits: usize = 64; // _WDt
    const engine_digits: usize = 64; // _EDt
    const result_digits: usize = @bitSizeOf(UInt); // __w_rt

    // Constructor
    var count: usize = w / log2_r + @intFromBool(w % log2_r != 0); // __n_
    var w0: usize = w / count;
    var y0: u64 = if (w0 < working_digits) (r >> @intCast(w0)) << @intCast(w0) else 0;
    if (r - y0 > y0 / count) {
        count += 1;
        w0 = w / count;
        y0 = if (w0 < working_digits) (r >> @intCast(w0)) << @intCast(w0) else 0;
    }
    const count0: usize = count - w % count; // __n0_
    const y1: u64 = if (w0 < working_digits - 1) (r >> @intCast(w0 + 1)) << @intCast(w0 + 1) else 0;
    const mask0: u64 = if (w0 > 0) ~@as(u64, 0) >> @intCast(engine_digits - w0) else 0;
    const mask1: u64 = if (w0 < engine_digits - 1) ~@as(u64, 0) >> @intCast(engine_digits - (w0 + 1)) else ~@as(u64, 0);

    // __eval(true_type)
    var sp: UInt = 0;
    for (0..count0) |_| {
        var u: u64 = undefined;
        while (true) {
            u = @as(u64, self.next()) - min_value;
            if (u < y0) break;
        }
        if (w0 < result_digits)
            sp <<= @intCast(w0)
        else
            sp = 0;
        sp +%= @truncate(u & mask0);
    }
    for (count0..count) |_| {
        var u: u64 = undefined;
        while (true) {
            u = @as(u64, self.next()) - min_value;
            if (u < y1) break;
        }
        if (w0 < result_digits - 1)
            sp <<= @intCast(w0 + 1)
        else
            sp = 0;
        sp +%= @truncate(u & mask1);
    }
    return sp;
}

/// libc++'s `std::uniform_int_distribution<T>(a, b)(engine)`: a uniformly distributed integer in [a, b]. It draws the
/// smallest number of bits that can hold b - a and rejects values above b - a.
pub fn uniformIntLibcxx(self: *Mt19937, comptime T: type, a: T, b: T) T {
    // _UIntType: uint32_t for types of at most 32 bits, the unsigned type otherwise
    const UInt = if (@sizeOf(T) <= @sizeOf(u32)) u32 else std.meta.Int(.unsigned, @bitSizeOf(T));
    const digits: usize = @bitSizeOf(UInt); // __dt
    const rp: UInt = @as(UInt, @bitCast(@as(std.meta.Int(@typeInfo(T).int.signedness, digits), b))) -%
        @as(UInt, @bitCast(@as(std.meta.Int(@typeInfo(T).int.signedness, digits), a))) +% 1;
    if (rp == 1)
        return a;
    if (rp == 0)
        return fromUInt(T, self.independentBits(UInt, digits));
    var w: usize = digits - @clz(rp) - 1;
    if ((rp & (@as(UInt, std.math.maxInt(UInt)) >> @intCast(digits - w))) != 0)
        w += 1;
    var u: UInt = undefined;
    while (true) {
        u = self.independentBits(UInt, w);
        if (u < rp) break;
    }
    return fromUInt(T, u +% @as(UInt, @bitCast(@as(std.meta.Int(@typeInfo(T).int.signedness, digits), a))));
}

/// `static_cast<result_type>(u)`
fn fromUInt(comptime T: type, u: anytype) T {
    const Same = std.meta.Int(@typeInfo(T).int.signedness, @bitSizeOf(@TypeOf(u)));
    return @truncate(@as(Same, @bitCast(u)));
}

/// libc++'s `std::shuffle(items.begin(), items.end(), engine)`: for each position but the last, draw an index into the
/// rest of the range (`uniform_int_distribution<ptrdiff_t>`) and swap when it is not 0
pub fn shuffleLibcxx(self: *Mt19937, comptime T: type, items: []T) void {
    var d: isize = @intCast(items.len);
    if (d > 1) {
        var first: usize = 0;
        const last: usize = items.len - 1;
        d -= 1;
        while (first < last) : ({
            first += 1;
            d -= 1;
        }) {
            const i: isize = self.uniformIntLibcxx(isize, 0, d);
            if (i != 0)
                std.mem.swap(T, &items[first], &items[first + @as(usize, @intCast(i))]);
        }
    }
}

test "10000th output of a default constructed generator matches the C++ standard" {
    // [rand.predef]: "Required behavior: The 10000th consecutive invocation of a default-constructed object of type mt19937 shall produce the value 4123659995."
    var rng = Mt19937.init(default_seed);
    var v: u32 = 0;
    for (0..10000) |_| v = rng.next();
    try std.testing.expectEqual(@as(u32, 4123659995), v);
}

test "uniformIntLibcxx: one engine call per attempt and the rejection of libc++" {
    // The range [0, 9] needs 4 bits: a value is the low 4 bits of one engine call, values above 9 are rejected
    var rng = Mt19937.init(default_seed);
    var check = Mt19937.init(default_seed);
    for (0..1000) |_| {
        const v = rng.uniformIntLibcxx(i64, 0, 9);
        var expected: u32 = undefined;
        while (true) {
            expected = check.next() & 0xf;
            if (expected <= 9) break;
        }
        try std.testing.expectEqual(@as(i64, expected), v);
    }

    // A range of one value draws nothing
    try std.testing.expectEqual(@as(i64, 5), rng.uniformIntLibcxx(i64, 5, 5));
    try std.testing.expectEqual(check.next(), rng.next());

    // A power of two range is never rejected: [0, 255] is the low 8 bits
    try std.testing.expectEqual(@as(i64, check.next() & 0xff), rng.uniformIntLibcxx(i64, 0, 255));

    // Negative bounds, a 32 bit type and the full 64 bit range (two engine calls, high word first)
    try std.testing.expectEqual(@as(i32, @intCast(check.next() & 0x3)) - 2, rng.uniformIntLibcxx(i32, -2, 1));
    const hi: u64 = check.next();
    const lo: u64 = check.next();
    try std.testing.expectEqual(hi << 32 | lo, rng.uniformIntLibcxx(u64, 0, std.math.maxInt(u64)));
}

test "shuffleLibcxx: a permutation, deterministic, short ranges untouched" {
    var items: [100]u32 = undefined;
    for (&items, 0..) |*v, i| v.* = @intCast(i);
    var rng = Mt19937.init(default_seed);
    rng.shuffleLibcxx(u32, &items);

    // A permutation of the input
    var seen: [100]bool = @splat(false);
    for (items) |v| {
        try std.testing.expect(!seen[v]);
        seen[v] = true;
    }

    // The same seed gives the same order
    var again: [100]u32 = undefined;
    for (&again, 0..) |*v, i| v.* = @intCast(i);
    var rng2 = Mt19937.init(default_seed);
    rng2.shuffleLibcxx(u32, &again);
    try std.testing.expectEqualSlices(u32, &items, &again);

    // 0 and 1 items: no engine call
    var one = [_]u32{7};
    var rng3 = Mt19937.init(default_seed);
    rng3.shuffleLibcxx(u32, one[0..0]);
    rng3.shuffleLibcxx(u32, &one);
    try std.testing.expectEqual(@as(u32, 7), one[0]);
    try std.testing.expectEqual(Mt19937.init(default_seed).state, rng3.state);
    try std.testing.expectEqual(@as(u32, n), rng3.index);
}
