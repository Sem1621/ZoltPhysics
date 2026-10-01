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

test "10000th output of a default constructed generator matches the C++ standard" {
    // [rand.predef]: "Required behavior: The 10000th consecutive invocation of a default-constructed object of type mt19937 shall produce the value 4123659995."
    var rng = Mt19937.init(default_seed);
    var v: u32 = 0;
    for (0..10000) |_| v = rng.next();
    try std.testing.expectEqual(@as(u32, 4123659995), v);
}
