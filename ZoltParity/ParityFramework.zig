//! Shared helpers for the parity tests (see ZoltParity/parity.zig).
//!
//! Import with `const fw = @import("../ParityFramework.zig");` from a file in a subdirectory.

const std = @import("std");

/// Default number of random inputs per parity test
pub const iterations = 100_000;

/// Deterministic input generator (xorshift32). Areas with special needs (e.g. Math) extend it locally.
pub const Rng = struct {
    state: u32 = 0x12345678,

    pub fn next(self: *Rng) u32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        return self.state;
    }

    /// Random integer in [min, max]
    pub fn intRange(self: *Rng, comptime T: type, min: T, max: T) T {
        std.debug.assert(min <= max);
        const range: u64 = @intCast(@as(i128, max) - @as(i128, min) + 1);
        const offset: u64 = (@as(u64, self.next()) << 32 | self.next()) % range;
        return @intCast(@as(i128, min) + offset);
    }

    /// Random float in [min, max)
    pub fn float(self: *Rng, min: f32, max: f32) f32 {
        const t = @as(f32, @floatFromInt(self.next() >> 8)) / 16777216.0;
        return min + (max - min) * t;
    }

    /// Array of random floats in [min, max)
    pub fn floatArray(self: *Rng, comptime n: usize, min: f32, max: f32) [n]f32 {
        var result: [n]f32 = undefined;
        for (&result) |*v| v.* = self.float(min, max);
        return result;
    }
};

/// Bitwise equality, all NaNs are considered equal (NaN payloads are not part of the determinism contract)
pub fn sameBits(a: anytype, b: @TypeOf(a)) bool {
    if (std.math.isNan(a) and std.math.isNan(b)) return true;
    const Bits = std.meta.Int(.unsigned, @bitSizeOf(@TypeOf(a)));
    return @as(Bits, @bitCast(a)) == @as(Bits, @bitCast(b));
}

/// Bitwise equality of numbers, bools, arrays and structs of them (see sameBits)
pub fn sameValue(a: anytype, b: @TypeOf(a)) bool {
    switch (@typeInfo(@TypeOf(a))) {
        .float => return sameBits(a, b),
        .int, .bool => return a == b,
        .array => {
            for (a, b) |x, y|
                if (!sameValue(x, y)) return false;
            return true;
        },
        .@"struct" => |s| {
            inline for (s.fields) |field|
                if (!sameValue(@field(a, field.name), @field(b, field.name))) return false;
            return true;
        },
        else => @compileError("unsupported result type " ++ @typeName(@TypeOf(a))),
    }
}

/// Collects mismatches of one test and reports the first few
pub const Checker = struct {
    name: []const u8,
    mismatches: usize = 0,

    pub fn check(self: *Checker, input: anytype, zolt_result: anytype, jolt_result: @TypeOf(zolt_result)) void {
        const equal = sameValue(zolt_result, jolt_result);
        if (!equal) {
            if (self.mismatches < 5)
                std.debug.print("{s}: mismatch for input {any}\n  zolt: {any}\n  jolt: {any}\n", .{ self.name, input, zolt_result, jolt_result });
            self.mismatches += 1;
        }
    }

    pub fn finish(self: *const Checker) !void {
        if (self.mismatches > 0) {
            std.debug.print("{s}: {d} mismatches\n", .{ self.name, self.mismatches });
            return error.ParityMismatch;
        }
    }
};

/// Report the result of every checker (not only the first one that fails), fails if any of them found a mismatch
pub fn finishAll(checkers: []const *const Checker) !void {
    var failed = false;
    for (checkers) |checker|
        checker.finish() catch {
            failed = true;
        };
    if (failed) return error.ParityMismatch;
}

test "Rng.intRange stays in range" {
    var rng: Rng = .{};
    for (0..1000) |_| {
        const v = rng.intRange(i32, -3, 5);
        try std.testing.expect(v >= -3 and v <= 5);
    }
}
