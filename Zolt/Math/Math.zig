//! Port of: Jolt/Math/Math.h
//! Status: complete
//!
//! Scalar helpers. Semantics follow Jolt built with JPH_CROSS_PLATFORM_DETERMINISTIC
//! (see Docs/Zolt/PortingGuide.md, "Floating point determinism").

const std = @import("std");

/// The constant pi (JPH_PI)
pub const pi: f32 = 3.14159265358979323846;

/// A large floating point value which, when squared, is still much smaller than FLT_MAX (cLargeFloat)
pub const large_float: f32 = 1.0e15;

/// Smallest positive normal f32 (FLT_MIN)
pub const flt_min: f32 = std.math.floatMin(f32);

/// Largest f32 (FLT_MAX)
pub const flt_max: f32 = std.math.floatMax(f32);

/// Machine epsilon for f32 (FLT_EPSILON)
pub const flt_epsilon: f32 = std.math.floatEps(f32);

/// Convert a value from degrees to radians
pub fn degreesToRadians(v: f32) f32 {
    return v * (pi / 180.0);
}

/// Convert a value from radians to degrees
pub fn radiansToDegrees(v: f32) f32 {
    return v * (180.0 / pi);
}

/// Convert angle in radians to the range [-pi, pi]
pub fn centerAngleAroundZero(v_in: f32) f32 {
    var v = v_in;
    if (v < -pi) {
        while (true) {
            v += 2.0 * pi;
            if (!(v < -pi)) break;
        }
    } else if (v > pi) {
        while (true) {
            v -= 2.0 * pi;
            if (!(v > pi)) break;
        }
    }
    std.debug.assert(v >= -pi and v <= pi);
    return v;
}

/// Calculates a * b - c * d with higher accuracy when fused multiply add instructions are available.
/// Zolt never uses FMA (it would break cross platform determinism), so this is the plain formula.
/// See: https://pharr.org/matt/blog/2019/11/03/difference-of-floats
pub fn differenceOfProducts(a: f32, b: f32, c: f32, d: f32) f32 {
    return a * b - c * d;
}

/// C++ `std::min(a, b)`: returns `a` unless `b < a`. Use this instead of `@min` for floats when porting
/// Jolt code, `@min` treats NaN and -0.0 differently which can break bit exact results.
pub fn min(a: anytype, b: @TypeOf(a)) @TypeOf(a) {
    return if (b < a) b else a;
}

/// C++ `std::max(a, b)`: returns `a` unless `a < b`. See `min`.
pub fn max(a: anytype, b: @TypeOf(a)) @TypeOf(a) {
    return if (a < b) b else a;
}

/// Clamp a value between two values (Clamp)
pub fn clamp(v: anytype, lo: @TypeOf(v), hi: @TypeOf(v)) @TypeOf(v) {
    return min(max(v, lo), hi);
}

/// Square a value
pub fn square(v: anytype) @TypeOf(v) {
    return v * v;
}

/// Returns v^3
pub fn cubed(v: anytype) @TypeOf(v) {
    return v * v * v;
}

/// Take the square root of an f32 or f64 (Sqrt). IEEE correctly rounded on every platform.
pub fn sqrt(v: anytype) @TypeOf(v) {
    return @sqrt(v);
}

/// Calculates approximate a / sqrt(b).
/// In cross platform deterministic mode this is the exact division, which is what Zolt always uses.
/// Note that b must be bigger or equal than FLT_MIN, otherwise the result can be Inf.
pub fn mulRSqrtApproximate(a: f32, b: f32) f32 {
    std.debug.assert(b >= flt_min);
    return a / @sqrt(b);
}

/// Get the sign of a value, returns -1 when negative and 1 otherwise (including for -0)
pub fn sign(v: anytype) @TypeOf(v) {
    return if (v < 0) -1 else 1;
}

/// Check if v is a power of 2
pub fn isPowerOf2(v: anytype) bool {
    return v > 0 and (v & (v - 1)) == 0;
}

/// Align v up to the next `alignment` bytes. Works on integers and pointers.
pub fn alignUp(v: anytype, alignment: u64) @TypeOf(v) {
    std.debug.assert(isPowerOf2(alignment));
    const T = @TypeOf(v);
    if (@typeInfo(T) == .pointer) {
        return @ptrFromInt((@intFromPtr(v) + alignment - 1) & ~(alignment - 1));
    } else {
        return @intCast((@as(u64, @intCast(v)) + alignment - 1) & ~(alignment - 1));
    }
}

/// Check if v is `alignment` aligned. Works on integers and pointers.
pub fn isAligned(v: anytype, alignment: u64) bool {
    std.debug.assert(isPowerOf2(alignment));
    const as_int: u64 = if (@typeInfo(@TypeOf(v)) == .pointer) @intFromPtr(v) else @intCast(v);
    return (as_int & (alignment - 1)) == 0;
}

/// Compute number of trailing zero bits (how many low bits are zero), returns 32 for 0
pub fn countTrailingZeros(value: u32) u32 {
    return @ctz(value);
}

/// Compute the number of leading zero bits (how many high bits are zero), returns 32 for 0
pub fn countLeadingZeros(value: u32) u32 {
    return @clz(value);
}

/// Count the number of 1 bits in a value
pub fn countBits(value: u32) u32 {
    return @popCount(value);
}

/// Get the next higher power of 2 of a value, or the value itself if the value is already a power of 2
pub fn getNextPowerOf2(value: u32) u32 {
    return if (value <= 1) 1 else @as(u32, 1) << @intCast(32 - countLeadingZeros(value - 1));
}

test "alignUp / isAligned" {
    try std.testing.expectEqual(@as(u32, 16), alignUp(@as(u32, 9), 16));
    try std.testing.expect(isAligned(@as(usize, 32), 16));
    try std.testing.expect(!isAligned(@as(usize, 33), 16));
}
