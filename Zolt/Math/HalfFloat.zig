//! Port of: Jolt/Math/HalfFloat.h
//! Status: complete
//!
//! The `HalfFloatConversion` namespace of Jolt is this file (re-exported as `zolt.half_float`), so
//! `HalfFloatConversion::FromFloat<HalfFloatConversion::ROUND_TO_NEAREST>(v)` becomes
//! `half_float.fromFloat(.round_to_nearest, v)`. `toFloat` uses F16C (one vcvtph2ps) when the target has it, like
//! Jolt with JPH_USE_F16C. Other targets and `fromFloat` use the portable fallback (no NEON path). Both give bit
//! identical results, except that F16C turns a signaling NaN into a quiet NaN. Jolt's x86-64-v3 build does the same.

const std = @import("std");
const builtin = @import("builtin");
const UVec4 = @import("UVec4.zig").UVec4;
const Vec4 = @import("Vec4.zig").Vec4;

/// The target converts half floats to floats in hardware (JPH_USE_F16C)
const use_f16c = (builtin.cpu.arch == .x86 or builtin.cpu.arch == .x86_64) and std.Target.x86.featureSetHas(builtin.cpu.features, .f16c);

pub const HalfFloat = u16;

// Define half float constant values
pub const half_flt_max: HalfFloat = 0x7bff;
pub const half_flt_max_negative: HalfFloat = 0xfbff;
pub const half_flt_inf: HalfFloat = 0x7c00;
pub const half_flt_inf_negative: HalfFloat = 0xfc00;
pub const half_flt_nanq: HalfFloat = 0x7e00;
pub const half_flt_nanq_negative: HalfFloat = 0xfe00;

// Layout of a float
pub const float_sign_pos = 31;
pub const float_exponent_pos = 23;
pub const float_exponent_bits = 8;
pub const float_exponent_mask = (1 << float_exponent_bits) - 1;
pub const float_exponent_bias = 127;
pub const float_mantissa_bits = 23;
pub const float_mantissa_mask = (1 << float_mantissa_bits) - 1;
pub const float_exponent_and_mantissa_mask = float_mantissa_mask + (float_exponent_mask << float_exponent_pos);

// Layout of half float
pub const half_flt_sign_pos = 15;
pub const half_flt_exponent_pos = 10;
pub const half_flt_exponent_bits = 5;
pub const half_flt_exponent_mask = (1 << half_flt_exponent_bits) - 1;
pub const half_flt_exponent_bias = 15;
pub const half_flt_mantissa_bits = 10;
pub const half_flt_mantissa_mask = (1 << half_flt_mantissa_bits) - 1;
pub const half_flt_exponent_and_mantissa_mask = half_flt_mantissa_mask + (half_flt_exponent_mask << half_flt_exponent_pos);

/// Define half-float rounding modes (ERoundingMode)
pub const RoundingMode = enum {
    /// Round to negative infinity
    round_to_neg_inf,
    /// Round to positive infinity
    round_to_pos_inf,
    /// Round to nearest value
    round_to_nearest,
};

/// Convert a float (32-bits) to a half float (16-bits), fallback version when no intrinsics available
pub fn fromFloatFallback(comptime rounding_mode: RoundingMode, v: f32) HalfFloat {
    // Reinterpret the float as an uint32
    const value: u32 = @bitCast(v);

    // Extract exponent
    const exponent: u32 = (value >> float_exponent_pos) & float_exponent_mask;

    // Extract mantissa
    var mantissa: u32 = value & float_mantissa_mask;

    // Extract the sign and move it into the right spot for the half float (so we can just or it in at the end)
    const hf_sign: HalfFloat = @as(HalfFloat, @truncate(value >> (float_sign_pos - half_flt_sign_pos))) & (1 << half_flt_sign_pos);

    // Check NaN or INF
    if (exponent == float_exponent_mask) // NaN or INF
        return hf_sign | (if (mantissa == 0) half_flt_inf else half_flt_nanq);

    // Rebias the exponent for half floats
    const rebiased_exponent: i32 = @as(i32, @intCast(exponent)) - float_exponent_bias + half_flt_exponent_bias;

    // Check overflow to infinity
    if (rebiased_exponent >= half_flt_exponent_mask) {
        const round_up = rounding_mode == .round_to_nearest or (hf_sign == 0) == (rounding_mode == .round_to_pos_inf);
        return hf_sign | (if (round_up) half_flt_inf else half_flt_max);
    }

    // Check underflow to zero
    if (rebiased_exponent < -half_flt_mantissa_bits) {
        const round_up = rounding_mode != .round_to_nearest and (hf_sign == 0) == (rounding_mode == .round_to_pos_inf) and (value & float_exponent_and_mantissa_mask) != 0;
        return hf_sign | @as(HalfFloat, if (round_up) 1 else 0);
    }

    var hf_exponent: HalfFloat = undefined;
    var shift: i32 = undefined;
    if (rebiased_exponent <= 0) {
        // Underflow to denormalized number
        hf_exponent = 0;
        mantissa |= 1 << float_mantissa_bits; // Add the implicit 1 bit to the mantissa
        shift = float_mantissa_bits - half_flt_mantissa_bits + 1 - rebiased_exponent;
    } else {
        // Normal half float
        hf_exponent = @intCast(rebiased_exponent << half_flt_exponent_pos);
        shift = float_mantissa_bits - half_flt_mantissa_bits;
    }
    const shift_amount: u5 = @intCast(shift);

    // Compose the half float
    const hf_mantissa: HalfFloat = @truncate(mantissa >> shift_amount);
    var hf: HalfFloat = hf_sign | hf_exponent | hf_mantissa;

    // Calculate the remaining bits that we're discarding
    const remainder: u32 = mantissa & ((@as(u32, 1) << shift_amount) - 1);

    if (rounding_mode == .round_to_nearest) {
        // Round to nearest
        const round_threshold: u32 = @as(u32, 1) << (shift_amount - 1);
        if (remainder > round_threshold // Above threshold, we must always round
        or (remainder == round_threshold and (hf_mantissa & 1) != 0)) // When equal, round to nearest even
            hf += 1; // May overflow to infinity
    } else {
        // Round up or down (truncate) depending on the rounding mode
        const round_up = (hf_sign == 0) == (rounding_mode == .round_to_pos_inf) and remainder != 0;
        if (round_up)
            hf += 1; // May overflow to infinity
    }

    return hf;
}

/// Convert a float (32-bits) to a half float (16-bits)
pub fn fromFloat(comptime rounding_mode: RoundingMode, v: f32) HalfFloat {
    return fromFloatFallback(rounding_mode, v);
}

/// Convert 4 half floats (lower 64 bits) to floats, fallback version when no intrinsics available
pub fn toFloatFallback(half_floats: UVec4) Vec4 {
    // Unpack half floats to 4 uint32's
    const value = half_floats.expand4Uint16Lo();

    // Normal half float path, extract the exponent and mantissa, shift them into place and update the exponent bias
    const exponent_mantissa = UVec4.bitAnd(value, UVec4.replicate(half_flt_exponent_and_mantissa_mask)).logicalShiftLeft(float_exponent_pos - half_flt_exponent_pos).add(UVec4.replicate((float_exponent_bias - half_flt_exponent_bias) << float_exponent_pos));

    // Denormalized half float path, renormalize the float
    const exponent_mantissa_denormalized = exponent_mantissa.add(UVec4.replicate(1 << float_exponent_pos)).reinterpretAsFloat().sub(UVec4.replicate((float_exponent_bias - half_flt_exponent_bias + 1) << float_exponent_pos).reinterpretAsFloat()).reinterpretAsInt();

    // NaN / INF path, set all exponent bits
    const exponent_mantissa_nan_inf = UVec4.bitOr(exponent_mantissa, UVec4.replicate(float_exponent_mask << float_exponent_pos));

    // Get the exponent to determine which of the paths we should take
    const exponent_mask = UVec4.replicate(half_flt_exponent_mask << half_flt_exponent_pos);
    const exponent = UVec4.bitAnd(value, exponent_mask);
    const is_denormalized = UVec4.equals(exponent, UVec4.zero());
    const is_nan_inf = UVec4.equals(exponent, exponent_mask);

    // Select the correct result
    const result_exponent_mantissa = UVec4.select(UVec4.select(exponent_mantissa, exponent_mantissa_nan_inf, is_nan_inf), exponent_mantissa_denormalized, is_denormalized);

    // Extract the sign bit and shift it to the left
    const sign = UVec4.bitAnd(value, UVec4.replicate(1 << half_flt_sign_pos)).logicalShiftLeft(float_sign_pos - half_flt_sign_pos);

    // Construct the float
    return UVec4.bitOr(sign, result_exponent_mantissa).reinterpretAsFloat();
}

/// Convert 4 half floats (lower 64 bits) to floats
pub fn toFloat(half_floats: UVec4) Vec4 {
    if (comptime use_f16c) {
        // _mm_cvtph_ps: LLVM turns the widening of 4 f16 lanes into one vcvtph2ps
        const halves: @Vector(8, f16) = @bitCast(half_floats.value);
        const lower = @shuffle(f16, halves, undefined, @Vector(4, i32){ 0, 1, 2, 3 });
        return .{ .value = @floatCast(lower) };
    }
    return toFloatFallback(half_floats);
}

fn isSignalingNaN(h: HalfFloat) bool {
    return (h & half_flt_exponent_and_mantissa_mask) > half_flt_inf and (h & (1 << (half_flt_mantissa_bits - 1))) == 0;
}

test "toFloat equals toFloatFallback for all 65536 half floats" {
    // With F16C, toFloat is the hardware conversion and this compares it with the fallback. Without F16C both are the
    // fallback, and the check against the compiler's f16 conversion below still tests the fallback.
    var v: u32 = 0;
    while (v < 0x10000) : (v += 4) {
        // The upper 64 bits must be ignored
        const half_floats = UVec4.init(v | ((v + 1) << 16), (v + 2) | ((v + 3) << 16), 0xffffffff, 0x7c017c01);
        const result = toFloat(half_floats).reinterpretAsInt();
        const fallback = toFloatFallback(half_floats).reinterpretAsInt();
        inline for (0..4) |i| {
            const h: HalfFloat = @intCast(v + i);
            if (isSignalingNaN(h)) {
                // F16C sets the quiet bit of a signaling NaN, the fallback keeps it. The payload is the same.
                const quiet_bit: u32 = 1 << (float_mantissa_bits - 1);
                try std.testing.expectEqual(fallback.value[i] | quiet_bit, result.value[i] | quiet_bit);
                try std.testing.expect(std.math.isNan(@as(f32, @bitCast(result.value[i]))));
            } else {
                try std.testing.expectEqual(fallback.value[i], result.value[i]);

                // The compiler's f16 -> f32 conversion (F16C or compiler-rt) gives the same bits
                const native: f32 = @floatCast(@as(f16, @bitCast(h)));
                try std.testing.expectEqual(@as(u32, @bitCast(native)), fallback.value[i]);
            }
        }
    }
}
