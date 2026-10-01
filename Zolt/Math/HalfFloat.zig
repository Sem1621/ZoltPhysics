//! Port of: Jolt/Math/HalfFloat.h
//! Status: complete
//!
//! The `HalfFloatConversion` namespace of Jolt is this file (re-exported as `zolt.half_float`), so
//! `HalfFloatConversion::FromFloat<HalfFloatConversion::ROUND_TO_NEAREST>(v)` becomes
//! `half_float.fromFloat(.round_to_nearest, v)`. Zolt always uses the portable fallback
//! implementations (no F16C / NEON), which give bit identical results.

const UVec4 = @import("UVec4.zig").UVec4;
const Vec4 = @import("Vec4.zig").Vec4;

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
    return toFloatFallback(half_floats);
}
