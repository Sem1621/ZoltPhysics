//! Port of: UnitTests/Math/HalfFloatTests.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const expect = fw.expect;
const expectEqual = fw.expectEqual;
const half_float = zolt.half_float;
const HalfFloat = zolt.HalfFloat;
const UVec4 = zolt.UVec4;
const Vec4 = zolt.Vec4;

// In Jolt, TestHalfFloatToFloat and TestFloatToHalfFloat only exist when JPH_USE_F16C or JPH_USE_NEON
// is defined and compare the hardware intrinsics with the fallback version. Zolt always uses the
// fallback, so these tests compare `toFloat` / `fromFloat` with the fallback (trivially equal) and
// additionally compare with the f16 conversions of the Zig compiler (F16C or compiler-rt), which
// play the role of the hardware intrinsics. Zig only has round to nearest conversions.

test "TestHalfFloatToFloat" {
    // Check all half float values, 4 at a time, skip NaN's and INF
    var v: u32 = 0;
    while (v < 0x7c00) : (v += 2) {
        // Test value, next value and negative variants of both
        const half_float_values = UVec4.init(v | ((v + 1) << 16), (v | 0x8000) | (((v + 1) | 0x8000) << 16), 0, 0);

        // Compare intrinsic version with fallback version
        const flt1 = half_float.toFloat(half_float_values);
        const flt2 = half_float.toFloatFallback(half_float_values);

        const flt1_as_int = flt1.reinterpretAsInt();
        const flt2_as_int = flt2.reinterpretAsInt();
        if (!flt1_as_int.eql(flt2_as_int))
            try expect(false); // Not using expect(flt1_as_int.eql(flt2_as_int)) to mirror the C++ test

        // Compare with the compiler's f16 -> f32 conversion
        const halfs = [4]u16{ @truncate(v), @truncate(v + 1), @truncate(v | 0x8000), @truncate((v + 1) | 0x8000) };
        inline for (0..4) |i| {
            const native: f32 = @floatCast(@as(f16, @bitCast(halfs[i])));
            if (@as(u32, @bitCast(native)) != flt2_as_int.value[i])
                try expect(false);
        }
    }
}

// Helper function to compare the intrinsics version with the fallback version
fn checkFloatToHalfFloat(value: u32, sign: u32) !void {
    const fvalue: f32 = @bitCast(value + sign * 0x80000000);

    var hf1 = half_float.fromFloat(.round_to_nearest, fvalue);
    var hf2 = half_float.fromFloatFallback(.round_to_nearest, fvalue);
    if (hf1 != hf2)
        try expect(false); // Not using expectEqual(hf1, hf2) to mirror the C++ test

    // Compare with the compiler's f32 -> f16 conversion (round to nearest)
    const native: u16 = @bitCast(@as(f16, @floatCast(fvalue)));
    if (native != hf2)
        try expect(false);

    hf1 = half_float.fromFloat(.round_to_pos_inf, fvalue);
    hf2 = half_float.fromFloatFallback(.round_to_pos_inf, fvalue);
    if (hf1 != hf2)
        try expect(false);

    hf1 = half_float.fromFloat(.round_to_neg_inf, fvalue);
    hf2 = half_float.fromFloatFallback(.round_to_neg_inf, fvalue);
    if (hf1 != hf2)
        try expect(false);
}

test "TestFloatToHalfFloat" {
    var sign: u32 = 0;
    while (sign < 2) : (sign += 1) {
        // Zero and smallest possible float
        var value: u32 = 0;
        while (value < 2) : (value += 1)
            try checkFloatToHalfFloat(value, sign);

        // Floats that are large enough to become a denormalized half float, incrementing by smallest increment that can make a difference
        value = (half_float.float_exponent_bias - half_float.half_flt_exponent_bias - half_float.half_flt_mantissa_bits) << half_float.float_exponent_pos;
        while (value < half_float.float_exponent_mask << half_float.float_exponent_pos) : (value += 1 << (half_float.float_mantissa_bits - half_float.half_flt_mantissa_bits - 2))
            try checkFloatToHalfFloat(value, sign);

        // INF
        try checkFloatToHalfFloat(0x7f800000, sign);

        // Nan
        try checkFloatToHalfFloat(0x7fc00000, sign);
    }
}

test "TestHalfFloatINF" {
    // Float -> half float
    try expectEqual(half_float.half_flt_inf, half_float.fromFloatFallback(.round_to_nearest, @bitCast(@as(u32, 0x7f800000))));
    try expectEqual(half_float.half_flt_inf_negative, half_float.fromFloatFallback(.round_to_nearest, @bitCast(@as(u32, 0xff800000))));

    // Half float -> float
    const half_float_values = UVec4.init(@as(u32, half_float.half_flt_inf) | (@as(u32, half_float.half_flt_inf_negative) << 16), 0, 0, 0);
    const flt = half_float.toFloatFallback(half_float_values).reinterpretAsInt();
    try expect(flt.eql(UVec4.init(0x7f800000, 0xff800000, 0, 0)));
}

test "TestHalfFloatNaN" {
    // Float -> half float
    try expectEqual(half_float.half_flt_nanq, half_float.fromFloatFallback(.round_to_nearest, @bitCast(@as(u32, 0x7fc00000))));
    try expectEqual(half_float.half_flt_nanq_negative, half_float.fromFloatFallback(.round_to_nearest, @bitCast(@as(u32, 0xffc00000))));

    // Half float -> float
    const half_float_values = UVec4.init(@as(u32, half_float.half_flt_nanq) | (@as(u32, half_float.half_flt_nanq_negative) << 16), 0, 0, 0);
    const flt = half_float.toFloatFallback(half_float_values).reinterpretAsInt();
    try expect(flt.eql(UVec4.init(0x7fc00000, 0xffc00000, 0, 0)));
}
