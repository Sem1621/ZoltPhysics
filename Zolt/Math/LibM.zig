//! Zolt addition, no Jolt file: the C library's `expf` and `powf` (C++ `std::exp(float)`, `std::pow(float, float)`).
//! Status: complete
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R13:
//! - Hair is the only Jolt code that calls the C library for floats: `std::exp(float)` (twice per material in
//!   Hair::Update) and `std::pow(float, float)` (HairSettings::Gradient::MakeStepDependent). Jolt's result then depends
//!   on the C library. Zolt uses this file, so its hair is the same on every platform.
//! - This is the generic algorithm that ARM's optimized-routines, musl and glibc's generic variants share (Copyright (c)
//!   2017-2018, Arm Limited, SPDX-License-Identifier: MIT): musl's src/math/expf.c, powf.c, exp2f_data.c and
//!   powf_data.c, with musl's configuration (TOINT_INTRINSICS 0, no errno, signaling NaNs are not told apart). Zig 0.16
//!   ships powf.c and the two data files; expf.c is the same file of optimized-routines and musl.
//! - Plain f64 arithmetic in the C operation order, no FMA. Zig's `std.math` and `compiler_rt` expf are an older
//!   algorithm with other results: never use them for Jolt's `std::exp` / `std::pow`.
//! - The parity reference pins `expf` and `powf` to the same C sources (ZoltParity/Hair/LibMReference.c), because the
//!   reference would otherwise call compiler_rt's `expf` and glibc's `powf`. ZoltParity/Hair/LibMParity.zig compares
//!   both functions and checks that the pin took.

const std = @import("std");

// exp2f_data.h / exp2f_data.c: shared between expf, exp2f and powf -----------------------------------------------------

const exp2f_table_bits = 5;
const exp2f_n = 1 << exp2f_table_bits;

/// tab[i] = uint(2^(i/N)) - (i << 52-BITS)
/// used for computing 2^(k/N) for an int |k| < 150 N as
/// double(tab[k%N] + (k << 52-BITS))
const exp2f_tab = [exp2f_n]u64{
    0x3ff0000000000000, 0x3fefd9b0d3158574, 0x3fefb5586cf9890f, 0x3fef9301d0125b51,
    0x3fef72b83c7d517b, 0x3fef54873168b9aa, 0x3fef387a6e756238, 0x3fef1e9df51fdee1,
    0x3fef06fe0a31b715, 0x3feef1a7373aa9cb, 0x3feedea64c123422, 0x3feece086061892d,
    0x3feebfdad5362a27, 0x3feeb42b569d4f82, 0x3feeab07dd485429, 0x3feea47eb03a5585,
    0x3feea09e667f3bcd, 0x3fee9f75e8ec5f74, 0x3feea11473eb0187, 0x3feea589994cce13,
    0x3feeace5422aa0db, 0x3feeb737b0cdc5e5, 0x3feec49182a3f090, 0x3feed503b23e255d,
    0x3feee89f995ad3ad, 0x3feeff76f2fb5e47, 0x3fef199bdd85529c, 0x3fef3720dcef9069,
    0x3fef5818dcfba487, 0x3fef7c97337b9b5f, 0x3fefa4afa2a490da, 0x3fefd0765b6e4540,
};
const exp2f_shift_scaled: f64 = 0x1.8p+52 / @as(f64, exp2f_n);
const exp2f_poly = [3]f64{ 0x1.c6af84b912394p-5, 0x1.ebfce50fac4f3p-3, 0x1.62e42ff0c52d6p-1 };
const exp2f_shift: f64 = 0x1.8p+52;
const exp2f_invln2_scaled: f64 = 0x1.71547652b82fep+0 * @as(f64, exp2f_n);
const exp2f_poly_scaled = [3]f64{
    0x1.c6af84b912394p-5 / @as(f64, exp2f_n) / @as(f64, exp2f_n) / @as(f64, exp2f_n),
    0x1.ebfce50fac4f3p-3 / @as(f64, exp2f_n) / @as(f64, exp2f_n),
    0x1.62e42ff0c52d6p-1 / @as(f64, exp2f_n),
};

// powf_data.h / powf_data.c -----------------------------------------------------------------------------------------

const powf_log2_table_bits = 4;
/// POWF_SCALE_BITS is 0 without TOINT_INTRINSICS, so POWF_SCALE is 1 and the scale factors of powf_data.c drop out
const powf_scale: f64 = 1.0;

const InvCLogC = struct { invc: f64, logc: f64 };

const powf_log2_tab = [1 << powf_log2_table_bits]InvCLogC{
    .{ .invc = 0x1.661ec79f8f3bep+0, .logc = -0x1.efec65b963019p-2 },
    .{ .invc = 0x1.571ed4aaf883dp+0, .logc = -0x1.b0b6832d4fca4p-2 },
    .{ .invc = 0x1.49539f0f010bp+0, .logc = -0x1.7418b0a1fb77bp-2 },
    .{ .invc = 0x1.3c995b0b80385p+0, .logc = -0x1.39de91a6dcf7bp-2 },
    .{ .invc = 0x1.30d190c8864a5p+0, .logc = -0x1.01d9bf3f2b631p-2 },
    .{ .invc = 0x1.25e227b0b8eap+0, .logc = -0x1.97c1d1b3b7afp-3 },
    .{ .invc = 0x1.1bb4a4a1a343fp+0, .logc = -0x1.2f9e393af3c9fp-3 },
    .{ .invc = 0x1.12358f08ae5bap+0, .logc = -0x1.960cbbf788d5cp-4 },
    .{ .invc = 0x1.0953f419900a7p+0, .logc = -0x1.a6f9db6475fcep-5 },
    .{ .invc = 0x1p+0, .logc = 0x0p+0 },
    .{ .invc = 0x1.e608cfd9a47acp-1, .logc = 0x1.338ca9f24f53dp-4 },
    .{ .invc = 0x1.ca4b31f026aap-1, .logc = 0x1.476a9543891bap-3 },
    .{ .invc = 0x1.b2036576afce6p-1, .logc = 0x1.e840b4ac4e4d2p-3 },
    .{ .invc = 0x1.9c2d163a1aa2dp-1, .logc = 0x1.40645f0c6651cp-2 },
    .{ .invc = 0x1.886e6037841edp-1, .logc = 0x1.88e9c2c1b9ff8p-2 },
    .{ .invc = 0x1.767dcf5534862p-1, .logc = 0x1.ce0a44eb17bccp-2 },
};
const powf_log2_poly = [5]f64{
    0x1.27616c9496e0bp-2, -0x1.71969a075c67ap-2,
    0x1.ec70a6ca7baddp-2, -0x1.7154748bef6c8p-1,
    0x1.71547652ab82bp0,
};

// libm.h helpers ----------------------------------------------------------------------------------------------------

inline fn asuint(f: f32) u32 {
    return @bitCast(f);
}

inline fn asfloat(i: u32) f32 {
    return @bitCast(i);
}

inline fn asuint64(f: f64) u64 {
    return @bitCast(f);
}

inline fn asdouble(i: u64) f64 {
    return @bitCast(i);
}

/// __math_oflowf / __math_uflowf (math/__math_xflowf.c): the overflowing or underflowing product, with the sign
fn xflowf(sign: u32, y: f32) f32 {
    const signed_y = if (sign != 0) -y else y;
    return signed_y * y;
}

fn oflowf(sign: u32) f32 {
    return xflowf(sign, 0x1p97);
}

fn uflowf(sign: u32) f32 {
    return xflowf(sign, 0x1p-95);
}

/// __math_invalidf (math/__math_invalidf.c): NaN (errno is not set, like musl)
fn invalidf(x: f32) f32 {
    return (x - x) / (x - x);
}

// expf.c --------------------------------------------------------------------------------------------------------------

/// EXP2F_TABLE_BITS = 5
/// EXP2F_POLY_ORDER = 3
///
/// ULP error: 0.502 (nearest rounding.)
/// Relative error: 1.69 * 2^-34 in [-ln2/64, ln2/64] (before rounding.)
/// Wrong count: 170635 (all nearest rounding wrong results with fma.)
/// Non-nearest ULP error: 1 (rounded ULP error)
inline fn top12(x: f32) u32 {
    return asuint(x) >> 20;
}

/// Single-precision e^x function (C `expf`, C++ `std::exp(float)`)
pub fn expf(x: f32) f32 {
    const xd: f64 = x;
    const abstop: u32 = top12(x) & 0x7ff;
    if (abstop >= top12(88.0)) {
        // |x| >= 88 or x is nan.
        if (asuint(x) == asuint(-std.math.inf(f32)))
            return 0.0;
        if (abstop >= top12(std.math.inf(f32)))
            return x + x;
        if (x > 0x1.62e42ep6) // x > log(0x1p128) ~= 88.72
            return oflowf(0);
        if (x < -0x1.9fe368p6) // x < log(0x1p-150) ~= -103.97
            return uflowf(0);
    }

    // x*N/Ln2 = k + r with r in [-1/2, 1/2] and int k.
    var z: f64 = exp2f_invln2_scaled * xd;

    // Round and convert z to int, the result is in [-150*N, 128*N] and
    // ideally ties-to-even rule is used, otherwise the magnitude of r
    // can be bigger which gives larger approximation error.
    var kd: f64 = z + exp2f_shift;
    const ki: u64 = asuint64(kd);
    kd -= exp2f_shift;
    const r: f64 = z - kd;

    // exp(x) = 2^(k/N) * 2^(r/N) ~= s * (C0*r^3 + C1*r^2 + C2*r + 1)
    var t: u64 = exp2f_tab[ki % exp2f_n];
    t +%= ki << (52 - exp2f_table_bits);
    const s: f64 = asdouble(t);
    z = exp2f_poly_scaled[0] * r + exp2f_poly_scaled[1];
    const r2: f64 = r * r;
    var y: f64 = exp2f_poly_scaled[2] * r + 1;
    y = z * r2 + y;
    y = y * s;
    return @floatCast(y);
}

// powf.c --------------------------------------------------------------------------------------------------------------

/// POWF_LOG2_POLY_ORDER = 5
/// EXP2F_TABLE_BITS = 5
///
/// ULP error: 0.82 (~ 0.5 + relerr*2^24)
/// relerr: 1.27 * 2^-26 (Relative error ~= 128*Ln2*relerr_log2 + relerr_exp2)
/// relerr_log2: 1.83 * 2^-33 (Relative error of logx.)
/// relerr_exp2: 1.69 * 2^-34 (Relative error of exp2(ylogx).)
const powf_n = 1 << powf_log2_table_bits;
const powf_off: u32 = 0x3f330000;

/// Subnormal input is normalized so ix has negative biased exponent.
/// Output is multiplied by N (POWF_SCALE) if TOINT_INTRINICS is set.
inline fn log2Inline(ix: u32) f64 {
    // x = 2^k z; where z is in range [OFF,2*OFF] and exact.
    // The range is split into N subintervals.
    // The ith subinterval contains z and c is near its center.
    const tmp: u32 = ix -% powf_off;
    const i: u32 = (tmp >> (23 - powf_log2_table_bits)) % powf_n;
    const top: u32 = tmp & 0xff800000;
    const iz: u32 = ix -% top;
    const k: i32 = @as(i32, @bitCast(top)) >> 23; // arithmetic shift (23 - POWF_SCALE_BITS)
    const invc: f64 = powf_log2_tab[i].invc;
    const logc: f64 = powf_log2_tab[i].logc;
    const z: f64 = asfloat(iz);

    // log2(x) = log1p(z/c-1)/ln2 + log2(c) + k
    const r: f64 = z * invc - 1;
    const y0: f64 = logc + @as(f64, @floatFromInt(k));

    // Pipelined polynomial evaluation to approximate log1p(r)/ln2.
    const r2: f64 = r * r;
    var y: f64 = powf_log2_poly[0] * r + powf_log2_poly[1];
    const p: f64 = powf_log2_poly[2] * r + powf_log2_poly[3];
    const r4: f64 = r2 * r2;
    var q: f64 = powf_log2_poly[4] * r + y0;
    q = p * r2 + q;
    y = y * r4 + q;
    return y;
}

const sign_bias_negative: u32 = 1 << (exp2f_table_bits + 11);

/// The output of log2 and thus the input of exp2 is either scaled by N
/// (in case of fast toint intrinsics) or not.  The unscaled xd must be
/// in [-1021,1023], sign_bias sets the sign of the result.
inline fn exp2Inline(xd: f64, sign_bias: u32) f32 {
    // x = k/N + r with r in [-1/(2N), 1/(2N)]
    var kd: f64 = xd + exp2f_shift_scaled;
    const ki: u64 = asuint64(kd);
    kd -= exp2f_shift_scaled; // k/N
    const r: f64 = xd - kd;

    // exp2(x) = 2^(k/N) * 2^r ~= s * (C0*r^3 + C1*r^2 + C2*r + 1)
    var t: u64 = exp2f_tab[ki % exp2f_n];
    const ski: u64 = ki +% sign_bias;
    t +%= ski << (52 - exp2f_table_bits);
    const s: f64 = asdouble(t);
    const z: f64 = exp2f_poly[0] * r + exp2f_poly[1];
    const r2: f64 = r * r;
    var y: f64 = exp2f_poly[2] * r + 1;
    y = z * r2 + y;
    y = y * s;
    return @floatCast(y);
}

/// Returns 0 if not int, 1 if odd int, 2 if even int.  The argument is
/// the bit representation of a non-zero finite floating-point value.
inline fn checkint(iy: u32) u32 {
    const e: u32 = iy >> 23 & 0xff;
    if (e < 0x7f)
        return 0;
    if (e > 0x7f + 23)
        return 2;
    const shift: u5 = @intCast(0x7f + 23 - e);
    if (iy & ((@as(u32, 1) << shift) - 1) != 0)
        return 0;
    if (iy & (@as(u32, 1) << shift) != 0)
        return 1;
    return 2;
}

inline fn zeroinfnan(ix: u32) bool {
    return 2 *% ix -% 1 >= 2 * 0x7f800000 - 1;
}

/// musl's issignalingf_inline without WANT_SNAN: signaling NaNs are not told apart
inline fn issignalingf(x: f32) bool {
    _ = x;
    return false;
}

/// Single-precision x^y function (C `powf`, C++ `std::pow(float, float)`)
pub fn powf(x: f32, y: f32) f32 {
    var sign_bias: u32 = 0;
    var ix: u32 = asuint(x);
    const iy: u32 = asuint(y);
    if (ix -% 0x00800000 >= 0x7f800000 - 0x00800000 or zeroinfnan(iy)) {
        // Either (x < 0x1p-126 or inf or nan) or (y is 0 or inf or nan).
        if (zeroinfnan(iy)) {
            if (2 *% iy == 0)
                return if (issignalingf(x)) x + y else 1.0;
            if (ix == 0x3f800000)
                return if (issignalingf(y)) x + y else 1.0;
            if (2 *% ix > 2 * 0x7f800000 or 2 *% iy > 2 * 0x7f800000)
                return x + y;
            if (2 *% ix == 2 * 0x3f800000)
                return 1.0;
            if ((2 *% ix < 2 * 0x3f800000) == (iy & 0x80000000 == 0))
                return 0.0; // |x|<1 && y==inf or |x|>1 && y==-inf.
            return y * y;
        }
        if (zeroinfnan(ix)) {
            var x2: f32 = x * x;
            if (ix & 0x80000000 != 0 and checkint(iy) == 1)
                x2 = -x2;
            // Without the barrier some versions of clang hoist the 1/x2 and
            // thus division by zero exception can be signaled spuriously.
            return if (iy & 0x80000000 != 0) 1 / x2 else x2;
        }
        // x and y are non-zero finite.
        if (ix & 0x80000000 != 0) {
            // Finite x < 0.
            const yint = checkint(iy);
            if (yint == 0)
                return invalidf(x);
            if (yint == 1)
                sign_bias = sign_bias_negative;
            ix &= 0x7fffffff;
        }
        if (ix < 0x00800000) {
            // Normalize subnormal x so exponent becomes negative.
            ix = asuint(x * 0x1p23);
            ix &= 0x7fffffff;
            ix -%= 23 << 23;
        }
    }
    const logx: f64 = log2Inline(ix);
    const ylogx: f64 = @as(f64, y) * logx; // cannot overflow, y is single prec.
    if ((asuint64(ylogx) >> 47 & 0xffff) >= asuint64(126.0 * powf_scale) >> 47) {
        // |y*log(x)| >= 126.
        if (ylogx > 0x1.fffffffd1d571p+6 * powf_scale)
            return oflowf(sign_bias);
        if (ylogx <= -150.0 * powf_scale)
            return uflowf(sign_bias);
    }
    return exp2Inline(ylogx, sign_bias);
}

// Tests ---------------------------------------------------------------------------------------------------------------

const testing = std.testing;

fn expectBits(expected: u32, actual: f32) !void {
    try testing.expectEqual(expected, @as(u32, @bitCast(actual)));
}

test "LibM: expf special values and known results" {
    // Exact and special results
    try expectBits(0x3f800000, expf(0.0)); // e^0 = 1
    try expectBits(0x3f800000, expf(-0.0));
    try expectBits(0x7f800000, expf(std.math.inf(f32)));
    try expectBits(0x00000000, expf(-std.math.inf(f32)));
    try testing.expect(std.math.isNan(expf(std.math.nan(f32))));
    try expectBits(0x7f800000, expf(89.0)); // overflow
    try expectBits(0x00000000, expf(-104.0)); // underflow

    // Results of the generic algorithm (the same bits as glibc's expf)
    try expectBits(0x402df854, expf(1.0)); // e
    try expectBits(0x3ebc5ab2, expf(-1.0)); // 1 / e
    try expectBits(0x3fd3094c, expf(0.5));
    try expectBits(0x3f7cded8, expf(-0.0123));
    try expectBits(0x46ac14ee, expf(10.0));
    try expectBits(0x3b85ea53, expf(-5.5));
}

test "LibM: powf special values and known results" {
    try expectBits(0x3f800000, powf(2.0, 0.0)); // x^0 = 1
    try expectBits(0x3f800000, powf(1.0, std.math.nan(f32))); // 1^y = 1
    try testing.expect(std.math.isNan(powf(std.math.nan(f32), 2.0)));
    try testing.expect(std.math.isNan(powf(-2.0, 0.5))); // negative base, not an integer exponent
    try expectBits(0x41000000, powf(2.0, 3.0)); // 8
    try expectBits(0xc1000000, powf(-2.0, 3.0)); // odd integer exponent keeps the sign
    try expectBits(0x41800000, powf(-2.0, 4.0)); // even integer exponent
    try expectBits(0x3f000000, powf(4.0, -0.5)); // 1/2
    try expectBits(0x00000000, powf(0.0, 2.0));
    try expectBits(0x7f800000, powf(0.0, -1.0)); // 1 / 0
    try expectBits(0xff800000, powf(-0.0, -1.0)); // 1 / -0
    try expectBits(0x7f800000, powf(10.0, 39.0)); // overflow
    try expectBits(0x00000000, powf(10.0, -46.0)); // underflow
    try expectBits(0x00000000, powf(0.5, std.math.inf(f32))); // |x| < 1, y = inf
    try expectBits(0x7f800000, powf(2.0, std.math.inf(f32)));
    try expectBits(0x3f800000, powf(-1.0, std.math.inf(f32)));

    // Subnormal base: normalized before the logarithm
    try expectBits(0x1c800000, powf(0x1p-140, 0.5)); // 2^-70

    // Results of the generic algorithm (the same bits as glibc's powf)
    try expectBits(0x3f772a50, powf(0.9, 1.0 / 3.0));
    try expectBits(0x3eb504f3, powf(0.5, 1.5));
    try expectBits(0x3f7ae7d6, powf(0.99, 2.0));

    // MakeStepDependent with a time ratio of 1: 1 - (1 - v)^1 must be exactly v for these values
    for ([_]f32{ 0.0, 0.01, 0.05, 0.25, 0.5, 1.0 }) |v| try testing.expectEqual(1.0 - v, powf(1.0 - v, 1.0));
}

test "LibM: the inputs where other expf and powf implementations differ" {
    // The probe inputs of the parity (ZoltParity/Hair/LibMParity.zig). compiler_rt's expf and glibc's FMA variant give
    // 0x56fc9f1c for 0x4202422f; glibc's FMA variant gives 0x11fa2993 for 0xc27c65d9.
    try expectBits(0x56fc9f1b, expf(@bitCast(@as(u32, 0x4202422f))));
    try expectBits(0x11fa2992, expf(@bitCast(@as(u32, 0xc27c65d9))));
    // compiler_rt's expf gives 0x3f8009a6 and 0x3f8059c0
    try expectBits(0x3f8009a5, expf(@bitCast(@as(u32, 0x399a522f))));
    try expectBits(0x3f8059c1, expf(@bitCast(@as(u32, 0x3b33422f))));
    // musl does not tell signaling NaNs apart: x^0 and 1^y are 1 (glibc returns a NaN for a signaling NaN)
    try expectBits(0x3f800000, powf(@bitCast(@as(u32, 0xff9bc154)), 0.0));
    try expectBits(0x3f800000, powf(1.0, @bitCast(@as(u32, 0x7f82bb67))));
}
