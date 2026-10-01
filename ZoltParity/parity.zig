//! Parity tests: run Zolt and the C++ Jolt library (compiled by `zig build parity` with
//! JPH_CROSS_PLATFORM_DETERMINISTIC and -ffp-contract=off) on the same inputs and require identical bits.
//!
//! This is the strongest check that a port is exact. Add a test here for every ported algorithm
//! whose result is a number (math, geometry, collision queries, and later whole simulation steps),
//! with a matching C ABI wrapper in ZoltParity/JoltReference.cpp.
//!
//! Run: zig build parity [-Ddouble_precision=true] [-Dtest-filter=Vec4]

const std = @import("std");
const zolt = @import("zolt");

const half_float = zolt.half_float;
const DMat44 = zolt.DMat44;
const DVec3 = zolt.DVec3;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const UVec4 = zolt.UVec4;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

/// The C++ reference functions, see JoltReference.cpp
const jolt = struct {
    extern fn jolt_acos_approximate(x: f32) f32;
    extern fn jolt_vec4_sincos(v: *const [4]f32, out_sin: *[4]f32, out_cos: *[4]f32) void;
    extern fn jolt_vec4_tan(v: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_vec4_asin(v: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_vec4_acos(v: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_vec4_atan(v: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_vec4_atan2(y: *const [4]f32, x: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_vec4_normalized(v: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_vec4_dot(a: *const [4]f32, b: *const [4]f32) f32;
    extern fn jolt_vec4_length(v: *const [4]f32) f32;
    extern fn jolt_vec4_reduce_min(v: *const [4]f32) f32;
    extern fn jolt_vec4_reduce_max(v: *const [4]f32) f32;
    extern fn jolt_vec4_compress_unit_vector(v: *const [4]f32) u32;
    extern fn jolt_vec4_decompress_unit_vector(value: u32, out: *[4]f32) void;
    extern fn jolt_vec3_normalized(v: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_vec3_cross(a: *const [3]f32, b: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_vec3_dot(a: *const [3]f32, b: *const [3]f32) f32;
    extern fn jolt_vec3_length(v: *const [3]f32) f32;
    extern fn jolt_vec3_normalized_perpendicular(v: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_vec3_unit_spherical(theta: f32, phi: f32, out: *[3]f32) void;
    extern fn jolt_vec3_compress_unit_vector(v: *const [3]f32) u32;
    extern fn jolt_vec3_decompress_unit_vector(value: u32, out: *[3]f32) void;

    extern fn jolt_quat_rotation(axis: *const [3]f32, angle: f32, out: *[4]f32) void;
    extern fn jolt_quat_mul(a: *const [4]f32, b: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_quat_mul_vec3(q: *const [4]f32, v: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_quat_inverse_rotate(q: *const [4]f32, v: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_quat_multiply_imaginary(v: *const [3]f32, q: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_quat_rotate_axis_x(q: *const [4]f32, out: *[3]f32) void;
    extern fn jolt_quat_rotate_axis_y(q: *const [4]f32, out: *[3]f32) void;
    extern fn jolt_quat_rotate_axis_z(q: *const [4]f32, out: *[3]f32) void;
    extern fn jolt_quat_inversed(q: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_quat_conjugated(q: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_quat_normalized(q: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_quat_ensure_w_positive(q: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_quat_get_axis_angle(q: *const [4]f32, out_axis: *[3]f32, out_angle: *f32) void;
    extern fn jolt_quat_get_angular_velocity(q: *const [4]f32, delta_time: f32, out: *[3]f32) void;
    extern fn jolt_quat_get_swing_twist(q: *const [4]f32, out_swing: *[4]f32, out_twist: *[4]f32) void;
    extern fn jolt_quat_get_twist(q: *const [4]f32, axis: *const [3]f32, out: *[4]f32) void;
    extern fn jolt_quat_get_rotation_angle(q: *const [4]f32, axis: *const [3]f32) f32;
    extern fn jolt_quat_get_euler_angles(q: *const [4]f32, out: *[3]f32) void;
    extern fn jolt_quat_euler_angles(angles: *const [3]f32, out: *[4]f32) void;
    extern fn jolt_quat_from_to(from: *const [3]f32, to: *const [3]f32, out: *[4]f32) void;
    extern fn jolt_quat_lerp(a: *const [4]f32, b: *const [4]f32, fraction: f32, out: *[4]f32) void;
    extern fn jolt_quat_slerp(a: *const [4]f32, b: *const [4]f32, fraction: f32, out: *[4]f32) void;

    extern fn jolt_mat44_rotation_x(angle: f32, out: *[16]f32) void;
    extern fn jolt_mat44_rotation_y(angle: f32, out: *[16]f32) void;
    extern fn jolt_mat44_rotation_z(angle: f32, out: *[16]f32) void;
    extern fn jolt_mat44_rotation(axis: *const [3]f32, angle: f32, out: *[16]f32) void;
    extern fn jolt_mat44_rotation_quat(q: *const [4]f32, out: *[16]f32) void;
    extern fn jolt_mat44_inverse_rotation_translation(q: *const [4]f32, t: *const [3]f32, out: *[16]f32) void;
    extern fn jolt_mat44_outer_product(a: *const [3]f32, b: *const [3]f32, out: *[16]f32) void;
    extern fn jolt_mat44_cross_product(v: *const [3]f32, out: *[16]f32) void;
    extern fn jolt_mat44_quat_left_multiply(q: *const [4]f32, out: *[16]f32) void;
    extern fn jolt_mat44_quat_right_multiply(q: *const [4]f32, out: *[16]f32) void;
    extern fn jolt_mat44_look_at(pos: *const [3]f32, target: *const [3]f32, up: *const [3]f32, out: *[16]f32) void;
    extern fn jolt_mat44_perspective(fov_y: f32, aspect: f32, near: f32, far: f32, out: *[16]f32) void;
    extern fn jolt_mat44_mul(a: *const [16]f32, b: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_mul_vec3(m: *const [16]f32, v: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_mat44_mul_vec4(m: *const [16]f32, v: *const [4]f32, out: *[4]f32) void;
    extern fn jolt_mat44_multiply3x3(m: *const [16]f32, v: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_mat44_multiply3x3_transposed(m: *const [16]f32, v: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_mat44_multiply3x3_mat44(a: *const [16]f32, b: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_multiply3x3_left_transposed(a: *const [16]f32, b: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_multiply3x3_right_transposed(a: *const [16]f32, b: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_mul_scalar(m: *const [16]f32, v: f32, out: *[16]f32) void;
    extern fn jolt_mat44_add(a: *const [16]f32, b: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_sub(a: *const [16]f32, b: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_negate(m: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_transposed(m: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_transposed3x3(m: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_inversed(m: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_inversed_rotation_translation(m: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_get_determinant3x3(m: *const [16]f32) f32;
    extern fn jolt_mat44_adjointed3x3(m: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_inversed3x3(m: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_set_inversed3x3(m: *const [16]f32, io: *[16]f32) bool;
    extern fn jolt_mat44_get_quaternion(m: *const [16]f32, out: *[4]f32) void;
    extern fn jolt_mat44_get_direction_preserving_matrix(m: *const [16]f32, out: *[16]f32) void;
    extern fn jolt_mat44_pre_translated(m: *const [16]f32, t: *const [3]f32, out: *[16]f32) void;
    extern fn jolt_mat44_post_translated(m: *const [16]f32, t: *const [3]f32, out: *[16]f32) void;
    extern fn jolt_mat44_pre_scaled(m: *const [16]f32, s: *const [3]f32, out: *[16]f32) void;
    extern fn jolt_mat44_post_scaled(m: *const [16]f32, s: *const [3]f32, out: *[16]f32) void;
    extern fn jolt_mat44_decompose(m: *const [16]f32, out: *[16]f32, out_scale: *[3]f32) void;

    extern fn jolt_dvec3_add(a: *const [3]f64, b: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_sub(a: *const [3]f64, b: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_mul(a: *const [3]f64, b: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_div(a: *const [3]f64, b: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_add_vec3(a: *const [3]f64, b: *const [3]f32, out: *[3]f64) void;
    extern fn jolt_dvec3_sub_vec3(a: *const [3]f64, b: *const [3]f32, out: *[3]f64) void;
    extern fn jolt_dvec3_mul_scalar(a: *const [3]f64, b: f64, out: *[3]f64) void;
    extern fn jolt_dvec3_div_scalar(a: *const [3]f64, b: f64, out: *[3]f64) void;
    extern fn jolt_dvec3_negate(a: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_abs(a: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_reciprocal(a: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_sqrt(a: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_get_sign(a: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_min(a: *const [3]f64, b: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_max(a: *const [3]f64, b: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_cross(a: *const [3]f64, b: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_dot(a: *const [3]f64, b: *const [3]f64) f64;
    extern fn jolt_dvec3_length_sq(a: *const [3]f64) f64;
    extern fn jolt_dvec3_length(a: *const [3]f64) f64;
    extern fn jolt_dvec3_normalized(a: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_to_vec3(a: *const [3]f64, out: *[3]f32) void;
    extern fn jolt_dvec3_prepare_round_to_zero(a: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_prepare_round_to_inf(a: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dvec3_to_vec3_round_down(a: *const [3]f64, out: *[3]f32) void;
    extern fn jolt_dvec3_to_vec3_round_up(a: *const [3]f64, out: *[3]f32) void;

    extern fn jolt_dmat44_mul(a_cols: *const [12]f32, a_t: *const [3]f64, b_cols: *const [12]f32, b_t: *const [3]f64, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_mul_mat44(a_cols: *const [12]f32, a_t: *const [3]f64, b: *const [16]f32, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_mul_vec3(cols: *const [12]f32, t: *const [3]f64, v: *const [3]f32, out: *[3]f64) void;
    extern fn jolt_dmat44_mul_dvec3(cols: *const [12]f32, t: *const [3]f64, v: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dmat44_multiply3x3(cols: *const [12]f32, t: *const [3]f64, v: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_dmat44_multiply3x3_dvec3(cols: *const [12]f32, t: *const [3]f64, v: *const [3]f64, out: *[3]f64) void;
    extern fn jolt_dmat44_multiply3x3_transposed(cols: *const [12]f32, t: *const [3]f64, v: *const [3]f32, out: *[3]f32) void;
    extern fn jolt_dmat44_inversed(cols: *const [12]f32, t: *const [3]f64, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_inversed_rotation_translation(cols: *const [12]f32, t: *const [3]f64, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_rotation_translation(q: *const [4]f32, t: *const [3]f64, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_inverse_rotation_translation(q: *const [4]f32, t: *const [3]f64, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_pre_translated(cols: *const [12]f32, t: *const [3]f64, v: *const [3]f32, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_pre_translated_dvec3(cols: *const [12]f32, t: *const [3]f64, v: *const [3]f64, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_post_translated(cols: *const [12]f32, t: *const [3]f64, v: *const [3]f32, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_post_translated_dvec3(cols: *const [12]f32, t: *const [3]f64, v: *const [3]f64, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_pre_scaled(cols: *const [12]f32, t: *const [3]f64, s: *const [3]f32, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_post_scaled(cols: *const [12]f32, t: *const [3]f64, s: *const [3]f32, out_cols: *[12]f32, out_t: *[3]f64) void;
    extern fn jolt_dmat44_decompose(cols: *const [12]f32, t: *const [3]f64, out_cols: *[12]f32, out_t: *[3]f64, out_scale: *[3]f32) void;
    extern fn jolt_dmat44_to_mat44(cols: *const [12]f32, t: *const [3]f64, out: *[16]f32) void;

    extern fn jolt_half_from_float(v: f32, rounding_mode: c_int) u16;
    extern fn jolt_half_from_float_fallback(v: f32, rounding_mode: c_int) u16;
    extern fn jolt_half_to_float(v: *const [4]u16, out: *[4]f32) void;
    extern fn jolt_half_to_float_fallback(v: *const [4]u16, out: *[4]f32) void;
};

/// Number of random inputs per test
const iterations = 100_000;

/// Values that tend to expose differences, mixed into the random inputs
const special_values = [_]f32{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 1.0e-30, -1.0e-30, 1.0e-6, zolt.math.pi, -zolt.math.pi, 0.5 * zolt.math.pi, 0.25 * zolt.math.pi, 0.4142135623730950, 2.414213562373095, 100.0, -100.0 };

/// Deterministic input generator (xorshift32)
const Rng = struct {
    state: u32 = 0x12345678,

    fn next(self: *Rng) u32 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 17;
        self.state ^= self.state << 5;
        return self.state;
    }

    /// Random float in [min, max), or one of the special values (10% of the time)
    fn float(self: *Rng, min: f32, max: f32) f32 {
        if (self.next() % 10 == 0) {
            const v = special_values[self.next() % special_values.len];
            if (v >= min and v <= max) return v;
        }
        const t = @as(f32, @floatFromInt(self.next() >> 8)) / 16777216.0;
        return min + (max - min) * t;
    }

    fn array(self: *Rng, comptime n: usize, min: f32, max: f32) [n]f32 {
        var result: [n]f32 = undefined;
        for (&result) |*v| v.* = self.float(min, max);
        return result;
    }

    /// Random vector that is not too close to zero
    fn nonZero(self: *Rng, comptime n: usize, min: f32, max: f32) [n]f32 {
        while (true) {
            const v = self.array(n, min, max);
            var len_sq: f32 = 0;
            for (v) |c| len_sq += c * c;
            if (len_sq > 1.0e-6) return v;
        }
    }

    fn unitVector(self: *Rng, comptime n: usize) [n]f32 {
        return normalize(n, self.nonZero(n, -1, 1));
    }

    /// Random unit quaternion (x, y, z, w), or one of the special cases (identity, 180 degree rotations,
    /// x = w = 0, tiny rotations, negative w) that take separate branches in the quaternion functions
    fn unitQuat(self: *Rng) [4]f32 {
        switch (self.next() % 8) {
            0 => return special_quats[self.next() % special_quats.len],
            1 => {
                // Small rotation (near identity), sometimes with negative w
                const axis = self.unitVector(3);
                const s = self.float(-0.02, 0.02);
                const w: f32 = if (self.next() % 2 == 0) 1 else -1;
                return normalize(4, .{ axis[0] * s, axis[1] * s, axis[2] * s, w });
            },
            else => return self.unitVector(4),
        }
    }

    /// Random double in [min, max) with a full 53 bit mantissa, or one of the special values (10% of the time)
    fn double(self: *Rng, min: f64, max: f64) f64 {
        if (self.next() % 10 == 0) {
            const v = special_doubles[self.next() % special_doubles.len];
            if (v >= min and v <= max) return v;
        }
        const bits = (@as(u64, self.next()) << 21) ^ @as(u64, self.next() >> 11);
        const t = @as(f64, @floatFromInt(bits & ((1 << 53) - 1))) / 9007199254740992.0;
        return min + (max - min) * t;
    }

    fn doubleArray(self: *Rng, comptime n: usize, min: f64, max: f64) [n]f64 {
        var result: [n]f64 = undefined;
        for (&result) |*v| v.* = self.double(min, max);
        return result;
    }

    /// Random double with any magnitude: random sign, exponent and mantissa, sometimes any bit pattern (NaN, inf, denormals)
    fn anyDouble(self: *Rng) f64 {
        return switch (self.next() % 8) {
            0 => @bitCast((@as(u64, self.next()) << 32) | self.next()),
            1 => special_doubles[self.next() % special_doubles.len],
            else => blk: {
                const sign: f64 = if (self.next() % 2 == 0) 1 else -1;
                const exponent: i32 = @as(i32, @intCast(self.next() % 320)) - 170; // float range is 2^-149 .. 2^128
                break :blk sign * std.math.ldexp(self.double(1, 2), exponent);
            },
        };
    }

    /// Random float with any bit pattern (NaN, inf, denormals)
    fn anyFloat(self: *Rng) f32 {
        return @bitCast(self.next());
    }

    /// Random matrix (16 floats, column major):
    /// - kind 0: rotation * scale + translation (scale may be negative)
    /// - kind 1: all elements random
    /// - kind 2: random 3x3 part and translation, bottom row (0, 0, 0, 1)
    fn matrix(self: *Rng) [16]f32 {
        switch (self.next() % 3) {
            0 => {
                var m = arr16(Mat44.rotationQuat(quat(self.unitQuat())));
                const s = self.array(3, -10, 10);
                for (0..3) |c| {
                    for (0..3) |r| m[4 * c + r] *= s[c];
                }
                const t = self.array(3, -100, 100);
                m[12] = t[0];
                m[13] = t[1];
                m[14] = t[2];
                return m;
            },
            1 => return self.array(16, -10, 10),
            else => {
                var m = self.array(16, -10, 10);
                m[3] = 0;
                m[7] = 0;
                m[11] = 0;
                m[15] = 1;
                return m;
            },
        }
    }

    /// Random matrix like `matrix` but with the W components of the first 3 columns 0 (required by some 3x3 functions)
    fn matrix3x3(self: *Rng) [16]f32 {
        var m = self.matrix();
        m[3] = 0;
        m[7] = 0;
        m[11] = 0;
        return m;
    }

    /// Random rotation + translation matrix
    fn rotationTranslation(self: *Rng) [16]f32 {
        var m = arr16(Mat44.rotationQuat(quat(self.unitQuat())));
        const t = self.array(3, -100, 100);
        m[12] = t[0];
        m[13] = t[1];
        m[14] = t[2];
        return m;
    }

    /// Random DMat44: rotation (and scale) columns or random columns, the translation has a full double mantissa
    fn dmatrix(self: *Rng) DMat44Data {
        const m = if (self.next() % 2 == 0) self.matrix3x3() else self.rotationTranslation();
        const range: f64 = switch (self.next() % 3) {
            0 => 10.0,
            1 => 1.0e4,
            else => 1.0e7,
        };
        return .{ .cols = m[0..12].*, .t = self.doubleArray(3, -range, range) };
    }
};

/// Values that tend to expose differences in double precision code
const special_doubles = [_]f64{ 0.0, -0.0, 1.0, -1.0, 0.5, -0.5, 1.0e-300, -1.0e-300, 1.0e-6, 1.0e10, -1.0e10, std.math.pi, 3.4028234663852886e38, 3.4028235677973366e38, -3.4028235677973366e38, 1.1754943508222875e-38, 1.401298464324817e-45, 7.006492321624085e-46, 16777217.0, 0.1, std.math.inf(f64), -std.math.inf(f64), std.math.nan(f64) };

/// Unit quaternions that take special branches
const special_quats = [_][4]f32{
    .{ 0, 0, 0, 1 },
    .{ 0, 0, 0, -1 },
    .{ 1, 0, 0, 0 },
    .{ 0, 1, 0, 0 },
    .{ 0, 0, 1, 0 },
    .{ 0, 0.6, 0.8, 0 },
    .{ 0, -0.8, 0.6, 0 },
    .{ 0.70710677, 0, 0, 0.70710677 },
    .{ 0, 0.70710677, 0, 0.70710677 },
    .{ 0, 0, 0.70710677, -0.70710677 },
    .{ 0.5, 0.5, 0.5, 0.5 },
    .{ -0.5, 0.5, -0.5, 0.5 },
};

fn normalize(comptime n: usize, v: [n]f32) [n]f32 {
    var len_sq: f32 = 0;
    for (v) |c| len_sq += c * c;
    var result: [n]f32 = undefined;
    for (&result, v) |*r, c| r.* = c / @sqrt(len_sq);
    return result;
}

/// Bitwise equality, all NaNs are considered equal (NaN payloads are not part of the determinism contract)
fn sameBits(a: anytype, b: @TypeOf(a)) bool {
    if (std.math.isNan(a) and std.math.isNan(b)) return true;
    const Bits = std.meta.Int(.unsigned, @bitSizeOf(@TypeOf(a)));
    return @as(Bits, @bitCast(a)) == @as(Bits, @bitCast(b));
}

/// Bitwise equality of numbers, bools, arrays and structs of them (see sameBits)
fn sameValue(a: anytype, b: @TypeOf(a)) bool {
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
const Checker = struct {
    name: []const u8,
    mismatches: usize = 0,

    fn check(self: *Checker, input: anytype, zolt_result: anytype, jolt_result: @TypeOf(zolt_result)) void {
        const equal = sameValue(zolt_result, jolt_result);
        if (!equal) {
            if (self.mismatches < 5)
                std.debug.print("{s}: mismatch for input {any}\n  zolt: {any}\n  jolt: {any}\n", .{ self.name, input, zolt_result, jolt_result });
            self.mismatches += 1;
        }
    }

    fn finish(self: *const Checker) !void {
        if (self.mismatches > 0) {
            std.debug.print("{s}: {d} mismatches\n", .{ self.name, self.mismatches });
            return error.ParityMismatch;
        }
    }
};

/// Report the result of every checker (not only the first one that fails), fails if any of them found a mismatch
fn finishAll(checkers: []const *const Checker) !void {
    var failed = false;
    for (checkers) |checker|
        checker.finish() catch {
            failed = true;
        };
    if (failed) return error.ParityMismatch;
}

fn vec4(a: [4]f32) Vec4 {
    return Vec4.init(a[0], a[1], a[2], a[3]);
}

fn arr4(v: Vec4) [4]f32 {
    return v.value;
}

fn vec3(a: [3]f32) Vec3 {
    return Vec3.init(a[0], a[1], a[2]);
}

fn arr3(v: Vec3) [3]f32 {
    return .{ v.getX(), v.getY(), v.getZ() };
}

fn quat(a: [4]f32) Quat {
    return Quat.init(a[0], a[1], a[2], a[3]);
}

fn arrQ(q: Quat) [4]f32 {
    return q.value.value;
}

fn mat44(a: [16]f32) Mat44 {
    return Mat44.init(vec4(a[0..4].*), vec4(a[4..8].*), vec4(a[8..12].*), vec4(a[12..16].*));
}

fn arr16(m: Mat44) [16]f32 {
    return arr4(m.col[0]) ++ arr4(m.col[1]) ++ arr4(m.col[2]) ++ arr4(m.col[3]);
}

fn dvec3(a: [3]f64) DVec3 {
    return DVec3.init(a[0], a[1], a[2]);
}

fn arrD3(v: DVec3) [3]f64 {
    return .{ v.getX(), v.getY(), v.getZ() };
}

/// A DMat44 in the format of the C ABI: 3 rotation columns of 4 floats and a translation of 3 doubles
const DMat44Data = struct {
    cols: [12]f32,
    t: [3]f64,

    fn toDMat44(self: DMat44Data) DMat44 {
        return DMat44.init(vec4(self.cols[0..4].*), vec4(self.cols[4..8].*), vec4(self.cols[8..12].*), dvec3(self.t));
    }

    fn fromDMat44(m: DMat44) DMat44Data {
        return .{ .cols = arr4(m.col[0]) ++ arr4(m.col[1]) ++ arr4(m.col[2]), .t = arrD3(m.col3) };
    }
};

/// Determinant of the upper left 3x3 part of a column major matrix, calculated in double precision
fn determinant3x3F64(m: [16]f32) f64 {
    var d: [16]f64 = undefined;
    for (&d, m) |*o, i| o.* = i;
    return d[0] * (d[5] * d[10] - d[9] * d[6]) - d[4] * (d[1] * d[10] - d[9] * d[2]) + d[8] * (d[1] * d[6] - d[5] * d[2]);
}

/// Determinant of a column major 4x4 matrix, calculated in double precision (Laplace expansion along the last row)
fn determinant4x4F64(m: [16]f32) f64 {
    var result: f64 = 0;
    for (0..4) |skip_col| {
        // Minor without the last row and column skip_col
        var minor: [16]f32 = undefined;
        var c_out: usize = 0;
        for (0..4) |c| {
            if (c == skip_col) continue;
            for (0..3) |r| minor[4 * c_out + r] = m[4 * c + r];
            c_out += 1;
        }
        const sign: f64 = if ((3 + skip_col) % 2 == 0) 1 else -1;
        result += sign * @as(f64, m[4 * skip_col + 3]) * determinant3x3F64(minor);
    }
    return result;
}

/// Largest absolute element of the first `n` rows / columns of a column major matrix
fn maxAbsElement(m: [16]f32, comptime n: usize) f64 {
    var result: f64 = 0;
    for (0..n) |c| {
        for (0..n) |r| result = @max(result, @abs(@as(f64, m[4 * c + r])));
    }
    return result;
}

/// True if the matrix is far enough from singular to make its inverse meaningful
fn isWellConditioned3x3(m: [16]f32) bool {
    const scale = maxAbsElement(m, 3);
    return @abs(determinant3x3F64(m)) > 1.0e-3 * scale * scale * scale;
}

fn isWellConditioned4x4(m: [16]f32) bool {
    const scale = maxAbsElement(m, 4);
    return @abs(determinant4x4F64(m)) > 1.0e-3 * scale * scale * scale * scale;
}

/// Run a Vec4 -> Vec4 function over `iterations` random inputs in [min, max)
fn checkVec4Unary(name: []const u8, comptime zolt_fn: fn (Vec4) Vec4, comptime jolt_fn: fn (*const [4]f32, *[4]f32) callconv(.c) void, min: f32, max: f32) !void {
    var rng: Rng = .{};
    var checker: Checker = .{ .name = name };
    for (0..iterations) |_| {
        const in = rng.array(4, min, max);
        var expected: [4]f32 = undefined;
        jolt_fn(&in, &expected);
        checker.check(in, arr4(zolt_fn(vec4(in))), expected);
    }
    try checker.finish();
}

test "Trigonometry.acosApproximate" {
    var rng: Rng = .{};
    var checker: Checker = .{ .name = "acosApproximate" };
    for (0..iterations) |_| {
        const x = rng.float(-1.5, 1.5);
        checker.check(x, zolt.trigonometry.acosApproximate(x), jolt.jolt_acos_approximate(x));
    }
    try checker.finish();
}

test "Vec4.sinCos" {
    var rng: Rng = .{};
    var sin_checker: Checker = .{ .name = "Vec4.sinCos (sin)" };
    var cos_checker: Checker = .{ .name = "Vec4.sinCos (cos)" };
    for (0..iterations) |_| {
        const in = rng.array(4, -1000, 1000);
        var expected_sin: [4]f32 = undefined;
        var expected_cos: [4]f32 = undefined;
        jolt.jolt_vec4_sincos(&in, &expected_sin, &expected_cos);
        const sc = vec4(in).sinCos();
        sin_checker.check(in, arr4(sc.sin), expected_sin);
        cos_checker.check(in, arr4(sc.cos), expected_cos);
    }
    try finishAll(&.{ &sin_checker, &cos_checker });
}

test "Vec4.tan" {
    try checkVec4Unary("Vec4.tan", Vec4.tan, jolt.jolt_vec4_tan, -1000, 1000);
}

test "Vec4.asin" {
    try checkVec4Unary("Vec4.asin", Vec4.asin, jolt.jolt_vec4_asin, -1.1, 1.1);
}

test "Vec4.acos" {
    try checkVec4Unary("Vec4.acos", Vec4.acos, jolt.jolt_vec4_acos, -1.1, 1.1);
}

test "Vec4.atan" {
    try checkVec4Unary("Vec4.atan", Vec4.atan, jolt.jolt_vec4_atan, -1.0e4, 1.0e4);
}

test "Vec4.atan2" {
    var rng: Rng = .{};
    var checker: Checker = .{ .name = "Vec4.atan2" };
    for (0..iterations) |_| {
        const y = rng.array(4, -100, 100);
        const x = rng.array(4, -100, 100);
        var expected: [4]f32 = undefined;
        jolt.jolt_vec4_atan2(&y, &x, &expected);
        checker.check(.{ y, x }, arr4(Vec4.atan2(vec4(y), vec4(x))), expected);
    }
    try checker.finish();
}

test "Vec4 normalized / dot / length / reduce" {
    var rng: Rng = .{};
    var normalized: Checker = .{ .name = "Vec4.normalized" };
    var dot: Checker = .{ .name = "Vec4.dot" };
    var length: Checker = .{ .name = "Vec4.length" };
    var reduce_min: Checker = .{ .name = "Vec4.reduceMin" };
    var reduce_max: Checker = .{ .name = "Vec4.reduceMax" };
    for (0..iterations) |_| {
        const a = rng.nonZero(4, -1000, 1000);
        const b = rng.array(4, -1000, 1000);
        var expected: [4]f32 = undefined;
        jolt.jolt_vec4_normalized(&a, &expected);
        normalized.check(a, arr4(vec4(a).normalized()), expected);
        dot.check(.{ a, b }, vec4(a).dot(vec4(b)), jolt.jolt_vec4_dot(&a, &b));
        length.check(a, vec4(a).length(), jolt.jolt_vec4_length(&a));
        reduce_min.check(b, vec4(b).reduceMin(), jolt.jolt_vec4_reduce_min(&b));
        reduce_max.check(b, vec4(b).reduceMax(), jolt.jolt_vec4_reduce_max(&b));
    }
    try finishAll(&.{ &normalized, &dot, &length, &reduce_min, &reduce_max });
}

test "Vec4 compress / decompress unit vector" {
    var rng: Rng = .{};
    var compress: Checker = .{ .name = "Vec4.compressUnitVector" };
    var decompress: Checker = .{ .name = "Vec4.decompressUnitVector" };
    for (0..iterations) |_| {
        const v = rng.unitVector(4);
        const value = jolt.jolt_vec4_compress_unit_vector(&v);
        compress.check(v, vec4(v).compressUnitVector(), value);
        var expected: [4]f32 = undefined;
        jolt.jolt_vec4_decompress_unit_vector(value, &expected);
        decompress.check(value, arr4(Vec4.decompressUnitVector(value)), expected);
    }
    try finishAll(&.{ &compress, &decompress });
}

test "Vec3 normalized / cross / dot / length / perpendicular" {
    var rng: Rng = .{};
    var normalized: Checker = .{ .name = "Vec3.normalized" };
    var cross: Checker = .{ .name = "Vec3.cross" };
    var dot: Checker = .{ .name = "Vec3.dot" };
    var length: Checker = .{ .name = "Vec3.length" };
    var perpendicular: Checker = .{ .name = "Vec3.getNormalizedPerpendicular" };
    for (0..iterations) |_| {
        const a = rng.nonZero(3, -1000, 1000);
        const b = rng.array(3, -1000, 1000);
        var expected: [3]f32 = undefined;
        jolt.jolt_vec3_normalized(&a, &expected);
        normalized.check(a, arr3(vec3(a).normalized()), expected);
        jolt.jolt_vec3_cross(&a, &b, &expected);
        cross.check(.{ a, b }, arr3(vec3(a).cross(vec3(b))), expected);
        dot.check(.{ a, b }, vec3(a).dot(vec3(b)), jolt.jolt_vec3_dot(&a, &b));
        length.check(a, vec3(a).length(), jolt.jolt_vec3_length(&a));
        jolt.jolt_vec3_normalized_perpendicular(&a, &expected);
        perpendicular.check(a, arr3(vec3(a).getNormalizedPerpendicular()), expected);
    }
    try finishAll(&.{ &normalized, &cross, &dot, &length, &perpendicular });
}

test "Vec3 unitSpherical / compress / decompress" {
    var rng: Rng = .{};
    var spherical: Checker = .{ .name = "Vec3.unitSpherical" };
    var compress: Checker = .{ .name = "Vec3.compressUnitVector" };
    var decompress: Checker = .{ .name = "Vec3.decompressUnitVector" };
    for (0..iterations) |_| {
        const theta = rng.float(0, zolt.math.pi);
        const phi = rng.float(0, 2 * zolt.math.pi);
        var expected: [3]f32 = undefined;
        jolt.jolt_vec3_unit_spherical(theta, phi, &expected);
        spherical.check(.{ theta, phi }, arr3(Vec3.unitSpherical(theta, phi)), expected);

        const v = rng.unitVector(3);
        const value = jolt.jolt_vec3_compress_unit_vector(&v);
        compress.check(v, vec3(v).compressUnitVector(), value);
        jolt.jolt_vec3_decompress_unit_vector(value, &expected);
        decompress.check(value, arr3(Vec3.decompressUnitVector(value)), expected);
    }
    try finishAll(&.{ &spherical, &compress, &decompress });
}

test "Quat rotation / mul / mulVec3 / inverseRotate / multiplyImaginary / rotateAxis" {
    var rng: Rng = .{};
    var rotation: Checker = .{ .name = "Quat.rotation" };
    var mul: Checker = .{ .name = "Quat.mul" };
    var mul_vec3: Checker = .{ .name = "Quat.mulVec3" };
    var inverse_rotate: Checker = .{ .name = "Quat.inverseRotate" };
    var multiply_imaginary: Checker = .{ .name = "Quat.multiplyImaginary" };
    var rotate_axis_x: Checker = .{ .name = "Quat.rotateAxisX" };
    var rotate_axis_y: Checker = .{ .name = "Quat.rotateAxisY" };
    var rotate_axis_z: Checker = .{ .name = "Quat.rotateAxisZ" };
    for (0..iterations) |_| {
        var expected4: [4]f32 = undefined;
        var expected3: [3]f32 = undefined;

        const axis = rng.unitVector(3);
        const angle = rng.float(-10, 10);
        jolt.jolt_quat_rotation(&axis, angle, &expected4);
        rotation.check(.{ axis, angle }, arrQ(Quat.rotation(vec3(axis), angle)), expected4);

        // Multiplication of unit quaternions and of arbitrary quaternions
        const a = rng.unitQuat();
        const b = if (rng.next() % 2 == 0) rng.unitQuat() else rng.array(4, -10, 10);
        jolt.jolt_quat_mul(&a, &b, &expected4);
        mul.check(.{ a, b }, arrQ(quat(a).mul(quat(b))), expected4);

        const v = rng.array(3, -100, 100);
        jolt.jolt_quat_mul_vec3(&a, &v, &expected3);
        mul_vec3.check(.{ a, v }, arr3(quat(a).mulVec3(vec3(v))), expected3);
        jolt.jolt_quat_inverse_rotate(&a, &v, &expected3);
        inverse_rotate.check(.{ a, v }, arr3(quat(a).inverseRotate(vec3(v))), expected3);
        jolt.jolt_quat_multiply_imaginary(&v, &b, &expected4);
        multiply_imaginary.check(.{ v, b }, arrQ(Quat.multiplyImaginary(vec3(v), quat(b))), expected4);

        jolt.jolt_quat_rotate_axis_x(&a, &expected3);
        rotate_axis_x.check(a, arr3(quat(a).rotateAxisX()), expected3);
        jolt.jolt_quat_rotate_axis_y(&a, &expected3);
        rotate_axis_y.check(a, arr3(quat(a).rotateAxisY()), expected3);
        jolt.jolt_quat_rotate_axis_z(&a, &expected3);
        rotate_axis_z.check(a, arr3(quat(a).rotateAxisZ()), expected3);
    }
    try finishAll(&.{ &rotation, &mul, &mul_vec3, &inverse_rotate, &multiply_imaginary, &rotate_axis_x, &rotate_axis_y, &rotate_axis_z });
}

test "Quat inversed / conjugated / normalized / getAxisAngle / getSwingTwist / getTwist / getRotationAngle / Euler angles" {
    var rng: Rng = .{};
    var inversed: Checker = .{ .name = "Quat.inversed" };
    var conjugated: Checker = .{ .name = "Quat.conjugated" };
    var normalized: Checker = .{ .name = "Quat.normalized" };
    var ensure_w_positive: Checker = .{ .name = "Quat.ensureWPositive" };
    var axis_angle_axis: Checker = .{ .name = "Quat.getAxisAngle (axis)" };
    var axis_angle_angle: Checker = .{ .name = "Quat.getAxisAngle (angle)" };
    var angular_velocity: Checker = .{ .name = "Quat.getAngularVelocity" };
    var swing: Checker = .{ .name = "Quat.getSwingTwist (swing)" };
    var twist: Checker = .{ .name = "Quat.getSwingTwist (twist)" };
    var get_twist: Checker = .{ .name = "Quat.getTwist" };
    var rotation_angle: Checker = .{ .name = "Quat.getRotationAngle" };
    var get_euler_angles: Checker = .{ .name = "Quat.getEulerAngles" };
    var euler_angles: Checker = .{ .name = "Quat.eulerAngles" };
    const axes = [_][3]f32{ .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 }, .{ 0, -1, 0 } };
    for (0..iterations) |_| {
        var expected4: [4]f32 = undefined;
        var expected4b: [4]f32 = undefined;
        var expected3: [3]f32 = undefined;

        const q = rng.unitQuat();
        const any = if (rng.next() % 2 == 0) q else rng.nonZero(4, -10, 10);
        jolt.jolt_quat_inversed(&any, &expected4);
        inversed.check(any, arrQ(quat(any).inversed()), expected4);
        jolt.jolt_quat_conjugated(&any, &expected4);
        conjugated.check(any, arrQ(quat(any).conjugated()), expected4);
        jolt.jolt_quat_normalized(&any, &expected4);
        normalized.check(any, arrQ(quat(any).normalized()), expected4);
        jolt.jolt_quat_ensure_w_positive(&any, &expected4);
        ensure_w_positive.check(any, arrQ(quat(any).ensureWPositive()), expected4);

        var expected_angle: f32 = undefined;
        jolt.jolt_quat_get_axis_angle(&q, &expected3, &expected_angle);
        const axis_angle = quat(q).getAxisAngle();
        axis_angle_axis.check(q, arr3(axis_angle.axis), expected3);
        axis_angle_angle.check(q, axis_angle.angle, expected_angle);

        const delta_time = rng.float(1.0e-3, 1);
        jolt.jolt_quat_get_angular_velocity(&q, delta_time, &expected3);
        angular_velocity.check(.{ q, delta_time }, arr3(quat(q).getAngularVelocity(delta_time)), expected3);

        jolt.jolt_quat_get_swing_twist(&q, &expected4, &expected4b);
        const swing_twist = quat(q).getSwingTwist();
        swing.check(q, arrQ(swing_twist.swing), expected4);
        twist.check(q, arrQ(swing_twist.twist), expected4b);

        const axis = if (rng.next() % 4 == 0) axes[rng.next() % axes.len] else rng.unitVector(3);
        jolt.jolt_quat_get_twist(&q, &axis, &expected4);
        get_twist.check(.{ q, axis }, arrQ(quat(q).getTwist(vec3(axis))), expected4);
        rotation_angle.check(.{ q, axis }, quat(q).getRotationAngle(vec3(axis)), jolt.jolt_quat_get_rotation_angle(&q, &axis));

        jolt.jolt_quat_get_euler_angles(&q, &expected3);
        get_euler_angles.check(q, arr3(quat(q).getEulerAngles()), expected3);
        const angles = rng.array(3, -2 * zolt.math.pi, 2 * zolt.math.pi);
        jolt.jolt_quat_euler_angles(&angles, &expected4);
        euler_angles.check(angles, arrQ(Quat.eulerAngles(vec3(angles))), expected4);
    }
    try finishAll(&.{ &inversed, &conjugated, &normalized, &ensure_w_positive, &axis_angle_axis, &axis_angle_angle, &angular_velocity, &swing, &twist, &get_twist, &rotation_angle, &get_euler_angles, &euler_angles });
}

test "Quat fromTo / lerp / slerp" {
    var rng: Rng = .{};
    var from_to: Checker = .{ .name = "Quat.fromTo" };
    var lerp: Checker = .{ .name = "Quat.lerp" };
    var slerp: Checker = .{ .name = "Quat.slerp" };
    for (0..iterations) |_| {
        var expected: [4]f32 = undefined;

        const from: [3]f32 = if (rng.next() % 50 == 0) .{ 0, 0, 0 } else rng.nonZero(3, -10, 10);
        const to: [3]f32 = switch (rng.next() % 4) {
            0 => .{ -2 * from[0], -2 * from[1], -2 * from[2] }, // Opposite direction: takes the perpendicular branch
            1 => from,
            else => rng.nonZero(3, -10, 10),
        };
        jolt.jolt_quat_from_to(&from, &to, &expected);
        from_to.check(.{ from, to }, arrQ(Quat.fromTo(vec3(from), vec3(to))), expected);

        const a = rng.unitQuat();
        const b: [4]f32 = switch (rng.next() % 5) {
            0 => a, // Identical: linear interpolation
            1 => .{ -a[0], -a[1], -a[2], -a[3] }, // Opposite sign
            2 => blk: {
                // Very close: linear interpolation or barely slerp
                const d = rng.array(4, -1.0e-2, 1.0e-2);
                break :blk normalize(4, .{ a[0] + d[0], a[1] + d[1], a[2] + d[2], a[3] + d[3] });
            },
            else => rng.unitQuat(),
        };
        const fraction = rng.float(0, 1);
        jolt.jolt_quat_lerp(&a, &b, fraction, &expected);
        lerp.check(.{ a, b, fraction }, arrQ(quat(a).lerp(quat(b), fraction)), expected);
        jolt.jolt_quat_slerp(&a, &b, fraction, &expected);
        slerp.check(.{ a, b, fraction }, arrQ(quat(a).slerp(quat(b), fraction)), expected);
    }
    try finishAll(&.{ &from_to, &lerp, &slerp });
}

test "Mat44 constructors (rotation, lookAt, perspective, ...)" {
    var rng: Rng = .{};
    var rotation_x: Checker = .{ .name = "Mat44.rotationX" };
    var rotation_y: Checker = .{ .name = "Mat44.rotationY" };
    var rotation_z: Checker = .{ .name = "Mat44.rotationZ" };
    var rotation: Checker = .{ .name = "Mat44.rotation" };
    var rotation_quat: Checker = .{ .name = "Mat44.rotationQuat" };
    var inverse_rotation_translation: Checker = .{ .name = "Mat44.inverseRotationTranslation" };
    var outer_product: Checker = .{ .name = "Mat44.outerProduct" };
    var cross_product: Checker = .{ .name = "Mat44.crossProduct" };
    var quat_left_multiply: Checker = .{ .name = "Mat44.quatLeftMultiply" };
    var quat_right_multiply: Checker = .{ .name = "Mat44.quatRightMultiply" };
    var look_at: Checker = .{ .name = "Mat44.lookAt" };
    var perspective: Checker = .{ .name = "Mat44.perspective" };
    for (0..iterations) |_| {
        var expected: [16]f32 = undefined;

        const angle = rng.float(-10, 10);
        jolt.jolt_mat44_rotation_x(angle, &expected);
        rotation_x.check(angle, arr16(Mat44.rotationX(angle)), expected);
        jolt.jolt_mat44_rotation_y(angle, &expected);
        rotation_y.check(angle, arr16(Mat44.rotationY(angle)), expected);
        jolt.jolt_mat44_rotation_z(angle, &expected);
        rotation_z.check(angle, arr16(Mat44.rotationZ(angle)), expected);

        const axis = rng.unitVector(3);
        jolt.jolt_mat44_rotation(&axis, angle, &expected);
        rotation.check(.{ axis, angle }, arr16(Mat44.rotation(vec3(axis), angle)), expected);

        const q = rng.unitQuat();
        jolt.jolt_mat44_rotation_quat(&q, &expected);
        rotation_quat.check(q, arr16(Mat44.rotationQuat(quat(q))), expected);
        const t = rng.array(3, -100, 100);
        jolt.jolt_mat44_inverse_rotation_translation(&q, &t, &expected);
        inverse_rotation_translation.check(.{ q, t }, arr16(Mat44.inverseRotationTranslation(quat(q), vec3(t))), expected);

        const a = rng.array(3, -100, 100);
        const b = rng.array(3, -100, 100);
        jolt.jolt_mat44_outer_product(&a, &b, &expected);
        outer_product.check(.{ a, b }, arr16(Mat44.outerProduct(vec3(a), vec3(b))), expected);
        jolt.jolt_mat44_cross_product(&a, &expected);
        cross_product.check(a, arr16(Mat44.crossProduct(vec3(a))), expected);

        const any_q = rng.array(4, -10, 10);
        jolt.jolt_mat44_quat_left_multiply(&any_q, &expected);
        quat_left_multiply.check(any_q, arr16(Mat44.quatLeftMultiply(quat(any_q))), expected);
        jolt.jolt_mat44_quat_right_multiply(&any_q, &expected);
        quat_right_multiply.check(any_q, arr16(Mat44.quatRightMultiply(quat(any_q))), expected);

        // Look at, including degenerate cases (target == position, up parallel to the view direction)
        const pos = rng.array(3, -100, 100);
        const target = if (rng.next() % 20 == 0) pos else rng.array(3, -100, 100);
        const up: [3]f32 = switch (rng.next() % 10) {
            0 => .{ target[0] - pos[0], target[1] - pos[1], target[2] - pos[2] },
            1, 2, 3 => .{ 0, 1, 0 },
            else => rng.array(3, -1, 1),
        };
        jolt.jolt_mat44_look_at(&pos, &target, &up, &expected);
        look_at.check(.{ pos, target, up }, arr16(Mat44.lookAt(vec3(pos), vec3(target), vec3(up))), expected);

        const fov_y = rng.float(0.1, 3.0);
        const aspect = rng.float(0.5, 2.0);
        const near = rng.float(0.01, 1.0);
        const far = rng.float(10.0, 1000.0);
        jolt.jolt_mat44_perspective(fov_y, aspect, near, far, &expected);
        perspective.check(.{ fov_y, aspect, near, far }, arr16(Mat44.perspective(fov_y, aspect, near, far)), expected);
    }
    try finishAll(&.{ &rotation_x, &rotation_y, &rotation_z, &rotation, &rotation_quat, &inverse_rotation_translation, &outer_product, &cross_product, &quat_left_multiply, &quat_right_multiply, &look_at, &perspective });
}

test "Mat44 multiply / transpose / add / translate / scale" {
    var rng: Rng = .{};
    var mul: Checker = .{ .name = "Mat44.mul" };
    var mul_vec3: Checker = .{ .name = "Mat44.mulVec3" };
    var mul_vec4: Checker = .{ .name = "Mat44.mulVec4" };
    var multiply3x3: Checker = .{ .name = "Mat44.multiply3x3" };
    var multiply3x3_transposed: Checker = .{ .name = "Mat44.multiply3x3Transposed" };
    var multiply3x3_mat44: Checker = .{ .name = "Mat44.multiply3x3Mat44" };
    var multiply3x3_left_transposed: Checker = .{ .name = "Mat44.multiply3x3LeftTransposed" };
    var multiply3x3_right_transposed: Checker = .{ .name = "Mat44.multiply3x3RightTransposed" };
    var mul_scalar: Checker = .{ .name = "Mat44.mulScalar" };
    var add: Checker = .{ .name = "Mat44.add" };
    var sub: Checker = .{ .name = "Mat44.sub" };
    var negate: Checker = .{ .name = "Mat44.negate" };
    var transposed: Checker = .{ .name = "Mat44.transposed" };
    var transposed3x3: Checker = .{ .name = "Mat44.transposed3x3" };
    var pre_translated: Checker = .{ .name = "Mat44.preTranslated" };
    var post_translated: Checker = .{ .name = "Mat44.postTranslated" };
    var pre_scaled: Checker = .{ .name = "Mat44.preScaled" };
    var post_scaled: Checker = .{ .name = "Mat44.postScaled" };
    for (0..iterations) |_| {
        var expected: [16]f32 = undefined;
        var expected4: [4]f32 = undefined;
        var expected3: [3]f32 = undefined;

        const a = rng.matrix();
        const b = rng.matrix();
        const a3 = rng.matrix3x3();
        const v4 = rng.array(4, -100, 100);
        const v3 = rng.array(3, -100, 100);

        jolt.jolt_mat44_mul(&a, &b, &expected);
        mul.check(.{ a, b }, arr16(mat44(a).mul(mat44(b))), expected);
        jolt.jolt_mat44_mul_vec3(&a, &v3, &expected3);
        mul_vec3.check(.{ a, v3 }, arr3(mat44(a).mulVec3(vec3(v3))), expected3);
        jolt.jolt_mat44_mul_vec4(&a, &v4, &expected4);
        mul_vec4.check(.{ a, v4 }, arr4(mat44(a).mulVec4(vec4(v4))), expected4);
        jolt.jolt_mat44_multiply3x3(&a, &v3, &expected3);
        multiply3x3.check(.{ a, v3 }, arr3(mat44(a).multiply3x3(vec3(v3))), expected3);
        jolt.jolt_mat44_multiply3x3_transposed(&a, &v3, &expected3);
        multiply3x3_transposed.check(.{ a, v3 }, arr3(mat44(a).multiply3x3Transposed(vec3(v3))), expected3);
        jolt.jolt_mat44_multiply3x3_mat44(&a3, &b, &expected);
        multiply3x3_mat44.check(.{ a3, b }, arr16(mat44(a3).multiply3x3Mat44(mat44(b))), expected);
        jolt.jolt_mat44_multiply3x3_left_transposed(&a, &b, &expected);
        multiply3x3_left_transposed.check(.{ a, b }, arr16(mat44(a).multiply3x3LeftTransposed(mat44(b))), expected);
        jolt.jolt_mat44_multiply3x3_right_transposed(&a3, &b, &expected);
        multiply3x3_right_transposed.check(.{ a3, b }, arr16(mat44(a3).multiply3x3RightTransposed(mat44(b))), expected);

        const s = rng.float(-10, 10);
        jolt.jolt_mat44_mul_scalar(&a, s, &expected);
        mul_scalar.check(.{ a, s }, arr16(mat44(a).mulScalar(s)), expected);
        jolt.jolt_mat44_add(&a, &b, &expected);
        add.check(.{ a, b }, arr16(mat44(a).add(mat44(b))), expected);
        jolt.jolt_mat44_sub(&a, &b, &expected);
        sub.check(.{ a, b }, arr16(mat44(a).sub(mat44(b))), expected);
        jolt.jolt_mat44_negate(&a, &expected);
        negate.check(a, arr16(mat44(a).negate()), expected);
        jolt.jolt_mat44_transposed(&a, &expected);
        transposed.check(a, arr16(mat44(a).transposed()), expected);
        jolt.jolt_mat44_transposed3x3(&a, &expected);
        transposed3x3.check(a, arr16(mat44(a).transposed3x3()), expected);

        jolt.jolt_mat44_pre_translated(&a, &v3, &expected);
        pre_translated.check(.{ a, v3 }, arr16(mat44(a).preTranslated(vec3(v3))), expected);
        jolt.jolt_mat44_post_translated(&a, &v3, &expected);
        post_translated.check(.{ a, v3 }, arr16(mat44(a).postTranslated(vec3(v3))), expected);
        const scale = rng.array(3, -10, 10);
        jolt.jolt_mat44_pre_scaled(&a, &scale, &expected);
        pre_scaled.check(.{ a, scale }, arr16(mat44(a).preScaled(vec3(scale))), expected);
        jolt.jolt_mat44_post_scaled(&a, &scale, &expected);
        post_scaled.check(.{ a, scale }, arr16(mat44(a).postScaled(vec3(scale))), expected);
    }
    try finishAll(&.{ &mul, &mul_vec3, &mul_vec4, &multiply3x3, &multiply3x3_transposed, &multiply3x3_mat44, &multiply3x3_left_transposed, &multiply3x3_right_transposed, &mul_scalar, &add, &sub, &negate, &transposed, &transposed3x3, &pre_translated, &post_translated, &pre_scaled, &post_scaled });
}

test "Mat44 inverse / determinant / decompose / getQuaternion" {
    var rng: Rng = .{};
    var inversed: Checker = .{ .name = "Mat44.inversed" };
    var inversed_rotation_translation: Checker = .{ .name = "Mat44.inversedRotationTranslation" };
    var determinant3x3: Checker = .{ .name = "Mat44.getDeterminant3x3" };
    var adjointed3x3: Checker = .{ .name = "Mat44.adjointed3x3" };
    var inversed3x3: Checker = .{ .name = "Mat44.inversed3x3" };
    var set_inversed3x3: Checker = .{ .name = "Mat44.setInversed3x3" };
    var set_inversed3x3_result: Checker = .{ .name = "Mat44.setInversed3x3 (return value)" };
    var get_quaternion: Checker = .{ .name = "Mat44.getQuaternion" };
    var direction_preserving: Checker = .{ .name = "Mat44.getDirectionPreservingMatrix" };
    var decompose: Checker = .{ .name = "Mat44.decompose" };
    var decompose_scale: Checker = .{ .name = "Mat44.decompose (scale)" };
    for (0..iterations) |_| {
        var expected: [16]f32 = undefined;
        var expected4: [4]f32 = undefined;
        var expected3: [3]f32 = undefined;

        const m = rng.matrix();
        if (isWellConditioned4x4(m)) {
            jolt.jolt_mat44_inversed(&m, &expected);
            inversed.check(m, arr16(mat44(m).inversed()), expected);
        }

        const rt = if (rng.next() % 4 == 0) m else rng.rotationTranslation();
        jolt.jolt_mat44_inversed_rotation_translation(&rt, &expected);
        inversed_rotation_translation.check(rt, arr16(mat44(rt).inversedRotationTranslation()), expected);

        determinant3x3.check(m, mat44(m).getDeterminant3x3(), jolt.jolt_mat44_get_determinant3x3(&m));
        jolt.jolt_mat44_adjointed3x3(&m, &expected);
        adjointed3x3.check(m, arr16(mat44(m).adjointed3x3()), expected);
        if (isWellConditioned3x3(m)) {
            jolt.jolt_mat44_inversed3x3(&m, &expected);
            inversed3x3.check(m, arr16(mat44(m).inversed3x3()), expected);
        }

        // SetInversed3x3, sometimes on a singular matrix (det == 0, the target is left unchanged)
        var singular = m;
        if (rng.next() % 10 == 0) {
            singular[4] = 0;
            singular[5] = 0;
            singular[6] = 0;
        }
        const initial = rng.matrix();
        var zolt_target = mat44(initial);
        const zolt_result = zolt_target.setInversed3x3(mat44(singular));
        expected = initial;
        const jolt_result = jolt.jolt_mat44_set_inversed3x3(&singular, &expected);
        set_inversed3x3.check(singular, arr16(zolt_target), expected);
        set_inversed3x3_result.check(singular, zolt_result, jolt_result);

        // Quaternion from rotation matrices (all branches of the trace test) and from arbitrary matrices
        const r = if (rng.next() % 4 == 0) m else arr16(Mat44.rotationQuat(quat(rng.unitQuat())));
        jolt.jolt_mat44_get_quaternion(&r, &expected4);
        get_quaternion.check(r, arrQ(mat44(r).getQuaternion()), expected4);

        const m3 = rng.matrix3x3();
        if (isWellConditioned3x3(m3)) {
            jolt.jolt_mat44_get_direction_preserving_matrix(&m3, &expected);
            direction_preserving.check(m3, arr16(mat44(m3).getDirectionPreservingMatrix()), expected);
        }

        jolt.jolt_mat44_decompose(&m, &expected, &expected3);
        const d = mat44(m).decompose();
        decompose.check(m, arr16(d.rotation_translation), expected);
        decompose_scale.check(m, arr3(d.scale), expected3);
    }
    try finishAll(&.{ &inversed, &inversed_rotation_translation, &determinant3x3, &adjointed3x3, &inversed3x3, &set_inversed3x3, &set_inversed3x3_result, &get_quaternion, &direction_preserving, &decompose, &decompose_scale });
}

/// Random DVec3 input: mostly regular values, sometimes any magnitude / NaN / inf
fn dvec3Input(rng: *Rng) [3]f64 {
    return switch (rng.next() % 4) {
        0 => .{ rng.anyDouble(), rng.anyDouble(), rng.anyDouble() },
        1 => rng.doubleArray(3, -1.0e10, 1.0e10),
        else => rng.doubleArray(3, -1000, 1000),
    };
}

test "DVec3 arithmetic / normalized / length / dot / cross" {
    var rng: Rng = .{};
    var add: Checker = .{ .name = "DVec3.add" };
    var sub: Checker = .{ .name = "DVec3.sub" };
    var mul: Checker = .{ .name = "DVec3.mul" };
    var div: Checker = .{ .name = "DVec3.div" };
    var add_vec3: Checker = .{ .name = "DVec3.addVec3" };
    var sub_vec3: Checker = .{ .name = "DVec3.subVec3" };
    var mul_scalar: Checker = .{ .name = "DVec3.mulScalar" };
    var div_scalar: Checker = .{ .name = "DVec3.divScalar" };
    var negate: Checker = .{ .name = "DVec3.negate" };
    var abs: Checker = .{ .name = "DVec3.abs" };
    var reciprocal: Checker = .{ .name = "DVec3.reciprocal" };
    var sqrt: Checker = .{ .name = "DVec3.sqrt" };
    var get_sign: Checker = .{ .name = "DVec3.getSign" };
    var min: Checker = .{ .name = "DVec3.min" };
    var max: Checker = .{ .name = "DVec3.max" };
    var cross: Checker = .{ .name = "DVec3.cross" };
    var dot: Checker = .{ .name = "DVec3.dot" };
    var length_sq: Checker = .{ .name = "DVec3.lengthSq" };
    var length: Checker = .{ .name = "DVec3.length" };
    var normalized: Checker = .{ .name = "DVec3.normalized" };
    for (0..iterations) |_| {
        var expected: [3]f64 = undefined;

        const a = dvec3Input(&rng);
        const b = dvec3Input(&rng);
        const f = rng.array(3, -1000, 1000);
        const s = rng.double(-1000, 1000);

        jolt.jolt_dvec3_add(&a, &b, &expected);
        add.check(.{ a, b }, arrD3(dvec3(a).add(dvec3(b))), expected);
        jolt.jolt_dvec3_sub(&a, &b, &expected);
        sub.check(.{ a, b }, arrD3(dvec3(a).sub(dvec3(b))), expected);
        jolt.jolt_dvec3_mul(&a, &b, &expected);
        mul.check(.{ a, b }, arrD3(dvec3(a).mul(dvec3(b))), expected);
        jolt.jolt_dvec3_div(&a, &b, &expected);
        div.check(.{ a, b }, arrD3(dvec3(a).div(dvec3(b))), expected);
        jolt.jolt_dvec3_add_vec3(&a, &f, &expected);
        add_vec3.check(.{ a, f }, arrD3(dvec3(a).addVec3(vec3(f))), expected);
        jolt.jolt_dvec3_sub_vec3(&a, &f, &expected);
        sub_vec3.check(.{ a, f }, arrD3(dvec3(a).subVec3(vec3(f))), expected);
        jolt.jolt_dvec3_mul_scalar(&a, s, &expected);
        mul_scalar.check(.{ a, s }, arrD3(dvec3(a).mulScalar(s)), expected);
        jolt.jolt_dvec3_div_scalar(&a, s, &expected);
        div_scalar.check(.{ a, s }, arrD3(dvec3(a).divScalar(s)), expected);
        jolt.jolt_dvec3_negate(&a, &expected);
        negate.check(a, arrD3(dvec3(a).negate()), expected);
        jolt.jolt_dvec3_abs(&a, &expected);
        abs.check(a, arrD3(dvec3(a).abs()), expected);
        jolt.jolt_dvec3_reciprocal(&a, &expected);
        reciprocal.check(a, arrD3(dvec3(a).reciprocal()), expected);
        jolt.jolt_dvec3_sqrt(&a, &expected);
        sqrt.check(a, arrD3(dvec3(a).sqrt()), expected);
        if (!dvec3(a).isNaN()) {
            // Jolt's AVX512 path (_mm256_fixupimm_pd) returns NaN for a NaN component, its other paths return +/-1 like Zolt
            jolt.jolt_dvec3_get_sign(&a, &expected);
            get_sign.check(a, arrD3(dvec3(a).getSign()), expected);
        }
        jolt.jolt_dvec3_min(&a, &b, &expected);
        min.check(.{ a, b }, arrD3(DVec3.min(dvec3(a), dvec3(b))), expected);
        jolt.jolt_dvec3_max(&a, &b, &expected);
        max.check(.{ a, b }, arrD3(DVec3.max(dvec3(a), dvec3(b))), expected);
        jolt.jolt_dvec3_cross(&a, &b, &expected);
        cross.check(.{ a, b }, arrD3(dvec3(a).cross(dvec3(b))), expected);
        dot.check(.{ a, b }, dvec3(a).dot(dvec3(b)), jolt.jolt_dvec3_dot(&a, &b));
        length_sq.check(a, dvec3(a).lengthSq(), jolt.jolt_dvec3_length_sq(&a));
        length.check(a, dvec3(a).length(), jolt.jolt_dvec3_length(&a));
        jolt.jolt_dvec3_normalized(&a, &expected);
        normalized.check(a, arrD3(dvec3(a).normalized()), expected);
    }
    try finishAll(&.{ &add, &sub, &mul, &div, &add_vec3, &sub_vec3, &mul_scalar, &div_scalar, &negate, &abs, &reciprocal, &sqrt, &get_sign, &min, &max, &cross, &dot, &length_sq, &length, &normalized });
}

test "DVec3 toVec3 / prepareRoundToZero / prepareRoundToInf / toVec3RoundDown / toVec3RoundUp" {
    var rng: Rng = .{};
    var to_vec3: Checker = .{ .name = "DVec3.toVec3" };
    var round_to_zero: Checker = .{ .name = "DVec3.prepareRoundToZero" };
    var round_to_inf: Checker = .{ .name = "DVec3.prepareRoundToInf" };
    var round_down: Checker = .{ .name = "DVec3.toVec3RoundDown" };
    var round_up: Checker = .{ .name = "DVec3.toVec3RoundUp" };
    for (0..iterations) |_| {
        var expected: [3]f64 = undefined;
        var expected_f: [3]f32 = undefined;

        const v: [3]f64 = switch (rng.next() % 3) {
            0 => rng.doubleArray(3, -1000, 1000),
            // Doubles that are exactly representable as floats (nothing to round)
            1 => .{ rng.float(-1000, 1000), rng.anyFloat(), rng.float(-1.0e-30, 1.0e-30) },
            else => .{ rng.anyDouble(), rng.anyDouble(), rng.anyDouble() },
        };

        jolt.jolt_dvec3_to_vec3(&v, &expected_f);
        to_vec3.check(v, arr3(dvec3(v).toVec3()), expected_f);
        jolt.jolt_dvec3_prepare_round_to_zero(&v, &expected);
        round_to_zero.check(v, arrD3(dvec3(v).prepareRoundToZero()), expected);
        jolt.jolt_dvec3_prepare_round_to_inf(&v, &expected);
        round_to_inf.check(v, arrD3(dvec3(v).prepareRoundToInf()), expected);
        jolt.jolt_dvec3_to_vec3_round_down(&v, &expected_f);
        round_down.check(v, arr3(dvec3(v).toVec3RoundDown()), expected_f);
        jolt.jolt_dvec3_to_vec3_round_up(&v, &expected_f);
        round_up.check(v, arr3(dvec3(v).toVec3RoundUp()), expected_f);
    }
    try finishAll(&.{ &to_vec3, &round_to_zero, &round_to_inf, &round_down, &round_up });
}

test "DMat44 multiply / translate / scale / toMat44" {
    var rng: Rng = .{};
    var mul: Checker = .{ .name = "DMat44.mul" };
    var mul_mat44: Checker = .{ .name = "DMat44.mulMat44" };
    var mul_vec3: Checker = .{ .name = "DMat44.mulVec3" };
    var mul_dvec3: Checker = .{ .name = "DMat44.mulDVec3" };
    var multiply3x3: Checker = .{ .name = "DMat44.multiply3x3" };
    var multiply3x3_dvec3: Checker = .{ .name = "DMat44.multiply3x3DVec3" };
    var multiply3x3_transposed: Checker = .{ .name = "DMat44.multiply3x3Transposed" };
    var pre_translated: Checker = .{ .name = "DMat44.preTranslated" };
    var pre_translated_dvec3: Checker = .{ .name = "DMat44.preTranslatedDVec3" };
    var post_translated: Checker = .{ .name = "DMat44.postTranslated" };
    var post_translated_dvec3: Checker = .{ .name = "DMat44.postTranslatedDVec3" };
    var pre_scaled: Checker = .{ .name = "DMat44.preScaled" };
    var post_scaled: Checker = .{ .name = "DMat44.postScaled" };
    var to_mat44: Checker = .{ .name = "DMat44.toMat44" };
    for (0..iterations) |_| {
        var expected: DMat44Data = undefined;
        var expected_d3: [3]f64 = undefined;
        var expected_f3: [3]f32 = undefined;
        var expected16: [16]f32 = undefined;

        const a = rng.dmatrix();
        const b = rng.dmatrix();
        const m = rng.matrix();
        const v = rng.array(3, -1000, 1000);
        const dv = dvec3Input(&rng);

        jolt.jolt_dmat44_mul(&a.cols, &a.t, &b.cols, &b.t, &expected.cols, &expected.t);
        mul.check(.{ a, b }, DMat44Data.fromDMat44(a.toDMat44().mul(b.toDMat44())), expected);
        jolt.jolt_dmat44_mul_mat44(&a.cols, &a.t, &m, &expected.cols, &expected.t);
        mul_mat44.check(.{ a, m }, DMat44Data.fromDMat44(a.toDMat44().mulMat44(mat44(m))), expected);
        jolt.jolt_dmat44_mul_vec3(&a.cols, &a.t, &v, &expected_d3);
        mul_vec3.check(.{ a, v }, arrD3(a.toDMat44().mulVec3(vec3(v))), expected_d3);
        jolt.jolt_dmat44_mul_dvec3(&a.cols, &a.t, &dv, &expected_d3);
        mul_dvec3.check(.{ a, dv }, arrD3(a.toDMat44().mulDVec3(dvec3(dv))), expected_d3);
        jolt.jolt_dmat44_multiply3x3(&a.cols, &a.t, &v, &expected_f3);
        multiply3x3.check(.{ a, v }, arr3(a.toDMat44().multiply3x3(vec3(v))), expected_f3);
        jolt.jolt_dmat44_multiply3x3_dvec3(&a.cols, &a.t, &dv, &expected_d3);
        multiply3x3_dvec3.check(.{ a, dv }, arrD3(a.toDMat44().multiply3x3DVec3(dvec3(dv))), expected_d3);
        jolt.jolt_dmat44_multiply3x3_transposed(&a.cols, &a.t, &v, &expected_f3);
        multiply3x3_transposed.check(.{ a, v }, arr3(a.toDMat44().multiply3x3Transposed(vec3(v))), expected_f3);

        jolt.jolt_dmat44_pre_translated(&a.cols, &a.t, &v, &expected.cols, &expected.t);
        pre_translated.check(.{ a, v }, DMat44Data.fromDMat44(a.toDMat44().preTranslated(vec3(v))), expected);
        jolt.jolt_dmat44_pre_translated_dvec3(&a.cols, &a.t, &dv, &expected.cols, &expected.t);
        pre_translated_dvec3.check(.{ a, dv }, DMat44Data.fromDMat44(a.toDMat44().preTranslatedDVec3(dvec3(dv))), expected);
        jolt.jolt_dmat44_post_translated(&a.cols, &a.t, &v, &expected.cols, &expected.t);
        post_translated.check(.{ a, v }, DMat44Data.fromDMat44(a.toDMat44().postTranslated(vec3(v))), expected);
        jolt.jolt_dmat44_post_translated_dvec3(&a.cols, &a.t, &dv, &expected.cols, &expected.t);
        post_translated_dvec3.check(.{ a, dv }, DMat44Data.fromDMat44(a.toDMat44().postTranslatedDVec3(dvec3(dv))), expected);

        const scale = rng.array(3, -10, 10);
        jolt.jolt_dmat44_pre_scaled(&a.cols, &a.t, &scale, &expected.cols, &expected.t);
        pre_scaled.check(.{ a, scale }, DMat44Data.fromDMat44(a.toDMat44().preScaled(vec3(scale))), expected);
        jolt.jolt_dmat44_post_scaled(&a.cols, &a.t, &scale, &expected.cols, &expected.t);
        post_scaled.check(.{ a, scale }, DMat44Data.fromDMat44(a.toDMat44().postScaled(vec3(scale))), expected);

        jolt.jolt_dmat44_to_mat44(&a.cols, &a.t, &expected16);
        to_mat44.check(a, arr16(a.toDMat44().toMat44()), expected16);
    }
    try finishAll(&.{ &mul, &mul_mat44, &mul_vec3, &mul_dvec3, &multiply3x3, &multiply3x3_dvec3, &multiply3x3_transposed, &pre_translated, &pre_translated_dvec3, &post_translated, &post_translated_dvec3, &pre_scaled, &post_scaled, &to_mat44 });
}

test "DMat44 inverse / rotationTranslation / decompose" {
    var rng: Rng = .{};
    var inversed: Checker = .{ .name = "DMat44.inversed" };
    var inversed_rotation_translation: Checker = .{ .name = "DMat44.inversedRotationTranslation" };
    var rotation_translation: Checker = .{ .name = "DMat44.rotationTranslation" };
    var inverse_rotation_translation: Checker = .{ .name = "DMat44.inverseRotationTranslation" };
    var decompose: Checker = .{ .name = "DMat44.decompose" };
    var decompose_scale: Checker = .{ .name = "DMat44.decompose (scale)" };
    for (0..iterations) |_| {
        var expected: DMat44Data = undefined;
        var expected_f3: [3]f32 = undefined;

        const a = rng.dmatrix();
        const a16: [16]f32 = a.cols ++ [4]f32{ 0, 0, 0, 1 };
        if (isWellConditioned3x3(a16)) {
            jolt.jolt_dmat44_inversed(&a.cols, &a.t, &expected.cols, &expected.t);
            inversed.check(a, DMat44Data.fromDMat44(a.toDMat44().inversed()), expected);
        }
        jolt.jolt_dmat44_inversed_rotation_translation(&a.cols, &a.t, &expected.cols, &expected.t);
        inversed_rotation_translation.check(a, DMat44Data.fromDMat44(a.toDMat44().inversedRotationTranslation()), expected);

        const q = rng.unitQuat();
        jolt.jolt_dmat44_rotation_translation(&q, &a.t, &expected.cols, &expected.t);
        rotation_translation.check(.{ q, a.t }, DMat44Data.fromDMat44(DMat44.rotationTranslation(quat(q), dvec3(a.t))), expected);
        jolt.jolt_dmat44_inverse_rotation_translation(&q, &a.t, &expected.cols, &expected.t);
        inverse_rotation_translation.check(.{ q, a.t }, DMat44Data.fromDMat44(DMat44.inverseRotationTranslation(quat(q), dvec3(a.t))), expected);

        jolt.jolt_dmat44_decompose(&a.cols, &a.t, &expected.cols, &expected.t, &expected_f3);
        const d = a.toDMat44().decompose();
        decompose.check(a, DMat44Data.fromDMat44(d.rotation_translation), expected);
        decompose_scale.check(a, arr3(d.scale), expected_f3);
    }
    try finishAll(&.{ &inversed, &inversed_rotation_translation, &rotation_translation, &inverse_rotation_translation, &decompose, &decompose_scale });
}

/// Map all half float NaNs to one value (NaN payloads are not part of the determinism contract)
fn canonicalHalf(h: u16) u16 {
    const is_nan = (h & 0x7c00) == 0x7c00 and (h & 0x03ff) != 0;
    return if (is_nan) 0x7e00 else h;
}

test "HalfFloat toFloat (all half floats)" {
    var to_float: Checker = .{ .name = "half_float.toFloat" };
    var to_float_fallback: Checker = .{ .name = "half_float.toFloat vs C++ ToFloatFallback" };
    var i: u32 = 0;
    while (i < 0x10000) : (i += 4) {
        const halfs = [4]u16{ @intCast(i), @intCast(i + 1), @intCast(i + 2), @intCast(i + 3) };
        const packed_halfs = UVec4.init(i | ((i + 1) << 16), (i + 2) | ((i + 3) << 16), 0, 0);
        const result = arr4(half_float.toFloat(packed_halfs));
        var expected: [4]f32 = undefined;
        jolt.jolt_half_to_float(&halfs, &expected);
        to_float.check(halfs, result, expected);
        jolt.jolt_half_to_float_fallback(&halfs, &expected);
        to_float_fallback.check(halfs, result, expected);
    }
    try finishAll(&.{ &to_float, &to_float_fallback });
}

const rounding_modes = [_]half_float.RoundingMode{ .round_to_neg_inf, .round_to_pos_inf, .round_to_nearest };

/// Compare half_float.fromFloat in all rounding modes with the C++ FromFloat (checkers[0]) and FromFloatFallback (checkers[1])
fn checkFromFloat(checkers: *[2][rounding_modes.len]Checker, v: f32) void {
    inline for (rounding_modes, 0..) |mode, m| {
        const result = canonicalHalf(half_float.fromFloat(mode, v));
        checkers[0][m].check(v, result, canonicalHalf(jolt.jolt_half_from_float(v, @intFromEnum(mode))));
        checkers[1][m].check(v, result, canonicalHalf(jolt.jolt_half_from_float_fallback(v, @intFromEnum(mode))));
    }
}

test "HalfFloat fromFloat (all rounding modes)" {
    var checkers: [2][rounding_modes.len]Checker = undefined;
    inline for (rounding_modes, 0..) |mode, m| {
        checkers[0][m] = .{ .name = "half_float.fromFloat(." ++ @tagName(mode) ++ ") vs C++ FromFloat" };
        checkers[1][m] = .{ .name = "half_float.fromFloat(." ++ @tagName(mode) ++ ") vs C++ FromFloatFallback" };
    }

    // Every half float, its neighbors and the midpoints between consecutive half floats (the rounding boundaries)
    var h: u32 = 0;
    while (h < 0x10000) : (h += 1) {
        const halfs = UVec4.init(h | (((h + 1) & 0xffff) << 16), 0, 0, 0);
        const floats = half_float.toFloat(halfs);
        const f = floats.getX();
        if (std.math.isNan(f)) continue;
        checkFromFloat(&checkers, f);
        checkFromFloat(&checkers, std.math.nextAfter(f32, f, std.math.inf(f32)));
        checkFromFloat(&checkers, std.math.nextAfter(f32, f, -std.math.inf(f32)));
        if ((h & 0x7fff) < 0x7bff) {
            // Midpoint to the next half float of larger magnitude (exactly representable as a float)
            const next = floats.getY();
            const mid: f32 = @floatCast((@as(f64, f) + @as(f64, next)) * 0.5);
            checkFromFloat(&checkers, mid);
            checkFromFloat(&checkers, std.math.nextAfter(f32, mid, std.math.inf(f32)));
            checkFromFloat(&checkers, std.math.nextAfter(f32, mid, -std.math.inf(f32)));
        }
    }

    // Random floats of any bit pattern and in the half float range
    var rng: Rng = .{};
    for (0..iterations) |_| {
        checkFromFloat(&checkers, rng.anyFloat());
        checkFromFloat(&checkers, rng.float(-70000, 70000));
        checkFromFloat(&checkers, rng.float(-1.0e-4, 1.0e-4));
    }

    const c = &checkers;
    try finishAll(&.{ &c[0][0], &c[0][1], &c[0][2], &c[1][0], &c[1][1], &c[1][2] });
}
