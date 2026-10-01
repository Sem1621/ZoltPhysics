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
        const v = self.nonZero(n, -1, 1);
        var len_sq: f32 = 0;
        for (v) |c| len_sq += c * c;
        var result: [n]f32 = undefined;
        for (&result, v) |*r, c| r.* = c / @sqrt(len_sq);
        return result;
    }
};

/// Bitwise equality, all NaNs are considered equal (NaN payloads are not part of the determinism contract)
fn sameBits(a: f32, b: f32) bool {
    if (std.math.isNan(a) and std.math.isNan(b)) return true;
    return @as(u32, @bitCast(a)) == @as(u32, @bitCast(b));
}

/// Collects mismatches of one test and reports the first few
const Checker = struct {
    name: []const u8,
    mismatches: usize = 0,

    fn check(self: *Checker, input: anytype, zolt_result: anytype, jolt_result: @TypeOf(zolt_result)) void {
        const equal = switch (@typeInfo(@TypeOf(zolt_result))) {
            .float => sameBits(zolt_result, jolt_result),
            .int => zolt_result == jolt_result,
            .array => blk: {
                for (zolt_result, jolt_result) |a, b| {
                    if (!sameBits(a, b)) break :blk false;
                }
                break :blk true;
            },
            else => @compileError("unsupported result type"),
        };
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
    try sin_checker.finish();
    try cos_checker.finish();
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
    try normalized.finish();
    try dot.finish();
    try length.finish();
    try reduce_min.finish();
    try reduce_max.finish();
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
    try compress.finish();
    try decompress.finish();
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
    try normalized.finish();
    try cross.finish();
    try dot.finish();
    try length.finish();
    try perpendicular.finish();
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
    try spherical.finish();
    try compress.finish();
    try decompress.finish();
}
