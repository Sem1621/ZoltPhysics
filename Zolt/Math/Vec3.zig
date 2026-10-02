//! Port of: Jolt/Math/Vec3.h, Jolt/Math/Vec3.inl, Jolt/Math/Vec3.cpp
//! Status: complete
//!
//! Like Jolt, a Vec3 is stored in 4 floats (16 bytes, SIMD friendly). Zolt always keeps the
//! W component equal to Z (Jolt only guarantees this with JPH_FLOATING_POINT_EXCEPTIONS_ENABLED),
//! so W never holds garbage, and divisions never divide by an uninitialized lane.

const std = @import("std");
const math = @import("Math.zig");
const HashCombine = @import("../Core/HashCombine.zig");
const Swizzle = @import("Swizzle.zig").Swizzle;
const Float3 = @import("Float3.zig").Float3;
const UVec4 = @import("UVec4.zig").UVec4;
const Vec4 = @import("Vec4.zig").Vec4;

/// 3 component vector (stored as 4 vectors).
/// Note that we keep the 4th component the same as the 3rd component to avoid divisions by zero when JPH_FLOATING_POINT_EXCEPTIONS_ENABLED defined
pub const Vec3 = extern struct {
    pub const Type = @Vector(4, f32);

    value: Type,

    /// Create a vector from 3 components
    pub fn init(x: f32, y: f32, z: f32) Vec3 {
        return .{ .value = .{ x, y, z, z } };
    }

    /// Create from a raw SIMD value, W is replaced by Z (Vec3(Type))
    pub fn fromType(v: Type) Vec3 {
        return .{ .value = fixW(v) };
    }

    /// Convert from Vec4, the W component is dropped (explicit Vec3(Vec4Arg))
    pub fn fromVec4(v: Vec4) Vec3 {
        return fromType(v.value);
    }

    /// Load 3 floats from memory (explicit Vec3(const Float3 &))
    pub fn fromFloat3(v: Float3) Vec3 {
        return init(v.x, v.y, v.z);
    }

    /// Vector with all zeros
    pub fn zero() Vec3 {
        return replicate(0.0);
    }

    /// Vector with all ones
    pub fn one() Vec3 {
        return replicate(1.0);
    }

    /// Vector with all NaN's
    pub fn nan() Vec3 {
        return replicate(std.math.nan(f32));
    }

    /// Vectors with the principal axis
    pub fn axisX() Vec3 {
        return init(1, 0, 0);
    }
    pub fn axisY() Vec3 {
        return init(0, 1, 0);
    }
    pub fn axisZ() Vec3 {
        return init(0, 0, 1);
    }

    /// Replicate v across all components
    pub fn replicate(v: f32) Vec3 {
        return .{ .value = @splat(v) };
    }

    /// Load 3 floats from memory (reads 32 bits extra which it doesn't use in Jolt, Zolt only reads 3 floats)
    pub fn loadFloat3Unsafe(v: *const Float3) Vec3 {
        return init(v.x, v.y, v.z);
    }

    /// Return the minimum value of each of the components (returns v2 when not v1 < v2, like _mm_min_ps)
    pub fn min(v1: Vec3, v2: Vec3) Vec3 {
        return .{ .value = @select(f32, v1.value < v2.value, v1.value, v2.value) };
    }

    /// Return the maximum of each of the components (returns v2 when not v1 > v2, like _mm_max_ps)
    pub fn max(v1: Vec3, v2: Vec3) Vec3 {
        return .{ .value = @select(f32, v1.value > v2.value, v1.value, v2.value) };
    }

    /// Clamp a vector between min and max (component wise)
    pub fn clamp(v: Vec3, min_v: Vec3, max_v: Vec3) Vec3 {
        return max(min(v, max_v), min_v);
    }

    /// Equals (component wise), W is the same as Z
    pub fn equals(v1: Vec3, v2: Vec3) UVec4 {
        return UVec4.fromBools(v1.value == v2.value);
    }

    /// Less than (component wise), W is the same as Z
    pub fn less(v1: Vec3, v2: Vec3) UVec4 {
        return UVec4.fromBools(v1.value < v2.value);
    }

    /// Less than or equal (component wise), W is the same as Z
    pub fn lessOrEqual(v1: Vec3, v2: Vec3) UVec4 {
        return UVec4.fromBools(v1.value <= v2.value);
    }

    /// Greater than (component wise), W is the same as Z
    pub fn greater(v1: Vec3, v2: Vec3) UVec4 {
        return UVec4.fromBools(v1.value > v2.value);
    }

    /// Greater than or equal (component wise), W is the same as Z
    pub fn greaterOrEqual(v1: Vec3, v2: Vec3) UVec4 {
        return UVec4.fromBools(v1.value >= v2.value);
    }

    /// Calculates mul1 * mul2 + add. Never fused in Zolt, to stay cross platform deterministic.
    pub fn fusedMultiplyAdd(mul1: Vec3, mul2: Vec3, add_v: Vec3) Vec3 {
        return .{ .value = mul1.value * mul2.value + add_v.value };
    }

    /// Component wise select, returns not_set when highest bit of control = 0 and set when highest bit of control = 1
    pub fn select(not_set_value: Vec3, set_value: Vec3, control: UVec4) Vec3 {
        return fromType(@select(f32, control.toBools(), set_value.value, not_set_value.value));
    }

    /// Logical or (component wise) (sOr)
    pub fn bitOr(v1: Vec3, v2: Vec3) Vec3 {
        return .{ .value = UVec4.bitOr(v1.reinterpretAsInt(), v2.reinterpretAsInt()).reinterpretAsFloat().value };
    }

    /// Logical xor (component wise) (sXor)
    pub fn bitXor(v1: Vec3, v2: Vec3) Vec3 {
        return .{ .value = UVec4.bitXor(v1.reinterpretAsInt(), v2.reinterpretAsInt()).reinterpretAsFloat().value };
    }

    /// Logical and (component wise) (sAnd)
    pub fn bitAnd(v1: Vec3, v2: Vec3) Vec3 {
        return .{ .value = UVec4.bitAnd(v1.reinterpretAsInt(), v2.reinterpretAsInt()).reinterpretAsFloat().value };
    }

    /// Get unit vector given spherical coordinates
    /// theta in [0, pi] is angle between vector and z-axis, phi in [0, 2 pi] is the angle in the xy-plane starting from the x axis and rotating counter clockwise around the z-axis
    pub fn unitSpherical(theta: f32, phi: f32) Vec3 {
        const sc = Vec4.init(theta, phi, 0, 0).sinCos();
        return init(sc.sin.getX() * sc.cos.getY(), sc.sin.getX() * sc.sin.getY(), sc.cos.getX());
    }

    /// Get random unit vector.
    /// `rng` is a pointer to a random bit generator (see Core/Mt19937.zig), the equivalent of a C++
    /// UniformRandomBitGenerator: it needs `next() -> u32` and `min_value` / `max_value` declarations.
    pub fn random(rng: anytype) Vec3 {
        const R = @TypeOf(rng.*);
        const range: f32 = @floatFromInt(R.max_value - R.min_value);

        // Generating uniform unit random vectors in Rn - Andersen Ang
        // See: https://angms.science/doc/RM/randUnitVec.pdf
        const z = -1.0 + 2.0 * @as(f32, @floatFromInt(rng.next() - R.min_value)) / range;
        const r = @sqrt(1.0 - math.square(z));
        const theta = 2.0 * math.pi * @as(f32, @floatFromInt(rng.next() - R.min_value)) / range;
        const sc = Vec4.replicate(theta).sinCos();
        return init(r * sc.sin.getX(), r * sc.cos.getX(), z);
    }

    /// Get individual components
    pub fn getX(self: Vec3) f32 {
        return self.value[0];
    }
    pub fn getY(self: Vec3) f32 {
        return self.value[1];
    }
    pub fn getZ(self: Vec3) f32 {
        return self.value[2];
    }

    /// Set individual components
    pub fn setX(self: *Vec3, x: f32) void {
        self.value[0] = x;
    }
    pub fn setY(self: *Vec3, y: f32) void {
        self.value[1] = y;
    }
    pub fn setZ(self: *Vec3, z: f32) void {
        // Assure Z and W are the same
        self.value[2] = z;
        self.value[3] = z;
    }

    /// Set all components
    pub fn set(self: *Vec3, x: f32, y: f32, z: f32) void {
        self.* = init(x, y, z);
    }

    /// Get float component by index (operator [] const)
    pub fn getComponent(self: Vec3, coordinate: u32) f32 {
        std.debug.assert(coordinate < 3);
        const components: [4]@TypeOf(self.value[0]) = self.value; // Vectors can only be indexed with comptime indices
        return components[coordinate];
    }

    /// Set float component by index
    pub fn setComponent(self: *Vec3, coordinate: u32, v: f32) void {
        std.debug.assert(coordinate < 3);
        var components: [4]@TypeOf(self.value[0]) = self.value; // Vectors can only be indexed with comptime indices
        components[coordinate] = v;
        self.value = components;
        self.value = fixW(self.value); // Assure Z and W are the same
    }

    /// Comparison (operator ==)
    pub fn eql(self: Vec3, other: Vec3) bool {
        return equals(self, other).testAllXYZTrue();
    }

    /// Test if two vectors are close
    pub fn isClose(self: Vec3, other: Vec3, opts: struct { max_dist_sq: f32 = 1.0e-12 }) bool {
        return other.sub(self).lengthSq() <= opts.max_dist_sq;
    }

    /// Test if vector is near zero
    pub fn isNearZero(self: Vec3, opts: struct { max_dist_sq: f32 = 1.0e-12 }) bool {
        return self.lengthSq() <= opts.max_dist_sq;
    }

    /// Test if length^2 of this vector is within the range [1 - tolerance, 1 + tolerance]
    pub fn isNormalized(self: Vec3, opts: struct { tolerance: f32 = 1.0e-6 }) bool {
        return @abs(self.lengthSq() - 1.0) <= opts.tolerance;
    }

    /// Test if vector contains NaN elements
    pub fn isNaN(self: Vec3) bool {
        return @reduce(.Or, self.value != self.value);
    }

    /// Multiply two float vectors (component wise) (operator *)
    pub fn mul(self: Vec3, other: Vec3) Vec3 {
        return .{ .value = self.value * other.value };
    }

    /// Multiply vector with float (operator * (float))
    pub fn mulScalar(self: Vec3, v: f32) Vec3 {
        return .{ .value = self.value * @as(Type, @splat(v)) };
    }

    /// Divide vector by float (operator / (float))
    pub fn divScalar(self: Vec3, v: f32) Vec3 {
        return .{ .value = self.value / @as(Type, @splat(v)) };
    }

    /// Add two float vectors (component wise) (operator +)
    pub fn add(self: Vec3, other: Vec3) Vec3 {
        return .{ .value = self.value + other.value };
    }

    /// Negate (operator - ()). Computed as 0 - v, which maps -0 to +0 (JPH_CROSS_PLATFORM_DETERMINISTIC).
    pub fn negate(self: Vec3) Vec3 {
        return .{ .value = @as(Type, @splat(0.0)) - self.value };
    }

    /// Subtract two float vectors (component wise) (operator -)
    pub fn sub(self: Vec3, other: Vec3) Vec3 {
        return .{ .value = self.value - other.value };
    }

    /// Same as `add`. Exists so that `RVec3 + Vec3` is written `addVec3` in single and double precision (see DVec3.addVec3)
    pub const addVec3 = add;

    /// Same as `sub`. Exists so that `RVec3 - Vec3` is written `subVec3` in single and double precision (see DVec3.subVec3)
    pub const subVec3 = sub;

    /// Divide (component wise) (operator /)
    pub fn div(self: Vec3, other: Vec3) Vec3 {
        return .{ .value = self.value / other.value };
    }

    /// Swizzle the elements in the vector
    pub fn swizzle(self: Vec3, comptime sx: Swizzle, comptime sy: Swizzle, comptime sz: Swizzle) Vec3 {
        comptime std.debug.assert(sx != .w and sy != .w and sz != .w);
        return .{ .value = @shuffle(f32, self.value, undefined, @Vector(4, i32){ @intFromEnum(sx), @intFromEnum(sy), @intFromEnum(sz), @intFromEnum(sz) }) };
    }

    /// Replicate the X component to all components
    pub fn splatX(self: Vec3) Vec4 {
        return Vec4.replicate(self.value[0]);
    }

    /// Replicate the Y component to all components
    pub fn splatY(self: Vec3) Vec4 {
        return Vec4.replicate(self.value[1]);
    }

    /// Replicate the Z component to all components
    pub fn splatZ(self: Vec3) Vec4 {
        return Vec4.replicate(self.value[2]);
    }

    /// Get index of component with lowest value
    pub fn getLowestComponentIndex(self: Vec3) u32 {
        const x = self.getX();
        const y = self.getY();
        const z = self.getZ();
        return if (x < y) (if (z < x) 2 else 0) else (if (z < y) 2 else 1);
    }

    /// Get index of component with highest value
    pub fn getHighestComponentIndex(self: Vec3) u32 {
        const x = self.getX();
        const y = self.getY();
        const z = self.getZ();
        return if (x > y) (if (z > x) 2 else 0) else (if (z > y) 2 else 1);
    }

    /// Return the absolute value of each of the components
    pub fn abs(self: Vec3) Vec3 {
        // Like Jolt's SSE/AVX path: max(0 - v, v), which keeps -0 as -0 (the scalar fallback, NEON and AVX512 return +0)
        return max(self.negate(), self);
    }

    /// Reciprocal vector (1 / value) for each of the components
    pub fn reciprocal(self: Vec3) Vec3 {
        return one().div(self);
    }

    /// Calculates a * b - c * d. Never fused in Zolt, see math.differenceOfProducts.
    pub fn differenceOfProducts(a: Vec3, b: Vec3, c: Vec3, d: Vec3) Vec3 {
        return a.mul(b).sub(c.mul(d));
    }

    /// Cross product
    pub fn cross(self: Vec3, other: Vec3) Vec3 {
        const a = self.value;
        const b = other.value;
        return init(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
    }

    /// Cross product, more precise when FMA instructions are available (identical to `cross` in Zolt)
    pub fn crossPrecise(self: Vec3, other: Vec3) Vec3 {
        return differenceOfProducts(self, other.swizzle(.y, .z, .x), self.swizzle(.y, .z, .x), other).swizzle(.y, .z, .x);
    }

    /// Dot product, returns the dot product in X, Y and Z components
    pub fn dotV(self: Vec3, other: Vec3) Vec3 {
        return replicate(self.dot(other));
    }

    /// Dot product, returns the dot product in X, Y, Z and W components
    pub fn dotV4(self: Vec3, other: Vec3) Vec4 {
        return Vec4.replicate(self.dot(other));
    }

    /// Dot product
    pub fn dot(self: Vec3, other: Vec3) f32 {
        return self.mul(other).reduceSum();
    }

    /// Squared length of vector
    pub fn lengthSq(self: Vec3) f32 {
        return self.dot(self);
    }

    /// Length of vector
    pub fn length(self: Vec3) f32 {
        return @sqrt(self.lengthSq());
    }

    /// Normalize vector
    pub fn normalized(self: Vec3) Vec3 {
        return self.divScalar(self.length());
    }

    /// Normalize vector or return zero_value if the length of the vector is zero
    pub fn normalizedOr(self: Vec3, zero_value: Vec3) Vec3 {
        const len_sq = self.lengthSq();
        if (len_sq <= math.flt_min)
            return zero_value
        else
            return self.divScalar(@sqrt(len_sq));
    }

    /// Store 3 floats to memory
    pub fn storeFloat3(self: Vec3, out: *Float3) void {
        out.* = .{ .x = self.value[0], .y = self.value[1], .z = self.value[2] };
    }

    /// Convert each component from a float to an int (truncating), W is the same as Z.
    /// Follows Jolt's SSE path (_mm_cvttps_epi32): NaN and components outside the range of an i32 give 0x80000000.
    pub fn toInt(self: Vec3) UVec4 {
        return Vec4.fromVec3(self).toInt();
    }

    /// Reinterpret Vec3 as a UVec4 (doesn't change the bits), W is the same as Z
    pub fn reinterpretAsInt(self: Vec3) UVec4 {
        return .{ .value = @bitCast(self.value) };
    }

    /// Get the minimum of X, Y and Z
    pub fn reduceMin(self: Vec3) f32 {
        var v = min(self, self.swizzle(.y, .unused, .z));
        v = min(v, v.swizzle(.z, .unused, .unused));
        return v.getX();
    }

    /// Get the maximum of X, Y and Z
    pub fn reduceMax(self: Vec3) f32 {
        var v = max(self, self.swizzle(.y, .unused, .z));
        v = max(v, v.swizzle(.z, .unused, .unused));
        return v.getX();
    }

    /// Sum X, Y and Z
    pub fn reduceSum(self: Vec3) f32 {
        // Ensure that we handle -0.0f correctly when cross platform deterministic behavior is required.
        return (self.value[0] + self.value[1]) + (self.value[2] + 0.0);
    }

    /// Component wise square root
    pub fn sqrt(self: Vec3) Vec3 {
        return .{ .value = @sqrt(self.value) };
    }

    /// Get normalized vector that is perpendicular to this vector
    pub fn getNormalizedPerpendicular(self: Vec3) Vec3 {
        const x = self.value[0];
        const y = self.value[1];
        const z = self.value[2];
        const xx = x * x;
        const yy = y * y;
        const zz = z * z;
        const perp_x = init(z, 0.0, 0.0 - x);
        const perp_y = init(0.0, z, 0.0 - y);
        return (if (xx > yy) perp_x else perp_y).divScalar(@sqrt(math.max(xx, yy) + zz));
    }

    /// Get vector that contains the sign of each element (returns 1.0 if positive, -1.0 if negative)
    pub fn getSign(self: Vec3) Vec3 {
        const one_bits: UVec4 = one().reinterpretAsInt();
        const sign_bits = UVec4.bitAnd(self.reinterpretAsInt(), UVec4.replicate(0x80000000));
        return .{ .value = UVec4.bitOr(one_bits, sign_bits).reinterpretAsFloat().value };
    }

    /// Flips the signs of the components, e.g. flipSign(-1, 1, -1) will flip the signs of the X and Z components
    pub fn flipSign(self: Vec3, comptime x: comptime_int, comptime y: comptime_int, comptime z: comptime_int) Vec3 {
        comptime {
            for ([_]comptime_int{ x, y, z }) |s| {
                if (s != 1 and s != -1) @compileError("flipSign arguments must be 1 or -1");
            }
        }
        const mask = comptime init(if (x > 0) 0.0 else -0.0, if (y > 0) 0.0 else -0.0, if (z > 0) 0.0 else -0.0);
        return bitXor(self, mask);
    }

    /// Compress a unit vector to a 32 bit value, precision is around 10^-4
    pub fn compressUnitVector(self: Vec3) u32 {
        const one_over_sqrt2: f32 = 0.70710678;
        const num_bits = 14;
        const mask: u32 = (1 << num_bits) - 1;
        const max_value: u32 = mask - 1; // Need odd number of buckets to quantize to or else we can't encode 0
        const scale: f32 = @as(f32, @floatFromInt(max_value)) / (2.0 * one_over_sqrt2);

        // Store sign bit
        var v = self;
        const max_element = v.abs().getHighestComponentIndex();
        var value: u32 = 0;
        if (v.getComponent(max_element) < 0.0) {
            value = 0x80000000;
            v = v.negate();
        }

        // Store highest component
        value |= max_element << 29;

        // Store the other two components in a compressed format
        var compressed = clamp(v.add(replicate(one_over_sqrt2)).mulScalar(scale).add(replicate(0.5)), zero(), replicate(@floatFromInt(max_value))).toInt();
        switch (max_element) {
            0 => compressed = compressed.swizzle(.y, .z, .unused, .unused),
            1 => compressed = compressed.swizzle(.x, .z, .unused, .unused),
            else => {},
        }

        value |= compressed.getX();
        value |= compressed.getY() << num_bits;
        return value;
    }

    /// Decompress a unit vector from a 32 bit value
    pub fn decompressUnitVector(value: u32) Vec3 {
        const one_over_sqrt2: f32 = 0.70710678;
        const num_bits = 14;
        const mask: u32 = (1 << num_bits) - 1;
        const max_value: u32 = mask - 1; // Need odd number of buckets to quantize to or else we can't encode 0
        const half_max_value: i32 = @intCast(max_value >> 1);
        const scale: f32 = 2.0 * one_over_sqrt2 / @as(f32, @floatFromInt(max_value));

        // Restore two components
        const cx: i32 = @as(i32, @intCast(value & mask)) - half_max_value;
        const cy: i32 = @as(i32, @intCast((value >> num_bits) & mask)) - half_max_value;
        var v = init(@floatFromInt(cx), @floatFromInt(cy), 0).mulScalar(scale);
        std.debug.assert(v.getZ() == 0.0);

        // Restore the highest component
        v.setZ(@sqrt(math.max(1.0 - v.lengthSq(), 0.0)));

        // Extract sign
        if ((value & 0x80000000) != 0)
            v = v.negate();

        // Swizzle the components in place
        switch ((value >> 29) & 3) {
            0 => v = v.swizzle(.z, .x, .y),
            1 => v = v.swizzle(.x, .z, .y),
            else => {},
        }

        return v;
    }

    /// JPH_MAKE_HASHABLE(JPH::Vec3, t.GetX(), t.GetY(), t.GetZ())
    pub fn getHash(self: Vec3) u64 {
        return HashCombine.hashCombineArgs(.{ self.getX(), self.getY(), self.getZ() });
    }

    pub fn format(self: Vec3, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try writer.print("{d}, {d}, {d}", .{ self.value[0], self.value[1], self.value[2] });
    }

    /// Internal helper function that ensures that the Z component is replicated to the W component (sFixW)
    fn fixW(v: Type) Type {
        return @shuffle(f32, v, undefined, @Vector(4, i32){ 0, 1, 2, 2 });
    }
};
