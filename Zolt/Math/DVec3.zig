//! Port of: Jolt/Math/DVec3.h, Jolt/Math/DVec3.inl
//! Status: complete
//!
//! Like Jolt, a DVec3 is stored in 4 doubles (32 bytes, SIMD friendly). Zolt always keeps the
//! W component equal to Z (Jolt only guarantees this with JPH_FLOATING_POINT_EXCEPTIONS_ENABLED),
//! so W never holds garbage, and divisions never divide by an uninitialized lane. Because of this
//! Jolt's CheckW() is not needed.
//!
//! Semantics follow the portable scalar fallback of DVec3.inl. The comparison functions return a
//! DVec3 mask where a component is `true_value` (all bits set) or `false_value` (0.0), like Jolt.

const std = @import("std");
const Core = @import("../Core/Core.zig");
const HashCombine = @import("../Core/HashCombine.zig");
const Double3 = @import("Double3.zig").Double3;
const Vec3 = @import("Vec3.zig").Vec3;
const Vec4 = @import("Vec4.zig").Vec4;

/// The bits of a DVec3, used for the bitwise operations
const BitsType = @Vector(4, u64);

/// 3 component vector of doubles (stored as 4 vectors).
/// Note that we keep the 4th component the same as the 3rd component to avoid divisions by zero when JPH_FLOATING_POINT_EXCEPTIONS_ENABLED defined
pub const DVec3 = extern struct {
    /// Underlying vector type
    pub const Type = @Vector(4, f64);

    value: Type align(Core.dvector_alignment),

    /// Representations of true and false for boolean operations (cTrue / cFalse)
    pub const true_value: f64 = @bitCast(~@as(u64, 0));
    pub const false_value: f64 = 0.0;

    comptime {
        std.debug.assert(@sizeOf(DVec3) == 32);
        std.debug.assert(@alignOf(DVec3) == Core.dvector_alignment);
    }

    /// Create a vector from 3 components
    pub fn init(x: f64, y: f64, z: f64) DVec3 {
        return .{ .value = .{ x, y, z, z } };
    }

    /// Create from a raw SIMD value, W is replaced by Z (DVec3(TypeArg))
    pub fn fromType(v: Type) DVec3 {
        return .{ .value = fixW(v) };
    }

    /// Convert from Vec3, every component is converted to a double (explicit DVec3(Vec3Arg))
    pub fn fromVec3(v: Vec3) DVec3 {
        // Vec3 keeps W == Z, so the converted W is the same as the converted Z
        return .{ .value = @floatCast(v.value) };
    }

    /// Convert from Vec4, the W component is dropped (explicit DVec3(Vec4Arg))
    pub fn fromVec4(v: Vec4) DVec3 {
        return fromVec3(Vec3.fromVec4(v));
    }

    /// Load 3 doubles from memory (explicit DVec3(const Double3 &))
    pub fn fromDouble3(v: Double3) DVec3 {
        return init(v.x, v.y, v.z);
    }

    /// Vector with all zeros
    pub fn zero() DVec3 {
        return init(0, 0, 0);
    }

    /// Vector with all ones
    pub fn one() DVec3 {
        return replicate(1.0);
    }

    /// Vectors with the principal axis
    pub fn axisX() DVec3 {
        return init(1, 0, 0);
    }
    pub fn axisY() DVec3 {
        return init(0, 1, 0);
    }
    pub fn axisZ() DVec3 {
        return init(0, 0, 1);
    }

    /// Replicate v across all components
    pub fn replicate(v: f64) DVec3 {
        return init(v, v, v);
    }

    /// Vector with all NaN's
    pub fn nan() DVec3 {
        return replicate(std.math.nan(f64));
    }

    /// Load 3 doubles from memory (reads 64 bits extra which it doesn't use in Jolt, Zolt only reads 3 doubles)
    pub fn loadDouble3Unsafe(v: *const Double3) DVec3 {
        return init(v.x, v.y, v.z);
    }

    /// Store 3 doubles to memory
    pub fn storeDouble3(self: DVec3, out: *Double3) void {
        out.* = .{ .x = self.value[0], .y = self.value[1], .z = self.value[2] };
    }

    /// Convert to float vector 3 rounding to nearest (explicit operator Vec3())
    pub fn toVec3(self: DVec3) Vec3 {
        // @floatCast rounds to nearest, like (float)GetX()
        return Vec3.fromType(@floatCast(self.value));
    }

    /// Prepare to convert to float vector 3 rounding towards zero (returns DVec3 that can be converted to a Vec3 to get the rounding)
    pub fn prepareRoundToZero(self: DVec3) DVec3 {
        // Float has 23 bit mantissa, double 52 bit mantissa => we lose 29 bits when converting from double to float
        const double_to_float_mantissa_loss: u64 = (1 << 29) - 1;

        const bits: BitsType = @bitCast(self.value);
        return .{ .value = @bitCast(bits & @as(BitsType, @splat(~double_to_float_mantissa_loss))) };
    }

    /// Prepare to convert to float vector 3 rounding towards positive/negative inf (returns DVec3 that can be converted to a Vec3 to get the rounding)
    pub fn prepareRoundToInf(self: DVec3) DVec3 {
        // Float has 23 bit mantissa, double 52 bit mantissa => we lose 29 bits when converting from double to float
        const double_to_float_mantissa_loss: u64 = (1 << 29) - 1;

        const bits: BitsType = @bitCast(self.value);
        const loss: BitsType = @splat(double_to_float_mantissa_loss);
        const no_loss = (bits & loss) == @as(BitsType, @splat(0));
        return .{ .value = @bitCast(@select(u64, no_loss, bits, bits | loss)) };
    }

    /// Convert to float vector 3 rounding down
    pub fn toVec3RoundDown(self: DVec3) Vec3 {
        const to_zero = self.prepareRoundToZero();
        const to_inf = self.prepareRoundToInf();
        return select(to_zero, to_inf, less(self, zero())).toVec3();
    }

    /// Convert to float vector 3 rounding up
    pub fn toVec3RoundUp(self: DVec3) Vec3 {
        const to_zero = self.prepareRoundToZero();
        const to_inf = self.prepareRoundToInf();
        return select(to_inf, to_zero, less(self, zero())).toVec3();
    }

    /// Return the minimum value of each of the components (returns v2 when not v1 < v2, like _mm256_min_pd)
    pub fn min(v1: DVec3, v2: DVec3) DVec3 {
        return .{ .value = @select(f64, v1.value < v2.value, v1.value, v2.value) };
    }

    /// Return the maximum of each of the components (returns v2 when not v1 > v2, like _mm256_max_pd)
    pub fn max(v1: DVec3, v2: DVec3) DVec3 {
        return .{ .value = @select(f64, v1.value > v2.value, v1.value, v2.value) };
    }

    /// Clamp a vector between min and max (component wise)
    pub fn clamp(v: DVec3, min_v: DVec3, max_v: DVec3) DVec3 {
        return max(min(v, max_v), min_v);
    }

    /// Equals (component wise)
    pub fn equals(v1: DVec3, v2: DVec3) DVec3 {
        return fromBools(v1.value == v2.value);
    }

    /// Less than (component wise)
    pub fn less(v1: DVec3, v2: DVec3) DVec3 {
        return fromBools(v1.value < v2.value);
    }

    /// Less than or equal (component wise)
    pub fn lessOrEqual(v1: DVec3, v2: DVec3) DVec3 {
        return fromBools(v1.value <= v2.value);
    }

    /// Greater than (component wise)
    pub fn greater(v1: DVec3, v2: DVec3) DVec3 {
        return fromBools(v1.value > v2.value);
    }

    /// Greater than or equal (component wise)
    pub fn greaterOrEqual(v1: DVec3, v2: DVec3) DVec3 {
        return fromBools(v1.value >= v2.value);
    }

    /// Calculates mul1 * mul2 + add. Never fused in Zolt, to stay cross platform deterministic.
    pub fn fusedMultiplyAdd(mul1: DVec3, mul2: DVec3, add_v: DVec3) DVec3 {
        return mul1.mul(mul2).add(add_v);
    }

    /// Component wise select, returns not_set when highest bit of control = 0 and set when highest bit of control = 1
    pub fn select(not_set_value: DVec3, set_value: DVec3, control: DVec3) DVec3 {
        return fromType(@select(f64, control.toBools(), set_value.value, not_set_value.value));
    }

    /// Logical or (component wise) (sOr)
    pub fn bitOr(v1: DVec3, v2: DVec3) DVec3 {
        return .{ .value = @bitCast(v1.toBits() | v2.toBits()) };
    }

    /// Logical xor (component wise) (sXor)
    pub fn bitXor(v1: DVec3, v2: DVec3) DVec3 {
        return .{ .value = @bitCast(v1.toBits() ^ v2.toBits()) };
    }

    /// Logical and (component wise) (sAnd)
    pub fn bitAnd(v1: DVec3, v2: DVec3) DVec3 {
        return .{ .value = @bitCast(v1.toBits() & v2.toBits()) };
    }

    /// Store if X is true in bit 0, Y in bit 1, Z in bit 2 and W in bit 3 (true is when highest bit of component is set)
    pub fn getTrues(self: DVec3) u32 {
        const bits = self.toBits();
        return @intCast((bits[0] >> 63) | ((bits[1] >> 63) << 1) | ((bits[2] >> 63) << 2));
    }

    /// Test if any of the components are true (true is when highest bit of component is set)
    pub fn testAnyTrue(self: DVec3) bool {
        return self.getTrues() != 0;
    }

    /// Test if all components are true (true is when highest bit of component is set)
    pub fn testAllTrue(self: DVec3) bool {
        return self.getTrues() == 0x7;
    }

    /// Get individual components
    pub fn getX(self: DVec3) f64 {
        return self.value[0];
    }
    pub fn getY(self: DVec3) f64 {
        return self.value[1];
    }
    pub fn getZ(self: DVec3) f64 {
        return self.value[2];
    }

    /// Set individual components
    pub fn setX(self: *DVec3, x: f64) void {
        self.value[0] = x;
    }
    pub fn setY(self: *DVec3, y: f64) void {
        self.value[1] = y;
    }
    pub fn setZ(self: *DVec3, z: f64) void {
        // Assure Z and W are the same
        self.value[2] = z;
        self.value[3] = z;
    }

    /// Set all components
    pub fn set(self: *DVec3, x: f64, y: f64, z: f64) void {
        self.* = init(x, y, z);
    }

    /// Get double component by index (operator [] const)
    pub fn getComponent(self: DVec3, coordinate: u32) f64 {
        std.debug.assert(coordinate < 3);
        const components: [4]f64 = self.value; // Vectors can only be indexed with comptime indices
        return components[coordinate];
    }

    /// Set double component by index
    pub fn setComponent(self: *DVec3, coordinate: u32, v: f64) void {
        std.debug.assert(coordinate < 3);
        var components: [4]f64 = self.value; // Vectors can only be indexed with comptime indices
        components[coordinate] = v;
        self.value = fixW(components); // Assure Z and W are the same
    }

    /// Comparison (operator ==)
    pub fn eql(self: DVec3, other: DVec3) bool {
        return equals(self, other).testAllTrue();
    }

    /// Test if two vectors are close
    pub fn isClose(self: DVec3, other: DVec3, opts: struct { max_dist_sq: f64 = 1.0e-24 }) bool {
        return other.sub(self).lengthSq() <= opts.max_dist_sq;
    }

    /// Test if vector is near zero
    pub fn isNearZero(self: DVec3, opts: struct { max_dist_sq: f64 = 1.0e-24 }) bool {
        return self.lengthSq() <= opts.max_dist_sq;
    }

    /// Test if length^2 of this vector is within the range [1 - tolerance, 1 + tolerance]
    pub fn isNormalized(self: DVec3, opts: struct { tolerance: f64 = 1.0e-12 }) bool {
        return @abs(self.lengthSq() - 1.0) <= opts.tolerance;
    }

    /// Test if vector contains NaN elements
    pub fn isNaN(self: DVec3) bool {
        // W is the same as Z, so it doesn't change the result
        return @reduce(.Or, self.value != self.value);
    }

    /// Multiply two double vectors (component wise) (operator *)
    pub fn mul(self: DVec3, other: DVec3) DVec3 {
        return .{ .value = self.value * other.value };
    }

    /// Multiply vector with double (operator * (double), operator * (double, DVec3))
    pub fn mulScalar(self: DVec3, v: f64) DVec3 {
        return .{ .value = self.value * @as(Type, @splat(v)) };
    }

    /// Divide vector by double (operator / (double))
    pub fn divScalar(self: DVec3, v: f64) DVec3 {
        return .{ .value = self.value / @as(Type, @splat(v)) };
    }

    /// Add two vectors (component wise), the Vec3 is converted to double first (operator + (Vec3Arg))
    pub fn addVec3(self: DVec3, other: Vec3) DVec3 {
        return .{ .value = self.value + @as(Type, @floatCast(other.value)) };
    }

    /// Add two double vectors (component wise) (operator +)
    pub fn add(self: DVec3, other: DVec3) DVec3 {
        return .{ .value = self.value + other.value };
    }

    /// Negate (operator - ()). Computed as 0 - v, which maps -0 to +0 (JPH_CROSS_PLATFORM_DETERMINISTIC).
    pub fn negate(self: DVec3) DVec3 {
        return .{ .value = @as(Type, @splat(0.0)) - self.value };
    }

    /// Subtract two vectors (component wise), the Vec3 is converted to double first (operator - (Vec3Arg))
    pub fn subVec3(self: DVec3, other: Vec3) DVec3 {
        return .{ .value = self.value - @as(Type, @floatCast(other.value)) };
    }

    /// Subtract two double vectors (component wise) (operator -)
    pub fn sub(self: DVec3, other: DVec3) DVec3 {
        return .{ .value = self.value - other.value };
    }

    /// Divide (component wise) (operator /)
    pub fn div(self: DVec3, other: DVec3) DVec3 {
        return .{ .value = self.value / other.value };
    }

    /// Return the absolute value of each of the components
    pub fn abs(self: DVec3) DVec3 {
        return .{ .value = @abs(self.value) };
    }

    /// Reciprocal vector (1 / value) for each of the components
    pub fn reciprocal(self: DVec3) DVec3 {
        return one().div(self);
    }

    /// Cross product
    pub fn cross(self: DVec3, other: DVec3) DVec3 {
        const a = self.value;
        const b = other.value;
        return init(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
    }

    /// Dot product
    pub fn dot(self: DVec3, other: DVec3) f64 {
        const a = self.value;
        const b = other.value;
        var result: f64 = 0.0;
        inline for (0..3) |i|
            result += a[i] * b[i];
        return result;
    }

    /// Squared length of vector
    pub fn lengthSq(self: DVec3) f64 {
        return self.dot(self);
    }

    /// Length of vector
    pub fn length(self: DVec3) f64 {
        return @sqrt(self.dot(self));
    }

    /// Normalize vector
    pub fn normalized(self: DVec3) DVec3 {
        return self.divScalar(self.length());
    }

    /// Component wise square root
    pub fn sqrt(self: DVec3) DVec3 {
        return .{ .value = @sqrt(self.value) };
    }

    /// Get vector that contains the sign of each element (returns 1 if positive, -1 if negative)
    pub fn getSign(self: DVec3) DVec3 {
        return .{ .value = @select(f64, self.toBools(), @as(Type, @splat(-1.0)), @as(Type, @splat(1.0))) };
    }

    /// JPH_MAKE_HASHABLE(JPH::DVec3, t.GetX(), t.GetY(), t.GetZ())
    pub fn getHash(self: DVec3) u64 {
        return HashCombine.hashCombineArgs(.{ self.getX(), self.getY(), self.getZ() });
    }

    pub fn format(self: DVec3, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try writer.print("{d}, {d}, {d}", .{ self.value[0], self.value[1], self.value[2] });
    }

    // Zolt helpers (no Jolt equivalent)

    /// Convert to a vector of bools, a component is true when its highest bit (the sign bit) is set
    pub fn toBools(self: DVec3) @Vector(4, bool) {
        return (self.toBits() >> @splat(63)) != @as(BitsType, @splat(0));
    }

    /// Convert a vector of bools to a mask (true -> true_value, false -> false_value)
    pub fn fromBools(b: @Vector(4, bool)) DVec3 {
        return .{ .value = @select(f64, b, @as(Type, @splat(true_value)), @as(Type, @splat(false_value))) };
    }

    /// Reinterpret the components as 64 bit integers (doesn't change the bits)
    fn toBits(self: DVec3) BitsType {
        return @bitCast(self.value);
    }

    /// Internal helper function that ensures that the Z component is replicated to the W component to prevent divisions by zero (sFixW)
    fn fixW(v: Type) Type {
        return @shuffle(f64, v, undefined, @Vector(4, i32){ 0, 1, 2, 2 });
    }
};
