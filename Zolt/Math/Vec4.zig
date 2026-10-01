//! Port of: Jolt/Math/Vec4.h, Jolt/Math/Vec4.inl
//! Status: complete

const std = @import("std");
const math = @import("Math.zig");
const Swizzle = @import("Swizzle.zig").Swizzle;
const Float4 = @import("Float4.zig").Float4;
const UVec4 = @import("UVec4.zig").UVec4;
const Vec3 = @import("Vec3.zig").Vec3;

/// Vector of 4 floats, aligned to 16 bytes
pub const Vec4 = extern struct {
    pub const Type = @Vector(4, f32);

    value: Type,

    /// Create a vector from 4 components
    pub fn init(x: f32, y: f32, z: f32, w: f32) Vec4 {
        return .{ .value = .{ x, y, z, w } };
    }

    /// Convert from Vec3, W component will be the same as Z (explicit Vec4(Vec3Arg))
    pub fn fromVec3(v: Vec3) Vec4 {
        return .{ .value = v.value };
    }

    /// Create from a Vec3 and a W component (Vec4(Vec3Arg, float))
    pub fn fromVec3W(v: Vec3, w: f32) Vec4 {
        var result: Vec4 = .{ .value = v.value };
        result.value[3] = w;
        return result;
    }

    /// Vector with all zeros
    pub fn zero() Vec4 {
        return replicate(0.0);
    }

    /// Vector with all ones
    pub fn one() Vec4 {
        return replicate(1.0);
    }

    /// Vector with all NaN's
    pub fn nan() Vec4 {
        return replicate(std.math.nan(f32));
    }

    /// Replicate v across all components
    pub fn replicate(v: f32) Vec4 {
        return .{ .value = @splat(v) };
    }

    /// Load 4 floats from memory
    pub fn loadFloat4(v: *const Float4) Vec4 {
        return init(v.x, v.y, v.z, v.w);
    }

    /// Load 4 floats from memory, 16 bytes aligned
    pub fn loadFloat4Aligned(v: *align(16) const Float4) Vec4 {
        return init(v.x, v.y, v.z, v.w);
    }

    /// Gather 4 floats from memory at base + offsets[i] * scale (bytes)
    pub fn gatherFloat4(comptime scale: u32, base: [*]const f32, offsets: UVec4) Vec4 {
        const bytes: [*]const u8 = @ptrCast(base);
        var result: Vec4 = undefined;
        inline for (0..4) |i| {
            const p: *align(1) const f32 = @ptrCast(bytes + @as(usize, offsets.value[i]) * scale);
            result.value[i] = p.*;
        }
        return result;
    }

    /// Return the minimum value of each of the components (returns v2 when not v1 < v2, like _mm_min_ps)
    pub fn min(v1: Vec4, v2: Vec4) Vec4 {
        return .{ .value = @select(f32, v1.value < v2.value, v1.value, v2.value) };
    }

    /// Return the maximum of each of the components (returns v2 when not v1 > v2, like _mm_max_ps)
    pub fn max(v1: Vec4, v2: Vec4) Vec4 {
        return .{ .value = @select(f32, v1.value > v2.value, v1.value, v2.value) };
    }

    /// Clamp a vector between min and max (component wise)
    pub fn clamp(v: Vec4, min_v: Vec4, max_v: Vec4) Vec4 {
        return max(min(v, max_v), min_v);
    }

    /// Equals (component wise)
    pub fn equals(v1: Vec4, v2: Vec4) UVec4 {
        return UVec4.fromBools(v1.value == v2.value);
    }

    /// Less than (component wise)
    pub fn less(v1: Vec4, v2: Vec4) UVec4 {
        return UVec4.fromBools(v1.value < v2.value);
    }

    /// Less than or equal (component wise)
    pub fn lessOrEqual(v1: Vec4, v2: Vec4) UVec4 {
        return UVec4.fromBools(v1.value <= v2.value);
    }

    /// Greater than (component wise)
    pub fn greater(v1: Vec4, v2: Vec4) UVec4 {
        return UVec4.fromBools(v1.value > v2.value);
    }

    /// Greater than or equal (component wise)
    pub fn greaterOrEqual(v1: Vec4, v2: Vec4) UVec4 {
        return UVec4.fromBools(v1.value >= v2.value);
    }

    /// Calculates mul1 * mul2 + add. Never fused in Zolt, to stay cross platform deterministic.
    pub fn fusedMultiplyAdd(mul1: Vec4, mul2: Vec4, add_v: Vec4) Vec4 {
        return .{ .value = mul1.value * mul2.value + add_v.value };
    }

    /// Component wise select, returns not_set when highest bit of control = 0 and set when highest bit of control = 1
    pub fn select(not_set_value: Vec4, set_value: Vec4, control: UVec4) Vec4 {
        return .{ .value = @select(f32, control.toBools(), set_value.value, not_set_value.value) };
    }

    /// Logical or (component wise) (sOr)
    pub fn bitOr(v1: Vec4, v2: Vec4) Vec4 {
        return UVec4.bitOr(v1.reinterpretAsInt(), v2.reinterpretAsInt()).reinterpretAsFloat();
    }

    /// Logical xor (component wise) (sXor)
    pub fn bitXor(v1: Vec4, v2: Vec4) Vec4 {
        return UVec4.bitXor(v1.reinterpretAsInt(), v2.reinterpretAsInt()).reinterpretAsFloat();
    }

    /// Logical and (component wise) (sAnd)
    pub fn bitAnd(v1: Vec4, v2: Vec4) Vec4 {
        return UVec4.bitAnd(v1.reinterpretAsInt(), v2.reinterpretAsInt()).reinterpretAsFloat();
    }

    /// Sort the four elements of value and sort index at the same time.
    /// Based on a sorting network: http://en.wikipedia.org/wiki/Sorting_network
    pub fn sort4(value: *Vec4, index: *UVec4) void {
        // Pass 1, test 1st vs 3rd, 2nd vs 4th
        const v1 = value.swizzle(.z, .w, .x, .y);
        const i_1 = index.swizzle(.z, .w, .x, .y);
        const c1 = less(value.*, v1).swizzle(.z, .w, .z, .w);
        value.* = select(value.*, v1, c1);
        index.* = UVec4.select(index.*, i_1, c1);

        // Pass 2, test 1st vs 2nd, 3rd vs 4th
        const v2 = value.swizzle(.y, .x, .w, .z);
        const i_2 = index.swizzle(.y, .x, .w, .z);
        const c2 = less(value.*, v2).swizzle(.y, .y, .w, .w);
        value.* = select(value.*, v2, c2);
        index.* = UVec4.select(index.*, i_2, c2);

        // Pass 3, test 2nd vs 3rd component
        const v3 = value.swizzle(.x, .z, .y, .w);
        const i_3 = index.swizzle(.x, .z, .y, .w);
        const c3 = less(value.*, v3).swizzle(.x, .z, .z, .w);
        value.* = select(value.*, v3, c3);
        index.* = UVec4.select(index.*, i_3, c3);
    }

    /// Reverse sort the four elements of value (highest first) and sort index at the same time.
    /// Based on a sorting network: http://en.wikipedia.org/wiki/Sorting_network
    pub fn sort4Reverse(value: *Vec4, index: *UVec4) void {
        // Pass 1, test 1st vs 3rd, 2nd vs 4th
        const v1 = value.swizzle(.z, .w, .x, .y);
        const i_1 = index.swizzle(.z, .w, .x, .y);
        const c1 = greater(value.*, v1).swizzle(.z, .w, .z, .w);
        value.* = select(value.*, v1, c1);
        index.* = UVec4.select(index.*, i_1, c1);

        // Pass 2, test 1st vs 2nd, 3rd vs 4th
        const v2 = value.swizzle(.y, .x, .w, .z);
        const i_2 = index.swizzle(.y, .x, .w, .z);
        const c2 = greater(value.*, v2).swizzle(.y, .y, .w, .w);
        value.* = select(value.*, v2, c2);
        index.* = UVec4.select(index.*, i_2, c2);

        // Pass 3, test 2nd vs 3rd component
        const v3 = value.swizzle(.x, .z, .y, .w);
        const i_3 = index.swizzle(.x, .z, .y, .w);
        const c3 = greater(value.*, v3).swizzle(.x, .z, .z, .w);
        value.* = select(value.*, v3, c3);
        index.* = UVec4.select(index.*, i_3, c3);
    }

    /// Get individual components
    pub fn getX(self: Vec4) f32 {
        return self.value[0];
    }
    pub fn getY(self: Vec4) f32 {
        return self.value[1];
    }
    pub fn getZ(self: Vec4) f32 {
        return self.value[2];
    }
    pub fn getW(self: Vec4) f32 {
        return self.value[3];
    }

    /// Set individual components
    pub fn setX(self: *Vec4, x: f32) void {
        self.value[0] = x;
    }
    pub fn setY(self: *Vec4, y: f32) void {
        self.value[1] = y;
    }
    pub fn setZ(self: *Vec4, z: f32) void {
        self.value[2] = z;
    }
    pub fn setW(self: *Vec4, w: f32) void {
        self.value[3] = w;
    }

    /// Set all components
    pub fn set(self: *Vec4, x: f32, y: f32, z: f32, w: f32) void {
        self.* = init(x, y, z, w);
    }

    /// Get float component by index (operator [] const)
    pub fn getComponent(self: Vec4, coordinate: u32) f32 {
        std.debug.assert(coordinate < 4);
        const components: [4]@TypeOf(self.value[0]) = self.value; // Vectors can only be indexed with comptime indices
        return components[coordinate];
    }

    /// Set float component by index (operator [])
    pub fn setComponent(self: *Vec4, coordinate: u32, v: f32) void {
        std.debug.assert(coordinate < 4);
        var components: [4]@TypeOf(self.value[0]) = self.value; // Vectors can only be indexed with comptime indices
        components[coordinate] = v;
        self.value = components;
    }

    /// Comparison (operator ==)
    pub fn eql(self: Vec4, other: Vec4) bool {
        return equals(self, other).testAllTrue();
    }

    /// Test if two vectors are close
    pub fn isClose(self: Vec4, other: Vec4, opts: struct { max_dist_sq: f32 = 1.0e-12 }) bool {
        return other.sub(self).lengthSq() <= opts.max_dist_sq;
    }

    /// Test if vector is near zero
    pub fn isNearZero(self: Vec4, opts: struct { max_dist_sq: f32 = 1.0e-12 }) bool {
        return self.lengthSq() <= opts.max_dist_sq;
    }

    /// Test if length^2 of this vector is within the range [1 - tolerance, 1 + tolerance]
    pub fn isNormalized(self: Vec4, opts: struct { tolerance: f32 = 1.0e-6 }) bool {
        return @abs(self.lengthSq() - 1.0) <= opts.tolerance;
    }

    /// Test if vector contains NaN elements
    pub fn isNaN(self: Vec4) bool {
        return @reduce(.Or, self.value != self.value);
    }

    /// Multiply two float vectors (component wise) (operator *)
    pub fn mul(self: Vec4, other: Vec4) Vec4 {
        return .{ .value = self.value * other.value };
    }

    /// Multiply vector with float (operator * (float))
    pub fn mulScalar(self: Vec4, v: f32) Vec4 {
        return .{ .value = self.value * @as(Type, @splat(v)) };
    }

    /// Divide vector by float (operator / (float))
    pub fn divScalar(self: Vec4, v: f32) Vec4 {
        return .{ .value = self.value / @as(Type, @splat(v)) };
    }

    /// Add two float vectors (component wise) (operator +)
    pub fn add(self: Vec4, other: Vec4) Vec4 {
        return .{ .value = self.value + other.value };
    }

    /// Negate (operator - ()). Computed as 0 - v, which maps -0 to +0 (JPH_CROSS_PLATFORM_DETERMINISTIC).
    pub fn negate(self: Vec4) Vec4 {
        return .{ .value = @as(Type, @splat(0.0)) - self.value };
    }

    /// Subtract two float vectors (component wise) (operator -)
    pub fn sub(self: Vec4, other: Vec4) Vec4 {
        return .{ .value = self.value - other.value };
    }

    /// Divide (component wise) (operator /)
    pub fn div(self: Vec4, other: Vec4) Vec4 {
        return .{ .value = self.value / other.value };
    }

    /// Swizzle the elements in the vector
    pub fn swizzle(self: Vec4, comptime sx: Swizzle, comptime sy: Swizzle, comptime sz: Swizzle, comptime sw: Swizzle) Vec4 {
        return .{ .value = @shuffle(f32, self.value, undefined, @Vector(4, i32){ @intFromEnum(sx), @intFromEnum(sy), @intFromEnum(sz), @intFromEnum(sw) }) };
    }

    /// Replicate the X component to all components
    pub fn splatX(self: Vec4) Vec4 {
        return replicate(self.value[0]);
    }

    /// Replicate the Y component to all components
    pub fn splatY(self: Vec4) Vec4 {
        return replicate(self.value[1]);
    }

    /// Replicate the Z component to all components
    pub fn splatZ(self: Vec4) Vec4 {
        return replicate(self.value[2]);
    }

    /// Replicate the W component to all components
    pub fn splatW(self: Vec4) Vec4 {
        return replicate(self.value[3]);
    }

    /// Replicate the X component to all components
    pub fn splatX3(self: Vec4) Vec3 {
        return Vec3.replicate(self.value[0]);
    }

    /// Replicate the Y component to all components
    pub fn splatY3(self: Vec4) Vec3 {
        return Vec3.replicate(self.value[1]);
    }

    /// Replicate the Z component to all components
    pub fn splatZ3(self: Vec4) Vec3 {
        return Vec3.replicate(self.value[2]);
    }

    /// Replicate the W component to all components
    pub fn splatW3(self: Vec4) Vec3 {
        return Vec3.replicate(self.value[3]);
    }

    /// Get index of component with lowest value
    pub fn getLowestComponentIndex(self: Vec4) u32 {
        // Get the minimum value in all 4 components
        var v = min(self, self.swizzle(.y, .x, .w, .z));
        v = min(v, v.swizzle(.z, .w, .x, .y));

        // Compare with the original vector to find which component is equal to the minimum value
        return math.countTrailingZeros(equals(self, v).getTrues());
    }

    /// Get index of component with highest value
    pub fn getHighestComponentIndex(self: Vec4) u32 {
        // Get the maximum value in all 4 components
        var v = max(self, self.swizzle(.y, .x, .w, .z));
        v = max(v, v.swizzle(.z, .w, .x, .y));

        // Compare with the original vector to find which component is equal to the maximum value
        return math.countTrailingZeros(equals(self, v).getTrues());
    }

    /// Return the absolute value of each of the components
    pub fn abs(self: Vec4) Vec4 {
        return .{ .value = @abs(self.value) };
    }

    /// Reciprocal vector (1 / value) for each of the components
    pub fn reciprocal(self: Vec4) Vec4 {
        return one().div(self);
    }

    /// Calculates a * b - c * d. Never fused in Zolt, see math.differenceOfProducts.
    pub fn differenceOfProducts(a: Vec4, b: Vec4, c: Vec4, d: Vec4) Vec4 {
        return a.mul(b).sub(c.mul(d));
    }

    /// Dot product, returns the dot product in X, Y, Z and W components
    pub fn dotV(self: Vec4, other: Vec4) Vec4 {
        return replicate(self.dot(other));
    }

    /// Dot product
    pub fn dot(self: Vec4, other: Vec4) f32 {
        return self.mul(other).reduceSum();
    }

    /// Squared length of vector
    pub fn lengthSq(self: Vec4) f32 {
        return self.dot(self);
    }

    /// Length of vector
    pub fn length(self: Vec4) f32 {
        return @sqrt(self.lengthSq());
    }

    /// Normalize vector
    pub fn normalized(self: Vec4) Vec4 {
        return self.divScalar(self.length());
    }

    /// Store 4 floats to memory
    pub fn storeFloat4(self: Vec4, out: *Float4) void {
        out.* = .{ .x = self.value[0], .y = self.value[1], .z = self.value[2], .w = self.value[3] };
    }

    /// Convert each component from a float to an int (truncating, like _mm_cvttps_epi32).
    /// Components must be in the range of an i32, this is safety checked in Debug and ReleaseSafe.
    pub fn toInt(self: Vec4) UVec4 {
        const as_int: @Vector(4, i32) = @intFromFloat(self.value);
        return .{ .value = @bitCast(as_int) };
    }

    /// Reinterpret Vec4 as a UVec4 (doesn't change the bits)
    pub fn reinterpretAsInt(self: Vec4) UVec4 {
        return .{ .value = @bitCast(self.value) };
    }

    /// Store if X is negative in bit 0, Y in bit 1, Z in bit 2 and W in bit 3
    pub fn getSignBits(self: Vec4) u32 {
        return self.reinterpretAsInt().getTrues();
    }

    /// Get the minimum of X, Y, Z and W
    pub fn reduceMin(self: Vec4) f32 {
        var v = min(self, self.swizzle(.y, .unused, .w, .unused));
        v = min(v, v.swizzle(.z, .unused, .unused, .unused));
        return v.getX();
    }

    /// Get the maximum of X, Y, Z and W
    pub fn reduceMax(self: Vec4) f32 {
        var v = max(self, self.swizzle(.y, .unused, .w, .unused));
        v = max(v, v.swizzle(.z, .unused, .unused, .unused));
        return v.getX();
    }

    /// Sum X, Y, Z and W
    pub fn reduceSum(self: Vec4) f32 {
        // Brackets placed so that the order is consistent with the vectorized version
        return (self.value[0] + self.value[1]) + (self.value[2] + self.value[3]);
    }

    /// Component wise square root
    pub fn sqrt(self: Vec4) Vec4 {
        return .{ .value = @sqrt(self.value) };
    }

    /// Get vector that contains the sign of each element (returns 1.0 if positive, -1.0 if negative)
    pub fn getSign(self: Vec4) Vec4 {
        const one_bits: UVec4 = one().reinterpretAsInt();
        const sign_bits = UVec4.bitAnd(self.reinterpretAsInt(), UVec4.replicate(0x80000000));
        return UVec4.bitOr(one_bits, sign_bits).reinterpretAsFloat();
    }

    /// Flips the signs of the components, e.g. flipSign(-1, 1, -1, 1) will flip the signs of the X and Z components
    pub fn flipSign(self: Vec4, comptime x: comptime_int, comptime y: comptime_int, comptime z: comptime_int, comptime w: comptime_int) Vec4 {
        comptime {
            for ([_]comptime_int{ x, y, z, w }) |s| {
                if (s != 1 and s != -1) @compileError("flipSign arguments must be 1 or -1");
            }
        }
        const mask = comptime init(if (x > 0) 0.0 else -0.0, if (y > 0) 0.0 else -0.0, if (z > 0) 0.0 else -0.0, if (w > 0) 0.0 else -0.0);
        return bitXor(self, mask);
    }

    /// Result of `sinCos`
    pub const SinCos = struct { sin: Vec4, cos: Vec4 };

    /// Calculate the sine and cosine for each element of this vector (input in radians)
    pub fn sinCos(self: Vec4) SinCos {
        // Implementation based on sinf.c from the cephes library, combines sinf and cosf in a single function, changes octants to quadrants and vectorizes it
        // Original implementation by Stephen L. Moshier (See: http://www.moshier.net/)

        // Make argument positive and remember sign for sin only since cos is symmetric around x (highest bit of a float is the sign bit)
        var sin_sign = UVec4.bitAnd(self.reinterpretAsInt(), UVec4.replicate(0x80000000));
        var x = bitXor(self, sin_sign.reinterpretAsFloat());

        // x / (PI / 2) rounded to nearest int gives us the quadrant closest to x
        const quadrant = x.mulScalar(0.6366197723675814).add(replicate(0.5)).toInt();

        // Make x relative to the closest quadrant.
        // This does x = x - quadrant * PI / 2 using a two step Cody-Waite argument reduction.
        // This improves the accuracy of the result by avoiding loss of significant bits in the subtraction.
        // We start with x = x - quadrant * PI / 2, PI / 2 in hexadecimal notation is 0x3fc90fdb, we remove the lowest 16 bits to
        // get 0x3fc90000 (= 1.5703125) this means we can now multiply with a number of up to 2^16 without losing any bits.
        // This leaves us with: x = (x - quadrant * 1.5703125) - quadrant * (PI / 2 - 1.5703125).
        // PI / 2 - 1.5703125 in hexadecimal is 0x39fdaa22, stripping the lowest 12 bits we get 0x39fda000 (= 0.0004837512969970703125)
        // This leaves uw with: x = ((x - quadrant * 1.5703125) - quadrant * 0.0004837512969970703125) - quadrant * (PI / 2 - 1.5703125 - 0.0004837512969970703125)
        // See: https://stackoverflow.com/questions/42455143/sine-cosine-modular-extended-precision-arithmetic
        // After this we have x in the range [-PI / 4, PI / 4].
        const float_quadrant = quadrant.toFloat();
        x = x.sub(float_quadrant.mulScalar(1.5703125)).sub(float_quadrant.mulScalar(0.0004837512969970703125)).sub(float_quadrant.mulScalar(7.549789948768648e-8));

        // Calculate x2 = x^2
        const x2 = x.mul(x);

        // Taylor expansion:
        // Cos(x) = 1 - x^2/2! + x^4/4! - x^6/6! + x^8/8! + ... = (((x2/8!- 1/6!) * x2 + 1/4!) * x2 - 1/2!) * x2 + 1
        const taylor_cos = x2.mulScalar(2.443315711809948e-5).sub(replicate(1.388731625493765e-3)).mul(x2).add(replicate(4.166664568298827e-2)).mul(x2).mul(x2).sub(x2.mulScalar(0.5)).add(one());
        // Sin(x) = x - x^3/3! + x^5/5! - x^7/7! + ... = ((-x2/7! + 1/5!) * x2 - 1/3!) * x2 * x + x
        const taylor_sin = x2.mulScalar(-1.9515295891e-4).add(replicate(8.3321608736e-3)).mul(x2).sub(replicate(1.6666654611e-1)).mul(x2).mul(x).add(x);

        // The lowest 2 bits of quadrant indicate the quadrant that we are in.
        // Let x be the original input value and x' our value that has been mapped to the range [-PI / 4, PI / 4].
        // since cos(x) = sin(x - PI / 2) and since we want to use the Taylor expansion as close as possible to 0,
        // we can alternate between using the Taylor expansion for sin and cos according to the following table:
        //
        // quadrant  sin(x)    cos(x)
        // XXX00b    sin(x')   cos(x')
        // XXX01b    cos(x')  -sin(x')
        // XXX10b   -sin(x')  -cos(x')
        // XXX11b   -cos(x')   sin(x')
        //
        // So: sin_sign = bit2, cos_sign = bit1 ^ bit2, bit1 determines if we use sin or cos Taylor expansion
        const bit1 = quadrant.logicalShiftLeft(31);
        const bit2 = UVec4.bitAnd(quadrant.logicalShiftLeft(30), UVec4.replicate(0x80000000));

        // Select which one of the results is sin and which one is cos
        const s = select(taylor_sin, taylor_cos, bit1);
        const c = select(taylor_cos, taylor_sin, bit1);

        // Update the signs
        sin_sign = UVec4.bitXor(sin_sign, bit2);
        const cos_sign = UVec4.bitXor(bit1, bit2);

        // Correct the signs
        return .{
            .sin = bitXor(s, sin_sign.reinterpretAsFloat()),
            .cos = bitXor(c, cos_sign.reinterpretAsFloat()),
        };
    }

    /// Calculate the tangent for each element of this vector (input in radians)
    pub fn tan(self: Vec4) Vec4 {
        // Implementation based on tanf.c from the cephes library, see Vec4::SinCos for further details
        // Original implementation by Stephen L. Moshier (See: http://www.moshier.net/)

        // Make argument positive
        const tan_sign = UVec4.bitAnd(self.reinterpretAsInt(), UVec4.replicate(0x80000000));
        var x = bitXor(self, tan_sign.reinterpretAsFloat());

        // x / (PI / 2) rounded to nearest int gives us the quadrant closest to x
        const quadrant = x.mulScalar(0.6366197723675814).add(replicate(0.5)).toInt();

        // Remap x to range [-PI / 4, PI / 4], see Vec4::SinCos
        const float_quadrant = quadrant.toFloat();
        x = x.sub(float_quadrant.mulScalar(1.5703125)).sub(float_quadrant.mulScalar(0.0004837512969970703125)).sub(float_quadrant.mulScalar(7.549789948768648e-8));

        // Calculate x2 = x^2
        const x2 = x.mul(x);

        // Roughly equivalent to the Taylor expansion:
        // Tan(x) = x + x^3/3 + 2*x^5/15 + 17*x^7/315 + 62*x^9/2835 + ...
        var t = x2.mulScalar(9.38540185543e-3).add(replicate(3.11992232697e-3)).mul(x2).add(replicate(2.44301354525e-2)).mul(x2)
            .add(replicate(5.34112807005e-2)).mul(x2).add(replicate(1.33387994085e-1)).mul(x2).add(replicate(3.33331568548e-1)).mul(x2).mul(x).add(x);

        // For the 2nd and 4th quadrant we need to invert the value
        const bit1 = quadrant.logicalShiftLeft(31);
        t = select(t, replicate(-1.0).div(t), bit1);

        // Put the sign back
        return bitXor(t, tan_sign.reinterpretAsFloat());
    }

    /// Calculate the arc sine for each element of this vector (returns value in the range [-PI / 2, PI / 2])
    /// Note that all input values will be clamped to the range [-1, 1] and this function will not return NaNs like std::asin
    pub fn asin(self: Vec4) Vec4 {
        // Implementation based on asinf.c from the cephes library
        // Original implementation by Stephen L. Moshier (See: http://www.moshier.net/)

        // Make argument positive
        const asin_sign = UVec4.bitAnd(self.reinterpretAsInt(), UVec4.replicate(0x80000000));
        var a = bitXor(self, asin_sign.reinterpretAsFloat());

        // ASin is not defined outside the range [-1, 1] but it often happens that a value is slightly above 1 so we just clamp here
        a = min(a, one());

        // When |x| <= 0.5 we use the asin approximation as is
        const z1 = a.mul(a);
        const x1 = a;

        // When |x| > 0.5 we use the identity asin(x) = PI / 2 - 2 * asin(sqrt((1 - x) / 2))
        const z2 = one().sub(a).mulScalar(0.5);
        const x2 = z2.sqrt();

        // Select which of the two situations we have
        const greater_than_half = greater(a, replicate(0.5));
        var z = select(z1, z2, greater_than_half);
        const x = select(x1, x2, greater_than_half);

        // Polynomial approximation of asin
        z = z.mulScalar(4.2163199048e-2).add(replicate(2.4181311049e-2)).mul(z).add(replicate(4.5470025998e-2)).mul(z).add(replicate(7.4953002686e-2)).mul(z).add(replicate(1.6666752422e-1)).mul(z).mul(x).add(x);

        // If |x| > 0.5 we need to apply the remainder of the identity above
        z = select(z, replicate(0.5 * math.pi).sub(z.add(z)), greater_than_half);

        // Put the sign back
        return bitXor(z, asin_sign.reinterpretAsFloat());
    }

    /// Calculate the arc cosine for each element of this vector (returns value in the range [0, PI])
    /// Note that all input values will be clamped to the range [-1, 1] and this function will not return NaNs like std::acos
    pub fn acos(self: Vec4) Vec4 {
        // Not the most accurate, but simple
        return replicate(0.5 * math.pi).sub(self.asin());
    }

    /// Calculate the arc tangent for each element of this vector (returns value in the range [-PI / 2, PI / 2])
    pub fn atan(self: Vec4) Vec4 {
        // Implementation based on atanf.c from the cephes library
        // Original implementation by Stephen L. Moshier (See: http://www.moshier.net/)

        // Make argument positive
        const atan_sign = UVec4.bitAnd(self.reinterpretAsInt(), UVec4.replicate(0x80000000));
        var x = bitXor(self, atan_sign.reinterpretAsFloat());
        var y = zero();

        // If x > Tan(PI / 8)
        const greater1 = greater(x, replicate(0.4142135623730950));
        const x1 = x.sub(one()).div(x.add(one()));

        // If x > Tan(3 * PI / 8)
        const greater2 = greater(x, replicate(2.414213562373095));
        const x2 = replicate(-1.0).div(x);

        // Apply first if
        x = select(x, x1, greater1);
        y = select(y, replicate(0.25 * math.pi), greater1);

        // Apply second if
        x = select(x, x2, greater2);
        y = select(y, replicate(0.5 * math.pi), greater2);

        // Polynomial approximation
        const z = x.mul(x);
        y = y.add(z.mulScalar(8.05374449538e-2).sub(replicate(1.38776856032e-1)).mul(z).add(replicate(1.99777106478e-1)).mul(z).sub(replicate(3.33329491539e-1)).mul(z).mul(x).add(x));

        // Put the sign back
        return bitXor(y, atan_sign.reinterpretAsFloat());
    }

    /// Calculate the arc tangent of y / x using the signs of the arguments to determine the correct quadrant (returns value in the range [-PI, PI])
    pub fn atan2(y: Vec4, x: Vec4) Vec4 {
        const sign_mask = UVec4.replicate(0x80000000);

        // Determine absolute value and sign of y
        const y_sign = UVec4.bitAnd(y.reinterpretAsInt(), sign_mask);
        const y_abs = bitXor(y, y_sign.reinterpretAsFloat());

        // Determine absolute value and sign of x
        const x_sign = UVec4.bitAnd(x.reinterpretAsInt(), sign_mask);
        const x_abs = bitXor(x, x_sign.reinterpretAsFloat());

        // Always divide smallest / largest to avoid dividing by zero
        const x_is_numerator = less(x_abs, y_abs);
        const numerator = select(y_abs, x_abs, x_is_numerator);
        const denominator = select(x_abs, y_abs, x_is_numerator);
        var result = numerator.div(denominator).atan();

        // If we calculated x / y instead of y / x the result is PI / 2 - result (note that this is true because we know the result is positive because the input was positive)
        result = select(result, replicate(0.5 * math.pi).sub(result), x_is_numerator);

        // Now we need to map to the correct quadrant
        // x_sign  y_sign  result
        // +1      +1      atan
        // -1      +1      -atan + PI
        // -1      -1      atan - PI
        // +1      -1      -atan
        // This can be written as: x_sign * y_sign * (atan - (x_sign < 0? PI : 0))
        result = result.sub(bitAnd(x_sign.arithmeticShiftRight(31).reinterpretAsFloat(), replicate(math.pi)));
        result = bitXor(result, UVec4.bitXor(x_sign, y_sign).reinterpretAsFloat());
        return result;
    }

    /// Compress a unit vector to a 32 bit value, precision is around 0.5 * 10^-3
    pub fn compressUnitVector(self: Vec4) u32 {
        const one_over_sqrt2: f32 = 0.70710678;
        const num_bits = 9;
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

        // Store the other three components in a compressed format
        var compressed = clamp(v.add(replicate(one_over_sqrt2)).mulScalar(scale).add(replicate(0.5)), zero(), replicate(@floatFromInt(max_value))).toInt();
        switch (max_element) {
            0 => compressed = compressed.swizzle(.y, .z, .w, .unused),
            1 => compressed = compressed.swizzle(.x, .z, .w, .unused),
            2 => compressed = compressed.swizzle(.x, .y, .w, .unused),
            else => {},
        }

        value |= compressed.getX();
        value |= compressed.getY() << num_bits;
        value |= compressed.getZ() << 2 * num_bits;
        return value;
    }

    /// Decompress a unit vector from a 32 bit value
    pub fn decompressUnitVector(value: u32) Vec4 {
        const one_over_sqrt2: f32 = 0.70710678;
        const num_bits = 9;
        const mask: u32 = (1 << num_bits) - 1;
        const max_value: u32 = mask - 1; // Need odd number of buckets to quantize to or else we can't encode 0
        const half_max_value: i32 = @intCast(max_value >> 1);
        const scale: f32 = 2.0 * one_over_sqrt2 / @as(f32, @floatFromInt(max_value));

        // Restore three components
        const cx: i32 = @as(i32, @intCast(value & mask)) - half_max_value;
        const cy: i32 = @as(i32, @intCast((value >> num_bits) & mask)) - half_max_value;
        const cz: i32 = @as(i32, @intCast((value >> (2 * num_bits)) & mask)) - half_max_value;
        var v = init(@floatFromInt(cx), @floatFromInt(cy), @floatFromInt(cz), 0).mulScalar(scale);
        std.debug.assert(v.getW() == 0.0);

        // Restore the highest component
        v.setW(@sqrt(math.max(1.0 - v.lengthSq(), 0.0)));

        // Extract sign
        if ((value & 0x80000000) != 0)
            v = v.negate();

        // Swizzle the components in place
        switch ((value >> 29) & 3) {
            0 => v = v.swizzle(.w, .x, .y, .z),
            1 => v = v.swizzle(.x, .w, .y, .z),
            2 => v = v.swizzle(.x, .y, .w, .z),
            else => {},
        }

        return v;
    }

    pub fn format(self: Vec4, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try writer.print("{d}, {d}, {d}, {d}", .{ self.value[0], self.value[1], self.value[2], self.value[3] });
    }
};
