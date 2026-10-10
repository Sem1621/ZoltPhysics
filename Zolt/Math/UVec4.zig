//! Port of: Jolt/Math/UVec4.h, Jolt/Math/UVec4.inl
//! Status: complete

const std = @import("std");
const Swizzle = @import("Swizzle.zig").Swizzle;
const Vec4 = @import("Vec4.zig").Vec4;

/// Vector of 4 32 bit unsigned integers. Also used as a mask for the comparison functions of
/// Vec3 / Vec4, where "true" means that the highest bit of a component is set.
pub const UVec4 = extern struct {
    pub const Type = @Vector(4, u32);

    value: Type,

    /// Create a vector from 4 integer components
    pub fn init(x: u32, y: u32, z: u32, w: u32) UVec4 {
        return .{ .value = .{ x, y, z, w } };
    }

    /// Vector with all zeros
    pub fn zero() UVec4 {
        return .{ .value = @splat(0) };
    }

    /// Replicate int v across all components
    pub fn replicate(v: u32) UVec4 {
        return .{ .value = @splat(v) };
    }

    /// Load 1 int from memory and place it in the X component, zeros Y, Z and W
    pub fn loadInt(v: *const u32) UVec4 {
        return init(v.*, 0, 0, 0);
    }

    /// Load 4 ints from memory
    pub fn loadInt4(v: *const [4]u32) UVec4 {
        return .{ .value = v.* };
    }

    /// Load 4 ints from memory, aligned to 16 bytes
    pub fn loadInt4Aligned(v: *align(16) const [4]u32) UVec4 {
        return .{ .value = v.* };
    }

    /// Gather 4 ints from memory at base + offsets[i] * scale (bytes)
    pub fn gatherInt4(comptime scale: u32, base: [*]const u32, offsets: UVec4) UVec4 {
        const bytes: [*]const u8 = @ptrCast(base);
        var result: UVec4 = undefined;
        inline for (0..4) |i| {
            const p: *align(1) const u32 = @ptrCast(bytes + @as(usize, offsets.value[i]) * scale);
            result.value[i] = p.*;
        }
        return result;
    }

    /// Return the minimum value of each of the components
    pub fn min(v1: UVec4, v2: UVec4) UVec4 {
        return .{ .value = @min(v1.value, v2.value) };
    }

    /// Return the maximum of each of the components
    pub fn max(v1: UVec4, v2: UVec4) UVec4 {
        return .{ .value = @max(v1.value, v2.value) };
    }

    /// Equals (component wise)
    pub fn equals(v1: UVec4, v2: UVec4) UVec4 {
        return fromBools(v1.value == v2.value);
    }

    /// Component wise select, returns not_set when highest bit of control = 0 and set when highest bit of control = 1
    pub fn select(not_set_value: UVec4, set_value: UVec4, control: UVec4) UVec4 {
        return .{ .value = @select(u32, control.toBools(), set_value.value, not_set_value.value) };
    }

    /// Logical or (component wise) (sOr)
    pub fn bitOr(v1: UVec4, v2: UVec4) UVec4 {
        return .{ .value = v1.value | v2.value };
    }

    /// Logical xor (component wise) (sXor)
    pub fn bitXor(v1: UVec4, v2: UVec4) UVec4 {
        return .{ .value = v1.value ^ v2.value };
    }

    /// Logical and (component wise) (sAnd)
    pub fn bitAnd(v1: UVec4, v2: UVec4) UVec4 {
        return .{ .value = v1.value & v2.value };
    }

    /// Logical not (component wise) (sNot)
    pub fn bitNot(v1: UVec4) UVec4 {
        return .{ .value = ~v1.value };
    }

    /// Sorts the elements in index so that the values that correspond to trues in value are the first elements.
    /// The remaining elements will be set to value.w.
    /// I.e. if value = (true, false, true, false) and index = (1, 2, 3, 4) the function returns (1, 3, 4, 4).
    pub fn sort4True(value: UVec4, index: UVec4) UVec4 {
        // If value.z is false then shift W to Z
        var v = select(index.swizzle(.x, .y, .w, .w), index, value.splatZ());

        // If value.y is false then shift Z and further to Y and further
        v = select(v.swizzle(.x, .z, .w, .w), v, value.splatY());

        // If value.x is false then shift X and further to Y and further
        v = select(v.swizzle(.y, .z, .w, .w), v, value.splatX());

        return v;
    }

    /// Get individual components
    pub fn getX(self: UVec4) u32 {
        return self.value[0];
    }
    pub fn getY(self: UVec4) u32 {
        return self.value[1];
    }
    pub fn getZ(self: UVec4) u32 {
        return self.value[2];
    }
    pub fn getW(self: UVec4) u32 {
        return self.value[3];
    }

    /// Set individual components
    pub fn setX(self: *UVec4, x: u32) void {
        self.value[0] = x;
    }
    pub fn setY(self: *UVec4, y: u32) void {
        self.value[1] = y;
    }
    pub fn setZ(self: *UVec4, z: u32) void {
        self.value[2] = z;
    }
    pub fn setW(self: *UVec4, w: u32) void {
        self.value[3] = w;
    }

    /// Get component by index (operator [] const)
    pub fn getComponent(self: UVec4, coordinate: u32) u32 {
        std.debug.assert(coordinate < 4);
        const components: [4]@TypeOf(self.value[0]) = self.value; // Vectors can only be indexed with comptime indices
        return components[coordinate];
    }

    /// Set component by index (operator [])
    pub fn setComponent(self: *UVec4, coordinate: u32, v: u32) void {
        std.debug.assert(coordinate < 4);
        var components: [4]@TypeOf(self.value[0]) = self.value; // Vectors can only be indexed with comptime indices
        components[coordinate] = v;
        self.value = components;
    }

    /// Comparison (operator ==)
    pub fn eql(self: UVec4, other: UVec4) bool {
        return @reduce(.And, self.value == other.value);
    }

    /// Swizzle the elements in the vector
    pub fn swizzle(self: UVec4, comptime sx: Swizzle, comptime sy: Swizzle, comptime sz: Swizzle, comptime sw: Swizzle) UVec4 {
        return .{ .value = @shuffle(u32, self.value, undefined, @Vector(4, i32){ @intFromEnum(sx), @intFromEnum(sy), @intFromEnum(sz), @intFromEnum(sw) }) };
    }

    /// Component wise multiplication of two integer vectors (stores low 32 bits of result only) (operator *)
    pub fn mul(self: UVec4, other: UVec4) UVec4 {
        return .{ .value = self.value *% other.value };
    }

    /// Add two integer vectors (component wise) (operator +)
    pub fn add(self: UVec4, other: UVec4) UVec4 {
        return .{ .value = self.value +% other.value };
    }

    /// Subtract two integer vectors (component wise) (operator -)
    pub fn sub(self: UVec4, other: UVec4) UVec4 {
        return .{ .value = self.value -% other.value };
    }

    /// Replicate the X component to all components
    pub fn splatX(self: UVec4) UVec4 {
        return replicate(self.value[0]);
    }

    /// Replicate the Y component to all components
    pub fn splatY(self: UVec4) UVec4 {
        return replicate(self.value[1]);
    }

    /// Replicate the Z component to all components
    pub fn splatZ(self: UVec4) UVec4 {
        return replicate(self.value[2]);
    }

    /// Replicate the W component to all components
    pub fn splatW(self: UVec4) UVec4 {
        return replicate(self.value[3]);
    }

    /// Convert each component from an int to a float
    pub fn toFloat(self: UVec4) Vec4 {
        // Jolt's SSE path (_mm_cvtepi32_ps) converts the components as signed integers, while its NEON and scalar
        // paths convert them as unsigned. They differ for components >= 0x80000000; Zolt follows SSE (guide section 8)
        const signed: @Vector(4, i32) = @bitCast(self.value);
        return .{ .value = @floatFromInt(signed) };
    }

    /// Reinterpret UVec4 as a Vec4 (doesn't change the bits)
    pub fn reinterpretAsFloat(self: UVec4) Vec4 {
        return .{ .value = @bitCast(self.value) };
    }

    /// Dot product, returns the dot product in X, Y, Z and W components
    pub fn dotV(self: UVec4, other: UVec4) UVec4 {
        return replicate(self.dot(other));
    }

    /// Dot product
    pub fn dot(self: UVec4, other: UVec4) u32 {
        return @reduce(.Add, self.value *% other.value);
    }

    /// Store 4 ints to memory
    pub fn storeInt4(self: UVec4, out: *[4]u32) void {
        out.* = self.value;
    }

    /// Store 4 ints to memory, aligned to 16 bytes
    pub fn storeInt4Aligned(self: UVec4, out: *align(16) [4]u32) void {
        out.* = self.value;
    }

    /// Test if any of the components are true (true is when highest bit of component is set)
    pub fn testAnyTrue(self: UVec4) bool {
        return self.getTrues() != 0;
    }

    /// Test if any of X, Y or Z components are true (true is when highest bit of component is set)
    pub fn testAnyXYZTrue(self: UVec4) bool {
        return (self.getTrues() & 0b111) != 0;
    }

    /// Test if all components are true (true is when highest bit of component is set)
    pub fn testAllTrue(self: UVec4) bool {
        return self.getTrues() == 0b1111;
    }

    /// Test if X, Y and Z components are true (true is when highest bit of component is set)
    pub fn testAllXYZTrue(self: UVec4) bool {
        return (self.getTrues() & 0b111) == 0b111;
    }

    /// Count the number of components that are true (true is when highest bit of component is set)
    pub fn countTrues(self: UVec4) u32 {
        return @reduce(.Add, self.value >> @splat(31));
    }

    /// Store if X is true in bit 0, Y in bit 1, Z in bit 2 and W in bit 3 (true is when highest bit of component is set)
    pub fn getTrues(self: UVec4) u32 {
        // Note: not implemented as @bitCast(@Vector(4, bool)) -> u4, the Zig 0.16 self-hosted x86_64 backend miscompiles that
        const v = self.value >> @splat(31);
        return v[0] | (v[1] << 1) | (v[2] << 2) | (v[3] << 3);
    }

    /// Shift all components by count bits to the left (filling with zeros from the left)
    pub fn logicalShiftLeft(self: UVec4, comptime count: u5) UVec4 {
        return .{ .value = self.value << @splat(count) };
    }

    /// Shift all components by count bits to the right (filling with zeros from the right)
    pub fn logicalShiftRight(self: UVec4, comptime count: u5) UVec4 {
        return .{ .value = self.value >> @splat(count) };
    }

    /// Shift all components by count bits to the right (shifting in the value of the highest bit)
    pub fn arithmeticShiftRight(self: UVec4, comptime count: u5) UVec4 {
        const signed: @Vector(4, i32) = @bitCast(self.value);
        return .{ .value = @bitCast(signed >> @splat(count)) };
    }

    /// Takes the lower 4 16 bits and expands them to X, Y, Z and W
    pub fn expand4Uint16Lo(self: UVec4) UVec4 {
        const v = self.value;
        return init(v[0] & 0xffff, (v[0] >> 16) & 0xffff, v[1] & 0xffff, (v[1] >> 16) & 0xffff);
    }

    /// Takes the upper 4 16 bits and expands them to X, Y, Z and W
    pub fn expand4Uint16Hi(self: UVec4) UVec4 {
        const v = self.value;
        return init(v[2] & 0xffff, (v[2] >> 16) & 0xffff, v[3] & 0xffff, (v[3] >> 16) & 0xffff);
    }

    /// Takes byte 0 .. 3 and expands them to X, Y, Z and W
    pub fn expand4Byte0(self: UVec4) UVec4 {
        return expand4Byte(self.value[0]);
    }

    /// Takes byte 4 .. 7 and expands them to X, Y, Z and W
    pub fn expand4Byte4(self: UVec4) UVec4 {
        return expand4Byte(self.value[1]);
    }

    /// Takes byte 8 .. 11 and expands them to X, Y, Z and W
    pub fn expand4Byte8(self: UVec4) UVec4 {
        return expand4Byte(self.value[2]);
    }

    /// Takes byte 12 .. 15 and expands them to X, Y, Z and W
    pub fn expand4Byte12(self: UVec4) UVec4 {
        return expand4Byte(self.value[3]);
    }

    /// Shift vector components by 4 - count floats to the left, so if count = 1 the resulting vector is (W, 0, 0, 0),
    /// when count = 3 the resulting vector is (Y, Z, W, 0)
    pub fn shiftComponents4Minus(self: UVec4, count: u32) UVec4 {
        std.debug.assert(count <= 4);
        const components: [4]u32 = self.value;
        var result = [4]u32{ 0, 0, 0, 0 };
        for (0..count) |i|
            result[i] = components[i + 4 - count];
        return .{ .value = result };
    }

    pub fn format(self: UVec4, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try writer.print("{d}, {d}, {d}, {d}", .{ self.value[0], self.value[1], self.value[2], self.value[3] });
    }

    // Zolt helpers (no Jolt equivalent)

    /// Convert to a vector of bools, a component is true when its highest bit is set
    pub fn toBools(self: UVec4) @Vector(4, bool) {
        return (self.value & @as(Type, @splat(0x80000000))) != @as(Type, @splat(0));
    }

    /// Convert a vector of bools to a mask (true -> 0xffffffff, false -> 0)
    pub fn fromBools(b: @Vector(4, bool)) UVec4 {
        return .{ .value = @select(u32, b, @as(Type, @splat(0xffffffff)), @as(Type, @splat(0))) };
    }

    fn expand4Byte(v: u32) UVec4 {
        return init(v & 0xff, (v >> 8) & 0xff, (v >> 16) & 0xff, (v >> 24) & 0xff);
    }
};
