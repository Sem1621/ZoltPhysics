//! Port of: Jolt/Math/BVec16.h, Jolt/Math/BVec16.inl
//! Status: complete
//!
//! Semantics follow the portable fallback of BVec16.inl: `equals` sets only the highest bit of a
//! component (0x80) when the bytes are equal, the SSE version of Jolt sets all bits (0xff). Both
//! are "true" for every function that tests the highest bit.

const std = @import("std");
const Core = @import("../Core/Core.zig");

/// A vector consisting of 16 bytes
pub const BVec16 = extern struct {
    /// Underlying vector type
    pub const Type = @Vector(16, u8);

    value: Type align(Core.vector_alignment),

    comptime {
        std.debug.assert(@sizeOf(BVec16) == 16);
        std.debug.assert(@alignOf(BVec16) == Core.vector_alignment);
    }

    /// Create a vector from 16 bytes
    pub fn init(b0: u8, b1: u8, b2: u8, b3: u8, b4: u8, b5: u8, b6: u8, b7: u8, b8: u8, b9: u8, b10: u8, b11: u8, b12: u8, b13: u8, b14: u8, b15: u8) BVec16 {
        return .{ .value = .{ b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11, b12, b13, b14, b15 } };
    }

    /// Create a vector from two uint64's, v0 holds bytes 0 .. 7 and v1 bytes 8 .. 15 in native byte order (BVec16(uint64, uint64))
    pub fn fromUint64(v0: u64, v1: u64) BVec16 {
        return .{ .value = @bitCast([2]u64{ v0, v1 }) };
    }

    /// Comparison (operator ==)
    pub fn eql(self: BVec16, other: BVec16) bool {
        return equals(self, other).testAllTrue();
    }

    /// Vector with all zeros
    pub fn zero() BVec16 {
        return .{ .value = @splat(0) };
    }

    /// Replicate int v across all components
    pub fn replicate(v: u8) BVec16 {
        return .{ .value = @splat(v) };
    }

    /// Load 16 bytes from memory
    pub fn loadByte16(v: *const [16]u8) BVec16 {
        return .{ .value = v.* };
    }

    /// Equals (component wise), highest bit of each component that is set is considered true
    pub fn equals(v1: BVec16, v2: BVec16) BVec16 {
        // The fallback implementation of Jolt only sets the highest bit of each byte that is equal
        return .{ .value = @select(u8, v1.value == v2.value, @as(Type, @splat(0x80)), @as(Type, @splat(0))) };
    }

    /// Logical or (component wise) (sOr)
    pub fn bitOr(v1: BVec16, v2: BVec16) BVec16 {
        return .{ .value = v1.value | v2.value };
    }

    /// Logical xor (component wise) (sXor)
    pub fn bitXor(v1: BVec16, v2: BVec16) BVec16 {
        return .{ .value = v1.value ^ v2.value };
    }

    /// Logical and (component wise) (sAnd)
    pub fn bitAnd(v1: BVec16, v2: BVec16) BVec16 {
        return .{ .value = v1.value & v2.value };
    }

    /// Logical not (component wise) (sNot)
    pub fn bitNot(v1: BVec16) BVec16 {
        return .{ .value = ~v1.value };
    }

    /// Get component by index (operator [] const)
    pub fn getComponent(self: BVec16, coordinate: u32) u8 {
        std.debug.assert(coordinate < 16);
        const components: [16]u8 = self.value; // Vectors can only be indexed with comptime indices
        return components[coordinate];
    }

    /// Set component by index (operator [])
    pub fn setComponent(self: *BVec16, coordinate: u32, v: u8) void {
        std.debug.assert(coordinate < 16);
        var components: [16]u8 = self.value; // Vectors can only be indexed with comptime indices
        components[coordinate] = v;
        self.value = components;
    }

    /// Test if any of the components are true (true is when highest bit of component is set)
    pub fn testAnyTrue(self: BVec16) bool {
        return @reduce(.Or, self.value & @as(Type, @splat(0x80))) != 0;
    }

    /// Test if all components are true (true is when highest bit of component is set)
    pub fn testAllTrue(self: BVec16) bool {
        return @reduce(.And, self.value & @as(Type, @splat(0x80))) == 0x80;
    }

    /// Store if component 0 is true in bit 0, component 1 in bit 1, etc. (true is when highest bit of component is set)
    pub fn getTrues(self: BVec16) u32 {
        const high_bits: @Vector(16, u16) = self.value >> @splat(7);
        const shifts = comptime std.simd.iota(u4, 16);
        return @reduce(.Or, high_bits << shifts);
    }

    pub fn format(self: BVec16, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        const components: [16]u8 = self.value;
        for (components, 0..) |c, i| {
            if (i > 0) try writer.writeAll(", ");
            try writer.print("{d}", .{c});
        }
    }
};
