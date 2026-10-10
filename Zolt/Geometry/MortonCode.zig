//! Port of: Jolt/Geometry/MortonCode.h
//! Status: complete

const std = @import("std");
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const AABox = @import("AABox.zig").AABox;

pub const MortonCode = struct {
    /// First converts a floating point value in the range [0, 1] to a 10 bit fixed point integer.
    /// Then expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
    pub fn expandBits(v_in: f32) u32 {
        std.debug.assert(v_in >= 0.0 and v_in <= 1.0);
        var v: u32 = @intFromFloat(v_in * 1023.0 + 0.5);
        std.debug.assert(v < 1024);
        v = (v *% 0x00010001) & 0xFF0000FF;
        v = (v *% 0x00000101) & 0x0F00F00F;
        v = (v *% 0x00000011) & 0xC30C30C3;
        v = (v *% 0x00000005) & 0x49249249;
        return v;
    }

    /// Calculate the morton code for vector, given that all vectors lie in vector_bounds
    pub fn getMortonCode(vector: Vec3, vector_bounds: AABox) u32 {
        // Convert to 10 bit fixed point
        const scaled = vector.sub(vector_bounds.min).div(vector_bounds.getSize());
        const x = expandBits(scaled.getX());
        const y = expandBits(scaled.getY());
        const z = expandBits(scaled.getZ());
        return (x << 2) +% (y << 1) +% z;
    }
};

test "MortonCode" {
    try std.testing.expectEqual(@as(u32, 0), MortonCode.expandBits(0));
    try std.testing.expectEqual(@as(u32, 0x09249249), MortonCode.expandBits(1)); // 1023 = 10 bits set, every 3rd bit
    try std.testing.expectEqual(@as(u32, 0x08000000), MortonCode.expandBits(512.0 / 1023.0)); // Highest bit only

    const bounds = AABox.init(Vec3.zero(), Vec3.one());
    try std.testing.expectEqual(@as(u32, 0), MortonCode.getMortonCode(Vec3.zero(), bounds));
    try std.testing.expectEqual(@as(u32, 0x3fffffff), MortonCode.getMortonCode(Vec3.one(), bounds));
    try std.testing.expectEqual(@as(u32, 0x24924924), MortonCode.getMortonCode(Vec3.init(1, 0, 0), bounds));
    try std.testing.expectEqual(@as(u32, 0x12492492), MortonCode.getMortonCode(Vec3.init(0, 1, 0), bounds));
    try std.testing.expectEqual(@as(u32, 0x09249249), MortonCode.getMortonCode(Vec3.init(0, 0, 1), bounds));
}
