//! Port of: Jolt/Math/Float4.h
//! Status: complete

const std = @import("std");

/// Class that holds 4 floats. Used as a storage class. Convert to Vec4 for calculations.
pub const Float4 = extern struct {
    x: f32,
    y: f32,
    z: f32,
    w: f32,

    pub fn init(x: f32, y: f32, z: f32, w: f32) Float4 {
        return .{ .x = x, .y = y, .z = z, .w = w };
    }

    /// operator [] const
    pub fn getComponent(self: Float4, coordinate: u32) f32 {
        std.debug.assert(coordinate < 4);
        return switch (coordinate) {
            0 => self.x,
            1 => self.y,
            2 => self.z,
            else => self.w,
        };
    }

    /// operator ==
    pub fn eql(self: Float4, other: Float4) bool {
        return self.x == other.x and self.y == other.y and self.z == other.z and self.w == other.w;
    }
};
