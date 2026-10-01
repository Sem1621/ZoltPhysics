//! Port of: Jolt/Math/Float2.h
//! Status: complete

const std = @import("std");

/// Class that holds 2 floats, used as a storage class mainly
pub const Float2 = extern struct {
    x: f32,
    y: f32,

    pub fn init(x: f32, y: f32) Float2 {
        return .{ .x = x, .y = y };
    }

    /// operator ==
    pub fn eql(self: Float2, other: Float2) bool {
        return self.x == other.x and self.y == other.y;
    }

    pub fn format(self: Float2, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try writer.print("{d}, {d}", .{ self.x, self.y });
    }
};
