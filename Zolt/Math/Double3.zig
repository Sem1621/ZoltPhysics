//! Port of: Jolt/Math/Double3.h
//! Status: complete

const std = @import("std");
const HashCombine = @import("../Core/HashCombine.zig");

/// Class that holds 3 doubles. Used as a storage class. Convert to DVec3 for calculations.
pub const Double3 = extern struct {
    x: f64,
    y: f64,
    z: f64,

    pub fn init(x: f64, y: f64, z: f64) Double3 {
        return .{ .x = x, .y = y, .z = z };
    }

    /// operator [] const
    pub fn getComponent(self: Double3, coordinate: u32) f64 {
        std.debug.assert(coordinate < 3);
        return switch (coordinate) {
            0 => self.x,
            1 => self.y,
            else => self.z,
        };
    }

    /// operator ==
    pub fn eql(self: Double3, other: Double3) bool {
        return self.x == other.x and self.y == other.y and self.z == other.z;
    }

    /// JPH_MAKE_HASHABLE(JPH::Double3, t.x, t.y, t.z)
    pub fn getHash(self: Double3) u64 {
        return HashCombine.hashCombineArgs(.{ self.x, self.y, self.z });
    }
};
