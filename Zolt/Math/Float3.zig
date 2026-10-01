//! Port of: Jolt/Math/Float3.h
//! Status: complete

const std = @import("std");
const HashCombine = @import("../Core/HashCombine.zig");

/// Class that holds 3 floats. Used as a storage class. Convert to Vec3 for calculations.
pub const Float3 = extern struct {
    x: f32,
    y: f32,
    z: f32,

    pub fn init(x: f32, y: f32, z: f32) Float3 {
        return .{ .x = x, .y = y, .z = z };
    }

    /// operator [] const
    pub fn getComponent(self: Float3, coordinate: u32) f32 {
        std.debug.assert(coordinate < 3);
        return switch (coordinate) {
            0 => self.x,
            1 => self.y,
            else => self.z,
        };
    }

    /// operator ==
    pub fn eql(self: Float3, other: Float3) bool {
        return self.x == other.x and self.y == other.y and self.z == other.z;
    }

    /// JPH_MAKE_HASHABLE(JPH::Float3, t.x, t.y, t.z)
    pub fn getHash(self: Float3) u64 {
        return HashCombine.hashCombineArgs(.{ self.x, self.y, self.z });
    }
};

/// Array<Float3> (VertexList), use with an explicit allocator
pub const VertexList = std.ArrayList(Float3);
