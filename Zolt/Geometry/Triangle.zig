//! Port of: Jolt/Geometry/Triangle.h
//! Status: complete

const std = @import("std");
const Float3 = @import("../Math/Float3.zig").Float3;
const Vec3 = @import("../Math/Vec3.zig").Vec3;

/// A simple triangle and its material
pub const Triangle = extern struct {
    /// Vertices
    v: [3]Float3,
    /// Follows v[3] so that we can read v as 4 vectors
    material_index: u32 = 0,
    /// User data that can be used for anything by the application, e.g. for tracking the original index of the triangle
    user_data: u32 = 0,

    comptime {
        std.debug.assert(@sizeOf(Triangle) == 11 * @sizeOf(u32));
    }

    /// Optional arguments of the constructors
    pub const Options = struct {
        material_index: u32 = 0,
        user_data: u32 = 0,
    };

    /// Constructor (Triangle(Vec3Arg, Vec3Arg, Vec3Arg, uint32 inMaterialIndex = 0, uint32 inUserData = 0))
    pub fn init(v1: Vec3, v2: Vec3, v3: Vec3, opts: Options) Triangle {
        var result: Triangle = .{ .v = undefined, .material_index = opts.material_index, .user_data = opts.user_data };
        v1.storeFloat3(&result.v[0]);
        v2.storeFloat3(&result.v[1]);
        v3.storeFloat3(&result.v[2]);
        return result;
    }

    /// Constructor (Triangle(const Float3 &, const Float3 &, const Float3 &, uint32 inMaterialIndex = 0, uint32 inUserData = 0))
    pub fn fromFloat3(v1: Float3, v2: Float3, v3: Float3, opts: Options) Triangle {
        return .{ .v = .{ v1, v2, v3 }, .material_index = opts.material_index, .user_data = opts.user_data };
    }

    /// Get center of triangle
    pub fn getCentroid(self: Triangle) Vec3 {
        const one_third: f32 = @as(f32, 1.0) / @as(f32, 3.0);
        return Vec3.loadFloat3Unsafe(&self.v[0]).add(Vec3.loadFloat3Unsafe(&self.v[1])).add(Vec3.loadFloat3Unsafe(&self.v[2])).mulScalar(one_third);
    }
};

/// Array<Triangle>, use with an explicit allocator
pub const TriangleList = std.ArrayList(Triangle);

test "Triangle" {
    const t = Triangle.init(Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(7, 8, 9), .{ .material_index = 3, .user_data = 4 });
    try std.testing.expectEqual(@as(u32, 3), t.material_index);
    try std.testing.expectEqual(@as(u32, 4), t.user_data);
    try std.testing.expect(t.v[1].eql(Float3.init(4, 5, 6)));
    try std.testing.expect(t.getCentroid().isClose(Vec3.init(4, 5, 6), .{}));

    const t2 = Triangle.fromFloat3(Float3.init(1, 2, 3), Float3.init(4, 5, 6), Float3.init(7, 8, 9), .{});
    try std.testing.expectEqual(@as(u32, 0), t2.material_index);
    try std.testing.expectEqual(@as(u32, 0), t2.user_data);
    try std.testing.expect(t2.getCentroid().eql(t.getCentroid()));

    // mMaterialIndex follows mV[3] (the vertices can be read as 4 vectors)
    try std.testing.expectEqual(@as(usize, 36), @offsetOf(Triangle, "material_index"));
}
