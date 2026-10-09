//! Port of: Jolt/Physics/Collision/AABoxCast.h
//! Status: complete
//!
//! Plain data without defaults, like the C++ (initialize all fields: `.{ .box = ..., .direction = ... }`).

const AABox = @import("../../Geometry/AABox.zig").AABox;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;

/// Structure that holds AABox moving linearly through 3d space
pub const AABoxCast = struct {
    /// Axis aligned box at starting location
    box: AABox,
    /// Direction and length of the cast (anything beyond this length will not be reported as a hit)
    direction: Vec3,
};

test "AABoxCast" {
    const std = @import("std");
    const cast: AABoxCast = .{ .box = .init(Vec3.replicate(-1), Vec3.replicate(1)), .direction = Vec3.init(0, 2, 0) };
    try std.testing.expect(cast.box.getCenter().eql(Vec3.zero()));
    try std.testing.expect(cast.direction.eql(Vec3.init(0, 2, 0)));
}
