//! Port of: Jolt/Physics/Collision/CollidePointResult.h
//! Status: complete

const BodyID = @import("../Body/BodyID.zig").BodyID;
const SubShapeID = @import("Shape/SubShapeID.zig").SubShapeID;

/// Structure that holds the result of colliding a point against a shape
pub const CollidePointResult = struct {
    /// Body that was hit
    body_id: BodyID = .invalid,
    /// Sub shape ID of shape that we collided against
    sub_shape_id2: SubShapeID = .empty,

    /// Function required by the CollisionCollector. A smaller fraction is considered to be a 'better hit'. For point queries there is no sensible return value.
    pub fn getEarlyOutFraction(self: *const CollidePointResult) f32 {
        _ = self;
        return 0.0;
    }
};

test "CollidePointResult" {
    const std = @import("std");
    const r: CollidePointResult = .{};
    try std.testing.expect(r.body_id.isInvalid() and r.sub_shape_id2.isEmpty());
    try std.testing.expectEqual(@as(f32, 0.0), r.getEarlyOutFraction());
}
