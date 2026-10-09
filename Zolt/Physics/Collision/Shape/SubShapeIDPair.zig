//! Port of: Jolt/Physics/Collision/Shape/SubShapeIDPair.h
//! Status: complete
//!
//! An `extern struct` in Jolt's field order (16 bytes, hashed as bytes). The default constructor is the field
//! defaults (invalid body IDs, empty sub shape IDs), `operator ==` is `eql`, `operator <` is `lessThan` and
//! JPH_MAKE_STD_HASH is `getHash`.

const std = @import("std");
const HashCombine = @import("../../../Core/HashCombine.zig");
const UVec4 = @import("../../../Math/UVec4.zig").UVec4;
const BodyID = @import("../../Body/BodyID.zig").BodyID;
const SubShapeID = @import("SubShapeID.zig").SubShapeID;

/// A pair of bodies and their sub shape ID's. Can be used as a key in a map to find a contact point.
pub const SubShapeIDPair = extern struct {
    body1_id: BodyID = .invalid,
    sub_shape_id1: SubShapeID = .empty,
    body2_id: BodyID = .invalid,
    sub_shape_id2: SubShapeID = .empty,

    /// Constructor
    pub fn init(body1_id: BodyID, sub_shape_id1: SubShapeID, body2_id: BodyID, sub_shape_id2: SubShapeID) SubShapeIDPair {
        return .{ .body1_id = body1_id, .sub_shape_id1 = sub_shape_id1, .body2_id = body2_id, .sub_shape_id2 = sub_shape_id2 };
    }

    /// Equality operator
    pub fn eql(self: *const SubShapeIDPair, other: *const SubShapeIDPair) bool {
        return UVec4.loadInt4(@ptrCast(self)).eql(UVec4.loadInt4(@ptrCast(other)));
    }

    /// Less than operator, used to consistently order contact points for a deterministic simulation
    pub fn lessThan(self: *const SubShapeIDPair, other: *const SubShapeIDPair) bool {
        if (!self.body1_id.eql(other.body1_id))
            return self.body1_id.lessThan(other.body1_id);

        if (self.sub_shape_id1.getValue() != other.sub_shape_id1.getValue())
            return self.sub_shape_id1.getValue() < other.sub_shape_id1.getValue();

        if (!self.body2_id.eql(other.body2_id))
            return self.body2_id.lessThan(other.body2_id);

        return self.sub_shape_id2.getValue() < other.sub_shape_id2.getValue();
    }

    pub fn getBody1ID(self: *const SubShapeIDPair) BodyID {
        return self.body1_id;
    }

    pub fn getSubShapeID1(self: *const SubShapeIDPair) SubShapeID {
        return self.sub_shape_id1;
    }

    pub fn getBody2ID(self: *const SubShapeIDPair) BodyID {
        return self.body2_id;
    }

    pub fn getSubShapeID2(self: *const SubShapeIDPair) SubShapeID {
        return self.sub_shape_id2;
    }

    /// Hash of the 16 bytes (GetHash, also used for std::hash / JPH::Hash)
    pub fn getHash(self: *const SubShapeIDPair) u64 {
        return HashCombine.hashBytes(std.mem.asBytes(self));
    }

    comptime {
        std.debug.assert(@sizeOf(SubShapeIDPair) == 16); // Unexpected size
        std.debug.assert(@alignOf(SubShapeIDPair) == 4); // Assuming 4 byte aligned
    }
};

test "SubShapeIDPair" {
    var s1: SubShapeID = .{};
    s1.setValue(3);
    const a = SubShapeIDPair.init(.init(1), s1, .init(2), .empty);
    const b = SubShapeIDPair.init(.init(1), s1, .init(2), .empty);
    const c = SubShapeIDPair.init(.init(1), .empty, .init(2), .empty);
    try std.testing.expect(a.eql(&b));
    try std.testing.expect(!a.eql(&c));
    try std.testing.expect(a.lessThan(&c)); // 3 < 0xffffffff
    try std.testing.expect(!c.lessThan(&a));
    try std.testing.expect(!a.lessThan(&b));
    try std.testing.expectEqual(a.getHash(), b.getHash());
    try std.testing.expectEqual(a.getHash(), HashCombine.hash(a));
    try std.testing.expect(a.getBody1ID().eql(.init(1)) and a.getBody2ID().eql(.init(2)));
    try std.testing.expect(a.getSubShapeID1().eql(s1) and a.getSubShapeID2().isEmpty());
    const default: SubShapeIDPair = .{};
    try std.testing.expect(default.getBody1ID().isInvalid() and default.getSubShapeID1().isEmpty());
}
