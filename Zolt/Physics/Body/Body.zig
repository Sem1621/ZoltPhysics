//! Port of: Jolt/Physics/Body/Body.h, Jolt/Physics/Body/Body.inl, Jolt/Physics/Body/Body.cpp
//! Status: stub
//!
//! Stub for Phase 4 (collision), replaced by the real port in Phase 5 (bodies and the physics system). The collision
//! code only uses `Body` as a pointer type: `CollisionCollector.onBody(*const Body)`, the ContactListener callbacks,
//! `SimShapeFilter.shouldCollide(*const Body, ...)` and BodyFilter. This stub keeps the C++ names of the members
//! and accessors it has, so code written against it keeps compiling when Phase 5 replaces it:
//! - `getID()`: the ID of the body (Jolt's `mID`);
//! - `getUserData()` / `setUserData()`: the user data (Jolt's `mUserData`), what user collectors typically read in
//!   `onBody`.
//! Tests create bodies with `Body{ .id = BodyID.init(n) }`.

const BodyID = @import("BodyID.zig").BodyID;

/// A rigid body that can be simulated using the physics system (stub, see the file comment)
pub const Body = struct {
    /// ID of the body (index in body array + sequence number)
    id: BodyID = .invalid,

    /// User data, can be used for anything by the application
    user_data: u64 = 0,

    /// Get the id of this body
    pub fn getID(self: *const Body) BodyID {
        return self.id;
    }

    /// Access to the user data, can be used for anything by the application
    pub fn getUserData(self: *const Body) u64 {
        return self.user_data;
    }

    /// Access to the user data, can be used for anything by the application
    pub fn setUserData(self: *Body, user_data: u64) void {
        self.user_data = user_data;
    }
};

test "Body stub" {
    const std = @import("std");
    var body: Body = .{ .id = .init(7) };
    body.setUserData(42);
    try std.testing.expect(body.getID().eql(.init(7)));
    try std.testing.expectEqual(@as(u64, 42), body.getUserData());
}
