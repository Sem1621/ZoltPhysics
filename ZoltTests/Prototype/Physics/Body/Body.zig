//! Port of: Jolt/Physics/Body/Body.h (stub: only what the collision code of Phase 4 references, Body is Phase 5)
//! Status: stub

const BodyID = @import("BodyID.zig").BodyID;

/// A rigid body that can be simulated using the physics system (stub)
pub const Body = struct {
    id: BodyID = .invalid,

    /// Get the id of this body
    pub fn getID(self: *const Body) BodyID {
        return self.id;
    }
};
