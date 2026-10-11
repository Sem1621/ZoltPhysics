//! Port of: Jolt/Physics/Collision/NarrowPhaseQuery.h, Jolt/Physics/Collision/NarrowPhaseQuery.cpp
//! Status: stub
//! Missing: the queries (Wave A, A7): CastRay, CollidePoint, CollideShape, CollideShapeWithInternalEdgeRemoval,
//! CastShape, CollectTransformedShapes.
//!
//! F2 stub: PhysicsSystem holds a locking and a non locking NarrowPhaseQuery (Jolt: mNarrowPhaseQueryLocking /
//! mNarrowPhaseQueryNoLock) and initializes them in its init, so the type, its members and `init` exist with their
//! final names.

const BodyLockInterface = @import("../Body/BodyLockInterface.zig").BodyLockInterface;
const BroadPhaseQuery = @import("BroadPhase/BroadPhaseQuery.zig").BroadPhaseQuery;

/// Class that provides an interface for doing precise collision detection against the broad and then the narrow phase.
/// Unlike a BroadPhaseQuery, the NarrowPhaseQuery will test against shapes and will return collision information against triangles, spheres etc.
pub const NarrowPhaseQuery = struct {
    body_lock_interface: ?*const BodyLockInterface = null,
    broad_phase_query: ?*const BroadPhaseQuery = null,

    /// Not initialized (Jolt's constructor)
    pub const empty: NarrowPhaseQuery = .{};

    /// Initialize the interface (should only be called by PhysicsSystem)
    pub fn init(body_lock_interface: *const BodyLockInterface, broad_phase_query: *const BroadPhaseQuery) NarrowPhaseQuery {
        return .{ .body_lock_interface = body_lock_interface, .broad_phase_query = broad_phase_query };
    }
};
