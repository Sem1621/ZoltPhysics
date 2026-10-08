//! Port of: Jolt/Physics/Collision/ActiveEdgeMode.h
//! Status: complete

/// How to treat active/inactive edges.
/// An active edge is an edge that either has no neighbouring edge or if the angle between the two connecting faces is too large, see: ActiveEdges
pub const ActiveEdgeMode = enum(u8) {
    /// Do not collide with inactive edges. For physics simulation, this gives less ghost collisions.
    collide_only_with_active,
    /// Collide with all edges. Use this when you're interested in all collisions.
    collide_with_all,
};
