//! Port of: Jolt/Physics/Collision/BackFaceMode.h
//! Status: complete

/// How collision detection functions will treat back facing triangles
pub const BackFaceMode = enum(u8) {
    /// Ignore collision with back facing surfaces/triangles
    ignore_back_faces,
    /// Collide with back facing surfaces/triangles
    collide_with_back_faces,
};
