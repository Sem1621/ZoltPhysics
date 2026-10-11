//! Port of: Jolt/Physics/Collision/CollectFacesMode.h
//! Status: complete

/// Whether or not to collect faces, used by CastShape and CollideShape
pub const CollectFacesMode = enum(u8) {
    /// mShape1/2Face is desired
    collect_faces,
    /// mShape1/2Face is not desired
    no_faces,
};
