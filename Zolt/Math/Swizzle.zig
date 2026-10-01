//! Port of: Jolt/Math/Swizzle.h
//! Status: complete

/// Component to use when swizzling (SWIZZLE_X .. SWIZZLE_W). Used as `v.swizzle(.y, .x, .unused, .w)`.
pub const Swizzle = enum(u2) {
    /// Use the X component
    x = 0,
    /// Use the Y component
    y = 1,
    /// Use the Z component
    z = 2,
    /// Use the W component
    w = 3,

    /// We always use the Z component when we don't specifically want to initialize a value, this is
    /// consistent with what is done in Vec3.init(x, y, z), Vec3.fromFloat3 and Vec3.loadFloat3Unsafe (SWIZZLE_UNUSED)
    pub const unused: Swizzle = .z;
};
