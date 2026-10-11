//! Port of: Jolt/Geometry/RayCapsule.h
//! Status: complete

const std = @import("std");
const math = @import("../Math/Math.zig");
const rayInfiniteCylinder = @import("RayCylinder.zig").rayInfiniteCylinder;
const raySphere = @import("RaySphere.zig").raySphere;
const Vec3 = @import("../Math/Vec3.zig").Vec3;

/// Tests a ray starting at ray_origin and extending infinitely in ray_direction
/// against a capsule centered around the origin with its axis along the Y axis and half height specified.
/// @return FLT_MAX if there is no intersection, otherwise the fraction along the ray.
/// @param ray_direction Ray direction. Does not need to be normalized.
/// @param ray_origin Origin of the ray. If the ray starts inside the capsule, the returned fraction will be 0.
/// @param capsule_half_height Distance from the origin to the center of the top sphere (or that of the bottom)
/// @param capsule_radius Radius of the top/bottom sphere
pub fn rayCapsule(ray_origin: Vec3, ray_direction: Vec3, capsule_half_height: f32, capsule_radius: f32) f32 {
    // Test infinite cylinder
    const cylinder = rayInfiniteCylinder(ray_origin, ray_direction, capsule_radius);
    if (cylinder == math.flt_max)
        return math.flt_max;

    // If this hit is in the finite cylinder we have our fraction
    if (@abs(ray_origin.getY() + cylinder * ray_direction.getY()) <= capsule_half_height)
        return cylinder;

    // Test upper and lower sphere
    const sphere_center = Vec3.init(0, capsule_half_height, 0);
    const upper = raySphere(ray_origin, ray_direction, sphere_center, capsule_radius);
    const lower = raySphere(ray_origin, ray_direction, sphere_center.negate(), capsule_radius);
    return math.min(upper, lower);
}

test "rayCapsule" {
    // Hit the cylinder part
    try std.testing.expectEqual(@as(f32, 1), rayCapsule(Vec3.init(-3, 0, 0), Vec3.init(2, 0, 0), 2, 1));

    // Hit the top and bottom sphere
    try std.testing.expectEqual(@as(f32, 1), rayCapsule(Vec3.init(0, 5, 0), Vec3.init(0, -2, 0), 2, 1));
    try std.testing.expectEqual(@as(f32, 1), rayCapsule(Vec3.init(0, -5, 0), Vec3.init(0, 2, 0), 2, 1));

    // Start inside
    try std.testing.expectEqual(@as(f32, 0), rayCapsule(Vec3.init(0, 2.5, 0), Vec3.init(1, 0, 0), 2, 1));

    // Miss
    try std.testing.expectEqual(math.flt_max, rayCapsule(Vec3.init(-3, 0, 0), Vec3.init(-1, 0, 0), 2, 1));
    try std.testing.expectEqual(math.flt_max, rayCapsule(Vec3.init(-3, 3.5, 0), Vec3.init(1, 0, 0), 2, 1));
}
