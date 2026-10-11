//! Port of: Jolt/Geometry/RayCylinder.h
//! Status: complete
//!
//! The two RayCylinder overloads get distinct names: `rayInfiniteCylinder(origin, direction, radius)` (infinite
//! cylinder) and `rayCylinder(origin, direction, half_height, radius)` (finite cylinder).

const std = @import("std");
const math = @import("../Math/Math.zig");
const findRoot = @import("../Math/FindRoot.zig").findRoot;
const UVec4 = @import("../Math/UVec4.zig").UVec4;
const Vec3 = @import("../Math/Vec3.zig").Vec3;

/// Tests a ray starting at ray_origin and extending infinitely in ray_direction
/// against an infinite cylinder centered along the Y axis
/// @return FLT_MAX if there is no intersection, otherwise the fraction along the ray.
/// @param ray_direction Direction of the ray. Does not need to be normalized.
/// @param ray_origin Origin of the ray. If the ray starts inside the cylinder, the returned fraction will be 0.
/// @param cylinder_radius Radius of the infinite cylinder
/// (the RayCylinder overload without half height)
pub fn rayInfiniteCylinder(ray_origin: Vec3, ray_direction: Vec3, cylinder_radius: f32) f32 {
    // Remove Y component of ray to see of ray intersects with infinite cylinder
    const mask_y = UVec4.init(0, 0xffffffff, 0, 0);
    const origin_xz = Vec3.select(ray_origin, Vec3.zero(), mask_y);
    const origin_xz_len_sq = origin_xz.lengthSq();
    const r_sq = math.square(cylinder_radius);
    if (origin_xz_len_sq > r_sq) {
        // Ray starts outside of the infinite cylinder
        // Solve: |RayOrigin_xz + fraction * RayDirection_xz|^2 = r^2 to find fraction
        const direction_xz = Vec3.select(ray_direction, Vec3.zero(), mask_y);
        const a = direction_xz.lengthSq();
        const b = 2.0 * origin_xz.dot(direction_xz);
        const c = origin_xz_len_sq - r_sq;
        const roots = findRoot(f32, a, b, c);
        if (roots.num_roots == 0)
            return math.flt_max; // No intersection with infinite cylinder

        // Get fraction corresponding to the ray entering the circle
        const fraction = math.min(roots.x1, roots.x2);
        if (fraction >= 0.0)
            return fraction;
    } else {
        // Ray starts inside the infinite cylinder
        return 0.0;
    }

    // No collision
    return math.flt_max;
}

/// Test a ray against a cylinder centered around the origin with its axis along the Y axis and half height specified.
/// @return FLT_MAX if there is no intersection, otherwise the fraction along the ray.
/// @param ray_direction Ray direction. Does not need to be normalized.
/// @param ray_origin Origin of the ray. If the ray starts inside the cylinder, the returned fraction will be 0.
/// @param cylinder_radius Radius of the cylinder
/// @param cylinder_half_height Distance from the origin to the top (or bottom) of the cylinder
pub fn rayCylinder(ray_origin: Vec3, ray_direction: Vec3, cylinder_half_height: f32, cylinder_radius: f32) f32 {
    // Test infinite cylinder
    const fraction = rayInfiniteCylinder(ray_origin, ray_direction, cylinder_radius);
    if (fraction == math.flt_max)
        return math.flt_max;

    // If this hit is in the finite cylinder we have our fraction
    if (@abs(ray_origin.getY() + fraction * ray_direction.getY()) <= cylinder_half_height)
        return fraction;

    // Check if ray could hit the top or bottom plane of the cylinder
    const direction_y = ray_direction.getY();
    if (direction_y != 0.0) {
        // Solving line equation: x = ray_origin + fraction * ray_direction
        // and plane equation: plane_normal . x + plane_constant = 0
        // fraction = (-plane_constant - plane_normal . ray_origin) / (plane_normal . ray_direction)
        // when the ray_direction.y < 0:
        // plane_constant = -cylinder_half_height, plane_normal = (0, 1, 0)
        // else
        // plane_constant = -cylinder_half_height, plane_normal = (0, -1, 0)
        const origin_y = ray_origin.getY();
        const plane_fraction = if (direction_y < 0.0)
            (cylinder_half_height - origin_y) / direction_y
        else
            -(cylinder_half_height + origin_y) / direction_y;

        // Check if the hit is in front of the ray
        if (plane_fraction >= 0.0) {
            // Test if this hit is inside the cylinder
            const point = ray_origin.add(ray_direction.mulScalar(plane_fraction));
            const dist_sq = math.square(point.getX()) + math.square(point.getZ());
            if (dist_sq <= math.square(cylinder_radius))
                return plane_fraction;
        }
    }

    // No collision
    return math.flt_max;
}

test "rayInfiniteCylinder" {
    // Hit from the outside
    try std.testing.expectEqual(@as(f32, 1), rayInfiniteCylinder(Vec3.init(-3, 10, 0), Vec3.init(2, 5, 0), 1));

    // Start inside
    try std.testing.expectEqual(@as(f32, 0), rayInfiniteCylinder(Vec3.init(0.5, 10, 0), Vec3.init(2, 5, 0), 1));

    // Miss: pointing away, parallel to the axis and passing by
    try std.testing.expectEqual(math.flt_max, rayInfiniteCylinder(Vec3.init(-3, 0, 0), Vec3.init(-1, 0, 0), 1));
    try std.testing.expectEqual(math.flt_max, rayInfiniteCylinder(Vec3.init(-3, 0, 0), Vec3.init(0, 1, 0), 1));
    try std.testing.expectEqual(math.flt_max, rayInfiniteCylinder(Vec3.init(-3, 0, 2), Vec3.init(1, 0, 0), 1));
}

test "rayCylinder" {
    // Hit the side
    try std.testing.expectEqual(@as(f32, 1), rayCylinder(Vec3.init(-3, 0, 0), Vec3.init(2, 0, 0), 2, 1));

    // Hit the top and the bottom cap
    try std.testing.expectEqual(@as(f32, 2), rayCylinder(Vec3.init(0, 6, 0), Vec3.init(0, -2, 0), 2, 1));
    try std.testing.expectEqual(@as(f32, 2), rayCylinder(Vec3.init(0, -6, 0), Vec3.init(0, 2, 0), 2, 1));

    // Start inside
    try std.testing.expectEqual(@as(f32, 0), rayCylinder(Vec3.zero(), Vec3.init(0, 1, 0), 2, 1));

    // Miss: above the cylinder, horizontal ray and ray pointing away from the cap
    try std.testing.expectEqual(math.flt_max, rayCylinder(Vec3.init(-3, 3, 0), Vec3.init(1, 0, 0), 2, 1));
    try std.testing.expectEqual(math.flt_max, rayCylinder(Vec3.init(0, 6, 0), Vec3.init(0, 1, 0), 2, 1));
}
