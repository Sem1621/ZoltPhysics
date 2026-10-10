//! Port of: Jolt/Geometry/RaySphere.h
//! Status: complete
//!
//! The overload with the outMinFraction / outMaxFraction out parameters is `raySphereMinMax`, it returns
//! `RaySphereMinMax{ .num_intersections, .min_fraction, .max_fraction }`.

const std = @import("std");
const math = @import("../Math/Math.zig");
const findRoot = @import("../Math/FindRoot.zig").findRoot;
const Vec3 = @import("../Math/Vec3.zig").Vec3;

/// Tests a ray starting at ray_origin and extending infinitely in ray_direction against a sphere,
/// @return FLT_MAX if there is no intersection, otherwise the fraction along the ray.
/// @param ray_origin Ray origin. If the ray starts inside the sphere, the returned fraction will be 0.
/// @param ray_direction Ray direction. Does not need to be normalized.
/// @param sphere_center Position of the center of the sphere
/// @param sphere_radius Radius of the sphere
pub fn raySphere(ray_origin: Vec3, ray_direction: Vec3, sphere_center: Vec3, sphere_radius: f32) f32 {
    // Solve: |RayOrigin + fraction * RayDirection - SphereCenter|^2 = SphereRadius^2 for fraction
    const center_origin = ray_origin.sub(sphere_center);
    const a = ray_direction.lengthSq();
    const b = 2.0 * ray_direction.dot(center_origin);
    const c = center_origin.lengthSq() - sphere_radius * sphere_radius;
    const roots = findRoot(f32, a, b, c);
    if (roots.num_roots == 0)
        return if (c <= 0.0) 0.0 else math.flt_max; // Return if origin is inside the sphere

    // Sort so that the smallest is first
    var fraction1 = roots.x1;
    var fraction2 = roots.x2;
    if (fraction1 > fraction2)
        std.mem.swap(f32, &fraction1, &fraction2);

    // Test solution with lowest fraction, this will be the ray entering the sphere
    if (fraction1 >= 0.0)
        return fraction1; // Sphere is before the ray start

    // Test solution with highest fraction, this will be the ray leaving the sphere
    if (fraction2 >= 0.0)
        return 0.0; // We start inside the sphere

    // No solution
    return math.flt_max;
}

/// Result of `raySphereMinMax` (the return value and the outMinFraction / outMaxFraction out parameters)
pub const RaySphereMinMax = struct {
    /// The amount of intersections with the sphere (0, 1 or 2)
    num_intersections: i32,
    /// Lowest intersection fraction. Undefined when num_intersections is 0 (Jolt leaves outMinFraction untouched).
    min_fraction: f32,
    /// Highest intersection fraction. Undefined when num_intersections is 0 (Jolt leaves outMaxFraction untouched).
    max_fraction: f32,
};

/// Tests a ray starting at ray_origin and extending infinitely in ray_direction against a sphere.
/// Outputs entry and exit points (min_fraction and max_fraction) along the ray (which could be negative if the hit point is before the start of the ray).
/// @param ray_origin Ray origin. If the ray starts inside the sphere, the returned fraction will be 0.
/// @param ray_direction Ray direction. Does not need to be normalized.
/// @param sphere_center Position of the center of the sphere.
/// @param sphere_radius Radius of the sphere.
/// @return The amount of intersections with the sphere (num_intersections), the lowest (min_fraction) and highest (max_fraction) intersection fraction.
/// If 1 intersection is returned min_fraction will be equal to max_fraction
/// (the RaySphere overload with outMinFraction / outMaxFraction)
pub fn raySphereMinMax(ray_origin: Vec3, ray_direction: Vec3, sphere_center: Vec3, sphere_radius: f32) RaySphereMinMax {
    // Solve: |RayOrigin + fraction * RayDirection - SphereCenter|^2 = SphereRadius^2 for fraction
    const center_origin = ray_origin.sub(sphere_center);
    const a = ray_direction.lengthSq();
    const b = 2.0 * ray_direction.dot(center_origin);
    const c = center_origin.lengthSq() - sphere_radius * sphere_radius;
    const roots = findRoot(f32, a, b, c);
    switch (roots.num_roots) {
        0 => {
            if (c <= 0.0) {
                // Origin inside sphere
                return .{ .num_intersections = 1, .min_fraction = 0.0, .max_fraction = 0.0 };
            } else {
                // Origin outside of the sphere
                return .{ .num_intersections = 0, .min_fraction = undefined, .max_fraction = undefined };
            }
        },

        // Ray is touching the sphere
        1 => return .{ .num_intersections = 1, .min_fraction = roots.x1, .max_fraction = roots.x1 },

        else => {
            // Ray enters and exits the sphere

            // Sort so that the smallest is first
            var fraction1 = roots.x1;
            var fraction2 = roots.x2;
            if (fraction1 > fraction2)
                std.mem.swap(f32, &fraction1, &fraction2);

            return .{ .num_intersections = 2, .min_fraction = fraction1, .max_fraction = fraction2 };
        },
    }
}

test "raySphere" {
    const center = Vec3.init(0, 0, 5);

    // Hit from the outside
    try std.testing.expectEqual(@as(f32, 4), raySphere(Vec3.zero(), Vec3.init(0, 0, 1), center, 1));
    var r = raySphereMinMax(Vec3.zero(), Vec3.init(0, 0, 1), center, 1);
    try std.testing.expectEqual(@as(i32, 2), r.num_intersections);
    try std.testing.expectEqual(@as(f32, 4), r.min_fraction);
    try std.testing.expectEqual(@as(f32, 6), r.max_fraction);

    // Start inside
    try std.testing.expectEqual(@as(f32, 0), raySphere(center, Vec3.init(0, 0, 1), center, 1));

    // Sphere behind the ray
    try std.testing.expectEqual(math.flt_max, raySphere(Vec3.zero(), Vec3.init(0, 0, -1), center, 1));
    r = raySphereMinMax(Vec3.zero(), Vec3.init(0, 0, -1), center, 1);
    try std.testing.expectEqual(@as(i32, 2), r.num_intersections);
    try std.testing.expectEqual(@as(f32, -6), r.min_fraction);
    try std.testing.expectEqual(@as(f32, -4), r.max_fraction);

    // Miss
    try std.testing.expectEqual(math.flt_max, raySphere(Vec3.init(2, 0, 0), Vec3.init(0, 0, 1), center, 1));
    r = raySphereMinMax(Vec3.init(2, 0, 0), Vec3.init(0, 0, 1), center, 1);
    try std.testing.expectEqual(@as(i32, 0), r.num_intersections);

    // Zero direction, origin inside and outside
    try std.testing.expectEqual(@as(f32, 0), raySphere(center, Vec3.zero(), center, 1));
    try std.testing.expectEqual(math.flt_max, raySphere(Vec3.zero(), Vec3.zero(), center, 1));
    r = raySphereMinMax(center, Vec3.zero(), center, 1);
    try std.testing.expectEqual(@as(i32, 1), r.num_intersections);
    try std.testing.expectEqual(@as(f32, 0), r.min_fraction);
    try std.testing.expectEqual(@as(f32, 0), r.max_fraction);

    // Touching (q == 0)
    r = raySphereMinMax(Vec3.init(0, 0, 4), Vec3.init(1, 0, 0), center, 1);
    try std.testing.expectEqual(@as(i32, 1), r.num_intersections);
    try std.testing.expectEqual(@as(f32, 0), r.min_fraction);
}
