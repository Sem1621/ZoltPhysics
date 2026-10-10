//! Port of: Jolt/Geometry/RayAABox.h
//! Status: complete
//!
//! The overloads get distinct names: `RayAABox(..., outMin, outMax)` is `rayAABoxMinMax` (returns
//! `RayAABoxMinMax{ .min, .max }`) and the `RayAABoxHits` overload that takes the ray direction instead of a
//! `RayInvDirection` is `rayAABoxHitsDirection`.

const std = @import("std");
const math = @import("../Math/Math.zig");
const UVec4 = @import("../Math/UVec4.zig").UVec4;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;

/// Helper structure holding the reciprocal of a ray for Ray vs AABox testing.
/// Jolt's default constructor leaves the members uninitialized: `var r: RayInvDirection = undefined;`
pub const RayInvDirection = struct {
    /// 1 / ray direction
    inv_direction: Vec3,
    /// for each component if it is parallel to the coordinate axis
    is_parallel: UVec4,

    /// Constructor (explicit RayInvDirection(Vec3Arg inDirection))
    pub fn init(direction: Vec3) RayInvDirection {
        var result: RayInvDirection = undefined;
        result.set(direction);
        return result;
    }

    /// Set reciprocal from ray direction
    pub fn set(self: *RayInvDirection, direction: Vec3) void {
        // if (abs(direction) <= Epsilon) the ray is nearly parallel to the slab.
        self.is_parallel = Vec3.lessOrEqual(direction.abs(), Vec3.replicate(1.0e-20));

        // Calculate 1 / direction while avoiding division by zero
        self.inv_direction = Vec3.select(direction, Vec3.one(), self.is_parallel).reciprocal();
    }
};

/// Intersect AABB with ray, returns minimal distance along ray or FLT_MAX if no hit
/// Note: Can return negative value if ray starts in box
pub fn rayAABox(origin: Vec3, inv_direction: RayInvDirection, bounds_min: Vec3, bounds_max: Vec3) f32 {
    // Constants
    const flt_min = Vec3.replicate(-math.flt_max);
    const flt_max = Vec3.replicate(math.flt_max);

    // Test against all three axes simultaneously.
    const t1 = bounds_min.sub(origin).mul(inv_direction.inv_direction);
    const t2 = bounds_max.sub(origin).mul(inv_direction.inv_direction);

    // Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
    // use the results from any directions parallel to the slab.
    var t_min = Vec3.select(Vec3.min(t1, t2), flt_min, inv_direction.is_parallel);
    var t_max = Vec3.select(Vec3.max(t1, t2), flt_max, inv_direction.is_parallel);

    // t_min.xyz = maximum(t_min.x, t_min.y, t_min.z);
    t_min = Vec3.max(t_min, t_min.swizzle(.y, .z, .x));
    t_min = Vec3.max(t_min, t_min.swizzle(.z, .x, .y));

    // t_max.xyz = minimum(t_max.x, t_max.y, t_max.z);
    t_max = Vec3.min(t_max, t_max.swizzle(.y, .z, .x));
    t_max = Vec3.min(t_max, t_max.swizzle(.z, .x, .y));

    // if (t_min > t_max) return FLT_MAX;
    var no_intersection = Vec3.greater(t_min, t_max);

    // if (t_max < 0.0f) return FLT_MAX;
    no_intersection = UVec4.bitOr(no_intersection, Vec3.less(t_max, Vec3.zero()));

    // if (inv_direction.is_parallel && !(Min <= origin && origin <= Max)) return FLT_MAX; else return t_min;
    const no_parallel_overlap = UVec4.bitOr(Vec3.less(origin, bounds_min), Vec3.greater(origin, bounds_max));
    no_intersection = UVec4.bitOr(no_intersection, UVec4.bitAnd(inv_direction.is_parallel, no_parallel_overlap));
    no_intersection = UVec4.bitOr(no_intersection, no_intersection.splatY());
    no_intersection = UVec4.bitOr(no_intersection, no_intersection.splatZ());
    return Vec3.select(t_min, flt_max, no_intersection).getX();
}

/// Intersect 4 AABBs with ray, returns minimal distance along ray or FLT_MAX if no hit
/// Note: Can return negative value if ray starts in box
pub fn rayAABox4(origin: Vec3, inv_direction: RayInvDirection, bounds_min_x: Vec4, bounds_min_y: Vec4, bounds_min_z: Vec4, bounds_max_x: Vec4, bounds_max_y: Vec4, bounds_max_z: Vec4) Vec4 {
    // Constants
    const flt_min = Vec4.replicate(-math.flt_max);
    const flt_max = Vec4.replicate(math.flt_max);

    // Origin
    const originx = origin.splatX();
    const originy = origin.splatY();
    const originz = origin.splatZ();

    // Parallel
    const parallelx = inv_direction.is_parallel.splatX();
    const parallely = inv_direction.is_parallel.splatY();
    const parallelz = inv_direction.is_parallel.splatZ();

    // Inverse direction
    const invdirx = inv_direction.inv_direction.splatX();
    const invdiry = inv_direction.inv_direction.splatY();
    const invdirz = inv_direction.inv_direction.splatZ();

    // Test against all three axes simultaneously.
    const t1x = bounds_min_x.sub(originx).mul(invdirx);
    const t1y = bounds_min_y.sub(originy).mul(invdiry);
    const t1z = bounds_min_z.sub(originz).mul(invdirz);
    const t2x = bounds_max_x.sub(originx).mul(invdirx);
    const t2y = bounds_max_y.sub(originy).mul(invdiry);
    const t2z = bounds_max_z.sub(originz).mul(invdirz);

    // Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
    // use the results from any directions parallel to the slab.
    const t_minx = Vec4.select(Vec4.min(t1x, t2x), flt_min, parallelx);
    const t_miny = Vec4.select(Vec4.min(t1y, t2y), flt_min, parallely);
    const t_minz = Vec4.select(Vec4.min(t1z, t2z), flt_min, parallelz);
    const t_maxx = Vec4.select(Vec4.max(t1x, t2x), flt_max, parallelx);
    const t_maxy = Vec4.select(Vec4.max(t1y, t2y), flt_max, parallely);
    const t_maxz = Vec4.select(Vec4.max(t1z, t2z), flt_max, parallelz);

    // t_min.xyz = maximum(t_min.x, t_min.y, t_min.z);
    const t_min = Vec4.max(Vec4.max(t_minx, t_miny), t_minz);

    // t_max.xyz = minimum(t_max.x, t_max.y, t_max.z);
    const t_max = Vec4.min(Vec4.min(t_maxx, t_maxy), t_maxz);

    // if (t_min > t_max) return FLT_MAX;
    var no_intersection = Vec4.greater(t_min, t_max);

    // if (t_max < 0.0f) return FLT_MAX;
    no_intersection = UVec4.bitOr(no_intersection, Vec4.less(t_max, Vec4.zero()));

    // if bounds are invalid return FLOAT_MAX;
    const bounds_invalid = UVec4.bitOr(UVec4.bitOr(Vec4.greater(bounds_min_x, bounds_max_x), Vec4.greater(bounds_min_y, bounds_max_y)), Vec4.greater(bounds_min_z, bounds_max_z));
    no_intersection = UVec4.bitOr(no_intersection, bounds_invalid);

    // if (inv_direction.is_parallel && !(Min <= origin && origin <= Max)) return FLT_MAX; else return t_min;
    const no_parallel_overlapx = UVec4.bitAnd(parallelx, UVec4.bitOr(Vec4.less(originx, bounds_min_x), Vec4.greater(originx, bounds_max_x)));
    const no_parallel_overlapy = UVec4.bitAnd(parallely, UVec4.bitOr(Vec4.less(originy, bounds_min_y), Vec4.greater(originy, bounds_max_y)));
    const no_parallel_overlapz = UVec4.bitAnd(parallelz, UVec4.bitOr(Vec4.less(originz, bounds_min_z), Vec4.greater(originz, bounds_max_z)));
    no_intersection = UVec4.bitOr(no_intersection, UVec4.bitOr(UVec4.bitOr(no_parallel_overlapx, no_parallel_overlapy), no_parallel_overlapz));
    return Vec4.select(t_min, flt_max, no_intersection);
}

/// Result of `rayAABoxMinMax` (the outMin / outMax out parameters of the RayAABox overload)
pub const RayAABoxMinMax = struct {
    /// Minimal distance along the ray, FLT_MAX if no hit
    min: f32,
    /// Maximal distance along the ray, -FLT_MAX if no hit
    max: f32,
};

/// Intersect AABB with ray, returns minimal and maximal distance along ray or FLT_MAX, -FLT_MAX if no hit
/// Note: Can return negative value for min if ray starts in box
/// (the RayAABox overload with outMin / outMax)
pub fn rayAABoxMinMax(origin: Vec3, inv_direction: RayInvDirection, bounds_min: Vec3, bounds_max: Vec3) RayAABoxMinMax {
    // Constants
    const flt_min = Vec3.replicate(-math.flt_max);
    const flt_max = Vec3.replicate(math.flt_max);

    // Test against all three axes simultaneously.
    const t1 = bounds_min.sub(origin).mul(inv_direction.inv_direction);
    const t2 = bounds_max.sub(origin).mul(inv_direction.inv_direction);

    // Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
    // use the results from any directions parallel to the slab.
    var t_min = Vec3.select(Vec3.min(t1, t2), flt_min, inv_direction.is_parallel);
    var t_max = Vec3.select(Vec3.max(t1, t2), flt_max, inv_direction.is_parallel);

    // t_min.xyz = maximum(t_min.x, t_min.y, t_min.z);
    t_min = Vec3.max(t_min, t_min.swizzle(.y, .z, .x));
    t_min = Vec3.max(t_min, t_min.swizzle(.z, .x, .y));

    // t_max.xyz = minimum(t_max.x, t_max.y, t_max.z);
    t_max = Vec3.min(t_max, t_max.swizzle(.y, .z, .x));
    t_max = Vec3.min(t_max, t_max.swizzle(.z, .x, .y));

    // if (t_min > t_max) return FLT_MAX;
    var no_intersection = Vec3.greater(t_min, t_max);

    // if (t_max < 0.0f) return FLT_MAX;
    no_intersection = UVec4.bitOr(no_intersection, Vec3.less(t_max, Vec3.zero()));

    // if (inv_direction.is_parallel && !(Min <= origin && origin <= Max)) return FLT_MAX; else return t_min;
    const no_parallel_overlap = UVec4.bitOr(Vec3.less(origin, bounds_min), Vec3.greater(origin, bounds_max));
    no_intersection = UVec4.bitOr(no_intersection, UVec4.bitAnd(inv_direction.is_parallel, no_parallel_overlap));
    no_intersection = UVec4.bitOr(no_intersection, no_intersection.splatY());
    no_intersection = UVec4.bitOr(no_intersection, no_intersection.splatZ());
    return .{
        .min = Vec3.select(t_min, flt_max, no_intersection).getX(),
        .max = Vec3.select(t_max, flt_min, no_intersection).getX(),
    };
}

/// Intersect AABB with ray, returns true if there is a hit closer than closest
pub fn rayAABoxHits(origin: Vec3, inv_direction: RayInvDirection, bounds_min: Vec3, bounds_max: Vec3, closest: f32) bool {
    // Constants
    const flt_min = Vec3.replicate(-math.flt_max);
    const flt_max = Vec3.replicate(math.flt_max);

    // Test against all three axes simultaneously.
    const t1 = bounds_min.sub(origin).mul(inv_direction.inv_direction);
    const t2 = bounds_max.sub(origin).mul(inv_direction.inv_direction);

    // Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
    // use the results from any directions parallel to the slab.
    var t_min = Vec3.select(Vec3.min(t1, t2), flt_min, inv_direction.is_parallel);
    var t_max = Vec3.select(Vec3.max(t1, t2), flt_max, inv_direction.is_parallel);

    // t_min.xyz = maximum(t_min.x, t_min.y, t_min.z);
    t_min = Vec3.max(t_min, t_min.swizzle(.y, .z, .x));
    t_min = Vec3.max(t_min, t_min.swizzle(.z, .x, .y));

    // t_max.xyz = minimum(t_max.x, t_max.y, t_max.z);
    t_max = Vec3.min(t_max, t_max.swizzle(.y, .z, .x));
    t_max = Vec3.min(t_max, t_max.swizzle(.z, .x, .y));

    // if (t_min > t_max) return false;
    var no_intersection = Vec3.greater(t_min, t_max);

    // if (t_max < 0.0f) return false;
    no_intersection = UVec4.bitOr(no_intersection, Vec3.less(t_max, Vec3.zero()));

    // if (t_min > closest) return false;
    no_intersection = UVec4.bitOr(no_intersection, Vec3.greater(t_min, Vec3.replicate(closest)));

    // if (inv_direction.is_parallel && !(Min <= origin && origin <= Max)) return false; else return true;
    const no_parallel_overlap = UVec4.bitOr(Vec3.less(origin, bounds_min), Vec3.greater(origin, bounds_max));
    no_intersection = UVec4.bitOr(no_intersection, UVec4.bitAnd(inv_direction.is_parallel, no_parallel_overlap));

    return !no_intersection.testAnyXYZTrue();
}

/// Intersect AABB with ray without hit fraction, based on separating axis test
/// @see http://www.codercorner.com/RayAABB.cpp
/// (the RayAABoxHits overload that takes the ray direction)
pub fn rayAABoxHitsDirection(origin: Vec3, direction: Vec3, bounds_min: Vec3, bounds_max: Vec3) bool {
    const extents = bounds_max.sub(bounds_min);

    const diff = origin.mulScalar(2.0).sub(bounds_min).sub(bounds_max);
    const abs_diff = diff.abs();

    var no_intersection = UVec4.bitAnd(Vec3.greater(abs_diff, extents), Vec3.greaterOrEqual(diff.mul(direction), Vec3.zero()));

    const abs_dir = direction.abs();
    const abs_dir_yzz = abs_dir.swizzle(.y, .z, .z);
    const abs_dir_xyx = abs_dir.swizzle(.x, .y, .x);

    const extents_yzz = extents.swizzle(.y, .z, .z);
    const extents_xyx = extents.swizzle(.x, .y, .x);

    const diff_yzx = diff.swizzle(.y, .z, .x);

    const dir_yzx = direction.swizzle(.y, .z, .x);

    no_intersection = UVec4.bitOr(no_intersection, Vec3.greater(direction.mul(diff_yzx).sub(dir_yzx.mul(diff)).abs(), extents_xyx.mul(abs_dir_yzz).add(extents_yzz.mul(abs_dir_xyx))));

    return !no_intersection.testAnyXYZTrue();
}

test "rayAABox" {
    const bounds_min = Vec3.init(-1, -1, -1);
    const bounds_max = Vec3.init(1, 1, 1);

    // Ray along the x axis hitting the box
    const inv = RayInvDirection.init(Vec3.init(1, 0, 0));
    try std.testing.expect(inv.is_parallel.getX() == 0 and inv.is_parallel.getY() != 0 and inv.is_parallel.getZ() != 0);
    try std.testing.expectEqual(@as(f32, 1), rayAABox(Vec3.init(-2, 0, 0), inv, bounds_min, bounds_max));
    const min_max = rayAABoxMinMax(Vec3.init(-2, 0, 0), inv, bounds_min, bounds_max);
    try std.testing.expectEqual(@as(f32, 1), min_max.min);
    try std.testing.expectEqual(@as(f32, 3), min_max.max);
    try std.testing.expect(rayAABoxHits(Vec3.init(-2, 0, 0), inv, bounds_min, bounds_max, 2));
    try std.testing.expect(!rayAABoxHits(Vec3.init(-2, 0, 0), inv, bounds_min, bounds_max, 0.5));
    try std.testing.expect(rayAABoxHitsDirection(Vec3.init(-2, 0, 0), Vec3.init(1, 0, 0), bounds_min, bounds_max));

    // Ray starting inside the box returns a negative fraction
    try std.testing.expectEqual(@as(f32, -1), rayAABox(Vec3.zero(), inv, bounds_min, bounds_max));

    // Parallel ray missing the box
    try std.testing.expectEqual(math.flt_max, rayAABox(Vec3.init(-2, 2, 0), inv, bounds_min, bounds_max));
    const miss = rayAABoxMinMax(Vec3.init(-2, 2, 0), inv, bounds_min, bounds_max);
    try std.testing.expectEqual(math.flt_max, miss.min);
    try std.testing.expectEqual(-math.flt_max, miss.max);
    try std.testing.expect(!rayAABoxHits(Vec3.init(-2, 2, 0), inv, bounds_min, bounds_max, math.flt_max));
    try std.testing.expect(!rayAABoxHitsDirection(Vec3.init(-2, 2, 0), Vec3.init(1, 0, 0), bounds_min, bounds_max));

    // Box behind the ray
    try std.testing.expectEqual(math.flt_max, rayAABox(Vec3.init(2, 0, 0), inv, bounds_min, bounds_max));
    try std.testing.expect(!rayAABoxHitsDirection(Vec3.init(2, 0, 0), Vec3.init(1, 0, 0), bounds_min, bounds_max));
}

test "rayAABox4" {
    const inv = RayInvDirection.init(Vec3.init(0, 0, 2));
    const origin = Vec3.init(0, 0, -5);
    // Box 0: hit at z = -1 (fraction 2), box 1: missed, box 2: invalid bounds, box 3: origin inside
    const result = rayAABox4(origin, inv, Vec4.init(-1, 2, 1, -1), Vec4.init(-1, 2, 1, -1), Vec4.init(-1, -1, 1, -6), Vec4.init(1, 3, -1, 1), Vec4.init(1, 3, -1, 1), Vec4.init(1, 1, -1, 1));
    try std.testing.expectEqual(@as(f32, 2), result.getX());
    try std.testing.expectEqual(math.flt_max, result.getY());
    try std.testing.expectEqual(math.flt_max, result.getZ());
    try std.testing.expectEqual(@as(f32, -0.5), result.getW());
}
