//! Port of: Jolt/Geometry/ClosestPoint.h
//! Status: complete
//!
//! Helper utils to find the closest point to a line segment, triangle or tetrahedron.
//! This file is Jolt's `ClosestPoint` namespace (re-exported as `zolt.ClosestPoint`).
//!
//! The out parameters of Jolt's functions are returned as structs: `getBaryCentricCoordinates` returns
//! `BaryCentricLine{ .u, .v, .valid }`, `getBaryCentricCoordinatesTriangle` returns
//! `BaryCentricTriangle{ .u, .v, .w, .valid }` and the `getClosestPointOn*` functions return
//! `PointAndSet{ .point, .set }`. The `MustIncludeC` / `MustIncludeD` template parameters are comptime
//! options: `getClosestPointOnTriangle(a, b, c, .{ .must_include_c = true })`.

const std = @import("std");
const math = @import("../Math/Math.zig");
const UVec4 = @import("../Math/UVec4.zig").UVec4;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;

/// Barycentric coordinates of the closest point on a line (the outU / outV out parameters and the return value of
/// the 2 point GetBaryCentricCoordinates). The point is a * u + b * v.
pub const BaryCentricLine = struct {
    u: f32,
    v: f32,
    /// False if the points a, b do not form a line (are at the same point)
    valid: bool,
};

/// Barycentric coordinates of the closest point on a plane (the outU / outV / outW out parameters and the return
/// value of the 3 point GetBaryCentricCoordinates). The point is a * u + b * v + c * w.
pub const BaryCentricTriangle = struct {
    u: f32,
    v: f32,
    w: f32,
    /// False if the points a, b, c do not form a plane (are on the same line or at the same point)
    valid: bool,
};

/// Closest point and the feature it lies on (the return value and the outSet out parameter of the
/// GetClosestPointOn* functions)
pub const PointAndSet = struct {
    /// The closest point to the origin
    point: Vec3,
    /// Bit mask of the vertices that form the closest feature: 1 = a, 2 = b, 4 = c, 8 = d
    set: u32,
};

/// Compute barycentric coordinates of closest point to origin for infinite line defined by (a, b)
/// Point can then be computed as a * u + b * v
/// Returns valid = false if the points a, b do not form a line (are at the same point)
pub fn getBaryCentricCoordinates(a: Vec3, b: Vec3) BaryCentricLine {
    const ab = b.sub(a);
    const denominator = ab.lengthSq();
    if (denominator < math.square(math.flt_epsilon)) {
        // Degenerate line segment, fallback to points
        if (a.lengthSq() < b.lengthSq()) {
            // A closest
            return .{ .u = 1.0, .v = 0.0, .valid = false };
        } else {
            // B closest
            return .{ .u = 0.0, .v = 1.0, .valid = false };
        }
    } else {
        const v = -a.dot(ab) / denominator;
        return .{ .u = 1.0 - v, .v = v, .valid = true };
    }
}

/// Compute barycentric coordinates of closest point to origin for plane defined by (a, b, c)
/// Point can then be computed as a * u + b * v + c * w
/// Returns valid = false if the points a, b, c do not form a plane (are on the same line or at the same point)
/// (the 3 point overload of GetBaryCentricCoordinates)
pub fn getBaryCentricCoordinatesTriangle(a: Vec3, b: Vec3, c: Vec3) BaryCentricTriangle {
    // Taken from: Real-Time Collision Detection - Christer Ericson (Section: Barycentric Coordinates)
    // With p = 0
    // Adjusted to always include the shortest edge of the triangle in the calculation to improve numerical accuracy

    // First calculate the three edges
    const v0 = b.sub(a);
    const v1 = c.sub(a);
    const v2 = c.sub(b);

    // Make sure that the shortest edge is included in the calculation to keep the products a * b - c * d as small as possible to preserve accuracy
    const d00 = v0.lengthSq();
    const d11 = v1.lengthSq();
    const d22 = v2.lengthSq();
    if (d00 <= d22) {
        // Use v0 and v1 to calculate barycentric coordinates
        const d01 = v0.dot(v1);

        // Denominator must be positive:
        // |v0|^2 * |v1|^2 - (v0 . v1)^2 = |v0|^2 * |v1|^2 * (1 - cos(angle)^2) >= 0
        const denominator = math.differenceOfProducts(d00, d11, d01, d01);
        // (1 - cos(angle)^2) == 1.0e-6 or angle ~ 0.06 degrees which means the triangle is degenerate and we fall back to calculating the closest point on the longest edge
        if (denominator <= 1.0e-6 * d00 * d11) {
            // Degenerate triangle, return coordinates along longest edge
            if (d00 > d11) {
                const line = getBaryCentricCoordinates(a, b);
                return .{ .u = line.u, .v = line.v, .w = 0.0, .valid = false };
            } else {
                const line = getBaryCentricCoordinates(a, c);
                return .{ .u = line.u, .v = 0.0, .w = line.v, .valid = false };
            }
        } else {
            const a0 = a.dot(v0);
            const a1 = a.dot(v1);
            const v = math.differenceOfProducts(d01, a1, d11, a0) / denominator;
            const w = math.differenceOfProducts(d01, a0, d00, a1) / denominator;
            const u = 1.0 - v - w;
            return .{ .u = u, .v = v, .w = w, .valid = true };
        }
    } else {
        // Use v1 and v2 to calculate barycentric coordinates
        const d12 = v1.dot(v2);

        const denominator = math.differenceOfProducts(d11, d22, d12, d12);
        if (denominator <= 1.0e-6 * d11 * d22) {
            // Degenerate triangle, return coordinates along longest edge
            if (d11 > d22) {
                const line = getBaryCentricCoordinates(a, c);
                return .{ .u = line.u, .v = 0.0, .w = line.v, .valid = false };
            } else {
                const line = getBaryCentricCoordinates(b, c);
                return .{ .u = 0.0, .v = line.u, .w = line.v, .valid = false };
            }
        } else {
            const c1 = c.dot(v1);
            const c2 = c.dot(v2);
            const u = math.differenceOfProducts(d22, c1, d12, c2) / denominator;
            const v = math.differenceOfProducts(d11, c2, d12, c1) / denominator;
            const w = 1.0 - u - v;
            return .{ .u = u, .v = v, .w = w, .valid = true };
        }
    }
}

/// Get the closest point to the origin of line (a, b)
/// set describes which features are closest: 1 = a, 2 = b, 3 = line segment ab
pub fn getClosestPointOnLine(a: Vec3, b: Vec3) PointAndSet {
    const bary = getBaryCentricCoordinates(a, b);
    if (bary.v <= 0.0) {
        // a is closest point
        return .{ .point = a, .set = 0b0001 };
    } else if (bary.u <= 0.0) {
        // b is closest point
        return .{ .point = b, .set = 0b0010 };
    } else {
        // Closest point lies on line a b
        return .{ .point = a.mulScalar(bary.u).add(b.mulScalar(bary.v)), .set = 0b0011 };
    }
}

/// Get the closest point to the origin of triangle (a, b, c)
/// set describes which features are closest: 1 = a, 2 = b, 4 = c, 5 = line segment ac, 7 = triangle interior etc.
/// If must_include_c is true, the function assumes that C is part of the closest feature (vertex, edge, face) and does less work, if the assumption is not true then a closest point to the other features is returned.
pub fn getClosestPointOnTriangle(a: Vec3, b: Vec3, c: Vec3, comptime opts: struct { must_include_c: bool = false }) PointAndSet {
    // Taken from: Real-Time Collision Detection - Christer Ericson (Section: Closest Point on Triangle to Point)
    // With p = 0

    // The most accurate normal is calculated by using the two shortest edges
    // See: https://box2d.org/posts/2014/01/troublesome-triangle/
    // The difference in normals is most pronounced when one edge is much smaller than the others (in which case the other 2 must have roughly the same length).
    // Therefore we can suffice by just picking the shortest from 2 edges and use that with the 3rd edge to calculate the normal.
    // In this case we ensure that ab is shorter than bc by swapping a and c if it is not.
    var swap_ac: UVec4 = undefined;
    {
        const ba = a.sub(b);
        const bc = c.sub(b);
        swap_ac = Vec4.less(bc.dotV4(bc), ba.dotV4(ba));
    }
    // Jolt's locals `a` and `c` (a and c, swapped when swap_ac is set) are `sa` and `sc` in Zolt (Zig forbids shadowing the parameters)
    const sa = Vec3.select(a, c, swap_ac);
    const sc = Vec3.select(c, a, swap_ac);

    // Calculate normal
    var ab = b.sub(sa);
    const ac = sc.sub(sa);
    const n = ab.crossPrecise(ac);
    const n_len_sq = n.lengthSq();

    // Check degenerate
    if (n_len_sq < 1.0e-10) // Square(FLT_EPSILON) was too small and caused numerical problems, see test case TestCollideParallelTriangleVsCapsule
    {
        // Degenerate, fallback to vertices and edges

        // Start with vertex C being the closest
        var closest_set: u32 = 0b0100;
        var closest_point = c;
        var best_dist_sq = c.lengthSq();

        // If the closest point must include C then A or B cannot be closest
        // Note that we test vertices first because we want to prefer a closest vertex over a closest edge (this results in an outSet with fewer bits set)
        if (!opts.must_include_c) {
            // Try vertex A
            const a_len_sq = a.lengthSq();
            if (a_len_sq < best_dist_sq) {
                closest_set = 0b0001;
                closest_point = a;
                best_dist_sq = a_len_sq;
            }

            // Try vertex B
            const b_len_sq = b.lengthSq();
            if (b_len_sq < best_dist_sq) {
                closest_set = 0b0010;
                closest_point = b;
                best_dist_sq = b_len_sq;
            }
        }

        // Edge AC
        const ac_len_sq = ac.lengthSq();
        if (ac_len_sq > math.square(math.flt_epsilon)) {
            const v = math.clamp(-sa.dot(ac) / ac_len_sq, 0.0, 1.0);
            const q = sa.add(ac.mulScalar(v));
            const dist_sq = q.lengthSq();
            if (dist_sq < best_dist_sq) {
                closest_set = 0b0101;
                closest_point = q;
                best_dist_sq = dist_sq;
            }
        }

        // Edge BC
        const bc = c.sub(b);
        const bc_len_sq = bc.lengthSq();
        if (bc_len_sq > math.square(math.flt_epsilon)) {
            const v = math.clamp(-b.dot(bc) / bc_len_sq, 0.0, 1.0);
            const q = b.add(bc.mulScalar(v));
            const dist_sq = q.lengthSq();
            if (dist_sq < best_dist_sq) {
                closest_set = 0b0110;
                closest_point = q;
                best_dist_sq = dist_sq;
            }
        }

        // If the closest point must include C then AB cannot be closest
        if (!opts.must_include_c) {
            // Edge AB
            ab = b.sub(a);
            const ab_len_sq = ab.lengthSq();
            if (ab_len_sq > math.square(math.flt_epsilon)) {
                const v = math.clamp(-a.dot(ab) / ab_len_sq, 0.0, 1.0);
                const q = a.add(ab.mulScalar(v));
                const dist_sq = q.lengthSq();
                if (dist_sq < best_dist_sq) {
                    closest_set = 0b0011;
                    closest_point = q;
                    best_dist_sq = dist_sq;
                }
            }
        }

        return .{ .point = closest_point, .set = closest_set };
    }

    const swapped = swap_ac.getX() != 0;

    // Check if P in vertex region outside A
    const ap = sa.negate();
    const d1 = ab.dot(ap);
    const d2 = ac.dot(ap);
    if (d1 <= 0.0 and d2 <= 0.0)
        return .{ .point = sa, .set = if (swapped) 0b0100 else 0b0001 }; // barycentric coordinates (1,0,0)

    // Check if P in vertex region outside B
    const bp = b.negate();
    const d3 = ab.dot(bp);
    const d4 = ac.dot(bp);
    if (d3 >= 0.0 and d4 <= d3)
        return .{ .point = b, .set = 0b0010 }; // barycentric coordinates (0,1,0)

    // Check if P in edge region of AB, if so return projection of P onto AB
    if (d1 * d4 <= d3 * d2 and d1 >= 0.0 and d3 <= 0.0) {
        const v = d1 / (d1 - d3);
        return .{ .point = sa.add(ab.mulScalar(v)), .set = if (swapped) 0b0110 else 0b0011 }; // barycentric coordinates (1-v,v,0)
    }

    // Check if P in vertex region outside C
    const cp = sc.negate();
    const d5 = ab.dot(cp);
    const d6 = ac.dot(cp);
    if (d6 >= 0.0 and d5 <= d6)
        return .{ .point = sc, .set = if (swapped) 0b0001 else 0b0100 }; // barycentric coordinates (0,0,1)

    // Check if P in edge region of AC, if so return projection of P onto AC
    if (d5 * d2 <= d1 * d6 and d2 >= 0.0 and d6 <= 0.0) {
        const w = d2 / (d2 - d6);
        return .{ .point = sa.add(ac.mulScalar(w)), .set = 0b0101 }; // barycentric coordinates (1-w,0,w)
    }

    // Check if P in edge region of BC, if so return projection of P onto BC
    const d4_d3 = d4 - d3;
    const d5_d6 = d5 - d6;
    if (d3 * d6 <= d5 * d4 and d4_d3 >= 0.0 and d5_d6 >= 0.0) {
        const w = d4_d3 / (d4_d3 + d5_d6);
        return .{ .point = b.add(sc.sub(b).mulScalar(w)), .set = if (swapped) 0b0011 else 0b0110 }; // barycentric coordinates (0,1-w,w)
    }

    // P inside face region.
    // Here we deviate from Christer Ericson's article to improve accuracy.
    // Determine distance between triangle and origin: distance = (centroid - origin) . normal / |normal|
    // Closest point to origin is then: distance . normal / |normal|
    // Note that this way of calculating the closest point is much more accurate than first calculating barycentric coordinates
    // and then calculating the closest point based on those coordinates.
    return .{ .point = n.mulScalar(sa.add(b).add(sc).dot(n)).divScalar(3.0 * n_len_sq), .set = 0b0111 };
}

/// Check if the origin is outside the plane of triangle (a, b, c). d specifies the front side of the plane.
pub fn originOutsideOfPlane(a: Vec3, b: Vec3, c: Vec3, d: Vec3) bool {
    // Taken from: Real-Time Collision Detection - Christer Ericson (Section: Closest Point on Tetrahedron to Point)
    // With p = 0

    // Test if point p and d lie on opposite sides of plane through abc
    const n = b.sub(a).crossPrecise(c.sub(a));
    const signp = a.dot(n); // [AP AB AC]
    const signd = d.sub(a).dot(n); // [AD AB AC]

    // Points on opposite sides if expression signs are the same
    // Note that we left out the minus sign in signp so we need to check > 0 instead of < 0 as in Christer's book
    // We compare against a small negative value to allow for a little bit of slop in the calculations
    return signp * signd > -math.flt_epsilon;
}

/// Returns for each of the planes of the tetrahedron if the origin is inside it
/// Roughly equivalent to:
/// [originOutsideOfPlane(a, b, c, d),
///  originOutsideOfPlane(a, c, d, b),
///  originOutsideOfPlane(a, d, b, c),
///  originOutsideOfPlane(b, d, c, a)]
pub fn originOutsideOfTetrahedronPlanes(a: Vec3, b: Vec3, c: Vec3, d: Vec3) UVec4 {
    const ab = b.sub(a);
    const ac = c.sub(a);
    const ad = d.sub(a);
    const bd = d.sub(b);
    const bc = c.sub(b);

    const ab_cross_ac = ab.crossPrecise(ac);
    const ac_cross_ad = ac.crossPrecise(ad);
    const ad_cross_ab = ad.crossPrecise(ab);
    const bd_cross_bc = bd.crossPrecise(bc);

    // For each plane get the side on which the origin is
    const signp0 = a.dot(ab_cross_ac); // ABC
    const signp1 = a.dot(ac_cross_ad); // ACD
    const signp2 = a.dot(ad_cross_ab); // ADB
    const signp3 = b.dot(bd_cross_bc); // BDC
    const signp = Vec4.init(signp0, signp1, signp2, signp3);

    // For each plane get the side that is outside (determined by the 4th point)
    const signd0 = ad.dot(ab_cross_ac); // D
    const signd1 = ab.dot(ac_cross_ad); // B
    const signd2 = ac.dot(ad_cross_ab); // C
    const signd3 = -ab.dot(bd_cross_bc); // A
    const signd = Vec4.init(signd0, signd1, signd2, signd3);

    // The winding of all triangles has been chosen so that signd should have the
    // same sign for all components. If this is not the case the tetrahedron
    // is degenerate and we return that the origin is in front of all sides
    const sign_bits = signd.getSignBits();
    switch (sign_bits) {
        // All positive
        0 => return Vec4.greaterOrEqual(signp, Vec4.replicate(-math.flt_epsilon)),

        // All negative
        0xf => return Vec4.lessOrEqual(signp, Vec4.replicate(math.flt_epsilon)),

        // Mixed signs, degenerate tetrahedron
        else => return UVec4.replicate(0xffffffff),
    }
}

/// Get the closest point between tetrahedron (a, b, c, d) to the origin
/// set specifies which feature was closest, 1 = a, 2 = b, 4 = c, 8 = d. Edges have 2 bits set, triangles 3 and if the point is in the interior 4 bits are set.
/// If must_include_d is true, the function assumes that D is part of the closest feature (vertex, edge, face, tetrahedron) and does less work, if the assumption is not true then a closest point to the other features is returned.
pub fn getClosestPointOnTetrahedron(a: Vec3, b: Vec3, c: Vec3, d: Vec3, comptime opts: struct { must_include_d: bool = false }) PointAndSet {
    // Taken from: Real-Time Collision Detection - Christer Ericson (Section: Closest Point on Tetrahedron to Point)
    // With p = 0

    // Start out assuming point inside all halfspaces, so closest to itself
    var closest_set: u32 = 0b1111;
    var closest_point = Vec3.zero();
    var best_dist_sq: f32 = math.flt_max;

    // Determine for each of the faces of the tetrahedron if the origin is in front of the plane
    const origin_out_of_planes = originOutsideOfTetrahedronPlanes(a, b, c, d);

    // If point outside face abc then compute closest point on abc
    if (origin_out_of_planes.getX() != 0) // originOutsideOfPlane(a, b, c, d)
    {
        if (opts.must_include_d) {
            // If the closest point must include D then ABC cannot be closest but the closest point
            // cannot be an interior point either so we return A as closest point
            closest_set = 0b0001;
            closest_point = a;
        } else {
            // Test the face normally
            const r = getClosestPointOnTriangle(a, b, c, .{ .must_include_c = false });
            closest_point = r.point;
            closest_set = r.set;
        }
        best_dist_sq = closest_point.lengthSq();
    }

    // Repeat test for face acd
    if (origin_out_of_planes.getY() != 0) // originOutsideOfPlane(a, c, d, b)
    {
        const r = getClosestPointOnTriangle(a, c, d, .{ .must_include_c = opts.must_include_d });
        const q = r.point;
        const set = r.set;
        const dist_sq = q.lengthSq();
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            closest_point = q;
            closest_set = (set & 0b0001) + ((set & 0b0110) << 1);
        }
    }

    // Repeat test for face adb
    if (origin_out_of_planes.getZ() != 0) // originOutsideOfPlane(a, d, b, c)
    {
        // Keep original vertex order, it doesn't matter if the triangle is facing inward or outward
        // and it improves consistency for GJK which will always add a new vertex D and keep the closest
        // feature from the previous iteration in ABC
        const r = getClosestPointOnTriangle(a, b, d, .{ .must_include_c = opts.must_include_d });
        const q = r.point;
        const set = r.set;
        const dist_sq = q.lengthSq();
        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            closest_point = q;
            closest_set = (set & 0b0011) + ((set & 0b0100) << 1);
        }
    }

    // Repeat test for face bdc
    if (origin_out_of_planes.getW() != 0) // originOutsideOfPlane(b, d, c, a)
    {
        // Keep original vertex order, it doesn't matter if the triangle is facing inward or outward
        // and it improves consistency for GJK which will always add a new vertex D and keep the closest
        // feature from the previous iteration in ABC
        const r = getClosestPointOnTriangle(b, c, d, .{ .must_include_c = opts.must_include_d });
        const q = r.point;
        const set = r.set;
        const dist_sq = q.lengthSq();
        if (dist_sq < best_dist_sq) {
            closest_point = q;
            closest_set = set << 1;
        }
    }

    return .{ .point = closest_point, .set = closest_set };
}

test "getClosestPointOnLine" {
    // Closest to a
    var r = getClosestPointOnLine(Vec3.init(1, 0, 0), Vec3.init(2, 0, 0));
    try std.testing.expectEqual(@as(u32, 0b0001), r.set);
    try std.testing.expect(r.point.eql(Vec3.init(1, 0, 0)));

    // Closest to b
    r = getClosestPointOnLine(Vec3.init(-2, 1, 0), Vec3.init(-1, 1, 0));
    try std.testing.expectEqual(@as(u32, 0b0010), r.set);
    try std.testing.expect(r.point.eql(Vec3.init(-1, 1, 0)));

    // Closest to interior
    r = getClosestPointOnLine(Vec3.init(-1, 1, 0), Vec3.init(1, 1, 0));
    try std.testing.expectEqual(@as(u32, 0b0011), r.set);
    try std.testing.expect(r.point.isClose(Vec3.init(0, 1, 0), .{}));

    // Degenerate line
    const bary = getBaryCentricCoordinates(Vec3.init(1, 2, 3), Vec3.init(1, 2, 3));
    try std.testing.expect(!bary.valid);
    try std.testing.expectEqual(@as(f32, 0), bary.u);
    try std.testing.expectEqual(@as(f32, 1), bary.v);
}

test "getBaryCentricCoordinatesTriangle" {
    // Origin inside the triangle
    const a = Vec3.init(-1, -1, 1);
    const b = Vec3.init(2, -1, 1);
    const c = Vec3.init(-1, 2, 1);
    const bary = getBaryCentricCoordinatesTriangle(a, b, c);
    try std.testing.expect(bary.valid);
    const p = a.mulScalar(bary.u).add(b.mulScalar(bary.v)).add(c.mulScalar(bary.w));
    try std.testing.expect(p.isClose(Vec3.init(0, 0, 1), .{}));

    // Degenerate triangles: all branches that fall back to an edge
    const line = Vec3.init(1, 0, 0);
    const bary1 = getBaryCentricCoordinatesTriangle(line, line.mulScalar(2), line.mulScalar(3)); // d00 <= d22, d00 <= d11
    try std.testing.expect(!bary1.valid);
    try std.testing.expectEqual(@as(f32, 0), bary1.v);
    const bary2 = getBaryCentricCoordinatesTriangle(line, Vec3.zero(), line.mulScalar(1.5)); // d00 <= d22, d00 > d11
    try std.testing.expect(!bary2.valid);
    try std.testing.expectEqual(@as(f32, 0), bary2.w);
    const bary3 = getBaryCentricCoordinatesTriangle(line, line.mulScalar(3), line.mulScalar(2)); // d00 > d22, d11 <= d22
    try std.testing.expect(!bary3.valid);
    try std.testing.expectEqual(@as(f32, 0), bary3.u);
    const bary4 = getBaryCentricCoordinatesTriangle(line.mulScalar(2), line.mulScalar(5), line.mulScalar(4)); // d00 > d22, d11 > d22
    try std.testing.expect(!bary4.valid);
    try std.testing.expectEqual(@as(f32, 0), bary4.v);
}

test "getClosestPointOnTriangle must_include_c" {
    // Degenerate triangle (all on the x axis) where the closest point is vertex a, with must_include_c the closest
    // point must be on an edge that includes c
    const a = Vec3.init(1, 1, 0);
    const b = Vec3.init(2, 1, 0);
    const c = Vec3.init(3, 1, 0);
    const r1 = getClosestPointOnTriangle(a, b, c, .{});
    try std.testing.expectEqual(@as(u32, 0b0001), r1.set);
    const r2 = getClosestPointOnTriangle(a, b, c, .{ .must_include_c = true });
    try std.testing.expect(r2.set & 0b0100 != 0);
    try std.testing.expect(r2.point.isClose(a, .{}));

    // Non degenerate, origin in the face region
    const r3 = getClosestPointOnTriangle(Vec3.init(-1, -1, 1), Vec3.init(2, -1, 1), Vec3.init(-1, 2, 1), .{ .must_include_c = true });
    try std.testing.expectEqual(@as(u32, 0b0111), r3.set);
    try std.testing.expect(r3.point.isClose(Vec3.init(0, 0, 1), .{}));
}

test "originOutsideOfPlane / getClosestPointOnTetrahedron" {
    const a = Vec3.init(-1, -1, -1);
    const b = Vec3.init(3, -1, -1);
    const c = Vec3.init(-1, 3, -1);
    const d = Vec3.init(-1, -1, 3);

    // Origin is inside
    try std.testing.expect(!originOutsideOfPlane(a, b, c, d));
    try std.testing.expect(originOutsideOfTetrahedronPlanes(a, b, c, d).eql(UVec4.zero()));
    var r = getClosestPointOnTetrahedron(a, b, c, d, .{});
    try std.testing.expectEqual(@as(u32, 0b1111), r.set);
    try std.testing.expect(r.point.eql(Vec3.zero()));

    // Origin outside of face abc (shift tetrahedron up)
    const up = Vec3.init(0, 0, 2);
    try std.testing.expect(originOutsideOfPlane(a.add(up), b.add(up), c.add(up), d.add(up)));
    r = getClosestPointOnTetrahedron(a.add(up), b.add(up), c.add(up), d.add(up), .{});
    try std.testing.expectEqual(@as(u32, 0b0111), r.set);
    try std.testing.expect(r.point.isClose(Vec3.init(0, 0, 1), .{}));

    // With must_include_d the face abc can't be the closest feature
    r = getClosestPointOnTetrahedron(a.add(up), b.add(up), c.add(up), d.add(up), .{ .must_include_d = true });
    try std.testing.expect(r.set & 0b1000 != 0 or r.set == 0b0001);

    // Origin closest to vertex d
    const shift = Vec3.init(1, 1, -4);
    r = getClosestPointOnTetrahedron(a.add(shift), b.add(shift), c.add(shift), d.add(shift), .{ .must_include_d = true });
    try std.testing.expectEqual(@as(u32, 0b1000), r.set);
    try std.testing.expect(r.point.isClose(d.add(shift), .{}));

    // Degenerate tetrahedron (flat): mixed signs, the origin is considered outside all planes
    const flat = originOutsideOfTetrahedronPlanes(a, b, c, Vec3.init(1, 1, -1));
    try std.testing.expect(flat.testAllTrue());
}
