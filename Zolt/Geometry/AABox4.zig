//! Port of: Jolt/Geometry/AABox4.h
//! Status: complete
//!
//! Helper functions that process 4 axis aligned boxes at the same time using SIMD. The boxes are passed in
//! SOA layout: one Vec4 per coordinate of min and max, lane i belongs to box i.
//!
//! Overloads: `aabox4VsBox` (vs AABox), `aabox4VsOrientedBox` (vs OrientedBox) and `aabox4VsOrientedBoxMat44`
//! (vs orientation + half extents); `aabox4DistanceSqToPoint` / `aabox4VsSphere` take the point / center as
//! 3 Vec4's, the `...Vec3` variants take a single Vec3 that is splatted.

const std = @import("std");
const math = @import("../Math/Math.zig");
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const UVec4 = @import("../Math/UVec4.zig").UVec4;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;
const AABox = @import("AABox.zig").AABox;
const OrientedBox = @import("OrientedBox.zig").OrientedBox;

/// 4 axis aligned boxes in SOA layout (the out parameters of AABox4Scale)
pub const AABox4Bounds = struct {
    min_x: Vec4,
    min_y: Vec4,
    min_z: Vec4,
    max_x: Vec4,
    max_y: Vec4,
    max_z: Vec4,
};

/// Optional arguments of the oriented box tests
pub const AABox4OrientedBoxOptions = struct { epsilon: f32 = 1.0e-6 };

/// Test if 4 bounding boxes overlap with 1 bounding box, splat 1 box (AABox4VsBox(const AABox &, ...))
pub fn aabox4VsBox(box1: AABox, box2_min_x: Vec4, box2_min_y: Vec4, box2_min_z: Vec4, box2_max_x: Vec4, box2_max_y: Vec4, box2_max_z: Vec4) UVec4 {
    // Splat values of box 1
    const box1_minx = box1.min.splatX();
    const box1_miny = box1.min.splatY();
    const box1_minz = box1.min.splatZ();
    const box1_maxx = box1.max.splatX();
    const box1_maxy = box1.max.splatY();
    const box1_maxz = box1.max.splatZ();

    // Test separation over each axis
    const nooverlapx = UVec4.bitOr(Vec4.greater(box1_minx, box2_max_x), Vec4.greater(box2_min_x, box1_maxx));
    const nooverlapy = UVec4.bitOr(Vec4.greater(box1_miny, box2_max_y), Vec4.greater(box2_min_y, box1_maxy));
    const nooverlapz = UVec4.bitOr(Vec4.greater(box1_minz, box2_max_z), Vec4.greater(box2_min_z, box1_maxz));

    // Return overlap
    return UVec4.bitNot(UVec4.bitOr(UVec4.bitOr(nooverlapx, nooverlapy), nooverlapz));
}

/// Scale 4 axis aligned boxes
pub fn aabox4Scale(scale: Vec3, box_min_x: Vec4, box_min_y: Vec4, box_min_z: Vec4, box_max_x: Vec4, box_max_y: Vec4, box_max_z: Vec4) AABox4Bounds {
    var out: AABox4Bounds = undefined;

    const scale_x = scale.splatX();
    const scaled_min_x = scale_x.mul(box_min_x);
    const scaled_max_x = scale_x.mul(box_max_x);
    out.min_x = Vec4.min(scaled_min_x, scaled_max_x); // Negative scale can flip min and max
    out.max_x = Vec4.max(scaled_min_x, scaled_max_x);

    const scale_y = scale.splatY();
    const scaled_min_y = scale_y.mul(box_min_y);
    const scaled_max_y = scale_y.mul(box_max_y);
    out.min_y = Vec4.min(scaled_min_y, scaled_max_y);
    out.max_y = Vec4.max(scaled_min_y, scaled_max_y);

    const scale_z = scale.splatZ();
    const scaled_min_z = scale_z.mul(box_min_z);
    const scaled_max_z = scale_z.mul(box_max_z);
    out.min_z = Vec4.min(scaled_min_z, scaled_max_z);
    out.max_z = Vec4.max(scaled_min_z, scaled_max_z);

    return out;
}

/// Enlarge 4 bounding boxes with extent (add to both sides)
pub fn aabox4EnlargeWithExtent(extent: Vec3, bounds_min_x: *Vec4, bounds_min_y: *Vec4, bounds_min_z: *Vec4, bounds_max_x: *Vec4, bounds_max_y: *Vec4, bounds_max_z: *Vec4) void {
    const extent_x = extent.splatX();
    bounds_min_x.* = bounds_min_x.sub(extent_x);
    bounds_max_x.* = bounds_max_x.add(extent_x);

    const extent_y = extent.splatY();
    bounds_min_y.* = bounds_min_y.sub(extent_y);
    bounds_max_y.* = bounds_max_y.add(extent_y);

    const extent_z = extent.splatZ();
    bounds_min_z.* = bounds_min_z.sub(extent_z);
    bounds_max_z.* = bounds_max_z.add(extent_z);
}

/// Test if 4 bounding boxes overlap with a point
pub fn aabox4VsPoint(point: Vec3, box_min_x: Vec4, box_min_y: Vec4, box_min_z: Vec4, box_max_x: Vec4, box_max_y: Vec4, box_max_z: Vec4) UVec4 {
    // Splat point to 4 component vectors
    const point_x = Vec4.fromVec3(point).splatX();
    const point_y = Vec4.fromVec3(point).splatY();
    const point_z = Vec4.fromVec3(point).splatZ();

    // Test if point overlaps with box
    const overlapx = UVec4.bitAnd(Vec4.greaterOrEqual(point_x, box_min_x), Vec4.lessOrEqual(point_x, box_max_x));
    const overlapy = UVec4.bitAnd(Vec4.greaterOrEqual(point_y, box_min_y), Vec4.lessOrEqual(point_y, box_max_y));
    const overlapz = UVec4.bitAnd(Vec4.greaterOrEqual(point_z, box_min_z), Vec4.lessOrEqual(point_z, box_max_z));

    // Test if all are overlapping
    return UVec4.bitAnd(UVec4.bitAnd(overlapx, overlapy), overlapz);
}

/// Test if 4 bounding boxes overlap with an oriented box (AABox4VsBox(Mat44Arg, Vec3Arg, ..., float inEpsilon = 1.0e-6f))
pub fn aabox4VsOrientedBoxMat44(orientation: Mat44, half_extents: Vec3, box_min_x: Vec4, box_min_y: Vec4, box_min_z: Vec4, box_max_x: Vec4, box_max_y: Vec4, box_max_z: Vec4, opts: AABox4OrientedBoxOptions) UVec4 {
    // Taken from: Real Time Collision Detection - Christer Ericson
    // Chapter 4.4.1, page 103-105.
    // Note that the code is swapped around: A is the aabox and B is the oriented box (this saves us from having to invert the orientation of the oriented box)

    // Compute translation vector t (the translation of B in the space of A)
    const t = [3]Vec4{
        orientation.getTranslation().splatX().sub(box_min_x.add(box_max_x).mulScalar(0.5)),
        orientation.getTranslation().splatY().sub(box_min_y.add(box_max_y).mulScalar(0.5)),
        orientation.getTranslation().splatZ().sub(box_min_z.add(box_max_z).mulScalar(0.5)),
    };

    // Compute common subexpressions. Add in an epsilon term to
    // counteract arithmetic errors when two edges are parallel and
    // their cross product is (near) null (see text for details)
    const epsilon = Vec3.replicate(opts.epsilon);
    const abs_r = [3]Vec3{ orientation.getAxisX().abs().add(epsilon), orientation.getAxisY().abs().add(epsilon), orientation.getAxisZ().abs().add(epsilon) };

    // Half extents for a
    const a_half_extents = [3]Vec4{
        box_max_x.sub(box_min_x).mulScalar(0.5),
        box_max_y.sub(box_min_y).mulScalar(0.5),
        box_max_z.sub(box_min_z).mulScalar(0.5),
    };

    // Half extents of b
    const b_half_extents_x = half_extents.splatX();
    const b_half_extents_y = half_extents.splatY();
    const b_half_extents_z = half_extents.splatZ();

    // Each component corresponds to 1 overlapping OBB vs ABB
    var overlaps = UVec4.init(0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff);

    // Element (r, c) of abs_r, abs_r[r][c] in Jolt
    const R = struct {
        abs_r: [3]Vec3,
        fn at(self: @This(), r: u32, c: u32) f32 {
            return self.abs_r[r].getComponent(c);
        }
    };
    const r: R = .{ .abs_r = abs_r };

    // Test axes L = A0, L = A1, L = A2
    var ra: Vec4 = undefined;
    var rb: Vec4 = undefined;
    for (0..3) |iu| {
        const i: u32 = @intCast(iu);
        ra = a_half_extents[i];
        rb = b_half_extents_x.mulScalar(r.at(0, i)).add(b_half_extents_y.mulScalar(r.at(1, i))).add(b_half_extents_z.mulScalar(r.at(2, i)));
        overlaps = UVec4.bitAnd(overlaps, Vec4.lessOrEqual(t[i].abs(), ra.add(rb)));
    }

    // Test axes L = B0, L = B1, L = B2
    for (0..3) |iu| {
        const i: u32 = @intCast(iu);
        ra = a_half_extents[0].mulScalar(r.at(i, 0)).add(a_half_extents[1].mulScalar(r.at(i, 1))).add(a_half_extents[2].mulScalar(r.at(i, 2)));
        rb = Vec4.replicate(half_extents.getComponent(i));
        overlaps = UVec4.bitAnd(overlaps, Vec4.lessOrEqual(t[0].mulScalar(orientation.get(0, i)).add(t[1].mulScalar(orientation.get(1, i))).add(t[2].mulScalar(orientation.get(2, i))).abs(), ra.add(rb)));
    }

    // Test axis L = A0 x B0
    ra = a_half_extents[1].mulScalar(r.at(0, 2)).add(a_half_extents[2].mulScalar(r.at(0, 1)));
    rb = b_half_extents_y.mulScalar(r.at(2, 0)).add(b_half_extents_z.mulScalar(r.at(1, 0)));
    overlaps = UVec4.bitAnd(overlaps, Vec4.lessOrEqual(t[2].mulScalar(orientation.get(1, 0)).sub(t[1].mulScalar(orientation.get(2, 0))).abs(), ra.add(rb)));

    // Test axis L = A0 x B1
    ra = a_half_extents[1].mulScalar(r.at(1, 2)).add(a_half_extents[2].mulScalar(r.at(1, 1)));
    rb = b_half_extents_x.mulScalar(r.at(2, 0)).add(b_half_extents_z.mulScalar(r.at(0, 0)));
    overlaps = UVec4.bitAnd(overlaps, Vec4.lessOrEqual(t[2].mulScalar(orientation.get(1, 1)).sub(t[1].mulScalar(orientation.get(2, 1))).abs(), ra.add(rb)));

    // Test axis L = A0 x B2
    ra = a_half_extents[1].mulScalar(r.at(2, 2)).add(a_half_extents[2].mulScalar(r.at(2, 1)));
    rb = b_half_extents_x.mulScalar(r.at(1, 0)).add(b_half_extents_y.mulScalar(r.at(0, 0)));
    overlaps = UVec4.bitAnd(overlaps, Vec4.lessOrEqual(t[2].mulScalar(orientation.get(1, 2)).sub(t[1].mulScalar(orientation.get(2, 2))).abs(), ra.add(rb)));

    // Test axis L = A1 x B0
    ra = a_half_extents[0].mulScalar(r.at(0, 2)).add(a_half_extents[2].mulScalar(r.at(0, 0)));
    rb = b_half_extents_y.mulScalar(r.at(2, 1)).add(b_half_extents_z.mulScalar(r.at(1, 1)));
    overlaps = UVec4.bitAnd(overlaps, Vec4.lessOrEqual(t[0].mulScalar(orientation.get(2, 0)).sub(t[2].mulScalar(orientation.get(0, 0))).abs(), ra.add(rb)));

    // Test axis L = A1 x B1
    ra = a_half_extents[0].mulScalar(r.at(1, 2)).add(a_half_extents[2].mulScalar(r.at(1, 0)));
    rb = b_half_extents_x.mulScalar(r.at(2, 1)).add(b_half_extents_z.mulScalar(r.at(0, 1)));
    overlaps = UVec4.bitAnd(overlaps, Vec4.lessOrEqual(t[0].mulScalar(orientation.get(2, 1)).sub(t[2].mulScalar(orientation.get(0, 1))).abs(), ra.add(rb)));

    // Test axis L = A1 x B2
    ra = a_half_extents[0].mulScalar(r.at(2, 2)).add(a_half_extents[2].mulScalar(r.at(2, 0)));
    rb = b_half_extents_x.mulScalar(r.at(1, 1)).add(b_half_extents_y.mulScalar(r.at(0, 1)));
    overlaps = UVec4.bitAnd(overlaps, Vec4.lessOrEqual(t[0].mulScalar(orientation.get(2, 2)).sub(t[2].mulScalar(orientation.get(0, 2))).abs(), ra.add(rb)));

    // Test axis L = A2 x B0
    ra = a_half_extents[0].mulScalar(r.at(0, 1)).add(a_half_extents[1].mulScalar(r.at(0, 0)));
    rb = b_half_extents_y.mulScalar(r.at(2, 2)).add(b_half_extents_z.mulScalar(r.at(1, 2)));
    overlaps = UVec4.bitAnd(overlaps, Vec4.lessOrEqual(t[1].mulScalar(orientation.get(0, 0)).sub(t[0].mulScalar(orientation.get(1, 0))).abs(), ra.add(rb)));

    // Test axis L = A2 x B1
    ra = a_half_extents[0].mulScalar(r.at(1, 1)).add(a_half_extents[1].mulScalar(r.at(1, 0)));
    rb = b_half_extents_x.mulScalar(r.at(2, 2)).add(b_half_extents_z.mulScalar(r.at(0, 2)));
    overlaps = UVec4.bitAnd(overlaps, Vec4.lessOrEqual(t[1].mulScalar(orientation.get(0, 1)).sub(t[0].mulScalar(orientation.get(1, 1))).abs(), ra.add(rb)));

    // Test axis L = A2 x B2
    ra = a_half_extents[0].mulScalar(r.at(2, 1)).add(a_half_extents[1].mulScalar(r.at(2, 0)));
    rb = b_half_extents_x.mulScalar(r.at(1, 2)).add(b_half_extents_y.mulScalar(r.at(0, 2)));
    overlaps = UVec4.bitAnd(overlaps, Vec4.lessOrEqual(t[1].mulScalar(orientation.get(0, 2)).sub(t[0].mulScalar(orientation.get(1, 2))).abs(), ra.add(rb)));

    // Return if the OBB vs AABBs are intersecting
    return overlaps;
}

/// Convenience function that tests 4 AABoxes vs OrientedBox (AABox4VsBox(const OrientedBox &, ..., float inEpsilon = 1.0e-6f))
pub fn aabox4VsOrientedBox(box: OrientedBox, box_min_x: Vec4, box_min_y: Vec4, box_min_z: Vec4, box_max_x: Vec4, box_max_y: Vec4, box_max_z: Vec4, opts: AABox4OrientedBoxOptions) UVec4 {
    return aabox4VsOrientedBoxMat44(box.orientation, box.half_extents, box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z, opts);
}

/// Get the squared distance between 4 AABoxes and a point
pub fn aabox4DistanceSqToPoint(point_x: Vec4, point_y: Vec4, point_z: Vec4, box_min_x: Vec4, box_min_y: Vec4, box_min_z: Vec4, box_max_x: Vec4, box_max_y: Vec4, box_max_z: Vec4) Vec4 {
    // Get closest point on box
    const closest_x = Vec4.min(Vec4.max(point_x, box_min_x), box_max_x);
    const closest_y = Vec4.min(Vec4.max(point_y, box_min_y), box_max_y);
    const closest_z = Vec4.min(Vec4.max(point_z, box_min_z), box_max_z);

    // Return the squared distance between the box and point
    return square(closest_x.sub(point_x)).add(square(closest_y.sub(point_y))).add(square(closest_z.sub(point_z)));
}

/// Get the squared distance between 4 AABoxes and a point (AABox4DistanceSqToPoint(Vec3, ...))
pub fn aabox4DistanceSqToPointVec3(point: Vec3, box_min_x: Vec4, box_min_y: Vec4, box_min_z: Vec4, box_max_x: Vec4, box_max_y: Vec4, box_max_z: Vec4) Vec4 {
    return aabox4DistanceSqToPoint(point.splatX(), point.splatY(), point.splatZ(), box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z);
}

/// Test 4 AABoxes vs a sphere
pub fn aabox4VsSphere(center_x: Vec4, center_y: Vec4, center_z: Vec4, radius_sq: Vec4, box_min_x: Vec4, box_min_y: Vec4, box_min_z: Vec4, box_max_x: Vec4, box_max_y: Vec4, box_max_z: Vec4) UVec4 {
    // Test the distance from the center of the sphere to the box is smaller than the radius
    const distance_sq = aabox4DistanceSqToPoint(center_x, center_y, center_z, box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z);
    return Vec4.lessOrEqual(distance_sq, radius_sq);
}

/// Test 4 AABoxes vs a sphere (AABox4VsSphere(Vec3Arg, float, ...))
pub fn aabox4VsSphereVec3(center: Vec3, radius_sq: f32, box_min_x: Vec4, box_min_y: Vec4, box_min_z: Vec4, box_max_x: Vec4, box_max_y: Vec4, box_max_z: Vec4) UVec4 {
    return aabox4VsSphere(center.splatX(), center.splatY(), center.splatZ(), Vec4.replicate(radius_sq), box_min_x, box_min_y, box_min_z, box_max_x, box_max_y, box_max_z);
}

/// Square(Vec4) = v * v
fn square(v: Vec4) Vec4 {
    return v.mul(v);
}

test "AABox4" {
    // 4 boxes: unit box at x = 0, 2, 4 and 10
    const min_x = Vec4.init(0, 2, 4, 10);
    const min_y = Vec4.zero();
    const min_z = Vec4.zero();
    const max_x = min_x.add(Vec4.one());
    const max_y = Vec4.one();
    const max_z = Vec4.one();

    // Box touching the first two boxes
    const overlap = aabox4VsBox(AABox.init(Vec3.init(1, 0.5, 0.5), Vec3.init(2, 0.6, 0.6)), min_x, min_y, min_z, max_x, max_y, max_z);
    try std.testing.expect(overlap.eql(UVec4.init(0xffffffff, 0xffffffff, 0, 0)));

    try std.testing.expect(aabox4VsPoint(Vec3.init(4.5, 1, 0), min_x, min_y, min_z, max_x, max_y, max_z).eql(UVec4.init(0, 0, 0xffffffff, 0)));

    const scaled = aabox4Scale(Vec3.init(-1, 2, 1), min_x, min_y, min_z, max_x, max_y, max_z);
    try std.testing.expect(scaled.min_x.eql(Vec4.init(-1, -3, -5, -11)));
    try std.testing.expect(scaled.max_x.eql(Vec4.init(0, -2, -4, -10)));
    try std.testing.expect(scaled.max_y.eql(Vec4.replicate(2)));

    var bx0 = min_x;
    var by0 = min_y;
    var bz0 = min_z;
    var bx1 = max_x;
    var by1 = max_y;
    var bz1 = max_z;
    aabox4EnlargeWithExtent(Vec3.init(1, 2, 3), &bx0, &by0, &bz0, &bx1, &by1, &bz1);
    try std.testing.expect(bx0.eql(Vec4.init(-1, 1, 3, 9)));
    try std.testing.expect(by1.eql(Vec4.replicate(3)));
    try std.testing.expect(bz0.eql(Vec4.replicate(-3)));

    const dist_sq = aabox4DistanceSqToPointVec3(Vec3.init(3, 0.5, 2), min_x, min_y, min_z, max_x, max_y, max_z);
    try std.testing.expect(dist_sq.eql(Vec4.init(4 + 1, 0 + 1, 1 + 1, 49 + 1)));
    try std.testing.expect(aabox4VsSphereVec3(Vec3.init(3, 0.5, 2), 2, min_x, min_y, min_z, max_x, max_y, max_z).eql(UVec4.init(0, 0xffffffff, 0xffffffff, 0)));

    // Oriented box around the box at x = 4
    const obox = OrientedBox.init(Mat44.rotationY(0.25 * math.pi).postTranslated(Vec3.init(4.5, 0.5, 0.5)), Vec3.replicate(0.2));
    try std.testing.expect(aabox4VsOrientedBox(obox, min_x, min_y, min_z, max_x, max_y, max_z, .{}).eql(UVec4.init(0, 0, 0xffffffff, 0)));
    const obox2 = OrientedBox.init(Mat44.translation(Vec3.init(3, 0.5, 0.5)), Vec3.replicate(1));
    try std.testing.expect(aabox4VsOrientedBoxMat44(obox2.orientation, obox2.half_extents, min_x, min_y, min_z, max_x, max_y, max_z, .{}).eql(UVec4.init(0, 0xffffffff, 0xffffffff, 0)));
}
