//! Port of: Jolt/Physics/Collision/ManifoldBetweenTwoFaces.h, Jolt/Physics/Collision/ManifoldBetweenTwoFaces.cpp
//! Status: complete
//!
//! The free functions `PruneContactPoints` / `ManifoldBetweenTwoFaces` are `pruneContactPoints` /
//! `manifoldBetweenTwoFaces`, the `ContactPoints &` in/out parameters are `*ContactPoints`.
//! JPH_DEBUG_RENDERER (the drawing behind ContactConstraintManager::sDrawContactPointReduction / sDrawContactPoint /
//! sDrawSupportingFaces and the `inCenterOfMass` parameter that only exists for it) is not ported yet:
//! TODO(debug_renderer). ManifoldBetweenTwoFaces.cpp includes ContactConstraintManager.h only for these flags.

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const StaticArray = @import("../../Core/StaticArray.zig").StaticArray;
const ClipPoly = @import("../../Geometry/ClipPoly.zig");
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Shape = @import("Shape/Shape.zig").Shape;
const ContactPoints = @import("ContactListener.zig").ContactPoints;

/// The supporting face of a shape (ConvexShape::SupportingFace, inherited from Shape)
const SupportingFace = Shape.SupportingFace;

/// Remove contact points if there are > 4 (no more than 4 are needed for a stable solution)
/// @param penetration_axis is the world space penetration axis (must be normalized)
/// @param contact_points_on1 The contact points on shape 1 relative to inCenterOfMass
/// @param contact_points_on2 The contact points on shape 2 relative to inCenterOfMass
/// On output contact_points_on1/2 are reduced to 4 or less points
pub fn pruneContactPoints(penetration_axis: Vec3, contact_points_on1: *ContactPoints, contact_points_on2: *ContactPoints) void {
    // Makes no sense to call this with 4 or less points
    std.debug.assert(contact_points_on1.len > 4);

    // Both arrays should have the same size
    std.debug.assert(contact_points_on1.len == contact_points_on2.len);

    // Penetration axis must be normalized
    // (only checked when asserts are enabled: Jolt's release build continues with a slightly denormalized axis)
    if (Core.enable_asserts) std.debug.assert(penetration_axis.isNormalized(.{}));

    // We use a heuristic of (distance to center of mass) * (penetration depth) to find the contact point that we should keep
    // Neither of those two terms should ever become zero, so we clamp against this minimum value
    const min_distance_sq: f32 = 1.0e-6; // 1 mm

    var projected: ContactPoints = .empty;
    var penetration_depth_sq: StaticArray(f32, 64) = .empty;
    for (0..contact_points_on1.len) |i| {
        // Project contact points on the plane through inCenterOfMass with normal penetration_axis and center around the center of mass of body 1
        // (note that since all points are relative to inCenterOfMass we can project onto the plane through the origin)
        const v1 = contact_points_on1.get(@intCast(i));
        projected.append(v1.sub(penetration_axis.mulScalar(v1.dot(penetration_axis))));

        // Calculate penetration depth^2 of each point and clamp against the minimal distance
        const v2 = contact_points_on2.get(@intCast(i));
        penetration_depth_sq.append(math.max(min_distance_sq, v2.sub(v1).lengthSq()));
    }

    // Find the point that is furthest away from the center of mass (its torque will have the biggest influence)
    // and the point that has the deepest penetration depth. Use the heuristic (distance to center of mass) * (penetration depth) for this.
    var point1: u32 = 0;
    var val = math.max(min_distance_sq, projected.get(0).lengthSq()) * penetration_depth_sq.get(0);
    for (0..projected.len) |i_usize| {
        const i: u32 = @intCast(i_usize);
        const v = math.max(min_distance_sq, projected.get(i).lengthSq()) * penetration_depth_sq.get(i);
        if (v > val) {
            val = v;
            point1 = i;
        }
    }
    const point1v = projected.get(point1);

    // Find point furthest from the first point forming a line segment with point1. Again combine this with the heuristic
    // for deepest point as per above.
    const invalid_index = std.math.maxInt(u32); // uint(-1)
    var point2: u32 = invalid_index;
    val = -math.flt_max;
    for (0..projected.len) |i_usize| {
        const i: u32 = @intCast(i_usize);
        if (i != point1) {
            const v = math.max(min_distance_sq, projected.get(i).sub(point1v).lengthSq()) * penetration_depth_sq.get(i);
            if (v > val) {
                val = v;
                point2 = i;
            }
        }
    }
    std.debug.assert(point2 != invalid_index);
    const point2v = projected.get(point2);

    // Find furthest points on both sides of the line segment in order to maximize the area
    var point3: u32 = invalid_index;
    var point4: u32 = invalid_index;
    var min_val: f32 = 0.0;
    var max_val: f32 = 0.0;
    const perp = point2v.sub(point1v).cross(penetration_axis);
    for (0..projected.len) |i_usize| {
        const i: u32 = @intCast(i_usize);
        if (i != point1 and i != point2) {
            const v = perp.dot(projected.get(i).sub(point1v));
            if (v < min_val) {
                min_val = v;
                point3 = i;
            } else if (v > max_val) {
                max_val = v;
                point4 = i;
            }
        }
    }

    // Add points to array (in order so they form a polygon)
    var points_to_keep_on_1: StaticArray(Vec3, 4) = .empty;
    var points_to_keep_on_2: StaticArray(Vec3, 4) = .empty;
    points_to_keep_on_1.append(contact_points_on1.get(point1));
    points_to_keep_on_2.append(contact_points_on2.get(point1));
    if (point3 != invalid_index) {
        points_to_keep_on_1.append(contact_points_on1.get(point3));
        points_to_keep_on_2.append(contact_points_on2.get(point3));
    }
    points_to_keep_on_1.append(contact_points_on1.get(point2));
    points_to_keep_on_2.append(contact_points_on2.get(point2));
    if (point4 != invalid_index) {
        std.debug.assert(point3 != point4);
        points_to_keep_on_1.append(contact_points_on1.get(point4));
        points_to_keep_on_2.append(contact_points_on2.get(point4));
    }

    // TODO(debug_renderer): Draw input polygon, primary axis and the contact points we kept (ContactConstraintManager::sDrawContactPointReduction)

    // Copy the points back to the input buffer
    contact_points_on1.assign(&points_to_keep_on_1);
    contact_points_on2.assign(&points_to_keep_on_2);
}

/// Determine contact points between 2 faces of 2 shapes and return them in out_contact_points 1 & 2
/// @param contact_point1 The contact point on shape 1 relative to inCenterOfMass
/// @param contact_point2 The contact point on shape 2 relative to inCenterOfMass
/// @param penetration_axis The local space penetration axis in world space
/// @param max_contact_distance After face 2 is clipped against face 1, each remaining point on face 2 is tested against the plane of face 1. If the distance on the positive side of the plane is larger than this distance, the point will be discarded as a contact point.
/// @param shape1_face The supporting faces on shape 1 relative to inCenterOfMass
/// @param shape2_face The supporting faces on shape 2 relative to inCenterOfMass
/// @param out_contact_points1 Returns the contact points between the two shapes for shape 1 relative to inCenterOfMass (any existing points in the output array are left as is)
/// @param out_contact_points2 Returns the contact points between the two shapes for shape 2 relative to inCenterOfMass (any existing points in the output array are left as is)
pub fn manifoldBetweenTwoFaces(contact_point1: Vec3, contact_point2: Vec3, penetration_axis_in: Vec3, max_contact_distance: f32, shape1_face_in: *const SupportingFace, shape2_face_in: *const SupportingFace, out_contact_points1: *ContactPoints, out_contact_points2: *ContactPoints) void {
    std.debug.assert(max_contact_distance > 0.0);

    // TODO(debug_renderer): Draw contact points and the contact normal (ContactConstraintManager::sDrawContactPoint)

    // Remember size before adding new points, to check at the end if we added some
    const old_size = out_contact_points1.len;

    // Both faces need to have at least 2 points or else there can never be more than 1 contact point
    // At least one face needs to have at least 3 points (in the case that it has 2 points only if the edges match exactly you can have 2 contact points, but this situation is unstable anyhow)
    if (@min(shape1_face_in.len, shape2_face_in.len) >= 2 and @max(shape1_face_in.len, shape2_face_in.len) >= 3) {
        // Swap the shapes if the 2nd face doesn't have enough vertices
        var shape1_face: *const SupportingFace = undefined;
        var shape2_face: *const SupportingFace = undefined;
        var contact_points1: *ContactPoints = undefined;
        var contact_points2: *ContactPoints = undefined;
        var penetration_axis: Vec3 = undefined;
        if (shape2_face_in.len >= 3) {
            shape1_face = shape1_face_in;
            shape2_face = shape2_face_in;
            contact_points1 = out_contact_points1;
            contact_points2 = out_contact_points2;
            penetration_axis = penetration_axis_in;
        } else {
            shape1_face = shape2_face_in;
            shape2_face = shape1_face_in;
            contact_points1 = out_contact_points2;
            contact_points2 = out_contact_points1;
            penetration_axis = penetration_axis_in.negate();
        }

        // Determine plane origin and first edge direction
        const plane_origin = shape1_face.get(0);
        const first_edge = shape1_face.get(1).sub(plane_origin);

        var plane_normal: Vec3 = undefined;
        var clipped_face: SupportingFace = .empty;
        if (shape1_face.len >= 3) {
            // Clip the polygon of face 2 against that of 1
            try ClipPoly.clipPolyVsPoly(shape2_face.constSlice(), shape1_face.constSlice(), penetration_axis, &clipped_face);

            // Three vertices, can just calculate the normal
            plane_normal = first_edge.cross(shape1_face.get(2).sub(plane_origin));
        } else {
            // Clip the polygon of face 2 against edge of 1
            try ClipPoly.clipPolyVsEdge(shape2_face.constSlice(), shape1_face.get(0), shape1_face.get(1), penetration_axis, &clipped_face);

            // Two vertices, first find a perpendicular to the edge and penetration axis and then use the perpendicular together with the edge to form a normal
            plane_normal = first_edge.cross(penetration_axis).cross(first_edge);
        }

        // If penetration axis and plane normal are perpendicular, fall back to the contact points
        const penetration_axis_dot_plane_normal = penetration_axis.dot(plane_normal);
        if (penetration_axis_dot_plane_normal != 0.0) {
            const penetration_axis_len = penetration_axis.length();

            for (clipped_face.constSlice()) |p2| {
                // Project clipped face back onto the plane of face 1, we do this by solving:
                // p1 = p2 + distance * penetration_axis / |penetration_axis|
                // (p1 - plane_origin) . plane_normal = 0
                // This gives us:
                // distance = -|penetration_axis| * (p2 - plane_origin) . plane_normal / penetration_axis . plane_normal
                const distance = p2.sub(plane_origin).dot(plane_normal) / penetration_axis_dot_plane_normal; // note left out -|penetration_axis| term

                // If the point is less than max_contact_distance in front of the plane of face 2, add it as a contact point
                if (distance * penetration_axis_len < max_contact_distance) {
                    const p1 = p2.sub(penetration_axis.mulScalar(distance));
                    contact_points1.append(p1);
                    contact_points2.append(p2);
                }
            }
        }

        // TODO(debug_renderer): Draw the clipped poly, the supporting faces, the plane normal and the contact points that remain
        // after the distance check (ContactConstraintManager::sDrawSupportingFaces)
    }

    // If the clipping result is empty, use the contact point itself
    if (out_contact_points1.len == old_size) {
        out_contact_points1.append(contact_point1);
        out_contact_points2.append(contact_point2);
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the bit exact comparison with Jolt is in ZoltParity/Physics/TrianglesParity.zig)

const testing = std.testing;

fn face(points: []const Vec3) SupportingFace {
    return .fromSlice(points);
}

test "manifoldBetweenTwoFaces: two overlapping quads give the clipped polygon projected on face 1" {
    // Face 1: square in the plane y = 0, face 2: a smaller square 0.1 below it. Shape 2 moves out of collision along +y, so
    // the faces penetrate by 0.1.
    const face1 = face(&.{ Vec3.init(-1, 0, -1), Vec3.init(-1, 0, 1), Vec3.init(1, 0, 1), Vec3.init(1, 0, -1) });
    const face2 = face(&.{ Vec3.init(-0.5, -0.1, -0.5), Vec3.init(0.5, -0.1, -0.5), Vec3.init(0.5, -0.1, 0.5), Vec3.init(-0.5, -0.1, 0.5) });
    var points1: ContactPoints = .empty;
    var points2: ContactPoints = .empty;
    manifoldBetweenTwoFaces(Vec3.init(0, 0, 0), Vec3.init(0, -0.1, 0), Vec3.init(0, 1, 0), 0.05, &face1, &face2, &points1, &points2);
    try testing.expectEqual(@as(u32, 4), points1.len);
    try testing.expectEqual(@as(u32, 4), points2.len);
    for (points1.constSlice(), points2.constSlice()) |p1, p2| {
        try testing.expectApproxEqAbs(@as(f32, 0.0), p1.getY(), 1.0e-6); // Projected on face 1
        try testing.expectApproxEqAbs(@as(f32, -0.1), p2.getY(), 1.0e-6);
        try testing.expectApproxEqAbs(@as(f32, 0.5), @abs(p2.getX()), 1.0e-6);
        try testing.expectApproxEqAbs(p1.getX(), p2.getX(), 1.0e-6);
    }
}

test "manifoldBetweenTwoFaces: points too far in front of face 1, edges, swapped faces and the fallback" {
    const face1 = face(&.{ Vec3.init(-1, 0, -1), Vec3.init(-1, 0, 1), Vec3.init(1, 0, 1), Vec3.init(1, 0, -1) });

    // Face 2 is 0.5 above face 1 (separated along +y) and the max contact distance is 0.25: fall back to the contact points
    const far = face(&.{ Vec3.init(-0.5, 0.5, -0.5), Vec3.init(0.5, 0.5, -0.5), Vec3.init(0.5, 0.5, 0.5) });
    var points1: ContactPoints = .empty;
    var points2: ContactPoints = .empty;
    points1.append(Vec3.init(9, 9, 9)); // Existing points are kept
    points2.append(Vec3.init(9, 9, 9));
    manifoldBetweenTwoFaces(Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(0, 1, 0), 0.25, &face1, &far, &points1, &points2);
    try testing.expectEqual(@as(u32, 2), points1.len);
    try testing.expect(points1.get(0).eql(Vec3.init(9, 9, 9)));
    try testing.expect(points1.get(1).eql(Vec3.init(1, 2, 3)) and points2.get(1).eql(Vec3.init(4, 5, 6)));

    // With a max contact distance of 1 the points are accepted
    points1.clear();
    points2.clear();
    manifoldBetweenTwoFaces(Vec3.init(1, 2, 3), Vec3.init(4, 5, 6), Vec3.init(0, 1, 0), 1.0, &face1, &far, &points1, &points2);
    try testing.expectEqual(@as(u32, 3), points1.len);

    // Face 2 is an edge: the faces are swapped (face 1 is clipped against the edge, the axis negated)
    const edge = face(&.{ Vec3.init(-2, -0.1, 0.25), Vec3.init(2, -0.1, 0.25) });
    points1.clear();
    points2.clear();
    manifoldBetweenTwoFaces(Vec3.zero(), Vec3.init(0, -0.1, 0), Vec3.init(0, 1, 0), 0.5, &face1, &edge, &points1, &points2);
    try testing.expectEqual(@as(u32, 2), points1.len);
    for (points1.constSlice(), points2.constSlice()) |p1, p2| {
        try testing.expectApproxEqAbs(@as(f32, 0.0), p1.getY(), 1.0e-6); // On face 1
        try testing.expectApproxEqAbs(@as(f32, -0.1), p2.getY(), 1.0e-6); // On the edge
        try testing.expectApproxEqAbs(@as(f32, 1.0), @abs(p1.getX()), 1.0e-6); // Clipped to face 1
    }

    // A single point face: the contact points themselves
    const point = face(&.{Vec3.init(0, -0.1, 0)});
    points1.clear();
    points2.clear();
    manifoldBetweenTwoFaces(Vec3.zero(), Vec3.init(0, -0.1, 0), Vec3.init(0, 1, 0), 0.5, &face1, &point, &points1, &points2);
    try testing.expectEqual(@as(u32, 1), points1.len);

    // Penetration axis perpendicular to the plane normal of face 1: the contact points themselves
    const square2 = face(&.{ Vec3.init(-0.5, -0.1, -0.5), Vec3.init(0.5, -0.1, -0.5), Vec3.init(0.5, -0.1, 0.5) });
    points1.clear();
    points2.clear();
    manifoldBetweenTwoFaces(Vec3.zero(), Vec3.init(0, -0.1, 0), Vec3.init(1, 0, 0), 0.5, &face1, &square2, &points1, &points2);
    try testing.expectEqual(@as(u32, 1), points1.len);
}

test "pruneContactPoints: reduces to at most 4 points that form a polygon" {
    // 8 points on a circle with equal penetration: the reduction keeps 4 points, the first one is the deepest / furthest
    var points1: ContactPoints = .empty;
    var points2: ContactPoints = .empty;
    for (0..8) |i| {
        const angle = @as(f32, @floatFromInt(i)) * (2.0 * math.pi / 8.0);
        const p = Vec3.init(@cos(angle), 0, @sin(angle)).mulScalar(if (i == 3) 3.0 else 1.0);
        points1.append(p);
        points2.append(p.add(Vec3.init(0, if (i == 6) -0.2 else -0.1, 0)));
    }
    const original1 = points1;
    pruneContactPoints(Vec3.init(0, 1, 0), &points1, &points2);
    try testing.expectEqual(@as(u32, 4), points1.len);
    try testing.expectEqual(@as(u32, 4), points2.len);
    // The point furthest from the center (index 3) wins the heuristic, it is the first point
    try testing.expect(points1.get(0).eql(original1.get(3)));
    // Every kept point is one of the input points, and the pairs stay together
    for (points1.constSlice(), points2.constSlice()) |p1, p2| {
        var found = false;
        for (original1.constSlice()) |o| found = found or o.eql(p1);
        try testing.expect(found);
        try testing.expectApproxEqAbs(@as(f32, 0.0), p1.getY(), 1.0e-6);
        try testing.expect(p2.getY() < 0.0);
    }

    // Collinear points: no points on either side of the line segment, only 2 remain
    points1.clear();
    points2.clear();
    for (0..6) |i| {
        const p = Vec3.init(@floatFromInt(i), 0, 0);
        points1.append(p);
        points2.append(p);
    }
    pruneContactPoints(Vec3.init(0, 1, 0), &points1, &points2);
    try testing.expectEqual(@as(u32, 2), points1.len);
    try testing.expect(points1.get(0).eql(Vec3.init(5, 0, 0))); // Furthest from the center of mass
    try testing.expect(points1.get(1).eql(Vec3.init(0, 0, 0))); // Furthest from the first point

    // The maximum number of contact points (64)
    points1.clear();
    points2.clear();
    for (0..ContactPoints.capacity) |i| {
        const angle = @as(f32, @floatFromInt(i)) * 0.1;
        const p = Vec3.init(@cos(angle), 0.01 * @as(f32, @floatFromInt(i)), @sin(angle));
        points1.append(p);
        points2.append(p);
    }
    pruneContactPoints(Vec3.init(0, 1, 0), &points1, &points2);
    try testing.expect(points1.len <= 4 and points1.len >= 2);
}
