//! Port of: Jolt/Physics/Collision/ActiveEdges.h
//! Status: complete
//!
//! The namespace `ActiveEdges` is this file (re-exported as `zolt.ActiveEdges`): `ActiveEdges::IsEdgeActive` is
//! `ActiveEdges.isEdgeActive`, `ActiveEdges::FixNormal` is `ActiveEdges.fixNormal`.
//!
//! An active edge is an edge that either has no neighbouring edge or if the angle between the two connecting faces is too large.

const std = @import("std");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const ClosestPoint = @import("../../Geometry/ClosestPoint.zig");

/// Helper function to check if an edge is active or not
/// @param inNormal1 Triangle normal of triangle on the left side of the edge (when looking along the edge from the top)
/// @param inNormal2 Triangle normal of triangle on the right side of the edge
/// @param inEdgeDirection Vector that points along the edge
/// @param inCosThresholdAngle Cosine of the threshold angle (if the angle between the two triangles is bigger than this, the edge is active, note that a concave edge is always inactive)
pub fn isEdgeActive(normal1: Vec3, normal2: Vec3, edge_direction: Vec3, cos_threshold_angle: f32) bool {
    // If normals are opposite the edges are active (the triangles are back to back)
    const cos_angle_normals = normal1.dot(normal2);
    if (cos_angle_normals < -0.999848) // cos(179 degrees)
        return true;

    // Check if concave edge, if so we are not active
    if (normal1.cross(normal2).dot(edge_direction) < 0.0)
        return false;

    // Convex edge, active when angle bigger than threshold
    return cos_angle_normals < cos_threshold_angle;
}

/// Replace normal by triangle normal if a hit is hitting an inactive edge
/// @param inV0 , inV1 , inV2 form the triangle
/// @param inTriangleNormal is the normal of the provided triangle (does not need to be normalized)
/// @param inActiveEdges bit 0 = edge v0..v1 is active, bit 1 = edge v1..v2 is active, bit 2 = edge v2..v0 is active
/// @param inPoint Collision point on the triangle
/// @param inNormal Collision normal on the triangle (does not need to be normalized)
/// @param inMovementDirection Can be zero. This gives an indication of in which direction the motion is to determine if when we hit an inactive edge/triangle we should return the triangle normal.
/// @return Returns inNormal if an active edge was hit, otherwise returns inTriangleNormal
pub fn fixNormal(v0: Vec3, v1: Vec3, v2: Vec3, triangle_normal: Vec3, active_edges: u8, point: Vec3, normal: Vec3, movement_direction: Vec3) Vec3 {
    // Check: All of the edges are active, we have the correct normal already. No need to call this function!
    std.debug.assert(active_edges != 0b111);

    // If inNormal would affect movement less than inTriangleNormal use inNormal
    // This is done since it is really hard to make a distinction between sliding over a horizontal triangulated grid and hitting an edge (in this case you want to use the triangle normal)
    // and sliding over a triangulated grid and grazing a vertical triangle with an inactive edge (in this case using the triangle normal will cause the object to bounce back so we want to use the calculated normal).
    // To solve this we take a movement hint to give an indication of what direction our object is moving. If the edge normal results in less motion difference than the triangle normal we use the edge normal.
    const normal_length = normal.length();
    const triangle_normal_length = triangle_normal.length();
    if (movement_direction.dot(normal) * triangle_normal_length < movement_direction.dot(triangle_normal) * normal_length)
        return normal;

    // Check: None of the edges are active, we need to use the triangle normal
    if (active_edges == 0)
        return triangle_normal;

    // Some edges are active.
    // If normal is parallel to the triangle normal we don't need to check the active edges.
    if (triangle_normal.dot(normal) > 0.999848 * normal_length * triangle_normal_length) // cos(1 degree)
        return normal;

    const epsilon: f32 = 1.0e-4;
    const one_minus_epsilon: f32 = 1.0 - epsilon;

    var colliding_edge: u32 = undefined;

    // Test where the contact point is in the triangle
    const bary = ClosestPoint.getBaryCentricCoordinatesTriangle(v0.sub(point), v1.sub(point), v2.sub(point));
    const u = bary.u;
    const v = bary.v;
    const w = bary.w;
    if (u > one_minus_epsilon) {
        // Colliding with v0, edge 0 or 2 needs to be active
        colliding_edge = 0b101;
    } else if (v > one_minus_epsilon) {
        // Colliding with v1, edge 0 or 1 needs to be active
        colliding_edge = 0b011;
    } else if (w > one_minus_epsilon) {
        // Colliding with v2, edge 1 or 2 needs to be active
        colliding_edge = 0b110;
    } else if (u < epsilon) {
        // Colliding with edge v1, v2, edge 1 needs to be active
        colliding_edge = 0b010;
    } else if (v < epsilon) {
        // Colliding with edge v0, v2, edge 2 needs to be active
        colliding_edge = 0b100;
    } else if (w < epsilon) {
        // Colliding with edge v0, v1, edge 0 needs to be active
        colliding_edge = 0b001;
    } else {
        // Interior hit
        return triangle_normal;
    }

    // If this edge is active, use the provided normal instead of the triangle normal
    return if ((active_edges & colliding_edge) != 0) normal else triangle_normal;
}

test "ActiveEdges.isEdgeActive" {
    const expect = std.testing.expect;
    const cos_5_degrees: f32 = 0.9961947;

    // Coplanar triangles: inactive
    const up = Vec3.init(0, 1, 0);
    const edge = Vec3.init(0, 0, 1);
    try expect(!isEdgeActive(up, up, edge, cos_5_degrees));

    // Back to back triangles: active
    try expect(isEdgeActive(up, up.negate(), edge, cos_5_degrees));

    // A convex edge of 45 degrees (the right triangle slopes down): active, inactive with a big threshold
    const sloped = Vec3.init(1, 1, 0).normalized();
    try expect(isEdgeActive(up, sloped, edge.negate(), cos_5_degrees));
    try expect(!isEdgeActive(up, sloped, edge.negate(), 0.5));

    // The same edge in the other direction is concave: inactive
    try expect(!isEdgeActive(up, sloped, edge, cos_5_degrees));
}

test "ActiveEdges.fixNormal" {
    const expect = std.testing.expect;

    const v0 = Vec3.init(0, 0, 0);
    const v1 = Vec3.init(0, 0, 1);
    const v2 = Vec3.init(1, 0, 0);
    const triangle_normal = Vec3.init(0, 2, 0); // Does not need to be normalized
    const normal = Vec3.init(-1, 1, 0);
    const no_movement = Vec3.zero();

    // No active edges: the triangle normal
    try expect(fixNormal(v0, v1, v2, triangle_normal, 0, Vec3.init(0, 0, 0.5), normal, no_movement).eql(triangle_normal));

    // The movement hint prefers the normal that affects the movement less
    try expect(fixNormal(v0, v1, v2, triangle_normal, 0, Vec3.init(0, 0, 0.5), normal, Vec3.init(1, -1, 0)).eql(normal));

    // A normal parallel to the triangle normal is kept
    const parallel = Vec3.init(0, 1, 0.001);
    try expect(fixNormal(v0, v1, v2, triangle_normal, 0b001, Vec3.init(0, 0, 0.5), parallel, no_movement).eql(parallel));

    // Hit on edge v0..v1 (edge 0): the normal when edge 0 is active
    try expect(fixNormal(v0, v1, v2, triangle_normal, 0b001, Vec3.init(0, 0, 0.5), normal, no_movement).eql(normal));
    try expect(fixNormal(v0, v1, v2, triangle_normal, 0b110, Vec3.init(0, 0, 0.5), normal, no_movement).eql(triangle_normal));

    // Hit on vertex v0: edge 0 or 2
    try expect(fixNormal(v0, v1, v2, triangle_normal, 0b100, v0, normal, no_movement).eql(normal));
    try expect(fixNormal(v0, v1, v2, triangle_normal, 0b010, v0, normal, no_movement).eql(triangle_normal));

    // Interior hit: the triangle normal
    try expect(fixNormal(v0, v1, v2, triangle_normal, 0b011, Vec3.init(0.25, 0, 0.25), normal, no_movement).eql(triangle_normal));
}
