//! Port of: Jolt/Physics/Collision/CollideSoftBodyVerticesVsTriangles.h
//! Status: complete
//!
//! A helper class that the triangle shapes (TriangleShape, MeshShape, HeightFieldShape) use for
//! `CollideSoftBodyVertices`: `startVertex`, `processTriangle` for every triangle near the vertex, `finishVertex`.
//! Jolt's `struct Visitor : public CollideSoftBodyVerticesVsTriangles` becomes a struct that embeds it as `base`.
//! - The static member `sTriangleThickness` is `CollideSoftBodyVerticesVsTriangles.triangle_thickness`, a `pub var`
//!   (porting guide: static member variables), a global setting like in Jolt.
//! - `FinishVertex` is const in Jolt and writes through the iterator's output pointers (`CollideSoftBodyVertexIterator`
//!   keeps that state behind pointer fields, Rule M).
//! - Members that the C++ constructor leaves uninitialized are `undefined` until `startVertex` / `processTriangle`.

const std = @import("std");
const ClosestPoint = @import("../../Geometry/ClosestPoint.zig");
const Plane = @import("../../Geometry/Plane.zig").Plane;
const math = @import("../../Math/Math.zig");
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const ScaleHelpers = @import("Shape/ScaleHelpers.zig");
const CollideSoftBodyVertexIterator = @import("CollideSoftBodyVertexIterator.zig").CollideSoftBodyVertexIterator;

/// Collision detection helper that collides soft body vertices vs triangles
pub const CollideSoftBodyVerticesVsTriangles = struct {
    /// Triangles are considered to have some thickness. This thickness extends backwards along the negative triangle normal.
    /// Make this value smaller than the smallest 'wall thickness' so that the back side of the triangle doesn't protrude through the other side.
    /// Make this value too small and tunneling is more likely to occur.
    pub var triangle_thickness: f32 = 0.1;

    transform: Mat44,
    inv_transform: Mat44,
    scale: Vec3,
    local_position: Vec3 = undefined,
    v0: Vec3 = undefined,
    v1: Vec3 = undefined,
    v2: Vec3 = undefined,
    closest_point: Vec3 = undefined,
    normal_sign: f32,
    closest_distance_sq: f32 = undefined,
    set: u32 = undefined,

    /// Constructor
    pub fn init(center_of_mass_transform: Mat44, scale: Vec3) CollideSoftBodyVerticesVsTriangles {
        return .{
            .transform = center_of_mass_transform,
            .inv_transform = center_of_mass_transform.inversedRotationTranslation(),
            .scale = scale,
            .normal_sign = if (ScaleHelpers.isInsideOut(scale)) -1.0 else 1.0,
        };
    }

    pub fn startVertex(self: *CollideSoftBodyVerticesVsTriangles, vertex: *const CollideSoftBodyVertexIterator) void {
        self.local_position = self.inv_transform.mulVec3(vertex.getPosition());
        self.closest_distance_sq = math.flt_max;
    }

    pub fn processTriangle(self: *CollideSoftBodyVerticesVsTriangles, v0_in: Vec3, v1_in: Vec3, v2_in: Vec3) void {
        // Apply the scale to the triangle
        const v0 = self.scale.mul(v0_in);
        const v1 = self.scale.mul(v1_in);
        const v2 = self.scale.mul(v2_in);

        // Get the closest point from the vertex to the triangle
        const closest = ClosestPoint.getClosestPointOnTriangle(v0.sub(self.local_position), v1.sub(self.local_position), v2.sub(self.local_position), .{});
        const dist_sq = closest.point.lengthSq();
        if (dist_sq < self.closest_distance_sq) {
            self.v0 = v0;
            self.v1 = v1;
            self.v2 = v2;
            self.closest_point = closest.point;
            self.closest_distance_sq = dist_sq;
            self.set = closest.set;
        }
    }

    pub fn finishVertex(self: *const CollideSoftBodyVerticesVsTriangles, vertex: *const CollideSoftBodyVertexIterator, colliding_shape_index: i32) void {
        if (self.closest_distance_sq < math.flt_max) {
            // Convert triangle to world space
            const v0 = self.transform.mulVec3(self.v0);
            const v1 = self.transform.mulVec3(self.v1);
            const v2 = self.transform.mulVec3(self.v2);
            const triangle_normal = v1.sub(v0).cross(v2.sub(v0)).normalizedOr(Vec3.axisY()).mulScalar(self.normal_sign);

            if (self.set == 0b111) {
                // Closest is interior to the triangle, use plane as collision plane but don't allow more than sTriangleThickness penetration
                // because otherwise a triangle half a level a way will have a huge penetration if it is back facing
                const penetration = triangle_normal.dot(v0.sub(vertex.getPosition()));
                if (penetration < CollideSoftBodyVerticesVsTriangles.triangle_thickness and vertex.updatePenetration(penetration))
                    vertex.setCollision(Plane.fromPointAndNormal(v0, triangle_normal), colliding_shape_index);
            } else {
                // Closest point is on an edge or vertex, use closest point as collision plane
                const closest_point = self.transform.mulVec3(self.local_position.add(self.closest_point));
                const normal = vertex.getPosition().sub(closest_point);
                if (normal.dot(triangle_normal) > 0.0) // Ignore back facing edges
                {
                    const normal_length = normal.length();
                    const penetration = -normal_length;
                    if (vertex.updatePenetration(penetration))
                        vertex.setCollision(Plane.fromPointAndNormal(closest_point, if (normal_length > 0.0) normal.divScalar(normal_length) else triangle_normal), colliding_shape_index);
                }
            }
        }
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the bit exact comparison with Jolt is in ZoltParity/Physics/TrianglesParity.zig)

const testing = std.testing;
const Quat = @import("../../Math/Quat.zig").Quat;

test "CollideSoftBodyVerticesVsTriangles: interior, edge, back facing, thickness, inside out scale" {
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisY(), 0.5), Vec3.init(1, 2, 3));
    const v0 = Vec3.init(0, 0, 0);
    const v1 = Vec3.init(0, 0, 2);
    const v2 = Vec3.init(2, 0, 0);
    const n = 5;
    var positions = [n]Vec3{
        transform.mulVec3(Vec3.init(0.5, -0.05, 0.5)), // Interior, 0.05 below: penetration 0.05
        transform.mulVec3(Vec3.init(0.5, -0.5, 0.5)), // Interior, deeper than the thickness: ignored
        transform.mulVec3(Vec3.init(-0.5, 0.5, 1)), // Closest to the edge v0..v1 in front: penetration -|d|
        transform.mulVec3(Vec3.init(-0.5, -0.5, 1)), // Closest to the edge v0..v1 behind: back facing, ignored
        transform.mulVec3(Vec3.init(5, 5, 5)), // No triangle processed for this vertex
    };
    var inv_masses = [_]f32{1} ** n;
    var planes = [_]Plane{Plane.init(Vec3.zero(), 0.0)} ** n;
    var penetrations = [_]f32{-math.flt_max} ** n;
    var indices = [_]i32{-1} ** n;
    const begin = CollideSoftBodyVertexIterator.init(.init(&positions[0], .{}), .init(&inv_masses[0], .{}), .init(&planes[0], .{}), .init(&penetrations[0], .{}), .init(&indices[0], .{}));

    var collider = CollideSoftBodyVerticesVsTriangles.init(transform, Vec3.one());
    var v = begin;
    for (0..n) |i| {
        collider.startVertex(&v);
        if (i != n - 1) {
            collider.processTriangle(v0.add(Vec3.init(10, 0, 0)), v1.add(Vec3.init(10, 0, 0)), v2.add(Vec3.init(10, 0, 0))); // Further away
            collider.processTriangle(v0, v1, v2);
        }
        collider.finishVertex(&v, 4);
        v.increment();
    }

    try testing.expectApproxEqAbs(@as(f32, 0.05), penetrations[0], 1.0e-6);
    try testing.expectEqual(@as(i32, 4), indices[0]);
    try testing.expect(transform.multiply3x3Transposed(planes[0].getNormal()).isClose(Vec3.axisY(), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectEqual(-math.flt_max, penetrations[1]);
    try testing.expectApproxEqAbs(-@sqrt(@as(f32, 0.5)), penetrations[2], 1.0e-6);
    try testing.expectEqual(@as(i32, 4), indices[2]);
    try testing.expect(transform.multiply3x3Transposed(planes[2].getNormal()).isClose(Vec3.init(-1, 1, 0).normalized(), .{ .max_dist_sq = 1.0e-10 }));
    try testing.expectEqual(-math.flt_max, penetrations[3]);
    try testing.expectEqual(@as(i32, -1), indices[3]);
    try testing.expectEqual(-math.flt_max, penetrations[4]);

    // Inside out scale: the normal flips, the vertex that was in front is behind now
    var inside_out = CollideSoftBodyVerticesVsTriangles.init(transform, Vec3.init(1, -1, 1));
    var penetration: f32 = -math.flt_max;
    var index: i32 = -1;
    var plane = Plane.init(Vec3.zero(), 0.0);
    const one = CollideSoftBodyVertexIterator.init(.init(&positions[2], .{}), .init(&inv_masses[0], .{}), .init(&plane, .{}), .init(&penetration, .{}), .init(&index, .{}));
    inside_out.startVertex(&one);
    inside_out.processTriangle(v0, v1, v2);
    inside_out.finishVertex(&one, 1);
    try testing.expectEqual(@as(i32, -1), index);
    const old = CollideSoftBodyVerticesVsTriangles.triangle_thickness;
    defer CollideSoftBodyVerticesVsTriangles.triangle_thickness = old;
    CollideSoftBodyVerticesVsTriangles.triangle_thickness = 0.6;
    const deep = CollideSoftBodyVertexIterator.init(.init(&positions[1], .{}), .init(&inv_masses[0], .{}), .init(&plane, .{}), .init(&penetration, .{}), .init(&index, .{}));
    collider.startVertex(&deep);
    collider.processTriangle(v0, v1, v2);
    collider.finishVertex(&deep, 2);
    try testing.expectEqual(@as(i32, 2), index); // Accepted with a bigger thickness
    try testing.expectApproxEqAbs(@as(f32, 0.5), penetration, 1.0e-6);
}
