//! Port of: Jolt/Physics/Collision/Shape/PolyhedronSubmergedVolumeCalculator.h
//! Status: complete
//!
//! - The constructor's `(const Vec3 *inPoints, int inPointStride, int inNumPoints, ..., Point *ioBuffer)` becomes
//!   `init(transform, points: StridedPtrConst(Vec3), num_points, surface, buffer: []Point)` (a strided pointer: the
//!   points can be the first member of a larger struct, e.g. ConvexHullShape's points). The buffer must hold
//!   `num_points` entries and outlive the calculator, like in Jolt (Jolt's ConvexShape uses JPH_STACK_ALLOC, Zolt a
//!   fixed array).
//! - The out parameters of the tetrahedron helpers and `GetResult` become returned structs (`VolumeAndCenter`,
//!   `SubmergedVolumeResult`). The helpers are static (they are only non-static with JPH_DEBUG_RENDERER, which draws
//!   the intersection: TODO(debug_renderer), including the `inBaseOffset` constructor parameter and `mBaseOffset`).

const std = @import("std");
const Core = @import("../../../Core/Core.zig");
const StridedPtrConst = @import("../../../Core/StridedPtr.zig").StridedPtrConst;
const Plane = @import("../../../Geometry/Plane.zig").Plane;
const math = @import("../../../Math/Math.zig");
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;

/// This class calculates the intersection between a fluid surface and a polyhedron and returns the submerged volume and its center of buoyancy
/// Construct this class and then one by one add all faces of the polyhedron using the AddFace function. After all faces have been added the result
/// can be gotten through GetResult.
pub const PolyhedronSubmergedVolumeCalculator = struct {
    /// A helper class that contains cached information about a polyhedron vertex
    pub const Point = struct {
        /// World space position of vertex
        position: Vec3,
        /// Signed distance to the surface (> 0 is above, < 0 is below)
        distance_to_surface: f32,
        /// If the point is above the surface (mDistanceToSurface > 0)
        above_surface: bool,
    };

    /// Submerged volume * 6 and center of mass * 4 of a tetrahedron (the out parameters of the sTetrahedronVolume functions)
    pub const VolumeAndCenter = struct {
        volume_times_6: f32,
        center_times_4: Vec3,
    };

    /// The result of getResult (Jolt's out parameters)
    pub const SubmergedVolumeResult = struct {
        submerged_volume: f32,
        center_of_buoyancy: Vec3,
    };

    // The precalculated points for this polyhedron
    points: []const Point,

    // If all points are above/below the surface
    all_below: bool = true,
    all_above: bool = true,

    // The lowest point
    reference_point_idx: u32 = 0,

    // Aggregator for submerged volume and center of buoyancy
    submerged_volume: f32 = 0.0,
    center_of_buoyancy: Vec3 = Vec3.zero(),

    // TODO(debug_renderer): RVec3 mBaseOffset (Base offset used for drawing)

    // Calculate submerged volume * 6 and center of mass * 4 for a tetrahedron with 4 vertices submerged
    // v1 .. v4 are submerged
    fn tetrahedronVolume4(v1: Vec3, v2: Vec3, v3: Vec3, v4: Vec3) VolumeAndCenter {
        // Calculate center of mass and mass of this tetrahedron,
        // see: https://en.wikipedia.org/wiki/Tetrahedron#Volume
        return .{
            .volume_times_6 = math.max(v1.sub(v4).dot(v2.sub(v4).cross(v3.sub(v4))), @as(f32, 0.0)), // All contributions should be positive because we use a reference point that is on the surface of the hull
            .center_times_4 = v1.add(v2).add(v3).add(v4),
        };
    }

    // Get the intersection point with a plane.
    // v1 is d1 distance away from the plane, v2 is d2 distance away from the plane
    fn getPlaneIntersection(v1: Vec3, d1: f32, v2: Vec3, d2: f32) Vec3 {
        if (Core.enable_asserts) std.debug.assert(math.sign(d1) != math.sign(d2)); // Assuming both points are on opposite ends of the plane (NaN distances violate this, Jolt's release build continues)
        const delta = d1 - d2;
        if (@abs(delta) < 1.0e-6)
            return v1 // Parallel to plane, just pick a point
        else
            return v1.add(v2.sub(v1).mulScalar(d1).divScalar(delta));
    }

    // Calculate submerged volume * 6 and center of mass * 4 for a tetrahedron with 1 vertex submerged
    // v1 is submerged, v2 .. v4 are not
    // d1 .. d4 are the distances from the points to the plane
    fn tetrahedronVolume1(v1_in: Vec3, d1: f32, v2_in: Vec3, d2: f32, v3_in: Vec3, d3: f32, v4_in: Vec3, d4: f32) VolumeAndCenter {
        // A tetrahedron with 1 point submerged is cut along 3 edges forming a new tetrahedron
        const v2 = getPlaneIntersection(v1_in, d1, v2_in, d2);
        const v3 = getPlaneIntersection(v1_in, d1, v3_in, d3);
        const v4 = getPlaneIntersection(v1_in, d1, v4_in, d4);

        // TODO(debug_renderer): Draw intersection between tetrahedron and surface (Shape::sDrawSubmergedVolumes)

        return tetrahedronVolume4(v1_in, v2, v3, v4);
    }

    // Calculate submerged volume * 6 and center of mass * 4 for a tetrahedron with 2 vertices submerged
    // v1, v2 are submerged, v3, v4 are not
    // d1 .. d4 are the distances from the points to the plane
    fn tetrahedronVolume2(v1: Vec3, d1: f32, v2: Vec3, d2: f32, v3: Vec3, d3: f32, v4: Vec3, d4: f32) VolumeAndCenter {
        // A tetrahedron with 2 points submerged is cut along 4 edges forming a quad
        const c = getPlaneIntersection(v1, d1, v3, d3);
        const d = getPlaneIntersection(v1, d1, v4, d4);
        const e = getPlaneIntersection(v2, d2, v4, d4);
        const f = getPlaneIntersection(v2, d2, v3, d3);

        // TODO(debug_renderer): Draw intersection between tetrahedron and surface (Shape::sDrawSubmergedVolumes)

        // We pick point c as reference (which is on the cut off surface)
        // This leaves us with three tetrahedrons to sum up (any faces that are in the same plane as c will have zero volume)
        const r1 = tetrahedronVolume4(e, f, v2, c);
        const r2 = tetrahedronVolume4(e, v1, d, c);
        const r3 = tetrahedronVolume4(e, v2, v1, c);

        // Tally up the totals
        const volume_times_6 = r1.volume_times_6 + r2.volume_times_6 + r3.volume_times_6;
        return .{
            .volume_times_6 = volume_times_6,
            .center_times_4 = if (volume_times_6 > 0.0) r1.center_times_4.mulScalar(r1.volume_times_6).add(r2.center_times_4.mulScalar(r2.volume_times_6)).add(r3.center_times_4.mulScalar(r3.volume_times_6)).divScalar(volume_times_6) else Vec3.zero(),
        };
    }

    // Calculate submerged volume * 6 and center of mass * 4 for a tetrahedron with 3 vertices submerged
    // v1, v2, v3 are submerged, v4 is not
    // d1 .. d4 are the distances from the points to the plane
    fn tetrahedronVolume3(v1_in: Vec3, d1: f32, v2_in: Vec3, d2: f32, v3_in: Vec3, d3: f32, v4_in: Vec3, d4: f32) VolumeAndCenter {
        // A tetrahedron with 1 point above the surface is cut along 3 edges forming a new tetrahedron
        const v1 = getPlaneIntersection(v1_in, d1, v4_in, d4);
        const v2 = getPlaneIntersection(v2_in, d2, v4_in, d4);
        const v3 = getPlaneIntersection(v3_in, d3, v4_in, d4);

        // TODO(debug_renderer): Draw intersection between tetrahedron and surface (Shape::sDrawSubmergedVolumes)

        // We first calculate the part that is above the surface
        const dry = tetrahedronVolume4(v1, v2, v3, v4_in);

        // Calculate the total volume
        const total = tetrahedronVolume4(v1_in, v2_in, v3_in, v4_in);

        // From this we can calculate the center and volume of the submerged part
        const volume_times_6 = math.max(total.volume_times_6 - dry.volume_times_6, @as(f32, 0.0));
        return .{
            .volume_times_6 = volume_times_6,
            .center_times_4 = if (volume_times_6 > 0.0) total.center_times_4.mulScalar(total.volume_times_6).sub(dry.center_times_4.mulScalar(dry.volume_times_6)).divScalar(volume_times_6) else Vec3.zero(),
        };
    }

    /// Constructor
    /// @param transform Transform to transform all incoming points with
    /// @param points Array of points that are part of the polyhedron (a strided pointer: inPoints + inPointStride, the
    ///   amount of bytes between each point, which should usually be sizeof(Vec3))
    /// @param num_points The amount of points
    /// @param surface The plane that forms the fluid surface (normal should point up)
    /// @param buffer A temporary buffer of Point's that should have num_points entries and should stay alive while this class is alive
    pub fn init(transform: Mat44, points: StridedPtrConst(Vec3), num_points: u32, surface: Plane, buffer: []Point) PolyhedronSubmergedVolumeCalculator {
        std.debug.assert(buffer.len >= num_points);
        var self: PolyhedronSubmergedVolumeCalculator = .{ .points = buffer[0..num_points] };

        // Convert the points to world space and determine the distance to the surface
        var reference_dist: f32 = math.flt_max;
        for (0..num_points) |p| {
            // Calculate values
            const transformed_point = transform.mulVec3(points.at(@intCast(p)).*);
            const dist = surface.signedDistance(transformed_point);
            const above = dist >= 0.0;

            // Keep track if all are above or below
            self.all_above = self.all_above and above;
            self.all_below = self.all_below and !above;

            // Calculate lowest point, we use this to create tetrahedrons out of all faces
            if (reference_dist > dist) {
                self.reference_point_idx = @intCast(p);
                reference_dist = dist;
            }

            // Store values
            buffer[p] = .{ .position = transformed_point, .distance_to_surface = dist, .above_surface = above };
        }

        return self;
    }

    /// Check if all points are above the surface. Should be used as early out.
    pub fn areAllAbove(self: *const PolyhedronSubmergedVolumeCalculator) bool {
        return self.all_above;
    }

    /// Check if all points are below the surface. Should be used as early out.
    pub fn areAllBelow(self: *const PolyhedronSubmergedVolumeCalculator) bool {
        return self.all_below;
    }

    /// Get the lowest point of the polyhedron. Used to form the 4th vertex to make a tetrahedron out of a polyhedron face.
    pub fn getReferencePointIdx(self: *const PolyhedronSubmergedVolumeCalculator) u32 {
        return self.reference_point_idx;
    }

    /// Add a polyhedron face. Supply the indices of the points that form the face (in counter clockwise order).
    pub fn addFace(self: *PolyhedronSubmergedVolumeCalculator, idx1: u32, idx2: u32, idx3: u32) void {
        std.debug.assert(idx1 != self.reference_point_idx and idx2 != self.reference_point_idx and idx3 != self.reference_point_idx); // A face using the reference point will not contribute to the volume

        // Find the points
        const ref = &self.points[self.reference_point_idx];
        const p1 = &self.points[idx1];
        const p2 = &self.points[idx2];
        const p3 = &self.points[idx3];

        // Determine which vertices are submerged
        const code: u3 = (if (p1.above_surface) @as(u3, 0) else 0b001) | (if (p2.above_surface) @as(u3, 0) else 0b010) | (if (p3.above_surface) @as(u3, 0) else 0b100);

        // Jolt's default case (should not be possible: volume 0, center zero) cannot happen for a 3 bit code
        const r: VolumeAndCenter = switch (code) {
            // One point submerged
            0b000 => tetrahedronVolume1(ref.position, ref.distance_to_surface, p3.position, p3.distance_to_surface, p2.position, p2.distance_to_surface, p1.position, p1.distance_to_surface),

            // Two points submerged
            0b001 => tetrahedronVolume2(ref.position, ref.distance_to_surface, p1.position, p1.distance_to_surface, p3.position, p3.distance_to_surface, p2.position, p2.distance_to_surface),

            // Two points submerged
            0b010 => tetrahedronVolume2(ref.position, ref.distance_to_surface, p2.position, p2.distance_to_surface, p1.position, p1.distance_to_surface, p3.position, p3.distance_to_surface),

            // Two points submerged
            0b100 => tetrahedronVolume2(ref.position, ref.distance_to_surface, p3.position, p3.distance_to_surface, p2.position, p2.distance_to_surface, p1.position, p1.distance_to_surface),

            // Three points submerged
            0b011 => tetrahedronVolume3(ref.position, ref.distance_to_surface, p2.position, p2.distance_to_surface, p1.position, p1.distance_to_surface, p3.position, p3.distance_to_surface),

            // Three points submerged
            0b101 => tetrahedronVolume3(ref.position, ref.distance_to_surface, p1.position, p1.distance_to_surface, p3.position, p3.distance_to_surface, p2.position, p2.distance_to_surface),

            // Three points submerged
            0b110 => tetrahedronVolume3(ref.position, ref.distance_to_surface, p3.position, p3.distance_to_surface, p2.position, p2.distance_to_surface, p1.position, p1.distance_to_surface),

            // Four points submerged
            0b111 => tetrahedronVolume4(ref.position, p3.position, p2.position, p1.position),
        };

        self.submerged_volume += r.volume_times_6;
        self.center_of_buoyancy = self.center_of_buoyancy.add(r.center_times_4.mulScalar(r.volume_times_6));
    }

    /// Call after all faces have been added. Returns the submerged volume and the center of buoyancy for the submerged volume.
    pub fn getResult(self: *const PolyhedronSubmergedVolumeCalculator) SubmergedVolumeResult {
        return .{
            .center_of_buoyancy = if (self.submerged_volume > 0.0) self.center_of_buoyancy.divScalar(4.0 * self.submerged_volume) else Vec3.zero(), // Do this before dividing submerged volume by 6 to get correct weight factor
            .submerged_volume = self.submerged_volume / 6.0,
        };
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests

const testing = std.testing;

/// Unit cube from (0, 0, 0) to (1, 1, 1) with the faces of ConvexShape::GetSubmergedVolume
const test_points = [_]Vec3{
    Vec3.init(-1, -1, -1), Vec3.init(1, -1, -1), Vec3.init(-1, 1, -1), Vec3.init(1, 1, -1),
    Vec3.init(-1, -1, 1),  Vec3.init(1, -1, 1),  Vec3.init(-1, 1, 1),  Vec3.init(1, 1, 1),
};
const test_faces = [_][4]u32{ .{ 0, 2, 3, 1 }, .{ 4, 6, 2, 0 }, .{ 4, 5, 7, 6 }, .{ 1, 3, 7, 5 }, .{ 2, 6, 7, 3 }, .{ 0, 1, 5, 4 } };

fn testCube(transform: Mat44, surface: Plane) PolyhedronSubmergedVolumeCalculator.SubmergedVolumeResult {
    var buffer: [8]PolyhedronSubmergedVolumeCalculator.Point = undefined;
    var calc = PolyhedronSubmergedVolumeCalculator.init(transform, .init(&test_points[0], .{}), 8, surface, &buffer);
    if (calc.areAllAbove()) return .{ .submerged_volume = 0.0, .center_of_buoyancy = Vec3.zero() };
    const reference_point_bit = @as(u32, 1) << @intCast(calc.getReferencePointIdx());
    for (test_faces) |f| {
        const mask = (@as(u32, 1) << @intCast(f[0])) | (@as(u32, 1) << @intCast(f[1])) | (@as(u32, 1) << @intCast(f[2])) | (@as(u32, 1) << @intCast(f[3]));
        if ((mask & reference_point_bit) == 0) {
            calc.addFace(f[0], f[1], f[2]);
            calc.addFace(f[0], f[2], f[3]);
        }
    }
    return calc.getResult();
}

test "PolyhedronSubmergedVolumeCalculator: cube cut by planes" {
    const scale = Mat44.scale(0.5); // Cube of size 1 around the origin

    // All above / all below
    {
        var buffer: [8]PolyhedronSubmergedVolumeCalculator.Point = undefined;
        const above = PolyhedronSubmergedVolumeCalculator.init(scale, .init(&test_points[0], .{}), 8, Plane.fromPointAndNormal(Vec3.init(0, -1, 0), Vec3.axisY()), &buffer);
        try testing.expect(above.areAllAbove() and !above.areAllBelow());
        const below = PolyhedronSubmergedVolumeCalculator.init(scale, .init(&test_points[0], .{}), 8, Plane.fromPointAndNormal(Vec3.init(0, 1, 0), Vec3.axisY()), &buffer);
        try testing.expect(!below.areAllAbove() and below.areAllBelow());
        try testing.expectEqual(@as(u32, 0), below.getReferencePointIdx()); // All points are below, the first lowest point is the reference
        try testing.expectEqual(@as(f32, -0.5), buffer[0].position.getX());
        try testing.expect(!buffer[0].above_surface);
    }

    // Half submerged in each direction: volume 0.5, center of buoyancy in the middle of the submerged half
    const normals = [_]Vec3{ Vec3.axisX(), Vec3.axisY(), Vec3.axisZ(), Vec3.axisX().negate(), Vec3.axisY().negate(), Vec3.axisZ().negate() };
    for (normals) |n| {
        const r = testCube(scale, Plane.fromPointAndNormal(Vec3.zero(), n));
        try testing.expectApproxEqAbs(@as(f32, 0.5), r.submerged_volume, 1.0e-6);
        try testing.expect(r.center_of_buoyancy.isClose(n.mulScalar(-0.25), .{ .max_dist_sq = 1.0e-10 }));
    }

    // Cut through a corner (one point, two and three points submerged per face), the volume grows monotonically
    var previous: f32 = 0.0;
    for (0..20) |i| {
        const d = -0.85 + 0.09 * @as(f32, @floatFromInt(i));
        const r = testCube(scale, Plane.fromPointAndNormal(Vec3.replicate(d), Vec3.replicate(1).normalized()));
        try testing.expect(r.submerged_volume >= previous - 1.0e-6 and r.submerged_volume <= 1.0 + 1.0e-6);
        previous = r.submerged_volume;
    }
    try testing.expect(previous > 0.9);

    // A strided pointer over a larger struct (like ConvexHullShape's points)
    const Padded = struct { position: Vec3, other: u32 };
    var padded: [8]Padded = undefined;
    for (&padded, test_points) |*p, v| p.* = .{ .position = v, .other = 7 };
    var buffer: [8]PolyhedronSubmergedVolumeCalculator.Point = undefined;
    const strided = PolyhedronSubmergedVolumeCalculator.init(scale, .init(&padded[0].position, .{ .stride = @sizeOf(Padded) }), 8, Plane.fromPointAndNormal(Vec3.zero(), Vec3.axisY()), &buffer);
    try testing.expect(!strided.areAllAbove() and !strided.areAllBelow());
    try testing.expect(buffer[7].position.eql(Vec3.replicate(0.5)) and buffer[7].above_surface);
}
