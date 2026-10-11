//! Port of: Jolt/Geometry/EPAPenetrationDepth.h
//! Status: complete
//! Not ported: JPH_EPA_PENETRATION_DEPTH_DEBUG (Trace output), JPH_EPA_CONVEX_BUILDER_DRAW (the drawing calls)
//!
//! Like GJKClosestPoint, the convex objects are passed by pointer (Jolt's `const A &`) and the in/out and out
//! parameters stay pointer parameters, because Jolt only writes them on some paths (e.g. `castShape` keeps the contact
//! points of the GJK cast when the EPA step fails, and leaves everything untouched on a miss).

const std = @import("std");
const Core = @import("../Core/Core.zig");
const math = @import("../Math/Math.zig");
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const ConvexSupport = @import("ConvexSupport.zig");
const AddConvexRadius = ConvexSupport.AddConvexRadius;
const TransformedConvexObject = ConvexSupport.TransformedConvexObject;
const GJKClosestPointFile = @import("GJKClosestPoint.zig");
const GJKClosestPoint = GJKClosestPointFile.GJKClosestPoint;
const ConvexObject = GJKClosestPointFile.ConvexObject;
const EPAConvexHullBuilder = @import("EPAConvexHullBuilder.zig").EPAConvexHullBuilder;

/// Implementation of Expanding Polytope Algorithm as described in:
///
/// Proximity Queries and Penetration Depth Computation on 3D Game Objects - Gino van den Bergen
///
/// The implementation of this algorithm does not completely follow the article, instead of splitting
/// triangles at each edge as in fig. 7 in the article, we build a convex hull (removing any triangles that
/// are facing the new point, thereby avoiding the problem of getting really oblong triangles as mentioned in
/// the article).
///
/// The algorithm roughly works like:
///
/// - Start with a simplex of the Minkowski sum (difference) of two objects that was calculated by GJK
/// - This simplex should contain the origin (or else GJK would have reported: no collision)
/// - In cases where the simplex consists of 1 - 3 points, find some extra support points (of the Minkowski sum) to get to at least 4 points
/// - Convert this into a convex hull with non-zero volume (which includes the origin)
/// - A: Calculate the closest point to the origin for all triangles of the hull and take the closest one
/// - Calculate a new support point (of the Minkowski sum) in this direction and add this point to the convex hull
/// - This will remove all faces that are facing the new point and will create new triangles to fill up the hole
/// - Loop to A until no closer point found
/// - The closest point indicates the position / direction of least penetration
///
/// Default construct with `var epa: EPAPenetrationDepth = .{};`.
pub const EPAPenetrationDepth = struct {
    // Typedefs
    const max_points = EPAConvexHullBuilder.max_points;
    const max_points_to_include_origin_in_hull = 32;
    comptime {
        std.debug.assert(max_points_to_include_origin_in_hull < max_points);
    }

    const Triangle = EPAConvexHullBuilder.Triangle;
    const Points = EPAConvexHullBuilder.Points;

    /// The GJK algorithm, used to start the EPA algorithm
    gjk: GJKClosestPoint = .{},

    /// Tolerance as passed to the GJK algorithm, used for asserting (only exists when Core.enable_asserts, JPH_ENABLE_ASSERTS).
    gjk_tolerance: if (Core.enable_asserts) f32 else void = if (Core.enable_asserts) 0.0 else {},

    /// A list of support points for the EPA algorithm
    const SupportPoints = struct {
        /// List of support points
        y: Points = .empty,
        p: [max_points]Vec3 = undefined,
        q: [max_points]Vec3 = undefined,

        /// Result of `add`: the new support point (w) and its index (Jolt's outIndex)
        const Added = struct {
            w: Vec3,
            index: u32,
        };

        /// Calculate and add new support point to the list of points
        fn add(self: *SupportPoints, a: anytype, b: anytype, direction: Vec3) Added {
            // Get support point of the minkowski sum A - B
            const p = a.getSupport(direction);
            const q = b.getSupport(direction.negate());
            const w = p.sub(q);

            // Store new point
            const index = self.y.len;
            self.y.append(w);
            self.p[index] = p;
            self.q[index] = q;

            return .{ .w = w, .index = index };
        }
    };

    /// Return code for getPenetrationDepthStepGJK
    pub const Status = enum {
        /// Returned if the objects don't collide, in this case point_a / point_b are invalid
        not_colliding,
        /// Returned if the objects penetrate
        colliding,
        /// Returned if the objects penetrate further than the convex radius. In this case you need to call getPenetrationDepthStepEPA to get the actual penetration depth.
        indeterminate,
    };

    /// Calculates penetration depth between two objects, first step of two (the GJK step)
    ///
    /// @param a_excluding_convex_radius Object A without convex radius.
    /// @param b_excluding_convex_radius Object B without convex radius.
    /// @param convex_radius_a Convex radius for A.
    /// @param convex_radius_b Convex radius for B.
    /// @param v Pass in previously returned value or (1, 0, 0). On return this value is changed to direction to move B out of collision along the shortest path (magnitude is meaningless).
    /// @param tolerance Minimal distance before A and B are considered colliding.
    /// @param point_a Position on A that has the least amount of penetration.
    /// @param point_b Position on B that has the least amount of penetration.
    /// Use |point_b - point_a| to get the distance of penetration.
    pub fn getPenetrationDepthStepGJK(self: *EPAPenetrationDepth, a_excluding_convex_radius: anytype, convex_radius_a: f32, b_excluding_convex_radius: anytype, convex_radius_b: f32, tolerance: f32, v: *Vec3, point_a: *Vec3, point_b: *Vec3) Status {
        if (Core.enable_asserts) self.gjk_tolerance = tolerance;

        // Don't supply a zero v, we only want to get points on the hull of the Minkowsky sum and not internal points.
        //
        // Note that if the assert below triggers, it is very likely that you have a MeshShape that contains a degenerate triangle (e.g. a sliver).
        // Go up a couple of levels in the call stack to see if we're indeed testing a triangle and if it is degenerate.
        // If this is the case then fix the triangles you supply to the MeshShape.
        // (Only evaluated when asserts are enabled: degenerate input can violate it, and Jolt continues in release builds.)
        if (Core.enable_asserts) std.debug.assert(!v.isNearZero(.{}));

        // Get closest points
        const combined_radius = convex_radius_a + convex_radius_b;
        const combined_radius_sq = combined_radius * combined_radius;
        const closest_points_dist_sq = self.gjk.getClosestPoints(a_excluding_convex_radius, b_excluding_convex_radius, tolerance, combined_radius_sq, v, point_a, point_b);
        if (closest_points_dist_sq > combined_radius_sq) {
            // No collision
            return .not_colliding;
        }
        if (closest_points_dist_sq > 0.0) {
            // Collision within convex radius, adjust points for convex radius
            const v_len = @sqrt(closest_points_dist_sq); // getClosestPoints function returns |v|^2 when return value < FLT_MAX
            point_a.* = point_a.add(v.mulScalar(convex_radius_a / v_len));
            point_b.* = point_b.sub(v.mulScalar(convex_radius_b / v_len));
            return .colliding;
        }

        return .indeterminate;
    }

    /// Calculates penetration depth between two objects, second step (the EPA step)
    ///
    /// @param a_including_convex_radius Object A with convex radius
    /// @param b_including_convex_radius Object B with convex radius
    /// @param tolerance A factor that determines the accuracy of the result. If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate. Should be bigger or equal to FLT_EPSILON.
    /// @param v Direction to move B out of collision along the shortest path (magnitude is meaningless)
    /// @param point_a Position on A that has the least amount of penetration
    /// @param point_b Position on B that has the least amount of penetration
    /// Use |point_b - point_a| to get the distance of penetration
    ///
    /// @return False if the objects don't collide, in this case point_a / point_b are invalid.
    /// True if the objects penetrate
    pub fn getPenetrationDepthStepEPA(self: *EPAPenetrationDepth, a_including_convex_radius: anytype, b_including_convex_radius: anytype, tolerance: f32, v: *Vec3, point_a: *Vec3, point_b: *Vec3) bool {
        _ = ConvexObject(@TypeOf(a_including_convex_radius));
        _ = ConvexObject(@TypeOf(b_including_convex_radius));

        // Check that the tolerance makes sense (smaller value than this will just result in needless iterations)
        // (Only evaluated when asserts are enabled: Jolt continues in release builds with a smaller tolerance.)
        if (Core.enable_asserts) std.debug.assert(tolerance >= math.flt_epsilon);

        // Fetch the simplex from GJK algorithm
        var support_points: SupportPoints = .{};
        support_points.y.len = self.gjk.getClosestPointsSimplex(&support_points.y.buffer, &support_points.p, &support_points.q);

        // Fill up the amount of support points to 4
        switch (support_points.y.len) {
            1 => {
                // 1 vertex, which must be at the origin, which is useless for our purpose
                if (Core.enable_asserts) std.debug.assert(support_points.y.get(0).isNearZero(.{ .max_dist_sq = math.square(self.gjk_tolerance) }));
                _ = support_points.y.pop();

                // Add support points in 4 directions to form a tetrahedron around the origin
                const p1 = support_points.add(a_including_convex_radius, b_including_convex_radius, Vec3.init(0, 1, 0)).index;
                const p2 = support_points.add(a_including_convex_radius, b_including_convex_radius, Vec3.init(-1, -1, -1)).index;
                const p3 = support_points.add(a_including_convex_radius, b_including_convex_radius, Vec3.init(1, -1, -1)).index;
                const p4 = support_points.add(a_including_convex_radius, b_including_convex_radius, Vec3.init(0, -1, 1)).index;
                std.debug.assert(p1 == 0);
                std.debug.assert(p2 == 1);
                std.debug.assert(p3 == 2);
                std.debug.assert(p4 == 3);
            },

            2 => {
                // Two vertices, create 3 extra by taking perpendicular axis and rotating it around in 120 degree increments
                const axis = support_points.y.get(1).sub(support_points.y.get(0)).normalized();
                const rotation = Mat44.rotation(axis, math.degreesToRadians(120.0));
                const dir1 = axis.getNormalizedPerpendicular();
                const dir2 = rotation.mulVec3(dir1);
                const dir3 = rotation.mulVec3(dir2);
                const p1 = support_points.add(a_including_convex_radius, b_including_convex_radius, dir1).index;
                const p2 = support_points.add(a_including_convex_radius, b_including_convex_radius, dir2).index;
                const p3 = support_points.add(a_including_convex_radius, b_including_convex_radius, dir3).index;
                std.debug.assert(p1 == 2);
                std.debug.assert(p2 == 3);
                std.debug.assert(p3 == 4);
            },

            3, 4 => {
                // We already have enough points
            },

            else => {},
        }

        // Create hull out of the initial points
        std.debug.assert(support_points.y.len >= 3);
        var hull = EPAConvexHullBuilder.init(&support_points.y);
        hull.initialize(0, 1, 2);
        var i: u32 = 3;
        while (i < support_points.y.len) : (i += 1) {
            const facing = hull.findFacingTriangle(support_points.y.get(i));
            if (facing.triangle) |t| {
                var new_triangles: EPAConvexHullBuilder.NewTriangles = .empty;
                if (!hull.addPoint(t, i, math.flt_max, &new_triangles)) {
                    // We can't recover from a failure to add a point to the hull because the old triangles have been unlinked already.
                    // Assume no collision. This can happen if the shapes touch in 1 point (or plane) in which case the hull is degenerate.
                    return false;
                }
            }
        }

        // Loop until we are sure that the origin is inside the hull
        while (true) {
            // Get the next closest triangle
            const t = hull.peekClosestTriangleInQueue();

            // Don't process removed triangles, just free them (because they're in a heap we don't remove them earlier since we would have to rebuild the sorted heap)
            if (t.removed) {
                _ = hull.popClosestTriangleFromQueue();

                // If we run out of triangles, we couldn't include the origin in the hull so there must be very little penetration and we report no collision.
                if (!hull.hasNextTriangle())
                    return false;

                hull.freeTriangle(t);
                continue;
            }

            // If the closest to the triangle is zero or positive, the origin is in the hull and we can proceed to the main algorithm
            if (t.closest_len_sq >= 0.0)
                break;

            // Remove the triangle from the queue before we start adding new ones (which may result in a new closest triangle at the front of the queue)
            _ = hull.popClosestTriangleFromQueue();

            // Add a support point to get the origin inside the hull
            const added = support_points.add(a_including_convex_radius, b_including_convex_radius, t.normal);
            const w = added.w;
            const new_index = added.index;

            // Add the point to the hull, if we fail we terminate and report no collision
            var new_triangles: EPAConvexHullBuilder.NewTriangles = .empty;
            if (!t.isFacing(w) or !hull.addPoint(t, new_index, math.flt_max, &new_triangles))
                return false;

            // The triangle is facing the support point "w" and can now be safely removed
            std.debug.assert(t.removed);
            hull.freeTriangle(t);

            // If we run out of triangles or points, we couldn't include the origin in the hull so there must be very little penetration and we report no collision.
            if (!hull.hasNextTriangle() or support_points.y.len >= max_points_to_include_origin_in_hull)
                return false;
        }

        // Current closest distance to origin
        var closest_dist_sq: f32 = math.flt_max;

        // Remember last good triangle
        var last: ?*Triangle = null;

        // If we want to flip the penetration depth
        var flip_v_sign = false;

        // Loop until closest point found
        main: while (true) {
            // The body of Jolt's do { ... } while (...) loop, a `continue` in Jolt is a `break :body` (it evaluates the loop condition)
            body: {
                // Get closest triangle to the origin
                const t = hull.popClosestTriangleFromQueue();

                // Don't process removed triangles, just free them (because they're in a heap we don't remove them earlier since we would have to rebuild the sorted heap)
                if (t.removed) {
                    hull.freeTriangle(t);
                    break :body;
                }

                // Check if next triangle is further away than closest point, we've found the closest point
                if (t.closest_len_sq >= closest_dist_sq)
                    break :main;

                // Replace last good with this triangle
                if (last) |l|
                    hull.freeTriangle(l);
                last = t;

                // Add support point in direction of normal of the plane
                // Note that the article uses the closest point between the origin and plane, but this always has the exact same direction as the normal (if the origin is behind the plane)
                // and this way we do less calculations and lose less precision
                const added = support_points.add(a_including_convex_radius, b_including_convex_radius, t.normal);
                const w = added.w;
                const new_index = added.index;

                // Project w onto the triangle normal
                const dot = t.normal.dot(w);

                // Check if we just found a separating axis. This can happen if the shape shrunk by convex radius and then expanded by
                // convex radius is bigger then the original shape due to inaccuracies in the shrinking process.
                if (dot < 0.0)
                    return false;

                // Get the distance squared (along normal) to the support point
                const dist_sq = math.square(dot) / t.normal.lengthSq();

                // If the error became small enough, we've converged
                if (dist_sq - t.closest_len_sq < t.closest_len_sq * tolerance) {
                    break :main;
                }

                // Keep track of the minimum distance
                closest_dist_sq = math.min(closest_dist_sq, dist_sq);

                // If the triangle thinks this point is not front facing, we've reached numerical precision and we're done
                if (!t.isFacing(w)) {
                    break :main;
                }

                // Add point to hull
                var new_triangles: EPAConvexHullBuilder.NewTriangles = .empty;
                if (!hull.addPoint(t, new_index, closest_dist_sq, &new_triangles)) {
                    break :main;
                }

                // If the hull is starting to form defects then we're reaching numerical precision and we have to stop
                var has_defect = false;
                for (new_triangles.constSlice()) |nt| {
                    if (nt.isFacingOrigin()) {
                        has_defect = true;
                        break;
                    }
                }
                if (has_defect) {
                    // When the hull has defects it is possible that the origin has been classified on the wrong side of the triangle
                    // so we do an additional check to see if the penetration in the -triangle normal direction is smaller than
                    // the penetration in the triangle normal direction. If so we must flip the sign of the penetration depth.
                    const w2 = a_including_convex_radius.getSupport(t.normal.negate()).sub(b_including_convex_radius.getSupport(t.normal));
                    const dot2 = -t.normal.dot(w2);
                    if (dot2 < dot)
                        flip_v_sign = true;
                    break :main;
                }
            }

            if (!(hull.hasNextTriangle() and support_points.y.len < max_points))
                break;
        }

        // Determine closest points, if last == null it means the hull was a plane so there's no penetration
        const l = last orelse return false;

        // Calculate penetration by getting the vector from the origin to the closest point on the triangle:
        // distance = (centroid - origin) . normal / |normal|, closest = origin + distance * normal / |normal|
        v.* = l.normal.mulScalar(l.centroid.dot(l.normal) / l.normal.lengthSq());

        // If penetration is near zero, treat this as a non collision since we cannot find a good normal
        if (v.isNearZero(.{}))
            return false;

        // Check if we have to flip the sign of the penetration depth
        if (flip_v_sign)
            v.* = v.negate();

        // Use the barycentric coordinates for the closest point to the origin to find the contact points on A and B
        const p0 = support_points.p[l.edge[0].start_idx];
        const p1 = support_points.p[l.edge[1].start_idx];
        const p2 = support_points.p[l.edge[2].start_idx];

        const q0 = support_points.q[l.edge[0].start_idx];
        const q1 = support_points.q[l.edge[1].start_idx];
        const q2 = support_points.q[l.edge[2].start_idx];

        if (l.lambda_relative_to_0) {
            // y0 was the reference vertex
            point_a.* = p0.add(p1.sub(p0).mulScalar(l.lambda[0])).add(p2.sub(p0).mulScalar(l.lambda[1]));
            point_b.* = q0.add(q1.sub(q0).mulScalar(l.lambda[0])).add(q2.sub(q0).mulScalar(l.lambda[1]));
        } else {
            // y1 was the reference vertex
            point_a.* = p1.add(p0.sub(p1).mulScalar(l.lambda[0])).add(p2.sub(p1).mulScalar(l.lambda[1]));
            point_b.* = q1.add(q0.sub(q1).mulScalar(l.lambda[0])).add(q2.sub(q1).mulScalar(l.lambda[1]));
        }

        return true;
    }

    /// This function combines the GJK and EPA steps and is provided as a convenience function.
    /// Note: less performant since you're providing all support functions in one go
    /// Note 2: You need to initialize v, see documentation at getPenetrationDepthStepGJK!
    pub fn getPenetrationDepth(self: *EPAPenetrationDepth, a_excluding_convex_radius: anytype, a_including_convex_radius: anytype, convex_radius_a: f32, b_excluding_convex_radius: anytype, b_including_convex_radius: anytype, convex_radius_b: f32, collision_tolerance_sq: f32, penetration_tolerance: f32, v: *Vec3, point_a: *Vec3, point_b: *Vec3) bool {
        // Check result of collision detection
        switch (self.getPenetrationDepthStepGJK(a_excluding_convex_radius, convex_radius_a, b_excluding_convex_radius, convex_radius_b, collision_tolerance_sq, v, point_a, point_b)) {
            .colliding => return true,

            .not_colliding => return false,

            .indeterminate => return self.getPenetrationDepthStepEPA(a_including_convex_radius, b_including_convex_radius, penetration_tolerance, v, point_a, point_b),
        }
    }

    /// Test if a cast shape a moving from start to lambda * start.getTranslation() + direction where lambda e [0, io_lambda> intersects b
    ///
    /// @param start Start position and orientation of the convex object
    /// @param direction Direction of the sweep (io_lambda * direction determines length)
    /// @param collision_tolerance The minimal distance between A and B before they are considered colliding
    /// @param penetration_tolerance A factor that determines the accuracy of the result. If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate. Should be bigger or equal to FLT_EPSILON.
    /// @param a The convex object A, must support the getSupport(Vec3) function.
    /// @param b The convex object B, must support the getSupport(Vec3) function.
    /// @param convex_radius_a The convex radius of A, this will be added on all sides to pad A.
    /// @param convex_radius_b The convex radius of B, this will be added on all sides to pad B.
    /// @param return_deepest_point If the shapes are initially intersecting this determines if the EPA algorithm will run to find the deepest point
    /// @param io_lambda The max fraction along the sweep, on output updated with the actual collision fraction.
    /// @param point_a is the contact point on A
    /// @param point_b is the contact point on B
    /// @param contact_normal is either the contact normal when the objects are touching or the penetration axis when the objects are penetrating at the start of the sweep (pointing from A to B, length will not be 1)
    ///
    /// @return true if the a hit was found, in which case io_lambda, point_a, point_b and contact_normal are updated.
    pub fn castShape(self: *EPAPenetrationDepth, start: Mat44, direction: Vec3, collision_tolerance: f32, penetration_tolerance: f32, a: anytype, b: anytype, convex_radius_a: f32, convex_radius_b: f32, return_deepest_point: bool, io_lambda: *f32, point_a: *Vec3, point_b: *Vec3, contact_normal: *Vec3) bool {
        const A = ConvexObject(@TypeOf(a));
        const B = ConvexObject(@TypeOf(b));

        if (Core.enable_asserts) self.gjk_tolerance = collision_tolerance;

        // First determine if there's a collision at all
        if (!self.gjk.castShapeWithConvexRadius(start, direction, collision_tolerance, a, b, convex_radius_a, convex_radius_b, io_lambda, point_a, point_b, contact_normal))
            return false;

        // When our contact normal is too small, we don't have an accurate result
        const contact_normal_invalid = contact_normal.isNearZero(.{ .max_dist_sq = math.square(collision_tolerance) });

        if (return_deepest_point and
            io_lambda.* == 0.0 and // Only when lambda = 0 we can have the bodies overlap
            (convex_radius_a + convex_radius_b == 0.0 or // When no convex radius was provided we can never trust contact points at lambda = 0
                contact_normal_invalid))
        {
            // If we're initially intersecting, we need to run the EPA algorithm in order to find the deepest contact point
            const add_convex_a = AddConvexRadius(A).init(a, convex_radius_a);
            const add_convex_b = AddConvexRadius(B).init(b, convex_radius_b);
            const transformed_a = TransformedConvexObject(AddConvexRadius(A)).init(start, &add_convex_a);
            if (!self.getPenetrationDepthStepEPA(&transformed_a, &add_convex_b, penetration_tolerance, contact_normal, point_a, point_b))
                contact_normal.* = direction; // Failed to get the deepest point, use points returned by GJK and use cast direction as normal
        } else if (contact_normal_invalid) {
            // If we weren't able to calculate a contact normal, use the cast direction instead
            contact_normal.* = direction;
        }

        return true;
    }
};

const AABox = @import("AABox.zig").AABox;
const PointConvexSupport = ConvexSupport.PointConvexSupport;

test "EPAPenetrationDepth getPenetrationDepthStepGJK" {
    // Two points with a convex radius of 1 are spheres
    const a: PointConvexSupport = .{ .point = Vec3.zero() };
    var epa: EPAPenetrationDepth = .{};
    var v = Vec3.init(1, 0, 0);
    const invalid = Vec3.init(-999, -999, -999);
    var point_a = invalid;
    var point_b = invalid;

    // Not colliding
    const far: PointConvexSupport = .{ .point = Vec3.init(3, 0, 0) };
    try std.testing.expectEqual(EPAPenetrationDepth.Status.not_colliding, epa.getPenetrationDepthStepGJK(&a, 1.0, &far, 1.0, 1.0e-4, &v, &point_a, &point_b));

    // Colliding within the convex radius: the points are moved onto the spheres
    const near: PointConvexSupport = .{ .point = Vec3.init(1.5, 0, 0) };
    v = Vec3.init(1, 0, 0);
    try std.testing.expectEqual(EPAPenetrationDepth.Status.colliding, epa.getPenetrationDepthStepGJK(&a, 1.0, &near, 1.0, 1.0e-4, &v, &point_a, &point_b));
    try std.testing.expect(point_a.isClose(Vec3.init(1, 0, 0), .{}));
    try std.testing.expect(point_b.isClose(Vec3.init(0.5, 0, 0), .{}));

    // The points coincide: only EPA can tell the penetration depth
    v = Vec3.init(1, 0, 0);
    try std.testing.expectEqual(EPAPenetrationDepth.Status.indeterminate, epa.getPenetrationDepthStepGJK(&a, 1.0, &a, 1.0, 1.0e-4, &v, &point_a, &point_b));
}

test "EPAPenetrationDepth getPenetrationDepthStepEPA" {
    // Two overlapping boxes, without convex radius: the GJK step can't determine the penetration
    const a = AABox.init(Vec3.init(-1, -1, -1), Vec3.init(1, 1, 1));
    const b = AABox.init(Vec3.init(0.5, -0.8, -0.9), Vec3.init(2.5, 0.7, 0.6));
    var epa: EPAPenetrationDepth = .{};
    var v = Vec3.init(1, 0, 0);
    var point_a: Vec3 = undefined;
    var point_b: Vec3 = undefined;
    try std.testing.expectEqual(EPAPenetrationDepth.Status.indeterminate, epa.getPenetrationDepthStepGJK(&a, 0.0, &b, 0.0, 1.0e-4, &v, &point_a, &point_b));
    try std.testing.expect(epa.getPenetrationDepthStepEPA(&a, &b, 1.0e-4, &v, &point_a, &point_b));

    // B needs to move 0.5 along +X to get out of collision
    try std.testing.expect(v.normalized().isClose(Vec3.init(1, 0, 0), .{ .max_dist_sq = 1.0e-6 }));
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), v.length(), 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), point_a.getX(), 1.0e-4);
    try std.testing.expectApproxEqAbs(@as(f32, 0.5), point_b.getX(), 1.0e-4);
}
