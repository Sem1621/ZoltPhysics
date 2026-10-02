//! Port of: Jolt/Geometry/GJKClosestPoint.h
//! Status: complete
//! Not ported: JPH_GJK_DEBUG (Trace output, DrawState, mGeometry, mOffset)
//!
//! The template functions over the convex objects A and B are comptime generics: `a` and `b` are pointers to convex
//! objects (Jolt's `const A &inA`), i.e. any type with `getSupport(direction: Vec3) Vec3`, see the convex object
//! protocol in ConvexSupport.zig. E.g. Jolt's `gjk.Intersects(sphere, box, tolerance, v)` is
//! `gjk.intersects(&sphere, &box, tolerance, &v)`.
//!
//! The in/out (`io`) parameters and the out parameters stay pointer parameters: Jolt only writes the out parameters
//! on some paths (e.g. `GetClosestPoints` leaves `outPointA` / `outPointB` untouched when the objects are further
//! apart than `inMaxDistSq`, and `CastShape` leaves everything untouched on a miss), and callers rely on the previous
//! values staying in place (EPAPenetrationDepth.castShape falls back to the points of the GJK cast).

const std = @import("std");
const builtin = @import("builtin");
const Core = @import("../Core/Core.zig");
const math = @import("../Math/Math.zig");
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const ClosestPoint = @import("ClosestPoint.zig");
const ConvexSupport = @import("ConvexSupport.zig");
const TransformedConvexObject = ConvexSupport.TransformedConvexObject;
const MinkowskiDifference = ConvexSupport.MinkowskiDifference;

/// Type of the convex object that `Ptr` points to. GJK takes its convex objects by pointer (Jolt's `const A &`).
fn ConvexObject(comptime Ptr: type) type {
    const info = @typeInfo(Ptr);
    if (info != .pointer or info.pointer.size != .one)
        @compileError("expected a pointer to a convex object, got " ++ @typeName(Ptr));
    return info.pointer.child;
}

/// Convex vs convex collision detection
/// Based on: A Fast and Robust GJK Implementation for Collision Detection of Convex Objects - Gino van den Bergen
/// NonCopyable in Jolt. Default construct with `var gjk: GJKClosestPoint = .{};`.
pub const GJKClosestPoint = struct {
    /// Support points on A - B
    y: [4]Vec3 = undefined,
    /// Support point on A
    p: [4]Vec3 = undefined,
    /// Support point on B
    q: [4]Vec3 = undefined,
    /// Number of points in y, p and q that are valid
    num_points: u32 = 0,

    /// Result of `getClosest` (the outV, outVLenSq and outSet out parameters)
    const Closest = struct {
        /// Closest point
        v: Vec3,
        /// |v|^2
        v_len_sq: f32,
        /// Set of points that form the new simplex closest to the origin (bit 1 = y[0], bit 2 = y[1], ...)
        set: u32,
    };

    /// Get new closest point to origin given simplex y of num_points points
    ///
    /// @param prev_v_len_sq Length of |v|^2 from the previous iteration, used as a maximum value when selecting a new closest point.
    ///
    /// If last_point_part_of_closest_feature is true then the last point added will be assumed to be part of the closest feature and the function will do less work.
    ///
    /// @return The new closest point (v), |v|^2 and the set of points that form the new simplex closest to the origin
    /// (bit 1 = y[0], bit 2 = y[1], ...) if a new closest point was found.
    /// Null if the function failed (Jolt returns false and does not modify the output variables).
    fn getClosest(self: *const GJKClosestPoint, comptime last_point_part_of_closest_feature: bool, prev_v_len_sq: f32) ?Closest {
        var set: u32 = undefined;
        var v: Vec3 = undefined;

        switch (self.num_points) {
            1 => {
                // Single point
                set = 0b0001;
                v = self.y[0];
            },

            2 => {
                // Line segment
                const r = ClosestPoint.getClosestPointOnLine(self.y[0], self.y[1]);
                v = r.point;
                set = r.set;
            },

            3 => {
                // Triangle
                const r = ClosestPoint.getClosestPointOnTriangle(self.y[0], self.y[1], self.y[2], .{ .must_include_c = last_point_part_of_closest_feature });
                v = r.point;
                set = r.set;
            },

            4 => {
                // Tetrahedron
                const r = ClosestPoint.getClosestPointOnTetrahedron(self.y[0], self.y[1], self.y[2], self.y[3], .{ .must_include_d = last_point_part_of_closest_feature });
                v = r.point;
                set = r.set;
            },

            else => {
                if (Core.enable_asserts) @panic("GJKClosestPoint: invalid number of points");
                return null;
            },
        }

        const v_len_sq = v.lengthSq();
        if (v_len_sq < prev_v_len_sq) // Note, comparison order important: If v_len_sq is NaN then this expression will be false so we will return false
        {
            // Return closest point
            return .{ .v = v, .v_len_sq = v_len_sq, .set = set };
        }

        // No better match found
        return null;
    }

    // Get max(|Y_0|^2 .. |Y_n|^2)
    fn getMaxYLengthSq(self: *const GJKClosestPoint) f32 {
        var y_len_sq = self.y[0].lengthSq();
        for (1..self.num_points) |i|
            y_len_sq = math.max(y_len_sq, self.y[i].lengthSq());
        return y_len_sq;
    }

    // Remove points that are not in the set, only updates y
    fn updatePointSetY(self: *GJKClosestPoint, set: u32) void {
        var num_points: u32 = 0;
        for (0..self.num_points) |i| {
            if ((set & (@as(u32, 1) << @intCast(i))) != 0) {
                self.y[num_points] = self.y[i];
                num_points += 1;
            }
        }
        self.num_points = num_points;
    }

    // Remove points that are not in the set, only updates p
    fn updatePointSetP(self: *GJKClosestPoint, set: u32) void {
        var num_points: u32 = 0;
        for (0..self.num_points) |i| {
            if ((set & (@as(u32, 1) << @intCast(i))) != 0) {
                self.p[num_points] = self.p[i];
                num_points += 1;
            }
        }
        self.num_points = num_points;
    }

    // Remove points that are not in the set, only updates p and q
    fn updatePointSetPQ(self: *GJKClosestPoint, set: u32) void {
        var num_points: u32 = 0;
        for (0..self.num_points) |i| {
            if ((set & (@as(u32, 1) << @intCast(i))) != 0) {
                self.p[num_points] = self.p[i];
                self.q[num_points] = self.q[i];
                num_points += 1;
            }
        }
        self.num_points = num_points;
    }

    // Remove points that are not in the set, updates y, p and q
    fn updatePointSetYPQ(self: *GJKClosestPoint, set: u32) void {
        var num_points: u32 = 0;
        for (0..self.num_points) |i| {
            if ((set & (@as(u32, 1) << @intCast(i))) != 0) {
                self.y[num_points] = self.y[i];
                self.p[num_points] = self.p[i];
                self.q[num_points] = self.q[i];
                num_points += 1;
            }
        }
        self.num_points = num_points;
    }

    // Calculate closest points on A and B
    fn calculatePointAAndB(self: *const GJKClosestPoint, point_a: *Vec3, point_b: *Vec3) void {
        switch (self.num_points) {
            1 => {
                point_a.* = self.p[0];
                point_b.* = self.q[0];
            },

            2 => {
                const bary = ClosestPoint.getBaryCentricCoordinates(self.y[0], self.y[1]);
                const u = bary.u;
                const v = bary.v;
                point_a.* = self.p[0].mulScalar(u).add(self.p[1].mulScalar(v));
                point_b.* = self.q[0].mulScalar(u).add(self.q[1].mulScalar(v));
            },

            3 => {
                const bary = ClosestPoint.getBaryCentricCoordinatesTriangle(self.y[0], self.y[1], self.y[2]);
                const u = bary.u;
                const v = bary.v;
                const w = bary.w;
                point_a.* = self.p[0].mulScalar(u).add(self.p[1].mulScalar(v)).add(self.p[2].mulScalar(w));
                point_b.* = self.q[0].mulScalar(u).add(self.q[1].mulScalar(v)).add(self.q[2].mulScalar(w));
            },

            4 => {
                if (builtin.mode == .Debug) {
                    @memset(std.mem.asBytes(point_a), 0xcd);
                    @memset(std.mem.asBytes(point_b), 0xcd);
                }
            },

            else => {},
        }
    }

    /// Test if a and b intersect
    ///
    /// @param a The convex object A, must support the getSupport(Vec3) function.
    /// @param b The convex object B, must support the getSupport(Vec3) function.
    /// @param tolerance Minimal distance between objects when the objects are considered to be colliding
    /// @param v is used as initial separating axis (provide a zero vector if you don't know yet)
    ///
    /// @return True if they intersect (in which case v = (0, 0, 0)).
    /// False if they don't intersect in which case v is a separating axis in the direction from A to B (magnitude is meaningless)
    pub fn intersects(self: *GJKClosestPoint, a: anytype, b: anytype, tolerance: f32, v: *Vec3) bool {
        _ = ConvexObject(@TypeOf(a));
        _ = ConvexObject(@TypeOf(b));

        const tolerance_sq = math.square(tolerance);

        // Reset state
        self.num_points = 0;

        // Previous length^2 of v
        var prev_v_len_sq: f32 = math.flt_max;

        while (true) {
            // Get support points for shape A and B in the direction of v
            const p = a.getSupport(v.*);
            const q = b.getSupport(v.negate());

            // Get support point of the minkowski sum A - B of v
            const w = p.sub(q);

            // If the support point sA-B(v) is in the opposite direction as v, then we have found a separating axis and there is no intersection
            if (v.dot(w) < 0.0) {
                // Separating axis found
                return false;
            }

            // Store the point for later use
            self.y[self.num_points] = w;
            self.num_points += 1;

            // Determine the new closest point
            const closest = self.getClosest(true, prev_v_len_sq) orelse return false;
            v.* = closest.v;
            const v_len_sq = closest.v_len_sq; // Length^2 of v
            const set = closest.set; // Set of points that form the new simplex

            // If there are 4 points, the origin is inside the tetrahedron and we're done
            if (set == 0xf) {
                v.* = Vec3.zero();
                return true;
            }

            // If v is very close to zero, we consider this a collision
            if (v_len_sq <= tolerance_sq) {
                v.* = Vec3.zero();
                return true;
            }

            // If v is very small compared to the length of y, we also consider this a collision
            if (v_len_sq <= math.flt_epsilon * self.getMaxYLengthSq()) {
                v.* = Vec3.zero();
                return true;
            }

            // The next separation axis to test is the negative of the closest point of the Minkowski sum to the origin
            // Note: This must be done before terminating as converged since the separating axis is -v
            v.* = v.negate();

            // If the squared length of v is not changing enough, we've converged and there is no collision
            std.debug.assert(prev_v_len_sq >= v_len_sq);
            if (prev_v_len_sq - v_len_sq <= math.flt_epsilon * prev_v_len_sq) {
                // v is a separating axis
                return false;
            }
            prev_v_len_sq = v_len_sq;

            // Update the points of the simplex
            self.updatePointSetY(set);
        }
    }

    /// Get closest points between a and b
    ///
    /// @param a The convex object A, must support the getSupport(Vec3) function.
    /// @param b The convex object B, must support the getSupport(Vec3) function.
    /// @param tolerance The minimal distance between A and B before the objects are considered colliding and processing is terminated.
    /// @param max_dist_sq The maximum squared distance between A and B before the objects are considered infinitely far away and processing is terminated.
    /// @param v Initial guess for the separating axis. Start with any non-zero vector if you don't know.
    ///     If return value is 0, v = (0, 0, 0).
    ///     If the return value is bigger than 0 but smaller than FLT_MAX, v will be the separating axis in the direction from A to B and its length the squared distance between A and B.
    ///     If the return value is FLT_MAX, v will be the separating axis in the direction from A to B and the magnitude of the vector is meaningless.
    /// @param point_a , point_b
    ///     If the return value is 0 the points are invalid.
    ///     If the return value is bigger than 0 but smaller than FLT_MAX these will contain the closest point on A and B.
    ///     If the return value is FLT_MAX the points are invalid.
    ///
    /// @return The squared distance between A and B or FLT_MAX when they are further away than max_dist_sq.
    pub fn getClosestPoints(self: *GJKClosestPoint, a: anytype, b: anytype, tolerance: f32, max_dist_sq: f32, v: *Vec3, point_a: *Vec3, point_b: *Vec3) f32 {
        _ = ConvexObject(@TypeOf(a));
        _ = ConvexObject(@TypeOf(b));

        const tolerance_sq = math.square(tolerance);

        // Reset state
        self.num_points = 0;

        // Length^2 of v
        var v_len_sq = v.lengthSq();

        // Previous length^2 of v
        var prev_v_len_sq: f32 = math.flt_max;

        while (true) {
            // Get support points for shape A and B in the direction of v
            const p = a.getSupport(v.*);
            const q = b.getSupport(v.negate());

            // Get support point of the minkowski sum A - B of v
            const w = p.sub(q);

            const dot = v.dot(w);

            // Test if we have a separation of more than max_dist_sq, in which case we terminate early
            if (dot < 0.0 and dot * dot > v_len_sq * max_dist_sq) {
                if (builtin.mode == .Debug) {
                    @memset(std.mem.asBytes(point_a), 0xcd);
                    @memset(std.mem.asBytes(point_b), 0xcd);
                }
                return math.flt_max;
            }

            // Store the point for later use
            self.y[self.num_points] = w;
            self.p[self.num_points] = p;
            self.q[self.num_points] = q;
            self.num_points += 1;

            const closest = self.getClosest(true, prev_v_len_sq) orelse {
                self.num_points -= 1; // Undo add last point
                break;
            };
            v.* = closest.v;
            v_len_sq = closest.v_len_sq;
            const set = closest.set;

            // If there are 4 points, the origin is inside the tetrahedron and we're done
            if (set == 0xf) {
                v.* = Vec3.zero();
                v_len_sq = 0.0;
                break;
            }

            // Update the points of the simplex
            self.updatePointSetYPQ(set);

            // If v is very close to zero, we consider this a collision
            if (v_len_sq <= tolerance_sq) {
                v.* = Vec3.zero();
                v_len_sq = 0.0;
                break;
            }

            // If v is very small compared to the length of y, we also consider this a collision
            if (v_len_sq <= math.flt_epsilon * self.getMaxYLengthSq()) {
                v.* = Vec3.zero();
                v_len_sq = 0.0;
                break;
            }

            // The next separation axis to test is the negative of the closest point of the Minkowski sum to the origin
            // Note: This must be done before terminating as converged since the separating axis is -v
            v.* = v.negate();

            // If the squared length of v is not changing enough, we've converged and there is no collision
            std.debug.assert(prev_v_len_sq >= v_len_sq);
            if (prev_v_len_sq - v_len_sq <= math.flt_epsilon * prev_v_len_sq) {
                // v is a separating axis
                break;
            }
            prev_v_len_sq = v_len_sq;
        }

        // Get the closest points
        self.calculatePointAAndB(point_a, point_b);

        // Jolt: JPH_ASSERT(ioV.LengthSq() == v_len_sq). Only evaluated when asserts are enabled: it does not hold for
        // NaN input, which Jolt tolerates in release builds.
        if (Core.enable_asserts) std.debug.assert(v.lengthSq() == v_len_sq);
        return v_len_sq;
    }

    /// Get the resulting simplex after the getClosestPoints algorithm finishes.
    /// If it returned a squared distance of 0, the origin will be contained in the simplex.
    /// `out_y`, `out_p` and `out_q` must have room for 4 points.
    /// @return The number of points in the simplex (Jolt's outNumPoints)
    pub fn getClosestPointsSimplex(self: *const GJKClosestPoint, out_y: []Vec3, out_p: []Vec3, out_q: []Vec3) u32 {
        const size = self.num_points;
        @memcpy(out_y[0..size], self.y[0..size]);
        @memcpy(out_p[0..size], self.p[0..size]);
        @memcpy(out_q[0..size], self.q[0..size]);
        return self.num_points;
    }

    /// Test if a ray ray_origin + lambda * ray_direction for lambda e [0, lambda> intersects a
    ///
    /// Code based upon: Ray Casting against General Convex Objects with Application to Continuous Collision Detection - Gino van den Bergen
    ///
    /// @param ray_origin Origin of the ray
    /// @param ray_direction Direction of the ray (lambda * direction determines length)
    /// @param tolerance The minimal distance between the ray and A before it is considered colliding
    /// @param a A convex object that has the getSupport(Vec3) function
    /// @param io_lambda The max fraction along the ray, on output updated with the actual collision fraction.
    ///
    /// @return true if a hit was found, io_lambda is the solution for lambda.
    pub fn castRay(self: *GJKClosestPoint, ray_origin: Vec3, ray_direction: Vec3, tolerance: f32, a: anytype, io_lambda: *f32) bool {
        _ = ConvexObject(@TypeOf(a));

        const tolerance_sq = math.square(tolerance);

        // Reset state
        self.num_points = 0;

        var lambda: f32 = 0.0;
        var x = ray_origin;
        var v = x.sub(a.getSupport(Vec3.zero()));
        var v_len_sq: f32 = math.flt_max;
        var allow_restart = false;

        while (true) {
            // Get new support point
            const p = a.getSupport(v);
            const w = x.sub(p);

            const v_dot_w = v.dot(w);
            if (v_dot_w > 0.0) {
                // If ray and normal are in the same direction, we've passed A and there's no collision
                const v_dot_r = v.dot(ray_direction);
                if (v_dot_r >= -1.0e-18) // Instead of checking >= 0, check with epsilon as we don't want the division below to overflow to infinity as it can cause a float exception
                    return false;

                // Update the lower bound for lambda
                const delta = v_dot_w / v_dot_r;
                const old_lambda = lambda;
                lambda -= delta;

                // If lambda didn't change, we cannot converge any further and we assume a hit
                if (old_lambda == lambda)
                    break;

                // If lambda is bigger or equal than max, we don't have a hit
                if (lambda >= io_lambda.*)
                    return false;

                // Update x to new closest point on the ray
                x = ray_origin.add(ray_direction.mulScalar(lambda));

                // We've shifted x, so reset v_len_sq so that it is not used as early out for GetClosest
                v_len_sq = math.flt_max;

                // We allow rebuilding the simplex once after x changes because the simplex was built
                // for another x and numerical round off builds up as you keep adding points to an
                // existing simplex
                allow_restart = true;
            }

            // Add p to set P: P = P U {p}
            self.p[self.num_points] = p;
            self.num_points += 1;

            // Calculate Y = {x} - P
            for (0..self.num_points) |i|
                self.y[i] = x.sub(self.p[i]);

            // Determine the new closest point from Y to origin
            if (self.getClosest(false, v_len_sq)) |closest| {
                v = closest.v;
                v_len_sq = closest.v_len_sq;
                const set = closest.set; // Set of points that form the new simplex

                if (set == 0xf) {
                    // We're inside the tetrahedron, we have a hit (verify that length of v is 0)
                    std.debug.assert(v_len_sq == 0.0);
                    break;
                }

                // Update the points P to form the new simplex
                // Note: We're not updating Y as Y will shift with x so we have to calculate it every iteration
                self.updatePointSetP(set);

                // Check if x is close enough to a
                if (v_len_sq <= tolerance_sq) {
                    break;
                }
            } else {
                // Only allow 1 restart, if we still can't get a closest point
                // we're so close that we return this as a hit
                if (!allow_restart)
                    break;

                // If we fail to converge, we start again with the last point as simplex
                allow_restart = false;
                self.p[0] = p;
                self.num_points = 1;
                v = x.sub(p);
                v_len_sq = math.flt_max;
                continue;
            }
        }

        // Store hit fraction
        io_lambda.* = lambda;
        return true;
    }

    /// Test if a cast shape a moving from start to lambda * start.getTranslation() + direction where lambda e [0, io_lambda> intersects b
    ///
    /// @param start Start position and orientation of the convex object
    /// @param direction Direction of the sweep (io_lambda * direction determines length)
    /// @param tolerance The minimal distance between A and B before they are considered colliding
    /// @param a The convex object A, must support the getSupport(Vec3) function.
    /// @param b The convex object B, must support the getSupport(Vec3) function.
    /// @param io_lambda The max fraction along the sweep, on output updated with the actual collision fraction.
    ///
    /// @return true if a hit was found, io_lambda is the solution for lambda.
    pub fn castShape(self: *GJKClosestPoint, start: Mat44, direction: Vec3, tolerance: f32, a: anytype, b: anytype, io_lambda: *f32) bool {
        const A = ConvexObject(@TypeOf(a));
        const B = ConvexObject(@TypeOf(b));

        // Transform the shape to be cast to the starting position
        const transformed_a = TransformedConvexObject(A).init(start, a);

        // Calculate the minkowski difference b - a
        // a is moving, so we need to add the back side of b to the front side of a
        const difference = MinkowskiDifference(B, TransformedConvexObject(A)).init(b, &transformed_a);

        // Do a raycast against the Minkowski difference
        return self.castRay(Vec3.zero(), direction, tolerance, &difference, io_lambda);
    }

    /// Test if a cast shape a moving from start to lambda * start.getTranslation() + direction where lambda e [0, io_lambda> intersects b
    /// (the CastShape overload with convex radii and contact information)
    ///
    /// @param start Start position and orientation of the convex object
    /// @param direction Direction of the sweep (io_lambda * direction determines length)
    /// @param tolerance The minimal distance between A and B before they are considered colliding
    /// @param a The convex object A, must support the getSupport(Vec3) function.
    /// @param b The convex object B, must support the getSupport(Vec3) function.
    /// @param convex_radius_a The convex radius of A, this will be added on all sides to pad A.
    /// @param convex_radius_b The convex radius of B, this will be added on all sides to pad B.
    /// @param io_lambda The max fraction along the sweep, on output updated with the actual collision fraction.
    /// @param point_a is the contact point on A (if separating_axis is near zero, this may not be not the deepest point)
    /// @param point_b is the contact point on B (if separating_axis is near zero, this may not be not the deepest point)
    /// @param separating_axis On return this will contain a vector that points from A to B along the smallest distance of separation.
    /// The length of this vector indicates the separation of A and B without their convex radius.
    /// If it is near zero, the direction may not be accurate as the bodies may overlap when lambda = 0.
    ///
    /// @return true if a hit was found, io_lambda is the solution for lambda and point_a, point_b and separating_axis are valid
    /// (on a miss they are not modified).
    pub fn castShapeWithConvexRadius(self: *GJKClosestPoint, start: Mat44, direction: Vec3, tolerance: f32, a: anytype, b: anytype, convex_radius_a: f32, convex_radius_b: f32, io_lambda: *f32, point_a: *Vec3, point_b: *Vec3, separating_axis: *Vec3) bool {
        const A = ConvexObject(@TypeOf(a));
        _ = ConvexObject(@TypeOf(b));

        var tolerance_sq = math.square(tolerance);

        // Calculate how close A and B (without their convex radius) need to be to each other in order for us to consider this a collision
        const sum_convex_radius = convex_radius_a + convex_radius_b;

        // Transform the shape to be cast to the starting position
        const transformed_a = TransformedConvexObject(A).init(start, a);

        // Reset state
        self.num_points = 0;

        var lambda: f32 = 0.0;
        var x = Vec3.zero(); // Since A is already transformed we can start the cast from zero
        var v = b.getSupport(Vec3.zero()).negate().add(transformed_a.getSupport(Vec3.zero())); // See castRay: v = x - a.getSupport(Vec3.zero()) where a is the Minkowski difference b - transformed_a (see castShape above) and x is zero
        var v_len_sq: f32 = math.flt_max;
        var allow_restart = false;

        // Keeps track of separating axis of the previous iteration.
        // Initialized at zero as we don't know if our first v is actually a separating axis.
        var prev_v = Vec3.zero();

        while (true) {
            // Calculate the minkowski difference b - a
            // a is moving, so we need to add the back side of b to the front side of a
            // Keep the support points on A and B separate so that in the end we can calculate a contact point
            const p = transformed_a.getSupport(v.negate());
            const q = b.getSupport(v);
            const w = x.sub(q.sub(p));

            // Difference from article to this code:
            // We did not include the convex radius in p and q in order to be able to calculate a good separating axis at the end of the algorithm.
            // However when moving forward along direction we do need to take this into account so that we keep A and B separated by the sum of their convex radii.
            // From p we have to subtract: convex_radius_a * v / |v|
            // To q we have to add: convex_radius_b * v / |v|
            // This means that to w we have to add: -(convex_radius_a + convex_radius_b) * v / |v|
            // So to v . w we have to add: v . (-(convex_radius_a + convex_radius_b) * v / |v|) = -(convex_radius_a + convex_radius_b) * |v|
            const v_dot_w = v.dot(w) - sum_convex_radius * v.length();
            if (v_dot_w > 0.0) {
                // If ray and normal are in the same direction, we've passed A and there's no collision
                const v_dot_r = v.dot(direction);
                if (v_dot_r >= -1.0e-18) // Instead of checking >= 0, check with epsilon as we don't want the division below to overflow to infinity as it can cause a float exception
                    return false;

                // Update the lower bound for lambda
                const delta = v_dot_w / v_dot_r;
                const old_lambda = lambda;
                lambda -= delta;

                // If lambda didn't change, we cannot converge any further and we assume a hit
                if (old_lambda == lambda)
                    break;

                // If lambda is bigger or equal than max, we don't have a hit
                if (lambda >= io_lambda.*)
                    return false;

                // Update x to new closest point on the ray
                x = direction.mulScalar(lambda);

                // We've shifted x, so reset v_len_sq so that it is not used as early out when GetClosest returns false
                v_len_sq = math.flt_max;

                // Now that we've moved, we know that A and B are not intersecting at lambda = 0, so we can update our tolerance to stop iterating
                // as soon as A and B are convex_radius_a + convex_radius_b apart
                tolerance_sq = math.square(tolerance + sum_convex_radius);

                // We allow rebuilding the simplex once after x changes because the simplex was built
                // for another x and numerical round off builds up as you keep adding points to an
                // existing simplex
                allow_restart = true;
            }

            // Add p to set P, q to set Q: P = P U {p}, Q = Q U {q}
            self.p[self.num_points] = p;
            self.q[self.num_points] = q;
            self.num_points += 1;

            // Calculate Y = {x} - (Q - P)
            for (0..self.num_points) |i|
                self.y[i] = x.sub(self.q[i].sub(self.p[i]));

            // Determine the new closest point from Y to origin
            if (self.getClosest(false, v_len_sq)) |closest| {
                v = closest.v;
                v_len_sq = closest.v_len_sq;
                const set = closest.set; // Set of points that form the new simplex

                if (set == 0xf) {
                    // We're inside the tetrahedron, we have a hit (verify that length of v is 0)
                    std.debug.assert(v_len_sq == 0.0);
                    break;
                }

                // Update the points P and Q to form the new simplex
                // Note: We're not updating Y as Y will shift with x so we have to calculate it every iteration
                self.updatePointSetPQ(set);

                // Check if A and B are touching according to our tolerance
                if (v_len_sq <= tolerance_sq) {
                    break;
                }

                // Store our v to return as separating axis
                prev_v = v;
            } else {
                // Only allow 1 restart, if we still can't get a closest point
                // we're so close that we return this as a hit
                if (!allow_restart) {
                    // The last support point did not produce a closer simplex, remove it so that
                    // the contact points are calculated from the previous valid simplex.
                    self.num_points -= 1;
                    break;
                }

                // If we fail to converge, we start again with the last point as simplex
                allow_restart = false;
                self.p[0] = p;
                self.q[0] = q;
                self.num_points = 1;
                v = x.sub(q);
                v_len_sq = math.flt_max;
                continue;
            }
        }

        // Calculate Y = {x} - (Q - P) again so we can calculate the contact points
        for (0..self.num_points) |i|
            self.y[i] = x.sub(self.q[i].sub(self.p[i]));

        // Calculate the offset we need to apply to A and B to correct for the convex radius
        const normalized_v = v.normalizedOr(Vec3.zero());
        const convex_radius_a_offset = normalized_v.mulScalar(convex_radius_a);
        const convex_radius_b_offset = normalized_v.mulScalar(convex_radius_b);

        // Get the contact point
        // Note that A and B will coincide when lambda > 0. In this case we calculate only B as it is more accurate as it contains less terms.
        switch (self.num_points) {
            1 => {
                point_b.* = self.q[0].add(convex_radius_b_offset);
                point_a.* = if (lambda > 0.0) point_b.* else self.p[0].sub(convex_radius_a_offset);
            },

            2 => {
                const bary = ClosestPoint.getBaryCentricCoordinates(self.y[0], self.y[1]);
                const bu = bary.u;
                const bv = bary.v;
                point_b.* = self.q[0].mulScalar(bu).add(self.q[1].mulScalar(bv)).add(convex_radius_b_offset);
                point_a.* = if (lambda > 0.0) point_b.* else self.p[0].mulScalar(bu).add(self.p[1].mulScalar(bv)).sub(convex_radius_a_offset);
            },

            3, 4 => { // 4: A full simplex, we can't properly determine a contact point! As contact point we take the closest point of the previous iteration.
                const bary = ClosestPoint.getBaryCentricCoordinatesTriangle(self.y[0], self.y[1], self.y[2]);
                const bu = bary.u;
                const bv = bary.v;
                const bw = bary.w;
                point_b.* = self.q[0].mulScalar(bu).add(self.q[1].mulScalar(bv)).add(self.q[2].mulScalar(bw)).add(convex_radius_b_offset);
                point_a.* = if (lambda > 0.0) point_b.* else self.p[0].mulScalar(bu).add(self.p[1].mulScalar(bv)).add(self.p[2].mulScalar(bw)).sub(convex_radius_a_offset);
            },

            else => {},
        }

        // Store separating axis, in case we have a convex radius we can just return v,
        // otherwise v will be very small and we resort to returning previous v as an approximation.
        separating_axis.* = if (sum_convex_radius > 0.0) v.negate() else prev_v.negate();

        // Store hit fraction
        io_lambda.* = lambda;
        return true;
    }
};

const Sphere = @import("Sphere.zig").Sphere;
const AABox = @import("AABox.zig").AABox;
const PointConvexSupport = ConvexSupport.PointConvexSupport;

test "GJKClosestPoint getClosestPoints / getClosestPointsSimplex" {
    // Two separated boxes: the closest points are on the facing faces
    const a = AABox.init(Vec3.init(-1, -1, -1), Vec3.init(1, 1, 1));
    const b = AABox.init(Vec3.init(3, -0.5, -0.5), Vec3.init(4, 0.5, 0.5));
    var gjk: GJKClosestPoint = .{};
    var v = Vec3.init(1, 0, 0);
    var point_a: Vec3 = undefined;
    var point_b: Vec3 = undefined;
    const dist_sq = gjk.getClosestPoints(&a, &b, 1.0e-4, math.large_float, &v, &point_a, &point_b);
    try std.testing.expectApproxEqAbs(@as(f32, 4.0), dist_sq, 1.0e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 1.0), point_a.getX(), 1.0e-5);
    try std.testing.expectApproxEqAbs(@as(f32, 3.0), point_b.getX(), 1.0e-5);
    try std.testing.expect(v.isClose(Vec3.init(2, 0, 0), .{ .max_dist_sq = 1.0e-8 })); // From A to B, length is the distance

    // The simplex: y = p - q for every point
    var y: [4]Vec3 = undefined;
    var p: [4]Vec3 = undefined;
    var q: [4]Vec3 = undefined;
    const num_points = gjk.getClosestPointsSimplex(&y, &p, &q);
    try std.testing.expect(num_points >= 1 and num_points <= 3);
    for (y[0..num_points], p[0..num_points], q[0..num_points]) |yi, pi, qi|
        try std.testing.expect(yi.eql(pi.sub(qi)));

    // Further away than the max distance
    v = Vec3.init(1, 0, 0);
    try std.testing.expectEqual(math.flt_max, gjk.getClosestPoints(&a, &b, 1.0e-4, 1.0, &v, &point_a, &point_b));

    // Overlapping: the origin is in the simplex
    const c = AABox.init(Vec3.init(0.5, -0.5, -0.5), Vec3.init(1.5, 0.5, 0.5));
    v = Vec3.init(1, 0, 0);
    try std.testing.expectEqual(@as(f32, 0.0), gjk.getClosestPoints(&a, &c, 1.0e-4, math.large_float, &v, &point_a, &point_b));
    try std.testing.expect(v.eql(Vec3.zero()));
}

test "GJKClosestPoint castShape" {
    const sphere = Sphere.init(Vec3.zero(), 1.0);
    var gjk: GJKClosestPoint = .{};

    // Hit: the spheres touch after moving 8 of the 20 units
    var lambda: f32 = 1.0 + math.flt_epsilon;
    try std.testing.expect(gjk.castShape(Mat44.translation(Vec3.init(-10, 0, 0)), Vec3.init(20, 0, 0), 1.0e-4, &sphere, &sphere, &lambda));
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), lambda, 1.0e-4);

    // Miss: passing by
    lambda = 1.0 + math.flt_epsilon;
    try std.testing.expect(!gjk.castShape(Mat44.translation(Vec3.init(-10, 2.1, 0)), Vec3.init(20, 0, 0), 1.0e-4, &sphere, &sphere, &lambda));
    try std.testing.expectEqual(1.0 + math.flt_epsilon, lambda);

    // Miss: not far enough
    lambda = 0.3;
    try std.testing.expect(!gjk.castShape(Mat44.translation(Vec3.init(-10, 0, 0)), Vec3.init(20, 0, 0), 1.0e-4, &sphere, &sphere, &lambda));
}

test "GJKClosestPoint castShapeWithConvexRadius" {
    // Two points with a convex radius of 1 are spheres
    const point_a: PointConvexSupport = .{ .point = Vec3.zero() };
    const point_b: PointConvexSupport = .{ .point = Vec3.zero() };
    var gjk: GJKClosestPoint = .{};
    var lambda: f32 = 1.0 + math.flt_epsilon;
    const invalid = Vec3.init(-999, -999, -999);
    var contact_a = invalid;
    var contact_b = invalid;
    var separating_axis = invalid;
    try std.testing.expect(gjk.castShapeWithConvexRadius(Mat44.translation(Vec3.init(-10, 0, 0)), Vec3.init(20, 0, 0), 1.0e-4, &point_a, &point_b, 1.0, 1.0, &lambda, &contact_a, &contact_b, &separating_axis));
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), lambda, 1.0e-4);
    try std.testing.expect(contact_a.isClose(Vec3.init(-1, 0, 0), .{ .max_dist_sq = 1.0e-6 }));
    try std.testing.expect(contact_b.isClose(Vec3.init(-1, 0, 0), .{ .max_dist_sq = 1.0e-6 }));
    try std.testing.expect(separating_axis.normalized().isClose(Vec3.init(1, 0, 0), .{ .max_dist_sq = 1.0e-6 }));

    // On a miss nothing is written
    lambda = 1.0 + math.flt_epsilon;
    contact_a = invalid;
    contact_b = invalid;
    separating_axis = invalid;
    try std.testing.expect(!gjk.castShapeWithConvexRadius(Mat44.translation(Vec3.init(-10, 2.5, 0)), Vec3.init(20, 0, 0), 1.0e-4, &point_a, &point_b, 1.0, 1.0, &lambda, &contact_a, &contact_b, &separating_axis));
    try std.testing.expectEqual(1.0 + math.flt_epsilon, lambda);
    try std.testing.expect(contact_a.eql(invalid) and contact_b.eql(invalid) and separating_axis.eql(invalid));
}

test "GJKClosestPoint castRay" {
    // Ray through a box, starting outside and inside
    const box = AABox.init(Vec3.init(-1, -1, -1), Vec3.init(1, 1, 1));
    var gjk: GJKClosestPoint = .{};
    var lambda: f32 = 1.0;
    try std.testing.expect(gjk.castRay(Vec3.init(-5, 0, 0), Vec3.init(10, 0, 0), 1.0e-4, &box, &lambda));
    try std.testing.expectApproxEqAbs(@as(f32, 0.4), lambda, 1.0e-4);
    lambda = 1.0;
    try std.testing.expect(gjk.castRay(Vec3.zero(), Vec3.init(10, 0, 0), 1.0e-4, &box, &lambda));
    try std.testing.expectEqual(@as(f32, 0.0), lambda);
    lambda = 1.0;
    try std.testing.expect(!gjk.castRay(Vec3.init(-5, 2, 0), Vec3.init(10, 0, 0), 1.0e-4, &box, &lambda));
}
