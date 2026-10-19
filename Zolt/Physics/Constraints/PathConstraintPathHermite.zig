//! Port of: Jolt/Physics/Constraints/PathConstraintPathHermite.h, Jolt/Physics/Constraints/PathConstraintPathHermite.cpp
//! Status: complete
//!
//! JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(PathConstraintPathHermite) and
//! JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(PathConstraintPathHermite::Point):
//! `rtti` with Jolt's attributes. `new Point` leaves the three Vec3 indeterminate in C++; Zolt zero fills them
//! (SerializationArchitecture.md D9). JPH_PROFILE_FUNCTION: Zolt has no profiler.
//!
//! - A final PathConstraintPath (pattern A, see PathConstraintPath.zig): `create(allocator)` is `new
//!   PathConstraintPathHermite` (reference count 0, put it in a `Ref` / `RefConst(PathConstraintPath)` through
//!   `asPath()`), `init(allocator)` the default constructor of a path on the stack or in a member (`setEmbedded()` before
//!   references are taken, `deinit()` at the end), `clone(allocator)` Jolt's implicit copy constructor (`new
//!   PathConstraintPathHermite(other)`, reference count 0).
//! - `mPoints` is an `std.ArrayList(Point)` owned by the allocator of the base class, so `addPoint` (Jolt's
//!   `push_back`) returns `Allocator.Error!void`.
//! - `Point` is an extern struct with Jolt's layout (three Vec3 of 16 bytes): SaveBinaryState writes the array with all
//!   its bytes, including the W components of the Vec3s (`Write(const Array<T> &)` writes trivially copyable elements
//!   at once), and RestoreBinaryState reads them back as they are, exactly like Jolt.
//! - The static helpers with out parameters return structs: sCalculatePositionAndTangent (`PositionAndTangent`),
//!   sCalculateClosestPointThroughBisection (`Interval`), sCalculateClosestPointThroughNewtonRaphson (`ClosestPoint`)
//!   and GetIndexAndT (`IndexAndT`).
//! - `int(trunc(inFraction))` follows what Jolt's x86-64 build computes (cvttss2si): NaN and fractions outside the range
//!   of an int give 0x80000000, where Zig's `@intFromFloat` would be undefined; the int arithmetic of GetIndexAndT wraps
//!   like the x86 instructions. Fractions in range convert identically.
//! - The asserts that user input can violate (a looping path whose first and last point are the same, a normal that is
//!   not normalized because the normals of the path are degenerate) only run with `Core.enable_asserts`, so ReleaseFast
//!   keeps Jolt's release behavior.

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const Core = @import("../../Core/Core.zig");
const math = @import("../../Math/Math.zig");
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const PathConstraintPath = @import("PathConstraintPath.zig").PathConstraintPath;

/// Calculated position and tangent of a Cubic Hermite Spline segment (the out parameters of sCalculatePositionAndTangent)
const PositionAndTangent = struct {
    position: Vec3,
    tangent: Vec3,
};

// Calculate position and tangent for a Cubic Hermite Spline segment
fn calculatePositionAndTangent(p1: Vec3, m1: Vec3, p2: Vec3, m2: Vec3, t: f32) PositionAndTangent {
    // Calculate factors for Cubic Hermite Spline
    // See: https://en.wikipedia.org/wiki/Cubic_Hermite_spline
    const t2 = t * t;
    const t3 = t * t2;
    const h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const h10 = t3 - 2.0 * t2 + t;
    const h01 = -2.0 * t3 + 3.0 * t2;
    const h11 = t3 - t2;

    // Calculate d/dt for factors to calculate the tangent
    const ddt_h00 = 6.0 * (t2 - t);
    const ddt_h10 = 3.0 * t2 - 4.0 * t + 1.0;
    const ddt_h01 = -ddt_h00;
    const ddt_h11 = 3.0 * t2 - 2.0 * t;

    return .{
        .position = p1.mulScalar(h00).add(m1.mulScalar(h10)).add(p2.mulScalar(h01)).add(m2.mulScalar(h11)),
        .tangent = p1.mulScalar(ddt_h00).add(m1.mulScalar(ddt_h10)).add(p2.mulScalar(ddt_h01)).add(m2.mulScalar(ddt_h11)),
    };
}

/// The interval [t_min, t_max] of sCalculateClosestPointThroughBisection (its out parameters)
const Interval = struct {
    t_min: f32,
    t_max: f32,
};

// Calculate the closest point to the origin for a Cubic Hermite Spline segment
// This is used to get an estimate for the interval in which the closest point can be found,
// the interval [0, 1] is too big for Newton Raphson to work on because it is solving a 5th degree polynomial which may
// have multiple local minima that are not the root. This happens especially when the path is straight (tangents aligned with inP2 - inP1).
// Based on the bisection method: https://en.wikipedia.org/wiki/Bisection_method
fn calculateClosestPointThroughBisection(p1: Vec3, m1: Vec3, p2: Vec3, m2: Vec3) Interval {
    var out: Interval = .{ .t_min = 0.0, .t_max = 1.0 };

    // To get the closest point of the curve to the origin we need to solve:
    // d/dt P(t) . P(t) = 0 for t, where P(t) is the point on the curve segment
    // Using d/dt (a(t) . b(t)) = d/dt a(t) . b(t) + a(t) . d/dt b(t)
    // See: https://proofwiki.org/wiki/Derivative_of_Dot_Product_of_Vector-Valued_Functions
    // d/dt P(t) . P(t) = 2 P(t) d/dt P(t) = 2 P(t) . Tangent(t)

    // Calculate the derivative at t = 0, we know P(0) = inP1 and Tangent(0) = inM1
    const ddt_min = p1.dot(m1); // Leaving out factor 2, we're only interested in the root
    if (@abs(ddt_min) < 1.0e-6) {
        // Derivative is near zero, we found our root
        out.t_max = 0.0;
        return out;
    }
    const ddt_min_negative = ddt_min < 0.0;

    // Calculate derivative at t = 1, we know P(1) = inP2 and Tangent(1) = inM2
    const ddt_max = p2.dot(m2);
    if (@abs(ddt_max) < 1.0e-6) {
        // Derivative is near zero, we found our root
        out.t_min = 1.0;
        return out;
    }
    const ddt_max_negative = ddt_max < 0.0;

    // If the signs of the derivative are not different, this algorithm can't find the root
    if (ddt_min_negative == ddt_max_negative)
        return out;

    // With 4 iterations we'll get a result accurate to 1 / 2^4 = 0.0625
    for (0..4) |_| {
        const t_mid = 0.5 * (out.t_min + out.t_max);
        const pt = calculatePositionAndTangent(p1, m1, p2, m2, t_mid);
        const ddt_mid = pt.position.dot(pt.tangent);
        if (@abs(ddt_mid) < 1.0e-6) {
            // Derivative is near zero, we found our root
            out.t_min = t_mid;
            out.t_max = t_mid;
            return out;
        }
        const ddt_mid_negative = ddt_mid < 0.0;

        // Update the search interval so that the signs of the derivative at both ends of the interval are still different
        if (ddt_mid_negative == ddt_min_negative)
            out.t_min = t_mid
        else
            out.t_max = t_mid;
    }
    return out;
}

/// The result of sCalculateClosestPointThroughNewtonRaphson: the return value and the out parameter outDistanceSq
const ClosestPoint = struct {
    t: f32,
    distance_sq: f32,
};

// Calculate the closest point to the origin for a Cubic Hermite Spline segment
// Only considers the range t e [inTMin, inTMax] and will stop as soon as the closest point falls outside of that range
fn calculateClosestPointThroughNewtonRaphson(p1: Vec3, m1: Vec3, p2: Vec3, m2: Vec3, t_min: f32, t_max: f32) ClosestPoint {
    // This is the closest position on the curve to the origin that we found
    var position: Vec3 = undefined;

    // Calculate the size of the interval
    const interval = t_max - t_min;

    // Start in the middle of the interval
    var t = 0.5 * (t_min + t_max);

    // Do max 10 iterations to prevent taking too much CPU time
    for (0..10) |_| {
        // Calculate derivative at t, see comment at sCalculateClosestPointThroughBisection for derivation of the equations
        const pt = calculatePositionAndTangent(p1, m1, p2, m2, t);
        position = pt.position;
        const tangent = pt.tangent;
        const ddt = position.dot(tangent); // Leaving out factor 2, we're only interested in the root

        // Calculate derivative of ddt: d^2/dt P(t) . P(t) = d/dt (2 P(t) . Tangent(t))
        // = 2 (d/dt P(t)) . Tangent(t) + P(t) . d/dt Tangent(t)) = 2 (Tangent(t) . Tangent(t) + P(t) . d/dt Tangent(t))
        const d2dt_h00 = 12.0 * t - 6.0;
        const d2dt_h10 = 6.0 * t - 4.0;
        const d2dt_h01 = -d2dt_h00;
        const d2dt_h11 = 6.0 * t - 2.0;
        const ddt_tangent = p1.mulScalar(d2dt_h00).add(m1.mulScalar(d2dt_h10)).add(p2.mulScalar(d2dt_h01)).add(m2.mulScalar(d2dt_h11));
        const d2dt = tangent.dot(tangent) + position.dot(ddt_tangent); // Leaving out factor 2, because we left it out above too

        // If d2dt is zero, the curve is flat and there are multiple t's for which we are closest to the origin, stop now
        if (d2dt == 0.0)
            break;

        // Do a Newton Raphson step
        // See: https://en.wikipedia.org/wiki/Newton%27s_method
        // Clamp against [-interval, interval] to avoid overshooting too much, we're not interested outside the interval
        const delta = math.clamp(-ddt / d2dt, -interval, interval);

        // If we're stepping away further from t e [inTMin, inTMax] stop now
        if ((t > t_max and delta > 0.0) or (t < t_min and delta < 0.0))
            break;

        // If we've converged, stop now
        t += delta;
        if (@abs(delta) < 1.0e-4)
            break;
    }

    // Calculate the distance squared for the origin to the curve
    return .{ .t = t, .distance_sq = position.lengthSq() };
}

/// `int(inValue)` of a float as Jolt's x86-64 build computes it (cvttss2si): truncate, NaN and values outside the range
/// of an int give 0x80000000 (Zig's @intFromFloat is undefined for them)
fn truncToInt(value: f32) i32 {
    if (value >= -2147483648.0 and value < 2147483648.0)
        return @intFromFloat(value);
    return std.math.minInt(i32);
}

/// A path that follows a Hermite spline
pub const PathConstraintPathHermite = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(PathConstraintPathHermite)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "PathConstraintPathHermite",
        .kind = .virtual,
        .bases = &.{PathConstraintPath},
        .attributes = &.{.attr("mPoints", .points)},
    });

    pub const overrides = .{ .getPathMaxFraction, .getClosestPoint, .getPointOnPath, .saveBinaryState, .restoreBinaryState };

    pub const Point = extern struct {
        /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(PathConstraintPathHermite::Point)
        pub const rtti = RTTI.declare(@This(), .{
            .name = "PathConstraintPathHermite::Point",
            .kind = .non_virtual,
            .attributes = &.{
                .attr("mPosition", .position),
                .attr("mTangent", .tangent),
                .attr("mNormal", .normal),
            },
        });

        /// Position on the path
        position: Vec3,
        /// Tangent of the path, does not need to be normalized (in the direction of the path)
        tangent: Vec3,
        /// Normal of the path (together with the tangent along the curve this forms a basis for the constraint)
        normal: Vec3,
    };

    comptime {
        std.debug.assert(@sizeOf(Point) == 48); // sizeof(PathConstraintPathHermite::Point), streamed as raw bytes
    }

    base: PathConstraintPath,

    /// Points on the Hermite spline (owned by base.allocator)
    points: std.ArrayList(Point) = .empty,

    /// Default constructor of a path that is not on the heap (the owner calls `deinit()`)
    pub fn init(allocator: Allocator) PathConstraintPathHermite {
        return .{ .base = .init(PathConstraintPathHermite, allocator) };
    }

    /// new PathConstraintPathHermite (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*PathConstraintPathHermite {
        const self = try allocator.create(PathConstraintPathHermite);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for PathConstraintPath.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor on the heap (`new PathConstraintPathHermite(*this)`): copies the points and the
    /// looping flag, not the reference count (reference count 0)
    pub fn clone(self: *const PathConstraintPathHermite, allocator: Allocator) Allocator.Error!*PathConstraintPathHermite {
        const copy = try allocator.create(PathConstraintPathHermite);
        errdefer allocator.destroy(copy);
        copy.* = .{ .base = .initCopy(self.base.vtable, allocator, &self.base), .points = try self.points.clone(allocator) };
        return copy;
    }

    /// ~PathConstraintPathHermite
    pub fn destruct(self: *PathConstraintPathHermite) void {
        self.points.deinit(self.base.allocator);
    }

    /// Destructor of a path that is not on the heap (embedded / stack), see PathConstraintPath.deinit
    pub fn deinit(self: *PathConstraintPathHermite) void {
        self.base.deinit();
    }

    /// Implicit upcast to the base class
    pub fn asPath(self: *PathConstraintPathHermite) *PathConstraintPath {
        return &self.base;
    }

    /// Implicit upcast to the base class (const)
    pub fn asPathConst(self: *const PathConstraintPathHermite) *const PathConstraintPath {
        return &self.base;
    }

    // See PathConstraintPath::GetPathMaxFraction
    pub fn getPathMaxFraction(self: *const PathConstraintPathHermite) f32 {
        // size_t arithmetic: an empty non looping path wraps around like Jolt's mPoints.size() - 1
        const n = self.points.items.len;
        return @floatFromInt(if (self.base.isLooping()) n else n -% 1);
    }

    // See PathConstraintPath::GetClosestPoint
    pub fn getClosestPoint(self: *const PathConstraintPathHermite, position: Vec3, fraction_hint: f32) f32 {
        _ = fraction_hint;

        const points = self.points.items;
        const num_points: i32 = @truncate(@as(isize, @bitCast(points.len)));

        // Start with last point on the path, in the non-looping case we won't be visiting this point
        var best_dist_sq = points[@intCast(num_points - 1)].position.sub(position).lengthSq();
        var best_t: f32 = @floatFromInt(num_points - 1);

        // Loop over all points
        const max_i = if (self.base.isLooping()) num_points else num_points - 1;
        var i: i32 = 0;
        while (i < max_i) : (i += 1) {
            const p1 = &points[@intCast(i)];
            const p2 = &points[@intCast(@rem(i + 1, num_points))];

            // Make the curve relative to inPosition
            const p1_pos = p1.position.sub(position);
            const p2_pos = p2.position.sub(position);

            // Get distance to p1
            var dist_sq = p1_pos.lengthSq();
            if (dist_sq < best_dist_sq) {
                best_t = @floatFromInt(i);
                best_dist_sq = dist_sq;
            }

            // First find an interval for the closest point so that we can start doing Newton Raphson steps
            const interval = calculateClosestPointThroughBisection(p1_pos, p1.tangent, p2_pos, p2.tangent);

            if (interval.t_min == interval.t_max) {
                // If the function above returned no interval then it found the root already and we can just calculate the distance
                const pt = calculatePositionAndTangent(p1_pos, p1.tangent, p2_pos, p2.tangent, interval.t_min);
                dist_sq = pt.position.lengthSq();
                if (dist_sq < best_dist_sq) {
                    best_t = @as(f32, @floatFromInt(i)) + interval.t_min;
                    best_dist_sq = dist_sq;
                }
            } else {
                // Get closest distance along curve segment
                const closest = calculateClosestPointThroughNewtonRaphson(p1_pos, p1.tangent, p2_pos, p2.tangent, interval.t_min, interval.t_max);
                const t = closest.t;
                dist_sq = closest.distance_sq;
                if (t >= 0.0 and t <= 1.0 and dist_sq < best_dist_sq) {
                    best_t = @as(f32, @floatFromInt(i)) + t;
                    best_dist_sq = dist_sq;
                }
            }
        }

        return best_t;
    }

    // See PathConstraintPath::GetPointOnPath
    pub fn getPointOnPath(self: *const PathConstraintPathHermite, fraction: f32) PathConstraintPath.PointOnPath {
        // Determine which hermite spline segment we need
        const index_and_t = self.getIndexAndT(fraction);
        const index = index_and_t.index;
        const t = index_and_t.t;

        // Get the points on the segment
        const points = self.points.items;
        const p1 = &points[@intCast(index)];
        const p2 = &points[@intCast(@rem(index + 1, @as(i32, @truncate(@as(isize, @bitCast(points.len))))))];

        // Calculate the position and tangent on the path
        const pt = calculatePositionAndTangent(p1.position, p1.tangent, p2.position, p2.tangent, t);
        var out: PathConstraintPath.PointOnPath = undefined;
        out.position = pt.position;
        out.tangent = pt.tangent.normalized();

        // Just linearly interpolate the normal
        const normal = p1.normal.mulScalar(1.0 - t).add(p2.normal.mulScalar(t));

        // Calculate binormal
        out.binormal = normal.cross(out.tangent).normalized();

        // Recalculate normal so it is perpendicular to both (linear interpolation will cause it not to be)
        out.normal = out.tangent.cross(out.binormal);
        if (Core.enable_asserts) std.debug.assert(out.normal.isNormalized(.{}));
        return out;
    }

    /// Adds a point to the path
    pub fn addPoint(self: *PathConstraintPathHermite, position: Vec3, tangent: Vec3, normal: Vec3) Allocator.Error!void {
        try self.points.append(self.base.allocator, .{ .position = position, .tangent = tangent, .normal = normal });
    }

    // See: PathConstraintPath::SaveBinaryState
    pub fn saveBinaryState(self: *const PathConstraintPathHermite, stream: StreamOut) void {
        PathConstraintPath.impl.saveBinaryState(&self.base, stream);

        stream.writeArray(Point, self.points.items);
    }

    // See: PathConstraintPath::RestoreBinaryState (protected in Jolt)
    pub fn restoreBinaryState(self: *PathConstraintPathHermite, stream: StreamIn) Allocator.Error!void {
        try PathConstraintPath.impl.restoreBinaryState(&self.base, stream);

        try stream.readArray(Point, self.base.allocator, &self.points);
    }

    /// The segment and the fraction on it (the out parameters of GetIndexAndT)
    const IndexAndT = struct {
        index: i32,
        t: f32,
    };

    /// Helper function that returns the index of the path segment and the fraction t on the path segment based on the full path fraction
    fn getIndexAndT(self: *const PathConstraintPathHermite, fraction: f32) IndexAndT {
        const points = self.points.items;
        const num_points: i32 = @truncate(@as(isize, @bitCast(points.len)));

        // Start by truncating the fraction to get the index and storing the remainder in t
        var index = truncToInt(@trunc(fraction));
        var t = fraction - @as(f32, @floatFromInt(index));

        if (self.base.isLooping()) {
            if (Core.enable_asserts) std.debug.assert(!points[0].position.isClose(points[points.len - 1].position, .{})); // A looping path should have a different first and last point!

            // Make sure index is positive by adding a multiple of num_points
            if (index < 0)
                index +%= (@divTrunc(-%index, num_points) +% 1) *% num_points;

            // Index needs to be modulo num_points
            index = @rem(index, num_points);
        } else {
            // Clamp against range of points
            if (index < 0) {
                index = 0;
                t = 0.0;
            } else if (index >= num_points - 1) {
                index = num_points - 2;
                t = 1.0;
            }
        }

        return .{ .index = index, .t = t };
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt's TestPathConstraintPathHermite is in ZoltTests/Physics/PathConstraintTests.zig, the parity of the path
// functions in ZoltParity/Physics/ConstraintsParity.zig)

const testing = std.testing;
const Ref = @import("../../Core/Reference.zig").Ref;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;

/// The binary state of a path
fn testSavePath(allocator: Allocator, path: *const PathConstraintPath) ![]u8 {
    var data: std.Io.Writer.Allocating = .init(allocator);
    defer data.deinit();
    var out = StreamOutWrapper.init(&data.writer);
    path.saveBinaryState(out.streamOut());
    return data.toOwnedSlice();
}

/// PathConstraintPath.restoreFromBinaryState of bytes
fn testRestorePath(allocator: Allocator, bytes: []const u8) Allocator.Error!PathConstraintPath.PathResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    return PathConstraintPath.restoreFromBinaryState(allocator, in.streamIn());
}

/// A circle of `num` points in the XZ plane (normal Y)
fn testCircle(path: *PathConstraintPathHermite, num: u32, radius: f32) !void {
    for (0..num) |p| {
        const a = 2.0 * math.pi * @as(f32, @floatFromInt(p)) / @as(f32, @floatFromInt(num));
        const sin_cos = @import("../../Math/Vec4.zig").Vec4.replicate(a).sinCos();
        const s = sin_cos.sin.getX();
        const c = sin_cos.cos.getX();
        try path.addPoint(Vec3.init(radius * c, 0, radius * s), Vec3.init(-s, 0, c).mulScalar(radius * 2.0 * math.pi / @as(f32, @floatFromInt(num))), Vec3.axisY());
    }
}

test "PathConstraintPathHermite: defaults, max fraction, the points" {
    const allocator = testing.allocator;
    var path = PathConstraintPathHermite.init(allocator);
    path.base.setEmbedded();
    defer path.deinit();
    try testing.expect(!path.base.isLooping());
    try testing.expectEqualStrings("PathConstraintPathHermite", path.base.vtable.rtti.name);

    // An empty non looping path: size_t wraps around like in Jolt
    try testing.expectEqual(@as(f32, 18446744073709551615.0), path.asPathConst().getPathMaxFraction());

    try path.addPoint(Vec3.init(0, 0, 0), Vec3.init(1, 0, 0), Vec3.axisY());
    try path.addPoint(Vec3.init(1, 0, 0), Vec3.init(1, 0, 0), Vec3.axisY());
    try path.addPoint(Vec3.init(2, 0, 1), Vec3.init(1, 0, 1), Vec3.axisY());
    try testing.expectEqual(@as(usize, 3), path.points.items.len);
    try testing.expect(path.points.items[2].position.eql(Vec3.init(2, 0, 1)) and path.points.items[2].tangent.eql(Vec3.init(1, 0, 1)) and path.points.items[2].normal.eql(Vec3.axisY()));
    try testing.expectEqual(@as(f32, 2.0), path.asPathConst().getPathMaxFraction());
    path.base.setIsLooping(true);
    try testing.expectEqual(@as(f32, 3.0), path.asPathConst().getPathMaxFraction());
}

test "PathConstraintPathHermite: points on a straight line, closest points, the index and fraction" {
    const allocator = testing.allocator;
    const path = try PathConstraintPathHermite.create(allocator);
    var ref = RefConst(PathConstraintPath).init(path.asPath());
    defer ref.deinit();
    for (0..4) |p|
        try path.addPoint(Vec3.init(@floatFromInt(p), 0, 0), Vec3.init(1, 0, 0), Vec3.axisY());
    const base = ref.get().?;

    // On a straight line with unit tangents the position is the fraction along X, the basis is the tangent X, the binormal
    // normal x tangent = Y x X = -Z and the normal tangent x binormal = X x -Z = Y
    for ([_]f32{ 0.0, 0.25, 1.0, 1.5, 2.75, 3.0 }) |fraction| {
        const point = base.getPointOnPath(fraction);
        try testing.expect(point.position.isClose(Vec3.init(fraction, 0, 0), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(point.tangent.isClose(Vec3.axisX(), .{}));
        try testing.expect(point.binormal.isClose(Vec3.axisZ().negate(), .{}));
        try testing.expect(point.normal.isClose(Vec3.axisY(), .{}));
        try testing.expectApproxEqAbs(fraction, base.getClosestPoint(point.position.add(Vec3.init(0, 0.5, -0.25)), 0.0), 1.0e-4);
    }

    // Outside the range of a non looping path: clamped to the first / last point
    try testing.expect(base.getPointOnPath(-1.5).position.isClose(Vec3.zero(), .{}));
    try testing.expect(base.getPointOnPath(7.25).position.isClose(Vec3.init(3, 0, 0), .{}));
    try testing.expectEqual(@as(f32, 0.0), base.getClosestPoint(Vec3.init(-5, 1, 0), 0.0));
    try testing.expectEqual(@as(f32, 3.0), base.getClosestPoint(Vec3.init(10, 0, 1), 0.0));

    // GetIndexAndT
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 1, .t = 0.5 }, path.getIndexAndT(1.5));
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 0, .t = -0.5 }, path.getIndexAndT(-0.5)); // trunc(-0.5) = 0: not clamped
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 0, .t = 0.0 }, path.getIndexAndT(-1.5));
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 2, .t = 1.0 }, path.getIndexAndT(3.0));
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 2, .t = 1.0 }, path.getIndexAndT(100.0));
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 0, .t = 0.0 }, path.getIndexAndT(1.0e20)); // cvttss2si: 0x80000000 is negative
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 0, .t = 0.0 }, path.getIndexAndT(std.math.nan(f32)));
    path.base.setIsLooping(true);

    // A looping path wraps around in both directions
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 3, .t = 0.5 }, path.getIndexAndT(3.5));
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 1, .t = 0.25 }, path.getIndexAndT(5.25));
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 3, .t = -0.5 }, path.getIndexAndT(-1.5)); // trunc(-1.5) = -1
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 0, .t = 0.0 }, path.getIndexAndT(-8.0));
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 0, .t = 0.0 }, path.getIndexAndT(0.0));
}

test "PathConstraintPathHermite: a looping circle" {
    const allocator = testing.allocator;
    var path = PathConstraintPathHermite.init(allocator);
    path.base.setEmbedded();
    defer path.deinit();
    path.base.setIsLooping(true);
    try testCircle(&path, 8, 2.0);
    const base = path.asPathConst();
    try testing.expectEqual(@as(f32, 8.0), base.getPathMaxFraction());

    // The points lie (almost) on the circle, the tangent follows it, the normal is Y
    for (0..33) |k| {
        const fraction = 0.25 * @as(f32, @floatFromInt(k));
        const point = base.getPointOnPath(fraction);
        try testing.expectApproxEqAbs(@as(f32, 2.0), point.position.length(), 0.01);
        try testing.expect(point.normal.isClose(Vec3.axisY(), .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(@abs(point.tangent.dot(point.position.normalized())) < 0.01);
        try testing.expect(point.tangent.cross(point.binormal).isClose(point.normal, .{ .max_dist_sq = 1.0e-10 }));

        // The closest point of a point slightly outside the circle is that fraction (modulo the length of the path)
        const closest = base.getClosestPoint(point.position.mulScalar(1.1), 0.0);
        const expected = @mod(fraction, 8.0);
        try testing.expect(@abs(closest - expected) < 1.0e-3 or @abs(closest - expected - 8.0) < 1.0e-3 or @abs(closest - expected + 8.0) < 1.0e-3);
    }

    // The fraction wraps around (a negative fraction keeps its negative remainder t: -6.5 is segment 2 at t = -0.5)
    try testing.expect(base.getPointOnPath(9.5).position.isClose(base.getPointOnPath(1.5).position, .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(base.getPointOnPath(-6.0).position.eql(base.getPointOnPath(2.0).position));
    try testing.expectEqual(PathConstraintPathHermite.IndexAndT{ .index = 2, .t = -0.5 }, path.getIndexAndT(-6.5));
}

test "PathConstraintPathHermite: binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;
    var path = PathConstraintPathHermite.init(allocator);
    path.base.setEmbedded();
    defer path.deinit();
    path.base.setIsLooping(true);
    try testCircle(&path, 5, 1.5);

    // The bytes: the hash, mIsLooping, the number of points and the points with all bytes of their Vec3s (W = Z)
    const bytes = try testSavePath(allocator, path.asPathConst());
    defer allocator.free(bytes);
    try testing.expectEqual(@as(usize, 4 + 1 + 4 + 5 * 48), bytes.len);
    try testing.expectEqual(RTTI.hashName("PathConstraintPathHermite"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(u8, 1), bytes[4]);
    try testing.expectEqual(@as(u32, 5), std.mem.readInt(u32, bytes[5..9], .little));
    try testing.expectEqualSlices(u8, std.mem.sliceAsBytes(path.points.items), bytes[9..]);

    // Restore through RegisterTypes.factory
    {
        var result = try testRestorePath(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(PathConstraintPathHermite, result.getPtr().?);
        try testing.expectEqual(@as(u32, 1), restored.base.getRefCount()); // The result holds the reference
        try testing.expect(restored.base.isLooping());
        try testing.expectEqualSlices(u8, std.mem.sliceAsBytes(path.points.items), std.mem.sliceAsBytes(restored.points.items));
        const resaved = try testSavePath(allocator, restored.asPathConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, bytes, resaved);
    }

    // Truncated: Jolt's error
    for ([_]usize{ 0, 3, 4, 5, 8, 9, 50, bytes.len - 1 }) |len| {
        var result = try testRestorePath(allocator, bytes[0..len]);
        defer result.deinit();
        try testing.expectEqualStrings(if (len < 4) "Failed to read type hash" else "Failed to restore object", result.getError());
    }

    // Out of memory while restoring: the error arrives, nothing leaks
    const Ctx = struct {
        fn run(a: Allocator, b: []const u8) !void {
            var result = try testRestorePath(a, b);
            result.deinit();
        }
    };
    try testing.checkAllAllocationFailures(allocator, Ctx.run, .{bytes});

    // The copy constructor copies the points and the looping flag, not the reference count
    path.base.addRef();
    defer path.base.release();
    var copy = Ref(PathConstraintPath).init((try path.clone(allocator)).asPath());
    defer copy.deinit();
    try testing.expectEqual(@as(u32, 1), copy.get().?.getRefCount());
    const copied = virtual.downcast(PathConstraintPathHermite, copy.get().?);
    try testing.expect(copied.base.isLooping());
    try testing.expectEqualSlices(u8, std.mem.sliceAsBytes(path.points.items), std.mem.sliceAsBytes(copied.points.items));
    try testing.expect(copied.points.items.ptr != path.points.items.ptr);
}

test "PathConstraintPathHermite: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var path = RefConst(PathConstraintPath).init((try PathConstraintPathHermite.create(allocator)).asPath());
            defer path.deinit();
            const hermite = virtual.downcast(PathConstraintPathHermite, @constCast(path.get().?));
            for (0..20) |p|
                try hermite.addPoint(Vec3.init(@floatFromInt(p), 0, 0), Vec3.axisX(), Vec3.axisY());
            var copy = Ref(PathConstraintPath).init((try hermite.clone(allocator)).asPath());
            defer copy.deinit();
        }
    };
    try testing.checkAllAllocationFailures(testing.allocator, S.run, .{});
}

const virtual = @import("../../Core/Virtual.zig");
