//! Port of: Jolt/Geometry/Sphere.h
//! Status: complete
//!
//! `getSupport` makes a Sphere a convex object for GJK / EPA (see ConvexSupport).

const std = @import("std");
const math = @import("../Math/Math.zig");
const Float3 = @import("../Math/Float3.zig").Float3;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const AABox = @import("AABox.zig").AABox;

pub const Sphere = extern struct {
    center: Float3,
    radius: f32,

    /// Constructor (Sphere(Vec3Arg inCenter, float inRadius))
    pub fn init(center: Vec3, radius: f32) Sphere {
        var result: Sphere = .{ .center = undefined, .radius = radius };
        center.storeFloat3(&result.center);
        return result;
    }

    /// Constructor (Sphere(const Float3 &inCenter, float inRadius))
    pub fn fromFloat3(center: Float3, radius: f32) Sphere {
        return .{ .center = center, .radius = radius };
    }

    /// Calculate the support vector for this convex shape.
    pub fn getSupport(self: Sphere, direction: Vec3) Vec3 {
        const length = direction.length();
        return if (length > 0.0) Vec3.loadFloat3Unsafe(&self.center).add(direction.mulScalar(self.radius / length)) else Vec3.loadFloat3Unsafe(&self.center);
    }

    // Properties
    pub fn getCenter(self: Sphere) Vec3 {
        return Vec3.loadFloat3Unsafe(&self.center);
    }
    pub fn getRadius(self: Sphere) f32 {
        return self.radius;
    }

    /// Test if two spheres overlap (Overlaps(const Sphere &))
    pub fn overlaps(self: Sphere, b: Sphere) bool {
        return Vec3.loadFloat3Unsafe(&self.center).sub(Vec3.loadFloat3Unsafe(&b.center)).lengthSq() <= math.square(self.radius + b.radius);
    }

    /// Check if this sphere overlaps with a box (Overlaps(const AABox &))
    pub fn overlapsAABox(self: Sphere, other: AABox) bool {
        return other.getSqDistanceTo(self.getCenter()) <= math.square(self.radius);
    }

    /// Create the minimal sphere that encapsulates this sphere and point
    pub fn encapsulatePoint(self: *Sphere, point: Vec3) void {
        // Calculate distance between point and center
        var center = self.getCenter();
        const d_vec = point.sub(center);
        const d_sq = d_vec.lengthSq();
        if (d_sq > math.square(self.radius)) {
            // It is further away than radius, we need to widen the sphere
            // The diameter of the new sphere is radius + d, so the new radius is half of that
            const d = @sqrt(d_sq);
            const radius = 0.5 * (self.radius + d);

            // The center needs to shift by new radius - old radius in the direction of d
            center = center.add(d_vec.mulScalar((radius - self.radius) / d));

            // Store new sphere
            center.storeFloat3(&self.center);
            self.radius = radius;
        }
    }
};

test "Sphere" {
    var s = Sphere.init(Vec3.init(1, 2, 3), 2);
    try std.testing.expect(s.getCenter().eql(Vec3.init(1, 2, 3)));
    try std.testing.expectEqual(@as(f32, 2), s.getRadius());
    try std.testing.expect(Sphere.fromFloat3(.init(1, 2, 3), 2).getCenter().eql(s.getCenter()));

    try std.testing.expect(s.getSupport(Vec3.init(0, 10, 0)).eql(Vec3.init(1, 4, 3)));
    try std.testing.expect(s.getSupport(Vec3.zero()).eql(Vec3.init(1, 2, 3)));

    // Touching spheres overlap
    try std.testing.expect(s.overlaps(.init(Vec3.init(1, 5, 3), 1)));
    try std.testing.expect(!s.overlaps(.init(Vec3.init(1, 5.1, 3), 1)));
    try std.testing.expect(s.overlapsAABox(.init(Vec3.init(3, 2, 3), Vec3.init(4, 4, 4))));
    try std.testing.expect(!s.overlapsAABox(.init(Vec3.init(3.1, 2, 3), Vec3.init(4, 4, 4))));

    // A point inside doesn't change the sphere
    s.encapsulatePoint(Vec3.init(2, 2, 3));
    try std.testing.expect(s.getCenter().eql(Vec3.init(1, 2, 3)));
    try std.testing.expectEqual(@as(f32, 2), s.getRadius());

    // A point outside grows the sphere
    s.encapsulatePoint(Vec3.init(1, 2, 9));
    try std.testing.expect(s.getCenter().eql(Vec3.init(1, 2, 5)));
    try std.testing.expectEqual(@as(f32, 4), s.getRadius());
}
