//! Port of: Jolt/Geometry/Ellipse.h
//! Status: complete

const std = @import("std");
const math = @import("../Math/Math.zig");
const Float2 = @import("../Math/Float2.zig").Float2;

/// Ellipse centered around the origin
/// @see https://en.wikipedia.org/wiki/Ellipse
pub const Ellipse = struct {
    /// Radius along X-axis
    a: f32,
    /// Radius along Y-axis
    b: f32,

    /// Construct ellipse with radius A along the X-axis and B along the Y-axis
    pub fn init(a: f32, b: f32) Ellipse {
        std.debug.assert(a > 0.0);
        std.debug.assert(b > 0.0);
        return .{ .a = a, .b = b };
    }

    /// Check if point is inside the ellipse
    pub fn isInside(self: Ellipse, point: Float2) bool {
        return math.square(point.x / self.a) + math.square(point.y / self.b) <= 1.0;
    }

    /// Get the closest point on the ellipse to point
    /// Assumes point is outside the ellipse
    /// @see Rotation Joint Limits in Quaternion Space by Gino van den Bergen, section 10.1 in Game Engine Gems 3.
    pub fn getClosestPoint(self: Ellipse, point: Float2) Float2 {
        const a_sq = math.square(self.a);
        const b_sq = math.square(self.b);

        // Equation of ellipse: f(x, y) = (x/a)^2 + (y/b)^2 - 1 = 0    [1]
        // Normal on surface: (df/dx, df/dy) = (2 x / a^2, 2 y / b^2)
        // Closest point (x', y') on ellipse to point (x, y): (x', y') + t (x / a^2, y / b^2) = (x, y)
        // <=> (x', y') = (a^2 x / (t + a^2), b^2 y / (t + b^2))
        // Requiring point to be on ellipse (substituting into [1]): g(t) = (a x / (t + a^2))^2 + (b y / (t + b^2))^2 - 1 = 0

        // Newton Raphson iteration, starting at t = 0
        var t: f32 = 0.0;
        while (true) {
            // Calculate g(t)
            const t_plus_a_sq = t + a_sq;
            const t_plus_b_sq = t + b_sq;
            const gt = math.square(self.a * point.x / t_plus_a_sq) + math.square(self.b * point.y / t_plus_b_sq) - 1.0;

            // Check if g(t) it is close enough to zero
            if (@abs(gt) < 1.0e-6)
                return .init(a_sq * point.x / t_plus_a_sq, b_sq * point.y / t_plus_b_sq);

            // Get derivative dg/dt = g'(t) = -2 (b^2 y^2 / (t + b^2)^3 + a^2 x^2 / (t + a^2)^3)
            const gt_accent = -2.0 *
                (a_sq * math.square(point.x) / math.cubed(t_plus_a_sq) + b_sq * math.square(point.y) / math.cubed(t_plus_b_sq));

            // Calculate t for next iteration: tn+1 = tn - g(t) / g'(t)
            const tn = t - gt / gt_accent;
            t = tn;
        }
    }

    /// Get normal at point (non-normalized vector)
    pub fn getNormal(self: Ellipse, point: Float2) Float2 {
        // Calculated by [d/dx f(x, y), d/dy f(x, y)], where f(x, y) is the ellipse equation from above
        return .init(point.x / math.square(self.a), point.y / math.square(self.b));
    }
};

test "Ellipse normal" {
    const e = Ellipse.init(1, 2);
    try std.testing.expect(e.getNormal(.init(1, 0)).eql(.init(1, 0)));
    try std.testing.expect(e.getNormal(.init(0, 2)).eql(.init(0, 0.5)));
}
