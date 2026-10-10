//! Port of: Jolt/Geometry/Plane.h
//! Status: complete

const std = @import("std");
const Core = @import("../Core/Core.zig");
const DVec3 = @import("../Math/DVec3.zig").DVec3;
const Float4 = @import("../Math/Float4.zig").Float4;
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;

/// An infinite plane described by the formula X . Normal + Constant = 0.
pub const Plane = extern struct {
    // TODO(serialization): JPH_OBJECT_STREAM friend CreateRTTIPlane (JPH_IMPLEMENT_SERIALIZABLE_OUTSIDE_CLASS)

    /// XYZ = normal, W = constant, plane: x . normal + constant = 0
    normal_and_constant: Vec4,

    /// Constructor (Plane(Vec3Arg inNormal, float inConstant))
    pub fn init(normal: Vec3, constant: f32) Plane {
        return .{ .normal_and_constant = Vec4.fromVec3W(normal, constant) };
    }

    /// Constructor (explicit Plane(Vec4Arg inNormalAndConstant))
    pub fn fromVec4(normal_and_constant: Vec4) Plane {
        return .{ .normal_and_constant = normal_and_constant };
    }

    /// Create from point and normal
    pub fn fromPointAndNormal(point: Vec3, normal: Vec3) Plane {
        return fromVec4(Vec4.fromVec3W(normal, -normal.dot(point)));
    }

    /// Create from point and normal, double precision version that more accurately calculates the plane constant
    pub fn fromPointAndNormalDVec3(point: DVec3, normal: Vec3) Plane {
        return fromVec4(Vec4.fromVec3W(normal, -@as(f32, @floatCast(DVec3.fromVec3(normal).dot(point)))));
    }

    /// `fromPointAndNormal` for an RVec3 point: the spelling that works in single and double precision
    pub const fromPointAndNormalRVec3 = if (Core.double_precision) fromPointAndNormalDVec3 else fromPointAndNormal;

    /// Create from 3 counter clockwise points
    pub fn fromPointsCCW(v1: Vec3, v2: Vec3, v3: Vec3) Plane {
        return fromPointAndNormal(v1, v2.sub(v1).cross(v3.sub(v1)).normalized());
    }

    // Properties
    pub fn getNormal(self: Plane) Vec3 {
        return Vec3.fromVec4(self.normal_and_constant);
    }
    pub fn setNormal(self: *Plane, normal: Vec3) void {
        self.normal_and_constant = Vec4.fromVec3W(normal, self.normal_and_constant.getW());
    }
    pub fn getConstant(self: Plane) f32 {
        return self.normal_and_constant.getW();
    }
    pub fn setConstant(self: *Plane, constant: f32) void {
        self.normal_and_constant.setW(constant);
    }

    /// Store as 4 floats
    pub fn storeFloat4(self: Plane, out: *Float4) void {
        self.normal_and_constant.storeFloat4(out);
    }

    /// Offset the plane (positive value means move it in the direction of the plane normal)
    pub fn offset(self: Plane, distance: f32) Plane {
        return fromVec4(self.normal_and_constant.sub(Vec4.fromVec3W(Vec3.zero(), distance)));
    }

    /// Transform the plane by a matrix
    pub fn getTransformed(self: Plane, transform: Mat44) Plane {
        const transformed_normal = transform.multiply3x3(self.getNormal());
        return init(transformed_normal, self.getConstant() - transform.getTranslation().dot(transformed_normal));
    }

    /// Scale the plane, can handle non-uniform and negative scaling
    pub fn scaled(self: Plane, scale: Vec3) Plane {
        const scaled_normal = self.getNormal().div(scale);
        const scaled_normal_length = scaled_normal.length();
        return init(scaled_normal.divScalar(scaled_normal_length), self.getConstant() / scaled_normal_length);
    }

    /// Distance point to plane
    pub fn signedDistance(self: Plane, point: Vec3) f32 {
        return point.dot(self.getNormal()) + self.getConstant();
    }

    /// Project point onto the plane
    pub fn projectPointOnPlane(self: Plane, point: Vec3) Vec3 {
        return point.sub(self.getNormal().mulScalar(self.signedDistance(point)));
    }

    /// Returns intersection point between 3 planes, null if there is none (Jolt returns false and leaves outPoint untouched)
    pub fn intersectPlanes(p1: Plane, p2: Plane, p3: Plane) ?Vec3 {
        // We solve the equation:
        // |ax, ay, az, aw|   | x |   | 0 |
        // |bx, by, bz, bw| * | y | = | 0 |
        // |cx, cy, cz, cw|   | z |   | 0 |
        // | 0,  0,  0,  1|   | 1 |   | 1 |
        // Where normal of plane 1 = (ax, ay, az), plane constant of 1 = aw, normal of plane 2 = (bx, by, bz) etc.
        // This involves inverting the matrix and multiplying it with [0, 0, 0, 1]

        // Fetch the normals and plane constants for the three planes
        const a = p1.normal_and_constant;
        const b = p2.normal_and_constant;
        const c = p3.normal_and_constant;

        // Result is a vector that we have to divide by:
        const denominator = Vec3.fromVec4(a).dot(Vec3.fromVec4(b).cross(Vec3.fromVec4(c)));
        if (denominator == 0.0)
            return null;

        // The numerator is:
        // [aw*(bz*cy-by*cz)+ay*(bw*cz-bz*cw)+az*(by*cw-bw*cy)]
        // [aw*(bx*cz-bz*cx)+ax*(bz*cw-bw*cz)+az*(bw*cx-bx*cw)]
        // [aw*(by*cx-bx*cy)+ax*(bw*cy-by*cw)+ay*(bx*cw-bw*cx)]
        const numerator =
            a.splatW().mul(b.swizzle(.z, .x, .y, .unused).mul(c.swizzle(.y, .z, .x, .unused)).sub(b.swizzle(.y, .z, .x, .unused).mul(c.swizzle(.z, .x, .y, .unused))))
                .add(a.swizzle(.y, .x, .x, .unused).mul(b.swizzle(.w, .z, .w, .unused).mul(c.swizzle(.z, .w, .y, .unused)).sub(b.swizzle(.z, .w, .y, .unused).mul(c.swizzle(.w, .z, .w, .unused)))))
                .add(a.swizzle(.z, .z, .y, .unused).mul(b.swizzle(.y, .w, .x, .unused).mul(c.swizzle(.w, .x, .w, .unused)).sub(b.swizzle(.w, .x, .w, .unused).mul(c.swizzle(.y, .w, .x, .unused)))));

        return Vec3.fromVec4(numerator).divScalar(denominator);
    }
};

test "Plane properties" {
    var p = Plane.init(Vec3.init(0, 1, 0), -2);
    try std.testing.expect(p.getNormal().eql(Vec3.init(0, 1, 0)));
    try std.testing.expectEqual(@as(f32, -2), p.getConstant());
    p.setNormal(Vec3.init(1, 0, 0));
    p.setConstant(3);
    try std.testing.expect(p.normal_and_constant.eql(Vec4.init(1, 0, 0, 3)));

    var f: Float4 = undefined;
    p.storeFloat4(&f);
    try std.testing.expect(f.eql(Float4.init(1, 0, 0, 3)));

    try std.testing.expect(Plane.fromVec4(Vec4.init(0, 0, 1, 4)).offset(1).normal_and_constant.eql(Vec4.init(0, 0, 1, 3)));
    try std.testing.expect(Plane.fromPointsCCW(Vec3.init(0, 0, 1), Vec3.init(1, 0, 1), Vec3.init(0, 1, 1)).normal_and_constant.eql(Vec4.init(0, 0, 1, -1)));
    try std.testing.expect(Plane.fromPointAndNormalDVec3(DVec3.init(0, 0, 1), Vec3.init(0, 0, 1)).normal_and_constant.eql(Vec4.init(0, 0, 1, -1)));
    try std.testing.expect(Plane.fromPointAndNormalRVec3(.init(0, 0, 1), Vec3.init(0, 0, 1)).normal_and_constant.eql(Vec4.init(0, 0, 1, -1)));

    // Point projection and scaling
    const p2 = Plane.fromPointAndNormal(Vec3.init(0, 2, 0), Vec3.init(0, 1, 0));
    try std.testing.expect(p2.projectPointOnPlane(Vec3.init(1, 5, 3)).eql(Vec3.init(1, 2, 3)));
    const s = p2.scaled(Vec3.init(1, 2, 1));
    try std.testing.expect(s.getNormal().eql(Vec3.init(0, 1, 0)));
    try std.testing.expectEqual(@as(f32, 4), -s.getConstant());
    const n = p2.scaled(Vec3.init(1, -2, 1));
    try std.testing.expect(n.getNormal().eql(Vec3.init(0, -1, 0)));
    try std.testing.expectEqual(@as(f32, -4), n.getConstant());
}
