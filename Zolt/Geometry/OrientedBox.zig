//! Port of: Jolt/Geometry/OrientedBox.h, Jolt/Geometry/OrientedBox.cpp
//! Status: complete

const std = @import("std");
const math = @import("../Math/Math.zig");
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const Vec4 = @import("../Math/Vec4.zig").Vec4;
const AABox = @import("AABox.zig").AABox;

/// Oriented box
pub const OrientedBox = extern struct {
    /// Transform that positions and rotates the local space axis aligned box into world space
    orientation: Mat44,
    /// Half extents (half the size of the edge) of the local space axis aligned box
    half_extents: Vec3,

    /// Optional arguments of the overlap tests
    pub const OverlapOptions = struct { epsilon: f32 = 1.0e-6 };

    /// Constructor
    pub fn init(orientation: Mat44, half_extents: Vec3) OrientedBox {
        return .{ .orientation = orientation, .half_extents = half_extents };
    }

    /// Construct from axis aligned box and transform. Only works for rotation/translation matrix (no scaling / shearing).
    pub fn fromAABox(orientation: Mat44, box: AABox) OrientedBox {
        return init(orientation.preTranslated(box.getCenter()), box.getExtent());
    }

    /// Test if oriented box overlaps with axis aligned box each other (Overlaps(const AABox &, float inEpsilon = 1.0e-6f))
    pub fn overlapsAABox(self: OrientedBox, box: AABox, opts: OverlapOptions) bool {
        // Taken from: Real Time Collision Detection - Christer Ericson
        // Chapter 4.4.1, page 103-105.
        // Note that the code is swapped around: A is the aabox and B is the oriented box (this saves us from having to invert the orientation of the oriented box)

        // Convert AABox to center / extent representation
        const a_center = box.getCenter();
        const a_half_extents = box.getExtent();

        // Compute rotation matrix expressing b in a's coordinate frame
        const rot = Mat44.init(self.orientation.getColumn4(0), self.orientation.getColumn4(1), self.orientation.getColumn4(2), self.orientation.getColumn4(3).sub(Vec4.fromVec3W(a_center, 0)));

        return separatingAxisTest(rot, a_half_extents, self.half_extents, opts.epsilon);
    }

    /// Test if two oriented boxes overlap each other (Overlaps(const OrientedBox &, float inEpsilon = 1.0e-6f))
    pub fn overlaps(self: OrientedBox, box: OrientedBox, opts: OverlapOptions) bool {
        // Taken from: Real Time Collision Detection - Christer Ericson
        // Chapter 4.4.1, page 103-105.
        // Note that A is this, B is box

        // Compute rotation matrix expressing b in a's coordinate frame
        const rot = self.orientation.inversedRotationTranslation().mul(box.orientation);

        return separatingAxisTest(rot, self.half_extents, box.half_extents, opts.epsilon);
    }

    /// The part that both Overlaps functions in OrientedBox.cpp share (Jolt repeats the code in both): test the
    /// 15 separating axes of box A (half extents a, at the origin) and box B (half extents b, transformed by rot).
    fn separatingAxisTest(rot: Mat44, a: Vec3, b: Vec3, epsilon_value: f32) bool {
        // Compute common subexpressions. Add in an epsilon term to
        // counteract arithmetic errors when two edges are parallel and
        // their cross product is (near) null (see text for details)
        const epsilon = Vec3.replicate(epsilon_value);
        const abs_r = [3]Vec3{ rot.getAxisX().abs().add(epsilon), rot.getAxisY().abs().add(epsilon), rot.getAxisZ().abs().add(epsilon) };

        // Test axes L = A0, L = A1, L = A2
        var ra: f32 = undefined;
        var rb: f32 = undefined;
        for (0..3) |iu| {
            const i: u32 = @intCast(iu);
            ra = a.getComponent(i);
            rb = b.getComponent(0) * abs_r[0].getComponent(i) + b.getComponent(1) * abs_r[1].getComponent(i) + b.getComponent(2) * abs_r[2].getComponent(i);
            if (@abs(rot.get(i, 3)) > ra + rb) return false;
        }

        // Test axes L = B0, L = B1, L = B2
        for (0..3) |iu| {
            const i: u32 = @intCast(iu);
            ra = a.dot(abs_r[i]);
            rb = b.getComponent(i);
            if (@abs(rot.getTranslation().dot(rot.getColumn3(i))) > ra + rb) return false;
        }

        // Test axis L = A0 x B0
        ra = a.getComponent(1) * abs_r[0].getComponent(2) + a.getComponent(2) * abs_r[0].getComponent(1);
        rb = b.getComponent(1) * abs_r[2].getComponent(0) + b.getComponent(2) * abs_r[1].getComponent(0);
        if (@abs(rot.get(2, 3) * rot.get(1, 0) - rot.get(1, 3) * rot.get(2, 0)) > ra + rb) return false;

        // Test axis L = A0 x B1
        ra = a.getComponent(1) * abs_r[1].getComponent(2) + a.getComponent(2) * abs_r[1].getComponent(1);
        rb = b.getComponent(0) * abs_r[2].getComponent(0) + b.getComponent(2) * abs_r[0].getComponent(0);
        if (@abs(rot.get(2, 3) * rot.get(1, 1) - rot.get(1, 3) * rot.get(2, 1)) > ra + rb) return false;

        // Test axis L = A0 x B2
        ra = a.getComponent(1) * abs_r[2].getComponent(2) + a.getComponent(2) * abs_r[2].getComponent(1);
        rb = b.getComponent(0) * abs_r[1].getComponent(0) + b.getComponent(1) * abs_r[0].getComponent(0);
        if (@abs(rot.get(2, 3) * rot.get(1, 2) - rot.get(1, 3) * rot.get(2, 2)) > ra + rb) return false;

        // Test axis L = A1 x B0
        ra = a.getComponent(0) * abs_r[0].getComponent(2) + a.getComponent(2) * abs_r[0].getComponent(0);
        rb = b.getComponent(1) * abs_r[2].getComponent(1) + b.getComponent(2) * abs_r[1].getComponent(1);
        if (@abs(rot.get(0, 3) * rot.get(2, 0) - rot.get(2, 3) * rot.get(0, 0)) > ra + rb) return false;

        // Test axis L = A1 x B1
        ra = a.getComponent(0) * abs_r[1].getComponent(2) + a.getComponent(2) * abs_r[1].getComponent(0);
        rb = b.getComponent(0) * abs_r[2].getComponent(1) + b.getComponent(2) * abs_r[0].getComponent(1);
        if (@abs(rot.get(0, 3) * rot.get(2, 1) - rot.get(2, 3) * rot.get(0, 1)) > ra + rb) return false;

        // Test axis L = A1 x B2
        ra = a.getComponent(0) * abs_r[2].getComponent(2) + a.getComponent(2) * abs_r[2].getComponent(0);
        rb = b.getComponent(0) * abs_r[1].getComponent(1) + b.getComponent(1) * abs_r[0].getComponent(1);
        if (@abs(rot.get(0, 3) * rot.get(2, 2) - rot.get(2, 3) * rot.get(0, 2)) > ra + rb) return false;

        // Test axis L = A2 x B0
        ra = a.getComponent(0) * abs_r[0].getComponent(1) + a.getComponent(1) * abs_r[0].getComponent(0);
        rb = b.getComponent(1) * abs_r[2].getComponent(2) + b.getComponent(2) * abs_r[1].getComponent(2);
        if (@abs(rot.get(1, 3) * rot.get(0, 0) - rot.get(0, 3) * rot.get(1, 0)) > ra + rb) return false;

        // Test axis L = A2 x B1
        ra = a.getComponent(0) * abs_r[1].getComponent(1) + a.getComponent(1) * abs_r[1].getComponent(0);
        rb = b.getComponent(0) * abs_r[2].getComponent(2) + b.getComponent(2) * abs_r[0].getComponent(2);
        if (@abs(rot.get(1, 3) * rot.get(0, 1) - rot.get(0, 3) * rot.get(1, 1)) > ra + rb) return false;

        // Test axis L = A2 x B2
        ra = a.getComponent(0) * abs_r[2].getComponent(1) + a.getComponent(1) * abs_r[2].getComponent(0);
        rb = b.getComponent(0) * abs_r[1].getComponent(2) + b.getComponent(1) * abs_r[0].getComponent(2);
        if (@abs(rot.get(1, 3) * rot.get(0, 2) - rot.get(0, 3) * rot.get(1, 2)) > ra + rb) return false;

        // Since no separating axis is found, the boxes must be intersecting
        return true;
    }
};

test "OrientedBox" {
    const box = AABox.init(Vec3.init(-1, -1, -1), Vec3.init(1, 1, 1));

    // Axis aligned oriented box, touching and separated
    const touching = OrientedBox.fromAABox(Mat44.translation(Vec3.init(2, 0, 0)), box);
    try std.testing.expect(touching.half_extents.eql(Vec3.one()));
    try std.testing.expect(touching.orientation.getTranslation().eql(Vec3.init(2, 0, 0)));
    try std.testing.expect(touching.overlapsAABox(box, .{}));
    const separated = OrientedBox.init(Mat44.translation(Vec3.init(2.1, 0, 0)), Vec3.one());
    try std.testing.expect(!separated.overlapsAABox(box, .{}));

    // Rotated by 45 degrees around Z, the corner reaches sqrt(2) further
    const rotated = OrientedBox.init(Mat44.rotationZ(0.25 * math.pi).postTranslated(Vec3.init(2.3, 0, 0)), Vec3.one());
    try std.testing.expect(rotated.overlapsAABox(box, .{}));
    try std.testing.expect(!OrientedBox.init(Mat44.rotationZ(0.25 * math.pi).postTranslated(Vec3.init(2.5, 0, 0)), Vec3.one()).overlapsAABox(box, .{}));

    // Oriented box vs oriented box
    const identity = OrientedBox.init(Mat44.identity(), Vec3.one());
    try std.testing.expect(identity.overlaps(touching, .{}));
    try std.testing.expect(!identity.overlaps(separated, .{}));
    try std.testing.expect(identity.overlaps(rotated, .{}));
    try std.testing.expect(rotated.overlaps(identity, .{}));

    // A big epsilon makes boxes overlap
    try std.testing.expect(separated.overlapsAABox(box, .{ .epsilon = 0.1 }));
    try std.testing.expect(identity.overlaps(separated, .{ .epsilon = 0.1 }));
}
