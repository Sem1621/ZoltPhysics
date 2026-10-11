//! Port of: Jolt/Geometry/AABox.h
//! Status: complete
//!
//! Overloads: the unsuffixed function takes an AABox (`encapsulate`, `contains`, `overlaps`), the others are
//! suffixed with the type they take (`encapsulateVec3`, `containsDVec3`, `overlapsPlane`, `transformedDMat44`, ...).
//! The `...RVec3` / `...RMat44` aliases pick the Vec3 / DVec3 (Mat44 / DMat44) overload that matches `Core.double_precision`.
//! The default constructor (an empty, invalid box) is `AABox.empty`.
//! `getSupport` / `getSupportingFace` make an AABox a convex object for GJK / EPA (see ConvexSupport).

const std = @import("std");
const Core = @import("../Core/Core.zig");
const math = @import("../Math/Math.zig");
const DMat44 = @import("../Math/DMat44.zig").DMat44;
const DVec3 = @import("../Math/DVec3.zig").DVec3;
const Float3 = @import("../Math/Float3.zig").Float3;
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const UVec4 = @import("../Math/UVec4.zig").UVec4;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const IndexedTriangle = @import("IndexedTriangle.zig").IndexedTriangle;
const Plane = @import("Plane.zig").Plane;
const Triangle = @import("Triangle.zig").Triangle;
const VertexArray = @import("VertexArray.zig");

/// Axis aligned box
pub const AABox = extern struct {
    /// Bounding box min and max
    min: Vec3,
    max: Vec3,

    /// Empty bounding box: min = FLT_MAX, max = -FLT_MAX (the default constructor AABox()). Encapsulating a point
    /// or box makes it valid.
    pub const empty: AABox = .{ .min = .replicate(math.flt_max), .max = .replicate(-math.flt_max) };

    /// Constructor
    pub fn init(min: Vec3, max: Vec3) AABox {
        return .{ .min = min, .max = max };
    }

    /// Constructor from double precision min / max, rounds min down and max up so the box contains the input (AABox(DVec3Arg, DVec3Arg))
    pub fn fromDVec3(min: DVec3, max: DVec3) AABox {
        return .{ .min = min.toVec3RoundDown(), .max = max.toVec3RoundUp() };
    }

    /// `init` for RVec3 min / max: the spelling that works in single and double precision
    pub const fromRVec3 = if (Core.double_precision) fromDVec3 else init;

    /// Constructor from a center and radius (AABox(Vec3Arg inCenter, float inRadius))
    pub fn fromCenterAndRadius(center: Vec3, radius: f32) AABox {
        return .{ .min = center.sub(Vec3.replicate(radius)), .max = center.add(Vec3.replicate(radius)) };
    }

    /// Create box from 2 points
    pub fn fromTwoPoints(p1: Vec3, p2: Vec3) AABox {
        return init(Vec3.min(p1, p2), Vec3.max(p1, p2));
    }

    /// Create box from indexed triangle
    pub fn fromTriangle(vertices: []const Float3, triangle: IndexedTriangle) AABox {
        var box = fromTwoPoints(Vec3.fromFloat3(vertices[triangle.idx[0]]), Vec3.fromFloat3(vertices[triangle.idx[1]]));
        box.encapsulateVec3(Vec3.fromFloat3(vertices[triangle.idx[2]]));
        return box;
    }

    /// Get bounding box of size FLT_MAX
    pub fn biggest() AABox {
        // Max half extent of AABox is 0.5 * FLT_MAX so that GetSize() remains finite
        return init(Vec3.replicate(-0.5 * math.flt_max), Vec3.replicate(0.5 * math.flt_max));
    }

    /// Comparison operators (operator ==), `!=` is `!a.eql(b)`
    pub fn eql(self: AABox, rhs: AABox) bool {
        return self.min.eql(rhs.min) and self.max.eql(rhs.max);
    }

    /// Reset the bounding box to an empty bounding box
    pub fn setEmpty(self: *AABox) void {
        self.min = Vec3.replicate(math.flt_max);
        self.max = Vec3.replicate(-math.flt_max);
    }

    /// Check if the bounding box is valid (max >= min)
    pub fn isValid(self: AABox) bool {
        return self.min.getX() <= self.max.getX() and self.min.getY() <= self.max.getY() and self.min.getZ() <= self.max.getZ();
    }

    /// Encapsulate point in bounding box (Encapsulate(Vec3Arg))
    pub fn encapsulateVec3(self: *AABox, pos: Vec3) void {
        self.min = Vec3.min(self.min, pos);
        self.max = Vec3.max(self.max, pos);
    }

    /// Encapsulate bounding box in bounding box (Encapsulate(const AABox &))
    pub fn encapsulate(self: *AABox, rhs: AABox) void {
        self.min = Vec3.min(self.min, rhs.min);
        self.max = Vec3.max(self.max, rhs.max);
    }

    /// Encapsulate triangle in bounding box (Encapsulate(const Triangle &))
    pub fn encapsulateTriangle(self: *AABox, rhs: Triangle) void {
        var v = Vec3.loadFloat3Unsafe(&rhs.v[0]);
        self.encapsulateVec3(v);
        v = Vec3.loadFloat3Unsafe(&rhs.v[1]);
        self.encapsulateVec3(v);
        v = Vec3.loadFloat3Unsafe(&rhs.v[2]);
        self.encapsulateVec3(v);
    }

    /// Encapsulate triangle in bounding box (Encapsulate(const VertexList &, const IndexedTriangle &))
    pub fn encapsulateIndexedTriangle(self: *AABox, vertices: []const Float3, triangle: IndexedTriangle) void {
        for (triangle.idx) |idx|
            self.encapsulateVec3(Vec3.fromFloat3(vertices[idx]));
    }

    /// Intersect this bounding box with other, returns the intersection
    pub fn intersect(self: AABox, other: AABox) AABox {
        return init(Vec3.max(self.min, other.min), Vec3.min(self.max, other.max));
    }

    /// Make sure that each edge of the bounding box has a minimal length
    pub fn ensureMinimalEdgeLength(self: *AABox, min_edge_length: f32) void {
        const min_length = Vec3.replicate(min_edge_length);
        self.max = Vec3.select(self.max, self.min.add(min_length), Vec3.less(self.max.sub(self.min), min_length));
    }

    /// Widen the box on both sides by vector
    pub fn expandBy(self: *AABox, vector: Vec3) void {
        self.min = self.min.sub(vector);
        self.max = self.max.add(vector);
    }

    /// Get center of bounding box
    pub fn getCenter(self: AABox) Vec3 {
        return self.min.add(self.max).mulScalar(0.5);
    }

    /// Get extent of bounding box (half of the size)
    pub fn getExtent(self: AABox) Vec3 {
        return self.max.sub(self.min).mulScalar(0.5);
    }

    /// Get size of bounding box
    pub fn getSize(self: AABox) Vec3 {
        return self.max.sub(self.min);
    }

    /// Get surface area of bounding box
    pub fn getSurfaceArea(self: AABox) f32 {
        const extent = self.max.sub(self.min);
        return 2.0 * (extent.getX() * extent.getY() + extent.getX() * extent.getZ() + extent.getY() * extent.getZ());
    }

    /// Get volume of bounding box
    pub fn getVolume(self: AABox) f32 {
        const extent = self.max.sub(self.min);
        return extent.getX() * extent.getY() * extent.getZ();
    }

    /// Check if this box contains another box (Contains(const AABox &))
    pub fn contains(self: AABox, other: AABox) bool {
        return UVec4.bitAnd(Vec3.lessOrEqual(self.min, other.min), Vec3.greaterOrEqual(self.max, other.max)).testAllXYZTrue();
    }

    /// Check if this box contains a point (Contains(Vec3Arg))
    pub fn containsVec3(self: AABox, other: Vec3) bool {
        return UVec4.bitAnd(Vec3.lessOrEqual(self.min, other), Vec3.greaterOrEqual(self.max, other)).testAllXYZTrue();
    }

    /// Check if this box contains a point (Contains(DVec3Arg)), the point is rounded to the nearest float
    pub fn containsDVec3(self: AABox, other: DVec3) bool {
        return self.containsVec3(other.toVec3());
    }

    /// `containsVec3` for an RVec3 point: the spelling that works in single and double precision
    pub const containsRVec3 = if (Core.double_precision) containsDVec3 else containsVec3;

    /// Check if this box overlaps with another box (Overlaps(const AABox &))
    pub fn overlaps(self: AABox, other: AABox) bool {
        return !UVec4.bitOr(Vec3.greater(self.min, other.max), Vec3.less(self.max, other.min)).testAnyXYZTrue();
    }

    /// Check if this box overlaps with a plane (Overlaps(const Plane &))
    pub fn overlapsPlane(self: AABox, plane: Plane) bool {
        const normal = plane.getNormal();
        const dist_normal = plane.signedDistance(self.getSupport(normal));
        const dist_min_normal = plane.signedDistance(self.getSupport(normal.negate()));
        return dist_normal * dist_min_normal <= 0.0; // If both support points are on the same side of the plane we don't overlap
    }

    /// Translate bounding box (Translate(Vec3Arg))
    pub fn translate(self: *AABox, translation: Vec3) void {
        self.min = self.min.add(translation);
        self.max = self.max.add(translation);
    }

    /// Translate bounding box (Translate(DVec3Arg)), rounds min down and max up so the box stays conservative
    pub fn translateDVec3(self: *AABox, translation: DVec3) void {
        self.min = DVec3.fromVec3(self.min).add(translation).toVec3RoundDown();
        self.max = DVec3.fromVec3(self.max).add(translation).toVec3RoundUp();
    }

    /// `translate` for an RVec3 translation: the spelling that works in single and double precision
    pub const translateRVec3 = if (Core.double_precision) translateDVec3 else translate;

    /// Transform bounding box (Transformed(Mat44Arg))
    pub fn transformed(self: AABox, matrix: Mat44) AABox {
        // Start with the translation of the matrix
        var new_min = matrix.getTranslation();
        var new_max = new_min;

        // Now find the extreme points by considering the product of the min and max with each column of matrix
        for (0..3) |c| {
            const col = matrix.getColumn3(@intCast(c));

            const a = col.mulScalar(self.min.getComponent(@intCast(c)));
            const b = col.mulScalar(self.max.getComponent(@intCast(c)));

            new_min = new_min.add(Vec3.min(a, b));
            new_max = new_max.add(Vec3.max(a, b));
        }

        // Return the new bounding box
        return init(new_min, new_max);
    }

    /// Transform bounding box (Transformed(DMat44Arg))
    pub fn transformedDMat44(self: AABox, matrix: DMat44) AABox {
        var result = self.transformed(matrix.getRotation());
        result.translateDVec3(matrix.getTranslation());
        return result;
    }

    /// `transformed` for an RMat44: the spelling that works in single and double precision
    pub const transformedRMat44 = if (Core.double_precision) transformedDMat44 else transformed;

    /// Scale this bounding box, can handle non-uniform and negative scaling
    pub fn scaled(self: AABox, scale: Vec3) AABox {
        return fromTwoPoints(self.min.mul(scale), self.max.mul(scale));
    }

    /// Calculate the support vector for this convex shape.
    pub fn getSupport(self: AABox, direction: Vec3) Vec3 {
        return Vec3.select(self.max, self.min, Vec3.less(direction, Vec3.zero()));
    }

    /// Get the vertices of the face that faces direction the most.
    /// `out_vertices` is a vertex array (`*StaticArray(Vec3, N)` or `VertexArrayList`), see VertexArray.zig.
    pub fn getSupportingFace(self: AABox, direction: Vec3, out_vertices: anytype) VertexArray.Error(@TypeOf(out_vertices))!void {
        try VertexArray.resize(out_vertices, 4);
        const out = VertexArray.items(out_vertices);

        const axis = direction.abs().getHighestComponentIndex();
        if (direction.getComponent(axis) < 0.0) {
            switch (axis) {
                0 => {
                    out[0] = Vec3.init(self.max.getX(), self.min.getY(), self.min.getZ());
                    out[1] = Vec3.init(self.max.getX(), self.max.getY(), self.min.getZ());
                    out[2] = Vec3.init(self.max.getX(), self.max.getY(), self.max.getZ());
                    out[3] = Vec3.init(self.max.getX(), self.min.getY(), self.max.getZ());
                },
                1 => {
                    out[0] = Vec3.init(self.min.getX(), self.max.getY(), self.min.getZ());
                    out[1] = Vec3.init(self.min.getX(), self.max.getY(), self.max.getZ());
                    out[2] = Vec3.init(self.max.getX(), self.max.getY(), self.max.getZ());
                    out[3] = Vec3.init(self.max.getX(), self.max.getY(), self.min.getZ());
                },
                2 => {
                    out[0] = Vec3.init(self.min.getX(), self.min.getY(), self.max.getZ());
                    out[1] = Vec3.init(self.max.getX(), self.min.getY(), self.max.getZ());
                    out[2] = Vec3.init(self.max.getX(), self.max.getY(), self.max.getZ());
                    out[3] = Vec3.init(self.min.getX(), self.max.getY(), self.max.getZ());
                },
                else => unreachable,
            }
        } else {
            switch (axis) {
                0 => {
                    out[0] = Vec3.init(self.min.getX(), self.min.getY(), self.min.getZ());
                    out[1] = Vec3.init(self.min.getX(), self.min.getY(), self.max.getZ());
                    out[2] = Vec3.init(self.min.getX(), self.max.getY(), self.max.getZ());
                    out[3] = Vec3.init(self.min.getX(), self.max.getY(), self.min.getZ());
                },
                1 => {
                    out[0] = Vec3.init(self.min.getX(), self.min.getY(), self.min.getZ());
                    out[1] = Vec3.init(self.max.getX(), self.min.getY(), self.min.getZ());
                    out[2] = Vec3.init(self.max.getX(), self.min.getY(), self.max.getZ());
                    out[3] = Vec3.init(self.min.getX(), self.min.getY(), self.max.getZ());
                },
                2 => {
                    out[0] = Vec3.init(self.min.getX(), self.min.getY(), self.min.getZ());
                    out[1] = Vec3.init(self.min.getX(), self.max.getY(), self.min.getZ());
                    out[2] = Vec3.init(self.max.getX(), self.max.getY(), self.min.getZ());
                    out[3] = Vec3.init(self.max.getX(), self.min.getY(), self.min.getZ());
                },
                else => unreachable,
            }
        }
    }

    /// Get the closest point on or in this box to point
    pub fn getClosestPoint(self: AABox, point: Vec3) Vec3 {
        return Vec3.min(Vec3.max(point, self.min), self.max);
    }

    /// Get the squared distance between point and this box (will be 0 if in Point is inside the box)
    pub fn getSqDistanceTo(self: AABox, point: Vec3) f32 {
        return self.getClosestPoint(point).sub(point).lengthSq();
    }
};

test "AABox construction" {
    var box: AABox = .empty;
    try std.testing.expect(!box.isValid());
    box.encapsulateVec3(Vec3.init(1, 2, 3));
    try std.testing.expect(box.isValid());
    try std.testing.expect(box.eql(.init(Vec3.init(1, 2, 3), Vec3.init(1, 2, 3))));
    box.encapsulateVec3(Vec3.init(-1, 5, 0));
    try std.testing.expect(box.eql(.init(Vec3.init(-1, 2, 0), Vec3.init(1, 5, 3))));
    box.setEmpty();
    try std.testing.expect(box.eql(AABox.empty));

    try std.testing.expect(AABox.fromTwoPoints(Vec3.init(1, -2, 3), Vec3.init(-1, 2, -3)).eql(.init(Vec3.init(-1, -2, -3), Vec3.init(1, 2, 3))));
    try std.testing.expect(AABox.fromCenterAndRadius(Vec3.init(1, 2, 3), 2).eql(.init(Vec3.init(-1, 0, 1), Vec3.init(3, 4, 5))));

    // Conversion from double rounds outward
    const d = AABox.fromDVec3(DVec3.init(0.1, -0.1, 1), DVec3.init(0.1, -0.1, 1));
    try std.testing.expect(@as(f64, d.min.getX()) < 0.1 and @as(f64, d.max.getX()) > 0.1);
    try std.testing.expect(@as(f64, d.min.getY()) < -0.1 and @as(f64, d.max.getY()) > -0.1);
    try std.testing.expect(d.min.getZ() == 1 and d.max.getZ() == 1);
    try std.testing.expect(AABox.fromRVec3(.init(1, 2, 3), .init(4, 5, 6)).eql(.init(Vec3.init(1, 2, 3), Vec3.init(4, 5, 6))));

    const big = AABox.biggest();
    try std.testing.expect(std.math.isFinite(big.getSize().getX()));
    try std.testing.expect(big.contains(.init(Vec3.replicate(-1.0e30), Vec3.replicate(1.0e30))));

    // From (indexed) triangles
    const vertices = [_]Float3{ .init(1, 2, 3), .init(-1, 0, 4), .init(2, -3, 0) };
    const expected = AABox.init(Vec3.init(-1, -3, 0), Vec3.init(2, 2, 4));
    try std.testing.expect(AABox.fromTriangle(&vertices, .init(0, 1, 2, .{})).eql(expected));
    var b2: AABox = .empty;
    b2.encapsulateIndexedTriangle(&vertices, .init(2, 0, 1, .{}));
    try std.testing.expect(b2.eql(expected));
    var b3: AABox = .empty;
    b3.encapsulateTriangle(.fromFloat3(vertices[0], vertices[1], vertices[2], .{}));
    try std.testing.expect(b3.eql(expected));
    var b4: AABox = .init(Vec3.zero(), Vec3.one());
    b4.encapsulate(expected);
    try std.testing.expect(b4.eql(.init(Vec3.init(-1, -3, 0), Vec3.init(2, 2, 4))));
}

test "AABox properties" {
    var box = AABox.init(Vec3.init(1, 2, 3), Vec3.init(3, 6, 9));
    try std.testing.expect(box.getCenter().eql(Vec3.init(2, 4, 6)));
    try std.testing.expect(box.getExtent().eql(Vec3.init(1, 2, 3)));
    try std.testing.expect(box.getSize().eql(Vec3.init(2, 4, 6)));
    try std.testing.expectEqual(@as(f32, 2 * (8 + 12 + 24)), box.getSurfaceArea());
    try std.testing.expectEqual(@as(f32, 48), box.getVolume());

    try std.testing.expect(box.containsVec3(Vec3.init(1, 2, 3)));
    try std.testing.expect(box.containsVec3(Vec3.init(3, 6, 9)));
    try std.testing.expect(!box.containsVec3(Vec3.init(0.9, 2, 3)));
    try std.testing.expect(box.containsDVec3(DVec3.init(2, 4, 6)));
    try std.testing.expect(box.containsRVec3(.init(2, 4, 6)));
    try std.testing.expect(box.contains(.init(Vec3.init(1, 2, 3), Vec3.init(2, 2, 3))));
    try std.testing.expect(!box.contains(.init(Vec3.init(1, 2, 3), Vec3.init(4, 2, 3))));

    // Touching boxes overlap
    try std.testing.expect(box.overlaps(.init(Vec3.init(3, 6, 9), Vec3.init(4, 7, 10))));
    try std.testing.expect(!box.overlaps(.init(Vec3.init(3.1, 6, 9), Vec3.init(4, 7, 10))));
    try std.testing.expect(box.overlapsPlane(.fromPointAndNormal(Vec3.init(0, 5, 0), Vec3.init(0, 1, 0))));
    try std.testing.expect(!box.overlapsPlane(.fromPointAndNormal(Vec3.init(0, 7, 0), Vec3.init(0, 1, 0))));

    try std.testing.expect(box.intersect(.init(Vec3.init(2, 0, 0), Vec3.init(10, 3, 4))).eql(.init(Vec3.init(2, 2, 3), Vec3.init(3, 3, 4))));

    box.expandBy(Vec3.init(1, 1, 1));
    try std.testing.expect(box.eql(.init(Vec3.init(0, 1, 2), Vec3.init(4, 7, 10))));
    box.translate(Vec3.init(1, 2, 3));
    try std.testing.expect(box.eql(.init(Vec3.init(1, 3, 5), Vec3.init(5, 9, 13))));
    box.translateDVec3(DVec3.init(-1, -2, -3));
    try std.testing.expect(box.eql(.init(Vec3.init(0, 1, 2), Vec3.init(4, 7, 10))));
    box.translateRVec3(.init(0, 0, 0));
    try std.testing.expect(box.eql(.init(Vec3.init(0, 1, 2), Vec3.init(4, 7, 10))));

    var thin = AABox.init(Vec3.init(0, 0, 0), Vec3.init(1, 0, 0.5));
    thin.ensureMinimalEdgeLength(0.75);
    try std.testing.expect(thin.eql(.init(Vec3.init(0, 0, 0), Vec3.init(1, 0.75, 0.75))));

    try std.testing.expect(box.getClosestPoint(Vec3.init(-1, 5, 20)).eql(Vec3.init(0, 5, 10)));
    try std.testing.expectEqual(@as(f32, 1 + 100), box.getSqDistanceTo(Vec3.init(-1, 5, 20)));
    try std.testing.expectEqual(@as(f32, 0), box.getSqDistanceTo(Vec3.init(1, 2, 3)));
}

test "AABox transform" {
    const box = AABox.init(Vec3.init(-1, -2, -3), Vec3.init(1, 2, 3));
    try std.testing.expect(box.scaled(Vec3.init(-1, 2, 1)).eql(.init(Vec3.init(-1, -4, -3), Vec3.init(1, 4, 3))));

    const m = Mat44.rotationZ(0.5 * math.pi).postTranslated(Vec3.init(10, 20, 30));
    const t = box.transformed(m);
    try std.testing.expect(t.min.isClose(Vec3.init(8, 19, 27), .{ .max_dist_sq = 1.0e-10 }));
    try std.testing.expect(t.max.isClose(Vec3.init(12, 21, 33), .{ .max_dist_sq = 1.0e-10 }));

    const dm = DMat44.fromMat44Translation(Mat44.rotationZ(0.5 * math.pi), DVec3.init(10, 20, 30));
    const dt = box.transformedDMat44(dm);
    try std.testing.expect(dt.min.isClose(t.min, .{ .max_dist_sq = 1.0e-10 }));
    try std.testing.expect(dt.max.isClose(t.max, .{ .max_dist_sq = 1.0e-10 }));
    _ = box.transformedRMat44(.identity());
}

test "AABox support" {
    const box = AABox.init(Vec3.init(-1, -2, -3), Vec3.init(1, 2, 3));
    try std.testing.expect(box.getSupport(Vec3.init(1, -1, 0)).eql(Vec3.init(1, -2, 3)));
    try std.testing.expect(box.getSupport(Vec3.init(-0.0, 1, -1)).eql(Vec3.init(1, 2, -3)));

    // Supporting face with a StaticArray
    var face: @import("../Core/StaticArray.zig").StaticArray(Vec3, 8) = .empty;
    try box.getSupportingFace(Vec3.init(0, 0, -1), &face);
    try std.testing.expectEqual(@as(u32, 4), face.len);
    for (face.constSlice()) |v|
        try std.testing.expectEqual(@as(f32, 3), v.getZ()); // Direction -Z gives the face at max Z (normal +Z)
    try box.getSupportingFace(Vec3.init(2, 1, 0), &face);
    try std.testing.expectEqual(@as(u32, 4), face.len);
    for (face.constSlice()) |v|
        try std.testing.expectEqual(@as(f32, -1), v.getX());

    // Supporting face with a std.ArrayList
    const allocator = std.testing.allocator;
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(allocator);
    try box.getSupportingFace(Vec3.init(0, -3, 1), VertexArray.VertexArrayList.init(allocator, &list));
    try std.testing.expectEqual(@as(usize, 4), list.items.len);
    for (list.items) |v|
        try std.testing.expectEqual(@as(f32, 2), v.getY());
}
