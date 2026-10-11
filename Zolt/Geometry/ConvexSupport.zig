//! Port of: Jolt/Geometry/ConvexSupport.h
//! Status: complete
//!
//! Helper functions to get the support point for a convex object.
//!
//! ## The convex object protocol
//!
//! Jolt's GJK / EPA code and the wrappers in this file are templates over a "convex object" type. In Zolt they
//! are comptime generics over any type `T` that declares:
//!
//! - `pub fn getSupport(self: *const T, direction: Vec3) Vec3` (or `self: T`): the point of the object that is
//!   furthest along `direction`. `direction` does not need to be normalized and can be zero.
//!
//! Objects that can also provide the face that faces a direction the most (needed by the collision code that
//! builds contact manifolds) declare:
//!
//! - `pub fn getSupportingFace(self: *const T, direction: Vec3, out_vertices: anytype)
//!   VertexArray.Error(@TypeOf(out_vertices))!void`: adds the vertices of that face to `out_vertices`.
//!
//! `out_vertices` (Jolt's `VERTEX_ARRAY &outVertices`) is a vertex array as described in `VertexArray.zig`: a
//! `*StaticArray(Vec3, N)` (in practice Jolt's `Shape::SupportingFace`, a `*StaticArray(Vec3, 32)`) or a
//! `VertexArrayList`. Implementations access it through the `VertexArray` helpers (`append`, `resize`, `items`, ...).
//! For a StaticArray the error set is empty, so callers can always `try` the call.
//!
//! The wrappers keep a pointer to the objects they wrap (Jolt keeps a `const ConvexObject &`), so the wrapped
//! objects must outlive the wrapper. Construct them with `init`, e.g. Jolt's
//! `TransformedConvexObject transformed_a(inStart, inA)` is
//! `const transformed_a = TransformedConvexObject(A).init(start, &a);`.
//!
//! `PolygonConvexSupport` takes its vertices as a read only vertex array (Jolt's `const VERTEX_ARRAY &`), which in
//! Zolt is a plain `[]const Vec3` slice (see `VertexArray.zig`), e.g. Jolt's `PolygonConvexSupport polygon(face)` is
//! `const polygon = PolygonConvexSupport.init(face.constSlice());`.

const std = @import("std");
const math = @import("../Math/Math.zig");
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const Quat = @import("../Math/Quat.zig").Quat;
const StaticArray = @import("../Core/StaticArray.zig").StaticArray;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const VertexArray = @import("VertexArray.zig");
const VertexArrayList = VertexArray.VertexArrayList;

/// Structure that transforms a convex object (supports only uniform scaling)
pub fn TransformedConvexObject(comptime ConvexObject: type) type {
    return struct {
        const Self = @This();

        transform: Mat44,
        object: *const ConvexObject,

        /// Create transformed convex object.
        pub fn init(transform: Mat44, object: *const ConvexObject) Self {
            return .{ .transform = transform, .object = object };
        }

        /// Calculate the support vector for this convex shape.
        pub fn getSupport(self: *const Self, direction: Vec3) Vec3 {
            return self.transform.mulVec3(self.object.getSupport(self.transform.multiply3x3Transposed(direction)));
        }

        /// Get the vertices of the face that faces direction the most.
        /// `out_vertices`: a vertex array, see the convex object protocol at the top of this file.
        /// Note that, like in Jolt, all vertices in out_vertices are transformed (also the ones that were in it before the call).
        pub fn getSupportingFace(self: *const Self, direction: Vec3, out_vertices: anytype) VertexArray.Error(@TypeOf(out_vertices))!void {
            try self.object.getSupportingFace(self.transform.multiply3x3Transposed(direction), out_vertices);

            for (VertexArray.items(out_vertices)) |*v|
                v.* = self.transform.mulVec3(v.*);
        }
    };
}

/// Structure that adds a convex radius
pub fn AddConvexRadius(comptime ConvexObject: type) type {
    return struct {
        const Self = @This();

        object: *const ConvexObject,
        radius: f32,

        pub fn init(object: *const ConvexObject, radius: f32) Self {
            return .{ .object = object, .radius = radius };
        }

        /// Calculate the support vector for this convex shape.
        pub fn getSupport(self: *const Self, direction: Vec3) Vec3 {
            const length = direction.length();
            return if (length > 0.0) self.object.getSupport(direction).add(direction.mulScalar(self.radius / length)) else self.object.getSupport(direction);
        }
    };
}

/// Structure that performs a Minkowski difference A - B
pub fn MinkowskiDifference(comptime ConvexObjectA: type, comptime ConvexObjectB: type) type {
    return struct {
        const Self = @This();

        object_a: *const ConvexObjectA,
        object_b: *const ConvexObjectB,

        pub fn init(object_a: *const ConvexObjectA, object_b: *const ConvexObjectB) Self {
            return .{ .object_a = object_a, .object_b = object_b };
        }

        /// Calculate the support vector for this convex shape.
        pub fn getSupport(self: *const Self, direction: Vec3) Vec3 {
            return self.object_a.getSupport(direction).sub(self.object_b.getSupport(direction.negate()));
        }
    };
}

/// Class that wraps a point so that it can be used with convex collision detection
pub const PointConvexSupport = struct {
    point: Vec3,

    /// Calculate the support vector for this convex shape.
    pub fn getSupport(self: *const PointConvexSupport, direction: Vec3) Vec3 {
        _ = direction;
        return self.point;
    }
};

/// Class that wraps a triangle so that it can used with convex collision detection
pub const TriangleConvexSupport = struct {
    /// The three vertices of the triangle
    v1: Vec3,
    v2: Vec3,
    v3: Vec3,

    /// Constructor
    pub fn init(v1: Vec3, v2: Vec3, v3: Vec3) TriangleConvexSupport {
        return .{ .v1 = v1, .v2 = v2, .v3 = v3 };
    }

    /// Calculate the support vector for this convex shape.
    pub fn getSupport(self: *const TriangleConvexSupport, direction: Vec3) Vec3 {
        // Project vertices on direction
        const d1 = self.v1.dot(direction);
        const d2 = self.v2.dot(direction);
        const d3 = self.v3.dot(direction);

        // Return vertex with biggest projection
        if (d1 > d2) {
            if (d1 > d3)
                return self.v1
            else
                return self.v3;
        } else {
            if (d2 > d3)
                return self.v2
            else
                return self.v3;
        }
    }

    /// Get the vertices of the face that faces direction the most.
    /// `out_vertices`: a vertex array, see the convex object protocol at the top of this file.
    pub fn getSupportingFace(self: *const TriangleConvexSupport, direction: Vec3, out_vertices: anytype) VertexArray.Error(@TypeOf(out_vertices))!void {
        _ = direction;
        try VertexArray.append(out_vertices, self.v1);
        try VertexArray.append(out_vertices, self.v2);
        try VertexArray.append(out_vertices, self.v3);
    }
};

/// Class that wraps a polygon so that it can used with convex collision detection.
/// Jolt's `PolygonConvexSupport<VERTEX_ARRAY>` keeps a `const VERTEX_ARRAY &`, here the vertices are a read only vertex
/// array: a `[]const Vec3` slice. Like the reference in Jolt it must stay valid while the wrapper is used.
pub const PolygonConvexSupport = struct {
    /// The vertices of the polygon
    vertices: []const Vec3,

    /// Constructor
    pub fn init(vertices: []const Vec3) PolygonConvexSupport {
        return .{ .vertices = vertices };
    }

    /// Calculate the support vector for this convex shape.
    pub fn getSupport(self: *const PolygonConvexSupport, direction: Vec3) Vec3 {
        var support_point = self.vertices[0];
        var best_dot = self.vertices[0].dot(direction);

        for (self.vertices[1..]) |v| {
            const dot = v.dot(direction);
            if (dot > best_dot) {
                best_dot = dot;
                support_point = v;
            }
        }

        return support_point;
    }

    /// Get the vertices of the face that faces direction the most.
    /// `out_vertices`: a vertex array, see the convex object protocol at the top of this file.
    pub fn getSupportingFace(self: *const PolygonConvexSupport, direction: Vec3, out_vertices: anytype) VertexArray.Error(@TypeOf(out_vertices))!void {
        _ = direction;
        for (self.vertices) |v|
            try VertexArray.append(out_vertices, v);
    }
};

/// Axis aligned box around the origin, a convex object for the tests that has getSupport and getSupportingFace.
/// Like AABox.getSupportingFace it fills the face with `resize` and writes the vertices through `items`.
/// TODO(Geometry merge): also test TransformedConvexObject(AABox).getSupportingFace once AABox.zig is merged.
const TestBox = struct {
    half_extent: Vec3,

    pub fn getSupport(self: TestBox, direction: Vec3) Vec3 {
        return Vec3.select(self.half_extent, self.half_extent.negate(), Vec3.less(direction, Vec3.zero()));
    }

    pub fn getSupportingFace(self: TestBox, direction: Vec3, out_vertices: anytype) VertexArray.Error(@TypeOf(out_vertices))!void {
        // Only the +X / -X faces, enough for the tests
        const x = if (direction.getX() < 0.0) -self.half_extent.getX() else self.half_extent.getX();
        const y = self.half_extent.getY();
        const z = self.half_extent.getZ();
        const first = VertexArray.len(out_vertices);
        try VertexArray.resize(out_vertices, first + 4);
        const out = VertexArray.items(out_vertices)[first..];
        out[0] = Vec3.init(x, -y, -z);
        out[1] = Vec3.init(x, y, -z);
        out[2] = Vec3.init(x, y, z);
        out[3] = Vec3.init(x, -y, z);
    }
};

test "PointConvexSupport / TriangleConvexSupport" {
    const point: PointConvexSupport = .{ .point = Vec3.init(1, 2, 3) };
    try std.testing.expect(point.getSupport(Vec3.init(-1, 0, 0)).eql(Vec3.init(1, 2, 3)));

    const v1 = Vec3.init(1, 0, 0);
    const v2 = Vec3.init(0, 1, 0);
    const v3 = Vec3.init(0, 0, 1);
    const triangle = TriangleConvexSupport.init(v1, v2, v3);
    try std.testing.expect(triangle.getSupport(Vec3.init(1, 0.5, 0.5)).eql(v1));
    try std.testing.expect(triangle.getSupport(Vec3.init(0.5, 1, 0.5)).eql(v2));
    try std.testing.expect(triangle.getSupport(Vec3.init(0.5, 0.5, 1)).eql(v3));
    try std.testing.expect(triangle.getSupport(Vec3.init(1, 0.5, 1)).eql(v3)); // d1 > d2, d3 >= d1
    try std.testing.expect(triangle.getSupport(Vec3.zero()).eql(v3)); // Ties go to v3

    // Supporting face into a StaticArray
    var face: StaticArray(Vec3, 32) = .empty;
    try triangle.getSupportingFace(Vec3.init(0, 0, 1), &face);
    try std.testing.expectEqual(@as(u32, 3), face.len);
    try std.testing.expect(face.get(0).eql(v1) and face.get(1).eql(v2) and face.get(2).eql(v3));

    // Supporting face into a VertexArrayList
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(std.testing.allocator);
    try triangle.getSupportingFace(Vec3.init(0, 0, 1), VertexArrayList.init(std.testing.allocator, &list));
    try std.testing.expectEqual(@as(usize, 3), list.items.len);
    try std.testing.expect(list.items[0].eql(v1) and list.items[1].eql(v2) and list.items[2].eql(v3));

    // Out of memory is reported
    var failing = std.testing.FailingAllocator.init(std.testing.allocator, .{ .fail_index = 0 });
    var failing_list: std.ArrayList(Vec3) = .empty;
    try std.testing.expectError(error.OutOfMemory, triangle.getSupportingFace(Vec3.zero(), VertexArrayList.init(failing.allocator(), &failing_list)));
}

test "PolygonConvexSupport" {
    const vertices = [_]Vec3{ Vec3.init(-1, -1, 0), Vec3.init(1, -1, 0), Vec3.init(1, 1, 0), Vec3.init(-1, 1, 0) };
    const direction = Vec3.init(1, 2, 0);
    const expected = vertices[2];

    // Array
    const polygon_array = PolygonConvexSupport.init(&vertices);
    try std.testing.expect(polygon_array.getSupport(direction).eql(expected));

    // StaticArray (Jolt: PolygonConvexSupport polygon(face) with a Shape::SupportingFace)
    const static_array = StaticArray(Vec3, 32).fromSlice(&vertices);
    const polygon_static = PolygonConvexSupport.init(static_array.constSlice());
    try std.testing.expect(polygon_static.getSupport(direction).eql(expected));

    // ArrayList
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(std.testing.allocator);
    try list.appendSlice(std.testing.allocator, &vertices);
    const polygon_list = PolygonConvexSupport.init(list.items);
    try std.testing.expect(polygon_list.getSupport(direction).eql(expected));

    // Ties keep the first vertex
    try std.testing.expect(polygon_list.getSupport(Vec3.init(1, 0, 0)).eql(vertices[1]));
    try std.testing.expect(polygon_list.getSupport(Vec3.zero()).eql(vertices[0]));

    // Supporting face returns all vertices
    var face: StaticArray(Vec3, 32) = .empty;
    try polygon_static.getSupportingFace(direction, &face);
    try std.testing.expectEqual(@as(u32, 4), face.len);
    for (face.constSlice(), vertices) |a, b|
        try std.testing.expect(a.eql(b));

    // Also into a VertexArrayList, appended after the vertices that are already in it
    var face_list: std.ArrayList(Vec3) = .empty;
    defer face_list.deinit(std.testing.allocator);
    try face_list.append(std.testing.allocator, Vec3.zero());
    try polygon_array.getSupportingFace(direction, VertexArrayList.init(std.testing.allocator, &face_list));
    try std.testing.expectEqual(@as(usize, 5), face_list.items.len);
    for (face_list.items[1..], vertices) |a, b|
        try std.testing.expect(a.eql(b));
}

test "AddConvexRadius / MinkowskiDifference / TransformedConvexObject" {
    const box: TestBox = .{ .half_extent = Vec3.init(1, 2, 3) };

    // Add a convex radius
    const rounded = AddConvexRadius(TestBox).init(&box, 0.5);
    try std.testing.expect(rounded.getSupport(Vec3.init(2, 0, 0)).isClose(Vec3.init(1.5, 2, 3), .{}));
    try std.testing.expect(rounded.getSupport(Vec3.zero()).eql(Vec3.init(1, 2, 3))); // Zero direction: no radius added

    // Minkowski difference of a box and a point
    const point: PointConvexSupport = .{ .point = Vec3.init(1, 1, 1) };
    const diff = MinkowskiDifference(AddConvexRadius(TestBox), PointConvexSupport).init(&rounded, &point);
    try std.testing.expect(diff.getSupport(Vec3.init(-2, 0, 0)).isClose(Vec3.init(-2.5, 1, 2), .{}));

    // Transform the box: rotate 90 degrees around Z and translate
    const transform = Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 0.5 * math.pi), Vec3.init(10, 0, 0));
    const transformed = TransformedConvexObject(TestBox).init(transform, &box);
    try std.testing.expect(transformed.getSupport(Vec3.init(1, 1, 1)).isClose(Vec3.init(12, 1, 3), .{ .max_dist_sq = 1.0e-10 }));

    // Supporting face is transformed, including the vertices that were already in the array
    var face: StaticArray(Vec3, 32) = .empty;
    face.append(Vec3.zero());
    try transformed.getSupportingFace(Vec3.init(0, 1, 0), &face);
    try std.testing.expectEqual(@as(u32, 5), face.len);
    try std.testing.expect(face.get(0).isClose(Vec3.init(10, 0, 0), .{ .max_dist_sq = 1.0e-10 }));
    for (face.constSlice()[1..]) |v|
        try std.testing.expect(@abs(v.getY() - 1) < 1.0e-5); // The +X face of the box is at Y = 1 after rotation

    // Same with a VertexArrayList
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(std.testing.allocator);
    try list.append(std.testing.allocator, Vec3.zero());
    try transformed.getSupportingFace(Vec3.init(0, 1, 0), VertexArrayList.init(std.testing.allocator, &list));
    try std.testing.expectEqual(@as(usize, 5), list.items.len);
    for (list.items, face.constSlice()) |a, b|
        try std.testing.expect(a.eql(b));

    // Transformed polygon and triangle (nested wrappers)
    const triangle = TriangleConvexSupport.init(Vec3.init(1, 0, 0), Vec3.init(0, 1, 0), Vec3.init(0, 0, 1));
    const transformed_triangle = TransformedConvexObject(TriangleConvexSupport).init(Mat44.translation(Vec3.init(0, 0, 5)), &triangle);
    face.clear();
    try transformed_triangle.getSupportingFace(Vec3.init(0, 0, 1), &face);
    try std.testing.expectEqual(@as(u32, 3), face.len);
    try std.testing.expect(face.get(2).eql(Vec3.init(0, 0, 6)));

    const vertices = [_]Vec3{ Vec3.init(-1, 0, 0), Vec3.init(1, 0, 0), Vec3.init(0, 1, 0) };
    const polygon = PolygonConvexSupport.init(&vertices);
    const transformed_polygon = TransformedConvexObject(PolygonConvexSupport).init(Mat44.translation(Vec3.init(0, 0, 5)), &polygon);
    try std.testing.expect(transformed_polygon.getSupport(Vec3.init(0, 1, 0)).eql(Vec3.init(0, 1, 5)));
    face.clear();
    try transformed_polygon.getSupportingFace(Vec3.init(0, 0, 1), &face);
    try std.testing.expectEqual(@as(u32, 3), face.len);
    try std.testing.expect(face.get(1).eql(Vec3.init(1, 0, 5)));

    // Doubly transformed box: the inner wrapper's error set is forwarded
    const transformed_twice = TransformedConvexObject(TransformedConvexObject(TestBox)).init(Mat44.translation(Vec3.init(0, 0, 1)), &transformed);
    list.clearRetainingCapacity();
    try transformed_twice.getSupportingFace(Vec3.init(0, 1, 0), VertexArrayList.init(std.testing.allocator, &list));
    try std.testing.expectEqual(@as(usize, 4), list.items.len);
    try std.testing.expect(list.items[0].isClose(Vec3.init(12, 1, -2), .{ .max_dist_sq = 1.0e-10 }));
}
