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
//! - `pub fn getSupportingFace(self: *const T, direction: Vec3, out_vertices: anytype) void`: appends the vertices
//!   of that face to `out_vertices`.
//!
//! `out_vertices` (Jolt's `VERTEX_ARRAY &outVertices`) is a pointer to a vertex array with the interface of
//! `Core/StaticArray.zig`: `append(Vec3) void` and `slice() []Vec3`. In practice it is a
//! `*StaticArray(Vec3, 32)` (Jolt's `Shape::SupportingFace`).
//!
//! The wrappers keep a pointer to the objects they wrap (Jolt keeps a `const ConvexObject &`), so the wrapped
//! objects must outlive the wrapper. Construct them with `init`, e.g. Jolt's
//! `TransformedConvexObject transformed_a(inStart, inA)` is
//! `const transformed_a = TransformedConvexObject(A).init(start, &a);`.
//!
//! `PolygonConvexSupport(VertexArray)` accepts the vertex array types `[]const Vec3` / `[]Vec3`, `[N]Vec3`,
//! `StaticArray(Vec3, N)` (anything with `constSlice()`) and `std.ArrayList(Vec3)` (anything with `items`).

const std = @import("std");
const math = @import("../Math/Math.zig");
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const Quat = @import("../Math/Quat.zig").Quat;
const StaticArray = @import("../Core/StaticArray.zig").StaticArray;
const Vec3 = @import("../Math/Vec3.zig").Vec3;

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
        /// `out_vertices`: pointer to a vertex array, see the convex object protocol at the top of this file.
        /// Note that, like in Jolt, all vertices in out_vertices are transformed (also the ones that were in it before the call).
        pub fn getSupportingFace(self: *const Self, direction: Vec3, out_vertices: anytype) void {
            self.object.getSupportingFace(self.transform.multiply3x3Transposed(direction), out_vertices);

            for (out_vertices.slice()) |*v|
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
    /// `out_vertices`: pointer to a vertex array, see the convex object protocol at the top of this file.
    pub fn getSupportingFace(self: *const TriangleConvexSupport, direction: Vec3, out_vertices: anytype) void {
        _ = direction;
        out_vertices.append(self.v1);
        out_vertices.append(self.v2);
        out_vertices.append(self.v3);
    }
};

/// Class that wraps a polygon so that it can used with convex collision detection.
/// `VertexArray` is one of the vertex array types listed at the top of this file.
pub fn PolygonConvexSupport(comptime VertexArray: type) type {
    return struct {
        const Self = @This();

        /// The vertices of the polygon
        vertices: *const VertexArray,

        /// Constructor
        pub fn init(vertices: *const VertexArray) Self {
            return .{ .vertices = vertices };
        }

        /// Calculate the support vector for this convex shape.
        pub fn getSupport(self: *const Self, direction: Vec3) Vec3 {
            const vertices = vertexSlice(VertexArray, self.vertices);

            var support_point = vertices[0];
            var best_dot = vertices[0].dot(direction);

            for (vertices[1..]) |v| {
                const dot = v.dot(direction);
                if (dot > best_dot) {
                    best_dot = dot;
                    support_point = v;
                }
            }

            return support_point;
        }

        /// Get the vertices of the face that faces direction the most.
        /// `out_vertices`: pointer to a vertex array, see the convex object protocol at the top of this file.
        pub fn getSupportingFace(self: *const Self, direction: Vec3, out_vertices: anytype) void {
            _ = direction;
            for (vertexSlice(VertexArray, self.vertices)) |v|
                out_vertices.append(v);
        }
    };
}

/// The vertices of a vertex array that PolygonConvexSupport accepts as a slice
fn vertexSlice(comptime VertexArray: type, vertices: *const VertexArray) []const Vec3 {
    switch (@typeInfo(VertexArray)) {
        .pointer => |pointer| if (pointer.size == .slice and pointer.child == Vec3) return vertices.*,
        .array => |array| if (array.child == Vec3) return vertices,
        .@"struct" => {
            if (@hasDecl(VertexArray, "constSlice")) return vertices.constSlice();
            if (@hasField(VertexArray, "items")) return vertices.items;
        },
        else => {},
    }
    @compileError("PolygonConvexSupport: unsupported vertex array type " ++ @typeName(VertexArray));
}

/// Axis aligned box around the origin, a convex object for the tests that has getSupport and getSupportingFace
const TestBox = struct {
    half_extent: Vec3,

    pub fn getSupport(self: TestBox, direction: Vec3) Vec3 {
        return Vec3.select(self.half_extent, self.half_extent.negate(), Vec3.less(direction, Vec3.zero()));
    }

    pub fn getSupportingFace(self: TestBox, direction: Vec3, out_vertices: anytype) void {
        // Only the +X / -X faces, enough for the tests
        const x = if (direction.getX() < 0.0) -self.half_extent.getX() else self.half_extent.getX();
        const y = self.half_extent.getY();
        const z = self.half_extent.getZ();
        out_vertices.append(Vec3.init(x, -y, -z));
        out_vertices.append(Vec3.init(x, y, -z));
        out_vertices.append(Vec3.init(x, y, z));
        out_vertices.append(Vec3.init(x, -y, z));
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

    var face: StaticArray(Vec3, 32) = .empty;
    triangle.getSupportingFace(Vec3.init(0, 0, 1), &face);
    try std.testing.expectEqual(@as(u32, 3), face.len);
    try std.testing.expect(face.get(0).eql(v1) and face.get(1).eql(v2) and face.get(2).eql(v3));
}

test "PolygonConvexSupport" {
    const vertices = [_]Vec3{ Vec3.init(-1, -1, 0), Vec3.init(1, -1, 0), Vec3.init(1, 1, 0), Vec3.init(-1, 1, 0) };
    const direction = Vec3.init(1, 2, 0);
    const expected = vertices[2];

    // Array
    const polygon_array = PolygonConvexSupport([4]Vec3).init(&vertices);
    try std.testing.expect(polygon_array.getSupport(direction).eql(expected));

    // Slice
    const slice: []const Vec3 = &vertices;
    const polygon_slice = PolygonConvexSupport([]const Vec3).init(&slice);
    try std.testing.expect(polygon_slice.getSupport(direction).eql(expected));

    // StaticArray
    const static_array = StaticArray(Vec3, 32).fromSlice(&vertices);
    const polygon_static = PolygonConvexSupport(StaticArray(Vec3, 32)).init(&static_array);
    try std.testing.expect(polygon_static.getSupport(direction).eql(expected));

    // ArrayList
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(std.testing.allocator);
    try list.appendSlice(std.testing.allocator, &vertices);
    const polygon_list = PolygonConvexSupport(std.ArrayList(Vec3)).init(&list);
    try std.testing.expect(polygon_list.getSupport(direction).eql(expected));

    // Ties keep the first vertex
    try std.testing.expect(polygon_list.getSupport(Vec3.init(1, 0, 0)).eql(vertices[1]));
    try std.testing.expect(polygon_list.getSupport(Vec3.zero()).eql(vertices[0]));

    // Supporting face returns all vertices
    var face: StaticArray(Vec3, 32) = .empty;
    polygon_static.getSupportingFace(direction, &face);
    try std.testing.expectEqual(@as(u32, 4), face.len);
    for (face.constSlice(), vertices) |a, b|
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
    transformed.getSupportingFace(Vec3.init(0, 1, 0), &face);
    try std.testing.expectEqual(@as(u32, 5), face.len);
    try std.testing.expect(face.get(0).isClose(Vec3.init(10, 0, 0), .{ .max_dist_sq = 1.0e-10 }));
    for (face.constSlice()[1..]) |v|
        try std.testing.expect(@abs(v.getY() - 1) < 1.0e-5); // The +X face of the box is at Y = 1 after rotation

    // Transformed polygon and triangle (nested wrappers)
    const triangle = TriangleConvexSupport.init(Vec3.init(1, 0, 0), Vec3.init(0, 1, 0), Vec3.init(0, 0, 1));
    const transformed_triangle = TransformedConvexObject(TriangleConvexSupport).init(Mat44.translation(Vec3.init(0, 0, 5)), &triangle);
    face.clear();
    transformed_triangle.getSupportingFace(Vec3.init(0, 0, 1), &face);
    try std.testing.expectEqual(@as(u32, 3), face.len);
    try std.testing.expect(face.get(2).eql(Vec3.init(0, 0, 6)));

    const vertices = [_]Vec3{ Vec3.init(-1, 0, 0), Vec3.init(1, 0, 0), Vec3.init(0, 1, 0) };
    const polygon = PolygonConvexSupport([3]Vec3).init(&vertices);
    const transformed_polygon = TransformedConvexObject(PolygonConvexSupport([3]Vec3)).init(Mat44.translation(Vec3.init(0, 0, 5)), &polygon);
    try std.testing.expect(transformed_polygon.getSupport(Vec3.init(0, 1, 0)).eql(Vec3.init(0, 1, 5)));
    face.clear();
    transformed_polygon.getSupportingFace(Vec3.init(0, 0, 1), &face);
    try std.testing.expectEqual(@as(u32, 3), face.len);
    try std.testing.expect(face.get(1).eql(Vec3.init(1, 0, 5)));
}
