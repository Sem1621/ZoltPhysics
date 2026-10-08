//! Port of: Jolt/Physics/Collision/Shape/GetTrianglesContext.h (prototype, reduced)
//! Status: partial
//! Missing: GetTrianglesContextMultiVertexList, sCreateUnitOpenCylinder
//!
//! D10: contexts are constructed in place in the caller's `Shape.GetTrianglesContext` (`context.emplace(T)` + in place
//! initialization, `context.get(T)` to cast back). They are never destroyed, so they must not own resources.
//! The vertex lists that Jolt builds with static initializers (lambdas) are computed at compile time with these
//! helpers (same operations, IEEE exact at comptime; a test compares them with the runtime result).

const std = @import("std");
const zolt = @import("zolt");
const Vec3 = zolt.Vec3;
const Mat44 = zolt.Mat44;
const Quat = zolt.Quat;
const Float3 = zolt.Float3;
const VertexArray = zolt.VertexArray;
const PhysicsMaterial = @import("../PhysicsMaterial.zig").PhysicsMaterial;
const Shape = @import("Shape.zig").Shape;
const ScaleHelpers = @import("ScaleHelpers.zig");

/// Implementation of GetTrianglesStart/Next that uses a fixed list of vertices for the triangles. These are transformed into world space when getting the triangles.
pub const GetTrianglesContextVertexList = struct {
    local_to_world: Mat44,
    triangle_vertices: []const Vec3,
    current_vertex: usize = 0,
    material: *const PhysicsMaterial,
    is_inside_out: bool,

    /// Constructor, to be called in GetTrianglesStart: `context.emplace(GetTrianglesContextVertexList).* = .init(...)`
    pub fn init(position_com: Vec3, rotation: Quat, scale: Vec3, local_transform: Mat44, triangle_vertices: []const Vec3, material: *const PhysicsMaterial) GetTrianglesContextVertexList {
        std.debug.assert(triangle_vertices.len % 3 == 0);
        return .{
            .local_to_world = Mat44.rotationTranslation(rotation, position_com).mul(Mat44.scaleVec3(scale)).mul(local_transform),
            .triangle_vertices = triangle_vertices,
            .material = material,
            .is_inside_out = ScaleHelpers.isInsideOut(scale),
        };
    }

    /// @see Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *GetTrianglesContextVertexList, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        std.debug.assert(max_triangles_requested >= Shape.get_triangles_min_triangles_requested);

        const total_num_vertices: usize = @min(@as(usize, max_triangles_requested) * 3, self.triangle_vertices.len - self.current_vertex);
        const vertices = self.triangle_vertices[self.current_vertex .. self.current_vertex + total_num_vertices];

        var out: usize = 0;
        var i: usize = 0;
        if (self.is_inside_out) {
            // Store triangles flipped
            while (i < vertices.len) : (i += 3) {
                self.local_to_world.mulVec3(vertices[i]).storeFloat3(&out_triangle_vertices[out]);
                self.local_to_world.mulVec3(vertices[i + 2]).storeFloat3(&out_triangle_vertices[out + 1]);
                self.local_to_world.mulVec3(vertices[i + 1]).storeFloat3(&out_triangle_vertices[out + 2]);
                out += 3;
            }
        } else {
            // Store triangles
            while (i < vertices.len) : (i += 3) {
                self.local_to_world.mulVec3(vertices[i]).storeFloat3(&out_triangle_vertices[out]);
                self.local_to_world.mulVec3(vertices[i + 1]).storeFloat3(&out_triangle_vertices[out + 1]);
                self.local_to_world.mulVec3(vertices[i + 2]).storeFloat3(&out_triangle_vertices[out + 2]);
                out += 3;
            }
        }

        // Update the current vertex to point to the next vertex to get
        self.current_vertex += total_num_vertices;
        const total_num_triangles = total_num_vertices / 3;

        // Store materials
        if (out_materials) |materials| {
            for (materials[0..total_num_triangles]) |*m|
                m.* = self.material;
        }

        return @intCast(total_num_triangles);
    }

    /// Helper function that creates a vertex list of a half unit sphere (top part)
    pub fn createHalfUnitSphereTop(vertices: anytype, detail_level: u32) VertexArray.Error(@TypeOf(vertices))!void {
        try createUnitSphereHelper(vertices, Vec3.axisX(), Vec3.axisY(), Vec3.axisZ(), detail_level);
        try createUnitSphereHelper(vertices, Vec3.axisY(), Vec3.axisX().negate(), Vec3.axisZ(), detail_level);
        try createUnitSphereHelper(vertices, Vec3.axisY(), Vec3.axisX(), Vec3.axisZ().negate(), detail_level);
        try createUnitSphereHelper(vertices, Vec3.axisX().negate(), Vec3.axisY(), Vec3.axisZ().negate(), detail_level);
    }

    /// Helper function that creates a vertex list of a half unit sphere (bottom part)
    pub fn createHalfUnitSphereBottom(vertices: anytype, detail_level: u32) VertexArray.Error(@TypeOf(vertices))!void {
        try createUnitSphereHelper(vertices, Vec3.axisX().negate(), Vec3.axisY().negate(), Vec3.axisZ(), detail_level);
        try createUnitSphereHelper(vertices, Vec3.axisY().negate(), Vec3.axisX(), Vec3.axisZ(), detail_level);
        try createUnitSphereHelper(vertices, Vec3.axisX(), Vec3.axisY().negate(), Vec3.axisZ().negate(), detail_level);
        try createUnitSphereHelper(vertices, Vec3.axisY().negate(), Vec3.axisX().negate(), Vec3.axisZ().negate(), detail_level);
    }

    /// Recursive helper function for creating a sphere
    fn createUnitSphereHelper(vertices: anytype, v1: Vec3, v2: Vec3, v3: Vec3, level: u32) VertexArray.Error(@TypeOf(vertices))!void {
        const center1 = v1.add(v2).normalized();
        const center2 = v2.add(v3).normalized();
        const center3 = v3.add(v1).normalized();

        if (level > 0) {
            const new_level = level - 1;
            try createUnitSphereHelper(vertices, v1, center1, center3, new_level);
            try createUnitSphereHelper(vertices, center1, center2, center3, new_level);
            try createUnitSphereHelper(vertices, center1, v2, center2, new_level);
            try createUnitSphereHelper(vertices, center3, center2, v3, new_level);
        } else {
            try VertexArray.append(vertices, v1);
            try VertexArray.append(vertices, v2);
            try VertexArray.append(vertices, v3);
        }
    }
};
