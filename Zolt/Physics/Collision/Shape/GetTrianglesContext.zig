//! Port of: Jolt/Physics/Collision/Shape/GetTrianglesContext.h
//! Status: complete
//!
//! D10 (Docs/Zolt/CollisionArchitecture.md): contexts are constructed in place in the caller's
//! `Shape.GetTrianglesContext` (`context.emplace(GetTrianglesContextVertexList).* = .init(...)`, `context.get(T)` to
//! cast back). They are never destroyed, so they must not own resources.
//!
//! - The vertex lists are slices (`inTriangleVertices` + `inNumTriangleVertices`), `GetTrianglesNext` takes slices
//!   for the output (`out_triangle_vertices` holds at least 3 * max_triangles_requested vertices).
//! - The `sCreate...` helpers append to a vertex array (`anytype`, see Geometry/VertexArray.zig) and return its error
//!   set. Shapes that build their vertex list in a static initializer call them at compile time.

const std = @import("std");
const StaticArray = @import("../../../Core/StaticArray.zig").StaticArray;
const VertexArray = @import("../../../Geometry/VertexArray.zig");
const math = @import("../../../Math/Math.zig");
const trigonometry = @import("../../../Math/Trigonometry.zig");
const Float3 = @import("../../../Math/Float3.zig").Float3;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Quat = @import("../../../Math/Quat.zig").Quat;
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
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

    comptime {
        std.debug.assert(@sizeOf(GetTrianglesContextVertexList) <= Shape.GetTrianglesContext.buffer_size); // GetTrianglesContext too small
    }

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
        var v: usize = 0;
        if (self.is_inside_out) {
            // Store triangles flipped
            while (v < vertices.len) : (v += 3) {
                self.local_to_world.mulVec3(vertices[v + 0]).storeFloat3(&out_triangle_vertices[out + 0]);
                self.local_to_world.mulVec3(vertices[v + 2]).storeFloat3(&out_triangle_vertices[out + 1]);
                self.local_to_world.mulVec3(vertices[v + 1]).storeFloat3(&out_triangle_vertices[out + 2]);
                out += 3;
            }
        } else {
            // Store triangles
            while (v < vertices.len) : (v += 3) {
                self.local_to_world.mulVec3(vertices[v + 0]).storeFloat3(&out_triangle_vertices[out + 0]);
                self.local_to_world.mulVec3(vertices[v + 1]).storeFloat3(&out_triangle_vertices[out + 1]);
                self.local_to_world.mulVec3(vertices[v + 2]).storeFloat3(&out_triangle_vertices[out + 2]);
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

    /// Helper function that creates an open cylinder of half height 1 and radius 1
    pub fn createUnitOpenCylinder(vertices: anytype, detail_level: u32) VertexArray.Error(@TypeOf(vertices))!void {
        const bottom_offset = Vec3.init(0.0, -2.0, 0.0);
        const num_verts: i32 = 4 * (@as(i32, 1) << @intCast(detail_level));
        var i: i32 = 0;
        while (i < num_verts) : (i += 1) {
            const angle1 = 2.0 * math.pi * (@as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(num_verts)));
            const angle2 = 2.0 * math.pi * (@as(f32, @floatFromInt(i + 1)) / @as(f32, @floatFromInt(num_verts)));

            const t1 = Vec3.init(trigonometry.sin(angle1), 1.0, trigonometry.cos(angle1));
            const t2 = Vec3.init(trigonometry.sin(angle2), 1.0, trigonometry.cos(angle2));
            const b1 = t1.add(bottom_offset);
            const b2 = t2.add(bottom_offset);

            try VertexArray.append(vertices, t1);
            try VertexArray.append(vertices, b1);
            try VertexArray.append(vertices, t2);

            try VertexArray.append(vertices, t2);
            try VertexArray.append(vertices, b1);
            try VertexArray.append(vertices, b2);
        }
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

/// Implementation of GetTrianglesStart/Next that uses a multiple fixed lists of vertices for the triangles. These are transformed into world space when getting the triangles.
pub const GetTrianglesContextMultiVertexList = struct {
    const Part = struct {
        local_to_world: Mat44,
        triangle_vertices: []const Vec3,
    };

    parts: StaticArray(Part, 3) = .empty,
    current_part: u32 = 0,
    current_vertex: usize = 0,
    material: *const PhysicsMaterial,
    is_inside_out: bool,

    comptime {
        std.debug.assert(@sizeOf(GetTrianglesContextMultiVertexList) <= Shape.GetTrianglesContext.buffer_size); // GetTrianglesContext too small
    }

    /// Constructor, to be called in GetTrianglesStart: `context.emplace(GetTrianglesContextMultiVertexList).* = .init(...)`
    pub fn init(is_inside_out: bool, material: *const PhysicsMaterial) GetTrianglesContextMultiVertexList {
        return .{ .material = material, .is_inside_out = is_inside_out };
    }

    /// Add a mesh part and its transform
    pub fn addPart(self: *GetTrianglesContextMultiVertexList, local_to_world: Mat44, triangle_vertices: []const Vec3) void {
        std.debug.assert(triangle_vertices.len % 3 == 0);

        self.parts.append(.{ .local_to_world = local_to_world, .triangle_vertices = triangle_vertices });
    }

    /// @see Shape::GetTrianglesNext
    pub fn getTrianglesNext(self: *GetTrianglesContextMultiVertexList, max_triangles_requested: u32, out_triangle_vertices: []Float3, out_materials: ?[]*const PhysicsMaterial) u32 {
        std.debug.assert(max_triangles_requested >= Shape.get_triangles_min_triangles_requested);

        var total_num_vertices: usize = 0;
        var max_vertices_requested: usize = @as(usize, max_triangles_requested) * 3;
        var out: usize = 0;

        // Loop over parts
        while (self.current_part < self.parts.len) : (self.current_part += 1) {
            const part = &self.parts.constSlice()[self.current_part];

            // Calculate how many vertices to take from this part
            const part_num_vertices = @min(max_vertices_requested, part.triangle_vertices.len - self.current_vertex);
            if (part_num_vertices == 0)
                break;

            max_vertices_requested -= part_num_vertices;
            total_num_vertices += part_num_vertices;

            const vertices = part.triangle_vertices[self.current_vertex .. self.current_vertex + part_num_vertices];
            var v: usize = 0;
            if (self.is_inside_out) {
                // Store triangles flipped
                while (v < vertices.len) : (v += 3) {
                    part.local_to_world.mulVec3(vertices[v + 0]).storeFloat3(&out_triangle_vertices[out + 0]);
                    part.local_to_world.mulVec3(vertices[v + 2]).storeFloat3(&out_triangle_vertices[out + 1]);
                    part.local_to_world.mulVec3(vertices[v + 1]).storeFloat3(&out_triangle_vertices[out + 2]);
                    out += 3;
                }
            } else {
                // Store triangles
                while (v < vertices.len) : (v += 3) {
                    part.local_to_world.mulVec3(vertices[v + 0]).storeFloat3(&out_triangle_vertices[out + 0]);
                    part.local_to_world.mulVec3(vertices[v + 1]).storeFloat3(&out_triangle_vertices[out + 1]);
                    part.local_to_world.mulVec3(vertices[v + 2]).storeFloat3(&out_triangle_vertices[out + 2]);
                    out += 3;
                }
            }

            // Update the current vertex to point to the next vertex to get
            self.current_vertex += part_num_vertices;

            // Check if we completed this part
            if (self.current_vertex < part.triangle_vertices.len)
                break;

            // Reset current vertex for the next part
            self.current_vertex = 0;
        }

        const total_num_triangles = total_num_vertices / 3;

        // Store materials
        if (out_materials) |materials| {
            for (materials[0..total_num_triangles]) |*m|
                m.* = self.material;
        }

        return @intCast(total_num_triangles);
    }
};

test "GetTrianglesContextVertexList: transform, blocks, inside out and materials" {
    const testing = std.testing;
    const expect = testing.expect;
    const VertexArrayList = VertexArray.VertexArrayList;
    const allocator = testing.allocator;

    // 70 triangles in a list
    var list: std.ArrayList(Vec3) = .empty;
    defer list.deinit(allocator);
    for (0..70) |i| {
        const f: f32 = @floatFromInt(i);
        try list.appendSlice(allocator, &.{ Vec3.init(f, 0, 0), Vec3.init(f, 1, 0), Vec3.init(f, 0, 1) });
    }

    const material = PhysicsMaterial.default;
    var context: Shape.GetTrianglesContext = .{};
    context.emplace(GetTrianglesContextVertexList).* = .init(Vec3.init(1, 2, 3), Quat.identity(), Vec3.init(1, 1, -1), Mat44.scale(2.0), list.items, material);
    const ctx = context.get(GetTrianglesContextVertexList);
    try expect(ctx.is_inside_out);

    var vertices: [3 * 32]Float3 = undefined;
    var materials: [32]*const PhysicsMaterial = undefined;
    try testing.expectEqual(@as(u32, 32), ctx.getTrianglesNext(32, &vertices, &materials));
    try testing.expectEqual(@as(u32, 32), ctx.getTrianglesNext(32, &vertices, &materials));
    try testing.expectEqual(@as(u32, 6), ctx.getTrianglesNext(32, &vertices, null));
    try testing.expectEqual(@as(u32, 0), ctx.getTrianglesNext(32, &vertices, &materials));
    try expect(materials[31] == material);

    // The last triangle (69): local_to_world = translate(1, 2, 3) * scale(1, 1, -1) * scale(2), flipped winding
    const last = vertices[3 * 5 ..][0..3];
    try expect(Vec3.fromFloat3(last[0]).eql(Vec3.init(139, 2, 3)));
    try expect(Vec3.fromFloat3(last[1]).eql(Vec3.init(139, 2, 1))); // v[2]
    try expect(Vec3.fromFloat3(last[2]).eql(Vec3.init(139, 4, 3))); // v[1]

    // The sphere helpers produce the same bits at compile time as at runtime (shapes build their tables at comptime)
    const comptime_sphere = comptime blk: {
        @setEvalBranchQuota(1_000_000);
        var v: StaticArray(Vec3, 384) = .empty;
        GetTrianglesContextVertexList.createHalfUnitSphereTop(&v, 2) catch unreachable;
        GetTrianglesContextVertexList.createHalfUnitSphereBottom(&v, 2) catch unreachable;
        break :blk v;
    };
    var runtime_sphere: std.ArrayList(Vec3) = .empty;
    defer runtime_sphere.deinit(allocator);
    var level: u32 = 2;
    _ = &level;
    try GetTrianglesContextVertexList.createHalfUnitSphereTop(VertexArrayList{ .allocator = allocator, .list = &runtime_sphere }, level);
    try GetTrianglesContextVertexList.createHalfUnitSphereBottom(VertexArrayList{ .allocator = allocator, .list = &runtime_sphere }, level);
    try testing.expectEqual(@as(usize, 384), runtime_sphere.items.len);
    try expect(std.mem.eql(u8, std.mem.sliceAsBytes(runtime_sphere.items), std.mem.sliceAsBytes(comptime_sphere.constSlice())));

    // Open cylinder: 4 * 2^level quads, top at y = 1, bottom at y = -1
    var cylinder: StaticArray(Vec3, 96) = .empty;
    try GetTrianglesContextVertexList.createUnitOpenCylinder(&cylinder, 2);
    try testing.expectEqual(@as(u32, 96), cylinder.len);
    try expect(cylinder.get(0).eql(Vec3.init(0, 1, 1)) and cylinder.get(1).eql(Vec3.init(0, -1, 1)));
    for (cylinder.constSlice()) |v| try expect(@abs(v.getY()) == 1.0 and @abs(v.getX() * v.getX() + v.getZ() * v.getZ() - 1.0) < 1.0e-6);
}

test "GetTrianglesContextMultiVertexList: parts are returned in order, blocks can span parts" {
    const testing = std.testing;
    const expect = testing.expect;

    var part1: [3 * 20]Vec3 = undefined;
    var part2: [3 * 40]Vec3 = undefined;
    for (&part1, 0..) |*v, i| v.* = Vec3.init(@floatFromInt(i / 3), 0, 0);
    for (&part2, 0..) |*v, i| v.* = Vec3.init(@floatFromInt(i / 3), @floatFromInt(i % 3), 1);

    var context: Shape.GetTrianglesContext = .{};
    const ctx = context.emplace(GetTrianglesContextMultiVertexList);
    ctx.* = .init(false, PhysicsMaterial.default);
    ctx.addPart(Mat44.identity(), &part1);
    ctx.addPart(Mat44.translation(Vec3.init(0, 0, 10)), &part2);

    var vertices: [3 * 32]Float3 = undefined;
    var materials: [32]*const PhysicsMaterial = undefined;
    try testing.expectEqual(@as(u32, 32), ctx.getTrianglesNext(32, &vertices, &materials)); // 20 + 12
    try expect(Vec3.fromFloat3(vertices[3 * 20]).eql(Vec3.init(0, 0, 11)));
    try testing.expectEqual(@as(u32, 28), ctx.getTrianglesNext(32, &vertices, &materials));
    try expect(Vec3.fromFloat3(vertices[3 * 27 + 2]).eql(Vec3.init(39, 2, 11)));
    try testing.expectEqual(@as(u32, 0), ctx.getTrianglesNext(32, &vertices, null));
    try expect(materials[0] == PhysicsMaterial.default);

    // Inside out: the winding of every triangle is flipped
    var flipped_context: Shape.GetTrianglesContext = .{};
    const flipped = flipped_context.emplace(GetTrianglesContextMultiVertexList);
    flipped.* = .init(true, PhysicsMaterial.default);
    flipped.addPart(Mat44.identity(), part2[0..6]);
    try testing.expectEqual(@as(u32, 2), flipped.getTrianglesNext(32, &vertices, null));
    try expect(Vec3.fromFloat3(vertices[1]).eql(part2[2]) and Vec3.fromFloat3(vertices[2]).eql(part2[1]));
}
