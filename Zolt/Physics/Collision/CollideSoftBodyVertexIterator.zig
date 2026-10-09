//! Port of: Jolt/Physics/Collision/CollideSoftBodyVertexIterator.h
//! Status: partial
//! Missing: the constructor from a SoftBodyVertex array (CollideSoftBodyVertexIterator(SoftBodyVertex *), needs
//!   SoftBodyVertex, Phase 9)
//!
//! Declared by the shape core (Phase 4, foundation part 2) because `Shape.collideSoftBodyVertices` takes it; the
//! iterator operations were added with the first shapes that implement CollideSoftBodyVertices (SphereShape, BoxShape).
//!
//! Operators: `++it` is `increment()`, `it + offset` is `add(offset)` (only sets the position, used for the end
//! iterator), `it != end` is `!it.eql(end)` (compares the position only). The copy constructor / assignment are plain
//! copies. The const functions `updatePenetration` / `setCollision` write through the strided output pointers (the
//! state lives behind pointer fields, Rule M of Docs/Zolt/CollisionArchitecture.md).

const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Plane = @import("../../Geometry/Plane.zig").Plane;
const StridedPtrFile = @import("../../Core/StridedPtr.zig");
const StridedPtr = StridedPtrFile.StridedPtr;
const StridedPtrConst = StridedPtrFile.StridedPtrConst;

/// Class that allows iterating over the vertices of a soft body.
/// It tracks the largest penetration and allows storing the resulting collision in a different structure than the soft body vertex itself.
pub const CollideSoftBodyVertexIterator = struct {
    /// Input data
    position: StridedPtrConst(Vec3) = .{},
    inv_mass: StridedPtrConst(f32) = .{},

    /// Output data
    collision_plane: StridedPtr(Plane) = .{},
    largest_penetration: StridedPtr(f32) = .{},
    colliding_shape_index: StridedPtr(i32) = .{},

    /// Construct using (strided) pointers
    pub fn init(position: StridedPtrConst(Vec3), inv_mass: StridedPtrConst(f32), collision_plane: StridedPtr(Plane), largest_penetration: StridedPtr(f32), colliding_shape_index: StridedPtr(i32)) CollideSoftBodyVertexIterator {
        return .{
            .position = position,
            .inv_mass = inv_mass,
            .collision_plane = collision_plane,
            .largest_penetration = largest_penetration,
            .colliding_shape_index = colliding_shape_index,
        };
    }

    /// Equality operator (C++ only has operator !=, `it != end` is `!it.eql(end)`).
    /// Note: Only used to determine end iterator, so we only compare position.
    pub fn eql(self: *const CollideSoftBodyVertexIterator, rhs: *const CollideSoftBodyVertexIterator) bool {
        return self.position.eql(rhs.position);
    }

    /// Next vertex (operator ++)
    pub fn increment(self: *CollideSoftBodyVertexIterator) void {
        self.position.increment();
        self.inv_mass.increment();
        self.collision_plane.increment();
        self.largest_penetration.increment();
        self.colliding_shape_index.increment();
    }

    /// Add an offset (operator +)
    /// Note: Only used to determine end iterator, so we only set position.
    pub fn add(self: *const CollideSoftBodyVertexIterator, offset: i32) CollideSoftBodyVertexIterator {
        return .init(self.position.add(offset), .{}, .{}, .{}, .{});
    }

    /// Get the position of the current vertex
    pub fn getPosition(self: *const CollideSoftBodyVertexIterator) Vec3 {
        return self.position.deref().*;
    }

    /// Get the inverse mass of the current vertex
    pub fn getInvMass(self: *const CollideSoftBodyVertexIterator) f32 {
        return self.inv_mass.deref().*;
    }

    /// Update penetration of the current vertex
    /// @return Returns true if the vertex has the largest penetration so far, this means you need to follow up by calling SetCollision
    pub fn updatePenetration(self: *const CollideSoftBodyVertexIterator, largest_penetration: f32) bool {
        const penetration = self.largest_penetration.deref();
        if (penetration.* >= largest_penetration)
            return false;
        penetration.* = largest_penetration;
        return true;
    }

    /// Update the collision of the current vertex
    pub fn setCollision(self: *const CollideSoftBodyVertexIterator, collision_plane: Plane, colliding_shape_index: i32) void {
        self.collision_plane.deref().* = collision_plane;
        self.colliding_shape_index.deref().* = colliding_shape_index;
    }
};

test "CollideSoftBodyVertexIterator: iterate, update penetration, set collision" {
    const std = @import("std");

    // Vertices in a larger struct (like SoftBodyVertex): strided pointers into it
    const Vertex = struct {
        position: Vec3,
        inv_mass: f32,
        plane: Plane = Plane.init(Vec3.zero(), 0.0),
        largest_penetration: f32 = -std.math.floatMax(f32),
        colliding_shape_index: i32 = -1,
    };
    var vertices = [_]Vertex{ .{ .position = Vec3.init(1, 2, 3), .inv_mass = 1.0 }, .{ .position = Vec3.init(4, 5, 6), .inv_mass = 0.0 } };
    const stride: i32 = @sizeOf(Vertex);
    const begin = CollideSoftBodyVertexIterator.init(.init(&vertices[0].position, .{ .stride = stride }), .init(&vertices[0].inv_mass, .{ .stride = stride }), .init(&vertices[0].plane, .{ .stride = stride }), .init(&vertices[0].largest_penetration, .{ .stride = stride }), .init(&vertices[0].colliding_shape_index, .{ .stride = stride }));
    const end = begin.add(2);

    var count: u32 = 0;
    var v = begin;
    while (!v.eql(&end)) : (v.increment()) {
        try std.testing.expect(v.getPosition().eql(vertices[count].position));
        try std.testing.expectEqual(vertices[count].inv_mass, v.getInvMass());
        try std.testing.expect(v.updatePenetration(0.5));
        try std.testing.expect(!v.updatePenetration(0.25)); // Not larger
        v.setCollision(Plane.init(Vec3.axisY(), 2.0), 7);
        count += 1;
    }
    try std.testing.expectEqual(@as(u32, 2), count);
    for (vertices) |vertex| {
        try std.testing.expectEqual(@as(f32, 0.5), vertex.largest_penetration);
        try std.testing.expectEqual(@as(i32, 7), vertex.colliding_shape_index);
        try std.testing.expectEqual(@as(f32, 2.0), vertex.plane.getConstant());
    }
}
