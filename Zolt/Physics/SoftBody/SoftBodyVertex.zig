//! Port of: Jolt/Physics/SoftBody/SoftBodyVertex.h
//! Status: complete
//!
//! RemainingFeaturesArchitecture.md R15: an extern struct in Jolt's field order. Its size (80 bytes) is checked at
//! compile time, because CollideSoftBodyVertexIterator strides over an array of vertices with `@sizeOf(SoftBodyVertex)`.
//! - Jolt's default constructor leaves every member uninitialized. The Zig fields have no default values, so a vertex is
//!   always written in full (SoftBodyMotionProperties.initialize fills it in).
//! - `MarkCCDContact` stores `int(id | BodyID::cBroadPhaseBit)`, a negative value: `@bitCast` of the u32.

const std = @import("std");
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Plane = @import("../../Geometry/Plane.zig").Plane;
const BodyID = @import("../Body/BodyID.zig").BodyID;

/// Run time information for a single particle of a soft body
/// Note that at run-time you should only modify the inverse mass and/or velocity of a vertex to control the soft body.
/// Modifying the position can lead to missed collisions.
/// The other members are used internally by the soft body solver.
pub const SoftBodyVertex = extern struct {
    /// Internal use only. Position at the previous time step
    previous_position: Vec3,
    /// Position, relative to the center of mass of the soft body
    position: Vec3,
    /// Velocity, relative to the center of mass of the soft body
    velocity: Vec3,
    /// Internal use only. Nearest collision plane, relative to the center of mass of the soft body
    collision_plane: Plane,
    /// Internal use only. Index in the colliding shapes list of the body we may collide with
    colliding_shape_index: i32,
    /// True if the vertex has collided with anything in the last update
    has_contact: bool,
    /// Internal use only. Used while finding the collision plane, stores the largest penetration found so far
    largest_penetration: f32,
    /// Inverse mass (1 / mass)
    inv_mass: f32,

    comptime {
        // sizeof(SoftBodyVertex): CollideSoftBodyVertexIterator strides over the vertices with this size
        std.debug.assert(@sizeOf(SoftBodyVertex) == 80);
        std.debug.assert(@alignOf(SoftBodyVertex) == 16);
    }

    /// Reset collision information to prepare for a new collision check
    pub fn resetCollision(self: *SoftBodyVertex) void {
        self.largest_penetration = -math.flt_max;
        self.colliding_shape_index = -1;
        self.has_contact = false;
    }

    /// Mark this vertex as being in contact with body_id
    pub fn markCCDContact(self: *SoftBodyVertex, body_id: BodyID, contact_plane: Plane) void {
        self.collision_plane = contact_plane;
        self.colliding_shape_index = @bitCast(body_id.getIndexAndSequenceNumber() | BodyID.broad_phase_bit); // We reuse the broad phase bit to indicate this is a CCD contact
        self.has_contact = true;
    }
};

test "SoftBodyVertex: layout, resetCollision and markCCDContact" {
    const expectEqual = std.testing.expectEqual;

    // Jolt's offsets: four 16 byte vectors, then the int, the bool (3 bytes padding) and the two floats
    try expectEqual(@as(usize, 0), @offsetOf(SoftBodyVertex, "previous_position"));
    try expectEqual(@as(usize, 16), @offsetOf(SoftBodyVertex, "position"));
    try expectEqual(@as(usize, 32), @offsetOf(SoftBodyVertex, "velocity"));
    try expectEqual(@as(usize, 48), @offsetOf(SoftBodyVertex, "collision_plane"));
    try expectEqual(@as(usize, 64), @offsetOf(SoftBodyVertex, "colliding_shape_index"));
    try expectEqual(@as(usize, 68), @offsetOf(SoftBodyVertex, "has_contact"));
    try expectEqual(@as(usize, 72), @offsetOf(SoftBodyVertex, "largest_penetration"));
    try expectEqual(@as(usize, 76), @offsetOf(SoftBodyVertex, "inv_mass"));

    var v: SoftBodyVertex = undefined;
    v.resetCollision();
    try expectEqual(-math.flt_max, v.largest_penetration);
    try expectEqual(@as(i32, -1), v.colliding_shape_index);
    try expectEqual(false, v.has_contact);

    // The broad phase bit marks a CCD contact: the index is negative, the body ID comes back with the bit cleared
    const id = BodyID.fromIndexAndSequenceNumber(5, 3);
    v.markCCDContact(id, Plane.init(Vec3.axisY(), -1.0));
    try std.testing.expect(v.has_contact);
    try std.testing.expect(v.colliding_shape_index < 0);
    try expectEqual(id.getIndexAndSequenceNumber(), @as(u32, @bitCast(v.colliding_shape_index)) ^ BodyID.broad_phase_bit);
    try expectEqual(@as(f32, -1.0), v.collision_plane.getConstant());
}
