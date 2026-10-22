//! Port of: Jolt/Physics/SoftBody/SoftBodyManifold.h
//! Status: complete
//!
//! RemainingFeaturesArchitecture.md R15: Jolt's three array references are slices of the motion properties' arrays.
//! Only SoftBodyMotionProperties creates a manifold (Jolt: a private constructor and a friend class): `init` is for
//! UpdateSoftBodyState (group S4) and for tests. The manifold must not outlive the update that created it.

const std = @import("std");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const SoftBodyVertex = @import("SoftBodyVertex.zig").SoftBodyVertex;
const SoftBodyMotionPropertiesFile = @import("SoftBodyMotionProperties.zig");
const SoftBodyMotionProperties = SoftBodyMotionPropertiesFile.SoftBodyMotionProperties;

/// An interface to query which vertices of a soft body are colliding with other bodies
pub const SoftBodyManifold = struct {
    const CollidingShape = SoftBodyMotionProperties.CollidingShape;
    const CollidingSensor = SoftBodyMotionProperties.CollidingSensor;

    vertices: []const SoftBodyVertex,
    colliding_shapes: []const CollidingShape,
    colliding_sensors: []const CollidingSensor,

    /// Constructor (private in Jolt, SoftBodyMotionProperties is a friend)
    pub fn init(motion_properties: *const SoftBodyMotionProperties) SoftBodyManifold {
        return .{
            .vertices = motion_properties.vertices.items,
            .colliding_shapes = motion_properties.colliding_shapes.items,
            .colliding_sensors = motion_properties.colliding_sensors.items,
        };
    }

    /// Get the vertices of the soft body for iterating
    pub fn getVertices(self: *const SoftBodyManifold) []const SoftBodyVertex {
        return self.vertices;
    }

    /// Check if a vertex has collided with something in this update
    pub fn hasContact(self: *const SoftBodyManifold, vertex: *const SoftBodyVertex) bool {
        _ = self;
        return vertex.has_contact;
    }

    /// Get the local space contact point (multiply by GetCenterOfMassTransform() of the soft body to get world space)
    pub fn getLocalContactPoint(self: *const SoftBodyManifold, vertex: *const SoftBodyVertex) Vec3 {
        _ = self;
        return vertex.position.sub(vertex.collision_plane.getNormal().mulScalar(vertex.collision_plane.signedDistance(vertex.position)));
    }

    /// Get the contact normal for the vertex (assumes there is a contact).
    pub fn getContactNormal(self: *const SoftBodyManifold, vertex: *const SoftBodyVertex) Vec3 {
        _ = self;
        return vertex.collision_plane.getNormal().negate();
    }

    /// Get the body with which the vertex has collided in this update
    pub fn getContactBodyID(self: *const SoftBodyManifold, vertex: *const SoftBodyVertex) BodyID {
        if (!vertex.has_contact)
            return .invalid;

        // If this is a CCD contact, the BodyID is encoded in mCollidingShapeIndex
        const index: u32 = @bitCast(vertex.colliding_shape_index);
        if ((index & BodyID.broad_phase_bit) != 0)
            return .init(index ^ BodyID.broad_phase_bit);

        // Otherwise we can get it from mCollidingShapes
        return self.colliding_shapes[index].body_id;
    }

    /// Get the number of sensors that are in contact with the soft body
    pub fn getNumSensorContacts(self: *const SoftBodyManifold) u32 {
        return @intCast(self.colliding_sensors.len);
    }

    /// Get the i-th sensor that is in contact with the soft body
    pub fn getSensorContactBodyID(self: *const SoftBodyManifold, index: u32) BodyID {
        return self.colliding_sensors[index].body_id;
    }
};

test "SoftBodyManifold: contacts of the vertices, CCD contacts and sensors" {
    const allocator = std.testing.allocator;
    const Plane = @import("../../Geometry/Plane.zig").Plane;
    const Mat44 = @import("../../Math/Mat44.zig").Mat44;
    const expect = std.testing.expect;
    const expectEqual = std.testing.expectEqual;

    var mp: SoftBodyMotionProperties = .{ .allocator = allocator };
    defer mp.deinit();
    try mp.vertices.appendNTimes(allocator, std.mem.zeroes(SoftBodyVertex), 3);
    for (mp.vertices.items) |*v| v.resetCollision();

    // Vertex 0 touches colliding shape 1 through the plane y = 1, vertex 1 has a CCD contact, vertex 2 has no contact
    const shape: SoftBodyMotionProperties.CollidingShape = .{ .center_of_mass_transform = Mat44.identity(), .body_id = .init(7), .motion_type = .dynamic, .inv_mass = 1, .friction = 0, .restitution = 0, .soft_body_inv_mass_scale = 1, .update_velocities = false, .inv_inertia = Mat44.identity(), .linear_velocity = .zero(), .angular_velocity = .zero(), .original_linear_velocity = .zero(), .original_angular_velocity = .zero() };
    try mp.colliding_shapes.appendNTimes(allocator, shape, 2);
    mp.colliding_shapes.items[1].body_id = .init(9);
    try mp.colliding_sensors.append(allocator, .{ .center_of_mass_transform = Mat44.identity(), .body_id = .init(11), .has_contact = true });

    const v0 = &mp.vertices.items[0];
    v0.position = Vec3.init(2, 3, 4);
    v0.collision_plane = Plane.init(Vec3.axisY(), -1.0);
    v0.colliding_shape_index = 1;
    v0.has_contact = true;
    mp.vertices.items[1].markCCDContact(.init(5), Plane.init(Vec3.axisX(), 0.0));

    const manifold: SoftBodyManifold = .init(&mp);
    try expectEqual(@as(usize, 3), manifold.getVertices().len);
    try expect(manifold.hasContact(&manifold.getVertices()[0]));
    try expect(!manifold.hasContact(&manifold.getVertices()[2]));
    try expect(manifold.getLocalContactPoint(v0).eql(Vec3.init(2, 1, 4)));
    try expect(manifold.getContactNormal(v0).eql(Vec3.init(0, -1, 0)));
    try expectEqual(@as(u32, 9), manifold.getContactBodyID(v0).getIndexAndSequenceNumber());
    try expectEqual(@as(u32, 5), manifold.getContactBodyID(&manifold.getVertices()[1]).getIndexAndSequenceNumber());
    try expect(manifold.getContactBodyID(&manifold.getVertices()[2]).isInvalid());
    try expectEqual(@as(u32, 1), manifold.getNumSensorContacts());
    try expectEqual(@as(u32, 11), manifold.getSensorContactBodyID(0).getIndexAndSequenceNumber());
}
