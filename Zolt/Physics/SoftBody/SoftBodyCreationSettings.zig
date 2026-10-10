//! Port of: Jolt/Physics/SoftBody/SoftBodyCreationSettings.h, Jolt/Physics/SoftBody/SoftBodyCreationSettings.cpp
//! Status: stub
//!
//! Phase 9 (SoftBody) stub, created by Phase 5 F1 (SimulationArchitecture D1): `Body.getSoftBodyCreationSettings` /
//! `applySoftBodyCreationSettings` read and write it. It has Jolt's fields with Jolt's defaults, the constructors and
//! the owned references (`clone` / `deinit`); the binary state (`SaveBinaryState`, `RestoreBinaryState`,
//! `SaveWithChildren`, `sRestoreWithChildren`) and serialization come with Phase 9.

const Quat = @import("../../Math/Quat.zig").Quat;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const ObjectLayer = @import("../Collision/ObjectLayer.zig").ObjectLayer;
const CollisionGroup = @import("../Collision/CollisionGroup.zig").CollisionGroup;
const SoftBodySharedSettings = @import("SoftBodySharedSettings.zig").SoftBodySharedSettings;

/// This class contains the information needed to create a soft body object
/// Note: Soft bodies are still in development and come with several caveats. Read the Architecture and API documentation for more information!
pub const SoftBodyCreationSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL

    /// Defines the configuration of this soft body
    settings: RefConst(SoftBodySharedSettings) = .empty,

    /// Initial position of the soft body
    position: RVec3 = RVec3.zero(),
    /// Initial rotation of the soft body
    rotation: Quat = Quat.identity(),

    /// User data value (can be used by application)
    user_data: u64 = 0,

    ///@name Collision settings
    /// The collision layer this body belongs to (determines if two objects can collide)
    object_layer: ObjectLayer = 0,
    /// The collision group this body belongs to (determines if two objects can collide)
    collision_group: CollisionGroup = .{},

    /// Number of solver iterations
    num_iterations: u32 = 5,
    /// Linear damping: dv/dt = -mLinearDamping * v. Value should be zero or positive and is usually close to 0.
    linear_damping: f32 = 0.1,
    /// Maximum linear velocity that a vertex can reach (m/s)
    max_linear_velocity: f32 = 500.0,
    /// Restitution when colliding
    restitution: f32 = 0.0,
    /// Friction coefficient when colliding
    friction: f32 = 0.2,
    /// n * R * T, amount of substance * ideal gas constant * absolute temperature, see https://en.wikipedia.org/wiki/Pressure
    pressure: f32 = 0.0,
    /// Value to multiply gravity with for this body
    gravity_factor: f32 = 1.0,
    /// How big the particles are, can be used to push the vertices a little bit away from the surface of other bodies to prevent z-fighting
    vertex_radius: f32 = 0.0,
    /// Update the position of the body while simulating (set to false for something that is attached to the static world)
    update_position: bool = true,
    /// Bake specified mRotation in the vertices and set the body rotation to identity (simulation is slightly more accurate if the rotation of a soft body is kept to identity)
    make_rotation_identity: bool = true,
    /// If this body can go to sleep or not
    allow_sleeping: bool = true,
    /// If the faces in this soft body should be treated as double sided for the purpose of collision detection (ray cast / collide shape / cast shape)
    faces_double_sided: bool = false,

    /// Constructor SoftBodyCreationSettings(const SoftBodySharedSettings *, position, rotation, object layer)
    pub fn init(settings: ?*const SoftBodySharedSettings, position: RVec3, rotation: Quat, object_layer: ObjectLayer) SoftBodyCreationSettings {
        return .{ .settings = .init(settings), .position = position, .rotation = rotation, .object_layer = object_layer };
    }

    /// Copy constructor (adds references to the shared settings and the group filter)
    pub fn clone(self: *const SoftBodyCreationSettings) SoftBodyCreationSettings {
        var result = self.*;
        result.settings = self.settings.clone();
        result.collision_group = self.collision_group.clone();
        return result;
    }

    /// Destructor (releases the references)
    pub fn deinit(self: *SoftBodyCreationSettings) void {
        self.settings.deinit();
        self.collision_group.deinit();
    }
};

test "SoftBodyCreationSettings defaults and references" {
    const std = @import("std");
    var shared: SoftBodySharedSettings = .{};
    shared.setEmbedded();
    var settings = SoftBodyCreationSettings.init(&shared, RVec3.init(1, 2, 3), Quat.identity(), 4);
    defer settings.deinit();
    try std.testing.expectEqual(@as(u32, 5), settings.num_iterations);
    try std.testing.expectEqual(@as(f32, 0.1), settings.linear_damping);
    try std.testing.expect(settings.update_position and settings.make_rotation_identity and settings.allow_sleeping and !settings.faces_double_sided);
    var copy = settings.clone();
    try std.testing.expectEqual(@as(u32, 2 + 0x0ebedded), shared.ref_count.get());
    copy.deinit();
    try std.testing.expectEqual(@as(u32, 1 + 0x0ebedded), shared.ref_count.get());
}
