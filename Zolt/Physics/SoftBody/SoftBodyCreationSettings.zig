//! Port of: Jolt/Physics/SoftBody/SoftBodyCreationSettings.h, Jolt/Physics/SoftBody/SoftBodyCreationSettings.cpp
//! Status: partial
//! Missing: SaveBinaryState, RestoreBinaryState, SaveWithChildren, sRestoreWithChildren and the map types (group S2,
//!   RemainingFeaturesArchitecture.md R19)
//!
//! Created by Phase 5 F1 (SimulationArchitecture D1): `Body.getSoftBodyCreationSettings` / `applySoftBodyCreationSettings`
//! read and write it. It has Jolt's fields with Jolt's defaults, the constructors and the owned references (`clone` /
//! `deinit`).
//! - JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL: `rtti` (R19). `mObjectLayer` is an
//!   enum attribute, like BodyCreationSettings'. `mSettings` is a RefConst to SoftBodySharedSettings, a RefTarget without
//!   virtual functions: the stream declares it `instance SoftBodySharedSettings` and holds an identifier.

const std = @import("std");
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const Quat = @import("../../Math/Quat.zig").Quat;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const ObjectLayer = @import("../Collision/ObjectLayer.zig").ObjectLayer;
const CollisionGroup = @import("../Collision/CollisionGroup.zig").CollisionGroup;
const SoftBodySharedSettings = @import("SoftBodySharedSettings.zig").SoftBodySharedSettings;

/// This class contains the information needed to create a soft body object
/// Note: Soft bodies are still in development and come with several caveats. Read the Architecture and API documentation for more information!
pub const SoftBodyCreationSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SoftBodyCreationSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "SoftBodyCreationSettings",
        .kind = .non_virtual,
        .attributes = &.{
            .attr("mSettings", .settings),
            .attr("mPosition", .position),
            .attr("mRotation", .rotation),
            .attr("mUserData", .user_data),
            .enumAttr("mObjectLayer", .object_layer), // ObjectLayer is an integer: uint32, truncated when read
            .attr("mCollisionGroup", .collision_group),
            .attr("mNumIterations", .num_iterations),
            .attr("mLinearDamping", .linear_damping),
            .attr("mMaxLinearVelocity", .max_linear_velocity),
            .attr("mRestitution", .restitution),
            .attr("mFriction", .friction),
            .attr("mPressure", .pressure),
            .attr("mGravityFactor", .gravity_factor),
            .attr("mVertexRadius", .vertex_radius),
            .attr("mUpdatePosition", .update_position),
            .attr("mMakeRotationIdentity", .make_rotation_identity),
            .attr("mAllowSleeping", .allow_sleeping),
            .attr("mFacesDoubleSided", .faces_double_sided),
        },
    });

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
    var shared: SoftBodySharedSettings = try .init(std.testing.allocator);
    defer shared.deinit();
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

/// Creation settings whose shared settings have data in every serialized array and a material of their own
fn testObjectStreamSettings(allocator: std.mem.Allocator) !SoftBodyCreationSettings {
    const Color = @import("../../Core/Color.zig").Color;
    const PhysicsMaterialSimple = @import("../Collision/PhysicsMaterialSimple.zig").PhysicsMaterialSimple;
    const GroupFilterTable = @import("../Collision/GroupFilterTable.zig").GroupFilterTable;
    const Mat44 = @import("../../Math/Mat44.zig").Mat44;
    const S = SoftBodySharedSettings;

    const shared = try S.create(allocator);
    var settings: SoftBodyCreationSettings = .init(shared, RVec3.init(1, 2, 3), Quat.rotation(.axisX(), 0.25), 3);
    errdefer settings.deinit();
    try shared.vertices.append(allocator, .init(.{ .x = 1, .y = 2, .z = 3 }, .{ .velocity = .{ .x = 0.5, .y = 0, .z = 0 }, .inv_mass = 0.25 }));
    try shared.vertices.append(allocator, .init(.{ .x = -1, .y = 0, .z = 0.5 }, .{}));
    try shared.vertices.append(allocator, .init(.{ .x = 0, .y = 1, .z = 0 }, .{}));
    try shared.addFace(.init(0, 1, 2, .{ .material_index = 1 }));
    try shared.edge_constraints.append(allocator, .init(0, 1, .{ .compliance = 0.125 }));
    try shared.dihedral_bend_constraints.append(allocator, .init(0, 1, 2, 2, .{ .compliance = 1.0e-4 }));
    try shared.volume_constraints.append(allocator, .init(0, 1, 2, 0, .{}));
    var skinned: S.Skinned = .init(2, 0.5, 0.25, 9.0);
    skinned.weights[0] = .init(0, 1.0);
    try shared.skinned_constraints.append(allocator, skinned);
    try shared.inv_bind_matrices.append(allocator, .init(4, Mat44.scale(3.0)));
    try shared.lra_constraints.append(allocator, .init(0, 2, 1.75));
    try shared.rod_stretch_shear_constraints.append(allocator, .init(1, 2, .{ .compliance = 0.5 }));
    try shared.rod_bend_twist_constraints.append(allocator, .init(0, 0, .{}));
    try shared.materials.ensureUnusedCapacity(allocator, 1); // No allocation between create and the reference
    const material = try PhysicsMaterialSimple.create(allocator, "Rubber", Color.init(1, 2, 3, 4));
    shared.materials.appendAssumeCapacity(.init(material.material()));
    settings.pressure = 2.5;
    settings.faces_double_sided = true;
    const filter = try GroupFilterTable.create(allocator, .{ .num_sub_groups = 2 });
    settings.collision_group.setGroupFilter(filter.asGroupFilter());
    return settings;
}

/// sWriteObject into an allocating writer: the writer fails only when it runs out of memory
fn writeObject(allocator: std.mem.Allocator, writer: *std.Io.Writer, stream_type: @import("../../ObjectStream/ObjectStream.zig").StreamType, settings: *const SoftBodyCreationSettings) !void {
    const ObjectStreamOut = @import("../../ObjectStream/ObjectStreamOut.zig").ObjectStreamOut;
    ObjectStreamOut.sWriteObject(SoftBodyCreationSettings, allocator, writer, stream_type, settings) catch |err| return switch (err) {
        error.WriteFailed => error.OutOfMemory,
        else => err,
    };
}

fn writeReadObjectStream(allocator: std.mem.Allocator) !void {
    const ObjectStreamIn = @import("../../ObjectStream/ObjectStreamIn.zig").ObjectStreamIn;
    const StreamType = @import("../../ObjectStream/ObjectStream.zig").StreamType;
    const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;

    var settings = try testObjectStreamSettings(allocator);
    defer settings.deinit();
    const shared = settings.settings.get().?;
    for ([_]StreamType{ .text, .binary }) |stream_type| {
        var out: std.Io.Writer.Allocating = .init(allocator);
        defer out.deinit();
        try writeObject(allocator, &out.writer, stream_type, &settings);

        var reader: std.Io.Reader = .fixed(out.written());
        const read = try ObjectStreamIn.sReadObject(SoftBodyCreationSettings, allocator, &reader, .{});
        defer RTTI.of(SoftBodyCreationSettings).destructObject(read, allocator);

        // The Ref is the only holder of the new shared settings (no reference of the stream is left)
        const read_shared = read.settings.get().?;
        try std.testing.expect(read_shared != shared);
        try std.testing.expectEqual(@as(u32, 1), read_shared.getRefCount());
        try std.testing.expectEqual(@as(f32, 2.5), read.pressure);
        try std.testing.expectEqual(@as(ObjectLayer, 3), read.object_layer);
        try std.testing.expect(read.faces_double_sided and read.collision_group.group_filter.get() != null);

        // Every serialized array came back (the binary state writes them all); the private arrays are not serialized
        try std.testing.expectEqual(@as(usize, 2), read_shared.materials.items.len);
        try std.testing.expectEqualStrings("Rubber", read_shared.materials.items[1].get().?.getDebugName());
        var expected: std.Io.Writer.Allocating = .init(allocator);
        defer expected.deinit();
        var expected_wrapper: StreamOutWrapper = .init(&expected.writer);
        shared.saveBinaryState(expected_wrapper.streamOut());
        var actual: std.Io.Writer.Allocating = .init(allocator);
        defer actual.deinit();
        var actual_wrapper: StreamOutWrapper = .init(&actual.writer);
        read_shared.saveBinaryState(actual_wrapper.streamOut());
        if (expected_wrapper.isFailed() or actual_wrapper.isFailed()) return error.OutOfMemory; // Only out of memory fails the allocating writers
        try std.testing.expectEqualSlices(u8, expected.written(), actual.written());

        // Written again: the same bytes
        var again: std.Io.Writer.Allocating = .init(allocator);
        defer again.deinit();
        try writeObject(allocator, &again.writer, stream_type, read);
        try std.testing.expectEqualSlices(u8, out.written(), again.written());
    }
}

test "SoftBodyCreationSettings: object stream of the shared settings (a Ref to a RefTarget without virtual functions)" {
    try writeReadObjectStream(std.testing.allocator);
}

test "SoftBodyCreationSettings: out of memory in the object stream" {
    try std.testing.checkAllAllocationFailures(std.testing.allocator, writeReadObjectStream, .{});
}
