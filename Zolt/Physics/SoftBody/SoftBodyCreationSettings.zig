//! Port of: Jolt/Physics/SoftBody/SoftBodyCreationSettings.h, Jolt/Physics/SoftBody/SoftBodyCreationSettings.cpp
//! Status: complete
//!
//! Created by Phase 5 F1 (SimulationArchitecture D1): `Body.getSoftBodyCreationSettings` / `applySoftBodyCreationSettings`
//! read and write it. It has Jolt's fields with Jolt's defaults, the constructors and the owned references (`clone` /
//! `deinit`).
//! - JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL: `rtti` (R19). `mObjectLayer` is an
//!   enum attribute, like BodyCreationSettings'. `mSettings` is a RefConst to SoftBodySharedSettings, a RefTarget without
//!   virtual functions: the stream declares it `instance SoftBodySharedSettings` and holds an identifier.
//! - The binary state (`saveBinaryState`, `restoreBinaryState`, `saveWithChildren`, `restoreWithChildren`) writes
//!   Jolt's bytes, statement by statement (RemainingFeaturesArchitecture.md R19). `saveWithChildren` /
//!   `restoreWithChildren` use the ID maps of SoftBodySharedSettings.saveWithMaterials / restoreWithMaterials and of
//!   StreamUtils (the group filter). They return Jolt's `SBCSResult` value with Jolt's error texts; allocation failure
//!   is `error.OutOfMemory`.
//! - Zolt: `restoreBinaryState` reads the bools as bytes (`!= 0`), like BodyCreationSettings. Jolt stores any byte.
//! - Zolt (R20 UB6): `saveWithChildren` with both maps and without shared settings panics with a message before it
//!   writes anything. Jolt dereferences the null `mSettings`.

const std = @import("std");
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const Quat = @import("../../Math/Quat.zig").Quat;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const ObjectLayer = @import("../Collision/ObjectLayer.zig").ObjectLayer;
const CollisionGroup = @import("../Collision/CollisionGroup.zig").CollisionGroup;
const SoftBodySharedSettings = @import("SoftBodySharedSettings.zig").SoftBodySharedSettings;
const Allocator = std.mem.Allocator;
const Result = @import("../../Core/Result.zig").Result;
const ObjectToIDMap = @import("../../Core/ObjectToIDMap.zig").ObjectToIDMap;
const IDToObjectMap = @import("../../Core/ObjectToIDMap.zig").IDToObjectMap;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const StreamUtils = @import("../../Core/StreamUtils.zig");
const GroupFilter = @import("../Collision/GroupFilter.zig").GroupFilter;
const PhysicsMaterial = @import("../Collision/PhysicsMaterial.zig").PhysicsMaterial;

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

    pub const GroupFilterToIDMap = ObjectToIDMap(GroupFilter);
    pub const IDToGroupFilterMap = IDToObjectMap(GroupFilter);
    pub const SharedSettingsToIDMap = SoftBodySharedSettings.SharedSettingsToIDMap;
    pub const IDToSharedSettingsMap = SoftBodySharedSettings.IDToSharedSettingsMap;
    pub const MaterialToIDMap = ObjectToIDMap(PhysicsMaterial);
    pub const IDToMaterialMap = IDToObjectMap(PhysicsMaterial);

    pub const SBCSResult = Result(SoftBodyCreationSettings);

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

    /// Saves the state of this object in binary form to inStream. Doesn't store the shared settings nor the group filter.
    pub fn saveBinaryState(self: *const SoftBodyCreationSettings, stream: StreamOut) void {
        stream.write(self.position);
        stream.write(self.rotation);
        stream.write(self.user_data);
        stream.write(self.object_layer);
        self.collision_group.saveBinaryState(stream);
        stream.write(self.num_iterations);
        stream.write(self.linear_damping);
        stream.write(self.max_linear_velocity);
        stream.write(self.restitution);
        stream.write(self.friction);
        stream.write(self.pressure);
        stream.write(self.gravity_factor);
        stream.write(self.vertex_radius);
        stream.write(self.update_position);
        stream.write(self.make_rotation_identity);
        stream.write(self.allow_sleeping);
        stream.write(self.faces_double_sided);
    }

    /// Restore the state of this object from inStream. Doesn't restore the shared settings nor the group filter.
    pub fn restoreBinaryState(self: *SoftBodyCreationSettings, stream: StreamIn) void {
        stream.read(&self.position);
        stream.read(&self.rotation);
        stream.read(&self.user_data);
        stream.read(&self.object_layer);
        self.collision_group.restoreBinaryState(stream);
        stream.read(&self.num_iterations);
        stream.read(&self.linear_damping);
        stream.read(&self.max_linear_velocity);
        stream.read(&self.restitution);
        stream.read(&self.friction);
        stream.read(&self.pressure);
        stream.read(&self.gravity_factor);
        stream.read(&self.vertex_radius);
        readBool(stream, &self.update_position);
        readBool(stream, &self.make_rotation_identity);
        readBool(stream, &self.allow_sleeping);
        readBool(stream, &self.faces_double_sided);
    }

    /// Read a bool as a byte (Jolt reads the raw byte into its bool)
    fn readBool(stream: StreamIn, value: *bool) void {
        var raw: u8 = @intFromBool(value.*);
        stream.read(&raw);
        value.* = raw != 0;
    }

    /// Save this body creation settings, its shared settings and group filter. Pass in an empty map in settings_map / material_map / group_filter_map or reuse the same map while saving multiple shapes to the same stream in order to avoid writing duplicates.
    /// Pass null to settings_map and material_map to skip saving shared settings and materials
    /// Pass null to group_filter_map to skip saving group filters
    /// (Zolt: `allocator` grows the maps. With both maps and without shared settings, Jolt dereferences the null
    /// mSettings: Zolt panics before it writes anything, R20 UB6.)
    pub fn saveWithChildren(self: *const SoftBodyCreationSettings, allocator: Allocator, stream: StreamOut, settings_map: ?*SharedSettingsToIDMap, material_map: ?*MaterialToIDMap, group_filter_map: ?*GroupFilterToIDMap) Allocator.Error!void {
        // Zolt (UB6): check the shared settings before anything is written
        const save_settings = settings_map != null and material_map != null;
        if (save_settings and self.settings.get() == null)
            @panic("SoftBodyCreationSettings.saveWithChildren: no shared settings to save (Jolt dereferences a null mSettings)");

        // Save creation settings
        self.saveBinaryState(stream);

        // Save shared settings
        if (save_settings)
            try self.settings.get().?.saveWithMaterials(allocator, stream, settings_map.?, material_map.?)
        else
            stream.write(~@as(u32, 0));

        // Save group filter
        try StreamUtils.saveObjectReference(GroupFilter, allocator, stream, self.collision_group.getGroupFilter(), group_filter_map);
    }

    /// Restore a shape, all its children and materials. Pass in an empty map in settings_map / material_map / group_filter_map or reuse the same map while reading multiple shapes from the same stream in order to restore duplicates.
    /// (Jolt: sRestoreWithChildren. The result owns the settings, `deinit` it.)
    pub fn restoreWithChildren(allocator: Allocator, stream: StreamIn, settings_map: *IDToSharedSettingsMap, material_map: *IDToMaterialMap, group_filter_map: *IDToGroupFilterMap) Allocator.Error!SBCSResult {
        var result: SBCSResult = .empty;

        // Read creation settings
        var settings: SoftBodyCreationSettings = .{};
        defer settings.deinit();
        settings.restoreBinaryState(stream);
        if (stream.isEOF() or stream.isFailed()) {
            result.setError("Error reading body creation settings");
            return result;
        }

        // Read shared settings
        var settings_result = try SoftBodySharedSettings.restoreWithMaterials(allocator, stream, settings_map, material_map);
        defer settings_result.deinit();
        if (settings_result.hasError()) {
            result.setError(settings_result.getError());
            return result;
        }
        settings.settings.set(settings_result.getPtr());

        // Read group filter
        var gfresult = try StreamUtils.restoreObjectReference(GroupFilter, allocator, stream, group_filter_map);
        defer gfresult.deinit();
        if (gfresult.hasError()) {
            result.setError(gfresult.getError());
            return result;
        }
        settings.collision_group.setGroupFilter(gfresult.getPtr());

        result.set(settings.clone());
        return result;
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

// ---------------------------------------------------------------------------------------------------------------------
// Tests of the binary state

const testing = std.testing;
const Real = @import("../../Math/Real.zig").Real;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");
const GroupFilterTable = @import("../Collision/GroupFilterTable.zig").GroupFilterTable;

/// Settings with every saved field different from its default (no shared settings, no group filter)
fn testSettings() SoftBodyCreationSettings {
    return .{
        .position = RVec3.init(1, 2, 3),
        .rotation = Quat.init(0.5, -0.5, 0.5, 0.5),
        .user_data = 0x1234_5678_9abc_def0,
        .object_layer = 3,
        .collision_group = .init(null, 7, 9),
        .num_iterations = 11,
        .linear_damping = 0.5,
        .max_linear_velocity = 100,
        .restitution = 0.25,
        .friction = 0.75,
        .pressure = 1000,
        .gravity_factor = 2,
        .vertex_radius = 0.125,
        .update_position = false,
        .make_rotation_identity = false,
        .allow_sleeping = false,
        .faces_double_sided = true,
    };
}

/// Size of the binary state: position, rotation, user data, object layer, group ids, the iterations, 7 floats and 4
/// bools
const binary_state_size = 3 * @sizeOf(Real) + 16 + 8 + @sizeOf(ObjectLayer) + 8 + 4 + 7 * 4 + 4;

/// True when the saved fields of `a` and `b` are equal (not the shared settings and the group filter)
fn sameSavedFields(a: *const SoftBodyCreationSettings, b: *const SoftBodyCreationSettings) bool {
    return a.position.eql(b.position) and a.rotation.eql(b.rotation) and a.user_data == b.user_data and a.object_layer == b.object_layer and
        a.collision_group.getGroupID() == b.collision_group.getGroupID() and a.collision_group.getSubGroupID() == b.collision_group.getSubGroupID() and
        a.num_iterations == b.num_iterations and a.linear_damping == b.linear_damping and a.max_linear_velocity == b.max_linear_velocity and
        a.restitution == b.restitution and a.friction == b.friction and a.pressure == b.pressure and a.gravity_factor == b.gravity_factor and
        a.vertex_radius == b.vertex_radius and a.update_position == b.update_position and a.make_rotation_identity == b.make_rotation_identity and
        a.allow_sleeping == b.allow_sleeping and a.faces_double_sided == b.faces_double_sided;
}

test "SoftBodyCreationSettings: saveBinaryState / restoreBinaryState" {
    var settings = testSettings();
    defer settings.deinit();

    var buffer: [256]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamWrapper.StreamOutWrapper.init(&writer);
    settings.saveBinaryState(out.streamOut());
    const bytes = writer.buffered();
    try testing.expectEqual(@as(usize, binary_state_size), bytes.len);
    try testing.expectEqual(@as(u8, 1), bytes[binary_state_size - 1]); // mFacesDoubleSided last

    // Restore into default settings: every saved field
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    var restored: SoftBodyCreationSettings = .{};
    defer restored.deinit();
    restored.restoreBinaryState(in.streamIn());
    try testing.expect(!in.streamIn().isEOF() and !in.streamIn().isFailed());
    try testing.expect(sameSavedFields(&restored, &settings));

    // A bool byte that is not 0 or 1 is true (Zolt: Jolt stores the byte)
    var odd = buffer;
    odd[binary_state_size - 4] = 2;
    var odd_reader: std.Io.Reader = .fixed(odd[0..bytes.len]);
    var odd_in = StreamWrapper.StreamInWrapper.init(&odd_reader);
    var odd_restored: SoftBodyCreationSettings = .{};
    defer odd_restored.deinit();
    odd_restored.restoreBinaryState(odd_in.streamIn());
    try testing.expect(odd_restored.update_position);
}

/// The maps of a restore, released by deinit
const RestoreMaps = struct {
    settings: SoftBodyCreationSettings.IDToSharedSettingsMap = .empty,
    materials: SoftBodyCreationSettings.IDToMaterialMap = .empty,
    group_filters: SoftBodyCreationSettings.IDToGroupFilterMap = .empty,

    fn deinit(self: *RestoreMaps, allocator: Allocator) void {
        for (self.settings.items) |*s| s.deinit();
        self.settings.deinit(allocator);
        for (self.materials.items) |*m| m.deinit();
        self.materials.deinit(allocator);
        for (self.group_filters.items) |*g| g.deinit();
        self.group_filters.deinit(allocator);
    }
};

/// The maps of a save
const SaveMaps = struct {
    settings: SoftBodyCreationSettings.SharedSettingsToIDMap = .empty,
    materials: SoftBodyCreationSettings.MaterialToIDMap = .empty,
    group_filters: SoftBodyCreationSettings.GroupFilterToIDMap = .empty,

    fn deinit(self: *SaveMaps, allocator: Allocator) void {
        self.settings.deinit(allocator);
        self.materials.deinit(allocator);
        self.group_filters.deinit(allocator);
    }
};

/// Three settings: the first two share their shared settings and a group filter, the third has other shared settings
/// that share the material of the first
const ChildrenScene = struct {
    settings: [3]SoftBodyCreationSettings = @splat(.{}),

    fn init(self: *ChildrenScene, allocator: Allocator) !void {
        errdefer self.deinit();
        self.settings[0] = try testObjectStreamSettings(allocator);
        self.settings[1] = self.settings[0].clone();
        self.settings[1].position = RVec3.init(-4, 5, 6);
        self.settings[1].collision_group.setSubGroupID(1);
        const first = self.settings[0].settings.get().?;
        const other = try SoftBodySharedSettings.create(allocator);
        self.settings[2] = testSettings();
        self.settings[2].settings = .init(other);
        try other.vertices.append(allocator, .init(.{ .x = 0, .y = 0, .z = 1 }, .{}));
        try other.materials.ensureUnusedCapacity(allocator, 1);
        other.materials.items[0].deinit();
        other.materials.items[0] = first.materials.items[1].clone();
    }

    fn deinit(self: *ChildrenScene) void {
        for (&self.settings) |*s| s.deinit();
    }
};

/// saveWithChildren of the settings into `out` (`with_settings` / `with_filters` select the maps)
fn saveChildren(allocator: Allocator, settings: []const SoftBodyCreationSettings, out: *std.Io.Writer.Allocating, maps: *SaveMaps, with_settings: bool, with_filters: bool) ![]const u8 {
    var wrapper = StreamWrapper.StreamOutWrapper.init(&out.writer);
    for (settings) |*s|
        try s.saveWithChildren(allocator, wrapper.streamOut(), if (with_settings) &maps.settings else null, if (with_settings) &maps.materials else null, if (with_filters) &maps.group_filters else null);
    if (wrapper.isFailed()) return error.OutOfMemory; // The allocating writer only fails when it runs out of memory
    return out.written();
}

/// restoreWithChildren of `out.len` settings, every result must be valid
fn restoreChildren(allocator: Allocator, bytes: []const u8, maps: *RestoreMaps, out: []SoftBodyCreationSettings) !void {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    for (out) |*o| {
        var result = try SoftBodyCreationSettings.restoreWithChildren(allocator, in.streamIn(), &maps.settings, &maps.materials, &maps.group_filters);
        defer result.deinit();
        if (!result.isValid()) return error.TestUnexpectedResult;
        o.deinit();
        o.* = result.get().clone();
    }
}

test "SoftBodyCreationSettings: saveWithChildren / restoreWithChildren with shared settings, materials and group filters" {
    const allocator = testing.allocator;
    var scene: ChildrenScene = .{};
    try scene.init(allocator);
    defer scene.deinit();

    // Save with all maps: the shared settings, the materials and the group filter are written once
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var save_maps: SaveMaps = .{};
    defer save_maps.deinit(allocator);
    const bytes = try saveChildren(allocator, &scene.settings, &out, &save_maps, true, true);
    try testing.expectEqual(@as(u32, 2), save_maps.settings.count());
    try testing.expectEqual(@as(u32, 2), save_maps.materials.count()); // The default material and rubber
    try testing.expectEqual(@as(u32, 1), save_maps.group_filters.count());

    var maps: RestoreMaps = .{};
    defer maps.deinit(allocator);
    var restored: [3]SoftBodyCreationSettings = @splat(.{});
    defer for (&restored) |*r| r.deinit();
    try restoreChildren(allocator, bytes, &maps, &restored);
    try testing.expectEqual(@as(usize, 2), maps.settings.items.len);
    try testing.expectEqual(@as(usize, 2), maps.materials.items.len);
    try testing.expectEqual(@as(usize, 1), maps.group_filters.items.len);

    // The values and the sharing
    for (&restored, &scene.settings) |*r, *original| try testing.expect(sameSavedFields(r, original));
    const shared = restored[0].settings.get().?;
    try testing.expect(shared == restored[1].settings.get().?);
    try testing.expect(shared != scene.settings[0].settings.get().?);
    try testing.expect(restored[2].settings.get().? != shared);
    try testing.expect(restored[2].settings.get().?.materials.items[0].get().? == shared.materials.items[1].get().?);
    try testing.expectEqualStrings("Rubber", shared.materials.items[1].get().?.getDebugName());
    try testing.expect(restored[0].collision_group.getGroupFilter().? == restored[1].collision_group.getGroupFilter().?);
    try testing.expect(restored[2].collision_group.getGroupFilter() == null);

    // Saved again: the same bytes
    var out2: std.Io.Writer.Allocating = .init(allocator);
    defer out2.deinit();
    var save_maps2: SaveMaps = .{};
    defer save_maps2.deinit(allocator);
    try testing.expectEqualSlices(u8, bytes, try saveChildren(allocator, &restored, &out2, &save_maps2, true, true));

    // Without maps: no shared settings and no group filters (also for settings without shared settings)
    var no_settings = testSettings();
    defer no_settings.deinit();
    const all = [_]SoftBodyCreationSettings{ scene.settings[0], scene.settings[1], scene.settings[2], no_settings };
    var out3: std.Io.Writer.Allocating = .init(allocator);
    defer out3.deinit();
    var save_maps3: SaveMaps = .{};
    defer save_maps3.deinit(allocator);
    const bytes3 = try saveChildren(allocator, &all, &out3, &save_maps3, false, false);
    try testing.expectEqual(4 * (binary_state_size + 8), bytes3.len);
    var maps3: RestoreMaps = .{};
    defer maps3.deinit(allocator);
    var restored3: [4]SoftBodyCreationSettings = @splat(.{});
    defer for (&restored3) |*r| r.deinit();
    try restoreChildren(allocator, bytes3, &maps3, &restored3);
    for (&restored3, &all) |*r, *original| {
        try testing.expect(sameSavedFields(r, original));
        try testing.expect(r.settings.get() == null and r.collision_group.getGroupFilter() == null);
    }
}

test "SoftBodyCreationSettings: restoreWithChildren errors" {
    const allocator = testing.allocator;
    var scene: ChildrenScene = .{};
    try scene.init(allocator);
    defer scene.deinit();
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var save_maps: SaveMaps = .{};
    defer save_maps.deinit(allocator);
    const bytes = try saveChildren(allocator, scene.settings[0..1], &out, &save_maps, true, true);

    // Truncated streams fail with one of Jolt's texts at every length, nothing leaks
    for (0..bytes.len) |len| {
        var reader: std.Io.Reader = .fixed(bytes[0..len]);
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        var maps: RestoreMaps = .{};
        defer maps.deinit(allocator);
        var result = try SoftBodyCreationSettings.restoreWithChildren(allocator, in.streamIn(), &maps.settings, &maps.materials, &maps.group_filters);
        defer result.deinit();
        if (len < binary_state_size) {
            try testing.expectEqualStrings("Error reading body creation settings", result.getError());
        } else if (len < binary_state_size + 4) {
            try testing.expectEqualStrings("Failed to read settings id", result.getError());
        } else {
            // Shared settings that cannot be read, or a group filter that cannot be read (Jolt does not check the EOF
            // of the filter ID, a missing ID is the null filter)
            try testing.expect(result.hasError() or result.get().collision_group.getGroupFilter() == null);
        }
    }

    // A group filter of an unknown type: the RTTI hash follows the filter ID 0 at the end of the stream
    const filter_hash_offset = blk: {
        const hash = GroupFilterTable.rtti.hash;
        var i: usize = bytes.len - 4;
        while (i > binary_state_size) : (i -= 1)
            if (std.mem.readInt(u32, bytes[i..][0..4], .little) == hash and std.mem.readInt(u32, bytes[i - 4 ..][0..4], .little) == 0) break :blk i;
        return error.TestUnexpectedResult;
    };
    const corrupt = try allocator.dupe(u8, bytes);
    defer allocator.free(corrupt);
    std.mem.writeInt(u32, corrupt[filter_hash_offset..][0..4], 0x12345678, .little);
    {
        var reader: std.Io.Reader = .fixed(corrupt);
        var in = StreamWrapper.StreamInWrapper.init(&reader);
        var maps: RestoreMaps = .{};
        defer maps.deinit(allocator);
        var result = try SoftBodyCreationSettings.restoreWithChildren(allocator, in.streamIn(), &maps.settings, &maps.materials, &maps.group_filters);
        defer result.deinit();
        try testing.expectEqualStrings("Failed to create instance of type", result.getError());
    }
}

fn saveRestoreChildren(allocator: Allocator) !void {
    var scene: ChildrenScene = .{};
    try scene.init(allocator);
    defer scene.deinit();
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var save_maps: SaveMaps = .{};
    defer save_maps.deinit(allocator);
    const bytes = try saveChildren(allocator, &scene.settings, &out, &save_maps, true, true);
    var maps: RestoreMaps = .{};
    defer maps.deinit(allocator);
    var restored: [3]SoftBodyCreationSettings = @splat(.{});
    defer for (&restored) |*r| r.deinit();
    try restoreChildren(allocator, bytes, &maps, &restored);
}

test "SoftBodyCreationSettings: out of memory in saveWithChildren / restoreWithChildren" {
    try testing.checkAllAllocationFailures(testing.allocator, saveRestoreChildren, .{});
}
