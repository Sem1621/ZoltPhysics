//! Parity tests of Phase 5 Waves A6 / A8: BodyInterface, the binary state of BodyCreationSettings and PhysicsScene.
//! C ABI wrappers: ZoltParity/Physics/BodyInterfaceReference.cpp.
//!
//! - "BodyInterface parity: random operations": a world of bodies on both sides (Zolt: a PhysicsSystem with the brute
//!   force broad phase and the layers of the unit tests; C++: a BodyManager, a BroadPhaseBruteForce and a
//!   BodyInterface initialized on them, so even the collection order of ActivateBodiesInAABox is the same). Every
//!   BodyInterface function (`OpKind`, and the creation functions in `createBody`) is applied in random order to live,
//!   removed, destroyed (stale) and invalid body IDs, with random values mixed with edge cases (unchanged positions,
//!   velocities near zero and above the maximum, kinematic moves, non uniformly scaled boxes, a body manager that runs
//!   out of bodies, custom IDs that are taken). Compared after every operation: the returned values (`Out`, pre-filled with a
//!   sentinel), and a snapshot of the world: per body (in ID order) the binary state of GetBodyCreationSettings (every
//!   setting, the mass and inverted inertia), the user data, the shape, IsActive, IsInBroadPhase, the bounds, the
//!   center of mass and the broad phase layer; the active body list in its raw order (the activation order); and
//!   BodyManager::SaveState.
//! - "BodyCreationSettings parity: binary state": random lists of settings (every field, shapes from a shared table
//!   with shared materials, shape settings that GetShape creates, shared group filters) saved with SaveWithChildren on
//!   both sides with and without the shape / group filter maps; the bytes must be equal, and each side restores the
//!   other side's bytes with sRestoreWithChildren and saves them again, which must reproduce the bytes. Truncated
//!   streams must give the same error text (or the same result) on both sides.
//! - "PhysicsScene parity: binary state and CreateBodies": the same for PhysicsScene::SaveBinaryState /
//!   sRestoreFromBinaryState (bodies and constraints: random constraint settings between the bodies of the scene and
//!   the world, of every kind whose Zolt class is ported, `scene_constraint_kinds`: the filled slots of
//!   ConstraintsParity.zig, whose descriptions `SceneConstraint` uses, so a Phase 6 porter adds nothing here), and
//!   FixInvalidScales + CreateBodies of a restored scene (the scaled sphere of the table gets a non uniform scale in
//!   half of the tests, which FixInvalidScales turns into a compound; compared: its result, the fixed scene's bytes
//!   with shapes, the bodies, SaveState of the bodies and the constraints, and the scene that FromPhysicsSystem makes of
//!   the system (the constraints through GetConstraintSettings); C++: a PhysicsSystem with its QuadTree, the active
//!   bodies are compared sorted, because QuadTree's AddBodiesPrepare sorts the bodies by layer), also when the system
//!   runs out of bodies.
//! - Soft bodies (Phase 9 group S2): the shared settings of a test are SaveBinaryState bytes of settings that Zolt
//!   builds (cubes, and cloths with createConstraints and optimize), with materials of the material table; both sides
//!   restore them. "SoftBodyCreationSettings parity: binary state" is the BodyCreationSettings test for soft bodies
//!   (SaveWithChildren with and without the maps, the cross restore, truncated streams). "SoftBodyCreationSettings
//!   parity: soft bodies created through BodyInterface" creates soft bodies with the four creation functions in a world
//!   that can run out of bodies, and compares the IDs, IsActive, IsAdded and the SaveWithChildren bytes of
//!   GetSoftBodyCreationSettings, with and without the maps. The scenes of "PhysicsScene parity" have soft bodies:
//!   SaveBinaryState (soft bodies always save their shared settings and materials), sRestoreFromBinaryState, CreateBodies
//!   (the active soft bodies; SaveState skips the soft bodies, whose state SoftBodyParity.zig and SimulationParity.zig
//!   compare) and FromPhysicsSystem. Constraints also connect soft bodies. FromPhysicsSystem stores the index of the
//!   next rigid body for a soft body (Jolt's quirk, R20), so its bytes compare that index.
//! The DeterminismLog text is not compared here: the C++ library is built without JPH_ENABLE_DETERMINISM_LOG (see
//! ZoltTests/Physics/DeterminismLogTests.zig).

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");
const TestLayers = @import("test_layers");
const ConstraintsParity = @import("ConstraintsParity.zig");

const Checker = fw.Checker;
const Rng = fw.Rng;

const Allocator = std.mem.Allocator;
const Core = zolt.Core;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const RVec3 = zolt.RVec3;
const RMat44 = zolt.RMat44;
const AABox = zolt.AABox;
const Color = zolt.Color;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyIDVector = zolt.BodyIDVector;
const BodyInterface = zolt.BodyInterface;
const BodyManager = zolt.BodyManager;
const BodyCreationSettings = zolt.BodyCreationSettings;
const Activation = zolt.Activation;
const MotionType = zolt.MotionType;
const AllowedDOFs = zolt.AllowedDOFs;
const ObjectLayer = zolt.ObjectLayer;
const CollisionGroup = zolt.CollisionGroup;
const GroupFilter = zolt.GroupFilter;
const GroupFilterTable = zolt.GroupFilterTable;
const PhysicsMaterial = zolt.PhysicsMaterial;
const PhysicsMaterialSimple = zolt.PhysicsMaterialSimple;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const SubShapeID = zolt.SubShapeID;
const SphereShapeSettings = zolt.SphereShapeSettings;
const BoxShapeSettings = zolt.BoxShapeSettings;
const CapsuleShapeSettings = zolt.CapsuleShapeSettings;
const OffsetCenterOfMassShapeSettings = zolt.OffsetCenterOfMassShapeSettings;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const ScaledShapeSettings = zolt.ScaledShapeSettings;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const PhysicsScene = zolt.PhysicsScene;
const StateRecorder = zolt.StateRecorder;
const StateRecorderImpl = zolt.StateRecorderImpl;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;
const SoftBodySharedSettings = zolt.SoftBodySharedSettings;
const SoftBodyCreationSettings = zolt.SoftBodyCreationSettings;
const StateRecorderFilter = zolt.StateRecorderFilter;

const io = std.testing.io;

/// The C++ reference functions, see BodyInterfaceReference.cpp
const jolt = struct {
    extern fn jolt_bi_struct_sizes(out_sizes: *[9]u32) void;
    extern fn jolt_bi_world_create(max_bodies: u32, num_body_mutexes: u32, tables: *const Tables) *anyopaque;
    extern fn jolt_bi_world_destroy(world: *anyopaque) void;
    extern fn jolt_bi_create_body(world: *anyopaque, desc: *const BodyDesc, mode: u32, custom_id: u32, activation: u32) u32;
    extern fn jolt_bi_op(world: *anyopaque, op: *const Op, out: *Out) void;
    extern fn jolt_bi_snapshot(world: *anyopaque, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_bi_bcs_save(tables: *const Tables, bodies: [*]const BodyDesc, num_bodies: u32, save_shapes: u32, save_group_filters: u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_bi_bcs_resave(bytes: [*]const u8, size: u32, num_bodies: u32, save_shapes: u32, save_group_filters: u32, out_bytes: [*]u8, capacity: u32, out_error: [*]u8, error_capacity: u32) u32;
    extern fn jolt_bi_scene_save(tables: *const Tables, bodies: [*]const BodyDesc, num_bodies: u32, constraint_bodies: [*]const [2]u32, constraint_settings: [*]const ConstraintsParity.CSSettings, num_constraints: u32, shared: [*]const SharedSettingsDesc, num_shared: u32, soft_bodies: [*]const SoftBodyDesc, num_soft_bodies: u32, save_shapes: u32, save_group_filters: u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_bi_scene_resave(bytes: [*]const u8, size: u32, save_shapes: u32, save_group_filters: u32, out_bytes: [*]u8, capacity: u32, out_error: [*]u8, error_capacity: u32) u32;
    extern fn jolt_bi_scene_create_bodies(bytes: [*]const u8, size: u32, max_bodies: u32, out_result: *u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_bi_sb_save(tables: *const Tables, shared: [*]const SharedSettingsDesc, num_shared: u32, bodies: [*]const SoftBodyDesc, num_bodies: u32, save_settings: u32, save_group_filters: u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_bi_sb_resave(bytes: [*]const u8, size: u32, num_bodies: u32, save_settings: u32, save_group_filters: u32, out_bytes: [*]u8, capacity: u32, out_error: [*]u8, error_capacity: u32) u32;
    extern fn jolt_bi_sb_create(tables: *const Tables, shared: [*]const SharedSettingsDesc, num_shared: u32, bodies: [*]const SoftBodyDesc, num_bodies: u32, max_bodies: u32, save_settings: u32, save_group_filters: u32, out_bytes: [*]u8, capacity: u32) u32;
};

/// No index (cBINone)
const none: u32 = 0xffffffff;
/// GetMaterial returned PhysicsMaterial::sDefault (cBIDefaultMaterial)
const default_material_index: u32 = 0xfffffffe;

/// Must match BIShapeDesc in BodyInterfaceReference.cpp
const ShapeDesc = extern struct {
    /// 0 sphere, 1 box, 2 capsule, 3 offset COM box, 4 rotated translated box, 5 scaled box, 6 static compound of a box and a sphere, 7 scaled sphere
    kind: u32 = 0,
    /// Index in the material table or `none`
    material: u32 = none,
    /// 0..2 half extent / radius / half height, 3 convex radius, 4..6 offset / translation / scale
    params: [8]f32 = @splat(0),
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
};

/// Must match BIMaterialDesc in BodyInterfaceReference.cpp
const MaterialDesc = extern struct {
    name: [16]u8 = @splat(0),
    color: u32 = 0,
};

/// Must match BIGroupFilterDesc in BodyInterfaceReference.cpp
const GroupFilterDesc = extern struct {
    num_sub_groups: u32 = 0,
    num_disabled: u32 = 0,
    disabled: [16]u32 = @splat(0),
};

/// Must match BITables in BodyInterfaceReference.cpp
const Tables = extern struct {
    shapes: [*]const ShapeDesc,
    num_shapes: u32,
    materials: [*]const MaterialDesc,
    num_materials: u32,
    filters: [*]const GroupFilterDesc,
    num_filters: u32,
};

/// Must match BIBodyDesc in BodyInterfaceReference.cpp
const BodyDesc = extern struct {
    user_data: u64 = 0,
    position: [3]Real = @splat(0),
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
    linear_velocity: [3]f32 = @splat(0),
    angular_velocity: [3]f32 = @splat(0),
    friction: f32 = 0.2,
    restitution: f32 = 0.0,
    linear_damping: f32 = 0.05,
    angular_damping: f32 = 0.05,
    max_linear_velocity: f32 = 500.0,
    max_angular_velocity: f32 = 0.25 * std.math.pi * 60.0,
    gravity_factor: f32 = 1.0,
    inertia_multiplier: f32 = 1.0,
    mass: f32 = 0.0,
    inertia: [16]f32 = @splat(0),
    object_layer: u32 = 0,
    motion_type: u32 = 2,
    allowed_dofs: u32 = 0b111111,
    allow_dynamic_or_kinematic: u32 = 0,
    is_sensor: u32 = 0,
    collide_kinematic_vs_non_dynamic: u32 = 0,
    use_manifold_reduction: u32 = 1,
    apply_gyroscopic_force: u32 = 0,
    motion_quality: u32 = 0,
    enhanced_internal_edge_removal: u32 = 0,
    allow_sleeping: u32 = 1,
    num_velocity_steps_override: u32 = 0,
    num_position_steps_override: u32 = 0,
    override_mass_properties: u32 = 0,
    shape: u32 = none,
    use_shape_settings: u32 = 0,
    group_filter: u32 = none,
    group_id: u32 = 0xffffffff,
    sub_group_id: u32 = 0xffffffff,
};

/// Must match BISharedSettingsDesc in BodyInterfaceReference.cpp
const SharedSettingsDesc = extern struct {
    /// SoftBodySharedSettings::SaveBinaryState bytes (Zolt writes them, both sides restore them)
    bytes: [*]const u8,
    size: u32,
    num_materials: u32 = 0,
    /// Index in the material table, `default_material_index` (PhysicsMaterial::sDefault) or `none` (null)
    materials: [4]u32 = @splat(none),
};

/// Must match BISoftBodyDesc in BodyInterfaceReference.cpp
const SoftBodyDesc = extern struct {
    user_data: u64 = 0,
    position: [3]Real = @splat(0),
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
    linear_damping: f32 = 0.1,
    max_linear_velocity: f32 = 500.0,
    restitution: f32 = 0.0,
    friction: f32 = 0.2,
    pressure: f32 = 0.0,
    gravity_factor: f32 = 1.0,
    vertex_radius: f32 = 0.0,
    num_iterations: u32 = 5,
    object_layer: u32 = 0,
    update_position: u32 = 1,
    make_rotation_identity: u32 = 1,
    allow_sleeping: u32 = 1,
    faces_double_sided: u32 = 0,
    /// Index in the shared settings table or `none` (no shared settings)
    shared: u32 = none,
    group_filter: u32 = none,
    group_id: u32 = 0xffffffff,
    sub_group_id: u32 = 0xffffffff,
    /// jolt_bi_sb_create: 0 createSoftBody + addBody, 1 createAndAddSoftBody, 2 createSoftBody, 3 createSoftBodyWithID +
    /// addBody, 4 createSoftBodyWithoutID + assignBodyID + addBody
    create_mode: u32 = 0,
    /// The ID of mode 3
    custom_id: u32 = 0,
    activation: u32 = 0,
};

/// The kinds of the scene constraints: the EConstraintSubType values of the constraint parity (ConstraintsParity.Kind)
const SceneConstraintKind = ConstraintsParity.Kind;

/// The kinds that the scenes create: every kind whose slot in ConstraintsParity.zig is filled (the Zolt class is
/// ported), except Path: Jolt's PathConstraint::GetConstraintSettings is not implemented (it asserts and returns
/// nullptr), so FromPhysicsSystem cannot convert a system with a path constraint
const scene_constraint_kinds: []const SceneConstraintKind = blk: {
    var kinds: [ConstraintsParity.ported_kinds.len]SceneConstraintKind = undefined;
    var n: usize = 0;
    for (ConstraintsParity.ported_kinds) |kind| {
        if (kind != .path) {
            kinds[n] = kind;
            n += 1;
        }
    }
    const result = kinds[0..n].*;
    break :blk &result;
};

/// A constraint of a PhysicsScene: the indices of its bodies and its settings, a description of the constraint parity
/// (ConstraintsParity.CSSettings: the kind is its EConstraintSubType value, the layout per kind is CSMakeSettings of
/// ConstraintsReference.cpp, which the C++ side calls through CSMakeConstraintSettings for every kind; the Zolt side is
/// ConstraintsParity.makeDescriptionSettings, the slot of the kind). The C ABI passes the body indices and the
/// descriptions as two arrays (`jolt_bi_scene_save`).
const SceneConstraint = struct {
    /// Index in the bodies of the scene or PhysicsScene.fixed_to_world. CreateBodies creates the soft bodies after the
    /// bodies, so index `num_bodies + i` is soft body i.
    bodies: [2]u32,
    settings: ConstraintsParity.CSSettings,

    /// A random constraint of a random kind of `scene_constraint_kinds` between the bodies of a scene with `num_bodies`
    /// bodies and soft bodies together (and the world)
    fn random(rng: *Rng, num_bodies: u32) SceneConstraint {
        const kind = scene_constraint_kinds[rng.intRange(usize, 0, scene_constraint_kinds.len - 1)];
        const body2 = rng.intRange(u32, 0, num_bodies - 1);
        const body1 = if (num_bodies == 1 or rng.intRange(u32, 0, 3) == 0) PhysicsScene.fixed_to_world else (body2 + rng.intRange(u32, 1, num_bodies - 1)) % num_bodies; // Never the same body twice
        return .{ .bodies = .{ body1, body2 }, .settings = ConstraintsParity.randomDescription(rng.next(), kind) };
    }

    /// The settings (one reference owned by the caller)
    fn makeSettings(self: *const SceneConstraint, allocator: Allocator) !Ref(zolt.ConstraintSettings) {
        return ConstraintsParity.makeDescriptionSettings(allocator, &self.settings);
    }
};

/// Must match BIOp in BodyInterfaceReference.cpp. `i[0]` is the activation mode, `i[1]` the number of bodies of the
/// list operations and `i[2..]` their IDs, or the integer arguments.
const Op = extern struct {
    r: [6]Real = @splat(0),
    f: [24]f32 = @splat(0),
    kind: u32 = 0,
    body: u32 = BodyID.invalid_body_id,
    i: [16]u32 = @splat(0),
};

/// Must match BIOut in BodyInterfaceReference.cpp
const Out = extern struct {
    r: [6]Real = @splat(sentinel_real),
    f: [48]f32 = @splat(sentinel),
    i: [24]u32 = @splat(0xdeadbeef),
};

const sentinel: f32 = -12345.0;
const sentinel_real: Real = -12345.0;

/// Must match EBIOp in BodyInterfaceReference.cpp
const OpKind = enum(u32) {
    set_position_and_rotation = 0,
    set_position_and_rotation_when_changed = 1,
    get_position_and_rotation = 2,
    set_position = 3,
    get_position = 4,
    get_center_of_mass_position = 5,
    set_rotation = 6,
    get_rotation = 7,
    get_world_transform = 8,
    get_center_of_mass_transform = 9,
    move_kinematic = 10,
    set_linear_and_angular_velocity = 11,
    get_linear_and_angular_velocity = 12,
    set_linear_velocity = 13,
    get_linear_velocity = 14,
    add_linear_velocity = 15,
    add_linear_and_angular_velocity = 16,
    set_angular_velocity = 17,
    get_angular_velocity = 18,
    get_point_velocity = 19,
    set_position_rotation_and_velocity = 20,
    add_force = 21,
    add_force_at_position = 22,
    add_torque = 23,
    add_force_and_torque = 24,
    add_impulse = 25,
    add_impulse_at_position = 26,
    add_angular_impulse = 27,
    apply_buoyancy_impulse = 28,
    get_body_type = 29,
    set_motion_type = 30,
    get_motion_type = 31,
    set_motion_quality = 32,
    get_motion_quality = 33,
    get_inverse_inertia = 34,
    set_restitution = 35,
    get_restitution = 36,
    set_friction = 37,
    get_friction = 38,
    set_gravity_factor = 39,
    get_gravity_factor = 40,
    set_max_linear_velocity = 41,
    get_max_linear_velocity = 42,
    set_max_angular_velocity = 43,
    get_max_angular_velocity = 44,
    set_use_manifold_reduction = 45,
    get_use_manifold_reduction = 46,
    set_is_sensor = 47,
    is_sensor = 48,
    set_collision_group = 49,
    get_collision_group = 50,
    get_transformed_shape = 51,
    set_user_data = 52,
    get_user_data = 53,
    get_material = 54,
    set_shape = 55,
    notify_shape_changed = 56,
    set_object_layer = 57,
    get_object_layer = 58,
    activate_body = 59,
    activate_bodies = 60,
    activate_bodies_in_aabox = 61,
    deactivate_body = 62,
    deactivate_bodies = 63,
    is_active = 64,
    reset_sleep_timer = 65,
    add_body = 66,
    remove_body = 67,
    is_added = 68,
    destroy_body = 69,
    add_bodies = 70,
    add_bodies_abort = 71,
    remove_bodies = 72,
    invalidate_contact_cache = 73,
    get_shape = 74,
    unassign_and_reassign = 75,
    unassign_and_destroy = 76,
    destroy_bodies = 77,
};

fn load3(v: []const f32) Vec3 {
    return Vec3.init(v[0], v[1], v[2]);
}

fn load4(v: []const f32) Vec4 {
    return Vec4.init(v[0], v[1], v[2], v[3]);
}

fn loadQuat(v: []const f32) Quat {
    return Quat.init(v[0], v[1], v[2], v[3]);
}

fn loadR3(v: []const Real) RVec3 {
    return RVec3.init(v[0], v[1], v[2]);
}

fn store3(v: Vec3, out: []f32) void {
    out[0] = v.getX();
    out[1] = v.getY();
    out[2] = v.getZ();
}

fn store4(v: Vec4, out: []f32) void {
    out[0] = v.getX();
    out[1] = v.getY();
    out[2] = v.getZ();
    out[3] = v.getW();
}

fn storeQuat(q: Quat, out: []f32) void {
    out[0] = q.getX();
    out[1] = q.getY();
    out[2] = q.getZ();
    out[3] = q.getW();
}

fn storeR3(v: RVec3, out: []Real) void {
    out[0] = v.getX();
    out[1] = v.getY();
    out[2] = v.getZ();
}

/// The shapes, materials and group filters of a test (BITableObjects)
const TableObjects = struct {
    allocator: Allocator,
    materials: std.ArrayList(RefConst(PhysicsMaterial)) = .empty,
    filters: std.ArrayList(RefConst(GroupFilter)) = .empty,
    shapes: std.ArrayList(RefConst(Shape)) = .empty,
    shape_descs: []const ShapeDesc = &.{},

    fn init(self: *TableObjects, allocator: Allocator, tables: *const Tables) !void {
        self.* = .{ .allocator = allocator };
        errdefer self.deinit();
        for (tables.materials[0..tables.num_materials]) |*m| {
            const name = std.mem.sliceTo(&m.name, 0);
            const material = try PhysicsMaterialSimple.create(allocator, name, Color.fromUInt32(m.color));
            try self.materials.append(allocator, .init(material.material()));
        }
        for (tables.filters[0..tables.num_filters]) |*f| {
            const filter = try GroupFilterTable.create(allocator, .{ .num_sub_groups = f.num_sub_groups });
            try self.filters.append(allocator, .init(filter.asGroupFilter()));
            for (0..f.num_disabled) |j|
                filter.disableCollision(f.disabled[2 * j], f.disabled[2 * j + 1]);
        }
        self.shape_descs = tables.shapes[0..tables.num_shapes];
        for (self.shape_descs) |*d| {
            var settings = Ref(ShapeSettings).init(try self.makeShapeSettings(d));
            defer settings.deinit();
            var result = try settings.get().?.createShape(allocator);
            defer result.deinit();
            try self.shapes.append(allocator, .init(result.getPtr().?));
        }
    }

    fn deinit(self: *TableObjects) void {
        for (self.shapes.items) |*s| s.deinit();
        self.shapes.deinit(self.allocator);
        for (self.filters.items) |*f| f.deinit();
        self.filters.deinit(self.allocator);
        for (self.materials.items) |*m| m.deinit();
        self.materials.deinit(self.allocator);
    }

    fn getMaterial(self: *const TableObjects, index: u32) ?*const PhysicsMaterial {
        return if (index < self.materials.items.len) self.materials.items[index].get() else null;
    }

    fn makeBoxSettings(self: *const TableObjects, desc: *const ShapeDesc) !*ShapeSettings {
        return (try BoxShapeSettings.create(self.allocator, load3(desc.params[0..3]), .{ .convex_radius = desc.params[3], .material = self.getMaterial(desc.material) })).asShapeSettings();
    }

    /// MakeShapeSettings of BodyInterfaceReference.cpp: the returned settings have no reference yet
    fn makeShapeSettings(self: *const TableObjects, desc: *const ShapeDesc) !*ShapeSettings {
        const a = self.allocator;
        const p = desc.params;
        const material = self.getMaterial(desc.material);
        return switch (desc.kind) {
            0 => (try SphereShapeSettings.create(a, p[0], .{ .material = material })).asShapeSettings(),
            1 => try self.makeBoxSettings(desc),
            2 => (try CapsuleShapeSettings.create(a, p[0], p[1], .{ .material = material })).asShapeSettings(),
            3 => (try OffsetCenterOfMassShapeSettings.create(a, load3(p[4..7]), try self.makeBoxSettings(desc))).asShapeSettings(),
            4 => (try RotatedTranslatedShapeSettings.create(a, load3(p[4..7]), loadQuat(&desc.rotation), try self.makeBoxSettings(desc))).asShapeSettings(),
            5 => (try ScaledShapeSettings.create(a, try self.makeBoxSettings(desc), load3(p[4..7]))).asShapeSettings(),
            6 => blk: {
                const compound = try StaticCompoundShapeSettings.create(a);
                try compound.base.addShape(load3(p[4..7]), loadQuat(&desc.rotation), try self.makeBoxSettings(desc), .{});
                try compound.base.addShape(load3(p[4..7]).negate(), Quat.identity(), (try SphereShapeSettings.create(a, p[0], .{ .material = material })).asShapeSettings(), .{});
                break :blk compound.asShapeSettings();
            },
            else => (try ScaledShapeSettings.create(a, (try SphereShapeSettings.create(a, p[0], .{ .material = material })).asShapeSettings(), load3(p[4..7]))).asShapeSettings(),
        };
    }

    fn shapeIndex(self: *const TableObjects, shape: ?*const Shape) u32 {
        for (self.shapes.items, 0..) |s, i|
            if (s.get() == shape) return @intCast(i);
        return none;
    }

    fn materialIndex(self: *const TableObjects, material: *const PhysicsMaterial) u32 {
        if (material == PhysicsMaterial.default) return default_material_index;
        for (self.materials.items, 0..) |m, i|
            if (m.get() == material) return @intCast(i);
        return none;
    }

    fn filterIndex(self: *const TableObjects, filter: ?*const GroupFilter) u32 {
        for (self.filters.items, 0..) |f, i|
            if (f.get() == filter) return @intCast(i);
        return none;
    }

    /// MakeSharedSettings of BodyInterfaceReference.cpp: the shared settings restored from their bytes, with their
    /// materials (the caller releases them)
    fn makeSharedSettings(self: *const TableObjects, descs: []const SharedSettingsDesc, out: *std.ArrayList(Ref(SoftBodySharedSettings))) !void {
        const a = self.allocator;
        for (descs) |*d| {
            var ref: Ref(SoftBodySharedSettings) = .init(try SoftBodySharedSettings.create(a));
            errdefer ref.deinit();
            const shared = ref.get().?;
            var reader: std.Io.Reader = .fixed(d.bytes[0..d.size]);
            var in = StreamInWrapper.init(&reader);
            try shared.restoreBinaryState(in.streamIn());
            for (shared.materials.items) |*m| m.deinit();
            shared.materials.clearRetainingCapacity();
            try shared.materials.ensureTotalCapacity(a, d.num_materials);
            for (d.materials[0..d.num_materials]) |m|
                shared.materials.appendAssumeCapacity(.init(if (m == default_material_index) PhysicsMaterial.default else self.getMaterial(m)));
            try out.append(a, ref);
        }
    }

    /// MakeSoftBodyCreationSettings of BodyInterfaceReference.cpp (the caller deinits the result)
    fn makeSoftBodyCreationSettings(self: *const TableObjects, desc: *const SoftBodyDesc, shared: []const Ref(SoftBodySharedSettings)) SoftBodyCreationSettings {
        var s: SoftBodyCreationSettings = .{};
        s.settings = .init(if (desc.shared < shared.len) shared[desc.shared].get() else null);
        s.position = loadR3(&desc.position);
        s.rotation = loadQuat(&desc.rotation);
        s.user_data = desc.user_data;
        s.object_layer = @intCast(desc.object_layer);
        s.collision_group = .init(if (desc.group_filter < self.filters.items.len) self.filters.items[desc.group_filter].get() else null, desc.group_id, desc.sub_group_id);
        s.num_iterations = desc.num_iterations;
        s.linear_damping = desc.linear_damping;
        s.max_linear_velocity = desc.max_linear_velocity;
        s.restitution = desc.restitution;
        s.friction = desc.friction;
        s.pressure = desc.pressure;
        s.gravity_factor = desc.gravity_factor;
        s.vertex_radius = desc.vertex_radius;
        s.update_position = desc.update_position != 0;
        s.make_rotation_identity = desc.make_rotation_identity != 0;
        s.allow_sleeping = desc.allow_sleeping != 0;
        s.faces_double_sided = desc.faces_double_sided != 0;
        return s;
    }

    /// MakeBodyCreationSettings of BodyInterfaceReference.cpp (the caller deinits the result)
    fn makeBodyCreationSettings(self: *const TableObjects, desc: *const BodyDesc) !BodyCreationSettings {
        var s: BodyCreationSettings = .{};
        errdefer s.deinit();
        s.position = loadR3(&desc.position);
        s.rotation = loadQuat(&desc.rotation);
        s.linear_velocity = load3(&desc.linear_velocity);
        s.angular_velocity = load3(&desc.angular_velocity);
        s.user_data = desc.user_data;
        s.object_layer = @intCast(desc.object_layer);
        s.collision_group = .init(if (desc.group_filter < self.filters.items.len) self.filters.items[desc.group_filter].get() else null, desc.group_id, desc.sub_group_id);
        s.motion_type = @enumFromInt(desc.motion_type);
        s.allowed_dofs = .fromInt(@intCast(desc.allowed_dofs));
        s.allow_dynamic_or_kinematic = desc.allow_dynamic_or_kinematic != 0;
        s.is_sensor = desc.is_sensor != 0;
        s.collide_kinematic_vs_non_dynamic = desc.collide_kinematic_vs_non_dynamic != 0;
        s.use_manifold_reduction = desc.use_manifold_reduction != 0;
        s.apply_gyroscopic_force = desc.apply_gyroscopic_force != 0;
        s.motion_quality = @enumFromInt(desc.motion_quality);
        s.enhanced_internal_edge_removal = desc.enhanced_internal_edge_removal != 0;
        s.allow_sleeping = desc.allow_sleeping != 0;
        s.friction = desc.friction;
        s.restitution = desc.restitution;
        s.linear_damping = desc.linear_damping;
        s.angular_damping = desc.angular_damping;
        s.max_linear_velocity = desc.max_linear_velocity;
        s.max_angular_velocity = desc.max_angular_velocity;
        s.gravity_factor = desc.gravity_factor;
        s.num_velocity_steps_override = desc.num_velocity_steps_override;
        s.num_position_steps_override = desc.num_position_steps_override;
        s.override_mass_properties = @enumFromInt(desc.override_mass_properties);
        s.inertia_multiplier = desc.inertia_multiplier;
        s.mass_properties_override.mass = desc.mass;
        s.mass_properties_override.inertia = Mat44.init(load4(desc.inertia[0..4]), load4(desc.inertia[4..8]), load4(desc.inertia[8..12]), load4(desc.inertia[12..16]));
        if (desc.shape < self.shapes.items.len) {
            if (desc.use_shape_settings != 0)
                s.setShapeSettings(try self.makeShapeSettings(&self.shape_descs[desc.shape]))
            else
                s.setShape(self.shapes.items[desc.shape].get());
        }
        return s;
    }
};

/// BIWriteBodies of BodyInterfaceReference.cpp
fn writeBodies(allocator: Allocator, bodies: []const *const Body, active: []const u32, tables: *const TableObjects, r: *StateRecorder) void {
    _ = allocator;
    r.write(@as(u32, @intCast(bodies.len)));
    for (bodies) |body| {
        r.write(body.getID().getIndexAndSequenceNumber());
        if (body.isSoftBody()) {
            var settings = body.getSoftBodyCreationSettings();
            defer settings.deinit();
            settings.saveBinaryState(r.streamOut());
        } else {
            var settings = body.getBodyCreationSettings();
            defer settings.deinit();
            settings.saveBinaryState(r.streamOut());
        }
        r.write(body.getUserData());
        r.write(tables.shapeIndex(body.getShape()));
        r.write(@as(u8, @intFromBool(body.isActive())));
        r.write(@as(u8, @intFromBool(body.isInBroadPhase())));
        r.write(body.getWorldSpaceBounds().min);
        r.write(body.getWorldSpaceBounds().max);
        r.write(body.getCenterOfMassPosition());
        r.write(body.getBroadPhaseLayer().getValue());
    }
    r.write(@as(u32, @intCast(active.len)));
    for (active) |a| r.write(a);
}

/// BIWriteSnapshot of BodyInterfaceReference.cpp
fn writeSnapshot(allocator: Allocator, bm: *const BodyManager, tables: *const TableObjects, out: *StateRecorderImpl) !void {
    var ids: BodyIDVector = .empty;
    defer ids.deinit(allocator);
    try bm.getBodyIDs(allocator, &ids);
    var bodies: std.ArrayList(*const Body) = .empty;
    defer bodies.deinit(allocator);
    for (ids.items) |id| try bodies.append(allocator, bm.getBody(id));
    var active: std.ArrayList(u32) = .empty;
    defer active.deinit(allocator);
    const num_active = bm.getNumActiveBodies(.rigid_body);
    for (bm.getActiveBodiesUnsafe(.rigid_body)[0..num_active]) |id| try active.append(allocator, id.getIndexAndSequenceNumber());
    writeBodies(allocator, bodies.items, active.items, tables, out.recorder());
    try bm.saveState(out.recorder(), null);
}

/// The world on both sides (BIWorld)
const World = struct {
    allocator: Allocator,
    layers: TestLayers.BPLayerInterfaceImpl,
    object_vs_broad_phase: TestLayers.ObjectVsBroadPhaseLayerFilterImpl = .{},
    pairs: TestLayers.ObjectLayerPairFilterImpl = .{},
    system: PhysicsSystem = .empty,
    tables: TableObjects,
    unassigned: std.ArrayList(*Body) = .empty,
    jolt_world: *anyopaque,

    fn init(self: *World, allocator: Allocator, max_bodies: u32, num_body_mutexes: u32, tables: *const Tables) !void {
        self.* = .{ .allocator = allocator, .layers = .init(), .tables = undefined, .jolt_world = undefined };
        try self.tables.init(allocator, tables);
        errdefer self.tables.deinit();
        try self.system.init(allocator, io, max_bodies, num_body_mutexes, 64, 64, &self.layers.base, &self.object_vs_broad_phase.base, &self.pairs.base, .{ .broad_phase = .brute_force });
        self.jolt_world = jolt.jolt_bi_world_create(max_bodies, num_body_mutexes, tables);
    }

    fn deinit(self: *World) void {
        jolt.jolt_bi_world_destroy(self.jolt_world);
        for (self.unassigned.items) |body| self.bi().destroyBodyWithoutID(body);
        self.unassigned.deinit(self.allocator);
        self.system.deinit();
        self.tables.deinit();
    }

    fn bi(self: *World) *BodyInterface {
        return self.system.getBodyInterfaceMut();
    }

    /// jolt_bi_create_body on the Zolt side
    fn createBody(self: *World, desc: *const BodyDesc, mode: u32, custom_id: u32, activation: Activation) !u32 {
        var settings = try self.tables.makeBodyCreationSettings(desc);
        defer settings.deinit();
        const b = self.bi();
        switch (mode) {
            0, 2 => {
                const body = (try b.createBody(&settings)) orelse return BodyID.invalid_body_id;
                if (mode == 0)
                    try b.addBody(body.getID(), activation);
                return body.getID().getIndexAndSequenceNumber();
            },
            1 => return (try b.createAndAddBody(&settings, activation)).getIndexAndSequenceNumber(),
            3 => {
                const body = (try b.createBodyWithID(BodyID.init(custom_id), &settings)) orelse return BodyID.invalid_body_id;
                try b.addBody(body.getID(), activation);
                return body.getID().getIndexAndSequenceNumber();
            },
            else => {
                const body = try b.createBodyWithoutID(&settings);
                const assigned = if (mode == 4) b.assignBodyID(body) else b.assignBodyIDWithID(body, BodyID.init(custom_id));
                if (!assigned) {
                    b.destroyBodyWithoutID(body);
                    return BodyID.invalid_body_id;
                }
                try b.addBody(body.getID(), activation);
                return body.getID().getIndexAndSequenceNumber();
            },
        }
    }

    /// jolt_bi_op on the Zolt side
    fn op(self: *World, o: *const Op, out: *Out) !void {
        const b = self.bi();
        const id = BodyID.init(o.body);
        const r = &o.r;
        const f = &o.f;
        const i = &o.i;
        const activation: Activation = @enumFromInt(i[0]);
        var ids_buffer: [14]BodyID = undefined;
        const num_ids = @min(i[1], 14);
        for (0..num_ids) |k| ids_buffer[k] = BodyID.init(i[2 + k]);
        const ids = ids_buffer[0..num_ids];

        switch (@as(OpKind, @enumFromInt(o.kind))) {
            .set_position_and_rotation => b.setPositionAndRotation(id, loadR3(r[0..3]), loadQuat(f[0..4]), activation),
            .set_position_and_rotation_when_changed => b.setPositionAndRotationWhenChanged(id, loadR3(r[0..3]), loadQuat(f[0..4]), activation),
            .get_position_and_rotation => {
                const pr = b.getPositionAndRotation(id);
                storeR3(pr.position, &out.r);
                storeQuat(pr.rotation, &out.f);
            },
            .set_position => b.setPosition(id, loadR3(r[0..3]), activation),
            .get_position => storeR3(b.getPosition(id), &out.r),
            .get_center_of_mass_position => storeR3(b.getCenterOfMassPosition(id), &out.r),
            .set_rotation => b.setRotation(id, loadQuat(f[0..4]), activation),
            .get_rotation => storeQuat(b.getRotation(id), &out.f),
            .get_world_transform, .get_center_of_mass_transform => {
                const m: RMat44 = if (@as(OpKind, @enumFromInt(o.kind)) == .get_world_transform) b.getWorldTransform(id) else b.getCenterOfMassTransform(id);
                for (0..3) |c| store4(m.getColumn4(@intCast(c)), out.f[4 * c ..][0..4]);
                storeR3(m.getTranslation(), &out.r);
            },
            .move_kinematic => b.moveKinematic(id, loadR3(r[0..3]), loadQuat(f[0..4]), f[4]),
            .set_linear_and_angular_velocity => b.setLinearAndAngularVelocity(id, load3(f[0..3]), load3(f[3..6])),
            .get_linear_and_angular_velocity => {
                const v = b.getLinearAndAngularVelocity(id);
                store3(v.linear_velocity, out.f[0..3]);
                store3(v.angular_velocity, out.f[3..6]);
            },
            .set_linear_velocity => b.setLinearVelocity(id, load3(f[0..3])),
            .get_linear_velocity => store3(b.getLinearVelocity(id), &out.f),
            .add_linear_velocity => b.addLinearVelocity(id, load3(f[0..3])),
            .add_linear_and_angular_velocity => b.addLinearAndAngularVelocity(id, load3(f[0..3]), load3(f[3..6])),
            .set_angular_velocity => b.setAngularVelocity(id, load3(f[0..3])),
            .get_angular_velocity => store3(b.getAngularVelocity(id), &out.f),
            .get_point_velocity => store3(b.getPointVelocity(id, loadR3(r[0..3])), &out.f),
            .set_position_rotation_and_velocity => b.setPositionRotationAndVelocity(id, loadR3(r[0..3]), loadQuat(f[0..4]), load3(f[4..7]), load3(f[7..10])),
            .add_force => b.addForce(id, load3(f[0..3]), .{ .activation = activation }),
            .add_force_at_position => b.addForceAtPosition(id, load3(f[0..3]), loadR3(r[0..3]), .{ .activation = activation }),
            .add_torque => b.addTorque(id, load3(f[0..3]), .{ .activation = activation }),
            .add_force_and_torque => b.addForceAndTorque(id, load3(f[0..3]), load3(f[3..6]), .{ .activation = activation }),
            .add_impulse => b.addImpulse(id, load3(f[0..3])),
            .add_impulse_at_position => b.addImpulseAtPosition(id, load3(f[0..3]), loadR3(r[0..3])),
            .add_angular_impulse => b.addAngularImpulse(id, load3(f[0..3])),
            .apply_buoyancy_impulse => out.i[0] = @intFromBool(b.applyBuoyancyImpulse(id, loadR3(r[0..3]), load3(f[0..3]), f[3], f[4], f[5], load3(f[6..9]), load3(f[9..12]), f[12])),
            .get_body_type => out.i[0] = @intFromEnum(b.getBodyType(id)),
            .set_motion_type => b.setMotionType(id, @enumFromInt(i[2]), activation),
            .get_motion_type => out.i[0] = @intFromEnum(b.getMotionType(id)),
            .set_motion_quality => b.setMotionQuality(id, @enumFromInt(i[2])),
            .get_motion_quality => out.i[0] = @intFromEnum(b.getMotionQuality(id)),
            .get_inverse_inertia => {
                const m = b.getInverseInertia(id);
                for (0..4) |c| store4(m.getColumn4(@intCast(c)), out.f[4 * c ..][0..4]);
            },
            .set_restitution => b.setRestitution(id, f[0]),
            .get_restitution => out.f[0] = b.getRestitution(id),
            .set_friction => b.setFriction(id, f[0]),
            .get_friction => out.f[0] = b.getFriction(id),
            .set_gravity_factor => b.setGravityFactor(id, f[0]),
            .get_gravity_factor => out.f[0] = b.getGravityFactor(id),
            .set_max_linear_velocity => b.setMaxLinearVelocity(id, f[0]),
            .get_max_linear_velocity => out.f[0] = b.getMaxLinearVelocity(id),
            .set_max_angular_velocity => b.setMaxAngularVelocity(id, f[0]),
            .get_max_angular_velocity => out.f[0] = b.getMaxAngularVelocity(id),
            .set_use_manifold_reduction => try b.setUseManifoldReduction(id, i[2] != 0),
            .get_use_manifold_reduction => out.i[0] = @intFromBool(b.getUseManifoldReduction(id)),
            .set_is_sensor => b.setIsSensor(id, i[2] != 0),
            .is_sensor => out.i[0] = @intFromBool(b.isSensor(id)),
            .set_collision_group => {
                var group = CollisionGroup.init(if (i[4] < self.tables.filters.items.len) self.tables.filters.items[i[4]].get() else null, i[2], i[3]);
                defer group.deinit();
                b.setCollisionGroup(id, &group);
            },
            .get_collision_group => {
                const group = b.getCollisionGroup(id);
                out.i[0] = group.getGroupID();
                out.i[1] = group.getSubGroupID();
                out.i[2] = self.tables.filterIndex(group.getGroupFilter());
            },
            .get_transformed_shape => {
                var ts = b.getTransformedShape(id);
                defer ts.deinit();
                out.i[0] = @intFromBool(ts.shape.get() != null);
                out.i[1] = self.tables.shapeIndex(ts.shape.get());
                out.i[2] = ts.body_id.getIndexAndSequenceNumber();
                out.i[3] = ts.sub_shape_id_creator.id.getValue();
                out.i[4] = ts.sub_shape_id_creator.getNumBitsWritten();
                store3(ts.getShapeScale(), out.f[0..3]);
                if (ts.shape.get() != null) {
                    storeR3(ts.shape_position_com, out.r[0..3]);
                    storeQuat(ts.shape_rotation, out.f[3..7]);
                    const bounds = ts.getWorldSpaceBounds();
                    store3(bounds.min, out.f[7..10]);
                    store3(bounds.max, out.f[10..13]);
                    const com = ts.getCenterOfMassTransform();
                    for (0..3) |c| store4(com.getColumn4(@intCast(c)), out.f[13 + 4 * c ..][0..4]);
                    storeR3(com.getTranslation(), out.r[3..6]);
                }
            },
            .set_user_data => b.setUserData(id, @as(u64, i[2]) | (@as(u64, i[3]) << 32)),
            .get_user_data => {
                const user_data = b.getUserData(id);
                out.i[0] = @truncate(user_data);
                out.i[1] = @truncate(user_data >> 32);
            },
            .get_material => out.i[0] = self.tables.materialIndex(b.getMaterial(id, .{ .value = i[2] })),
            .set_shape => try b.setShape(id, self.tables.shapes.items[i[2]].get().?, i[3] != 0, activation),
            .notify_shape_changed => try b.notifyShapeChanged(id, load3(f[0..3]), i[3] != 0, activation),
            .set_object_layer => try b.setObjectLayer(id, @intCast(i[2])),
            .get_object_layer => out.i[0] = b.getObjectLayer(id),
            .activate_body => b.activateBody(id),
            .activate_bodies => b.activateBodies(ids),
            .activate_bodies_in_aabox => try b.activateBodiesInAABox(&AABox.init(load3(f[0..3]), load3(f[3..6])), &.{}, &.{}),
            .deactivate_body => b.deactivateBody(id),
            .deactivate_bodies => b.deactivateBodies(ids),
            .is_active => out.i[0] = @intFromBool(b.isActive(id)),
            .reset_sleep_timer => b.resetSleepTimer(id),
            .add_body => try b.addBody(id, activation),
            .remove_body => b.removeBody(id),
            .is_added => out.i[0] = @intFromBool(b.isAdded(id)),
            .destroy_body => b.destroyBody(id),
            .add_bodies, .add_bodies_abort => {
                const state = try b.addBodiesPrepare(ids);
                if (@as(OpKind, @enumFromInt(o.kind)) == .add_bodies)
                    b.addBodiesFinalize(ids, state, activation)
                else
                    b.addBodiesAbort(ids, state);
                for (ids, 0..) |x, k| out.i[k] = x.getIndexAndSequenceNumber(); // The order after AddBodiesPrepare
            },
            .remove_bodies => {
                b.removeBodies(ids);
                for (ids, 0..) |x, k| out.i[k] = x.getIndexAndSequenceNumber();
            },
            .invalidate_contact_cache => try b.invalidateContactCache(id),
            .get_shape => {
                var shape = b.getShape(id);
                defer shape.deinit();
                out.i[0] = self.tables.shapeIndex(shape.get());
            },
            .unassign_and_reassign => {
                const body = b.unassignBodyID(id);
                const assigned = if (i[2] == BodyID.invalid_body_id) b.assignBodyID(body) else b.assignBodyIDWithID(body, BodyID.init(i[2]));
                if (assigned) {
                    out.i[0] = body.getID().getIndexAndSequenceNumber();
                } else {
                    out.i[0] = BodyID.invalid_body_id;
                    try self.unassigned.append(self.allocator, body);
                }
            },
            .unassign_and_destroy => {
                var bodies: [14]*Body = undefined;
                b.unassignBodyIDs(ids, bodies[0..ids.len]);
                for (bodies[0..ids.len]) |body| b.destroyBodyWithoutID(body);
            },
            .destroy_bodies => b.destroyBodies(ids),
        }
    }

    /// Compare the snapshots of both sides, prints the first differing byte
    fn compareSnapshots(self: *World, checker: *Checker, context: []const u8) !void {
        var zolt_snapshot = StateRecorderImpl.init(self.allocator);
        defer zolt_snapshot.deinit();
        try writeSnapshot(self.allocator, &self.system.body_manager, &self.tables, &zolt_snapshot);
        const z = zolt_snapshot.getData();

        var buffer: [65536]u8 = undefined;
        const size = jolt.jolt_bi_snapshot(self.jolt_world, &buffer, buffer.len);
        try std.testing.expect(size <= buffer.len);
        const j = buffer[0..size];
        if (!std.mem.eql(u8, z, j)) {
            if (checker.mismatches < 5) {
                const first = std.mem.indexOfDiff(u8, z, j) orelse 0;
                std.debug.print("{s}: snapshot mismatch after {s}: sizes {d} / {d}, first difference at byte {d}\n", .{ checker.name, context, z.len, j.len, first });
            }
            checker.mismatches += 1;
        }
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Random inputs

fn randomQuat(rng: *Rng) Quat {
    while (true) {
        const v = Vec4.init(rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1), rng.float(-1, 1));
        if (v.length() > 0.1) return Quat.fromVec4(v.normalized());
    }
}

fn randomVec3(rng: *Rng, range: f32) Vec3 {
    return Vec3.init(rng.float(-range, range), rng.float(-range, range), rng.float(-range, range));
}

/// A velocity: zero, near zero (below and above IsNearZero's 1.0e-12 squared length), small, or above the maximum
fn randomVelocity(rng: *Rng) Vec3 {
    return switch (rng.intRange(u32, 0, 9)) {
        0 => Vec3.zero(),
        1 => randomVec3(rng, 1.0e-7),
        2 => randomVec3(rng, 1.0e-5),
        3 => randomVec3(rng, 2000),
        else => randomVec3(rng, 10),
    };
}

/// A velocity of a new body: below the maximum velocities of randomBodyDesc (Body::SetLinearVelocity /
/// SetAngularVelocity assert that, only the BodyInterface setters clamp)
fn randomCreationVelocity(rng: *Rng) Vec3 {
    const v = randomVelocity(rng);
    return if (v.length() > 18) randomVec3(rng, 10) else v;
}

fn randomPosition(rng: *Rng) RVec3 {
    if (Core.double_precision and rng.intRange(u32, 0, 9) == 0)
        return RVec3.init(@as(Real, rng.float(-1, 1)) * 1.0e7, @as(Real, rng.float(-1, 1)) * 1.0e7, rng.float(-10, 10));
    return RVec3.init(rng.float(-30, 30), rng.float(-30, 30), rng.float(-30, 30));
}

fn randomShapeDesc(rng: *Rng, kind: u32, num_materials: u32) ShapeDesc {
    var d: ShapeDesc = .{ .kind = kind };
    d.material = if (rng.intRange(u32, 0, 3) == 0) none else rng.intRange(u32, 0, num_materials - 1);
    d.params[0] = rng.float(0.2, 2);
    d.params[1] = rng.float(0.2, 2);
    d.params[2] = rng.float(0.2, 2);
    d.params[3] = rng.float(0, 0.1);
    store3(randomVec3(rng, 1), d.params[4..7]);
    if (d.kind == 5 or d.kind == 7) {
        // Scales: uniform or not for the box, uniform for the sphere (a non uniformly scaled sphere is not a valid scale,
        // a body can't use it: see TableDescs.makeSphereScaleInvalid), never near zero
        const s = Vec3.init(rng.float(0.5, 2), rng.float(0.5, 2), rng.float(0.5, 2));
        store3(if (d.kind == 7 or rng.intRange(u32, 0, 1) == 0) Vec3.replicate(s.getX()) else s, d.params[4..7]);
    }
    storeQuat(randomQuat(rng), &d.rotation);
    return d;
}

fn randomMaterialDesc(rng: *Rng, index: usize) MaterialDesc {
    var d: MaterialDesc = .{ .color = rng.next() };
    _ = std.fmt.bufPrint(&d.name, "Mat{d}_{d}", .{ index, rng.intRange(u32, 0, 999) }) catch unreachable;
    return d;
}

fn randomGroupFilterDesc(rng: *Rng) GroupFilterDesc {
    var d: GroupFilterDesc = .{ .num_sub_groups = rng.intRange(u32, 2, 6) };
    d.num_disabled = rng.intRange(u32, 0, 8);
    for (0..d.num_disabled) |k| {
        d.disabled[2 * k] = rng.intRange(u32, 0, d.num_sub_groups - 1);
        d.disabled[2 * k + 1] = rng.intRange(u32, 0, d.num_sub_groups - 1);
        if (d.disabled[2 * k] == d.disabled[2 * k + 1]) d.disabled[2 * k + 1] = (d.disabled[2 * k] + 1) % d.num_sub_groups;
    }
    return d;
}

/// Allowed DOFs that keep a valid mass (Jolt asserts on None)
const allowed_dofs_choices = [_]u32{ 0b111111, 0b100011, 0b011111, 0b000111, 0b111000 | 0b011 };

fn randomBodyDesc(rng: *Rng, num_shapes: u32, num_filters: u32, opts: struct { allow_no_shape: bool = false, allow_settings: bool = true }) BodyDesc {
    var d: BodyDesc = .{};
    d.user_data = @as(u64, rng.next()) << 32 | rng.next();
    storeR3(randomPosition(rng), &d.position);
    storeQuat(randomQuat(rng), &d.rotation);
    store3(randomCreationVelocity(rng), &d.linear_velocity);
    store3(randomCreationVelocity(rng), &d.angular_velocity);
    d.friction = rng.float(-0.5, 1.5);
    d.restitution = rng.float(-0.5, 1.5);
    d.linear_damping = rng.float(0, 1);
    d.angular_damping = rng.float(0, 1);
    d.max_linear_velocity = rng.float(18, 600);
    d.max_angular_velocity = rng.float(18, 60);
    d.gravity_factor = rng.float(-2, 2);
    d.inertia_multiplier = rng.float(0.5, 2);
    d.object_layer = rng.intRange(u32, 0, 10);
    d.motion_type = rng.intRange(u32, 0, 2);
    d.allowed_dofs = allowed_dofs_choices[rng.intRange(usize, 0, allowed_dofs_choices.len - 1)];
    d.allow_dynamic_or_kinematic = rng.intRange(u32, 0, 1);
    d.is_sensor = rng.intRange(u32, 0, 1);
    d.collide_kinematic_vs_non_dynamic = rng.intRange(u32, 0, 1);
    d.use_manifold_reduction = rng.intRange(u32, 0, 1);
    d.apply_gyroscopic_force = rng.intRange(u32, 0, 1);
    d.motion_quality = rng.intRange(u32, 0, 1);
    d.enhanced_internal_edge_removal = rng.intRange(u32, 0, 1);
    d.allow_sleeping = rng.intRange(u32, 0, 1);
    d.num_velocity_steps_override = rng.intRange(u32, 0, 20);
    d.num_position_steps_override = rng.intRange(u32, 0, 20);
    d.override_mass_properties = rng.intRange(u32, 0, 2);
    d.mass = rng.float(0.5, 100);
    const diagonal = Vec3.init(rng.float(0.5, 10), rng.float(0.5, 10), rng.float(0.5, 10));
    const inertia_rotation = Mat44.rotationQuat(randomQuat(rng));
    const inertia = inertia_rotation.mul(Mat44.scaleVec3(diagonal)).mul(inertia_rotation.transposed3x3()); // Symmetric
    for (0..4) |c| store4(inertia.getColumn4(@intCast(c)), d.inertia[4 * c ..][0..4]);
    d.shape = if (opts.allow_no_shape and rng.intRange(u32, 0, 9) == 0) none else rng.intRange(u32, 0, num_shapes - 1);
    d.use_shape_settings = if (opts.allow_settings) @intFromBool(rng.intRange(u32, 0, 3) == 0) else 0;
    d.group_filter = if (rng.intRange(u32, 0, 2) == 0) none else rng.intRange(u32, 0, num_filters - 1);
    d.group_id = if (rng.intRange(u32, 0, 4) == 0) 0xffffffff else rng.intRange(u32, 0, 100);
    d.sub_group_id = rng.intRange(u32, 0, 5);
    return d;
}

/// Random tables (`shapes`, `materials` and `filters` hold the descriptions)
const TableDescs = struct {
    shapes: [8]ShapeDesc = undefined,
    materials: [3]MaterialDesc = undefined,
    filters: [2]GroupFilterDesc = undefined,

    fn random(rng: *Rng) TableDescs {
        var t: TableDescs = .{};
        for (&t.materials, 0..) |*m, k| m.* = randomMaterialDesc(rng, k);
        for (&t.filters) |*f| f.* = randomGroupFilterDesc(rng);
        for (&t.shapes, 0..) |*s, k|
            s.* = randomShapeDesc(rng, @intCast(k % 8), t.materials.len); // Every kind in every table
        return t;
    }

    /// Give the scaled sphere (kind 7) a non uniform scale: an invalid scale that PhysicsScene::FixInvalidScales fixes
    fn makeSphereScaleInvalid(self: *TableDescs, rng: *Rng) void {
        const d = &self.shapes[7];
        std.debug.assert(d.kind == 7);
        d.params[4] = rng.float(0.5, 0.9);
        d.params[5] = rng.float(1.1, 2);
        d.params[6] = rng.float(0.5, 2);
    }

    fn tables(self: *const TableDescs) Tables {
        return .{ .shapes = &self.shapes, .num_shapes = self.shapes.len, .materials = &self.materials, .num_materials = self.materials.len, .filters = &self.filters, .num_filters = self.filters.len };
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// BodyInterface

/// Branches that only some inputs reach, each must be hit `min_hits` times
const Coverage = struct {
    const min_hits = 10;

    out_of_bodies: u32 = 0,
    custom_id_taken: u32 = 0,
    lock_failed_getter: u32 = 0,
    position_unchanged: u32 = 0,
    buoyancy_applied: u32 = 0,
    buoyancy_not_applied: u32 = 0,
    aabox_activated_several: u32 = 0,
    activated_by_velocity: u32 = 0,
    not_activated_near_zero: u32 = 0,
    reassigned: u32 = 0,
    reassign_failed: u32 = 0,
    kinematic_moved: u32 = 0,
    set_shape_mass: u32 = 0,
    became_static: u32 = 0,

    fn check(self: *const Coverage) !void {
        var ok = true;
        inline for (@typeInfo(Coverage).@"struct".fields) |field| {
            if (@field(self, field.name) < min_hits) {
                std.debug.print("BodyInterface parity: coverage {s} = {d} < {d}\n", .{ field.name, @field(self, field.name), min_hits });
                ok = false;
            }
        }
        if (!ok) return error.TestCoverage;
    }
};

/// What the generator knows about a body (from the Zolt side, before the operation)
const BodyInfo = struct {
    id: BodyID,
    live: bool,
    added: bool,
    active: bool,
    motion_type: MotionType,
    has_motion_properties: bool,
};

fn bodyInfo(world: *World, raw_id: u32) BodyInfo {
    const id = BodyID.init(raw_id);
    const body = if (id.isInvalid()) null else world.system.body_manager.tryGetBody(id);
    if (body) |b| return .{ .id = id, .live = true, .added = b.isInBroadPhase(), .active = b.isActive(), .motion_type = b.getMotionType(), .has_motion_properties = b.getMotionPropertiesUnchecked() != null };
    return .{ .id = id, .live = false, .added = false, .active = false, .motion_type = .static, .has_motion_properties = false };
}

/// A random body ID: usually a live body that matches `filter`, sometimes a stale one or the invalid ID
fn pickBody(rng: *Rng, world: *World, known: []const u32, filter: anytype) ?u32 {
    if (rng.intRange(u32, 0, 9) == 0) {
        if (rng.intRange(u32, 0, 1) == 0 or known.len == 0) return BodyID.invalid_body_id;
        // Any known ID: a stale one (the lock fails), or a live body when it matches the filter (the filters exclude
        // the bodies Jolt asserts on)
        const raw = known[rng.intRange(usize, 0, known.len - 1)];
        const info = bodyInfo(world, raw);
        return if (!info.live or filter.accept(info)) raw else BodyID.invalid_body_id;
    }
    var candidates: [64]u32 = undefined;
    var n: usize = 0;
    for (known) |raw| {
        const info = bodyInfo(world, raw);
        if (info.live and filter.accept(info) and n < candidates.len) {
            candidates[n] = raw;
            n += 1;
        }
    }
    if (n == 0) return null;
    return candidates[rng.intRange(usize, 0, n - 1)];
}

const AnyLive = struct {
    fn accept(_: AnyLive, _: BodyInfo) bool {
        return true;
    }
};
const Added = struct {
    fn accept(_: Added, info: BodyInfo) bool {
        return info.added;
    }
};
const NotAdded = struct {
    fn accept(_: NotAdded, info: BodyInfo) bool {
        return !info.added;
    }
};
const NonStatic = struct {
    fn accept(_: NonStatic, info: BodyInfo) bool {
        return info.motion_type != .static;
    }
};
const Dynamic = struct {
    fn accept(_: Dynamic, info: BodyInfo) bool {
        return info.motion_type == .dynamic;
    }
};
const WithMotionProperties = struct {
    fn accept(_: WithMotionProperties, info: BodyInfo) bool {
        return info.has_motion_properties;
    }
};
const AddedActivatable = struct {
    /// Operations that may activate the body need it in the broad phase (Jolt asserts in ActivateBodies)
    fn accept(_: AddedActivatable, info: BodyInfo) bool {
        return info.added;
    }
};

/// A list of bodies for the list operations (unique, all matching `filter`)
fn pickBodies(rng: *Rng, world: *World, known: []const u32, filter: anytype, op: *Op) void {
    var n: u32 = 0;
    if (known.len == 0) {
        op.i[1] = 0;
        return;
    }
    const want = rng.intRange(u32, 0, 6);
    var tries: u32 = 0;
    while (n < want and tries < 30) : (tries += 1) {
        const raw = known[rng.intRange(usize, 0, known.len - 1)];
        const info = bodyInfo(world, raw);
        if (!info.live or !filter.accept(info)) continue;
        if (std.mem.indexOfScalar(u32, op.i[2 .. 2 + n], raw) != null) continue;
        op.i[2 + n] = raw;
        n += 1;
    }
    op.i[1] = n;
}

fn randomActivation(rng: *Rng) u32 {
    return rng.intRange(u32, 0, 1);
}

/// Fill a random operation, returns false when no body fits the operation
fn randomOp(rng: *Rng, world: *World, known: []const u32, num_shapes: u32, num_filters: u32, op: *Op) bool {
    op.* = .{};
    const kind: OpKind = @enumFromInt(rng.intRange(u32, 0, @intFromEnum(OpKind.destroy_bodies)));
    op.kind = @intFromEnum(kind);
    op.i[0] = randomActivation(rng);
    storeR3(randomPosition(rng), op.r[0..3]);
    storeQuat(randomQuat(rng), op.f[0..4]);
    switch (kind) {
        // Operations that are safe on any body: the lock fails on stale / invalid IDs, the body interface checks the
        // motion type itself
        .set_position_and_rotation, .set_position, .set_rotation, .get_position_and_rotation, .get_position, .get_center_of_mass_position, .get_rotation, .get_world_transform, .get_center_of_mass_transform, .get_linear_and_angular_velocity, .get_linear_velocity, .get_angular_velocity, .get_point_velocity, .get_body_type, .get_motion_type, .get_motion_quality, .get_restitution, .get_friction, .get_gravity_factor, .get_max_linear_velocity, .get_max_angular_velocity, .get_use_manifold_reduction, .is_sensor, .get_collision_group, .get_transformed_shape, .get_user_data, .get_object_layer, .is_active, .remove_body, .is_added, .invalidate_contact_cache, .get_shape, .set_gravity_factor, .set_max_linear_velocity, .set_max_angular_velocity, .set_motion_quality => {
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            op.f[0] = switch (kind) {
                .set_max_linear_velocity => rng.float(1, 600),
                .set_max_angular_velocity => rng.float(1, 60),
                .set_gravity_factor => rng.float(-2, 2),
                else => op.f[0],
            };
            op.i[2] = rng.intRange(u32, 0, 1); // Motion quality
        },
        .set_position_and_rotation_when_changed => {
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            const info = bodyInfo(world, op.body);
            if (info.live and rng.intRange(u32, 0, 2) != 0) {
                // The current position and rotation, or a change just below or above the tolerance of IsClose (a squared
                // distance of 1.0e-12 for Vec3, 1.0e-24 for DVec3)
                const body = world.system.body_manager.getBody(info.id);
                var p = body.getPosition();
                switch (rng.intRange(u32, 0, 2)) {
                    0 => {},
                    1 => p = p.addVec3(Vec3.replicate(if (Core.double_precision) 1.0e-13 else 1.0e-7)),
                    else => p = p.addVec3(Vec3.replicate(if (Core.double_precision) 1.0e-11 else 1.0e-5)),
                }
                storeR3(p, op.r[0..3]);
                storeQuat(body.getRotation(), op.f[0..4]);
            }
        },
        .set_linear_and_angular_velocity, .set_linear_velocity, .add_linear_velocity, .add_linear_and_angular_velocity, .set_angular_velocity => {
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            store3(randomVelocity(rng), op.f[0..3]);
            store3(randomVelocity(rng), op.f[3..6]);
        },
        .set_position_rotation_and_velocity => {
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            store3(randomVelocity(rng), op.f[4..7]);
            store3(randomVelocity(rng), op.f[7..10]);
        },
        .move_kinematic => {
            op.body = pickBody(rng, world, known, NonStatic{}) orelse return false;
            op.f[4] = rng.float(0.01, 0.1);
            const info = bodyInfo(world, op.body);
            if (info.live and rng.intRange(u32, 0, 3) == 0) {
                // Move to where the body is: zero velocity
                const body = world.system.body_manager.getBody(info.id);
                storeR3(body.getPosition(), op.r[0..3]);
                storeQuat(body.getRotation(), op.f[0..4]);
            }
        },
        .add_force, .add_force_at_position, .add_torque, .add_force_and_torque => {
            // Activates the body (Jolt asserts that it is in the broad phase) unless dont_activate
            op.body = (if (op.i[0] == 0) pickBody(rng, world, known, AddedActivatable{}) else pickBody(rng, world, known, AnyLive{})) orelse return false;
            store3(randomVec3(rng, 100), op.f[0..3]);
            store3(randomVec3(rng, 100), op.f[3..6]);
        },
        .add_impulse, .add_impulse_at_position, .add_angular_impulse => {
            op.body = pickBody(rng, world, known, AddedActivatable{}) orelse return false;
            store3(randomVec3(rng, 10), op.f[0..3]);
        },
        .apply_buoyancy_impulse => {
            op.body = pickBody(rng, world, known, AddedActivatable{}) orelse return false;
            const info = bodyInfo(world, op.body);
            if (info.live) {
                // A surface through or near the body
                const body = world.system.body_manager.getBody(info.id);
                storeR3(body.getCenterOfMassPosition().addVec3(randomVec3(rng, 1.5)), op.r[0..3]);
            }
            store3(Vec3.init(rng.float(-0.3, 0.3), 1, rng.float(-0.3, 0.3)).normalized(), op.f[0..3]);
            op.f[3] = rng.float(0.5, 2);
            op.f[4] = rng.float(0, 1);
            op.f[5] = rng.float(0, 1);
            store3(randomVec3(rng, 2), op.f[6..9]);
            store3(Vec3.init(0, -9.81, 0), op.f[9..12]);
            op.f[12] = rng.float(0.01, 0.1);
        },
        .set_motion_type => {
            // Only to static without motion properties, deactivated by the interface
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            const info = bodyInfo(world, op.body);
            op.i[2] = if (info.live and !info.has_motion_properties) 0 else rng.intRange(u32, 0, 2);
        },
        .get_inverse_inertia => {
            op.body = pickBody(rng, world, known, Dynamic{}) orelse return false;
            if (!bodyInfo(world, op.body).live) op.body = BodyID.invalid_body_id; // A stale body may have become static
        },
        .set_restitution, .set_friction => {
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            op.f[0] = rng.float(-0.5, 1.5);
        },
        .set_use_manifold_reduction, .set_is_sensor => {
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            op.i[2] = rng.intRange(u32, 0, 1);
        },
        .set_collision_group => {
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            op.i[2] = rng.intRange(u32, 0, 10);
            op.i[3] = rng.intRange(u32, 0, 5);
            op.i[4] = if (rng.intRange(u32, 0, 2) == 0) none else rng.intRange(u32, 0, num_filters - 1);
        },
        .set_user_data => {
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            op.i[2] = rng.next();
            op.i[3] = rng.next();
        },
        .get_material => {
            // The empty sub shape ID, or the box of a static compound (Jolt asserts on invalid IDs of convex shapes)
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            op.i[2] = SubShapeID.empty_value;
            const info = bodyInfo(world, op.body);
            if (info.live and rng.intRange(u32, 0, 1) == 0) {
                const index = world.tables.shapeIndex(world.system.body_manager.getBody(info.id).getShape());
                if (index != none and world.tables.shape_descs[index].kind == 6) op.i[2] = 0xfffffffe;
            }
        },
        .set_shape => {
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            const info = bodyInfo(world, op.body);
            op.i[2] = rng.intRange(u32, 0, num_shapes - 1);
            op.i[3] = rng.intRange(u32, 0, 1);
            if (op.i[0] == 0 and !info.added) op.i[0] = 1; // The interface only activates bodies in the broad phase
        },
        .notify_shape_changed => {
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            store3(randomVec3(rng, 0.5), op.f[0..3]);
            op.i[3] = rng.intRange(u32, 0, 1);
        },
        .set_object_layer => {
            op.body = pickBody(rng, world, known, AnyLive{}) orelse return false;
            op.i[2] = rng.intRange(u32, 0, 10);
        },
        .activate_body => op.body = pickBody(rng, world, known, AddedActivatable{}) orelse return false,
        .activate_bodies => pickBodies(rng, world, known, Added{}, op),
        .activate_bodies_in_aabox => {
            const center = randomVec3(rng, 30);
            const half = Vec3.replicate(rng.float(1, 40));
            store3(center.sub(half), op.f[0..3]);
            store3(center.add(half), op.f[3..6]);
        },
        .deactivate_body => op.body = pickBody(rng, world, known, Added{}) orelse return false,
        .deactivate_bodies, .remove_bodies => pickBodies(rng, world, known, Added{}, op),
        .reset_sleep_timer => op.body = pickBody(rng, world, known, WithMotionProperties{}) orelse return false,
        .add_body => op.body = pickBody(rng, world, known, NotAdded{}) orelse return false,
        .add_bodies, .add_bodies_abort, .destroy_bodies, .unassign_and_destroy => pickBodies(rng, world, known, NotAdded{}, op),
        .destroy_body => op.body = pickBody(rng, world, known, NotAdded{}) orelse return false,
        .unassign_and_reassign => {
            op.body = pickBody(rng, world, known, NotAdded{}) orelse return false;
            const info = bodyInfo(world, op.body);
            if (!info.live) return false; // UnassignBodyID needs a body
            op.i[2] = switch (rng.intRange(u32, 0, 2)) {
                0 => BodyID.invalid_body_id,
                1 => op.body, // Taken: the body just released it, so this succeeds
                else => BodyID.fromIndexAndSequenceNumber(rng.intRange(u32, 0, 20), @intCast(rng.intRange(u32, 0, 255))).getIndexAndSequenceNumber(),
            };
        },
    }
    // Stale / invalid bodies that were picked for an operation are fine (the lock fails on both sides), except for
    // DestroyBody, which calls the BodyManager without a lock (Jolt asserts on a stale ID)
    if (kind == .destroy_body and !bodyInfo(world, op.body).live) return false;
    return true;
}

fn checkOut(checker: *Checker, kind: OpKind, zolt_out: *const Out, jolt_out: *const Out) void {
    checker.check(kind, zolt_out.*, jolt_out.*);
}

test "BodyInterface parity: struct sizes" {
    var sizes: [9]u32 = undefined;
    jolt.jolt_bi_struct_sizes(&sizes);
    try std.testing.expectEqual([9]u32{ @sizeOf(ShapeDesc), @sizeOf(MaterialDesc), @sizeOf(GroupFilterDesc), @sizeOf(Tables), @sizeOf(BodyDesc), @sizeOf(Op), @sizeOf(Out), @sizeOf(SharedSettingsDesc), @sizeOf(SoftBodyDesc) }, sizes);
}

test "BodyInterface parity: random operations" {
    const allocator = std.testing.allocator;
    var rng: Rng = .{ .state = 0x51ed2704 };
    var out_checker: Checker = .{ .name = "BodyInterface parity (returned values)" };
    var state_checker: Checker = .{ .name = "BodyInterface parity (snapshot)" };
    var id_checker: Checker = .{ .name = "BodyInterface parity (created IDs)" };
    var coverage: Coverage = .{};

    const num_worlds = 60;
    const ops_per_world = 300;
    var num_ops: [@typeInfo(OpKind).@"enum".fields.len]u32 = @splat(0);

    for (0..num_worlds) |world_index| {
        const table_descs = TableDescs.random(&rng);
        const tables = table_descs.tables();
        const max_bodies: u32 = rng.intRange(u32, 6, 24);
        var world: World = undefined;
        try world.init(allocator, max_bodies, rng.intRange(u32, 0, 4), &tables);
        defer world.deinit();

        var known: std.ArrayList(u32) = .empty;
        defer known.deinit(allocator);

        var step: u32 = 0;
        while (step < ops_per_world) : (step += 1) {
            var context_buffer: [128]u8 = undefined;
            if (step < 4 or rng.intRange(u32, 0, 5) == 0) {
                // Create a body with one of the creation functions
                const desc = randomBodyDesc(&rng, @intCast(table_descs.shapes.len), @intCast(table_descs.filters.len), .{ .allow_settings = false });
                const mode = rng.intRange(u32, 0, 5);
                const custom_id = BodyID.fromIndexAndSequenceNumber(rng.intRange(u32, 0, max_bodies + 2), @intCast(rng.intRange(u32, 0, 255))).getIndexAndSequenceNumber();
                const activation = randomActivation(&rng);
                const zolt_id = try world.createBody(&desc, mode, custom_id, @enumFromInt(activation));
                const jolt_id = jolt.jolt_bi_create_body(world.jolt_world, &desc, mode, custom_id, activation);
                id_checker.check(.{ world_index, step, mode }, zolt_id, jolt_id);
                if (zolt_id == BodyID.invalid_body_id) {
                    if (mode == 3 or mode == 5) coverage.custom_id_taken += 1 else coverage.out_of_bodies += 1;
                } else {
                    try known.append(allocator, zolt_id);
                }
                try world.compareSnapshots(&state_checker, std.fmt.bufPrint(&context_buffer, "world {d} step {d}: create mode {d}", .{ world_index, step, mode }) catch "create");
                continue;
            }

            var o: Op = undefined;
            if (!randomOp(&rng, &world, known.items, @intCast(table_descs.shapes.len), @intCast(table_descs.filters.len), &o)) continue;
            const kind: OpKind = @enumFromInt(o.kind);
            num_ops[o.kind] += 1;

            // Coverage of the branches (from the Zolt side before the operation)
            const before = bodyInfo(&world, o.body);
            const num_active_before = world.system.getNumActiveBodies(.rigid_body);
            const position_before = if (before.live) world.system.body_manager.getBody(before.id).getPosition() else RVec3.zero();

            var zolt_out: Out = .{};
            var jolt_out: Out = .{};
            try world.op(&o, &zolt_out);
            jolt.jolt_bi_op(world.jolt_world, &o, &jolt_out);
            checkOut(&out_checker, kind, &zolt_out, &jolt_out);

            // Remember IDs that a reassign created
            if (kind == .unassign_and_reassign) {
                if (zolt_out.i[0] != BodyID.invalid_body_id) {
                    coverage.reassigned += 1;
                    try known.append(allocator, zolt_out.i[0]);
                } else coverage.reassign_failed += 1;
            }

            const after = bodyInfo(&world, o.body);
            const num_active_after = world.system.getNumActiveBodies(.rigid_body);
            if (!before.live) {
                switch (kind) {
                    .get_position, .get_rotation, .get_friction, .get_motion_type, .get_transformed_shape, .get_user_data, .get_material, .get_collision_group, .get_object_layer, .is_active, .is_added => coverage.lock_failed_getter += 1,
                    else => {},
                }
            } else switch (kind) {
                .set_position_and_rotation_when_changed => {
                    // A change below the tolerance: the body did not move
                    const position_after = world.system.body_manager.getBody(before.id).getPosition();
                    if (!loadR3(o.r[0..3]).eql(position_before) and position_after.eql(position_before))
                        coverage.position_unchanged += 1;
                },
                .apply_buoyancy_impulse => if (zolt_out.i[0] != 0) {
                    coverage.buoyancy_applied += 1;
                } else {
                    coverage.buoyancy_not_applied += 1;
                },
                .set_linear_velocity, .set_linear_and_angular_velocity, .set_angular_velocity => if (before.added and !before.active and before.motion_type != .static) {
                    if (after.active) coverage.activated_by_velocity += 1 else coverage.not_activated_near_zero += 1;
                },
                .move_kinematic => if (before.motion_type == .kinematic) {
                    coverage.kinematic_moved += 1;
                },
                .set_shape => if (o.i[3] != 0 and before.has_motion_properties) {
                    coverage.set_shape_mass += 1;
                },
                .set_motion_type => if (o.i[2] == 0 and before.active) {
                    coverage.became_static += 1;
                },
                else => {},
            }
            if (kind == .activate_bodies_in_aabox and num_active_after >= num_active_before + 2)
                coverage.aabox_activated_several += 1;

            try world.compareSnapshots(&state_checker, std.fmt.bufPrint(&context_buffer, "world {d} step {d}: {s} body {x}", .{ world_index, step, @tagName(kind), o.body }) catch "op");
            if (state_checker.mismatches > 0 or out_checker.mismatches > 0) break;
        }
        if (state_checker.mismatches > 0 or out_checker.mismatches > 0 or id_checker.mismatches > 0) break;
    }

    try fw.finishAll(&.{ &out_checker, &state_checker, &id_checker });
    for (num_ops, 0..) |n, k| {
        if (n < 20) {
            std.debug.print("BodyInterface parity: operation {s} ran {d} times\n", .{ @tagName(@as(OpKind, @enumFromInt(k))), n });
            return error.TestCoverage;
        }
    }
    try coverage.check();
}

// ---------------------------------------------------------------------------------------------------------------------
// BodyCreationSettings and PhysicsScene binary state

const RestoreMaps = struct {
    shapes: BodyCreationSettings.IDToShapeMap = .empty,
    materials: BodyCreationSettings.IDToMaterialMap = .empty,
    group_filters: BodyCreationSettings.IDToGroupFilterMap = .empty,

    fn deinit(self: *RestoreMaps, allocator: Allocator) void {
        for (self.shapes.items) |*s| s.deinit();
        self.shapes.deinit(allocator);
        for (self.materials.items) |*m| m.deinit();
        self.materials.deinit(allocator);
        for (self.group_filters.items) |*g| g.deinit();
        self.group_filters.deinit(allocator);
    }
};

const SaveMaps = struct {
    shapes: BodyCreationSettings.ShapeToIDMap = .empty,
    materials: BodyCreationSettings.MaterialToIDMap = .empty,
    group_filters: BodyCreationSettings.GroupFilterToIDMap = .empty,

    fn deinit(self: *SaveMaps, allocator: Allocator) void {
        self.shapes.deinit(allocator);
        self.materials.deinit(allocator);
        self.group_filters.deinit(allocator);
    }
};

/// The result of a restore + save: the bytes, or the error text
const Resaved = struct {
    bytes: std.ArrayList(u8) = .empty,
    err: ?[]const u8 = null,
    err_buffer: [128]u8 = undefined,

    fn deinit(self: *Resaved, allocator: Allocator) void {
        self.bytes.deinit(allocator);
    }

    fn setError(self: *Resaved, text: []const u8) void {
        const n = @min(text.len, self.err_buffer.len);
        @memcpy(self.err_buffer[0..n], text[0..n]);
        self.err = self.err_buffer[0..n];
    }

    fn eql(self: *const Resaved, other: *const Resaved) bool {
        if (self.err) |e| return other.err != null and std.mem.eql(u8, e, other.err.?);
        return other.err == null and std.mem.eql(u8, self.bytes.items, other.bytes.items);
    }
};

/// SaveWithChildren of the settings (jolt_bi_bcs_save on the Zolt side)
fn zoltBcsSave(allocator: Allocator, bodies: []const BodyCreationSettings, save_shapes: bool, save_filters: bool, out: *std.Io.Writer.Allocating) !void {
    var maps: SaveMaps = .{};
    defer maps.deinit(allocator);
    var wrapper = StreamOutWrapper.init(&out.writer);
    for (bodies) |*b|
        try b.saveWithChildren(allocator, wrapper.streamOut(), if (save_shapes) &maps.shapes else null, if (save_shapes) &maps.materials else null, if (save_filters) &maps.group_filters else null);
}

/// jolt_bi_bcs_resave on the Zolt side
fn zoltBcsResave(allocator: Allocator, bytes: []const u8, num_bodies: u32, save_shapes: bool, save_filters: bool, out: *Resaved) !void {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    var maps: RestoreMaps = .{};
    defer maps.deinit(allocator);
    var bodies: std.ArrayList(BodyCreationSettings) = .empty;
    defer {
        for (bodies.items) |*b| b.deinit();
        bodies.deinit(allocator);
    }
    for (0..num_bodies) |_| {
        var result = try BodyCreationSettings.restoreWithChildren(allocator, in.streamIn(), &maps.shapes, &maps.materials, &maps.group_filters);
        defer result.deinit();
        if (result.hasError()) {
            out.setError(result.getError());
            return;
        }
        try bodies.append(allocator, result.get().clone());
    }
    var writer: std.Io.Writer.Allocating = .init(allocator);
    defer writer.deinit();
    try zoltBcsSave(allocator, bodies.items, save_shapes, save_filters, &writer);
    try out.bytes.appendSlice(allocator, writer.written());
}

fn joltResaved(allocator: Allocator, size: u32, buffer: []const u8, err: []const u8, out: *Resaved) !void {
    if (size == none) {
        out.setError(std.mem.sliceTo(err, 0));
    } else {
        if (size > buffer.len) return error.TestUnexpectedResult;
        try out.bytes.appendSlice(allocator, buffer[0..size]);
    }
}

var big_buffer: [1 << 20]u8 = undefined;
var big_buffer2: [1 << 20]u8 = undefined;

test "BodyCreationSettings parity: binary state" {
    const allocator = std.testing.allocator;
    var rng: Rng = .{ .state = 0x0bc5_5afe };
    var save_checker: Checker = .{ .name = "BodyCreationSettings parity (SaveWithChildren bytes)" };
    var resave_checker: Checker = .{ .name = "BodyCreationSettings parity (restore + save of the other side's bytes)" };
    var error_checker: Checker = .{ .name = "BodyCreationSettings parity (truncated streams)" };
    var num_errors: u32 = 0;

    for (0..400) |iteration| {
        var table_descs = TableDescs.random(&rng);
        if (rng.intRange(u32, 0, 1) == 0) table_descs.makeSphereScaleInvalid(&rng);
        const tables = table_descs.tables();
        const save_shapes = rng.intRange(u32, 0, 1) != 0;
        const save_filters = rng.intRange(u32, 0, 1) != 0;
        var descs: [8]BodyDesc = undefined;
        const num_bodies = rng.intRange(u32, 1, descs.len);
        for (descs[0..num_bodies]) |*d| d.* = randomBodyDesc(&rng, @intCast(table_descs.shapes.len), @intCast(table_descs.filters.len), .{ .allow_no_shape = !save_shapes });

        // Zolt
        var objects: TableObjects = undefined;
        try objects.init(allocator, &tables);
        defer objects.deinit();
        var bodies: std.ArrayList(BodyCreationSettings) = .empty;
        defer {
            for (bodies.items) |*b| b.deinit();
            bodies.deinit(allocator);
        }
        for (descs[0..num_bodies]) |*d| try bodies.append(allocator, try objects.makeBodyCreationSettings(d));
        var zolt_bytes: std.Io.Writer.Allocating = .init(allocator);
        defer zolt_bytes.deinit();
        try zoltBcsSave(allocator, bodies.items, save_shapes, save_filters, &zolt_bytes);

        // C++
        const jolt_size = jolt.jolt_bi_bcs_save(&tables, &descs, num_bodies, @intFromBool(save_shapes), @intFromBool(save_filters), &big_buffer, big_buffer.len);
        try std.testing.expect(jolt_size <= big_buffer.len);
        const jolt_bytes = big_buffer[0..jolt_size];
        save_checker.check(iteration, std.mem.eql(u8, zolt_bytes.written(), jolt_bytes), true);

        // Each side restores the other side's bytes and saves them again
        var err_buffer: [128]u8 = undefined;
        var zolt_resaved: Resaved = .{};
        defer zolt_resaved.deinit(allocator);
        try zoltBcsResave(allocator, jolt_bytes, num_bodies, save_shapes, save_filters, &zolt_resaved);
        var jolt_resaved: Resaved = .{};
        defer jolt_resaved.deinit(allocator);
        try joltResaved(allocator, jolt.jolt_bi_bcs_resave(zolt_bytes.written().ptr, @intCast(zolt_bytes.written().len), num_bodies, @intFromBool(save_shapes), @intFromBool(save_filters), &big_buffer2, big_buffer2.len, &err_buffer, err_buffer.len), &big_buffer2, &err_buffer, &jolt_resaved);
        resave_checker.check(iteration, zolt_resaved.err == null and std.mem.eql(u8, zolt_resaved.bytes.items, jolt_bytes), true);
        resave_checker.check(iteration, jolt_resaved.err == null and std.mem.eql(u8, jolt_resaved.bytes.items, zolt_bytes.written()), true);

        // Truncated streams: the same error (or the same result) on both sides
        for (0..4) |_| {
            const len = rng.intRange(usize, 0, zolt_bytes.written().len);
            var z: Resaved = .{};
            defer z.deinit(allocator);
            try zoltBcsResave(allocator, zolt_bytes.written()[0..len], num_bodies, save_shapes, save_filters, &z);
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltResaved(allocator, jolt.jolt_bi_bcs_resave(zolt_bytes.written().ptr, @intCast(len), num_bodies, @intFromBool(save_shapes), @intFromBool(save_filters), &big_buffer2, big_buffer2.len, &err_buffer, err_buffer.len), &big_buffer2, &err_buffer, &j);
            if (z.err != null) num_errors += 1;
            error_checker.check(.{ iteration, len, z.err orelse "", j.err orelse "" }, z.eql(&j), true);
        }
    }

    try fw.finishAll(&.{ &save_checker, &resave_checker, &error_checker });
    try std.testing.expect(num_errors > 100);
}

// ---------------------------------------------------------------------------------------------------------------------
// Soft bodies: the shared settings table, the descriptions and the binary state of SoftBodyCreationSettings

/// A cloth of 2 x 2 to 5 x 5 vertices in the XZ plane with the constraints of createConstraints (random vertex
/// attributes, bend type and kinematic corners), optimized
fn randomCloth(allocator: Allocator, rng: *Rng) !Ref(SoftBodySharedSettings) {
    const S = SoftBodySharedSettings;
    var ref: Ref(S) = .init(try S.create(allocator));
    errdefer ref.deinit();
    const s = ref.get().?;
    const grid_size = rng.intRange(u32, 2, 5);
    const spacing = rng.float(0.1, 0.5);
    const fixed_corners = rng.intRange(u32, 0, 1) == 0;
    for (0..grid_size) |z| for (0..grid_size) |x| {
        const corner = (x == 0 or x == grid_size - 1) and (z == 0 or z == grid_size - 1);
        const position: zolt.Float3 = .{ .x = @as(f32, @floatFromInt(x)) * spacing, .y = 0, .z = @as(f32, @floatFromInt(z)) * spacing };
        try s.vertices.append(allocator, .init(position, .{ .inv_mass = if (corner and fixed_corners) 0 else 1 }));
    };
    var z: u32 = 0;
    while (z < grid_size - 1) : (z += 1) {
        var x: u32 = 0;
        while (x < grid_size - 1) : (x += 1) {
            try s.addFace(.init(x + z * grid_size, x + (z + 1) * grid_size, x + 1 + (z + 1) * grid_size, .{}));
            try s.addFace(.init(x + z * grid_size, x + 1 + (z + 1) * grid_size, x + 1 + z * grid_size, .{}));
        }
    }
    const lra_types = [_]S.LRAType{ .none, .euclidean_distance, .geodesic_distance };
    const bend_types = [_]S.BendType{ .none, .distance, .dihedral };
    const attributes = [_]S.VertexAttributes{.init(rng.float(0, 1.0e-3), rng.float(0, 1.0e-3), rng.float(0, 1.0e-3), .{ .lra_type = lra_types[rng.intRange(usize, 0, 2)], .lra_max_distance_multiplier = rng.float(1, 1.2) })};
    try s.createConstraints(&attributes, .{ .bend_type = bend_types[rng.intRange(usize, 0, 2)] });
    try s.optimize();
    return ref;
}

/// The shared settings of a test: SaveBinaryState bytes of settings that Zolt builds (cubes and cloths, optimized, so
/// soft bodies can be created with them), with random materials of the material table (also the default material and
/// null). Initialized in place: the descriptions point to the bytes.
const SharedTable = struct {
    bytes: [3][]u8 = undefined,
    descs: [3]SharedSettingsDesc = undefined,
    len: u32 = 0,

    fn init(self: *SharedTable, allocator: Allocator, rng: *Rng, num_materials: u32) !void {
        self.* = .{};
        errdefer self.deinit(allocator);
        const n = rng.intRange(u32, 1, self.descs.len);
        for (0..n) |i| {
            var settings = if (rng.intRange(u32, 0, 2) == 0)
                try SoftBodySharedSettings.createCube(allocator, rng.intRange(u32, 2, 3), rng.float(0.2, 1.0))
            else
                try randomCloth(allocator, rng);
            defer settings.deinit();
            var out: std.Io.Writer.Allocating = .init(allocator);
            defer out.deinit();
            var w = StreamOutWrapper.init(&out.writer);
            settings.get().?.saveBinaryState(w.streamOut());
            if (w.isFailed()) return error.OutOfMemory; // The allocating writer only fails when it runs out of memory
            self.bytes[i] = try out.toOwnedSlice();
            self.len += 1;
            var d: SharedSettingsDesc = .{ .bytes = self.bytes[i].ptr, .size = @intCast(self.bytes[i].len), .num_materials = rng.intRange(u32, 1, 4) };
            for (d.materials[0..d.num_materials]) |*m| {
                const r = rng.intRange(u32, 0, num_materials + 1);
                m.* = if (r < num_materials) r else if (r == num_materials) default_material_index else none;
            }
            self.descs[i] = d;
        }
    }

    fn deinit(self: *SharedTable, allocator: Allocator) void {
        for (self.bytes[0..self.len]) |b| allocator.free(b);
        self.len = 0;
    }

    fn slice(self: *const SharedTable) []const SharedSettingsDesc {
        return self.descs[0..self.len];
    }
};

fn randomSoftBodyDesc(rng: *Rng, num_shared: u32, num_filters: u32, opts: struct { allow_no_shared: bool = false, max_bodies: u32 = 16 }) SoftBodyDesc {
    var d: SoftBodyDesc = .{};
    d.user_data = @as(u64, rng.next()) << 32 | rng.next();
    storeR3(randomPosition(rng), &d.position);
    storeQuat(randomQuat(rng), &d.rotation);
    d.linear_damping = rng.float(0, 1);
    d.max_linear_velocity = rng.float(10, 600);
    d.restitution = rng.float(0, 1);
    d.friction = rng.float(0, 1);
    d.pressure = if (rng.intRange(u32, 0, 1) == 0) 0 else rng.float(0, 1000);
    d.gravity_factor = rng.float(-2, 2);
    d.vertex_radius = rng.float(0, 0.1);
    d.num_iterations = rng.intRange(u32, 1, 10);
    d.object_layer = rng.intRange(u32, 0, 10);
    d.update_position = rng.intRange(u32, 0, 1);
    d.make_rotation_identity = rng.intRange(u32, 0, 1);
    d.allow_sleeping = rng.intRange(u32, 0, 1);
    d.faces_double_sided = rng.intRange(u32, 0, 1);
    d.shared = if (opts.allow_no_shared and rng.intRange(u32, 0, 4) == 0) none else rng.intRange(u32, 0, num_shared - 1);
    d.group_filter = if (rng.intRange(u32, 0, 2) == 0) none else rng.intRange(u32, 0, num_filters - 1);
    d.group_id = if (rng.intRange(u32, 0, 4) == 0) 0xffffffff else rng.intRange(u32, 0, 100);
    d.sub_group_id = rng.intRange(u32, 0, 5);
    d.create_mode = rng.intRange(u32, 0, 4);
    d.custom_id = BodyID.fromIndexAndSequenceNumber(rng.intRange(u32, 0, opts.max_bodies + 2), @intCast(rng.intRange(u32, 0, 255))).getIndexAndSequenceNumber();
    d.activation = randomActivation(rng);
    return d;
}

/// The maps of a soft body restore, released by deinit
const SoftBodyRestoreMaps = struct {
    settings: SoftBodyCreationSettings.IDToSharedSettingsMap = .empty,
    materials: SoftBodyCreationSettings.IDToMaterialMap = .empty,
    group_filters: SoftBodyCreationSettings.IDToGroupFilterMap = .empty,

    fn deinit(self: *SoftBodyRestoreMaps, allocator: Allocator) void {
        for (self.settings.items) |*s| s.deinit();
        self.settings.deinit(allocator);
        for (self.materials.items) |*m| m.deinit();
        self.materials.deinit(allocator);
        for (self.group_filters.items) |*g| g.deinit();
        self.group_filters.deinit(allocator);
    }
};

/// The maps of a soft body save
const SoftBodySaveMaps = struct {
    settings: SoftBodyCreationSettings.SharedSettingsToIDMap = .empty,
    materials: SoftBodyCreationSettings.MaterialToIDMap = .empty,
    group_filters: SoftBodyCreationSettings.GroupFilterToIDMap = .empty,

    fn deinit(self: *SoftBodySaveMaps, allocator: Allocator) void {
        self.settings.deinit(allocator);
        self.materials.deinit(allocator);
        self.group_filters.deinit(allocator);
    }

    /// saveWithChildren of `settings` with these maps (`save_settings` / `save_filters` select them)
    fn save(self: *SoftBodySaveMaps, allocator: Allocator, settings: *const SoftBodyCreationSettings, stream: zolt.StreamOut, save_settings: bool, save_filters: bool) !void {
        try settings.saveWithChildren(allocator, stream, if (save_settings) &self.settings else null, if (save_settings) &self.materials else null, if (save_filters) &self.group_filters else null);
    }
};

/// The shared settings of the table (SharedTable) on the Zolt side, released by deinit
const SharedObjects = struct {
    list: std.ArrayList(Ref(SoftBodySharedSettings)) = .empty,

    fn deinit(self: *SharedObjects, allocator: Allocator) void {
        for (self.list.items) |*s| s.deinit();
        self.list.deinit(allocator);
    }
};

/// SaveWithChildren of the soft body creation settings (jolt_bi_sb_save on the Zolt side)
fn zoltSbSave(allocator: Allocator, settings: []const SoftBodyCreationSettings, save_settings: bool, save_filters: bool, out: *std.Io.Writer.Allocating) !void {
    var maps: SoftBodySaveMaps = .{};
    defer maps.deinit(allocator);
    var wrapper = StreamOutWrapper.init(&out.writer);
    for (settings) |*s| try maps.save(allocator, s, wrapper.streamOut(), save_settings, save_filters);
}

/// jolt_bi_sb_resave on the Zolt side
fn zoltSbResave(allocator: Allocator, bytes: []const u8, num_bodies: u32, save_settings: bool, save_filters: bool, out: *Resaved) !void {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    var maps: SoftBodyRestoreMaps = .{};
    defer maps.deinit(allocator);
    var bodies: std.ArrayList(SoftBodyCreationSettings) = .empty;
    defer {
        for (bodies.items) |*b| b.deinit();
        bodies.deinit(allocator);
    }
    for (0..num_bodies) |_| {
        var result = try SoftBodyCreationSettings.restoreWithChildren(allocator, in.streamIn(), &maps.settings, &maps.materials, &maps.group_filters);
        defer result.deinit();
        if (result.hasError()) {
            out.setError(result.getError());
            return;
        }
        try bodies.append(allocator, result.get().clone());
    }
    var writer: std.Io.Writer.Allocating = .init(allocator);
    defer writer.deinit();
    try zoltSbSave(allocator, bodies.items, save_settings, save_filters, &writer);
    try out.bytes.appendSlice(allocator, writer.written());
}

test "SoftBodyCreationSettings parity: binary state" {
    const allocator = std.testing.allocator;
    var rng: Rng = .{ .state = 0x50f7_b0d1 };
    var save_checker: Checker = .{ .name = "SoftBodyCreationSettings parity (SaveWithChildren bytes)" };
    var resave_checker: Checker = .{ .name = "SoftBodyCreationSettings parity (restore + save of the other side's bytes)" };
    var error_checker: Checker = .{ .name = "SoftBodyCreationSettings parity (truncated streams)" };
    var num_errors: u32 = 0;
    var num_shared_saved: u32 = 0;

    for (0..300) |iteration| {
        const table_descs = TableDescs.random(&rng);
        const tables = table_descs.tables();
        var shared_table: SharedTable = undefined;
        try shared_table.init(allocator, &rng, @intCast(table_descs.materials.len));
        defer shared_table.deinit(allocator);
        const shared_descs = shared_table.slice();
        const save_settings = rng.intRange(u32, 0, 2) != 0;
        const save_filters = rng.intRange(u32, 0, 1) != 0;
        var descs: [8]SoftBodyDesc = undefined;
        const num_bodies = rng.intRange(u32, 1, descs.len);
        for (descs[0..num_bodies]) |*d| d.* = randomSoftBodyDesc(&rng, shared_table.len, @intCast(table_descs.filters.len), .{ .allow_no_shared = !save_settings });
        if (save_settings) num_shared_saved += num_bodies;

        // Zolt
        var objects: TableObjects = undefined;
        try objects.init(allocator, &tables);
        defer objects.deinit();
        var shared: SharedObjects = .{};
        defer shared.deinit(allocator);
        try objects.makeSharedSettings(shared_descs, &shared.list);
        var bodies: std.ArrayList(SoftBodyCreationSettings) = .empty;
        defer {
            for (bodies.items) |*b| b.deinit();
            bodies.deinit(allocator);
        }
        for (descs[0..num_bodies]) |*d| {
            var settings = objects.makeSoftBodyCreationSettings(d, shared.list.items);
            errdefer settings.deinit();
            try bodies.append(allocator, settings);
        }
        var zolt_bytes: std.Io.Writer.Allocating = .init(allocator);
        defer zolt_bytes.deinit();
        try zoltSbSave(allocator, bodies.items, save_settings, save_filters, &zolt_bytes);

        // C++
        const jolt_size = jolt.jolt_bi_sb_save(&tables, shared_descs.ptr, @intCast(shared_descs.len), &descs, num_bodies, @intFromBool(save_settings), @intFromBool(save_filters), &big_buffer, big_buffer.len);
        try std.testing.expect(jolt_size <= big_buffer.len);
        const jolt_bytes = big_buffer[0..jolt_size];
        save_checker.check(iteration, std.mem.eql(u8, zolt_bytes.written(), jolt_bytes), true);

        // Each side restores the other side's bytes and saves them again
        var err_buffer: [128]u8 = undefined;
        var zolt_resaved: Resaved = .{};
        defer zolt_resaved.deinit(allocator);
        try zoltSbResave(allocator, jolt_bytes, num_bodies, save_settings, save_filters, &zolt_resaved);
        var jolt_resaved: Resaved = .{};
        defer jolt_resaved.deinit(allocator);
        try joltResaved(allocator, jolt.jolt_bi_sb_resave(zolt_bytes.written().ptr, @intCast(zolt_bytes.written().len), num_bodies, @intFromBool(save_settings), @intFromBool(save_filters), &big_buffer2, big_buffer2.len, &err_buffer, err_buffer.len), &big_buffer2, &err_buffer, &jolt_resaved);
        resave_checker.check(iteration, zolt_resaved.err == null and std.mem.eql(u8, zolt_resaved.bytes.items, jolt_bytes), true);
        resave_checker.check(iteration, jolt_resaved.err == null and std.mem.eql(u8, jolt_resaved.bytes.items, zolt_bytes.written()), true);

        // Truncated streams: the same error (or the same result) on both sides
        for (0..4) |_| {
            const len = rng.intRange(usize, 0, zolt_bytes.written().len);
            var z: Resaved = .{};
            defer z.deinit(allocator);
            try zoltSbResave(allocator, zolt_bytes.written()[0..len], num_bodies, save_settings, save_filters, &z);
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltResaved(allocator, jolt.jolt_bi_sb_resave(zolt_bytes.written().ptr, @intCast(len), num_bodies, @intFromBool(save_settings), @intFromBool(save_filters), &big_buffer2, big_buffer2.len, &err_buffer, err_buffer.len), &big_buffer2, &err_buffer, &j);
            if (z.err != null) num_errors += 1;
            error_checker.check(.{ iteration, len, z.err orelse "", j.err orelse "" }, z.eql(&j), true);
        }
    }

    try fw.finishAll(&.{ &save_checker, &resave_checker, &error_checker });
    try std.testing.expect(num_errors > 100);
    try std.testing.expect(num_shared_saved > 300);
}

/// jolt_bi_sb_create on the Zolt side
fn zoltSbCreate(allocator: Allocator, tables: *const Tables, shared_descs: []const SharedSettingsDesc, descs: []const SoftBodyDesc, max_bodies: u32, save_settings: bool, save_filters: bool, out: *std.Io.Writer.Allocating) !void {
    var objects: TableObjects = undefined;
    try objects.init(allocator, tables);
    defer objects.deinit();
    var shared: SharedObjects = .{};
    defer shared.deinit(allocator);
    try objects.makeSharedSettings(shared_descs, &shared.list);

    var layers: TestLayers.BPLayerInterfaceImpl = .init();
    var object_vs_broad_phase: TestLayers.ObjectVsBroadPhaseLayerFilterImpl = .{};
    var pairs: TestLayers.ObjectLayerPairFilterImpl = .{};
    var system: PhysicsSystem = .empty;
    try system.init(allocator, io, max_bodies, 0, 64, 64, &layers.base, &object_vs_broad_phase.base, &pairs.base, .{ .broad_phase = .brute_force });
    defer system.deinit();
    const b = system.getBodyInterfaceMut();

    var wrapper = StreamOutWrapper.init(&out.writer);
    const stream = wrapper.streamOut();
    var ids: std.ArrayList(BodyID) = .empty;
    defer ids.deinit(allocator);
    for (descs) |*d| {
        var s = objects.makeSoftBodyCreationSettings(d, shared.list.items);
        defer s.deinit();
        const activation: Activation = @enumFromInt(d.activation);
        var id: BodyID = .invalid;
        switch (d.create_mode) {
            0, 2 => if (try b.createSoftBody(&s)) |body| {
                id = body.getID();
                if (d.create_mode == 0)
                    try b.addBody(id, activation);
            },
            1 => id = try b.createAndAddSoftBody(&s, activation),
            3 => if (try b.createSoftBodyWithID(BodyID.init(d.custom_id), &s)) |body| {
                id = body.getID();
                try b.addBody(id, activation);
            },
            else => {
                const body = try b.createSoftBodyWithoutID(&s);
                if (b.assignBodyID(body)) {
                    id = body.getID();
                    try b.addBody(id, activation);
                } else b.destroyBodyWithoutID(body);
            },
        }
        try ids.append(allocator, id);
        stream.write(id.getIndexAndSequenceNumber());
        stream.write(@as(u8, @intFromBool(!id.isInvalid() and b.isActive(id))));
        stream.write(@as(u8, @intFromBool(!id.isInvalid() and b.isAdded(id))));
    }

    var maps: SoftBodySaveMaps = .{};
    defer maps.deinit(allocator);
    for (ids.items) |id| {
        if (id.isInvalid()) continue;
        var lock: zolt.BodyLockRead = .init(system.getBodyLockInterface(), id);
        defer lock.deinit();
        var s = lock.getBody().getSoftBodyCreationSettings();
        defer s.deinit();
        try maps.save(allocator, &s, stream, save_settings, save_filters);
    }
    if (wrapper.isFailed()) return error.OutOfMemory; // The allocating writer only fails when it runs out of memory
}

test "SoftBodyCreationSettings parity: soft bodies created through BodyInterface" {
    const allocator = std.testing.allocator;
    var rng: Rng = .{ .state = 0xc4ea_7e5b };
    var checker: Checker = .{ .name = "SoftBodyCreationSettings parity (created soft bodies)" };
    var num_created: u32 = 0;
    var num_failed: u32 = 0;
    var mode_counts: [5]u32 = @splat(0);

    for (0..150) |iteration| {
        const table_descs = TableDescs.random(&rng);
        const tables = table_descs.tables();
        var shared_table: SharedTable = undefined;
        try shared_table.init(allocator, &rng, @intCast(table_descs.materials.len));
        defer shared_table.deinit(allocator);
        const shared_descs = shared_table.slice();
        const save_settings = rng.intRange(u32, 0, 1) != 0;
        const save_filters = rng.intRange(u32, 0, 1) != 0;
        var descs: [10]SoftBodyDesc = undefined;
        const num_bodies = rng.intRange(u32, 1, descs.len);
        const max_bodies = if (rng.intRange(u32, 0, 3) == 0) rng.intRange(u32, 1, num_bodies) else num_bodies + 2;
        for (descs[0..num_bodies]) |*d| {
            d.* = randomSoftBodyDesc(&rng, shared_table.len, @intCast(table_descs.filters.len), .{ .max_bodies = max_bodies });
            mode_counts[d.create_mode] += 1;
        }

        var zolt_bytes: std.Io.Writer.Allocating = .init(allocator);
        defer zolt_bytes.deinit();
        try zoltSbCreate(allocator, &tables, shared_descs, descs[0..num_bodies], max_bodies, save_settings, save_filters, &zolt_bytes);
        const jolt_size = jolt.jolt_bi_sb_create(&tables, shared_descs.ptr, @intCast(shared_descs.len), &descs, num_bodies, max_bodies, @intFromBool(save_settings), @intFromBool(save_filters), &big_buffer, big_buffer.len);
        try std.testing.expect(jolt_size <= big_buffer.len);
        checker.check(.{ iteration, save_settings, save_filters }, std.mem.eql(u8, zolt_bytes.written(), big_buffer[0..jolt_size]), true);

        // Coverage: the IDs at the start of the bytes (4 bytes ID, 2 flags per description)
        for (0..num_bodies) |k| {
            const id = std.mem.readInt(u32, zolt_bytes.written()[6 * k ..][0..4], .little);
            if (id == BodyID.invalid_body_id) num_failed += 1 else num_created += 1;
        }
    }

    try fw.finishAll(&.{&checker});
    try std.testing.expect(num_created > 300);
    try std.testing.expect(num_failed > 30);
    for (mode_counts) |c| try std.testing.expect(c > 50);
}

// ---------------------------------------------------------------------------------------------------------------------
// PhysicsScene

/// BIRigidBodiesOnly of BodyInterfaceReference.cpp: SaveState without the soft bodies (SoftBodyMotionProperties.saveState
/// comes with group S4; SoftBodyParity.zig and SimulationParity.zig compare the soft body state)
const RigidBodiesOnly = struct {
    pub const overrides = .{.shouldSaveBody};
    base: StateRecorderFilter = .init(@This()),

    pub fn shouldSaveBody(self: *const @This(), body: *const Body) bool {
        _ = self;
        return !body.isSoftBody();
    }
};

/// The snapshot of a Zolt PhysicsSystem after FixInvalidScales + CreateBodies (jolt_bi_scene_create_bodies)
fn zoltSceneCreateBodies(allocator: Allocator, bytes: []const u8, max_bodies: u32, out_result: *u32, out: *StateRecorderImpl) !void {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    var result = try PhysicsScene.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    if (result.hasError()) {
        out_result.* = none;
        return;
    }
    const scene = result.getPtr().?;
    const fixed = try scene.fixInvalidScales();
    {
        var data: std.Io.Writer.Allocating = .init(allocator);
        defer data.deinit();
        var w = StreamOutWrapper.init(&data.writer);
        try scene.saveBinaryState(w.streamOut(), true, false);
        out.writeBytes(data.written());
    }

    var layers: TestLayers.BPLayerInterfaceImpl = .init();
    var object_vs_broad_phase: TestLayers.ObjectVsBroadPhaseLayerFilterImpl = .{};
    var pairs: TestLayers.ObjectLayerPairFilterImpl = .{};
    var system: PhysicsSystem = .empty;
    try system.init(allocator, io, max_bodies, 0, 64, 64, &layers.base, &object_vs_broad_phase.base, &pairs.base, .{ .broad_phase = .brute_force });
    defer system.deinit();
    out_result.* = @as(u32, @intFromBool(try scene.createBodies(&system))) | @as(u32, if (fixed) 2 else 0);

    var ids: BodyIDVector = .empty;
    defer ids.deinit(allocator);
    try system.getBodies(allocator, &ids);
    var bodies: std.ArrayList(*const Body) = .empty;
    defer bodies.deinit(allocator);
    for (ids.items) |id| try bodies.append(allocator, system.getBodyLockInterfaceNoLock().tryGetBody(id).?);
    var active_ids: BodyIDVector = .empty;
    defer active_ids.deinit(allocator);
    try system.getActiveBodies(allocator, .rigid_body, &active_ids);
    var active: std.ArrayList(u32) = .empty;
    defer active.deinit(allocator);
    for (active_ids.items) |id| try active.append(allocator, id.getIndexAndSequenceNumber());
    std.mem.sort(u32, active.items, {}, std.sort.asc(u32)); // Unique IDs: any sort gives the order of QuickSort

    const no_tables: TableObjects = .{ .allocator = allocator };
    writeBodies(allocator, bodies.items, active.items, &no_tables, out.recorder());

    // The active soft bodies, sorted like the rigid bodies
    var active_soft_body_ids: BodyIDVector = .empty;
    defer active_soft_body_ids.deinit(allocator);
    try system.getActiveBodies(allocator, .soft_body, &active_soft_body_ids);
    var active_soft_bodies: std.ArrayList(u32) = .empty;
    defer active_soft_bodies.deinit(allocator);
    for (active_soft_body_ids.items) |id| try active_soft_bodies.append(allocator, id.getIndexAndSequenceNumber());
    std.mem.sort(u32, active_soft_bodies.items, {}, std.sort.asc(u32)); // Unique IDs: any sort gives the order of QuickSort
    out.recorder().write(@as(u32, @intCast(active_soft_bodies.items.len)));
    for (active_soft_bodies.items) |a| out.recorder().write(a);

    const rigid_bodies_only: RigidBodiesOnly = .{};
    try system.saveState(out.recorder(), .{ .state = .{ .bodies = true, .constraints = true }, .filter = &rigid_bodies_only.base });

    // The system back to a scene (the constraints through getConstraintSettings)
    var from_system = Ref(PhysicsScene).init(try PhysicsScene.create(allocator));
    defer from_system.deinit();
    try from_system.get().?.fromPhysicsSystem(&system);
    {
        var data: std.Io.Writer.Allocating = .init(allocator);
        defer data.deinit();
        var w = StreamOutWrapper.init(&data.writer);
        try from_system.get().?.saveBinaryState(w.streamOut(), true, false);
        out.writeBytes(data.written());
    }
}

test "PhysicsScene parity: binary state and CreateBodies" {
    const allocator = std.testing.allocator;
    var rng: Rng = .{ .state = 0x5ce7e000 };
    var save_checker: Checker = .{ .name = "PhysicsScene parity (SaveBinaryState bytes)" };
    var resave_checker: Checker = .{ .name = "PhysicsScene parity (restore + save of the other side's bytes)" };
    var error_checker: Checker = .{ .name = "PhysicsScene parity (truncated streams)" };
    var create_checker: Checker = .{ .name = "PhysicsScene parity (CreateBodies)" };
    var num_out_of_bodies: u32 = 0;
    var num_invalid_scales: u32 = 0;
    var total_constraints: u32 = 0;
    var total_soft_bodies: u32 = 0;
    var num_created_soft_bodies: u32 = 0;
    var num_created_soft_body_constraints: u32 = 0;
    var kind_counts: std.EnumArray(SceneConstraintKind, u32) = .initFill(0);

    for (0..300) |iteration| {
        var table_descs = TableDescs.random(&rng);
        const invalid_scale = rng.intRange(u32, 0, 1) == 0;
        if (invalid_scale) table_descs.makeSphereScaleInvalid(&rng);
        const tables = table_descs.tables();
        const save_shapes = rng.intRange(u32, 0, 3) != 0;
        const save_filters = rng.intRange(u32, 0, 1) != 0;
        var descs: [10]BodyDesc = undefined;
        const num_bodies = rng.intRange(u32, 0, descs.len);
        for (descs[0..num_bodies]) |*d| d.* = randomBodyDesc(&rng, @intCast(table_descs.shapes.len), @intCast(table_descs.filters.len), .{ .allow_no_shape = !save_shapes });

        // Soft bodies with shared settings (a scene always saves them)
        var shared_table: SharedTable = undefined;
        try shared_table.init(allocator, &rng, @intCast(table_descs.materials.len));
        defer shared_table.deinit(allocator);
        const shared_descs = shared_table.slice();
        var soft_descs: [4]SoftBodyDesc = undefined;
        const num_soft_bodies = rng.intRange(u32, 0, soft_descs.len);
        for (soft_descs[0..num_soft_bodies]) |*d| d.* = randomSoftBodyDesc(&rng, shared_table.len, @intCast(table_descs.filters.len), .{});
        total_soft_bodies += num_soft_bodies;

        // Constraints between the bodies, the soft bodies and the world. For a soft body FromPhysicsSystem stores the
        // index of the next rigid body (Jolt's quirk, R20), so the CreateBodies snapshot compares that index.
        var constraints: [6]SceneConstraint = undefined;
        const num_scene_bodies = num_bodies + num_soft_bodies;
        const num_constraints: u32 = if (num_scene_bodies == 0) 0 else rng.intRange(u32, 0, constraints.len);
        for (constraints[0..num_constraints]) |*c| c.* = SceneConstraint.random(&rng, num_scene_bodies);
        total_constraints += num_constraints;
        var num_soft_body_constraints: u32 = 0;
        var constraint_bodies: [constraints.len][2]u32 = undefined;
        var constraint_settings: [constraints.len]ConstraintsParity.CSSettings = undefined;
        for (constraints[0..num_constraints], 0..) |*c, k| {
            constraint_bodies[k] = c.bodies;
            constraint_settings[k] = c.settings;
            kind_counts.getPtr(@enumFromInt(c.settings.kind)).* += 1;
            for (c.bodies) |b| {
                if (b != PhysicsScene.fixed_to_world and b >= num_bodies) {
                    num_soft_body_constraints += 1;
                    break;
                }
            }
        }

        // Zolt
        var objects: TableObjects = undefined;
        try objects.init(allocator, &tables);
        defer objects.deinit();
        var scene = Ref(PhysicsScene).init(try PhysicsScene.create(allocator));
        defer scene.deinit();
        for (descs[0..num_bodies]) |*d| {
            var settings = try objects.makeBodyCreationSettings(d);
            defer settings.deinit();
            try scene.get().?.addBody(&settings);
        }
        for (constraints[0..num_constraints]) |*c| {
            var settings = try c.makeSettings(allocator);
            defer settings.deinit();
            try scene.get().?.addConstraint(zolt.virtual.downcast(zolt.TwoBodyConstraintSettings, settings.get().?), c.bodies[0], c.bodies[1]);
        }

        // The end of the constraints: the bytes of the scene without its soft bodies, without the soft body count
        var constraints_end: usize = 0;
        {
            var prefix: std.Io.Writer.Allocating = .init(allocator);
            defer prefix.deinit();
            var prefix_wrapper = StreamOutWrapper.init(&prefix.writer);
            try scene.get().?.saveBinaryState(prefix_wrapper.streamOut(), save_shapes, save_filters);
            constraints_end = prefix.written().len - 4;
        }

        var shared: SharedObjects = .{};
        defer shared.deinit(allocator);
        try objects.makeSharedSettings(shared_descs, &shared.list);
        for (soft_descs[0..num_soft_bodies]) |*d| {
            var settings = objects.makeSoftBodyCreationSettings(d, shared.list.items);
            defer settings.deinit();
            try scene.get().?.addSoftBody(&settings);
        }
        var zolt_bytes: std.Io.Writer.Allocating = .init(allocator);
        defer zolt_bytes.deinit();
        var wrapper = StreamOutWrapper.init(&zolt_bytes.writer);
        try scene.get().?.saveBinaryState(wrapper.streamOut(), save_shapes, save_filters);

        // C++
        const jolt_size = jolt.jolt_bi_scene_save(&tables, &descs, num_bodies, &constraint_bodies, &constraint_settings, num_constraints, shared_descs.ptr, @intCast(shared_descs.len), &soft_descs, num_soft_bodies, @intFromBool(save_shapes), @intFromBool(save_filters), &big_buffer, big_buffer.len);
        try std.testing.expect(jolt_size <= big_buffer.len);
        const jolt_bytes = big_buffer[0..jolt_size];
        save_checker.check(iteration, std.mem.eql(u8, zolt_bytes.written(), jolt_bytes), true);

        // Each side restores the other side's bytes and saves them again
        var err_buffer: [128]u8 = undefined;
        {
            var reader: std.Io.Reader = .fixed(jolt_bytes);
            var in = StreamInWrapper.init(&reader);
            var result = try PhysicsScene.restoreFromBinaryState(allocator, in.streamIn());
            defer result.deinit();
            var resaved: std.Io.Writer.Allocating = .init(allocator);
            defer resaved.deinit();
            if (result.isValid()) {
                var w = StreamOutWrapper.init(&resaved.writer);
                try result.getPtr().?.saveBinaryState(w.streamOut(), save_shapes, save_filters);
            }
            resave_checker.check(iteration, result.isValid() and std.mem.eql(u8, resaved.written(), jolt_bytes), true);
        }
        var jolt_resaved: Resaved = .{};
        defer jolt_resaved.deinit(allocator);
        try joltResaved(allocator, jolt.jolt_bi_scene_resave(zolt_bytes.written().ptr, @intCast(zolt_bytes.written().len), @intFromBool(save_shapes), @intFromBool(save_filters), &big_buffer2, big_buffer2.len, &err_buffer, err_buffer.len), &big_buffer2, &err_buffer, &jolt_resaved);
        resave_checker.check(iteration, jolt_resaved.err == null and std.mem.eql(u8, jolt_resaved.bytes.items, zolt_bytes.written()), true);

        // Truncated streams: the same error (or the same scene) on both sides
        for (0..3) |_| {
            const len = rng.intRange(usize, 0, zolt_bytes.written().len);
            var z: Resaved = .{};
            defer z.deinit(allocator);
            {
                var reader: std.Io.Reader = .fixed(zolt_bytes.written()[0..len]);
                var in = StreamInWrapper.init(&reader);
                var result = try PhysicsScene.restoreFromBinaryState(allocator, in.streamIn());
                defer result.deinit();
                if (result.hasError()) {
                    z.setError(result.getError());
                } else {
                    var resaved: std.Io.Writer.Allocating = .init(allocator);
                    defer resaved.deinit();
                    var w = StreamOutWrapper.init(&resaved.writer);
                    try result.getPtr().?.saveBinaryState(w.streamOut(), false, save_filters);
                    try z.bytes.appendSlice(allocator, resaved.written());
                }
            }
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltResaved(allocator, jolt.jolt_bi_scene_resave(zolt_bytes.written().ptr, @intCast(len), 0, @intFromBool(save_filters), &big_buffer2, big_buffer2.len, &err_buffer, err_buffer.len), &big_buffer2, &err_buffer, &j);

            // Jolt's Array::resize default-constructs the ConnectedConstraints and mBody1 / mBody2 have no initializer:
            // when the stream ends inside the body indices of the last constraint (the 8 bytes before the soft body
            // count), Jolt keeps whatever was in memory in the bytes that were not read (undefined, Zolt: 0). Those
            // bytes of the resaved scenes (the same place: body1, body2, soft body count 0 at the end) are not compared.
            const body_indices = constraints_end -| 8;
            if (num_constraints > 0 and len >= body_indices and len < body_indices + 8 and z.err == null and j.err == null and z.bytes.items.len == j.bytes.items.len and z.bytes.items.len >= 12) {
                const first_unread = z.bytes.items.len - 12 + (len - body_indices);
                @memset(z.bytes.items[first_unread .. z.bytes.items.len - 4], 0);
                @memset(j.bytes.items[first_unread .. j.bytes.items.len - 4], 0);
            }
            error_checker.check(.{ iteration, len, z.err orelse "", j.err orelse "" }, z.eql(&j), true);
        }

        // CreateBodies of the restored scene (with shapes), sometimes with too few bodies
        if (save_shapes) {
            const max_bodies = if (rng.intRange(u32, 0, 3) == 0) rng.intRange(u32, 1, @max(num_bodies, 1)) else num_bodies + 4;
            var zolt_result: u32 = 0;
            var zolt_snapshot = StateRecorderImpl.init(allocator);
            defer zolt_snapshot.deinit();
            try zoltSceneCreateBodies(allocator, zolt_bytes.written(), max_bodies, &zolt_result, &zolt_snapshot);
            var jolt_result: u32 = 0;
            const snapshot_size = jolt.jolt_bi_scene_create_bodies(zolt_bytes.written().ptr, @intCast(zolt_bytes.written().len), max_bodies, &jolt_result, &big_buffer2, big_buffer2.len);
            try std.testing.expect(snapshot_size <= big_buffer2.len);
            create_checker.check(.{ iteration, max_bodies }, zolt_result, jolt_result);
            create_checker.check(.{ iteration, max_bodies }, std.mem.eql(u8, zolt_snapshot.getData(), big_buffer2[0..snapshot_size]), true);
            if (zolt_result & 1 == 0) num_out_of_bodies += 1;
            if (zolt_result & 1 != 0) num_created_soft_bodies += num_soft_bodies;
            if (zolt_result & 1 != 0) num_created_soft_body_constraints += num_soft_body_constraints;
            if (invalid_scale) {
                for (descs[0..num_bodies]) |*d| {
                    if (d.shape == 7) {
                        num_invalid_scales += 1;
                        break;
                    }
                }
            }
        }
    }

    try fw.finishAll(&.{ &save_checker, &resave_checker, &error_checker, &create_checker });
    try std.testing.expect(num_out_of_bodies > 10);
    try std.testing.expect(num_invalid_scales > 10);
    try std.testing.expect(total_constraints > 300);
    try std.testing.expect(total_soft_bodies > 300);
    try std.testing.expect(num_created_soft_bodies > 150);
    try std.testing.expect(num_created_soft_body_constraints > 100);
    for (scene_constraint_kinds) |kind| {
        if (kind_counts.get(kind) < 300 / scene_constraint_kinds.len) {
            std.debug.print("PhysicsScene parity: {d} constraints of kind {t}\n", .{ kind_counts.get(kind), kind });
            return error.KindNotReached;
        }
    }
}
