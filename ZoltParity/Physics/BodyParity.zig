//! Parity tests for the Body package (Phase 5 F1): MotionProperties, Body, BodyCreationSettings and BodyManager.
//! Bodies are created through a BodyManager on both sides (AllocateBody + AddBody / AddBodyWithCustomID, marked as
//! being in the broad phase, ActivateBodies), from the same BodyCreationSettings (`BodyDesc`: every shape kind, given
//! as ShapeSettings or as Shape, every motion type, allowed DOFs, sensors, flags, mass override modes).
//!
//! Compared bit for bit on random inputs mixed with edge cases (static / kinematic / dynamic, sensors, zero inertia
//! axes from the allowed DOFs, very large velocities, far positions in double precision, rotated / translated / scaled /
//! offset / compound shapes):
//! - after creating a body and after every operation: every getter of the body and its motion properties (positions,
//!   rotation, the world / center of mass / inverse transforms, bounds, velocities, accumulated force and torque, mass,
//!   inertia, world inverse inertia, DOF masks, flags, indices) and the SaveState bytes of the body (`BodyOut`);
//! - the operations (`BodyOpKind`): SetMassProperties with every allowed DOF combination and random inertias,
//!   ApplyForceTorqueAndDragInternal, ApplyGyroscopicForceInternal, ClampLinearVelocity / ClampAngularVelocity, the
//!   velocity setters with their clamps, AddForce / AddTorque / AddImpulse / AddAngularImpulse at positions,
//!   MoveKinematic (Body and MotionProperties), SetPositionAndRotationInternal, UpdateSleepStateInternal (the sleep test
//!   accumulation), ApplyBuoyancyImpulse, GetSubmergedVolume, GetWorldSpaceSurfaceNormal, GetPointVelocity(COM),
//!   GetInverseInertiaForRotation, MultiplyWorldSpaceInverseInertiaByVector, SetShapeInternal (UpdateCenterOfMassInternal),
//!   the position / rotation / velocity steps, ScaleToMass, SetMotionType, GetBodyCreationSettings, the flag and
//!   MotionProperties setters, the contact cache flags;
//! - BodyCreationSettings::GetMassProperties for every EOverrideMassProperties mode and shape kind;
//! - a BodyManager sequence (create / add with custom IDs / destroy / activate / deactivate / SetMotionQuality / out of
//!   bodies / save / restore, validating restores): the body IDs, the active body order, the CCD count, GetBodyIDs, the
//!   body stats, GetMutexMask and the SaveState bytes after every step.
//! C ABI wrappers: ZoltParity/Physics/BodyReference.cpp.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const finishAll = fw.finishAll;

const Allocator = std.mem.Allocator;
const AllowedDOFs = zolt.AllowedDOFs;
const Body = zolt.Body;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyID = zolt.BodyID;
const BodyIDVector = zolt.BodyIDVector;
const BodyManager = zolt.BodyManager;
const BoxShapeSettings = zolt.BoxShapeSettings;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const CapsuleShapeSettings = zolt.CapsuleShapeSettings;
const Core = zolt.Core;
const CylinderShapeSettings = zolt.CylinderShapeSettings;
const MassProperties = zolt.MassProperties;
const Mat44 = zolt.Mat44;
const MotionProperties = zolt.MotionProperties;
const ObjectLayer = zolt.ObjectLayer;
const OffsetCenterOfMassShapeSettings = zolt.OffsetCenterOfMassShapeSettings;
const Quat = zolt.Quat;
const Real = zolt.Real;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const RMat44 = zolt.RMat44;
const RotatedTranslatedShapeSettings = zolt.RotatedTranslatedShapeSettings;
const RVec3 = zolt.RVec3;
const ScaledShapeSettings = zolt.ScaledShapeSettings;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const SphereShapeSettings = zolt.SphereShapeSettings;
const StateRecorderImpl = zolt.StateRecorderImpl;
const StaticCompoundShapeSettings = zolt.StaticCompoundShapeSettings;
const SubShapeID = zolt.SubShapeID;
const UVec4 = zolt.UVec4;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;

const io = std.testing.io;

/// The C++ reference functions, see BodyReference.cpp
const jolt = struct {
    extern fn jolt_body_world_create(max_bodies: u32, num_body_mutexes: u32) *anyopaque;
    extern fn jolt_body_world_destroy(world: *anyopaque) void;
    extern fn jolt_body_world_create_body(world: *anyopaque, desc: *const BodyDesc, custom_id: u32, activate: u32) u32;
    extern fn jolt_body_world_destroy_body(world: *anyopaque, id: u32) void;
    extern fn jolt_body_world_activate(world: *anyopaque, ids: [*]const u32, count: u32, activate: u32) void;
    extern fn jolt_body_world_set_motion_quality(world: *anyopaque, id: u32, quality: u32) void;
    extern fn jolt_body_world_mutex_mask(world: *anyopaque, ids: [*]const u32, count: u32) u64;
    extern fn jolt_body_world_info(world: *anyopaque, out_counts: *[15]u32, out_active: [*]u32, out_body_ids: [*]u32, out_state: [*]u8, state_capacity: u32) void;
    extern fn jolt_body_world_restore(world: *anyopaque, state: [*]const u8, size: u32, validating: u32) u32;
    extern fn jolt_body_get(world: *anyopaque, id: u32, out: *BodyOut) void;
    extern fn jolt_body_op(world: *anyopaque, id: u32, op: *const BodyOp, out: *BodyOpResult) void;
    extern fn jolt_body_bcs_mass_properties(desc: *const BodyDesc, out_mp: *[17]f32) void;
};

/// Must match BodyShapeDesc in BodyReference.cpp
const BodyShapeDesc = extern struct {
    /// 0 sphere, 1 box, 2 capsule, 3 cylinder, 4 offset COM box, 5 rotated translated box, 6 scaled box, 7 static compound of a box and a sphere
    kind: u32 = 0,
    /// 1: BodyCreationSettings(const ShapeSettings *), 0: BodyCreationSettings(const Shape *)
    use_settings: u32 = 1,
    /// 0..2 half extent / radius / half height, 3 convex radius, 4..6 offset / translation / scale
    params: [8]f32 = @splat(0),
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
};

/// Must match BodyDesc in BodyReference.cpp
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
    shape: BodyShapeDesc = .{},
};

/// Must match BodyOp in BodyReference.cpp
const BodyOp = extern struct {
    r: [6]Real = @splat(0),
    f: [24]f32 = @splat(0),
    kind: u32 = 0,
    i: [7]u32 = @splat(0),
    shape: BodyShapeDesc = .{},
};

/// Must match BodyOpResult in BodyReference.cpp
const BodyOpResult = extern struct {
    r: [6]Real = @splat(0),
    f: [48]f32 = @splat(0),
    i: [16]u32 = @splat(0),
};

/// Must match BodyOut in BodyReference.cpp
const BodyOut = extern struct {
    user_data: u64,
    position: [3]Real,
    com_position: [3]Real,
    world_t: [3]Real,
    com_t: [3]Real,
    inv_com_t: [3]Real,
    rotation: [4]f32,
    world_cols: [12]f32,
    com_cols: [12]f32,
    inv_com_cols: [12]f32,
    bounds: [6]f32,
    linear_velocity: [3]f32,
    angular_velocity: [3]f32,
    force: [3]f32,
    torque: [3]f32,
    inv_mass: f32,
    inv_inertia_diagonal: [3]f32,
    inertia_rotation: [4]f32,
    local_inv_inertia: [16]f32,
    inv_inertia: [16]f32,
    max_linear_velocity: f32,
    max_angular_velocity: f32,
    linear_damping: f32,
    angular_damping: f32,
    gravity_factor: f32,
    friction: f32,
    restitution: f32,
    id: u32,
    body_type: u32,
    motion_type: u32,
    flags: u32,
    index_in_active_bodies: u32,
    island_index: u32,
    motion_quality: u32,
    allowed_dofs: u32,
    allow_sleeping: u32,
    num_velocity_steps_override: u32,
    num_position_steps_override: u32,
    object_layer: u32,
    broad_phase_layer: u32,
    has_motion_properties: u32,
    is_active: u32,
    linear_dofs_mask: [4]u32,
    angular_dofs_mask: [4]u32,
    state_size: u32,
    state: [512]u8,
};

/// The operations of jolt_body_op (see the switch in BodyReference.cpp)
const BodyOpKind = enum(u32) {
    set_mass_properties = 0,
    apply_force_torque_and_drag = 1,
    apply_gyroscopic_force = 2,
    set_velocities_and_clamp = 3,
    set_velocities_clamped = 4,
    forces = 5,
    impulses = 6,
    move_kinematic = 7,
    set_position_and_rotation = 8,
    update_sleep_state = 9,
    apply_buoyancy_impulse = 10,
    world_space_surface_normal = 11,
    point_velocity = 12,
    inverse_inertia = 13,
    set_shape = 14,
    position_steps = 15,
    scale_to_mass = 16,
    set_motion_type = 17,
    velocity_steps = 18,
    submerged_volume = 19,
    sleep_timer = 20,
    body_creation_settings = 21,
    flags = 22,
    motion_properties_setters = 23,
    contact_cache = 24,
    motion_properties_move_kinematic = 25,
};

const sentinel: f32 = -12345.0;

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

fn loadMat44(m: []const f32) Mat44 {
    return Mat44.init(load4(m[0..4]), load4(m[4..8]), load4(m[8..12]), load4(m[12..16]));
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

fn storeMat44(m: Mat44, out: []f32) void {
    for (0..4) |c| store4(m.getColumn4(@intCast(c)), out[4 * c ..][0..4]);
}

fn storeRMat44(m: RMat44, out_cols: []f32, out_t: []Real) void {
    for (0..3) |c| store4(m.getColumn4(@intCast(c)), out_cols[4 * c ..][0..4]);
    storeR3(m.getTranslation(), out_t);
}

/// Broad phase layer = object layer % 2 (ParityBPLayerInterface)
const ParityLayers = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    base: BroadPhaseLayerInterface = .init(ParityLayers),

    pub fn getNumBroadPhaseLayers(self: *const ParityLayers) u32 {
        _ = self;
        return 2;
    }

    pub fn getBroadPhaseLayer(self: *const ParityLayers, layer: ObjectLayer) BroadPhaseLayer {
        _ = self;
        return .init(@intCast(layer % 2));
    }
};

/// A BodyManager on both sides
const World = struct {
    allocator: Allocator,
    layers: ParityLayers = .{},
    body_manager: BodyManager = .empty,
    jolt_world: *anyopaque = undefined,

    fn init(self: *World, allocator: Allocator, max_bodies: u32, num_body_mutexes: u32) !void {
        self.* = .{ .allocator = allocator };
        try self.body_manager.init(allocator, io, max_bodies, num_body_mutexes, &self.layers.base);
        self.jolt_world = jolt.jolt_body_world_create(max_bodies, num_body_mutexes);
    }

    fn deinit(self: *World) void {
        jolt.jolt_body_world_destroy(self.jolt_world);
        self.body_manager.deinit();
    }

    /// jolt_body_world_create_body on the Zolt side
    fn createBody(self: *World, desc: *const BodyDesc, custom_id: u32, activate: bool) !u32 {
        var settings = try makeBodyCreationSettings(self.allocator, desc);
        defer settings.deinit();
        const bm = &self.body_manager;
        const body = try bm.allocateBody(&settings);
        const added = if (custom_id == BodyID.invalid_body_id) bm.addBody(body) else bm.addBodyWithCustomID(body, .init(custom_id));
        if (!added) {
            bm.freeBody(body);
            return BodyID.invalid_body_id;
        }
        body.setInBroadPhaseInternal(true);
        const id = body.getID();
        if (activate and !body.isStatic())
            bm.activateBodies(&.{id});
        return id.getIndexAndSequenceNumber();
    }

    /// jolt_body_world_destroy_body on the Zolt side
    fn destroyBody(self: *World, id: u32) void {
        const bm = &self.body_manager;
        const body_id = BodyID.init(id);
        bm.deactivateBodies(&.{body_id});
        bm.getBodyMut(body_id).setInBroadPhaseInternal(false);
        bm.destroyBodies(&.{body_id});
    }
};

fn makeBoxSettings(allocator: Allocator, desc: *const BodyShapeDesc) !*ShapeSettings {
    return (try BoxShapeSettings.create(allocator, load3(desc.params[0..3]), .{ .convex_radius = desc.params[3] })).asShapeSettings();
}

/// MakeShapeSettings of BodyReference.cpp: the returned settings have no reference yet
fn makeShapeSettings(allocator: Allocator, desc: *const BodyShapeDesc) !*ShapeSettings {
    const p = desc.params;
    return switch (desc.kind) {
        0 => (try SphereShapeSettings.create(allocator, p[0], .{})).asShapeSettings(),
        1 => try makeBoxSettings(allocator, desc),
        2 => (try CapsuleShapeSettings.create(allocator, p[0], p[1], .{})).asShapeSettings(),
        3 => (try CylinderShapeSettings.create(allocator, p[0], p[1], .{ .convex_radius = p[3] })).asShapeSettings(),
        4 => (try OffsetCenterOfMassShapeSettings.create(allocator, load3(p[4..7]), try makeBoxSettings(allocator, desc))).asShapeSettings(),
        5 => (try RotatedTranslatedShapeSettings.create(allocator, load3(p[4..7]), loadQuat(&desc.rotation), try makeBoxSettings(allocator, desc))).asShapeSettings(),
        6 => (try ScaledShapeSettings.create(allocator, try makeBoxSettings(allocator, desc), load3(p[4..7]))).asShapeSettings(),
        else => blk: {
            const compound = try StaticCompoundShapeSettings.create(allocator);
            try compound.base.addShape(load3(p[4..7]), loadQuat(&desc.rotation), try makeBoxSettings(allocator, desc), .{});
            try compound.base.addShape(load3(p[4..7]).negate(), Quat.identity(), (try SphereShapeSettings.create(allocator, p[0], .{})).asShapeSettings(), .{});
            break :blk compound.asShapeSettings();
        },
    };
}

/// MakeShape of BodyReference.cpp: the caller owns the returned reference
fn makeShape(allocator: Allocator, desc: *const BodyShapeDesc) !RefConst(Shape) {
    var settings = Ref(ShapeSettings).init(try makeShapeSettings(allocator, desc));
    defer settings.deinit();
    var result = try settings.get().?.createShape(allocator);
    defer result.deinit();
    return RefConst(Shape).init(result.getPtr().?);
}

/// MakeBodyCreationSettings of BodyReference.cpp (the caller deinits the result)
fn makeBodyCreationSettings(allocator: Allocator, desc: *const BodyDesc) !BodyCreationSettings {
    const position = loadR3(&desc.position);
    const rotation = loadQuat(&desc.rotation);
    const motion_type: zolt.MotionType = @enumFromInt(desc.motion_type);
    const layer: ObjectLayer = @intCast(desc.object_layer);
    var settings: BodyCreationSettings = undefined;
    if (desc.shape.use_settings != 0) {
        settings = .initWithSettings(try makeShapeSettings(allocator, &desc.shape), position, rotation, motion_type, layer);
    } else {
        var shape = try makeShape(allocator, &desc.shape);
        defer shape.deinit();
        settings = .initWithShape(shape.get().?, position, rotation, motion_type, layer);
    }
    settings.linear_velocity = load3(&desc.linear_velocity);
    settings.angular_velocity = load3(&desc.angular_velocity);
    settings.user_data = desc.user_data;
    settings.allowed_dofs = .fromInt(@intCast(desc.allowed_dofs));
    settings.allow_dynamic_or_kinematic = desc.allow_dynamic_or_kinematic != 0;
    settings.is_sensor = desc.is_sensor != 0;
    settings.collide_kinematic_vs_non_dynamic = desc.collide_kinematic_vs_non_dynamic != 0;
    settings.use_manifold_reduction = desc.use_manifold_reduction != 0;
    settings.apply_gyroscopic_force = desc.apply_gyroscopic_force != 0;
    settings.motion_quality = @enumFromInt(desc.motion_quality);
    settings.enhanced_internal_edge_removal = desc.enhanced_internal_edge_removal != 0;
    settings.allow_sleeping = desc.allow_sleeping != 0;
    settings.friction = desc.friction;
    settings.restitution = desc.restitution;
    settings.linear_damping = desc.linear_damping;
    settings.angular_damping = desc.angular_damping;
    settings.max_linear_velocity = desc.max_linear_velocity;
    settings.max_angular_velocity = desc.max_angular_velocity;
    settings.gravity_factor = desc.gravity_factor;
    settings.num_velocity_steps_override = desc.num_velocity_steps_override;
    settings.num_position_steps_override = desc.num_position_steps_override;
    settings.override_mass_properties = @enumFromInt(desc.override_mass_properties);
    settings.inertia_multiplier = desc.inertia_multiplier;
    settings.mass_properties_override.mass = desc.mass;
    settings.mass_properties_override.inertia = loadMat44(&desc.inertia);
    return settings;
}

fn saveBytes(recorder: *const StateRecorderImpl, out: []u8) u32 {
    const data = recorder.getData();
    if (data.len <= out.len)
        @memcpy(out[0..data.len], data);
    return @intCast(data.len);
}

/// GetBodyOut of BodyReference.cpp
fn getBodyOut(allocator: Allocator, body: *const Body) !BodyOut {
    var out = std.mem.zeroes(BodyOut);
    out.user_data = body.getUserData();
    storeR3(body.getPosition(), &out.position);
    storeR3(body.getCenterOfMassPosition(), &out.com_position);
    storeRMat44(body.getWorldTransform(), &out.world_cols, &out.world_t);
    storeRMat44(body.getCenterOfMassTransform(), &out.com_cols, &out.com_t);
    storeRMat44(body.getInverseCenterOfMassTransform(), &out.inv_com_cols, &out.inv_com_t);
    storeQuat(body.getRotation(), &out.rotation);
    store3(body.getWorldSpaceBounds().min, out.bounds[0..3]);
    store3(body.getWorldSpaceBounds().max, out.bounds[3..6]);
    store3(body.getLinearVelocity(), &out.linear_velocity);
    store3(body.getAngularVelocity(), &out.angular_velocity);
    if (body.getMotionPropertiesUnchecked()) |mp| {
        store3(mp.getAccumulatedForce(), &out.force);
        store3(mp.getAccumulatedTorque(), &out.torque);
        out.inv_mass = mp.getInverseMassUnchecked();
        store3(mp.inv_inertia_diagonal, &out.inv_inertia_diagonal); // GetInverseInertiaDiagonal without the dynamic assert
        storeQuat(mp.getInertiaRotation(), &out.inertia_rotation);
        storeMat44(mp.getLocalSpaceInverseInertiaUnchecked(), &out.local_inv_inertia);
        if (body.isDynamic())
            storeMat44(body.getInverseInertia(), &out.inv_inertia)
        else
            @memset(&out.inv_inertia, sentinel);
        out.max_linear_velocity = mp.getMaxLinearVelocity();
        out.max_angular_velocity = mp.getMaxAngularVelocity();
        out.linear_damping = mp.getLinearDamping();
        out.angular_damping = mp.getAngularDamping();
        out.gravity_factor = mp.getGravityFactor();
        out.island_index = mp.getIslandIndexInternal();
        out.motion_quality = @intFromEnum(mp.getMotionQuality());
        out.allowed_dofs = mp.getAllowedDOFs().toInt();
        out.allow_sleeping = @intFromBool(mp.getAllowSleeping());
        out.num_velocity_steps_override = mp.getNumVelocityStepsOverride();
        out.num_position_steps_override = mp.getNumPositionStepsOverride();
        out.has_motion_properties = 1;
        mp.getLinearDOFsMask().storeInt4(&out.linear_dofs_mask);
        mp.getAngularDOFsMask().storeInt4(&out.angular_dofs_mask);
    } else {
        @memset(&out.force, sentinel);
        @memset(&out.torque, sentinel);
        out.inv_mass = sentinel;
        @memset(&out.inv_inertia_diagonal, sentinel);
        @memset(&out.inertia_rotation, sentinel);
        @memset(&out.local_inv_inertia, sentinel);
        @memset(&out.inv_inertia, sentinel);
    }
    out.friction = body.getFriction();
    out.restitution = body.getRestitution();
    out.id = body.getID().getIndexAndSequenceNumber();
    out.body_type = @intFromEnum(body.getBodyType());
    out.motion_type = @intFromEnum(body.getMotionType());
    out.flags = @as(u32, @intFromBool(body.isSensor())) |
        (@as(u32, @intFromBool(body.getCollideKinematicVsNonDynamic())) << 1) |
        (@as(u32, @intFromBool(body.isInBroadPhase())) << 2) |
        (@as(u32, @intFromBool(body.isCollisionCacheInvalid())) << 3) |
        (@as(u32, @intFromBool(body.getUseManifoldReduction())) << 4) |
        (@as(u32, @intFromBool(body.getApplyGyroscopicForce())) << 5) |
        (@as(u32, @intFromBool(body.getEnhancedInternalEdgeRemoval())) << 6) |
        (@as(u32, @intFromBool(body.canBeKinematicOrDynamic())) << 7);
    out.index_in_active_bodies = body.getIndexInActiveBodiesInternal();
    out.object_layer = body.getObjectLayer();
    out.broad_phase_layer = body.getBroadPhaseLayer().getValue();
    out.is_active = @intFromBool(body.isActive());

    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    body.saveState(recorder.recorder());
    out.state_size = saveBytes(&recorder, &out.state);
    return out;
}

/// jolt_body_op on the Zolt side
fn applyOp(world: *World, id: u32, op: *const BodyOp) !BodyOpResult {
    const allocator = world.allocator;
    const bm = &world.body_manager;
    const body = bm.getBodyMut(.init(id));
    const mp_opt = body.getMotionPropertiesUncheckedMut();
    const f = &op.f;
    const r = &op.r;
    const i = &op.i;
    var result: BodyOpResult = .{};
    const of = &result.f;

    switch (@as(BodyOpKind, @enumFromInt(op.kind))) {
        .set_mass_properties => {
            var mass_properties: MassProperties = .{};
            mass_properties.mass = f[0];
            mass_properties.inertia = loadMat44(f[4..20]);
            mp_opt.?.setMassProperties(.fromInt(@intCast(i[0])), &mass_properties);
        },
        .apply_force_torque_and_drag => mp_opt.?.applyForceTorqueAndDragInternal(body.getRotation(), load3(f[0..3]), f[3]),
        .apply_gyroscopic_force => mp_opt.?.applyGyroscopicForceInternal(body.getRotation(), f[0]),
        .set_velocities_and_clamp => {
            // SetLinearVelocity / SetAngularVelocity without their debug asserts on the maximum (the C++ reference has no asserts)
            const mp = mp_opt.?;
            mp.linear_velocity = mp.lockTranslation(load3(f[0..3]));
            mp.angular_velocity = mp.lockAngular(load3(f[3..6]));
            mp.clampLinearVelocity();
            mp.clampAngularVelocity();
        },
        .set_velocities_clamped => {
            body.setLinearVelocityClamped(load3(f[0..3]));
            body.setAngularVelocityClamped(load3(f[3..6]));
        },
        .forces => {
            body.addForce(load3(f[0..3]));
            body.addForceAtPosition(load3(f[3..6]), loadR3(r[0..3]));
            body.addTorque(load3(f[6..9]));
        },
        .impulses => {
            body.addImpulse(load3(f[0..3]));
            body.addImpulseAtPosition(load3(f[3..6]), loadR3(r[0..3]));
            body.addAngularImpulse(load3(f[6..9]));
        },
        .move_kinematic => body.moveKinematic(loadR3(r[0..3]), loadQuat(f[0..4]), f[4]),
        .set_position_and_rotation => body.setPositionAndRotationInternal(loadR3(r[0..3]), loadQuat(f[0..4]), .{ .reset_sleep_timer = i[0] != 0 }),
        .update_sleep_state => result.i[0] = @intFromEnum(body.updateSleepStateInternal(f[0], f[1], f[2])),
        .apply_buoyancy_impulse => result.i[0] = @intFromBool(body.applyBuoyancyImpulse(loadR3(r[0..3]), load3(f[0..3]), f[3], f[4], f[5], load3(f[6..9]), load3(f[9..12]), f[12])),
        .world_space_surface_normal => store3(body.getWorldSpaceSurfaceNormal(.{ .value = i[0] }, loadR3(r[0..3])), of[0..3]),
        .point_velocity => {
            store3(body.getPointVelocity(loadR3(r[0..3])), of[0..3]);
            store3(body.getPointVelocityCOM(load3(f[0..3])), of[3..6]);
        },
        .inverse_inertia => {
            const mp = mp_opt.?;
            storeMat44(mp.getInverseInertiaForRotation(Mat44.rotationQuat(loadQuat(f[0..4]))), of[0..16]);
            store3(mp.multiplyWorldSpaceInverseInertiaByVector(loadQuat(f[4..8]), load3(f[8..11])), of[16..19]);
        },
        .set_shape => {
            var shape = try makeShape(allocator, &op.shape);
            defer shape.deinit();
            body.setShapeInternal(shape.get().?, i[0] != 0);
        },
        .position_steps => {
            body.addPositionStep(load3(f[0..3]));
            body.addRotationStep(load3(f[3..6]));
            body.subPositionStep(load3(f[6..9]));
            body.subRotationStep(load3(f[9..12]));
        },
        .scale_to_mass => mp_opt.?.scaleToMass(f[0]),
        .set_motion_type => body.setMotionType(@enumFromInt(i[0])),
        .velocity_steps => {
            const mp = mp_opt.?;
            mp.addLinearVelocityStep(load3(f[0..3]));
            mp.subLinearVelocityStep(load3(f[3..6]));
            mp.addAngularVelocityStep(load3(f[6..9]));
            mp.subAngularVelocityStep(load3(f[9..12]));
        },
        .submerged_volume => {
            const volume = body.getSubmergedVolume(loadR3(r[0..3]), load3(f[0..3]));
            of[0] = volume.total_volume;
            of[1] = volume.submerged_volume;
            store3(volume.relative_center_of_buoyancy, of[2..5]);
        },
        .sleep_timer => {
            body.resetSleepTimer();
            result.i[0] = @intFromEnum(mp_opt.?.accumulateSleepTime(f[0], f[1]));
        },
        .body_creation_settings => {
            var s = body.getBodyCreationSettings();
            defer s.deinit();
            storeR3(s.position, result.r[0..3]);
            storeQuat(s.rotation, of[0..4]);
            store3(s.linear_velocity, of[4..7]);
            store3(s.angular_velocity, of[7..10]);
            of[10] = s.friction;
            of[11] = s.restitution;
            of[12] = s.linear_damping;
            of[13] = s.angular_damping;
            of[14] = s.max_linear_velocity;
            of[15] = s.max_angular_velocity;
            of[16] = s.gravity_factor;
            of[17] = s.inertia_multiplier;
            of[18] = s.mass_properties_override.mass;
            storeMat44(s.mass_properties_override.inertia, of[19..35]);
            const oi = &result.i;
            oi[0] = @truncate(s.user_data);
            oi[1] = s.object_layer;
            oi[2] = @intFromEnum(s.motion_type);
            oi[3] = s.allowed_dofs.toInt();
            oi[4] = @as(u32, @intFromBool(s.allow_dynamic_or_kinematic)) | (@as(u32, @intFromBool(s.is_sensor)) << 1) | (@as(u32, @intFromBool(s.collide_kinematic_vs_non_dynamic)) << 2) | (@as(u32, @intFromBool(s.use_manifold_reduction)) << 3) |
                (@as(u32, @intFromBool(s.apply_gyroscopic_force)) << 4) | (@as(u32, @intFromBool(s.enhanced_internal_edge_removal)) << 5) | (@as(u32, @intFromBool(s.allow_sleeping)) << 6) | (@as(u32, @intFromBool((try s.getShape(allocator)) == body.getShape())) << 7);
            oi[5] = @intFromEnum(s.motion_quality);
            oi[6] = s.num_velocity_steps_override;
            oi[7] = s.num_position_steps_override;
            oi[8] = @intFromEnum(s.override_mass_properties);
            oi[9] = s.collision_group.getGroupID();
            oi[10] = s.collision_group.getSubGroupID();
        },
        .flags => {
            body.setIsSensor(i[0] != 0);
            body.setCollideKinematicVsNonDynamic(i[1] != 0);
            body.setUseManifoldReduction(i[2] != 0);
            body.setApplyGyroscopicForce(i[3] != 0);
            body.setEnhancedInternalEdgeRemoval(i[4] != 0);
            body.setFriction(f[0]);
            body.setRestitution(f[1]);
            if (mp_opt != null)
                body.setAllowSleeping(i[5] != 0);
        },
        .motion_properties_setters => {
            const mp = mp_opt.?;
            mp.setMaxLinearVelocity(f[0]);
            mp.setMaxAngularVelocity(f[1]);
            mp.setLinearDamping(f[2]);
            mp.setAngularDamping(f[3]);
            mp.setGravityFactor(f[4]);
            mp.setNumVelocityStepsOverride(i[0]);
            mp.setNumPositionStepsOverride(i[1]);
        },
        .contact_cache => {
            try bm.invalidateContactCacheForBody(body);
            if (i[0] != 0)
                bm.validateContactCacheForAllBodies();
        },
        .motion_properties_move_kinematic => mp_opt.?.moveKinematic(load3(f[0..3]), loadQuat(f[4..8]), f[8]),
    }
    return result;
}

/// Input generator
const Gen = struct {
    rng: fw.Rng = .{},

    fn index(self: *Gen, n: usize) usize {
        return self.rng.next() % n;
    }

    fn oneIn(self: *Gen, n: u32) bool {
        return self.rng.next() % n == 0;
    }

    fn float(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn vec(self: *Gen, scale: f32) [3]f32 {
        return .{ self.float(-scale, scale), self.float(-scale, scale), self.float(-scale, scale) };
    }

    /// A random vector with special values mixed in (zero, axis aligned)
    fn vecSpecial(self: *Gen, scale: f32) [3]f32 {
        return switch (self.index(8)) {
            0 => .{ 0, 0, 0 },
            1 => .{ scale, 0, 0 },
            2 => .{ 0, -scale, 0 },
            else => self.vec(scale),
        };
    }

    fn quat(self: *Gen) [4]f32 {
        if (self.oneIn(6)) return .{ 0, 0, 0, 1 };
        const q = Quat.fromVec4(Vec4.init(self.float(-1, 1), self.float(-1, 1), self.float(-1, 1), self.float(-1, 1)).normalized());
        var out: [4]f32 = undefined;
        storeQuat(q, &out);
        return out;
    }

    fn position(self: *Gen) [3]Real {
        const far: Real = if (Core.double_precision and self.oneIn(4)) 1.0e7 else 0.0;
        return .{ far + self.float(-100, 100), self.float(-100, 100), far + self.float(-100, 100) };
    }

    /// A symmetric positive definite inertia tensor R diag(d) R^T
    fn inertia(self: *Gen, out: *[16]f32) void {
        const rotation = Mat44.rotationQuat(loadQuat(&self.quat()));
        const diagonal = Vec3.init(self.float(0.2, 5), self.float(0.2, 5), self.float(0.2, 5));
        storeMat44(rotation.mul(Mat44.scaleVec3(diagonal)).mul(rotation.transposed()), out);
    }

    fn shape(self: *Gen) BodyShapeDesc {
        return self.shapeOfKind(@intCast(self.index(8)));
    }

    fn shapeOfKind(self: *Gen, kind: u32) BodyShapeDesc {
        var desc: BodyShapeDesc = .{ .kind = kind, .use_settings = @intFromBool(!self.oneIn(3)) };
        desc.params[0] = self.float(0.3, 2);
        desc.params[1] = self.float(0.3, 2);
        desc.params[2] = self.float(0.3, 2);
        desc.params[3] = self.float(0, 0.1);
        switch (desc.kind) {
            4, 5, 7 => {
                const offset = self.vec(1);
                @memcpy(desc.params[4..7], &offset);
            },
            6 => {
                desc.params[4] = self.float(0.5, 2);
                desc.params[5] = if (self.oneIn(3)) desc.params[4] else self.float(0.5, 2);
                desc.params[6] = if (self.oneIn(3)) desc.params[4] else self.float(0.5, 2);
            },
            else => {},
        }
        desc.rotation = self.quat();
        return desc;
    }

    fn bodyDesc(self: *Gen) BodyDesc {
        var desc: BodyDesc = .{};
        desc.user_data = (@as(u64, self.rng.next()) << 32) | self.rng.next();
        desc.position = self.position();
        desc.rotation = self.quat();
        desc.motion_type = @intCast(self.index(3));
        desc.allowed_dofs = if (self.oneIn(2)) 0b111111 else @intCast(1 + self.index(63));
        desc.allow_dynamic_or_kinematic = @intFromBool(self.oneIn(2));
        desc.is_sensor = @intFromBool(self.oneIn(5));
        desc.collide_kinematic_vs_non_dynamic = @intFromBool(self.oneIn(4));
        desc.use_manifold_reduction = @intFromBool(!self.oneIn(4));
        desc.apply_gyroscopic_force = @intFromBool(self.oneIn(3));
        desc.motion_quality = @intCast(self.index(2));
        desc.enhanced_internal_edge_removal = @intFromBool(self.oneIn(4));
        desc.allow_sleeping = @intFromBool(!self.oneIn(4));
        desc.friction = self.float(0, 1);
        desc.restitution = self.float(0, 1);
        desc.linear_damping = self.float(0, 0.5);
        desc.angular_damping = self.float(0, 0.5);
        // Very large velocities with a matching maximum, otherwise Jolt's defaults
        const large = self.oneIn(5);
        desc.max_linear_velocity = if (large) 1.0e6 else self.float(10, 600);
        desc.max_angular_velocity = if (large) 1.0e5 else self.float(10, 60);
        const linear_scale = 0.5 * desc.max_linear_velocity / @sqrt(3.0);
        const angular_scale = 0.5 * desc.max_angular_velocity / @sqrt(3.0);
        desc.linear_velocity = self.vecSpecial(linear_scale);
        desc.angular_velocity = self.vecSpecial(angular_scale);
        desc.gravity_factor = self.float(-1, 2);
        desc.num_velocity_steps_override = @intCast(self.index(10));
        desc.num_position_steps_override = @intCast(self.index(10));
        desc.override_mass_properties = @intCast(self.index(3));
        desc.inertia_multiplier = self.float(0.5, 3);
        desc.mass = self.float(0.5, 100);
        self.inertia(&desc.inertia);
        desc.object_layer = @intCast(self.index(4));
        desc.shape = self.shape();
        return desc;
    }

    /// A random operation that is valid for the body (Zolt's asserts are on in Debug, Jolt's reference has none)
    fn op(self: *Gen, body: *const Body) BodyOp {
        const has_mp = body.getMotionPropertiesUnchecked() != null;
        const dynamic = body.isDynamic();
        const translation_allowed = has_mp and body.getMotionPropertiesUnchecked().?.inv_mass > 0.0;
        while (true) {
            var o: BodyOp = .{ .kind = @intCast(self.index(26)) };
            for (&o.r) |*x| x.* = @floatCast(self.float(-100, 100));
            const com = body.getCenterOfMassPosition();
            // Positions near the body
            o.r[0] = com.getX() + o.r[0] * 0.02;
            o.r[1] = com.getY() + o.r[1] * 0.02;
            o.r[2] = com.getZ() + o.r[2] * 0.02;
            for (&o.f) |*x| x.* = self.float(-10, 10);
            for (&o.i) |*x| x.* = self.rng.next();
            switch (@as(BodyOpKind, @enumFromInt(o.kind))) {
                .set_mass_properties => {
                    if (!has_mp) continue;
                    o.i[0] = if (self.oneIn(3)) 0b111111 else @intCast(1 + self.index(63));
                    o.f[0] = self.float(0.1, 100);
                    var inertia_tensor: [16]f32 = undefined;
                    self.inertia(&inertia_tensor);
                    if (self.oneIn(10) and (o.i[0] & 0b111) != 0) inertia_tensor = @splat(0); // Cannot decompose: sphere fallback
                    @memcpy(o.f[4..20], &inertia_tensor);
                },
                .apply_force_torque_and_drag => {
                    if (!dynamic) continue;
                    o.f[3] = self.float(0.001, 0.1);
                },
                .apply_gyroscopic_force => {
                    if (!dynamic) continue;
                    o.f[0] = self.float(0.001, 0.1);
                },
                .set_velocities_and_clamp, .set_velocities_clamped => {
                    if (!has_mp) continue;
                    if (o.kind == @intFromEnum(BodyOpKind.set_velocities_clamped) and body.isStatic()) continue;
                    const mp = body.getMotionPropertiesUnchecked().?;
                    const v = self.vecSpecial(2.0 * mp.max_linear_velocity);
                    const w = self.vecSpecial(2.0 * mp.max_angular_velocity);
                    @memcpy(o.f[0..3], &v);
                    @memcpy(o.f[3..6], &w);
                },
                .forces, .impulses, .inverse_inertia, .velocity_steps => {
                    if (!dynamic) continue;
                    if (o.kind == @intFromEnum(BodyOpKind.inverse_inertia)) {
                        @memcpy(o.f[0..4], &self.quat());
                        @memcpy(o.f[4..8], &self.quat());
                    }
                },
                .move_kinematic => {
                    if (body.isStatic()) continue;
                    @memcpy(o.f[0..4], &self.quat());
                    o.f[4] = self.float(0.01, 1);
                },
                .set_position_and_rotation => {
                    const p = self.position();
                    @memcpy(o.r[0..3], &p);
                    @memcpy(o.f[0..4], &self.quat());
                    o.i[0] &= 1;
                },
                .update_sleep_state => {
                    if (!has_mp) continue;
                    o.f[0] = self.float(0.001, 0.5);
                    o.f[1] = self.float(0, 0.2);
                    o.f[2] = self.float(0, 1);
                },
                .apply_buoyancy_impulse => {
                    if (!dynamic or !translation_allowed) continue;
                    const n = Vec3.init(self.float(-0.3, 0.3), 1, self.float(-0.3, 0.3)).normalized();
                    store3(n, o.f[0..3]);
                    o.f[3] = self.float(0, 2);
                    o.f[4] = self.float(0, 1);
                    o.f[5] = self.float(0, 0.1);
                    o.f[9] = 0;
                    o.f[10] = -9.81;
                    o.f[11] = 0;
                    o.f[12] = self.float(0.001, 0.1);
                },
                .world_space_surface_normal => o.i[0] = SubShapeID.empty.getValue(),
                .point_velocity => {},
                .set_shape => {
                    if (!body.isRigidBody()) continue;
                    o.shape = self.shape();
                    o.i[0] &= 1;
                },
                .position_steps => {
                    if (!has_mp) continue;
                    for (o.f[0..12]) |*x| x.* *= 0.05;
                    if (self.oneIn(5)) @memset(o.f[3..6], 1.0e-7); // Below the rotation threshold
                },
                .scale_to_mass => {
                    if (!translation_allowed) continue;
                    o.f[0] = self.float(0.1, 100);
                },
                .set_motion_type => {
                    if (has_mp) {
                        o.i[0] = @intCast(self.index(3));
                        if (o.i[0] == 0 and body.isActive()) o.i[0] = 2;
                    } else o.i[0] = 0;
                },
                .submerged_volume => {
                    const n = Vec3.init(self.float(-1, 1), self.float(-1, 1), self.float(-1, 1)).normalizedOr(Vec3.axisY());
                    store3(n, o.f[0..3]);
                },
                .sleep_timer => {
                    if (!has_mp) continue;
                    o.f[0] = self.float(0, 1);
                    o.f[1] = self.float(0, 1);
                },
                .body_creation_settings => {},
                .flags => {
                    for (o.i[0..6]) |*x| x.* &= 1;
                    o.f[0] = self.float(0, 1);
                    o.f[1] = self.float(0, 1);
                },
                .motion_properties_setters => {
                    if (!has_mp) continue;
                    o.f[0] = self.float(1, 1000);
                    o.f[1] = self.float(1, 100);
                    o.f[2] = self.float(0, 1);
                    o.f[3] = self.float(0, 1);
                    o.f[4] = self.float(-2, 2);
                    o.i[0] = @intCast(self.index(256));
                    o.i[1] = @intCast(self.index(256));
                },
                .contact_cache => o.i[0] &= 1,
                .motion_properties_move_kinematic => {
                    if (!has_mp or body.isStatic()) continue;
                    @memcpy(o.f[4..8], &self.quat());
                    o.f[8] = self.float(0.01, 1);
                },
            }
            return o;
        }
    }
};

test "BodyCreationSettings::GetMassProperties for every mode and shape" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{};
    var check: Checker = .{ .name = "GetMassProperties" };
    for (0..2000) |n| {
        var desc = gen.bodyDesc();
        desc.override_mass_properties = @intCast(n % 3);
        desc.shape = gen.shapeOfKind(@intCast((n / 3) % 8));
        var settings = try makeBodyCreationSettings(allocator, &desc);
        defer settings.deinit();
        const mp = try settings.getMassProperties(allocator);
        var zolt_out: [17]f32 = undefined;
        zolt_out[0] = mp.mass;
        storeMat44(mp.inertia, zolt_out[1..17]);
        var jolt_out: [17]f32 = undefined;
        jolt.jolt_body_bcs_mass_properties(&desc, &jolt_out);
        check.check(desc, zolt_out, jolt_out);
    }
    try check.finish();
}

fn checkBody(world: *World, check: *Checker, id: u32, input: anytype) !void {
    const zolt_out = try getBodyOut(world.allocator, world.body_manager.getBody(.init(id)));
    var jolt_out: BodyOut = undefined;
    jolt.jolt_body_get(world.jolt_world, id, &jolt_out);
    check.check(input, zolt_out, jolt_out);
}

test "Body and MotionProperties operations" {
    const allocator = std.testing.allocator;
    var world: World = undefined;
    try world.init(allocator, 64, 8);
    defer world.deinit();

    var gen: Gen = .{ .rng = .{ .state = 0x9e3779b9 } };
    var check_create: Checker = .{ .name = "Body creation" };
    var check_ops: Checker = .{ .name = "Body operation" };
    var check_results: Checker = .{ .name = "Body operation result" };
    var op_counts: [26]u32 = @splat(0);

    var live: std.ArrayList(u32) = .empty;
    defer live.deinit(allocator);
    for (0..3000) |n| {
        // Create a body
        const desc = gen.bodyDesc();
        const activate = gen.oneIn(2);
        const zolt_id = try world.createBody(&desc, BodyID.invalid_body_id, activate);
        const jolt_id = jolt.jolt_body_world_create_body(world.jolt_world, &desc, BodyID.invalid_body_id, @intFromBool(activate));
        check_create.check(desc, zolt_id, jolt_id);
        if (zolt_id != jolt_id or zolt_id == BodyID.invalid_body_id) break;
        try checkBody(&world, &check_create, zolt_id, desc);
        try live.append(allocator, zolt_id);

        // Apply random operations, compare after each one
        for (0..10) |_| {
            const body = world.body_manager.getBody(.init(zolt_id));
            const o = gen.op(body);
            op_counts[o.kind] += 1;
            const zolt_result = try applyOp(&world, zolt_id, &o);
            var jolt_result: BodyOpResult = .{};
            jolt.jolt_body_op(world.jolt_world, jolt_id, &o, &jolt_result);
            check_results.check(.{ n, o.kind }, zolt_result, jolt_result);
            try checkBody(&world, &check_ops, zolt_id, .{ n, o.kind });
        }

        // Keep up to 32 bodies alive (the active body indices of the others change when a body is destroyed)
        if (live.items.len >= 32) {
            const victim = live.swapRemove(gen.index(live.items.len));
            world.destroyBody(victim);
            jolt.jolt_body_world_destroy_body(world.jolt_world, victim);
            for (live.items) |other| try checkBody(&world, &check_ops, other, .{ n, "destroy" });
        }
    }
    for (op_counts) |c| try std.testing.expect(c > 100); // Every operation was tested
    try finishAll(&.{ &check_create, &check_ops, &check_results });
}

/// The world state that is compared after every step of the sequence
const WorldInfo = struct {
    counts: [15]u32,
    active: [max_sequence_bodies]u32,
    body_ids: [max_sequence_bodies]u32,
    state: [state_capacity]u8,
};

const max_sequence_bodies = 24;
const state_capacity = 8192;

fn zoltWorldInfo(world: *World) !WorldInfo {
    const allocator = world.allocator;
    const bm = &world.body_manager;
    var info = std.mem.zeroes(WorldInfo);
    info.counts[0] = bm.getNumBodies();
    info.counts[1] = bm.getMaxBodies();
    const num_active = bm.getNumActiveBodies(.rigid_body);
    info.counts[2] = num_active;
    for (bm.getActiveBodiesUnsafe(.rigid_body)[0..num_active], 0..) |id, k| info.active[k] = id.getIndexAndSequenceNumber();
    info.counts[3] = bm.getNumActiveCCDBodies();
    var ids: BodyIDVector = .empty;
    defer ids.deinit(allocator);
    try bm.getBodyIDs(allocator, &ids);
    info.counts[4] = @intCast(ids.items.len);
    for (ids.items, 0..) |id, k| info.body_ids[k] = id.getIndexAndSequenceNumber();
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    try bm.saveState(recorder.recorder(), null);
    info.counts[5] = saveBytes(&recorder, &info.state);
    const stats = bm.getBodyStats();
    info.counts[6] = stats.num_bodies;
    info.counts[7] = stats.max_bodies;
    info.counts[8] = stats.num_bodies_static;
    info.counts[9] = stats.num_bodies_dynamic;
    info.counts[10] = stats.num_active_bodies_dynamic;
    info.counts[11] = stats.num_bodies_kinematic;
    info.counts[12] = stats.num_active_bodies_kinematic;
    info.counts[13] = stats.num_soft_bodies;
    info.counts[14] = stats.num_active_soft_bodies;
    return info;
}

fn joltWorldInfo(world: *World) WorldInfo {
    var info = std.mem.zeroes(WorldInfo);
    jolt.jolt_body_world_info(world.jolt_world, &info.counts, &info.active, &info.body_ids, &info.state, state_capacity);
    return info;
}

test "BodyManager sequence: IDs, active body order, mutex masks and state" {
    const allocator = std.testing.allocator;
    var gen: Gen = .{ .rng = .{ .state = 0x51ed2701 } };
    var check: Checker = .{ .name = "BodyManager sequence" };
    var check_ids: Checker = .{ .name = "BodyManager IDs" };
    var check_masks: Checker = .{ .name = "BodyManager mutex mask" };
    var num_out_of_bodies: u32 = 0;
    var num_custom: u32 = 0;
    var num_restores: u32 = 0;

    for (0..20) |round| {
        var world: World = undefined;
        try world.init(allocator, max_sequence_bodies, @as(u32, 1) << @intCast(round % 7));
        defer world.deinit();

        var live: std.ArrayList(u32) = .empty;
        defer live.deinit(allocator);
        for (0..300) |step| {
            const action = gen.index(10);
            switch (action) {
                0, 1, 2 => {
                    // Create, with AddBody or with a custom ID (that may be in use, freed or beyond the end)
                    const desc = gen.bodyDesc();
                    const custom = gen.oneIn(4);
                    const custom_id: u32 = if (custom) BodyID.fromIndexAndSequenceNumber(@intCast(gen.index(max_sequence_bodies + 2)), @truncate(gen.rng.next())).getIndexAndSequenceNumber() else BodyID.invalid_body_id;
                    const activate = !gen.oneIn(3);
                    const zolt_id = try world.createBody(&desc, custom_id, activate);
                    const jolt_id = jolt.jolt_body_world_create_body(world.jolt_world, &desc, custom_id, @intFromBool(activate));
                    check_ids.check(.{ round, step, custom_id }, zolt_id, jolt_id);
                    if (zolt_id != BodyID.invalid_body_id) {
                        try live.append(allocator, zolt_id);
                        if (custom) num_custom += 1;
                    } else num_out_of_bodies += 1;
                },
                3 => if (live.items.len > 0) {
                    const victim = live.swapRemove(gen.index(live.items.len));
                    world.destroyBody(victim);
                    jolt.jolt_body_world_destroy_body(world.jolt_world, victim);
                },
                4, 5 => if (live.items.len > 0) {
                    // Activate or deactivate a few bodies (with invalid IDs and duplicates)
                    var ids: [6]u32 = undefined;
                    for (&ids) |*id| id.* = if (gen.oneIn(6)) BodyID.invalid_body_id else live.items[gen.index(live.items.len)];
                    const activate = action == 4;
                    var body_ids: [6]BodyID = undefined;
                    for (ids, &body_ids) |id, *body_id| body_id.* = .init(id);
                    if (activate) world.body_manager.activateBodies(&body_ids) else world.body_manager.deactivateBodies(&body_ids);
                    jolt.jolt_body_world_activate(world.jolt_world, &ids, ids.len, @intFromBool(activate));
                },
                6 => if (live.items.len > 0) {
                    const id = live.items[gen.index(live.items.len)];
                    const quality: u32 = @intCast(gen.index(2));
                    world.body_manager.setMotionQuality(world.body_manager.getBodyMut(.init(id)), @enumFromInt(quality));
                    jolt.jolt_body_world_set_motion_quality(world.jolt_world, id, quality);
                },
                7 => {
                    // Mutex mask of a few bodies
                    var ids: [8]u32 = undefined;
                    const count = 1 + gen.index(ids.len);
                    for (ids[0..count]) |*id| id.* = if (gen.oneIn(5)) BodyID.invalid_body_id else @intCast(gen.index(max_sequence_bodies));
                    var body_ids: [8]BodyID = undefined;
                    for (ids[0..count], body_ids[0..count]) |id, *body_id| body_id.* = .init(id);
                    check_masks.check(.{ round, step }, world.body_manager.getMutexMask(body_ids[0..count]), jolt.jolt_body_world_mutex_mask(world.jolt_world, &ids, @intCast(count)));
                },
                8 => {
                    // Save, change velocities and activation, restore
                    const zolt_saved = try zoltWorldInfo(&world);
                    const jolt_saved = joltWorldInfo(&world);
                    for (live.items) |id| {
                        const body = world.body_manager.getBodyMut(.init(id));
                        if (body.getMotionPropertiesUncheckedMut()) |mp| {
                            if (!body.isStatic()) {
                                const v = gen.vec(1);
                                // The same change on both sides through the operations (SetLinearVelocityClamped)
                                var o: BodyOp = .{ .kind = @intFromEnum(BodyOpKind.set_velocities_clamped) };
                                @memcpy(o.f[0..3], &v);
                                _ = try applyOp(&world, id, &o);
                                var jolt_result: BodyOpResult = .{};
                                jolt.jolt_body_op(world.jolt_world, id, &o, &jolt_result);
                                _ = mp;
                            }
                        }
                    }
                    if (live.items.len > 0) {
                        const id = live.items[gen.index(live.items.len)];
                        const activate = gen.oneIn(2);
                        const body_id = BodyID.init(id);
                        if (activate) world.body_manager.activateBodies(&.{body_id}) else world.body_manager.deactivateBodies(&.{body_id});
                        jolt.jolt_body_world_activate(world.jolt_world, &.{id}, 1, @intFromBool(activate));
                    }
                    var recorder = StateRecorderImpl.init(allocator);
                    defer recorder.deinit();
                    recorder.recorder().writeBytes(zolt_saved.state[0..zolt_saved.counts[5]]);
                    const zolt_ok = try world.body_manager.restoreState(recorder.recorder());
                    const jolt_ok = jolt.jolt_body_world_restore(world.jolt_world, &jolt_saved.state, jolt_saved.counts[5], 0);
                    check.check(.{ round, step, "restore" }, @as(u32, @intFromBool(zolt_ok)), jolt_ok);
                    num_restores += 1;
                },
                else => {
                    // A validating restore of the current state changes nothing
                    const zolt_now = try zoltWorldInfo(&world);
                    const jolt_now = joltWorldInfo(&world);
                    var recorder = StateRecorderImpl.init(allocator);
                    defer recorder.deinit();
                    recorder.recorder().writeBytes(zolt_now.state[0..zolt_now.counts[5]]);
                    recorder.recorder().setValidating(true);
                    const zolt_ok = try world.body_manager.restoreState(recorder.recorder());
                    const jolt_ok = jolt.jolt_body_world_restore(world.jolt_world, &jolt_now.state, jolt_now.counts[5], 1);
                    check.check(.{ round, step, "validate" }, @as(u32, @intFromBool(zolt_ok)), jolt_ok);
                    if (recorder.num_mismatches != 0) return error.TestUnexpectedResult;
                },
            }
            check.check(.{ round, step, action }, try zoltWorldInfo(&world), joltWorldInfo(&world));
        }
    }
    try std.testing.expect(num_out_of_bodies > 0 and num_custom > 0 and num_restores > 0);
    try finishAll(&.{ &check, &check_ids, &check_masks });
}
