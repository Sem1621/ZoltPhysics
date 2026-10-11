//! SimScenes: the scene descriptions of the simulation parity tests (SimulationArchitecture D12), shared by the Zolt
//! side (SimulationParity.zig) and the C++ side (SimulationReference.cpp). Not a test file: SimulationParity.zig imports
//! it.
//!
//! A scene is data: a list of shapes, body creation settings, step listeners, constraints and physics settings, plus a
//! list of operations (`SimOp`) that are applied between the updates. Both sides build the PhysicsSystem from the same
//! `SimSceneDesc` (extern structs passed through the C ABI, booleans as u32, never bool) and get the same operations in
//! the same order, so the bodies get the same IDs and every update must produce the same `SaveState` bytes.
//!
//! How to add a scene:
//! 1. Write a builder function `fn buildFoo(b: *SceneBuilder) Allocator.Error!void` below. It sets the system
//!    parameters (`b.max_bodies`, `b.gravity`, `b.settings`, `b.delta_time`, `b.collision_steps`, `b.num_updates`),
//!    adds shapes (`b.sphere`, `b.box`, ..., decorated shapes and compounds refer to earlier shapes by index), body
//!    descriptions (`b.addBodyDesc(.{ ... })`, a `SimBody` is Jolt's BodyCreationSettings) and operations: `b.at(u)`
//!    selects the update before which the following operations run, `b.createBody(desc, .activate)` returns a body
//!    handle (the index of the created body in creation order), `b.op(.add_force, handle, .{ .f = ... })` adds any
//!    other operation (see `OpKind` for the fields each kind reads).
//! 2. Add one entry to `scenes` with its name and builder. A scene that Zolt cannot run yet gets
//!    `.blocked_by = .contacts` (or `.constraints`, `.ccd_hits`, ...): SimulationParity runs it as soon as
//!    `Blocker.isSupported` says so (flip it when the wave lands) and skips it until then. Skipped scenes are never
//!    silent: every blocked group is its own test that returns error.SkipZigTest, so the build summary counts them as
//!    "skipped".
//! 3. A scene with contacts needs nothing else (the C++ side and the Zolt side create every shape kind of `ShapeKind`,
//!    and the contact listener records are compared already). A new shape kind is a new `ShapeKind` value plus its
//!    creation in `SimulationReference.cpp` (MakeShape) and `SimulationParity.zig` (makeShapeSettings). Constraints
//!    are described by `SimConstraint`; the C++ side creates every `ConstraintKind`, the Zolt side gets the
//!    corresponding prong in SimulationParity.zig's createConstraint when Phase 6 ports the class.
//!
//! Rules for the gravity-only scenes (until A4 / Wave B): no two bodies whose layers collide may ever have
//! overlapping bounds (the broad phase would report a pair and ProcessBodyPair is REGION R2), and the linear cast
//! bodies must not find a body in their path (REGION R4). Bodies in the `Layers.unused*` layers collide with nothing,
//! the other layers are spread out on a grid (`gridPosition`).

const std = @import("std");
const zolt = @import("zolt");

const Allocator = std.mem.Allocator;
const Real = zolt.Real;

// -------------------------------------------------------------------------------------------------------------------
// The C ABI (must match the structs in SimulationReference.cpp, checked by the test "SimScenes struct layout")

/// Shape kinds and the meaning of `SimShape.params`
pub const ShapeKind = enum(u32) {
    /// params[0] radius
    sphere = 0,
    /// params[0..3] half extent, params[3] convex radius
    box = 1,
    /// params[0] half height of the cylinder, params[1] radius
    capsule = 2,
    /// params[0] half height of the tapered cylinder, params[1] top radius, params[2] bottom radius
    tapered_capsule = 3,
    /// params[0] half height, params[1] radius, params[3] convex radius
    cylinder = 4,
    /// params[0] half height, params[1] top radius, params[2] bottom radius, params[3] convex radius
    tapered_cylinder = 5,
    /// `count` points at floats[data..][0 .. 3 * count], params[3] max convex radius
    convex_hull = 6,
    /// params[0..9] the three vertices, params[9] convex radius
    triangle = 7,
    /// params[0..3] normal, params[3] constant, params[4] half extent
    plane = 8,
    /// `count` triangles at floats[data..][0 .. 9 * count]
    mesh = 9,
    /// params[0..3] center of mass
    empty = 10,
    /// `child` the inner shape, params[0..3] position, `rotation`
    rotated_translated = 11,
    /// `child` the inner shape, params[0..3] scale
    scaled = 12,
    /// `child` the inner shape, params[0..3] offset
    offset_center_of_mass = 13,
    /// `count` children at children[child..]
    static_compound = 14,
    /// `count` children at children[child..]
    mutable_compound = 15,
};

/// A shape, created once per scene from its settings (ShapeSettings::Create) on both sides. Bodies refer to it by
/// index (BodyCreationSettings with a `const Shape *`).
pub const SimShape = extern struct {
    kind: u32,
    /// Per kind parameters, see ShapeKind
    params: [12]f32 = @splat(0),
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
    /// Decorated shapes: index of the inner shape (an earlier entry); compounds: index of the first child
    child: u32 = 0,
    /// Compounds: number of children; convex hull: number of points; mesh: number of triangles
    count: u32 = 0,
    /// Convex hull / mesh: offset in `floats`
    data: u32 = 0,
    /// Convex shapes: the density (ConvexShapeSettings::SetDensity), 0 keeps Jolt's default
    density: f32 = 0,
};

/// A sub shape of a compound
pub const SimCompoundChild = extern struct {
    /// Index of the shape (an earlier entry)
    shape: u32,
    user_data: u32 = 0,
    position: [3]f32 = @splat(0),
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
};

/// Jolt's BodyCreationSettings (the defaults are Jolt's)
pub const SimBody = extern struct {
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
    /// MassProperties override (`override_mass_properties`)
    mass: f32 = 0.0,
    inertia: [16]f32 = @splat(0),
    /// Index of the shape
    shape: u32 = 0,
    object_layer: u32 = Layers.unused1,
    /// EMotionType: 0 static, 1 kinematic, 2 dynamic
    motion_type: u32 = 2,
    /// EAllowedDOFs bits
    allowed_dofs: u32 = 0b111111,
    allow_dynamic_or_kinematic: u32 = 0,
    is_sensor: u32 = 0,
    collide_kinematic_vs_non_dynamic: u32 = 0,
    use_manifold_reduction: u32 = 1,
    apply_gyroscopic_force: u32 = 0,
    /// EMotionQuality: 0 discrete, 1 linear cast
    motion_quality: u32 = 0,
    enhanced_internal_edge_removal: u32 = 0,
    allow_sleeping: u32 = 1,
    num_velocity_steps_override: u32 = 0,
    num_position_steps_override: u32 = 0,
    /// EOverrideMassProperties: 0 calculate mass and inertia, 1 calculate inertia, 2 mass and inertia provided
    override_mass_properties: u32 = 0,
    /// CollisionGroup without a group filter
    collision_group_id: u32 = std.math.maxInt(u32),
    collision_sub_group_id: u32 = std.math.maxInt(u32),
};

/// Jolt's PhysicsSettings (the defaults are Jolt's, checked by the test "SimScenes default settings are Jolt's
/// PhysicsSettings"; booleans as u32)
pub const SimPhysicsSettings = extern struct {
    max_in_flight_body_pairs: i32 = 16384,
    step_listeners_batch_size: i32 = 8,
    step_listener_batches_per_job: i32 = 1,
    baumgarte: f32 = 0.2,
    speculative_contact_distance: f32 = 0.02,
    penetration_slop: f32 = 0.02,
    linear_cast_threshold: f32 = 0.75,
    linear_cast_max_penetration: f32 = 0.25,
    manifold_tolerance: f32 = 1.0e-3,
    max_penetration_distance: f32 = 0.2,
    body_pair_cache_max_delta_position_sq: f32 = @as(f32, 0.001) * @as(f32, 0.001), // Square(0.001f)
    body_pair_cache_cos_max_delta_rotation_div2: f32 = 0.99984769515639123915701155881391,
    contact_normal_cos_max_delta_rotation: f32 = 0.99619469809174553229501040247389,
    contact_point_preserve_lambda_max_dist_sq: f32 = @as(f32, 0.01) * @as(f32, 0.01), // Square(0.01f)
    internal_edge_removal_vertex_tolerance_sq: f32 = 1.0e-8, // cDefaultInternalEdgeRemovalVertexToleranceSq
    num_velocity_steps: u32 = 10,
    num_position_steps: u32 = 2,
    min_velocity_for_restitution: f32 = 1.0,
    time_before_sleep: f32 = 0.5,
    point_velocity_sleep_threshold: f32 = 0.03,
    constraint_warm_start: u32 = 1,
    use_body_pair_contact_cache: u32 = 1,
    use_manifold_reduction: u32 = 1,
    use_large_island_splitter: u32 = 1,
    allow_sleeping: u32 = 1,
    check_active_edges: u32 = 1,
};

/// A step listener (added with `OpKind.add_step_listener`). It records every call and changes the velocity of one
/// body through the no-lock body interface (all bodies are locked during the update).
pub const SimStepListener = extern struct {
    /// The body handle it changes (`invalid_handle`: none)
    body: u32 = invalid_handle,
    /// StepListenerMode
    mode: u32 = 0,
    f: [4]f32 = @splat(0),
};

/// What a step listener does in OnStep (dt is the step's delta time)
pub const StepListenerMode = enum(u32) {
    /// SetLinearVelocity(v * f[0] + (f[1], f[2], f[3]) * dt), which activates the body when it is asleep and the new
    /// velocity is not near zero
    linear_velocity = 0,
    /// SetAngularVelocity(w * f[0] + (f[1], f[2], f[3]) * dt)
    angular_velocity = 1,
    /// AddLinearVelocity((f[1], f[2], f[3]) * dt) (activates the body)
    add_linear_velocity = 2,
    /// Only records the call
    record_only = 3,
};

/// Constraint kinds (Phase 6: the C++ side creates them already, the Zolt side when the class is ported)
pub const ConstraintKind = enum(u32) {
    /// FixedConstraintSettings with mAutoDetectPoint = true
    fixed = 0,
    /// PointConstraintSettings, f[0..3] point1, f[3..6] point2 (world space)
    point = 1,
    /// DistanceConstraintSettings, f[0..3] point1, f[3..6] point2, f[6] min distance, f[7] max distance
    distance = 2,
    /// HingeConstraintSettings, f[0..3] point, f[3..6] hinge axis, f[6..9] normal axis (world space, both bodies)
    hinge = 3,
    /// SliderConstraintSettings with mAutoDetectPoint = true, f[0..3] slider axis
    slider = 4,
};

/// A two body constraint (added with `OpKind.create_constraint`). Body handle `invalid_handle` is Body::sFixedToWorld.
pub const SimConstraint = extern struct {
    kind: u32,
    body1: u32,
    body2: u32,
    f: [16]f32 = @splat(0),
};

/// Operations between the updates. Unless noted otherwise they go through the locking BodyInterface, `body` is a body
/// handle, `activation` is EActivation (0 activate, 1 don't activate), `f` / `r` / `index` / `count` are listed per
/// kind. Every operation returns a u32 that both sides must agree on (a body ID, or IsActive(body) after the operation,
/// or 0).
pub const OpKind = enum(u32) {
    /// CreateBody(bodies[index]) + AddBody(activation); returns the BodyID (invalid when out of bodies, the handle is
    /// then invalid too)
    create_body = 0,
    /// CreateAndAddBody(bodies[index], activation); returns the BodyID
    create_and_add_body = 1,
    /// CreateBody(bodies[index]) without adding it; returns the BodyID
    create_body_no_add = 2,
    /// AddBody(activation)
    add_body = 3,
    /// RemoveBody
    remove_body = 4,
    /// DestroyBody (the body must not be in the broad phase)
    destroy_body = 5,
    /// AddBodiesPrepare / AddBodiesFinalize(activation) of the handles [body, body + count)
    add_bodies = 6,
    /// RemoveBodies of the handles [body, body + count)
    remove_bodies = 7,
    /// ActivateBody
    activate_body = 8,
    /// DeactivateBody
    deactivate_body = 9,
    /// ActivateBodies of the handles [body, body + count)
    activate_bodies = 10,
    /// DeactivateBodies of the handles [body, body + count)
    deactivate_bodies = 11,
    /// AddForce(f[0..3], activation)
    add_force = 12,
    /// AddForce(f[0..3], point r, activation)
    add_force_at_position = 13,
    /// AddTorque(f[0..3], activation)
    add_torque = 14,
    /// AddForceAndTorque(f[0..3], f[3..6], activation)
    add_force_and_torque = 15,
    /// AddImpulse(f[0..3])
    add_impulse = 16,
    /// AddImpulse(f[0..3], point r)
    add_impulse_at_position = 17,
    /// AddAngularImpulse(f[0..3])
    add_angular_impulse = 18,
    /// SetLinearVelocity(f[0..3])
    set_linear_velocity = 19,
    /// SetAngularVelocity(f[0..3])
    set_angular_velocity = 20,
    /// SetLinearAndAngularVelocity(f[0..3], f[3..6])
    set_linear_and_angular_velocity = 21,
    /// AddLinearVelocity(f[0..3])
    add_linear_velocity = 22,
    /// AddLinearAndAngularVelocity(f[0..3], f[3..6])
    add_linear_and_angular_velocity = 23,
    /// SetPosition(r, activation)
    set_position = 24,
    /// SetRotation(f[0..4], activation)
    set_rotation = 25,
    /// SetPositionAndRotation(r, f[0..4], activation)
    set_position_and_rotation = 26,
    /// SetPositionAndRotationWhenChanged(r, f[0..4], activation)
    set_position_and_rotation_when_changed = 27,
    /// SetPositionRotationAndVelocity(r, f[0..4], f[4..7], f[7..10])
    set_position_rotation_and_velocity = 28,
    /// MoveKinematic(r, f[0..4], delta time f[4])
    move_kinematic = 29,
    /// SetMotionType(EMotionType index, activation)
    set_motion_type = 30,
    /// SetMotionQuality(EMotionQuality index)
    set_motion_quality = 31,
    /// SetGravityFactor(f[0])
    set_gravity_factor = 32,
    /// SetMaxLinearVelocity(f[0])
    set_max_linear_velocity = 33,
    /// SetMaxAngularVelocity(f[0])
    set_max_angular_velocity = 34,
    /// ResetSleepTimer
    reset_sleep_timer = 35,
    /// PhysicsSystem::SetGravity(f[0..3])
    set_gravity = 36,
    /// PhysicsSystem::SetPhysicsSettings(physics_settings[index])
    set_physics_settings = 37,
    /// PhysicsSystem::AddStepListener(step_listeners[index]) (each listener is added at most once at a time)
    add_step_listener = 38,
    /// PhysicsSystem::RemoveStepListener(step_listeners[index])
    remove_step_listener = 39,
    /// PhysicsSystem::OptimizeBroadPhase
    optimize_broad_phase = 40,
    /// SetObjectLayer(index)
    set_object_layer = 41,
    /// SetUserData(index | count << 32)
    set_user_data = 42,
    /// SetFriction(f[0]), SetRestitution(f[1]) (no effect on the gravity path, for the contact scenes)
    set_friction_and_restitution = 43,
    /// CreateConstraint(constraints[index]) + PhysicsSystem::AddConstraint (Phase 6)
    create_constraint = 44,
    /// Driver: the following updates use delta time f[0] and `count` collision steps
    set_update = 100,
    /// Driver: both sides save their state in slot `index`
    snapshot = 101,
    /// Driver: Zolt restores the C++ state of slot `index` and C++ the Zolt state (RestoreState, not validating)
    restore_cross = 102,
};

/// An operation, applied before update `update`
pub const SimOp = extern struct {
    update: u32,
    kind: u32,
    body: u32 = invalid_handle,
    count: u32 = 0,
    activation: u32 = 0,
    index: u32 = 0,
    f: [12]f32 = @splat(0),
    r: [3]Real = @splat(0),
};

/// The description of a scene that both sides build from (the C ABI of jolt_sim_create)
pub const SimSceneDesc = extern struct {
    max_bodies: u32,
    num_body_mutexes: u32,
    max_body_pairs: u32,
    max_contact_constraints: u32,
    gravity: [3]f32,
    settings: SimPhysicsSettings,
    shapes: [*]const SimShape,
    num_shapes: u32,
    children: [*]const SimCompoundChild,
    num_children: u32,
    floats: [*]const f32,
    num_floats: u32,
    bodies: [*]const SimBody,
    num_bodies: u32,
    step_listeners: [*]const SimStepListener,
    num_step_listeners: u32,
    constraints: [*]const SimConstraint,
    num_constraints: u32,
    physics_settings: [*]const SimPhysicsSettings,
    num_physics_settings: u32,
};

/// Listener callback kinds of `SimRecord`
pub const RecordKind = enum(u32) {
    /// ids[0] body ID, data[0..2] user data (low, high)
    body_activated = 0,
    body_deactivated = 1,
    /// ids[0] listener index, ids[1] number of the call of this listener, data[0] delta time bits, data[1] first step,
    /// data[2] last step
    step = 2,
    /// ids[0..2] body IDs, ids[2..4] sub shape IDs, data[0] penetration depth, data[1..4] penetration axis
    contact_validate = 3,
    /// ids[0..2] body IDs, ids[2..4] sub shape IDs, data[0..3] world space normal, data[3] penetration depth, data[4]
    /// number of contact points, data[5] combined friction, data[6] combined restitution, data[7] is sensor
    contact_added = 4,
    contact_persisted = 5,
    /// ids: body 1, sub shape 1, body 2, sub shape 2
    contact_removed = 6,
};

/// One listener callback (floats as bits). Records are compared as sorted lists: the order of the callbacks depends
/// on the threads.
pub const SimRecord = extern struct {
    kind: u32,
    ids: [4]u32 = @splat(0),
    data: [8]u32 = @splat(0),

    /// Total order for sorting (the order itself is irrelevant, both lists are sorted the same way)
    pub fn lessThan(_: void, a: SimRecord, b: SimRecord) bool {
        const aa: [13]u32 = @bitCast(a);
        const bb: [13]u32 = @bitCast(b);
        return std.mem.order(u32, &aa, &bb) == .lt;
    }
};

pub const invalid_handle: u32 = std.math.maxInt(u32);

/// Object layers of UnitTests/Layers.h (ZoltTests/Layers.zig)
pub const Layers = struct {
    pub const unused1: u32 = 0;
    pub const unused2: u32 = 1;
    pub const unused3: u32 = 2;
    pub const unused4: u32 = 3;
    pub const unused5: u32 = 4;
    pub const non_moving: u32 = 5;
    pub const moving: u32 = 6;
    pub const moving2: u32 = 7;
    pub const hq_debris: u32 = 8;
    pub const lq_debris: u32 = 9;
    pub const sensor: u32 = 10;
};

// -------------------------------------------------------------------------------------------------------------------
// The scene table

/// What a scene needs that Zolt cannot do yet
pub const Blocker = enum {
    none,
    /// Body pairs reach the narrow phase: ProcessBodyPair and the contact solver (A4, Wave B R2 / R3)
    contacts,
    /// Constraints in islands: the constraint classes (Phase 6) and the solver (Wave B R3)
    constraints,
    /// Linear cast bodies that hit something (Wave B R4)
    ccd_hits,
    /// Soft bodies (Phase 9)
    soft_bodies,
    /// The scene violates a JPH_ASSERT that Jolt's release build (the reference library is built without asserts)
    /// ignores: it compares Jolt's release behavior and runs only without Zolt's asserts (ReleaseFast)
    release_only,

    /// True when Zolt can run the scenes with this blocker (flip when the wave lands)
    pub fn isSupported(self: Blocker) bool {
        return switch (self) {
            .none => true,
            .release_only => !zolt.Core.enable_asserts,
            .contacts, .constraints, .ccd_hits, .soft_bodies => false,
        };
    }
};

pub const SceneEntry = struct {
    name: []const u8,
    build: *const fn (b: *SceneBuilder) Allocator.Error!void,
    blocked_by: Blocker = .none,
    /// D12 sweeps (every temp allocation and every createJob of the update fails once) before every
    /// `sweep_stride`-th update and before the first one, 0: none
    sweep_stride: u32 = 25,
};

pub const scenes = [_]SceneEntry{
    .{ .name = "motion_types_and_qualities", .build = buildMotionTypes },
    .{ .name = "allowed_dofs", .build = buildAllowedDOFs },
    .{ .name = "damping_gravity_factor_and_clamps", .build = buildDampingAndClamps },
    .{ .name = "gyroscopic_rotation", .build = buildGyroscopic },
    .{ .name = "kinematic_move", .build = buildKinematic },
    .{ .name = "forces_torques_impulses", .build = buildForces },
    .{ .name = "velocity_and_position_setters", .build = buildSetters },
    .{ .name = "sleep_and_wake", .build = buildSleep },
    .{ .name = "sleep_settings", .build = buildSleepSettings },
    .{ .name = "add_remove_destroy", .build = buildAddRemoveDestroy },
    .{ .name = "out_of_bodies", .build = buildOutOfBodies },
    .{ .name = "custom_gravity", .build = buildCustomGravity },
    .{ .name = "collision_steps_1", .build = buildCollisionSteps(1) },
    .{ .name = "collision_steps_2", .build = buildCollisionSteps(2) },
    .{ .name = "collision_steps_3", .build = buildCollisionSteps(3) },
    .{ .name = "collision_steps_4", .build = buildCollisionSteps(4) },
    .{ .name = "varying_delta_time_and_steps", .build = buildVaryingUpdates },
    .{ .name = "step_listeners", .build = buildStepListeners },
    .{ .name = "step_listeners_without_active_bodies", .build = buildStepListenersOnly },
    .{ .name = "save_restore_cross", .build = buildSaveRestore },
    .{ .name = "linear_cast_without_hits", .build = buildLinearCast },
    .{ .name = "shapes", .build = buildShapes },
    .{ .name = "motion_type_and_quality_changes", .build = buildMotionChanges },
    .{ .name = "random_1", .build = buildRandom(1, 1) },
    .{ .name = "random_2", .build = buildRandom(2, 2) },
    .{ .name = "random_3", .build = buildRandom(3, 3) },
    .{ .name = "random_4", .build = buildRandom(4, 4) },
    .{ .name = "linear_cast_without_inner_radius", .build = buildLinearCastWithoutInnerRadius, .blocked_by = .release_only },
    // Later waves (blocked until Zolt has the parts, see Blocker)
    .{ .name = "contacts_box_on_floor", .build = buildBoxOnFloor, .blocked_by = .contacts },
    .{ .name = "contacts_pyramid", .build = buildPyramid, .blocked_by = .contacts },
    .{ .name = "contacts_sensor", .build = buildSensor, .blocked_by = .contacts },
    .{ .name = "ccd_bullet_hits_wall", .build = buildBulletHitsWall, .blocked_by = .ccd_hits },
    .{ .name = "constraints_chain", .build = buildConstraintChain, .blocked_by = .constraints },
};

/// A built scene: the description plus the operations and the update parameters (the memory is owned by the builder)
pub const SceneBuilder = struct {
    allocator: Allocator,
    max_bodies: u32 = 1024,
    num_body_mutexes: u32 = 0,
    max_body_pairs: u32 = 4096,
    max_contact_constraints: u32 = 1024,
    gravity: [3]f32 = .{ 0, -9.81, 0 },
    settings: SimPhysicsSettings = .{},
    delta_time: f32 = 1.0 / 60.0,
    collision_steps: u32 = 1,
    num_updates: u32 = 60,

    shapes: std.ArrayList(SimShape) = .empty,
    children: std.ArrayList(SimCompoundChild) = .empty,
    floats: std.ArrayList(f32) = .empty,
    bodies: std.ArrayList(SimBody) = .empty,
    step_listeners: std.ArrayList(SimStepListener) = .empty,
    constraints: std.ArrayList(SimConstraint) = .empty,
    physics_settings: std.ArrayList(SimPhysicsSettings) = .empty,
    ops: std.ArrayList(SimOp) = .empty,

    /// The update before which the next operations run
    current_update: u32 = 0,
    /// Number of body handles created so far
    num_handles: u32 = 0,
    /// Random numbers for the scenes (xorshift32, deterministic)
    rng: u32 = 0x9e3779b9,

    pub fn init(allocator: Allocator) SceneBuilder {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *SceneBuilder) void {
        self.shapes.deinit(self.allocator);
        self.children.deinit(self.allocator);
        self.floats.deinit(self.allocator);
        self.bodies.deinit(self.allocator);
        self.step_listeners.deinit(self.allocator);
        self.constraints.deinit(self.allocator);
        self.physics_settings.deinit(self.allocator);
        self.ops.deinit(self.allocator);
    }

    /// The C ABI description (valid until the builder changes)
    pub fn desc(self: *const SceneBuilder) SimSceneDesc {
        return .{
            .max_bodies = self.max_bodies,
            .num_body_mutexes = self.num_body_mutexes,
            .max_body_pairs = self.max_body_pairs,
            .max_contact_constraints = self.max_contact_constraints,
            .gravity = self.gravity,
            .settings = self.settings,
            .shapes = self.shapes.items.ptr,
            .num_shapes = @intCast(self.shapes.items.len),
            .children = self.children.items.ptr,
            .num_children = @intCast(self.children.items.len),
            .floats = self.floats.items.ptr,
            .num_floats = @intCast(self.floats.items.len),
            .bodies = self.bodies.items.ptr,
            .num_bodies = @intCast(self.bodies.items.len),
            .step_listeners = self.step_listeners.items.ptr,
            .num_step_listeners = @intCast(self.step_listeners.items.len),
            .constraints = self.constraints.items.ptr,
            .num_constraints = @intCast(self.constraints.items.len),
            .physics_settings = self.physics_settings.items.ptr,
            .num_physics_settings = @intCast(self.physics_settings.items.len),
        };
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Shapes

    pub fn addShape(self: *SceneBuilder, shape: SimShape) Allocator.Error!u32 {
        try self.shapes.append(self.allocator, shape);
        return @intCast(self.shapes.items.len - 1);
    }

    pub fn sphere(self: *SceneBuilder, radius: f32) Allocator.Error!u32 {
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.sphere), .params = params(&.{radius}) });
    }

    pub fn box(self: *SceneBuilder, half_extent: [3]f32, convex_radius: f32) Allocator.Error!u32 {
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.box), .params = params(&.{ half_extent[0], half_extent[1], half_extent[2], convex_radius }) });
    }

    pub fn capsule(self: *SceneBuilder, half_height: f32, radius: f32) Allocator.Error!u32 {
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.capsule), .params = params(&.{ half_height, radius }) });
    }

    pub fn taperedCapsule(self: *SceneBuilder, half_height: f32, top_radius: f32, bottom_radius: f32) Allocator.Error!u32 {
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.tapered_capsule), .params = params(&.{ half_height, top_radius, bottom_radius }) });
    }

    pub fn cylinder(self: *SceneBuilder, half_height: f32, radius: f32, convex_radius: f32) Allocator.Error!u32 {
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.cylinder), .params = params(&.{ half_height, radius, 0, convex_radius }) });
    }

    pub fn taperedCylinder(self: *SceneBuilder, half_height: f32, top_radius: f32, bottom_radius: f32, convex_radius: f32) Allocator.Error!u32 {
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.tapered_cylinder), .params = params(&.{ half_height, top_radius, bottom_radius, convex_radius }) });
    }

    pub fn convexHull(self: *SceneBuilder, points: []const [3]f32, max_convex_radius: f32) Allocator.Error!u32 {
        const data: u32 = @intCast(self.floats.items.len);
        for (points) |p| try self.floats.appendSlice(self.allocator, &p);
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.convex_hull), .params = params(&.{ 0, 0, 0, max_convex_radius }), .count = @intCast(points.len), .data = data });
    }

    pub fn triangle(self: *SceneBuilder, v: [9]f32, convex_radius: f32) Allocator.Error!u32 {
        var p: [12]f32 = @splat(0);
        @memcpy(p[0..9], &v);
        p[9] = convex_radius;
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.triangle), .params = p });
    }

    pub fn plane(self: *SceneBuilder, normal: [3]f32, constant: f32, half_extent: f32) Allocator.Error!u32 {
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.plane), .params = params(&.{ normal[0], normal[1], normal[2], constant, half_extent }) });
    }

    pub fn mesh(self: *SceneBuilder, triangles: []const [9]f32) Allocator.Error!u32 {
        const data: u32 = @intCast(self.floats.items.len);
        for (triangles) |t| try self.floats.appendSlice(self.allocator, &t);
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.mesh), .count = @intCast(triangles.len), .data = data });
    }

    pub fn empty(self: *SceneBuilder, center_of_mass: [3]f32) Allocator.Error!u32 {
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.empty), .params = params(&center_of_mass) });
    }

    pub fn rotatedTranslated(self: *SceneBuilder, child: u32, position: [3]f32, rotation_value: [4]f32) Allocator.Error!u32 {
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.rotated_translated), .child = child, .params = params(&position), .rotation = rotation_value });
    }

    pub fn scaled(self: *SceneBuilder, child: u32, scale: [3]f32) Allocator.Error!u32 {
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.scaled), .child = child, .params = params(&scale) });
    }

    pub fn offsetCenterOfMass(self: *SceneBuilder, child: u32, offset: [3]f32) Allocator.Error!u32 {
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.offset_center_of_mass), .child = child, .params = params(&offset) });
    }

    pub fn compound(self: *SceneBuilder, kind: ShapeKind, children: []const SimCompoundChild) Allocator.Error!u32 {
        std.debug.assert(kind == .static_compound or kind == .mutable_compound);
        const first: u32 = @intCast(self.children.items.len);
        try self.children.appendSlice(self.allocator, children);
        return self.addShape(.{ .kind = @intFromEnum(kind), .child = first, .count = @intCast(children.len) });
    }

    /// Set the density of a convex shape
    pub fn setDensity(self: *SceneBuilder, shape: u32, density: f32) void {
        self.shapes.items[shape].density = density;
    }

    fn params(values: []const f32) [12]f32 {
        var p: [12]f32 = @splat(0);
        @memcpy(p[0..values.len], values);
        return p;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Bodies, listeners, settings

    pub fn addBodyDesc(self: *SceneBuilder, body_desc: SimBody) Allocator.Error!u32 {
        try self.bodies.append(self.allocator, body_desc);
        return @intCast(self.bodies.items.len - 1);
    }

    pub fn addStepListener(self: *SceneBuilder, listener: SimStepListener) Allocator.Error!u32 {
        try self.step_listeners.append(self.allocator, listener);
        return @intCast(self.step_listeners.items.len - 1);
    }

    pub fn addConstraint(self: *SceneBuilder, constraint: SimConstraint) Allocator.Error!u32 {
        try self.constraints.append(self.allocator, constraint);
        return @intCast(self.constraints.items.len - 1);
    }

    pub fn addPhysicsSettings(self: *SceneBuilder, settings: SimPhysicsSettings) Allocator.Error!u32 {
        try self.physics_settings.append(self.allocator, settings);
        return @intCast(self.physics_settings.items.len - 1);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Operations

    /// The following operations run before update `update` (non-decreasing)
    pub fn at(self: *SceneBuilder, update: u32) void {
        std.debug.assert(update >= self.current_update);
        std.debug.assert(update < self.num_updates);
        self.current_update = update;
    }

    pub const OpArgs = struct {
        count: u32 = 0,
        activation: Activation = .activate,
        index: u32 = 0,
        f: []const f32 = &.{},
        r: [3]Real = @splat(0),
    };

    pub const Activation = enum(u32) { activate = 0, dont_activate = 1 };

    /// Add an operation on body handle `body` (invalid_handle for system operations)
    pub fn op(self: *SceneBuilder, kind: OpKind, handle: u32, args: OpArgs) Allocator.Error!void {
        var o: SimOp = .{ .update = self.current_update, .kind = @intFromEnum(kind), .body = handle, .count = args.count, .activation = @intFromEnum(args.activation), .index = args.index, .r = args.r };
        @memcpy(o.f[0..args.f.len], args.f);
        try self.ops.append(self.allocator, o);
    }

    /// A system operation
    pub fn sysOp(self: *SceneBuilder, kind: OpKind, args: OpArgs) Allocator.Error!void {
        return self.op(kind, invalid_handle, args);
    }

    fn createWith(self: *SceneBuilder, kind: OpKind, body_desc: u32, activation: Activation) Allocator.Error!u32 {
        try self.op(kind, invalid_handle, .{ .index = body_desc, .activation = activation });
        self.num_handles += 1;
        return self.num_handles - 1;
    }

    /// CreateBody + AddBody, returns the handle
    pub fn createBody(self: *SceneBuilder, body_desc: u32, activation: Activation) Allocator.Error!u32 {
        return self.createWith(.create_body, body_desc, activation);
    }

    /// CreateAndAddBody, returns the handle
    pub fn createAndAddBody(self: *SceneBuilder, body_desc: u32, activation: Activation) Allocator.Error!u32 {
        return self.createWith(.create_and_add_body, body_desc, activation);
    }

    /// CreateBody without adding it, returns the handle
    pub fn createBodyNoAdd(self: *SceneBuilder, body_desc: u32) Allocator.Error!u32 {
        return self.createWith(.create_body_no_add, body_desc, .dont_activate);
    }

    /// Add a body description and create the body
    pub fn body(self: *SceneBuilder, desc_value: SimBody, activation: Activation) Allocator.Error!u32 {
        return self.createBody(try self.addBodyDesc(desc_value), activation);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Random numbers

    pub fn next(self: *SceneBuilder) u32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        return self.rng;
    }

    /// Random float in [min, max)
    pub fn float(self: *SceneBuilder, min: f32, max: f32) f32 {
        const t = @as(f32, @floatFromInt(self.next() >> 8)) / 16777216.0;
        return min + (max - min) * t;
    }

    /// Random integer in [0, n)
    pub fn index(self: *SceneBuilder, n: u32) u32 {
        return self.next() % n;
    }

    pub fn oneIn(self: *SceneBuilder, n: u32) bool {
        return self.index(n) == 0;
    }

    pub fn vec3(self: *SceneBuilder, min: f32, max: f32) [3]f32 {
        return .{ self.float(min, max), self.float(min, max), self.float(min, max) };
    }

    /// Random unit quaternion (normalized in f64 so that both sides get the same bits from the table)
    pub fn rotation(self: *SceneBuilder) [4]f32 {
        return normalizedQuat(.{ self.float(-1, 1), self.float(-1, 1), self.float(-1, 1), self.float(-1, 1) });
    }
};

/// A normalized quaternion (x, y, z, w); the normalization runs in f64 in the scene builder only
pub fn normalizedQuat(q: [4]f32) [4]f32 {
    var len: f64 = 0;
    for (q) |v| len += @as(f64, v) * @as(f64, v);
    len = @sqrt(len);
    if (len < 1.0e-3) return .{ 0, 0, 0, 1 };
    var result: [4]f32 = undefined;
    for (q, &result) |v, *r| r.* = @floatCast(@as(f64, v) / len);
    return result;
}

/// Rotation of `angle` radians around a normalized axis (computed in f64 by the scene builder)
pub fn axisAngle(axis: [3]f32, angle: f64) [4]f32 {
    const s = @sin(angle * 0.5);
    return normalizedQuat(.{ @floatCast(axis[0] * s), @floatCast(axis[1] * s), @floatCast(axis[2] * s), @floatCast(@cos(angle * 0.5)) });
}

/// Position of cell `cell` of a grid with `spacing` meters between the cells (60 cells per row), at height `y`
pub fn gridPosition(cell: u32, spacing: Real, y: Real) [3]Real {
    const row = cell / 60;
    const col = cell % 60;
    return .{ @as(Real, @floatFromInt(col)) * spacing - 300.0, y, @as(Real, @floatFromInt(row)) * spacing - 300.0 };
}

const identity_rotation: [4]f32 = .{ 0, 0, 0, 1 };

// -------------------------------------------------------------------------------------------------------------------
// Gravity-only scenes

/// A few shapes with different inertia that the scenes use
const BasicShapes = struct {
    sphere: u32,
    box: u32,
    flat_box: u32,
    capsule: u32,
    cylinder: u32,
    compound: u32,

    fn add(b: *SceneBuilder) Allocator.Error!BasicShapes {
        const sphere = try b.sphere(0.5);
        const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
        const flat_box = try b.box(.{ 1.0, 0.2, 0.6 }, 0.05);
        const capsule = try b.capsule(0.8, 0.3);
        const cylinder = try b.cylinder(0.4, 0.6, 0.05);
        const compound = try b.compound(.static_compound, &.{
            .{ .shape = flat_box, .position = .{ 0.5, 0, 0 } },
            .{ .shape = sphere, .position = .{ -0.5, 0.3, 0.2 }, .rotation = axisAngle(.{ 0, 1, 0 }, 0.3) },
        });
        return .{ .sphere = sphere, .box = box, .flat_box = flat_box, .capsule = capsule, .cylinder = cylinder, .compound = compound };
    }

    fn pick(self: BasicShapes, i: u32) u32 {
        return switch (i % 6) {
            0 => self.sphere,
            1 => self.box,
            2 => self.flat_box,
            3 => self.capsule,
            4 => self.cylinder,
            else => self.compound,
        };
    }
};

/// Every motion type and motion quality, active and inactive, in colliding layers spread out on a grid and in the
/// unused layers on top of each other
fn buildMotionTypes(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    var cell: u32 = 0;
    const layers = [_]u32{ Layers.moving, Layers.moving2, Layers.hq_debris, Layers.lq_debris, Layers.unused2 };
    for (0..3) |motion_type| {
        for (0..2) |quality| {
            for (layers) |layer| {
                for ([_]SceneBuilder.Activation{ .activate, .dont_activate }) |activation| {
                    const i = cell;
                    _ = try b.body(.{
                        .position = gridPosition(cell, 20, 10),
                        .rotation = axisAngle(.{ 0, 0, 1 }, 0.1 * @as(f64, @floatFromInt(i))),
                        .linear_velocity = if (motion_type == 0) @splat(0) else .{ 0.5, @as(f32, @floatFromInt(i % 5)) - 2.0, -0.25 },
                        .angular_velocity = if (motion_type == 0) @splat(0) else .{ 0.1, 0.7, -0.3 },
                        .shape = shapes.pick(i),
                        .object_layer = if (motion_type == 0 and layer != Layers.unused2) Layers.non_moving else layer,
                        .motion_type = @intCast(motion_type),
                        .motion_quality = @intCast(quality),
                        .user_data = 1000 + i,
                    }, activation);
                    cell += 1;
                }
            }
        }
    }
    // Static and dynamic bodies that can switch motion type (they have motion properties)
    _ = try b.body(.{ .position = gridPosition(cell, 20, 5), .shape = shapes.box, .motion_type = 0, .allow_dynamic_or_kinematic = 1, .object_layer = Layers.unused1 }, .activate);
    _ = try b.body(.{ .position = gridPosition(cell + 1, 20, 5), .shape = shapes.capsule, .motion_type = 2, .allow_dynamic_or_kinematic = 1, .object_layer = Layers.moving }, .activate);
}

/// Every combination of allowed degrees of freedom, with velocities in all directions
fn buildAllowedDOFs(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    var dofs: u32 = 1;
    while (dofs < 64) : (dofs += 1) {
        _ = try b.body(.{
            .position = gridPosition(dofs, 0, 1), // The unused layer collides with nothing
            .rotation = axisAngle(.{ 0.267261, 0.534522, 0.801784 }, 0.05 * @as(f64, @floatFromInt(dofs))),
            .linear_velocity = .{ 1.0, 2.0, -3.0 },
            .angular_velocity = .{ 2.0, -1.0, 0.5 },
            .shape = shapes.pick(dofs),
            .allowed_dofs = dofs,
            .apply_gyroscopic_force = dofs & 1,
            .object_layer = Layers.unused3,
        }, .activate);
    }
    b.num_updates = 40;
    // Forces and torques on every body halfway (they are locked by the DOFs too)
    b.at(20);
    var handle: u32 = 0;
    while (handle < 63) : (handle += 1) {
        try b.op(.add_force_and_torque, handle, .{ .f = &.{ 100, 200, -300, 10, 20, -30 } });
        try b.op(.add_impulse_at_position, handle, .{ .f = &.{ 1, -2, 3 }, .r = .{ 0.5, 1.5, 0.25 } });
    }
}

/// Linear and angular damping, gravity factors and the max linear / angular velocity clamps
fn buildDampingAndClamps(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    const dampings = [_]f32{ 0.0, 0.05, 0.5, 2.0, 70.0 }; // 70 * dt > 1: the drag clamps the factor at 0
    const gravity_factors = [_]f32{ 1.0, 0.0, -0.5, 2.5 };
    var cell: u32 = 0;
    for (dampings) |linear_damping| {
        for (dampings) |angular_damping| {
            for (gravity_factors) |gravity_factor| {
                _ = try b.body(.{
                    .position = gridPosition(cell, 0, 0),
                    .linear_velocity = .{ 3.0, 1.0, -2.0 },
                    .angular_velocity = .{ 1.0, 4.0, -2.0 },
                    .linear_damping = linear_damping,
                    .angular_damping = angular_damping,
                    .gravity_factor = gravity_factor,
                    .shape = shapes.pick(cell),
                    .object_layer = Layers.unused1,
                }, .activate);
                cell += 1;
            }
        }
    }
    // Clamps: gravity accelerates into the maximum linear velocity, torques into the maximum angular velocity, the
    // clamping setters, tiny maxima (the initial velocities must be within the maxima, Jolt asserts that)
    const clamps = [_][2]f32{ .{ 500.0, 0.25 * std.math.pi * 60.0 }, .{ 5.0, 1.0 }, .{ 0.5, 0.1 }, .{ 1000.0, 100.0 }, .{ 0.0, 0.0 } };
    const first_clamped = cell;
    for (clamps) |clamp| {
        for ([_][3]f32{ .{ 6, -7, 8 }, .{ 0, 0, 0 }, .{ 3, 4, 0 } }) |velocity| {
            _ = try b.body(.{
                .position = gridPosition(cell, 0, 0),
                .linear_velocity = scaledToLength(velocity, 0.9 * clamp[0]),
                .angular_velocity = scaledToLength(.{ velocity[2], velocity[0], 5.0 }, 0.9 * clamp[1]),
                .max_linear_velocity = clamp[0],
                .max_angular_velocity = clamp[1],
                .linear_damping = 0,
                .angular_damping = 0,
                .gravity_factor = 5.0,
                .shape = shapes.pick(cell),
                .object_layer = Layers.unused1,
            }, .activate);
            cell += 1;
        }
    }
    b.at(5);
    var h = first_clamped;
    while (h < cell) : (h += 1) {
        try b.op(.add_torque, h, .{ .f = &.{ 1000, -2000, 500 } });
        if (h % 3 == 0)
            try b.op(.set_linear_and_angular_velocity, h, .{ .f = &.{ 600, -700, 800, 60, 70, -80 } }); // Clamped by the setter
        if (h % 3 == 1)
            try b.op(.add_impulse, h, .{ .f = &.{ 5000, 0, 0 } });
    }
    // Change the parameters between updates
    b.at(30);
    try b.op(.set_gravity_factor, 0, .{ .f = &.{3.0} });
    try b.op(.set_max_linear_velocity, 1, .{ .f = &.{0.1} });
    try b.op(.set_max_angular_velocity, 2, .{ .f = &.{0.2} });
    try b.op(.set_linear_velocity, cell - 1, .{ .f = &.{ 1000, 1000, 1000 } });
}

/// Rotating bodies with asymmetric inertia and the gyroscopic force, with and without the force
fn buildGyroscopic(b: *SceneBuilder) Allocator.Error!void {
    const thin = try b.box(.{ 2.0, 0.1, 0.5 }, 0.05);
    const long_cylinder = try b.cylinder(2.0, 0.2, 0.05);
    const offset = try b.offsetCenterOfMass(thin, .{ 0.3, -0.2, 0.1 });
    const shapes = [_]u32{ thin, long_cylinder, offset };
    b.gravity = .{ 0, 0, 0 };
    b.num_updates = 120;
    var cell: u32 = 0;
    for (shapes) |shape| {
        for (0..2) |gyro| {
            for ([_][3]f32{ .{ 0.1, 5.0, 0.2 }, .{ 3.0, 3.0, -3.0 }, .{ 0, 0, 20.0 }, .{ 10, 0.01, 0 } }) |w| {
                _ = try b.body(.{
                    .position = gridPosition(cell, 0, 0),
                    .rotation = axisAngle(.{ 0.6, 0.0, 0.8 }, 0.7),
                    .angular_velocity = w,
                    .angular_damping = 0,
                    .linear_damping = 0,
                    .apply_gyroscopic_force = @intCast(gyro),
                    .max_angular_velocity = 100.0,
                    .allow_sleeping = 0,
                    .shape = shape,
                    .object_layer = Layers.unused1,
                }, .activate);
                cell += 1;
            }
        }
    }
    // An inertia override and an inertia multiplier
    var inertia: [16]f32 = @splat(0);
    inertia[0] = 2.0;
    inertia[5] = 0.5;
    inertia[10] = 1.0;
    inertia[1] = 0.1;
    inertia[4] = 0.1;
    inertia[15] = 1.0;
    _ = try b.body(.{ .position = gridPosition(cell, 0, 0), .angular_velocity = .{ 1, 2, 3 }, .apply_gyroscopic_force = 1, .shape = thin, .override_mass_properties = 2, .mass = 3.0, .inertia = inertia, .object_layer = Layers.unused1 }, .activate);
    _ = try b.body(.{ .position = gridPosition(cell + 1, 0, 0), .angular_velocity = .{ 1, 2, 3 }, .apply_gyroscopic_force = 1, .shape = long_cylinder, .override_mass_properties = 1, .mass = 7.0, .inertia_multiplier = 2.5, .object_layer = Layers.unused1 }, .activate);
}

/// Kinematic bodies moved with MoveKinematic every update, then left alone (they keep their velocity), then stopped
/// (they fall asleep)
fn buildKinematic(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.num_updates = 100;
    const num_bodies = 8;
    for (0..num_bodies) |i| {
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 25, 3),
            .shape = shapes.pick(@intCast(i)),
            .motion_type = 1,
            .object_layer = if (i % 2 == 0) Layers.moving else Layers.unused1,
            .motion_quality = @intCast(i % 2),
        }, if (i % 3 == 0) .dont_activate else .activate);
    }
    var u: u32 = 0;
    while (u < 40) : (u += 1) {
        b.at(u);
        for (0..num_bodies) |i| {
            const t: f64 = @floatFromInt(u + 1);
            const base = gridPosition(@intCast(i), 25, 3);
            const p: [3]Real = .{ base[0] + @as(Real, @floatCast(0.1 * t)), base[1] + @as(Real, @floatCast(@sin(t * 0.2))), base[2] - @as(Real, @floatCast(0.05 * t * @as(f64, @floatFromInt(i)))) };
            try b.op(.move_kinematic, @intCast(i), .{ .r = p, .f = &(axisAngle(.{ 0, 1, 0 }, 0.05 * t * @as(f64, @floatFromInt(i + 1))) ++ [1]f32{b.delta_time}) });
        }
    }
    // Then they keep their velocity until update 50, where they move back slowly to their start, and at update 60 they
    // stop (zero velocity) and fall asleep later
    b.at(50);
    for (0..num_bodies) |i| {
        const base = gridPosition(@intCast(i), 25, 3);
        try b.op(.move_kinematic, @intCast(i), .{ .r = base, .f = &(identity_rotation ++ [1]f32{0.5}) });
    }
    b.at(60);
    for (0..num_bodies) |i|
        try b.op(.set_linear_and_angular_velocity, @intCast(i), .{ .f = &.{ 0, 0, 0, 0, 0, 0 } });
}

/// Forces, torques and impulses applied between the updates (with and without activation, on sleeping bodies)
fn buildForces(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.num_updates = 90;
    const num_bodies = 12;
    for (0..num_bodies) |i| {
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 0, 0),
            .rotation = axisAngle(.{ 1, 0, 0 }, 0.2 * @as(f64, @floatFromInt(i))),
            .shape = shapes.pick(@intCast(i)),
            .gravity_factor = if (i < 6) 0.0 else 1.0, // The ones without gravity fall asleep
            .object_layer = Layers.unused1,
        }, if (i % 4 == 3) .dont_activate else .activate);
    }
    var u: u32 = 0;
    while (u < 78) : (u += 7) {
        b.at(u);
        const h: u32 = (u / 7) % num_bodies;
        const s: f32 = @floatFromInt(u + 1);
        try b.op(.add_force, h, .{ .f = &.{ 100 * s, -20, 5 }, .activation = if (u % 3 == 0) .dont_activate else .activate });
        try b.op(.add_force_at_position, (h + 1) % num_bodies, .{ .f = &.{ 0, 50, 10 }, .r = gridPositionOffset(gridPosition((h + 1) % num_bodies, 0, 0), .{ 0.4, 0.1, -0.3 }) });
        try b.op(.add_torque, (h + 2) % num_bodies, .{ .f = &.{ 5, -3, 2 * s } });
        try b.op(.add_force_and_torque, (h + 3) % num_bodies, .{ .f = &.{ -10, 0, 7, 0, 3, 1 }, .activation = if (u % 2 == 0) .dont_activate else .activate });
        try b.op(.add_impulse, (h + 4) % num_bodies, .{ .f = &.{ 0.5, 2, -1 } });
        try b.op(.add_impulse_at_position, (h + 5) % num_bodies, .{ .f = &.{ 1, 0, 0 }, .r = gridPositionOffset(gridPosition((h + 5) % num_bodies, 0, 0), .{ 0, 0.5, 0 }) });
        try b.op(.add_angular_impulse, (h + 6) % num_bodies, .{ .f = &.{ 0, 0.3, -0.2 } });
    }
    // Forces twice in a row (they accumulate) and on a sleeping body without activation
    b.at(80);
    try b.op(.add_force, 0, .{ .f = &.{ 1, 2, 3 } });
    try b.op(.add_force, 0, .{ .f = &.{ 4, 5, 6 } });
    try b.op(.add_torque, 1, .{ .f = &.{ 1, 2, 3 }, .activation = .dont_activate });
}

/// `v` scaled to length `len` when it is longer (computed in f64 by the scene builder)
fn scaledToLength(v: [3]f32, len: f32) [3]f32 {
    const l = @sqrt(@as(f64, v[0]) * v[0] + @as(f64, v[1]) * v[1] + @as(f64, v[2]) * v[2]);
    if (l <= len) return v;
    const s = @as(f64, len) / l;
    return .{ @floatCast(v[0] * s), @floatCast(v[1] * s), @floatCast(v[2] * s) };
}

fn gridPositionOffset(p: [3]Real, offset: [3]f32) [3]Real {
    return .{ p[0] + offset[0], p[1] + offset[1], p[2] + offset[2] };
}

/// The velocity and position setters of the body interface
fn buildSetters(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.num_updates = 60;
    const num_bodies = 10;
    for (0..num_bodies) |i| {
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 30, 2),
            .shape = shapes.pick(@intCast(i)),
            .motion_type = if (i == 9) 1 else 2,
            .gravity_factor = if (i % 2 == 0) 0.0 else 1.0,
            .object_layer = if (i % 3 == 0) Layers.moving else Layers.unused1,
        }, .activate);
    }
    // A static body that is moved around
    const static_body = try b.body(.{ .position = gridPosition(20, 30, 2), .shape = shapes.box, .motion_type = 0, .object_layer = Layers.unused1 }, .dont_activate);
    var u: u32 = 0;
    while (u < 50) : (u += 5) {
        b.at(u);
        const s: f32 = @floatFromInt(u);
        const h: u32 = (u / 5) % num_bodies;
        try b.op(.set_linear_velocity, h, .{ .f = &.{ s * 0.1, 1, -0.5 } });
        try b.op(.set_angular_velocity, (h + 1) % num_bodies, .{ .f = &.{ 0, s * 0.05, 1 } });
        try b.op(.set_linear_and_angular_velocity, (h + 2) % num_bodies, .{ .f = &.{ 1, 0, 0, 0, 0, 2 } });
        try b.op(.add_linear_velocity, (h + 3) % num_bodies, .{ .f = &.{ 0, 0.5, 0 } });
        try b.op(.add_linear_and_angular_velocity, (h + 4) % num_bodies, .{ .f = &.{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 } });
        const p = gridPosition((h + 5) % num_bodies, 30, 2 + s * 0.1);
        try b.op(.set_position, (h + 5) % num_bodies, .{ .r = p, .activation = if (u % 10 == 0) .dont_activate else .activate });
        try b.op(.set_rotation, (h + 6) % num_bodies, .{ .f = &axisAngle(.{ 0, 0, 1 }, s * 0.03), .activation = .activate });
        try b.op(.set_position_and_rotation, (h + 7) % num_bodies, .{ .r = gridPosition((h + 7) % num_bodies, 30, 1), .f = &axisAngle(.{ 1, 0, 0 }, s * 0.02) });
        try b.op(.set_position_and_rotation_when_changed, (h + 8) % num_bodies, .{ .r = gridPosition((h + 8) % num_bodies, 30, 2), .f = &identity_rotation, .activation = .dont_activate });
        try b.op(.set_position_rotation_and_velocity, (h + 9) % num_bodies, .{ .r = gridPosition((h + 9) % num_bodies, 30, 3), .f = &(axisAngle(.{ 0, 1, 0 }, 0.5) ++ [6]f32{ 0, 2, 0, 0.1, 0, 0 }) });
        try b.op(.set_position_and_rotation, static_body, .{ .r = gridPosition(20, 30, s), .f = &axisAngle(.{ 0, 1, 0 }, s * 0.1), .activation = .dont_activate });
    }
    // Setting the velocity of a sleeping body to zero does not wake it
    b.at(55);
    try b.op(.set_linear_velocity, 0, .{ .f = &.{ 0, 0, 0 } });
}

/// Bodies that fall asleep (the sleep test of CheckSleepAndUpdateBounds without contacts) and are woken up again
fn buildSleep(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.num_updates = 150;
    const num_bodies = 16;
    for (0..num_bodies) |i| {
        const fi: f32 = @floatFromInt(i);
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 0, 0),
            .shape = shapes.pick(@intCast(i)),
            .gravity_factor = 0.0,
            .linear_velocity = .{ 0.01 * fi, 0, 0 }, // Below / above the sleep threshold
            .angular_velocity = .{ 0, 0.02 * fi, 0 },
            .linear_damping = 0.5,
            .angular_damping = 0.5,
            .allow_sleeping = if (i == 7) 0 else 1,
            .object_layer = Layers.unused1,
            .user_data = 0xdead_0000_0000 + i,
        }, .activate);
    }
    // Wake them up in different ways
    b.at(70);
    try b.op(.activate_body, 0, .{});
    try b.op(.add_force, 1, .{ .f = &.{ 10, 0, 0 } });
    try b.op(.add_impulse, 2, .{ .f = &.{ 0, 1, 0 } });
    try b.op(.set_linear_velocity, 3, .{ .f = &.{ 0.5, 0, 0 } });
    try b.op(.activate_bodies, 4, .{ .count = 3 });
    try b.op(.reset_sleep_timer, 8, .{});
    try b.op(.add_force, 9, .{ .f = &.{ 10, 0, 0 }, .activation = .dont_activate }); // Asleep: the force is dropped
    b.at(80);
    try b.op(.deactivate_body, 10, .{});
    try b.op(.deactivate_bodies, 0, .{ .count = 2 });
    try b.op(.activate_body, 9, .{});
    b.at(100);
    try b.op(.set_angular_velocity, 11, .{ .f = &.{ 0, 0, 3 } });
    try b.op(.reset_sleep_timer, 12, .{});
}

/// The sleep settings: sleeping disabled, other thresholds, then enabled again
fn buildSleepSettings(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.num_updates = 120;
    b.settings.time_before_sleep = 0.25;
    b.settings.point_velocity_sleep_threshold = 0.1;
    const num_bodies = 8;
    for (0..num_bodies) |i| {
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 0, 0),
            .shape = shapes.pick(@intCast(i)),
            .gravity_factor = 0.0,
            .linear_velocity = .{ 0.02 * @as(f32, @floatFromInt(i)), 0, 0 },
            .angular_velocity = .{ 0.05, 0, 0.01 * @as(f32, @floatFromInt(i)) },
            .linear_damping = 0.0,
            .angular_damping = 0.0,
            .object_layer = Layers.unused1,
        }, .activate);
    }
    var no_sleep: SimPhysicsSettings = b.settings;
    no_sleep.allow_sleeping = 0;
    const no_sleep_index = try b.addPhysicsSettings(no_sleep);
    var slow_sleep: SimPhysicsSettings = b.settings;
    slow_sleep.time_before_sleep = 1.0;
    slow_sleep.point_velocity_sleep_threshold = 0.5;
    const slow_sleep_index = try b.addPhysicsSettings(slow_sleep);
    b.at(10);
    try b.sysOp(.set_physics_settings, .{ .index = no_sleep_index });
    b.at(40);
    try b.op(.activate_bodies, 0, .{ .count = num_bodies });
    try b.sysOp(.set_physics_settings, .{ .index = slow_sleep_index });
}

/// Adding, removing and destroying bodies between updates: the IDs are reused with new sequence numbers, removed bodies
/// leave the saved state, the batch API
fn buildAddRemoveDestroy(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.num_updates = 80;
    var descs: [6]u32 = undefined;
    for (&descs, 0..) |*d, i| {
        d.* = try b.addBodyDesc(.{
            .position = gridPosition(@intCast(i), 0, 5),
            .linear_velocity = .{ 0, 1, 0 },
            .shape = shapes.pick(@intCast(i)),
            .object_layer = Layers.unused1,
            .motion_type = if (i == 5) 0 else 2,
        });
    }
    // 0..5 created and added
    for (descs) |d| _ = try b.createBody(d, .activate);
    // 6..8 created without adding, added later as a batch
    const not_added = try b.createBodyNoAdd(descs[0]);
    _ = try b.createBodyNoAdd(descs[1]);
    _ = try b.createBodyNoAdd(descs[2]);
    b.at(5);
    try b.op(.add_bodies, not_added, .{ .count = 3, .activation = .activate });
    b.at(10);
    try b.op(.remove_body, 1, .{});
    try b.op(.remove_bodies, 2, .{ .count = 2 });
    b.at(15);
    try b.op(.destroy_body, 1, .{});
    try b.op(.add_body, 2, .{ .activation = .dont_activate });
    // Reuses the ID of body 1 with the next sequence number
    _ = try b.createAndAddBody(descs[3], .activate);
    b.at(20);
    try b.op(.add_body, 3, .{ .activation = .activate });
    try b.op(.remove_body, 5, .{}); // Static
    try b.op(.remove_body, 0, .{});
    b.at(21);
    try b.op(.destroy_body, 0, .{});
    try b.op(.destroy_body, 5, .{});
    b.at(25);
    for (0..10) |i| _ = try b.createBody(descs[i % 6], if (i % 2 == 0) .activate else .dont_activate);
    b.at(40);
    try b.op(.remove_bodies, 10, .{ .count = 5 });
    b.at(41);
    try b.op(.add_bodies, 10, .{ .count = 5, .activation = .dont_activate });
    b.at(50);
    var h: u32 = 10;
    while (h < 15) : (h += 1) {
        try b.op(.remove_body, h, .{});
        try b.op(.destroy_body, h, .{});
    }
    for (0..5) |i| _ = try b.createAndAddBody(descs[i], .activate);
    b.at(60);
    try b.sysOp(.optimize_broad_phase, .{});
    try b.op(.set_object_layer, 3, .{ .index = Layers.unused2 });
    try b.op(.set_user_data, 4, .{ .index = 0x1234, .count = 0x5678 });
}

/// More bodies than the system can hold: CreateBody returns null, the IDs run out and are reused
fn buildOutOfBodies(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.max_bodies = 6;
    b.num_body_mutexes = 4;
    b.num_updates = 30;
    const d = try b.addBodyDesc(.{ .shape = shapes.sphere, .object_layer = Layers.unused1, .linear_velocity = .{ 1, 0, 0 } });
    for (0..8) |_| _ = try b.createBody(d, .activate); // The last two are invalid
    b.at(10);
    try b.op(.remove_body, 2, .{});
    try b.op(.destroy_body, 2, .{});
    _ = try b.createAndAddBody(d, .activate); // Fits again
    _ = try b.createAndAddBody(d, .activate); // Does not fit
}

/// Custom gravity, changed between updates
fn buildCustomGravity(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.gravity = .{ 1.5, -3.0, 0.25 };
    b.num_updates = 60;
    for (0..12) |i| {
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 0, 0),
            .shape = shapes.pick(@intCast(i)),
            .gravity_factor = @as(f32, @floatFromInt(i % 4)) * 0.5,
            .object_layer = Layers.unused1,
        }, .activate);
    }
    b.at(20);
    try b.sysOp(.set_gravity, .{ .f = &.{ 0, 0, 0 } });
    b.at(30);
    try b.sysOp(.set_gravity, .{ .f = &.{ -100, 50, 1.0e-5 } });
    b.at(40);
    try b.sysOp(.set_gravity, .{ .f = &.{ 0, -9.81, 0 } });
}

/// The same scene with 1..4 collision steps per update
fn buildCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const shapes = try BasicShapes.add(b);
            b.collision_steps = steps;
            b.num_updates = 50;
            for (0..24) |i| {
                _ = try b.body(.{
                    .position = gridPosition(@intCast(i), 20, 10),
                    .rotation = axisAngle(.{ 0, 1, 0 }, @floatFromInt(i)),
                    .linear_velocity = .{ 0, 5, 0 },
                    .angular_velocity = .{ 0.5, 0.2 * @as(f32, @floatFromInt(i % 3)), 0 },
                    .shape = shapes.pick(@intCast(i)),
                    .apply_gyroscopic_force = @intCast(i % 2),
                    .motion_type = if (i % 8 == 7) 1 else 2,
                    .motion_quality = @intCast((i / 2) % 2),
                    .gravity_factor = if (i % 5 == 0) 0.0 else 1.0, // These fall asleep
                    .linear_damping = if (i % 5 == 0) 0.9 else 0.05,
                    .angular_damping = if (i % 5 == 0) 0.9 else 0.05,
                    .object_layer = if (i % 2 == 0) Layers.moving else Layers.unused1,
                }, .activate);
            }
            b.at(25);
            try b.op(.add_impulse, 3, .{ .f = &.{ 5, 0, 0 } });
            try b.op(.activate_body, 0, .{});
        }
    }.build;
}

/// The delta time and the number of collision steps change between updates, including zero delta time (the early out
/// path of Update) and the warm start ratio of a larger step
fn buildVaryingUpdates(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.num_updates = 60;
    for (0..10) |i| {
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 0, 0),
            .shape = shapes.pick(@intCast(i)),
            .angular_velocity = .{ 1, 0, 0 },
            .apply_gyroscopic_force = 1,
            .gravity_factor = if (i < 3) 0.0 else 1.0,
            .linear_damping = 0.3,
            .object_layer = Layers.unused1,
        }, .activate);
    }
    const params = [_]struct { u32, f32, u32 }{
        .{ 5, 1.0 / 30.0, 2 },   .{ 10, 0.0, 1 },        .{ 11, 1.0 / 60.0, 3 }, .{ 15, 0.1, 4 },
        .{ 20, 1.0 / 120.0, 1 }, .{ 30, 1.0 / 90.0, 2 }, .{ 40, 0.0, 3 },        .{ 41, 0.05, 1 },
    };
    for (params) |p| {
        b.at(p[0]);
        try b.sysOp(.set_update, .{ .f = &.{p[1]}, .count = p[2] });
    }
}

/// Step listeners that change the velocities of bodies (more than one step listener job with several threads), added
/// and removed between the updates, one wakes a sleeping body up
fn buildStepListeners(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.num_updates = 80;
    b.collision_steps = 2;
    const num_bodies = 24;
    for (0..num_bodies) |i| {
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 0, 0),
            .shape = shapes.pick(@intCast(i)),
            .gravity_factor = if (i >= 20) 0.0 else 1.0,
            .object_layer = Layers.unused1,
        }, if (i == 23) .dont_activate else .activate);
    }
    // Listener i changes body i (each body is touched by at most one listener)
    for (0..num_bodies) |i| {
        const mode: StepListenerMode = switch (i % 4) {
            0 => .linear_velocity,
            1 => .angular_velocity,
            2 => .add_linear_velocity,
            else => .record_only,
        };
        const listener = try b.addStepListener(.{ .body = @intCast(i), .mode = @intFromEnum(mode), .f = .{ 0.99, 0.5, 9.0, -0.25 } });
        try b.sysOp(.add_step_listener, .{ .index = listener });
    }
    // The body that sleeps from the start is woken up by its listener at update 30
    const waker = try b.addStepListener(.{ .body = 23, .mode = @intFromEnum(StepListenerMode.linear_velocity), .f = .{ 1.0, 0.0, 1.0, 0.0 } });
    b.at(20);
    try b.sysOp(.remove_step_listener, .{ .index = 23 });
    try b.sysOp(.remove_step_listener, .{ .index = 5 });
    b.at(30);
    try b.sysOp(.add_step_listener, .{ .index = waker });
    b.at(50);
    try b.sysOp(.add_step_listener, .{ .index = 5 });
    var settings: SimPhysicsSettings = b.settings;
    settings.step_listeners_batch_size = 3;
    settings.step_listener_batches_per_job = 2;
    const settings_index = try b.addPhysicsSettings(settings);
    b.at(60);
    try b.sysOp(.set_physics_settings, .{ .index = settings_index });
}

/// Step listeners keep the update running when no body is active
fn buildStepListenersOnly(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.num_updates = 30;
    _ = try b.body(.{ .shape = shapes.box, .object_layer = Layers.unused1 }, .dont_activate);
    _ = try b.body(.{ .position = .{ 10, 0, 0 }, .shape = shapes.box, .motion_type = 0, .object_layer = Layers.unused1 }, .dont_activate);
    const recorder = try b.addStepListener(.{ .mode = @intFromEnum(StepListenerMode.record_only) });
    try b.sysOp(.add_step_listener, .{ .index = recorder });
    const waker = try b.addStepListener(.{ .body = 0, .mode = @intFromEnum(StepListenerMode.add_linear_velocity), .f = .{ 0, 1, 0, 0 } });
    b.at(10);
    try b.sysOp(.add_step_listener, .{ .index = waker });
    b.at(11);
    try b.sysOp(.remove_step_listener, .{ .index = waker });
    try b.sysOp(.set_update, .{ .f = &.{0.0}, .count = 1 });
    b.at(12);
    try b.sysOp(.set_update, .{ .f = &.{1.0 / 60.0}, .count = 1 });
}

/// SaveState / RestoreState round trips in the middle of a run: Zolt restores a C++ state and C++ a Zolt state. The
/// restored states have other active bodies: the bodies without velocity and gravity fall asleep after about 30
/// updates, restoring an older state wakes them up again and restoring a newer one puts them to sleep again.
fn buildSaveRestore(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.num_updates = 100;
    for (0..12) |i| {
        const still = i % 2 == 0; // These fall asleep
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 0, 0),
            .shape = shapes.pick(@intCast(i)),
            .angular_velocity = if (still) @splat(0) else .{ 0.3, 0.1, 0 },
            .gravity_factor = if (still) 0.0 else 1.0,
            .object_layer = Layers.unused1,
        }, if (i == 11) .dont_activate else .activate);
    }
    b.at(10);
    try b.sysOp(.snapshot, .{ .index = 0 });
    try b.sysOp(.set_gravity, .{ .f = &.{ 0, -20, 0 } });
    b.at(40);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Back to update 10 (the gravity is restored too): the sleeping bodies wake up
    b.at(70);
    try b.sysOp(.snapshot, .{ .index = 1 });
    try b.op(.add_force, 11, .{ .f = &.{ 0, 100, 0 } });
    b.at(80);
    try b.sysOp(.restore_cross, .{ .index = 1 }); // Body 11 is asleep again
    try b.sysOp(.restore_cross, .{ .index = 1 }); // Twice in a row
    b.at(90);
    try b.sysOp(.snapshot, .{ .index = 2 });
    try b.sysOp(.restore_cross, .{ .index = 2 }); // The current state
}

/// Linear cast bodies that fly fast enough for a cast but never find a body in their path (the CCD jobs without hits)
fn buildLinearCast(b: *SceneBuilder) Allocator.Error!void {
    const bullet = try b.sphere(0.1);
    const thin = try b.box(.{ 0.05, 0.5, 0.5 }, 0.01);
    b.num_updates = 40;
    b.collision_steps = 2;
    for (0..16) |i| {
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 40, 100),
            .linear_velocity = .{ 0, -50.0 - 10.0 * @as(f32, @floatFromInt(i)), 0 },
            .angular_velocity = .{ 5, 0, 0 },
            .shape = if (i % 2 == 0) bullet else thin,
            .motion_quality = 1,
            .is_sensor = if (i == 15) 1 else 0, // Sensors don't cast
            .motion_type = if (i == 14) 1 else 2, // Kinematic bodies don't cast
            .object_layer = if (i % 3 == 0) Layers.unused1 else Layers.moving,
        }, .activate);
    }
    // A static body far away from every path in a colliding layer
    _ = try b.body(.{ .position = .{ 1000, 0, 1000 }, .shape = thin, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    b.at(20);
    try b.op(.set_motion_quality, 1, .{ .index = 0 });
    try b.op(.set_motion_quality, 2, .{ .index = 1 });
}

/// Every shape kind (mass properties, inertia, center of mass, inner radius of the linear cast)
fn buildShapes(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 40;
    var list: std.ArrayList(u32) = .empty;
    defer list.deinit(b.allocator);
    const sphere = try b.sphere(0.4);
    const box = try b.box(.{ 0.3, 0.6, 0.9 }, 0.1);
    try list.appendSlice(b.allocator, &.{ sphere, box });
    try list.append(b.allocator, try b.capsule(0.7, 0.25));
    try list.append(b.allocator, try b.taperedCapsule(0.5, 0.2, 0.4));
    try list.append(b.allocator, try b.cylinder(0.5, 0.3, 0.05));
    try list.append(b.allocator, try b.taperedCylinder(0.6, 0.1, 0.5, 0.05));
    try list.append(b.allocator, try b.convexHull(&.{ .{ -0.5, -0.4, -0.3 }, .{ 0.6, -0.4, -0.2 }, .{ 0.1, 0.7, 0.1 }, .{ 0.0, -0.1, 0.8 }, .{ 0.3, 0.3, -0.6 } }, 0.05));
    const dense = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.setDensity(dense, 7800.0);
    try list.append(b.allocator, dense);
    try list.append(b.allocator, try b.rotatedTranslated(box, .{ 0.2, -0.1, 0.4 }, axisAngle(.{ 0, 0, 1 }, 0.4)));
    try list.append(b.allocator, try b.scaled(box, .{ 1.5, 0.5, 2.0 }));
    const mirrored = try b.scaled(sphere, .{ -2.0, -2.0, -2.0 }); // Negative inner radius: discrete only (Jolt asserts for a linear cast)
    try list.append(b.allocator, mirrored);
    try list.append(b.allocator, try b.offsetCenterOfMass(sphere, .{ 0.1, 0.2, -0.3 }));
    try list.append(b.allocator, try b.compound(.static_compound, &.{ .{ .shape = box, .position = .{ 0, 1, 0 } }, .{ .shape = sphere, .position = .{ 0.5, -0.5, 0 }, .user_data = 7 } }));
    try list.append(b.allocator, try b.compound(.mutable_compound, &.{ .{ .shape = box, .rotation = axisAngle(.{ 1, 0, 0 }, 1.0) }, .{ .shape = sphere, .position = .{ -1, 0, 0 } } }));
    for (list.items, 0..) |shape, i| {
        for (0..2) |gyro| {
            _ = try b.body(.{
                .position = gridPosition(@intCast(2 * i + gyro), 0, 0),
                .rotation = axisAngle(.{ 0.48, 0.6, 0.64 }, 0.3 * @as(f64, @floatFromInt(i))),
                .linear_velocity = .{ 0, 0, -1 },
                .angular_velocity = .{ 2, 1, 0.5 },
                .apply_gyroscopic_force = @intCast(gyro),
                .shape = shape,
                .motion_quality = if (shape == mirrored) 0 else @intCast(gyro),
                .object_layer = Layers.unused1,
            }, .activate);
        }
    }
    // Static shapes that cannot be dynamic
    const plane = try b.plane(.{ 0, 1, 0 }, 0, 10);
    const triangle = try b.triangle(.{ 0, 0, 0, 1, 0, 0, 0, 0, 1 }, 0);
    const mesh = try b.mesh(&.{ .{ 0, 0, 0, 1, 0, 0, 0, 0, 1 }, .{ 1, 0, 0, 1, 0, 1, 0, 0, 1 } });
    const empty_shape = try b.empty(.{ 0, 0, 0 });
    for ([_]u32{ plane, triangle, mesh, empty_shape }, 0..) |shape, i| {
        _ = try b.body(.{ .position = gridPosition(@intCast(100 + i), 0, -50), .shape = shape, .motion_type = 0, .object_layer = Layers.unused1 }, .dont_activate);
    }
    // A kinematic mesh and a dynamic mesh with provided mass properties
    var inertia: [16]f32 = @splat(0);
    inertia[0] = 1;
    inertia[5] = 1;
    inertia[10] = 1;
    inertia[15] = 1;
    _ = try b.body(.{ .position = gridPosition(110, 0, -50), .shape = mesh, .motion_type = 1, .override_mass_properties = 2, .mass = 1, .inertia = inertia, .angular_velocity = .{ 0, 1, 0 }, .object_layer = Layers.unused1 }, .activate);
    _ = try b.body(.{ .position = gridPosition(111, 0, -50), .shape = mesh, .motion_type = 2, .override_mass_properties = 2, .mass = 2, .inertia = inertia, .angular_velocity = .{ 1, 1, 0 }, .object_layer = Layers.unused1 }, .activate);
}

/// SetMotionType and SetMotionQuality between updates
fn buildMotionChanges(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    b.num_updates = 60;
    for (0..8) |i| {
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 40, 50),
            .linear_velocity = .{ 0, -2, 0 },
            .shape = shapes.pick(@intCast(i)),
            .motion_type = @intCast(i % 3),
            .allow_dynamic_or_kinematic = 1,
            .object_layer = if (i % 2 == 0) Layers.moving else Layers.unused1,
        }, .activate);
    }
    b.at(10);
    try b.op(.set_motion_type, 0, .{ .index = 2, .activation = .activate }); // Static -> dynamic
    try b.op(.set_motion_type, 1, .{ .index = 2, .activation = .dont_activate }); // Kinematic -> dynamic
    try b.op(.set_motion_type, 2, .{ .index = 1, .activation = .activate }); // Dynamic -> kinematic
    try b.op(.set_motion_quality, 3, .{ .index = 1 });
    b.at(20);
    try b.op(.set_motion_type, 5, .{ .index = 0, .activation = .dont_activate }); // Dynamic -> static
    try b.op(.set_motion_type, 2, .{ .index = 2, .activation = .activate });
    try b.op(.set_motion_quality, 3, .{ .index = 0 });
    try b.op(.set_motion_quality, 4, .{ .index = 1 });
    b.at(30);
    try b.op(.set_motion_type, 5, .{ .index = 2, .activation = .activate });
    try b.op(.set_motion_type, 4, .{ .index = 1, .activation = .activate });
    try b.op(.move_kinematic, 4, .{ .r = gridPosition(4, 40, 30), .f = &(identity_rotation ++ [1]f32{0.5}) });
}

/// Random bodies and random operations (seed `seed`, `steps` collision steps)
fn buildRandom(comptime seed: u32, comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.rng = 0x12345 * seed + 77;
            b.collision_steps = steps;
            b.num_updates = 80;
            b.gravity = .{ b.float(-1, 1), b.float(-12, 0), b.float(-1, 1) };
            if (b.oneIn(2)) b.settings.allow_sleeping = 0;
            b.settings.time_before_sleep = b.float(0.1, 1.0);
            b.settings.point_velocity_sleep_threshold = b.float(0.01, 0.2);
            const shapes = try BasicShapes.add(b);
            const extra = [_]u32{
                try b.taperedCapsule(0.4, 0.1, 0.3),
                try b.convexHull(&.{ .{ -0.5, 0, 0 }, .{ 0.5, 0, 0 }, .{ 0, 0.8, 0 }, .{ 0, 0, 0.6 } }, 0.02),
                try b.scaled(shapes.box, .{ 0.5, 1.5, 1.0 }),
            };
            const num_bodies = 40;
            for (0..num_bodies) |i| {
                const motion_type: u32 = if (b.oneIn(8)) 1 else 2;
                const dofs: u32 = if (b.oneIn(3)) 1 + b.index(63) else 0b111111;
                const shape = if (b.oneIn(4)) extra[b.index(extra.len)] else shapes.pick(b.index(6));
                _ = try b.body(.{
                    .position = gridPosition(@intCast(i), 0, @as(Real, b.float(-10, 10))),
                    .rotation = b.rotation(),
                    .linear_velocity = b.vec3(-5, 5),
                    .angular_velocity = b.vec3(-3, 3),
                    .linear_damping = if (b.oneIn(3)) b.float(0, 2) else 0.05,
                    .angular_damping = if (b.oneIn(3)) b.float(0, 2) else 0.05,
                    .max_linear_velocity = if (b.oneIn(5)) b.float(9, 20) else 500.0, // |v| < 8.7: Jolt asserts that the initial velocity is within the maximum
                    .max_angular_velocity = if (b.oneIn(5)) b.float(5.5, 10) else 0.25 * std.math.pi * 60.0, // |w| < 5.2
                    .gravity_factor = if (b.oneIn(4)) 0.0 else b.float(-1, 2),
                    .allowed_dofs = if (motion_type == 2) dofs else 0b111111,
                    .apply_gyroscopic_force = b.index(2),
                    .motion_quality = b.index(2),
                    .allow_sleeping = if (b.oneIn(6)) 0 else 1,
                    .inertia_multiplier = if (b.oneIn(4)) b.float(0.5, 3) else 1.0,
                    .motion_type = motion_type,
                    .allow_dynamic_or_kinematic = b.index(2),
                    .shape = shape,
                    .object_layer = Layers.unused1 + b.index(5),
                    .user_data = b.next(),
                }, if (b.oneIn(5)) .dont_activate else .activate);
            }
            var u: u32 = 0;
            while (u < b.num_updates) : (u += 1 + b.index(3)) {
                b.at(u);
                const num_ops = b.index(4);
                for (0..num_ops) |_| {
                    const h = b.index(num_bodies);
                    const v = b.vec3(-5, 5);
                    const w = b.vec3(-2, 2);
                    const activation: SceneBuilder.Activation = if (b.oneIn(3)) .dont_activate else .activate;
                    switch (b.index(14)) {
                        0 => try b.op(.add_force, h, .{ .f = &.{ v[0] * 100, v[1] * 100, v[2] * 100 }, .activation = activation }),
                        1 => try b.op(.add_torque, h, .{ .f = &w, .activation = activation }),
                        2 => try b.op(.add_impulse, h, .{ .f = &v }),
                        3 => try b.op(.add_angular_impulse, h, .{ .f = &w }),
                        4 => try b.op(.set_linear_velocity, h, .{ .f = &v }),
                        5 => try b.op(.set_angular_velocity, h, .{ .f = &w }),
                        6 => try b.op(.activate_body, h, .{}),
                        7 => try b.op(.deactivate_body, h, .{}),
                        8 => try b.op(.set_gravity_factor, h, .{ .f = &.{b.float(-1, 2)} }),
                        9 => try b.op(.reset_sleep_timer, h, .{}),
                        10 => try b.op(.set_motion_quality, h, .{ .index = b.index(2) }),
                        11 => try b.op(.add_force_at_position, h, .{ .f = &v, .r = gridPositionOffset(gridPosition(h, 0, 0), w), .activation = activation }),
                        12 => try b.op(.add_linear_and_angular_velocity, h, .{ .f = &(v ++ w) }),
                        else => try b.op(.set_max_angular_velocity, h, .{ .f = &.{b.float(0.5, 20)} }),
                    }
                }
                if (b.oneIn(15)) try b.sysOp(.set_gravity, .{ .f = &b.vec3(-10, 10) });
            }
        }
    }.build;
}

/// Linear cast bodies whose shape has no inner radius (a mesh, an empty shape) or a negative one (a mirrored
/// ScaledShape): Jolt asserts, its release build casts every movement (Blocker.release_only)
fn buildLinearCastWithoutInnerRadius(b: *SceneBuilder) Allocator.Error!void {
    const mesh = try b.mesh(&.{ .{ 0, 0, 0, 1, 0, 0, 0, 0, 1 }, .{ 1, 0, 0, 1, 0, 1, 0, 0, 1 } });
    const empty_shape = try b.empty(.{ 0.1, 0, 0 });
    const sphere = try b.sphere(0.3);
    const mirrored = try b.scaled(sphere, .{ -1.0, -1.0, -1.0 });
    const mirrored_box = try b.scaled(try b.box(.{ 0.3, 0.2, 0.4 }, 0.05), .{ 1.0, -1.0, 1.0 });
    b.num_updates = 30;
    b.collision_steps = 2;
    var inertia: [16]f32 = @splat(0);
    inertia[0] = 1;
    inertia[5] = 1;
    inertia[10] = 1;
    inertia[15] = 1;
    for ([_]u32{ mesh, empty_shape, mirrored, mirrored_box }, 0..) |shape, i| {
        for (0..2) |j| {
            _ = try b.body(.{
                .position = gridPosition(@intCast(2 * i + j), 40, 100),
                .linear_velocity = .{ 0, -20.0 * @as(f32, @floatFromInt(j)), 0 },
                .angular_velocity = .{ 1, 0, 0 },
                .shape = shape,
                .motion_quality = 1,
                .override_mass_properties = 2,
                .mass = 2,
                .inertia = inertia,
                .object_layer = if (j == 0) Layers.moving else Layers.unused1,
            }, .activate);
        }
    }
}

// -------------------------------------------------------------------------------------------------------------------
// Scenes of the later waves

/// A box falling on a static floor (contacts, sleeping on the floor)
fn buildBoxOnFloor(b: *SceneBuilder) Allocator.Error!void {
    const floor = try b.box(.{ 100, 1, 100 }, 0);
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    b.num_updates = 120;
    _ = try b.body(.{ .position = .{ 0, -1, 0 }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ 0, 2, 0 }, .rotation = axisAngle(.{ 1, 0, 0 }, 0.3), .shape = box, .object_layer = Layers.moving }, .activate);
}

/// A small pyramid of boxes on a floor (islands, the contact cache, the large island splitter)
fn buildPyramid(b: *SceneBuilder) Allocator.Error!void {
    const floor = try b.box(.{ 50, 1, 50 }, 0);
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    b.num_updates = 100;
    _ = try b.body(.{ .position = .{ 0, -1, 0 }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const height = 5;
    for (0..height) |y| {
        for (0..height - y) |x| {
            const px: Real = @as(Real, @floatFromInt(x)) * 1.05 + @as(Real, @floatFromInt(y)) * 0.525 - 2.5;
            _ = try b.body(.{ .position = .{ px, 0.5 + @as(Real, @floatFromInt(y)), 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
        }
    }
}

/// A sensor that a body falls through
fn buildSensor(b: *SceneBuilder) Allocator.Error!void {
    const sensor = try b.box(.{ 2, 0.5, 2 }, 0);
    const sphere = try b.sphere(0.5);
    b.num_updates = 60;
    _ = try b.body(.{ .shape = sensor, .motion_type = 0, .is_sensor = 1, .object_layer = Layers.sensor }, .dont_activate);
    _ = try b.body(.{ .position = .{ 0, 3, 0 }, .shape = sphere, .object_layer = Layers.moving }, .activate);
}

/// A linear cast bullet that hits a thin wall
fn buildBulletHitsWall(b: *SceneBuilder) Allocator.Error!void {
    const wall = try b.box(.{ 5, 5, 0.05 }, 0);
    const bullet = try b.sphere(0.05);
    b.num_updates = 30;
    b.gravity = .{ 0, 0, 0 };
    _ = try b.body(.{ .position = .{ 0, 0, 5 }, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .linear_velocity = .{ 0, 0, 200 }, .shape = bullet, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
}

/// A chain of boxes hanging from the world with point constraints
fn buildConstraintChain(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.5, 0.2 }, 0.05);
    b.num_updates = 60;
    var previous: u32 = invalid_handle;
    for (0..5) |i| {
        const y: f32 = -@as(f32, @floatFromInt(i));
        const h = try b.body(.{ .position = .{ 0.3 * @as(Real, @floatFromInt(i)), y, 0 }, .shape = box, .object_layer = Layers.unused1 }, .activate);
        const c = try b.addConstraint(.{ .kind = @intFromEnum(ConstraintKind.point), .body1 = previous, .body2 = h, .f = .{ 0.3 * @as(f32, @floatFromInt(i)), y + 0.5, 0, 0.3 * @as(f32, @floatFromInt(i)), y + 0.5, 0 } ++ @as([10]f32, @splat(0)) });
        try b.sysOp(.create_constraint, .{ .index = c });
        previous = h;
    }
}

test "SimScenes builders" {
    // Every scene builds and its operations are in update order
    for (scenes) |entry| {
        var b: SceneBuilder = .init(std.testing.allocator);
        defer b.deinit();
        try entry.build(&b);
        var last: u32 = 0;
        for (b.ops.items) |o| {
            try std.testing.expect(o.update >= last and o.update < b.num_updates);
            last = o.update;
        }
    }
}
