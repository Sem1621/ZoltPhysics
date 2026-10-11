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
//!    other operation (see `OpKind` for the fields each kind reads). Where a scene is written for a body state (an
//!    operation on a sleeping body), say so with `b.expectSleeping(handle)` / `b.expectAwake(handle)`: the driver
//!    checks it on both sides, a comment alone becomes false unnoticed when the scene changes.
//! 2. Add one entry to `scenes` with its name and builder. Every entry is its own test (`SceneTest("name", index)`,
//!    run one with `zig build parity -Dtest-filter=name`). A scene that Zolt cannot run yet gets
//!    `.blocked_by = .contacts` (or `.constraints`, `.ccd_hits`, ...): its test returns error.SkipZigTest until
//!    `Blocker.isSupported` says so (flip it when the wave lands), so the build summary counts it as "skipped", never
//!    silently. Meanwhile the test "SimScenes blocked scenes run on the C++ side" runs it on the C++ side with 0 and
//!    3 worker threads and checks that it reaches what it is written for (contact callbacks, constraints).
//!    `.blocked_by = .release_only` marks a scene that violates a JPH_ASSERT that Jolt's release build ignores: it runs
//!    without Zolt's asserts only (ReleaseFast).
//! 3. A scene with contacts needs nothing else (the C++ side and the Zolt side create every shape kind of `ShapeKind`,
//!    and the contact listener records are compared already). A new shape kind is a new `ShapeKind` value plus its
//!    creation in `SimulationReference.cpp` (MakeShape) and `SimulationParity.zig` (makeShapeSettings). Constraints
//!    are described by `SimConstraint`; the C++ side creates every `ConstraintKind`, the Zolt side gets the
//!    corresponding prong in SimulationParity.zig's `applyOpInner` (`.create_constraint`) when Phase 6 ports the class.
//!    A new operation is a new `OpKind` value plus its prong in ApplyOp (C++) and applyOpInner (Zolt).
//! 4. Avoid what Jolt asserts (Zolt's Debug build asserts the same): initial velocities above the maximum, a linear
//!    cast shape without inner radius, invalid scales, a kinematic / dynamic body without mass (a mesh needs
//!    `override_mass_properties`), setting the velocity of a static body.
//!
//! Rules for the gravity-only scenes: no two bodies whose layers collide may ever have overlapping bounds, the linear
//! cast bodies must not find a body in their path. Bodies in the `Layers.unused*` layers collide with nothing, the
//! other layers are spread out on a grid (`gridPosition`).
//!
//! Contact scenes (Wave B, `Blocker.contacts` is supported): bodies collide, the contact listener records are
//! compared. The contact listener, the combine functions and the SimShapeFilter of a scene are selected with
//! `b.contact_listener_mode` / `b.combine_mode` / `b.sim_shape_filter_mode`; in `ContactListenerMode.modify` and
//! `SimShapeFilterMode.filter_spheres` the bits of `ContactFlags` in the upper 32 bits of the user data of the two
//! bodies decide what the listener / filter does (the same code on both sides). A contact scene states what it
//! reaches in `SceneEntry.expect` (contact callbacks, removed contacts, sleeping bodies, FindCollisions jobs spawned
//! from inside jobs, a large island solved by the LargeIslandSplitter, CCD hits, several FindCCDContacts jobs);
//! `runEntry` checks it. Rule: the contact constraint buffer and the caches must be large enough
//! (`max_contact_constraints`, `max_body_pairs`): which contacts are dropped when they are full depends on the threads,
//! in Jolt too.
//!
//! CCD scenes (Wave B R4, `Blocker.ccd_hits` is supported): linear cast bodies hit things (JobFindCCDContacts,
//! sSolveCCDContact, JobResolveCCDContacts). Rules: a linear cast shape needs an inner radius (no mesh, height field,
//! plane or mirrored ScaledShape), no body may move faster than its max linear velocity (500 m/s by default), and at
//! most 64 sleeping bodies may be woken up by CCD hits in one collision step (Jolt then activates a batch while it
//! still resolves the CCD bodies, and a later CCD body that hit one of them reads past mActiveBodyToCCDBody).

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
    /// `count` * `count` height samples at floats[data..], params[0..3] offset, params[3..6] scale
    height_field = 16,
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
/// kind. Every operation returns a u32 that both sides must agree on: a body ID (the create operations), the
/// `BodyState` bits IsActive(body) | IsAdded(body) << 1 after a body operation (ApplyBuoyancyImpulse adds its result
/// as bit 2), or 0 (the other operations).
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
    /// SetShape(shapes[index], update mass properties = count != 0, activation)
    set_shape = 45,
    /// ActivateBodiesInAABox(AABox(f[0..3], f[3..6]), default filters)
    activate_bodies_in_aabox = 46,
    /// ApplyBuoyancyImpulse(surface position r, normal f[0..3], buoyancy f[3], linear drag f[4], angular drag f[5],
    /// fluid velocity f[6..9], gravity f[9..12], delta time f[12]); bit 2 of the result is its return value
    apply_buoyancy_impulse = 47,
    /// A check of the scene itself: changes nothing and returns the `BodyState` bits like every body operation, the
    /// driver fails unless they are `index` (`SceneBuilder.expectSleeping` / `expectAwake`). A scene states with it
    /// that it reaches the edge case it is written for (a sleeping body before an operation that must not wake it).
    expect_body_state = 48,
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
    f: [16]f32 = @splat(0),
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
    /// ContactListenerMode
    contact_listener_mode: u32,
    /// CombineMode (PhysicsSystem::SetCombineFriction / SetCombineRestitution)
    combine_mode: u32,
    /// SimShapeFilterMode (PhysicsSystem::SetSimShapeFilter)
    sim_shape_filter_mode: u32,
};

/// What the contact listener does besides recording the callbacks
pub const ContactListenerMode = enum(u32) {
    /// Record the callbacks, OnContactValidate returns AcceptAllContactsForThisBodyPair
    record = 0,
    /// Record the callbacks (the settings as Jolt passes them in) and then act on the `ContactFlags` of the two bodies
    modify = 1,
};

/// The combine functions of friction and restitution
pub const CombineMode = enum(u32) {
    /// Jolt's defaults: sqrt(friction1 * friction2), max(restitution1, restitution2)
    default = 0,
    /// friction 0.5 * (friction1 + friction2), restitution max(restitution1, restitution2) when both sub shape IDs are
    /// equal, else min(restitution1, restitution2)
    custom = 1,
};

/// The SimShapeFilter of the system
pub const SimShapeFilterMode = enum(u32) {
    /// No filter (nullptr)
    none = 0,
    /// ShouldCollide is false when one of the two bodies has `ContactFlags.filter_spheres` and one of the two shapes is
    /// a SphereShape (it is called for every level of a compound)
    filter_spheres = 1,
};

/// Bits in the upper 32 bits of a body's user data (`user_data_flags << 32`) that the contact listener in
/// `ContactListenerMode.modify` and the shape filter act on (the bits of both bodies are combined with or)
pub const ContactFlags = struct {
    /// OnContactValidate: RejectAllContactsForThisBodyPair
    pub const reject_all: u32 = 1 << 0;
    /// OnContactValidate: RejectContact when the penetration depth is negative (speculative), else AcceptContact
    pub const reject_speculative: u32 = 1 << 1;
    /// OnContactValidate: AcceptContact (it is called again for the next contact of the pair)
    pub const accept_each: u32 = 1 << 2;
    /// OnContactAdded / OnContactPersisted: mCombinedFriction = 0
    pub const zero_friction: u32 = 1 << 3;
    /// OnContactAdded / OnContactPersisted: mCombinedRestitution = 0.9
    pub const bouncy: u32 = 1 << 4;
    /// OnContactAdded / OnContactPersisted: inverse mass scales 0.5 / 2.0, inverse inertia scales 0.25 / 1.5
    pub const mass_scale: u32 = 1 << 5;
    /// OnContactAdded / OnContactPersisted: relative surface velocity (1.5, 0, 0.5), angular (0, 0.7, 0)
    pub const surface_velocity: u32 = 1 << 6;
    /// OnContactAdded / OnContactPersisted: mIsSensor = true
    pub const sensor: u32 = 1 << 7;
    /// SimShapeFilterMode.filter_spheres: filter the spheres of the pair
    pub const filter_spheres: u32 = 1 << 8;

    /// The user data of a body with these flags (and `low` in the lower 32 bits)
    pub fn userData(flags: u32, low: u32) u64 {
        return (@as(u64, flags) << 32) | low;
    }
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
    /// number of contact points | is sensor << 8, data[5] combined friction, data[6] combined restitution, data[7] the
    /// FNV-1a hash of the base offset and the contact points on both shapes (`hashManifold`)
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

/// FNV-1a (32 bit) over bytes, the hash of the contact listener records (SimHashBytes in SimulationReference.cpp)
pub fn hashBytes(hash: u32, bytes: []const u8) u32 {
    var h = hash;
    for (bytes) |c| {
        h ^= c;
        h *%= 16777619;
    }
    return h;
}

/// The FNV-1a offset basis
pub const hash_seed: u32 = 2166136261;

/// The result bits of a body operation: IsActive(body) | IsAdded(body) << 1
pub const BodyState = struct {
    pub const active: u32 = 1;
    pub const added: u32 = 2;
    /// In the broad phase and asleep
    pub const sleeping: u32 = added;
    /// In the broad phase and awake
    pub const awake: u32 = added | active;
};

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
    /// Body pairs reach the narrow phase: ProcessBodyPair and the contact solver (A4, Wave B R2 / R3, supported)
    contacts,
    /// Constraints in islands: the constraint classes (Phase 6) and the solver (Wave B R3)
    constraints,
    /// Linear cast bodies that hit something (Wave B R4, supported)
    ccd_hits,
    /// Soft bodies (Phase 9)
    soft_bodies,
    /// The scene violates a JPH_ASSERT that Jolt's release build (the reference library is built without asserts)
    /// ignores: it compares Jolt's release behavior and runs only without Zolt's asserts (ReleaseFast)
    release_only,

    /// True when Zolt can run the scenes with this blocker (flip when the wave lands)
    pub fn isSupported(self: Blocker) bool {
        return switch (self) {
            .none, .contacts, .ccd_hits => true,
            .release_only => !zolt.Core.enable_asserts,
            .constraints, .soft_bodies => false,
        };
    }
};

/// What a scene must reach (checked by SimulationParity.zig's runEntry, so that a scene that no longer does what it is
/// written for fails)
pub const Expect = struct {
    /// OnContactAdded is called (in every configuration)
    contacts: bool = false,
    /// OnContactRemoved is called
    removed: bool = false,
    /// OnContactValidate is called (a scene with ContactListenerMode.modify that rejects contacts)
    validated: bool = false,
    /// Bodies are asleep after some update
    sleeping: bool = false,
    /// FindCollisions jobs are spawned from inside jobs (TrySpawnJobFindCollisions) in a multi threaded configuration
    spawned_jobs: bool = false,
    /// Bodies are put to sleep from the final batch of a split island (Zolt side: the LargeIslandSplitter has split
    /// islands when OnBodyDeactivated is called from SolvePositionConstraints)
    large_island: bool = false,
    /// A linear cast body hits something: the contact cache of the (C++) state holds CCD manifolds after some update
    /// (ContactConstraintManager::OnCCDContactAdded, the worlds have a contact listener)
    ccd_hits: bool = false,
    /// At least this many FindCCDContacts jobs run in one update (in some configuration: their number is
    /// min(ceil(CCD bodies / 4), the concurrency of the job system))
    ccd_jobs: u32 = 0,
};

pub const SceneEntry = struct {
    name: []const u8,
    build: *const fn (b: *SceneBuilder) Allocator.Error!void,
    blocked_by: Blocker = .none,
    /// D12 sweeps (every temp allocation and every createJob of the update fails once) before every
    /// `sweep_stride`-th update and before the first one, 0: none
    sweep_stride: u32 = 25,
    expect: Expect = .{},
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
    .{ .name = "shape_changes_buoyancy_and_box_activation", .build = buildShapeChanges },
    .{ .name = "random_1", .build = buildRandom(1, 1) },
    .{ .name = "random_2", .build = buildRandom(2, 2) },
    .{ .name = "random_3", .build = buildRandom(3, 3) },
    .{ .name = "random_4", .build = buildRandom(4, 4) },
    .{ .name = "linear_cast_without_inner_radius", .build = buildLinearCastWithoutInnerRadius, .blocked_by = .release_only },
    // Contacts (Wave B R2 / R3)
    .{ .name = "contacts_box_on_floor", .build = buildBoxOnFloor, .blocked_by = .contacts, .expect = .{ .contacts = true, .sleeping = true } },
    .{ .name = "contacts_pyramid", .build = buildPyramid, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_sensor", .build = buildSensor, .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_large_pyramid_splitter", .build = buildLargePyramid, .blocked_by = .contacts, .sweep_stride = 45, .expect = .{ .contacts = true, .sleeping = true, .large_island = true } },
    .{ .name = "contacts_large_pyramid_collapse", .build = buildLargePyramidWithoutSplitter, .blocked_by = .contacts, .sweep_stride = 30, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_shapes_on_floor", .build = buildShapesOnFloor, .blocked_by = .contacts, .expect = .{ .contacts = true, .sleeping = true } },
    .{ .name = "contacts_shape_stacks", .build = buildShapeStacks, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_mesh_ground", .build = buildMeshGround, .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_height_field_ground", .build = buildHeightFieldGround, .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_static_shapes", .build = buildStaticShapes, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_friction_restitution", .build = buildFrictionRestitution, .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_combine_functions", .build = buildCombineFunctions, .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_sensors", .build = buildSensors, .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true, .sleeping = true } },
    .{ .name = "contacts_kinematic_push", .build = buildKinematicPush, .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_sleep_and_wake_by_hit", .build = buildSleepAndWakeByHit, .blocked_by = .contacts, .expect = .{ .contacts = true, .sleeping = true } },
    .{ .name = "contacts_listener_modify", .build = buildListenerModify, .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_listener_validate", .build = buildListenerValidate, .blocked_by = .contacts, .expect = .{ .contacts = true, .validated = true } },
    .{ .name = "contacts_without_manifold_reduction", .build = buildWithoutManifoldReduction, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_body_without_manifold_reduction", .build = buildBodyWithoutManifoldReduction, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_enhanced_internal_edge_removal", .build = buildEnhancedInternalEdgeRemoval, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_sim_shape_filter", .build = buildSimShapeFilter, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_spawned_find_collisions", .build = buildSpawnedFindCollisions, .blocked_by = .contacts, .expect = .{ .contacts = true, .spawned_jobs = true } },
    .{ .name = "contacts_many_bodies", .build = buildManyBodies, .blocked_by = .contacts, .sweep_stride = 30, .expect = .{ .contacts = true, .removed = true, .spawned_jobs = true } },
    .{ .name = "contacts_collision_steps_2", .build = buildContactCollisionSteps(2), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_collision_steps_3", .build = buildContactCollisionSteps(3), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_save_restore_cross", .build = buildContactSaveRestore, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_solver_settings", .build = buildSolverSettings, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_body_changes", .build = buildBodyChangesInContact, .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_allowed_dofs", .build = buildContactAllowedDOFs, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_far_from_origin", .build = buildFarFromOrigin, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_kinematic_vs_non_dynamic", .build = buildKinematicVsNonDynamic, .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_deep_penetration", .build = buildDeepPenetration, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_random_1", .build = buildContactRandom(1, 1), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_random_2", .build = buildContactRandom(2, 2), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_random_3", .build = buildContactRandom(3, 1), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_random_4", .build = buildContactRandom(4, 3), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    // CCD (Wave B R4: linear cast bodies that hit something)
    .{ .name = "ccd_bullet_hits_wall", .build = buildBulletHitsWall, .blocked_by = .ccd_hits, .sweep_stride = 1, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_static_targets", .build = buildCCDStaticTargets, .blocked_by = .ccd_hits, .sweep_stride = 3, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_bodies_hit_each_other", .build = buildCCDBodiesHitEachOther, .blocked_by = .ccd_hits, .sweep_stride = 1, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_kinematic_and_dynamic", .build = buildCCDKinematicAndDynamic, .blocked_by = .ccd_hits, .sweep_stride = 1, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_restitution_friction", .build = buildCCDRestitutionFriction, .blocked_by = .ccd_hits, .sweep_stride = 2, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_combine_custom", .build = buildCCDCombineCustom, .blocked_by = .ccd_hits, .sweep_stride = 2, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_listener", .build = buildCCDListener, .blocked_by = .ccd_hits, .sweep_stride = 3, .expect = .{ .contacts = true, .validated = true, .ccd_hits = true } },
    .{ .name = "ccd_starts_penetrating", .build = buildCCDStartsPenetrating, .blocked_by = .ccd_hits, .sweep_stride = 1, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_many_bodies", .build = buildCCDManyBodies, .blocked_by = .ccd_hits, .sweep_stride = 5, .expect = .{ .contacts = true, .ccd_hits = true, .ccd_jobs = 8 } },
    .{ .name = "ccd_sensors", .build = buildCCDSensors, .blocked_by = .ccd_hits, .sweep_stride = 2, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_terrain", .build = buildCCDTerrain, .blocked_by = .ccd_hits, .sweep_stride = 3, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_collision_steps_2", .build = buildCCDCollisionSteps(2), .blocked_by = .ccd_hits, .sweep_stride = 4, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_collision_steps_3", .build = buildCCDCollisionSteps(3), .blocked_by = .ccd_hits, .sweep_stride = 4, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_save_restore_cross", .build = buildCCDSaveRestore, .blocked_by = .ccd_hits, .sweep_stride = 3, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_far_from_origin", .build = buildCCDFarFromOrigin, .blocked_by = .ccd_hits, .sweep_stride = 2, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_random_1", .build = buildCCDRandom(1, 1), .blocked_by = .ccd_hits, .sweep_stride = 5, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_random_2", .build = buildCCDRandom(2, 2), .blocked_by = .ccd_hits, .sweep_stride = 5, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_random_3", .build = buildCCDRandom(3, 1), .blocked_by = .ccd_hits, .sweep_stride = 5, .expect = .{ .contacts = true, .ccd_hits = true } },
    // Later waves (blocked until Zolt has the parts, see Blocker)
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
    contact_listener_mode: ContactListenerMode = .record,
    combine_mode: CombineMode = .default,
    sim_shape_filter_mode: SimShapeFilterMode = .none,

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
            .contact_listener_mode = @intFromEnum(self.contact_listener_mode),
            .combine_mode = @intFromEnum(self.combine_mode),
            .sim_shape_filter_mode = @intFromEnum(self.sim_shape_filter_mode),
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

    /// A height field of `sample_count` * `sample_count` samples (row major, `samples.len == sample_count^2`)
    pub fn heightField(self: *SceneBuilder, samples: []const f32, sample_count: u32, offset: [3]f32, scale: [3]f32) Allocator.Error!u32 {
        std.debug.assert(samples.len == @as(usize, sample_count) * sample_count);
        const data: u32 = @intCast(self.floats.items.len);
        try self.floats.appendSlice(self.allocator, samples);
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.height_field), .params = params(&.{ offset[0], offset[1], offset[2], scale[0], scale[1], scale[2] }), .count = sample_count, .data = data });
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

    /// The body must be asleep at this point (in the broad phase, not active): the driver checks it on both sides
    pub fn expectSleeping(self: *SceneBuilder, handle: u32) Allocator.Error!void {
        return self.op(.expect_body_state, handle, .{ .index = BodyState.sleeping });
    }

    /// The body must be awake at this point (in the broad phase and active): the driver checks it on both sides
    pub fn expectAwake(self: *SceneBuilder, handle: u32) Allocator.Error!void {
        return self.op(.expect_body_state, handle, .{ .index = BodyState.awake });
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
    b.at(99);
    for (0..num_bodies) |i| try b.expectSleeping(@intCast(i));
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
    // A body without velocity and gravity that the loop below does not touch: it falls asleep
    const sleeper = try b.body(.{ .position = gridPosition(num_bodies, 0, 0), .shape = shapes.capsule, .gravity_factor = 0.0, .object_layer = Layers.unused1 }, .activate);
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
    // Forces twice in a row (they accumulate) and on an awake body without activation (applied)
    b.at(80);
    try b.op(.add_force, 0, .{ .f = &.{ 1, 2, 3 } });
    try b.op(.add_force, 0, .{ .f = &.{ 4, 5, 6 } });
    try b.expectAwake(1);
    try b.op(.add_torque, 1, .{ .f = &.{ 1, 2, 3 }, .activation = .dont_activate });
    // Forces and torques on a sleeping body without activation are dropped, the body stays asleep
    try b.expectSleeping(sleeper);
    try b.op(.add_force, sleeper, .{ .f = &.{ 10, 20, 30 }, .activation = .dont_activate });
    try b.op(.add_force_at_position, sleeper, .{ .f = &.{ 0, 50, 10 }, .r = gridPositionOffset(gridPosition(num_bodies, 0, 0), .{ 0.4, 0.1, -0.3 }), .activation = .dont_activate });
    try b.op(.add_torque, sleeper, .{ .f = &.{ 1, 2, 3 }, .activation = .dont_activate });
    try b.op(.add_force_and_torque, sleeper, .{ .f = &.{ -10, 0, 7, 0, 3, 1 }, .activation = .dont_activate });
    try b.expectSleeping(sleeper);
    // An impulse wakes it up (without the dropped forces)
    b.at(85);
    try b.expectSleeping(sleeper);
    try b.op(.add_impulse, sleeper, .{ .f = &.{ 0.5, 0, 0 } });
    try b.expectAwake(sleeper);
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
    // A body without velocity and gravity that the loop below does not touch: it falls asleep
    const sleeper = try b.body(.{ .position = gridPosition(21, 30, 2), .shape = shapes.compound, .gravity_factor = 0.0, .object_layer = Layers.unused1 }, .activate);
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
    // Setting a zero or near zero velocity (IsNearZero: length squared <= 1.0e-12) does not wake a sleeping body, the
    // near zero velocity is stored; moving it with SetPositionRotationAndVelocity and zero velocities neither
    b.at(55);
    try b.expectSleeping(sleeper);
    try b.op(.set_linear_velocity, sleeper, .{ .f = &.{ 0, 0, 0 } });
    try b.op(.set_angular_velocity, sleeper, .{ .f = &.{ 0, 0, 0 } });
    try b.op(.set_linear_and_angular_velocity, sleeper, .{ .f = &.{ 0, 0, 0, 0, 0, 0 } });
    try b.op(.set_position_rotation_and_velocity, sleeper, .{ .r = gridPosition(21, 30, 2.5), .f = &(axisAngle(.{ 0, 1, 0 }, 0.25) ++ [6]f32{ 0, 0, 0, 0, 0, 0 }) });
    try b.op(.set_linear_velocity, sleeper, .{ .f = &.{ 2.0e-7, 0, -3.0e-7 } });
    try b.op(.add_linear_velocity, sleeper, .{ .f = &.{ 1.0e-7, 1.0e-7, 0 } });
    try b.op(.set_angular_velocity, sleeper, .{ .f = &.{ 0, 4.0e-7, 0 } });
    try b.op(.add_linear_and_angular_velocity, sleeper, .{ .f = &.{ 0, 0, 1.0e-7, -1.0e-7, 0, 1.0e-7 } });
    try b.expectSleeping(sleeper);
    // Activated with its near zero velocity
    b.at(57);
    try b.expectSleeping(sleeper);
    try b.op(.activate_body, sleeper, .{});
    try b.expectAwake(sleeper);
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
    // A body without velocity that only the operations on sleeping bodies below touch
    const sleeper = try b.body(.{ .position = gridPosition(num_bodies, 0, 0), .shape = shapes.box, .gravity_factor = 0.0, .object_layer = Layers.unused1, .user_data = 0xdead_0000_0000 + num_bodies }, .activate);
    // Bodies 0..3 are asleep, wake them up in different ways (4.. are still awake)
    b.at(70);
    for (0..4) |h| try b.expectSleeping(@intCast(h));
    try b.op(.activate_body, 0, .{});
    try b.op(.add_force, 1, .{ .f = &.{ 10, 0, 0 } });
    try b.op(.add_impulse, 2, .{ .f = &.{ 0, 1, 0 } });
    try b.op(.set_linear_velocity, 3, .{ .f = &.{ 0.5, 0, 0 } });
    for (0..4) |h| try b.expectAwake(@intCast(h));
    try b.expectAwake(4);
    try b.op(.activate_bodies, 4, .{ .count = 3 }); // Awake: changes nothing
    try b.op(.reset_sleep_timer, 8, .{});
    try b.expectAwake(9);
    try b.op(.add_force, 9, .{ .f = &.{ 10, 0, 0 }, .activation = .dont_activate }); // Awake: the force is applied
    // Asleep: the force is dropped, the body stays asleep
    try b.expectSleeping(sleeper);
    try b.op(.add_force, sleeper, .{ .f = &.{ 10, 0, 0 }, .activation = .dont_activate });
    try b.expectSleeping(sleeper);
    b.at(80);
    try b.op(.deactivate_body, 10, .{});
    try b.op(.deactivate_bodies, 0, .{ .count = 2 });
    try b.op(.activate_body, 9, .{}); // Awake: changes nothing
    // Woken up without the dropped force
    try b.expectSleeping(sleeper);
    try b.op(.activate_body, sleeper, .{});
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

/// The same scene with 1..4 collision steps per update. Jolt checks sleeping in the last step only and accumulates
/// the step delta time (CheckSleepAndUpdateBounds): with the default time_before_sleep of 0.5 s a body needs 30 updates
/// per collision step to fall asleep, with 0.1 s at most 24 updates.
fn buildCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const shapes = try BasicShapes.add(b);
            b.collision_steps = steps;
            b.num_updates = 50;
            b.settings.time_before_sleep = 0.1; // Max movement 0.03 * 0.1 = 0.003 m
            for (0..24) |i| {
                _ = try b.body(.{
                    .position = gridPosition(@intCast(i), 20, 10),
                    .rotation = axisAngle(.{ 0, 1, 0 }, @floatFromInt(i)),
                    // The bodies i % 5 == 0 move slowly without gravity: they fall asleep
                    .linear_velocity = if (i % 5 == 0) .{ 0, 0.005, 0 } else .{ 0, 5, 0 },
                    .angular_velocity = if (i % 5 == 0) .{ 0.002, 0.001 * @as(f32, @floatFromInt(i % 3)), 0 } else .{ 0.5, 0.2 * @as(f32, @floatFromInt(i % 3)), 0 },
                    .shape = shapes.pick(@intCast(i)),
                    .apply_gyroscopic_force = @intCast(i % 2),
                    .motion_type = if (i % 8 == 7) 1 else 2,
                    .motion_quality = @intCast((i / 2) % 2),
                    .gravity_factor = if (i % 5 == 0) 0.0 else 1.0,
                    .linear_damping = if (i % 5 == 0) 0.9 else 0.05,
                    .angular_damping = if (i % 5 == 0) 0.9 else 0.05,
                    .object_layer = if (i % 2 == 0) Layers.moving else Layers.unused1,
                }, .activate);
            }
            b.at(25);
            try b.op(.add_impulse, 3, .{ .f = &.{ 5, 0, 0 } });
            b.at(40);
            for ([_]u32{ 0, 5, 10, 15, 20 }) |h| try b.expectSleeping(h);
            try b.op(.activate_body, 0, .{});
            try b.op(.add_impulse, 5, .{ .f = &.{ 0, 0, 1 } });
            try b.expectAwake(0);
            try b.expectAwake(5);
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
    try b.expectSleeping(23);
    try b.sysOp(.add_step_listener, .{ .index = waker });
    b.at(31);
    try b.expectAwake(23);
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
    try b.expectSleeping(0);
    try b.sysOp(.add_step_listener, .{ .index = waker });
    b.at(11);
    try b.expectAwake(0);
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
    try b.expectSleeping(0);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Back to update 10 (the gravity is restored too): the sleeping bodies wake up
    try b.expectAwake(0);
    b.at(70);
    try b.sysOp(.snapshot, .{ .index = 1 });
    try b.expectSleeping(11);
    try b.op(.add_force, 11, .{ .f = &.{ 0, 100, 0 } });
    b.at(80);
    try b.expectAwake(11);
    try b.sysOp(.restore_cross, .{ .index = 1 }); // Body 11 is asleep again
    try b.expectSleeping(11);
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

/// SetShape with and without updating the mass properties, ApplyBuoyancyImpulse, ActivateBodiesInAABox
fn buildShapeChanges(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BasicShapes.add(b);
    const big_box = try b.box(.{ 1.0, 0.3, 0.6 }, 0.1);
    b.num_updates = 60;
    for (0..8) |i| {
        _ = try b.body(.{
            .position = gridPosition(@intCast(i), 30, 0),
            .rotation = axisAngle(.{ 0, 0, 1 }, 0.2 * @as(f64, @floatFromInt(i))),
            .angular_velocity = if (i >= 6) @splat(0) else .{ 0.5, 0, 0 }, // 6 and 7 fall asleep
            .shape = shapes.pick(@intCast(i)),
            .gravity_factor = if (i >= 6) 0.0 else 1.0,
            .linear_damping = 2.0,
            .angular_damping = 2.0,
            .motion_type = if (i == 5) 1 else 2,
            .object_layer = if (i % 2 == 0) Layers.moving else Layers.unused1,
        }, .activate);
    }
    b.at(5);
    try b.op(.set_shape, 0, .{ .index = big_box, .count = 1 });
    try b.op(.set_shape, 1, .{ .index = shapes.compound, .count = 0 });
    try b.op(.set_shape, 5, .{ .index = shapes.capsule, .count = 1 }); // Kinematic
    var u: u32 = 10;
    while (u < 40) : (u += 3) {
        b.at(u);
        // Body 2 is under water (the impulse is applied), body 3 above it (it is not)
        try b.op(.apply_buoyancy_impulse, 2, .{ .r = gridPosition(2, 30, 100), .f = &.{ 0, 1, 0, 1.1, 0.3, 0.05, 1, 0, 0, 0, -9.81, 0, b.delta_time } });
        try b.op(.apply_buoyancy_impulse, 3, .{ .r = gridPosition(3, 30, -100), .f = &.{ 0.6, 0.8, 0, 2.0, 0.5, 0.5, 0, 0, -1, 0, -9.81, 0, b.delta_time } });
    }
    // Bodies 6 and 7 are asleep by now, the box wakes 6 up (and nothing else: the other bodies are far away)
    b.at(45);
    try b.expectSleeping(6);
    try b.expectSleeping(7);
    const p = gridPosition(6, 30, 0);
    try b.sysOp(.activate_bodies_in_aabox, .{ .f = &.{ @floatCast(p[0] - 1), @floatCast(p[1] - 1), @floatCast(p[2] - 1), @floatCast(p[0] + 1), @floatCast(p[1] + 1), @floatCast(p[2] + 1) } });
    try b.expectAwake(6);
    try b.expectSleeping(7);
    b.at(50);
    try b.op(.set_shape, 7, .{ .index = shapes.sphere, .count = 1, .activation = .activate });
    try b.expectAwake(7);
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
/// ScaledShape): Jolt asserts (Blocker.release_only), its release build compares the squared movement of a step with
/// Square(linear_cast_threshold * inner_radius). A zero radius casts every movement, a negative radius r every movement
/// longer than 0.75 * |r|: the mirrored sphere (r = -0.3) above 27 m/s, the mirrored box (r = -0.2) above 18 m/s (two
/// collision steps). The bodies j = 1 start at 40 m/s and all of them cast, of the bodies j = 0 (falling from rest) only
/// the ones with zero radius do.
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
                .linear_velocity = .{ 0, -40.0 * @as(f32, @floatFromInt(j)), 0 },
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
// Contact scenes (Wave B: ProcessBodyPair, the contact cache, the island solve, the LargeIslandSplitter)

fn real(v: anytype) Real {
    return switch (@typeInfo(@TypeOf(v))) {
        .int, .comptime_int => @floatFromInt(v),
        else => @floatCast(v),
    };
}

/// A static box floor (top at y = `top`) in the NON_MOVING layer, returns its handle
fn addFloor(b: *SceneBuilder, half_extent: f32, top: Real) Allocator.Error!u32 {
    const floor = try b.box(.{ half_extent, 1, half_extent }, 0);
    return b.body(.{ .position = .{ 0, top - 1, 0 }, .shape = floor, .motion_type = 0, .friction = 0.5, .object_layer = Layers.non_moving }, .dont_activate);
}

/// A floor and four walls around [-half_extent, half_extent]^2 (a container for the random scenes)
fn addContainer(b: *SceneBuilder, half_extent: f32, wall_height: f32) Allocator.Error!void {
    _ = try addFloor(b, half_extent + 1, 0);
    const wall_x = try b.box(.{ 0.5, wall_height, half_extent + 1 }, 0.05);
    const wall_z = try b.box(.{ half_extent + 1, wall_height, 0.5 }, 0.05);
    const h = real(half_extent + 0.5);
    for ([_][3]Real{ .{ -h, wall_height, 0 }, .{ h, wall_height, 0 } }) |p|
        _ = try b.body(.{ .position = p, .shape = wall_x, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    for ([_][3]Real{ .{ 0, wall_height, -h }, .{ 0, wall_height, h } }) |p|
        _ = try b.body(.{ .position = p, .shape = wall_z, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
}

/// Every dynamic shape kind of the scenes (convex shapes, compounds and decorated shapes)
const ContactShapes = struct {
    list: [14]u32,

    fn add(b: *SceneBuilder) Allocator.Error!ContactShapes {
        const sphere = try b.sphere(0.45);
        const box = try b.box(.{ 0.4, 0.3, 0.5 }, 0.05);
        const capsule = try b.capsule(0.4, 0.3);
        const tapered_capsule = try b.taperedCapsule(0.4, 0.2, 0.35);
        const cylinder = try b.cylinder(0.35, 0.4, 0.05);
        const tapered_cylinder = try b.taperedCylinder(0.4, 0.15, 0.45, 0.05);
        const hull = try b.convexHull(&.{ .{ -0.5, -0.4, -0.3 }, .{ 0.6, -0.4, -0.2 }, .{ 0.1, 0.5, 0.1 }, .{ 0.0, -0.1, 0.6 }, .{ 0.3, 0.3, -0.5 }, .{ -0.4, 0.2, 0.3 } }, 0.05);
        const static_compound = try b.compound(.static_compound, &.{
            .{ .shape = box, .position = .{ 0.4, 0, 0 } },
            .{ .shape = sphere, .position = .{ -0.5, 0.2, 0.1 }, .user_data = 3 },
            .{ .shape = capsule, .position = .{ 0, -0.2, 0.5 }, .rotation = axisAngle(.{ 1, 0, 0 }, std.math.pi / 2.0) },
        });
        const mutable_compound = try b.compound(.mutable_compound, &.{
            .{ .shape = box, .rotation = axisAngle(.{ 0, 1, 0 }, 0.7) },
            .{ .shape = sphere, .position = .{ 0, 0.6, 0 } },
        });
        const rotated = try b.rotatedTranslated(box, .{ 0.2, -0.1, 0.1 }, axisAngle(.{ 0, 0, 1 }, 0.4));
        const scaled = try b.scaled(box, .{ 1.5, 0.7, 1.2 });
        const scaled_sphere = try b.scaled(sphere, .{ 0.8, 0.8, 0.8 });
        const offset = try b.offsetCenterOfMass(box, .{ 0.1, -0.15, 0.05 });
        const nested = try b.compound(.static_compound, &.{
            .{ .shape = static_compound, .position = .{ 0, 0.3, 0 } },
            .{ .shape = rotated, .position = .{ 0.8, 0, 0 } },
        });
        return .{ .list = .{ sphere, box, capsule, tapered_capsule, cylinder, tapered_cylinder, hull, static_compound, mutable_compound, rotated, scaled, scaled_sphere, offset, nested } };
    }

    fn pick(self: ContactShapes, i: usize) u32 {
        return self.list[i % self.list.len];
    }
};

/// A wavy height function for the mesh and height field terrains
fn terrainHeight(x: f32, z: f32) f32 {
    return @floatCast(0.4 * @sin(0.7 * @as(f64, x)) * @cos(0.5 * @as(f64, z)) + 0.1 * @sin(1.9 * @as(f64, x + z)));
}

/// A mesh terrain of `n` x `n` quads of `cell` meters centered on the origin
fn addMeshTerrain(b: *SceneBuilder, n: u32, cell: f32, flat: bool) Allocator.Error!u32 {
    var triangles: std.ArrayList([9]f32) = .empty;
    defer triangles.deinit(b.allocator);
    const half = 0.5 * cell * @as(f32, @floatFromInt(n));
    for (0..n) |iz| {
        for (0..n) |ix| {
            const x0 = @as(f32, @floatFromInt(ix)) * cell - half;
            const z0 = @as(f32, @floatFromInt(iz)) * cell - half;
            const x1 = x0 + cell;
            const z1 = z0 + cell;
            const h00 = if (flat) 0 else terrainHeight(x0, z0);
            const h10 = if (flat) 0 else terrainHeight(x1, z0);
            const h01 = if (flat) 0 else terrainHeight(x0, z1);
            const h11 = if (flat) 0 else terrainHeight(x1, z1);
            try triangles.append(b.allocator, .{ x0, h00, z0, x0, h01, z1, x1, h11, z1 });
            try triangles.append(b.allocator, .{ x0, h00, z0, x1, h11, z1, x1, h10, z0 });
        }
    }
    return b.mesh(triangles.items);
}

/// A height field terrain of `n` x `n` samples `cell` meters apart centered on the origin
fn addHeightFieldTerrain(b: *SceneBuilder, n: u32, cell: f32) Allocator.Error!u32 {
    var samples: std.ArrayList(f32) = .empty;
    defer samples.deinit(b.allocator);
    const half = 0.5 * cell * @as(f32, @floatFromInt(n - 1));
    for (0..n) |iz| {
        for (0..n) |ix|
            try samples.append(b.allocator, terrainHeight(@as(f32, @floatFromInt(ix)) * cell - half, @as(f32, @floatFromInt(iz)) * cell - half));
    }
    return b.heightField(samples.items, n, .{ -half, 0, -half }, .{ cell, 1, cell });
}

/// A box falling on a static floor (contacts, sleeping on the floor)
fn buildBoxOnFloor(b: *SceneBuilder) Allocator.Error!void {
    const floor = try b.box(.{ 100, 1, 100 }, 0);
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    b.num_updates = 120;
    _ = try b.body(.{ .position = .{ 0, -1, 0 }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ 0, 2, 0 }, .rotation = axisAngle(.{ 1, 0, 0 }, 0.3), .shape = box, .object_layer = Layers.moving }, .activate);
}

/// A small pyramid of boxes on a floor (islands, the contact cache)
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

/// A pyramid with a base of 16 boxes (136 bodies, about 256 contacts in one island): the LargeIslandSplitter splits it
/// (an island of at least LargeIslandSplitter::cLargeIslandTreshold = 128 constraints and contacts), and the whole
/// island falls asleep from the final batch of the split island
fn buildLargePyramid(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    b.num_updates = 90;
    b.max_contact_constraints = 2048;
    b.settings.time_before_sleep = 0.25;
    _ = try addFloor(b, 30, 0);
    const base = 16;
    for (0..base) |y| {
        for (0..base - y) |x| {
            const px = real(x) * 1.05 + real(y) * 0.525 - 8.0;
            _ = try b.body(.{ .position = .{ px, 0.5 + real(y), 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
        }
    }
    // A second, smaller pyramid next to it: an island below the threshold in the same update (solved without a split)
    for (0..4) |y| {
        for (0..4 - y) |x| {
            const px = real(x) * 1.05 + real(y) * 0.525 + 12.0;
            _ = try b.body(.{ .position = .{ px, 0.5 + real(y), 3 }, .shape = box, .object_layer = Layers.moving }, .activate);
        }
    }
}

/// The same large pyramid without the LargeIslandSplitter and with 2 collision steps, a ball knocks it over
fn buildLargePyramidWithoutSplitter(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    const ball = try b.sphere(0.6);
    b.num_updates = 60;
    b.collision_steps = 2;
    b.max_contact_constraints = 2048;
    b.settings.use_large_island_splitter = 0;
    _ = try addFloor(b, 30, 0);
    const base = 12;
    for (0..base) |y| {
        for (0..base - y) |x| {
            const px = real(x) * 1.05 + real(y) * 0.525 - 6.0;
            _ = try b.body(.{ .position = .{ px, 0.5 + real(y), 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
        }
    }
    b.at(20);
    _ = try b.body(.{ .position = .{ -3, 3, 8 }, .linear_velocity = .{ 0, 0, -15 }, .shape = ball, .object_layer = Layers.moving }, .activate);
    b.at(40);
    try b.sysOp(.set_physics_settings, .{ .index = try b.addPhysicsSettings(.{}) }); // Back to Jolt's defaults: the splitter splits the collapsing island
}

/// Every dynamic shape kind dropped on a box floor (rotated, spinning), a dynamic triangle with provided mass properties
fn buildShapesOnFloor(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try ContactShapes.add(b);
    b.num_updates = 100;
    _ = try addFloor(b, 40, 0);
    for (shapes.list, 0..) |shape, i| {
        for (0..2) |j| {
            _ = try b.body(.{
                .position = .{ real(i) * 2.5 - 17.0, 1.5 + real(j) * 0.5, real(j) * 3.0 - 1.5 },
                .rotation = axisAngle(.{ 0.48, 0.6, 0.64 }, 0.4 * @as(f64, @floatFromInt(i + j))),
                .angular_velocity = if (j == 0) .{ 0, 0, 0 } else .{ 1, 2, 0.5 },
                .linear_velocity = .{ 0, -1, 0 },
                .friction = 0.4 + 0.1 * @as(f32, @floatFromInt(j)),
                .shape = shape,
                .object_layer = Layers.moving,
            }, .activate);
        }
    }
    var inertia: [16]f32 = @splat(0);
    inertia[0] = 0.1;
    inertia[5] = 0.1;
    inertia[10] = 0.1;
    inertia[15] = 1;
    const triangle = try b.triangle(.{ -0.5, 0, -0.3, 0.5, 0, -0.3, 0, 0, 0.6 }, 0.05);
    _ = try b.body(.{ .position = .{ 0, 2, 6 }, .rotation = axisAngle(.{ 1, 0, 0 }, 0.3), .shape = triangle, .override_mass_properties = 2, .mass = 1, .inertia = inertia, .object_layer = Layers.moving }, .activate);
}

/// Dynamic shapes stacked on each other (every shape kind against another kind), on a floor
fn buildShapeStacks(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try ContactShapes.add(b);
    b.num_updates = 90;
    _ = try addFloor(b, 40, 0);
    for (shapes.list, 0..) |shape, i| {
        const x = real(i) * 2.6 - 18.0;
        _ = try b.body(.{ .position = .{ x, 0.8, 0 }, .shape = shapes.pick(i + 5), .object_layer = Layers.moving }, .activate);
        _ = try b.body(.{ .position = .{ x + 0.1, 2.2, 0.05 }, .rotation = axisAngle(.{ 0, 1, 0 }, 0.3), .shape = shape, .object_layer = Layers.moving }, .activate);
        _ = try b.body(.{ .position = .{ x - 0.1, 3.6, -0.05 }, .shape = shapes.pick(i + 9), .object_layer = Layers.moving }, .activate);
    }
}

/// Shapes falling on a static mesh terrain, with and without enhanced internal edge removal
fn buildMeshGround(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try ContactShapes.add(b);
    const terrain = try addMeshTerrain(b, 12, 1.5, false);
    b.num_updates = 80;
    b.max_contact_constraints = 2048;
    _ = try b.body(.{ .shape = terrain, .motion_type = 0, .friction = 0.6, .object_layer = Layers.non_moving }, .dont_activate);
    for (0..24) |i| {
        const x = real(i % 6) * 2.4 - 6.0;
        const z = real(i / 6) * 2.4 - 3.6;
        _ = try b.body(.{
            .position = .{ x, 1.8, z },
            .rotation = axisAngle(.{ 0, 0.6, 0.8 }, 0.5 * @as(f64, @floatFromInt(i))),
            .linear_velocity = .{ 0.5, 0, -0.3 },
            .shape = shapes.pick(i),
            .enhanced_internal_edge_removal = @intCast(i % 2),
            .object_layer = Layers.moving,
        }, .activate);
    }
}

/// Shapes falling on a static height field terrain, with and without enhanced internal edge removal
fn buildHeightFieldGround(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try ContactShapes.add(b);
    const terrain = try addHeightFieldTerrain(b, 17, 1.0);
    b.num_updates = 80;
    b.max_contact_constraints = 2048;
    _ = try b.body(.{ .shape = terrain, .motion_type = 0, .friction = 0.6, .object_layer = Layers.non_moving }, .dont_activate);
    for (0..20) |i| {
        const x = real(i % 5) * 2.6 - 5.2;
        const z = real(i / 5) * 2.6 - 3.9;
        _ = try b.body(.{
            .position = .{ x, 1.6, z },
            .rotation = axisAngle(.{ 0.8, 0, 0.6 }, 0.3 * @as(f64, @floatFromInt(i))),
            .angular_velocity = .{ 0, 1, 0 },
            .shape = shapes.pick(i + 3),
            .enhanced_internal_edge_removal = @intCast((i / 2) % 2),
            .object_layer = Layers.moving,
        }, .activate);
    }
}

/// Static grounds of other shape kinds: a plane, a triangle, a convex hull, a compound, a scaled mesh, a rotated box
fn buildStaticShapes(b: *SceneBuilder) Allocator.Error!void {
    const sphere = try b.sphere(0.4);
    const box = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    const capsule = try b.capsule(0.3, 0.25);
    const plane = try b.plane(.{ 0, 1, 0 }, 0, 8);
    const triangle = try b.triangle(.{ -3, 0, -3, 3, 0.5, -3, 0, -0.2, 3 }, 0);
    const hull = try b.convexHull(&.{ .{ -3, -1, -3 }, .{ 3, -1, -3 }, .{ 3, -1, 3 }, .{ -3, -1, 3 }, .{ 0, 0.5, 0 } }, 0.05);
    const static_box = try b.box(.{ 2, 0.5, 2 }, 0.05);
    const compound = try b.compound(.static_compound, &.{ .{ .shape = static_box, .position = .{ -1, 0, 0 } }, .{ .shape = static_box, .position = .{ 2.5, 0.3, 0 }, .rotation = axisAngle(.{ 0, 0, 1 }, 0.2) } });
    const mesh = try b.scaled(try addMeshTerrain(b, 4, 1.5, false), .{ 1.0, 2.0, 1.0 });
    const rotated = try b.rotatedTranslated(static_box, .{ 0, 0.2, 0 }, axisAngle(.{ 1, 0, 0 }, 0.25));
    b.num_updates = 80;
    _ = try addFloor(b, 50, -5); // Catches what falls off
    const grounds = [_]u32{ plane, triangle, hull, compound, mesh, rotated };
    for (grounds, 0..) |ground, i| {
        const x = real(i) * 12.0 - 30.0;
        _ = try b.body(.{ .position = .{ x, 0, 0 }, .shape = ground, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
        for ([_]u32{ sphere, box, capsule }, 0..) |shape, j| {
            _ = try b.body(.{ .position = .{ x + real(j) * 1.2 - 1.2, 2.0 + real(j) * 0.3, real(j) * 0.4 - 0.4 }, .linear_velocity = .{ 0.4, 0, 0.2 }, .shape = shape, .object_layer = Layers.moving }, .activate);
        }
    }
}

/// Friction and restitution: boxes sliding down a ramp with different frictions, balls bouncing with different
/// restitutions, capsules rolling (the default combine functions)
fn buildFrictionRestitution(b: *SceneBuilder) Allocator.Error!void {
    const ramp = try b.box(.{ 12, 0.5, 10 }, 0);
    const box = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    const ball = try b.sphere(0.4);
    const capsule = try b.capsule(0.4, 0.3);
    b.num_updates = 120;
    _ = try addFloor(b, 50, -5);
    _ = try b.body(.{ .position = .{ 0, 0, 0 }, .rotation = axisAngle(.{ 0, 0, 1 }, 0.35), .shape = ramp, .friction = 0.6, .restitution = 0.2, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const frictions = [_]f32{ 0.0, 0.1, 0.3, 0.6, 1.0, 2.0 };
    for (frictions, 0..) |friction, i| {
        _ = try b.body(.{ .position = .{ 4, 2.2, real(i) * 1.5 - 4.0 }, .rotation = axisAngle(.{ 0, 0, 1 }, 0.35), .shape = box, .friction = friction, .object_layer = Layers.moving }, .activate);
    }
    const restitutions = [_]f32{ 0.0, 0.3, 0.6, 0.9, 1.0 };
    for (restitutions, 0..) |restitution, i| {
        _ = try b.body(.{ .position = .{ -20, real(i) * 0.5 + 2.0, real(i) * 1.5 - 4.0 }, .linear_velocity = .{ 0, -2, 0 }, .shape = ball, .restitution = restitution, .object_layer = Layers.moving }, .activate);
    }
    for (0..3) |i| {
        _ = try b.body(.{ .position = .{ 6, 3.0, real(i) * 2.0 + 5.0 }, .rotation = axisAngle(.{ 1, 0, 0 }, std.math.pi / 2.0), .shape = capsule, .friction = 0.5 * @as(f32, @floatFromInt(i)), .restitution = 0.2, .object_layer = Layers.moving }, .activate);
    }
    b.at(60);
    try b.op(.set_friction_and_restitution, 3, .{ .f = &.{ 0.0, 0.5 } });
    try b.op(.set_friction_and_restitution, 9, .{ .f = &.{ 1.0, 0.0 } });
}

/// The custom combine functions (friction and restitution of compound sub shapes, sub shape IDs differ)
fn buildCombineFunctions(b: *SceneBuilder) Allocator.Error!void {
    b.combine_mode = .custom;
    try buildFrictionRestitution(b);
    const shapes = try ContactShapes.add(b);
    for (0..4) |i| {
        _ = try b.body(.{ .position = .{ -10, 2.0, real(i) * 2.5 - 4 }, .shape = shapes.list[7 + i % 2], .friction = 0.3 * @as(f32, @floatFromInt(i)), .restitution = 0.25 * @as(f32, @floatFromInt(i)), .linear_velocity = .{ 2, -3, 0 }, .object_layer = Layers.moving }, .activate);
    }
}

/// Sensors: a static sensor that bodies fall through, a kinematic sensor that moves through sleeping and awake bodies,
/// a dynamic sensor that falls onto the floor (and through it), a kinematic sensor with CollideKinematicVsNonDynamic
/// that detects static bodies, a sensor in the SENSOR layer and one in the MOVING layer
fn buildSensors(b: *SceneBuilder) Allocator.Error!void {
    const sensor_box = try b.box(.{ 2, 0.5, 2 }, 0);
    const small_sensor = try b.box(.{ 0.5, 0.5, 0.5 }, 0);
    const sphere = try b.sphere(0.4);
    const box = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    b.num_updates = 100;
    b.settings.time_before_sleep = 0.3;
    _ = try addFloor(b, 40, 0);
    // Static sensor in the SENSOR layer above the floor, balls fall through it onto the floor
    _ = try b.body(.{ .position = .{ -10, 2, 0 }, .shape = sensor_box, .motion_type = 0, .is_sensor = 1, .object_layer = Layers.sensor }, .dont_activate);
    for (0..3) |i|
        _ = try b.body(.{ .position = .{ -11 + real(i), 5 + real(i), 0 }, .shape = sphere, .object_layer = Layers.moving }, .activate);
    // A row of boxes that fall asleep, then a kinematic sensor (SENSOR layer) moves through them
    const first_box = b.num_handles;
    for (0..4) |i|
        _ = try b.body(.{ .position = .{ real(i) * 1.5, 0.4, 5 }, .shape = box, .object_layer = Layers.moving }, .activate);
    const kinematic_sensor = try b.body(.{ .position = .{ -1.5, 0.5, 5 }, .shape = small_sensor, .motion_type = 1, .is_sensor = 1, .object_layer = Layers.sensor }, .dont_activate);
    // A dynamic sensor in the MOVING layer: it detects the floor and falls through it
    _ = try b.body(.{ .position = .{ 10, 3, 0 }, .shape = small_sensor, .is_sensor = 1, .object_layer = Layers.moving }, .activate);
    // A kinematic sensor with CollideKinematicVsNonDynamic in the MOVING layer moving over static bodies
    for (0..3) |i|
        _ = try b.body(.{ .position = .{ 15 + real(i) * 2, 0.5, -5 }, .shape = box, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const sweeper = try b.body(.{ .position = .{ 12, 0.6, -5 }, .linear_velocity = .{ 2, 0, 0 }, .shape = small_sensor, .motion_type = 1, .is_sensor = 1, .collide_kinematic_vs_non_dynamic = 1, .object_layer = Layers.moving }, .activate);
    b.at(50);
    try b.expectSleeping(first_box);
    try b.op(.set_linear_velocity, kinematic_sensor, .{ .f = &.{ 4, 0, 0 } });
    try b.expectAwake(kinematic_sensor);
    try b.expectSleeping(first_box); // A sensor does not wake bodies up
    b.at(90);
    try b.op(.set_linear_velocity, sweeper, .{ .f = &.{ -3, 0, 0 } });
}

/// Kinematic bodies pushing dynamic ones: a kinematic box with a velocity, a rotating kinematic platform with boxes on
/// it, MoveKinematic between the updates
fn buildKinematicPush(b: *SceneBuilder) Allocator.Error!void {
    const pusher = try b.box(.{ 0.5, 1, 3 }, 0.05);
    const platform = try b.box(.{ 3, 0.25, 3 }, 0.05);
    const box = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    const ball = try b.sphere(0.35);
    b.num_updates = 90;
    _ = try addFloor(b, 40, 0);
    _ = try b.body(.{ .position = .{ -6, 1, 0 }, .linear_velocity = .{ 2, 0, 0 }, .shape = pusher, .motion_type = 1, .object_layer = Layers.moving }, .activate);
    for (0..5) |i| {
        for (0..2) |j|
            _ = try b.body(.{ .position = .{ -3 + real(i) * 0.9, 0.4 + real(j) * 0.8, real(j) * 0.2 }, .shape = if ((i + j) % 2 == 0) box else ball, .object_layer = Layers.moving }, .activate);
    }
    const rotating = try b.body(.{ .position = .{ 10, 1, 0 }, .angular_velocity = .{ 0, 0.8, 0 }, .shape = platform, .motion_type = 1, .object_layer = Layers.moving }, .activate);
    for (0..4) |i|
        _ = try b.body(.{ .position = .{ 8.5 + real(i), 1.7, 1 }, .shape = box, .friction = 0.8, .object_layer = Layers.moving }, .activate);
    var u: u32 = 30;
    while (u < 60) : (u += 5) {
        b.at(u);
        const t: f32 = @floatFromInt(u - 30);
        try b.op(.move_kinematic, rotating, .{ .r = .{ 10, 1 + 0.05 * t, 0 }, .f = &(axisAngle(.{ 0, 1, 0 }, 0.05 * @as(f64, t)) ++ [1]f32{5 * b.delta_time}) });
    }
}

/// Bodies come to rest on the floor and fall asleep, a ball that is created later hits one of them and wakes it up
/// (and its island), another body stays asleep
fn buildSleepAndWakeByHit(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    const ball = try b.sphere(0.3);
    b.num_updates = 110;
    b.settings.time_before_sleep = 0.3;
    _ = try addFloor(b, 20, 0);
    const target = try b.body(.{ .position = .{ 0, 0.5, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
    const on_top = try b.body(.{ .position = .{ 0, 1.5, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
    const far = try b.body(.{ .position = .{ 8, 0.5, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
    const ball_desc = try b.addBodyDesc(.{ .position = .{ -4, 1.2, 0 }, .linear_velocity = .{ 10, 0, 0 }, .shape = ball, .object_layer = Layers.moving });
    b.at(60);
    try b.expectSleeping(target);
    try b.expectSleeping(on_top);
    try b.expectSleeping(far);
    _ = try b.createBody(ball_desc, .activate);
    b.at(95); // The ball hits the box at about update 77
    try b.expectAwake(target);
    try b.expectAwake(on_top); // Same island
    try b.expectSleeping(far);
}

/// The contact listener changes the contact settings (ContactListenerMode.modify): friction, restitution, mass scales,
/// surface velocity (a conveyor), sensor contacts
fn buildListenerModify(b: *SceneBuilder) Allocator.Error!void {
    b.contact_listener_mode = .modify;
    const box = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    const ball = try b.sphere(0.4);
    const conveyor = try b.box(.{ 4, 0.25, 1 }, 0.05);
    b.num_updates = 90;
    _ = try addFloor(b, 40, 0);
    const F = ContactFlags;
    // Boxes sliding with a velocity: zero friction slides on
    for ([_]u32{ 0, F.zero_friction }, 0..) |flags, i|
        _ = try b.body(.{ .position = .{ -10, 0.4, real(i) * 2 }, .linear_velocity = .{ 4, 0, 0 }, .shape = box, .user_data = ContactFlags.userData(flags, @intCast(i)), .object_layer = Layers.moving }, .activate);
    // Balls dropped: bouncy ones bounce
    for ([_]u32{ 0, F.bouncy, F.bouncy | F.zero_friction }, 0..) |flags, i|
        _ = try b.body(.{ .position = .{ -5, 3, real(i) * 2 }, .shape = ball, .user_data = ContactFlags.userData(flags, 10 + @as(u32, @intCast(i))), .object_layer = Layers.moving }, .activate);
    // Stacks with mass scales
    for (0..3) |j|
        _ = try b.body(.{ .position = .{ 0, 0.4 + real(j) * 0.85, 0 }, .shape = box, .user_data = ContactFlags.userData(if (j == 1) F.mass_scale else 0, 20), .object_layer = Layers.moving }, .activate);
    // A static conveyor with a surface velocity, boxes ride on it
    _ = try b.body(.{ .position = .{ 6, 0.25, 0 }, .shape = conveyor, .motion_type = 0, .user_data = ContactFlags.userData(F.surface_velocity, 30), .object_layer = Layers.non_moving }, .dont_activate);
    for (0..2) |i|
        _ = try b.body(.{ .position = .{ 4 + real(i) * 2, 1.0, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
    // A box whose contacts are sensor contacts: it falls through the floor
    _ = try b.body(.{ .position = .{ 10, 1, 4 }, .shape = box, .user_data = ContactFlags.userData(F.sensor, 40), .object_layer = Layers.moving }, .activate);
    // The flags change between the updates
    b.at(45);
    try b.op(.set_user_data, 2, .{ .index = 2, .count = F.zero_friction | F.mass_scale });
    try b.op(.set_linear_velocity, 2, .{ .f = &.{ -3, 0, 0 } });
}

/// OnContactValidate rejects contacts (ContactListenerMode.modify): all contacts of a pair (the body falls through
/// the floor), speculative contacts, and AcceptContact that keeps the callback being called for every hit
fn buildListenerValidate(b: *SceneBuilder) Allocator.Error!void {
    b.contact_listener_mode = .modify;
    const shapes = try ContactShapes.add(b);
    const terrain = try addMeshTerrain(b, 6, 2.0, false);
    b.num_updates = 70;
    _ = try addFloor(b, 40, -3);
    _ = try b.body(.{ .position = .{ 0, 0, 0 }, .shape = terrain, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const F = ContactFlags;
    const flag_sets = [_]u32{ 0, F.reject_all, F.reject_speculative, F.accept_each, F.accept_each | F.bouncy };
    for (flag_sets, 0..) |flags, i| {
        for (0..2) |j| {
            _ = try b.body(.{
                .position = .{ real(i) * 2.2 - 4.4, 1.5 + real(j), real(j) * 2.0 - 1.0 },
                .linear_velocity = .{ 0, -2, 0 },
                .shape = shapes.pick(i * 2 + j + 6),
                .user_data = ContactFlags.userData(flags, @intCast(i)),
                .object_layer = Layers.moving,
            }, .activate);
        }
    }
    b.at(35);
    try b.op(.set_user_data, 4, .{ .index = 1, .count = 0 }); // The first rejected body collides again (it fell through the terrain onto the floor)
}

/// PhysicsSettings::mUseManifoldReduction off: one manifold per sub shape pair (compounds and meshes)
fn buildWithoutManifoldReduction(b: *SceneBuilder) Allocator.Error!void {
    b.settings.use_manifold_reduction = 0;
    try buildManifoldReductionScene(b);
}

/// Body::SetUseManifoldReduction off on half of the bodies
fn buildBodyWithoutManifoldReduction(b: *SceneBuilder) Allocator.Error!void {
    try buildManifoldReductionScene(b);
}

fn buildManifoldReductionScene(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try ContactShapes.add(b);
    const terrain = try addMeshTerrain(b, 8, 1.0, true);
    b.num_updates = 60;
    b.max_contact_constraints = 2048;
    _ = try b.body(.{ .shape = terrain, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    for (0..12) |i| {
        _ = try b.body(.{
            .position = .{ real(i % 4) * 1.8 - 2.7, 1.0 + real(i / 4) * 1.4, real(i % 3) - 1.0 },
            .rotation = axisAngle(.{ 0, 1, 0 }, 0.2 * @as(f64, @floatFromInt(i))),
            .shape = shapes.list[7 + i % 7], // Compounds and decorated shapes: several sub shape pairs
            .use_manifold_reduction = @intCast(i % 2),
            .object_layer = Layers.moving,
        }, .activate);
    }
}

/// Boxes sliding over a flat mesh made of many triangles and over a height field, with and without enhanced internal
/// edge removal, then the same without PhysicsSettings::mCheckActiveEdges
fn buildEnhancedInternalEdgeRemoval(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.4, 0.2, 0.4 }, 0.02);
    const mesh = try addMeshTerrain(b, 16, 0.5, true);
    const height_field = try addHeightFieldTerrain(b, 9, 1.0);
    b.num_updates = 70;
    b.max_contact_constraints = 2048;
    _ = try b.body(.{ .position = .{ 0, 0, 0 }, .shape = mesh, .motion_type = 0, .friction = 0.1, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ 0, 0, 12 }, .shape = height_field, .motion_type = 0, .friction = 0.1, .object_layer = Layers.non_moving }, .dont_activate);
    for (0..4) |i| {
        for ([_]Real{ 0, 12 }) |z| {
            _ = try b.body(.{
                .position = .{ -3.5, 0.6, z + real(i) * 1.2 - 1.8 },
                .linear_velocity = .{ 4, 0, 0.3 },
                .shape = box,
                .friction = 0.1,
                .enhanced_internal_edge_removal = @intCast(i % 2),
                .object_layer = Layers.moving,
            }, .activate);
        }
    }
    var settings: SimPhysicsSettings = .{};
    settings.check_active_edges = 0;
    b.at(35);
    try b.sysOp(.set_physics_settings, .{ .index = try b.addPhysicsSettings(settings) });
    for (0..8) |h|
        try b.op(.set_linear_velocity, 2 + @as(u32, @intCast(h)), .{ .f = &.{ -4, 0, -0.3 } });
}

/// A SimShapeFilter (SimShapeFilterMode.filter_spheres): bodies with the flag ignore the spheres of compounds
fn buildSimShapeFilter(b: *SceneBuilder) Allocator.Error!void {
    b.sim_shape_filter_mode = .filter_spheres;
    const shapes = try ContactShapes.add(b);
    b.num_updates = 60;
    _ = try addFloor(b, 30, 0);
    for (0..8) |i| {
        const flags: u32 = if (i % 2 == 0) ContactFlags.filter_spheres else 0;
        _ = try b.body(.{ .position = .{ real(i) * 2.5 - 9, 1.5, 0 }, .shape = shapes.list[if (i < 4) 0 else 7 + i % 3], .user_data = ContactFlags.userData(flags, @intCast(i)), .object_layer = Layers.moving }, .activate);
        _ = try b.body(.{ .position = .{ real(i) * 2.5 - 9, 3.5, 0.2 }, .shape = shapes.list[i % 3], .object_layer = Layers.moving }, .activate);
    }
}

/// A dense grid of touching spheres on a floor: few batches of active bodies with many body pairs each, so that
/// FindCollisions jobs are spawned from inside jobs (TrySpawnJobFindCollisions) in the multi threaded configurations
fn buildSpawnedFindCollisions(b: *SceneBuilder) Allocator.Error!void {
    const sphere = try b.sphere(0.5);
    b.num_updates = 40;
    b.max_contact_constraints = 4096;
    _ = try addFloor(b, 20, 0);
    for (0..2) |layer| {
        for (0..4) |row| {
            for (0..8) |column| {
                _ = try b.body(.{
                    .position = .{ real(column) * 0.97 - 3.5 + real(layer) * 0.48, 0.5 + real(layer) * 0.8, real(row) * 0.97 - 1.5 + real(layer) * 0.48 },
                    .shape = sphere,
                    .allow_sleeping = 0,
                    .object_layer = Layers.moving,
                }, .activate);
            }
        }
    }
}

/// 240 random bodies dropped into a container (many body pairs, many islands, spawned FindCollisions jobs)
fn buildManyBodies(b: *SceneBuilder) Allocator.Error!void {
    b.rng = 0xabcdef;
    const shapes = try ContactShapes.add(b);
    b.num_updates = 60;
    b.max_body_pairs = 16384;
    b.max_contact_constraints = 8192;
    try addContainer(b, 8, 4);
    for (0..240) |i| {
        _ = try b.body(.{
            .position = .{ real(i % 8) * 2.0 - 7.0, 1.0 + real(i / 64) * 2.2, real((i / 8) % 8) * 2.0 - 7.0 },
            .rotation = b.rotation(),
            .linear_velocity = b.vec3(-2, 2),
            .angular_velocity = b.vec3(-2, 2),
            .shape = shapes.pick(b.index(14)),
            .friction = b.float(0.1, 1.0),
            .restitution = if (b.oneIn(4)) b.float(0, 0.8) else 0,
            .object_layer = Layers.moving,
        }, .activate);
    }
}

/// Several collision steps per update with contacts (a stack, rolling balls, a box on a ramp)
fn buildContactCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const box = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
            const ball = try b.sphere(0.4);
            const ramp = try b.box(.{ 4, 0.25, 2 }, 0.05);
            b.num_updates = 60;
            b.collision_steps = steps;
            _ = try addFloor(b, 30, 0);
            for (0..6) |j|
                _ = try b.body(.{ .position = .{ 0, 0.4 + real(j) * 0.82, real(j % 2) * 0.05 }, .shape = box, .object_layer = Layers.moving }, .activate);
            for (0..4) |i|
                _ = try b.body(.{ .position = .{ 5, 0.4, real(i) * 1.5 - 2 }, .linear_velocity = .{ -2.0 - @as(f32, @floatFromInt(i)), 0, 0.5 }, .shape = ball, .object_layer = Layers.moving }, .activate);
            _ = try b.body(.{ .position = .{ -8, 1.5, 0 }, .rotation = axisAngle(.{ 0, 0, 1 }, 0.3), .shape = ramp, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
            _ = try b.body(.{ .position = .{ -7, 3, 0 }, .shape = box, .friction = 0.1, .object_layer = Layers.moving }, .activate);
            b.at(30);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 45.0}, .count = steps + 1 });
        }
    }.build;
}

/// SaveState / RestoreState of the contact cache in the middle of a contact scene (C++ restores the Zolt state and
/// Zolt the C++ state)
fn buildContactSaveRestore(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    const ball = try b.sphere(0.4);
    b.num_updates = 90;
    b.settings.time_before_sleep = 0.4;
    _ = try addFloor(b, 30, 0);
    for (0..4) |j|
        _ = try b.body(.{ .position = .{ 0, 0.4 + real(j) * 0.82, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
    const roller = try b.body(.{ .position = .{ 4, 0.4, 0 }, .linear_velocity = .{ -1, 0, 1 }, .shape = ball, .object_layer = Layers.moving }, .activate);
    b.at(15);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(25);
    try b.op(.add_impulse, roller, .{ .f = &.{ -20, 5, 0 } });
    b.at(35);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Back to update 15 (the contact cache of update 15 included)
    b.at(50);
    try b.sysOp(.snapshot, .{ .index = 1 });
    b.at(55);
    try b.op(.add_impulse, 1, .{ .f = &.{ 5, 0, 0 } });
    try b.sysOp(.restore_cross, .{ .index = 1 });
    try b.sysOp(.restore_cross, .{ .index = 1 }); // Twice in a row
    b.at(80);
    try b.sysOp(.snapshot, .{ .index = 2 });
    try b.sysOp(.restore_cross, .{ .index = 2 }); // The current state
}

/// The solver settings change between the updates: no warm start, no body pair cache, other step counts, Baumgarte,
/// penetration slop, the speculative contact distance; bodies with their own velocity / position step counts
fn buildSolverSettings(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    const ball = try b.sphere(0.4);
    b.num_updates = 100;
    _ = try addFloor(b, 30, 0);
    for (0..3) |s| {
        for (0..4) |j|
            _ = try b.body(.{
                .position = .{ real(s) * 3 - 3, 0.4 + real(j) * 0.82, 0 },
                .shape = box,
                .num_velocity_steps_override = if (s == 1) 3 else if (s == 2) 15 else 0,
                .num_position_steps_override = if (s == 1) 1 else if (s == 2) 5 else 0,
                .object_layer = Layers.moving,
            }, .activate);
    }
    _ = try b.body(.{ .position = .{ 6, 0.4, 0 }, .linear_velocity = .{ -3, 0, 0 }, .shape = ball, .object_layer = Layers.moving }, .activate);
    var no_warm_start: SimPhysicsSettings = .{};
    no_warm_start.constraint_warm_start = 0;
    var no_cache: SimPhysicsSettings = .{};
    no_cache.use_body_pair_contact_cache = 0;
    var steps: SimPhysicsSettings = .{};
    steps.num_velocity_steps = 4;
    steps.num_position_steps = 1;
    steps.baumgarte = 0.5;
    steps.penetration_slop = 0.05;
    steps.speculative_contact_distance = 0.05;
    steps.min_velocity_for_restitution = 0.5;
    var other: SimPhysicsSettings = .{};
    other.max_penetration_distance = 0.05;
    other.contact_point_preserve_lambda_max_dist_sq = 0.0;
    other.body_pair_cache_max_delta_position_sq = 1.0e-8;
    other.manifold_tolerance = 0.01;
    const indices = [_]u32{ try b.addPhysicsSettings(no_warm_start), try b.addPhysicsSettings(no_cache), try b.addPhysicsSettings(steps), try b.addPhysicsSettings(other), try b.addPhysicsSettings(.{}) };
    for (indices, 0..) |index, i| {
        b.at(15 + 15 * @as(u32, @intCast(i)));
        try b.sysOp(.set_physics_settings, .{ .index = index });
        try b.op(.add_impulse, 13, .{ .f = &.{ if (i % 2 == 0) 4 else -4, 0, 0.5 } }); // The ball
    }
}

/// Bodies change while they are in contact: removed and added again (contact removed callbacks), a new shape (the
/// contact cache of the body is invalidated), a layer that does not collide, another motion type, teleported,
/// destroyed, deactivated
fn buildBodyChangesInContact(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    const ball = try b.sphere(0.45);
    const capsule = try b.capsule(0.3, 0.3);
    b.num_updates = 80;
    _ = try addFloor(b, 30, 0);
    // Stack i: bottom box handle 1 + 2 i, top box 2 + 2 i
    for (0..6) |i| {
        for (0..2) |j|
            _ = try b.body(.{ .position = .{ real(i) * 2 - 5, 0.4 + real(j) * 0.82, 0 }, .shape = box, .allow_dynamic_or_kinematic = 1, .object_layer = Layers.moving }, .activate);
    }
    b.at(20);
    try b.op(.remove_body, 2, .{}); // The top box of stack 0
    try b.op(.set_shape, 3, .{ .index = ball, .count = 1, .activation = .activate });
    try b.op(.set_object_layer, 6, .{ .index = Layers.moving2 }); // Collides with the floor, not with the box below: falls through it
    try b.op(.set_motion_type, 7, .{ .index = 1, .activation = .activate }); // Dynamic -> kinematic with a box on it
    try b.op(.set_position, 11, .{ .r = .{ 3, 3, 2 }, .activation = .activate }); // Teleported, the box on it falls
    b.at(30);
    try b.op(.add_body, 2, .{ .activation = .activate });
    try b.op(.set_shape, 3, .{ .index = capsule, .count = 0, .activation = .dont_activate });
    try b.op(.set_motion_type, 7, .{ .index = 2, .activation = .activate });
    try b.op(.set_motion_type, 9, .{ .index = 0, .activation = .dont_activate }); // Dynamic -> static with a box on it
    b.at(40);
    try b.op(.remove_body, 12, .{});
    try b.op(.destroy_body, 12, .{});
    try b.op(.set_object_layer, 6, .{ .index = Layers.moving });
    try b.op(.set_rotation, 10, .{ .f = &axisAngle(.{ 0, 0, 1 }, 0.3), .activation = .activate });
    b.at(50);
    try b.op(.set_motion_type, 9, .{ .index = 2, .activation = .activate });
    try b.op(.deactivate_body, 1, .{}); // A body in contact goes to sleep, the box on it keeps it in its island
}

/// Bodies with restricted degrees of freedom and gyroscopic forces colliding
fn buildContactAllowedDOFs(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try ContactShapes.add(b);
    b.num_updates = 70;
    _ = try addFloor(b, 30, 0);
    const dofs = [_]u32{ 0b111111, 0b100011, 0b000111, 0b111000, 0b010111, 0b101101 };
    for (dofs, 0..) |d, i| {
        _ = try b.body(.{ .position = .{ real(i) * 2.2 - 5.5, 1.5, 0 }, .rotation = axisAngle(.{ 0, 0, 1 }, 0.4), .angular_velocity = .{ 1, 3, 0 }, .shape = shapes.pick(i + 1), .allowed_dofs = d, .apply_gyroscopic_force = @intCast(i % 2), .object_layer = Layers.moving }, .activate);
        _ = try b.body(.{ .position = .{ real(i) * 2.2 - 5.5, 3.5, 0.1 }, .shape = shapes.pick(i + 4), .allowed_dofs = dofs[(i + 1) % dofs.len], .object_layer = Layers.moving }, .activate);
    }
}

/// A floor and a stack far from the origin (the base offsets of the narrow phase and the cache, double precision)
fn buildFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const floor = try b.box(.{ 20, 1, 20 }, 0);
    const box = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    const ball = try b.sphere(0.4);
    b.num_updates = 60;
    const o: Real = 5000.0;
    _ = try b.body(.{ .position = .{ o, -1, -o }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    for (0..4) |j|
        _ = try b.body(.{ .position = .{ o, 0.4 + real(j) * 0.82, -o + real(j % 2) * 0.03 }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ o + 4, 0.4, -o }, .linear_velocity = .{ -4, 0, 0 }, .shape = ball, .object_layer = Layers.moving }, .activate);
}

/// Kinematic and static bodies with CollideKinematicVsNonDynamic: contacts without a response between non-dynamic
/// bodies, kinematic vs kinematic
fn buildKinematicVsNonDynamic(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    b.num_updates = 60;
    _ = try addFloor(b, 30, 0);
    for (0..3) |i|
        _ = try b.body(.{ .position = .{ real(i) * 2, 0.5, 0 }, .shape = box, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ -3, 0.6, 0 }, .linear_velocity = .{ 2, 0, 0 }, .shape = box, .motion_type = 1, .collide_kinematic_vs_non_dynamic = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ -3, 0.6, 3 }, .linear_velocity = .{ 2, 0, 0 }, .shape = box, .motion_type = 1, .collide_kinematic_vs_non_dynamic = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 3, 0.6, 3 }, .linear_velocity = .{ -2, 0, 0 }, .shape = box, .motion_type = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 1, 3, 3 }, .shape = box, .object_layer = Layers.moving }, .activate); // A dynamic box on the kinematic ones
}

/// Bodies that start deeply penetrating each other (MaxPenetrationDistance, the position solver pushes them apart)
fn buildDeepPenetration(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    const ball = try b.sphere(0.5);
    b.num_updates = 50;
    _ = try addFloor(b, 30, 0);
    for (0..4) |i|
        _ = try b.body(.{ .position = .{ real(i) * 0.4, 0.3 + real(i) * 0.5, real(i % 2) * 0.3 }, .shape = if (i % 2 == 0) box else ball, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 5, -0.3, 0 }, .shape = ball, .object_layer = Layers.moving }, .activate); // Half inside the floor
}

/// Random shapes, random bodies and random operations in a container (seed `seed`, `steps` collision steps), random
/// listener, combine and filter modes and settings
fn buildContactRandom(comptime seed: u32, comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.rng = 0x51ed27 * seed + 13;
            b.collision_steps = steps;
            b.num_updates = 80;
            b.max_contact_constraints = 4096;
            b.gravity = .{ b.float(-2, 2), b.float(-15, -5), b.float(-2, 2) };
            if (b.oneIn(2)) b.contact_listener_mode = .modify;
            if (b.oneIn(2)) b.combine_mode = .custom;
            if (b.oneIn(2)) b.sim_shape_filter_mode = .filter_spheres;
            if (b.oneIn(3)) b.settings.use_manifold_reduction = 0;
            if (b.oneIn(3)) b.settings.allow_sleeping = 0;
            if (b.oneIn(4)) b.settings.use_large_island_splitter = 0;
            b.settings.time_before_sleep = b.float(0.2, 1.0);
            const shapes = try ContactShapes.add(b);
            try addContainer(b, 6, 3);
            const num_bodies = 60;
            const first: u32 = b.num_handles;
            const F = ContactFlags;
            const flag_choices = [_]u32{ 0, 0, 0, F.zero_friction, F.bouncy, F.mass_scale, F.accept_each, F.reject_speculative, F.filter_spheres, F.surface_velocity };
            for (0..num_bodies) |i| {
                const motion_type: u32 = if (b.oneIn(12)) 1 else 2;
                const dofs: u32 = if (b.oneIn(6)) 0b100011 else 0b111111;
                // A body that is not activated has no velocity (Jolt asserts it in ProcessBodyPair: Body::ValidateMotion)
                const activation: SceneBuilder.Activation = if (b.oneIn(6)) .dont_activate else .activate;
                const v = b.vec3(-3, 3);
                const w = b.vec3(-2, 2);
                _ = try b.body(.{
                    .position = .{ real(i % 6) * 1.9 - 4.75, 0.8 + real(i / 36) * 2.0 + b.float(0, 0.3), real((i / 6) % 6) * 1.9 - 4.75 },
                    .rotation = b.rotation(),
                    .linear_velocity = if (activation == .activate) v else @splat(0),
                    .angular_velocity = if (activation == .activate) w else @splat(0),
                    .friction = b.float(0, 1),
                    .restitution = if (b.oneIn(3)) b.float(0, 1) else 0,
                    .linear_damping = if (b.oneIn(3)) b.float(0, 1) else 0.05,
                    .gravity_factor = if (b.oneIn(6)) b.float(0, 2) else 1.0,
                    .allowed_dofs = if (motion_type == 2) dofs else 0b111111, // Kinematic bodies have all DOFs
                    .apply_gyroscopic_force = b.index(2),
                    .enhanced_internal_edge_removal = b.index(2),
                    .use_manifold_reduction = if (b.oneIn(4)) 0 else 1,
                    .num_velocity_steps_override = if (b.oneIn(8)) 1 + b.index(15) else 0,
                    .num_position_steps_override = if (b.oneIn(8)) 1 + b.index(5) else 0,
                    .motion_type = motion_type,
                    .allow_dynamic_or_kinematic = 1,
                    .is_sensor = if (b.oneIn(15)) 1 else 0,
                    .shape = shapes.pick(b.index(14)),
                    .user_data = ContactFlags.userData(flag_choices[b.index(flag_choices.len)], @intCast(i)),
                    .object_layer = if (b.oneIn(10)) Layers.moving2 else Layers.moving,
                }, activation);
            }
            var u: u32 = 1;
            while (u < b.num_updates) : (u += 1 + b.index(4)) {
                b.at(u);
                for (0..b.index(4)) |_| {
                    const h = first + b.index(num_bodies);
                    const v = b.vec3(-4, 4);
                    const w = b.vec3(-2, 2);
                    switch (b.index(12)) {
                        0 => try b.op(.add_impulse, h, .{ .f = &.{ v[0] * 3, v[1] * 3, v[2] * 3 } }),
                        1 => try b.op(.add_angular_impulse, h, .{ .f = &w }),
                        2 => try b.op(.set_linear_velocity, h, .{ .f = &v }),
                        3 => try b.op(.activate_body, h, .{}),
                        4 => try b.op(.deactivate_body, h, .{}),
                        5 => try b.op(.set_friction_and_restitution, h, .{ .f = &.{ b.float(0, 1), b.float(0, 1) } }),
                        6 => try b.op(.set_user_data, h, .{ .index = h, .count = flag_choices[b.index(flag_choices.len)] }),
                        7 => try b.op(.set_motion_type, h, .{ .index = 1 + b.index(2), .activation = .activate }),
                        8 => try b.op(.set_shape, h, .{ .index = shapes.pick(b.index(14)), .count = 1, .activation = .activate }),
                        9 => try b.op(.add_force, h, .{ .f = &.{ v[0] * 100, v[1] * 100, v[2] * 100 } }),
                        10 => try b.op(.set_position, h, .{ .r = .{ real(b.float(-4, 4)), real(b.float(1, 4)), real(b.float(-4, 4)) }, .activation = .activate }),
                        else => try b.op(.reset_sleep_timer, h, .{}),
                    }
                }
            }
        }
    }.build;
}

// -------------------------------------------------------------------------------------------------------------------
// CCD scenes (Wave B R4: JobFindCCDContacts, sSolveCCDContact, JobResolveCCDContacts)

/// A linear cast bullet that hits a thin wall
fn buildBulletHitsWall(b: *SceneBuilder) Allocator.Error!void {
    const wall = try b.box(.{ 5, 5, 0.05 }, 0);
    const bullet = try b.sphere(0.05);
    b.num_updates = 30;
    b.gravity = .{ 0, 0, 0 };
    _ = try b.body(.{ .position = .{ 0, 0, 5 }, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .linear_velocity = .{ 0, 0, 200 }, .shape = bullet, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
}

/// The dynamic shapes of the CCD scenes (every convex kind, compounds and decorated shapes, all with an inner radius)
const BulletShapes = struct {
    list: [10]u32,

    fn add(b: *SceneBuilder) Allocator.Error!BulletShapes {
        const sphere = try b.sphere(0.2);
        const box = try b.box(.{ 0.2, 0.15, 0.25 }, 0.05);
        const capsule = try b.capsule(0.2, 0.15);
        const tapered_capsule = try b.taperedCapsule(0.2, 0.1, 0.18);
        const cylinder = try b.cylinder(0.2, 0.2, 0.05);
        const hull = try b.convexHull(&.{ .{ -0.25, -0.2, -0.15 }, .{ 0.3, -0.2, -0.1 }, .{ 0.05, 0.25, 0.05 }, .{ 0.0, -0.05, 0.3 }, .{ 0.15, 0.15, -0.25 } }, 0.03);
        const compound = try b.compound(.static_compound, &.{
            .{ .shape = box, .position = .{ 0.2, 0, 0 } },
            .{ .shape = sphere, .position = .{ -0.25, 0.1, 0 }, .user_data = 5 },
        });
        const rotated = try b.rotatedTranslated(box, .{ 0.1, -0.05, 0.05 }, axisAngle(.{ 0, 0, 1 }, 0.4));
        const scaled = try b.scaled(sphere, .{ 1.5, 1.5, 1.5 });
        const offset = try b.offsetCenterOfMass(box, .{ 0.05, -0.05, 0.02 });
        return .{ .list = .{ sphere, box, capsule, tapered_capsule, cylinder, hull, compound, rotated, scaled, offset } };
    }

    fn pick(self: BulletShapes, i: usize) u32 {
        return self.list[i % self.list.len];
    }
};

/// Vertical quad (two triangles) in the plane x = 0, 4 x 4 m, its front faces point to -x (`front`) or +x
fn wallTriangles(front: bool) [2][9]f32 {
    if (front)
        return .{ .{ 0, -2, -2, 0, -2, 2, 0, 2, 2 }, .{ 0, -2, -2, 0, 2, 2, 0, 2, -2 } };
    return .{ .{ 0, -2, -2, 0, 2, 2, 0, -2, 2 }, .{ 0, -2, -2, 0, 2, -2, 0, 2, 2 } };
}

/// Linear cast bullets of every dynamic shape kind fired at static targets of every kind, one lane per target: a thin
/// box, a mesh wall (front faces and back faces, which the cast ignores), a triangle, a convex hull, a compound, a
/// scaled box, a rotated wall (an oblique hit with friction), a plane, a wall in a layer the bullet does not collide
/// with in the broad phase (lq_debris) and one that only the object layer filter rejects (hq_debris vs hq_debris)
fn buildCCDStaticTargets(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BulletShapes.add(b);
    const wall = try b.box(.{ 0.05, 2, 2 }, 0);
    const mesh_front = try b.mesh(&wallTriangles(true));
    const mesh_back = try b.mesh(&wallTriangles(false));
    const triangle = try b.triangle(.{ 0, -2, -2, 0, -2, 2, 0, 2, 0 }, 0);
    const hull = try b.convexHull(&.{ .{ -0.3, -2, -2 }, .{ -0.3, -2, 2 }, .{ -0.3, 2, -2 }, .{ -0.3, 2, 2 }, .{ 0.3, 0, 0 } }, 0.05);
    const thin = try b.box(.{ 0.05, 1, 2 }, 0);
    const compound = try b.compound(.static_compound, &.{ .{ .shape = thin, .position = .{ 0, 1, 0 } }, .{ .shape = thin, .position = .{ 0.3, -1, 0 }, .user_data = 9 } });
    const scaled = try b.scaled(wall, .{ 0.5, 1.5, 1 });
    const plane = try b.plane(.{ -1, 0, 0 }, 0, 2);
    b.num_updates = 40;
    _ = try addFloor(b, 60, -8); // Catches what falls down

    const Target = struct { shape: u32, rotation: [4]f32 = identity_rotation, layer: u32 = Layers.non_moving, bullet_layer: u32 = Layers.moving, friction: f32 = 0.2, restitution: f32 = 0.0 };
    const targets = [_]Target{
        .{ .shape = wall },
        .{ .shape = mesh_front, .friction = 0.5 },
        .{ .shape = mesh_back },
        .{ .shape = triangle },
        .{ .shape = hull, .restitution = 0.5 },
        .{ .shape = compound },
        .{ .shape = scaled, .restitution = 1.0 },
        .{ .shape = wall, .rotation = axisAngle(.{ 0, 1, 0 }, 0.6), .friction = 0.8 },
        .{ .shape = plane },
        .{ .shape = wall, .layer = Layers.lq_debris },
        .{ .shape = wall, .layer = Layers.hq_debris, .bullet_layer = Layers.hq_debris },
    };
    for (targets, 0..) |t, i| {
        const z = real(i) * 6.0 - 30.0;
        _ = try b.body(.{ .position = .{ 20, 2, z }, .rotation = t.rotation, .shape = t.shape, .motion_type = 0, .friction = t.friction, .restitution = t.restitution, .object_layer = t.layer }, .dont_activate);
        for (0..2) |j| {
            _ = try b.body(.{
                .position = .{ 0, 1.5 + real(j), z + real(j) * 0.5 - 0.25 },
                .rotation = axisAngle(.{ 0.6, 0.8, 0 }, 0.3 * @as(f64, @floatFromInt(i + j))),
                .linear_velocity = .{ 120.0 + 30.0 * @as(f32, @floatFromInt(j)), 0, 0 },
                .angular_velocity = if (j == 0) .{ 0, 0, 0 } else .{ 0, 3, 1 },
                .shape = shapes.pick(i + 3 * j),
                .restitution = if (j == 1) 0.3 else 0.0,
                .motion_quality = 1,
                .object_layer = t.bullet_layer,
            }, .activate);
        }
    }
}

/// Linear cast bodies hitting each other: head on (elastic boxes as in MotionQualityLinearCastTests, inelastic spheres
/// of different mass), a body that chases another one which hits a wall first (no response for the chaser), a body that
/// hits another linear cast body before that one hits a wall (the other body's hit is reset and its motion shortened),
/// bodies moving in parallel (relative movement below the threshold), a slow linear cast body that does not cast, and
/// three bodies converging
fn buildCCDBodiesHitEachOther(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    const sphere = try b.sphere(0.4);
    const heavy_sphere = try b.sphere(0.4);
    b.setDensity(heavy_sphere, 3000);
    const small_sphere = try b.sphere(0.3);
    const big_sphere = try b.sphere(0.5);
    const wall = try b.box(.{ 0.05, 2, 2 }, 0);
    b.num_updates = 40;
    b.gravity = .{ 0, 0, 0 };
    // Head on, elastic
    _ = try b.body(.{ .position = .{ -1, 0, -24 }, .linear_velocity = .{ 120, 0, 0 }, .restitution = 1.0, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 1, 0, -24 }, .linear_velocity = .{ -120, 0, 0 }, .restitution = 1.0, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    // Head on, inelastic, different masses and speeds, slightly offset (angular velocity)
    _ = try b.body(.{ .position = .{ -3, 0, -18 }, .linear_velocity = .{ 200, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 3, 0.2, -18 }, .linear_velocity = .{ -100, 0, 0 }, .shape = heavy_sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    // A chases B, B hits the wall first: A moves to its hit without a response
    _ = try b.body(.{ .position = .{ 4, 0, -12 }, .linear_velocity = .{ 360, 0, 0 }, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 8, 0, -12 }, .linear_velocity = .{ 120, 0, 0 }, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 10, 0, -12 }, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    // A hits B before B hits the wall behind A: B's hit is reset and B's motion shortened
    _ = try b.body(.{ .position = .{ 0, 0, -6 }, .linear_velocity = .{ 90, 0, 0 }, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 4, 0, -6 }, .linear_velocity = .{ -432, 0, 0 }, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ -3, 0, -6 }, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    // Moving in parallel and one behind the other with the same velocity, then a wall
    _ = try b.body(.{ .position = .{ 0, 0, 0 }, .linear_velocity = .{ 150, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 0, 0, 0.9 }, .linear_velocity = .{ 150, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 1.0, 0, 0.3 }, .linear_velocity = .{ 150, 0, 0 }, .shape = small_sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 30, 0, 0.5 }, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    // A bullet hits a slow linear cast body (it moves less than the threshold: no cast, it moved already)
    _ = try b.body(.{ .position = .{ 0, 0, 6 }, .linear_velocity = .{ 150, 0, 0 }, .shape = small_sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 4, 0.1, 6 }, .linear_velocity = .{ -10, 0, 0 }, .shape = big_sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    // Three bodies converging
    _ = try b.body(.{ .position = .{ -4, 0, 12 }, .linear_velocity = .{ 200, 0, 0 }, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 4, 0, 12 }, .linear_velocity = .{ -200, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 0, 4, 12 }, .linear_velocity = .{ 0, -200, 0 }, .rotation = axisAngle(.{ 0, 0, 1 }, 0.5), .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
}

/// Linear cast bodies against kinematic and dynamic bodies: a kinematic box moving towards the bullet (no correction
/// for its motion, it moved already) and a rotating kinematic platform (the response leaves them unchanged), a
/// sleeping dynamic box (woken up by the hit), a slow dynamic box, a fast discrete box (moved before the casts: the
/// bullet tunnels through it), a heavy dynamic body
fn buildCCDKinematicAndDynamic(b: *SceneBuilder) Allocator.Error!void {
    const sphere = try b.sphere(0.25);
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    const platform = try b.box(.{ 0.3, 2, 2 }, 0.05);
    const heavy = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    b.setDensity(heavy, 20000);
    b.num_updates = 40;
    b.gravity = .{ 0, 0, 0 };
    b.settings.time_before_sleep = 0.2;
    _ = try b.body(.{ .position = .{ 8, 0, -12 }, .linear_velocity = .{ -5, 0, 0 }, .shape = box, .motion_type = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 0, 0, -12 }, .linear_velocity = .{ 150, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 8, 0, -6 }, .angular_velocity = .{ 0, 4, 0 }, .shape = platform, .motion_type = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 0, 0.3, -5.5 }, .linear_velocity = .{ 160, 0, 0 }, .shape = box, .motion_quality = 1, .friction = 0.7, .object_layer = Layers.moving }, .activate);
    const sleeping = try b.body(.{ .position = .{ 8, 0, 0 }, .shape = box, .object_layer = Layers.moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ 0, 0.2, 0 }, .linear_velocity = .{ 180, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 8, 0, 6 }, .linear_velocity = .{ -4, 0, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 0, 0, 6 }, .linear_velocity = .{ 150, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 2.5, 0, 12 }, .linear_velocity = .{ -150, 0, 0 }, .shape = box, .object_layer = Layers.moving }, .activate); // Ends where the bullet starts: the bullet moves out of it (a back face)
    _ = try b.body(.{ .position = .{ 0, 0, 12 }, .linear_velocity = .{ 150, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 8, 0, 18 }, .shape = heavy, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 0, 0.4, 18 }, .linear_velocity = .{ 200, 0, 0 }, .angular_velocity = .{ 5, 0, 0 }, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    try b.expectSleeping(sleeping);
    b.at(5);
    try b.expectAwake(sleeping); // Woken up by the CCD hit in update 0
}

/// Linear cast bodies fired diagonally onto a floor with every combination of friction and restitution (the friction
/// part of sSolveCCDContact, the restitution bias above MinVelocityForRestitution)
fn buildCCDRestitutionFriction(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BulletShapes.add(b);
    b.num_updates = 60;
    const floor = try b.box(.{ 40, 0.05, 40 }, 0);
    _ = try b.body(.{ .position = .{ 0, -0.05, 0 }, .shape = floor, .motion_type = 0, .friction = 0.6, .restitution = 0.2, .object_layer = Layers.non_moving }, .dont_activate);
    const frictions = [_]f32{ 0.0, 0.3, 1.0 };
    const restitutions = [_]f32{ 0.0, 0.5, 1.0 };
    for (frictions, 0..) |friction, i| {
        for (restitutions, 0..) |restitution, j| {
            _ = try b.body(.{
                .position = .{ -20 + real(i) * 2, 10, real(j) * 6.0 - 6.0 },
                .linear_velocity = .{ 60, -150, 8.0 * @as(f32, @floatFromInt(j)) },
                .angular_velocity = .{ 0, 0, 2.0 * @as(f32, @floatFromInt(i)) },
                .shape = shapes.pick(3 * i + j),
                .friction = friction,
                .restitution = restitution,
                .motion_quality = 1,
                .object_layer = Layers.moving,
            }, .activate);
        }
    }
}

/// The custom combine functions with CCD hits on a compound target (the sub shape IDs of the CCD manifold)
fn buildCCDCombineCustom(b: *SceneBuilder) Allocator.Error!void {
    b.combine_mode = .custom;
    try buildCCDRestitutionFriction(b);
    const thin = try b.box(.{ 2, 0.05, 2 }, 0);
    const compound = try b.compound(.static_compound, &.{ .{ .shape = thin, .position = .{ -2, 0, 0 } }, .{ .shape = thin, .position = .{ 2, 0, 0 }, .user_data = 2 } });
    const sphere = try b.sphere(0.2);
    _ = try b.body(.{ .position = .{ 20, 2, 20 }, .shape = compound, .motion_type = 0, .friction = 0.9, .restitution = 0.7, .object_layer = Layers.non_moving }, .dont_activate);
    for (0..4) |i|
        _ = try b.body(.{ .position = .{ 17 + real(i) * 2, 12, 20 }, .linear_velocity = .{ 5, -160, 0 }, .restitution = 0.25 * @as(f32, @floatFromInt(i)), .friction = 0.3, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
}

/// The contact listener with CCD hits (ContactListenerMode.modify): OnContactValidate rejects all contacts of a pair (the
/// bullet goes through the wall), rejects speculative contacts (a cast hit has no negative depth), accepts each contact;
/// OnContactAdded makes the contact a sensor (no response), bouncy, frictionless, with mass scales (the bullet created
/// before and after its target: the body pair is swapped for the callback and the scales swapped back)
fn buildCCDListener(b: *SceneBuilder) Allocator.Error!void {
    b.contact_listener_mode = .modify;
    const shapes = try BulletShapes.add(b);
    const wall = try b.box(.{ 0.05, 2, 2 }, 0);
    const target = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    b.num_updates = 40;
    b.gravity = .{ 0, 0, 0 };
    const F = ContactFlags;
    const flag_sets = [_]u32{ 0, F.reject_all, F.reject_speculative, F.accept_each, F.sensor, F.bouncy, F.zero_friction, F.mass_scale, F.bouncy | F.zero_friction, F.surface_velocity, F.accept_each | F.mass_scale };
    for (flag_sets, 0..) |flags, i| {
        const z = real(i) * 6.0 - 30.0;
        _ = try b.body(.{ .position = .{ 20, 0, z }, .rotation = axisAngle(.{ 0, 1, 0 }, 0.3), .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
        _ = try b.body(.{ .position = .{ 0, 0, z }, .linear_velocity = .{ 150, 0, 10 }, .shape = shapes.pick(i), .user_data = ContactFlags.userData(flags, @intCast(i)), .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    }
    // Dynamic targets: created before the bullet (lower ID) and after it
    for (0..2) |i| {
        const z = 40.0 + real(i) * 6.0;
        const flags: u32 = F.mass_scale | F.bouncy;
        if (i == 0) _ = try b.body(.{ .position = .{ 10, 0, z }, .shape = target, .object_layer = Layers.moving }, .activate);
        _ = try b.body(.{ .position = .{ 0, 0, z }, .linear_velocity = .{ 150, 0, 0 }, .shape = shapes.pick(i), .user_data = ContactFlags.userData(flags, 20), .motion_quality = 1, .object_layer = Layers.moving }, .activate);
        if (i == 1) _ = try b.body(.{ .position = .{ 10, 0, z }, .shape = target, .object_layer = Layers.moving }, .activate);
    }
}

/// Linear cast bodies that start penetrating: a sphere half inside a thick box moving deeper (a hit at fraction 0, the
/// deepest point), a sphere inside a box moving out (the back face is ignored), a box touching a wall, two linear cast
/// bodies starting in contact and moving towards each other
fn buildCCDStartsPenetrating(b: *SceneBuilder) Allocator.Error!void {
    const thick = try b.box(.{ 1, 1, 1 }, 0.05);
    const sphere = try b.sphere(0.2);
    const box = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    b.num_updates = 30;
    b.gravity = .{ 0, 0, 0 };
    _ = try b.body(.{ .position = .{ 0, 0, -10 }, .shape = thick, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ -1.1, 0, -10 }, .linear_velocity = .{ 120, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 0, 0, -5 }, .shape = thick, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ 0.2, 0.1, -5 }, .linear_velocity = .{ 120, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 0, 0, 0 }, .shape = thick, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ -1.3, 0, 0 }, .linear_velocity = .{ 90, 30, 0 }, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 0, 0, 5 }, .linear_velocity = .{ 100, 0, 0 }, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 0.55, 0.05, 5 }, .linear_velocity = .{ -100, 0, 0 }, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
}

/// 72 linear cast bodies in a container of thin walls, fired in every direction: many CCD bodies per step, so that
/// several FindCCDContacts jobs run (one per 4 CCD bodies, up to the concurrency of the job system), hits against the
/// walls and against each other
fn buildCCDManyBodies(b: *SceneBuilder) Allocator.Error!void {
    b.rng = 0x5eed1;
    const shapes = try BulletShapes.add(b);
    b.num_updates = 40;
    b.max_contact_constraints = 4096;
    const floor = try b.box(.{ 12, 0.1, 12 }, 0);
    const wall_x = try b.box(.{ 0.05, 4, 12 }, 0);
    const wall_z = try b.box(.{ 12, 4, 0.05 }, 0);
    _ = try b.body(.{ .position = .{ 0, -0.1, 0 }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    for ([_]Real{ -11, 11 }) |x|
        _ = try b.body(.{ .position = .{ x, 4, 0 }, .shape = wall_x, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    for ([_]Real{ -11, 11 }) |z|
        _ = try b.body(.{ .position = .{ 0, 4, z }, .shape = wall_z, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    for (0..72) |i| {
        const dir = b.vec3(-1, 1);
        _ = try b.body(.{
            .position = .{ real(i % 6) * 2.5 - 6.25, 1.0 + real(i / 36) * 3.0, real((i / 6) % 6) * 2.5 - 6.25 },
            .rotation = b.rotation(),
            .linear_velocity = .{ dir[0] * 150.0, dir[1] * 40.0, dir[2] * 150.0 },
            .angular_velocity = b.vec3(-3, 3),
            .shape = shapes.pick(b.index(10)),
            .restitution = if (b.oneIn(3)) b.float(0, 1) else 0,
            .friction = b.float(0, 1),
            .motion_quality = 1,
            .object_layer = Layers.moving,
        }, .activate);
    }
}

/// Sensors in the path of linear cast bodies: a static sensor and a moving kinematic sensor between the bullets and a
/// wall (the casts ignore sensors), a dynamic linear cast sensor (no cast: CCD sensors are not supported)
fn buildCCDSensors(b: *SceneBuilder) Allocator.Error!void {
    const sensor_box = try b.box(.{ 0.5, 2, 2 }, 0);
    const wall = try b.box(.{ 0.05, 2, 2 }, 0);
    const sphere = try b.sphere(0.25);
    b.num_updates = 30;
    b.gravity = .{ 0, 0, 0 };
    _ = try b.body(.{ .position = .{ 10, 0, 0 }, .shape = sensor_box, .motion_type = 0, .is_sensor = 1, .object_layer = Layers.sensor }, .dont_activate);
    _ = try b.body(.{ .position = .{ 10, 0, 6 }, .linear_velocity = .{ 0, 0, 1 }, .shape = sensor_box, .motion_type = 1, .is_sensor = 1, .object_layer = Layers.sensor }, .activate);
    for ([_]Real{ 0, 6 }) |z| {
        _ = try b.body(.{ .position = .{ 20, 0, z }, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
        _ = try b.body(.{ .position = .{ 0, 0, z }, .linear_velocity = .{ 150, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
        _ = try b.body(.{ .position = .{ 9, 0.5, z + 0.5 }, .linear_velocity = .{ 150, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate); // Starts inside the sensor
    }
    _ = try b.body(.{ .position = .{ 0, 0, 12 }, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ -10, 0, 12 }, .linear_velocity = .{ 150, 0, 0 }, .shape = sphere, .is_sensor = 1, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
}

/// Fast bodies falling onto a mesh terrain and a height field terrain (back faces, active edges, enhanced internal edge
/// removal on some bodies), bodies thrown fast over the terrain, two collision steps
fn buildCCDTerrain(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try BulletShapes.add(b);
    const mesh = try addMeshTerrain(b, 10, 1.5, false);
    const height_field = try addHeightFieldTerrain(b, 13, 1.25);
    b.num_updates = 50;
    b.collision_steps = 2;
    b.max_contact_constraints = 2048;
    _ = try b.body(.{ .position = .{ -10, 0, 0 }, .shape = mesh, .motion_type = 0, .friction = 0.5, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ 10, 0, 0 }, .shape = height_field, .motion_type = 0, .friction = 0.5, .object_layer = Layers.non_moving }, .dont_activate);
    for (0..16) |i| {
        const x = real(i % 4) * 2.5 - 13.75 + real(i / 8) * 20.0;
        const z = real((i / 4) % 2) * 3.0 - 1.5;
        _ = try b.body(.{
            .position = .{ x, 8 + real(i % 3), z },
            .rotation = axisAngle(.{ 0.6, 0, 0.8 }, 0.4 * @as(f64, @floatFromInt(i))),
            .linear_velocity = .{ 10.0 * @as(f32, @floatFromInt(i % 3)) - 10.0, -200, 3 },
            .shape = shapes.pick(i),
            .restitution = if (i % 4 == 0) 0.5 else 0,
            .enhanced_internal_edge_removal = @intCast(i % 2),
            .motion_quality = 1,
            .object_layer = Layers.moving,
        }, .activate);
    }
    for (0..2) |i|
        _ = try b.body(.{ .position = .{ -16 + real(i) * 20, 1.0, 4 }, .linear_velocity = .{ 120, -20, 0 }, .shape = shapes.pick(i), .motion_quality = 1, .object_layer = Layers.moving }, .activate);
}

/// Linear cast bodies with several collision steps per update and changing delta times
fn buildCCDCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const shapes = try BulletShapes.add(b);
            const wall = try b.box(.{ 0.05, 3, 3 }, 0);
            b.num_updates = 40;
            b.collision_steps = steps;
            _ = try addFloor(b, 40, -3);
            for (0..6) |i| {
                const z = real(i) * 6.0 - 15.0;
                _ = try b.body(.{ .position = .{ -15, 0, z }, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
                _ = try b.body(.{ .position = .{ 15, 0, z }, .rotation = axisAngle(.{ 0, 1, 0 }, 0.2 * @as(f64, @floatFromInt(i))), .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
                _ = try b.body(.{ .position = .{ 0, 0, z }, .linear_velocity = .{ 100.0 + 40.0 * @as(f32, @floatFromInt(i)), 5, 0 }, .restitution = if (i % 2 == 0) 0.6 else 0, .shape = shapes.pick(i), .motion_quality = 1, .object_layer = Layers.moving }, .activate);
            }
            b.at(15);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 30.0}, .count = steps + 1 });
            for (0..6) |i|
                try b.op(.set_linear_velocity, 3 + 3 * @as(u32, @intCast(i)), .{ .f = &.{ -150, 0, 10 } }); // The bullet of lane i (handle 0 is the floor)
        }
    }.build;
}

/// SaveState / RestoreState around CCD hits (the CCD manifolds of the contact cache), C++ restores the Zolt state and
/// Zolt the C++ state
fn buildCCDSaveRestore(b: *SceneBuilder) Allocator.Error!void {
    const sphere = try b.sphere(0.2);
    const wall = try b.box(.{ 0.05, 2, 2 }, 0);
    b.num_updates = 40;
    b.gravity = .{ 0, 0, 0 };
    for (0..3) |i| {
        const z = real(i) * 5.0;
        _ = try b.body(.{ .position = .{ 20, 0, z }, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
        _ = try b.body(.{ .position = .{ 0, 0, z }, .linear_velocity = .{ 100.0 + 20.0 * @as(f32, @floatFromInt(i)), 0, 0 }, .restitution = 0.5, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    }
    b.at(9);
    try b.sysOp(.snapshot, .{ .index = 0 }); // Just before the first hit
    b.at(12);
    try b.sysOp(.snapshot, .{ .index = 1 }); // The CCD manifolds of the hits are in the cache
    b.at(20);
    try b.sysOp(.restore_cross, .{ .index = 0 });
    b.at(25);
    try b.op(.set_linear_velocity, 1, .{ .f = &.{ -150, 0, 0 } });
    try b.sysOp(.restore_cross, .{ .index = 1 });
}

/// CCD hits far from the origin (the base offsets of the cast and the manifold, double precision)
fn buildCCDFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const sphere = try b.sphere(0.2);
    const box = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    const wall = try b.box(.{ 0.05, 2, 2 }, 0);
    b.num_updates = 30;
    const o: Real = 5000.0;
    _ = try b.body(.{ .position = .{ o + 20, 0, -o }, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ o, 0, -o }, .linear_velocity = .{ 150, 1, 0 }, .shape = sphere, .restitution = 0.8, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ o + 5, 0.5, -o + 1 }, .linear_velocity = .{ 120, 0, -5 }, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ o + 30, 0, -o + 0.5 }, .linear_velocity = .{ -150, 0, 0 }, .shape = box, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
}

/// Random linear cast and discrete bodies with random high velocities in a container of thin walls (seed `seed`,
/// `steps` collision steps), random listener, combine and filter modes, random operations (velocities, motion
/// qualities, impulses, activation)
fn buildCCDRandom(comptime seed: u32, comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.rng = 0xccd5eed * seed + 7;
            b.collision_steps = steps;
            b.num_updates = 60;
            b.max_contact_constraints = 4096;
            b.gravity = .{ b.float(-2, 2), b.float(-15, -5), b.float(-2, 2) };
            if (b.oneIn(2)) b.contact_listener_mode = .modify;
            if (b.oneIn(2)) b.combine_mode = .custom;
            if (b.oneIn(2)) b.sim_shape_filter_mode = .filter_spheres;
            if (b.oneIn(3)) b.settings.check_active_edges = 0;
            b.settings.time_before_sleep = b.float(0.2, 1.0);
            const shapes = try BulletShapes.add(b);
            const half: f32 = 8;
            const floor = try b.box(.{ half + 1, 0.1, half + 1 }, 0);
            const wall_x = try b.box(.{ 0.1, 4, half + 1 }, 0.02);
            const wall_z = try b.box(.{ half + 1, 4, 0.1 }, 0.02);
            const ceiling = try b.box(.{ half + 1, 0.1, half + 1 }, 0);
            _ = try b.body(.{ .position = .{ 0, -0.1, 0 }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
            _ = try b.body(.{ .position = .{ 0, 8.1, 0 }, .shape = ceiling, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
            for ([_]Real{ -half, half }) |x|
                _ = try b.body(.{ .position = .{ x, 4, 0 }, .shape = wall_x, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
            for ([_]Real{ -half, half }) |z|
                _ = try b.body(.{ .position = .{ 0, 4, z }, .shape = wall_z, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
            const num_bodies = 48;
            const first: u32 = b.num_handles;
            const F = ContactFlags;
            const flag_choices = [_]u32{ 0, 0, 0, F.zero_friction, F.bouncy, F.mass_scale, F.accept_each, F.reject_speculative, F.filter_spheres, F.sensor };
            for (0..num_bodies) |i| {
                const motion_type: u32 = if (b.oneIn(10)) 1 else 2;
                const activation: SceneBuilder.Activation = if (b.oneIn(8)) .dont_activate else .activate;
                const speed = if (b.oneIn(4)) b.float(1, 10) else b.float(50, 200);
                const dir = b.vec3(-1, 1);
                const w = b.vec3(-4, 4);
                _ = try b.body(.{
                    .position = .{ real(i % 6) * 2.4 - 6.0, 1.0 + real(i / 36) * 3.0 + b.float(0, 0.5), real((i / 6) % 6) * 2.4 - 6.0 },
                    .rotation = b.rotation(),
                    .linear_velocity = if (activation == .activate) .{ dir[0] * speed, dir[1] * speed * 0.5, dir[2] * speed } else @splat(0),
                    .angular_velocity = if (activation == .activate) w else @splat(0),
                    .friction = b.float(0, 1),
                    .restitution = if (b.oneIn(3)) b.float(0, 1) else 0,
                    .gravity_factor = if (b.oneIn(6)) b.float(0, 2) else 1.0,
                    .allowed_dofs = if (motion_type == 2 and b.oneIn(8)) 0b100011 else 0b111111,
                    .enhanced_internal_edge_removal = b.index(2),
                    .motion_type = motion_type,
                    .motion_quality = if (b.oneIn(3)) 0 else 1,
                    .allow_dynamic_or_kinematic = 1,
                    .is_sensor = if (b.oneIn(16)) 1 else 0,
                    .shape = shapes.pick(b.index(10)),
                    .user_data = ContactFlags.userData(flag_choices[b.index(flag_choices.len)], @intCast(i)),
                    .object_layer = Layers.moving,
                }, activation);
            }
            var u: u32 = 1;
            while (u < b.num_updates) : (u += 1 + b.index(5)) {
                b.at(u);
                for (0..b.index(4)) |_| {
                    const h = first + b.index(num_bodies);
                    const dir = b.vec3(-1, 1);
                    const speed = b.float(20, 180);
                    switch (b.index(8)) {
                        0 => try b.op(.set_linear_velocity, h, .{ .f = &.{ dir[0] * speed, dir[1] * speed * 0.5, dir[2] * speed } }),
                        1 => try b.op(.set_motion_quality, h, .{ .index = b.index(2) }),
                        2 => try b.op(.add_impulse, h, .{ .f = &.{ dir[0] * 30, dir[1] * 30, dir[2] * 30 } }),
                        3 => try b.op(.activate_body, h, .{}),
                        4 => try b.op(.set_user_data, h, .{ .index = h, .count = flag_choices[b.index(flag_choices.len)] }),
                        5 => try b.op(.set_friction_and_restitution, h, .{ .f = &.{ b.float(0, 1), b.float(0, 1) } }),
                        6 => try b.op(.set_motion_type, h, .{ .index = 1 + b.index(2), .activation = .activate }),
                        else => {
                            const w = b.vec3(-5, 5);
                            try b.op(.set_angular_velocity, h, .{ .f = &w });
                        },
                    }
                }
            }
        }
    }.build;
}

// -------------------------------------------------------------------------------------------------------------------
// Scenes of the later waves

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
