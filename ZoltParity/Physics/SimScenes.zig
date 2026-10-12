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
//!    `.blocked_by = .contacts` (or `.constraints`, `.hinge_constraints`, `.ccd_hits`, ...): its test returns
//!    error.SkipZigTest until `Blocker.isSupported` says so (flip it when the wave lands; the constraint blockers flip
//!    themselves), so the build summary counts it as "skipped", never silently. Meanwhile the test "SimScenes blocked
//!    scenes run on the C++ side" runs it on the C++ side with 0 and 3 worker threads and checks that it reaches what it
//!    is written for (contact callbacks, active constraints, its expect_* operations).
//!    `.blocked_by = .release_only` marks a scene that violates a JPH_ASSERT that Jolt's release build ignores: it runs
//!    without Zolt's asserts only (ReleaseFast).
//! 3. A scene with contacts needs nothing else (the C++ side and the Zolt side create every shape kind of `ShapeKind`,
//!    and the contact listener records are compared already). A new shape kind is a new `ShapeKind` value plus its
//!    creation in `SimulationReference.cpp` (MakeShape) and `SimulationParity.zig` (makeShapeSettings). Constraints:
//!    see "Constraint scenes" below.
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
//! `b.contact_listener_mode` / `b.combine_mode` / `b.sim_shape_filter_mode` (the body vs body collision function with
//! `b.sim_collide_mode`); in `ContactListenerMode.modify` and
//! `SimShapeFilterMode.filter_spheres` the bits of `ContactFlags` in the upper 32 bits of the user data of the two
//! bodies decide what the listener / filter does (the same code on both sides). A contact scene states what it
//! reaches in `SceneEntry.expect` (contact callbacks, removed contacts, sleeping bodies, FindCollisions jobs spawned
//! from inside jobs, a large island solved by the LargeIslandSplitter, a full manifold array in ProcessBodyPair, CCD
//! hits, several FindCCDContacts jobs); `runEntry` checks it. Rule: the contact constraint buffer and the caches must be
//! large enough (`max_contact_constraints`, `max_body_pairs`): which contacts are dropped when they are full depends on
//! the threads, in Jolt too.
//!
//! CCD scenes (Wave B R4, `Blocker.ccd_hits` is supported): linear cast bodies hit things (JobFindCCDContacts,
//! sSolveCCDContact, JobResolveCCDContacts). Rules: a linear cast shape needs an inner radius (no mesh, height field,
//! plane or mirrored ScaledShape), no body may move faster than its max linear velocity (500 m/s by default), and a CCD
//! body that is resolved after JobResolveCCDContacts activated a full batch of 64 woken up bodies must not have hit one
//! of the bodies of that batch (sGetCCDBody then reads past mActiveBodyToCCDBody, Zolt asserts in safe builds). More
//! than 64 sleeping bodies may be woken up by CCD hits in one collision step when each of them is hit by one CCD body
//! only (`ccd_wake_batch`).
//!
//! # Constraint scenes (Phase 6)
//!
//! Every Jolt two body constraint class can be described and created, and every one of its setters applied, on both
//! sides; the C++ side (SimulationReference.cpp) covers all of them since the Phase 6 harness, the Zolt side
//! (SimulationParity.zig) the classes that are ported.
//! - **Descriptions.** A `SimConstraint` is the ConstraintSettings members (enabled, priority, step overrides, draw size,
//!   user data) plus the members of the settings class in `i` / `r` / `f`, laid out per `ConstraintKind` (listed at each
//!   value; the layout of CSMakeSettings of the constraint parity, ConstraintsReference.cpp, plus the SetRatio helpers of
//!   Gear / RackAndPinion). `Desc.defaults(kind)` holds the defaults of Jolt's settings class (test "SimScenes
//!   Desc.defaults are the defaults of Jolt's constraint settings"), `Desc.point`, `Desc.fixedAuto`, `Desc.hinge`, ...
//!   fill in the geometry of the common setups. Body handle `invalid_handle` is Body::sFixedToWorld (body 1 or body 2).
//!   `constraint1` / `constraint2` are the constraint handles that a Gear / RackAndPinion gets with SetConstraints.
//! - **Creation and handles.** `b.constraint(desc)` adds a description and creates the constraint with
//!   `OpKind.create_constraint` (BodyInterface::CreateConstraint + PhysicsSystem::AddConstraint, returns the number of
//!   constraints); `b.createConstraintNoAdd(index)` creates one that is added later. Both return a constraint handle
//!   (the n-th created constraint): both worlds keep a reference to every created constraint, so a removed constraint
//!   can be added again.
//! - **Operations.** `b.constraintOp(kind, handle, .{ .i, .f, .r, .body })` adds an operation on a constraint: the
//!   generic ones (`ConstraintOpRange.generic`: enable, priority, step overrides, user data, NotifyShapeChanged,
//!   ResetWarmStart, remove / add one or a batch, ActivateConstraint, the hash of GetConstraintSettings, and
//!   `b.expectConstraintState(handle, state)`, the check that a constraint is (not) active at that point) and the
//!   setters of each class in the range of its group. They return the `ConstraintState` bits (enabled | active << 1) of
//!   the constraint, which both sides must agree on. `snapshot` takes the EStateRecorderState bits in i[0] (partial
//!   states, e.g. the constraints only).
//! - **Blockers.** A scene declares the constraint kinds it needs with its `Blocker` (`.constraints` for Fixed / Point,
//!   one per other kind, `.gear_constraints` / `.rack_and_pinion_constraints` include their hinges / slider). They are
//!   supported as soon as the Zolt class is ported (`ConstraintKind.isPorted`: the settings no longer declare
//!   `is_stub`), so nothing has to be flipped by hand; until then the scene is skipped and runs on the C++ side alone
//!   ("SimScenes blocked scenes run on the C++ side": two C++ worlds in lockstep, its expectations, finite results). A
//!   supported scene that creates a kind that is still a stub fails, and so does a ported class whose slot in
//!   SimulationParity.zig is still `StubSlot` (error.SceneNotSupported).
//! - **Expectations.** Every scene with constraints must have active constraints (they are solved);
//!   `Expect.active_constraints` asks for at least that many in one update. Bodies woken up by a constraint
//!   (BuildIslands) count in `update_activations`, a constraint island split by the LargeIslandSplitter shows in
//!   `large_island` when it falls asleep from the final batch of a split island.
//! - **Rules.** Jolt asserts on: two equal bodies or two invalid body handles in CreateConstraint, removing a constraint
//!   that is not in the system and adding one that is, a restore of a snapshot taken with other constraints in the
//!   system (RestoreState reads constraint indices), the limits of the class setters (HingeConstraint::SetLimits,
//!   SliderConstraint::SetLimits, SetDistance, SetLength, SetHalfConeAngle, SetTargetPathFraction, ...), a motor state
//!   other than Off with invalid motor settings, SetSliderAxis outside world space, step overrides of 256 or more.
//!   PathConstraint::GetConstraintSettings is not implemented in Jolt (no `constraint_settings_hash` on a path).
//!
//! ## Reserved numbers
//!
//! OpKind: 0..52 body and system operations, 53..99 reserved for operations of later phases that are not constraint
//! operations, 100..102 driver operations, 200..399 constraint operations in ranges of 20 (`ConstraintOpRange`):
//! 200..219 generic, 220..239 Fixed / Point, 240..259 Distance / Pulley, 260..279 Hinge, 280..299 Slider, 300..319
//! Cone / SwingTwist, 320..339 SixDOF, 340..359 Path, 360..379 Gear / RackAndPinion, 380..399 spare. ConstraintKind:
//! 0..11 (the order of the Phase 6 plan, not EConstraintSubType: `subType()` maps them), new kinds from 12. Blocker: one
//! value per kind. `Expect` fields are named, not numbered. The scene table and the builders have one region per
//! worker group, marked "(append the ... scenes here)".
//!
//! ## Recipe for a Phase 6 worker (adding a constraint class to the simulation parity)
//!
//! 1. Port the class (FixedConstraint.zig is the template); removing `is_stub` from its settings makes the scenes with
//!    its Blocker run instead of being skipped.
//! 2. SimulationParity.zig: replace `const XSlot = StubSlot;` of the group with a struct like `FixedPointSlot`:
//!    `makeSettings` (the members of the settings class from `i` / `r` / `f`, exactly SimMakeConstraintSettings of
//!    SimulationReference.cpp), `applyOp` (the operations of the group's range, exactly ApplyConstraintOp of the .cpp,
//!    `castConstraint` checks the subtype), `stateFields` (the SaveState layout of the class after `enabled`, for the
//!    state decoder) and for Gear / RackAndPinion `setConstraints`. The tests "SimScenes constraint descriptions are
//!    ConstraintsParity's" and every scene's `checkConstraintDescriptions` compare the binary state of the settings
//!    that the slot makes with the C++ ones.
//! 3. Run `zig build parity -Dtest-filter=constraints_<kind>`: the C++ scene of the group (`constraints_<kind>_ops`,
//!    every setter of the class) now runs against Zolt, bit exact in the whole thread matrix with the D12 sweeps.
//! 4. Add scenes of the group in its regions (builders in "---- <group> ----" below, entries in the scene table):
//!    limits and springs reached, motors in every EMotorState, the class in islands with other kinds and contacts, a
//!    LargeIslandSplitter island, sleeping bodies, cross save / restore with the class' state, far from the origin.
//!    A new operation of the class goes into the group's range (`OpKind`, ApplyConstraintOp in the .cpp, the slot's
//!    `applyOp`); a new generic operation into 214..219.
//! 5. Prove the scenes bite (break one operation of the class, a scene must fail) and run the parity in Debug,
//!    ReleaseFast and double precision.

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
    /// MeshShapeSettings(VertexList, IndexedTriangleList): `count` vertices at floats[data..][0 .. 3 * count], followed
    /// by params[0] triangles of 3 vertex indices each (stored as floats, exact below 2^24), params[1] the max number
    /// of triangles per leaf (mMaxTrianglesPerLeaf)
    indexed_mesh = 17,
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

/// Constraint kinds (Phase 6): every Jolt two body constraint class. The C++ side creates every kind and applies every
/// operation of `OpKind`; the Zolt side the kinds whose class is ported (`isPorted`, SimulationParity.zig's constraint
/// slots). A scene that creates a kind whose Zolt class is still a stub declares it with its `Blocker` and is skipped.
/// The description of each kind (`SimConstraint.i` / `r` / `f`) is listed per value.
pub const ConstraintKind = enum(u32) {
    /// FixedConstraintSettings: i[0] space, i[1] auto detect point; r[0..3] point 1, r[3..6] point 2; f[0..3] axis X1,
    /// f[3..6] axis Y1, f[6..9] axis X2, f[9..12] axis Y2
    fixed = 0,
    /// PointConstraintSettings: i[0] space; r[0..3] point 1, r[3..6] point 2
    point = 1,
    /// DistanceConstraintSettings: i[0] space; r[0..3] point 1, r[3..6] point 2; f[0] min distance, f[1] max distance,
    /// limits spring i[1] f[2..4]
    distance = 2,
    /// HingeConstraintSettings: i[0] space; r[0..3] point 1, r[3..6] point 2; f[0..3] hinge axis 1, f[3..6] normal axis
    /// 1, f[6..9] hinge axis 2, f[9..12] normal axis 2, f[12] limits min, f[13] limits max, limits spring i[1]
    /// f[14..16], f[16] max friction torque, motor i[2] f[17..23]
    hinge = 3,
    /// SliderConstraintSettings: i[0] space, i[1] auto detect point, i[2] != 0: SetSliderAxis(f[0..3]) instead of the
    /// four axes; r[0..3] point 1, r[3..6] point 2; f[0..3] slider axis 1, f[3..6] normal axis 1, f[6..9] slider axis 2,
    /// f[9..12] normal axis 2, f[12] limits min, f[13] limits max, limits spring i[3] f[14..16], f[16] max friction
    /// force, motor i[4] f[17..23]
    slider = 4,
    /// ConeConstraintSettings: i[0] space; r[0..3] point 1, r[3..6] point 2; f[0..3] twist axis 1, f[3..6] twist axis
    /// 2, f[6] half cone angle
    cone = 5,
    /// SwingTwistConstraintSettings: i[0] space, i[1] swing type; r[0..3] position 1, r[3..6] position 2; f[0..3] twist
    /// axis 1, f[3..6] plane axis 1, f[6..9] twist axis 2, f[9..12] plane axis 2, f[12] normal half cone angle, f[13]
    /// plane half cone angle, f[14] twist min angle, f[15] twist max angle, f[16] max friction torque, swing motor i[2]
    /// f[17..23], twist motor i[3] f[23..29]
    swing_twist = 6,
    /// SixDOFConstraintSettings: i[0] space, i[1] swing type; r[0..3] position 1, r[3..6] position 2; f[0..3] axis X1,
    /// f[3..6] axis Y1, f[6..9] axis X2, f[9..12] axis Y2, f[12..18] max friction, f[18..24] limit min, f[24..30] limit
    /// max, limits spring of translation axis k (k < 3) i[2 + k] f[30 + 2k .. 32 + 2k], motor of axis k (k < 6) i[5 + k]
    /// f[36 + 6k .. 42 + 6k]
    six_dof = 7,
    /// PathConstraintSettings with a PathConstraintPathHermite: i[1] path is looping, i[2] number of path points (1 ..
    /// 16), i[3] rotation constraint type, motor i[4]; f[0..3] path position, f[3..7] path rotation, f[7] path fraction,
    /// f[8] max friction force, motor f[9..15], point p: f[16 + 9p ..] position, tangent, normal
    path = 8,
    /// GearConstraintSettings: i[0] space, i[1] != 0: SetRatio(i[2], i[3]) (numbers of teeth) instead of f[6]; f[0..3]
    /// hinge axis 1, f[3..6] hinge axis 2, f[6] ratio. `SimConstraint.constraint1` / `constraint2`: the two hinges
    gear = 9,
    /// RackAndPinionConstraintSettings: i[0] space, i[1] != 0: SetRatio(i[2] teeth of the rack, f[7] rack length, i[3]
    /// teeth of the pinion) instead of f[6]; f[0..3] hinge axis, f[3..6] slider axis, f[6] ratio. Body 1 is the
    /// pinion, body 2 the rack; `SimConstraint.constraint1` / `constraint2`: the pinion hinge and the rack slider
    rack_and_pinion = 10,
    /// PulleyConstraintSettings: i[0] space; r[0..3] body point 1, r[3..6] fixed point 1, r[6..9] body point 2,
    /// r[9..12] fixed point 2; f[0] ratio, f[1] min length, f[2] max length
    pulley = 11,

    /// The Zolt settings class of the kind
    pub fn Settings(comptime self: ConstraintKind) type {
        return switch (self) {
            .fixed => zolt.FixedConstraintSettings,
            .point => zolt.PointConstraintSettings,
            .distance => zolt.DistanceConstraintSettings,
            .hinge => zolt.HingeConstraintSettings,
            .slider => zolt.SliderConstraintSettings,
            .cone => zolt.ConeConstraintSettings,
            .swing_twist => zolt.SwingTwistConstraintSettings,
            .six_dof => zolt.SixDOFConstraintSettings,
            .path => zolt.PathConstraintSettings,
            .gear => zolt.GearConstraintSettings,
            .rack_and_pinion => zolt.RackAndPinionConstraintSettings,
            .pulley => zolt.PulleyConstraintSettings,
        };
    }

    /// True when the Zolt class of the kind is ported: its settings no longer declare the stub marker `is_stub` (a
    /// path needs PathConstraintPathHermite too)
    pub fn isPorted(self: ConstraintKind) bool {
        return switch (self) {
            .path => !@hasDecl(zolt.PathConstraintSettings, "is_stub") and !@hasDecl(zolt.PathConstraintPathHermite, "is_stub"),
            inline else => |k| !@hasDecl(k.Settings(), "is_stub"),
        };
    }

    /// The EConstraintSubType value (the kind numbers of ConstraintsParity.zig / ConstraintsReference.cpp)
    pub fn subType(self: ConstraintKind) u32 {
        return switch (self) {
            .fixed => 0,
            .point => 1,
            .hinge => 2,
            .slider => 3,
            .distance => 4,
            .cone => 5,
            .swing_twist => 6,
            .six_dof => 7,
            .path => 8,
            .rack_and_pinion => 10,
            .gear => 11,
            .pulley => 12,
        };
    }
};

/// EConstraintSpace values for `SimConstraint.i[0]`
pub const ConstraintSpace = struct {
    pub const local_to_body_com: u32 = 0;
    pub const world_space: u32 = 1;
};

/// A two body constraint, created with `OpKind.create_constraint` (or `create_constraint_no_add`): the
/// ConstraintSettings members plus the members of the settings class in `i` / `r` / `f` (the layout of each kind is
/// listed at `ConstraintKind`; it is the layout of CSMakeSettings in ConstraintsReference.cpp, plus the SetRatio
/// helpers of Gear / RackAndPinion, checked by "SimScenes constraint descriptions are ConstraintsParity's"). A spring is
/// its mode in `i` and frequency / stiffness, damping in `f`; a motor is its spring mode in `i` and frequency /
/// stiffness, damping, min / max force limit, min / max torque limit in `f`.
pub const SimConstraint = extern struct {
    /// ConstraintKind
    kind: u32,
    /// Body handles, `invalid_handle`: Body::sFixedToWorld (BodyInterface::CreateConstraint with an invalid BodyID)
    body1: u32 = invalid_handle,
    body2: u32 = invalid_handle,
    /// Gear / RackAndPinion: SetConstraints(constraint handle `constraint1`, constraint handle `constraint2`) after the
    /// creation, before AddConstraint (`invalid_handle`: nullptr; SetConstraints is not called when both are invalid)
    constraint1: u32 = invalid_handle,
    constraint2: u32 = invalid_handle,
    /// The ConstraintSettings members (Jolt's defaults)
    enabled: u32 = 1,
    constraint_priority: u32 = 0,
    num_velocity_steps_override: u32 = 0,
    num_position_steps_override: u32 = 0,
    draw_constraint_size: f32 = 1.0,
    user_data: u64 = 0,
    /// The members of the settings class, per ConstraintKind
    i: [32]u32 = @splat(0),
    r: [12]Real = @splat(0),
    f: [192]f32 = @splat(0),
};

/// The result bits of a constraint operation: GetEnabled() | IsActive() << 1 of the constraint afterwards
pub const ConstraintState = struct {
    pub const enabled: u32 = 1;
    pub const active: u32 = 2;
    /// Enabled, but neither body is active (or neither is dynamic)
    pub const inactive: u32 = enabled;
};

/// The reserved OpKind ranges of the constraint operations (Phase 6, see the file header): every worker group adds its
/// operations in its own range only, so that branches never clash
pub const ConstraintOpRange = enum(u32) {
    /// Operations of every constraint (Constraint, TwoBodyConstraint, PhysicsSystem / BodyInterface constraint functions)
    generic = 200,
    /// FixedConstraint / PointConstraint
    fixed_point = 220,
    /// DistanceConstraint / PulleyConstraint
    distance_pulley = 240,
    /// HingeConstraint
    hinge = 260,
    /// SliderConstraint
    slider = 280,
    /// ConeConstraint / SwingTwistConstraint
    cone_swing_twist = 300,
    /// SixDOFConstraint
    six_dof = 320,
    /// PathConstraint (+ PathConstraintPathHermite)
    path = 340,
    /// GearConstraint / RackAndPinionConstraint
    gear_rack_and_pinion = 360,
    /// Spare (380..399), unassigned
    spare = 380,

    /// Each range has 20 numbers
    pub const size: u32 = 20;

    /// The range of an operation number (null: not a constraint operation)
    pub fn of(op: u32) ?ConstraintOpRange {
        if (op < 200 or op >= 400) return null;
        return @enumFromInt(200 + (op - 200) / size * size);
    }
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
    /// BodyInterface::CreateConstraint(constraints[index]) (+ SetConstraints of a Gear / RackAndPinion) +
    /// PhysicsSystem::AddConstraint (Phase 6). The constraint gets the next constraint handle (the harness keeps a
    /// reference to it, so it can be removed and added again); returns the number of constraints of the system
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
    /// SetUseManifoldReduction(count != 0) (invalidates the contact cache of the body)
    set_use_manifold_reduction = 49,
    /// Body::ApplyBodyCreationSettings(bodies[index], GetBroadPhaseLayerInterface()) under a BodyLockWrite (the body
    /// must not be in the broad phase)
    apply_body_creation_settings = 50,
    /// PhysicsSystem::WereBodiesInContact(body, handle `index`) | WereBodiesInContact(handle `index`, body) << 1 (changes
    /// nothing)
    were_bodies_in_contact = 51,
    /// Body::SetCollideKinematicVsNonDynamic(count != 0) under a BodyLockWrite (BodyInterface has no setter)
    set_collide_kinematic_vs_non_dynamic = 52,
    // 53..99: reserved for operations of later phases that are not constraint operations

    /// Driver: the following updates use delta time f[0] and `count` collision steps
    set_update = 100,
    /// Driver: both sides save their state in slot `index` (the EStateRecorderState bits i[0], 0: All)
    snapshot = 101,
    /// Driver: Zolt restores the C++ state of slot `index` and C++ the Zolt state (RestoreState, not validating)
    restore_cross = 102,

    // Constraint operations (Phase 6), in the reserved ranges of `ConstraintOpRange`. `index` is a constraint handle (the
    // n-th constraint created by create_constraint / create_constraint_no_add), `i` holds the integer arguments, `f` /
    // `r` the others. Unless noted otherwise they return the `ConstraintState` bits of the constraint afterwards. The
    // class operations check the subtype of the constraint (a scene error otherwise).

    // ---- ConstraintOpRange.generic (200..219) ----
    /// Constraint::SetEnabled(i[0] != 0)
    set_constraint_enabled = 200,
    /// Constraint::SetConstraintPriority(i[0])
    set_constraint_priority = 201,
    /// Constraint::SetNumVelocityStepsOverride(i[0]), SetNumPositionStepsOverride(i[1]) (both < 256)
    set_constraint_steps_override = 202,
    /// Constraint::SetUserData(i[0] | i[1] << 32) (the state filter of the end of a scene reads bit 0 of the user data);
    /// returns the FNV-1a hash (`hashBytes`) of the 8 bytes of GetUserData afterwards
    set_constraint_user_data = 203,
    /// Constraint::NotifyShapeChanged(body handle `body`, delta COM f[0..3]) (`invalid_handle`: an invalid BodyID, which
    /// is neither body of the constraint)
    notify_constraint_shape_changed = 204,
    /// Constraint::ResetWarmStart
    reset_constraint_warm_start = 205,
    /// PhysicsSystem::RemoveConstraint (the harness keeps its reference); returns the number of constraints of the system
    remove_constraint = 206,
    /// PhysicsSystem::AddConstraint of a constraint that is not in the system; returns the number of constraints
    add_constraint = 207,
    /// PhysicsSystem::RemoveConstraints of the handles [index, index + i[0]); returns the number of constraints
    remove_constraints = 208,
    /// PhysicsSystem::AddConstraints of the handles [index, index + i[0]); returns the number of constraints
    add_constraints = 209,
    /// BodyInterface::ActivateConstraint
    activate_constraint = 210,
    /// BodyInterface::CreateConstraint(constraints[index]) (+ SetConstraints) without adding it to the system (add it
    /// with add_constraint / add_constraints): the next constraint handle; returns its ConstraintState bits
    create_constraint_no_add = 211,
    /// A check of the scene: changes nothing, the driver fails unless the ConstraintState bits are i[0]
    /// (`SceneBuilder.expectConstraintState`)
    expect_constraint_state = 212,
    /// Returns the FNV-1a hash (`hashBytes`) of the SaveBinaryState bytes of GetConstraintSettings (not for a
    /// PathConstraint: Jolt's PathConstraint::GetConstraintSettings is not implemented)
    constraint_settings_hash = 213,
    // 214..219: free for generic constraint operations

    // ---- ConstraintOpRange.fixed_point (220..239) ----
    /// PointConstraint::SetPoint1(EConstraintSpace(i[0]), r)
    point_set_point1 = 220,
    /// PointConstraint::SetPoint2(EConstraintSpace(i[0]), r)
    point_set_point2 = 221,

    // ---- ConstraintOpRange.distance_pulley (240..259) ----
    /// DistanceConstraint::SetDistance(f[0], f[1])
    distance_set_distance = 240,
    /// DistanceConstraint::SetLimitsSpringSettings(spring i[0] f[0..2])
    distance_set_limits_spring_settings = 241,
    /// PulleyConstraint::SetLength(f[0], f[1])
    pulley_set_length = 250,

    // ---- ConstraintOpRange.hinge (260..279) ----
    /// HingeConstraint::SetMaxFrictionTorque(f[0])
    hinge_set_max_friction_torque = 260,
    /// HingeConstraint::SetMotorState(EMotorState(i[0]))
    hinge_set_motor_state = 261,
    /// HingeConstraint::SetTargetAngularVelocity(f[0])
    hinge_set_target_angular_velocity = 262,
    /// HingeConstraint::SetTargetAngle(f[0])
    hinge_set_target_angle = 263,
    /// HingeConstraint::SetTargetOrientationBS(f[0..4])
    hinge_set_target_orientation_bs = 264,
    /// HingeConstraint::SetLimits(f[0], f[1])
    hinge_set_limits = 265,
    /// HingeConstraint::SetLimitsSpringSettings(spring i[0] f[0..2])
    hinge_set_limits_spring_settings = 266,
    /// HingeConstraint::GetMotorSettings() = motor i[0] f[0..6]
    hinge_set_motor_settings = 267,

    // ---- ConstraintOpRange.slider (280..299) ----
    /// SliderConstraint::SetMaxFrictionForce(f[0])
    slider_set_max_friction_force = 280,
    /// SliderConstraint::SetMotorState(EMotorState(i[0]))
    slider_set_motor_state = 281,
    /// SliderConstraint::SetTargetVelocity(f[0])
    slider_set_target_velocity = 282,
    /// SliderConstraint::SetTargetPosition(f[0])
    slider_set_target_position = 283,
    /// SliderConstraint::SetLimits(f[0], f[1])
    slider_set_limits = 284,
    /// SliderConstraint::SetLimitsSpringSettings(spring i[0] f[0..2])
    slider_set_limits_spring_settings = 285,
    /// SliderConstraint::GetMotorSettings() = motor i[0] f[0..6]
    slider_set_motor_settings = 286,

    // ---- ConstraintOpRange.cone_swing_twist (300..319) ----
    /// ConeConstraint::SetHalfConeAngle(f[0])
    cone_set_half_cone_angle = 300,
    /// SwingTwistConstraint::SetNormalHalfConeAngle(f[0])
    swing_twist_set_normal_half_cone_angle = 305,
    /// SwingTwistConstraint::SetPlaneHalfConeAngle(f[0])
    swing_twist_set_plane_half_cone_angle = 306,
    /// SwingTwistConstraint::SetTwistMinAngle(f[0])
    swing_twist_set_twist_min_angle = 307,
    /// SwingTwistConstraint::SetTwistMaxAngle(f[0])
    swing_twist_set_twist_max_angle = 308,
    /// SwingTwistConstraint::SetMaxFrictionTorque(f[0])
    swing_twist_set_max_friction_torque = 309,
    /// SwingTwistConstraint::SetSwingMotorState(EMotorState(i[0]))
    swing_twist_set_swing_motor_state = 310,
    /// SwingTwistConstraint::SetTwistMotorState(EMotorState(i[0]))
    swing_twist_set_twist_motor_state = 311,
    /// SwingTwistConstraint::SetTargetAngularVelocityCS(f[0..3])
    swing_twist_set_target_angular_velocity_cs = 312,
    /// SwingTwistConstraint::SetTargetAngularVelocityBS(f[0..3])
    swing_twist_set_target_angular_velocity_bs = 313,
    /// SwingTwistConstraint::SetTargetOrientationCS(f[0..4])
    swing_twist_set_target_orientation_cs = 314,
    /// SwingTwistConstraint::SetTargetOrientationBS(f[0..4])
    swing_twist_set_target_orientation_bs = 315,
    /// SwingTwistConstraint::GetSwingMotorSettings() = motor i[0] f[0..6]
    swing_twist_set_swing_motor_settings = 316,
    /// SwingTwistConstraint::GetTwistMotorSettings() = motor i[0] f[0..6]
    swing_twist_set_twist_motor_settings = 317,

    // ---- ConstraintOpRange.six_dof (320..339) ----
    /// SixDOFConstraint::SetTranslationLimits(f[0..3], f[3..6])
    six_dof_set_translation_limits = 320,
    /// SixDOFConstraint::SetRotationLimits(f[0..3], f[3..6])
    six_dof_set_rotation_limits = 321,
    /// SixDOFConstraint::SetLimitsSpringSettings(EAxis(i[0]), spring i[1] f[0..2]) (a translation axis)
    six_dof_set_limits_spring_settings = 322,
    /// SixDOFConstraint::SetMaxFriction(EAxis(i[0]), f[0])
    six_dof_set_max_friction = 323,
    /// SixDOFConstraint::SetMotorState(EAxis(i[0]), EMotorState(i[1]))
    six_dof_set_motor_state = 324,
    /// SixDOFConstraint::SetTargetVelocityCS(f[0..3])
    six_dof_set_target_velocity_cs = 325,
    /// SixDOFConstraint::SetTargetAngularVelocityCS(f[0..3])
    six_dof_set_target_angular_velocity_cs = 326,
    /// SixDOFConstraint::SetTargetPositionCS(f[0..3])
    six_dof_set_target_position_cs = 327,
    /// SixDOFConstraint::SetTargetOrientationCS(f[0..4])
    six_dof_set_target_orientation_cs = 328,
    /// SixDOFConstraint::SetTargetOrientationBS(f[0..4])
    six_dof_set_target_orientation_bs = 329,
    /// SixDOFConstraint::GetMotorSettings(EAxis(i[0])) = motor i[1] f[0..6]
    six_dof_set_motor_settings = 330,

    // ---- ConstraintOpRange.path (340..359) ----
    /// PathConstraint::SetPath(i[0] != 0? a new PathConstraintPathHermite from the path of constraints[i[1]] (a
    /// description of kind path) : nullptr, path fraction f[0]) (a constraint without path is not active)
    path_set_path = 340,
    /// PathConstraint::SetMaxFrictionForce(f[0])
    path_set_max_friction_force = 341,
    /// PathConstraint::SetPositionMotorState(EMotorState(i[0]))
    path_set_position_motor_state = 342,
    /// PathConstraint::SetTargetVelocity(f[0])
    path_set_target_velocity = 343,
    /// PathConstraint::SetTargetPathFraction(f[0])
    path_set_target_path_fraction = 344,
    /// PathConstraint::GetPositionMotorSettings() = motor i[0] f[0..6]
    path_set_position_motor_settings = 345,

    // ---- ConstraintOpRange.gear_rack_and_pinion (360..379) ----
    /// GearConstraint::SetConstraints(constraint handle i[0], constraint handle i[1]) (`invalid_handle`: nullptr)
    gear_set_constraints = 360,
    /// RackAndPinionConstraint::SetConstraints(pinion constraint handle i[0], rack constraint handle i[1])
    rack_and_pinion_set_constraints = 370,

    // ---- ConstraintOpRange.spare (380..399): unassigned ----
};

/// An operation, applied before update `update`
pub const SimOp = extern struct {
    update: u32,
    kind: u32,
    body: u32 = invalid_handle,
    count: u32 = 0,
    activation: u32 = 0,
    index: u32 = 0,
    /// Integer arguments (the constraint operations, the state bits of `snapshot`)
    i: [4]u32 = @splat(0),
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
    /// Size of the TempAllocatorImpl of the world in bytes
    temp_allocator_size: u32,
    /// SimCollideMode (PhysicsSystem::SetSimCollideBodyVsBody)
    sim_collide_mode: u32,
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

/// The function that collides two bodies during the simulation (PhysicsSystem::SetSimCollideBodyVsBody): the three
/// functions of SensorTests' TestSimCollideBodyVsBody
pub const SimCollideMode = enum(u32) {
    /// PhysicsSystem::sDefaultSimCollideBodyVsBody
    default = 0,
    /// sCollideBodyVsBodyAll: the default function with EBackFaceMode::CollideWithBackFaces (hits with all triangles)
    all = 1,
    /// sCollideBodyVsBodyPerLeaf: CollideShapeVsShapePerLeaf with an AnyHitCollisionCollector (max 1 hit per leaf shape
    /// pair)
    per_leaf = 2,
    /// sCollideBodyVsBodyPerBody: CollisionDispatch::sCollideShapeVsShape into an AnyHitCollisionCollector (max 1 hit
    /// per body pair)
    per_body = 3,
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
    /// OnContactValidate (checked after reject_all): RejectContact when body 2 is a compound and the sub shape of the
    /// contact (mSubShapeID2) is a child with the compound user data `marked_child_user_data`, else the other flags
    /// decide (only this hit is rejected: the next contacts of the pair, e.g. the next hit of a CCD cast, are validated
    /// again)
    pub const reject_marked_child: u32 = 1 << 9;
    /// The compound user data (`SimCompoundChild.user_data`) of the children that `reject_marked_child` rejects
    pub const marked_child_user_data: u32 = 77;
    /// OnContactAdded / OnContactPersisted (after `mass_scale`): the inverse mass and inertia scale of each body is
    /// `massOverrideScale` of its user data (sGetInvMassScale of ContactListenerTests' TestMassOverride: 1, 0, 0.5 or 2;
    /// 0 gives the body infinite mass for the contact)
    pub const mass_override: u32 = 1 << 10;

    /// The inverse mass / inertia scale of a body with `user_data` for `mass_override` (sGetInvMassScale of
    /// UnitTests/Physics/ContactListenerTests.cpp, also SimMassOverrideScale in SimulationReference.cpp)
    pub fn massOverrideScale(user_data: u64) f32 {
        const index = (if ((user_data & 1) != 0) (user_data >> 1) else (user_data >> 3)) & 0b11;
        const mass_overrides = [_]f32{ 1.0, 0.0, 0.5, 2.0 };
        return mass_overrides[@intCast(index)];
    }

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
    /// Constraints in islands with FixedConstraint / PointConstraint (Phase 6 foundation, supported). A scene that
    /// creates other kinds names the blocker of its kind below.
    constraints,
    /// Linear cast bodies that hit something (Wave B R4, supported)
    ccd_hits,
    /// Soft bodies (Phase 9)
    soft_bodies,
    /// The scene violates a JPH_ASSERT that Jolt's release build (the reference library is built without asserts)
    /// ignores: it compares Jolt's release behavior and runs only without Zolt's asserts (ReleaseFast)
    release_only,

    // Phase 6, one blocker per constraint kind: supported as soon as the Zolt class is ported (ConstraintKind.isPorted,
    // nothing to flip by hand). A scene with several kinds names the blocker that needs all of them (gear: gear and
    // hinge; rack and pinion: rack and pinion, hinge and slider) or, for other combinations, the kind of the group that
    // runs it. SimulationParity's runEntry fails a supported scene that creates a kind that is still a stub.
    distance_constraints,
    pulley_constraints,
    hinge_constraints,
    slider_constraints,
    cone_constraints,
    swing_twist_constraints,
    six_dof_constraints,
    path_constraints,
    /// GearConstraint between two hinges: GearConstraint and HingeConstraint
    gear_constraints,
    /// RackAndPinionConstraint between a hinge and a slider: RackAndPinionConstraint, HingeConstraint, SliderConstraint
    rack_and_pinion_constraints,

    /// True when Zolt can run the scenes with this blocker (flip when the wave lands)
    pub fn isSupported(self: Blocker) bool {
        return switch (self) {
            .none, .contacts, .ccd_hits, .constraints => true,
            .release_only => !zolt.Core.enable_asserts,
            .soft_bodies => false,
            .distance_constraints => ConstraintKind.distance.isPorted(),
            .pulley_constraints => ConstraintKind.pulley.isPorted(),
            .hinge_constraints => ConstraintKind.hinge.isPorted(),
            .slider_constraints => ConstraintKind.slider.isPorted(),
            .cone_constraints => ConstraintKind.cone.isPorted(),
            .swing_twist_constraints => ConstraintKind.swing_twist.isPorted(),
            .six_dof_constraints => ConstraintKind.six_dof.isPorted(),
            .path_constraints => ConstraintKind.path.isPorted(),
            .gear_constraints => ConstraintKind.gear.isPorted() and ConstraintKind.hinge.isPorted(),
            .rack_and_pinion_constraints => ConstraintKind.rack_and_pinion.isPorted() and ConstraintKind.hinge.isPorted() and ConstraintKind.slider.isPorted(),
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
    /// The manifold array of ProcessBodyPair's ReductionCollideShapeCollector is full: a body pair gets
    /// `max_manifolds_per_body_pair` manifolds (OnContactAdded + OnContactPersisted calls) in one update, so its hits
    /// with other normals take the "array full" branch of AddHit (keep the manifolds with the deepest penetration). Only
    /// for scenes with one collision step per update (the count is per update).
    full_manifolds: bool = false,
    /// A linear cast body hits something: the contact cache of the (C++) state holds CCD manifolds after some update
    /// (ContactConstraintManager::OnCCDContactAdded, the worlds have a contact listener; only a hit whose sub shape pair
    /// has no discrete contact in the same update creates one)
    ccd_hits: bool = false,
    /// At least this many FindCCDContacts jobs run in one update (in some configuration: their number is
    /// min(ceil(CCD bodies / 4), the concurrency of the job system))
    ccd_jobs: u32 = 0,
    /// At least this many bodies are activated from inside one update (OnBodyActivated during PhysicsSystem::Update,
    /// not by the operations before it: woken up by contacts or CCD hits; more than 64 CCD hits on sleeping bodies in
    /// one collision step fill the batch of JobResolveCCDContacts)
    update_activations: u32 = 0,
    /// Phase 6: at least this many constraints are active (solved) in one update (Zolt side: the constraints of the
    /// system whose IsActive() is true before the update, as DetermineActiveConstraints sees them). Bodies woken up by
    /// a constraint (TwoBodyConstraint::BuildIslands activates the sleeping body of an active constraint) count in
    /// `update_activations` (in a scene without contacts they are the only activations from inside an update).
    active_constraints: u32 = 0,
};

/// The capacity of the manifold array of ProcessBodyPair's ReductionCollideShapeCollector (Jolt:
/// `using Manifolds = StaticArray<MyManifold, 32>`)
pub const max_manifolds_per_body_pair = 32;

pub const SceneEntry = struct {
    name: []const u8,
    build: *const fn (b: *SceneBuilder) Allocator.Error!void,
    blocked_by: Blocker = .none,
    /// D12 sweeps (every temp allocation and every createJob of the update fails once) before every
    /// `sweep_stride`-th update and before the first one, 0: none
    sweep_stride: u32 = 25,
    /// Creating the scene runs under std.testing.checkAllAllocationFailures (D12). Its cost is quadratic in the number
    /// of bodies: false for the PerformanceTest scenes with thousands of bodies, whose creation runs the same code as
    /// the other scenes.
    creation_sweep: bool = true,
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
    .{ .name = "contacts_many_manifolds", .build = buildManyManifolds, .blocked_by = .contacts, .expect = .{ .contacts = true, .full_manifolds = true } },
    .{ .name = "contacts_random_1", .build = buildContactRandom(1, 1), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_random_2", .build = buildContactRandom(2, 2), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_random_3", .build = buildContactRandom(3, 1), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_random_4", .build = buildContactRandom(4, 3), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_hello_world", .build = buildHelloWorld, .blocked_by = .contacts, .expect = .{ .contacts = true, .sleeping = true } },
    .{ .name = "contacts_mass_override", .build = buildMassOverride, .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_zero_delta_time", .build = buildZeroDeltaTime, .blocked_by = .contacts, .sweep_stride = 1, .expect = .{ .contacts = true } },
    .{ .name = "contacts_zero_sized_shapes", .build = buildZeroSizedShapes, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_cube_slide_active_edges", .build = buildDiscreteCubeSlide(1), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_cube_slide_all_edges", .build = buildDiscreteCubeSlide(0), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "contacts_sim_collide_all", .build = buildSimCollideBodyVsBody(.all), .blocked_by = .contacts, .sweep_stride = 3, .expect = .{ .contacts = true } },
    .{ .name = "contacts_sim_collide_per_leaf", .build = buildSimCollideBodyVsBody(.per_leaf), .blocked_by = .contacts, .sweep_stride = 3, .expect = .{ .contacts = true } },
    .{ .name = "contacts_sim_collide_per_body", .build = buildSimCollideBodyVsBody(.per_body), .blocked_by = .contacts, .sweep_stride = 3, .expect = .{ .contacts = true } },
    .{ .name = "contacts_were_bodies_in_contact", .build = buildWereBodiesInContact, .blocked_by = .contacts, .sweep_stride = 3, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "contacts_sensor_vs_static", .build = buildSensorVsStatic, .blocked_by = .contacts, .sweep_stride = 1, .expect = .{ .contacts = true, .removed = true } },
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
    .{ .name = "ccd_wake_batch", .build = buildCCDWakeBatch, .blocked_by = .ccd_hits, .sweep_stride = 1, .expect = .{ .contacts = true, .ccd_hits = true, .update_activations = ccd_wake_batch_targets } },
    .{ .name = "ccd_sensors", .build = buildCCDSensors, .blocked_by = .ccd_hits, .sweep_stride = 2, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_terrain", .build = buildCCDTerrain, .blocked_by = .ccd_hits, .sweep_stride = 3, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_collision_steps_2", .build = buildCCDCollisionSteps(2), .blocked_by = .ccd_hits, .sweep_stride = 4, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_collision_steps_3", .build = buildCCDCollisionSteps(3), .blocked_by = .ccd_hits, .sweep_stride = 4, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_save_restore_cross", .build = buildCCDSaveRestore, .blocked_by = .ccd_hits, .sweep_stride = 3, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_far_from_origin", .build = buildCCDFarFromOrigin, .blocked_by = .ccd_hits, .sweep_stride = 2, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_random_1", .build = buildCCDRandom(1, 1), .blocked_by = .ccd_hits, .sweep_stride = 5, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_random_2", .build = buildCCDRandom(2, 2), .blocked_by = .ccd_hits, .sweep_stride = 5, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_random_3", .build = buildCCDRandom(3, 1), .blocked_by = .ccd_hits, .sweep_stride = 5, .expect = .{ .contacts = true, .ccd_hits = true } },
    // The PerformanceTest scenes of the determinism check (milestone M6), exactly as ZoltPerformanceTest sets them up
    .{ .name = "perf_pyramid", .build = buildPerfPyramid, .blocked_by = .contacts, .sweep_stride = 0, .creation_sweep = false, .expect = .{ .contacts = true } },
    .{ .name = "perf_convex_vs_mesh", .build = buildPerfConvexVsMesh, .blocked_by = .contacts, .sweep_stride = 0, .creation_sweep = false, .expect = .{ .contacts = true } },
    .{ .name = "perf_high_speed", .build = buildPerfHighSpeed, .blocked_by = .ccd_hits, .sweep_stride = 0, .creation_sweep = false, .expect = .{ .contacts = true, .ccd_hits = true, .ccd_jobs = 4 } },
    .{ .name = "ccd_infinite_mass_override", .build = buildInfiniteMassOverride, .blocked_by = .ccd_hits, .sweep_stride = 3, .expect = .{ .contacts = true, .ccd_hits = true } },
    .{ .name = "ccd_cube_slide_active_edges", .build = buildLinearCastCubeSlide(1), .blocked_by = .ccd_hits, .sweep_stride = 1, .expect = .{ .contacts = true } },
    .{ .name = "ccd_cube_slide_all_edges", .build = buildLinearCastCubeSlide(0), .blocked_by = .ccd_hits, .sweep_stride = 1, .expect = .{ .contacts = true, .ccd_hits = true } },
    // Later waves (blocked until Zolt has the parts, see Blocker)
    // Scenes of the ported PhysicsTests (Wave C)
    .{ .name = "physics_tests_restitution_1", .build = buildTestsRestitution(1), .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "physics_tests_restitution_2", .build = buildTestsRestitution(2), .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "physics_tests_restitution_4", .build = buildTestsRestitution(4), .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "physics_tests_min_velocity_for_restitution_inelastic", .build = buildTestsMinVelocityForRestitution(1.01), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "physics_tests_min_velocity_for_restitution_elastic", .build = buildTestsMinVelocityForRestitution(0.99), .blocked_by = .contacts, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "physics_tests_penetration_slop_1", .build = buildTestsPenetrationSlop(1), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "physics_tests_penetration_slop_2", .build = buildTestsPenetrationSlop(2), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "physics_tests_penetration_slop_4", .build = buildTestsPenetrationSlop(4), .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "physics_tests_speculative_contacts", .build = buildTestsSpeculativeContacts, .blocked_by = .contacts, .sweep_stride = 2, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "physics_tests_speculative_no_hit", .build = buildTestsSpeculativeNoHit, .blocked_by = .contacts, .sweep_stride = 2, .expect = .{ .contacts = true } },
    .{ .name = "physics_tests_activate_during_step", .build = buildTestsActivateDuringStep, .blocked_by = .contacts, .sweep_stride = 1, .expect = .{ .contacts = true, .update_activations = 18 } },
    .{ .name = "physics_tests_broad_phase_layers", .build = buildTestsBroadPhaseLayers, .blocked_by = .contacts, .sweep_stride = 50, .expect = .{ .contacts = true, .sleeping = true } },
    .{ .name = "physics_tests_friction", .build = buildTestsFriction, .blocked_by = .contacts, .expect = .{ .contacts = true } },
    .{ .name = "physics_tests_allowed_dofs_vs_collision", .build = buildTestsAllowedDOFsVsCollision, .blocked_by = .contacts, .sweep_stride = 5, .expect = .{ .contacts = true } },
    .{ .name = "physics_tests_contacts_vs_simulation_islands", .build = buildTestsContactsVsSimulationIslands, .blocked_by = .contacts, .sweep_stride = 5, .expect = .{ .contacts = true } },
    .{ .name = "physics_tests_switch_use_manifold_reduction_1", .build = buildTestsSwitchUseManifoldReduction(1), .blocked_by = .contacts, .sweep_stride = 1, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "physics_tests_switch_use_manifold_reduction_2", .build = buildTestsSwitchUseManifoldReduction(2), .blocked_by = .contacts, .sweep_stride = 1, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "physics_tests_apply_body_creation_settings", .build = buildTestsApplyBodyCreationSettings, .blocked_by = .contacts, .sweep_stride = 1, .expect = .{ .contacts = true, .removed = true } },

    // Constraints (Phase 6), one region per worker group (append a group's scenes at the end of its region only)
    // ---- Fixed / Point (Phase 6 foundation) ----
    .{ .name = "constraints_chain", .build = buildConstraintChain, .blocked_by = .constraints },
    .{ .name = "constraints_fixed", .build = buildConstraintsFixed, .blocked_by = .constraints, .sweep_stride = 10 },
    .{ .name = "constraints_pendulum_hits_stack", .build = buildConstraintsPendulumHitsStack, .blocked_by = .constraints, .sweep_stride = 15, .expect = .{ .contacts = true, .removed = true } },
    .{ .name = "constraints_net_splitter", .build = buildConstraintsNet, .blocked_by = .constraints, .sweep_stride = 40, .expect = .{ .contacts = true, .sleeping = true, .large_island = true } },
    .{ .name = "constraints_sleep_and_wake", .build = buildConstraintsSleepAndWake, .blocked_by = .constraints, .sweep_stride = 20, .expect = .{ .sleeping = true } },
    .{ .name = "constraints_save_restore_and_settings", .build = buildConstraintsSaveRestore, .blocked_by = .constraints, .sweep_stride = 15 },
    .{ .name = "constraints_ragdoll_pile", .build = buildConstraintsRagdollPile, .blocked_by = .constraints, .sweep_stride = 50, .expect = .{ .contacts = true, .active_constraints = 88 } },
    .{ .name = "constraints_ragdolls_lying_splitter", .build = buildConstraintsRagdollsLying, .blocked_by = .constraints, .sweep_stride = 40, .expect = .{ .contacts = true, .sleeping = true, .large_island = true, .active_constraints = 95 } },
    .{ .name = "constraints_world_and_static_bodies", .build = buildConstraintsWorldAndStatic, .blocked_by = .constraints, .sweep_stride = 20, .expect = .{ .active_constraints = 7 } },
    .{ .name = "constraints_curtain_and_hub_splitter", .build = buildConstraintsCurtain, .blocked_by = .constraints, .sweep_stride = 40, .expect = .{ .sleeping = true, .large_island = true, .active_constraints = 276 + 131 } },
    .{ .name = "constraints_enable_disable_remove", .build = buildConstraintsEnableRemove, .blocked_by = .constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 15 } },
    .{ .name = "constraints_priorities_and_step_overrides", .build = buildConstraintsPriorities, .blocked_by = .constraints, .sweep_stride = 20, .expect = .{ .active_constraints = 18 } },
    .{ .name = "constraints_sleeping_bodies", .build = buildConstraintsSleepingBodies, .blocked_by = .constraints, .sweep_stride = 5, .expect = .{ .sleeping = true, .update_activations = 2 } },
    .{ .name = "constraints_partial_states", .build = buildConstraintsPartialStates, .blocked_by = .constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 5 } },
    .{ .name = "constraints_collision_steps_2", .build = buildConstraintsCollisionSteps(2), .blocked_by = .constraints, .sweep_stride = 10 },
    .{ .name = "constraints_collision_steps_3", .build = buildConstraintsCollisionSteps(3), .blocked_by = .constraints, .sweep_stride = 10 },
    .{ .name = "constraints_far_from_origin", .build = buildConstraintsFarFromOrigin, .blocked_by = .constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_random_1", .build = buildConstraintsRandom(1), .blocked_by = .constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_random_2", .build = buildConstraintsRandom(2), .blocked_by = .constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    // (append the Fixed / Point scenes here)
    // ---- Distance / Pulley ----
    .{ .name = "constraints_distance_ops", .build = buildDistanceOps, .blocked_by = .distance_constraints, .expect = .{ .active_constraints = 2 } },
    .{ .name = "constraints_pulley_ops", .build = buildPulleyOps, .blocked_by = .pulley_constraints, .expect = .{ .active_constraints = 2 } },
    // (append the distance / pulley scenes here)
    // ---- Hinge ----
    .{ .name = "constraints_hinge_ops", .build = buildHingeOps, .blocked_by = .hinge_constraints, .expect = .{ .active_constraints = 3 } },
    // (append the hinge scenes here)
    // ---- Slider ----
    .{ .name = "constraints_slider_ops", .build = buildSliderOps, .blocked_by = .slider_constraints, .expect = .{ .active_constraints = 3 } },
    // (append the slider scenes here)
    // ---- Cone / SwingTwist ----
    .{ .name = "constraints_cone_ops", .build = buildConeOps, .blocked_by = .cone_constraints, .expect = .{ .active_constraints = 2 } },
    .{ .name = "constraints_swing_twist_ops", .build = buildSwingTwistOps, .blocked_by = .swing_twist_constraints, .expect = .{ .active_constraints = 2 } },
    // (append the cone / swing twist scenes here)
    // ---- SixDOF ----
    .{ .name = "constraints_six_dof_ops", .build = buildSixDOFOps, .blocked_by = .six_dof_constraints, .expect = .{ .active_constraints = 2 } },
    .{ .name = "constraints_six_dof_limits_and_springs", .build = buildSixDOFLimits, .blocked_by = .six_dof_constraints, .sweep_stride = 20, .expect = .{ .active_constraints = 11 } },
    .{ .name = "constraints_six_dof_motors", .build = buildSixDOFMotors, .blocked_by = .six_dof_constraints, .sweep_stride = 20, .expect = .{ .active_constraints = 16 } },
    .{ .name = "constraints_six_dof_ragdolls", .build = buildSixDOFRagdolls, .blocked_by = .six_dof_constraints, .sweep_stride = 30, .expect = .{ .contacts = true, .removed = true, .active_constraints = 33 } },
    .{ .name = "constraints_six_dof_net_splitter", .build = buildSixDOFNetSplitter, .blocked_by = .six_dof_constraints, .sweep_stride = 40, .expect = .{ .contacts = true, .sleeping = true, .large_island = true, .active_constraints = 241 } },
    .{ .name = "constraints_six_dof_sleep_and_save_restore", .build = buildSixDOFSleepAndSaveRestore, .blocked_by = .six_dof_constraints, .sweep_stride = 10, .expect = .{ .sleeping = true, .update_activations = 1, .active_constraints = 3 } },
    .{ .name = "constraints_six_dof_collision_steps_2", .build = buildSixDOFCollisionSteps(2), .blocked_by = .six_dof_constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_six_dof_collision_steps_3", .build = buildSixDOFCollisionSteps(3), .blocked_by = .six_dof_constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_six_dof_far_from_origin", .build = buildSixDOFFarFromOrigin, .blocked_by = .six_dof_constraints, .sweep_stride = 20, .expect = .{ .contacts = true, .active_constraints = 6 } },
    .{ .name = "constraints_six_dof_random_1", .build = buildSixDOFRandom(1), .blocked_by = .six_dof_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_six_dof_random_2", .build = buildSixDOFRandom(2), .blocked_by = .six_dof_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_six_dof_random_3", .build = buildSixDOFRandom(3), .blocked_by = .six_dof_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    // (append the six dof scenes here)
    // ---- Path ----
    .{ .name = "constraints_path_ops", .build = buildPathOps, .blocked_by = .path_constraints, .expect = .{ .active_constraints = 2 } },
    // (append the path scenes here)
    // ---- Gear / RackAndPinion ----
    .{ .name = "constraints_gear_ops", .build = buildGearOps, .blocked_by = .gear_constraints, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_rack_and_pinion_ops", .build = buildRackAndPinionOps, .blocked_by = .rack_and_pinion_constraints, .expect = .{ .active_constraints = 6 } },
    // (append the gear / rack and pinion scenes here)
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
    temp_allocator_size: u32 = 10 * 1024 * 1024,
    sim_collide_mode: SimCollideMode = .default,

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
    /// Number of constraint handles created so far (create_constraint, create_constraint_no_add)
    num_constraint_handles: u32 = 0,
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
            .temp_allocator_size = self.temp_allocator_size,
            .sim_collide_mode = @intFromEnum(self.sim_collide_mode),
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

    /// MeshShapeSettings(vertices, indexed triangles) with `max_triangles_per_leaf` (the construction of the
    /// PerformanceTest scenes: no Indexify, the triangle order is the given one)
    pub fn indexedMesh(self: *SceneBuilder, vertices: []const [3]f32, triangles: []const [3]u32, max_triangles_per_leaf: u32) Allocator.Error!u32 {
        const data: u32 = @intCast(self.floats.items.len);
        try self.floats.ensureUnusedCapacity(self.allocator, 3 * (vertices.len + triangles.len));
        for (vertices) |v| self.floats.appendSliceAssumeCapacity(&v);
        for (triangles) |t| {
            for (t) |vertex_index| {
                std.debug.assert(vertex_index < 1 << 24); // Exact as a float
                self.floats.appendAssumeCapacity(@floatFromInt(vertex_index));
            }
        }
        return self.addShape(.{ .kind = @intFromEnum(ShapeKind.indexed_mesh), .params = params(&.{ @floatFromInt(triangles.len), @floatFromInt(max_triangles_per_leaf) }), .count = @intCast(vertices.len), .data = data });
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

    /// Add a constraint description (create it with `createConstraint`), returns its index
    pub fn addConstraint(self: *SceneBuilder, desc_value: SimConstraint) Allocator.Error!u32 {
        try self.constraints.append(self.allocator, desc_value);
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
        i: []const u32 = &.{},
        f: []const f32 = &.{},
        r: [3]Real = @splat(0),
    };

    pub const Activation = enum(u32) { activate = 0, dont_activate = 1 };

    /// Add an operation on body handle `body` (invalid_handle for system operations)
    pub fn op(self: *SceneBuilder, kind: OpKind, handle: u32, args: OpArgs) Allocator.Error!void {
        var o: SimOp = .{ .update = self.current_update, .kind = @intFromEnum(kind), .body = handle, .count = args.count, .activation = @intFromEnum(args.activation), .index = args.index, .r = args.r };
        @memcpy(o.i[0..args.i.len], args.i);
        @memcpy(o.f[0..args.f.len], args.f);
        try self.ops.append(self.allocator, o);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Constraints (Phase 6)

    pub const ConstraintOpArgs = struct {
        /// The body handle of notify_constraint_shape_changed
        body: u32 = invalid_handle,
        i: []const u32 = &.{},
        f: []const f32 = &.{},
        r: [3]Real = @splat(0),
    };

    /// Add a constraint operation (`OpKind` 200..399) on constraint handle `constraint_handle`
    pub fn constraintOp(self: *SceneBuilder, kind: OpKind, constraint_handle: u32, args: ConstraintOpArgs) Allocator.Error!void {
        std.debug.assert(ConstraintOpRange.of(@intFromEnum(kind)) != null);
        return self.op(kind, args.body, .{ .index = constraint_handle, .i = args.i, .f = args.f, .r = args.r });
    }

    /// Create the constraint of description `desc_index` and add it to the system (create_constraint), returns its
    /// constraint handle
    pub fn createConstraint(self: *SceneBuilder, desc_index: u32) Allocator.Error!u32 {
        try self.sysOp(.create_constraint, .{ .index = desc_index });
        self.num_constraint_handles += 1;
        return self.num_constraint_handles - 1;
    }

    /// Create the constraint of description `desc_index` without adding it (create_constraint_no_add), returns its
    /// constraint handle
    pub fn createConstraintNoAdd(self: *SceneBuilder, desc_index: u32) Allocator.Error!u32 {
        try self.op(.create_constraint_no_add, invalid_handle, .{ .index = desc_index });
        self.num_constraint_handles += 1;
        return self.num_constraint_handles - 1;
    }

    /// Add a constraint description and create the constraint (create_constraint), returns its constraint handle
    pub fn constraint(self: *SceneBuilder, desc_value: SimConstraint) Allocator.Error!u32 {
        return self.createConstraint(try self.addConstraint(desc_value));
    }

    /// The ConstraintState bits of a constraint must be `state` at this point (`ConstraintState.active`, ...): the driver
    /// checks it on both sides
    pub fn expectConstraintState(self: *SceneBuilder, constraint_handle: u32, state: u32) Allocator.Error!void {
        return self.constraintOp(.expect_constraint_state, constraint_handle, .{ .i = &.{state} });
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

/// The number of updates of HelloWorld/HelloWorld.cpp: its sphere is awake before each of them and falls asleep in the
/// last one, so HelloWorld prints this many "Step ..." lines (HelloWorldParity.zig checks it with HelloWorld's own loop)
pub const hello_world_steps: u32 = 48;

/// HelloWorld/HelloWorld.cpp (milestone M5): a sphere (radius 0.5, at y = 2, thrown down at 5 m/s) bounces on a floor box
/// until it falls asleep, with HelloWorld's system parameters, bodies and OptimizeBroadPhase. HelloWorldParity.zig runs
/// it with HelloWorld's own layer classes, listeners and loop; this entry adds the D12 sweeps and the thread matrix of
/// SimulationParity.
fn buildHelloWorld(b: *SceneBuilder) Allocator.Error!void {
    b.max_bodies = 1024;
    b.num_body_mutexes = 0;
    b.max_body_pairs = 1024;
    b.max_contact_constraints = 1024;
    b.num_updates = hello_world_steps;
    const floor_shape = try b.box(.{ 100, 1, 100 }, zolt.physics_settings.default_convex_radius); // BoxShapeSettings(Vec3(100, 1, 100))
    const sphere_shape = try b.sphere(0.5);
    _ = try b.body(.{ .position = .{ 0, -1, 0 }, .shape = floor_shape, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const sphere = try b.createAndAddBody(try b.addBodyDesc(.{ .position = .{ 0, 2, 0 }, .shape = sphere_shape, .object_layer = Layers.moving }), .activate);
    try b.op(.set_linear_velocity, sphere, .{ .f = &.{ 0, -5, 0 } });
    try b.sysOp(.optimize_broad_phase, .{});

    // HelloWorld's loop runs while the sphere is active: it is before the last update (Expect.sleeping: asleep after it)
    b.at(hello_world_steps - 1);
    try b.expectAwake(sphere);
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

/// Compounds of many small spheres that start pressed into a bumpy mesh (random heights) and a disc of spheres on a
/// static sphere: a body pair finds more contact normals than ReductionCollideShapeCollector holds manifolds, so its
/// "array full" branch keeps the manifolds with the deepest penetration (Expect.full_manifolds), the disc with ties
/// between the penetration depths
fn buildManyManifolds(b: *SceneBuilder) Allocator.Error!void {
    b.rng = 0x3a1f05;
    b.num_updates = 40;
    b.max_contact_constraints = 8192;

    // A static mesh of 24 x 24 cells of 0.25 m with random heights in [-0.25, 0.25)
    const n = 24;
    const cell: f32 = 0.25;
    const half = 0.5 * cell * @as(f32, n);
    var heights: [(n + 1) * (n + 1)]f32 = undefined;
    for (&heights) |*h| h.* = b.float(-0.25, 0.25);
    var triangles: std.ArrayList([9]f32) = .empty;
    defer triangles.deinit(b.allocator);
    for (0..n) |iz| {
        for (0..n) |ix| {
            const x0 = @as(f32, @floatFromInt(ix)) * cell - half;
            const z0 = @as(f32, @floatFromInt(iz)) * cell - half;
            const x1 = x0 + cell;
            const z1 = z0 + cell;
            const h00 = heights[iz * (n + 1) + ix];
            const h10 = heights[iz * (n + 1) + ix + 1];
            const h01 = heights[(iz + 1) * (n + 1) + ix];
            const h11 = heights[(iz + 1) * (n + 1) + ix + 1];
            try triangles.append(b.allocator, .{ x0, h00, z0, x0, h01, z1, x1, h11, z1 });
            try triangles.append(b.allocator, .{ x0, h00, z0, x1, h11, z1, x1, h10, z0 });
        }
    }
    const terrain = try b.mesh(triangles.items);

    // A static compound of 10 x 10 spheres and a mutable compound of 8 x 8 spheres (every sphere touches several
    // triangles with different normals)
    const sphere = try b.sphere(0.12);
    var children: [100]SimCompoundChild = undefined;
    for (&children, 0..) |*child, i|
        child.* = .{ .shape = sphere, .position = .{ @as(f32, @floatFromInt(i % 10)) * 0.22 - 0.99, 0, @as(f32, @floatFromInt(i / 10)) * 0.22 - 0.99 } };
    const static_compound = try b.compound(.static_compound, &children);
    for (children[0..64], 0..) |*child, i|
        child.* = .{ .shape = sphere, .position = .{ @as(f32, @floatFromInt(i % 8)) * 0.25 - 0.875, 0, @as(f32, @floatFromInt(i / 8)) * 0.25 - 0.875 } };
    const mutable_compound = try b.compound(.mutable_compound, children[0..64]);

    _ = try b.body(.{ .shape = terrain, .motion_type = 0, .friction = 0.6, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ -1.5, 0, -0.2 }, .rotation = axisAngle(.{ 0, 1, 0 }, 0.3), .shape = static_compound, .object_layer = Layers.moving }, .activate);
    _ = try b.body(.{ .position = .{ 1.5, 0, 0.3 }, .rotation = axisAngle(.{ 0, 1, 0 }, -0.5), .shape = mutable_compound, .object_layer = Layers.moving }, .activate);

    // Equal penetration depths: a flat disc of spheres (radius 0.25, on a grid of 0.25 m) lying on a static sphere of
    // radius 2. Every coordinate is a multiple of 0.25 and the children come in pairs (p, -p), so the center of mass of
    // the compound is exactly 0 and in the first update the spheres at (±x, ±z) get bit identical penetration depths
    // with normals more than 5 degrees apart: the "array full" branch meets ties between the manifold with the least
    // penetration and other manifolds or the new hit (`<` versus `<=` there decides which manifold is kept)
    const disc_sphere = try b.sphere(0.25);
    const big_sphere = try b.sphere(2);
    var disc: std.ArrayList(SimCompoundChild) = .empty;
    defer disc.deinit(b.allocator);
    try disc.append(b.allocator, .{ .shape = disc_sphere });
    for (1..5) |i| {
        for (0..5) |j| {
            // The (i, j) with i > 0, j >= 0 and their rotations by 90, 180 and 270 degrees cover every grid point but
            // the center once; i^2 + j^2 <= 18 are the spheres within the speculative contact distance of the big sphere
            if (i * i + j * j > 18) continue;
            const x = 0.25 * @as(f32, @floatFromInt(i));
            const z = 0.25 * @as(f32, @floatFromInt(j));
            for ([_][2]f32{ .{ x, z }, .{ -z, x } }) |p| {
                try disc.append(b.allocator, .{ .shape = disc_sphere, .position = .{ p[0], 0, p[1] } });
                try disc.append(b.allocator, .{ .shape = disc_sphere, .position = .{ -p[0], 0, -p[1] } });
            }
        }
    }
    const disc_compound = try b.compound(.static_compound, disc.items);
    _ = try b.body(.{ .position = .{ 10, 0, 0 }, .shape = big_sphere, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ 10, 2, 0 }, .shape = disc_compound, .object_layer = Layers.moving }, .activate);
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

/// The contact listener with CCD hits (ContactListenerMode.modify): OnContactValidate rejects all contacts of a pair
/// (the bullet goes through the wall), rejects speculative contacts (a cast hit has no negative depth), accepts each
/// contact, rejects the hit of one child of a compound (RejectContact: the cast goes on and the bullet hits the other
/// child); OnContactAdded makes the contact a sensor (no response), bouncy, frictionless, with mass scales (the bullet
/// created before and after its target: the body pair is swapped for the callback and the scales swapped back)
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
    // Static compounds of two parallel plates 0.8 m apart, the child at -x marked (reject_marked_child). The bullets
    // move 2.5 m per step and reach both plates in the same cast (step 8, they start it 0.35 m or more before the first
    // plate). From -x the marked plate is hit first: its hit is rejected (RejectContact, not stored, no early out) and
    // the next hit of the same body pair, the far plate, is validated again (AcceptAllContactsForThisBodyPair, with
    // accept_each AcceptContact): the bullet goes through the marked plate and stops at the far one. From +x the
    // unmarked plate is hit first and the marked one lies behind the early out fraction.
    const plate = try b.box(.{ 0.05, 2, 2 }, 0);
    const plates = try b.compound(.static_compound, &.{
        .{ .shape = plate, .position = .{ -0.4, 0, 0 }, .user_data = F.marked_child_user_data },
        .{ .shape = plate, .position = .{ 0.4, 0, 0 } },
    });
    const marked_bullets = [_]struct { x: Real, vx: f32, flags: u32 }{
        .{ .x = 0, .vx = 150, .flags = F.reject_marked_child },
        .{ .x = 0, .vx = 150, .flags = F.reject_marked_child | F.accept_each },
        .{ .x = 42, .vx = -150, .flags = F.reject_marked_child },
    };
    for (marked_bullets, 0..) |bullet, i| {
        const z = 54.0 + real(i) * 6.0;
        _ = try b.body(.{ .position = .{ 21, 0, z }, .shape = plates, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
        _ = try b.body(.{ .position = .{ bullet.x, 0, z }, .linear_velocity = .{ bullet.vx, 0, 0 }, .shape = shapes.pick(i), .user_data = ContactFlags.userData(bullet.flags, 30 + @as(u32, @intCast(i))), .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    }
}

/// Linear cast bodies that start penetrating: a sphere half inside a thick box moving deeper, a box touching a box and
/// two linear cast boxes starting in contact and moving towards each other (the discrete contacts at the start of the
/// step take their velocity into the boxes away before they are integrated, so they move less than the cast threshold),
/// a sphere inside a box moving out of it (it casts, the back face is ignored). Hits at fraction 0 (penetrating at the
/// start of the cast, the deepest point) happen in the other CCD scenes, where bodies lie against a wall and are pushed
/// into it.
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
    // A bullet that starts away from the box: the only hit of the scene without a discrete contact of the same sub shape
    // pair in the same update, so the contact cache records a CCD manifold (Expect.ccd_hits)
    _ = try b.body(.{ .position = .{ 0, 0, 10 }, .shape = thick, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .position = .{ -8, 0, 10 }, .linear_velocity = .{ 120, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
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

/// 70 sleeping dynamic boxes woken up by CCD hits in the same collision step (update 1): JobResolveCCDContacts
/// activates a full batch of 64 bodies while it still resolves the CCD bodies, the other 6 at the end
/// (Expect.update_activations). Each box is hit by its own bullet (the rule for CCD scenes at the top), the boxes are a
/// little further away in some lanes so that the CCD bodies are resolved in an order that differs from their body IDs.
fn buildCCDWakeBatch(b: *SceneBuilder) Allocator.Error!void {
    const sphere = try b.sphere(0.25);
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    b.num_updates = 8;
    b.gravity = .{ 0, 0, 0 };
    var boxes: [ccd_wake_batch_targets]u32 = undefined;
    for (&boxes, 0..) |*handle, i| {
        // The bullets move 2.5 m per step: after update 0 they are 0.75 m (plus up to 0.1 m) before their box
        const y = real(i / 10) * 2.5;
        const z = real(i % 10) * 2.5;
        handle.* = try b.body(.{ .position = .{ 4.0 + 0.01 * real((i * 7) % 11), y, z }, .shape = box, .object_layer = Layers.moving }, .dont_activate);
        _ = try b.body(.{ .position = .{ 0, y, z }, .linear_velocity = .{ 150, 0, 0 }, .shape = sphere, .motion_quality = 1, .object_layer = Layers.moving }, .activate);
    }
    b.at(1);
    for (boxes) |handle| try b.expectSleeping(handle);
    b.at(2);
    for (boxes) |handle| try b.expectAwake(handle);
}

/// Number of sleeping boxes of `buildCCDWakeBatch` (more than the batch of 64 bodies of JobResolveCCDContacts)
const ccd_wake_batch_targets = 70;

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
// The PerformanceTest scenes (milestone M6, ZoltPerformanceTest/): the setup of PyramidScene.h, ConvexVsMeshScene.h and
// HighSpeedScene.h with the arguments of the determinism check (-q=LinearCast), PerformanceTest.cpp's PhysicsSystem
// (10240 bodies, 0 body mutexes, 65536 body pairs, 20480 contact constraints, the default settings and gravity, 60 Hz,
// 1 collision step) and its OptimizeBroadPhase after StartTest. The layers NON_MOVING / MOVING of
// PerformanceTest/Layers.h behave like Layers.non_moving / Layers.moving here (only the broad phase layers differ, which
// does not change the simulation). The values are computed with Zolt's functions exactly as ZoltPerformanceTest does
// (Mt19937, Quat.random, Vec3.random, Sin, Cos), whose hashes equal Jolt's published ones. They run without the D12
// sweeps (thousands of bodies; the other scenes cover the sweeps) and for as many updates as keep the run time of the
// suite reasonable.

/// The PhysicsSystem of PerformanceTest.cpp (PerformanceTestScene's defaults)
fn perfSystem(b: *SceneBuilder) void {
    b.max_bodies = 10240;
    b.num_body_mutexes = 0;
    b.max_body_pairs = 65536;
    b.max_contact_constraints = 20480;
    b.temp_allocator_size = 32 * 1024 * 1024;
}

/// The rotation of a body description
fn quatToArray(q: zolt.Quat) [4]f32 {
    return .{ q.getX(), q.getY(), q.getZ(), q.getW() };
}

/// PyramidScene: 1240 boxes without convex radius that may not sleep, one large island (50 updates)
fn buildPerfPyramid(b: *SceneBuilder) Allocator.Error!void {
    perfSystem(b);
    b.num_updates = 50;

    // Floor
    const floor = try b.box(.{ 50.0, 1.0, 50.0 }, 0.0);
    _ = try b.createAndAddBody(try b.addBodyDesc(.{ .position = .{ 0.0, -1.0, 0.0 }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }), .dont_activate);

    const box_size: f32 = 2.0;
    const box_separation: f32 = 0.5;
    const half_box_size: f32 = 0.5 * box_size;
    const pyramid_height: i32 = 15;

    const box = try b.box(.{ half_box_size, half_box_size, half_box_size }, 0.0); // No convex radius to force more collisions

    // Pyramid
    var i: i32 = 0;
    while (i < pyramid_height) : (i += 1) {
        var j: i32 = @divTrunc(i, 2);
        while (j < pyramid_height - @divTrunc(i + 1, 2)) : (j += 1) {
            var k: i32 = @divTrunc(i, 2);
            while (k < pyramid_height - @divTrunc(i + 1, 2)) : (k += 1) {
                // PyramidScene computes the coordinates in float and converts them to Real
                const offset: f32 = if (i & 1 != 0) half_box_size else 0.0;
                const position: [3]Real = .{
                    @as(f32, @floatFromInt(-pyramid_height)) + box_size * @as(f32, @floatFromInt(j)) + offset,
                    1.0 + (box_size + box_separation) * @as(f32, @floatFromInt(i)),
                    @as(f32, @floatFromInt(-pyramid_height)) + box_size * @as(f32, @floatFromInt(k)) + offset,
                };
                _ = try b.createAndAddBody(try b.addBodyDesc(.{ .position = position, .shape = box, .object_layer = Layers.moving, .allow_sleeping = 0 }), .activate); // No sleeping to force the large island to stay awake
            }
        }
    }

    try b.sysOp(.optimize_broad_phase, .{});
}

/// ConvexVsMeshScene: 1764 convex shapes (linear cast) dropped on a 100 x 100 mesh terrain with 4 velocity steps and 1
/// position step (80 updates: the first bodies reach the terrain after about 55)
fn buildPerfConvexVsMesh(b: *SceneBuilder) Allocator.Error!void {
    perfSystem(b);
    b.num_updates = 80;

    // Load: the mesh
    const n = 100;
    const cell_size: f32 = 3.0;
    const max_height: f32 = 5.0;
    const center: f32 = @as(f32, n) * cell_size / 2;
    const vertices = try b.allocator.alloc([3]f32, (n + 1) * (n + 1));
    defer b.allocator.free(vertices);
    for (0..n + 1) |x| {
        for (0..n + 1) |z| {
            const fx: f32 = @floatFromInt(x);
            const fz: f32 = @floatFromInt(z);
            const height = zolt.trigonometry.sin(fx * 50.0 / @as(f32, n)) * zolt.trigonometry.cos(fz * 50.0 / @as(f32, n));
            vertices[z * (n + 1) + x] = .{ cell_size * fx, max_height * height, cell_size * fz };
        }
    }
    const triangles = try b.allocator.alloc([3]u32, n * n * 2);
    defer b.allocator.free(triangles);
    var next: usize = 0;
    for (0..n) |x| {
        for (0..n) |z| {
            const start: u32 = @intCast((n + 1) * z + x);
            triangles[next] = .{ start, start + n + 1, start + 1 };
            triangles[next + 1] = .{ start + 1, start + n + 1, start + n + 2 };
            next += 2;
        }
    }
    const mesh = try b.indexedMesh(vertices, triangles, 4);

    // Load: the other shapes
    const shapes = [_]u32{
        try b.box(.{ 0.5, 0.75, 1.0 }, 0.05),
        try b.sphere(0.5),
        try b.capsule(0.75, 0.5),
        try b.convexHull(&.{ .{ 0, 1, 0 }, .{ 1, 0, 0 }, .{ -1, 0, 0 }, .{ 0, 0, 1 }, .{ 0, 0, -1 } }, 0.05),
    };

    // StartTest: reduce the solver iteration count, the scene doesn't have any constraints
    b.settings.num_velocity_steps = 4;
    b.settings.num_position_steps = 1;

    // Create background
    _ = try b.createAndAddBody(try b.addBodyDesc(.{ .position = .{ -center, max_height, -center }, .shape = mesh, .motion_type = 0, .object_layer = Layers.non_moving, .friction = 0.5, .restitution = 0.6 }), .dont_activate);

    // Construct bodies
    var x: i32 = -10;
    while (x <= 10) : (x += 1) {
        for (shapes, 0..) |shape, y| {
            var z: i32 = -10;
            while (z <= 10) : (z += 1) {
                const position: [3]Real = .{ 7.5 * real(x), 15.0 + 2.0 * real(y), 7.5 * real(z) };
                _ = try b.createAndAddBody(try b.addBodyDesc(.{ .position = position, .shape = shape, .motion_quality = 1, .object_layer = Layers.moving, .friction = 0.5, .restitution = 0.6 }), .activate);
            }
        }
    }

    try b.sysOp(.optimize_broad_phase, .{});
}

/// HighSpeedScene: 5000 linear cast bodies at 250 m/s in a closed box (20 updates)
fn buildPerfHighSpeed(b: *SceneBuilder) Allocator.Error!void {
    perfSystem(b);
    b.num_updates = 20;

    // Load: the shapes
    const shape_size: f32 = 0.5;
    const capsule = try b.capsule(shape_size, 0.25 * shape_size);
    const compound_box = try b.box(.{ shape_size, 0.25 * shape_size, 0.25 * shape_size }, 0.05);
    const shapes = [_]u32{
        try b.sphere(shape_size),
        try b.box(.{ 0.5 * shape_size, shape_size, 1.0 * shape_size }, 0.05),
        try b.compound(.static_compound, &.{ .{ .shape = capsule, .position = .{ 0, shape_size, 0 } }, .{ .shape = compound_box, .position = .{ shape_size, 0, 0 } } }),
    };

    // StartTest
    const speed: f32 = 250.0;
    const wall_thickness: f32 = 0.2;
    const half_box_size: f32 = 50.0;
    const pos_range: f32 = 0.9 * half_box_size;

    // Create hollow box to enclose the objects
    const wall = try b.box(.{ half_box_size, wall_thickness, half_box_size }, 0.05);
    const rotation_x = quatToArray(zolt.Quat.rotation(zolt.Vec3.axisX(), 0.5 * zolt.math.pi));
    const rotation_z = quatToArray(zolt.Quat.rotation(zolt.Vec3.axisZ(), 0.5 * zolt.math.pi));
    const h: Real = half_box_size;
    const Wall = struct { position: [3]Real, rotation: [4]f32 };
    const walls = [_]Wall{
        .{ .position = .{ 0, -h, 0 }, .rotation = identity_rotation },
        .{ .position = .{ 0, h, 0 }, .rotation = identity_rotation },
        .{ .position = .{ 0, 0, -h }, .rotation = rotation_x },
        .{ .position = .{ 0, 0, h }, .rotation = rotation_x },
        .{ .position = .{ -h, 0, 0 }, .rotation = rotation_z },
        .{ .position = .{ h, 0, 0 }, .rotation = rotation_z },
    };
    for (walls) |w|
        _ = try b.createAndAddBody(try b.addBodyDesc(.{ .position = w.position, .rotation = w.rotation, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving, .friction = 1.0, .restitution = 1.0 }), .dont_activate);

    // Create many instances with high velocity (HighSpeedScene's random_float lambda)
    const randomFloat = struct {
        fn f(rnd: *zolt.Mt19937, min: f32, max: f32) f32 {
            return min + @as(f32, @floatFromInt(rnd.next() - zolt.Mt19937.min_value)) * (max - min) / @as(f32, @floatFromInt(zolt.Mt19937.max_value - zolt.Mt19937.min_value));
        }
    }.f;
    var rnd = zolt.Mt19937.init(zolt.Mt19937.default_seed);
    for (0..5000) |i| {
        // In HighSpeedScene's order: x, y, z, rotation, friction, restitution, velocity
        const x = randomFloat(&rnd, -pos_range, pos_range);
        const y = randomFloat(&rnd, -pos_range, pos_range);
        const z = randomFloat(&rnd, -pos_range, pos_range);
        const rotation = quatToArray(zolt.Quat.random(&rnd));
        const friction = randomFloat(&rnd, 0.5, 1.0);
        const restitution = randomFloat(&rnd, 0.9, 1.0);
        const velocity = zolt.Vec3.random(&rnd).mulScalar(speed);
        _ = try b.createAndAddBody(try b.addBodyDesc(.{
            .position = .{ x, y, z },
            .rotation = rotation,
            .linear_velocity = .{ velocity.getX(), velocity.getY(), velocity.getZ() },
            .shape = shapes[i % shapes.len],
            .motion_quality = 1,
            .object_layer = Layers.moving,
            .friction = friction,
            .restitution = restitution,
        }), .activate);
    }

    try b.sysOp(.optimize_broad_phase, .{});
}

// Scenes of UnitTests/Physics/PhysicsTests.cpp (Wave C): ZoltTests/Physics/PhysicsTests.zig checks these scenarios
// with Jolt's tolerances, the scenes compare them with Jolt bit for bit. The bodies are made like PhysicsTestContext
// makes them (no damping), the floors like CreateFloor (a box without convex radius, top at y = 0).

/// The body description of PhysicsTestContext::CreateBody (the shape settings overload): no linear and angular damping
fn testBody(desc_value: SimBody) SimBody {
    var result = desc_value;
    result.linear_damping = 0;
    result.angular_damping = 0;
    return result;
}

/// PhysicsTestContext::CreateFloor at (x, z) with `half_extent` instead of 100, returns its handle
fn addTestFloor(b: *SceneBuilder, x: Real, z: Real, half_extent: f32, friction: f32, restitution: f32) Allocator.Error!u32 {
    const floor = try b.box(.{ half_extent, 1, half_extent }, 0);
    return b.body(.{ .position = .{ x, -1, z }, .shape = floor, .motion_type = 0, .friction = friction, .restitution = restitution, .object_layer = Layers.non_moving }, .dont_activate);
}

/// TestPhysicsCollisionElastic, TestPhysicsCollisionPartiallyElastic, TestPhysicsCollisionElasticDynamic,
/// TestPhysicsCollisionInelastic: spheres with restitution 1, 0.9 and 0 bouncing on the floor, two spheres with
/// restitution 1 colliding in the air (with `steps` collision steps of 1/60 s per update, like the tests)
fn buildTestsRestitution(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const ball = try b.sphere(1.0);
            const big_ball = try b.sphere(2.0);
            b.delta_time = @as(f32, @floatFromInt(steps)) / 60.0;
            b.collision_steps = steps;
            b.num_updates = 300 / steps;
            _ = try addTestFloor(b, 0, 0, 100, 0.2, 0);
            _ = try b.body(testBody(.{ .position = .{ -20, 0.9999 + 4.905, 0 }, .shape = ball, .restitution = 1.0, .object_layer = Layers.moving }), .activate);
            _ = try b.body(testBody(.{ .position = .{ -10, 0.9999 + 4.905, 0 }, .shape = ball, .restitution = 0.0, .object_layer = Layers.moving }), .activate);
            _ = try b.body(testBody(.{ .position = .{ 0, 10, 0 }, .shape = big_ball, .restitution = 0.9, .object_layer = Layers.moving }), .activate);
            _ = try b.body(testBody(.{ .position = .{ 18, 30, 0 }, .linear_velocity = .{ 5, 0, 0 }, .shape = ball, .restitution = 1.0, .object_layer = Layers.moving }), .activate);
            _ = try b.body(testBody(.{ .position = .{ 22, 30, 0 }, .linear_velocity = .{ -10, 0, 0 }, .shape = ball, .restitution = 1.0, .object_layer = Layers.moving }), .activate);
        }
    }.build;
}

/// TestMinVelocityForRestitution: two spheres with restitution 1 without gravity, the speed of the first one is 1,
/// PhysicsSettings::mMinVelocityForRestitution is `min_velocity` (1.01: inelastic, 0.99: elastic)
fn buildTestsMinVelocityForRestitution(comptime min_velocity: f32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const ball = try b.sphere(1.0);
            b.gravity = .{ 0, 0, 0 };
            b.settings.min_velocity_for_restitution = min_velocity;
            b.num_updates = 150;
            _ = try b.body(testBody(.{ .position = .{ 0, -2, 0 }, .linear_velocity = .{ 0, 1, 0 }, .shape = ball, .restitution = 1.0, .object_layer = Layers.moving }), .activate);
            _ = try b.body(testBody(.{ .position = .{ 0, 2, 0 }, .shape = ball, .restitution = 1.0, .object_layer = Layers.moving }), .activate);
        }
    }.build;
}

/// TestPhysicsPenetrationSlop1, 2 and 3: boxes that start penetrating the floor by the penetration slop, by twice the
/// slop and by a tenth of it (with `steps` collision steps of 1/60 s per update, like the tests)
fn buildTestsPenetrationSlop(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const box = try b.box(.{ 1, 1, 1 }, 0.05);
            b.delta_time = @as(f32, @floatFromInt(steps)) / 60.0;
            b.collision_steps = steps;
            b.num_updates = 60 / steps;
            const slop = b.settings.penetration_slop;
            _ = try addTestFloor(b, 0, 0, 100, 0.2, 0);
            _ = try b.body(testBody(.{ .position = .{ -10, 1.0 - slop, 0 }, .shape = box, .object_layer = Layers.moving }), .activate);
            _ = try b.body(testBody(.{ .position = .{ 0, 1.0 - 2.0 * slop, 0 }, .shape = box, .object_layer = Layers.moving }), .activate);
            _ = try b.body(testBody(.{ .position = .{ 10, 1.0 - 0.1 * slop, 0 }, .shape = box, .object_layer = Layers.moving }), .activate);
        }
    }.build;
}

/// TestPhysicsOutsideOfSpeculativeContactDistance and the TestPhysicsInsideSpeculativeContactDistance cases without
/// gravity, each on its own floor: a box and a sphere moving 1 m per update towards the floor from just outside / just
/// inside the speculative contact distance (without and with restitution), a sensor sphere, and a box and a sphere
/// moving away from the floor
fn buildTestsSpeculativeContacts(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 1, 1, 1 }, 0.05);
    const ball = try b.sphere(1.0);
    b.gravity = .{ 0, 0, 0 };
    b.num_updates = 10;
    const speculative_contact_distance = b.settings.speculative_contact_distance;
    const down: f32 = -1.0 / b.delta_time;
    const Case = struct { distance: f32, velocity: f32, restitution: f32, sensor: bool };
    const cases = [_]Case{
        .{ .distance = 1.1 * speculative_contact_distance, .velocity = down, .restitution = 0, .sensor = false }, // Outside
        .{ .distance = 0.9 * speculative_contact_distance, .velocity = down, .restitution = 0, .sensor = false }, // Inside, no restitution
        .{ .distance = 0.9 * speculative_contact_distance, .velocity = down, .restitution = 1, .sensor = false }, // Inside, with restitution
        .{ .distance = 0.9 * speculative_contact_distance, .velocity = down, .restitution = 0, .sensor = true }, // Inside, sensor
        .{ .distance = 0.9 * speculative_contact_distance, .velocity = -down, .restitution = 1, .sensor = false }, // Inside, moving away
    };
    for (cases, 0..) |case, i| {
        const x = real(i) * 25.0;
        _ = try addTestFloor(b, x, 0, 8, 0.2, 0);
        const y: f32 = 1.0 + case.distance;
        if (!case.sensor)
            _ = try b.body(testBody(.{ .position = .{ x, y, 0 }, .linear_velocity = .{ 0, case.velocity, 0 }, .shape = box, .restitution = case.restitution, .object_layer = Layers.moving }), .activate);
        _ = try b.body(testBody(.{ .position = .{ x + 5, y, 0 }, .linear_velocity = .{ 0, case.velocity, 0 }, .shape = ball, .restitution = case.restitution, .is_sensor = @intFromBool(case.sensor), .object_layer = Layers.moving }), .activate);
    }
}

/// TestPhysicsInsideSpeculativeContactDistanceNoHit: a sphere with restitution 1 inside the speculative contact
/// distance of a floor with restitution 1, too slow to touch it in one update, mMinVelocityForRestitution = 0
fn buildTestsSpeculativeNoHit(b: *SceneBuilder) Allocator.Error!void {
    const ball = try b.sphere(1.0);
    b.gravity = .{ 0, 0, 0 };
    b.settings.min_velocity_for_restitution = 0.0;
    b.num_updates = 10;
    const distance_above_floor = 0.9 * b.settings.speculative_contact_distance;
    _ = try addTestFloor(b, 0, 0, 100, 0.2, 1.0);
    _ = try b.body(testBody(.{ .position = .{ 0, 1.0 + distance_above_floor, 0 }, .linear_velocity = .{ 0, -0.9 * distance_above_floor / b.delta_time, 0 }, .shape = ball, .restitution = 1.0, .object_layer = Layers.moving }), .activate);
}

/// TestPhysicsActivateDuringStep: two rows of sleeping boxes that penetrate their neighbors by the penetration slop
/// (one row created left to right, one right to left), the left most box of each row gets a velocity that would make
/// it tunnel through the second box: the whole row is woken up and collides in the same update
fn buildTestsActivateDuringStep(b: *SceneBuilder) Allocator.Error!void {
    const box_extent: f32 = 0.5;
    const box = try b.box(.{ box_extent, box_extent, box_extent }, 0.05);
    b.num_updates = 6;
    const penetration_slop = b.settings.penetration_slop;
    const num_bodies = 10;
    var first: [2]u32 = undefined;
    for (0..2) |row| {
        for (0..num_bodies) |k| {
            const i = if (row == 0) k else num_bodies - 1 - k;
            const x: f32 = @as(f32, @floatFromInt(i)) * (2.0 * box_extent - penetration_slop);
            const h = try b.body(testBody(.{ .position = .{ x, 0, real(row) * 10.0 }, .shape = box, .object_layer = Layers.moving }), .dont_activate);
            if (i == 0) first[row] = h;
        }
    }
    for (first) |h|
        try b.op(.set_linear_velocity, h, .{ .f = &.{ 500, 0, 0 } });
}

/// TestPhysicsBroadPhaseLayers: boxes in the MOVING, HQ_DEBRIS and LQ_DEBRIS layers falling on each other (without
/// penetration slop), then the HQ_DEBRIS boxes change to LQ_DEBRIS (another broad phase layer) and later the MOVING
/// boxes to HQ_DEBRIS (the same broad phase layer)
fn buildTestsBroadPhaseLayers(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    b.settings.penetration_slop = 0.0;
    b.num_updates = 900;
    _ = try addTestFloor(b, 0, 0, 100, 0.2, 0);
    const layers = [_]u32{ Layers.moving, Layers.moving, Layers.hq_debris, Layers.hq_debris, Layers.lq_debris, Layers.lq_debris };
    var handles: [layers.len]u32 = undefined;
    for (layers, &handles, 0..) |layer, *h, i|
        h.* = try b.body(testBody(.{ .position = .{ 0, real(i + 1), 0 }, .shape = box, .object_layer = layer }), .activate);
    b.at(300);
    for (handles) |h|
        try b.expectSleeping(h);
    for (handles[2..4]) |h|
        try b.op(.set_object_layer, h, .{ .index = Layers.lq_debris });
    for (handles[2..4]) |h|
        try b.op(.activate_body, h, .{});
    b.at(600);
    for (handles) |h|
        try b.expectSleeping(h);
    for (handles[0..2]) |h|
        try b.op(.set_object_layer, h, .{ .index = Layers.hq_debris });
    for (handles[0..2]) |h|
        try b.op(.activate_body, h, .{});
}

/// TestFriction (boxes sliding over a floor in 12 directions) and TestAngularFriction (a box spinning on a floor), each
/// on its own floor
fn buildTestsFriction(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 1, 1, 1 }, 0.05);
    b.num_updates = 60;
    _ = try addTestFloor(b, 0, 0, 150, 0.9, 0);
    for (0..12) |i| {
        // 60 m apart: a box slides about 24 m
        const angle = std.math.degreesToRadians(30.0 * @as(f64, @floatFromInt(i)));
        const v: [3]f32 = .{ @floatCast(20.0 * @sin(angle)), 0, @floatCast(20.0 * @cos(angle)) };
        const x = (real(i % 4) - 1.5) * 60.0;
        const z = (real(i / 4) - 1.0) * 60.0;
        _ = try b.body(.{ .position = .{ x, 0.999, z }, .linear_velocity = v, .shape = box, .friction = 0.8, .linear_damping = 0, .object_layer = Layers.moving }, .activate);
    }
    _ = try addTestFloor(b, 400, 0, 30, 0.2, 0);
    _ = try b.body(.{ .position = .{ 410, 0.999, 10 }, .angular_velocity = .{ 0, 10, 0 }, .shape = box, .friction = 0.3, .angular_damping = 0, .object_layer = Layers.moving }, .activate);
}

/// TestAllowedDOFsVsCollision (a box that can only rotate around Y penetrates the floor: the contact has an infinite
/// effective mass and must not stop the rotation) and TestAllowedDOFsVsSmallRotation (a 2D box with a denormal
/// rotation around the locked X axis: a denormal effective mass for the angular friction)
fn buildTestsAllowedDOFsVsCollision(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 1, 1, 1 }, 0.05);
    const small_box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    b.num_updates = 30;
    _ = try addTestFloor(b, 0, 0, 100, 1.0, 0);
    const rotating = try b.body(.{ .position = .{ 0, @as(f32, 0.99), 0 }, .shape = box, .allowed_dofs = 0b010000, .angular_damping = 0, .override_mass_properties = 1, .mass = 1.0, .friction = 1.0, .object_layer = Layers.moving }, .activate);
    try b.op(.add_torque, rotating, .{ .f = &.{ 0, 100, 0 } });
    _ = try b.body(.{ .position = .{ 10, 0.5 - real(b.settings.penetration_slop), 0 }, .rotation = .{ 1.0e-20, 0, 0, 1 }, .linear_velocity = .{ 0, -1, 0 }, .shape = small_box, .allowed_dofs = 0b100011, .friction = 1.0, .object_layer = Layers.moving }, .activate);
}

/// TestContactsVsSimulationIslands: rows of three touching boxes with every combination of static, kinematic and
/// dynamic motion types
fn buildTestsContactsVsSimulationIslands(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 1, 1, 1 }, 0.05);
    b.num_updates = 20;
    var cell: u32 = 0;
    for (0..3) |m1| {
        for (0..3) |m2| {
            for (0..3) |m3| {
                const p = gridPosition(cell, 20, 0);
                const motion_types = [_]u32{ @intCast(m1), @intCast(m2), @intCast(m3) };
                const offsets = [_]f32{ 0, 1.99, 3.98 };
                for (motion_types, offsets) |m, dx|
                    _ = try b.body(testBody(.{ .position = .{ p[0] + dx, p[1], p[2] }, .shape = box, .motion_type = m, .object_layer = Layers.moving }), .activate);
                cell += 1;
            }
        }
    }
}

/// TestSwitchUseManifoldReduction: a compound of four boxes embedded in the floor, manifold reduction is switched off
/// and on again (BodyInterface::SetUseManifoldReduction flags the contact cache of the body invalid until the next
/// update; the sweeps of the updates after the switches check that an update that runs out of memory keeps the flag,
/// with 2 collision steps also after the first step reset it)
fn buildTestsSwitchUseManifoldReduction(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.delta_time = @as(f32, @floatFromInt(steps)) / 60.0;
            b.collision_steps = steps;
            try buildSwitchUseManifoldReductionScene(b);
        }
    }.build;
}

fn buildSwitchUseManifoldReductionScene(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 2, 2, 2 }, 0.05);
    const compound = try b.compound(.static_compound, &.{
        .{ .shape = box, .position = .{ 5, 0, 0 } },
        .{ .shape = box, .position = .{ -5, 0, 0 } },
        .{ .shape = box, .position = .{ 0, 0, 5 } },
        .{ .shape = box, .position = .{ 0, 0, -5 } },
    });
    b.num_updates = 8;
    _ = try addTestFloor(b, 0, 0, 100, 0.2, 0);
    const body_handle = try b.body(.{ .position = .{ 0, 1.99, 0 }, .shape = compound, .use_manifold_reduction = 1, .object_layer = Layers.moving }, .activate);
    b.at(1);
    try b.op(.set_use_manifold_reduction, body_handle, .{ .count = 0 });
    b.at(2);
    try b.op(.set_use_manifold_reduction, body_handle, .{ .count = 1 });
    b.at(5);
    try b.op(.set_use_manifold_reduction, body_handle, .{ .count = 0 });
}

/// TestApplyBodyCreationSettings: a box in contact with the floor is removed, gets non default settings for
/// everything (Body::ApplyBodyCreationSettings: a kinematic linear cast sensor sphere in another layer), is added again
/// (the contact persists) and finally destroyed
fn buildTestsApplyBodyCreationSettings(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 1, 1, 1 }, 0.05);
    const ball = try b.sphere(1.0);
    b.num_updates = 5;
    _ = try addTestFloor(b, 0, 0, 100, 0.2, 0);
    const handle = try b.body(testBody(.{ .position = .{ 0, 0.99, 0 }, .shape = box, .object_layer = Layers.moving }), .activate);
    var inertia: [16]f32 = @splat(0);
    inertia[0] = 8;
    inertia[5] = 8;
    inertia[10] = 8;
    inertia[15] = 1;
    const settings = try b.addBodyDesc(.{
        .position = .{ 1.0, 0.99, 0 },
        .rotation = axisAngle(.{ 1, 0, 0 }, 0.1),
        .motion_type = 1,
        .shape = ball,
        .object_layer = Layers.moving2,
        .collision_group_id = 1,
        .collision_sub_group_id = 2,
        .linear_velocity = .{ -0.1, -0.2, -0.3 },
        .angular_velocity = .{ 0.1, 0.2, 0.3 },
        .user_data = 1234,
        .allow_dynamic_or_kinematic = 1,
        .is_sensor = 1,
        .collide_kinematic_vs_non_dynamic = 1,
        .use_manifold_reduction = 0,
        .apply_gyroscopic_force = 1,
        .motion_quality = 1,
        .enhanced_internal_edge_removal = 1,
        .allow_sleeping = 0,
        .friction = 0.3,
        .restitution = 0.4,
        .linear_damping = 0.01,
        .angular_damping = 0.02,
        .max_linear_velocity = 100.0,
        .max_angular_velocity = std.math.pi * 60.0,
        .gravity_factor = 0.5,
        .num_velocity_steps_override = 1,
        .num_position_steps_override = 3,
        .override_mass_properties = 2,
        .mass = 4.0,
        .inertia = inertia,
    });
    b.at(1);
    try b.op(.deactivate_body, handle, .{});
    try b.op(.remove_body, handle, .{});
    try b.op(.apply_body_creation_settings, handle, .{ .index = settings });
    try b.op(.add_body, handle, .{ .activation = .activate });
    b.at(3);
    try b.op(.deactivate_body, handle, .{});
    try b.op(.remove_body, handle, .{});
    try b.op(.destroy_body, handle, .{});
}

// -------------------------------------------------------------------------------------------------------------------
// Mirrors of the Wave C unit tests (ContactListenerTests, ShapeTests, ActiveEdgesTests): the bit exact counterpart of
// the cases that the other scenes do not reach yet

/// TestMassOverride of ContactListenerTests: two spheres on a collision course for every pair of motion types (not both
/// static) and each of the 16 user data values of the test; the listener overrides the inverse mass and inertia scales
/// of both bodies (ContactFlags.mass_override: 1, 0, 0.5 or 2 by user data, so some pairs have no body with mass and
/// create no contact constraint); zero gravity, restitution 1, no linear damping. The pairs of the test collide head on,
/// a second set hits off center (friction turns the spheres, so the inverse inertia scales matter too).
fn buildMassOverride(b: *SceneBuilder) Allocator.Error!void {
    const F = ContactFlags;
    b.contact_listener_mode = .modify;
    b.gravity = .{ 0, 0, 0 };
    b.num_updates = 60;
    const sphere = try b.sphere(1.0);
    var group: u32 = 0;
    for ([_]Real{ 0, 0.6 }) |y| {
        for (0..3) |m1| {
            for (0..3) |m2| {
                if (m1 == 0 and m2 == 0)
                    continue;
                for (0..16) |i| {
                    const x = real(group) * 30;
                    const z = real(i) * 3;
                    const low: u32 = @intCast(i << 1);
                    _ = try b.body(.{
                        .position = .{ x - 2, 0, z },
                        .linear_velocity = .{ if (m1 != 0) 3.0 else 0.0, 0, 0 },
                        .shape = sphere,
                        .motion_type = @intCast(m1),
                        .override_mass_properties = 1,
                        .mass = 1.0,
                        .restitution = 1.0,
                        .linear_damping = 0.0,
                        .user_data = F.userData(F.mass_override, low),
                        .object_layer = if (m1 != 0) Layers.moving else Layers.non_moving,
                    }, .activate);
                    _ = try b.body(.{
                        .position = .{ x + 2, y, z },
                        .linear_velocity = .{ if (m2 != 0) -4.0 else 0.0, 0, 0 },
                        .shape = sphere,
                        .motion_type = @intCast(m2),
                        .override_mass_properties = 1,
                        .mass = 2.0,
                        .restitution = 1.0,
                        .linear_damping = 0.0,
                        .user_data = F.userData(F.mass_override, low + 1),
                        .object_layer = if (m2 != 0) Layers.moving else Layers.non_moving,
                    }, .activate);
                }
                group += 1;
            }
        }
    }
}

/// TestInfiniteMassOverride of ContactListenerTests: a sphere (discrete or linear cast) hits a box that the listener
/// gives infinite mass (inverse mass and inertia scale 0, ContactFlags.mass_override), with the box created first or
/// second (the listener sees the bodies in both orders); zero gravity
fn buildInfiniteMassOverride(b: *SceneBuilder) Allocator.Error!void {
    const F = ContactFlags;
    b.contact_listener_mode = .modify;
    b.gravity = .{ 0, 0, 0 };
    b.num_updates = 30;
    const box = try b.box(.{ 2, 2, 2 }, 0.05);
    const sphere = try b.sphere(2);
    for ([_]bool{ false, true }, 0..) |do_swap, s| {
        for (0..2) |quality| {
            const z = real(s * 2 + quality) * 20;
            const box_desc = try b.addBodyDesc(.{ .position = .{ 0, 2, z }, .shape = box, .override_mass_properties = 1, .mass = 1.0, .user_data = F.userData(F.mass_override, 8), .object_layer = Layers.moving });
            const sphere_desc = try b.addBodyDesc(.{ .position = .{ 30, 2, z }, .linear_velocity = .{ -100, 0, 0 }, .shape = sphere, .override_mass_properties = 1, .mass = 10.0, .restitution = 0.1, .linear_damping = 0.0, .motion_quality = @intCast(quality), .object_layer = Layers.moving });
            if (do_swap) {
                // Swap the bodies so that the contact listener will receive the bodies in the opposite order
                _ = try b.createAndAddBody(sphere_desc, .activate);
                _ = try b.createAndAddBody(box_desc, .activate);
            } else {
                _ = try b.createAndAddBody(box_desc, .activate);
                _ = try b.createAndAddBody(sphere_desc, .activate);
            }
        }
    }
}

/// TestZeroDeltaTime of ContactListenerTests: updates with zero delta time call no contact callbacks and keep the
/// contact cache, the next update reports the contacts as persisted (or added for a body created in between)
fn buildZeroDeltaTime(b: *SceneBuilder) Allocator.Error!void {
    const floor = try b.box(.{ 100, 1, 100 }, 0);
    const sphere = try b.sphere(1.0);
    b.num_updates = 12;
    _ = try b.body(.{ .position = .{ 0, -1, 0 }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try b.body(.{ .shape = sphere, .linear_damping = 0, .angular_damping = 0, .collision_group_id = 0, .object_layer = Layers.moving }, .activate);
    try b.sysOp(.set_update, .{ .f = &.{0.0}, .count = 1 });
    b.at(1);
    try b.sysOp(.set_update, .{ .f = &.{1.0 / 60.0}, .count = 1 });
    b.at(2);
    _ = try b.body(.{ .position = .{ 4, 0, 0 }, .shape = sphere, .linear_damping = 0, .angular_damping = 0, .collision_group_id = 0, .object_layer = Layers.moving }, .activate);
    try b.sysOp(.set_update, .{ .f = &.{0.0}, .count = 1 });
    b.at(3);
    try b.sysOp(.set_update, .{ .f = &.{1.0 / 60.0}, .count = 1 });
    b.at(6);
    try b.sysOp(.set_update, .{ .f = &.{0.0}, .count = 2 });
    b.at(7);
    try b.sysOp(.set_update, .{ .f = &.{1.0 / 60.0}, .count = 2 });
}

/// The PhysicsTestContext parts of TestBoxShape, TestCylinderShape and TestTaperedCylinderShape of ShapeTests: zero
/// sized shapes (the convex radius is adjusted to zero) with the mass and inertia provided land on a floor, with the
/// speculative contact distance and the penetration slop set to 0
fn buildZeroSizedShapes(b: *SceneBuilder) Allocator.Error!void {
    b.settings.speculative_contact_distance = 0.0;
    b.settings.penetration_slop = 0.0;
    b.num_updates = 90;
    const floor = try b.box(.{ 100, 1, 100 }, 0);
    _ = try b.body(.{ .position = .{ 0, -1, 0 }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const shapes = [_]u32{
        try b.box(.{ 0, 0, 0 }, 1.0),
        try b.cylinder(0.0, 0.0, 1.0),
        try b.taperedCylinder(1.0e-12, 0.0, 1.0e-12, 1.0), // Top != bottom or else we'll be creating a CylinderShape instead
    };
    var identity: [16]f32 = @splat(0);
    for (0..4) |i| identity[i * 5] = 1.0;
    for (shapes, 0..) |shape, i| {
        _ = try b.body(.{ .position = .{ real(i), 1, 0 }, .shape = shape, .override_mass_properties = 2, .mass = 1.0, .inertia = identity, .object_layer = Layers.moving }, .activate);
    }
}

/// The flat floors of ActiveEdgesTests in lanes 10 m apart along z: the 7 x 7 quad mesh, the 8 x 8 sample height field
/// and both mirrored with ScaledShape (-1, 1, 1) (sCreateMeshShape / sCreateHeightFieldShape), static and frictionless
fn addActiveEdgesFloors(b: *SceneBuilder) Allocator.Error![4]Real {
    const mesh = try addMeshTerrain(b, 7, 1.0, true);
    const zeros: [8 * 8]f32 = @splat(0);
    const height_field = try b.heightField(&zeros, 8, .{ -3.5, 0, -3.5 }, .{ 1, 1, 1 });
    const floors = [_]u32{ mesh, height_field, try b.scaled(mesh, .{ -1, 1, 1 }), try b.scaled(height_field, .{ -1, 1, 1 }) };
    var lanes: [4]Real = undefined;
    for (floors, 0..) |floor, i| {
        lanes[i] = real(i) * 10;
        _ = try b.body(.{ .position = .{ 0, 0, lanes[i] }, .shape = floor, .motion_type = 0, .friction = 0.0, .collision_group_id = 0, .object_layer = Layers.non_moving }, .dont_activate);
    }
    return lanes;
}

/// sDiscreteCubeSlide of ActiveEdgesTests: a frictionless box slides over each floor of `addActiveEdgesFloors`, with
/// PhysicsSettings::mCheckActiveEdges `check_active_edges` (without it the box bumps into the internal edges)
fn buildDiscreteCubeSlide(comptime check_active_edges: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.settings.check_active_edges = check_active_edges;
            b.num_updates = 120;
            const box = try b.box(.{ 0.1, 0.1, 0.1 }, 0.05);
            const penetration_slop = b.settings.penetration_slop;
            for (try addActiveEdgesFloors(b)) |z|
                _ = try b.body(.{ .position = .{ -3, 0.1 - penetration_slop, z }, .linear_velocity = .{ 3, 0, 0 }, .shape = box, .friction = 0.0, .linear_damping = 0.0, .angular_damping = 0.0, .collision_group_id = 0, .object_layer = Layers.moving }, .activate);
        }
    }.build;
}

/// sLinearCastCubeSlide of ActiveEdgesTests: a frictionless linear cast box crosses each floor of
/// `addActiveEdgesFloors` in one step, ending 0.5 * penetration slop below it (the vertical velocity keeps the faces
/// from being back face culled), with PhysicsSettings::mCheckActiveEdges `check_active_edges` (without it the cast hits
/// an internal edge); zero gravity
fn buildLinearCastCubeSlide(comptime check_active_edges: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.settings.check_active_edges = check_active_edges;
            b.gravity = .{ 0, 0, 0 };
            b.num_updates = 4;
            const box = try b.box(.{ 0.1, 0.1, 0.1 }, 0.05);
            const penetration_slop = b.settings.penetration_slop;
            for (try addActiveEdgesFloors(b)) |z|
                _ = try b.body(.{ .position = .{ -3, 0.1 + penetration_slop, z }, .linear_velocity = .{ 6 * 60, -1.5 * penetration_slop * 60, 0 }, .shape = box, .friction = 0.0, .linear_damping = 0.0, .angular_damping = 0.0, .motion_quality = 1, .collision_group_id = 0, .object_layer = Layers.moving }, .activate);
        }
    }.build;
}

/// TestSimCollideBodyVsBody of SensorTests: a kinematic sensor that also detects static bodies (without manifold
/// reduction) overlaps 11 x 11 pyramids of a static compound floor of 21 x 21 pyramid meshes, with the body vs body
/// collision function `mode` (all triangles, one hit per leaf shape pair, one hit per body pair)
fn buildSimCollideBodyVsBody(comptime mode: SimCollideMode) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.sim_collide_mode = mode;
            b.num_updates = 6;
            b.max_contact_constraints = 4096;

            // Create pyramid with flat top
            const v = [_][3]f32{ .{ 1, 0, 1 }, .{ 1, 0, -1 }, .{ -1, 0, -1 }, .{ -1, 0, 1 }, .{ 0.1, 1, 0.1 }, .{ 0.1, 1, -0.1 }, .{ -0.1, 1, -0.1 }, .{ -0.1, 1, 0.1 } };
            const indices = [_][3]u32{ .{ 0, 1, 4 }, .{ 4, 1, 5 }, .{ 1, 2, 5 }, .{ 2, 6, 5 }, .{ 2, 3, 6 }, .{ 3, 7, 6 }, .{ 3, 0, 7 }, .{ 0, 4, 7 }, .{ 4, 5, 6 }, .{ 4, 6, 7 } };
            var triangles: [indices.len][9]f32 = undefined;
            for (indices, &triangles) |t, *out|
                out.* = v[t[0]] ++ v[t[1]] ++ v[t[2]];
            const pyramid = try b.mesh(&triangles);

            // Create floor of many pyramids
            var children: std.ArrayList(SimCompoundChild) = .empty;
            defer children.deinit(b.allocator);
            var x: i32 = -10;
            while (x <= 10) : (x += 1) {
                var z: i32 = -10;
                while (z <= 10) : (z += 1)
                    try children.append(b.allocator, .{ .shape = pyramid, .position = .{ @as(f32, @floatFromInt(x)) * 2.0, 0, @as(f32, @floatFromInt(z)) * 2.0 } });
            }
            const compound = try b.compound(.static_compound, children.items);
            _ = try b.body(.{ .shape = compound, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);

            // A kinematic sensor that also detects static bodies
            // Note that the size has been picked so that it is slightly smaller than 11 x 11 pyramids and incorporates all triangles in those pyramids
            const sensor = try b.box(.{ 10.99, 10.99, 10.99 }, 0.05);
            _ = try b.body(.{ .position = .{ 0, 5, 0 }, .shape = sensor, .motion_type = 1, .is_sensor = 1, .collide_kinematic_vs_non_dynamic = 1, .use_manifold_reduction = 0, .object_layer = Layers.moving }, .activate);
            b.at(3);
            try b.op(.set_linear_velocity, 1, .{ .f = &.{ 0.5, 0, 0.3 } });
        }
    }.build;
}

/// TestWereBodiesInContact of ContactListenerTests: a dynamic compound of two spheres on a floor is lifted on one side
/// and then on the other by impulses; WereBodiesInContact (in both orders) is compared after every step, also while
/// manifold reduction picks the other sphere's sub shape for the contact
fn buildWereBodiesInContact(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 12;
    const floor_shape = try b.box(.{ 100, 1, 100 }, 0);
    const sphere = try b.sphere(1);
    const compound = try b.compound(.static_compound, &.{ .{ .shape = sphere, .position = .{ -2, 0, 0 } }, .{ .shape = sphere, .position = .{ 2, 0, 0 } } });
    const signs = [_]Real{ -1, 1 };
    var floors: [2]u32 = undefined;
    var bodies: [2]u32 = undefined;
    for (signs, 0..) |_, lane| {
        const z = real(lane) * 210; // The floors are 200 m wide
        floors[lane] = try b.body(.{ .position = .{ 0, -1, z }, .shape = floor_shape, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
        bodies[lane] = try b.body(.{ .position = .{ 0, 0.999, z }, .shape = compound, .object_layer = Layers.moving }, .activate);
    }
    for (0..b.num_updates) |u| {
        b.at(@intCast(u));
        for (signs, floors, bodies, 0..) |sign, floor, body, lane| {
            const z = real(lane) * 210;
            try b.op(.were_bodies_in_contact, floor, .{ .index = body });
            try b.op(.were_bodies_in_contact, body, .{ .index = floor });
            if (u == 1) // Impulse on one side
                try b.op(.add_impulse_at_position, body, .{ .f = &.{ 0, 10000, 0 }, .r = .{ -sign * 2, 0, z } });
            if (u == 3) // Impulse on the other side
                try b.op(.add_impulse_at_position, body, .{ .f = &.{ 0, 10000, 0 }, .r = .{ sign * 2, 0, z } });
        }
    }
}

/// TestSensorVsStatic of SensorTests: a kinematic sensor between two static spheres (created before and after it)
/// detects them only while Body::SetCollideKinematicVsNonDynamic is on
fn buildSensorVsStatic(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 6;
    const sphere = try b.sphere(1.0);
    const box = try b.box(.{ 1, 1, 1 }, 0.05);
    _ = try b.body(.{ .shape = sphere, .motion_type = 0, .linear_damping = 0, .angular_damping = 0, .collision_group_id = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const sensor = try b.body(.{ .shape = box, .motion_type = 1, .is_sensor = 1, .object_layer = Layers.moving }, .activate); // Put in layer that collides with static
    _ = try b.body(.{ .shape = sphere, .motion_type = 0, .linear_damping = 0, .angular_damping = 0, .collision_group_id = 0, .object_layer = Layers.non_moving }, .dont_activate); // Created after the sensor to force higher body ID
    b.at(1);
    try b.op(.set_collide_kinematic_vs_non_dynamic, sensor, .{ .count = 1 }); // Start detecting static
    b.at(3);
    try b.op(.set_collide_kinematic_vs_non_dynamic, sensor, .{ .count = 0 }); // Stop detecting static
    b.at(4);
    try b.op(.set_collide_kinematic_vs_non_dynamic, sensor, .{ .count = 1 });
}

// -------------------------------------------------------------------------------------------------------------------
// Constraint scenes (Phase 6)
//
// Descriptions: `Desc.defaults(kind)` is a description with Jolt's defaults of the settings class (world space, the
// default axes, limits, springs and motors), the other `Desc` functions fill in the geometry of the common setups. A
// scene changes the remaining members in `i` / `r` / `f` (the layout is listed at ConstraintKind) before
// `b.constraint(desc)` creates the constraint.

/// Descriptions of constraints (`SimConstraint`)
pub const Desc = struct {
    const x_axis: [3]f32 = .{ 1, 0, 0 };
    const y_axis: [3]f32 = .{ 0, 1, 0 };

    /// Jolt's default SpringSettings: FrequencyAndDamping, frequency 0, damping 0
    pub fn setSpring(c: *SimConstraint, i_index: usize, f_index: usize, mode: u32, frequency: f32, damping: f32) void {
        c.i[i_index] = mode;
        c.f[f_index] = frequency;
        c.f[f_index + 1] = damping;
    }

    /// A MotorSettings: the spring (mode, frequency / stiffness, damping), min / max force limit, min / max torque limit
    pub fn setMotor(c: *SimConstraint, i_index: usize, f_index: usize, mode: u32, values: [6]f32) void {
        c.i[i_index] = mode;
        @memcpy(c.f[f_index..][0..6], &values);
    }

    /// Jolt's default MotorSettings (spring FrequencyAndDamping, frequency 2, damping 1, no force / torque limits)
    pub const default_motor: [6]f32 = .{ 2, 1, -flt_max, flt_max, -flt_max, flt_max };

    fn set3(dst: []f32, v: [3]f32) void {
        @memcpy(dst[0..3], &v);
    }

    fn setR3(dst: []Real, v: [3]Real) void {
        @memcpy(dst[0..3], &v);
    }

    /// A description with the defaults of Jolt's settings class of `kind` (the members a constructor sets in C++)
    pub fn defaults(kind: ConstraintKind) SimConstraint {
        var c: SimConstraint = .{ .kind = @intFromEnum(kind) };
        c.i[0] = ConstraintSpace.world_space; // mSpace = EConstraintSpace::WorldSpace (unused by the path)
        switch (kind) {
            .point, .pulley => {},
            .fixed => {
                set3(c.f[0..], x_axis);
                set3(c.f[3..], y_axis);
                set3(c.f[6..], x_axis);
                set3(c.f[9..], y_axis);
            },
            .distance => {
                c.f[0] = -1.0; // mMinDistance
                c.f[1] = -1.0; // mMaxDistance
            },
            .hinge => {
                set3(c.f[0..], y_axis);
                set3(c.f[3..], x_axis);
                set3(c.f[6..], y_axis);
                set3(c.f[9..], x_axis);
                c.f[12] = -std.math.pi; // -JPH_PI
                c.f[13] = std.math.pi;
                setMotor(&c, 2, 17, 0, default_motor);
            },
            .slider => {
                set3(c.f[0..], x_axis);
                set3(c.f[3..], y_axis);
                set3(c.f[6..], x_axis);
                set3(c.f[9..], y_axis);
                c.f[12] = -flt_max;
                c.f[13] = flt_max;
                setMotor(&c, 4, 17, 0, default_motor);
            },
            .cone => {
                set3(c.f[0..], x_axis);
                set3(c.f[3..], x_axis);
            },
            .swing_twist => {
                set3(c.f[0..], x_axis);
                set3(c.f[3..], y_axis);
                set3(c.f[6..], x_axis);
                set3(c.f[9..], y_axis);
                setMotor(&c, 2, 17, 0, default_motor);
                setMotor(&c, 3, 23, 0, default_motor);
            },
            .six_dof => {
                set3(c.f[0..], x_axis);
                set3(c.f[3..], y_axis);
                set3(c.f[6..], x_axis);
                set3(c.f[9..], y_axis);
                @memset(c.f[18..24], -flt_max);
                @memset(c.f[24..30], flt_max);
                for (0..6) |a| setMotor(&c, 5 + a, 36 + 6 * a, 0, default_motor);
            },
            .path => {
                c.f[6] = 1.0; // mPathRotation = identity (x, y, z, w)
                setMotor(&c, 4, 9, 0, default_motor);
            },
            .gear, .rack_and_pinion => {
                set3(c.f[0..], x_axis);
                set3(c.f[3..], x_axis);
                c.f[6] = 1.0; // mRatio
            },
        }
        if (kind == .pulley) {
            c.f[0] = 1.0; // mRatio
            c.f[1] = 0.0; // mMinLength
            c.f[2] = -1.0; // mMaxLength
        }
        return c;
    }

    /// FixedConstraintSettings with mAutoDetectPoint (world space, Jolt's default axes)
    pub fn fixedAuto(body1: u32, body2: u32) SimConstraint {
        var c = defaults(.fixed);
        c.body1 = body1;
        c.body2 = body2;
        c.i[1] = 1;
        return c;
    }

    /// FixedConstraintSettings with given points and axes in `space`
    pub fn fixed(body1: u32, body2: u32, space: u32, point1: [3]Real, axis_x1: [3]f32, axis_y1: [3]f32, point2: [3]Real, axis_x2: [3]f32, axis_y2: [3]f32) SimConstraint {
        var c = defaults(.fixed);
        c.body1 = body1;
        c.body2 = body2;
        c.i[0] = space;
        setR3(c.r[0..], point1);
        setR3(c.r[3..], point2);
        set3(c.f[0..], axis_x1);
        set3(c.f[3..], axis_y1);
        set3(c.f[6..], axis_x2);
        set3(c.f[9..], axis_y2);
        return c;
    }

    /// PointConstraintSettings in `space`
    pub fn point(body1: u32, body2: u32, space: u32, point1: [3]Real, point2: [3]Real) SimConstraint {
        var c = defaults(.point);
        c.body1 = body1;
        c.body2 = body2;
        c.i[0] = space;
        setR3(c.r[0..], point1);
        setR3(c.r[3..], point2);
        return c;
    }

    /// PointConstraintSettings with the same world space point on both bodies
    pub fn pointWorld(body1: u32, body2: u32, p: [3]Real) SimConstraint {
        return point(body1, body2, ConstraintSpace.world_space, p, p);
    }

    /// DistanceConstraintSettings in world space
    pub fn distance(body1: u32, body2: u32, point1: [3]Real, point2: [3]Real, min_distance: f32, max_distance: f32) SimConstraint {
        var c = defaults(.distance);
        c.body1 = body1;
        c.body2 = body2;
        setR3(c.r[0..], point1);
        setR3(c.r[3..], point2);
        c.f[0] = min_distance;
        c.f[1] = max_distance;
        return c;
    }

    /// HingeConstraintSettings in world space, the same point and axes on both bodies
    pub fn hinge(body1: u32, body2: u32, p: [3]Real, hinge_axis: [3]f32, normal_axis: [3]f32) SimConstraint {
        var c = defaults(.hinge);
        c.body1 = body1;
        c.body2 = body2;
        setR3(c.r[0..], p);
        setR3(c.r[3..], p);
        set3(c.f[0..], hinge_axis);
        set3(c.f[3..], normal_axis);
        set3(c.f[6..], hinge_axis);
        set3(c.f[9..], normal_axis);
        return c;
    }

    /// SliderConstraintSettings with mAutoDetectPoint and SetSliderAxis (world space)
    pub fn slider(body1: u32, body2: u32, slider_axis: [3]f32) SimConstraint {
        var c = defaults(.slider);
        c.body1 = body1;
        c.body2 = body2;
        c.i[1] = 1;
        c.i[2] = 1;
        set3(c.f[0..], slider_axis);
        return c;
    }

    /// SliderConstraintSettings in world space, the same point and axes on both bodies
    pub fn sliderAxes(body1: u32, body2: u32, p: [3]Real, slider_axis: [3]f32, normal_axis: [3]f32) SimConstraint {
        var c = defaults(.slider);
        c.body1 = body1;
        c.body2 = body2;
        setR3(c.r[0..], p);
        setR3(c.r[3..], p);
        set3(c.f[0..], slider_axis);
        set3(c.f[3..], normal_axis);
        set3(c.f[6..], slider_axis);
        set3(c.f[9..], normal_axis);
        return c;
    }

    /// ConeConstraintSettings in world space, the same point and twist axis on both bodies
    pub fn cone(body1: u32, body2: u32, p: [3]Real, twist_axis: [3]f32, half_cone_angle: f32) SimConstraint {
        var c = defaults(.cone);
        c.body1 = body1;
        c.body2 = body2;
        setR3(c.r[0..], p);
        setR3(c.r[3..], p);
        set3(c.f[0..], twist_axis);
        set3(c.f[3..], twist_axis);
        c.f[6] = half_cone_angle;
        return c;
    }

    /// SwingTwistConstraintSettings in world space, the same position and axes on both bodies, with the swing and twist
    /// limits (normal half cone angle, plane half cone angle, twist min / max angle)
    pub fn swingTwist(body1: u32, body2: u32, p: [3]Real, twist_axis: [3]f32, plane_axis: [3]f32, limits: [4]f32) SimConstraint {
        var c = defaults(.swing_twist);
        c.body1 = body1;
        c.body2 = body2;
        setR3(c.r[0..], p);
        setR3(c.r[3..], p);
        set3(c.f[0..], twist_axis);
        set3(c.f[3..], plane_axis);
        set3(c.f[6..], twist_axis);
        set3(c.f[9..], plane_axis);
        @memcpy(c.f[12..16], &limits);
        return c;
    }

    /// SixDOFConstraintSettings in world space, the same position and axes on both bodies, every axis free
    pub fn sixDOF(body1: u32, body2: u32, p: [3]Real, axis_x: [3]f32, axis_y: [3]f32) SimConstraint {
        var c = defaults(.six_dof);
        c.body1 = body1;
        c.body2 = body2;
        setR3(c.r[0..], p);
        setR3(c.r[3..], p);
        set3(c.f[0..], axis_x);
        set3(c.f[3..], axis_y);
        set3(c.f[6..], axis_x);
        set3(c.f[9..], axis_y);
        return c;
    }

    /// The limits of SixDOF axis `axis` (EAxis: 0..2 translation X / Y / Z, 3..5 rotation X / Y / Z; min >= max fixes it)
    pub fn setSixDOFLimits(c: *SimConstraint, axis: usize, min: f32, max: f32) void {
        c.f[18 + axis] = min;
        c.f[24 + axis] = max;
    }

    /// PathConstraintSettings with a Hermite path of `points` (position, tangent, normal; at most 16) in the space of
    /// body 1 at the path position / rotation, body 2 starts at `path_fraction`
    pub fn path(body1: u32, body2: u32, points: []const [9]f32, is_looping: bool, path_position: [3]f32, path_rotation: [4]f32, path_fraction: f32, rotation_constraint_type: u32) SimConstraint {
        std.debug.assert(points.len >= 1 and points.len <= 16);
        var c = defaults(.path);
        c.body1 = body1;
        c.body2 = body2;
        c.i[1] = @intFromBool(is_looping);
        c.i[2] = @intCast(points.len);
        c.i[3] = rotation_constraint_type;
        set3(c.f[0..], path_position);
        @memcpy(c.f[3..7], &path_rotation);
        c.f[7] = path_fraction;
        for (points, 0..) |p, k| @memcpy(c.f[16 + 9 * k ..][0..9], &p);
        return c;
    }

    /// GearConstraintSettings in world space between two bodies with hinges `hinge1` / `hinge2` (constraint handles)
    pub fn gear(body1: u32, body2: u32, hinge_axis1: [3]f32, hinge_axis2: [3]f32, ratio: f32, hinge1: u32, hinge2: u32) SimConstraint {
        var c = defaults(.gear);
        c.body1 = body1;
        c.body2 = body2;
        c.constraint1 = hinge1;
        c.constraint2 = hinge2;
        set3(c.f[0..], hinge_axis1);
        set3(c.f[3..], hinge_axis2);
        c.f[6] = ratio;
        return c;
    }

    /// RackAndPinionConstraintSettings in world space: body 1 the pinion on hinge `pinion`, body 2 the rack on slider
    /// `rack` (constraint handles)
    pub fn rackAndPinion(body1: u32, body2: u32, hinge_axis: [3]f32, slider_axis: [3]f32, ratio: f32, pinion: u32, rack: u32) SimConstraint {
        var c = defaults(.rack_and_pinion);
        c.body1 = body1;
        c.body2 = body2;
        c.constraint1 = pinion;
        c.constraint2 = rack;
        set3(c.f[0..], hinge_axis);
        set3(c.f[3..], slider_axis);
        c.f[6] = ratio;
        return c;
    }

    /// PulleyConstraintSettings in world space
    pub fn pulley(body1: u32, body2: u32, body_point1: [3]Real, fixed_point1: [3]Real, body_point2: [3]Real, fixed_point2: [3]Real, ratio: f32, min_length: f32, max_length: f32) SimConstraint {
        var c = defaults(.pulley);
        c.body1 = body1;
        c.body2 = body2;
        setR3(c.r[0..], body_point1);
        setR3(c.r[3..], fixed_point1);
        setR3(c.r[6..], body_point2);
        setR3(c.r[9..], fixed_point2);
        c.f[0] = ratio;
        c.f[1] = min_length;
        c.f[2] = max_length;
        return c;
    }
};

const flt_max = std.math.floatMax(f32);

/// A [3]f32 as [3]Real
fn r3(v: [3]f32) [3]Real {
    return .{ v[0], v[1], v[2] };
}

// ---- Fixed / Point (Phase 6 foundation, Blocker.constraints) ----

/// A chain of boxes hanging from the world with point constraints
fn buildConstraintChain(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.5, 0.2 }, 0.05);
    b.num_updates = 60;
    var previous: u32 = invalid_handle;
    for (0..5) |i| {
        const y: f32 = -@as(f32, @floatFromInt(i));
        const h = try b.body(.{ .position = .{ 0.3 * @as(Real, @floatFromInt(i)), y, 0 }, .shape = box, .object_layer = Layers.unused1 }, .activate);
        _ = try b.constraint(Desc.pointWorld(previous, h, .{ 0.3 * @as(Real, @floatFromInt(i)), y + 0.5, 0 }));
        previous = h;
    }
}

/// A point constraint between two body handles (`invalid_handle`: Body::sFixedToWorld) in world space, returns its
/// constraint handle
fn addPointConstraint(b: *SceneBuilder, body1: u32, body2: u32, point1: [3]f32, point2: [3]f32) Allocator.Error!u32 {
    return b.constraint(Desc.point(body1, body2, ConstraintSpace.world_space, r3(point1), r3(point2)));
}

/// A fixed constraint (mAutoDetectPoint) between two body handles, returns its constraint handle
fn addFixedConstraint(b: *SceneBuilder, body1: u32, body2: u32) Allocator.Error!u32 {
    return b.constraint(Desc.fixedAuto(body1, body2));
}

/// Fixed constraints (mAutoDetectPoint): a chain of boxes welded to each other that hangs from the world on a point
/// constraint; a box welded to a kinematic body that moves and turns; a box welded to the world (Body::sFixedToWorld as
/// body 1: the anchor is the center of mass of the box); a light and a heavy box welded together (the anchor is weighted
/// towards the light one) and a box welded to a static body. Forces, torques and impulses shake them.
fn buildConstraintsFixed(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    const offset_box = try b.offsetCenterOfMass(box, .{ 0.1, -0.05, 0.02 });
    b.num_updates = 80;

    // A chain welded with fixed constraints, hanging from the world on a point constraint
    var previous: u32 = invalid_handle;
    for (0..4) |i| {
        const x: f32 = 0.7 * @as(f32, @floatFromInt(i));
        const h = try b.body(.{ .position = .{ x, 5, 0 }, .rotation = axisAngle(.{ 0, 1, 0 }, 0.1 * @as(f64, @floatFromInt(i))), .shape = if (i % 2 == 0) box else offset_box, .object_layer = Layers.unused1 }, .activate);
        if (i == 0)
            _ = try addPointConstraint(b, invalid_handle, h, .{ x - 0.35, 5.3, 0 }, .{ x - 0.35, 5.3, 0 })
        else
            _ = try addFixedConstraint(b, previous, h);
        previous = h;
    }

    // A kinematic body that moves, with a dynamic box welded to it
    const kinematic = try b.body(.{ .position = .{ 6, 5, 0 }, .linear_velocity = .{ 0.5, 0, 0 }, .angular_velocity = .{ 0, 0.7, 0 }, .shape = box, .motion_type = 1, .object_layer = Layers.unused1 }, .activate);
    const welded = try b.body(.{ .position = .{ 6.7, 5.2, 0.1 }, .shape = box, .object_layer = Layers.unused1 }, .activate);
    _ = try addFixedConstraint(b, kinematic, welded);

    // A box welded to the world
    const to_world = try b.body(.{ .position = .{ -4, 3, 0 }, .rotation = axisAngle(.{ 1, 0, 0 }, 0.3), .shape = offset_box, .object_layer = Layers.unused1 }, .activate);
    _ = try addFixedConstraint(b, invalid_handle, to_world);

    // A light and a heavy box (mass override): the anchor is weighted towards the light one
    const light = try b.body(.{ .position = .{ 0, 8, 4 }, .shape = box, .mass = 0.5, .override_mass_properties = 1, .object_layer = Layers.unused1 }, .activate);
    const heavy = try b.body(.{ .position = .{ 0.8, 8.3, 4 }, .shape = box, .mass = 20, .override_mass_properties = 1, .object_layer = Layers.unused1 }, .activate);
    _ = try addFixedConstraint(b, light, heavy);

    // A box welded to a static body (the anchor is the center of mass of the box)
    const static_body = try b.body(.{ .position = .{ 4, 2, -4 }, .shape = box, .motion_type = 0, .object_layer = Layers.unused1 }, .dont_activate);
    const on_static = try b.body(.{ .position = .{ 4.5, 2.6, -4 }, .shape = box, .object_layer = Layers.unused1 }, .activate);
    _ = try addFixedConstraint(b, static_body, on_static);

    b.at(20);
    try b.op(.add_force, previous, .{ .f = &.{ 0, 0, 300 } });
    try b.op(.add_torque, welded, .{ .f = &.{ 50, 0, 0 } });
    b.at(35);
    try b.op(.add_impulse, to_world, .{ .f = &.{ 2, 5, 0 } });
    try b.op(.add_angular_impulse, heavy, .{ .f = &.{ 0, 0, 10 } });
    b.at(50);
    try b.op(.set_linear_velocity, kinematic, .{ .f = &.{ 0, 0.5, 0 } });
}

/// A box on a point constraint swings from the world into a stack of boxes on the floor: contacts and constraints in
/// the same island
fn buildConstraintsPendulumHitsStack(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.5, 0.5, 0.5 }, 0.05);
    const bob = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    b.num_updates = 90;
    _ = try addFloor(b, 20, 0);
    for (0..4) |y| _ = try b.body(.{ .position = .{ 0, 0.5 + real(y) * 1.01, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);

    // The pendulum hangs from (-4, 6, 0) and starts horizontal
    const pendulum = try b.body(.{ .position = .{ -8, 6, 0 }, .shape = bob, .object_layer = Layers.moving }, .activate);
    _ = try addPointConstraint(b, invalid_handle, pendulum, .{ -4, 6, 0 }, .{ -4, 6, 0 });

    // A second pendulum made of two boxes on point constraints
    const upper = try b.body(.{ .position = .{ 4, 5, 0.2 }, .shape = bob, .object_layer = Layers.moving }, .activate);
    _ = try addPointConstraint(b, invalid_handle, upper, .{ 4, 6, 0.2 }, .{ 4, 6, 0.2 });
    const lower = try b.body(.{ .position = .{ 5, 4, 0.2 }, .shape = bob, .object_layer = Layers.moving }, .activate);
    _ = try addPointConstraint(b, upper, lower, .{ 4.5, 4.5, 0.2 }, .{ 4.5, 4.5, 0.2 });
}

/// A net of boxes lying on the floor, each connected to its neighbours by point constraints and the corners pinned to
/// the world: an island with more than 128 constraints that the LargeIslandSplitter splits, and that falls asleep
fn buildConstraintsNet(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.4, 0.2, 0.4 }, 0.05);
    b.num_updates = 100;
    b.max_contact_constraints = 2048;
    b.settings.time_before_sleep = 0.25;
    _ = try addFloor(b, 20, 0);
    const nx = 10;
    const nz = 14;
    var handles: [nx * nz]u32 = undefined;
    for (0..nz) |z| {
        for (0..nx) |x| {
            const px: f32 = @as(f32, @floatFromInt(x)) - 5.0;
            const pz: f32 = @as(f32, @floatFromInt(z)) - 7.0;
            handles[z * nx + x] = try b.body(.{ .position = .{ px, 0.25, pz }, .shape = box, .object_layer = Layers.moving }, .activate);
            if (x > 0) _ = try addPointConstraint(b, handles[z * nx + x - 1], handles[z * nx + x], .{ px - 0.5, 0.25, pz }, .{ px - 0.5, 0.25, pz });
            if (z > 0) _ = try addPointConstraint(b, handles[(z - 1) * nx + x], handles[z * nx + x], .{ px, 0.25, pz - 0.5 }, .{ px, 0.25, pz - 0.5 });
        }
    }
    // Pin the corners to the world
    for ([_]usize{ 0, nx - 1, (nz - 1) * nx, nz * nx - 1 }) |i| {
        const p: [3]f32 = .{ @as(f32, @floatFromInt(i % nx)) - 5.0, 0.25, @as(f32, @floatFromInt(i / nx)) - 7.0 };
        _ = try addPointConstraint(b, invalid_handle, handles[i], p, p);
    }
    b.at(10);
    try b.op(.add_impulse, handles[nx * nz / 2], .{ .f = &.{ 0, 3, 0 } });
}

/// A chain hanging straight down from the world comes to rest and falls asleep; a force on its lowest box wakes the
/// chain link by link (BuildIslands of an active constraint activates its sleeping body, a constraint between two
/// sleeping bodies is not active) and a fixed constraint welds a sleeping box to it later
fn buildConstraintsSleepAndWake(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.4, 0.2 }, 0.05);
    b.num_updates = 120;
    b.settings.time_before_sleep = 0.2;
    var previous: u32 = invalid_handle;
    var chain: [4]u32 = undefined;
    for (0..4) |i| {
        const y: f32 = 5.0 - @as(f32, @floatFromInt(i));
        const h = try b.body(.{ .position = .{ 0, y, 0 }, .shape = box, .object_layer = Layers.unused1 }, .activate);
        _ = try addPointConstraint(b, previous, h, .{ 0, y + 0.5, 0 }, .{ 0, y + 0.5, 0 });
        chain[i] = h;
        previous = h;
    }
    const extra = try b.body(.{ .position = .{ 0.5, 1.6, 0 }, .shape = box, .gravity_factor = 0, .object_layer = Layers.unused1 }, .dont_activate);
    b.at(60);
    try b.expectSleeping(chain[0]);
    try b.expectSleeping(chain[3]);
    try b.op(.add_force, chain[3], .{ .f = &.{ 200, 0, 0 } });
    b.at(61);
    try b.expectAwake(chain[2]); // The force activated the lowest box, BuildIslands of its constraint the box above it
    try b.expectSleeping(chain[0]); // Only active constraints wake bodies: one more link per update
    b.at(63);
    try b.expectAwake(chain[0]);
    b.at(70);
    _ = try addFixedConstraint(b, chain[3], extra);
}

/// Constraints across a cross save / restore (the warm start impulses are state) and changes of the physics settings:
/// no constraint warm start, other solver iteration counts and Baumgarte factor
fn buildConstraintsSaveRestore(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    b.num_updates = 90;
    var no_warm_start = b.settings;
    no_warm_start.constraint_warm_start = 0;
    no_warm_start.num_velocity_steps = 4;
    no_warm_start.num_position_steps = 1;
    const settings1 = try b.addPhysicsSettings(no_warm_start);
    var more_steps = b.settings;
    more_steps.num_velocity_steps = 15;
    more_steps.num_position_steps = 5;
    more_steps.baumgarte = 0.5;
    const settings2 = try b.addPhysicsSettings(more_steps);

    var previous: u32 = invalid_handle;
    for (0..6) |i| {
        const x: f32 = 0.6 * @as(f32, @floatFromInt(i));
        const h = try b.body(.{ .position = .{ x, 6, 0 }, .angular_velocity = .{ 0, 0, 0.5 }, .shape = box, .object_layer = Layers.unused1 }, .activate);
        if (i % 3 == 2)
            _ = try addFixedConstraint(b, previous, h)
        else
            _ = try addPointConstraint(b, previous, h, .{ x - 0.3, 6, 0 }, .{ x - 0.3, 6, 0 });
        previous = h;
    }
    b.at(15);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(30);
    try b.sysOp(.set_physics_settings, .{ .index = settings1 });
    b.at(45);
    try b.sysOp(.restore_cross, .{ .index = 0 });
    b.at(50);
    try b.sysOp(.set_physics_settings, .{ .index = settings2 });
    b.at(60);
    try b.sysOp(.snapshot, .{ .index = 1 });
    try b.op(.add_impulse, previous, .{ .f = &.{ 0, 0, 3 } });
    b.at(70);
    try b.sysOp(.restore_cross, .{ .index = 1 });
}

/// The shapes of a ragdoll (`addRagdoll`)
const RagdollShapes = struct {
    pelvis: u32,
    spine: u32,
    chest: u32,
    head: u32,
    upper_arm: u32,
    lower_arm: u32,
    upper_leg: u32,
    lower_leg: u32,

    fn add(b: *SceneBuilder) Allocator.Error!RagdollShapes {
        return .{
            .pelvis = try b.box(.{ 0.15, 0.1, 0.1 }, 0.05),
            .spine = try b.box(.{ 0.12, 0.12, 0.08 }, 0.05),
            .chest = try b.box(.{ 0.18, 0.12, 0.1 }, 0.05),
            .head = try b.sphere(0.1),
            .upper_arm = try b.capsule(0.09, 0.06),
            .lower_arm = try b.capsule(0.1, 0.05),
            .upper_leg = try b.capsule(0.13, 0.07),
            .lower_leg = try b.capsule(0.14, 0.06),
        };
    }
};

/// A ragdoll-like tree of 12 bodies around the pelvis at `base` (the parts do not touch: every joint has a small gap):
/// the spine on a point constraint, the chest welded to the spine (FixedConstraint with auto detect point), the head,
/// arms and legs on point constraints (11 constraints). Returns the handle of the pelvis.
fn addRagdoll(b: *SceneBuilder, shapes: RagdollShapes, base: [3]Real, layer: u32, angular_velocity: [3]f32, lying: bool) Allocator.Error!u32 {
    // Lying on its back: a quarter turn around X, (x, y, z) -> (x, -z, y)
    const rotation: [4]f32 = if (lying) axisAngle(.{ 1, 0, 0 }, 0.5 * std.math.pi) else identity_rotation;
    const place = struct {
        fn place(origin: [3]Real, offset: [3]f32, turned: bool) [3]Real {
            const o: [3]f32 = if (turned) .{ offset[0], -offset[2], offset[1] } else offset;
            return .{ origin[0] + o[0], origin[1] + o[1], origin[2] + o[2] };
        }
    }.place;
    const Part = struct { shape: u32, offset: [3]f32 };
    const parts = [_]Part{
        .{ .shape = shapes.pelvis, .offset = .{ 0, 0, 0 } }, // 0
        .{ .shape = shapes.spine, .offset = .{ 0, 0.25, 0 } }, // 1
        .{ .shape = shapes.chest, .offset = .{ 0, 0.52, 0 } }, // 2
        .{ .shape = shapes.head, .offset = .{ 0, 0.8, 0 } }, // 3
        .{ .shape = shapes.upper_arm, .offset = .{ -0.3, 0.48, 0 } }, // 4
        .{ .shape = shapes.upper_arm, .offset = .{ 0.3, 0.48, 0 } }, // 5
        .{ .shape = shapes.lower_arm, .offset = .{ -0.3, 0.16, 0 } }, // 6
        .{ .shape = shapes.lower_arm, .offset = .{ 0.3, 0.16, 0 } }, // 7
        .{ .shape = shapes.upper_leg, .offset = .{ -0.09, -0.33, 0 } }, // 8
        .{ .shape = shapes.upper_leg, .offset = .{ 0.09, -0.33, 0 } }, // 9
        .{ .shape = shapes.lower_leg, .offset = .{ -0.09, -0.76, 0 } }, // 10
        .{ .shape = shapes.lower_leg, .offset = .{ 0.09, -0.76, 0 } }, // 11
    };
    var handles: [parts.len]u32 = undefined;
    for (parts, &handles) |part, *h|
        h.* = try b.body(.{ .position = place(base, part.offset, lying), .rotation = rotation, .shape = part.shape, .angular_velocity = angular_velocity, .linear_damping = 0.3, .angular_damping = 0.6, .object_layer = layer }, .activate);
    const Joint = struct { parent: usize, child: usize, point: [3]f32 };
    const joints = [_]Joint{
        .{ .parent = 0, .child = 1, .point = .{ 0, 0.115, 0 } },
        .{ .parent = 2, .child = 3, .point = .{ 0, 0.67, 0 } },
        .{ .parent = 2, .child = 4, .point = .{ -0.21, 0.6, 0 } },
        .{ .parent = 2, .child = 5, .point = .{ 0.21, 0.6, 0 } },
        .{ .parent = 4, .child = 6, .point = .{ -0.3, 0.32, 0 } },
        .{ .parent = 5, .child = 7, .point = .{ 0.3, 0.32, 0 } },
        .{ .parent = 0, .child = 8, .point = .{ -0.09, -0.115, 0 } },
        .{ .parent = 0, .child = 9, .point = .{ 0.09, -0.115, 0 } },
        .{ .parent = 8, .child = 10, .point = .{ -0.09, -0.545, 0 } },
        .{ .parent = 9, .child = 11, .point = .{ 0.09, -0.545, 0 } },
    };
    for (joints) |j|
        _ = try b.constraint(Desc.pointWorld(handles[j.parent], handles[j.child], place(base, j.point, lying)));
    _ = try b.constraint(Desc.fixedAuto(handles[1], handles[2])); // The chest is welded to the spine
    return handles[0];
}

/// Ragdoll-like trees: 8 ragdolls in two columns fall onto a floor and onto each other and tumble: contacts between the
/// parts of different ragdolls and constraints in one island that the LargeIslandSplitter splits
fn buildConstraintsRagdollPile(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try RagdollShapes.add(b);
    b.num_updates = 120;
    b.max_contact_constraints = 4096;
    _ = try addFloor(b, 20, 0);
    for (0..8) |i| {
        const fi: f32 = @floatFromInt(i);
        const column: Real = if (i % 2 == 0) -0.4 else 0.4;
        const base: [3]Real = .{ column, 1.2 + 2.0 * real(i / 2), real(0.2 * @sin(fi)) };
        _ = try addRagdoll(b, shapes, base, Layers.moving, .{ 0.5 * @sin(2 * fi), 0.3, 0.5 * @cos(3 * fi) }, false);
    }
}

/// 8 ragdolls lying on their backs in a row, holding hands (a point constraint between the lower arms of neighbours):
/// one island of 95 constraints and the floor contacts that the LargeIslandSplitter splits, which settles and falls
/// asleep from the final batch of a split island; a push wakes the island up again through its constraints
fn buildConstraintsRagdollsLying(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try RagdollShapes.add(b);
    b.num_updates = 110;
    b.max_contact_constraints = 4096;
    b.settings.time_before_sleep = 0.25;
    _ = try addFloor(b, 20, 0);
    const lower_arm_left = 6; // Handle offsets from the pelvis (addRagdoll's parts)
    const lower_arm_right = 7;
    var pelvis: [8]u32 = undefined;
    for (&pelvis, 0..) |*h, i| {
        const base: [3]Real = .{ 0.75 * real(i), 0.2, 0 };
        h.* = try addRagdoll(b, shapes, base, Layers.moving, @splat(0), true);
        if (i > 0) _ = try b.constraint(Desc.pointWorld(pelvis[i - 1] + lower_arm_right, h.* + lower_arm_left, .{ base[0] - 0.375, 0.2, 0.16 }));
    }
    b.at(95);
    try b.expectSleeping(pelvis[0]);
    try b.op(.add_impulse, pelvis[5], .{ .f = &.{ 0, 60, 0 } });
}

/// Body::sFixedToWorld as body 2 and in local space, static and kinematic bodies: a pendulum whose world anchor is body 2;
/// a box welded to the world as body 2 (auto detect: the anchor is its center of mass); a point constraint in local space
/// from the world (point 1 is relative to the world origin); two boxes welded in local space with frames that differ by a
/// quarter turn (the fixed constraint turns them into place); a box welded in world space to a static body with given
/// points and axes; a box hanging from a kinematic body that moves (MoveKinematic); a point constraint between two
/// kinematic bodies (not active: neither body is dynamic); a constraint between a static body and a dynamic body far away
fn buildConstraintsWorldAndStatic(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    const offset_box = try b.offsetCenterOfMass(box, .{ 0.05, 0.1, -0.05 });
    b.num_updates = 70;
    // A pendulum with the world as body 2
    const pendulum = try b.body(.{ .position = .{ 1, 5, 0 }, .shape = box }, .activate);
    _ = try b.constraint(Desc.point(pendulum, invalid_handle, ConstraintSpace.world_space, .{ 0, 6, 0 }, .{ 0, 6, 0 }));
    // Welded to the world as body 2
    const welded = try b.body(.{ .position = .{ 3, 5, 0 }, .rotation = axisAngle(.{ 0, 0, 1 }, 0.3), .shape = offset_box, .angular_velocity = .{ 1, 0, 0 } }, .activate);
    _ = try b.constraint(Desc.fixedAuto(welded, invalid_handle));
    // Local space from the world: point 1 is a world position, point 2 is relative to the center of mass of body 2
    const local = try b.body(.{ .position = .{ 6, 5, 0 }, .shape = offset_box }, .activate);
    _ = try b.constraint(Desc.point(invalid_handle, local, ConstraintSpace.local_to_body_com, .{ 6.3, 5.6, 0 }, .{ 0.3, 0.5, 0.05 }));
    // Two boxes welded in local space with frames a quarter turn apart
    const left = try b.body(.{ .position = .{ 9, 5, 0 }, .shape = box, .gravity_factor = 0 }, .activate);
    const right = try b.body(.{ .position = .{ 9.5, 5, 0 }, .shape = offset_box, .gravity_factor = 0 }, .activate);
    _ = try b.constraint(Desc.fixed(left, right, ConstraintSpace.local_to_body_com, .{ 0.25, 0, 0 }, .{ 0, 0, 1 }, .{ 0, 1, 0 }, .{ -0.3, -0.1, 0.05 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }));
    // Welded in world space to a static body with given points and axes
    const static_body = try b.body(.{ .position = .{ 12, 5, 0 }, .shape = box, .motion_type = 0 }, .dont_activate);
    const on_static = try b.body(.{ .position = .{ 12.5, 5.2, 0 }, .shape = box }, .activate);
    _ = try b.constraint(Desc.fixed(static_body, on_static, ConstraintSpace.world_space, .{ 12.25, 5.1, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 12.25, 5.1, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }));
    // Hanging from a kinematic body
    const kinematic = try b.body(.{ .position = .{ 15, 6, 0 }, .shape = box, .motion_type = 1 }, .activate);
    const hanging = try b.body(.{ .position = .{ 15, 5, 0 }, .shape = box }, .activate);
    _ = try b.constraint(Desc.pointWorld(kinematic, hanging, .{ 15, 5.6, 0 }));
    // Between two kinematic bodies: not active
    const k1 = try b.body(.{ .position = .{ 18, 5, 0 }, .shape = box, .motion_type = 1, .linear_velocity = .{ 0, 0.2, 0 } }, .activate);
    const k2 = try b.body(.{ .position = .{ 18.5, 5, 0 }, .shape = box, .motion_type = 1 }, .activate);
    const kinematic_pair = try b.constraint(Desc.pointWorld(k1, k2, .{ 18.25, 5, 0 }));
    try b.expectConstraintState(kinematic_pair, ConstraintState.inactive);
    // A static body and a far away dynamic body (a long rigid rod)
    const anchor = try b.body(.{ .position = .{ 21, 8, 0 }, .shape = box, .motion_type = 0 }, .dont_activate);
    const far = try b.body(.{ .position = .{ 24, 6, 1 }, .shape = box }, .activate);
    _ = try b.constraint(Desc.pointWorld(anchor, far, .{ 21, 8, 0 }));
    b.at(10);
    try b.op(.move_kinematic, kinematic, .{ .r = .{ 15.5, 6.2, 0 }, .f = &(axisAngle(.{ 0, 1, 0 }, 0.5) ++ [1]f32{0.5}) });
    b.at(30);
    try b.op(.move_kinematic, kinematic, .{ .r = .{ 15, 6.5, 0.5 }, .f = &(axisAngle(.{ 1, 0, 0 }, 0.3) ++ [1]f32{0.5}) });
    try b.op(.add_impulse, welded, .{ .f = &.{ 0, 3, 0 } });
    try b.op(.add_angular_impulse, right, .{ .f = &.{ 0, 0, 1 } });
}

/// An island of constraints without contacts that the LargeIslandSplitter splits: a curtain of 12 x 12 boxes on point
/// constraints that hangs from 12 pins and falls asleep, then a push wakes it up; and a hub with 130 spokes on point
/// constraints (every constraint of the island has the hub as a body, so at most one fits in each split: the others go
/// to the non-parallel split, and the splits with too few constraints are merged into it)
fn buildConstraintsCurtain(b: *SceneBuilder) Allocator.Error!void {
    const tile = try b.box(.{ 0.2, 0.2, 0.05 }, 0.05);
    const spoke = try b.box(.{ 0.05, 0.2, 0.05 }, 0.02);
    const hub_shape = try b.sphere(0.5);
    b.num_updates = 120;
    b.settings.time_before_sleep = 0.25;
    const n = 12;
    var tiles: [n * n]u32 = undefined;
    for (0..n) |row| {
        for (0..n) |col| {
            const x: Real = 0.5 * real(col) - 3;
            const y: Real = 8 - 0.5 * real(row);
            const h = try b.body(.{ .position = .{ x, y, 0 }, .shape = tile, .linear_damping = 0.3, .angular_damping = 0.3 }, .activate);
            tiles[row * n + col] = h;
            if (row == 0) _ = try b.constraint(Desc.pointWorld(invalid_handle, h, .{ x, y + 0.25, 0 }));
            if (col > 0) _ = try b.constraint(Desc.pointWorld(tiles[row * n + col - 1], h, .{ x - 0.25, y, 0 }));
            if (row > 0) _ = try b.constraint(Desc.pointWorld(tiles[(row - 1) * n + col], h, .{ x, y + 0.25, 0 }));
        }
    }
    // The hub hangs from the world, the spokes hang from the hub
    const hub = try b.body(.{ .position = .{ 10, 8, 0 }, .shape = hub_shape }, .activate);
    _ = try b.constraint(Desc.pointWorld(invalid_handle, hub, .{ 10, 8.5, 0 }));
    for (0..130) |i| {
        const a: f64 = 2.0 * std.math.pi * @as(f64, @floatFromInt(i)) / 130.0;
        const p: [3]Real = .{ 10 + real(0.45 * @cos(a)), 7.5, real(0.45 * @sin(a)) };
        const h = try b.body(.{ .position = .{ p[0], p[1] - 0.25, p[2] }, .shape = spoke }, .activate);
        _ = try b.constraint(Desc.pointWorld(hub, h, p));
    }
    b.at(100);
    try b.expectSleeping(tiles[0]);
    try b.op(.add_impulse, tiles[n * n - 1], .{ .f = &.{ 0, 0, 20 } }); // Wakes the curtain up through its constraints
}

/// Constraints that are disabled, enabled, removed and added between the updates: three chains hanging from the world, a
/// constraint disabled and enabled again (expect_constraint_state), one removed from the middle of the constraint list
/// (Jolt swaps the last one into its place) and added again, a whole chain removed and added as a batch, a constraint
/// created without adding it and added later, the user data, GetConstraintSettings after the bodies moved, a shape
/// change with NotifyShapeChanged, SetPoint1 / SetPoint2 and ResetWarmStart
fn buildConstraintsEnableRemove(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.15, 0.2, 0.15 }, 0.05);
    const offset_box = try b.offsetCenterOfMass(box, .{ 0, 0.08, 0.03 });
    b.num_updates = 90;
    var chains: [3][5]u32 = undefined;
    var links: [3][5]u32 = undefined;
    for (0..3) |c| {
        var previous: u32 = invalid_handle;
        for (0..5) |i| {
            const x: Real = 2 * real(c) + 0.1 * real(i);
            const y: Real = 6 - 0.5 * real(i);
            const h = try b.body(.{ .position = .{ x, y, 0 }, .shape = box }, .activate);
            links[c][i] = if (c == 1 and i == 2)
                try b.constraint(Desc.fixedAuto(previous, h)) // A fixed constraint in the middle of chain 1
            else
                try b.constraint(Desc.pointWorld(previous, h, .{ x, y + 0.25, 0 }));
            chains[c][i] = h;
            previous = h;
        }
    }
    // A constraint that is created now and added later
    const late_desc = try b.addConstraint(Desc.pointWorld(chains[0][4], chains[2][4], .{ 2.2, 4, 0.5 }));
    b.at(10);
    try b.constraintOp(.set_constraint_enabled, links[0][2], .{ .i = &.{0} }); // Chain 0 breaks
    try b.expectConstraintState(links[0][2], 0);
    try b.constraintOp(.set_constraint_user_data, links[0][3], .{ .i = &.{ 0x1234, 0x5678 } });
    b.at(20);
    try b.constraintOp(.set_constraint_enabled, links[0][2], .{ .i = &.{1} });
    try b.expectConstraintState(links[0][2], ConstraintState.enabled | ConstraintState.active);
    try b.constraintOp(.remove_constraint, links[1][1], .{}); // From the middle of the list
    b.at(25);
    try b.constraintOp(.add_constraint, links[1][1], .{});
    try b.constraintOp(.remove_constraints, links[2][0], .{ .i = &.{5} }); // Chain 2 falls
    b.at(32);
    try b.constraintOp(.add_constraints, links[2][0], .{ .i = &.{5} });
    const late = try b.createConstraintNoAdd(late_desc);
    try b.expectConstraintState(late, ConstraintState.enabled | ConstraintState.active); // Active, but not in the system
    b.at(40);
    try b.constraintOp(.add_constraint, late, .{});
    try b.constraintOp(.constraint_settings_hash, links[0][1], .{});
    try b.constraintOp(.constraint_settings_hash, links[1][2], .{});
    b.at(50);
    // The center of mass of chains[1][3] moves by (0, 0.08, 0.03): the constraints of the body are told
    try b.op(.set_shape, chains[1][3], .{ .index = offset_box, .count = 1 });
    try b.constraintOp(.notify_constraint_shape_changed, links[1][3], .{ .body = chains[1][3], .f = &.{ 0, 0.08, 0.03 } });
    try b.constraintOp(.notify_constraint_shape_changed, links[1][4], .{ .body = chains[1][3], .f = &.{ 0, 0.08, 0.03 } });
    try b.constraintOp(.notify_constraint_shape_changed, links[1][2], .{ .body = chains[0][0], .f = &.{ 1, 1, 1 } }); // Neither body: nothing
    b.at(60);
    try b.constraintOp(.point_set_point1, links[0][4], .{ .i = &.{ConstraintSpace.world_space}, .r = .{ 0.4, 4.3, 0.1 } });
    try b.constraintOp(.point_set_point2, links[2][4], .{ .i = &.{ConstraintSpace.local_to_body_com}, .r = .{ 0.05, 0.2, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, links[2][3], .{});
    try b.constraintOp(.constraint_settings_hash, links[0][4], .{});
    b.at(70);
    try b.constraintOp(.remove_constraints, links[0][0], .{ .i = &.{2} });
    try b.constraintOp(.set_constraint_enabled, links[1][0], .{ .i = &.{0} });
}

/// Constraint priorities and step overrides: with 2 velocity steps and 1 position step two equal chains differ only in
/// the priorities of their constraints (ConstraintManager::sSortConstraints solves higher priorities last, equal ones in
/// index order); a third chain has a constraint with step overrides (the island gets the largest override, also of
/// a body); the priorities and overrides change between the updates
fn buildConstraintsPriorities(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.15, 0.25, 0.15 }, 0.05);
    b.num_updates = 70;
    b.settings.num_velocity_steps = 2;
    b.settings.num_position_steps = 1;
    var links: [3][6]u32 = undefined;
    var bodies: [3][6]u32 = undefined;
    for (0..3) |c| {
        var previous: u32 = invalid_handle;
        for (0..6) |i| {
            const x: Real = 3 * real(c) + 0.4 * real(i);
            const y: Real = 6;
            const h = try b.body(.{ .position = .{ x, y, 0 }, .shape = box, .num_velocity_steps_override = if (c == 2 and i == 5) 6 else 0 }, .activate);
            var desc = if (i == 3) Desc.fixedAuto(previous, h) else Desc.pointWorld(previous, h, .{ x - 0.2, y, 0 });
            if (c == 1) desc.constraint_priority = @intCast(10 - i); // Reversed priorities
            if (c == 2 and i == 2) {
                desc.num_velocity_steps_override = 8;
                desc.num_position_steps_override = 4;
            }
            links[c][i] = try b.constraint(desc);
            bodies[c][i] = h;
            previous = h;
        }
    }
    b.at(20);
    for (0..6) |i| try b.constraintOp(.set_constraint_priority, links[0][i], .{ .i = &.{@intCast(i % 3)} });
    try b.constraintOp(.set_constraint_steps_override, links[2][2], .{ .i = &.{ 1, 0 } });
    try b.constraintOp(.set_constraint_steps_override, links[2][4], .{ .i = &.{ 12, 3 } });
    b.at(40);
    try b.constraintOp(.set_constraint_priority, links[1][0], .{ .i = &.{0} });
    try b.constraintOp(.set_constraint_steps_override, links[2][4], .{ .i = &.{ 0, 0 } });
    try b.op(.add_impulse, bodies[0][5], .{ .f = &.{ 0, 0, 5 } });
    try b.op(.add_impulse, bodies[1][5], .{ .f = &.{ 0, 0, 5 } });
}

/// Constraints and sleeping bodies (no contacts, without gravity): a constraint created between an awake and a
/// sleeping body wakes the sleeping one from inside the update (TwoBodyConstraint::BuildIslands); a constraint between
/// two sleeping bodies is not active and wakes nothing until ActivateConstraint; a disabled constraint between an awake
/// and a sleeping body wakes nothing until it is enabled; removing a constraint from a sleeping pair keeps it asleep; a
/// kinematic body that moves wakes the body hanging from it; a static body and a sleeping body: not active
fn buildConstraintsSleepingBodies(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.num_updates = 60;
    b.gravity = .{ 0, 0, 0 };
    b.settings.time_before_sleep = 0.2;
    const awake_a = try b.body(.{ .position = .{ 0, 5, 0 }, .shape = box, .linear_velocity = .{ 0.5, 0, 0 }, .allow_sleeping = 0 }, .activate);
    const asleep_a = try b.body(.{ .position = .{ 0, 4.4, 0 }, .shape = box }, .dont_activate);
    const asleep_b1 = try b.body(.{ .position = .{ 3, 5, 0 }, .shape = box }, .dont_activate);
    const asleep_b2 = try b.body(.{ .position = .{ 3, 4.4, 0 }, .shape = box }, .dont_activate);
    const awake_c = try b.body(.{ .position = .{ 6, 5, 0 }, .shape = box, .angular_velocity = .{ 0, 1, 0 }, .allow_sleeping = 0 }, .activate);
    const asleep_c = try b.body(.{ .position = .{ 6, 4.4, 0 }, .shape = box }, .dont_activate);
    const asleep_d1 = try b.body(.{ .position = .{ 9, 5, 0 }, .shape = box }, .dont_activate);
    const asleep_d2 = try b.body(.{ .position = .{ 9, 4.4, 0 }, .shape = box }, .dont_activate);
    const kinematic = try b.body(.{ .position = .{ 12, 5, 0 }, .shape = box, .motion_type = 1 }, .dont_activate);
    const asleep_e = try b.body(.{ .position = .{ 12, 4.4, 0 }, .shape = box }, .dont_activate);
    const static_body = try b.body(.{ .position = .{ 15, 5, 0 }, .shape = box, .motion_type = 0 }, .dont_activate);
    const asleep_f = try b.body(.{ .position = .{ 15, 4.4, 0 }, .shape = box }, .dont_activate);
    b.at(5);
    try b.expectSleeping(asleep_a);
    _ = try b.constraint(Desc.pointWorld(awake_a, asleep_a, .{ 0, 4.7, 0 })); // Wakes asleep_a in the next update
    const sleeping_pair = try b.constraint(Desc.fixedAuto(asleep_b1, asleep_b2));
    try b.expectConstraintState(sleeping_pair, ConstraintState.inactive);
    var disabled = Desc.pointWorld(awake_c, asleep_c, .{ 6, 4.7, 0 });
    disabled.enabled = 0;
    const disabled_handle = try b.constraint(disabled);
    const removed = try b.constraint(Desc.pointWorld(asleep_d1, asleep_d2, .{ 9, 4.7, 0 }));
    const kinematic_link = try b.constraint(Desc.pointWorld(kinematic, asleep_e, .{ 12, 4.7, 0 }));
    const static_link = try b.constraint(Desc.pointWorld(static_body, asleep_f, .{ 15, 4.7, 0 }));
    try b.expectConstraintState(static_link, ConstraintState.inactive);
    b.at(6);
    try b.expectAwake(asleep_a);
    try b.expectSleeping(asleep_b1);
    try b.expectSleeping(asleep_c);
    b.at(10);
    try b.constraintOp(.remove_constraint, removed, .{});
    try b.expectSleeping(asleep_d1);
    try b.constraintOp(.activate_constraint, sleeping_pair, .{});
    try b.expectAwake(asleep_b1);
    try b.expectAwake(asleep_b2);
    b.at(15);
    try b.expectSleeping(asleep_c);
    try b.constraintOp(.set_constraint_enabled, disabled_handle, .{ .i = &.{1} }); // Wakes asleep_c in the next update
    try b.op(.set_linear_velocity, kinematic, .{ .f = &.{ 0, 0.5, 0 } }); // Wakes the kinematic body, then asleep_e
    try b.expectConstraintState(kinematic_link, ConstraintState.enabled | ConstraintState.active);
    b.at(16);
    try b.expectAwake(asleep_c);
    try b.expectAwake(asleep_e);
    try b.expectSleeping(asleep_f);
    try b.expectSleeping(asleep_d2);
}

/// Partial states of the constraints across cross save / restore: a snapshot of everything, of the constraints only
/// (restoring it brings back a disabled constraint's enabled flag and the warm start impulses) and of the bodies and
/// constraints; ResetWarmStart before a restore
fn buildConstraintsPartialStates(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.num_updates = 70;
    const state_constraints: u32 = 8; // EStateRecorderState::Constraints
    const state_bodies: u32 = 2; // EStateRecorderState::Bodies
    var previous: u32 = invalid_handle;
    var links: [5]u32 = undefined;
    var bodies: [5]u32 = undefined;
    for (0..5) |i| {
        const x: Real = 0.5 * real(i);
        const h = try b.body(.{ .position = .{ x, 6, 0 }, .shape = box, .angular_velocity = .{ 0, 0.5, 0 } }, .activate);
        links[i] = if (i == 2) try b.constraint(Desc.fixedAuto(previous, h)) else try b.constraint(Desc.pointWorld(previous, h, .{ x - 0.25, 6, 0 }));
        bodies[i] = h;
        previous = h;
    }
    b.at(10);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(12);
    try b.constraintOp(.set_constraint_enabled, links[3], .{ .i = &.{0} });
    try b.sysOp(.snapshot, .{ .index = 1, .i = &.{state_constraints} });
    b.at(20);
    try b.constraintOp(.set_constraint_enabled, links[3], .{ .i = &.{1} });
    try b.sysOp(.restore_cross, .{ .index = 1 }); // The constraints of update 12: links[3] is disabled again
    try b.expectConstraintState(links[3], 0);
    b.at(30);
    try b.constraintOp(.set_constraint_enabled, links[3], .{ .i = &.{1} });
    try b.sysOp(.snapshot, .{ .index = 2, .i = &.{state_bodies | state_constraints} });
    b.at(40);
    try b.constraintOp(.reset_constraint_warm_start, links[1], .{});
    try b.op(.add_impulse, bodies[4], .{ .f = &.{ 0, 0, 2 } });
    try b.sysOp(.restore_cross, .{ .index = 2 });
    b.at(50);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Everything of update 10
    try b.expectConstraintState(links[3], ConstraintState.enabled | ConstraintState.active);
}

/// Fixed and point constraints with several collision steps per update and changing update parameters (the warm start
/// impulses are scaled by the step ratio, Jolt's mPreviousStepDeltaTime)
fn buildConstraintsCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const box = try b.box(.{ 0.2, 0.3, 0.2 }, 0.05);
            b.num_updates = 50;
            b.collision_steps = steps;
            b.delta_time = @as(f32, @floatFromInt(steps)) / 60.0;
            var previous: u32 = invalid_handle;
            for (0..6) |i| {
                const x: Real = 0.45 * real(i);
                const h = try b.body(.{ .position = .{ x, 6, 0 }, .shape = box }, .activate);
                if (i % 2 == 1)
                    _ = try b.constraint(Desc.fixedAuto(previous, h))
                else
                    _ = try b.constraint(Desc.pointWorld(previous, h, .{ x - 0.2, 6, 0 }));
                previous = h;
            }
            b.at(20);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 90.0}, .count = 1 });
            b.at(30);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 20.0}, .count = steps + 1 });
        }
    }.build;
}

/// Constraints far from the origin (world space points are Real: in double precision the anchors keep their
/// precision): a chain hanging from the world, a welded pair that falls onto a floor next to a box
fn buildConstraintsFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    b.num_updates = 60;
    const o: [3]Real = .{ 5000, 200, -12000 };
    const floor = try b.box(.{ 10, 1, 10 }, 0);
    _ = try b.body(.{ .position = .{ o[0], o[1] - 1, o[2] }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    var previous: u32 = invalid_handle;
    for (0..4) |i| {
        const p: [3]Real = .{ o[0] - 4 + 0.6 * real(i), o[1] + 5, o[2] };
        const h = try b.body(.{ .position = p, .shape = box, .object_layer = Layers.moving }, .activate);
        _ = try b.constraint(Desc.pointWorld(previous, h, .{ p[0] - 0.3, p[1], p[2] }));
        previous = h;
    }
    const w1 = try b.body(.{ .position = .{ o[0] + 3, o[1] + 2, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
    const w2 = try b.body(.{ .position = .{ o[0] + 3.5, o[1] + 2.4, o[2] + 0.1 }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.fixedAuto(w1, w2));
    _ = try b.body(.{ .position = .{ o[0] - 1.6, o[1] + 0.25, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
}

/// Random bodies (dynamic, kinematic, static; every basic shape) with random point and fixed constraints between them and
/// to the world, on a floor in a container, and random constraint operations between the updates (enable, priority,
/// step overrides, user data, remove / add, ActivateConstraint, ResetWarmStart, impulses)
fn buildConstraintsRandom(comptime seed: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.rng = 0x51ab_cd00 +% seed *% 0x9e3779b9;
            const shapes = try BasicShapes.add(b);
            b.num_updates = 80;
            b.max_contact_constraints = 2048;
            try addContainer(b, 6, 2);
            const num_bodies = 24;
            var bodies: [num_bodies]u32 = undefined;
            for (&bodies, 0..) |*h, i| {
                const motion_type: u32 = if (b.oneIn(8)) 0 else if (b.oneIn(7)) 1 else 2;
                const p = b.vec3(-4, 4);
                h.* = try b.body(.{
                    .position = .{ p[0], 1.5 + 0.8 * real(i), p[2] },
                    .rotation = b.rotation(),
                    .shape = shapes.pick(b.index(6)),
                    .motion_type = motion_type,
                    .linear_velocity = if (motion_type == 1) b.vec3(-0.5, 0.5) else @splat(0),
                    .angular_velocity = if (motion_type != 0) b.vec3(-1, 1) else @splat(0),
                    .object_layer = if (motion_type == 0) Layers.non_moving else Layers.moving,
                }, if (motion_type == 0) .dont_activate else .activate);
            }
            // Constraints between random pairs and to the world
            const num_constraints = 20;
            var handles: [num_constraints]u32 = undefined;
            var in_system: [num_constraints]bool = @splat(true);
            for (&handles) |*c| {
                const index1 = b.index(num_bodies + 1); // num_bodies: the world
                var index2 = b.index(num_bodies);
                if (index2 == index1) index2 = (index2 + 1) % num_bodies;
                const body1 = if (index1 == num_bodies) invalid_handle else bodies[index1];
                const body2 = bodies[index2];
                var desc = if (b.oneIn(3))
                    Desc.fixedAuto(body1, body2)
                else
                    Desc.point(body1, body2, ConstraintSpace.world_space, r3(b.vec3(-4, 4)), r3(b.vec3(-4, 4)));
                desc.constraint_priority = b.index(3);
                if (b.oneIn(5)) desc.num_velocity_steps_override = 1 + b.index(15);
                if (b.oneIn(5)) desc.num_position_steps_override = 1 + b.index(5);
                if (b.oneIn(8)) desc.enabled = 0;
                desc.user_data = b.next(); // Odd: not saved by the state filter of the end of the scene
                c.* = try b.constraint(desc);
            }
            var update: u32 = 5;
            while (update < b.num_updates) : (update += 1 + b.index(8)) {
                b.at(update);
                for (0..1 + b.index(3)) |_| {
                    const k = b.index(num_constraints);
                    const c = handles[k];
                    switch (b.index(8)) {
                        0 => try b.constraintOp(.set_constraint_enabled, c, .{ .i = &.{b.index(2)} }),
                        1 => try b.constraintOp(.set_constraint_priority, c, .{ .i = &.{b.index(4)} }),
                        2 => try b.constraintOp(.set_constraint_steps_override, c, .{ .i = &.{ b.index(12), b.index(4) } }),
                        3 => {
                            try b.constraintOp(if (in_system[k]) .remove_constraint else .add_constraint, c, .{});
                            in_system[k] = !in_system[k];
                        },
                        4 => try b.constraintOp(.activate_constraint, c, .{}),
                        5 => try b.constraintOp(.reset_constraint_warm_start, c, .{}),
                        6 => try b.constraintOp(.set_constraint_user_data, c, .{ .i = &.{ b.next(), b.next() } }),
                        else => try b.op(.add_impulse, bodies[b.index(num_bodies)], .{ .f = &b.vec3(-3, 3) }),
                    }
                }
            }
        }
    }.build;
}

// (append the scenes of the fixed / point group here)

// ---- Distance / Pulley (Blocker.distance_constraints / .pulley_constraints, OpKind 240..259) ----

/// Every operation of DistanceConstraint (the C++ side runs it until the class is ported): a box on a rope of fixed
/// length from the world (min = max = -1: the distance at creation), a second box on a soft rope with a range and a
/// limits spring given in local space, the setters, NotifyShapeChanged, ResetWarmStart and GetConstraintSettings
fn buildDistanceOps(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.num_updates = 60;
    const a = try b.body(.{ .position = .{ 1, 4, 0 }, .shape = box }, .activate);
    const rope = try b.constraint(Desc.distance(invalid_handle, a, .{ 0, 6, 0 }, .{ 1, 4.2, 0 }, -1, -1));
    const c = try b.body(.{ .position = .{ 1.5, 2.5, 0.5 }, .shape = box }, .activate);
    var soft = Desc.distance(a, c, .{ 0, -0.2, 0 }, .{ 0, 0.2, 0 }, 0.8, 1.6);
    soft.i[0] = ConstraintSpace.local_to_body_com;
    Desc.setSpring(&soft, 1, 2, 0, 3, 0.3);
    const soft_rope = try b.constraint(soft);
    b.at(10);
    try b.constraintOp(.distance_set_distance, rope, .{ .f = &.{ 1.5, 2.5 } });
    try b.expectConstraintState(soft_rope, ConstraintState.enabled | ConstraintState.active);
    b.at(20);
    try b.constraintOp(.distance_set_limits_spring_settings, soft_rope, .{ .i = &.{1}, .f = &.{ 40, 2 } }); // StiffnessAndDamping
    try b.constraintOp(.constraint_settings_hash, soft_rope, .{});
    b.at(30);
    try b.constraintOp(.notify_constraint_shape_changed, rope, .{ .body = a, .f = &.{ 0.05, 0, 0 } });
    try b.constraintOp(.notify_constraint_shape_changed, soft_rope, .{ .body = c, .f = &.{ 0, -0.05, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, rope, .{});
    b.at(40);
    try b.constraintOp(.distance_set_distance, soft_rope, .{ .f = &.{ 1, 1 } });
    try b.constraintOp(.distance_set_limits_spring_settings, soft_rope, .{ .i = &.{2}, .f = &.{ 10, 1 } }); // MassNormalizedStiffnessAndDamping
    try b.constraintOp(.constraint_settings_hash, rope, .{});
}

/// Every operation of PulleyConstraint: two boxes on a rope over two fixed points with a ratio, the second pulley in local
/// space with a length range; SetLength, NotifyShapeChanged, ResetWarmStart, GetConstraintSettings
fn buildPulleyOps(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    b.num_updates = 60;
    // Nearly in balance with a ratio of 2 (m2 = 2 m1): body 2 sinks slowly
    const a = try b.body(.{ .position = .{ -1, 3, 0 }, .shape = box, .mass = 50, .override_mass_properties = 1 }, .activate);
    const c = try b.body(.{ .position = .{ 1, 3.5, 0 }, .shape = box, .mass = 110, .override_mass_properties = 1 }, .activate);
    const pulley = try b.constraint(Desc.pulley(a, c, .{ -1, 3.3, 0 }, .{ -1, 6, 0 }, .{ 1, 3.8, 0 }, .{ 1, 6, 0 }, 2, 0, -1));
    const d = try b.body(.{ .position = .{ -1, 3, 4 }, .shape = box }, .activate);
    const e = try b.body(.{ .position = .{ 1, 3, 4 }, .shape = box }, .activate);
    var local = Desc.pulley(d, e, .{ 0, 0.3, 0 }, .{ -1, 6, 4 }, .{ 0, 0.3, 0 }, .{ 1, 6, 4 }, 1, 4, 6);
    local.i[0] = ConstraintSpace.local_to_body_com; // The fixed points stay in world space
    const local_pulley = try b.constraint(local);
    b.at(15);
    try b.constraintOp(.pulley_set_length, pulley, .{ .f = &.{ 4, 7 } });
    try b.constraintOp(.constraint_settings_hash, local_pulley, .{});
    b.at(30);
    try b.constraintOp(.pulley_set_length, local_pulley, .{ .f = &.{ 5, 5 } });
    try b.constraintOp(.notify_constraint_shape_changed, pulley, .{ .body = c, .f = &.{ 0, 0.1, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, local_pulley, .{});
    try b.constraintOp(.constraint_settings_hash, pulley, .{});
}

// (append the scenes of the distance / pulley group here)

// ---- Hinge (Blocker.hinge_constraints, OpKind 260..279) ----

/// Every operation of HingeConstraint: a door hinged to the world with limits and friction driven by its motor (velocity,
/// then position with a target angle and a target orientation), a hinged pair of dynamic boxes with a limits spring
fn buildHingeOps(b: *SceneBuilder) Allocator.Error!void {
    const door = try b.box(.{ 0.5, 1, 0.05 }, 0.02);
    const box = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    b.num_updates = 70;
    const d = try b.body(.{ .position = .{ 0.5, 3, 0 }, .shape = door }, .activate);
    var hinge = Desc.hinge(invalid_handle, d, .{ 0, 3, 0 }, .{ 0, 1, 0 }, .{ 1, 0, 0 });
    hinge.f[12] = -1.2; // Limits
    hinge.f[13] = 1.2;
    hinge.f[16] = 0.1; // Max friction torque
    const door_hinge = try b.constraint(hinge);
    const a = try b.body(.{ .position = .{ 4, 5, 0 }, .shape = box }, .activate);
    const c = try b.body(.{ .position = .{ 4.7, 5, 0 }, .shape = box, .angular_velocity = .{ 0, 0, 2 } }, .activate);
    _ = try b.constraint(Desc.pointWorld(invalid_handle, a, .{ 4, 5.4, 0 }));
    var pair = Desc.hinge(a, c, .{ 4.35, 5, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 });
    pair.f[12] = -0.5;
    pair.f[13] = 0.5;
    Desc.setSpring(&pair, 1, 14, 0, 4, 0.5);
    const pair_hinge = try b.constraint(pair);
    b.at(5);
    try b.constraintOp(.hinge_set_max_friction_torque, door_hinge, .{ .f = &.{0.5} });
    try b.constraintOp(.hinge_set_motor_settings, door_hinge, .{ .i = &.{0}, .f = &.{ 4, 1, -1000, 1000, -50, 50 } });
    try b.constraintOp(.hinge_set_motor_state, door_hinge, .{ .i = &.{1} }); // Velocity
    try b.constraintOp(.hinge_set_target_angular_velocity, door_hinge, .{ .f = &.{1.5} });
    b.at(20);
    try b.constraintOp(.hinge_set_motor_state, door_hinge, .{ .i = &.{2} }); // Position
    try b.constraintOp(.hinge_set_target_angle, door_hinge, .{ .f = &.{-0.7} });
    b.at(35);
    try b.constraintOp(.hinge_set_target_orientation_bs, door_hinge, .{ .f = &axisAngle(.{ 0, 1, 0 }, 0.4) });
    try b.constraintOp(.hinge_set_limits, pair_hinge, .{ .f = &.{ -0.2, 0.8 } });
    try b.constraintOp(.hinge_set_limits_spring_settings, pair_hinge, .{ .i = &.{1}, .f = &.{ 30, 1 } });
    b.at(45);
    try b.constraintOp(.hinge_set_motor_state, door_hinge, .{ .i = &.{3} }); // PositionAndVelocity
    try b.constraintOp(.hinge_set_target_angular_velocity, door_hinge, .{ .f = &.{-0.5} });
    try b.constraintOp(.constraint_settings_hash, door_hinge, .{});
    b.at(55);
    try b.constraintOp(.hinge_set_motor_state, door_hinge, .{ .i = &.{0} });
    try b.constraintOp(.notify_constraint_shape_changed, pair_hinge, .{ .body = a, .f = &.{ 0, 0.02, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, door_hinge, .{});
    try b.constraintOp(.constraint_settings_hash, pair_hinge, .{});
}

// (append the scenes of the hinge group here)

// ---- Slider (Blocker.slider_constraints, OpKind 280..299) ----

/// Every operation of SliderConstraint: a box on a horizontal slider from the world (auto detect point and
/// SetSliderAxis) driven by its motor (velocity, then position), with limits and friction; a slider between two boxes
/// with explicit axes and a limits spring
fn buildSliderOps(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    b.num_updates = 70;
    const a = try b.body(.{ .position = .{ 0, 3, 0 }, .shape = box }, .activate);
    var s1 = Desc.slider(invalid_handle, a, .{ 1, 0, 0 });
    s1.f[12] = -1.5; // Limits
    s1.f[13] = 1.5;
    const rail = try b.constraint(s1);
    const c = try b.body(.{ .position = .{ 4, 5, 0 }, .shape = box }, .activate);
    const e = try b.body(.{ .position = .{ 4, 4, 0.2 }, .shape = box, .linear_velocity = .{ 0, 0, 1 } }, .activate);
    _ = try b.constraint(Desc.fixedAuto(invalid_handle, c));
    var s2 = Desc.sliderAxes(c, e, .{ 4, 4.5, 0.1 }, .{ 0, 0, 1 }, .{ 0, 1, 0 });
    s2.f[12] = -0.4;
    s2.f[13] = 0.4;
    Desc.setSpring(&s2, 3, 14, 0, 5, 0.5);
    const piston = try b.constraint(s2);
    b.at(5);
    try b.constraintOp(.slider_set_max_friction_force, rail, .{ .f = &.{2} });
    try b.constraintOp(.slider_set_motor_settings, rail, .{ .i = &.{0}, .f = &.{ 3, 1, -200, 200, -1000, 1000 } });
    try b.constraintOp(.slider_set_motor_state, rail, .{ .i = &.{1} });
    try b.constraintOp(.slider_set_target_velocity, rail, .{ .f = &.{0.8} });
    b.at(25);
    try b.constraintOp(.slider_set_motor_state, rail, .{ .i = &.{2} });
    try b.constraintOp(.slider_set_target_position, rail, .{ .f = &.{-0.6} });
    try b.constraintOp(.slider_set_limits, piston, .{ .f = &.{ -0.1, 0.6 } });
    try b.constraintOp(.slider_set_limits_spring_settings, piston, .{ .i = &.{1}, .f = &.{ 50, 2 } });
    b.at(45);
    try b.constraintOp(.slider_set_limits, rail, .{ .f = &.{ -1, 0.5 } });
    try b.constraintOp(.slider_set_target_position, rail, .{ .f = &.{3} }); // Clamped to the limits
    try b.constraintOp(.constraint_settings_hash, rail, .{});
    b.at(55);
    try b.constraintOp(.slider_set_motor_state, rail, .{ .i = &.{0} });
    try b.constraintOp(.notify_constraint_shape_changed, piston, .{ .body = e, .f = &.{ 0.01, 0, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, piston, .{});
    try b.constraintOp(.constraint_settings_hash, piston, .{});
}

// (append the scenes of the slider group here)

// ---- Cone / SwingTwist (Blocker.cone_constraints / .swing_twist_constraints, OpKind 300..319) ----

/// Every operation of ConeConstraint: boxes hanging from the world and from each other in a cone, SetHalfConeAngle,
/// NotifyShapeChanged, ResetWarmStart, GetConstraintSettings
fn buildConeOps(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.1, 0.4, 0.1 }, 0.05);
    b.num_updates = 60;
    const a = try b.body(.{ .position = .{ 0.3, 4.6, 0 }, .shape = box, .angular_velocity = .{ 0, 0, 1 } }, .activate);
    const cone = try b.constraint(Desc.cone(invalid_handle, a, .{ 0, 5, 0 }, .{ 0, -1, 0 }, 0.6));
    const c = try b.body(.{ .position = .{ 0.6, 3.8, 0.2 }, .shape = box }, .activate);
    const cone2 = try b.constraint(Desc.cone(a, c, .{ 0.45, 4.2, 0.1 }, .{ 0.3, -1, 0.1 }, 0.3));
    b.at(20);
    try b.constraintOp(.cone_set_half_cone_angle, cone, .{ .f = &.{0.2} });
    try b.constraintOp(.constraint_settings_hash, cone, .{});
    b.at(40);
    try b.constraintOp(.cone_set_half_cone_angle, cone2, .{ .f = &.{1.0} });
    try b.constraintOp(.notify_constraint_shape_changed, cone2, .{ .body = c, .f = &.{ 0, 0.05, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, cone, .{});
    try b.constraintOp(.constraint_settings_hash, cone2, .{});
}

/// Every operation of SwingTwistConstraint: an arm on a shoulder joint (cone swing) and a forearm on an elbow (pyramid
/// swing) with friction, swing and twist motors (velocity, then position with target orientations in constraint and
/// body space), the limit setters
fn buildSwingTwistOps(b: *SceneBuilder) Allocator.Error!void {
    const limb = try b.box(.{ 0.1, 0.4, 0.1 }, 0.05);
    b.num_updates = 80;
    const arm = try b.body(.{ .position = .{ 0, 4.5, 0 }, .shape = limb }, .activate);
    const shoulder = try b.constraint(Desc.swingTwist(invalid_handle, arm, .{ 0, 4.9, 0 }, .{ 0, -1, 0 }, .{ 1, 0, 0 }, .{ 0.6, 0.4, -0.3, 0.3 }));
    const forearm = try b.body(.{ .position = .{ 0, 3.7, 0 }, .shape = limb, .angular_velocity = .{ 1, 0, 0.5 } }, .activate);
    var elbow_desc = Desc.swingTwist(arm, forearm, .{ 0, 4.1, 0 }, .{ 0, -1, 0 }, .{ 0, 0, 1 }, .{ 0.8, 0.2, -0.1, 0.5 });
    elbow_desc.i[1] = 1; // ESwingType::Pyramid
    elbow_desc.f[16] = 0.05; // Max friction torque
    const elbow = try b.constraint(elbow_desc);
    b.at(5);
    try b.constraintOp(.swing_twist_set_max_friction_torque, shoulder, .{ .f = &.{0.1} });
    try b.constraintOp(.swing_twist_set_swing_motor_settings, shoulder, .{ .i = &.{0}, .f = &.{ 5, 1, -1000, 1000, -20, 20 } });
    try b.constraintOp(.swing_twist_set_twist_motor_settings, shoulder, .{ .i = &.{0}, .f = &.{ 5, 1, -1000, 1000, -20, 20 } });
    try b.constraintOp(.swing_twist_set_swing_motor_state, shoulder, .{ .i = &.{1} });
    try b.constraintOp(.swing_twist_set_twist_motor_state, shoulder, .{ .i = &.{1} });
    try b.constraintOp(.swing_twist_set_target_angular_velocity_cs, shoulder, .{ .f = &.{ 0.3, 0.2, -0.1 } });
    b.at(20);
    try b.constraintOp(.swing_twist_set_target_angular_velocity_bs, shoulder, .{ .f = &.{ -0.2, 0.1, 0.3 } });
    try b.constraintOp(.swing_twist_set_normal_half_cone_angle, elbow, .{ .f = &.{0.5} });
    try b.constraintOp(.swing_twist_set_plane_half_cone_angle, elbow, .{ .f = &.{0.3} });
    try b.constraintOp(.swing_twist_set_twist_min_angle, elbow, .{ .f = &.{-0.4} });
    try b.constraintOp(.swing_twist_set_twist_max_angle, elbow, .{ .f = &.{0.2} });
    b.at(35);
    try b.constraintOp(.swing_twist_set_swing_motor_state, shoulder, .{ .i = &.{2} });
    try b.constraintOp(.swing_twist_set_twist_motor_state, shoulder, .{ .i = &.{2} });
    try b.constraintOp(.swing_twist_set_target_orientation_cs, shoulder, .{ .f = &axisAngle(.{ 0, 0, 1 }, 0.3) });
    b.at(50);
    try b.constraintOp(.swing_twist_set_target_orientation_bs, shoulder, .{ .f = &axisAngle(.{ 1, 0, 0 }, -0.2) });
    try b.constraintOp(.swing_twist_set_max_friction_torque, elbow, .{ .f = &.{0.3} });
    try b.constraintOp(.constraint_settings_hash, shoulder, .{});
    b.at(65);
    try b.constraintOp(.swing_twist_set_swing_motor_state, shoulder, .{ .i = &.{0} });
    try b.constraintOp(.swing_twist_set_twist_motor_state, shoulder, .{ .i = &.{3} }); // PositionAndVelocity
    try b.constraintOp(.notify_constraint_shape_changed, elbow, .{ .body = forearm, .f = &.{ 0, -0.02, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, elbow, .{});
    try b.constraintOp(.constraint_settings_hash, elbow, .{});
}

// (append the scenes of the cone / swing twist group here)

// ---- SixDOF (Blocker.six_dof_constraints, OpKind 320..339) ----

/// Every operation of SixDOFConstraint: a box hanging from the world with limited, fixed and free axes, a soft
/// translation limit, friction, and motors on a translation and a rotation axis (velocity, then position with target
/// positions and orientations); a second box with every axis fixed (the fully constrained path)
fn buildSixDOFOps(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    b.num_updates = 80;
    const a = try b.body(.{ .position = .{ 0, 4, 0 }, .shape = box, .angular_velocity = .{ 0.3, 0.5, 0.2 } }, .activate);
    var six = Desc.sixDOF(invalid_handle, a, .{ 0, 4, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 });
    Desc.setSixDOFLimits(&six, 0, -0.5, 0.5); // Translation X limited
    Desc.setSixDOFLimits(&six, 1, 0, 0); // Translation Y fixed
    // Translation Z free
    Desc.setSixDOFLimits(&six, 3, 0, 0); // Rotation X fixed
    Desc.setSixDOFLimits(&six, 4, -0.4, 0.4); // Rotation Y limited
    Desc.setSixDOFLimits(&six, 5, -0.6, 0.6); // Rotation Z limited
    Desc.setSpring(&six, 2, 30, 0, 2, 0.5); // A soft limit on translation X
    six.f[12 + 5] = 0.2; // Friction on rotation Z
    const joint = try b.constraint(six);
    const c = try b.body(.{ .position = .{ 3, 4, 0 }, .shape = box }, .activate);
    var fixed = Desc.sixDOF(invalid_handle, c, .{ 3, 4.3, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 });
    for (0..6) |axis| Desc.setSixDOFLimits(&fixed, axis, 0, 0);
    fixed.i[1] = 1; // ESwingType::Pyramid
    const welded = try b.constraint(fixed);
    b.at(5);
    try b.constraintOp(.six_dof_set_motor_settings, joint, .{ .i = &.{ 2, 0 }, .f = &.{ 4, 1, -100, 100, -1000, 1000 } });
    try b.constraintOp(.six_dof_set_motor_settings, joint, .{ .i = &.{ 5, 0 }, .f = &.{ 4, 1, -1000, 1000, -10, 10 } });
    try b.constraintOp(.six_dof_set_motor_state, joint, .{ .i = &.{ 2, 1 } }); // Translation Z: velocity
    try b.constraintOp(.six_dof_set_motor_state, joint, .{ .i = &.{ 5, 1 } }); // Rotation Z: velocity
    try b.constraintOp(.six_dof_set_target_velocity_cs, joint, .{ .f = &.{ 0, 0, 0.5 } });
    try b.constraintOp(.six_dof_set_target_angular_velocity_cs, joint, .{ .f = &.{ 0, 0, 0.4 } });
    b.at(25);
    try b.constraintOp(.six_dof_set_motor_state, joint, .{ .i = &.{ 2, 2 } }); // Position
    try b.constraintOp(.six_dof_set_motor_state, joint, .{ .i = &.{ 5, 2 } });
    try b.constraintOp(.six_dof_set_target_position_cs, joint, .{ .f = &.{ 0, 0, -0.3 } });
    try b.constraintOp(.six_dof_set_target_orientation_cs, joint, .{ .f = &axisAngle(.{ 0, 0, 1 }, 0.3) });
    try b.constraintOp(.six_dof_set_max_friction, joint, .{ .i = &.{4}, .f = &.{0.5} });
    b.at(40);
    try b.constraintOp(.six_dof_set_target_orientation_bs, joint, .{ .f = &axisAngle(.{ 0, 0, 1 }, -0.2) });
    try b.constraintOp(.six_dof_set_translation_limits, joint, .{ .f = &.{ -0.2, 0, -1, 0.3, 0, 1 } });
    try b.constraintOp(.six_dof_set_rotation_limits, joint, .{ .f = &.{ 0, -0.2, -0.8, 0, 0.2, 0.8 } });
    try b.constraintOp(.six_dof_set_limits_spring_settings, joint, .{ .i = &.{ 0, 1 }, .f = &.{ 60, 3 } });
    try b.constraintOp(.constraint_settings_hash, joint, .{});
    b.at(60);
    try b.constraintOp(.six_dof_set_motor_state, joint, .{ .i = &.{ 2, 0 } });
    try b.constraintOp(.six_dof_set_motor_state, joint, .{ .i = &.{ 5, 3 } }); // PositionAndVelocity
    try b.constraintOp(.notify_constraint_shape_changed, welded, .{ .body = c, .f = &.{ 0.02, 0, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, joint, .{});
    try b.constraintOp(.constraint_settings_hash, welded, .{});
}

/// A SixDOF from the world (Body::sFixedToWorld as body 1) at `p`, the constraint frame (axis_x, axis_y) on both sides,
/// every translation fixed and every rotation free
fn worldSixDOF(body2: u32, p: [3]Real, axis_x: [3]f32, axis_y: [3]f32) SimConstraint {
    var c = Desc.sixDOF(invalid_handle, body2, p, axis_x, axis_y);
    for (0..3) |axis| Desc.setSixDOFLimits(&c, axis, 0, 0);
    return c;
}

/// The limits spring of SixDOF translation axis `axis` (0..2)
fn setSixDOFSpring(c: *SimConstraint, axis: usize, mode: u32, frequency: f32, damping: f32) void {
    Desc.setSpring(c, 2 + axis, 30 + 2 * axis, mode, frequency, damping);
}

/// The motor settings of SixDOF axis `axis` (0..5)
fn setSixDOFMotor(c: *SimConstraint, axis: usize, mode: u32, values: [6]f32) void {
    Desc.setMotor(c, 5 + axis, 36 + 6 * axis, mode, values);
}

/// SixDOF limits and their springs: boxes hanging from the world drop into a hard translation limit and into soft
/// limits in all three spring modes; a box with every translation fixed at an offset (the point part with
/// GetTranslationLimitsMin) and every rotation fixed at an angle (the Euler part with GetRotationLimitsMin); a spinning
/// bar into twist and cone swing limits; a bar into asymmetric pyramid swing limits; a box on fixed translations with
/// soft springs (TestSixDOFSpring); a pair of dynamic boxes in local space whose frames differ, one hanging from a
/// point constraint; a box dragged by a kinematic body; SetTranslationLimits / SetRotationLimits (narrower limits the
/// body is outside of, a fixed axis that becomes limited, everything free) and SetLimitsSpringSettings while they act
fn buildSixDOFLimits(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    const bar = try b.box(.{ 0.5, 0.1, 0.1 }, 0.05);
    b.num_updates = 100;
    const x_axis: [3]f32 = .{ 1, 0, 0 };
    const y_axis: [3]f32 = .{ 0, 1, 0 };

    // Boxes that drop along Y into a hard limit and soft limits (the other axes fixed)
    var drops: [4]u32 = undefined;
    for (&drops, 0..) |*h, k| {
        const x: Real = 2 * real(k);
        const body = try b.body(.{ .position = .{ x, 6, 0 }, .shape = box, .allow_sleeping = 0, .object_layer = Layers.unused1 }, .activate);
        var c = worldSixDOF(body, .{ x, 6, 0 }, x_axis, y_axis);
        Desc.setSixDOFLimits(&c, 1, -0.6, 0.2);
        for (3..6) |axis| Desc.setSixDOFLimits(&c, axis, 0, 0);
        switch (k) {
            1 => setSixDOFSpring(&c, 1, 0, 2, 0.3), // Frequency and damping
            2 => setSixDOFSpring(&c, 1, 1, 300, 5), // Stiffness and damping
            3 => setSixDOFSpring(&c, 1, 2, 40, 2), // Mass normalized
            else => {},
        }
        h.* = try b.constraint(c);
    }

    // Fixed at an offset and an orientation: the position solver moves the box there
    const offset_box = try b.body(.{ .position = .{ 9, 6, 0 }, .shape = box, .object_layer = Layers.unused1 }, .activate);
    var offset = worldSixDOF(offset_box, .{ 9, 6, 0 }, x_axis, y_axis);
    Desc.setSixDOFLimits(&offset, 0, 0.3, 0.3);
    Desc.setSixDOFLimits(&offset, 2, -0.2, -0.2);
    Desc.setSixDOFLimits(&offset, 3, 0.2, 0.2);
    Desc.setSixDOFLimits(&offset, 4, -0.3, -0.3);
    Desc.setSixDOFLimits(&offset, 5, 0.1, 0.1);
    offset.i[1] = 1; // Pyramid: the swing limits are not made symmetric
    const offset_handle = try b.constraint(offset);

    // A spinning bar into twist limits (constraint X is the bar's long axis) and cone swing limits
    const twist_bar = try b.body(.{ .position = .{ 12.5, 6, 0 }, .shape = bar, .angular_velocity = .{ 3, 1, 0.5 }, .object_layer = Layers.unused1 }, .activate);
    var twist = worldSixDOF(twist_bar, .{ 12, 6, 0 }, x_axis, y_axis);
    Desc.setSixDOFLimits(&twist, 3, -0.3, 0.5);
    Desc.setSixDOFLimits(&twist, 4, -0.4, 0.4);
    Desc.setSixDOFLimits(&twist, 5, -0.6, 0.6);
    twist.f[12 + 3] = 0.5; // Twist friction
    const twist_handle = try b.constraint(twist);

    // A bar that swings down into asymmetric pyramid swing limits, the twist free
    const swing_bar = try b.body(.{ .position = .{ 15.5, 6, 0 }, .shape = bar, .object_layer = Layers.unused1 }, .activate);
    var swing = worldSixDOF(swing_bar, .{ 15, 6, 0 }, x_axis, y_axis);
    swing.i[1] = 1; // Pyramid
    Desc.setSixDOFLimits(&swing, 4, -0.2, 0.6);
    Desc.setSixDOFLimits(&swing, 5, -0.5, 0.1);
    _ = try b.constraint(swing);

    // Soft springs on every translation axis (the fixed axes become axis parts), started away from the anchor
    const spring_box = try b.body(.{ .position = .{ 18.4, 6.3, 0.2 }, .shape = box, .gravity_factor = 0, .allow_sleeping = 0, .object_layer = Layers.unused1 }, .activate);
    var spring = Desc.sixDOF(invalid_handle, spring_box, .{ 18, 6, 0 }, x_axis, y_axis);
    spring.r[3] = 18.4;
    spring.r[4] = 6.3;
    spring.r[5] = 0.2;
    for (0..3) |axis| {
        Desc.setSixDOFLimits(&spring, axis, 0, 0);
        setSixDOFSpring(&spring, axis, @intCast(axis), if (axis == 1) 200 else 1.5, 0.1);
    }
    const spring_handle = try b.constraint(spring);

    // A pair of dynamic boxes: a hangs from the world on a point constraint, c on a SixDOF in local space whose frames
    // differ by a turn around Z, translation limited along the constraint X axis, a cone swing limit
    const a = try b.body(.{ .position = .{ 25, 6, 0 }, .shape = box, .object_layer = Layers.unused1 }, .activate);
    const c = try b.body(.{ .position = .{ 25.8, 6, 0 }, .shape = box, .angular_velocity = .{ 0, 2, 1 }, .object_layer = Layers.unused1 }, .activate);
    _ = try b.constraint(Desc.pointWorld(invalid_handle, a, .{ 25, 6.4, 0 }));
    var local = Desc.sixDOF(a, c, .{ 0.4, 0, 0 }, x_axis, y_axis);
    local.i[0] = ConstraintSpace.local_to_body_com;
    local.r[3] = -0.4;
    local.r[4] = 0;
    local.r[5] = 0;
    local.f[6] = @floatCast(@cos(std.math.pi / 6.0)); // Axis X2
    local.f[7] = @floatCast(@sin(std.math.pi / 6.0));
    local.f[9] = @floatCast(-@sin(std.math.pi / 6.0)); // Axis Y2
    local.f[10] = @floatCast(@cos(std.math.pi / 6.0));
    Desc.setSixDOFLimits(&local, 0, -0.1, 0.2);
    Desc.setSixDOFLimits(&local, 1, 0, 0);
    Desc.setSixDOFLimits(&local, 2, 0, 0);
    Desc.setSixDOFLimits(&local, 4, -0.3, 0.3);
    Desc.setSixDOFLimits(&local, 5, -0.3, 0.3);
    const pair = try b.constraint(local);

    // A box hanging from a kinematic body that turns around Z, rotation limited: the box is dragged along
    const kinematic = try b.body(.{ .position = .{ 30, 6, 0 }, .shape = box, .motion_type = 1, .angular_velocity = .{ 0, 0, 1.5 }, .object_layer = Layers.unused1 }, .activate);
    const dragged = try b.body(.{ .position = .{ 30, 5.2, 0 }, .shape = box, .object_layer = Layers.unused1 }, .activate);
    var kinematic_desc = Desc.sixDOF(kinematic, dragged, .{ 30, 5.6, 0 }, x_axis, y_axis);
    for (0..3) |axis| Desc.setSixDOFLimits(&kinematic_desc, axis, 0, 0);
    Desc.setSixDOFLimits(&kinematic_desc, 3, -0.2, 0.2);
    Desc.setSixDOFLimits(&kinematic_desc, 4, -0.2, 0.2);
    Desc.setSixDOFLimits(&kinematic_desc, 5, -0.4, 0.4);
    _ = try b.constraint(kinematic_desc);

    b.at(40);
    try b.constraintOp(.six_dof_set_translation_limits, drops[0], .{ .f = &.{ 0, -0.3, 0, 0, 0.2, 0 } }); // Outside: pushed back
    try b.constraintOp(.six_dof_set_limits_spring_settings, drops[1], .{ .i = &.{ 1, 0 }, .f = &.{ 0, 0 } }); // Hard now
    try b.constraintOp(.six_dof_set_limits_spring_settings, drops[0], .{ .i = &.{ 1, 1 }, .f = &.{ 150, 2 } }); // Soft now
    try b.constraintOp(.constraint_settings_hash, pair, .{});
    b.at(55);
    try b.constraintOp(.six_dof_set_rotation_limits, twist_handle, .{ .f = &.{ -flt_max, -flt_max, -flt_max, flt_max, flt_max, flt_max } }); // Free
    try b.constraintOp(.six_dof_set_max_friction, twist_handle, .{ .i = &.{3}, .f = &.{2} });
    try b.constraintOp(.constraint_settings_hash, offset_handle, .{});
    b.at(70);
    try b.constraintOp(.six_dof_set_translation_limits, offset_handle, .{ .f = &.{ -0.2, 0, -0.2, 0.5, 0, -0.2 } }); // X limited now
    try b.constraintOp(.six_dof_set_rotation_limits, offset_handle, .{ .f = &.{ 0, 0, 0, 0, 0, 0 } }); // Fixed at zero
    try b.constraintOp(.six_dof_set_limits_spring_settings, spring_handle, .{ .i = &.{ 0, 0 }, .f = &.{ 0, 0 } }); // Hard X
    try b.constraintOp(.notify_constraint_shape_changed, pair, .{ .body = c, .f = &.{ 0.02, 0, 0 } });
    try b.constraintOp(.constraint_settings_hash, twist_handle, .{});
}

/// SixDOF motors in every EMotorState on translation and rotation axes (zero gravity, every body on a SixDOF to the
/// world): friction only (motor off); translation velocity motors with a force limit and a changing target; translation
/// position motors, one without stiffness (inactive) that becomes a damping only position and velocity motor; rotation
/// velocity motors; rotation position motors on every combination of axes (the projections of the difference to the
/// target orientation) with SetTargetOrientationCS / BS; a mass normalized motor with asymmetric torque limits;
/// rotation motors on a fully fixed rotation (not solved); and one box that goes through every state
fn buildSixDOFMotors(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.3, 0.2, 0.25 }, 0.05);
    b.num_updates = 90;
    b.gravity = .{ 0, 0, 0 };
    const x_axis: [3]f32 = .{ 1, 0, 0 };
    const y_axis: [3]f32 = .{ 0, 1, 0 };
    const num = 16;
    var bodies: [num]u32 = undefined;
    var joints: [num]u32 = undefined;
    for (&bodies, &joints, 0..) |*h, *j, k| {
        const x: Real = 2 * real(k);
        const velocity: [3]f32 = if (k == 0) .{ 2, 0, 1 } else @splat(0);
        const spin: [3]f32 = if (k == 0) .{ 0, 2, 0 } else @splat(0);
        h.* = try b.body(.{ .position = .{ x, 3, 0 }, .shape = box, .linear_velocity = velocity, .angular_velocity = spin, .allow_sleeping = 0, .object_layer = Layers.unused1 }, .activate);
        var c = Desc.sixDOF(invalid_handle, h.*, .{ x, 3, 0 }, x_axis, y_axis);
        switch (k) {
            0 => {
                c.f[12 + 0] = 3; // Translation X friction
                c.f[12 + 4] = 1; // Rotation Y friction
            },
            1 => setSixDOFMotor(&c, 0, 0, .{ 2, 1, -5, 5, -flt_max, flt_max }), // Force limited
            3 => setSixDOFMotor(&c, 2, 1, .{ 0, 20, -flt_max, flt_max, -flt_max, flt_max }), // Damping only
            4 => Desc.setSixDOFLimits(&c, 1, -0.2, 0.2), // Velocity motors with a limited axis
            12 => {
                setSixDOFMotor(&c, 3, 2, .{ 30, 4, -flt_max, flt_max, -2, 10 }); // Mass normalized, asymmetric torque limits
                setSixDOFMotor(&c, 5, 2, .{ 30, 4, -flt_max, flt_max, -10, 10 });
            },
            13 => for (3..6) |axis| Desc.setSixDOFLimits(&c, axis, 0, 0), // Rotation fully fixed
            else => {},
        }
        j.* = try b.constraint(c);
    }
    const target = axisAngle(.{ 0.3, -0.5, 0.8 }, 0.9);
    b.at(1);
    try b.constraintOp(.six_dof_set_motor_state, joints[1], .{ .i = &.{ 0, 1 } }); // Translation X velocity
    try b.constraintOp(.six_dof_set_target_velocity_cs, joints[1], .{ .f = &.{ 1.5, 0, 0 } });
    for (0..3) |axis| try b.constraintOp(.six_dof_set_motor_state, joints[2], .{ .i = &.{ @intCast(axis), 2 } }); // Translation position
    try b.constraintOp(.six_dof_set_target_position_cs, joints[2], .{ .f = &.{ 0.5, -0.3, 0.2 } });
    try b.constraintOp(.six_dof_set_motor_settings, joints[3], .{ .i = &.{ 2, 1 }, .f = &.{ 0, 0, -flt_max, flt_max, -flt_max, flt_max } }); // No stiffness
    try b.constraintOp(.six_dof_set_motor_state, joints[3], .{ .i = &.{ 2, 2 } }); // Position without stiffness: inactive
    try b.constraintOp(.six_dof_set_target_position_cs, joints[3], .{ .f = &.{ 0, 0, 1 } });
    try b.constraintOp(.six_dof_set_motor_state, joints[4], .{ .i = &.{ 4, 1 } }); // Rotation Y velocity
    try b.constraintOp(.six_dof_set_motor_state, joints[4], .{ .i = &.{ 1, 1 } }); // Translation Y velocity (into the limit)
    try b.constraintOp(.six_dof_set_target_angular_velocity_cs, joints[4], .{ .f = &.{ 0, 1.2, 0 } });
    try b.constraintOp(.six_dof_set_target_velocity_cs, joints[4], .{ .f = &.{ 0, 0.8, 0 } });
    // Rotation position motors on every combination of axes (masks 1..7)
    for (1..8) |mask| {
        const j = joints[4 + mask];
        for (0..3) |axis| {
            if ((mask & (@as(usize, 1) << @intCast(axis))) != 0)
                try b.constraintOp(.six_dof_set_motor_state, j, .{ .i = &.{ @intCast(3 + axis), if (mask == 7 and axis == 1) 3 else 2 } });
        }
        if (mask % 2 == 0)
            try b.constraintOp(.six_dof_set_target_orientation_cs, j, .{ .f = &target })
        else
            try b.constraintOp(.six_dof_set_target_orientation_bs, j, .{ .f = &target });
    }
    try b.constraintOp(.six_dof_set_motor_state, joints[12], .{ .i = &.{ 3, 2 } });
    try b.constraintOp(.six_dof_set_motor_state, joints[12], .{ .i = &.{ 5, 3 } });
    try b.constraintOp(.six_dof_set_target_orientation_cs, joints[12], .{ .f = &axisAngle(.{ 1, 0, 1 }, -2.5) });
    try b.constraintOp(.six_dof_set_target_angular_velocity_cs, joints[12], .{ .f = &.{ 0, 0, 0.5 } });
    try b.constraintOp(.six_dof_set_motor_state, joints[13], .{ .i = &.{ 3, 1 } }); // Rotation fixed: the motor is not solved
    try b.constraintOp(.six_dof_set_target_angular_velocity_cs, joints[13], .{ .f = &.{ 2, 0, 0 } });
    try b.constraintOp(.six_dof_set_motor_state, joints[14], .{ .i = &.{ 5, 1 } }); // The box that goes through every state
    try b.constraintOp(.six_dof_set_target_angular_velocity_cs, joints[14], .{ .f = &.{ 0, 0, 1 } });
    try b.constraintOp(.six_dof_set_motor_state, joints[14], .{ .i = &.{ 2, 1 } });
    try b.constraintOp(.six_dof_set_target_velocity_cs, joints[14], .{ .f = &.{ 0, 0, -0.5 } });
    try b.constraintOp(.six_dof_set_max_friction, joints[15], .{ .i = &.{1}, .f = &.{2} });
    try b.constraintOp(.six_dof_set_max_friction, joints[15], .{ .i = &.{3}, .f = &.{1} });
    b.at(20);
    try b.constraintOp(.six_dof_set_motor_state, joints[14], .{ .i = &.{ 5, 2 } });
    try b.constraintOp(.six_dof_set_motor_state, joints[14], .{ .i = &.{ 2, 2 } });
    try b.constraintOp(.six_dof_set_target_orientation_cs, joints[14], .{ .f = &axisAngle(.{ 0, 0, 1 }, 0.7) });
    try b.constraintOp(.six_dof_set_target_position_cs, joints[14], .{ .f = &.{ 0, 0, 0.4 } });
    try b.op(.add_impulse, bodies[15], .{ .f = &.{ 0, 2, 0 } });
    try b.op(.add_angular_impulse, bodies[15], .{ .f = &.{ 1, 0, 0 } });
    b.at(30);
    try b.constraintOp(.six_dof_set_target_velocity_cs, joints[1], .{ .f = &.{ -1, 0, 0 } });
    try b.constraintOp(.six_dof_set_motor_state, joints[3], .{ .i = &.{ 2, 3 } }); // Damping only: active
    try b.constraintOp(.six_dof_set_target_velocity_cs, joints[3], .{ .f = &.{ 0, 0, 0.6 } });
    try b.constraintOp(.constraint_settings_hash, joints[12], .{});
    b.at(40);
    try b.constraintOp(.six_dof_set_motor_state, joints[14], .{ .i = &.{ 5, 3 } });
    try b.constraintOp(.six_dof_set_motor_state, joints[14], .{ .i = &.{ 2, 3 } });
    try b.constraintOp(.six_dof_set_target_angular_velocity_cs, joints[14], .{ .f = &.{ 0, 0, -0.3 } });
    try b.constraintOp(.six_dof_set_motor_settings, joints[2], .{ .i = &.{ 0, 1 }, .f = &.{ 80, 6, -20, 20, -flt_max, flt_max } });
    try b.constraintOp(.six_dof_set_target_orientation_bs, joints[6], .{ .f = &axisAngle(.{ 0, 1, 0 }, -0.6) });
    b.at(50);
    try b.constraintOp(.six_dof_set_motor_state, joints[1], .{ .i = &.{ 0, 0 } }); // Off: the box slides on
    try b.constraintOp(.six_dof_set_motor_state, joints[9], .{ .i = &.{ 4, 1 } }); // Mask 0b111 -> 0b101 with a velocity motor
    try b.constraintOp(.six_dof_set_target_angular_velocity_cs, joints[9], .{ .f = &.{ 0, -0.5, 0 } });
    try b.constraintOp(.six_dof_set_rotation_limits, joints[13], .{ .f = &.{ -0.5, -0.5, -0.5, 0.5, 0.5, 0.5 } }); // The motor acts now
    b.at(60);
    try b.constraintOp(.six_dof_set_motor_state, joints[14], .{ .i = &.{ 5, 0 } });
    try b.constraintOp(.six_dof_set_motor_state, joints[14], .{ .i = &.{ 2, 0 } });
    try b.constraintOp(.six_dof_set_max_friction, joints[14], .{ .i = &.{5}, .f = &.{3} });
    try b.constraintOp(.six_dof_set_max_friction, joints[0], .{ .i = &.{0}, .f = &.{0} });
    try b.constraintOp(.constraint_settings_hash, joints[14], .{});
    b.at(75);
    try b.constraintOp(.reset_constraint_warm_start, joints[2], .{});
    try b.constraintOp(.six_dof_set_motor_state, joints[13], .{ .i = &.{ 4, 2 } });
    try b.constraintOp(.six_dof_set_target_orientation_cs, joints[13], .{ .f = &axisAngle(.{ 0, 1, 0 }, 0.3) });
}

/// Ragdolls built from SixDOF joints (translation fixed, twist and swing limits of both swing types, friction) with a
/// welded head (FixedConstraint) and hands on point constraints, in one island with contacts: they drop onto a floor,
/// crates and each other; one ragdoll's knee has a position motor
fn buildSixDOFRagdolls(b: *SceneBuilder) Allocator.Error!void {
    const torso = try b.box(.{ 0.18, 0.25, 0.1 }, 0.05);
    const head = try b.sphere(0.12);
    const limb = try b.capsule(0.15, 0.06);
    const hand = try b.sphere(0.06);
    const crate = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    b.num_updates = 110;
    b.max_contact_constraints = 2048;
    _ = try addFloor(b, 15, 0);
    for (0..3) |k| _ = try b.body(.{ .position = .{ -1 + real(k), 0.4, 0.3 * real(k) }, .shape = crate, .object_layer = Layers.moving }, .activate);
    for (0..3) |r| {
        const ox: Real = -0.6 + 0.6 * real(r);
        const oy: Real = 2.5 + 1.4 * real(r);
        const oz: Real = 0.4 * real(r);
        const tilt = axisAngle(.{ 1, 0, 0.3 }, 0.4 * real(r));
        const t = try b.body(.{ .position = .{ ox, oy, oz }, .rotation = tilt, .shape = torso, .friction = 0.6, .object_layer = Layers.moving }, .activate);
        const h = try b.body(.{ .position = .{ ox, oy + 0.4, oz }, .shape = head, .object_layer = Layers.moving }, .activate);
        _ = try b.constraint(Desc.fixedAuto(t, h));
        // Arms and legs: SixDOF joints at the shoulders and hips, the twist around the limb (constraint X along Y)
        var knee: u32 = invalid_handle;
        for (0..4) |l| {
            const side: Real = if (l % 2 == 0) -1 else 1;
            const arm = l < 2;
            const joint_p: [3]Real = .{ ox + side * @as(Real, if (arm) 0.22 else 0.1), oy + @as(Real, if (arm) 0.2 else -0.27), oz };
            const upper = try b.body(.{ .position = .{ joint_p[0], joint_p[1] - 0.22, joint_p[2] }, .shape = limb, .friction = 0.6, .object_layer = Layers.moving }, .activate);
            var c = Desc.sixDOF(t, upper, joint_p, .{ 0, -1, 0 }, .{ 1, 0, 0 });
            for (0..3) |axis| Desc.setSixDOFLimits(&c, axis, 0, 0);
            Desc.setSixDOFLimits(&c, 3, -0.4, 0.4);
            if (arm) {
                Desc.setSixDOFLimits(&c, 4, -1.2, 1.2); // Cone
                Desc.setSixDOFLimits(&c, 5, -0.8, 0.8);
            } else {
                c.i[1] = 1; // Pyramid
                Desc.setSixDOFLimits(&c, 4, -0.3, 1.2);
                Desc.setSixDOFLimits(&c, 5, -0.4, 0.2);
            }
            c.f[12 + 3] = 0.05; // Joint friction
            c.f[12 + 4] = 0.05;
            _ = try b.constraint(c);
            // The lower limb on a SixDOF with one rotation (an elbow / a knee), the hand on a point constraint
            const lower_p: [3]Real = .{ joint_p[0], joint_p[1] - 0.44, joint_p[2] };
            const lower = try b.body(.{ .position = .{ lower_p[0], lower_p[1] - 0.22, lower_p[2] }, .shape = limb, .friction = 0.6, .object_layer = Layers.moving }, .activate);
            var hinge_like = Desc.sixDOF(upper, lower, lower_p, .{ 1, 0, 0 }, .{ 0, 1, 0 });
            for (0..3) |axis| Desc.setSixDOFLimits(&hinge_like, axis, 0, 0);
            Desc.setSixDOFLimits(&hinge_like, 3, @as(f32, if (arm) -1.5 else 0), @as(f32, if (arm) 0 else 1.5));
            Desc.setSixDOFLimits(&hinge_like, 4, 0, 0);
            Desc.setSixDOFLimits(&hinge_like, 5, 0, 0);
            const joint = try b.constraint(hinge_like);
            if (!arm and r == 1) knee = joint;
            if (arm) {
                const hp: [3]Real = .{ lower_p[0], lower_p[1] - 0.5, lower_p[2] };
                const hd = try b.body(.{ .position = hp, .shape = hand, .object_layer = Layers.moving }, .activate);
                _ = try b.constraint(Desc.pointWorld(lower, hd, .{ hp[0], hp[1] + 0.06, hp[2] }));
            }
        }
        if (knee != invalid_handle) {
            b.at(1);
            try b.constraintOp(.six_dof_set_motor_state, knee, .{ .i = &.{ 3, 2 } });
            try b.constraintOp(.six_dof_set_target_orientation_cs, knee, .{ .f = &axisAngle(.{ 1, 0, 0 }, 1.0) });
            b.at(60);
            try b.constraintOp(.six_dof_set_motor_state, knee, .{ .i = &.{ 3, 0 } });
            try b.constraintOp(.constraint_settings_hash, knee, .{});
        }
    }
}

/// A net of boxes lying on the floor, each joined to its neighbours by SixDOF joints (translation fixed, every third
/// one fully fixed, the others with twist and cone limits) and the corners pinned to the world with point constraints:
/// one island of more than 128 constraints that the LargeIslandSplitter splits, which falls asleep from the final
/// batch of a split island; an impulse wakes it through its constraints
fn buildSixDOFNetSplitter(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.4, 0.15, 0.4 }, 0.05);
    b.num_updates = 120;
    b.max_contact_constraints = 2048;
    b.settings.time_before_sleep = 0.25;
    _ = try addFloor(b, 20, 0);
    const nx = 10;
    const nz = 13;
    var handles: [nx * nz]u32 = undefined;
    var count: u32 = 0;
    for (0..nz) |z| {
        for (0..nx) |x| {
            const px: Real = real(x) - 5.0;
            const pz: Real = real(z) - 6.5;
            const h = try b.body(.{ .position = .{ px, 0.151, pz }, .shape = box, .object_layer = Layers.moving }, .activate);
            handles[z * nx + x] = h;
            const neighbours = [2]?struct { u32, [3]Real, [3]f32 }{
                if (x > 0) .{ handles[z * nx + x - 1], .{ px - 0.5, 0.151, pz }, .{ 1, 0, 0 } } else null,
                if (z > 0) .{ handles[(z - 1) * nx + x], .{ px, 0.151, pz - 0.5 }, .{ 0, 0, 1 } } else null,
            };
            for (neighbours) |neighbour| {
                const n = neighbour orelse continue;
                var c = Desc.sixDOF(n[0], h, n[1], n[2], .{ 0, 1, 0 });
                for (0..3) |axis| Desc.setSixDOFLimits(&c, axis, 0, 0);
                if (count % 3 == 0) {
                    for (3..6) |axis| Desc.setSixDOFLimits(&c, axis, 0, 0);
                } else {
                    Desc.setSixDOFLimits(&c, 3, -0.2, 0.2);
                    Desc.setSixDOFLimits(&c, 4, -0.3, 0.3);
                    Desc.setSixDOFLimits(&c, 5, -0.3, 0.3);
                }
                count += 1;
                _ = try b.constraint(c);
            }
        }
    }
    for ([_]usize{ 0, nx - 1, (nz - 1) * nx, nz * nx - 1 }) |i| {
        const p: [3]Real = .{ real(i % nx) - 5.0, 0.151, real(i / nx) - 6.5 };
        _ = try b.constraint(Desc.pointWorld(invalid_handle, handles[i], p));
    }
    b.at(10);
    try b.op(.add_impulse, handles[nx * nz / 2], .{ .f = &.{ 0, 3, 0 } });
    b.at(100);
    try b.expectSleeping(handles[0]);
    try b.expectSleeping(handles[nx * nz - 1]);
    try b.op(.add_impulse, handles[nx * nz / 2 + 3], .{ .f = &.{ 0, 4, 0 } });
    b.at(101);
    try b.expectAwake(handles[nx * nz / 2 + 3]);
}

/// SixDOF constraints and sleeping bodies, and cross save / restore of the SixDOF state (the warm start impulses of
/// every part, the motor states and the four targets; the limits, springs, friction and motor settings are not state):
/// a bar with friction and limits comes to rest and falls asleep, a motor set on the sleeping constraint wakes nothing
/// until an impulse does; a SixDOF created between an awake and a sleeping body wakes the sleeping one; a box with
/// motors on every axis is restored from snapshots of everything and of the constraints only after its motor states and
/// targets changed
fn buildSixDOFSleepAndSaveRestore(b: *SceneBuilder) Allocator.Error!void {
    const bar = try b.box(.{ 0.1, 0.5, 0.1 }, 0.05);
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.num_updates = 90;
    b.settings.time_before_sleep = 0.2;
    const state_constraints: u32 = 8; // EStateRecorderState::Constraints
    const x_axis: [3]f32 = .{ 1, 0, 0 };
    const y_axis: [3]f32 = .{ 0, 1, 0 };

    // A bar hanging from a SixDOF with friction and limits, at rest
    const hanging = try b.body(.{ .position = .{ 0, 4.5, 0 }, .shape = bar, .object_layer = Layers.unused1 }, .activate);
    var hanging_desc = worldSixDOF(hanging, .{ 0, 5, 0 }, x_axis, y_axis);
    Desc.setSixDOFLimits(&hanging_desc, 3, -0.3, 0.3);
    Desc.setSixDOFLimits(&hanging_desc, 4, -0.3, 0.3);
    Desc.setSixDOFLimits(&hanging_desc, 5, -0.3, 0.3);
    for (3..6) |axis| hanging_desc.f[12 + axis] = 5;
    const hanging_joint = try b.constraint(hanging_desc);

    // An awake body that spins and a sleeping body
    const awake = try b.body(.{ .position = .{ 3, 5, 0 }, .shape = box, .angular_velocity = .{ 0, 0, 1 }, .gravity_factor = 0, .allow_sleeping = 0, .object_layer = Layers.unused1 }, .activate);
    const asleep = try b.body(.{ .position = .{ 3.5, 5, 0 }, .shape = box, .gravity_factor = 0, .object_layer = Layers.unused1 }, .dont_activate);

    // A box with limits and motors on every axis
    const m = try b.body(.{ .position = .{ 6, 3, 0 }, .shape = box, .gravity_factor = 0, .allow_sleeping = 0, .object_layer = Layers.unused1 }, .activate);
    var motor_desc = Desc.sixDOF(invalid_handle, m, .{ 6, 3, 0 }, x_axis, y_axis);
    Desc.setSixDOFLimits(&motor_desc, 0, -1, 1);
    Desc.setSixDOFLimits(&motor_desc, 4, -0.8, 0.8);
    Desc.setSixDOFLimits(&motor_desc, 5, -0.8, 0.8);
    const motor_joint = try b.constraint(motor_desc);

    b.at(2);
    try b.constraintOp(.six_dof_set_motor_state, motor_joint, .{ .i = &.{ 0, 1 } });
    try b.constraintOp(.six_dof_set_motor_state, motor_joint, .{ .i = &.{ 3, 1 } });
    try b.constraintOp(.six_dof_set_motor_state, motor_joint, .{ .i = &.{ 4, 2 } });
    try b.constraintOp(.six_dof_set_motor_state, motor_joint, .{ .i = &.{ 5, 2 } });
    try b.constraintOp(.six_dof_set_target_velocity_cs, motor_joint, .{ .f = &.{ 0.5, 0, 0 } });
    try b.constraintOp(.six_dof_set_target_angular_velocity_cs, motor_joint, .{ .f = &.{ 0.8, 0, 0 } });
    try b.constraintOp(.six_dof_set_target_orientation_cs, motor_joint, .{ .f = &axisAngle(.{ 0, 1, 1 }, 0.5) });
    b.at(5);
    try b.expectSleeping(asleep);
    var pair_desc = Desc.sixDOF(awake, asleep, .{ 3.25, 5, 0 }, x_axis, y_axis);
    for (0..3) |axis| Desc.setSixDOFLimits(&pair_desc, axis, 0, 0);
    Desc.setSixDOFLimits(&pair_desc, 3, 0, 0);
    const pair_joint = try b.constraint(pair_desc); // Wakes asleep in the next update
    b.at(6);
    try b.expectAwake(asleep);
    b.at(20);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(25);
    try b.constraintOp(.six_dof_set_motor_state, motor_joint, .{ .i = &.{ 0, 2 } });
    try b.constraintOp(.six_dof_set_motor_state, motor_joint, .{ .i = &.{ 3, 3 } });
    try b.constraintOp(.six_dof_set_motor_state, motor_joint, .{ .i = &.{ 4, 0 } });
    try b.constraintOp(.six_dof_set_target_position_cs, motor_joint, .{ .f = &.{ -0.5, 0, 0 } });
    try b.constraintOp(.six_dof_set_target_orientation_bs, motor_joint, .{ .f = &axisAngle(.{ 1, 0, 0 }, -0.4) });
    try b.constraintOp(.six_dof_set_translation_limits, motor_joint, .{ .f = &.{ -0.8, -flt_max, -flt_max, 0.8, flt_max, flt_max } }); // Not state
    b.at(35);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // The motors of update 20 again
    b.at(40);
    try b.expectSleeping(hanging);
    try b.constraintOp(.six_dof_set_motor_state, hanging_joint, .{ .i = &.{ 4, 1 } }); // The bar sleeps: nothing happens
    try b.constraintOp(.six_dof_set_target_angular_velocity_cs, hanging_joint, .{ .f = &.{ 0, 1, 0 } });
    try b.expectConstraintState(hanging_joint, ConstraintState.inactive);
    try b.sysOp(.snapshot, .{ .index = 1, .i = &.{state_constraints} });
    b.at(41);
    try b.expectSleeping(hanging);
    b.at(45);
    try b.op(.add_impulse, hanging, .{ .f = &.{ 1, 0, 0 } }); // Wakes the bar, the motor drives it into its limit
    try b.constraintOp(.six_dof_set_motor_state, motor_joint, .{ .i = &.{ 5, 3 } });
    try b.constraintOp(.reset_constraint_warm_start, pair_joint, .{});
    b.at(55);
    try b.expectAwake(hanging);
    try b.sysOp(.restore_cross, .{ .index = 1 }); // The constraints of update 40: the motor states and their impulses
    try b.constraintOp(.constraint_settings_hash, motor_joint, .{});
    b.at(70);
    try b.sysOp(.snapshot, .{ .index = 2 });
    try b.op(.add_angular_impulse, m, .{ .f = &.{ 0, 2, 0 } });
    b.at(80);
    try b.sysOp(.restore_cross, .{ .index = 2 });
}

/// A chain of bars on SixDOF joints (translation fixed, alternating twist / swing limits, a soft translation spring, a
/// fully fixed joint) with a velocity motor on the first joint, with several collision steps per update and changing
/// update parameters (the warm start impulses are scaled by the step ratio)
fn buildSixDOFCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const bar = try b.box(.{ 0.3, 0.08, 0.08 }, 0.04);
            b.num_updates = 50;
            b.collision_steps = steps;
            b.delta_time = @as(f32, @floatFromInt(steps)) / 60.0;
            var previous: u32 = invalid_handle;
            var first: u32 = 0;
            for (0..6) |i| {
                const x: Real = 0.7 * real(i);
                const h = try b.body(.{ .position = .{ x + 0.3, 6, 0 }, .shape = bar, .object_layer = Layers.unused1 }, .activate);
                var c = Desc.sixDOF(previous, h, .{ x - 0.05, 6, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 });
                for (0..3) |axis| Desc.setSixDOFLimits(&c, axis, 0, 0);
                switch (i % 3) {
                    0 => Desc.setSixDOFLimits(&c, 3, -0.4, 0.4),
                    1 => {
                        Desc.setSixDOFLimits(&c, 4, -0.5, 0.5);
                        Desc.setSixDOFLimits(&c, 5, -0.5, 0.5);
                    },
                    else => {
                        c.i[1] = 1;
                        Desc.setSixDOFLimits(&c, 4, -0.2, 0.6);
                    },
                }
                if (i == 4) setSixDOFSpring(&c, 1, 0, 3, 0.5);
                if (i == 5) for (3..6) |axis| Desc.setSixDOFLimits(&c, axis, 0, 0);
                const handle = try b.constraint(c);
                if (i == 0) first = handle;
                previous = h;
            }
            b.at(1);
            try b.constraintOp(.six_dof_set_motor_state, first, .{ .i = &.{ 3, 1 } });
            try b.constraintOp(.six_dof_set_target_angular_velocity_cs, first, .{ .f = &.{ -1, 0, 0 } });
            b.at(20);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 90.0}, .count = 1 });
            try b.constraintOp(.six_dof_set_motor_state, first, .{ .i = &.{ 3, 2 } });
            try b.constraintOp(.six_dof_set_target_orientation_cs, first, .{ .f = &axisAngle(.{ 1, 0, 0 }, 0.3) });
            b.at(30);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 20.0}, .count = steps + 1 });
        }
    }.build;
}

/// SixDOF joints far from the origin (world space positions are Real: in double precision the anchors keep their
/// precision): a chain of planks hangs from the world on SixDOF joints and swings onto a floor and a box, a door on a
/// SixDOF with a rotation position motor and a translation limit
fn buildSixDOFFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const plank = try b.box(.{ 0.3, 0.05, 0.25 }, 0.02);
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    const door = try b.box(.{ 0.5, 1, 0.05 }, 0.02);
    b.num_updates = 70;
    const o: [3]Real = .{ -7000, 150, 11000 };
    const floor = try b.box(.{ 10, 1, 10 }, 0);
    _ = try b.body(.{ .position = .{ o[0], o[1] - 1, o[2] }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    var previous: u32 = invalid_handle;
    for (0..5) |i| {
        const p: [3]Real = .{ o[0] - 3 + 0.65 * real(i), o[1] + 2.5, o[2] };
        const h = try b.body(.{ .position = p, .shape = plank, .object_layer = Layers.moving }, .activate);
        var c = Desc.sixDOF(previous, h, .{ p[0] - 0.325, p[1], p[2] }, .{ 0, 0, 1 }, .{ 1, 0, 0 });
        for (0..3) |axis| Desc.setSixDOFLimits(&c, axis, 0, 0);
        Desc.setSixDOFLimits(&c, 3, -1.2, 1.2);
        Desc.setSixDOFLimits(&c, 4, -0.1, 0.1);
        Desc.setSixDOFLimits(&c, 5, -0.1, 0.1);
        _ = try b.constraint(c);
        previous = h;
    }
    _ = try b.body(.{ .position = .{ o[0] - 1, o[1] + 0.25, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
    const d = try b.body(.{ .position = .{ o[0] + 4.5, o[1] + 1.2, o[2] }, .shape = door, .object_layer = Layers.moving }, .activate);
    var door_desc = Desc.sixDOF(invalid_handle, d, .{ o[0] + 4, o[1] + 1.2, o[2] }, .{ 0, 1, 0 }, .{ 1, 0, 0 });
    Desc.setSixDOFLimits(&door_desc, 0, -0.05, 0.05);
    Desc.setSixDOFLimits(&door_desc, 1, 0, 0);
    Desc.setSixDOFLimits(&door_desc, 2, 0, 0);
    Desc.setSixDOFLimits(&door_desc, 3, -1.5, 1.5);
    Desc.setSixDOFLimits(&door_desc, 4, 0, 0);
    Desc.setSixDOFLimits(&door_desc, 5, 0, 0);
    const door_joint = try b.constraint(door_desc);
    b.at(1);
    try b.constraintOp(.six_dof_set_motor_state, door_joint, .{ .i = &.{ 3, 2 } });
    try b.constraintOp(.six_dof_set_target_orientation_cs, door_joint, .{ .f = &axisAngle(.{ 1, 0, 0 }, 1.3) });
    b.at(40);
    try b.constraintOp(.six_dof_set_target_orientation_bs, door_joint, .{ .f = &axisAngle(.{ 0, 1, 0 }, -0.7) });
}

/// Random bodies (dynamic, kinematic, static; every basic shape) in a forest of random SixDOF joints (each body hangs
/// from the world or an earlier body; world or local space, random frames, both swing types, limits per axis free,
/// fixed (at zero or at a value), limited; soft translation limits in every spring mode, friction, motor settings), on
/// a floor in a container, and random operations between the updates: every SixDOF setter, every motor state, the
/// generic constraint operations and impulses
fn buildSixDOFRandom(comptime seed: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        /// `v` rotated by the unit quaternion `q` (x, y, z, w), in f64 (`inverse`: by its conjugate)
        fn rotate(q: [4]f32, v: [3]f64, inverse: bool) [3]f64 {
            const u: [3]f64 = if (inverse) .{ -@as(f64, q[0]), -@as(f64, q[1]), -@as(f64, q[2]) } else .{ q[0], q[1], q[2] };
            const s: f64 = q[3];
            const dot = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
            const uu = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
            const cross: [3]f64 = .{ u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0] };
            var r: [3]f64 = undefined;
            for (0..3) |k| r[k] = 2 * dot * u[k] + (s * s - uu) * v[k] + 2 * s * cross[k];
            return r;
        }

        fn toF32(v: [3]f64) [3]f32 {
            return .{ @floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]) };
        }

        /// The limits of axis `axis`: free, fixed (at zero or at a value), limited (cone swing: symmetric)
        fn limits(b: *SceneBuilder, axis: usize, pyramid: bool) [2]f32 {
            switch (b.index(5)) {
                0 => return .{ -flt_max, flt_max },
                1 => return .{ flt_max, -flt_max },
                2 => {
                    const v: f32 = if (b.oneIn(2)) 0 else if (axis < 3) b.float(-0.3, 0.3) else if (axis == 3 or pyramid) b.float(-0.5, 0.5) else 0;
                    return .{ v, v };
                },
                else => {
                    if (axis < 3) return .{ -b.float(0, 0.5), b.float(0, 0.5) };
                    if (axis == 3 or pyramid) return .{ -b.float(0.05, std.math.pi), b.float(0.05, std.math.pi) };
                    const v = b.float(0.05, std.math.pi);
                    return .{ -v, v };
                },
            }
        }

        fn motor(b: *SceneBuilder) [7]f32 {
            const mode: f32 = @floatFromInt(b.index(3));
            const frequency = if (b.oneIn(5)) 0 else if (mode == 0) b.float(0.5, 8) else b.float(5, 300);
            const force = if (b.oneIn(3)) flt_max else b.float(5, 200);
            const torque = if (b.oneIn(3)) flt_max else b.float(5, 200);
            return .{ mode, frequency, if (b.oneIn(4)) 0 else b.float(0, 2), -force, force, -torque, torque };
        }

        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.rng = 0x6d0f_5000 +% seed *% 0x9e3779b9;
            const shapes = try BasicShapes.add(b);
            b.num_updates = 80;
            b.max_contact_constraints = 2048;
            try addContainer(b, 6, 2);
            const num_bodies = 20;
            var bodies: [num_bodies]u32 = undefined;
            var positions: [num_bodies][3]f64 = undefined;
            var rotations: [num_bodies][4]f32 = undefined;
            for (&bodies, &positions, &rotations, 0..) |*h, *p, *q, i| {
                const motion_type: u32 = if (b.oneIn(8)) 0 else if (b.oneIn(7)) 1 else 2;
                const v = b.vec3(-3.5, 3.5);
                p.* = .{ v[0], 1.5 + 0.7 * @as(f64, @floatFromInt(i)), v[2] };
                q.* = b.rotation();
                h.* = try b.body(.{
                    .position = .{ real(p[0]), real(p[1]), real(p[2]) },
                    .rotation = q.*,
                    .shape = shapes.pick(b.index(6)),
                    .motion_type = motion_type,
                    .linear_velocity = if (motion_type == 1) b.vec3(-0.3, 0.3) else @splat(0),
                    .angular_velocity = if (motion_type != 0) b.vec3(-1, 1) else @splat(0),
                    .object_layer = if (motion_type == 0) Layers.non_moving else Layers.moving,
                }, if (motion_type == 0) .dont_activate else .activate);
            }

            // A forest of SixDOF joints: every body hangs from the world or from an earlier body, the joint between the
            // two bodies, the frame consistent with the poses of the creation (world space, or local space computed from
            // the poses), the frame of body 2 sometimes turned around the X axis (the twist starts away from zero)
            var handles: [num_bodies]u32 = undefined;
            var in_system: [num_bodies]bool = @splat(true);
            var pyramid: [num_bodies]bool = undefined;
            for (&handles, 0..) |*c, k| {
                const to_world = k == 0 or b.oneIn(4);
                const parent: usize = if (to_world) 0 else b.index(@intCast(k));
                const body1 = if (to_world) invalid_handle else bodies[parent];
                const offset = b.vec3(-0.6, 0.6);
                var point: [3]f64 = undefined;
                for (0..3) |n| point[n] = (if (to_world) positions[k][n] else 0.5 * (positions[k][n] + positions[parent][n])) + offset[n];
                const q = b.rotation();
                const axis_x = rotate(q, .{ 1, 0, 0 }, false);
                const axis_y1 = rotate(q, .{ 0, 1, 0 }, false);
                const angle: f64 = if (b.oneIn(2)) 0 else b.float(-0.5, 0.5);
                const axis_z = rotate(q, .{ 0, 0, 1 }, false);
                var axis_y2: [3]f64 = undefined;
                for (0..3) |n| axis_y2[n] = @cos(angle) * axis_y1[n] + @sin(angle) * axis_z[n];
                var desc = Desc.defaults(.six_dof);
                desc.body1 = body1;
                desc.body2 = bodies[k];
                pyramid[k] = b.oneIn(2);
                desc.i[1] = @intFromBool(pyramid[k]);
                if (b.oneIn(3)) {
                    // Local space: the position and the axes in the center of mass space of each body (the shapes have
                    // their center of mass at the origin, the world has no rotation)
                    desc.i[0] = ConstraintSpace.local_to_body_com;
                    var local1 = point;
                    var x1 = axis_x;
                    var y1 = axis_y1;
                    if (!to_world) {
                        var d: [3]f64 = undefined;
                        for (0..3) |n| d[n] = point[n] - positions[parent][n];
                        local1 = rotate(rotations[parent], d, true);
                        x1 = rotate(rotations[parent], axis_x, true);
                        y1 = rotate(rotations[parent], axis_y1, true);
                    }
                    var d2: [3]f64 = undefined;
                    for (0..3) |n| d2[n] = point[n] - positions[k][n];
                    const local2 = rotate(rotations[k], d2, true);
                    for (0..3) |n| {
                        desc.r[n] = real(local1[n]);
                        desc.r[3 + n] = real(local2[n]);
                    }
                    @memcpy(desc.f[0..3], &toF32(x1));
                    @memcpy(desc.f[3..6], &toF32(y1));
                    @memcpy(desc.f[6..9], &toF32(rotate(rotations[k], axis_x, true)));
                    @memcpy(desc.f[9..12], &toF32(rotate(rotations[k], axis_y2, true)));
                } else {
                    for (0..3) |n| {
                        desc.r[n] = real(point[n]);
                        desc.r[3 + n] = real(point[n]);
                    }
                    @memcpy(desc.f[0..3], &toF32(axis_x));
                    @memcpy(desc.f[3..6], &toF32(axis_y1));
                    @memcpy(desc.f[6..9], &toF32(axis_x));
                    @memcpy(desc.f[9..12], &toF32(axis_y2));
                }
                for (0..6) |axis| {
                    const l = limits(b, axis, pyramid[k]);
                    Desc.setSixDOFLimits(&desc, axis, l[0], l[1]);
                    if (b.oneIn(4)) desc.f[12 + axis] = b.float(0, 20);
                    const m = motor(b);
                    setSixDOFMotor(&desc, axis, @intFromFloat(m[0]), m[1..7].*);
                }
                for (0..3) |axis| {
                    if (b.oneIn(3)) setSixDOFSpring(&desc, axis, b.index(3), b.float(1, 50), b.float(0, 1));
                }
                desc.constraint_priority = b.index(3);
                if (b.oneIn(5)) desc.num_velocity_steps_override = 1 + b.index(15);
                if (b.oneIn(8)) desc.enabled = 0;
                c.* = try b.constraint(desc);
            }
            var update: u32 = 3;
            while (update < b.num_updates) : (update += 1 + b.index(6)) {
                b.at(update);
                for (0..1 + b.index(3)) |_| {
                    const k = b.index(num_bodies);
                    const c = handles[k];
                    switch (b.index(16)) {
                        0 => {
                            var f: [6]f32 = undefined;
                            for (0..3) |axis| {
                                const l = limits(b, axis, pyramid[k]);
                                f[axis] = l[0];
                                f[3 + axis] = l[1];
                            }
                            try b.constraintOp(.six_dof_set_translation_limits, c, .{ .f = &f });
                        },
                        1 => {
                            var f: [6]f32 = undefined;
                            for (0..3) |axis| {
                                const l = limits(b, 3 + axis, pyramid[k]);
                                f[axis] = l[0];
                                f[3 + axis] = l[1];
                            }
                            try b.constraintOp(.six_dof_set_rotation_limits, c, .{ .f = &f });
                        },
                        2 => try b.constraintOp(.six_dof_set_limits_spring_settings, c, .{ .i = &.{ b.index(3), b.index(3) }, .f = &.{ if (b.oneIn(3)) 0 else b.float(1, 50), b.float(0, 1) } }),
                        3 => try b.constraintOp(.six_dof_set_max_friction, c, .{ .i = &.{b.index(6)}, .f = &.{if (b.oneIn(3)) 0 else b.float(0, 20)} }),
                        4, 5, 6 => try b.constraintOp(.six_dof_set_motor_state, c, .{ .i = &.{ b.index(6), b.index(4) } }),
                        7 => try b.constraintOp(.six_dof_set_target_velocity_cs, c, .{ .f = &b.vec3(-2, 2) }),
                        8 => try b.constraintOp(.six_dof_set_target_angular_velocity_cs, c, .{ .f = &b.vec3(-2, 2) }),
                        9 => try b.constraintOp(.six_dof_set_target_position_cs, c, .{ .f = &b.vec3(-0.5, 0.5) }),
                        10 => try b.constraintOp(if (b.oneIn(2)) .six_dof_set_target_orientation_cs else .six_dof_set_target_orientation_bs, c, .{ .f = &b.rotation() }),
                        11 => {
                            const m = motor(b);
                            try b.constraintOp(.six_dof_set_motor_settings, c, .{ .i = &.{ b.index(6), @intFromFloat(m[0]) }, .f = m[1..7] });
                        },
                        12 => {
                            try b.constraintOp(if (in_system[k]) .remove_constraint else .add_constraint, c, .{});
                            in_system[k] = !in_system[k];
                        },
                        13 => try b.constraintOp(.set_constraint_enabled, c, .{ .i = &.{b.index(2)} }),
                        14 => try b.constraintOp(.reset_constraint_warm_start, c, .{}),
                        else => try b.op(.add_impulse, bodies[b.index(num_bodies)], .{ .f = &b.vec3(-3, 3) }),
                    }
                }
            }
        }
    }.build;
}

// (append the scenes of the six dof group here)

// ---- Path (Blocker.path_constraints, OpKind 340..359) ----

/// The points of a Hermite path on a circle of `radius` in the XZ plane (position, tangent, normal per point; a closed
/// loop when `num` points cover the circle), computed in f64 by the scene builder
fn circlePathPoints(comptime num: usize, radius: f64, arc: f64) [num][9]f32 {
    var points: [num][9]f32 = undefined;
    for (&points, 0..) |*p, k| {
        const a = arc * @as(f64, @floatFromInt(k)) / @as(f64, @floatFromInt(num));
        const step = arc / @as(f64, @floatFromInt(num));
        p.* = .{ @floatCast(radius * @cos(a)), 0, @floatCast(radius * @sin(a)), @floatCast(-radius * step * @sin(a)), 0, @floatCast(radius * step * @cos(a)), 0, 1, 0 };
    }
    return points;
}

/// Every operation of PathConstraint: a box that follows an open arc path from the world (rotation constrained around the
/// tangent) with friction and a position motor (velocity, then a target path fraction), a box on a looping circle on a
/// dynamic plate (rotation fully constrained); SetPath with another path and with no path (the constraint is then not
/// active), NotifyShapeChanged, ResetWarmStart
fn buildPathOps(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    const plate = try b.box(.{ 2.5, 0.2, 2.5 }, 0.05);
    b.num_updates = 80;
    const arc = circlePathPoints(6, 2, 0.75 * std.math.pi * 2.0);
    const a = try b.body(.{ .position = .{ 2, 5, 0 }, .shape = box, .gravity_factor = 0.5 }, .activate);
    var open = Desc.path(invalid_handle, a, &arc, false, .{ 0, 5, 0 }, .{ 0, 0, 0, 1 }, 0, 1); // ConstrainAroundTangent
    open.f[8] = 0.5; // Max friction force
    const follower = try b.constraint(open);
    const circle = circlePathPoints(8, 2, std.math.pi * 2.0);
    const base = try b.body(.{ .position = .{ 8, 3, 0 }, .shape = plate, .gravity_factor = 0 }, .activate);
    const rider = try b.body(.{ .position = .{ 10, 3.5, 0 }, .shape = box }, .activate);
    const loop_desc = try b.addConstraint(Desc.path(base, rider, &circle, true, .{ 0, 0.5, 0 }, axisAngle(.{ 1, 0, 0 }, 0.1), 0, 5)); // FullyConstrained
    const loop = try b.createConstraint(loop_desc);
    // A description that only provides another path for path_set_path
    const other_path = try b.addConstraint(Desc.path(invalid_handle, a, &circlePathPoints(5, 1.5, std.math.pi), false, .{ 0, 5, 0 }, .{ 0, 0, 0, 1 }, 0, 0));
    b.at(5);
    try b.constraintOp(.path_set_position_motor_settings, follower, .{ .i = &.{0}, .f = &.{ 3, 1, -50, 50, -1000, 1000 } });
    try b.constraintOp(.path_set_position_motor_state, follower, .{ .i = &.{1} });
    try b.constraintOp(.path_set_target_velocity, follower, .{ .f = &.{1} });
    try b.constraintOp(.path_set_position_motor_state, loop, .{ .i = &.{1} });
    try b.constraintOp(.path_set_target_velocity, loop, .{ .f = &.{-2} });
    b.at(25);
    try b.constraintOp(.path_set_position_motor_state, follower, .{ .i = &.{2} });
    try b.constraintOp(.path_set_target_path_fraction, follower, .{ .f = &.{3.5} });
    try b.constraintOp(.path_set_max_friction_force, loop, .{ .f = &.{1} });
    b.at(40);
    try b.constraintOp(.path_set_path, follower, .{ .i = &.{ 1, other_path }, .f = &.{1.5} });
    try b.constraintOp(.path_set_target_path_fraction, follower, .{ .f = &.{0.5} });
    try b.constraintOp(.notify_constraint_shape_changed, loop, .{ .body = rider, .f = &.{ 0, 0.05, 0 } });
    try b.constraintOp(.notify_constraint_shape_changed, loop, .{ .body = base, .f = &.{ 0.1, 0, 0 } });
    b.at(55);
    try b.constraintOp(.path_set_path, loop, .{ .i = &.{0}, .f = &.{0} }); // No path: not active
    try b.expectConstraintState(loop, ConstraintState.inactive);
    try b.constraintOp(.reset_constraint_warm_start, follower, .{});
    b.at(65);
    try b.constraintOp(.path_set_path, loop, .{ .i = &.{ 1, loop_desc }, .f = &.{2} }); // The circle again
    try b.constraintOp(.path_set_position_motor_state, follower, .{ .i = &.{0} });
}

// (append the scenes of the path group here)

// ---- Gear / RackAndPinion (Blocker.gear_constraints / .rack_and_pinion_constraints, OpKind 360..379) ----

/// Every operation of GearConstraint: two gears on hinges to the world, the first driven by its hinge motor, coupled by a
/// gear constraint with a ratio from SetRatio (and a second pair with an explicit ratio and no hinges at first);
/// SetConstraints, NotifyShapeChanged, ResetWarmStart, GetConstraintSettings
fn buildGearOps(b: *SceneBuilder) Allocator.Error!void {
    const wheel = try b.cylinder(0.1, 0.5, 0.05);
    const small_wheel = try b.cylinder(0.1, 0.25, 0.05);
    b.num_updates = 60;
    const lying = axisAngle(.{ 1, 0, 0 }, 0.5 * std.math.pi); // The cylinder axis (Y) along Z
    const g1 = try b.body(.{ .position = .{ 0, 4, 0 }, .rotation = lying, .shape = wheel }, .activate);
    const g2 = try b.body(.{ .position = .{ 0.75, 4, 0 }, .rotation = lying, .shape = small_wheel }, .activate);
    var h1_desc = Desc.hinge(invalid_handle, g1, .{ 0, 4, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 });
    Desc.setMotor(&h1_desc, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -100, 100 });
    const h1 = try b.constraint(h1_desc);
    const h2 = try b.constraint(Desc.hinge(invalid_handle, g2, .{ 0.75, 4, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 }));
    var gear = Desc.gear(g1, g2, .{ 0, 0, 1 }, .{ 0, 0, 1 }, 1, h1, h2);
    gear.i[1] = 1; // SetRatio(20 teeth, 10 teeth)
    gear.i[2] = 20;
    gear.i[3] = 10;
    const g12 = try b.constraint(gear);
    const g3 = try b.body(.{ .position = .{ 3, 4, 0 }, .rotation = lying, .shape = wheel }, .activate);
    const g4 = try b.body(.{ .position = .{ 4, 4, 0 }, .rotation = lying, .shape = wheel, .angular_velocity = .{ 0, 0, 3 } }, .activate);
    const h3 = try b.constraint(Desc.hinge(invalid_handle, g3, .{ 3, 4, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 }));
    const h4 = try b.constraint(Desc.hinge(invalid_handle, g4, .{ 4, 4, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 }));
    const g34 = try b.constraint(Desc.gear(g3, g4, .{ 0, 0, 1 }, .{ 0, 0, 1 }, 1.5, invalid_handle, invalid_handle));
    b.at(5);
    try b.constraintOp(.hinge_set_motor_state, h1, .{ .i = &.{1} });
    try b.constraintOp(.hinge_set_target_angular_velocity, h1, .{ .f = &.{3} });
    b.at(20);
    try b.constraintOp(.gear_set_constraints, g34, .{ .i = &.{ h3, h4 } });
    try b.constraintOp(.constraint_settings_hash, g12, .{});
    b.at(35);
    try b.constraintOp(.gear_set_constraints, g12, .{ .i = &.{ h1, invalid_handle } });
    try b.constraintOp(.notify_constraint_shape_changed, g34, .{ .body = g3, .f = &.{ 0.1, 0, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, g12, .{});
    b.at(45);
    try b.constraintOp(.gear_set_constraints, g12, .{ .i = &.{ h1, h2 } });
    try b.constraintOp(.constraint_settings_hash, g34, .{});
}

/// Every operation of RackAndPinionConstraint: a pinion on a hinge driven by its motor and a rack on a slider, coupled with
/// a ratio from SetRatio; a second pair with an explicit ratio that gets its hinge and slider later (SetConstraints)
fn buildRackAndPinionOps(b: *SceneBuilder) Allocator.Error!void {
    const wheel = try b.cylinder(0.1, 0.5, 0.05);
    const rack_shape = try b.box(.{ 2, 0.1, 0.1 }, 0.05);
    b.num_updates = 60;
    const lying = axisAngle(.{ 1, 0, 0 }, 0.5 * std.math.pi);
    const pinion_body = try b.body(.{ .position = .{ 0, 4, 0 }, .rotation = lying, .shape = wheel }, .activate);
    const rack_body = try b.body(.{ .position = .{ 0, 3.4, 0 }, .shape = rack_shape, .gravity_factor = 0 }, .activate);
    var hinge = Desc.hinge(invalid_handle, pinion_body, .{ 0, 4, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 });
    Desc.setMotor(&hinge, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -100, 100 });
    const pinion = try b.constraint(hinge);
    var slider = Desc.slider(invalid_handle, rack_body, .{ 1, 0, 0 });
    slider.f[12] = -1.5;
    slider.f[13] = 1.5;
    const rack = try b.constraint(slider);
    var rp = Desc.rackAndPinion(pinion_body, rack_body, .{ 0, 0, 1 }, .{ 1, 0, 0 }, 1, pinion, rack);
    rp.i[1] = 1; // SetRatio(30 teeth on the rack, rack length 4, 10 teeth on the pinion)
    rp.i[2] = 30;
    rp.i[3] = 10;
    rp.f[7] = 4;
    const rack_and_pinion = try b.constraint(rp);
    const pinion_body2 = try b.body(.{ .position = .{ 0, 8, 0 }, .rotation = lying, .shape = wheel, .angular_velocity = .{ 0, 0, -2 } }, .activate);
    const rack_body2 = try b.body(.{ .position = .{ 0, 7.4, 0 }, .shape = rack_shape, .gravity_factor = 0 }, .activate);
    const pinion2 = try b.constraint(Desc.hinge(invalid_handle, pinion_body2, .{ 0, 8, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 }));
    const rack2 = try b.constraint(Desc.slider(invalid_handle, rack_body2, .{ 1, 0, 0 }));
    const rack_and_pinion2 = try b.constraint(Desc.rackAndPinion(pinion_body2, rack_body2, .{ 0, 0, 1 }, .{ 1, 0, 0 }, 2, invalid_handle, invalid_handle));
    b.at(5);
    try b.constraintOp(.hinge_set_motor_state, pinion, .{ .i = &.{1} });
    try b.constraintOp(.hinge_set_target_angular_velocity, pinion, .{ .f = &.{1} });
    b.at(20);
    try b.constraintOp(.rack_and_pinion_set_constraints, rack_and_pinion2, .{ .i = &.{ pinion2, rack2 } });
    try b.constraintOp(.constraint_settings_hash, rack_and_pinion, .{});
    b.at(35);
    try b.constraintOp(.notify_constraint_shape_changed, rack_and_pinion, .{ .body = rack_body, .f = &.{ 0.1, 0, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, rack_and_pinion2, .{});
    try b.constraintOp(.rack_and_pinion_set_constraints, rack_and_pinion, .{ .i = &.{ invalid_handle, rack } });
    b.at(45);
    try b.constraintOp(.rack_and_pinion_set_constraints, rack_and_pinion, .{ .i = &.{ pinion, rack } });
    try b.constraintOp(.constraint_settings_hash, rack_and_pinion2, .{});
}

// (append the scenes of the gear / rack and pinion group here)

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
