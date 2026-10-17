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
//!   A Gear / RackAndPinion must only reference (Create or SetConstraints) hinges / sliders whose bodies are in its own
//!   island: SolvePositionConstraint reads the bodies of the referenced constraints (HingeConstraint::GetCurrentAngle,
//!   SliderConstraint::GetCurrentPosition), and the islands are solved by parallel jobs, so a read across islands
//!   depends on the thread timing, in Jolt too.
//!
//! # Characters, ragdolls and vehicles (Phase 7)
//!
//! Character, CharacterVirtual (with CharacterVsCharacterCollisionSimple and the CharacterContactListener), Ragdoll and
//! VehicleConstraint (wheeled, tracked and motorcycle controllers, the ray / cast sphere / cast cylinder collision
//! testers) can be described, created and driven by every public setter and action on both sides; the C++ side
//! (SimulationReference.cpp) covers all of them since the Phase 7 foundation, the Zolt side (SimulationParity.zig) the
//! classes that are ported (one slot per class group).
//! - **Descriptions.** `SimCharacter`, `SimCharacterVirtual`, `SimRagdoll` (+ `SimRagdollJoint`,
//!   `SimRagdollAdditionalConstraint`, `SimRagdollPose`) and `SimVehicle` (+ `SimVehicleWheel`, `SimVehicleDifferential`,
//!   `SimVehicleAntiRollBar`, the engine, transmission, tracks and tester) hold every member of the settings classes,
//!   with Jolt's defaults (test "SimScenes Phase 7 description defaults are Jolt's"; `SimVehicle.defaults(kind)` /
//!   `SimVehicleWheel.defaults(kind)` for the other controller and wheel classes). Curves, gear ratios and pose joint
//!   states live in the float pool (`addFloats`, `addRagdollPose`), the track wheel indices in the integer pool
//!   (`addUints`). A ragdoll's parts are body descriptions, its joints constraint descriptions (their body handles are not
//!   used). The character virtual IDs are explicit (`SceneBuilder.characterVirtual` numbers them 1, 2, ...): CharacterID's
//!   counter is global in both libraries.
//! - **Creation and handles.** `b.character(desc)`, `b.characterVirtual(desc)`, `b.createRagdoll(addRagdoll(...))` and
//!   `b.createVehicle(addVehicle(...))` create an object of the next handle of its class (character, character virtual,
//!   ragdoll, vehicle handles count separately). A Character starts outside the system (character_add_to_physics_system);
//!   a CharacterVirtual gets its listener (`SimCharacterVirtual.listener`) and collision group from the description; a
//!   vehicle is added to the system with its step listener.
//! - **Operations.** `b.objectOp(kind, handle, .{ .i, .f, .r, .body, .count, .activation })` (OpKind 400..599, one
//!   `Phase7OpRange` per class group: the vehicle core group and the two vehicle variants have ranges of their own),
//!   `b.characterVirtualQuery` / `b.extendedUpdate` for the CharacterVirtual queries with their filters
//!   (`CharacterQuery`). The operations run between the updates on the user thread, the character virtual queries with
//!   the world's TempAllocator, like PerformanceTest and the unit tests call them; `i[3] != 0` selects the no-lock
//!   interfaces where Jolt has `inLockBodies`. `b.expectCharacterGroundState`, `expectCharacterVirtualGroundState`,
//!   `expectRagdollActive`, `expectWheelContacts` and `expectVehicleConstraintState` state what a scene reaches at that
//!   point.
//! - **What is compared.** Before a scene runs: the binary state of the vehicle descriptions' settings (every
//!   `VehicleSettingsPart`) and of the ragdoll descriptions' RagdollSettings (with the shapes and the group filter), and
//!   their round trips: each side restores the other side's bytes and saves again (RagdollSettings::
//!   sRestoreFromBinaryState, the vehicle settings without wheels, the controller, wheel, engine, transmission, track,
//!   differential and anti-roll bar settings; on the C++ side alone while the classes are stubs); the tables of the
//!   RagdollSettings (GetBodyIndexToConstraintIndex, GetConstraintIndexToBodyIdxPair, the result of Stabilize). After
//!   every update, besides the PhysicsSystem state (which holds the ragdolls' bodies and constraints and the
//!   VehicleConstraint state) and the sorted records (with the vehicle callbacks, RecordKind 18..20): the object state
//!   (`ObjectField`: the SaveState bytes of every character and character virtual, and the getters that the states do
//!   not hold: the active contacts, the ground material and user data, CanWalkStairs, the transforms and transformed
//!   shapes, a ragdoll's pose, root transform, bounds and constraints, the vehicle's Constraint interface, every wheel's
//!   contact and transforms, the engine, transmission, differentials, tracks, lean parameters), decoded to field, object
//!   handle, item and component on a mismatch; and the CharacterContactListener records (RecordKind 7..17) in call
//!   order. `snapshot` / `restore_cross` include the SaveState of every character and character virtual (every byte must
//!   be read). The test "SimulationParity CharacterID consumption" compares which constructions consume a CharacterID
//!   (`characterIdSequence`, M8 depends on it).
//! - **Listener modes.** `CharacterListenerMode` record / modify: in modify mode the `CharacterFlags` in the upper 32
//!   bits of the user data of a body, another character or the character itself reject contacts, stop pushing, stop
//!   impulses, make a conveyor belt and slow the solved velocity (the same code on both sides). The vehicle callbacks
//!   (`VehicleCallbacks`, with a steering pre-step callback), `VehicleCombineFriction` and `VehicleTireMaxImpulse` are
//!   functions of the description or of an operation.
//! - **Blockers.** `.characters`, `.character_virtuals`, `.ragdolls`, `.wheeled_vehicles`, `.tracked_vehicles`,
//!   `.motorcycles`: supported as soon as the Zolt classes of the group are ported (`Phase7Classes`: the settings and
//!   testers no longer declare `is_stub`). Until then the scene runs on the C++ side alone ("SimScenes blocked scenes run
//!   on the C++ side": two C++ worlds with 0 and 3 threads in lockstep with their object states and character records,
//!   its expectations, finite and completely decoded object states, the settings round trips, no assert of an asserting
//!   reference). A supported scene whose classes are still stubs, or whose slot in SimulationParity.zig is still
//!   `Phase7StubSlot`, fails (checkPhase7Slots).
//! - **Rules.** Avoid what Jolt asserts or leaves undefined:
//!   - Wheel leaves its contact members (mContactPosition, mContactPointVelocity, mContactNormal, mContactLongitudinal,
//!     mContactLateral, mAxlePlaneConstant) uninitialized until the first contact, and VehicleConstraint::SaveState
//!     writes some of them: the reference zero-fills every allocation (SimInstallHooks of SimulationReference.cpp) and
//!     Zolt zero-initializes these members (a documented deviation), so a vehicle may start in the air
//!     (vehicle_wheeled_in_air);
//!   - the body of a vehicle stays in the system while the vehicle exists, vehicles do not drive on each other (their
//!     step listeners run in parallel);
//!   - a vehicle needs at least one wheel and normalized axes, a wheeled controller a differential when the object state
//!     reads GetWheelSpeedAtClutch (0 / 0 otherwise), the tracked ratios are never 0; the wheel indices of the anti-roll
//!     bars, the differentials and the tracks are valid (-1 where Jolt allows it), also after vehicle_set_anti_roll_bar,
//!     wheeled_set_differential and tracked_set_track; the current gear stays within the ratio arrays after
//!     vehicle_transmission_set_gear_ratios and vehicle_transmission_set;
//!   - a motorcycle keeps mLeanSpringIntegrationCoefficient at 0 in scenes with sweeps or restores (Jolt's
//!     MotorcycleController::SaveState does not save mLeanSpringIntegratedDeltaAngle; vehicle_motorcycle_lean_integration
//!     covers the coefficient without sweeps and restores);
//!   - the Zolt port of the custom friction callback (SimVehicleCombineFriction) uses the Math min helper, not @min;
//!   - a Character has a shape, is removed before it is destroyed; a CharacterVirtual's shape is convex, decorated or
//!     compound (sCorrectFractionForCharacterPadding asserts on other shapes), its listener does not change between
//!     StartTrackingContactChanges and FinishTrackingContactChanges, its inner body is never removed by a body operation;
//!   - a ragdoll's joints list the parents before the children, its motors drive SwingTwist and Hinge constraints only
//!     (DriveToPoseUsingMotors asserts on other classes), it is removed before it is destroyed;
//!   - no character, character virtual, ragdoll or vehicle is created or destroyed between a snapshot and its
//!     restore_cross (the restore reads the objects of the snapshot);
//!   - the object layers of the tester and the queries are layers of `Layers` (the layer filters assert on others).
//!   Check new scenes against Jolt's asserts: `zig build parity -Dreference_asserts=true -Dtest-filter="blocked scenes
//!   run on the C++ side"` (the reference library with JPH_ENABLE_ASSERTS: a failed assert is printed and counted, the
//!   simulation parity fails a scene whose C++ side hit one; the Phase 7 scenes hit none).
//!
//! ## Recipe for a Phase 7 worker (adding a class group to the simulation parity)
//!
//! 1. Port the classes of the group (CharacterBase.zig, the foundation's port, shows the pattern A layout of a
//!    character); removing `is_stub` from the settings (and testers) makes the scenes with the group's Blocker run instead
//!    of being skipped, and the test "SimulationParity Phase 7 slots follow the ported classes" fails until step 2.
//! 2. SimulationParity.zig: replace `const XSlot = Phase7StubSlot;` of the group with a struct (the interface of each
//!    slot is documented above Phase7StubSlot): `Storage` (the objects of the handles), `create` and `applyOp` /
//!    `applyVehicleOp` (exactly ApplyCharacterOp / ApplyCharacterVirtualOp / ApplyRagdollOp / ApplyVehicleOp /
//!    ApplyTrackedOp / ApplyMotorcycleOp of SimulationReference.cpp, with the same results), `writeState` (the fields of
//!    `objectFieldLayout` in the order of WriteObjectState of the .cpp), `saveStates` / `restoreStates` (characters),
//!    `destroyAll`, and the hooks of the slot (the CharacterID test, the settings round trips, the vehicle variants'
//!    controller settings, wheel settings and object state parts). CharacterVirtualSlot ports
//!    SimCharacterContactListener (record / modify, `CharacterRecords` builds the records), the vehicle slots the
//!    callbacks of SetVehicleCallbacks, SimVehicleCombineFriction and SimVehicleTireMaxImpulse (`VehicleRecords`).
//! 3. Run `zig build parity -Dtest-filter=<scene>`: the C++ scenes of the group (`character_ops`,
//!    `character_virtual_ops`, `character_virtual_characters`, `ragdoll_ops`, `vehicle_wheeled_ops`,
//!    `vehicle_wheeled_in_air`, `vehicle_tracked_ops`, `vehicle_motorcycle_ops`, `vehicle_motorcycle_lean_integration`)
//!    now run against Zolt, bit exact in the whole thread matrix with the D12 sweeps; a mismatch names the field of the
//!    object state ("object state differs at byte ...: cv_active_contact (object handle 0, item 2) contact_normal.y zolt
//!    ... jolt ...") or the first differing character record.
//! 4. Add scenes of the group in its regions (builders in "---- <group> ----" below, entries in the scene table):
//!    stairs and slopes, moving platforms, many characters, ragdolls in piles and LargeIslandSplitter islands, vehicles
//!    on meshes and height fields, cross save / restore, far from the origin. The CharacterVirtual group also does
//!    milestone M8 (ZoltPerformanceTest's CharacterVirtual scene, a `perf_character_virtual` scene here, the CI steps).
//!    A new operation of the group goes into its range (`OpKind`, ApplyObjectOp of the .cpp, the slot's `applyOp`), a
//!    new object state field gets a new `ObjectField` value with its `objectFieldLayout`, a new record kind a value in
//!    the group's RecordKind block.
//! 5. Prove the scenes bite (break one operation of the class, a scene must fail), check new scenes against an asserting
//!    reference (see the rules above) and run the parity in Debug, ReleaseFast and double precision.
//!
//! ## Reserved numbers
//!
//! OpKind: 0..52 body and system operations, 53..99 reserved for operations of later phases that are not constraint
//! operations, 100..102 driver operations, 200..399 constraint operations in ranges of 20 (`ConstraintOpRange`):
//! 200..219 generic, 220..239 Fixed / Point, 240..259 Distance / Pulley, 260..279 Hinge, 280..299 Slider, 300..319
//! Cone / SwingTwist, 320..339 SixDOF, 340..359 Path, 360..379 Gear / RackAndPinion, 380..399 spare; 400..599 the
//! Phase 7 operations (`Phase7OpRange`): 400..429 Character (423..429 free), 430..489 CharacterVirtual (471..489 free),
//! 490..519 Ragdoll (511..519 free), 520..569 the vehicle core group (563..569 free), 570..579 TrackedVehicleController
//! (576..579 free), 580..589 MotorcycleController (587..589 free), 590..599 spare. RecordKind: 0..6 bodies, steps and
//! contacts, 7..17 the CharacterContactListener (call order), 18..20 the vehicle callbacks; blocks for new kinds: 21..29
//! Character, 30..39 CharacterVirtual (call order), 40..49 Ragdoll, 50..59 vehicle core, 60..69 vehicle variants, 70..79
//! later phases (`num_record_kinds` = 80). ObjectField: 1..9 Character, 10..19 CharacterVirtual, 20..29 Ragdoll, 30..49
//! vehicles, new fields in the group's block. ConstraintKind: 0..11 (the order of the Phase 6 plan, not
//! EConstraintSubType: `subType()` maps them), new kinds from 12. Blocker: one value per kind or class group. `Expect`
//! fields are named, not numbered. The scene table and the builders have one region per worker group, marked "(append
//! the ... scenes here)".
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

// -------------------------------------------------------------------------------------------------------------------
// Characters, ragdolls and vehicles (Phase 7, see the file header): the descriptions (Must match SimulationReference.cpp,
// checked by the test "SimScenes struct layout"), the operation ranges and the listener modes

/// The members of CharacterBaseSettings (Jolt's defaults), part of SimCharacter and SimCharacterVirtual
pub const SimCharacterBase = extern struct {
    up: [3]f32 = .{ 0, 1, 0 },
    /// mSupportingVolume: normal xyz, constant
    supporting_volume: [4]f32 = .{ 0, 1, 0, -1.0e10 },
    max_slope_angle: f32 = zolt.math.degreesToRadians(50.0),
    enhanced_internal_edge_removal: u32 = 0,
    /// mShape: shape index (`invalid_handle`: none)
    shape: u32 = invalid_handle,
};

/// A Character (OpKind.create_character): CharacterSettings (Jolt's defaults) and the constructor arguments
pub const SimCharacter = extern struct {
    base: SimCharacterBase = .{},
    layer: u32 = 0,
    mass: f32 = 80.0,
    friction: f32 = 0.2,
    gravity_factor: f32 = 1.0,
    /// EAllowedDOFs bits (Jolt's default: TranslationX | TranslationY | TranslationZ)
    allowed_dofs: u32 = 0b000111,
    /// Character(settings, position, rotation, user data, system)
    position: [3]Real = @splat(0),
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
    user_data: u64 = 0,
};

/// A CharacterVirtual (OpKind.create_character_virtual): CharacterVirtualSettings (Jolt's defaults), the constructor
/// arguments and the harness' setup of the new character
pub const SimCharacterVirtual = extern struct {
    base: SimCharacterBase = .{},
    /// mID: the CharacterID value. Set explicitly: CharacterID's counter is global in both libraries and shared by the
    /// two C++ worlds of a run. `SceneBuilder.characterVirtual` replaces `invalid_handle` with the next ID of the scene
    /// (1, 2, ... like PerformanceTest after CharacterID::sSetNextCharacterID()).
    id: u32 = invalid_handle,
    mass: f32 = 70.0,
    max_strength: f32 = 100.0,
    shape_offset: [3]f32 = @splat(0),
    /// EBackFaceMode: 0 IgnoreBackFaces, 1 CollideWithBackFaces
    back_face_mode: u32 = 1,
    predictive_contact_distance: f32 = 0.1,
    max_collision_iterations: u32 = 5,
    max_constraint_iterations: u32 = 15,
    min_time_remaining: f32 = 1.0e-4,
    collision_tolerance: f32 = 1.0e-3,
    character_padding: f32 = 0.02,
    max_num_hits: u32 = 256,
    hit_reduction_cos_max_angle: f32 = 0.999,
    penetration_recovery_speed: f32 = 1.0,
    /// mInnerBodyShape: shape index (`invalid_handle`: no inner body)
    inner_body_shape: u32 = invalid_handle,
    /// mInnerBodyIDOverride: BodyID value (0xffffffff: none, CreateBody chooses the ID)
    inner_body_id_override: u32 = 0xffffffff,
    inner_body_layer: u32 = 0,
    /// CharacterVirtual(settings, position, rotation, user data, system)
    position: [3]Real = @splat(0),
    rotation: [4]f32 = .{ 0, 0, 0, 1 },
    user_data: u64 = 0,
    /// Harness: SetListener after the creation (CharacterListenerMode)
    listener: u32 = @intFromEnum(CharacterListenerMode.record),
    /// Harness: the CharacterVsCharacterCollisionSimple (index < `num_character_collision_groups`) that the new
    /// character is added to (Add) and uses (SetCharacterVsCharacterCollision); `invalid_handle`: none
    collision_group: u32 = invalid_handle,
};

/// The number of CharacterVsCharacterCollisionSimple objects of a world (the `collision_group` indices)
pub const num_character_collision_groups = 4;

/// What the CharacterContactListener of a CharacterVirtual does (`SimCharacterVirtual.listener`,
/// OpKind.cv_set_listener). Every callback is recorded (RecordKind 7..17, compared in call order).
pub const CharacterListenerMode = enum(u32) {
    /// SetListener(nullptr)
    none = 0,
    /// Record the callbacks, keep Jolt's defaults (validate returns true, the settings are not changed)
    record = 1,
    /// Record the callbacks (the arguments as Jolt passes them in), then act on the `CharacterFlags`
    modify = 2,
};

/// Bits in the upper 32 bits of a user data value that the character listener in `CharacterListenerMode.modify` acts
/// on: the user data of the body or of the other character of a contact (CharacterContact::mUserData), of the body in
/// OnAdjustBodyVelocity, or of the character itself in the solve callbacks. The bits do not overlap `ContactFlags`.
pub const CharacterFlags = struct {
    /// OnContactValidate / OnCharacterContactValidate return false
    pub const reject: u32 = 1 << 16;
    /// OnContactAdded / OnContactPersisted (and the character versions): mCanPushCharacter = false
    pub const cannot_push: u32 = 1 << 17;
    /// OnContactAdded / OnContactPersisted (and the character versions): mCanReceiveImpulses = false
    pub const no_impulses: u32 = 1 << 18;
    /// OnAdjustBodyVelocity: linear velocity += (1.5, 0, 0), angular velocity += (0, 0.5, 0) (a conveyor belt)
    pub const conveyor: u32 = 1 << 19;
    /// OnContactSolve / OnCharacterContactSolve (the character's own user data): the new character velocity * 0.5
    pub const slow_solve: u32 = 1 << 20;

    /// The user data of a body / character with these flags (and `low` in the lower 32 bits)
    pub fn userData(flags: u32, low: u32) u64 {
        return (@as(u64, flags) << 32) | low;
    }
};

/// The filters of the CharacterVirtual queries (Update, ExtendedUpdate, StickToFloor, WalkStairs, RefreshContacts,
/// SetShape, CheckCollision): op.i[0] the object layer of PhysicsSystem::GetDefaultBroadPhaseLayerFilter /
/// GetDefaultLayerFilter (`no_layer_filter`: the accept-all BroadPhaseLayerFilter / ObjectLayerFilter), op.i[1] the
/// CharacterBodyFilter, op.i[2] the CharacterShapeFilter, op.body the body handle of CharacterBodyFilter.ignore_body
pub const CharacterQueryFilter = struct {
    pub const no_layer_filter: u32 = invalid_handle;
};

/// The BodyFilter of a CharacterVirtual query (op.i[1])
pub const CharacterBodyFilter = enum(u32) {
    /// BodyFilter (accepts every body)
    all = 0,
    /// IgnoreSingleBodyFilter(body handle op.body)
    ignore_body = 1,
    /// IgnoreSingleBodyFilter(GetInnerBodyID())
    ignore_inner_body = 2,
};

/// The ShapeFilter of a CharacterVirtual query (op.i[2])
pub const CharacterShapeFilter = enum(u32) {
    /// ShapeFilter (accepts every shape pair)
    all = 0,
    /// ShouldCollide is false when shape 2 is a SphereShape
    no_spheres = 1,
};

/// The value of a character virtual that OpKind.cv_expect_value checks (i[0])
pub const CharacterVirtualValue = enum(u32) {
    /// GetMaxHitsExceeded() (1 / 0)
    max_hits_exceeded = 0,
    /// The number of GetActiveContacts()
    num_active_contacts = 1,
    /// The number of active contacts with mHadCollision
    num_colliding_contacts = 2,
    /// The number of active contacts with mHadCollision and another character (mCharacterB != nullptr)
    num_colliding_character_contacts = 3,
    /// GetGroundBodyID() is the body of body handle op.body (1 / 0)
    ground_body_is = 4,
    /// GetGroundVelocity() is not zero (!IsNearZero(1.0e-6f): 1 / 0)
    ground_velocity_not_zero = 5,
    /// The shape index of GetShape()
    shape = 6,
};

/// A ragdoll (OpKind.create_ragdoll). The RagdollSettings of a description is built at the first create_ragdoll of
/// the description and shared by the ragdolls created from it (Jolt's usage): a Skeleton with one joint per
/// SimRagdollJoint (Skeleton::AddJoint("Joint<index>", parent), parents before children), one Part per joint (its
/// BodyCreationSettings from `bodies`, mToParent from `constraints`), the additional constraints, then the preparation
/// below, then CalculateBodyIndexToConstraintIndex and CalculateConstraintIndexToBodyIdxPair.
pub const SimRagdoll = extern struct {
    /// The joints: `num_joints` at ragdoll_joints[first_joint..]
    first_joint: u32 = 0,
    num_joints: u32 = 0,
    /// RagdollSettings::mAdditionalConstraints: `num_additional` at ragdoll_additional_constraints[first_additional..]
    first_additional: u32 = 0,
    num_additional: u32 = 0,
    /// RagdollSettings::Stabilize
    stabilize: u32 = 0,
    /// RagdollSettings::CalculateConstraintPriorities(base_priority) when != 0
    calculate_constraint_priorities: u32 = 0,
    base_priority: u32 = 0,
    /// RagdollSettings::DisableParentChildCollisions: 0 not called, 1 without joint matrices, 2 with the joint matrices
    /// of ragdoll_poses[pose] (SkeletonPose::CalculateJointMatrices) and `min_separation_distance`
    disable_parent_child_collisions: u32 = 0,
    pose: u32 = 0,
    min_separation_distance: f32 = 0.0,
    /// CreateRagdoll(collision group, user data, system)
    collision_group: u32 = 0,
    user_data: u64 = 0,
};

/// A joint of a ragdoll: the skeleton joint and the part attached to it
pub const SimRagdollJoint = extern struct {
    /// The parent joint index (-1: a root joint)
    parent: i32 = -1,
    /// The part's BodyCreationSettings: index of a body description (`bodies`)
    body: u32,
    /// mToParent: index of a constraint description (`constraints`, its body handles are not used: CreateRagdoll
    /// connects the parent part and this part); `invalid_handle`: none (the root)
    to_parent: u32 = invalid_handle,
};

/// RagdollSettings::AdditionalConstraint
pub const SimRagdollAdditionalConstraint = extern struct {
    /// mBodyIdx: the indices of the two parts
    body1: i32,
    body2: i32,
    /// mConstraint: index of a constraint description (`constraints`)
    constraint: u32,
};

/// A pose of a ragdoll's skeleton (the ragdoll operations that take a pose)
pub const SimRagdollPose = extern struct {
    /// SkeletonPose::SetRootOffset
    root_offset: [3]Real = @splat(0),
    /// `count` joint states at floats[data..][0 .. 7 * count]: translation xyz, rotation xyzw. The SkeletonPose
    /// operations use them as local joint states (CalculateJointMatrices), the `_matrices` operations as model space
    /// joint matrices (Mat44::sRotationTranslation(rotation, translation))
    data: u32 = 0,
    count: u32 = 0,
};

/// The controller of a vehicle (`SimVehicle.controller`)
pub const VehicleControllerKind = enum(u32) {
    /// WheeledVehicleControllerSettings (wheels of kind wv)
    wheeled = 0,
    /// TrackedVehicleControllerSettings (wheels of kind tv)
    tracked = 1,
    /// MotorcycleControllerSettings (a WheeledVehicleControllerSettings, wheels of kind wv)
    motorcycle = 2,
};

/// The wheel settings class of a vehicle wheel (`SimVehicleWheel.kind`)
pub const VehicleWheelKind = enum(u32) {
    /// WheelSettingsWV
    wv = 0,
    /// WheelSettingsTV
    tv = 1,
};

/// The VehicleCollisionTester class of a vehicle (`SimVehicleTester.kind`, OpKind.vehicle_set_collision_tester)
pub const VehicleTesterKind = enum(u32) {
    /// VehicleCollisionTesterRay(layer, up, max slope angle)
    ray = 0,
    /// VehicleCollisionTesterCastSphere(layer, radius, up, max slope angle)
    cast_sphere = 1,
    /// VehicleCollisionTesterCastCylinder(layer, convex radius fraction)
    cast_cylinder = 2,
};

/// VehicleEngineSettings (Jolt's defaults)
pub const SimVehicleEngine = extern struct {
    max_torque: f32 = 500.0,
    min_rpm: f32 = 1000.0,
    max_rpm: f32 = 6000.0,
    /// mNormalizedTorque: `num_torque_points` (x, y) points at floats[torque_points..] (`invalid_handle`: the curve of
    /// the constructor)
    torque_points: u32 = invalid_handle,
    num_torque_points: u32 = 0,
    inertia: f32 = 0.5,
    angular_damping: f32 = 0.2,
};

/// VehicleTransmissionSettings (Jolt's defaults)
pub const SimVehicleTransmission = extern struct {
    /// ETransmissionMode: 0 Auto, 1 Manual
    mode: u32 = 0,
    /// mGearRatios: `num_gear_ratios` at floats[gear_ratios..] (`invalid_handle`: the ratios of the controller
    /// settings' constructor)
    gear_ratios: u32 = invalid_handle,
    num_gear_ratios: u32 = 0,
    /// mReverseGearRatios, like `gear_ratios`
    reverse_gear_ratios: u32 = invalid_handle,
    num_reverse_gear_ratios: u32 = 0,
    switch_time: f32 = 0.5,
    clutch_release_time: f32 = 0.3,
    switch_latency: f32 = 0.5,
    shift_up_rpm: f32 = 4000.0,
    shift_down_rpm: f32 = 2000.0,
    clutch_strength: f32 = 10.0,
};

/// VehicleTrackSettings (Jolt's defaults; mDrivenWheel has none in Jolt)
pub const SimVehicleTrack = extern struct {
    driven_wheel: u32 = 0,
    /// mWheels: `num_wheels` wheel indices at uints[wheels..]
    wheels: u32 = 0,
    num_wheels: u32 = 0,
    inertia: f32 = 10.0,
    angular_damping: f32 = 0.5,
    max_brake_torque: f32 = 15000.0,
    differential_ratio: f32 = 6.0,
};

/// VehicleDifferentialSettings (Jolt's defaults)
pub const SimVehicleDifferential = extern struct {
    left_wheel: i32 = -1,
    right_wheel: i32 = -1,
    differential_ratio: f32 = 3.42,
    left_right_split: f32 = 0.5,
    limited_slip_ratio: f32 = 1.4,
    engine_torque_ratio: f32 = 1.0,
};

/// VehicleAntiRollBar (Jolt's defaults)
pub const SimVehicleAntiRollBar = extern struct {
    left_wheel: i32 = 0,
    right_wheel: i32 = 1,
    stiffness: f32 = 1000.0,
};

/// A wheel: WheelSettings plus the members of WheelSettingsWV or WheelSettingsTV (`kind`), Jolt's defaults
pub const SimVehicleWheel = extern struct {
    /// VehicleWheelKind
    kind: u32 = @intFromEnum(VehicleWheelKind.wv),
    position: [3]f32 = @splat(0),
    suspension_force_point: [3]f32 = @splat(0),
    suspension_direction: [3]f32 = .{ 0, -1, 0 },
    steering_axis: [3]f32 = .{ 0, 1, 0 },
    wheel_up: [3]f32 = .{ 0, 1, 0 },
    wheel_forward: [3]f32 = .{ 0, 0, 1 },
    suspension_min_length: f32 = 0.3,
    suspension_max_length: f32 = 0.5,
    suspension_preload_length: f32 = 0.0,
    /// mSuspensionSpring: ESpringMode, frequency / stiffness, damping
    suspension_spring_mode: u32 = 0,
    suspension_frequency: f32 = 1.5,
    suspension_damping: f32 = 0.5,
    radius: f32 = 0.3,
    width: f32 = 0.1,
    enable_suspension_force_point: u32 = 0,
    /// WheelSettingsWV
    inertia: f32 = 0.9,
    angular_damping: f32 = 0.2,
    max_steer_angle: f32 = zolt.math.degreesToRadians(70.0),
    /// mLongitudinalFriction: `num_longitudinal_friction` (x, y) points at floats[longitudinal_friction..]
    /// (`invalid_handle`: the curve of the constructor)
    longitudinal_friction: u32 = invalid_handle,
    num_longitudinal_friction: u32 = 0,
    /// mLateralFriction, like `longitudinal_friction`
    lateral_friction: u32 = invalid_handle,
    num_lateral_friction: u32 = 0,
    max_brake_torque: f32 = 1500.0,
    max_hand_brake_torque: f32 = 4000.0,
    /// WheelSettingsTV: mLongitudinalFriction, mLateralFriction
    longitudinal_friction_tv: f32 = 4.0,
    lateral_friction_tv: f32 = 2.0,

    /// Jolt's defaults of a wheel of `kind` (both classes keep the WheelSettings defaults; checked by the test "SimScenes
    /// Phase 7 description defaults are Jolt's")
    pub fn defaults(kind: VehicleWheelKind) SimVehicleWheel {
        return .{ .kind = @intFromEnum(kind) };
    }
};

/// The VehicleCollisionTester of a vehicle (the constructor arguments of the class of `kind`, with the default arguments
/// of the constructors)
pub const SimVehicleTester = extern struct {
    /// VehicleTesterKind
    kind: u32 = @intFromEnum(VehicleTesterKind.ray),
    /// The object layer of the wheels (inObjectLayer, no default in Jolt)
    object_layer: u32 = 0,
    /// Ray, CastSphere: inUp, inMaxSlopeAngle
    up: [3]f32 = .{ 0, 1, 0 },
    max_slope_angle: f32 = zolt.math.degreesToRadians(80.0),
    /// CastSphere: inRadius (no default in Jolt: 0.3 is the harness' choice, not a Jolt default)
    radius: f32 = 0.3,
    /// CastCylinder: inConvexRadiusFraction
    convex_radius_fraction: f32 = 0.1,
};

/// A vehicle (OpKind.create_vehicle): VehicleConstraintSettings with its wheels, anti-roll bars and controller settings
/// (Jolt's defaults of the wheeled controller; `SimVehicle.defaults(kind)` gives the defaults of each kind), the
/// collision tester and the harness' callbacks. The harness creates `VehicleConstraint(body, settings)`, sets the
/// collision tester, adds the constraint (AddConstraint) and the step listener (AddStepListener).
pub const SimVehicle = extern struct {
    /// The vehicle body (body handle, created before)
    body: u32 = invalid_handle,
    /// The ConstraintSettings members (Jolt's defaults)
    enabled: u32 = 1,
    constraint_priority: u32 = 0,
    num_velocity_steps_override: u32 = 0,
    num_position_steps_override: u32 = 0,
    draw_constraint_size: f32 = 1.0,
    user_data: u64 = 0,
    up: [3]f32 = .{ 0, 1, 0 },
    forward: [3]f32 = .{ 0, 0, 1 },
    max_pitch_roll_angle: f32 = std.math.pi,
    /// mWheels: `num_wheels` at vehicle_wheels[first_wheel..]
    first_wheel: u32 = 0,
    num_wheels: u32 = 0,
    /// mAntiRollBars: `num_anti_roll_bars` at vehicle_anti_roll_bars[first_anti_roll_bar..]
    first_anti_roll_bar: u32 = 0,
    num_anti_roll_bars: u32 = 0,
    /// mController: VehicleControllerKind
    controller: u32 = @intFromEnum(VehicleControllerKind.wheeled),
    engine: SimVehicleEngine = .{},
    transmission: SimVehicleTransmission = .{},
    /// Wheeled / motorcycle: mDifferentials (`num_differentials` at vehicle_differentials[first_differential..]),
    /// mDifferentialLimitedSlipRatio
    first_differential: u32 = 0,
    num_differentials: u32 = 0,
    differential_limited_slip_ratio: f32 = 1.4,
    /// Tracked: mTracks (left, right)
    tracks: [2]SimVehicleTrack = .{ .{}, .{} },
    /// Motorcycle: MotorcycleControllerSettings
    max_lean_angle: f32 = zolt.math.degreesToRadians(45.0),
    lean_spring_constant: f32 = 5000.0,
    lean_spring_damping: f32 = 1000.0,
    lean_spring_integration_coefficient: f32 = 0.0,
    lean_spring_integration_coefficient_decay: f32 = 4.0,
    lean_smoothing_factor: f32 = 0.8,
    /// SetVehicleCollisionTester
    tester: SimVehicleTester = .{},
    /// Harness: VehicleCallbacks bits (SetPreStepCallback / SetPostCollideCallback / SetPostStepCallback)
    callbacks: u32 = 0,
    /// Harness: VehicleCombineFriction (SetCombineFriction)
    combine_friction: u32 = 0,
    /// Harness: VehicleTireMaxImpulse (WheeledVehicleController::SetTireMaxImpulseCallback, wheeled / motorcycle)
    tire_max_impulse: u32 = 0,

    /// Jolt's defaults of the controller kind: the engine and transmission of TrackedVehicleControllerSettings'
    /// constructor for a tracked vehicle, else WheeledVehicleControllerSettings' (checked by the test "SimScenes Phase 7
    /// description defaults are Jolt's")
    pub fn defaults(kind: VehicleControllerKind) SimVehicle {
        var v: SimVehicle = .{ .controller = @intFromEnum(kind) };
        if (kind == .tracked) {
            v.engine.min_rpm = 500.0;
            v.engine.max_rpm = 4000.0;
            v.engine.max_torque = 500.0;
            v.transmission.shift_down_rpm = 1000.0;
            v.transmission.shift_up_rpm = 3500.0;
        }
        return v;
    }
};

/// The step callbacks of a vehicle (bits of `SimVehicle.callbacks`, OpKind.vehicle_set_callbacks). They run in the step
/// listener jobs, their records (RecordKind.vehicle_*) are compared sorted.
pub const VehicleCallbacks = struct {
    /// SetPreStepCallback: record
    pub const pre_step: u32 = 1 << 0;
    /// SetPostCollideCallback: record
    pub const post_collide: u32 = 1 << 1;
    /// SetPostStepCallback: record
    pub const post_step: u32 = 1 << 2;
    /// Wheeled / motorcycle controller: the pre-step callback steers, SetRightInput(GetRightInput() > 0 ? -0.25 : 0.5)
    /// (alternating, from the controller's saved state, so that a restore or a D12 sweep gives the same steering)
    pub const steer: u32 = 1 << 3;
};

/// The friction combine function of a vehicle (`SimVehicle.combine_friction`, OpKind.vehicle_set_combine_friction)
pub const VehicleCombineFriction = enum(u32) {
    /// Jolt's default: sqrt(tire friction * body friction) in both directions
    default = 0,
    /// longitudinal = 0.5 * (longitudinal + body friction), lateral = min(lateral, body friction), then both *= 0.9 on
    /// the wheels with an odd index (SimVehicleCombineFriction of SimulationReference.cpp)
    custom = 1,
};

/// The tire max impulse callback of a wheeled / motorcycle controller (`SimVehicle.tire_max_impulse`)
pub const VehicleTireMaxImpulse = enum(u32) {
    /// Jolt's default: friction * suspension impulse in both directions
    default = 0,
    /// longitudinal = (0.9 * longitudinal friction) * suspension impulse, lateral = lateral friction * suspension impulse,
    /// then both *= 0.5 on wheel 0 (SimVehicleTireMaxImpulse of SimulationReference.cpp)
    custom = 1,
};

/// The parts of the settings of a vehicle description whose binary state is compared before a scene runs (the
/// SaveBinaryState bytes on both sides, and the round trips: each side restores the other side's bytes into a default
/// object of the part's class and saves again; jolt_sim_vehicle_settings_part of SimulationReference.cpp, VehicleSlot's
/// `settingsPart` of SimulationParity.zig)
pub const VehicleSettingsPart = enum(u32) {
    /// VehicleConstraintSettings with its wheels: saved only (Jolt's RestoreBinaryState dereferences the null wheel
    /// settings that it creates, CharacterVehicleArchitecture E6)
    settings = 0,
    /// VehicleConstraintSettings without its wheels (mWheels cleared): restored with ConstraintSettings::
    /// sRestoreFromBinaryState (the RTTI hash, the controller from its hash)
    settings_without_wheels = 1,
    /// mController (the class of `SimVehicle.controller`)
    controller = 2,
    /// mWheels[item] (WheelSettingsWV / WheelSettingsTV by the wheel's kind)
    wheel = 3,
    /// The controller's mEngine (VehicleEngineSettings)
    engine = 4,
    /// The controller's mTransmission (VehicleTransmissionSettings)
    transmission = 5,
    /// Tracked: mTracks[item] (VehicleTrackSettings)
    track = 6,
    /// Wheeled / motorcycle: mDifferentials[item] (VehicleDifferentialSettings)
    differential = 7,
    /// mAntiRollBars[item] (VehicleAntiRollBar)
    anti_roll_bar = 8,

    /// The number of items of the part in a vehicle description
    pub fn numItems(part: VehicleSettingsPart, vehicle: *const SimVehicle) u32 {
        const tracked = vehicle.controller == @intFromEnum(VehicleControllerKind.tracked);
        return switch (part) {
            .settings, .settings_without_wheels, .controller, .engine, .transmission => 1,
            .wheel => vehicle.num_wheels,
            .track => if (tracked) 2 else 0,
            .differential => if (tracked) 0 else vehicle.num_differentials,
            .anti_roll_bar => vehicle.num_anti_roll_bars,
        };
    }

    /// True when the part is compared by round trips too (all but `settings`)
    pub fn roundTrips(part: VehicleSettingsPart) bool {
        return part != .settings;
    }
};

/// The number of values of the CharacterID test (jolt_sim_character_id_sequence of SimulationReference.cpp,
/// CharacterVirtualSlot's `characterIdSequence` of SimulationParity.zig): after sSetNextCharacterID(first) a default
/// constructed CharacterVirtualSettings a (its mID, then the ID that sNextCharacterID would return), a copy b of a (mID,
/// next), c default constructed then assigned from a (mID, next), a CharacterVirtual of a with shape 0 at the origin
/// (GetID, next), GetCharacterVirtualSettings of it (mID, next)
pub const character_id_sequence_len = 10;

/// The CharacterID values that Jolt gives in the CharacterID test (CharacterVehicleArchitecture "Settled in the
/// foundation review": the default initializer of CharacterVirtualSettings and GetCharacterVirtualSettings consume an ID,
/// the copy constructor and the assignment do not, CharacterVirtual takes the ID of its settings)
pub fn characterIdSequence(first: u32) [character_id_sequence_len]u32 {
    return .{ first, first + 1, first, first + 1, first, first + 2, first, first + 2, first, first + 3 };
}

/// The reserved OpKind ranges of the Phase 7 operations (see the file header): every worker group adds operations in
/// its own range only. ZoltWorld.applyPhase7Op of SimulationParity.zig routes each range to its slot.
pub const Phase7OpRange = enum(u32) {
    /// Character (400..429, 423..429 free)
    character = 400,
    /// CharacterVirtual and CharacterVsCharacterCollisionSimple (430..489, 468..489 free)
    character_virtual = 430,
    /// Ragdoll (490..519, 511..519 free)
    ragdoll = 490,
    /// The vehicle core group (VehicleSlot): VehicleConstraint, the collision testers, the wheels, the engine, the
    /// transmission, the anti-roll bars and the wheeled controller (520..569, 563..569 free)
    vehicle = 520,
    /// The vehicle variants group: TrackedVehicleController (TrackedVehicleSlot, 570..579, 576..579 free)
    vehicle_tracked = 570,
    /// The vehicle variants group: MotorcycleController (MotorcycleSlot, 580..589, 587..589 free)
    vehicle_motorcycle = 580,
    /// Spare (590..599), unassigned
    spare = 590,

    /// The range of an operation number (null: not a Phase 7 operation)
    pub fn of(op: u32) ?Phase7OpRange {
        if (op < 400 or op >= 600) return null;
        if (op < 430) return .character;
        if (op < 490) return .character_virtual;
        if (op < 520) return .ragdoll;
        if (op < 570) return .vehicle;
        if (op < 580) return .vehicle_tracked;
        if (op < 590) return .vehicle_motorcycle;
        return .spare;
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

    // Character, CharacterVirtual, Ragdoll and Vehicle operations (Phase 7), in the reserved ranges of `Phase7OpRange`.
    // `index` is the object handle (the n-th created object of the class: character, character virtual, ragdoll or
    // vehicle handle; the create operations take the description index instead), `i` holds the integer arguments,
    // `f` / `r` the others, `body` a body handle. `lock` means: i[3] != 0 passes inLockBodies = false (the no-lock
    // interfaces; the operations run between the updates, nothing else touches the bodies), else Jolt's default true.
    // The `expect_*` operations are checks of the scene: they change nothing, the driver fails unless their result is
    // `count` (SceneBuilder.expect*).

    // ---- Phase7OpRange.character (400..429): Character ----
    /// new Character(characters[index], system) (the next character handle); returns GetBodyID() (invalid when out of
    /// bodies)
    create_character = 400,
    /// AddToPhysicsSystem(activation, lock); the character operations return the BodyState bits of the character's body
    character_add_to_physics_system = 401,
    /// RemoveFromPhysicsSystem(lock)
    character_remove_from_physics_system = 402,
    /// Activate(lock)
    character_activate = 403,
    /// PostSimulation(max separation distance f[0], lock); returns GetGroundState()
    character_post_simulation = 404,
    /// SetLinearAndAngularVelocity(f[0..3], f[3..6], lock)
    character_set_linear_and_angular_velocity = 405,
    /// SetLinearVelocity(f[0..3], lock)
    character_set_linear_velocity = 406,
    /// AddLinearVelocity(f[0..3], lock)
    character_add_linear_velocity = 407,
    /// AddImpulse(f[0..3], lock)
    character_add_impulse = 408,
    /// SetPositionAndRotation(r, f[0..4], activation, lock)
    character_set_position_and_rotation = 409,
    /// SetPosition(r, activation, lock)
    character_set_position = 410,
    /// SetRotation(f[0..4], activation, lock)
    character_set_rotation = 411,
    /// SetLayer(i[0], lock)
    character_set_layer = 412,
    /// SetShape(shapes[i[0]], max penetration depth f[0], lock); returns its result
    character_set_shape = 413,
    /// CharacterBase::SetMaxSlopeAngle(f[0])
    character_set_max_slope_angle = 414,
    /// CharacterBase::SetUp(f[0..3])
    character_set_up = 415,
    /// CharacterBase::SetSupportingVolume(Plane(f[0..3], f[3]))
    character_set_supporting_volume = 416,
    /// delete (the character must be removed from the system: Jolt destroys its body); returns 0
    character_destroy = 417,
    /// Returns the hash (`hashBytes`) of the members of GetCharacterSettings(lock) (shapes as their index)
    character_settings_hash = 418,
    /// CheckCollision(position r, rotation f[0..4], movement direction f[4..7], max separation distance f[7],
    /// shapes[i[0]], base offset r, a collector that hashes every hit in call order, lock); returns that hash
    /// (SimCheckCollisionCollector of SimulationReference.cpp: contact points, penetration axis and depth, sub shape IDs,
    /// body ID 2 and the character ID of the collector's user data, then the number of hits)
    character_check_collision = 419,
    /// Check: GetGroundState() must be `count`
    character_expect_ground_state = 420,
    /// Returns the hash of the getters with `lock`: GetLinearVelocity, GetPositionAndRotation, GetPosition, GetRotation,
    /// GetCenterOfMassPosition, GetWorldTransform (3 rotation columns, the translation), GetTransformedShape
    /// (mShapePositionCOM, mShapeRotation, GetShapeScale, mBodyID, the shape index)
    character_body_hash = 421,
    /// Check: the BodyState bits (IsActive | IsAdded << 1) of the character's body must be `count`
    /// (`SceneBuilder.expectCharacterBodyState`: a character asleep before an operation that must not wake it)
    character_expect_body_state = 422,
    // 423..429: free for Character operations

    // ---- Phase7OpRange.character_virtual (430..489): CharacterVirtual, CharacterVsCharacterCollisionSimple ----
    /// new CharacterVirtual(character_virtuals[index], system) plus the harness setup of the description (the next
    /// character virtual handle); returns GetInnerBodyID(). The character virtual operations return 0 unless noted.
    create_character_virtual = 430,
    /// Remove the character from every CharacterVsCharacterCollisionSimple, then release it (destroys its inner body)
    cv_destroy = 431,
    /// SetLinearVelocity(f[0..3])
    cv_set_linear_velocity = 432,
    /// SetPosition(r)
    cv_set_position = 433,
    /// SetRotation(f[0..4])
    cv_set_rotation = 434,
    /// CharacterBase::SetUp(f[0..3])
    cv_set_up = 435,
    /// CharacterBase::SetMaxSlopeAngle(f[0])
    cv_set_max_slope_angle = 436,
    /// CharacterBase::SetSupportingVolume(Plane(f[0..3], f[3]))
    cv_set_supporting_volume = 437,
    /// SetMass(f[0])
    cv_set_mass = 438,
    /// SetMaxStrength(f[0])
    cv_set_max_strength = 439,
    /// SetPenetrationRecoverySpeed(f[0])
    cv_set_penetration_recovery_speed = 440,
    /// SetEnhancedInternalEdgeRemoval(i[0] != 0)
    cv_set_enhanced_internal_edge_removal = 441,
    /// SetMaxNumHits(i[0])
    cv_set_max_num_hits = 442,
    /// SetHitReductionCosMaxAngle(f[0])
    cv_set_hit_reduction_cos_max_angle = 443,
    /// SetShapeOffset(f[0..3])
    cv_set_shape_offset = 444,
    /// SetUserData(i[0] | i[1] << 32)
    cv_set_user_data = 445,
    /// SetListener(CharacterListenerMode(i[0]))
    cv_set_listener = 446,
    /// SetCharacterVsCharacterCollision(collision group i[0], `invalid_handle`: nullptr)
    cv_set_character_vs_character_collision = 447,
    /// CharacterVsCharacterCollisionSimple(i[0])::Add
    cv_add_to_collision_group = 448,
    /// CharacterVsCharacterCollisionSimple(i[0])::Remove
    cv_remove_from_collision_group = 449,
    /// Update(delta time f[0], gravity f[1..4], filters (CharacterQueryFilter), the world's TempAllocator)
    cv_update = 450,
    /// ExtendedUpdate(delta time f[0], gravity f[1..4], ExtendedUpdateSettings f[4..16] (stick to floor step down
    /// f[4..7], walk stairs step up f[7..10], min step forward f[10], step forward test f[11], cos angle forward contact
    /// f[12], step down extra f[13..16]; `SceneBuilder.extendedUpdate`), filters, TempAllocator)
    cv_extended_update = 451,
    /// StickToFloor(step down f[0..3], filters, TempAllocator); returns its result
    cv_stick_to_floor = 452,
    /// WalkStairs(delta time f[0], step up f[1..4], step forward f[4..7], step forward test f[7..10], step down extra
    /// f[10..13], filters, TempAllocator); returns its result
    cv_walk_stairs = 453,
    /// Returns CanWalkStairs(f[0..3])
    cv_can_walk_stairs = 454,
    /// RefreshContacts(filters, TempAllocator)
    cv_refresh_contacts = 455,
    /// UpdateGroundVelocity()
    cv_update_ground_velocity = 456,
    /// SetShape(shapes[i[3]], max penetration depth f[0], filters, TempAllocator); returns its result
    cv_set_shape = 457,
    /// SetInnerBodyShape(shapes[i[0]])
    cv_set_inner_body_shape = 458,
    /// Returns the hash of CancelVelocityTowardsSteepSlopes(f[0..3])
    cv_cancel_velocity_towards_steep_slopes = 459,
    /// StartTrackingContactChanges()
    cv_start_tracking_contact_changes = 460,
    /// FinishTrackingContactChanges()
    cv_finish_tracking_contact_changes = 461,
    /// CheckCollision(position r, rotation f[0..4], movement direction f[4..7], max separation distance f[7],
    /// shapes[i[3]], base offset r, the collector of character_check_collision, filters); returns the hash of the hits
    /// in their order (each with the character ID that CharacterVsCharacterCollisionSimple passes as user data)
    cv_check_collision = 462,
    /// Returns the hash of the members of GetCharacterVirtualSettings() (shapes as their index)
    cv_settings_hash = 463,
    /// Returns HasCollidedWith(body handle `body`)
    cv_has_collided_with_body = 464,
    /// Returns HasCollidedWith(GetID() of character virtual handle i[0]) | HasCollidedWith(that character) << 1
    cv_has_collided_with_character = 465,
    /// Check: GetGroundState() must be `count`
    cv_expect_ground_state = 466,
    /// SaveState into a StateRecorderImpl, then RestoreState from it (i[0] != 0: a validating recorder, which drops the
    /// active contacts without mHadCollision); returns the hash of the bytes
    cv_save_restore_state = 467,
    /// Check: the value of the character that i[0] selects (`CharacterVirtualValue`) must be `count`; `body` is the body
    /// handle of CharacterVirtualValue.ground_body_is
    cv_expect_value = 468,
    /// The velocity of Character::Step of CharacterVirtualTests.cpp: on the ground and not moving away from it (vertical
    /// velocity - vertical ground velocity < 0.1) GetGroundVelocity() + (0, jump speed f[7], 0), else the vertical
    /// component of GetLinearVelocity(); plus gravity f[3..6] * delta time f[6], plus the input f[0..3]; then
    /// SetLinearVelocity
    cv_set_step_velocity = 469,
    /// The step of a character in CharacterVirtualScene::UpdateTest of PerformanceTest: the new velocity from the ground
    /// state, GetLinearVelocity, GetPosition and the deterministic random input of UpdateTest's counter `count` (delta
    /// time f[0], PhysicsSystem::GetGravity()), then ExtendedUpdate (Jolt's default settings) with the filters of
    /// op.i[0..3] and the world's TempAllocator
    cv_perf_update = 470,
    // 471..489: free for CharacterVirtual operations

    // ---- Phase7OpRange.ragdoll (490..519): Ragdoll, RagdollSettings ----
    /// ragdolls[index]: the RagdollSettings (built at the first creation of the description) CreateRagdoll (the next
    /// ragdoll handle, null when out of bodies); returns GetBodyCount() (0: null). The ragdoll operations return
    /// IsActive() of the ragdoll unless noted.
    create_ragdoll = 490,
    /// AddToPhysicsSystem(activation, lock)
    ragdoll_add_to_physics_system = 491,
    /// RemoveFromPhysicsSystem(lock)
    ragdoll_remove_from_physics_system = 492,
    /// Activate(lock)
    ragdoll_activate = 493,
    /// Returns IsActive(lock)
    ragdoll_is_active = 494,
    /// SetGroupID(i[0], lock)
    ragdoll_set_group_id = 495,
    /// SetPose(SkeletonPose of ragdoll_poses[i[0]], lock)
    ragdoll_set_pose = 496,
    /// SetPose(root offset, joint matrices of ragdoll_poses[i[0]], lock)
    ragdoll_set_pose_matrices = 497,
    /// ResetWarmStart()
    ragdoll_reset_warm_start = 498,
    /// DriveToPoseUsingKinematics(SkeletonPose of ragdoll_poses[i[0]], delta time f[0], lock)
    ragdoll_drive_to_pose_using_kinematics = 499,
    /// DriveToPoseUsingKinematics(root offset, joint matrices of ragdoll_poses[i[0]], delta time f[0], lock)
    ragdoll_drive_to_pose_using_kinematics_matrices = 500,
    /// DriveToPoseUsingMotors(SkeletonPose of ragdoll_poses[i[0]]) (SwingTwist / Hinge constraints only)
    ragdoll_drive_to_pose_using_motors = 501,
    /// DriveToPoseUsingMotors(previous pose ragdoll_poses[i[1]], pose ragdoll_poses[i[0]], delta time f[0] > 0)
    ragdoll_drive_to_pose_using_motors_velocity = 502,
    /// SetLinearAndAngularVelocity(f[0..3], f[3..6], lock)
    ragdoll_set_linear_and_angular_velocity = 503,
    /// SetLinearVelocity(f[0..3], lock)
    ragdoll_set_linear_velocity = 504,
    /// AddLinearVelocity(f[0..3], lock)
    ragdoll_add_linear_velocity = 505,
    /// AddImpulse(f[0..3], lock)
    ragdoll_add_impulse = 506,
    /// delete (the ragdoll must be removed from the system: Jolt destroys its bodies); returns 0
    ragdoll_destroy = 507,
    /// Returns the hash of GetRagdollSettings()->SaveBinaryState(stream, save shapes i[0] != 0, save group filter
    /// i[1] != 0) (`index` is the ragdoll handle)
    ragdoll_settings_hash = 508,
    /// Returns the hash of GetPose(SkeletonPose &, lock): the root offset and the joint matrices
    ragdoll_get_pose_hash = 509,
    /// Check: IsActive() must be `count`
    ragdoll_expect_active = 510,
    // 511..519: free for Ragdoll operations

    // ---- Phase7OpRange.vehicle (520..569): VehicleConstraint, the collision testers, the wheels, the engine, the
    // transmission, the anti-roll bars, the wheeled controller (the vehicle core group, VehicleSlot) ----
    /// The vehicles[index] constraint (the next vehicle handle): VehicleConstraint(body, settings), the collision
    /// tester, AddConstraint and AddStepListener; returns the number of constraints of the system. The vehicle
    /// operations (also the tracked and motorcycle ones) return the ConstraintState bits of the vehicle constraint
    /// unless noted.
    create_vehicle = 520,
    /// RemoveStepListener + RemoveConstraint; returns the number of constraints
    vehicle_remove = 521,
    /// AddConstraint + AddStepListener of a removed vehicle; returns the number of constraints
    vehicle_add = 522,
    /// SetMaxPitchRollAngle(f[0])
    vehicle_set_max_pitch_roll_angle = 523,
    /// OverrideGravity(f[0..3])
    vehicle_override_gravity = 524,
    /// ResetGravityOverride()
    vehicle_reset_gravity_override = 525,
    /// SetNumStepsBetweenCollisionTestActive(i[0])
    vehicle_set_num_steps_between_collision_test_active = 526,
    /// SetNumStepsBetweenCollisionTestInactive(i[0])
    vehicle_set_num_steps_between_collision_test_inactive = 527,
    /// SetVehicleCollisionTester(a new tester of VehicleTesterKind(i[0]), object layer i[1], up f[0..3], max slope angle
    /// f[3], radius f[4], convex radius fraction f[5])
    vehicle_set_collision_tester = 528,
    /// The vehicle's tester: SetObjectLayer(i[0])
    vehicle_tester_set_object_layer = 529,
    /// The vehicle's tester: i[0] == 0: SetBroadPhaseLayerFilter / SetObjectLayerFilter / SetBodyFilter(nullptr), else
    /// the default filters of object layer i[1] and an IgnoreMultipleBodiesFilter of the vehicle body and body handle
    /// `body` (`invalid_handle`: the vehicle body only; a body filter replaces the tester's own IgnoreSingleBodyFilter of
    /// the vehicle body)
    vehicle_tester_set_filters = 530,
    /// SetCombineFriction(VehicleCombineFriction(i[0]))
    vehicle_set_combine_friction = 531,
    /// The step callbacks (VehicleCallbacks bits i[0])
    vehicle_set_callbacks = 532,
    /// GetWheel(i[0])->SetAngularVelocity(f[0])
    vehicle_wheel_set_angular_velocity = 533,
    /// GetWheel(i[0])->SetRotationAngle(f[0])
    vehicle_wheel_set_rotation_angle = 534,
    /// GetWheel(i[0])->SetSteerAngle(f[0])
    vehicle_wheel_set_steer_angle = 535,
    /// GetAntiRollBars()[i[0]] = (left wheel i[1], right wheel i[2], stiffness f[0])
    vehicle_set_anti_roll_bar = 536,
    /// Returns the hash of GetConstraintSettings()->SaveBinaryState
    vehicle_settings_hash = 537,
    /// Returns the hash of GetController()->GetSettings()->SaveBinaryState
    vehicle_controller_settings_hash = 538,
    /// Check: the bits of the wheels with HasContact() must be `count`
    vehicle_expect_wheel_contacts = 539,
    /// Engine (wheeled, tracked, motorcycle): GetEngine().SetCurrentRPM(f[0])
    vehicle_engine_set_current_rpm = 540,
    /// Engine: mMaxTorque f[0], mMinRPM f[1], mMaxRPM f[2], mInertia f[3], mAngularDamping f[4] of GetEngine()
    vehicle_engine_set_settings = 541,
    /// Engine: mNormalizedTorque = i[1] (x, y) points at floats[i[0]..]
    vehicle_engine_set_torque_curve = 542,
    /// Transmission: GetTransmission().Set(gear i[0] as int, clutch friction f[0])
    vehicle_transmission_set = 543,
    /// Transmission: mMode i[0], mSwitchTime f[0], mClutchReleaseTime f[1], mSwitchLatency f[2], mShiftUpRPM f[3],
    /// mShiftDownRPM f[4], mClutchStrength f[5]
    vehicle_transmission_set_settings = 544,
    /// Transmission: mGearRatios = i[1] at floats[i[0]..], mReverseGearRatios = i[3] at floats[i[2]..]
    vehicle_transmission_set_gear_ratios = 545,
    /// Wheeled / motorcycle: SetDriverInput(f[0], f[1], f[2], f[3])
    wheeled_set_driver_input = 546,
    /// Wheeled / motorcycle: SetForwardInput(f[0])
    wheeled_set_forward_input = 547,
    /// Wheeled / motorcycle: SetRightInput(f[0])
    wheeled_set_right_input = 548,
    /// Wheeled / motorcycle: SetBrakeInput(f[0])
    wheeled_set_brake_input = 549,
    /// Wheeled / motorcycle: SetHandBrakeInput(f[0])
    wheeled_set_hand_brake_input = 550,
    /// Wheeled / motorcycle: GetDifferentials()[i[0]] = (left i[1] as int, right i[2] as int, ratio f[0], left right
    /// split f[1], limited slip ratio f[2], engine torque ratio f[3])
    wheeled_set_differential = 551,
    /// Wheeled / motorcycle: SetDifferentialLimitedSlipRatio(f[0])
    wheeled_set_differential_limited_slip_ratio = 552,
    /// Wheeled / motorcycle: SetTireMaxImpulseCallback(VehicleTireMaxImpulse(i[0]))
    wheeled_set_tire_max_impulse_callback = 553,
    /// The Constraint interface of the vehicle (a vehicle is not a constraint handle): SetEnabled(i[0] != 0). A disabled
    /// vehicle whose body is active runs OnStep (the callbacks, the wheel collision tests) but is not solved
    /// (VehicleConstraint::IsActive includes mEnabled).
    vehicle_set_enabled = 554,
    /// ResetWarmStart()
    vehicle_reset_warm_start = 555,
    /// SetConstraintPriority(i[0])
    vehicle_set_constraint_priority = 556,
    /// SetNumVelocityStepsOverride(i[0]), SetNumPositionStepsOverride(i[1]) (each < 256)
    vehicle_set_steps_override = 557,
    /// SetUserData(i[0] | i[1] << 32)
    vehicle_set_user_data = 558,
    /// Check: the ConstraintState bits of the vehicle | IsActive(vehicle body) << 2 must be `count`
    vehicle_expect_constraint_state = 559,
    /// Wheeled / motorcycle: GetWheel(i[0]) as WheelWV: ApplyTorque(torque f[0], delta time f[1])
    vehicle_wheel_wv_apply_torque = 560,
    /// Engine: ApplyTorque(torque f[0], delta time f[1])
    vehicle_engine_apply_torque = 561,
    /// Engine: ApplyDamping(delta time f[0])
    vehicle_engine_apply_damping = 562,
    // 563..569: free for vehicle core operations

    // ---- Phase7OpRange.vehicle_tracked (570..579): TrackedVehicleController (the vehicle variants group,
    // TrackedVehicleSlot) ----
    /// SetDriverInput(f[0], f[1], f[2], f[3]) (the ratios must not be 0)
    tracked_set_driver_input = 570,
    /// SetForwardInput(f[0])
    tracked_set_forward_input = 571,
    /// SetLeftRatio(f[0]) (not 0)
    tracked_set_left_ratio = 572,
    /// SetRightRatio(f[0]) (not 0)
    tracked_set_right_ratio = 573,
    /// SetBrakeInput(f[0])
    tracked_set_brake_input = 574,
    /// GetTracks()[i[0]]: mAngularVelocity f[0], mInertia f[1], mAngularDamping f[2], mMaxBrakeTorque f[3],
    /// mDifferentialRatio f[4]
    tracked_set_track = 575,
    // 576..579: free for tracked operations

    // ---- Phase7OpRange.vehicle_motorcycle (580..589): MotorcycleController (the vehicle variants group,
    // MotorcycleSlot) ----
    /// EnableLeanController(i[0] != 0)
    motorcycle_enable_lean_controller = 580,
    /// EnableLeanSteeringLimit(i[0] != 0)
    motorcycle_enable_lean_steering_limit = 581,
    /// SetLeanSpringConstant(f[0])
    motorcycle_set_lean_spring_constant = 582,
    /// SetLeanSpringDamping(f[0])
    motorcycle_set_lean_spring_damping = 583,
    /// SetLeanSpringIntegrationCoefficient(f[0])
    motorcycle_set_lean_spring_integration_coefficient = 584,
    /// SetLeanSpringIntegrationCoefficientDecay(f[0])
    motorcycle_set_lean_spring_integration_coefficient_decay = 585,
    /// SetLeanSmoothingFactor(f[0])
    motorcycle_set_lean_smoothing_factor = 586,
    // 587..589: free for motorcycle operations

    // ---- Phase7OpRange.spare (590..599): unassigned ----
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

    // Phase 7: the descriptions of the characters, ragdolls and vehicles (created by their operations), and a pool of
    // integers (track wheel indices; the float pool `floats` holds the curves, gear ratios and pose joint states)
    characters: [*]const SimCharacter,
    num_characters: u32,
    character_virtuals: [*]const SimCharacterVirtual,
    num_character_virtuals: u32,
    ragdolls: [*]const SimRagdoll,
    num_ragdolls: u32,
    ragdoll_joints: [*]const SimRagdollJoint,
    num_ragdoll_joints: u32,
    ragdoll_additional_constraints: [*]const SimRagdollAdditionalConstraint,
    num_ragdoll_additional_constraints: u32,
    ragdoll_poses: [*]const SimRagdollPose,
    num_ragdoll_poses: u32,
    vehicles: [*]const SimVehicle,
    num_vehicles: u32,
    vehicle_wheels: [*]const SimVehicleWheel,
    num_vehicle_wheels: u32,
    vehicle_differentials: [*]const SimVehicleDifferential,
    num_vehicle_differentials: u32,
    vehicle_anti_roll_bars: [*]const SimVehicleAntiRollBar,
    num_vehicle_anti_roll_bars: u32,
    uints: [*]const u32,
    num_uints: u32,
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

    // Phase 7: the CharacterContactListener callbacks of the character virtuals (the user thread calls them in Jolt's
    // deterministic order: they are compared in call order, `isCharacterRecord`). A contact record
    // (`characterContactRecord`) has ids: character ID, CharacterContact::mBodyB, mCharacterIDB, mSubShapeIDB; data[0..3]
    // mContactNormal, data[3] mDistance, data[4] mFraction, data[5] the flags (mMotionTypeB | mIsSensorB << 8 |
    // mHadCollision << 9 | mWasDiscarded << 10 | mCanPushCharacter << 11 | mIsBackFacingContact << 12 | (mCharacterB !=
    // null) << 13, and for added / persisted the incoming CharacterContactSettings mCanPushCharacter << 16 |
    // mCanReceiveImpulses << 17), data[6] the hash of the other members (`hashCharacterContact`), data[7] the result:
    // the return value of validate, the settings after the callback for added / persisted (can push | can receive << 1).
    /// OnContactValidate (a contact record)
    character_contact_validate = 7,
    /// OnContactAdded (a contact record)
    character_contact_added = 8,
    /// OnContactPersisted (a contact record)
    character_contact_persisted = 9,
    /// OnContactRemoved: ids character ID, body ID 2, invalid, sub shape ID 2
    character_contact_removed = 10,
    /// OnCharacterContactValidate (a contact record)
    character_vs_character_contact_validate = 11,
    /// OnCharacterContactAdded (a contact record)
    character_vs_character_contact_added = 12,
    /// OnCharacterContactPersisted (a contact record)
    character_vs_character_contact_persisted = 13,
    /// OnCharacterContactRemoved: ids character ID, invalid, other character ID, sub shape ID 2
    character_vs_character_contact_removed = 14,
    /// OnContactSolve: ids character ID, body ID 2, invalid, sub shape ID 2; data[0..3] ioNewCharacterVelocity as passed
    /// in, data[3..6] inCharacterVelocity, data[6] the hash of the contact position, normal, velocity and the material's
    /// debug name, data[7] the hash of ioNewCharacterVelocity after the callback
    character_contact_solve = 15,
    /// OnCharacterContactSolve: like character_contact_solve, ids[2] the other character's ID, ids[1] invalid
    character_vs_character_contact_solve = 16,
    /// OnAdjustBodyVelocity: ids character ID, body ID 2; data[0..3] the linear velocity as passed in, data[3..6] the
    /// angular velocity, data[6] the hash of both after the callback
    character_adjust_body_velocity = 17,

    // Phase 7: the step callbacks of a vehicle (VehicleCallbacks), called from the step listener jobs: compared sorted.
    // ids[0] vehicle handle, ids[1] the BodyID of the vehicle body; data[0] delta time bits, data[1] first step, data[2]
    // last step, data[3] the bits of the wheels with HasContact(), data[4] the FNV-1a hash of GetSuspensionLength,
    // GetAngularVelocity and GetSteerAngle of every wheel in order (SimVehicleRecord of SimulationReference.cpp)
    vehicle_pre_step = 18,
    vehicle_post_collide = 19,
    vehicle_post_step = 20,

    // Reserved blocks for new kinds of the Phase 7 groups (see "Reserved numbers" in the file header): 21..29
    // Character, 30..39 CharacterVirtual (compared in call order like 7..17: added to the character recorder), 40..49
    // Ragdoll, 50..59 the vehicle core group, 60..69 the vehicle variants group, 70..79 later phases.

    /// True for the records of the character virtuals (the CharacterContactListener and the CharacterVirtual block:
    /// compared in call order, they come from the character recorder)
    pub fn isCharacterRecord(kind: u32) bool {
        return (kind >= @intFromEnum(RecordKind.character_contact_validate) and kind <= @intFromEnum(RecordKind.character_adjust_body_velocity)) or (kind >= 30 and kind < 40);
    }
};

/// The size of the arrays indexed by RecordKind (every kind of the reserved blocks is below it)
pub const num_record_kinds = 80;

comptime {
    for (@typeInfo(RecordKind).@"enum".fields) |field| {
        if (field.value >= num_record_kinds) @compileError("RecordKind." ++ field.name ++ " is not below num_record_kinds");
    }
}

/// The hash of the members of a CharacterContact that a contact record does not hold directly (data[6], see RecordKind;
/// SimHashCharacterContact in SimulationReference.cpp): FNV-1a over mPosition (3 Real), mLinearVelocity, mSurfaceNormal
/// (3 floats each), mUserData (u64) and the hash of the material's debug name (u32). mCharacterB is never dereferenced
/// (Jolt: "may be a dangling pointer"), only whether it is null is compared (flag bit 13, `has_character_b`); its ID
/// is mCharacterIDB.
pub fn hashCharacterContact(position: [3]Real, linear_velocity: [3]f32, surface_normal: [3]f32, user_data: u64, material_hash: u32) u32 {
    var h = hash_seed;
    h = hashBytes(h, std.mem.asBytes(&position));
    h = hashBytes(h, std.mem.asBytes(&linear_velocity));
    h = hashBytes(h, std.mem.asBytes(&surface_normal));
    h = hashBytes(h, std.mem.asBytes(&user_data));
    h = hashBytes(h, std.mem.asBytes(&material_hash));
    return h;
}

/// The identity of a material in the records and the object state: the hash of GetDebugName()
pub fn materialHash(debug_name: []const u8) u32 {
    return hashBytes(hash_seed, debug_name);
}

/// The fields of the object state (Phase 7): after every update both sides write the state of every character,
/// character virtual, ragdoll and vehicle (in handle order, destroyed objects are skipped) as a list of fields
/// `[field: u16][handle: u16][item: u16][size: u16][size bytes]` (`ObjectStateWriter`, SimObjectStateWriter in
/// SimulationReference.cpp; values little endian, floats as bits, Real as f32 or f64). `item` is the index of a contact,
/// wheel, joint, ... (0 for the fields of the object). `objectFieldLayout` names the components of each field for the
/// report of a mismatch. The `*_save_state` fields hold the bytes of SaveState.
pub const ObjectField = enum(u16) {
    // Character
    character_save_state = 1,
    /// GetBodyID, GetLayer, the shape index of GetShape, IsSupported, the material hash of GetGroundMaterial,
    /// GetGroundUserData
    character_properties = 2,
    /// GetUp, GetCosMaxSlopeAngle, GetSupportingVolume
    character_base = 3,
    /// GetPosition, GetRotation, GetCenterOfMassPosition, GetLinearVelocity, GetWorldTransform (lock bodies; not written
    /// for a character without body, out of bodies)
    character_body = 4,
    /// GetTransformedShape: mShapePositionCOM, mShapeRotation, GetShapeScale, mBodyID, the shape index (not written for a
    /// character without body)
    character_transformed_shape = 5,

    // CharacterVirtual
    cv_save_state = 10,
    /// GetID, GetUserData, GetInnerBodyID, the shape index of GetShape, GetMass, GetMaxStrength,
    /// GetPenetrationRecoverySpeed, GetEnhancedInternalEdgeRemoval, GetCharacterPadding, GetMaxNumHits,
    /// GetHitReductionCosMaxAngle, GetMaxHitsExceeded, GetShapeOffset
    cv_properties = 11,
    /// GetUp, GetCosMaxSlopeAngle, GetSupportingVolume, the material hash of GetGroundMaterial, GetGroundUserData,
    /// IsSupported, CanWalkStairs(GetLinearVelocity())
    cv_base = 12,
    /// GetCenterOfMassPosition, GetWorldTransform, GetCenterOfMassTransform
    cv_transforms = 13,
    /// GetTransformedShape
    cv_transformed_shape = 14,
    /// The number of GetActiveContacts()
    cv_num_active_contacts = 15,
    /// GetActiveContacts()[item], every member (mCharacterB != null, never dereferenced; mMaterial as its hash)
    cv_active_contact = 16,

    // Ragdoll
    /// GetBodyCount, GetConstraintCount, IsActive
    ragdoll_properties = 20,
    /// GetBodyID(item)
    ragdoll_body = 21,
    /// GetRootTransform
    ragdoll_root_transform = 22,
    /// GetPose(RVec3 &, Mat44 *): the root offset
    ragdoll_pose_root_offset = 23,
    /// GetPose(RVec3 &, Mat44 *): joint matrix `item`
    ragdoll_pose_joint = 24,
    /// GetWorldSpaceBounds
    ragdoll_bounds = 25,
    /// GetConstraint(item): GetSubType, GetConstraintPriority, GetEnabled, IsActive, the body IDs
    ragdoll_constraint = 26,

    // Vehicle
    /// IsActive, GetEnabled, GetNumStepsBetweenCollisionTestActive / Inactive, GetMaxPitchRollAngle,
    /// IsGravityOverridden, GetGravityOverride, GetLocalForward, GetLocalUp, GetWorldUp, the number of wheels, the body
    /// ID, GetConstraintPriority, GetNumVelocityStepsOverride, GetNumPositionStepsOverride, GetUserData
    vehicle_properties = 30,
    /// GetWheel(item): GetAngularVelocity, GetRotationAngle, GetSteerAngle, HasContact, GetContactBodyID,
    /// GetContactSubShapeID, GetSuspensionLength, HasHitHardPoint, GetSuspensionLambda, GetLongitudinalLambda,
    /// GetLateralLambda
    vehicle_wheel = 31,
    /// GetWheel(item) with a contact: GetContactPosition, GetContactPointVelocity, GetContactNormal,
    /// GetContactLongitudinal, GetContactLateral
    vehicle_wheel_contact = 32,
    /// GetWheelLocalBasis, GetWheelLocalTransform and GetWheelWorldTransform of wheel `item` (wheel right (0, 1, 0),
    /// wheel up (1, 0, 0))
    vehicle_wheel_transform = 33,
    /// WheelWV `item`: mLongitudinalSlip, mLateralSlip, mCombinedLongitudinalFriction, mCombinedLateralFriction,
    /// mBrakeImpulse
    vehicle_wheel_wv = 34,
    /// WheelTV `item`: mTrackIndex, mCombinedLongitudinalFriction, mCombinedLateralFriction, mBrakeImpulse
    vehicle_wheel_tv = 35,
    /// GetAntiRollBars()[item]
    vehicle_anti_roll_bar = 36,
    /// The engine: GetCurrentRPM, GetAngularVelocity, GetTorque(1), AllowSleep and its settings
    vehicle_engine = 37,
    /// The transmission: GetCurrentGear, GetClutchFriction, IsSwitchingGear, GetCurrentRatio, AllowSleep, mMode
    vehicle_transmission = 38,
    /// Wheeled / motorcycle: the four inputs, GetDifferentialLimitedSlipRatio, GetWheelSpeedAtClutch
    vehicle_wheeled = 39,
    /// Wheeled / motorcycle: GetDifferentials()[item] and its CalculateTorqueRatio of the current wheel speeds
    vehicle_differential = 40,
    /// Tracked: the four inputs
    vehicle_tracked = 41,
    /// Tracked: GetTracks()[item]
    vehicle_track = 42,
    /// Motorcycle: GetWheelBase, IsLeanControllerEnabled, IsLeanSteeringLimitEnabled, the five lean parameters
    vehicle_motorcycle = 43,
};

/// The header of a field of the object state
pub const ObjectFieldHeader = extern struct {
    field: u16,
    handle: u16,
    item: u16,
    size: u16,
};

/// The kind of a component of a field of the object state (u8: the bools and EMotionType of the SaveState fields)
pub const ObjectComponentKind = enum { u8, u32, i32, u64, f32, real };

/// A component of a field of the object state (for the report of a mismatch)
pub const ObjectComponent = struct {
    name: []const u8,
    kind: ObjectComponentKind,
    /// Number of values (3 for a Vec3)
    count: u8 = 1,
};

/// The components of a field of the object state (the order in which both sides write them; null: a block of bytes)
pub fn objectFieldLayout(field: ObjectField) ?[]const ObjectComponent {
    return switch (field) {
        .character_save_state, .cv_save_state => null,
        .character_properties => &.{ .{ .name = "body_id", .kind = .u32 }, .{ .name = "layer", .kind = .u32 }, .{ .name = "shape", .kind = .u32 }, .{ .name = "is_supported", .kind = .u32 }, .{ .name = "ground_material", .kind = .u32 }, .{ .name = "ground_user_data", .kind = .u64 } },
        .character_base => &.{ .{ .name = "up", .kind = .f32, .count = 3 }, .{ .name = "cos_max_slope_angle", .kind = .f32 }, .{ .name = "supporting_volume", .kind = .f32, .count = 4 } },
        .character_body => &.{ .{ .name = "position", .kind = .real, .count = 3 }, .{ .name = "rotation", .kind = .f32, .count = 4 }, .{ .name = "center_of_mass_position", .kind = .real, .count = 3 }, .{ .name = "linear_velocity", .kind = .f32, .count = 3 }, .{ .name = "world_transform.rotation", .kind = .f32, .count = 12 }, .{ .name = "world_transform.translation", .kind = .real, .count = 3 } },
        .character_transformed_shape, .cv_transformed_shape => &.{ .{ .name = "shape_position_com", .kind = .real, .count = 3 }, .{ .name = "shape_rotation", .kind = .f32, .count = 4 }, .{ .name = "shape_scale", .kind = .f32, .count = 3 }, .{ .name = "body_id", .kind = .u32 }, .{ .name = "shape", .kind = .u32 } },
        .cv_properties => &.{ .{ .name = "id", .kind = .u32 }, .{ .name = "user_data", .kind = .u64 }, .{ .name = "inner_body_id", .kind = .u32 }, .{ .name = "shape", .kind = .u32 }, .{ .name = "mass", .kind = .f32 }, .{ .name = "max_strength", .kind = .f32 }, .{ .name = "penetration_recovery_speed", .kind = .f32 }, .{ .name = "enhanced_internal_edge_removal", .kind = .u32 }, .{ .name = "character_padding", .kind = .f32 }, .{ .name = "max_num_hits", .kind = .u32 }, .{ .name = "hit_reduction_cos_max_angle", .kind = .f32 }, .{ .name = "max_hits_exceeded", .kind = .u32 }, .{ .name = "shape_offset", .kind = .f32, .count = 3 } },
        .cv_base => &.{ .{ .name = "up", .kind = .f32, .count = 3 }, .{ .name = "cos_max_slope_angle", .kind = .f32 }, .{ .name = "supporting_volume", .kind = .f32, .count = 4 }, .{ .name = "ground_material", .kind = .u32 }, .{ .name = "ground_user_data", .kind = .u64 }, .{ .name = "is_supported", .kind = .u32 }, .{ .name = "can_walk_stairs", .kind = .u32 } },
        .cv_transforms => &.{ .{ .name = "center_of_mass_position", .kind = .real, .count = 3 }, .{ .name = "world_transform.rotation", .kind = .f32, .count = 12 }, .{ .name = "world_transform.translation", .kind = .real, .count = 3 }, .{ .name = "center_of_mass_transform.rotation", .kind = .f32, .count = 12 }, .{ .name = "center_of_mass_transform.translation", .kind = .real, .count = 3 } },
        .cv_num_active_contacts => &.{.{ .name = "num_active_contacts", .kind = .u32 }},
        .cv_active_contact => &.{ .{ .name = "body_b", .kind = .u32 }, .{ .name = "character_id_b", .kind = .u32 }, .{ .name = "sub_shape_id_b", .kind = .u32 }, .{ .name = "position", .kind = .real, .count = 3 }, .{ .name = "linear_velocity", .kind = .f32, .count = 3 }, .{ .name = "contact_normal", .kind = .f32, .count = 3 }, .{ .name = "surface_normal", .kind = .f32, .count = 3 }, .{ .name = "distance", .kind = .f32 }, .{ .name = "fraction", .kind = .f32 }, .{ .name = "motion_type_b", .kind = .u32 }, .{ .name = "is_sensor_b", .kind = .u32 }, .{ .name = "has_character_b", .kind = .u32 }, .{ .name = "user_data", .kind = .u64 }, .{ .name = "material", .kind = .u32 }, .{ .name = "had_collision", .kind = .u32 }, .{ .name = "was_discarded", .kind = .u32 }, .{ .name = "can_push_character", .kind = .u32 }, .{ .name = "is_back_facing_contact", .kind = .u32 } },
        .ragdoll_properties => &.{ .{ .name = "body_count", .kind = .u32 }, .{ .name = "constraint_count", .kind = .u32 }, .{ .name = "is_active", .kind = .u32 } },
        .ragdoll_body => &.{.{ .name = "body_id", .kind = .u32 }},
        .ragdoll_root_transform => &.{ .{ .name = "position", .kind = .real, .count = 3 }, .{ .name = "rotation", .kind = .f32, .count = 4 } },
        .ragdoll_pose_root_offset => &.{.{ .name = "root_offset", .kind = .real, .count = 3 }},
        .ragdoll_pose_joint => &.{.{ .name = "joint_matrix", .kind = .f32, .count = 16 }},
        .ragdoll_bounds => &.{ .{ .name = "min", .kind = .f32, .count = 3 }, .{ .name = "max", .kind = .f32, .count = 3 } },
        .ragdoll_constraint => &.{ .{ .name = "sub_type", .kind = .u32 }, .{ .name = "priority", .kind = .u32 }, .{ .name = "enabled", .kind = .u32 }, .{ .name = "is_active", .kind = .u32 }, .{ .name = "body1_id", .kind = .u32 }, .{ .name = "body2_id", .kind = .u32 } },
        .vehicle_properties => &.{ .{ .name = "is_active", .kind = .u32 }, .{ .name = "enabled", .kind = .u32 }, .{ .name = "num_steps_between_collision_test_active", .kind = .u32 }, .{ .name = "num_steps_between_collision_test_inactive", .kind = .u32 }, .{ .name = "max_pitch_roll_angle", .kind = .f32 }, .{ .name = "is_gravity_overridden", .kind = .u32 }, .{ .name = "gravity_override", .kind = .f32, .count = 3 }, .{ .name = "local_forward", .kind = .f32, .count = 3 }, .{ .name = "local_up", .kind = .f32, .count = 3 }, .{ .name = "world_up", .kind = .f32, .count = 3 }, .{ .name = "num_wheels", .kind = .u32 }, .{ .name = "body_id", .kind = .u32 }, .{ .name = "constraint_priority", .kind = .u32 }, .{ .name = "num_velocity_steps_override", .kind = .u32 }, .{ .name = "num_position_steps_override", .kind = .u32 }, .{ .name = "user_data", .kind = .u64 } },
        .vehicle_wheel => &.{ .{ .name = "angular_velocity", .kind = .f32 }, .{ .name = "rotation_angle", .kind = .f32 }, .{ .name = "steer_angle", .kind = .f32 }, .{ .name = "has_contact", .kind = .u32 }, .{ .name = "contact_body_id", .kind = .u32 }, .{ .name = "contact_sub_shape_id", .kind = .u32 }, .{ .name = "suspension_length", .kind = .f32 }, .{ .name = "has_hit_hard_point", .kind = .u32 }, .{ .name = "suspension_lambda", .kind = .f32 }, .{ .name = "longitudinal_lambda", .kind = .f32 }, .{ .name = "lateral_lambda", .kind = .f32 } },
        .vehicle_wheel_contact => &.{ .{ .name = "contact_position", .kind = .real, .count = 3 }, .{ .name = "contact_point_velocity", .kind = .f32, .count = 3 }, .{ .name = "contact_normal", .kind = .f32, .count = 3 }, .{ .name = "contact_longitudinal", .kind = .f32, .count = 3 }, .{ .name = "contact_lateral", .kind = .f32, .count = 3 } },
        .vehicle_wheel_transform => &.{ .{ .name = "local_basis.forward", .kind = .f32, .count = 3 }, .{ .name = "local_basis.up", .kind = .f32, .count = 3 }, .{ .name = "local_basis.right", .kind = .f32, .count = 3 }, .{ .name = "local_transform", .kind = .f32, .count = 16 }, .{ .name = "world_transform.rotation", .kind = .f32, .count = 12 }, .{ .name = "world_transform.translation", .kind = .real, .count = 3 } },
        .vehicle_wheel_wv => &.{ .{ .name = "longitudinal_slip", .kind = .f32 }, .{ .name = "lateral_slip", .kind = .f32 }, .{ .name = "combined_longitudinal_friction", .kind = .f32 }, .{ .name = "combined_lateral_friction", .kind = .f32 }, .{ .name = "brake_impulse", .kind = .f32 } },
        .vehicle_wheel_tv => &.{ .{ .name = "track_index", .kind = .i32 }, .{ .name = "combined_longitudinal_friction", .kind = .f32 }, .{ .name = "combined_lateral_friction", .kind = .f32 }, .{ .name = "brake_impulse", .kind = .f32 } },
        .vehicle_anti_roll_bar => &.{ .{ .name = "left_wheel", .kind = .i32 }, .{ .name = "right_wheel", .kind = .i32 }, .{ .name = "stiffness", .kind = .f32 } },
        .vehicle_engine => &.{ .{ .name = "current_rpm", .kind = .f32 }, .{ .name = "angular_velocity", .kind = .f32 }, .{ .name = "torque_at_full_gas", .kind = .f32 }, .{ .name = "allow_sleep", .kind = .u32 }, .{ .name = "max_torque", .kind = .f32 }, .{ .name = "min_rpm", .kind = .f32 }, .{ .name = "max_rpm", .kind = .f32 }, .{ .name = "inertia", .kind = .f32 }, .{ .name = "angular_damping", .kind = .f32 } },
        .vehicle_transmission => &.{ .{ .name = "current_gear", .kind = .i32 }, .{ .name = "clutch_friction", .kind = .f32 }, .{ .name = "is_switching_gear", .kind = .u32 }, .{ .name = "current_ratio", .kind = .f32 }, .{ .name = "allow_sleep", .kind = .u32 }, .{ .name = "mode", .kind = .u32 } },
        .vehicle_wheeled => &.{ .{ .name = "forward_input", .kind = .f32 }, .{ .name = "right_input", .kind = .f32 }, .{ .name = "brake_input", .kind = .f32 }, .{ .name = "hand_brake_input", .kind = .f32 }, .{ .name = "differential_limited_slip_ratio", .kind = .f32 }, .{ .name = "wheel_speed_at_clutch", .kind = .f32 } },
        .vehicle_differential => &.{ .{ .name = "left_wheel", .kind = .i32 }, .{ .name = "right_wheel", .kind = .i32 }, .{ .name = "differential_ratio", .kind = .f32 }, .{ .name = "left_right_split", .kind = .f32 }, .{ .name = "limited_slip_ratio", .kind = .f32 }, .{ .name = "engine_torque_ratio", .kind = .f32 }, .{ .name = "left_torque_fraction", .kind = .f32 }, .{ .name = "right_torque_fraction", .kind = .f32 } },
        .vehicle_tracked => &.{ .{ .name = "forward_input", .kind = .f32 }, .{ .name = "left_ratio", .kind = .f32 }, .{ .name = "right_ratio", .kind = .f32 }, .{ .name = "brake_input", .kind = .f32 } },
        .vehicle_track => &.{ .{ .name = "angular_velocity", .kind = .f32 }, .{ .name = "driven_wheel", .kind = .u32 }, .{ .name = "num_wheels", .kind = .u32 }, .{ .name = "inertia", .kind = .f32 }, .{ .name = "angular_damping", .kind = .f32 }, .{ .name = "max_brake_torque", .kind = .f32 }, .{ .name = "differential_ratio", .kind = .f32 } },
        .vehicle_motorcycle => &.{ .{ .name = "wheel_base", .kind = .f32 }, .{ .name = "lean_controller_enabled", .kind = .u32 }, .{ .name = "lean_steering_limit_enabled", .kind = .u32 }, .{ .name = "lean_spring_constant", .kind = .f32 }, .{ .name = "lean_spring_damping", .kind = .f32 }, .{ .name = "lean_spring_integration_coefficient", .kind = .f32 }, .{ .name = "lean_spring_integration_coefficient_decay", .kind = .f32 }, .{ .name = "lean_smoothing_factor", .kind = .f32 } },
    };
}

/// The size of a component kind
pub fn objectComponentSize(kind: ObjectComponentKind) usize {
    return switch (kind) {
        .u8 => 1,
        .u32, .i32, .f32 => 4,
        .u64 => 8,
        .real => @sizeOf(Real),
    };
}

/// Writes the object state (the Zolt side, SimObjectStateWriter of SimulationReference.cpp): `begin` a field, the
/// values in the order of `objectFieldLayout`, `end`
pub const ObjectStateWriter = struct {
    allocator: Allocator,
    bytes: std.ArrayList(u8) = .empty,
    /// Offset of the header of the open field
    start: usize = 0,

    pub fn deinit(self: *ObjectStateWriter) void {
        self.bytes.deinit(self.allocator);
    }

    pub fn begin(self: *ObjectStateWriter, field: ObjectField, handle: u32, item: u32) Allocator.Error!void {
        self.start = self.bytes.items.len;
        const header: ObjectFieldHeader = .{ .field = @intFromEnum(field), .handle = @intCast(handle), .item = @intCast(item), .size = 0 };
        try self.bytes.appendSlice(self.allocator, std.mem.asBytes(&header));
    }

    pub fn end(self: *ObjectStateWriter) void {
        const size = self.bytes.items.len - self.start - @sizeOf(ObjectFieldHeader);
        std.mem.writeInt(u16, self.bytes.items[self.start + 6 ..][0..2], @intCast(size), .little);
    }

    /// A value of any type that the C++ writer writes as raw bytes (u32, i32, u64, f32, Real, arrays of them)
    pub fn value(self: *ObjectStateWriter, v: anytype) Allocator.Error!void {
        try self.bytes.appendSlice(self.allocator, std.mem.asBytes(&v));
    }

    /// A bool as u32
    pub fn boolean(self: *ObjectStateWriter, v: bool) Allocator.Error!void {
        try self.value(@as(u32, @intFromBool(v)));
    }

    /// A Vec3 (3 floats)
    pub fn vec3(self: *ObjectStateWriter, v: zolt.Vec3) Allocator.Error!void {
        try self.value([3]f32{ v.getX(), v.getY(), v.getZ() });
    }

    /// An RVec3 (3 Real)
    pub fn rvec3(self: *ObjectStateWriter, v: zolt.RVec3) Allocator.Error!void {
        try self.value([3]Real{ v.getX(), v.getY(), v.getZ() });
    }

    /// A Quat (x, y, z, w)
    pub fn quat(self: *ObjectStateWriter, q: zolt.Quat) Allocator.Error!void {
        try self.value([4]f32{ q.getX(), q.getY(), q.getZ(), q.getW() });
    }

    /// A Mat44 (16 floats, column major)
    pub fn mat44(self: *ObjectStateWriter, m: zolt.Mat44) Allocator.Error!void {
        for (0..4) |c| {
            const col = m.getColumn4(@intCast(c));
            try self.value([4]f32{ col.getX(), col.getY(), col.getZ(), col.getW() });
        }
    }

    /// An RMat44: the 3 rotation columns (12 floats) and the translation (3 Real)
    pub fn rmat44(self: *ObjectStateWriter, m: zolt.RMat44) Allocator.Error!void {
        for (0..3) |c| {
            const col = m.getColumn4(@intCast(c));
            try self.value([4]f32{ col.getX(), col.getY(), col.getZ(), col.getW() });
        }
        try self.rvec3(m.getTranslation());
    }

    /// A block of bytes (the `*_save_state` fields)
    pub fn raw(self: *ObjectStateWriter, data: []const u8) Allocator.Error!void {
        try self.bytes.appendSlice(self.allocator, data);
    }
};

/// One listener callback (floats as bits). Records are compared as sorted lists: the order of the callbacks depends
/// on the threads (the character records are compared in call order, see `RecordKind.isCharacterRecord`).
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

    // Phase 7, one blocker per class group: supported as soon as the Zolt classes the scenes need are ported (their
    // stub markers `is_stub` are gone, `Phase7Classes`), nothing to flip by hand. SimulationParity's runEntry fails a
    // supported scene whose slot (CharacterSlot, CharacterVirtualSlot, ...) is still a stub.
    /// Character
    characters,
    /// CharacterVirtual (with CharacterVsCharacterCollisionSimple and the CharacterContactListener)
    character_virtuals,
    /// Ragdoll (needs the Skeleton files too, which RagdollSettings uses)
    ragdolls,
    /// VehicleConstraint with the wheeled controller, WheelSettingsWV and the three collision testers
    wheeled_vehicles,
    /// VehicleConstraint with the tracked controller and WheelSettingsTV (and the collision testers)
    tracked_vehicles,
    /// VehicleConstraint with the motorcycle controller (a wheeled controller)
    motorcycles,

    /// True when Zolt can run the scenes with this blocker (flip when the wave lands)
    pub fn isSupported(self: Blocker) bool {
        return switch (self) {
            .characters => Phase7Classes.characters(),
            .character_virtuals => Phase7Classes.characterVirtuals(),
            .ragdolls => Phase7Classes.ragdolls(),
            .wheeled_vehicles => Phase7Classes.wheeledVehicles(),
            .tracked_vehicles => Phase7Classes.trackedVehicles(),
            .motorcycles => Phase7Classes.motorcycles(),
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

/// Which Phase 7 classes are ported: a class is ported when its stub marker `is_stub` is gone (the stub files of the
/// foundation declare it on every type the scenes need)
pub const Phase7Classes = struct {
    fn ported(comptime T: type) bool {
        return !@hasDecl(T, "is_stub");
    }

    pub fn characters() bool {
        return ported(zolt.CharacterSettings);
    }

    pub fn characterVirtuals() bool {
        return ported(zolt.CharacterVirtualSettings);
    }

    pub fn ragdolls() bool {
        return ported(zolt.RagdollSettings);
    }

    /// VehicleConstraint and the collision testers (the vehicle core group)
    pub fn vehicleCore() bool {
        return ported(zolt.VehicleConstraintSettings) and ported(zolt.VehicleCollisionTesterRay) and ported(zolt.VehicleCollisionTesterCastSphere) and ported(zolt.VehicleCollisionTesterCastCylinder);
    }

    pub fn wheeledVehicles() bool {
        return vehicleCore() and ported(zolt.WheeledVehicleControllerSettings) and ported(zolt.WheelSettingsWV);
    }

    pub fn trackedVehicles() bool {
        return vehicleCore() and ported(zolt.TrackedVehicleControllerSettings) and ported(zolt.WheelSettingsTV);
    }

    pub fn motorcycles() bool {
        return wheeledVehicles() and ported(zolt.MotorcycleControllerSettings);
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

    // Phase 7 (also checked on the C++ side by "SimScenes blocked scenes run on the C++ side")
    /// The CharacterContactListener records of the scene include each of these RecordKinds at least once (bit
    /// `1 << kind`, `recordBit`)
    character_records: u128 = 0,
    /// The characters and character virtuals reach each of these ground states (bit `1 << EGroundState`) after some
    /// update (from their saved state)
    ground_states: u32 = 0,
    /// Some wheel of a vehicle has a contact after some update
    wheel_contacts: bool = false,
};

/// The bit of a RecordKind in `Expect.character_records`
pub fn recordBit(kind: RecordKind) u128 {
    return @as(u128, 1) << @intCast(@intFromEnum(kind));
}

/// The bit of a ground state (CharacterBase::EGroundState: 0 OnGround, 1 OnSteepGround, 2 NotSupported, 3 InAir) in
/// `Expect.ground_states`
pub fn groundStateBit(state: u32) u32 {
    return @as(u32, 1) << @intCast(state);
}

/// CharacterBase::EGroundState values
pub const GroundState = struct {
    pub const on_ground: u32 = 0;
    pub const on_steep_ground: u32 = 1;
    pub const not_supported: u32 = 2;
    pub const in_air: u32 = 3;
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
    .{ .name = "constraints_distance_limits_and_springs", .build = buildDistanceLimitsAndSprings, .blocked_by = .distance_constraints, .sweep_stride = 15, .expect = .{ .active_constraints = 9 } },
    .{ .name = "constraints_pulley_lengths", .build = buildPulleyLengths, .blocked_by = .pulley_constraints, .sweep_stride = 15, .expect = .{ .active_constraints = 7 } },
    .{ .name = "constraints_distance_pulley_islands_and_contacts", .build = buildDistancePulleyIslands, .blocked_by = .pulley_constraints, .sweep_stride = 15, .expect = .{ .contacts = true, .removed = true, .active_constraints = 8 } },
    .{ .name = "constraints_distance_net_splitter", .build = buildDistanceNet, .blocked_by = .pulley_constraints, .sweep_stride = 40, .expect = .{ .contacts = true, .sleeping = true, .large_island = true, .active_constraints = 262 } },
    .{ .name = "constraints_distance_pulley_sleeping_bodies", .build = buildDistancePulleySleeping, .blocked_by = .pulley_constraints, .sweep_stride = 10, .expect = .{ .sleeping = true, .update_activations = 2, .active_constraints = 4 } },
    .{ .name = "constraints_distance_pulley_save_restore", .build = buildDistancePulleySaveRestore, .blocked_by = .pulley_constraints, .sweep_stride = 15, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_distance_pulley_collision_steps_2", .build = buildDistancePulleyCollisionSteps(2), .blocked_by = .pulley_constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_distance_pulley_collision_steps_3", .build = buildDistancePulleyCollisionSteps(3), .blocked_by = .pulley_constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_distance_pulley_far_from_origin", .build = buildDistancePulleyFarFromOrigin, .blocked_by = .pulley_constraints, .sweep_stride = 20, .expect = .{ .contacts = true, .active_constraints = 6 } },
    .{ .name = "constraints_distance_pulley_random_1", .build = buildDistancePulleyRandom(1), .blocked_by = .pulley_constraints, .sweep_stride = 20, .expect = .{ .contacts = true, .active_constraints = 19 } },
    .{ .name = "constraints_distance_pulley_random_2", .build = buildDistancePulleyRandom(2), .blocked_by = .pulley_constraints, .sweep_stride = 20, .expect = .{ .contacts = true, .active_constraints = 16 } },
    // (append the distance / pulley scenes here)
    // ---- Hinge ----
    .{ .name = "constraints_hinge_ops", .build = buildHingeOps, .blocked_by = .hinge_constraints, .expect = .{ .active_constraints = 3 } },
    .{ .name = "constraints_hinge_limits_and_springs", .build = buildHingeLimits, .blocked_by = .hinge_constraints, .sweep_stride = 20, .expect = .{ .active_constraints = 10 } },
    .{ .name = "constraints_hinge_motors", .build = buildHingeMotors, .blocked_by = .hinge_constraints, .sweep_stride = 20, .expect = .{ .active_constraints = 9 } },
    .{ .name = "constraints_hinge_car", .build = buildHingeCar, .blocked_by = .hinge_constraints, .sweep_stride = 30, .expect = .{ .contacts = true, .removed = true, .active_constraints = 9 } },
    .{ .name = "constraints_hinge_mat_splitter", .build = buildHingeMatSplitter, .blocked_by = .hinge_constraints, .sweep_stride = 40, .expect = .{ .contacts = true, .sleeping = true, .large_island = true, .active_constraints = 155 } },
    .{ .name = "constraints_hinge_sleep_and_save_restore", .build = buildHingeSleepAndSaveRestore, .blocked_by = .hinge_constraints, .sweep_stride = 10, .expect = .{ .sleeping = true, .update_activations = 1, .active_constraints = 3 } },
    .{ .name = "constraints_hinge_collision_steps_2", .build = buildHingeCollisionSteps(2), .blocked_by = .hinge_constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_hinge_collision_steps_3", .build = buildHingeCollisionSteps(3), .blocked_by = .hinge_constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_hinge_far_from_origin", .build = buildHingeFarFromOrigin, .blocked_by = .hinge_constraints, .sweep_stride = 20, .expect = .{ .contacts = true, .active_constraints = 6 } },
    .{ .name = "constraints_hinge_random_1", .build = buildHingeRandom(1), .blocked_by = .hinge_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_hinge_random_2", .build = buildHingeRandom(2), .blocked_by = .hinge_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_hinge_random_3", .build = buildHingeRandom(3), .blocked_by = .hinge_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    // (append the hinge scenes here)
    // ---- Slider ----
    .{ .name = "constraints_slider_ops", .build = buildSliderOps, .blocked_by = .slider_constraints, .expect = .{ .active_constraints = 3 } },
    .{ .name = "constraints_slider_limits_and_springs", .build = buildSliderLimitsAndSprings, .blocked_by = .slider_constraints, .sweep_stride = 15, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_slider_motors", .build = buildSliderMotors, .blocked_by = .slider_constraints, .sweep_stride = 15, .expect = .{ .active_constraints = 11 } },
    .{ .name = "constraints_slider_islands_and_contacts", .build = buildSliderIslands, .blocked_by = .slider_constraints, .sweep_stride = 15, .expect = .{ .contacts = true, .active_constraints = 11 } },
    .{ .name = "constraints_slider_net_splitter", .build = buildSliderNet, .blocked_by = .slider_constraints, .sweep_stride = 40, .expect = .{ .contacts = true, .sleeping = true, .large_island = true } },
    .{ .name = "constraints_slider_sleeping_bodies", .build = buildSliderSleeping, .blocked_by = .slider_constraints, .sweep_stride = 10, .expect = .{ .sleeping = true, .update_activations = 1 } },
    .{ .name = "constraints_slider_save_restore", .build = buildSliderSaveRestore, .blocked_by = .slider_constraints, .sweep_stride = 15, .expect = .{ .active_constraints = 5 } },
    .{ .name = "constraints_slider_collision_steps_2", .build = buildSliderCollisionSteps(2), .blocked_by = .slider_constraints, .sweep_stride = 10 },
    .{ .name = "constraints_slider_collision_steps_3", .build = buildSliderCollisionSteps(3), .blocked_by = .slider_constraints, .sweep_stride = 10 },
    .{ .name = "constraints_slider_far_from_origin", .build = buildSliderFarFromOrigin, .blocked_by = .slider_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_slider_random_1", .build = buildSliderRandom(1), .blocked_by = .slider_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_slider_random_2", .build = buildSliderRandom(2), .blocked_by = .slider_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    // (append the slider scenes here)
    // ---- Cone / SwingTwist ----
    .{ .name = "constraints_cone_ops", .build = buildConeOps, .blocked_by = .cone_constraints, .expect = .{ .active_constraints = 2 } },
    .{ .name = "constraints_swing_twist_ops", .build = buildSwingTwistOps, .blocked_by = .swing_twist_constraints, .expect = .{ .active_constraints = 2 } },
    .{ .name = "constraints_swing_twist_ragdoll_pile", .build = buildSwingTwistRagdollPile, .blocked_by = .swing_twist_constraints, .sweep_stride = 50, .expect = .{ .contacts = true, .active_constraints = 88 } },
    .{ .name = "constraints_swing_twist_ragdolls_lying_splitter", .build = buildSwingTwistRagdollsLying, .blocked_by = .swing_twist_constraints, .sweep_stride = 40, .expect = .{ .contacts = true, .sleeping = true, .large_island = true, .active_constraints = 95 } },
    .{ .name = "constraints_swing_twist_motors", .build = buildSwingTwistMotors, .blocked_by = .swing_twist_constraints, .sweep_stride = 20, .expect = .{ .active_constraints = 11 } },
    .{ .name = "constraints_cone_swing_twist_save_restore", .build = buildConeSwingTwistSaveRestore, .blocked_by = .swing_twist_constraints, .sweep_stride = 15, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_cone_swing_twist_sleeping", .build = buildConeSwingTwistSleeping, .blocked_by = .swing_twist_constraints, .sweep_stride = 10, .expect = .{ .sleeping = true, .update_activations = 1, .active_constraints = 5 } },
    .{ .name = "constraints_cone_swing_twist_collision_steps_2", .build = buildConeSwingTwistCollisionSteps(2), .blocked_by = .swing_twist_constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_cone_swing_twist_collision_steps_3", .build = buildConeSwingTwistCollisionSteps(3), .blocked_by = .swing_twist_constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_cone_swing_twist_far_from_origin", .build = buildConeSwingTwistFarFromOrigin, .blocked_by = .swing_twist_constraints, .sweep_stride = 20, .expect = .{ .contacts = true, .active_constraints = 8 } },
    .{ .name = "constraints_cone_swing_twist_random_1", .build = buildConeSwingTwistRandom(1), .blocked_by = .swing_twist_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_cone_swing_twist_random_2", .build = buildConeSwingTwistRandom(2), .blocked_by = .swing_twist_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
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
    .{ .name = "constraints_path_limits_and_friction", .build = buildPathLimitsAndFriction, .blocked_by = .path_constraints, .sweep_stride = 20, .expect = .{ .active_constraints = 5 } },
    .{ .name = "constraints_path_motors", .build = buildPathMotors, .blocked_by = .path_constraints, .sweep_stride = 20, .expect = .{ .active_constraints = 10 } },
    .{ .name = "constraints_path_rotation_types", .build = buildPathRotationTypes, .blocked_by = .path_constraints, .sweep_stride = 20, .expect = .{ .active_constraints = 7, .update_activations = 1 } },
    .{ .name = "constraints_path_train_with_contacts", .build = buildPathTrainWithContacts, .blocked_by = .path_constraints, .sweep_stride = 30, .expect = .{ .contacts = true, .active_constraints = 12 } },
    .{ .name = "constraints_path_net_splitter", .build = buildPathNetSplitter, .blocked_by = .path_constraints, .sweep_stride = 40, .expect = .{ .contacts = true, .sleeping = true, .large_island = true } },
    .{ .name = "constraints_path_sleep_and_wake", .build = buildPathSleepAndWake, .blocked_by = .path_constraints, .sweep_stride = 20, .expect = .{ .sleeping = true, .update_activations = 1 } },
    .{ .name = "constraints_path_save_restore", .build = buildPathSaveRestore, .blocked_by = .path_constraints, .sweep_stride = 20, .expect = .{ .active_constraints = 7 } },
    .{ .name = "constraints_path_collision_steps_2", .build = buildPathCollisionSteps(2), .blocked_by = .path_constraints, .sweep_stride = 10 },
    .{ .name = "constraints_path_collision_steps_3", .build = buildPathCollisionSteps(3), .blocked_by = .path_constraints, .sweep_stride = 10 },
    .{ .name = "constraints_path_far_from_origin", .build = buildPathFarFromOrigin, .blocked_by = .path_constraints, .sweep_stride = 20, .expect = .{ .contacts = true, .active_constraints = 3 } },
    // (append the path scenes here)
    // ---- Gear / RackAndPinion ----
    .{ .name = "constraints_gear_ops", .build = buildGearOps, .blocked_by = .gear_constraints, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_rack_and_pinion_ops", .build = buildRackAndPinionOps, .blocked_by = .rack_and_pinion_constraints, .expect = .{ .active_constraints = 6 } },
    .{ .name = "constraints_gear_motors", .build = buildGearMotors, .blocked_by = .gear_constraints, .sweep_stride = 30, .expect = .{ .active_constraints = 32 } },
    .{ .name = "constraints_rack_and_pinion_motors", .build = buildRackAndPinionMotors, .blocked_by = .rack_and_pinion_constraints, .sweep_stride = 30, .expect = .{ .active_constraints = 27 } },
    .{ .name = "constraints_gear_islands_and_contacts", .build = buildGearIslands, .blocked_by = .rack_and_pinion_constraints, .sweep_stride = 20, .expect = .{ .contacts = true, .active_constraints = 14 } },
    .{ .name = "constraints_gear_splitter", .build = buildGearSplitter, .blocked_by = .gear_constraints, .sweep_stride = 40, .expect = .{ .sleeping = true, .large_island = true, .update_activations = 1, .active_constraints = 139 } },
    .{ .name = "constraints_gear_sleep_and_save_restore", .build = buildGearSleepAndSaveRestore, .blocked_by = .rack_and_pinion_constraints, .sweep_stride = 10, .expect = .{ .sleeping = true, .update_activations = 1, .active_constraints = 9 } },
    .{ .name = "constraints_gear_collision_steps_2", .build = buildGearCollisionSteps(2), .blocked_by = .rack_and_pinion_constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 8 } },
    .{ .name = "constraints_gear_collision_steps_3", .build = buildGearCollisionSteps(3), .blocked_by = .rack_and_pinion_constraints, .sweep_stride = 10, .expect = .{ .active_constraints = 8 } },
    .{ .name = "constraints_gear_far_from_origin", .build = buildGearFarFromOrigin, .blocked_by = .rack_and_pinion_constraints, .sweep_stride = 20, .expect = .{ .contacts = true, .active_constraints = 6 } },
    .{ .name = "constraints_gear_random_1", .build = buildGearRandom(1), .blocked_by = .rack_and_pinion_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_gear_random_2", .build = buildGearRandom(2), .blocked_by = .rack_and_pinion_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_gear_random_4", .build = buildGearRandom(4), .blocked_by = .rack_and_pinion_constraints, .sweep_stride = 20, .expect = .{ .contacts = true } },
    .{ .name = "constraints_gear_kinematic_release", .build = buildGearKinematicRelease, .blocked_by = .release_only, .expect = .{ .active_constraints = 4 } },
    // (append the gear / rack and pinion scenes here)

    // Characters, ragdolls and vehicles (Phase 7), one region per worker group (append a group's scenes at the end of its
    // region only)
    // ---- Character ----
    .{ .name = "character_ops", .build = buildCharacterOps, .blocked_by = .characters, .sweep_stride = 20, .expect = .{ .contacts = true, .ground_states = all_ground_states } },
    .{ .name = "character_ground_states_boxes", .build = buildCharacterGroundStatesBoxes, .blocked_by = .characters, .sweep_stride = 25, .expect = .{ .contacts = true, .ground_states = all_ground_states } },
    .{ .name = "character_ground_states_meshes", .build = buildCharacterGroundStatesMeshes, .blocked_by = .characters, .sweep_stride = 25, .expect = .{ .contacts = true, .ground_states = all_ground_states } },
    .{ .name = "character_moving_platforms", .build = buildCharacterMovingPlatforms, .blocked_by = .characters, .sweep_stride = 25, .expect = .{ .contacts = true, .ground_states = groundStateBit(GroundState.on_ground) | groundStateBit(GroundState.in_air) } },
    .{ .name = "character_crouch_stand", .build = buildCharacterCrouchStand, .blocked_by = .characters, .sweep_stride = 20, .expect = .{ .contacts = true, .ground_states = groundStateBit(GroundState.on_ground) } },
    .{ .name = "character_layers_sleep_wake", .build = buildCharacterLayersSleepWake, .blocked_by = .characters, .sweep_stride = 40, .expect = .{ .contacts = true, .sleeping = true, .ground_states = groundStateBit(GroundState.on_ground) | groundStateBit(GroundState.in_air) } },
    .{ .name = "character_far_from_origin", .build = buildCharacterFarFromOrigin, .blocked_by = .characters, .sweep_stride = 30, .expect = .{ .contacts = true, .ground_states = groundStateBit(GroundState.on_ground) | groundStateBit(GroundState.on_steep_ground) | groundStateBit(GroundState.in_air) } },
    .{ .name = "character_many", .build = buildCharacterMany, .blocked_by = .characters, .sweep_stride = 30, .expect = .{ .contacts = true, .ground_states = groundStateBit(GroundState.on_ground) | groundStateBit(GroundState.in_air) } },
    // (append the Character scenes here)
    // ---- CharacterVirtual ----
    // No D12 sweeps in the CharacterVirtual scenes (`sweep_stride = 0`): the swept updates rebuild the Zolt broad phase
    // trees (BroadPhaseQuadTree's UpdatePrepare / UpdateFinalize also run in an update that fails later, every restore of
    // the snapshot marks all bodies as changed), which then get another layout than the C++ trees: the layout depends on
    // the history and is not part of the saved state. A CharacterVirtual query receives its hits in the order of the
    // layout, and Jolt calls OnAdjustBodyVelocity / OnContactValidate (ContactCastCollector) in that order, before it
    // sorts the contacts, so the character records (compared in call order) and the cv_check_collision hashes differ after
    // a sweep (character_virtual_characters with sweeps: record 40 of update 37 adjusts body 300 in Jolt and body
    // 0x00800002 in Zolt, both are hit by the same query). Without sweeps both sides have the same history (restore_cross
    // too: both sides restore).
    .{ .name = "character_virtual_ops", .build = buildCharacterVirtualOps, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .contacts = true, .character_records = character_body_records, .ground_states = all_ground_states } },
    .{ .name = "character_virtual_characters", .build = buildCharacterVirtualCharacters, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .contacts = true, .character_records = character_body_records | character_vs_character_records } },
    .{ .name = "character_virtual_stairs_slopes", .build = buildCharacterVirtualStairsSlopes, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .character_records = character_body_records, .ground_states = groundStateBit(GroundState.on_ground) | groundStateBit(GroundState.on_steep_ground) | groundStateBit(GroundState.in_air) } },
    .{ .name = "character_virtual_platforms", .build = buildCharacterVirtualPlatforms, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .character_records = character_body_contact_records, .ground_states = groundStateBit(GroundState.on_ground) } },
    .{ .name = "character_virtual_crowd", .build = buildCharacterVirtualCrowd, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .character_records = character_body_contact_records | character_vs_character_records, .ground_states = groundStateBit(GroundState.on_ground) } },
    .{ .name = "character_virtual_inner_body", .build = buildCharacterVirtualInnerBody, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .contacts = true, .character_records = character_body_records, .ground_states = groundStateBit(GroundState.on_ground) } },
    .{ .name = "character_virtual_contact_limits", .build = buildCharacterVirtualContactLimits, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .character_records = character_body_contact_records, .ground_states = groundStateBit(GroundState.on_ground) } },
    .{ .name = "character_virtual_penetration", .build = buildCharacterVirtualPenetration, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .character_records = character_body_contact_records, .ground_states = groundStateBit(GroundState.on_ground) } },
    .{ .name = "character_virtual_set_shape", .build = buildCharacterVirtualSetShape, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .character_records = character_body_contact_records, .ground_states = groundStateBit(GroundState.on_ground) } },
    .{ .name = "character_virtual_internal_edges", .build = buildCharacterVirtualInternalEdges, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .character_records = character_body_records, .ground_states = groundStateBit(GroundState.on_ground) } },
    .{ .name = "character_virtual_terrain", .build = buildCharacterVirtualTerrain, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .character_records = character_body_contact_records, .ground_states = groundStateBit(GroundState.on_ground) | groundStateBit(GroundState.in_air) } },
    .{ .name = "character_virtual_cross_restore", .build = buildCharacterVirtualCrossRestore, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .contacts = true, .character_records = character_body_contact_records | character_vs_character_records, .ground_states = groundStateBit(GroundState.on_ground) | groundStateBit(GroundState.in_air) } },
    .{ .name = "character_virtual_far_from_origin", .build = buildCharacterVirtualFarFromOrigin, .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .character_records = character_body_contact_records | character_vs_character_records, .ground_states = groundStateBit(GroundState.on_ground) } },
    .{ .name = "character_virtual_listener_none", .build = buildCharacterVirtualListener(.none), .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .contacts = true, .ground_states = groundStateBit(GroundState.on_ground) } },
    .{ .name = "character_virtual_listener_record", .build = buildCharacterVirtualListener(.record), .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .contacts = true, .character_records = character_body_records | character_vs_character_records, .ground_states = groundStateBit(GroundState.on_ground) } },
    .{ .name = "character_virtual_listener_modify", .build = buildCharacterVirtualListener(.modify), .blocked_by = .character_virtuals, .sweep_stride = 0, .expect = .{ .contacts = true, .character_records = character_body_records | character_vs_character_records, .ground_states = groundStateBit(GroundState.on_ground) } },
    // PerformanceTest's CharacterVirtualScene (milestone M8), exactly as ZoltPerformanceTest sets it up
    .{ .name = "perf_character_virtual", .build = buildPerfCharacterVirtual, .blocked_by = .character_virtuals, .sweep_stride = 0, .creation_sweep = false, .expect = .{ .contacts = true, .character_records = character_body_contact_records, .ground_states = groundStateBit(GroundState.on_ground) } },
    // (append the CharacterVirtual scenes here)
    // ---- Ragdoll ----
    .{ .name = "ragdoll_ops", .build = buildRagdollOps, .blocked_by = .ragdolls, .sweep_stride = 30, .expect = .{ .contacts = true, .active_constraints = 10 } },
    // (append the Ragdoll scenes here)
    // ---- Vehicles ----
    .{ .name = "vehicle_wheeled_ops", .build = buildVehicleWheeledOps, .blocked_by = .wheeled_vehicles, .sweep_stride = 25, .expect = .{ .wheel_contacts = true, .active_constraints = 1 } },
    .{ .name = "vehicle_tracked_ops", .build = buildVehicleTrackedOps, .blocked_by = .tracked_vehicles, .sweep_stride = 25, .expect = .{ .wheel_contacts = true, .active_constraints = 1 } },
    .{ .name = "vehicle_motorcycle_ops", .build = buildVehicleMotorcycleOps, .blocked_by = .motorcycles, .sweep_stride = 25, .expect = .{ .wheel_contacts = true, .active_constraints = 1 } },
    .{ .name = "vehicle_motorcycle_lean_integration", .build = buildVehicleMotorcycleLeanIntegration, .blocked_by = .motorcycles, .sweep_stride = 0, .expect = .{ .wheel_contacts = true, .active_constraints = 1 } },
    .{ .name = "vehicle_wheeled_in_air", .build = buildVehicleWheeledInAir, .blocked_by = .wheeled_vehicles, .sweep_stride = 15, .expect = .{ .wheel_contacts = true, .active_constraints = 1 } },
    .{ .name = "vehicle_wheeled_terrains", .build = buildVehicleWheeledTerrains, .blocked_by = .wheeled_vehicles, .sweep_stride = 35, .expect = .{ .wheel_contacts = true, .active_constraints = 3 } },
    .{ .name = "vehicle_wheeled_driving", .build = buildVehicleWheeledDriving, .blocked_by = .wheeled_vehicles, .sweep_stride = 40, .expect = .{ .wheel_contacts = true, .active_constraints = 4 } },
    .{ .name = "vehicle_wheeled_pushing_boxes", .build = buildVehicleWheeledPushingBoxes, .blocked_by = .wheeled_vehicles, .sweep_stride = 80, .expect = .{ .contacts = true, .sleeping = true, .large_island = true, .wheel_contacts = true, .active_constraints = 1, .update_activations = 80 } },
    .{ .name = "vehicle_wheeled_sleep_and_wake", .build = buildVehicleWheeledSleepAndWake, .blocked_by = .wheeled_vehicles, .sweep_stride = 30, .expect = .{ .sleeping = true, .wheel_contacts = true, .active_constraints = 1 } },
    .{ .name = "vehicle_wheeled_far_from_origin", .build = buildVehicleWheeledFarFromOrigin, .blocked_by = .wheeled_vehicles, .sweep_stride = 25, .expect = .{ .wheel_contacts = true, .active_constraints = 3 } },
    // The vehicle variants group (tracked vehicles and motorcycles)
    .{ .name = "vehicle_tracked_terrains", .build = buildVehicleTrackedTerrains, .blocked_by = .tracked_vehicles, .sweep_stride = 35, .expect = .{ .wheel_contacts = true, .active_constraints = 2 } },
    .{ .name = "vehicle_tracked_layouts", .build = buildVehicleTrackedLayouts, .blocked_by = .tracked_vehicles, .sweep_stride = 30, .expect = .{ .wheel_contacts = true, .active_constraints = 3 } },
    .{ .name = "vehicle_tracked_pushing_boxes", .build = buildVehicleTrackedPushingBoxes, .blocked_by = .tracked_vehicles, .sweep_stride = 75, .expect = .{ .contacts = true, .sleeping = true, .large_island = true, .wheel_contacts = true, .active_constraints = 1, .update_activations = 30 } },
    .{ .name = "vehicle_tracked_sleep_and_wake", .build = buildVehicleTrackedSleepAndWake, .blocked_by = .tracked_vehicles, .sweep_stride = 30, .expect = .{ .sleeping = true, .wheel_contacts = true, .active_constraints = 1 } },
    .{ .name = "vehicle_tracked_far_from_origin", .build = buildVehicleTrackedFarFromOrigin, .blocked_by = .tracked_vehicles, .sweep_stride = 25, .expect = .{ .wheel_contacts = true, .active_constraints = 2 } },
    .{ .name = "vehicle_motorcycle_turns", .build = buildVehicleMotorcycleTurns, .blocked_by = .motorcycles, .sweep_stride = 40, .expect = .{ .wheel_contacts = true, .active_constraints = 3 } },
    .{ .name = "vehicle_motorcycle_slopes_and_braking", .build = buildVehicleMotorcycleSlopesAndBraking, .blocked_by = .motorcycles, .sweep_stride = 30, .expect = .{ .wheel_contacts = true, .active_constraints = 2 } },
    .{ .name = "vehicle_motorcycle_lean_integration_slope", .build = buildVehicleMotorcycleLeanIntegrationSlope, .blocked_by = .motorcycles, .sweep_stride = 0, .expect = .{ .wheel_contacts = true, .active_constraints = 2 } },
    .{ .name = "vehicle_motorcycle_far_from_origin", .build = buildVehicleMotorcycleFarFromOrigin, .blocked_by = .motorcycles, .sweep_stride = 25, .expect = .{ .wheel_contacts = true, .active_constraints = 2 } },
    // (append the vehicle scenes here)
};

/// Expect.ground_states: every EGroundState
const all_ground_states = groundStateBit(GroundState.on_ground) | groundStateBit(GroundState.on_steep_ground) | groundStateBit(GroundState.not_supported) | groundStateBit(GroundState.in_air);

/// Expect.character_records: the callbacks of a character virtual that touches bodies
const character_body_records = recordBit(.character_contact_validate) | recordBit(.character_contact_added) | recordBit(.character_contact_persisted) | recordBit(.character_contact_removed) | recordBit(.character_contact_solve) | recordBit(.character_adjust_body_velocity);

/// Expect.character_records: the callbacks of character virtuals that touch each other
const character_vs_character_records = recordBit(.character_vs_character_contact_validate) | recordBit(.character_vs_character_contact_added) | recordBit(.character_vs_character_contact_persisted) | recordBit(.character_vs_character_contact_removed) | recordBit(.character_vs_character_contact_solve);
/// The CharacterContactListener callbacks of the body contacts that every character on a floor gets (no OnContactRemoved)
const character_body_contact_records = recordBit(.character_contact_validate) | recordBit(.character_contact_added) | recordBit(.character_contact_persisted) | recordBit(.character_contact_solve) | recordBit(.character_adjust_body_velocity);

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
    // Phase 7
    characters: std.ArrayList(SimCharacter) = .empty,
    character_virtuals: std.ArrayList(SimCharacterVirtual) = .empty,
    ragdolls: std.ArrayList(SimRagdoll) = .empty,
    ragdoll_joints: std.ArrayList(SimRagdollJoint) = .empty,
    ragdoll_additional_constraints: std.ArrayList(SimRagdollAdditionalConstraint) = .empty,
    ragdoll_poses: std.ArrayList(SimRagdollPose) = .empty,
    vehicles: std.ArrayList(SimVehicle) = .empty,
    vehicle_wheels: std.ArrayList(SimVehicleWheel) = .empty,
    vehicle_differentials: std.ArrayList(SimVehicleDifferential) = .empty,
    vehicle_anti_roll_bars: std.ArrayList(SimVehicleAntiRollBar) = .empty,
    uints: std.ArrayList(u32) = .empty,

    /// The update before which the next operations run
    current_update: u32 = 0,
    /// Number of body handles created so far
    num_handles: u32 = 0,
    /// Number of constraint handles created so far (create_constraint, create_constraint_no_add)
    num_constraint_handles: u32 = 0,
    /// Number of character, character virtual, ragdoll and vehicle handles created so far (Phase 7)
    num_character_handles: u32 = 0,
    num_character_virtual_handles: u32 = 0,
    num_ragdoll_handles: u32 = 0,
    num_vehicle_handles: u32 = 0,
    /// The CharacterID that `characterVirtual` gives the next description without an ID
    next_character_id: u32 = 1,
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
        self.characters.deinit(self.allocator);
        self.character_virtuals.deinit(self.allocator);
        self.ragdolls.deinit(self.allocator);
        self.ragdoll_joints.deinit(self.allocator);
        self.ragdoll_additional_constraints.deinit(self.allocator);
        self.ragdoll_poses.deinit(self.allocator);
        self.vehicles.deinit(self.allocator);
        self.vehicle_wheels.deinit(self.allocator);
        self.vehicle_differentials.deinit(self.allocator);
        self.vehicle_anti_roll_bars.deinit(self.allocator);
        self.uints.deinit(self.allocator);
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
            .characters = self.characters.items.ptr,
            .num_characters = @intCast(self.characters.items.len),
            .character_virtuals = self.character_virtuals.items.ptr,
            .num_character_virtuals = @intCast(self.character_virtuals.items.len),
            .ragdolls = self.ragdolls.items.ptr,
            .num_ragdolls = @intCast(self.ragdolls.items.len),
            .ragdoll_joints = self.ragdoll_joints.items.ptr,
            .num_ragdoll_joints = @intCast(self.ragdoll_joints.items.len),
            .ragdoll_additional_constraints = self.ragdoll_additional_constraints.items.ptr,
            .num_ragdoll_additional_constraints = @intCast(self.ragdoll_additional_constraints.items.len),
            .ragdoll_poses = self.ragdoll_poses.items.ptr,
            .num_ragdoll_poses = @intCast(self.ragdoll_poses.items.len),
            .vehicles = self.vehicles.items.ptr,
            .num_vehicles = @intCast(self.vehicles.items.len),
            .vehicle_wheels = self.vehicle_wheels.items.ptr,
            .num_vehicle_wheels = @intCast(self.vehicle_wheels.items.len),
            .vehicle_differentials = self.vehicle_differentials.items.ptr,
            .num_vehicle_differentials = @intCast(self.vehicle_differentials.items.len),
            .vehicle_anti_roll_bars = self.vehicle_anti_roll_bars.items.ptr,
            .num_vehicle_anti_roll_bars = @intCast(self.vehicle_anti_roll_bars.items.len),
            .uints = self.uints.items.ptr,
            .num_uints = @intCast(self.uints.items.len),
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
    // Characters, ragdolls and vehicles (Phase 7)

    pub const ObjectOpArgs = struct {
        /// A body handle (cv_has_collided_with_body, CharacterBodyFilter.ignore_body, vehicle_tester_set_filters)
        body: u32 = invalid_handle,
        /// The expected result of the `expect_*` operations
        count: u32 = 0,
        activation: Activation = .activate,
        i: []const u32 = &.{},
        f: []const f32 = &.{},
        r: [3]Real = @splat(0),
    };

    /// Add an operation (OpKind 400..599) on object handle `handle` (a character, character virtual, ragdoll or vehicle
    /// handle; the description index for the create operations)
    pub fn objectOp(self: *SceneBuilder, kind: OpKind, handle: u32, args: ObjectOpArgs) Allocator.Error!void {
        std.debug.assert(Phase7OpRange.of(@intFromEnum(kind)) != null);
        var o: SimOp = .{ .update = self.current_update, .kind = @intFromEnum(kind), .body = args.body, .count = args.count, .activation = @intFromEnum(args.activation), .index = handle, .r = args.r };
        @memcpy(o.i[0..args.i.len], args.i);
        @memcpy(o.f[0..args.f.len], args.f);
        try self.ops.append(self.allocator, o);
    }

    /// The filters of a CharacterVirtual query (CharacterQueryFilter: op.i[0..3] and op.body)
    pub const CharacterQuery = struct {
        layer: u32 = CharacterQueryFilter.no_layer_filter,
        body_filter: CharacterBodyFilter = .all,
        shape_filter: CharacterShapeFilter = .all,
        /// The body handle of CharacterBodyFilter.ignore_body
        body: u32 = invalid_handle,
    };

    /// A CharacterVirtual query operation (cv_update, cv_extended_update, ...) with its filters, `i3` is op.i[3] (the shape
    /// of cv_set_shape / cv_check_collision)
    pub fn characterVirtualQuery(self: *SceneBuilder, kind: OpKind, handle: u32, query: CharacterQuery, args: struct { f: []const f32 = &.{}, r: [3]Real = @splat(0), i3: u32 = 0 }) Allocator.Error!void {
        return self.objectOp(kind, handle, .{ .body = query.body, .i = &.{ query.layer, @intFromEnum(query.body_filter), @intFromEnum(query.shape_filter), args.i3 }, .f = args.f, .r = args.r });
    }

    /// CharacterVirtual::ExtendedUpdateSettings (Jolt's defaults; null: Cos(DegreesToRadians(75.0f)))
    pub const ExtendedUpdateSettings = struct {
        stick_to_floor_step_down: [3]f32 = .{ 0, -0.5, 0 },
        walk_stairs_step_up: [3]f32 = .{ 0, 0.4, 0 },
        walk_stairs_min_step_forward: f32 = 0.02,
        walk_stairs_step_forward_test: f32 = 0.15,
        walk_stairs_cos_angle_forward_contact: ?f32 = null,
        walk_stairs_step_down_extra: [3]f32 = @splat(0),

        /// The 12 floats of f[4..16] of cv_extended_update
        pub fn toFloats(self: ExtendedUpdateSettings) [12]f32 {
            const cos_angle = self.walk_stairs_cos_angle_forward_contact orelse zolt.trigonometry.cos(zolt.math.degreesToRadians(75.0));
            return self.stick_to_floor_step_down ++ self.walk_stairs_step_up ++ [_]f32{ self.walk_stairs_min_step_forward, self.walk_stairs_step_forward_test, cos_angle } ++ self.walk_stairs_step_down_extra;
        }
    };

    /// cv_extended_update(delta time, gravity, settings) with the filters of `query`
    pub fn extendedUpdate(self: *SceneBuilder, handle: u32, delta_time: f32, gravity: [3]f32, settings: ExtendedUpdateSettings, query: CharacterQuery) Allocator.Error!void {
        const f = [_]f32{delta_time} ++ gravity ++ settings.toFloats();
        return self.characterVirtualQuery(.cv_extended_update, handle, query, .{ .f = &f });
    }

    /// Add floats to the pool, returns their offset
    pub fn addFloats(self: *SceneBuilder, values: []const f32) Allocator.Error!u32 {
        const offset: u32 = @intCast(self.floats.items.len);
        try self.floats.appendSlice(self.allocator, values);
        return offset;
    }

    /// Add integers to the pool, returns their offset
    pub fn addUints(self: *SceneBuilder, values: []const u32) Allocator.Error!u32 {
        const offset: u32 = @intCast(self.uints.items.len);
        try self.uints.appendSlice(self.allocator, values);
        return offset;
    }

    /// Add a character description and create the character (create_character), returns its character handle
    pub fn character(self: *SceneBuilder, desc_value: SimCharacter) Allocator.Error!u32 {
        try self.characters.append(self.allocator, desc_value);
        try self.objectOp(.create_character, @intCast(self.characters.items.len - 1), .{});
        self.num_character_handles += 1;
        return self.num_character_handles - 1;
    }

    /// Add a character virtual description (without an ID it gets the scene's next CharacterID) and create the character
    /// (create_character_virtual), returns its character virtual handle
    pub fn characterVirtual(self: *SceneBuilder, desc_value: SimCharacterVirtual) Allocator.Error!u32 {
        var d = desc_value;
        if (d.id == invalid_handle) {
            d.id = self.next_character_id;
            self.next_character_id += 1;
        }
        try self.character_virtuals.append(self.allocator, d);
        try self.objectOp(.create_character_virtual, @intCast(self.character_virtuals.items.len - 1), .{});
        self.num_character_virtual_handles += 1;
        return self.num_character_virtual_handles - 1;
    }

    /// Add a ragdoll description with its joints and additional constraints (`first_joint` / `first_additional` and the
    /// counts are filled in), returns the description index
    pub fn addRagdoll(self: *SceneBuilder, desc_value: SimRagdoll, joints: []const SimRagdollJoint, additional: []const SimRagdollAdditionalConstraint) Allocator.Error!u32 {
        var d = desc_value;
        d.first_joint = @intCast(self.ragdoll_joints.items.len);
        d.num_joints = @intCast(joints.len);
        d.first_additional = @intCast(self.ragdoll_additional_constraints.items.len);
        d.num_additional = @intCast(additional.len);
        try self.ragdoll_joints.appendSlice(self.allocator, joints);
        try self.ragdoll_additional_constraints.appendSlice(self.allocator, additional);
        try self.ragdolls.append(self.allocator, d);
        return @intCast(self.ragdolls.items.len - 1);
    }

    /// Create a ragdoll of description `desc_index` (create_ragdoll), returns its ragdoll handle
    pub fn createRagdoll(self: *SceneBuilder, desc_index: u32) Allocator.Error!u32 {
        try self.objectOp(.create_ragdoll, desc_index, .{});
        self.num_ragdoll_handles += 1;
        return self.num_ragdoll_handles - 1;
    }

    /// Add a pose of a ragdoll skeleton: the root offset and per joint the translation and rotation (x, y, z, x, y, z,
    /// w), returns the pose index
    pub fn addRagdollPose(self: *SceneBuilder, root_offset: [3]Real, joints: []const [7]f32) Allocator.Error!u32 {
        const data: u32 = @intCast(self.floats.items.len);
        for (joints) |j| try self.floats.appendSlice(self.allocator, &j);
        try self.ragdoll_poses.append(self.allocator, .{ .root_offset = root_offset, .data = data, .count = @intCast(joints.len) });
        return @intCast(self.ragdoll_poses.items.len - 1);
    }

    /// Add a vehicle description with its wheels, differentials and anti-roll bars (the first / num members are filled
    /// in), returns the description index
    pub fn addVehicle(self: *SceneBuilder, desc_value: SimVehicle, wheels: []const SimVehicleWheel, differentials: []const SimVehicleDifferential, anti_roll_bars: []const SimVehicleAntiRollBar) Allocator.Error!u32 {
        var d = desc_value;
        d.first_wheel = @intCast(self.vehicle_wheels.items.len);
        d.num_wheels = @intCast(wheels.len);
        d.first_differential = @intCast(self.vehicle_differentials.items.len);
        d.num_differentials = @intCast(differentials.len);
        d.first_anti_roll_bar = @intCast(self.vehicle_anti_roll_bars.items.len);
        d.num_anti_roll_bars = @intCast(anti_roll_bars.len);
        try self.vehicle_wheels.appendSlice(self.allocator, wheels);
        try self.vehicle_differentials.appendSlice(self.allocator, differentials);
        try self.vehicle_anti_roll_bars.appendSlice(self.allocator, anti_roll_bars);
        try self.vehicles.append(self.allocator, d);
        return @intCast(self.vehicles.items.len - 1);
    }

    /// Create the vehicle of description `desc_index` (create_vehicle), returns its vehicle handle
    pub fn createVehicle(self: *SceneBuilder, desc_index: u32) Allocator.Error!u32 {
        try self.objectOp(.create_vehicle, desc_index, .{});
        self.num_vehicle_handles += 1;
        return self.num_vehicle_handles - 1;
    }

    /// Check: the ground state of character handle `handle` (GroundState) at this point
    pub fn expectCharacterGroundState(self: *SceneBuilder, handle: u32, state: u32) Allocator.Error!void {
        return self.objectOp(.character_expect_ground_state, handle, .{ .count = state });
    }

    /// Check: the ground state of character virtual handle `handle` (GroundState) at this point
    /// The body of a character must be in BodyState `state` (`BodyState.sleeping`, `BodyState.awake`) at this point: the
    /// driver checks it on both sides
    pub fn expectCharacterBodyState(self: *SceneBuilder, handle: u32, state: u32) Allocator.Error!void {
        return self.objectOp(.character_expect_body_state, handle, .{ .count = state });
    }

    pub fn expectCharacterVirtualGroundState(self: *SceneBuilder, handle: u32, state: u32) Allocator.Error!void {
        return self.objectOp(.cv_expect_ground_state, handle, .{ .count = state });
    }

    /// Check: a value of character virtual handle `handle` (`CharacterVirtualValue`) at this point; `body`: the body
    /// handle of `ground_body_is`
    pub fn expectCharacterVirtualValue(self: *SceneBuilder, handle: u32, value: CharacterVirtualValue, expected: u32, args: struct { body: u32 = invalid_handle }) Allocator.Error!void {
        return self.objectOp(.cv_expect_value, handle, .{ .i = &.{@intFromEnum(value)}, .count = expected, .body = args.body });
    }

    /// Check: Ragdoll::IsActive of ragdoll handle `handle` at this point
    pub fn expectRagdollActive(self: *SceneBuilder, handle: u32, active: bool) Allocator.Error!void {
        return self.objectOp(.ragdoll_expect_active, handle, .{ .count = @intFromBool(active) });
    }

    /// Check: the bits of the wheels of vehicle handle `handle` that have a contact at this point
    pub fn expectWheelContacts(self: *SceneBuilder, handle: u32, bits: u32) Allocator.Error!void {
        return self.objectOp(.vehicle_expect_wheel_contacts, handle, .{ .count = bits });
    }

    /// Check: the ConstraintState bits of vehicle handle `handle` | IsActive(vehicle body) << 2 at this point
    pub fn expectVehicleConstraintState(self: *SceneBuilder, handle: u32, state: u32, body_active: bool) Allocator.Error!void {
        return self.objectOp(.vehicle_expect_constraint_state, handle, .{ .count = state | (@as(u32, @intFromBool(body_active)) << 2) });
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

/// A distance description from body handle `body1` to `body2` (`invalid_handle`: Body::sFixedToWorld) in world space
/// with a limits spring (mode, frequency / stiffness, damping)
fn distanceSpring(body1: u32, body2: u32, point1: [3]Real, point2: [3]Real, min_distance: f32, max_distance: f32, mode: u32, frequency: f32, damping: f32) SimConstraint {
    var c = Desc.distance(body1, body2, point1, point2, min_distance, max_distance);
    Desc.setSpring(&c, 1, 2, mode, frequency, damping);
    return c;
}

/// Distance limits and springs: a box on a rope with a range from the world falls freely until the rope is taut (hard
/// max limit) and swings; a box thrown up towards its pivot on a rod that only pushes (hard min limit); spheres that fall
/// into soft max limits in the three spring modes (frequency, stiffness, mass normalized stiffness); a sphere on a spring
/// with min = max = 0 (TestDistanceSpring, stiffness mode); a box with point 1 = point 2 in world space (a cheap point
/// constraint: the distance is 0, the normal of the constructor); a spinning dumbbell of two dynamic boxes on a single
/// distance without gravity (forces in both directions); a box on a rope in local space with the world as body 2;
/// SetDistance and SetLimitsSpringSettings between the updates (a range becomes a single distance, a hard limit soft, a
/// soft limit hard, the rope gets shorter and longer), NotifyShapeChanged, ResetWarmStart, GetConstraintSettings
fn buildDistanceLimitsAndSprings(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    const ball = try b.sphere(0.2);
    b.num_updates = 120;

    // A rope with a range [0.5, 2]: the box starts 1 below the pivot and falls into the max limit
    const falling = try b.body(.{ .position = .{ 0.3, 9, 0 }, .shape = box, .allow_sleeping = 0 }, .activate);
    const rope = try b.constraint(Desc.distance(invalid_handle, falling, .{ 0, 10, 0 }, .{ 0.3, 9, 0 }, 0.5, 2));

    // A rod that only pushes: the box is thrown up into the min limit
    const thrown = try b.body(.{ .position = .{ 3, 8, 0 }, .shape = box, .linear_velocity = .{ 0.5, 7, 0 }, .allow_sleeping = 0 }, .activate);
    const rod = try b.constraint(Desc.distance(invalid_handle, thrown, .{ 3, 10, 0 }, .{ 3, 8, 0 }, 1, 3));

    // Soft max limits in the three spring modes
    var soft: [3]u32 = undefined;
    const springs = [3][3]f32{ .{ 0, 2, 0.3 }, .{ 1, 300, 4 }, .{ 2, 60, 1 } };
    for (springs, &soft, 0..) |spring, *handle, i| {
        const x: Real = 6 + 2 * real(i);
        const h = try b.body(.{ .position = .{ x, 9, 0 }, .shape = ball, .allow_sleeping = 0 }, .activate);
        handle.* = try b.constraint(distanceSpring(invalid_handle, h, .{ x, 10, 0 }, .{ x, 9, 0 }, 0, 1, @intFromFloat(spring[0]), spring[1], spring[2]));
    }

    // A spring with min = max = 0 in stiffness mode: the sphere starts 2 away from its rest point
    const sprung = try b.body(.{ .position = .{ 14, 9, 0 }, .shape = ball, .linear_damping = 0, .allow_sleeping = 0 }, .activate);
    _ = try b.constraint(distanceSpring(invalid_handle, sprung, .{ 14, 9, 2 }, .{ 14, 9, 0 }, 0, 0, 1, 150, 2));

    // Point 1 = point 2 at a corner of a box (distance 0: the normal of the constructor, then of the last update)
    const corner = try b.body(.{ .position = .{ 17, 9, 0 }, .shape = box, .linear_velocity = .{ 1, 0, 0.5 }, .allow_sleeping = 0 }, .activate);
    _ = try b.constraint(Desc.distance(invalid_handle, corner, .{ 17.2, 9.2, 0.2 }, .{ 17.2, 9.2, 0.2 }, -1, -1));

    // A spinning dumbbell without gravity: a single distance between two dynamic boxes
    const left = try b.body(.{ .position = .{ 20, 9, 0 }, .shape = box, .gravity_factor = 0, .linear_velocity = .{ 0, 1, 0 }, .angular_velocity = .{ 0, 0, 2 }, .allow_sleeping = 0 }, .activate);
    const right = try b.body(.{ .position = .{ 21, 9, 0 }, .shape = ball, .gravity_factor = 0, .linear_velocity = .{ 0, -1, 0.5 }, .allow_sleeping = 0 }, .activate);
    const dumbbell = try b.constraint(Desc.distance(left, right, .{ 20.1, 9, 0 }, .{ 20.9, 9, 0 }, -1, -1));

    // Local space with the world as body 2: point 1 relative to the box, point 2 relative to the world origin
    const local_box = try b.body(.{ .position = .{ 24, 8, 0 }, .rotation = axisAngle(.{ 0, 0, 1 }, 0.4), .shape = box, .allow_sleeping = 0 }, .activate);
    var local = Desc.distance(local_box, invalid_handle, .{ 0.2, 0.2, 0 }, .{ 24.5, 10, 0 }, 1.5, 2.5);
    local.i[0] = ConstraintSpace.local_to_body_com;
    const local_rope = try b.constraint(local);

    b.at(30);
    try b.constraintOp(.distance_set_distance, rope, .{ .f = &.{ 0.5, 1.2 } }); // Shorter: pulls the box up
    try b.constraintOp(.distance_set_limits_spring_settings, soft[0], .{ .i = &.{0}, .f = &.{ 0, 0 } }); // Soft becomes hard
    try b.constraintOp(.constraint_settings_hash, local_rope, .{});
    b.at(50);
    try b.constraintOp(.distance_set_limits_spring_settings, rope, .{ .i = &.{1}, .f = &.{ 200, 8 } }); // Hard becomes soft
    try b.constraintOp(.distance_set_distance, rod, .{ .f = &.{ 2.5, 2.5 } }); // A single distance
    try b.constraintOp(.distance_set_distance, dumbbell, .{ .f = &.{ 0.5, 1.5 } }); // The dumbbell gets a range
    try b.constraintOp(.constraint_settings_hash, soft[1], .{});
    b.at(70);
    try b.constraintOp(.distance_set_distance, soft[2], .{ .f = &.{ 0.2, 0.4 } });
    try b.constraintOp(.distance_set_distance, rope, .{ .f = &.{ 2, 3 } }); // Longer: the box falls into the soft limit
    try b.constraintOp(.notify_constraint_shape_changed, dumbbell, .{ .body = right, .f = &.{ 0.05, 0, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, rod, .{});
    b.at(90);
    try b.constraintOp(.distance_set_limits_spring_settings, soft[2], .{ .i = &.{2}, .f = &.{ 0, 3 } }); // No stiffness: hard
    try b.constraintOp(.constraint_settings_hash, rope, .{});
    try b.constraintOp(.constraint_settings_hash, dumbbell, .{});
}

/// Pulley lengths: Jolt's defaults (min 0, max the length at creation) with equal masses in balance, an impulse throws
/// body 2 up (the rope goes slack) and it falls back until the rope is taut; a block and tackle (ratio 2, min = max =
/// the length) with equal masses; ratio 0.5 with a min length above the length at creation (pushes the bodies away from
/// their fixed points); a min length above the max length (Jolt's constructor accepts it: forces in both directions);
/// Body::sFixedToWorld as body 1 with its point on fixed point 1 (segment 1 has length 0 and keeps its normal); a static
/// body 1 in local space; a box whose center of mass is on fixed point 2 (local space body point 0) falls away from it;
/// SetLength between the updates (longer, shorter, min = max), NotifyShapeChanged, ResetWarmStart, GetConstraintSettings
fn buildPulleyLengths(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    const light = try b.box(.{ 0.15, 0.15, 0.15 }, 0.05);
    b.num_updates = 120;

    // Jolt's default lengths, in balance
    const a1 = try b.body(.{ .position = .{ -1, 6, 0 }, .shape = box, .allow_sleeping = 0 }, .activate);
    const a2 = try b.body(.{ .position = .{ 1, 6.5, 0 }, .shape = box, .allow_sleeping = 0 }, .activate);
    const slack = try b.constraint(Desc.pulley(a1, a2, .{ -1, 6.25, 0 }, .{ -1, 9, 0 }, .{ 1, 6.75, 0 }, .{ 1, 9, 0 }, 1, 0, -1));

    // A block and tackle
    const b1 = try b.body(.{ .position = .{ 3, 6, 0 }, .shape = box, .allow_sleeping = 0 }, .activate);
    const b2 = try b.body(.{ .position = .{ 5, 6, 0 }, .shape = box, .allow_sleeping = 0 }, .activate);
    const tackle = try b.constraint(Desc.pulley(b1, b2, .{ 3, 6.25, 0 }, .{ 3, 9, 0 }, .{ 5, 6.25, 0 }, .{ 5.2, 9, 0 }, 2, -1, -1));

    // Ratio 0.5, a min length above the length at creation (and a max length)
    const c1 = try b.body(.{ .position = .{ 7, 7, 0 }, .shape = light, .allow_sleeping = 0 }, .activate);
    const c2 = try b.body(.{ .position = .{ 9, 7, 0 }, .shape = box, .allow_sleeping = 0 }, .activate);
    const pushing = try b.constraint(Desc.pulley(c1, c2, .{ 7, 7, 0 }, .{ 7, 9, 0 }, .{ 9, 7, 0 }, .{ 9, 9, 0 }, 0.5, 4, 5));

    // A min length above the max length
    const d1 = try b.body(.{ .position = .{ 11, 7, 0 }, .shape = light, .linear_velocity = .{ 0, -1, 0.5 }, .allow_sleeping = 0 }, .activate);
    const d2 = try b.body(.{ .position = .{ 13, 7, 0 }, .shape = box, .allow_sleeping = 0 }, .activate);
    _ = try b.constraint(Desc.pulley(d1, d2, .{ 11, 7, 0 }, .{ 11, 9, 0 }, .{ 13, 7, 0 }, .{ 13, 9, 0 }, 1.5, 6, 4));

    // The world as body 1 with its point on fixed point 1: a rope over a fixed point for body 2 only
    const e2 = try b.body(.{ .position = .{ 15.5, 6, 0 }, .shape = box, .allow_sleeping = 0 }, .activate);
    const world_pulley = try b.constraint(Desc.pulley(invalid_handle, e2, .{ 15, 9, 0 }, .{ 15, 9, 0 }, .{ 15.5, 6.25, 0 }, .{ 15, 9, 0 }, 1, 0, -1));

    // A static body 1 in local space (body points relative to the centers of mass, the fixed points in world space)
    const anchor = try b.body(.{ .position = .{ 18, 6, 0 }, .shape = box, .motion_type = 0 }, .dont_activate);
    const f2 = try b.body(.{ .position = .{ 19, 6, 0 }, .rotation = axisAngle(.{ 0, 0, 1 }, 0.5), .shape = box, .angular_velocity = .{ 0, 1, 0 }, .allow_sleeping = 0 }, .activate);
    var static_desc = Desc.pulley(anchor, f2, .{ 0, 0.25, 0 }, .{ 18, 9, 0 }, .{ 0.1, 0.2, 0 }, .{ 19.3, 9, 0 }, 1.25, -1, -1);
    static_desc.i[0] = ConstraintSpace.local_to_body_com;
    const static_pulley = try b.constraint(static_desc);

    // The center of mass of body 2 on fixed point 2 (segment 2 has length 0 until the box falls away)
    const g1 = try b.body(.{ .position = .{ 21, 6, 0 }, .shape = box, .allow_sleeping = 0 }, .activate);
    const g2 = try b.body(.{ .position = .{ 23, 7, 0 }, .shape = light, .allow_sleeping = 0 }, .activate);
    var on_fixed = Desc.pulley(g1, g2, .{ 0, 0.25, 0 }, .{ 21, 9, 0 }, .{ 0, 0, 0 }, .{ 23, 7, 0 }, 1, 0, -1);
    on_fixed.i[0] = ConstraintSpace.local_to_body_com;
    _ = try b.constraint(on_fixed);

    b.at(10);
    try b.op(.add_impulse, a2, .{ .f = &.{ 0, 600, 0 } }); // Thrown up: the rope goes slack
    try b.constraintOp(.constraint_settings_hash, static_pulley, .{});
    b.at(40);
    try b.constraintOp(.pulley_set_length, tackle, .{ .f = &.{ 0, 12 } }); // A slack block and tackle
    try b.constraintOp(.pulley_set_length, world_pulley, .{ .f = &.{ 2.5, 2.5 } }); // Shorter: lifts body 2
    try b.constraintOp(.notify_constraint_shape_changed, pushing, .{ .body = c2, .f = &.{ 0, 0.1, 0 } });
    b.at(60);
    try b.constraintOp(.pulley_set_length, slack, .{ .f = &.{ 5, 5 } });
    try b.constraintOp(.pulley_set_length, pushing, .{ .f = &.{ 0, 3 } });
    try b.constraintOp(.reset_constraint_warm_start, tackle, .{});
    b.at(80);
    try b.constraintOp(.pulley_set_length, tackle, .{ .f = &.{ 7, 9 } }); // Too short: pushes
    try b.constraintOp(.constraint_settings_hash, tackle, .{});
    try b.constraintOp(.constraint_settings_hash, world_pulley, .{});
}

/// Distance and pulley constraints in islands with point / fixed constraints and contacts: a chain that starts
/// horizontal (a single distance from the world, a point constraint, a rope with a range, a box welded to the last link)
/// swings into a stack of boxes on the floor; a counterweight on a pulley lifts a crate off the floor; a slack rope
/// between two boxes on the floor: a force drags the first box until the rope is taut and drags the second one along; a
/// box on a distance rope swings into a box hanging from a point constraint
fn buildDistancePulleyIslands(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    const crate_shape = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    b.num_updates = 120;
    _ = try addFloor(b, 20, 0);

    // The stack
    for (0..4) |y| _ = try b.body(.{ .position = .{ 0, 0.25 + real(y) * 0.51, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);

    // The chain hangs from (-3, 3, 0) and starts horizontal
    const l1 = try b.body(.{ .position = .{ -4, 3, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.distance(invalid_handle, l1, .{ -3, 3, 0 }, .{ -3.75, 3, 0 }, -1, -1));
    const l2 = try b.body(.{ .position = .{ -4.6, 3, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.pointWorld(l1, l2, .{ -4.3, 3, 0 }));
    const l3 = try b.body(.{ .position = .{ -5.5, 3, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
    const range = try b.constraint(Desc.distance(l2, l3, .{ -4.85, 3, 0 }, .{ -5.25, 3, 0 }, 0.2, 0.6));
    const weight = try b.body(.{ .position = .{ -5.5, 3.55, 0 }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.fixedAuto(l3, weight));

    // A counterweight lifts a crate off the floor (the crate is the lighter body)
    const crate = try b.body(.{ .position = .{ 5, 0.4, 0 }, .shape = crate_shape, .object_layer = Layers.moving, .mass = 10, .override_mass_properties = 1 }, .activate);
    const counterweight = try b.body(.{ .position = .{ 7, 3, 0 }, .shape = box, .object_layer = Layers.moving, .mass = 15, .override_mass_properties = 1 }, .activate);
    const lift = try b.constraint(Desc.pulley(crate, counterweight, .{ 5, 0.8, 0 }, .{ 5, 5, 0 }, .{ 7, 3.25, 0 }, .{ 7, 5, 0 }, 1, -1, -1));

    // A slack rope between two boxes on the floor
    const puller = try b.body(.{ .position = .{ 0, 0.25, 4 }, .shape = box, .object_layer = Layers.moving }, .activate);
    const dragged = try b.body(.{ .position = .{ 1, 0.25, 4 }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.distance(puller, dragged, .{ 0, 0.25, 4 }, .{ 1, 0.25, 4 }, 0, 1.5));

    // A box on a soft rope swings into a box on a point constraint
    const swinger = try b.body(.{ .position = .{ -2, 4, -4 }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(distanceSpring(invalid_handle, swinger, .{ 0, 4, -4 }, .{ -2, 4, -4 }, 0, 2, 0, 8, 0.5));
    const hanging = try b.body(.{ .position = .{ 0.5, 2, -4 }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.pointWorld(invalid_handle, hanging, .{ 0.5, 3, -4 }));

    b.at(10);
    try b.op(.add_impulse, puller, .{ .f = &.{ -400, 0, 0 } }); // Slides until the rope is taut, then drags the other box
    b.at(60);
    try b.constraintOp(.pulley_set_length, lift, .{ .f = &.{ 0, 8 } }); // Slack: the crate and the counterweight fall onto the floor
    try b.constraintOp(.distance_set_distance, range, .{ .f = &.{ 0.4, 0.4 } });
}

/// A net of boxes on the floor, the neighbours along X connected by single distances and along Z by distance ranges,
/// some pairs by point constraints, the corners hanging on ropes from the world and two pulleys across the net: an island
/// with more than 128 constraints that the LargeIslandSplitter splits, and that falls asleep
fn buildDistanceNet(b: *SceneBuilder) Allocator.Error!void {
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
            const px: Real = real(x) - 5.0;
            const pz: Real = real(z) - 7.0;
            handles[z * nx + x] = try b.body(.{ .position = .{ px, 0.25, pz }, .shape = box, .object_layer = Layers.moving }, .activate);
            if (x > 0) _ = try b.constraint(Desc.distance(handles[z * nx + x - 1], handles[z * nx + x], .{ px - 0.6, 0.25, pz }, .{ px - 0.4, 0.25, pz }, -1, -1));
            if (z > 0) {
                if ((x + z) % 4 == 0)
                    _ = try b.constraint(Desc.pointWorld(handles[(z - 1) * nx + x], handles[z * nx + x], .{ px, 0.25, pz - 0.5 }))
                else
                    _ = try b.constraint(Desc.distance(handles[(z - 1) * nx + x], handles[z * nx + x], .{ px, 0.25, pz - 0.6 }, .{ px, 0.25, pz - 0.4 }, 0.1, 0.3));
            }
        }
    }
    // The corners hang on ropes from the world
    for ([_]usize{ 0, nx - 1, (nz - 1) * nx, nz * nx - 1 }) |i| {
        const p: [3]Real = .{ real(i % nx) - 5.0, 0.25, real(i / nx) - 7.0 };
        _ = try b.constraint(Desc.distance(invalid_handle, handles[i], .{ p[0], 2, p[2] }, p, 0, 1.8));
    }
    // Two pulleys across the net
    _ = try b.constraint(Desc.pulley(handles[0], handles[nz * nx - 1], .{ -5, 0.25, -7 }, .{ -5, 3, 0 }, .{ 4, 0.25, 6 }, .{ 4, 3, 0 }, 1, 0, 14.6));
    _ = try b.constraint(Desc.pulley(handles[nx - 1], handles[(nz - 1) * nx], .{ 4, 0.25, -7 }, .{ 4, 3, -1 }, .{ -5, 0.25, 6 }, .{ -5, 3, -1 }, 1, 0, 14.6));
    b.at(10);
    try b.op(.add_impulse, handles[nx * nz / 2], .{ .f = &.{ 0, 3, 0 } });
}

/// Distance / pulley constraints and sleeping bodies (the first part without gravity): a rope created between an awake
/// body and a sleeping one wakes the sleeping one from inside the update; a slack rope between an awake and a sleeping
/// body is active (TwoBodyConstraint::IsActive) and wakes it too; a pulley between two sleeping bodies is not active,
/// SetLength wakes nothing until ActivateConstraint; with gravity, a box on a rope and two boxes on a pulley come to rest
/// and fall asleep, SetDistance / SetLength / SetLimitsSpringSettings do not wake them until ActivateConstraint
fn buildDistancePulleySleeping(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.num_updates = 120;
    b.settings.time_before_sleep = 0.2;
    const awake_a = try b.body(.{ .position = .{ 0, 5, 0 }, .shape = box, .gravity_factor = 0, .linear_velocity = .{ 0, 0, 0.3 }, .allow_sleeping = 0 }, .activate);
    const asleep_a = try b.body(.{ .position = .{ 0, 4, 0 }, .shape = box, .gravity_factor = 0 }, .dont_activate);
    const awake_b = try b.body(.{ .position = .{ 3, 5, 0 }, .shape = box, .gravity_factor = 0, .linear_velocity = .{ 0.2, 0, 0 }, .allow_sleeping = 0 }, .activate);
    const asleep_b = try b.body(.{ .position = .{ 3, 4, 0 }, .shape = box, .gravity_factor = 0 }, .dont_activate);
    const pair1 = try b.body(.{ .position = .{ 6, 5, 0 }, .shape = box, .gravity_factor = 0 }, .dont_activate);
    const pair2 = try b.body(.{ .position = .{ 7, 5, 0 }, .shape = box, .gravity_factor = 0 }, .dont_activate);

    // With gravity: a box on a rope, two boxes on a pulley (they settle and fall asleep)
    const hanging = try b.body(.{ .position = .{ 10, 4, 0 }, .shape = box, .linear_damping = 0.5 }, .activate);
    const rope = try b.constraint(Desc.distance(invalid_handle, hanging, .{ 10, 5, 0 }, .{ 10, 4.2, 0 }, -1, -1));
    const p1 = try b.body(.{ .position = .{ 13, 4, 0 }, .shape = box, .linear_damping = 0.5 }, .activate);
    const p2 = try b.body(.{ .position = .{ 15, 4, 0 }, .shape = box, .linear_damping = 0.5 }, .activate);
    const resting_pulley = try b.constraint(Desc.pulley(p1, p2, .{ 13, 4.2, 0 }, .{ 13, 6, 0 }, .{ 15, 4.2, 0 }, .{ 15, 6, 0 }, 1, -1, -1));

    b.at(5);
    try b.expectSleeping(asleep_a);
    _ = try b.constraint(Desc.distance(awake_a, asleep_a, .{ 0, 4.8, 0 }, .{ 0, 4.2, 0 }, -1, -1)); // Wakes asleep_a in the next update
    const slack_rope = try b.constraint(Desc.distance(awake_b, asleep_b, .{ 3, 4.8, 0 }, .{ 3, 4.2, 0 }, 0, 3));
    try b.expectConstraintState(slack_rope, ConstraintState.enabled | ConstraintState.active);
    const sleeping_pulley = try b.constraint(Desc.pulley(pair1, pair2, .{ 6, 5.2, 0 }, .{ 6, 7, 0 }, .{ 7, 5.2, 0 }, .{ 7, 7, 0 }, 1, -1, -1));
    try b.expectConstraintState(sleeping_pulley, ConstraintState.inactive);
    b.at(6);
    try b.expectAwake(asleep_a);
    try b.expectAwake(asleep_b);
    try b.expectSleeping(pair1);
    b.at(10);
    try b.constraintOp(.pulley_set_length, sleeping_pulley, .{ .f = &.{ 1, 2 } }); // Violated, but wakes nothing
    b.at(12);
    try b.expectSleeping(pair1);
    try b.expectSleeping(pair2);
    try b.constraintOp(.activate_constraint, sleeping_pulley, .{});
    try b.expectAwake(pair1);
    try b.expectAwake(pair2);
    b.at(80);
    try b.expectSleeping(hanging);
    try b.expectSleeping(p1);
    try b.constraintOp(.distance_set_distance, rope, .{ .f = &.{ 0.3, 0.5 } });
    try b.constraintOp(.pulley_set_length, resting_pulley, .{ .f = &.{ 1, 2.5 } });
    try b.constraintOp(.distance_set_limits_spring_settings, rope, .{ .i = &.{0}, .f = &.{ 3, 0.5 } });
    b.at(82);
    try b.expectSleeping(hanging);
    try b.expectSleeping(p2);
    try b.expectConstraintState(rope, ConstraintState.inactive);
    try b.constraintOp(.activate_constraint, rope, .{});
    try b.constraintOp(.activate_constraint, resting_pulley, .{});
    try b.expectAwake(hanging);
    try b.expectAwake(p2);
}

/// Distance and pulley constraints across cross save / restore (the total lambdas and the world space normals are
/// state; the normal matters when the points coincide) and partial states (the constraints only, bodies and
/// constraints), with changes of the physics settings (no warm start, other iteration counts and Baumgarte factor) and of
/// the constraints between the snapshots
fn buildDistancePulleySaveRestore(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    b.num_updates = 100;
    const state_constraints: u32 = 8; // EStateRecorderState::Constraints
    const state_bodies: u32 = 2; // EStateRecorderState::Bodies
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

    // A chain of ropes from the world
    var previous: u32 = invalid_handle;
    var links: [4]u32 = undefined;
    for (0..4) |i| {
        const x: Real = 0.7 * real(i);
        const h = try b.body(.{ .position = .{ x, 6, 0 }, .angular_velocity = .{ 0, 0, 0.5 }, .shape = box }, .activate);
        links[i] = if (i == 2)
            try b.constraint(distanceSpring(previous, h, .{ x - 0.45, 6, 0 }, .{ x - 0.25, 6, 0 }, 0, 0.4, 1, 500, 10))
        else
            try b.constraint(Desc.distance(previous, h, .{ x - 0.45, 6, 0 }, .{ x - 0.25, 6, 0 }, -1, -1));
        previous = h;
    }
    // Two pulleys, one with a point on its fixed point
    const a1 = try b.body(.{ .position = .{ 5, 5, 0 }, .shape = box }, .activate);
    const a2 = try b.body(.{ .position = .{ 7, 5.5, 0 }, .shape = box, .linear_velocity = .{ 0, 0, 1 } }, .activate);
    const pulley = try b.constraint(Desc.pulley(a1, a2, .{ 5, 5.25, 0 }, .{ 5, 8, 0 }, .{ 7, 5.75, 0 }, .{ 7, 8, 0 }, 1.5, -1, -1));
    const c2 = try b.body(.{ .position = .{ 10, 7, 0 }, .shape = box, .linear_velocity = .{ 0.5, 0, 0 } }, .activate);
    var on_fixed = Desc.pulley(invalid_handle, c2, .{ 9, 8, 0 }, .{ 9, 8, 0 }, .{ 0, 0, 0 }, .{ 10, 7, 0 }, 1, 0, 1);
    on_fixed.i[0] = ConstraintSpace.local_to_body_com;
    const fixed_point_pulley = try b.constraint(on_fixed);

    b.at(15);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(25);
    try b.sysOp(.set_physics_settings, .{ .index = settings1 });
    try b.constraintOp(.distance_set_distance, links[1], .{ .f = &.{ 0.1, 0.3 } });
    b.at(35);
    try b.constraintOp(.set_constraint_enabled, links[3], .{ .i = &.{0} });
    try b.sysOp(.snapshot, .{ .index = 1, .i = &.{state_constraints} });
    b.at(45);
    try b.constraintOp(.set_constraint_enabled, links[3], .{ .i = &.{1} });
    try b.sysOp(.restore_cross, .{ .index = 1 }); // The constraints of update 35: links[3] is disabled again
    try b.expectConstraintState(links[3], 0);
    b.at(50);
    try b.sysOp(.set_physics_settings, .{ .index = settings2 });
    try b.constraintOp(.set_constraint_enabled, links[3], .{ .i = &.{1} });
    try b.sysOp(.snapshot, .{ .index = 2, .i = &.{state_bodies | state_constraints} });
    b.at(60);
    try b.constraintOp(.reset_constraint_warm_start, pulley, .{});
    try b.constraintOp(.reset_constraint_warm_start, fixed_point_pulley, .{});
    try b.op(.add_impulse, a2, .{ .f = &.{ 0, 0, 30 } });
    try b.sysOp(.restore_cross, .{ .index = 2 });
    b.at(70);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Everything of update 15
    try b.expectConstraintState(links[3], ConstraintState.enabled | ConstraintState.active);
    b.at(85);
    try b.sysOp(.snapshot, .{ .index = 3 });
    try b.op(.add_impulse, previous, .{ .f = &.{ 0, 0, 20 } });
    b.at(90);
    try b.sysOp(.restore_cross, .{ .index = 3 });
}

/// Distance and pulley constraints with several collision steps per update and changing update parameters (the warm
/// start impulses are scaled by the step ratio)
fn buildDistancePulleyCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const box = try b.box(.{ 0.2, 0.3, 0.2 }, 0.05);
            b.num_updates = 50;
            b.collision_steps = steps;
            b.delta_time = @as(f32, @floatFromInt(steps)) / 60.0;
            var previous: u32 = invalid_handle;
            for (0..5) |i| {
                const x: Real = 0.6 * real(i);
                const h = try b.body(.{ .position = .{ x, 6, 0 }, .shape = box }, .activate);
                if (i % 2 == 1)
                    _ = try b.constraint(distanceSpring(previous, h, .{ x - 0.4, 6, 0 }, .{ x - 0.2, 6, 0 }, 0, 0.3, 0, 4, 0.5))
                else
                    _ = try b.constraint(Desc.distance(previous, h, .{ x - 0.4, 6, 0 }, .{ x - 0.2, 6, 0 }, -1, -1));
                previous = h;
            }
            const p1 = try b.body(.{ .position = .{ 5, 5, 0 }, .shape = box }, .activate);
            const p2 = try b.body(.{ .position = .{ 7, 5.5, 0 }, .shape = box, .gravity_factor = 1.5 }, .activate);
            _ = try b.constraint(Desc.pulley(p1, p2, .{ 5, 5.3, 0 }, .{ 5, 8, 0 }, .{ 7, 5.8, 0 }, .{ 7, 8, 0 }, 2, -1, -1));
            b.at(20);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 90.0}, .count = 1 });
            b.at(30);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 20.0}, .count = steps + 1 });
        }
    }.build;
}

/// Distance and pulley constraints far from the origin (the world space points and the fixed points are Real: in double
/// precision they keep their precision): a chain of ropes hanging from the world, a box on a soft rope and a pulley
/// whose body falls onto a floor next to a box
fn buildDistancePulleyFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    b.num_updates = 80;
    const o: [3]Real = .{ 5000, 200, -12000 };
    const floor = try b.box(.{ 10, 1, 10 }, 0);
    _ = try b.body(.{ .position = .{ o[0], o[1] - 1, o[2] }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    var previous: u32 = invalid_handle;
    for (0..4) |i| {
        const p: [3]Real = .{ o[0] - 4 + 0.6 * real(i), o[1] + 5, o[2] };
        const h = try b.body(.{ .position = p, .shape = box, .object_layer = Layers.moving }, .activate);
        _ = try b.constraint(Desc.distance(previous, h, .{ p[0] - 0.4, p[1], p[2] }, .{ p[0] - 0.2, p[1], p[2] }, -1, -1));
        previous = h;
    }
    const soft = try b.body(.{ .position = .{ o[0] + 2, o[1] + 3, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(distanceSpring(invalid_handle, soft, .{ o[0] + 2, o[1] + 4, o[2] }, .{ o[0] + 2, o[1] + 3.25, o[2] }, 0, 1, 2, 30, 1));
    const p1 = try b.body(.{ .position = .{ o[0] + 4, o[1] + 3, o[2] }, .shape = box, .object_layer = Layers.moving, .mass = 20, .override_mass_properties = 1 }, .activate);
    const p2 = try b.body(.{ .position = .{ o[0] + 6, o[1] + 2, o[2] }, .shape = box, .object_layer = Layers.moving, .mass = 50, .override_mass_properties = 1 }, .activate);
    _ = try b.constraint(Desc.pulley(p1, p2, .{ o[0] + 4, o[1] + 3.25, o[2] }, .{ o[0] + 4, o[1] + 6, o[2] }, .{ o[0] + 6, o[1] + 2.25, o[2] }, .{ o[0] + 6, o[1] + 6, o[2] }, 1, 0, -1));
    _ = try b.body(.{ .position = .{ o[0] + 6.2, o[1] + 0.25, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
}

/// Random bodies (dynamic, kinematic, static; every basic shape) with random distance and pulley constraints between
/// them and to the world (pulleys only between static or dynamic bodies, IndependentAxisConstraintPart asserts on a
/// kinematic body), on a floor in a container, and random operations between the updates (SetDistance,
/// SetLimitsSpringSettings, SetLength, enable, remove / add, ActivateConstraint, ResetWarmStart, NotifyShapeChanged,
/// impulses)
fn buildDistancePulleyRandom(comptime seed: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.rng = 0xd157_0000 +% seed *% 0x9e3779b9;
            const shapes = try BasicShapes.add(b);
            b.num_updates = 80;
            b.max_contact_constraints = 2048;
            try addContainer(b, 6, 2);
            const num_bodies = 24;
            var bodies: [num_bodies]u32 = undefined;
            var kinematic: [num_bodies]bool = undefined;
            for (&bodies, &kinematic, 0..) |*h, *k, i| {
                const motion_type: u32 = if (b.oneIn(8)) 0 else if (b.oneIn(7)) 1 else 2;
                k.* = motion_type == 1;
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
            var is_pulley: [num_constraints]bool = undefined;
            var in_system: [num_constraints]bool = @splat(true);
            for (&handles, &is_pulley) |*c, *pulley| {
                pulley.* = b.oneIn(3);
                var index1 = b.index(num_bodies + 1); // num_bodies: the world
                var index2 = b.index(num_bodies);
                if (pulley.*) {
                    while (index1 < num_bodies and kinematic[index1]) index1 = b.index(num_bodies + 1);
                    while (kinematic[index2] or index2 == index1) index2 = b.index(num_bodies);
                } else if (index2 == index1) index2 = (index2 + 1) % num_bodies;
                const body1 = if (index1 == num_bodies) invalid_handle else bodies[index1];
                const body2 = bodies[index2];
                var desc: SimConstraint = undefined;
                if (pulley.*) {
                    const lengths = [_][2]f32{ .{ 0, -1 }, .{ -1, -1 }, .{ 2, 12 }, .{ -1, 15 } };
                    const l = lengths[b.index(lengths.len)];
                    desc = Desc.pulley(body1, body2, r3(b.vec3(-4, 4)), r3(b.vec3(-4, 4)), r3(b.vec3(-4, 4)), r3(b.vec3(-4, 4)), b.float(0.5, 2), l[0], l[1]);
                    desc.r[4] = 6 + desc.r[4]; // The fixed points above the bodies
                    desc.r[10] = 6 + desc.r[10];
                } else {
                    const min = if (b.oneIn(3)) -1.0 else b.float(0, 2);
                    const max = if (min < 0 and b.oneIn(2)) -1.0 else (if (min < 0) 0 else min) + b.float(0, 2);
                    desc = Desc.distance(body1, body2, r3(b.vec3(-4, 4)), r3(b.vec3(-4, 4)), min, max);
                    if (b.oneIn(2)) Desc.setSpring(&desc, 1, 2, b.index(3), if (b.oneIn(3)) 0 else b.float(1, 10), b.float(0, 1));
                }
                if (b.oneIn(4)) desc.i[0] = ConstraintSpace.local_to_body_com;
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
                    switch (b.index(9)) {
                        0 => if (is_pulley[k]) {
                            const min = b.float(0, 6);
                            try b.constraintOp(.pulley_set_length, c, .{ .f = &.{ min, min + b.float(0, 6) } });
                        } else {
                            const min = b.float(0, 3);
                            try b.constraintOp(.distance_set_distance, c, .{ .f = &.{ min, min + (if (b.oneIn(3)) 0 else b.float(0, 2)) } });
                        },
                        1 => if (!is_pulley[k]) try b.constraintOp(.distance_set_limits_spring_settings, c, .{ .i = &.{b.index(3)}, .f = &.{ if (b.oneIn(3)) 0 else b.float(1, 10), b.float(0, 1) } }),
                        2 => try b.constraintOp(.set_constraint_enabled, c, .{ .i = &.{b.index(2)} }),
                        3 => {
                            try b.constraintOp(if (in_system[k]) .remove_constraint else .add_constraint, c, .{});
                            in_system[k] = !in_system[k];
                        },
                        4 => try b.constraintOp(.activate_constraint, c, .{}),
                        5 => try b.constraintOp(.reset_constraint_warm_start, c, .{}),
                        6 => try b.constraintOp(.notify_constraint_shape_changed, c, .{ .body = bodies[b.index(num_bodies)], .f = &b.vec3(-0.1, 0.1) }),
                        7 => try b.constraintOp(.constraint_settings_hash, c, .{}),
                        else => try b.op(.add_impulse, bodies[b.index(num_bodies)], .{ .f = &b.vec3(-3, 3) }),
                    }
                }
            }
        }
    }.build;
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

/// A hinge to the world (Body::sFixedToWorld as body 1) around `axis` at `pivot`, with limits, returns the description
fn worldHinge(body2: u32, pivot: [3]Real, axis: [3]f32, normal: [3]f32, limits_min: f32, limits_max: f32) SimConstraint {
    var c = Desc.hinge(invalid_handle, body2, pivot, axis, normal);
    c.f[12] = limits_min;
    c.f[13] = limits_max;
    return c;
}

/// Hinge limits and their springs: bars hinged to the world around Z start horizontal and swing into hard limits (the
/// minimum limit, a maximum limit only), soft limits in all three spring modes, equal limits with a spring (a torsion
/// spring like TestHingeSpring), limits with friction; a pair of dynamic bodies with limits in local space whose frames
/// differ (the zero angle is not the angle of the creation); a box hinged to a kinematic body that turns; SetLimits
/// (narrower limits that the bar is outside of, equal hard limits, no limits) and SetLimitsSpringSettings while they act
fn buildHingeLimits(b: *SceneBuilder) Allocator.Error!void {
    const bar = try b.box(.{ 0.5, 0.1, 0.1 }, 0.05);
    const box = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    b.num_updates = 100;
    const Pendulum = struct { side: f32, min: f32, max: f32, spring_mode: u32 = 0, frequency: f32 = 0, damping: f32 = 0, friction: f32 = 0 };
    const pendulums = [_]Pendulum{
        .{ .side = 1, .min = -0.6, .max = 0.3 }, // Swings down into the hard minimum limit
        .{ .side = -1, .min = -std.math.pi, .max = 0.4 }, // A maximum limit only
        .{ .side = 1, .min = -0.5, .max = 0.5, .spring_mode = 0, .frequency = 2, .damping = 0.2 }, // Soft: frequency and damping
        .{ .side = 1, .min = -0.5, .max = 0.5, .spring_mode = 1, .frequency = 400, .damping = 5 }, // Soft: stiffness and damping
        .{ .side = -1, .min = -0.5, .max = 0.5, .spring_mode = 2, .frequency = 60, .damping = 3 }, // Soft: mass normalized
        .{ .side = 1, .min = 0, .max = 0, .spring_mode = 0, .frequency = 1.5, .damping = 0.1 }, // A torsion spring
        .{ .side = 1, .min = -1, .max = 1, .friction = 30 }, // Friction and limits
    };
    var hinges: [pendulums.len]u32 = undefined;
    for (pendulums, &hinges, 0..) |p, *h, k| {
        const x: Real = 3 * real(k);
        const body = try b.body(.{ .position = .{ x + 0.5 * p.side, 5, 0 }, .shape = bar, .allow_sleeping = 0, .object_layer = Layers.unused1 }, .activate);
        var c = worldHinge(body, .{ x, 5, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 }, p.min, p.max);
        Desc.setSpring(&c, 1, 14, p.spring_mode, p.frequency, p.damping);
        c.f[16] = p.friction;
        h.* = try b.constraint(c);
    }

    // A pair of dynamic boxes: a hangs from the world on a point constraint, c on a hinge in local space whose normal
    // axes differ by 30 degrees, with hard limits that the spinning c reaches
    const a = try b.body(.{ .position = .{ 25, 5, 0 }, .shape = box, .object_layer = Layers.unused1 }, .activate);
    const c = try b.body(.{ .position = .{ 25.8, 5, 0 }, .shape = box, .angular_velocity = .{ 0, 0, 3 }, .object_layer = Layers.unused1 }, .activate);
    _ = try b.constraint(Desc.pointWorld(invalid_handle, a, .{ 25, 5.4, 0 }));
    var local = Desc.hinge(a, c, .{ 0.4, 0, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 });
    local.i[0] = ConstraintSpace.local_to_body_com;
    local.r[3] = -0.4;
    local.r[4] = 0;
    local.r[5] = 0;
    local.f[9] = @floatCast(@cos(std.math.pi / 6.0)); // Normal axis 2
    local.f[10] = @floatCast(@sin(std.math.pi / 6.0));
    local.f[12] = -0.3;
    local.f[13] = 0.3;
    const pair_hinge = try b.constraint(local);

    // A box hinged to a kinematic body that turns around X (the hinge axis), with limits: the box is dragged along
    const kinematic = try b.body(.{ .position = .{ 30, 5, 0 }, .shape = box, .motion_type = 1, .angular_velocity = .{ 1.5, 0, 0 }, .object_layer = Layers.unused1 }, .activate);
    const dragged = try b.body(.{ .position = .{ 30, 4.2, 0 }, .shape = box, .object_layer = Layers.unused1 }, .activate);
    var kinematic_hinge = Desc.hinge(kinematic, dragged, .{ 30, 4.6, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 });
    kinematic_hinge.f[12] = -0.5;
    kinematic_hinge.f[13] = 0.5;
    _ = try b.constraint(kinematic_hinge);

    b.at(40);
    try b.constraintOp(.hinge_set_limits, hinges[0], .{ .f = &.{ -0.3, 0.3 } }); // The bar is outside: pushed back
    try b.constraintOp(.hinge_set_limits_spring_settings, hinges[2], .{ .i = &.{0}, .f = &.{ 0, 0 } }); // Hard now
    try b.constraintOp(.hinge_set_limits_spring_settings, hinges[1], .{ .i = &.{1}, .f = &.{ 200, 2 } }); // Soft now
    try b.constraintOp(.constraint_settings_hash, pair_hinge, .{});
    b.at(55);
    try b.constraintOp(.hinge_set_limits, hinges[5], .{ .f = &.{ -std.math.pi, std.math.pi } }); // The torsion spring is gone
    try b.constraintOp(.hinge_set_max_friction_torque, hinges[6], .{ .f = &.{0} });
    try b.constraintOp(.constraint_settings_hash, hinges[3], .{});
    b.at(70);
    try b.constraintOp(.hinge_set_limits, hinges[0], .{ .f = &.{ 0, 0 } }); // Equal hard limits: the angle is fixed
    try b.constraintOp(.hinge_set_limits, pair_hinge, .{ .f = &.{ -0.1, 0.05 } });
    try b.constraintOp(.notify_constraint_shape_changed, pair_hinge, .{ .body = c, .f = &.{ 0.02, 0, 0 } });
    try b.constraintOp(.constraint_settings_hash, hinges[0], .{});
}

/// Hinge motors in every EMotorState: doors hinged to the world around Y (vertical): off with friction; a velocity motor
/// with a torque limit whose target changes, then off; a position motor (target angle, SetTargetOrientationBS, a target
/// clamped by limits set later, then a velocity motor); a position motor without stiffness (inactive) that becomes a
/// position and velocity motor with damping only; a position and velocity motor with a spring; a position motor with
/// asymmetric torque limits; a mass normalized motor spring with limits; a velocity motor with the world as body 2; and
/// one door that goes through every state. The doors never sleep: a motor state set on the hinge of a sleeping door
/// does nothing (the constraint is not active, see constraints_hinge_sleep_and_save_restore)
fn buildHingeMotors(b: *SceneBuilder) Allocator.Error!void {
    const door = try b.box(.{ 0.5, 1, 0.05 }, 0.02);
    b.num_updates = 90;
    var doors: [9]u32 = undefined;
    var hinges: [9]u32 = undefined;
    for (&doors, &hinges, 0..) |*d, *h, k| {
        const x: Real = 3 * real(k);
        const spin: [3]f32 = if (k == 0) .{ 0, 3, 0 } else @splat(0);
        d.* = try b.body(.{ .position = .{ x + 0.5, 3, 0 }, .shape = door, .angular_velocity = spin, .allow_sleeping = 0, .object_layer = Layers.unused1 }, .activate);
        var c = if (k == 7)
            Desc.hinge(d.*, invalid_handle, .{ x, 3, 0 }, .{ 0, 1, 0 }, .{ 1, 0, 0 }) // The world as body 2
        else
            Desc.hinge(invalid_handle, d.*, .{ x, 3, 0 }, .{ 0, 1, 0 }, .{ 1, 0, 0 });
        switch (k) {
            0 => c.f[16] = 5, // Friction
            3 => Desc.setMotor(&c, 2, 17, 1, .{ 0, 20, -flt_max, flt_max, -flt_max, flt_max }), // Damping only
            4 => Desc.setMotor(&c, 2, 17, 0, .{ 3, 0.5, -flt_max, flt_max, -100, 100 }),
            5 => Desc.setMotor(&c, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -2, 100 }), // Asymmetric torque limits
            6 => {
                Desc.setMotor(&c, 2, 17, 2, .{ 30, 4, -flt_max, flt_max, -flt_max, flt_max }); // Mass normalized
                c.f[12] = -0.8;
                c.f[13] = 0.8;
            },
            else => {},
        }
        h.* = try b.constraint(c);
    }
    b.at(1);
    try b.constraintOp(.hinge_set_motor_settings, hinges[1], .{ .i = &.{0}, .f = &.{ 2, 1, -flt_max, flt_max, -20, 20 } });
    try b.constraintOp(.hinge_set_motor_state, hinges[1], .{ .i = &.{1} }); // Velocity
    try b.constraintOp(.hinge_set_target_angular_velocity, hinges[1], .{ .f = &.{2} });
    try b.constraintOp(.hinge_set_motor_state, hinges[2], .{ .i = &.{2} }); // Position
    try b.constraintOp(.hinge_set_target_angle, hinges[2], .{ .f = &.{1} });
    try b.constraintOp(.hinge_set_motor_state, hinges[3], .{ .i = &.{2} }); // Position without stiffness: inactive
    try b.constraintOp(.hinge_set_target_angle, hinges[3], .{ .f = &.{1} });
    try b.constraintOp(.hinge_set_motor_state, hinges[4], .{ .i = &.{3} }); // Position and velocity
    try b.constraintOp(.hinge_set_target_angle, hinges[4], .{ .f = &.{0.5} });
    try b.constraintOp(.hinge_set_target_angular_velocity, hinges[4], .{ .f = &.{0.3} });
    try b.constraintOp(.hinge_set_motor_state, hinges[5], .{ .i = &.{2} });
    try b.constraintOp(.hinge_set_target_angle, hinges[5], .{ .f = &.{1.2} });
    try b.constraintOp(.hinge_set_motor_state, hinges[6], .{ .i = &.{2} });
    try b.constraintOp(.hinge_set_target_angle, hinges[6], .{ .f = &.{-1} }); // Clamped to the limit -0.8
    try b.constraintOp(.hinge_set_motor_state, hinges[7], .{ .i = &.{1} });
    try b.constraintOp(.hinge_set_target_angular_velocity, hinges[7], .{ .f = &.{-1.5} });
    try b.constraintOp(.hinge_set_motor_state, hinges[8], .{ .i = &.{1} });
    try b.constraintOp(.hinge_set_target_angular_velocity, hinges[8], .{ .f = &.{1} });
    b.at(20);
    try b.constraintOp(.hinge_set_motor_state, hinges[8], .{ .i = &.{2} });
    try b.constraintOp(.hinge_set_target_angle, hinges[8], .{ .f = &.{-0.5} });
    b.at(25);
    try b.constraintOp(.hinge_set_target_orientation_bs, hinges[2], .{ .f = &axisAngle(.{ 0, 1, 0 }, -0.6) });
    b.at(30);
    try b.constraintOp(.hinge_set_target_angular_velocity, hinges[1], .{ .f = &.{-1} });
    try b.constraintOp(.hinge_set_motor_state, hinges[3], .{ .i = &.{3} }); // Damping only: active
    try b.constraintOp(.hinge_set_target_angular_velocity, hinges[3], .{ .f = &.{0.8} });
    try b.constraintOp(.constraint_settings_hash, hinges[5], .{});
    b.at(40);
    try b.constraintOp(.hinge_set_motor_state, hinges[8], .{ .i = &.{3} });
    try b.constraintOp(.hinge_set_target_angular_velocity, hinges[8], .{ .f = &.{0.4} });
    b.at(45);
    try b.constraintOp(.hinge_set_limits, hinges[2], .{ .f = &.{ -0.5, 0.5 } });
    try b.constraintOp(.hinge_set_target_angle, hinges[2], .{ .f = &.{1.2} }); // Clamped to 0.5
    try b.constraintOp(.hinge_set_max_friction_torque, hinges[0], .{ .f = &.{40} });
    b.at(50);
    try b.constraintOp(.hinge_set_motor_state, hinges[1], .{ .i = &.{0} }); // Off: no friction, the door swings on
    try b.constraintOp(.hinge_set_motor_settings, hinges[4], .{ .i = &.{1}, .f = &.{ 50, 5, -flt_max, flt_max, -30, 30 } });
    b.at(60);
    try b.constraintOp(.hinge_set_motor_state, hinges[2], .{ .i = &.{1} });
    try b.constraintOp(.hinge_set_target_angular_velocity, hinges[2], .{ .f = &.{-2} }); // Into the limit
    try b.constraintOp(.hinge_set_motor_state, hinges[8], .{ .i = &.{0} });
    try b.constraintOp(.hinge_set_max_friction_torque, hinges[8], .{ .f = &.{3} });
    try b.constraintOp(.constraint_settings_hash, hinges[8], .{});
    b.at(75);
    try b.constraintOp(.reset_constraint_warm_start, hinges[4], .{});
    try b.constraintOp(.hinge_set_motor_state, hinges[7], .{ .i = &.{2} });
    try b.constraintOp(.hinge_set_target_orientation_bs, hinges[7], .{ .f = &axisAngle(.{ 0, 1, 0 }, 0.3) });
}

/// A car on hinges in one island with contacts and other constraint kinds: a chassis with four wheels (cylinders) on
/// hinges, the rear wheels driven by velocity motors, the front wheels with friction; a roof welded to the chassis
/// (FixedConstraint) and a trailer with two free wheels on a point constraint hitch; the car drives over the floor into
/// boxes and a gate hinged to the world, reverses, then brakes (motors off, friction)
fn buildHingeCar(b: *SceneBuilder) Allocator.Error!void {
    const chassis_shape = try b.box(.{ 1.0, 0.25, 1.6 }, 0.05);
    const roof_shape = try b.box(.{ 0.8, 0.2, 0.8 }, 0.05);
    const trailer_shape = try b.box(.{ 0.8, 0.3, 0.9 }, 0.05);
    const wheel_shape = try b.cylinder(0.15, 0.45, 0.05);
    const crate = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    const gate_shape = try b.box(.{ 1.2, 0.8, 0.1 }, 0.05);
    const wheel_rotation = axisAngle(.{ 0, 0, 1 }, 0.5 * std.math.pi); // The cylinder axis along X
    b.num_updates = 150;
    b.max_contact_constraints = 2048;
    _ = try addFloor(b, 30, 0);

    const chassis = try b.body(.{ .position = .{ 0, 0.95, 0 }, .shape = chassis_shape, .friction = 0.5, .object_layer = Layers.moving }, .activate);
    const roof = try b.body(.{ .position = .{ 0, 1.42, -0.3 }, .shape = roof_shape, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.fixedAuto(chassis, roof));
    var wheel_hinges: [4]u32 = undefined;
    for (&wheel_hinges, 0..) |*h, k| {
        const x: Real = if (k % 2 == 0) -1.3 else 1.3;
        const z: Real = if (k < 2) 1.1 else -1.1; // Front wheels first
        const wheel = try b.body(.{ .position = .{ x, 0.45, z }, .rotation = wheel_rotation, .shape = wheel_shape, .friction = 1, .object_layer = Layers.moving }, .activate);
        var c = Desc.hinge(chassis, wheel, .{ x, 0.45, z }, .{ 1, 0, 0 }, .{ 0, 1, 0 });
        if (k < 2) c.f[16] = 0.5; // Front wheels: a little friction
        Desc.setMotor(&c, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -400, 400 });
        h.* = try b.constraint(c);
    }

    // The trailer on one axle with free wheels, hitched with a point constraint
    const trailer = try b.body(.{ .position = .{ 0, 0.85, -3.2 }, .shape = trailer_shape, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.pointWorld(chassis, trailer, .{ 0, 0.85, -2.0 }));
    for ([_]Real{ -1.05, 1.05 }) |x| {
        const wheel = try b.body(.{ .position = .{ x, 0.45, -3.2 }, .rotation = wheel_rotation, .shape = wheel_shape, .friction = 1, .object_layer = Layers.moving }, .activate);
        _ = try b.constraint(Desc.hinge(trailer, wheel, .{ x, 0.45, -3.2 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }));
    }

    // Crates and a gate hinged to the world (vertical axis, limits, friction) in the way
    for (0..3) |k| _ = try b.body(.{ .position = .{ -0.9 + 0.9 * real(k), 0.4, 5.5 }, .shape = crate, .object_layer = Layers.moving }, .activate);
    const gate = try b.body(.{ .position = .{ 1.2, 0.85, 9 }, .shape = gate_shape, .object_layer = Layers.moving }, .activate);
    var gate_hinge = worldHinge(gate, .{ 0, 0.85, 9 }, .{ 0, 1, 0 }, .{ 1, 0, 0 }, -1.4, 1.4);
    gate_hinge.f[16] = 20;
    const gate_handle = try b.constraint(gate_hinge);

    b.at(1);
    for (wheel_hinges[2..]) |h| {
        try b.constraintOp(.hinge_set_motor_state, h, .{ .i = &.{1} });
        try b.constraintOp(.hinge_set_target_angular_velocity, h, .{ .f = &.{6} });
    }
    b.at(90);
    for (wheel_hinges[2..]) |h| try b.constraintOp(.hinge_set_target_angular_velocity, h, .{ .f = &.{-3} });
    try b.constraintOp(.constraint_settings_hash, gate_handle, .{});
    b.at(120);
    for (wheel_hinges) |h| {
        try b.constraintOp(.hinge_set_motor_state, h, .{ .i = &.{0} });
        try b.constraintOp(.hinge_set_max_friction_torque, h, .{ .f = &.{150} });
    }
}

/// A mat of 12 strips of 12 plates lying on the floor: the plates of a strip are hinged to each other, the strips are
/// joined by point constraints at both ends and the first plate is hinged to the world. One island of 155 constraints
/// and the floor contacts that the LargeIslandSplitter splits, which settles and falls asleep from the final batch of a
/// split island; a push wakes the mat up through its constraints. The gaps between the plates (0.12) are wider than the
/// speculative contact distance: with gaps of 0.02 (body pairs between neighbouring plates) Jolt itself gives different
/// contact caches with 0 and with 3 / 7 worker threads two updates after the push (one body pair more single threaded,
/// two C++ worlds in lockstep, the bodies equal), which no scene may reach.
fn buildHingeMatSplitter(b: *SceneBuilder) Allocator.Error!void {
    const plate = try b.box(.{ 0.24, 0.05, 0.24 }, 0.02);
    b.num_updates = 120;
    b.max_contact_constraints = 4096;
    b.settings.time_before_sleep = 0.25;
    _ = try addFloor(b, 20, 0);
    const n = 12;
    var plates: [n * n]u32 = undefined;
    const y: Real = 0.051;
    for (0..n) |row| {
        for (0..n) |col| {
            const x: Real = 0.6 * real(col) - 3;
            const z: Real = 0.6 * real(row) - 3;
            const h = try b.body(.{ .position = .{ x, y, z }, .shape = plate, .object_layer = Layers.moving }, .activate);
            plates[row * n + col] = h;
            if (col > 0) _ = try b.constraint(Desc.hinge(plates[row * n + col - 1], h, .{ x - 0.3, y, z }, .{ 0, 0, 1 }, .{ 0, 1, 0 }));
            if (row > 0 and (col == 0 or col == n - 1)) _ = try b.constraint(Desc.pointWorld(plates[(row - 1) * n + col], h, .{ x, y, z - 0.3 }));
        }
    }
    _ = try b.constraint(worldHinge(plates[0], .{ -3.3, y, -3 }, .{ 0, 0, 1 }, .{ 0, 1, 0 }, -0.5, 0.5));
    b.at(100);
    try b.expectSleeping(plates[0]);
    try b.expectSleeping(plates[n * n - 1]);
    try b.op(.add_impulse, plates[n * n / 2], .{ .f = &.{ 0, 3, 0 } });
    b.at(101);
    try b.expectAwake(plates[n * n / 2]);
}

/// Hinges and sleeping bodies, and cross save / restore of the hinge state (the warm start impulses of the four parts,
/// the motor state, the target velocity and the target angle; the limits, springs, friction and motor settings are not
/// state): a bar with friction comes to rest and falls asleep, a motor set on the sleeping hinge wakes nothing until an
/// impulse does; a hinge created between an awake and a sleeping body wakes the sleeping one; a door with a motor and
/// limits is restored from snapshots of everything and of the constraints only after its motor state changed
fn buildHingeSleepAndSaveRestore(b: *SceneBuilder) Allocator.Error!void {
    const bar = try b.box(.{ 0.1, 0.5, 0.1 }, 0.05);
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    const door = try b.box(.{ 0.5, 1, 0.05 }, 0.02);
    b.num_updates = 90;
    b.settings.time_before_sleep = 0.2;
    const state_constraints: u32 = 8; // EStateRecorderState::Constraints

    // A bar hanging from a hinge with friction and limits, at rest
    const hanging = try b.body(.{ .position = .{ 0, 4.5, 0 }, .shape = bar, .object_layer = Layers.unused1 }, .activate);
    var hanging_desc = worldHinge(hanging, .{ 0, 5, 0 }, .{ 0, 0, 1 }, .{ 0, 1, 0 }, -0.3, 0.3);
    hanging_desc.f[16] = 20;
    const hanging_hinge = try b.constraint(hanging_desc);

    // An awake body that spins and a sleeping body
    const awake = try b.body(.{ .position = .{ 3, 5, 0 }, .shape = box, .angular_velocity = .{ 0, 0, 1 }, .gravity_factor = 0, .allow_sleeping = 0, .object_layer = Layers.unused1 }, .activate);
    const asleep = try b.body(.{ .position = .{ 3.5, 5, 0 }, .shape = box, .gravity_factor = 0, .object_layer = Layers.unused1 }, .dont_activate);

    // A door with limits that a velocity motor drives
    const d = try b.body(.{ .position = .{ 6.5, 3, 0 }, .shape = door, .object_layer = Layers.unused1 }, .activate);
    const door_hinge = try b.constraint(worldHinge(d, .{ 6, 3, 0 }, .{ 0, 1, 0 }, .{ 1, 0, 0 }, -1, 1));

    b.at(2);
    try b.constraintOp(.hinge_set_motor_state, door_hinge, .{ .i = &.{1} });
    try b.constraintOp(.hinge_set_target_angular_velocity, door_hinge, .{ .f = &.{0.8} });
    b.at(5);
    try b.expectSleeping(asleep);
    const pair_hinge = try b.constraint(Desc.hinge(awake, asleep, .{ 3.25, 5, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 })); // Wakes asleep in the next update
    b.at(6);
    try b.expectAwake(asleep);
    b.at(20);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(25);
    try b.constraintOp(.hinge_set_motor_state, door_hinge, .{ .i = &.{2} });
    try b.constraintOp(.hinge_set_target_angle, door_hinge, .{ .f = &.{-0.5} });
    try b.constraintOp(.hinge_set_target_angular_velocity, door_hinge, .{ .f = &.{0} });
    try b.constraintOp(.hinge_set_limits, door_hinge, .{ .f = &.{ -0.9, 0.9 } }); // Not state: stays after the restore
    b.at(35);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // The velocity motor of update 20 again
    b.at(40);
    try b.expectSleeping(hanging);
    try b.constraintOp(.hinge_set_motor_state, hanging_hinge, .{ .i = &.{1} }); // The bar sleeps: nothing happens
    try b.constraintOp(.hinge_set_target_angular_velocity, hanging_hinge, .{ .f = &.{1} });
    try b.expectConstraintState(hanging_hinge, ConstraintState.inactive);
    try b.sysOp(.snapshot, .{ .index = 1, .i = &.{state_constraints} });
    b.at(41);
    try b.expectSleeping(hanging);
    b.at(45);
    try b.op(.add_impulse, hanging, .{ .f = &.{ 1, 0, 0 } }); // Wakes the bar, the motor drives it into its limit
    try b.constraintOp(.hinge_set_motor_state, door_hinge, .{ .i = &.{3} });
    try b.constraintOp(.reset_constraint_warm_start, pair_hinge, .{});
    b.at(55);
    try b.expectAwake(hanging);
    try b.sysOp(.restore_cross, .{ .index = 1 }); // The constraints of update 40: the motor states and their impulses
    try b.constraintOp(.constraint_settings_hash, door_hinge, .{});
    b.at(70);
    try b.sysOp(.snapshot, .{ .index = 2 });
    try b.op(.add_angular_impulse, d, .{ .f = &.{ 0, 5, 0 } });
    b.at(80);
    try b.sysOp(.restore_cross, .{ .index = 2 });
}

/// A chain of hinged bars with limits and a velocity motor on the first hinge, with several collision steps per update
/// and changing update parameters (the warm start impulses are scaled by the step ratio)
fn buildHingeCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
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
                const axis: [3]f32 = if (i % 2 == 0) .{ 0, 0, 1 } else .{ 0, 1, 0 };
                const normal: [3]f32 = if (i % 2 == 0) .{ 0, 1, 0 } else .{ 0, 0, 1 };
                var c = Desc.hinge(previous, h, .{ x - 0.05, 6, 0 }, axis, normal);
                if (i % 3 == 1) {
                    c.f[12] = -0.4;
                    c.f[13] = 0.4;
                }
                if (i == 4) Desc.setSpring(&c, 1, 14, 0, 3, 0.5);
                const handle = try b.constraint(c);
                if (i == 0) first = handle;
                previous = h;
            }
            b.at(1);
            try b.constraintOp(.hinge_set_motor_state, first, .{ .i = &.{1} });
            try b.constraintOp(.hinge_set_target_angular_velocity, first, .{ .f = &.{-1} });
            b.at(20);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 90.0}, .count = 1 });
            try b.constraintOp(.hinge_set_motor_state, first, .{ .i = &.{2} });
            try b.constraintOp(.hinge_set_target_angle, first, .{ .f = &.{0.3} });
            b.at(30);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 20.0}, .count = steps + 1 });
        }
    }.build;
}

/// Hinges far from the origin (world space points are Real: in double precision the anchors keep their precision): a
/// chain of hinged planks hangs from the world and swings onto a floor and a box, a door with a position motor
fn buildHingeFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const plank = try b.box(.{ 0.3, 0.05, 0.25 }, 0.02);
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    const door = try b.box(.{ 0.5, 1, 0.05 }, 0.02);
    b.num_updates = 70;
    const o: [3]Real = .{ 5000, 200, -12000 };
    const floor = try b.box(.{ 10, 1, 10 }, 0);
    _ = try b.body(.{ .position = .{ o[0], o[1] - 1, o[2] }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    var previous: u32 = invalid_handle;
    for (0..5) |i| {
        const p: [3]Real = .{ o[0] - 3 + 0.65 * real(i), o[1] + 2.5, o[2] };
        const h = try b.body(.{ .position = p, .shape = plank, .object_layer = Layers.moving }, .activate);
        var c = Desc.hinge(previous, h, .{ p[0] - 0.325, p[1], p[2] }, .{ 0, 0, 1 }, .{ 1, 0, 0 });
        c.f[12] = -1.2;
        c.f[13] = 1.2;
        _ = try b.constraint(c);
        previous = h;
    }
    _ = try b.body(.{ .position = .{ o[0] - 1, o[1] + 0.25, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
    const d = try b.body(.{ .position = .{ o[0] + 4.5, o[1] + 1.2, o[2] }, .shape = door, .object_layer = Layers.moving }, .activate);
    const door_hinge = try b.constraint(worldHinge(d, .{ o[0] + 4, o[1] + 1.2, o[2] }, .{ 0, 1, 0 }, .{ 1, 0, 0 }, -1.5, 1.5));
    b.at(1);
    try b.constraintOp(.hinge_set_motor_state, door_hinge, .{ .i = &.{2} });
    try b.constraintOp(.hinge_set_target_angle, door_hinge, .{ .f = &.{1.3} });
    b.at(40);
    try b.constraintOp(.hinge_set_target_orientation_bs, door_hinge, .{ .f = &axisAngle(.{ 0, 1, 0 }, -0.7) });
}

/// Random bodies (dynamic, kinematic, static; every basic shape) in a forest of random hinges (each body hangs from the
/// world or an earlier body; world or local space, random frames, limits hard or soft in every spring mode, friction,
/// motor settings), on a floor in a container, and random operations between the updates: every hinge setter, every
/// motor state, the generic constraint operations and impulses (the compound's center of mass is not at its origin:
/// its local space hinges start with an error that the solver corrects)
fn buildHingeRandom(comptime seed: u32) fn (b: *SceneBuilder) Allocator.Error!void {
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

        fn limits(b: *SceneBuilder) [2]f32 {
            if (b.oneIn(3)) return .{ -std.math.pi, std.math.pi };
            return .{ -b.float(0.05, std.math.pi), b.float(0.05, std.math.pi) };
        }

        fn motor(b: *SceneBuilder) [7]f32 {
            const mode: f32 = @floatFromInt(b.index(3));
            const frequency = if (b.oneIn(5)) 0 else if (mode == 0) b.float(0.5, 8) else b.float(5, 300);
            const torque = if (b.oneIn(3)) flt_max else b.float(5, 200);
            return .{ mode, frequency, if (b.oneIn(4)) 0 else b.float(0, 2), -flt_max, flt_max, -torque, torque };
        }

        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.rng = 0x41b9_e000 +% seed *% 0x9e3779b9;
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

            // A forest of hinges: every body hangs from the world or from an earlier body, the hinge point between the two
            // bodies, the hinge frame consistent with the poses of the creation (world space, or local space computed
            // from the poses), normal axes that sometimes differ by a turn around the hinge axis (another zero angle)
            var handles: [num_bodies]u32 = undefined;
            var in_system: [num_bodies]bool = @splat(true);
            for (&handles, 0..) |*c, k| {
                const to_world = k == 0 or b.oneIn(4);
                const parent: usize = if (to_world) 0 else b.index(@intCast(k));
                const body1 = if (to_world) invalid_handle else bodies[parent];
                const offset = b.vec3(-0.6, 0.6);
                var point: [3]f64 = undefined;
                for (0..3) |n| point[n] = (if (to_world) positions[k][n] else 0.5 * (positions[k][n] + positions[parent][n])) + offset[n];
                const q = b.rotation();
                const axis = rotate(q, .{ 1, 0, 0 }, false);
                const normal1 = rotate(q, .{ 0, 1, 0 }, false);
                const angle: f64 = if (b.oneIn(2)) 0 else b.float(-2, 2);
                const binormal = rotate(q, .{ 0, 0, 1 }, false);
                var normal2: [3]f64 = undefined;
                for (0..3) |n| normal2[n] = @cos(angle) * normal1[n] + @sin(angle) * binormal[n];
                var desc = Desc.defaults(.hinge);
                desc.body1 = body1;
                desc.body2 = bodies[k];
                if (b.oneIn(3)) {
                    // Local space: the point and the axes in the center of mass space of each body (the shapes have their
                    // center of mass at the origin, the world has no rotation)
                    desc.i[0] = ConstraintSpace.local_to_body_com;
                    var local1 = point;
                    var axis1 = axis;
                    var n1 = normal1;
                    if (!to_world) {
                        var d: [3]f64 = undefined;
                        for (0..3) |n| d[n] = point[n] - positions[parent][n];
                        local1 = rotate(rotations[parent], d, true);
                        axis1 = rotate(rotations[parent], axis, true);
                        n1 = rotate(rotations[parent], normal1, true);
                    }
                    var d2: [3]f64 = undefined;
                    for (0..3) |n| d2[n] = point[n] - positions[k][n];
                    const local2 = rotate(rotations[k], d2, true);
                    for (0..3) |n| {
                        desc.r[n] = real(local1[n]);
                        desc.r[3 + n] = real(local2[n]);
                    }
                    @memcpy(desc.f[0..3], &toF32(axis1));
                    @memcpy(desc.f[3..6], &toF32(n1));
                    @memcpy(desc.f[6..9], &toF32(rotate(rotations[k], axis, true)));
                    @memcpy(desc.f[9..12], &toF32(rotate(rotations[k], normal2, true)));
                } else {
                    for (0..3) |n| {
                        desc.r[n] = real(point[n]);
                        desc.r[3 + n] = real(point[n]);
                    }
                    @memcpy(desc.f[0..3], &toF32(axis));
                    @memcpy(desc.f[3..6], &toF32(normal1));
                    @memcpy(desc.f[6..9], &toF32(axis));
                    @memcpy(desc.f[9..12], &toF32(normal2));
                }
                const l = limits(b);
                desc.f[12] = l[0];
                desc.f[13] = l[1];
                if (b.oneIn(3)) Desc.setSpring(&desc, 1, 14, b.index(3), b.float(1, 50), b.float(0, 1));
                if (b.oneIn(3)) desc.f[16] = b.float(0, 20);
                const m = motor(b);
                Desc.setMotor(&desc, 2, 17, @intFromFloat(m[0]), m[1..7].*);
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
                    switch (b.index(13)) {
                        0 => try b.constraintOp(.hinge_set_max_friction_torque, c, .{ .f = &.{if (b.oneIn(3)) 0 else b.float(0, 20)} }),
                        1, 2 => try b.constraintOp(.hinge_set_motor_state, c, .{ .i = &.{b.index(4)} }),
                        3 => try b.constraintOp(.hinge_set_target_angular_velocity, c, .{ .f = &.{b.float(-4, 4)} }),
                        4 => try b.constraintOp(.hinge_set_target_angle, c, .{ .f = &.{b.float(-3.5, 3.5)} }),
                        5 => try b.constraintOp(.hinge_set_target_orientation_bs, c, .{ .f = &b.rotation() }),
                        6 => {
                            const l = limits(b);
                            try b.constraintOp(.hinge_set_limits, c, .{ .f = &l });
                        },
                        7 => try b.constraintOp(.hinge_set_limits_spring_settings, c, .{ .i = &.{b.index(3)}, .f = &.{ if (b.oneIn(3)) 0 else b.float(1, 50), b.float(0, 1) } }),
                        8 => {
                            const m = motor(b);
                            try b.constraintOp(.hinge_set_motor_settings, c, .{ .i = &.{@intFromFloat(m[0])}, .f = m[1..7] });
                        },
                        9 => {
                            try b.constraintOp(if (in_system[k]) .remove_constraint else .add_constraint, c, .{});
                            in_system[k] = !in_system[k];
                        },
                        10 => try b.constraintOp(.set_constraint_enabled, c, .{ .i = &.{b.index(2)} }),
                        11 => try b.constraintOp(.reset_constraint_warm_start, c, .{}),
                        else => try b.op(.add_impulse, bodies[b.index(num_bodies)], .{ .f = &b.vec3(-3, 3) }),
                    }
                }
            }
        }
    }.build;
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

/// A unit vector of the scene builder (normalized in f64, like normalizedQuat)
fn sliderUnit(v: [3]f64) [3]f32 {
    const len = @sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return .{ @floatCast(v[0] / len), @floatCast(v[1] / len), @floatCast(v[2] / len) };
}

/// The X and Y axes of a rotation (a slider axis and a normal axis that are perpendicular), computed in f64
fn sliderFrame(q: [4]f32) [2][3]f32 {
    const x: f64 = q[0];
    const y: f64 = q[1];
    const z: f64 = q[2];
    const w: f64 = q[3];
    return .{
        sliderUnit(.{ 1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y) }),
        sliderUnit(.{ 2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x) }),
    };
}

/// A slider description from the world (`invalid_handle`) or a body to `body2` with mAutoDetectPoint and SetSliderAxis
/// and limits
fn sliderWithLimits(body1: u32, body2: u32, axis: [3]f32, limits_min: f32, limits_max: f32) SimConstraint {
    var s = Desc.slider(body1, body2, axis);
    s.f[12] = limits_min;
    s.f[13] = limits_max;
    return s;
}

/// The motor slider operations: SetMotorState(state)
fn setSliderMotorState(b: *SceneBuilder, handle: u32, state: u32) Allocator.Error!void {
    try b.constraintOp(.slider_set_motor_state, handle, .{ .i = &.{state} });
}

/// EMotorState values
const motor_off: u32 = 0;
const motor_velocity: u32 = 1;
const motor_position: u32 = 2;
const motor_position_and_velocity: u32 = 3;

/// Slider limits and springs: a box on an inclined slider from the world slides down into its hard min limit; a box on
/// a vertical slider falls into a soft limit (frequency and damping) and bounces on its spring; boxes on springs with
/// equal limits (min = max = 0, TestSliderSpring) in the stiffness and the mass normalized stiffness modes start away
/// from their rest position; a one sided limit (min = -FLT_MAX) on an incline the other way; two dynamic boxes on a
/// slider with limits that impulses pull apart into the max limit and push together into the min limit; SetLimits and
/// SetLimitsSpringSettings between the updates (a soft limit becomes hard, a hard limit soft, a limit moves past the
/// box, the limits are removed)
fn buildSliderLimitsAndSprings(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    b.num_updates = 90;

    // An inclined slider from the world with hard limits: the box slides down into its min limit
    const down = try b.body(.{ .position = .{ 0, 5, 0 }, .shape = box }, .activate);
    const hard = try b.constraint(sliderWithLimits(invalid_handle, down, sliderUnit(.{ 1, 1, 0 }), -1.2, 0.8));

    // A vertical slider from the world with a soft limit (frequency and damping): the box falls into the spring
    const falling = try b.body(.{ .position = .{ 3, 5, 0 }, .shape = box }, .activate);
    var s_soft = sliderWithLimits(invalid_handle, falling, .{ 0, 1, 0 }, -0.5, flt_max);
    Desc.setSpring(&s_soft, 3, 14, 0, 3, 0.2);
    const soft = try b.constraint(s_soft);

    // Springs with equal limits (min = max = 0): point 1 is the rest position, point 2 the start position of the box
    for (0..2) |m| {
        const x: Real = 6 + 3 * real(m);
        const h = try b.body(.{ .position = .{ x + 0.75, 5, 0.25 }, .shape = box }, .activate);
        var s = Desc.defaults(.slider);
        s.body1 = invalid_handle;
        s.body2 = h;
        Desc.setR3(s.r[0..], .{ x, 5, 0.25 });
        Desc.setR3(s.r[3..], .{ x + 0.75, 5, 0.25 });
        s.f[12] = 0;
        s.f[13] = 0;
        if (m == 0)
            Desc.setSpring(&s, 3, 14, 1, 2000, 100) // StiffnessAndDamping
        else
            Desc.setSpring(&s, 3, 14, 2, 50, 0.5); // MassNormalizedStiffnessAndDamping
        _ = try b.constraint(s);
    }

    // A one sided limit (min = -FLT_MAX): the slider axis points down the incline, the box slides into its max limit
    const one_sided = try b.body(.{ .position = .{ 12, 5, 0 }, .shape = box }, .activate);
    _ = try b.constraint(sliderWithLimits(invalid_handle, one_sided, sliderUnit(.{ 1, -1, 0 }), -flt_max, 0.3));

    // Two dynamic boxes on a slider with limits (without gravity)
    const left = try b.body(.{ .position = .{ 15, 5, 0 }, .shape = box, .gravity_factor = 0 }, .activate);
    const right = try b.body(.{ .position = .{ 15.6, 5, 0 }, .shape = box, .gravity_factor = 0 }, .activate);
    var s_pair = Desc.sliderAxes(left, right, .{ 15.3, 5, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 });
    s_pair.f[12] = -0.05;
    s_pair.f[13] = 0.4;
    const pair = try b.constraint(s_pair);

    b.at(10);
    try b.op(.add_impulse, right, .{ .f = &.{ 300, 0, 0 } }); // Apart into the max limit
    b.at(30);
    try b.op(.add_impulse, right, .{ .f = &.{ -600, 0, 50 } }); // Together into the min limit
    b.at(40);
    try b.constraintOp(.slider_set_limits_spring_settings, soft, .{ .i = &.{0}, .f = &.{ 0, 0 } }); // The soft limit becomes hard
    try b.constraintOp(.slider_set_limits, hard, .{ .f = &.{ -0.6, 0.8 } }); // The box is beyond the new min limit
    try b.constraintOp(.constraint_settings_hash, hard, .{});
    b.at(60);
    try b.constraintOp(.slider_set_limits_spring_settings, hard, .{ .i = &.{2}, .f = &.{ 30, 1 } }); // The hard limit becomes soft
    try b.constraintOp(.slider_set_limits, soft, .{ .f = &.{ -flt_max, flt_max } }); // No limits: the box falls
    try b.constraintOp(.slider_set_limits, pair, .{ .f = &.{ -0.2, 0.0 } });
    try b.constraintOp(.constraint_settings_hash, pair, .{});
}

/// Slider motors in every EMotorState: friction with the motor off (a box thrown along a horizontal slider stops);
/// velocity motors with symmetric and asymmetric force limits (one can push but not pull); position motors with a
/// frequency spring, a stiffness spring and without stiffness (deactivated: the box falls along its slider into a
/// limit); position and velocity motors with a spring, with damping only (a stiffness mode with zero stiffness) and
/// without stiffness and damping (deactivated until its motor settings change); a
/// velocity motor between two dynamic bodies; a position motor on a slider from a kinematic body that moves. The states,
/// targets, friction and motor settings change between the updates.
fn buildSliderMotors(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.num_updates = 80;

    // Friction with the motor off
    const thrown = try b.body(.{ .position = .{ 0, 5, 0 }, .shape = box, .linear_velocity = .{ 3, 0, 0 } }, .activate);
    var s_friction = Desc.slider(invalid_handle, thrown, .{ 1, 0, 0 });
    s_friction.f[16] = 150; // mMaxFrictionForce
    const friction = try b.constraint(s_friction);

    // Velocity motors: symmetric force limits; asymmetric (pushes up to 300 N, cannot pull)
    const vel1 = try b.body(.{ .position = .{ 2, 5, 0 }, .shape = box }, .activate);
    var s_vel1 = Desc.slider(invalid_handle, vel1, .{ 0, 1, 0 });
    Desc.setMotor(&s_vel1, 4, 17, 0, .{ 2, 1, -400, 400, -flt_max, flt_max });
    const velocity1 = try b.constraint(s_vel1);
    const vel2 = try b.body(.{ .position = .{ 4, 5, 0 }, .shape = box }, .activate);
    var s_vel2 = Desc.slider(invalid_handle, vel2, .{ 0, 1, 0 });
    Desc.setMotor(&s_vel2, 4, 17, 0, .{ 2, 1, 0, 300, -flt_max, flt_max });
    const velocity2 = try b.constraint(s_vel2);

    // Position motors (vertical sliders against gravity): frequency spring, stiffness spring, no stiffness (with limits)
    const pos1 = try b.body(.{ .position = .{ 6, 5, 0 }, .shape = box }, .activate);
    const position1 = try b.constraint(Desc.slider(invalid_handle, pos1, .{ 0, 1, 0 }));
    const pos2 = try b.body(.{ .position = .{ 8, 5, 0 }, .shape = box }, .activate);
    var s_pos2 = Desc.slider(invalid_handle, pos2, .{ 0, 1, 0 });
    Desc.setMotor(&s_pos2, 4, 17, 1, .{ 3000, 150, -2000, 2000, -flt_max, flt_max });
    const position2 = try b.constraint(s_pos2);
    const pos3 = try b.body(.{ .position = .{ 10, 5, 0 }, .shape = box }, .activate);
    var s_pos3 = sliderWithLimits(invalid_handle, pos3, .{ 0, 1, 0 }, -1, 1);
    Desc.setMotor(&s_pos3, 4, 17, 0, .{ 0, 0, -flt_max, flt_max, -flt_max, flt_max });
    const position3 = try b.constraint(s_pos3);

    // Position and velocity motors: a frequency spring; damping only (StiffnessAndDamping with zero stiffness)
    const pv1 = try b.body(.{ .position = .{ 12, 5, 0 }, .shape = box }, .activate);
    var s_pv1 = Desc.slider(invalid_handle, pv1, sliderUnit(.{ 1, 1, 0 }));
    Desc.setMotor(&s_pv1, 4, 17, 0, .{ 2, 0.7, -flt_max, flt_max, -flt_max, flt_max });
    const pos_vel1 = try b.constraint(s_pv1);
    const pv2 = try b.body(.{ .position = .{ 14, 5, 0 }, .shape = box, .gravity_factor = 0 }, .activate);
    var s_pv2 = Desc.slider(invalid_handle, pv2, .{ 1, 0, 0 });
    Desc.setMotor(&s_pv2, 4, 17, 1, .{ 0, 200, -flt_max, flt_max, -flt_max, flt_max });
    const pos_vel2 = try b.constraint(s_pv2);
    const pv3 = try b.body(.{ .position = .{ 14, 5, 2 }, .shape = box, .gravity_factor = 0, .linear_velocity = .{ 0.5, 0, 0 } }, .activate);
    var s_pv3 = Desc.slider(invalid_handle, pv3, .{ 1, 0, 0 });
    Desc.setMotor(&s_pv3, 4, 17, 0, .{ 0, 1, -flt_max, flt_max, -flt_max, flt_max }); // FrequencyAndDamping without frequency: deactivated
    const pos_vel3 = try b.constraint(s_pv3);

    // A velocity motor between two dynamic bodies (without gravity): equal and opposite impulses
    const m1 = try b.body(.{ .position = .{ 16, 5, 0 }, .shape = box, .gravity_factor = 0 }, .activate);
    const m2 = try b.body(.{ .position = .{ 16.5, 5, 0 }, .shape = box, .gravity_factor = 0, .mass = 20, .override_mass_properties = 1 }, .activate);
    var s_pair = Desc.sliderAxes(m1, m2, .{ 16.25, 5, 0 }, sliderUnit(.{ 1, 0, 1 }), .{ 0, 1, 0 });
    Desc.setMotor(&s_pair, 4, 17, 0, .{ 2, 1, -100, 100, -flt_max, flt_max });
    const pair = try b.constraint(s_pair);

    // A position motor on a slider from a kinematic body that moves and turns
    const kinematic = try b.body(.{ .position = .{ 19, 5, 0 }, .shape = box, .motion_type = 1, .linear_velocity = .{ 0, 0.3, 0 }, .angular_velocity = .{ 0, 0.5, 0 } }, .activate);
    const follower = try b.body(.{ .position = .{ 19.6, 5, 0 }, .shape = box }, .activate);
    var s_follow = Desc.sliderAxes(kinematic, follower, .{ 19.3, 5, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 });
    s_follow.f[12] = -0.3;
    s_follow.f[13] = 0.5;
    const follow = try b.constraint(s_follow);

    // The motor states and targets
    try setSliderMotorState(b, velocity1, motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, velocity1, .{ .f = &.{1.5} });
    try setSliderMotorState(b, velocity2, motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, velocity2, .{ .f = &.{-2} }); // Cannot pull: falls slower than free fall
    try setSliderMotorState(b, position1, motor_position);
    try b.constraintOp(.slider_set_target_position, position1, .{ .f = &.{0.5} });
    try setSliderMotorState(b, position2, motor_position);
    try b.constraintOp(.slider_set_target_position, position2, .{ .f = &.{-0.4} });
    try setSliderMotorState(b, position3, motor_position); // No stiffness: the motor is deactivated
    try setSliderMotorState(b, pos_vel1, motor_position_and_velocity);
    try b.constraintOp(.slider_set_target_velocity, pos_vel1, .{ .f = &.{0.2} });
    try b.constraintOp(.slider_set_target_position, pos_vel1, .{ .f = &.{0.6} });
    try setSliderMotorState(b, pos_vel2, motor_position_and_velocity);
    try b.constraintOp(.slider_set_target_velocity, pos_vel2, .{ .f = &.{0.8} });
    try setSliderMotorState(b, pos_vel3, motor_position_and_velocity); // No stiffness or damping: the motor is deactivated
    try b.constraintOp(.slider_set_target_velocity, pos_vel3, .{ .f = &.{-0.3} });
    try setSliderMotorState(b, pair, motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, pair, .{ .f = &.{0.5} });
    try setSliderMotorState(b, follow, motor_position);
    try b.constraintOp(.slider_set_target_position, follow, .{ .f = &.{0.2} });

    b.at(25);
    try setSliderMotorState(b, velocity1, motor_position); // Holds the position it drives to
    try b.constraintOp(.slider_set_target_position, velocity1, .{ .f = &.{1} });
    try setSliderMotorState(b, position1, motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, position1, .{ .f = &.{-0.5} });
    try b.constraintOp(.slider_set_motor_settings, position3, .{ .i = &.{0}, .f = &.{ 4, 1, -flt_max, flt_max, -flt_max, flt_max } }); // Now with stiffness
    try b.constraintOp(.slider_set_target_position, position3, .{ .f = &.{3} }); // Clamped to the max limit
    try b.constraintOp(.slider_set_max_friction_force, friction, .{ .f = &.{40} });
    try setSliderMotorState(b, pos_vel2, motor_off); // No friction: slides on
    b.at(45);
    try setSliderMotorState(b, pair, motor_position_and_velocity);
    try b.constraintOp(.slider_set_target_velocity, pair, .{ .f = &.{0} });
    try b.constraintOp(.slider_set_target_position, pair, .{ .f = &.{0} });
    try setSliderMotorState(b, pos_vel1, motor_off);
    try b.constraintOp(.slider_set_max_friction_force, pos_vel1, .{ .f = &.{500} }); // Friction holds it on the incline
    try b.constraintOp(.slider_set_motor_settings, position2, .{ .i = &.{2}, .f = &.{ 40, 2, -flt_max, flt_max, -flt_max, flt_max } }); // MassNormalizedStiffnessAndDamping
    try b.constraintOp(.constraint_settings_hash, position2, .{});
    b.at(60);
    try setSliderMotorState(b, velocity2, motor_off);
    try setSliderMotorState(b, follow, motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, follow, .{ .f = &.{-1} }); // Into the min limit
    try b.constraintOp(.slider_set_motor_settings, pos_vel3, .{ .i = &.{0}, .f = &.{ 3, 1, -flt_max, flt_max, -flt_max, flt_max } }); // Now it drives
    try b.constraintOp(.constraint_settings_hash, follow, .{});
}

/// Sliders in islands with other constraint kinds and contacts: a train of carts on sliders from a static rail body
/// (body 1 of the sliders), coupled by point constraints, with a chimney welded on (fixed constraint), driven by the
/// velocity motor of the first cart into boxes on the floor; a piston (a box on a vertical slider with a soft limit)
/// that a falling box lands on; a box on a slider from the world with hard limits that a box on a point constraint
/// swings into
fn buildSliderIslands(b: *SceneBuilder) Allocator.Error!void {
    const cart = try b.box(.{ 0.25, 0.2, 0.25 }, 0.05);
    const small = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.num_updates = 90;
    _ = try addFloor(b, 20, 0);
    const rail = try b.body(.{ .position = .{ 0, 4, -3 }, .shape = small, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);

    // The train (the slider of the front cart has the motor)
    var carts: [4]u32 = undefined;
    var drive: u32 = undefined;
    for (&carts, 0..) |*h, i| {
        const x: Real = -4 + 0.6 * real(i);
        h.* = try b.body(.{ .position = .{ x, 0.25, 0 }, .shape = cart, .object_layer = Layers.moving }, .activate);
        var s = Desc.slider(rail, h.*, .{ 1, 0, 0 });
        if (i == carts.len - 1) Desc.setMotor(&s, 4, 17, 0, .{ 2, 1, -3000, 3000, -flt_max, flt_max });
        drive = try b.constraint(s);
        if (i > 0) _ = try b.constraint(Desc.pointWorld(carts[i - 1], h.*, .{ x - 0.3, 0.3, 0 }));
    }
    const chimney = try b.body(.{ .position = .{ -4, 0.66, 0 }, .shape = small, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.fixedAuto(carts[0], chimney));
    try setSliderMotorState(b, drive, motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, drive, .{ .f = &.{2} });

    // The boxes in front of the train
    for (0..3) |i| _ = try b.body(.{ .position = .{ -1 + 0.6 * real(i), 0.2, 0.05 * real(i) }, .shape = small, .object_layer = Layers.moving }, .activate);

    // The piston and the box that falls on it
    const piston = try b.body(.{ .position = .{ 4, 1, 0 }, .shape = cart, .object_layer = Layers.moving }, .activate);
    var s_piston = sliderWithLimits(invalid_handle, piston, .{ 0, 1, 0 }, -0.5, 0.5);
    Desc.setSpring(&s_piston, 3, 14, 0, 4, 0.3);
    _ = try b.constraint(s_piston);
    _ = try b.body(.{ .position = .{ 4.05, 3, 0 }, .shape = small, .object_layer = Layers.moving }, .activate);

    // A pendulum swings into a box on a slider with hard limits
    const target = try b.body(.{ .position = .{ 8.15, 2, 0 }, .shape = small, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(sliderWithLimits(invalid_handle, target, .{ 1, 0, 0 }, -0.2, 0.6));
    const bob = try b.body(.{ .position = .{ 6.3, 3.5, 0 }, .shape = small, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.pointWorld(invalid_handle, bob, .{ 7.8, 3.5, 0 }));

    b.at(50);
    try b.constraintOp(.slider_set_target_velocity, drive, .{ .f = &.{-1} });
}

/// A net of boxes on the floor, the neighbours along X connected by sliders with limits and along Z by point
/// constraints, the corners pinned to the world: an island with more than 128 constraints that the LargeIslandSplitter
/// splits, and that falls asleep
fn buildSliderNet(b: *SceneBuilder) Allocator.Error!void {
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
            const px: Real = real(x) - 5.0;
            const pz: Real = real(z) - 7.0;
            handles[z * nx + x] = try b.body(.{ .position = .{ px, 0.2, pz }, .shape = box, .object_layer = Layers.moving }, .activate);
            if (x > 0) {
                var s = Desc.sliderAxes(handles[z * nx + x - 1], handles[z * nx + x], .{ px - 0.5, 0.2, pz }, .{ 1, 0, 0 }, .{ 0, 1, 0 });
                s.f[12] = -0.1;
                s.f[13] = 0.1;
                _ = try b.constraint(s);
            }
            if (z > 0) _ = try b.constraint(Desc.pointWorld(handles[(z - 1) * nx + x], handles[z * nx + x], .{ px, 0.2, pz - 0.5 }));
        }
    }
    // Pin the corners to the world
    for ([_]usize{ 0, nx - 1, (nz - 1) * nx, nz * nx - 1 }) |i| {
        const p: [3]Real = .{ real(i % nx) - 5.0, 0.2, real(i / nx) - 7.0 };
        _ = try b.constraint(Desc.pointWorld(invalid_handle, handles[i], p));
    }
    b.at(10);
    try b.op(.add_impulse, handles[nx * nz / 2], .{ .f = &.{ 0, 3, 0 } });
}

/// Sliders and sleeping bodies (without gravity): a slider created between an awake body and a sleeping one wakes the
/// sleeping one from inside the update; a slider with a velocity motor between two sleeping bodies is not active (setting
/// the motor state wakes nothing) until ActivateConstraint; a box that a position motor drove to its target falls asleep,
/// a new target does not wake it (SetTargetPosition does not activate bodies) until ActivateConstraint; a box at rest at
/// its hard limit falls asleep, SetLimits wakes nothing either
fn buildSliderSleeping(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.num_updates = 90;
    b.gravity = .{ 0, 0, 0 };
    b.settings.time_before_sleep = 0.2;
    const awake = try b.body(.{ .position = .{ 0, 5, 0 }, .shape = box, .linear_velocity = .{ 0, 0, 0.5 }, .allow_sleeping = 0 }, .activate);
    const asleep = try b.body(.{ .position = .{ 0.6, 5, 0 }, .shape = box }, .dont_activate);
    const pair1 = try b.body(.{ .position = .{ 3, 5, 0 }, .shape = box }, .dont_activate);
    const pair2 = try b.body(.{ .position = .{ 3.6, 5, 0 }, .shape = box }, .dont_activate);
    const driven = try b.body(.{ .position = .{ 6, 5, 0 }, .shape = box }, .activate);
    const at_limit = try b.body(.{ .position = .{ 9, 5, 0 }, .shape = box, .linear_velocity = .{ 1, 0, 0 } }, .activate);

    const motor_pair = try b.constraint(Desc.sliderAxes(pair1, pair2, .{ 3.3, 5, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }));
    try setSliderMotorState(b, motor_pair, motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, motor_pair, .{ .f = &.{0.5} });
    try b.expectConstraintState(motor_pair, ConstraintState.inactive);
    const position_motor = try b.constraint(Desc.slider(invalid_handle, driven, .{ 0, 1, 0 }));
    try setSliderMotorState(b, position_motor, motor_position);
    try b.constraintOp(.slider_set_target_position, position_motor, .{ .f = &.{0.3} });
    const limit = try b.constraint(sliderWithLimits(invalid_handle, at_limit, .{ 1, 0, 0 }, -0.5, 0.2));
    b.at(5);
    try b.expectSleeping(asleep);
    _ = try b.constraint(Desc.sliderAxes(awake, asleep, .{ 0.3, 5, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 })); // Wakes it in the next update
    b.at(6);
    try b.expectAwake(asleep);
    try b.expectSleeping(pair1);
    b.at(10);
    try b.constraintOp(.activate_constraint, motor_pair, .{});
    try b.expectAwake(pair1);
    try b.expectAwake(pair2);
    b.at(60);
    try b.expectSleeping(driven);
    try b.expectSleeping(at_limit);
    try b.constraintOp(.slider_set_target_position, position_motor, .{ .f = &.{-0.4} });
    try b.constraintOp(.slider_set_limits, limit, .{ .f = &.{ -0.5, 0.1 } });
    b.at(65);
    try b.expectSleeping(driven);
    try b.expectSleeping(at_limit);
    try b.constraintOp(.activate_constraint, position_motor, .{});
    try b.constraintOp(.activate_constraint, limit, .{});
    try b.expectAwake(driven);
    try b.expectAwake(at_limit);
}

/// Sliders across cross save / restore: the warm start impulses of the four parts, the motor state and the targets are
/// state. A snapshot, then motor state and target changes, then a restore of the other world's snapshot brings them
/// back; a snapshot of the constraints only; other physics settings (no warm start, other step counts and Baumgarte)
fn buildSliderSaveRestore(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.num_updates = 90;
    const state_constraints: u32 = 8; // EStateRecorderState::Constraints
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

    // A chain: sliders between neighbours with limits and motors, hanging from the world on a point constraint
    var previous: u32 = invalid_handle;
    var links: [5]u32 = undefined;
    var bodies: [5]u32 = undefined;
    for (0..5) |i| {
        const x: Real = 0.5 * real(i);
        const h = try b.body(.{ .position = .{ x, 6, 0 }, .angular_velocity = .{ 0, 0, 0.3 }, .shape = box }, .activate);
        if (i == 0) {
            links[i] = try b.constraint(Desc.pointWorld(invalid_handle, h, .{ x - 0.25, 6, 0 }));
        } else {
            var s = Desc.sliderAxes(previous, h, .{ x - 0.25, 6, 0 }, sliderUnit(.{ 1, 0.2, 0 }), sliderUnit(.{ -0.2, 1, 0 }));
            s.f[12] = -0.15;
            s.f[13] = 0.15;
            if (i % 2 == 0) Desc.setSpring(&s, 3, 14, 0, 5, 0.5);
            s.f[16] = 20;
            links[i] = try b.constraint(s);
        }
        bodies[i] = h;
        previous = h;
    }
    try setSliderMotorState(b, links[1], motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, links[1], .{ .f = &.{0.3} });
    try setSliderMotorState(b, links[3], motor_position);
    try b.constraintOp(.slider_set_target_position, links[3], .{ .f = &.{0.1} });
    b.at(15);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(18);
    try setSliderMotorState(b, links[1], motor_position_and_velocity);
    try b.constraintOp(.slider_set_target_position, links[3], .{ .f = &.{-0.1} });
    try b.constraintOp(.slider_set_target_velocity, links[2], .{ .f = &.{1} });
    try b.sysOp(.snapshot, .{ .index = 1, .i = &.{state_constraints} });
    b.at(25);
    try setSliderMotorState(b, links[1], motor_off);
    try setSliderMotorState(b, links[4], motor_velocity);
    try b.sysOp(.set_physics_settings, .{ .index = settings1 });
    b.at(30);
    try b.sysOp(.restore_cross, .{ .index = 1 }); // The constraints of update 18: the motor states and targets come back
    b.at(40);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Everything of update 15
    try b.sysOp(.set_physics_settings, .{ .index = settings2 });
    b.at(55);
    try b.sysOp(.snapshot, .{ .index = 2 });
    try b.op(.add_impulse, bodies[4], .{ .f = &.{ 0, 0, 3 } });
    try b.constraintOp(.reset_constraint_warm_start, links[2], .{});
    b.at(70);
    try b.sysOp(.restore_cross, .{ .index = 2 });
}

/// Sliders with limits, springs and motors with several collision steps per update and changing update parameters
/// (the warm start impulses are scaled by the step ratio)
fn buildSliderCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const box = try b.box(.{ 0.2, 0.3, 0.2 }, 0.05);
            b.num_updates = 50;
            b.collision_steps = steps;
            b.delta_time = @as(f32, @floatFromInt(steps)) / 60.0;
            var previous: u32 = invalid_handle;
            for (0..5) |i| {
                const x: Real = 0.45 * real(i);
                const h = try b.body(.{ .position = .{ x, 6, 0 }, .shape = box }, .activate);
                var s = sliderWithLimits(previous, h, sliderUnit(.{ 1, -0.5, 0 }), -0.2, 0.3);
                if (i % 2 == 1) Desc.setSpring(&s, 3, 14, 1, 800, 20);
                const c = try b.constraint(s);
                if (i == 2) {
                    try setSliderMotorState(b, c, motor_velocity);
                    try b.constraintOp(.slider_set_target_velocity, c, .{ .f = &.{-0.5} });
                } else if (i == 4) {
                    try setSliderMotorState(b, c, motor_position);
                    try b.constraintOp(.slider_set_target_position, c, .{ .f = &.{0.2} });
                }
                previous = h;
            }
            b.at(20);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 90.0}, .count = 1 });
            b.at(30);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 20.0}, .count = steps + 1 });
        }
    }.build;
}

/// Sliders far from the origin (the world space points are Real: in double precision the anchors keep their
/// precision): a box on a slider from the world with a velocity motor pushes a box over a floor, a slider pair with a
/// soft limit falls onto the floor
fn buildSliderFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    b.num_updates = 60;
    const o: [3]Real = .{ 5000, 200, -12000 };
    const floor = try b.box(.{ 10, 1, 10 }, 0);
    _ = try b.body(.{ .position = .{ o[0], o[1] - 1, o[2] }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const pusher = try b.body(.{ .position = .{ o[0] - 3, o[1] + 0.3, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
    const push = try b.constraint(sliderWithLimits(invalid_handle, pusher, .{ 1, 0, 0 }, -0.5, 4));
    try setSliderMotorState(b, push, motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, push, .{ .f = &.{3} });
    _ = try b.body(.{ .position = .{ o[0] - 1.5, o[1] + 0.25, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
    const p1 = try b.body(.{ .position = .{ o[0] + 3, o[1] + 2, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
    const p2 = try b.body(.{ .position = .{ o[0] + 3.6, o[1] + 2.2, o[2] + 0.1 }, .shape = box, .object_layer = Layers.moving2 }, .activate);
    var s = Desc.slider(p1, p2, sliderUnit(.{ 1, 0.3, 0 }));
    s.f[12] = -0.3;
    s.f[13] = 0.1;
    Desc.setSpring(&s, 3, 14, 0, 6, 0.5);
    _ = try b.constraint(s);
}

/// A random slider description between two body handles: a random space (world: given points and axes, auto detect, or
/// SetSliderAxis; local: points near the centers of mass and random frames), limits (none, one sided, both sides, equal
/// with a soft spring), a limits spring, friction and a motor (sometimes without stiffness)
fn randomSliderDescription(b: *SceneBuilder, body1: u32, body2: u32, p: [3]Real) SimConstraint {
    var s = Desc.defaults(.slider);
    s.body1 = body1;
    s.body2 = body2;
    const frame1 = sliderFrame(b.rotation());
    const frame2 = if (b.oneIn(2)) frame1 else sliderFrame(b.rotation());
    @memcpy(s.f[0..3], &frame1[0]);
    @memcpy(s.f[3..6], &frame1[1]);
    @memcpy(s.f[6..9], &frame2[0]);
    @memcpy(s.f[9..12], &frame2[1]);
    if (b.oneIn(4)) {
        s.i[0] = ConstraintSpace.local_to_body_com;
        Desc.setR3(s.r[0..], r3(b.vec3(-0.3, 0.3)));
        Desc.setR3(s.r[3..], r3(b.vec3(-0.3, 0.3)));
    } else {
        s.i[1] = @intFromBool(b.oneIn(3)); // mAutoDetectPoint
        s.i[2] = @intFromBool(b.oneIn(3)); // SetSliderAxis
        Desc.setR3(s.r[0..], p);
        Desc.setR3(s.r[3..], if (b.oneIn(2)) p else .{ p[0] + real(b.float(-0.2, 0.2)), p[1] + real(b.float(-0.2, 0.2)), p[2] });
    }
    const soft = !b.oneIn(3);
    Desc.setSpring(&s, 3, 14, b.index(3), if (soft) b.float(2, 8) else 0, b.float(0, 1));
    if (s.i[3] != 0 and soft) s.f[14] *= 100; // Stiffness
    switch (b.index(5)) {
        0 => {}, // No limits
        1 => s.f[13] = b.float(0, 0.5), // One sided
        2 => s.f[12] = -b.float(0, 0.5),
        3 => if (soft) {
            s.f[12] = 0; // Equal limits with a soft spring
            s.f[13] = 0;
        },
        else => {
            s.f[12] = -b.float(0.05, 0.5);
            s.f[13] = b.float(0.05, 0.5);
        },
    }
    if (b.oneIn(3)) s.f[16] = b.float(0, 200); // Friction
    Desc.setMotor(&s, 4, 17, b.index(3), .{ if (b.oneIn(4)) 0 else b.float(1, 5), b.float(0, 1), -b.float(100, 2000), b.float(100, 2000), -flt_max, flt_max });
    if (s.i[4] != 0) s.f[17] *= 100; // Stiffness
    return s;
}

/// Random bodies (dynamic, kinematic, static; every basic shape) in a container with random slider constraints between
/// them and to the world (randomSliderDescription) and some point and fixed constraints, and random slider operations
/// between the updates (motor states, targets, limits, springs, friction, motor settings) and generic ones
fn buildSliderRandom(comptime seed: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.rng = 0x511d_e700 +% seed *% 0x9e3779b9;
            const shapes = try BasicShapes.add(b);
            b.num_updates = 80;
            b.max_contact_constraints = 2048;
            try addContainer(b, 6, 2);
            const num_bodies = 20;
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
            const num_constraints = 16;
            var handles: [num_constraints]u32 = undefined;
            var is_slider: [num_constraints]bool = undefined;
            for (&handles, &is_slider) |*c, *slider| {
                const index1 = b.index(num_bodies + 1); // num_bodies: the world
                var index2 = b.index(num_bodies);
                if (index2 == index1) index2 = (index2 + 1) % num_bodies;
                const body1 = if (index1 == num_bodies) invalid_handle else bodies[index1];
                const body2 = bodies[index2];
                const p = r3(b.vec3(-4, 4));
                slider.* = !b.oneIn(4);
                var desc = if (slider.*)
                    randomSliderDescription(b, body1, body2, .{ p[0], 1 + p[1] / 2, p[2] })
                else if (b.oneIn(2))
                    Desc.fixedAuto(body1, body2)
                else
                    Desc.pointWorld(body1, body2, p);
                desc.constraint_priority = b.index(3);
                if (b.oneIn(5)) desc.num_velocity_steps_override = 1 + b.index(15);
                if (b.oneIn(8)) desc.enabled = 0;
                c.* = try b.constraint(desc);
            }
            var update: u32 = 3;
            while (update < b.num_updates) : (update += 1 + b.index(6)) {
                b.at(update);
                for (0..1 + b.index(3)) |_| {
                    const k = b.index(num_constraints);
                    const c = handles[k];
                    if (!is_slider[k]) {
                        try b.constraintOp(.set_constraint_enabled, c, .{ .i = &.{@intFromBool(!b.oneIn(3))} });
                        continue;
                    }
                    switch (b.index(10)) {
                        0 => try b.constraintOp(.slider_set_max_friction_force, c, .{ .f = &.{if (b.oneIn(3)) 0 else b.float(0, 300)} }),
                        1, 2 => try setSliderMotorState(b, c, b.index(4)),
                        3 => try b.constraintOp(.slider_set_target_velocity, c, .{ .f = &.{b.float(-2, 2)} }),
                        4 => try b.constraintOp(.slider_set_target_position, c, .{ .f = &.{b.float(-1, 1)} }),
                        5 => {
                            const min: f32 = if (b.oneIn(4)) -flt_max else -b.float(0, 0.6);
                            const max: f32 = if (b.oneIn(4)) flt_max else b.float(0, 0.6);
                            try b.constraintOp(.slider_set_limits, c, .{ .f = &.{ min, max } });
                        },
                        6 => {
                            const mode = b.index(3);
                            try b.constraintOp(.slider_set_limits_spring_settings, c, .{ .i = &.{mode}, .f = &.{ if (b.oneIn(3)) 0 else if (mode == 0) b.float(2, 8) else b.float(200, 800), b.float(0, 1) } });
                        },
                        7 => {
                            const mode = b.index(3);
                            try b.constraintOp(.slider_set_motor_settings, c, .{ .i = &.{mode}, .f = &.{ if (b.oneIn(4)) 0 else if (mode == 0) b.float(1, 5) else b.float(100, 500), b.float(0, 1), -b.float(100, 2000), b.float(100, 2000), -flt_max, flt_max } });
                        },
                        8 => try b.constraintOp(.reset_constraint_warm_start, c, .{}),
                        else => try b.op(.add_impulse, bodies[b.index(num_bodies)], .{ .f = &b.vec3(-3, 3) }),
                    }
                }
            }
        }
    }.build;
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

/// The EMotorState values of the swing twist motor operations
const MotorStateValue = struct {
    const off: u32 = 0;
    const velocity: u32 = 1;
    const position: u32 = 2;
    const position_and_velocity: u32 = 3;
};

/// The ESpringMode values of the motor springs
const SpringModeValue = struct {
    const frequency_and_damping: u32 = 0;
    const stiffness_and_damping: u32 = 1;
    const mass_normalized_stiffness_and_damping: u32 = 2;
};

/// Sets the swing motor (mode i[2], f[17..23]) or the twist motor (mode i[3], f[23..29]) of a swing twist description
fn setSwingTwistMotor(c: *SimConstraint, twist: bool, mode: u32, values: [6]f32) void {
    if (twist) Desc.setMotor(c, 3, 23, mode, values) else Desc.setMotor(c, 2, 17, mode, values);
}

/// A swing twist description in world space with the swing type and the max friction torque
fn swingTwistDesc(body1: u32, body2: u32, p: [3]Real, twist_axis: [3]f32, plane_axis: [3]f32, limits: [4]f32, swing_type: u32, friction: f32) SimConstraint {
    var c = Desc.swingTwist(body1, body2, p, twist_axis, plane_axis, limits);
    c.i[1] = swing_type;
    c.f[16] = friction;
    return c;
}

/// (x, y, z) -> (x, -z, y): a quarter turn around X (the parts of a lying ragdoll, see `addRagdoll`)
fn turnVector(v: [3]f32, turned: bool) [3]f32 {
    return if (turned) .{ v[0], -v[2], v[1] } else v;
}

/// The position of a ragdoll part or joint at `offset` from the pelvis at `origin` (turned for a lying ragdoll)
fn ragdollPlace(origin: [3]Real, offset: [3]f32, turned: bool) [3]Real {
    const o = turnVector(offset, turned);
    return .{ origin[0] + o[0], origin[1] + o[1], origin[2] + o[2] };
}

/// The constraints of `addSwingTwistRagdoll`
const SwingTwistRagdoll = struct {
    /// The body handle of the pelvis (the parts follow in `addRagdoll`'s order)
    pelvis: u32,
    /// The constraint handles: lower back, chest weld, neck, shoulders, elbows, hips, knees
    lower_back: u32,
    neck: u32,
    shoulders: [2]u32,
    elbows: [2]u32,
    hips: [2]u32,
    knees: [2]u32,
};

/// The ragdoll of `addRagdoll` (the same 12 parts) with Jolt's ragdoll joints: swing twist joints at the lower back (cone
/// swing), the shoulders (cone swing with friction), the elbows (pyramid swing that only bends one way: an almost hinge
/// with a locked swing axis) and the hips (pyramid swing), cone joints at the neck and the knees, the chest welded to the
/// spine (a FixedConstraint with auto detect point): 11 constraints, 7 swing twists, 3 cones, 1 fixed. `hip_motor` puts
/// a position motor (spring `hip_spring_mode`) on the swing and twist of the hips with the rest pose as the target.
fn addSwingTwistRagdoll(b: *SceneBuilder, shapes: RagdollShapes, base: [3]Real, layer: u32, angular_velocity: [3]f32, lying: bool, hip_motor: bool) Allocator.Error!SwingTwistRagdoll {
    const rotation: [4]f32 = if (lying) axisAngle(.{ 1, 0, 0 }, 0.5 * std.math.pi) else identity_rotation;
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
        h.* = try b.body(.{ .position = ragdollPlace(base, part.offset, lying), .rotation = rotation, .shape = part.shape, .angular_velocity = angular_velocity, .linear_damping = 0.3, .angular_damping = 0.6, .object_layer = layer }, .activate);

    const up = turnVector(.{ 0, 1, 0 }, lying);
    const down = turnVector(.{ 0, -1, 0 }, lying);
    const right = turnVector(.{ 1, 0, 0 }, lying);
    const forward = turnVector(.{ 0, 0, 1 }, lying);
    const joint = struct {
        fn swingTwist(bb: *SceneBuilder, origin: [3]Real, turned: bool, parent: u32, child: u32, point: [3]f32, twist: [3]f32, plane: [3]f32, limits: [4]f32, swing_type: u32, friction: f32) Allocator.Error!u32 {
            return bb.constraint(swingTwistDesc(parent, child, ragdollPlace(origin, point, turned), twist, plane, limits, swing_type, friction));
        }
        fn cone(bb: *SceneBuilder, origin: [3]Real, turned: bool, parent: u32, child: u32, point: [3]f32, twist: [3]f32, half_cone_angle: f32) Allocator.Error!u32 {
            return bb.constraint(Desc.cone(parent, child, ragdollPlace(origin, point, turned), twist, half_cone_angle));
        }
    };
    var r: SwingTwistRagdoll = undefined;
    r.pelvis = handles[0];
    r.lower_back = try joint.swingTwist(b, base, lying, handles[0], handles[1], .{ 0, 0.115, 0 }, up, right, .{ 0.3, 0.2, -0.2, 0.2 }, 0, 0);
    _ = try b.constraint(Desc.fixedAuto(handles[1], handles[2])); // The chest is welded to the spine
    r.neck = try joint.cone(b, base, lying, handles[2], handles[3], .{ 0, 0.67, 0 }, up, 0.5);
    r.shoulders[0] = try joint.swingTwist(b, base, lying, handles[2], handles[4], .{ -0.21, 0.6, 0 }, down, right, .{ 0.9, 0.6, -0.6, 0.6 }, 0, 0.3);
    r.shoulders[1] = try joint.swingTwist(b, base, lying, handles[2], handles[5], .{ 0.21, 0.6, 0 }, down, right, .{ 0.9, 0.6, -0.6, 0.6 }, 0, 0.3);
    // Elbows: rotation around the plane axis (swing Z, the normal half cone angle) is locked, the arm bends around the
    // normal axis (swing Y, the plane half cone angle)
    r.elbows[0] = try joint.swingTwist(b, base, lying, handles[4], handles[6], .{ -0.3, 0.32, 0 }, down, forward, .{ 0, 1.2, -0.1, 0.1 }, 1, 0);
    r.elbows[1] = try joint.swingTwist(b, base, lying, handles[5], handles[7], .{ 0.3, 0.32, 0 }, down, forward, .{ 0, 1.2, -0.1, 0.1 }, 1, 0);
    for (0..2) |side| {
        const x: f32 = if (side == 0) -0.09 else 0.09;
        var hip = swingTwistDesc(handles[0], handles[8 + side], ragdollPlace(base, .{ x, -0.115, 0 }, lying), down, forward, .{ 0.7, 0.4, -0.3, 0.3 }, 1, 0);
        if (hip_motor) {
            setSwingTwistMotor(&hip, false, SpringModeValue.stiffness_and_damping, .{ 30, 2, -flt_max, flt_max, -20, 20 });
            setSwingTwistMotor(&hip, true, SpringModeValue.frequency_and_damping, .{ 3, 0.8, -flt_max, flt_max, -5, 5 });
        }
        r.hips[side] = try b.constraint(hip);
        r.knees[side] = try joint.cone(b, base, lying, handles[8 + side], handles[10 + side], .{ x, -0.545, 0 }, down, 0.6);
    }
    if (hip_motor) {
        for (r.hips) |h| {
            try b.constraintOp(.swing_twist_set_swing_motor_state, h, .{ .i = &.{MotorStateValue.position} });
            try b.constraintOp(.swing_twist_set_twist_motor_state, h, .{ .i = &.{MotorStateValue.position} });
        }
    }
    return r;
}

/// 8 ragdolls with swing twist and cone joints (`addSwingTwistRagdoll`, half of them with hip motors) fall in two
/// columns onto a floor and onto each other and tumble: the joint limits are reached, contacts between the parts of
/// different ragdolls, cones, swing twists and fixed constraints in one island that the LargeIslandSplitter splits
fn buildSwingTwistRagdollPile(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try RagdollShapes.add(b);
    b.num_updates = 120;
    b.max_contact_constraints = 4096;
    _ = try addFloor(b, 20, 0);
    for (0..8) |i| {
        const fi: f32 = @floatFromInt(i);
        const column: Real = if (i % 2 == 0) -0.4 else 0.4;
        const base: [3]Real = .{ column, 1.2 + 2.0 * real(i / 2), real(0.2 * @sin(fi)) };
        _ = try addSwingTwistRagdoll(b, shapes, base, Layers.moving, .{ 0.8 * @sin(2 * fi), 0.5, 0.8 * @cos(3 * fi) }, false, i % 2 == 1);
    }
}

/// 8 swing twist ragdolls lying on their backs in a row, holding hands (a cone constraint between the lower arms of
/// neighbours): one island of 95 constraints and the floor contacts that the LargeIslandSplitter splits, which settles
/// and falls asleep from the final batch of a split island; a push wakes the island up again through its constraints
fn buildSwingTwistRagdollsLying(b: *SceneBuilder) Allocator.Error!void {
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
        h.* = (try addSwingTwistRagdoll(b, shapes, base, Layers.moving, @splat(0), true, i % 3 == 0)).pelvis;
        if (i > 0) _ = try b.constraint(Desc.cone(pelvis[i - 1] + lower_arm_right, h.* + lower_arm_left, .{ base[0] - 0.375, 0.2, 0.16 }, .{ 1, 0, 0 }, 0.4));
    }
    b.at(95);
    try b.expectSleeping(pelvis[0]);
    try b.op(.add_impulse, pelvis[5], .{ .f = &.{ 0, 60, 0 } });
}

/// Swing twist motors in every EMotorState with springs in every ESpringMode, friction and the limits, and cone
/// constraints in every situation. Five arms hang from the world on swing twist shoulders (cone and pyramid swing):
/// - friction only (swing and twist off), the arm spins against it;
/// - velocity motors that drive the arm into the swing and twist limits, then a twist position motor (frequency mode) and
///   the swing motor off without friction (the motor parts are deactivated), then the twist motor off;
/// - position motors (stiffness mode) towards a target orientation outside the limits (clamped by SetTargetOrientationCS),
///   then position and velocity motors with a target angular velocity in body space;
/// - a swing position motor without stiffness (the parts are deactivated) and a twist position and velocity motor with
///   damping only, small torque limits;
/// - mass normalized springs, asymmetric torque limits, target orientations in body space, motor settings changed while
///   the motors run, the limits changed while they are reached.
/// The cones: a ball tilted beyond its cone (the limit pushes it back at once) with the world as body 2, a box on a cone
/// of half angle 0 from another box (the twist axes stay aligned), a cone of half angle pi (never active), a cone in
/// local space from a kinematic body that moves and turns, SetHalfConeAngle while the limit is reached. A swing twist in
/// local space from the kinematic body.
fn buildSwingTwistMotors(b: *SceneBuilder) Allocator.Error!void {
    const limb = try b.box(.{ 0.1, 0.4, 0.1 }, 0.05);
    const ball = try b.sphere(0.2);
    b.num_updates = 120;

    var arms: [5]u32 = undefined;
    var shoulders: [5]u32 = undefined;
    for (0..5) |k| {
        const x: Real = 1.5 * real(k);
        arms[k] = try b.body(.{ .position = .{ x, 4.5, 0 }, .shape = limb, .angular_velocity = if (k == 0) .{ 0.5, 3, 0.2 } else @splat(0) }, .activate);
        var d = swingTwistDesc(invalid_handle, arms[k], .{ x, 4.9, 0 }, .{ 0, -1, 0 }, .{ 1, 0, 0 }, .{ 0.8, 0.5, -0.7, 0.7 }, @intCast(k % 2), if (k == 0) 0.4 else 0);
        switch (k) {
            1 => setSwingTwistMotor(&d, true, SpringModeValue.frequency_and_damping, .{ 4, 0.5, -flt_max, flt_max, -flt_max, flt_max }),
            2 => {
                setSwingTwistMotor(&d, false, SpringModeValue.stiffness_and_damping, .{ 60, 3, -flt_max, flt_max, -40, 40 });
                setSwingTwistMotor(&d, true, SpringModeValue.stiffness_and_damping, .{ 40, 2, -flt_max, flt_max, -40, 40 });
            },
            3 => {
                setSwingTwistMotor(&d, false, SpringModeValue.stiffness_and_damping, .{ 0, 4, -flt_max, flt_max, -1, 1 });
                setSwingTwistMotor(&d, true, SpringModeValue.stiffness_and_damping, .{ 0, 4, -flt_max, flt_max, -0.2, 0.2 });
            },
            4 => {
                setSwingTwistMotor(&d, false, SpringModeValue.mass_normalized_stiffness_and_damping, .{ 50, 5, -flt_max, flt_max, -1, 6 });
                setSwingTwistMotor(&d, true, SpringModeValue.mass_normalized_stiffness_and_damping, .{ 80, 2, -flt_max, flt_max, -6, 1 });
            },
            else => {},
        }
        shoulders[k] = try b.constraint(d);
    }

    // The cones
    // A ball on a cone from the world (body 2): the twist axis of the ball points from the pivot to the ball, the one of
    // the world down, 45 degrees apart (outside the cone)
    const tilted = try b.body(.{ .position = .{ -2, 4, 0 }, .shape = ball }, .activate);
    var tilted_desc = Desc.cone(tilted, invalid_handle, .{ -2.6, 4.6, 0 }, .{ 0.70710677, -0.70710677, 0 }, 0.3);
    Desc.set3(tilted_desc.f[3..], .{ 0, -1, 0 });
    const tilted_cone = try b.constraint(tilted_desc);
    const upper = try b.body(.{ .position = .{ -4, 4.5, 0 }, .shape = limb }, .activate);
    _ = try b.constraint(Desc.pointWorld(invalid_handle, upper, .{ -4, 4.9, 0 }));
    const lower = try b.body(.{ .position = .{ -4, 3.7, 0 }, .shape = limb, .angular_velocity = .{ 0, 0, 2 } }, .activate);
    _ = try b.constraint(Desc.cone(upper, lower, .{ -4, 4.1, 0 }, .{ 0, -1, 0 }, 0));
    const free = try b.body(.{ .position = .{ -6, 4.5, 0 }, .shape = limb, .angular_velocity = .{ 2, 0, 1 } }, .activate);
    _ = try b.constraint(Desc.cone(invalid_handle, free, .{ -6, 4.9, 0 }, .{ 0, -1, 0 }, std.math.pi));
    const kinematic = try b.body(.{ .position = .{ 10, 5, 0 }, .shape = limb, .motion_type = 1 }, .activate);
    const on_kinematic = try b.body(.{ .position = .{ 10, 4.2, 0 }, .shape = limb }, .activate);
    var local_cone = Desc.cone(kinematic, on_kinematic, .{ 0, -0.4, 0 }, .{ 0, -1, 0 }, 0.4);
    local_cone.i[0] = ConstraintSpace.local_to_body_com;
    local_cone.r[3] = 0;
    local_cone.r[4] = 0.4;
    local_cone.r[5] = 0;
    const kinematic_cone = try b.constraint(local_cone);
    const on_kinematic2 = try b.body(.{ .position = .{ 11, 4.5, 0 }, .shape = limb }, .activate);
    var local_swing_twist = swingTwistDesc(kinematic, on_kinematic2, .{ 0.5, 0.1, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0.6, 0.6, -1, 1 }, 0, 0.1);
    local_swing_twist.i[0] = ConstraintSpace.local_to_body_com;
    local_swing_twist.r[3] = -0.5;
    local_swing_twist.r[4] = 0.6;
    local_swing_twist.r[5] = 0;
    const kinematic_swing_twist = try b.constraint(local_swing_twist);

    // Arm 1: velocity motors into the limits
    try b.constraintOp(.swing_twist_set_twist_motor_state, shoulders[1], .{ .i = &.{MotorStateValue.velocity} });
    try b.constraintOp(.swing_twist_set_swing_motor_state, shoulders[1], .{ .i = &.{MotorStateValue.velocity} });
    try b.constraintOp(.swing_twist_set_target_angular_velocity_cs, shoulders[1], .{ .f = &.{ 3, 1.5, -1 } });
    // Arm 2: position motors towards a target outside the limits (clamped)
    try b.constraintOp(.swing_twist_set_swing_motor_state, shoulders[2], .{ .i = &.{MotorStateValue.position} });
    try b.constraintOp(.swing_twist_set_twist_motor_state, shoulders[2], .{ .i = &.{MotorStateValue.position} });
    try b.constraintOp(.swing_twist_set_target_orientation_cs, shoulders[2], .{ .f = &axisAngle(normalize3(.{ 0.6, 0.8, 0 }), 1.6) });
    // Arm 3: a swing position motor without stiffness, a twist position and velocity motor with damping only
    try b.constraintOp(.swing_twist_set_swing_motor_state, shoulders[3], .{ .i = &.{MotorStateValue.position} });
    try b.constraintOp(.swing_twist_set_twist_motor_state, shoulders[3], .{ .i = &.{MotorStateValue.position_and_velocity} });
    try b.constraintOp(.swing_twist_set_target_angular_velocity_cs, shoulders[3], .{ .f = &.{ 4, 0.5, 0.5 } });
    // Arm 4: mass normalized springs towards a target in body space
    try b.constraintOp(.swing_twist_set_swing_motor_state, shoulders[4], .{ .i = &.{MotorStateValue.position_and_velocity} });
    try b.constraintOp(.swing_twist_set_twist_motor_state, shoulders[4], .{ .i = &.{MotorStateValue.position} });
    try b.constraintOp(.swing_twist_set_target_orientation_bs, shoulders[4], .{ .f = &axisAngle(.{ 0, 0, 1 }, 0.4) });
    try b.constraintOp(.swing_twist_set_target_angular_velocity_bs, shoulders[4], .{ .f = &.{ 0, 0.2, 0.3 } });
    // The kinematic body moves and turns
    try b.op(.set_linear_velocity, kinematic, .{ .f = &.{ 0.3, 0, 0.2 } });
    try b.op(.set_angular_velocity, kinematic, .{ .f = &.{ 0, 0.8, 0.4 } });

    b.at(30);
    try b.constraintOp(.swing_twist_set_twist_motor_state, shoulders[1], .{ .i = &.{MotorStateValue.position} });
    try b.constraintOp(.swing_twist_set_swing_motor_state, shoulders[1], .{ .i = &.{MotorStateValue.off} });
    try b.constraintOp(.swing_twist_set_target_orientation_cs, shoulders[1], .{ .f = &axisAngle(.{ 1, 0, 0 }, -0.4) });
    try b.constraintOp(.swing_twist_set_swing_motor_state, shoulders[2], .{ .i = &.{MotorStateValue.position_and_velocity} });
    try b.constraintOp(.swing_twist_set_twist_motor_state, shoulders[2], .{ .i = &.{MotorStateValue.position_and_velocity} });
    try b.constraintOp(.swing_twist_set_target_angular_velocity_bs, shoulders[2], .{ .f = &.{ 0.5, -0.5, 0.2 } });
    try b.constraintOp(.swing_twist_set_swing_motor_settings, shoulders[4], .{ .i = &.{SpringModeValue.frequency_and_damping}, .f = &.{ 1.5, 0.3, -flt_max, flt_max, -3, 3 } });
    try b.constraintOp(.cone_set_half_cone_angle, tilted_cone, .{ .f = &.{0.6} });
    try b.constraintOp(.constraint_settings_hash, shoulders[2], .{});
    b.at(50);
    try b.constraintOp(.swing_twist_set_normal_half_cone_angle, shoulders[1], .{ .f = &.{0.2} });
    try b.constraintOp(.swing_twist_set_plane_half_cone_angle, shoulders[1], .{ .f = &.{0.3} });
    try b.constraintOp(.swing_twist_set_twist_min_angle, shoulders[2], .{ .f = &.{-0.2} });
    try b.constraintOp(.swing_twist_set_twist_max_angle, shoulders[2], .{ .f = &.{0.1} });
    try b.constraintOp(.swing_twist_set_max_friction_torque, shoulders[0], .{ .f = &.{2} });
    try b.constraintOp(.swing_twist_set_twist_motor_settings, shoulders[3], .{ .i = &.{SpringModeValue.stiffness_and_damping}, .f = &.{ 20, 1, -flt_max, flt_max, -0.5, 2 } });
    try b.constraintOp(.swing_twist_set_swing_motor_state, shoulders[3], .{ .i = &.{MotorStateValue.velocity} });
    try b.constraintOp(.swing_twist_set_target_orientation_bs, shoulders[4], .{ .f = &axisAngle(.{ 1, 0, 0 }, 0.9) });
    try b.constraintOp(.cone_set_half_cone_angle, kinematic_cone, .{ .f = &.{0.1} });
    try b.op(.set_angular_velocity, kinematic, .{ .f = &.{ 0.7, 0, -0.5 } });
    b.at(80);
    try b.constraintOp(.swing_twist_set_twist_motor_state, shoulders[1], .{ .i = &.{MotorStateValue.off} });
    try b.constraintOp(.swing_twist_set_swing_motor_state, shoulders[2], .{ .i = &.{MotorStateValue.velocity} });
    try b.constraintOp(.swing_twist_set_target_angular_velocity_cs, shoulders[2], .{ .f = &.{ 0, 0, 0 } });
    try b.constraintOp(.reset_constraint_warm_start, shoulders[4], .{});
    try b.constraintOp(.notify_constraint_shape_changed, kinematic_swing_twist, .{ .body = on_kinematic2, .f = &.{ 0, 0.05, 0 } });
    try b.constraintOp(.constraint_settings_hash, kinematic_swing_twist, .{});
    try b.constraintOp(.constraint_settings_hash, kinematic_cone, .{});
    try b.constraintOp(.constraint_settings_hash, tilted_cone, .{});
}

/// A normalized [3]f32 (computed in f64 by the scene builder)
fn normalize3(v: [3]f32) [3]f32 {
    const len = @sqrt(@as(f64, v[0]) * v[0] + @as(f64, v[1]) * v[1] + @as(f64, v[2]) * v[2]);
    return .{ @floatCast(v[0] / len), @floatCast(v[1] / len), @floatCast(v[2] / len) };
}

/// A chain hanging from the world: links alternate between swing twist joints (cone swing, pyramid swing) with motors and
/// cone joints, 6 boxes; returns the constraint handles
fn addConeSwingTwistChain(b: *SceneBuilder, shape: u32, origin: [3]Real, layer: u32, handles: *[6]u32, bodies: *[6]u32) Allocator.Error!void {
    var previous: u32 = invalid_handle;
    for (0..6) |i| {
        const x: Real = origin[0] + 0.5 * real(i);
        const y: Real = origin[1];
        const z: Real = origin[2];
        const h = try b.body(.{ .position = .{ x, y, z }, .shape = shape, .angular_velocity = .{ 0, 0.4, 0.3 }, .object_layer = layer }, .activate);
        const p: [3]Real = .{ x - 0.25, y, z };
        handles[i] = if (i % 2 == 0)
            try b.constraint(swingTwistDesc(previous, h, p, .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0.5, 0.4, -0.3, 0.6 }, @intCast((i / 2) % 2), 0.05))
        else
            try b.constraint(Desc.cone(previous, h, p, .{ 1, 0, 0 }, 0.35));
        bodies[i] = h;
        previous = h;
    }
}

/// Cone and swing twist constraints across cross save / restore and partial states: the motor states, the targets, the
/// warm start impulses and the rotation axis of the cones are state. Snapshots of everything and of the constraints
/// only, motor states and targets changed between snapshot and restore, ResetWarmStart before a restore, other physics
/// settings (no warm start, more steps)
fn buildConeSwingTwistSaveRestore(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.25, 0.12, 0.12 }, 0.05);
    b.num_updates = 90;
    const state_constraints: u32 = 8; // EStateRecorderState::Constraints
    const state_bodies: u32 = 2; // EStateRecorderState::Bodies
    var no_warm_start = b.settings;
    no_warm_start.constraint_warm_start = 0;
    no_warm_start.num_velocity_steps = 4;
    const settings1 = try b.addPhysicsSettings(no_warm_start);
    var links: [6]u32 = undefined;
    var bodies: [6]u32 = undefined;
    try addConeSwingTwistChain(b, box, .{ 0, 6, 0 }, Layers.unused1, &links, &bodies);
    try b.constraintOp(.swing_twist_set_twist_motor_state, links[0], .{ .i = &.{MotorStateValue.velocity} });
    try b.constraintOp(.swing_twist_set_target_angular_velocity_cs, links[0], .{ .f = &.{ 1, 0, 0 } });
    try b.constraintOp(.swing_twist_set_swing_motor_state, links[2], .{ .i = &.{MotorStateValue.position} });
    try b.constraintOp(.swing_twist_set_target_orientation_cs, links[2], .{ .f = &axisAngle(.{ 0, 0, 1 }, 0.3) });
    b.at(15);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(20);
    try b.constraintOp(.swing_twist_set_twist_motor_state, links[0], .{ .i = &.{MotorStateValue.off} });
    try b.constraintOp(.swing_twist_set_swing_motor_state, links[2], .{ .i = &.{MotorStateValue.position_and_velocity} });
    try b.constraintOp(.swing_twist_set_target_orientation_cs, links[2], .{ .f = &axisAngle(.{ 0, 1, 0 }, -0.3) });
    try b.constraintOp(.swing_twist_set_target_angular_velocity_cs, links[2], .{ .f = &.{ 0, 0.5, 0 } });
    try b.sysOp(.snapshot, .{ .index = 1, .i = &.{state_constraints} });
    b.at(30);
    try b.constraintOp(.swing_twist_set_twist_motor_state, links[4], .{ .i = &.{MotorStateValue.velocity} });
    try b.constraintOp(.reset_constraint_warm_start, links[1], .{});
    try b.sysOp(.restore_cross, .{ .index = 0 }); // The motors of update 15 again
    b.at(40);
    try b.sysOp(.set_physics_settings, .{ .index = settings1 });
    try b.constraintOp(.swing_twist_set_swing_motor_state, links[2], .{ .i = &.{MotorStateValue.off} });
    try b.sysOp(.restore_cross, .{ .index = 1 }); // The constraints of update 20: the swing motor of links[2] runs again
    b.at(50);
    try b.sysOp(.snapshot, .{ .index = 2, .i = &.{state_bodies | state_constraints} });
    try b.op(.add_impulse, bodies[5], .{ .f = &.{ 0, 0, 2 } });
    try b.constraintOp(.cone_set_half_cone_angle, links[3], .{ .f = &.{0.05} });
    b.at(60);
    try b.sysOp(.restore_cross, .{ .index = 2 });
    try b.constraintOp(.reset_constraint_warm_start, links[5], .{});
    b.at(70);
    try b.sysOp(.snapshot, .{ .index = 3 });
    b.at(75);
    try b.sysOp(.restore_cross, .{ .index = 3 });
}

/// Cone and swing twist constraints and sleeping bodies (without contacts): a chain held up by a swing position motor
/// comes to rest and falls asleep; a new target orientation does not wake it (the setters do not activate bodies), a
/// force on its lowest box wakes the chain link by link; a cone between two sleeping bodies is not active until
/// ActivateConstraint; a swing twist between an awake body and a sleeping one wakes the sleeping one from inside the
/// update
fn buildConeSwingTwistSleeping(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.12, 0.25, 0.12 }, 0.05);
    b.num_updates = 120;
    b.settings.time_before_sleep = 0.2;
    var chain: [4]u32 = undefined;
    var links: [4]u32 = undefined;
    var previous: u32 = invalid_handle;
    for (0..4) |i| {
        const y: Real = 5 - 0.55 * real(i);
        const h = try b.body(.{ .position = .{ 0, y, 0 }, .shape = box, .linear_damping = 0.5, .angular_damping = 0.5 }, .activate);
        const p: [3]Real = .{ 0, y + 0.275, 0 };
        links[i] = if (i % 2 == 0)
            try b.constraint(swingTwistDesc(previous, h, p, .{ 0, -1, 0 }, .{ 1, 0, 0 }, .{ 0.4, 0.4, -0.2, 0.2 }, 0, 0.2))
        else
            try b.constraint(Desc.cone(previous, h, p, .{ 0, -1, 0 }, 0.3));
        chain[i] = h;
        previous = h;
    }
    try b.constraintOp(.swing_twist_set_swing_motor_state, links[0], .{ .i = &.{MotorStateValue.position} });
    try b.constraintOp(.swing_twist_set_twist_motor_state, links[2], .{ .i = &.{MotorStateValue.position} });
    const asleep1 = try b.body(.{ .position = .{ 3, 5, 0 }, .shape = box, .gravity_factor = 0 }, .dont_activate);
    const asleep2 = try b.body(.{ .position = .{ 3, 4.45, 0 }, .shape = box, .gravity_factor = 0 }, .dont_activate);
    const awake = try b.body(.{ .position = .{ 6, 5, 0 }, .shape = box, .gravity_factor = 0, .angular_velocity = .{ 0, 1, 0 }, .allow_sleeping = 0 }, .activate);
    const asleep3 = try b.body(.{ .position = .{ 6, 4.45, 0 }, .shape = box, .gravity_factor = 0 }, .dont_activate);
    b.at(5);
    const sleeping_cone = try b.constraint(Desc.cone(asleep1, asleep2, .{ 3, 4.725, 0 }, .{ 0, -1, 0 }, 0.2));
    try b.expectConstraintState(sleeping_cone, ConstraintState.inactive);
    _ = try b.constraint(swingTwistDesc(awake, asleep3, .{ 6, 4.725, 0 }, .{ 0, -1, 0 }, .{ 1, 0, 0 }, .{ 0.3, 0.3, -0.5, 0.5 }, 1, 0)); // Wakes asleep3 in the next update
    b.at(6);
    try b.expectAwake(asleep3);
    try b.expectSleeping(asleep1);
    b.at(60);
    try b.expectSleeping(chain[0]);
    try b.expectSleeping(chain[3]);
    try b.constraintOp(.swing_twist_set_target_orientation_cs, links[0], .{ .f = &axisAngle(.{ 0, 0, 1 }, 0.3) });
    b.at(61);
    try b.expectSleeping(chain[0]); // Setting a target does not wake the bodies
    try b.constraintOp(.activate_constraint, sleeping_cone, .{});
    try b.expectAwake(asleep1);
    try b.expectAwake(asleep2);
    try b.op(.add_force, chain[3], .{ .f = &.{ 50, 0, 0 } });
    b.at(62);
    try b.expectAwake(chain[2]); // The force activated the lowest box, BuildIslands of its constraint the box above it
    b.at(65);
    try b.expectAwake(chain[0]);
}

/// Cone and swing twist chains with several collision steps per update and changing update parameters (the warm start
/// impulses are scaled by the step ratio), with motors
fn buildConeSwingTwistCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const box = try b.box(.{ 0.25, 0.12, 0.12 }, 0.05);
            b.num_updates = 50;
            b.collision_steps = steps;
            b.delta_time = @as(f32, @floatFromInt(steps)) / 60.0;
            var links: [6]u32 = undefined;
            var bodies: [6]u32 = undefined;
            try addConeSwingTwistChain(b, box, .{ 0, 6, 0 }, Layers.unused1, &links, &bodies);
            try b.constraintOp(.swing_twist_set_twist_motor_state, links[0], .{ .i = &.{MotorStateValue.velocity} });
            try b.constraintOp(.swing_twist_set_target_angular_velocity_cs, links[0], .{ .f = &.{ 2, 0, 0 } });
            try b.constraintOp(.swing_twist_set_swing_motor_state, links[4], .{ .i = &.{MotorStateValue.position_and_velocity} });
            try b.constraintOp(.swing_twist_set_target_orientation_cs, links[4], .{ .f = &axisAngle(.{ 0, 1, 0 }, 0.3) });
            b.at(20);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 90.0}, .count = 1 });
            b.at(30);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 20.0}, .count = steps + 1 });
        }
    }.build;
}

/// Cone and swing twist chains far from the origin (world space positions are Real: in double precision the anchors keep
/// their precision) that fall onto a floor: contacts and constraints in the same islands
fn buildConeSwingTwistFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.25, 0.12, 0.12 }, 0.05);
    b.num_updates = 70;
    const o: [3]Real = .{ -7000, 150, 9000 };
    const floor = try b.box(.{ 10, 1, 10 }, 0);
    _ = try b.body(.{ .position = .{ o[0], o[1] - 1, o[2] }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    var links: [6]u32 = undefined;
    var bodies: [6]u32 = undefined;
    try addConeSwingTwistChain(b, box, .{ o[0] - 1.5, o[1] + 2, o[2] }, Layers.moving, &links, &bodies);
    try b.constraintOp(.swing_twist_set_twist_motor_state, links[2], .{ .i = &.{MotorStateValue.velocity} });
    try b.constraintOp(.swing_twist_set_target_angular_velocity_cs, links[2], .{ .f = &.{ 3, 0, 0 } });
    // A loose pair (cone + swing twist between dynamic bodies) falls onto the floor
    const a = try b.body(.{ .position = .{ o[0] + 3, o[1] + 1.5, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
    const c = try b.body(.{ .position = .{ o[0] + 3.5, o[1] + 1.6, o[2] + 0.1 }, .shape = box, .angular_velocity = .{ 2, 1, 0 }, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(swingTwistDesc(a, c, .{ o[0] + 3.25, o[1] + 1.55, o[2] + 0.05 }, .{ 1, 0, 0 }, .{ 0, 0, 1 }, .{ 0.2, 0.6, -0.4, 0.4 }, 1, 0.1));
    const d = try b.body(.{ .position = .{ o[0] + 4, o[1] + 1.7, o[2] + 0.2 }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.cone(c, d, .{ o[0] + 3.75, o[1] + 1.65, o[2] + 0.15 }, .{ 1, 0, 0 }, 0.1));
    b.at(40);
    try b.op(.add_impulse, bodies[5], .{ .f = &.{ 0, 1, 1 } });
}

/// Random bodies (dynamic, kinematic, static; every basic shape) with random cone, swing twist, point and fixed
/// constraints between them and to the world (random frames, limits, swing types, friction, motors with springs in
/// every mode, both spaces), on a floor in a container, and random operations between the updates (every setter of the
/// two classes, motor states, targets, enable, remove / add, ResetWarmStart, impulses)
fn buildConeSwingTwistRandom(comptime seed: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.rng = 0xc0e5_7700 +% seed *% 0x9e3779b9;
            const shapes = try BasicShapes.add(b);
            b.num_updates = 80;
            b.max_contact_constraints = 2048;
            try addContainer(b, 6, 2);
            const num_bodies = 20;
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
            const num_constraints = 18;
            var handles: [num_constraints]u32 = undefined;
            var is_swing_twist: [num_constraints]bool = @splat(false);
            var is_cone: [num_constraints]bool = @splat(false);
            var in_system: [num_constraints]bool = @splat(true);
            for (&handles, 0..) |*c, k| {
                const index1 = b.index(num_bodies + 1); // num_bodies: the world
                var index2 = b.index(num_bodies);
                if (index2 == index1) index2 = (index2 + 1) % num_bodies;
                const body1 = if (index1 == num_bodies) invalid_handle else bodies[index1];
                const body2 = bodies[index2];
                const p1 = b.vec3(-4, 4);
                const p2 = if (b.oneIn(2)) p1 else b.vec3(-4, 4);
                const space = if (b.oneIn(4)) ConstraintSpace.local_to_body_com else ConstraintSpace.world_space;
                const q1 = b.rotation();
                const q2 = if (b.oneIn(2)) q1 else b.rotation();
                var desc: SimConstraint = undefined;
                switch (b.index(5)) {
                    0, 1 => {
                        desc = Desc.defaults(.swing_twist);
                        desc.i[0] = space;
                        desc.i[1] = b.index(2);
                        Desc.setR3(desc.r[0..], r3(p1));
                        Desc.setR3(desc.r[3..], r3(p2));
                        Desc.set3(desc.f[0..], rotateAxis(q1, 0));
                        Desc.set3(desc.f[3..], rotateAxis(q1, 1));
                        Desc.set3(desc.f[6..], rotateAxis(q2, 0));
                        Desc.set3(desc.f[9..], rotateAxis(q2, 1));
                        desc.f[12] = randomAngle(b, 0, std.math.pi);
                        desc.f[13] = randomAngle(b, 0, std.math.pi);
                        desc.f[14] = randomAngle(b, -std.math.pi, 0);
                        desc.f[15] = randomAngle(b, 0, std.math.pi);
                        desc.f[16] = if (b.oneIn(2)) 0 else b.float(0, 3);
                        randomMotor(b, &desc, false);
                        randomMotor(b, &desc, true);
                        is_swing_twist[k] = true;
                    },
                    2, 3 => {
                        desc = Desc.defaults(.cone);
                        desc.i[0] = space;
                        Desc.setR3(desc.r[0..], r3(p1));
                        Desc.setR3(desc.r[3..], r3(p2));
                        Desc.set3(desc.f[0..], rotateAxis(q1, 0));
                        Desc.set3(desc.f[3..], rotateAxis(q2, 0));
                        desc.f[6] = randomAngle(b, 0, std.math.pi);
                        is_cone[k] = true;
                    },
                    else => desc = if (b.oneIn(2)) Desc.fixedAuto(body1, body2) else Desc.point(body1, body2, ConstraintSpace.world_space, r3(p1), r3(p2)),
                }
                desc.body1 = body1;
                desc.body2 = body2;
                desc.constraint_priority = b.index(3);
                if (b.oneIn(6)) desc.num_velocity_steps_override = 1 + b.index(15);
                if (b.oneIn(6)) desc.num_position_steps_override = 1 + b.index(5);
                if (b.oneIn(10)) desc.enabled = 0;
                desc.user_data = b.next(); // Odd: not saved by the state filter of the end of the scene
                c.* = try b.constraint(desc);
            }
            var update: u32 = 3;
            while (update < b.num_updates) : (update += 1 + b.index(6)) {
                b.at(update);
                for (0..1 + b.index(3)) |_| {
                    const k = b.index(num_constraints);
                    const c = handles[k];
                    if (is_swing_twist[k] and !b.oneIn(4)) {
                        switch (b.index(13)) {
                            0 => try b.constraintOp(.swing_twist_set_normal_half_cone_angle, c, .{ .f = &.{randomAngle(b, 0, std.math.pi)} }),
                            1 => try b.constraintOp(.swing_twist_set_plane_half_cone_angle, c, .{ .f = &.{randomAngle(b, 0, std.math.pi)} }),
                            2 => try b.constraintOp(.swing_twist_set_twist_min_angle, c, .{ .f = &.{randomAngle(b, -std.math.pi, 0)} }),
                            3 => try b.constraintOp(.swing_twist_set_twist_max_angle, c, .{ .f = &.{randomAngle(b, 0, std.math.pi)} }),
                            4 => try b.constraintOp(.swing_twist_set_max_friction_torque, c, .{ .f = &.{b.float(0, 3)} }),
                            5, 6 => try b.constraintOp(.swing_twist_set_swing_motor_state, c, .{ .i = &.{b.index(4)} }),
                            7 => try b.constraintOp(.swing_twist_set_twist_motor_state, c, .{ .i = &.{b.index(4)} }),
                            8 => try b.constraintOp(.swing_twist_set_target_angular_velocity_cs, c, .{ .f = &b.vec3(-3, 3) }),
                            9 => try b.constraintOp(.swing_twist_set_target_angular_velocity_bs, c, .{ .f = &b.vec3(-3, 3) }),
                            10 => try b.constraintOp(.swing_twist_set_target_orientation_cs, c, .{ .f = &b.rotation() }),
                            11 => try b.constraintOp(.swing_twist_set_target_orientation_bs, c, .{ .f = &b.rotation() }),
                            else => {
                                var m: [6]f32 = undefined;
                                const mode = randomMotorValues(b, &m);
                                try b.constraintOp(if (b.oneIn(2)) .swing_twist_set_swing_motor_settings else .swing_twist_set_twist_motor_settings, c, .{ .i = &.{mode}, .f = &m });
                            },
                        }
                    } else if (is_cone[k] and !b.oneIn(3)) {
                        try b.constraintOp(.cone_set_half_cone_angle, c, .{ .f = &.{randomAngle(b, 0, std.math.pi)} });
                    } else switch (b.index(6)) {
                        0 => try b.constraintOp(.set_constraint_enabled, c, .{ .i = &.{b.index(2)} }),
                        1 => {
                            try b.constraintOp(if (in_system[k]) .remove_constraint else .add_constraint, c, .{});
                            in_system[k] = !in_system[k];
                        },
                        2 => try b.constraintOp(.activate_constraint, c, .{}),
                        3 => try b.constraintOp(.reset_constraint_warm_start, c, .{}),
                        4 => try b.constraintOp(.constraint_settings_hash, c, .{}),
                        else => try b.op(.add_impulse, bodies[b.index(num_bodies)], .{ .f = &b.vec3(-3, 3) }),
                    }
                }
            }
        }
    }.build;
}

/// Axis `axis` (0: X, 1: Y) of the rotation `q` (x, y, z, w) (computed in f64 by the scene builder)
fn rotateAxis(q: [4]f32, axis: u32) [3]f32 {
    const x: f64 = q[0];
    const y: f64 = q[1];
    const z: f64 = q[2];
    const w: f64 = q[3];
    const v: [3]f64 = if (axis == 0)
        .{ 1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y) }
    else
        .{ 2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x) };
    return normalize3(.{ @floatCast(v[0]), @floatCast(v[1]), @floatCast(v[2]) });
}

/// A random angle in [min, max], the bounds themselves now and then
fn randomAngle(b: *SceneBuilder, min: f32, max: f32) f32 {
    return switch (b.index(6)) {
        0 => min,
        1 => max,
        else => b.float(min, max),
    };
}

/// A valid random motor (MotorSettings::IsValid): returns the spring mode, the frequency / stiffness, damping and the
/// limits in `m`
fn randomMotorValues(b: *SceneBuilder, m: *[6]f32) u32 {
    const mode = b.index(3);
    m[0] = if (b.oneIn(4)) 0 else if (mode == 0) b.float(0.5, 10) else b.float(1, 100);
    m[1] = if (b.oneIn(3)) 0 else b.float(0, 2);
    m[2] = -flt_max;
    m[3] = flt_max;
    const torque = if (b.oneIn(3)) flt_max else b.float(0.1, 50);
    m[4] = -torque;
    m[5] = torque;
    return mode;
}

/// A random swing (`twist` false) or twist motor of a swing twist description
fn randomMotor(b: *SceneBuilder, c: *SimConstraint, twist: bool) void {
    var m: [6]f32 = undefined;
    const mode = randomMotorValues(b, &m);
    setSwingTwistMotor(c, twist, mode, m);
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

/// EPathRotationConstraintType values for `Desc.path`
const PathRotation = struct {
    const free: u32 = 0;
    const around_tangent: u32 = 1;
    const around_normal: u32 = 2;
    const around_binormal: u32 = 3;
    const to_path: u32 = 4;
    const fully_constrained: u32 = 5;
};

/// The points of a straight Hermite path from `start` to `end` (`num` points, the tangents are the segments, normal
/// `normal`), computed in f64 by the scene builder
fn linePathPoints(comptime num: usize, start: [3]f64, end: [3]f64, normal: [3]f32) [num][9]f32 {
    var points: [num][9]f32 = undefined;
    const segments: f64 = @floatFromInt(num - 1);
    for (&points, 0..) |*p, k| {
        const t = @as(f64, @floatFromInt(k)) / segments;
        for (0..3) |c| {
            p[c] = @floatCast(start[c] + (end[c] - start[c]) * t);
            p[3 + c] = @floatCast((end[c] - start[c]) / segments);
            p[6 + c] = normal[c];
        }
    }
    return points;
}

/// A path description with a motor: `Desc.path` plus the max friction force and the motor (spring mode, the six motor
/// values of `Desc.setMotor`)
fn pathWithMotor(body1: u32, body2: u32, points: []const [9]f32, is_looping: bool, path_position: [3]f32, path_fraction: f32, rotation_type: u32, max_friction_force: f32, motor_mode: u32, motor: [6]f32) SimConstraint {
    var c = Desc.path(body1, body2, points, is_looping, path_position, identity_rotation, path_fraction, rotation_type);
    c.f[8] = max_friction_force;
    Desc.setMotor(&c, 4, 9, motor_mode, motor);
    return c;
}

/// The world space position of a point of a path whose start is at `origin` (identity rotation)
fn pathPoint(origin: [3]Real, points: []const [9]f32, k: usize) [3]Real {
    return .{ origin[0] + points[k][0], origin[1] + points[k][1], origin[2] + points[k][2] };
}

/// Bodies on open slopes from the world under gravity: one slides down and stops at the limit at the end of the path,
/// one is pushed up beyond the start (the limit at fraction 0) and slides back, one is held by friction until the
/// friction is removed, one slides slowly against weak friction, one rides on a slope attached to a heavy cart (body
/// 1 dynamic), each with another rotation constraint type; SetPath with the same path at another fraction re-attaches
/// a body where it is
fn buildPathLimitsAndFriction(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    const cart_shape = try b.box(.{ 1.5, 0.25, 0.6 }, 0.05);
    b.num_updates = 100;
    const slope = linePathPoints(4, .{ 0, 0, 0 }, .{ 6, -3, 0 }, .{ 0, 0, 1 });
    const mass: f32 = 0.4 * 0.4 * 0.4 * 1000.0; // The density of the box shape is 1000
    const along_slope: f32 = mass * 9.81 * 0.4472136; // m g sin(atan(3 / 6))
    var handles: [5]u32 = undefined;
    var constraints: [4]u32 = undefined;
    var descs: [4]u32 = undefined;
    const Setup = struct { z: f32, fraction: f32, rotation: u32, friction: f32, velocity: [3]f32 };
    const setups = [_]Setup{
        .{ .z = 0, .fraction = 0.5, .rotation = PathRotation.to_path, .friction = 0, .velocity = .{ 0, 0, 0 } },
        .{ .z = 2, .fraction = 1.5, .rotation = PathRotation.free, .friction = 0, .velocity = .{ -6, 3, 0 } },
        .{ .z = 4, .fraction = 1.0, .rotation = PathRotation.fully_constrained, .friction = 2.0 * along_slope, .velocity = .{ 0, 0, 0 } },
        .{ .z = 6, .fraction = 0.25, .rotation = PathRotation.around_tangent, .friction = 0.5 * along_slope, .velocity = .{ 0, 0, 0 } },
    };
    for (setups, 0..) |s, k| {
        // On the straight path starting at (0, 8, z): a fraction f is at (2 f, 8 - f, z)
        const p: [3]Real = .{ 2.0 * real(s.fraction), 8.0 - real(s.fraction), s.z };
        handles[k] = try b.body(.{ .position = p, .shape = box, .linear_velocity = s.velocity, .angular_damping = 0.5 }, .activate);
        descs[k] = try b.addConstraint(pathWithMotor(invalid_handle, handles[k], &slope, false, .{ 0, 8, s.z }, s.fraction, s.rotation, s.friction, 0, Desc.default_motor));
        constraints[k] = try b.createConstraint(descs[k]);
    }
    // A heavy cart without gravity carries a slope (in its own space), the box rides on it around the binormal
    const cart = try b.body(.{ .position = .{ 3, 2, 10 }, .shape = cart_shape, .gravity_factor = 0, .linear_velocity = .{ 0.5, 0, 0 } }, .activate);
    handles[4] = try b.body(.{ .position = .{ 2, 3, 10 }, .shape = box }, .activate);
    const riding = try b.constraint(pathWithMotor(cart, handles[4], &slope, false, .{ -3, 2, 0 }, 1.0, PathRotation.around_binormal, 0, 0, Desc.default_motor));
    b.at(40);
    try b.constraintOp(.path_set_max_friction_force, constraints[2], .{ .f = &.{0} }); // The held box starts to slide
    b.at(50);
    try b.constraintOp(.path_set_path, constraints[3], .{ .i = &.{ 1, descs[3] }, .f = &.{2.5} }); // The slow box is attached at fraction 2.5 where it is
    try b.constraintOp(.notify_constraint_shape_changed, riding, .{ .body = cart, .f = &.{ 0.1, 0, 0 } });
    b.at(70);
    try b.op(.add_impulse, handles[4], .{ .f = &.{ -400, 0, 0 } }); // Up the cart's slope, against the start limit
}

/// The position motor of PathConstraint in every EMotorState without gravity: a velocity motor with force limits on a
/// looping circle; position motors with springs in frequency, stiffness and mass normalized stiffness mode (one drives
/// across the end of a looping path, CalculateConstraintValue's fmod); a position motor without stiffness and a
/// position + velocity motor without stiffness or damping (the motor part is deactivated: the bodies coast); a position
/// + velocity motor with damping only; friction without motor. The states, targets, settings and friction change
/// between the updates, also to an int that is no EMotorState (no case of Jolt's switches matches). Body 1 is the world,
/// a static body or a kinematic body that turns.
fn buildPathMotors(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.15, 0.15, 0.15 }, 0.05);
    const post = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.num_updates = 90;
    b.gravity = .{ 0, 0, 0 };
    const circle = circlePathPoints(8, 2, std.math.pi * 2.0);
    const arc = circlePathPoints(6, 2, 0.75 * std.math.pi * 2.0);
    const Setup = struct { looping: bool, fraction: f32, rotation: u32, friction: f32, mode: u32, motor: [6]f32, state: u32, target_velocity: f32, target_fraction: f32, velocity: [3]f32 };
    const setups = [_]Setup{
        // Velocity motor with force limits, rotation constrained to the path
        .{ .looping = true, .fraction = 0, .rotation = PathRotation.to_path, .friction = 0, .mode = 0, .motor = .{ 2, 1, -150, 150, -flt_max, flt_max }, .state = 1, .target_velocity = 3, .target_fraction = 0, .velocity = .{ 0, 0, 0 } },
        // Position motor, frequency and damping
        .{ .looping = false, .fraction = 0, .rotation = PathRotation.free, .friction = 0, .mode = 0, .motor = .{ 2, 1, -flt_max, flt_max, -flt_max, flt_max }, .state = 2, .target_velocity = 0, .target_fraction = 4, .velocity = .{ 0, 0, 0 } },
        // Position motor, stiffness and damping
        .{ .looping = false, .fraction = 4, .rotation = PathRotation.around_normal, .friction = 0, .mode = 1, .motor = .{ 500, 40, -300, 300, -flt_max, flt_max }, .state = 2, .target_velocity = 0, .target_fraction = 1, .velocity = .{ 0, 0, 0 } },
        // Position motor, mass normalized stiffness, across the end of a looping path (7 -> 0.5 is 1.5 forward)
        .{ .looping = true, .fraction = 7, .rotation = PathRotation.around_tangent, .friction = 0, .mode = 2, .motor = .{ 20, 4, -flt_max, flt_max, -flt_max, flt_max }, .state = 2, .target_velocity = 0, .target_fraction = 0.5, .velocity = .{ 0, 0, 0 } },
        // Position motor without stiffness: no motor, the body coasts
        .{ .looping = true, .fraction = 2, .rotation = PathRotation.free, .friction = 0, .mode = 0, .motor = .{ 0, 1, -flt_max, flt_max, -flt_max, flt_max }, .state = 2, .target_velocity = 0, .target_fraction = 5, .velocity = .{ 0, 0, 1 } },
        // Position and velocity motor with damping only
        .{ .looping = false, .fraction = 1, .rotation = PathRotation.fully_constrained, .friction = 0, .mode = 1, .motor = .{ 0, 30, -flt_max, flt_max, -flt_max, flt_max }, .state = 3, .target_velocity = 1, .target_fraction = 3, .velocity = .{ 0, 0, 0 } },
        // Position and velocity motor without stiffness or damping: no motor
        .{ .looping = true, .fraction = 4, .rotation = PathRotation.around_binormal, .friction = 0, .mode = 0, .motor = .{ 0, 0, -flt_max, flt_max, -flt_max, flt_max }, .state = 3, .target_velocity = 2, .target_fraction = 6, .velocity = .{ 1, 0, 0 } },
        // No motor, friction slows the body down
        .{ .looping = true, .fraction = 6, .rotation = PathRotation.to_path, .friction = 20, .mode = 0, .motor = Desc.default_motor, .state = 0, .target_velocity = 0, .target_fraction = 0, .velocity = .{ 0, 0, -2 } },
    };
    var constraints: [setups.len + 2]u32 = undefined;
    var descs: [setups.len + 2]u32 = undefined;
    for (setups, 0..) |s, k| {
        const origin: [3]Real = .{ 6.0 * real(k), 2, 0 };
        const points: []const [9]f32 = if (s.looping) &circle else &arc;
        const h = try b.body(.{ .position = pathPoint(origin, points, @intFromFloat(s.fraction)), .shape = box, .linear_velocity = s.velocity }, .activate);
        descs[k] = try b.addConstraint(pathWithMotor(invalid_handle, h, points, s.looping, .{ @floatCast(origin[0]), @floatCast(origin[1]), @floatCast(origin[2]) }, s.fraction, s.rotation, s.friction, s.mode, s.motor));
        constraints[k] = try b.createConstraint(descs[k]);
        try b.constraintOp(.path_set_position_motor_state, constraints[k], .{ .i = &.{s.state} });
        try b.constraintOp(.path_set_target_velocity, constraints[k], .{ .f = &.{s.target_velocity} });
        try b.constraintOp(.path_set_target_path_fraction, constraints[k], .{ .f = &.{s.target_fraction} });
    }
    // On a static post: a velocity motor; on a kinematic post that turns: a position motor
    const static_post = try b.body(.{ .position = .{ 0, 2, 8 }, .shape = post, .motion_type = 0 }, .dont_activate);
    const rider1 = try b.body(.{ .position = .{ 2, 2, 8 }, .shape = box }, .activate);
    descs[setups.len] = try b.addConstraint(pathWithMotor(static_post, rider1, &circle, true, .{ 0, 0, 0 }, 0, PathRotation.to_path, 0, 0, Desc.default_motor));
    constraints[setups.len] = try b.createConstraint(descs[setups.len]);
    try b.constraintOp(.path_set_position_motor_state, constraints[setups.len], .{ .i = &.{1} });
    try b.constraintOp(.path_set_target_velocity, constraints[setups.len], .{ .f = &.{-1.5} });
    const kinematic_post = try b.body(.{ .position = .{ 8, 2, 8 }, .shape = post, .motion_type = 1, .angular_velocity = .{ 0, 0.8, 0 } }, .activate);
    const rider2 = try b.body(.{ .position = .{ 10, 2, 8 }, .shape = box }, .activate);
    descs[setups.len + 1] = try b.addConstraint(pathWithMotor(kinematic_post, rider2, &circle, true, .{ 0, 0, 0 }, 0, PathRotation.fully_constrained, 0, 0, .{ 3, 0.5, -flt_max, flt_max, -flt_max, flt_max }));
    constraints[setups.len + 1] = try b.createConstraint(descs[setups.len + 1]);
    try b.constraintOp(.path_set_position_motor_state, constraints[setups.len + 1], .{ .i = &.{2} });
    try b.constraintOp(.path_set_target_path_fraction, constraints[setups.len + 1], .{ .f = &.{4} });

    b.at(25);
    try b.constraintOp(.path_set_position_motor_state, constraints[0], .{ .i = &.{2} }); // Velocity -> position
    try b.constraintOp(.path_set_target_path_fraction, constraints[0], .{ .f = &.{-3.25} }); // Any fraction on a looping path
    try b.constraintOp(.path_set_position_motor_state, constraints[1], .{ .i = &.{3} }); // Position -> position and velocity
    try b.constraintOp(.path_set_target_velocity, constraints[1], .{ .f = &.{-0.5} });
    try b.constraintOp(.path_set_position_motor_state, constraints[4], .{ .i = &.{1} }); // Coasting -> velocity
    try b.constraintOp(.path_set_target_velocity, constraints[4], .{ .f = &.{2} });
    try b.constraintOp(.path_set_max_friction_force, constraints[7], .{ .f = &.{0} });
    b.at(40);
    try b.constraintOp(.path_set_position_motor_settings, constraints[0], .{ .i = &.{1}, .f = &.{ 300, 30, -200, 200, -1, 1 } });
    try b.constraintOp(.path_set_position_motor_state, constraints[2], .{ .i = &.{0} }); // Off
    try b.constraintOp(.path_set_max_friction_force, constraints[2], .{ .f = &.{5} });
    try b.constraintOp(.path_set_position_motor_state, constraints[5], .{ .i = &.{5} }); // No EMotorState: the part keeps its last setup
    try b.constraintOp(.path_set_position_motor_settings, constraints[3], .{ .i = &.{2}, .f = &.{ 60, 0, -flt_max, flt_max, -flt_max, flt_max } });
    try b.constraintOp(.path_set_target_path_fraction, constraints[3], .{ .f = &.{6.5} }); // Backwards across the end
    b.at(55);
    try b.constraintOp(.path_set_position_motor_state, constraints[5], .{ .i = &.{1} });
    try b.constraintOp(.path_set_target_velocity, constraints[5], .{ .f = &.{-1} });
    try b.constraintOp(.path_set_position_motor_state, constraints[6], .{ .i = &.{2} });
    try b.constraintOp(.path_set_position_motor_settings, constraints[6], .{ .i = &.{0}, .f = &.{ 1.5, 0.3, -flt_max, flt_max, -flt_max, flt_max } });
    try b.constraintOp(.reset_constraint_warm_start, constraints[0], .{});
    b.at(70);
    try b.constraintOp(.path_set_position_motor_state, constraints[setups.len + 1], .{ .i = &.{3} });
    try b.constraintOp(.path_set_target_velocity, constraints[setups.len + 1], .{ .f = &.{0.5} });
    try b.constraintOp(.path_set_path, constraints[setups.len], .{ .i = &.{ 1, descs[1] }, .f = &.{2} }); // The arc
}

/// Every EPathRotationConstraintType (and an int that is no EPathRotationConstraintType: no rotation constraint) on
/// curved paths attached to plates that turn freely (body 1 dynamic without gravity), the bodies on the paths spin and
/// are driven by velocity motors; one path is attached to a sleeping plate (BuildIslands wakes it)
fn buildPathRotationTypes(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.15, 0.25, 0.1 }, 0.05);
    const plate = try b.box(.{ 1.2, 0.1, 1.2 }, 0.05);
    b.num_updates = 70;
    b.gravity = .{ 0, -2, 0 };
    const arc = circlePathPoints(5, 1, std.math.pi);
    const rotation_types = [_]u32{ PathRotation.free, PathRotation.around_tangent, PathRotation.around_normal, PathRotation.around_binormal, PathRotation.to_path, PathRotation.fully_constrained, 7 };
    var constraints: [rotation_types.len]u32 = undefined;
    var riders: [rotation_types.len]u32 = undefined;
    for (rotation_types, 0..) |rotation, k| {
        const x: Real = 4.0 * real(k);
        // The plate of the binormal type sleeps (a sleeping body has no velocity) until BuildIslands wakes it
        const asleep = k == 3;
        const base = try b.body(.{ .position = .{ x, 3, 0 }, .shape = plate, .gravity_factor = 0, .angular_velocity = if (asleep) .{ 0, 0, 0 } else .{ 0.1, 0.3, -0.2 } }, if (asleep) .dont_activate else .activate);
        const h = try b.body(.{ .position = .{ x + 1, 3.5, 0 }, .rotation = axisAngle(.{ 0, 0, 1 }, 0.2), .shape = box, .angular_velocity = .{ 1, -2, 0.5 } }, .activate);
        riders[k] = h;
        constraints[k] = try b.constraint(pathWithMotor(base, h, &arc, false, .{ 0, 0.5, 0 }, 0.0, rotation, 0, 0, Desc.default_motor));
        try b.constraintOp(.path_set_position_motor_state, constraints[k], .{ .i = &.{1} });
        try b.constraintOp(.path_set_target_velocity, constraints[k], .{ .f = &.{0.6} });
    }
    b.at(30);
    for (constraints) |c| try b.constraintOp(.path_set_target_velocity, c, .{ .f = &.{-0.4} });
    try b.constraintOp(.notify_constraint_shape_changed, constraints[4], .{ .body = riders[4], .f = &.{ 0, 0.02, 0 } }); // Body 2
}

/// A train on a looping track (a path from the world): six cars, each on the track (rotation constrained to the path),
/// linked by point constraints, the first one driven by a velocity motor with a force limit, the last one with cargo
/// welded on it (fixed constraint); boxes fall onto the cars and the floor (contacts in the island of the train)
fn buildPathTrainWithContacts(b: *SceneBuilder) Allocator.Error!void {
    const car = try b.box(.{ 0.3, 0.2, 0.2 }, 0.05);
    const crate = try b.box(.{ 0.15, 0.15, 0.15 }, 0.05);
    b.num_updates = 90;
    _ = try addFloor(b, 15, 0);
    const radius = 4.0;
    const track = circlePathPoints(16, radius, std.math.pi * 2.0);
    const origin: [3]f32 = .{ 0, 1, 0 };
    const num_cars = 6;
    var cars: [num_cars]u32 = undefined;
    for (0..num_cars) |k| {
        const fraction: f32 = @floatFromInt(2 * k);
        const p = pathPoint(.{ 0, 1, 0 }, &track, 2 * k);
        const tangent = track[2 * k][3..6];
        const heading = std.math.atan2(-@as(f64, tangent[2]), @as(f64, tangent[0]));
        cars[k] = try b.body(.{ .position = p, .rotation = axisAngle(.{ 0, 1, 0 }, heading), .shape = car, .object_layer = Layers.moving }, .activate);
        const c = try b.constraint(pathWithMotor(invalid_handle, cars[k], &track, true, origin, fraction, PathRotation.to_path, 0, 0, .{ 2, 1, -400, 400, -flt_max, flt_max }));
        if (k == 0) {
            try b.constraintOp(.path_set_position_motor_state, c, .{ .i = &.{1} });
            try b.constraintOp(.path_set_target_velocity, c, .{ .f = &.{-1.5} });
        }
        if (k > 0) {
            // The coupling halfway between the cars (on the chord)
            const q = pathPoint(.{ 0, 1, 0 }, &track, 2 * k - 1);
            _ = try addPointConstraint(b, cars[k - 1], cars[k], .{ @floatCast(q[0]), @floatCast(q[1]), @floatCast(q[2]) }, .{ @floatCast(q[0]), @floatCast(q[1]), @floatCast(q[2]) });
        }
    }
    _ = try b.body(.{ .position = .{ 0, 1.5, 0 }, .shape = crate, .object_layer = Layers.moving }, .activate); // Falls onto the floor
    const last = pathPoint(.{ 0, 1, 0 }, &track, 2 * (num_cars - 1));
    const welded = try b.body(.{ .position = .{ last[0], last[1] + 0.36, last[2] }, .shape = crate, .object_layer = Layers.moving }, .activate);
    _ = try addFixedConstraint(b, cars[num_cars - 1], welded);
    b.at(10);
    for ([_]usize{ 0, 4, 8, 13 }) |k| {
        // On cars 0, 2 and 4, and next to the track
        const p = pathPoint(.{ 0, 2.5, 0 }, &track, k);
        _ = try b.body(.{ .position = p, .shape = crate, .object_layer = Layers.moving }, .activate);
    }
    b.at(45);
    try b.constraintOp(.path_set_target_velocity, 0, .{ .f = &.{2} });
}

/// A net of boxes on the floor: neighbours along X are linked by point constraints, along Z by path constraints on
/// straight paths in the space of the previous box (with friction, fully constrained rotation); the corners hang on the
/// world. More than 128 constraints in one island: the LargeIslandSplitter splits it, and it falls asleep.
fn buildPathNetSplitter(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.4, 0.2, 0.4 }, 0.05);
    b.num_updates = 100;
    b.max_contact_constraints = 2048;
    b.settings.time_before_sleep = 0.25;
    _ = try addFloor(b, 20, 0);
    const nx = 8;
    const nz = 12;
    const link = linePathPoints(3, .{ 0, 0, 0.5 }, .{ 0, 0, 1.5 }, .{ 0, 1, 0 });
    var handles: [nx * nz]u32 = undefined;
    for (0..nz) |z| {
        for (0..nx) |x| {
            const px: f32 = @as(f32, @floatFromInt(x)) - 4.0;
            const pz: f32 = @as(f32, @floatFromInt(z)) - 6.0;
            handles[z * nx + x] = try b.body(.{ .position = .{ px, 0.25, pz }, .shape = box, .object_layer = Layers.moving }, .activate);
            if (x > 0) _ = try addPointConstraint(b, handles[z * nx + x - 1], handles[z * nx + x], .{ px - 0.5, 0.25, pz }, .{ px - 0.5, 0.25, pz });
            if (z > 0) _ = try b.constraint(pathWithMotor(handles[(z - 1) * nx + x], handles[z * nx + x], &link, false, .{ 0, 0, 0 }, 1.0, if ((x + z) % 2 == 0) PathRotation.fully_constrained else PathRotation.around_normal, 50, 0, Desc.default_motor));
        }
    }
    // Pin the corners to the world
    for ([_]usize{ 0, nx - 1, (nz - 1) * nx, nz * nx - 1 }) |i| {
        const p: [3]f32 = .{ @as(f32, @floatFromInt(i % nx)) - 4.0, 0.25, @as(f32, @floatFromInt(i / nx)) - 6.0 };
        _ = try addPointConstraint(b, invalid_handle, handles[i], p, p);
    }
    b.at(10);
    try b.op(.add_impulse, handles[nx * nz / 2], .{ .f = &.{ 0, 300, 400 } });
}

/// Sleeping and waking: a box held by friction on a slope falls asleep and an impulse wakes it; a path constraint created
/// between an awake body and a sleeping one wakes the sleeping body (BuildIslands), one between two sleeping bodies is
/// not active until ActivateConstraint; SetPath(nullptr) makes a constraint inactive, SetPath with a path again active
fn buildPathSleepAndWake(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    b.num_updates = 80;
    b.settings.time_before_sleep = 0.2;
    const slope = linePathPoints(3, .{ 0, 0, 0 }, .{ 4, -1, 0 }, .{ 0, 0, 1 });
    const held = try b.body(.{ .position = .{ 2, 4.5, 0 }, .shape = box }, .activate);
    const holding = try b.constraint(pathWithMotor(invalid_handle, held, &slope, false, .{ 0, 5, 0 }, 1.0, PathRotation.fully_constrained, 400, 0, Desc.default_motor));
    const awake = try b.body(.{ .position = .{ 0, 5, 4 }, .shape = box, .gravity_factor = 0, .angular_velocity = .{ 0, 1, 0 }, .allow_sleeping = 0 }, .activate);
    const sleeper = try b.body(.{ .position = .{ 1, 5, 4 }, .shape = box, .gravity_factor = 0 }, .dont_activate);
    const sleeper1 = try b.body(.{ .position = .{ 0, 5, 8 }, .shape = box, .gravity_factor = 0 }, .dont_activate);
    const sleeper2 = try b.body(.{ .position = .{ 1, 5, 8 }, .shape = box, .gravity_factor = 0 }, .dont_activate);
    const local = linePathPoints(3, .{ 0, 0, 0 }, .{ 2, 0, 0 }, .{ 0, 1, 0 });
    b.at(5);
    try b.expectSleeping(sleeper);
    const waking_desc = try b.addConstraint(pathWithMotor(awake, sleeper, &local, false, .{ 0, 0, 0 }, 1.0, PathRotation.around_normal, 0, 0, Desc.default_motor));
    const waking = try b.createConstraint(waking_desc);
    const sleeping_pair = try b.constraint(pathWithMotor(sleeper1, sleeper2, &local, false, .{ 0, 0, 0 }, 1.0, PathRotation.free, 0, 0, Desc.default_motor));
    try b.expectConstraintState(sleeping_pair, ConstraintState.inactive);
    b.at(6);
    try b.expectAwake(sleeper);
    try b.expectSleeping(sleeper1);
    b.at(15);
    try b.constraintOp(.activate_constraint, sleeping_pair, .{});
    try b.expectAwake(sleeper1);
    try b.expectAwake(sleeper2);
    try b.constraintOp(.path_set_position_motor_state, sleeping_pair, .{ .i = &.{1} });
    try b.constraintOp(.path_set_target_velocity, sleeping_pair, .{ .f = &.{0.3} });
    b.at(40);
    try b.expectSleeping(held);
    try b.op(.add_impulse, held, .{ .f = &.{ 0, 300, 0 } });
    try b.constraintOp(.path_set_path, waking, .{ .i = &.{0}, .f = &.{0} }); // No path: not active
    try b.expectConstraintState(waking, ConstraintState.inactive);
    b.at(41);
    try b.expectAwake(held);
    b.at(50);
    try b.constraintOp(.path_set_max_friction_force, holding, .{ .f = &.{10} }); // It slides down to the end
    try b.constraintOp(.path_set_path, waking, .{ .i = &.{ 1, waking_desc }, .f = &.{1.5} });
    try b.expectConstraintState(waking, ConstraintState.enabled | ConstraintState.active);
}

/// Path constraints across cross save / restore: the restored state brings back the warm start impulses, the friction,
/// the motor settings (written as raw bytes), the motor state, the targets and the path fraction, but not the path
/// (SetPath between the snapshot and the restore keeps the other path); changes of the physics settings (no warm start,
/// other iteration counts); a snapshot of the constraints only
fn buildPathSaveRestore(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
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
    const state_constraints: u32 = 8; // EStateRecorderState::Constraints

    const circle = circlePathPoints(8, 1.5, std.math.pi * 2.0);
    const arc = circlePathPoints(6, 1.5, 0.75 * std.math.pi * 2.0);
    var bodies: [4]u32 = undefined;
    var constraints: [4]u32 = undefined;
    for (0..4) |k| {
        const origin: [3]Real = .{ 5.0 * real(k), 4, 0 };
        bodies[k] = try b.body(.{ .position = pathPoint(origin, &circle, k), .shape = box, .angular_velocity = .{ 0, 0.5, 0 } }, .activate);
        constraints[k] = try b.constraint(pathWithMotor(invalid_handle, bodies[k], &circle, true, .{ @floatCast(origin[0]), 4, 0 }, @floatFromInt(k), @intCast(k + 2), 10, @intCast(k % 3), .{ 2, 1, -500, 500, -flt_max, flt_max }));
        try b.constraintOp(.path_set_position_motor_state, constraints[k], .{ .i = &.{@intCast(k)} });
        try b.constraintOp(.path_set_target_velocity, constraints[k], .{ .f = &.{1} });
        try b.constraintOp(.path_set_target_path_fraction, constraints[k], .{ .f = &.{@floatFromInt(k + 3)} });
    }
    const arc_desc = try b.addConstraint(Desc.path(invalid_handle, bodies[0], &arc, false, .{ 0, 4, 0 }, identity_rotation, 0, 0));
    // A chain hanging from the last body (point constraints)
    var previous = bodies[3];
    for (0..3) |k| {
        const p = pathPoint(.{ 15, 4, 0 }, &circle, 3);
        const h = try b.body(.{ .position = .{ p[0], p[1] - 0.5 * real(k + 1), p[2] }, .shape = box }, .activate);
        _ = try addPointConstraint(b, previous, h, .{ @floatCast(p[0]), @floatCast(p[1] - 0.5 * real(k) - 0.25), @floatCast(p[2]) }, .{ @floatCast(p[0]), @floatCast(p[1] - 0.5 * real(k) - 0.25), @floatCast(p[2]) });
        previous = h;
    }
    b.at(15);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(25);
    try b.sysOp(.set_physics_settings, .{ .index = settings1 });
    try b.constraintOp(.path_set_position_motor_state, constraints[1], .{ .i = &.{3} });
    try b.constraintOp(.path_set_position_motor_settings, constraints[1], .{ .i = &.{1}, .f = &.{ 100, 5, -20, 20, -1, 1 } });
    try b.constraintOp(.path_set_max_friction_force, constraints[0], .{ .f = &.{0} });
    try b.constraintOp(.path_set_target_velocity, constraints[2], .{ .f = &.{-2} });
    try b.constraintOp(.path_set_path, constraints[3], .{ .i = &.{ 1, arc_desc }, .f = &.{2} });
    b.at(35);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // The motor of constraints[1] and the friction come back, the arc stays
    b.at(45);
    try b.sysOp(.set_physics_settings, .{ .index = settings2 });
    try b.sysOp(.snapshot, .{ .index = 1, .i = &.{state_constraints} });
    try b.op(.add_impulse, bodies[2], .{ .f = &.{ 0, 0, 100 } });
    b.at(55);
    try b.constraintOp(.reset_constraint_warm_start, constraints[2], .{});
    try b.constraintOp(.path_set_target_path_fraction, constraints[2], .{ .f = &.{7.5} });
    try b.sysOp(.restore_cross, .{ .index = 1 }); // The constraints only
    b.at(70);
    try b.sysOp(.snapshot, .{ .index = 2 });
    try b.op(.add_impulse, bodies[1], .{ .f = &.{ 50, 0, 0 } });
    b.at(75);
    try b.sysOp(.restore_cross, .{ .index = 2 });
}

/// Path constraints with several collision steps per update and changing update parameters (the warm start impulses
/// are scaled by the step ratio): bodies on looping and open paths with motors and limits, a chain on point constraints
fn buildPathCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const box = try b.box(.{ 0.2, 0.3, 0.2 }, 0.05);
            b.num_updates = 50;
            b.collision_steps = steps;
            b.delta_time = @as(f32, @floatFromInt(steps)) / 60.0;
            const circle = circlePathPoints(8, 2, std.math.pi * 2.0);
            const slope = linePathPoints(4, .{ 0, 0, 0 }, .{ 3, -3, 0 }, .{ 0, 0, 1 });
            const a = try b.body(.{ .position = .{ 2, 5, 0 }, .shape = box }, .activate);
            const looping = try b.constraint(pathWithMotor(invalid_handle, a, &circle, true, .{ 0, 5, 0 }, 0, PathRotation.to_path, 0, 0, .{ 3, 1, -200, 200, -flt_max, flt_max }));
            try b.constraintOp(.path_set_position_motor_state, looping, .{ .i = &.{2} });
            try b.constraintOp(.path_set_target_path_fraction, looping, .{ .f = &.{5} });
            const c = try b.body(.{ .position = .{ 6, 5, 0 }, .shape = box }, .activate);
            _ = try b.constraint(pathWithMotor(invalid_handle, c, &slope, false, .{ 6, 5, 0 }, 0, PathRotation.around_tangent, 0, 0, Desc.default_motor));
            const d = try b.body(.{ .position = .{ 6, 4.4, 0 }, .shape = box }, .activate);
            _ = try addPointConstraint(b, c, d, .{ 6, 4.7, 0 }, .{ 6, 4.7, 0 });
            b.at(20);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 90.0}, .count = 1 });
            b.at(30);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 20.0}, .count = steps + 1 });
        }
    }.build;
}

/// Path constraints far from the origin: a path in world space whose start (`mPathPosition`, a float) is far away, a
/// path attached to a static and to a kinematic body far away (in the space of the body: the anchors keep their
/// precision in double precision), boxes on them fall onto a floor
fn buildPathFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const box = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    const post = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    b.num_updates = 60;
    const o: [3]Real = .{ 5000, 200, -12000 };
    const floor = try b.box(.{ 10, 1, 10 }, 0);
    _ = try b.body(.{ .position = .{ o[0], o[1] - 1, o[2] }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const slope = linePathPoints(4, .{ 0, 0, 0 }, .{ 3, -2, 0 }, .{ 0, 0, 1 });
    const circle = circlePathPoints(8, 1.5, std.math.pi * 2.0);
    const a = try b.body(.{ .position = .{ o[0] - 4, o[1] + 3, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(pathWithMotor(invalid_handle, a, &slope, false, .{ @floatCast(o[0] - 4), @floatCast(o[1] + 3), @floatCast(o[2]) }, 0, PathRotation.to_path, 0, 0, Desc.default_motor));
    const static_post = try b.body(.{ .position = .{ o[0] + 2, o[1] + 2, o[2] }, .shape = post, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const c = try b.body(.{ .position = .{ o[0] + 3.5, o[1] + 2, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
    const on_static = try b.constraint(pathWithMotor(static_post, c, &circle, true, .{ 0, 0, 0 }, 0, PathRotation.fully_constrained, 0, 0, Desc.default_motor));
    try b.constraintOp(.path_set_position_motor_state, on_static, .{ .i = &.{1} });
    try b.constraintOp(.path_set_target_velocity, on_static, .{ .f = &.{2} });
    const kinematic_post = try b.body(.{ .position = .{ o[0] - 2, o[1] + 4, o[2] + 4 }, .shape = post, .motion_type = 1, .linear_velocity = .{ 0.3, 0, 0 }, .angular_velocity = .{ 0, 0.5, 0 }, .object_layer = Layers.moving }, .activate);
    const d = try b.body(.{ .position = .{ o[0] - 0.5, o[1] + 4, o[2] + 4 }, .shape = box, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(pathWithMotor(kinematic_post, d, &circle, true, .{ 0, 0, 0 }, 0, PathRotation.around_normal, 5, 0, Desc.default_motor));
    _ = try b.body(.{ .position = .{ o[0] - 1, o[1] + 0.2, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);
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

/// The rotation of a lying gear (a cylinder whose axis, Y, lies along Z)
const gear_lying = axisAngle(.{ 1, 0, 0 }, 0.5 * std.math.pi);

/// A gear body: a lying cylinder at `p` (no collisions unless `layer` says otherwise)
fn gearBody(b: *SceneBuilder, shape: u32, p: [3]Real, opts: struct { spin: f32 = 0, layer: u32 = Layers.unused1, rotation: [4]f32 = gear_lying, gravity_factor: f32 = 1 }) Allocator.Error!u32 {
    return b.body(.{ .position = p, .rotation = opts.rotation, .shape = shape, .angular_velocity = .{ 0, 0, opts.spin }, .gravity_factor = opts.gravity_factor, .object_layer = opts.layer }, .activate);
}

/// The hinge of a gear: from the world around Z at its center
fn gearHinge(body: u32, p: [3]Real) SimConstraint {
    return Desc.hinge(invalid_handle, body, p, .{ 0, 0, 1 }, .{ 1, 0, 0 });
}

/// A GearConstraint around Z on both bodies with SetRatio(teeth1, teeth2), coupled to the hinges `hinge1` / `hinge2`
fn gearWithTeeth(body1: u32, body2: u32, hinge1: u32, hinge2: u32, teeth1: u32, teeth2: u32) SimConstraint {
    var c = Desc.gear(body1, body2, .{ 0, 0, 1 }, .{ 0, 0, 1 }, 1, hinge1, hinge2);
    c.i[1] = 1;
    c.i[2] = teeth1;
    c.i[3] = teeth2;
    return c;
}

/// A RackAndPinionConstraint (pinion around Z, rack along X) with SetRatio(rack teeth, rack length, pinion teeth),
/// coupled to the pinion hinge `pinion` and the rack slider `rack`
fn rackAndPinionWithTeeth(pinion_body: u32, rack_body: u32, pinion: u32, rack: u32, rack_teeth: u32, rack_length: f32, pinion_teeth: u32) SimConstraint {
    var c = Desc.rackAndPinion(pinion_body, rack_body, .{ 0, 0, 1 }, .{ 1, 0, 0 }, 1, pinion, rack);
    c.i[1] = 1;
    c.i[2] = rack_teeth;
    c.i[3] = pinion_teeth;
    c.f[7] = rack_length;
    return c;
}

/// The motor hinge operations: SetMotorState(state)
fn setHingeMotorState(b: *SceneBuilder, handle: u32, state: u32) Allocator.Error!void {
    try b.constraintOp(.hinge_set_motor_state, handle, .{ .i = &.{state} });
}

/// Gear trains driven by hinge motors in every EMotorState, ratios from SetRatio and explicit (negative for a bevel gear
/// with perpendicular axes), long enough for the hinge angles to wrap around (the position correction then takes
/// fmod(gear1rot + ratio gear2rot, 2 pi)): a velocity motor driving three gears; a position motor (whose targets jump
/// across +-pi); a position and velocity motor with damping only and later with a spring; the motor off with friction
/// on a spinning train; a bevel gear; a driven gear with hard limits and one with a soft limit (the motors are torque
/// limited); a gear without hinges at first (drift without position correction, then corrected after SetConstraints,
/// then one hinge removed again); a gear in local space driven by a position motor in the stiffness mode.
/// Jolt's GearConstraintPart applies lambda I2^-1 b to body 2 where its Jacobian [a, r b] asks for r lambda I2^-1 b
/// (reproduced): an iteration scales the velocity error by 1 - (A + r B) / (A + r^2 B) (A = a . I1^-1 a, B = b . I2^-1
/// b), which diverges unless 0 < (A + r B) / (A + r^2 B) < 2: the gear scenes use ratios of at least 0.5, negative
/// ratios only with A > |r| B (Zolt asserts on the infinite velocities that Jolt's release build would produce)
fn buildGearMotors(b: *SceneBuilder) Allocator.Error!void {
    const big = try b.cylinder(0.1, 0.5, 0.05);
    const small = try b.cylinder(0.1, 0.25, 0.05);
    b.num_updates = 150;
    const z: [3]f32 = .{ 0, 0, 1 };
    const y: Real = 4;

    // A velocity motor driving three gears (20 -> 10 -> 20 teeth)
    const t0 = [3]u32{ try gearBody(b, big, .{ 0, y, 0 }, .{}), try gearBody(b, small, .{ 0.8, y, 0 }, .{}), try gearBody(b, big, .{ 1.6, y, 0 }, .{}) };
    var h0_desc = gearHinge(t0[0], .{ 0, y, 0 });
    Desc.setMotor(&h0_desc, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -500, 500 });
    const h0 = try b.constraint(h0_desc);
    const h0b = try b.constraint(gearHinge(t0[1], .{ 0.8, y, 0 }));
    const h0c = try b.constraint(gearHinge(t0[2], .{ 1.6, y, 0 }));
    _ = try b.constraint(gearWithTeeth(t0[0], t0[1], h0, h0b, 20, 10));
    _ = try b.constraint(gearWithTeeth(t0[1], t0[2], h0b, h0c, 10, 20));

    // A position motor (12 -> 36 teeth)
    const t1 = [2]u32{ try gearBody(b, big, .{ 4, y, 0 }, .{}), try gearBody(b, small, .{ 4.8, y, 0 }, .{}) };
    var h1_desc = gearHinge(t1[0], .{ 4, y, 0 });
    Desc.setMotor(&h1_desc, 2, 17, 0, .{ 3, 1, -flt_max, flt_max, -500, 500 });
    const h1 = try b.constraint(h1_desc);
    const h1b = try b.constraint(gearHinge(t1[1], .{ 4.8, y, 0 }));
    _ = try b.constraint(gearWithTeeth(t1[0], t1[1], h1, h1b, 12, 36));

    // A position and velocity motor with damping only (stiffness mode, zero stiffness)
    const t2 = [2]u32{ try gearBody(b, big, .{ 8, y, 0 }, .{}), try gearBody(b, big, .{ 9.1, y, 0 }, .{}) };
    var h2_desc = gearHinge(t2[0], .{ 8, y, 0 });
    Desc.setMotor(&h2_desc, 2, 17, 1, .{ 0, 20, -flt_max, flt_max, -flt_max, flt_max });
    const h2 = try b.constraint(h2_desc);
    const h2b = try b.constraint(gearHinge(t2[1], .{ 9.1, y, 0 }));
    _ = try b.constraint(gearWithTeeth(t2[0], t2[1], h2, h2b, 16, 16));

    // The motor off with friction on a spinning train
    const t3 = [2]u32{ try gearBody(b, big, .{ 12, y, 0 }, .{ .spin = 6 }), try gearBody(b, small, .{ 12.8, y, 0 }, .{ .spin = -12 }) };
    var h3_desc = gearHinge(t3[0], .{ 12, y, 0 });
    h3_desc.f[16] = 0.5; // mMaxFrictionTorque
    const h3 = try b.constraint(h3_desc);
    var h3b_desc = gearHinge(t3[1], .{ 12.8, y, 0 });
    h3b_desc.f[16] = 0.1;
    const h3b = try b.constraint(h3b_desc);
    _ = try b.constraint(gearWithTeeth(t3[0], t3[1], h3, h3b, 24, 12));

    // A bevel gear: the driven gear stands (its axis along X), explicit negative ratio (a small driver: with Jolt's
    // GearConstraintPart a negative ratio r only converges when a . I1^-1 a > |r| b . I2^-1 b, see buildGearMotors)
    const t4 = [2]u32{ try gearBody(b, small, .{ 16, y, 0 }, .{}), try gearBody(b, big, .{ 16.7, y, 0.6 }, .{ .rotation = axisAngle(z, 0.5 * std.math.pi) }) };
    var h4_desc = gearHinge(t4[0], .{ 16, y, 0 });
    Desc.setMotor(&h4_desc, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -200, 200 });
    const h4 = try b.constraint(h4_desc);
    const h4b = try b.constraint(Desc.hinge(invalid_handle, t4[1], .{ 16.7, y, 0.6 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }));
    _ = try b.constraint(Desc.gear(t4[0], t4[1], z, .{ 1, 0, 0 }, -0.5, h4, h4b));

    // A driven gear with hard limits (the torque limited velocity motor stalls at them)
    const t5 = [2]u32{ try gearBody(b, big, .{ 20, y, 0 }, .{}), try gearBody(b, big, .{ 21.1, y, 0 }, .{}) };
    var h5_desc = gearHinge(t5[0], .{ 20, y, 0 });
    Desc.setMotor(&h5_desc, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -50, 50 });
    const h5 = try b.constraint(h5_desc);
    var h5b_desc = gearHinge(t5[1], .{ 21.1, y, 0 });
    h5b_desc.f[12] = -1;
    h5b_desc.f[13] = 1;
    const h5b = try b.constraint(h5b_desc);
    _ = try b.constraint(gearWithTeeth(t5[0], t5[1], h5, h5b, 10, 10));

    // A driven gear with a soft limit (frequency and damping)
    const t6 = [2]u32{ try gearBody(b, big, .{ 24, y, 0 }, .{}), try gearBody(b, small, .{ 24.8, y, 0 }, .{}) };
    var h6_desc = gearHinge(t6[0], .{ 24, y, 0 });
    Desc.setMotor(&h6_desc, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -20, 20 });
    const h6 = try b.constraint(h6_desc);
    var h6b_desc = gearHinge(t6[1], .{ 24.8, y, 0 });
    h6b_desc.f[12] = -0.5;
    h6b_desc.f[13] = 0.5;
    Desc.setSpring(&h6b_desc, 1, 14, 0, 2, 0.5);
    const h6b = try b.constraint(h6b_desc);
    _ = try b.constraint(Desc.gear(t6[0], t6[1], z, z, 0.75, h6, h6b));

    // A gear without hinges at first
    const t7 = [2]u32{ try gearBody(b, big, .{ 28, y, 0 }, .{}), try gearBody(b, small, .{ 28.8, y, 0 }, .{}) };
    var h7_desc = gearHinge(t7[0], .{ 28, y, 0 });
    Desc.setMotor(&h7_desc, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -300, 300 });
    const h7 = try b.constraint(h7_desc);
    const h7b = try b.constraint(gearHinge(t7[1], .{ 28.8, y, 0 }));
    const g7 = try b.constraint(Desc.gear(t7[0], t7[1], z, z, 2, invalid_handle, invalid_handle));

    // A gear in local space (the local Y axis of the lying cylinders is world Z)
    const t8 = [2]u32{ try gearBody(b, big, .{ 32, y, 0 }, .{}), try gearBody(b, big, .{ 33.1, y, 0 }, .{}) };
    var h8_desc = gearHinge(t8[0], .{ 32, y, 0 });
    Desc.setMotor(&h8_desc, 2, 17, 1, .{ 500, 50, -flt_max, flt_max, -flt_max, flt_max });
    const h8 = try b.constraint(h8_desc);
    const h8b = try b.constraint(gearHinge(t8[1], .{ 33.1, y, 0 }));
    var g8_desc = gearWithTeeth(t8[0], t8[1], h8, h8b, 15, 25);
    g8_desc.i[0] = ConstraintSpace.local_to_body_com;
    Desc.set3(g8_desc.f[0..], .{ 0, 1, 0 });
    Desc.set3(g8_desc.f[3..], .{ 0, 1, 0 });
    const g8 = try b.constraint(g8_desc);

    // A high ratio (5 -> 35 teeth: gear1rot + 7 gear2rot reaches 6 pi, where fmod differs from repeated subtractions of
    // 2 pi, which are exact up to 2 pi + 8) with a small driver (see above)
    const t9 = [2]u32{ try gearBody(b, small, .{ 36, y, 0 }, .{}), try gearBody(b, big, .{ 36.8, y, 0 }, .{}) };
    var h9_desc = gearHinge(t9[0], .{ 36, y, 0 });
    Desc.setMotor(&h9_desc, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -500, 500 });
    const h9 = try b.constraint(h9_desc);
    const h9b = try b.constraint(gearHinge(t9[1], .{ 36.8, y, 0 }));
    _ = try b.constraint(gearWithTeeth(t9[0], t9[1], h9, h9b, 5, 35));

    b.at(1);
    try setHingeMotorState(b, h9, motor_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, h9, .{ .f = &.{10} });
    try setHingeMotorState(b, h0, motor_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, h0, .{ .f = &.{3} });
    try setHingeMotorState(b, h1, motor_position);
    try b.constraintOp(.hinge_set_target_angle, h1, .{ .f = &.{2.5} });
    try setHingeMotorState(b, h2, motor_position_and_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, h2, .{ .f = &.{-2} });
    try setHingeMotorState(b, h4, motor_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, h4, .{ .f = &.{2} });
    try setHingeMotorState(b, h5, motor_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, h5, .{ .f = &.{1.5} });
    try setHingeMotorState(b, h6, motor_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, h6, .{ .f = &.{2} });
    try setHingeMotorState(b, h7, motor_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, h7, .{ .f = &.{4} });
    try setHingeMotorState(b, h8, motor_position);
    try b.constraintOp(.hinge_set_target_angle, h8, .{ .f = &.{1.5} });
    b.at(40);
    try b.constraintOp(.hinge_set_target_angle, h1, .{ .f = &.{-2.5} }); // Across +-pi
    try b.constraintOp(.hinge_set_motor_settings, h2, .{ .i = &.{0}, .f = &.{ 1.5, 0.8, -flt_max, flt_max, -flt_max, flt_max } });
    try b.constraintOp(.hinge_set_target_angle, h2, .{ .f = &.{0.5} });
    try b.constraintOp(.constraint_settings_hash, g8, .{});
    b.at(60);
    try b.constraintOp(.gear_set_constraints, g7, .{ .i = &.{ h7, h7b } });
    try b.constraintOp(.hinge_set_target_angular_velocity, h5, .{ .f = &.{-1.5} });
    try b.constraintOp(.hinge_set_target_angle, h8, .{ .f = &.{-3} });
    b.at(80);
    try setHingeMotorState(b, h0, motor_off);
    try b.constraintOp(.hinge_set_max_friction_torque, h0, .{ .f = &.{5} });
    try setHingeMotorState(b, h4, motor_position_and_velocity);
    try b.constraintOp(.hinge_set_target_angle, h4, .{ .f = &.{-1} });
    try b.constraintOp(.hinge_set_target_angular_velocity, h4, .{ .f = &.{0.5} });
    try b.constraintOp(.hinge_set_limits_spring_settings, h6b, .{ .i = &.{1}, .f = &.{ 200, 5 } }); // Stiffness mode
    b.at(100);
    try b.constraintOp(.gear_set_constraints, g7, .{ .i = &.{ h7, invalid_handle } });
    try setHingeMotorState(b, h1, motor_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, h1, .{ .f = &.{-5} });
    try b.constraintOp(.reset_constraint_warm_start, g7, .{});
    b.at(120);
    try b.op(.set_rotation, t3[0], .{ .f = &axisAngle(z, 3) }); // A position error beyond pi
}

/// Racks and pinions driven by slider motors in every EMotorState (and one pinion driven by its hinge motor), ratios
/// from SetRatio and explicit: a velocity motor into the hard limit of the rack and back; a position motor; a position
/// and velocity motor with damping only; the motor off with friction on a thrown rack; a velocity motor into a soft
/// limit (frequency and damping); a hinge motor on the pinion pushing the rack into a soft limit in the stiffness mode; a
/// rack and pinion in local space without its constraints at first; a pinion with hard limits that stop a force limited
/// rack; a high ratio (rotation - ratio * translation beyond 2 pi + 8, where fmod differs from repeated subtractions of
/// 2 pi)
fn buildRackAndPinionMotors(b: *SceneBuilder) Allocator.Error!void {
    const wheel = try b.cylinder(0.1, 0.3, 0.05);
    const rack_shape = try b.box(.{ 1.2, 0.08, 0.08 }, 0.04);
    b.num_updates = 150;
    const y: Real = 4;
    const ry: Real = 3.62;
    var pinion_bodies: [9]u32 = undefined;
    var rack_bodies: [9]u32 = undefined;
    var pinions: [9]u32 = undefined;
    var racks: [9]u32 = undefined;
    var rps: [9]u32 = undefined;
    for (0..9) |k| {
        const x: Real = 4 * real(k);
        pinion_bodies[k] = try gearBody(b, wheel, .{ x, y, 0 }, .{});
        rack_bodies[k] = try b.body(.{ .position = .{ x, ry, 0 }, .shape = rack_shape, .linear_velocity = if (k == 3) .{ 2, 0, 0 } else @splat(0), .gravity_factor = if (k == 3) 0 else 1 }, .activate);
        var hinge = gearHinge(pinion_bodies[k], .{ x, y, 0 });
        var slider = Desc.slider(invalid_handle, rack_bodies[k], .{ 1, 0, 0 });
        switch (k) {
            0 => {
                slider.f[12] = -1.5;
                slider.f[13] = 1.5;
            },
            3 => {
                slider.f[16] = 2; // mMaxFrictionForce
                hinge.f[16] = 0.1;
            },
            4 => {
                slider.f[12] = -0.5;
                slider.f[13] = 0.5;
                Desc.setSpring(&slider, 3, 14, 0, 3, 0.3);
                Desc.setMotor(&slider, 4, 17, 0, .{ 2, 1, -30, 30, -flt_max, flt_max });
            },
            5 => {
                slider.f[12] = -0.3;
                slider.f[13] = 0.3;
                Desc.setSpring(&slider, 3, 14, 1, 200, 5);
                Desc.setMotor(&hinge, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -20, 20 });
            },
            7 => {
                hinge.f[12] = -1.2;
                hinge.f[13] = 1.2;
                Desc.setMotor(&slider, 4, 17, 0, .{ 2, 1, -40, 40, -flt_max, flt_max });
            },
            else => {},
        }
        if (k == 2) Desc.setMotor(&slider, 4, 17, 1, .{ 0, 30, -flt_max, flt_max, -flt_max, flt_max });
        pinions[k] = try b.constraint(hinge);
        racks[k] = try b.constraint(slider);
        var rp = switch (k) {
            0 => rackAndPinionWithTeeth(pinion_bodies[k], rack_bodies[k], pinions[k], racks[k], 30, 3, 10),
            1 => rackAndPinionWithTeeth(pinion_bodies[k], rack_bodies[k], pinions[k], racks[k], 20, 4, 10),
            2, 4 => rackAndPinionWithTeeth(pinion_bodies[k], rack_bodies[k], pinions[k], racks[k], 12, 2.4, 9),
            3 => Desc.rackAndPinion(pinion_bodies[k], rack_bodies[k], .{ 0, 0, 1 }, .{ 1, 0, 0 }, -3, pinions[k], racks[k]),
            6 => Desc.rackAndPinion(pinion_bodies[k], rack_bodies[k], .{ 0, 1, 0 }, .{ 1, 0, 0 }, -2, invalid_handle, invalid_handle),
            8 => rackAndPinionWithTeeth(pinion_bodies[k], rack_bodies[k], pinions[k], racks[k], 40, 2, 8), // 5 pi rad / m
            else => Desc.rackAndPinion(pinion_bodies[k], rack_bodies[k], .{ 0, 0, 1 }, .{ 1, 0, 0 }, 1.0 / 0.3, pinions[k], racks[k]),
        };
        if (k == 6) rp.i[0] = ConstraintSpace.local_to_body_com; // The local Y axis of the lying pinion is world Z
        rps[k] = try b.constraint(rp);
    }

    b.at(1);
    try setSliderMotorState(b, racks[0], motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, racks[0], .{ .f = &.{1} });
    try setSliderMotorState(b, racks[1], motor_position);
    try b.constraintOp(.slider_set_target_position, racks[1], .{ .f = &.{0.8} });
    try setSliderMotorState(b, racks[2], motor_position_and_velocity);
    try b.constraintOp(.slider_set_target_velocity, racks[2], .{ .f = &.{0.7} });
    try setSliderMotorState(b, racks[4], motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, racks[4], .{ .f = &.{1} });
    try setHingeMotorState(b, pinions[5], motor_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, pinions[5], .{ .f = &.{2} });
    try setSliderMotorState(b, racks[6], motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, racks[6], .{ .f = &.{-0.6} });
    try setSliderMotorState(b, racks[7], motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, racks[7], .{ .f = &.{0.8} });
    try setSliderMotorState(b, racks[8], motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, racks[8], .{ .f = &.{0.6} }); // rotation - ratio translation beyond 2 pi + 8
    b.at(50);
    try b.constraintOp(.rack_and_pinion_set_constraints, rps[6], .{ .i = &.{ pinions[6], racks[6] } });
    try b.constraintOp(.slider_set_target_position, racks[1], .{ .f = &.{-0.6} });
    try b.constraintOp(.constraint_settings_hash, rps[6], .{});
    b.at(70);
    try setSliderMotorState(b, racks[2], motor_position_and_velocity);
    try b.constraintOp(.slider_set_motor_settings, racks[2], .{ .i = &.{0}, .f = &.{ 2, 0.8, -flt_max, flt_max, -flt_max, flt_max } });
    try b.constraintOp(.slider_set_target_position, racks[2], .{ .f = &.{-0.2} });
    try b.constraintOp(.slider_set_target_velocity, racks[2], .{ .f = &.{0} });
    try b.constraintOp(.hinge_set_target_angular_velocity, pinions[5], .{ .f = &.{-2} });
    b.at(90);
    try b.constraintOp(.rack_and_pinion_set_constraints, rps[6], .{ .i = &.{ invalid_handle, racks[6] } });
    try b.constraintOp(.reset_constraint_warm_start, rps[4], .{});
    try setSliderMotorState(b, racks[7], motor_off);
    try b.constraintOp(.slider_set_max_friction_force, racks[7], .{ .f = &.{5} });
    b.at(100);
    try b.constraintOp(.slider_set_target_velocity, racks[0], .{ .f = &.{-1.5} });
    try b.constraintOp(.constraint_settings_hash, rps[0], .{});
    b.at(120);
    try b.op(.set_position, rack_bodies[1], .{ .r = .{ 4.6, ry, 0 } }); // A jump of the rack
}

/// Gears and racks in islands with other constraint kinds and contacts: a cart on the floor carries two gears and a
/// pinion on hinges from the cart body (not the world: the angles are relative to the moving cart); a velocity motor on
/// the first gear drives the second one, whose welded arm (FixedConstraint) swings into the cart and a crate, and the
/// pinion, which moves a rack on a slider from the cart into a crate; a chain of point constraints hangs from the first
/// gear onto the floor; crates fall on a gear pair on the floor
fn buildGearIslands(b: *SceneBuilder) Allocator.Error!void {
    const cart_shape = try b.box(.{ 2.0, 0.2, 0.6 }, 0.05);
    const wheel = try b.cylinder(0.1, 0.4, 0.05);
    const small_wheel = try b.cylinder(0.1, 0.2, 0.05);
    const arm_shape = try b.box(.{ 0.05, 0.5, 0.05 }, 0.02);
    const rack_shape = try b.box(.{ 0.8, 0.06, 0.06 }, 0.03);
    const crate = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    const link = try b.box(.{ 0.05, 0.15, 0.05 }, 0.02);
    b.num_updates = 120;
    b.max_contact_constraints = 2048;
    _ = try addFloor(b, 20, 0);

    // The cart (moving layer: it collides with the floor, the arm, the rack and the crates; the gears are in moving2)
    const cart = try b.body(.{ .position = .{ 0, 0.2, 0 }, .shape = cart_shape, .friction = 0.8, .object_layer = Layers.moving }, .activate);
    const g1 = try gearBody(b, wheel, .{ -0.6, 1.0, 0 }, .{ .layer = Layers.moving2 });
    const g2 = try gearBody(b, small_wheel, .{ 0.05, 1.0, 0 }, .{ .layer = Layers.moving2 });
    const pinion_body = try gearBody(b, small_wheel, .{ 0.7, 0.75, 0 }, .{ .layer = Layers.moving2 });
    var h1_desc = Desc.hinge(cart, g1, .{ -0.6, 1.0, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 });
    Desc.setMotor(&h1_desc, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -100, 100 });
    const h1 = try b.constraint(h1_desc);
    const h2 = try b.constraint(Desc.hinge(cart, g2, .{ 0.05, 1.0, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 }));
    const hp = try b.constraint(Desc.hinge(cart, pinion_body, .{ 0.7, 0.75, 0 }, .{ 0, 0, 1 }, .{ 1, 0, 0 }));
    _ = try b.constraint(gearWithTeeth(g1, g2, h1, h2, 20, 10));
    _ = try b.constraint(gearWithTeeth(g2, pinion_body, h2, hp, 10, 10));

    // An arm welded to the second gear, a crate on the cart next to it
    const arm = try b.body(.{ .position = .{ 0.05, 1.5, 0.25 }, .shape = arm_shape, .object_layer = Layers.moving }, .activate);
    _ = try b.constraint(Desc.fixedAuto(g2, arm));
    _ = try b.body(.{ .position = .{ -0.9, 0.61, 0.3 }, .shape = crate, .object_layer = Layers.moving }, .activate);

    // A rack on a slider from the cart, driven by the pinion, and a crate on the cart in its way
    const rack_body = try b.body(.{ .position = .{ 0.7, 0.5, 0.3 }, .shape = rack_shape, .object_layer = Layers.moving }, .activate);
    var rack_desc = Desc.sliderAxes(cart, rack_body, .{ 0.7, 0.5, 0.3 }, .{ 1, 0, 0 }, .{ 0, 1, 0 });
    rack_desc.f[12] = -0.6;
    rack_desc.f[13] = 0.6;
    const rack = try b.constraint(rack_desc);
    _ = try b.constraint(rackAndPinionWithTeeth(pinion_body, rack_body, hp, rack, 16, 1.6, 8));
    _ = try b.body(.{ .position = .{ 1.75, 0.61, 0.3 }, .shape = crate, .object_layer = Layers.moving }, .activate);

    // A chain hanging from the first gear onto the floor (moving2: the floor only)
    var previous = g1;
    for (0..3) |k| {
        const p: [3]Real = .{ -0.25, 0.75 - 0.3 * real(k), 0.2 };
        const l = try b.body(.{ .position = p, .shape = link, .object_layer = Layers.moving2 }, .activate);
        _ = try b.constraint(Desc.pointWorld(previous, l, .{ p[0], p[1] + 0.15, p[2] }));
        previous = l;
    }

    // Crates falling on a gear pair on the floor
    const f1 = try gearBody(b, wheel, .{ 4, 0.5, 0 }, .{ .layer = Layers.moving });
    const f2 = try gearBody(b, wheel, .{ 4.85, 0.5, 0 }, .{ .layer = Layers.moving });
    const fh1 = try b.constraint(gearHinge(f1, .{ 4, 0.5, 0 }));
    const fh2 = try b.constraint(gearHinge(f2, .{ 4.85, 0.5, 0 }));
    _ = try b.constraint(gearWithTeeth(f1, f2, fh1, fh2, 18, 18));
    for (0..3) |k| _ = try b.body(.{ .position = .{ 3.7 + 0.5 * real(k), 2 + 0.6 * real(k), 0 }, .shape = crate, .object_layer = Layers.moving }, .activate);

    b.at(1);
    try setHingeMotorState(b, h1, motor_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, h1, .{ .f = &.{-2} });
    b.at(60);
    try b.constraintOp(.hinge_set_target_angular_velocity, h1, .{ .f = &.{3} });
    try b.op(.add_impulse, cart, .{ .f = &.{ 0, 0, 200 } });
    b.at(90);
    try setHingeMotorState(b, h1, motor_position);
    try b.constraintOp(.hinge_set_target_angle, h1, .{ .f = &.{0} });
}

/// A chain of 70 gears on hinges from the world (alternating 10 and 20 teeth, friction on the hinges): one island of 139
/// constraints that the LargeIslandSplitter splits; the first gear spins, the train slows down and falls asleep from the
/// final batch of a split island; an angular impulse on a gear in the middle wakes its neighbours through the gears
fn buildGearSplitter(b: *SceneBuilder) Allocator.Error!void {
    const big = try b.cylinder(0.05, 0.3, 0.02);
    const small = try b.cylinder(0.05, 0.15, 0.02);
    b.num_updates = 140;
    b.settings.time_before_sleep = 0.25;
    const n = 70;
    var gears: [n]u32 = undefined;
    var hinges: [n]u32 = undefined;
    for (&gears, &hinges, 0..) |*g, *h, k| {
        const p: [3]Real = .{ 0.5 * real(k % 35), 3 + 2 * real(k / 35), 0 };
        g.* = try gearBody(b, if (k % 2 == 0) big else small, p, .{ .spin = if (k == 0) 2 else 0 });
        var hinge = gearHinge(g.*, p);
        hinge.f[16] = 0.5;
        h.* = try b.constraint(hinge);
        if (k > 0) _ = try b.constraint(gearWithTeeth(gears[k - 1], g.*, hinges[k - 1], h.*, if (k % 2 == 1) 20 else 10, if (k % 2 == 1) 10 else 20));
    }
    b.at(110);
    try b.expectSleeping(gears[0]);
    try b.expectSleeping(gears[n - 1]);
    try b.op(.add_angular_impulse, gears[n / 2], .{ .f = &.{ 0, 0, 0.05 } });
    b.at(111);
    try b.expectAwake(gears[n / 2 - 1]); // Woken by their gear constraints (one constraint per update)
    try b.expectAwake(gears[n / 2 + 1]);
}

/// Gears and racks with sleeping bodies, and cross save / restore of their state (the total lambda of the part; the
/// constraints of SetConstraints are not state): a gear pair with friction comes to rest and falls asleep, SetConstraints
/// on the sleeping pair wakes nothing until an angular impulse does; a gear created between an awake and a sleeping gear
/// wakes the sleeping one; a rack and pinion driven by a slider motor is restored from snapshots of everything and of the
/// constraints only after its constraints changed
fn buildGearSleepAndSaveRestore(b: *SceneBuilder) Allocator.Error!void {
    const wheel = try b.cylinder(0.1, 0.4, 0.05);
    const rack_shape = try b.box(.{ 1.0, 0.06, 0.06 }, 0.03);
    b.num_updates = 100;
    b.settings.time_before_sleep = 0.2;
    const state_constraints: u32 = 8; // EStateRecorderState::Constraints

    // A gear pair with friction that comes to rest
    const r1 = try gearBody(b, wheel, .{ 0, 4, 0 }, .{ .spin = 1 });
    const r2 = try gearBody(b, wheel, .{ 0.85, 4, 0 }, .{});
    var rh1_desc = gearHinge(r1, .{ 0, 4, 0 });
    rh1_desc.f[16] = 30;
    const rh1 = try b.constraint(rh1_desc);
    const rh2 = try b.constraint(gearHinge(r2, .{ 0.85, 4, 0 }));
    const resting = try b.constraint(Desc.gear(r1, r2, .{ 0, 0, 1 }, .{ 0, 0, 1 }, 1, invalid_handle, invalid_handle));

    // An awake gear that spins and a sleeping one
    const awake = try gearBody(b, wheel, .{ 4, 4, 0 }, .{ .spin = 2 });
    const asleep = try b.body(.{ .position = .{ 4.85, 4, 0 }, .rotation = gear_lying, .shape = wheel, .object_layer = Layers.unused1 }, .dont_activate);
    var ah_desc = gearHinge(awake, .{ 4, 4, 0 });
    Desc.setMotor(&ah_desc, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -50, 50 });
    const awake_hinge = try b.constraint(ah_desc);
    const asleep_hinge = try b.constraint(gearHinge(asleep, .{ 4.85, 4, 0 }));

    // A rack and pinion driven by a slider motor
    const pinion_body = try gearBody(b, wheel, .{ 8, 4, 0 }, .{});
    const rack_body = try b.body(.{ .position = .{ 8, 3.5, 0 }, .shape = rack_shape, .object_layer = Layers.unused1 }, .activate);
    const pinion = try b.constraint(gearHinge(pinion_body, .{ 8, 4, 0 }));
    const rack = try b.constraint(sliderWithLimits(invalid_handle, rack_body, .{ 1, 0, 0 }, -0.8, 0.8));
    const rp = try b.constraint(rackAndPinionWithTeeth(pinion_body, rack_body, pinion, rack, 10, 2, 8));

    b.at(2);
    try setSliderMotorState(b, rack, motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, rack, .{ .f = &.{0.5} });
    try setHingeMotorState(b, awake_hinge, motor_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, awake_hinge, .{ .f = &.{2} });
    b.at(5);
    try b.expectSleeping(asleep);
    const pair = try b.constraint(gearWithTeeth(awake, asleep, awake_hinge, asleep_hinge, 12, 12)); // Wakes asleep in the next update
    b.at(6);
    try b.expectAwake(asleep);
    b.at(20);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(25);
    try b.constraintOp(.rack_and_pinion_set_constraints, rp, .{ .i = &.{ invalid_handle, invalid_handle } }); // Not state
    try b.constraintOp(.slider_set_target_velocity, rack, .{ .f = &.{-0.5} });
    b.at(35);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // The bodies and impulses of update 20, the motor goes on with -0.5
    b.at(60);
    try b.expectSleeping(r1);
    try b.constraintOp(.gear_set_constraints, resting, .{ .i = &.{ rh1, rh2 } }); // The pair sleeps: nothing happens
    try b.expectConstraintState(resting, ConstraintState.inactive);
    try b.sysOp(.snapshot, .{ .index = 1, .i = &.{state_constraints} });
    b.at(61);
    try b.expectSleeping(r2);
    b.at(65);
    try b.op(.add_angular_impulse, r2, .{ .f = &.{ 0, 0, 2 } }); // Wakes the pair through the gear
    try b.constraintOp(.rack_and_pinion_set_constraints, rp, .{ .i = &.{ pinion, rack } });
    try b.constraintOp(.reset_constraint_warm_start, pair, .{});
    b.at(75);
    try b.expectAwake(r1);
    try b.sysOp(.restore_cross, .{ .index = 1 }); // The constraints of update 60: the total lambdas and the motor states
    try b.constraintOp(.constraint_settings_hash, rp, .{});
    b.at(85);
    try b.sysOp(.snapshot, .{ .index = 2 });
    try b.op(.add_angular_impulse, pinion_body, .{ .f = &.{ 0, 0, 1 } });
    b.at(92);
    try b.sysOp(.restore_cross, .{ .index = 2 });
}

/// A gear train and a rack and pinion with several collision steps per update and changing update parameters (the warm
/// start impulses are scaled by the step ratio)
fn buildGearCollisionSteps(comptime steps: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const wheel = try b.cylinder(0.1, 0.4, 0.05);
            const rack_shape = try b.box(.{ 1.0, 0.06, 0.06 }, 0.03);
            b.num_updates = 50;
            b.collision_steps = steps;
            b.delta_time = @as(f32, @floatFromInt(steps)) / 60.0;
            var gears: [3]u32 = undefined;
            var hinges: [3]u32 = undefined;
            for (&gears, &hinges, 0..) |*g, *h, k| {
                const p: [3]Real = .{ 0.85 * real(k), 5, 0 };
                g.* = try gearBody(b, wheel, p, .{});
                var hinge = gearHinge(g.*, p);
                if (k == 0) Desc.setMotor(&hinge, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -100, 100 });
                h.* = try b.constraint(hinge);
            }
            _ = try b.constraint(gearWithTeeth(gears[0], gears[1], hinges[0], hinges[1], 10, 15));
            _ = try b.constraint(Desc.gear(gears[1], gears[2], .{ 0, 0, 1 }, .{ 0, 0, 1 }, 0.8, hinges[1], hinges[2]));
            const pinion_body = try gearBody(b, wheel, .{ 4, 5, 0 }, .{});
            const rack_body = try b.body(.{ .position = .{ 4, 4.5, 0 }, .shape = rack_shape, .object_layer = Layers.unused1 }, .activate);
            const pinion = try b.constraint(gearHinge(pinion_body, .{ 4, 5, 0 }));
            const rack = try b.constraint(sliderWithLimits(invalid_handle, rack_body, .{ 1, 0, 0 }, -0.7, 0.7));
            _ = try b.constraint(rackAndPinionWithTeeth(pinion_body, rack_body, pinion, rack, 20, 2.5, 10));
            b.at(1);
            try setHingeMotorState(b, hinges[0], motor_velocity);
            try b.constraintOp(.hinge_set_target_angular_velocity, hinges[0], .{ .f = &.{-2} });
            try setSliderMotorState(b, rack, motor_velocity);
            try b.constraintOp(.slider_set_target_velocity, rack, .{ .f = &.{0.5} });
            b.at(20);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 90.0}, .count = 1 });
            try setHingeMotorState(b, hinges[0], motor_position);
            try b.constraintOp(.hinge_set_target_angle, hinges[0], .{ .f = &.{0.3} });
            try setSliderMotorState(b, rack, motor_position);
            try b.constraintOp(.slider_set_target_position, rack, .{ .f = &.{-0.4} });
            b.at(30);
            try b.sysOp(.set_update, .{ .f = &.{1.0 / 20.0}, .count = steps + 1 });
        }
    }.build;
}

/// Gears and a rack and pinion far from the origin (world space points are Real: in double precision the hinges and
/// the slider keep their precision): a gear train on a cart that falls onto the floor and rolls on, a rack and pinion
/// driven by a slider motor that pushes a box
fn buildGearFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const wheel = try b.cylinder(0.1, 0.35, 0.05);
    const rack_shape = try b.box(.{ 0.8, 0.06, 0.06 }, 0.03);
    const cart_shape = try b.box(.{ 0.9, 0.15, 0.5 }, 0.05);
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    b.num_updates = 80;
    const o: [3]Real = .{ -8000, 300, 15000 };
    const floor = try b.box(.{ 10, 1, 10 }, 0);
    _ = try b.body(.{ .position = .{ o[0], o[1] - 1, o[2] }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);

    // A cart with two gears on hinges from the cart
    const cart = try b.body(.{ .position = .{ o[0] - 3, o[1] + 0.5, o[2] }, .shape = cart_shape, .linear_velocity = .{ 1, 0, 0 }, .object_layer = Layers.moving }, .activate);
    const g1 = try gearBody(b, wheel, .{ o[0] - 3.4, o[1] + 1.1, o[2] }, .{ .layer = Layers.moving2 });
    const g2 = try gearBody(b, wheel, .{ o[0] - 2.65, o[1] + 1.1, o[2] }, .{ .layer = Layers.moving2 });
    var h1_desc = Desc.hinge(cart, g1, .{ o[0] - 3.4, o[1] + 1.1, o[2] }, .{ 0, 0, 1 }, .{ 1, 0, 0 });
    Desc.setMotor(&h1_desc, 2, 17, 0, .{ 2, 1, -flt_max, flt_max, -80, 80 });
    const h1 = try b.constraint(h1_desc);
    const h2 = try b.constraint(Desc.hinge(cart, g2, .{ o[0] - 2.65, o[1] + 1.1, o[2] }, .{ 0, 0, 1 }, .{ 1, 0, 0 }));
    _ = try b.constraint(gearWithTeeth(g1, g2, h1, h2, 14, 21));

    // A rack and pinion from the world, the rack pushes a box
    const pinion_body = try gearBody(b, wheel, .{ o[0] + 2, o[1] + 0.9, o[2] }, .{ .layer = Layers.moving2 });
    const rack_body = try b.body(.{ .position = .{ o[0] + 2, o[1] + 0.49, o[2] }, .shape = rack_shape, .object_layer = Layers.moving }, .activate);
    const pinion = try b.constraint(gearHinge(pinion_body, .{ o[0] + 2, o[1] + 0.9, o[2] }));
    const rack = try b.constraint(sliderWithLimits(invalid_handle, rack_body, .{ 1, 0, 0 }, -1, 1));
    _ = try b.constraint(rackAndPinionWithTeeth(pinion_body, rack_body, pinion, rack, 18, 1.8, 12));
    _ = try b.body(.{ .position = .{ o[0] + 3.1, o[1] + 0.25, o[2] }, .shape = box, .object_layer = Layers.moving }, .activate);

    b.at(1);
    try setHingeMotorState(b, h1, motor_velocity);
    try b.constraintOp(.hinge_set_target_angular_velocity, h1, .{ .f = &.{4} });
    try setSliderMotorState(b, rack, motor_velocity);
    try b.constraintOp(.slider_set_target_velocity, rack, .{ .f = &.{0.8} });
    b.at(40);
    try setSliderMotorState(b, rack, motor_position);
    try b.constraintOp(.slider_set_target_position, rack, .{ .f = &.{-0.5} });
}

/// Random gear trains and racks and pinions (random axes in world or local space, ratios from SetRatio or explicit,
/// hinge and slider motors in every state, limits, friction) with random operations between the updates: SetConstraints
/// (also with nullptr), the motor setters, impulses, rotations and the generic constraint operations (enable / disable,
/// remove / add, ResetWarmStart, the settings hash); gears in the moving layer collide with each other and with boxes on
/// the floor.
/// Only the committed seeds (1, 2, 4) are known to converge: the ratio and teeth rules do not rule out a combination of
/// ratios, limits and motors that makes the velocities infinite (seed 5 does, in Jolt too: Zolt's Debug build asserts in
/// MotionProperties.clampAngularVelocity, ReleaseFast matches Jolt bit for bit). Run a new seed in Debug before adding it.
fn buildGearRandom(comptime seed: u32) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        /// A ratio: positive from 0.6 (or of either sign for a rack and pinion, whose part is consistent with its
        /// Jacobian), negative above -0.8 for a gear between equal wheels (see buildGearMotors). This keeps most pairs
        /// convergent, not all of them (see above)
        fn ratio(b: *SceneBuilder, rack: bool) f32 {
            if (rack) return if (b.oneIn(3)) -b.float(0.3, 3) else b.float(0.3, 3);
            return if (b.oneIn(4)) -b.float(0.2, 0.8) else b.float(0.6, 3);
        }

        /// The numbers of teeth for SetRatio of a gear: gear 2 has at least 2/3 of the teeth of gear 1
        fn teeth(b: *SceneBuilder) [2]u32 {
            const teeth1 = 6 + b.index(24);
            return .{ teeth1, if (b.oneIn(2)) teeth1 + b.index(20) else teeth1 - b.index(teeth1 / 3) };
        }

        fn motor(b: *SceneBuilder) [7]f32 {
            const mode: f32 = @floatFromInt(b.index(3));
            const frequency = if (b.oneIn(5)) 0 else if (mode == 0) b.float(0.5, 8) else b.float(5, 300);
            const limit = if (b.oneIn(3)) flt_max else b.float(5, 200);
            return .{ mode, frequency, if (b.oneIn(4)) 0 else b.float(0, 2), -limit, limit, -limit, limit };
        }

        fn build(b: *SceneBuilder) Allocator.Error!void {
            b.rng = 0x6ea7_0000 +% seed *% 0x9e3779b9;
            const wheel = try b.cylinder(0.1, 0.35, 0.05);
            const rack_shape = try b.box(.{ 0.8, 0.06, 0.06 }, 0.03);
            const crate = try b.box(.{ 0.2, 0.2, 0.2 }, 0.05);
            b.num_updates = 90;
            b.max_contact_constraints = 2048;
            _ = try addFloor(b, 20, 0);
            const num_pairs = 8;
            var hinges: [2 * num_pairs]u32 = undefined; // The hinges of the gear bodies
            var sliders: [num_pairs]u32 = undefined; // The sliders of the racks (pairs that are racks and pinions)
            var couplings: [num_pairs]u32 = undefined;
            var is_rack: [num_pairs]bool = undefined;
            var bodies: [2 * num_pairs]u32 = undefined;
            var in_system: [num_pairs]bool = @splat(true);
            for (0..num_pairs) |k| {
                is_rack[k] = b.oneIn(3);
                const x: Real = 2.5 * real(k % 4) - 4;
                const z: Real = 2.5 * real(k / 4) - 1.5;
                const y: Real = 1.5 + real(b.float(0, 1));
                const layer = if (b.oneIn(2)) Layers.moving else Layers.unused1;
                // The axis of the pair: Z, X or a random direction around which the bodies are rotated
                const q = if (b.oneIn(2)) [4]f32{ 0, 0, 0, 1 } else b.rotation();
                const axis = quatRotate(q, .{ 0, 0, 1 });
                const normal = quatRotate(q, .{ 1, 0, 0 });
                const body_rotation = quatMul(q, gear_lying);
                const p1: [3]Real = .{ x, y, z };
                bodies[2 * k] = try b.body(.{ .position = p1, .rotation = body_rotation, .shape = wheel, .angular_velocity = if (b.oneIn(3)) scale3(axis, b.float(-3, 3)) else @splat(0), .object_layer = layer }, .activate);
                var h = Desc.hinge(invalid_handle, bodies[2 * k], p1, axis, normal);
                if (b.oneIn(3)) {
                    h.f[12] = -b.float(0.3, 3);
                    h.f[13] = b.float(0.3, 3);
                }
                if (b.oneIn(3)) h.f[16] = b.float(0, 5);
                const m = motor(b);
                Desc.setMotor(&h, 2, 17, @intFromFloat(m[0]), m[1..7].*);
                hinges[2 * k] = try b.constraint(h);
                if (is_rack[k]) {
                    // The rack next to the pinion (along the hinge axis, clear of it), sliding along the normal
                    const p2: [3]Real = .{ x + real(0.5 * axis[0]), y + real(0.5 * axis[1]), z + real(0.5 * axis[2]) };
                    bodies[2 * k + 1] = try b.body(.{ .position = p2, .rotation = q, .shape = rack_shape, .gravity_factor = if (b.oneIn(2)) 0 else 1, .object_layer = layer }, .activate);
                    var s = Desc.sliderAxes(invalid_handle, bodies[2 * k + 1], p2, normal, axis);
                    if (b.oneIn(2)) {
                        s.f[12] = -b.float(0.2, 1);
                        s.f[13] = b.float(0.2, 1);
                        if (b.oneIn(2)) Desc.setSpring(&s, 3, 14, b.index(3), b.float(1, 50), b.float(0, 1));
                    }
                    if (b.oneIn(3)) s.f[16] = b.float(0, 10);
                    const sm = motor(b);
                    Desc.setMotor(&s, 4, 17, @intFromFloat(sm[0]), sm[1..7].*);
                    sliders[k] = try b.constraint(s);
                    hinges[2 * k + 1] = sliders[k];
                    var rp = Desc.rackAndPinion(bodies[2 * k], bodies[2 * k + 1], axis, normal, ratio(b, true), hinges[2 * k], sliders[k]);
                    if (b.oneIn(2)) {
                        rp.i[1] = 1;
                        rp.i[2] = 4 + b.index(30);
                        rp.f[7] = b.float(0.5, 4);
                        rp.i[3] = 4 + b.index(30);
                    }
                    if (b.oneIn(4)) {
                        rp.constraint1 = invalid_handle;
                        rp.constraint2 = if (b.oneIn(2)) sliders[k] else invalid_handle;
                    }
                    couplings[k] = try b.constraint(rp);
                } else {
                    // The second gear next to the first one, its axis parallel or perpendicular (a bevel gear)
                    const bevel = b.oneIn(3);
                    const axis2 = if (bevel) normal else axis;
                    const normal2 = if (bevel) axis else normal;
                    const offset: f32 = 0.75;
                    const p2: [3]Real = .{ x + real(offset * normal[0]), y + real(offset * normal[1]), z + real(offset * normal[2]) };
                    const rotation2 = if (bevel) quatMul(q, axisAngle(.{ 0, 0, 1 }, 0.5 * std.math.pi)) else body_rotation;
                    bodies[2 * k + 1] = try b.body(.{ .position = p2, .rotation = rotation2, .shape = wheel, .object_layer = layer }, .activate);
                    var h2 = Desc.hinge(invalid_handle, bodies[2 * k + 1], p2, axis2, normal2);
                    if (b.oneIn(3)) h2.f[16] = b.float(0, 2);
                    hinges[2 * k + 1] = try b.constraint(h2);
                    var g = Desc.gear(bodies[2 * k], bodies[2 * k + 1], axis, axis2, ratio(b, false), hinges[2 * k], hinges[2 * k + 1]);
                    if (b.oneIn(2)) {
                        const n = teeth(b);
                        g.i[1] = 1;
                        g.i[2] = n[0];
                        g.i[3] = n[1];
                    }
                    if (b.oneIn(3)) {
                        // Local space: the axes in the space of the bodies (the shapes have their center of mass at the
                        // origin)
                        g.i[0] = ConstraintSpace.local_to_body_com;
                        Desc.set3(g.f[0..], .{ 0, 1, 0 });
                        Desc.set3(g.f[3..], .{ 0, 1, 0 });
                    }
                    if (b.oneIn(4)) {
                        g.constraint1 = if (b.oneIn(2)) hinges[2 * k] else invalid_handle;
                        g.constraint2 = invalid_handle;
                    }
                    couplings[k] = try b.constraint(g);
                }
            }
            for (0..6) |k| _ = try b.body(.{ .position = .{ 2 * real(k) - 5, 4 + real(k), 0.3 * real(k) - 1 }, .shape = crate, .object_layer = Layers.moving }, .activate);

            var update: u32 = 2;
            while (update < b.num_updates) : (update += 1 + b.index(5)) {
                b.at(update);
                for (0..1 + b.index(3)) |_| {
                    const k = b.index(num_pairs);
                    const c = couplings[k];
                    switch (b.index(12)) {
                        0, 1 => {
                            // SetConstraints: the own constraints or nullptr. Never a hinge of another pair: its body is
                            // in another island, GearConstraint::SolvePositionConstraint would read the rotation of a body
                            // that another job may be correcting at the same time (thread dependent, in Jolt too; see
                            // "Constraint scenes"). The draws of the former choice of another pair are kept so that the
                            // scenes of the committed seeds do not change otherwise.
                            const first = if (b.oneIn(4)) invalid_handle else hinges[2 * k];
                            const second = if (b.oneIn(4)) invalid_handle else hinges[2 * k + 1];
                            if (is_rack[k]) {
                                try b.constraintOp(.rack_and_pinion_set_constraints, c, .{ .i = &.{ first, second } });
                            } else {
                                const other = b.index(num_pairs);
                                if (!is_rack[other]) _ = b.oneIn(3);
                                try b.constraintOp(.gear_set_constraints, c, .{ .i = &.{ first, second } });
                            }
                        },
                        2, 3 => try b.constraintOp(.hinge_set_motor_state, hinges[2 * k], .{ .i = &.{b.index(4)} }),
                        4 => try b.constraintOp(.hinge_set_target_angular_velocity, hinges[2 * k], .{ .f = &.{b.float(-4, 4)} }),
                        5 => try b.constraintOp(.hinge_set_target_angle, hinges[2 * k], .{ .f = &.{b.float(-3, 3)} }),
                        6 => if (is_rack[k]) {
                            try b.constraintOp(.slider_set_motor_state, sliders[k], .{ .i = &.{b.index(4)} });
                            try b.constraintOp(.slider_set_target_velocity, sliders[k], .{ .f = &.{b.float(-1, 1)} });
                            try b.constraintOp(.slider_set_target_position, sliders[k], .{ .f = &.{b.float(-1, 1)} });
                        } else {
                            const m = motor(b);
                            try b.constraintOp(.hinge_set_motor_settings, hinges[2 * k], .{ .i = &.{@intFromFloat(m[0])}, .f = m[1..7] });
                        },
                        7 => {
                            try b.constraintOp(if (in_system[k]) .remove_constraint else .add_constraint, c, .{});
                            in_system[k] = !in_system[k];
                        },
                        8 => try b.constraintOp(.set_constraint_enabled, c, .{ .i = &.{b.index(2)} }),
                        9 => try b.constraintOp(.reset_constraint_warm_start, c, .{}),
                        10 => try b.constraintOp(.constraint_settings_hash, c, .{}),
                        else => try b.op(.add_angular_impulse, bodies[2 * k + b.index(2)], .{ .f = &b.vec3(-1, 1) }),
                    }
                }
            }
        }
    }.build;
}

/// `v` rotated by the unit quaternion `q` (x, y, z, w), computed in f64
fn quatRotate(q: [4]f32, v: [3]f32) [3]f32 {
    const u: [3]f64 = .{ q[0], q[1], q[2] };
    const s: f64 = q[3];
    const w: [3]f64 = .{ v[0], v[1], v[2] };
    const dot = u[0] * w[0] + u[1] * w[1] + u[2] * w[2];
    const uu = u[0] * u[0] + u[1] * u[1] + u[2] * u[2];
    const cross: [3]f64 = .{ u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0] };
    var r: [3]f32 = undefined;
    for (0..3) |k| r[k] = @floatCast(2 * dot * u[k] + (s * s - uu) * w[k] + 2 * s * cross[k]);
    return r;
}

/// The product `a b` of two unit quaternions (x, y, z, w), computed in f64 and normalized
fn quatMul(a: [4]f32, b: [4]f32) [4]f32 {
    const ax: f64 = a[0];
    const ay: f64 = a[1];
    const az: f64 = a[2];
    const aw: f64 = a[3];
    const bx: f64 = b[0];
    const by: f64 = b[1];
    const bz: f64 = b[2];
    const bw: f64 = b[3];
    return normalizedQuat(.{
        @floatCast(aw * bx + ax * bw + ay * bz - az * by),
        @floatCast(aw * by - ax * bz + ay * bw + az * bx),
        @floatCast(aw * bz + ax * by - ay * bx + az * bw),
        @floatCast(aw * bw - ax * bx - ay * by - az * bz),
    });
}

/// `v` times `s`
fn scale3(v: [3]f32, s: f32) [3]f32 {
    return .{ v[0] * s, v[1] * s, v[2] * s };
}

/// Jolt's release build only (Blocker.release_only: Jolt asserts that the gear and rack and pinion parts only see
/// dynamic bodies, its release build reads the motion properties of a kinematic body and changes its velocities): a
/// gear between a kinematic driver gear (MoveKinematic) and a dynamic gear, and a rack and pinion with a kinematic rack
fn buildGearKinematicRelease(b: *SceneBuilder) Allocator.Error!void {
    const wheel = try b.cylinder(0.1, 0.4, 0.05);
    const rack_shape = try b.box(.{ 1.0, 0.06, 0.06 }, 0.03);
    b.num_updates = 60;
    const driver = try b.body(.{ .position = .{ 0, 4, 0 }, .rotation = gear_lying, .shape = wheel, .motion_type = 1, .angular_velocity = .{ 0, 0, 2 }, .object_layer = Layers.unused1 }, .activate);
    const driven = try gearBody(b, wheel, .{ 0.85, 4, 0 }, .{});
    const dh1 = try b.constraint(gearHinge(driver, .{ 0, 4, 0 }));
    const dh2 = try b.constraint(gearHinge(driven, .{ 0.85, 4, 0 }));
    _ = try b.constraint(gearWithTeeth(driver, driven, dh1, dh2, 10, 20));
    const pinion_body = try gearBody(b, wheel, .{ 4, 4, 0 }, .{});
    const rack_body = try b.body(.{ .position = .{ 4, 3.5, 0 }, .shape = rack_shape, .motion_type = 1, .linear_velocity = .{ 0.4, 0, 0 }, .object_layer = Layers.unused1 }, .activate);
    const pinion = try b.constraint(gearHinge(pinion_body, .{ 4, 4, 0 }));
    const rack = try b.constraint(Desc.sliderAxes(invalid_handle, rack_body, .{ 4, 3.5, 0 }, .{ 1, 0, 0 }, .{ 0, 1, 0 }));
    _ = try b.constraint(rackAndPinionWithTeeth(pinion_body, rack_body, pinion, rack, 10, 2, 10));
    b.at(30);
    try b.op(.move_kinematic, driver, .{ .r = .{ 0, 4, 0 }, .f = &.{ 0, 0, 0, 1, 1.0 / 60.0 } });
}

// (append the scenes of the gear / rack and pinion group here)

// -------------------------------------------------------------------------------------------------------------------
// Characters, ragdolls and vehicles (Phase 7), one region per worker group (append a group's scenes at the end of its
// region only). They run on the C++ side alone until the classes are ported ("SimScenes blocked scenes run on the C++
// side"), see "Recipe for a Phase 7 worker" in the file header.

/// i[3] of the operations with `lock`: the no-lock interfaces (inLockBodies = false)
const no_lock = [_]u32{ 0, 0, 0, 1 };

/// The shapes of the character scenes: Jolt's CharacterTest capsules with the base at the origin (standing and
/// crouching), a sphere around the character that reaches into the floor (SetShape with a max penetration depth fails)
/// and the smaller capsule of an inner body
const CharacterShapes = struct {
    standing: u32,
    crouching: u32,
    big: u32,
    inner: u32,

    fn add(b: *SceneBuilder) Allocator.Error!CharacterShapes {
        return .{
            .standing = try b.rotatedTranslated(try b.capsule(0.6, 0.3), .{ 0, 0.9, 0 }, identity_rotation),
            .crouching = try b.rotatedTranslated(try b.capsule(0.3, 0.3), .{ 0, 0.6, 0 }, identity_rotation),
            .big = try b.rotatedTranslated(try b.sphere(1.0), .{ 0, 0.8, 0 }, identity_rotation),
            .inner = try b.rotatedTranslated(try b.capsule(0.55, 0.25), .{ 0, 0.85, 0 }, identity_rotation),
        };
    }
};

/// A static box (non_moving layer), returns its body handle
fn addStaticBox(b: *SceneBuilder, half_extent: [3]f32, position: [3]Real, rotation_value: [4]f32, user_data: u64) Allocator.Error!u32 {
    const shape = try b.box(half_extent, 0.05);
    return b.body(.{ .position = position, .rotation = rotation_value, .shape = shape, .motion_type = 0, .friction = 0.5, .object_layer = Layers.non_moving, .user_data = user_data }, .dont_activate);
}

/// The gravity of the character virtual operations
const character_gravity: [3]f32 = .{ 0, -9.81, 0 };

// ---- Character (Phase 7, Blocker.characters) ----

/// Every Character operation: two characters walk and one jumps on a floor with a crate (PostSimulation after every
/// update), change their velocity in every way, their shape (a switch that fits and one that collides), layer, position
/// and rotation, are removed and added again, one is destroyed after its removal and a third one with all degrees of
/// freedom is created; the up vector, the max slope angle and the supporting volume give the four ground states; the
/// queries (CheckCollision, GetCharacterSettings, the getters) with and without locking the bodies; two snapshots
/// restored cross (with the character states)
fn buildCharacterOps(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 90;
    _ = try addFloor(b, 20, 0);
    const crate_shape = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    _ = try b.body(.{ .position = .{ 1.6, 0.25, 0 }, .shape = crate_shape, .object_layer = Layers.moving }, .activate);

    const c0 = try b.character(.{ .base = .{ .shape = shapes.standing }, .layer = Layers.moving, .position = .{ 0, 0.02, 0 }, .user_data = 7 });
    const c1 = try b.character(.{ .base = .{ .shape = shapes.standing, .max_slope_angle = zolt.math.degreesToRadians(40.0) }, .layer = Layers.moving, .position = .{ 4, 0.02, 2 }, .friction = 0.5, .mass = 60, .gravity_factor = 1.2 });
    try b.objectOp(.character_add_to_physics_system, c0, .{});
    try b.objectOp(.character_add_to_physics_system, c1, .{ .activation = .dont_activate, .i = &no_lock });
    try b.expectCharacterGroundState(c0, GroundState.in_air);
    var c1_alive = true;
    var c2: ?u32 = null;

    for (1..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        // PostSimulation of the previous update
        try b.objectOp(.character_post_simulation, c0, .{ .f = &.{0.05} });
        if (c1_alive) try b.objectOp(.character_post_simulation, c1, .{ .f = &.{0.05}, .i = &no_lock });
        if (c2) |c| try b.objectOp(.character_post_simulation, c, .{ .f = &.{0.1} });
        switch (u) {
            1 => {
                try b.objectOp(.character_set_linear_velocity, c0, .{ .f = &.{ 2, 0, 0 } });
                try b.objectOp(.character_set_linear_and_angular_velocity, c1, .{ .f = &.{ -1, 0, 0, 0, 0, 0 }, .i = &no_lock });
            },
            4 => try b.expectCharacterGroundState(c0, GroundState.on_ground),
            6 => {
                try b.objectOp(.character_add_impulse, c0, .{ .f = &.{ 0, 120, 0 } });
                try b.objectOp(.character_add_linear_velocity, c1, .{ .f = &.{ -0.5, 0, 0.2 }, .i = &no_lock });
            },
            10 => {
                try b.expectCharacterGroundState(c0, GroundState.in_air);
                try b.objectOp(.character_check_collision, c0, .{ .i = &.{shapes.standing}, .r = .{ 0.5, 0.1, 0 }, .f = &.{ 0, 0, 0, 1, 1, 0, 0, 0.1 } });
                try b.objectOp(.character_check_collision, c1, .{ .i = &.{ shapes.crouching, 0, 0, 1 }, .r = .{ 3.5, 0.0, 2 }, .f = &.{ 0, 0, 0, 1, 0, -1, 0, 0.05 } });
            },
            12 => {
                try b.objectOp(.character_settings_hash, c0, .{});
                try b.objectOp(.character_settings_hash, c1, .{ .i = &no_lock });
            },
            14 => try b.objectOp(.character_set_layer, c1, .{ .i = &.{ Layers.moving2, 0, 0, 1 } }),
            16 => try b.objectOp(.character_set_shape, c1, .{ .i = &.{ shapes.crouching, 0, 0, 1 }, .f = &.{flt_max} }),
            18 => try b.objectOp(.character_set_position_and_rotation, c1, .{ .r = .{ 3, 0.05, 3 }, .f = &axisAngle(.{ 0, 1, 0 }, 0.4), .i = &no_lock }),
            20 => try b.objectOp(.character_set_position, c1, .{ .r = .{ 3.5, 0.05, 3 }, .activation = .dont_activate }),
            22 => {
                try b.objectOp(.character_set_rotation, c1, .{ .f = &axisAngle(.{ 0, 1, 0 }, -0.2), .i = &no_lock });
                try b.objectOp(.character_set_layer, c1, .{ .i = &.{Layers.moving} });
            },
            24 => try b.sysOp(.snapshot, .{ .index = 0 }),
            26 => {
                try b.objectOp(.character_body_hash, c0, .{});
                try b.objectOp(.character_body_hash, c1, .{ .i = &no_lock });
            },
            27 => try b.sysOp(.restore_cross, .{ .index = 0 }), // Back to update 24, the characters in the air
            // On the ground again: a too steep ground through the up vector and the max slope angle
            30 => {
                try b.objectOp(.character_set_up, c0, .{ .f = &.{ 0.8, 0.6, 0 } });
                try b.objectOp(.character_set_max_slope_angle, c0, .{ .f = &.{zolt.math.degreesToRadians(30.0)} });
            },
            32 => {
                try b.expectCharacterGroundState(c0, GroundState.on_steep_ground);
                try b.objectOp(.character_set_up, c0, .{ .f = &.{ 0, 1, 0 } });
                try b.objectOp(.character_set_max_slope_angle, c0, .{ .f = &.{zolt.math.degreesToRadians(50.0)} });
                // The contact point lies on the positive side of the supporting volume: not supported
                try b.objectOp(.character_set_supporting_volume, c0, .{ .f = &.{ 0, 1, 0, 0.5 } });
            },
            34 => {
                try b.expectCharacterGroundState(c0, GroundState.not_supported);
                try b.objectOp(.character_set_supporting_volume, c0, .{ .f = &.{ 0, 1, 0, -1.0e10 } });
            },
            36 => {
                try b.expectCharacterGroundState(c0, GroundState.on_ground);
                // A shape that reaches 0.2 into the floor: the switch fails, then one that fits
                try b.objectOp(.character_set_shape, c0, .{ .i = &.{shapes.big}, .f = &.{0.1} });
                try b.objectOp(.character_set_shape, c0, .{ .i = &.{shapes.crouching}, .f = &.{0.1} });
            },
            40 => try b.objectOp(.character_set_shape, c0, .{ .i = &.{shapes.standing}, .f = &.{flt_max} }),
            44 => try b.objectOp(.character_remove_from_physics_system, c1, .{ .i = &no_lock }),
            48 => try b.objectOp(.character_add_to_physics_system, c1, .{}),
            50 => {
                try b.objectOp(.character_set_linear_velocity, c0, .{ .f = &.{ -1, 0, 1 }, .i = &no_lock });
                try b.objectOp(.character_activate, c1, .{ .i = &no_lock });
            },
            56 => try b.objectOp(.character_remove_from_physics_system, c1, .{}),
            57 => {
                try b.objectOp(.character_destroy, c1, .{});
                c1_alive = false;
                // A third character with all degrees of freedom (it can tip over), enhanced internal edge removal and
                // its own gravity factor
                c2 = try b.character(.{ .base = .{ .shape = shapes.crouching, .enhanced_internal_edge_removal = 1 }, .layer = Layers.moving, .position = .{ -2, 0.3, -1 }, .rotation = axisAngle(.{ 0, 0, 1 }, 0.2), .allowed_dofs = 0b111111, .gravity_factor = 0.5, .friction = 0.8, .user_data = 0x1234 });
                try b.objectOp(.character_add_to_physics_system, c2.?, .{});
                try b.objectOp(.character_set_linear_and_angular_velocity, c2.?, .{ .f = &.{ 0.5, 0, 0, 0, 0, 1 } });
            },
            60 => {
                try b.objectOp(.character_settings_hash, c2.?, .{});
                try b.objectOp(.character_check_collision, c2.?, .{ .i = &.{ shapes.big, 0, 0, 1 }, .r = .{ -2, 0, -1 }, .f = &.{ 0, 0, 0, 1, 0, 0, 1, 0.2 } });
            },
            62 => try b.sysOp(.snapshot, .{ .index = 1 }),
            64 => try b.objectOp(.character_body_hash, c2.?, .{ .i = &no_lock }),
            66 => try b.sysOp(.restore_cross, .{ .index = 1 }), // The tipping character of all degrees of freedom
            70 => try b.objectOp(.character_add_impulse, c2.?, .{ .f = &.{ 0, 30, 0 }, .i = &no_lock }),
            80 => try b.objectOp(.character_set_rotation, c0, .{ .f = &axisAngle(.{ 0, 1, 0 }, 1.0), .activation = .dont_activate }),
            else => {},
        }
    }
}

/// CharacterTest's supporting volume, Plane(Vec3::sAxisY(), -cCharacterRadiusStanding): only contacts below the center of
/// the bottom sphere of the capsule support the character
const character_test_supporting_volume: [4]f32 = .{ 0, 1, 0, -0.3 };

/// PostSimulation (max separation distance `distance`) of the characters before the current update (the ground after the
/// previous update), every other call without locking the bodies
fn postSimulateCharacters(b: *SceneBuilder, characters: []const u32, distance: f32) Allocator.Error!void {
    for (characters, 0..) |c, n| {
        const lock = lockIf((n + b.current_update) % 2 != 0);
        try b.objectOp(.character_post_simulation, c, .{ .f = &.{distance}, .i = lock });
    }
}

/// i[3] of an operation with `lock`: the no-lock interfaces when `no_lock_bodies`
fn lockIf(no_lock_bodies: bool) []const u32 {
    return if (no_lock_bodies) &no_lock else &.{};
}

/// A static box tilted by `degrees` around Z (its top face rises towards +x) with the center of its top face at
/// `face_center` (`offset` added); returns the body handle and the position of a character whose capsule hangs 0.01 above
/// the center of the face
const Slope = struct {
    body: u32,
    start: [3]Real,

    fn add(b: *SceneBuilder, half_extent: [3]f32, face_center: [3]f64, degrees: f64, offset: [3]Real, user_data: u64) Allocator.Error!Slope {
        const angle = degrees * std.math.pi / 180.0;
        const n = [3]f64{ -@sin(angle), @cos(angle), 0 };
        const h: f64 = half_extent[1];
        const center = [3]Real{ offset[0] + real(face_center[0] - h * n[0]), offset[1] + real(face_center[1] - h * n[1]), offset[2] + real(face_center[2]) };
        const body_handle = try addStaticBox(b, half_extent, center, axisAngle(.{ 0, 0, 1 }, angle), user_data);
        return .{ .body = body_handle, .start = .{ offset[0] + real(face_center[0] + 0.31 * n[0]), offset[1] + real(face_center[1] + 0.31 * n[1] - 0.3), offset[2] + real(face_center[2]) } };
    }
};

/// Characters on boxes and slopes reach the four ground states with CharacterTest's supporting volume: one walks across the
/// floor and up a 30 degree ramp (OnGround on the ramp) and jumps (InAir); one walks off a raised platform (the edge, then
/// InAir); one slides down a 60 degree slope (OnSteepGround) onto the floor; one jumps under a low ceiling (NotSupported:
/// only the ceiling is in reach) and one along a wall that leans over it (NotSupported: the wall touches the top of the
/// capsule). The walkers set their velocity every update like CharacterTest (the vertical component is not kept); the
/// queries with and without locking; a snapshot in the middle of the jumps restored cross
fn buildCharacterGroundStatesBoxes(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 150;
    _ = try addFloor(b, 30, 0);
    const zero: [3]Real = @splat(0);
    _ = try Slope.add(b, .{ 2.5, 0.25, 1.5 }, .{ 5.165, 1.25, 0 }, 30, zero, 1); // From x = 3 (on the floor) up to y = 2.5
    const slope = try Slope.add(b, .{ 2.5, 0.25, 1.5 }, .{ -6, 2.5, 4 }, 60, zero, 2);
    _ = try addStaticBox(b, .{ 2, 0.5, 2 }, .{ -6, 0.5, -6 }, identity_rotation, 3); // A platform, top at y = 1
    _ = try addStaticBox(b, .{ 1.5, 0.25, 1.5 }, .{ 0, 2.55, -5 }, identity_rotation, 4); // A ceiling, bottom at y = 2.3
    _ = try addStaticBox(b, .{ 0.25, 2, 1.5 }, .{ 8, 2, -5 }, axisAngle(.{ 0, 0, 1 }, 10.0 * std.math.pi / 180.0), 5); // Leans over x < 8

    const base: SimCharacterBase = .{ .shape = shapes.standing, .supporting_volume = character_test_supporting_volume };
    const walker = try b.character(.{ .base = base, .layer = Layers.moving, .position = .{ 1.5, 0.01, 0 }, .user_data = 1 });
    const edge_walker = try b.character(.{ .base = base, .layer = Layers.moving, .position = .{ -5, 1.01, -6 }, .mass = 60 });
    const slider = try b.character(.{ .base = base, .layer = Layers.moving, .position = slope.start, .friction = 0.1 });
    const jumper = try b.character(.{ .base = base, .layer = Layers.moving, .position = .{ 0, 0.01, -5 }, .gravity_factor = 0.8 });
    const wall_jumper = try b.character(.{ .base = base, .layer = Layers.moving, .position = .{ 7.509, 0.01, -5 } });
    const all = [_]u32{ walker, edge_walker, slider, jumper, wall_jumper };
    for (all, 0..) |c, n|
        try b.objectOp(.character_add_to_physics_system, c, .{ .i = lockIf(n % 2 != 0) });

    for (1..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        try postSimulateCharacters(b, &all, if (u < 100) 0.05 else 0.1);
        if (u <= 70) try b.objectOp(.character_set_linear_velocity, walker, .{ .f = &.{ 2.5, 0, 0 }, .i = lockIf(u % 3 == 0) });
        if (u <= 35) try b.objectOp(.character_set_linear_velocity, edge_walker, .{ .f = &.{ 2, 0, 0 } });
        switch (u) {
            5 => {
                try b.expectCharacterGroundState(walker, GroundState.on_ground);
                try b.expectCharacterGroundState(slider, GroundState.on_steep_ground);
                try b.expectCharacterGroundState(jumper, GroundState.on_ground);
                try b.expectCharacterGroundState(wall_jumper, GroundState.on_ground); // The floor is more up than the wall
            },
            10, 70 => {
                try b.objectOp(.character_add_linear_velocity, jumper, .{ .f = &.{ 0, 4, 0 } });
                try b.objectOp(.character_add_impulse, wall_jumper, .{ .f = &.{ 0, 320, 0 }, .i = &no_lock });
            },
            30 => {
                try b.objectOp(.character_check_collision, walker, .{ .i = &.{shapes.standing}, .r = .{ 3.2, 0.3, 0 }, .f = &.{ 0, 0, 0, 1, 1, 0, 0, 0.05 } });
                try b.objectOp(.character_check_collision, wall_jumper, .{ .i = &.{ shapes.big, 0, 0, 1 }, .r = .{ 7.4, 0.5, -5 }, .f = &.{ 0, 0, 0, 1, 0, 1, 0, 0.1 } });
                try b.objectOp(.character_body_hash, slider, .{ .i = &no_lock });
            },
            45 => try b.sysOp(.snapshot, .{ .index = 0 }),
            50 => try b.objectOp(.character_settings_hash, edge_walker, .{}),
            55 => try b.sysOp(.restore_cross, .{ .index = 0 }), // Back to update 45 (jumps, slides, the walker on the ramp)
            76 => try b.objectOp(.character_add_linear_velocity, walker, .{ .f = &.{ -1, 4, 0 } }), // Jumps off the ramp
            100 => try b.objectOp(.character_body_hash, walker, .{}),
            else => {},
        }
    }
}

/// An indexed mesh with a flat floor (y = 0 over x in [10, 22], z in [-6, 6]), a 65 degree ramp (rising from x = 16 to 18,
/// z in [-2, 2]) and a ceiling over z in [3, 6] (facing down, at y = 2.3)
fn addMeshCourse(b: *SceneBuilder) Allocator.Error!u32 {
    const h: f32 = 2.0 * 2.1445069; // 2 * tan(65 degrees)
    const vertices = [_][3]f32{
        .{ 10, 0, -6 }, .{ 22, 0, -6 }, .{ 22, 0, 6 }, .{ 10, 0, 6 }, // Floor 0..3
        .{ 16, 0, -2 }, .{ 16, 0, 2 }, .{ 18, h, 2 }, .{ 18, h, -2 }, // Ramp 4..7
        .{ 10, 2.3, 3 }, .{ 14, 2.3, 3 }, .{ 14, 2.3, 6 }, .{ 10, 2.3, 6 }, // Ceiling 8..11
    };
    const triangles = [_][3]u32{
        .{ 0, 3, 2 }, .{ 0, 2, 1 }, // Floor, facing up
        .{ 4, 5, 6 }, .{ 4, 6, 7 }, // Ramp, facing up / -x
        .{ 8, 9, 11 }, .{ 9, 10, 11 }, // Ceiling, facing down
    };
    return b.indexedMesh(&vertices, &triangles, 4);
}

/// A height field of 9 x 9 samples, 1 m apart, with low bumps and a plateau of 2.5 m from x = 5 (a cliff of 68 degrees)
fn addHeightFieldCliff(b: *SceneBuilder) Allocator.Error!u32 {
    var samples: [81]f32 = undefined;
    for (0..9) |iz| {
        for (0..9) |ix| {
            const x: f32 = @floatFromInt(ix);
            const z: f32 = @floatFromInt(iz);
            samples[iz * 9 + ix] = if (ix >= 5) 2.5 else 0.1 * terrainHeight(x, z);
        }
    }
    return b.heightField(&samples, 9, .{ 0, 0, 0 }, .{ 1, 1, 1 });
}

/// Characters on meshes and a height field reach the four ground states: walkers cross the hills of a mesh terrain (one
/// with enhanced internal edge removal), one slides down a 65 degree mesh ramp (OnSteepGround), one jumps under a mesh
/// ceiling (NotSupported), one walks off the cliff of a height field and one slides down the cliff (OnSteepGround, then
/// InAir and OnGround)
fn buildCharacterGroundStatesMeshes(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 150;
    b.max_contact_constraints = 2048;
    const terrain = try addMeshTerrain(b, 12, 1.5, false);
    _ = try b.body(.{ .shape = terrain, .motion_type = 0, .friction = 0.6, .object_layer = Layers.non_moving, .user_data = 10 }, .dont_activate);
    const course = try addMeshCourse(b);
    _ = try b.body(.{ .shape = course, .motion_type = 0, .friction = 0.4, .object_layer = Layers.non_moving, .user_data = 11 }, .dont_activate);
    const cliff = try addHeightFieldCliff(b);
    _ = try b.body(.{ .position = .{ -24, 0, -4 }, .shape = cliff, .motion_type = 0, .friction = 0.5, .object_layer = Layers.non_moving, .user_data = 12 }, .dont_activate);

    const base: SimCharacterBase = .{ .shape = shapes.standing, .supporting_volume = character_test_supporting_volume };
    const hill_walker = try b.character(.{ .base = base, .layer = Layers.moving, .position = .{ -6, 1.0, -2 } });
    const edge_walker = try b.character(.{ .base = .{ .shape = shapes.standing, .supporting_volume = character_test_supporting_volume, .enhanced_internal_edge_removal = 1 }, .layer = Layers.moving, .position = .{ -6, 1.0, 3 }, .friction = 0.5 });
    // On the ramp: the face center (17, h / 2, 0) and its normal (-h, 2, 0) / |(-h, 2, 0)|
    const h: f64 = 2.0 * 2.1445069;
    const len = @sqrt(h * h + 4.0);
    const ramp_slider = try b.character(.{ .base = base, .layer = Layers.moving, .position = .{ real(17.0 - 0.31 * h / len), real(0.5 * h + 0.31 * 2.0 / len - 0.3), 0 } });
    const ceiling_jumper = try b.character(.{ .base = base, .layer = Layers.moving, .position = .{ 12, 0.01, 4.5 } });
    const cliff_walker = try b.character(.{ .base = base, .layer = Layers.moving, .position = .{ -17.5, 2.51, 0 } });
    // On the cliff: the cell from x = 4 to 5 (height 0.1 * bumps to 2.5), its face around (4.5, 1.25)
    const cliff_normal = [2]f64{ -2.5 / @sqrt(2.5 * 2.5 + 1.0), 1.0 / @sqrt(2.5 * 2.5 + 1.0) };
    const cliff_slider = try b.character(.{ .base = base, .layer = Layers.moving, .position = .{ real(-24.0 + 4.5 + 0.35 * cliff_normal[0]), real(1.3 + 0.35 * cliff_normal[1] - 0.3), -1.5 } });
    const all = [_]u32{ hill_walker, edge_walker, ramp_slider, ceiling_jumper, cliff_walker, cliff_slider };
    for (all) |c|
        try b.objectOp(.character_add_to_physics_system, c, .{});

    for (1..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        try postSimulateCharacters(b, &all, 0.05);
        if (u >= 20 and u < 120) {
            try b.objectOp(.character_set_linear_velocity, hill_walker, .{ .f = &.{ 2, 0, 0.5 } });
            try b.objectOp(.character_set_linear_velocity, edge_walker, .{ .f = &.{ 2, 0, -0.5 }, .i = &no_lock });
        }
        if (u >= 10 and u < 60) try b.objectOp(.character_set_linear_velocity, cliff_walker, .{ .f = &.{ -2, 0, 0 } });
        switch (u) {
            15, 80 => try b.objectOp(.character_add_linear_velocity, ceiling_jumper, .{ .f = &.{ 0, 4.5, 0 } }),
            60 => {
                try b.objectOp(.character_check_collision, hill_walker, .{ .i = &.{shapes.crouching}, .r = .{ 0, 0.2, 0 }, .f = &.{ 0, 0, 0, 1, 1, 0, 0, 0.1 } });
                try b.objectOp(.character_check_collision, cliff_walker, .{ .i = &.{ shapes.standing, 0, 0, 1 }, .r = .{ -19.5, 0.5, 0 }, .f = &.{ 0, 0, 0, 1, -1, 0, 0, 0.1 } });
            },
            100 => try b.objectOp(.character_add_impulse, hill_walker, .{ .f = &.{ 0, 300, 0 } }),
            else => {},
        }
    }
}

/// Characters on moving bodies: a kinematic platform that moves back and forth (MoveKinematic every update), a kinematic
/// disc that rotates (the ground velocity of a point away from the axis), a dynamic raft that slides over the floor; a
/// walker pushes a row of crates; the platform rider jumps and lands again; a snapshot on the moving platforms restored cross
fn buildCharacterMovingPlatforms(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 150;
    _ = try addFloor(b, 30, 0);
    const platform_shape = try b.box(.{ 1.5, 0.2, 1.5 }, 0.05);
    const platform = try b.body(.{ .position = .{ 0, 1, 0 }, .shape = platform_shape, .motion_type = 1, .friction = 1.0, .object_layer = Layers.moving, .user_data = 21 }, .activate);
    const disc_shape = try b.cylinder(0.2, 2.0, 0.05);
    _ = try b.body(.{ .position = .{ 7, 0.5, 0 }, .shape = disc_shape, .motion_type = 1, .angular_velocity = .{ 0, 0.8, 0 }, .friction = 1.0, .object_layer = Layers.moving, .user_data = 22 }, .activate);
    const raft_shape = try b.box(.{ 1.5, 0.25, 1.5 }, 0.05);
    const raft = try b.body(.{ .position = .{ -7, 0.25, 0 }, .shape = raft_shape, .friction = 0.8, .object_layer = Layers.moving, .user_data = 23 }, .activate);
    const crate_shape = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    for (0..4) |i|
        _ = try b.body(.{ .position = .{ 2 + real(i) * 0.8, 0.3, 6 }, .shape = crate_shape, .object_layer = Layers.moving, .user_data = 30 + i }, .activate);

    const rider = try b.character(.{ .base = .{ .shape = shapes.standing }, .layer = Layers.moving, .position = .{ 0, 1.21, 0 }, .user_data = 1 });
    const spinner = try b.character(.{ .base = .{ .shape = shapes.standing }, .layer = Layers.moving, .position = .{ 8.2, 0.71, 0 }, .user_data = 2 });
    const rafter = try b.character(.{ .base = .{ .shape = shapes.crouching }, .layer = Layers.moving, .position = .{ -7, 0.51, 0.5 }, .user_data = 3 });
    const pusher = try b.character(.{ .base = .{ .shape = shapes.standing }, .layer = Layers.moving, .position = .{ 0, 0.01, 6 }, .mass = 90, .user_data = 4 });
    const all = [_]u32{ rider, spinner, rafter, pusher };
    for (all) |c|
        try b.objectOp(.character_add_to_physics_system, c, .{});

    for (1..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        try postSimulateCharacters(b, &all, 0.05);
        // The platform moves along x from 0 to 3 and back in 4 s (smoothly, from rest)
        const t = @as(f64, @floatFromInt(u)) / 60.0;
        try b.op(.move_kinematic, platform, .{ .r = .{ real(1.5 * (1.0 - @cos(t * std.math.pi / 2.0))), 1, 0 }, .f = &.{ 0, 0, 0, 1, 1.0 / 60.0 } });
        if (u >= 10 and u < 110) try b.objectOp(.character_set_linear_velocity, pusher, .{ .f = &.{ 1.5, 0, 0 }, .i = lockIf(u % 2 == 0) });
        switch (u) {
            20 => try b.op(.set_linear_velocity, raft, .{ .f = &.{ 3, 0, 0.5 } }),
            40 => try b.sysOp(.snapshot, .{ .index = 0 }),
            45 => try b.objectOp(.character_body_hash, rider, .{}),
            50 => try b.sysOp(.restore_cross, .{ .index = 0 }), // Back to update 40: the rider and the spinner moving with their ground
            70 => {
                try b.expectCharacterGroundState(rider, GroundState.on_ground);
                try b.expectCharacterGroundState(spinner, GroundState.on_ground);
            },
            90 => try b.objectOp(.character_add_linear_velocity, rider, .{ .f = &.{ 0, 3, 0 } }),
            100 => try b.op(.set_linear_velocity, raft, .{ .f = &.{ -2, 0, 0 } }),
            else => {},
        }
    }
}

/// SetShape (crouch / stand) with and without max penetration depth, with and without locking: a character crouches under
/// a ceiling (bottom at y = 1.6) and is not allowed to stand up (the standing capsule reaches 1.8 m), then is allowed with a
/// larger max penetration depth (squeezed between the floor and the ceiling until it crouches again); another one walks
/// out from under the ceiling while it tries to stand up every 5 updates (rejected below the ceiling, accepted outside); a
/// third one switches shapes in the open, to a sphere that reaches into the floor (rejected) and then forced (FLT_MAX: no
/// check, the solver pushes it out)
fn buildCharacterCrouchStand(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 120;
    _ = try addFloor(b, 20, 0);
    _ = try addStaticBox(b, .{ 2, 0.5, 2 }, .{ -2, 2.1, 0 }, identity_rotation, 1);

    const squeezed = try b.character(.{ .base = .{ .shape = shapes.crouching }, .layer = Layers.moving, .position = .{ -2, 0.01, -1 } });
    const leaver = try b.character(.{ .base = .{ .shape = shapes.crouching }, .layer = Layers.moving, .position = .{ -2, 0.01, 1 } });
    const switcher = try b.character(.{ .base = .{ .shape = shapes.standing }, .layer = Layers.moving, .position = .{ 4, 0.01, 0 } });
    const all = [_]u32{ squeezed, leaver, switcher };
    for (all) |c|
        try b.objectOp(.character_add_to_physics_system, c, .{});

    for (1..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        try postSimulateCharacters(b, &all, 0.05);
        if (u >= 10 and u < 70) try b.objectOp(.character_set_linear_velocity, leaver, .{ .f = &.{ 2.5, 0, 0 } });
        if (u >= 10 and u % 5 == 0 and u < 100) try b.objectOp(.character_set_shape, leaver, .{ .i = &.{ shapes.standing, 0, 0, u % 2 }, .f = &.{0.05} });
        switch (u) {
            5 => try b.objectOp(.character_set_shape, squeezed, .{ .i = &.{shapes.standing}, .f = &.{0.1} }), // Rejected: 0.2 m into the ceiling
            8 => try b.objectOp(.character_set_shape, squeezed, .{ .i = &.{ shapes.standing, 0, 0, 1 }, .f = &.{0.0} }), // Rejected
            12 => try b.objectOp(.character_set_shape, squeezed, .{ .i = &.{ shapes.standing, 0, 0, 1 }, .f = &.{0.3} }), // Allowed
            30 => try b.objectOp(.character_set_shape, squeezed, .{ .i = &.{shapes.crouching}, .f = &.{flt_max} }),
            32 => try b.objectOp(.character_check_collision, squeezed, .{ .i = &.{shapes.standing}, .r = .{ -2, 0, -1 }, .f = &.{ 0, 0, 0, 1, 0, 1, 0, 0.0 } }),
            15 => try b.objectOp(.character_set_shape, switcher, .{ .i = &.{shapes.crouching}, .f = &.{0.05} }), // Allowed
            20 => try b.objectOp(.character_set_shape, switcher, .{ .i = &.{ shapes.big, 0, 0, 1 }, .f = &.{0.1} }), // Rejected: 0.2 m into the floor
            25 => try b.objectOp(.character_set_shape, switcher, .{ .i = &.{shapes.big}, .f = &.{flt_max} }), // Forced
            40 => try b.objectOp(.character_set_shape, switcher, .{ .i = &.{ shapes.standing, 0, 0, 1 }, .f = &.{0.05} }),
            41 => try b.objectOp(.character_settings_hash, switcher, .{ .i = &no_lock }),
            60 => try b.sysOp(.snapshot, .{ .index = 0 }),
            65 => try b.objectOp(.character_set_shape, switcher, .{ .i = &.{shapes.crouching}, .f = &.{flt_max} }),
            68 => try b.sysOp(.restore_cross, .{ .index = 0 }), // The shapes of the characters are not part of the state: the switcher stays crouched
            else => {},
        }
    }
}

/// Layer changes, sleeping and waking: a walker pushes crates, switches to a layer that does not collide with them
/// (MOVING2) and walks through, then back; another one switches to a layer that collides with nothing and falls through
/// the floor (InAir), then is put back; a third one stands still until it falls asleep, operations that do not activate
/// keep it asleep (SetPosition / SetRotation / SetPositionAndRotation with DontActivate, SetLayer, PostSimulation), then
/// Activate, AddImpulse and SetLinearVelocity wake it up; a snapshot of the sleeping character restored cross
fn buildCharacterLayersSleepWake(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 260;
    _ = try addFloor(b, 20, 0);
    const crate_shape = try b.box(.{ 0.4, 0.4, 0.4 }, 0.05);
    for (0..3) |i|
        _ = try b.body(.{ .position = .{ 2, 0.4, -1 + real(i) }, .shape = crate_shape, .object_layer = Layers.moving, .user_data = 40 + i }, .activate);

    const walker = try b.character(.{ .base = .{ .shape = shapes.standing }, .layer = Layers.moving, .position = .{ 0.5, 0.01, 0 } });
    const diver = try b.character(.{ .base = .{ .shape = shapes.standing }, .layer = Layers.moving, .position = .{ -4, 0.01, 4 } });
    const sleeper = try b.character(.{ .base = .{ .shape = shapes.standing }, .layer = Layers.moving, .position = .{ -4, 0.01, -4 }, .user_data = 77 });
    const all = [_]u32{ walker, diver, sleeper };
    for (all) |c|
        try b.objectOp(.character_add_to_physics_system, c, .{});

    for (1..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        try postSimulateCharacters(b, &all, 0.05);
        if (u < 100) try b.objectOp(.character_set_linear_velocity, walker, .{ .f = &.{ 1.2, 0, 0 } });
        switch (u) {
            55 => try b.objectOp(.character_set_layer, walker, .{ .i = &.{Layers.moving2} }), // Walks through the crates
            85 => try b.objectOp(.character_set_layer, walker, .{ .i = &.{ Layers.moving, 0, 0, 1 } }), // Inside a crate
            20 => try b.objectOp(.character_set_layer, diver, .{ .i = &.{ Layers.unused1, 0, 0, 1 } }), // Falls through the floor
            40 => {
                try b.expectCharacterGroundState(diver, GroundState.in_air);
                try b.objectOp(.character_set_position, diver, .{ .r = .{ -4, 0.5, 4 } });
                try b.objectOp(.character_set_linear_velocity, diver, .{ .f = &.{ 0, 0, 0 } });
                try b.objectOp(.character_set_layer, diver, .{ .i = &.{Layers.moving} });
            },
            90 => {
                try b.expectCharacterBodyState(sleeper, BodyState.sleeping);
                try b.expectCharacterGroundState(sleeper, GroundState.on_ground);
                try b.objectOp(.character_set_position, sleeper, .{ .r = .{ -4, 0.02, -3.5 }, .activation = .dont_activate });
                try b.objectOp(.character_set_rotation, sleeper, .{ .f = &axisAngle(.{ 0, 1, 0 }, 0.5), .activation = .dont_activate, .i = &no_lock });
                try b.objectOp(.character_set_position_and_rotation, sleeper, .{ .r = .{ -4, 0.01, -3.6 }, .f = &axisAngle(.{ 0, 1, 0 }, 0.6), .activation = .dont_activate });
                try b.objectOp(.character_set_layer, sleeper, .{ .i = &.{Layers.moving2} });
                try b.expectCharacterBodyState(sleeper, BodyState.sleeping);
            },
            92 => try b.sysOp(.snapshot, .{ .index = 0 }),
            95 => try b.objectOp(.character_activate, sleeper, .{ .i = &no_lock }),
            96 => try b.expectCharacterBodyState(sleeper, BodyState.awake),
            98 => try b.sysOp(.restore_cross, .{ .index = 0 }), // Asleep again
            99 => try b.expectCharacterBodyState(sleeper, BodyState.sleeping),
            100 => try b.objectOp(.character_activate, sleeper, .{}),
            180 => {
                try b.expectCharacterBodyState(sleeper, BodyState.sleeping);
                try b.objectOp(.character_add_impulse, sleeper, .{ .f = &.{ 20, 0, 0 } });
                try b.expectCharacterBodyState(sleeper, BodyState.awake);
            },
            250 => {
                try b.expectCharacterBodyState(sleeper, BodyState.sleeping);
                try b.objectOp(.character_set_linear_velocity, sleeper, .{ .f = &.{ 0, 0, 1 }, .i = &no_lock });
                try b.expectCharacterBodyState(sleeper, BodyState.awake);
            },
            else => {},
        }
    }
}

/// The character scenes far from the origin (5 km in single precision, 100 km in double precision): characters walk on a
/// floor and a ramp, stand on a moving kinematic platform, jump, switch their shape; a snapshot restored cross
fn buildCharacterFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 120;
    const o: Real = if (zolt.Core.double_precision) 100000.0 else 5000.0;
    const offset: [3]Real = .{ o, 0, -o };
    const floor = try b.box(.{ 20, 1, 20 }, 0);
    _ = try b.body(.{ .position = .{ o, -1, -o }, .shape = floor, .motion_type = 0, .friction = 0.5, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try Slope.add(b, .{ 2.5, 0.25, 1.5 }, .{ 5.165, 1.25, 0 }, 30, offset, 1);
    const steep = try Slope.add(b, .{ 2.5, 0.25, 1.5 }, .{ -6, 2.5, 4 }, 60, offset, 2);
    const platform_shape = try b.box(.{ 1.5, 0.2, 1.5 }, 0.05);
    const platform = try b.body(.{ .position = .{ o, 1, -o - 6 }, .shape = platform_shape, .motion_type = 1, .friction = 1.0, .object_layer = Layers.moving }, .activate);

    const walker = try b.character(.{ .base = .{ .shape = shapes.standing }, .layer = Layers.moving, .position = .{ o + 1.5, 0.01, -o } });
    const slider = try b.character(.{ .base = .{ .shape = shapes.standing }, .layer = Layers.moving, .position = steep.start });
    const rider = try b.character(.{ .base = .{ .shape = shapes.crouching }, .layer = Layers.moving, .position = .{ o, 1.21, -o - 6 } });
    const all = [_]u32{ walker, slider, rider };
    for (all) |c|
        try b.objectOp(.character_add_to_physics_system, c, .{});

    for (1..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        try postSimulateCharacters(b, &all, 0.05);
        const t = @as(f64, @floatFromInt(u)) / 60.0;
        try b.op(.move_kinematic, platform, .{ .r = .{ o + real(1.5 * (1.0 - @cos(t * std.math.pi / 2.0))), 1, -o - 6 }, .f = &.{ 0, 0, 0, 1, 1.0 / 60.0 } });
        if (u <= 60) try b.objectOp(.character_set_linear_velocity, walker, .{ .f = &.{ 2.5, 0, 0 }, .i = lockIf(u % 2 == 0) });
        switch (u) {
            30 => {
                try b.objectOp(.character_check_collision, walker, .{ .i = &.{shapes.standing}, .r = .{ o + 3.2, 0.3, -o }, .f = &.{ 0, 0, 0, 1, 1, 0, 0, 0.05 } });
                try b.objectOp(.character_body_hash, rider, .{ .i = &no_lock });
                try b.objectOp(.character_set_shape, rider, .{ .i = &.{shapes.standing}, .f = &.{0.05} });
            },
            40 => try b.sysOp(.snapshot, .{ .index = 0 }),
            50 => try b.sysOp(.restore_cross, .{ .index = 0 }),
            70 => try b.objectOp(.character_add_linear_velocity, walker, .{ .f = &.{ -1, 4, 0 } }),
            80 => try b.objectOp(.character_settings_hash, slider, .{}),
            else => {},
        }
    }
}

/// Many characters on a mesh terrain with crates: 40 characters walk in random directions (new ones every 20 updates),
/// jump at random, push the crates and each other; they cover all lanes of the thread matrix (large islands of characters
/// and crates)
fn buildCharacterMany(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 120;
    b.max_contact_constraints = 4096;
    b.max_body_pairs = 8192;
    b.rng = 0x5eed;
    const terrain = try addMeshTerrain(b, 16, 1.5, false);
    _ = try b.body(.{ .shape = terrain, .motion_type = 0, .friction = 0.6, .object_layer = Layers.non_moving }, .dont_activate);
    const crate_shape = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    for (0..12) |i|
        _ = try b.body(.{ .position = .{ real(b.float(-8, 8)), 1.2, real(b.float(-8, 8)) }, .rotation = b.rotation(), .shape = crate_shape, .object_layer = Layers.moving, .user_data = 50 + i }, .activate);

    var characters: [40]u32 = undefined;
    for (&characters, 0..) |*c, i| {
        const x: Real = real(@as(f64, @floatFromInt(i % 8)) * 2.2 - 7.7);
        const z: Real = real(@as(f64, @floatFromInt(i / 8)) * 2.6 - 5.2);
        c.* = try b.character(.{
            .base = .{ .shape = if (i % 5 == 0) shapes.crouching else shapes.standing, .max_slope_angle = zolt.math.degreesToRadians(b.float(30, 60)) },
            .layer = Layers.moving,
            .position = .{ x, 1.0, z },
            .mass = b.float(50, 100),
            .friction = b.float(0.1, 0.8),
            .user_data = i,
        });
        try b.objectOp(.character_add_to_physics_system, c.*, .{ .i = lockIf(i % 3 == 0) });
    }

    var velocities: [40][3]f32 = @splat(.{ 0, 0, 0 });
    for (1..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        try postSimulateCharacters(b, &characters, 0.05);
        if (u % 20 == 1) {
            for (&velocities) |*v| v.* = .{ b.float(-2, 2), 0, b.float(-2, 2) };
        }
        for (characters, velocities, 0..) |c, v, i| {
            if (u >= 10 and (u + i) % 2 == 0)
                try b.objectOp(.character_set_linear_velocity, c, .{ .f = &v, .i = lockIf(i % 4 == 0) });
            if (b.oneIn(150))
                try b.objectOp(.character_add_linear_velocity, c, .{ .f = &.{ 0, 4, 0 } });
        }
        if (u == 60) try b.sysOp(.snapshot, .{ .index = 0 });
        if (u == 66) try b.sysOp(.restore_cross, .{ .index = 0 });
    }
}

// (append the Character scenes here)

// ---- CharacterVirtual (Phase 7, Blocker.character_virtuals) ----

/// The query of the character virtual scenes: the default filters of the moving layer
const cv_query: SceneBuilder.CharacterQuery = .{ .layer = Layers.moving };

/// Set the velocity of a character virtual for the next update (the scene does not integrate the vertical velocity),
/// then Update or ExtendedUpdate (Jolt's default settings)
fn moveCharacterVirtual(b: *SceneBuilder, handle: u32, velocity: [3]f32, extended: bool, query: SceneBuilder.CharacterQuery) Allocator.Error!void {
    try b.objectOp(.cv_set_linear_velocity, handle, .{ .f = &velocity });
    if (extended)
        try b.extendedUpdate(handle, b.delta_time, character_gravity, .{}, query)
    else
        try b.characterVirtualQuery(.cv_update, handle, query, .{ .f = &([_]f32{b.delta_time} ++ character_gravity) });
}

/// Every CharacterVirtual operation in three lanes of a floor that is a conveyor belt for the listener in modify mode:
/// character 0 runs into a post (that the modify mode rejects), climbs stairs (ExtendedUpdate, WalkStairs,
/// StickToFloor) and walks off them; character 1 falls onto the floor and pushes a crate (whose flags stop the pushing in
/// modify mode); character 2 runs into a steep slope and a wall through a sensor. The listener modes (none, record,
/// modify: the rejected post, the conveyor, the slowed solves), the contact tracking, the shape switches (one
/// collides), every setter, SaveState / RestoreState (validating too), the queries, the collision groups, a snapshot
/// restored cross (with the character states).
fn buildCharacterVirtualOps(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 120;
    const floor = try addFloor(b, 20, 0);
    try b.op(.set_user_data, floor, .{ .index = 0, .count = CharacterFlags.conveyor }); // A conveyor belt in modify mode
    // Lane 0 (z = 0): a post that the modify mode rejects, stairs of three steps of 0.2 m from x = 1
    _ = try addStaticBox(b, .{ 0.1, 0.5, 0.1 }, .{ 0.6, 0.5, 0 }, identity_rotation, CharacterFlags.userData(CharacterFlags.reject, 2));
    for (0..3) |s| {
        const fs: f32 = @floatFromInt(s);
        _ = try addStaticBox(b, .{ 0.3, 0.1 + 0.1 * fs, 1.0 }, .{ 1.3 + 0.6 * fs, 0.1 + 0.1 * fs, 0 }, identity_rotation, 0);
    }
    // Lane 1 (z = -2.5): a crate, pushing it stops in modify mode
    const crate_shape = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    const crate = try b.body(.{ .position = .{ 1.0, 0.3, -2.5 }, .shape = crate_shape, .object_layer = Layers.moving, .user_data = CharacterFlags.userData(CharacterFlags.cannot_push | CharacterFlags.no_impulses, 1) }, .activate);
    // Lane 2 (z = 2.5): a sensor, a 60 degree slope, a wall
    const sensor_shape = try b.box(.{ 0.2, 0.5, 0.5 }, 0.05);
    const sensor = try b.body(.{ .position = .{ 0.8, 0.5, 2.5 }, .shape = sensor_shape, .motion_type = 0, .is_sensor = 1, .object_layer = Layers.sensor }, .dont_activate);
    _ = try addStaticBox(b, .{ 1.0, 0.1, 1.0 }, .{ 2.0, 0.6, 2.5 }, axisAngle(.{ 0, 0, 1 }, std.math.pi / 3.0), 0);
    _ = try addStaticBox(b, .{ 0.1, 1.0, 1.0 }, .{ -2.0, 1.0, 2.5 }, identity_rotation, 0);

    const cv0 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0, 0 }, .collision_group = 0, .user_data = CharacterFlags.userData(CharacterFlags.slow_solve, 10) });
    const cv1 = try b.characterVirtual(.{ .base = .{ .shape = shapes.crouching }, .position = .{ -0.2, 0.6, -2.5 }, .listener = @intFromEnum(CharacterListenerMode.none), .collision_group = 0, .mass = 80, .max_strength = 400 });
    const cv2 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing, .max_slope_angle = zolt.math.degreesToRadians(45.0) }, .position = .{ 0, 0, 2.5 }, .back_face_mode = 0, .predictive_contact_distance = 0.15, .character_padding = 0.03, .collision_group = 1 });
    try b.characterVirtualQuery(.cv_refresh_contacts, cv0, cv_query, .{});
    try b.characterVirtualQuery(.cv_refresh_contacts, cv2, cv_query, .{});
    try b.expectCharacterVirtualGroundState(cv0, GroundState.on_ground);
    try b.expectCharacterVirtualGroundState(cv1, GroundState.in_air);
    var cv1_alive = true;

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        // Character 0: into the post, up the stairs (ExtendedUpdate) and off them, then back
        const v0: [3]f32 = if (u < 70) .{ 2.0, -0.5, 0 } else .{ -2.0, -0.5, 0 };
        try moveCharacterVirtual(b, cv0, v0, u >= 4, cv_query);
        // Character 1: falls, pushes the crate (a body filter that ignores the sensor, a shape filter without spheres)
        if (cv1_alive) try moveCharacterVirtual(b, cv1, .{ 1.5, -3.0, 0 }, false, .{ .layer = Layers.moving, .body_filter = .ignore_body, .body = sensor, .shape_filter = .no_spheres });
        // Character 2: through the sensor into the slope, then back into the wall (without layer filters from update 20)
        const v2: [3]f32 = if (u < 50) .{ 2.0, -1.0, 0 } else .{ -3.0, -1.0, 0 };
        try moveCharacterVirtual(b, cv2, v2, false, if (u < 20) cv_query else .{});
        switch (u) {
            2 => {
                try b.characterVirtualQuery(.cv_check_collision, cv0, cv_query, .{ .r = .{ 0.5, 0, 0 }, .f = &.{ 0, 0, 0, 1, 1, 0, 0, 0.1 }, .i3 = shapes.standing });
                try b.objectOp(.cv_settings_hash, cv0, .{});
                try b.objectOp(.cv_cancel_velocity_towards_steep_slopes, cv0, .{ .f = &.{ 2, 0, 0 } });
            },
            6 => try b.objectOp(.cv_set_listener, cv1, .{ .i = &.{@intFromEnum(CharacterListenerMode.record)} }),
            8 => {
                try b.objectOp(.cv_can_walk_stairs, cv0, .{ .f = &.{ 2, 0, 0 } });
                try b.objectOp(.cv_cancel_velocity_towards_steep_slopes, cv0, .{ .f = &.{ 2, 0, 0 } });
                try b.characterVirtualQuery(.cv_walk_stairs, cv0, cv_query, .{ .f = &.{ b.delta_time, 0, 0.4, 0, 0.05, 0, 0, 0.15, 0, 0, 0, -0.1, 0 } });
                try b.characterVirtualQuery(.cv_stick_to_floor, cv0, cv_query, .{ .f = &.{ 0, -0.5, 0 } });
            },
            12 => {
                try b.objectOp(.cv_set_listener, cv0, .{ .i = &.{@intFromEnum(CharacterListenerMode.modify)} });
                try b.objectOp(.cv_set_listener, cv1, .{ .i = &.{@intFromEnum(CharacterListenerMode.modify)} });
            },
            16 => try b.objectOp(.cv_update_ground_velocity, cv0, .{}),
            20 => {
                try b.objectOp(.cv_start_tracking_contact_changes, cv2, .{});
                try b.objectOp(.cv_has_collided_with_body, cv1, .{ .body = crate });
            },
            22 => try b.objectOp(.cv_finish_tracking_contact_changes, cv2, .{}),
            24 => {
                try b.objectOp(.cv_set_listener, cv0, .{ .i = &.{@intFromEnum(CharacterListenerMode.record)} });
                try b.objectOp(.cv_has_collided_with_body, cv2, .{ .body = sensor });
                try b.objectOp(.cv_has_collided_with_character, cv0, .{ .i = &.{cv1} });
            },
            30 => {
                try b.objectOp(.cv_save_restore_state, cv0, .{ .i = &.{0} });
                try b.objectOp(.cv_save_restore_state, cv2, .{ .i = &.{1} });
            },
            32 => {
                try b.characterVirtualQuery(.cv_set_shape, cv2, cv_query, .{ .f = &.{0.05}, .i3 = shapes.big });
                try b.characterVirtualQuery(.cv_set_shape, cv2, cv_query, .{ .f = &.{flt_max}, .i3 = shapes.crouching });
                try b.objectOp(.cv_set_inner_body_shape, cv2, .{ .i = &.{shapes.inner} });
            },
            36 => {
                try b.objectOp(.cv_set_mass, cv1, .{ .f = &.{100} });
                try b.objectOp(.cv_set_max_strength, cv1, .{ .f = &.{50} });
                try b.objectOp(.cv_set_penetration_recovery_speed, cv1, .{ .f = &.{0.5} });
                try b.objectOp(.cv_set_enhanced_internal_edge_removal, cv1, .{ .i = &.{1} });
                try b.objectOp(.cv_set_hit_reduction_cos_max_angle, cv1, .{ .f = &.{-1} });
                try b.objectOp(.cv_set_user_data, cv1, .{ .i = &.{ 5, 0 } });
            },
            40 => {
                try b.objectOp(.cv_set_max_num_hits, cv2, .{ .i = &.{1} });
                try b.objectOp(.cv_set_shape_offset, cv2, .{ .f = &.{ 0, 0.02, 0 } });
            },
            44 => {
                try b.objectOp(.cv_set_max_num_hits, cv2, .{ .i = &.{256} });
                try b.characterVirtualQuery(.cv_set_shape, cv2, cv_query, .{ .f = &.{0.1}, .i3 = shapes.standing });
            },
            46 => try b.sysOp(.snapshot, .{ .index = 0 }),
            50 => try b.sysOp(.restore_cross, .{ .index = 0 }), // Back to update 46: character 1 pushing the crate
            // The ground states of the up vector, the max slope angle and the supporting volume (character 2)
            56 => {
                try b.objectOp(.cv_set_up, cv2, .{ .f = &.{ 0.8, 0.6, 0 } });
                try b.objectOp(.cv_set_max_slope_angle, cv2, .{ .f = &.{zolt.math.degreesToRadians(30.0)} });
            },
            58 => {
                try b.objectOp(.cv_set_up, cv2, .{ .f = &.{ 0, 1, 0 } });
                try b.objectOp(.cv_set_max_slope_angle, cv2, .{ .f = &.{zolt.math.degreesToRadians(50.0)} });
                try b.objectOp(.cv_set_supporting_volume, cv2, .{ .f = &.{ 0, 1, 0, 0.5 } });
            },
            60 => try b.objectOp(.cv_set_supporting_volume, cv2, .{ .f = &.{ 0, 1, 0, -1.0e10 } }),
            64 => {
                try b.objectOp(.cv_set_position, cv1, .{ .r = .{ 1.5, 0.05, -1.5 } });
                try b.objectOp(.cv_set_rotation, cv1, .{ .f = &axisAngle(.{ 0, 1, 0 }, 0.5) });
                try b.characterVirtualQuery(.cv_refresh_contacts, cv1, cv_query, .{});
            },
            80 => {
                try b.objectOp(.cv_remove_from_collision_group, cv1, .{ .i = &.{0} });
                try b.objectOp(.cv_add_to_collision_group, cv1, .{ .i = &.{2} });
                try b.objectOp(.cv_set_character_vs_character_collision, cv1, .{ .i = &.{invalid_handle} });
            },
            90 => {
                try b.objectOp(.cv_destroy, cv1, .{});
                cv1_alive = false;
            },
            100 => try b.objectOp(.cv_set_character_vs_character_collision, cv0, .{ .i = &.{2} }),
            else => {},
        }
    }
}

/// Character vs character on a mesh floor: three character virtuals with inner bodies (one with mInnerBodyIDOverride)
/// walk into each other in one CharacterVsCharacterCollisionSimple (the character contact callbacks, solved by both
/// sides), a fourth one in another group passes through them; the inner bodies push a crate (contact listener callbacks
/// of the system), one inner body gets another shape (SetInnerBodyShape) and one character is destroyed (its inner body
/// leaves the system); a snapshot restored cross while the characters touch each other
fn buildCharacterVirtualCharacters(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 100;
    // A mesh floor of 8 x 8 quads over [-8, 8]^2
    var triangles: [128][9]f32 = undefined;
    for (0..8) |i| {
        for (0..8) |j| {
            const x0 = -8.0 + 2.0 * @as(f32, @floatFromInt(i));
            const z0 = -8.0 + 2.0 * @as(f32, @floatFromInt(j));
            const k = 2 * (8 * i + j);
            triangles[k] = .{ x0, 0, z0, x0, 0, z0 + 2, x0 + 2, 0, z0 + 2 };
            triangles[k + 1] = .{ x0, 0, z0, x0 + 2, 0, z0 + 2, x0 + 2, 0, z0 };
        }
    }
    const mesh = try b.mesh(&triangles);
    _ = try b.body(.{ .shape = mesh, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const crate_shape = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    _ = try b.body(.{ .position = .{ 0, 0.3, 2.2 }, .shape = crate_shape, .object_layer = Layers.moving }, .activate);

    const a = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ -1.2, 0, 0 }, .inner_body_shape = shapes.inner, .inner_body_layer = Layers.moving, .collision_group = 0 });
    const c = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 1.2, 0, 0 }, .inner_body_shape = shapes.inner, .inner_body_layer = Layers.moving, .inner_body_id_override = 300, .collision_group = 0, .listener = @intFromEnum(CharacterListenerMode.modify), .user_data = CharacterFlags.userData(CharacterFlags.cannot_push, 3) });
    const d = try b.characterVirtual(.{ .base = .{ .shape = shapes.crouching }, .position = .{ 0, 0, 1.0 }, .inner_body_shape = shapes.inner, .inner_body_layer = Layers.moving, .collision_group = 0 });
    const e = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0.2, 0, -1.5 }, .collision_group = 1 });
    var d_alive = true;

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        const towards: f32 = if (u < 60) 1.5 else -1.5;
        try moveCharacterVirtual(b, a, .{ towards, -0.5, 0 }, false, cv_query);
        try moveCharacterVirtual(b, c, .{ -towards, -0.5, 0 }, true, cv_query);
        if (d_alive) try moveCharacterVirtual(b, d, .{ 0, -0.5, towards }, false, .{ .layer = Layers.moving, .body_filter = .ignore_inner_body });
        try moveCharacterVirtual(b, e, .{ 0, -0.5, 2.0 }, false, cv_query);
        switch (u) {
            30 => {
                try b.objectOp(.cv_has_collided_with_character, a, .{ .i = &.{c} });
                try b.characterVirtualQuery(.cv_check_collision, a, cv_query, .{ .r = .{ -0.5, 0, 0 }, .f = &.{ 0, 0, 0, 1, 1, 0, 0, 0.2 }, .i3 = shapes.standing });
            },
            33 => try b.sysOp(.snapshot, .{ .index = 0 }),
            37 => try b.sysOp(.restore_cross, .{ .index = 0 }),
            40 => try b.objectOp(.cv_set_inner_body_shape, a, .{ .i = &.{shapes.standing} }),
            50 => try b.objectOp(.cv_set_user_data, c, .{ .i = &.{ 4, CharacterFlags.slow_solve } }),
            70 => {
                try b.objectOp(.cv_destroy, d, .{});
                d_alive = false;
            },
            else => {},
        }
    }
}

/// cv_set_step_velocity (Character::Step of the unit tests: the ground velocity when on the ground, a jump, gravity,
/// the `input` velocity), then ExtendedUpdate with `extended` (null: Update) and the filters of `query`
fn stepCharacterVirtual(b: *SceneBuilder, handle: u32, input: [3]f32, jump: f32, extended: ?SceneBuilder.ExtendedUpdateSettings, query: SceneBuilder.CharacterQuery) Allocator.Error!void {
    try b.objectOp(.cv_set_step_velocity, handle, .{ .f = &(input ++ character_gravity ++ [_]f32{ b.delta_time, jump }) });
    if (extended) |settings|
        try b.extendedUpdate(handle, b.delta_time, character_gravity, settings, query)
    else
        try b.characterVirtualQuery(.cv_update, handle, query, .{ .f = &([_]f32{b.delta_time} ++ character_gravity) });
}

/// The triangles of a staircase of `num_steps` steps of `step_height` that rises along +z from z = 0, 4 m wide (the
/// stairs of TestWalkStairs in CharacterVirtualTests.cpp)
fn stairsTriangles(allocator: Allocator, num_steps: u32, step_height: f32) Allocator.Error![][9]f32 {
    const triangles = try allocator.alloc([9]f32, 4 * num_steps);
    for (0..num_steps) |i| {
        const h = step_height;
        const y = step_height * @as(f32, @floatFromInt(i));
        // Start of step: base (0, y, y); left side b1, s1, p1, right side b2, s2, p2 (4 m to -x)
        const b1 = [3]f32{ 2, y, y };
        const s1 = [3]f32{ 2, y + h, y };
        const p1 = [3]f32{ 2, y + h, y + h };
        const b2 = [3]f32{ -2, y, y };
        const s2 = [3]f32{ -2, y + h, y };
        const p2 = [3]f32{ -2, y + h, y + h };
        triangles[4 * i + 0] = s1 ++ b1 ++ s2;
        triangles[4 * i + 1] = b1 ++ b2 ++ s2;
        triangles[4 * i + 2] = s1 ++ p2 ++ p1;
        triangles[4 * i + 3] = s1 ++ s2 ++ p2;
    }
    return triangles;
}

/// Stairs and slopes: character 0 climbs five box steps with ExtendedUpdate (WalkStairs, StickToFloor), crosses a
/// platform and drops off its edge (InAir, then OnGround on the floor); character 1 climbs a staircase of triangles (a mesh) with
/// custom ExtendedUpdateSettings and explicit CanWalkStairs / WalkStairs / StickToFloor, then walks off its top;
/// character 2 lands on a 60 degree slope and slides off it (OnSteepGround); character 3 runs down a 30 degree slope
/// that it sticks to (StickToFloor), then walks up again.
fn buildCharacterVirtualStairsSlopes(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 170;
    const floor = try addFloor(b, 30, 0);

    // Lane 0 (z = 0): five box steps of 0.2 m from x = 1 up to a platform at 1 m height that ends at x = 5
    for (0..5) |s| {
        const fs: f32 = @floatFromInt(s);
        _ = try addStaticBox(b, .{ 0.3, 0.1 + 0.1 * fs, 1.0 }, .{ 1.3 + 0.6 * fs, 0.1 + 0.1 * fs, 0 }, identity_rotation, 0);
    }
    const platform = try addStaticBox(b, .{ 0.5, 0.5, 1.0 }, .{ 4.5, 0.5, 0 }, identity_rotation, 0);

    // Lane 1 (x = -10): a staircase mesh of six steps of 0.3 m from z = 2
    const stairs = try stairsTriangles(b.allocator, 6, 0.3);
    defer b.allocator.free(stairs);
    const stairs_mesh = try b.mesh(stairs);
    const stairs_body = try b.body(.{ .position = .{ -10, 0, 2 }, .shape = stairs_mesh, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);

    // Lane 2 (z = -6): a slab tilted by 60 degrees (rising towards +x), lane 3 (z = -12): one tilted by 30 degrees
    const slab = try addStaticBox(b, .{ 1.5, 0.1, 1.5 }, .{ 0, 1.5, -6 }, axisAngle(.{ 0, 0, 1 }, std.math.pi / 3.0), 0);
    const ramp = try addStaticBox(b, .{ 3.0, 0.1, 1.5 }, .{ 3, 1.5, -12 }, axisAngle(.{ 0, 0, 1 }, std.math.pi / 6.0), 0);

    const cv0 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0, 0 } });
    const cv1 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ -10, 0, 0 } });
    const cv2 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0.3, 3.0, -6 } });
    const cv3 = try b.characterVirtual(.{ .base = .{ .shape = shapes.crouching }, .position = .{ 5.0, 3.0, -12 } });
    const stairs_settings: SceneBuilder.ExtendedUpdateSettings = .{ .walk_stairs_step_down_extra = .{ 0, -0.1, 0 }, .walk_stairs_cos_angle_forward_contact = 0.5, .walk_stairs_min_step_forward = 0.05 };

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        try stepCharacterVirtual(b, cv0, .{ 2.5, 0, 0 }, 0, .{}, cv_query);
        try stepCharacterVirtual(b, cv1, .{ 0, 0, 2.5 }, 0, stairs_settings, cv_query);
        try stepCharacterVirtual(b, cv2, .{ 0, 0, 0 }, 0, null, cv_query);
        try stepCharacterVirtual(b, cv3, if (u < 80) .{ -4, 0, 0 } else .{ 3, 0, 0 }, 0, .{}, cv_query);
        switch (u) {
            16 => {
                try b.objectOp(.cv_can_walk_stairs, cv1, .{ .f = &.{ 0, 0, 2.5 } });
                try b.characterVirtualQuery(.cv_walk_stairs, cv1, cv_query, .{ .f = &.{ b.delta_time, 0, 0.4, 0, 0, 0, 0.05, 0, 0, 0.15, 0, -0.1, 0 } });
                try b.objectOp(.cv_has_collided_with_body, cv1, .{ .body = stairs_body });
            },
            30 => try b.expectCharacterVirtualValue(cv2, .ground_body_is, 1, .{ .body = slab }), // On steep ground
            40 => try b.expectCharacterVirtualValue(cv3, .ground_body_is, 1, .{ .body = ramp }),
            60 => try b.characterVirtualQuery(.cv_stick_to_floor, cv1, cv_query, .{ .f = &.{ 0, -0.5, 0 } }),
            120 => try b.expectCharacterVirtualValue(cv0, .ground_body_is, 1, .{ .body = platform }),
            165 => try b.expectCharacterVirtualValue(cv0, .ground_body_is, 1, .{ .body = floor }), // Dropped off the platform
            else => {},
        }
    }
}

/// Moving and rotating platforms (kinematic bodies): character 0 rides an elevator up and down, character 1 a turntable
/// (the ground velocity of a rotating body, CalculateCharacterGroundVelocity), character 2 a platform that slides back
/// and forth, character 3 walks towards the center of the turntable. Every character takes the velocity of its ground
/// (cv_set_step_velocity) before ExtendedUpdate; UpdateGroundVelocity in between.
fn buildCharacterVirtualPlatforms(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 120;
    _ = try addFloor(b, 30, 0);
    const platform_shape = try b.box(.{ 1.5, 0.2, 1.5 }, 0.05);
    const turntable_shape = try b.box(.{ 3.0, 0.2, 3.0 }, 0.05);
    const elevator = try b.body(.{ .position = .{ 0, 0.25, 0 }, .shape = platform_shape, .motion_type = 1, .object_layer = Layers.moving, .allow_sleeping = 0 }, .activate);
    const turntable = try b.body(.{ .position = .{ 8, 0.25, 0 }, .angular_velocity = .{ 0, 1, 0 }, .shape = turntable_shape, .motion_type = 1, .object_layer = Layers.moving, .allow_sleeping = 0 }, .activate);
    const slider = try b.body(.{ .position = .{ -8, 0.25, 0 }, .shape = platform_shape, .motion_type = 1, .object_layer = Layers.moving, .allow_sleeping = 0 }, .activate);

    const cv0 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0.45, 0 } });
    const cv1 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 10, 0.45, 0 } });
    const cv2 = try b.characterVirtual(.{ .base = .{ .shape = shapes.crouching }, .position = .{ -8, 0.45, 0 } });
    const cv3 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 8, 0.45, 2.5 }, .inner_body_shape = shapes.inner, .inner_body_layer = Layers.moving });

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        switch (u) {
            10 => {
                try b.op(.set_linear_velocity, elevator, .{ .f = &.{ 0, 0.6, 0 } });
                try b.op(.set_linear_velocity, slider, .{ .f = &.{ 1.5, 0, 0 } });
            },
            50 => try b.op(.set_linear_velocity, elevator, .{ .f = &.{ 0, -0.6, 0 } }),
            60 => try b.op(.set_linear_velocity, slider, .{ .f = &.{ -1.5, 0, 0 } }),
            85 => try b.op(.set_linear_velocity, elevator, .{ .f = &.{ 0, 0, 0 } }),
            100 => try b.op(.set_angular_velocity, turntable, .{ .f = &.{ 0, -2, 0 } }),
            else => {},
        }
        try stepCharacterVirtual(b, cv0, .{ 0, 0, 0 }, 0, .{}, cv_query);
        try stepCharacterVirtual(b, cv1, .{ 0, 0, 0 }, 0, .{}, cv_query);
        try stepCharacterVirtual(b, cv2, .{ 0, 0, 0 }, 0, .{}, cv_query);
        try stepCharacterVirtual(b, cv3, if (u < 40) .{ 0, 0, -0.5 } else .{ 0, 0, 0 }, 0, null, cv_query);
        switch (u) {
            30 => {
                try b.expectCharacterVirtualValue(cv0, .ground_body_is, 1, .{ .body = elevator });
                try b.expectCharacterVirtualValue(cv0, .ground_velocity_not_zero, 1, .{});
                try b.expectCharacterVirtualValue(cv1, .ground_body_is, 1, .{ .body = turntable });
                try b.expectCharacterVirtualValue(cv1, .ground_velocity_not_zero, 1, .{});
                try b.expectCharacterVirtualValue(cv2, .ground_body_is, 1, .{ .body = slider });
                try b.objectOp(.cv_update_ground_velocity, cv1, .{});
            },
            70 => {
                try b.objectOp(.cv_update_ground_velocity, cv0, .{});
                try b.objectOp(.cv_update_ground_velocity, cv3, .{});
            },
            else => {},
        }
    }
}

/// Many characters in two CharacterVsCharacterCollisionSimple groups: 16 characters on a circle walk to its center
/// (Update and ExtendedUpdate), push each other and the inner bodies of every fourth one, then walk back
fn buildCharacterVirtualCrowd(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 90;
    _ = try addFloor(b, 20, 0);
    const n = 16;
    var handles: [n]u32 = undefined;
    var directions: [n][3]f32 = undefined;
    for (&handles, &directions, 0..) |*h, *d, k| {
        const angle = 2.0 * std.math.pi * @as(f64, @floatFromInt(k)) / n;
        d.* = .{ @floatCast(@cos(angle)), 0, @floatCast(@sin(angle)) };
        h.* = try b.characterVirtual(.{
            .base = .{ .shape = if (k % 3 == 0) shapes.crouching else shapes.standing },
            .position = .{ 2.0 * d[0], 0, 2.0 * d[2] },
            .collision_group = @intCast(k % 2),
            .inner_body_shape = if (k % 4 == 0) shapes.inner else invalid_handle,
            .inner_body_layer = Layers.moving,
            .character_padding = if (k % 5 == 0) 0.04 else 0.02,
        });
    }

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        const speed: f32 = if (u < 45) -2.0 else 2.0;
        for (handles, directions, 0..) |h, d, k|
            try stepCharacterVirtual(b, h, .{ speed * d[0], 0, speed * d[2] }, 0, if (k % 2 == 0) .{} else null, cv_query);
        if (u == 40) {
            try b.expectCharacterVirtualValue(handles[0], .num_colliding_character_contacts, 2, .{});
            try b.objectOp(.cv_has_collided_with_character, handles[0], .{ .i = &.{handles[2]} });
        }
    }
}

/// The inner bodies push and are pushed: character 0 pushes three crates (HandleContact impulses, its inner body
/// collides with them in the simulation), a ball rolls into the inner body of standing character 1 (the character
/// sees the ball approach and gets pushed), character 2 stands on a big crate (Update adds its weight as an impulse to
/// the crate) and walks off it
fn buildCharacterVirtualInnerBody(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 120;
    _ = try addFloor(b, 30, 0);
    const crate_shape = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    for (0..3) |i|
        _ = try b.body(.{ .position = .{ 1.5 + 0.61 * real(i), 0.3, 0 }, .shape = crate_shape, .friction = 0.5, .object_layer = Layers.moving }, .activate);
    const ball_shape = try b.sphere(0.25);
    _ = try b.body(.{ .position = .{ 4, 0.25, 4 }, .linear_velocity = .{ -6, 0, 0 }, .shape = ball_shape, .object_layer = Layers.moving }, .activate);
    const big_shape = try b.box(.{ 0.8, 0.4, 0.8 }, 0.05);
    const big = try b.body(.{ .position = .{ 0, 0.4, -4 }, .shape = big_shape, .friction = 0.8, .object_layer = Layers.moving }, .activate);

    const cv0 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0, 0 }, .inner_body_shape = shapes.inner, .inner_body_layer = Layers.moving, .max_strength = 500 });
    const cv1 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0, 4 }, .inner_body_shape = shapes.inner, .inner_body_layer = Layers.moving });
    const cv2 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0.82, -4 }, .inner_body_shape = shapes.inner, .inner_body_layer = Layers.moving, .mass = 150 });

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        try stepCharacterVirtual(b, cv0, if (u < 90) .{ 1.5, 0, 0 } else .{ -1.5, 0, 0 }, 0, .{}, cv_query);
        try stepCharacterVirtual(b, cv1, .{ 0, 0, 0 }, 0, null, cv_query);
        try stepCharacterVirtual(b, cv2, if (u < 60) .{ 0, 0, 0 } else .{ 0, 0, 2 }, 0, if (u < 60) null else .{}, cv_query);
        if (u == 30) try b.expectCharacterVirtualValue(cv2, .ground_body_is, 1, .{ .body = big });
    }
}

/// The contact limits of TestContactPointLimit: character 0 walks in a half cylinder of 2 x 128 segments with
/// mMaxNumHits 32 (the hit reduction merges similar contacts, GetMaxHitsExceeded), from update 50 without hit reduction
/// (the collector gives up when full); character 1 walks next to the cylinder
fn buildCharacterVirtualContactLimits(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 100;
    _ = try addFloor(b, 20, 0);

    // The half cylinder (its first and last segments are walls) with end caps
    const character_radius: f32 = 0.3 + 0.02;
    const pos_segments = 2;
    const angle_segments = 128;
    const cylinder_length: f32 = 2.0;
    var vertices: [pos_segments * angle_segments][3]f32 = undefined;
    var triangles: std.ArrayList([3]u32) = .empty;
    defer triangles.deinit(b.allocator);
    for (0..pos_segments) |pos| {
        for (0..angle_segments) |angle| {
            const start: u32 = @intCast(pos * angle_segments + angle);
            const radius = character_radius + 0.01;
            const angle_rad = (-0.5 + @as(f32, @floatFromInt(angle)) / angle_segments) * zolt.math.pi;
            const s = zolt.trigonometry.sin(angle_rad);
            const c = zolt.trigonometry.cos(angle_rad);
            const x = cylinder_length * (-0.5 + @as(f32, @floatFromInt(pos)) / (pos_segments - 1));
            const y = if (angle == 0 or angle == angle_segments - 1) 0.5 else (1.0 - c) * radius;
            vertices[start] = .{ x, y, s * radius };
            if (pos > 0 and angle > 0) {
                try triangles.append(b.allocator, .{ start, start - 1, start - angle_segments });
                try triangles.append(b.allocator, .{ start - 1, start - angle_segments - 1, start - angle_segments });
            }
        }
    }
    const end: u32 = angle_segments * (pos_segments - 1);
    for (0..angle_segments - 1) |angle_usize| {
        const angle: u32 = @intCast(angle_usize);
        try triangles.append(b.allocator, .{ 0, angle + 1, angle });
        try triangles.append(b.allocator, .{ end, end + angle, end + angle + 1 });
    }
    const cylinder = try b.indexedMesh(&vertices, triangles.items, 8);
    _ = try b.body(.{ .position = .{ 0, 1, 0 }, .shape = cylinder, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);

    const cv0 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 1, 0 }, .max_num_hits = 32 });
    const cv1 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0, 1 }, .max_num_hits = 32 });

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        const vx: f32 = if (u < 30 or (u >= 50 and u < 75)) 2.0 else -2.0;
        try stepCharacterVirtual(b, cv0, .{ vx, 0, 0 }, 0, null, cv_query);
        try stepCharacterVirtual(b, cv1, .{ vx, 0, 0 }, 0, null, cv_query);
        switch (u) {
            10 => {
                try b.expectCharacterVirtualValue(cv0, .max_hits_exceeded, 1, .{});
                try b.expectCharacterVirtualValue(cv1, .max_hits_exceeded, 0, .{});
            },
            50 => try b.objectOp(.cv_set_hit_reduction_cos_max_angle, cv0, .{ .f = &.{-1} }),
            60 => {
                try b.expectCharacterVirtualValue(cv0, .max_hits_exceeded, 1, .{});
                try b.expectCharacterVirtualValue(cv0, .num_active_contacts, 32, .{});
            },
            80 => try b.objectOp(.cv_set_max_num_hits, cv0, .{ .i = &.{256} }),
            else => {},
        }
    }
}

/// Penetration recovery: three characters start inside a box (turned by 30 degrees, so that the contact normals have
/// no zero components) with penetration recovery speeds 0, 0.5 and 1 (TestInitiallyIntersecting; the first one changes
/// to 0.75 later), a fourth one with speed 0.75 between two 75 degree slabs that intersect it
/// (TestInitiallyIntersecting2) is driven upwards, then walks out of them
fn buildCharacterVirtualPenetration(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 60;
    _ = try addFloor(b, 20, 0);
    const speeds = [_]f32{ 0.0, 0.5, 1.0 };
    var handles: [3]u32 = undefined;
    for (&handles, speeds, 0..) |*h, speed, i| {
        const z = 3 * real(i);
        _ = try addStaticBox(b, .{ 0.5, 0.5, 0.5 }, .{ -0.5, 0.5, z }, axisAngle(.{ 0, 1, 0 }, std.math.pi / 6.0), 0);
        h.* = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0, z }, .penetration_recovery_speed = speed });
    }
    _ = try addStaticBox(b, .{ 1.0, 0.1, 1.0 }, .{ 0, 0, -4 }, axisAngle(.{ 0, 0, 1 }, 75.0 * std.math.pi / 180.0), 0);
    _ = try addStaticBox(b, .{ 1.0, 0.1, 1.0 }, .{ 0, 0, -4 }, axisAngle(.{ 0, 0, 1 }, -75.0 * std.math.pi / 180.0), 0);
    const cv3 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0, -4 }, .penetration_recovery_speed = 0.75 });

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        for (handles) |h|
            try stepCharacterVirtual(b, h, .{ 0, 0, 0 }, 0, null, cv_query);
        try stepCharacterVirtual(b, cv3, if (u < 20) .{ 0, 0, 0 } else .{ 2, 0, 0 }, 0, null, cv_query);
        switch (u) {
            5 => try b.expectCharacterVirtualValue(handles[0], .num_colliding_contacts, 2, .{}),
            20 => try b.objectOp(.cv_set_penetration_recovery_speed, handles[0], .{ .f = &.{0.75} }),
            else => {},
        }
    }
}

/// SetShape with and without a max penetration depth under a low ceiling: character 0 cannot stand up under it (the
/// switch fails), walks out and stands up; character 1 stands up without the check (it penetrates the ceiling) and
/// gets a new inner body shape; character 2 switches to no shape (fails) and back to its own shape (succeeds)
fn buildCharacterVirtualSetShape(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 80;
    _ = try addFloor(b, 20, 0);
    _ = try addStaticBox(b, .{ 1.0, 0.25, 1.5 }, .{ 0, 1.7, 0 }, identity_rotation, 0); // A ceiling 1.45 m above the floor

    const cv0 = try b.characterVirtual(.{ .base = .{ .shape = shapes.crouching }, .position = .{ 0, 0, -0.7 } });
    const cv1 = try b.characterVirtual(.{ .base = .{ .shape = shapes.crouching }, .position = .{ 0, 0, 0.7 }, .inner_body_shape = shapes.inner, .inner_body_layer = Layers.moving });
    const cv2 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 4, 0, 0 } });

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        try stepCharacterVirtual(b, cv0, if (u >= 10 and u < 40) .{ 3, 0, 0 } else .{ 0, 0, 0 }, 0, null, cv_query);
        try stepCharacterVirtual(b, cv1, .{ 0, 0, 0 }, 0, null, cv_query);
        try stepCharacterVirtual(b, cv2, .{ 0, 0, 0 }, 0, .{}, cv_query);
        switch (u) {
            5 => {
                try b.characterVirtualQuery(.cv_set_shape, cv0, cv_query, .{ .f = &.{0.05}, .i3 = shapes.standing });
                try b.expectCharacterVirtualValue(cv0, .shape, shapes.crouching, .{});
                try b.characterVirtualQuery(.cv_set_shape, cv1, cv_query, .{ .f = &.{flt_max}, .i3 = shapes.standing });
                try b.expectCharacterVirtualValue(cv1, .shape, shapes.standing, .{});
                try b.characterVirtualQuery(.cv_set_shape, cv2, cv_query, .{ .f = &.{0.05}, .i3 = invalid_handle });
                try b.expectCharacterVirtualValue(cv2, .shape, shapes.standing, .{});
            },
            10 => try b.objectOp(.cv_set_inner_body_shape, cv1, .{ .i = &.{shapes.standing} }),
            45 => {
                try b.characterVirtualQuery(.cv_set_shape, cv0, cv_query, .{ .f = &.{0.05}, .i3 = shapes.standing });
                try b.expectCharacterVirtualValue(cv0, .shape, shapes.standing, .{});
                try b.characterVirtualQuery(.cv_set_shape, cv2, cv_query, .{ .f = &.{0.05}, .i3 = shapes.crouching });
                try b.expectCharacterVirtualValue(cv2, .shape, shapes.crouching, .{});
            },
            50 => {
                try b.characterVirtualQuery(.cv_set_shape, cv2, cv_query, .{ .f = &.{flt_max}, .i3 = shapes.big });
                try b.characterVirtualQuery(.cv_set_shape, cv1, cv_query, .{ .f = &.{0.05}, .i3 = shapes.crouching });
            },
            else => {},
        }
    }
}

/// Enhanced internal edge removal: two characters slide over a mesh floor of 1 m quads with a row of small bumps,
/// pressed onto it (Update with a downward velocity); character 0 starts with mEnhancedInternalEdgeRemoval
/// (CollideShapeWithInternalEdgeRemoval), character 1 without, they swap at update 40 (SetEnhancedInternalEdgeRemoval)
fn buildCharacterVirtualInternalEdges(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 80;
    const floor = try addMeshTerrain(b, 12, 1.0, true);
    _ = try b.body(.{ .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    // Bumps: thin boxes 2 cm high across the lanes
    for (0..4) |i|
        _ = try addStaticBox(b, .{ 0.05, 0.01, 3.0 }, .{ -2.0 + 1.5 * real(i), 0.01, 0 }, identity_rotation, 0);

    const cv0 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing, .enhanced_internal_edge_removal = 1 }, .position = .{ -4, 0, -1 } });
    const cv1 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ -4, 0, 1 } });

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        const vx: f32 = if (u < 50) 3.0 else -3.0;
        try moveCharacterVirtual(b, cv0, .{ vx, -1.0, 0.2 }, false, cv_query);
        try moveCharacterVirtual(b, cv1, .{ vx, -1.0, -0.2 }, u >= 60, cv_query);
        if (u == 40) {
            try b.objectOp(.cv_set_enhanced_internal_edge_removal, cv0, .{ .i = &.{0} });
            try b.objectOp(.cv_set_enhanced_internal_edge_removal, cv1, .{ .i = &.{1} });
        }
    }
}

/// Terrain: two characters walk over a height field terrain and two over a mesh terrain with ExtendedUpdate, turning
/// every 30 updates; one of each has enhanced internal edge removal
fn buildCharacterVirtualTerrain(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 120;
    const height_field = try addHeightFieldTerrain(b, 17, 1.0);
    _ = try b.body(.{ .shape = height_field, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    const mesh = try addMeshTerrain(b, 16, 1.0, false);
    _ = try b.body(.{ .position = .{ 20, 0, 0 }, .shape = mesh, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);

    const handles = [_]u32{
        try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ -3, 1, -2 } }),
        try b.characterVirtual(.{ .base = .{ .shape = shapes.crouching, .enhanced_internal_edge_removal = 1 }, .position = .{ 2, 1, 3 } }),
        try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 17, 1, -2 } }),
        try b.characterVirtual(.{ .base = .{ .shape = shapes.standing, .enhanced_internal_edge_removal = 1 }, .position = .{ 22, 1, 3 } }),
    };
    const directions = [_][3]f32{ .{ 2, 0, 0 }, .{ 0, 0, 2 }, .{ -2, 0, 0 }, .{ 0, 0, -2 } };

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        for (handles, 0..) |h, k|
            try stepCharacterVirtual(b, h, directions[(k + u / 30) % 4], 0, .{}, cv_query);
    }
}

/// Cross save / restore of the character states: snapshots while the characters touch the stairs (character 0), a
/// crate (character 1) and each other (characters 3 and 4, they walk apart again from update 45), and while character
/// 2 jumps (no contacts); restore_cross back to them (twice in a row once, every restore sets the scene back by 10
/// updates); SaveState / RestoreState of single characters, validating too
fn buildCharacterVirtualCrossRestore(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 80;
    _ = try addFloor(b, 20, 0);
    for (0..3) |s| {
        const fs: f32 = @floatFromInt(s);
        _ = try addStaticBox(b, .{ 0.3, 0.1 + 0.1 * fs, 1.0 }, .{ 1.3 + 0.6 * fs, 0.1 + 0.1 * fs, 0 }, identity_rotation, 0);
    }
    const crate_shape = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    _ = try b.body(.{ .position = .{ 1.2, 0.3, -3 }, .shape = crate_shape, .object_layer = Layers.moving }, .activate);

    const cv0 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0, 0 }, .collision_group = 0 });
    const cv1 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0.2, 0, -3 }, .inner_body_shape = shapes.inner, .inner_body_layer = Layers.moving, .collision_group = 0 });
    const cv2 = try b.characterVirtual(.{ .base = .{ .shape = shapes.crouching }, .position = .{ 0, 0, -1.5 }, .collision_group = 0, .listener = @intFromEnum(CharacterListenerMode.modify) });
    const cv3 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ -0.65, 0, 3 }, .collision_group = 0 });
    const cv4 = try b.characterVirtual(.{ .base = .{ .shape = shapes.crouching }, .position = .{ 0.65, 0, 3 }, .collision_group = 0 });

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        switch (u) {
            10 => try b.sysOp(.snapshot, .{ .index = 0 }),
            20 => try b.sysOp(.restore_cross, .{ .index = 0 }),
            30 => try b.sysOp(.snapshot, .{ .index = 1 }),
            40 => try b.sysOp(.restore_cross, .{ .index = 1 }),
            41 => try b.sysOp(.restore_cross, .{ .index = 1 }),
            50 => {
                try b.objectOp(.cv_save_restore_state, cv0, .{ .i = &.{0} });
                try b.objectOp(.cv_save_restore_state, cv1, .{ .i = &.{1} });
            },
            else => {},
        }
        try stepCharacterVirtual(b, cv0, .{ 2, 0, 0 }, 0, .{}, cv_query);
        try stepCharacterVirtual(b, cv1, .{ 1.5, 0, 0 }, 0, null, cv_query);
        try stepCharacterVirtual(b, cv2, .{ 0, 0, -0.5 }, if (u == 28) 4.0 else 0.0, null, cv_query);
        try stepCharacterVirtual(b, cv3, if (u < 45) .{ 1.5, 0, 0 } else .{ -1.5, 0, 0 }, 0, .{}, cv_query);
        try stepCharacterVirtual(b, cv4, if (u < 45) .{ -1.5, 0, 0 } else .{ 1.5, 0, 0 }, 0, null, cv_query);
    }
}

/// Far from the origin (the base offsets of the character queries are the character's position; in double precision
/// the positions keep their precision): stairs, a slope and two characters that walk into each other and apart again
fn buildCharacterVirtualFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 100;
    const o: [3]Real = .{ 5000, 200, -12000 };
    const floor = try b.box(.{ 20, 1, 20 }, 0);
    _ = try b.body(.{ .position = .{ o[0], o[1] - 1, o[2] }, .shape = floor, .motion_type = 0, .object_layer = Layers.non_moving }, .dont_activate);
    for (0..3) |s| {
        const fs: f32 = @floatFromInt(s);
        _ = try addStaticBox(b, .{ 0.3, 0.1 + 0.1 * fs, 1.0 }, .{ o[0] + real(1.3 + 0.6 * fs), o[1] + real(0.1 + 0.1 * fs), o[2] }, identity_rotation, 0);
    }
    _ = try addStaticBox(b, .{ 3.0, 0.1, 1.5 }, .{ o[0] + 3, o[1] + 1.5, o[2] - 6 }, axisAngle(.{ 0, 0, 1 }, std.math.pi / 6.0), 0);

    const cv0 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = o, .collision_group = 0, .inner_body_shape = shapes.inner, .inner_body_layer = Layers.moving });
    const cv1 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ o[0] + 5, o[1] + 2.5, o[2] - 6 }, .collision_group = 0 });
    const cv2 = try b.characterVirtual(.{ .base = .{ .shape = shapes.crouching }, .position = .{ o[0] - 2, o[1], o[2] + 4 }, .collision_group = 0 });
    const cv3 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ o[0] + 2, o[1], o[2] + 4 }, .collision_group = 0 });

    for (0..b.num_updates) |update| {
        const u: u32 = @intCast(update);
        b.at(u);
        try stepCharacterVirtual(b, cv0, .{ 2.5, 0, 0 }, 0, .{}, cv_query);
        try stepCharacterVirtual(b, cv1, .{ -3, 0, 0 }, 0, .{}, cv_query);
        try stepCharacterVirtual(b, cv2, if (u < 60) .{ 2, 0, 0 } else .{ -2, 0, 0 }, 0, null, cv_query);
        try stepCharacterVirtual(b, cv3, if (u < 60) .{ -2, 0, 0 } else .{ 2, 0, 0 }, 0, .{}, cv_query);
        if (u == 30)
            try b.characterVirtualQuery(.cv_check_collision, cv2, cv_query, .{ .r = .{ o[0] - 0.5, o[1], o[2] + 4 }, .f = &.{ 0, 0, 0, 1, 1, 0, 0, 0.2 }, .i3 = shapes.standing });
    }
}

/// One scene per CharacterListenerMode (none, record, modify) on the same setup: a floor that is a conveyor belt in
/// modify mode, a post that it rejects, a crate that cannot push / receive impulses, a free crate; character 0 walks
/// into them (it hits the post off center and slides around it unless the post is rejected), characters 1 and 2 walk
/// into each other inside a sensor and apart again (character 2's user data stops it from pushing, the slowed solves
/// of character 1)
fn buildCharacterVirtualListener(comptime mode: CharacterListenerMode) fn (b: *SceneBuilder) Allocator.Error!void {
    return struct {
        fn build(b: *SceneBuilder) Allocator.Error!void {
            const shapes = try CharacterShapes.add(b);
            b.num_updates = 70;
            const floor = try addFloor(b, 20, 0);
            try b.op(.set_user_data, floor, .{ .index = 0, .count = CharacterFlags.conveyor });
            _ = try addStaticBox(b, .{ 0.1, 0.5, 0.1 }, .{ 0.8, 0.5, 0 }, identity_rotation, CharacterFlags.userData(CharacterFlags.reject, 2));
            const crate_shape = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
            _ = try b.body(.{ .position = .{ 1.8, 0.3, 0 }, .shape = crate_shape, .object_layer = Layers.moving, .user_data = CharacterFlags.userData(CharacterFlags.cannot_push | CharacterFlags.no_impulses, 1) }, .activate);
            _ = try b.body(.{ .position = .{ 2.8, 0.3, 0.1 }, .shape = crate_shape, .object_layer = Layers.moving }, .activate);
            const sensor_shape = try b.box(.{ 0.2, 0.5, 0.5 }, 0.05);
            _ = try b.body(.{ .position = .{ 0.75, 0.5, -3 }, .shape = sensor_shape, .motion_type = 0, .is_sensor = 1, .object_layer = Layers.sensor }, .dont_activate);

            const m: u32 = @intFromEnum(mode);
            const cv0 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0, 0.25 }, .listener = m, .inner_body_shape = shapes.inner, .inner_body_layer = Layers.moving });
            const cv1 = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ 0, 0, -3 }, .listener = m, .collision_group = 0, .user_data = CharacterFlags.userData(CharacterFlags.slow_solve, 1) });
            const cv2 = try b.characterVirtual(.{ .base = .{ .shape = shapes.crouching }, .position = .{ 1.5, 0, -3 }, .listener = m, .collision_group = 0, .user_data = CharacterFlags.userData(CharacterFlags.cannot_push, 2) });

            for (0..b.num_updates) |update| {
                const u: u32 = @intCast(update);
                b.at(u);
                try stepCharacterVirtual(b, cv0, .{ 1.5, 0, 0 }, 0, .{}, cv_query);
                try stepCharacterVirtual(b, cv1, if (u < 50) .{ 1.5, 0, 0 } else .{ -1.5, 0, 0 }, 0, null, cv_query);
                try stepCharacterVirtual(b, cv2, if (u < 50) .{ -1.5, 0, 0 } else .{ 1.5, 0, 0 }, 0, .{}, cv_query);
                if (u == 20) {
                    try b.objectOp(.cv_start_tracking_contact_changes, cv0, .{});
                    try b.objectOp(.cv_start_tracking_contact_changes, cv0, .{});
                }
                if (u == 22) try b.objectOp(.cv_finish_tracking_contact_changes, cv0, .{});
                if (u == 24) try b.objectOp(.cv_finish_tracking_contact_changes, cv0, .{});
            }
        }
    }.build;
}

/// PerformanceTest's CharacterVirtualScene (milestone M8, -q=Discrete): the terrain, the stairs, the wall and the 100
/// boxes of Load, the 100 characters with inner bodies in one CharacterVsCharacterCollisionSimple of StartTest (IDs 1 to
/// 100 like after CharacterID::sSetNextCharacterID), OptimizeBroadPhase, and UpdateTest's character steps
/// (cv_perf_update with UpdateTest's counter) before every update, compared after every update for the first 100. The
/// listener records the callbacks instead of hashing them (a listener that changes nothing behaves like the scene's).
fn buildPerfCharacterVirtual(b: *SceneBuilder) Allocator.Error!void {
    perfSystem(b);
    b.num_updates = 100;

    // Load: the mesh terrain
    const n = 100;
    const cell_size: f32 = 0.5;
    const max_height: f32 = 2.0;
    const center: f32 = @as(f32, n) * cell_size / 2;
    const vertices = try b.allocator.alloc([3]f32, (n + 1) * (n + 1));
    defer b.allocator.free(vertices);
    for (0..n + 1) |x| {
        for (0..n + 1) |z| {
            const fx: f32 = @floatFromInt(x);
            const fz: f32 = @floatFromInt(z);
            const height = zolt.trigonometry.sin(fx * 20.0 / @as(f32, n)) * zolt.trigonometry.cos(fz * 20.0 / @as(f32, n));
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
    const mesh = try b.indexedMesh(vertices, triangles, 8);
    _ = try b.createAndAddBody(try b.addBodyDesc(.{ .position = .{ -center, 0, -center }, .shape = mesh, .motion_type = 0, .object_layer = Layers.non_moving }), .dont_activate);

    // Load: the pyramid stairs
    const stairs_step_height: f32 = 0.3;
    for (0..10) |i| {
        const fi: f32 = @floatFromInt(i);
        const width = 4.0 - 0.4 * fi;
        const step = try b.box(.{ width, 0.5 * stairs_step_height, width }, 0.05);
        _ = try b.createAndAddBody(try b.addBodyDesc(.{ .position = .{ -4.0, -1.0 + @as(Real, fi * stairs_step_height), 0 }, .shape = step, .motion_type = 0, .object_layer = Layers.non_moving }), .dont_activate);
    }

    // Load: the wall of vertical pillars
    const wall = try b.box(.{ 0.1, 2.5, 0.1 }, 0.0);
    for (0..10) |z|
        _ = try b.createAndAddBody(try b.addBodyDesc(.{ .position = .{ 2.0, 1.0, 2.0 + 0.2 * real(z) }, .shape = wall, .motion_type = 0, .object_layer = Layers.non_moving }), .dont_activate);

    // Load: the dynamic boxes (CalculateInertia, mass 1)
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    for (0..10) |x| {
        for (0..10) |z|
            _ = try b.createAndAddBody(try b.addBodyDesc(.{ .position = .{ 4.0 * real(x) - 20.0, 5.0, 4.0 * real(z) - 20.0 }, .shape = box, .object_layer = Layers.moving, .override_mass_properties = 1, .mass = 1.0 }), .activate);
    }

    // StartTest: the characters
    const character_height_standing: f32 = 1.35;
    const character_radius_standing: f32 = 0.3;
    const inner_shape_fraction: f32 = 0.9;
    const standing_shape = try b.rotatedTranslated(try b.capsule(0.5 * character_height_standing, character_radius_standing), .{ 0, 0.5 * character_height_standing + character_radius_standing, 0 }, identity_rotation);
    const inner_standing_shape = try b.rotatedTranslated(try b.capsule(0.5 * inner_shape_fraction * character_height_standing, inner_shape_fraction * character_radius_standing), .{ 0, 0.5 * character_height_standing + character_radius_standing, 0 }, identity_rotation);
    var handles: [100]u32 = undefined;
    for (0..10) |y| {
        for (0..10) |x|
            handles[10 * y + x] = try b.characterVirtual(.{ .base = .{ .shape = standing_shape, .supporting_volume = .{ 0, 1, 0, -character_radius_standing } }, .position = .{ 4.0 * real(x) - 20.0, 2.0, 4.0 * real(y) - 20.0 }, .inner_body_shape = inner_standing_shape, .inner_body_layer = Layers.moving, .collision_group = 0 });
    }

    // Optimize the broadphase to prevent an expensive first frame (PerformanceTest.cpp)
    try b.sysOp(.optimize_broad_phase, .{});

    // UpdateTest: change direction every 2 seconds
    var time: f32 = 0.0;
    for (0..b.num_updates) |update| {
        b.at(@intCast(update));
        time += b.delta_time;
        var count: u64 = @as(u64, @intFromFloat(time / 2.0)) * 100;
        for (handles) |h| {
            try b.characterVirtualQuery(.cv_perf_update, h, .{ .layer = Layers.moving }, .{ .f = &.{b.delta_time} });
            b.ops.items[b.ops.items.len - 1].count = @intCast(count); // UpdateTest's counter (Jolt's uint64)
            count += 1;
        }
    }
}

// (append the CharacterVirtual scenes here)

// ---- Ragdoll (Phase 7, Blocker.ragdolls) ----

/// The ragdoll of the ragdoll scenes: a skeleton of 4 joints (pelvis, spine, head, leg) at the pelvis position `root`,
/// swing twists to the spine and the head, a hinge to the leg (DriveToPoseUsingMotors drives these two classes) and an
/// additional distance constraint between the head and the leg. Returns the description index.
fn addTestRagdoll(b: *SceneBuilder, root: [3]Real, desc_value: SimRagdoll) Allocator.Error!u32 {
    const pelvis = try b.box(.{ 0.15, 0.1, 0.1 }, 0.05);
    const spine = try b.box(.{ 0.12, 0.12, 0.08 }, 0.05);
    const head = try b.sphere(0.1);
    const leg = try b.capsule(0.2, 0.06);
    const Part = struct { shape: u32, offset: [3]f32 };
    const parts = [_]Part{
        .{ .shape = pelvis, .offset = .{ 0, 0, 0 } },
        .{ .shape = spine, .offset = .{ 0, 0.3, 0 } },
        .{ .shape = head, .offset = .{ 0, 0.55, 0 } },
        .{ .shape = leg, .offset = .{ 0.08, -0.4, 0 } },
    };
    var bodies: [parts.len]u32 = undefined;
    for (parts, &bodies) |part, *body_desc|
        body_desc.* = try b.addBodyDesc(.{ .position = gridPositionOffset(root, part.offset), .shape = part.shape, .object_layer = Layers.moving, .linear_damping = 0.2, .angular_damping = 0.4 });
    const spine_joint = try b.addConstraint(swingTwistDesc(invalid_handle, invalid_handle, gridPositionOffset(root, .{ 0, 0.14, 0 }), .{ 0, 1, 0 }, .{ 1, 0, 0 }, .{ 0.4, 0.3, -0.3, 0.3 }, 0, 0.5));
    const neck_joint = try b.addConstraint(swingTwistDesc(invalid_handle, invalid_handle, gridPositionOffset(root, .{ 0, 0.44, 0 }), .{ 0, 1, 0 }, .{ 1, 0, 0 }, .{ 0.5, 0.5, -0.4, 0.4 }, 1, 0));
    var knee = Desc.hinge(invalid_handle, invalid_handle, gridPositionOffset(root, .{ 0.08, -0.12, 0 }), .{ 1, 0, 0 }, .{ 0, 1, 0 });
    knee.f[12] = -1.0; // Limits
    knee.f[13] = 1.0;
    const leg_joint = try b.addConstraint(knee);
    const additional = try b.addConstraint(Desc.distance(invalid_handle, invalid_handle, gridPositionOffset(root, .{ 0, 0.55, 0 }), gridPositionOffset(root, .{ 0.08, -0.4, 0 }), 0.8, 1.1));
    return b.addRagdoll(desc_value, &.{
        .{ .body = bodies[0] },
        .{ .parent = 0, .body = bodies[1], .to_parent = spine_joint },
        .{ .parent = 1, .body = bodies[2], .to_parent = neck_joint },
        .{ .parent = 0, .body = bodies[3], .to_parent = leg_joint },
    }, &.{.{ .body1 = 2, .body2 = 3, .constraint = additional }});
}

/// The local joint states of a pose of `addTestRagdoll` (translation relative to the parent, rotation): the bind pose
/// with the spine bent by `bend` around Z and the knee by `knee` around X
fn testRagdollPose(bend: f32, knee: f32) [4][7]f32 {
    const spine = axisAngle(.{ 0, 0, 1 }, bend);
    const leg = axisAngle(.{ 1, 0, 0 }, knee);
    return .{
        .{ 0, 0, 0, 0, 0, 0, 1 },
        .{ 0, 0.3, 0, spine[0], spine[1], spine[2], spine[3] },
        .{ 0, 0.25, 0, 0, 0, 0, 1 },
        .{ 0.08, -0.4, 0, leg[0], leg[1], leg[2], leg[3] },
    };
}

/// Every Ragdoll operation: three ragdolls of two descriptions (the first prepared with Stabilize, the constraint
/// priorities and DisableParentChildCollisions of a pose, its settings shared by two ragdolls; the second without
/// preparation and DisableParentChildCollisions without pose) fall on a floor; poses set (SkeletonPose and matrices),
/// driven by kinematics and motors (with velocity), the velocity setters and impulses with and without locking, the
/// group ID, the settings hashes with and without shapes, one ragdoll removed, added again and destroyed
fn buildRagdollOps(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 90;
    _ = try addFloor(b, 20, 0);
    const prepared = try addTestRagdoll(b, .{ 0, 1.2, 0 }, .{ .stabilize = 1, .calculate_constraint_priorities = 1, .base_priority = 10, .disable_parent_child_collisions = 2, .pose = 0, .min_separation_distance = 0.05, .collision_group = 1, .user_data = 11 });
    const plain = try addTestRagdoll(b, .{ 0, 1.2, 0 }, .{ .disable_parent_child_collisions = 1, .collision_group = 2, .user_data = 22 });
    const bind = try b.addRagdollPose(.{ 0, 1.2, 0 }, &testRagdollPose(0, 0));
    const bent = try b.addRagdollPose(.{ 2, 1.2, 0 }, &testRagdollPose(0.3, -0.6));
    const left = try b.addRagdollPose(.{ -2, 1.5, 0 }, &testRagdollPose(-0.2, 0.4));
    const bent_more = try b.addRagdollPose(.{ 2, 1.2, 0 }, &testRagdollPose(0.4, -0.9));

    const rag0 = try b.createRagdoll(prepared);
    try b.objectOp(.ragdoll_add_to_physics_system, rag0, .{});
    try b.expectRagdollActive(rag0, true);
    const rag1 = try b.createRagdoll(prepared); // Shares the settings of rag0
    try b.objectOp(.ragdoll_set_pose_matrices, rag1, .{ .i = &.{bent} });
    try b.objectOp(.ragdoll_add_to_physics_system, rag1, .{ .activation = .dont_activate, .i = &no_lock });
    try b.expectRagdollActive(rag1, false);
    const rag2 = try b.createRagdoll(plain);
    try b.objectOp(.ragdoll_set_pose, rag2, .{ .i = &.{ left, 0, 0, 1 } });
    try b.objectOp(.ragdoll_add_to_physics_system, rag2, .{});
    try b.objectOp(.ragdoll_settings_hash, rag0, .{ .i = &.{ 1, 1 } });
    try b.objectOp(.ragdoll_settings_hash, rag2, .{ .i = &.{ 0, 0 } });

    b.at(3);
    try b.objectOp(.ragdoll_activate, rag1, .{ .i = &no_lock });
    try b.objectOp(.ragdoll_drive_to_pose_using_motors, rag1, .{ .i = &.{bent_more} });
    try b.objectOp(.ragdoll_set_linear_velocity, rag2, .{ .f = &.{ 0.5, 0, 0 } });
    b.at(6);
    try b.objectOp(.ragdoll_drive_to_pose_using_motors_velocity, rag0, .{ .i = &.{ bent, bind }, .f = &.{b.delta_time} });
    try b.objectOp(.ragdoll_add_linear_velocity, rag2, .{ .f = &.{ 0, 0, 0.3 }, .i = &no_lock });
    b.at(10);
    try b.objectOp(.ragdoll_set_linear_and_angular_velocity, rag0, .{ .f = &.{ 0, 1, 0, 0, 2, 0 } });
    try b.objectOp(.ragdoll_add_impulse, rag2, .{ .f = &.{ 0, 5, 0 } });
    try b.objectOp(.ragdoll_get_pose_hash, rag1, .{});
    b.at(15);
    try b.objectOp(.ragdoll_reset_warm_start, rag0, .{});
    try b.objectOp(.ragdoll_set_group_id, rag2, .{ .i = &.{ 5, 0, 0, 1 } });
    try b.objectOp(.ragdoll_is_active, rag2, .{ .i = &no_lock });
    b.at(20);
    try b.objectOp(.ragdoll_set_linear_velocity, rag1, .{ .f = &.{ 0, 0, -0.5 }, .i = &no_lock });
    try b.objectOp(.ragdoll_set_linear_and_angular_velocity, rag2, .{ .f = &.{ 0, 0, 0, 1, 0, 0 }, .i = &no_lock });
    b.at(25);
    try b.objectOp(.ragdoll_add_impulse, rag1, .{ .f = &.{ 2, 0, 0 }, .i = &no_lock });
    try b.objectOp(.ragdoll_add_linear_velocity, rag0, .{ .f = &.{ 0.2, 0, 0 } });
    b.at(30);
    try b.objectOp(.ragdoll_remove_from_physics_system, rag1, .{});
    try b.objectOp(.ragdoll_get_pose_hash, rag0, .{ .i = &no_lock });
    b.at(32);
    try b.objectOp(.ragdoll_set_pose, rag1, .{ .i = &.{bent} });
    try b.objectOp(.ragdoll_add_to_physics_system, rag1, .{});
    b.at(40);
    try b.objectOp(.ragdoll_remove_from_physics_system, rag1, .{ .i = &no_lock });
    b.at(41);
    try b.objectOp(.ragdoll_destroy, rag1, .{});
    b.at(45);
    // A ragdoll created later reuses the settings of the description
    const rag3 = try b.createRagdoll(prepared);
    try b.objectOp(.ragdoll_drive_to_pose_using_kinematics, rag3, .{ .i = &.{bent}, .f = &.{b.delta_time} });
    try b.objectOp(.ragdoll_add_to_physics_system, rag3, .{});
    b.at(46);
    try b.objectOp(.ragdoll_drive_to_pose_using_kinematics_matrices, rag3, .{ .i = &.{ bent_more, 0, 0, 1 }, .f = &.{b.delta_time} });
    b.at(60);
    try b.objectOp(.ragdoll_drive_to_pose_using_motors, rag3, .{ .i = &.{bind} });
}

// (append the Ragdoll scenes here)

// ---- Vehicles (Phase 7, Blocker.wheeled_vehicles / tracked_vehicles / motorcycles) ----

/// The vehicle body of the vehicle scenes: a box of `half_extent` with its center of mass lowered by its half height
/// (Jolt's VehicleConstraintTest) and the mass `mass`
fn addVehicleBody(b: *SceneBuilder, half_extent: [3]f32, mass: f32, position: [3]Real, rotation_value: [4]f32) Allocator.Error!u32 {
    const shape = try b.offsetCenterOfMass(try b.box(half_extent, 0.05), .{ 0, -half_extent[1], 0 });
    return b.body(.{ .position = position, .rotation = rotation_value, .shape = shape, .object_layer = Layers.moving, .override_mass_properties = 1, .mass = mass }, .activate);
}

/// A wheeled car (Jolt's VehicleConstraintTest: four wheels, front steering, a differential per axle that gets half of
/// the engine torque, two anti-roll bars), returns the vehicle handle. Every wheel touches the floor at y = 0 in the first
/// update when `position` is 0.85 above it (higher: the car starts in the air, see vehicle_wheeled_in_air).
fn addTestCar(b: *SceneBuilder, position: [3]Real, desc_value: SimVehicle) Allocator.Error!u32 {
    var v = desc_value;
    v.body = try addVehicleBody(b, .{ 0.9, 0.2, 2.0 }, 1500, position, identity_rotation);
    var wheels: [4]SimVehicleWheel = undefined;
    for (&wheels, 0..) |*w, i| {
        const front = i < 2;
        w.* = .{ .position = .{ if (i % 2 == 0) 0.9 else -0.9, -0.18, if (front) 1.4 else -1.4 }, .max_steer_angle = if (front) zolt.math.degreesToRadians(30.0) else 0, .max_hand_brake_torque = if (front) 0 else 4000 };
    }
    return b.createVehicle(try b.addVehicle(v, &wheels, &.{ .{ .left_wheel = 0, .right_wheel = 1, .engine_torque_ratio = 0.5 }, .{ .left_wheel = 2, .right_wheel = 3, .engine_torque_ratio = 0.5 } }, &.{ .{ .left_wheel = 0, .right_wheel = 1 }, .{ .left_wheel = 2, .right_wheel = 3 } }));
}

/// Every operation of VehicleConstraint, the wheeled controller and the collision testers: a car drives off, shifts
/// gears, steers through its pre-step callback, brakes and hand brakes over a floor with a bump; its tester is replaced
/// by a cast sphere, a cast cylinder and a ray again and gets filters and another layer; the custom friction and tire
/// impulse functions; the engine, transmission, differential, anti-roll bar and wheel setters, the torques of a wheel
/// and of the engine; gravity overrides; the steps between the collision tests; the Constraint interface (priority,
/// step overrides, user data, warm start, disabled for three updates while its body is active: OnStep runs, the
/// constraint is not solved); its removal and addition; a snapshot that is restored cross after the car drove on
fn buildVehicleWheeledOps(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 120;
    _ = try addFloor(b, 40, 0);
    _ = try addStaticBox(b, .{ 2.0, 0.05, 0.3 }, .{ 0, 0.0, 6 }, identity_rotation, 0); // A bump across the way
    const torque_curve = try b.addFloats(&.{ 0, 0.8, 0.5, 1.0, 1.0, 0.7 });
    const gears = try b.addFloats(&.{ 3.0, 2.0, 1.4, 1.0 });
    const reverse_gears = try b.addFloats(&.{-3.0});
    const car = try addTestCar(b, .{ 0, 0.85, 0 }, .{ .tester = .{ .object_layer = Layers.moving }, .callbacks = VehicleCallbacks.pre_step | VehicleCallbacks.post_collide | VehicleCallbacks.post_step, .max_pitch_roll_angle = zolt.math.degreesToRadians(60.0), .draw_constraint_size = 0.1, .user_data = 6 });
    const crate_shape = try b.box(.{ 0.3, 0.3, 0.3 }, 0.05);
    const crate = try b.body(.{ .position = .{ 3, 0.3, 4 }, .shape = crate_shape, .object_layer = Layers.moving }, .activate);

    b.at(1);
    try b.expectWheelContacts(car, 0b1111);
    try b.objectOp(.wheeled_set_driver_input, car, .{ .f = &.{ 1.0, 0, 0, 0 } });
    try b.objectOp(.vehicle_settings_hash, car, .{});
    try b.objectOp(.vehicle_controller_settings_hash, car, .{});
    b.at(5);
    try b.objectOp(.vehicle_set_callbacks, car, .{ .i = &.{VehicleCallbacks.pre_step | VehicleCallbacks.post_step | VehicleCallbacks.steer} });
    try b.objectOp(.vehicle_set_combine_friction, car, .{ .i = &.{@intFromEnum(VehicleCombineFriction.custom)} });
    try b.objectOp(.wheeled_set_tire_max_impulse_callback, car, .{ .i = &.{@intFromEnum(VehicleTireMaxImpulse.custom)} });
    b.at(10);
    try b.sysOp(.snapshot, .{ .index = 0 });
    try b.objectOp(.wheeled_set_brake_input, car, .{ .f = &.{0.2} });
    b.at(14);
    try b.objectOp(.wheeled_set_hand_brake_input, car, .{ .f = &.{0.5} });
    try b.objectOp(.wheeled_set_differential, car, .{ .i = &.{ 0, 0, 1 }, .f = &.{ 3.0, 0.6, 1.5, 0.5 } });
    try b.objectOp(.wheeled_set_differential_limited_slip_ratio, car, .{ .f = &.{1.6} });
    b.at(18);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Back to update 10 (the brakes are part of the controller state)
    b.at(20);
    try b.objectOp(.vehicle_set_collision_tester, car, .{ .i = &.{ @intFromEnum(VehicleTesterKind.cast_sphere), Layers.moving }, .f = &.{ 0, 1, 0, zolt.math.degreesToRadians(80.0), 0.3, 0.1 } });
    b.at(25);
    try b.objectOp(.vehicle_tester_set_object_layer, car, .{ .i = &.{Layers.moving2} });
    try b.objectOp(.wheeled_set_forward_input, car, .{ .f = &.{0.7} });
    try b.objectOp(.wheeled_set_right_input, car, .{ .f = &.{-0.3} });
    b.at(30);
    try b.objectOp(.vehicle_tester_set_filters, car, .{ .i = &.{ 1, Layers.moving }, .body = crate });
    b.at(35);
    try b.objectOp(.vehicle_set_collision_tester, car, .{ .i = &.{ @intFromEnum(VehicleTesterKind.cast_cylinder), Layers.moving }, .f = &.{ 0, 1, 0, 0, 0, 0.2 } });
    try b.objectOp(.vehicle_tester_set_filters, car, .{ .i = &.{ 1, Layers.moving } });
    b.at(40);
    try b.objectOp(.vehicle_tester_set_filters, car, .{ .i = &.{0} });
    b.at(42);
    try b.objectOp(.vehicle_set_collision_tester, car, .{ .i = &.{ @intFromEnum(VehicleTesterKind.ray), Layers.moving }, .f = &.{ 0, 1, 0, zolt.math.degreesToRadians(70.0), 0, 0 } });
    b.at(45);
    try b.objectOp(.vehicle_wheel_set_angular_velocity, car, .{ .i = &.{2}, .f = &.{5.0} });
    try b.objectOp(.vehicle_wheel_set_rotation_angle, car, .{ .i = &.{3}, .f = &.{1.0} });
    try b.objectOp(.vehicle_wheel_set_steer_angle, car, .{ .i = &.{0}, .f = &.{0.1} });
    try b.objectOp(.vehicle_set_anti_roll_bar, car, .{ .i = &.{ 1, 2, 3 }, .f = &.{2000} });
    b.at(47);
    try b.objectOp(.vehicle_set_constraint_priority, car, .{ .i = &.{7} });
    try b.objectOp(.vehicle_set_steps_override, car, .{ .i = &.{ 6, 3 } });
    try b.objectOp(.vehicle_set_user_data, car, .{ .i = &.{ 0x12345678, 9 } });
    b.at(50);
    try b.objectOp(.vehicle_engine_set_current_rpm, car, .{ .f = &.{3000} });
    try b.objectOp(.vehicle_engine_set_settings, car, .{ .f = &.{ 600, 900, 6500, 0.6, 0.25 } });
    try b.objectOp(.vehicle_engine_set_torque_curve, car, .{ .i = &.{ torque_curve, 3 } });
    b.at(52);
    try b.objectOp(.vehicle_wheel_wv_apply_torque, car, .{ .i = &.{2}, .f = &.{ 150, b.delta_time } });
    try b.objectOp(.vehicle_engine_apply_torque, car, .{ .f = &.{ 80, b.delta_time } });
    b.at(53);
    try b.objectOp(.vehicle_engine_apply_damping, car, .{ .f = &.{b.delta_time} });
    b.at(55);
    try b.objectOp(.vehicle_transmission_set_settings, car, .{ .i = &.{1}, .f = &.{ 0.4, 0.25, 0.4, 4500, 1800, 12 } });
    try b.objectOp(.vehicle_transmission_set_gear_ratios, car, .{ .i = &.{ gears, 4, reverse_gears, 1 } });
    try b.objectOp(.vehicle_transmission_set, car, .{ .i = &.{2}, .f = &.{0.8} });
    try b.objectOp(.vehicle_controller_settings_hash, car, .{});
    b.at(60);
    try b.objectOp(.vehicle_override_gravity, car, .{ .f = &.{ 0, -6, 0 } });
    try b.objectOp(.vehicle_set_max_pitch_roll_angle, car, .{ .f = &.{zolt.math.degreesToRadians(45.0)} });
    try b.objectOp(.vehicle_set_num_steps_between_collision_test_active, car, .{ .i = &.{2} });
    try b.objectOp(.vehicle_set_num_steps_between_collision_test_inactive, car, .{ .i = &.{3} });
    b.at(64);
    try b.objectOp(.vehicle_set_enabled, car, .{ .i = &.{0} });
    try b.expectVehicleConstraintState(car, 0, true);
    b.at(66);
    try b.expectVehicleConstraintState(car, 0, true); // Two updates later: OnStep ran, the callbacks were called
    try b.objectOp(.vehicle_reset_warm_start, car, .{});
    b.at(67);
    try b.objectOp(.vehicle_set_enabled, car, .{ .i = &.{1} });
    try b.expectVehicleConstraintState(car, ConstraintState.enabled | ConstraintState.active, true);
    try b.objectOp(.vehicle_set_steps_override, car, .{ .i = &.{ 0, 0 } });
    b.at(70);
    try b.objectOp(.vehicle_reset_gravity_override, car, .{});
    try b.objectOp(.vehicle_set_num_steps_between_collision_test_active, car, .{ .i = &.{1} });
    try b.objectOp(.vehicle_settings_hash, car, .{});
    b.at(75);
    try b.objectOp(.vehicle_set_callbacks, car, .{ .i = &.{0} });
    try b.objectOp(.vehicle_set_combine_friction, car, .{ .i = &.{@intFromEnum(VehicleCombineFriction.default)} });
    try b.objectOp(.wheeled_set_tire_max_impulse_callback, car, .{ .i = &.{@intFromEnum(VehicleTireMaxImpulse.default)} });
    b.at(80);
    try b.objectOp(.vehicle_remove, car, .{});
    b.at(85);
    try b.objectOp(.vehicle_add, car, .{});
    b.at(90);
    try b.objectOp(.vehicle_transmission_set_settings, car, .{ .i = &.{0}, .f = &.{ 0.5, 0.3, 0.5, 4000, 2000, 10 } });
    try b.objectOp(.wheeled_set_driver_input, car, .{ .f = &.{ -0.5, 0.4, 0, 0 } });
    b.at(105);
    try b.objectOp(.wheeled_set_driver_input, car, .{ .f = &.{ 0, 0, 1, 1 } });
}

/// A tank with two tracks of four wheels each (Jolt's TankTest, smaller), returns the vehicle handle: the wheels touch
/// the floor at y = 0 in the first update when `position` is 0.95 above it
fn addTestTank(b: *SceneBuilder, position: [3]Real, desc_value: SimVehicle) Allocator.Error!u32 {
    var v = desc_value;
    v.body = try addVehicleBody(b, .{ 1.2, 0.4, 2.0 }, 4000, position, identity_rotation);
    var wheels: [8]SimVehicleWheel = undefined;
    for (0..2) |t| {
        for (0..4) |i| {
            const z: f32 = 1.5 - @as(f32, @floatFromInt(i));
            wheels[4 * t + i] = .{ .kind = @intFromEnum(VehicleWheelKind.tv), .position = .{ if (t == 0) 1.2 else -1.2, -0.3, z }, .suspension_frequency = 1.0, .longitudinal_friction_tv = 4.0 - 0.5 * @as(f32, @floatFromInt(i)) };
        }
        v.tracks[t].wheels = try b.addUints(&.{ 4 * @as(u32, @intCast(t)), 4 * @as(u32, @intCast(t)) + 1, 4 * @as(u32, @intCast(t)) + 2, 4 * @as(u32, @intCast(t)) + 3 });
        v.tracks[t].num_wheels = 4;
        v.tracks[t].driven_wheel = 4 * @as(u32, @intCast(t)) + 3;
    }
    return b.createVehicle(try b.addVehicle(v, &wheels, &.{}, &.{}));
}

/// Every operation of the tracked controller: a tank drives off, turns with the track ratios (one negative: it turns on
/// the spot), brakes; a track and the engine / transmission are changed, the callbacks and the custom friction, a
/// snapshot restored cross
fn buildVehicleTrackedOps(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 100;
    _ = try addFloor(b, 40, 0);
    const tank = try addTestTank(b, .{ 0, 0.95, 0 }, blk: {
        var v = SimVehicle.defaults(.tracked);
        v.tester = .{ .object_layer = Layers.moving };
        v.max_pitch_roll_angle = zolt.math.degreesToRadians(60.0);
        v.callbacks = VehicleCallbacks.post_collide | VehicleCallbacks.post_step;
        v.combine_friction = @intFromEnum(VehicleCombineFriction.custom);
        break :blk v;
    });
    b.at(1);
    try b.expectWheelContacts(tank, 0xff);
    try b.objectOp(.tracked_set_driver_input, tank, .{ .f = &.{ 1.0, 1.0, 1.0, 0 } });
    try b.objectOp(.vehicle_controller_settings_hash, tank, .{});
    b.at(10);
    try b.objectOp(.tracked_set_left_ratio, tank, .{ .f = &.{0.5} });
    try b.objectOp(.vehicle_set_callbacks, tank, .{ .i = &.{VehicleCallbacks.pre_step} });
    b.at(15);
    try b.sysOp(.snapshot, .{ .index = 1 });
    b.at(20);
    try b.objectOp(.tracked_set_forward_input, tank, .{ .f = &.{0.8} });
    try b.objectOp(.tracked_set_brake_input, tank, .{ .f = &.{0.1} });
    b.at(25);
    try b.sysOp(.restore_cross, .{ .index = 1 });
    b.at(30);
    try b.objectOp(.tracked_set_track, tank, .{ .i = &.{0}, .f = &.{ 2.0, 12, 0.6, 16000, 5.5 } });
    try b.objectOp(.tracked_set_right_ratio, tank, .{ .f = &.{-0.5} });
    b.at(40);
    try b.objectOp(.vehicle_engine_set_current_rpm, tank, .{ .f = &.{1500} });
    try b.objectOp(.vehicle_transmission_set, tank, .{ .i = &.{1}, .f = &.{1.0} });
    try b.objectOp(.vehicle_set_combine_friction, tank, .{ .i = &.{@intFromEnum(VehicleCombineFriction.default)} });
    b.at(50);
    try b.objectOp(.tracked_set_driver_input, tank, .{ .f = &.{ 0.6, 1.0, 0.8, 0 } });
    b.at(70);
    try b.objectOp(.tracked_set_brake_input, tank, .{ .f = &.{1.0} });
    try b.objectOp(.tracked_set_forward_input, tank, .{ .f = &.{0} });
    try b.objectOp(.vehicle_settings_hash, tank, .{});
}

/// A motorcycle (Jolt's MotorcycleTest: a front wheel with a caster angle and an overridden suspension force point, a
/// back wheel, one differential to the back wheel), returns the vehicle handle: the wheels touch the floor at y = 0 in the
/// first update when `position` is 0.9 above it and `rotation_value` the identity
fn addTestMotorcycle(b: *SceneBuilder, position: [3]Real, rotation_value: [4]f32, desc_value: SimVehicle) Allocator.Error!u32 {
    var v = desc_value;
    v.body = try addVehicleBody(b, .{ 0.2, 0.3, 0.4 }, 240, position, rotation_value);
    const caster = zolt.math.degreesToRadians(30.0);
    const tan_caster: f32 = @floatCast(@tan(@as(f64, caster)));
    const len: f32 = @floatCast(@sqrt(1.0 + @as(f64, tan_caster) * tan_caster));
    const direction: [3]f32 = .{ 0, -1.0 / len, tan_caster / len };
    const front_position: [3]f32 = .{ 0, -0.27, 0.75 };
    const wheels = [_]SimVehicleWheel{
        .{ .position = front_position, .max_steer_angle = zolt.math.degreesToRadians(30.0), .suspension_direction = direction, .steering_axis = .{ 0, -direction[1], -direction[2] }, .radius = 0.31, .width = 0.05, .max_brake_torque = 500, .enable_suspension_force_point = 1, .suspension_force_point = .{ 0, front_position[1] + 0.3 * direction[1], front_position[2] + 0.3 * direction[2] } },
        .{ .position = .{ 0, -0.27, -0.75 }, .max_steer_angle = 0, .radius = 0.31, .width = 0.05, .suspension_frequency = 2.0, .max_brake_torque = 250 },
    };
    v.engine.max_torque = 150;
    v.engine.max_rpm = 10000;
    v.transmission.shift_down_rpm = 2000;
    v.transmission.shift_up_rpm = 8000;
    v.transmission.gear_ratios = try b.addFloats(&.{ 2.27, 1.63, 1.3, 1.09, 0.96, 0.88 });
    v.transmission.num_gear_ratios = 6;
    v.transmission.reverse_gear_ratios = try b.addFloats(&.{-4.0});
    v.transmission.num_reverse_gear_ratios = 1;
    v.transmission.clutch_strength = 2.0;
    return b.createVehicle(try b.addVehicle(v, &wheels, &.{.{ .left_wheel = -1, .right_wheel = 1, .differential_ratio = 1.93 * 40.0 / 16.0 }}, &.{}));
}

/// Every operation of the motorcycle controller: a motorcycle drives off and leans into a turn (the lean controller and
/// the lean steering limit switched off and on, every lean parameter changed; the integration coefficient stays 0:
/// Jolt's MotorcycleController::SaveState does not save mLeanSpringIntegratedDeltaAngle, the D12 sweeps and the cross
/// restores would see it), the wheeled operations on it, a snapshot restored cross
fn buildVehicleMotorcycleOps(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 100;
    _ = try addFloor(b, 40, 0);
    const bike = try addTestMotorcycle(b, .{ 0, 0.9, 0 }, identity_rotation, .{ .controller = @intFromEnum(VehicleControllerKind.motorcycle), .tester = .{ .object_layer = Layers.moving }, .max_pitch_roll_angle = zolt.math.degreesToRadians(60.0), .callbacks = VehicleCallbacks.pre_step | VehicleCallbacks.post_step, .tire_max_impulse = @intFromEnum(VehicleTireMaxImpulse.custom) });
    b.at(1);
    try b.expectWheelContacts(bike, 0b11);
    try b.objectOp(.wheeled_set_driver_input, bike, .{ .f = &.{ 1.0, 0, 0, 0 } });
    try b.objectOp(.vehicle_controller_settings_hash, bike, .{});
    b.at(10);
    try b.objectOp(.wheeled_set_right_input, bike, .{ .f = &.{0.3} });
    try b.objectOp(.motorcycle_set_lean_spring_constant, bike, .{ .f = &.{6000} });
    try b.objectOp(.motorcycle_set_lean_spring_damping, bike, .{ .f = &.{1200} });
    b.at(15);
    try b.sysOp(.snapshot, .{ .index = 2 });
    try b.objectOp(.motorcycle_enable_lean_steering_limit, bike, .{ .i = &.{0} });
    b.at(20);
    try b.objectOp(.motorcycle_set_lean_spring_integration_coefficient, bike, .{ .f = &.{0} });
    try b.objectOp(.motorcycle_set_lean_spring_integration_coefficient_decay, bike, .{ .f = &.{3.0} });
    try b.objectOp(.motorcycle_set_lean_smoothing_factor, bike, .{ .f = &.{0.7} });
    b.at(25);
    try b.sysOp(.restore_cross, .{ .index = 2 });
    b.at(30);
    try b.objectOp(.motorcycle_enable_lean_controller, bike, .{ .i = &.{0} });
    b.at(33);
    try b.objectOp(.motorcycle_enable_lean_controller, bike, .{ .i = &.{1} });
    try b.objectOp(.motorcycle_enable_lean_steering_limit, bike, .{ .i = &.{1} });
    b.at(40);
    try b.objectOp(.wheeled_set_right_input, bike, .{ .f = &.{-0.2} });
    try b.objectOp(.vehicle_settings_hash, bike, .{});
    b.at(70);
    try b.objectOp(.wheeled_set_driver_input, bike, .{ .f = &.{ 0, 0, 0.5, 0 } });
}

/// The lean spring integration of the motorcycle controller (MotorcycleController::PreCollide integrates the lean angle
/// error, PostCollide adds mLeanSpringIntegrationCoefficient times it to the lean impulse while both wheels touch the
/// ground and decays it otherwise): the motorcycle starts in the air rolled to the side, lands and leans into turns with
/// a non-zero integration coefficient that changes, the lean controller is switched off (which resets the integrated
/// angle) and on again. No D12 sweeps and no restore (`sweep_stride = 0`): MotorcycleController::SaveState does not
/// save mLeanSpringIntegratedDeltaAngle, so the restore after a failed update of a sweep would keep the value of the
/// failed update.
fn buildVehicleMotorcycleLeanIntegration(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 120;
    _ = try addFloor(b, 40, 0);
    const bike = try addTestMotorcycle(b, .{ 0, 1.2, 0 }, axisAngle(.{ 0, 0, 1 }, 0.15), .{
        .controller = @intFromEnum(VehicleControllerKind.motorcycle),
        .tester = .{ .object_layer = Layers.moving },
        .max_pitch_roll_angle = zolt.math.degreesToRadians(60.0),
        .callbacks = VehicleCallbacks.post_collide,
        .lean_spring_integration_coefficient = 1000,
        .lean_spring_integration_coefficient_decay = 2,
    });
    b.at(1);
    try b.expectWheelContacts(bike, 0); // In the air: the integration, the decay
    try b.objectOp(.wheeled_set_driver_input, bike, .{ .f = &.{ 0.6, 0, 0, 0 } });
    b.at(30);
    try b.expectWheelContacts(bike, 0b11);
    try b.objectOp(.wheeled_set_right_input, bike, .{ .f = &.{0.4} });
    b.at(60);
    try b.objectOp(.motorcycle_set_lean_spring_integration_coefficient, bike, .{ .f = &.{2500} });
    try b.objectOp(.wheeled_set_right_input, bike, .{ .f = &.{-0.3} });
    b.at(80);
    try b.objectOp(.motorcycle_enable_lean_controller, bike, .{ .i = &.{0} }); // Resets the integrated angle
    b.at(83);
    try b.objectOp(.motorcycle_enable_lean_controller, bike, .{ .i = &.{1} });
    try b.objectOp(.motorcycle_set_lean_spring_integration_coefficient_decay, bike, .{ .f = &.{6} });
    b.at(100);
    try b.objectOp(.wheeled_set_driver_input, bike, .{ .f = &.{ 0, 0, 0.6, 0 } });
}

/// A car that starts in the air (the foundation review M6): its wheels have no contact for the first updates, so
/// VehicleConstraint::SaveState writes the contact members of Wheel that Jolt never initialized (zero-filled by the
/// allocation hooks of the reference, zero-initialized by Zolt, see the rules in the file header); a snapshot in the air
/// restored cross, then it lands and drives off
fn buildVehicleWheeledInAir(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 60;
    _ = try addFloor(b, 40, 0);
    const car = try addTestCar(b, .{ 0, 1.45, 0 }, .{ .tester = .{ .object_layer = Layers.moving }, .callbacks = VehicleCallbacks.post_collide | VehicleCallbacks.post_step });
    b.at(1);
    try b.expectWheelContacts(car, 0);
    try b.objectOp(.wheeled_set_driver_input, car, .{ .f = &.{ 0.5, 0, 0, 0 } });
    b.at(3);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(6);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Back to update 3, still in the air
    try b.expectWheelContacts(car, 0);
    b.at(40);
    try b.expectWheelContacts(car, 0b1111);
}

// ---- The vehicle core group's scenes (wheeled vehicles) ----

/// The options of `addCar`
const CarOptions = struct {
    rotation: [4]f32 = identity_rotation,
    /// The differentials (default: one per axle with half of the engine torque, limited slip, like addTestCar)
    differentials: []const SimVehicleDifferential = &.{ .{ .left_wheel = 0, .right_wheel = 1, .engine_torque_ratio = 0.5 }, .{ .left_wheel = 2, .right_wheel = 3, .engine_torque_ratio = 0.5 } },
    /// The anti-roll bars (default: one per axle)
    anti_roll_bars: []const SimVehicleAntiRollBar = &.{ .{ .left_wheel = 0, .right_wheel = 1 }, .{ .left_wheel = 2, .right_wheel = 3 } },
    /// The longitudinal / lateral friction curves of the wheels ((x, y) points in the float pool, `invalid_handle`: the
    /// curves of WheelSettingsWV's constructor)
    longitudinal_friction: u32 = invalid_handle,
    num_longitudinal_friction: u32 = 0,
    lateral_friction: u32 = invalid_handle,
    num_lateral_friction: u32 = 0,
};

/// The car of addTestCar (wheel 0 front left, 1 front right, 2 rear left, 3 rear right) with other differentials,
/// anti-roll bars, friction curves or a rotation; returns the vehicle handle
fn addCar(b: *SceneBuilder, position: [3]Real, desc_value: SimVehicle, options: CarOptions) Allocator.Error!u32 {
    var v = desc_value;
    v.body = try addVehicleBody(b, .{ 0.9, 0.2, 2.0 }, 1500, position, options.rotation);
    var wheels: [4]SimVehicleWheel = undefined;
    for (&wheels, 0..) |*w, i| {
        const front = i < 2;
        w.* = .{
            .position = .{ if (i % 2 == 0) 0.9 else -0.9, -0.18, if (front) 1.4 else -1.4 },
            .max_steer_angle = if (front) zolt.math.degreesToRadians(30.0) else 0,
            .max_hand_brake_torque = if (front) 0 else 4000,
            .longitudinal_friction = options.longitudinal_friction,
            .num_longitudinal_friction = options.num_longitudinal_friction,
            .lateral_friction = options.lateral_friction,
            .num_lateral_friction = options.num_lateral_friction,
        };
    }
    return b.createVehicle(try b.addVehicle(v, &wheels, options.differentials, options.anti_roll_bars));
}

/// The three collision testers on three grounds: a car with the ray tester on a wavy mesh terrain, one with the cast
/// sphere tester on a height field terrain, one with the cast cylinder tester on a floor with a ramp (a rotated static
/// box) that it drives up and off. All three start a bit above their ground (they land), drive with other wheel friction
/// curves and steer left and right.
fn buildVehicleWheeledTerrains(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 140;
    const mesh = try addMeshTerrain(b, 12, 1.5, false);
    _ = try b.body(.{ .position = .{ -20, 0, 0 }, .shape = mesh, .motion_type = 0, .friction = 0.8, .object_layer = Layers.non_moving }, .dont_activate);
    const height_field = try addHeightFieldTerrain(b, 17, 1.0);
    _ = try b.body(.{ .shape = height_field, .motion_type = 0, .friction = 0.7, .object_layer = Layers.non_moving }, .dont_activate);
    _ = try addStaticBox(b, .{ 4, 0.5, 10 }, .{ 20, -0.5, 0 }, identity_rotation, 0);
    _ = try addStaticBox(b, .{ 3, 0.5, 3 }, .{ 20, -0.3, 4 }, axisAngle(.{ 1, 0, 0 }, -0.25), 0); // Rises from the floor at z = 3

    const longitudinal = try b.addFloats(&.{ 0, 0, 0.1, 1.1, 0.3, 0.9, 1.0, 0.7 });
    const lateral = try b.addFloats(&.{ 0, 0, 5, 1.1, 30, 0.8 });
    const ray = try addCar(b, .{ -20, 1.4, -3 }, .{ .tester = .{ .kind = @intFromEnum(VehicleTesterKind.ray), .object_layer = Layers.moving }, .callbacks = VehicleCallbacks.post_collide }, .{ .longitudinal_friction = longitudinal, .num_longitudinal_friction = 4 });
    const sphere = try addCar(b, .{ 0, 1.4, -3 }, .{ .tester = .{ .kind = @intFromEnum(VehicleTesterKind.cast_sphere), .object_layer = Layers.moving, .radius = 0.3 }, .callbacks = VehicleCallbacks.post_step }, .{ .lateral_friction = lateral, .num_lateral_friction = 3 });
    const cylinder = try addCar(b, .{ 20, 0.9, -5 }, .{ .tester = .{ .kind = @intFromEnum(VehicleTesterKind.cast_cylinder), .object_layer = Layers.moving, .convex_radius_fraction = 0.3 } }, .{});
    const cars = [_]u32{ ray, sphere, cylinder };

    b.at(1);
    for (cars) |car|
        try b.objectOp(.wheeled_set_driver_input, car, .{ .f = &.{ 1.0, 0, 0, 0 } });
    b.at(30);
    try b.expectWheelContacts(cylinder, 0b1111);
    b.at(50);
    for (cars) |car|
        try b.objectOp(.wheeled_set_right_input, car, .{ .f = &.{0.3} });
    b.at(80);
    for (cars) |car|
        try b.objectOp(.wheeled_set_right_input, car, .{ .f = &.{-0.3} });
    b.at(110);
    try b.objectOp(.wheeled_set_driver_input, ray, .{ .f = &.{ 0, 0, 1.0, 0 } });
    try b.objectOp(.wheeled_set_driver_input, sphere, .{ .f = &.{ 0, 0.5, 0, 1.0 } });
}

/// Driving with two collision steps per update: a four wheel drive car with limited slip differentials and the automatic
/// transmission shifts up under full throttle, brakes (shifts down), backs up and hand brakes in a turn; a car with the
/// manual transmission goes through its gears (with a slipping clutch), into reverse and neutral; two rear wheel drive
/// cars on split friction (the left wheels on ice), one with open differentials and one with a limited slip
/// differential, steer hard with stiff anti-roll bars. The step callbacks see the first and the last collision step.
fn buildVehicleWheeledDriving(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 240;
    b.collision_steps = 2;
    _ = try addStaticBox(b, .{ 18, 0.5, 30 }, .{ 0, -0.5, 0 }, identity_rotation, 0); // The lanes of the first two cars
    // The split friction lanes: ice on the left (x < lane), asphalt on the right
    const ice = try b.box(.{ 2, 0.5, 30 }, 0.05);
    const asphalt = try b.box(.{ 2, 0.5, 30 }, 0.05);
    for ([_]Real{ 24, 34 }) |x| {
        _ = try b.body(.{ .position = .{ x - 2, -0.5, 0 }, .shape = ice, .motion_type = 0, .friction = 0.05, .object_layer = Layers.non_moving }, .dont_activate);
        _ = try b.body(.{ .position = .{ x + 2, -0.5, 0 }, .shape = asphalt, .motion_type = 0, .friction = 1.0, .object_layer = Layers.non_moving }, .dont_activate);
    }

    var auto_desc: SimVehicle = .{ .tester = .{ .object_layer = Layers.moving }, .callbacks = VehicleCallbacks.pre_step | VehicleCallbacks.post_collide | VehicleCallbacks.post_step, .differential_limited_slip_ratio = 1.6 };
    auto_desc.engine.max_torque = 300;
    auto_desc.transmission.shift_up_rpm = 2000; // With the grippy tires the wheels slip less than 0.1: it shifts up
    auto_desc.transmission.shift_down_rpm = 1300;
    const grippy = try b.addFloats(&.{ 0, 0, 0.02, 1.5, 0.2, 1.2 });
    const auto = try addCar(b, .{ -8, 0.85, -20 }, auto_desc, .{
        .longitudinal_friction = grippy,
        .num_longitudinal_friction = 3,
        .differentials = &.{ .{ .left_wheel = 0, .right_wheel = 1, .engine_torque_ratio = 0.5, .limited_slip_ratio = 1.3 }, .{ .left_wheel = 2, .right_wheel = 3, .engine_torque_ratio = 0.5, .limited_slip_ratio = 1.3 } },
    });
    var manual_desc: SimVehicle = .{ .tester = .{ .object_layer = Layers.moving } };
    manual_desc.transmission.mode = 1;
    manual_desc.transmission.clutch_strength = 6;
    const manual = try addCar(b, .{ 8, 0.85, -20 }, manual_desc, .{});
    const rear_wheel_drive = [_]SimVehicleDifferential{.{ .left_wheel = 2, .right_wheel = 3, .limited_slip_ratio = flt_max }};
    const stiff_bars = [_]SimVehicleAntiRollBar{ .{ .left_wheel = 0, .right_wheel = 1, .stiffness = 6000 }, .{ .left_wheel = 2, .right_wheel = 3, .stiffness = 6000 } };
    const open = try addCar(b, .{ 24, 0.85, -20 }, .{ .tester = .{ .object_layer = Layers.moving }, .differential_limited_slip_ratio = flt_max }, .{ .differentials = &rear_wheel_drive, .anti_roll_bars = &stiff_bars });
    const limited_slip = try addCar(b, .{ 34, 0.85, -20 }, .{ .tester = .{ .object_layer = Layers.moving }, .differential_limited_slip_ratio = flt_max }, .{ .differentials = &.{.{ .left_wheel = 2, .right_wheel = 3, .limited_slip_ratio = 1.1 }}, .anti_roll_bars = &stiff_bars });

    b.at(1);
    for ([_]u32{ auto, manual, open, limited_slip }) |car|
        try b.expectWheelContacts(car, 0b1111);
    try b.objectOp(.wheeled_set_driver_input, auto, .{ .f = &.{ 1.0, 0, 0, 0 } });
    try b.objectOp(.vehicle_transmission_set, manual, .{ .i = &.{1}, .f = &.{1.0} });
    try b.objectOp(.wheeled_set_forward_input, manual, .{ .f = &.{1.0} });
    try b.objectOp(.wheeled_set_driver_input, open, .{ .f = &.{ 1.0, 0, 0, 0 } });
    try b.objectOp(.wheeled_set_driver_input, limited_slip, .{ .f = &.{ 1.0, 0, 0, 0 } });
    b.at(40);
    try b.objectOp(.vehicle_transmission_set, manual, .{ .i = &.{2}, .f = &.{0.4} }); // A slipping clutch
    b.at(50);
    try b.objectOp(.wheeled_set_right_input, open, .{ .f = &.{0.7} });
    try b.objectOp(.wheeled_set_right_input, limited_slip, .{ .f = &.{-0.7} });
    b.at(60);
    try b.objectOp(.vehicle_transmission_set, manual, .{ .i = &.{3}, .f = &.{1.0} });
    b.at(90);
    try b.objectOp(.vehicle_transmission_set, manual, .{ .i = &.{@bitCast(@as(i32, -1))}, .f = &.{1.0} }); // Reverse while rolling forward
    b.at(115);
    try b.objectOp(.vehicle_transmission_set, manual, .{ .i = &.{0}, .f = &.{0.0} }); // Neutral
    b.at(140);
    try b.objectOp(.wheeled_set_driver_input, open, .{ .f = &.{ 0, 0, 0.5, 1.0 } });
    b.at(170);
    try b.objectOp(.wheeled_set_driver_input, auto, .{ .f = &.{ 0, 0, 1.0, 0 } });
    b.at(200);
    try b.objectOp(.wheeled_set_driver_input, auto, .{ .f = &.{ -1.0, 0, 0, 0 } });
    b.at(220);
    try b.objectOp(.wheeled_set_driver_input, auto, .{ .f = &.{ 0.6, -0.8, 0, 1.0 } });
}

/// Islands with other bodies: a car drives over loose planks that are asleep (VehicleConstraint::BuildIslands wakes them
/// and links them into its island) into a wall of 80 boxes, which it pushes over (a large island that the
/// LargeIslandSplitter splits, the vehicle constraint in the non parallel split), then brakes; the boxes come to rest
fn buildVehicleWheeledPushingBoxes(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 320;
    b.max_contact_constraints = 4096;
    _ = try addFloor(b, 40, 0);
    const car = try addCar(b, .{ 0, 0.85, -7 }, .{ .tester = .{ .object_layer = Layers.moving }, .callbacks = VehicleCallbacks.post_step }, .{});
    const plank = try b.box(.{ 1.2, 0.04, 0.25 }, 0.02);
    for (0..4) |i|
        _ = try b.body(.{ .position = .{ 0, 0.04, -3.5 + 0.8 * real(i) }, .shape = plank, .object_layer = Layers.moving, .friction = 0.6 }, .dont_activate);
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    for (0..8) |y| {
        for (0..10) |x|
            _ = try b.body(.{ .position = .{ real(x) * 0.5 - 2.25, 0.25 + 0.5 * real(y), 3.25 }, .shape = box, .object_layer = Layers.moving, .friction = 0.5 }, .dont_activate);
    }

    b.at(1);
    try b.objectOp(.wheeled_set_driver_input, car, .{ .f = &.{ 1.0, 0, 0, 0 } });
    b.at(200);
    try b.objectOp(.wheeled_set_driver_input, car, .{ .f = &.{ 0, 0, 1.0, 1.0 } });
}

/// Sleeping and waking: a car without input comes to rest and falls asleep (the collision tests of an inactive vehicle
/// every third step, the constraint inactive), is woken up by ActivateBody and drives off; a snapshot while it drives is
/// restored cross; it is disabled for a few updates while it drives (OnStep runs, it is not solved), and the inactive
/// collision steps are 0 (no collision tests) while it sleeps again at the end
fn buildVehicleWheeledSleepAndWake(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 300;
    _ = try addFloor(b, 40, 0);
    const car = try addCar(b, .{ 0, 0.9, 0 }, .{ .tester = .{ .object_layer = Layers.moving }, .callbacks = VehicleCallbacks.post_collide }, .{});
    const body = b.num_handles - 1; // The vehicle body (the last body handle)

    b.at(1);
    try b.objectOp(.vehicle_set_num_steps_between_collision_test_inactive, car, .{ .i = &.{3} });
    b.at(90);
    try b.expectSleeping(body);
    try b.expectVehicleConstraintState(car, ConstraintState.inactive, false);
    b.at(95);
    try b.op(.activate_body, body, .{});
    try b.objectOp(.wheeled_set_driver_input, car, .{ .f = &.{ 0.8, 0.2, 0, 0 } });
    b.at(100);
    try b.expectVehicleConstraintState(car, ConstraintState.enabled | ConstraintState.active, true);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(110);
    try b.objectOp(.vehicle_set_enabled, car, .{ .i = &.{0} });
    b.at(114);
    try b.objectOp(.vehicle_set_enabled, car, .{ .i = &.{1} });
    b.at(120);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Back to update 100
    b.at(130);
    try b.objectOp(.wheeled_set_driver_input, car, .{ .f = &.{ 0, 0, 1.0, 0 } });
    try b.objectOp(.vehicle_set_num_steps_between_collision_test_inactive, car, .{ .i = &.{0} });
    b.at(299);
    try b.expectSleeping(body);
}

/// Far from the origin (the positions are Real: in double precision the wheel contacts keep their precision): the three
/// testers on a floor and a ramp, steering, and a snapshot restored cross
fn buildVehicleWheeledFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 100;
    const o: [3]Real = .{ 5000, 200, -12000 };
    _ = try addStaticBox(b, .{ 30, 0.5, 30 }, .{ o[0], o[1] - 0.5, o[2] }, identity_rotation, 0);
    _ = try addStaticBox(b, .{ 3, 0.5, 3 }, .{ o[0] + 8, o[1] - 0.3, o[2] + 2 }, axisAngle(.{ 1, 0, 0 }, -0.2), 0);
    var cars: [3]u32 = undefined;
    for (&cars, 0..) |*car, i|
        car.* = try addCar(b, .{ o[0] - 8 + 8 * real(i), o[1] + 0.85, o[2] - 6 }, .{ .tester = .{ .kind = @intCast(i), .object_layer = Layers.moving } }, .{});

    b.at(1);
    for (cars) |car|
        try b.objectOp(.wheeled_set_driver_input, car, .{ .f = &.{ 1.0, 0, 0, 0 } });
    b.at(30);
    for (cars) |car|
        try b.expectWheelContacts(car, 0b1111);
    try b.sysOp(.snapshot, .{ .index = 0 });
    try b.objectOp(.wheeled_set_right_input, cars[0], .{ .f = &.{0.4} });
    b.at(45);
    try b.sysOp(.restore_cross, .{ .index = 0 });
    b.at(60);
    try b.objectOp(.wheeled_set_right_input, cars[1], .{ .f = &.{-0.5} });
    try b.objectOp(.wheeled_set_hand_brake_input, cars[2], .{ .f = &.{0.6} });
}

// ---- The vehicle variants group's scenes (tracked vehicles and motorcycles) ----

/// The options of `addTank`
const TankOptions = struct {
    rotation: [4]f32 = identity_rotation,
    /// The half extent and the mass of the body
    half_extent: [3]f32 = .{ 1.2, 0.4, 2.0 },
    mass: f32 = 4000,
    /// The wheels of each track, from the front (z = (n - 1) / 2 * spacing) to the back; wheel i of track t is wheel
    /// t * wheels_per_track + i of the vehicle (the left track is at x = half_extent[0])
    wheels_per_track: u32 = 4,
    spacing: f32 = 1.0,
    /// The driven wheel of each track (an index into its wheels)
    driven: [2]u32 = .{ 3, 3 },
    /// The radius of the first and the last wheel of a track (a sprocket and an idler), the others have the default 0.3
    end_radius: f32 = 0.3,
    /// WheelSettingsTV's frictions of every wheel
    longitudinal_friction: f32 = 4.0,
    lateral_friction: f32 = 2.0,
};

/// A tank of the vehicle variants group's scenes, returns the vehicle handle (the wheels touch the floor at y = 0 in the
/// first update when `position` is 0.95 above it). The tracks of `desc_value` get their wheels and driven wheels here,
/// their other members are the description's.
fn addTank(b: *SceneBuilder, position: [3]Real, desc_value: SimVehicle, options: TankOptions) Allocator.Error!u32 {
    var v = desc_value;
    v.body = try addVehicleBody(b, options.half_extent, options.mass, position, options.rotation);
    var wheels: [2 * 8]SimVehicleWheel = undefined;
    const n = options.wheels_per_track;
    std.debug.assert(n <= 8);
    for (0..2) |t| {
        var indices: [8]u32 = undefined;
        for (0..n) |i| {
            const end = i == 0 or i == n - 1;
            const z = (0.5 * @as(f32, @floatFromInt(n - 1)) - @as(f32, @floatFromInt(i))) * options.spacing;
            wheels[t * n + i] = .{
                .kind = @intFromEnum(VehicleWheelKind.tv),
                .position = .{ if (t == 0) options.half_extent[0] else -options.half_extent[0], -0.3, z },
                .suspension_frequency = 1.0,
                .radius = if (end) options.end_radius else 0.3,
                .longitudinal_friction_tv = options.longitudinal_friction,
                .lateral_friction_tv = options.lateral_friction,
            };
            indices[i] = @intCast(t * n + i);
        }
        v.tracks[t].wheels = try b.addUints(indices[0..n]);
        v.tracks[t].num_wheels = n;
        v.tracks[t].driven_wheel = indices[options.driven[t]];
    }
    return b.createVehicle(try b.addVehicle(v, wheels[0 .. 2 * n], &.{}, &.{}));
}

/// The description of a tank of the variants' scenes: Jolt's tracked defaults, the tester in the moving layer
fn tankDesc(tester: VehicleTesterKind) SimVehicle {
    var v = SimVehicle.defaults(.tracked);
    v.tester = .{ .kind = @intFromEnum(tester), .object_layer = Layers.moving };
    v.max_pitch_roll_angle = zolt.math.degreesToRadians(60.0);
    return v;
}

/// Tanks on terrain: one with the cast cylinder tester on a wavy mesh, one with the cast sphere tester on a height field.
/// They start in the air with the gas pressed, land and drive off, steer with the track ratios (one track slower: the sync of the tracks keeps the ratio of their angular
/// velocities, SyncLeftRightTracks' same direction branch), turn on the spot (opposite ratios: the other branch, the
/// transmission does not shift up while the product of the ratios is negative), drive on, brake until the tracks lock
/// (the brake impulse spread over the wheels with a contact) and back up with the brake partly pressed
fn buildVehicleTrackedTerrains(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 170;
    const mesh = try addMeshTerrain(b, 20, 1.5, false);
    _ = try b.body(.{ .position = .{ -25, 0, 0 }, .shape = mesh, .motion_type = 0, .friction = 0.8, .object_layer = Layers.non_moving }, .dont_activate);
    const height_field = try addHeightFieldTerrain(b, 33, 1.0);
    _ = try b.body(.{ .position = .{ 25, 0, 0 }, .shape = height_field, .motion_type = 0, .friction = 0.6, .object_layer = Layers.non_moving }, .dont_activate);

    var on_mesh_desc = tankDesc(.cast_cylinder);
    on_mesh_desc.tester.convex_radius_fraction = 0.2;
    on_mesh_desc.callbacks = VehicleCallbacks.post_collide;
    const on_mesh = try addTank(b, .{ -25, 1.8, -6 }, on_mesh_desc, .{ .end_radius = 0.35 });
    var on_height_field_desc = tankDesc(.cast_sphere);
    on_height_field_desc.tester.radius = 0.3;
    on_height_field_desc.callbacks = VehicleCallbacks.post_step;
    on_height_field_desc.combine_friction = @intFromEnum(VehicleCombineFriction.custom);
    const on_height_field = try addTank(b, .{ 25, 1.8, -6 }, on_height_field_desc, .{});
    const tanks = [_]u32{ on_mesh, on_height_field };

    b.at(1);
    for (tanks) |tank| {
        try b.expectWheelContacts(tank, 0); // In the air: no friction, the engine revs against the tracks alone
        try b.objectOp(.tracked_set_driver_input, tank, .{ .f = &.{ 1.0, 1.0, 1.0, 0 } });
    }
    b.at(45);
    try b.objectOp(.tracked_set_right_ratio, on_mesh, .{ .f = &.{0.6} }); // Steer right
    try b.objectOp(.tracked_set_left_ratio, on_height_field, .{ .f = &.{0.5} }); // Steer left
    b.at(75);
    try b.objectOp(.tracked_set_driver_input, on_mesh, .{ .f = &.{ 1.0, -1.0, 1.0, 0 } }); // Turn left on the spot
    try b.objectOp(.tracked_set_driver_input, on_height_field, .{ .f = &.{ 0.8, 1.0, -0.8, 0 } }); // Turn right on the spot
    b.at(105);
    for (tanks) |tank|
        try b.objectOp(.tracked_set_driver_input, tank, .{ .f = &.{ 0.6, 1.0, 1.0, 0 } });
    b.at(125);
    for (tanks) |tank|
        try b.objectOp(.tracked_set_driver_input, tank, .{ .f = &.{ 0, 1.0, 1.0, 1.0 } }); // Lock the tracks
    b.at(150);
    try b.objectOp(.tracked_set_driver_input, on_mesh, .{ .f = &.{ -0.7, 1.0, 1.0, 0.05 } }); // Reverse against the brakes
    try b.objectOp(.tracked_set_driver_input, on_height_field, .{ .f = &.{ -1.0, 1.0, 0.7, 0 } });
}

/// Tanks with other track layouts, two collision steps per update: five wheels per track driven by the front wheel with
/// bigger end wheels, asymmetric tracks (inertia, damping, differential ratio) and a manual transmission (the engine revs
/// in neutral with the gas pressed, then gears with a slipping clutch and reverse); six wheels per track driven by a
/// middle wheel (a different one on each side), automatic transmission into reverse; three wheels per track on a tank
/// that faces +x, with the step callbacks and a gravity override, whose tracks are changed while it drives
fn buildVehicleTrackedLayouts(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 150;
    b.collision_steps = 2;
    _ = try addFloor(b, 50, 0);

    var manual_desc = tankDesc(.ray);
    manual_desc.transmission.mode = 1;
    manual_desc.transmission.clutch_strength = 4.0;
    manual_desc.tracks[0].inertia = 8.0;
    manual_desc.tracks[1].inertia = 14.0;
    manual_desc.tracks[0].differential_ratio = 6.0;
    manual_desc.tracks[1].differential_ratio = 5.0;
    manual_desc.tracks[1].angular_damping = 1.5;
    const manual = try addTank(b, .{ -12, 0.95, -10 }, manual_desc, .{ .wheels_per_track = 5, .spacing = 0.9, .driven = .{ 0, 0 }, .end_radius = 0.35, .longitudinal_friction = 3.0, .lateral_friction = 1.5 });

    var middle_desc = tankDesc(.cast_sphere);
    middle_desc.tester.radius = 0.25;
    middle_desc.engine.max_torque = 650.0;
    middle_desc.tracks[0].max_brake_torque = 9000.0;
    middle_desc.combine_friction = @intFromEnum(VehicleCombineFriction.custom);
    const middle = try addTank(b, .{ 0, 0.95, -10 }, middle_desc, .{ .half_extent = .{ 1.2, 0.4, 2.4 }, .wheels_per_track = 6, .spacing = 0.8, .driven = .{ 2, 3 } });

    var sideways_desc = tankDesc(.ray);
    sideways_desc.callbacks = VehicleCallbacks.pre_step | VehicleCallbacks.post_collide | VehicleCallbacks.post_step;
    const sideways = try addTank(b, .{ 12, 0.95, -6 }, sideways_desc, .{ .rotation = axisAngle(.{ 0, 1, 0 }, std.math.pi / 2.0), .half_extent = .{ 1.0, 0.4, 1.4 }, .mass = 2500, .wheels_per_track = 3, .spacing = 1.1, .driven = .{ 2, 2 } });

    b.at(1);
    try b.expectWheelContacts(manual, 0x3ff);
    try b.expectWheelContacts(middle, 0xfff);
    try b.expectWheelContacts(sideways, 0x3f);
    try b.objectOp(.tracked_set_forward_input, manual, .{ .f = &.{1.0} }); // Neutral: the engine revs
    try b.objectOp(.tracked_set_driver_input, middle, .{ .f = &.{ 1.0, 1.0, 1.0, 0 } });
    try b.objectOp(.tracked_set_driver_input, sideways, .{ .f = &.{ 0.7, 1.0, 1.0, 0 } });
    b.at(10);
    try b.objectOp(.vehicle_transmission_set, manual, .{ .i = &.{1}, .f = &.{1.0} });
    b.at(30);
    try b.objectOp(.vehicle_transmission_set, manual, .{ .i = &.{2}, .f = &.{0.4} }); // A slipping clutch
    try b.objectOp(.tracked_set_track, sideways, .{ .i = &.{1}, .f = &.{ 3.0, 6.0, 0.2, 5000, 7.0 } });
    try b.objectOp(.vehicle_override_gravity, sideways, .{ .f = &.{ 0, -7, 0 } });
    b.at(50);
    try b.objectOp(.tracked_set_driver_input, middle, .{ .f = &.{ 1.0, 0.7, 1.0, 0 } });
    try b.objectOp(.vehicle_reset_gravity_override, sideways, .{});
    b.at(60);
    try b.objectOp(.vehicle_transmission_set, manual, .{ .i = &.{3}, .f = &.{1.0} });
    try b.objectOp(.tracked_set_left_ratio, manual, .{ .f = &.{0.8} });
    b.at(80);
    try b.objectOp(.tracked_set_driver_input, middle, .{ .f = &.{ -1.0, 1.0, 1.0, 0 } }); // Reverse while rolling forward
    try b.objectOp(.tracked_set_driver_input, sideways, .{ .f = &.{ 0.7, -0.5, 1.0, 0 } });
    b.at(100);
    try b.objectOp(.vehicle_transmission_set, manual, .{ .i = &.{@bitCast(@as(i32, -1))}, .f = &.{1.0} }); // Reverse
    try b.objectOp(.tracked_set_brake_input, sideways, .{ .f = &.{0.4} });
    b.at(125);
    try b.objectOp(.vehicle_transmission_set, manual, .{ .i = &.{0}, .f = &.{0.0} }); // Neutral
    try b.objectOp(.tracked_set_brake_input, manual, .{ .f = &.{1.0} });
    try b.objectOp(.vehicle_controller_settings_hash, middle, .{});
}

/// A tank pushes a wall of 64 boxes that are asleep (the wheels and the body wake them, the vehicle constraint links the
/// boxes under its wheels into its island, a large island that the LargeIslandSplitter splits), steers while it pushes,
/// then brakes; the boxes come to rest and fall asleep
fn buildVehicleTrackedPushingBoxes(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 300;
    b.max_contact_constraints = 4096;
    _ = try addFloor(b, 40, 0);
    var desc = tankDesc(.ray);
    desc.callbacks = VehicleCallbacks.post_step;
    const tank = try addTank(b, .{ 0, 0.95, -8 }, desc, .{});
    const box = try b.box(.{ 0.25, 0.25, 0.25 }, 0.05);
    for (0..8) |y| {
        for (0..8) |x|
            _ = try b.body(.{ .position = .{ real(x) * 0.5 - 1.75, 0.25 + 0.5 * real(y), 1.0 }, .shape = box, .object_layer = Layers.moving, .friction = 0.5 }, .dont_activate);
    }
    const plank = try b.box(.{ 1.5, 0.05, 0.3 }, 0.02);
    _ = try b.body(.{ .position = .{ 0, 0.05, -3.5 }, .shape = plank, .object_layer = Layers.moving, .friction = 0.6 }, .dont_activate); // Under the wheels

    b.at(1);
    try b.objectOp(.tracked_set_driver_input, tank, .{ .f = &.{ 0.8, 1.0, 1.0, 0 } });
    b.at(120);
    try b.objectOp(.tracked_set_right_ratio, tank, .{ .f = &.{0.6} });
    b.at(170);
    try b.objectOp(.tracked_set_driver_input, tank, .{ .f = &.{ 0, 1.0, 1.0, 1.0 } });
}

/// Sleeping and waking: a tank without input comes to rest and falls asleep (the engine idles, the transmission shifts
/// to neutral; the collision tests of the inactive vehicle every third step, the constraint inactive), is woken up by
/// ActivateBody and drives off turning; a snapshot while it drives is restored cross; it is disabled for a few updates
/// while it drives (OnStep runs, it is not solved); it brakes and falls asleep again without collision tests (0 inactive
/// steps)
fn buildVehicleTrackedSleepAndWake(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 300;
    _ = try addFloor(b, 40, 0);
    var desc = tankDesc(.cast_sphere);
    desc.callbacks = VehicleCallbacks.post_collide;
    const tank = try addTank(b, .{ 0, 1.0, 0 }, desc, .{});
    const body = b.num_handles - 1; // The vehicle body (the last body handle)

    b.at(1);
    try b.objectOp(.vehicle_set_num_steps_between_collision_test_inactive, tank, .{ .i = &.{3} });
    b.at(90);
    try b.expectSleeping(body);
    try b.expectVehicleConstraintState(tank, ConstraintState.inactive, false);
    b.at(95);
    try b.op(.activate_body, body, .{});
    try b.objectOp(.tracked_set_driver_input, tank, .{ .f = &.{ 0.8, 1.0, 0.7, 0 } });
    b.at(100);
    try b.expectVehicleConstraintState(tank, ConstraintState.enabled | ConstraintState.active, true);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(110);
    try b.objectOp(.vehicle_set_enabled, tank, .{ .i = &.{0} });
    b.at(114);
    try b.objectOp(.vehicle_set_enabled, tank, .{ .i = &.{1} });
    b.at(120);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Back to update 100
    b.at(140);
    try b.objectOp(.tracked_set_driver_input, tank, .{ .f = &.{ 0, 1.0, 1.0, 1.0 } });
    try b.objectOp(.vehicle_set_num_steps_between_collision_test_inactive, tank, .{ .i = &.{0} });
    b.at(299);
    try b.expectSleeping(body);
}

/// Far from the origin (the positions are Real: in double precision the wheel contacts keep their precision): two tanks
/// with the ray and the cast cylinder tester on a floor with a ramp, steering, turning on the spot, a snapshot restored
/// cross
fn buildVehicleTrackedFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 110;
    const o: [3]Real = .{ -7000, 150, 9000 };
    _ = try addStaticBox(b, .{ 30, 0.5, 30 }, .{ o[0], o[1] - 0.5, o[2] }, identity_rotation, 0);
    _ = try addStaticBox(b, .{ 3, 0.5, 4 }, .{ o[0] - 6, o[1] - 0.3, o[2] + 3 }, axisAngle(.{ 1, 0, 0 }, -0.2), 0);
    const ray = try addTank(b, .{ o[0] - 6, o[1] + 0.95, o[2] - 6 }, tankDesc(.ray), .{});
    const cylinder = try addTank(b, .{ o[0] + 6, o[1] + 0.95, o[2] - 6 }, tankDesc(.cast_cylinder), .{ .end_radius = 0.33 });

    b.at(1);
    try b.objectOp(.tracked_set_driver_input, ray, .{ .f = &.{ 1.0, 1.0, 1.0, 0 } });
    try b.objectOp(.tracked_set_driver_input, cylinder, .{ .f = &.{ 0.8, 1.0, 0.6, 0 } });
    b.at(30);
    try b.expectWheelContacts(ray, 0xff);
    try b.expectWheelContacts(cylinder, 0xff);
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(50);
    try b.objectOp(.tracked_set_driver_input, cylinder, .{ .f = &.{ 0.8, 1.0, -1.0, 0 } });
    b.at(60);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Back to update 30
    b.at(90);
    try b.objectOp(.tracked_set_driver_input, ray, .{ .f = &.{ 0, 1.0, 1.0, 1.0 } });
}

/// The description of a motorcycle of the variants' scenes: the motorcycle controller with Jolt's defaults (the lean
/// spring integration coefficient stays 0 in scenes with D12 sweeps or restores: MotorcycleController::SaveState does
/// not save the integrated angle), the tester in the moving layer
fn motorcycleDesc(tester: VehicleTesterKind) SimVehicle {
    var v = SimVehicle.defaults(.motorcycle);
    v.tester = .{ .kind = @intFromEnum(tester), .object_layer = Layers.moving, .radius = 0.05 };
    v.max_pitch_roll_angle = zolt.math.degreesToRadians(60.0);
    return v;
}

/// Motorcycles leaning into turns at different speeds with different lean springs: the default lean spring at half
/// throttle; a small max lean angle (the target lean is clamped) with a softer, less damped spring and less smoothing at
/// full throttle; a strongly smoothed spring whose lean steering limit is switched off while it is fast (the front wheel
/// steers its full angle) and on again, and whose lean controller is off for a few updates (the target lean is the world
/// up, the integrated angle is reset). They turn right and left at speed, then roll out.
fn buildVehicleMotorcycleTurns(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 200;
    _ = try addFloor(b, 70, 0);
    const default_bike = try addTestMotorcycle(b, .{ -12, 0.9, -40 }, identity_rotation, motorcycleDesc(.cast_sphere));
    var clamped_desc = motorcycleDesc(.ray);
    clamped_desc.max_lean_angle = zolt.math.degreesToRadians(8.0);
    clamped_desc.lean_spring_constant = 3000.0;
    clamped_desc.lean_spring_damping = 600.0;
    clamped_desc.lean_smoothing_factor = 0.5;
    clamped_desc.callbacks = VehicleCallbacks.post_step;
    const clamped = try addTestMotorcycle(b, .{ 0, 0.9, -40 }, identity_rotation, clamped_desc);
    var smooth_desc = motorcycleDesc(.cast_cylinder);
    smooth_desc.lean_smoothing_factor = 0.95;
    smooth_desc.lean_spring_constant = 7000.0;
    smooth_desc.tire_max_impulse = @intFromEnum(VehicleTireMaxImpulse.custom);
    const smooth = try addTestMotorcycle(b, .{ 12, 0.9, -40 }, identity_rotation, smooth_desc);

    b.at(1);
    try b.expectWheelContacts(default_bike, 0b11);
    try b.objectOp(.wheeled_set_driver_input, default_bike, .{ .f = &.{ 0.5, 0, 0, 0 } });
    try b.objectOp(.wheeled_set_driver_input, clamped, .{ .f = &.{ 1.0, 0, 0, 0 } });
    try b.objectOp(.wheeled_set_driver_input, smooth, .{ .f = &.{ 0.8, 0, 0, 0 } });
    b.at(60);
    try b.objectOp(.wheeled_set_right_input, default_bike, .{ .f = &.{0.4} });
    try b.objectOp(.wheeled_set_right_input, clamped, .{ .f = &.{0.6} });
    try b.objectOp(.motorcycle_enable_lean_steering_limit, smooth, .{ .i = &.{0} });
    try b.objectOp(.wheeled_set_right_input, smooth, .{ .f = &.{0.15} });
    b.at(75);
    try b.objectOp(.motorcycle_enable_lean_steering_limit, smooth, .{ .i = &.{1} });
    try b.objectOp(.wheeled_set_right_input, smooth, .{ .f = &.{0.5} });
    b.at(100);
    try b.objectOp(.motorcycle_enable_lean_controller, smooth, .{ .i = &.{0} });
    b.at(104);
    try b.objectOp(.motorcycle_enable_lean_controller, smooth, .{ .i = &.{1} });
    b.at(120);
    try b.objectOp(.wheeled_set_right_input, default_bike, .{ .f = &.{-0.4} });
    try b.objectOp(.wheeled_set_right_input, clamped, .{ .f = &.{-0.5} });
    try b.objectOp(.wheeled_set_right_input, smooth, .{ .f = &.{-0.3} });
    b.at(170);
    for ([_]u32{ default_bike, clamped, smooth }) |bike|
        try b.objectOp(.wheeled_set_driver_input, bike, .{ .f = &.{ 0, 0, 0, 0 } });
}

/// A ramp for the motorcycle scenes: a static box of half extent (4, 0.5, 12) rotated by -0.15 rad around x whose top
/// rises from the floor (y = 0) at z = 0 to y = 3.6 at z = 23.8 (x: the center)
fn addMotorcycleRamp(b: *SceneBuilder, x: Real) Allocator.Error!u32 {
    return addStaticBox(b, .{ 4, 0.5, 12 }, .{ x, 1.2986, 12 }, axisAngle(.{ 1, 0, 0 }, -0.15), 0);
}

/// Motorcycles on a slope and braking: one rides up a ramp, steers on it and brakes until it stands on the slope (the
/// lean target from the ground impulses on a slope); one brakes and hand brakes in a turn on the flat floor with a
/// gravity override (the max steer angle factor uses the overridden gravity) and its lean steering limit only active at
/// speed; a snapshot in the turn is restored cross
fn buildVehicleMotorcycleSlopesAndBraking(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 180;
    _ = try addFloor(b, 50, 0);
    _ = try addMotorcycleRamp(b, 0);
    var climber_desc = motorcycleDesc(.cast_sphere);
    climber_desc.callbacks = VehicleCallbacks.post_collide;
    const climber = try addTestMotorcycle(b, .{ 0, 0.9, -2 }, identity_rotation, climber_desc);
    var braker_desc = motorcycleDesc(.ray);
    braker_desc.lean_spring_damping = 1300.0;
    const braker = try addTestMotorcycle(b, .{ 14, 0.9, -20 }, identity_rotation, braker_desc);

    b.at(1);
    try b.expectWheelContacts(climber, 0b11);
    try b.objectOp(.wheeled_set_driver_input, climber, .{ .f = &.{ 0.7, 0, 0, 0 } });
    try b.objectOp(.wheeled_set_driver_input, braker, .{ .f = &.{ 1.0, 0, 0, 0 } });
    b.at(40);
    try b.objectOp(.vehicle_override_gravity, braker, .{ .f = &.{ 0, -7.5, 0 } });
    try b.objectOp(.wheeled_set_right_input, braker, .{ .f = &.{0.5} });
    b.at(60);
    try b.sysOp(.snapshot, .{ .index = 0 });
    try b.objectOp(.wheeled_set_right_input, climber, .{ .f = &.{-0.2} });
    b.at(75);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Back to update 60
    b.at(80);
    try b.objectOp(.wheeled_set_driver_input, braker, .{ .f = &.{ 0, 0.4, 0.8, 0.5 } }); // Brake and hand brake in the turn
    b.at(100);
    try b.objectOp(.wheeled_set_driver_input, climber, .{ .f = &.{ 0, 0.1, 1.0, 0 } }); // Brake on the slope
    try b.objectOp(.vehicle_reset_gravity_override, braker, .{});
    b.at(130);
    try b.objectOp(.wheeled_set_driver_input, braker, .{ .f = &.{ 0.6, -0.3, 0, 0 } });
}

/// The lean spring integration on a slope (no D12 sweeps and no restore, `sweep_stride = 0`: MotorcycleController::
/// SaveState does not save the integrated angle): a motorcycle with a non-zero integration coefficient rides up the ramp
/// and turns left and right on it (the target lean follows the ground impulses of the sloped contact), its lean
/// controller is off for a few updates (the integrated angle is reset), the coefficient and its decay change, it brakes
/// on the slope; a second one drops from above the floor rolled to the side (the integrated angle decays while its
/// wheels are in the air) and rides off
fn buildVehicleMotorcycleLeanIntegrationSlope(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 150;
    _ = try addFloor(b, 50, 0);
    _ = try addMotorcycleRamp(b, 0);
    var climber_desc = motorcycleDesc(.cast_sphere);
    climber_desc.lean_spring_integration_coefficient = 1500.0;
    climber_desc.lean_spring_integration_coefficient_decay = 3.0;
    climber_desc.callbacks = VehicleCallbacks.post_step;
    const climber = try addTestMotorcycle(b, .{ 0, 0.9, -2 }, identity_rotation, climber_desc);
    var dropped_desc = motorcycleDesc(.ray);
    dropped_desc.lean_spring_integration_coefficient = 800.0;
    const dropped = try addTestMotorcycle(b, .{ 14, 1.4, -10 }, axisAngle(.{ 0, 0, 1 }, -0.2), dropped_desc);

    b.at(1);
    try b.expectWheelContacts(dropped, 0); // In the air
    try b.objectOp(.wheeled_set_driver_input, climber, .{ .f = &.{ 0.8, 0, 0, 0 } });
    try b.objectOp(.wheeled_set_driver_input, dropped, .{ .f = &.{ 0.5, 0, 0, 0 } });
    b.at(40);
    try b.expectWheelContacts(dropped, 0b11);
    try b.objectOp(.wheeled_set_right_input, climber, .{ .f = &.{-0.4} });
    try b.objectOp(.wheeled_set_right_input, dropped, .{ .f = &.{0.3} });
    b.at(60);
    try b.objectOp(.wheeled_set_right_input, climber, .{ .f = &.{0.4} });
    try b.objectOp(.motorcycle_set_lean_spring_integration_coefficient, climber, .{ .f = &.{2500.0} });
    b.at(75);
    try b.objectOp(.motorcycle_enable_lean_controller, dropped, .{ .i = &.{0} }); // Resets the integrated angle
    b.at(79);
    try b.objectOp(.motorcycle_enable_lean_controller, dropped, .{ .i = &.{1} });
    try b.objectOp(.motorcycle_set_lean_spring_integration_coefficient_decay, dropped, .{ .f = &.{8.0} });
    b.at(110);
    try b.objectOp(.wheeled_set_driver_input, climber, .{ .f = &.{ 0, 0, 1.0, 0 } });
    try b.objectOp(.wheeled_set_right_input, dropped, .{ .f = &.{-0.3} });
}

/// Far from the origin: a motorcycle rides up a ramp and turns, another leans into a turn on the floor; a snapshot
/// restored cross
fn buildVehicleMotorcycleFarFromOrigin(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 120;
    const o: [3]Real = .{ 6000, -80, -11000 };
    _ = try addStaticBox(b, .{ 40, 0.5, 40 }, .{ o[0], o[1] - 0.5, o[2] }, identity_rotation, 0);
    _ = try addStaticBox(b, .{ 4, 0.5, 12 }, .{ o[0], o[1] + 1.2986, o[2] + 12 }, axisAngle(.{ 1, 0, 0 }, -0.15), 0);
    const climber = try addTestMotorcycle(b, .{ o[0], o[1] + 0.9, o[2] - 2 }, identity_rotation, motorcycleDesc(.cast_sphere));
    const turner = try addTestMotorcycle(b, .{ o[0] + 15, o[1] + 0.9, o[2] - 20 }, identity_rotation, motorcycleDesc(.cast_cylinder));

    b.at(1);
    try b.objectOp(.wheeled_set_driver_input, climber, .{ .f = &.{ 0.7, 0, 0, 0 } });
    try b.objectOp(.wheeled_set_driver_input, turner, .{ .f = &.{ 0.8, 0, 0, 0 } });
    b.at(40);
    try b.expectWheelContacts(turner, 0b11);
    try b.objectOp(.wheeled_set_right_input, turner, .{ .f = &.{0.4} });
    try b.sysOp(.snapshot, .{ .index = 0 });
    b.at(55);
    try b.objectOp(.wheeled_set_right_input, climber, .{ .f = &.{0.2} });
    b.at(65);
    try b.sysOp(.restore_cross, .{ .index = 0 }); // Back to update 40
    b.at(90);
    try b.objectOp(.wheeled_set_driver_input, climber, .{ .f = &.{ 0, 0, 1.0, 0 } });
}

// (append the vehicle scenes here)

/// The scene of the test "SimulationParity object state decoder": a character, a character virtual on the floor (an
/// active contact after its first update), a ragdoll and a car (every object state field but the tracked and the
/// motorcycle ones), for a few updates on the C++ side
pub fn buildObjectStateTestScene(b: *SceneBuilder) Allocator.Error!void {
    const shapes = try CharacterShapes.add(b);
    b.num_updates = 3;
    _ = try addFloor(b, 20, 0);
    const c = try b.character(.{ .base = .{ .shape = shapes.standing }, .layer = Layers.moving, .position = .{ 5, 0.02, 5 } });
    try b.objectOp(.character_add_to_physics_system, c, .{});
    try b.objectOp(.character_post_simulation, c, .{ .f = &.{0.05} });
    const cv = try b.characterVirtual(.{ .base = .{ .shape = shapes.standing }, .position = .{ -5, 0, 5 } });
    try moveCharacterVirtual(b, cv, .{ 0.5, -0.5, 0 }, false, cv_query);
    const ragdoll = try b.createRagdoll(try addTestRagdoll(b, .{ 5, 1.2, -5 }, .{ .stabilize = 1 }));
    try b.objectOp(.ragdoll_add_to_physics_system, ragdoll, .{});
    _ = try addTestCar(b, .{ -5, 0.85, -5 }, .{ .tester = .{ .object_layer = Layers.moving } });
}

/// The scene of the test "SimulationParity CharacterID consumption": shape 0 is the shape of its CharacterVirtual
pub fn buildCharacterIdTestScene(b: *SceneBuilder) Allocator.Error!void {
    b.num_updates = 1;
    _ = try b.capsule(0.6, 0.3);
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
