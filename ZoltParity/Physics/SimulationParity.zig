//! Simulation parity (SimulationArchitecture D12): Zolt's PhysicsSystem against Jolt's, update by update.
//!
//! Every scene of SimScenes.zig is built on both sides from the same description (`SimSceneDesc`, the layers of
//! UnitTests/Layers.h / ZoltTests/Layers.zig), the same operations are applied in the same order between the updates,
//! and after every update the following must be identical:
//! - the PhysicsSystem::SaveState(EStateRecorderState::All) bytes. On a mismatch the first differing byte is decoded to
//!   (section, body, field) with the layout of PhysicsSystem / BodyManager / Body / MotionProperties::SaveState (and
//!   the contact cache / constraint sections), and the first differing fields are listed with both values;
//! - the EPhysicsUpdateError bits returned by Update and the result of every operation (body IDs, IsActive / IsAdded).
//!   The `expect_body_state` operations must also return what the scene expects (a body asleep before an operation
//!   that must not wake it): a scene that no longer reaches the case it is written for fails;
//! - the listener callbacks of the update and of the operations before it as sorted records (body activation listener,
//!   step listeners, contact listener): their order depends on the threads, their content does not.
//! At the end of a scene the partial states (each EStateRecorderState flag) are compared too, and the states filtered by a
//! StateRecorderFilter (`StateFilter` / SimStateFilter: some bodies, the constraints with even user data, some contacts),
//! which each side then restores from the other side's bytes (ShouldRestoreContact filters the contacts) before the
//! full states are compared again.
//!
//! Thread matrix: Zolt with JobSystemSingleThreaded and JobSystemThreadPool with 0, 1, 3 and 7 workers, C++ with a
//! JobSystemThreadPool with 0, 3 and 7 workers (`configs`). Run it in Debug and ReleaseFast, single and double precision.
//!
//! Per scene (D12), in the configurations with `sweeps`: before update 0 and every `sweep_stride`-th update every temp
//! allocation of the update fails once (FailingTempAllocator: error.OutOfMemory, the temp allocator is empty, a failure
//! in the first two allocations (the steps, the body pairs: before any job ran) leaves the state unchanged, and after
//! restoring the state saved before the update one more update gives the C++ bytes), then every createJob fails once
//! (FailingJobSystem: a failure while building the job graph returns error.OutOfMemory and changes nothing, a job that
//! cannot be spawned from inside a job changes no result). Creating the scene (system, shapes, the bodies of update 0)
//! runs under std.testing.checkAllAllocationFailures (unless `SceneEntry.creation_sweep` is false: the PerformanceTest
//! scenes with thousands of bodies, for which it is quadratic). The InvalidateContactCache flags that the operations
//! before the update set (SetShape, SetUseManifoldReduction, ...) are not saved state, in Jolt neither: every swept update
//! starts from the snapshot with them set again (`restoreZoltBeforeUpdate`), while the recovery after a failed update
//! relies on PhysicsSystem.update putting them back (scenes physics_tests_switch_use_manifold_reduction_1 and _2). The
//! failed updates change the layout of the broad phase trees, which the saved state does not hold: scenes with
//! `SceneEntry.restore_broad_phase` (the object operations query the broad phase in hit order) redo the update after the
//! sweeps from the snapshot and the broad phase layout of that instant (`BroadPhaseSnapshot`), the other scenes keep the
//! free running layout (the physics pipeline does not depend on it).
//!
//! The PerformanceTest scenes (`perf_*`, milestone M6) are the exact setup of ZoltPerformanceTest's Pyramid,
//! ConvexVsMesh and HighSpeed scenes with -q=LinearCast, compared after every update for their first 50 / 80 / 20
//! updates: a hash of the determinism check that no longer matches is found here first, with the first differing field.
//! Building the terrain of perf_convex_vs_mesh with 8 instead of the scene's 4 triangles per leaf on the Zolt side only
//! is reported as "state differs at byte 46837 ...: bodies[350] (id 0x0080015e) rotation.x zolt 0x3d6041ab jolt
//! 0x3d604197" (update 59).
//!
//! Every scene is its own test (`SceneTest(name, index)`, filter with -Dtest-filter=<scene name>). Every Zolt class is
//! ported, so every Blocker is supported and every scene runs against Zolt. A scene whose `blocked_by` is not supported
//! (SimScenes.Blocker.isSupported: a class it needs is a stub again) fails (checkSupported). Add a scene in
//! SimScenes.zig (see its header). `verbose = true` prints what every scene did (activations, sleeping bodies, step
//! listener calls, CCD bodies, active constraints, sweep counts): check it when you write a scene.
//!
//! Both PhysicsSystems use BroadPhaseQuadTree (the BROAD_PHASE define of PhysicsSystem.cpp, Zolt's default since A1;
//! no comparison was ever loosened for the brute force broad phase that Zolt used before). The trees themselves are
//! compared by BroadPhaseQuadTreeParity.zig.
//!
//! Proof that it bites: computing `(gravity * gravity_factor) * dt + (force * inv_mass) * dt` instead of Jolt's
//! `(gravity * gravity_factor + force * inv_mass) * dt` in MotionProperties.applyForceTorqueAndDragInternal is reported
//! as "state differs at byte 862 ...: bodies[6] (id 0x00800006) motion_properties.linear_velocity.y zolt 0xc0dcbfe1
//! jolt 0xc0dcbfe2" (scene forces_torques_impulses, update 42); passing user data + 1 to OnBodyDeactivated is reported
//! as a differing listener record. Computing r1_plus_u of JobResolveCCDContacts as
//! `(contact point - center of mass) - fraction * delta` instead of Jolt's `contact point - (center of mass + fraction *
//! delta)` is reported as "state differs at byte 254 ...: bodies[3] (id 0x00800003)
//! motion_properties.linear_velocity.x zolt 0x41cc9e9c jolt 0x41cc9ea4" (scene ccd_static_targets, update 7).
//! Solving the point part of FixedConstraint.solveVelocityConstraint before its rotation part is reported as "state
//! differs at byte 26 ...: bodies[0] (id 0x00800000) position.x zolt 0xba004cc4 jolt 0xba023e82" (scene
//! constraints_fixed, update 0).
//!
//! CCD (Wave B R4): the driver counts the FindCCDContacts jobs of every update (Zolt side) and the updates after which
//! the contact cache of the C++ state holds CCD manifolds (OnCCDContactAdded of a hit without a discrete contact of the
//! same sub shape pair in the same update), for SimScenes.Expect.ccd_jobs / ccd_hits.
//!
//! Wave C: the cases of the ported unit tests (ContactListenerTests, SensorTests, ShapeTests, ActiveEdgesTests) that the
//! other scenes did not reach are mirrored by scenes of their own, with ContactFlags.mass_override (TestMassOverride's
//! sGetInvMassScale), SimCollideMode (the sCollideBodyVsBody functions of SensorTests) and the operations
//! `were_bodies_in_contact` / `set_collide_kinematic_vs_non_dynamic`. Proof that they bite: giving body 2 the inverse
//! mass (or inertia) scale of body 1 fails contacts_mass_override (and ccd_infinite_mass_override), installing the
//! default function for SimCollideMode.all fails contacts_sim_collide_all, and returning only one order of
//! WereBodiesInContact is reported as "operation 8 (were_bodies_in_contact on handle 0) returned 1 in zolt and 3 in
//! jolt".
//!
//! Constraints (Phase 6, the recipe for a worker is in the SimScenes header): both worlds keep a reference to every
//! created constraint (constraint handles). The Zolt side of each ConstraintKind and of the class operations is a slot
//! per worker group (`FixedPointSlot`, `HingeSlot`, ...; `StubSlot` while the classes were not ported; a ported class
//! with a stub slot fails with error.SceneNotSupported). Before a
//! scene runs, `checkConstraintDescriptions` compares the binary state of the settings that the slots make from the
//! scene's descriptions with the C++ settings (SimMakeConstraintSettings), so a mismatch later is the library's, not the
//! harness'. The state decoder names the fields of a constraint class (`stateFields` of its slot, e.g.
//! "constraints[3] (id 0x00000003) point_constraint_part.total_lambda.x"). The driver counts the active constraints
//! before every update (Expect.active_constraints; every scene with constraints must solve some) and checks the
//! `expect_constraint_state` operations. Proof that it bites: computing SetPoint1's world space point with body 2's
//! inverse center of mass transform is reported as "operation 51 (constraint_settings_hash on constraint handle 4)
//! returned ... in zolt and ... in jolt" (scene constraints_enable_disable_remove, update 60); adding the delta in
//! PointConstraint.notifyShapeChanged instead of subtracting it as "state differs at byte 696 ...: bodies[5] (id
//! 0x00800005) position.x" (same scene, update 50); saving the constraints that the filter rejects as "the filtered
//! states differ at byte 1979" (scene constraints_random_1); reading axis Y2 of a fixed constraint description from
//! f[6..9] in the slot fails "SimScenes constraint descriptions are ConstraintsParity's" and every fixed constraint scene
//! before it runs ("constraint description 1 (fixed): the settings of the Zolt slot differ from SimMakeConstraintSettings").
//!
//! Characters, ragdolls and vehicles (Phase 7, the recipe for a worker is in the SimScenes header): both worlds keep
//! their characters, character virtuals, ragdolls and vehicles by handle. The Zolt side of each class group is a slot
//! (`CharacterSlot`, `CharacterVirtualSlot`, `RagdollSlot`, `VehicleSlot`, `TrackedVehicleSlot`, `MotorcycleSlot`;
//! `Phase7StubSlot` until the group was ported; a supported scene with a stub slot fails in checkPhase7Slots). After
//! every update (and after the D12 sweeps and every restore_cross) the object states are compared
//! (SimScenes.ObjectField: the SaveState bytes of the characters and character virtuals and the getters that the states
//! do not hold), a mismatch is decoded to the field, the object handle, the item and the component
//! (`describeObjectStateDifference`, the SaveState fields with the members of CharacterBase / CharacterVirtual /
//! CharacterContact::SaveState); the CharacterContactListener records (RecordKind 7..17) are compared in call order
//! (they are the callbacks of the user thread's character operations; a record that cannot be stored fails the
//! operation that made the callback); `snapshot` / `restore_cross` include the SaveState of every character, and a
//! restore must read every byte; the `expect_*` operations of Phase 7 must return the scene's `count`. Before a scene
//! runs, checkPhase7Settings compares the binary state of the vehicle and ragdoll settings that the filled slots make
//! with the C++ settings, the round trips (each side restores the other side's bytes and saves again) and the
//! RagdollSettings tables. The test "SimulationParity CharacterID consumption" compares which constructions consume a
//! CharacterID. With a reference built with JPH_ENABLE_ASSERTS (`zig build parity -Dreference_asserts=true`) a scene
//! fails when its C++ side hits a Jolt assert (the release_only scenes excepted). A release_only scene runs in every
//! build: a handler counts Zolt's recoverable asserts and lets the code continue like Jolt's release build
//! (RemainingFeaturesArchitecture.md R8). Every run must hit an assert, the same number in every configuration, and
//! with -Dreference_asserts=true as many as the C++ side (checkReleaseAsserts).
//!
//! Proof that the Phase 7 checks bite: a Zolt object state field that the C++ side does not write as "object state
//! differs at byte 0 (zolt 20 bytes, jolt 0 bytes): zolt writes more, starting with ragdoll_properties (object handle
//! 0, item 0)"; a supported Blocker whose classes are stubs as "scene character_ops creates Character objects, whose
//! Zolt classes are still stubs, but its blocker .characters is supported"; a ported class (no `is_stub`) with a stub
//! slot as "the Character classes are ported but their slot in SimulationParity.zig is still Phase7StubSlot" (and the
//! test "SimulationParity Phase 7 slots follow the ported classes" fails). CharacterBaseParity.zig compares
//! CharacterBase's SaveState bytes: writing the ground normal in place of the ground velocity fails it. The checks of
//! the foundation review: a character state restore that does not check that every byte was read (the check before the
//! review) fails "SimulationParity restoring the character states reads every byte"; an applyOp that ignores the
//! character recorder fails "SimulationParity a character record that cannot be stored fails the operation" ("expected
//! error.OutOfMemory, found 0"); a copy of CharacterVirtualSettings that consumes an ID (default construction and
//! assignment in place of the copy constructor) fails "SimulationParity CharacterID consumption" ("first difference
//! occurs at index 3"). The Ragdoll group: computing the body 1 to body 2 rotation in
//! Ragdoll.driveToPoseUsingMotorsWithVelocity the other way around (body 1 conjugated times body 2) is reported as
//! "scene ragdoll_motors_animation, ... update 2: after the update, state f: state differs at byte 1667 ...: bodies[13]
//! (id 0x0080000d) position.x zolt 0x394972b8 ... jolt 0x3945ee04" (and ragdoll_save_restore at update 1); finalizing
//! the bodies of Ragdoll.addToPhysicsSystem with Activate whenever it does not lock as "scene ragdoll_add_activation,
//! ... update 0: operation 18 (ragdoll_add_to_physics_system on object handle / description 3) returned 1 in zolt and 0
//! in jolt" (and ragdoll_ops).
//!
//! Proof that the vehicle scenes bite (VehicleSlot, the vehicle core group): multiplying WheelWV's lateral slip angle by
//! 1.0001 is reported as "scene vehicle_wheeled_ops, ... update 11: ... state differs at byte 92 ...: bodies[2] (id
//! 0x00800002) position.x zolt 0xbaa3b604 jolt 0xbaa3b60b" (and vehicle_wheeled_in_air at update 28); the convex radius
//! of VehicleCollisionTesterCastCylinder from the larger instead of the smaller of the wheel's half width and radius as
//! "scene vehicle_wheeled_ops, ... update 35: ... bodies[2] (id 0x00800002) position.x" (the scene switches to the cast
//! cylinder tester at update 35); a WheeledVehicleController::GetWheelSpeedAtClutch that is 1.0001 times Jolt's (only the
//! object state sees it) as "update 1: ... object state differs at byte 1516 ...: vehicle_wheeled (object handle 0, item 0)
//! wheel_speed_at_clutch".
//!
//! Soft bodies (Phase 9, group SH; see "Soft bodies" in the SimScenes header): both worlds generate the shared settings
//! of every SimSoftBodySettings with the same code (SoftBodySlot.makeSettings here, SimMakeSoftBodySettings in the
//! .cpp). Before a scene runs, checkSoftBodySettings compares their SaveBinaryState bytes. A soft body gets a body
//! handle. SoftBodySlot applies the soft body operations, SoftBodyListenerImpl records the callbacks of the soft body
//! contact listener (RecordKind 70..73, compared sorted with the other records). The state decoder reads a soft body's
//! SaveState with the array sizes of jolt_sim_soft_body_info ("soft_body.vertices[5].velocity.y"); the test
//! "SimulationParity state decoder: soft bodies" checks it against the C++ getters, because Zolt's
//! SoftBodyMotionProperties::SaveState is group S4's. The D12 sweeps give every update that starts from the snapshot
//! the skin flags of the snapshot again (`SkinFlags`). After every update the soft body listener records of the C++
//! side must agree with each other (checkSoftBodyRecords).
//!
//! Soft body scenes against Zolt (group S5): the Zolt side also counts the steps in which the CCD response changes a
//! soft body (Expect.soft_body_ccd, SpawnCountingJobSystem), checks the update groups of the shared settings
//! (Expect.soft_body_groups) and counts the bodies deactivated in one update (Expect.update_deactivations). Proof that
//! it bites (the first difference, single threaded):
//! - JobSoftBodyPrepare skips the collide claim loop for the missing Collide jobs: soft_body_cube_on_floor, the sweep
//!   of update 30, "a job could not be spawned: ... bodies[1] (id 0x00800001) position.x";
//! - JobSoftBodyPrepare does not sort the active soft bodies by BodyID: soft_body_several_on_one_body, update 17,
//!   bodies[1] motion_properties.linear_velocity.x (1 ULP);
//! - the CCD face response computes inv_m2 without vertex 2: the CCD scenes, update 0, bodies[0] position.x;
//! - the mass of a fixed vertex is 1e9 instead of 1e10, or vertex 1 tests `inv_mass >= 0`:
//!   soft_body_ccd_fixed_and_rotated, update 0, bodies[3] (the heavy sphere) motion_properties.linear_velocity.x;
//! - v2 rotates with Multiply3x3 instead of Multiply3x3Transposed, or delta_v2 with Multiply3x3Transposed:
//!   soft_body_ccd_fixed_and_rotated, update 0, bodies[0] position.x;
//! - local_normal rotates with Multiply3x3Transposed: soft_body_ccd_fixed_and_rotated, update 0, the
//!   soft_body_contact_body record of the fixed vertices (they keep their CCD contact marks);
//! - the in-loop flush of SoftBodyFinalize's sleep batch deactivates one body less: soft_body_many_go_to_sleep, update
//!   31, bodies[64] is_active.
//!
//! C ABI wrappers: ZoltParity/Physics/SimulationReference.cpp.

const std = @import("std");
const zolt = @import("zolt");
const TestLayers = @import("test_layers");
const SimScenes = @import("SimScenes.zig");
const ParityRenderer = @import("../Renderer/ParityRenderer.zig");

const Allocator = std.mem.Allocator;
const Core = zolt.Core;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const RVec3 = zolt.RVec3;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Mutex = zolt.Mutex;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyInterface = zolt.BodyInterface;
const BodyCreationSettings = zolt.BodyCreationSettings;
const Activation = zolt.Activation;
const MotionType = zolt.MotionType;
const MotionQuality = zolt.MotionQuality;
const AllowedDOFs = zolt.AllowedDOFs;
const ObjectLayer = zolt.ObjectLayer;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const PhysicsSettings = zolt.PhysicsSettings;
const PhysicsStepListener = zolt.PhysicsStepListener;
const PhysicsStepListenerContext = zolt.PhysicsStepListenerContext;
const BodyActivationListener = zolt.BodyActivationListener;
const ContactListener = zolt.ContactListener;
const ContactManifold = zolt.ContactManifold;
const ContactSettings = zolt.ContactSettings;
const ValidateResult = zolt.ValidateResult;
const CollideShapeResult = zolt.CollideShapeResult;
const SubShapeIDPair = zolt.SubShapeIDPair;
const StateRecorderImpl = zolt.StateRecorderImpl;
const StateRecorderState = zolt.StateRecorderState;
const PhysicsUpdateError = zolt.PhysicsUpdateError;
const TempAllocator = zolt.TempAllocator;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const FailingTempAllocator = zolt.FailingTempAllocator;
const JobSystem = zolt.JobSystem;
const JobSystemSingleThreaded = zolt.JobSystemSingleThreaded;
const JobSystemThreadPool = zolt.JobSystemThreadPool;
const FailingJobSystem = zolt.FailingJobSystem;

const SimShape = SimScenes.SimShape;
const SimBody = SimScenes.SimBody;
const SimPhysicsSettings = SimScenes.SimPhysicsSettings;
const SimStepListener = SimScenes.SimStepListener;
const SimConstraint = SimScenes.SimConstraint;
const ConstraintKind = SimScenes.ConstraintKind;
const ConstraintState = SimScenes.ConstraintState;
const ConstraintOpRange = SimScenes.ConstraintOpRange;
const Constraint = zolt.Constraint;
const TwoBodyConstraint = zolt.TwoBodyConstraint;
const ConstraintSettings = zolt.ConstraintSettings;
const SimOp = SimScenes.SimOp;
const SimSceneDesc = SimScenes.SimSceneDesc;
const SimRecord = SimScenes.SimRecord;
const SimCompoundChild = SimScenes.SimCompoundChild;
const SceneBuilder = SimScenes.SceneBuilder;
const SceneEntry = SimScenes.SceneEntry;
const OpKind = SimScenes.OpKind;
const ShapeKind = SimScenes.ShapeKind;
const ContactListenerMode = SimScenes.ContactListenerMode;
const CombineMode = SimScenes.CombineMode;
const SimShapeFilterMode = SimScenes.SimShapeFilterMode;
const SimCollideMode = SimScenes.SimCollideMode;
const ContactFlags = SimScenes.ContactFlags;
const SimShapeFilter = zolt.SimShapeFilter;
const SubShapeID = zolt.SubShapeID;
const Color = zolt.Color;
const RecordKind = SimScenes.RecordKind;
const StepListenerMode = SimScenes.StepListenerMode;
const invalid_handle = SimScenes.invalid_handle;
const Phase7OpRange = SimScenes.Phase7OpRange;
const Phase7Classes = SimScenes.Phase7Classes;
const ObjectField = SimScenes.ObjectField;
const ObjectFieldHeader = SimScenes.ObjectFieldHeader;
const ObjectComponent = SimScenes.ObjectComponent;
const ObjectStateWriter = SimScenes.ObjectStateWriter;
const VehicleControllerKind = SimScenes.VehicleControllerKind;
const VehicleSettingsPart = SimScenes.VehicleSettingsPart;
const SimVehicle = SimScenes.SimVehicle;
const SimVehicleWheel = SimScenes.SimVehicleWheel;
const VehicleConstraint = zolt.VehicleConstraint;
const VehicleControllerSettings = zolt.VehicleControllerSettings;
const WheelSettings = zolt.WheelSettings;
const Wheel = zolt.Wheel;

const io = std.testing.io;

/// The result of a C function of SimulationReference.cpp that has no bytes for its arguments (cNoBytes)
const no_bytes: u32 = 0xffffffff;

/// The C++ reference functions, see SimulationReference.cpp
const jolt = struct {
    extern fn jolt_sim_struct_sizes(out_sizes: *[16]u32) void;
    extern fn jolt_sim_default_settings(out_settings: *SimPhysicsSettings) void;
    extern fn jolt_sim_create(desc: *const SimSceneDesc, num_threads: u32) *anyopaque;
    extern fn jolt_sim_destroy(world: *anyopaque) void;
    extern fn jolt_sim_apply_op(world: *anyopaque, op: *const SimOp) u32;
    extern fn jolt_sim_update(world: *anyopaque, delta_time: f32, collision_steps: u32) u32;
    extern fn jolt_sim_save_state(world: *anyopaque, state: u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_sim_restore_state(world: *anyopaque, bytes: [*]const u8, size: u32) u32;
    extern fn jolt_sim_save_state_filtered(world: *anyopaque, state: u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_sim_restore_state_filtered(world: *anyopaque, bytes: [*]const u8, size: u32) u32;
    extern fn jolt_sim_take_records(world: *anyopaque, out_records: [*]SimRecord, capacity: u32) u32;
    extern fn jolt_sim_body_has_motion_properties(world: *anyopaque, body_id: u32) u32;
    extern fn jolt_sim_constraint_state_size(world: *anyopaque, index: u32, out_num_constraints: *u32) u32;
    extern fn jolt_sim_constraint_sub_type(world: *anyopaque, index: u32) u32;
    extern fn jolt_sim_num_active_constraints(world: *anyopaque) u32;
    extern fn jolt_sim_default_constraint_settings_save(kind: u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_sim_constraint_settings_save(constraint: *const SimConstraint, out_bytes: [*]u8, capacity: u32) u32;
    // Phase 7: characters, ragdolls and vehicles
    extern fn jolt_sim_phase7_struct_sizes(out_sizes: *[32]u32) void;
    extern fn jolt_sim_object_state(world: *anyopaque, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_sim_take_character_records(world: *anyopaque, out_records: [*]SimRecord, capacity: u32) u32;
    extern fn jolt_sim_save_character_states(world: *anyopaque, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_sim_restore_character_states(world: *anyopaque, bytes: [*]const u8, size: u32) u32;
    extern fn jolt_sim_default_character(out_character: *SimScenes.SimCharacter) void;
    extern fn jolt_sim_default_character_virtual(out_character: *SimScenes.SimCharacterVirtual) void;
    extern fn jolt_sim_default_extended_update(out_floats: *[12]f32) void;
    extern fn jolt_sim_default_vehicle(kind: u32, out_vehicle: *SimScenes.SimVehicle) void;
    extern fn jolt_sim_default_vehicle_wheel(kind: u32, out_wheel: *SimScenes.SimVehicleWheel) void;
    extern fn jolt_sim_vehicle_settings_part(desc: *const SimSceneDesc, index: u32, part: u32, item: u32, restore: u32, bytes: [*]const u8, size: u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_sim_ragdoll_settings_bytes(world: *anyopaque, index: u32, restore: u32, bytes: [*]const u8, size: u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_sim_ragdoll_settings_info(world: *anyopaque, index: u32, out_values: [*]u32, capacity: u32) u32;
    extern fn jolt_sim_character_id_sequence(world: *anyopaque, first_id: u32, out_values: *[SimScenes.character_id_sequence_len]u32) void;
    extern fn jolt_sim_zero_filling_check() u32;
    extern fn jolt_sim_reference_assert_count() u32;
    extern fn jolt_sim_reference_has_asserts() u32;
    extern fn jolt_sim_test_assert_hook(out_before: *u32, out_after: *u32) void;
    // Phase 9: soft bodies
    extern fn jolt_sim_soft_body_struct_sizes(out_sizes: *[8]u32) void;
    extern fn jolt_sim_default_soft_body(out_soft_body: *SimScenes.SimSoftBody) void;
    extern fn jolt_sim_default_soft_body_settings(out_settings: *SimScenes.SimSoftBodySettings) void;
    extern fn jolt_sim_soft_body_settings_bytes(desc: *const SimSceneDesc, index: u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_sim_soft_body_info(world: *anyopaque, body_id: u32, out_counts: *[3]u32) u32;
    extern fn jolt_sim_soft_body_values(world: *anyopaque, body_id: u32, out_values: [*]f32, capacity: u32) u32;
    // Group D3: the simulation-time draws (SimDraw)
    extern fn jolt_sim_draw_begin() u32;
    extern fn jolt_sim_draw_end() void;
    extern fn jolt_sim_draw_take_log(out_bytes: [*]u8, capacity: u64) u64;
    // Group D4: the draws after an update (SimDraw.postDraws)
    extern fn jolt_sim_draw_world(world: *anyopaque, body_bits: u32, shape_color: u32, soft_body_constraint_color: u32, constraints: u32, rpm_meter: ?*const [4]f32, draw_size: f32) void;
    // ConstraintsReference.cpp (the constraint parity of Phase 6): the descriptions of SimScenes use its layout
    extern fn jolt_cs_struct_sizes(out_sizes: *[12]u32) void;
    extern fn jolt_cs_settings_save(settings: *const CSSettings, out_bytes: [*]u8, capacity: u32) u32;
};

/// CSSettings of ConstraintsReference.cpp (ConstraintsParity.zig): the constraint descriptions of the constraint parity,
/// whose layout SimScenes.SimConstraint shares (test "SimScenes constraint descriptions are ConstraintsParity's")
const CSSettings = extern struct {
    /// EConstraintSubType
    kind: u32,
    enabled: u32,
    constraint_priority: u32,
    num_velocity_steps_override: u32,
    num_position_steps_override: u32,
    draw_constraint_size: f32,
    user_data: u64,
    i: [32]u32,
    r: [12]Real,
    f: [192]f32,
};

// -------------------------------------------------------------------------------------------------------------------
// Conversions (the C++ side does the same in SimulationReference.cpp)

fn load3(v: []const f32) Vec3 {
    return Vec3.init(v[0], v[1], v[2]);
}

fn loadQuat(v: []const f32) Quat {
    return Quat.init(v[0], v[1], v[2], v[3]);
}

fn loadR3(v: []const Real) RVec3 {
    return RVec3.init(v[0], v[1], v[2]);
}

fn toPhysicsSettings(s: *const SimPhysicsSettings) PhysicsSettings {
    return .{
        .max_in_flight_body_pairs = s.max_in_flight_body_pairs,
        .step_listeners_batch_size = s.step_listeners_batch_size,
        .step_listener_batches_per_job = s.step_listener_batches_per_job,
        .baumgarte = s.baumgarte,
        .speculative_contact_distance = s.speculative_contact_distance,
        .penetration_slop = s.penetration_slop,
        .linear_cast_threshold = s.linear_cast_threshold,
        .linear_cast_max_penetration = s.linear_cast_max_penetration,
        .manifold_tolerance = s.manifold_tolerance,
        .max_penetration_distance = s.max_penetration_distance,
        .body_pair_cache_max_delta_position_sq = s.body_pair_cache_max_delta_position_sq,
        .body_pair_cache_cos_max_delta_rotation_div2 = s.body_pair_cache_cos_max_delta_rotation_div2,
        .contact_normal_cos_max_delta_rotation = s.contact_normal_cos_max_delta_rotation,
        .contact_point_preserve_lambda_max_dist_sq = s.contact_point_preserve_lambda_max_dist_sq,
        .internal_edge_removal_vertex_tolerance_sq = s.internal_edge_removal_vertex_tolerance_sq,
        .num_velocity_steps = s.num_velocity_steps,
        .num_position_steps = s.num_position_steps,
        .min_velocity_for_restitution = s.min_velocity_for_restitution,
        .time_before_sleep = s.time_before_sleep,
        .point_velocity_sleep_threshold = s.point_velocity_sleep_threshold,
        .constraint_warm_start = s.constraint_warm_start != 0,
        .use_body_pair_contact_cache = s.use_body_pair_contact_cache != 0,
        .use_manifold_reduction = s.use_manifold_reduction != 0,
        .use_large_island_splitter = s.use_large_island_splitter != 0,
        .allow_sleeping = s.allow_sleeping != 0,
        .check_active_edges = s.check_active_edges != 0,
    };
}

fn fromPhysicsSettings(s: *const PhysicsSettings) SimPhysicsSettings {
    return .{
        .max_in_flight_body_pairs = s.max_in_flight_body_pairs,
        .step_listeners_batch_size = s.step_listeners_batch_size,
        .step_listener_batches_per_job = s.step_listener_batches_per_job,
        .baumgarte = s.baumgarte,
        .speculative_contact_distance = s.speculative_contact_distance,
        .penetration_slop = s.penetration_slop,
        .linear_cast_threshold = s.linear_cast_threshold,
        .linear_cast_max_penetration = s.linear_cast_max_penetration,
        .manifold_tolerance = s.manifold_tolerance,
        .max_penetration_distance = s.max_penetration_distance,
        .body_pair_cache_max_delta_position_sq = s.body_pair_cache_max_delta_position_sq,
        .body_pair_cache_cos_max_delta_rotation_div2 = s.body_pair_cache_cos_max_delta_rotation_div2,
        .contact_normal_cos_max_delta_rotation = s.contact_normal_cos_max_delta_rotation,
        .contact_point_preserve_lambda_max_dist_sq = s.contact_point_preserve_lambda_max_dist_sq,
        .internal_edge_removal_vertex_tolerance_sq = s.internal_edge_removal_vertex_tolerance_sq,
        .num_velocity_steps = s.num_velocity_steps,
        .num_position_steps = s.num_position_steps,
        .min_velocity_for_restitution = s.min_velocity_for_restitution,
        .time_before_sleep = s.time_before_sleep,
        .point_velocity_sleep_threshold = s.point_velocity_sleep_threshold,
        .constraint_warm_start = @intFromBool(s.constraint_warm_start),
        .use_body_pair_contact_cache = @intFromBool(s.use_body_pair_contact_cache),
        .use_manifold_reduction = @intFromBool(s.use_manifold_reduction),
        .use_large_island_splitter = @intFromBool(s.use_large_island_splitter),
        .allow_sleeping = @intFromBool(s.allow_sleeping),
        .check_active_edges = @intFromBool(s.check_active_edges),
    };
}

fn floatBits(v: f32) u32 {
    return @bitCast(v);
}

// -------------------------------------------------------------------------------------------------------------------
// Listeners of the Zolt side (callbacks come from the worker threads)

/// Collects the callback records (a failed append is reported by `checkError`: the callbacks cannot return errors)
const Recorder = struct {
    allocator: Allocator,
    mutex: Mutex = .{},
    records: std.ArrayList(SimRecord) = .empty,
    out_of_memory: bool = false,

    fn deinit(self: *Recorder) void {
        self.records.deinit(self.allocator);
    }

    fn add(self: *Recorder, record: SimRecord) void {
        self.mutex.lock(io);
        defer self.mutex.unlock(io);
        self.records.append(self.allocator, record) catch {
            self.out_of_memory = true;
        };
    }

    fn checkError(self: *const Recorder) Allocator.Error!void {
        if (self.out_of_memory) return error.OutOfMemory;
    }
};

const ActivationListenerImpl = struct {
    recorder: *Recorder,
    world: *ZoltWorld,

    fn record(self: *ActivationListenerImpl, kind: RecordKind, body_id: BodyID, body_user_data: u64) void {
        var r: SimRecord = .{ .kind = @intFromEnum(kind) };
        r.ids[0] = body_id.getIndexAndSequenceNumber();
        r.data[0] = @truncate(body_user_data);
        r.data[1] = @truncate(body_user_data >> 32);
        self.recorder.add(r);
    }

    pub fn onBodyActivated(self: *ActivationListenerImpl, body_id: BodyID, body_user_data: u64) void {
        self.record(.body_activated, body_id, body_user_data);
    }

    pub fn onBodyDeactivated(self: *ActivationListenerImpl, body_id: BodyID, body_user_data: u64) void {
        self.record(.body_deactivated, body_id, body_user_data);

        // Zolt only (Expect.large_island): bodies that go to sleep from the final batch of a split island are deactivated
        // while the LargeIslandSplitter holds its split islands (between JobFinalizeIslands and the reset in
        // StartNextStep / the end of the update, no job writes it then); between the updates it is 0
        if (self.world.system.large_island_splitter.num_split_islands > 0)
            _ = self.world.num_large_island_deactivations.fetchAdd(1, .monotonic);
    }
};

/// The bits of ContactFlags of a body pair
fn contactFlags(body1: *const Body, body2: *const Body) u32 {
    return @truncate((body1.getUserData() | body2.getUserData()) >> 32);
}

/// ContactFlags.reject_marked_child: the sub shape of the body is a child of a compound with the marked user data
/// (SimIsMarkedChild in SimulationReference.cpp)
fn isMarkedChild(body: *const Body, sub_shape_id: SubShapeID) bool {
    const shape = body.getShape();
    if (shape.getType() != .compound)
        return false;
    const compound = shape.cast(zolt.CompoundShape);
    return compound.getCompoundUserData(compound.getSubShapeIndexFromID(sub_shape_id).index) == ContactFlags.marked_child_user_data;
}

/// The hash of the base offset and the contact points of a manifold (SimHashManifold in SimulationReference.cpp)
fn hashManifold(manifold: *const ContactManifold) u32 {
    var h = SimScenes.hash_seed;
    const offset = [3]Real{ manifold.base_offset.getX(), manifold.base_offset.getY(), manifold.base_offset.getZ() };
    h = SimScenes.hashBytes(h, std.mem.sliceAsBytes(&offset));
    for ([_]*const zolt.ContactPoints{ &manifold.relative_contact_points_on1, &manifold.relative_contact_points_on2 }) |points| {
        for (points.constSlice()) |p| {
            const v = [3]f32{ p.getX(), p.getY(), p.getZ() };
            h = SimScenes.hashBytes(h, std.mem.sliceAsBytes(&v));
        }
    }
    return h;
}

/// SimContactListener of SimulationReference.cpp
const ContactListenerImpl = struct {
    recorder: *Recorder,
    mode: ContactListenerMode,

    pub fn onContactValidate(self: *ContactListenerImpl, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult {
        _ = base_offset;
        var r: SimRecord = .{ .kind = @intFromEnum(RecordKind.contact_validate) };
        r.ids = .{ body1.getID().getIndexAndSequenceNumber(), body2.getID().getIndexAndSequenceNumber(), collision_result.sub_shape_id1.getValue(), collision_result.sub_shape_id2.getValue() };
        r.data[0] = floatBits(collision_result.penetration_depth);
        r.data[1] = floatBits(collision_result.penetration_axis.getX());
        r.data[2] = floatBits(collision_result.penetration_axis.getY());
        r.data[3] = floatBits(collision_result.penetration_axis.getZ());
        self.recorder.add(r);

        if (self.mode == .modify) {
            const flags = contactFlags(body1, body2);
            if ((flags & ContactFlags.reject_all) != 0)
                return .reject_all_contacts_for_this_body_pair;
            if ((flags & ContactFlags.reject_marked_child) != 0 and isMarkedChild(body2, collision_result.sub_shape_id2))
                return .reject_contact;
            if ((flags & ContactFlags.reject_speculative) != 0)
                return if (collision_result.penetration_depth < 0.0) .reject_contact else .accept_contact;
            if ((flags & ContactFlags.accept_each) != 0)
                return .accept_contact;
        }
        return .accept_all_contacts_for_this_body_pair;
    }

    /// ContactFlags of ContactListenerMode.modify
    fn modify(self: *const ContactListenerImpl, body1: *const Body, body2: *const Body, settings: *ContactSettings) void {
        if (self.mode != .modify)
            return;
        const flags = contactFlags(body1, body2);
        if ((flags & ContactFlags.zero_friction) != 0)
            settings.combined_friction = 0.0;
        if ((flags & ContactFlags.bouncy) != 0)
            settings.combined_restitution = 0.9;
        if ((flags & ContactFlags.mass_scale) != 0) {
            settings.inv_mass_scale1 = 0.5;
            settings.inv_inertia_scale1 = 0.25;
            settings.inv_mass_scale2 = 2.0;
            settings.inv_inertia_scale2 = 1.5;
        }
        if ((flags & ContactFlags.mass_override) != 0) {
            const scale1 = ContactFlags.massOverrideScale(body1.getUserData());
            settings.inv_mass_scale1 = scale1;
            settings.inv_inertia_scale1 = scale1;
            const scale2 = ContactFlags.massOverrideScale(body2.getUserData());
            settings.inv_mass_scale2 = scale2;
            settings.inv_inertia_scale2 = scale2;
        }
        if ((flags & ContactFlags.surface_velocity) != 0) {
            settings.relative_linear_surface_velocity = Vec3.init(1.5, 0.0, 0.5);
            settings.relative_angular_surface_velocity = Vec3.init(0.0, 0.7, 0.0);
        }
        if ((flags & ContactFlags.sensor) != 0)
            settings.is_sensor = true;
    }

    fn record(self: *ContactListenerImpl, kind: RecordKind, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *const ContactSettings) void {
        var r: SimRecord = .{ .kind = @intFromEnum(kind) };
        r.ids = .{ body1.getID().getIndexAndSequenceNumber(), body2.getID().getIndexAndSequenceNumber(), manifold.sub_shape_id1.getValue(), manifold.sub_shape_id2.getValue() };
        r.data = .{
            floatBits(manifold.world_space_normal.getX()),
            floatBits(manifold.world_space_normal.getY()),
            floatBits(manifold.world_space_normal.getZ()),
            floatBits(manifold.penetration_depth),
            @as(u32, manifold.relative_contact_points_on1.len) | (@as(u32, @intFromBool(settings.is_sensor)) << 8),
            floatBits(settings.combined_friction),
            floatBits(settings.combined_restitution),
            hashManifold(manifold),
        };
        self.recorder.add(r);
    }

    pub fn onContactAdded(self: *ContactListenerImpl, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        self.record(.contact_added, body1, body2, manifold, settings);
        self.modify(body1, body2, settings);
    }

    pub fn onContactPersisted(self: *ContactListenerImpl, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        self.record(.contact_persisted, body1, body2, manifold, settings);
        self.modify(body1, body2, settings);
    }

    pub fn onContactRemoved(self: *ContactListenerImpl, sub_shape_pair: *const SubShapeIDPair) void {
        var r: SimRecord = .{ .kind = @intFromEnum(RecordKind.contact_removed) };
        r.ids = .{ sub_shape_pair.body1_id.getIndexAndSequenceNumber(), sub_shape_pair.sub_shape_id1.getValue(), sub_shape_pair.body2_id.getIndexAndSequenceNumber(), sub_shape_pair.sub_shape_id2.getValue() };
        self.recorder.add(r);
    }
};

/// The state filter of the end of a scene (SimStateFilter of SimulationReference.cpp): bodies whose index is not 1 modulo
/// 3, constraints whose user data is even, contacts of body pairs whose index sum is even; restored are the contacts of
/// body pairs whose index sum is not a multiple of 3
const StateFilter = struct {
    pub const overrides = .{ .shouldSaveBody, .shouldSaveConstraint, .shouldSaveContact, .shouldRestoreContact };

    base: zolt.StateRecorderFilter = .init(@This()),

    pub fn shouldSaveBody(self: *const StateFilter, body: *const Body) bool {
        _ = self;
        return body.getID().getIndex() % 3 != 1;
    }

    pub fn shouldSaveConstraint(self: *const StateFilter, constraint: *const Constraint) bool {
        _ = self;
        return (constraint.getUserData() & 1) == 0;
    }

    pub fn shouldSaveContact(self: *const StateFilter, body1: BodyID, body2: BodyID) bool {
        _ = self;
        return ((body1.getIndex() + body2.getIndex()) & 1) == 0;
    }

    pub fn shouldRestoreContact(self: *const StateFilter, body1: BodyID, body2: BodyID) bool {
        _ = self;
        return (body1.getIndex() + body2.getIndex()) % 3 != 0;
    }
};

/// CombineMode.custom (SimCombineFriction / SimCombineRestitution of SimulationReference.cpp)
const CustomCombine = struct {
    fn friction(body1: *const Body, sub_shape_id1: SubShapeID, body2: *const Body, sub_shape_id2: SubShapeID) f32 {
        _ = .{ sub_shape_id1, sub_shape_id2 };
        return 0.5 * (body1.getFriction() + body2.getFriction());
    }

    fn restitution(body1: *const Body, sub_shape_id1: SubShapeID, body2: *const Body, sub_shape_id2: SubShapeID) f32 {
        return if (sub_shape_id1.getValue() == sub_shape_id2.getValue())
            zolt.math.max(body1.getRestitution(), body2.getRestitution())
        else
            zolt.math.min(body1.getRestitution(), body2.getRestitution());
    }
};

/// SimShapeFilterMode.filter_spheres (SimSphereFilter of SimulationReference.cpp)
const SphereFilter = struct {
    pub const overrides = .{.shouldCollide};

    base: SimShapeFilter = .init(@This()),

    pub fn shouldCollide(self: *const SphereFilter, body1: *const Body, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, body2: *const Body, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
        _ = .{ self, sub_shape_id_of_shape1, sub_shape_id_of_shape2 };
        if ((contactFlags(body1, body2) & ContactFlags.filter_spheres) == 0)
            return true;
        return shape1.getSubType() != .sphere and shape2.getSubType() != .sphere;
    }
};

/// The body vs body collision functions of SimCollideMode (SimCollideBodyVsBody* of SimulationReference.cpp, the
/// sCollideBodyVsBody* functions of SensorTests)
const SimCollide = struct {
    const CollideShapeCollector = zolt.CollideShapeCollector;
    const CollideShapeSettings = zolt.CollideShapeSettings;
    const AnyHitCollector = zolt.AnyHitCollisionCollector(CollideShapeCollector);
    const SubShapeIDCreator = zolt.SubShapeIDCreator;
    const ShapeFilter = zolt.ShapeFilter;

    /// SimCollideMode.all: the default function with EBackFaceMode::CollideWithBackFaces
    fn all(context: ?*anyopaque, allocator: std.mem.Allocator, body1: *const Body, body2: *const Body, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, collide_shape_settings: *CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) std.mem.Allocator.Error!void {
        _ = context;

        // Override the back face mode so we get hits with all triangles
        collide_shape_settings.back_face_mode = .collide_with_back_faces;
        try PhysicsSystem.defaultSimCollideBodyVsBody(allocator, body1, body2, center_of_mass_transform1, center_of_mass_transform2, collide_shape_settings, collector, shape_filter);
    }

    /// SimCollideMode.per_body: max 1 hit per body pair
    fn perBody(context: ?*anyopaque, allocator: std.mem.Allocator, body1: *const Body, body2: *const Body, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, collide_shape_settings: *CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) std.mem.Allocator.Error!void {
        _ = .{ context, allocator, shape_filter };

        var any_hit_collector = AnyHitCollector.init();
        defer any_hit_collector.deinit();
        const part1: SubShapeIDCreator = .{};
        const part2: SubShapeIDCreator = .{};
        zolt.CollisionDispatch.collideShapeVsShape(body1.getShape(), body2.getShape(), Vec3.one(), Vec3.one(), center_of_mass_transform1, center_of_mass_transform2, part1, part2, collide_shape_settings, &any_hit_collector.base, &.{});
        if (any_hit_collector.hadHit())
            collector.addHit(&any_hit_collector.hit);
    }

    /// SimCollideMode.per_leaf: max 1 hit per leaf shape pair
    fn perLeaf(context: ?*anyopaque, allocator: std.mem.Allocator, body1: *const Body, body2: *const Body, center_of_mass_transform1: Mat44, center_of_mass_transform2: Mat44, collide_shape_settings: *CollideShapeSettings, collector: *CollideShapeCollector, shape_filter: *const ShapeFilter) std.mem.Allocator.Error!void {
        _ = context;

        const part1: SubShapeIDCreator = .{};
        const part2: SubShapeIDCreator = .{};
        try zolt.collideShapeVsShapePerLeaf(AnyHitCollector, allocator, body1.getShape(), body2.getShape(), Vec3.one(), Vec3.one(), center_of_mass_transform1, center_of_mass_transform2, part1, part2, collide_shape_settings, collector, shape_filter);
    }
};

/// Wraps the job system of a world and counts the FindCollisions jobs that are spawned from inside jobs
/// (TrySpawnJobFindCollisions creates them without dependencies, the job graph with dependencies) and the
/// FindCCDContacts jobs (PostIntegrateVelocity creates them from inside its job).
/// Phase 9 (Expect.soft_body_ccd): in a world with soft bodies (`soft_bodies_of`), the ResolveCCDContacts and
/// SoftBodyPrepare jobs run inside `runWrapped`. It counts the steps in which the soft bodies change between the start
/// of ResolveCCDContacts and the start of SoftBodyPrepare. Only the CCD response against a soft body face writes the
/// vertex velocities and the CCD contact marks there: the jobs in between (ContactRemovedCallbacks,
/// SolvePositionConstraints, BodySetIslandIndex) do not touch the soft bodies. The end of ResolveCCDContacts is too
/// late: its job releases SolvePositionConstraints, and the single threaded job system runs the jobs that follow
/// inside that call.
const SpawnCountingJobSystem = struct {
    inner: JobSystem,
    num_spawned_find_collisions: std.atomic.Value(u32) = .init(0),
    num_find_ccd_contacts: std.atomic.Value(u32) = .init(0),
    /// Phase 9: the system whose soft bodies `runWrapped` looks at (null: the scene has no soft bodies)
    soft_bodies_of: ?*const PhysicsSystem = null,
    /// The functions of the ResolveCCDContacts and SoftBodyPrepare jobs that wait to run (rings: an update creates one
    /// of each per collision step)
    wrapped_functions: [wrapped_names.len][16]JobSystem.JobFunction = undefined,
    num_wrapped: [wrapped_names.len]std.atomic.Value(u32) = @splat(.init(0)),
    /// The hash of the soft bodies at the start of the last ResolveCCDContacts job (the steps run one after the other,
    /// SoftBodyPrepare of a step depends on its ResolveCCDContacts)
    soft_bodies_before_ccd: ?u32 = null,
    /// Number of steps in which the CCD response changed a soft body
    num_soft_body_ccd_responses: std.atomic.Value(u32) = .init(0),

    const wrapped_names = [_][]const u8{ "ResolveCCDContacts", "SoftBodyPrepare" };

    fn jobSystem(self: *SpawnCountingJobSystem) JobSystem {
        return .init(self);
    }

    pub fn getMaxConcurrency(self: *const SpawnCountingJobSystem) i32 {
        return self.inner.getMaxConcurrency();
    }

    pub fn createJob(self: *SpawnCountingJobSystem, job_name: []const u8, color: Color, job_function: JobSystem.JobFunction, opts: struct { num_dependencies: u32 = 0 }) error{OutOfMemory}!JobSystem.JobHandle {
        var function = job_function;
        if (self.soft_bodies_of != null) {
            for (wrapped_names, 0..) |name, kind| {
                if (!std.mem.eql(u8, job_name, name)) continue;
                const slot: u32 = self.num_wrapped[kind].fetchAdd(1, .monotonic) % @as(u32, self.wrapped_functions[kind].len);
                self.wrapped_functions[kind][slot] = job_function;
                function = .init(runWrapped, .{ self, @as(u32, @intCast(kind)), slot });
            }
        }
        const handle = try self.inner.createJob(job_name, color, function, .{ .num_dependencies = opts.num_dependencies });
        if (opts.num_dependencies == 0 and std.mem.eql(u8, job_name, "FindCollisions"))
            _ = self.num_spawned_find_collisions.fetchAdd(1, .monotonic);
        if (std.mem.eql(u8, job_name, "FindCCDContacts"))
            _ = self.num_find_ccd_contacts.fetchAdd(1, .monotonic);
        return handle;
    }

    /// A wrapped job (`kind` indexes `wrapped_names`): ResolveCCDContacts hashes the soft bodies first, SoftBodyPrepare
    /// first compares them with that hash
    fn runWrapped(self: *SpawnCountingJobSystem, kind: u32, slot: u32) void {
        if (kind == 0) {
            self.soft_bodies_before_ccd = self.hashSoftBodies();
        } else if (self.soft_bodies_before_ccd) |before| {
            if (self.hashSoftBodies() != before)
                _ = self.num_soft_body_ccd_responses.fetchAdd(1, .monotonic);
            self.soft_bodies_before_ccd = null;
        }
        self.wrapped_functions[kind][slot].call();
    }

    /// A hash of what the CCD response writes to the soft bodies: the vertex velocities and the CCD contact marks
    fn hashSoftBodies(self: *const SpawnCountingJobSystem) u32 {
        var hash = SimScenes.hash_seed;
        for (self.soft_bodies_of.?.body_manager.getBodies()) |slot| {
            if (!slot.isValidBodyPointer()) continue;
            const body = slot.getBody();
            if (!body.isSoftBody()) continue;
            for (zolt.SoftBodyMotionProperties.fromBaseConst(body.getMotionProperties()).getVertices()) |*v| {
                const velocity = [3]f32{ v.velocity.getX(), v.velocity.getY(), v.velocity.getZ() };
                hash = SimScenes.hashBytes(hash, std.mem.asBytes(&velocity));
                hash = SimScenes.hashBytes(hash, std.mem.asBytes(&v.colliding_shape_index));
                hash = SimScenes.hashBytes(hash, std.mem.asBytes(&v.has_contact));
            }
        }
        return hash;
    }

    pub fn createBarrier(self: *SpawnCountingJobSystem) ?*JobSystem.Barrier {
        return self.inner.createBarrier();
    }

    pub fn destroyBarrier(self: *SpawnCountingJobSystem, barrier: *JobSystem.Barrier) void {
        self.inner.destroyBarrier(barrier);
    }

    pub fn waitForJobs(self: *SpawnCountingJobSystem, barrier: *JobSystem.Barrier) void {
        self.inner.waitForJobs(barrier);
    }

    pub fn queueJob(self: *SpawnCountingJobSystem, job: *JobSystem.Job) void {
        self.inner.vtable.queueJob(self.inner.ptr, job); // The jobs belong to `inner` (they queue themselves there), not reached
    }

    pub fn queueJobs(self: *SpawnCountingJobSystem, jobs: []const *JobSystem.Job) void {
        self.inner.vtable.queueJobs(self.inner.ptr, jobs); // The jobs belong to `inner` (they queue themselves there), not reached
    }

    pub fn freeJob(self: *SpawnCountingJobSystem, job: *JobSystem.Job) void {
        self.inner.vtable.freeJob(self.inner.ptr, job); // The jobs belong to `inner` (they queue themselves there), not reached
    }
};

/// SimStepListenerImpl of SimulationReference.cpp
const StepListenerImpl = struct {
    world: *ZoltWorld,
    index: u32,
    num_calls: u32 = 0,

    pub fn onStep(self: *StepListenerImpl, context: *const PhysicsStepListenerContext) void {
        var r: SimRecord = .{ .kind = @intFromEnum(RecordKind.step) };
        r.ids[0] = self.index;
        r.ids[1] = self.num_calls;
        self.num_calls += 1;
        r.data[0] = floatBits(context.delta_time);
        r.data[1] = @intFromBool(context.is_first_step);
        r.data[2] = @intFromBool(context.is_last_step);
        self.world.recorder.add(r);

        // Change the velocity of the body (StepListenerMode), all bodies are locked: the no-lock interface
        const desc = &self.world.desc.step_listeners[self.index];
        if (desc.body == invalid_handle)
            return;
        const id = self.world.getHandle(desc.body);
        const bi = context.physics_system.getBodyInterfaceNoLockMut();
        const dt = context.delta_time;
        const delta = Vec3.init(desc.f[1], desc.f[2], desc.f[3]).mulScalar(dt);
        switch (@as(StepListenerMode, @enumFromInt(desc.mode))) {
            .linear_velocity => bi.setLinearVelocity(id, bi.getLinearVelocity(id).mulScalar(desc.f[0]).add(delta)),
            .angular_velocity => bi.setAngularVelocity(id, bi.getAngularVelocity(id).mulScalar(desc.f[0]).add(delta)),
            .add_linear_velocity => bi.addLinearVelocity(id, delta),
            .record_only => {},
        }
    }
};

// -------------------------------------------------------------------------------------------------------------------
// The Zolt side

const JobKind = union(enum) {
    single_threaded,
    thread_pool: i32, // number of worker threads
};

/// A job system of either kind, initialized in place
const AnyJobSystem = struct {
    single: JobSystemSingleThreaded = .empty,
    pool: JobSystemThreadPool = .empty,
    is_pool: bool = false,

    fn init(self: *AnyJobSystem, allocator: Allocator, kind: JobKind) !void {
        switch (kind) {
            .single_threaded => self.single = try .init(allocator, io, zolt.physics_settings.max_physics_jobs),
            .thread_pool => |n| {
                self.is_pool = true;
                try self.pool.init(allocator, io, zolt.physics_settings.max_physics_jobs, zolt.physics_settings.max_physics_barriers, .{ .num_threads = n });
            },
        }
    }

    fn deinit(self: *AnyJobSystem) void {
        if (self.is_pool) self.pool.deinit() else self.single.deinit();
    }

    fn jobSystem(self: *AnyJobSystem) JobSystem {
        return if (self.is_pool) self.pool.jobSystem() else self.single.jobSystem();
    }
};

/// SimWorld of SimulationReference.cpp: a PhysicsSystem built from a scene description (heap allocated, the system
/// must not move)
const ZoltWorld = struct {
    allocator: Allocator,
    desc: *const SimSceneDesc,
    temp: TempAllocatorImpl,
    jobs: AnyJobSystem = .{},
    broad_phase_layer_interface: TestLayers.BPLayerInterfaceImpl,
    object_vs_broad_phase_layer_filter: TestLayers.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_layer_pair_filter: TestLayers.ObjectLayerPairFilterImpl = .{},
    recorder: Recorder,
    activation_listener: ActivationListenerImpl = undefined,
    contact_listener: ContactListenerImpl = undefined,
    sphere_filter: SphereFilter = .{},
    /// The job system the updates run on (counts the spawned FindCollisions jobs, see `jobSystem`)
    counting_jobs: SpawnCountingJobSystem = undefined,
    /// Number of OnBodyDeactivated calls while the LargeIslandSplitter held split islands (Expect.large_island)
    num_large_island_deactivations: std.atomic.Value(u32) = .init(0),
    system: PhysicsSystem = .empty,
    shapes: std.ArrayList(RefConst(Shape)) = .empty,
    handles: std.ArrayList(BodyID) = .empty,
    step_listeners: []StepListenerImpl = &.{},
    /// Which step listeners are added to the system (destroy removes them like ~SimWorld)
    step_listener_added: []bool = &.{},
    /// The constraints of the constraint handles (one reference each, also for constraints that are not in the system)
    constraint_handles: std.ArrayList(Ref(Constraint)) = .empty,
    /// Phase 7: the CharacterContactListener records of the character virtuals (`CharacterRecords` builds them, compared
    /// in call order)
    character_recorder: Recorder,
    /// Phase 7: the objects of the slots (characters, character virtuals, ragdolls, vehicles)
    phase7: Phase7Storage = .{},
    /// Phase 9: the shared settings of the soft body descriptions and the soft body contact listener
    soft_body: SoftBodyStorage = .{},

    fn create(allocator: Allocator, desc: *const SimSceneDesc, job_kind: JobKind) !*ZoltWorld {
        const self = try allocator.create(ZoltWorld);
        errdefer allocator.destroy(self);
        self.* = .{
            .allocator = allocator,
            .desc = desc,
            .temp = try .init(allocator, desc.temp_allocator_size),
            .broad_phase_layer_interface = .init(),
            .recorder = .{ .allocator = allocator },
            .character_recorder = .{ .allocator = allocator },
        };
        errdefer self.temp.deinit();
        errdefer self.recorder.deinit();
        errdefer self.character_recorder.deinit();
        try self.jobs.init(allocator, job_kind);
        errdefer self.jobs.deinit();

        try self.system.init(allocator, io, desc.max_bodies, desc.num_body_mutexes, desc.max_body_pairs, desc.max_contact_constraints, &self.broad_phase_layer_interface.base, &self.object_vs_broad_phase_layer_filter.base, &self.object_layer_pair_filter.base, .{});
        errdefer self.system.deinit();
        const settings = toPhysicsSettings(&desc.settings);
        self.system.setPhysicsSettings(&settings);
        self.system.setGravity(load3(&desc.gravity));
        self.activation_listener = .{ .recorder = &self.recorder, .world = self };
        self.contact_listener = .{ .recorder = &self.recorder, .mode = @enumFromInt(desc.contact_listener_mode) };
        self.system.setBodyActivationListener(.init(&self.activation_listener));
        self.system.setContactListener(.init(&self.contact_listener));
        switch (@as(CombineMode, @enumFromInt(desc.combine_mode))) {
            .default => {},
            .custom => {
                self.system.setCombineFriction(CustomCombine.friction);
                self.system.setCombineRestitution(CustomCombine.restitution);
            },
        }
        switch (@as(SimShapeFilterMode, @enumFromInt(desc.sim_shape_filter_mode))) {
            .none => {},
            .filter_spheres => self.system.setSimShapeFilter(&self.sphere_filter.base),
        }
        switch (@as(SimCollideMode, @enumFromInt(desc.sim_collide_mode))) {
            .default => {},
            .all => self.system.setSimCollideBodyVsBody(.{ .function = SimCollide.all }),
            .per_leaf => self.system.setSimCollideBodyVsBody(.{ .function = SimCollide.perLeaf }),
            .per_body => self.system.setSimCollideBodyVsBody(.{ .function = SimCollide.perBody }),
        }
        self.counting_jobs = .{ .inner = self.jobs.jobSystem(), .soft_bodies_of = if (desc.num_soft_bodies > 0) &self.system else null };

        errdefer self.shapes.deinit(allocator);
        errdefer for (self.shapes.items) |*s| s.deinit();
        try self.shapes.ensureTotalCapacity(allocator, desc.num_shapes);
        for (desc.shapes[0..desc.num_shapes]) |*shape|
            self.shapes.appendAssumeCapacity(try self.makeShape(shape));

        // Phase 9: the shared settings of every SimSoftBodySettings (the soft bodies of a description share them), the
        // soft body contact listener records with the other listeners (installed by set_soft_body_contact_listener)
        errdefer self.soft_body.deinit(allocator);
        try self.soft_body.settings.ensureTotalCapacity(allocator, desc.num_soft_body_settings);
        for (desc.soft_body_settings[0..desc.num_soft_body_settings]) |*s|
            self.soft_body.settings.appendAssumeCapacity(try SoftBodySlot.makeSettings(allocator, desc, s));
        self.soft_body.listener = .{ .recorder = &self.recorder };

        self.step_listeners = try allocator.alloc(StepListenerImpl, desc.num_step_listeners);
        errdefer allocator.free(self.step_listeners);
        for (self.step_listeners, 0..) |*l, i|
            l.* = .{ .world = self, .index = @intCast(i) };
        self.step_listener_added = try allocator.alloc(bool, desc.num_step_listeners);
        @memset(self.step_listener_added, false);
        return self;
    }

    fn destroy(self: *ZoltWorld) void {
        const allocator = self.allocator;
        // The records of the destruction are not compared: the body activation listener goes first, so that removing the
        // active bodies of the Phase 7 objects (OnBodyDeactivated) allocates no record. Otherwise the creation sweep sees
        // the failed allocation of such a record as a swallowed out of memory error (nothing checks the recorder after
        // the destruction).
        self.system.setBodyActivationListener(null);
        // Phase 7 objects first (~SimWorld of the .cpp): they remove their bodies and constraints from the system
        self.destroyPhase7();
        for (self.step_listeners, self.step_listener_added) |*listener, added| {
            if (added) self.system.removeStepListener(.init(listener));
        }
        self.system.deinit();
        self.soft_body.deinit(allocator); // The soft bodies released their references with the system
        for (self.constraint_handles.items) |*c| c.deinit();
        self.constraint_handles.deinit(allocator);
        for (self.shapes.items) |*s| s.deinit();
        self.shapes.deinit(allocator);
        self.handles.deinit(allocator);
        allocator.free(self.step_listeners);
        allocator.free(self.step_listener_added);
        self.jobs.deinit();
        self.recorder.deinit();
        self.character_recorder.deinit();
        self.temp.deinit();
        allocator.destroy(self);
    }

    /// The job system of the updates (the world's job system, counting the spawned FindCollisions jobs)
    fn jobSystem(self: *ZoltWorld) JobSystem {
        return self.counting_jobs.jobSystem();
    }

    fn getHandle(self: *const ZoltWorld, handle: u32) BodyID {
        return if (handle < self.handles.items.len) self.handles.items[handle] else .invalid;
    }

    /// MakeShape of SimulationReference.cpp
    fn makeShape(self: *const ZoltWorld, shape: *const SimShape) !RefConst(Shape) {
        const allocator = self.allocator;
        var compound: ?*zolt.CompoundShapeSettings = null;
        var settings = Ref(ShapeSettings).init(try self.makeShapeSettings(shape, &compound));
        defer settings.deinit();
        if (compound) |c| {
            for (self.desc.children[shape.child..][0..shape.count]) |*child|
                try c.addShapePtr(load3(&child.position), loadQuat(&child.rotation), self.shapes.items[child.shape].get().?, .{ .user_data = child.user_data });
        }
        var result = try settings.get().?.createShape(allocator);
        defer result.deinit();
        if (!result.isValid()) {
            std.debug.print("SimulationParity: shape error: {s}\n", .{result.getError()});
            return error.InvalidShape;
        }
        return RefConst(Shape).init(result.getPtr().?);
    }

    /// The settings of a shape (without reference; a compound gets its children after the caller took a reference)
    fn makeShapeSettings(self: *const ZoltWorld, shape: *const SimShape, out_compound: *?*zolt.CompoundShapeSettings) !*ShapeSettings {
        const allocator = self.allocator;
        const p = &shape.params;
        const floats = self.desc.floats;
        return switch (@as(ShapeKind, @enumFromInt(shape.kind))) {
            .sphere => convex(try zolt.SphereShapeSettings.create(allocator, p[0], .{}), shape),
            .box => convex(try zolt.BoxShapeSettings.create(allocator, load3(p[0..3]), .{ .convex_radius = p[3] }), shape),
            .capsule => convex(try zolt.CapsuleShapeSettings.create(allocator, p[0], p[1], .{}), shape),
            .tapered_capsule => convex(try zolt.TaperedCapsuleShapeSettings.create(allocator, p[0], p[1], p[2], .{}), shape),
            .cylinder => convex(try zolt.CylinderShapeSettings.create(allocator, p[0], p[1], .{ .convex_radius = p[3] }), shape),
            .tapered_cylinder => convex(try zolt.TaperedCylinderShapeSettings.create(allocator, p[0], p[1], p[2], .{ .convex_radius = p[3] }), shape),
            .convex_hull => blk: {
                const points = try allocator.alloc(Vec3, shape.count);
                defer allocator.free(points);
                for (points, 0..) |*point, i|
                    point.* = load3(floats[shape.data + 3 * i ..][0..3]);
                break :blk convex(try zolt.ConvexHullShapeSettings.create(allocator, points, .{ .max_convex_radius = p[3] }), shape);
            },
            .triangle => convex(try zolt.TriangleShapeSettings.create(allocator, load3(p[0..3]), load3(p[3..6]), load3(p[6..9]), .{ .convex_radius = p[9] }), shape),
            .plane => (try zolt.PlaneShapeSettings.create(allocator, zolt.Plane.init(load3(p[0..3]), p[3]), .{ .half_extent = p[4] })).asShapeSettings(),
            .mesh => blk: {
                const triangles = try allocator.alloc(zolt.Triangle, shape.count);
                defer allocator.free(triangles);
                for (triangles, 0..) |*t, i| {
                    const v = floats[shape.data + 9 * i ..][0..9];
                    t.* = .init(load3(v[0..3]), load3(v[3..6]), load3(v[6..9]), .{});
                }
                break :blk (try zolt.MeshShapeSettings.create(allocator, triangles, .{})).asShapeSettings();
            },
            .indexed_mesh => blk: {
                const vertices = try allocator.alloc(zolt.Float3, shape.count);
                defer allocator.free(vertices);
                for (vertices, 0..) |*v, i| {
                    const f = floats[shape.data + 3 * i ..][0..3];
                    v.* = .init(f[0], f[1], f[2]);
                }
                const indices = floats[shape.data + 3 * shape.count ..];
                const triangles = try allocator.alloc(zolt.IndexedTriangle, @intFromFloat(p[0]));
                defer allocator.free(triangles);
                for (triangles, 0..) |*t, i|
                    t.* = .init(@intFromFloat(indices[3 * i]), @intFromFloat(indices[3 * i + 1]), @intFromFloat(indices[3 * i + 2]), .{});
                const mesh = try zolt.MeshShapeSettings.createIndexed(allocator, vertices, triangles, .{});
                mesh.max_triangles_per_leaf = @intFromFloat(p[1]);
                break :blk mesh.asShapeSettings();
            },
            .height_field => (try zolt.HeightFieldShapeSettings.create(allocator, floats[shape.data..][0 .. @as(usize, shape.count) * shape.count], load3(p[0..3]), load3(p[3..6]), shape.count, .{})).asShapeSettings(),
            .empty => (try zolt.EmptyShapeSettings.create(allocator, load3(p[0..3]))).asShapeSettings(),
            .rotated_translated => (try zolt.RotatedTranslatedShapeSettings.createPtr(allocator, load3(p[0..3]), loadQuat(&shape.rotation), self.shapes.items[shape.child].get().?)).asShapeSettings(),
            .scaled => (try zolt.ScaledShapeSettings.createPtr(allocator, self.shapes.items[shape.child].get().?, load3(p[0..3]))).asShapeSettings(),
            .offset_center_of_mass => (try zolt.OffsetCenterOfMassShapeSettings.createPtr(allocator, load3(p[0..3]), self.shapes.items[shape.child].get().?)).asShapeSettings(),
            .static_compound => blk: {
                const s = try zolt.StaticCompoundShapeSettings.create(allocator);
                out_compound.* = &s.base;
                break :blk s.asShapeSettings();
            },
            .mutable_compound => blk: {
                const s = try zolt.MutableCompoundShapeSettings.create(allocator);
                out_compound.* = &s.base;
                break :blk s.asShapeSettings();
            },
        };
    }

    /// A convex shape's settings with the density of the scene
    fn convex(settings: anytype, shape: *const SimShape) *ShapeSettings {
        if (shape.density != 0.0)
            settings.base.setDensity(shape.density);
        return settings.asShapeSettings();
    }

    /// MakeBodyCreationSettings of SimulationReference.cpp (the caller deinits the result)
    fn makeBodyCreationSettings(self: *const ZoltWorld, b: *const SimBody) BodyCreationSettings {
        var s: BodyCreationSettings = .initWithShape(self.shapes.items[b.shape].get().?, loadR3(&b.position), loadQuat(&b.rotation), @enumFromInt(b.motion_type), @intCast(b.object_layer));
        s.linear_velocity = load3(&b.linear_velocity);
        s.angular_velocity = load3(&b.angular_velocity);
        s.user_data = b.user_data;
        s.collision_group = .init(null, b.collision_group_id, b.collision_sub_group_id);
        s.allowed_dofs = .fromInt(@intCast(b.allowed_dofs));
        s.allow_dynamic_or_kinematic = b.allow_dynamic_or_kinematic != 0;
        s.is_sensor = b.is_sensor != 0;
        s.collide_kinematic_vs_non_dynamic = b.collide_kinematic_vs_non_dynamic != 0;
        s.use_manifold_reduction = b.use_manifold_reduction != 0;
        s.apply_gyroscopic_force = b.apply_gyroscopic_force != 0;
        s.motion_quality = @enumFromInt(b.motion_quality);
        s.enhanced_internal_edge_removal = b.enhanced_internal_edge_removal != 0;
        s.allow_sleeping = b.allow_sleeping != 0;
        s.friction = b.friction;
        s.restitution = b.restitution;
        s.linear_damping = b.linear_damping;
        s.angular_damping = b.angular_damping;
        s.max_linear_velocity = b.max_linear_velocity;
        s.max_angular_velocity = b.max_angular_velocity;
        s.gravity_factor = b.gravity_factor;
        s.num_velocity_steps_override = b.num_velocity_steps_override;
        s.num_position_steps_override = b.num_position_steps_override;
        s.override_mass_properties = @enumFromInt(b.override_mass_properties);
        s.inertia_multiplier = b.inertia_multiplier;
        s.mass_properties_override.mass = b.mass;
        const m = &b.inertia;
        s.mass_properties_override.inertia = Mat44.init(Vec4.init(m[0], m[1], m[2], m[3]), Vec4.init(m[4], m[5], m[6], m[7]), Vec4.init(m[8], m[9], m[10], m[11]), Vec4.init(m[12], m[13], m[14], m[15]));
        return s;
    }

    fn getHandles(self: *const ZoltWorld, op: *const SimOp) ![]BodyID {
        const ids = try self.allocator.alloc(BodyID, op.count);
        for (ids, 0..) |*id, i|
            id.* = self.getHandle(op.body + @as(u32, @intCast(i)));
        return ids;
    }

    /// ApplyOp of SimulationReference.cpp. A record of a callback of the operation that could not be stored (out of
    /// memory: the callbacks cannot return errors, Recorder.add latches the failure) fails the operation, also when the
    /// records are not taken afterwards (createScene under checkAllAllocationFailures).
    fn applyOp(self: *ZoltWorld, op: *const SimOp) !u32 {
        const result = try self.applyOpInner(op);
        try self.recorder.checkError();
        try self.character_recorder.checkError();
        return result;
    }

    fn applyOpInner(self: *ZoltWorld, op: *const SimOp) !u32 {
        const allocator = self.allocator;
        const bi = self.system.getBodyInterfaceMut();
        const id = self.getHandle(op.body);
        const activation: Activation = if (op.activation == 0) .activate else .dont_activate;
        const f = &op.f;
        const r = loadR3(&op.r);
        const kind: OpKind = @enumFromInt(op.kind);

        switch (kind) {
            .create_body, .create_and_add_body, .create_body_no_add => {
                try self.handles.ensureUnusedCapacity(allocator, 1);
                var settings = self.makeBodyCreationSettings(&self.desc.bodies[op.index]);
                defer settings.deinit();
                var new_id: BodyID = .invalid;
                if (kind == .create_and_add_body) {
                    new_id = try bi.createAndAddBody(&settings, activation);
                } else if (try bi.createBody(&settings)) |body| {
                    new_id = body.getID();
                    if (kind == .create_body)
                        bi.addBody(new_id, activation) catch |err| {
                            bi.destroyBody(new_id);
                            return err;
                        };
                }
                self.handles.appendAssumeCapacity(new_id);
                return new_id.getIndexAndSequenceNumber();
            },

            .add_body => try bi.addBody(id, activation),
            .remove_body => bi.removeBody(id),
            .destroy_body => {
                bi.destroyBody(id);
                return 0;
            },

            .add_bodies => {
                const ids = try self.getHandles(op);
                defer allocator.free(ids);
                const state = try bi.addBodiesPrepare(ids);
                bi.addBodiesFinalize(ids, state, activation);
                return 0;
            },

            .remove_bodies => {
                const ids = try self.getHandles(op);
                defer allocator.free(ids);
                bi.removeBodies(ids);
                return 0;
            },

            .activate_body => bi.activateBody(id),
            .deactivate_body => bi.deactivateBody(id),

            .activate_bodies, .deactivate_bodies => {
                const ids = try self.getHandles(op);
                defer allocator.free(ids);
                if (kind == .activate_bodies) bi.activateBodies(ids) else bi.deactivateBodies(ids);
                return 0;
            },

            .add_force => bi.addForce(id, load3(f[0..3]), .{ .activation = activation }),
            .add_force_at_position => bi.addForceAtPosition(id, load3(f[0..3]), r, .{ .activation = activation }),
            .add_torque => bi.addTorque(id, load3(f[0..3]), .{ .activation = activation }),
            .add_force_and_torque => bi.addForceAndTorque(id, load3(f[0..3]), load3(f[3..6]), .{ .activation = activation }),
            .add_impulse => bi.addImpulse(id, load3(f[0..3])),
            .add_impulse_at_position => bi.addImpulseAtPosition(id, load3(f[0..3]), r),
            .add_angular_impulse => bi.addAngularImpulse(id, load3(f[0..3])),
            .set_linear_velocity => bi.setLinearVelocity(id, load3(f[0..3])),
            .set_angular_velocity => bi.setAngularVelocity(id, load3(f[0..3])),
            .set_linear_and_angular_velocity => bi.setLinearAndAngularVelocity(id, load3(f[0..3]), load3(f[3..6])),
            .add_linear_velocity => bi.addLinearVelocity(id, load3(f[0..3])),
            .add_linear_and_angular_velocity => bi.addLinearAndAngularVelocity(id, load3(f[0..3]), load3(f[3..6])),
            .set_position => bi.setPosition(id, r, activation),
            .set_rotation => bi.setRotation(id, loadQuat(f[0..4]), activation),
            .set_position_and_rotation => bi.setPositionAndRotation(id, r, loadQuat(f[0..4]), activation),
            .set_position_and_rotation_when_changed => bi.setPositionAndRotationWhenChanged(id, r, loadQuat(f[0..4]), activation),
            .set_position_rotation_and_velocity => bi.setPositionRotationAndVelocity(id, r, loadQuat(f[0..4]), load3(f[4..7]), load3(f[7..10])),
            .move_kinematic => bi.moveKinematic(id, r, loadQuat(f[0..4]), f[4]),
            .set_motion_type => bi.setMotionType(id, @enumFromInt(op.index), activation),
            .set_motion_quality => bi.setMotionQuality(id, @enumFromInt(op.index)),
            .set_gravity_factor => bi.setGravityFactor(id, f[0]),
            .set_max_linear_velocity => bi.setMaxLinearVelocity(id, f[0]),
            .set_max_angular_velocity => bi.setMaxAngularVelocity(id, f[0]),
            .reset_sleep_timer => bi.resetSleepTimer(id),
            .set_object_layer => try bi.setObjectLayer(id, @intCast(op.index)),
            .set_user_data => bi.setUserData(id, @as(u64, op.index) | (@as(u64, op.count) << 32)),
            .set_friction_and_restitution => {
                bi.setFriction(id, f[0]);
                bi.setRestitution(id, f[1]);
            },

            .set_gravity => {
                self.system.setGravity(load3(f[0..3]));
                return 0;
            },
            .set_physics_settings => {
                const settings = toPhysicsSettings(&self.desc.physics_settings[op.index]);
                self.system.setPhysicsSettings(&settings);
                return 0;
            },
            .add_step_listener => {
                try self.system.addStepListener(.init(&self.step_listeners[op.index]));
                self.step_listener_added[op.index] = true;
                return 0;
            },
            .remove_step_listener => {
                self.system.removeStepListener(.init(&self.step_listeners[op.index]));
                self.step_listener_added[op.index] = false;
                return 0;
            },
            .optimize_broad_phase => {
                try self.system.optimizeBroadPhase();
                return 0;
            },
            .set_shape => try bi.setShape(id, self.shapes.items[op.index].get().?, op.count != 0, activation),
            .activate_bodies_in_aabox => {
                try bi.activateBodiesInAABox(&zolt.AABox.init(load3(f[0..3]), load3(f[3..6])), &.{}, &.{});
                return 0;
            },
            .apply_buoyancy_impulse => {
                const applied = bi.applyBuoyancyImpulse(id, r, load3(f[0..3]), f[3], f[4], f[5], load3(f[6..9]), load3(f[9..12]), f[12], .{});
                return (@as(u32, @intFromBool(applied)) << 2) | @as(u32, @intFromBool(bi.isActive(id))) | (@as(u32, @intFromBool(bi.isAdded(id))) << 1);
            },
            .expect_body_state => {}, // Changes nothing, the driver checks the result
            .set_use_manifold_reduction => try bi.setUseManifoldReduction(id, op.count != 0),
            .apply_body_creation_settings => {
                var settings = self.makeBodyCreationSettings(&self.desc.bodies[op.index]);
                defer settings.deinit();
                var lock: zolt.BodyLockWrite = .init(self.system.getBodyLockInterface(), id);
                defer lock.deinit();
                if (!lock.succeeded()) return error.SceneNotSupported;
                try lock.getBody().applyBodyCreationSettings(allocator, &settings, self.system.getBroadPhaseLayerInterface());
            },
            .were_bodies_in_contact => {
                const id2 = self.getHandle(op.index);
                return @as(u32, @intFromBool(self.system.wereBodiesInContact(id, id2))) | (@as(u32, @intFromBool(self.system.wereBodiesInContact(id2, id))) << 1);
            },
            .set_collide_kinematic_vs_non_dynamic => {
                var lock: zolt.BodyLockWrite = .init(self.system.getBodyLockInterface(), id);
                defer lock.deinit();
                if (lock.succeeded())
                    lock.getBody().setCollideKinematicVsNonDynamic(op.count != 0);
            },
            .create_constraint => {
                // CreateConstraint(constraints[index]) (+ SetConstraints) + AddConstraint
                try self.constraint_handles.ensureUnusedCapacity(allocator, 1);
                const constraint = try self.createConstraint(op.index);
                errdefer {
                    self.constraint_handles.items[self.constraint_handles.items.len - 1].deinit();
                    self.constraint_handles.items.len -= 1;
                }
                try self.system.addConstraint(&constraint.base);
                return self.system.constraint_manager.getNumConstraints();
            },

            .set_update, .snapshot, .restore_cross => unreachable, // Driver operations

            // The constraint operations (Phase 6) and the character, ragdoll and vehicle operations (Phase 7): every OpKind
            // in a ConstraintOpRange / Phase7OpRange, without a list that the worker groups would extend at the same place
            inline else => |k| {
                if (comptime ConstraintOpRange.of(@intFromEnum(k)) != null) {
                    return self.applyConstraintOp(op);
                } else if (comptime Phase7OpRange.of(@intFromEnum(k)) != null) {
                    return self.applyPhase7Op(op);
                } else if (comptime SimScenes.isSoftBodyOp(@intFromEnum(k))) {
                    return SoftBodySlot.applyOp(self, op);
                } else {
                    @compileError("operation without prong: " ++ @tagName(k));
                }
            },
        }

        // Body operations return IsActive and IsAdded of the body afterwards
        return @as(u32, @intFromBool(bi.isActive(id))) | (@as(u32, @intFromBool(bi.isAdded(id))) << 1);
    }

    /// The constraint of a constraint handle (null: not created)
    fn getConstraint(self: *const ZoltWorld, handle: u32) ?*TwoBodyConstraint {
        if (handle >= self.constraint_handles.items.len) return null;
        return zolt.virtual.downcast(TwoBodyConstraint, self.constraint_handles.items[handle].get().?);
    }

    /// CreateConstraint of description `desc_index` (+ SetConstraints of a Gear / RackAndPinion) as the next constraint
    /// handle (CreateConstraint of SimulationReference.cpp; the caller reserved the handle)
    fn createConstraint(self: *ZoltWorld, desc_index: u32) !*TwoBodyConstraint {
        std.debug.assert(self.constraint_handles.unusedCapacitySlice().len > 0);
        const c = &self.desc.constraints[desc_index];
        var settings = try makeConstraintSettings(self.allocator, c);
        defer settings.deinit();
        const constraint = try self.system.getBodyInterfaceMut().createConstraint(zolt.virtual.downcast(zolt.TwoBodyConstraintSettings, settings.get().?), self.getHandle(c.body1), self.getHandle(c.body2));
        var ref = Ref(Constraint).init(&constraint.base); // The harness' reference (Jolt: a Ref<TwoBodyConstraint> in mConstraintHandles)
        errdefer ref.deinit();
        if (c.constraint1 != invalid_handle or c.constraint2 != invalid_handle)
            try GearRackAndPinionSlot.setConstraints(self, constraint, c.constraint1, c.constraint2);
        self.constraint_handles.appendAssumeCapacity(ref);
        return constraint;
    }

    /// The constraints of the handles [op.index, op.index + op.i[0]) (the caller frees the slice)
    fn getConstraintHandles(self: *const ZoltWorld, op: *const SimOp) ![]*Constraint {
        const constraints = try self.allocator.alloc(*Constraint, op.i[0]);
        errdefer self.allocator.free(constraints);
        for (constraints, 0..) |*c, i|
            c.* = &(self.getConstraint(op.index + @as(u32, @intCast(i))) orelse return error.SceneError).base;
        return constraints;
    }

    /// A constraint operation (ApplyConstraintOp of SimulationReference.cpp): the generic ones here, the class ones in
    /// the slot of their ConstraintOpRange
    fn applyConstraintOp(self: *ZoltWorld, op: *const SimOp) !u32 {
        const allocator = self.allocator;
        const kind: OpKind = @enumFromInt(op.kind);
        const i = &op.i;
        const f = &op.f;

        // The operations that create constraints or change the constraints of the system
        switch (kind) {
            .create_constraint_no_add => {
                try self.constraint_handles.ensureUnusedCapacity(allocator, 1);
                return constraintState(&(try self.createConstraint(op.index)).base);
            },
            .remove_constraint => {
                self.system.removeConstraint(&(self.getConstraint(op.index) orelse return error.SceneError).base);
                return self.system.constraint_manager.getNumConstraints();
            },
            .add_constraint => {
                try self.system.addConstraint(&(self.getConstraint(op.index) orelse return error.SceneError).base);
                return self.system.constraint_manager.getNumConstraints();
            },
            .remove_constraints, .add_constraints => {
                const constraints = try self.getConstraintHandles(op);
                defer allocator.free(constraints);
                if (kind == .remove_constraints)
                    self.system.removeConstraints(constraints)
                else
                    try self.system.addConstraints(constraints);
                return self.system.constraint_manager.getNumConstraints();
            },
            else => {},
        }

        const c = self.getConstraint(op.index) orelse return error.SceneError;
        const range = ConstraintOpRange.of(op.kind).?;
        switch (range) {
            .generic => switch (kind) {
                .set_constraint_enabled => c.base.setEnabled(i[0] != 0),
                .set_constraint_priority => c.base.setConstraintPriority(i[0]),
                .set_constraint_steps_override => {
                    c.base.setNumVelocityStepsOverride(i[0]);
                    c.base.setNumPositionStepsOverride(i[1]);
                },
                .set_constraint_user_data => {
                    c.base.setUserData(@as(u64, i[0]) | (@as(u64, i[1]) << 32));
                    // The whole value read back: the ConstraintState bits and the state filter (bit 0) do not show it
                    const user_data = c.base.getUserData();
                    return SimScenes.hashBytes(SimScenes.hash_seed, std.mem.asBytes(&user_data));
                },
                .notify_constraint_shape_changed => c.base.notifyShapeChanged(self.getHandle(op.body), load3(f[0..3])),
                .reset_constraint_warm_start => c.base.resetWarmStart(),
                .activate_constraint => self.system.getBodyInterfaceMut().activateConstraint(c),
                .expect_constraint_state => {}, // Changes nothing, the driver checks the result
                .constraint_settings_hash => {
                    var settings = try c.base.getConstraintSettings();
                    defer settings.deinit();
                    var out: std.Io.Writer.Allocating = .init(allocator);
                    defer out.deinit();
                    var wrapper = zolt.StreamOutWrapper.init(&out.writer);
                    settings.get().?.saveBinaryState(wrapper.streamOut());
                    if (wrapper.isFailed()) return error.OutOfMemory;
                    return SimScenes.hashBytes(SimScenes.hash_seed, out.written());
                },
                else => return error.SceneError, // Handled above
            },
            .fixed_point => try FixedPointSlot.applyOp(self, op, c),
            .distance_pulley => try DistancePulleySlot.applyOp(self, op, c),
            .hinge => try HingeSlot.applyOp(self, op, c),
            .slider => try SliderSlot.applyOp(self, op, c),
            .cone_swing_twist => try ConeSwingTwistSlot.applyOp(self, op, c),
            .six_dof => try SixDOFSlot.applyOp(self, op, c),
            .path => try PathSlot.applyOp(self, op, c),
            .gear_rack_and_pinion => try GearRackAndPinionSlot.applyOp(self, op, c),
            .spare => return error.SceneNotSupported,
        }
        return constraintState(&c.base);
    }

    fn update(self: *ZoltWorld, delta_time: f32, collision_steps: u32, temp_allocator: TempAllocator, job_system: JobSystem) !u32 {
        const errors = try self.system.update(delta_time, collision_steps, temp_allocator, job_system);
        return errors.toInt();
    }

    fn saveState(self: *const ZoltWorld, state: StateRecorderState, out: *std.ArrayList(u8)) !void {
        var recorder = StateRecorderImpl.init(self.allocator);
        defer recorder.deinit();
        try self.system.saveState(recorder.recorder(), .{ .state = state });
        if (recorder.isFailed()) return error.OutOfMemory;
        out.clearRetainingCapacity();
        try out.appendSlice(self.allocator, recorder.getData());
    }

    fn restoreState(self: *ZoltWorld, bytes: []const u8) !bool {
        var recorder = StateRecorderImpl.init(self.allocator);
        defer recorder.deinit();
        recorder.writeBytes(bytes);
        if (recorder.isFailed()) return error.OutOfMemory;
        recorder.rewind();
        return self.system.restoreState(recorder.recorder(), .{});
    }

    fn takeRecords(self: *ZoltWorld, out: *std.ArrayList(SimRecord)) !void {
        try self.recorder.checkError();
        self.recorder.mutex.lock(io);
        defer self.recorder.mutex.unlock(io);
        try out.appendSlice(self.allocator, self.recorder.records.items);
        self.recorder.records.clearRetainingCapacity();
    }

    fn discardRecords(self: *ZoltWorld) void {
        self.recorder.records.clearRetainingCapacity();
    }

    /// The number of records of this kind that were not taken yet
    fn countRecords(self: *ZoltWorld, kind: RecordKind) u32 {
        self.recorder.mutex.lock(io);
        defer self.recorder.mutex.unlock(io);
        var n: u32 = 0;
        for (self.recorder.records.items) |r| {
            if (r.kind == @intFromEnum(kind)) n += 1;
        }
        return n;
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Phase 7: characters, ragdolls and vehicles (the slots, see "Characters, ragdolls and vehicles" below)

    /// A Phase 7 operation (ApplyObjectOp of SimulationReference.cpp): the slot of its Phase7OpRange. The operations of
    /// the vehicle variants get the vehicle of their handle from VehicleSlot and return its ConstraintState
    /// (ApplyVehicleOp of the .cpp)
    fn applyPhase7Op(self: *ZoltWorld, op: *const SimOp) !u32 {
        const kind: OpKind = @enumFromInt(op.kind);
        return switch (Phase7OpRange.of(op.kind).?) {
            .character => if (kind == .create_character) CharacterSlot.create(self, op) else CharacterSlot.applyOp(self, op),
            .character_virtual => if (kind == .create_character_virtual) CharacterVirtualSlot.create(self, op) else CharacterVirtualSlot.applyOp(self, op),
            .ragdoll => if (kind == .create_ragdoll) RagdollSlot.create(self, op) else RagdollSlot.applyOp(self, op),
            .vehicle => if (kind == .create_vehicle) VehicleSlot.create(self, op) else VehicleSlot.applyOp(self, op),
            .vehicle_tracked => {
                const vehicle = try VehicleSlot.vehicle(self, op.index);
                try TrackedVehicleSlot.applyVehicleOp(self, op, vehicle);
                return VehicleSlot.constraintState(vehicle);
            },
            .vehicle_motorcycle => {
                const vehicle = try VehicleSlot.vehicle(self, op.index);
                try MotorcycleSlot.applyVehicleOp(self, op, vehicle);
                return VehicleSlot.constraintState(vehicle);
            },
            .spare => error.SceneNotSupported,
        };
    }

    /// The object state (WriteObjectState of SimulationReference.cpp): every character, character virtual, ragdoll and
    /// vehicle in handle order, the destroyed ones skipped
    fn writeObjectState(self: *ZoltWorld, out: *std.ArrayList(u8)) !void {
        var writer: ObjectStateWriter = .{ .allocator = self.allocator, .bytes = out.* };
        defer out.* = writer.bytes;
        writer.bytes.clearRetainingCapacity();
        try CharacterSlot.writeState(self, &writer);
        try CharacterVirtualSlot.writeState(self, &writer);
        try RagdollSlot.writeState(self, &writer);
        try VehicleSlot.writeState(self, &writer);
    }

    /// The SaveState bytes of every character and character virtual in handle order (SaveCharacterStates of the .cpp)
    fn saveCharacterStates(self: *ZoltWorld, out: *std.ArrayList(u8)) !void {
        var recorder = StateRecorderImpl.init(self.allocator);
        defer recorder.deinit();
        try CharacterSlot.saveStates(self, &recorder);
        try CharacterVirtualSlot.saveStates(self, &recorder);
        if (recorder.isFailed()) return error.OutOfMemory;
        out.clearRetainingCapacity();
        try out.appendSlice(self.allocator, recorder.getData());
    }

    /// RestoreState of every character and character virtual from the bytes of `saveCharacterStates` (of either side):
    /// true when the bytes were read without error and completely (jolt_sim_restore_character_states of the .cpp: the
    /// stream sets eof only on a read past the end, so one more byte is read, which must fail at the end; no bytes: true
    /// when there are no characters)
    fn restoreCharacterStates(self: *ZoltWorld, bytes: []const u8) !bool {
        if (bytes.len == 0) {
            var states: std.ArrayList(u8) = .empty;
            defer states.deinit(self.allocator);
            try self.saveCharacterStates(&states);
            return states.items.len == 0;
        }
        var recorder = StateRecorderImpl.init(self.allocator);
        defer recorder.deinit();
        recorder.writeBytes(bytes);
        if (recorder.isFailed()) return error.OutOfMemory;
        recorder.rewind();
        try CharacterSlot.restoreStates(self, &recorder);
        try CharacterVirtualSlot.restoreStates(self, &recorder);
        const read = !recorder.isFailed();
        var extra: [1]u8 = undefined;
        recorder.readBytes(&extra); // Fails at the end of the bytes
        return read and recorder.isEOF();
    }

    fn takeCharacterRecords(self: *ZoltWorld, out: *std.ArrayList(SimRecord)) !void {
        try self.character_recorder.checkError();
        self.character_recorder.mutex.lock(io);
        defer self.character_recorder.mutex.unlock(io);
        try out.appendSlice(self.allocator, self.character_recorder.records.items);
        self.character_recorder.records.clearRetainingCapacity();
    }

    /// Destroy the Phase 7 objects (before the system, ~SimWorld of the .cpp: vehicles, characters, character virtuals,
    /// ragdolls)
    fn destroyPhase7(self: *ZoltWorld) void {
        VehicleSlot.destroyAll(self);
        CharacterSlot.destroyAll(self);
        CharacterVirtualSlot.destroyAll(self);
        RagdollSlot.destroyAll(self);
    }

    /// The BroadPhaseQuadTree of the system (the worlds use Jolt's default broad phase)
    fn broadPhase(self: *ZoltWorld) *zolt.BroadPhaseQuadTree {
        return @alignCast(@fieldParentPtr("base", self.system.broad_phase.?));
    }
};

/// The layout of the Zolt broad phase at one instant, for the D12 sweeps of the scenes with
/// SceneEntry.restore_broad_phase: the node pages (BroadPhaseQuadTree.initBroadPhase commits all of them), the state of
/// the node free list, the tracking of every body, the state of every tree (number of bodies, roots, dirty flag, the
/// batch of nodes to free) and the next tree to update. The failed updates of the sweeps change all of these, and none
/// is part of the saved state. Captured and restored between updates (no thread uses the broad phase); the sweeps
/// restore it together with the saved state, so that the bodies and the trees match again.
const BroadPhaseSnapshot = struct {
    const BroadPhaseQuadTree = zolt.BroadPhaseQuadTree;
    const QuadTree = zolt.QuadTree;
    const NodeAllocator = QuadTree.NodeAllocator;
    const FreeListState = @FieldType(NodeAllocator, "free_list");

    const Layer = struct {
        tree_state: QuadTree.TreeState,
        free_node_batch: NodeAllocator.Batch,
    };

    allocator: Allocator,
    /// The bytes of the allocated node pages (object storage: the node and its free list link)
    pages: []u8,
    num_objects_allocated: u32,
    allocation_tag: u32,
    first_free_object_and_tag: u64,
    first_free_object_in_new_page: u32,
    num_free_objects: @FieldType(FreeListState, "num_free_objects"),
    tracking: []QuadTree.Tracking,
    layers: []Layer,
    next_layer_to_update: u32,
    query_lock_idx: u32,

    fn pageBytes(nodes: *const NodeAllocator) usize {
        return @as(usize, nodes.page_size) * NodeAllocator.object_storage_size;
    }

    fn numPages(nodes: *const NodeAllocator) usize {
        return nodes.num_objects_allocated.load(.monotonic) / nodes.page_size;
    }

    fn capture(allocator: Allocator, bp: *const BroadPhaseQuadTree) Allocator.Error!BroadPhaseSnapshot {
        const nodes = &bp.node_allocator;
        const page_bytes = pageBytes(nodes);
        const num_pages = numPages(nodes);
        const pages = try allocator.alloc(u8, num_pages * page_bytes);
        errdefer allocator.free(pages);
        for (0..num_pages) |p| {
            const page: [*]const u8 = @ptrCast(nodes.pages.?[p]);
            @memcpy(pages[p * page_bytes ..][0..page_bytes], page[0..page_bytes]);
        }
        const tracking = try allocator.dupe(QuadTree.Tracking, bp.tracking);
        errdefer allocator.free(tracking);
        const layers = try allocator.alloc(Layer, bp.layers.len);
        for (layers, bp.layers) |*l, *tree|
            l.* = .{ .tree_state = tree.tree_state, .free_node_batch = tree.free_node_batch };
        const free_list = &nodes.free_list;
        return .{
            .allocator = allocator,
            .pages = pages,
            .num_objects_allocated = nodes.num_objects_allocated.load(.monotonic),
            .allocation_tag = free_list.allocation_tag.load(.monotonic),
            .first_free_object_and_tag = free_list.first_free_object_and_tag.load(.monotonic),
            .first_free_object_in_new_page = free_list.first_free_object_in_new_page.load(.monotonic),
            .num_free_objects = free_list.num_free_objects,
            .tracking = tracking,
            .layers = layers,
            .next_layer_to_update = bp.next_layer_to_update,
            .query_lock_idx = bp.query_lock_idx.load(.monotonic),
        };
    }

    /// Write the layout back (error.SceneError when the broad phase allocated node pages since the capture: it commits
    /// all of them in initBroadPhase)
    fn restore(self: *const BroadPhaseSnapshot, bp: *BroadPhaseQuadTree) !void {
        const nodes = &bp.node_allocator;
        if (nodes.num_objects_allocated.load(.monotonic) != self.num_objects_allocated or bp.tracking.len != self.tracking.len or bp.layers.len != self.layers.len) {
            std.debug.print("SimulationParity: the broad phase changed its node pages, bodies or layers since the snapshot\n", .{});
            return error.SceneError;
        }
        const page_bytes = pageBytes(nodes);
        for (0..numPages(nodes)) |p| {
            const page: [*]u8 = @ptrCast(nodes.pages.?[p]);
            @memcpy(page[0..page_bytes], self.pages[p * page_bytes ..][0..page_bytes]);
        }
        const free_list = &nodes.free_list;
        free_list.allocation_tag.store(self.allocation_tag, .monotonic);
        free_list.first_free_object_and_tag.store(self.first_free_object_and_tag, .monotonic);
        free_list.first_free_object_in_new_page.store(self.first_free_object_in_new_page, .monotonic);
        free_list.num_free_objects = self.num_free_objects;
        @memcpy(bp.tracking, self.tracking);
        for (bp.layers, self.layers) |*tree, *l| {
            tree.tree_state = l.tree_state;
            tree.free_node_batch = l.free_node_batch;
        }
        bp.next_layer_to_update = self.next_layer_to_update;
        bp.query_lock_idx.store(self.query_lock_idx, .monotonic);
    }

    fn deinit(self: *BroadPhaseSnapshot) void {
        self.allocator.free(self.pages);
        self.allocator.free(self.tracking);
        self.allocator.free(self.layers);
    }
};

// -------------------------------------------------------------------------------------------------------------------
// The C++ side

const CppWorld = struct {
    handle: *anyopaque,

    fn init(desc: *const SimSceneDesc, num_threads: u32) CppWorld {
        return .{ .handle = jolt.jolt_sim_create(desc, num_threads) };
    }

    fn deinit(self: CppWorld) void {
        jolt.jolt_sim_destroy(self.handle);
    }

    fn applyOp(self: CppWorld, op: *const SimOp) u32 {
        return jolt.jolt_sim_apply_op(self.handle, op);
    }

    fn update(self: CppWorld, delta_time: f32, collision_steps: u32) u32 {
        return jolt.jolt_sim_update(self.handle, delta_time, collision_steps);
    }

    fn saveState(self: CppWorld, allocator: Allocator, state: StateRecorderState, out: *std.ArrayList(u8)) !void {
        out.clearRetainingCapacity();
        while (true) {
            const size = jolt.jolt_sim_save_state(self.handle, state.toInt(), out.items.ptr, @intCast(out.capacity));
            if (size <= out.capacity) {
                out.items.len = size;
                return;
            }
            try out.ensureTotalCapacity(allocator, size);
        }
    }

    fn restoreState(self: CppWorld, bytes: []const u8) bool {
        return jolt.jolt_sim_restore_state(self.handle, bytes.ptr, @intCast(bytes.len)) != 0;
    }

    fn takeRecords(self: CppWorld, allocator: Allocator, out: *std.ArrayList(SimRecord)) !void {
        const n = jolt.jolt_sim_take_records(self.handle, out.items.ptr, 0);
        if (n == 0) return;
        try out.ensureUnusedCapacity(allocator, n);
        const unused = out.unusedCapacitySlice();
        const taken = jolt.jolt_sim_take_records(self.handle, unused.ptr, @intCast(unused.len));
        std.debug.assert(taken <= unused.len);
        out.items.len += taken;
    }

    fn hasMotionProperties(self: CppWorld, body_id: u32) bool {
        return jolt.jolt_sim_body_has_motion_properties(self.handle, body_id) != 0;
    }

    fn constraintStateSize(self: CppWorld, index: u32) u32 {
        var num: u32 = 0;
        return jolt.jolt_sim_constraint_state_size(self.handle, index, &num);
    }

    /// The subtype of constraint `index` of the system (null: none, or a subtype Zolt does not know)
    fn constraintSubType(self: CppWorld, index: u32) ?zolt.ConstraintSubType {
        const sub_type = jolt.jolt_sim_constraint_sub_type(self.handle, index);
        return if (sub_type < @typeInfo(zolt.ConstraintSubType).@"enum".fields.len) @enumFromInt(sub_type) else null;
    }

    /// The number of constraints of the system whose IsActive() is true
    fn numActiveConstraints(self: CppWorld) u32 {
        return jolt.jolt_sim_num_active_constraints(self.handle);
    }

    /// Phase 9, for the state decoder: the numbers of vertices, rod states and skin states of soft body `body_id` (null:
    /// not a soft body)
    fn softBodyInfo(self: CppWorld, body_id: u32) ?[3]u32 {
        var counts: [3]u32 = undefined;
        return if (jolt.jolt_sim_soft_body_info(self.handle, body_id, &counts) != 0) counts else null;
    }

    /// Phase 9: the SaveBinaryState bytes of the shared settings that SimMakeSoftBodySettings generates from soft body
    /// settings description `index` of `desc`
    fn softBodySettingsBytes(allocator: Allocator, desc: *const SimSceneDesc, index: u32, out: *std.ArrayList(u8)) !void {
        out.clearRetainingCapacity();
        while (true) {
            const size = jolt.jolt_sim_soft_body_settings_bytes(desc, index, out.items.ptr, @intCast(out.capacity));
            if (size <= out.capacity) {
                out.items.len = size;
                return;
            }
            try out.ensureTotalCapacity(allocator, size);
        }
    }

    /// Phase 7: the object state (WriteObjectState)
    fn objectState(self: CppWorld, allocator: Allocator, out: *std.ArrayList(u8)) !void {
        try getBytes(allocator, out, self.handle, &jolt.jolt_sim_object_state);
    }

    /// Phase 7: the SaveState bytes of every character and character virtual in handle order
    fn saveCharacterStates(self: CppWorld, allocator: Allocator, out: *std.ArrayList(u8)) !void {
        try getBytes(allocator, out, self.handle, &jolt.jolt_sim_save_character_states);
    }

    fn restoreCharacterStates(self: CppWorld, bytes: []const u8) bool {
        return jolt.jolt_sim_restore_character_states(self.handle, bytes.ptr, @intCast(bytes.len)) != 0;
    }

    /// Phase 7: the CharacterContactListener records in call order
    fn takeCharacterRecords(self: CppWorld, allocator: Allocator, out: *std.ArrayList(SimRecord)) !void {
        const n = jolt.jolt_sim_take_character_records(self.handle, out.items.ptr, 0);
        if (n == 0) return;
        try out.ensureUnusedCapacity(allocator, n);
        const unused = out.unusedCapacitySlice();
        const taken = jolt.jolt_sim_take_character_records(self.handle, unused.ptr, @intCast(unused.len));
        std.debug.assert(taken <= unused.len);
        out.items.len += taken;
    }

    /// Phase 7: the bytes of a part of vehicle description `index` (jolt_sim_vehicle_settings_part: SaveBinaryState of
    /// the part, or with `restore_from` a default object of its class restores those bytes and is saved again); false:
    /// no such part, or the restore failed
    fn vehicleSettingsPart(allocator: Allocator, desc: *const SimSceneDesc, index: u32, part: VehicleSettingsPart, item: u32, restore_from: ?[]const u8, out: *std.ArrayList(u8)) !bool {
        const bytes = restore_from orelse &.{};
        out.clearRetainingCapacity();
        while (true) {
            const size = jolt.jolt_sim_vehicle_settings_part(desc, index, @intFromEnum(part), item, @intFromBool(restore_from != null), bytes.ptr, @intCast(bytes.len), out.items.ptr, @intCast(out.capacity));
            if (size == no_bytes) return false;
            if (size <= out.capacity) {
                out.items.len = size;
                return true;
            }
            try out.ensureTotalCapacity(allocator, size);
        }
    }

    /// Phase 7: the SaveBinaryState bytes of the RagdollSettings of ragdoll description `index` (with the shapes and the
    /// group filter), or with `restore_from` of RagdollSettings::sRestoreFromBinaryState of those bytes; false: the
    /// restore failed
    fn ragdollSettingsBytes(self: CppWorld, allocator: Allocator, index: u32, restore_from: ?[]const u8, out: *std.ArrayList(u8)) !bool {
        const bytes = restore_from orelse &.{};
        out.clearRetainingCapacity();
        while (true) {
            const size = jolt.jolt_sim_ragdoll_settings_bytes(self.handle, index, @intFromBool(restore_from != null), bytes.ptr, @intCast(bytes.len), out.items.ptr, @intCast(out.capacity));
            if (size == no_bytes) return false;
            if (size <= out.capacity) {
                out.items.len = size;
                return true;
            }
            try out.ensureTotalCapacity(allocator, size);
        }
    }

    /// Phase 7: the tables of the RagdollSettings of ragdoll description `index` (jolt_sim_ragdoll_settings_info)
    fn ragdollSettingsInfo(self: CppWorld, allocator: Allocator, index: u32, out: *std.ArrayList(u32)) !void {
        out.clearRetainingCapacity();
        while (true) {
            const size = jolt.jolt_sim_ragdoll_settings_info(self.handle, index, out.items.ptr, @intCast(out.capacity));
            if (size <= out.capacity) {
                out.items.len = size;
                return;
            }
            try out.ensureTotalCapacity(allocator, size);
        }
    }

    /// The bytes of a C function that returns their size and copies them when they fit
    fn getBytes(allocator: Allocator, out: *std.ArrayList(u8), handle: *anyopaque, function: *const fn (*anyopaque, [*]u8, u32) callconv(.c) u32) !void {
        out.clearRetainingCapacity();
        while (true) {
            const size = function(handle, out.items.ptr, @intCast(out.capacity));
            if (size <= out.capacity) {
                out.items.len = size;
                return;
            }
            try out.ensureTotalCapacity(allocator, size);
        }
    }
};

// -------------------------------------------------------------------------------------------------------------------
// The state decoder: offsets of PhysicsSystem::SaveState decoded to (section, item, field)

const FieldKind = enum { u8, bool, u16, u32, f32, f64, bytes };

const Field = struct {
    offset: usize,
    size: usize,
    kind: FieldKind,
    section: []const u8,
    /// Index in the section (body, body pair, manifold, constraint) or null
    item: ?usize = null,
    /// The BodyID (or constraint index) of the item
    id: ?u32 = null,
    name: []const u8,
    component: ?u8 = null,
    /// Phase 9: the index of an element of the field (a vertex, rod or skin state of a soft body) and its member
    sub_item: ?usize = null,
    member: []const u8 = "",
};

/// Walks the bytes of a saved state like PhysicsSystem::RestoreState reads them. `cpp` (a CppWorld, or the C++ world of
/// another parity test) answers which bodies have motion properties (`hasMotionProperties(body_id) bool`) and the sizes
/// of the constraint states (`constraintStateSize(index) u32`); the decoded bytes are the C++ ones.
pub fn StateLayoutFor(comptime Cpp: type) type {
    return struct {
        const Self = @This();

        allocator: Allocator,
        bytes: []const u8,
        cpp: Cpp,
        fields: std.ArrayList(Field) = .empty,
        pos: usize = 0,
        truncated: bool = false,

        fn deinit(self: *Self) void {
            self.fields.deinit(self.allocator);
        }

        fn add(self: *Self, kind: FieldKind, size: usize, section: []const u8, item: ?usize, id: ?u32, name: []const u8, component: ?u8) !void {
            if (self.truncated or self.pos + size > self.bytes.len) {
                self.truncated = true;
                return;
            }
            try self.fields.append(self.allocator, .{ .offset = self.pos, .size = size, .kind = kind, .section = section, .item = item, .id = id, .name = name, .component = component });
            self.pos += size;
        }

        fn scalar(self: *Self, comptime T: type, section: []const u8, item: ?usize, id: ?u32, name: []const u8) !void {
            const kind: FieldKind = switch (T) {
                u8 => .u8,
                bool => .bool,
                u16 => .u16,
                u32 => .u32,
                f32 => .f32,
                f64 => .f64,
                else => @compileError("unsupported"),
            };
            try self.add(kind, @sizeOf(T), section, item, id, name, null);
        }

        fn vector(self: *Self, comptime T: type, n: u8, section: []const u8, item: ?usize, id: ?u32, name: []const u8) !void {
            for (0..n) |c|
                try self.add(if (T == f64) .f64 else .f32, @sizeOf(T), section, item, id, name, @intCast(c));
        }

        /// The floats of member `member` of element `sub_item` of `name` of body `item` (Phase 9: soft bodies)
        fn elementVector(self: *Self, n: u8, item: usize, id: u32, name: []const u8, sub_item: usize, member: []const u8) !void {
            for (0..n) |c| {
                try self.add(.f32, 4, "bodies", item, id, name, @intCast(c));
                if (!self.truncated) {
                    const field = &self.fields.items[self.fields.items.len - 1];
                    field.sub_item = sub_item;
                    field.member = member;
                }
            }
        }

        /// SoftBodyMotionProperties::SaveState after the MotionProperties part: the position and velocity of every
        /// vertex, the rotation and angular velocity of every rod, the previous position, position and normal of every
        /// skin state, the local bounds and the local predicted bounds. `counts`: the numbers of vertices, rod states
        /// and skin states (jolt_sim_soft_body_info).
        fn softBodyFields(self: *Self, item: usize, id: u32, counts: [3]u32) !void {
            for (0..counts[0]) |v| {
                try self.elementVector(3, item, id, "soft_body.vertices", v, "position");
                try self.elementVector(3, item, id, "soft_body.vertices", v, "velocity");
            }
            for (0..counts[1]) |r| {
                try self.elementVector(4, item, id, "soft_body.rod_states", r, "rotation");
                try self.elementVector(3, item, id, "soft_body.rod_states", r, "angular_velocity");
            }
            for (0..counts[2]) |s| {
                try self.elementVector(3, item, id, "soft_body.skin_state", s, "previous_position");
                try self.elementVector(3, item, id, "soft_body.skin_state", s, "position");
                try self.elementVector(3, item, id, "soft_body.skin_state", s, "normal");
            }
            for ([_][]const u8{ "soft_body.local_bounds.min", "soft_body.local_bounds.max", "soft_body.local_predicted_bounds.min", "soft_body.local_predicted_bounds.max" }) |name|
                try self.vector(f32, 3, "bodies", item, id, name);
        }

        fn read(self: *const Self, comptime T: type) ?T {
            if (self.pos + @sizeOf(T) > self.bytes.len) return null;
            return std.mem.bytesToValue(T, self.bytes[self.pos..][0..@sizeOf(T)]);
        }

        fn decode(self: *Self) !void {
            const state_byte = self.read(u8) orelse return;
            try self.scalar(u8, "header", null, null, "state");
            const state = StateRecorderState.fromInt(state_byte);

            if (state.global) {
                try self.scalar(f32, "global", null, null, "previous_step_delta_time");
                try self.vector(f32, 3, "global", null, null, "gravity");
            }

            if (state.bodies) {
                const num_bodies = self.read(u32) orelse return;
                try self.scalar(u32, "bodies", null, null, "num_bodies");
                for (0..num_bodies) |i| {
                    const id = self.read(u32) orelse return;
                    try self.scalar(u32, "bodies", i, id, "id");
                    try self.scalar(bool, "bodies", i, id, "is_active");
                    try self.vector(Real, 3, "bodies", i, id, "position");
                    try self.vector(f32, 4, "bodies", i, id, "rotation");
                    if (self.cpp.hasMotionProperties(id)) {
                        try self.vector(f32, 3, "bodies", i, id, "motion_properties.linear_velocity");
                        try self.vector(f32, 3, "bodies", i, id, "motion_properties.angular_velocity");
                        try self.vector(f32, 3, "bodies", i, id, "motion_properties.force");
                        try self.vector(f32, 3, "bodies", i, id, "motion_properties.torque");
                        if (Core.double_precision)
                            try self.vector(f64, 3, "bodies", i, id, "motion_properties.sleep_test_offset");
                        for ([_][]const u8{ "motion_properties.sleep_test_spheres[0]", "motion_properties.sleep_test_spheres[1]", "motion_properties.sleep_test_spheres[2]" }) |name| {
                            try self.vector(f32, 3, "bodies", i, id, name); // center
                            try self.add(.f32, 4, "bodies", i, id, name, 3); // radius
                        }
                        try self.scalar(f32, "bodies", i, id, "motion_properties.sleep_test_timer");
                        try self.scalar(bool, "bodies", i, id, "motion_properties.allow_sleeping");
                        // Phase 9: a soft body continues with its arrays and bounds (`softBodyInfo` of the C++ world)
                        if (comptime @hasDecl(Cpp, "softBodyInfo")) {
                            if (self.cpp.softBodyInfo(id)) |counts|
                                try self.softBodyFields(i, id, counts);
                        }
                    }
                }
            }

            if (state.contacts) {
                const num_body_pairs = self.read(u32) orelse return;
                try self.scalar(u32, "contacts", null, null, "num_body_pairs");
                var manifold_index: usize = 0;
                for (0..num_body_pairs) |i| {
                    const body_a = self.read(u32) orelse return;
                    try self.scalar(u32, "contacts.body_pairs", i, body_a, "key.body_a");
                    try self.scalar(u32, "contacts.body_pairs", i, body_a, "key.body_b");
                    try self.vector(f32, 3, "contacts.body_pairs", i, body_a, "delta_position");
                    try self.vector(f32, 3, "contacts.body_pairs", i, body_a, "delta_rotation");
                    const num_manifolds = self.read(u32) orelse return;
                    try self.scalar(u32, "contacts.body_pairs", i, body_a, "num_manifolds");
                    for (0..num_manifolds) |_| {
                        const m = manifold_index;
                        manifold_index += 1;
                        try self.scalar(u32, "contacts.manifolds", m, body_a, "key.body1");
                        try self.scalar(u32, "contacts.manifolds", m, body_a, "key.sub_shape1");
                        try self.scalar(u32, "contacts.manifolds", m, body_a, "key.body2");
                        try self.scalar(u32, "contacts.manifolds", m, body_a, "key.sub_shape2");
                        const num_points = self.read(u16) orelse return;
                        try self.scalar(u16, "contacts.manifolds", m, body_a, "num_contact_points");
                        try self.vector(f32, 3, "contacts.manifolds", m, body_a, "contact_normal");
                        try self.vector(f32, 2, "contacts.manifolds", m, body_a, "friction_lambda");
                        try self.scalar(f32, "contacts.manifolds", m, body_a, "angular_friction_lambda");
                        for (0..num_points) |_| {
                            try self.vector(f32, 3, "contacts.manifolds", m, body_a, "contact_point.position1");
                            try self.vector(f32, 3, "contacts.manifolds", m, body_a, "contact_point.position2");
                            try self.scalar(f32, "contacts.manifolds", m, body_a, "contact_point.non_penetration_lambda");
                        }
                    }
                }
                const num_ccd = self.read(u32) orelse return;
                try self.scalar(u32, "contacts", null, null, "num_ccd_manifolds");
                for (0..num_ccd) |i| {
                    for ([_][]const u8{ "key.body1", "key.sub_shape1", "key.body2", "key.sub_shape2" }) |name|
                        try self.scalar(u32, "contacts.ccd_manifolds", i, null, name);
                }
            }

            if (state.constraints) {
                const num_constraints = self.read(u32) orelse return;
                try self.scalar(u32, "constraints", null, null, "num_constraints");
                for (0..num_constraints) |i| {
                    const index = self.read(u32) orelse return;
                    try self.scalar(u32, "constraints", i, index, "constraint_index");
                    try self.scalar(bool, "constraints", i, index, "enabled");
                    const size = self.cpp.constraintStateSize(@intCast(i));
                    if (size > 1)
                        try self.constraintClassFields(i, index, size - 1);
                }
            }
        }

        /// The SaveState of a constraint class after `enabled`: the fields of its slot (`constraintStateFields`) when
        /// the C++ world tells the subtype and their size adds up, else one block of bytes
        fn constraintClassFields(self: *Self, i: usize, index: u32, size: u32) !void {
            const fields: ?[]const StateField = if (comptime @hasDecl(Cpp, "constraintSubType"))
                if (self.cpp.constraintSubType(@intCast(i))) |sub_type| constraintStateFields(sub_type) else null
            else
                null;
            if (fields) |list| {
                var total: usize = 0;
                for (list) |field| total += fieldSize(field.kind) * field.count;
                if (total == size) {
                    for (list) |field| {
                        for (0..field.count) |c|
                            try self.add(field.kind, fieldSize(field.kind), "constraints", i, index, field.name, if (field.count > 1) @intCast(c) else null);
                    }
                    return;
                }
            }
            try self.add(.bytes, size, "constraints", i, index, "state of the constraint class", null);
        }

        fn fieldSize(kind: FieldKind) usize {
            return switch (kind) {
                .u8, .bool => 1,
                .u16 => 2,
                .u32, .f32 => 4,
                .f64 => 8,
                .bytes => 1,
            };
        }

        fn find(self: *const Self, offset: usize) ?*const Field {
            for (self.fields.items) |*field| {
                if (offset >= field.offset and offset < field.offset + field.size)
                    return field;
            }
            return null;
        }
    };
}

const StateLayout = StateLayoutFor(CppWorld);

fn writeValue(w: *std.Io.Writer, field: *const Field, bytes: []const u8) !void {
    if (field.offset + field.size > bytes.len) {
        try w.writeAll("(missing)");
        return;
    }
    const b = bytes[field.offset..][0..field.size];
    switch (field.kind) {
        .u8 => try w.print("{d}", .{b[0]}),
        .bool => try w.print("{d}", .{b[0]}),
        .u16 => try w.print("{d}", .{std.mem.bytesToValue(u16, b[0..2])}),
        .u32 => try w.print("0x{x:0>8}", .{std.mem.bytesToValue(u32, b[0..4])}),
        .f32 => {
            const v = std.mem.bytesToValue(f32, b[0..4]);
            try w.print("0x{x:0>8} ({e})", .{ @as(u32, @bitCast(v)), v });
        },
        .f64 => {
            const v = std.mem.bytesToValue(f64, b[0..8]);
            try w.print("0x{x:0>16} ({e})", .{ @as(u64, @bitCast(v)), v });
        },
        .bytes => try w.print("{x}", .{b}),
    }
}

fn writeField(w: *std.Io.Writer, field: *const Field) !void {
    try w.print("{s}", .{field.section});
    if (field.item) |item| try w.print("[{d}]", .{item});
    if (field.id) |id| try w.print(" (id 0x{x:0>8})", .{id});
    try w.print(" {s}", .{field.name});
    if (field.sub_item) |s| try w.print("[{d}]", .{s});
    if (field.member.len > 0) try w.print(".{s}", .{field.member});
    if (field.component) |c| try w.print(".{c}", .{"xyzw"[c]});
}

/// Describe where the Zolt state differs from the Jolt state (null when they are equal). `cpp` is the C++ world (see
/// StateLayoutFor), the caller owns the returned text.
pub fn describeStateDifference(allocator: Allocator, zolt_bytes: []const u8, jolt_bytes: []const u8, cpp: anytype) !?[]u8 {
    const first = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes) orelse return null;

    var layout: StateLayoutFor(@TypeOf(cpp)) = .{ .allocator = allocator, .bytes = jolt_bytes, .cpp = cpp };
    defer layout.deinit();
    try layout.decode();

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const w = &out.writer;
    try w.print("state differs at byte {d} (zolt {d} bytes, jolt {d} bytes)", .{ first, zolt_bytes.len, jolt_bytes.len });
    if (layout.find(first)) |field| {
        try w.writeAll(": ");
        try writeField(w, field);
        try w.writeAll(" zolt ");
        try writeValue(w, field, zolt_bytes);
        try w.writeAll(" jolt ");
        try writeValue(w, field, jolt_bytes);
    } else {
        try w.writeAll(": past the decoded fields");
    }

    // The first differing fields
    var listed: u32 = 0;
    for (layout.fields.items) |*field| {
        if (field.offset < first) continue;
        const end = field.offset + field.size;
        const differs = end > zolt_bytes.len or !std.mem.eql(u8, zolt_bytes[field.offset..end], jolt_bytes[field.offset..end]);
        if (!differs) continue;
        if (listed == 0) try w.writeAll("\n  differing fields (jolt layout):");
        try w.writeAll("\n    ");
        try writeField(w, field);
        try w.writeAll(": zolt ");
        try writeValue(w, field, zolt_bytes);
        try w.writeAll(" jolt ");
        try writeValue(w, field, jolt_bytes);
        listed += 1;
        if (listed == 12) break;
    }
    return try out.toOwnedSlice();
}

// -------------------------------------------------------------------------------------------------------------------
// Phase 7: the object state decoder. The object state (SimScenes.ObjectField, written by the slots' `writeState` and
// WriteObjectState of SimulationReference.cpp) is a list of fields; a mismatch is reported with the field, the object
// handle, the item (contact, wheel, joint, ...) and the component, the SaveState fields with the members of
// CharacterBase / CharacterVirtual / CharacterContact::SaveState.

/// A field of an object state: its header and the offset of the header in the bytes
const ObjectFieldView = struct {
    header: ObjectFieldHeader,
    offset: usize,

    fn data(self: ObjectFieldView, bytes: []const u8) []const u8 {
        return bytes[self.offset + @sizeOf(ObjectFieldHeader) ..][0..self.header.size];
    }

    fn field(self: ObjectFieldView) ?ObjectField {
        return std.enums.fromInt(ObjectField, self.header.field);
    }
};

/// Iterates the fields of an object state (error.InvalidObjectState when a field does not fit in the bytes)
const ObjectFieldIterator = struct {
    bytes: []const u8,
    pos: usize = 0,

    fn next(self: *ObjectFieldIterator) !?ObjectFieldView {
        if (self.pos == self.bytes.len) return null;
        if (self.pos + @sizeOf(ObjectFieldHeader) > self.bytes.len) return error.InvalidObjectState;
        const header = std.mem.bytesToValue(ObjectFieldHeader, self.bytes[self.pos..][0..@sizeOf(ObjectFieldHeader)]);
        const end = self.pos + @sizeOf(ObjectFieldHeader) + header.size;
        if (end > self.bytes.len) return error.InvalidObjectState;
        const view: ObjectFieldView = .{ .header = header, .offset = self.pos };
        self.pos = end;
        return view;
    }
};

/// The members of CharacterBase::SaveState (the start of the `*_save_state` fields)
const character_base_save_state = [_]ObjectComponent{
    .{ .name = "ground_state", .kind = .u32 },
    .{ .name = "ground_body_id", .kind = .u32 },
    .{ .name = "ground_body_sub_shape_id", .kind = .u32 },
    .{ .name = "ground_position", .kind = .real, .count = 3 },
    .{ .name = "ground_normal", .kind = .f32, .count = 3 },
    .{ .name = "ground_velocity", .kind = .f32, .count = 3 },
};

/// The members of CharacterVirtual::SaveState after CharacterBase's (then `num_contacts` contacts)
const character_virtual_save_state = [_]ObjectComponent{
    .{ .name = "position", .kind = .real, .count = 3 },
    .{ .name = "rotation", .kind = .f32, .count = 4 },
    .{ .name = "linear_velocity", .kind = .f32, .count = 3 },
    .{ .name = "last_delta_time", .kind = .f32 },
    .{ .name = "max_hits_exceeded", .kind = .u8 },
    .{ .name = "num_contacts", .kind = .u32 },
};

/// The members of CharacterContact::SaveState (the contacts with mHadCollision of CharacterVirtual::SaveState)
const character_contact_save_state = [_]ObjectComponent{
    .{ .name = "contact.body_b", .kind = .u32 },
    .{ .name = "contact.character_id_b", .kind = .u32 },
    .{ .name = "contact.sub_shape_id_b", .kind = .u32 },
    .{ .name = "contact.position", .kind = .real, .count = 3 },
    .{ .name = "contact.linear_velocity", .kind = .f32, .count = 3 },
    .{ .name = "contact.contact_normal", .kind = .f32, .count = 3 },
    .{ .name = "contact.surface_normal", .kind = .f32, .count = 3 },
    .{ .name = "contact.distance", .kind = .f32 },
    .{ .name = "contact.fraction", .kind = .f32 },
    .{ .name = "contact.motion_type_b", .kind = .u8 },
    .{ .name = "contact.is_sensor_b", .kind = .u8 },
    .{ .name = "contact.had_collision", .kind = .u8 },
    .{ .name = "contact.was_discarded", .kind = .u8 },
    .{ .name = "contact.can_push_character", .kind = .u8 },
};

fn componentsSize(components: []const ObjectComponent) usize {
    var size: usize = 0;
    for (components) |c| size += SimScenes.objectComponentSize(c.kind) * c.count;
    return size;
}

/// The components of a field of an object state (the caller frees the slice; null: an unknown field number)
fn objectFieldComponents(allocator: Allocator, view: ObjectFieldView, bytes: []const u8) !?[]ObjectComponent {
    const field = view.field() orelse return null;
    var list: std.ArrayList(ObjectComponent) = .empty;
    errdefer list.deinit(allocator);
    switch (field) {
        .character_save_state => try list.appendSlice(allocator, &character_base_save_state),
        .cv_save_state => {
            try list.appendSlice(allocator, &character_base_save_state);
            try list.appendSlice(allocator, &character_virtual_save_state);
            const data = view.data(bytes);
            const offset = componentsSize(list.items) - 4;
            if (offset + 4 <= data.len) {
                // The contacts (bounded by the size of the field, the number may be garbage in a mismatch)
                const num_contacts = std.mem.readInt(u32, data[offset..][0..4], .little);
                const max_contacts = (data.len - offset - 4) / componentsSize(&character_contact_save_state);
                for (0..@min(num_contacts, max_contacts)) |_|
                    try list.appendSlice(allocator, &character_contact_save_state);
            }
        },
        else => try list.appendSlice(allocator, SimScenes.objectFieldLayout(field).?),
    }
    return try list.toOwnedSlice(allocator);
}

/// A value of an object state
const ObjectValue = struct {
    offset: usize,
    kind: SimScenes.ObjectComponentKind,
    header: ObjectFieldHeader,
    name: []const u8,
    /// The element of a vector / matrix component (null: a scalar)
    index: ?u8 = null,
};

/// The values of an object state (the layout of the C++ bytes). `complete` is false when a field is unknown or its
/// components do not add up to its size.
const ObjectStateLayout = struct {
    allocator: Allocator,
    values: std.ArrayList(ObjectValue) = .empty,
    complete: bool = true,

    fn deinit(self: *ObjectStateLayout) void {
        self.values.deinit(self.allocator);
    }

    fn decode(self: *ObjectStateLayout, bytes: []const u8) !void {
        var it: ObjectFieldIterator = .{ .bytes = bytes };
        while (true) {
            const view = (it.next() catch {
                self.complete = false;
                return;
            }) orelse return;
            const components = (try objectFieldComponents(self.allocator, view, bytes)) orelse {
                self.complete = false;
                continue;
            };
            defer self.allocator.free(components);
            var offset = view.offset + @sizeOf(ObjectFieldHeader);
            const end = offset + view.header.size;
            components: for (components) |c| {
                const size = SimScenes.objectComponentSize(c.kind);
                for (0..c.count) |k| {
                    if (offset + size > end) {
                        self.complete = false;
                        break :components;
                    }
                    try self.values.append(self.allocator, .{ .offset = offset, .kind = c.kind, .header = view.header, .name = c.name, .index = if (c.count > 1) @intCast(k) else null });
                    offset += size;
                }
            }
            if (offset != end) self.complete = false;
        }
    }

    /// The first value that is not finite (a float or Real)
    fn firstNotFinite(self: *const ObjectStateLayout, bytes: []const u8) ?*const ObjectValue {
        for (self.values.items) |*v| {
            const finite = switch (v.kind) {
                .f32 => std.math.isFinite(std.mem.bytesToValue(f32, bytes[v.offset..][0..4])),
                .real => std.math.isFinite(std.mem.bytesToValue(Real, bytes[v.offset..][0..@sizeOf(Real)])),
                else => true,
            };
            if (!finite) return v;
        }
        return null;
    }
};

fn writeObjectFieldName(w: *std.Io.Writer, header: ObjectFieldHeader) !void {
    if (std.enums.fromInt(ObjectField, header.field)) |field|
        try w.print("{t}", .{field})
    else
        try w.print("field {d}", .{header.field});
    try w.print(" (object handle {d}, item {d})", .{ header.handle, header.item });
}

fn writeObjectValueName(w: *std.Io.Writer, value: *const ObjectValue) !void {
    try writeObjectFieldName(w, value.header);
    try w.print(" {s}", .{value.name});
    if (value.index) |i| {
        if (i < 4) try w.print(".{c}", .{"xyzw"[i]}) else try w.print("[{d}]", .{i});
    }
}

fn writeObjectValue(w: *std.Io.Writer, value: *const ObjectValue, bytes: []const u8) !void {
    const size = SimScenes.objectComponentSize(value.kind);
    if (value.offset + size > bytes.len) {
        try w.writeAll("(missing)");
        return;
    }
    const b = bytes[value.offset..][0..size];
    switch (value.kind) {
        .u8 => try w.print("{d}", .{b[0]}),
        .u32 => try w.print("0x{x:0>8}", .{std.mem.bytesToValue(u32, b[0..4])}),
        .i32 => try w.print("{d}", .{std.mem.bytesToValue(i32, b[0..4])}),
        .u64 => try w.print("0x{x:0>16}", .{std.mem.bytesToValue(u64, b[0..8])}),
        .f32 => {
            const v = std.mem.bytesToValue(f32, b[0..4]);
            try w.print("0x{x:0>8} ({e})", .{ @as(u32, @bitCast(v)), v });
        },
        .real => {
            const v = std.mem.bytesToValue(Real, b[0..@sizeOf(Real)]);
            try w.print("0x{x} ({e})", .{ @as(std.meta.Int(.unsigned, @bitSizeOf(Real)), @bitCast(v)), v });
        },
    }
}

/// The names of the two object states that `describeObjectStateDifference` compares: the checked one and the reference,
/// whose layout decodes the bytes
pub const ObjectStateSides = struct {
    checked: []const u8 = "zolt",
    reference: []const u8 = "jolt",
};

/// Describe where the Zolt object state differs from the Jolt one (null when they are equal), the caller owns the text
pub fn describeObjectStateDifference(allocator: Allocator, zolt_bytes: []const u8, jolt_bytes: []const u8, sides: ObjectStateSides) !?[]u8 {
    const first = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes) orelse return null;

    var layout: ObjectStateLayout = .{ .allocator = allocator };
    defer layout.deinit();
    try layout.decode(jolt_bytes);

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const w = &out.writer;
    try w.print("object state differs at byte {d} ({s} {d} bytes, {s} {d} bytes)", .{ first, sides.checked, zolt_bytes.len, sides.reference, jolt_bytes.len });

    // Zolt writes more: the fields after the end of the jolt bytes
    if (first == jolt_bytes.len) {
        try w.print(": {s} writes more, starting with ", .{sides.checked});
        if (first + @sizeOf(ObjectFieldHeader) <= zolt_bytes.len)
            try writeObjectFieldName(w, std.mem.bytesToValue(ObjectFieldHeader, zolt_bytes[first..][0..@sizeOf(ObjectFieldHeader)]))
        else
            try w.writeAll("a truncated field");
        return try out.toOwnedSlice();
    }

    // A field header: the sides write other fields or objects here
    var it: ObjectFieldIterator = .{ .bytes = jolt_bytes };
    while (it.next() catch null) |view| {
        if (first >= view.offset + @sizeOf(ObjectFieldHeader)) continue;
        if (first < view.offset) break;
        try w.print(": the fields differ, {s} ", .{sides.reference});
        try writeObjectFieldName(w, view.header);
        try w.print(" of {d} bytes, {s} ", .{ view.header.size, sides.checked });
        if (view.offset + @sizeOf(ObjectFieldHeader) <= zolt_bytes.len) {
            const zolt_header = std.mem.bytesToValue(ObjectFieldHeader, zolt_bytes[view.offset..][0..@sizeOf(ObjectFieldHeader)]);
            try writeObjectFieldName(w, zolt_header);
            try w.print(" of {d} bytes", .{zolt_header.size});
        } else try w.writeAll("nothing");
        return try out.toOwnedSlice();
    }

    // The first differing values
    var listed: u32 = 0;
    for (layout.values.items) |*value| {
        const size = SimScenes.objectComponentSize(value.kind);
        if (value.offset + size <= first) continue;
        const differs = value.offset + size > zolt_bytes.len or !std.mem.eql(u8, zolt_bytes[value.offset..][0..size], jolt_bytes[value.offset..][0..size]);
        if (!differs) continue;
        switch (listed) {
            0 => try w.writeAll(": "),
            1 => try w.print("\n  further differing values ({s} layout):\n    ", .{sides.reference}),
            else => try w.writeAll("\n    "),
        }
        try writeObjectValueName(w, value);
        try w.print(" {s} ", .{sides.checked});
        try writeObjectValue(w, value, zolt_bytes);
        try w.print(" {s} ", .{sides.reference});
        try writeObjectValue(w, value, jolt_bytes);
        listed += 1;
        if (listed == 12) break;
    }
    if (listed == 0) try w.writeAll(": past the decoded values");
    return try out.toOwnedSlice();
}

/// What the object states of a run show (Expect.ground_states, Expect.wheel_contacts, Expect.vehicle_*)
const ObjectStats = struct {
    /// The ground states (bit `1 << EGroundState`) of the characters and character virtuals after some update
    ground_states: u32 = 0,
    /// Some wheel had a contact after some update
    wheel_contacts: bool = false,
    /// The transmission gears of the vehicles (SimScenes.gearBit) after some update
    vehicle_gears: u32 = 0,
    /// Forward shifts up / down between two consecutive object states of a vehicle
    vehicle_up_shifts: u32 = 0,
    vehicle_down_shifts: u32 = 0,
    /// The gear of each vehicle handle in the previous object state (null: none yet)
    previous_gears: [max_tracked_vehicles]?i32 = @splat(null),

    const max_tracked_vehicles = 16;
};

/// Add what an object state shows to `stats`
fn scanObjectState(bytes: []const u8, stats: *ObjectStats) !void {
    var it: ObjectFieldIterator = .{ .bytes = bytes };
    while (try it.next()) |view| {
        const data = view.data(bytes);
        switch (view.field() orelse continue) {
            .character_save_state, .cv_save_state => if (data.len >= 4) {
                const ground_state = std.mem.readInt(u32, data[0..4], .little);
                if (ground_state < 32) stats.ground_states |= @as(u32, 1) << @intCast(ground_state);
            },
            .vehicle_wheel => if (data.len >= 16 and std.mem.readInt(u32, data[12..16], .little) != 0) {
                stats.wheel_contacts = true;
            },
            .vehicle_transmission => if (data.len >= 4) {
                const gear = std.mem.readInt(i32, data[0..4], .little);
                if (gear >= -8 and gear < 24) stats.vehicle_gears |= SimScenes.gearBit(gear);
                const handle = view.header.handle;
                if (handle < ObjectStats.max_tracked_vehicles) {
                    if (stats.previous_gears[handle]) |previous| {
                        if (previous >= 1 and gear == previous + 1) stats.vehicle_up_shifts += 1;
                        if (previous >= 2 and gear == previous - 1) stats.vehicle_down_shifts += 1;
                    }
                    stats.previous_gears[handle] = gear;
                }
            },
            else => {},
        }
    }
}

// -------------------------------------------------------------------------------------------------------------------
// The driver

const Config = struct {
    zolt: JobKind,
    cpp_threads: u32,
    /// Run the D12 sweeps (every temp allocation and every createJob fails once) in this configuration
    sweeps: bool = false,
    /// Group D3: record the states for the run that draws, or draw and compare (see SimDraw)
    draw: ?*SimDraw = null,
};

/// Zolt with JobSystemSingleThreaded and thread pools with 0, 1, 3 and 7 workers, C++ with 0, 3 and 7 workers
const configs = [_]Config{
    .{ .zolt = .single_threaded, .cpp_threads = 0, .sweeps = true },
    .{ .zolt = .{ .thread_pool = 0 }, .cpp_threads = 3 },
    .{ .zolt = .{ .thread_pool = 1 }, .cpp_threads = 0 },
    .{ .zolt = .{ .thread_pool = 3 }, .cpp_threads = 7, .sweeps = true },
    .{ .zolt = .{ .thread_pool = 7 }, .cpp_threads = 3 },
};

const all_states = [_]StateRecorderState{
    .{ .global = true },
    .{ .bodies = true },
    .{ .contacts = true },
    .{ .constraints = true },
    .{ .bodies = true, .contacts = true },
    .all,
};

/// The EStateRecorderState of a `snapshot` operation (i[0], 0: All)
fn snapshotState(op: *const SimOp) StateRecorderState {
    return if (op.i[0] == 0) .all else .fromInt(@intCast(op.i[0]));
}

/// One scene in one configuration, both sides in lockstep
const Run = struct {
    allocator: Allocator,
    entry: *const SceneEntry,
    config: Config,
    builder: *const SceneBuilder,
    desc: *const SimSceneDesc,
    zw: *ZoltWorld,
    cw: CppWorld,
    update: u32 = 0,
    zolt_state: std.ArrayList(u8) = .empty,
    jolt_state: std.ArrayList(u8) = .empty,
    zolt_records: std.ArrayList(SimRecord) = .empty,
    jolt_records: std.ArrayList(SimRecord) = .empty,
    snapshots: [4][2]std.ArrayList(u8) = @splat(.{ .empty, .empty }),
    /// Phase 7: the object states, the character listener records and the character states of the snapshots
    zolt_objects: std.ArrayList(u8) = .empty,
    jolt_objects: std.ArrayList(u8) = .empty,
    zolt_character_records: std.ArrayList(SimRecord) = .empty,
    jolt_character_records: std.ArrayList(SimRecord) = .empty,
    character_snapshots: [4][2]std.ArrayList(u8) = @splat(.{ .empty, .empty }),
    /// Statistics for the sanity checks of the test
    stats: RunStats = .{},
    /// R8: the counting assert handler of a release_only scene (null for the other scenes), and the Zolt asserts of the
    /// updates that only Zolt runs (the D12 sweeps), which are not compared with the C++ side
    asserts: ?*zolt.IssueReporting.CountAsserts = null,
    uncompared_asserts: u32 = 0,
    /// Phase 9: the unsaved skin flags of the soft bodies at the snapshot of a D12 sweep (restoreZolt puts them back)
    skin_flags: SkinFlags = .{},

    fn deinit(self: *Run) void {
        self.skin_flags.deinit(self.allocator);
        self.zolt_state.deinit(self.allocator);
        self.jolt_state.deinit(self.allocator);
        self.zolt_records.deinit(self.allocator);
        self.jolt_records.deinit(self.allocator);
        for (&self.snapshots) |*s| {
            s[0].deinit(self.allocator);
            s[1].deinit(self.allocator);
        }
        self.zolt_objects.deinit(self.allocator);
        self.jolt_objects.deinit(self.allocator);
        self.zolt_character_records.deinit(self.allocator);
        self.jolt_character_records.deinit(self.allocator);
        for (&self.character_snapshots) |*s| {
            s[0].deinit(self.allocator);
            s[1].deinit(self.allocator);
        }
    }

    /// Phase 7: compare the object states of both sides (`what` describes the moment)
    fn compareObjectStates(self: *Run, what: []const u8) !void {
        try self.zw.writeObjectState(&self.zolt_objects);
        try self.cw.objectState(self.allocator, &self.jolt_objects);
        if (try describeObjectStateDifference(self.allocator, self.zolt_objects.items, self.jolt_objects.items, .{})) |message| {
            defer self.allocator.free(message);
            return self.fail("{s}: {s}", .{ what, message });
        }
        try scanObjectState(self.jolt_objects.items, &self.stats.objects);
    }

    /// Phase 7: compare the CharacterContactListener records since the last comparison in call order (the user thread
    /// calls them in Jolt's deterministic order: the character operations run between the updates)
    fn compareCharacterRecords(self: *Run) !void {
        self.zolt_character_records.clearRetainingCapacity();
        self.jolt_character_records.clearRetainingCapacity();
        try self.zw.takeCharacterRecords(&self.zolt_character_records);
        try self.cw.takeCharacterRecords(self.allocator, &self.jolt_character_records);
        for (self.jolt_character_records.items) |r|
            self.stats.records[r.kind] += 1;
        const n = @min(self.zolt_character_records.items.len, self.jolt_character_records.items.len);
        for (self.zolt_character_records.items[0..n], self.jolt_character_records.items[0..n], 0..) |z, j, i| {
            if (!std.meta.eql(z, j))
                return self.fail("character listener record {d} of {d} (call order) differs:\n  zolt {f}\n  jolt {f}", .{ i, self.jolt_character_records.items.len, RecordText{ .record = z }, RecordText{ .record = j } });
        }
        if (self.zolt_character_records.items.len != self.jolt_character_records.items.len)
            return self.fail("{d} character listener records in zolt, {d} in jolt", .{ self.zolt_character_records.items.len, self.jolt_character_records.items.len });
    }

    fn fail(self: *const Run, comptime fmt: []const u8, args: anytype) error{ParityMismatch} {
        std.debug.print("SimulationParity: scene {s}, zolt {any}, jolt {d} threads, before / after update {d}: ", .{ self.entry.name, self.config.zolt, self.config.cpp_threads, self.update });
        std.debug.print(fmt ++ "\n", args);
        return error.ParityMismatch;
    }

    /// Compare the saved states of both sides (`what` describes the moment)
    fn compareStates(self: *Run, state: StateRecorderState, what: []const u8) !void {
        try self.zw.saveState(state, &self.zolt_state);
        try self.cw.saveState(self.allocator, state, &self.jolt_state);
        if (try describeStateDifference(self.allocator, self.zolt_state.items, self.jolt_state.items, self.cw)) |message| {
            defer self.allocator.free(message);
            return self.fail("{s}, state {x}: {s}", .{ what, state.toInt(), message });
        }
    }

    /// The filtered states (StateFilter / SimStateFilter): both sides save the same bytes, then each side restores the
    /// filtered bytes of the other side (the contacts through ShouldRestoreContact) and the full states must still match
    fn compareFilteredStates(self: *Run) !void {
        const allocator = self.allocator;
        var filter: StateFilter = .{};
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        try self.zw.system.saveState(recorder.recorder(), .{ .filter = &filter.base });
        if (recorder.isFailed()) return error.OutOfMemory;
        var jolt_bytes: std.ArrayList(u8) = .empty;
        defer jolt_bytes.deinit(allocator);
        const size = jolt.jolt_sim_save_state_filtered(self.cw.handle, StateRecorderState.all.toInt(), jolt_bytes.items.ptr, 0);
        try jolt_bytes.resize(allocator, size);
        _ = jolt.jolt_sim_save_state_filtered(self.cw.handle, StateRecorderState.all.toInt(), jolt_bytes.items.ptr, size);
        if (std.mem.indexOfDiff(u8, recorder.getData(), jolt_bytes.items)) |first|
            return self.fail("the filtered states differ at byte {d} (zolt {d} bytes, jolt {d} bytes)", .{ first, recorder.getData().len, jolt_bytes.items.len });

        // Cross restore of the filtered states
        var restore = StateRecorderImpl.init(allocator);
        defer restore.deinit();
        restore.writeBytes(jolt_bytes.items);
        if (restore.isFailed()) return error.OutOfMemory;
        restore.rewind();
        if (!try self.zw.system.restoreState(restore.recorder(), .{ .filter = &filter.base }))
            return self.fail("zolt could not restore the filtered jolt state", .{});
        if (jolt.jolt_sim_restore_state_filtered(self.cw.handle, recorder.getData().ptr, @intCast(recorder.getData().len)) == 0)
            return self.fail("jolt could not restore the filtered zolt state", .{});
        try self.compareStates(.all, "after the filtered cross restore");
    }

    /// Compare the listener callbacks since the last comparison as sorted records
    fn compareRecords(self: *Run) !void {
        self.zolt_records.clearRetainingCapacity();
        self.jolt_records.clearRetainingCapacity();
        try self.zw.takeRecords(&self.zolt_records);
        try self.cw.takeRecords(self.allocator, &self.jolt_records);
        try self.compareTakenRecords();
    }

    fn compareTakenRecords(self: *Run) !void {
        std.sort.pdq(SimRecord, self.zolt_records.items, {}, SimRecord.lessThan);
        std.sort.pdq(SimRecord, self.jolt_records.items, {}, SimRecord.lessThan);
        for (self.jolt_records.items) |r|
            self.stats.records[r.kind] += 1;
        const n = @min(self.zolt_records.items.len, self.jolt_records.items.len);
        for (self.zolt_records.items[0..n], self.jolt_records.items[0..n], 0..) |z, j, i| {
            if (!std.meta.eql(z, j))
                return self.fail("listener record {d} of {d} differs:\n  zolt {any}\n  jolt {any}", .{ i, self.jolt_records.items.len, z, j });
        }
        if (self.zolt_records.items.len != self.jolt_records.items.len)
            return self.fail("{d} listener records in zolt, {d} in jolt", .{ self.zolt_records.items.len, self.jolt_records.items.len });
        try checkSoftBodyRecords(self.allocator, self.entry, self.update, self.jolt_records.items);
        self.stats.max_manifolds_per_body_pair = @max(self.stats.max_manifolds_per_body_pair, try maxManifoldsPerBodyPair(self.allocator, self.jolt_records.items));
    }

    fn applyOps(self: *Run, op_index: *usize, delta_time: *f32, collision_steps: *u32) !void {
        const ops = self.builder.ops.items;
        while (op_index.* < ops.len and ops[op_index.*].update == self.update) : (op_index.* += 1) {
            const op = &ops[op_index.*];
            switch (@as(OpKind, @enumFromInt(op.kind))) {
                .set_update => {
                    delta_time.* = op.f[0];
                    collision_steps.* = op.count;
                },
                .snapshot => {
                    const slot = &self.snapshots[op.index];
                    const state = snapshotState(op);
                    try self.zw.saveState(state, &slot[0]);
                    try self.cw.saveState(self.allocator, state, &slot[1]);
                    // Phase 7: the characters are not part of the PhysicsSystem state
                    const character_slot = &self.character_snapshots[op.index];
                    try self.zw.saveCharacterStates(&character_slot[0]);
                    try self.cw.saveCharacterStates(self.allocator, &character_slot[1]);
                },
                .restore_cross => {
                    // Zolt restores the C++ state, C++ the Zolt state
                    const slot = &self.snapshots[op.index];
                    try self.zw.saveState(.all, &self.zolt_state);
                    if (!std.mem.eql(u8, self.zolt_state.items, slot[1].items))
                        self.stats.restores_changing_the_state += 1;
                    if (!try self.zw.restoreState(slot[1].items))
                        return self.fail("zolt could not restore the jolt state of slot {d}", .{op.index});
                    if (!self.cw.restoreState(slot[0].items))
                        return self.fail("jolt could not restore the zolt state of slot {d}", .{op.index});
                    const character_slot = &self.character_snapshots[op.index];
                    if (!try self.zw.restoreCharacterStates(character_slot[1].items))
                        return self.fail("zolt could not restore the jolt character states of slot {d} (a read failed or bytes are left)", .{op.index});
                    if (!self.cw.restoreCharacterStates(character_slot[0].items))
                        return self.fail("jolt could not restore the zolt character states of slot {d} (a read failed or bytes are left)", .{op.index});
                    try self.compareStates(.all, "after restore_cross");
                    try self.compareObjectStates("after restore_cross");
                },
                else => |kind| {
                    const zolt_result = try self.zw.applyOp(op);
                    const jolt_result = self.cw.applyOp(op);
                    if (zolt_result != jolt_result) {
                        if (ConstraintOpRange.of(op.kind) != null)
                            return self.fail("operation {d} ({t} on constraint handle {d}) returned {d} in zolt and {d} in jolt", .{ op_index.*, kind, op.index, zolt_result, jolt_result });
                        if (Phase7OpRange.of(op.kind) != null)
                            return self.fail("operation {d} ({t} on object handle / description {d}) returned {d} in zolt and {d} in jolt", .{ op_index.*, kind, op.index, zolt_result, jolt_result });
                        return self.fail("operation {d} ({t} on handle {d}) returned {d} in zolt and {d} in jolt", .{ op_index.*, kind, op.body, zolt_result, jolt_result });
                    }
                    if (isPhase7Expectation(kind)) {
                        if (jolt_result != op.count)
                            return self.fail("operation {d}: the scene expects {t} of object handle {d} to be {d}, it is {d} on both sides (the scene does not reach what it is written for)", .{ op_index.*, kind, op.index, op.count, jolt_result });
                        self.stats.num_expectations += 1;
                    }
                    if (kind == .expect_body_state) {
                        if (jolt_result != op.index)
                            return self.fail("operation {d}: the scene expects handle {d} in BodyState {d}, it is in BodyState {d} on both sides (the scene does not reach what it is written for)", .{ op_index.*, op.body, op.index, jolt_result });
                        self.stats.num_expectations += 1;
                    }
                    if (kind == .expect_constraint_state) {
                        if (jolt_result != op.i[0])
                            return self.fail("operation {d}: the scene expects constraint handle {d} in ConstraintState {d}, it is in ConstraintState {d} on both sides (the scene does not reach what it is written for)", .{ op_index.*, op.index, op.i[0], jolt_result });
                        self.stats.num_expectations += 1;
                    }
                },
            }
        }
    }

    /// One update on both sides, then the comparisons
    fn step(self: *Run, delta_time: f32, collision_steps: u32) !void {
        self.countActiveConstraints();
        const sweep = self.config.sweeps and self.entry.sweep_stride != 0 and self.update % self.entry.sweep_stride == 0;
        if (sweep)
            return self.sweepStep(delta_time, collision_steps);

        const ccd_jobs_before = self.zw.counting_jobs.num_find_ccd_contacts.load(.monotonic);
        const activations_before = self.zw.countRecords(.body_activated);
        const deactivations_before = self.zw.countRecords(.body_deactivated);
        const zolt_errors = try self.zw.update(delta_time, collision_steps, self.zw.temp.tempAllocator(), self.zw.jobSystem());
        self.countCCDJobs(ccd_jobs_before);
        self.countUpdateActivations(activations_before);
        self.countUpdateDeactivations(deactivations_before);
        if (!self.zw.temp.isEmpty()) return self.fail("the temp allocator is not empty after the update", .{});
        const jolt_errors = self.cw.update(delta_time, collision_steps);
        if (zolt_errors != jolt_errors)
            return self.fail("update errors zolt 0x{x} jolt 0x{x}", .{ zolt_errors, jolt_errors });
        try self.compareStates(.all, "after the update");
        if (self.config.draw) |d| try d.afterUpdate(self);
        try self.countCCDManifolds();
        try self.compareRecords();
        try self.compareObjectStates("after the update");
        try self.compareCharacterRecords();
        self.updateStats();
    }

    /// The number of active constraints before the update (Zolt side: IsActive as DetermineActiveConstraints sees it)
    fn countActiveConstraints(self: *Run) void {
        var num_active: u32 = 0;
        for (self.zw.system.constraint_manager.constraints.items) |*c| {
            if (c.get().?.isActive()) num_active += 1;
        }
        self.stats.max_active_constraints = @max(self.stats.max_active_constraints, num_active);
    }

    /// The number of bodies activated from inside the update that just ran (Zolt side, `before`: the activation
    /// records of the operations before it)
    fn countUpdateActivations(self: *Run, before: u32) void {
        const num_activations = self.zw.countRecords(.body_activated) - before;
        self.stats.max_update_activations = @max(self.stats.max_update_activations, num_activations);
    }

    /// Phase 9: the number of bodies deactivated from inside the update that just ran (Zolt side, `before`: the
    /// deactivation records of the operations before it)
    fn countUpdateDeactivations(self: *Run, before: u32) void {
        const num_deactivations = self.zw.countRecords(.body_deactivated) - before;
        self.stats.max_update_deactivations = @max(self.stats.max_update_deactivations, num_deactivations);
    }

    /// The number of FindCCDContacts jobs of the update that just ran (Zolt side)
    fn countCCDJobs(self: *Run, before: u32) void {
        const num_jobs = self.zw.counting_jobs.num_find_ccd_contacts.load(.monotonic) - before;
        self.stats.max_find_ccd_contacts_jobs = @max(self.stats.max_find_ccd_contacts_jobs, num_jobs);
    }

    /// Count the update when the contact cache of the C++ state after it holds CCD manifolds (a linear cast body hit
    /// something: ContactConstraintManager::OnCCDContactAdded, the worlds have a contact listener)
    fn countCCDManifolds(self: *Run) !void {
        var layout: StateLayout = .{ .allocator = self.allocator, .bytes = self.jolt_state.items, .cpp = self.cw };
        defer layout.deinit();
        try layout.decode();
        for (layout.fields.items) |*field| {
            if (std.mem.eql(u8, field.name, "num_ccd_manifolds")) {
                if (std.mem.bytesToValue(u32, self.jolt_state.items[field.offset..][0..4]) > 0)
                    self.stats.updates_with_ccd_manifolds += 1;
                return;
            }
        }
    }

    fn updateStats(self: *Run) void {
        const system = &self.zw.system;
        const num_active = system.getNumActiveBodies(.rigid_body);
        self.stats.max_active_bodies = @max(self.stats.max_active_bodies, num_active);
        const body_stats = system.getBodyStats();
        if (body_stats.num_bodies_dynamic + body_stats.num_bodies_kinematic > num_active or body_stats.num_soft_bodies > body_stats.num_active_soft_bodies) // Phase 9: soft bodies sleep too
            self.stats.updates_with_sleeping_bodies += 1;
        if (system.body_manager.getNumActiveCCDBodies() > 0)
            self.stats.updates_with_ccd_bodies += 1;
    }

    /// The update with the D12 sweeps: every temp allocation and every createJob fails once, the system recovers by
    /// restoring the state saved before the update, the final update gives the C++ result
    fn sweepStep(self: *Run, delta_time: f32, collision_steps: u32) !void {
        const allocator = self.allocator;
        const zw = self.zw;

        // The records of the operations before the update belong to this update
        var zolt_records: std.ArrayList(SimRecord) = .empty;
        defer zolt_records.deinit(allocator);
        try zw.takeRecords(&zolt_records);

        var snapshot: std.ArrayList(u8) = .empty;
        defer snapshot.deinit(allocator);
        try zw.saveState(.all, &snapshot);

        // SceneEntry.restore_broad_phase: the layout of the broad phase at the snapshot (the C++ side updates this tree)
        var broad_phase_snapshot: ?BroadPhaseSnapshot = if (self.entry.restore_broad_phase) try BroadPhaseSnapshot.capture(allocator, zw.broadPhase()) else null;
        defer if (broad_phase_snapshot) |*s| s.deinit();

        // The bodies whose contact cache is invalid (the operations before the update flagged them): the flags are not
        // part of the saved state (in Jolt neither), every update that starts from the snapshot gets them again
        const pending_invalid = try allocator.dupe(BodyID, zw.system.body_manager.bodies_cache_invalid.items);
        defer allocator.free(pending_invalid);

        // Phase 9: the same for mSkinStatePreviousPositionValid of the soft bodies (restoreZolt puts it back)
        try self.skin_flags.capture(allocator, zw);
        defer self.skin_flags.clear();

        // C++ first: the reference bytes after the update
        const jolt_errors = self.cw.update(delta_time, collision_steps);
        var reference: std.ArrayList(u8) = .empty;
        defer reference.deinit(allocator);
        try self.cw.saveState(allocator, .all, &reference);

        // The update counting the temp allocations is the one that is compared (records and asserts included)
        var counting: FailingTempAllocator = .init(zw.temp.tempAllocator(), .{});
        const ccd_jobs_before = zw.counting_jobs.num_find_ccd_contacts.load(.monotonic);
        const activations_before = zw.countRecords(.body_activated);
        const deactivations_before = zw.countRecords(.body_deactivated);
        const zolt_errors = try zw.update(delta_time, collision_steps, counting.tempAllocator(), zw.jobSystem());
        const compared_asserts = if (self.asserts) |a| a.get() else 0;
        self.countCCDJobs(ccd_jobs_before);
        self.countUpdateActivations(activations_before);
        self.countUpdateDeactivations(deactivations_before);
        if (zolt_errors != jolt_errors)
            return self.fail("update errors zolt 0x{x} jolt 0x{x}", .{ zolt_errors, jolt_errors });
        try self.compareStates(.all, "after the update");
        try self.countCCDManifolds();
        try zw.takeRecords(&zolt_records);
        const step_calls = try allocator.alloc(u32, zw.step_listeners.len);
        defer allocator.free(step_calls);
        for (zw.step_listeners, step_calls) |*l, *c| c.* = l.num_calls;

        // Every temp allocation fails once
        const num_allocations = counting.num_allocations;
        for (0..num_allocations) |fail_index| {
            try self.restoreZoltBeforeUpdate(snapshot.items, pending_invalid);
            var failing: FailingTempAllocator = .init(zw.temp.tempAllocator(), .{ .fail_index = @intCast(fail_index) });
            if (zw.update(delta_time, collision_steps, failing.tempAllocator(), zw.jobSystem())) |_| {
                return self.fail("temp allocation {d} of {d} failed but the update succeeded", .{ fail_index, num_allocations });
            } else |err| if (err != error.OutOfMemory) return err;
            if (!failing.has_induced_failure) return self.fail("temp allocation {d} did not fail", .{fail_index});
            if (!zw.temp.isEmpty()) return self.fail("temp allocation {d} failed: the temp allocator is not empty", .{fail_index});
            if (fail_index < 2) // The steps and the body pairs are allocated before any job runs
                try self.expectZoltState(snapshot.items, "temp allocation failed while building the jobs");
            try self.recoverZolt(snapshot.items, reference.items, delta_time, collision_steps, "after temp allocation failure");
            self.stats.num_temp_failures += 1;
        }

        // Every createJob fails once: count the jobs first (a normal update)
        try self.restoreZoltBeforeUpdate(snapshot.items, pending_invalid);
        var counting_jobs: FailingJobSystem = .init(zw.jobSystem(), .{});
        _ = try zw.update(delta_time, collision_steps, zw.temp.tempAllocator(), counting_jobs.jobSystem());
        try self.expectZoltState(reference.items, "after the update counting the jobs");
        const num_jobs = counting_jobs.num_created.load(.monotonic);
        for (0..num_jobs + 2) |fail_index| {
            try self.restoreZoltBeforeUpdate(snapshot.items, pending_invalid);
            var failing: FailingJobSystem = .init(zw.jobSystem(), .{ .fail_index = @intCast(fail_index) });
            if (zw.update(delta_time, collision_steps, zw.temp.tempAllocator(), failing.jobSystem())) |_| {
                // A job that could not be spawned from inside a job changes no result
                try self.expectZoltState(reference.items, "a job could not be spawned");
            } else |err| {
                if (err != error.OutOfMemory) return err;
                // A failure while building the jobs changes nothing
                try self.expectZoltState(snapshot.items, "createJob failed while building the jobs");
                try self.recoverZolt(snapshot.items, reference.items, delta_time, collision_steps, "after a createJob failure");
            }
            if (!zw.temp.isEmpty()) return self.fail("createJob {d} failed: the temp allocator is not empty", .{fail_index});
            if (failing.has_induced_failure.load(.monotonic)) self.stats.num_job_failures += 1;
        }

        // SceneEntry.restore_broad_phase: the failed updates changed the layout of the broad phase trees; one more update
        // from the snapshot and the broad phase of that instant leaves the trees of the C++ update, so that the queries of
        // the Phase 7 objects after this update find their hits in the C++ order
        if (broad_phase_snapshot) |*s| {
            try self.restoreZoltBeforeUpdate(snapshot.items, pending_invalid);
            try s.restore(zw.broadPhase());
            _ = try zw.update(delta_time, collision_steps, zw.temp.tempAllocator(), zw.jobSystem());
            if (!zw.temp.isEmpty()) return self.fail("the update from the broad phase snapshot: the temp allocator is not empty", .{});
            try self.expectZoltState(reference.items, "after the update from the broad phase snapshot");
        }

        // The Zolt side is at the reference state again: drop the records of the sweeps, restore the listener counters
        zw.discardRecords();
        if (self.asserts) |a| self.uncompared_asserts += a.get() - compared_asserts;
        for (zw.step_listeners, step_calls) |*l, c| l.num_calls = c;
        try self.expectZoltState(reference.items, "after the sweeps");

        self.zolt_records.clearRetainingCapacity();
        self.jolt_records.clearRetainingCapacity();
        try self.zolt_records.appendSlice(allocator, zolt_records.items);
        try self.cw.takeRecords(allocator, &self.jolt_records);
        try self.compareTakenRecords();
        // Phase 7: the objects after the last update of the sweeps (a full update from the snapshot, like the C++ one)
        try self.compareObjectStates("after the sweeps");
        try self.compareCharacterRecords();
        self.updateStats();
    }

    /// Restore the snapshot of a sweep, with the skin flags of the soft bodies at the snapshot (Phase 9: Jolt's SaveState
    /// does not hold them, see SkinFlags)
    fn restoreZolt(self: *Run, snapshot: []const u8) !void {
        if (!try self.zw.restoreState(snapshot))
            return self.fail("zolt could not restore its snapshot", .{});
        self.skin_flags.restore(self.zw);
    }

    /// Restore the snapshot and flag the contact caches that were invalid when it was taken (the first update after the
    /// snapshot reset the flags). The recovery after a failed update (recoverZolt) does not: PhysicsSystem.update puts
    /// the flags back itself when it runs out of memory (BodyManager.restoreContactCacheInvalidation).
    fn restoreZoltBeforeUpdate(self: *Run, snapshot: []const u8, pending_invalid: []const BodyID) !void {
        try self.restoreZolt(snapshot);
        const body_manager = &self.zw.system.body_manager;
        for (pending_invalid) |id| {
            if (body_manager.tryGetBodyMut(id)) |body|
                try body_manager.invalidateContactCacheForBody(body);
        }
    }

    fn expectZoltState(self: *Run, expected: []const u8, what: []const u8) !void {
        try self.zw.saveState(.all, &self.zolt_state);
        if (try describeStateDifference(self.allocator, self.zolt_state.items, expected, self.cw)) |message| {
            defer self.allocator.free(message);
            return self.fail("{s}: {s}", .{ what, message });
        }
    }

    /// Restore the snapshot and redo the update with enough memory: the C++ bytes
    fn recoverZolt(self: *Run, snapshot: []const u8, reference: []const u8, delta_time: f32, collision_steps: u32, what: []const u8) !void {
        try self.restoreZolt(snapshot);
        _ = try self.zw.update(delta_time, collision_steps, self.zw.temp.tempAllocator(), self.zw.jobSystem());
        if (!self.zw.temp.isEmpty()) return self.fail("{s}: the temp allocator is not empty", .{what});
        try self.expectZoltState(reference, what);
    }
};

/// What a scene did (the sanity checks of runEntry, printed with `verbose`)
const RunStats = struct {
    /// Number of listener records per RecordKind
    records: [SimScenes.num_record_kinds]u32 = @splat(0),
    num_temp_failures: u32 = 0,
    num_job_failures: u32 = 0,
    /// Largest number of active bodies after an update
    max_active_bodies: u32 = 0,
    /// Number of updates that ended with sleeping bodies (bodies in the broad phase that are not active)
    updates_with_sleeping_bodies: u32 = 0,
    /// Number of updates with active linear cast bodies
    updates_with_ccd_bodies: u32 = 0,
    /// Number of updates after which the contact cache (C++ state) holds CCD manifolds: a linear cast body hit something
    updates_with_ccd_manifolds: u32 = 0,
    /// Largest number of FindCCDContacts jobs in one update (Zolt side)
    max_find_ccd_contacts_jobs: u32 = 0,
    /// Largest number of bodies activated from inside one update (Zolt side, the activations of the operations before
    /// the update not counted)
    max_update_activations: u32 = 0,
    /// Phase 9: largest number of bodies deactivated from inside one update (Zolt side, the deactivations of the
    /// operations before the update not counted; Expect.update_deactivations)
    max_update_deactivations: u32 = 0,
    /// Number of restore_cross operations that changed the state
    restores_changing_the_state: u32 = 0,
    /// Number of checked expect_body_state operations
    num_expectations: u32 = 0,
    /// Number of FindCollisions jobs spawned from inside jobs (Zolt side, also in the sweeps)
    num_spawned_jobs: u32 = 0,
    /// Number of OnBodyDeactivated calls from the final batch of a split island (Zolt side, also in the sweeps)
    num_large_island_deactivations: u32 = 0,
    /// Phase 9: number of steps in which the CCD response changed a soft body (Zolt side, also in the sweeps;
    /// Expect.soft_body_ccd)
    num_soft_body_ccd_responses: u32 = 0,
    /// Largest number of manifolds of one body pair in one update (OnContactAdded + OnContactPersisted records)
    max_manifolds_per_body_pair: u32 = 0,
    /// Largest number of active constraints before an update (Zolt side, Expect.active_constraints)
    max_active_constraints: u32 = 0,
    /// Phase 7: what the object states after the updates show (Expect.ground_states, Expect.wheel_contacts)
    objects: ObjectStats = .{},
    /// R8, release_only scenes: the Zolt asserts of the run (the D12 sweeps not counted) and the C++ asserts of the run
    /// (0 without -Dreference_asserts=true)
    num_zolt_asserts: u32 = 0,
    num_jolt_asserts: u32 = 0,
};

/// The `expect_*` operations of Phase 7: they change nothing, the driver fails unless they return `op.count`
fn isPhase7Expectation(kind: OpKind) bool {
    return switch (kind) {
        .character_expect_ground_state, .character_expect_body_state, .character_expect_value, .cv_expect_ground_state, .cv_expect_value, .ragdoll_expect_active, .vehicle_expect_wheel_contacts, .vehicle_expect_constraint_state => true,
        else => false,
    };
}

/// True when `records` (counts per RecordKind) has every RecordKind of the bits (Expect.character_records); the missing
/// kinds are printed
fn recordsReach(records: []const u32, bits: u128) bool {
    var reached = true;
    for (records, 0..) |count, kind| {
        if (kind < 128 and (bits & (@as(u128, 1) << @intCast(kind))) != 0 and count == 0) {
            std.debug.print("SimulationParity: no {t} records\n", .{@as(RecordKind, @enumFromInt(kind))});
            reached = false;
        }
    }
    return reached;
}

/// The largest number of OnContactAdded + OnContactPersisted records of one body pair in `records` (Expect.full_manifolds)
fn maxManifoldsPerBodyPair(allocator: Allocator, records: []const SimRecord) Allocator.Error!u32 {
    var counts: std.AutoHashMapUnmanaged([2]u32, u32) = .empty;
    defer counts.deinit(allocator);
    var max: u32 = 0;
    for (records) |r| {
        if (r.kind != @intFromEnum(RecordKind.contact_added) and r.kind != @intFromEnum(RecordKind.contact_persisted))
            continue;
        const count = try counts.getOrPutValue(allocator, .{ r.ids[0], r.ids[1] }, 0);
        count.value_ptr.* += 1;
        max = @max(max, count.value_ptr.*);
    }
    return max;
}

/// Print what every scene did (a passing test must not print: keep false)
const verbose = false;

/// Run a scene in one configuration
/// `asserts`: the counting assert handler of a release_only scene (R8), null for the other scenes
fn runScene(allocator: Allocator, entry: *const SceneEntry, builder: *const SceneBuilder, config: Config, asserts: ?*zolt.IssueReporting.CountAsserts) !RunStats {
    const zolt_asserts_before = if (asserts) |a| a.get() else 0;
    const jolt_asserts_before = jolt.jolt_sim_reference_assert_count();
    const desc = builder.desc();
    const zw = try ZoltWorld.create(allocator, &desc, config.zolt);
    defer zw.destroy();
    const cw = CppWorld.init(&desc, config.cpp_threads);
    defer cw.deinit();
    if (config.draw) |d| try d.begin(zw);
    defer if (config.draw) |d| d.end(zw);

    var run: Run = .{ .allocator = allocator, .entry = entry, .config = config, .builder = builder, .desc = &desc, .zw = zw, .cw = cw, .asserts = asserts };
    defer run.deinit();

    // The new system
    try run.compareStates(.all, "after creating the system");

    var delta_time = builder.delta_time;
    var collision_steps = builder.collision_steps;
    var op_index: usize = 0;
    while (run.update < builder.num_updates) : (run.update += 1) {
        try run.applyOps(&op_index, &delta_time, &collision_steps);
        try run.step(delta_time, collision_steps);
    }
    std.debug.assert(op_index == builder.ops.items.len);

    // Every part of the state, and the filtered state
    for (all_states) |state|
        try run.compareStates(state, "at the end");
    try run.compareFilteredStates();

    run.stats.num_spawned_jobs = zw.counting_jobs.num_spawned_find_collisions.load(.monotonic);
    run.stats.num_large_island_deactivations = zw.num_large_island_deactivations.load(.monotonic);
    run.stats.num_soft_body_ccd_responses = zw.counting_jobs.num_soft_body_ccd_responses.load(.monotonic);
    if (asserts) |a| run.stats.num_zolt_asserts = a.get() - zolt_asserts_before - run.uncompared_asserts;
    run.stats.num_jolt_asserts = jolt.jolt_sim_reference_assert_count() - jolt_asserts_before;
    return run.stats;
}

/// Creating the scene (the system, the shapes, the operations before the first update) with every allocation failing
/// once (D12)
fn createScene(allocator: Allocator, builder: *const SceneBuilder) !void {
    const desc = builder.desc();
    const zw = try ZoltWorld.create(allocator, &desc, .single_threaded);
    defer zw.destroy();
    for (builder.ops.items) |*op| {
        if (op.update != 0) break;
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .set_update, .snapshot, .restore_cross => {},
            else => _ = try zw.applyOp(op),
        }
    }
}

// -------------------------------------------------------------------------------------------------------------------
// Constraints (Phase 6): the Zolt side of the ConstraintKinds and of the class operations, one slot per worker group
// (see the SimScenes header for the recipe; the C++ side, SimulationReference.cpp, covers every kind and operation).
//
// A slot is a struct with
// - `makeSettings(allocator, c: *const SimConstraint) !Ref(ConstraintSettings)`: the settings of a description of one of
//   the kinds of the slot, the members of the settings class only (exactly SimMakeConstraintSettings of
//   SimulationReference.cpp; makeConstraintSettings sets the ConstraintSettings members);
// - `applyOp(world: *ZoltWorld, op: *const SimOp, c: *TwoBodyConstraint) !void`: an operation of the OpKind range of the
//   slot (ApplyConstraintOp of SimulationReference.cpp, `castConstraint` checks the subtype);
// - `stateFields(sub_type: zolt.ConstraintSubType) ?[]const StateField`: the SaveState layout of the classes of the slot
//   after Constraint::SaveState's `enabled` (for the state decoder, null: decoded as one block of bytes);
// - `setConstraints(world, c: *TwoBodyConstraint, handle1: u32, handle2: u32) !void`: Gear / RackAndPinion only.
// A slot whose classes are not ported yet is `StubSlot`: scenes with its kinds are skipped through their Blocker, an
// operation of its range returns error.SceneNotSupported.

/// The ConstraintState bits of a constraint (SimConstraintState of SimulationReference.cpp)
fn constraintState(c: *const Constraint) u32 {
    return @as(u32, @intFromBool(c.getEnabled())) | (@as(u32, @intFromBool(c.isActive())) << 1);
}

/// A field of the SaveState of a constraint class (for the state decoder)
const StateField = struct {
    name: []const u8,
    kind: FieldKind,
    /// Number of components (3 for a Vec3, 1 for a float)
    count: u8 = 1,
};

/// The constraint `c` as class T after checking its subtype (SimCast of SimulationReference.cpp)
fn castConstraint(comptime T: type, c: *TwoBodyConstraint, sub_type: zolt.ConstraintSubType) !*T {
    if (c.base.getSubType() != sub_type) {
        std.debug.print("SimulationParity: a {t} operation on a {t} constraint (an error of the scene)\n", .{ sub_type, c.base.getSubType() });
        return error.SceneError;
    }
    return zolt.virtual.downcast(T, c);
}

/// The EConstraintSpace of a description member
fn loadSpace(v: u32) zolt.ConstraintSpace {
    return @enumFromInt(@as(i32, @bitCast(v)));
}

/// A spring of a description: mode, frequency / stiffness, damping (SimLoadSpring)
fn loadSpring(mode: u32, f: []const f32) zolt.SpringSettings {
    var s: zolt.SpringSettings = .{};
    s.mode = @enumFromInt(@as(u8, @intCast(mode)));
    s.frequency = f[0];
    s.damping = f[1];
    return s;
}

/// A motor of a description: the spring, min / max force limit, min / max torque limit (SimLoadMotor)
fn loadMotor(mode: u32, f: []const f32) zolt.MotorSettings {
    var m: zolt.MotorSettings = .{};
    m.spring_settings = loadSpring(mode, f);
    m.min_force_limit = f[2];
    m.max_force_limit = f[3];
    m.min_torque_limit = f[4];
    m.max_torque_limit = f[5];
    return m;
}

/// The slot of a worker group whose classes are not ported yet
const StubSlot = struct {
    fn makeSettings(allocator: Allocator, c: *const SimConstraint) !Ref(ConstraintSettings) {
        _ = .{ allocator, c };
        return error.SceneNotSupported;
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp, c: *TwoBodyConstraint) !void {
        _ = .{ world, op, c };
        return error.SceneNotSupported;
    }

    fn stateFields(sub_type: zolt.ConstraintSubType) ?[]const StateField {
        _ = sub_type;
        return null;
    }

    fn setConstraints(world: *ZoltWorld, c: *TwoBodyConstraint, handle1: u32, handle2: u32) !void {
        _ = .{ world, c, handle1, handle2 };
        return error.SceneNotSupported;
    }
};

// ---- SLOT fixed_point: FixedConstraint, PointConstraint (ConstraintKind fixed / point, OpKind 220..239) ----
const FixedPointSlot = struct {
    fn makeSettings(allocator: Allocator, c: *const SimConstraint) !Ref(ConstraintSettings) {
        const i = &c.i;
        const r = &c.r;
        const f = &c.f;
        switch (@as(ConstraintKind, @enumFromInt(c.kind))) {
            .fixed => {
                const s = try zolt.FixedConstraintSettings.create(allocator);
                s.space = loadSpace(i[0]);
                s.auto_detect_point = i[1] != 0;
                s.point1 = loadR3(r[0..3]);
                s.axis_x1 = load3(f[0..3]);
                s.axis_y1 = load3(f[3..6]);
                s.point2 = loadR3(r[3..6]);
                s.axis_x2 = load3(f[6..9]);
                s.axis_y2 = load3(f[9..12]);
                return .init(s.asSettings());
            },
            .point => {
                const s = try zolt.PointConstraintSettings.create(allocator);
                s.space = loadSpace(i[0]);
                s.point1 = loadR3(r[0..3]);
                s.point2 = loadR3(r[3..6]);
                return .init(s.asSettings());
            },
            else => unreachable,
        }
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp, c: *TwoBodyConstraint) !void {
        _ = world;
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .point_set_point1 => (try castConstraint(zolt.PointConstraint, c, .point)).setPoint1(loadSpace(op.i[0]), loadR3(&op.r)),
            .point_set_point2 => (try castConstraint(zolt.PointConstraint, c, .point)).setPoint2(loadSpace(op.i[0]), loadR3(&op.r)),
            else => return error.SceneNotSupported,
        }
    }

    fn stateFields(sub_type: zolt.ConstraintSubType) ?[]const StateField {
        return switch (sub_type) {
            .fixed => &.{ .{ .name = "point_constraint_part.total_lambda", .kind = .f32, .count = 3 }, .{ .name = "rotation_constraint_part.total_lambda", .kind = .f32, .count = 3 } },
            .point => &.{.{ .name = "point_constraint_part.total_lambda", .kind = .f32, .count = 3 }},
            else => null,
        };
    }
};

// ---- SLOT distance_pulley: DistanceConstraint, PulleyConstraint (ConstraintKind distance / pulley, OpKind 240..259) ----
const DistancePulleySlot = struct {
    fn makeSettings(allocator: Allocator, c: *const SimConstraint) !Ref(ConstraintSettings) {
        const i = &c.i;
        const r = &c.r;
        const f = &c.f;
        switch (@as(ConstraintKind, @enumFromInt(c.kind))) {
            .distance => {
                const s = try zolt.DistanceConstraintSettings.create(allocator);
                s.space = loadSpace(i[0]);
                s.point1 = loadR3(r[0..3]);
                s.point2 = loadR3(r[3..6]);
                s.min_distance = f[0];
                s.max_distance = f[1];
                s.limits_spring_settings = loadSpring(i[1], f[2..4]);
                return .init(s.asSettings());
            },
            .pulley => {
                const s = try zolt.PulleyConstraintSettings.create(allocator);
                s.space = loadSpace(i[0]);
                s.body_point1 = loadR3(r[0..3]);
                s.fixed_point1 = loadR3(r[3..6]);
                s.body_point2 = loadR3(r[6..9]);
                s.fixed_point2 = loadR3(r[9..12]);
                s.ratio = f[0];
                s.min_length = f[1];
                s.max_length = f[2];
                return .init(s.asSettings());
            },
            else => unreachable,
        }
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp, c: *TwoBodyConstraint) !void {
        _ = world;
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .distance_set_distance => (try castConstraint(zolt.DistanceConstraint, c, .distance)).setDistance(op.f[0], op.f[1]),
            .distance_set_limits_spring_settings => (try castConstraint(zolt.DistanceConstraint, c, .distance)).setLimitsSpringSettings(loadSpring(op.i[0], op.f[0..2])),
            .pulley_set_length => (try castConstraint(zolt.PulleyConstraint, c, .pulley)).setLength(op.f[0], op.f[1]),
            else => return error.SceneNotSupported,
        }
    }

    fn stateFields(sub_type: zolt.ConstraintSubType) ?[]const StateField {
        return switch (sub_type) {
            .distance => &.{ .{ .name = "axis_constraint.total_lambda", .kind = .f32 }, .{ .name = "world_space_normal", .kind = .f32, .count = 3 } },
            .pulley => &.{ .{ .name = "independent_axis_constraint_part.total_lambda", .kind = .f32 }, .{ .name = "world_space_normal1", .kind = .f32, .count = 3 }, .{ .name = "world_space_normal2", .kind = .f32, .count = 3 } },
            else => null,
        };
    }
};

// ---- SLOT hinge: HingeConstraint (ConstraintKind hinge, OpKind 260..279) ----
const HingeSlot = struct {
    fn makeSettings(allocator: Allocator, c: *const SimConstraint) !Ref(ConstraintSettings) {
        const i = &c.i;
        const r = &c.r;
        const f = &c.f;
        switch (@as(ConstraintKind, @enumFromInt(c.kind))) {
            .hinge => {
                const s = try zolt.HingeConstraintSettings.create(allocator);
                s.space = loadSpace(i[0]);
                s.point1 = loadR3(r[0..3]);
                s.hinge_axis1 = load3(f[0..3]);
                s.normal_axis1 = load3(f[3..6]);
                s.point2 = loadR3(r[3..6]);
                s.hinge_axis2 = load3(f[6..9]);
                s.normal_axis2 = load3(f[9..12]);
                s.limits_min = f[12];
                s.limits_max = f[13];
                s.limits_spring_settings = loadSpring(i[1], f[14..16]);
                s.max_friction_torque = f[16];
                s.motor_settings = loadMotor(i[2], f[17..23]);
                return .init(s.asSettings());
            },
            else => unreachable,
        }
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp, c: *TwoBodyConstraint) !void {
        _ = world;
        const i = &op.i;
        const f = &op.f;
        const hinge = try castConstraint(zolt.HingeConstraint, c, .hinge);
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .hinge_set_max_friction_torque => hinge.setMaxFrictionTorque(f[0]),
            .hinge_set_motor_state => hinge.setMotorState(@enumFromInt(@as(i32, @bitCast(i[0])))),
            .hinge_set_target_angular_velocity => hinge.setTargetAngularVelocity(f[0]),
            .hinge_set_target_angle => hinge.setTargetAngle(f[0]),
            .hinge_set_target_orientation_bs => hinge.setTargetOrientationBS(loadQuat(f[0..4])),
            .hinge_set_limits => hinge.setLimits(f[0], f[1]),
            .hinge_set_limits_spring_settings => hinge.setLimitsSpringSettings(loadSpring(i[0], f[0..2])),
            .hinge_set_motor_settings => hinge.getMotorSettingsMut().* = loadMotor(i[0], f[0..6]),
            else => return error.SceneNotSupported,
        }
    }

    fn stateFields(sub_type: zolt.ConstraintSubType) ?[]const StateField {
        return switch (sub_type) {
            .hinge => &.{
                .{ .name = "motor_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "rotation_constraint_part.total_lambda", .kind = .f32, .count = 2 },
                .{ .name = "point_constraint_part.total_lambda", .kind = .f32, .count = 3 },
                .{ .name = "rotation_limits_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "motor_state", .kind = .u32 },
                .{ .name = "target_angular_velocity", .kind = .f32 },
                .{ .name = "target_angle", .kind = .f32 },
            },
            else => null,
        };
    }
};

// ---- SLOT slider: SliderConstraint (ConstraintKind slider, OpKind 280..299) ----
const SliderSlot = struct {
    fn makeSettings(allocator: Allocator, c: *const SimConstraint) !Ref(ConstraintSettings) {
        const i = &c.i;
        const r = &c.r;
        const f = &c.f;
        switch (@as(ConstraintKind, @enumFromInt(c.kind))) {
            .slider => {
                const s = try zolt.SliderConstraintSettings.create(allocator);
                s.space = loadSpace(i[0]);
                s.auto_detect_point = i[1] != 0;
                s.point1 = loadR3(r[0..3]);
                s.point2 = loadR3(r[3..6]);
                if (i[2] != 0) {
                    s.setSliderAxis(load3(f[0..3]));
                } else {
                    s.slider_axis1 = load3(f[0..3]);
                    s.normal_axis1 = load3(f[3..6]);
                    s.slider_axis2 = load3(f[6..9]);
                    s.normal_axis2 = load3(f[9..12]);
                }
                s.limits_min = f[12];
                s.limits_max = f[13];
                s.limits_spring_settings = loadSpring(i[3], f[14..16]);
                s.max_friction_force = f[16];
                s.motor_settings = loadMotor(i[4], f[17..23]);
                return .init(s.asSettings());
            },
            else => unreachable,
        }
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp, c: *TwoBodyConstraint) !void {
        _ = world;
        const f = &op.f;
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .slider_set_max_friction_force => (try castConstraint(zolt.SliderConstraint, c, .slider)).setMaxFrictionForce(f[0]),
            .slider_set_motor_state => (try castConstraint(zolt.SliderConstraint, c, .slider)).setMotorState(@enumFromInt(@as(i32, @bitCast(op.i[0])))),
            .slider_set_target_velocity => (try castConstraint(zolt.SliderConstraint, c, .slider)).setTargetVelocity(f[0]),
            .slider_set_target_position => (try castConstraint(zolt.SliderConstraint, c, .slider)).setTargetPosition(f[0]),
            .slider_set_limits => (try castConstraint(zolt.SliderConstraint, c, .slider)).setLimits(f[0], f[1]),
            .slider_set_limits_spring_settings => (try castConstraint(zolt.SliderConstraint, c, .slider)).setLimitsSpringSettings(loadSpring(op.i[0], f[0..2])),
            .slider_set_motor_settings => (try castConstraint(zolt.SliderConstraint, c, .slider)).getMotorSettingsMut().* = loadMotor(op.i[0], f[0..6]),
            else => return error.SceneNotSupported,
        }
    }

    fn stateFields(sub_type: zolt.ConstraintSubType) ?[]const StateField {
        return switch (sub_type) {
            .slider => &.{
                .{ .name = "motor_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "position_constraint_part.total_lambda", .kind = .f32, .count = 2 },
                .{ .name = "rotation_constraint_part.total_lambda", .kind = .f32, .count = 3 },
                .{ .name = "position_limits_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "motor_state", .kind = .u32 },
                .{ .name = "target_velocity", .kind = .f32 },
                .{ .name = "target_position", .kind = .f32 },
            },
            else => null,
        };
    }
};

// ---- SLOT cone_swing_twist: ConeConstraint, SwingTwistConstraint (ConstraintKind cone / swing_twist, OpKind 300..319) ----
const ConeSwingTwistSlot = struct {
    fn makeSettings(allocator: Allocator, c: *const SimConstraint) !Ref(ConstraintSettings) {
        const i = &c.i;
        const r = &c.r;
        const f = &c.f;
        switch (@as(ConstraintKind, @enumFromInt(c.kind))) {
            .cone => {
                const s = try zolt.ConeConstraintSettings.create(allocator);
                s.space = loadSpace(i[0]);
                s.point1 = loadR3(r[0..3]);
                s.twist_axis1 = load3(f[0..3]);
                s.point2 = loadR3(r[3..6]);
                s.twist_axis2 = load3(f[3..6]);
                s.half_cone_angle = f[6];
                return .init(s.asSettings());
            },
            .swing_twist => {
                const s = try zolt.SwingTwistConstraintSettings.create(allocator);
                s.space = loadSpace(i[0]);
                s.swing_type = @enumFromInt(@as(u8, @truncate(i[1])));
                s.position1 = loadR3(r[0..3]);
                s.twist_axis1 = load3(f[0..3]);
                s.plane_axis1 = load3(f[3..6]);
                s.position2 = loadR3(r[3..6]);
                s.twist_axis2 = load3(f[6..9]);
                s.plane_axis2 = load3(f[9..12]);
                s.normal_half_cone_angle = f[12];
                s.plane_half_cone_angle = f[13];
                s.twist_min_angle = f[14];
                s.twist_max_angle = f[15];
                s.max_friction_torque = f[16];
                s.swing_motor_settings = loadMotor(i[2], f[17..23]);
                s.twist_motor_settings = loadMotor(i[3], f[23..29]);
                return .init(s.asSettings());
            },
            else => unreachable,
        }
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp, c: *TwoBodyConstraint) !void {
        _ = world;
        const f = &op.f;
        const motor_state: zolt.MotorState = @enumFromInt(@as(i32, @bitCast(op.i[0])));
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .cone_set_half_cone_angle => (try castConstraint(zolt.ConeConstraint, c, .cone)).setHalfConeAngle(f[0]),
            .swing_twist_set_normal_half_cone_angle => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).setNormalHalfConeAngle(f[0]),
            .swing_twist_set_plane_half_cone_angle => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).setPlaneHalfConeAngle(f[0]),
            .swing_twist_set_twist_min_angle => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).setTwistMinAngle(f[0]),
            .swing_twist_set_twist_max_angle => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).setTwistMaxAngle(f[0]),
            .swing_twist_set_max_friction_torque => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).setMaxFrictionTorque(f[0]),
            .swing_twist_set_swing_motor_state => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).setSwingMotorState(motor_state),
            .swing_twist_set_twist_motor_state => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).setTwistMotorState(motor_state),
            .swing_twist_set_target_angular_velocity_cs => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).setTargetAngularVelocityCS(load3(f[0..3])),
            .swing_twist_set_target_angular_velocity_bs => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).setTargetAngularVelocityBS(load3(f[0..3])),
            .swing_twist_set_target_orientation_cs => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).setTargetOrientationCS(loadQuat(f[0..4])),
            .swing_twist_set_target_orientation_bs => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).setTargetOrientationBS(loadQuat(f[0..4])),
            .swing_twist_set_swing_motor_settings => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).getSwingMotorSettingsMut().* = loadMotor(op.i[0], f[0..6]),
            .swing_twist_set_twist_motor_settings => (try castConstraint(zolt.SwingTwistConstraint, c, .swing_twist)).getTwistMotorSettingsMut().* = loadMotor(op.i[0], f[0..6]),
            else => return error.SceneNotSupported,
        }
    }

    fn stateFields(sub_type: zolt.ConstraintSubType) ?[]const StateField {
        return switch (sub_type) {
            .cone => &.{
                .{ .name = "point_constraint_part.total_lambda", .kind = .f32, .count = 3 },
                .{ .name = "angle_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "world_space_rotation_axis", .kind = .f32, .count = 3 },
            },
            .swing_twist => &.{
                .{ .name = "point_constraint_part.total_lambda", .kind = .f32, .count = 3 },
                .{ .name = "swing_twist_constraint_part.swing_limit_y_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "swing_twist_constraint_part.swing_limit_z_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "swing_twist_constraint_part.twist_limit_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "motor_constraint_part[0].total_lambda", .kind = .f32 },
                .{ .name = "motor_constraint_part[1].total_lambda", .kind = .f32 },
                .{ .name = "motor_constraint_part[2].total_lambda", .kind = .f32 },
                .{ .name = "swing_motor_state", .kind = .u32 },
                .{ .name = "twist_motor_state", .kind = .u32 },
                .{ .name = "target_angular_velocity", .kind = .f32, .count = 3 },
                .{ .name = "target_orientation", .kind = .f32, .count = 4 },
            },
            else => null,
        };
    }
};

// ---- SLOT six_dof: SixDOFConstraint (ConstraintKind six_dof, OpKind 320..339) ----
const SixDOFSlot = struct {
    const Axis = zolt.SixDOFConstraintSettings.Axis;

    fn makeSettings(allocator: Allocator, c: *const SimConstraint) !Ref(ConstraintSettings) {
        const i = &c.i;
        const r = &c.r;
        const f = &c.f;
        switch (@as(ConstraintKind, @enumFromInt(c.kind))) {
            .six_dof => {
                const s = try zolt.SixDOFConstraintSettings.create(allocator);
                s.space = loadSpace(i[0]);
                s.swing_type = @enumFromInt(@as(u8, @intCast(i[1])));
                s.position1 = loadR3(r[0..3]);
                s.axis_x1 = load3(f[0..3]);
                s.axis_y1 = load3(f[3..6]);
                s.position2 = loadR3(r[3..6]);
                s.axis_x2 = load3(f[6..9]);
                s.axis_y2 = load3(f[9..12]);
                for (0..Axis.num) |a| {
                    s.max_friction[a] = f[12 + a];
                    s.limit_min[a] = f[18 + a];
                    s.limit_max[a] = f[24 + a];
                    s.motor_settings[a] = loadMotor(i[5 + a], f[36 + 6 * a ..][0..6]);
                }
                for (0..Axis.num_translation) |a|
                    s.limits_spring_settings[a] = loadSpring(i[2 + a], f[30 + 2 * a ..][0..2]);
                return .init(s.asSettings());
            },
            else => unreachable,
        }
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp, c: *TwoBodyConstraint) !void {
        _ = world;
        const i = &op.i;
        const f = &op.f;
        const six_dof = try castConstraint(zolt.SixDOFConstraint, c, .six_dof);
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .six_dof_set_translation_limits => six_dof.setTranslationLimits(load3(f[0..3]), load3(f[3..6])),
            .six_dof_set_rotation_limits => six_dof.setRotationLimits(load3(f[0..3]), load3(f[3..6])),
            .six_dof_set_limits_spring_settings => six_dof.setLimitsSpringSettings(@enumFromInt(i[0]), loadSpring(i[1], f[0..2])),
            .six_dof_set_max_friction => six_dof.setMaxFriction(@enumFromInt(i[0]), f[0]),
            .six_dof_set_motor_state => six_dof.setMotorState(@enumFromInt(i[0]), @enumFromInt(@as(i32, @bitCast(i[1])))),
            .six_dof_set_target_velocity_cs => six_dof.setTargetVelocityCS(load3(f[0..3])),
            .six_dof_set_target_angular_velocity_cs => six_dof.setTargetAngularVelocityCS(load3(f[0..3])),
            .six_dof_set_target_position_cs => six_dof.setTargetPositionCS(load3(f[0..3])),
            .six_dof_set_target_orientation_cs => six_dof.setTargetOrientationCS(loadQuat(f[0..4])),
            .six_dof_set_target_orientation_bs => six_dof.setTargetOrientationBS(loadQuat(f[0..4])),
            .six_dof_set_motor_settings => six_dof.getMotorSettingsMut(@enumFromInt(i[0])).* = loadMotor(i[1], f[0..6]),
            else => return error.SceneNotSupported,
        }
    }

    fn stateFields(sub_type: zolt.ConstraintSubType) ?[]const StateField {
        return switch (sub_type) {
            .six_dof => &.{
                .{ .name = "translation_constraint_part[0].total_lambda", .kind = .f32 },
                .{ .name = "translation_constraint_part[1].total_lambda", .kind = .f32 },
                .{ .name = "translation_constraint_part[2].total_lambda", .kind = .f32 },
                .{ .name = "point_constraint_part.total_lambda", .kind = .f32, .count = 3 },
                .{ .name = "swing_twist_constraint_part.swing_limit_y_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "swing_twist_constraint_part.swing_limit_z_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "swing_twist_constraint_part.twist_limit_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "rotation_constraint_part.total_lambda", .kind = .f32, .count = 3 },
                .{ .name = "motor_translation_constraint_part[0].total_lambda", .kind = .f32 },
                .{ .name = "motor_translation_constraint_part[1].total_lambda", .kind = .f32 },
                .{ .name = "motor_translation_constraint_part[2].total_lambda", .kind = .f32 },
                .{ .name = "motor_rotation_constraint_part[0].total_lambda", .kind = .f32 },
                .{ .name = "motor_rotation_constraint_part[1].total_lambda", .kind = .f32 },
                .{ .name = "motor_rotation_constraint_part[2].total_lambda", .kind = .f32 },
                .{ .name = "motor_state", .kind = .u32, .count = 6 },
                .{ .name = "target_velocity", .kind = .f32, .count = 3 },
                .{ .name = "target_angular_velocity", .kind = .f32, .count = 3 },
                .{ .name = "target_position", .kind = .f32, .count = 3 },
                .{ .name = "target_orientation", .kind = .f32, .count = 4 },
            },
            else => null,
        };
    }
};

// ---- SLOT path: PathConstraint, PathConstraintPathHermite (ConstraintKind path, OpKind 340..359) ----
const PathSlot = struct {
    /// SimMakePath: the Hermite path of a description of kind path (one reference owned by the caller)
    fn makePath(allocator: Allocator, c: *const SimConstraint) !Ref(zolt.PathConstraintPath) {
        const hermite = try zolt.PathConstraintPathHermite.create(allocator);
        var path = Ref(zolt.PathConstraintPath).init(hermite.asPath());
        errdefer path.deinit();
        hermite.base.setIsLooping(c.i[1] != 0);
        for (0..c.i[2]) |p| {
            const pf = c.f[16 + 9 * p ..];
            try hermite.addPoint(load3(pf[0..3]), load3(pf[3..6]), load3(pf[6..9]));
        }
        return path;
    }

    fn makeSettings(allocator: Allocator, c: *const SimConstraint) !Ref(ConstraintSettings) {
        const i = &c.i;
        const f = &c.f;
        var path = try makePath(allocator, c);
        defer path.deinit();
        const s = try zolt.PathConstraintSettings.create(allocator);
        s.path.set(path.get());
        s.path_position = load3(f[0..3]);
        s.path_rotation = loadQuat(f[3..7]);
        s.path_fraction = f[7];
        s.max_friction_force = f[8];
        s.position_motor_settings = loadMotor(i[4], f[9..15]);
        s.rotation_constraint_type = @enumFromInt(@as(i32, @bitCast(i[3])));
        return .init(s.asSettings());
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp, c: *TwoBodyConstraint) !void {
        const i = &op.i;
        const f = &op.f;
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .path_set_path => {
                var path: Ref(zolt.PathConstraintPath) = if (i[0] != 0) try makePath(world.allocator, &world.desc.constraints[i[1]]) else .empty;
                defer path.deinit();
                (try castConstraint(zolt.PathConstraint, c, .path)).setPath(path.get(), f[0]);
            },
            .path_set_max_friction_force => (try castConstraint(zolt.PathConstraint, c, .path)).setMaxFrictionForce(f[0]),
            .path_set_position_motor_state => (try castConstraint(zolt.PathConstraint, c, .path)).setPositionMotorState(@enumFromInt(@as(i32, @bitCast(i[0])))),
            .path_set_target_velocity => (try castConstraint(zolt.PathConstraint, c, .path)).setTargetVelocity(f[0]),
            .path_set_target_path_fraction => (try castConstraint(zolt.PathConstraint, c, .path)).setTargetPathFraction(f[0]),
            .path_set_position_motor_settings => (try castConstraint(zolt.PathConstraint, c, .path)).getPositionMotorSettingsMut().* = loadMotor(i[0], f[0..6]),
            else => return error.SceneNotSupported,
        }
    }

    fn stateFields(sub_type: zolt.ConstraintSubType) ?[]const StateField {
        return switch (sub_type) {
            .path => &.{
                .{ .name = "position_constraint_part.total_lambda", .kind = .f32, .count = 2 },
                .{ .name = "position_limits_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "position_motor_constraint_part.total_lambda", .kind = .f32 },
                .{ .name = "hinge_constraint_part.total_lambda", .kind = .f32, .count = 2 },
                .{ .name = "rotation_constraint_part.total_lambda", .kind = .f32, .count = 3 },
                .{ .name = "max_friction_force", .kind = .f32 },
                .{ .name = "position_motor_settings.spring_settings.mode", .kind = .u8 },
                .{ .name = "position_motor_settings (padding of the C++ struct)", .kind = .bytes, .count = 3 },
                .{ .name = "position_motor_settings.spring_settings.frequency", .kind = .f32 },
                .{ .name = "position_motor_settings.spring_settings.damping", .kind = .f32 },
                .{ .name = "position_motor_settings.min_force_limit", .kind = .f32 },
                .{ .name = "position_motor_settings.max_force_limit", .kind = .f32 },
                .{ .name = "position_motor_settings.min_torque_limit", .kind = .f32 },
                .{ .name = "position_motor_settings.max_torque_limit", .kind = .f32 },
                .{ .name = "position_motor_state", .kind = .u32 },
                .{ .name = "target_velocity", .kind = .f32 },
                .{ .name = "target_path_fraction", .kind = .f32 },
                .{ .name = "path_fraction", .kind = .f32 },
            },
            else => null,
        };
    }
};

// ---- SLOT gear_rack_and_pinion: GearConstraint, RackAndPinionConstraint (ConstraintKind gear / rack_and_pinion,
// OpKind 360..379, `setConstraints` for SimConstraint.constraint1 / constraint2) ----
const GearRackAndPinionSlot = struct {
    fn makeSettings(allocator: Allocator, c: *const SimConstraint) !Ref(ConstraintSettings) {
        const i = &c.i;
        const f = &c.f;
        switch (@as(ConstraintKind, @enumFromInt(c.kind))) {
            .gear => {
                const s = try zolt.GearConstraintSettings.create(allocator);
                s.space = loadSpace(i[0]);
                s.hinge_axis1 = load3(f[0..3]);
                s.hinge_axis2 = load3(f[3..6]);
                if (i[1] != 0)
                    s.setRatio(@bitCast(i[2]), @bitCast(i[3]))
                else
                    s.ratio = f[6];
                return .init(s.asSettings());
            },
            .rack_and_pinion => {
                const s = try zolt.RackAndPinionConstraintSettings.create(allocator);
                s.space = loadSpace(i[0]);
                s.hinge_axis = load3(f[0..3]);
                s.slider_axis = load3(f[3..6]);
                if (i[1] != 0)
                    s.setRatio(@bitCast(i[2]), f[7], @bitCast(i[3]))
                else
                    s.ratio = f[6];
                return .init(s.asSettings());
            },
            else => unreachable,
        }
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp, c: *TwoBodyConstraint) !void {
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .gear_set_constraints => try setConstraints(world, &(try castConstraint(zolt.GearConstraint, c, .gear)).base, op.i[0], op.i[1]),
            .rack_and_pinion_set_constraints => try setConstraints(world, &(try castConstraint(zolt.RackAndPinionConstraint, c, .rack_and_pinion)).base, op.i[0], op.i[1]),
            else => return error.SceneNotSupported,
        }
    }

    fn stateFields(sub_type: zolt.ConstraintSubType) ?[]const StateField {
        return switch (sub_type) {
            .gear => &.{.{ .name = "gear_constraint_part.total_lambda", .kind = .f32 }},
            .rack_and_pinion => &.{.{ .name = "rack_and_pinion_constraint_part.total_lambda", .kind = .f32 }},
            else => null,
        };
    }

    /// SetConstraints of SimulationReference.cpp: SetConstraints of a Gear / RackAndPinion with two constraint handles
    /// (`invalid_handle`, or a handle that is not created yet: nullptr)
    fn setConstraints(world: *ZoltWorld, c: *TwoBodyConstraint, handle1: u32, handle2: u32) !void {
        const c1: ?*const Constraint = if (world.getConstraint(handle1)) |t| &t.base else null;
        const c2: ?*const Constraint = if (world.getConstraint(handle2)) |t| &t.base else null;
        switch (c.base.getSubType()) {
            .gear => zolt.virtual.downcast(zolt.GearConstraint, c).setConstraints(c1, c2),
            .rack_and_pinion => zolt.virtual.downcast(zolt.RackAndPinionConstraint, c).setConstraints(c1, c2),
            else => {
                std.debug.print("SimulationParity: SetConstraints on a constraint of subtype {t}\n", .{c.base.getSubType()});
                return error.SceneError;
            },
        }
    }
};

// ---- end of the slots ----

/// SimMakeConstraintSettings of SimulationReference.cpp: the settings of a description (one reference owned by the
/// caller), error.SceneNotSupported while the slot of the kind is a stub
fn makeConstraintSettings(allocator: Allocator, c: *const SimConstraint) !Ref(ConstraintSettings) {
    var settings = switch (@as(ConstraintKind, @enumFromInt(c.kind))) {
        .fixed, .point => try FixedPointSlot.makeSettings(allocator, c),
        .distance, .pulley => try DistancePulleySlot.makeSettings(allocator, c),
        .hinge => try HingeSlot.makeSettings(allocator, c),
        .slider => try SliderSlot.makeSettings(allocator, c),
        .cone, .swing_twist => try ConeSwingTwistSlot.makeSettings(allocator, c),
        .six_dof => try SixDOFSlot.makeSettings(allocator, c),
        .path => try PathSlot.makeSettings(allocator, c),
        .gear, .rack_and_pinion => try GearRackAndPinionSlot.makeSettings(allocator, c),
    };
    errdefer settings.deinit();

    // The ConstraintSettings members
    const s = settings.get().?;
    s.enabled = c.enabled != 0;
    s.constraint_priority = c.constraint_priority;
    s.num_velocity_steps_override = c.num_velocity_steps_override;
    s.num_position_steps_override = c.num_position_steps_override;
    s.draw_constraint_size = c.draw_constraint_size;
    s.user_data = c.user_data;
    return settings;
}

/// The SaveState layout of a constraint class after Constraint::SaveState's `enabled` (the slot of the class)
fn constraintStateFields(sub_type: zolt.ConstraintSubType) ?[]const StateField {
    return switch (sub_type) {
        .fixed, .point => FixedPointSlot.stateFields(sub_type),
        .distance, .pulley => DistancePulleySlot.stateFields(sub_type),
        .hinge => HingeSlot.stateFields(sub_type),
        .slider => SliderSlot.stateFields(sub_type),
        .cone, .swing_twist => ConeSwingTwistSlot.stateFields(sub_type),
        .six_dof => SixDOFSlot.stateFields(sub_type),
        .path => PathSlot.stateFields(sub_type),
        .gear, .rack_and_pinion => GearRackAndPinionSlot.stateFields(sub_type),
        .vehicle, .user1, .user2, .user3, .user4 => null,
    };
}

/// The SaveBinaryState bytes of settings
fn saveSettingsBytes(allocator: Allocator, settings: *const ConstraintSettings) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var wrapper = zolt.StreamOutWrapper.init(&out.writer);
    settings.saveBinaryState(wrapper.streamOut());
    if (wrapper.isFailed()) return error.OutOfMemory;
    return out.toOwnedSlice();
}

/// Before a scene runs: every constraint kind of the scene is ported (a supported Blocker must cover its kinds), and the
/// settings that the Zolt slot makes from each description have the binary state of the C++ settings (a mismatch is an
/// error of the slot's makeSettings, not of the library)
fn checkConstraintDescriptions(allocator: Allocator, entry: *const SceneEntry, builder: *const SceneBuilder) !void {
    var jolt_bytes: std.ArrayList(u8) = .empty;
    defer jolt_bytes.deinit(allocator);
    for (builder.constraints.items, 0..) |*c, index| {
        const kind: ConstraintKind = @enumFromInt(c.kind);
        if (!kind.isPorted()) {
            std.debug.print("SimulationParity: scene {s} creates {t} constraints, whose Zolt class is still a stub, but its blocker .{t} is supported (declare the Blocker of the kind)\n", .{ entry.name, kind, entry.blocked_by });
            return error.TestUnexpectedResult;
        }
        var settings = makeConstraintSettings(allocator, c) catch |err| {
            if (err == error.SceneNotSupported)
                std.debug.print("SimulationParity: scene {s}: the {t} class is ported but the slot of its kind in SimulationParity.zig is still a stub\n", .{ entry.name, kind });
            return err;
        };
        defer settings.deinit();
        const zolt_bytes = try saveSettingsBytes(allocator, settings.get().?);
        defer allocator.free(zolt_bytes);
        const size = jolt.jolt_sim_constraint_settings_save(c, jolt_bytes.items.ptr, 0);
        try jolt_bytes.resize(allocator, size);
        _ = jolt.jolt_sim_constraint_settings_save(c, jolt_bytes.items.ptr, size);
        if (!std.mem.eql(u8, zolt_bytes, jolt_bytes.items)) {
            std.debug.print("SimulationParity: scene {s}, constraint description {d} ({t}): the settings of the Zolt slot differ from SimMakeConstraintSettings (byte {?d} of {d})\n", .{ entry.name, index, kind, std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes.items), jolt_bytes.items.len });
            return error.ParityMismatch;
        }
    }
}

// -------------------------------------------------------------------------------------------------------------------
// Characters, ragdolls and vehicles (Phase 7): the Zolt side of the classes, one slot per class group (the recipe is in
// the SimScenes header; the C++ side, SimulationReference.cpp, covers every class and operation). `Phase7StubSlot` is
// the slot of a group whose classes are not ported yet and declares every function of every slot below; a filled slot
// implements the functions of its own list (the driver and VehicleSlot call nothing else on it).
//
// Every slot:
// - `Storage`: what the world keeps of the objects of its handles (a field of ZoltWorld.phase7, initialized with .{};
//   VehicleSlot's holds the vehicles of all three vehicle slots).
// - `create(world: *ZoltWorld, op: *const SimOp) !u32`: the create operation of the slot (create_character,
//   create_character_virtual, create_ragdoll, create_vehicle: VehicleSlot creates the tracked and motorcycle vehicles too,
//   with the hooks of the variants): the object of the next handle, the result of ApplyObjectOp in
//   SimulationReference.cpp.
// - `applyOp(world, op) !u32`: the other operations of its Phase7OpRange (CharacterSlot, CharacterVirtualSlot,
//   RagdollSlot, VehicleSlot), exactly ApplyCharacterOp / ApplyCharacterVirtualOp / ApplyRagdollOp / ApplyVehicleOp of the
//   .cpp (lock: i[3] == 0, the filters of SimCharacterFilters, the world's temp allocator for the character virtual
//   queries).
// - `writeState(world, w: *ObjectStateWriter) !void`: the object state of its objects in handle order, the fields of
//   SimScenes.objectFieldLayout in the order of WriteObjectState of the .cpp (CharacterSlot, CharacterVirtualSlot,
//   RagdollSlot, VehicleSlot; VehicleSlot writes the parts of the tracked and motorcycle controllers with their hooks).
// - `saveStates(world, recorder: *StateRecorderImpl) !void` / `restoreStates(world, recorder) !void`: SaveState /
//   RestoreState of its characters in handle order (CharacterSlot, CharacterVirtualSlot; the others do nothing).
// - `destroyAll(world: *ZoltWorld) void`: remove its objects from the system, destroy them and free its Storage (~SimWorld
//   of the .cpp; ZoltWorld.destroy calls it before the system is destroyed). It must not allocate, also not through a
//   listener record (OnBodyDeactivated of a removed active body): the creation sweep counts an allocation of the
//   teardown that fails as a swallowed error.OutOfMemory (see RagdollSlot.destroyAll).
//
// CharacterVirtualSlot also (the CharacterVirtual group):
// - `characterIdSequence(world, first_id: u32, out: *[SimScenes.character_id_sequence_len]u32) !void`: the CharacterID
//   test (jolt_sim_character_id_sequence, with shape 0 of the world), compared by the test "SimulationParity CharacterID
//   consumption" (M8 depends on which constructions consume an ID).
// RagdollSlot also (the Ragdoll group):
// - `settingsBytes(world, index: u32, restore_from: ?[]const u8, out: *std.ArrayList(u8)) !bool`: the SaveBinaryState
//   bytes (with the shapes and the group filter) of the RagdollSettings of ragdoll description `index` as `create` builds
//   them, or RagdollSettings.restoreFromBinaryState of `restore_from` (every byte read) saved again; false when the
//   restore failed (jolt_sim_ragdoll_settings_bytes).
// - `settingsInfo(world, index: u32, out: *std.ArrayList(u32)) !void`: the result of Stabilize (1 / 0, 2: not called),
//   the number of parts and GetBodyIndexToConstraintIndex, the number of constraints and the pairs of
//   GetConstraintIndexToBodyIdxPair (jolt_sim_ragdoll_settings_info).
// VehicleSlot also (the vehicle core group):
// - `vehicle(world, handle: u32) !*VehicleConstraint`: the vehicle of a vehicle handle (for the variants' operations).
// - `constraintState(vehicle: *VehicleConstraint) u32`: its ConstraintState bits (the result of the vehicle operations).
// - `settingsPart(allocator, scene: *const SimSceneDesc, index: u32, part: VehicleSettingsPart, item: u32,
//   restore_from: ?[]const u8, out: *std.ArrayList(u8)) !bool`: the bytes of a part of vehicle description `index`
//   (jolt_sim_vehicle_settings_part: SaveBinaryState of the part as `create` makes it, or a default object of the part's
//   class restores `restore_from`, every byte read, and saves again; false: no such part, or the restore failed). The
//   classes of the variants come from their hooks (`makeControllerSettings` / `makeWheelSettings` of the description, of
//   `SimVehicle.defaults(kind)` / `SimVehicleWheel.defaults(kind)` for a default object), their bytes from the virtual
//   saveBinaryState / restoreBinaryState of the settings classes.
// - For the variants (called only by the filled variant slots, so not part of Phase7StubSlot): the vehicle_engine and
//   vehicle_transmission fields of an engine / transmission (WriteEngineState / WriteTransmissionState of the .cpp: the
//   tracked controller writes its own engine and transmission with them) and the wheeled members of controller settings
//   (SimFillWheeled: MotorcycleControllerSettings is a WheeledVehicleControllerSettings).
// TrackedVehicleSlot and MotorcycleSlot also (the vehicle variants group; VehicleSlot and the driver call them, so that
// the variants never edit VehicleSlot):
// - `makeControllerSettings(allocator, scene: *const SimSceneDesc, vehicle: *const SimVehicle) !*VehicleControllerSettings`:
//   new controller settings of a description (no reference yet, like the `create` functions of the library;
//   SimMakeTrackedControllerSettings / SimMakeMotorcycleControllerSettings of the .cpp).
// - `makeWheelSettings(allocator, scene, wheel: *const SimVehicleWheel) !*WheelSettings`: TrackedVehicleSlot only: new
//   WheelSettingsTV of a wheel of kind tv (SimMakeWheelSettingsTV).
// - `applyVehicleOp(world, op, vehicle: *VehicleConstraint) !void`: the operations of its range (ApplyTrackedOp /
//   ApplyMotorcycleOp of the .cpp); the driver returns VehicleSlot.constraintState(vehicle).
// - `writeWheelState(world, w, handle: u32, index: u32, wheel: *const Wheel) !void`: TrackedVehicleSlot only: the
//   vehicle_wheel_tv field of wheel `index` (WriteTrackedWheelState), where VehicleSlot writes vehicle_wheel_wv.
// - `writeControllerState(world, w, handle: u32, vehicle: *const VehicleConstraint) !void`: the controller's fields
//   (WriteTrackedControllerState: the engine and the transmission, the inputs, the tracks;
//   WriteMotorcycleControllerState: vehicle_motorcycle, after the wheeled fields that VehicleSlot writes).
// - `engine(vehicle: *VehicleConstraint) !*VehicleEngine` / `transmission(vehicle) !*VehicleTransmission`:
//   TrackedVehicleSlot only: the engine / transmission of the TrackedVehicleController, for VehicleSlot's engine and
//   transmission operations on a tracked vehicle (GetEngine / GetTransmission of the .cpp; a motorcycle controller is a
//   wheeled controller, VehicleSlot reaches its engine itself).
//
// Notes for the groups:
// - CharacterVirtualSlot: its listener is the port of SimCharacterContactListener (CharacterListenerMode record /
//   modify, the CharacterFlags) and adds `CharacterRecords` to world.character_recorder (a record that cannot be stored
//   fails the operation, see ZoltWorld.applyOp). The driver does not sweep the temp allocations of the character virtual
//   operations (they run between the updates, the D12 sweeps cover PhysicsSystem.update): the CharacterVirtual group
//   sweeps them in its own tests. Jolt's non-validating RestoreState keeps the members that it does not restore
//   (mIsBackFacingContact, ...) of the contacts that mActiveContacts.resize keeps (CharacterVirtual.cpp:1977-1981): the
//   object state compares them (cv_active_contact is_back_facing_contact). `mutable mMaxHitsExceeded`
//   (CharacterVirtual.h:752) falls under Rule M (PortingGuide section 5). The CharacterVirtual and Character scenes set
//   `restore_broad_phase`: the character queries find their hits in the order of the broad phase layout, which the
//   failed updates of the D12 sweeps change (BroadPhaseSnapshot restores it after the sweeps). Any slot whose operations
//   query the broad phase in hit order (vehicle testers that keep the first hit, ragdoll queries) can use it too.
// - The vehicle slots: the callbacks (VehicleCallbacks, VehicleCombineFriction, VehicleTireMaxImpulse) add
//   `VehicleRecords` to world.recorder; the custom friction uses the Math min helper (not @min). Wheel's contact members
//   are zero-initialized in Zolt (the reference zero-fills its allocations, see the rules of the SimScenes header).
// A slot whose classes are not ported yet is `Phase7StubSlot` (`is_stub`): the scenes of its classes are skipped through
// their Blocker, and a supported scene that creates its objects fails (checkPhase7Slots).
//
// The interface changes of the foundation review (for a group that started on the earlier interface):
// - TrackedVehicleSlot / MotorcycleSlot: `applyOp(world, op) !u32` is now `applyVehicleOp(world, op, vehicle) !void`
//   (the driver finds the vehicle with VehicleSlot.vehicle and returns VehicleSlot.constraintState). The ranges are
//   routed by Phase7OpRange: the vehicle core 520..569, tracked 570..579, motorcycle 580..589 (were 554..559 and
//   560..566; the variant operations are renumbered on both sides). New hooks of the variants: makeControllerSettings,
//   makeWheelSettings, writeWheelState, writeControllerState, engine, transmission (VehicleSlot calls them in create,
//   writeState, settingsPart and its engine / transmission operations, so the variants never edit VehicleSlot).
// - New functions: CharacterVirtualSlot.characterIdSequence, RagdollSlot.settingsBytes / settingsInfo,
//   VehicleSlot.vehicle / constraintState / settingsPart (the CharacterID test and checkPhase7Settings call them).
// - New operations: character_body_hash (421), the vehicle core operations 554..562 (the Constraint interface of
//   VehicleConstraint, vehicle_expect_constraint_state, WheelWV::ApplyTorque, VehicleEngine::ApplyTorque /
//   ApplyDamping); SimScenes.ObjectField.vehicle_properties holds the constraint priority, the step overrides and the
//   user data of a vehicle; a character state restore must read every byte (ZoltWorld.restoreCharacterStates);
//   destroyAll frees the slot's Storage too.

/// The slot of a class group whose classes are not ported yet: the functions of every slot, every operation returns
/// error.SceneNotSupported
const Phase7StubSlot = struct {
    pub const is_stub = true;

    const Storage = struct {};

    fn create(world: *ZoltWorld, op: *const SimOp) !u32 {
        _ = .{ world, op };
        return error.SceneNotSupported;
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp) !u32 {
        _ = .{ world, op };
        return error.SceneNotSupported;
    }

    fn writeState(world: *ZoltWorld, w: *ObjectStateWriter) !void {
        _ = .{ world, w };
    }

    fn saveStates(world: *ZoltWorld, recorder: *StateRecorderImpl) !void {
        _ = .{ world, recorder };
    }

    fn restoreStates(world: *ZoltWorld, recorder: *StateRecorderImpl) !void {
        _ = .{ world, recorder };
    }

    fn destroyAll(world: *ZoltWorld) void {
        _ = world;
    }

    // CharacterVirtualSlot

    fn characterIdSequence(world: *ZoltWorld, first_id: u32, out: *[SimScenes.character_id_sequence_len]u32) !void {
        _ = .{ world, first_id, out };
        return error.SceneNotSupported;
    }

    // RagdollSlot

    fn settingsBytes(world: *ZoltWorld, index: u32, restore_from: ?[]const u8, out: *std.ArrayList(u8)) !bool {
        _ = .{ world, index, restore_from, out };
        return error.SceneNotSupported;
    }

    fn settingsInfo(world: *ZoltWorld, index: u32, out: *std.ArrayList(u32)) !void {
        _ = .{ world, index, out };
        return error.SceneNotSupported;
    }

    // VehicleSlot

    fn vehicle(world: *ZoltWorld, handle: u32) !*VehicleConstraint {
        _ = .{ world, handle };
        return error.SceneNotSupported;
    }

    fn constraintState(v: *VehicleConstraint) u32 {
        _ = v;
        return 0;
    }

    fn settingsPart(allocator: Allocator, scene: *const SimSceneDesc, index: u32, part: VehicleSettingsPart, item: u32, restore_from: ?[]const u8, out: *std.ArrayList(u8)) !bool {
        _ = .{ allocator, scene, index, part, item, restore_from, out };
        return error.SceneNotSupported;
    }

    // TrackedVehicleSlot, MotorcycleSlot

    fn makeControllerSettings(allocator: Allocator, scene: *const SimSceneDesc, v: *const SimVehicle) !*VehicleControllerSettings {
        _ = .{ allocator, scene, v };
        return error.SceneNotSupported;
    }

    fn makeWheelSettings(allocator: Allocator, scene: *const SimSceneDesc, wheel: *const SimVehicleWheel) !*WheelSettings {
        _ = .{ allocator, scene, wheel };
        return error.SceneNotSupported;
    }

    fn applyVehicleOp(world: *ZoltWorld, op: *const SimOp, v: *VehicleConstraint) !void {
        _ = .{ world, op, v };
        return error.SceneNotSupported;
    }

    fn writeWheelState(world: *ZoltWorld, w: *ObjectStateWriter, handle: u32, index: u32, wheel: *const Wheel) !void {
        _ = .{ world, w, handle, index, wheel };
        return error.SceneNotSupported;
    }

    fn writeControllerState(world: *ZoltWorld, w: *ObjectStateWriter, handle: u32, v: *const VehicleConstraint) !void {
        _ = .{ world, w, handle, v };
        return error.SceneNotSupported;
    }

    // TrackedVehicleSlot: the engine and the transmission of a tracked vehicle for VehicleSlot's operations. The return
    // types name the classes of the vehicle core group, which are not ported yet: Zig analyzes these functions only when
    // a filled VehicleSlot calls them, when the classes exist.

    fn engine(v: *VehicleConstraint) !*zolt.VehicleEngine {
        _ = v;
        return error.SceneNotSupported;
    }

    fn transmission(v: *VehicleConstraint) !*zolt.VehicleTransmission {
        _ = v;
        return error.SceneNotSupported;
    }
};

// ---- SLOT character: Character (Phase7OpRange.character, Blocker.characters) ----
/// The Zolt side of Character: ApplyCharacterOp, WriteCharacterState, SaveCharacterStates / RestoreCharacterStates (the
/// characters) and ~SimWorld (the characters) of SimulationReference.cpp. Proof that the Character scenes bite (each
/// breakage of Zolt/Physics/Character/Character.zig, then reverted): taking the ground position from contact_point_on1
/// in PostSimulation's collector is reported as "scene character_ops ... update 1: ... object state differs at byte 24
/// ...: character_save_state (object handle 0, item 0) ground_position.y zolt 0x3c8d88e0 jolt 0x0"; subtracting the
/// centers of mass the other way round in the check of SetShape as "scene character_crouch_stand ... update 12: operation
/// 54 (character_set_shape on object handle / description 0) returned 0 in zolt and 1 in jolt" (and in
/// character_far_from_origin); EActiveEdgeMode::CollideWithAll in CheckCollision as "scene
/// character_ground_states_meshes ... update 7: ... character_save_state (object handle 5, item 0) ground_normal.y zolt
/// 0x3ebeaa9e jolt 0x3ebeaa9f"; the ground velocity at the character's position instead of the ground position as
/// "scene character_moving_platforms ... update 1: ... character_save_state (object handle 1, item 0) ground_velocity.x
/// zolt 0x00000000 jolt 0x2fbdb005" (and in character_many); CharacterBodyFilter not ignoring sensors as "scene
/// character_ground_states_boxes ... update 30: operation 238 (character_check_collision on object handle / description
/// 0) returned 862843812 in zolt and 1268118805 in jolt" (the walker's check next to the sensor box it crosses).
/// Gap: SimShape has no materials, so GetGroundMaterial (character_properties.ground_material) is only compared as
/// PhysicsMaterial::sDefault (the material of a shape without one); a ground with a non-default material is not tested.
const CharacterSlot = struct {
    /// The characters of the character handles, one reference each (Jolt: Array<Ref<Character>>; an empty reference is
    /// a destroyed character)
    const Storage = struct {
        characters: std.ArrayList(Ref(zolt.CharacterBase)) = .empty,
    };

    /// FNV-1a over values (SimHasher of SimulationReference.cpp)
    const Hasher = struct {
        hash: u32 = SimScenes.hash_seed,

        fn value(h: *Hasher, v: anytype) void {
            h.hash = SimScenes.hashBytes(h.hash, std.mem.asBytes(&v));
        }

        fn boolean(h: *Hasher, v: bool) void {
            h.value(@as(u32, @intFromBool(v)));
        }

        fn vec3(h: *Hasher, v: Vec3) void {
            h.value([3]f32{ v.getX(), v.getY(), v.getZ() });
        }

        fn rvec3(h: *Hasher, v: RVec3) void {
            h.value([3]Real{ v.getX(), v.getY(), v.getZ() });
        }

        fn quat(h: *Hasher, q: Quat) void {
            h.value([4]f32{ q.getX(), q.getY(), q.getZ(), q.getW() });
        }

        fn plane(h: *Hasher, p: zolt.Plane) void {
            h.vec3(p.getNormal());
            h.value(p.getConstant());
        }

        fn rmat44(h: *Hasher, m: zolt.RMat44) void {
            for (0..3) |c| {
                const col = m.getColumn4(@intCast(c));
                h.value([4]f32{ col.getX(), col.getY(), col.getZ(), col.getW() });
            }
            h.rvec3(m.getTranslation());
        }
    };

    /// The collector of character_check_collision (SimCheckCollisionCollector of SimulationReference.cpp): the hash of
    /// the hits in their order. Character::CheckCollision never sets user data (only CharacterVsCharacterCollisionSimple
    /// passes the other character, which the CharacterVirtual operations use), so the character ID is always invalid.
    const CheckCollisionCollector = struct {
        pub const overrides = .{ .addHit, .setUserData };

        base: zolt.CollideShapeCollector = .init(CheckCollisionCollector),
        hasher: Hasher = .{},
        num_hits: u32 = 0,
        user_data: u64 = 0,

        pub fn setUserData(self: *CheckCollisionCollector, user_data: u64) void {
            self.user_data = user_data;
        }

        pub fn addHit(self: *CheckCollisionCollector, result: *const CollideShapeResult) void {
            const h = &self.hasher;
            h.vec3(result.contact_point_on1);
            h.vec3(result.contact_point_on2);
            h.vec3(result.penetration_axis);
            h.value(result.penetration_depth);
            h.value(result.sub_shape_id1.getValue());
            h.value(result.sub_shape_id2.getValue());
            h.value(result.body_id2.getIndexAndSequenceNumber());
            h.value(zolt.CharacterID.invalid_character_id);
            self.num_hits += 1;
        }

        fn finish(self: *CheckCollisionCollector) !u32 {
            if (self.user_data != 0) return error.SceneNotSupported; // A character virtual's ID: not for Character
            self.hasher.value(self.num_hits);
            return self.hasher.hash;
        }
    };

    /// The character of a handle (a handle that does not exist or a destroyed character is an error of the scene)
    fn character(world: *ZoltWorld, handle: u32) !*zolt.Character {
        const characters = world.phase7.characters.characters.items;
        if (handle >= characters.len) return error.SceneError;
        const base = characters[handle].get() orelse return error.SceneError;
        return zolt.virtual.downcast(zolt.Character, base);
    }

    /// GetShapeIndex of SimulationReference.cpp (invalid_handle: not a shape of the scene, or null)
    fn shapeIndex(world: *const ZoltWorld, shape: ?*const Shape) u32 {
        for (world.shapes.items, 0..) |s, i| {
            if (s.get() == shape) return @intCast(i);
        }
        return invalid_handle;
    }

    /// The shape of a shape index (a scene must not pass invalid_handle where Jolt dereferences the shape)
    fn shapeOf(world: *const ZoltWorld, index: u32) !*const Shape {
        if (index >= world.shapes.items.len) return error.SceneError;
        return world.shapes.items[index].get().?;
    }

    /// SimFillCharacterBaseSettings of SimulationReference.cpp
    fn fillBaseSettings(world: *const ZoltWorld, d: *const SimScenes.SimCharacterBase, out: *zolt.CharacterBaseSettings) void {
        out.up = load3(&d.up);
        out.supporting_volume = zolt.Plane.init(load3(d.supporting_volume[0..3]), d.supporting_volume[3]);
        out.max_slope_angle = d.max_slope_angle;
        out.enhanced_internal_edge_removal = d.enhanced_internal_edge_removal != 0;
        out.shape.set(if (d.shape != invalid_handle) world.shapes.items[d.shape].get() else null);
    }

    /// GetBodyState of SimulationReference.cpp: IsActive | IsAdded << 1 through the locking BodyInterface
    fn bodyState(world: *ZoltWorld, body_id: BodyID) u32 {
        const bi = world.system.getBodyInterface();
        return @as(u32, @intFromBool(bi.isActive(body_id))) | (@as(u32, @intFromBool(bi.isAdded(body_id))) << 1);
    }

    fn create(world: *ZoltWorld, op: *const SimOp) !u32 {
        const allocator = world.allocator;
        const d = &world.desc.characters[op.index];
        try world.phase7.characters.characters.ensureUnusedCapacity(allocator, 1);
        const settings = try zolt.CharacterSettings.create(allocator);
        var settings_ref = RefConst(zolt.CharacterBaseSettings).init(&settings.base);
        defer settings_ref.deinit();
        fillBaseSettings(world, &d.base, &settings.base);
        settings.layer = @intCast(d.layer);
        settings.mass = d.mass;
        settings.friction = d.friction;
        settings.gravity_factor = d.gravity_factor;
        settings.allowed_dofs = .fromInt(@intCast(d.allowed_dofs));
        const c = try zolt.Character.create(allocator, settings, loadR3(&d.position), loadQuat(&d.rotation), d.user_data, &world.system);
        world.phase7.characters.characters.appendAssumeCapacity(.init(&c.base));
        return c.getBodyID().getIndexAndSequenceNumber();
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp) !u32 {
        const i = &op.i;
        const f = &op.f;
        const r = loadR3(&op.r);
        const lock_bodies = i[3] == 0;
        const lock: zolt.Character.LockOptions = .{ .lock_bodies = lock_bodies };
        const activation: zolt.Character.ActivationOptions = .{ .activation = if (op.activation == 0) .activate else .dont_activate, .lock_bodies = lock_bodies };
        const c = try character(world, op.index);
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .character_add_to_physics_system => try c.addToPhysicsSystem(activation),
            .character_remove_from_physics_system => c.removeFromPhysicsSystem(lock),
            .character_activate => c.activate(lock),
            .character_post_simulation => {
                c.postSimulation(f[0], lock);
                return @bitCast(@intFromEnum(c.base.getGroundState()));
            },
            .character_set_linear_and_angular_velocity => c.setLinearAndAngularVelocity(load3(f[0..3]), load3(f[3..6]), lock),
            .character_set_linear_velocity => c.setLinearVelocity(load3(f[0..3]), lock),
            .character_add_linear_velocity => c.addLinearVelocity(load3(f[0..3]), lock),
            .character_add_impulse => c.addImpulse(load3(f[0..3]), lock),
            .character_set_position_and_rotation => c.setPositionAndRotation(r, loadQuat(f[0..4]), activation),
            .character_set_position => c.setPosition(r, activation),
            .character_set_rotation => c.setRotation(loadQuat(f[0..4]), activation),
            .character_set_layer => try c.setLayer(@intCast(i[0]), lock),
            .character_set_shape => return @intFromBool(try c.setShape(try shapeOf(world, i[0]), f[0], lock)),
            .character_set_max_slope_angle => c.base.setMaxSlopeAngle(f[0]),
            .character_set_up => c.base.setUp(load3(f[0..3])),
            .character_set_supporting_volume => c.base.setSupportingVolume(zolt.Plane.init(load3(f[0..3]), f[3])),

            .character_destroy => {
                world.phase7.characters.characters.items[op.index].deinit(); // The last reference: destroys the body
                return 0;
            },

            .character_settings_hash => {
                var s = c.getCharacterSettings(lock);
                defer s.base.deinit();
                var h: Hasher = .{};
                h.vec3(s.base.up);
                h.plane(s.base.supporting_volume);
                h.value(s.base.max_slope_angle);
                h.boolean(s.base.enhanced_internal_edge_removal);
                h.value(shapeIndex(world, s.base.shape.get()));
                h.value(@as(u32, s.layer));
                h.value(s.mass);
                h.value(s.friction);
                h.value(s.gravity_factor);
                h.value(@as(u32, s.allowed_dofs.toInt()));
                return h.hash;
            },

            .character_check_collision => {
                var collector: CheckCollisionCollector = .{};
                c.checkCollision(r, loadQuat(f[0..4]), load3(f[4..7]), f[7], try shapeOf(world, i[0]), r, &collector.base, lock);
                return collector.finish();
            },

            .character_expect_ground_state => return @bitCast(@intFromEnum(c.base.getGroundState())), // Changes nothing, the driver checks the result
            .character_expect_body_state => {}, // Changes nothing, the driver checks the BodyState bits
            .character_expect_value => return switch (i[0]) { // Changes nothing, the driver checks the result
                @intFromEnum(SimScenes.CharacterValue.shape) => shapeIndex(world, c.base.getShape()),
                @intFromEnum(SimScenes.CharacterValue.body_shape) => blk: {
                    var body_shape = world.system.getBodyInterface().getShape(c.getBodyID());
                    defer body_shape.deinit();
                    break :blk shapeIndex(world, body_shape.get());
                },
                @intFromEnum(SimScenes.CharacterValue.layer) => @as(u32, c.getLayer()),
                @intFromEnum(SimScenes.CharacterValue.ground_body_is) => @intFromBool(c.base.getGroundBodyID().eql(world.getHandle(op.body))),
                else => error.SceneError,
            },

            .character_body_hash => {
                var h: Hasher = .{};
                h.vec3(c.getLinearVelocity(lock));
                const pr = c.getPositionAndRotation(lock);
                h.rvec3(pr.position);
                h.quat(pr.rotation);
                h.rvec3(c.getPosition(lock));
                h.quat(c.getRotation(lock));
                h.rvec3(c.getCenterOfMassPosition(lock));
                h.rmat44(c.getWorldTransform(lock));
                var ts = c.getTransformedShape(lock);
                defer ts.deinit();
                h.rvec3(ts.shape_position_com);
                h.quat(ts.shape_rotation);
                h.vec3(ts.getShapeScale());
                h.value(ts.body_id.getIndexAndSequenceNumber());
                h.value(shapeIndex(world, ts.shape.get()));
                return h.hash;
            },

            else => return error.SceneError,
        }
        return bodyState(world, c.getBodyID());
    }

    /// WriteCharacterState of SimulationReference.cpp for every character in handle order
    fn writeState(world: *ZoltWorld, w: *ObjectStateWriter) !void {
        for (world.phase7.characters.characters.items, 0..) |*ref, handle| {
            const base = ref.get() orelse continue;
            const c = zolt.virtual.downcast(zolt.Character, base);
            const h: u32 = @intCast(handle);

            var recorder = StateRecorderImpl.init(world.allocator);
            defer recorder.deinit();
            c.base.saveState(recorder.recorder());
            if (recorder.isFailed()) return error.OutOfMemory;
            try w.begin(.character_save_state, h, 0);
            try w.raw(recorder.getData());
            w.end();

            try w.begin(.character_properties, h, 0);
            try w.value(c.getBodyID().getIndexAndSequenceNumber());
            try w.value(@as(u32, c.getLayer()));
            try w.value(shapeIndex(world, c.base.getShape()));
            try w.boolean(c.base.isSupported());
            try w.value(SimScenes.materialHash(c.base.getGroundMaterial().getDebugName()));
            try w.value(c.base.getGroundUserData());
            w.end();

            try w.begin(.character_base, h, 0);
            try w.vec3(c.base.getUp());
            try w.value(c.base.getCosMaxSlopeAngle());
            const plane = c.base.getSupportingVolume();
            try w.vec3(plane.getNormal());
            try w.value(plane.getConstant());
            w.end();

            // The body (a character without body: out of bodies)
            if (c.getBodyID().isInvalid())
                continue;
            try w.begin(.character_body, h, 0);
            try w.rvec3(c.getPosition(.{}));
            try w.quat(c.getRotation(.{}));
            try w.rvec3(c.getCenterOfMassPosition(.{}));
            try w.vec3(c.getLinearVelocity(.{}));
            try w.rmat44(c.getWorldTransform(.{}));
            w.end();

            var ts = c.getTransformedShape(.{});
            defer ts.deinit();
            try w.begin(.character_transformed_shape, h, 0);
            try w.rvec3(ts.shape_position_com);
            try w.quat(ts.shape_rotation);
            try w.vec3(ts.getShapeScale());
            try w.value(ts.body_id.getIndexAndSequenceNumber());
            try w.value(shapeIndex(world, ts.shape.get()));
            w.end();
        }
    }

    fn saveStates(world: *ZoltWorld, recorder: *StateRecorderImpl) !void {
        for (world.phase7.characters.characters.items) |*ref| {
            if (ref.get()) |base| base.saveState(recorder.recorder());
        }
    }

    fn restoreStates(world: *ZoltWorld, recorder: *StateRecorderImpl) !void {
        for (world.phase7.characters.characters.items) |*ref| {
            if (ref.get()) |base| try base.restoreState(recorder.recorder());
        }
    }

    /// ~SimWorld: remove the characters that are in the system (Jolt asserts that a destroyed body is not in the broad
    /// phase), then release them while the system exists (the destructor destroys the body)
    fn destroyAll(world: *ZoltWorld) void {
        const storage = &world.phase7.characters;
        const bi = world.system.getBodyInterface();
        for (storage.characters.items) |*ref| {
            const base = ref.get() orelse continue;
            const c = zolt.virtual.downcast(zolt.Character, base);
            if (!c.getBodyID().isInvalid() and bi.isAdded(c.getBodyID()))
                c.removeFromPhysicsSystem(.{});
        }
        for (storage.characters.items) |*ref| ref.deinit();
        storage.characters.deinit(world.allocator);
        storage.* = .{};
    }
};

// ---- SLOT character_virtual: CharacterVirtual, CharacterVsCharacterCollisionSimple and the CharacterContactListener
// (Phase7OpRange.character_virtual, Blocker.character_virtuals) ----
const CharacterVirtualSlot = struct {
    const CharacterVirtual = zolt.CharacterVirtual;
    const CharacterVirtualSettings = zolt.CharacterVirtualSettings;
    const CharacterContact = zolt.CharacterContact;
    const CharacterContactSettings = zolt.CharacterContactSettings;
    const CharacterContactListener = zolt.CharacterContactListener;
    const CharacterVsCharacterCollisionSimple = zolt.CharacterVsCharacterCollisionSimple;
    const CharacterID = zolt.CharacterID;
    const PhysicsMaterial = zolt.PhysicsMaterial;
    const CharacterFlags = SimScenes.CharacterFlags;
    const SimCharacterVirtual = SimScenes.SimCharacterVirtual;

    const Storage = struct {
        /// The character virtuals of the handles (one reference each, null: destroyed)
        characters: std.ArrayList(?*CharacterVirtual) = .empty,
        /// The CharacterVsCharacterCollisionSimple objects (`collision_group`), initialized by the first creation
        groups: [SimScenes.num_character_collision_groups]CharacterVsCharacterCollisionSimple = undefined,
        groups_initialized: bool = false,
        /// CharacterListenerMode.record / modify
        record_listener: SimCharacterContactListener = .{},
        modify_listener: SimCharacterContactListener = .{ .modify = true },
    };

    /// SimCharacterContactListener of SimulationReference.cpp: records every callback in call order, in modify mode acts
    /// on the CharacterFlags
    const SimCharacterContactListener = struct {
        recorder: *Recorder = undefined,
        modify: bool = false,

        fn flags(user_data: u64) u32 {
            return @truncate(user_data >> 32);
        }

        pub fn onAdjustBodyVelocity(self: *SimCharacterContactListener, character: *const CharacterVirtual, body2: *const Body, linear_velocity: *Vec3, angular_velocity: *Vec3) void {
            const linear_in = linear_velocity.*;
            const angular_in = angular_velocity.*;
            if (self.modify and (flags(body2.getUserData()) & CharacterFlags.conveyor) != 0) {
                linear_velocity.* = linear_velocity.add(Vec3.init(1.5, 0.0, 0.0));
                angular_velocity.* = angular_velocity.add(Vec3.init(0.0, 0.5, 0.0));
            }
            self.recorder.add(CharacterRecords.adjustBodyVelocity(character.getID().getValue(), body2.getID(), linear_in, angular_in, linear_velocity.*, angular_velocity.*));
        }

        pub fn onContactValidate(self: *SimCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact) bool {
            return self.validate(.character_contact_validate, character, contact);
        }

        pub fn onContactAdded(self: *SimCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
            self.settingsCallback(.character_contact_added, character, contact, settings);
        }

        pub fn onContactPersisted(self: *SimCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
            self.settingsCallback(.character_contact_persisted, character, contact, settings);
        }

        pub fn onContactRemoved(self: *SimCharacterContactListener, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID) void {
            self.recorder.add(CharacterRecords.removed(.character_contact_removed, character.getID().getValue(), body_id2, CharacterID.invalid_character_id, sub_shape_id2));
        }

        pub fn onCharacterContactValidate(self: *SimCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact) bool {
            return self.validate(.character_vs_character_contact_validate, character, contact);
        }

        pub fn onCharacterContactAdded(self: *SimCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
            self.settingsCallback(.character_vs_character_contact_added, character, contact, settings);
        }

        pub fn onCharacterContactPersisted(self: *SimCharacterContactListener, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
            self.settingsCallback(.character_vs_character_contact_persisted, character, contact, settings);
        }

        pub fn onCharacterContactRemoved(self: *SimCharacterContactListener, character: *const CharacterVirtual, other_character_id: CharacterID, sub_shape_id2: SubShapeID) void {
            self.recorder.add(CharacterRecords.removed(.character_vs_character_contact_removed, character.getID().getValue(), .invalid, other_character_id.getValue(), sub_shape_id2));
        }

        pub fn onContactSolve(self: *SimCharacterContactListener, character: *const CharacterVirtual, body_id2: BodyID, sub_shape_id2: SubShapeID, contact_position: RVec3, contact_normal: Vec3, contact_velocity: Vec3, contact_material: *const PhysicsMaterial, character_velocity: Vec3, new_character_velocity: *Vec3) void {
            self.solve(.character_contact_solve, character, body_id2, CharacterID.invalid_character_id, sub_shape_id2, contact_position, contact_normal, contact_velocity, contact_material, character_velocity, new_character_velocity);
        }

        pub fn onCharacterContactSolve(self: *SimCharacterContactListener, character: *const CharacterVirtual, other_character: *const CharacterVirtual, sub_shape_id2: SubShapeID, contact_position: RVec3, contact_normal: Vec3, contact_velocity: Vec3, contact_material: *const PhysicsMaterial, character_velocity: Vec3, new_character_velocity: *Vec3) void {
            self.solve(.character_vs_character_contact_solve, character, .invalid, other_character.getID().getValue(), sub_shape_id2, contact_position, contact_normal, contact_velocity, contact_material, character_velocity, new_character_velocity);
        }

        /// A contact record (ContactRecord of the .cpp)
        fn contactRecord(kind: RecordKind, character: *const CharacterVirtual, contact: *const CharacterContact, settings_in: u32, result: u32) SimRecord {
            const flag_bits = CharacterRecords.contactFlags(@intFromEnum(contact.motion_type_b), contact.is_sensor_b, contact.had_collision, contact.was_discarded, contact.can_push_character, contact.is_back_facing_contact, contact.character_b != null);
            const p = contact.position;
            const v = contact.linear_velocity;
            const n = contact.surface_normal;
            const hash = SimScenes.hashCharacterContact(.{ p.getX(), p.getY(), p.getZ() }, .{ v.getX(), v.getY(), v.getZ() }, .{ n.getX(), n.getY(), n.getZ() }, contact.user_data, materialHash(contact.material));
            return CharacterRecords.contact(kind, character.getID().getValue(), contact.base.body_b, contact.base.character_id_b.getValue(), contact.base.sub_shape_id_b, contact.contact_normal, contact.distance, contact.fraction, flag_bits, settings_in, hash, result);
        }

        fn validate(self: *SimCharacterContactListener, kind: RecordKind, character: *const CharacterVirtual, contact: *const CharacterContact) bool {
            const result = !(self.modify and (flags(contact.user_data) & CharacterFlags.reject) != 0);
            self.recorder.add(contactRecord(kind, character, contact, 0, @intFromBool(result)));
            return result;
        }

        fn settingsCallback(self: *SimCharacterContactListener, kind: RecordKind, character: *const CharacterVirtual, contact: *const CharacterContact, settings: *CharacterContactSettings) void {
            const settings_in = CharacterRecords.settingsBits(settings.can_push_character, settings.can_receive_impulses);
            if (self.modify) {
                const f = flags(contact.user_data);
                if ((f & CharacterFlags.cannot_push) != 0)
                    settings.can_push_character = false;
                if ((f & CharacterFlags.no_impulses) != 0)
                    settings.can_receive_impulses = false;
            }
            self.recorder.add(contactRecord(kind, character, contact, settings_in, CharacterRecords.settingsBits(settings.can_push_character, settings.can_receive_impulses)));
        }

        fn solve(self: *SimCharacterContactListener, kind: RecordKind, character: *const CharacterVirtual, body_id2: BodyID, character_id2: u32, sub_shape_id2: SubShapeID, contact_position: RVec3, contact_normal: Vec3, contact_velocity: Vec3, contact_material: *const PhysicsMaterial, character_velocity: Vec3, new_character_velocity: *Vec3) void {
            const new_velocity_in = new_character_velocity.*;
            if (self.modify and (flags(character.getUserData()) & CharacterFlags.slow_solve) != 0)
                new_character_velocity.* = new_character_velocity.mulScalar(0.5);
            self.recorder.add(CharacterRecords.solve(kind, character.getID().getValue(), body_id2, character_id2, sub_shape_id2, contact_position, contact_normal, contact_velocity, materialHash(contact_material), character_velocity, new_velocity_in, new_character_velocity.*));
        }
    };

    /// SimMaterialHash: the hash of the debug name ("null" for no material)
    fn materialHash(material: ?*const PhysicsMaterial) u32 {
        return SimScenes.materialHash(if (material) |m| m.getDebugName() else "null");
    }

    /// SimHasher of the .cpp (FNV-1a over the values in order)
    const Hasher = struct {
        hash: u32 = SimScenes.hash_seed,

        fn value(self: *Hasher, v: anytype) void {
            self.hash = SimScenes.hashBytes(self.hash, std.mem.asBytes(&v));
        }

        fn boolean(self: *Hasher, v: bool) void {
            self.value(@as(u32, @intFromBool(v)));
        }

        fn vec3(self: *Hasher, v: Vec3) void {
            self.value([3]f32{ v.getX(), v.getY(), v.getZ() });
        }

        fn plane(self: *Hasher, p: zolt.Plane) void {
            self.vec3(p.getNormal());
            self.value(p.getConstant());
        }
    };

    /// CharacterShapeFilter.no_spheres (SimNoSpheresShapeFilter): shape 2 must not be a sphere
    const NoSpheresShapeFilter = struct {
        pub const overrides = .{ .shouldCollide, .shouldCollidePair };

        base: zolt.ShapeFilter = .init(NoSpheresShapeFilter),

        pub fn shouldCollide(self: *const NoSpheresShapeFilter, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = .{ self, sub_shape_id_of_shape2 };
            return shape2.getSubType() != .sphere;
        }

        pub fn shouldCollidePair(self: *const NoSpheresShapeFilter, shape1: *const Shape, sub_shape_id_of_shape1: SubShapeID, shape2: *const Shape, sub_shape_id_of_shape2: SubShapeID) bool {
            _ = .{ self, shape1, sub_shape_id_of_shape1, sub_shape_id_of_shape2 };
            return shape2.getSubType() != .sphere;
        }
    };

    /// SimCharacterFilters: the filters of a character virtual query (CharacterQueryFilter, op.i[0..3] and op.body); must
    /// not move while a query uses it
    const Filters = struct {
        layer: u32,
        body_filter_kind: u32,
        shape_filter_kind: u32,
        all_broad_phase_layers: zolt.BroadPhaseLayerFilter = .{},
        all_object_layers: zolt.ObjectLayerFilter = .{},
        default_broad_phase_layer_filter: zolt.DefaultBroadPhaseLayerFilter,
        default_object_layer_filter: zolt.DefaultObjectLayerFilter,
        all_bodies: zolt.BodyFilter = .{},
        ignore_body: zolt.IgnoreSingleBodyFilter,
        ignore_inner_body: zolt.IgnoreSingleBodyFilter,
        no_spheres: NoSpheresShapeFilter = .{},

        fn init(world: *const ZoltWorld, op: *const SimOp, inner_body: BodyID) Filters {
            const layer: ObjectLayer = if (op.i[0] == invalid_handle) 0 else @intCast(op.i[0]);
            return .{
                .layer = op.i[0],
                .body_filter_kind = op.i[1],
                .shape_filter_kind = op.i[2],
                .default_broad_phase_layer_filter = .init(&world.object_vs_broad_phase_layer_filter.base, layer),
                .default_object_layer_filter = .init(&world.object_layer_pair_filter.base, layer),
                .ignore_body = .init(world.getHandle(op.body)),
                .ignore_inner_body = .init(inner_body),
            };
        }

        fn broadPhase(self: *const Filters) *const zolt.BroadPhaseLayerFilter {
            return if (self.layer == invalid_handle) &self.all_broad_phase_layers else &self.default_broad_phase_layer_filter.base;
        }

        fn objectLayers(self: *const Filters) *const zolt.ObjectLayerFilter {
            return if (self.layer == invalid_handle) &self.all_object_layers else &self.default_object_layer_filter.base;
        }

        fn bodies(self: *const Filters) *const zolt.BodyFilter {
            return switch (self.body_filter_kind) {
                1 => &self.ignore_body.base,
                2 => &self.ignore_inner_body.base,
                else => &self.all_bodies,
            };
        }

        /// null: the accept-all ShapeFilter
        fn shapes(self: *Filters) ?*zolt.ShapeFilter {
            return if (self.shape_filter_kind == 1) &self.no_spheres.base else null;
        }
    };

    /// SimCheckCollisionCollector: the hash of the hits in their order, with the ID of the character that
    /// CharacterVsCharacterCollisionSimple passes through SetUserData
    const CheckCollisionCollector = struct {
        pub const overrides = .{ .setUserData, .addHit };

        base: zolt.CollideShapeCollector = .init(CheckCollisionCollector),
        hash: Hasher = .{},
        num_hits: u32 = 0,
        other_character: ?*const CharacterVirtual = null,

        pub fn setUserData(self: *CheckCollisionCollector, user_data: u64) void {
            self.other_character = @ptrFromInt(user_data);
        }

        pub fn addHit(self: *CheckCollisionCollector, result: *const CollideShapeResult) void {
            self.hash.vec3(result.contact_point_on1);
            self.hash.vec3(result.contact_point_on2);
            self.hash.vec3(result.penetration_axis);
            self.hash.value(result.penetration_depth);
            self.hash.value(result.sub_shape_id1.getValue());
            self.hash.value(result.sub_shape_id2.getValue());
            self.hash.value(result.body_id2.getIndexAndSequenceNumber());
            self.hash.value(if (self.other_character) |c| c.getID().getValue() else CharacterID.invalid_character_id);
            self.num_hits += 1;
        }

        fn finish(self: *CheckCollisionCollector) u32 {
            self.hash.value(self.num_hits);
            return self.hash.hash;
        }
    };

    /// The shape of a shape index (GetShape of the .cpp, `invalid_handle`: null)
    fn getShape(world: *const ZoltWorld, index: u32) ?*const Shape {
        return if (index != invalid_handle) world.shapes.items[index].get().? else null;
    }

    /// GetShapeIndex of the .cpp (`invalid_handle` for a shape that is not a scene shape)
    fn getShapeIndex(world: *const ZoltWorld, shape: ?*const Shape) u32 {
        for (world.shapes.items, 0..) |s, i| {
            if (s.get() == shape) return @intCast(i);
        }
        return invalid_handle;
    }

    /// GetObject of the .cpp: the character of a handle (a scene that uses a destroyed or unknown handle is an error)
    fn getCharacter(world: *ZoltWorld, index: u32) !*CharacterVirtual {
        const characters = world.phase7.character_virtuals.characters.items;
        if (index >= characters.len or characters[index] == null) {
            std.debug.print("SimulationParity: operation on character virtual handle {d} that does not exist\n", .{index});
            return error.InvalidScene;
        }
        return characters[index].?;
    }

    /// GetCharacterListener of the .cpp
    fn getListener(world: *ZoltWorld, mode: u32) !?CharacterContactListener {
        const storage = &world.phase7.character_virtuals;
        storage.record_listener.recorder = &world.character_recorder;
        storage.modify_listener.recorder = &world.character_recorder;
        return switch (mode) {
            0 => null,
            1 => CharacterContactListener.init(&storage.record_listener),
            2 => CharacterContactListener.init(&storage.modify_listener),
            else => {
                std.debug.print("SimulationParity: unknown character listener mode {d}\n", .{mode});
                return error.InvalidScene;
            },
        };
    }

    /// The CharacterVsCharacterCollisionSimple of a collision group
    fn getGroup(world: *ZoltWorld, index: u32) *CharacterVsCharacterCollisionSimple {
        const storage = &world.phase7.character_virtuals;
        if (!storage.groups_initialized) {
            for (&storage.groups) |*g| g.* = .init(world.allocator);
            storage.groups_initialized = true;
        }
        return &storage.groups[index];
    }

    fn create(world: *ZoltWorld, op: *const SimOp) !u32 {
        const allocator = world.allocator;
        const d: *const SimCharacterVirtual = &world.desc.character_virtuals[op.index];
        var settings = CharacterVirtualSettings.init(allocator);
        defer settings.deinit();
        settings.base.up = load3(&d.base.up);
        settings.base.supporting_volume = zolt.Plane.init(load3(d.base.supporting_volume[0..3]), d.base.supporting_volume[3]);
        settings.base.max_slope_angle = d.base.max_slope_angle;
        settings.base.enhanced_internal_edge_removal = d.base.enhanced_internal_edge_removal != 0;
        settings.base.shape.set(getShape(world, d.base.shape));
        settings.id = CharacterID.init(d.id);
        settings.mass = d.mass;
        settings.max_strength = d.max_strength;
        settings.shape_offset = load3(&d.shape_offset);
        settings.back_face_mode = @enumFromInt(d.back_face_mode);
        settings.predictive_contact_distance = d.predictive_contact_distance;
        settings.max_collision_iterations = d.max_collision_iterations;
        settings.max_constraint_iterations = d.max_constraint_iterations;
        settings.min_time_remaining = d.min_time_remaining;
        settings.collision_tolerance = d.collision_tolerance;
        settings.character_padding = d.character_padding;
        settings.max_num_hits = d.max_num_hits;
        settings.hit_reduction_cos_max_angle = d.hit_reduction_cos_max_angle;
        settings.penetration_recovery_speed = d.penetration_recovery_speed;
        settings.inner_body_shape.set(getShape(world, d.inner_body_shape));
        settings.inner_body_id_override = BodyID.init(d.inner_body_id_override);
        settings.inner_body_layer = @intCast(d.inner_body_layer);

        const storage = &world.phase7.character_virtuals;
        try storage.characters.ensureUnusedCapacity(allocator, 1);
        const character = try CharacterVirtual.create(allocator, &settings, loadR3(&d.position), loadQuat(&d.rotation), d.user_data, &world.system);
        character.base.addRef();
        storage.characters.appendAssumeCapacity(character);
        character.setListener(try getListener(world, d.listener));
        if (d.collision_group != invalid_handle) {
            const group = getGroup(world, d.collision_group);
            try group.add(character);
            character.setCharacterVsCharacterCollision(group.characterVsCharacterCollision());
        }
        return character.getInnerBodyID().getIndexAndSequenceNumber();
    }

    fn applyOp(world: *ZoltWorld, op: *const SimOp) !u32 {
        const i = &op.i;
        const f = &op.f;
        const r = loadR3(&op.r);
        const temp = world.temp.tempAllocator();

        const c = try getCharacter(world, op.index);
        var filters = Filters.init(world, op, c.getInnerBodyID());
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .cv_destroy => {
                const storage = &world.phase7.character_virtuals;
                if (storage.groups_initialized) {
                    for (&storage.groups) |*g| g.remove(c);
                }
                storage.characters.items[op.index] = null;
                c.base.release();
                return 0;
            },
            .cv_set_linear_velocity => c.setLinearVelocity(load3(f[0..3])),
            .cv_set_position => c.setPosition(r),
            .cv_set_rotation => c.setRotation(loadQuat(f[0..4])),
            .cv_set_up => c.base.setUp(load3(f[0..3])),
            .cv_set_max_slope_angle => c.base.setMaxSlopeAngle(f[0]),
            .cv_set_supporting_volume => c.base.setSupportingVolume(zolt.Plane.init(load3(f[0..3]), f[3])),
            .cv_set_mass => c.setMass(f[0]),
            .cv_set_max_strength => c.setMaxStrength(f[0]),
            .cv_set_penetration_recovery_speed => c.setPenetrationRecoverySpeed(f[0]),
            .cv_set_enhanced_internal_edge_removal => c.setEnhancedInternalEdgeRemoval(i[0] != 0),
            .cv_set_max_num_hits => c.setMaxNumHits(i[0]),
            .cv_set_hit_reduction_cos_max_angle => c.setHitReductionCosMaxAngle(f[0]),
            .cv_set_shape_offset => c.setShapeOffset(load3(f[0..3])),
            .cv_set_user_data => c.setUserData(@as(u64, i[0]) | (@as(u64, i[1]) << 32)),
            .cv_set_listener => c.setListener(try getListener(world, i[0])),
            .cv_set_character_vs_character_collision => c.setCharacterVsCharacterCollision(if (i[0] != invalid_handle) getGroup(world, i[0]).characterVsCharacterCollision() else null),
            .cv_add_to_collision_group => try getGroup(world, i[0]).add(c),
            .cv_remove_from_collision_group => getGroup(world, i[0]).remove(c),
            .cv_update => try c.update(f[0], load3(f[1..4]), filters.broadPhase(), filters.objectLayers(), filters.bodies(), filters.shapes(), temp),
            .cv_extended_update => {
                // SimLoadExtendedUpdateSettings(f + 4)
                const s: CharacterVirtual.ExtendedUpdateSettings = .{
                    .stick_to_floor_step_down = load3(f[4..7]),
                    .walk_stairs_step_up = load3(f[7..10]),
                    .walk_stairs_min_step_forward = f[10],
                    .walk_stairs_step_forward_test = f[11],
                    .walk_stairs_cos_angle_forward_contact = f[12],
                    .walk_stairs_step_down_extra = load3(f[13..16]),
                };
                try c.extendedUpdate(f[0], load3(f[1..4]), &s, filters.broadPhase(), filters.objectLayers(), filters.bodies(), filters.shapes(), temp);
            },
            .cv_stick_to_floor => return @intFromBool(try c.stickToFloor(load3(f[0..3]), filters.broadPhase(), filters.objectLayers(), filters.bodies(), filters.shapes(), temp)),
            .cv_walk_stairs => return @intFromBool(try c.walkStairs(f[0], load3(f[1..4]), load3(f[4..7]), load3(f[7..10]), load3(f[10..13]), filters.broadPhase(), filters.objectLayers(), filters.bodies(), filters.shapes(), temp)),
            .cv_can_walk_stairs => return @intFromBool(c.canWalkStairs(load3(f[0..3]))),
            .cv_refresh_contacts => try c.refreshContacts(filters.broadPhase(), filters.objectLayers(), filters.bodies(), filters.shapes(), temp),
            .cv_update_ground_velocity => c.updateGroundVelocity(),
            .cv_set_shape => return @intFromBool(try c.setShape(getShape(world, i[3]), f[0], filters.broadPhase(), filters.objectLayers(), filters.bodies(), filters.shapes(), temp)),
            .cv_set_inner_body_shape => try c.setInnerBodyShape(getShape(world, i[0]).?),
            .cv_cancel_velocity_towards_steep_slopes => {
                var h: Hasher = .{};
                h.vec3(c.cancelVelocityTowardsSteepSlopes(load3(f[0..3])));
                return h.hash;
            },
            .cv_start_tracking_contact_changes => try c.startTrackingContactChanges(),
            .cv_finish_tracking_contact_changes => c.finishTrackingContactChanges(),
            .cv_check_collision => {
                var collector: CheckCollisionCollector = .{};
                try c.checkCollision(r, loadQuat(f[0..4]), load3(f[4..7]), f[7], getShape(world, i[3]).?, r, &collector.base, filters.broadPhase(), filters.objectLayers(), filters.bodies(), filters.shapes());
                return collector.finish();
            },
            .cv_settings_hash => {
                var s = c.getCharacterVirtualSettings();
                defer s.deinit();
                var h: Hasher = .{};
                h.vec3(s.base.up);
                h.plane(s.base.supporting_volume);
                h.value(s.base.max_slope_angle);
                h.boolean(s.base.enhanced_internal_edge_removal);
                h.value(getShapeIndex(world, s.base.shape.get()));
                h.value(s.id.getValue());
                h.value(s.mass);
                h.value(s.max_strength);
                h.vec3(s.shape_offset);
                h.value(@as(u32, @intFromEnum(s.back_face_mode)));
                h.value(s.predictive_contact_distance);
                h.value(s.max_collision_iterations);
                h.value(s.max_constraint_iterations);
                h.value(s.min_time_remaining);
                h.value(s.collision_tolerance);
                h.value(s.character_padding);
                h.value(s.max_num_hits);
                h.value(s.hit_reduction_cos_max_angle);
                h.value(s.penetration_recovery_speed);
                h.value(getShapeIndex(world, s.inner_body_shape.get()));
                h.value(s.inner_body_id_override.getIndexAndSequenceNumber());
                h.value(@as(u32, s.inner_body_layer));
                return h.hash;
            },
            .cv_has_collided_with_body => return @intFromBool(c.hasCollidedWith(world.getHandle(op.body))),
            .cv_has_collided_with_character => {
                const other = try getCharacter(world, i[0]);
                return @as(u32, @intFromBool(c.hasCollidedWithCharacterID(other.getID()))) | (@as(u32, @intFromBool(c.hasCollidedWithCharacter(other))) << 1);
            },
            .cv_expect_ground_state => return @bitCast(@intFromEnum(c.base.getGroundState())), // Changes nothing, the driver checks the result
            .cv_save_restore_state => {
                var recorder = StateRecorderImpl.init(world.allocator);
                defer recorder.deinit();
                c.saveState(recorder.recorder());
                if (recorder.isFailed()) return error.OutOfMemory;
                const hash = SimScenes.hashBytes(SimScenes.hash_seed, recorder.getData());
                recorder.rewind();
                recorder.recorder().setValidating(i[0] != 0);
                try c.restoreState(recorder.recorder());
                return hash;
            },
            .cv_expect_value => return switch (i[0]) { // Changes nothing, the driver checks the result
                0 => @intFromBool(c.getMaxHitsExceeded()),
                1 => @intCast(c.getActiveContacts().len),
                2, 3 => blk: {
                    var n: u32 = 0;
                    for (c.getActiveContacts()) |*contact| {
                        if (contact.had_collision and (i[0] == 2 or contact.character_b != null))
                            n += 1;
                    }
                    break :blk n;
                },
                4 => @intFromBool(c.base.getGroundBodyID().eql(world.getHandle(op.body))),
                5 => @intFromBool(!c.base.getGroundVelocity().isNearZero(.{ .max_dist_sq = 1.0e-6 })),
                6 => getShapeIndex(world, c.getShape()),
                7 => @intFromBool(!c.getInnerBodyID().isInvalid()),
                else => {
                    std.debug.print("SimulationParity: unknown character virtual value {d}\n", .{i[0]});
                    return error.InvalidScene;
                },
            },
            .cv_set_step_velocity => {
                // Character::Step of CharacterVirtualTests.cpp
                const current_vertical_velocity = Vec3.init(0, c.getLinearVelocity().getY(), 0);
                const ground_velocity = c.base.getGroundVelocity();
                var new_velocity: Vec3 = undefined;
                if (c.base.getGroundState() == .on_ground and // If on ground
                    (current_vertical_velocity.getY() - ground_velocity.getY()) < 0.1) // And not moving away from ground
                {
                    // Assume velocity of ground when on ground
                    new_velocity = ground_velocity;

                    // Jump
                    new_velocity = new_velocity.add(Vec3.init(0, f[7], 0));
                } else new_velocity = current_vertical_velocity;

                // Gravity
                new_velocity = new_velocity.add(load3(f[3..6]).mulScalar(f[6]));

                // Player input
                new_velocity = new_velocity.add(load3(f[0..3]));

                c.setLinearVelocity(new_velocity);
            },
            .cv_perf_update => {
                // The step of a character in CharacterVirtualScene::UpdateTest of PerformanceTest

                // Calculate new vertical velocity
                var new_velocity: Vec3 = undefined;
                if (c.base.getGroundState() == .on_ground and // If on ground
                    c.getLinearVelocity().getY() < 0.1) // And not moving away from ground
                    new_velocity = Vec3.zero()
                else
                    new_velocity = c.getLinearVelocity().mul(Vec3.init(0, 1, 0));
                new_velocity = new_velocity.add(world.system.getGravity().mulScalar(f[0]));

                // Deterministic random input
                const count: u64 = op.count;
                const hash = zolt.HashCombine.hash(count);
                const x: i32 = @intCast(hash % 10);
                const y: i32 = @intCast((hash / 10) % 10);
                const speed: i32 = @intCast((hash / 100) % 10);

                // Determine target position
                const target = RVec3.init(4.0 * @as(Real, @floatFromInt(x)) - 20.0, 5.0, 4.0 * @as(Real, @floatFromInt(y)) - 20.0);

                // Determine new character velocity
                var direction = target.sub(c.getPosition()).toVec3().normalizedOr(Vec3.zero());
                direction.setY(0);
                new_velocity = new_velocity.add(direction.mulScalar(5.0 + 0.5 * @as(f32, @floatFromInt(speed))));
                c.setLinearVelocity(new_velocity);

                // Update the character position
                const update_settings: CharacterVirtual.ExtendedUpdateSettings = .{};
                try c.extendedUpdate(f[0], world.system.getGravity(), &update_settings, filters.broadPhase(), filters.objectLayers(), filters.bodies(), filters.shapes(), temp);
            },
            else => |kind| {
                std.debug.print("SimulationParity: unknown character virtual operation {t}\n", .{kind});
                return error.InvalidScene;
            },
        }
        return 0;
    }

    fn writeState(world: *ZoltWorld, w: *ObjectStateWriter) !void {
        for (world.phase7.character_virtuals.characters.items, 0..) |maybe_character, index| {
            const c = maybe_character orelse continue;
            const h: u32 = @intCast(index);

            // The SaveState bytes (WriteSaveState)
            var recorder = StateRecorderImpl.init(world.allocator);
            defer recorder.deinit();
            c.saveState(recorder.recorder());
            if (recorder.isFailed()) return error.OutOfMemory;
            try w.begin(.cv_save_state, h, 0);
            try w.raw(recorder.getData());
            w.end();

            try w.begin(.cv_properties, h, 0);
            try w.value(c.getID().getValue());
            try w.value(c.getUserData());
            try w.value(c.getInnerBodyID().getIndexAndSequenceNumber());
            try w.value(getShapeIndex(world, c.getShape()));
            try w.value(c.getMass());
            try w.value(c.getMaxStrength());
            try w.value(c.getPenetrationRecoverySpeed());
            try w.boolean(c.getEnhancedInternalEdgeRemoval());
            try w.value(c.getCharacterPadding());
            try w.value(c.getMaxNumHits());
            try w.value(c.getHitReductionCosMaxAngle());
            try w.boolean(c.getMaxHitsExceeded());
            try w.vec3(c.getShapeOffset());
            w.end();

            try w.begin(.cv_base, h, 0);
            try w.vec3(c.base.getUp());
            try w.value(c.base.getCosMaxSlopeAngle());
            const supporting_volume = c.base.getSupportingVolume();
            try w.vec3(supporting_volume.getNormal());
            try w.value(supporting_volume.getConstant());
            try w.value(materialHash(c.base.getGroundMaterial()));
            try w.value(c.base.getGroundUserData());
            try w.boolean(c.base.isSupported());
            try w.boolean(c.canWalkStairs(c.getLinearVelocity()));
            w.end();

            try w.begin(.cv_transforms, h, 0);
            try w.rvec3(c.getCenterOfMassPosition());
            try w.rmat44(c.getWorldTransform());
            try w.rmat44(c.getCenterOfMassTransform());
            w.end();

            // WriteTransformedShape
            var ts = c.getTransformedShape();
            defer ts.deinit();
            try w.begin(.cv_transformed_shape, h, 0);
            try w.rvec3(ts.shape_position_com);
            try w.quat(ts.shape_rotation);
            try w.vec3(ts.getShapeScale());
            try w.value(ts.body_id.getIndexAndSequenceNumber());
            try w.value(getShapeIndex(world, ts.shape.get()));
            w.end();

            const contacts = c.getActiveContacts();
            try w.begin(.cv_num_active_contacts, h, 0);
            try w.value(@as(u32, @intCast(contacts.len)));
            w.end();
            for (contacts, 0..) |*contact, item| {
                try w.begin(.cv_active_contact, h, @intCast(item));
                try w.value(contact.base.body_b.getIndexAndSequenceNumber());
                try w.value(contact.base.character_id_b.getValue());
                try w.value(contact.base.sub_shape_id_b.getValue());
                try w.rvec3(contact.position);
                try w.vec3(contact.linear_velocity);
                try w.vec3(contact.contact_normal);
                try w.vec3(contact.surface_normal);
                try w.value(contact.distance);
                try w.value(contact.fraction);
                try w.value(@as(u32, @intFromEnum(contact.motion_type_b)));
                try w.boolean(contact.is_sensor_b);
                try w.boolean(contact.character_b != null);
                try w.value(contact.user_data);
                try w.value(materialHash(contact.material));
                try w.boolean(contact.had_collision);
                try w.boolean(contact.was_discarded);
                try w.boolean(contact.can_push_character);
                try w.boolean(contact.is_back_facing_contact);
                w.end();
            }
        }
    }

    fn saveStates(world: *ZoltWorld, recorder: *StateRecorderImpl) !void {
        for (world.phase7.character_virtuals.characters.items) |maybe_character| {
            if (maybe_character) |c| c.saveState(recorder.recorder());
        }
    }

    fn restoreStates(world: *ZoltWorld, recorder: *StateRecorderImpl) !void {
        for (world.phase7.character_virtuals.characters.items) |maybe_character| {
            if (maybe_character) |c| try c.restoreState(recorder.recorder());
        }
    }

    /// ~SimWorld: the group lists are cleared, then the character virtuals released in handle order (each removes and
    /// destroys its inner body)
    fn destroyAll(world: *ZoltWorld) void {
        const storage = &world.phase7.character_virtuals;
        if (storage.groups_initialized) {
            for (&storage.groups) |*g| g.deinit();
            storage.groups_initialized = false;
        }
        for (storage.characters.items) |maybe_character| {
            if (maybe_character) |c| c.base.release();
        }
        storage.characters.deinit(world.allocator);
        storage.characters = .empty;
    }

    /// The CharacterID test (jolt_sim_character_id_sequence of the .cpp): which constructions consume an ID
    fn characterIdSequence(world: *ZoltWorld, first_id: u32, out: *[SimScenes.character_id_sequence_len]u32) !void {
        const allocator = world.allocator;
        const next = struct {
            fn next() u32 {
                const value = CharacterID.nextCharacterID().getValue();
                CharacterID.setNextCharacterID(.{ .next_value = value });
                return value;
            }
        }.next;
        CharacterID.setNextCharacterID(.{ .next_value = first_id });
        defer CharacterID.setNextCharacterID(.{});
        var a = CharacterVirtualSettings.init(allocator);
        defer a.deinit();
        a.base.shape.set(getShape(world, 0));
        out[0] = a.id.getValue();
        out[1] = next();
        var b = CharacterVirtualSettings.initCopy(allocator, &a);
        defer b.deinit();
        out[2] = b.id.getValue();
        out[3] = next();
        var c = CharacterVirtualSettings.init(allocator);
        defer c.deinit();
        c.assign(&a);
        out[4] = c.id.getValue();
        out[5] = next();
        {
            const character = try CharacterVirtual.create(allocator, &a, RVec3.zero(), Quat.identity(), 0, &world.system);
            character.base.addRef();
            defer character.base.release();
            out[6] = character.getID().getValue();
            out[7] = next();
            var s = character.getCharacterVirtualSettings();
            defer s.deinit();
            out[8] = s.id.getValue();
            out[9] = next();
        }
    }
};

// ---- SLOT ragdoll: Ragdoll, RagdollSettings (Phase7OpRange.ragdoll, Blocker.ragdolls) ----
const RagdollSlot = struct {
    const Ragdoll = zolt.Ragdoll;
    const RagdollSettings = zolt.RagdollSettings;
    const Skeleton = zolt.Skeleton;
    const SkeletonPose = zolt.SkeletonPose;
    const TwoBodyConstraintSettings = zolt.TwoBodyConstraintSettings;

    /// mRagdollSettings and mRagdolls of SimWorld
    const Storage = struct {
        /// Per ragdoll description, built at its first create_ragdoll (.empty: not built yet)
        settings: std.ArrayList(Ref(RagdollSettings)) = .empty,
        /// The ragdolls of the ragdoll handles (.empty: destroyed or out of bodies)
        ragdolls: std.ArrayList(Ref(Ragdoll)) = .empty,
    };

    /// The TwoBodyConstraintSettings in the settings of a constraint description (every ConstraintKind is a two body
    /// constraint)
    fn twoBody(settings: Ref(ConstraintSettings)) ?*TwoBodyConstraintSettings {
        return zolt.virtual.downcast(TwoBodyConstraintSettings, settings.get().?);
    }

    /// SimMakePose of SimulationReference.cpp: the SkeletonPose of a pose description (the joint states, then
    /// CalculateJointMatrices)
    fn makePose(desc: *const SimSceneDesc, skeleton: *const Skeleton, pose_index: u32, out_pose: *SkeletonPose) !void {
        const p = &desc.ragdoll_poses[pose_index];
        try out_pose.setSkeleton(skeleton);
        out_pose.setRootOffset(loadR3(&p.root_offset));
        var j: usize = 0;
        while (j < p.count and j < out_pose.getJointCount()) : (j += 1) {
            const f = desc.floats[p.data + 7 * j ..][0..7];
            const state = out_pose.getJointMut(j);
            state.translation = load3(f[0..3]);
            state.rotation = loadQuat(f[3..7]);
        }
        out_pose.calculateJointMatrices();
    }

    /// SimMakePoseMatrices of SimulationReference.cpp: the joint matrices of a pose description taken as model space
    /// transforms (the `_matrices` operations; the caller frees them)
    fn makePoseMatrices(allocator: Allocator, desc: *const SimSceneDesc, pose_index: u32, num_joints: usize) ![]Mat44 {
        const p = &desc.ragdoll_poses[pose_index];
        const matrices = try allocator.alloc(Mat44, num_joints);
        @memset(matrices, Mat44.identity());
        var j: usize = 0;
        while (j < p.count and j < num_joints) : (j += 1) {
            const f = desc.floats[p.data + 7 * j ..][0..7];
            matrices[j] = Mat44.rotationTranslation(loadQuat(f[3..7]), load3(f[0..3]));
        }
        return matrices;
    }

    /// SimWorld::MakeRagdollSettings / SimMakeRagdollSettings of SimulationReference.cpp: the RagdollSettings of ragdoll
    /// description `index` (one reference owned by the caller); `out_stabilize`: the result of Stabilize (1 / 0, 2: not
    /// called)
    fn makeSettings(world: *const ZoltWorld, index: u32, out_stabilize: ?*u32) !Ref(RagdollSettings) {
        const allocator = world.allocator;
        const desc = world.desc;
        const rd = &desc.ragdolls[index];
        const joints = desc.ragdoll_joints[rd.first_joint..][0..rd.num_joints];

        var settings = Ref(RagdollSettings).init(try RagdollSettings.create(allocator));
        errdefer settings.deinit();
        const s = settings.get().?;
        s.skeleton = .init(try Skeleton.create(allocator));
        const skeleton = s.skeleton.get().?;
        for (joints, 0..) |*joint, j| {
            var name_buffer: [32]u8 = undefined;
            const name = try std.fmt.bufPrint(&name_buffer, "Joint{d}", .{j});
            if (joint.parent < @as(i32, @intCast(j))) {
                _ = try skeleton.addJointWithParentIndex(name, joint.parent);
            } else {
                // A joint that is its own parent: the parent index is set directly (as the .cpp does)
                _ = try skeleton.addJoint(name, .{});
                skeleton.getJointMut(j).parent_joint_index = joint.parent;
            }
        }

        try s.parts.ensureTotalCapacity(allocator, joints.len);
        for (joints) |*joint| {
            var part_settings = world.makeBodyCreationSettings(&desc.bodies[joint.body]);
            defer part_settings.deinit();
            var to_parent: Ref(ConstraintSettings) = .empty;
            defer to_parent.deinit();
            if (joint.to_parent != invalid_handle)
                to_parent = try makeConstraintSettings(allocator, &desc.constraints[joint.to_parent]);
            s.parts.appendAssumeCapacity(.init(&part_settings, if (to_parent.get() != null) twoBody(to_parent) else null));
        }
        try s.additional_constraints.ensureTotalCapacity(allocator, rd.num_additional);
        for (desc.ragdoll_additional_constraints[rd.first_additional..][0..rd.num_additional]) |*c| {
            var constraint = try makeConstraintSettings(allocator, &desc.constraints[c.constraint]);
            defer constraint.deinit();
            s.additional_constraints.appendAssumeCapacity(.init(c.body1, c.body2, twoBody(constraint)));
        }

        // The preparation, in the order of the members of SimRagdoll
        var stabilize: u32 = 2;
        if (rd.stabilize != 0)
            stabilize = @intFromBool(try s.stabilize());
        if (out_stabilize) |out| out.* = stabilize;
        if (rd.calculate_constraint_priorities != 0)
            try s.calculateConstraintPriorities(.{ .base_priority = rd.base_priority });
        if (rd.disable_parent_child_collisions == 1) {
            try s.disableParentChildCollisions(.{});
        } else if (rd.disable_parent_child_collisions == 2) {
            var pose = SkeletonPose.init(allocator);
            defer pose.deinit();
            try makePose(desc, skeleton, rd.pose, &pose);
            try s.disableParentChildCollisions(.{ .joint_matrices = pose.getJointMatrices(), .min_separation_distance = rd.min_separation_distance });
        }
        try s.calculateBodyIndexToConstraintIndex();
        try s.calculateConstraintIndexToBodyIdxPair();
        return settings;
    }

    /// GetObject of SimulationReference.cpp (which aborts on a handle without ragdoll)
    fn ragdoll(world: *ZoltWorld, op: *const SimOp) !*Ragdoll {
        const ragdolls = world.phase7.ragdolls.ragdolls.items;
        if (op.index < ragdolls.len) {
            if (ragdolls[op.index].get()) |r| return r;
        }
        std.debug.print("SimulationParity: operation {d} on ragdoll handle {d} that does not exist\n", .{ op.kind, op.index });
        return error.SceneError;
    }

    /// The hash of the SaveBinaryState bytes of settings (ragdoll_settings_hash)
    fn settingsHash(allocator: Allocator, settings: *const RagdollSettings, save_shapes: bool, save_group_filter: bool) !u32 {
        var out: std.Io.Writer.Allocating = .init(allocator);
        defer out.deinit();
        var wrapper = zolt.StreamOutWrapper.init(&out.writer);
        try settings.saveBinaryState(wrapper.streamOut(), save_shapes, save_group_filter);
        if (wrapper.isFailed()) return error.OutOfMemory;
        return SimScenes.hashBytes(2166136261, out.written());
    }

    /// The create_ragdoll operation (ApplyRagdollOp of SimulationReference.cpp)
    fn create(world: *ZoltWorld, op: *const SimOp) !u32 {
        const allocator = world.allocator;
        const storage = &world.phase7.ragdolls;

        // The settings of the description, built at its first creation and shared by its ragdolls
        const d = op.index;
        if (storage.settings.items.len < world.desc.num_ragdolls)
            try storage.settings.appendNTimes(allocator, .empty, world.desc.num_ragdolls - storage.settings.items.len);
        try storage.ragdolls.ensureUnusedCapacity(allocator, 1);
        if (storage.settings.items[d].get() == null)
            storage.settings.items[d] = try makeSettings(world, d, null);
        const rd = &world.desc.ragdolls[d];
        const new_ragdoll = try storage.settings.items[d].get().?.createRagdoll(rd.collision_group, rd.user_data, &world.system);
        storage.ragdolls.appendAssumeCapacity(.init(new_ragdoll));
        return if (new_ragdoll) |r| @intCast(r.getBodyCount()) else 0;
    }

    /// ApplyRagdollOp of SimulationReference.cpp: the operations return IsActive() unless noted
    fn applyOp(world: *ZoltWorld, op: *const SimOp) !u32 {
        const allocator = world.allocator;
        const desc = world.desc;
        const i = &op.i;
        const f = &op.f;
        const activation: Activation = if (op.activation == 0) .activate else .dont_activate;
        const lock = i[3] == 0;

        const rag = try ragdoll(world, op);
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .ragdoll_add_to_physics_system => try rag.addToPhysicsSystem(activation, .{ .lock_bodies = lock }),
            .ragdoll_remove_from_physics_system => try rag.removeFromPhysicsSystem(.{ .lock_bodies = lock }),
            .ragdoll_activate => rag.activate(.{ .lock_bodies = lock }),
            .ragdoll_is_active => return @intFromBool(rag.isActive(.{ .lock_bodies = lock })),
            .ragdoll_set_group_id => rag.setGroupID(i[0], .{ .lock_bodies = lock }),

            .ragdoll_set_pose => {
                var pose = SkeletonPose.init(allocator);
                defer pose.deinit();
                try makePose(desc, rag.getRagdollSettings().?.getSkeleton().?, i[0], &pose);
                rag.setPose(&pose, .{ .lock_bodies = lock });
            },

            .ragdoll_set_pose_matrices => {
                const matrices = try makePoseMatrices(allocator, desc, i[0], rag.getBodyCount());
                defer allocator.free(matrices);
                rag.setPoseMatrices(loadR3(&desc.ragdoll_poses[i[0]].root_offset), matrices, .{ .lock_bodies = lock });
            },

            .ragdoll_reset_warm_start => rag.resetWarmStart(),

            .ragdoll_drive_to_pose_using_kinematics => {
                var pose = SkeletonPose.init(allocator);
                defer pose.deinit();
                try makePose(desc, rag.getRagdollSettings().?.getSkeleton().?, i[0], &pose);
                rag.driveToPoseUsingKinematics(&pose, f[0], .{ .lock_bodies = lock });
            },

            .ragdoll_drive_to_pose_using_kinematics_matrices => {
                const matrices = try makePoseMatrices(allocator, desc, i[0], rag.getBodyCount());
                defer allocator.free(matrices);
                rag.driveToPoseUsingKinematicsMatrices(loadR3(&desc.ragdoll_poses[i[0]].root_offset), matrices, f[0], .{ .lock_bodies = lock });
            },

            .ragdoll_drive_to_pose_using_motors => {
                var pose = SkeletonPose.init(allocator);
                defer pose.deinit();
                try makePose(desc, rag.getRagdollSettings().?.getSkeleton().?, i[0], &pose);
                rag.driveToPoseUsingMotors(&pose);
            },

            .ragdoll_drive_to_pose_using_motors_velocity => {
                const skeleton = rag.getRagdollSettings().?.getSkeleton().?;
                var prev_pose = SkeletonPose.init(allocator);
                defer prev_pose.deinit();
                var pose = SkeletonPose.init(allocator);
                defer pose.deinit();
                try makePose(desc, skeleton, i[1], &prev_pose);
                try makePose(desc, skeleton, i[0], &pose);
                rag.driveToPoseUsingMotorsWithVelocity(&prev_pose, &pose, f[0]);
            },

            .ragdoll_set_linear_and_angular_velocity => rag.setLinearAndAngularVelocity(load3(f[0..3]), load3(f[3..6]), .{ .lock_bodies = lock }),
            .ragdoll_set_linear_velocity => rag.setLinearVelocity(load3(f[0..3]), .{ .lock_bodies = lock }),
            .ragdoll_add_linear_velocity => rag.addLinearVelocity(load3(f[0..3]), .{ .lock_bodies = lock }),
            .ragdoll_add_impulse => rag.addImpulse(load3(f[0..3]), .{ .lock_bodies = lock }),

            .ragdoll_destroy => {
                world.phase7.ragdolls.ragdolls.items[op.index].deinit();
                return 0;
            },

            .ragdoll_settings_hash => return try settingsHash(allocator, rag.getRagdollSettings().?, i[0] != 0, i[1] != 0),

            .ragdoll_get_pose_hash => {
                var pose = SkeletonPose.init(allocator);
                defer pose.deinit();
                try pose.setSkeleton(rag.getRagdollSettings().?.getSkeleton().?);
                rag.getPose(&pose, .{ .lock_bodies = lock });
                const root_offset = pose.getRootOffset();
                const offset = [3]Real{ root_offset.getX(), root_offset.getY(), root_offset.getZ() };
                var hash = SimScenes.hashBytes(2166136261, std.mem.asBytes(&offset));
                for (pose.getJointMatrices()) |m| {
                    for (0..4) |col| {
                        const v = m.getColumn4(@intCast(col));
                        const values = [4]f32{ v.getX(), v.getY(), v.getZ(), v.getW() };
                        hash = SimScenes.hashBytes(hash, std.mem.asBytes(&values));
                    }
                }
                return hash;
            },

            .ragdoll_expect_active => {}, // Changes nothing, the driver checks the result

            else => {
                std.debug.print("SimulationParity: unknown ragdoll operation {d}\n", .{op.kind});
                return error.SceneError;
            },
        }
        return @intFromBool(rag.isActive(.{}));
    }

    /// WriteRagdollState of SimulationReference.cpp for every ragdoll in handle order (the destroyed ones skipped)
    fn writeState(world: *ZoltWorld, w: *ObjectStateWriter) !void {
        const allocator = world.allocator;
        for (world.phase7.ragdolls.ragdolls.items, 0..) |r_ref, handle_index| {
            const r = r_ref.get() orelse continue;
            const handle: u32 = @intCast(handle_index);
            const num_bodies = r.getBodyCount();
            try w.begin(.ragdoll_properties, handle, 0);
            try w.value(@as(u32, @intCast(num_bodies)));
            try w.value(@as(u32, @intCast(r.getConstraintCount())));
            try w.boolean(r.isActive(.{}));
            w.end();

            for (0..num_bodies) |b| {
                try w.begin(.ragdoll_body, handle, @intCast(b));
                try w.value(r.getBodyID(b).getIndexAndSequenceNumber());
                w.end();
            }

            const root = r.getRootTransform(.{});
            try w.begin(.ragdoll_root_transform, handle, 0);
            try w.rvec3(root.position);
            try w.quat(root.rotation);
            w.end();

            var root_offset: RVec3 = undefined;
            const joints = try allocator.alloc(Mat44, num_bodies);
            defer allocator.free(joints);
            r.getPoseMatrices(&root_offset, joints, .{});
            try w.begin(.ragdoll_pose_root_offset, handle, 0);
            try w.rvec3(root_offset);
            w.end();
            for (joints, 0..) |m, b| {
                try w.begin(.ragdoll_pose_joint, handle, @intCast(b));
                try w.mat44(m);
                w.end();
            }

            const bounds = r.getWorldSpaceBounds(.{});
            try w.begin(.ragdoll_bounds, handle, 0);
            try w.vec3(bounds.min);
            try w.vec3(bounds.max);
            w.end();

            for (0..r.getConstraintCount()) |c| {
                const constraint = r.getConstraint(c);
                try w.begin(.ragdoll_constraint, handle, @intCast(c));
                try w.value(@as(u32, @intFromEnum(constraint.base.getSubType())));
                try w.value(constraint.base.getConstraintPriority());
                try w.boolean(constraint.base.getEnabled());
                try w.boolean(constraint.base.isActive());
                try w.value(constraint.getBody1().getID().getIndexAndSequenceNumber());
                try w.value(constraint.getBody2().getID().getIndexAndSequenceNumber());
                w.end();
            }
        }
    }

    /// Ragdolls have no SaveState of their own (their bodies and constraints are in the PhysicsSystem state)
    fn saveStates(world: *ZoltWorld, recorder: *StateRecorderImpl) !void {
        _ = .{ world, recorder };
    }

    fn restoreStates(world: *ZoltWorld, recorder: *StateRecorderImpl) !void {
        _ = .{ world, recorder };
    }

    /// ~SimWorld of SimulationReference.cpp: the ragdolls that are in the system are removed, then all are released
    /// (which destroys their bodies), then the settings. Nothing is compared afterwards, and nothing may allocate here:
    /// under the creation sweep (std.testing.checkAllAllocationFailures) an allocation of the teardown that fails would
    /// be swallowed (error.SwallowedOutOfMemoryError). So the constraints and the bodies are removed one by one
    /// (RemoveFromPhysicsSystem allocates the copy of the body IDs and the constraint list) and the activation listener
    /// is detached first (OnBodyDeactivated of a removed active body adds a record, which can allocate).
    fn destroyAll(world: *ZoltWorld) void {
        const allocator = world.allocator;
        const storage = &world.phase7.ragdolls;
        const bi = world.system.getBodyInterfaceMut();
        world.system.setBodyActivationListener(null);
        for (storage.ragdolls.items) |r_ref| {
            const r = r_ref.get() orelse continue;
            if (!bi.isAdded(r.getBodyID(0))) continue;
            for (r.constraints.items) |c| world.system.removeConstraint(c.get().?);
            for (r.body_ids.items) |id| bi.removeBody(id);
        }
        for (storage.ragdolls.items) |*r| r.deinit();
        storage.ragdolls.deinit(allocator);
        for (storage.settings.items) |*s| s.deinit();
        storage.settings.deinit(allocator);
        storage.* = .{};
    }

    /// jolt_sim_ragdoll_settings_bytes of SimulationReference.cpp
    fn settingsBytes(world: *ZoltWorld, index: u32, restore_from: ?[]const u8, out: *std.ArrayList(u8)) !bool {
        const allocator = world.allocator;
        var settings: Ref(RagdollSettings) = .empty;
        defer settings.deinit();
        if (restore_from) |bytes| {
            var reader: std.Io.Reader = .fixed(bytes);
            var wrapper = zolt.StreamInWrapper.init(&reader);
            const stream = wrapper.streamIn();
            var result = try RagdollSettings.restoreFromBinaryState(allocator, stream);
            defer result.deinit();
            if (result.hasError()) return false;
            // SimReadCompletely: the stream sets eof only on a read past the end, so one more byte is read
            const read = !stream.isFailed();
            var extra: [1]u8 = undefined;
            stream.readBytes(&extra);
            if (!read or !stream.isEOF()) return false;
            settings.set(result.getPtr().?);
        } else {
            settings = try makeSettings(world, index, null);
        }
        var bytes: std.Io.Writer.Allocating = .init(allocator);
        defer bytes.deinit();
        var wrapper = zolt.StreamOutWrapper.init(&bytes.writer);
        try settings.get().?.saveBinaryState(wrapper.streamOut(), true, true);
        if (wrapper.isFailed()) return error.OutOfMemory;
        out.clearRetainingCapacity();
        try out.appendSlice(allocator, bytes.written());
        return true;
    }

    /// jolt_sim_ragdoll_settings_info of SimulationReference.cpp
    fn settingsInfo(world: *ZoltWorld, index: u32, out: *std.ArrayList(u32)) !void {
        const allocator = world.allocator;
        var stabilize: u32 = 2;
        var settings = try makeSettings(world, index, &stabilize);
        defer settings.deinit();
        const s = settings.get().?;
        out.clearRetainingCapacity();
        try out.append(allocator, stabilize);
        const body_to_constraint = s.getBodyIndexToConstraintIndex();
        try out.append(allocator, @intCast(body_to_constraint.len));
        for (body_to_constraint) |c|
            try out.append(allocator, @bitCast(c));
        const pairs = s.getConstraintIndexToBodyIdxPair();
        try out.append(allocator, @intCast(pairs.len));
        for (pairs) |p| {
            try out.append(allocator, @bitCast(p.first));
            try out.append(allocator, @bitCast(p.second));
        }
    }
};

// ---- SLOT vehicle: VehicleConstraint, the collision testers, the wheels, the engine, the transmission, the anti-roll
// bars, WheeledVehicleController (Phase7OpRange.vehicle 520..569, Blocker.wheeled_vehicles) ----

/// The vehicle core group: the vehicles of all three controller kinds (the tracked and motorcycle parts through the hooks
/// of TrackedVehicleSlot / MotorcycleSlot). The port of the vehicle code of SimulationReference.cpp: SimMakeVehicleSettings
/// and its helpers (`makeVehicleSettings`, `fillWheeled`, ...), SimVehicleData (`Vehicle`), ApplyVehicleOp (`create`,
/// `applyOp`), WriteVehicleState (`writeState`), SimVehicleSettingsPart (`settingsPart`), the callbacks of
/// SetVehicleCallbacks, SimVehicleCombineFriction and SimVehicleTireMaxImpulse.
const VehicleSlot = struct {
    const WheeledVehicleController = zolt.WheeledVehicleController;
    const WheeledVehicleControllerSettings = zolt.WheeledVehicleControllerSettings;
    const WheelSettingsWV = zolt.WheelSettingsWV;
    const WheelWV = zolt.WheelWV;
    const VehicleConstraintSettings = zolt.VehicleConstraintSettings;
    const VehicleCollisionTester = zolt.VehicleCollisionTester;
    const VehicleEngine = zolt.VehicleEngine;
    const VehicleEngineSettings = zolt.VehicleEngineSettings;
    const VehicleTransmission = zolt.VehicleTransmission;
    const VehicleTransmissionSettings = zolt.VehicleTransmissionSettings;
    const VehicleTrackSettings = zolt.VehicleTrackSettings;
    const VehicleDifferentialSettings = zolt.VehicleDifferentialSettings;
    const VehicleAntiRollBar = zolt.VehicleAntiRollBar;
    const LinearCurve = zolt.LinearCurve;
    const SimVehicleEngine = SimScenes.SimVehicleEngine;
    const SimVehicleTransmission = SimScenes.SimVehicleTransmission;
    const SimVehicleTrack = SimScenes.SimVehicleTrack;
    const SimVehicleDifferential = SimScenes.SimVehicleDifferential;
    const SimVehicleAntiRollBar = SimScenes.SimVehicleAntiRollBar;
    const VehicleWheelKind = SimScenes.VehicleWheelKind;
    const VehicleTesterKind = SimScenes.VehicleTesterKind;
    const VehicleCallbacks = SimScenes.VehicleCallbacks;

    /// A vehicle of the world (SimVehicleData of the .cpp): the constraint, its tester, the defaults of its callbacks and
    /// the filters of vehicle_tester_set_filters (the tester keeps pointers to them). Heap allocated: the step callbacks
    /// get it as their context.
    const Vehicle = struct {
        world: *ZoltWorld,
        /// The harness' reference (Ref<VehicleConstraint> mConstraint)
        constraint: Ref(Constraint),
        tester: Ref(VehicleCollisionTester) = .empty,
        handle: u32,
        controller: VehicleControllerKind,
        added: bool = false,
        /// The VehicleCallbacks bits of the step callbacks (the value that the lambdas of SetVehicleCallbacks capture: the
        /// callbacks are set again whenever it changes)
        callbacks: u32 = 0,
        default_combine_friction: VehicleConstraint.CombineFunction = .default,
        default_tire_max_impulse: WheeledVehicleController.TireMaxImpulseCallback = .default,
        broad_phase_layer_filter: ?*zolt.DefaultBroadPhaseLayerFilter = null,
        object_layer_filter: ?*zolt.DefaultObjectLayerFilter = null,
        body_filter: ?*zolt.IgnoreMultipleBodiesFilter = null,

        fn get(self: *const Vehicle) *VehicleConstraint {
            return zolt.virtual.downcast(VehicleConstraint, self.constraint.ptr.?);
        }

        /// ~SimVehicleData: releases the constraint and the tester, frees the filters
        fn destroy(self: *Vehicle, allocator: Allocator) void {
            self.constraint.deinit();
            self.tester.deinit();
            freeFilters(allocator, self.broad_phase_layer_filter, self.object_layer_filter, self.body_filter);
            allocator.destroy(self);
        }
    };

    const Storage = struct {
        /// The vehicles of the vehicle handles
        vehicles: std.ArrayList(*Vehicle) = .empty,
    };

    fn freeFilters(allocator: Allocator, broad_phase_layer_filter: ?*zolt.DefaultBroadPhaseLayerFilter, object_layer_filter: ?*zolt.DefaultObjectLayerFilter, body_filter: ?*zolt.IgnoreMultipleBodiesFilter) void {
        if (broad_phase_layer_filter) |f| allocator.destroy(f);
        if (object_layer_filter) |f| allocator.destroy(f);
        if (body_filter) |f| {
            f.deinit();
            allocator.destroy(f);
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // The settings of a description (SimFillEngine, SimFillTransmission, ... of the .cpp)

    /// SimLoadCurve: a LinearCurve from (x, y) points of the float pool (the curve is replaced; built aside first so that
    /// an allocation failure leaves the curve unchanged)
    fn loadCurve(allocator: Allocator, scene: *const SimSceneDesc, offset: u32, num_points: u32, curve: *LinearCurve) !void {
        var new: LinearCurve = .{};
        errdefer new.deinit(allocator);
        for (0..num_points) |p|
            try new.addPoint(allocator, scene.floats[offset + 2 * p], scene.floats[offset + 2 * p + 1]);
        curve.deinit(allocator);
        curve.* = new;
    }

    /// SimLoadFloats: an array of floats of the float pool (the array is replaced)
    fn loadFloats(allocator: Allocator, scene: *const SimSceneDesc, offset: u32, count: u32, array: *std.ArrayList(f32)) !void {
        var new: std.ArrayList(f32) = .empty;
        try new.appendSlice(allocator, scene.floats[offset..][0..count]);
        array.deinit(allocator);
        array.* = new;
    }

    /// SimFillEngine (the curve is replaced when the description has one; `allocator` is the allocator of the settings)
    fn fillEngine(allocator: Allocator, scene: *const SimSceneDesc, d: *const SimVehicleEngine, engine_settings: *VehicleEngineSettings) !void {
        engine_settings.max_torque = d.max_torque;
        engine_settings.min_rpm = d.min_rpm;
        engine_settings.max_rpm = d.max_rpm;
        if (d.torque_points != invalid_handle)
            try loadCurve(allocator, scene, d.torque_points, d.num_torque_points, &engine_settings.normalized_torque);
        engine_settings.inertia = d.inertia;
        engine_settings.angular_damping = d.angular_damping;
    }

    /// SimFillTransmission (the ratios are replaced when the description has them)
    fn fillTransmission(allocator: Allocator, scene: *const SimSceneDesc, d: *const SimVehicleTransmission, transmission_settings: *VehicleTransmissionSettings) !void {
        transmission_settings.mode = @enumFromInt(@as(u8, @truncate(d.mode))); // ETransmissionMode(uint32): a uint8 enum
        if (d.gear_ratios != invalid_handle)
            try loadFloats(allocator, scene, d.gear_ratios, d.num_gear_ratios, &transmission_settings.gear_ratios);
        if (d.reverse_gear_ratios != invalid_handle)
            try loadFloats(allocator, scene, d.reverse_gear_ratios, d.num_reverse_gear_ratios, &transmission_settings.reverse_gear_ratios);
        transmission_settings.switch_time = d.switch_time;
        transmission_settings.clutch_release_time = d.clutch_release_time;
        transmission_settings.switch_latency = d.switch_latency;
        transmission_settings.shift_up_rpm = d.shift_up_rpm;
        transmission_settings.shift_down_rpm = d.shift_down_rpm;
        transmission_settings.clutch_strength = d.clutch_strength;
    }

    /// SimFillTrack
    fn fillTrack(allocator: Allocator, scene: *const SimSceneDesc, d: *const SimVehicleTrack, track: *VehicleTrackSettings) !void {
        track.driven_wheel = d.driven_wheel;
        track.wheels.clearRetainingCapacity();
        try track.wheels.appendSlice(allocator, scene.uints[d.wheels..][0..d.num_wheels]);
        track.inertia = d.inertia;
        track.angular_damping = d.angular_damping;
        track.max_brake_torque = d.max_brake_torque;
        track.differential_ratio = d.differential_ratio;
    }

    /// SimFillDifferential
    fn fillDifferential(d: *const SimVehicleDifferential) VehicleDifferentialSettings {
        return .{ .left_wheel = d.left_wheel, .right_wheel = d.right_wheel, .differential_ratio = d.differential_ratio, .left_right_split = d.left_right_split, .limited_slip_ratio = d.limited_slip_ratio, .engine_torque_ratio = d.engine_torque_ratio };
    }

    /// SimFillAntiRollBar
    fn fillAntiRollBar(d: *const SimVehicleAntiRollBar) VehicleAntiRollBar {
        return .{ .left_wheel = d.left_wheel, .right_wheel = d.right_wheel, .stiffness = d.stiffness };
    }

    /// SimFillWheeled: the wheeled controller members of a description (WheeledVehicleControllerSettings, also the base
    /// of MotorcycleControllerSettings: MotorcycleSlot.makeControllerSettings calls it). The arrays are allocated with the
    /// settings' allocator.
    fn fillWheeled(scene: *const SimSceneDesc, v: *const SimVehicle, settings: *WheeledVehicleControllerSettings) !void {
        const allocator = settings.base.allocator;
        try fillEngine(allocator, scene, &v.engine, &settings.engine);
        try fillTransmission(allocator, scene, &v.transmission, &settings.transmission);
        try settings.differentials.ensureUnusedCapacity(allocator, v.num_differentials);
        for (scene.vehicle_differentials[v.first_differential..][0..v.num_differentials]) |*d|
            settings.differentials.appendAssumeCapacity(fillDifferential(d));
        settings.differential_limited_slip_ratio = v.differential_limited_slip_ratio;
    }

    /// The new controller settings of a description (SimMakeControllerSettings, no reference yet): the wheeled settings
    /// here, the tracked and motorcycle ones through the hooks of the vehicle variants
    fn makeControllerSettings(allocator: Allocator, scene: *const SimSceneDesc, v: *const SimVehicle) !*VehicleControllerSettings {
        return switch (try controllerKind(v)) {
            .wheeled => {
                const s = try WheeledVehicleControllerSettings.create(allocator);
                errdefer {
                    s.base.addRef();
                    s.base.release();
                }
                try fillWheeled(scene, v, s);
                return &s.base;
            },
            .tracked => TrackedVehicleSlot.makeControllerSettings(allocator, scene, v),
            .motorcycle => MotorcycleSlot.makeControllerSettings(allocator, scene, v),
        };
    }

    /// SimFillWheelSettings: the WheelSettings members of a wheel description
    fn fillWheelSettings(w: *const SimVehicleWheel, s: *WheelSettings) void {
        s.position = load3(&w.position);
        s.suspension_force_point = load3(&w.suspension_force_point);
        s.suspension_direction = load3(&w.suspension_direction);
        s.steering_axis = load3(&w.steering_axis);
        s.wheel_up = load3(&w.wheel_up);
        s.wheel_forward = load3(&w.wheel_forward);
        s.suspension_min_length = w.suspension_min_length;
        s.suspension_max_length = w.suspension_max_length;
        s.suspension_preload_length = w.suspension_preload_length;
        s.suspension_spring.mode = @enumFromInt(@as(u8, @truncate(w.suspension_spring_mode))); // ESpringMode: a uint8 enum
        s.suspension_spring.frequency = w.suspension_frequency;
        s.suspension_spring.damping = w.suspension_damping;
        s.radius = w.radius;
        s.width = w.width;
        s.enable_suspension_force_point = w.enable_suspension_force_point != 0;
    }

    /// The new wheel settings of a description (SimMakeWheelSettings, no reference yet): WheelSettingsWV here,
    /// WheelSettingsTV through the hook of the vehicle variants
    fn makeWheelSettings(allocator: Allocator, scene: *const SimSceneDesc, w: *const SimVehicleWheel) !*WheelSettings {
        if (w.kind != @intFromEnum(VehicleWheelKind.wv))
            return TrackedVehicleSlot.makeWheelSettings(allocator, scene, w);
        const wv = try WheelSettingsWV.create(allocator);
        errdefer {
            wv.base.addRef();
            wv.base.release();
        }
        fillWheelSettings(w, &wv.base);
        wv.inertia = w.inertia;
        wv.angular_damping = w.angular_damping;
        wv.max_steer_angle = w.max_steer_angle;
        if (w.longitudinal_friction != invalid_handle)
            try loadCurve(allocator, scene, w.longitudinal_friction, w.num_longitudinal_friction, &wv.longitudinal_friction);
        if (w.lateral_friction != invalid_handle)
            try loadCurve(allocator, scene, w.lateral_friction, w.num_lateral_friction, &wv.lateral_friction);
        wv.max_brake_torque = w.max_brake_torque;
        wv.max_hand_brake_torque = w.max_hand_brake_torque;
        return &wv.base;
    }

    /// SimMakeVehicleSettings: the VehicleConstraintSettings of a vehicle description
    fn makeVehicleSettings(allocator: Allocator, scene: *const SimSceneDesc, v: *const SimVehicle) !Ref(ConstraintSettings) {
        const s = try VehicleConstraintSettings.create(allocator);
        var result = Ref(ConstraintSettings).init(s.asSettings());
        errdefer result.deinit();
        s.base.enabled = v.enabled != 0;
        s.base.constraint_priority = v.constraint_priority;
        s.base.num_velocity_steps_override = v.num_velocity_steps_override;
        s.base.num_position_steps_override = v.num_position_steps_override;
        s.base.draw_constraint_size = v.draw_constraint_size;
        s.base.user_data = v.user_data;
        s.up = load3(&v.up);
        s.forward = load3(&v.forward);
        s.max_pitch_roll_angle = v.max_pitch_roll_angle;
        for (scene.vehicle_wheels[v.first_wheel..][0..v.num_wheels]) |*w| {
            var wheel = Ref(WheelSettings).init(try makeWheelSettings(allocator, scene, w));
            defer wheel.deinit();
            try s.addWheel(wheel.ptr.?);
        }
        for (scene.vehicle_anti_roll_bars[v.first_anti_roll_bar..][0..v.num_anti_roll_bars]) |*bar|
            try s.addAntiRollBar(fillAntiRollBar(bar));
        var controller = Ref(VehicleControllerSettings).init(try makeControllerSettings(allocator, scene, v));
        defer controller.deinit();
        s.setController(controller.ptr);
        return result;
    }

    /// SimMakeTester: a new collision tester (no reference yet)
    fn makeTester(allocator: Allocator, kind: u32, object_layer: u32, up: Vec3, max_slope_angle: f32, radius: f32, convex_radius_fraction: f32) !*VehicleCollisionTester {
        const layer: ObjectLayer = @truncate(object_layer);
        return switch (std.enums.fromInt(VehicleTesterKind, kind) orelse return error.SceneError) {
            .ray => &(try zolt.VehicleCollisionTesterRay.create(allocator, layer, .{ .up = up, .max_slope_angle = max_slope_angle })).base,
            .cast_sphere => &(try zolt.VehicleCollisionTesterCastSphere.create(allocator, layer, radius, .{ .up = up, .max_slope_angle = max_slope_angle })).base,
            .cast_cylinder => &(try zolt.VehicleCollisionTesterCastCylinder.create(allocator, layer, .{ .convex_radius_fraction = convex_radius_fraction })).base,
        };
    }

    fn controllerKind(v: *const SimVehicle) !VehicleControllerKind {
        return std.enums.fromInt(VehicleControllerKind, v.controller) orelse error.SceneError;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // The callbacks (SetVehicleCallbacks, SimVehicleCombineFriction, SimVehicleTireMaxImpulse of the .cpp)

    /// SimVehicleRecord: the vehicle handle and body, the step, the wheels with a contact and the hash of the suspension
    /// length, angular velocity and steer angle of every wheel
    fn record(kind: RecordKind, handle: u32, v: *const VehicleConstraint, context: *const PhysicsStepListenerContext) SimRecord {
        var contacts: u32 = 0;
        var hash = SimScenes.hash_seed;
        for (v.getWheels(), 0..) |wheel, w| {
            if (wheel.hasContact())
                contacts |= @as(u32, 1) << @intCast(w);
            hash = VehicleRecords.hashWheel(hash, wheel.getSuspensionLength(), wheel.getAngularVelocity(), wheel.getSteerAngle());
        }
        return VehicleRecords.record(kind, handle, v.getVehicleBody().getID(), context, contacts, hash);
    }

    fn preStep(self: *Vehicle, v: *VehicleConstraint, context: *const PhysicsStepListenerContext) void {
        if ((self.callbacks & VehicleCallbacks.pre_step) != 0)
            self.world.recorder.add(record(.vehicle_pre_step, self.handle, v, context));
        if ((self.callbacks & VehicleCallbacks.steer) != 0 and self.controller != .tracked) {
            const c = zolt.virtual.downcast(WheeledVehicleController, v.getControllerMut());
            c.setRightInput(if (c.getRightInput() > 0.0) -0.25 else 0.5);
        }
    }

    fn postCollide(self: *Vehicle, v: *VehicleConstraint, context: *const PhysicsStepListenerContext) void {
        self.world.recorder.add(record(.vehicle_post_collide, self.handle, v, context));
    }

    fn postStep(self: *Vehicle, v: *VehicleConstraint, context: *const PhysicsStepListenerContext) void {
        self.world.recorder.add(record(.vehicle_post_step, self.handle, v, context));
    }

    /// SetVehicleCallbacks: the step callbacks of VehicleCallbacks bits `callbacks`
    fn setCallbacks(self: *Vehicle, callbacks: u32) void {
        const v = self.get();
        self.callbacks = callbacks;
        v.setPreStepCallback(if ((callbacks & (VehicleCallbacks.pre_step | VehicleCallbacks.steer)) != 0) VehicleConstraint.StepCallback.init(self, preStep) else null);
        v.setPostCollideCallback(if ((callbacks & VehicleCallbacks.post_collide) != 0) VehicleConstraint.StepCallback.init(self, postCollide) else null);
        v.setPostStepCallback(if ((callbacks & VehicleCallbacks.post_step) != 0) VehicleConstraint.StepCallback.init(self, postStep) else null);
    }

    /// VehicleCombineFriction.custom (SimVehicleCombineFriction)
    fn customCombineFriction(context: ?*anyopaque, wheel_index: u32, longitudinal_friction: *f32, lateral_friction: *f32, body2: *const Body, sub_shape_id2: SubShapeID) void {
        _ = .{ context, sub_shape_id2 };
        const body_friction = body2.getFriction();
        longitudinal_friction.* = 0.5 * (longitudinal_friction.* + body_friction);
        lateral_friction.* = zolt.math.min(lateral_friction.*, body_friction);
        if ((wheel_index & 1) != 0) {
            longitudinal_friction.* *= 0.9;
            lateral_friction.* *= 0.9;
        }
    }

    /// VehicleTireMaxImpulse.custom (SimVehicleTireMaxImpulse)
    fn customTireMaxImpulse(context: ?*anyopaque, wheel_index: u32, longitudinal_impulse: *f32, lateral_impulse: *f32, suspension_impulse: f32, longitudinal_friction: f32, lateral_friction: f32, longitudinal_slip: f32, lateral_slip: f32, delta_time: f32) void {
        _ = .{ context, longitudinal_slip, lateral_slip, delta_time };
        longitudinal_impulse.* = 0.9 * longitudinal_friction * suspension_impulse;
        lateral_impulse.* = lateral_friction * suspension_impulse;
        if (wheel_index == 0) {
            longitudinal_impulse.* *= 0.5;
            lateral_impulse.* *= 0.5;
        }
    }

    /// SetVehicleCombineFriction
    fn setCombineFriction(self: *Vehicle, mode: u32) void {
        self.get().setCombineFriction(if (mode == 1) .{ .function = customCombineFriction } else self.default_combine_friction);
    }

    /// SetVehicleTireMaxImpulse (`op` null: from the description)
    fn setTireMaxImpulse(self: *Vehicle, mode: u32, op: ?*const SimOp) !void {
        if (self.controller == .tracked) {
            if (mode == 0 and op == null)
                return; // A tracked vehicle has no tire callback, its description has the default
            std.debug.print("SimulationParity: tire max impulse callback on the tracked vehicle handle {d}\n", .{self.handle});
            return error.SceneError;
        }
        wheeled(self.get()).setTireMaxImpulseCallback(if (mode == 1) .{ .function = customTireMaxImpulse } else self.default_tire_max_impulse);
    }

    /// The wheeled controller of a wheeled or motorcycle vehicle (a MotorcycleController is a WheeledVehicleController)
    fn wheeled(v: *VehicleConstraint) *WheeledVehicleController {
        return zolt.virtual.downcast(WheeledVehicleController, v.getControllerMut());
    }

    /// GetWheeledController of the .cpp: a wheeled operation on a tracked vehicle is a scene error
    fn wheeledOf(self: *const Vehicle, op: *const SimOp) !*WheeledVehicleController {
        if (self.controller == .tracked) {
            std.debug.print("SimulationParity: wheeled operation {d} on the tracked vehicle handle {d}\n", .{ op.kind, op.index });
            return error.SceneError;
        }
        return wheeled(self.get());
    }

    /// GetEngine / GetTransmission of the .cpp (every controller kind has them; the tracked controller's through the hooks)
    fn engineOf(self: *const Vehicle) !*VehicleEngine {
        return if (self.controller == .tracked) TrackedVehicleSlot.engine(self.get()) else wheeled(self.get()).getEngineMut();
    }

    fn transmissionOf(self: *const Vehicle) !*VehicleTransmission {
        return if (self.controller == .tracked) TrackedVehicleSlot.transmission(self.get()) else wheeled(self.get()).getTransmissionMut();
    }

    // -----------------------------------------------------------------------------------------------------------------
    // The slot

    /// create_vehicle (ApplyVehicleOp of the .cpp): the vehicle of the next handle; returns the number of constraints
    fn create(world: *ZoltWorld, op: *const SimOp) !u32 {
        const allocator = world.allocator;
        const storage = &world.phase7.vehicles;
        const d = &world.desc.vehicles[op.index];
        try storage.vehicles.ensureUnusedCapacity(allocator, 1);
        var settings = try makeVehicleSettings(allocator, world.desc, d);
        defer settings.deinit();

        const constraint = blk: {
            var lock: zolt.BodyLockWrite = .init(world.system.getBodyLockInterface(), world.getHandle(d.body));
            defer lock.deinit();
            if (!lock.succeeded()) {
                std.debug.print("SimulationParity: vehicle description {d}: the body handle {d} does not exist\n", .{ op.index, d.body });
                return error.SceneError;
            }
            break :blk try VehicleConstraint.create(allocator, lock.getBody(), zolt.virtual.downcast(VehicleConstraintSettings, settings.get().?));
        };
        var constraint_ref = Ref(Constraint).init(&constraint.base);
        errdefer constraint_ref.deinit();
        var tester = Ref(VehicleCollisionTester).init(try makeTester(allocator, d.tester.kind, d.tester.object_layer, load3(&d.tester.up), d.tester.max_slope_angle, d.tester.radius, d.tester.convex_radius_fraction));
        errdefer tester.deinit();

        const v = try allocator.create(Vehicle);
        errdefer allocator.destroy(v);
        v.* = .{ .world = world, .constraint = constraint_ref, .tester = tester, .handle = @intCast(storage.vehicles.items.len), .controller = try controllerKind(d) };
        constraint.setVehicleCollisionTester(v.tester.ptr);
        v.default_combine_friction = constraint.getCombineFriction();
        if (v.controller != .tracked)
            v.default_tire_max_impulse = wheeled(constraint).getTireMaxImpulseCallback();
        setCallbacks(v, d.callbacks);
        setCombineFriction(v, d.combine_friction);
        try setTireMaxImpulse(v, d.tire_max_impulse, null);
        try world.system.addConstraint(&constraint.base);
        errdefer world.system.removeConstraint(&constraint.base);
        try world.system.addStepListener(constraint.stepListener());
        v.added = true;
        storage.vehicles.appendAssumeCapacity(v);
        return world.system.constraint_manager.getNumConstraints();
    }

    /// The vehicle of a handle (GetVehicle of the .cpp)
    fn getVehicle(world: *ZoltWorld, handle: u32) !*Vehicle {
        const vehicles = world.phase7.vehicles.vehicles.items;
        if (handle >= vehicles.len) {
            std.debug.print("SimulationParity: operation on vehicle handle {d} that does not exist\n", .{handle});
            return error.SceneError;
        }
        return vehicles[handle];
    }

    fn vehicle(world: *ZoltWorld, handle: u32) !*VehicleConstraint {
        return (try getVehicle(world, handle)).get();
    }

    /// SimConstraintState of the vehicle constraint
    fn constraintState(v: *VehicleConstraint) u32 {
        return @as(u32, @intFromBool(v.base.getEnabled())) | (@as(u32, @intFromBool(v.base.isActive())) << 1);
    }

    /// The SaveBinaryState bytes of settings as a hash (HashBinaryState of the .cpp)
    fn hashBinaryState(allocator: Allocator, settings: anytype) !u32 {
        var out: std.Io.Writer.Allocating = .init(allocator);
        defer out.deinit();
        var wrapper = zolt.StreamOutWrapper.init(&out.writer);
        settings.saveBinaryState(wrapper.streamOut());
        if (wrapper.isFailed()) return error.OutOfMemory;
        return SimScenes.hashBytes(SimScenes.hash_seed, out.written());
    }

    /// The operations of Phase7OpRange.vehicle but create_vehicle (ApplyVehicleOp of the .cpp)
    fn applyOp(world: *ZoltWorld, op: *const SimOp) !u32 {
        const allocator = world.allocator;
        const i = &op.i;
        const f = &op.f;
        const v = try getVehicle(world, op.index);
        const c = v.get();
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .vehicle_remove => {
                world.system.removeStepListener(c.stepListener());
                world.system.removeConstraint(&c.base);
                v.added = false;
                return world.system.constraint_manager.getNumConstraints();
            },
            .vehicle_add => {
                try world.system.addConstraint(&c.base);
                errdefer world.system.removeConstraint(&c.base);
                try world.system.addStepListener(c.stepListener());
                v.added = true;
                return world.system.constraint_manager.getNumConstraints();
            },
            .vehicle_set_max_pitch_roll_angle => c.setMaxPitchRollAngle(f[0]),
            .vehicle_override_gravity => c.overrideGravity(load3(f[0..3])),
            .vehicle_reset_gravity_override => c.resetGravityOverride(),
            .vehicle_set_num_steps_between_collision_test_active => c.setNumStepsBetweenCollisionTestActive(i[0]),
            .vehicle_set_num_steps_between_collision_test_inactive => c.setNumStepsBetweenCollisionTestInactive(i[0]),
            .vehicle_set_collision_tester => {
                const tester = Ref(VehicleCollisionTester).init(try makeTester(allocator, i[0], i[1], load3(f[0..3]), f[3], f[4], f[5]));
                v.tester.deinit();
                v.tester = tester;
                c.setVehicleCollisionTester(v.tester.ptr);
            },
            .vehicle_tester_set_object_layer => v.tester.ptr.?.setObjectLayer(@truncate(i[0])),
            .vehicle_tester_set_filters => {
                const tester = v.tester.ptr.?;
                if (i[0] == 0) {
                    tester.setBroadPhaseLayerFilter(null);
                    tester.setObjectLayerFilter(null);
                    tester.setBodyFilter(null);
                } else {
                    const layer: ObjectLayer = @truncate(i[1]);
                    const broad_phase_layer_filter = try allocator.create(zolt.DefaultBroadPhaseLayerFilter);
                    errdefer allocator.destroy(broad_phase_layer_filter);
                    broad_phase_layer_filter.* = .init(&world.object_vs_broad_phase_layer_filter.base, layer);
                    const object_layer_filter = try allocator.create(zolt.DefaultObjectLayerFilter);
                    errdefer allocator.destroy(object_layer_filter);
                    object_layer_filter.* = .init(&world.object_layer_pair_filter.base, layer);
                    const body_filter = try allocator.create(zolt.IgnoreMultipleBodiesFilter);
                    errdefer allocator.destroy(body_filter);
                    body_filter.* = .init(allocator);
                    errdefer body_filter.deinit();
                    try body_filter.ignoreBody(c.getVehicleBody().getID());
                    const ignored = world.getHandle(op.body);
                    if (!ignored.isInvalid())
                        try body_filter.ignoreBody(ignored);

                    // Replace the filters (std::unique_ptr::operator=)
                    freeFilters(allocator, v.broad_phase_layer_filter, v.object_layer_filter, v.body_filter);
                    v.broad_phase_layer_filter = broad_phase_layer_filter;
                    v.object_layer_filter = object_layer_filter;
                    v.body_filter = body_filter;
                    tester.setBroadPhaseLayerFilter(&broad_phase_layer_filter.base);
                    tester.setObjectLayerFilter(&object_layer_filter.base);
                    tester.setBodyFilter(&body_filter.base);
                }
            },
            .vehicle_set_combine_friction => setCombineFriction(v, i[0]),
            .vehicle_set_callbacks => setCallbacks(v, i[0]),
            .vehicle_wheel_set_angular_velocity => c.getWheelMut(i[0]).setAngularVelocity(f[0]),
            .vehicle_wheel_set_rotation_angle => c.getWheelMut(i[0]).setRotationAngle(f[0]),
            .vehicle_wheel_set_steer_angle => c.getWheelMut(i[0]).setSteerAngle(f[0]),
            .vehicle_set_anti_roll_bar => {
                const bar = &c.getAntiRollBarsMut().items[i[0]];
                bar.left_wheel = @bitCast(i[1]);
                bar.right_wheel = @bitCast(i[2]);
                bar.stiffness = f[0];
            },
            .vehicle_settings_hash => {
                var settings = try c.base.getConstraintSettings();
                defer settings.deinit();
                return hashBinaryState(allocator, settings.get().?);
            },
            .vehicle_controller_settings_hash => {
                var settings = try c.getController().getSettings();
                defer settings.deinit();
                return hashBinaryState(allocator, settings.get().?);
            },
            .vehicle_expect_wheel_contacts => {
                // Changes nothing, the driver checks the result
                var bits: u32 = 0;
                for (c.getWheels(), 0..) |wheel, w| {
                    if (wheel.hasContact())
                        bits |= @as(u32, 1) << @intCast(w);
                }
                return bits;
            },
            .vehicle_engine_set_current_rpm => (try engineOf(v)).setCurrentRPM(f[0]),
            .vehicle_engine_set_settings => {
                const engine = try engineOf(v);
                engine.base.max_torque = f[0];
                engine.base.min_rpm = f[1];
                engine.base.max_rpm = f[2];
                engine.base.inertia = f[3];
                engine.base.angular_damping = f[4];
            },
            .vehicle_engine_set_torque_curve => try loadCurve(c.getController().allocator, world.desc, i[0], i[1], &(try engineOf(v)).base.normalized_torque),
            .vehicle_transmission_set => (try transmissionOf(v)).set(@bitCast(i[0]), f[0]),
            .vehicle_transmission_set_settings => {
                const transmission = try transmissionOf(v);
                transmission.base.mode = @enumFromInt(@as(u8, @truncate(i[0])));
                transmission.base.switch_time = f[0];
                transmission.base.clutch_release_time = f[1];
                transmission.base.switch_latency = f[2];
                transmission.base.shift_up_rpm = f[3];
                transmission.base.shift_down_rpm = f[4];
                transmission.base.clutch_strength = f[5];
            },
            .vehicle_transmission_set_gear_ratios => {
                // Both arrays are replaced (build both first: an allocation failure changes nothing)
                const controller_allocator = c.getController().allocator;
                const transmission = try transmissionOf(v);
                var gear_ratios: std.ArrayList(f32) = .empty;
                errdefer gear_ratios.deinit(controller_allocator);
                try loadFloats(controller_allocator, world.desc, i[0], i[1], &gear_ratios);
                var reverse_gear_ratios: std.ArrayList(f32) = .empty;
                try loadFloats(controller_allocator, world.desc, i[2], i[3], &reverse_gear_ratios);
                transmission.base.gear_ratios.deinit(controller_allocator);
                transmission.base.gear_ratios = gear_ratios;
                transmission.base.reverse_gear_ratios.deinit(controller_allocator);
                transmission.base.reverse_gear_ratios = reverse_gear_ratios;
            },
            .wheeled_set_driver_input => (try wheeledOf(v, op)).setDriverInput(f[0], f[1], f[2], f[3]),
            .wheeled_set_forward_input => (try wheeledOf(v, op)).setForwardInput(f[0]),
            .wheeled_set_right_input => (try wheeledOf(v, op)).setRightInput(f[0]),
            .wheeled_set_brake_input => (try wheeledOf(v, op)).setBrakeInput(f[0]),
            .wheeled_set_hand_brake_input => (try wheeledOf(v, op)).setHandBrakeInput(f[0]),
            .wheeled_set_differential => {
                const d = &(try wheeledOf(v, op)).getDifferentialsMut()[i[0]];
                d.left_wheel = @bitCast(i[1]);
                d.right_wheel = @bitCast(i[2]);
                d.differential_ratio = f[0];
                d.left_right_split = f[1];
                d.limited_slip_ratio = f[2];
                d.engine_torque_ratio = f[3];
            },
            .wheeled_set_differential_limited_slip_ratio => (try wheeledOf(v, op)).setDifferentialLimitedSlipRatio(f[0]),
            .wheeled_set_tire_max_impulse_callback => try setTireMaxImpulse(v, i[0], op),

            // The Constraint interface of the VehicleConstraint (a vehicle is not a constraint handle)
            .vehicle_set_enabled => c.base.setEnabled(i[0] != 0),
            .vehicle_reset_warm_start => c.base.resetWarmStart(),
            .vehicle_set_constraint_priority => c.base.setConstraintPriority(i[0]),
            .vehicle_set_steps_override => {
                c.base.setNumVelocityStepsOverride(i[0]);
                c.base.setNumPositionStepsOverride(i[1]);
            },
            .vehicle_set_user_data => c.base.setUserData(@as(u64, i[0]) | (@as(u64, i[1]) << 32)),
            .vehicle_expect_constraint_state => {
                // Changes nothing, the driver checks the result: a disabled vehicle whose body is active runs OnStep (its
                // callbacks, the wheel collision tests) but is not solved (IsActive includes mEnabled)
                return VehicleSlot.constraintState(c) | (@as(u32, @intFromBool(world.system.getBodyInterface().isActive(c.getVehicleBody().getID()))) << 2);
            },
            .vehicle_wheel_wv_apply_torque => {
                _ = try wheeledOf(v, op);
                zolt.virtual.downcast(WheelWV, c.getWheelMut(i[0])).applyTorque(f[0], f[1]);
            },
            .vehicle_engine_apply_torque => (try engineOf(v)).applyTorque(f[0], f[1]),
            .vehicle_engine_apply_damping => (try engineOf(v)).applyDamping(f[0]),
            .wheeled_set_differentials => {
                const controller = try wheeledOf(v, op);
                const differentials = try allocator.alloc(VehicleDifferentialSettings, i[1]);
                defer allocator.free(differentials);
                for (differentials, world.desc.vehicle_differentials[i[0]..][0..i[1]]) |*d, *desc|
                    d.* = fillDifferential(desc);
                try controller.setDifferentials(differentials);
            },
            else => {
                std.debug.print("SimulationParity: unknown vehicle operation {d}\n", .{op.kind});
                return error.SceneError;
            },
        }
        return VehicleSlot.constraintState(c);
    }

    /// WriteEngineState of the .cpp: the vehicle_engine field of an engine (also for the tracked controller)
    fn writeEngineState(w: *ObjectStateWriter, handle: u32, engine: *const VehicleEngine) !void {
        try w.begin(.vehicle_engine, handle, 0);
        try w.value(engine.getCurrentRPM());
        try w.value(engine.getAngularVelocity());
        try w.value(engine.getTorque(1.0));
        try w.boolean(engine.allowSleep());
        try w.value(engine.base.max_torque);
        try w.value(engine.base.min_rpm);
        try w.value(engine.base.max_rpm);
        try w.value(engine.base.inertia);
        try w.value(engine.base.angular_damping);
        w.end();
    }

    /// WriteTransmissionState of the .cpp: the vehicle_transmission field of a transmission (also for the tracked
    /// controller)
    fn writeTransmissionState(w: *ObjectStateWriter, handle: u32, transmission: *const VehicleTransmission) !void {
        try w.begin(.vehicle_transmission, handle, 0);
        try w.value(transmission.getCurrentGear());
        try w.value(transmission.getClutchFriction());
        try w.boolean(transmission.isSwitchingGear());
        try w.value(transmission.getCurrentRatio());
        try w.boolean(transmission.allowSleep());
        try w.value(@as(u32, @intFromEnum(transmission.base.mode)));
        w.end();
    }

    /// WriteVehicleState of the .cpp for every vehicle in handle order
    fn writeState(world: *ZoltWorld, w: *ObjectStateWriter) !void {
        for (world.phase7.vehicles.vehicles.items) |v|
            try writeVehicleState(world, w, v);
    }

    fn writeVehicleState(world: *ZoltWorld, w: *ObjectStateWriter, v: *const Vehicle) !void {
        const h = v.handle;
        const c = v.get();
        const num_wheels: u32 = @intCast(c.getWheels().len);

        try w.begin(.vehicle_properties, h, 0);
        try w.boolean(c.base.isActive());
        try w.boolean(c.base.getEnabled());
        try w.value(c.getNumStepsBetweenCollisionTestActive());
        try w.value(c.getNumStepsBetweenCollisionTestInactive());
        try w.value(c.getMaxPitchRollAngle());
        try w.boolean(c.isGravityOverridden());
        try w.vec3(c.getGravityOverride());
        try w.vec3(c.getLocalForward());
        try w.vec3(c.getLocalUp());
        try w.vec3(c.getWorldUp());
        try w.value(num_wheels);
        try w.value(c.getVehicleBody().getID().getIndexAndSequenceNumber());
        try w.value(c.base.getConstraintPriority());
        try w.value(c.base.getNumVelocityStepsOverride());
        try w.value(c.base.getNumPositionStepsOverride());
        try w.value(c.base.getUserData());
        w.end();

        for (c.getWheels(), 0..) |wheel, index| {
            const item: u32 = @intCast(index);
            try w.begin(.vehicle_wheel, h, item);
            try w.value(wheel.getAngularVelocity());
            try w.value(wheel.getRotationAngle());
            try w.value(wheel.getSteerAngle());
            try w.boolean(wheel.hasContact());
            try w.value(wheel.getContactBodyID().getIndexAndSequenceNumber());
            try w.value(wheel.getContactSubShapeID().getValue());
            try w.value(wheel.getSuspensionLength());
            try w.boolean(wheel.hasHitHardPoint());
            try w.value(wheel.getSuspensionLambda());
            try w.value(wheel.getLongitudinalLambda());
            try w.value(wheel.getLateralLambda());
            w.end();

            if (wheel.hasContact()) {
                try w.begin(.vehicle_wheel_contact, h, item);
                try w.rvec3(wheel.getContactPosition());
                try w.vec3(wheel.getContactPointVelocity());
                try w.vec3(wheel.getContactNormal());
                try w.vec3(wheel.getContactLongitudinal());
                try w.vec3(wheel.getContactLateral());
                w.end();
            }

            const basis = c.getWheelLocalBasis(wheel);
            try w.begin(.vehicle_wheel_transform, h, item);
            try w.vec3(basis.forward);
            try w.vec3(basis.up);
            try w.vec3(basis.right);
            try w.mat44(c.getWheelLocalTransform(item, Vec3.axisY(), Vec3.axisX()));
            try w.rmat44(c.getWheelWorldTransform(item, Vec3.axisY(), Vec3.axisX()));
            w.end();

            if (v.controller == .tracked) {
                try TrackedVehicleSlot.writeWheelState(world, w, h, item, wheel);
            } else {
                const wv = zolt.virtual.downcast(WheelWV, wheel);
                try w.begin(.vehicle_wheel_wv, h, item);
                try w.value(wv.longitudinal_slip);
                try w.value(wv.lateral_slip);
                try w.value(wv.combined_longitudinal_friction);
                try w.value(wv.combined_lateral_friction);
                try w.value(wv.brake_impulse);
                w.end();
            }
        }

        for (c.getAntiRollBars(), 0..) |bar, b| {
            try w.begin(.vehicle_anti_roll_bar, h, @intCast(b));
            try w.value(bar.left_wheel);
            try w.value(bar.right_wheel);
            try w.value(bar.stiffness);
            w.end();
        }

        if (v.controller == .tracked) {
            try TrackedVehicleSlot.writeControllerState(world, w, h, c);
            return;
        }

        const controller = zolt.virtual.downcast(WheeledVehicleController, c.getController());
        try writeEngineState(w, h, controller.getEngine());
        try writeTransmissionState(w, h, controller.getTransmission());
        try w.begin(.vehicle_wheeled, h, 0);
        try w.value(controller.getForwardInput());
        try w.value(controller.getRightInput());
        try w.value(controller.getBrakeInput());
        try w.value(controller.getHandBrakeInput());
        try w.value(controller.getDifferentialLimitedSlipRatio());
        try w.value(controller.getWheelSpeedAtClutch());
        w.end();

        for (controller.getDifferentials(), 0..) |*d, index| {
            const left_angular_velocity: f32 = if (d.left_wheel != -1) c.getWheel(@intCast(d.left_wheel)).getAngularVelocity() else 0.0;
            const right_angular_velocity: f32 = if (d.right_wheel != -1) c.getWheel(@intCast(d.right_wheel)).getAngularVelocity() else 0.0;
            const ratio = d.calculateTorqueRatio(left_angular_velocity, right_angular_velocity);
            try w.begin(.vehicle_differential, h, @intCast(index));
            try w.value(d.left_wheel);
            try w.value(d.right_wheel);
            try w.value(d.differential_ratio);
            try w.value(d.left_right_split);
            try w.value(d.limited_slip_ratio);
            try w.value(d.engine_torque_ratio);
            try w.value(ratio.left);
            try w.value(ratio.right);
            w.end();
        }

        if (v.controller == .motorcycle)
            try MotorcycleSlot.writeControllerState(world, w, h, c);
    }

    fn saveStates(world: *ZoltWorld, recorder: *StateRecorderImpl) !void {
        _ = .{ world, recorder }; // Vehicles have no character state
    }

    fn restoreStates(world: *ZoltWorld, recorder: *StateRecorderImpl) !void {
        _ = .{ world, recorder };
    }

    /// ~SimWorld of the .cpp: remove the vehicles that are in the system, release them, free the storage
    fn destroyAll(world: *ZoltWorld) void {
        const allocator = world.allocator;
        const storage = &world.phase7.vehicles;
        for (storage.vehicles.items) |v| {
            if (v.added) {
                const c = v.get();
                world.system.removeStepListener(c.stepListener());
                world.system.removeConstraint(&c.base);
            }
            v.destroy(allocator);
        }
        storage.vehicles.deinit(allocator);
        storage.* = .{};
    }

    // -----------------------------------------------------------------------------------------------------------------
    // The settings round trips (SimVehicleSettingsPart of the .cpp)

    /// SaveBinaryState of `object` into `out`
    fn saveInto(allocator: Allocator, object: anytype, out: *std.ArrayList(u8)) !void {
        var data: std.Io.Writer.Allocating = .init(allocator);
        defer data.deinit();
        var wrapper = zolt.StreamOutWrapper.init(&data.writer);
        object.saveBinaryState(wrapper.streamOut());
        if (wrapper.isFailed()) return error.OutOfMemory;
        out.clearRetainingCapacity();
        try out.appendSlice(allocator, data.written());
    }

    /// SimReadCompletely: true when the stream was read without error and completely (the stream sets EOF only on a
    /// read past the end: one more byte is read, which must fail at the end)
    fn readCompletely(wrapper: *zolt.StreamInWrapper) bool {
        const read = !wrapper.isFailed();
        var extra: u8 = 0;
        wrapper.streamIn().read(&extra);
        return read and wrapper.isEOF();
    }

    /// The stream of SimRestoreBinaryState (initialized in place: the wrapper points to the reader)
    const RestoreStream = struct {
        reader: std.Io.Reader,
        wrapper: zolt.StreamInWrapper,

        fn init(self: *RestoreStream, bytes: []const u8) void {
            self.reader = .fixed(bytes);
            self.wrapper = .init(&self.reader);
        }

        fn stream(self: *RestoreStream) zolt.StreamIn {
            return self.wrapper.streamIn();
        }

        /// True when every byte was read (SimReadCompletely)
        fn completely(self: *RestoreStream) bool {
            return readCompletely(&self.wrapper);
        }
    };

    /// The bytes of a part of vehicle description `index` (jolt_sim_vehicle_settings_part): without `restore_from` the
    /// SaveBinaryState bytes of the part as `create` makes it, with it a default object of the part's class restores the
    /// bytes (every byte read) and is saved again. False: the part does not exist (an item out of range, a part of another
    /// controller kind, the restore of `settings`: Jolt's VehicleConstraintSettings::RestoreBinaryState dereferences null
    /// wheel settings, CharacterVehicleArchitecture E6) or the restore failed.
    fn settingsPart(allocator: Allocator, scene: *const SimSceneDesc, index: u32, part: VehicleSettingsPart, item: u32, restore_from: ?[]const u8, out: *std.ArrayList(u8)) !bool {
        const v = &scene.vehicles[index];
        out.clearRetainingCapacity();
        switch (part) {
            .settings => {
                if (item != 0 or restore_from != null)
                    return false;
                var s = try makeVehicleSettings(allocator, scene, v);
                defer s.deinit();
                try saveInto(allocator, s.get().?, out);
            },
            .settings_without_wheels => {
                if (item != 0)
                    return false;
                if (restore_from) |bytes| {
                    // The RTTI hash, then VehicleConstraintSettings::RestoreBinaryState (the controller from its hash)
                    var reader: std.Io.Reader = .fixed(bytes);
                    var wrapper = zolt.StreamInWrapper.init(&reader);
                    var result = try ConstraintSettings.restoreFromBinaryState(allocator, wrapper.streamIn());
                    defer result.deinit();
                    if (result.hasError() or !readCompletely(&wrapper))
                        return false;
                    try saveInto(allocator, result.get().get().?, out);
                } else {
                    var s = try makeVehicleSettings(allocator, scene, v);
                    defer s.deinit();
                    const vs = zolt.virtual.downcast(VehicleConstraintSettings, s.get().?);
                    for (vs.wheels.items) |*w| w.deinit();
                    vs.wheels.clearRetainingCapacity();
                    try saveInto(allocator, s.get().?, out);
                }
            },
            .controller => {
                if (item != 0)
                    return false;
                var s = Ref(VehicleControllerSettings).init(if (restore_from == null)
                    try makeControllerSettings(allocator, scene, v)
                else switch (try controllerKind(v)) {
                    .wheeled => &(try WheeledVehicleControllerSettings.create(allocator)).base,
                    .tracked => try TrackedVehicleSlot.makeControllerSettings(allocator, scene, &SimVehicle.defaults(.tracked)),
                    .motorcycle => try MotorcycleSlot.makeControllerSettings(allocator, scene, &SimVehicle.defaults(.motorcycle)),
                });
                defer s.deinit();
                if (restore_from) |bytes| {
                    var in: RestoreStream = undefined;
                    in.init(bytes);
                    try s.get().?.restoreBinaryState(in.stream());
                    if (!in.completely())
                        return false;
                }
                try saveInto(allocator, s.get().?, out);
            },
            .wheel => {
                if (item >= v.num_wheels)
                    return false;
                const w = &scene.vehicle_wheels[v.first_wheel + item];
                var s = Ref(WheelSettings).init(if (restore_from == null)
                    try makeWheelSettings(allocator, scene, w)
                else if (w.kind == @intFromEnum(VehicleWheelKind.wv))
                    &(try WheelSettingsWV.create(allocator)).base
                else
                    try TrackedVehicleSlot.makeWheelSettings(allocator, scene, &SimVehicleWheel.defaults(.tv)));
                defer s.deinit();
                if (restore_from) |bytes| {
                    var in: RestoreStream = undefined;
                    in.init(bytes);
                    try s.get().?.restoreBinaryState(in.stream());
                    if (!in.completely())
                        return false;
                }
                try saveInto(allocator, s.get().?, out);
            },
            .engine => {
                if (item != 0)
                    return false;
                var s = try VehicleEngineSettings.init(allocator);
                defer s.deinit(allocator);
                if (restore_from) |bytes| {
                    var in: RestoreStream = undefined;
                    in.init(bytes);
                    try s.restoreBinaryState(allocator, in.stream());
                    if (!in.completely())
                        return false;
                } else try fillEngine(allocator, scene, &v.engine, &s);
                try saveInto(allocator, &s, out);
            },
            .transmission => {
                if (item != 0)
                    return false;
                var s = try VehicleTransmissionSettings.init(allocator);
                defer s.deinit(allocator);
                if (restore_from) |bytes| {
                    var in: RestoreStream = undefined;
                    in.init(bytes);
                    try s.restoreBinaryState(allocator, in.stream());
                    if (!in.completely())
                        return false;
                } else try fillTransmission(allocator, scene, &v.transmission, &s);
                try saveInto(allocator, &s, out);
            },
            .track => {
                if (v.controller != @intFromEnum(VehicleControllerKind.tracked) or item >= 2)
                    return false;
                var s: VehicleTrackSettings = .{}; // mDrivenWheel: 0 (uninitialized in Jolt)
                defer s.deinit(allocator);
                if (restore_from) |bytes| {
                    var in: RestoreStream = undefined;
                    in.init(bytes);
                    try s.restoreBinaryState(allocator, in.stream());
                    if (!in.completely())
                        return false;
                } else try fillTrack(allocator, scene, &v.tracks[item], &s);
                try saveInto(allocator, &s, out);
            },
            .differential => {
                if (v.controller == @intFromEnum(VehicleControllerKind.tracked) or item >= v.num_differentials)
                    return false;
                var s: VehicleDifferentialSettings = .{};
                if (restore_from) |bytes| {
                    var in: RestoreStream = undefined;
                    in.init(bytes);
                    s.restoreBinaryState(in.stream());
                    if (!in.completely())
                        return false;
                } else s = fillDifferential(&scene.vehicle_differentials[v.first_differential + item]);
                try saveInto(allocator, &s, out);
            },
            .anti_roll_bar => {
                if (item >= v.num_anti_roll_bars)
                    return false;
                var s: VehicleAntiRollBar = .{};
                if (restore_from) |bytes| {
                    var in: RestoreStream = undefined;
                    in.init(bytes);
                    s.restoreBinaryState(in.stream());
                    if (!in.completely())
                        return false;
                } else s = fillAntiRollBar(&scene.vehicle_anti_roll_bars[v.first_anti_roll_bar + item]);
                try saveInto(allocator, &s, out);
            },
        }
        return true;
    }
};

// ---- SLOT tracked_vehicle: TrackedVehicleController, WheelSettingsTV (Phase7OpRange.vehicle_tracked 570..579,
// Blocker.tracked_vehicles) ----

/// The vehicle variants group, tracked vehicles: the hooks that VehicleSlot calls (SimMakeTrackedControllerSettings,
/// SimMakeWheelSettingsTV, WriteTrackedWheelState, WriteTrackedControllerState, GetEngine / GetTransmission of a
/// tracked vehicle) and ApplyTrackedOp of SimulationReference.cpp. Proof that the tracked scenes bite (each breakage of
/// Zolt/Physics/Vehicle/TrackedVehicleController.zig, then reverted): reassociating PostCollide's
/// `differential_torque * delta_time / inertia` as `differential_torque * (delta_time / inertia)` is reported as "scene
/// vehicle_tracked_ops ... update 2: after the update, state f: state differs at byte 59 ...: bodies[1] (id 0x00800001)
/// position.x zolt 0x2fd23654 jolt 0x2fcecaea"; `-=` instead of `+=` for the right track in SyncLeftRightTracks' branch
/// of opposite ratios as "scene vehicle_tracked_terrains ... update 75: ... bodies[2] (id 0x00800002) position.x zolt
/// 0xc1c6f3ff jolt 0xc1c6f2d7" (turning on the spot; VehiclePartsParity's SyncLeftRightTracks test fails too); summing
/// the radius of every wheel of a braking track instead of those with a contact as "scene vehicle_tracked_terrains ...
/// update 125: ... bodies[3] (id 0x00800003) position.x zolt 0x41cccda3 jolt 0x41cccd22" (and in
/// vehicle_tracked_pushing_boxes and vehicle_tracked_far_from_origin, the scenes that brake with wheels in the air).
const TrackedVehicleSlot = struct {
    const TrackedVehicleController = zolt.TrackedVehicleController;
    const TrackedVehicleControllerSettings = zolt.TrackedVehicleControllerSettings;
    const WheelSettingsTV = zolt.WheelSettingsTV;
    const WheelTV = zolt.WheelTV;

    /// SimMakeTrackedControllerSettings: the engine, the transmission and the tracks of a description (no reference yet)
    fn makeControllerSettings(allocator: Allocator, scene: *const SimSceneDesc, v: *const SimVehicle) !*VehicleControllerSettings {
        const s = try TrackedVehicleControllerSettings.create(allocator);
        errdefer {
            s.base.addRef();
            s.base.release();
        }
        try VehicleSlot.fillEngine(allocator, scene, &v.engine, &s.engine);
        try VehicleSlot.fillTransmission(allocator, scene, &v.transmission, &s.transmission);
        for (&s.tracks, &v.tracks) |*track, *d|
            try VehicleSlot.fillTrack(allocator, scene, d, track);
        return &s.base;
    }

    /// SimMakeWheelSettingsTV: the WheelSettingsTV of a wheel description (no reference yet)
    fn makeWheelSettings(allocator: Allocator, scene: *const SimSceneDesc, w: *const SimVehicleWheel) !*WheelSettings {
        _ = scene;
        const tv = try WheelSettingsTV.create(allocator);
        VehicleSlot.fillWheelSettings(w, &tv.base);
        tv.longitudinal_friction = w.longitudinal_friction_tv;
        tv.lateral_friction = w.lateral_friction_tv;
        return &tv.base;
    }

    /// The tracked controller of a vehicle
    fn controller(v: *VehicleConstraint) *TrackedVehicleController {
        return zolt.virtual.downcast(TrackedVehicleController, v.getControllerMut());
    }

    /// GetEngine of the .cpp for a tracked vehicle
    fn engine(v: *VehicleConstraint) !*zolt.VehicleEngine {
        return controller(v).getEngineMut();
    }

    /// GetTransmission of the .cpp for a tracked vehicle
    fn transmission(v: *VehicleConstraint) !*zolt.VehicleTransmission {
        return controller(v).getTransmissionMut();
    }

    /// ApplyTrackedOp (GetTrackedController: an operation on a vehicle of another controller kind is a scene error)
    fn applyVehicleOp(world: *ZoltWorld, op: *const SimOp, v: *VehicleConstraint) !void {
        const f = &op.f;
        if ((try VehicleSlot.getVehicle(world, op.index)).controller != .tracked) {
            std.debug.print("SimulationParity: tracked operation {d} on the vehicle handle {d}\n", .{ op.kind, op.index });
            return error.SceneError;
        }
        const c = controller(v);
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .tracked_set_driver_input => c.setDriverInput(f[0], f[1], f[2], f[3]),
            .tracked_set_forward_input => c.setForwardInput(f[0]),
            .tracked_set_left_ratio => c.setLeftRatio(f[0]),
            .tracked_set_right_ratio => c.setRightRatio(f[0]),
            .tracked_set_brake_input => c.setBrakeInput(f[0]),
            .tracked_set_track => {
                const track = &c.getTracksMut()[op.i[0]];
                track.angular_velocity = f[0];
                track.base.inertia = f[1];
                track.base.angular_damping = f[2];
                track.base.max_brake_torque = f[3];
                track.base.differential_ratio = f[4];
            },
            else => {
                std.debug.print("SimulationParity: unknown tracked vehicle operation {d}\n", .{op.kind});
                return error.SceneError;
            },
        }
    }

    /// WriteTrackedWheelState: the vehicle_wheel_tv field of a wheel
    fn writeWheelState(world: *ZoltWorld, w: *ObjectStateWriter, handle: u32, index: u32, wheel: *const Wheel) !void {
        _ = world;
        const tv = zolt.virtual.downcast(WheelTV, wheel);
        try w.begin(.vehicle_wheel_tv, handle, index);
        try w.value(tv.track_index);
        try w.value(tv.combined_longitudinal_friction);
        try w.value(tv.combined_lateral_friction);
        try w.value(tv.brake_impulse);
        w.end();
    }

    /// WriteTrackedControllerState: the engine and the transmission (VehicleSlot's functions), the inputs, the tracks
    fn writeControllerState(world: *ZoltWorld, w: *ObjectStateWriter, handle: u32, v: *const VehicleConstraint) !void {
        _ = world;
        const tracked = zolt.virtual.downcast(TrackedVehicleController, v.getController());
        try VehicleSlot.writeEngineState(w, handle, tracked.getEngine());
        try VehicleSlot.writeTransmissionState(w, handle, tracked.getTransmission());
        try w.begin(.vehicle_tracked, handle, 0);
        try w.value(tracked.getForwardInput());
        try w.value(tracked.getLeftRatio());
        try w.value(tracked.getRightRatio());
        try w.value(tracked.getBrakeInput());
        w.end();
        for (tracked.getTracks(), 0..) |*track, t| {
            try w.begin(.vehicle_track, handle, @intCast(t));
            try w.value(track.angular_velocity);
            try w.value(track.base.driven_wheel);
            try w.value(@as(u32, @intCast(track.base.wheels.items.len)));
            try w.value(track.base.inertia);
            try w.value(track.base.angular_damping);
            try w.value(track.base.max_brake_torque);
            try w.value(track.base.differential_ratio);
            w.end();
        }
    }
};

// ---- SLOT motorcycle: MotorcycleController (Phase7OpRange.vehicle_motorcycle 580..589, Blocker.motorcycles) ----

/// The vehicle variants group, motorcycles: the hooks that VehicleSlot calls (SimMakeMotorcycleControllerSettings,
/// WriteMotorcycleControllerState) and ApplyMotorcycleOp of SimulationReference.cpp. A MotorcycleController is a
/// WheeledVehicleController: VehicleSlot reaches its wheeled parts itself. Proof that the motorcycle scenes bite (each
/// breakage of Zolt/Physics/Vehicle/MotorcycleController.zig, then reverted): no lean steering limit (a max steer angle
/// 10 rad higher) is reported as "scene vehicle_motorcycle_turns ... update 60: after the update, state f: state
/// differs at byte 193 ...: bodies[2] (id 0x00800002) position.x zolt 0xba28cdaa jolt 0xba36e5cf" (and in
/// vehicle_motorcycle_slopes_and_braking); clamping the steer angle while the lean steering limit is disabled
/// (PreCollide ignoring mEnableLeanSteeringLimit) as "scene vehicle_motorcycle_turns ... update 66: ... bodies[2] (id
/// 0x00800002) position.x zolt 0xbd1e8f1f jolt 0xbd236fc2"; the gravity of the system instead of the overridden gravity
/// in the max steer angle factor as "scene vehicle_motorcycle_slopes_and_braking ... update 47: ... bodies[3] (id
/// 0x00800003) position.x zolt 0x415f15dd jolt 0x415f1658" (these two passed every scene while the limit was only
/// switched off on motorcycles too slow to be clamped and the braker was too slow for its 45 degree max lean angle);
/// clamping the target lean to the other side (`-Sign(w_angle) * mMaxLeanAngle`) as "scene vehicle_motorcycle_turns ...
/// update 62: ... bodies[2] (id 0x00800002) position.x zolt 0xbbd38af5 jolt 0xbc1755c0" (the clamp is reached; and in
/// vehicle_motorcycle_slopes_and_braking); subtracting the integration term of the lean impulse as "scene
/// vehicle_motorcycle_lean_integration ... update 13: ... bodies[1] (id 0x00800001) position.x zolt 0x3d358665 jolt
/// 0x3d35d8f6" (and in vehicle_motorcycle_lean_integration_slope); copying the controller's lean spring damping in
/// GetSettings (Jolt assigns the settings' own value) as "scene vehicle_motorcycle_ops ... update 40: operation 19
/// (vehicle_settings_hash on object handle / description 0) returned 869730697 in zolt and 4196340125 in jolt" (and
/// VehiclePartsParity's controller test).
const MotorcycleSlot = struct {
    const MotorcycleController = zolt.MotorcycleController;
    const MotorcycleControllerSettings = zolt.MotorcycleControllerSettings;

    /// SimMakeMotorcycleControllerSettings: the wheeled members (VehicleSlot.fillWheeled), then the lean members (no
    /// reference yet)
    fn makeControllerSettings(allocator: Allocator, scene: *const SimSceneDesc, v: *const SimVehicle) !*VehicleControllerSettings {
        const s = try MotorcycleControllerSettings.create(allocator);
        errdefer {
            s.asControllerSettings().addRef();
            s.asControllerSettings().release();
        }
        try VehicleSlot.fillWheeled(scene, v, &s.base);
        s.max_lean_angle = v.max_lean_angle;
        s.lean_spring_constant = v.lean_spring_constant;
        s.lean_spring_damping = v.lean_spring_damping;
        s.lean_spring_integration_coefficient = v.lean_spring_integration_coefficient;
        s.lean_spring_integration_coefficient_decay = v.lean_spring_integration_coefficient_decay;
        s.lean_smoothing_factor = v.lean_smoothing_factor;
        return s.asControllerSettings();
    }

    /// The motorcycle controller of a vehicle
    fn controller(v: *VehicleConstraint) *MotorcycleController {
        return zolt.virtual.downcast(MotorcycleController, v.getControllerMut());
    }

    /// ApplyMotorcycleOp (GetMotorcycleController: an operation on a vehicle of another controller kind is a scene error)
    fn applyVehicleOp(world: *ZoltWorld, op: *const SimOp, v: *VehicleConstraint) !void {
        const f = &op.f;
        if ((try VehicleSlot.getVehicle(world, op.index)).controller != .motorcycle) {
            std.debug.print("SimulationParity: motorcycle operation {d} on the vehicle handle {d}\n", .{ op.kind, op.index });
            return error.SceneError;
        }
        const c = controller(v);
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .motorcycle_enable_lean_controller => c.enableLeanController(op.i[0] != 0),
            .motorcycle_enable_lean_steering_limit => c.enableLeanSteeringLimit(op.i[0] != 0),
            .motorcycle_set_lean_spring_constant => c.setLeanSpringConstant(f[0]),
            .motorcycle_set_lean_spring_damping => c.setLeanSpringDamping(f[0]),
            .motorcycle_set_lean_spring_integration_coefficient => c.setLeanSpringIntegrationCoefficient(f[0]),
            .motorcycle_set_lean_spring_integration_coefficient_decay => c.setLeanSpringIntegrationCoefficientDecay(f[0]),
            .motorcycle_set_lean_smoothing_factor => c.setLeanSmoothingFactor(f[0]),
            else => {
                std.debug.print("SimulationParity: unknown motorcycle operation {d}\n", .{op.kind});
                return error.SceneError;
            },
        }
    }

    /// WriteMotorcycleControllerState: the vehicle_motorcycle field (after the wheeled fields that VehicleSlot writes)
    fn writeControllerState(world: *ZoltWorld, w: *ObjectStateWriter, handle: u32, v: *const VehicleConstraint) !void {
        _ = world;
        const motorcycle = zolt.virtual.downcast(MotorcycleController, v.getController());
        try w.begin(.vehicle_motorcycle, handle, 0);
        try w.value(motorcycle.getWheelBase());
        try w.boolean(motorcycle.isLeanControllerEnabled());
        try w.boolean(motorcycle.isLeanSteeringLimitEnabled());
        try w.value(motorcycle.getLeanSpringConstant());
        try w.value(motorcycle.getLeanSpringDamping());
        try w.value(motorcycle.getLeanSpringIntegrationCoefficient());
        try w.value(motorcycle.getLeanSpringIntegrationCoefficientDecay());
        try w.value(motorcycle.getLeanSmoothingFactor());
        w.end();
    }
};

// ---- end of the Phase 7 slots ----

/// The objects of the Phase 7 slots (ZoltWorld.phase7; the vehicle variants keep their vehicles in VehicleSlot's)
const Phase7Storage = struct {
    characters: CharacterSlot.Storage = .{},
    character_virtuals: CharacterVirtualSlot.Storage = .{},
    ragdolls: RagdollSlot.Storage = .{},
    vehicles: VehicleSlot.Storage = .{},
};

/// A record in a failure message (`{f}`): the kind by name, the ids and the data; "nothing" when a side has no record
const RecordText = struct {
    record: ?SimRecord,

    pub fn format(self: RecordText, w: *std.Io.Writer) std.Io.Writer.Error!void {
        const r = self.record orelse return w.writeAll("nothing");
        if (std.enums.fromInt(RecordKind, r.kind)) |kind|
            try w.print("{t}", .{kind})
        else
            try w.print("kind {d}", .{r.kind});
        try w.print(" ids {any} data {any}", .{ r.ids, r.data });
    }
};

/// The CharacterContactListener records of the Zolt side (SimCharacterContactListener of SimulationReference.cpp, the
/// layouts of RecordKind 7..17): the listener of CharacterVirtualSlot adds them to world.character_recorder
const CharacterRecords = struct {
    fn vec3Bits(v: Vec3) [3]u32 {
        return .{ floatBits(v.getX()), floatBits(v.getY()), floatBits(v.getZ()) };
    }

    fn hashVec3(hash: u32, v: Vec3) u32 {
        const values = [3]f32{ v.getX(), v.getY(), v.getZ() };
        return SimScenes.hashBytes(hash, std.mem.asBytes(&values));
    }

    fn hashRVec3(hash: u32, v: RVec3) u32 {
        const values = [3]Real{ v.getX(), v.getY(), v.getZ() };
        return SimScenes.hashBytes(hash, std.mem.asBytes(&values));
    }

    /// The flags of a contact record (SimCharacterContactFlags): mMotionTypeB | mIsSensorB << 8 | mHadCollision << 9 |
    /// mWasDiscarded << 10 | mCanPushCharacter << 11 | mIsBackFacingContact << 12 | (mCharacterB != null) << 13
    pub fn contactFlags(motion_type_b: u32, is_sensor_b: bool, had_collision: bool, was_discarded: bool, can_push_character: bool, is_back_facing_contact: bool, has_character_b: bool) u32 {
        return motion_type_b |
            (@as(u32, @intFromBool(is_sensor_b)) << 8) |
            (@as(u32, @intFromBool(had_collision)) << 9) |
            (@as(u32, @intFromBool(was_discarded)) << 10) |
            (@as(u32, @intFromBool(can_push_character)) << 11) |
            (@as(u32, @intFromBool(is_back_facing_contact)) << 12) |
            (@as(u32, @intFromBool(has_character_b)) << 13);
    }

    /// The bits of CharacterContactSettings (SimCharacterContactSettingsBits): mCanPushCharacter | mCanReceiveImpulses << 1
    pub fn settingsBits(can_push_character: bool, can_receive_impulses: bool) u32 {
        return @as(u32, @intFromBool(can_push_character)) | (@as(u32, @intFromBool(can_receive_impulses)) << 1);
    }

    /// A contact record (validate / added / persisted and the character versions): `flags` from contactFlags, `settings_in`
    /// the settingsBits passed in (added / persisted, else 0), `hash` SimScenes.hashCharacterContact of the contact,
    /// `result` the return value of validate (1 / 0) or the settingsBits after the callback
    pub fn contact(kind: RecordKind, character_id: u32, body_b: BodyID, character_id_b: u32, sub_shape_id_b: SubShapeID, contact_normal: Vec3, distance: f32, fraction: f32, flags: u32, settings_in: u32, hash: u32, result: u32) SimRecord {
        const n = vec3Bits(contact_normal);
        return .{
            .kind = @intFromEnum(kind),
            .ids = .{ character_id, body_b.getIndexAndSequenceNumber(), character_id_b, sub_shape_id_b.getValue() },
            .data = .{ n[0], n[1], n[2], floatBits(distance), floatBits(fraction), flags | (settings_in << 16), hash, result },
        };
    }

    /// OnContactRemoved (body_id2, invalid character ID) / OnCharacterContactRemoved (invalid BodyID, character_id2)
    pub fn removed(kind: RecordKind, character_id: u32, body_id2: BodyID, character_id2: u32, sub_shape_id2: SubShapeID) SimRecord {
        return .{ .kind = @intFromEnum(kind), .ids = .{ character_id, body_id2.getIndexAndSequenceNumber(), character_id2, sub_shape_id2.getValue() } };
    }

    /// OnContactSolve (body_id2, invalid character ID) / OnCharacterContactSolve (invalid BodyID, the other character's
    /// ID): the new velocity as passed in and after the callback
    pub fn solve(kind: RecordKind, character_id: u32, body_id2: BodyID, character_id2: u32, sub_shape_id2: SubShapeID, contact_position: RVec3, contact_normal: Vec3, contact_velocity: Vec3, material_hash: u32, character_velocity: Vec3, new_velocity_in: Vec3, new_velocity_out: Vec3) SimRecord {
        const v_in = vec3Bits(new_velocity_in);
        const v_character = vec3Bits(character_velocity);
        var h = hashRVec3(SimScenes.hash_seed, contact_position);
        h = hashVec3(h, contact_normal);
        h = hashVec3(h, contact_velocity);
        h = SimScenes.hashBytes(h, std.mem.asBytes(&material_hash));
        return .{
            .kind = @intFromEnum(kind),
            .ids = .{ character_id, body_id2.getIndexAndSequenceNumber(), character_id2, sub_shape_id2.getValue() },
            .data = .{ v_in[0], v_in[1], v_in[2], v_character[0], v_character[1], v_character[2], h, hashVec3(SimScenes.hash_seed, new_velocity_out) },
        };
    }

    /// OnAdjustBodyVelocity: the velocities as passed in and after the callback
    pub fn adjustBodyVelocity(character_id: u32, body_id2: BodyID, linear_in: Vec3, angular_in: Vec3, linear_out: Vec3, angular_out: Vec3) SimRecord {
        const l = vec3Bits(linear_in);
        const a = vec3Bits(angular_in);
        return .{
            .kind = @intFromEnum(RecordKind.character_adjust_body_velocity),
            .ids = .{ character_id, body_id2.getIndexAndSequenceNumber(), 0, 0 },
            .data = .{ l[0], l[1], l[2], a[0], a[1], a[2], hashVec3(hashVec3(SimScenes.hash_seed, linear_out), angular_out), 0 },
        };
    }
};

/// The step callback records of a vehicle (SimVehicleRecord of SimulationReference.cpp, RecordKind 18..20)
const VehicleRecords = struct {
    /// The hash of the wheels of a record: per wheel in order (from SimScenes.hash_seed) GetSuspensionLength,
    /// GetAngularVelocity and GetSteerAngle
    pub fn hashWheel(hash: u32, suspension_length: f32, angular_velocity: f32, steer_angle: f32) u32 {
        const values = [3]f32{ suspension_length, angular_velocity, steer_angle };
        return SimScenes.hashBytes(hash, std.mem.asBytes(&values));
    }

    /// A record of a vehicle callback: `wheel_contacts` the bits of the wheels with HasContact(), `wheel_hash` from hashWheel
    pub fn record(kind: RecordKind, handle: u32, body_id: BodyID, context: *const PhysicsStepListenerContext, wheel_contacts: u32, wheel_hash: u32) SimRecord {
        return .{
            .kind = @intFromEnum(kind),
            .ids = .{ handle, body_id.getIndexAndSequenceNumber(), 0, 0 },
            .data = .{ floatBits(context.delta_time), @intFromBool(context.is_first_step), @intFromBool(context.is_last_step), wheel_contacts, wheel_hash, 0, 0, 0 },
        };
    }
};

/// Before a scene runs: the Phase 7 classes that it creates are ported (a supported Blocker must cover them) and their
/// slots are no longer stubs
fn checkPhase7Slots(entry: *const SceneEntry, builder: *const SceneBuilder) !void {
    if (builder.characters.items.len > 0)
        try checkPhase7Slot(entry, "Character", Phase7Classes.characters(), !@hasDecl(CharacterSlot, "is_stub"));
    if (builder.character_virtuals.items.len > 0)
        try checkPhase7Slot(entry, "CharacterVirtual", Phase7Classes.characterVirtuals(), !@hasDecl(CharacterVirtualSlot, "is_stub"));
    if (builder.ragdolls.items.len > 0)
        try checkPhase7Slot(entry, "Ragdoll", Phase7Classes.ragdolls(), !@hasDecl(RagdollSlot, "is_stub"));
    for (builder.vehicles.items) |*v| switch (@as(VehicleControllerKind, @enumFromInt(v.controller))) {
        .wheeled => try checkPhase7Slot(entry, "wheeled vehicle", Phase7Classes.wheeledVehicles(), !@hasDecl(VehicleSlot, "is_stub")),
        .tracked => try checkPhase7Slot(entry, "tracked vehicle", Phase7Classes.trackedVehicles(), !@hasDecl(VehicleSlot, "is_stub") and !@hasDecl(TrackedVehicleSlot, "is_stub")),
        .motorcycle => try checkPhase7Slot(entry, "motorcycle", Phase7Classes.motorcycles(), !@hasDecl(VehicleSlot, "is_stub") and !@hasDecl(MotorcycleSlot, "is_stub")),
    };
}

/// Byte buffers of the settings checks of Phase 7
const SettingsBuffers = struct {
    allocator: Allocator,
    zolt: std.ArrayList(u8) = .empty,
    jolt: std.ArrayList(u8) = .empty,
    zolt_round_trip: std.ArrayList(u8) = .empty,
    jolt_round_trip: std.ArrayList(u8) = .empty,

    fn deinit(self: *SettingsBuffers) void {
        self.zolt.deinit(self.allocator);
        self.jolt.deinit(self.allocator);
        self.zolt_round_trip.deinit(self.allocator);
        self.jolt_round_trip.deinit(self.allocator);
    }
};

/// Phase 7, before a scene runs (the foundation review M3): the binary state of the settings that the filled Zolt slots
/// make from the scene's vehicle and ragdoll descriptions is the C++ one, each side restores the other side's bytes and
/// saves them unchanged (every round-tripping VehicleSettingsPart; RagdollSettings with the shapes and the group
/// filter), and the tables of the RagdollSettings are the C++ ones. A mismatch is an error of the slot or of the
/// library's binary state functions, reported before the simulation.
fn checkPhase7Settings(allocator: Allocator, entry: *const SceneEntry, builder: *const SceneBuilder) !void {
    const desc = builder.desc();
    var buffers: SettingsBuffers = .{ .allocator = allocator };
    defer buffers.deinit();

    if (!@hasDecl(VehicleSlot, "is_stub")) {
        for (builder.vehicles.items, 0..) |*v, i| {
            const index: u32 = @intCast(i);
            for (std.enums.values(VehicleSettingsPart)) |part| {
                for (0..part.numItems(v)) |n| {
                    const item: u32 = @intCast(n);
                    const has_zolt = try VehicleSlot.settingsPart(allocator, &desc, index, part, item, null, &buffers.zolt);
                    const has_jolt = try CppWorld.vehicleSettingsPart(allocator, &desc, index, part, item, null, &buffers.jolt);
                    if (!has_zolt or !has_jolt or !std.mem.eql(u8, buffers.zolt.items, buffers.jolt.items)) {
                        std.debug.print("SimulationParity: scene {s}, vehicle description {d}, part {t}, item {d}: the binary state of the Zolt slot differs from SimVehicleSettingsPart (byte {?d} of {d})\n", .{ entry.name, index, part, item, std.mem.indexOfDiff(u8, buffers.zolt.items, buffers.jolt.items), buffers.jolt.items.len });
                        return error.ParityMismatch;
                    }
                    if (!part.roundTrips()) continue;
                    const zolt_restored = try VehicleSlot.settingsPart(allocator, &desc, index, part, item, buffers.jolt.items, &buffers.zolt_round_trip);
                    const jolt_restored = try CppWorld.vehicleSettingsPart(allocator, &desc, index, part, item, buffers.zolt.items, &buffers.jolt_round_trip);
                    if (!zolt_restored or !std.mem.eql(u8, buffers.zolt_round_trip.items, buffers.jolt.items)) {
                        std.debug.print("SimulationParity: scene {s}, vehicle description {d}, part {t}, item {d}: Zolt does not restore the C++ bytes (restored {}, saved again: byte {?d} differs)\n", .{ entry.name, index, part, item, zolt_restored, std.mem.indexOfDiff(u8, buffers.zolt_round_trip.items, buffers.jolt.items) });
                        return error.ParityMismatch;
                    }
                    if (!jolt_restored or !std.mem.eql(u8, buffers.jolt_round_trip.items, buffers.zolt.items)) {
                        std.debug.print("SimulationParity: scene {s}, vehicle description {d}, part {t}, item {d}: Jolt does not restore the Zolt bytes (restored {}, saved again: byte {?d} differs)\n", .{ entry.name, index, part, item, jolt_restored, std.mem.indexOfDiff(u8, buffers.jolt_round_trip.items, buffers.zolt.items) });
                        return error.ParityMismatch;
                    }
                }
            }
        }
    }

    if (builder.ragdolls.items.len > 0 and !@hasDecl(RagdollSlot, "is_stub")) {
        // The parts of a ragdoll use the shapes of a world
        const cw = CppWorld.init(&desc, 0);
        defer cw.deinit();
        const zw = try ZoltWorld.create(allocator, &desc, .single_threaded);
        defer zw.destroy();
        var zolt_info: std.ArrayList(u32) = .empty;
        defer zolt_info.deinit(allocator);
        var jolt_info: std.ArrayList(u32) = .empty;
        defer jolt_info.deinit(allocator);
        for (0..builder.ragdolls.items.len) |i| {
            const index: u32 = @intCast(i);
            const has_zolt = try RagdollSlot.settingsBytes(zw, index, null, &buffers.zolt);
            const has_jolt = try cw.ragdollSettingsBytes(allocator, index, null, &buffers.jolt);
            if (!has_zolt or !has_jolt or !std.mem.eql(u8, buffers.zolt.items, buffers.jolt.items)) {
                std.debug.print("SimulationParity: scene {s}, ragdoll description {d}: the binary state of the Zolt RagdollSettings differs (byte {?d} of {d})\n", .{ entry.name, index, std.mem.indexOfDiff(u8, buffers.zolt.items, buffers.jolt.items), buffers.jolt.items.len });
                return error.ParityMismatch;
            }
            const zolt_restored = try RagdollSlot.settingsBytes(zw, index, buffers.jolt.items, &buffers.zolt_round_trip);
            const jolt_restored = try cw.ragdollSettingsBytes(allocator, index, buffers.zolt.items, &buffers.jolt_round_trip);
            if (!zolt_restored or !jolt_restored or !std.mem.eql(u8, buffers.zolt_round_trip.items, buffers.jolt.items) or !std.mem.eql(u8, buffers.jolt_round_trip.items, buffers.zolt.items)) {
                std.debug.print("SimulationParity: scene {s}, ragdoll description {d}: the RagdollSettings round trips differ (Zolt restored {}, Jolt restored {})\n", .{ entry.name, index, zolt_restored, jolt_restored });
                return error.ParityMismatch;
            }
            try RagdollSlot.settingsInfo(zw, index, &zolt_info);
            try cw.ragdollSettingsInfo(allocator, index, &jolt_info);
            if (!std.mem.eql(u32, zolt_info.items, jolt_info.items)) {
                std.debug.print("SimulationParity: scene {s}, ragdoll description {d}: the RagdollSettings tables differ (Stabilize, GetBodyIndexToConstraintIndex, GetConstraintIndexToBodyIdxPair)\n  zolt {any}\n  jolt {any}\n", .{ entry.name, index, zolt_info.items, jolt_info.items });
                return error.ParityMismatch;
            }
        }
    }
}

// -------------------------------------------------------------------------------------------------------------------
// Soft bodies (Phase 9, group SH; see "Soft bodies" in the SimScenes header): the Zolt side of the soft body
// descriptions, operations and listener records, the same code as the soft body parts of SimulationReference.cpp

/// The soft body objects of a world: the shared settings of every SimSoftBodySettings (made once, the soft bodies of a
/// description share them) and the soft body contact listener
const SoftBodyStorage = struct {
    settings: std.ArrayList(Ref(zolt.SoftBodySharedSettings)) = .empty,
    listener: SoftBodyListenerImpl = undefined,

    fn deinit(self: *SoftBodyStorage, allocator: Allocator) void {
        for (self.settings.items) |*s| s.deinit();
        self.settings.deinit(allocator);
    }
};

/// FNV-1a over the bytes of a value (the hashes of the soft body operations)
fn hashValue(hash: u32, value: anytype) u32 {
    return SimScenes.hashBytes(hash, std.mem.asBytes(&value));
}

/// SimSoftBodyContactListener of SimulationReference.cpp: records the callbacks (RecordKind 70..73, called from the soft
/// body jobs), then OnSoftBodyContactValidate acts in its SoftBodyListenerMode
const SoftBodyListenerImpl = struct {
    recorder: *Recorder,
    mode: SimScenes.SoftBodyListenerMode = .none,

    fn settingsBits(settings: *const zolt.SoftBodyContactSettings) [4]u32 {
        return .{ floatBits(settings.inv_mass_scale1), floatBits(settings.inv_mass_scale2), floatBits(settings.inv_inertia_scale2), @intFromBool(settings.is_sensor) };
    }

    pub fn onSoftBodyContactValidate(self: *SoftBodyListenerImpl, soft_body: *const Body, other_body: *const Body, settings: *zolt.SoftBodyContactSettings) zolt.SoftBodyValidateResult {
        var r: SimRecord = .{ .kind = @intFromEnum(RecordKind.soft_body_contact_validate) };
        r.ids[0] = soft_body.getID().getIndexAndSequenceNumber();
        r.ids[1] = other_body.getID().getIndexAndSequenceNumber();
        r.data[0..4].* = settingsBits(settings);
        var result: zolt.SoftBodyValidateResult = .accept_contact;
        switch (self.mode) {
            .reject => result = .reject_contact,
            .sensor => settings.is_sensor = true,
            .mass_scales => {
                settings.inv_mass_scale1 = 0.5;
                settings.inv_mass_scale2 = 2.0;
                settings.inv_inertia_scale2 = 1.5;
            },
            .none, .accept => {},
        }
        r.ids[2] = @intFromBool(result != .accept_contact);
        r.data[4..8].* = settingsBits(settings);
        self.recorder.add(r);
        return result;
    }

    pub fn onSoftBodyContactAdded(self: *SoftBodyListenerImpl, soft_body: *const Body, manifold: *const zolt.SoftBodyManifold) void {
        const id = soft_body.getID().getIndexAndSequenceNumber();
        const vertices = manifold.getVertices();

        // The call
        var r: SimRecord = .{ .kind = @intFromEnum(RecordKind.soft_body_contact_added) };
        var hash = SimScenes.hash_seed;
        var num_contacts: u32 = 0;
        for (vertices) |*v| {
            const has_contact: u32 = @intFromBool(manifold.hasContact(v));
            hash = hashValue(hash, has_contact);
            num_contacts += has_contact;
        }
        r.ids = .{ id, @intCast(vertices.len), num_contacts, manifold.getNumSensorContacts() };
        r.data[0] = hash;
        self.recorder.add(r);

        // One record per body that vertices have a contact with (its first vertex makes the record)
        for (vertices, 0..) |*v, first| {
            if (!manifold.hasContact(v)) continue;
            const body_id = manifold.getContactBodyID(v);
            const seen = for (vertices[0..first]) |*w| {
                if (manifold.hasContact(w) and manifold.getContactBodyID(w).eql(body_id)) break true;
            } else false;
            if (seen) continue;
            var b: SimRecord = .{ .kind = @intFromEnum(RecordKind.soft_body_contact_body) };
            var body_hash = SimScenes.hash_seed;
            var count: u32 = 0;
            for (vertices[first..]) |*w| {
                if (manifold.hasContact(w) and manifold.getContactBodyID(w).eql(body_id)) {
                    const p = manifold.getLocalContactPoint(w);
                    const n = manifold.getContactNormal(w);
                    body_hash = hashValue(body_hash, [6]f32{ p.getX(), p.getY(), p.getZ(), n.getX(), n.getY(), n.getZ() });
                    count += 1;
                }
            }
            b.ids = .{ id, body_id.getIndexAndSequenceNumber(), count, 0 };
            b.data[0] = body_hash;
            self.recorder.add(b);
        }

        // The sensor contacts (the order of the sensors depends on the broad phase: not recorded)
        for (0..manifold.getNumSensorContacts()) |i| {
            var s: SimRecord = .{ .kind = @intFromEnum(RecordKind.soft_body_sensor_contact) };
            s.ids[0] = id;
            s.ids[1] = manifold.getSensorContactBodyID(@intCast(i)).getIndexAndSequenceNumber();
            self.recorder.add(s);
        }
    }
};

/// The soft body operations and the generators of the shared settings (the soft body parts of SimulationReference.cpp)
const SoftBodySlot = struct {
    const S = zolt.SoftBodySharedSettings;
    const SimSoftBodySettings = SimScenes.SimSoftBodySettings;

    /// SimMakeSoftBodySettings of SimulationReference.cpp: the shared settings of a SimSoftBodySettings
    /// (SoftBodySettingsKind)
    fn makeSettings(allocator: Allocator, desc: *const SimSceneDesc, s: *const SimSoftBodySettings) !Ref(S) {
        const attributes = [1]S.VertexAttributes{.init(s.compliance, s.shear_compliance, s.bend_compliance, .{ .lra_type = @enumFromInt(s.lra_type), .lra_max_distance_multiplier = s.lra_max_distance_multiplier })};
        const bend_type: S.BendType = @enumFromInt(s.bend_type);
        const kind: SimScenes.SoftBodySettingsKind = @enumFromInt(s.kind);
        if (kind == .cube)
            return S.createCube(allocator, s.count_x, s.spacing);

        var ref: Ref(S) = .init(try S.create(allocator));
        errdefer ref.deinit();
        const settings = ref.get().?;
        switch (kind) {
            .cube => unreachable,

            .cloth, .skinned_cloth => {
                try addCloth(settings, s);
                try settings.createConstraints(&attributes, .{ .bend_type = bend_type });
                if (s.optimize != 0)
                    try settings.optimize();
                if (kind == .skinned_cloth) {
                    try addSkin(settings, s);
                    try settings.calculateSkinnedConstraintNormals();
                    if (s.optimize != 0)
                        try settings.optimize();
                }
            },

            // SoftBodyCreator::CreateSphere
            .sphere => {
                const radius = s.spacing;
                const num_theta = s.count_x;
                const num_phi = s.count_z;
                var v: S.Vertex = .{};
                Vec3.unitSpherical(0, 0).mulScalar(radius).storeFloat3(&v.position);
                try settings.vertices.append(allocator, v);
                Vec3.unitSpherical(zolt.math.pi, 0).mulScalar(radius).storeFloat3(&v.position);
                try settings.vertices.append(allocator, v);
                for (1..num_theta - 1) |theta| {
                    for (0..num_phi) |phi| {
                        const t = zolt.math.pi * @as(f32, @floatFromInt(theta)) / @as(f32, @floatFromInt(num_theta - 1));
                        const p = 2.0 * zolt.math.pi * @as(f32, @floatFromInt(phi)) / @as(f32, @floatFromInt(num_phi));
                        Vec3.unitSpherical(t, p).mulScalar(radius).storeFloat3(&v.position);
                        try settings.vertices.append(allocator, v);
                    }
                }
                const vertexIndex = struct {
                    fn get(n_theta: u32, n_phi: u32, theta: u32, phi: u32) u32 {
                        if (theta == 0) return 0;
                        if (theta == n_theta - 1) return 1;
                        return 2 + (theta - 1) * n_phi + phi % n_phi;
                    }
                }.get;
                for (0..num_phi) |phi_index| {
                    const phi: u32 = @intCast(phi_index);
                    for (0..num_theta - 2) |theta_index| {
                        const theta: u32 = @intCast(theta_index);
                        var f: S.Face = .init(vertexIndex(num_theta, num_phi, theta, phi), vertexIndex(num_theta, num_phi, theta + 1, phi), vertexIndex(num_theta, num_phi, theta + 1, phi + 1), .{});
                        try settings.addFace(f);
                        if (theta > 0) {
                            f.vertex[1] = vertexIndex(num_theta, num_phi, theta + 1, phi + 1);
                            f.vertex[2] = vertexIndex(num_theta, num_phi, theta, phi + 1);
                            try settings.addFace(f);
                        }
                    }
                    try settings.addFace(.init(vertexIndex(num_theta, num_phi, num_theta - 2, phi + 1), vertexIndex(num_theta, num_phi, num_theta - 2, phi), vertexIndex(num_theta, num_phi, num_theta - 1, 0), .{}));
                }
                try settings.createConstraints(&attributes, .{ .bend_type = bend_type });
                if (s.optimize != 0)
                    try settings.optimize();
            },

            // The helix of SoftBodyCosseratRodConstraintTest
            .rod_chain => {
                for (0..s.count_x) |index| {
                    const i: u32 = @intCast(index);
                    const fraction = @as(f32, @floatFromInt(i)) / @as(f32, @floatFromInt(s.count_x - 1));
                    const alpha = 4.0 * zolt.math.pi * fraction;
                    const inv_mass: f32 = if (i == 0 and s.kinematic != 0) 0.0 else 1.0;
                    try settings.vertices.append(allocator, .init(.{ .x = s.spacing * zolt.trigonometry.sin(alpha), .y = s.height * (0.5 - fraction), .z = s.spacing * zolt.trigonometry.cos(alpha) }, .{ .inv_mass = inv_mass }));
                    if (i > 0)
                        try settings.rod_stretch_shear_constraints.append(allocator, .init(i - 1, i, .{ .compliance = s.compliance }));
                    if (i > 1)
                        try settings.rod_bend_twist_constraints.append(allocator, .init(i - 2, i - 1, .{ .compliance = s.shear_compliance }));
                }
                try settings.calculateRodProperties();
                if (s.optimize != 0)
                    try settings.optimize();
            },

            .explicit => {
                const f = desc.floats[s.floats..][0 .. 7 * @as(usize, s.count_x)];
                for (0..s.count_x) |i| {
                    const p = f[7 * i ..][0..7];
                    try settings.vertices.append(allocator, .init(.{ .x = p[0], .y = p[1], .z = p[2] }, .{ .velocity = .{ .x = p[3], .y = p[4], .z = p[5] }, .inv_mass = p[6] }));
                }
                const u = desc.uints[s.uints..][0 .. 3 * @as(usize, s.count_z) + 4 * @as(usize, s.count_extra)];
                for (0..s.count_z) |i|
                    try settings.addFace(.init(u[3 * i], u[3 * i + 1], u[3 * i + 2], .{}));
                const t = u[3 * @as(usize, s.count_z) ..];
                for (0..s.count_extra) |i|
                    try settings.volume_constraints.append(allocator, .init(t[4 * i], t[4 * i + 1], t[4 * i + 2], t[4 * i + 3], .{ .compliance = s.compliance }));
                if (settings.faces.items.len > 0)
                    try settings.createConstraints(&attributes, .{ .bend_type = bend_type });
                if (settings.volume_constraints.items.len > 0)
                    settings.calculateVolumeConstraintVolumes();
                if (s.optimize != 0)
                    try settings.optimize();
            },
        }
        return ref;
    }

    /// SimAddCloth of SimulationReference.cpp: the vertices and faces of a cloth (SoftBodyCreator::CreateCloth)
    fn addCloth(settings: *S, s: *const SimSoftBodySettings) !void {
        const allocator = settings.allocator;
        const offset_x = -0.5 * s.spacing * @as(f32, @floatFromInt(s.count_x - 1));
        const offset_z = -0.5 * s.spacing * @as(f32, @floatFromInt(s.count_z - 1));
        for (0..s.count_z) |z| {
            for (0..s.count_x) |x| {
                const first_or_last_row = z == 0 or z == s.count_z - 1;
                const kinematic = if (s.kinematic == 1) first_or_last_row and (x == 0 or x == s.count_x - 1) else s.kinematic == 2 and first_or_last_row;
                const position: zolt.Float3 = .{ .x = offset_x + @as(f32, @floatFromInt(x)) * s.spacing, .y = 0.0, .z = offset_z + @as(f32, @floatFromInt(z)) * s.spacing };
                try settings.vertices.append(allocator, .init(position, .{ .inv_mass = if (kinematic) 0.0 else 1.0 }));
            }
        }
        for (0..s.count_z - 1) |z| {
            for (0..s.count_x - 1) |x| {
                const i: u32 = @intCast(x + z * s.count_x);
                try settings.addFace(.init(i, i + s.count_x, i + s.count_x + 1, .{}));
                try settings.addFace(.init(i, i + s.count_x + 1, i + 1, .{}));
            }
        }
    }

    /// SimAddSkin of SimulationReference.cpp: the joints, the inverse bind matrices and the skinned constraints of a
    /// skinned cloth (SoftBodySkinnedConstraintTest::Initialize)
    fn addSkin(settings: *S, s: *const SimSoftBodySettings) !void {
        const allocator = settings.allocator;
        const num_joints = s.count_extra;
        const offset_z = -0.5 * s.spacing * @as(f32, @floatFromInt(s.count_z - 1));
        const step = @as(f32, @floatFromInt(s.count_z - 1)) * s.spacing / @as(f32, @floatFromInt(num_joints - 1));
        const joint_z = try allocator.alloc(f32, num_joints);
        defer allocator.free(joint_z);
        for (joint_z, 0..) |*z, j| {
            z.* = offset_z + @as(f32, @floatFromInt(j)) * step;
            try settings.inv_bind_matrices.append(allocator, .init(@intCast(j), Mat44.translation(Vec3.init(0, 0, z.*)).inversed()));
        }
        for (0..settings.vertices.items.len) |v| {
            const vertex = settings.vertices.items[v];
            var skinned: S.Skinned = .init(@intCast(v), if (vertex.inv_mass > 0.0) s.skin_max_distance else 0.0, s.back_stop_distance, s.back_stop_radius);

            // Find the two closest joints
            var closest: u32 = 0;
            var prev_closest: u32 = 0;
            var closest_dist: f32 = zolt.math.flt_max;
            var prev_closest_dist: f32 = zolt.math.flt_max;
            for (joint_z, 0..) |jz, j| {
                const dist = @abs(vertex.position.z - jz);
                if (dist < closest_dist) {
                    prev_closest = closest;
                    prev_closest_dist = closest_dist;
                    closest = @intCast(j);
                    closest_dist = dist;
                } else if (dist < prev_closest_dist) {
                    prev_closest = @intCast(j);
                    prev_closest_dist = dist;
                }
            }
            if (closest_dist == 0.0) {
                // Hard skin to the closest joint
                skinned.weights[0] = .init(closest, 1.0);
            } else {
                // Skin to the two closest joints
                skinned.weights[0] = .init(closest, 1.0 / closest_dist);
                skinned.weights[1] = .init(prev_closest, 1.0 / prev_closest_dist);
                skinned.normalizeWeights();
            }
            try settings.skinned_constraints.append(allocator, skinned);
        }
    }

    /// MakeSoftBodyCreationSettings of SimulationReference.cpp (the caller deinits the result)
    fn makeCreationSettings(zw: *const ZoltWorld, b: *const SimScenes.SimSoftBody) zolt.SoftBodyCreationSettings {
        var s: zolt.SoftBodyCreationSettings = .init(zw.soft_body.settings.items[b.settings].get().?, loadR3(&b.position), loadQuat(&b.rotation), @intCast(b.object_layer));
        s.user_data = b.user_data;
        s.collision_group = .init(null, b.collision_group_id, b.collision_sub_group_id);
        s.num_iterations = b.num_iterations;
        s.linear_damping = b.linear_damping;
        s.max_linear_velocity = b.max_linear_velocity;
        s.restitution = b.restitution;
        s.friction = b.friction;
        s.pressure = b.pressure;
        s.gravity_factor = b.gravity_factor;
        s.vertex_radius = b.vertex_radius;
        s.update_position = b.update_position != 0;
        s.make_rotation_identity = b.make_rotation_identity != 0;
        s.allow_sleeping = b.allow_sleeping != 0;
        s.faces_double_sided = b.faces_double_sided != 0;
        return s;
    }

    /// SimHashSoftBody of SimulationReference.cpp: the result of the soft body operations 56..67 (the hash of
    /// Body::SaveState, then of the values that the state does not hold)
    fn hashSoftBody(allocator: Allocator, body: *const Body) !u32 {
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        body.saveState(recorder.recorder());
        if (recorder.isFailed()) return error.OutOfMemory;
        var h = SimScenes.hashBytes(SimScenes.hash_seed, recorder.getData());
        const mp = zolt.SoftBodyMotionProperties.fromBaseConst(body.getMotionProperties());
        h = hashValue(h, mp.getNumIterations());
        h = hashValue(h, mp.getPressure());
        h = hashValue(h, @as(u32, @intFromBool(mp.getUpdatePosition())));
        h = hashValue(h, @as(u32, @intFromBool(mp.getFacesDoubleSided())));
        h = hashValue(h, @as(u32, @intFromBool(mp.getEnableSkinConstraints())));
        h = hashValue(h, mp.getSkinnedMaxDistanceMultiplier());
        h = hashValue(h, mp.getVertexRadius());
        for (mp.getVertices()) |*v|
            h = hashValue(h, v.inv_mass);
        return h;
    }

    /// HashSoftBodyCreationSettings of SimulationReference.cpp: the result of soft_body_creation_settings_hash
    fn hashCreationSettings(zw: *const ZoltWorld, s: *const zolt.SoftBodyCreationSettings) !u32 {
        var h = SimScenes.hash_seed;
        {
            var out: std.Io.Writer.Allocating = .init(zw.allocator);
            defer out.deinit();
            var wrapper = zolt.StreamOutWrapper.init(&out.writer);
            s.saveBinaryState(wrapper.streamOut());
            if (wrapper.isFailed()) return error.OutOfMemory;
            h = SimScenes.hashBytes(h, out.written());
        }
        h = hashValue(h, @as(u32, @intFromBool(s.collision_group.getGroupFilter() != null)));
        const shared = s.settings.get().?;
        var index: u32 = invalid_handle;
        for (zw.soft_body.settings.items, 0..) |ref, i| {
            if (@intFromPtr(ref.get().?) == @intFromPtr(shared)) {
                index = @intCast(i);
                break;
            }
        }
        h = hashValue(h, index);
        var out: std.Io.Writer.Allocating = .init(zw.allocator);
        defer out.deinit();
        var wrapper = zolt.StreamOutWrapper.init(&out.writer);
        shared.saveBinaryState(wrapper.streamOut());
        if (wrapper.isFailed()) return error.OutOfMemory;
        return SimScenes.hashBytes(h, out.written());
    }

    /// ApplySoftBodyOp of SimulationReference.cpp: a soft body operation (OpKind 53..69)
    fn applyOp(zw: *ZoltWorld, op: *const SimOp) !u32 {
        const allocator = zw.allocator;
        const bi = zw.system.getBodyInterfaceMut();
        const activation: Activation = if (op.activation == 0) .activate else .dont_activate;
        const kind: OpKind = @enumFromInt(op.kind);
        const i = &op.i;
        const f = &op.f;

        switch (kind) {
            .create_soft_body, .create_and_add_soft_body, .create_soft_body_no_add => {
                try zw.handles.ensureUnusedCapacity(allocator, 1);
                var settings = makeCreationSettings(zw, &zw.desc.soft_bodies[op.index]);
                defer settings.deinit();
                var new_id: BodyID = .invalid;
                if (kind == .create_and_add_soft_body) {
                    new_id = try bi.createAndAddSoftBody(&settings, activation);
                } else if (try bi.createSoftBody(&settings)) |body| {
                    new_id = body.getID();
                    if (kind == .create_soft_body)
                        bi.addBody(new_id, activation) catch |err| {
                            bi.destroyBody(new_id);
                            return err;
                        };
                }
                zw.handles.appendAssumeCapacity(new_id);
                return new_id.getIndexAndSequenceNumber();
            },
            .set_soft_body_contact_listener => {
                zw.soft_body.listener.mode = @enumFromInt(op.index);
                zw.system.setSoftBodyContactListener(if (op.index == 0) null else .init(&zw.soft_body.listener));
                return 0;
            },
            else => {},
        }

        // The soft body of the body handle. The operations run between the updates: the no-lock interface (CustomUpdate
        // locks the bodies it collides with).
        const body = zw.system.body_manager.tryGetBodyMut(zw.getHandle(op.body)) orelse return error.SceneError;
        if (!body.isSoftBody()) return error.SceneError;
        const mp = zolt.SoftBodyMotionProperties.fromBase(body.getMotionPropertiesMut());
        switch (kind) {
            .soft_body_set_vertex_position => mp.getVertexMut(i[0]).position = load3(f[0..3]),
            .soft_body_set_vertex_velocity => mp.getVertexMut(i[0]).velocity = load3(f[0..3]),
            .soft_body_set_vertex_inv_mass => mp.getVertexMut(i[0]).inv_mass = f[0],
            .soft_body_set_pressure => mp.setPressure(f[0]),
            .soft_body_set_num_iterations => mp.setNumIterations(i[0]),
            .soft_body_set_update_position => mp.setUpdatePosition(i[0] != 0),
            .soft_body_set_faces_double_sided => mp.setFacesDoubleSided(i[0] != 0),
            .soft_body_set_vertex_radius => mp.setVertexRadius(f[0]),
            .soft_body_set_enable_skin_constraints => mp.setEnableSkinConstraints(i[0] != 0),
            .soft_body_set_skinned_max_distance_multiplier => mp.setSkinnedMaxDistanceMultiplier(f[0]),
            .soft_body_skin_vertices => {
                const joints = try allocator.alloc(Mat44, i[1]);
                defer allocator.free(joints);
                for (joints, 0..) |*m, j| {
                    const v = zw.desc.floats[i[0] + 16 * j ..][0..16];
                    m.* = Mat44.init(Vec4.init(v[0], v[1], v[2], v[3]), Vec4.init(v[4], v[5], v[6], v[7]), Vec4.init(v[8], v[9], v[10], v[11]), Vec4.init(v[12], v[13], v[14], v[15]));
                }
                try mp.skinVertices(body.getCenterOfMassTransform(), joints, i[2] != 0, zw.temp.tempAllocator());
            },
            .soft_body_custom_update => try mp.customUpdate(f[0], body, &zw.system),
            .soft_body_creation_settings_hash => {
                var settings = body.getSoftBodyCreationSettings();
                defer settings.deinit();
                return hashCreationSettings(zw, &settings);
            },
            else => return error.SceneError,
        }
        return hashSoftBody(allocator, body);
    }
};

/// The unsaved soft body state of a D12 sweep (see "D12 sweeps" in the soft body section of the SimScenes header):
/// mSkinStatePreviousPositionValid of every soft body at the snapshot. SkinVertices sets it, the update clears it, and
/// Jolt's SaveState does not hold it. Every update that starts from the snapshot of a sweep gets the values of the
/// snapshot again (Run.restoreZolt), like the InvalidateContactCache flags (restoreZoltBeforeUpdate).
const SkinFlags = struct {
    const Flag = struct { id: BodyID, valid: bool };

    flags: std.ArrayList(Flag) = .empty,

    fn deinit(self: *SkinFlags, allocator: Allocator) void {
        self.flags.deinit(allocator);
    }

    fn clear(self: *SkinFlags) void {
        self.flags.clearRetainingCapacity();
    }

    /// The flags of the soft bodies of the body handles now
    fn capture(self: *SkinFlags, allocator: Allocator, zw: *ZoltWorld) !void {
        self.clear();
        for (zw.handles.items) |id| {
            const body = zw.system.body_manager.tryGetBody(id) orelse continue;
            if (!body.isSoftBody()) continue;
            const mp = zolt.SoftBodyMotionProperties.fromBaseConst(body.getMotionProperties());
            try self.flags.append(allocator, .{ .id = id, .valid = mp.skin_state_previous_position_valid });
        }
    }

    /// Put the captured flags back
    fn restore(self: *const SkinFlags, zw: *ZoltWorld) void {
        for (self.flags.items) |flag| {
            const body = zw.system.body_manager.tryGetBodyMut(flag.id) orelse continue;
            zolt.SoftBodyMotionProperties.fromBase(body.getMotionPropertiesMut()).skin_state_previous_position_valid = flag.valid;
        }
    }
};

/// Phase 9, before a scene runs: the SaveBinaryState bytes of every shared settings object that SoftBodySlot.makeSettings
/// generates are the bytes of SimMakeSoftBodySettings (C++). A mismatch is an error of a generator or of
/// SoftBodySharedSettings, reported before the simulation. The name only labels the message.
fn checkSoftBodySettings(allocator: Allocator, name: []const u8, builder: *const SceneBuilder) !void {
    const desc = builder.desc();
    var jolt_bytes: std.ArrayList(u8) = .empty;
    defer jolt_bytes.deinit(allocator);
    for (builder.soft_body_settings.items, 0..) |*s, i| {
        var settings = try SoftBodySlot.makeSettings(allocator, &desc, s);
        defer settings.deinit();
        var out: std.Io.Writer.Allocating = .init(allocator);
        defer out.deinit();
        var wrapper = zolt.StreamOutWrapper.init(&out.writer);
        settings.get().?.saveBinaryState(wrapper.streamOut());
        if (wrapper.isFailed()) return error.OutOfMemory;
        try CppWorld.softBodySettingsBytes(allocator, &desc, @intCast(i), &jolt_bytes);
        if (std.mem.indexOfDiff(u8, out.written(), jolt_bytes.items)) |first| {
            std.debug.print("SimulationParity: scene {s}, soft body settings description {d} ({t}): the Zolt settings differ from SimMakeSoftBodySettings at byte {d} (zolt {d} bytes, jolt {d} bytes)\n", .{ name, i, @as(SimScenes.SoftBodySettingsKind, @enumFromInt(s.kind)), first, out.written().len, jolt_bytes.items.len });
            return error.TestUnexpectedResult;
        }
    }
}

/// Phase 9, Expect.soft_body_groups: some shared settings object of the scene has at least 2 parallel update groups and
/// a non-parallel group with constraints (the last group of SoftBodySharedSettings.update_groups, as Optimize builds
/// them on the Zolt side; the bytes are Jolt's, see checkSoftBodySettings)
fn checkSoftBodyGroups(allocator: Allocator, entry: *const SceneEntry, builder: *const SceneBuilder) !void {
    if (!entry.expect.soft_body_groups) return;
    const desc = builder.desc();
    for (builder.soft_body_settings.items) |*s| {
        var settings = try SoftBodySlot.makeSettings(allocator, &desc, s);
        defer settings.deinit();
        const groups = settings.get().?.update_groups.items;
        if (groups.len < 3) continue; // At least 2 parallel groups and the non-parallel group
        // The non-parallel group has constraints: an end index differs from the one of the last parallel group
        const last_parallel = &groups[groups.len - 2];
        const non_parallel = &groups[groups.len - 1];
        if (!std.mem.eql(u8, std.mem.asBytes(last_parallel), std.mem.asBytes(non_parallel))) return;
    }
    try expectReached(entry, "a shared settings object with several parallel update groups and a non-parallel group with constraints", false);
}

/// The soft body listener records of one update agree with each other (a check of SimSoftBodyContactListener, run on the
/// C++ records after they are equal to Zolt's): per soft body, the vertices with a contact of its
/// soft_body_contact_added records are the vertices of its soft_body_contact_body records, and its sensor contacts are
/// its soft_body_sensor_contact records (summed over the collision steps of the update)
fn checkSoftBodyRecords(allocator: Allocator, entry: *const SceneEntry, update: usize, records: []const SimRecord) !void {
    // Per soft body: vertices with a contact, sensor contacts (soft_body_contact_added); vertices of the body records,
    // sensor records
    var counts: std.AutoArrayHashMapUnmanaged(u32, [4]u32) = .empty;
    defer counts.deinit(allocator);
    for (records) |r| {
        const index: usize = switch (r.kind) {
            @intFromEnum(RecordKind.soft_body_contact_added) => 0,
            @intFromEnum(RecordKind.soft_body_contact_body) => 2,
            @intFromEnum(RecordKind.soft_body_sensor_contact) => 3,
            else => continue,
        };
        const entry_value = try counts.getOrPutValue(allocator, r.ids[0], @splat(0));
        const c = entry_value.value_ptr;
        switch (index) {
            0 => {
                c[0] += r.ids[2];
                c[1] += r.ids[3];
            },
            2 => c[2] += r.ids[2],
            else => c[3] += 1,
        }
    }
    var it = counts.iterator();
    while (it.next()) |kv| {
        const c = kv.value_ptr.*;
        if (c[0] != c[2] or c[1] != c[3]) {
            std.debug.print("SimulationParity: scene {s}, update {d}: the soft body listener records of soft body 0x{x:0>8} do not agree: {d} vertices with a contact and {d} sensor contacts in OnSoftBodyContactAdded, {d} vertices in the body records, {d} sensor records\n", .{ entry.name, update, kv.key_ptr.*, c[0], c[1], c[2], c[3] });
            return error.TestUnexpectedResult;
        }
    }
}

/// Every Zolt class is ported, so every scene runs against Zolt: a scene whose Blocker is not supported (a class that it
/// needs is a stub again) fails instead of being skipped
fn checkSupported(entry: *const SceneEntry) !void {
    if (!entry.blocked_by.isSupported()) {
        std.debug.print("SimulationParity: scene {s}: its blocker .{t} is not supported (a Zolt class that it needs is a stub), but every scene must run against Zolt\n", .{ entry.name, entry.blocked_by });
        return error.SceneNotSupported;
    }
}

/// Phase 7: the Jolt asserts that the C++ side hit since `before` (a reference built with
/// `-Dreference_asserts=true` prints them to stderr; without asserts the count stays 0)
fn checkReferenceAsserts(entry: *const SceneEntry, before: u32) !void {
    const after = jolt.jolt_sim_reference_assert_count();
    if (after != before) {
        std.debug.print("SimulationParity: scene {s}: the C++ side hit {d} Jolt asserts (printed above): the scene violates what Jolt asserts\n", .{ entry.name, after - before });
        return error.TestUnexpectedResult;
    }
}

/// R8, a release_only scene: it violates a JPH_ASSERT that Jolt's release build ignores, and Zolt's assert at that site
/// is recoverable (the counting handler of runEntry lets the code continue like Jolt's release build). Every run hits at
/// least one Zolt assert (the scene reaches what it is written for), the same number in every configuration (no assert
/// depends on the threads), and with -Dreference_asserts=true as many as the C++ side
fn checkReleaseAsserts(entry: *const SceneEntry, stats: []const RunStats) !void {
    if (!zolt.Core.enable_asserts) return;
    const has_reference_asserts = jolt.jolt_sim_reference_has_asserts() != 0;
    for (stats, 0..) |s, i| {
        if (s.num_zolt_asserts == 0 or s.num_zolt_asserts != stats[0].num_zolt_asserts or (has_reference_asserts and s.num_zolt_asserts != s.num_jolt_asserts)) {
            std.debug.print("SimulationParity: scene {s}, configuration {d}: Zolt hit {d} asserts (configuration 0: {d}), the C++ side {d} (counted: {})\n", .{ entry.name, i, s.num_zolt_asserts, stats[0].num_zolt_asserts, s.num_jolt_asserts, has_reference_asserts });
            return error.TestUnexpectedResult;
        }
    }
}

fn checkPhase7Slot(entry: *const SceneEntry, what: []const u8, ported: bool, slot_filled: bool) !void {
    if (!ported) {
        std.debug.print("SimulationParity: scene {s} creates {s} objects, whose Zolt classes are still stubs, but its blocker .{t} is supported (declare the Blocker of the class)\n", .{ entry.name, what, entry.blocked_by });
        return error.TestUnexpectedResult;
    }
    if (!slot_filled) {
        std.debug.print("SimulationParity: scene {s}: the {s} classes are ported but their slot in SimulationParity.zig is still Phase7StubSlot\n", .{ entry.name, what });
        return error.SceneNotSupported;
    }
}

fn runEntry(entry: *const SceneEntry) !void {
    try checkSupported(entry);

    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    try entry.build(&builder);
    try checkConstraintDescriptions(allocator, entry, &builder);
    try checkPhase7Slots(entry, &builder);
    try checkPhase7Settings(allocator, entry, &builder);
    try checkSoftBodySettings(allocator, entry.name, &builder);
    try checkSoftBodyGroups(allocator, entry, &builder);

    // R8: a release_only scene runs with a handler that counts Zolt's asserts and lets the code continue (also during
    // the creation sweep below)
    const release_only = entry.blocked_by == .release_only;
    var asserts: zolt.IssueReporting.CountAsserts = undefined;
    if (release_only) asserts.init();
    defer if (release_only) asserts.restore();

    const asserts_before = jolt.jolt_sim_reference_assert_count();
    var stats: [configs.len]RunStats = undefined;
    for (configs, &stats) |config, *s|
        s.* = try runScene(allocator, entry, &builder, config, if (release_only) &asserts else null);
    if (release_only)
        try checkReleaseAsserts(entry, &stats)
    else
        try checkReferenceAsserts(entry, asserts_before);

    if (verbose)
        std.debug.print("{s}: {any}\n", .{ entry.name, stats[0] });

    // The listener records do not depend on the threads (their number neither); the sweeps ran in every configuration
    // with sweeps, temp allocations and createJob calls failed
    for (stats[1..]) |s|
        try std.testing.expectEqualSlices(u32, &stats[0].records, &s.records);

    // The scene reaches what it is written for (SimScenes.Expect)
    const expect = entry.expect;
    const records = &stats[0].records;
    if (expect.contacts) try expectReached(entry, "contacts", records[@intFromEnum(RecordKind.contact_added)] > 0);
    if (expect.removed) try expectReached(entry, "removed contacts", records[@intFromEnum(RecordKind.contact_removed)] > 0);
    if (expect.validated) try expectReached(entry, "OnContactValidate", records[@intFromEnum(RecordKind.contact_validate)] > 0);
    if (expect.sleeping) try expectReached(entry, "sleeping bodies", stats[0].updates_with_sleeping_bodies > 0);
    var num_spawned_jobs: u32 = 0;
    var num_large_island_deactivations: u32 = 0;
    for (stats) |s| {
        num_spawned_jobs += s.num_spawned_jobs;
        num_large_island_deactivations += s.num_large_island_deactivations;
    }
    if (expect.spawned_jobs) try expectReached(entry, "FindCollisions jobs spawned from inside jobs", num_spawned_jobs > 0 or try observeSpawnedJobs(allocator, &builder));
    if (expect.large_island) try expectReached(entry, "a large island", num_large_island_deactivations > 0);
    if (expect.full_manifolds) {
        try std.testing.expectEqual(@as(u32, 1), builder.collision_steps); // The records of an update are the manifolds of one step
        try expectReached(entry, "a full manifold array in ProcessBodyPair", stats[0].max_manifolds_per_body_pair >= SimScenes.max_manifolds_per_body_pair);
    }
    if (expect.ccd_hits) try expectReached(entry, "CCD hits", stats[0].updates_with_ccd_manifolds > 0);
    var max_find_ccd_contacts_jobs: u32 = 0;
    for (stats) |s| max_find_ccd_contacts_jobs = @max(max_find_ccd_contacts_jobs, s.max_find_ccd_contacts_jobs);
    if (expect.ccd_jobs > 0) try expectReached(entry, "several FindCCDContacts jobs in one update", max_find_ccd_contacts_jobs >= expect.ccd_jobs);
    if (expect.update_activations > 0) try expectReached(entry, "bodies activated from inside one update", stats[0].max_update_activations >= expect.update_activations);
    if (expect.active_constraints > 0) try expectReached(entry, "active constraints", stats[0].max_active_constraints >= expect.active_constraints);
    if (builder.constraints.items.len > 0) try expectReached(entry, "constraints that are solved (every scene with constraints)", stats[0].max_active_constraints > 0);
    if (expect.character_records != 0) try expectReached(entry, "the character listener callbacks of Expect.character_records", recordsReach(records, expect.character_records));
    if (expect.ground_states != 0) try expectReached(entry, "the ground states of Expect.ground_states", (stats[0].objects.ground_states & expect.ground_states) == expect.ground_states);
    if (expect.wheel_contacts) try expectReached(entry, "wheel contacts", stats[0].objects.wheel_contacts);
    try expectVehicleGears(entry, &stats[0].objects);
    if (expect.soft_body_records != 0) try expectReached(entry, "the soft body listener callbacks of Expect.soft_body_records", recordsReach(records, expect.soft_body_records));
    var num_soft_body_ccd_responses: u32 = 0;
    for (stats) |s| num_soft_body_ccd_responses += s.num_soft_body_ccd_responses;
    if (expect.soft_body_ccd) try expectReached(entry, "the CCD response against a soft body face", num_soft_body_ccd_responses > 0);
    if (expect.update_deactivations > 0) try expectReached(entry, "bodies deactivated from inside one update", stats[0].max_update_deactivations >= expect.update_deactivations);
    for (configs, stats) |config, s| {
        try std.testing.expectEqual(stats[0].max_active_constraints, s.max_active_constraints);
        try std.testing.expectEqual(stats[0].num_expectations, s.num_expectations);
        try std.testing.expectEqual(stats[0].objects, s.objects);
        if (config.sweeps and entry.sweep_stride != 0) {
            try std.testing.expect(s.num_temp_failures > 0);
            try std.testing.expect(s.num_job_failures > 0);
        }
    }

    if (entry.creation_sweep)
        try std.testing.checkAllAllocationFailures(allocator, createScene, .{&builder});
}

/// The number of Zolt-only runs of observeSpawnedJobs
const max_spawn_observations = 10;

/// Expect.spawned_jobs when no configuration of the thread matrix spawned a FindCollisions job from inside a job: a job
/// spawns another one when it finishes while the others still find body pairs, which depends on the scheduling (on a
/// loaded machine the workers can run the jobs one after the other, then none spawns: contacts_many_bodies in
/// ReleaseFast next to other builds). The scene runs again on the Zolt side alone (no comparison) with 3 and 7 workers,
/// up to max_spawn_observations times, until a job spawns one: the scene reaches the path, the scheduling decides when.
fn observeSpawnedJobs(allocator: Allocator, builder: *const SceneBuilder) !bool {
    const desc = builder.desc();
    for (0..max_spawn_observations) |attempt| {
        const zw = try ZoltWorld.create(allocator, &desc, .{ .thread_pool = if (attempt % 2 == 0) 3 else 7 });
        defer zw.destroy();
        var delta_time = builder.delta_time;
        var collision_steps = builder.collision_steps;
        var op_index: usize = 0;
        for (0..builder.num_updates) |update| {
            while (op_index < builder.ops.items.len and builder.ops.items[op_index].update == update) : (op_index += 1) {
                const op = &builder.ops.items[op_index];
                switch (@as(OpKind, @enumFromInt(op.kind))) {
                    .set_update => {
                        delta_time = op.f[0];
                        collision_steps = op.count;
                    },
                    .snapshot, .restore_cross => {},
                    else => _ = try zw.applyOp(op),
                }
            }
            _ = try zw.update(delta_time, collision_steps, zw.temp.tempAllocator(), zw.jobSystem());
            zw.discardRecords();
            if (zw.counting_jobs.num_spawned_find_collisions.load(.monotonic) > 0) return true;
        }
    }
    return false;
}

fn expectReached(entry: *const SceneEntry, what: []const u8, reached: bool) !void {
    if (!reached) {
        std.debug.print("SimulationParity: scene {s} does not reach what it is written for: {s}\n", .{ entry.name, what });
        return error.TestUnexpectedResult;
    }
}

// -------------------------------------------------------------------------------------------------------------------
// Simulation-time draws (group D3, only with -Ddebug_renderer=true)
//
// The scenes of `draw_scenes` run once more with drawing on: Zolt and C++ each with a JobSystemThreadPool without
// workers (one thread on each side, so the draws come in the same order), the Zolt system with a ParityRenderer as its
// debug renderer and every draw flag of the contacts (`renderer.flags.contact_constraint_manager`) and of the linear cast
// CCD (`renderer.flags.physics_system`) set, the C++ worlds with a ParityRenderer as DebugRenderer::sInstance and the
// static flags set (jolt_sim_draw_begin). After every update the call logs of the update and of the operations before it
// must match call for call, and the Zolt state must be the state of the same scene in the same configuration without
// drawing (recorded by a run before): drawing changes nothing.
//
// Group D4: every CharacterVirtual flag is set too (`renderer.flags.character_virtual`, the static flags of
// CharacterVirtual), so the character scenes draw the constraints, the walk stairs and stick to floor sweeps and the
// supporting volume. The scenes of `post_draw_scenes` also draw after every update (`SimDraw.postDraws`, before the logs
// are compared): the vehicle scenes draw the constraints the three ways (VehicleConstraint draws its controller: the
// wheeled, tracked and motorcycle draws, the RPM meter, the status and wheel texts), after update 0 with Jolt's default
// RPM meter and draw size, then with an RPM meter and a draw size that change with the update; the soft body scenes draw
// the bodies with the shapes and every soft body flag of DrawSettings (all of them in even updates, one flag after the
// other in odd updates) in the three ESoftBodyConstraintColors. The conditional D4 draws must be reached: each scene of
// `draw_scene_reaches` fails unless it reaches its branches of `DrawBranch` (the WalkStairs sweeps, StickToFloor, the
// three parts of the RPM meter, the wheel speed line, the wheels with and without contact, the soft body constraint
// kinds, the update groups after the first), and every branch is listed there. Proof that it bites: a ConstraintOrder
// fraction divided by `end` instead of `end - start` fails soft_body_large_cloth only (update 2, a draw_line color); a
// default RPM meter size of 0.6 fails the wheeled and motorcycle scenes at update 0; StickToFloor drawn on neither side
// fails the 4 scenes that list stick_to_floor, while their logs match.

/// The scenes that also run with drawing (contacts, CCD, constraints, characters)
const draw_scenes = [_][]const u8{
    "contacts_pyramid",
    "contacts_shapes_on_floor",
    "contacts_mesh_ground",
    "contacts_height_field_ground",
    "contacts_without_manifold_reduction",
    "contacts_many_manifolds",
    "contacts_sensors",
    "ccd_bullet_hits_wall",
    "ccd_bodies_hit_each_other",
    "ccd_terrain",
    "constraints_ragdoll_pile",
    "constraints_hinge_car",
    "character_ops",
    "character_virtual_ops",
    // Group D4: the CharacterVirtual draws (every character virtual scene but perf_character_virtual)
    "character_virtual_characters",
    "character_virtual_stairs_slopes",
    "character_virtual_platforms",
    "character_virtual_far_from_origin",
    "character_virtual_crowd",
    "character_virtual_inner_body",
    "character_virtual_contact_limits",
    "character_virtual_penetration",
    "character_virtual_set_shape",
    "character_virtual_internal_edges",
    "character_virtual_terrain",
    "character_virtual_cross_restore",
    "character_virtual_listener_none",
    "character_virtual_listener_record",
    "character_virtual_listener_modify",
    "character_virtual_shapes",
};

/// Group D4: the draws after every update of a scene (`SimDraw.postDraws`)
const PostDraws = struct {
    /// BodyManager::Draw with the shapes and the soft body flags
    bodies: bool = false,
    /// The three constraint draws of the system (the vehicles)
    constraints: bool = false,
};

/// Group D4: the scenes that also run with drawing and draw after every update (vehicles, soft bodies)
const post_draw_scenes = [_]struct { name: []const u8, post: PostDraws }{
    .{ .name = "vehicle_wheeled_ops", .post = .{ .constraints = true } },
    .{ .name = "vehicle_tracked_ops", .post = .{ .constraints = true } },
    .{ .name = "vehicle_motorcycle_ops", .post = .{ .constraints = true } },
    .{ .name = "vehicle_wheeled_driving", .post = .{ .constraints = true } },
    .{ .name = "vehicle_wheeled_in_air", .post = .{ .constraints = true } },
    .{ .name = "vehicle_tracked_terrains", .post = .{ .constraints = true } },
    .{ .name = "vehicle_motorcycle_turns", .post = .{ .constraints = true } },
    .{ .name = "vehicle_wheeled_far_from_origin", .post = .{ .constraints = true, .bodies = true } },
    .{ .name = "soft_body_cube_on_floor", .post = .{ .bodies = true } },
    .{ .name = "soft_body_ops", .post = .{ .bodies = true } },
    .{ .name = "soft_body_cloth_lra", .post = .{ .bodies = true } },
    .{ .name = "soft_body_cloth_dihedral", .post = .{ .bodies = true } },
    .{ .name = "soft_body_pressure_sphere", .post = .{ .bodies = true } },
    .{ .name = "soft_body_rod_chain", .post = .{ .bodies = true } },
    .{ .name = "soft_body_skinned_back_stops", .post = .{ .bodies = true } },
    .{ .name = "soft_body_several_on_one_body", .post = .{ .bodies = true } },
    .{ .name = "soft_body_update_position_false", .post = .{ .bodies = true } },
    .{ .name = "soft_body_test_bend_constraint", .post = .{ .bodies = true } },
    // The only scene with several parallel update groups and a non-parallel group with constraints: the constraints of
    // the groups after the first (start > 0) in the ConstraintGroup and ConstraintOrder colors
    .{ .name = "soft_body_large_cloth", .post = .{ .bodies = true } },
};

/// Group D4: the branches of the conditional D4 draws that the layer must reach. `SimDraw.reached` collects them: the
/// CharacterVirtual branches from the Zolt call log of the update (`markCharacterArrows`), the vehicle and soft body
/// branches before the draws of postDraws, with the conditions of the draw functions (`markPostDrawBranches`). A scene
/// of `draw_scene_reaches` fails when it does not reach its branches, and every branch is listed there (checked at
/// compile time).
const DrawBranch = enum {
    /// CharacterVirtual::WalkStairs: the sweep down finds a slope that is too steep (red arrows right after the white
    /// arrow of the horizontal sweep)
    walk_stairs_too_steep,
    /// WalkStairs: the 2nd horizontal sweep to inStepForwardTest (cyan arrow)
    walk_stairs_second_sweep,
    /// WalkStairs: the 2nd sweep down finds a floor that is not too steep (cyan arrows right after the cyan arrow)
    walk_stairs_second_sweep_down,
    /// WalkStairs: the 2nd sweep down finds a slope that is too steep (red arrows right after the cyan arrow)
    walk_stairs_second_sweep_down_too_steep,
    /// CharacterVirtual::StickToFloor (orange arrow)
    stick_to_floor,
    /// VehicleEngine::DrawRPM: the RPM is below the shift down RPM (the red and dark red parts)
    rpm_below_shift_down,
    /// DrawRPM: the RPM is between the shift down and the shift up RPM (the orange and dark orange parts)
    rpm_between_shifts,
    /// DrawRPM: the RPM is above the shift up RPM (the green and dark green parts)
    rpm_above_shift_up,
    /// WheeledVehicleController::Draw: the wheel speed at the clutch (a transmission ratio that is not 0)
    wheel_speed_line,
    /// The wheeled and tracked Draw: a wheel with a contact (the contact lines and the slip text)
    wheel_contact,
    /// The wheeled and tracked Draw: a wheel without a contact (the 'no hit' text)
    wheel_no_hit,
    /// BodyManager::Draw of a soft body with skinned constraints (SoftBodyMotionProperties::DrawSkinConstraints)
    soft_body_skin_constraints,
    /// ... with LRA constraints (DrawLRAConstraints)
    soft_body_lra_constraints,
    /// ... with rods (DrawRods)
    soft_body_rods,
    /// ... with rods (DrawRodStates)
    soft_body_rod_states,
    /// ... with rod bend twist constraints (DrawRodBendTwistConstraints)
    soft_body_rod_bend_twist_constraints,
    /// ... with dihedral bend constraints (DrawBendConstraints)
    soft_body_bend_constraints,
    /// ... with volume constraints (DrawVolumeConstraints)
    soft_body_volume_constraints,
    /// DrawConstraints: drawn constraints in an update group after the first (start > 0), in ConstraintGroup colors
    /// (the distinct color of the group)
    soft_body_later_group_colors,
    /// DrawConstraints: the same in ConstraintOrder colors (the fraction (idx - start) / (end - start))
    soft_body_later_group_order,
};

/// Group D4: the branches of `DrawBranch` that a scene of `draw_scenes` or `post_draw_scenes` must reach (all that it
/// reaches today; `verbose` prints them)
const draw_scene_reaches = [_]struct { name: []const u8, reaches: []const DrawBranch }{
    .{ .name = "character_virtual_ops", .reaches = &.{.stick_to_floor} },
    .{ .name = "character_virtual_stairs_slopes", .reaches = &.{ .walk_stairs_too_steep, .walk_stairs_second_sweep, .walk_stairs_second_sweep_down, .stick_to_floor } },
    .{ .name = "character_virtual_inner_body", .reaches = &.{ .walk_stairs_too_steep, .walk_stairs_second_sweep, .walk_stairs_second_sweep_down_too_steep, .stick_to_floor } },
    .{ .name = "character_virtual_terrain", .reaches = &.{.stick_to_floor} },
    .{ .name = "character_virtual_cross_restore", .reaches = &.{ .walk_stairs_too_steep, .walk_stairs_second_sweep, .walk_stairs_second_sweep_down } },
    .{ .name = "character_virtual_listener_none", .reaches = &.{ .walk_stairs_too_steep, .walk_stairs_second_sweep, .walk_stairs_second_sweep_down_too_steep } },
    .{ .name = "character_virtual_listener_record", .reaches = &.{ .walk_stairs_too_steep, .walk_stairs_second_sweep, .walk_stairs_second_sweep_down_too_steep } },
    .{ .name = "character_virtual_listener_modify", .reaches = &.{ .walk_stairs_too_steep, .walk_stairs_second_sweep, .walk_stairs_second_sweep_down_too_steep } },
    .{ .name = "vehicle_wheeled_ops", .reaches = &.{ .rpm_below_shift_down, .rpm_between_shifts, .wheel_speed_line, .wheel_contact } },
    .{ .name = "vehicle_tracked_ops", .reaches = &.{ .rpm_below_shift_down, .rpm_between_shifts, .rpm_above_shift_up, .wheel_contact } },
    .{ .name = "vehicle_motorcycle_ops", .reaches = &.{ .rpm_below_shift_down, .rpm_between_shifts, .wheel_speed_line, .wheel_contact } },
    .{ .name = "vehicle_wheeled_driving", .reaches = &.{ .rpm_below_shift_down, .rpm_between_shifts, .rpm_above_shift_up, .wheel_speed_line, .wheel_contact, .wheel_no_hit } },
    .{ .name = "vehicle_wheeled_in_air", .reaches = &.{ .rpm_below_shift_down, .wheel_speed_line, .wheel_contact, .wheel_no_hit } },
    .{ .name = "vehicle_tracked_terrains", .reaches = &.{ .rpm_below_shift_down, .rpm_between_shifts, .rpm_above_shift_up, .wheel_contact, .wheel_no_hit } },
    .{ .name = "vehicle_motorcycle_turns", .reaches = &.{ .rpm_below_shift_down, .rpm_between_shifts, .wheel_speed_line, .wheel_contact } },
    .{ .name = "vehicle_wheeled_far_from_origin", .reaches = &.{ .rpm_below_shift_down, .rpm_between_shifts, .rpm_above_shift_up, .wheel_speed_line, .wheel_contact } },
    .{ .name = "soft_body_cube_on_floor", .reaches = &.{.soft_body_volume_constraints} },
    .{ .name = "soft_body_ops", .reaches = &.{ .soft_body_skin_constraints, .soft_body_lra_constraints, .soft_body_rods, .soft_body_rod_states, .soft_body_rod_bend_twist_constraints, .soft_body_bend_constraints, .soft_body_volume_constraints } },
    .{ .name = "soft_body_cloth_lra", .reaches = &.{.soft_body_lra_constraints} },
    .{ .name = "soft_body_cloth_dihedral", .reaches = &.{.soft_body_bend_constraints} },
    .{ .name = "soft_body_rod_chain", .reaches = &.{ .soft_body_rods, .soft_body_rod_states, .soft_body_rod_bend_twist_constraints } },
    .{ .name = "soft_body_skinned_back_stops", .reaches = &.{.soft_body_skin_constraints} },
    .{ .name = "soft_body_several_on_one_body", .reaches = &.{.soft_body_volume_constraints} },
    .{ .name = "soft_body_update_position_false", .reaches = &.{.soft_body_volume_constraints} },
    .{ .name = "soft_body_test_bend_constraint", .reaches = &.{.soft_body_bend_constraints} },
    .{ .name = "soft_body_large_cloth", .reaches = &.{ .soft_body_lra_constraints, .soft_body_bend_constraints, .soft_body_later_group_colors, .soft_body_later_group_order } },
};

comptime {
    @setEvalBranchQuota(100000);
    var listed: std.EnumSet(DrawBranch) = .initEmpty();
    for (draw_scene_reaches) |r| {
        for (r.reaches) |branch| listed.insert(branch);
        const known = for (draw_scenes) |name| {
            if (std.mem.eql(u8, name, r.name)) break true;
        } else for (post_draw_scenes) |scene| {
            if (std.mem.eql(u8, scene.name, r.name)) break true;
        } else false;
        if (!known) @compileError("draw_scene_reaches: " ++ r.name ++ " is not a scene of draw_scenes or post_draw_scenes");
    }
    var it = listed.complement().iterator();
    if (it.next()) |branch| @compileError("draw_scene_reaches: no scene reaches the draw branch " ++ @tagName(branch));
}

/// The draws of a run (see above). Mode `record_states` records the Zolt state after every update, mode `draw` draws and
/// compares.
const SimDraw = struct {
    allocator: Allocator,
    mode: enum { record_states, draw },
    /// The Zolt state after every update of the run without drawing
    states: std.ArrayList([]u8) = .empty,
    renderer: if (Core.debug_renderer) ParityRenderer.ParityRenderer else void = undefined,
    jolt_log: std.ArrayList(u8) = .empty,
    /// The number of log bytes compared (the scene must draw something)
    num_log_bytes: usize = 0,
    /// Group D4: the draws after every update
    post: PostDraws = .{},
    /// Group D4: the branches of the conditional draws that the run reached
    reached: std.EnumSet(DrawBranch) = .initEmpty(),

    fn deinit(self: *SimDraw) void {
        for (self.states.items) |s| self.allocator.free(s);
        self.states.deinit(self.allocator);
        self.jolt_log.deinit(self.allocator);
    }

    /// Mode `draw`: the renderers and the flags of both sides
    fn begin(self: *SimDraw, zw: *ZoltWorld) !void {
        if (comptime !Core.debug_renderer) unreachable;
        if (self.mode != .draw) return;
        try self.renderer.init(self.allocator);
        self.renderer.log.bytes.clearRetainingCapacity();
        self.renderer.base.flags.contact_constraint_manager = .{ .draw_contact_point = true, .draw_supporting_faces = true, .draw_contact_point_reduction = true, .draw_contact_manifolds = true };
        self.renderer.base.flags.physics_system.draw_motion_quality_linear_cast = true;
        self.renderer.base.flags.character_virtual = .{ .draw_constraints = true, .draw_walk_stairs = true, .draw_stick_to_floor = true, .draw_supporting_volume = true };
        zw.system.setDebugRenderer(&self.renderer.base);
        std.debug.assert(jolt.jolt_sim_draw_begin() == 1);
    }

    fn end(self: *SimDraw, zw: *ZoltWorld) void {
        if (comptime !Core.debug_renderer) unreachable;
        if (self.mode != .draw) return;
        zw.system.setDebugRenderer(null);
        jolt.jolt_sim_draw_end();
        self.renderer.deinit();
    }

    /// Group D4: the draws after an update (only with the option)
    const postDraws = if (Core.debug_renderer) postDrawsImpl else {};

    /// After every update (Run.step, `run.zolt_state` holds the Zolt state): record the state, or compare it with the
    /// recorded one and compare the call logs since the last update
    fn afterUpdate(self: *SimDraw, run: *Run) !void {
        if (comptime !Core.debug_renderer) unreachable;
        switch (self.mode) {
            .record_states => try self.states.append(self.allocator, try self.allocator.dupe(u8, run.zolt_state.items)),
            .draw => {
                const without = self.states.items[run.update];
                if (std.mem.indexOfDiff(u8, run.zolt_state.items, without)) |first|
                    return run.fail("the state with drawing differs from the state without drawing at byte {d} ({d} / {d} bytes)", .{ first, run.zolt_state.items.len, without.len });
                markCharacterArrows(self.renderer.log.bytes.items, &self.reached);
                self.postDraws(run);
                self.jolt_log.clearRetainingCapacity();
                const size = jolt.jolt_sim_draw_take_log(self.jolt_log.items.ptr, 0);
                try self.jolt_log.resize(self.allocator, @intCast(size));
                std.debug.assert(jolt.jolt_sim_draw_take_log(self.jolt_log.items.ptr, size) == size);
                const zolt_log = &self.renderer.log.bytes;
                ParityRenderer.compareLogs("simulation draws", zolt_log.items, self.jolt_log.items) catch |err| {
                    _ = run.fail("the draw call logs of the update differ (see above)", .{}) catch {};
                    return err;
                };
                self.num_log_bytes += zolt_log.items.len;
                zolt_log.clearRetainingCapacity();
            },
        }
    }
};

/// The bits of `ParityRenderer.BodyDrawBits` that postDraws uses
const PostDrawBits = struct {
    const Bits = ParityRenderer.BodyDrawBits;
    const num_soft_body_flags = Bits.count - Bits.first_soft_body;
    const all_soft_body: u32 = Bits.all & ~((@as(u32, 1) << Bits.first_soft_body) - 1);
};

/// Group D4 (see above): the draws after update `run.update` on both sides. From update 1 on, the RPM meters and draw
/// sizes change with the update (the draws after update 0 use the defaults: Jolt's mRPMMeterPosition, mRPMMeterSize and
/// mDrawConstraintSize); so do the soft body flags (all of them, or one) and the ESoftBodyConstraintColor.
fn postDrawsImpl(self: *SimDraw, run: *Run) void {
    if (!self.post.bodies and !self.post.constraints) return;
    const u = run.update;
    const rpm_meter = [4]f32{ 0.25 * @as(f32, @floatFromInt(u % 3)), 1.0 + 0.25 * @as(f32, @floatFromInt(u % 4)), -0.5, 0.5 + 0.125 * @as(f32, @floatFromInt(u % 5)) };
    const draw_size: f32 = 0.5 + 0.25 * @as(f32, @floatFromInt(u % 4));
    const set_meters = u > 0;
    if (set_meters) {
        for (run.zw.phase7.vehicles.vehicles.items) |v| {
            const constraint = v.get();
            const controller = constraint.getControllerMut();
            const position = Vec3.init(rpm_meter[0], rpm_meter[1], rpm_meter[2]);
            switch (v.controller) {
                .wheeled => zolt.virtual.downcast(zolt.WheeledVehicleController, controller).setRPMMeter(position, rpm_meter[3]),
                .tracked => zolt.virtual.downcast(zolt.TrackedVehicleController, controller).setRPMMeter(position, rpm_meter[3]),
                .motorcycle => zolt.virtual.downcast(zolt.MotorcycleController, controller).base.setRPMMeter(position, rpm_meter[3]),
            }
            constraint.base.setDrawConstraintSize(draw_size);
        }
    }

    const body_bits: u32 = if (!self.post.bodies) 0 else ParityRenderer.BodyDrawBits.draw_shape | if (u % 2 == 0) PostDrawBits.all_soft_body else @as(u32, 1) << @intCast(ParityRenderer.BodyDrawBits.first_soft_body + (u / 2) % PostDrawBits.num_soft_body_flags);
    const shape_color: u8 = @intCast(u % ParityRenderer.BodyDrawBits.num_shape_colors);
    const soft_body_constraint_color: u8 = @intCast(u % ParityRenderer.BodyDrawBits.num_soft_body_constraint_colors);
    const renderer = &self.renderer.base;
    const settings = ParityRenderer.bodyDrawSettings(body_bits, shape_color, soft_body_constraint_color);
    markPostDrawBranches(self, run, if (body_bits != 0) &settings else null);
    if (body_bits != 0)
        run.zw.system.drawBodies(&settings, renderer, .{});
    if (self.post.constraints) {
        run.zw.system.drawConstraints(renderer);
        run.zw.system.drawConstraintLimits(renderer);
        run.zw.system.drawConstraintReferenceFrame(renderer);
    }
    jolt.jolt_sim_draw_world(run.cw.handle, body_bits, shape_color, soft_body_constraint_color, @intFromBool(self.post.constraints), if (set_meters) &rpm_meter else null, draw_size);
}

/// Group D4: marks the vehicle and soft body branches of `DrawBranch` that the draws of postDrawsImpl reach (`settings`:
/// the DrawSettings of BodyManager::Draw, null when it does not draw), with the conditions of the draw functions
fn markPostDrawBranches(self: *SimDraw, run: *Run, settings: ?*const zolt.BodyManager.DrawSettings) void {
    if (comptime !Core.debug_renderer) return;
    const reached = &self.reached;
    if (self.post.constraints) {
        for (run.zw.phase7.vehicles.vehicles.items) |v| {
            const constraint = v.get();
            const controller = constraint.getControllerMut();
            const engine: *const zolt.VehicleEngine, const transmission: *const zolt.VehicleTransmission = switch (v.controller) {
                .wheeled, .motorcycle => blk: {
                    const c = if (v.controller == .wheeled) zolt.virtual.downcast(zolt.WheeledVehicleController, controller) else &zolt.virtual.downcast(zolt.MotorcycleController, controller).base;
                    // WheeledVehicleController::Draw: the wheel speed line
                    if (c.transmission.getCurrentRatio() != 0.0) reached.insert(.wheel_speed_line);
                    break :blk .{ &c.engine, &c.transmission };
                },
                .tracked => blk: {
                    const c = zolt.virtual.downcast(zolt.TrackedVehicleController, controller);
                    break :blk .{ &c.engine, &c.transmission };
                },
            };

            // VehicleEngine::DrawRPM
            const rpm = engine.getCurrentRPM();
            const shift_down_rpm = transmission.base.shift_down_rpm;
            const shift_up_rpm = transmission.base.shift_up_rpm;
            if (rpm < shift_down_rpm) reached.insert(.rpm_below_shift_down);
            if (rpm > shift_down_rpm and rpm < shift_up_rpm) reached.insert(.rpm_between_shifts);
            if (rpm > shift_up_rpm) reached.insert(.rpm_above_shift_up);

            for (constraint.getWheels()) |w|
                reached.insert(if (w.hasContact()) .wheel_contact else .wheel_no_hit);
        }
    }

    const s = settings orelse return;
    for (run.zw.system.body_manager.getBodies()) |slot| {
        if (!slot.isValidBodyPointer()) continue;
        const body = slot.getBody();
        if (!body.isInBroadPhase() or !body.isSoftBody()) continue;
        const shared = zolt.SoftBodyMotionProperties.fromBaseConst(body.getMotionProperties()).settings.get().?;
        if (s.draw_soft_body_skin_constraints and shared.skinned_constraints.items.len > 0) reached.insert(.soft_body_skin_constraints);
        if (s.draw_soft_body_lra_constraints and shared.lra_constraints.items.len > 0) reached.insert(.soft_body_lra_constraints);
        if (s.draw_soft_body_rods and shared.rod_stretch_shear_constraints.items.len > 0) reached.insert(.soft_body_rods);
        if (s.draw_soft_body_rod_states and shared.rod_stretch_shear_constraints.items.len > 0) reached.insert(.soft_body_rod_states);
        if (s.draw_soft_body_rod_bend_twist_constraints and shared.rod_bend_twist_constraints.items.len > 0) reached.insert(.soft_body_rod_bend_twist_constraints);
        if (s.draw_soft_body_bend_constraints and shared.dihedral_bend_constraints.items.len > 0) reached.insert(.soft_body_bend_constraints);
        if (s.draw_soft_body_volume_constraints and shared.volume_constraints.items.len > 0) reached.insert(.soft_body_volume_constraints);

        // SoftBodyMotionProperties::DrawConstraints: a drawn kind of constraint with constraints in a group after the first
        if (s.draw_soft_body_constraint_color == .constraint_type) continue;
        const kinds = .{
            .{ s.draw_soft_body_edge_constraints, "edge_end_index" },
            .{ s.draw_soft_body_rods or s.draw_soft_body_rod_states, "rod_stretch_shear_end_index" },
            .{ s.draw_soft_body_rod_bend_twist_constraints, "rod_bend_twist_end_index" },
            .{ s.draw_soft_body_bend_constraints, "dihedral_bend_end_index" },
            .{ s.draw_soft_body_volume_constraints, "volume_end_index" },
            .{ s.draw_soft_body_skin_constraints, "skinned_end_index" },
            .{ s.draw_soft_body_lra_constraints, "lra_end_index" },
        };
        const groups = shared.update_groups.items;
        inline for (kinds) |kind| {
            if (kind[0]) {
                for (1..groups.len) |i| {
                    if (@field(groups[i], kind[1]) > @field(groups[i - 1], kind[1]))
                        reached.insert(if (s.draw_soft_body_constraint_color == .constraint_order) .soft_body_later_group_order else .soft_body_later_group_colors);
                }
            }
        }
    }
}

/// Group D4: marks the CharacterVirtual branches of `DrawBranch` in a Zolt call log. DebugRenderer.drawArrow logs 3
/// lines of one color that end in the same point; the 2 lines of the head have the length sqrt(2) * size. WalkStairs and
/// StickToFloor draw arrows of size 0.01. No other draw of an update in the draw scenes does (the 0.01 arrows of the
/// soft bodies come from postDraws, after the scan).
fn markCharacterArrows(log: []const u8, reached: *std.EnumSet(DrawBranch)) void {
    const Line = struct { from: [3]Real, to: [3]Real, color: u32 };
    const white = Color.white.getUInt32();
    const red = Color.red.getUInt32();
    const cyan = Color.cyan.getUInt32();
    const orange = Color.orange.getUInt32();
    const head_length = @sqrt(2.0) * 0.01;

    var window: [3]Line = undefined;
    var num_lines: usize = 0; // The number of lines in a row at the end of the window
    var previous_color: ?u32 = null; // The color of the last arrow, null after a record that is not a line
    var lines_since_arrow: usize = 0; // The number of lines after the last arrow
    var offset: usize = 0;
    while (offset < log.len) {
        const size = ParityRenderer.recordSize(log[offset..]) orelse return;
        const record = log[offset..][0..size];
        offset += size;
        if (record[0] != @intFromEnum(ParityRenderer.LogOp.draw_line)) {
            num_lines = 0;
            previous_color = null;
            continue;
        }
        const vec_size = 3 * @sizeOf(Real);
        window[0] = window[1];
        window[1] = window[2];
        window[2] = .{
            .from = std.mem.bytesToValue([3]Real, record[1..][0..vec_size]),
            .to = std.mem.bytesToValue([3]Real, record[1 + vec_size ..][0..vec_size]),
            .color = std.mem.readInt(u32, record[1 + 2 * vec_size ..][0..4], .little),
        };
        num_lines += 1;
        lines_since_arrow += 1;
        if (num_lines < 3 or (previous_color != null and lines_since_arrow < 3)) continue;

        // An arrow of size 0.01: one color, one end point, the head lines of the right length
        const base = window[0];
        const is_arrow = for (window[1..]) |head| {
            if (head.color != base.color or !std.mem.eql(Real, &head.to, &base.to)) break false;
            var length_sq: f64 = 0;
            for (head.from, head.to) |a, b| length_sq += (@as(f64, a) - b) * (@as(f64, a) - b);
            if (@abs(@sqrt(length_sq) - head_length) > 0.01 * head_length) break false;
        } else true;
        if (!is_arrow) continue;

        // The color of the arrow right before this one (no record in between)
        const color = base.color;
        const after: ?u32 = if (lines_since_arrow == 3) previous_color else null;
        if (color == orange) reached.insert(.stick_to_floor);
        if (color == cyan) reached.insert(.walk_stairs_second_sweep);
        if (after) |p| {
            if (p == white and color == red) reached.insert(.walk_stairs_too_steep);
            if (p == cyan and color == cyan) reached.insert(.walk_stairs_second_sweep_down);
            if (p == cyan and color == red) reached.insert(.walk_stairs_second_sweep_down_too_steep);
        }
        previous_color = color;
        lines_since_arrow = 0;
    }
}

/// Run a scene of `draw_scenes` without drawing (recording the states), then with drawing
fn runDrawScene(name: []const u8) !void {
    return runDrawSceneWith(name, .{});
}

/// runDrawScene with the draws after every update of `post` (group D4)
fn runDrawSceneWith(name: []const u8, post: PostDraws) !void {
    if (!Core.debug_renderer) return error.SkipZigTest;
    const entry = for (&SimScenes.scenes) |*e| {
        if (std.mem.eql(u8, e.name, name)) break e;
    } else return error.UnknownScene;
    try checkSupported(entry);
    if (entry.blocked_by == .release_only)
        return error.SkipZigTest;

    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    try entry.build(&builder);

    var draw: SimDraw = .{ .allocator = allocator, .mode = .record_states, .post = post };
    defer draw.deinit();
    const config: Config = .{ .zolt = .{ .thread_pool = 0 }, .cpp_threads = 0, .draw = &draw };
    _ = try runScene(allocator, entry, &builder, config, null);
    try std.testing.expectEqual(@as(usize, builder.num_updates), draw.states.items.len);
    draw.mode = .draw;
    _ = try runScene(allocator, entry, &builder, config, null);
    if (draw.num_log_bytes == 0) {
        std.debug.print("SimulationParity: scene {s} draws nothing\n", .{name});
        return error.TestUnexpectedResult;
    }

    // Group D4: the branches of the conditional draws that the scene must reach
    if (verbose) {
        var it = draw.reached.iterator();
        std.debug.print("SimulationParity draws: scene {s} reaches", .{name});
        while (it.next()) |branch| std.debug.print(" .{t},", .{branch});
        std.debug.print("\n", .{});
    }
    var missed = false;
    for (draw_scene_reaches) |r| {
        if (!std.mem.eql(u8, r.name, name)) continue;
        for (r.reaches) |branch| {
            if (!draw.reached.contains(branch)) {
                std.debug.print("SimulationParity: scene {s} does not reach the draw branch {t}\n", .{ name, branch });
                missed = true;
            }
        }
    }
    if (missed) return error.TestUnexpectedResult;
}

fn DrawSceneTest(comptime name: []const u8) type {
    return struct {
        test "SimulationParity draws" {
            try runDrawScene(name);
        }
    };
}

comptime {
    for (draw_scenes) |name|
        _ = DrawSceneTest(name);
}

fn PostDrawSceneTest(comptime name: []const u8, comptime post: PostDraws) type {
    return struct {
        test "SimulationParity draws (after the updates)" {
            try runDrawSceneWith(name, post);
        }
    };
}

comptime {
    for (post_draw_scenes) |scene|
        _ = PostDrawSceneTest(scene.name, scene.post);
}

/// One test per scene: `SceneTest("name", index)`
fn SceneTest(comptime name: []const u8, comptime index: usize) type {
    _ = name;
    return struct {
        test "SimulationParity scene" {
            try runEntry(&SimScenes.scenes[index]);
        }
    };
}

comptime {
    for (SimScenes.scenes, 0..) |entry, i|
        _ = SceneTest(entry.name, i);
}

test {
    _ = SimScenes;
}

/// Expect.vehicle_gears / vehicle_up_shifts / vehicle_down_shifts
fn expectVehicleGears(entry: *const SceneEntry, objects: *const ObjectStats) !void {
    const expect = &entry.expect;
    if (expect.vehicle_gears != 0)
        try expectReached(entry, "the transmission gears of Expect.vehicle_gears", (objects.vehicle_gears & expect.vehicle_gears) == expect.vehicle_gears);
    if (expect.vehicle_up_shifts > 0)
        try expectReached(entry, "the forward shifts up of Expect.vehicle_up_shifts", objects.vehicle_up_shifts >= expect.vehicle_up_shifts);
    if (expect.vehicle_down_shifts > 0)
        try expectReached(entry, "the forward shifts down of Expect.vehicle_down_shifts", objects.vehicle_down_shifts >= expect.vehicle_down_shifts);
}

test "SimScenes struct layout matches SimulationReference.cpp" {
    var sizes: [16]u32 = undefined;
    jolt.jolt_sim_struct_sizes(&sizes);
    const expected = [16]u32{
        @sizeOf(SimShape),
        @sizeOf(SimCompoundChild),
        @sizeOf(SimBody),
        @sizeOf(SimPhysicsSettings),
        @sizeOf(SimStepListener),
        @sizeOf(SimConstraint),
        @sizeOf(SimOp),
        @sizeOf(SimSceneDesc),
        @sizeOf(SimRecord),
        @offsetOf(SimBody, "shape"),
        @offsetOf(SimSceneDesc, "shapes"),
        @offsetOf(SimOp, "r"),
        @offsetOf(SimOp, "i"),
        @offsetOf(SimConstraint, "user_data"),
        @offsetOf(SimConstraint, "r"),
        @offsetOf(SimConstraint, "f"),
    };
    try std.testing.expectEqualSlices(u32, &expected, &sizes);

    // CSSettings of ConstraintsReference.cpp (test "SimScenes constraint descriptions are ConstraintsParity's")
    var cs_sizes: [12]u32 = undefined;
    jolt.jolt_cs_struct_sizes(&cs_sizes);
    try std.testing.expectEqual(@as(u32, @sizeOf(CSSettings)), cs_sizes[1]);
    try std.testing.expectEqual(@as(u32, @offsetOf(CSSettings, "user_data")), cs_sizes[5]);
    try std.testing.expectEqual(@as(u32, @offsetOf(CSSettings, "r")), cs_sizes[6]);
    try std.testing.expectEqual(@as(u32, @offsetOf(CSSettings, "f")), cs_sizes[7]);
}

/// A random constraint description of `kind` (valid enum values, any floats; Gear / RackAndPinion with SetRatio when
/// `use_set_ratio`)
fn randomConstraintDescription(b: *SceneBuilder, kind: ConstraintKind, use_set_ratio: bool) SimConstraint {
    var c: SimConstraint = .{ .kind = @intFromEnum(kind), .body1 = b.index(4), .body2 = b.index(4) };
    c.enabled = b.index(2);
    c.constraint_priority = b.next();
    c.num_velocity_steps_override = b.index(256);
    c.num_position_steps_override = b.index(256);
    c.draw_constraint_size = b.float(0, 3);
    c.user_data = @as(u64, b.next()) << 32 | b.next();
    for (&c.i) |*v| v.* = b.index(2); // Spaces, booleans, spring and motor modes, swing types
    for (&c.r) |*v| v.* = b.float(-100, 100);
    for (&c.f) |*v| v.* = b.float(-10, 10);
    switch (kind) {
        .slider => {
            if (c.i[2] != 0) c.i[0] = SimScenes.ConstraintSpace.world_space; // SetSliderAxis asserts world space
        },
        .path => {
            c.i[2] = 1 + b.index(16); // Number of path points
            c.i[3] = b.index(6); // EPathRotationConstraintType
        },
        .gear, .rack_and_pinion => {
            c.i[1] = @intFromBool(use_set_ratio);
            c.i[2] = 1 + b.index(50); // Numbers of teeth
            c.i[3] = 1 + b.index(50);
            c.f[7] = b.float(0.5, 5); // Rack length
        },
        else => {},
    }
    return c;
}

/// The SaveBinaryState bytes of the C++ settings of a description
fn cppSettingsBytes(allocator: Allocator, c: *const SimConstraint, out: *std.ArrayList(u8)) !void {
    const size = jolt.jolt_sim_constraint_settings_save(c, out.items.ptr, 0);
    try out.resize(allocator, size);
    _ = jolt.jolt_sim_constraint_settings_save(c, out.items.ptr, size);
}

test "SimScenes constraint descriptions are ConstraintsParity's" {
    // The C++ settings of SimMakeConstraintSettings (SimulationReference.cpp) and CSMakeSettings (ConstraintsReference.cpp)
    // of the same random description have the same binary state, so SimConstraint has the layout of the constraint
    // parity (minus the SetRatio helpers of Gear / RackAndPinion, which CSMakeSettings does not have); for the ported
    // kinds the settings of the Zolt slot (makeConstraintSettings) have the same bytes too
    const allocator = std.testing.allocator;
    var b: SceneBuilder = .init(allocator);
    defer b.deinit();
    var sim_bytes: std.ArrayList(u8) = .empty;
    defer sim_bytes.deinit(allocator);
    var cs_bytes: std.ArrayList(u8) = .empty;
    defer cs_bytes.deinit(allocator);
    var num_zolt: u32 = 0;
    for (0..200) |iteration| {
        inline for (@typeInfo(ConstraintKind).@"enum".fields) |field| {
            const kind: ConstraintKind = @enumFromInt(field.value);
            const use_set_ratio = (kind == .gear or kind == .rack_and_pinion) and iteration % 2 == 1;
            const c = randomConstraintDescription(&b, kind, use_set_ratio);
            try cppSettingsBytes(allocator, &c, &sim_bytes);

            if (!use_set_ratio) {
                const cs: CSSettings = .{ .kind = kind.subType(), .enabled = c.enabled, .constraint_priority = c.constraint_priority, .num_velocity_steps_override = c.num_velocity_steps_override, .num_position_steps_override = c.num_position_steps_override, .draw_constraint_size = c.draw_constraint_size, .user_data = c.user_data, .i = c.i, .r = c.r, .f = c.f };
                const size = jolt.jolt_cs_settings_save(&cs, cs_bytes.items.ptr, 0);
                try cs_bytes.resize(allocator, size);
                _ = jolt.jolt_cs_settings_save(&cs, cs_bytes.items.ptr, size);
                try std.testing.expectEqualSlices(u8, cs_bytes.items, sim_bytes.items);
            }

            if (kind.isPorted()) {
                var settings = try makeConstraintSettings(allocator, &c);
                defer settings.deinit();
                const zolt_bytes = try saveSettingsBytes(allocator, settings.get().?);
                defer allocator.free(zolt_bytes);
                try std.testing.expectEqualSlices(u8, sim_bytes.items, zolt_bytes);
                num_zolt += 1;
            }
        }
    }
    try std.testing.expect(num_zolt >= 2 * 200); // Fixed and Point at least
}

test "SimScenes Desc.defaults are the defaults of Jolt's constraint settings" {
    // The description that the scenes start from (SimScenes.Desc.defaults) gives the settings of Jolt's default
    // constructors, for every kind
    const allocator = std.testing.allocator;
    var desc_bytes: std.ArrayList(u8) = .empty;
    defer desc_bytes.deinit(allocator);
    var default_bytes: std.ArrayList(u8) = .empty;
    defer default_bytes.deinit(allocator);
    for (std.enums.values(ConstraintKind)) |kind| {
        const c = SimScenes.Desc.defaults(kind);
        try cppSettingsBytes(allocator, &c, &desc_bytes);
        const size = jolt.jolt_sim_default_constraint_settings_save(@intFromEnum(kind), default_bytes.items.ptr, 0);
        try default_bytes.resize(allocator, size);
        _ = jolt.jolt_sim_default_constraint_settings_save(@intFromEnum(kind), default_bytes.items.ptr, size);
        std.testing.expectEqualSlices(u8, default_bytes.items, desc_bytes.items) catch |err| {
            std.debug.print("SimScenes.Desc.defaults({t}) is not Jolt's default\n", .{kind});
            return err;
        };
    }
}

test "SimScenes default settings are Jolt's PhysicsSettings" {
    var jolt_settings: SimPhysicsSettings = undefined;
    jolt.jolt_sim_default_settings(&jolt_settings);
    const zolt_default: PhysicsSettings = .{};
    const zolt_settings = fromPhysicsSettings(&zolt_default);
    const scene_default: SimPhysicsSettings = .{};
    try std.testing.expectEqualSlices(u8, std.mem.asBytes(&jolt_settings), std.mem.asBytes(&zolt_settings));
    try std.testing.expectEqualSlices(u8, std.mem.asBytes(&jolt_settings), std.mem.asBytes(&scene_default));
}

test "SimulationParity state decoder" {
    // A small scene: decode the C++ state and check that every byte belongs to a field, then change one byte of the
    // Zolt copy in the linear velocity of the second body and check the report
    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    const sphere = try builder.sphere(0.5);
    _ = try builder.body(.{ .shape = sphere, .motion_type = 0 }, .dont_activate);
    _ = try builder.body(.{ .shape = sphere, .linear_velocity = .{ 1, 2, 3 } }, .activate);
    const desc = builder.desc();
    const cw = CppWorld.init(&desc, 0);
    defer cw.deinit();
    for (builder.ops.items) |*op| _ = cw.applyOp(op);
    _ = cw.update(1.0 / 60.0, 1);

    var bytes: std.ArrayList(u8) = .empty;
    defer bytes.deinit(allocator);
    try cw.saveState(allocator, .all, &bytes);

    var layout: StateLayout = .{ .allocator = allocator, .bytes = bytes.items, .cpp = cw };
    defer layout.deinit();
    try layout.decode();
    try std.testing.expect(!layout.truncated);
    try std.testing.expectEqual(bytes.items.len, layout.pos);

    const copy = try allocator.dupe(u8, bytes.items);
    defer allocator.free(copy);
    var target: ?*const Field = null;
    for (layout.fields.items) |*field| {
        if (field.item == 1 and std.mem.eql(u8, field.name, "motion_properties.linear_velocity") and field.component == 1)
            target = field;
    }
    copy[target.?.offset] ^= 1;
    const message = (try describeStateDifference(allocator, copy, bytes.items, cw)).?;
    defer allocator.free(message);
    try std.testing.expect(std.mem.indexOf(u8, message, "bodies[1]") != null);
    try std.testing.expect(std.mem.indexOf(u8, message, "motion_properties.linear_velocity.y") != null);
    try std.testing.expect(try describeStateDifference(allocator, bytes.items, bytes.items, cw) == null);
}

test "SimulationParity state decoder: constraints" {
    // The SaveState of a constraint class is decoded with the fields of its slot (`stateFields`): a byte changed in the
    // rotation part of a fixed constraint is reported with its name
    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    const box = try builder.box(.{ 0.2, 0.2, 0.2 }, 0.05);
    const a = try builder.body(.{ .shape = box, .position = .{ 0, 2, 0 } }, .activate);
    const b = try builder.body(.{ .shape = box, .position = .{ 0.5, 2, 0 }, .angular_velocity = .{ 0, 1, 0 } }, .activate);
    _ = try builder.constraint(SimScenes.Desc.pointWorld(SimScenes.invalid_handle, a, .{ 0, 2.3, 0 }));
    _ = try builder.constraint(SimScenes.Desc.fixedAuto(a, b));
    const desc = builder.desc();
    const cw = CppWorld.init(&desc, 0);
    defer cw.deinit();
    for (builder.ops.items) |*op| _ = cw.applyOp(op);
    _ = cw.update(1.0 / 60.0, 1);

    var bytes: std.ArrayList(u8) = .empty;
    defer bytes.deinit(allocator);
    try cw.saveState(allocator, .all, &bytes);
    var layout: StateLayout = .{ .allocator = allocator, .bytes = bytes.items, .cpp = cw };
    defer layout.deinit();
    try layout.decode();
    try std.testing.expect(!layout.truncated);
    try std.testing.expectEqual(bytes.items.len, layout.pos);

    const copy = try allocator.dupe(u8, bytes.items);
    defer allocator.free(copy);
    var target: ?*const Field = null;
    for (layout.fields.items) |*field| {
        if (field.item == 1 and std.mem.eql(u8, field.name, "rotation_constraint_part.total_lambda") and field.component == 2)
            target = field;
    }
    copy[target.?.offset] ^= 1;
    const message = (try describeStateDifference(allocator, copy, bytes.items, cw)).?;
    defer allocator.free(message);
    try std.testing.expect(std.mem.indexOf(u8, message, "constraints[1] (id 0x00000001) rotation_constraint_part.total_lambda.z") != null);
}

// -------------------------------------------------------------------------------------------------------------------
// Phase 7

test "SimScenes Phase 7 struct layout matches SimulationReference.cpp" {
    var sizes: [32]u32 = undefined;
    jolt.jolt_sim_phase7_struct_sizes(&sizes);
    const S = SimScenes;
    const expected = [32]u32{
        @sizeOf(S.SimCharacterBase),
        @sizeOf(S.SimCharacter),
        @sizeOf(S.SimCharacterVirtual),
        @sizeOf(S.SimRagdoll),
        @sizeOf(S.SimRagdollJoint),
        @sizeOf(S.SimRagdollAdditionalConstraint),
        @sizeOf(S.SimRagdollPose),
        @sizeOf(S.SimVehicleEngine),
        @sizeOf(S.SimVehicleTransmission),
        @sizeOf(S.SimVehicleTrack),
        @sizeOf(S.SimVehicleDifferential),
        @sizeOf(S.SimVehicleAntiRollBar),
        @sizeOf(S.SimVehicleWheel),
        @sizeOf(S.SimVehicleTester),
        @sizeOf(S.SimVehicle),
        @offsetOf(S.SimCharacter, "position"),
        @offsetOf(S.SimCharacter, "user_data"),
        @offsetOf(S.SimCharacterVirtual, "position"),
        @offsetOf(S.SimCharacterVirtual, "user_data"),
        @offsetOf(S.SimCharacterVirtual, "collision_group"),
        @offsetOf(S.SimRagdoll, "user_data"),
        @offsetOf(S.SimVehicle, "user_data"),
        @offsetOf(S.SimVehicle, "engine"),
        @offsetOf(S.SimVehicle, "tracks"),
        @offsetOf(S.SimVehicle, "tester"),
        @offsetOf(S.SimVehicle, "tire_max_impulse"),
        @offsetOf(S.SimVehicleWheel, "lateral_friction_tv"),
        @offsetOf(SimSceneDesc, "characters"),
        @offsetOf(SimSceneDesc, "vehicles"),
        @offsetOf(SimSceneDesc, "uints"),
        @sizeOf(SimSceneDesc),
        @offsetOf(S.SimRagdollPose, "data"),
    };
    try std.testing.expectEqualSlices(u32, &expected, &sizes);
    // The header of an object state field: four u16 (SimObjectStateWriter)
    try std.testing.expectEqual(@as(usize, 8), @sizeOf(ObjectFieldHeader));
}

/// Compare two extern structs field by field (the bytes of the values, not the padding)
fn expectEqualFields(comptime T: type, expected: T, actual: T) !void {
    inline for (@typeInfo(T).@"struct".fields) |field| {
        const e = @field(expected, field.name);
        const a = @field(actual, field.name);
        switch (@typeInfo(field.type)) {
            .@"struct" => try expectEqualFields(field.type, e, a),
            .array => |info| if (@typeInfo(info.child) == .@"struct") {
                for (e, a) |ee, aa| try expectEqualFields(info.child, ee, aa);
            } else if (!std.mem.eql(u8, std.mem.asBytes(&e), std.mem.asBytes(&a))) {
                std.debug.print("{s}.{s}: expected {any}, got {any}\n", .{ @typeName(T), field.name, e, a });
                return error.TestExpectedEqual;
            },
            else => if (!std.mem.eql(u8, std.mem.asBytes(&e), std.mem.asBytes(&a))) {
                std.debug.print("{s}.{s}: expected {any}, got {any}\n", .{ @typeName(T), field.name, e, a });
                return error.TestExpectedEqual;
            },
        }
    }
}

test "SimScenes Phase 7 description defaults are Jolt's" {
    // The descriptions that the scenes start from have the defaults of Jolt's settings classes (CharacterSettings,
    // CharacterVirtualSettings, VehicleConstraintSettings with the controller settings of each kind, WheelSettingsWV /
    // TV, ExtendedUpdateSettings), bit for bit
    var character: SimScenes.SimCharacter = undefined;
    jolt.jolt_sim_default_character(&character);
    try expectEqualFields(SimScenes.SimCharacter, .{}, character);
    var character_virtual: SimScenes.SimCharacterVirtual = undefined;
    jolt.jolt_sim_default_character_virtual(&character_virtual);
    try expectEqualFields(SimScenes.SimCharacterVirtual, .{}, character_virtual);
    var extended_update: [12]f32 = undefined;
    jolt.jolt_sim_default_extended_update(&extended_update);
    const zolt_extended_update = (SceneBuilder.ExtendedUpdateSettings{}).toFloats();
    try std.testing.expectEqualSlices(u8, std.mem.asBytes(&extended_update), std.mem.asBytes(&zolt_extended_update));
    for (std.enums.values(VehicleControllerKind)) |kind| {
        var vehicle: SimScenes.SimVehicle = undefined;
        jolt.jolt_sim_default_vehicle(@intFromEnum(kind), &vehicle);
        expectEqualFields(SimScenes.SimVehicle, SimScenes.SimVehicle.defaults(kind), vehicle) catch |err| {
            std.debug.print("SimVehicle.defaults({t}) is not Jolt's default\n", .{kind});
            return err;
        };
    }
    for (std.enums.values(SimScenes.VehicleWheelKind)) |kind| {
        var wheel: SimScenes.SimVehicleWheel = undefined;
        jolt.jolt_sim_default_vehicle_wheel(@intFromEnum(kind), &wheel);
        expectEqualFields(SimScenes.SimVehicleWheel, SimScenes.SimVehicleWheel.defaults(kind), wheel) catch |err| {
            std.debug.print("SimVehicleWheel.defaults({t}) is not Jolt's default\n", .{kind});
            return err;
        };
    }
}

test "SimulationParity Phase 7 slots follow the ported classes" {
    // A slot is filled when the classes of its group are ported: removing `is_stub` from a class requires its slot
    // (the recipe in the SimScenes header), and a filled slot needs the ported classes
    try std.testing.expectEqual(Phase7Classes.characters(), !@hasDecl(CharacterSlot, "is_stub"));
    try std.testing.expectEqual(Phase7Classes.characterVirtuals(), !@hasDecl(CharacterVirtualSlot, "is_stub"));
    try std.testing.expectEqual(Phase7Classes.ragdolls(), !@hasDecl(RagdollSlot, "is_stub"));
    try std.testing.expectEqual(Phase7Classes.wheeledVehicles(), !@hasDecl(VehicleSlot, "is_stub"));
    try std.testing.expectEqual(Phase7Classes.trackedVehicles(), !@hasDecl(TrackedVehicleSlot, "is_stub"));
    try std.testing.expectEqual(Phase7Classes.motorcycles(), !@hasDecl(MotorcycleSlot, "is_stub"));
}

test "SimulationParity CharacterID consumption" {
    // CharacterVirtualSettings' default initializer and CharacterVirtual::GetCharacterVirtualSettings consume a
    // CharacterID, the copy constructor and the assignment do not (CharacterVehicleArchitecture "Settled in the foundation
    // review"; the IDs of the M8 scene depend on it): Jolt gives SimScenes.characterIdSequence, and the Zolt side of
    // CharacterVirtualSlot the same once the class is ported
    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    try SimScenes.buildCharacterIdTestScene(&builder);
    const desc = builder.desc();
    const first_id = 100;
    const cw = CppWorld.init(&desc, 0);
    defer cw.deinit();
    var jolt_values: [SimScenes.character_id_sequence_len]u32 = undefined;
    jolt.jolt_sim_character_id_sequence(cw.handle, first_id, &jolt_values);
    try std.testing.expectEqual(SimScenes.characterIdSequence(first_id), jolt_values);
    if (!@hasDecl(CharacterVirtualSlot, "is_stub")) {
        const zw = try ZoltWorld.create(allocator, &desc, .single_threaded);
        defer zw.destroy();
        var zolt_values: [SimScenes.character_id_sequence_len]u32 = undefined;
        try CharacterVirtualSlot.characterIdSequence(zw, first_id, &zolt_values);
        try std.testing.expectEqual(jolt_values, zolt_values);
    }
}

test "SimulationParity a character record that cannot be stored fails the operation" {
    // The CharacterContactListener callbacks run inside the character virtual operations and cannot return errors:
    // Recorder.add latches an out of memory, and ZoltWorld.applyOp reports it, also when nobody takes the character
    // records afterwards (createScene under std.testing.checkAllAllocationFailures)
    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    try SimScenes.buildCharacterIdTestScene(&builder);
    const desc = builder.desc();
    const world = try ZoltWorld.create(allocator, &desc, .single_threaded);
    defer world.destroy();
    var failing: std.testing.FailingAllocator = .init(allocator, .{ .fail_index = 0 });
    world.character_recorder.allocator = failing.allocator();
    world.character_recorder.add(.{ .kind = @intFromEnum(RecordKind.character_contact_added) });
    world.character_recorder.allocator = allocator;
    const op: SimOp = .{ .update = 0, .kind = @intFromEnum(OpKind.set_gravity), .f = .{ 0, -9.81, 0 } ++ @as([13]f32, @splat(0)) };
    try std.testing.expectError(error.OutOfMemory, world.applyOp(&op));
}

test "SimulationParity restoring the character states reads every byte" {
    // restore_cross restores the character states of the other side: a restore that leaves bytes unread fails on both
    // sides (a stream sets eof only on a read past the end, so both read one more byte); no bytes restore a world
    // without characters
    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    try SimScenes.buildCharacterIdTestScene(&builder);
    const desc = builder.desc();
    const cw = CppWorld.init(&desc, 0);
    defer cw.deinit();
    const zw = try ZoltWorld.create(allocator, &desc, .single_threaded);
    defer zw.destroy();
    const unread = [_]u8{ 1, 2, 3 };
    try std.testing.expect(!cw.restoreCharacterStates(&unread));
    try std.testing.expect(!try zw.restoreCharacterStates(&unread));
    try std.testing.expect(cw.restoreCharacterStates(&.{}));
    try std.testing.expect(try zw.restoreCharacterStates(&.{}));
}

test "SimulationParity the reference counts its asserts" {
    // With -Dreference_asserts=true the hook of SimulationReference.cpp prints and counts a failed Jolt assert (the
    // scenes check the count), without asserts nothing is counted
    var before: u32 = undefined;
    var after: u32 = undefined;
    jolt.jolt_sim_test_assert_hook(&before, &after);
    try std.testing.expectEqual(before + jolt.jolt_sim_reference_has_asserts(), after);
}

test "SimulationParity the reference zero-fills its allocations" {
    // The allocation hooks of the parity process (ZoltParity/ParityHooks.h): Jolt's members that are
    // never initialized (Wheel's contact members before the first contact) are 0, as Zolt zero-initializes them
    try std.testing.expectEqual(@as(u32, 1), jolt.jolt_sim_zero_filling_check());
}

test "SimulationParity Phase 7 record helpers" {
    // The layouts of RecordKind 7..20 (SimCharacterContactListener / SimVehicleRecord of SimulationReference.cpp)
    try std.testing.expectEqual(@as(u32, 2 | (1 << 8) | (1 << 13)), CharacterRecords.contactFlags(2, true, false, false, false, false, true));
    try std.testing.expectEqual(@as(u32, (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12)), CharacterRecords.contactFlags(0, false, true, true, true, true, false));
    try std.testing.expectEqual(@as(u32, 2), CharacterRecords.settingsBits(false, true));
    const contact = CharacterRecords.contact(.character_contact_added, 1, .invalid, 2, .empty, Vec3.init(0, 1, 0), -0.01, 0.5, 9, 3, 77, 1);
    try std.testing.expectEqual(@as(u32, 8), contact.kind);
    try std.testing.expectEqual([4]u32{ 1, 0xffffffff, 2, 0xffffffff }, contact.ids);
    try std.testing.expectEqual(@as(u32, 9 | (3 << 16)), contact.data[5]);
    try std.testing.expectEqual(@as(u32, 0x3f800000), contact.data[1]);
    const removed = CharacterRecords.removed(.character_vs_character_contact_removed, 1, .invalid, 4, .empty);
    try std.testing.expectEqual([4]u32{ 1, 0xffffffff, 4, 0xffffffff }, removed.ids);
    var buffer: [256]u8 = undefined;
    const text = try std.fmt.bufPrint(&buffer, "{f}", .{RecordText{ .record = removed }});
    try std.testing.expect(std.mem.startsWith(u8, text, "character_vs_character_contact_removed ids { 1, 4294967295, 4, 4294967295 } data "));
    try std.testing.expectEqualStrings("nothing", try std.fmt.bufPrint(&buffer, "{f}", .{RecordText{ .record = null }}));
    const v = Vec3.init(1, 2, 3);
    const solve = CharacterRecords.solve(.character_contact_solve, 1, .invalid, 0xffffffff, .empty, RVec3.init(1, 0, 0), v, v, 5, v, v, v);
    try std.testing.expectEqual(solve.data[7], CharacterRecords.hashVec3(SimScenes.hash_seed, v));
    const adjust = CharacterRecords.adjustBodyVelocity(1, .invalid, v, v, v, v);
    try std.testing.expectEqual(@as(u32, @intFromEnum(RecordKind.character_adjust_body_velocity)), adjust.kind);
    const context: PhysicsStepListenerContext = .{ .delta_time = 0.5, .is_first_step = true, .is_last_step = false, .physics_system = undefined };
    const record = VehicleRecords.record(.vehicle_post_step, 3, .invalid, &context, 0b101, VehicleRecords.hashWheel(SimScenes.hash_seed, 0.4, 2, 0));
    try std.testing.expectEqual([8]u32{ 0x3f000000, 1, 0, 0b101, record.data[4], 0, 0, 0 }, record.data);
}

test "SimulationParity object state decoder" {
    // The object state of a C++ world with every kind of object decodes completely (the layouts of objectFieldLayout and
    // the SaveState fields add up to the sizes of the fields that SimulationReference.cpp writes); a byte changed in a
    // value of the Zolt copy is reported with the field, the object handle, the item and the component
    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    try SimScenes.buildObjectStateTestScene(&builder);
    const desc = builder.desc();
    const cw = CppWorld.init(&desc, 0);
    defer cw.deinit();
    for (builder.ops.items) |*op| _ = cw.applyOp(op);
    for (0..3) |_| _ = cw.update(1.0 / 60.0, 1);
    var bytes: std.ArrayList(u8) = .empty;
    defer bytes.deinit(allocator);
    try cw.objectState(allocator, &bytes);

    var layout: ObjectStateLayout = .{ .allocator = allocator };
    defer layout.deinit();
    try layout.decode(bytes.items);
    try std.testing.expect(layout.complete);
    try std.testing.expect(layout.firstNotFinite(bytes.items) == null);

    // Every field kind of the scene is there
    var seen: std.EnumSet(ObjectField) = .initEmpty();
    var it: ObjectFieldIterator = .{ .bytes = bytes.items };
    while (try it.next()) |view| seen.insert(view.field().?);
    for ([_]ObjectField{ .character_save_state, .character_properties, .character_base, .character_body, .character_transformed_shape, .cv_save_state, .cv_properties, .cv_base, .cv_transforms, .cv_transformed_shape, .cv_num_active_contacts, .cv_active_contact, .ragdoll_properties, .ragdoll_body, .ragdoll_root_transform, .ragdoll_pose_root_offset, .ragdoll_pose_joint, .ragdoll_bounds, .ragdoll_constraint, .vehicle_properties, .vehicle_wheel, .vehicle_wheel_contact, .vehicle_wheel_transform, .vehicle_wheel_wv, .vehicle_anti_roll_bar, .vehicle_engine, .vehicle_transmission, .vehicle_wheeled, .vehicle_differential }) |field| {
        if (!seen.contains(field)) {
            std.debug.print("the object state has no {t} field\n", .{field});
            return error.TestUnexpectedResult;
        }
    }

    // A changed contact normal of the active contact of the character virtual
    const copy = try allocator.dupe(u8, bytes.items);
    defer allocator.free(copy);
    var target: ?*const ObjectValue = null;
    for (layout.values.items) |*value| {
        if (value.header.field == @intFromEnum(ObjectField.cv_active_contact) and std.mem.eql(u8, value.name, "contact_normal") and value.index == 1)
            target = value;
    }
    copy[target.?.offset] ^= 1;
    const message = (try describeObjectStateDifference(allocator, copy, bytes.items, .{})).?;
    defer allocator.free(message);
    try std.testing.expect(std.mem.indexOf(u8, message, "cv_active_contact (object handle 0, item 0) contact_normal.y zolt ") != null);

    // A changed ground state in the SaveState bytes of the character
    @memcpy(copy, bytes.items);
    var save_state: ?*const ObjectValue = null;
    for (layout.values.items) |*value| {
        if (value.header.field == @intFromEnum(ObjectField.character_save_state) and std.mem.eql(u8, value.name, "ground_state"))
            save_state = value;
    }
    copy[save_state.?.offset] ^= 2;
    const message2 = (try describeObjectStateDifference(allocator, copy, bytes.items, .{ .checked = "3 threads", .reference = "0 threads" })).?;
    defer allocator.free(message2);
    try std.testing.expect(std.mem.indexOf(u8, message2, "character_save_state (object handle 0, item 0) ground_state 3 threads ") != null);

    // A missing field
    const message3 = (try describeObjectStateDifference(allocator, bytes.items[0 .. bytes.items.len - 1], bytes.items, .{})).?;
    defer allocator.free(message3);
    try std.testing.expect(try describeObjectStateDifference(allocator, bytes.items, bytes.items, .{}) == null);

    // The scan for the Expect fields
    var stats: ObjectStats = .{};
    try scanObjectState(bytes.items, &stats);
    try std.testing.expect(stats.wheel_contacts);
    try std.testing.expect(stats.ground_states != 0);
}

// -------------------------------------------------------------------------------------------------------------------
// Phase 9: soft bodies

test "SimScenes soft body struct layout matches SimulationReference.cpp" {
    var sizes: [8]u32 = undefined;
    jolt.jolt_sim_soft_body_struct_sizes(&sizes);
    const expected = [8]u32{
        @sizeOf(SimScenes.SimSoftBodySettings),
        @sizeOf(SimScenes.SimSoftBody),
        @offsetOf(SimScenes.SimSoftBody, "position"),
        @offsetOf(SimScenes.SimSoftBody, "settings"),
        @offsetOf(SimScenes.SimSoftBody, "faces_double_sided"),
        @offsetOf(SimScenes.SimSoftBodySettings, "back_stop_radius"),
        @offsetOf(SimSceneDesc, "soft_body_settings"),
        @sizeOf(SimSceneDesc),
    };
    try std.testing.expectEqualSlices(u32, &expected, &sizes);
}

test "SimScenes soft body description defaults are Jolt's" {
    // SoftBodyCreationSettings, and the VertexAttributes, the bend type of CreateConstraints and Skinned of the generators
    var soft_body: SimScenes.SimSoftBody = undefined;
    jolt.jolt_sim_default_soft_body(&soft_body);
    try expectEqualFields(SimScenes.SimSoftBody, .{}, soft_body);
    var settings: SimScenes.SimSoftBodySettings = .{ .kind = 0 };
    jolt.jolt_sim_default_soft_body_settings(&settings);
    try expectEqualFields(SimScenes.SimSoftBodySettings, .{ .kind = 0 }, settings);
}

/// A scene of the table by name
fn sceneByName(name: []const u8) *const SceneEntry {
    for (&SimScenes.scenes) |*entry| {
        if (std.mem.eql(u8, entry.name, name)) return entry;
    }
    unreachable;
}

test "SimulationParity state decoder: soft bodies" {
    // soft_body_ops on the C++ side (soft bodies with vertices, rod states and skin states) for a few updates: the decoder
    // reads every byte of the state with the array sizes of jolt_sim_soft_body_info, its soft body fields hold the values
    // of the C++ getters (vertices, rod states, local bounds), and a changed byte is reported with its vertex and member.
    // Zolt's SaveState of a soft body is group S4's: the decoder is checked against the C++ bytes only.
    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    try sceneByName("soft_body_ops").build(&builder);
    const desc = builder.desc();
    const cw = CppWorld.init(&desc, 0);
    defer cw.deinit();
    for (0..3) |update| {
        for (builder.ops.items) |*op| {
            if (op.update != update) continue;
            switch (@as(OpKind, @enumFromInt(op.kind))) {
                .set_update, .snapshot, .restore_cross => {},
                else => _ = cw.applyOp(op),
            }
        }
        _ = cw.update(builder.delta_time, builder.collision_steps);
    }

    var bytes: std.ArrayList(u8) = .empty;
    defer bytes.deinit(allocator);
    try cw.saveState(allocator, .all, &bytes);
    var layout: StateLayout = .{ .allocator = allocator, .bytes = bytes.items, .cpp = cw };
    defer layout.deinit();
    try layout.decode();
    try std.testing.expect(!layout.truncated);
    try std.testing.expectEqual(bytes.items.len, layout.pos);

    // Every soft body: the decoded vertices, rod states and local bounds are the values of the C++ getters
    var num_soft_bodies: u32 = 0;
    var arrays_reached: [3]bool = @splat(false);
    var decoded: std.ArrayList(f32) = .empty;
    defer decoded.deinit(allocator);
    var values: std.ArrayList(f32) = .empty;
    defer values.deinit(allocator);
    for (layout.fields.items) |*id_field| {
        if (!std.mem.eql(u8, id_field.section, "bodies") or !std.mem.eql(u8, id_field.name, "id")) continue;
        const id = id_field.id.?;
        const counts = cw.softBodyInfo(id) orelse continue;
        num_soft_bodies += 1;
        for (counts, &arrays_reached) |count, *reached| reached.* = reached.* or count > 0;
        decoded.clearRetainingCapacity();
        for (layout.fields.items) |*field| {
            if (field.id != id or !std.mem.startsWith(u8, field.name, "soft_body.")) continue;
            if (std.mem.eql(u8, field.name, "soft_body.skin_state") or std.mem.startsWith(u8, field.name, "soft_body.local_predicted_bounds")) continue;
            try decoded.append(allocator, std.mem.bytesToValue(f32, bytes.items[field.offset..][0..4]));
        }
        const size = jolt.jolt_sim_soft_body_values(cw.handle, id, values.items.ptr, 0);
        try values.resize(allocator, size);
        try std.testing.expectEqual(size, jolt.jolt_sim_soft_body_values(cw.handle, id, values.items.ptr, size));
        try std.testing.expectEqualSlices(u8, std.mem.sliceAsBytes(values.items), std.mem.sliceAsBytes(decoded.items));
    }
    try std.testing.expectEqual(@as(u32, 5), num_soft_bodies); // The sphere of CustomUpdate is not in the system
    try std.testing.expect(arrays_reached[0] and arrays_reached[1] and arrays_reached[2]);

    // A changed byte of the velocity of vertex 5 of the first soft body
    const copy = try allocator.dupe(u8, bytes.items);
    defer allocator.free(copy);
    var target: ?*const Field = null;
    for (layout.fields.items) |*field| {
        if (target == null and std.mem.eql(u8, field.name, "soft_body.vertices") and field.sub_item == 5 and std.mem.eql(u8, field.member, "velocity") and field.component == 1)
            target = field;
    }
    copy[target.?.offset] ^= 1;
    const message = (try describeStateDifference(allocator, copy, bytes.items, cw)).?;
    defer allocator.free(message);
    try std.testing.expect(std.mem.indexOf(u8, message, "soft_body.vertices[5].velocity.y zolt ") != null);
}

test "SimulationParity soft body listener records" {
    // The Zolt listener on a hand-made manifold (the C++ listener makes the same records from the same callbacks; the
    // soft body scenes compare them once Zolt simulates soft bodies): vertices 0 and 3 touch body 9 through colliding
    // shape 1, vertex 1 has a CCD contact with body 5, vertex 2 none; one sensor (body 11)
    const allocator = std.testing.allocator;
    var recorder: Recorder = .{ .allocator = allocator };
    defer recorder.deinit();
    var listener: SoftBodyListenerImpl = .{ .recorder = &recorder, .mode = .accept };

    var vertices: [4]zolt.SoftBodyVertex = @splat(std.mem.zeroes(zolt.SoftBodyVertex));
    for (&vertices) |*v| v.resetCollision();
    for ([_]usize{ 0, 3 }) |i| {
        vertices[i].position = Vec3.init(2, 3, @floatFromInt(i));
        vertices[i].collision_plane = zolt.Plane.init(Vec3.axisY(), -1.0);
        vertices[i].colliding_shape_index = 1;
        vertices[i].has_contact = true;
    }
    vertices[1].markCCDContact(.init(5), zolt.Plane.init(Vec3.axisX(), 0.0));
    const CollidingShape = zolt.SoftBodyMotionProperties.CollidingShape;
    const shape: CollidingShape = .{ .center_of_mass_transform = Mat44.identity(), .body_id = .init(7), .motion_type = .dynamic, .inv_mass = 1, .friction = 0, .restitution = 0, .soft_body_inv_mass_scale = 1, .update_velocities = false, .inv_inertia = Mat44.identity(), .linear_velocity = .zero(), .angular_velocity = .zero(), .original_linear_velocity = .zero(), .original_angular_velocity = .zero() };
    var shapes = [_]CollidingShape{ shape, shape };
    shapes[1].body_id = .init(9);
    const sensors = [_]zolt.SoftBodyMotionProperties.CollidingSensor{.{ .center_of_mass_transform = Mat44.identity(), .body_id = .init(11), .has_contact = true }};
    const manifold: zolt.SoftBodyManifold = .{ .vertices = &vertices, .colliding_shapes = &shapes, .colliding_sensors = &sensors };
    const soft_body: Body = .{ .id = .init(3) };
    const other_body: Body = .{ .id = .init(9) };

    listener.onSoftBodyContactAdded(&soft_body, &manifold);
    const records = recorder.records.items;
    try std.testing.expectEqual(@as(usize, 4), records.len);
    try std.testing.expectEqual(@as(u32, @intFromEnum(RecordKind.soft_body_contact_added)), records[0].kind);
    try std.testing.expectEqualSlices(u32, &.{ 3, 4, 3, 1 }, &records[0].ids);
    try std.testing.expectEqual(@as(u32, @intFromEnum(RecordKind.soft_body_contact_body)), records[1].kind);
    try std.testing.expectEqualSlices(u32, &.{ 3, 9, 2, 0 }, &records[1].ids); // Vertices 0 and 3
    try std.testing.expectEqualSlices(u32, &.{ 3, 5, 1, 0 }, &records[2].ids); // The CCD contact of vertex 1
    try std.testing.expectEqual(@as(u32, @intFromEnum(RecordKind.soft_body_sensor_contact)), records[3].kind);
    try std.testing.expectEqualSlices(u32, &.{ 3, 11, 0, 0 }, &records[3].ids);
    // The hash of body 9 holds the local contact point (2, 1, z) and the normal (0, -1, 0) of vertex 0, then of vertex 3
    var expected_hash = SimScenes.hash_seed;
    for ([_]usize{ 0, 3 }) |i| {
        const p = manifold.getLocalContactPoint(&vertices[i]);
        const n = manifold.getContactNormal(&vertices[i]);
        try std.testing.expect(p.eql(Vec3.init(2, 1, @floatFromInt(i))) and n.eql(Vec3.init(0, -1, 0)));
        expected_hash = hashValue(expected_hash, [6]f32{ p.getX(), p.getY(), p.getZ(), n.getX(), n.getY(), n.getZ() });
    }
    try std.testing.expectEqual(expected_hash, records[1].data[0]);

    // OnSoftBodyContactValidate in every mode: the settings before and after, the result
    const one = floatBits(1.0);
    for ([_]SimScenes.SoftBodyListenerMode{ .accept, .reject, .sensor, .mass_scales }) |mode| {
        recorder.records.clearRetainingCapacity();
        listener.mode = mode;
        var settings: zolt.SoftBodyContactSettings = .{ .is_sensor = false };
        const result = listener.onSoftBodyContactValidate(&soft_body, &other_body, &settings);
        const r = recorder.records.items[0];
        try std.testing.expectEqual(@as(u32, @intFromEnum(RecordKind.soft_body_contact_validate)), r.kind);
        try std.testing.expectEqualSlices(u32, &.{ 3, 9, @intFromBool(mode == .reject), 0 }, &r.ids);
        try std.testing.expectEqual(mode == .reject, result == .reject_contact);
        try std.testing.expectEqualSlices(u32, &.{ one, one, one, 0 }, r.data[0..4]);
        const after: [4]u32 = switch (mode) {
            .sensor => .{ one, one, one, 1 },
            .mass_scales => .{ floatBits(0.5), floatBits(2.0), floatBits(1.5), 0 },
            else => .{ one, one, one, 0 },
        };
        try std.testing.expectEqualSlices(u32, &after, r.data[4..8]);
    }
}

test "SimulationParity soft body settings generators" {
    // Every generator kind and option makes the bytes of SimMakeSoftBodySettings, also for settings that are not optimized
    // and for every bend type and LRA type (checkSoftBodySettings of every soft body scene runs the same comparison)
    const allocator = std.testing.allocator;
    const K = SimScenes.SoftBodySettingsKind;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    for ([_]u32{ 0, 1 }) |optimize| {
        for ([_]u32{ 0, 1, 2 }) |variant| {
            _ = try builder.softBodySettings(.{ .kind = @intFromEnum(K.cloth), .count_x = 5, .count_z = 4, .spacing = 0.25, .compliance = 1.0e-4, .shear_compliance = 2.0e-4, .bend_compliance = 1.0e-3, .lra_type = variant, .lra_max_distance_multiplier = 1.1, .bend_type = variant, .optimize = optimize, .kinematic = variant });
            _ = try builder.softBodySettings(.{ .kind = @intFromEnum(K.sphere), .count_x = 5, .count_z = 7, .spacing = 0.6, .compliance = 1.0e-4, .bend_compliance = 1.0e-2, .bend_type = variant, .optimize = optimize });
        }
        _ = try builder.softBodySettings(.{ .kind = @intFromEnum(K.rod_chain), .count_x = 7, .spacing = 0.3, .height = 2, .compliance = 1.0e-5, .shear_compliance = 1.0e-3, .kinematic = optimize, .optimize = optimize });
        _ = try builder.softBodySettings(.{ .kind = @intFromEnum(K.skinned_cloth), .count_x = 4, .count_z = 9, .count_extra = 4, .spacing = 0.15, .kinematic = 2, .skin_max_distance = 0.2, .back_stop_distance = 0.01, .back_stop_radius = 5, .optimize = optimize });
        _ = try builder.explicitSoftBodySettings(.{ .kind = @intFromEnum(K.explicit), .compliance = 1.0e-3, .bend_type = 2, .lra_type = 2, .optimize = optimize }, &.{
            .{ 0, 0, 0, 0, 0, 0, 0 },
            .{ 1, 0, 0, 0, 1, 0, 1 },
            .{ 0, 0, 1, 0, 0, 0, 1 },
            .{ 1, 0.2, 1, 0, 0, 1, 2 },
            .{ 0.5, 1, 0.5, 0, 0, 0, 1 },
        }, &.{ .{ 0, 1, 2 }, .{ 1, 3, 2 }, .{ 0, 4, 1 } }, &.{ .{ 0, 1, 2, 4 }, .{ 1, 3, 2, 4 } });
    }
    _ = try builder.softBodySettings(.{ .kind = @intFromEnum(K.cube), .count_x = 3, .spacing = 0.5 });
    try checkSoftBodySettings(allocator, "(the generators test)", &builder);
}

/// The bits of the soft body operations in `ops` (bit k for OpKind k). The other operations of a soft body scene
/// (constraint and object operations from 200 on) do not fit in the mask, so they are skipped.
fn softBodyOpBits(ops: []const SimOp) u128 {
    var bits: u128 = 0;
    for (ops) |op| {
        if (SimScenes.isSoftBodyOp(op.kind)) bits |= @as(u128, 1) << @intCast(op.kind);
    }
    return bits;
}

test "SimScenes softBodyOpBits skips the other operations" {
    // A soft body scene may hold body, constraint, object and driver operations too.
    const ops = [_]SimOp{
        .{ .update = 0, .kind = @intFromEnum(OpKind.create_body) },
        .{ .update = 0, .kind = @intFromEnum(OpKind.create_soft_body) },
        .{ .update = 0, .kind = @intFromEnum(OpKind.create_constraint_no_add) },
        .{ .update = 1, .kind = @intFromEnum(OpKind.set_soft_body_contact_listener) },
        .{ .update = 1, .kind = @intFromEnum(OpKind.motorcycle_set_lean_smoothing_factor) },
    };
    const expected = (@as(u128, 1) << @intFromEnum(OpKind.create_soft_body)) | (@as(u128, 1) << @intFromEnum(OpKind.set_soft_body_contact_listener));
    try std.testing.expectEqual(expected, softBodyOpBits(&ops));
}

test "SimScenes the soft body scenes use every soft body operation and record" {
    // Together the scenes with soft bodies use every soft body OpKind and expect every soft body RecordKind.
    const allocator = std.testing.allocator;
    var ops: u128 = 0;
    var records: u128 = 0;
    var num_scenes: u32 = 0;
    for (&SimScenes.scenes) |*entry| {
        var builder: SceneBuilder = .init(allocator);
        defer builder.deinit();
        try entry.build(&builder);
        if (builder.soft_bodies.items.len == 0) continue;
        num_scenes += 1;
        ops |= softBodyOpBits(builder.ops.items);
        records |= entry.expect.soft_body_records;
    }
    for (@intFromEnum(OpKind.create_soft_body)..@intFromEnum(OpKind.set_soft_body_contact_listener) + 1) |k| {
        try std.testing.expect(SimScenes.isSoftBodyOp(@intCast(k)));
        if (ops & (@as(u128, 1) << @intCast(k)) == 0) {
            std.debug.print("SimScenes: no soft body scene uses OpKind {t}\n", .{@as(OpKind, @enumFromInt(k))});
            return error.TestUnexpectedResult;
        }
    }
    for ([_]RecordKind{ .soft_body_contact_validate, .soft_body_contact_added, .soft_body_contact_body, .soft_body_sensor_contact }) |kind|
        try std.testing.expect(records & SimScenes.recordBit(kind) != 0);
    // Groups SH and S5 wrote 23 soft body scenes
    try std.testing.expect(num_scenes >= 23);
}

test "SimulationParity soft body creation, creation settings and skin flags" {
    // The parts of the soft body harness that a scene run does not check, in both worlds of soft_body_ops.
    // The create operations and the listener operation of update 0 give the same results.
    // The GetSoftBodyCreationSettings round trip gives the same hash for every soft body.
    // SkinFlags puts the skin flags of a capture back.
    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    try sceneByName("soft_body_ops").build(&builder);
    const desc = builder.desc();
    const zw = try ZoltWorld.create(allocator, &desc, .single_threaded);
    defer zw.destroy();
    const cw = CppWorld.init(&desc, 0);
    defer cw.deinit();

    var num_soft_bodies: u32 = 0;
    for (builder.ops.items) |*op| {
        if (op.update != 0) break;
        const kind: OpKind = @enumFromInt(op.kind);
        switch (kind) {
            .create_soft_body, .create_and_add_soft_body, .create_soft_body_no_add => num_soft_bodies += 1,
            .create_body, .set_soft_body_contact_listener => {},
            else => continue, // The other soft body operations return the hash of the SaveState of the soft body (S4)
        }
        try std.testing.expectEqual(cw.applyOp(op), try zw.applyOp(op));
    }
    try std.testing.expectEqual(@as(u32, 6), num_soft_bodies);
    try std.testing.expectEqual(SimScenes.SoftBodyListenerMode.accept, zw.soft_body.listener.mode);

    // GetSoftBodyCreationSettings of every soft body (the soft bodies use 6 of the 7 settings objects)
    var num_hashes: u32 = 0;
    for (zw.handles.items, 0..) |id, handle| {
        const body = zw.system.body_manager.tryGetBody(id) orelse continue;
        if (!body.isSoftBody()) continue;
        const op: SimOp = .{ .update = 0, .kind = @intFromEnum(OpKind.soft_body_creation_settings_hash), .body = @intCast(handle) };
        try std.testing.expectEqual(cw.applyOp(&op), try zw.applyOp(&op));
        num_hashes += 1;
    }
    try std.testing.expectEqual(num_soft_bodies, num_hashes);

    // The skin flags: captured, changed, put back
    var flags: SkinFlags = .{};
    defer flags.deinit(allocator);
    var skinned: ?*Body = null;
    for (zw.handles.items) |id| {
        const body = zw.system.body_manager.tryGetBodyMut(id) orelse continue;
        if (body.isSoftBody() and zolt.SoftBodyMotionProperties.fromBaseConst(body.getMotionProperties()).skin_state.items.len > 0)
            skinned = body;
    }
    const mp = zolt.SoftBodyMotionProperties.fromBase(skinned.?.getMotionPropertiesMut());
    mp.skin_state_previous_position_valid = true;
    try flags.capture(allocator, zw);
    try std.testing.expectEqual(@as(usize, 6), flags.flags.items.len);
    mp.skin_state_previous_position_valid = false;
    flags.restore(zw);
    try std.testing.expect(mp.skin_state_previous_position_valid);
}
