# Character, ragdoll and vehicle architecture (Phase 7)

This is the binding design for Phase 7:
- `Jolt/Skeleton`;
- `Jolt/Physics/Ragdoll`;
- `Jolt/Physics/Character`;
- `Jolt/Physics/Vehicle`.

It adds to [PortingGuide.md](PortingGuide.md), [CollisionArchitecture.md](CollisionArchitecture.md) and
[SimulationArchitecture.md](SimulationArchitecture.md); read those first. Section 9 of SimulationArchitecture has
the constraint porter template and the harness recipes, which Phase 7 reuses. Every decision below is an instruction for porters.
A porter who finds a decision wrong reports it with the evidence instead of working around it.

## 1. Scope

| Group | Jolt files | Tests | Lines (C++, `wc -l`) |
|---|---|---|---|
| Skeleton | `Skeleton/Skeleton`, `SkeletalAnimation`, `SkeletonPose`, `SkeletonMapper` | (none in UnitTests) | 962 |
| Ragdoll | `Physics/Ragdoll/Ragdoll` | (none in UnitTests) | 1038 |
| Character base | `Physics/Character/CharacterID`, `CharacterBase` | (none) | 318 |
| Character | `Physics/Character/Character` | (none) | 513 |
| CharacterVirtual | `Physics/Character/CharacterVirtual` | `UnitTests/Physics/CharacterVirtualTests.cpp`, `UnitTests/LoggingCharacterContactListener.h` | 2778 |
| Vehicle core | `Physics/Vehicle/VehicleEngine`, `VehicleTransmission`, `VehicleDifferential`, `VehicleTrack`, `VehicleAntiRollBar`, `Wheel`, `VehicleController`, `VehicleConstraint`, `VehicleCollisionTester`, `WheeledVehicleController` | `UnitTests/Physics/WheeledVehicleTests.cpp` | 3662 |
| Vehicle variants | `Physics/Vehicle/TrackedVehicleController`, `MotorcycleController` | (none) | 1141 |

The lines are `wc -l` of the `.h`, `.inl` and `.cpp` files in the "Jolt files" column. The tests are not counted.

**Milestone M8.** ZoltPerformanceTest runs `PerformanceTest/CharacterVirtualScene.h`. It reproduces the hashes of
`determinism_check.yml` with `-q=Discrete -t=max -s=CharacterVirtual`:
- `0xc1d4e05b164b1221` in single precision;
- `0xa0b16727aa16fdb0` in double precision.

The hashes must also hold with `-t=1` and `-t=2`, and Zolt's CI checks them. The scene needs no assets.

`PerformanceTest/RagdollScene.h` loads `Assets/Human.tof` through ObjectStreamIn. The Ragdoll hash is therefore M9
(Phase 8); Phase 7 ports and checks the Ragdoll class without it. M9 was reached in Phase 8.

## 2. Decisions

### E1 Serialization

- ObjectStream was Phase 8. In Phase 7 `JPH_DECLARE_SERIALIZABLE_*` / `JPH_IMPLEMENT_SERIALIZABLE_*` became a
  `// TODO(serialization)` comment, which Phase 8 replaced with a `pub const rtti = RTTI.declare(...)` declaration.
  New code declares `rtti` at once ([SerializationArchitecture.md](SerializationArchitecture.md) sections 2 and 3).
- Everything Jolt streams through `StreamOut` / `StreamIn` is ported now, byte compatible:
  - `SaveBinaryState`, `RestoreBinaryState`, `sRestoreFromBinaryState` of Skeleton, SkeletalAnimation,
    RagdollSettings, VehicleConstraintSettings, the controller and wheel settings, engine, transmission,
    differential, track and anti-roll bar.
  - Everything Jolt records through `StateRecorder` (`SaveState` / `RestoreState`).
- Restoring a class by its RTTI hash goes through `RegisterTypes.factory`, in the order of Jolt's `RegisterTypes.cpp`
  (E3).

### E2 Class patterns

| Jolt | Zolt |
|---|---|
| RefTarget classes without virtual functions: Skeleton, SkeletalAnimation, SkeletonMapper, RagdollSettings, Ragdoll | Plain structs with `ref_count: RefCount` that keep their allocator: `create` / `addRef` / `release`, like `PhysicsScene.zig` |
| `SkeletonPose` | Value struct owning its arrays (`init` / `deinit` with the allocator) |
| Non-virtual inheritance: `SkeletalAnimation::Keyframe : JointState`, `RagdollSettings::Part : BodyCreationSettings`, `CharacterContact : CharacterContactKey`, `VehicleEngine : VehicleEngineSettings`, `VehicleTransmission`, `VehicleTrack` | The parent is embedded as the field `base`, like `ShapeCastResult.base` |
| `CharacterBaseSettings` → `CharacterSettings`, `CharacterVirtualSettings` | Pattern A root with `ref_count` (Jolt's only virtual is the destructor) |
| `CharacterBase` → `Character`, `CharacterVirtual` | Pattern A root with `ref_count`; `saveState` / `restoreState` are virtual. `restoreState` returns `Allocator.Error!void`, because CharacterVirtual's restore resizes its active contacts |
| `CharacterContactListener` | Pattern B; every callback is optional (Jolt's defaults do nothing or return `true`) |
| `CharacterVsCharacterCollision` → `CharacterVsCharacterCollisionSimple` | Pattern B: a user implemented interface, held by CharacterVirtual as an optional fat pointer |
| `CharacterVirtual::ContactCollector` / `ContactCastCollector` | Collectors embedding `base: CollideShapeCollector` / `CastShapeCollector`, overriding `addHit` and `setUserData` (CollisionArchitecture D5) |
| `VehicleConstraint` (a `Constraint` and a `PhysicsStepListener`) | A pattern A level directly under `Constraint`, not TwoBodyConstraint. `stepListener()` returns its pattern B `PhysicsStepListener` for `PhysicsSystem.addStepListener` / `removeStepListener` |
| `VehicleControllerSettings` → Wheeled → Motorcycle; Tracked | Pattern A root with `ref_count`, abstract: `constructController`, `saveBinaryState`, `restoreBinaryState` |
| `VehicleController` → Wheeled → Motorcycle; Tracked | Pattern A root without `ref_count`. VehicleConstraint owns it (`delete mController` is `controller.destroy()`, the rule for BroadPhase) |
| `WheelSettings` → `WheelSettingsWV`, `WheelSettingsTV` | Pattern A root with `ref_count`, concrete (`saveBinaryState` / `restoreBinaryState` are virtual) |
| `Wheel` → `WheelWV`, `WheelTV` | Pattern A root without `ref_count`, owned by VehicleConstraint |
| `VehicleCollisionTester` → Ray, CastSphere, CastCylinder | Pattern A root with `ref_count`, abstract |
| `IsKindOf(&x, JPH_RTTI(T))` plus a static cast | A checked downcast (`virtual.downcast` with the rtti check, as `shape.cast(T)`) |

- Ownership follows Jolt's `Ref` / `RefConst` / owning pointers exactly, so the reference counts match. The parity
  compares them where Jolt exposes them.
- **Allocation failure.**
  - Every Zolt function that allocates returns `Allocator.Error` (or a `Result`) instead of crashing.
  - A creation that fails with an error leaves nothing behind: a ragdoll with half of its bodies, a vehicle without
    its controller or a character without its inner body is never left in the system. (Out of bodies is not an error,
    as in Jolt: `createRagdoll` returns null, a Character has no body and a CharacterVirtual no inner body, E5.)
  - Each group sweeps its creation and update functions with `FailingAllocator` (SimulationArchitecture D5, D12).

### E3 Registries

- Phase 7 used comptime lists in `RegisterTypes.zig`: `constraint_settings_types` and the new
  `vehicle_controller_settings_types`. Phase 8 replaced them with `RegisterTypes.factory`
  ([SerializationArchitecture.md](SerializationArchitecture.md) S8 and S9).
- `factory_types` has Jolt's `RegisterTypes.cpp` order: `VehicleConstraintSettings` after `PathConstraintSettings`,
  then WheeledVehicleControllerSettings, WheelSettingsWV, TrackedVehicleControllerSettings, WheelSettingsTV and
  MotorcycleControllerSettings. Skeleton, SkeletalAnimation and RagdollSettings are in it too.
- `VehicleConstraintSettings.restoreBinaryState` creates the controller settings from the RTTI hash with
  `RegisterTypes.factory.findHash` (`Factory::sInstance->Find(hash)`). A user adds classes through
  `zolt_user_types.serializable_types`.

### E4 Character base and the CharacterID counter

- `CharacterID`'s counter (`sNextCharacterID`, reset by `sSetNextCharacterID`) is a static atomic in Jolt. Zolt
  keeps it as a `pub var` atomic in `CharacterID.zig`.
  - This follows the rename table's rule for static member variables (`sFoo` becomes a `pub var`), like
    `NarrowPhaseStats`.
  - Reason: the IDs are part of the M8 hash and of the contact ordering, and PerformanceTest resets the counter
    before every run. A per-system counter would give characters in different systems other IDs than Jolt.
- `CharacterBase` keeps `system: *PhysicsSystem`, like Jolt's `mSystem`. The ground material starts as
  `PhysicsMaterial.default`, Zolt's `PhysicsMaterial::sDefault`, and the character holds a reference to it, as Jolt's
  `RefConst` does.

### E5 CharacterVirtual

- **Calls run where the user makes them.** CharacterVirtual runs outside `PhysicsSystem.update`, on the thread that
  calls it (`update`, `extendedUpdate`, `stickToFloor`, `walkStairs`, `refreshContacts`, `setShape`, `canWalkStairs`).
  Its queries go through `PhysicsSystem.getNarrowPhaseQuery` with the caller's `TempAllocator`. It must lock exactly
  what Jolt locks (`BodyLockRead` / `BodyLockWrite`), because step listeners and the user may call it while the
  system is not updating.
- **Contact order is deterministic.** The contacts and listener callbacks happen in Jolt's sorted order
  (`ContactOrderingPredicate`, sorted by `CharacterContactKey`).
  - The listener-contact bookkeeping (`ListenerContacts`, an `UnorderedMap` in Jolt) must call the callbacks in
    exactly Jolt's order. Check what Jolt iterates over before choosing a container.
  - When Jolt iterates an unordered map, the order is part of the hash. Port the container (`Core/UnorderedMap.zig`
    is ported) instead of replacing it.
- **Character vs character.** `CharacterVsCharacterCollisionSimple` keeps Jolt's `Array<CharacterVirtual *>` order.
  It passes the other character through `setUserData(@intFromPtr(character))`, as Jolt passes
  `reinterpret_cast<uint64>`.
- **Inner body.** The inner body (`mInnerBodyID`) is a kinematic body that CharacterVirtual creates, moves and
  destroys through the BodyInterface. Its shape and object layer come from the settings (`mInnerBodyShape`,
  `mInnerBodyLayer`). Its user data is the `inUserData` argument of the constructor. Its creation can fail
  ("out of bodies", or an override ID that is out of range or in use); then the character has no inner body, exactly
  like Jolt, which does not assert there.

### E6 Vehicles

- **Step listener.** `VehicleConstraint.onStep` runs inside the step listener job, in parallel with the other step
  listeners. It reads and writes only its own vehicle body and the bodies under its wheels, through the
  BodyLockInterfaceNoLock of the step context, as Jolt does.
- **Islands.** `buildIslands` / `buildIslandSplits` link the vehicle body with the bodies under the wheels, exactly
  as Jolt does. The order of the wheels decides the order of the links.
- **Jolt bug found in the design review: restoring vehicle settings with wheels.**
  - `VehicleConstraintSettings::RestoreBinaryState` resizes `mWheels` and then calls `w->RestoreBinaryState` on the new
    null `Ref`s. Every restore of settings with wheels dereferences a null pointer.
  - The stream also does not say whether a wheel is WV or TV.
  - Zolt panics there with a message, as it does for Path's null path. Document this in the file header. The parity
    does not restore vehicle settings with wheels; settings without wheels restore and compare.
- **Collision testers.** VehicleCollisionTesterRay, CastSphere and CastCylinder go through NarrowPhaseQuery `castRay`
  / `castShape`. CastSphere and CastCylinder build their query shape on the stack and embed it like Jolt
  (`setEmbedded`, `deinit` at the end of the query).

### E7 Ragdoll

- **Building a ragdoll.** `RagdollSettings.createRagdoll` creates the bodies with `BodyInterface.createBody` and the
  constraints with `TwoBodyConstraintSettings.createConstraint`.
  - `Ragdoll.addToPhysicsSystem` uses `addBodiesPrepare` / `addBodiesFinalize` and `PhysicsSystem.addConstraints`, as
    Jolt does. Zolt adds the constraints between the two body calls (section 6).
  - The `inLockBodies` flag selects the locking or the no-lock BodyLockInterface.
- **Driving to a pose.** `driveToPoseUsingMotors` sets SwingTwist / Hinge motor targets
  (`SwingTwistConstraint.setTargetOrientationBS`, `HingeConstraint.setTargetOrientationBS` /
  `setTargetAngularVelocity`). Other constraint classes are a `JPH_ASSERT` in Jolt: Zolt asserts in safe builds and
  ignores them in release builds. `driveToPoseUsingKinematics` calls `moveKinematic`.
- **Stabilization and collision filtering.** `stabilize` and `disableParentChildCollisions` are numeric and compared
  bit for bit. `disableParentChildCollisions` uses CollideShape and fills the GroupFilterTable.

### E8 Skeleton

- Pure computations with exact float order: SkeletonPose joint matrices, SkeletalAnimation sampling (including
  looping and the keyframe search), SkeletonMapper mapping, locking and unmapped joints.
- They are compared through a new pure parity harness, `ZoltParity/Physics/SkeletonParity.zig` +
  `SkeletonReference.cpp`.

## 3. Parity harness (Phase 7 foundation)

The simulation parity gets characters, ragdolls and vehicles the way Phase 6 gave it constraints. The C++ side
supports every Phase 7 class from the start. The Zolt side has one stub slot per class, which the group that ports the
class fills.

- **Descriptions.** `SimSceneDesc` gets arrays of character, character virtual, ragdoll and vehicle descriptions
  (extern structs, booleans as `u32`). They cover every member of the settings classes, including nested settings:
  engine, transmission, differentials, wheels and anti-roll bars.
  - A ragdoll description is a procedurally built skeleton plus parts.
  - The ragdoll's parts and constraints reuse the existing shape and constraint descriptions.
- **Reserved numbers.**
  - `OpKind` 400..599, one `Phase7OpRange` per group:
    - 400..429 Character;
    - 430..489 CharacterVirtual;
    - 490..519 Ragdoll;
    - 520..569 vehicle core (554..559 the Constraint interface of a vehicle, 560..562 wheel and engine torque, 563
      `wheeled_set_differentials`);
    - 570..579 TrackedVehicleController;
    - 580..589 MotorcycleController;
    - 590..599 spare.
  - `RecordKind` 7..17 for the character contact callbacks, 18..20 for the vehicle callbacks, then one block per group
    for new kinds (21..79).
  - New `Blocker`s: `characters`, `character_virtuals`, `ragdolls`, `wheeled_vehicles`, `tracked_vehicles`,
    `motorcycles`. They flip automatically when the class is ported (its settings no longer declare `is_stub`), as
    the Phase 6 constraint blockers do.
- **What is compared after every update.**
  - The physics system's `saveState` bytes, which include ragdoll bodies and constraints and the VehicleConstraint
    state.
  - Each Character's and CharacterVirtual's `saveState` bytes.
  - The character contact listener records, in call order: the character operations run on the user thread, and the
    order is part of the M8 hash. The vehicle callbacks come from the step listener jobs and are sorted with the other
    records.
  - The getters that are not in the state:
    - CharacterVirtual's active contacts (its `SaveState` writes only the contacts with a collision, without the back
      facing flag, the character B pointer, the user data and the material);
    - the ground material and the ground user data of both character classes (`CharacterBase::SaveState` does not
      write them);
    - `canWalkStairs`;
    - a ragdoll's root transform and pose;
    - the wheel values that no `SaveState` writes: the steer angle and the hard point flag; the contact's sub shape,
      point velocity and longitudinal direction; the slip, friction and brake impulse of `WheelWV` and `WheelTV`; the
      wheel transforms (VehicleSlot.writeState of SimulationParity.zig lists them all).
- **Runs.** These scenes run in the existing thread matrix and D12 failure sweeps. Section 6 lists the three scenes
  without sweeps and the broad phase restore of the Character and CharacterVirtual scenes.
- **Character updates happen between `PhysicsSystem.update` calls.** That is where PerformanceTest and the unit tests
  call them, through operations that run before an update. A character op that needs the temp allocator uses the
  world's.
- **Documentation.** The foundation writes a "Recipe for a Phase 7 worker" in the SimScenes header, the counterpart of
  the Phase 6 recipe.

### Settled in the foundation review

The foundation found these Jolt behaviors, and the review confirmed every one against the Jolt source. Porters
follow the decision column.

| Jolt behavior | Decision |
|---|---|
| `Wheel` leaves `mContactPosition`, `mContactNormal`, `mContactLateral` (and the neighbouring contact members) uninitialized until the first contact; `VehicleConstraint::SaveState` writes them | Keep the byte layout and zero-initialize these members in Zolt (documented deviation). The reference library zero-fills its allocations, so a vehicle that starts in the air compares too |
| `MotorcycleController::SaveState` omits `mLeanSpringIntegratedDeltaAngle` | Reproduce exactly: the state bytes are Jolt's |
| `VehicleTrackSettings::mDrivenWheel` is uninitialized | Default it to 0 (undefined in Jolt, nothing to reproduce) and document it |
| `VehicleEngineSettings` Save/RestoreBinaryState omit `mInertia` and `mAngularDamping` | Reproduce exactly |
| `VehicleConstraintSettings::RestoreBinaryState` dereferences null wheel settings (E6) and a null RTTI for an unknown controller hash | Panic with a message in both cases |
| `CharacterVirtualSettings`' default initializer and `CharacterVirtual::GetCharacterVirtualSettings` each consume a CharacterID; the copy constructor does not | Reproduce exactly (`init` / `create` and `getCharacterVirtualSettings` consume one, a copy does not). M8 depends on it; the parity has a counter test |
| `WheeledVehicleController::GetWheelSpeedAtClutch` computes 0/0 without driven wheels | Reproduce: no guard |

## 4. Groups and order

1. **Foundation:** CharacterID and CharacterBase (complete), stub files for Character, CharacterVirtual, Ragdoll and
   VehicleConstraint (with `is_stub`), and the harness of section 3.
   - **In parallel: Skeleton group,** the four Skeleton files with SkeletonParity. It does not touch the foundation's
     files.
2. After the foundation merge, at most two groups at a time (the machine has 4 cores; a parity run takes 20 to 40
   minutes):
   - **CharacterVirtual group:** CharacterVirtual, CharacterVsCharacterCollisionSimple, the port of
     LoggingCharacterContactListener, CharacterVirtualTests and scenes. It also does M8: ZoltPerformanceTest's
     CharacterVirtual scene, a `perf_character_virtual` simulation scene, and the CI steps.
   - **Vehicle core group:** the ten core files, WheeledVehicleTests, the vehicle registry and scenes.
   - **Ragdoll group:** Ragdoll and scenes; needs the Skeleton group.
   - **Character group:** Character and scenes.
   - **Vehicle variants group:** TrackedVehicleController, MotorcycleController and scenes; needs Vehicle core.
3. **Workflow per group:** port, then review, then fix, then merge and run the full verification:
   - tests in 4 configs;
   - parity in Debug, double precision and ReleaseFast;
   - the published hashes.

**Merging.** Shared files are the registries:
- `zolt.zig`, `unit_tests.zig`, `parity.zig`, `reference_sources.zig`, merged with the union driver plus
  `tools/tidy_registry.py`;
- `RegisterTypes.zig`;
- the reserved regions of SimScenes, `SimulationReference.cpp` and `SimulationParity.zig`.

A group edits only its own region, slot, files and registry lines.

## 5. Checklist for porters

- File headers:
  - `//! Port of: <C++ files>` and `//! Status: complete|partial|stub`.
  - Keep Jolt's doc and implementation comments.
  - Deviations from Jolt go into the file header.
  - Port `Draw*` under the gates of [RemainingFeaturesArchitecture.md](RemainingFeaturesArchitecture.md) R2. Never leave
    a `TODO(debug_renderer)`: `port_status.py --check` fails on one.
  - A serializable class declares `pub const rtti` ([SerializationArchitecture.md](SerializationArchitecture.md)
    section 3).
- Unit tests: port them with the same names and values. Add inline tests for allocation failure and for the asserts
  Zolt keeps.
- Parity:
  - Parity for every function that computes numbers, plus simulation scenes that reach the class's branches (state the
    branches in `SceneEntry.expect`).
  - Prove that the tests bite: break one operation temporarily and see a test fail.
- Jolt asserts on user input: Zolt asserts the same in safe builds. Scenes keep away from them, or are
  `release_only` to compare Jolt's release behavior.
- Done criteria (CLAUDE.md's list, with more build modes):
  - `zig build test` in Debug, double precision, ReleaseFast and `-Duse_llvm=false`;
  - since Phase 9: `zig build check -Ddebug_renderer=true`, and `zig build parity-check` in both renderer
    configurations;
  - `zig build parity` in Debug, double precision and ReleaseFast. For a file with a draw gate (since Phase 9), also
    the filtered parity with `-Ddebug_renderer=true`;
  - `zig fmt --check`;
  - `python3 tools/port_status.py --write`, then `--check`.

## 6. Settled in the Phase 7 groups

Each group was reviewed after its port. A second reviewer checked every finding against the Jolt source, and the group
fixed the real ones before its merge. The reviews found no result difference from Jolt in library code. They found
crashes on input that Jolt accepts, one missing API, coverage gaps and wrong docs. These points were settled on the
way. They add to "Settled in the foundation review" and bind later phases that change these classes. The file headers
have the details.

| Jolt behavior | Decision |
|---|---|
| `CharacterVirtual::RestoreState` resizes `mActiveContacts`, the only allocation of a restore | `CharacterBase.restoreState` returns `Allocator.Error!void` (E2), also for Character. The Character tests and the harness slots call it with `try` |
| CharacterVirtual gets no inner body when `CreateBody` / `CreateBodyWithID` returns null (out of bodies, an override ID that is out of range or taken); nothing asserts | The same in Zolt (E5). The scene `character_virtual_inner_body` has a character whose override ID is taken |
| CharacterVirtual: `RestoreState` does not restore `mIsBackFacingContact`; a listener contact that is added twice in one tracking session gets `OnContactPersisted`; the `SolveConstraints` tie breaker orders by descending motion type (its comment says static first); `MoveToContact` matches contacts by body and sub shape only; `UpdateSupportingContact` solves its support test with a delta time of 1 | Reproduce exactly. The object state compares the stale back facing flag |
| A Character without a body (out of bodies): `~Character` calls `DestroyBody` with the invalid ID, `GetCharacterSettings` dereferences the missing body | Undefined in Jolt. Zolt's destructor skips the invalid ID, `getCharacterSettings` panics with a message. The functions that go through the BodyInterface do nothing, as in Jolt |
| `SkeletonMapper::Map` loops with a signed `j < size - 1`, so chains of 1 or 2 joints (added through the writable `GetChains`) work | The same in Zolt (`while (j + 1 < len)`; the first port overflowed). An empty chain panics: Jolt reads `front()` / `back()` of an empty array |
| `SkeletalAnimation::Sample` asserts that the keyframe fraction is in [0, 1] | Unsorted keyframes are valid input: the binary search always ends on two keyframes that bracket the time. Only a NaN fraction (times near the end of the float range) breaks the assert, which Zolt checks in safe builds only |
| `CreateRagdoll` stores the body of a part before it reads the parent's slot, so a joint that is its own parent connects its body with itself | The same in Zolt (the first port panicked). Zolt panics only for a parent index that is negative or greater than the joint's own: there Jolt reads outside its array or an uninitialized slot. Scene `ragdoll_self_parented_joint` |
| `Ragdoll::DriveToPoseUsingMotors` sets the motor states and targets but does not activate the bodies | Reproduce: the motors do not wake a sleeping ragdoll. `ragdoll_motors_animation` activates its hanging ragdoll every tenth update |
| `WheeledVehicleController::PostCollide` allocates its arrays and matrices on every step, and the pivots of GaussianElimination are a `JPH_STACK_ALLOC(n)`: there is no limit on the number of differentials | No limit in Zolt either. The scratch for D differentials (at most 2 * D driven wheels, a matrix of 2 * D + 1 rows and columns, 2 * D + 1 pivots) is allocated with the controller, and `gaussianElimination` takes the caller's pivot buffer (`opts.ipiv`). The first port had a fixed buffer of 256 pivots, which 128 differentials with two wheels each overflowed |
| The writable `GetDifferentials()` returns the array, so the number of differentials can change between steps | `getDifferentialsMut()` returns a slice: values only. The Zolt function `setDifferentials` replaces the differentials on the user thread. It allocates the new array and scratch first, so a failure keeps the old ones. Harness operation 563 `wheeled_set_differentials` |
| The writable `VehicleConstraint::GetWheels()` returns the array | `getWheelsMut()` returns a slice: the number of wheels is fixed at creation, because the body ID buffer of BuildIslands (`body_ids_scratch`, wheels + 1) is allocated with the constraint |
| `VehicleConstraint::IsActive` reads `mIsActive`, which OnStep computes and SaveState does not save; OnStep sets the gravity factor of the vehicle body to 0 while the gravity override is on | Reproduce: a restore keeps the `is_active` of the restoring constraint |
| `MotorcycleController::GetSettings` assigns `mLeanSpringDamping` to itself | Reproduce: the returned settings have the default damping of 1000. VehiclePartsParity compares the GetSettings bytes |
| `MotorcycleController::SaveState` / `RestoreState` cover only `mTargetLean`, not the integrated lean angle or `mAppliedImpulse` | Reproduce. A restore keeps the integrated angle of the restoring controller, so a restored run with an integration coefficient other than 0 diverges, in Jolt too. Scenes with restores or sweeps keep the coefficient at 0. The two integration scenes run without sweeps (`sweep_stride = 0`) and without restores |
| A tracked vehicle wheel in no track keeps `mTrackIndex = -1`, and Jolt indexes `mTracks[-1]` | Undefined in Jolt. Zolt's safe builds panic in the index cast; ReleaseFast stays undefined, as in C++. The same for track wheel indices and driven wheels outside the wheels. The scenes put every wheel in a track |
| `VehicleTrackSettings::mDrivenWheel` is uninitialized (foundation review: Zolt defaults it to 0) | The C++ reference helpers set it to 0, so both sides start equal |
| The determinism log of `MotorcycleController::PreCollide` uses the label `WheeledVehicleController::PreCollide: mTargetLean: ` | Zolt keeps Jolt's string |

**Zolt error paths that change the order of Jolt's steps.** A failed call changes nothing. A successful call gives
Jolt's result:
- `Ragdoll.addToPhysicsSystem` adds the constraints between `addBodiesPrepare` and `addBodiesFinalize` (Jolt:
  AddBodiesFinalize, then AddConstraints), so that a failure can abort the prepared bodies. The constraint manager does
  not read the bodies, and the constraints get Jolt's indices. During the call another thread may see the constraints
  before the bodies are in the broad phase. Jolt's call has an in-between state too, so the review accepted the order.
- `RagdollSettings.createRagdoll` stores the ID of a new body right after its creation (Jolt: after its constraint),
  so that a failing constraint creation destroys that body too.
- `RagdollSettings.stabilize` calculates the mass properties before it changes the first part.
  `disableParentChildCollisions` creates the shapes of the parts before it assigns the group filter.
- `Character.setShape` sets the shape of the body before the character's (Jolt: the character's first; the reference
  counts end the same). `Character.setLayer` restores the character's layer when the broad phase fails.
- CharacterVirtual enters a contact in the listener bookkeeping before its `OnContactAdded` callback. The listener
  cannot see the order, and the bookkeeping keeps Jolt's iteration order.

**Undefined behavior in Jolt that Zolt does not reproduce.** Zolt panics with a message, also in optimized builds,
unless noted:
- Skeleton: an animated joint that is not in the pose's skeleton or has no keyframes, a pose without a skeleton,
  LockAllTranslations before Initialize, Map with a locked joint that has no parent (Jolt reads index -1).
- Ragdoll: a null skeleton, null ragdoll settings or a null additional constraint, a body that cannot be locked. A
  pose of a ragdoll without bodies gets a zero root offset (Jolt: uninitialized).
- CharacterVirtual:
  - FinishTrackingContactChanges with an active contact that is not tracked: Jolt asserts there. Without asserts it
    writes through an `end()` iterator. Zolt panics in safe builds (Debug, ReleaseSafe), like Jolt's assert.
    ReleaseFast and ReleaseSmall skip the contact.
  - A restored motion type byte that is not valid becomes Static in every build, without a panic. Jolt keeps the
    invalid value.
- Vehicles: no controller settings, a null wheel settings `Ref`, OnStep without a collision tester.

**Harness rules.**
- **Broad phase layout.** A CharacterVirtual query gets its hits in the order in which the quad tree returns bodies.
  Jolt keeps the first of equal hits (ContactCastCollector), calls OnContactValidate / OnAdjustBodyVelocity in hit
  order and reduces the hits at `mMaxNumHits` in hit order. The Character queries do the same:
  `Character::PostSimulation` keeps the first of equal hits (`dot > mBestDot`), and the harness operation
  `character_check_collision` hashes the hits in call order. So the results depend on the layout of the broad phase
  trees, in Jolt too, and the layout is not part of the saved state. The D12 sweeps fail Zolt updates and restore the
  saved state, while the C++ side runs each update once. The failed updates change the Zolt trees: without a fix,
  `character_virtual_characters` and `character_virtual_crowd` differ after a sweep.
  - Scenes with `SceneEntry.restore_broad_phase` capture the Zolt broad phase with the snapshot of a swept update
    (`BroadPhaseSnapshot` in SimulationParity.zig: the node pages, the free list, the tracking, the trees and the next
    tree to update). After the sweeps the driver restores both and runs the update once more. This leaves the trees of
    the C++ update.
  - This is a harness rule, not a library deviation. Zolt builds its trees as Jolt does, so the same updates give the
    same trees. Only the failure injection gives Zolt updates that the C++ side never ran. The C++ side is unchanged.
  - Every Character and CharacterVirtual scene with sweeps sets it. The physics scenes keep the free running layout:
    they show that the physics pipeline does not depend on it.
  - The vehicle testers are the exception in the pipeline. They query the broad phase in `PhysicsSystem::Update`,
    through the VehicleConstraint step listener. They keep the first of equal hits (strict `<` in
    VehicleCollisionTester.cpp). So a vehicle can depend on the layout when two bodies give the same hit. The vehicle
    scenes do not set `restore_broad_phase`: they are bit exact after the sweeps without it, so no tie in them changes a
    result. A vehicle scene that differs only after a sweep sets it.
- **Character records.** The CharacterContactListener records are compared one by one in call order, not sorted
  (section 3), after every update and after the sweeps.
- **An active body for the sweeps.** PhysicsSystem.update allocates nothing without an active body. Scenes without one
  get a kinematic bystander that never sleeps (`addBystander`), so that the sweeps and the broad phase restore have
  work to do.
- **What the driver does not sweep.** The D12 sweeps cover the scene creation and `PhysicsSystem.update`. The object
  operations between the updates (the CharacterVirtual queries, the ragdoll and vehicle calls) are swept by the inline
  tests of the classes. Three scenes run without sweeps: `perf_character_virtual` (like the other `perf_*` scenes) and
  the two motorcycle lean integration scenes.
- **Teardown.** The `destroyAll` of a slot must not allocate. `ZoltWorld.destroy` removes the body activation listener
  before it removes the Phase 7 objects: a teardown record whose allocation fails would show up in the creation sweep
  as a swallowed `error.OutOfMemory`.
- **Spawned jobs.** Whether a FindCollisions job is spawned from inside a job depends on the scheduling
  (`contacts_many_bodies` in ReleaseFast on a loaded machine). When no run of the thread matrix spawned one,
  `observeSpawnedJobs` runs the scene again on the Zolt side alone, with 3 and 7 workers in turn, up to 10 times, until
  one does.
- **Asserting reference.** New scenes are checked against an asserting reference (`zig build parity
  -Dreference_asserts=true`, see the rules in the SimScenes header). At the end of Phase 7 the full asserting parity run
  passes: no scene hits a Jolt assert.
- **Known gap.** `SimShape` has no materials, so the ground material of a character is only compared as the default
  material.
