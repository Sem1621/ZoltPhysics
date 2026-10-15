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

| Group | Jolt files | Tests | Lines (C++) |
|---|---|---|---|
| Skeleton | `Skeleton/Skeleton`, `SkeletalAnimation`, `SkeletonPose`, `SkeletonMapper` | (none in UnitTests) | 1100 |
| Ragdoll | `Physics/Ragdoll/Ragdoll` | (none in UnitTests) | 1040 |
| Character base | `Physics/Character/CharacterID`, `CharacterBase` | (none) | 320 |
| Character | `Physics/Character/Character` | (none) | 510 |
| CharacterVirtual | `Physics/Character/CharacterVirtual` | `UnitTests/Physics/CharacterVirtualTests.cpp`, `UnitTests/LoggingCharacterContactListener.h` | 2780 |
| Vehicle core | `Physics/Vehicle/VehicleEngine`, `VehicleTransmission`, `VehicleDifferential`, `VehicleTrack`, `VehicleAntiRollBar`, `Wheel`, `VehicleController`, `VehicleConstraint`, `VehicleCollisionTester`, `WheeledVehicleController` | `UnitTests/Physics/WheeledVehicleTests.cpp` | 3620 |
| Vehicle variants | `Physics/Vehicle/TrackedVehicleController`, `MotorcycleController` | (none) | 1140 |

**Milestone M8.** ZoltPerformanceTest runs `PerformanceTest/CharacterVirtualScene.h`. It reproduces the hashes of
`determinism_check.yml` with `-q=Discrete -t=max -s=CharacterVirtual`:
- `0xc1d4e05b164b1221` in single precision;
- `0xa0b16727aa16fdb0` in double precision.

The hashes must also hold with `-t=1` and `-t=2`, and Zolt's CI checks them. The scene needs no assets.

`PerformanceTest/RagdollScene.h` loads `Assets/Human.tof` through ObjectStreamIn. The Ragdoll hash is therefore M9
(Phase 8); Phase 7 ports and checks the Ragdoll class without it.

## 2. Decisions

### E1 Serialization

- ObjectStream is Phase 8. `JPH_DECLARE_SERIALIZABLE_*` / `JPH_IMPLEMENT_SERIALIZABLE_*` become a
  `// TODO(serialization)` comment at the same place, as in the earlier phases.
- `port_status.py --next` lists the ObjectStream includes as blockers. Ignore that for these groups.
- Everything Jolt streams through `StreamOut` / `StreamIn` is ported now, byte compatible:
  - `SaveBinaryState`, `RestoreBinaryState`, `sRestoreFromBinaryState` of Skeleton, SkeletalAnimation,
    RagdollSettings, VehicleConstraintSettings, the controller and wheel settings, engine, transmission,
    differential, track and anti-roll bar.
  - Everything Jolt records through `StateRecorder` (`SaveState` / `RestoreState`).
- Restoring a class by its RTTI hash goes through comptime registries in `RegisterTypes.zig`, in the order of Jolt's
  `RegisterTypes.cpp` (E3).

### E2 Class patterns

| Jolt | Zolt |
|---|---|
| RefTarget classes without virtual functions: Skeleton, SkeletalAnimation, SkeletonMapper, RagdollSettings, Ragdoll | Plain structs with `ref_count: RefCount` that keep their allocator: `create` / `addRef` / `release`, like `PhysicsScene.zig` |
| `SkeletonPose` | Value struct owning its arrays (`init` / `deinit` with the allocator) |
| Non-virtual inheritance: `SkeletalAnimation::Keyframe : JointState`, `RagdollSettings::Part : BodyCreationSettings`, `CharacterContact : CharacterContactKey`, `VehicleEngine : VehicleEngineSettings`, `VehicleTransmission`, `VehicleTrack` | The parent is embedded as the field `base`, like `ShapeCastResult.base` |
| `CharacterBaseSettings` → `CharacterSettings`, `CharacterVirtualSettings` | Pattern A root with `ref_count` (Jolt's only virtual is the destructor) |
| `CharacterBase` → `Character`, `CharacterVirtual` | Pattern A root with `ref_count`; `saveState` / `restoreState` are virtual |
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
  - A failed creation leaves nothing behind: a ragdoll with half of its bodies, a vehicle without its controller or a
    character without its inner body is never left in the system.
  - Each group sweeps its creation and update functions with `FailingAllocator` (SimulationArchitecture D5, D12).

### E3 Registries

- **`constraint_settings_types`:** `VehicleConstraintSettings` takes its marked place after Path ("VehicleConstraintSettings
  follows here").
- **`vehicle_controller_settings_types` (new):** WheeledVehicleControllerSettings, TrackedVehicleControllerSettings,
  MotorcycleControllerSettings (Jolt's order), plus the user's (`zolt_user_types`). They are the classes that
  `VehicleConstraintSettings.restoreBinaryState` creates from the RTTI hash (`Factory::sInstance->Find(hash)`).
- **Skeleton, SkeletalAnimation, RagdollSettings, WheelSettingsWV and WheelSettingsTV:** Jolt registers them only for
  ObjectStream. Their registration is Phase 8.

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
  what Jolt locks (`BodyLockRead` / `BodyLockMultiWrite`), because step listeners and the user may call it while the
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
  destroys through the BodyInterface. Its user data and object layer come from the settings. Its creation can fail
  ("out of bodies") and is reported like Jolt does: no inner body, and an assert in debug builds.

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
  / `castShape`. CastCylinder builds its cylinder shape on the stack, which Zolt embeds (`is_static`, as for the Phase
  5 fixed-to-world body).

### E7 Ragdoll

- **Building a ragdoll.** `RagdollSettings.createRagdoll` creates the bodies with `BodyInterface.createBody` and the
  constraints with `TwoBodyConstraintSettings.createConstraint`.
  - `Ragdoll.addToPhysicsSystem` uses `addBodiesPrepare` / `addBodiesFinalize` and `PhysicsSystem.addConstraints`, as
    Jolt does.
  - The `inLockBodies` flag selects the locking or the no-lock BodyLockInterface.
- **Driving to a pose.** `driveToPoseUsingMotors` sets SwingTwist / SixDOF motor targets
  (`SwingTwistConstraint.setTargetOrientationBS`, ...). `driveToPoseUsingKinematics` calls `moveKinematic`.
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
  - `OpKind` 400..599:
    - 400..429 Character;
    - 430..489 CharacterVirtual;
    - 490..519 Ragdoll;
    - 520..579 Vehicle;
    - 580..599 spare.
  - New `RecordKind`s for the character contact callbacks.
  - New `Blocker`s: `characters`, `character_virtuals`, `ragdolls`, `wheeled_vehicles`, `tracked_vehicles`,
    `motorcycles`. They flip automatically when the class is ported (its settings no longer declare `is_stub`), as
    the Phase 6 constraint blockers do.
- **What is compared after every update.**
  - The physics system's `saveState` bytes, which include ragdoll bodies and constraints and the VehicleConstraint
    state.
  - Each Character's and CharacterVirtual's `saveState` bytes.
  - The character contact listener records: sorted when they come from parallel jobs; in call order when the user
    thread makes the calls, because the order is part of the M8 hash.
  - The getters that are not in the state: CharacterVirtual's active contacts, ground state, ground body and sub
    shape, and `canWalkStairs`; a ragdoll's root transform and pose; per-wheel contact information.
- **Runs.** These scenes run in the existing thread matrix and D12 failure sweeps.
- **Character updates happen between `PhysicsSystem.update` calls.** That is where PerformanceTest and the unit tests
  call them, through operations that run before an update. A character op that needs the temp allocator uses the
  world's.
- **Documentation.** The foundation writes a "Recipe for a Phase 7 worker" in the SimScenes header, the counterpart of
  the Phase 6 recipe.

## 4. Groups and order

1. **Foundation:** CharacterID and CharacterBase (complete), stub files for Character, CharacterVirtual, Ragdoll and
   VehicleConstraint (with `is_stub`), and the harness of section 3.
   - **In parallel: Skeleton group,** the four Skeleton files with SkeletonParity. It does not touch the foundation's
     files.
2. After the foundation merge, at most two groups at a time (the machine has 4 cores; a parity run takes 20 to 40
   minutes):
   - **CharacterVirtual group:** CharacterVirtual, CharacterVsCharacterCollisionSimple, the port of
     LoggingCharacterContactListener, CharacterVirtualTests and scenes.
   - **Vehicle core group:** the ten core files, WheeledVehicleTests, the vehicle registry and scenes.
   - **Ragdoll group:** Ragdoll and scenes; needs the Skeleton group.
   - **Character group:** Character and scenes.
   - **Vehicle variants group:** TrackedVehicleController, MotorcycleController and scenes; needs Vehicle core.
   - **M8:** ZoltPerformanceTest's CharacterVirtual scene, a `perf_character_virtual` simulation scene, and the CI
     steps; needs CharacterVirtual.
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
  - Deviations from Jolt go into the file header (`TODO(serialization)`, `TODO(debug renderer)` for `Draw*`).
- Unit tests: port them with the same names and values. Add inline tests for allocation failure and for the asserts
  Zolt keeps.
- Parity:
  - Parity for every function that computes numbers, plus simulation scenes that reach the class's branches (state the
    branches in `SceneEntry.expect`).
  - Prove that the tests bite: break one operation temporarily and see a test fail.
- Jolt asserts on user input: Zolt asserts the same in safe builds. Scenes keep away from them, or are
  `release_only` to compare Jolt's release behavior.
- Done criteria (CLAUDE.md):
  - `zig build test` in Debug, double precision, ReleaseFast and `-Duse_llvm=false`;
  - `zig build parity` in Debug, double precision and ReleaseFast;
  - `zig fmt --check`;
  - `python3 tools/port_status.py --write`.
