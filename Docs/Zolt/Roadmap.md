# Zolt Roadmap

The plan for porting Jolt Physics (~87k lines of C++ in scope) to Zig 0.16. Phases follow the
`#include` dependency graph, so every phase can be compiled and tested on its own. Per-file status
lives in [Progress.md](Progress.md) (generated); `python3 tools/port_status.py --next` lists the
files whose dependencies are already ported.

## Strategy

- **Bottom-up, file by file.** Port a header together with its `.inl`/`.cpp` and its unit tests,
  then move up the dependency graph. Never port a file before its dependencies (stub them if a
  cycle forces it, and mark the stub `//! Status: stub`).
- **Faithful first, idiomatic second.** Same algorithms and FP operation order as Jolt
  (see [PortingGuide.md](PortingGuide.md)). Optimizations and API redesigns come after the
  port reproduces Jolt bit for bit, never during.
- **Tests are the spec.** Each phase ports the matching `UnitTests/` files. A file is
  `complete` only when its tests pass in single and double precision.
- **Parity with the C++ library at every step.** `zig build parity` compiles `Jolt/` with Zig's
  C++ compiler and compares Zolt with it bit for bit (`ZoltParity/`). Each phase extends it: math
  functions, then geometry (GJK/EPA results), collision queries, and finally full simulation steps
  (state hash per step), so a divergence is caught where it is introduced instead of in the final
  determinism hashes.
- **Determinism is the acceptance test.** Jolt's CI publishes the hashes that a
  `CROSS_PLATFORM_DETERMINISTIC` build produces for 5 PerformanceTest scenes
  (`.github/workflows/determinism_check.yml`). Reproducing them proves the port is exact:

  | Scene            | Single precision       | Double precision       | Needs                                      |
  |------------------|------------------------|------------------------|--------------------------------------------|
  | Pyramid          | `0x74d0118836ac0892`   | `0xf07f04597a102a82`   | BoxShape, core simulation                  |
  | HighSpeed        | `0x58d8d892536f4021`   | `0x183f48cb94739ce`    | + Sphere/Capsule, StaticCompound, LinearCast, Mt19937 |
  | ConvexVsMesh     | `0x918f27cacae9752b`   | `0x3729e7890b93a289`   | + MeshShape, convex shapes                 |
  | CharacterVirtual | `0xc1d4e05b164b1221`   | `0xa0b16727aa16fdb0`   | + CharacterVirtual                         |
  | Ragdoll          | `0x3be6ea76c07a2ac8`   | `0x8dd6bc501859a228`   | + Skeleton, Ragdoll, constraints, ObjectStream loading of `Assets/` |

  The hash is FNV-1a (`HashBytes`) over each body's position (3 `Real`s) and rotation (`Quat`)
  in body ID order, plus the scene's own `UpdateHash`, see `PerformanceTest/PerformanceTest.cpp`.
  The scenes run with the `-q=LinearCast` (or `-q=Discrete` for CharacterVirtual) motion quality.
- **Parallel work.** Independent subtrees (e.g. shapes vs. constraints) can be ported in parallel
  in separate git worktrees (one branch per worker, based on the current port branch). The
  registry files (`Zolt/zolt.zig`, `ZoltTests/unit_tests.zig`, `ZoltParity/parity.zig`,
  `ZoltParity/reference_sources.zig`) are the only shared files; git
  merges them with the `union` driver (`.gitattributes`), after which `python3 tools/tidy_registry.py`
  removes duplicates and restores the sort order. Workers must never "fix" shared code they don't
  own; they report the problem instead.

## Phases

### Phase 0: Infrastructure ✅
`build.zig` (options `double_precision`, `object_layer_bits`), module layout, porting guide,
`tools/strip_isa.py`, `tools/port_status.py`, `tools/tidy_registry.py`, `tools/install_zig.sh`,
CI, SessionStart hook, and the parity harness (`zig build parity`, `ZoltParity/`).

### Phase 1: Math (`Jolt/Math`, `UnitTests/Math`) ✅
Vec3, Vec4, UVec4, Float2/3/4, Swizzle, Math, Trigonometry, Quat, Mat44, DVec3, Double3, DMat44,
Real, BVec16, HalfFloat, Vector, Matrix, DynMatrix, GaussianElimination, EigenValueSymmetric,
FindRoot. Plus `Core/HashCombine` and `Core/Mt19937` (std::mt19937 replacement).
**Milestone M1:** all of `UnitTests/Math` passes in both precisions. ✅ Reached: every Math
function also has parity tests against the C++ library (bit exact, both precisions).

### Phase 2: Core (`Jolt/Core`, `UnitTests/Core`) ✅
HashCombine, Mt19937, QuickSort, InsertionSort, BinaryHeap, StaticArray, Reference, Color, Atomics,
Prefetch, plus five groups ported in parallel:
- Containers: HashTable, UnorderedMap, UnorderedSet, ObjectToIDMap (Jolt's own, for deterministic
  iteration order; parity tests compare bucket indices and iteration order with the C++).
- Concurrency: Mutex/SharedMutex, MutexArray, Semaphore, FixedSizeFreeList, LockFreeHashMap
  (parity tests compare free list indices, hash map handles and iteration order).
- Job system: JobSystem, JobSystemWithBarrier, JobSystemThreadPool, JobSystemSingleThreaded
  (std.Thread + std.Io primitives, see the guide's Threading section).
- Memory: TempAllocator (TempAllocatorImpl, TempAllocatorMalloc, TempAllocatorImplWithMallocFallback),
  STLTempAllocator, STLLocalAllocator (as `std.mem.Allocator` adapters), ByteBuffer, StridedPtr.
- Misc: StringTools, TickCounter, FPControlWord, FPFlushDenormals, StreamIn/StreamOut/StreamWrapper
  (over std.Io.Reader/Writer), LinearCurve, ConfigurationString.
Partial: `Core.zig` (only what ported code needs). Deferred: Profiler (developer tooling, compiled out in
the Distribution configuration). RTTI, Factory, StreamUtils and the ObjectStream declarations of LinearCurve
came with Phase 8.
**Milestone M2:** `UnitTests/Core` passes, including JobSystemTest. ✅ Reached: all of `UnitTests/Core`
passes in both precisions. Parity tests against the C++ library cover the sorts, BinaryHeap, the hashes,
Mt19937, the hash containers, FixedSizeFreeList, LockFreeHashMap, MutexArray, Semaphore, the streams,
LinearCurve and StringTools (TempAllocator offsets are tested against values derived from the C++).

Notes for later phases:
- `LockFreeHashMap` stores `KeyValue` as an extern struct (the value must be the last field, followed by
  the extra bytes, and handles are byte offsets as in Jolt), so its key and value types must be extern
  compatible: `BodyPair`, `SubShapeIDPair` and the cached manifold types of the ContactConstraintManager
  have to be `extern struct`s.
- Out of memory is `error.OutOfMemory` where Jolt aborts or does not check (`TempAllocator.allocate`,
  `FixedSizeFreeList.constructObject`, `JobSystem.createJob`), so physics code that uses them returns an
  error union.

### Phase 3: Geometry (`Jolt/Geometry`, `Jolt/AABBTree`, `Jolt/TriangleSplitter`) ✅
Five groups, in dependency order:
- Primitives: Triangle, IndexedTriangle, Plane, AABox, Sphere, OrientedBox, AABox4, ClipPoly,
  MortonCode, Indexify, Ellipse.
- Queries: ClosestPoint, RayAABox, RayTriangle, RaySphere, RayCylinder, RayCapsule, ConvexSupport,
  ConvexHullBuilder2D.
- GJK / EPA: GJKClosestPoint, EPAConvexHullBuilder, EPAPenetrationDepth.
- ConvexHullBuilder.
- AABB tree: TriangleSplitter (+ Binning, Mean), AABBTreeBuilder, AABBTreeToBuffer,
  NodeCodecQuadTreeHalfFloat, TriangleCodecIndexed8BitPackSOA4Flags (used by MeshShape).
**Milestone M3:** `UnitTests/Geometry` passes. ✅ Reached, except the four `TestGJKRay*Shape` cases of
`GJKTests`, which need the convex shapes and are ported with Phase 4. Every Geometry function has parity
tests against the C++ library (bit exact, both precisions), including GJK/EPA results on separated,
touching and penetrating pairs, the complete face lists of the 3D hull builder and the AABB tree buffers
byte for byte.

Notes for later phases:
- `SortReverseAndStore` / `CountAndSortTrues` (`Physics/Collision/SortReverseAndStore.h`) have private
  copies in the AABB tree tests; replace them with the real port in Phase 4.
- Jolt asserts on some degenerate inputs that its release build tolerates (EPA near-zero vectors,
  near-degenerate hulls, empty meshes). Zolt panics on them with asserts enabled, like an assert-enabled
  Jolt build, and matches Jolt's release behavior in ReleaseFast (guide section 7).

### Phase 4: Collision (`Jolt/Physics/Collision`) ✅
Architecture: [CollisionArchitecture.md](CollisionArchitecture.md) (decided by three competing compiled
prototypes, two judges and a synthesis). Port order:
1. Foundation F1: `Core/Virtual.zig`, `Core/PlacementBuffer.zig`, `Core/Result.zig`, BodyID,
   MassProperties, a `Body` stub, PhysicsSettings, ObjectLayer, BroadPhaseLayer, the mode enums,
   SubShapeID(Pair), RayCast, AABoxCast, CastResult, CollidePointResult, CollideShape, ShapeCast,
   SortReverseAndStore, CollisionCollector(Impl), ShapeFilter, SimShapeFilter, PhysicsMaterial(Simple).
2. Foundation F2: Shape (+ ShapeSettings, ShapeFunctions), ConvexShape, DecoratedShape, CompoundShape,
   CompoundShapeVisitors, ScaleHelpers, GetTrianglesContext, CollisionDispatch, TransformedShape,
   RegisterTypes with a stub for every shape.
3. Wave A (parallel): the convex shapes (Sphere, Box, Capsule, TaperedCapsule, Cylinder,
   TaperedCylinder, ConvexHull + PolyhedronSubmergedVolumeCalculator, Plane, Empty), the decorated
   shapes (Scaled, RotatedTranslated, OffsetCenterOfMass), the triangle algorithms (ActiveEdges,
   ManifoldBetweenTwoFaces, Collide/Cast Convex/Sphere VsTriangles, CollideShapeVsShapePerLeaf,
   InternalEdgeRemovingCollector), filters and groups (GroupFilter(Table), CollisionGroup,
   ObjectLayerPairFilter*, BroadPhaseLayerInterface*, ObjectVsBroadPhaseLayerFilter*) and the
   ContactListener types.
4. Wave B: TriangleShape, StaticCompoundShape, MutableCompoundShape, MeshShape, HeightFieldShape.
The broad phase, NarrowPhaseQuery and EstimateCollisionResponse need `BodyManager` and move to Phase 5.
Most `UnitTests/Physics` files mix pure shape cases with cases that need `PhysicsTestContext`
(a `PhysicsSystem`): Phase 4 ports the pure cases and marks the files partial; parity tests against the
C++ library carry most of the verification.
**Milestone M4:** ✅ reached. Every shape and collision algorithm is ported and reviewed; besides the
per-shape parity tests, `ZoltParity/Physics/PairwiseParity.zig` sweeps every ordered pair of 61 catalogue
shapes (all shape types, decorators, nested compounds, meshes, height fields) through collide, cast,
ray and point queries bit exact in both precisions and in Debug / ReleaseFast. The PhysicsTestContext
parts of the shared test files are listed in their `//! Missing:` headers for Phase 5. Definition:
every collision function has bit exact parity tests, and the shape-only cases of
`ShapeTests`, `RayShapeTests`, `CollideShapeTests`, `CastShapeTests`, `ConvexVsTrianglesTest`,
`CollidePointTests`, `SubShapeIDTest`, `CollisionGroupTests`, `ObjectLayerPairFilter*Tests`,
`HeightFieldShapeTests`, `MutableCompoundShapeTests`, `OffsetCenterOfMassShapeTests`,
`TaperedCylinderShapeTests`, `TransformedShapeTests`, `ShapeFilterTests` and `ActiveEdgesTests` pass.

### Phase 5: Bodies and simulation (`Jolt/Physics`, `Jolt/Physics/Body`, `Jolt/Physics/Constraints` core) ✅
Architecture and port order: [SimulationArchitecture.md](SimulationArchitecture.md) (decided by three
competing compiled prototypes, a judge and a synthesis).
BroadPhase, BroadPhaseQuery, QuadTree, BroadPhaseQuadTree, BroadPhaseBruteForce, NarrowPhaseQuery,
EstimateCollisionResponse, SimShapeFilterWrapper (moved from Phase 4), then the remaining
`PhysicsTestContext` cases of the Phase 4 test files.
MotionType, MotionQuality, AllowedDOFs, BodyID, MassProperties, MotionProperties, Body,
BodyCreationSettings, BodyManager, BodyLock*, BodyInterface, BodyFilter, PhysicsSettings,
ContactConstraintManager, ConstraintPart/*, CalculateSolverSteps, ConstraintManager, IslandBuilder,
LargeIslandSplitter, PhysicsUpdateContext, PhysicsSystem, StateRecorder(Impl), PhysicsScene.
**Milestone M5:** a Zig port of `HelloWorld/HelloWorld.cpp` runs and prints the same positions
and velocities as the C++ version (step by step, bit for bit). ✅ Reached:
- `zig build hello-world-check` (part of `zig build parity`) runs both programs and finds their stdout
  byte identical in both precisions.
- HelloWorldParity compares the state bits after every step.

**Milestone M6:** a Zig port of `PerformanceTest` (`ZoltPerformanceTest/`) reproduces the
**Pyramid** hashes in single and double precision, single and multithreaded. ✅ Reached and
exceeded:
- The Pyramid, **ConvexVsMesh** and **HighSpeed** hashes of `determinism_check.yml` all match, in both
  precisions, with `-t=max`, `-t=1` and `-t=2`. No library fix was needed. Zolt's CI checks them.
- Every Phase 5 file is ported, apart from the soft body parts of Phase 9.
- The simulation parity compares Zolt with the C++ PhysicsSystem after every update over about 120
  scenes (gravity, contacts, CCD, the ported unit test cases, HelloWorld, the PerformanceTest scenes).
  It runs in a thread matrix and with failure sweeps.
- The PhysicsTests, PhysicsDeterminismTests, ContactListenerTests, SensorTests, BroadPhaseTests,
  MotionQualityLinearCastTests and PhysicsStepListenerTests suites, and the simulating cases of the
  Phase 4 test files, are ported. Only the cases that need concrete constraint types are left.
- Design and settled decisions: [SimulationArchitecture.md](SimulationArchitecture.md).

### Phase 6: Constraints (`Jolt/Physics/Constraints`) ✅
Fixed, Point, Distance, Hinge, Slider, Cone, SwingTwist, SixDOF, Path (+ PathConstraintPath,
Hermite), Gear, RackAndPinion, Pulley. MotorSettings, SpringSettings, every ConstraintPart and
CalculateSolverSteps were ported in Phase 5. The concrete classes declare `overrides` (D8). Their
settings types went into a comptime `constraint_settings_types` list; Phase 8 replaced it with
`RegisterTypes.factory`.
**Milestone M7:** the constraint unit tests and the constraint cases of PhysicsTests pass. The
simulation parity scene `constraints_chain`, and the constraint part of the PhysicsScene parity, run
bit exact. (The HighSpeed and ConvexVsMesh hashes already match since M6.) ✅ Reached and exceeded:
- All 14 files of the phase are ported; `Jolt/Physics/Constraints` is complete. The parity found no
  numeric bug in the ported classes. The reviews found one safety bug: PhysicsScene unwrapped the
  null settings of a PathConstraint.
- The constraint unit tests of `UnitTests/Physics` are ported (Distance, Hinge, Path, SixDOF, Slider),
  and so are the constraint cases of PhysicsTests (TestPhysicsConstraintUserData) and
  PhysicsDeterminismTests (TestGridOfBoxesConstrained). Zolt adds simulation tests for the seven
  classes without a Jolt test file, and ConstraintManagerTests.
- ConstraintsParity compares every class with the C++ library after every call: creation between
  bodies of every supported motion type, every setter, the solver calls, the state and the settings
  binary state.
- The simulation parity runs about 100 constraint scenes bit exact in its thread matrix with the
  D12 failure sweeps: every class, its limits, springs and motors, mixed islands with contacts,
  LargeIslandSplitter islands, sleeping and waking, cross save / restore. The PhysicsScene parity
  creates random constraints of every class but Path (Jolt's PathConstraint cannot return its
  settings).
- Settled points, the Jolt bugs that are reproduced on purpose and the degenerate cases that the
  harnesses avoid: [SimulationArchitecture.md](SimulationArchitecture.md), section 9.

### Phase 7: Character, Skeleton, Ragdoll, Vehicles ✅
Character, CharacterVirtual, CharacterBase, Skeleton, SkeletalAnimation, SkeletonMapper,
SkeletonPose, Ragdoll, Vehicle* (wheeled, tracked, motorcycle).
Design, groups and harness plan: [CharacterVehicleArchitecture.md](CharacterVehicleArchitecture.md).
**Milestone M8:** **CharacterVirtual** hash matches. ✅ Reached and exceeded:
- All 21 files of the phase are ported: `Jolt/Skeleton` (4 files), `Jolt/Physics/Character` (4),
  `Jolt/Physics/Ragdoll` (1) and `Jolt/Physics/Vehicle` (12) are complete.
- ZoltPerformanceTest runs PerformanceTest's CharacterVirtual scene. With `-q=Discrete` it gives Jolt's hashes
  `0xc1d4e05b164b1221` (single precision) and `0xa0b16727aa16fdb0` (double precision), with `-t=max`, `-t=1` and
  `-t=2`. Zolt's CI checks them.
- CharacterVirtualTests and WheeledVehicleTests are ported, with LoggingCharacterContactListener. Jolt has no unit
  tests for the other classes. Zolt adds inline tests for every class, with allocation failure sweeps.
- SkeletonParity, CharacterBaseParity, RagdollParity and VehiclePartsParity compare the pure functions with the C++
  library bit for bit: animation sampling, poses and skeleton mapping; the character base; the ragdoll settings with
  Stabilize, the constraint priorities and the collision filtering; the engine, transmission, differentials, track
  syncing and the binary state of every settings class.
- The simulation parity runs 62 scenes with characters, character virtuals, ragdolls and wheeled, tracked and
  motorcycle vehicles. They run bit exact in the thread matrix, all but three with the D12 sweeps, and hit no Jolt
  assert (`-Dreference_asserts=true`). After every update the parity also compares each object's state (its SaveState
  bytes and getters) and the character listener records in call order. `perf_character_virtual` compares the first 100
  updates of the M8 scene.
- The reviews found no result difference from Jolt in library code. They found crashes on input that Jolt accepts (a
  ragdoll joint that is its own parent, SkeletonMapper chains of 1 or 2 joints, a wheeled vehicle with many
  differentials), no way to change the number of differentials, coverage gaps and wrong docs. Each was fixed or
  documented before the merges. A last audit of the merged phase against the Jolt headers found no missing API. It
  found one more Zolt-only bug (`createRagdoll` read its settings after releasing them when out of bodies) and doc
  errors, which are fixed.
- The Ragdoll hash stayed M9: PerformanceTest's RagdollScene loads `Assets/Human.tof` through ObjectStream (Phase 8).
  The Ragdoll class itself is complete and compared.
- Settled points, the Jolt bugs that are reproduced on purpose and the harness rules:
  [CharacterVehicleArchitecture.md](CharacterVehicleArchitecture.md), section 6.

### Phase 8: ObjectStream and serialization (`Jolt/ObjectStream`) ✅
RTTI, Factory, StreamUtils, SerializableObject, ObjectStreamIn/Out (text and binary), TypeDeclarations and
the serializable declarations of every class outside SoftBody and Hair. Each class declares its stream schema
next to its fields, with Jolt's names, order and aliases (SerializationArchitecture S5). Reflection over all
fields was rejected (its section 9). The file formats are Jolt's, so `Assets/*.tof` / `*.bof` load.
Design, groups and harness plan: [SerializationArchitecture.md](SerializationArchitecture.md).
**Milestone M9:** **Ragdoll** hash matches → all 5 determinism hashes match. The port is exact. ✅ Reached:
- The 14 Zig files of `Zolt/ObjectStream` (one per Jolt header), RTTI, Factory and StreamUtils are complete, and so
  is the port of ObjectStreamTest. All 81 serializable classes outside SoftBody and Hair are declared, in 64 Zig
  files, and RegisterTypes has Jolt's Factory (`RegisterTypes.factory`). Every restore by RTTI hash goes through it.
- ZoltPerformanceTest runs PerformanceTest's Ragdoll scene, which loads `Assets/Human.tof`,
  `Assets/Human/dead_pose1.tof` and `Assets/terrain2.bof` through ObjectStreamIn. With `-q=LinearCast` it gives
  Jolt's hashes `0x3be6ea76c07a2ac8` (single precision) and `0x8dd6bc501859a228` (double precision), with
  `-t=max`, `-t=1` and `-t=2`. Zolt's CI checks them. First it runs the C++ PerformanceTest of the parity reference
  (`zig build jolt-performance-test`), which must give the same hashes.
- All 5 determinism hashes of `determinism_check.yml` now match, in both precisions and with 1, 2 and all
  threads.
- The parity layers P1 to P10 compare Zolt with the C++ library built with `JPH_OBJECT_STREAM`, in both
  precisions: the schema of every class and the IsKindOf matrix, the defaults, random streams, old schemas,
  truncated streams, more than 10^5 number words, the three assets, the object of ObjectStreamTest, the
  existing binary state parity, and `perf_ragdoll` (the loaded Ragdoll scene and its first updates in a thread
  matrix).
- The Zolt-only sweeps Z1 to Z4 write and read every creatable class, fail allocations, read truncated streams
  and mutate them: no leak, no crash. Zolt-only tests pin Zolt's defined behavior where Jolt is undefined (D1
  to D17).
- During the groups, the parity found one result difference in library code: the number parser accepted words
  just below the smallest normal number, which glibc rejects (P6). The fix came with the F3 merge. The reviews
  found a stack overflow on a class that contains itself, stream values that reach undefined behavior (the
  constraint step overrides), a C++ harness that ran without its allocation hooks, and doc errors. Each was fixed
  before its group merged. G2's allocation failure tests found a double free when a writer fails. It was fixed
  with a test after G2 merged.
- A last audit of the merged phase against the Jolt sources confirmed 37 findings. Among them were Jolt
  behaviors on input that no generator produces (class names with a NUL, a second write on one stream, string
  lengths of 4 GB or more), a class that uses its parent's RTTI written from the wrong address, abstract classes
  that could not be destroyed, a main object of a derived class freed with the wrong size (D17), missing compile
  checks, and a harness and a CI check that let a pending class pass. Each was fixed or documented.
- Phase 9 boundary: the SoftBody and Hair schemas (SoftBodyCreationSettings, SoftBodySharedSettings with its 11
  nested classes, the 10 Hair classes) and `PhysicsScene.mSoftBodies`. A scene stream with soft bodies fails to
  read until then.
- Settled points, the Jolt bugs that are reproduced on purpose and the harness rules:
  [SerializationArchitecture.md](SerializationArchitecture.md), sections 11 and 12.

### Phase 9: Remaining features
SoftBody (with its ObjectStream schemas and `PhysicsScene.mSoftBodies`), the compute interfaces and the CPU compute
backend with the shaders as Zig (`Jolt/Compute`, `Jolt/Compute/CPU`, `Jolt/Shaders`), Hair (CPU path, with its
schemas), the debug renderer (`Jolt/Renderer` and the `JPH_DEBUG_RENDERER` code, behind `-Ddebug_renderer`),
recoverable asserts with `ExpectAssert`, and the remaining unit tests (all of `UnitTests/` ported).
Design, decisions (R1 to R22), groups and harness plan: [RemainingFeaturesArchitecture.md](RemainingFeaturesArchitecture.md).
**Milestone M10:** 100% of `UnitTests/` ported and passing (`port_status.py --check-m10`), in Debug, ReleaseSafe,
ReleaseFast, double precision, `-Duse_llvm=false` and `-Ddebug_renderer=true`.

### Deferred / out of scope for now
- `Jolt/Compute/DX12`, `Jolt/Compute/VK`, `Jolt/Compute/MTL`: the GPU compute backends (and `ComputeSystem::mShaderLoader`).
  Zolt runs hair on the CPU backend (RemainingFeaturesArchitecture.md R21).
- `Jolt/Core/Profiler`: developer tooling, compiled out in the Distribution configuration.
- Samples, JoltViewer, TestFramework: application code; a Zig sample app may come later.
- A C API (`zolt.h`) for use from other languages.

### After the port
Idiomatic API layer, performance work (compare with C++ using PerformanceTest), SIMD tuning
with `@Vector` widths, allocation strategy, and documentation.

## Syncing with upstream Jolt

The port tracks Jolt **v5.6.1 at commit `5830c34`**. To pick up upstream changes:
1. Merge upstream into this repository (the C++ stays authoritative).
2. `git diff 5830c34..<new> --stat -- Jolt UnitTests` lists the C++ files that changed;
   re-port the corresponding Zig files (`Progress.md` shows the mapping).
3. Update the tracked commit here and in `CLAUDE.md`, and the expected hashes in the table above
   from the new `determinism_check.yml`.
4. Run `python3 tools/gen_jolt_sources.py` (the C++ file list of the parity build) and
   `zig build parity`: the parity tests show which Zig code no longer matches the new C++.

## Session checklist (for Claude)

1. Read `CLAUDE.md`, `Docs/Zolt/PortingGuide.md` and this roadmap.
2. `python3 tools/port_status.py --next` to find ready files in the current phase.
3. Port file + tests, run `zig build test` in both precisions, `zig fmt`, regenerate `Progress.md`.
4. Add new conventions to the guide's rename table; commit per coherent unit.
