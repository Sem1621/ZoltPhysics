# Zolt Roadmap

The plan for porting Jolt Physics (~91k lines of C++ in scope) to Zig 0.16. Phases follow the
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
`GJKTests`, which needed the convex shapes. Phase 9 ported them (group F0). Every Geometry function has parity
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
- Every Phase 5 file is ported, apart from the soft body parts, which Phase 9 ported.
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
  nested classes, the 10 Hair classes) and `PhysicsScene.mSoftBodies` came with Phase 9. Until then a scene stream
  with soft bodies failed to read.
- Settled points, the Jolt bugs that are reproduced on purpose and the harness rules:
  [SerializationArchitecture.md](SerializationArchitecture.md), sections 11 and 12.

### Phase 9: Remaining features ✅
SoftBody (with its ObjectStream schemas and `PhysicsScene.mSoftBodies`), the compute interfaces and the CPU compute
backend with the shaders as Zig (`Jolt/Compute`, `Jolt/Compute/CPU`, `Jolt/Shaders`), Hair (CPU path, with its
schemas), the debug renderer (`Jolt/Renderer` and the `JPH_DEBUG_RENDERER` code, behind `-Ddebug_renderer`),
recoverable asserts with `ExpectAssert`, and the remaining unit tests (all of `UnitTests/` ported).
Design, decisions (R1 to R22), groups and harness plan: [RemainingFeaturesArchitecture.md](RemainingFeaturesArchitecture.md).
**Milestone M10:** 100% of `UnitTests/` ported and passing (`port_status.py --check-m10`), in Debug, ReleaseSafe,
ReleaseFast, double precision, `-Duse_llvm=false` and `-Ddebug_renderer=true`. ✅ Reached:
- Every file of the phase is complete:
  - SoftBody: the 12 files of `Jolt/Physics/SoftBody` (8 Zig files), the soft body parts of Body, BodyManager,
    BodyInterface, PhysicsSystem (the four soft body jobs, the CCD response against a soft body face) and
    PhysicsScene, the 13 soft body schemas and `PhysicsScene.mSoftBodies`;
  - the compute interfaces and the CPU backend (`Zolt/Compute`, `Zolt/Compute/CPU`) with the HLSL emulation;
  - the 17 shaders of `Jolt/Shaders` as Zig, with their shared headers and wrappers (`Zolt/Shaders`);
  - Hair on the CPU backend with its 10 schemas, `LibM.expf` and `powf`, and libc++'s `std::shuffle`;
  - the debug renderer (`Zolt/Renderer`: the interface with every helper, the recorder, DebugRendererSimple and the
    playback) and every draw site of the 103 Jolt files with `JPH_DEBUG_RENDERER`. The draw code exists only with
    `-Ddebug_renderer=true`. Without it every hook is `void`, so drawing cannot change a result;
  - recoverable asserts with `ExpectAssert` (R8): the asserts that tests expect let the code continue on Jolt's release
    path, so these tests run in every build;
  - the remaining unit tests: the four ray cases of GJKTests, ComputeTests, SoftBodyTests with
    LoggingSoftBodyContactListener, and the `ExpectAssert` part of TestOutOfContactConstraints.
- Every one of the 590 `TEST_CASE`s of `UnitTests/` has a Zig test of the same name, except the three GPU cases of
  ComputeTests. `port_status.py --check` fails on an M10 gap and on any `TODO(debug_renderer)`. CI runs the tests in
  every M10 configuration.
- The parity layers compare Zolt with the C++ library built with `JPH_USE_CPU_COMPUTE`, in both precisions and in both
  renderer configurations (the second reference has `JPH_DEBUG_RENDERER`):
  - the soft body settings, the SoftBodyShape queries in the pairwise sweep, and the soft body update without a
    PhysicsSystem;
  - 23 soft body scenes in the simulation parity (310 scenes in all), bit exact in the thread matrix with the D12
    sweeps;
  - every shader, dispatch by dispatch on random buffers, against Jolt's CPU backend;
  - `expf` and `powf`, the shuffle, the hair settings, and the hair scenes, which compare every compute and read back
    buffer after every update;
  - the call log of every draw: the helpers, shapes, constraints, bodies, characters, vehicles, skeletons, soft bodies
    and hair; the recorder's `.jor` bytes; drawn scenes, whose state must equal the state without drawing;
  - the release only tests and scenes, which follow Jolt's release build on inputs that Jolt asserts on. Only an
    asserting reference (`-Dreference_asserts=true`) counts Jolt's asserts, and no CI row builds one: it is a manual
    run. Then each case must hit as many asserts on both sides. Without it, a release only test checks (in Debug and
    ReleaseSafe) that its cases hit Zolt's asserts, and a release only scene checks that each run hits at least one.
- The debug renderer changes no result. With `-Ddebug_renderer=true` ZoltPerformanceTest gives the same 5 hashes, also
  while it records every frame (`-r`). `zig build debug-renderer-check` finds its `.jor` bytes equal to those of the C++
  PerformanceTest for the 5 scenes. Zolt's CI checks both, in both precisions.
- The parity and the reviews found Jolt bugs and undefined behavior:
  - the pulley solver reads uninitialized memory: IndependentAxisConstraintPart with a static body (UB22). Zolt starts
    those members at zero, which is Jolt's result on zero-filled memory;
  - `uint3(float3)` of NaN or of a value outside the `uint32` range depends on clang's vectorization at the call site
    (UB11). Zolt uses the scalar conversion for every component;
  - where Jolt is undefined, Zolt has one defined behavior, pinned by Zolt-only tests: UB1 to UB34 of R20. For
    example, a soft body without vertices makes Jolt's soft body job spin forever (UB1);
  - Jolt's quirks where it is defined are reproduced (R20): CreateConstraints adds the last shared edge of a closed mesh
    twice, FromPhysicsSystem stores a soft body's index of the wrong array, and the density loops of `Hair::Draw` swap
    their bounds.
- The reference's `expf` was the older algorithm of Zig's `compiler_rt`, not glibc's. The reference now pins `expf`
  and `powf` (R1, R13), and Zolt's hair results do not depend on the C library.
- Each group was reviewed before its merge. Three major findings were fixed: the `uint3(float3)` conversion (C0), the
  D5 rows with several failed job creations (S5) and the allocation failure sweeps of the hair update (H2). A last
  audit of the merged phase against the Jolt sources confirmed 34 minor findings. Each was fixed or documented.
- Deferred: only the GPU compute backends and the Profiler (see below).
- Settled points, the Jolt quirks that are reproduced on purpose and the harness rules:
  [RemainingFeaturesArchitecture.md](RemainingFeaturesArchitecture.md), R20 and section 6.

### Phase 10: performance and divergence ✅
After Phase 9 Zolt was bit exact, but 1.4 to 14 times slower than C++ Jolt. A benchmark, callgrind profiles and a
read-only analysis found the cause: hidden zero-fills of large stack buffers. This was a bug of the port, not of Jolt.
Phase 10 removed the fills, closed the rest of the codegen gap, fixed memory-safety bugs, and added measurement tools
and a regression guard. Zolt may now diverge from Jolt, but every change of this phase is class A:
- **Class A:** results stay bit exact. The 840 parity tests and Jolt's 5 hashes verify the change.
- **Class B:** results stay deterministic but change. The change needs new golden hashes (see the policy below).
- **Class C:** not about speed: bugs, safety, API.

What was done:
- **W1, hidden zero-fills.** In Zig 0.16 a value with comptime-known fields (`.empty`, `.{}`, a field default) is one
  constant. Storing it fills its `undefined` bytes too. ProcessBodyPair filled 68,624 bytes per body pair, EPA filled
  its 24 KB triangle factory per run, and the contact manifolds, collide results and clip buffers were filled or
  copied whole. They are now built in place, like Jolt's constructors: `StaticArray.init()` (sets only `len`),
  `addOne`, `assign` (copies only the used elements) and `fn init(self: *T, ...)`. memset fell from 80-96% of
  Zolt's instructions to below 0.3%.
- **W2, codegen.** `Core.swap` replaces `std.mem.swap`, which swaps byte by byte. Half floats decode with F16C. The
  QuadTree walks load the node bounds and child IDs with one vector load (inline asm on x86-64,
  [SimulationArchitecture.md](SimulationArchitecture.md) D9). `loadFloat3Unsafe` is one 16 byte load like Jolt's, and
  `Vec3.canLoadFloat3Unsafe` checks each struct it reads. `UVec4.shiftComponents4Minus` uses Jolt's pshufb table, and
  `Vec3.cross`, `Mat44.rotationQuat` and `UVec4.getTrues` use Jolt's SIMD forms. Each change has a test that compares
  its bits with the old code. The asm and intrinsic paths need the LLVM backend and keep a portable fallback.
- **W3, memory safety.** Jolt undefined behavior that valid input reaches now has a defined guard: UB35 (a body woken
  during the CCD resolve has no CCD body), UB36 (empty height field blocks outside the grid) and UB37
  (GetTrianglesNext no longer repeats a block forever). Settings that make Jolt index out of bounds fail at creation
  with `error.InvalidSettings`: UB38 (vehicle wheels, tracks, differentials and anti-roll bars) and UB39 (paths with
  too few points). The rule is in [PortingGuide.md](PortingGuide.md) section 7, the rows in
  [RemainingFeaturesArchitecture.md](RemainingFeaturesArchitecture.md) R20.
  **API change:** `TwoBodyConstraintSettings.createConstraint` returns `TwoBodyConstraintSettings.CreateError`
  (`OutOfMemory` or `InvalidSettings`). The error sets of `BodyInterface.createConstraint`,
  `RagdollSettings.createRagdoll` and `PhysicsScene.createBodies` grow with it. `VehicleConstraint.create` returns
  `VehicleConstraint.CreateError` (the same two errors), and `PathConstraint.setPath` returns
  `error{InvalidSettings}!void`.
- **W5, measurement.** `-Djob_profile=true` records a timeline of the jobs and the idle time of every job thread, and
  `ZoltPerformanceTest -p` writes it as a Chrome trace. `-phase_times` times the scene updates and the physics updates
  apart. The vtable thunks have readable names: `Core.Virtual.Thunk(<class>,.<method>).callN`.
- **W6, CI cost.** The upstream C++ workflows run only when upstream files change. One parity row runs per push, the
  other four every night.
- **W4, regression guard.** All parity tests and hashes passed while 80-96% of the instructions were spurious fills.
  Bit exactness cannot see speed, so three layers guard it. CI runs both steps.
  - The rule "Large values" in [PortingGuide.md](PortingGuide.md) section 5: in code that runs per step, body,
    contact or query, no `.empty`, `.{}` or field default for a value over 256 bytes. Build such values in place, and
    use `Core.swap`.
  - `zig build codegen-check` (`tools/check_codegen.py`) reads `objdump -d -l --inlines` of ZoltPerformanceTest
    (ReleaseFast, x86-64-v3). It fails when library code calls memset or memcpy with a constant size over 512 bytes,
    copies more than 256 bytes from read-only data, or runs `std.mem.swap`. Library code that LLVM inlined into the
    test program counts too (the per-step code of CharacterVirtual is inlined into the scene update). A few sites
    that do not run per body pair are allowed by function name, kind and size: system, constraint and body creation,
    teardown, debug drawing (only with `-Ddebug_renderer`), and one 7 KB copy per update (the steps of
    `PhysicsUpdateContext`). It sees only the code that ZoltPerformanceTest links. It passes with the default options,
    with `-Ddouble_precision=true` and with `-Ddebug_renderer=true`. CI checks the default options. It also fails when the scan proves nothing: a binary that is not x86-64, too few
    library functions, no `PhysicsSystem.update`, or no allowed site.
  - `zig build perf-budget` (`tools/perf_budget.py`) counts the instructions of the physics updates of short single
    threaded runs with callgrind, in ZoltPerformanceTest and in the C++ PerformanceTest. It fails when Zolt/C++ of a
    scene is over its budget (the measurement plus about 10%). Ragdoll and CharacterVirtual have absolute Zolt-only
    budgets: valgrind 3.22 stops the C++ binary at an AVX instruction it does not know. A count below half of its
    measurement fails too: then callgrind did not find the function to count (ZoltPerformanceTest calls
    `PhysicsSystem.update` with `@call(.never_inline, ...)` so that the symbol exists).
  - Self-tests: with `.{}` put back for the EPA edges in `addPoint` and for the reduction manifolds, and with one
    `std.mem.swap` in BinaryHeap, codegen-check names `addPoint` (2,056 bytes), `processBodyPair` (68,624 bytes) and
    the swap. perf-budget then fails every scene: ConvexVsMesh runs 17.4 times the C++ instructions, Pyramid 4.0
    times, HighSpeed 2.7 times, and Ragdoll 66.6e9 instructions instead of 1.68e9. The hashes stay equal. A
    1,024 byte `.empty` put into `CharacterVirtual.extendedUpdate`, which LLVM inlines into the scene update, makes
    codegen-check name `extendedUpdate`. perf-budget fails with "COUNT TOO LOW" on a binary without the
    `PhysicsSystem.update` symbol. Both scripts have a `--self-test`, which CI runs.

Instructions of the physics updates (callgrind, `-t=1`, ReleaseFast, x86-64-v3, `zig build perf-budget`). The
"before" counts come from the analysis at the start of the phase.

| Scene | Steps | Zolt before | Zolt after | C++ | Zolt/C++ before | Zolt/C++ after |
|---|---|---|---|---|---|---|
| ConvexVsMesh | 200 | 36.5e9 | 1.98e9 | 2.04e9 | 17 | 0.97 |
| Pyramid | 60 | 23.8e9 | 4.28e9 | 4.35e9 | 5.4 | 0.98 |
| HighSpeed | 60 | not measured | 2.00e9 | 2.04e9 | | 0.98 |
| CharacterVirtual | 60 | not measured | 0.58e9 | valgrind stops | | |
| Ragdoll | 15 | 67.7e9 (with the setup) | 1.68e9 | valgrind stops | | |

Steps per second (ReleaseFast, x86-64-v3, 4 cores, the same hashes on every run). The three binaries ran one after the
other on a quiet machine: Zolt before Phase 10, Zolt after Phase 10 and the C++ PerformanceTest. Runs vary by a few
percent, so a small difference either way is noise.

| Scene | Zolt 1 thread before | Zolt 1 thread after | C++ 1 thread | Zolt 4 threads before | Zolt 4 threads after | C++ 4 threads |
|---|---|---|---|---|---|---|
| Ragdoll | 3.46 | 48.2 | 50.2 | 13.0 | 93.4 | 95.4 |
| ConvexVsMesh | 28.4 | 244.6 | 229.5 | 98.1 | 359.4 | 389.5 |
| HighSpeed | 64.6 | 138.8 | 143.9 | 144.7 | 218.9 | 207.0 |
| CharacterVirtual | 240.0 | 397.8 | 398.4 | 307.5 | 437.1 | 437.0 |
| Pyramid | 38.6 | 54.5 | 57.7 | 81.7 | 87.7 | 92.3 |

**Policy for work that changes results (class B).**
- Class A work comes first and keeps the full oracle. Only bit exact parity can prove a class A change correct.
- `-Djolt_compat` (default false) comes with the first class B change. A local class B change is written as
  `if (Core.jolt_compat) <Jolt's code> else <new code>`. A change that touches all math goes through one helper
  switch, never through copies of each kernel. (FMA mode needs no `-Djolt_compat`: it follows Jolt's own
  `JPH_USE_FMADD` option, so the C++ library is its oracle. See Phase 11.)
- CI runs `zig build parity -Djolt_compat=true` and Jolt's 5 hashes with the option on. They keep guarding later
  class A work and the syncs with upstream Jolt.
- Zolt owns golden tests for the new behavior, without C++: a state hash per update for each SimScenes scene, a
  multi-threaded run against a single threaded run, and Zolt's own PerformanceTest hashes with `jolt_compat=false`.
  A class B change updates them in the same commit and says why.
- Cross-platform hash rows come before the first class B change: macOS arm64, Windows, aarch64 Linux and x86-64
  `-Dcpu=baseline` must print the same hashes (Jolt's hashes with the option on). Today only Ubuntu x86-64 checks
  them. These rows also hash the portable fallbacks of the W2 paths. A speedup that depends on the ISA and changes
  results must be an explicit build option, never automatic feature detection.
- The C++ emulation layers stay: LibM, `shuffleLibcxx`, `cppFloat` and HLSLToCPP.
- The ratio budgets of `perf-budget` compare the `jolt_compat=true` build with C++. Builds with the option off get
  absolute budgets.

**Later (not in Phase 10).**
- Tier 1b (class A, beyond Jolt; measure with the job profiler first): SIMD contact solving inside the splits of
  large islands (done in Phase 11), a cached AABB prefilter for characters, island scheduling, spinning in the
  thread pool.
- Tier 2 (class B, behind `-Djolt_compat`): box-box SAT instead of EPA, graph-colored batches of small islands,
  8-wide trees. (The FMA math mode came in Phase 11 as `-Dfma`, with Jolt's own sites and a C++ oracle.)
- Class C: four Jolt bugs that Zolt reproduces (the gear Jacobian, CCD double motion, a duplicate soft body edge, the
  ray against a 3-point hull), a complete SaveState, settings round trips, errors instead of panics for data-driven
  input, an optional API facade.
- Not planned: rsqrt / rcp approximations.

### Phase 11: SIMD contact solver and FMA mode ✅
Phase 11 did the next two items of the Phase 10 analysis.

**SIMD contact solver (class A, on by default).** `Zolt/Physics/Constraints/ContactConstraintSimd.zig` solves the
contacts of a batch of a parallel split 8 at a time, one contact per lane of `@Vector(8, f32)`. In a parallel split
no dynamic body appears twice, so the contacts of a batch do not share a velocity, and each lane runs the operations of
the scalar solver in the same order. Results do not change: the parity suite and Jolt's hashes prove it, and a
Zolt-only test compares the SIMD and the scalar solver bit for bit on random batches. The non-parallel split, the
islands that are not split, and contacts with a static or kinematic body stay scalar. `-Dsimd_contact_solver=false`
turns it off for A/B tests (same bits). The design and the exactness rules are in
[SimulationArchitecture.md](SimulationArchitecture.md) section 10. Pyramid's physics updates run 2.3% fewer
instructions (4.28e9 to 4.18e9, 0.96 of C++), and the contact solve goes from 26.7% to 24.6% of them. Wall clock gains
more, because 8 lanes fill the wait on the dependency chain from one contact point to the next. Scenes without large
islands do not change.

**FMA mode `-Dfma=true` (off by default).** Jolt has 8 fused multiply-add sites behind `JPH_USE_FMADD`: Vec4, Vec3 and
DVec3 `sFusedMultiplyAdd`, the scalar, Vec4 and Vec3 `DifferenceOfProducts` (and so `Vec3::CrossPrecise`),
`Quat::operator*` and `Quat::sMultiplyImaginary`. With `-Dfma=true` Zolt uses `@mulAdd` at exactly these sites
(`if (Core.use_fmadd) ...`), and the C++ reference of the parity tests is built with `-DJPH_USE_FMADD`. Jolt's
`CROSS_PLATFORM_DETERMINISTIC` only stops the automatic define, so the reference fuses exactly these sites. So FMA
mode has a C++ oracle too: the parity suite proves it bit for bit, and its 5 hashes come from the C++ PerformanceTest
built the same way (CLAUDE.md lists them). It needs no `-Djolt_compat`. Rules:
- FMA mode needs hardware FMA (x86-64 with FMA3, such as `-Dcpu=x86_64_v3`, or aarch64): Zig's software fma is not
  correctly rounded, so `Core.zig` stops the build on a CPU without it.
- Only x86-64 can be the oracle: NEON's `vmlaq_f32` does not fuse under `-ffp-contract=off`. On another target the
  steps that build the C++ reference fail with `-Dfma=true`. On aarch64 Zolt's `@mulAdd` fuses like x86-64's FMA3, so a
  macOS arm64 CI row checks the C++ FMA hashes of the short runs.
- `zig build codegen-check` fails on an FMA instruction in the library without `-Dfma`, and with `-Dfma` on one that
  does not come from the 8 sites.
- The 8 sites are mostly about precision. Ragdoll (many quaternion products) runs about 8% faster with them, in C++
  and in Zolt alike. The other scenes do not change beyond the noise.

Wall clock (steps/s, ReleaseFast, x86-64-v3, 4 cores). The six binaries ran interleaved in 5 rounds on a quiet
machine; the table shows the median. Runs vary by 5-10%, so only differences beyond that are real. Phase 10 is Zolt
before Phase 11, "SIMD off" is `-Dsimd_contact_solver=false`, and the FMA columns are `-Dfma=true` and the C++
PerformanceTest built with `-DJPH_USE_FMADD`. Every run printed its expected hash.

| Scene | Threads | C++ | C++ FMA | Zolt Phase 10 | Zolt Phase 11 | SIMD off | Zolt FMA |
|---|---|---|---|---|---|---|---|
| Ragdoll | 1 | 49.7 | 53.6 | 48.4 | 45.1 | 48.1 | 52.6 |
| Ragdoll | 4 | 98.5 | 106.2 | 94.3 | 95.4 | 96.2 | 105.4 |
| ConvexVsMesh | 1 | 240.3 | 234.1 | 238.6 | 237.1 | 240.9 | 234.7 |
| ConvexVsMesh | 4 | 419.0 | 422.4 | 408.0 | 400.6 | 378.0 | 390.6 |
| HighSpeed | 1 | 164.4 | 163.5 | 152.3 | 159.7 | 149.4 | 154.1 |
| HighSpeed | 4 | 217.2 | 211.1 | 215.9 | 211.7 | 214.6 | 214.8 |
| CharacterVirtual | 1 | 451.3 | 451.2 | 429.2 | 410.1 | 408.7 | 416.6 |
| CharacterVirtual | 4 | 459.8 | 468.2 | 438.9 | 427.0 | 415.8 | 412.4 |
| Pyramid | 1 | 57.4 | 56.8 | 56.9 | 76.5 | 56.9 | 74.2 |
| Pyramid | 4 | 94.5 | 92.7 | 94.2 | 106.4 | 92.4 | 106.5 |

The SIMD contact solver makes Pyramid 34% faster on 1 thread and 13% faster on 4 threads, 33% and 13% faster than
C++. The other scenes have no large islands, so it does not change them (their differences are noise: Phase 10 and
"SIMD off" run the same code).

### Deferred / out of scope for now
- `Jolt/Compute/DX12`, `Jolt/Compute/VK`, `Jolt/Compute/MTL`: the GPU compute backends (and `ComputeSystem::mShaderLoader`).
  Zolt runs hair on the CPU backend (RemainingFeaturesArchitecture.md R21).
- `Jolt/Core/Profiler`: developer tooling, compiled out in the Distribution configuration.
- Samples, JoltViewer, TestFramework: application code; a Zig sample app may come later.
- A C API (`zolt.h`) for use from other languages.

### After the port
Idiomatic API layer, more performance work (Phase 10 lists what comes next), SIMD tuning
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
