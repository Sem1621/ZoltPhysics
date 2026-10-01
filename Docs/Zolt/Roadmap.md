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
  in separate git worktrees; the registry files `Zolt/zolt.zig` and `ZoltTests/unit_tests.zig`
  are the only expected merge conflicts (keep their lists sorted to make them trivial).

## Phases

### Phase 0: Infrastructure ✅
`build.zig` (options `double_precision`, `object_layer_bits`), module layout, porting guide,
`tools/strip_isa.py`, `tools/port_status.py`, `tools/install_zig.sh`, CI, SessionStart hook.

### Phase 1: Math (`Jolt/Math`, `UnitTests/Math`)
Vec3, Vec4, UVec4, Float2/3/4, Swizzle, Math, Trigonometry, Quat, Mat44, DVec3, Double3, DMat44,
Real, BVec16, HalfFloat, Vector, Matrix, DynMatrix, GaussianElimination, EigenValueSymmetric,
FindRoot. Plus `Core/HashCombine` and `Core/Mt19937` (std::mt19937 replacement).
**Milestone M1:** all of `UnitTests/Math` passes in both precisions.

### Phase 2: Core (`Jolt/Core`, `UnitTests/Core`)
Containers and utilities the physics code depends on:
- Containers: StaticArray, HashTable + UnorderedMap/UnorderedSet (Jolt's own, for deterministic
  iteration order), BinaryHeap, FixedSizeFreeList, LockFreeHashMap, ByteBuffer, StridedPtr,
  ObjectToIDMap.
- Algorithms: QuickSort, InsertionSort (Jolt's own, std sorts are not guaranteed identical for equal keys).
- Memory: TempAllocator (TempAllocatorImpl, TempAllocatorMalloc), STLTempAllocator / STLLocalAllocator
  equivalents as `std.mem.Allocator` adapters.
- Reference counting: Reference.h (RefTarget/Ref/RefConst, see the guide).
- Threading: Atomics, Mutex, MutexArray, Semaphore, JobSystem, JobSystemWithBarrier,
  JobSystemThreadPool, JobSystemSingleThreaded (std.Thread + std.Io primitives).
- Misc: Color, StringTools, LinearCurve, TickCounter, Factory/RTTI (minimal, grows with ObjectStream),
  StreamIn/StreamOut/StreamUtils/StreamWrapper (over std.Io.Reader/Writer), FPControlWord,
  FPFlushDenormals, Profiler (no-op).
**Milestone M2:** `UnitTests/Core` passes, including JobSystemTest.

### Phase 3: Geometry (`Jolt/Geometry`, `Jolt/AABBTree`, `Jolt/TriangleSplitter`)
AABox, OrientedBox, Plane, Sphere, Triangle, IndexedTriangle, Indexify, Ellipse, ClosestPoint,
ConvexSupport, GJKClosestPoint, EPAPenetrationDepth (+ EPAConvexHullBuilder), RayAABox,
RayTriangle, RaySphere, RayCapsule, RayCylinder, ConvexHullBuilder (+2D), MortonCode, AABox4,
then AABBTree builder and TriangleSplitter (used by MeshShape).
**Milestone M3:** `UnitTests/Geometry` passes.

### Phase 4: Collision (`Jolt/Physics/Collision`)
1. Basics: ObjectLayer, BroadPhaseLayer, SubShapeID, PhysicsMaterial, GroupFilter(s),
   CollisionCollector(s), ShapeFilter, BackFaceMode, ActiveEdgeMode, CollectFacesMode,
   ContactListener types, CastResult, CollidePointResult, RayCast, ShapeCast, AABoxCast.
2. Shape base + convex shapes: Shape, ConvexShape, SphereShape, BoxShape, CapsuleShape,
   TaperedCapsuleShape, CylinderShape, TaperedCylinderShape, ConvexHullShape, TriangleShape,
   PlaneShape, EmptyShape (vtable pattern A from the guide).
3. Collision algorithms: CollisionDispatch, CollideConvexVsTriangles, CollideSphereVsTriangles,
   CastConvexVsTriangles, CastSphereVsTriangles, ManifoldBetweenTwoFaces, ActiveEdges,
   InternalEdgeRemovingCollector, TransformedShape, NarrowPhaseQuery, EstimateCollisionResponse.
4. Composite shapes: DecoratedShape, RotatedTranslatedShape, ScaledShape,
   OffsetCenterOfMassShape, CompoundShape, StaticCompoundShape, MutableCompoundShape,
   MeshShape, HeightFieldShape.
5. Broad phase: BroadPhase, QuadTree, BroadPhaseQuadTree, BroadPhaseBruteForce.
**Milestone M4:** shape and collision unit tests (`UnitTests/Physics/*Shape*`, `CollideShapeTests`,
`RayShapeTests`, `CastShapeTests`, `ConvexVsTrianglesTest`, `BroadPhaseTests`, ...) pass.

### Phase 5: Bodies and simulation (`Jolt/Physics`, `Jolt/Physics/Body`, `Jolt/Physics/Constraints` core)
MotionType, MotionQuality, AllowedDOFs, BodyID, MassProperties, MotionProperties, Body,
BodyCreationSettings, BodyManager, BodyLock*, BodyInterface, BodyFilter, PhysicsSettings,
ContactConstraintManager, ConstraintPart/*, CalculateSolverSteps, ConstraintManager, IslandBuilder,
LargeIslandSplitter, PhysicsUpdateContext, PhysicsSystem, StateRecorder(Impl), PhysicsScene.
**Milestone M5:** a Zig port of `HelloWorld/HelloWorld.cpp` runs and prints the same positions
and velocities as the C++ version (step by step, bit for bit).
**Milestone M6:** a Zig port of `PerformanceTest` (`ZoltPerformanceTest/`) reproduces the
**Pyramid** hashes in single and double precision, single and multithreaded.

### Phase 6: Constraints (`Jolt/Physics/Constraints`)
Fixed, Point, Distance, Hinge, Slider, Cone, SwingTwist, SixDOF, Path (+ PathConstraintPath,
Hermite), Gear, RackAndPinion, Pulley, MotorSettings, SpringSettings, ConstraintPart extras.
**Milestone M7:** constraint unit tests pass; **HighSpeed** and **ConvexVsMesh** hashes match.

### Phase 7: Character, Skeleton, Ragdoll, Vehicles
Character, CharacterVirtual, CharacterBase, Skeleton, SkeletalAnimation, SkeletonMapper,
SkeletonPose, Ragdoll, Vehicle* (wheeled, tracked, motorcycle).
**Milestone M8:** **CharacterVirtual** hash matches.

### Phase 8: ObjectStream and serialization (`Jolt/ObjectStream`)
RTTI, Factory, SerializableObject, ObjectStreamIn/Out (text and binary), TypeDeclarations,
`SaveBinaryState` / `sRestoreFromBinaryState` of shapes, constraints and settings. Prefer comptime
reflection over Jolt's macro-generated RTTI tables, but keep the file formats identical so that
`Assets/*.tof` / `*.bof` load.
**Milestone M9:** **Ragdoll** hash matches → all 5 determinism hashes match. The port is exact.

### Phase 9: Remaining features
SoftBody, Hair (CPU path), debug renderer interface (`Jolt/Renderer`, behind a build option),
`ConfigurationString`, remaining unit tests (all of `UnitTests/` ported).
**Milestone M10:** 100% of `UnitTests/` ported and passing.

### Deferred / out of scope for now
- `Jolt/Compute`, `Jolt/Shaders`: GPU compute backends (DX12/Vulkan/Metal), only used for GPU hair.
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

## Session checklist (for Claude)

1. Read `CLAUDE.md`, `Docs/Zolt/PortingGuide.md` and this roadmap.
2. `python3 tools/port_status.py --next` to find ready files in the current phase.
3. Port file + tests, run `zig build test` in both precisions, `zig fmt`, regenerate `Progress.md`.
4. Add new conventions to the guide's rename table; commit per coherent unit.
