# Remaining features architecture (Phase 9)

This is the binding design for Phase 9:
- SoftBody (`Jolt/Physics/SoftBody`, the soft body parts of Body, BodyManager, BodyInterface, PhysicsSystem and
  PhysicsScene), with its ObjectStream schemas and `PhysicsScene.mSoftBodies`;
- the compute interfaces and the CPU compute backend (`Jolt/Compute`, `Jolt/Compute/CPU`) and the shaders
  (`Jolt/Shaders`), all as Zig;
- Hair, on the CPU backend, with its schemas;
- the debug renderer (`Jolt/Renderer` and the `JPH_DEBUG_RENDERER` code of 103 Jolt files), behind a build option;
- recoverable asserts and `ExpectAssert`;
- the remaining unit tests (milestone M10).

It adds to [PortingGuide.md](PortingGuide.md), [CollisionArchitecture.md](CollisionArchitecture.md),
[SimulationArchitecture.md](SimulationArchitecture.md) (D1, D4, D5, D12),
[CharacterVehicleArchitecture.md](CharacterVehicleArchitecture.md) and
[SerializationArchitecture.md](SerializationArchitecture.md) (S5, S7, S15 to S18). Read those first. Every decision below
is an instruction for porters. A porter who finds a decision wrong reports it with the evidence instead of working
around it.

The design was chosen from three competing prototypes (fidelity, Zig design, delivery), two judges and a synthesis.
The plan, the groups and the soft body job rules come from the delivery proposal. The Zig machinery (the void gates, the
renderer, the recorder, the recoverable asserts) comes from the Zig design proposal. The reference build fixes, the
compute and soft body seeds and the table of Zolt-defined behavior come from the fidelity proposal. Every critical flaw
that the judges found was checked against the Jolt sources and fixed here (section 11).

The synthesis prototype was the start of the foundation (section 10): the build option, the void gates, the renderer
interface and its recorder, the compute interfaces, the recoverable asserts and the reference build change.

Phase 9 is done. Every group of section 6 is merged, every file of the scope is complete and milestone M10 is reached.
A completeness audit checked the merged phase against the Jolt sources ("Settled in the audit", section 6). Section 10
has the final state.

## 1. Scope

| Area | Jolt files | Tests | Lines (C++, `wc -l`) |
|---|---|---|---|
| Soft body | `Physics/SoftBody/*` (12 files); the soft body parts of `PhysicsSystem`, `BodyManager`, `BodyInterface`, `Body`, `PhysicsScene`, `CollideSoftBodyVertexIterator` | `UnitTests/Physics/SoftBodyTests.cpp`, `UnitTests/LoggingSoftBodyContactListener.h` | 4588, plus about 400 in the other files |
| Compute interfaces | `Compute/ComputeBuffer.h`, `ComputeQueue.h`, `ComputeShader.h`, `ComputeSystem.h/.cpp` | `UnitTests/Compute/ComputeTests.cpp` (`TestComputeCPU`) | 286 |
| CPU backend | `Compute/CPU/*` (ComputeBufferCPU, ComputeQueueCPU, ComputeShaderCPU, ComputeSystemCPU, HLSLToCPP, ShaderWrapper, WrapShaderBegin / Bindings / End) | (same) | 1096 |
| Shaders | `Shaders/*` (31 `.h` / `.cpp` units, 17 `.hlsl` files) | through ComputeTests and Hair | 1088 + 766 |
| Hair | `Physics/Hair/Hair`, `HairSettings`, `HairShaders`, `RegisterHair` | (none in Jolt) | 2611 |
| Debug renderer | `Renderer/DebugRenderer`, `DebugRendererRecorder`, `DebugRendererPlayback`, `DebugRendererSimple`; `JPH_DEBUG_RENDERER` blocks in 103 files (about 2600 guarded lines) | (none in Jolt; `RecordDebugOutput` of `UnitTests/PhysicsTestContext`) | 2163 |
| Asserts | `Core/IssueReporting`, `ExpectAssert` of `UnitTests/UnitTestFramework.h` | `TestOutOfContactConstraints` | (small) |
| Tests | `UnitTests/Geometry/GJKTests.cpp` (4 cases left) | itself | 248 |

- The lines are `wc -l` of the files in the "Jolt files" column. Progress.md counts non-blank lines.
- At the start of Phase 9, Zolt had 5 soft body stubs, 2 complete soft body files, 6 partial files with soft body gaps
  and 142 `TODO(debug_renderer)` markers in 53 files (138 after the prototype).
- At the end of Phase 9, every file of the scope is complete and no `TODO(debug_renderer)` is left. `Core.zig` stays
  partial, as since Phase 2: it holds only the configuration values that the code needs, `debug_renderer` among them.
- Stays deferred: the GPU compute backends (`Compute/DX12`, `VK` and `MTL` with its `.mm` files, 3589 lines) and
  `Core/Profiler` (R21).

**Milestone M10.** 100% of `UnitTests/` is ported and passes:
- every unit of `UnitTests/` is complete or n/a in Progress.md;
- `python3 tools/port_status.py --check-m10` passes: every `TEST_CASE` of `UnitTests/` has a Zig `test` of the same name
  in the file that ports it (R22);
- `zig build test` passes in Debug, ReleaseSafe, ReleaseFast, double precision, `-Duse_llvm=false` and
  `-Ddebug_renderer=true`.

M10 needs 5 units: GJKTests, UnitTestFramework (ExpectAssert), ComputeTests, SoftBodyTests and
LoggingSoftBodyContactListener. Hair and the debug renderer are Phase 9 scope, but no unit test needs them.

M10 is reached (S5). Since the audit, `port_status.py --check` fails on an M10 gap, and CI runs every configuration of
the list above (R22, section 5).

**Phase 9 is done** when M10 holds, every file of the scope is complete, no `TODO(debug_renderer)` is left, and the full
parity and the five determinism hashes pass in both renderer configurations. `port_status.py --check` fails on a
`TODO(debug_renderer)` line.

## 2. Decisions

### R1 Reference build

- The C++ reference always gets `-DJPH_USE_CPU_COMPUTE` (Jolt's CMake default, `USE_CPU_COMPUTE=ON`).
  - It compiles the CPU backend and the shaders as C++ (`Shaders/HairWrapper.cpp`, `TestComputeWrapper.cpp`).
  - It is not a `JPH_VERSION_ID` feature bit.
  - The delivery prototype preprocessed all 143 Jolt translation units of the parity build with and without it: 134 are
    identical. The 9 others are the compute, shader and hair units. So no existing result can change.
- The C++ reference gets `-DJPH_DEBUG_RENDERER` exactly when `-Ddebug_renderer=true` (R2).
  - It is bit 6 of `JPH_VERSION_ID`, so every C++ unit gets the same value: Jolt, the references, HelloWorld and
    PerformanceTest.
  - It changes results that the parity sees: `Constraint::ToConstraintSettings` copies `mDrawConstraintSize`, and the
    configuration string gains "(Debug Renderer) ". Zolt does the same under `Core.debug_renderer`.
  - Bite proof (fidelity prototype): with the define and without Zolt's copy, "Constraint parity: fixed" (1476 records),
    "Constraint parity: hinge" (1516) and the scene `vehicle_wheeled_ops` fail.
  - Jolt's own results do not change with the define, even while recording: Jolt and PerformanceTest built with it give
    all 5 published hashes with `-r` (delivery prototype).
- 12 reference files are fixed so that they compile with the define: Capsules, Composite, Compounds, ConvexHullShape,
  Convex, Cylinders, Decorated, MeshShape, PlaneEmpty, ShapeCore, TriangleShape and Triangles references. The fixes are
  `JPH_IF_DEBUG_RENDERER(, RVec3::sZero())` arguments, the overrides with the extra parameter, and empty `Draw`
  overrides of the parity test shapes. `VehiclePartsReference.cpp` needs no fix: its `Controller` class is never
  instantiated.
- The reference pins `expf` and `powf` (R13): a new reference file defines both with the generic algorithm.
  - Zig links its own `compiler_rt` into the parity executable. It defines a weak, hidden `expf` with an older
    algorithm. The current parity binary resolves `expf` to it (`nm`: `t expf` at the address of
    `compiler_rt.exp.expf`). A judge measured 8.2 million differences from glibc's `expf` on the floats in [-16, 0).
  - `powf` resolves to glibc (`powf@GLIBC_2.27`, the FMA ifunc variant on this CPU).
  - A strong definition in a reference object wins over both (verified by a judge). Only Hair calls `std::exp` and
    `std::pow` on floats, so nothing else changes.
- Every new reference file installs Jolt's allocation hooks itself, through `ZoltParity/ParityHooks.h` (zero-filling
  `Allocate` / `Reallocate` / `AlignedAllocate`, the counting `AssertFailed` of SimulationReference.cpp). A filtered
  parity run links only the reference objects that it uses. Without its own hooks a reference may see Jolt's default
  allocator or a null `Allocate` (the delivery prototype crashed on that).
- Each renderer configuration is its own reference build (about 5 minutes). `-Dreference_asserts` is unchanged.

### R2 The `debug_renderer` option and its void gates

- `-Ddebug_renderer=true|false`, default **false**. It is `zolt_options.debug_renderer` and `Core.debug_renderer`.
- Default false, because:
  - Zolt follows Jolt's Distribution configuration, which produces the determinism hashes and has no renderer;
  - the default build stays the Phase 8 build;
  - one `zig build parity` needs one C++ reference.
- Every `#ifdef JPH_DEBUG_RENDERER` declaration exists only with the option. Without it the declaration is `void`. So the
  compiler proves that drawing cannot change a result.

| Jolt | Zolt |
|---|---|
| a member under `#ifdef JPH_DEBUG_RENDERER` (`mDrawConstraintSize`, `mRPMMeterPosition`, the shape caches, `mSkinStateTransform`) | `field: if (Core.debug_renderer) T else void = if (Core.debug_renderer) value else {}` (the asserts-only member pattern) |
| a virtual function under the define (`Shape::Draw`, `Constraint::DrawConstraint`, `VehicleController::Draw`, ...) | vtable entry `virtual.OptionalVirtual(Core.debug_renderer, *const fn (...) void)`: the function pointer with the option, `void` (no slot) without it |
| the dispatcher of that virtual function | `pub const draw = if (Core.debug_renderer) drawDispatch else {};` with a private `fn drawDispatch` |
| an override or a non-virtual function under the define | a gated declaration: `pub const draw = if (Core.debug_renderer) drawImpl else {};` with a private `fn drawImpl`. List `.draw` in `overrides` in both configurations |
| an accessor under the define (`Get/SetDrawConstraintSize`, `SetRPMMeter`) | a gated declaration, like an override |
| a parameter under the define (`inBaseOffset`, `inCenterOfMass`, `inDrawConstraints`) | internal function: a trailing parameter `if (Core.debug_renderer) T else void`, callers pass `if (Core.debug_renderer) value else {}`. Public function (`getSubmergedVolume`, `applyBuoyancyImpulse`): a field of its trailing options struct with a default, so `.{}` compiles in both configurations |
| `JPH_IF_DEBUG_RENDERER(x)` / `JPH_IF_NOT_DEBUG_RENDERER(x)` | `if (Core.debug_renderer) x else {}` / `if (!Core.debug_renderer) x else {}` |
| a statement block under the define | `if (Core.debug_renderer) { ... }` |

- Why gated declarations: without the option the body of `drawImpl` is never analyzed. So a body may read a member that
  is `void` then. A `pub fn draw` that is always compiled fails as soon as it reads such a member (a judge reproduced
  this under `zolt.zig`'s `refAllDeclsRecursive`).
- `Virtual.zig` treats a `void` entry as a virtual function that is compiled out: `make` stores `{}`, the name may be
  listed in `overrides`, and the signature check only runs when the entry exists (prototype, with a test of both
  configurations).
- Draw code is type checked only with the option. CI runs `zig build check -Ddebug_renderer=true`, tests and parity with
  the option (section 5), so the draw code cannot rot.
- `Zolt/Renderer/*.zig` always compile and are tested in both configurations. They are a library of types and helpers.
  Only the simulation hooks are gated.
- Pure virtual draw functions kept a placeholder in the root's `impl` until their group had ported every override:
  `Shape.impl.draw` until D2, `Constraint.impl.drawConstraint` until D3, `VehicleController.impl.draw` until D4. A
  placeholder panicked with "TODO(debug_renderer)". Every placeholder is gone, and `make` enforces Jolt's pure virtual
  functions again.
- A default body of a virtual function under the define is a gated declaration too: `Shape.impl.drawGetSupportFunction`
  and `drawGetSupportingFace` (settled in the audit). The test "DebugRenderer: the hooks compile away without
  -Ddebug_renderer" checks that both are `void` without the option.
- Zero cost when off is checked by a test: every shape vtable has the size of its non-void entries (prototype). D2, D3
  and D4 added the same check for their vtables and `@sizeOf` pins of the classes with debug-only members (Constraint,
  the cached shapes, SoftBodyMotionProperties, the wheeled and tracked controllers).
- `getConfigurationString` adds "(Debug Renderer) " after "(Cross Platform Deterministic) " (prototype).

### R3 The renderer interface, recorder, simple renderer and playback

- `DebugRenderer` (`Zolt/Renderer/DebugRenderer.zig`) is a pattern A root. Jolt keeps data in the base: the
  predefined geometry and four caches. Its vtable is Jolt's 6 pure virtual functions: `drawLine`, `drawTriangle`,
  `createTriangleBatch`, `createTriangleBatchIndexed`, `drawGeometry` and `drawText3D` (plus the generated `deinit`).
- Types nest like Jolt's:
  - `Vertex` (36 bytes) and `Triangle` (108 bytes) are extern structs with size checks. The recorder writes their bytes.
  - `CastShadow`, `DrawMode`, `CullMode` are `enum(i32)`, because Jolt writes them as `int`.
  - `BatchTarget` is Jolt's `RefTargetVirtual` (a pattern A root with a reference count). `Batch` is `Ref(BatchTarget)`.
  - `LOD`, `Geometry` (a RefTarget) and `GeometryRef`.
  - `DrawFlags` holds Jolt's 14 static draw flags (R4).
- Overloads get names: `drawWireBoxMatrix` (`DrawWireBox(RMat44, AABox)`), `drawWireBoxOriented`
  (`DrawWireBox(OrientedBox)`), `drawBoxMatrix`, `createTriangleBatchIndexed`, `drawGeometryAutoBounds` (the
  non-virtual `DrawGeometry` that computes the bounds and the LOD scale). Jolt's default arguments are options structs.
- `init(comptime T, allocator, io)` is Jolt's constructor. `initialize()` is Jolt's `Initialize`: it creates the 25
  predefined batches in Jolt's order and returns `Allocator.Error`. `io` serves the locks of R6 and of the recorder.
- Every helper is a literal port in Jolt's operation order: the corner order of `DrawWireBox`, the recursion of
  `DrawWireSphere`, `ceil` in `DrawPie`, `(max - min) * i / 16` in the swing pyramid, `CreateCylinder`,
  `Create8thSphere`, the 4 LODs.
- The four caches (swing cone, swing pyramid, pie, tapered cylinder) and their `prev` twins use Zolt's `UnorderedMap`
  with Jolt's key hash and equality. A cache hit decides whether a batch is created and recorded. A NaN key misses
  every frame in Jolt and in Zolt.
- `ConvexShape::DrawGetSupportingFace` iterates an `UnorderedMap<SupportingFace, Array<Vec3>>`. Zolt uses its port of
  Jolt's HashTable with Jolt's `Hash` of the key, so the draw order and the distinct colors are Jolt's.
- `DebugRendererRecorder` is complete (prototype). It writes Jolt's `.jor` stream byte for byte: `u8` commands, `u32`
  IDs, raw vertices and triangles, `RVec3` as 12 or 24 bytes, `RMat44` as 64 or 72 bytes, enums as `i32`. Create
  commands are written at call time under its mutex; frame data at `endFrame`. Its geometry map holds a reference to
  every geometry and is never cleared, like Jolt's. It is initialized in place (`init(self, allocator, io, stream)`),
  because its base is handed out as `*DebugRenderer`.
- `DebugRendererSimple` is an abstract level (pattern A, `impl`). It implements the batches as triangle lists and
  `drawGeometry` with Jolt's LOD selection (LOD 0 until `setCameraPos`). It takes no lock, like Jolt.
- `DebugRendererPlayback` is a plain struct: `init(allocator, renderer)`, `parse(stream) !void`, `getNumFrames`,
  `drawFrame(i)`. Jolt replays lines, then triangles, then texts, then geometries. A batch or geometry ID that the
  stream does not define and a stream cut short follow UB7.

### R4 No `sInstance`, no static draw flags

- Jolt's renderer is a process singleton (`DebugRenderer::sInstance`). 14 static flags select what the simulation draws.
  Zolt keeps neither. Every reader has an owner that can hold the state.

| Jolt | Read by | Zolt |
|---|---|---|
| the 14 static draw flags (`MeshShape::sDrawTriangleGroups`, `ContactConstraintManager::sDrawContactPoint`, `CharacterVirtual::sDrawConstraints`, ...) | code that draws into a renderer | fields of `DebugRenderer.flags: DrawFlags`, grouped by class: `Class::sDrawFoo` is `flags.class.draw_foo` (`renderer.flags.mesh_shape.draw_triangle_groups`) |
| `DebugRenderer::sInstance` in the PhysicsSystem jobs (CCD draws, contact point and manifold draws) | ProcessBodyPair, ManifoldBetweenTwoFaces, the contact manager, the CCD jobs | `PhysicsSystem.debug_renderer: if (Core.debug_renderer) ?*DebugRenderer else void`, set with `setDebugRenderer` between updates. The contact manager reads it through its system. ManifoldBetweenTwoFaces receives it in its debug parameter |
| `sInstance` in CharacterVirtual (`Update`, `WalkStairs`, `StickToFloor`) | CharacterVirtual | the renderer of its `system` (`system.debug_renderer`) |
| `sInstance` in the `GetSubmergedVolume` chain and `Body::ApplyBuoyancyImpulse` | public API without a system | `getSubmergedVolume(..., .{ .debug = .{ .base_offset, .renderer } })` and `applyBuoyancyImpulse(..., .{ .debug_renderer })`: fields that are `void` without the option |
| `sInstance` in the vehicle controllers' `Draw` (the per-wheel texts) | `Draw`, which receives `inRenderer` | `inRenderer` |

- Jolt allows one renderer at a time. With one renderer, the flags on the renderer behave like Jolt's static flags.
  `inRenderer` is `sInstance` in the vehicle `Draw`.
- Jolt dereferences a null `sInstance` when a flag is set without a renderer. Zolt draws nothing then.
- Set the flags and `setDebugRenderer` between updates. The simulation reads them from job threads.
- The tests and tools that have a renderer pass it: `PhysicsTestContext.recordDebugOutput` and ZoltPerformanceTest's
  `-r` call `setDebugRenderer` with their recorder, where Jolt's recorder constructor sets `sInstance`.
- The rule "static member `sFoo` → `pub var foo`" stays for the statics that must be global (`CharacterID.next_id`).

### R5 Allocation failure while drawing

- Draw functions return `void`, like Jolt's. Draws are called from jobs and from deep inside the simulation. An error
  union there would add error paths to the simulation for output that never affects results.
- A draw call whose allocation fails drops that primitive and sets the renderer's `out_of_memory` latch. The latch is an
  atomic, because draws come from job threads.
- `DebugRendererRecorder.endFrame` writes the frame and then returns `error.OutOfMemory` when the latch is set
  (prototype).
- Construction (`initialize`, the recorder's `init`) returns `Allocator.Error`.
- A draw never poisons the physics update (UB9).

### R6 Shape draw caches

- Jolt caches the draw geometry of ConvexHullShape, TaperedCapsuleShape, MeshShape and HeightFieldShape in `mutable`
  members written by the const `Draw`. Rule M forbids writing through `*const Shape`.
- The cache is a heap side block: `draw_cache: if (Core.debug_renderer) ?*DrawCache else void` (null for a shape that
  was never built from settings, restored or cloned: it draws nothing; settled in D2). It holds Jolt's members
  (`geometry`, `cached_use_material_colors`, `cached_triangles_colored_per_group`) and a `zolt.Mutex`.
  - It is allocated with the shape's allocator in the fallible construction paths (`initFromSettings`,
    `restoreBinaryState`, `clone`) and freed in `destruct`.
  - It is written through the pointer, under the mutex, with the renderer's `io`.
  - `HeightFieldShape.clone` shares the geometry reference; `setHeights` clears it, like Jolt (Jolt's `SetMaterials`
    keeps it; settled in D2).
- Jolt races when two CCD jobs draw the same shape for the first time. Zolt does not, and a single-threaded draw stream
  is byte identical (UB10).
- The four caches of the renderer base (R3) are locked the same way: a tapered cylinder can be drawn by several CCD
  jobs at once. The cache lock is always taken before the recorder's lock.

### R7 Draw text

- Jolt's draw texts use `printf` with `%.0f`, `%.1f`, `%.2f` and `%.3f`. `std.fmt` rounds differently on ties.
- `StringTools.writeFloatFixed(writer, value, decimals)` and its `{f}` wrapper `StringTools.cppFixed(value, decimals)`
  reproduce glibc's fixed notation: the exact decimal expansion of the value, rounded half to even. They reuse the exact
  integer arithmetic of M5's `%g` writer, extended to more than 17 significant digits for large values.
- A draw site writes `stringFormat(allocator, "{f}", .{StringTools.cppFixed(value, 2)})` where Jolt writes
  `StringFormat("%.2f", value)`.
- Parity: CoreParity's snprintf layer gets `%.0f` to `%.3f` on 10^5 values, ties and values up to 1e30 included.

### R8 Recoverable asserts and `ExpectAssert`

- A Zig assert cannot return. Jolt's `ExpectAssert` needs `AssertFailed` to return false, so that the code continues on
  Jolt's release path.
- `Zolt/Core/IssueReporting.zig` (prototype) ports the hook:
  - `AssertHandler` is a pattern A interface stored in an atomic (its identity is its address);
  - `setAssertHandler(handler) ?*AssertHandler` installs it and returns the previous one;
  - `assertRecoverable(ok, comptime message, @src())` and `assertFailed(comptime message, @src())` ask the handler.
    Without a handler, or when it returns true, they panic as before. When it returns false the code continues.
  - Both are `inline` and do nothing without asserts. Call sites keep their `if (Core.enable_asserts)` guard, so
    ReleaseFast does not even evaluate the condition.
- The handler pointer is the library's one new global. It is an explicit exception to "no globals":
  - it is Jolt's own global (`AssertFailed`);
  - asserts fire on job threads and in code without a system (shapes, skeletons, constraints);
  - it is `void` without asserts, so it cannot influence ReleaseFast results.
- `ExpectAssert` lives in the library, next to the hook, like `FailingTempAllocator`. Inline tests of `Zolt/` cannot
  import `ZoltTests/`. `ZoltTests/UnitTestFramework.zig` re-exports it and is complete. Usage:

```zig
var expect_assert: fw.ExpectAssert = undefined;
expect_assert.init(1); // JPH_IF_ENABLE_ASSERTS(ExpectAssert expect_assert(1);)
errdefer expect_assert.restore();
errors = try c.simulateSingleStep();
try expect_assert.finish(); // ~ExpectAssert: CHECK(sCount == 0)
```

- Only the asserts that a test expects become recoverable. They are the release-reachable asserts of the guide (the
  `if (Core.enable_asserts) std.debug.assert(...)` and `@panic(...)` rows) at these sites:

| Assert | Unlocked test |
|---|---|
| `PhysicsSystem.update`: errors == None | `TestOutOfContactConstraints` (prototype) |
| PlaneShape and MeshShape `GetSubmergedVolume` "Not supported" | their inline tests (prototype) |
| `PathConstraint.getConstraintSettings` | "TestPathConstraintFromPhysicsSystem (Jolt's release behavior)" |
| `jobIntegrateVelocity` inner radius | "PhysicsSystem linear cast body without inner radius (Jolt's release behavior)" and the scene `linear_cast_without_inner_radius` |
| SkeletonMapper "Should not be initialized yet", `Constraint.zig` step overrides | their inline tests |
| the asserts that the release only parity tests reach | the parity tests "VehicleParts: VehicleDifferentialSettings CalculateTorqueRatio outside Jolt's asserts (release only)", "Ragdoll parity: RagdollSettings (release only)" and "Skeleton parity (release only): Sample at negative times and with NaN fractions, ...", and the scene `constraints_gear_kinematic_release` |
| DebugRendererPlayback's unknown command (`JPH_ASSERT(false)` in `Parse`, settled in the audit) | "DebugRendererPlayback: an unknown command in Jolt's release build" |

- Each unlocked test runs with `ExpectAssert(n)`, where n is the number of Jolt asserts. A conversion that would continue
  into a Zig safety check stays a panic, and its test stays skipped with the reason in a comment.
- Settled in F0: the tests above reach these asserts, and each one is a single converted line (`Core.assertRecoverable`
  / `Core.assertFailed`, re-exported from `IssueReporting`): PathConstraint `GetConstraintSettings`; `jobIntegrateVelocity`
  inner radius; Constraint's two constructor and two setter step overrides; SkeletonMapper "Should not be initialized
  yet", "Skeleton 1 should be the low detail skeleton!", the two `AreJointsCorrectlyOrdered` and "Joints must be
  ordered"; SkeletonPose "Joints must be ordered"; SkeletalAnimation `inTime >= 0`, the fraction and the sampled
  rotation; Ragdoll "Failed to decompose the inertia tensor!", the base priority overflow and the joint order;
  EigenValueSymmetric "Too many iterations" (reached by the NaN inertia before the Ragdoll assert); VehicleDifferential's
  three asserts; MotionProperties' dynamic checks of `GetInverseMass`, `MultiplyWorldSpaceInverseInertiaByVector`,
  `ApplyLinearVelocityStep` and `ApplyAngularVelocityStep` (the kinematic gear and rack). None continues into a Zig safety
  check, so no test stays skipped.
- Settled in the audit: one more site. DebugRendererPlayback's unknown command (`JPH_ASSERT(false)` in `Parse`) is a
  recoverable assert. After it the parse reads the next byte as a command, like Jolt's release build. The test
  "DebugRendererPlayback: an unknown command in Jolt's release build" runs in every build with `ExpectAssert(1)`. No
  test of R8 stays skipped.
- The parity tests of Jolt's release behavior count the asserts of both sides (`ZoltParity/ParityHooks.zig`
  `ReleaseAsserts`, Zolt's `IssueReporting.CountAsserts`). With `-Dreference_asserts=true` every case and every run of
  a release_only scene hits as many asserts on both sides. Without it they check that the cases reach an assert.
- The two `release_only` scenes run in safe builds too. SimulationParity installs a counting handler. With
  `-Dreference_asserts=true` it compares Zolt's count with `jolt_sim_reference_assert_count`.
- All other asserts stay. Invariant asserts (`std.debug.assert`) guard Zig undefined behavior: there is no release path
  to continue on. New Phase 9 code follows the same rule: an assert becomes recoverable only when a test expects it.

### R9 Compute interfaces

- `ComputeBuffer`, `ComputeShader`, `ComputeQueue` and `ComputeSystem` (`Zolt/Compute/`) are pattern A roots with
  `ref_count` and `allocator`, like Jolt's RefTargets with virtual functions (prototype).
  - Hair and HairShaders take a `*ComputeSystem` / `*ComputeQueue`, like Jolt. A comptime backend would make them
    generic and would leave no room for a GPU backend.
  - The cost is one indirect call per bind or dispatch, nothing per shader thread.
- `ComputeSystem` declares `rtti` (`.abstract`, Jolt's `JPH_DECLARE_RTTI_ABSTRACT_BASE`). Its vtable has the `rtti` data
  entry, so `RTTI.isKindOfObject` works (ComputeTests checks it). Neither class is in a Factory, like Jolt.
- Jolt's `Result<Ref<T>>` returns are error unions (porting guide section 5):
  - `createComputeShader(name, group_size_x, .{ .group_size_y, .group_size_z }) CreateShaderError!Ref(ComputeShader)`,
    with `CreateShaderError = Allocator.Error || error{ShaderNotFound}` ("Compute shader not found");
  - `createComputeBuffer(type, size, stride, data: ?[]const u8) Allocator.Error!Ref(ComputeBuffer)`;
  - `createComputeQueue() Allocator.Error!Ref(ComputeQueue)`;
  - `createComputeSystem(allocator) CreateSystemError!Ref(ComputeSystem)` is always `error.NotImplemented`
    ("Not implemented"): Zolt has no GPU backend.
  - The returned `Ref` holds the caller's reference.
- The queue's bind functions return `Allocator.Error!void`: the CPU queue allocates the wrapper and the entry that keeps
  a bound buffer alive. `scheduleReadback`, `execute` and `wait` return `void`, like Jolt.
- `ComputeBuffer.map(mode)` returns a slice of `size * stride` bytes. `mapAs(T, mode)` is `Map<T>`. `is_mapped` exists
  only with asserts.
- `mShaderLoader` is not ported: only the GPU backends call it.

### R10 The CPU backend

- `ComputeSystemCPU`, `ComputeBufferCPU`, `ComputeQueueCPU` and `ComputeShaderCPU` (`Zolt/Compute/CPU/`) are literal.
- `ComputeSystemCPU.create(allocator) Allocator.Error!Ref(ComputeSystem)` is `CreateComputeSystemCPU()`. It declares
  `rtti` (`.virtual`, base ComputeSystem) and `createDefault`.
- The shader registry is per instance: `shader_registry: std.StringHashMapUnmanaged(ShaderWrapper.Create)`. It is only
  searched, so its order does not matter. `registerShader(comptime Shader)` is `JPH_REGISTER_SHADER(sys, X)`. A second
  registration overwrites the first, like Jolt. There is no global registry.
- `createComputeShader` looks the name up. The group sizes come from the caller, not from the shader, like Jolt.
- `ComputeBufferCPU` is one 16-byte aligned allocation of `size * stride` bytes. Zolt zero-fills it (UB4). A
  `size * stride` that does not fit in a `usize` is `error.OutOfMemory` (UB28). `createReadBackBuffer` returns the
  buffer itself with a new reference.
- `ComputeQueueCPU`:
  - `setShader` asserts that no shader is pending and creates the shader's wrapper;
  - a bind of a null buffer is ignored; the buffer types are asserted (`ConstantBuffer` for `setConstantBuffer`; Upload,
    Buffer or RW for `setBuffer`; RW for `setRWBuffer`);
  - a bound buffer gets a reference in `used_buffers` until the dispatch ends;
  - `dispatch` runs synchronously on the calling thread, for z, then y, then x (x fastest), with the global thread ID.
    Then it destroys the wrapper, releases the buffers and clears the shader;
  - `scheduleReadback`, `execute` and `wait` do nothing; barriers are ignored.
- The CPU backend is always compiled. Zig compiles lazily, and Jolt's CMake has it on by default.

### R11 The HLSL emulation

- `Zolt/Compute/CPU/HLSLToCPP.zig` ports `HLSLToCPP.h`. One Zig function per C++ operator, in its operation order.
- Types keep HLSL's names (`hlsl.float2`, `float3`, `float4`, `uint3`, `uint4`, `int3`, `int4`; `Quat` and `Plane` are
  `float4`; `Mat44` is 4 `float4` columns). A shader port is reviewed line by line against the HLSL text.
- They are extern structs of scalars, not `@Vector`. Buffer layouts must equal ShaderCore.h's (`float3` is 12 bytes).
- Semantics that differ from Zolt's `Vec3` (pinned by tests):
  - `dot` is left to right: `a.x * b.x + a.y * b.y + a.z * b.z`;
  - `normalize(v)` divides each component by `length(v)` (no reciprocal); `length` is `@sqrt(dot(v, v))`;
  - `min` / `max` are `math.min` / `math.max` per component (`std::min` / `std::max`), not Jolt's `Vec3.min`;
  - `clamp(v, lo, hi)` is `min(max(v, lo), hi)`;
  - unary minus flips the sign bit per component (not `0 - v`);
  - `round` is `@round` (half away from zero, like `std::round`); `asint` is `@bitCast`;
  - `uint(f)` is the 64-bit truncation of `cvttss2si` cut to 32 bits; `int(f)` is the 32-bit `cvttss2si` (porting guide
    section 8, rule 12); `float(uint)` is correctly rounded;
  - integer arithmetic wraps (`+%`, `*%`), like C++ unsigned arithmetic;
  - `JPH_AtomicAdd` on `int` is `@atomicRmw(.Add, .seq_cst)` plus the operand.
- Seed: the fidelity prototype's `ZoltParity/Compute/Prototype/HLSL.zig` (commit 26c2ebf). It is bit exact against
  Jolt's CPU backend on more than 10^5 inputs.

### R12 Shaders as Zig files

- One Zig file per shader in `Zolt/Shaders/` (mirrors `Jolt/Shaders`). `X.zig` ports `X.hlsl`, `XBindings.h` and, when it
  exists, the helper header `X.h` (HairIntegrate, HairUpdateVelocity, HairApplyGlobalPose,
  HairCalculateRenderPositions). Its header lists all of them in `//! Port of:`.
- Shared headers are files of their own: `ShaderCore.zig`, `ShaderMath.zig`, `ShaderVec3.zig`, `ShaderQuat.zig`,
  `ShaderMat44.zig`, `ShaderPlane.zig`, `HairCommon.zig` and `HairStructs.zig`.
- `HairStructs.zig` holds the extern structs that the shaders and Hair share, with Jolt's sizes checked at compile time
  (HairUpdateContext 256 bytes, HairMaterial 160, HairPosition 28, ...), and the `cHair*` constants.
- The shader helpers are the HLSL reimplementations (`JPH_QuatMulVec3`, `JPH_Mat44Mul3x4Vec3`, ...), never Zolt's
  `Quat` / `Mat44`: their operation order differs.
- The shader file contract (section 3 has the full recipe):
  - `pub const name = "HairGridClear"` (JPH_SHADER_NAME, the registry key);
  - the shader's constants (`cHairPerGridCellBatch` → `hair_per_grid_cell_batch`);
  - `pub const Bindings = struct { ... }`: the fields have the HLSL binding names, the strings that Hair binds by, and
    typed wrappers: `hlsl.Constants(T)` (a constant block, copied at bind time like Jolt's `memcpy`), `hlsl.Buffer(T)`
    and `hlsl.RWBuffer(T)` (slices over the buffer memory);
  - a shader that includes `HairStructs.h` also has its two constant blocks, `gContext` and `gIterationContext` (Jolt
    declares them in `HairStructs.h`), first, then the bindings of its `Bindings.h` in order;
  - members of shader structs and constant blocks are snake_case: the HLSL `cNumGridPoints` is
    `b.gContext.value.num_grid_points`;
  - `pub fn main(b: *Bindings, tid: hlsl.uint3) void`; HLSL functions are camelCase private functions.
- `ShaderWrapper.zig` ports `ShaderWrapper.h` and the three `WrapShader*.h` macro headers. `ShaderWrapper` is a pattern A
  root with `bind` and `main`. `ShaderWrapper.Of(Shader)` generates the wrapper class: `bind` is an `inline for` over
  `Shader.Bindings` (Jolt's switch on the name hash), `main` calls `Shader.main`.
- An unknown binding name is Jolt's `JPH_ASSERT(false, "Buffer cannot be bound to this shader")`:
  `if (Core.enable_asserts) @panic(...)`, then the bind is ignored like Jolt's release build.
- A constant buffer larger than the block asserts and copies the block size (UB5).
- Buffers are slices, so a shader read out of bounds panics in safe builds where Jolt reads past the allocation (UB3).
- `HairWrapper.zig` is `registerHairShaders(system: *ComputeSystemCPU) Allocator.Error!void` (the 15 hair shaders).
  `TestComputeWrapper.zig` is `registerTestCompute` and `registerTestCompute2`.
- Constant folding: `1.0f / float(cFloatToFixed)` and the other folded constants are comptime f32 expressions (IEEE
  single rounding, bit exact in the fidelity prototype).

### R13 libm and `std::shuffle`

- Hair is the only Jolt code that calls the C library for floats: `std::exp(float)` (twice per material) and
  `std::pow(float, float)` (`Gradient::MakeStepDependent`). `HairSettings` calls `std::shuffle` with a default-seeded
  `std::mt19937`.
- `Zolt/Math/LibM.zig` (Zolt addition) ports `expf` and `powf` from the generic algorithm that ARM's optimized-routines,
  musl and glibc's generic variants share. Zig ships the sources (`lib/libc/musl/src/math/expf.c`, `powf.c`,
  `exp2f_data.c`, `powf_data.c`). Plain f64 arithmetic, no FMA.
  - The fact finding measured 0 differences from glibc's generic variants on all 2^32 `expf` inputs and on 3 × 10^8
    `powf` pairs, and 2 `expf` inputs (far outside the hair domain) where glibc's FMA variant differs.
  - Zig's `std.math` and `compiler_rt` `expf` are an older algorithm and are never used.
  - So Zolt's hair is the same on every platform. Jolt's depends on the C library.
- The reference pins both functions (R1): `ZoltParity/Hair/LibMReference.c` defines `expf` and `powf` with the same C
  sources. A probe test checks that the pin took: it compares the reference's `expf` with the generic algorithm on
  inputs where `compiler_rt`'s algorithm differs (for example `0x4202422f`). An opt-in test compares `LibM.zig` with the
  host's `libm.so.6` through `dlopen` and lists the differences of glibc's FMA variant.
- `Mt19937.shuffleLibcxx(T, items)` and `uniformIntLibcxx` (Zolt additions in `Core/Mt19937.zig`) reproduce libc++'s
  `std::shuffle` and `uniform_int_distribution`: for i in 0..n-2, draw j in [0, n-1-i] with one 64-bit engine call per
  attempt, a bit mask and rejection; swap when j != 0. Jolt's result depends on the standard library; Zolt follows
  libc++, the reference's. The file header says so.

### R14 Hair

- `HairSettings` is a non-virtual RefTarget with `allocator`, Jolt's fields and defaults, and Jolt's functions:
  `initRenderAndSimulationStrands` (it appends to the simulation arrays, literal), `resample`, `init`, `initCompute`,
  `prepareForScalpSkinning`, `skinScalpVertices`, `saveBinaryState` / `restoreBinaryState`.
  - `mSimulationBounds` is written as the raw 32-byte AABox. Zolt's `Vec3` keeps W = Z (porting guide section 8, rule 9).
    Settled in H1: Jolt's `StreamIn::Read(Vec3)` fixes W only when `JPH_FLOATING_POINT_EXCEPTIONS_ENABLED` is defined
    (MSVC Debug and Release), so after `RestoreBinaryState` and `Init` a non-MSVC Jolt writes stale W lanes there. Zolt
    writes what Jolt writes with floating point exceptions enabled. The HairSettings parity models the reference's W lanes
    (`joltBoundsW`) and compares every other byte bit for bit.
  - The C++ `GridSampler` uses `Vec3` min / max; the shader's `GridPositionToIndexAndFraction` uses `std::min` /
    `std::max`. Each keeps its own semantics.
  - The serializable classes that derive from a shader struct without adding members (`SkinWeight`, `SkinPoint`,
    `SVertexInfluence`) are the extern structs of `HairStructs.zig` with the `rtti` declaration; `HairSettings.SkinWeight`
    is an alias.
- The 10 classes get declarations with Jolt's nested names (`"HairSettings::SVertex"`); `SStrand` has the base `RStrand`;
  `RVertex.mInfluences[3]` is a fixed array.
- `RegisterHair.serializable_types = .{HairSettings}`. An application adds it to `zolt_user_types.serializable_types`.
  Jolt's `RegisterHair()` registers at run time; Zolt's factory is comptime (S8). `SkinPoint` is not reachable, like in
  Jolt.
- `HairShaders` is a RefTarget with 15 shader references and `init(system) Allocator.Error!void`. A missing shader stays
  null, like Jolt.
- `Hair` is a plain struct (Jolt: NonCopyable, not a RefTarget): `init(allocator, settings, position, rotation, layer)`,
  `deinit`, `initCompute(system)` (Jolt's `Init`), `update(...) Allocator.Error!void`, `readBackGPUState`,
  `lockReadBackBuffers` / `unlockReadBackBuffers`, the override setters, `draw` (R2).
  - It runs on the caller's thread outside `PhysicsSystem.update`, like CharacterVirtual (CharacterVehicleArchitecture
    E5). It reads bodies through the locking interfaces and never writes them. Never update it from a step listener.
  - Jolt's unbounded `JPH_STACK_ALLOC` arrays use the Hair's allocator and are freed before returning (guide section 5).
  - Collision data comes from ConvexHullShape leaves only, like Jolt.
  - The render position override (`std::function`) is a `{ context, function }` callback.
- Hair has no SaveState and no job: it has no D5 rows.

### R15 Soft body types and layouts

| Jolt | Zolt |
|---|---|
| `SoftBodyVertex` | `extern struct` in Jolt's field order with `comptime @sizeOf == 80` (the iterator strides over it); `resetCollision`, `markCCDContact(body_id, plane)` (`colliding_shape_index = @bitCast(id | BodyID.broad_phase_bit)`, negative) |
| `CollideSoftBodyVertexIterator(SoftBodyVertex *)` | `initFromVertices([*]SoftBodyVertex)`: the 5 strided pointers with stride `@sizeOf(SoftBodyVertex)` |
| `SoftBodySharedSettings` (RefTarget, no virtual functions) | non-virtual RefTarget like PhysicsScene: `ref_count`, `allocator`, `create(allocator)` (fallible: `materials` starts with one element, `PhysicsMaterial.default`), `addRef` / `release`, `clone`, `deinit`. Private members (`closest_kinematic`, `update_groups`, `skinned_constraint_normals`) are fields with the doc comment "private in Jolt" |
| its nested structs that `SaveBinaryState` writes raw: Vertex, Face, Edge, DihedralBend, Volume, Skinned (with `normal_info`), LRA, UpdateGroup | `extern struct`s with Jolt's defaults and size checks: 28, 16, 16, 24, 24, 52, 12 and 28 bytes; `Skinned.weights: [4]SkinWeight` |
| InvBind, RodStretchShear, RodBendTwist (written per field: 68, 36, 28 bytes), ClosestKinematic, VertexAttributes, OptimizationResults | plain structs; `BendType`, `LRAType` are `enum(i32)` |
| `SoftBodyMotionProperties : MotionProperties` | `base: MotionProperties` first (Phase 5 D1), `fromBase` / `fromBaseConst`, then Jolt's fields in Jolt's order, plus `allocator` (the BodyManager's). Arrays are `std.ArrayList`. `deinit` frees them and releases `settings` and the leaf shapes |
| `RodState { Quat mRotation; union { Vec3 mAngularVelocity; Quat mPreviousRotationInternal; } }` | `extern struct { rotation: Quat, u: extern union { angular_velocity: Vec3, previous_rotation_internal: Quat } }`: the storage reuse of IntegratePositions is defined for an extern union |
| `mSkinStateTransform` (only with JPH_DEBUG_RENDERER) | `skin_state_transform: if (Core.debug_renderer) RMat44 else void`; its two writes during simulation are `if (Core.debug_renderer)` statements |
| `atomic<bool> mNeedContactCallback`; `mNumSensors` (Jolt's TSAN copy) | `std.atomic.Value(bool)` and `std.atomic.Value(u32)`, Jolt's relaxed order is `.monotonic` |
| `SoftBodyManifold` | `{ vertices: []const SoftBodyVertex, colliding_shapes: []const CollidingShape, colliding_sensors: []const CollidingSensor }`, built only by `updateSoftBodyState`, with Jolt's accessors (the CCD decode of `getContactBodyID`) |
| `SoftBodyShape` | final pattern A shape: `base: Shape`, `motion_properties: *const SoftBodyMotionProperties`; `register` fills the dispatch table like `sRegister` (convex and sphere vs soft body, both directions, no construct function, DarkGreen). Soft body vs soft body, mesh, height field and compound stays unregistered |
| `SoftBodyUpdateContext`, `SoftBodyContactListener` | complete. Contexts live in a `TempArray`, so nothing in them may be aligned above `rvector_alignment` (JPH_RVECTOR_ALIGNMENT: 16, or 32 in double precision), the alignment of TempAllocator blocks. A comptime assert checks it |

- Mutable access follows the rename table: `getVertices` / `getVerticesMut`, `getVertex` / `getVertexMut`.
- The delivery prototype parsed Jolt's `SaveBinaryState` bytes with exactly these layouts (`ZoltParity/Physics/
  SoftBodyParity.zig`, commit 3759ab6).

### R16 Soft body allocation and lifetime

- `BodyManager.allocateSoftBody(settings) CreateBodyError!*Body` is one allocation, like Jolt's
  `new SoftBodyWithMotionPropertiesAndShape`:
  1. `allocator.create(SoftBodyWithMotionPropertiesAndShape)` (the struct exists).
  2. `shape.motion_properties = &motion_properties`, then `shape.base.setEmbedded()`.
  3. `body.body_type = .soft_body`, `body.motion_properties = &motion_properties.base`, `body.shape = .init(&shape.base)`.
  4. `body.applySoftBodyCreationSettings(...)` with `errdefer self.deleteBody(body)`.
- `Body.applySoftBodyCreationSettings` becomes `Allocator.Error!void`: `SoftBodyMotionProperties.initialize` allocates
  the vertices, rod states and skin states. A second call resizes and never leaks.
- `BodyManager.deleteBody`, soft body branch: release the body's shape reference, then `motion_properties.deinit()`
  (Jolt's `~SoftBodyMotionProperties`, missing in Phase 5), assert that the embedded shape is unreferenced, release the
  collision group, destroy the object. A leak test covers it.
- `BodyInterface.createSoftBody`, `createSoftBodyWithID`, `createSoftBodyWithoutID` and `createAndAddSoftBody` mirror
  the four `createBody*` functions and their error conventions (D6).
- `SoftBodyMotionProperties.allocator` grows the colliding shape and leaf arrays inside the SoftBodyCollide job. That is
  a heap allocation inside a job, latched (R17).

### R17 Soft body jobs and their D5 rows

- `JobSoftBodyPrepare`, `JobSoftBodyCollide`, `JobSoftBodySimulate` and `JobSoftBodyFinalize` are literal ports at the
  lambda positions (D4):
  - Prepare sorts the active soft bodies by BodyID with Zolt's `QuickSort`;
  - the contexts live in `PhysicsUpdateContext.soft_body_update_contexts` (a `TempArray`);
  - Prepare creates Finalize, then N Simulate jobs, then N Collide jobs (N = `getMaxConcurrency()`); Collide jobs start
    at creation;
  - Simulate calls `std.Thread.yield() catch {}` after a pass without work;
  - the atomics use Jolt's memory orders (relaxed is `.monotonic`), `fetch_add(64)` per batch even for a partial batch.
- Why results do not depend on the thread count: vertex batches, sensors and parallel constraint groups write disjoint
  data, and every order-sensitive step is serial (the non-parallel group, collision response, pressure, integration,
  `updateSoftBodyState`, Finalize in sorted BodyID order). The delivery prototype ran three C++ scenes with 0, 1, 3 and
  7 workers: identical `SaveState(All)` bytes and sorted listener records after every update.
- The existing D5 contract stays: an allocation failure is latched and `update` returns `error.OutOfMemory`; a job that
  cannot be created from inside a job is not an error and changes no result. The new rows (S5 adds them to
  SimulationArchitecture D5):

| Event | Handling |
|---|---|
| Prepare cannot copy the active bodies or allocate the contexts | latch, take the "no active soft bodies" path (kick StartNextStep). Error (allocation) |
| Finalize cannot be created (it is the first) | Prepare runs the whole update inline: the collide claim loop, the simulate loop (`parallelUpdate` over all contexts until every one is Done), the finalize body, each under the grants of the job it replaces. Then it kicks StartNextStep. Not an error |
| Simulate job k cannot be created | remove one Finalize dependency for it. Its handle stays invalid; Collide jobs skip invalid handles. Not an error while one Simulate job exists: each one loops until every context is Done |
| no Simulate job could be created | create **no** Collide job. Prepare runs the collide claim loop, then the simulate loop, inline, then removes the N Finalize dependencies. Not an error. (A Collide job that started could still be filling `colliding_shapes` while an inline simulate reads it: a race) |
| Collide job k cannot be created | after the creations, Prepare runs the collide claim loop itself under the Collide grant (Read, Read), until `soft_body_to_collide` runs out. Then it removes the dependencies of the missing Collide jobs from every Simulate job. Not an error |
| `determineCollidingShapes` cannot allocate (in a Collide job or in Prepare's inline loop) | latch and stop claiming; partial colliding lists are fine. Error (allocation) |
| Simulate (job or inline loop) | checks the latch at entry only, never inside `parallelUpdate`: other Simulate jobs wait for claimed batches. A poisoned Simulate only removes its Finalize dependency. Simulate never allocates |
| Finalize when poisoned | skips `updateRigidBodyVelocities`, the position update and the bounds and sleep batches (the contexts may not be Done). Always clears every context's colliding shapes and sensors (shape references) and frees the contexts at Jolt's position (LIFO) |

- The FailingJobSystem sweep of every soft body scene needs no special case: every failed creation must give the
  reference bytes. A FailingHeapAllocator test over a soft body scene (PhysicsSystemTests) covers the heap allocation.
- `CustomUpdate` is literal: a single-threaded state machine with the locking interfaces. A body that was created but
  never added has an ID, so its lock succeeds and `SetPosition` moves it. Only the broad phase notify and the activation
  need a body in the broad phase. A body without an ID (`createSoftBodyWithoutID`) does not move. The test
  "SoftBodyMotionProperties: customUpdate moves a soft body that was never added, not one without an ID" pins it.

### R18 CCD against a soft body face

- The `@panic` in `jobResolveCCDContacts` becomes the literal port of `PhysicsSystem.cpp:2283-2341`:
  - the face from `SoftBodyShape.getFaceIndex` and the motion properties' `getFace`;
  - vertex masses `1 / inv_mass`, or `1e10` for a kinematic vertex;
  - the barycentric velocity, the impulse on body 1 and on the 3 vertices through `getVertexMut` (the resolve job is
    serial and `body2.motion_properties` is a mutable pointer, so Rule M holds);
  - with a soft body contact listener only: `markCCDContact` on the 3 vertices and `requestContactCallback`.

### R19 Soft body schemas, binary state and PhysicsScene

- The 13 classes get `rtti` declarations with Jolt's names (`"SoftBodySharedSettings::Vertex"`), all `.non_virtual`.
  - `Face.mVertex[3]`, `Edge.mVertex[2]` and `Skinned.mWeights[4]` are fixed arrays.
  - `SoftBodyCreationSettings.mObjectLayer` is an enum attribute, like BodyCreationSettings.
  - `mNormalInfo`, `mUpdateGroups`, `mSkinnedConstraintNormals` and `mClosestKinematic` are not serialized.
  - `mSettings` is the first `Ref` to a non-virtual RefTarget. The ReferenceCodec is generic over RTTI; it needs
    `createDefault`, `addRef` and `release`, which R15 gives. S0 adds a codec test.
- `SoftBodyCreationSettings` is the 49th `RegisterTypes.factory_types` entry (Jolt's `types[]` order).
- The S17 exclusions are gone: `phase9_classes` (the 13 classes), `gen_serializable.py` `EXCLUDED_DIRS` (SoftBody),
  `port_status.py` `PHASE9_SERIALIZABLE` (SoftBody) and the RegisterTypes test of the boundary (S0). S2 removed the
  PhysicsScene cross-read, `EXCLUDED_ATTRIBUTES` and the `pickCount` special case.
- Binary state is statement by statement Jolt (S11): `saveBinaryState`, `restoreBinaryState`, `saveWithMaterials`,
  `restoreWithMaterials` (an id at or above the map size creates new settings, no equality check), `saveWithChildren`,
  `restoreWithChildren`. Jolt's texts are `Result` values ("Failed to read settings id", "Error reading body creation
  settings").
- `PhysicsScene` gets `soft_bodies: std.ArrayList(SoftBodyCreationSettings)` as its third attribute, `addSoftBody`,
  `getNumSoftBodies`, `getSoftBodies` / `getSoftBodiesMut`, and Jolt's soft body parts of CreateBodies, SaveBinaryState
  (soft bodies always save their shared settings and materials), sRestoreFromBinaryState and FromPhysicsSystem (with its
  index quirk).
- A soft body's `SaveState` / `RestoreState` (StateRecorder) is Jolt's; `Body.saveState` already dispatches.

### R20 Jolt's quirks and Zolt's defined behavior where Jolt is undefined

Jolt's quirks where Jolt's behavior is defined are reproduced:
- `CreateConstraints` adds the last shared edge of a closed mesh twice (7 edges for a tetrahedron surface, 13 with
  distance bends; both prototypes confirmed it on Jolt);
- `Optimize` uses a `mClosestKinematic` that `CreateConstraints` cached, even when constraints were added after;
- the tie order of Jolt's `QuickSort` in the edge sort and the group size sort, and Dijkstra's heap ties
  (`BinaryHeap`); `std.sort` gave 30087 mismatches in the fidelity prototype;
- `FromPhysicsSystem` stores a soft body's index of the wrong array;
- `HairSettings.initRenderAndSimulationStrands` appends to the simulation arrays;
- `mScalpJointMatricesCB` is 64 times too large (memory only);
- Jolt's density loops of `Hair::Draw` bound z by `GetX()` and x by `GetZ()`. This is defined only when
  `GetX() <= GetZ()`. When `GetX() < GetZ()`, x passes `GetX()`: `GetNeutralDensity` asserts in Jolt's debug build
  (recoverable, R8), and the release build reads another cell of the grid (UB24 is the case `GetX() > GetZ()`);
- the vehicle per-wheel texts (R4 notes the renderer);
- the recorder's stale batch IDs across recorders (the shape caches outlive a recorder in both).

Where Jolt is undefined, Zolt has one defined behavior. The parity generators never produce these inputs. Zolt-only
tests pin them.

| # | Input | Jolt | Zolt |
|---|---|---|---|
| UB1 | a soft body with 0 vertices | the state machine never leaves DetermineCollisionPlanes: JobSoftBodySimulate spins forever | the first claimant of the empty vertex range advances the state, as the last batch would. The body simulates nothing |
| UB2 | shared settings without update groups (read from an ObjectStream, `optimize` not called) | an assert, then `size() - 1` underflows and ProcessGroup reads out of bounds | `initialize` panics: "SoftBodySharedSettings: call optimize() before creating a soft body" |
| UB3 | collision enabled, leaf shapes in bounds, no convex hull | `mCollisionShapesCB` has 0 elements and HairUpdateVelocity reads element 0 | the buffer has `max(n, 1)` zeroed elements, like the planes and vertices buffers |
| UB4 | padding vertices of short strands (`InitCompute`'s `resize`), RW compute buffers, `ComputeBufferCPU` without data | uninitialized memory, read by HairGridAccumulate | zero (the reference runs with zero-filling hooks too) |
| UB5 | a constant buffer larger than the shader's block | overwrites the members after the block | asserts, copies the block size |
| UB6 | `SaveWithChildren` with a null `mSettings` and both maps | null dereference | panics with a message |
| UB7 | playback of a batch or geometry ID that the stream does not define, or of a stream that ends inside a command | `find()->second` of `end()`; uninitialized locals, and frame elements that stay uninitialized | an unknown batch ID is `error.InvalidStream`, except ID 0, which is an empty batch; `drawFrame` skips an unknown geometry ID (Jolt looks it up only there); a cut stream is `error.InvalidStream` and keeps every frame Jolt keeps, each with the elements read in full |
| UB8 | on any material run, `mSimulationStrandsFraction` selects ceil(fraction * run size) strands, and that is NaN, negative or more than the strands left from the start of the run (settled in the audit) | reads past the index array | panics with "HairSettings: mSimulationStrandsFraction selects more strands than there are". A run whose simulated strands stay inside the index array is literal: they can run into the next material |
| UB9 | an allocation failure in a draw | abort | the primitive is dropped and latched (R5) |
| UB10 | two threads draw the same uncached shape | a data race on the cache | a mutex: one cached geometry (R6) |
| UB11 | `uint3(float3)` of NaN or of a value outside the `uint32` range (settled in C0) | the result depends on clang's vectorization at the call site: three scalar 64-bit conversions in HairGridAccumulate, a `cvttps2dq` pair for x and y in the `ApplyGrid` of HairIntegrate and HairUpdateVelocityIntegrate | the scalar conversion of porting guide rule 12 for every component (`uint3.fromFloat3`), pinned by a Zolt-only test. The dispatch parity and the hair scenes never produce such grid positions: no zero-size bounds axis without padding, no NaN positions, no diverging scenes |
| UB12 | `createConstraints` with an LRA type, or `optimize`, after vertices were added or removed while the closest kinematic cache was filled (settled in S1) | reads `mClosestKinematic` or `mVertices` out of bounds | panics with "SoftBodySharedSettings: vertices were added or removed after the closest kinematic vertices were cached" when the cache is not empty and its size is not `vertices.len`, at the LRA block of `createConstraints` and before the sorts of `optimize` (S2 implements it) |
| UB13 | `createConstraints` with an empty `vertex_attributes` slice and at least one face (settled in S1) | `inVertexAttributes[min(v, 0u - 1)]` reads out of bounds | panics with "SoftBodySharedSettings.createConstraints: no vertex attributes" (S2 implements it) |
| UB14 | `optimize` with a vertex that no group reaches (a NaN, infinite or `FLT_MAX` position; settled in S1) | a constraint with no grouped vertex, or a skinned constraint on an ungrouped vertex, reads `groups[-1]`; a constraint with some grouped vertices goes to the non-parallel group | the mixed case stays on Jolt's release path (`if (Core.enable_asserts)` asserts); a constraint without a grouped vertex and a skinned constraint on an ungrouped vertex panic with a message (S2 implements it) |
| UB15 | `DrawPie` with a NaN or infinite angle range, or one that needs more than `maxInt(i32) / 3` parts (settled in D1) | converts NaN or an out-of-range float to `int`, then `JPH_STACK_ALLOC` with a negative or huge size | draws nothing; the empty cache entry stays, as after Jolt's early returns. Pinned by a Zolt-only test |
| UB16 | HairUpdateStrands with a vertex in the middle of a strand whose strand fraction byte is 255 (settled in C1b). Hair gives it when the last segment is too short to change the float sum of the segment lengths (for example zero) | `mBendComplianceMultiplier[idx + 1]` with idx 3 reads past the float4 (HLSLToCPP's `(&x)[inIndex]`). The measured build reads the next member, `mGridVelocityFactor.x` | reads `mGridVelocityFactor.x`, pinned by the HairUpdateStrands inline test, never generated by the parity. No panic: a step gradient (equal min and max fractions) makes `mGridVelocityFactor.x` infinite or NaN, so valid Hair data reaches this read. Both sides then skip the bend and twist constraint of that vertex |
| UB17 | a null entry in a MeshShape's material list, drawn with material colors and without triangle groups (settled in D2) | `MeshShape.cpp:607` calls `GetDebugColor` on null | the color of `PhysicsMaterial.default`, like the `getMaterial` rule; pinned by a DebugRendererZoltTests test |
| UB18 | `HeightFieldShape.clone` while another thread draws the same height field (settled in D2) | `Clone` copies `mGeometry` while `Draw` appends to it: a data race | the same race: `clone` has no `io` (porting guide section 9), so it copies the list of shared geometries without the cache lock. Caller rule, as in Jolt: do not clone a height field while another thread draws it |
| UB19 | `HairSettings.init` with a simulated strand of 0 vertices, or of 1 vertex at vertex 0 or with no longer strand before it in `mSimStrands` order; `resample` with a strand of 0 or 1 vertices at the end (or 0 at the start) of the vertex array (settled in H1) | reads past the vertex array or wraps an index | panics with a message. A strand of 1 vertex after a longer strand follows Jolt's release path: it reads the kept `x[1]` after a recoverable assert (R8), pinned by an inline test with `ExpectAssert(2)` and a parity case against the asserting reference |
| UB20 | `HairSettings.init` with a scalp when the settings already have more skin points than simulated strands (a second `init`, or `init` after restoring an initialized groom; settled in H1) | indexes `mSimStrands` out of bounds | panics with "HairSettings.init: the settings have more skin points than simulated strands (init called twice with a scalp?)" |
| UB21 | `Hair.update` with a HairShaders that misses a shader (`HairShaders.init` on a compute system without `registerHairShaders`; settled in H2) | `HairShaders::Init` stores null, and `ComputeQueueCPU::SetShader` calls `CreateWrapper()` on it: a null dereference | panics with "Hair.update: a hair shader is missing (register the shaders with registerHairShaders before HairShaders.init)". Not pinned by a test (a Zig test cannot catch a panic) |
| UB22 | IndependentAxisConstraintPart with a static (or fixed to world) body, e.g. a PulleyConstraint (settled in D3) | `CalculateConstraintProperties` writes `mR1xN1` / `mInvI1_R1xN1` only for a non-static body 1 and `mRatioR2xN2` / `mInvI2_RatioR2xN2` only for a non-static body 2; `SolveVelocityConstraint` multiplies the unset members by the static body's zero angular velocity, so a NaN or an infinity in that memory gives a NaN velocity | the four members start at zero, Jolt's result on zero-filled memory. Pinned by an inline test. Like UB4, the parity generates this input with a zero-filling C++ side (the constraint reference now installs the ParityHooks.h hooks) |
| UB23 | a PathConstraint drawn before its first `SetupVelocityConstraint` (a new constraint, or one that is never active; settled in D3) | `DrawConstraint` reads `mR1`, `mR2` and the three path axes that only `CalculateConstraintProperties` writes | they start at zero: the anchors are drawn at the centers of mass and the arrows have zero length. Pinned by a DebugRendererZoltTests test; the draw parity steps every scenario before drawing it |
| UB24 | a density draw of `Hair.draw` (neutral density, grid density or grid velocity) with GridSize `GetX() > GetZ()` (settled in H3) | the quirk lets z run to `GetX()`: the planes z >= `GetZ()` read past `mNeutralDensity` and past the read back grid | the z loop stops at `GetZ()`. Pinned by the Hair.zig density draw test (8 x 6 x 4 grid). The hair draw parity draws no density when `GetX() > GetZ()` |
| UB25 | `Hair.draw` before the first `readBackGPUState` (or before `initCompute`; settled in H3) | `LockReadBackBuffers` calls Map on the null `mVelocityAndDensityReadBackCB` | draws nothing. Pinned by a Hair.zig test. The parity draws only after an update and its read back |
| UB26 | `Hair.draw` with `draw_skin_points` when the hair has skin points but no skinned scalp vertices (no scalp vertices buffer at the first read back, for example settings without an inverse bind pose; settled in H3) | `Draw` reads the null `mScalpVertices` for every selected skin point | draws no skin point: Jolt's result when the strand range selects none. Pinned by a Hair.zig test. The parity masks the skin points then |
| UB27 | `SkeletonPose.draw` on a pose without a skeleton (settled in D4) | dereferences the null `mSkeleton` | panics with a message, like SkeletonPose's other functions that need the skeleton |
| UB28 | `ComputeBufferCPU` whose `size * stride` bytes do not fit in a `usize` (settled in the audit) | `size_t(mSize) * mStride` wraps: it allocates and copies the wrapped number of bytes, and later accesses to the `mSize` elements read or write past the allocation | `create` returns `error.OutOfMemory`, like a failed allocation. Pinned by the ComputeBufferCPU inline test |
| UB29 | `SoftBodySharedSettings.createCube` with a grid size of 0 (settled in the audit) | `inGridSize - 1` wraps, and the volume constraint loops run about 2^96 times until memory runs out | panics with "SoftBodySharedSettings.createCube: the grid size is 0". Not pinned by a test (a Zig test cannot catch a panic). A grid size of 1 is literal (one vertex, no constraints), pinned by an inline test |
| UB30 | a vertex samples a hair grid whose `GridSize` is below 2 on X, Y or Z (settled in the audit). `HairSettings.init` samples each simulated vertex with `mInvMass > 0`. `Hair.update` runs the grid shaders when a material `NeedsGrid()`, the update has iterations and is not global pose only. They sample each vertex that is not the first of its strand and that InitCompute does not mark fixed: an inv mass above 0 (or NaN), or a padding vertex of a strand shorter than `mMaxVerticesPerStrand` | `GridSampler` subtracts 2 from the size, which wraps, and every sample indexes past the grid (`mNeutralDensity` in Init, the grid buffers in the shaders) | panics before the first such sample: "HairSettings.init: a vertex samples a grid that is smaller than 2 on X, Y or Z", "Hair.update: the grid shaders sample a grid that is smaller than 2 on X, Y or Z". Without such a sample a small grid is literal: `init`, both `initCompute`, `update`, the read back and `draw` work (a groom whose vertices are all fixed, or no simulated strands). Pinned by inline tests: the smallest grid 2 x 2 x 2, all-fixed grooms with the grids 0 x 0 x 0, 1 x 1 x 1, 1 x 2 x 2 and 2 x 2 x 0, and `HairSettings.gridShadersSampleAVertex`. The panics are not pinned |
| UB31 | `HairSettings.initRenderAndSimulationStrands` with a simulated strand of 0 vertices that makes the closest strand search read past `mSimVertices` (settled in the audit). This happens when the strand is at the end of `mSimVertices`, or when it starts at vertex 0: its last vertex index, `mEndVtx - 1`, then wraps, and the walk along the strand can pass the end. Its material must also have a strand that is not simulated | the closest strand search reads `mSimVertices` past its end | panics at the read past the array: "HairSettings.initRenderAndSimulationStrands: a simulated strand of 0 vertices reads past mSimVertices". The reads inside the array stay literal (`init` then panics for the strand, UB19). Not pinned by a test |
| UB32 | `Hair.draw` with `draw_render_strands` and a render strand of 0 vertices at the end of the vertex array (a strand that is not simulated; settled in the audit) | reads the first vertex of the strand past `mRenderVertices` and past the render positions | skips the strand: it draws nothing and does not change the color of the next strand. Inside the array such a strand is literal (it reads the first vertex of the next strand and draws no line). Pinned by a Hair.zig test |
| UB33 | a Hair or HairSettings call before its prerequisite (settled in the audit): `Hair.update` before `initCompute`; `Hair.update` with render vertices and an override of the render positions without `setRenderPositionsCB`; the first `readBackGPUState` before `initCompute`, or with an override without `setRenderPositionsCB`; `lockReadBackBuffers` or `unlockReadBackBuffers` before the first `readBackGPUState`; `HairSettings.initCompute` before `init` | dereferences a null buffer; `InitCompute` writes past the transposed vertex arrays (`mMaxVerticesPerStrand` is 0) | panics with a message: "Hair.update: call initCompute first" (also after an `initCompute` that failed before its last buffer), "Hair.update: the render positions are overridden, call setRenderPositionsCB first", the same two messages for "Hair.readBackGPUState: ...", "Hair.lockReadBackBuffers: call readBackGPUState first" (and "Hair.unlockReadBackBuffers: ..."), "HairSettings.initCompute: a strand has more vertices than max_vertices_per_strand (call init first)". Two nearby cases are defined in Jolt and literal in Zolt. One is `update` of an override without render vertices and without `setRenderPositionsCB`: Jolt binds the null buffer, which ComputeQueueCPU::SetRWBuffer ignores, and dispatches no render vertex. The other is a later `readBackGPUState` after `setRenderPositionsCB(null)`: the CPU backend's ScheduleReadback does nothing. A Hair.zig test pins both. The panics are not pinned. UB25 stays: `draw` before the first read back draws nothing |
| UB34 | a geometry without LODs, or a LOD with an empty `Batch` (playback ID 0, `Geometry.createWithBatch(allocator, .empty, ...)`, an allocation failure), drawn into DebugRendererSimple or DebugRendererRecorder; `Geometry.getLOD` of a geometry without LODs (settled in the audit) | DebugRendererSimple reads `mLODs.data()` and the null batch; the recorder reads the ID of the null batch; `mLODs.back()` of an empty list | DebugRendererSimple draws nothing; the recorder writes batch ID 0, which the playback reads as an empty batch (UB7), and 0 LODs for a geometry without LODs (Jolt's defined case); `getLOD` panics with "DebugRenderer.Geometry.getLOD: the geometry has no LODs". Pinned by the DebugRendererSimple and DebugRendererRecorder inline tests |

### R21 Port status and scope

- `tools/port_status.py` (done in the prototype commit):
  - `DEFERRED` keeps only `Jolt/Compute/DX12`, `Jolt/Compute/VK`, `Jolt/Compute/MTL` and `Jolt/Core/Profiler`;
  - `.hlsl` is a tracked extension. A shader `X.hlsl` is its own unit next to `XBindings.h`; one Zig file ports both.
    The 17 shaders count with their 766 lines;
  - `Core/IssueReporting` is no longer n/a: `Zolt/Core/IssueReporting.zig` ports it.
- The Roadmap's deferred section lists the GPU backends and the profiler, and the application code that is out of
  scope.
- Stays deferred:
  - the GPU backends and the shader loader: they need platform GPU APIs, and GPU results are not bit exact with the CPU
    path; the pattern A interfaces leave room for them;
  - `Core/Profiler` (8 `TODO(profiler)` markers): developer tooling, compiled out in the Distribution configuration;
  - Jolt's developer debug switches (`JPH_GJK_DEBUG`, `JPH_DEBUG_HEIGHT_FIELD`, ...): "Not ported" header lines.

### R22 The remaining unit tests and M10

| Unit | Port | Group |
|---|---|---|
| `Geometry/GJKTests.cpp` | TestGJKRaySphereShape, TestGJKRayBoxShape, TestGJKRayCapsuleShape, TestGJKRayCylinderShape: literal, everything exists (`testRay`, `getSupportFunction(.include_convex_radius, ...)`, `rayCapsule`, `rayCylinder`); remove the stale header comment | F0 |
| `UnitTestFramework` | `ExpectAssert` (R8) | prototype |
| `Physics/PhysicsTests.cpp` | the ExpectAssert part of TestOutOfContactConstraints | prototype |
| `Compute/ComputeTests.cpp` | `TestComputeCPU`: the RTTI checks, the two test shaders, `RunTests` (the failing shader name, 10 dispatches of TestCompute with the null optional buffer from iteration 2 and the references released after binding, TestCompute2's products equal to Zolt's `Mat44` exactly). The shader loader lambda is a comment. TestComputeDX12 / VK / MTL are listed in `//! Not ported:` (GPU only) and do not affect the status | C0 |
| `LoggingSoftBodyContactListener.h` | the listener (`zolt.Mutex` with `std.testing.io`, `UnorderedSet`; the test only uses `contains`) | S5 |
| `Physics/SoftBodyTests.cpp` | TestBendConstraint, TestApplyForce, TestLinearCastSphereVsSoftBody, values verbatim; the six systems share one `RefConst` cube | S5 |
| `PhysicsTestContext::RecordDebugOutput` | `recordDebugOutput(path) !void` with the option; `simulateSingleStep` draws bodies and constraints and ends the frame | D3 |

- `python3 tools/port_status.py --check-m10` (F0) maps every `TEST_CASE` of `UnitTests/**/*.cpp` to the Zig file of its
  unit and fails unless a `test "<name>"` exists there. Exceptions: the `2` suffix of duplicate names, and the names in
  the file's `//! Not ported:` line. It also fails unless every `UnitTests/` unit is complete or n/a.
- Since the audit, `python3 tools/port_status.py --check` runs the `--check-m10` rules too. It also fails on any
  `TODO(debug_renderer)` in Zolt/, ZoltTests/, ZoltParity/, ZoltPerformanceTest/, ZoltHelloWorld/ and build.zig (`.zig`,
  `.cpp`, `.c` and `.h` files).
- CI runs both. The lint job runs `--check` and an enforced "Check milestone M10" step. The step repeats the M10 rules
  under its own name, so an M10 gap shows as a failed step of its own. The test matrix has a row for M10's
  `-Duse_llvm=false` (ubuntu, Debug). Every include row of the matrix sets every key.
- The omissions inside ported cases stay, with their reasons: the profiler layer names, Jolt's read of an
  uninitialized hit in CollideShapeTests, `data() == nullptr` of STLLocalAllocator, EPA_TESTS_TRACE.

## 3. Recipes

**A Draw site** (in the tree: `BoxShape.zig`, `Constraint.zig`):

```zig
pub const overrides = .{ .getLocalBounds, ..., .draw, ... }; // listed in both configurations

/// Size of constraint when drawing it through the debug renderer (a debug-only member)
draw_constraint_size: if (Core.debug_renderer) f32 else void = if (Core.debug_renderer) 1.0 else {},

// See Shape::Draw (JPH_DEBUG_RENDERER)
pub const draw = if (Core.debug_renderer) drawImpl else {};
fn drawImpl(self: *const BoxShape, renderer: *DebugRenderer, center_of_mass_transform: RMat44, scale: Vec3, color: Color, use_material_colors: bool, draw_wireframe: bool) void {
    const draw_mode: DebugRenderer.DrawMode = if (draw_wireframe) .wireframe else .solid;
    renderer.drawBoxMatrix(center_of_mass_transform.mul(Mat44.scaleVec3(scale.abs())), self.getLocalBounds(), ...);
}

// JPH_IF_DEBUG_RENDERER(outSettings.mDrawConstraintSize = mDrawConstraintSize;)
if (Core.debug_renderer) out_settings.draw_constraint_size = self.draw_constraint_size;

// A static flag read during the simulation: the renderer of the system, then its flags (R4)
if (Core.debug_renderer) {
    if (self.debug_renderer) |renderer| {
        if (renderer.flags.physics_system.draw_motion_quality_linear_cast) {
            ...
        }
    }
}
```

Steps for every site:
1. Port Jolt's block literally under the gate of R2. Keep Jolt's draw order, colors and texts (R7).
2. Update the file header. Never leave a `TODO(debug_renderer)` line: `port_status.py --check` fails on one.
3. Add the site to the call-log parity of section 4 (one entry per draw function).
4. Run `zig build check` and `zig build check -Ddebug_renderer=true`, and the filtered parity with the option.

**A shader** (`Zolt/Shaders/HairGridClear.zig`):

```zig
//! Port of: Jolt/Shaders/HairGridClear.hlsl, Jolt/Shaders/HairGridClearBindings.h
//! Status: complete
const hlsl = @import("../Compute/CPU/HLSLToCPP.zig");
const HairStructs = @import("HairStructs.zig");

/// The name the shader is registered with (JPH_SHADER_NAME)
pub const name = "HairGridClear";

/// The constant blocks of HairStructs.h, then JPH_SHADER_BIND_BEGIN(JPH_HairGridClear): the field names are the binding
/// names of the HLSL code
pub const Bindings = struct {
    gContext: hlsl.Constants(HairStructs.HairUpdateContext) = .{},
    gIterationContext: hlsl.Constants(HairStructs.HairIterationContext) = .{},
    gVelocityAndDensity: hlsl.RWBuffer(hlsl.int4) = .{},
};

/// JPH_SHADER_FUNCTION_BEGIN(void, main, cHairPerGridCellBatch, 1, 1)
pub fn main(b: *Bindings, tid: hlsl.uint3) void {
    const index = tid.x;
    if (index >= b.gContext.value.num_grid_points)
        return;

    b.gVelocityAndDensity.at(index).* = hlsl.int4.init(0, 0, 0, 0);
}
```

Steps: port the `.hlsl`, the `Bindings.h` and the helper header in HLSL's operation order with `hlsl.*` (R11). Register
it in `HairWrapper.zig` (or the test wrapper). Add it to the dispatch parity (section 4) with valid indices, and break
one operation to see the layer fail.

**A soft body job** (the Finalize creation inside Prepare, R17):

```zig
step.soft_body_finalize = context.job_system.createJob("SoftBodyFinalize", color_soft_body_finalize, .init(struct {
    fn call(c: *PhysicsUpdateContext, s: *Step) void {
        c.physics_system.jobSoftBodyFinalize(c); // protocol job: frees the contexts in every case (R17)

        // Kick the next step
        if (s.start_next_step.isValid())
            s.start_next_step.removeDependency(.{});
    }
}.call, .{ context, step }), .{ .num_dependencies = num_soft_body_jobs }) catch {
    // Zolt (R17): not an error, Prepare does the work of the jobs it could not create
    self.softBodyUpdateInline(context, step);
    return;
}; // depends on: soft body simulate
context.barrier.addJob(&step.soft_body_finalize);
```

Steps: keep Jolt's creation order and `// depends on:` comments; check the latch only at job entry or before a claim;
add each failure row of R17 to the FailingJobSystem sweep of a soft body scene.

**A recoverable assert** (R8, in the tree: `PhysicsSystem.zig`, `PlaneShape.zig`):

```zig
if (Core.enable_asserts) IssueReporting.assertRecoverable(errors.isNone(), "An error occurred during the physics update, see EPhysicsUpdateError for more information", @src());
```

Steps: convert only an assert that a test expects; check that the code after it has no Zig safety check that Jolt's
release build passes; give the test `ExpectAssert(n)` instead of its skip.

## 4. Parity plan

Every layer runs in single and double precision, with a bite proof (break one operation, see the layer fail, revert;
the commit message says which operation).

| Layer | Group | What it compares |
|---|---|---|
| SoftBodySettings | S1 | random cloths, spheres, tetrahedra, rods and skinned meshes with every VertexAttributes, bend type and LRA type, with and without `optimize`: the SaveBinaryState bytes, the 7 remaps, `clone`, the skinned normals, `createCube` for grid sizes 2 to 8. Seeds: the fidelity CreateConstraints port and its 3000-case generator (`ZoltParity/SoftBody/*`, 26c2ebf) and the delivery layout test (`ZoltParity/Physics/SoftBodyParity.zig`, 3759ab6) |
| SoftBodyShape | S3 | PairwiseParity stops skipping `soft_body`: collide and cast convex and sphere vs soft body, single and double sided; ray (both), point, triangles, supporting face, surface normal, material, sub shape ID bits (0 faces included) |
| SoftBodyUpdate | S4 | `customUpdate` and `skinVertices` on both sides (no PhysicsSystem): the vertex, rod and skin state after each call, mass and inertia, volume |
| Soft body simulation | SH, S5 | SimulationParity (below) |
| ObjectStream | S0, S2 | P1 to P7 cover the 13 classes once the exclusions go; PhysicsScene is compared directly; P7 and P9 get soft bodies; BodyInterfaceParity compares the soft body bytes |
| Compute dispatch | C0, C1a, C1b | a generic C ABI over Jolt's ComputeSystemCPU (`jolt_compute_dispatch`: a shader by name, buffers by name, RW buffers read back): every shader on random buffers with valid indices. Float buffers compare with `sameBits`, NaNs excluded (clang may commute a product's operands); integer buffers byte for byte. Seed: the fidelity ComputeParity (26c2ebf) |
| LibM and shuffle | H1 | `expf` on a 2^20 sample in CI (all 2^32 opt-in), `powf` on [0, 1] × the hair exponents and random pairs, the pin probe (R13); shuffle sequences and engine draw counts for n ≤ 2000 |
| HairSettings | H1 | procedural grooms (straight, curly, several materials with a simulation fraction below 1) on `Assets/face.bin`'s scalp and without a scalp: the SaveBinaryState bytes after `initRenderAndSimulationStrands` and `init`, the `initCompute` buffer bytes; P1 to P7 with a RegisterHair factory |
| Hair scenes | H2 | a PhysicsSystem with kinematic hull compounds, scaled and inside-out hulls; uniform strands; dt 0, 1/60, 1/90, odd values and above `mMaxDeltaTime`; teleport, rotation, a global-pose-only material, grid on and off, LRA off. After each update: the read-back positions, velocities, grid and render positions, bit for bit. The C++ side runs with the zero-filling hooks; scenes avoid UB3 |
| Draw call log | D0 to D4, H3 | a `ParityRenderer` on both sides logs every virtual call with all arguments, including what `.jor` files drop (world space bounds, LOD scale, batch contents). One op range per group: helpers, shapes (the PairwiseParity catalogue with every argument combination and flag), constraints (the ConstraintsParity catalogue), bodies, characters, vehicles, skeleton, soft body. The hair draws run in the hair scenes instead (the ops `draw` and `record` of HairSceneParity.zig): a draw needs a full scene |
| `.jor` bytes | D3 | `zig build debug-renderer-check -Ddebug_renderer=true`: `jolt-performance-test -r -i=20` against `performance-test -r -i=20` for the 5 scenes, byte for byte (DrawBodies runs on the main thread, so any thread count works) |
| Simulation-time draws | D3 | selected SimScenes single-threaded with every flag on and the call log on both sides; the state bytes must equal those of the same scene without drawing |
| Text | D0 | `%.0f` to `%.3f` against snprintf (R7) |
| Asserts | F0 | the release_only scenes with a counting handler, against `-Dreference_asserts=true` (R8) |

**Soft bodies in SimulationParity** (SH builds the harness, S5 the scenes):
- `SimSoftBodySettings`: a generator kind (cube, cloth, sphere, rod chain, skinned cloth) with VertexAttributes, bend type,
  LRA and optimize, or explicit data from the float and uint pools. The same generator runs on both sides, and the
  SaveBinaryState bytes are compared before the scene runs.
- `SimSoftBody`: every SoftBodyCreationSettings member.
- OpKind 53..69: create and add; vertex position, velocity and inverse mass; pressure, iterations, update position,
  double sided, vertex radius; skin constraints and the max distance multiplier; `skinVertices` (joint matrices from the
  pool); `customUpdate`; the listener mode (none, accept, reject, sensor, mass scales); the `getSoftBodyCreationSettings`
  round trip.
- RecordKind 70..74: soft body validate (both IDs, the settings in and out); soft body added (per contact body the
  vertex count and an FNV hash of the local contact points and normals); the sensor contacts.
- The state decoder learns each soft body's vertex, rod and skin counts from `jolt_sim_soft_body_info`.
- The soft body scenes have no blocker. Until the audit they had `blocked_by = .soft_bodies`, which became supported
  when `SoftBodyMotionProperties.is_stub` and `PhysicsSystem.soft_body_jobs_pending` were gone. The audit removed the
  gate (section 6).
- **Unsaved state in the D12 sweeps.** Jolt's `SaveState` does not save `mSkinStatePreviousPositionValid`: `SkinVertices`
  sets it and `UpdateSoftBodyState` clears it, and it selects the skin interpolation factor. A sweep that restores the
  snapshot and repeats the update must put it back, like the existing `pending_invalid` contact cache flags of
  `restoreZoltBeforeUpdate`. Without that, a skinned scene with 2 collision steps differs after a retry in step 2.
- Scenes: cube on a floor and under a box; cloth with fixed corners and LRA; dihedral cloth; pressure sphere; rod chain;
  skinned cloth with back stops; sensor overlap; LinearCast sphere vs cloth and vs cube in every listener mode (the CCD
  face); sleeping and waking through ProcessBodyPair; update position false; several soft bodies on one rigid body
  (the Finalize order); save, restore and cross restore; 2 collision steps; the 3 unit test scenes.
- Small bodies do not reach the whole constraint group protocol: a 10 × 10 cloth has only the serial group, the 6³ cube
  has one parallel group and an empty non-parallel group. So the list adds a 30 × 30 cloth (several parallel groups and
  a non-empty non-parallel group). Vertex counts that are not a multiple of 64 reach the partial last batch.
- Scenes whose colliding shape order depends on the tree use `restore_broad_phase`. At most 30 soft body scenes, sweep
  stride 25 or more, so the parity stays under 45 minutes.

## 5. Reference build change and CI

Done in the prototype commit (section 10), in `build.zig`:
- the option `-Ddebug_renderer` (`zolt_options.debug_renderer`, also for the determinism log build);
- `-DJPH_USE_CPU_COMPUTE` in `cpp_flags`, always;
- `-DJPH_DEBUG_RENDERER` in `cpp_flags` with the option;
- the 12 reference fixes (R1).

F0 added to `.github/workflows/zolt.yml`:
- `lint`: `zig build check -Ddebug_renderer=true`;
- `test`: rows `ubuntu Debug debug_renderer=true` and `ubuntu ReleaseFast double debug_renderer=true`;
- `parity`: a row `ReleaseFast debug_renderer=true` (it builds the second reference);
- `determinism`: the 5 hashes once more with `-Ddebug_renderer=true`. Since D3 these rows also record every frame
  (`-r`) and run the `.jor` check (`zig build debug-renderer-check`), in both precisions.

The audit added:
- `lint`: `port_status.py --check` with the M10 rules and the `TODO(debug_renderer)` check, and an enforced "Check
  milestone M10" step (R22);
- `test`: a row `ubuntu Debug use_llvm=false` (M10). Every include row sets every key of the matrix.

The first parity build after the change recompiles the C++ library (about 5 minutes per configuration).

## 6. Groups and order

**Rules for every group.**
- One worktree and one branch per group, based on the phase branch after its dependencies merged.
- Port, then review (a second agent checks every finding against Jolt), then fix. The lead merges and verifies.
- At most 3 groups at a time and one heavy build per agent. During the work run filtered tests and parity
  (`-Dtest-filter=SoftBody`, `Compute`, `Hair`, `DebugRenderer`, `soft_body`). Run one full parity before the merge.
  Delete the worktree's `.zig-cache` after the merge.
- The registries (`zolt.zig`, `unit_tests.zig`, `parity.zig`, `reference_sources.zig`) merge with the union driver and
  `tools/tidy_registry.py`. Since the audit, `tidy_registry.py` also sorts and deduplicates the `c_files` list of
  `reference_sources.zig`.
- Every other shared file is edited only in the group's own region. A group that needs a change elsewhere reports it.

| Group | Content | Owns | Shared (region) | After | Jolt lines |
|---|---|---|---|---|---|
| **F0** phase foundation (lead) | the prototype commit; `ZoltParity/ParityHooks.h` (first hour); the remaining recoverable asserts and unlocked tests of R8; GJKTests; `port_status.py --check-m10`; the CI rows (section 5); the full parity once in both renderer configurations; the PortingGuide rows (HLSL types, LibM, shader files, gated declarations) | `build.zig`, `Core/Core.zig`, `Core/Virtual.zig`, `Core/IssueReporting.zig`, `tools/port_status.py`, `.github/`, `ZoltTests/UnitTestFramework.zig`, `ZoltTests/Geometry/GJKTests.zig`, `ZoltParity/ParityHooks.h`, `Docs/` | one line per converted assert (PathConstraint, PhysicsSystem `jobIntegrateVelocity`, SkeletonMapper, Constraint, the parity sites) | – | ~300 |
| **S0** soft body foundation | R15, R16, R19's declarations: every type and field, `SoftBodyVertex.zig`, the iterator constructor, the manifold, the shared settings' `saveBinaryState` / `restoreBinaryState`, `allocateSoftBody`, `createSoftBody*`, the `deleteBody` fix, the fallible `applySoftBodyCreationSettings`, the 13 RTTI declarations and factory entry 49, the S17 exclusions of the 13 classes; algorithm bodies are `@panic("Phase 9 S1")`-style stubs; `SoftBodyMotionProperties.is_stub` | `Physics/SoftBody/*.zig`, `Collision/CollideSoftBodyVertexIterator.zig` | `Body.zig`, `BodyManager.zig`, `BodyInterface.zig` (soft body functions); `RegisterTypes.zig` (factory); `ObjectStreamSchema.zig`, `gen_serializable.py`, `port_status.py` (`PHASE9_SERIALIZABLE`) | – | ~1100 |
| **C0** compute core | R9 to R12 core: the CPU backend, `HLSLToCPP.zig`, `ShaderWrapper.zig`, `ShaderCore/Math/Vec3/Quat/Mat44/Plane`, `HairStructs`, `HairCommon`, TestCompute / TestCompute2 / TestComputeWrapper, ComputeTests (M10), the dispatch parity for them | `Zolt/Compute/**`, `Zolt/Shaders/{Shader*,HairStructs,HairCommon,TestCompute*}.zig`, `ZoltTests/Compute/`, `ZoltParity/Compute/` | – | – | ~1300 |
| **S1** shared settings | R20's settings quirks: CreateConstraints, closest kinematic, rods, bends, volumes, skinned normals, Optimize, Clone, `createCube`; the SoftBodySettings layer | `SoftBodySharedSettings.zig` (algorithm region), `ZoltParity/Physics/SoftBodySettings*` | – | S0 | ~1235 |
| **S3** soft body shape | `SoftBodyShape` (every override, the queries, `register`); the soft body part of PairwiseParity | `SoftBodyShape.zig` | `PairwiseParity.zig` / `PairwiseReference.cpp` (soft body region) | S0 | ~340 |
| **D0** renderer foundation | the gated vtable entries and placeholders of Constraint and VehicleController; `PhysicsSystem.debug_renderer` and `setDebugRenderer`; DebugRendererSimple, DebugRendererPlayback; `writeFloatFixed` / `cppFixed` and their parity (R7); the call-log harness on both sides; the `debug-renderer-check` step | `Renderer/DebugRendererSimple.zig`, `Renderer/DebugRendererPlayback.zig`, `ZoltParity/Renderer/` | `Constraint.zig`, `VehicleController.zig` (vtable lines); `PhysicsSystem.zig` (field and setter); `StringTools.zig` (fixed region); `build.zig` (the step) | – | ~700 |
| **S4** motion properties | colliding shapes, collision planes, sensors, pressure, integration, the 7 solvers, collision response, `updateSoftBodyState`, `updateRigidBodyVelocities`, the state machine (UB1), `skinVertices`, `customUpdate`, Save/RestoreState; the SoftBodyUpdate layer | `SoftBodyMotionProperties.zig` | `SoftBodyReference.cpp` (customUpdate region) | S1, S3 | ~1310 |
| **S2** soft body binary state and scene | creation settings binary state, `saveWithChildren` / `restoreWithChildren`, `saveWithMaterials` / `restoreWithMaterials`, `PhysicsScene.mSoftBodies`; the PhysicsScene exclusions; P7, P9, BodyInterfaceParity | `SoftBodyCreationSettings.zig`, `PhysicsScene.zig` | `SoftBodySharedSettings.zig` (with-materials region); the ObjectStream harness | S0 (S1 for `createCube` in tests) | ~410 |
| **SH** soft body simulation harness | section 4: descriptions, generators on both sides, OpKind 53..69, RecordKind 70..74, the decoder, the skin flag in the sweeps, the `Blocker` flip, one smoke scene | – | `SimScenes.zig`, `SimulationReference.cpp`, `SimulationParity.zig` (soft body regions) | S0 | harness |
| **S5** soft body integration | R17, R18: the 4 jobs and their D5 rows, the CCD face, LoggingSoftBodyContactListener, SoftBodyTests (**M10 at its merge**), the soft body scenes and their bites, the FailingHeapAllocator test | `ZoltTests/LoggingSoftBodyContactListener.zig`, `ZoltTests/Physics/SoftBodyTests.zig` | `PhysicsSystem.zig` (soft body jobs, CCD face); `SimScenes.zig` (soft body scene list); `SimulationArchitecture.md` (D5 rows) | S4, SH | ~260 + scenes |
| **C1a** hair shaders I | HairTeleport, HairApplyDeltaTransform, HairSkinVertices, HairSkinRoots, HairApplyGlobalPose (+ .h), HairUpdateRoots, HairCalculateRenderPositions (+ .h), HairGridClear / Accumulate / Normalize; their dispatch parity (seed: the fidelity grid shaders) | those `Zolt/Shaders` files | – | C0 | ~700 |
| **D1** renderer helpers | every helper of DebugRenderer.cpp, `initialize` with all 25 batches, the caches and `nextFrame` (R3, R6's base lock); the helper call-log layer | `Renderer/DebugRenderer.zig` | – | D0 | ~800 |
| **C1b** hair shaders II | HairCalculateCollisionPlanes, HairIntegrate (+ .h), HairUpdateStrands, HairUpdateVelocity (+ .h), HairUpdateVelocityIntegrate, HairWrapper (all 15 registrations); their dispatch parity | those `Zolt/Shaders` files | – | C0, C1a | ~800 |
| **H1** hair settings | HairSettings, RegisterHair, HairShaders, the 10 schemas, LibM, the libc++ shuffle, `LibMReference.c`; the LibM, shuffle and HairSettings layers | `Physics/Hair/HairSettings.zig`, `RegisterHair.zig`, `HairShaders.zig`, `Math/LibM.zig`, `ZoltParity/Hair/` | `Core/Mt19937.zig` (shuffle region); the ObjectStream harness (RegisterHair factory); `gen_serializable.py`, `port_status.py` (the Hair exclusions) | C0 | ~1400 |
| **D2** shape draws | every shape's Draw / DrawGetSupportFunction / DrawGetSupportingFace, the caches (R6), the `getSubmergedVolume` debug options and their call sites, PolyhedronSubmergedVolumeCalculator, the Body buoyancy draw; the user and parity test shapes get a `draw`; then `Shape.impl.draw` goes | the draw regions of the 21 shape files, `PolyhedronSubmergedVolumeCalculator.zig` | `Body.zig` (buoyancy) | D1 | ~780 |
| **H2** hair runtime | Hair (without draw); the hair scenes | `Physics/Hair/Hair.zig` | `ZoltParity/Hair/` (scenes) | H1, C1a, C1b | ~950 |
| **D3** constraint, body and system draws | the 13 constraints, ConstraintManager, TwoBody, PathConstraintPath; BodyManager.draw (without its soft body section), DrawSettings, EShapeColor, ESoftBodyConstraintColor, BodyDrawFilter; the PhysicsSystem draw API and CCD draws; ContactConstraintManager and ManifoldBetweenTwoFaces; `recordDebugOutput`; PerformanceTest `-r`; the `.jor` check; then `Constraint.impl.drawConstraint` goes | the draw regions of those files | `PhysicsSystem.zig`, `BodyManager.zig` (draw regions) | D1, D2, S5 | ~920 |
| **D4** character, vehicle, skeleton and soft body draws | CharacterVirtual, the vehicle controllers and engine, SkeletonPose; the SoftBodyMotionProperties draws, SoftBodyShape.draw, BodyManager.draw's soft body section; then `VehicleController.impl.draw` goes | those draw regions | `BodyManager.zig` (soft body draw section) | D1, D3, S4 | ~600 |
| **H3** hair draw | `Hair.draw` and its DrawSettings | `Hair.zig` (draw region) | – | H2, D1 | ~315 |
| **A** audit | a completeness audit against the Jolt sources (34 findings, each fixed or documented); `--check-m10` and "no `TODO(debug_renderer)`" in `port_status.py --check`; Roadmap, Progress, this document's "settled" sections; the final verification | `Docs/` | – | all | – |

**Order (at most 3 groups at a time, the M10 path first).**

| Wave | Slot 1 | Slot 2 | Slot 3 |
|---|---|---|---|
| 1 | F0 | S0 | C0 |
| 2 | S1 | S3 | D0 |
| 3 | S4 | S2 | SH |
| 4 | S5 (M10 is reachable at its merge) | C1a | D1 |
| 5 | C1b | H1 | D2 |
| 6 | H2 | D3 | – |
| 7 | D4 | H3 | – |
| 8 | A | – | – |

All eight waves are done: every group is merged, and the audit ended the phase.

- Settled in wave 4:
  - D1 (R6): the cached helpers create their batches under the cache lock, so the cache lock is always taken before the
    recorder's lock. The final `drawGeometry` call runs after the cache lock is released, with its own reference to the
    geometry: the lock stays short, and a renderer whose `drawGeometry` draws a cached helper cannot deadlock.
  - D1 (R5): when a renderer returns an empty batch and latches, the cached helpers and
    `createTriangleGeometryForConvex` drop the geometry instead of caching one with an empty LOD. Callers (D2: the convex
    shapes, GJK and EPA draws) skip the draw when the geometry or batch is empty.
  - S5: the soft body scenes ran with `blocked_by = .soft_bodies` until the audit. The blocker reported them supported
    once `SoftBodyMotionProperties.is_stub` and `PhysicsSystem.soft_body_jobs_pending` were gone. The audit removed the
    gate: `Blocker.soft_bodies`, `SoftBodyClasses` and the 23 markers are gone, and the soft body scenes have no
    blocker.
  - M10 is reached with S5: `python3 tools/port_status.py --check-m10` passes.
- Settled in wave 5:
  - D2: the draw caches add a pointer only to MeshShape (88 to 96 bytes); the others fit it in padding. The C++
    reference keeps Jolt's static shape draw flags; `jolt_dr_run_shape_ops` resets them around each run.
  - H1: one more recoverable assert site (R8): the `Array` index assert of `HairSettings.init` (UB19).
  - H1: `LibM.expf` is written from ARM optimized-routines (Zig 0.16 ships musl's `exp2f_data.c`, `powf.c` and
    `powf_data.c`, not `expf.c`). The reference compiles `ZoltParity/Hair/LibMReference.c` (C files get their own flags in
    `build.zig`) and forces it into the link with a marker call.
  - For H2: the hair scenes avoid UB16 (no last segment that leaves the float sum of the segment lengths unchanged
    together with a step gradient on the grid velocity factor) and UB11 (no NaN grid positions).
- Settled in wave 6:
  - D3: `ZoltPerformanceTest -r` records the bodies like PerformanceTest; `zig build debug-renderer-check
    -Ddebug_renderer=true` compares Zolt's `.jor` bytes with Jolt's for the 5 scenes (CI runs it in ReleaseFast).
  - D3: a filtered parity run analyzes only the tests it runs. `zig build parity-check` compiles every parity test and
    the C++ reference without running them; groups run it in both renderer configurations before they report.
  - H2: "uniform strands" means strands with the same number of vertices (no padding vertices); two scenes cover it.
- Settled in wave 7:
  - H3: one more recoverable assert site (R8): `HairSettings.getNeutralDensity` (reached by `Hair.draw`'s density
    loops, the quirk of R20), tested with `ExpectAssert` and counted by the hair draw parity against
    `-Dreference_asserts=true`. Jolt's hair draw flags are per-call `DrawSettings` members, not statics.
  - D4: every `TODO(debug_renderer)` is gone; the vehicle per-wheel texts go to the controller's renderer argument (R4).
- Settled in wave 2: DebugRendererPlayback (UB7): a LOD batch ID 0 is `Batch.empty` unless the stream defined a
  batch with ID 0. An unknown batch ID and a stream that ends inside a command are `error.InvalidStream`. `drawFrame`
  skips a geometry ID that the stream did not define, because Jolt looks it up only there. A later `parse` can still
  define it. A stream that ends between commands is complete; the frames that Jolt replays with defined behavior stay
  after an error. `TwoBodyConstraint.impl.drawConstraintReferenceFrame` was a placeholder until D3, like
  `Constraint.impl.drawConstraint`.
- Settled in wave 1: S0 ported `initialize`, `calculateMassAndInertia`, `getVolume` and the trivial SoftBodyShape
  overrides, because R16's fallible creation needs them; the creation parity layer covers them. S4 does not port them
  again. `SoftBodyReference.cpp` is S0's file (C prefix `jolt_sb_`); S4 and SH add their parts in their own regions.
- M10 needs F0, C0 and S5. S5 is on the critical path: F0 / S0, then S1 and S3, then S4 and SH, then S5.
- F0, S0 and C0 share no file but the registries and `port_status.py` (different regions).
- The two groups that edit `PhysicsSystem.zig` (S5, D3) and the three that edit `BodyManager.zig` (S0, D3, D4) run in
  different waves.
- A group that grows past one agent session splits along existing seams, each half with its own tests: S1 into
  CreateConstraints with the Calculate* functions and Optimize with `createCube`; C0 into the backend with TestCompute
  and the shader helpers with TestCompute2; D1 into `initialize` with the helpers and the caches.

**Settled in the audit.** The audit checked the merged phase against the Jolt sources. It confirmed 34 minor findings,
and each one is fixed or documented. It found no result difference from Jolt in library code. These points were
settled:
- Library:
  - UB28 to UB34 (R20) give Zolt one defined behavior for more inputs on which Jolt is undefined. The UB8 row now
    covers every material run.
  - R15 (the alignment of the soft body contexts) and R17 (`CustomUpdate` on a body that was never added) said the
    wrong thing. The code was right; the text is fixed.
  - `Shape.impl`'s two default draw bodies are gated declarations (R2).
  - DebugRendererPlayback's unknown command is a recoverable assert (R8).
- Tests: allocation failure sweeps for DebugRendererRecorder and for `HairShaders.init`. The DebugRendererSimple sweep
  checks its success round and reaches the triangle buffer of the indexed batch. A test of
  `TransformedConvexObject(AABox).getSupportingFace` replaces a stale TODO.
- Harness:
  - The soft body gate is gone (wave 4 above). Every Blocker is supported. A scene whose Blocker is not supported now
    fails (`checkSupported` in SimulationParity.zig) instead of being skipped.
  - The test "SimScenes blocked scenes run on the C++ side" is removed, with `checkCppSettingsRoundTrips` and
    `checkSoftBodyBlocker`: no scene is blocked any more.
  - Its checks moved into the scene tests. In its first configuration (`Config.check_cpp_values`), every scene test
    checks that every C++ object state and the final C++ state decode completely and are finite. `runEntry` checks
    that a scene with a Phase 7 blocker creates the objects of its classes, and that a `.contacts` or `.ccd_hits`
    scene reaches contacts. A C++-only run over all 310 scenes found no scene that fails these checks.
  - A new scene is checked against Jolt's asserts by its name:
    `zig build parity -Dreference_asserts=true -Dtest-filter=<scene name>`.
  - `DebugRendererCheck.zig` has no `zolt_records` gate any more (D3 ported the Zolt side).
- Tooling and CI: `port_status.py --check` runs the M10 rules and fails on any `TODO(debug_renderer)` (R22). CI
  enforces M10 and has a `-Duse_llvm=false` test row (section 5). `tidy_registry.py` sorts the `c_files` list.
  Progress.md calls the deferred files "files that stay deferred or out of scope".
- Docs: this document, the Roadmap (Phase 9 and M10), SimulationArchitecture D1, CollisionArchitecture,
  SerializationArchitecture S17 and S18, CharacterVehicleArchitecture, the PortingGuide and CLAUDE.md describe the final
  state.

## 7. Checklist for porters

- File headers: `//! Port of:` (with the `.hlsl` and helper headers of a shader) and `//! Status:`. Keep Jolt's doc and
  implementation comments. Never leave a `TODO(debug_renderer)` line: `port_status.py --check` fails on one.
- Debug renderer code: the gates of R2 only. A draw function is a gated declaration. A debug-only member is `void`
  without the option. Never read a static flag from anywhere but `renderer.flags` (R4).
- No new globals. The assert handler is the only one (R8).
- Shaders: HLSLToCPP semantics (R11), never Zolt's `Vec3` / `Quat` operations.
- Soft bodies: Zolt's `QuickSort` and `BinaryHeap`, never `std.sort`; Jolt's memory orders; the D5 rows of R17.
- Libm and shuffle: `LibM.expf` / `powf` and `Mt19937.shuffleLibcxx`, never `std.math`.
- Jolt's undefined behavior: the UB rows of R20, pinned by Zolt-only tests, never generated by parity.
- Parity: the group's layer of section 4 and a bite proof.
- Done criteria:
  - `zig build test` in Debug, double precision, ReleaseFast and `-Duse_llvm=false`, and with `-Ddebug_renderer=true`
    for a group that touches a gate;
  - `zig build check` and `zig build check -Ddebug_renderer=true`;
  - `zig build parity-check` in both renderer configurations: a filtered parity run compiles only the tests it runs;
  - `zig build parity` filtered during the work, full at the end; with `-Ddebug_renderer=true` too for D groups and for
    any file with a gate;
  - `zig fmt --check Zolt ZoltTests ZoltParity ZoltHelloWorld ZoltPerformanceTest build.zig build.zig.zon`;
  - `python3 tools/port_status.py --write`, then `--check` (it also runs the M10 rules and the `TODO(debug_renderer)`
    check).
- Delete the worktree's `.zig-cache` at the end.

## 8. Out of scope

- The GPU compute backends (`Compute/DX12`, `VK`, `MTL`, the `.mm` files) and `ComputeSystem::mShaderLoader`.
- The shader binding override macros (`JPH_SHADER_BIND_SCALP_VERTICES`, `_TRIANGLES`, `_RENDER_POSITIONS`): Zolt keeps the
  default buffer formats. Hair's setters for external buffers are ported.
- `Core/Profiler`.
- TestFramework (the renderer backends, fonts), Samples, JoltViewer. Zolt's `.jor` files play in JoltViewer, because the
  bytes are Jolt's.
- Jolt's developer debug switches and their `sInstance` draws.
- `JPH_DEBUG_RENDERER_EXPORT`, `DEBUG_RENDERER_IN_DISTRIBUTION`, a shared library, a C API.
- Hair level of detail, wind, grid advection and non-hull collision: missing in Jolt too.
- A parallel CPU dispatch: it would not change results, but it is a Zolt extension.

## 9. Rejected alternatives

- **The static draw flags and `sInstance` as `pub var`** (the static member rule). Every reader has an owner. Process
  globals read on job threads are what "no globals" exists for, and a global renderer makes two systems share one.
- **Draw functions that are always compiled, only their references gated.** A body that reads a member that is `void`
  without the option does not compile (a judge reproduced it). Gated declarations compile in both configurations.
- **Vtable entries always present, only call sites gated.** Every vtable grows, every draw function links, and "zero cost
  when off" would rest on review.
- **Default `debug_renderer` by optimize mode** (CMake's `DEBUG_RENDERER_IN_DEBUG_AND_RELEASE`). Two references for one
  `zig build parity`, and a default build that is not Phase 8's.
- **Draw functions returning `Allocator.Error`.** 140 sites would carry an error into the jobs for output that never
  affects results.
- **Shape draw caches in the renderer.** It changes Jolt's `.jor` bytes (fresh batches per renderer) and keeps shapes
  alive through the renderer.
- **A comptime CPU-only compute backend.** Generic Hair, no `IsKindOf`, no room for a GPU backend.
- **Shaders written with Zolt's `Vec3` / `Quat` or `@Vector`.** Other operation orders, other min / max semantics, other
  layouts.
- **Comparing Zolt's hair with the reference's unpinned `expf`.** It is `compiler_rt`'s, an older algorithm that differs
  from glibc on millions of inputs; on another host it would be another library.
- **`GLIBC_TUNABLES` on the parity run.** It cannot reach `compiler_rt`'s `expf`, and it is glibc only.
- **Converting all 250 release-reachable asserts.** It touches 79 files that other groups own, and no test needs it.
  ReleaseFast already continues on Jolt's release path.
- **`ExpectAssert` in `ZoltTests/` only.** The inline tests of `Zolt/` cannot import it.
- **A per-system or thread-local assert handler.** Asserts fire on job threads and in code without a system.
- **Replicating Jolt's hang for a soft body without vertices.** A hang is never a useful result (UB1).
- **S4 and S5 as one group.** About 1570 lines, too much for one session; `customUpdate` lets S4 be verified without the
  jobs.
- **A separate soft body simulation parity.** SimulationParity already has the thread matrix, the D12 sweeps, the decoder
  and the cross restore.
- **A strictly serial foundation.** The track foundations (S0, C0, D0) start beside F0, so the critical path stays short.

## 10. The foundation prototype and the final state

**The final state (end of Phase 9).**

| Area | Zig files | Status |
|---|---|---|
| Soft body | the 8 files of `Zolt/Physics/SoftBody/` (12 Jolt files); the soft body parts of `Body`, `BodyManager`, `BodyInterface`, `PhysicsSystem` and `PhysicsScene`; `CollideSoftBodyVertexIterator.zig` | complete |
| Compute | the 4 files of `Zolt/Compute/` and the 6 files of `Zolt/Compute/CPU/` | complete. The GPU backends stay deferred (R21) |
| Shaders | the 27 files of `Zolt/Shaders/`: the 17 shaders, the 8 shared headers and the 2 wrappers | complete |
| Hair | the 4 files of `Zolt/Physics/Hair/`; `Zolt/Math/LibM.zig` and the libc++ shuffle of `Core/Mt19937.zig` (R13) | complete |
| Debug renderer | the 4 files of `Zolt/Renderer/`; the draw code of the 103 Jolt files with `JPH_DEBUG_RENDERER`, behind `-Ddebug_renderer` | complete. No `TODO(debug_renderer)` is left |
| Asserts | `Zolt/Core/IssueReporting.zig` with `ExpectAssert` | complete |
| Unit tests | GJKTests, UnitTestFramework, ComputeTests, SoftBodyTests, LoggingSoftBodyContactListener, the `ExpectAssert` part of PhysicsTests | complete. Every unit of `UnitTests/` is complete or n/a (M10) |
| Schemas | the 13 soft body classes and the 10 hair classes | declared. With Phase 8's 81 classes, 104 classes are declared and none is pending (Progress.md) |

- `python3 tools/port_status.py --check` and `--check-m10` pass. In Progress.md every Phase 9 directory is at 100%.
  The library is at 99.6%, because `Core.zig` stays partial (section 1).
- CI runs the tests in every M10 configuration, the full parity in both renderer configurations, the 5 determinism
  hashes in both precisions and both renderer configurations (with `-r` when the option is on), and the `.jor` check
  (section 5).
- Every parity layer of section 4 is in the tree. Every scene of SimScenes runs against Zolt: every Blocker is supported
  (section 6, "Settled in the audit").

**The prototype (at the design commit).**

| File | What it was |
|---|---|
| `build.zig` | the option `-Ddebug_renderer`; `-DJPH_USE_CPU_COMPUTE` always; `-DJPH_DEBUG_RENDERER` with the option (R1, R2) |
| `Zolt/Core/Core.zig` | `debug_renderer` |
| `Zolt/Core/Virtual.zig` | `OptionalVirtual`, void entries in `make`, `isVirtual` and `checkTopLevel`; a test of both configurations with a gated override that reads a void member |
| `Zolt/Core/IssueReporting.zig` (complete) | the assert handler, `assertRecoverable`, `assertFailed`, `ExpectAssert`; tests (4 threads × 100 asserts) |
| `Zolt/Renderer/DebugRenderer.zig` (partial then, complete since D1) | the interface, the types, `DrawFlags`, the atomic latch, the box geometry of `initialize`, DrawWireBox (2), DrawMarker, DrawArrow, DrawBox (2), the 6-argument DrawGeometry |
| `Zolt/Renderer/DebugRendererRecorder.zig` (complete) | Jolt's `.jor` stream |
| `Zolt/Physics/Collision/Shape/Shape.zig`, `BoxShape.zig` | the three gated vtable entries, dispatchers and `impl` placeholders (gone since D2); `BoxShape.draw` as a gated declaration |
| `Zolt/Physics/Constraints/Constraint.zig` | `draw_constraint_size` (void without the option), the constructor copy, the `toConstraintSettings` copy, gated accessors |
| `Zolt/ConfigurationString.zig` | "(Debug Renderer) " |
| `Zolt/Compute/ComputeBuffer.zig`, `ComputeShader.zig`, `ComputeQueue.zig` (complete), `ComputeSystem.zig` (complete) | the pattern A interfaces, `ComputeSystem.rtti`, `createComputeSystem`; a test of IsKindOf, dispatch and destruction |
| `PhysicsSystem.zig`, `PlaneShape.zig`, `MeshShape.zig` | three recoverable asserts; the PlaneShape and MeshShape tests run with asserts |
| `ZoltTests/Physics/PhysicsTests.zig` | TestOutOfContactConstraints with `ExpectAssert(1)` |
| `ZoltTests/UnitTestFramework.zig` (complete) | re-exports `ExpectAssert` |
| `ZoltTests/Renderer/DebugRendererZoltTests.zig` + `golden/` | the hooks compile away; the recorder writes Jolt's box batch and frames (golden bytes written by Jolt); a user renderer; the latch; the draw size round trip |
| 12 `ZoltParity/Physics/*Reference.cpp` | the reference compiles with `JPH_DEBUG_RENDERER` |
| `tools/port_status.py`, `Docs/Zolt/Progress.md` | R21 |
| `Docs/Zolt/PortingGuide.md` | the rows of `#ifdef JPH_DEBUG_RENDERER`, static members and `ExpectAssert` |

**What the prototype showed.**
- Filtered tests (Virtual, IssueReporting, TestOutOfContactConstraints, DebugRenderer, ComputeSystem, the GetSubmergedVolume
  tests, getConfigurationString, TestPhysicsConstraintUserData) pass in Debug, double precision, with
  `-Ddebug_renderer=true` in both precisions, and in ReleaseFast with the option: 66 tests, 2 skipped without the option
  (the BoxShape draw test needs it), 1 skipped in ReleaseFast (the threaded assert test needs asserts).
- `zig build check` passes in both renderer configurations.
- The recorder writes Jolt's bytes: the box batch of `initialize` (1021 bytes, both precisions) and two frames with box
  draws, wire boxes, a marker and two arrows (1251 bytes, single precision). The goldens were written by Jolt in the Zig
  design prototype; swapping two lines of `drawWireBox` failed at byte 64 there.
- With the option off, every shape vtable has only its old entries (a test), and `BoxShape.draw` and
  `Constraint.draw_constraint_size` are `void`.
- `GetConstraintSettings` returns the constraint's draw size with the option and 1.0 without it, like Jolt (a test).
- The filtered parity passes in both renderer configurations, each time with a C++ reference rebuilt with the new
  defines: 65 of 65 tests and the HelloWorld comparison, about 11 minutes each:

  ```sh
  zig build parity [-Ddebug_renderer=true] -Dtest-filter="Constraint parity" -Dtest-filter="vehicle_" \
      -Dtest-filter="GetSubmergedVolume" -Dtest-filter="ManifoldBetweenTwoFaces" -Dtest-filter="TriangleShape parity" \
      -Dtest-filter="ObjectStream P1" -Dtest-filter="ObjectStream P2" -Dtest-filter="constraints_chain"
  ```

  Every reference file is compiled into the reference library, so the second build proves that the 12 fixes of R1
  compile. The filters run most of the fixed calls (the submerged volumes, the manifold), the constraint settings that
  the define changes, the vehicle scenes and the schemas.
- Bite proof, repeated here: without Zolt's copy of the draw size in `toConstraintSettings`, the parity with
  `-Ddebug_renderer=true` fails ("Constraint parity: fixed" with 1476 mismatching records, the scene
  `vehicle_wheeled_ops`). So the option must drive both sides.
- `zig fmt --check` and `port_status.py --check` are clean.

**Seeds in the proposal commits** (readable in the shared object store; the groups copy what they need):
- `26c2ebf` (fidelity): `ZoltParity/Compute/Prototype/{HLSL,Compute,ShaderMath,TestShaders,HairGridShaders}.zig`,
  `ZoltParity/Compute/{ComputeParity.zig,ComputeReference.cpp}` (C0, C1a);
  `ZoltParity/SoftBody/Prototype/SoftBodyCreateConstraints.zig`, `ZoltParity/SoftBody/{SoftBodyParity.zig,
  SoftBodyReference.cpp}` (S1).
- `b90caaf` (Zig design): `ZoltTests/Prototype/ComputePrototype.zig` (the `Bindings` struct and `ShaderWrapperOf`, C0).
- `3759ab6` (delivery): `ZoltParity/Physics/{SoftBodyParity.zig,SoftBodyReference.cpp}` (the layout, decoder and
  thread-count tests, S0 and SH), `ZoltParity/Compute/{ComputeParity.zig,ComputeReference.cpp}` (C0).

## 11. Judges' findings and how they were settled

| Finding | Checked against | Settled |
|---|---|---|
| The fidelity proposal's static flags and `sInstance` are process globals | the readers of each flag (fact sheet, Jolt sources) | R4: flags on the renderer, the renderer passed or owned |
| "No Simulate job" could race with running Collide jobs | `PhysicsSystem.cpp:2653-2732`: Collide jobs start at creation, each Simulate job depends on all of them | R17: no Collide job is created then |
| Converting all 250 asserts conflicts with every group | the skipped tests (8 sites) | R8: only the sites tests expect |
| `ExpectAssert` in `ZoltTests/` cannot serve the inline tests | the module root is `Zolt/zolt.zig` | R8: in `Core/IssueReporting.zig`, re-exported |
| "Every draw function compiles in both builds" fails for bodies that read void members | `refAllDeclsRecursive` in `zolt.zig` | R2: gated declarations (the Virtual test reads a void member) |
| A serial foundation is a bottleneck | the group sizes | section 6: three track foundations in wave 1 |
| The placeholder of `Shape.impl.draw` panics with the option | R2 | the placeholder goes with D2; no test draws earlier |
| The reference fix list was wrong (VehiclePartsReference, ConvexHullShapeReference) | the fidelity build and the design commit's parity run with the define | R1: the 12 files that need it |
| The OOM latch of the delivery sketch was a plain bool | draws come from job threads | R5: an atomic |
| The libm premise: the reference's `expf` is not glibc's | `nm` of the parity binary: `t expf` = `compiler_rt.exp.expf` | R1, R13: pin both functions in the reference |
| `mSkinStatePreviousPositionValid` is not in `SaveState` | `SoftBodyMotionProperties.cpp:519, 917, 1229, 1446-1475` | section 4: the sweeps restore it |

## 12. Open questions for the lead

- CI cost: the `debug_renderer=true` parity row rebuilds the reference and reruns the full parity (30 to 40 minutes). If
  that is too slow, run it nightly, or restrict it to the renderer, simulation and perf filters. Settled: CI runs the
  full row on every change (section 5).
- Hair has no `.hair` groom asset in the repository (`Assets/download_hair.sh`). The parity uses procedural grooms. A
  small real groom could be added for an end-to-end scene. Still open: the hair parity uses procedural grooms, on the
  scalp of `Assets/face.bin` and without a scalp.
- UB1 (a soft body without vertices) advances the state. A panic at creation would be simpler, but it rejects input that
  Jolt accepts at creation. Settled: UB1 stays.
