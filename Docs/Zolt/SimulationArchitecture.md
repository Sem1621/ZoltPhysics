# Simulation architecture (Phase 5)

This is the binding design for porting Jolt's simulation core: `Jolt/Physics/Body`, the broad phase
(`Jolt/Physics/Collision/BroadPhase`), the constraint core (`Constraint`, `TwoBodyConstraint`,
`ConstraintManager`, `ContactConstraintManager`, `ConstraintPart/*`) and `PhysicsSystem` with its
helpers. Every decision (D1..D13) is written as an instruction for porters. It was decided with
three competing compiled prototypes (fidelity first, Zig idiom and safety first, performance and
determinism first), a judge and a synthesis. The synthesized prototype is
`ZoltTests/Prototype/SimulationPrototype.zig` plus `ZoltTests/Prototype/Simulation/*.zig`. It was
added with this document and is removed at the end of F2. The code examples below are copied from
it. Read `PortingGuide.md` and `CollisionArchitecture.md` first: this document only adds to them.

**What the prototype proves.** It runs against the real library (`@import("zolt")`), and its 14
tests pass in Debug, ReleaseFast, ReleaseSafe and `-Duse_llvm=false`, in both precisions, with
leak checking:
- Jolt's complete job graph per collision step, with dynamic FindCollisions and FindCCDContacts
  jobs and real islands, gives byte-identical `saveState` output after each of 60 updates. This
  holds for `JobSystemSingleThreaded` and for thread pools with 0, 1, 3 and 7 workers, while the
  order of contacts and activations differs between runs.
- Every temp allocation, every `createJob` and a heap allocation inside a job fail in turn. A
  failure while building the job graph changes nothing. A failure inside a job leaves the temp
  allocator empty and recovers through `restoreState`.
- Two deliberate breakages are caught by the tests. Without the contact sort the determinism tests
  fail. Without the start gate the `createJob` sweep fails and leaks.

**What it reduces.** The prototype simplifies these parts, so they are not evidence for the final
code:
- The contact manager is a stand-in: one closest hit per pair and no cache.
- CCD casts against the plane y = 0.
- The broad phase is BruteForce with QuadTree's locking and allocation protocol.
- There is no sleeping and no LargeIslandSplitter.

## 0. The rules every Phase 5 porter must know

1. **Rule M** (CollisionArchitecture section 0) applies unchanged, with its two exceptions. Phase 5
   adds no exception. It uses these patterns:
   - A C++ `mutable` mutex or lock becomes a field of a heap block `mutable_fields: *MutableFields`,
     allocated in `init`. Const functions then keep `*const` receivers. Never make a query receiver
     mutable to lock a mutex.
   - A `const_cast` of a `LockFreeHashMap` find result becomes `findMut`.
   - Jolt's `Body &` references to `sFixedToWorld` become `Body.fixedToWorld()`, the one audited
     `@constCast` of Phase 5 (D1).
2. **Bit exact and independent of the thread count.**
   - Keep Jolt's floating point order, Jolt's atomic memory orders 1:1 (a `seq_cst` stays
     `.seq_cst`), Jolt's sorts with Jolt's comparators (`zolt.quickSort` / `zolt.insertionSort`,
     never `std.sort`) and Jolt's job graph (D4).
   - Contact order, activation order and broad phase pair order are nondeterministic in Jolt too.
     Only Jolt's own sorts make them deterministic. Never "fix" an order by other means.
3. **Jolt's TSAN-suppressed races become `.monotonic` atomics.** The atomic load or store compiles
   to the same instruction on x86 and ARM, and it removes the undefined behavior. This applies to:
   - `BodyManager.bodies_size`;
   - `MotionProperties.index_in_active_bodies`;
   - the body pair queue slots;
   - the QuadTree node bounds, loaded per lane.
4. **Two error channels** (CollisionArchitecture section 0):
   - Jolt's soft errors stay values: `PhysicsUpdateError`, `bool` returns, null pointers, invalid
     IDs.
   - Allocation failure is `error.OutOfMemory`, as `Allocator.Error` or `TempAllocator.Error`
     (the same set). Inside jobs it is latched (D5).
   - Where Jolt crashes ("QuadTree: Out of nodes!", no barrier), Zolt calls `@panic` with Jolt's
     text.
5. **`io`.** The owners that block store it at `init`: BodyManager, the broad phase,
   ConstraintManager and PhysicsSystem. Everything else reads `body_manager.io`. User-facing
   functions keep Jolt's signatures and never take `io`.

## 1. Decisions

### D1 Body, MotionProperties and the fixed-to-world body

**Layout.** `Body` is a plain struct with Jolt's fields in Jolt's order and Jolt's alignment group
comments. Jolt's size is checked at compile time; it also holds with Zig's field reordering:

```zig
    comptime {
        // static_assert(sizeof(Body) == JPH_IF_SINGLE_PRECISION_ELSE(128, 160)) holds with Zig's field ordering too
        if (@sizeOf(usize) == 8 and Core.object_layer_bits == 16)
            std.debug.assert(@sizeOf(Body) == if (Core.double_precision) 160 else 128);
        std.debug.assert(@alignOf(Body) == @max(Core.vector_alignment, zolt.rvector_alignment));
    }
```

**Flags.**
- Private flag enums that Jolt changes with atomic or / and (`Body::EFlags`,
  `CachedManifold::EFlags`) become integer constants in a `Flags` namespace on an atomic field
  (`flags: std.atomic.Value(u8)`). Jolt's `memory_order_relaxed` is `.monotonic`:

  ```zig
      pub fn setInBroadPhaseInternal(self: *Body, in_broad_phase: bool) void {
          if (in_broad_phase)
              _ = self.flags.fetchOr(Flags.is_in_broad_phase, .monotonic)
          else
              _ = self.flags.fetchAnd(~Flags.is_in_broad_phase, .monotonic);
      }
  ```
- Public bit flag enums (`EAllowedDOFs`, `EStateRecorderState`, `EPhysicsUpdateError`) become
  `packed struct(uN)` with Jolt's bit order. Jolt's named combinations become decls
  (`AllowedDOFs.all`, `AllowedDOFs.plane_2d`, `StateRecorderState.all`). A comptime block checks
  the bits.

**Races and debug state.**
- `MotionProperties.index_in_active_bodies` is a `std.atomic.Value(u32)`, loaded and stored with
  `.monotonic`. `FindCollidingPairsCanCollide` reads it while another job activates the body.
- Jolt members that exist only with `JPH_ENABLE_ASSERTS` are typed `void` when asserts are off. The
  compiler then proves that they cannot influence release results:

  ```zig
      cached_motion_type: if (Core.enable_asserts) MotionType else void = if (Core.enable_asserts) .static else {},
  ```

**Allocation.**
- `BodyWithMotionProperties { body: Body, motion_properties: MotionProperties }` is one
  `allocator.create`.
- `BodyManager.deleteBody` frees it through `@fieldParentPtr("body", body)` when
  `motion_properties != null`.
- Phase 9 adds `SoftBodyWithMotionPropertiesAndShape` the same way.

**Fixed to world.** `Body::sFixedToWorld` becomes a compile time constant in read-only memory. Its
dummy shape is a static `EmptyShape`: `Shape.is_static` was added with this document, and it makes
`addRef` / `release` do nothing, like `PhysicsMaterial.default`:

```zig
    /// static const EmptyShape sFixedToWorldShape: a static shape (never reference counted) in read-only memory
    const fixed_to_world_shape: EmptyShape = .initStatic(Vec3.zero());

    pub fn fixedToWorld() *Body {
        return @constCast(&fixed_to_world_storage);
    }
```

- `fixed_to_world_storage` is Jolt's `Body(bool)` constructor. Its shape is
  `.shape = .{ .ptr = fixed_to_world_shape.asShape() }`.
- Constraints store `*Body` like Jolt's `Body &`. The solver never writes a static body, because
  every write path first checks `isDynamic` or the motion properties.
- A stray write, such as a user's `setUserData` on it, faults instead of silently changing state
  that all physics systems share.
- `RefConst(Shape).init(body.getShape())` and `TransformedShape` on it are legal.

**Soft bodies.**
- Port every soft body branch of Body, BodyManager, BodyInterface and PhysicsSystem now.
- The soft body types are stubs with their final names and signatures:
  - `SoftBodyMotionProperties` has `fromBase` / `fromBaseConst`; `SoftBodyCreationSettings` and
    `SoftBodyUpdateContext` are also stubs.
  - `SoftBodyContactListener` is the real pattern B interface.
- The creation entry points (`BodyInterface.createSoftBody`, `createAndAddSoftBody`,
  `BodyManager.allocateSoftBody`) are not declared until Phase 9 and are listed under
  `//! Missing:`. No soft body can then exist, so every `@panic("soft bodies: Phase 9")` stub is
  unreachable from the public API, and Phase 9 only adds code.

### D2 BodyManager

**Init.** BodyManager is initialized in place (guide section 5), because the lock interfaces point
into it:

```zig
    pub fn init(self: *BodyManager, allocator: Allocator, io: std.Io, max_bodies: u32, num_body_mutexes_in: u32, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void {
```

The number of body mutexes is Jolt's:
`clamp(ceilPowerOfTwo(n, or 2 * CPU count when n is 0), 1, 64)` entries of
`MutexArray(SharedMutex)`.

**Body array.**
- `bodies: []BodySlot` is allocated once with `max_bodies` entries. `bodies.len` is
  `GetMaxBodies()`.
- `bodies_size: std.atomic.Value(u32)` is Jolt's `mBodies.size()`. It is stored with `.release`
  under the bodies mutex, after the slot is written, and loaded with `.acquire` by the lock-free
  `tryGetBody` and `getBodies`.
- A slot is a body pointer or a freed index, with Jolt's encoding:

```zig
    pub const BodySlot = enum(usize) {
        /// Indicates that there are no more freed body IDs (cBodyIDFreeListEnd)
        free_list_end = ~@as(usize, 0),
        _,

        pub fn fromBody(body: *Body) BodySlot {
            return @enumFromInt(@intFromPtr(body));
        }

        /// Test if a body pointer is valid (sIsValidBodyPointer)
        pub fn isValidBodyPointer(self: BodySlot) bool {
            return (@intFromEnum(self) & is_freed_body) == 0;
        }
```

**IDs.** ID allocation is literal: a LIFO free list in the freed slots, `+%=` on the `u8` sequence
numbers, and `addBodyWithCustomID`'s O(N) unlink. The prototype test "BodyManager ID allocation
like Jolt" pins the sequence.

**Active bodies.**
- `active_bodies: [body_type_count][]BodyID` is preallocated.
- `num_active_bodies: [body_type_count]std.atomic.Value(u32)` uses Jolt's orders:
  `fetchAdd(1, .release)` after the ID is written, and `.acquire` in `getNumActiveBodies`.

**`mutable` mutexes.** They go in the pointer-field block, so `getNumBodies`, `lockAllBodies`,
`getActiveBodies` and `saveState` keep `*const BodyManager`:

```zig
    /// The C++ `mutable` members that const functions lock (Rule M: behind a pointer, never through @constCast)
    const MutableFields = struct {
        /// Protects mBodies array (but not the bodies it points to), mNumBodies and mBodyIDFreeListStart
        bodies_mutex: Mutex = .{},
        /// Mutex that protects the mActiveBodies array
        active_bodies_mutex: Mutex = .{},
    };
```

**Errors.**
- `allocateBody(settings) CreateBodyError!*Body` resolves the shape first, so a failure leaves
  nothing to undo.
- `addBody` keeps Jolt's `bool` for "out of IDs".
- `getActiveBodies`, `saveState` and `restoreState` return `Allocator.Error`, because they fill
  caller arrays or build sorted arrays.

### D3 Locks: BodyLockInterface, BodyLock*, PhysicsLock

**BodyLockInterface** is a closed tagged struct, because Jolt has exactly two `final`
implementations and nothing else derives from the base. Both C++ class bodies stay side by side in
an exhaustive switch:

```zig
    /// Overloads get suffixes: LockRead(BodyID) / LockRead(MutexMask) -> lockRead / lockReadMask
    pub fn lockRead(self: *const BodyLockInterface, body_id: BodyID) ?*SharedMutex {
        switch (self.kind) {
            .no_lock => return null,
            .locking => {
                const bm = self.body_manager;
                const mutex = bm.getMutexForBody(body_id);
                PhysicsLock.lockShared(mutex, bm.io, bm, .per_body);
                return mutex;
            },
        }
    }
```

- Construct the two variants with `initNoLock` / `initLocking`.
- PhysicsSystem embeds both values and hands out `*const BodyLockInterface`.
- The guide documents this as the exception for closed `final` class sets.

**RAII locks.**
- `BodyLockBase(comptime write: bool)` and `BodyLockMultiBase(comptime write: bool)` keep Jolt's
  names: `releaseLock`, `succeeded`, `succeededAndIsInBroadPhase`, `getBody`.
- `getBody` returns `*const Body` for read locks and `*Body` for write locks.
- Use them with init and a deferred deinit:

```zig
        var lock: BodyLockWrite = .init(self.body_lock_interface, body_id);
        defer lock.deinit();
```

**PhysicsLock.**
- `PhysicsLockContext = *const BodyManager`.
- `PhysicsLock.lock(mutex, io, context, lock_type)` and its relatives keep Jolt's lock order
  checks.
- `UniqueLock(T)` (init / deinit) replaces Jolt's `UniqueLock` / `SharedLock`.
- Jolt's `static thread_local` debug table is the one kind of per-thread state allowed in Phase 5,
  and it is `void` without asserts:

```zig
    threadlocal var locks: if (Core.enable_asserts) [4]LockData else void = if (Core.enable_asserts) @splat(.{}) else {};
```

`BodyAccess` (`Grant`, `GrantActiveBodiesAccess`) uses the same scheme.

### D4 PhysicsUpdateContext and the job graph

**Context and steps.**
- The context lives on `update`'s stack.
- `steps: TempArray(Step)` is Jolt's `Array<Step, STLTempAllocator<Step>>`. It is the first temp
  allocation, it is freed last, and it is never resized.
- `Step` keeps all of Jolt's fields and job handles, including the CCD fields and the soft body
  handles.
- `PhysicsUpdateContext.resizeHandles` default-initializes the handles, because
  `StaticArray.resize` leaves new elements undefined.
- Padded atomics are extern structs. Do not use `align(64)`, because temp blocks are only 16 / 32
  byte aligned:

```zig
const PaddedAtomicU32 = extern struct {
    value: std.atomic.Value(u32) = .init(0),
    padding: [Core.cache_line_size - @sizeOf(u32)]u8 = undefined,
};
```

**Builder.** `buildJobs` transliterates the "Build Jobs" block of `PhysicsSystem::Update`
statement by statement:
- Every lambda becomes an inline closure at the lambda's position.
- Captures map 1:1: `[&context, &step]` becomes `.{ context, step }`, `[&step, i]` becomes
  `.{ step, i }` and `[this, next_step]` becomes `.{ self, next_step }`. At most 4 words fit
  (`JobFunction.max_capture_size`).
- Keep every dependency count with its `// depends on:` comment, the creation order and the kick
  order:

```zig
            for (step.apply_gravity.slice()) |*h|
                h.* = try job_system.createJob("ApplyGravity", color_apply_gravity, .init(struct {
                    fn call(c: *PhysicsUpdateContext, s: *Step) void {
                        if (!c.isOutOfMemory())
                            c.physics_system.jobApplyGravity(c, s);

                        JobHandle.removeDependenciesStaticArray(&s.find_collisions, .{});
                    }
                }.call, .{ context, step }), .{ .num_dependencies = if (num_step_listener_jobs > 0) num_step_listener_jobs else previous_step_dependency_count }); // depends on: step listeners (or previous step if no step listeners)
```

**Start gate.**
- Jolt creates the first step's root jobs with 0 dependencies, so they run while the graph is
  still being built. Zolt gives them the 1 dependency that later steps have:

  ```zig
              // Zolt start gate: 1 for the first step too, `update` kicks the first step after building (kickStep)
              const previous_step_dependency_count: u32 = 1;
  ```
- After a successful build, `update` opens the gate with `kickStep(&context.steps.items[0])`. This
  is the same function that `StartNextStep` calls for the following steps.
- Results are unchanged, because Jolt's results do not depend on the schedule. The determinism
  tests compare state bytes per update across job systems.
- A failing `createJob` or temp allocation during the build has no side effects: no job ran, so
  the update returns `error.OutOfMemory` with an unchanged state (test "createJob fails at every
  job").

**Barrier.**
- `createBarrier() orelse @panic(...)` (Jolt crashes).
- Clone the handles in Jolt's order into a `StaticArray(JobHandle, max_physics_jobs)`, then
  `addJobs`, then `waitForJobs`. `finishUpdate` destroys the barrier.
- Jobs created inside jobs (`TrySpawnJobFindCollisions`, `FindCCDContacts`, and the soft body jobs
  in Phase 9) are added to the barrier with `step.context.barrier.addJob(&job)` and released with
  `defer job.deinit()`.

### D5 Errors: update, the out of memory latch, temp memory

**Signature.**

```zig
    pub fn update(self: *Self, delta_time: f32, collision_steps: u32, temp_allocator: TempAllocator, job_system: JobSystem) error{OutOfMemory}!PhysicsUpdateError {
```

- `PhysicsUpdateError` is a `packed struct(u32)` with Jolt's bits and `isNone()`.
- Jolt's final `JPH_ASSERT(errors == EPhysicsUpdateError::None)` becomes
  `if (Core.enable_asserts) std.debug.assert(errors.isNone())`.

**Build phase.**
- All temp allocations of the first step happen during the build, in Jolt's order: steps, body
  pairs, active constraints, the contact constraint buffer, the contact links.
- On failure, `update` runs `finishUpdate` (Jolt's tail: frees, unlocks), restores
  `previous_step_delta_time` and returns the error. The early-out path (no active bodies) unlocks
  and returns when `updatePrepare` fails.

**Inside jobs: the latch.** The latch lives in the context:

```zig
    /// Latch an allocation failure (D5)
    pub fn setOutOfMemory(self: *PhysicsUpdateContext) void {
        self.out_of_memory.store(true, .release);
    }

    /// True after an allocation failure: jobs skip their work
    pub fn isOutOfMemory(self: *const PhysicsUpdateContext) bool {
        return self.out_of_memory.load(.acquire);
    }
```

**The rule.** A poisoned job skips its work and its allocations. It always does its frees, its
commits and its dependency bookkeeping, so the barrier always drains:
- Job bodies that allocate are literal ports that return `Allocator.Error!void` with `try`.
- The closure wraps them and then does the lambda's bookkeeping unconditionally:

```zig
                    if (!c.isOutOfMemory())
                        c.physics_system.jobBuildIslandsFromConstraints(c, s) catch c.setOutOfMemory();

                    s.find_collisions.slice()[0].removeDependency(.{}); // The first collisions job cannot start running until we've finished building islands and activated all bodies
                    s.finalize_islands.removeDependency(.{});
```

**Protocol jobs.** These jobs carry protocol in their bodies, so the body always runs and checks
the latch itself:

| Job | When poisoned |
|---|---|
| BroadPhasePrepare | skips `updatePrepare`; the `UpdateState` stays zeroed (also when `updatePrepare` fails) |
| UpdateBroadPhaseFinalize | always runs `updateFinalize` (a commit); a zeroed state is a no-op |
| FindCollisions | takes no new batch and no new pair, then leaves through Jolt's "found nothing" exit (finalizes the ContactAllocator, clears its mask bit, removes dependencies) |
| PostIntegrateVelocity | takes the "no CCD bodies" path |
| ResolveCCDContacts | skips the work, always frees `active_body_to_ccd_body` and `ccd_bodies` |
| ContactRemovedCallbacks | discards the write cache instead of swapping (D10) |
| StartNextStep | always runs `resetIslands`, the splitter reset and the kick; skips `prepareContactConstraints` |
| SoftBodyPrepare | the active soft body list stays empty, so its "nothing to do" path kicks StartNextStep |

**Where to check the latch.** Only at job entry, or before a claim (an atomic batch counter, a
queue read, an island claim). Never check it between a claim and its completion. In particular,
never check it inside the LargeIslandSplitter batch protocol (fetch, process, mark done): other
SolveVelocity jobs wait for the processing count to drain, and leaving early hangs them.

**Temp memory.** `TempArray(T)` is added to `Core/TempAllocator.zig` in F1. An empty array means
"nothing allocated": the allocation failed, was skipped after poisoning, or had size 0. `free`
skips empty arrays:

```zig
        /// Free the block (Jolt: inTempAllocator->Free(p, n * sizeof(T))), nothing for an empty array
        pub fn free(self: *Self, temp_allocator: TempAllocator) void {
            if (self.items.len != 0)
                temp_allocator.free(@ptrCast(self.items.ptr), @intCast(self.items.len * @sizeOf(T)));
            self.* = .empty;
        }
```

- Every free stays at Jolt's position and is never skipped: ResolveCCDContacts, StartNextStep and
  `finishUpdate` (`IslandBuilder.resetIslands` in reverse order,
  `ContactConstraintManager.finishConstraintBuffer`, `LargeIslandSplitter.reset`), and
  SoftBodyFinalize in Phase 9. LIFO order then holds by construction, also after a failure, and
  the tests assert `temp.isEmpty()` after every failing update.
- `IslandBuilder.finalize` restores the body links. When the update was poisoned before or
  during finalize, `resetIslands` restores them instead (`links_need_reset`).

**Failed spawns are not errors.** A job that cannot be created from inside a job changes no
result:

```zig
                }.call, .{ step, job_index }), .{}) catch {
                    // Zolt (D5): not an error, undo the claim and the dependencies, the running jobs do the work
                    _ = step.active_find_collision_jobs.fetchAnd(~job_mask, .release);
                    step.update_broadphase_finalize.removeDependency(.{});
                    step.finalize_islands.removeDependency(.{});
                    return;
                };
```

- PostIntegrateVelocity removes the dependencies of the FindCCDContacts jobs it could not
  create.
- When no FindCCDContacts job could be created at all, PostIntegrateVelocity does the work itself,
  because the created jobs take bodies until `next_ccd_body` runs out.

**Heap allocation inside a job.** `BroadPhase.updatePrepare` is the one heap allocation inside a
job (QuadTree's `new NodeID[]`, D9). It is latched:

```zig
                    if (!c.isOutOfMemory())
                        s.broad_phase_update_state = c.physics_system.broad_phase.updatePrepare() catch blk: {
                            c.setOutOfMemory();
                            break :blk .{};
                        };
```

**Contract after `error.OutOfMemory` from a job.**
- The state is mid-step but memory safe. No lock is held, nothing leaked, and the temp allocator
  is empty.
- The caller restores a saved state or deinits the system. The doc comment of `update` says so.
- The build-phase failure is the only one that leaves the state unchanged.

### D6 BodyInterface and BodyCreationSettings

**BodyInterface** is `{ body_lock_interface: *const BodyLockInterface, body_manager: *BodyManager,
broad_phase: *BroadPhase }`. PhysicsSystem holds two instances (locking and no-lock), set up in
place. Each function keeps Jolt's body and lock scope. Signatures:

| Jolt | Zolt |
|---|---|
| `Body *CreateBody(settings)` (nullptr: out of bodies) | `createBody(settings) CreateBodyError!?*Body`; `CreateBodyError = error{InvalidShape} \|\| Allocator.Error` (Jolt crashes on a missing shape) |
| `BodyID CreateAndAddBody(settings, activation)` | `CreateBodyError!BodyID`; invalid ID when out of bodies; `errdefer` destroys the body |
| `void AddBody(id, activation)` | `Allocator.Error!void` (QuadTree allocates in AddBodiesPrepare) |
| `AddState AddBodiesPrepare(ids, n)` | `Allocator.Error!BroadPhase.AddState`; Finalize / Abort never fail |
| `TwoBodyConstraint *CreateConstraint(settings, id1, id2)` | `Allocator.Error!*TwoBodyConstraint`, `Body.fixedToWorld()` for an invalid body |
| `RefConst<Shape> GetShape(id)` | `RefConst(Shape)` that the caller owns (`deinit`) |
| defaulted `EActivation inActivationMode = Activate` | an options struct: `addForce(id, force, .{ .activation = .activate })` |
| several out parameters (`GetPositionAndRotation`) | a returned struct |
| pointer + count | a slice |

Getters return Jolt's defaults when the lock fails. `createAndAddBody` shows the error channels:

```zig
    pub fn createAndAddBody(self: *const BodyInterface, settings: *const BodyCreationSettings, activation_mode: Activation) CreateBodyError!BodyID {
        const body = (try self.createBody(settings)) orelse return .invalid; // Out of bodies
        const id = body.getID();
        errdefer self.destroyBody(id);
        try self.addBody(id, activation_mode);
        return id;
    }
```

**BodyCreationSettings.** Jolt's private, mutually exclusive `mShape` and `mShapePtr`
(BodyCreationSettings.h:122-125) become a tagged union. It holds a `Ref`, because creating the
shape writes the settings' cache (Rule M, CollisionArchitecture D3):

```zig
    pub const ShapeSource = union(enum) {
        none,
        /// `Ref`, not Jolt's RefConst: creating the shape writes the settings' cache (Rule M, CollisionArchitecture D3)
        settings: Ref(ShapeSettings),
        shape: RefConst(Shape),
    };
```

- `getShape(self: *const BodyCreationSettings, allocator) Allocator.Error!?*const Shape` keeps
  Jolt's semantics. It returns null for `.none`. On a creation error it logs, panics when asserts
  are on, and returns null.
- `convertShapeSettings` keeps Jolt's texts ("No shape present!").
- `clone` / `deinit` copy and release the references.
- The binary state is byte-identical to Jolt's. `saveWithChildren` / `restoreWithChildren` use the
  Phase 4 ID maps and return a `Result` value.

### D7 Listeners

CollisionArchitecture D7 applies unchanged: pattern B, `ptr: *anyopaque`, non-null entries for pure
virtual functions, nullable entries for Jolt's empty defaults, stored by value. Every doc comment
states that the callbacks run concurrently on worker threads. Phase 5 adds:
- `BodyActivationListener` lives in BodyManager and is called with the active bodies mutex locked.
- The Phase 4 `ContactListener.zig` switches to the real `Body` in F1.
- `PhysicsStepListener.onStep(context)` listeners are kept in an ArrayList under
  `step_listeners_mutex`. Adding one returns `Allocator.Error`; removing one finds it by `ptr` and
  swap-removes it, like Jolt.
- `SoftBodyContactListener` is the real interface already.
- `CombineFunction` stays a function pointer.
- `SimCollideBodyVsBody` (a `std::function` in Jolt) becomes `{ function, context }`.

### D8 Constraints

**Pattern A** with `virtual.make`, one level per C++ class, the same mechanism as Shape:
- `Constraint` is a RefTarget root holding Jolt's data. Its VTable follows Jolt's declaration
  order. `buildIslands(index, *IslandBuilder, *BodyManager)` and `buildIslandSplits` are entries.
  `getConstraintSettings` returns `Allocator.Error`.
- `TwoBodyConstraint` extends the VTable with the two matrix functions and keeps its abstract
  bodies (`getType`, `isActive`, `buildIslands`) in `impl`.

**Settings.** Jolt's virtual `TwoBodyConstraintSettings::Create(Body &, Body &)` is renamed,
because `create(allocator, ...)` is the guide's `new XSettings`:

```zig
        createConstraint: *const fn (self: *const TwoBodyConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint,
```

A concrete settings class lists it in `overrides`:

```zig
    pub const overrides = .{.createConstraint};
```

**ConstraintManager.**
- `ArrayList(Ref(Constraint))` with Jolt's swap-remove indices.
- Its mutex lives in `mutable_fields`.
- `add` reserves first, so it is all or nothing:

```zig
        try self.constraints.ensureUnusedCapacity(self.allocator, constraints.len);
```

- `sortConstraints` (priority, then index) uses `zolt.quickSort`.

**Constraint parts and Phase 6.**
- ConstraintPart files are value structs. Jolt's template motion types become comptime parameters
  with `inline else` dispatch.
- `SpringSettings` / `MotorSettings` keep a byte-compatible binary state.
- Phase 6 concrete classes declare `overrides`. Their settings types go into a comptime
  `constraint_settings_types` list until the Phase 8 Factory.

### D9 Broad phase

**Hierarchy.** Pattern A, `BroadPhaseQuery` → `BroadPhase` → `BroadPhaseBruteForce` /
`BroadPhaseQuadTree`.
- `BroadPhase.VTable` starts with `base: BroadPhaseQuery.VTable` and keeps Jolt's default bodies in
  `impl`.
- The root has no `ref_count`. PhysicsSystem owns `broad_phase: *BroadPhase` and calls
  `self.broad_phase.base.destroy()` (`delete mBroadPhase`). The generated `destroy` skips the
  reference count check for such roots: Virtual.zig was changed with this document.
- `PhysicsSystem.init` takes a trailing options struct, `.{ .broad_phase = .quad_tree }`
  (`.brute_force` for tests), instead of Jolt's `#if`. F2 defaults to `.brute_force`, and A1
  switches the default.
- Jolt's `Init` is `initBroadPhase`, because `init` is the constructor.

**Queries** take `*const` receivers and defaulted filters as options. The query locks are in
`mutable_fields`:

```zig
    pub fn collideAABox(self: *const BroadPhaseQuery, box: *const AABox, collector: *CollideShapeBodyCollector, opts: struct { broad_phase_layer_filter: *const BroadPhaseLayerFilter = &.{}, object_layer_filter: *const ObjectLayerFilter = &.{} }) void {
```

**Allocation.** QuadTree allocates with `new` in two places, so two functions return errors:
- `addBodiesPrepare(bodies) Allocator.Error!AddState`: `new LayerState[mNumLayers]`
  (BroadPhaseQuadTree.cpp:168) and `BuildTree`'s `new Vec3[]` (QuadTree.cpp:544).
- `updatePrepare() Allocator.Error!UpdateState`: `new NodeID[mNumBodies]` (QuadTree.cpp:300) and
  the STLLocalAllocator stack with its heap fallback (:304).

On error, the function frees what it built (arrays, and nodes back to the free list) and leaves
the tree unchanged. A zeroed `UpdateState` makes `updateFinalize` a no-op
(BroadPhaseQuadTree.cpp:143):

```zig
    pub const UpdateState = extern struct {
        data: [4]?*anyopaque = @splat(null),
    };
```

**Node pages.**
- QuadTree commits all node pages in `initBroadPhase`
  (`FixedSizeFreeList.reserveAllPages`, added in F1).
- `addBodiesFinalize` → `TryCreateNewRoot` → `AllocateNode` (QuadTree.cpp:772) then cannot fail
  on memory. Finalize and Abort return `void`.
- Running out of nodes stays Jolt's abort: `@panic("QuadTree: Out of nodes!")`.

**QuadTree port.**
- Port QuadTree literally, including the partition code and the double-buffered root swap.
- Node bounds are `[4]std.atomic.Value(f32)` per axis. `NotifyBodiesAABBChanged` widens them with
  compare-exchange. Readers load them per lane with `.monotonic` into a Vec4, which gives the same
  bits as Jolt's vector load without the race. A racy vector load is only an option after
  measurement.
- The pair order of `findCollidingPairs` may vary between runs. Determinism comes from
  `findCollidingPairsCanCollide` plus the contact sorts.
- `frameSync` discards the old tree under the old query lock (lock type `.broad_phase_query`).

### D10 ContactConstraintManager

**Layout.** It is a literal port on `LockFreeHashMap`. The cache types are extern structs with
Jolt v5.6.1's layout (ContactConstraintManager.h:274-327), with comptime size and alignment
checks:

| Type | Fields | Size |
|---|---|---|
| `CachedContactPoint` | `position1`, `position2` (Float3), `non_penetration_lambda` | 28 |
| `CachedManifold` | ..., `friction_lambda: Vector(2)`, `angular_friction_lambda`, flags | 60 |
| `CachedBodyPair` | | 28 |
| `BodyPair` | `body_a`, `body_b` | 8 |

**Access.**
- Flexible contact points use `getRequiredExtraSize`.
- `LockFreeHashMap.findMut` (F1) replaces Jolt's `const_cast` when setting the manifold flags. Do
  not widen Rule M.
- `sLoadFloat3Unsafe` is Zolt's `Vec3.loadFloat3Unsafe`, which reads the three floats and never
  past the end of the array. The x, y and z bits are identical.

**Update data.**
- The ManifoldCache is double buffered.
- Each FindCollisions job has a `ContactAllocator` on its stack. It is folded into the step with
  `fetchAdd` / `fetchOr`. Running out of space sets Jolt's soft error flags.
- The constraint buffer comes from the TempAllocator (a `TempArray`).
- Motion type specialization uses `inline else`.
- `SortContacts` and `GetAll*Sorted` use `zolt.quickSort` with Jolt's comparators. Listener calls
  stay at Jolt's call sites.

**Body pair queue slots** are 8-byte atomics (Jolt's TSAN-suppressed read before the CAS):

```zig
fn storeBodyPair(slot: *BodyPairSlot, pair: BodyPair) void {
    slot.store(std.mem.bytesToValue(u64, std.mem.asBytes(&pair)), .monotonic);
}
```

**Out of memory.** `finalizeContactCacheAndCallContactPointRemovedCallbacks` swaps a write cache
that a poisoned update left incomplete. Calling it would report spurious removed contacts, and a
later `restoreState` would insert into a cache that is not empty. A poisoned ContactRemovedCallbacks
job therefore calls `finalizeContactCacheAfterOutOfMemory()` (a Zolt addition):
- It clears the write cache (`ManifoldCache.clear`, keeping its bucket count).
- It keeps the read cache and calls no listener.
- Both caches stay structurally valid, and `restoreState` works.

The A4 worker proves this with the FailingTempAllocator sweep on a scene with contacts.

### D11 StateRecorder and DeterminismLog

**StateRecorder.**
- It is a pattern A root
  `{ vtable{ readBytes, isEOF, isFailed, writeBytes }, is_validating, is_last_part }`.
- `streamIn()` / `streamOut()` hand out the Phase 2 pattern B streams, so `write` / `read` produce
  Jolt's bytes and every Phase 4 `saveBinaryState(StreamOut)` accepts a recorder.
- `StateRecorderImpl` is backed by an `ArrayList(u8)`. A failed write sets `failed`, like a
  stringstream that cannot grow. A validation mismatch logs every differing byte, counts it in
  `num_mismatches` and panics when asserts are on (Jolt: `JPH_BREAKPOINT`).
- `StateRecorderFilter` is pattern A with accept-all defaults.
- `StateRecorderState` is a `packed struct(u8)` with Jolt's bits.

**Save and restore.**
- `saveState(stream, state, filter) Allocator.Error!void`, because it builds sorted arrays.
- `restoreState(stream) Allocator.Error!bool`, keeping Jolt's `bool`.
- Jolt's validating restore path is ported literally.

**DeterminismLog.**
- It sits behind the build option `-Ddeterminism_log`, which is off by default. The prototype uses
  `pub const enabled = false;`.
- The module-level writer exists only under the option, because the `JPH_DET_LOG` call sites in
  MotionProperties (MotionProperties.h:230, 241) have no system pointer:

```zig
/// DeterminismLog::sLog: where the log goes (set by the application). `void` when the log is disabled.
pub var writer: if (enabled) ?*std.Io.Writer else void = if (enabled) null else {};
```

- `JPH_DET_LOG(a << b)` becomes `if (DeterminismLog.enabled) DeterminismLog.log(.{ a, b });`.
- The output is Jolt's text: floats as hex bits, BodyIDs as 8 hex digits. Like Jolt's log, it is
  only meaningful single threaded.

### D12 Tests and parity

**PhysicsTestContext** (`ZoltTests/PhysicsTestContext.zig`) is initialized in place and owns a
TempAllocatorImpl, a JobSystemThreadPool, the Layers and the PhysicsSystem.

**Simulation parity** lives in `ZoltParity/Physics/SimulationReference.cpp` and
`SimulationParity.zig`, built on shared scene data (`SimScenes`). The C ABI has no `bool`
parameters.
- After every step, compare the `saveState(.all)` bytes. On a mismatch, report the first differing
  offset decoded to (section, body, field).
- Compare the listener callbacks as sorted records.
- Thread matrix: Zig `JobSystemSingleThreaded` and pools of 0, 1, 3 and 7 workers against C++ with
  0 and N threads. Run Debug and ReleaseFast, single and double precision.
- Optionally check that Zolt's temp memory use is at most Jolt's.

**Mandatory per scene test:**
- A FailingTempAllocator sweep (every allocation fails once; `temp.isEmpty()`; recovery through
  `restoreState`).
- A FailingJobSystem sweep for files that create jobs.
- `checkAllAllocationFailures` on creation.
- A ReleaseFast run.

## 2. Phase 5 port order (D13)

About 62 files, one per Jolt header at the same paths (guide section 3), plus the soft body stubs.

**F1 Foundation, part 1** (one worker; nothing else starts before it is merged):
- Core:
  - `FixedSizeFreeList.reserveAllPages`, `LockFreeHashMap.findMut` and `TempArray(T)` in
    `TempAllocator.zig`, each with tests.
  - The Virtual.zig `destroy` change and `Shape.is_static` / `EmptyShape.initStatic` are already
    done with this document.
- `Physics/Body`: MotionType, MotionQuality, BodyType, AllowedDOFs, BodyAccess, BodyPair,
  BodyFilter, BodyActivationListener, MotionProperties, Body (complete, D1), BodyCreationSettings
  (without binary state), BodyManager, BodyLockInterface, BodyLock, BodyLockMulti.
- `Physics`: EActivation, PhysicsLock, StateRecorder, StateRecorderImpl.
- The soft body stubs (D1). Switch `ContactListener.zig` to the real Body.
- The guide updates (section 6).
- Tests: the prototype's Body, BodyManager and lock tests, adapted.

**F2 Foundation, part 2** (worker 1; worker 2 builds the parity harness in parallel):
- `BroadPhaseQuery`, `BroadPhase`, `BroadPhaseBruteForce`.
- `Constraint`, `TwoBodyConstraint`, `ConstraintManager`.
- `EPhysicsUpdateError`, `PhysicsStepListener`, `PhysicsUpdateContext`.
- `IslandBuilder`, moved here from Wave A because the prototype's port is nearly complete and the
  gravity-only scene needs it.
- `PhysicsSystem` with the final builder, gate, latch and TempArray discipline (D4, D5). The job
  bodies on the gravity-only path are real: ApplyGravity, IntegrateVelocity without CCD,
  FinalizeIslands, and the CheckSleepAndUpdateBounds part of SolvePositionConstraints. All other
  job bodies are stubs, each marked with its region (`// REGION R1` ... `// REGION R5`, Wave B).
- The core of `BodyInterface` (create, destroy, add, remove, activate, position and velocity,
  createConstraint) and `ZoltTests/PhysicsTestContext.zig`.
- Worker 2: `SimulationReference.cpp`, `SimulationParity.zig` and SimScenes, with the gravity-only
  scene bit exact end to end (D12).
- At the end of F2, remove `ZoltTests/Prototype/` and its line in `unit_tests.zig`.

**Wave A** (parallel, after F2):

| Worker | Files | Needs |
|---|---|---|
| A1 | `QuadTree`, `BroadPhaseQuadTree` (D9), BroadPhaseTests; switch the PhysicsSystem default | F2 |
| A2 | Contact path parts: `SpringPart`, `AxisConstraintPart`, `AngularFrictionConstraintPart`, `ContactConstraintPart`, `DualAxisConstraintPart`, `AngleConstraintPart`, plus `SpringSettings`, `CalculateSolverSteps` | F2 |
| A3 | The other parts: `PointConstraintPart`, `HingeRotationConstraintPart`, `RotationEulerConstraintPart`, `RotationQuatConstraintPart`, `SwingTwistConstraintPart`, `IndependentAxisConstraintPart`, `GearConstraintPart`, `RackAndPinionConstraintPart`, plus `MotorSettings` | F2 |
| A4 | `ContactConstraintManager` (D10), the critical path; it starts when A2's contact parts land | A2 |
| A5 | `LargeIslandSplitter` | F2, A2 (CalculateSolverSteps) |
| A6 | The rest of `BodyInterface`, the BodyCreationSettings binary state, `saveWithChildren` | F2 |
| A7 | `NarrowPhaseQuery`, `EstimateCollisionResponse`, `SimShapeFilterWrapper` | F2 |
| A8 | `DeterminismLog` (build option), `PhysicsScene` | F2, A6 |

**Wave B:** the PhysicsSystem job bodies, one worker per region. The builder is final since F2,
so the regions do not interact:

| Region | Jobs | Needs |
|---|---|---|
| R1 | StepListeners, DetermineActiveConstraints, SetupVelocityConstraints, BuildIslandsFromConstraints, the rest of IntegrateVelocity, PostIntegrateVelocity | F2 |
| R2 | FindCollisions, ProcessBodyPair, TrySpawnJobFindCollisions | A4 |
| R3 | FinalizeIslands with the splitter, SolveVelocityConstraints, SolvePositionConstraints, CheckSleepAndUpdateBounds, BodySetIslandIndex | A4, A5 |
| R4 | CCD: FindCCDContacts, ResolveCCDContacts, sSolveCCDContact | A4, A7 |
| R5 | SaveState / RestoreState, OptimizeBroadPhase, the remaining PhysicsSystem API, the SoftBodyPrepare empty path | A1, A4 |

**Wave C** (after Wave B):
- The tests that need PhysicsTestContext: PhysicsTests, ContactListenerTests, SensorTests,
  PhysicsStepListenerTests, PhysicsDeterminismTests, MotionQualityLinearCastTests,
  EstimateCollisionResponseTest, and the `//! Missing:` cases of the Phase 4 test files.
- M5 and M6 (section 3), and the determinism CI job.

## 3. Milestones

**M5 HelloWorld.**
1. `ZoltHelloWorld/HelloWorld.zig` ports `HelloWorld/HelloWorld.cpp` statement by statement:
   - the same constants (1024 bodies, 0 mutexes, 1024 body pairs, 1024 contact constraints, a
     10 MB temp allocator, 60 Hz, 1 collision step);
   - the same listeners, which print Jolt's texts;
   - `optimizeBroadPhase`;
   - the loop while the sphere is active.

   It is built and run by `zig build hello-world`. `RegisterTypes` / `Factory` are comptime in
   Zolt.
2. `ZoltParity/Physics/HelloWorldParity.zig` runs the HelloWorld scene in C++ (through the
   SimulationReference C ABI) and in Zolt. After every step it compares the position and velocity
   bits, the `saveState` bytes and the callback records. This is the acceptance test of M5.
3. Text equality of the two programs' stdout needs a C++-compatible float formatter (open
   question). Until it exists, M5 is judged on step 2.

Needed: everything up to Wave B (sleeping, the contact cache with persisted and removed callbacks,
QuadTree).

**M6 Pyramid.**
1. `ZoltPerformanceTest/` ports `PerformanceTest.cpp` with its arguments (`-s`, `-q`, `-t`, `-i`,
   `-no_sleep`, `-rs`, `-vs`, `-validate_hash`, `-repeat`), `PerformanceTestScene` and
   `PyramidScene`. The hash is FNV-1a
   (`HashBytes`) over the position and rotation of every body in body ID order after 500
   iterations, plus the scene's `UpdateHash`.
2. CI runs `zig build performance-test -Doptimize=ReleaseFast -- -q=LinearCast -t=max -s=Pyramid
   -validate_hash=0x74d0118836ac0892`, and the same with `-Ddouble_precision=true` and
   `0xf07f04597a102a82`. These are exactly the arguments of `determinism_check.yml`. CI also runs
   `-t=1` and `-t=2`: the hash must not depend on the thread count.
3. PyramidScene ignores the motion quality and sets `mAllowSleeping = false`. M6 therefore needs
   the QuadTree, the full ContactConstraintManager, IslandBuilder, LargeIslandSplitter (one large
   island) and CalculateSolverSteps, but not CCD and not sleeping.
4. To find a divergence: a `PyramidParity` scene in SimulationParity compares the `saveState`
   bytes after every step for the first 50 steps, and the first differing field names the culprit.

**Status.**
- M5 is reached. `zig build hello-world-check` runs the C++ HelloWorld and its Zig port and fails unless
  their stdout is byte identical; it is part of `zig build parity`. HelloWorldParity and the
  `contacts_hello_world` scene compare the state bits after every step.
- M6 is reached, and goes beyond the plan above. ZoltPerformanceTest reproduces Jolt's published
  hashes from `determinism_check.yml` for Pyramid, ConvexVsMesh and HighSpeed (`-q=LinearCast`). This
  holds in both precisions and with `-t=max`, `-t=1` and `-t=2`. Zolt's CI runs them in its
  determinism job. Ragdoll and CharacterVirtual need Phases 7 and 8.

## 4. Checklist for porters

- [ ] One file per Jolt header, with the `//! Port of:` and `//! Status:` header, registered in
  `zolt.zig`. Jolt's doc and implementation comments are kept.
- [ ] Rule M:
  - `mutable` members go in `mutable_fields`; queries keep `*const` receivers;
  - no `@constCast` writes, the only exception being `Body.fixedToWorld()`;
  - `findMut` instead of `const_cast`.
- [ ] Jolt's TSAN-suppressed races are `.monotonic` atomics. Every other atomic keeps Jolt's
  memory order.
- [ ] Allocating functions return `Allocator.Error` (or `CreateBodyError`). Jolt's soft errors
  stay values. A crash in Jolt is a `@panic` with Jolt's text.
- [ ] In PhysicsSystem:
  - a lambda is an inline closure at its C++ position, with Jolt's captures, dependency counts and
    comments;
  - job bodies return `Allocator.Error!void` and are wrapped;
  - protocol jobs follow the D5 table;
  - latch checks only at job entry or before a claim.
- [ ] Temp memory uses `TempArray`, with frees at Jolt's positions and never skipped. A spawn
  failure is not an error.
- [ ] No `align` above 16 on anything that lives in temp memory (padded extern structs instead).
- [ ] Sorting uses `zolt.quickSort` / `zolt.insertionSort` with Jolt's comparators.
- [ ] Defaulted parameters become options structs. Overloads get suffixes
  (`lockRead` / `lockReadMask`). The rename table (section 6) is followed.
- [ ] Debug-only state is typed `void` without asserts.
- [ ] Tests:
  - the Jolt unit tests with the same names and values;
  - SimulationParity scenes for what the file computes;
  - the FailingTempAllocator / FailingJobSystem / `checkAllAllocationFailures` sweeps;
  - ReleaseFast and double precision runs.
- [ ] `zig build test` (both precisions), `zig build parity`, `zig fmt --check`,
  `python3 tools/port_status.py --write`.

## 5. Judge's critical flaws and how this design resolves them

| Flaw | Resolution |
|---|---|
| QuadTree heap allocations had no error path (AddBodiesPrepare `new LayerState[]`, BuildTree `new Vec3[]`, UpdatePrepare `new NodeID[]` and the STLLocalAllocator fallback, inside the BroadPhasePrepare job) | `addBodiesPrepare` returns `Allocator.Error!AddState`. `updatePrepare` returns `Allocator.Error!UpdateState`, cleans up a half-built tree, and its job latches the failure. A zeroed state is a no-op in Finalize. Node pages are committed at init, so Finalize / Abort cannot fail. Running out of nodes panics as Jolt aborts (D9). Prototype test "out of heap memory inside a job" |
| P2's job graph was not Jolt's DAG (BodySetIslandIndex, ResolveCCDContacts, SoftBodyPrepare missing; wrong counts; no LockModifications) | The prototype builds Jolt's graph statement by statement, with Jolt's counts, creation and kick order (checked against PhysicsSystem.cpp), and calls LockModifications in both paths (D4) |
| P2 solved Rule M with mutable receivers on queries and BodyManager getters | `mutable_fields` blocks. `collideAABox`, `getNumBodies`, `lockAllBodies`, `getActiveBodies` and `saveState` keep `*const` receivers (D2, D3, D9) |
| P3 had no start gate, so a `createJob` failure panicked with every lock held | The start gate with `kickStep` (D4). Test "createJob fails at every job" in single and multi threaded mode: build failures change nothing, spawn failures change no result |
| P3's D10 used the pre-v5 cache layout (56 bytes against its own `== 60` assert) | The v5.6.1 layout with comptime size checks (D10) |
| P3 read QuadTree bounds with a racy 16-byte vector load by default | Per-lane `.monotonic` atomic loads by default, with identical bits (D9) |
| The settings virtual `create` clashed with the constructor `create(allocator, ...)` | It is renamed `createConstraint(allocator, body1, body2)` (D8). The prototype's settings use it |
| `fixedToWorld()` was read-only, but its shape was reference counted (any `RefConst` on it faulted); P3 had no shape at all | `Shape.is_static` and `EmptyShape.initStatic`: Jolt's dummy EmptyShape, never counted. Tests in EmptyShape.zig and the prototype (D1) |
| P1 moved the frees of skipped jobs into a hand-maintained tail list | TempArray is empty on failure, and frees stay unconditional at Jolt's positions, so LIFO holds by construction. Every failing update asserts `temp.isEmpty()` (D5) |
| P1 failed the update when TrySpawnJobFindCollisions' `createJob` failed | A spawn failure undoes the claim and the dependencies and is not an error. FindCCDContacts too (D5) |
| P1 and P2 kept Jolt's TSAN-suppressed races as plain accesses | `bodies_size`, `index_in_active_bodies` and the body pair slots are atomics (rule 3, D2, D10) |
| P3's per-system DeterminismLog could not reach MotionProperties' `JPH_DET_LOG` | A module-level writer under the build option, `void` otherwise (D11) |
| P2 checked the abort latch inside the splitter's batch and spin loops (risk of a hang) | Latch checks only at job entry or before a claim, never inside the LargeIslandSplitter batch protocol (D5) |

## 6. Porting guide updates (done in F1)

- **Section 5, Rule M.** Add the Phase 5 patterns: `mutable_fields` blocks for `mutable` mutexes,
  `findMut` for `const_cast` of hash map results, and the audited `Body.fixedToWorld()`.
- **Section 5, Allocation.** Add `TempArray(T)` and the D5 rules: the latch in jobs, frees never
  skipped, a spawn failure is not an error, the contract after `error.OutOfMemory`.
- **Section 5, Reference counting.** Add `Shape.is_static` next to the material's `is_static`.
- **Section 6.** A closed set of `final` classes (BodyLockInterface) may be a tagged struct with an
  exhaustive switch. A pattern A root without `ref_count` is owned through a plain pointer and
  freed with `destroy`.
- **Section 7.** C++ lambdas passed to the job system become inline closures
  `.init(struct { fn call(...) void { ... } }.call, .{ captures })` at the lambda's position.
  `JPH_DET_LOG` becomes `if (DeterminismLog.enabled) DeterminismLog.log(.{ ... })`.
- **Section 9.**
  - Jolt's TSAN-suppressed races become `.monotonic` atomics.
  - Padded atomics in temp memory are extern structs with explicit padding.
  - Debug-only `thread_local` state is a `threadlocal var` typed `void` without asserts.
- **Bit flags.** Public flag enums become packed structs with Jolt's bits and named combinations.
  Private flags changed with atomics become integer constants.
- **Rename table:**

  | Jolt | Zolt |
  |---|---|
  | `TwoBodyConstraintSettings::Create` | `createConstraint` |
  | `BroadPhase::Init` | `initBroadPhase` |
  | `Body::sFixedToWorld` | `Body.fixedToWorld()` |
  | `LockRead(MutexMask)` | `lockReadMask` |
  | `mBodies.size()` | `bodies_size` |
  | `Body::EFlags` | `Body.Flags` |
  | `mShape` / `mShapePtr` | `shape: ShapeSource` |
  | `EActivation` defaults | `.{ .activation = ... }` |
  | `BodyLockInterfaceLocking` / `NoLock` | `BodyLockInterface.initLocking` / `initNoLock` |

## 7. Questions settled after the review

- **Barrier exhaustion.** `createBarrier` returning null stays `@panic` (Jolt crashes). It means too
  many concurrent updates on one job system, a configuration error and not an allocation failure.
- **Text equality for M5.** Add a C++-compatible float formatter (`std::ostream` default, `%g` with
  precision 6) to `Core/StringTools.zig` in Wave C. M5 needs both the bit parity of HelloWorldParity
  and equal stdout. ConvexHullShape's error texts switch to the formatter too.
- **QuadTree pre-commit.** Accepted. About 170 bytes per max body is what Jolt itself uses once the
  tree is full (the allocator holds twice the estimated node count for the rebuild in `Update`),
  and it keeps Finalize and Abort free of errors. The `initBroadPhase` doc comment states the cost.
- **Per-lane bound loads.** Kept. PerformanceTest measures them after M6; a racy vector load needs
  that measurement first.
- **Poisoned contact cache.** A4 confirms `finalizeContactCacheAfterOutOfMemory` against Jolt's
  restore path with the FailingTempAllocator sweep on a scene with contacts (D10).
- **Snapshot on failure.** No internal snapshot. The contract of D5 stands: after
  `error.OutOfMemory` from a job the caller restores a saved state or deinits the system.
- **DeterminismLog threading.** Not forced. Like Jolt's, the log is only meaningful with a single
  threaded job system, and its doc comment says so.
- **BodyAccess.** F1 ports the grants (D3) and checks them with the unit tests that use them.

## 8. Settled during the port

**F1 (Core additions, Body package).**
- `LockFreeHashMap.findMut` takes a mutable map (`self: *Self`). Every place Jolt writes through a
  `const_cast` of a `Find` result is inside a non-const ContactConstraintManager function, so A4
  calls `findMut` on its non-const cache pointers.
- `FailingTempAllocator` (in `TempAllocator.zig`) and `FailingJobSystem` (in `JobSystem.zig`) are
  public Zolt test helpers. `FailingTempAllocator` fails exactly the n-th call, size 0 calls
  included.
- `TempArray.alloc` returns `error.OutOfMemory` when `n * @sizeOf(T)` does not fit in a `u32`
  (Jolt truncates the size there).
- Non-const C++ overloads of getters are named `...Mut`: `getMotionPropertiesMut`, `getBodyMut`,
  `tryGetBodyMut`, `getBodyInterfaceMut`.
- `SharedLock(T)` exists next to `UniqueLock(T)`, because Jolt has both.
- BodyManager's `bodies_cache_invalid_mutex` is in `mutable_fields` too: it is locked from const
  paths.
- A thread pool that a test fills must have room for every job it creates: the queue keeps a
  reference to each job until a worker takes it, so jobs that already ran through the barrier
  still occupy a slot.

**F2 (PhysicsSystem core, constraint parts, simulation parity).**
- The F2 port covers all of REGION R1, not only the gravity-only job bodies. The CCD broad phase
  cast and the no-hit resolve path are real, so linear cast bodies without hits are bit exact.
  REGION R2, the island solve of R3 and the hit path of R4 are stubs that `@panic` where Jolt
  would do work, so a result is never silently wrong.
- Debug grants: JobFindCollisions and JobSoftBodyPrepare remove their dependencies after the scope
  of their BodyAccess grants (Jolt: inside it). With JobSystemSingleThreaded the kicked jobs run
  inside the call, and their grants assert that no grant is active. Jolt hits that from the
  second collision step on, Zolt with the start gate already in the first. Results are
  unchanged.
- `PhysicsSystem.init` options: `.broad_phase = .brute_force` until A1, or `.custom = *BroadPhase`
  (a Zolt addition; the system owns it after a successful init).
- `PhysicsUpdateContext` is complete. The ContactConstraintManager stub writes Jolt's bytes for
  an empty contact cache (two `u32` zeros).
- `zolt.test_constraints` (`Zolt/Physics/Constraints/TestConstraints.zig`) is a test-only
  TwoBodyConstraint made with `virtual.make`, registered through the test `zolt_user_types`. The
  constraint part inline tests use `ConstraintPart/ConstraintPartTestContext.zig` (test only).
- Constraint parts:
  - The templated contact parts `ContactConstraintPart(T1, T2)` and
    `AngularFrictionConstraintPart(T1, T2)` are extern structs with Jolt's layout, because A4
    stores them in a raw byte buffer. Their size is checked at comptime and against C++
    `sizeof`.
  - Template specializations that add members become `void` fields, and multiple inheritance
    becomes `base1` / `base2` fields plus forwarders.
  - `SpringSettings.frequency` doubles as Jolt's anonymous union with `mStiffness`
    (`getStiffness` / `setStiffness`).
  - `SpringMode` and `MotorState` accept every value of their underlying integer, as Jolt's enums
    do, so restoring an out-of-range byte is not illegal behavior.
- Jolt asserts that a kinematic body never reaches the Gear, RackAndPinion and IndependentAxis
  parts or AngularFriction without a dynamic body. Zolt asserts the same with asserts on and
  matches Jolt's release behavior in ReleaseFast (covered by the ReleaseFast parity).
- Simulation parity (`ZoltParity/Physics/SimulationParity.zig`, `SimulationReference.cpp`,
  `SimScenes.zig`):
  - Zolt and the C++ PhysicsSystem step in lockstep and are compared after every update, not every
    collision step: SaveState cannot run inside an update.
  - The thread matrix pairs Zolt single threaded and pools of 0, 1, 3 and 7 workers with C++
    pools of 0, 3 and 7 threads.
  - Scenes that need a later wave are marked blocked and are skipped visibly
    (`error.SkipZigTest`). Their C++ side is already checked on its own.
  - Every scene asserts that it reaches the edge cases it is meant to cover.
  - The C++ reference always uses the QuadTree (Jolt's `BROAD_PHASE` define). Gravity-only scenes
    do not depend on it; contact scenes need A1.
- PhysicsSystem's linear cast inner radius assert only runs with asserts on (PortingGuide
  section 7). With a mirrored ScaledShape Jolt's inner radius is negative and its debug build
  asserts; the release-only scene compares the release behavior.

**Wave A (QuadTree, ContactConstraintManager, LargeIslandSplitter, NarrowPhaseQuery, BodyInterface,
DeterminismLog, PhysicsScene).**
- The QuadTree is the PhysicsSystem default, as Jolt's `BROAD_PHASE` define makes it; `.brute_force`
  stays selectable, and a test checks that it gives the same simulation results.
- Jolt also heap-allocates in `Optimize`, `NotifyBodiesLayerChanged` and in the
  `STLLocalAllocator<., 128>` walk stacks. Allocation failures are handled as follows:
  - `findCollidingPairs`, `optimize` and `notifyBodiesLayerChanged` return `Allocator.Error`, and
    the collision job latches the failure.
  - `notifyBodiesLayerChanged` allocates everything first; `BodyInterface.setObjectLayer` restores
    the layer on failure.
  - `castAABoxNoLock` returns `Allocator.Error` too; the FindCCDContacts job latches it. The
    locking queries have no error channel: they `@panic` when a walk stack deeper than the inline
    128 entries cannot be allocated.
- A fixed BuildTree / AddBodiesAbort stack that overflows (undefined behavior in Jolt) `@panic`s
  with a message.
- `optimizeBroadPhase` is not safe while queries run, as in Jolt (Jolt's `Optimize` swaps trees
  without flipping the query lock index).
- ContactConstraintManager:
  - The cache types and the constraint buffer have Jolt's exact layout. `CachedManifold` flags are
    `std.atomic.Value(u16)` with `.seq_cst`.
  - New cache entries are zero-filled like Jolt's value-initialization.
  - Motion type pairs for which Jolt's dispatch table holds `nullptr` `@panic`.
  - `restoreState` returns `Allocator.Error!bool`.
  - `finalizeContactCacheAfterOutOfMemory` clears the write cache, resets the read cache's persisted
    marks and calls no listener.
- Two Jolt bugs are reproduced: re-found manifolds of a failed constraint creation (or of CCD) read
  uninitialized friction lambdas, and a restore into a cache that is too small desynchronizes the
  stream.
- LargeIslandSplitter:
  - `Splits` is an extern struct with Jolt's layout.
  - The temp buffers are TempArrays freed LIFO in Reset. `resetAfterOutOfMemory` frees without
    Reset's completion asserts after a poisoned update.
  - `SplitIsland` takes the contact manager as `anytype`.
- **Rule for R3 (refines D5).** In SolveVelocityConstraints, check the out-of-memory latch only at
  job entry. A job that has entered must keep claiming and splitting islands even when poisoned.
  Otherwise `next_split_island` never reaches `num_split_islands`, and jobs in the batch loop spin
  forever on `FetchNextBatch`.
- NarrowPhaseQuery:
  - Every query takes `*const` receivers; the shape filter is `?*ShapeFilter` where Jolt writes its
    `mBodyID2`.
  - `collideShapeWithInternalEdgeRemoval` takes an allocator for the collector's heap fallback.
  - The single-hit `castRay` returns `hit.fraction <= 1.0` like Jolt's code (not its doc comment).
- BodyCreationSettings binary state rejects out-of-range enum bytes with Jolt's error text.
  `saveWithChildren` returns `error.InvalidShape` without a shape (Jolt dereferences null).
- PhysicsScene:
  - `createBodies` undoes everything on an allocation error.
  - Soft bodies are rejected until Phase 9.
  - Constraint parity waits for the Phase 6 constraint settings.
- DeterminismLog: the unit tests run a second time with the log enabled. The text is checked by
  construction and with expected strings, not against Jolt's log: the C++ reference is built
  without `JPH_ENABLE_DETERMINISM_LOG`.

**Wave B (ProcessBodyPair, the island solve, CCD hits).**
- `PhysicsSystem.zig` has no REGION markers left. Only the soft body work of Phase 9 is missing:
  the soft body jobs after the "no active soft bodies" path, and a CCD hit on a soft body face.
- ProcessBodyPair:
  - Its collectors are local structs (`overrides = .{.addHit}`).
  - It returns `Allocator.Error` from the InternalEdgeRemovingCollector's heap fallback (the
    default SimCollideBodyVsBody), and both FindCollisions call sites latch it. This is the third
    heap allocation inside a job after `updatePrepare` and `findCollidingPairs` (D5).
  - With manifold reduction, the manifolds collected before such an error are dropped; without it,
    the contacts added before the error stay.
  - A single failed allocation of a growing STLLocalAllocator array is absorbed by its retry, so
    heap sweeps need a "heap runs out from allocation n on" mode (`FailingHeapAllocator.fail_from`).
- JobSolveVelocityConstraints checks the latch only at job entry (section 8, Wave A rule) and never
  allocates. `applied_impulse |= x` is `applied_impulse = x or applied_impulse`, so the contacts
  are always solved.
- CCD:
  - A CCD manifold is only recorded through `OnCCDContactAdded` when the discrete narrow phase did
    not create a manifold for the same sub shape pair in that update; otherwise Jolt calls
    `OnContactPersisted`.
  - `sCalculateBodyMotion` returns `v * dt` even for a linear cast body below the cast threshold,
    which already moved in IntegrateVelocity. This is reproduced, and a scene covers it.
- A latent Jolt bug is kept out of every scene. JobResolveCCDContacts activates woken bodies in
  batches of 64 while it still loops over the CCD bodies. A later CCD body that hit a body of an
  earlier batch then reads past `mActiveBodyToCCDBody`: Jolt asserts in debug, Zolt asserts in safe
  builds. The SimScenes header states the rule for scene authors.
- Simulation parity now runs 36 contact scenes and 18 CCD scenes bit exact, besides the gravity
  scenes. They cover stacks and pyramids with and without the LargeIslandSplitter, every shape type,
  sensors, listeners that modify or reject contacts, FindCollisions jobs spawned inside jobs,
  several FindCCDContacts jobs, cross save / restore, and far from the origin.
  - Each scene declares in `SceneEntry.expect` the cases it must reach (contacts, removals,
    sleeping, spawned jobs, a split island, a full manifold array, CCD hits).
  - Only `constraints_chain` stays blocked: it needs the Phase 6 constraint types (it runs since
    the Phase 6 foundation).

**Wave C (tests, M5, M6).**
- Library fix: after `error.OutOfMemory` inside a job, the InvalidateContactCache flags set before
  the update (SetShape, NotifyShapeChanged, SetUseManifoldReduction, InvalidateContactCache) were
  lost. They are not saved state, and ContactRemovedCallbacks clears them. BodyManager now saves
  them before the update (`saveContactCacheInvalidation`, its capacity reserved where the flag is
  set) and restores them after a failed update (`restoreContactCacheInvalidation`). A successful
  update behaves exactly like Jolt. A flag that a step listener sets during a failed update is
  dropped: per D5 the caller restores a saved state, and the retried update sets it again.
- `StringTools.cppFloat` / `writeFloatPrecision` format like `std::ostream` / `%g` (PortingGuide
  section 5, "Strings, I/O and logging"). The `format` methods of the Math types use it, so
  `convertToString` equals Jolt's ConvertToString.
- ZoltPerformanceTest:
  - It ports every argument, the state recording and validation, and Jolt's output texts.
  - The PerformanceTestScene interface is pattern B.
  - `Trace` goes to stdout through a custom `std_options.logFn`.
  - The `perf_*` simulation parity scenes (Pyramid 50, ConvexVsMesh 80, HighSpeed 20 updates) find a
    hash mismatch at its first differing field.
- Performance (information, not a bit exactness issue): ConvexVsMesh runs 4 to 9 times slower than
  the C++ build. Most of the time is a memset of ProcessBodyPair's local
  `StaticArray(MyManifold, 32)` (about 70 KB) per body pair, plus smaller ones in
  `CollideShapeResult.init` and ClipPoly. Pyramid and HighSpeed are 1.2 to 2 times slower. A later
  performance pass should avoid zero-filling these `undefined` buffers.

## 9. Phase 6: constraints

Phase 6 adds the twelve two body constraint classes, PathConstraintPath and
PathConstraintPathHermite on top of D8. It was ported as a foundation (Fixed, Point, the harnesses,
stubs of the other classes) and then one worker group per class family. These points were settled
on the way and bind later phases that add constraints (VehicleConstraint, Ragdoll).

**Structure.**
- The header of `FixedConstraint.zig` is the porter template for a constraint class: pattern A with
  `overrides`, the settings with `createConstraint`, the binary state, the state recorder layout and
  the parity slots to fill.
- `RegisterTypes.constraint_settings_types` follows Jolt's RegisterTypes.cpp order (VehicleConstraint
  goes after Path in Phase 7). `path_constraint_path_types` holds PathConstraintPathHermite. A user
  adds classes through `zolt_user_types`.
- A class that is not ported yet is a stub file whose settings declare `pub const is_stub = true`.
  `restoreFromBinaryState` skips stubs, and the parity harnesses skip the scenes and kinds that
  need them; removing `is_stub` turns them on. No stub is left after Phase 6.
- Enums that the binary state restores (an `enum class` has a fixed underlying type)
  (`ConstraintSpace`, `SpringMode`, `MotorState`, `SwingType`, `PathRotationConstraintType`) are
  non-exhaustive Zig enums. A stream can hold any value, and the code must treat unknown values as
  Jolt does (e.g. the SwingTwist part treats every value but Cone as Pyramid).
- Gear and RackAndPinion read the referenced Hinge / Slider through `getCurrentAngle` /
  `getCurrentPosition`, so they import those classes.

**Harnesses.**
- `ConstraintsParity.zig` (+ `ConstraintsReference.cpp`) compares one constraint between two bodies
  after every call: creation for every supported motion type combination (fixed to world, static
  with motion properties, partially allowed DOFs, offset centers of mass, far from the origin in
  double precision), every setter, the solver calls, the getters, `saveState` and the settings
  binary state with a cross restore. Its header explains how a class fills its slot.
- The SimScenes header ("Constraint scenes (Phase 6)", "Reserved numbers", "Recipe for a Phase 6
  worker") describes the constraint descriptions, the operations in the reserved OpKind ranges
  (200..399, 380..399 spare), the per kind blockers and the rules for scene authors. 105
  constraint scenes run bit exact in the thread matrix with the D12 sweeps.
- BodyInterfaceParity's PhysicsScene parity creates random constraints of every class but Path.

**Jolt behavior reproduced on purpose.** Each is documented where it is ported and covered by a
test or a parity case:
- `SixDOFConstraint::GetConstraintSettings` does not copy `mLimitsSpringSettings`, so the returned
  settings have hard limits. `SetLimitsSpringSettings` does not call `UpdateFixedFreeAxis`, and
  `SetMaxFriction(0)` clears the motor flag without deactivating the motor parts.
- `PathConstraint::GetConstraintSettings` is not implemented: with asserts enabled Zolt asserts like
  Jolt's debug build, without them it returns an empty `Ref` like the release build.
  `PhysicsScene.fromPhysicsSystem` then adds the constraint without settings, and `createBodies` /
  `saveBinaryState` panic where Jolt dereferences the null pointer. The test of this behavior runs
  only without asserts.
- `PathConstraintSettings::RestoreBinaryState` ignores a failed path restore and keeps reading.
  `GetIndexAndT` maps fractions outside the int range or NaN to `INT_MIN`. Negative fractions on a
  looping path extrapolate instead of wrapping.
- The GearConstraintPart applies `lambda * I2^-1 b` to body 2 where its Jacobian needs
  `r * lambda * I2^-1 b`. It diverges for some valid settings (ratios below 0.5 with a heavy gear 1,
  most negative ratios). The gear position correction uses hinge angles that wrap at ±pi.
- DistanceConstraintSettings computes a negative distance from the world positions in local space
  too, and `< 0.0f` treats -0.0 as a given distance. PulleyConstraint accepts `min > max` and any
  negative minimum in its constructor; only `SetLength` asserts.
- Setters (SetMotorState, SetTargetPosition, SetLimits, SetHalfConeAngle, SetTargetOrientation*)
  do not wake sleeping bodies; only `ActivateConstraint` does. The cone's Baumgarte correction acts
  on cos(angle), so a body can fall asleep slightly outside a narrowed cone.
- `PathConstraint::SaveState` writes `MotorSettings` raw, with 3 padding bytes. Zolt writes zeros,
  like the C++ parity build; a C++ build with garbage there would differ only in those bytes.

**Jolt undefined behavior that Zolt does not reproduce.** A 1-point open path indexes
`mPoints[-1]` and an empty path indexes out of bounds: Zolt's safe builds panic, ReleaseFast stays
undefined as in C++. A null path dereference is an explicit `@panic` with a message.

**Degenerate cases that the harnesses keep away from.** The C++ reference library is built without
`JPH_ASSERT`, so it cannot prove that a scene respects Jolt's debug asserts; Zolt's Debug build
asserts the same and finds them. The generators avoid them instead of changing library code:
- A dynamic body with partially allowed rotation DOFs as the only rotating body of a Hinge or
  SixDOF: the effective mass is singular up to rounding noise, so the velocities become NaN (Jolt
  asserts in `MotionProperties::ApplyAngularVelocityStep`).
- ConeConstraint position correction with partially masked rotation DOFs and an axis almost
  perpendicular to the allowed ones (NaN), and SwingTwist's "swing parts should turn on / off
  together" assert with an axis aligned swing motor axis masked by AllowedDOFs. Both are compared in
  ReleaseFast only, where Jolt release and Zolt agree bit for bit, NaNs included.
- Gear / RackAndPinion with a body that is not dynamic (Jolt asserts; compared in ReleaseFast
  only), and Gear ratios that diverge.
- A Gear / RackAndPinion that references a hinge or slider in another island: SolvePositionConstraint
  reads bodies that a parallel job may be moving, so the result depends on thread timing, in Jolt
  too.

**Open: a Jolt thread dependency.** A 12x12 mat of hinged plates spaced 0.5 apart (gaps of 0.02,
inside the speculative contact distance) falls asleep and is woken by an impulse. Two updates later
C++ Jolt with 0 worker threads has one contact cache body pair more (199) than with 3 or 7 threads
(198), while the body states are still equal. This reproduces with two C++ worlds alone. The cause
was not found; the scene spaces the plates 0.6 apart.
