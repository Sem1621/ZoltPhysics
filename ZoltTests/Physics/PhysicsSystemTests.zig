//! Zolt addition, no Jolt file: tests of PhysicsSystem.update, the job graph and BodyInterface (SimulationArchitecture
//! D4, D5, D6, D12)
//! Status: partial
//! Missing: the scenes with constraints in islands (the Phase 6 constraint types)
//!
//! The gravity-only scene (adapted from the Phase 5 architecture prototype, SimulationPrototype.zig, deleted in F2) is
//! laid out so that no two bodies ever overlap (every body has its own vertical column, the floor is far below), while
//! every job runs (the bullets go through the CCD jobs without hits).
//! - Determinism: JobSystemSingleThreaded against thread pools with 0, 1, 3 and 7 worker threads, the saved state is
//!   compared after every update.
//! - D5: every temp allocation of an update fails in turn (single and multi threaded), every createJob of an update
//!   fails in turn, a heap allocation inside a job fails (a custom broad phase that allocates in updatePrepare, which
//!   runs in the UpdateBroadPhasePrepare job, or fails castAABoxNoLock in the FindCCDContacts jobs). The update returns error.OutOfMemory, the temp allocator is empty, a
//!   failure while building the jobs changes nothing, and the system recovers by restoring the state saved before.
//! - Every allocation of creating the scene fails in turn (std.testing.checkAllAllocationFailures).
//! - The BodyInterface functions that flag the contact cache invalid (setShape, notifyShapeChanged,
//!   setUseManifoldReduction, invalidateContactCache): a failed call changes nothing and can be repeated.
//!
//! The contact scenes (Wave B R2 / R3: ProcessBodyPair, TrySpawnJobFindCollisions, the island solve):
//! - A dense grid of spheres (few batches of active bodies with many body pairs each), a sleeping box woken up by a
//!   contact, a sensor, a kinematic body pushing a dynamic one and a large box with enhanced internal edge removal on
//!   a fine mesh. Determinism against thread pools of 0, 1, 3 and 7 workers with the saved state after every update:
//!   FindCollisions jobs are spawned from inside jobs and the order of the contact callbacks differs between the runs
//!   (both asserted), the states are bit identical.
//! - Save / restore of the contact cache (also validating).
//! - D12 sweeps: every temp allocation fails in turn (the poisoned contact cache is discarded, the temp allocator is
//!   empty, restoreState recovers), every createJob of the job graph fails in turn, spawned FindCollisions jobs fail
//!   (not an error, the results do not change), the heap runs out at every allocation inside the jobs (the QuadTree and
//!   the InternalEdgeRemovingCollector in ProcessBodyPair), checkAllAllocationFailures on creating the scene.
//! - The LargeIslandSplitter path: a pyramid of 136 boxes is one island above the splitting threshold; it is solved in
//!   splits (bit identical with thread pools, different from the solve without the splitter) and goes to sleep from
//!   the final batch of the split island.
//!
//! The CCD scene (Wave B R4: JobFindCCDContacts, sSolveCCDContact, JobResolveCCDContacts): 32 linear cast bullets
//! fired at the thin walls of a container (one rejected by OnContactValidate and one turned into a sensor contact go
//! through), two bullets hitting each other, a bullet that wakes a sleeping box up and one that hits a kinematic box.
//! - Determinism against thread pools of 0, 1, 3 and 7 workers with the saved state after every update, with
//!   min(ceil(CCD bodies / 4), concurrency) FindCCDContacts jobs per step (asserted).
//! - D12 sweeps on the update in which the bullets hit the walls: every temp allocation fails in turn, every createJob
//!   of the job graph fails in turn, every FindCCDContacts job (and every job from it on) cannot be created (not an
//!   error, PostIntegrateVelocity does the work itself when none could be created: the results do not change), the heap
//!   runs out at every allocation inside the jobs, the broad phase cast of FindCCDContacts runs out of memory,
//!   checkAllAllocationFailures on creating the scene; save and restore with the CCD manifolds in the contact cache.

const std = @import("std");
const zolt = @import("zolt");
const LayersFile = @import("../Layers.zig");
const Layers = LayersFile.Layers;
const PhysicsTestContext = @import("../PhysicsTestContext.zig").PhysicsTestContext;
const LoggingBodyActivationListener = @import("../LoggingBodyActivationListener.zig").LoggingBodyActivationListener;

const Allocator = std.mem.Allocator;
const Core = zolt.Core;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const AABox = zolt.AABox;
const OrientedBox = zolt.OrientedBox;
const Ref = zolt.Ref;
const HashCombine = zolt.HashCombine;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyManager = zolt.BodyManager;
const BodyInterface = zolt.BodyInterface;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BodyLockRead = zolt.BodyLockRead;
const BodyLockWrite = zolt.BodyLockWrite;
const BodyLockMultiWrite = zolt.BodyLockMultiWrite;
const BoxShape = zolt.BoxShape;
const BoxShapeSettings = zolt.BoxShapeSettings;
const SphereShapeSettings = zolt.SphereShapeSettings;
const Constraint = zolt.Constraint;
const PhysicsSystem = zolt.PhysicsSystem;
const PhysicsStepListenerContext = zolt.PhysicsStepListenerContext;
const BroadPhase = zolt.BroadPhase;
const BroadPhaseBruteForce = zolt.BroadPhaseBruteForce;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const BroadPhaseLayerFilter = zolt.BroadPhaseLayerFilter;
const ObjectLayerFilter = zolt.ObjectLayerFilter;
const ObjectVsBroadPhaseLayerFilter = zolt.ObjectVsBroadPhaseLayerFilter;
const ObjectLayerPairFilter = zolt.ObjectLayerPairFilter;
const BodyPairCollector = zolt.BodyPairCollector;
const RayCastBodyCollector = zolt.RayCastBodyCollector;
const CastShapeBodyCollector = zolt.CastShapeBodyCollector;
const CollideShapeBodyCollector = zolt.CollideShapeBodyCollector;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const RayCast = zolt.RayCast;
const AABoxCast = zolt.AABoxCast;
const StateRecorderImpl = zolt.StateRecorderImpl;
const StateRecorderFilter = zolt.StateRecorderFilter;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const FailingTempAllocator = zolt.FailingTempAllocator;
const JobSystem = zolt.JobSystem;
const JobSystemThreadPool = zolt.JobSystemThreadPool;
const JobSystemSingleThreaded = zolt.JobSystemSingleThreaded;
const FailingJobSystem = zolt.FailingJobSystem;
const TestConstraint = zolt.test_constraints.TestConstraint;
const TestConstraintSettings = zolt.test_constraints.TestConstraintSettings;

const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;

const io = std.testing.io;

/// Print statistics of the test scene (a passing test must not print, keep false)
const verbose = false;

// -------------------------------------------------------------------------------------------------------------------
// Listeners (called from worker threads: atomics)

const ActivationCounter = struct {
    num_activated: std.atomic.Value(u32) = .init(0),
    num_deactivated: std.atomic.Value(u32) = .init(0),

    pub fn onBodyActivated(self: *ActivationCounter, body_id: BodyID, body_user_data: u64) void {
        _ = body_id;
        _ = body_user_data;
        _ = self.num_activated.fetchAdd(1, .monotonic);
    }

    pub fn onBodyDeactivated(self: *ActivationCounter, body_id: BodyID, body_user_data: u64) void {
        _ = body_id;
        _ = body_user_data;
        _ = self.num_deactivated.fetchAdd(1, .monotonic);
    }
};

/// Pushes one body every step through the no-lock body interface (all bodies are locked by the update)
const PushListener = struct {
    body_interface: *BodyInterface = undefined,
    body_id: BodyID = .invalid,
    num_steps: std.atomic.Value(u32) = .init(0),

    pub fn onStep(self: *PushListener, context: *const PhysicsStepListenerContext) void {
        _ = self.num_steps.fetchAdd(1, .monotonic);
        self.body_interface.addLinearVelocity(self.body_id, Vec3.init(-0.5 * context.delta_time, 0, 0));
    }
};

// -------------------------------------------------------------------------------------------------------------------
// A broad phase that allocates in updatePrepare (a heap allocation inside the UpdateBroadPhasePrepare job, like the new
// tree of Jolt's BroadPhaseQuadTree), can fail castAABoxNoLock (the node stack of BroadPhaseQuadTree's cast growing on
// the heap inside the FindCCDContacts job) and delegates everything else to a BroadPhaseBruteForce

const AllocatingBroadPhase = struct {
    pub const overrides = .{ .initBroadPhase, .optimize, .frameSync, .lockModifications, .updatePrepare, .updateFinalize, .unlockModifications, .addBodiesPrepare, .addBodiesFinalize, .addBodiesAbort, .removeBodies, .notifyBodiesAABBChanged, .notifyBodiesLayerChanged, .castRay, .collideAABox, .collideSphere, .collidePoint, .collideOrientedBox, .castAABoxNoLock, .castAABox, .findCollidingPairs, .getBounds };

    const Block = [64]u32;

    base: BroadPhase,
    inner: *BroadPhaseBruteForce,
    num_prepared: u32 = 0,
    /// castAABoxNoLock returns error.OutOfMemory (read by the FindCCDContacts jobs)
    fail_cast: std.atomic.Value(bool) = .init(false),

    fn create(allocator: Allocator) Allocator.Error!*AllocatingBroadPhase {
        const inner = try BroadPhaseBruteForce.create(allocator);
        errdefer inner.asBroadPhase().base.destroy();
        const self = try allocator.create(AllocatingBroadPhase);
        self.* = .{ .base = .init(AllocatingBroadPhase, allocator), .inner = inner };
        return self;
    }

    pub fn destruct(self: *AllocatingBroadPhase) void {
        self.inner.asBroadPhase().base.destroy();
    }

    fn innerBroadPhase(self: *const AllocatingBroadPhase) *BroadPhase {
        return self.inner.asBroadPhase();
    }

    fn innerQuery(self: *const AllocatingBroadPhase) *const zolt.BroadPhaseQuery {
        return self.inner.asBroadPhase().asQuery();
    }

    pub fn initBroadPhase(self: *AllocatingBroadPhase, body_manager: *BodyManager, layer_interface: *const BroadPhaseLayerInterface) Allocator.Error!void {
        try BroadPhase.impl.initBroadPhase(&self.base, body_manager, layer_interface);
        try self.innerBroadPhase().initBroadPhase(body_manager, layer_interface);
    }

    pub fn optimize(self: *AllocatingBroadPhase) Allocator.Error!void {
        return self.innerBroadPhase().optimize();
    }

    pub fn frameSync(self: *AllocatingBroadPhase) void {
        self.innerBroadPhase().frameSync();
    }

    pub fn lockModifications(self: *AllocatingBroadPhase) void {
        self.innerBroadPhase().lockModifications();
    }

    pub fn updatePrepare(self: *AllocatingBroadPhase) Allocator.Error!BroadPhase.UpdateState {
        var state = try self.innerBroadPhase().updatePrepare();
        std.debug.assert(state.data[3] == null);
        const block = try self.base.base.allocator.create(Block);
        block.* = @splat(self.num_prepared);
        state.data[3] = block;
        return state;
    }

    pub fn updateFinalize(self: *AllocatingBroadPhase, update_state: *const BroadPhase.UpdateState) void {
        self.innerBroadPhase().updateFinalize(update_state);
        if (update_state.data[3]) |data| {
            const block: *Block = @ptrCast(@alignCast(data));
            self.num_prepared += 1;
            self.base.base.allocator.destroy(block);
        }
    }

    pub fn unlockModifications(self: *AllocatingBroadPhase) void {
        self.innerBroadPhase().unlockModifications();
    }

    pub fn addBodiesPrepare(self: *AllocatingBroadPhase, bodies: []BodyID) Allocator.Error!BroadPhase.AddState {
        return self.innerBroadPhase().addBodiesPrepare(bodies);
    }

    pub fn addBodiesFinalize(self: *AllocatingBroadPhase, bodies: []BodyID, add_state: BroadPhase.AddState) void {
        self.innerBroadPhase().addBodiesFinalize(bodies, add_state);
    }

    pub fn addBodiesAbort(self: *AllocatingBroadPhase, bodies: []BodyID, add_state: BroadPhase.AddState) void {
        self.innerBroadPhase().addBodiesAbort(bodies, add_state);
    }

    pub fn removeBodies(self: *AllocatingBroadPhase, bodies: []BodyID) void {
        self.innerBroadPhase().removeBodies(bodies);
    }

    pub fn notifyBodiesAABBChanged(self: *AllocatingBroadPhase, bodies: []BodyID, take_lock: bool) void {
        self.innerBroadPhase().notifyBodiesAABBChanged(bodies, .{ .take_lock = take_lock });
    }

    pub fn notifyBodiesLayerChanged(self: *AllocatingBroadPhase, bodies: []BodyID) Allocator.Error!void {
        return self.innerBroadPhase().notifyBodiesLayerChanged(bodies);
    }

    pub fn castRay(self: *const AllocatingBroadPhase, ray: *const RayCast, collector: *RayCastBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        self.innerQuery().castRay(ray, collector, .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter });
    }

    pub fn collideAABox(self: *const AllocatingBroadPhase, box: *const AABox, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        self.innerQuery().collideAABox(box, collector, .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter });
    }

    pub fn collideSphere(self: *const AllocatingBroadPhase, center: Vec3, radius: f32, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        self.innerQuery().collideSphere(center, radius, collector, .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter });
    }

    pub fn collidePoint(self: *const AllocatingBroadPhase, point: Vec3, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        self.innerQuery().collidePoint(point, collector, .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter });
    }

    pub fn collideOrientedBox(self: *const AllocatingBroadPhase, box: *const OrientedBox, collector: *CollideShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        self.innerQuery().collideOrientedBox(box, collector, .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter });
    }

    pub fn castAABoxNoLock(self: *const AllocatingBroadPhase, box: *const AABoxCast, collector: *CastShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) Allocator.Error!void {
        if (self.fail_cast.load(.monotonic))
            return error.OutOfMemory;
        return self.innerBroadPhase().castAABoxNoLock(box, collector, broad_phase_layer_filter, object_layer_filter);
    }

    pub fn castAABox(self: *const AllocatingBroadPhase, box: *const AABoxCast, collector: *CastShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        self.innerQuery().castAABox(box, collector, .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter });
    }

    pub fn findCollidingPairs(self: *const AllocatingBroadPhase, active_bodies: []BodyID, speculative_contact_distance: f32, object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, object_layer_pair_filter: *const ObjectLayerPairFilter, pair_collector: *BodyPairCollector) Allocator.Error!void {
        return self.innerBroadPhase().findCollidingPairs(active_bodies, speculative_contact_distance, object_vs_broad_phase_layer_filter, object_layer_pair_filter, pair_collector);
    }

    pub fn getBounds(self: *const AllocatingBroadPhase) AABox {
        return self.innerQuery().getBounds();
    }
};

// -------------------------------------------------------------------------------------------------------------------
// Test scene

/// Number of free falling bodies (spheres and boxes, some with a lower gravity factor, without damping or in 2D)
const num_falling = 120;
/// Number of fast bullets with the linear cast motion quality (PostIntegrateVelocity spawns FindCCDContacts jobs)
const num_bullets = 3;
/// Number of columns of the grid of cells
const num_cells_x = 12;

/// The position of cell `cell`: every body gets its own vertical column, 4 m from its neighbours, so that bodies never
/// touch (no body pairs, see the file comment)
fn cellPosition(cell: u32, y: f32) RVec3 {
    const x: f32 = 4.0 * @as(f32, @floatFromInt(cell % num_cells_x)) - 22.0;
    const z: f32 = 4.0 * @as(f32, @floatFromInt(cell / num_cells_x)) - 20.0;
    return RVec3.init(x, y, z);
}

fn f(i: u32) f32 {
    return @floatFromInt(i);
}

const Scene = struct {
    broad_phase_layer_interface: LayersFile.BPLayerInterfaceImpl = .init(),
    object_vs_broad_phase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_layer_pair_filter: LayersFile.ObjectLayerPairFilterImpl = .{},
    system: PhysicsSystem = .empty,
    activation_listener: ActivationCounter = .{},
    step_listener: PushListener = .{},
    /// Two disabled constraints and an enabled one between a static and a sleeping body: never active
    constraints: [3]*TestConstraint = undefined,
    sleeping_body: BodyID = .invalid,
    inactive_body: BodyID = .invalid,
    inactive_position: RVec3 = RVec3.zero(),
    kinematic_body: BodyID = .invalid,
    bullets: [num_bullets]BodyID = @splat(.invalid),

    /// Builds the scene in place (the physics system must not move)
    fn init(self: *Scene, allocator: Allocator, opts: PhysicsSystem.InitOptions) !void {
        try self.initSystem(allocator, opts);
        errdefer self.deinit();
        try self.populate(allocator);
    }

    /// Initializes the physics system (on error a custom broad phase still belongs to the caller)
    fn initSystem(self: *Scene, allocator: Allocator, opts: PhysicsSystem.InitOptions) !void {
        self.* = .{};
        try self.system.init(allocator, io, 1024, 0, 4096, 1024, &self.broad_phase_layer_interface.base, &self.object_vs_broad_phase_layer_filter.base, &self.object_layer_pair_filter.base, opts);

        // The body at rest falls asleep after 24 updates of 2 collision steps (the sleep timer advances one collision
        // step per update)
        var settings = self.system.getPhysicsSettings().*;
        settings.time_before_sleep = 0.2;
        self.system.setPhysicsSettings(&settings);
    }

    /// Creates the bodies, the constraints and the listeners
    fn populate(self: *Scene, allocator: Allocator) !void {
        self.system.setBodyActivationListener(.init(&self.activation_listener));
        const bi = self.system.getBodyInterfaceMut();

        // Floor (a Shape) far below the bodies
        const floor_shape = try BoxShape.create(allocator, Vec3.init(60, 1, 60), .{});
        var floor_settings = BodyCreationSettings.initWithShape(floor_shape.asShape(), RVec3.init(0, -200, 0), Quat.identity(), .static, Layers.non_moving);
        defer floor_settings.deinit();
        try expect(!(try bi.createAndAddBody(&floor_settings, .dont_activate)).isInvalid());

        // Shared ShapeSettings, converted on first use by a const BodyCreationSettings
        var sphere_settings = BodyCreationSettings.initWithSettings((try SphereShapeSettings.create(allocator, 0.5, .{})).asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
        defer sphere_settings.deinit();
        var box_settings = BodyCreationSettings.initWithSettings((try BoxShapeSettings.create(allocator, Vec3.init(0.4, 0.5, 0.3), .{})).asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
        defer box_settings.deinit();

        // Free falling bodies
        var cell: u32 = 0;
        while (cell < num_falling) : (cell += 1) {
            const settings = if (cell % 2 == 0) &sphere_settings else &box_settings;
            settings.position = cellPosition(cell, 5.0 + 2.5 * f(cell % 7));
            settings.rotation = Quat.rotation(Vec3.init(0, 1, 0), 0.1 * f(cell % 5));
            settings.linear_velocity = Vec3.init(0.02 * f(cell % 3) - 0.02, 0.3 * f(cell % 5), 0.01 * f(cell % 4) - 0.015);
            settings.angular_velocity = Vec3.init(0.3 * f(cell % 3), 0.2, -0.1 * f(cell % 2));
            settings.gravity_factor = if (cell % 9 == 0) 0.5 else 1.0;
            settings.allowed_dofs = if (cell % 11 == 0) .plane_2d else .all;
            settings.linear_damping = if (cell % 4 == 0) 0.0 else 0.05;
            const id = try bi.createAndAddBody(settings, .activate);
            try expect(!id.isInvalid());
            if (cell == 0) self.step_listener.body_id = id; // A corner of the grid, pushed outwards
        }
        for ([_]*BodyCreationSettings{ &sphere_settings, &box_settings }) |settings| {
            settings.rotation = Quat.identity();
            settings.gravity_factor = 1.0;
            settings.allowed_dofs = .all;
            settings.linear_damping = 0.05;
        }

        // Fast bullets with the linear cast motion quality: they move more than the linear cast threshold per step
        sphere_settings.motion_quality = .linear_cast;
        sphere_settings.linear_velocity = Vec3.init(0, -60, 0);
        sphere_settings.angular_velocity = Vec3.zero();
        for (&self.bullets, 0..) |*bullet, i| {
            sphere_settings.position = cellPosition(cell, 20.0 + 3.0 * @as(f32, @floatFromInt(i)));
            bullet.* = try bi.createAndAddBody(&sphere_settings, .activate);
            try expect(!bullet.isInvalid());
            cell += 1;
        }
        sphere_settings.motion_quality = .discrete;
        sphere_settings.linear_velocity = Vec3.zero();

        // A body at rest without gravity: falls asleep after PhysicsSettings.time_before_sleep
        sphere_settings.position = cellPosition(cell, 3.0);
        sphere_settings.gravity_factor = 0.0;
        self.sleeping_body = try bi.createAndAddBody(&sphere_settings, .activate);
        sphere_settings.gravity_factor = 1.0;
        cell += 1;

        // A body that is never activated
        self.inactive_position = cellPosition(cell, 10.0);
        box_settings.position = self.inactive_position;
        self.inactive_body = try bi.createAndAddBody(&box_settings, .dont_activate);
        cell += 1;

        // Kinematic bodies: one moving up, one rotating in place
        box_settings.motion_type = .kinematic;
        box_settings.position = cellPosition(cell, 4.0);
        box_settings.linear_velocity = Vec3.init(0, 1, 0);
        box_settings.angular_velocity = Vec3.init(0, 0.5, 0);
        self.kinematic_body = try bi.createAndAddBody(&box_settings, .activate);
        cell += 1;
        sphere_settings.motion_type = .kinematic;
        sphere_settings.position = cellPosition(cell, 4.0);
        sphere_settings.angular_velocity = Vec3.init(0, 2, 0);
        _ = try bi.createAndAddBody(&sphere_settings, .activate);
        cell += 1;
        box_settings.motion_type = .dynamic;
        box_settings.linear_velocity = Vec3.zero();
        box_settings.angular_velocity = Vec3.zero();
        sphere_settings.motion_type = .dynamic;
        sphere_settings.angular_velocity = Vec3.zero();

        // A static body above everything
        box_settings.motion_type = .static;
        box_settings.object_layer = Layers.non_moving;
        box_settings.position = cellPosition(cell, 60.0);
        const static_body = try bi.createAndAddBody(&box_settings, .dont_activate);
        box_settings.motion_type = .dynamic;
        box_settings.object_layer = Layers.moving;
        cell += 1;

        // Constraints that are never active (the gravity-only scene; constraints in islands need the Phase 6 constraint types)
        var constraint_settings = TestConstraintSettings.init(allocator);
        defer constraint_settings.asSettings().deinit();
        constraint_settings.base.base.enabled = false;
        constraint_settings.value = 3.5;
        {
            // Two bodies with TwoBodyConstraintSettings::Create(body1, body2) with the bodies locked
            sphere_settings.position = cellPosition(cell, 8.0);
            const a = try bi.createAndAddBody(&sphere_settings, .activate);
            cell += 1;
            sphere_settings.position = cellPosition(cell, 8.0);
            const b = try bi.createAndAddBody(&sphere_settings, .activate);
            cell += 1;
            var lock: BodyLockMultiWrite = .init(self.system.getBodyLockInterface(), &.{ a, b });
            defer lock.deinit();
            const two_body = try constraint_settings.base.createConstraint(allocator, lock.getBody(0).?, lock.getBody(1).?);
            var c = Ref(Constraint).init(&two_body.base);
            defer c.deinit();
            try self.system.addConstraint(c.get().?);
            self.constraints[0] = zolt.virtual.downcast(TestConstraint, two_body);
        }
        {
            // A body attached to the world with BodyInterface::CreateConstraint
            box_settings.position = cellPosition(cell, 8.0);
            const id = try bi.createAndAddBody(&box_settings, .activate);
            cell += 1;
            const two_body = try bi.createConstraint(&constraint_settings.base, .invalid, id);
            var c = Ref(Constraint).init(&two_body.base);
            defer c.deinit();
            try self.system.addConstraint(c.get().?);
            self.constraints[1] = zolt.virtual.downcast(TestConstraint, two_body);
        }
        {
            // Enabled, but its bodies are static and inactive
            constraint_settings.base.base.enabled = true;
            const two_body = try bi.createConstraint(&constraint_settings.base, static_body, self.inactive_body);
            var c = Ref(Constraint).init(&two_body.base);
            defer c.deinit();
            try self.system.addConstraint(c.get().?);
            self.constraints[2] = zolt.virtual.downcast(TestConstraint, two_body);
        }

        // Step listener
        self.step_listener.body_interface = self.system.getBodyInterfaceNoLockMut();
        try self.system.addStepListener(.init(&self.step_listener));
    }

    fn deinit(self: *Scene) void {
        self.system.deinit(); // Releases the constraint references, destroys the bodies
    }

    /// The PerformanceTest hash: FNV-1a over position (3 Reals) and rotation of every body in body ID order
    fn hash(self: *const Scene) u64 {
        var h: u64 = HashCombine.fnv1a_seed;
        for (self.system.body_manager.getBodies()) |slot| {
            if (!BodyManager.isValidBodyPointer(slot)) continue;
            const body = slot.getBody();
            const p = body.getPosition();
            const r = body.getRotation();
            const values = [_]zolt.Real{ p.getX(), p.getY(), p.getZ() };
            h = HashCombine.hashBytesSeeded(std.mem.sliceAsBytes(&values), h);
            const q = [_]f32{ r.getX(), r.getY(), r.getZ(), r.getW() };
            h = HashCombine.hashBytesSeeded(std.mem.sliceAsBytes(&q), h);
        }
        return h;
    }
};

const JobSystemKind = union(enum) {
    single_threaded,
    thread_pool: i32, // number of worker threads
};

/// A job system of either kind, initialized in place
const AnyJobSystem = struct {
    single: JobSystemSingleThreaded = .empty,
    pool: JobSystemThreadPool = .empty,
    is_pool: bool = false,

    fn init(self: *AnyJobSystem, allocator: Allocator, kind: JobSystemKind) !void {
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

const num_updates = 60;
const delta_time: f32 = 1.0 / 60.0;
const collision_steps = 2;

const RunStats = struct {
    counts: [3]u32, // activations, deactivations, step listener calls
};

/// Runs the scene and records the saved state after every update
fn runScene(allocator: Allocator, kind: JobSystemKind, states: *std.ArrayList(u8), offsets: *std.ArrayList(usize), stats: *RunStats) !u64 {
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit();

    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(Scene);
    defer allocator.destroy(scene);
    try scene.init(allocator, .{});
    defer scene.deinit();

    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();

    const bullet_start_y = scene.system.getBodyInterface().getPosition(scene.bullets[0]).getY();
    const kinematic_start_y = scene.system.getBodyInterface().getPosition(scene.kinematic_body).getY();

    for (0..num_updates) |_| {
        const errors = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
        try expect(errors.isNone());
        try expect(temp.isEmpty());

        recorder.clear();
        try scene.system.saveState(recorder.recorder(), .{});
        try expect(!recorder.isFailed());
        try offsets.append(allocator, states.items.len);
        try states.appendSlice(allocator, recorder.getData());
    }

    stats.* = .{
        .counts = .{ scene.activation_listener.num_activated.load(.monotonic), scene.activation_listener.num_deactivated.load(.monotonic), scene.step_listener.num_steps.load(.monotonic) },
    };

    // Sanity: the bullets fell through the CCD jobs, the kinematic body moved up, the body at rest fell asleep, the
    // inactive body did not move and no constraint was ever active
    const bi = scene.system.getBodyInterface();
    try expect(bi.getPosition(scene.bullets[0]).getY() < bullet_start_y - 50.0);
    try expect(bi.getPosition(scene.kinematic_body).getY() > kinematic_start_y + 0.9);
    try expect(!bi.isActive(scene.sleeping_body));
    try expect(!bi.isActive(scene.inactive_body));
    try expect(bi.getPosition(scene.inactive_body).eql(scene.inactive_position));
    for (scene.constraints) |c| {
        try expectEqual(@as(u32, 0), c.num_setup.load(.monotonic));
        try expectEqual(@as(u32, 0), c.num_solve_velocity.load(.monotonic));
    }
    try expectEqual(@as(usize, 1 + num_falling + num_bullets + 8), scene.system.getNumBodies());
    if (verbose)
        std.debug.print("{any}: activations {d} deactivations {d} steps {d}\n", .{ kind, stats.counts[0], stats.counts[1], stats.counts[2] });
    return scene.hash();
}

// -------------------------------------------------------------------------------------------------------------------
// Tests

/// Runs the scene on JobSystemSingleThreaded (the reference) and on thread pools: the saved state after every update
/// must be bit identical
fn determinismMatrix(kinds: []const JobSystemKind) !void {
    const allocator = std.testing.allocator;

    // Reference: single threaded job system
    var ref_states: std.ArrayList(u8) = .empty;
    defer ref_states.deinit(allocator);
    var ref_offsets: std.ArrayList(usize) = .empty;
    defer ref_offsets.deinit(allocator);
    var ref_stats: RunStats = undefined;
    const ref_hash = try runScene(allocator, .single_threaded, &ref_states, &ref_offsets, &ref_stats);
    try expectEqual(@as(u32, num_updates * collision_steps), ref_stats.counts[2]);
    try expect(ref_stats.counts[0] >= num_falling + num_bullets); // Every body that was added active
    try expectEqual(@as(u32, 1), ref_stats.counts[1]); // The body at rest

    for (kinds) |kind| {
        var states: std.ArrayList(u8) = .empty;
        defer states.deinit(allocator);
        var offsets: std.ArrayList(usize) = .empty;
        defer offsets.deinit(allocator);
        var stats: RunStats = undefined;
        const h = try runScene(allocator, kind, &states, &offsets, &stats);

        // Bit identical state after every update
        try std.testing.expectEqualSlices(usize, ref_offsets.items, offsets.items);
        for (ref_offsets.items, 0..) |offset, i| {
            const end = if (i + 1 < ref_offsets.items.len) ref_offsets.items[i + 1] else ref_states.items.len;
            if (!std.mem.eql(u8, ref_states.items[offset..end], states.items[offset..end])) {
                std.debug.print("state differs after update {d} with {any}\n", .{ i, kind });
                return error.TestUnexpectedResult;
            }
        }
        try expectEqual(ref_hash, h);
        try expectEqual(ref_stats.counts, stats.counts);
    }
}

test "PhysicsSystem determinism single vs multi threaded" {
    // Thread pools with 0 (barrier executes everything), 1, 3 and 7 worker threads, several times each
    try determinismMatrix(&.{ .{ .thread_pool = 0 }, .{ .thread_pool = 1 }, .{ .thread_pool = 3 }, .{ .thread_pool = 7 }, .{ .thread_pool = 3 }, .{ .thread_pool = 7 } });
}

test "PhysicsSystem state recorder save, restore, validation and filter" {
    const allocator = std.testing.allocator;
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, .{ .thread_pool = 3 });
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(Scene);
    defer allocator.destroy(scene);
    try scene.init(allocator, .{});
    defer scene.deinit();

    for (0..10) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    // Snapshot, simulate, restore, simulate again: identical
    var snapshot = StateRecorderImpl.init(allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .{});

    var after1 = StateRecorderImpl.init(allocator);
    defer after1.deinit();
    for (0..30) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    try scene.system.saveState(after1.recorder(), .{});
    try expect(!scene.system.getBodyInterface().isActive(scene.sleeping_body)); // Fell asleep in between

    snapshot.rewind();
    try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
    try expect(scene.system.getBodyInterface().isActive(scene.sleeping_body)); // Awake again
    var restored = StateRecorderImpl.init(allocator);
    defer restored.deinit();
    try scene.system.saveState(restored.recorder(), .{});
    try expect(restored.isEqual(&snapshot));

    var after2 = StateRecorderImpl.init(allocator);
    defer after2.deinit();
    for (0..30) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    try scene.system.saveState(after2.recorder(), .{});
    try expect(after2.isEqual(&after1));

    // Validating restore: reading the state we just saved finds no mismatch
    after2.rewind();
    after2.recorder().setValidating(true);
    try expect(try scene.system.restoreState(after2.recorder(), .{}));
    try expectEqual(@as(u32, 0), after2.num_mismatches);

    // The state of the constraints goes through the virtual saveState / restoreState of the TestConstraint
    scene.constraints[0].value = 7.0;
    snapshot.rewind();
    try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
    try expectEqual(@as(f32, 3.5), scene.constraints[0].value);

    // A partial state (bodies only) with a filter, restored again
    const SkipStatic = struct {
        pub const overrides = .{.shouldSaveBody};
        base: StateRecorderFilter = .init(@This()),
        pub fn shouldSaveBody(self: *const @This(), body: *const Body) bool {
            _ = self;
            return !body.isStatic();
        }
    };
    const filter: SkipStatic = .{};
    var partial = StateRecorderImpl.init(allocator);
    defer partial.deinit();
    try scene.system.saveState(partial.recorder(), .{ .state = .{ .bodies = true }, .filter = &filter.base });
    try expect(partial.getData().len < after2.getData().len);
    for (0..5) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    try expect(try scene.system.restoreState(partial.recorder(), .{}));
    var partial_again = StateRecorderImpl.init(allocator);
    defer partial_again.deinit();
    try scene.system.saveState(partial_again.recorder(), .{ .state = .{ .bodies = true }, .filter = &filter.base });
    try expect(partial_again.isEqual(&partial));
}

/// Restore `snapshot`, run one update with enough memory and compare with `reference`
fn expectRecovery(scene: *Scene, snapshot: *StateRecorderImpl, reference: *StateRecorderImpl, temp: *TempAllocatorImpl, job_system: JobSystem) !void {
    const allocator = std.testing.allocator;
    snapshot.rewind();
    try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), job_system);
    try expect(temp.isEmpty());
    var redo = StateRecorderImpl.init(allocator);
    defer redo.deinit();
    try scene.system.saveState(redo.recorder(), .{});
    try expect(redo.isEqual(reference));
}

fn expectState(scene: *Scene, expected: *StateRecorderImpl) !void {
    var state = StateRecorderImpl.init(std.testing.allocator);
    defer state.deinit();
    try scene.system.saveState(state.recorder(), .{});
    try expect(state.isEqual(expected));
}

/// Number of temp allocations while building the jobs: the steps, the body pairs, the active constraints, the contact
/// constraint buffer and its index table (ContactConstraintManager.prepareConstraintBuffer) and the contact links of
/// the island builder. A failure there changes nothing.
const num_build_temp_allocations = 6;

fn outOfTempMemory(kind: JobSystemKind) !void {
    const allocator = std.testing.allocator;
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(Scene);
    defer allocator.destroy(scene);
    try scene.init(allocator, .{});
    defer scene.deinit();
    for (0..5) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    var snapshot = StateRecorderImpl.init(allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .{});

    // Reference: one update with enough memory, counting the temp allocations (the build allocations, then per step
    // in jobs: the constraint links, the islands, the CCD buffers, the sorted CCD bodies, the next step's contact links)
    var counting: FailingTempAllocator = .init(temp.tempAllocator(), .{});
    _ = try scene.system.update(delta_time, collision_steps, counting.tempAllocator(), jobs.jobSystem());
    var reference = StateRecorderImpl.init(allocator);
    defer reference.deinit();
    try scene.system.saveState(reference.recorder(), .{});
    const num_allocations = counting.num_allocations;
    if (verbose) std.debug.print("{any}: temp allocations per update {d}\n", .{ kind, num_allocations });
    try expect(num_allocations >= num_build_temp_allocations + collision_steps * 6);

    // Fail every allocation in turn
    for (0..num_allocations) |fail_index| {
        snapshot.rewind();
        try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
        const steps_before = scene.step_listener.num_steps.load(.monotonic);

        var failing: FailingTempAllocator = .init(temp.tempAllocator(), .{ .fail_index = @intCast(fail_index) });
        try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, failing.tempAllocator(), jobs.jobSystem()));
        try expect(failing.has_induced_failure);
        try expect(temp.isEmpty()); // LIFO: every block was freed at Jolt's positions, also after skipped jobs

        // Failure while building the jobs: no job ran
        if (fail_index < num_build_temp_allocations) {
            try expectEqual(steps_before, scene.step_listener.num_steps.load(.monotonic));
            try expectState(scene, &snapshot);
        }

        // The system recovers: restore and redo the update
        try expectRecovery(scene, &snapshot, &reference, &temp, jobs.jobSystem());
    }
}

test "PhysicsSystem out of temp memory at every allocation, single threaded" {
    try outOfTempMemory(.single_threaded);
}

test "PhysicsSystem out of temp memory at every allocation, multi threaded" {
    try outOfTempMemory(.{ .thread_pool = 3 });
}

/// Number of jobs per step that are created while building the jobs, at least one of each kind
const num_job_kinds_per_step = 18;

fn failingCreateJob(kind: JobSystemKind) !void {
    const allocator = std.testing.allocator;
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit(); // JobSystemSingleThreaded / ThreadPool: every job was freed
    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(Scene);
    defer allocator.destroy(scene);
    try scene.init(allocator, .{});
    defer scene.deinit();
    for (0..5) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    var snapshot = StateRecorderImpl.init(allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .{});

    // Reference: one update, counting the jobs (the bullets are still falling: FindCCDContacts jobs are spawned)
    var counting: FailingJobSystem = .init(jobs.jobSystem(), .{});
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), counting.jobSystem());
    var reference = StateRecorderImpl.init(allocator);
    defer reference.deinit();
    try scene.system.saveState(reference.recorder(), .{});
    const num_jobs = counting.num_created.load(.monotonic);

    // Fail every createJob in turn: while building the graph the update fails without side effects (start gate), a job
    // that cannot be spawned from inside a job is not an error and does not change the results
    var num_build_failures: u32 = 0;
    var num_spawn_failures: u32 = 0;
    for (0..num_jobs + 4) |fail_index| {
        snapshot.rewind();
        try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
        const steps_before = scene.step_listener.num_steps.load(.monotonic);

        var failing: FailingJobSystem = .init(jobs.jobSystem(), .{ .fail_index = @intCast(fail_index) });
        if (scene.system.update(delta_time, collision_steps, temp.tempAllocator(), failing.jobSystem())) |errors| {
            try expect(errors.isNone());
            if (failing.has_induced_failure.load(.monotonic)) num_spawn_failures += 1;
            try expectState(scene, &reference);
        } else |err| {
            try expectEqual(error.OutOfMemory, err);
            num_build_failures += 1;
            try expectEqual(steps_before, scene.step_listener.num_steps.load(.monotonic)); // No job ran
            try expectState(scene, &snapshot); // Nothing changed
            try expectRecovery(scene, &snapshot, &reference, &temp, jobs.jobSystem());
        }
        try expect(temp.isEmpty());
    }
    if (verbose) std.debug.print("{any}: jobs {d} build failures {d} spawn failures {d}\n", .{ kind, num_jobs, num_build_failures, num_spawn_failures });
    try expect(num_build_failures >= collision_steps * num_job_kinds_per_step + 1); // + StartNextStep of the first step, more jobs with more threads
    try expect(num_spawn_failures > 0); // The FindCCDContacts spawns
    try expectEqual(num_jobs, num_build_failures + num_spawn_failures);
    if (kind == .single_threaded) {
        // One job of each kind per step (a concurrency of 1), one FindCCDContacts job per step (3 bullets)
        try expectEqual(@as(u32, collision_steps * num_job_kinds_per_step + 1), num_build_failures);
        try expectEqual(@as(u32, collision_steps), num_spawn_failures);
    }
}

test "PhysicsSystem createJob fails at every job, single threaded" {
    try failingCreateJob(.single_threaded);
}

test "PhysicsSystem createJob fails at every job, multi threaded" {
    try failingCreateJob(.{ .thread_pool = 3 });
}

/// A thread safe allocator that fails its n-th allocation (heap allocations happen inside jobs on worker threads)
const FailingHeapAllocator = struct {
    inner: Allocator,
    fail_index: std.atomic.Value(u32) = .init(std.math.maxInt(u32)),
    /// Every allocation from this one on fails (an allocator that ran out: a failed remap of a growing array is retried
    /// with an allocation, which fails too)
    fail_from: std.atomic.Value(u32) = .init(std.math.maxInt(u32)),
    num_allocations: std.atomic.Value(u32) = .init(0),

    fn allocator(self: *FailingHeapAllocator) Allocator {
        return .{ .ptr = self, .vtable = &.{ .alloc = alloc, .resize = Allocator.noResize, .remap = Allocator.noRemap, .free = free } };
    }

    fn alloc(ctx: *anyopaque, len: usize, alignment: std.mem.Alignment, ret_addr: usize) ?[*]u8 {
        const self: *FailingHeapAllocator = @ptrCast(@alignCast(ctx));
        const n = self.num_allocations.fetchAdd(1, .monotonic);
        if (n == self.fail_index.load(.monotonic) or n >= self.fail_from.load(.monotonic))
            return null;
        return self.inner.rawAlloc(len, alignment, ret_addr);
    }

    fn free(ctx: *anyopaque, memory: []u8, alignment: std.mem.Alignment, ret_addr: usize) void {
        const self: *FailingHeapAllocator = @ptrCast(@alignCast(ctx));
        self.inner.rawFree(memory, alignment, ret_addr);
    }
};

test "PhysicsSystem out of heap memory inside a job (BroadPhase.updatePrepare)" {
    var heap: FailingHeapAllocator = .{ .inner = std.testing.allocator };
    const allocator = heap.allocator();
    var jobs: AnyJobSystem = .{};
    try jobs.init(std.testing.allocator, .{ .thread_pool = 3 });
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(std.testing.allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    // The system owns the custom broad phase after a successful init
    const scene = try std.testing.allocator.create(Scene);
    defer std.testing.allocator.destroy(scene);
    const broad_phase = try AllocatingBroadPhase.create(allocator);
    scene.initSystem(allocator, .{ .broad_phase = .{ .custom = &broad_phase.base } }) catch |err| {
        broad_phase.base.base.destroy();
        return err;
    };
    defer scene.deinit();
    try scene.populate(allocator);
    for (0..5) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    try expectEqual(@as(u32, 5 * collision_steps), broad_phase.num_prepared);

    var snapshot = StateRecorderImpl.init(std.testing.allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .{});

    // Reference: count the heap allocations of one update (one block per step in the UpdateBroadPhasePrepare job)
    const before = heap.num_allocations.load(.monotonic);
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    const num_allocations = heap.num_allocations.load(.monotonic) - before;
    try expectEqual(@as(u32, collision_steps), num_allocations);
    var reference = StateRecorderImpl.init(std.testing.allocator);
    defer reference.deinit();
    try scene.system.saveState(reference.recorder(), .{});

    for (0..num_allocations) |i| {
        snapshot.rewind();
        try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
        const num_prepared = broad_phase.num_prepared;
        heap.fail_index.store(heap.num_allocations.load(.monotonic) + @as(u32, @intCast(i)), .monotonic);
        try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem()));
        heap.fail_index.store(std.math.maxInt(u32), .monotonic);
        try expectEqual(num_prepared + @as(u32, @intCast(i)), broad_phase.num_prepared); // The steps before the failure finalized their block
        try expect(temp.isEmpty());
        try expectRecovery(scene, &snapshot, &reference, &temp, jobs.jobSystem());
    }
}

fn outOfHeapMemoryInCCDCast(kind: JobSystemKind) !void {
    const allocator = std.testing.allocator;
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    // The system owns the custom broad phase after a successful init
    const scene = try allocator.create(Scene);
    defer allocator.destroy(scene);
    const broad_phase = try AllocatingBroadPhase.create(allocator);
    scene.initSystem(allocator, .{ .broad_phase = .{ .custom = &broad_phase.base } }) catch |err| {
        broad_phase.base.base.destroy();
        return err;
    };
    defer scene.deinit();
    try scene.populate(allocator);
    for (0..5) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    var snapshot = StateRecorderImpl.init(allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .{});
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    var reference = StateRecorderImpl.init(allocator);
    defer reference.deinit();
    try scene.system.saveState(reference.recorder(), .{});

    // The casts of the bullets fail: the FindCCDContacts jobs latch the error, the update returns it
    snapshot.rewind();
    try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
    broad_phase.fail_cast.store(true, .monotonic);
    try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem()));
    broad_phase.fail_cast.store(false, .monotonic);
    try expect(temp.isEmpty());
    try expectRecovery(scene, &snapshot, &reference, &temp, jobs.jobSystem());
}

test "PhysicsSystem out of heap memory inside a job (BroadPhase.castAABoxNoLock), single threaded" {
    try outOfHeapMemoryInCCDCast(.single_threaded);
}

test "PhysicsSystem out of heap memory inside a job (BroadPhase.castAABoxNoLock), multi threaded" {
    try outOfHeapMemoryInCCDCast(.{ .thread_pool = 3 });
}

test "PhysicsSystem out of heap memory inside a job (BroadPhaseQuadTree.updatePrepare)" {
    // The default broad phase allocates the node IDs and BuildTree's centers in the UpdateBroadPhasePrepare job
    var heap: FailingHeapAllocator = .{ .inner = std.testing.allocator };
    const allocator = heap.allocator();
    var jobs: AnyJobSystem = .{};
    try jobs.init(std.testing.allocator, .{ .thread_pool = 3 });
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(std.testing.allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    const scene = try std.testing.allocator.create(Scene);
    defer std.testing.allocator.destroy(scene);
    try scene.init(allocator, .{});
    defer scene.deinit();
    for (0..5) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    var snapshot = StateRecorderImpl.init(std.testing.allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .{});

    // Reference: count the heap allocations of one update (the first step rebuilds the tree of the moving bodies, the
    // second step cannot update it again before the next FrameSync discards its old tree)
    const before = heap.num_allocations.load(.monotonic);
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    const num_allocations = heap.num_allocations.load(.monotonic) - before;
    try expect(num_allocations >= 2); // The node IDs and the centers
    var reference = StateRecorderImpl.init(std.testing.allocator);
    defer reference.deinit();
    try scene.system.saveState(reference.recorder(), .{});

    for (0..num_allocations) |i| {
        snapshot.rewind();
        try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
        heap.fail_index.store(heap.num_allocations.load(.monotonic) + @as(u32, @intCast(i)), .monotonic);
        try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem()));
        heap.fail_index.store(std.math.maxInt(u32), .monotonic);
        try expect(temp.isEmpty());
        try expectRecovery(scene, &snapshot, &reference, &temp, jobs.jobSystem());
    }
}

test "PhysicsSystem with the brute force broad phase gives the results of the quad tree" {
    // The broad phase does not change the results (Jolt's determinism comes from the contact sorts)
    const allocator = std.testing.allocator;
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, .single_threaded);
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    var states: [2]StateRecorderImpl = .{ .init(allocator), .init(allocator) };
    defer for (&states) |*s| s.deinit();
    for ([_]PhysicsSystem.BroadPhaseOption{ .quad_tree, .brute_force }, &states) |option, *state| {
        const scene = try allocator.create(Scene);
        defer allocator.destroy(scene);
        try scene.init(allocator, .{ .broad_phase = option });
        defer scene.deinit();
        try scene.system.optimizeBroadPhase();
        for (0..10) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
        try scene.system.saveState(state.recorder(), .{});
    }
    try expect(states[0].isEqual(&states[1]));
}

test "PhysicsSystem BodyInterface setObjectLayer out of memory changes nothing" {
    const allocator = std.testing.allocator;
    var failing: std.testing.FailingAllocator = .init(allocator, .{});
    const scene = try allocator.create(Scene);
    defer allocator.destroy(scene);
    try scene.init(failing.allocator(), .{});
    defer scene.deinit();
    const bi = scene.system.getBodyInterfaceMut();

    // The quad tree moves the body to the tree of its new broad phase layer: the layer state and BuildTree's centers
    var k: usize = 0;
    while (true) : (k += 1) {
        failing.fail_index = failing.alloc_index + k;
        const result = bi.setObjectLayer(scene.kinematic_body, Layers.moving2);
        failing.fail_index = std.math.maxInt(usize);
        if (result) |_| break else |err| {
            try expectEqual(error.OutOfMemory, err);
            try expectEqual(Layers.moving, bi.getObjectLayer(scene.kinematic_body));
            try expect(bi.isAdded(scene.kinematic_body));
        }
    }
    try expect(k >= 2);
    try expectEqual(Layers.moving2, bi.getObjectLayer(scene.kinematic_body));
    try expect(bi.isAdded(scene.kinematic_body));
}

test "PhysicsSystem out of memory creating the scene" {
    // Every allocation of scene creation fails in turn (system, shapes, bodies, AddBodiesPrepare, constraints,
    // listeners): errors propagate, nothing leaks
    const Ctx = struct {
        fn build(allocator: Allocator) !void {
            const scene = try allocator.create(Scene);
            defer allocator.destroy(scene);
            try scene.init(allocator, .{});
            scene.deinit();
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, Ctx.build, .{});
}

test "PhysicsSystem out of memory with a custom broad phase" {
    // A failing init leaves the custom broad phase to the caller, afterwards the system owns it
    const Ctx = struct {
        fn build(allocator: Allocator) !void {
            const scene = try allocator.create(Scene);
            defer allocator.destroy(scene);
            const broad_phase = try AllocatingBroadPhase.create(allocator);
            scene.initSystem(allocator, .{ .broad_phase = .{ .custom = &broad_phase.base } }) catch |err| {
                try expect(scene.system.broad_phase == null); // The system is `empty` again
                broad_phase.base.base.destroy();
                return err;
            };
            defer scene.deinit();
            try scene.populate(allocator);
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, Ctx.build, .{});
}

test "PhysicsSystem BodyInterface ID allocation like Jolt" {
    const allocator = std.testing.allocator;
    var scene: Scene = .{};
    try scene.system.init(allocator, io, 4, 0, 16, 16, &scene.broad_phase_layer_interface.base, &scene.object_vs_broad_phase_layer_filter.base, &scene.object_layer_pair_filter.base, .{});
    defer scene.system.deinit();
    const bi = scene.system.getBodyInterfaceMut();

    const box = try BoxShape.create(allocator, Vec3.replicate(1), .{});
    var settings = BodyCreationSettings.initWithShape(box.asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();

    // Sequential allocation: index 0..3, sequence number 1
    var ids: [4]BodyID = undefined;
    for (&ids, 0..) |*id, i| {
        id.* = try bi.createAndAddBody(&settings, .activate);
        try expectEqual(@as(u32, @intCast(i)), id.getIndex());
        try expectEqual(@as(u8, 1), id.getSequenceNumber());
    }

    // Out of bodies: Jolt returns an invalid ID (createBody returns null)
    try expect((try bi.createAndAddBody(&settings, .activate)).isInvalid());
    try expect((try bi.createBody(&settings)) == null);

    // Freed slots are reused in LIFO order of the free list, with the next sequence number
    bi.removeBody(ids[1]);
    bi.destroyBody(ids[1]);
    bi.removeBody(ids[3]);
    bi.destroyBody(ids[3]);
    const reused1 = try bi.createAndAddBody(&settings, .activate);
    try expectEqual(@as(u32, 3), reused1.getIndex());
    try expectEqual(@as(u8, 2), reused1.getSequenceNumber());
    const reused2 = try bi.createAndAddBody(&settings, .activate);
    try expectEqual(@as(u32, 1), reused2.getIndex());
    try expectEqual(@as(u8, 2), reused2.getSequenceNumber());

    // A stale ID no longer locks a body
    {
        var lock: BodyLockRead = .init(scene.system.getBodyLockInterface(), ids[1]);
        defer lock.deinit();
        try expect(!lock.succeeded());
    }

    // CreateBodyWithID takes the slot from the free list and establishes the sequence number
    bi.removeBody(reused2);
    bi.destroyBody(reused2);
    const custom = (try bi.createBodyWithID(.fromIndexAndSequenceNumber(1, 7), &settings)).?;
    try expectEqual(@as(u8, 7), custom.getID().getSequenceNumber());
    try expect((try bi.createBodyWithID(.fromIndexAndSequenceNumber(1, 8), &settings)) == null); // Slot in use
    bi.destroyBody(custom.getID());
    const after_custom = try bi.createAndAddBody(&settings, .activate);
    try expectEqual(@as(u32, 1), after_custom.getIndex());
    try expectEqual(@as(u8, 8), after_custom.getSequenceNumber());

    // No shape at all: error.InvalidShape (Jolt: an assert and a null body)
    var empty: BodyCreationSettings = .{};
    defer empty.deinit();
    try std.testing.expectError(error.InvalidShape, bi.createBody(&empty));
    try std.testing.expectError(error.InvalidShape, bi.createAndAddBody(&empty, .activate));
}

/// The BodyInterface functions that flag the contact cache invalid (an allocation in
/// BodyManager.invalidateContactCacheForBody) on the first body of a fresh system: the first call fails, it changes
/// nothing and repeating it gives the same body as the same call on an identical body without a failure
fn testContactCacheFailure(comptime Call: type) !void {
    const allocator = std.testing.allocator;
    var failing: std.testing.FailingAllocator = .init(allocator, .{});
    var scene: Scene = .{};
    try scene.system.init(failing.allocator(), io, 4, 0, 16, 16, &scene.broad_phase_layer_interface.base, &scene.object_vs_broad_phase_layer_filter.base, &scene.object_layer_pair_filter.base, .{});
    defer scene.system.deinit();
    const bi = scene.system.getBodyInterfaceMut();

    // Two identical sleeping boxes in the broad phase
    const box = try BoxShape.create(allocator, Vec3.init(0.5, 0.75, 1.0), .{});
    var settings = BodyCreationSettings.initWithShape(box.asShape(), RVec3.init(1, 2, 3), Quat.rotation(Vec3.init(0, 1, 0), 0.3), .dynamic, Layers.moving);
    defer settings.deinit();
    const failed = (try bi.createBody(&settings)).?;
    try bi.addBody(failed.getID(), .dont_activate);
    const reference = (try bi.createBody(&settings)).?;
    try bi.addBody(reference.getID(), .dont_activate);
    const cache_invalid = &scene.system.body_manager.bodies_cache_invalid;
    try expectEqual(@as(usize, 0), cache_invalid.capacity); // The next flag allocates

    // The failed call changes nothing
    failing.fail_index = failing.alloc_index;
    try std.testing.expectError(error.OutOfMemory, Call.run(bi, failed.getID()));
    failing.fail_index = std.math.maxInt(usize);
    try expectSameBody(failed, reference);
    try expect(!failed.isCollisionCacheInvalid());
    try expectEqual(@as(usize, 0), cache_invalid.items.len);

    // Repeating it gives the result of a call that did not fail
    try Call.run(bi, failed.getID());
    try Call.run(bi, reference.getID());
    try expectSameBody(failed, reference);
    try expect(failed.isCollisionCacheInvalid());
    try expectEqual(@as(usize, 2), cache_invalid.items.len);
    try expect(cache_invalid.items[0].eql(failed.getID()) and cache_invalid.items[1].eql(reference.getID()));
    try Call.check(failed);
}

/// Everything that the contact cache functions change on a body (the bodies have different IDs)
fn expectSameBody(a: *const Body, b: *const Body) !void {
    try expect(a.getShape() == b.getShape());
    try expect(a.getCenterOfMassPosition().eql(b.getCenterOfMassPosition()));
    try expect(a.getRotation().eql(b.getRotation()));
    try expect(a.getWorldSpaceBounds().min.eql(b.getWorldSpaceBounds().min));
    try expect(a.getWorldSpaceBounds().max.eql(b.getWorldSpaceBounds().max));
    try expectEqual(a.getMotionProperties().getInverseMass(), b.getMotionProperties().getInverseMass());
    try expect(a.getMotionProperties().getInverseInertiaDiagonal().eql(b.getMotionProperties().getInverseInertiaDiagonal()));
    try expectEqual(a.isActive(), b.isActive());
    try expectEqual(a.isCollisionCacheInvalid(), b.isCollisionCacheInvalid());
    try expectEqual(a.getUseManifoldReduction(), b.getUseManifoldReduction());
}

test "PhysicsSystem BodyInterface setShape out of memory can be repeated" {
    const sphere = try zolt.SphereShape.create(std.testing.allocator, 0.7, .{});
    sphere.asShape().addRef();
    defer sphere.asShape().release();
    const Call = struct {
        var shape: *const zolt.Shape = undefined;
        fn run(bi: *BodyInterface, id: BodyID) !void {
            try bi.setShape(id, shape, true, .activate);
        }
        fn check(body: *const Body) !void {
            // The shape and the mass changed, the body woke up
            try expect(body.getShape() == shape);
            try expect(body.isActive());
            try expectEqual(1.0 / shape.getMassProperties().mass, body.getMotionProperties().getInverseMass());
        }
    };
    Call.shape = sphere.asShape();
    try testContactCacheFailure(Call);
}

test "PhysicsSystem BodyInterface notifyShapeChanged out of memory can be repeated" {
    const Call = struct {
        fn run(bi: *BodyInterface, id: BodyID) !void {
            // The center of mass of the shape moved from (0.25, 0, 0) to the origin: the center of mass of the body
            // moves along the body's x axis
            try bi.notifyShapeChanged(id, Vec3.init(0.25, 0, 0), true, .activate);
        }
        fn check(body: *const Body) !void {
            try expect(body.isActive());
            const expected = RVec3.init(1, 2, 3).addVec3(Quat.rotation(Vec3.init(0, 1, 0), 0.3).mulVec3(Vec3.init(-0.25, 0, 0)));
            try expect(body.getCenterOfMassPosition().eql(expected));
        }
    };
    try testContactCacheFailure(Call);
}

test "PhysicsSystem BodyInterface setUseManifoldReduction out of memory can be repeated" {
    const Call = struct {
        fn run(bi: *BodyInterface, id: BodyID) !void {
            try bi.setUseManifoldReduction(id, false);
        }
        fn check(body: *const Body) !void {
            try expect(!body.getUseManifoldReduction());
            try expect(!body.isActive());
        }
    };
    try testContactCacheFailure(Call);
}

test "PhysicsSystem BodyInterface invalidateContactCache out of memory can be repeated" {
    const Call = struct {
        fn run(bi: *BodyInterface, id: BodyID) !void {
            try bi.invalidateContactCache(id);
        }
        fn check(body: *const Body) !void {
            try expect(!body.isActive());
        }
    };
    try testContactCacheFailure(Call);
}

test "PhysicsSystem BodyInterface locks, threads, the batch API and a const broad phase query" {
    const allocator = std.testing.allocator;
    var scene: Scene = .{};
    try scene.system.init(allocator, io, 64, 8, 16, 16, &scene.broad_phase_layer_interface.base, &scene.object_vs_broad_phase_layer_filter.base, &scene.object_layer_pair_filter.base, .{});
    defer scene.system.deinit();
    const system = &scene.system;
    const bi = system.getBodyInterfaceMut();

    const box = try BoxShape.create(allocator, Vec3.replicate(1), .{});
    var settings = BodyCreationSettings.initWithShape(box.asShape(), RVec3.init(1, 2, 3), Quat.identity(), .dynamic, Layers.moving);
    defer settings.deinit();
    const id = try bi.createAndAddBody(&settings, .dont_activate);
    try expect(!bi.isActive(id));

    {
        var lock: BodyLockWrite = .init(system.getBodyLockInterface(), id);
        defer lock.deinit();
        try expect(lock.succeededAndIsInBroadPhase());
        lock.getBody().setUserData(42);
        // The body's mutex is held exclusively
        try expect(system.body_manager.getMutexForBody(id).isLocked());
    }
    try expect(!system.body_manager.getMutexForBody(id).isLocked());
    {
        var lock: BodyLockRead = .init(system.getBodyLockInterfaceNoLock(), id);
        defer lock.deinit();
        try expectEqual(@as(u64, 42), lock.getBody().getUserData());
        try expect(!system.body_manager.getMutexForBody(id).isLocked());
    }
    try expectEqual(@as(u64, 42), bi.getUserData(id));

    // getShape returns a reference the caller owns
    {
        var shape = bi.getShape(id);
        defer shape.deinit();
        try expect(shape.get().? == box.asShape());
    }

    // addForce with Jolt's defaulted activation argument as an option
    bi.addForce(id, Vec3.init(0, 1, 0), .{ .activation = .dont_activate });
    try expect(!bi.isActive(id)); // Ignored: the body is not active and may not be activated
    bi.addForce(id, Vec3.init(0, 1, 0), .{});
    try expect(bi.isActive(id));

    // Concurrent readers and writers through the locking body interface
    const Worker = struct {
        fn run(body_interface: *BodyInterface, body_id: BodyID, n: u32) void {
            for (0..n) |_| {
                body_interface.addLinearVelocity(body_id, Vec3.init(1, 0, 0));
                _ = body_interface.getLinearVelocity(body_id);
            }
        }
    };
    var threads: [4]std.Thread = undefined;
    for (&threads) |*t| t.* = try std.Thread.spawn(.{}, Worker.run, .{ bi, id, 100 });
    for (threads) |t| t.join();
    try expectEqual(@as(f32, 400), bi.getLinearVelocity(id).getX());

    // The batch API: prepare can fail (allocates in the broad phase), finalize / abort cannot
    var batch = [_]BodyID{ try bi.createAndAddBody(&settings, .dont_activate), .invalid };
    bi.removeBody(batch[0]);
    batch[1] = (try bi.createBody(&settings)).?.getID();
    const add_state = try bi.addBodiesPrepare(&batch);
    bi.addBodiesFinalize(&batch, add_state, .activate);
    try expect(bi.isAdded(batch[0]) and bi.isAdded(batch[1]) and bi.isActive(batch[1]));
    var aborted = [_]BodyID{(try bi.createBody(&settings)).?.getID()};
    const abort_state = try bi.addBodiesPrepare(&aborted);
    bi.addBodiesAbort(&aborted, abort_state); // Leak checked
    try expect(!bi.isAdded(aborted[0]));

    // A const broad phase query (Rule M: the query locks are behind `mutable_fields`)
    const query = system.getBroadPhaseQuery();
    var collector = AllHitCollisionCollector(CollideShapeBodyCollector).init(allocator);
    defer collector.deinit();
    query.collideAABox(&AABox.init(Vec3.init(0, 1, 2), Vec3.init(2, 3, 4)), &collector.base, .{});
    try collector.checkError();
    try expectEqual(@as(usize, 3), collector.hits.items.len); // id, batch[0] and batch[1] (same box at the same place)

    // getBodies / getActiveBodies
    var bodies: zolt.BodyIDVector = .empty;
    defer bodies.deinit(allocator);
    try system.getBodies(allocator, &bodies);
    try expectEqual(@as(usize, 4), bodies.items.len);
    try system.getActiveBodies(allocator, .rigid_body, &bodies);
    try expectEqual(@as(usize, 3), bodies.items.len); // id, batch[0] and batch[1] (AddBodiesFinalize activates the batch)
    try expectEqual(@as(u32, 3), system.getNumActiveBodies(.rigid_body));
}

test "PhysicsSystem activation listener: BodyInterface activation and falling asleep" {
    // TestPhysicsActivationDeactivation without the floor (the test with the floor belongs to PhysicsTests): without
    // gravity the activated box stays at rest and falls asleep
    var c: PhysicsTestContext = undefined;
    try c.init(std.testing.allocator, .{});
    defer c.deinit();
    c.zeroGravity();

    var listener: LoggingBodyActivationListener = .init(std.testing.allocator);
    defer listener.deinit();
    c.getSystem().setBodyActivationListener(.init(&listener));

    const box = try c.createBox(RVec3.init(0, 5, 0), Quat.identity(), .dynamic, .discrete, Layers.moving, Vec3.replicate(0.5), .{ .activation = .dont_activate });
    try expect(!box.isActive());

    // Box should not activate by itself
    _ = try c.simulate(1.0, {});
    try expect(box.getPosition().eql(RVec3.init(0, 5, 0)));
    try expect(!box.isActive());
    try expectEqual(@as(usize, 0), listener.getEntryCount());

    // Activate the body and validate it is active now
    c.getBodyInterface().activateBody(box.getID());
    try expect(box.isActive());
    try expectEqual(@as(usize, 1), listener.getEntryCount());
    try expect(listener.contains(.activated, box.getID()));
    listener.clear();

    // It falls asleep after PhysicsSettings.time_before_sleep and does not move
    _ = try c.simulate(0.4, {});
    try expect(box.isActive());
    _ = try c.simulate(0.2, {});
    try expect(!box.isActive());
    try expect(box.getPosition().eql(RVec3.init(0, 5, 0)));
    try expectEqual(@as(usize, 1), listener.getEntryCount());
    try expect(listener.contains(.deactivated, box.getID()));
    try expect(!listener.contains(.activated, box.getID()));
    try listener.checkError();

    // A log entry that cannot be stored is reported by checkError
    var failing_allocator: std.testing.FailingAllocator = .init(std.testing.allocator, .{ .fail_index = 0 });
    var failing_listener: LoggingBodyActivationListener = .init(failing_allocator.allocator());
    defer failing_listener.deinit();
    c.getSystem().setBodyActivationListener(.init(&failing_listener));
    c.getBodyInterface().activateBody(box.getID());
    try std.testing.expectError(error.OutOfMemory, failing_listener.checkError());
    c.getSystem().setBodyActivationListener(null);
}

test "PhysicsSystem linear cast body without inner radius (Jolt's release behavior)" {
    // A dynamic mesh or a mirrored ScaledShape with the linear cast motion quality has a zero or negative inner
    // radius. Jolt asserts (a usage error), its release build compares the squared movement of a step with
    // Square(linear_cast_threshold * inner_radius): a zero radius casts every movement, a negative radius r every
    // movement longer than linear_cast_threshold * |r|. Zolt's ReleaseFast build does the same instead of reaching
    // undefined behavior (the assert in jobIntegrateVelocity was unconditional). Nothing is in the way, so the bodies
    // move exactly like their discrete twins. (SimulationParity compares this with Jolt in the scene
    // linear_cast_without_inner_radius.)
    if (Core.enable_asserts) return error.SkipZigTest;

    const allocator = std.testing.allocator;
    var c: PhysicsTestContext = undefined;
    try c.init(allocator, .{ .collision_steps = 2 });
    defer c.deinit();

    const mesh = try zolt.MeshShapeSettings.create(allocator, &.{zolt.Triangle.init(Vec3.zero(), Vec3.init(1, 0, 0), Vec3.init(0, 0, 1), .{})}, .{});
    var mesh_ref = Ref(zolt.ShapeSettings).init(mesh.asShapeSettings());
    defer mesh_ref.deinit();
    const mirrored = try zolt.ScaledShapeSettings.create(allocator, (try SphereShapeSettings.create(allocator, 0.5, .{})).asShapeSettings(), Vec3.replicate(-1.0));
    var mirrored_ref = Ref(zolt.ShapeSettings).init(mirrored.asShapeSettings());
    defer mirrored_ref.deinit();

    const speed: f32 = 60.0;
    var bodies: [4]*Body = undefined;
    for (&bodies, 0..) |*body, i| {
        var settings: BodyCreationSettings = .{};
        defer settings.deinit();
        settings.setShapeSettings(if (i < 2) mesh.asShapeSettings() else mirrored.asShapeSettings());
        settings.position = RVec3.init(@floatFromInt(20 * i), 10, 0);
        settings.linear_velocity = Vec3.init(0, -speed, 0);
        settings.motion_quality = if (i % 2 == 0) .linear_cast else .discrete;
        settings.object_layer = Layers.moving;
        settings.override_mass_properties = .mass_and_inertia_provided;
        settings.mass_properties_override.mass = 2.0;
        settings.mass_properties_override.inertia = zolt.Mat44.identity();
        body.* = try c.createBody(&settings, .activate);
    }
    try expect(bodies[0].getShape().getInnerRadius() == 0.0);
    try expect(bodies[2].getShape().getInnerRadius() < 0.0);

    // Both linear cast bodies take the cast path: every step moves at least speed * step delta time (gravity only adds
    // to it), the negative radius needs more than 0.75 * 0.5 m per step (45 m/s)
    const step_movement = speed * c.getStepDeltaTime();
    for ([_]*Body{ bodies[0], bodies[2] }) |body| {
        const threshold = c.getSystem().getPhysicsSettings().linear_cast_threshold * body.getShape().getInnerRadius();
        try expect(step_movement * step_movement > threshold * threshold);
    }

    _ = try c.simulate(0.5, {});
    for (0..2) |pair| {
        const cast = bodies[2 * pair].getPosition();
        const discrete = bodies[2 * pair + 1].getPosition();
        try expect(cast.getY() < 0.0); // It moved
        try expectEqual(discrete.getY(), cast.getY());
        try expectEqual(discrete.getZ(), cast.getZ());
    }
}

// -------------------------------------------------------------------------------------------------------------------
// Contact scenes (Wave B: ProcessBodyPair, TrySpawnJobFindCollisions, the island solve, the LargeIslandSplitter)

/// Counts the contact callbacks and hashes the order in which they arrive (called from the worker threads: atomics)
const ContactOrderListener = struct {
    num_added: std.atomic.Value(u32) = .init(0),
    num_persisted: std.atomic.Value(u32) = .init(0),
    num_removed: std.atomic.Value(u32) = .init(0),
    /// Order dependent hash of the callbacks as they arrive: it differs between multi threaded runs (Jolt's contact
    /// order is not deterministic), the simulation results do not
    order_hash: std.atomic.Value(u64) = .init(0),

    fn mix(self: *ContactOrderListener, key: u64) void {
        var old = self.order_hash.load(.monotonic);
        while (self.order_hash.cmpxchgWeak(old, (old *% 0x100000001b3) ^ key, .monotonic, .monotonic)) |actual| old = actual;
    }

    fn pairKey(body1: *const Body, body2: *const Body, manifold: *const zolt.ContactManifold) u64 {
        return (@as(u64, body1.getID().getIndexAndSequenceNumber()) << 40) ^ (@as(u64, body2.getID().getIndexAndSequenceNumber()) << 16) ^ manifold.sub_shape_id2.getValue();
    }

    pub fn onContactAdded(self: *ContactOrderListener, body1: *const Body, body2: *const Body, manifold: *const zolt.ContactManifold, settings: *zolt.ContactSettings) void {
        _ = settings;
        _ = self.num_added.fetchAdd(1, .monotonic);
        self.mix(pairKey(body1, body2, manifold));
    }

    pub fn onContactPersisted(self: *ContactOrderListener, body1: *const Body, body2: *const Body, manifold: *const zolt.ContactManifold, settings: *zolt.ContactSettings) void {
        _ = settings;
        _ = self.num_persisted.fetchAdd(1, .monotonic);
        self.mix(pairKey(body1, body2, manifold) +% 1);
    }

    pub fn onContactRemoved(self: *ContactOrderListener, sub_shape_pair: *const zolt.SubShapeIDPair) void {
        _ = self.num_removed.fetchAdd(1, .monotonic);
        self.mix((@as(u64, sub_shape_pair.getBody1ID().getIndexAndSequenceNumber()) << 32) | sub_shape_pair.getBody2ID().getIndexAndSequenceNumber());
    }
};

/// Wraps a job system and counts the FindCollisions jobs that are spawned from inside jobs (TrySpawnJobFindCollisions
/// creates them without dependencies, the job graph of `update` creates its FindCollisions jobs with dependencies)
const SpawnCountingJobSystem = struct {
    inner: JobSystem,
    num_spawned_find_collisions: std.atomic.Value(u32) = .init(0),

    fn jobSystem(self: *SpawnCountingJobSystem) JobSystem {
        return .init(self);
    }

    pub fn getMaxConcurrency(self: *const SpawnCountingJobSystem) i32 {
        return self.inner.getMaxConcurrency();
    }

    pub fn createJob(self: *SpawnCountingJobSystem, job_name: []const u8, color: zolt.Color, job_function: JobSystem.JobFunction, opts: struct { num_dependencies: u32 = 0 }) error{OutOfMemory}!JobSystem.JobHandle {
        const handle = try self.inner.createJob(job_name, color, job_function, .{ .num_dependencies = opts.num_dependencies });
        if (opts.num_dependencies == 0 and std.mem.eql(u8, job_name, "FindCollisions"))
            _ = self.num_spawned_find_collisions.fetchAdd(1, .monotonic);
        return handle;
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

    // The jobs belong to `inner` and queue themselves there: not reached
    pub fn queueJob(self: *SpawnCountingJobSystem, job: *JobSystem.Job) void {
        self.inner.vtable.queueJob(self.inner.ptr, job);
    }

    pub fn queueJobs(self: *SpawnCountingJobSystem, jobs: []const *JobSystem.Job) void {
        self.inner.vtable.queueJobs(self.inner.ptr, jobs);
    }

    pub fn freeJob(self: *SpawnCountingJobSystem, job: *JobSystem.Job) void {
        self.inner.vtable.freeJob(self.inner.ptr, job);
    }
};

/// A scene with contacts: a dense grid of spheres in two layers on a floor (few batches of active bodies with many body
/// pairs each, so TrySpawnJobFindCollisions spawns FindCollisions jobs inside jobs when there are threads), a sleeping
/// box woken up by a falling ball, a ball falling through a sensor, a kinematic box pushing a dynamic box, and a large
/// box with enhanced internal edge removal on a fine mesh (the InternalEdgeRemovingCollector outgrows its local
/// buffers: a heap allocation inside the FindCollisions jobs)
const ContactScene = struct {
    broad_phase_layer_interface: LayersFile.BPLayerInterfaceImpl = .init(),
    object_vs_broad_phase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_layer_pair_filter: LayersFile.ObjectLayerPairFilterImpl = .{},
    system: PhysicsSystem = .empty,
    activation_listener: ActivationCounter = .{},
    contact_listener: ContactOrderListener = .{},
    sleeping_box: BodyID = .invalid,
    sensor_ball: BodyID = .invalid,
    pushed_box: BodyID = .invalid,
    big_box: BodyID = .invalid,

    const Options = struct {
        /// The large box on the mesh uses enhanced internal edge removal
        enhanced_internal_edge_removal: bool = true,
        /// The large box on the mesh uses manifold reduction (without it NonReductionCollideShapeCollector adds every
        /// hit to the contact constraints at once)
        use_manifold_reduction: bool = true,
    };

    /// Builds the scene in place (the physics system must not move)
    fn init(self: *ContactScene, allocator: Allocator, opts: Options) !void {
        self.* = .{};
        try self.system.init(allocator, io, 1024, 0, 8192, 2048, &self.broad_phase_layer_interface.base, &self.object_vs_broad_phase_layer_filter.base, &self.object_layer_pair_filter.base, .{});
        errdefer self.system.deinit();
        var settings = self.system.getPhysicsSettings().*;
        settings.time_before_sleep = 0.2;
        self.system.setPhysicsSettings(&settings);
        self.system.setBodyActivationListener(.init(&self.activation_listener));
        self.system.setContactListener(.init(&self.contact_listener));
        const bi = self.system.getBodyInterfaceMut();

        // Floor, its top at y = 0
        const floor_shape = try BoxShape.create(allocator, Vec3.init(60, 1, 60), .{ .convex_radius = 0.0 });
        var floor_settings = BodyCreationSettings.initWithShape(floor_shape.asShape(), RVec3.init(0, -1, 0), Quat.identity(), .static, Layers.non_moving);
        defer floor_settings.deinit();
        try expect(!(try bi.createAndAddBody(&floor_settings, .dont_activate)).isInvalid());

        // A dense grid of spheres in two layers, never asleep
        var sphere_settings = BodyCreationSettings.initWithSettings((try SphereShapeSettings.create(allocator, 0.5, .{})).asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
        defer sphere_settings.deinit();
        sphere_settings.allow_sleeping = false;
        for (0..2) |layer| {
            for (0..4) |row| {
                for (0..8) |column| {
                    const l = f(@intCast(layer));
                    sphere_settings.position = RVec3.init(f(@intCast(column)) * 0.97 - 3.5 + 0.48 * l, 0.5 + 0.8 * l, f(@intCast(row)) * 0.97 - 1.5 + 0.48 * l);
                    try expect(!(try bi.createAndAddBody(&sphere_settings, .activate)).isInvalid());
                }
            }
        }
        sphere_settings.allow_sleeping = true;

        // A sleeping box and a ball that falls on it: the contact wakes the box up
        var box_settings = BodyCreationSettings.initWithSettings((try BoxShapeSettings.create(allocator, Vec3.replicate(0.5), .{})).asShapeSettings(), RVec3.init(12, 0.5, 0), Quat.identity(), .dynamic, Layers.moving);
        defer box_settings.deinit();
        self.sleeping_box = try bi.createAndAddBody(&box_settings, .dont_activate);
        sphere_settings.position = RVec3.init(12.2, 2.2, 0.1);
        _ = try bi.createAndAddBody(&sphere_settings, .activate);

        // A static sensor and a ball that falls through it
        const sensor_shape = try BoxShape.create(allocator, Vec3.init(2, 0.5, 2), .{ .convex_radius = 0.0 });
        var sensor_settings = BodyCreationSettings.initWithShape(sensor_shape.asShape(), RVec3.init(-12, 1.2, 0), Quat.identity(), .static, Layers.sensor);
        defer sensor_settings.deinit();
        sensor_settings.is_sensor = true;
        _ = try bi.createAndAddBody(&sensor_settings, .dont_activate);
        sphere_settings.position = RVec3.init(-12, 2.4, 0);
        sphere_settings.linear_velocity = Vec3.init(0, -3, 0);
        self.sensor_ball = try bi.createAndAddBody(&sphere_settings, .activate);
        sphere_settings.linear_velocity = Vec3.zero();

        // A kinematic box that pushes a dynamic box
        box_settings.motion_type = .kinematic;
        box_settings.position = RVec3.init(0, 0.55, 10);
        box_settings.linear_velocity = Vec3.init(1.5, 0, 0);
        _ = try bi.createAndAddBody(&box_settings, .activate);
        box_settings.motion_type = .dynamic;
        box_settings.linear_velocity = Vec3.zero();
        box_settings.position = RVec3.init(1.2, 0.5, 10);
        self.pushed_box = try bi.createAndAddBody(&box_settings, .activate);

        // A fine static mesh with a large rotated box on it (enhanced internal edge removal: many triangles under the
        // box, more delayed results and voided features than the collector holds without the heap)
        var triangles: std.ArrayList(zolt.Triangle) = .empty;
        defer triangles.deinit(allocator);
        const cell: f32 = 0.25;
        for (0..24) |iz| {
            for (0..24) |ix| {
                const x0 = f(@intCast(ix)) * cell - 3.0;
                const z0 = f(@intCast(iz)) * cell - 3.0;
                const v00 = Vec3.init(x0, 0, z0);
                const v10 = Vec3.init(x0 + cell, 0, z0);
                const v01 = Vec3.init(x0, 0, z0 + cell);
                const v11 = Vec3.init(x0 + cell, 0, z0 + cell);
                try triangles.append(allocator, .init(v00, v01, v11, .{}));
                try triangles.append(allocator, .init(v00, v11, v10, .{}));
            }
        }
        var mesh_settings = BodyCreationSettings.initWithSettings((try zolt.MeshShapeSettings.create(allocator, triangles.items, .{})).asShapeSettings(), RVec3.init(0, 0.5, -15), Quat.identity(), .static, Layers.non_moving);
        defer mesh_settings.deinit();
        _ = try bi.createAndAddBody(&mesh_settings, .dont_activate);
        box_settings.setShapeSettings((try BoxShapeSettings.create(allocator, Vec3.init(2, 0.25, 2), .{})).asShapeSettings());
        box_settings.position = RVec3.init(0.1, 0.75, -15.05); // Resting on the mesh (its top at y = 0.5)
        box_settings.rotation = Quat.rotation(Vec3.init(0, 1, 0), 0.3);
        box_settings.linear_velocity = Vec3.init(0.5, 0, 0.2);
        box_settings.enhanced_internal_edge_removal = opts.enhanced_internal_edge_removal;
        box_settings.use_manifold_reduction = opts.use_manifold_reduction;
        self.big_box = try bi.createAndAddBody(&box_settings, .activate);
    }

    fn deinit(self: *ContactScene) void {
        self.system.deinit();
    }
};

const contact_num_updates = 40;

/// What a run of the contact scene did
const ContactRunStats = struct {
    order_hash: u64,
    num_spawned_jobs: u32,
    /// Added, persisted and removed contact callbacks
    counts: [3]u32,
};

/// Runs the contact scene and records the saved state after every update
fn runContactScene(allocator: Allocator, kind: JobSystemKind, states: *std.ArrayList(u8), offsets: *std.ArrayList(usize)) !ContactRunStats {
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit();
    var counting: SpawnCountingJobSystem = .{ .inner = jobs.jobSystem() };

    var temp: TempAllocatorImpl = try .init(allocator, 8 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(ContactScene);
    defer allocator.destroy(scene);
    try scene.init(allocator, .{});
    defer scene.deinit();

    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();

    const bi = scene.system.getBodyInterface();
    const pushed_start_x = bi.getPosition(scene.pushed_box).getX();
    var woken = false;
    for (0..contact_num_updates) |_| {
        const errors = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), counting.jobSystem());
        try expect(errors.isNone());
        try expect(temp.isEmpty());
        woken = woken or bi.isActive(scene.sleeping_box);

        recorder.clear();
        try scene.system.saveState(recorder.recorder(), .{});
        try expect(!recorder.isFailed());
        try offsets.append(allocator, states.items.len);
        try states.appendSlice(allocator, recorder.getData());
    }

    // Sanity: the ball woke the sleeping box up, the ball fell through the sensor onto the floor, the kinematic box
    // pushed the dynamic box, contacts were added, persisted and removed
    try expect(woken);
    try expect(bi.getPosition(scene.sensor_ball).getY() < 0.6);
    try expect(bi.getPosition(scene.pushed_box).getX() > pushed_start_x + 0.5);
    const listener = &scene.contact_listener;
    const counts = [3]u32{ listener.num_added.load(.monotonic), listener.num_persisted.load(.monotonic), listener.num_removed.load(.monotonic) };
    try expect(counts[0] > 0 and counts[1] > 0 and counts[2] > 0);
    return .{ .order_hash = listener.order_hash.load(.monotonic), .num_spawned_jobs = counting.num_spawned_find_collisions.load(.monotonic), .counts = counts };
}

test "PhysicsSystem determinism with contacts and FindCollisions jobs spawned inside jobs" {
    // JobSystemSingleThreaded against thread pools: the saved state after every update is bit identical while the
    // order of the contact callbacks differs, and the dense grid makes TrySpawnJobFindCollisions spawn jobs from
    // inside the FindCollisions jobs
    const allocator = std.testing.allocator;
    var ref_states: std.ArrayList(u8) = .empty;
    defer ref_states.deinit(allocator);
    var ref_offsets: std.ArrayList(usize) = .empty;
    defer ref_offsets.deinit(allocator);
    const ref = try runContactScene(allocator, .single_threaded, &ref_states, &ref_offsets);
    try expectEqual(@as(u32, 0), ref.num_spawned_jobs); // One FindCollisions job, no room for more

    const kinds = [_]JobSystemKind{ .{ .thread_pool = 0 }, .{ .thread_pool = 1 }, .{ .thread_pool = 3 }, .{ .thread_pool = 7 }, .{ .thread_pool = 3 }, .{ .thread_pool = 7 }, .{ .thread_pool = 7 } };
    var num_spawned_jobs: u32 = 0;
    var num_different_orders: u32 = 0;
    for (kinds) |kind| {
        var states: std.ArrayList(u8) = .empty;
        defer states.deinit(allocator);
        var offsets: std.ArrayList(usize) = .empty;
        defer offsets.deinit(allocator);
        const stats = try runContactScene(allocator, kind, &states, &offsets);

        try std.testing.expectEqualSlices(usize, ref_offsets.items, offsets.items);
        for (ref_offsets.items, 0..) |offset, i| {
            const end = if (i + 1 < ref_offsets.items.len) ref_offsets.items[i + 1] else ref_states.items.len;
            if (!std.mem.eql(u8, ref_states.items[offset..end], states.items[offset..end])) {
                std.debug.print("state differs after update {d} with {any}\n", .{ i, kind });
                return error.TestUnexpectedResult;
            }
        }
        try expectEqual(ref.counts, stats.counts); // The same callbacks, in another order
        num_spawned_jobs += stats.num_spawned_jobs;
        if (stats.order_hash != ref.order_hash) num_different_orders += 1;
    }
    if (verbose) std.debug.print("contacts: spawned FindCollisions jobs {d}, runs with a different contact order {d} of {d}\n", .{ num_spawned_jobs, num_different_orders, kinds.len });
    try expect(num_spawned_jobs > 0);
    try expect(num_different_orders > 0);
}

test "PhysicsSystem contacts: save and restore the contact cache" {
    const allocator = std.testing.allocator;
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, .{ .thread_pool = 3 });
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(allocator, 8 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(ContactScene);
    defer allocator.destroy(scene);
    try scene.init(allocator, .{});
    defer scene.deinit();
    for (0..10) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    // Snapshot with a full contact cache, simulate, restore, simulate again: identical, also through a validating
    // restore
    var snapshot = StateRecorderImpl.init(allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .{});
    var contacts_only = StateRecorderImpl.init(allocator);
    defer contacts_only.deinit();
    try scene.system.saveState(contacts_only.recorder(), .{ .state = .{ .contacts = true } });
    try expect(contacts_only.getData().len > 1000); // The cache holds the manifolds of the grid

    var after1 = StateRecorderImpl.init(allocator);
    defer after1.deinit();
    for (0..10) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    try scene.system.saveState(after1.recorder(), .{});

    snapshot.rewind();
    try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
    var after2 = StateRecorderImpl.init(allocator);
    defer after2.deinit();
    for (0..10) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    try scene.system.saveState(after2.recorder(), .{});
    try expect(after2.isEqual(&after1));

    after2.rewind();
    after2.recorder().setValidating(true);
    try expect(try scene.system.restoreState(after2.recorder(), .{}));
    try expectEqual(@as(u32, 0), after2.num_mismatches);
}

/// Restore `snapshot`, run one update with enough memory and compare with `reference`
fn expectContactRecovery(scene: *ContactScene, snapshot: *StateRecorderImpl, reference: *StateRecorderImpl, temp: *TempAllocatorImpl, job_system: JobSystem) !void {
    snapshot.rewind();
    try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), job_system);
    try expect(temp.isEmpty());
    try expectContactState(scene, reference);
}

fn expectContactState(scene: *ContactScene, expected: *StateRecorderImpl) !void {
    var state = StateRecorderImpl.init(std.testing.allocator);
    defer state.deinit();
    try scene.system.saveState(state.recorder(), .{});
    try expect(state.isEqual(expected));
}

fn contactOutOfTempMemory(kind: JobSystemKind) !void {
    const allocator = std.testing.allocator;
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(allocator, 8 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(ContactScene);
    defer allocator.destroy(scene);
    try scene.init(allocator, .{});
    defer scene.deinit();
    for (0..5) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    var snapshot = StateRecorderImpl.init(allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .{});

    // Reference: one update with enough memory, counting the temp allocations
    var counting: FailingTempAllocator = .init(temp.tempAllocator(), .{});
    _ = try scene.system.update(delta_time, collision_steps, counting.tempAllocator(), jobs.jobSystem());
    var reference = StateRecorderImpl.init(allocator);
    defer reference.deinit();
    try scene.system.saveState(reference.recorder(), .{});
    const num_allocations = counting.num_allocations;
    try expect(num_allocations >= num_build_temp_allocations + collision_steps * 4);

    // Fail every allocation in turn: the update returns error.OutOfMemory, the temp allocator is empty (the frees at
    // Jolt's positions, also of the contact constraint buffer), the poisoned contact cache is discarded and restoring
    // the snapshot recovers the reference
    for (0..num_allocations) |fail_index| {
        snapshot.rewind();
        try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
        const added_before = scene.contact_listener.num_added.load(.monotonic);

        var failing: FailingTempAllocator = .init(temp.tempAllocator(), .{ .fail_index = @intCast(fail_index) });
        try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, failing.tempAllocator(), jobs.jobSystem()));
        try expect(failing.has_induced_failure);
        try expect(temp.isEmpty());

        // Failure while building the jobs: no job ran, no contact callback
        if (fail_index < num_build_temp_allocations) {
            try expectEqual(added_before, scene.contact_listener.num_added.load(.monotonic));
            try expectContactState(scene, &snapshot);
        }

        try expectContactRecovery(scene, &snapshot, &reference, &temp, jobs.jobSystem());
    }
}

test "PhysicsSystem contacts: out of temp memory at every allocation, single threaded" {
    try contactOutOfTempMemory(.single_threaded);
}

test "PhysicsSystem contacts: out of temp memory at every allocation, multi threaded" {
    try contactOutOfTempMemory(.{ .thread_pool = 3 });
}

fn contactFailingCreateJob(kind: JobSystemKind) !void {
    const allocator = std.testing.allocator;
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(allocator, 8 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(ContactScene);
    defer allocator.destroy(scene);
    try scene.init(allocator, .{});
    defer scene.deinit();
    for (0..5) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    var snapshot = StateRecorderImpl.init(allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .{});

    // Reference: one update, counting the jobs that the update builds (they are created before any job runs, the
    // start gate) and the FindCollisions jobs spawned from inside jobs (after them)
    var spawn_counting: SpawnCountingJobSystem = .{ .inner = jobs.jobSystem() };
    var counting: FailingJobSystem = .init(spawn_counting.jobSystem(), .{});
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), counting.jobSystem());
    var reference = StateRecorderImpl.init(allocator);
    defer reference.deinit();
    try scene.system.saveState(reference.recorder(), .{});
    const num_spawned = spawn_counting.num_spawned_find_collisions.load(.monotonic);
    const num_built = counting.num_created.load(.monotonic) - num_spawned;

    // Fail every createJob of the job graph in turn: the update fails without side effects
    for (0..num_built) |fail_index| {
        snapshot.rewind();
        try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
        const added_before = scene.contact_listener.num_added.load(.monotonic);
        var failing: FailingJobSystem = .init(jobs.jobSystem(), .{ .fail_index = @intCast(fail_index) });
        try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, temp.tempAllocator(), failing.jobSystem()));
        try expect(temp.isEmpty());
        try expectEqual(added_before, scene.contact_listener.num_added.load(.monotonic)); // No job ran
        try expectContactState(scene, &snapshot); // Nothing changed
        try expectContactRecovery(scene, &snapshot, &reference, &temp, jobs.jobSystem());
    }

    // A FindCollisions job that cannot be spawned from inside a job is not an error and changes no result (D5: the
    // claim and the dependencies are undone, the running jobs do the work). The number of spawned jobs differs
    // between runs: repeat until a spawn failed (the first ones of every run, then later ones).
    if (kind == .single_threaded) {
        try expectEqual(@as(u32, 0), num_spawned); // One FindCollisions job, no room for more
        return;
    }
    var num_spawn_failures: u32 = 0;
    var attempt: u32 = 0;
    while (attempt < 200 and num_spawn_failures < 8) : (attempt += 1) {
        snapshot.rewind();
        try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
        var failing: FailingJobSystem = .init(jobs.jobSystem(), .{ .fail_index = num_built + attempt % 4 });
        const errors = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), failing.jobSystem());
        try expect(errors.isNone());
        try expect(temp.isEmpty());
        if (failing.has_induced_failure.load(.monotonic)) num_spawn_failures += 1;
        try expectContactState(scene, &reference);
    }
    if (verbose) std.debug.print("{any}: built jobs {d}, spawned {d}, spawn failures {d} in {d} attempts\n", .{ kind, num_built, num_spawned, num_spawn_failures, attempt });
    try expect(num_spawn_failures > 0);
}

test "PhysicsSystem contacts: createJob fails at every job, single threaded" {
    try contactFailingCreateJob(.single_threaded);
}

test "PhysicsSystem contacts: createJob fails at every job and at spawned FindCollisions jobs, multi threaded" {
    try contactFailingCreateJob(.{ .thread_pool = 7 });
}

fn countHeapAllocationsOfUpdate(opts: ContactScene.Options) !u32 {
    var heap: FailingHeapAllocator = .{ .inner = std.testing.allocator };
    var jobs: AnyJobSystem = .{};
    try jobs.init(std.testing.allocator, .single_threaded);
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(std.testing.allocator, 8 * 1024 * 1024);
    defer temp.deinit();
    const scene = try std.testing.allocator.create(ContactScene);
    defer std.testing.allocator.destroy(scene);
    try scene.init(heap.allocator(), opts);
    defer scene.deinit();
    for (0..3) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    const before = heap.num_allocations.load(.monotonic);
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    return heap.num_allocations.load(.monotonic) - before;
}

fn contactOutOfHeapMemory(kind: JobSystemKind, opts: ContactScene.Options) !void {
    // The heap allocations of an update happen inside jobs: the QuadTree in UpdateBroadPhasePrepare and the
    // InternalEdgeRemovingCollector of the large box in ProcessBodyPair (the FindCollisions jobs). The heap runs out at
    // each of them in turn (that allocation and all later ones fail: a single failure of the collector is absorbed,
    // its growing arrays first try to remap and then allocate): the job latches the error (D5), the update returns
    // error.OutOfMemory, the temp allocator is empty and the system recovers through restoreState. With manifold
    // reduction the manifolds of the large box are not added; without it NonReductionCollideShapeCollector adds the
    // contacts forwarded before the failure to the contact constraints and the cache at once, the update fails the same
    // way.
    var without_collector = opts;
    without_collector.enhanced_internal_edge_removal = false;
    try expect(try countHeapAllocationsOfUpdate(opts) > try countHeapAllocationsOfUpdate(without_collector)); // The collector allocates

    var heap: FailingHeapAllocator = .{ .inner = std.testing.allocator };
    const allocator = heap.allocator();
    var jobs: AnyJobSystem = .{};
    try jobs.init(std.testing.allocator, kind);
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(std.testing.allocator, 8 * 1024 * 1024);
    defer temp.deinit();

    const scene = try std.testing.allocator.create(ContactScene);
    defer std.testing.allocator.destroy(scene);
    try scene.init(allocator, opts);
    defer scene.deinit();
    for (0..3) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    var snapshot = StateRecorderImpl.init(std.testing.allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .{});
    const before = heap.num_allocations.load(.monotonic);
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    const num_allocations = heap.num_allocations.load(.monotonic) - before;
    var reference = StateRecorderImpl.init(std.testing.allocator);
    defer reference.deinit();
    try scene.system.saveState(reference.recorder(), .{});

    for (0..num_allocations) |i| {
        snapshot.rewind();
        try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
        heap.fail_from.store(heap.num_allocations.load(.monotonic) + @as(u32, @intCast(i)), .monotonic);
        try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem()));
        heap.fail_from.store(std.math.maxInt(u32), .monotonic);
        try expect(temp.isEmpty());
        try expectContactRecovery(scene, &snapshot, &reference, &temp, jobs.jobSystem());
    }
}

test "PhysicsSystem contacts: out of heap memory inside a job (ProcessBodyPair), single threaded" {
    try contactOutOfHeapMemory(.single_threaded, .{});
}

test "PhysicsSystem contacts: out of heap memory inside a job (ProcessBodyPair), multi threaded" {
    try contactOutOfHeapMemory(.{ .thread_pool = 3 }, .{});
}

test "PhysicsSystem contacts: out of heap memory inside a job (ProcessBodyPair without manifold reduction), single threaded" {
    try contactOutOfHeapMemory(.single_threaded, .{ .use_manifold_reduction = false });
}

test "PhysicsSystem contacts: out of heap memory inside a job (ProcessBodyPair without manifold reduction), multi threaded" {
    try contactOutOfHeapMemory(.{ .thread_pool = 3 }, .{ .use_manifold_reduction = false });
}

test "PhysicsSystem contacts: out of memory creating the scene" {
    const Ctx = struct {
        fn build(allocator: Allocator) !void {
            const scene = try allocator.create(ContactScene);
            defer allocator.destroy(scene);
            try scene.init(allocator, .{});
            scene.deinit();
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, Ctx.build, .{});
}

/// Counts the bodies that are deactivated while the LargeIslandSplitter holds split islands: a large island that falls
/// asleep is put to sleep by CheckSleepAndUpdateBounds from the final batch of the split island (in
/// SolvePositionConstraints, between JobFinalizeIslands and the reset of the splitter)
const SplitIslandSleepListener = struct {
    system: *const PhysicsSystem,
    num_deactivated: std.atomic.Value(u32) = .init(0),
    num_deactivated_from_split_island: std.atomic.Value(u32) = .init(0),

    pub fn onBodyActivated(self: *SplitIslandSleepListener, body_id: BodyID, body_user_data: u64) void {
        _ = .{ self, body_id, body_user_data };
    }

    pub fn onBodyDeactivated(self: *SplitIslandSleepListener, body_id: BodyID, body_user_data: u64) void {
        _ = .{ body_id, body_user_data };
        _ = self.num_deactivated.fetchAdd(1, .monotonic);
        if (self.system.large_island_splitter.num_split_islands > 0)
            _ = self.num_deactivated_from_split_island.fetchAdd(1, .monotonic);
    }
};

/// A pyramid of boxes with a base of 16 (136 bodies, about 256 contacts in one island)
const PyramidScene = struct {
    const base = 16;
    const num_boxes = base * (base + 1) / 2;

    broad_phase_layer_interface: LayersFile.BPLayerInterfaceImpl = .init(),
    object_vs_broad_phase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_layer_pair_filter: LayersFile.ObjectLayerPairFilterImpl = .{},
    system: PhysicsSystem = .empty,
    listener: SplitIslandSleepListener = undefined,

    fn init(self: *PyramidScene, allocator: Allocator, use_large_island_splitter: bool) !void {
        self.* = .{};
        try self.system.init(allocator, io, 1024, 0, 4096, 2048, &self.broad_phase_layer_interface.base, &self.object_vs_broad_phase_layer_filter.base, &self.object_layer_pair_filter.base, .{});
        errdefer self.system.deinit();
        var settings = self.system.getPhysicsSettings().*;
        settings.time_before_sleep = 0.25;
        settings.use_large_island_splitter = use_large_island_splitter;
        self.system.setPhysicsSettings(&settings);
        self.listener = .{ .system = &self.system };
        self.system.setBodyActivationListener(.init(&self.listener));
        const bi = self.system.getBodyInterfaceMut();

        const floor_shape = try BoxShape.create(allocator, Vec3.init(30, 1, 30), .{ .convex_radius = 0.0 });
        var floor_settings = BodyCreationSettings.initWithShape(floor_shape.asShape(), RVec3.init(0, -1, 0), Quat.identity(), .static, Layers.non_moving);
        defer floor_settings.deinit();
        _ = try bi.createAndAddBody(&floor_settings, .dont_activate);

        var box_settings = BodyCreationSettings.initWithSettings((try BoxShapeSettings.create(allocator, Vec3.replicate(0.5), .{})).asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
        defer box_settings.deinit();
        for (0..base) |y| {
            for (0..base - y) |x| {
                box_settings.position = RVec3.init(f(@intCast(x)) * 1.05 + f(@intCast(y)) * 0.525 - 8.0, 0.5 + f(@intCast(y)), 0);
                _ = try bi.createAndAddBody(&box_settings, .activate);
            }
        }
    }

    fn deinit(self: *PyramidScene) void {
        self.system.deinit();
    }
};

/// Runs the pyramid until it is asleep, returns the state after every update
fn runPyramid(allocator: Allocator, kind: JobSystemKind, use_large_island_splitter: bool, states: *std.ArrayList(u8), offsets: *std.ArrayList(usize)) !SplitIslandSleepListener {
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(allocator, 8 * 1024 * 1024);
    defer temp.deinit();
    const scene = try allocator.create(PyramidScene);
    defer allocator.destroy(scene);
    try scene.init(allocator, use_large_island_splitter);
    defer scene.deinit();

    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    for (0..60) |_| {
        const errors = try scene.system.update(delta_time, 1, temp.tempAllocator(), jobs.jobSystem());
        try expect(errors.isNone());
        try expect(temp.isEmpty());
        recorder.clear();
        try scene.system.saveState(recorder.recorder(), .{});
        try offsets.append(allocator, states.items.len);
        try states.appendSlice(allocator, recorder.getData());
    }
    try expectEqual(@as(u32, 0), scene.system.getNumActiveBodies(.rigid_body)); // The pyramid is asleep
    return scene.listener;
}

test "PhysicsSystem large island: the LargeIslandSplitter path" {
    // The pyramid is one island of about 256 contacts (more than LargeIslandSplitter::cLargeIslandTreshold): its
    // velocities and positions are solved in batches of splits by all SolveVelocityConstraints /
    // SolvePositionConstraints jobs, and the whole island goes to sleep from the final batch of the split island
    const allocator = std.testing.allocator;
    var ref_states: std.ArrayList(u8) = .empty;
    defer ref_states.deinit(allocator);
    var ref_offsets: std.ArrayList(usize) = .empty;
    defer ref_offsets.deinit(allocator);
    const ref = try runPyramid(allocator, .single_threaded, true, &ref_states, &ref_offsets);
    try expectEqual(@as(u32, PyramidScene.num_boxes), ref.num_deactivated.load(.monotonic));
    try expectEqual(@as(u32, PyramidScene.num_boxes), ref.num_deactivated_from_split_island.load(.monotonic));

    // Bit identical with thread pools (the batches of the splits run on all threads)
    for ([_]JobSystemKind{ .{ .thread_pool = 0 }, .{ .thread_pool = 1 }, .{ .thread_pool = 3 }, .{ .thread_pool = 7 }, .{ .thread_pool = 7 } }) |kind| {
        var states: std.ArrayList(u8) = .empty;
        defer states.deinit(allocator);
        var offsets: std.ArrayList(usize) = .empty;
        defer offsets.deinit(allocator);
        const listener = try runPyramid(allocator, kind, true, &states, &offsets);
        try expectEqual(@as(u32, PyramidScene.num_boxes), listener.num_deactivated_from_split_island.load(.monotonic));
        try std.testing.expectEqualSlices(usize, ref_offsets.items, offsets.items);
        try std.testing.expectEqualSlices(u8, ref_states.items, states.items);
    }

    // Without the splitter the island is solved in one piece by one job: other results, nothing from a split island
    var states: std.ArrayList(u8) = .empty;
    defer states.deinit(allocator);
    var offsets: std.ArrayList(usize) = .empty;
    defer offsets.deinit(allocator);
    const without = try runPyramid(allocator, .{ .thread_pool = 3 }, false, &states, &offsets);
    try expectEqual(@as(u32, PyramidScene.num_boxes), without.num_deactivated.load(.monotonic));
    try expectEqual(@as(u32, 0), without.num_deactivated_from_split_island.load(.monotonic));
    try expect(!std.mem.eql(u8, ref_states.items, states.items));
}

// -------------------------------------------------------------------------------------------------------------------
// CCD scenes (Wave B R4: JobFindCCDContacts, sSolveCCDContact, JobResolveCCDContacts)

/// The user data bits of the bullets that the CCD contact listener acts on
const CCDFlags = struct {
    /// OnContactValidate: RejectAllContactsForThisBodyPair (the bullet goes through the wall)
    const reject: u64 = 1 << 32;
    /// OnContactAdded / OnContactPersisted: a sensor contact (no response, the bullet goes through)
    const sensor: u64 = 2 << 32;
};

/// Counts the contact callbacks and acts on CCDFlags (called from the worker threads: atomics)
const CCDContactListener = struct {
    num_validated: std.atomic.Value(u32) = .init(0),
    num_added: std.atomic.Value(u32) = .init(0),
    num_persisted: std.atomic.Value(u32) = .init(0),
    num_removed: std.atomic.Value(u32) = .init(0),

    pub fn onContactValidate(self: *CCDContactListener, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const zolt.CollideShapeResult) zolt.ValidateResult {
        _ = .{ base_offset, collision_result };
        _ = self.num_validated.fetchAdd(1, .monotonic);
        return if (((body1.getUserData() | body2.getUserData()) & CCDFlags.reject) != 0) .reject_all_contacts_for_this_body_pair else .accept_all_contacts_for_this_body_pair;
    }

    fn modify(body1: *const Body, body2: *const Body, settings: *zolt.ContactSettings) void {
        if (((body1.getUserData() | body2.getUserData()) & CCDFlags.sensor) != 0)
            settings.is_sensor = true;
    }

    pub fn onContactAdded(self: *CCDContactListener, body1: *const Body, body2: *const Body, manifold: *const zolt.ContactManifold, settings: *zolt.ContactSettings) void {
        _ = manifold;
        _ = self.num_added.fetchAdd(1, .monotonic);
        modify(body1, body2, settings);
    }

    pub fn onContactPersisted(self: *CCDContactListener, body1: *const Body, body2: *const Body, manifold: *const zolt.ContactManifold, settings: *zolt.ContactSettings) void {
        _ = manifold;
        _ = self.num_persisted.fetchAdd(1, .monotonic);
        modify(body1, body2, settings);
    }

    pub fn onContactRemoved(self: *CCDContactListener, sub_shape_pair: *const zolt.SubShapeIDPair) void {
        _ = sub_shape_pair;
        _ = self.num_removed.fetchAdd(1, .monotonic);
    }
};

/// Wraps a job system: counts the FindCCDContacts jobs (PostIntegrateVelocity creates them from inside its job) and
/// fails the FindCCDContacts jobs with index `fail_from` (counting from 0 over the whole update) and later
const CCDJobSystem = struct {
    inner: JobSystem,
    fail_from: u32 = std.math.maxInt(u32),
    num_find_ccd_contacts: std.atomic.Value(u32) = .init(0),
    num_failed: std.atomic.Value(u32) = .init(0),

    fn jobSystem(self: *CCDJobSystem) JobSystem {
        return .init(self);
    }

    pub fn getMaxConcurrency(self: *const CCDJobSystem) i32 {
        return self.inner.getMaxConcurrency();
    }

    pub fn createJob(self: *CCDJobSystem, job_name: []const u8, color: zolt.Color, job_function: JobSystem.JobFunction, opts: struct { num_dependencies: u32 = 0 }) error{OutOfMemory}!JobSystem.JobHandle {
        if (std.mem.eql(u8, job_name, "FindCCDContacts")) {
            if (self.num_find_ccd_contacts.fetchAdd(1, .monotonic) >= self.fail_from) {
                _ = self.num_failed.fetchAdd(1, .monotonic);
                return error.OutOfMemory;
            }
        }
        return self.inner.createJob(job_name, color, job_function, .{ .num_dependencies = opts.num_dependencies });
    }

    pub fn createBarrier(self: *CCDJobSystem) ?*JobSystem.Barrier {
        return self.inner.createBarrier();
    }

    pub fn destroyBarrier(self: *CCDJobSystem, barrier: *JobSystem.Barrier) void {
        self.inner.destroyBarrier(barrier);
    }

    pub fn waitForJobs(self: *CCDJobSystem, barrier: *JobSystem.Barrier) void {
        self.inner.waitForJobs(barrier);
    }

    // The jobs belong to `inner` and queue themselves there: not reached
    pub fn queueJob(self: *CCDJobSystem, job: *JobSystem.Job) void {
        self.inner.vtable.queueJob(self.inner.ptr, job);
    }

    pub fn queueJobs(self: *CCDJobSystem, jobs: []const *JobSystem.Job) void {
        self.inner.vtable.queueJobs(self.inner.ptr, jobs);
    }

    pub fn freeJob(self: *CCDJobSystem, job: *JobSystem.Job) void {
        self.inner.vtable.freeJob(self.inner.ptr, job);
    }
};

/// A scene with continuous collision detection: 32 linear cast bullets in a container of thin walls, fired at the walls
/// (most bullets, one rejected by OnContactValidate, one whose contact OnContactAdded turns into a sensor contact: both
/// go through the wall), two bullets hitting each other head on, a bullet that hits a sleeping box (it wakes up) and
/// one that hits a kinematic box. Many CCD bodies per collision step: several FindCCDContacts jobs with threads.
const CCDScene = struct {
    const num_ccd_bullets = 32;
    /// The linear cast bodies: the bullets, the two bullets hitting each other head on and the bullets that hit the
    /// sleeping box and the kinematic box
    const num_ccd_bodies = num_ccd_bullets + 4;
    const half_extent = 12.0;

    broad_phase_layer_interface: LayersFile.BPLayerInterfaceImpl = .init(),
    object_vs_broad_phase_layer_filter: LayersFile.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_layer_pair_filter: LayersFile.ObjectLayerPairFilterImpl = .{},
    system: PhysicsSystem = .empty,
    activation_listener: ActivationCounter = .{},
    contact_listener: CCDContactListener = .{},
    bullets: [num_ccd_bullets]BodyID = @splat(.invalid),
    rejected_bullet: BodyID = .invalid,
    sensor_bullet: BodyID = .invalid,
    sleeping_box: BodyID = .invalid,

    /// Builds the scene in place (the physics system must not move)
    fn init(self: *CCDScene, allocator: Allocator, opts: PhysicsSystem.InitOptions) !void {
        try self.initSystem(allocator, opts);
        errdefer self.deinit();
        try self.populate(allocator);
    }

    /// Initializes the physics system (on error a custom broad phase still belongs to the caller)
    fn initSystem(self: *CCDScene, allocator: Allocator, opts: PhysicsSystem.InitOptions) !void {
        self.* = .{};
        try self.system.init(allocator, io, 1024, 0, 4096, 2048, &self.broad_phase_layer_interface.base, &self.object_vs_broad_phase_layer_filter.base, &self.object_layer_pair_filter.base, opts);
    }

    fn populate(self: *CCDScene, allocator: Allocator) !void {
        self.system.setBodyActivationListener(.init(&self.activation_listener));
        self.system.setContactListener(.init(&self.contact_listener));
        const bi = self.system.getBodyInterfaceMut();

        // Floor and four thin walls
        const floor_shape = try BoxShape.create(allocator, Vec3.init(40, 1, 40), .{ .convex_radius = 0.0 });
        var static_settings = BodyCreationSettings.initWithShape(floor_shape.asShape(), RVec3.init(0, -1, 0), Quat.identity(), .static, Layers.non_moving);
        defer static_settings.deinit();
        try expect(!(try bi.createAndAddBody(&static_settings, .dont_activate)).isInvalid());
        static_settings.setShape((try BoxShape.create(allocator, Vec3.init(0.05, 4, half_extent), .{ .convex_radius = 0.0 })).asShape());
        for ([_]f32{ -half_extent, half_extent }) |x| {
            static_settings.position = RVec3.init(x, 4, 0);
            _ = try bi.createAndAddBody(&static_settings, .dont_activate);
        }
        static_settings.setShape((try BoxShape.create(allocator, Vec3.init(half_extent, 4, 0.05), .{ .convex_radius = 0.0 })).asShape());
        for ([_]f32{ -half_extent, half_extent }) |z| {
            static_settings.position = RVec3.init(0, 4, z);
            _ = try bi.createAndAddBody(&static_settings, .dont_activate);
        }

        // The bullets, fired at the walls in all directions
        var sphere_settings = BodyCreationSettings.initWithSettings((try SphereShapeSettings.create(allocator, 0.2, .{})).asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
        defer sphere_settings.deinit();
        var box_settings = BodyCreationSettings.initWithSettings((try BoxShapeSettings.create(allocator, Vec3.init(0.2, 0.15, 0.25), .{ .convex_radius = 0.05 })).asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.moving);
        defer box_settings.deinit();
        for (&self.bullets, 0..) |*bullet, i| {
            const settings = if (i % 2 == 0) &sphere_settings else &box_settings;
            const angle = 2.0 * std.math.pi * f(@intCast(i)) / num_ccd_bullets;
            settings.position = RVec3.init(3.5 * @cos(angle), 1.0 + 0.1 * f(@intCast(i % 7)), 3.5 * @sin(angle));
            settings.rotation = Quat.rotation(Vec3.init(0, 1, 0), 0.3 * f(@intCast(i)));
            settings.linear_velocity = Vec3.init(@cos(angle), 0.05, @sin(angle)).mulScalar(100.0 + 5.0 * f(@intCast(i % 5)));
            settings.angular_velocity = Vec3.init(0, 2, 0);
            settings.restitution = if (i % 3 == 0) 0.5 else 0.0;
            settings.friction = 0.2 + 0.1 * f(@intCast(i % 4));
            settings.motion_quality = .linear_cast;
            settings.user_data = if (i == 5) CCDFlags.reject else if (i == 9) CCDFlags.sensor else 0;
            bullet.* = try bi.createAndAddBody(settings, .activate);
        }
        self.rejected_bullet = self.bullets[5];
        self.sensor_bullet = self.bullets[9];
        sphere_settings.user_data = 0;
        box_settings.user_data = 0;

        // Two bullets hitting each other head on (elastic)
        sphere_settings.restitution = 1.0;
        sphere_settings.rotation = Quat.identity();
        sphere_settings.angular_velocity = Vec3.zero();
        sphere_settings.position = RVec3.init(-1, 6, 6);
        sphere_settings.linear_velocity = Vec3.init(120, 0, 0);
        _ = try bi.createAndAddBody(&sphere_settings, .activate);
        sphere_settings.position = RVec3.init(1, 6, 6);
        sphere_settings.linear_velocity = Vec3.init(-120, 0, 0);
        _ = try bi.createAndAddBody(&sphere_settings, .activate);

        // A sleeping box and a bullet that hits it
        box_settings.motion_quality = .discrete;
        box_settings.rotation = Quat.identity();
        box_settings.angular_velocity = Vec3.zero();
        box_settings.linear_velocity = Vec3.zero();
        box_settings.restitution = 0.0;
        box_settings.position = RVec3.init(4, 0.15, -6);
        self.sleeping_box = try bi.createAndAddBody(&box_settings, .dont_activate);
        sphere_settings.restitution = 0.0;
        sphere_settings.position = RVec3.init(-4, 0.2, -6);
        sphere_settings.linear_velocity = Vec3.init(150, 0, 0);
        _ = try bi.createAndAddBody(&sphere_settings, .activate);

        // A kinematic box and a bullet that hits it
        box_settings.motion_type = .kinematic;
        box_settings.position = RVec3.init(4, 3, -8);
        box_settings.linear_velocity = Vec3.init(-2, 0, 0);
        _ = try bi.createAndAddBody(&box_settings, .activate);
        sphere_settings.position = RVec3.init(-4, 3, -8);
        sphere_settings.linear_velocity = Vec3.init(150, 0, 0);
        _ = try bi.createAndAddBody(&sphere_settings, .activate);
    }

    fn deinit(self: *CCDScene) void {
        self.system.deinit();
    }
};

const ccd_num_updates = 30;

/// What a run of the CCD scene did
const CCDRunStats = struct {
    /// Largest number of FindCCDContacts jobs in one update
    max_find_ccd_contacts_jobs: u32,
    /// Validated, added, persisted and removed contact callbacks
    counts: [4]u32,
};

/// Runs the CCD scene and records the saved state after every update
fn runCCDScene(allocator: Allocator, kind: JobSystemKind, states: *std.ArrayList(u8), offsets: *std.ArrayList(usize)) !CCDRunStats {
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit();
    var counting: CCDJobSystem = .{ .inner = jobs.jobSystem() };

    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(CCDScene);
    defer allocator.destroy(scene);
    try scene.init(allocator, .{});
    defer scene.deinit();

    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();

    const bi = scene.system.getBodyInterface();
    try expectEqual(@as(u32, CCDScene.num_ccd_bodies), scene.system.body_manager.getNumActiveCCDBodies());
    var max_jobs: u32 = 0;
    for (0..ccd_num_updates) |_| {
        const before = counting.num_find_ccd_contacts.load(.monotonic);
        const errors = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), counting.jobSystem());
        max_jobs = @max(max_jobs, counting.num_find_ccd_contacts.load(.monotonic) - before);
        try expect(errors.isNone());
        try expect(temp.isEmpty());

        recorder.clear();
        try scene.system.saveState(recorder.recorder(), .{});
        try expect(!recorder.isFailed());
        try offsets.append(allocator, states.items.len);
        try states.appendSlice(allocator, recorder.getData());
    }

    // Sanity: the bullets stayed in the container (the walls are thinner than a step of a bullet), except the one whose
    // hit was rejected and the one whose hit became a sensor contact; the sleeping box was woken up
    for (scene.bullets) |bullet| {
        const p = bi.getPosition(bullet);
        const inside = @abs(p.getX()) < CCDScene.half_extent and @abs(p.getZ()) < CCDScene.half_extent;
        try expectEqual(!bullet.eql(scene.rejected_bullet) and !bullet.eql(scene.sensor_bullet), inside);
    }
    try expect(bi.getPosition(scene.sleeping_box).getX() > 4.5);
    const listener = &scene.contact_listener;
    const counts = [4]u32{ listener.num_validated.load(.monotonic), listener.num_added.load(.monotonic), listener.num_persisted.load(.monotonic), listener.num_removed.load(.monotonic) };
    try expect(counts[0] > 0 and counts[1] > 0 and counts[2] > 0);
    return .{ .max_find_ccd_contacts_jobs = max_jobs, .counts = counts };
}

test "PhysicsSystem CCD: determinism single vs multi threaded with several FindCCDContacts jobs" {
    // JobSystemSingleThreaded against thread pools: the saved state after every update is bit identical, the CCD
    // bodies are spread over several FindCCDContacts jobs with threads
    const allocator = std.testing.allocator;
    var ref_states: std.ArrayList(u8) = .empty;
    defer ref_states.deinit(allocator);
    var ref_offsets: std.ArrayList(usize) = .empty;
    defer ref_offsets.deinit(allocator);
    const ref = try runCCDScene(allocator, .single_threaded, &ref_states, &ref_offsets);
    try expectEqual(@as(u32, collision_steps), ref.max_find_ccd_contacts_jobs); // One job per collision step

    const kinds = [_]JobSystemKind{ .{ .thread_pool = 0 }, .{ .thread_pool = 1 }, .{ .thread_pool = 3 }, .{ .thread_pool = 7 }, .{ .thread_pool = 3 }, .{ .thread_pool = 7 }, .{ .thread_pool = 7 } };
    for (kinds) |kind| {
        var states: std.ArrayList(u8) = .empty;
        defer states.deinit(allocator);
        var offsets: std.ArrayList(usize) = .empty;
        defer offsets.deinit(allocator);
        const stats = try runCCDScene(allocator, kind, &states, &offsets);

        try std.testing.expectEqualSlices(usize, ref_offsets.items, offsets.items);
        for (ref_offsets.items, 0..) |offset, i| {
            const end = if (i + 1 < ref_offsets.items.len) ref_offsets.items[i + 1] else ref_states.items.len;
            if (!std.mem.eql(u8, ref_states.items[offset..end], states.items[offset..end])) {
                std.debug.print("state differs after update {d} with {any}\n", .{ i, kind });
                return error.TestUnexpectedResult;
            }
        }
        try expectEqual(ref.counts, stats.counts);

        // min(ceil(CCD bodies / 4), concurrency) jobs per step (all CCD bodies cast in the first update): the 36 CCD
        // bodies need 9 jobs, capped at 8 by the concurrency of 7 workers
        const concurrency: u32 = @intCast(kind.thread_pool + 1);
        const jobs_per_step: u32 = @min(concurrency, (CCDScene.num_ccd_bodies + 3) / 4);
        try expectEqual(collision_steps * jobs_per_step, stats.max_find_ccd_contacts_jobs);
    }
}

/// Restore `snapshot`, run one update with enough memory and compare with `reference`
fn expectCCDRecovery(scene: *CCDScene, snapshot: *StateRecorderImpl, reference: *StateRecorderImpl, temp: *TempAllocatorImpl, job_system: JobSystem) !void {
    snapshot.rewind();
    try expect(try scene.system.restoreState(snapshot.recorder(), .{}));
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), job_system);
    try expect(temp.isEmpty());
    try expectCCDState(scene, reference);
}

fn expectCCDState(scene: *CCDScene, expected: *StateRecorderImpl) !void {
    var state = StateRecorderImpl.init(std.testing.allocator);
    defer state.deinit();
    try scene.system.saveState(state.recorder(), .{});
    try expect(state.isEqual(expected));
}

/// The CCD scene before the bullets reach the walls (`num_updates` updates), with a snapshot of the state, and the
/// reference state after one more update in which the bullets hit the walls
const CCDFixture = struct {
    jobs: AnyJobSystem = .{},
    temp: TempAllocatorImpl,
    scene: *CCDScene,
    snapshot: StateRecorderImpl,
    reference: StateRecorderImpl,

    /// The update in which the bullets that fly at 45 degrees hit the walls (they move 0.85 to 1 m per collision step
    /// from 3.5 m off the center, the walls are at 12 m; the bullets along the axes hit them an update earlier)
    const hit_update = 5;

    /// `opts.broad_phase` may be a custom broad phase: the fixture owns it from the call on
    fn init(self: *CCDFixture, kind: JobSystemKind, scene_allocator: Allocator, opts: PhysicsSystem.InitOptions) !void {
        const allocator = std.testing.allocator;
        self.* = .{ .temp = undefined, .scene = undefined, .snapshot = .init(allocator), .reference = .init(allocator) };
        errdefer self.reference.deinit();
        errdefer self.snapshot.deinit();
        self.scene = allocator.create(CCDScene) catch |err| {
            if (opts.broad_phase == .custom) opts.broad_phase.custom.base.destroy();
            return err;
        };
        errdefer allocator.destroy(self.scene);
        self.scene.initSystem(scene_allocator, opts) catch |err| {
            if (opts.broad_phase == .custom) opts.broad_phase.custom.base.destroy();
            return err;
        };
        errdefer self.scene.deinit();
        try self.scene.populate(scene_allocator);
        self.temp = try .init(allocator, 4 * 1024 * 1024);
        errdefer self.temp.deinit();
        try self.jobs.init(allocator, kind);
        errdefer self.jobs.deinit();
        for (0..hit_update) |_| _ = try self.scene.system.update(delta_time, collision_steps, self.temp.tempAllocator(), self.jobs.jobSystem());
        try self.scene.system.saveState(self.snapshot.recorder(), .{});
        _ = try self.scene.system.update(delta_time, collision_steps, self.temp.tempAllocator(), self.jobs.jobSystem());
        try self.scene.system.saveState(self.reference.recorder(), .{});
    }

    fn deinit(self: *CCDFixture) void {
        self.reference.deinit();
        self.snapshot.deinit();
        self.scene.deinit();
        std.testing.allocator.destroy(self.scene);
        self.jobs.deinit();
        self.temp.deinit();
    }

    fn restore(self: *CCDFixture) !void {
        self.snapshot.rewind();
        try expect(try self.scene.system.restoreState(self.snapshot.recorder(), .{}));
    }
};

fn ccdOutOfTempMemory(kind: JobSystemKind) !void {
    var fx: CCDFixture = undefined;
    try fx.init(kind, std.testing.allocator, .{});
    defer fx.deinit();
    const scene = fx.scene;

    // The update of the hits again, counting the temp allocations (per step: the CCD bodies, the mapping from active
    // bodies to CCD bodies, the sorted CCD bodies in ResolveCCDContacts)
    try fx.restore();
    var counting: FailingTempAllocator = .init(fx.temp.tempAllocator(), .{});
    _ = try scene.system.update(delta_time, collision_steps, counting.tempAllocator(), fx.jobs.jobSystem());
    try expectCCDState(scene, &fx.reference);
    const num_allocations = counting.num_allocations;
    try expect(num_allocations >= num_build_temp_allocations + collision_steps * 3);

    // Fail every allocation in turn: the update returns error.OutOfMemory, the temp allocator is empty (the CCD arrays
    // are freed by ResolveCCDContacts also when it skipped its work) and restoring the snapshot recovers the reference
    for (0..num_allocations) |fail_index| {
        try fx.restore();
        var failing: FailingTempAllocator = .init(fx.temp.tempAllocator(), .{ .fail_index = @intCast(fail_index) });
        try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, failing.tempAllocator(), fx.jobs.jobSystem()));
        try expect(failing.has_induced_failure);
        try expect(fx.temp.isEmpty());
        if (fail_index < num_build_temp_allocations)
            try expectCCDState(scene, &fx.snapshot); // No job ran
        try expectCCDRecovery(scene, &fx.snapshot, &fx.reference, &fx.temp, fx.jobs.jobSystem());
    }
}

test "PhysicsSystem CCD: out of temp memory at every allocation, single threaded" {
    try ccdOutOfTempMemory(.single_threaded);
}

test "PhysicsSystem CCD: out of temp memory at every allocation, multi threaded" {
    try ccdOutOfTempMemory(.{ .thread_pool = 7 });
}

fn ccdFailingCreateJob(kind: JobSystemKind) !void {
    var fx: CCDFixture = undefined;
    try fx.init(kind, std.testing.allocator, .{});
    defer fx.deinit();
    const scene = fx.scene;

    // Count the jobs of the update that the update builds (they are created before any job runs, the start gate) and
    // the FindCCDContacts jobs that PostIntegrateVelocity creates (it runs after every built job was created)
    try fx.restore();
    var ccd_counting: CCDJobSystem = .{ .inner = fx.jobs.jobSystem() };
    var counting: FailingJobSystem = .init(ccd_counting.jobSystem(), .{});
    _ = try scene.system.update(delta_time, collision_steps, fx.temp.tempAllocator(), counting.jobSystem());
    try expectCCDState(scene, &fx.reference);
    const num_ccd_jobs = ccd_counting.num_find_ccd_contacts.load(.monotonic);
    try expect(num_ccd_jobs >= collision_steps);

    // Every createJob of the job graph fails in turn: the update fails without side effects
    var fail_index: u32 = 0;
    var num_build_failures: u32 = 0;
    while (true) : (fail_index += 1) {
        try fx.restore();
        var failing: FailingJobSystem = .init(fx.jobs.jobSystem(), .{ .fail_index = fail_index });
        if (scene.system.update(delta_time, collision_steps, fx.temp.tempAllocator(), failing.jobSystem())) |errors| {
            // Past the jobs of the graph (or a job spawned from inside a job failed): no error, the reference results
            try expect(errors.isNone());
            try expectCCDState(scene, &fx.reference);
            try expect(fx.temp.isEmpty());
            if (!failing.has_induced_failure.load(.monotonic)) break;
            continue;
        } else |err| {
            try expectEqual(error.OutOfMemory, err);
            num_build_failures += 1;
            try expect(fx.temp.isEmpty());
            try expectCCDState(scene, &fx.snapshot); // Nothing changed
            try expectCCDRecovery(scene, &fx.snapshot, &fx.reference, &fx.temp, fx.jobs.jobSystem());
        }
    }
    try expect(num_build_failures >= collision_steps * (num_job_kinds_per_step - 1) + 1); // No step listener job

    // A FindCCDContacts job that cannot be created is not an error and changes no result (D5): fail the FindCCDContacts
    // job with every index and every job from that index on (none created in a step: PostIntegrateVelocity does the
    // work itself)
    for (0..num_ccd_jobs + 1) |ccd_index| {
        try fx.restore();
        var failing: CCDJobSystem = .{ .inner = fx.jobs.jobSystem(), .fail_from = @intCast(ccd_index) };
        const errors = try scene.system.update(delta_time, collision_steps, fx.temp.tempAllocator(), failing.jobSystem());
        try expect(errors.isNone());
        try expect(fx.temp.isEmpty());
        try expectCCDState(scene, &fx.reference);
        try expectEqual(num_ccd_jobs - @min(num_ccd_jobs, @as(u32, @intCast(ccd_index))) > 0, failing.num_failed.load(.monotonic) > 0);
    }
}

test "PhysicsSystem CCD: createJob fails at every job and at every FindCCDContacts job, single threaded" {
    try ccdFailingCreateJob(.single_threaded);
}

test "PhysicsSystem CCD: createJob fails at every job and at every FindCCDContacts job, multi threaded" {
    try ccdFailingCreateJob(.{ .thread_pool = 7 });
}

fn ccdOutOfHeapMemory(kind: JobSystemKind) !void {
    // The heap allocations inside the jobs of the update of the hits (the QuadTree in UpdateBroadPhasePrepare) fail in
    // turn, every later one too: the job latches the error (D5), the update returns error.OutOfMemory, the temp allocator
    // is empty and the system recovers through restoreState
    var heap: FailingHeapAllocator = .{ .inner = std.testing.allocator };
    var fx: CCDFixture = undefined;
    try fx.init(kind, heap.allocator(), .{});
    defer fx.deinit();
    const scene = fx.scene;

    try fx.restore();
    const before = heap.num_allocations.load(.monotonic);
    _ = try scene.system.update(delta_time, collision_steps, fx.temp.tempAllocator(), fx.jobs.jobSystem());
    const num_allocations = heap.num_allocations.load(.monotonic) - before;
    try expect(num_allocations > 0);
    try expectCCDState(scene, &fx.reference);

    for (0..num_allocations) |i| {
        try fx.restore();
        heap.fail_from.store(heap.num_allocations.load(.monotonic) + @as(u32, @intCast(i)), .monotonic);
        try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, fx.temp.tempAllocator(), fx.jobs.jobSystem()));
        heap.fail_from.store(std.math.maxInt(u32), .monotonic);
        try expect(fx.temp.isEmpty());
        try expectCCDRecovery(scene, &fx.snapshot, &fx.reference, &fx.temp, fx.jobs.jobSystem());
    }
}

test "PhysicsSystem CCD: out of heap memory inside a job, single threaded" {
    try ccdOutOfHeapMemory(.single_threaded);
}

test "PhysicsSystem CCD: out of heap memory inside a job, multi threaded" {
    try ccdOutOfHeapMemory(.{ .thread_pool = 3 });
}

fn ccdOutOfHeapMemoryInCast(kind: JobSystemKind) !void {
    // The broad phase cast of the FindCCDContacts jobs fails (BroadPhaseQuadTree's node stack growing on the heap) while
    // the bullets hit the walls: the jobs latch the error, ResolveCCDContacts skips its work (and frees), the update
    // returns error.OutOfMemory and the system recovers
    const broad_phase = try AllocatingBroadPhase.create(std.testing.allocator);
    var fx: CCDFixture = undefined;
    try fx.init(kind, std.testing.allocator, .{ .broad_phase = .{ .custom = &broad_phase.base } });
    defer fx.deinit();
    const scene = fx.scene;

    try fx.restore();
    broad_phase.fail_cast.store(true, .monotonic);
    try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, fx.temp.tempAllocator(), fx.jobs.jobSystem()));
    broad_phase.fail_cast.store(false, .monotonic);
    try expect(fx.temp.isEmpty());
    try expectCCDRecovery(scene, &fx.snapshot, &fx.reference, &fx.temp, fx.jobs.jobSystem());
}

test "PhysicsSystem CCD: out of heap memory in the broad phase cast of FindCCDContacts, single threaded" {
    try ccdOutOfHeapMemoryInCast(.single_threaded);
}

test "PhysicsSystem CCD: out of heap memory in the broad phase cast of FindCCDContacts, multi threaded" {
    try ccdOutOfHeapMemoryInCast(.{ .thread_pool = 7 });
}

test "PhysicsSystem CCD: save and restore with CCD hits" {
    // Snapshot before the hits, simulate, restore, simulate again: identical (the CCD manifolds in the contact cache),
    // also through a validating restore
    var fx: CCDFixture = undefined;
    try fx.init(.{ .thread_pool = 3 }, std.testing.allocator, .{});
    defer fx.deinit();
    const scene = fx.scene;
    const allocator = std.testing.allocator;

    var after1 = StateRecorderImpl.init(allocator);
    defer after1.deinit();
    for (0..10) |_| _ = try scene.system.update(delta_time, collision_steps, fx.temp.tempAllocator(), fx.jobs.jobSystem());
    try scene.system.saveState(after1.recorder(), .{});

    try fx.restore();
    var after2 = StateRecorderImpl.init(allocator);
    defer after2.deinit();
    for (0..11) |_| _ = try scene.system.update(delta_time, collision_steps, fx.temp.tempAllocator(), fx.jobs.jobSystem());
    try scene.system.saveState(after2.recorder(), .{});
    try expect(after2.isEqual(&after1));

    after2.rewind();
    after2.recorder().setValidating(true);
    try expect(try scene.system.restoreState(after2.recorder(), .{}));
    try expectEqual(@as(u32, 0), after2.num_mismatches);
}

test "PhysicsSystem CCD: out of memory creating the scene" {
    const Ctx = struct {
        fn build(allocator: Allocator) !void {
            const scene = try allocator.create(CCDScene);
            defer allocator.destroy(scene);
            try scene.init(allocator, .{});
            scene.deinit();
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, Ctx.build, .{});
}
