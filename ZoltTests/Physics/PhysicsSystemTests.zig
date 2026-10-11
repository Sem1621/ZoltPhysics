//! Zolt addition, no Jolt file: tests of PhysicsSystem.update, the job graph and BodyInterface (SimulationArchitecture
//! D4, D5, D6, D12)
//! Status: partial
//! Missing: the scenes with contacts, constraints and CCD hits (REGION R2..R4 of PhysicsSystem.zig, Wave A/B): a
//! determinism run with FindCollisions jobs spawned from inside jobs (TrySpawnJobFindCollisions only spawns when body
//! pairs are found), createJob failures of those spawns, and the contact / activation-by-contact checks of the Phase 5
//! architecture prototype
//!
//! Adapted from the Phase 5 architecture prototype (SimulationPrototype.zig, deleted in F2) for the real library, on
//! the gravity-only path: the scene is laid out so that no two bodies ever overlap (every body has its own vertical
//! column, the floor is far below), so ProcessBodyPair (REGION R2), the constraint solver (R3) and the narrow phase of
//! the CCD (R4) are never reached, while every other job runs (the bullets go through the CCD jobs without hits).
//! - Determinism: JobSystemSingleThreaded against thread pools with 0, 1, 3 and 7 worker threads, the saved state is
//!   compared after every update.
//! - D5: every temp allocation of an update fails in turn (single and multi threaded), every createJob of an update
//!   fails in turn, a heap allocation inside a job fails (a custom broad phase that allocates in updatePrepare, which
//!   runs in the UpdateBroadPhasePrepare job). The update returns error.OutOfMemory, the temp allocator is empty, a
//!   failure while building the jobs changes nothing, and the system recovers by restoring the state saved before.
//! - Every allocation of creating the scene fails in turn (std.testing.checkAllAllocationFailures).

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
// tree of Jolt's BroadPhaseQuadTree) and delegates everything else to a BroadPhaseBruteForce

const AllocatingBroadPhase = struct {
    pub const overrides = .{ .initBroadPhase, .optimize, .frameSync, .lockModifications, .updatePrepare, .updateFinalize, .unlockModifications, .addBodiesPrepare, .addBodiesFinalize, .addBodiesAbort, .removeBodies, .notifyBodiesAABBChanged, .notifyBodiesLayerChanged, .castRay, .collideAABox, .collideSphere, .collidePoint, .collideOrientedBox, .castAABoxNoLock, .castAABox, .findCollidingPairs, .getBounds };

    const Block = [64]u32;

    base: BroadPhase,
    inner: *BroadPhaseBruteForce,
    num_prepared: u32 = 0,

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

    pub fn optimize(self: *AllocatingBroadPhase) void {
        self.innerBroadPhase().optimize();
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

    pub fn notifyBodiesLayerChanged(self: *AllocatingBroadPhase, bodies: []BodyID) void {
        self.innerBroadPhase().notifyBodiesLayerChanged(bodies);
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

    pub fn castAABoxNoLock(self: *const AllocatingBroadPhase, box: *const AABoxCast, collector: *CastShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        self.innerBroadPhase().castAABoxNoLock(box, collector, broad_phase_layer_filter, object_layer_filter);
    }

    pub fn castAABox(self: *const AllocatingBroadPhase, box: *const AABoxCast, collector: *CastShapeBodyCollector, broad_phase_layer_filter: *const BroadPhaseLayerFilter, object_layer_filter: *const ObjectLayerFilter) void {
        self.innerQuery().castAABox(box, collector, .{ .broad_phase_layer_filter = broad_phase_layer_filter, .object_layer_filter = object_layer_filter });
    }

    pub fn findCollidingPairs(self: *const AllocatingBroadPhase, active_bodies: []BodyID, speculative_contact_distance: f32, object_vs_broad_phase_layer_filter: *const ObjectVsBroadPhaseLayerFilter, object_layer_pair_filter: *const ObjectLayerPairFilter, pair_collector: *BodyPairCollector) void {
        self.innerBroadPhase().findCollidingPairs(active_bodies, speculative_contact_distance, object_vs_broad_phase_layer_filter, object_layer_pair_filter, pair_collector);
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

        // Constraints that are never active (an active constraint needs the solver of REGION R3)
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
    num_allocations: std.atomic.Value(u32) = .init(0),

    fn allocator(self: *FailingHeapAllocator) Allocator {
        return .{ .ptr = self, .vtable = &.{ .alloc = alloc, .resize = Allocator.noResize, .remap = Allocator.noRemap, .free = free } };
    }

    fn alloc(ctx: *anyopaque, len: usize, alignment: std.mem.Alignment, ret_addr: usize) ?[*]u8 {
        const self: *FailingHeapAllocator = @ptrCast(@alignCast(ctx));
        if (self.num_allocations.fetchAdd(1, .monotonic) == self.fail_index.load(.monotonic))
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
    // TestPhysicsActivationDeactivation without the floor (the floor needs contacts, REGION R2): without gravity the
    // activated box stays at rest and falls asleep
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
    // radius. Jolt asserts (a usage error) and its release build casts every movement; Zolt's ReleaseFast build does
    // the same instead of reaching undefined behavior (the assert in jobIntegrateVelocity was unconditional). Nothing
    // is in the way, so the bodies move exactly like their discrete twins. (SimulationParity compares this with Jolt
    // in the scene linear_cast_without_inner_radius.)
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

    var bodies: [4]*Body = undefined;
    for (&bodies, 0..) |*body, i| {
        var settings: BodyCreationSettings = .{};
        defer settings.deinit();
        settings.setShapeSettings(if (i < 2) mesh.asShapeSettings() else mirrored.asShapeSettings());
        settings.position = RVec3.init(@floatFromInt(20 * i), 10, 0);
        settings.linear_velocity = Vec3.init(0, -30, 0);
        settings.motion_quality = if (i % 2 == 0) .linear_cast else .discrete;
        settings.object_layer = Layers.moving;
        settings.override_mass_properties = .mass_and_inertia_provided;
        settings.mass_properties_override.mass = 2.0;
        settings.mass_properties_override.inertia = zolt.Mat44.identity();
        body.* = try c.createBody(&settings, .activate);
    }
    try expect(bodies[0].getShape().getInnerRadius() == 0.0);
    try expect(bodies[2].getShape().getInnerRadius() < 0.0);

    _ = try c.simulate(0.5, {});
    for (0..2) |pair| {
        const cast = bodies[2 * pair].getPosition();
        const discrete = bodies[2 * pair + 1].getPosition();
        try expect(cast.getY() < 0.0); // It moved
        try expectEqual(discrete.getY(), cast.getY());
        try expectEqual(discrete.getZ(), cast.getZ());
    }
}
