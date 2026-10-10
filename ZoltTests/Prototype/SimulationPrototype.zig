//! Phase 5 architecture prototype (Docs/Zolt/SimulationArchitecture.md). Not a port: proves that the decided mechanisms
//! compile and work in Zig 0.16 with the real library (`@import("zolt")`):
//! - Body / BodyManager with Jolt's ID allocation, body mutexes and the read-only Body.fixedToWorld() (D1, D2);
//! - the tagged BodyLockInterface, the RAII locks as init / deinit, PhysicsLock's debug lock order (D3);
//! - PhysicsSystem.update running Jolt's job graph on JobSystemThreadPool and JobSystemSingleThreaded with identical
//!   bits, including dynamic FindCollisions / FindCCDContacts jobs, the start gate and real islands (D4);
//! - the out of memory latch: every temp allocation, every heap allocation inside a job and every createJob of an
//!   update fails in turn (D5);
//! - BodyCreationSettings with the shape union (Rule M), BodyInterface error channels (D6), pattern B listeners on
//!   worker threads (D7), constraints through virtual.make with `createConstraint` (D8), the broad phase hierarchy with
//!   const queries (D9), StateRecorder save / restore / validation and the determinism log format (D11).
//!
//! Run: zig build test -Dtest-filter=SimulationPrototype (also with -Doptimize=ReleaseFast and -Duse_llvm=false)

const std = @import("std");
const zolt = @import("zolt");
const Core = zolt.Core;
const Vec3 = zolt.Vec3;
const Quat = zolt.Quat;
const RVec3 = zolt.RVec3;
const BodyID = zolt.BodyID;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Shape = zolt.Shape;
const AABox = zolt.AABox;
const Color = zolt.Color;
const virtual = zolt.virtual;
const SphereShapeSettings = zolt.SphereShapeSettings;
const BoxShapeSettings = zolt.BoxShapeSettings;
const BoxShape = zolt.BoxShape;
const TempAllocator = zolt.TempAllocator;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const JobSystem = zolt.JobSystem;
const JobHandle = zolt.JobHandle;
const JobSystemThreadPool = zolt.JobSystemThreadPool;
const JobSystemSingleThreaded = zolt.JobSystemSingleThreaded;
const AllHitCollisionCollector = zolt.AllHitCollisionCollector;
const HashCombine = zolt.HashCombine;
const Allocator = std.mem.Allocator;
const expect = std.testing.expect;
const expectEqual = std.testing.expectEqual;

const Layers = @import("../Layers.zig");
const BodyFile = @import("Simulation/Body.zig");
const Body = BodyFile.Body;
const BodyCreationSettings = BodyFile.BodyCreationSettings;
const AllowedDOFs = BodyFile.AllowedDOFs;
const BodyManagerFile = @import("Simulation/BodyManager.zig");
const BodyLockRead = BodyManagerFile.BodyLockRead;
const BodyLockWrite = BodyManagerFile.BodyLockWrite;
const BodyLockMultiWrite = BodyManagerFile.BodyLockMultiWrite;
const BroadPhaseFile = @import("Simulation/BroadPhase.zig");
const ConstraintFile = @import("Simulation/Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const TwoBodyConstraint = ConstraintFile.TwoBodyConstraint;
const ToyDistanceConstraintSettings = ConstraintFile.ToyDistanceConstraintSettings;
const PhysicsSystemFile = @import("Simulation/PhysicsSystem.zig");
const PhysicsSystem = PhysicsSystemFile.PhysicsSystem;
const BodyInterface = PhysicsSystemFile.BodyInterface;
const PhysicsStepListenerContext = PhysicsSystemFile.PhysicsStepListenerContext;
const StateRecorderFile = @import("Simulation/StateRecorder.zig");
const StateRecorderImpl = StateRecorderFile.StateRecorderImpl;
const DeterminismLog = @import("Simulation/DeterminismLog.zig");

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

const ContactCounter = struct {
    num_added: std.atomic.Value(u32) = .init(0),
    /// Order dependent hash of the contacts as they arrive: differs between multithreaded runs (the arrival order is
    /// not deterministic), while the simulation results are identical
    order_hash: std.atomic.Value(u64) = .init(0),

    pub fn onContactAdded(self: *ContactCounter, body1: *const Body, body2: *const Body) void {
        std.debug.assert(!body1.getID().eql(body2.getID()));
        _ = self.num_added.fetchAdd(1, .monotonic);
        const key = (@as(u64, body1.getID().getIndexAndSequenceNumber()) << 32) | body2.getID().getIndexAndSequenceNumber();
        var old = self.order_hash.load(.monotonic);
        while (self.order_hash.cmpxchgWeak(old, (old *% 0x100000001b3) ^ key, .monotonic, .monotonic)) |actual| old = actual;
    }
};

/// Pushes one body every step through the no-lock body interface (all bodies are locked by the update)
const PushListener = struct {
    body_id: BodyID = .invalid,
    num_steps: std.atomic.Value(u32) = .init(0),

    pub fn onStep(self: *PushListener, context: *const PhysicsStepListenerContext) void {
        _ = self.num_steps.fetchAdd(1, .monotonic);
        context.physics_system.getBodyInterfaceNoLock().addLinearVelocity(self.body_id, Vec3.init(0.5 * context.delta_time, 0, 0));
    }
};

// -------------------------------------------------------------------------------------------------------------------
// Test scene

/// The spheres of the scene: columns that fall and topple (134 active bodies), or a dense grid resting on the floor
/// (few active body batches with many pairs each: TrySpawnJobFindCollisions spawns FindCollisions jobs inside jobs)
const Layout = enum { columns, grid };
const num_columns = 6;
const num_per_column = 20;
const grid_size_x = 8;
const grid_size_z = 4;
const num_bullets = 3;

const Scene = struct {
    broad_phase_layer_interface: Layers.BPLayerInterfaceImpl = .init(),
    object_vs_broad_phase_layer_filter: Layers.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_layer_pair_filter: Layers.ObjectLayerPairFilterImpl = .{},
    system: PhysicsSystem = .empty,
    activation_listener: ActivationCounter = .{},
    contact_listener: ContactCounter = .{},
    step_listener: PushListener = .{},
    constraints: [2]*TwoBodyConstraint = undefined,
    sleeping_body: BodyID = .invalid,
    bullets: [num_bullets]BodyID = @splat(.invalid),

    /// Builds the scene in place (the physics system must not move)
    fn init(self: *Scene, allocator: Allocator, layout: Layout) !void {
        self.* = .{};
        try self.system.init(allocator, io, 1024, 0, 4096, &self.broad_phase_layer_interface.base, &self.object_vs_broad_phase_layer_filter.base, &self.object_layer_pair_filter.base);
        errdefer self.system.deinit();
        self.system.setBodyActivationListener(.init(&self.activation_listener));
        self.system.setContactListener(.init(&self.contact_listener));
        const bi = self.system.getBodyInterface();

        // Floor (a Shape), its top at y = 0
        const floor_shape = try BoxShape.create(allocator, Vec3.init(50, 1, 50), .{});
        var floor_settings = BodyCreationSettings.initWithShape(floor_shape.asShape(), RVec3.init(0, -1, 0), Quat.identity(), .static, Layers.Layers.non_moving);
        defer floor_settings.deinit();
        _ = try bi.createAndAddBody(&floor_settings, .dont_activate);

        // Columns of spheres (shared ShapeSettings, converted on first use by a const BodyCreationSettings)
        var sphere_settings = BodyCreationSettings.initWithSettings((try SphereShapeSettings.create(allocator, 0.5, .{})).asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.Layers.moving);
        defer sphere_settings.deinit();
        var first_sphere: BodyID = .invalid;
        switch (layout) {
            .columns => for (0..num_columns) |column| {
                for (0..num_per_column) |i| {
                    const x: f32 = @as(f32, @floatFromInt(column)) * 1.5 - 4.0 + 0.01 * @as(f32, @floatFromInt(i % 3));
                    const y: f32 = 1.0 + 1.05 * @as(f32, @floatFromInt(i));
                    const z: f32 = 0.02 * @as(f32, @floatFromInt(i % 5)) - 0.5 * @as(f32, @floatFromInt(column % 2));
                    sphere_settings.position = RVec3.init(x, y, z);
                    sphere_settings.linear_velocity = Vec3.init(0.1 * @as(f32, @floatFromInt(column)), 0, -0.05 * @as(f32, @floatFromInt(i % 4)));
                    sphere_settings.angular_velocity = Vec3.init(0.3, 0.1 * @as(f32, @floatFromInt(i % 2)), 0);
                    const id = try bi.createAndAddBody(&sphere_settings, .activate);
                    try expect(!id.isInvalid());
                    if (first_sphere.isInvalid()) first_sphere = id;
                }
            },
            .grid => for (0..grid_size_z) |row| {
                for (0..grid_size_x) |column| {
                    // Touching neighbours on the floor: about 3 pairs per sphere
                    sphere_settings.position = RVec3.init(@as(f32, @floatFromInt(column)) - 4.0, 0.5, @as(f32, @floatFromInt(row)) - 4.0);
                    sphere_settings.linear_velocity = Vec3.init(0.01 * @as(f32, @floatFromInt(column % 3)), 0, 0);
                    const id = try bi.createAndAddBody(&sphere_settings, .activate);
                    try expect(!id.isInvalid());
                    if (first_sphere.isInvalid()) first_sphere = id;
                }
            },
        }

        // A sleeping sphere on the floor and a sphere that falls on it: a contact in a FindCollisions job wakes it up
        sphere_settings.linear_velocity = Vec3.zero();
        sphere_settings.angular_velocity = Vec3.zero();
        sphere_settings.position = RVec3.init(6, 0.5, 3);
        self.sleeping_body = try bi.createAndAddBody(&sphere_settings, .dont_activate);
        sphere_settings.position = RVec3.init(6, 3, 3);
        _ = try bi.createAndAddBody(&sphere_settings, .activate);

        // Fast bullets with the linear cast motion quality (CCD: PostIntegrate spawns FindCCDContacts jobs)
        sphere_settings.motion_quality = .linear_cast;
        sphere_settings.linear_velocity = Vec3.init(0, -60, 0);
        for (&self.bullets, 0..) |*bullet, i| {
            sphere_settings.position = RVec3.init(10, 20 + 3 * @as(f32, @floatFromInt(i)), 10 + 2 * @as(f32, @floatFromInt(i)));
            bullet.* = try bi.createAndAddBody(&sphere_settings, .activate);
        }
        sphere_settings.motion_quality = .discrete;

        // A pendulum attached to the world and a pair of spheres: the toy constraint (Phase 6 stand-in)
        var constraint_settings = ToyDistanceConstraintSettings.init(allocator);
        sphere_settings.position = RVec3.init(-8, 6, 0);
        sphere_settings.linear_velocity = Vec3.init(0, 0, 2);
        const pendulum = (try bi.createBody(&sphere_settings)).?;
        try bi.addBody(pendulum.getID(), .activate);
        {
            // Ref<TwoBodyConstraint> c = BodyInterface::CreateConstraint(settings, invalid, body); AddConstraint(c)
            var c = Ref(Constraint).init(&(try bi.createConstraint(&constraint_settings.base, .invalid, pendulum.getID())).base);
            defer c.deinit();
            try self.system.addConstraint(c.get().?);
            self.constraints[0] = virtual.downcast(TwoBodyConstraint, c.get().?);
        }

        sphere_settings.position = RVec3.init(8, 3, 0);
        sphere_settings.linear_velocity = Vec3.init(1, 0, 0);
        const a = try bi.createAndAddBody(&sphere_settings, .activate);
        sphere_settings.position = RVec3.init(8, 5, 0);
        sphere_settings.linear_velocity = Vec3.zero();
        const b = try bi.createAndAddBody(&sphere_settings, .dont_activate); // Activated through BuildIslandsFromConstraints
        {
            // TwoBodyConstraintSettings::Create(body1, body2) with the bodies locked
            var lock: BodyLockMultiWrite = .init(self.system.getBodyLockInterface(), &.{ a, b });
            defer lock.deinit();
            var c = Ref(Constraint).init(&(try constraint_settings.base.createConstraint(allocator, lock.getBody(0).?, lock.getBody(1).?)).base);
            defer c.deinit();
            try self.system.addConstraint(c.get().?);
            self.constraints[1] = virtual.downcast(TwoBodyConstraint, c.get().?);
        }

        // Step listener
        self.step_listener.body_id = first_sphere;
        try self.system.addStepListener(.init(&self.step_listener));
    }

    fn deinit(self: *Scene) void {
        self.system.deinit(); // Releases the constraint references, destroys the bodies
    }

    /// The PerformanceTest hash: FNV-1a over position (3 Reals) and rotation of every body in body ID order
    fn hash(self: *Scene) u64 {
        var h: u64 = HashCombine.fnv1a_seed;
        for (self.system.body_manager.getBodies()) |slot| {
            if (!slot.isValidBodyPointer()) continue;
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
    counts: [3]u32, // contacts added, activations, step listener calls
    order_hash: u64,
    num_ccd_bodies: u32,
    num_spawned_jobs: u32,
};

/// Runs the scene and records the saved state after every update
fn runScene(allocator: Allocator, kind: JobSystemKind, layout: Layout, states: *std.ArrayList(u8), offsets: *std.ArrayList(usize), stats: *RunStats) !u64 {
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit();

    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(Scene);
    defer allocator.destroy(scene);
    try scene.init(allocator, layout);
    defer scene.deinit();

    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();

    for (0..num_updates) |_| {
        const errors = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
        try expect(errors.isNone());
        try expect(temp.isEmpty());

        recorder.clear();
        try scene.system.saveState(recorder.recorder(), .all, null);
        try expect(!recorder.isFailed());
        try offsets.append(allocator, states.items.len);
        try states.appendSlice(allocator, recorder.getData());
    }

    stats.* = .{
        .counts = .{ scene.contact_listener.num_added.load(.monotonic), scene.activation_listener.num_activated.load(.monotonic), scene.step_listener.num_steps.load(.monotonic) },
        .order_hash = scene.contact_listener.order_hash.load(.monotonic),
        .num_ccd_bodies = scene.system.debug_num_ccd_bodies.load(.monotonic),
        .num_spawned_jobs = scene.system.debug_num_spawned_jobs.load(.monotonic),
    };

    // Sanity: the spheres rest on the floor (nothing fell through, the bullets were stopped by CCD), the pendulum keeps
    // its length approximately, the sleeping sphere was woken up by a contact
    var min_y: f32 = std.math.floatMax(f32);
    for (scene.system.body_manager.getBodies()) |slot| {
        if (slot.isValidBodyPointer() and slot.getBody().isDynamic())
            min_y = @min(min_y, @as(f32, @floatCast(slot.getBody().getPosition().getY())));
    }
    try expect(min_y > 0.3);
    for (scene.bullets) |bullet| {
        const y: f32 = @floatCast(scene.system.body_manager.getBody(bullet).getPosition().getY());
        try expect(y > 0.3 and y < 0.7);
    }
    const pendulum_length = scene.constraints[0].body2.getPosition().length();
    try expect(@abs(pendulum_length - 10.0) < 0.5);
    try expect(scene.system.body_manager.getBody(scene.sleeping_body).isActive());
    if (verbose)
        std.debug.print("{any}: contacts {d} activations {d} ccd {d} spawned {d} min y {d} pendulum {d} order {x}\n", .{ kind, stats.counts[0], stats.counts[1], stats.num_ccd_bodies, stats.num_spawned_jobs, min_y, pendulum_length, stats.order_hash });
    return scene.hash();
}

// -------------------------------------------------------------------------------------------------------------------
// Tests

test "SimulationPrototype body layout, fixed to world and the static shape" {
    if (@sizeOf(usize) == 8 and Core.object_layer_bits == 16)
        try expectEqual(@as(usize, if (Core.double_precision) 160 else 128), @sizeOf(Body));
    try expectEqual(@as(u8, 0b100011), @as(u8, @bitCast(AllowedDOFs.plane_2d)));

    const fixed = Body.fixedToWorld();
    try expect(fixed.isStatic());
    try expect(!fixed.isActive());
    try expect(fixed.getID().isInvalid());
    try expect(fixed.getShape().getSubType() == .empty);
    try expectEqual(@as(u32, Body.inactive_index), fixed.getIndexInActiveBodiesInternal());

    // The dummy shape is static (read-only memory, never reference counted): references to it are legal, also in
    // ReleaseFast, where a write to read-only memory would fault
    var shape = RefConst(Shape).init(fixed.getShape());
    var copy = shape.clone();
    copy.deinit();
    shape.deinit();
    try expect(fixed.getShape().is_static);
}

test "SimulationPrototype BodyManager ID allocation like Jolt" {
    const allocator = std.testing.allocator;
    var scene: Scene = .{};
    try scene.system.init(allocator, io, 4, 0, 16, &scene.broad_phase_layer_interface.base, &scene.object_vs_broad_phase_layer_filter.base, &scene.object_layer_pair_filter.base);
    defer scene.system.deinit();
    const bi = scene.system.getBodyInterface();

    const box = try BoxShape.create(allocator, Vec3.replicate(1), .{});
    var settings = BodyCreationSettings.initWithShape(box.asShape(), RVec3.zero(), Quat.identity(), .dynamic, Layers.Layers.moving);
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

    // AddBodyWithCustomID removes the slot from the free list and establishes the sequence number
    bi.removeBody(reused2);
    bi.destroyBody(reused2);
    const custom = (try bi.createBodyWithID(.fromIndexAndSequenceNumber(1, 7), &settings)).?;
    try expectEqual(@as(u8, 7), custom.getID().getSequenceNumber());
    try expect((try bi.createBodyWithID(.fromIndexAndSequenceNumber(1, 8), &settings)) == null); // Slot in use
    bi.destroyBody(custom.getID());
    const after_custom = try bi.createAndAddBody(&settings, .activate);
    try expectEqual(@as(u32, 1), after_custom.getIndex());
    try expectEqual(@as(u8, 8), after_custom.getSequenceNumber());
}

test "SimulationPrototype body locks, the tagged lock interface and BodyInterface" {
    const allocator = std.testing.allocator;
    var scene: Scene = .{};
    try scene.system.init(allocator, io, 64, 8, 16, &scene.broad_phase_layer_interface.base, &scene.object_vs_broad_phase_layer_filter.base, &scene.object_layer_pair_filter.base);
    defer scene.system.deinit();
    const system = &scene.system;
    const bi = system.getBodyInterface();

    const box = try BoxShape.create(allocator, Vec3.replicate(1), .{});
    var settings = BodyCreationSettings.initWithShape(box.asShape(), RVec3.init(1, 2, 3), Quat.identity(), .dynamic, Layers.Layers.moving);
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

    // Invalid IDs never lock
    var invalid_lock: BodyLockRead = .init(system.getBodyLockInterface(), .invalid);
    try expect(!invalid_lock.succeeded());
    invalid_lock.deinit();

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
        fn run(body_interface: *const BodyInterface, body_id: BodyID, n: u32) void {
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
    var collector = AllHitCollisionCollector(BroadPhaseFile.CollideShapeBodyCollector).init(allocator);
    defer collector.deinit();
    query.collideAABox(&AABox.init(Vec3.init(0, 1, 2), Vec3.init(2, 3, 4)), &collector.base, .{});
    try collector.checkError();
    try expectEqual(@as(usize, 3), collector.hits.items.len); // id, batch[0] and batch[1] (same box at the same place)
}

test "SimulationPrototype BodyCreationSettings shape union (Rule M) and CreateBodyError" {
    const allocator = std.testing.allocator;

    // A const BodyCreationSettings converts the settings on demand: the cache is written through the Ref (Rule M)
    const box_settings = try BoxShapeSettings.create(allocator, Vec3.init(1, 2, 3), .{});
    var settings = BodyCreationSettings.initWithSettings(box_settings.asShapeSettings(), RVec3.zero(), Quat.identity(), .dynamic, Layers.Layers.moving);
    defer settings.deinit();
    const const_settings: *const BodyCreationSettings = &settings;
    const shape1 = (try const_settings.getShape(allocator)).?;
    const shape2 = (try const_settings.getShape(allocator)).?;
    try expect(shape1 == shape2); // Cached in the ShapeSettings
    try expect(!box_settings.base.base.cached_result.isEmpty());

    // ConvertShapeSettings releases the settings and keeps the shape
    var result = try settings.convertShapeSettings(allocator);
    defer result.deinit();
    try expect(result.isValid());
    try expect(settings.getShapeSettings() == null);
    try expect((try const_settings.getShape(allocator)).? == shape1);

    // No shape at all: Jolt's text from convertShapeSettings, null from getShape, error.InvalidShape from createBody
    var empty: BodyCreationSettings = .{};
    defer empty.deinit();
    var no_shape = try empty.convertShapeSettings(allocator);
    defer no_shape.deinit();
    try expect(no_shape.hasError());
    try std.testing.expectEqualStrings("No shape present!", no_shape.getError());
    try expect((try empty.getShape(allocator)) == null);

    var scene: Scene = .{};
    try scene.system.init(allocator, io, 4, 0, 16, &scene.broad_phase_layer_interface.base, &scene.object_vs_broad_phase_layer_filter.base, &scene.object_layer_pair_filter.base);
    defer scene.system.deinit();
    try std.testing.expectError(error.InvalidShape, scene.system.getBodyInterface().createBody(&empty));
}

/// Runs the scene on JobSystemSingleThreaded (the reference) and on thread pools: the saved state after every update
/// must be bit identical. Returns the number of FindCollisions jobs spawned from inside jobs.
fn determinismMatrix(layout: Layout, kinds: []const JobSystemKind) !u32 {
    const allocator = std.testing.allocator;

    // Reference: single threaded job system
    var ref_states: std.ArrayList(u8) = .empty;
    defer ref_states.deinit(allocator);
    var ref_offsets: std.ArrayList(usize) = .empty;
    defer ref_offsets.deinit(allocator);
    var ref_stats: RunStats = undefined;
    const ref_hash = try runScene(allocator, .single_threaded, layout, &ref_states, &ref_offsets, &ref_stats);
    try expectEqual(@as(u32, num_updates * collision_steps), ref_stats.counts[2]);
    try expect(ref_stats.counts[0] > 0);
    try expect(ref_stats.num_ccd_bodies > 0); // The bullets went through the CCD jobs

    var num_different_orders: u32 = 0;
    var num_spawned_jobs: u32 = 0;
    for (kinds) |kind| {
        var states: std.ArrayList(u8) = .empty;
        defer states.deinit(allocator);
        var offsets: std.ArrayList(usize) = .empty;
        defer offsets.deinit(allocator);
        var stats: RunStats = undefined;
        const h = try runScene(allocator, kind, layout, &states, &offsets, &stats);
        if (stats.order_hash != ref_stats.order_hash) num_different_orders += 1;
        num_spawned_jobs += stats.num_spawned_jobs;

        // Bit identical state after every update (the order of contacts and activations differs between runs)
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
        try expectEqual(ref_stats.num_ccd_bodies, stats.num_ccd_bodies);
    }
    if (verbose) std.debug.print("{any}: runs with a different contact order than single threaded: {d} of {d}, spawned jobs {d}\n", .{ layout, num_different_orders, kinds.len, num_spawned_jobs });
    return num_spawned_jobs;
}

test "SimulationPrototype determinism single vs multi threaded" {
    // Thread pools with 0 (barrier executes everything), 1, 3 and 7 worker threads, several times each
    _ = try determinismMatrix(.columns, &.{ .{ .thread_pool = 0 }, .{ .thread_pool = 1 }, .{ .thread_pool = 3 }, .{ .thread_pool = 7 }, .{ .thread_pool = 3 }, .{ .thread_pool = 7 } });
}

test "SimulationPrototype determinism with FindCollisions jobs spawned inside jobs" {
    // Few active body batches: the initial FindCollisions jobs leave job slots free, TrySpawnJobFindCollisions fills them
    const num_spawned_jobs = try determinismMatrix(.grid, &.{ .{ .thread_pool = 3 }, .{ .thread_pool = 7 }, .{ .thread_pool = 7 } });
    if ((std.Thread.getCpuCount() catch 1) >= 4)
        try expect(num_spawned_jobs > 0);
}

test "SimulationPrototype state recorder save, restore, validation and filter" {
    const allocator = std.testing.allocator;
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, .{ .thread_pool = 3 });
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(Scene);
    defer allocator.destroy(scene);
    try scene.init(allocator, .columns);
    defer scene.deinit();

    for (0..10) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    // Snapshot, simulate, restore, simulate again: identical
    var snapshot = StateRecorderImpl.init(allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .all, null);

    var after1 = StateRecorderImpl.init(allocator);
    defer after1.deinit();
    for (0..10) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    try scene.system.saveState(after1.recorder(), .all, null);

    snapshot.rewind();
    try expect(try scene.system.restoreState(snapshot.recorder()));
    var restored = StateRecorderImpl.init(allocator);
    defer restored.deinit();
    try scene.system.saveState(restored.recorder(), .all, null);
    try expect(restored.isEqual(&snapshot));

    var after2 = StateRecorderImpl.init(allocator);
    defer after2.deinit();
    for (0..10) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    try scene.system.saveState(after2.recorder(), .all, null);
    try expect(after2.isEqual(&after1));

    // Validating restore: reading the state we just saved finds no mismatch
    after2.rewind();
    after2.base.setValidating(true);
    try expect(try scene.system.restoreState(after2.recorder()));
    try expectEqual(@as(u32, 0), after2.num_mismatches);

    // A partial state (bodies only) with a filter
    const SkipFloor = struct {
        pub const overrides = .{.shouldSaveBody};
        base: StateRecorderFile.StateRecorderFilter = .init(@This()),
        pub fn shouldSaveBody(self: *const @This(), body: *const Body) bool {
            _ = self;
            return !body.isStatic();
        }
    };
    const filter: SkipFloor = .{};
    var partial = StateRecorderImpl.init(allocator);
    defer partial.deinit();
    try scene.system.saveState(partial.recorder(), .{ .bodies = true }, &filter.base);
    try expect(partial.getData().len < after2.getData().len);
}

/// TempAllocator that fails its n-th allocation (counts all calls; temp allocations are serialized by the job graph)
const FailingTempAllocator = struct {
    inner: *TempAllocatorImpl,
    fail_index: u32 = std.math.maxInt(u32),
    num_allocations: u32 = 0,

    pub fn allocate(self: *FailingTempAllocator, size: u32) TempAllocator.Error!?TempAllocator.Block {
        defer self.num_allocations += 1;
        if (self.num_allocations == self.fail_index)
            return error.OutOfMemory;
        return self.inner.allocate(size);
    }

    pub fn free(self: *FailingTempAllocator, address: ?*anyopaque, size: u32) void {
        self.inner.free(address, size);
    }
};

/// Restore `snapshot`, run one update with enough memory and compare with `reference`
fn expectRecovery(scene: *Scene, snapshot: *StateRecorderImpl, reference: *const StateRecorderImpl, temp: *TempAllocatorImpl, job_system: JobSystem) !void {
    const allocator = std.testing.allocator;
    snapshot.rewind();
    try expect(try scene.system.restoreState(snapshot.recorder()));
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), job_system);
    var redo = StateRecorderImpl.init(allocator);
    defer redo.deinit();
    try scene.system.saveState(redo.recorder(), .all, null);
    try expect(redo.isEqual(reference));
}

fn expectState(scene: *Scene, expected: *const StateRecorderImpl) !void {
    var state = StateRecorderImpl.init(std.testing.allocator);
    defer state.deinit();
    try scene.system.saveState(state.recorder(), .all, null);
    try expect(state.isEqual(expected));
}

fn outOfTempMemory(kind: JobSystemKind) !void {
    const allocator = std.testing.allocator;
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(Scene);
    defer allocator.destroy(scene);
    try scene.init(allocator, .columns);
    defer scene.deinit();
    for (0..5) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    var snapshot = StateRecorderImpl.init(allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .all, null);

    // Reference: one update with enough memory, counting the temp allocations (5 while building, the rest in jobs:
    // island builder, CCD buffers, the sorted CCD bodies, the next step's contact links)
    var counting: FailingTempAllocator = .{ .inner = &temp };
    _ = try scene.system.update(delta_time, collision_steps, TempAllocator.init(&counting), jobs.jobSystem());
    var reference = StateRecorderImpl.init(allocator);
    defer reference.deinit();
    try scene.system.saveState(reference.recorder(), .all, null);
    const num_allocations = counting.num_allocations;
    try expect(num_allocations >= 20);

    // Fail every allocation in turn
    for (0..num_allocations) |fail_index| {
        snapshot.rewind();
        try expect(try scene.system.restoreState(snapshot.recorder()));

        var failing: FailingTempAllocator = .{ .inner = &temp, .fail_index = @intCast(fail_index) };
        try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, TempAllocator.init(&failing), jobs.jobSystem()));
        try expect(temp.isEmpty()); // LIFO: every block was freed at Jolt's positions, also after skipped jobs

        // Failure while building the jobs (steps, body pairs, active constraints, contacts, contact links): no job ran
        if (fail_index < 5)
            try expectState(scene, &snapshot);

        // The system recovers: restore and redo the update
        try expectRecovery(scene, &snapshot, &reference, &temp, jobs.jobSystem());
    }
}

test "SimulationPrototype out of temp memory at every allocation, single threaded" {
    try outOfTempMemory(.single_threaded);
}

test "SimulationPrototype out of temp memory at every allocation, multi threaded" {
    try outOfTempMemory(.{ .thread_pool = 3 });
}

/// A job system that fails its n-th createJob (wraps another job system; jobs belong to the inner job system)
const FailingJobSystem = struct {
    inner: JobSystem,
    fail_index: u32 = std.math.maxInt(u32),
    num_created: std.atomic.Value(u32) = .init(0),
    failed: std.atomic.Value(bool) = .init(false),

    pub fn jobSystem(self: *FailingJobSystem) JobSystem {
        return .init(self);
    }
    pub fn getMaxConcurrency(self: *const FailingJobSystem) i32 {
        return self.inner.getMaxConcurrency();
    }
    pub fn createJob(self: *FailingJobSystem, job_name: []const u8, color: Color, job_function: JobSystem.JobFunction, opts: struct { num_dependencies: u32 = 0 }) error{OutOfMemory}!JobHandle {
        if (self.num_created.fetchAdd(1, .monotonic) == self.fail_index) {
            self.failed.store(true, .monotonic);
            return error.OutOfMemory;
        }
        return self.inner.createJob(job_name, color, job_function, .{ .num_dependencies = opts.num_dependencies });
    }
    pub fn createBarrier(self: *FailingJobSystem) ?*JobSystem.Barrier {
        return self.inner.createBarrier();
    }
    pub fn destroyBarrier(self: *FailingJobSystem, barrier: *JobSystem.Barrier) void {
        self.inner.destroyBarrier(barrier);
    }
    pub fn waitForJobs(self: *FailingJobSystem, barrier: *JobSystem.Barrier) void {
        self.inner.waitForJobs(barrier);
    }
    pub fn queueJob(self: *FailingJobSystem, job: *JobSystem.Job) void {
        self.inner.vtable.queueJob(self.inner.ptr, job);
    }
    pub fn queueJobs(self: *FailingJobSystem, jobs: []const *JobSystem.Job) void {
        self.inner.vtable.queueJobs(self.inner.ptr, jobs);
    }
    pub fn freeJob(self: *FailingJobSystem, job: *JobSystem.Job) void {
        self.inner.vtable.freeJob(self.inner.ptr, job);
    }
};

fn failingCreateJob(kind: JobSystemKind, layout: Layout) !void {
    const allocator = std.testing.allocator;
    var jobs: AnyJobSystem = .{};
    try jobs.init(allocator, kind);
    defer jobs.deinit(); // JobSystemSingleThreaded / ThreadPool: every job was freed
    var temp: TempAllocatorImpl = try .init(allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    const scene = try allocator.create(Scene);
    defer allocator.destroy(scene);
    try scene.init(allocator, layout);
    defer scene.deinit();
    for (0..5) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    var snapshot = StateRecorderImpl.init(allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .all, null);

    // Reference: one update, counting the jobs (the bullets are still falling: FindCCDContacts jobs are spawned)
    var counting: FailingJobSystem = .{ .inner = jobs.jobSystem() };
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), counting.jobSystem());
    var reference = StateRecorderImpl.init(allocator);
    defer reference.deinit();
    try scene.system.saveState(reference.recorder(), .all, null);
    const num_jobs = counting.num_created.load(.monotonic);

    // Fail every createJob in turn: while building the graph the update fails without side effects (start gate), a job
    // that cannot be spawned from inside a job is not an error and does not change the results
    var num_build_failures: u32 = 0;
    var num_spawn_failures: u32 = 0;
    for (0..num_jobs + 4) |fail_index| {
        snapshot.rewind();
        try expect(try scene.system.restoreState(snapshot.recorder()));
        const steps_before = scene.step_listener.num_steps.load(.monotonic);

        var failing: FailingJobSystem = .{ .inner = jobs.jobSystem(), .fail_index = @intCast(fail_index) };
        if (scene.system.update(delta_time, collision_steps, temp.tempAllocator(), failing.jobSystem())) |errors| {
            try expect(errors.isNone());
            if (failing.failed.load(.monotonic)) num_spawn_failures += 1;
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
    try expect(num_build_failures >= 2 * 18); // 18 job types per step (17 handles + StartNextStep for the first step), more with more threads
    try expect(num_spawn_failures > 0); // At least the FindCCDContacts spawns
}

test "SimulationPrototype createJob fails at every job, single threaded" {
    try failingCreateJob(.single_threaded, .columns);
}

test "SimulationPrototype createJob fails at every job, multi threaded" {
    try failingCreateJob(.{ .thread_pool = 3 }, .grid); // Also fails FindCollisions spawns
}

/// A thread safe allocator that fails its n-th allocation (heap allocations happen inside jobs on worker threads)
const FailingHeapAllocator = struct {
    inner: Allocator,
    fail_index: u32 = std.math.maxInt(u32),
    num_allocations: std.atomic.Value(u32) = .init(0),

    fn allocator(self: *FailingHeapAllocator) Allocator {
        return .{ .ptr = self, .vtable = &.{ .alloc = alloc, .resize = Allocator.noResize, .remap = Allocator.noRemap, .free = free } };
    }

    fn alloc(ctx: *anyopaque, len: usize, alignment: std.mem.Alignment, ret_addr: usize) ?[*]u8 {
        const self: *FailingHeapAllocator = @ptrCast(@alignCast(ctx));
        if (self.num_allocations.fetchAdd(1, .monotonic) == self.fail_index)
            return null;
        return self.inner.rawAlloc(len, alignment, ret_addr);
    }

    fn free(ctx: *anyopaque, memory: []u8, alignment: std.mem.Alignment, ret_addr: usize) void {
        const self: *FailingHeapAllocator = @ptrCast(@alignCast(ctx));
        self.inner.rawFree(memory, alignment, ret_addr);
    }
};

test "SimulationPrototype out of heap memory inside a job (BroadPhase.updatePrepare)" {
    var heap: FailingHeapAllocator = .{ .inner = std.testing.allocator };
    const allocator = heap.allocator();
    var jobs: AnyJobSystem = .{};
    try jobs.init(std.testing.allocator, .{ .thread_pool = 3 });
    defer jobs.deinit();
    var temp: TempAllocatorImpl = try .init(std.testing.allocator, 4 * 1024 * 1024);
    defer temp.deinit();

    const scene = try std.testing.allocator.create(Scene);
    defer std.testing.allocator.destroy(scene);
    try scene.init(allocator, .columns);
    defer scene.deinit();
    for (0..5) |_| _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());

    var snapshot = StateRecorderImpl.init(std.testing.allocator);
    defer snapshot.deinit();
    try scene.system.saveState(snapshot.recorder(), .all, null);

    // Reference: count the heap allocations of one update (the new broad phase "tree" built in the BroadPhasePrepare job)
    const before = heap.num_allocations.load(.monotonic);
    _ = try scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem());
    const num_allocations = heap.num_allocations.load(.monotonic) - before;
    try expect(num_allocations >= 1);
    var reference = StateRecorderImpl.init(std.testing.allocator);
    defer reference.deinit();
    try scene.system.saveState(reference.recorder(), .all, null);

    for (0..num_allocations) |i| {
        snapshot.rewind();
        try expect(try scene.system.restoreState(snapshot.recorder()));
        heap.fail_index = heap.num_allocations.load(.monotonic) + @as(u32, @intCast(i));
        try std.testing.expectError(error.OutOfMemory, scene.system.update(delta_time, collision_steps, temp.tempAllocator(), jobs.jobSystem()));
        heap.fail_index = std.math.maxInt(u32);
        try expect(temp.isEmpty());
        try expectRecovery(scene, &snapshot, &reference, &temp, jobs.jobSystem());
    }
}

test "SimulationPrototype out of memory creating the scene" {
    // Every allocation of scene creation fails in turn (system, shapes, bodies, AddBodiesPrepare, constraints,
    // listeners): errors propagate, nothing leaks
    const Ctx = struct {
        fn build(allocator: Allocator) !void {
            const scene = try allocator.create(Scene);
            defer allocator.destroy(scene);
            try scene.init(allocator, .columns);
            scene.deinit();
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, Ctx.build, .{});
}

test "SimulationPrototype determinism log format" {
    // Jolt's text: std::hex with fill '0', floats as their bits, BodyIDs as 8 hex digits (sequence number << 23 | index)
    var buffer: [256]u8 = undefined;
    var w: std.Io.Writer = .fixed(&buffer);
    try DeterminismLog.write(&w, .{ "JobIntegrateVelocity: id: ", BodyID.fromIndexAndSequenceNumber(1, 2), " v: ", Vec3.init(1, 0, -1), " i: ", @as(i32, -3) });
    try std.testing.expectEqualStrings("JobIntegrateVelocity: id: 01000001 v: 3f800000 00000000 bf800000 i: -3\n", w.buffered());
    if (!DeterminismLog.enabled) try expectEqual(void, @TypeOf(DeterminismLog.writer)); // The global only exists when enabled
}
