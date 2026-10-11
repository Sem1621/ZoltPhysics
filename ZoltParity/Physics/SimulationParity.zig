//! Simulation parity (SimulationArchitecture D12): Zolt's PhysicsSystem against Jolt's, update by update.
//!
//! Every scene of SimScenes.zig is built on both sides from the same description (`SimSceneDesc`, the layers of
//! UnitTests/Layers.h / ZoltTests/Layers.zig), the same operations are applied in the same order between the updates,
//! and after every update the following must be identical:
//! - the PhysicsSystem::SaveState(EStateRecorderState::All) bytes. On a mismatch the first differing byte is decoded to
//!   (section, body, field) with the layout of PhysicsSystem / BodyManager / Body / MotionProperties::SaveState (and
//!   the contact cache / constraint sections), and the first differing fields are listed with both values;
//! - the EPhysicsUpdateError bits returned by Update and the result of every operation (body IDs, IsActive / IsAdded);
//! - the listener callbacks of the update and of the operations before it as sorted records (body activation listener,
//!   step listeners, contact listener): their order depends on the threads, their content does not.
//! At the end of a scene the partial states (each EStateRecorderState flag) are compared too.
//!
//! Thread matrix: Zolt with JobSystemSingleThreaded and JobSystemThreadPool with 0, 1, 3 and 7 workers, C++ with a
//! JobSystemThreadPool with 0, 3 and 7 workers (`configs`). Run it in Debug and ReleaseFast, single and double precision.
//!
//! Per scene (D12), in the configurations with `sweeps`: before update 0 and every `sweep_stride`-th update every temp
//! allocation of the update fails once (FailingTempAllocator: error.OutOfMemory, the temp allocator is empty, a failure
//! in the first two allocations (the steps, the body pairs: before any job ran) leaves the state unchanged, and after
//! restoring the state saved before the update one more update gives the C++ bytes), then every createJob fails once
//! (FailingJobSystem: a failure while building the job graph returns error.OutOfMemory and changes nothing, a job that
//! cannot be spawned from inside a job changes no result). Creating the scene (system, shapes, the bodies of update 0)
//! runs under std.testing.checkAllAllocationFailures.
//!
//! Every scene is its own test (`SceneTest(name, index)`). A scene whose `blocked_by` is not supported yet
//! (SimScenes.Blocker.isSupported) returns error.SkipZigTest: the build summary counts it as skipped. Add a scene in
//! SimScenes.zig (see its header).
//!
//! C ABI wrappers: ZoltParity/Physics/SimulationReference.cpp.

const std = @import("std");
const zolt = @import("zolt");
const TestLayers = @import("test_layers");
const SimScenes = @import("SimScenes.zig");

const Allocator = std.mem.Allocator;
const Core = zolt.Core;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const RVec3 = zolt.RVec3;
const Ref = zolt.Ref;
const RefConst = zolt.RefConst;
const Mutex = zolt.Mutex;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyInterface = zolt.BodyInterface;
const BodyCreationSettings = zolt.BodyCreationSettings;
const Activation = zolt.Activation;
const MotionType = zolt.MotionType;
const MotionQuality = zolt.MotionQuality;
const AllowedDOFs = zolt.AllowedDOFs;
const ObjectLayer = zolt.ObjectLayer;
const Shape = zolt.Shape;
const ShapeSettings = zolt.ShapeSettings;
const PhysicsSystem = zolt.PhysicsSystem;
const PhysicsSettings = zolt.PhysicsSettings;
const PhysicsStepListener = zolt.PhysicsStepListener;
const PhysicsStepListenerContext = zolt.PhysicsStepListenerContext;
const BodyActivationListener = zolt.BodyActivationListener;
const ContactListener = zolt.ContactListener;
const ContactManifold = zolt.ContactManifold;
const ContactSettings = zolt.ContactSettings;
const ValidateResult = zolt.ValidateResult;
const CollideShapeResult = zolt.CollideShapeResult;
const SubShapeIDPair = zolt.SubShapeIDPair;
const StateRecorderImpl = zolt.StateRecorderImpl;
const StateRecorderState = zolt.StateRecorderState;
const PhysicsUpdateError = zolt.PhysicsUpdateError;
const TempAllocator = zolt.TempAllocator;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const FailingTempAllocator = zolt.FailingTempAllocator;
const JobSystem = zolt.JobSystem;
const JobSystemSingleThreaded = zolt.JobSystemSingleThreaded;
const JobSystemThreadPool = zolt.JobSystemThreadPool;
const FailingJobSystem = zolt.FailingJobSystem;

const SimShape = SimScenes.SimShape;
const SimBody = SimScenes.SimBody;
const SimPhysicsSettings = SimScenes.SimPhysicsSettings;
const SimStepListener = SimScenes.SimStepListener;
const SimConstraint = SimScenes.SimConstraint;
const SimOp = SimScenes.SimOp;
const SimSceneDesc = SimScenes.SimSceneDesc;
const SimRecord = SimScenes.SimRecord;
const SimCompoundChild = SimScenes.SimCompoundChild;
const SceneBuilder = SimScenes.SceneBuilder;
const SceneEntry = SimScenes.SceneEntry;
const OpKind = SimScenes.OpKind;
const ShapeKind = SimScenes.ShapeKind;
const RecordKind = SimScenes.RecordKind;
const StepListenerMode = SimScenes.StepListenerMode;
const invalid_handle = SimScenes.invalid_handle;

const io = std.testing.io;

/// The C++ reference functions, see SimulationReference.cpp
const jolt = struct {
    extern fn jolt_sim_struct_sizes(out_sizes: *[12]u32) void;
    extern fn jolt_sim_default_settings(out_settings: *SimPhysicsSettings) void;
    extern fn jolt_sim_create(desc: *const SimSceneDesc, num_threads: u32) *anyopaque;
    extern fn jolt_sim_destroy(world: *anyopaque) void;
    extern fn jolt_sim_apply_op(world: *anyopaque, op: *const SimOp) u32;
    extern fn jolt_sim_update(world: *anyopaque, delta_time: f32, collision_steps: u32) u32;
    extern fn jolt_sim_save_state(world: *anyopaque, state: u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_sim_restore_state(world: *anyopaque, bytes: [*]const u8, size: u32) u32;
    extern fn jolt_sim_take_records(world: *anyopaque, out_records: [*]SimRecord, capacity: u32) u32;
    extern fn jolt_sim_body_has_motion_properties(world: *anyopaque, body_id: u32) u32;
    extern fn jolt_sim_constraint_state_size(world: *anyopaque, index: u32, out_num_constraints: *u32) u32;
};

// -------------------------------------------------------------------------------------------------------------------
// Conversions (the C++ side does the same in SimulationReference.cpp)

fn load3(v: []const f32) Vec3 {
    return Vec3.init(v[0], v[1], v[2]);
}

fn loadQuat(v: []const f32) Quat {
    return Quat.init(v[0], v[1], v[2], v[3]);
}

fn loadR3(v: []const Real) RVec3 {
    return RVec3.init(v[0], v[1], v[2]);
}

fn toPhysicsSettings(s: *const SimPhysicsSettings) PhysicsSettings {
    return .{
        .max_in_flight_body_pairs = s.max_in_flight_body_pairs,
        .step_listeners_batch_size = s.step_listeners_batch_size,
        .step_listener_batches_per_job = s.step_listener_batches_per_job,
        .baumgarte = s.baumgarte,
        .speculative_contact_distance = s.speculative_contact_distance,
        .penetration_slop = s.penetration_slop,
        .linear_cast_threshold = s.linear_cast_threshold,
        .linear_cast_max_penetration = s.linear_cast_max_penetration,
        .manifold_tolerance = s.manifold_tolerance,
        .max_penetration_distance = s.max_penetration_distance,
        .body_pair_cache_max_delta_position_sq = s.body_pair_cache_max_delta_position_sq,
        .body_pair_cache_cos_max_delta_rotation_div2 = s.body_pair_cache_cos_max_delta_rotation_div2,
        .contact_normal_cos_max_delta_rotation = s.contact_normal_cos_max_delta_rotation,
        .contact_point_preserve_lambda_max_dist_sq = s.contact_point_preserve_lambda_max_dist_sq,
        .internal_edge_removal_vertex_tolerance_sq = s.internal_edge_removal_vertex_tolerance_sq,
        .num_velocity_steps = s.num_velocity_steps,
        .num_position_steps = s.num_position_steps,
        .min_velocity_for_restitution = s.min_velocity_for_restitution,
        .time_before_sleep = s.time_before_sleep,
        .point_velocity_sleep_threshold = s.point_velocity_sleep_threshold,
        .constraint_warm_start = s.constraint_warm_start != 0,
        .use_body_pair_contact_cache = s.use_body_pair_contact_cache != 0,
        .use_manifold_reduction = s.use_manifold_reduction != 0,
        .use_large_island_splitter = s.use_large_island_splitter != 0,
        .allow_sleeping = s.allow_sleeping != 0,
        .check_active_edges = s.check_active_edges != 0,
    };
}

fn fromPhysicsSettings(s: *const PhysicsSettings) SimPhysicsSettings {
    return .{
        .max_in_flight_body_pairs = s.max_in_flight_body_pairs,
        .step_listeners_batch_size = s.step_listeners_batch_size,
        .step_listener_batches_per_job = s.step_listener_batches_per_job,
        .baumgarte = s.baumgarte,
        .speculative_contact_distance = s.speculative_contact_distance,
        .penetration_slop = s.penetration_slop,
        .linear_cast_threshold = s.linear_cast_threshold,
        .linear_cast_max_penetration = s.linear_cast_max_penetration,
        .manifold_tolerance = s.manifold_tolerance,
        .max_penetration_distance = s.max_penetration_distance,
        .body_pair_cache_max_delta_position_sq = s.body_pair_cache_max_delta_position_sq,
        .body_pair_cache_cos_max_delta_rotation_div2 = s.body_pair_cache_cos_max_delta_rotation_div2,
        .contact_normal_cos_max_delta_rotation = s.contact_normal_cos_max_delta_rotation,
        .contact_point_preserve_lambda_max_dist_sq = s.contact_point_preserve_lambda_max_dist_sq,
        .internal_edge_removal_vertex_tolerance_sq = s.internal_edge_removal_vertex_tolerance_sq,
        .num_velocity_steps = s.num_velocity_steps,
        .num_position_steps = s.num_position_steps,
        .min_velocity_for_restitution = s.min_velocity_for_restitution,
        .time_before_sleep = s.time_before_sleep,
        .point_velocity_sleep_threshold = s.point_velocity_sleep_threshold,
        .constraint_warm_start = @intFromBool(s.constraint_warm_start),
        .use_body_pair_contact_cache = @intFromBool(s.use_body_pair_contact_cache),
        .use_manifold_reduction = @intFromBool(s.use_manifold_reduction),
        .use_large_island_splitter = @intFromBool(s.use_large_island_splitter),
        .allow_sleeping = @intFromBool(s.allow_sleeping),
        .check_active_edges = @intFromBool(s.check_active_edges),
    };
}

fn floatBits(v: f32) u32 {
    return @bitCast(v);
}

// -------------------------------------------------------------------------------------------------------------------
// Listeners of the Zolt side (callbacks come from the worker threads)

/// Collects the callback records (a failed append is reported by `checkError`: the callbacks cannot return errors)
const Recorder = struct {
    allocator: Allocator,
    mutex: Mutex = .{},
    records: std.ArrayList(SimRecord) = .empty,
    out_of_memory: bool = false,

    fn deinit(self: *Recorder) void {
        self.records.deinit(self.allocator);
    }

    fn add(self: *Recorder, record: SimRecord) void {
        self.mutex.lock(io);
        defer self.mutex.unlock(io);
        self.records.append(self.allocator, record) catch {
            self.out_of_memory = true;
        };
    }

    fn checkError(self: *const Recorder) Allocator.Error!void {
        if (self.out_of_memory) return error.OutOfMemory;
    }
};

const ActivationListenerImpl = struct {
    recorder: *Recorder,

    fn record(self: *ActivationListenerImpl, kind: RecordKind, body_id: BodyID, body_user_data: u64) void {
        var r: SimRecord = .{ .kind = @intFromEnum(kind) };
        r.ids[0] = body_id.getIndexAndSequenceNumber();
        r.data[0] = @truncate(body_user_data);
        r.data[1] = @truncate(body_user_data >> 32);
        self.recorder.add(r);
    }

    pub fn onBodyActivated(self: *ActivationListenerImpl, body_id: BodyID, body_user_data: u64) void {
        self.record(.body_activated, body_id, body_user_data);
    }

    pub fn onBodyDeactivated(self: *ActivationListenerImpl, body_id: BodyID, body_user_data: u64) void {
        self.record(.body_deactivated, body_id, body_user_data);
    }
};

const ContactListenerImpl = struct {
    recorder: *Recorder,

    pub fn onContactValidate(self: *ContactListenerImpl, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult {
        _ = base_offset;
        var r: SimRecord = .{ .kind = @intFromEnum(RecordKind.contact_validate) };
        r.ids = .{ body1.getID().getIndexAndSequenceNumber(), body2.getID().getIndexAndSequenceNumber(), collision_result.sub_shape_id1.getValue(), collision_result.sub_shape_id2.getValue() };
        r.data[0] = floatBits(collision_result.penetration_depth);
        r.data[1] = floatBits(collision_result.penetration_axis.getX());
        r.data[2] = floatBits(collision_result.penetration_axis.getY());
        r.data[3] = floatBits(collision_result.penetration_axis.getZ());
        self.recorder.add(r);
        return .accept_all_contacts_for_this_body_pair;
    }

    fn record(self: *ContactListenerImpl, kind: RecordKind, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *const ContactSettings) void {
        var r: SimRecord = .{ .kind = @intFromEnum(kind) };
        r.ids = .{ body1.getID().getIndexAndSequenceNumber(), body2.getID().getIndexAndSequenceNumber(), manifold.sub_shape_id1.getValue(), manifold.sub_shape_id2.getValue() };
        r.data = .{
            floatBits(manifold.world_space_normal.getX()),
            floatBits(manifold.world_space_normal.getY()),
            floatBits(manifold.world_space_normal.getZ()),
            floatBits(manifold.penetration_depth),
            @intCast(manifold.relative_contact_points_on1.len),
            floatBits(settings.combined_friction),
            floatBits(settings.combined_restitution),
            @intFromBool(settings.is_sensor),
        };
        self.recorder.add(r);
    }

    pub fn onContactAdded(self: *ContactListenerImpl, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        self.record(.contact_added, body1, body2, manifold, settings);
    }

    pub fn onContactPersisted(self: *ContactListenerImpl, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        self.record(.contact_persisted, body1, body2, manifold, settings);
    }

    pub fn onContactRemoved(self: *ContactListenerImpl, sub_shape_pair: *const SubShapeIDPair) void {
        var r: SimRecord = .{ .kind = @intFromEnum(RecordKind.contact_removed) };
        r.ids = .{ sub_shape_pair.body1_id.getIndexAndSequenceNumber(), sub_shape_pair.sub_shape_id1.getValue(), sub_shape_pair.body2_id.getIndexAndSequenceNumber(), sub_shape_pair.sub_shape_id2.getValue() };
        self.recorder.add(r);
    }
};

/// SimStepListenerImpl of SimulationReference.cpp
const StepListenerImpl = struct {
    world: *ZoltWorld,
    index: u32,
    num_calls: u32 = 0,

    pub fn onStep(self: *StepListenerImpl, context: *const PhysicsStepListenerContext) void {
        var r: SimRecord = .{ .kind = @intFromEnum(RecordKind.step) };
        r.ids[0] = self.index;
        r.ids[1] = self.num_calls;
        self.num_calls += 1;
        r.data[0] = floatBits(context.delta_time);
        r.data[1] = @intFromBool(context.is_first_step);
        r.data[2] = @intFromBool(context.is_last_step);
        self.world.recorder.add(r);

        // Change the velocity of the body (StepListenerMode), all bodies are locked: the no-lock interface
        const desc = &self.world.desc.step_listeners[self.index];
        if (desc.body == invalid_handle)
            return;
        const id = self.world.getHandle(desc.body);
        const bi = context.physics_system.getBodyInterfaceNoLockMut();
        const dt = context.delta_time;
        const delta = Vec3.init(desc.f[1], desc.f[2], desc.f[3]).mulScalar(dt);
        switch (@as(StepListenerMode, @enumFromInt(desc.mode))) {
            .linear_velocity => bi.setLinearVelocity(id, bi.getLinearVelocity(id).mulScalar(desc.f[0]).add(delta)),
            .angular_velocity => bi.setAngularVelocity(id, bi.getAngularVelocity(id).mulScalar(desc.f[0]).add(delta)),
            .add_linear_velocity => bi.addLinearVelocity(id, delta),
            .record_only => {},
        }
    }
};

// -------------------------------------------------------------------------------------------------------------------
// The Zolt side

const JobKind = union(enum) {
    single_threaded,
    thread_pool: i32, // number of worker threads
};

/// A job system of either kind, initialized in place
const AnyJobSystem = struct {
    single: JobSystemSingleThreaded = .empty,
    pool: JobSystemThreadPool = .empty,
    is_pool: bool = false,

    fn init(self: *AnyJobSystem, allocator: Allocator, kind: JobKind) !void {
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

/// SimWorld of SimulationReference.cpp: a PhysicsSystem built from a scene description (heap allocated, the system
/// must not move)
const ZoltWorld = struct {
    allocator: Allocator,
    desc: *const SimSceneDesc,
    temp: TempAllocatorImpl,
    jobs: AnyJobSystem = .{},
    broad_phase_layer_interface: TestLayers.BPLayerInterfaceImpl,
    object_vs_broad_phase_layer_filter: TestLayers.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_layer_pair_filter: TestLayers.ObjectLayerPairFilterImpl = .{},
    recorder: Recorder,
    activation_listener: ActivationListenerImpl = undefined,
    contact_listener: ContactListenerImpl = undefined,
    system: PhysicsSystem = .empty,
    shapes: std.ArrayList(RefConst(Shape)) = .empty,
    handles: std.ArrayList(BodyID) = .empty,
    step_listeners: []StepListenerImpl = &.{},
    step_listener_added: []bool = &.{},

    fn create(allocator: Allocator, desc: *const SimSceneDesc, job_kind: JobKind) !*ZoltWorld {
        const self = try allocator.create(ZoltWorld);
        errdefer allocator.destroy(self);
        self.* = .{
            .allocator = allocator,
            .desc = desc,
            .temp = try .init(allocator, 10 * 1024 * 1024),
            .broad_phase_layer_interface = .init(),
            .recorder = .{ .allocator = allocator },
        };
        errdefer self.temp.deinit();
        errdefer self.recorder.deinit();
        try self.jobs.init(allocator, job_kind);
        errdefer self.jobs.deinit();

        try self.system.init(allocator, io, desc.max_bodies, desc.num_body_mutexes, desc.max_body_pairs, desc.max_contact_constraints, &self.broad_phase_layer_interface.base, &self.object_vs_broad_phase_layer_filter.base, &self.object_layer_pair_filter.base, .{});
        errdefer self.system.deinit();
        const settings = toPhysicsSettings(&desc.settings);
        self.system.setPhysicsSettings(&settings);
        self.system.setGravity(load3(&desc.gravity));
        self.activation_listener = .{ .recorder = &self.recorder };
        self.contact_listener = .{ .recorder = &self.recorder };
        self.system.setBodyActivationListener(.init(&self.activation_listener));
        self.system.setContactListener(.init(&self.contact_listener));

        errdefer self.shapes.deinit(allocator);
        errdefer for (self.shapes.items) |*s| s.deinit();
        try self.shapes.ensureTotalCapacity(allocator, desc.num_shapes);
        for (desc.shapes[0..desc.num_shapes]) |*shape|
            self.shapes.appendAssumeCapacity(try self.makeShape(shape));

        self.step_listeners = try allocator.alloc(StepListenerImpl, desc.num_step_listeners);
        errdefer allocator.free(self.step_listeners);
        for (self.step_listeners, 0..) |*l, i|
            l.* = .{ .world = self, .index = @intCast(i) };
        self.step_listener_added = try allocator.alloc(bool, desc.num_step_listeners);
        @memset(self.step_listener_added, false);
        return self;
    }

    fn destroy(self: *ZoltWorld) void {
        const allocator = self.allocator;
        self.system.deinit();
        for (self.shapes.items) |*s| s.deinit();
        self.shapes.deinit(allocator);
        self.handles.deinit(allocator);
        allocator.free(self.step_listeners);
        allocator.free(self.step_listener_added);
        self.jobs.deinit();
        self.recorder.deinit();
        self.temp.deinit();
        allocator.destroy(self);
    }

    fn getHandle(self: *const ZoltWorld, handle: u32) BodyID {
        return if (handle < self.handles.items.len) self.handles.items[handle] else .invalid;
    }

    /// MakeShape of SimulationReference.cpp
    fn makeShape(self: *const ZoltWorld, shape: *const SimShape) !RefConst(Shape) {
        const allocator = self.allocator;
        var compound: ?*zolt.CompoundShapeSettings = null;
        var settings = Ref(ShapeSettings).init(try self.makeShapeSettings(shape, &compound));
        defer settings.deinit();
        if (compound) |c| {
            for (self.desc.children[shape.child..][0..shape.count]) |*child|
                try c.addShapePtr(load3(&child.position), loadQuat(&child.rotation), self.shapes.items[child.shape].get().?, .{ .user_data = child.user_data });
        }
        var result = try settings.get().?.createShape(allocator);
        defer result.deinit();
        if (!result.isValid()) {
            std.debug.print("SimulationParity: shape error: {s}\n", .{result.getError()});
            return error.InvalidShape;
        }
        return RefConst(Shape).init(result.getPtr().?);
    }

    /// The settings of a shape (without reference; a compound gets its children after the caller took a reference)
    fn makeShapeSettings(self: *const ZoltWorld, shape: *const SimShape, out_compound: *?*zolt.CompoundShapeSettings) !*ShapeSettings {
        const allocator = self.allocator;
        const p = &shape.params;
        const floats = self.desc.floats;
        return switch (@as(ShapeKind, @enumFromInt(shape.kind))) {
            .sphere => convex(try zolt.SphereShapeSettings.create(allocator, p[0], .{}), shape),
            .box => convex(try zolt.BoxShapeSettings.create(allocator, load3(p[0..3]), .{ .convex_radius = p[3] }), shape),
            .capsule => convex(try zolt.CapsuleShapeSettings.create(allocator, p[0], p[1], .{}), shape),
            .tapered_capsule => convex(try zolt.TaperedCapsuleShapeSettings.create(allocator, p[0], p[1], p[2], .{}), shape),
            .cylinder => convex(try zolt.CylinderShapeSettings.create(allocator, p[0], p[1], .{ .convex_radius = p[3] }), shape),
            .tapered_cylinder => convex(try zolt.TaperedCylinderShapeSettings.create(allocator, p[0], p[1], p[2], .{ .convex_radius = p[3] }), shape),
            .convex_hull => blk: {
                const points = try allocator.alloc(Vec3, shape.count);
                defer allocator.free(points);
                for (points, 0..) |*point, i|
                    point.* = load3(floats[shape.data + 3 * i ..][0..3]);
                break :blk convex(try zolt.ConvexHullShapeSettings.create(allocator, points, .{ .max_convex_radius = p[3] }), shape);
            },
            .triangle => convex(try zolt.TriangleShapeSettings.create(allocator, load3(p[0..3]), load3(p[3..6]), load3(p[6..9]), .{ .convex_radius = p[9] }), shape),
            .plane => (try zolt.PlaneShapeSettings.create(allocator, zolt.Plane.init(load3(p[0..3]), p[3]), .{ .half_extent = p[4] })).asShapeSettings(),
            .mesh => blk: {
                const triangles = try allocator.alloc(zolt.Triangle, shape.count);
                defer allocator.free(triangles);
                for (triangles, 0..) |*t, i| {
                    const v = floats[shape.data + 9 * i ..][0..9];
                    t.* = .init(load3(v[0..3]), load3(v[3..6]), load3(v[6..9]), .{});
                }
                break :blk (try zolt.MeshShapeSettings.create(allocator, triangles, .{})).asShapeSettings();
            },
            .empty => (try zolt.EmptyShapeSettings.create(allocator, load3(p[0..3]))).asShapeSettings(),
            .rotated_translated => (try zolt.RotatedTranslatedShapeSettings.createPtr(allocator, load3(p[0..3]), loadQuat(&shape.rotation), self.shapes.items[shape.child].get().?)).asShapeSettings(),
            .scaled => (try zolt.ScaledShapeSettings.createPtr(allocator, self.shapes.items[shape.child].get().?, load3(p[0..3]))).asShapeSettings(),
            .offset_center_of_mass => (try zolt.OffsetCenterOfMassShapeSettings.createPtr(allocator, load3(p[0..3]), self.shapes.items[shape.child].get().?)).asShapeSettings(),
            .static_compound => blk: {
                const s = try zolt.StaticCompoundShapeSettings.create(allocator);
                out_compound.* = &s.base;
                break :blk s.asShapeSettings();
            },
            .mutable_compound => blk: {
                const s = try zolt.MutableCompoundShapeSettings.create(allocator);
                out_compound.* = &s.base;
                break :blk s.asShapeSettings();
            },
        };
    }

    /// A convex shape's settings with the density of the scene
    fn convex(settings: anytype, shape: *const SimShape) *ShapeSettings {
        if (shape.density != 0.0)
            settings.base.setDensity(shape.density);
        return settings.asShapeSettings();
    }

    /// MakeBodyCreationSettings of SimulationReference.cpp (the caller deinits the result)
    fn makeBodyCreationSettings(self: *const ZoltWorld, b: *const SimBody) BodyCreationSettings {
        var s: BodyCreationSettings = .initWithShape(self.shapes.items[b.shape].get().?, loadR3(&b.position), loadQuat(&b.rotation), @enumFromInt(b.motion_type), @intCast(b.object_layer));
        s.linear_velocity = load3(&b.linear_velocity);
        s.angular_velocity = load3(&b.angular_velocity);
        s.user_data = b.user_data;
        s.collision_group = .init(null, b.collision_group_id, b.collision_sub_group_id);
        s.allowed_dofs = .fromInt(@intCast(b.allowed_dofs));
        s.allow_dynamic_or_kinematic = b.allow_dynamic_or_kinematic != 0;
        s.is_sensor = b.is_sensor != 0;
        s.collide_kinematic_vs_non_dynamic = b.collide_kinematic_vs_non_dynamic != 0;
        s.use_manifold_reduction = b.use_manifold_reduction != 0;
        s.apply_gyroscopic_force = b.apply_gyroscopic_force != 0;
        s.motion_quality = @enumFromInt(b.motion_quality);
        s.enhanced_internal_edge_removal = b.enhanced_internal_edge_removal != 0;
        s.allow_sleeping = b.allow_sleeping != 0;
        s.friction = b.friction;
        s.restitution = b.restitution;
        s.linear_damping = b.linear_damping;
        s.angular_damping = b.angular_damping;
        s.max_linear_velocity = b.max_linear_velocity;
        s.max_angular_velocity = b.max_angular_velocity;
        s.gravity_factor = b.gravity_factor;
        s.num_velocity_steps_override = b.num_velocity_steps_override;
        s.num_position_steps_override = b.num_position_steps_override;
        s.override_mass_properties = @enumFromInt(b.override_mass_properties);
        s.inertia_multiplier = b.inertia_multiplier;
        s.mass_properties_override.mass = b.mass;
        const m = &b.inertia;
        s.mass_properties_override.inertia = Mat44.init(Vec4.init(m[0], m[1], m[2], m[3]), Vec4.init(m[4], m[5], m[6], m[7]), Vec4.init(m[8], m[9], m[10], m[11]), Vec4.init(m[12], m[13], m[14], m[15]));
        return s;
    }

    fn getHandles(self: *const ZoltWorld, op: *const SimOp) ![]BodyID {
        const ids = try self.allocator.alloc(BodyID, op.count);
        for (ids, 0..) |*id, i|
            id.* = self.getHandle(op.body + @as(u32, @intCast(i)));
        return ids;
    }

    /// ApplyOp of SimulationReference.cpp
    fn applyOp(self: *ZoltWorld, op: *const SimOp) !u32 {
        const result = try self.applyOpInner(op);
        try self.recorder.checkError();
        return result;
    }

    fn applyOpInner(self: *ZoltWorld, op: *const SimOp) !u32 {
        const allocator = self.allocator;
        const bi = self.system.getBodyInterfaceMut();
        const id = self.getHandle(op.body);
        const activation: Activation = if (op.activation == 0) .activate else .dont_activate;
        const f = &op.f;
        const r = loadR3(&op.r);
        const kind: OpKind = @enumFromInt(op.kind);

        switch (kind) {
            .create_body, .create_and_add_body, .create_body_no_add => {
                try self.handles.ensureUnusedCapacity(allocator, 1);
                var settings = self.makeBodyCreationSettings(&self.desc.bodies[op.index]);
                defer settings.deinit();
                var new_id: BodyID = .invalid;
                if (kind == .create_and_add_body) {
                    new_id = try bi.createAndAddBody(&settings, activation);
                } else if (try bi.createBody(&settings)) |body| {
                    new_id = body.getID();
                    if (kind == .create_body)
                        bi.addBody(new_id, activation) catch |err| {
                            bi.destroyBody(new_id);
                            return err;
                        };
                }
                self.handles.appendAssumeCapacity(new_id);
                return new_id.getIndexAndSequenceNumber();
            },

            .add_body => try bi.addBody(id, activation),
            .remove_body => bi.removeBody(id),
            .destroy_body => {
                bi.destroyBody(id);
                return 0;
            },

            .add_bodies => {
                const ids = try self.getHandles(op);
                defer allocator.free(ids);
                const state = try bi.addBodiesPrepare(ids);
                bi.addBodiesFinalize(ids, state, activation);
                return 0;
            },

            .remove_bodies => {
                const ids = try self.getHandles(op);
                defer allocator.free(ids);
                bi.removeBodies(ids);
                return 0;
            },

            .activate_body => bi.activateBody(id),
            .deactivate_body => bi.deactivateBody(id),

            .activate_bodies, .deactivate_bodies => {
                const ids = try self.getHandles(op);
                defer allocator.free(ids);
                if (kind == .activate_bodies) bi.activateBodies(ids) else bi.deactivateBodies(ids);
                return 0;
            },

            .add_force => bi.addForce(id, load3(f[0..3]), .{ .activation = activation }),
            .add_force_at_position => bi.addForceAtPosition(id, load3(f[0..3]), r, .{ .activation = activation }),
            .add_torque => bi.addTorque(id, load3(f[0..3]), .{ .activation = activation }),
            .add_force_and_torque => bi.addForceAndTorque(id, load3(f[0..3]), load3(f[3..6]), .{ .activation = activation }),
            .add_impulse => bi.addImpulse(id, load3(f[0..3])),
            .add_impulse_at_position => bi.addImpulseAtPosition(id, load3(f[0..3]), r),
            .add_angular_impulse => bi.addAngularImpulse(id, load3(f[0..3])),
            .set_linear_velocity => bi.setLinearVelocity(id, load3(f[0..3])),
            .set_angular_velocity => bi.setAngularVelocity(id, load3(f[0..3])),
            .set_linear_and_angular_velocity => bi.setLinearAndAngularVelocity(id, load3(f[0..3]), load3(f[3..6])),
            .add_linear_velocity => bi.addLinearVelocity(id, load3(f[0..3])),
            .add_linear_and_angular_velocity => bi.addLinearAndAngularVelocity(id, load3(f[0..3]), load3(f[3..6])),
            .set_position => bi.setPosition(id, r, activation),
            .set_rotation => bi.setRotation(id, loadQuat(f[0..4]), activation),
            .set_position_and_rotation => bi.setPositionAndRotation(id, r, loadQuat(f[0..4]), activation),
            .set_position_and_rotation_when_changed => bi.setPositionAndRotationWhenChanged(id, r, loadQuat(f[0..4]), activation),
            .set_position_rotation_and_velocity => bi.setPositionRotationAndVelocity(id, r, loadQuat(f[0..4]), load3(f[4..7]), load3(f[7..10])),
            .move_kinematic => bi.moveKinematic(id, r, loadQuat(f[0..4]), f[4]),
            .set_motion_type => bi.setMotionType(id, @enumFromInt(op.index), activation),
            .set_motion_quality => bi.setMotionQuality(id, @enumFromInt(op.index)),
            .set_gravity_factor => bi.setGravityFactor(id, f[0]),
            .set_max_linear_velocity => bi.setMaxLinearVelocity(id, f[0]),
            .set_max_angular_velocity => bi.setMaxAngularVelocity(id, f[0]),
            .reset_sleep_timer => bi.resetSleepTimer(id),
            .set_object_layer => bi.setObjectLayer(id, @intCast(op.index)),
            .set_user_data => bi.setUserData(id, @as(u64, op.index) | (@as(u64, op.count) << 32)),
            .set_friction_and_restitution => {
                bi.setFriction(id, f[0]);
                bi.setRestitution(id, f[1]);
            },

            .set_gravity => {
                self.system.setGravity(load3(f[0..3]));
                return 0;
            },
            .set_physics_settings => {
                const settings = toPhysicsSettings(&self.desc.physics_settings[op.index]);
                self.system.setPhysicsSettings(&settings);
                return 0;
            },
            .add_step_listener => {
                try self.system.addStepListener(.init(&self.step_listeners[op.index]));
                self.step_listener_added[op.index] = true;
                return 0;
            },
            .remove_step_listener => {
                self.system.removeStepListener(.init(&self.step_listeners[op.index]));
                self.step_listener_added[op.index] = false;
                return 0;
            },
            .optimize_broad_phase => {
                self.system.optimizeBroadPhase();
                return 0;
            },
            .set_shape => try bi.setShape(id, self.shapes.items[op.index].get().?, op.count != 0, activation),
            .activate_bodies_in_aabox => {
                try bi.activateBodiesInAABox(&zolt.AABox.init(load3(f[0..3]), load3(f[3..6])), &.{}, &.{});
                return 0;
            },
            .apply_buoyancy_impulse => {
                const applied = bi.applyBuoyancyImpulse(id, r, load3(f[0..3]), f[3], f[4], f[5], load3(f[6..9]), load3(f[9..12]), f[12]);
                return (@as(u32, @intFromBool(applied)) << 2) | @as(u32, @intFromBool(bi.isActive(id))) | (@as(u32, @intFromBool(bi.isAdded(id))) << 1);
            },
            .create_constraint => {
                // Phase 6: one prong per ConstraintKind when the class is ported (the scenes that need it are blocked
                // by SimScenes.Blocker.constraints until then)
                return error.SceneNotSupported;
            },

            .set_update, .snapshot, .restore_cross => unreachable, // Driver operations
        }

        // Body operations return IsActive and IsAdded of the body afterwards
        return @as(u32, @intFromBool(bi.isActive(id))) | (@as(u32, @intFromBool(bi.isAdded(id))) << 1);
    }

    fn update(self: *ZoltWorld, delta_time: f32, collision_steps: u32, temp_allocator: TempAllocator, job_system: JobSystem) !u32 {
        const errors = try self.system.update(delta_time, collision_steps, temp_allocator, job_system);
        return errors.toInt();
    }

    fn saveState(self: *const ZoltWorld, state: StateRecorderState, out: *std.ArrayList(u8)) !void {
        var recorder = StateRecorderImpl.init(self.allocator);
        defer recorder.deinit();
        try self.system.saveState(recorder.recorder(), .{ .state = state });
        if (recorder.isFailed()) return error.OutOfMemory;
        out.clearRetainingCapacity();
        try out.appendSlice(self.allocator, recorder.getData());
    }

    fn restoreState(self: *ZoltWorld, bytes: []const u8) !bool {
        var recorder = StateRecorderImpl.init(self.allocator);
        defer recorder.deinit();
        recorder.writeBytes(bytes);
        if (recorder.isFailed()) return error.OutOfMemory;
        recorder.rewind();
        return self.system.restoreState(recorder.recorder(), .{});
    }

    fn takeRecords(self: *ZoltWorld, out: *std.ArrayList(SimRecord)) !void {
        try self.recorder.checkError();
        self.recorder.mutex.lock(io);
        defer self.recorder.mutex.unlock(io);
        try out.appendSlice(self.allocator, self.recorder.records.items);
        self.recorder.records.clearRetainingCapacity();
    }

    fn discardRecords(self: *ZoltWorld) void {
        self.recorder.records.clearRetainingCapacity();
    }
};

// -------------------------------------------------------------------------------------------------------------------
// The C++ side

const CppWorld = struct {
    handle: *anyopaque,

    fn init(desc: *const SimSceneDesc, num_threads: u32) CppWorld {
        return .{ .handle = jolt.jolt_sim_create(desc, num_threads) };
    }

    fn deinit(self: CppWorld) void {
        jolt.jolt_sim_destroy(self.handle);
    }

    fn applyOp(self: CppWorld, op: *const SimOp) u32 {
        return jolt.jolt_sim_apply_op(self.handle, op);
    }

    fn update(self: CppWorld, delta_time: f32, collision_steps: u32) u32 {
        return jolt.jolt_sim_update(self.handle, delta_time, collision_steps);
    }

    fn saveState(self: CppWorld, allocator: Allocator, state: StateRecorderState, out: *std.ArrayList(u8)) !void {
        out.clearRetainingCapacity();
        while (true) {
            const size = jolt.jolt_sim_save_state(self.handle, state.toInt(), out.items.ptr, @intCast(out.capacity));
            if (size <= out.capacity) {
                out.items.len = size;
                return;
            }
            try out.ensureTotalCapacity(allocator, size);
        }
    }

    fn restoreState(self: CppWorld, bytes: []const u8) bool {
        return jolt.jolt_sim_restore_state(self.handle, bytes.ptr, @intCast(bytes.len)) != 0;
    }

    fn takeRecords(self: CppWorld, allocator: Allocator, out: *std.ArrayList(SimRecord)) !void {
        const n = jolt.jolt_sim_take_records(self.handle, out.items.ptr, 0);
        if (n == 0) return;
        try out.ensureUnusedCapacity(allocator, n);
        const unused = out.unusedCapacitySlice();
        const taken = jolt.jolt_sim_take_records(self.handle, unused.ptr, @intCast(unused.len));
        std.debug.assert(taken <= unused.len);
        out.items.len += taken;
    }

    fn hasMotionProperties(self: CppWorld, body_id: u32) bool {
        return jolt.jolt_sim_body_has_motion_properties(self.handle, body_id) != 0;
    }

    fn constraintStateSize(self: CppWorld, index: u32) u32 {
        var num: u32 = 0;
        return jolt.jolt_sim_constraint_state_size(self.handle, index, &num);
    }
};

// -------------------------------------------------------------------------------------------------------------------
// The state decoder: offsets of PhysicsSystem::SaveState decoded to (section, item, field)

const FieldKind = enum { u8, bool, u16, u32, f32, f64, bytes };

const Field = struct {
    offset: usize,
    size: usize,
    kind: FieldKind,
    section: []const u8,
    /// Index in the section (body, body pair, manifold, constraint) or null
    item: ?usize = null,
    /// The BodyID (or constraint index) of the item
    id: ?u32 = null,
    name: []const u8,
    component: ?u8 = null,
};

/// Walks the bytes of a saved state like PhysicsSystem::RestoreState reads them. `cpp` answers which bodies have
/// motion properties and the sizes of the constraint states (the decoded bytes are the C++ ones).
const StateLayout = struct {
    allocator: Allocator,
    bytes: []const u8,
    cpp: CppWorld,
    fields: std.ArrayList(Field) = .empty,
    pos: usize = 0,
    truncated: bool = false,

    fn deinit(self: *StateLayout) void {
        self.fields.deinit(self.allocator);
    }

    fn add(self: *StateLayout, kind: FieldKind, size: usize, section: []const u8, item: ?usize, id: ?u32, name: []const u8, component: ?u8) !void {
        if (self.truncated or self.pos + size > self.bytes.len) {
            self.truncated = true;
            return;
        }
        try self.fields.append(self.allocator, .{ .offset = self.pos, .size = size, .kind = kind, .section = section, .item = item, .id = id, .name = name, .component = component });
        self.pos += size;
    }

    fn scalar(self: *StateLayout, comptime T: type, section: []const u8, item: ?usize, id: ?u32, name: []const u8) !void {
        const kind: FieldKind = switch (T) {
            u8 => .u8,
            bool => .bool,
            u16 => .u16,
            u32 => .u32,
            f32 => .f32,
            f64 => .f64,
            else => @compileError("unsupported"),
        };
        try self.add(kind, @sizeOf(T), section, item, id, name, null);
    }

    fn vector(self: *StateLayout, comptime T: type, n: u8, section: []const u8, item: ?usize, id: ?u32, name: []const u8) !void {
        for (0..n) |c|
            try self.add(if (T == f64) .f64 else .f32, @sizeOf(T), section, item, id, name, @intCast(c));
    }

    fn read(self: *const StateLayout, comptime T: type) ?T {
        if (self.pos + @sizeOf(T) > self.bytes.len) return null;
        return std.mem.bytesToValue(T, self.bytes[self.pos..][0..@sizeOf(T)]);
    }

    fn decode(self: *StateLayout) !void {
        const state_byte = self.read(u8) orelse return;
        try self.scalar(u8, "header", null, null, "state");
        const state = StateRecorderState.fromInt(state_byte);

        if (state.global) {
            try self.scalar(f32, "global", null, null, "previous_step_delta_time");
            try self.vector(f32, 3, "global", null, null, "gravity");
        }

        if (state.bodies) {
            const num_bodies = self.read(u32) orelse return;
            try self.scalar(u32, "bodies", null, null, "num_bodies");
            for (0..num_bodies) |i| {
                const id = self.read(u32) orelse return;
                try self.scalar(u32, "bodies", i, id, "id");
                try self.scalar(bool, "bodies", i, id, "is_active");
                try self.vector(Real, 3, "bodies", i, id, "position");
                try self.vector(f32, 4, "bodies", i, id, "rotation");
                if (self.cpp.hasMotionProperties(id)) {
                    try self.vector(f32, 3, "bodies", i, id, "motion_properties.linear_velocity");
                    try self.vector(f32, 3, "bodies", i, id, "motion_properties.angular_velocity");
                    try self.vector(f32, 3, "bodies", i, id, "motion_properties.force");
                    try self.vector(f32, 3, "bodies", i, id, "motion_properties.torque");
                    if (Core.double_precision)
                        try self.vector(f64, 3, "bodies", i, id, "motion_properties.sleep_test_offset");
                    for ([_][]const u8{ "motion_properties.sleep_test_spheres[0]", "motion_properties.sleep_test_spheres[1]", "motion_properties.sleep_test_spheres[2]" }) |name| {
                        try self.vector(f32, 3, "bodies", i, id, name); // center
                        try self.add(.f32, 4, "bodies", i, id, name, 3); // radius
                    }
                    try self.scalar(f32, "bodies", i, id, "motion_properties.sleep_test_timer");
                    try self.scalar(bool, "bodies", i, id, "motion_properties.allow_sleeping");
                }
            }
        }

        if (state.contacts) {
            const num_body_pairs = self.read(u32) orelse return;
            try self.scalar(u32, "contacts", null, null, "num_body_pairs");
            var manifold_index: usize = 0;
            for (0..num_body_pairs) |i| {
                const body_a = self.read(u32) orelse return;
                try self.scalar(u32, "contacts.body_pairs", i, body_a, "key.body_a");
                try self.scalar(u32, "contacts.body_pairs", i, body_a, "key.body_b");
                try self.vector(f32, 3, "contacts.body_pairs", i, body_a, "delta_position");
                try self.vector(f32, 3, "contacts.body_pairs", i, body_a, "delta_rotation");
                const num_manifolds = self.read(u32) orelse return;
                try self.scalar(u32, "contacts.body_pairs", i, body_a, "num_manifolds");
                for (0..num_manifolds) |_| {
                    const m = manifold_index;
                    manifold_index += 1;
                    try self.scalar(u32, "contacts.manifolds", m, body_a, "key.body1");
                    try self.scalar(u32, "contacts.manifolds", m, body_a, "key.sub_shape1");
                    try self.scalar(u32, "contacts.manifolds", m, body_a, "key.body2");
                    try self.scalar(u32, "contacts.manifolds", m, body_a, "key.sub_shape2");
                    const num_points = self.read(u16) orelse return;
                    try self.scalar(u16, "contacts.manifolds", m, body_a, "num_contact_points");
                    try self.vector(f32, 3, "contacts.manifolds", m, body_a, "contact_normal");
                    try self.vector(f32, 2, "contacts.manifolds", m, body_a, "friction_lambda");
                    try self.scalar(f32, "contacts.manifolds", m, body_a, "angular_friction_lambda");
                    for (0..num_points) |_| {
                        try self.vector(f32, 3, "contacts.manifolds", m, body_a, "contact_point.position1");
                        try self.vector(f32, 3, "contacts.manifolds", m, body_a, "contact_point.position2");
                        try self.scalar(f32, "contacts.manifolds", m, body_a, "contact_point.non_penetration_lambda");
                    }
                }
            }
            const num_ccd = self.read(u32) orelse return;
            try self.scalar(u32, "contacts", null, null, "num_ccd_manifolds");
            for (0..num_ccd) |i| {
                for ([_][]const u8{ "key.body1", "key.sub_shape1", "key.body2", "key.sub_shape2" }) |name|
                    try self.scalar(u32, "contacts.ccd_manifolds", i, null, name);
            }
        }

        if (state.constraints) {
            const num_constraints = self.read(u32) orelse return;
            try self.scalar(u32, "constraints", null, null, "num_constraints");
            for (0..num_constraints) |i| {
                const index = self.read(u32) orelse return;
                try self.scalar(u32, "constraints", i, index, "constraint_index");
                try self.scalar(bool, "constraints", i, index, "enabled");
                const size = self.cpp.constraintStateSize(@intCast(i));
                if (size > 1)
                    try self.add(.bytes, size - 1, "constraints", i, index, "state of the constraint class", null);
            }
        }
    }

    fn find(self: *const StateLayout, offset: usize) ?*const Field {
        for (self.fields.items) |*field| {
            if (offset >= field.offset and offset < field.offset + field.size)
                return field;
        }
        return null;
    }
};

fn writeValue(w: *std.Io.Writer, field: *const Field, bytes: []const u8) !void {
    if (field.offset + field.size > bytes.len) {
        try w.writeAll("(missing)");
        return;
    }
    const b = bytes[field.offset..][0..field.size];
    switch (field.kind) {
        .u8 => try w.print("{d}", .{b[0]}),
        .bool => try w.print("{d}", .{b[0]}),
        .u16 => try w.print("{d}", .{std.mem.bytesToValue(u16, b[0..2])}),
        .u32 => try w.print("0x{x:0>8}", .{std.mem.bytesToValue(u32, b[0..4])}),
        .f32 => {
            const v = std.mem.bytesToValue(f32, b[0..4]);
            try w.print("0x{x:0>8} ({e})", .{ @as(u32, @bitCast(v)), v });
        },
        .f64 => {
            const v = std.mem.bytesToValue(f64, b[0..8]);
            try w.print("0x{x:0>16} ({e})", .{ @as(u64, @bitCast(v)), v });
        },
        .bytes => try w.print("{x}", .{b}),
    }
}

fn writeField(w: *std.Io.Writer, field: *const Field) !void {
    try w.print("{s}", .{field.section});
    if (field.item) |item| try w.print("[{d}]", .{item});
    if (field.id) |id| try w.print(" (id 0x{x:0>8})", .{id});
    try w.print(" {s}", .{field.name});
    if (field.component) |c| try w.print(".{c}", .{"xyzw"[c]});
}

/// Describe where the Zolt state differs from the Jolt state (null when they are equal)
fn describeStateDifference(allocator: Allocator, zolt_bytes: []const u8, jolt_bytes: []const u8, cpp: CppWorld) !?[]u8 {
    const first = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes) orelse return null;

    var layout: StateLayout = .{ .allocator = allocator, .bytes = jolt_bytes, .cpp = cpp };
    defer layout.deinit();
    try layout.decode();

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const w = &out.writer;
    try w.print("state differs at byte {d} (zolt {d} bytes, jolt {d} bytes)", .{ first, zolt_bytes.len, jolt_bytes.len });
    if (layout.find(first)) |field| {
        try w.writeAll(": ");
        try writeField(w, field);
        try w.writeAll(" zolt ");
        try writeValue(w, field, zolt_bytes);
        try w.writeAll(" jolt ");
        try writeValue(w, field, jolt_bytes);
    } else {
        try w.writeAll(": past the decoded fields");
    }

    // The first differing fields
    var listed: u32 = 0;
    for (layout.fields.items) |*field| {
        if (field.offset < first) continue;
        const end = field.offset + field.size;
        const differs = end > zolt_bytes.len or !std.mem.eql(u8, zolt_bytes[field.offset..end], jolt_bytes[field.offset..end]);
        if (!differs) continue;
        if (listed == 0) try w.writeAll("\n  differing fields (jolt layout):");
        try w.writeAll("\n    ");
        try writeField(w, field);
        try w.writeAll(": zolt ");
        try writeValue(w, field, zolt_bytes);
        try w.writeAll(" jolt ");
        try writeValue(w, field, jolt_bytes);
        listed += 1;
        if (listed == 12) break;
    }
    return try out.toOwnedSlice();
}

// -------------------------------------------------------------------------------------------------------------------
// The driver

const Config = struct {
    zolt: JobKind,
    cpp_threads: u32,
    /// Run the D12 sweeps (every temp allocation and every createJob fails once) in this configuration
    sweeps: bool = false,
};

/// Zolt with JobSystemSingleThreaded and thread pools with 0, 1, 3 and 7 workers, C++ with 0, 3 and 7 workers
const configs = [_]Config{
    .{ .zolt = .single_threaded, .cpp_threads = 0, .sweeps = true },
    .{ .zolt = .{ .thread_pool = 0 }, .cpp_threads = 3 },
    .{ .zolt = .{ .thread_pool = 1 }, .cpp_threads = 0 },
    .{ .zolt = .{ .thread_pool = 3 }, .cpp_threads = 7, .sweeps = true },
    .{ .zolt = .{ .thread_pool = 7 }, .cpp_threads = 3 },
};

const all_states = [_]StateRecorderState{
    .{ .global = true },
    .{ .bodies = true },
    .{ .contacts = true },
    .{ .constraints = true },
    .{ .bodies = true, .contacts = true },
    .all,
};

/// One scene in one configuration, both sides in lockstep
const Run = struct {
    allocator: Allocator,
    entry: *const SceneEntry,
    config: Config,
    builder: *const SceneBuilder,
    desc: *const SimSceneDesc,
    zw: *ZoltWorld,
    cw: CppWorld,
    update: u32 = 0,
    zolt_state: std.ArrayList(u8) = .empty,
    jolt_state: std.ArrayList(u8) = .empty,
    zolt_records: std.ArrayList(SimRecord) = .empty,
    jolt_records: std.ArrayList(SimRecord) = .empty,
    snapshots: [4][2]std.ArrayList(u8) = @splat(.{ .empty, .empty }),
    /// Statistics for the sanity checks of the test
    stats: RunStats = .{},

    fn deinit(self: *Run) void {
        self.zolt_state.deinit(self.allocator);
        self.jolt_state.deinit(self.allocator);
        self.zolt_records.deinit(self.allocator);
        self.jolt_records.deinit(self.allocator);
        for (&self.snapshots) |*s| {
            s[0].deinit(self.allocator);
            s[1].deinit(self.allocator);
        }
    }

    fn fail(self: *const Run, comptime fmt: []const u8, args: anytype) error{ParityMismatch} {
        std.debug.print("SimulationParity: scene {s}, zolt {any}, jolt {d} threads, before / after update {d}: ", .{ self.entry.name, self.config.zolt, self.config.cpp_threads, self.update });
        std.debug.print(fmt ++ "\n", args);
        return error.ParityMismatch;
    }

    /// Compare the saved states of both sides (`what` describes the moment)
    fn compareStates(self: *Run, state: StateRecorderState, what: []const u8) !void {
        try self.zw.saveState(state, &self.zolt_state);
        try self.cw.saveState(self.allocator, state, &self.jolt_state);
        if (try describeStateDifference(self.allocator, self.zolt_state.items, self.jolt_state.items, self.cw)) |message| {
            defer self.allocator.free(message);
            return self.fail("{s}, state {x}: {s}", .{ what, state.toInt(), message });
        }
    }

    /// Compare the listener callbacks since the last comparison as sorted records
    fn compareRecords(self: *Run) !void {
        self.zolt_records.clearRetainingCapacity();
        self.jolt_records.clearRetainingCapacity();
        try self.zw.takeRecords(&self.zolt_records);
        try self.cw.takeRecords(self.allocator, &self.jolt_records);
        try self.compareTakenRecords();
    }

    fn compareTakenRecords(self: *Run) !void {
        std.sort.pdq(SimRecord, self.zolt_records.items, {}, SimRecord.lessThan);
        std.sort.pdq(SimRecord, self.jolt_records.items, {}, SimRecord.lessThan);
        for (self.jolt_records.items) |r|
            self.stats.records[r.kind] += 1;
        const n = @min(self.zolt_records.items.len, self.jolt_records.items.len);
        for (self.zolt_records.items[0..n], self.jolt_records.items[0..n], 0..) |z, j, i| {
            if (!std.meta.eql(z, j))
                return self.fail("listener record {d} of {d} differs:\n  zolt {any}\n  jolt {any}", .{ i, self.jolt_records.items.len, z, j });
        }
        if (self.zolt_records.items.len != self.jolt_records.items.len)
            return self.fail("{d} listener records in zolt, {d} in jolt", .{ self.zolt_records.items.len, self.jolt_records.items.len });
    }

    fn applyOps(self: *Run, op_index: *usize, delta_time: *f32, collision_steps: *u32) !void {
        const ops = self.builder.ops.items;
        while (op_index.* < ops.len and ops[op_index.*].update == self.update) : (op_index.* += 1) {
            const op = &ops[op_index.*];
            switch (@as(OpKind, @enumFromInt(op.kind))) {
                .set_update => {
                    delta_time.* = op.f[0];
                    collision_steps.* = op.count;
                },
                .snapshot => {
                    const slot = &self.snapshots[op.index];
                    try self.zw.saveState(.all, &slot[0]);
                    try self.cw.saveState(self.allocator, .all, &slot[1]);
                },
                .restore_cross => {
                    // Zolt restores the C++ state, C++ the Zolt state
                    const slot = &self.snapshots[op.index];
                    try self.zw.saveState(.all, &self.zolt_state);
                    if (!std.mem.eql(u8, self.zolt_state.items, slot[1].items))
                        self.stats.restores_changing_the_state += 1;
                    if (!try self.zw.restoreState(slot[1].items))
                        return self.fail("zolt could not restore the jolt state of slot {d}", .{op.index});
                    if (!self.cw.restoreState(slot[0].items))
                        return self.fail("jolt could not restore the zolt state of slot {d}", .{op.index});
                    try self.compareStates(.all, "after restore_cross");
                },
                else => {
                    const zolt_result = try self.zw.applyOp(op);
                    const jolt_result = self.cw.applyOp(op);
                    if (zolt_result != jolt_result)
                        return self.fail("operation {d} ({t} on handle {d}) returned {d} in zolt and {d} in jolt", .{ op_index.*, @as(OpKind, @enumFromInt(op.kind)), op.body, zolt_result, jolt_result });
                },
            }
        }
    }

    /// One update on both sides, then the comparisons
    fn step(self: *Run, delta_time: f32, collision_steps: u32) !void {
        const sweep = self.config.sweeps and self.entry.sweep_stride != 0 and self.update % self.entry.sweep_stride == 0;
        if (sweep)
            return self.sweepStep(delta_time, collision_steps);

        const zolt_errors = try self.zw.update(delta_time, collision_steps, self.zw.temp.tempAllocator(), self.zw.jobs.jobSystem());
        if (!self.zw.temp.isEmpty()) return self.fail("the temp allocator is not empty after the update", .{});
        const jolt_errors = self.cw.update(delta_time, collision_steps);
        if (zolt_errors != jolt_errors)
            return self.fail("update errors zolt 0x{x} jolt 0x{x}", .{ zolt_errors, jolt_errors });
        try self.compareStates(.all, "after the update");
        try self.compareRecords();
        self.updateStats();
    }

    fn updateStats(self: *Run) void {
        const system = &self.zw.system;
        const num_active = system.getNumActiveBodies(.rigid_body);
        self.stats.max_active_bodies = @max(self.stats.max_active_bodies, num_active);
        if (system.getBodyStats().num_bodies_dynamic + system.getBodyStats().num_bodies_kinematic > num_active)
            self.stats.updates_with_sleeping_bodies += 1;
        if (system.body_manager.getNumActiveCCDBodies() > 0)
            self.stats.updates_with_ccd_bodies += 1;
    }

    /// The update with the D12 sweeps: every temp allocation and every createJob fails once, the system recovers by
    /// restoring the state saved before the update, the final update gives the C++ result
    fn sweepStep(self: *Run, delta_time: f32, collision_steps: u32) !void {
        const allocator = self.allocator;
        const zw = self.zw;

        // The records of the operations before the update belong to this update
        var zolt_records: std.ArrayList(SimRecord) = .empty;
        defer zolt_records.deinit(allocator);
        try zw.takeRecords(&zolt_records);

        var snapshot: std.ArrayList(u8) = .empty;
        defer snapshot.deinit(allocator);
        try zw.saveState(.all, &snapshot);

        // C++ first: the reference bytes after the update
        const jolt_errors = self.cw.update(delta_time, collision_steps);
        var reference: std.ArrayList(u8) = .empty;
        defer reference.deinit(allocator);
        try self.cw.saveState(allocator, .all, &reference);

        // The update counting the temp allocations is the one that is compared (records included)
        var counting: FailingTempAllocator = .init(zw.temp.tempAllocator(), .{});
        const zolt_errors = try zw.update(delta_time, collision_steps, counting.tempAllocator(), zw.jobs.jobSystem());
        if (zolt_errors != jolt_errors)
            return self.fail("update errors zolt 0x{x} jolt 0x{x}", .{ zolt_errors, jolt_errors });
        try self.compareStates(.all, "after the update");
        try zw.takeRecords(&zolt_records);
        const step_calls = try allocator.alloc(u32, zw.step_listeners.len);
        defer allocator.free(step_calls);
        for (zw.step_listeners, step_calls) |*l, *c| c.* = l.num_calls;

        // Every temp allocation fails once
        const num_allocations = counting.num_allocations;
        for (0..num_allocations) |fail_index| {
            try self.restoreZolt(snapshot.items);
            var failing: FailingTempAllocator = .init(zw.temp.tempAllocator(), .{ .fail_index = @intCast(fail_index) });
            if (zw.update(delta_time, collision_steps, failing.tempAllocator(), zw.jobs.jobSystem())) |_| {
                return self.fail("temp allocation {d} of {d} failed but the update succeeded", .{ fail_index, num_allocations });
            } else |err| if (err != error.OutOfMemory) return err;
            if (!failing.has_induced_failure) return self.fail("temp allocation {d} did not fail", .{fail_index});
            if (!zw.temp.isEmpty()) return self.fail("temp allocation {d} failed: the temp allocator is not empty", .{fail_index});
            if (fail_index < 2) // The steps and the body pairs are allocated before any job runs
                try self.expectZoltState(snapshot.items, "temp allocation failed while building the jobs");
            try self.recoverZolt(snapshot.items, reference.items, delta_time, collision_steps, "after temp allocation failure");
            self.stats.num_temp_failures += 1;
        }

        // Every createJob fails once: count the jobs first (a normal update)
        try self.restoreZolt(snapshot.items);
        var counting_jobs: FailingJobSystem = .init(zw.jobs.jobSystem(), .{});
        _ = try zw.update(delta_time, collision_steps, zw.temp.tempAllocator(), counting_jobs.jobSystem());
        try self.expectZoltState(reference.items, "after the update counting the jobs");
        const num_jobs = counting_jobs.num_created.load(.monotonic);
        for (0..num_jobs + 2) |fail_index| {
            try self.restoreZolt(snapshot.items);
            var failing: FailingJobSystem = .init(zw.jobs.jobSystem(), .{ .fail_index = @intCast(fail_index) });
            if (zw.update(delta_time, collision_steps, zw.temp.tempAllocator(), failing.jobSystem())) |_| {
                // A job that could not be spawned from inside a job changes no result
                try self.expectZoltState(reference.items, "a job could not be spawned");
            } else |err| {
                if (err != error.OutOfMemory) return err;
                // A failure while building the jobs changes nothing
                try self.expectZoltState(snapshot.items, "createJob failed while building the jobs");
                try self.recoverZolt(snapshot.items, reference.items, delta_time, collision_steps, "after a createJob failure");
            }
            if (!zw.temp.isEmpty()) return self.fail("createJob {d} failed: the temp allocator is not empty", .{fail_index});
            if (failing.has_induced_failure.load(.monotonic)) self.stats.num_job_failures += 1;
        }

        // The Zolt side is at the reference state again: drop the records of the sweeps, restore the listener counters
        zw.discardRecords();
        for (zw.step_listeners, step_calls) |*l, c| l.num_calls = c;
        try self.expectZoltState(reference.items, "after the sweeps");

        self.zolt_records.clearRetainingCapacity();
        self.jolt_records.clearRetainingCapacity();
        try self.zolt_records.appendSlice(allocator, zolt_records.items);
        try self.cw.takeRecords(allocator, &self.jolt_records);
        try self.compareTakenRecords();
        self.updateStats();
    }

    fn restoreZolt(self: *Run, snapshot: []const u8) !void {
        if (!try self.zw.restoreState(snapshot))
            return self.fail("zolt could not restore its snapshot", .{});
    }

    fn expectZoltState(self: *Run, expected: []const u8, what: []const u8) !void {
        try self.zw.saveState(.all, &self.zolt_state);
        if (try describeStateDifference(self.allocator, self.zolt_state.items, expected, self.cw)) |message| {
            defer self.allocator.free(message);
            return self.fail("{s}: {s}", .{ what, message });
        }
    }

    /// Restore the snapshot and redo the update with enough memory: the C++ bytes
    fn recoverZolt(self: *Run, snapshot: []const u8, reference: []const u8, delta_time: f32, collision_steps: u32, what: []const u8) !void {
        try self.restoreZolt(snapshot);
        _ = try self.zw.update(delta_time, collision_steps, self.zw.temp.tempAllocator(), self.zw.jobs.jobSystem());
        if (!self.zw.temp.isEmpty()) return self.fail("{s}: the temp allocator is not empty", .{what});
        try self.expectZoltState(reference, what);
    }
};

/// What a scene did (the sanity checks of runEntry, printed with `verbose`)
const RunStats = struct {
    /// Number of listener records per RecordKind
    records: [@typeInfo(RecordKind).@"enum".fields.len]u32 = @splat(0),
    num_temp_failures: u32 = 0,
    num_job_failures: u32 = 0,
    /// Largest number of active bodies after an update
    max_active_bodies: u32 = 0,
    /// Number of updates that ended with sleeping bodies (bodies in the broad phase that are not active)
    updates_with_sleeping_bodies: u32 = 0,
    /// Number of updates with active linear cast bodies
    updates_with_ccd_bodies: u32 = 0,
    /// Number of restore_cross operations that changed the state
    restores_changing_the_state: u32 = 0,
};

/// Print what every scene did (a passing test must not print: keep false)
const verbose = false;

/// Run a scene in one configuration
fn runScene(allocator: Allocator, entry: *const SceneEntry, builder: *const SceneBuilder, config: Config) !RunStats {
    const desc = builder.desc();
    const zw = try ZoltWorld.create(allocator, &desc, config.zolt);
    defer zw.destroy();
    const cw = CppWorld.init(&desc, config.cpp_threads);
    defer cw.deinit();

    var run: Run = .{ .allocator = allocator, .entry = entry, .config = config, .builder = builder, .desc = &desc, .zw = zw, .cw = cw };
    defer run.deinit();

    // The new system
    try run.compareStates(.all, "after creating the system");

    var delta_time = builder.delta_time;
    var collision_steps = builder.collision_steps;
    var op_index: usize = 0;
    while (run.update < builder.num_updates) : (run.update += 1) {
        try run.applyOps(&op_index, &delta_time, &collision_steps);
        try run.step(delta_time, collision_steps);
    }
    std.debug.assert(op_index == builder.ops.items.len);

    // Every part of the state
    for (all_states) |state|
        try run.compareStates(state, "at the end");

    return run.stats;
}

/// Creating the scene (the system, the shapes, the operations before the first update) with every allocation failing
/// once (D12)
fn createScene(allocator: Allocator, builder: *const SceneBuilder) !void {
    const desc = builder.desc();
    const zw = try ZoltWorld.create(allocator, &desc, .single_threaded);
    defer zw.destroy();
    for (builder.ops.items) |*op| {
        if (op.update != 0) break;
        switch (@as(OpKind, @enumFromInt(op.kind))) {
            .set_update, .snapshot, .restore_cross => {},
            else => _ = try zw.applyOp(op),
        }
    }
}

fn runEntry(entry: *const SceneEntry) !void {
    if (!entry.blocked_by.isSupported())
        return error.SkipZigTest;

    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    try entry.build(&builder);

    var stats: [configs.len]RunStats = undefined;
    for (configs, &stats) |config, *s|
        s.* = try runScene(allocator, entry, &builder, config);

    if (verbose)
        std.debug.print("{s}: {any}\n", .{ entry.name, stats[0] });

    // The listener records do not depend on the threads (their number neither); the sweeps ran
    for (stats[1..]) |s|
        try std.testing.expectEqualSlices(u32, &stats[0].records, &s.records);
    if (entry.sweep_stride != 0) {
        try std.testing.expect(stats[0].num_job_failures > 0);
        try std.testing.expect(stats[3].num_job_failures > 0);
    }

    try std.testing.checkAllAllocationFailures(allocator, createScene, .{&builder});
}

/// One test per scene: `SceneTest("name", index)` (a blocked scene is skipped)
fn SceneTest(comptime name: []const u8, comptime index: usize) type {
    _ = name;
    return struct {
        test "SimulationParity scene" {
            try runEntry(&SimScenes.scenes[index]);
        }
    };
}

comptime {
    for (SimScenes.scenes, 0..) |entry, i|
        _ = SceneTest(entry.name, i);
}

test {
    _ = SimScenes;
}

// The scenes of the later waves run on the C++ side already, with 0 and 3 worker threads in lockstep: their
// descriptions build, Jolt is deterministic on them, and they reach what they are written for (contact callbacks,
// constraints in the state). When a wave lands only the Zolt side is new.
test "SimScenes blocked scenes run on the C++ side" {
    const allocator = std.testing.allocator;
    var a_state: std.ArrayList(u8) = .empty;
    defer a_state.deinit(allocator);
    var b_state: std.ArrayList(u8) = .empty;
    defer b_state.deinit(allocator);
    var a_records: std.ArrayList(SimRecord) = .empty;
    defer a_records.deinit(allocator);
    var b_records: std.ArrayList(SimRecord) = .empty;
    defer b_records.deinit(allocator);

    for (&SimScenes.scenes) |*entry| {
        if (entry.blocked_by == .none or entry.blocked_by == .release_only) continue;
        var builder: SceneBuilder = .init(allocator);
        defer builder.deinit();
        try entry.build(&builder);
        const desc = builder.desc();
        const a = CppWorld.init(&desc, 0);
        defer a.deinit();
        const b = CppWorld.init(&desc, 3);
        defer b.deinit();

        var delta_time = builder.delta_time;
        var collision_steps = builder.collision_steps;
        var records: [@typeInfo(RecordKind).@"enum".fields.len]u32 = @splat(0);
        var op_index: usize = 0;
        for (0..builder.num_updates) |update| {
            while (op_index < builder.ops.items.len and builder.ops.items[op_index].update == update) : (op_index += 1) {
                const op = &builder.ops.items[op_index];
                switch (@as(OpKind, @enumFromInt(op.kind))) {
                    .set_update => {
                        delta_time = op.f[0];
                        collision_steps = op.count;
                    },
                    .snapshot, .restore_cross => return error.SkipZigTest, // Not used by the blocked scenes yet
                    else => try std.testing.expectEqual(a.applyOp(op), b.applyOp(op)),
                }
            }
            try std.testing.expectEqual(a.update(delta_time, collision_steps), b.update(delta_time, collision_steps));
            try a.saveState(allocator, .all, &a_state);
            try b.saveState(allocator, .all, &b_state);
            try std.testing.expectEqualSlices(u8, a_state.items, b_state.items);
            a_records.clearRetainingCapacity();
            b_records.clearRetainingCapacity();
            try a.takeRecords(allocator, &a_records);
            try b.takeRecords(allocator, &b_records);
            std.sort.pdq(SimRecord, a_records.items, {}, SimRecord.lessThan);
            std.sort.pdq(SimRecord, b_records.items, {}, SimRecord.lessThan);
            try std.testing.expectEqualSlices(u8, std.mem.sliceAsBytes(a_records.items), std.mem.sliceAsBytes(b_records.items));
            for (a_records.items) |r| records[r.kind] += 1;
        }

        // The state decodes completely
        var layout: StateLayout = .{ .allocator = allocator, .bytes = a_state.items, .cpp = a };
        defer layout.deinit();
        try layout.decode();
        try std.testing.expect(!layout.truncated and layout.pos == a_state.items.len);

        switch (entry.blocked_by) {
            .contacts, .ccd_hits => try std.testing.expect(records[@intFromEnum(RecordKind.contact_added)] > 0),
            .constraints => {
                var num_constraints: u32 = 0;
                _ = jolt.jolt_sim_constraint_state_size(a.handle, 0, &num_constraints);
                try std.testing.expect(num_constraints > 0);
            },
            .none, .soft_bodies, .release_only => {},
        }
    }
}

test "SimScenes struct layout matches SimulationReference.cpp" {
    var sizes: [12]u32 = undefined;
    jolt.jolt_sim_struct_sizes(&sizes);
    const expected = [12]u32{
        @sizeOf(SimShape),
        @sizeOf(SimCompoundChild),
        @sizeOf(SimBody),
        @sizeOf(SimPhysicsSettings),
        @sizeOf(SimStepListener),
        @sizeOf(SimConstraint),
        @sizeOf(SimOp),
        @sizeOf(SimSceneDesc),
        @sizeOf(SimRecord),
        @offsetOf(SimBody, "shape"),
        @offsetOf(SimSceneDesc, "shapes"),
        @offsetOf(SimOp, "r"),
    };
    try std.testing.expectEqualSlices(u32, &expected, &sizes);
}

test "SimScenes default settings are Jolt's PhysicsSettings" {
    var jolt_settings: SimPhysicsSettings = undefined;
    jolt.jolt_sim_default_settings(&jolt_settings);
    const zolt_default: PhysicsSettings = .{};
    const zolt_settings = fromPhysicsSettings(&zolt_default);
    const scene_default: SimPhysicsSettings = .{};
    try std.testing.expectEqualSlices(u8, std.mem.asBytes(&jolt_settings), std.mem.asBytes(&zolt_settings));
    try std.testing.expectEqualSlices(u8, std.mem.asBytes(&jolt_settings), std.mem.asBytes(&scene_default));
}

test "SimulationParity state decoder" {
    // A small scene: decode the C++ state and check that every byte belongs to a field, then change one byte of the
    // Zolt copy in the linear velocity of the second body and check the report
    const allocator = std.testing.allocator;
    var builder: SceneBuilder = .init(allocator);
    defer builder.deinit();
    const sphere = try builder.sphere(0.5);
    _ = try builder.body(.{ .shape = sphere, .motion_type = 0 }, .dont_activate);
    _ = try builder.body(.{ .shape = sphere, .linear_velocity = .{ 1, 2, 3 } }, .activate);
    const desc = builder.desc();
    const cw = CppWorld.init(&desc, 0);
    defer cw.deinit();
    for (builder.ops.items) |*op| _ = cw.applyOp(op);
    _ = cw.update(1.0 / 60.0, 1);

    var bytes: std.ArrayList(u8) = .empty;
    defer bytes.deinit(allocator);
    try cw.saveState(allocator, .all, &bytes);

    var layout: StateLayout = .{ .allocator = allocator, .bytes = bytes.items, .cpp = cw };
    defer layout.deinit();
    try layout.decode();
    try std.testing.expect(!layout.truncated);
    try std.testing.expectEqual(bytes.items.len, layout.pos);

    const copy = try allocator.dupe(u8, bytes.items);
    defer allocator.free(copy);
    var target: ?*const Field = null;
    for (layout.fields.items) |*field| {
        if (field.item == 1 and std.mem.eql(u8, field.name, "motion_properties.linear_velocity") and field.component == 1)
            target = field;
    }
    copy[target.?.offset] ^= 1;
    const message = (try describeStateDifference(allocator, copy, bytes.items, cw)).?;
    defer allocator.free(message);
    try std.testing.expect(std.mem.indexOf(u8, message, "bodies[1]") != null);
    try std.testing.expect(std.mem.indexOf(u8, message, "motion_properties.linear_velocity.y") != null);
    try std.testing.expect(try describeStateDifference(allocator, bytes.items, bytes.items, cw) == null);
}
