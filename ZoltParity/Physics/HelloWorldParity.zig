//! HelloWorld parity (milestone M5, SimulationArchitecture section 3): the scene of HelloWorld/HelloWorld.cpp in Zolt and
//! in the C++ PhysicsSystem (HelloWorldReference.cpp), stepped in lockstep by HelloWorld's loop (while the sphere is
//! active). Compared:
//! - before every step: IsActive(sphere) (the loop condition), the bits of the position and the velocity of the sphere
//!   that HelloWorld prints, and the printed line itself ("Step 1: Position = (0, 2, 0), Velocity = (0, -5, 0)": Zolt's
//!   C++ compatible float formatter against a std::stringstream that streams the same expression as HelloWorld);
//! - after every step: the EPhysicsUpdateError bits, the PhysicsSystem::SaveState bytes (a mismatch is decoded to the
//!   first differing field by SimulationParity's decoder) and the listener callbacks as sorted records (the layout of
//!   SimScenes.SimRecord; HelloWorld's listeners print one line per callback, these record its arguments);
//! - the callbacks of the creation (the activation of the sphere), the body IDs and the number of steps until the sphere
//!   falls asleep (`hello_world_steps`, the loop of both programs ends after it).
//!
//! Both sides build the scene like HelloWorld: its layer classes (copied below and in HelloWorldReference.cpp), its
//! constants (1024 bodies, 0 body mutexes, 1024 body pairs, 1024 contact constraints, a 10 MB TempAllocatorImpl), its
//! bodies, OptimizeBroadPhase, 1 / 60 s with 1 collision step. Thread matrix: Zolt with JobSystemSingleThreaded and
//! JobSystemThreadPool with 0, 1, 3 and 7 workers and with HelloWorld's hardware_concurrency() - 1, C++ with 0, 3 and
//! hardware_concurrency() - 1 workers.
//!
//! The SimScenes entry `contacts_hello_world` runs the same bodies through SimulationParity (with the D12 allocation and
//! job failure sweeps), and `zig build hello-world-check` (part of `zig build parity`) compares the stdout of the C++
//! HelloWorld with its port ZoltHelloWorld/HelloWorld.zig.
//!
//! Proof that it bites: multiplying the Baumgarte lambda of ContactConstraintPart.solvePositionConstraint by 1.0001 is
//! reported as "after step 16: state differs at byte 63 (zolt 306 bytes, jolt 306 bytes): bodies[1] (id 0x00800001)
//! position.y zolt 0x3ee64ec7 (4.4981977e-1) jolt 0x3ee64e94 (4.4981825e-1)" (hello-world-check reports "Step 17:
//! Position = (0, 0.44982, 0)" against the C++ "(0, 0.449818, 0)"); computing the combined friction as
//! sqrt(f1) * sqrt(f2) instead of sqrt(f1 * f2) is reported as a differing OnContactAdded record in step 16 (data[5]
//! 1045220556 against 1045220557), which the text of HelloWorld does not show.

const std = @import("std");
const zolt = @import("zolt");
const SimScenes = @import("SimScenes.zig");
const SimulationParity = @import("SimulationParity.zig");

const Allocator = std.mem.Allocator;
const Core = zolt.Core;
const Real = zolt.Real;
const Vec3 = zolt.Vec3;
const RVec3 = zolt.RVec3;
const Quat = zolt.Quat;
const Mutex = zolt.Mutex;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const BodyCreationSettings = zolt.BodyCreationSettings;
const BoxShapeSettings = zolt.BoxShapeSettings;
const SphereShape = zolt.SphereShape;
const ObjectLayer = zolt.ObjectLayer;
const ObjectLayerPairFilter = zolt.ObjectLayerPairFilter;
const BroadPhaseLayer = zolt.BroadPhaseLayer;
const BroadPhaseLayerInterface = zolt.BroadPhaseLayerInterface;
const ObjectVsBroadPhaseLayerFilter = zolt.ObjectVsBroadPhaseLayerFilter;
const ContactManifold = zolt.ContactManifold;
const ContactSettings = zolt.ContactSettings;
const CollideShapeResult = zolt.CollideShapeResult;
const SubShapeIDPair = zolt.SubShapeIDPair;
const ValidateResult = zolt.ValidateResult;
const PhysicsSystem = zolt.PhysicsSystem;
const StateRecorderImpl = zolt.StateRecorderImpl;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const JobSystem = zolt.JobSystem;
const JobSystemSingleThreaded = zolt.JobSystemSingleThreaded;
const JobSystemThreadPool = zolt.JobSystemThreadPool;
const physics_settings = zolt.physics_settings;
const cppFloat = zolt.cppFloat;
const SimRecord = SimScenes.SimRecord;
const RecordKind = SimScenes.RecordKind;

const io = std.testing.io;

/// The number of steps of HelloWorld: the sphere falls asleep in the update of the last step, both programs print this
/// many "Step ..." lines
const hello_world_steps = SimScenes.hello_world_steps;

/// The C++ reference functions, see HelloWorldReference.cpp
const jolt = struct {
    extern fn jolt_hello_world_create(num_threads: c_int) *anyopaque;
    extern fn jolt_hello_world_destroy(world: *anyopaque) void;
    extern fn jolt_hello_world_is_active(world: *anyopaque) u32;
    extern fn jolt_hello_world_body_ids(world: *anyopaque, out_floor_id: *u32, out_sphere_id: *u32) void;
    extern fn jolt_hello_world_sphere_state(world: *anyopaque, out_position: *[3]Real, out_velocity: *[3]f32) void;
    extern fn jolt_hello_world_step_line(world: *anyopaque, step: u32, out_chars: [*]u8, capacity: u32) u32;
    extern fn jolt_hello_world_update(world: *anyopaque) u32;
    extern fn jolt_hello_world_save_state(world: *anyopaque, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_hello_world_take_records(world: *anyopaque, out_records: [*]SimRecord, capacity: u32) u32;
    extern fn jolt_hello_world_body_has_motion_properties(world: *anyopaque, body_id: u32) u32;
};

// -------------------------------------------------------------------------------------------------------------------
// HelloWorld's layers (HelloWorld.cpp, ZoltHelloWorld/HelloWorld.zig)

const Layers = struct {
    const non_moving: ObjectLayer = 0;
    const moving: ObjectLayer = 1;
    const num_layers: ObjectLayer = 2;
};

/// Class that determines if two object layers can collide
const ObjectLayerPairFilterImpl = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectLayerPairFilter = .init(ObjectLayerPairFilterImpl),

    pub fn shouldCollide(self: *const ObjectLayerPairFilterImpl, object1: ObjectLayer, object2: ObjectLayer) bool {
        _ = self;
        switch (object1) {
            Layers.non_moving => return object2 == Layers.moving, // Non moving only collides with moving
            Layers.moving => return true, // Moving collides with everything
            else => {
                if (Core.enable_asserts) @panic("invalid object layer"); // JPH_ASSERT(false)
                return false;
            },
        }
    }
};

const BroadPhaseLayers = struct {
    const non_moving: BroadPhaseLayer = .init(0);
    const moving: BroadPhaseLayer = .init(1);
    const num_layers: u32 = 2;
};

/// BroadPhaseLayerInterface implementation
const BPLayerInterfaceImpl = struct {
    pub const overrides = .{ .getNumBroadPhaseLayers, .getBroadPhaseLayer };

    base: BroadPhaseLayerInterface = .init(BPLayerInterfaceImpl),
    object_to_broad_phase: [Layers.num_layers]BroadPhaseLayer,

    fn init() BPLayerInterfaceImpl {
        var self: BPLayerInterfaceImpl = .{ .object_to_broad_phase = undefined };
        self.object_to_broad_phase[Layers.non_moving] = BroadPhaseLayers.non_moving;
        self.object_to_broad_phase[Layers.moving] = BroadPhaseLayers.moving;
        return self;
    }

    pub fn getNumBroadPhaseLayers(self: *const BPLayerInterfaceImpl) u32 {
        _ = self;
        return BroadPhaseLayers.num_layers;
    }

    pub fn getBroadPhaseLayer(self: *const BPLayerInterfaceImpl, layer: ObjectLayer) BroadPhaseLayer {
        std.debug.assert(layer < Layers.num_layers);
        return self.object_to_broad_phase[layer];
    }
};

/// Class that determines if an object layer can collide with a broadphase layer
const ObjectVsBroadPhaseLayerFilterImpl = struct {
    pub const overrides = .{.shouldCollide};

    base: ObjectVsBroadPhaseLayerFilter = .init(ObjectVsBroadPhaseLayerFilterImpl),

    pub fn shouldCollide(self: *const ObjectVsBroadPhaseLayerFilterImpl, layer1: ObjectLayer, layer2: BroadPhaseLayer) bool {
        _ = self;
        switch (layer1) {
            Layers.non_moving => return layer2.eql(BroadPhaseLayers.moving),
            Layers.moving => return true,
            else => {
                if (Core.enable_asserts) @panic("invalid object layer"); // JPH_ASSERT(false)
                return false;
            },
        }
    }
};

// -------------------------------------------------------------------------------------------------------------------
// HelloWorld's listeners, recording their arguments (MyContactListener / MyBodyActivationListener of
// HelloWorldReference.cpp)

fn floatBits(v: f32) u32 {
    return @bitCast(v);
}

/// The callback records (the listeners run on the job system's threads; a failed append is reported by `checkError`)
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

/// The hash of the base offset and the contact points of a manifold (HashManifold of HelloWorldReference.cpp)
fn hashManifold(manifold: *const ContactManifold) u32 {
    var hash = SimScenes.hash_seed;
    const offset = [3]Real{ manifold.base_offset.getX(), manifold.base_offset.getY(), manifold.base_offset.getZ() };
    hash = SimScenes.hashBytes(hash, std.mem.sliceAsBytes(&offset));
    for ([_]*const zolt.ContactPoints{ &manifold.relative_contact_points_on1, &manifold.relative_contact_points_on2 }) |points| {
        for (points.constSlice()) |p| {
            const v = [3]f32{ p.getX(), p.getY(), p.getZ() };
            hash = SimScenes.hashBytes(hash, std.mem.sliceAsBytes(&v));
        }
    }
    return hash;
}

const MyContactListener = struct {
    recorder: *Recorder,

    pub fn onContactValidate(self: *MyContactListener, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult {
        _ = base_offset;
        var r: SimRecord = .{ .kind = @intFromEnum(RecordKind.contact_validate) };
        r.ids = .{ body1.getID().getIndexAndSequenceNumber(), body2.getID().getIndexAndSequenceNumber(), collision_result.sub_shape_id1.getValue(), collision_result.sub_shape_id2.getValue() };
        r.data[0] = floatBits(collision_result.penetration_depth);
        r.data[1] = floatBits(collision_result.penetration_axis.getX());
        r.data[2] = floatBits(collision_result.penetration_axis.getY());
        r.data[3] = floatBits(collision_result.penetration_axis.getZ());
        self.recorder.add(r);

        // Allows you to ignore a contact before it is created (using layers to not make objects collide is cheaper!)
        return .accept_all_contacts_for_this_body_pair;
    }

    pub fn onContactAdded(self: *MyContactListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        self.record(.contact_added, body1, body2, manifold, settings);
    }

    pub fn onContactPersisted(self: *MyContactListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        self.record(.contact_persisted, body1, body2, manifold, settings);
    }

    pub fn onContactRemoved(self: *MyContactListener, sub_shape_pair: *const SubShapeIDPair) void {
        var r: SimRecord = .{ .kind = @intFromEnum(RecordKind.contact_removed) };
        r.ids = .{ sub_shape_pair.body1_id.getIndexAndSequenceNumber(), sub_shape_pair.sub_shape_id1.getValue(), sub_shape_pair.body2_id.getIndexAndSequenceNumber(), sub_shape_pair.sub_shape_id2.getValue() };
        self.recorder.add(r);
    }

    fn record(self: *MyContactListener, kind: RecordKind, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *const ContactSettings) void {
        var r: SimRecord = .{ .kind = @intFromEnum(kind) };
        r.ids = .{ body1.getID().getIndexAndSequenceNumber(), body2.getID().getIndexAndSequenceNumber(), manifold.sub_shape_id1.getValue(), manifold.sub_shape_id2.getValue() };
        r.data = .{
            floatBits(manifold.world_space_normal.getX()),
            floatBits(manifold.world_space_normal.getY()),
            floatBits(manifold.world_space_normal.getZ()),
            floatBits(manifold.penetration_depth),
            manifold.relative_contact_points_on1.len | (@as(u32, @intFromBool(settings.is_sensor)) << 8),
            floatBits(settings.combined_friction),
            floatBits(settings.combined_restitution),
            hashManifold(manifold),
        };
        self.recorder.add(r);
    }
};

const MyBodyActivationListener = struct {
    recorder: *Recorder,

    pub fn onBodyActivated(self: *MyBodyActivationListener, body_id: BodyID, body_user_data: u64) void {
        self.record(.body_activated, body_id, body_user_data);
    }

    pub fn onBodyDeactivated(self: *MyBodyActivationListener, body_id: BodyID, body_user_data: u64) void {
        self.record(.body_deactivated, body_id, body_user_data);
    }

    fn record(self: *MyBodyActivationListener, kind: RecordKind, body_id: BodyID, body_user_data: u64) void {
        var r: SimRecord = .{ .kind = @intFromEnum(kind) };
        r.ids[0] = body_id.getIndexAndSequenceNumber();
        r.data[0] = @truncate(body_user_data);
        r.data[1] = @truncate(body_user_data >> 32);
        self.recorder.add(r);
    }
};

// -------------------------------------------------------------------------------------------------------------------
// The two worlds

const JobKind = union(enum) {
    single_threaded,
    /// Number of worker threads (-1: hardware_concurrency() - 1, HelloWorld's)
    thread_pool: i32,
};

/// The objects of HelloWorld's main() in Zolt (heap allocated: the system and the listeners must not move)
const ZoltHelloWorld = struct {
    allocator: Allocator,
    temp_allocator: TempAllocatorImpl,
    single_threaded: JobSystemSingleThreaded = .empty,
    thread_pool: JobSystemThreadPool = .empty,
    job_kind: JobKind,
    broad_phase_layer_interface: BPLayerInterfaceImpl = .init(),
    object_vs_broadphase_layer_filter: ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_vs_object_layer_filter: ObjectLayerPairFilterImpl = .{},
    physics_system: PhysicsSystem = .empty,
    recorder: Recorder,
    body_activation_listener: MyBodyActivationListener = undefined,
    contact_listener: MyContactListener = undefined,
    floor_id: BodyID = .invalid,
    sphere_id: BodyID = .invalid,

    /// We simulate the physics world in discrete time steps. 60 Hz is a good rate to update the physics system.
    const delta_time: f32 = 1.0 / 60.0;

    fn create(allocator: Allocator, job_kind: JobKind) !*ZoltHelloWorld {
        const self = try allocator.create(ZoltHelloWorld);
        errdefer allocator.destroy(self);
        self.* = .{
            .allocator = allocator,
            .temp_allocator = try .init(allocator, 10 * 1024 * 1024),
            .job_kind = job_kind,
            .recorder = .{ .allocator = allocator },
        };
        errdefer self.temp_allocator.deinit();
        errdefer self.recorder.deinit();
        switch (job_kind) {
            .single_threaded => self.single_threaded = try .init(allocator, io, physics_settings.max_physics_jobs),
            .thread_pool => |n| try self.thread_pool.init(allocator, io, physics_settings.max_physics_jobs, physics_settings.max_physics_barriers, .{ .num_threads = n }),
        }
        errdefer switch (job_kind) {
            .single_threaded => self.single_threaded.deinit(),
            .thread_pool => self.thread_pool.deinit(),
        };

        // HelloWorld's constants
        const max_bodies: u32 = 1024;
        const num_body_mutexes: u32 = 0;
        const max_body_pairs: u32 = 1024;
        const max_contact_constraints: u32 = 1024;
        try self.physics_system.init(allocator, io, max_bodies, num_body_mutexes, max_body_pairs, max_contact_constraints, &self.broad_phase_layer_interface.base, &self.object_vs_broadphase_layer_filter.base, &self.object_vs_object_layer_filter.base, .{});
        errdefer self.physics_system.deinit();

        // The listeners
        self.body_activation_listener = .{ .recorder = &self.recorder };
        self.physics_system.setBodyActivationListener(.init(&self.body_activation_listener));
        self.contact_listener = .{ .recorder = &self.recorder };
        self.physics_system.setContactListener(.init(&self.contact_listener));

        const body_interface = self.physics_system.getBodyInterfaceMut();

        // Next we can create a rigid body to serve as the floor, we make a large box
        var floor_shape_settings: BoxShapeSettings = .init(allocator, Vec3.init(100.0, 1.0, 100.0), .{});
        floor_shape_settings.asShapeSettings().setEmbedded();
        defer floor_shape_settings.deinit();
        var floor_shape_result = try floor_shape_settings.createShape(allocator);
        defer floor_shape_result.deinit();
        var floor_shape = floor_shape_result.get().clone();
        defer floor_shape.deinit();
        var floor_settings: BodyCreationSettings = .initWithShape(floor_shape.get(), RVec3.init(0.0, -1.0, 0.0), Quat.identity(), .static, Layers.non_moving);
        defer floor_settings.deinit();
        const floor = (try body_interface.createBody(&floor_settings)) orelse return error.OutOfBodies;
        try body_interface.addBody(floor.getID(), .dont_activate);
        self.floor_id = floor.getID();

        // Now create a dynamic body to bounce on the floor
        var sphere_settings: BodyCreationSettings = .initWithShape((try SphereShape.create(allocator, 0.5, .{})).asShape(), RVec3.init(0.0, 2.0, 0.0), Quat.identity(), .dynamic, Layers.moving);
        defer sphere_settings.deinit();
        self.sphere_id = try body_interface.createAndAddBody(&sphere_settings, .activate);

        // Now you can interact with the dynamic body, in this case we're going to give it a velocity.
        body_interface.setLinearVelocity(self.sphere_id, Vec3.init(0.0, -5.0, 0.0));

        // Optional step: Before starting the physics simulation you can optimize the broad phase.
        try self.physics_system.optimizeBroadPhase();
        return self;
    }

    fn destroy(self: *ZoltHelloWorld) void {
        const allocator = self.allocator;
        const body_interface = self.physics_system.getBodyInterfaceMut();

        // HelloWorld's cleanup
        if (!self.sphere_id.isInvalid()) {
            body_interface.removeBody(self.sphere_id);
            body_interface.destroyBody(self.sphere_id);
        }
        if (!self.floor_id.isInvalid()) {
            body_interface.removeBody(self.floor_id);
            body_interface.destroyBody(self.floor_id);
        }

        self.physics_system.deinit();
        switch (self.job_kind) {
            .single_threaded => self.single_threaded.deinit(),
            .thread_pool => self.thread_pool.deinit(),
        }
        self.recorder.deinit();
        self.temp_allocator.deinit();
        allocator.destroy(self);
    }

    fn jobSystem(self: *ZoltHelloWorld) JobSystem {
        return switch (self.job_kind) {
            .single_threaded => self.single_threaded.jobSystem(),
            .thread_pool => self.thread_pool.jobSystem(),
        };
    }

    fn isActive(self: *const ZoltHelloWorld) bool {
        return self.physics_system.getBodyInterface().isActive(self.sphere_id);
    }

    /// HelloWorld's line of a step (without the new line)
    fn stepLine(self: *const ZoltHelloWorld, buffer: []u8, step: u32) ![]const u8 {
        const body_interface = self.physics_system.getBodyInterface();
        const position = body_interface.getCenterOfMassPosition(self.sphere_id);
        const velocity = body_interface.getLinearVelocity(self.sphere_id);
        return std.fmt.bufPrint(buffer, "Step {d}: Position = ({f}, {f}, {f}), Velocity = ({f}, {f}, {f})", .{ step, cppFloat(position.getX()), cppFloat(position.getY()), cppFloat(position.getZ()), cppFloat(velocity.getX()), cppFloat(velocity.getY()), cppFloat(velocity.getZ()) });
    }

    fn update(self: *ZoltHelloWorld) !zolt.PhysicsUpdateError {
        const collision_steps: u32 = 1;
        const errors = try self.physics_system.update(delta_time, collision_steps, self.temp_allocator.tempAllocator(), self.jobSystem());
        if (!self.temp_allocator.isEmpty()) return error.TempAllocatorNotEmpty;
        return errors;
    }

    fn saveState(self: *const ZoltHelloWorld, out: *std.ArrayList(u8)) !void {
        var recorder = StateRecorderImpl.init(self.allocator);
        defer recorder.deinit();
        try self.physics_system.saveState(recorder.recorder(), .{});
        if (recorder.isFailed()) return error.OutOfMemory;
        out.clearRetainingCapacity();
        try out.appendSlice(self.allocator, recorder.getData());
    }

    fn takeRecords(self: *ZoltHelloWorld, out: *std.ArrayList(SimRecord)) !void {
        try self.recorder.checkError();
        self.recorder.mutex.lock(io);
        defer self.recorder.mutex.unlock(io);
        try out.appendSlice(self.allocator, self.recorder.records.items);
        self.recorder.records.clearRetainingCapacity();
    }
};

/// HelloWorldReference.cpp's world
const CppHelloWorld = struct {
    handle: *anyopaque,

    fn init(num_threads: i32) CppHelloWorld {
        return .{ .handle = jolt.jolt_hello_world_create(num_threads) };
    }

    fn deinit(self: CppHelloWorld) void {
        jolt.jolt_hello_world_destroy(self.handle);
    }

    fn isActive(self: CppHelloWorld) bool {
        return jolt.jolt_hello_world_is_active(self.handle) != 0;
    }

    fn stepLine(self: CppHelloWorld, buffer: []u8, step: u32) ![]const u8 {
        const size = jolt.jolt_hello_world_step_line(self.handle, step, buffer.ptr, @intCast(buffer.len));
        if (size > buffer.len) return error.LineTooLong;
        return buffer[0..size];
    }

    fn update(self: CppHelloWorld) u32 {
        return jolt.jolt_hello_world_update(self.handle);
    }

    fn saveState(self: CppHelloWorld, allocator: Allocator, out: *std.ArrayList(u8)) !void {
        out.clearRetainingCapacity();
        while (true) {
            const size = jolt.jolt_hello_world_save_state(self.handle, out.items.ptr, @intCast(out.capacity));
            if (size <= out.capacity) {
                out.items.len = size;
                return;
            }
            try out.ensureTotalCapacity(allocator, size);
        }
    }

    fn takeRecords(self: CppHelloWorld, allocator: Allocator, out: *std.ArrayList(SimRecord)) !void {
        while (true) {
            const unused = out.unusedCapacitySlice();
            const count = jolt.jolt_hello_world_take_records(self.handle, unused.ptr, @intCast(unused.len));
            if (count <= unused.len) {
                out.items.len += count;
                return;
            }
            try out.ensureUnusedCapacity(allocator, count);
        }
    }

    // For SimulationParity's state decoder

    pub fn hasMotionProperties(self: CppHelloWorld, body_id: u32) bool {
        return jolt.jolt_hello_world_body_has_motion_properties(self.handle, body_id) != 0;
    }

    pub fn constraintStateSize(self: CppHelloWorld, index: u32) u32 {
        _ = self;
        _ = index;
        return 0; // HelloWorld has no constraints
    }
};

// -------------------------------------------------------------------------------------------------------------------
// The driver

/// Sort both record lists and compare them
fn expectSameRecords(allocator: Allocator, what: []const u8, step: u32, zolt_records: []SimRecord, jolt_records: []SimRecord) !void {
    _ = allocator;
    std.mem.sort(SimRecord, zolt_records, {}, SimRecord.lessThan);
    std.mem.sort(SimRecord, jolt_records, {}, SimRecord.lessThan);
    const equal = zolt_records.len == jolt_records.len and for (zolt_records, jolt_records) |a, b| {
        if (!std.meta.eql(a, b)) break false;
    } else true;
    if (!equal) {
        std.debug.print("HelloWorld: the callbacks {s} {d} differ\n  zolt ({d}): {any}\n  jolt ({d}): {any}\n", .{ what, step, zolt_records.len, zolt_records, jolt_records.len, jolt_records });
        return error.ParityMismatch;
    }
}

/// What a run did (the same in every configuration)
const RunResult = struct {
    steps: u32,
    num_records: [7]u32 = @splat(0),
};

/// HelloWorld's loop on both sides in lockstep
fn runHelloWorld(allocator: Allocator, zolt_jobs: JobKind, cpp_threads: i32) !RunResult {
    const zw = try ZoltHelloWorld.create(allocator, zolt_jobs);
    defer zw.destroy();
    const cw = CppHelloWorld.init(cpp_threads);
    defer cw.deinit();

    var result: RunResult = .{ .steps = 0 };
    var zolt_records: std.ArrayList(SimRecord) = .empty;
    defer zolt_records.deinit(allocator);
    var jolt_records: std.ArrayList(SimRecord) = .empty;
    defer jolt_records.deinit(allocator);
    var zolt_state: std.ArrayList(u8) = .empty;
    defer zolt_state.deinit(allocator);
    var jolt_state: std.ArrayList(u8) = .empty;
    defer jolt_state.deinit(allocator);

    // The same bodies and the callbacks of the creation
    var floor_id: u32 = 0;
    var sphere_id: u32 = 0;
    jolt.jolt_hello_world_body_ids(cw.handle, &floor_id, &sphere_id);
    try std.testing.expectEqual(floor_id, zw.floor_id.getIndexAndSequenceNumber());
    try std.testing.expectEqual(sphere_id, zw.sphere_id.getIndexAndSequenceNumber());
    try zw.takeRecords(&zolt_records);
    try cw.takeRecords(allocator, &jolt_records);
    try expectSameRecords(allocator, "of the creation, step", 0, zolt_records.items, jolt_records.items);
    for (jolt_records.items) |r| result.num_records[r.kind] += 1;

    // Now we're ready to simulate the body, keep simulating until it goes to sleep
    var zolt_line_buffer: [512]u8 = undefined;
    var jolt_line_buffer: [512]u8 = undefined;
    while (true) {
        const active = zw.isActive();
        try std.testing.expectEqual(cw.isActive(), active);
        if (!active) break;

        // Next step
        result.steps += 1;
        const step = result.steps;
        if (step > 1000) return error.SphereNeverSleeps;

        // The position and velocity of the sphere, and HelloWorld's line
        var jolt_position: [3]Real = undefined;
        var jolt_velocity: [3]f32 = undefined;
        jolt.jolt_hello_world_sphere_state(cw.handle, &jolt_position, &jolt_velocity);
        const body_interface = zw.physics_system.getBodyInterface();
        const position = body_interface.getCenterOfMassPosition(zw.sphere_id);
        const velocity = body_interface.getLinearVelocity(zw.sphere_id);
        const zolt_position = [3]Real{ position.getX(), position.getY(), position.getZ() };
        const zolt_velocity = [3]f32{ velocity.getX(), velocity.getY(), velocity.getZ() };
        if (!std.mem.eql(u8, std.mem.asBytes(&zolt_position), std.mem.asBytes(&jolt_position)) or !std.mem.eql(u8, std.mem.asBytes(&zolt_velocity), std.mem.asBytes(&jolt_velocity))) {
            std.debug.print("HelloWorld: the sphere differs before step {d}\n  zolt: position {any} velocity {any}\n  jolt: position {any} velocity {any}\n", .{ step, zolt_position, zolt_velocity, jolt_position, jolt_velocity });
            return error.ParityMismatch;
        }
        const zolt_line = try zw.stepLine(&zolt_line_buffer, step);
        const jolt_line = try cw.stepLine(&jolt_line_buffer, step);
        if (!std.mem.eql(u8, zolt_line, jolt_line)) {
            std.debug.print("HelloWorld: the line of step {d} differs\n  zolt: \"{s}\"\n  jolt: \"{s}\"\n", .{ step, zolt_line, jolt_line });
            return error.ParityMismatch;
        }

        // Step the world
        const zolt_errors = try zw.update();
        const jolt_errors = cw.update();
        try std.testing.expectEqual(jolt_errors, @as(u32, @bitCast(zolt_errors)));

        // The state after the step
        try zw.saveState(&zolt_state);
        try cw.saveState(allocator, &jolt_state);
        if (try SimulationParity.describeStateDifference(allocator, zolt_state.items, jolt_state.items, cw)) |message| {
            defer allocator.free(message);
            std.debug.print("HelloWorld: after step {d}: {s}\n", .{ step, message });
            return error.ParityMismatch;
        }

        // The callbacks of the step
        zolt_records.clearRetainingCapacity();
        jolt_records.clearRetainingCapacity();
        try zw.takeRecords(&zolt_records);
        try cw.takeRecords(allocator, &jolt_records);
        try expectSameRecords(allocator, "of step", step, zolt_records.items, jolt_records.items);
        for (jolt_records.items) |r| result.num_records[r.kind] += 1;
    }
    return result;
}

/// The number of worker threads of HelloWorld's job system: thread::hardware_concurrency() - 1 (-1 for JobSystemThreadPool)
const hardware_threads: i32 = -1;

const Config = struct {
    zolt: JobKind,
    cpp_threads: i32,
};

const configs = [_]Config{
    .{ .zolt = .single_threaded, .cpp_threads = 0 },
    .{ .zolt = .{ .thread_pool = 0 }, .cpp_threads = hardware_threads },
    .{ .zolt = .{ .thread_pool = 1 }, .cpp_threads = 3 },
    .{ .zolt = .{ .thread_pool = 3 }, .cpp_threads = 0 },
    .{ .zolt = .{ .thread_pool = 7 }, .cpp_threads = hardware_threads },
    .{ .zolt = .{ .thread_pool = hardware_threads }, .cpp_threads = hardware_threads },
};

test "HelloWorld parity: the sphere, the state and the callbacks after every step" {
    const allocator = std.testing.allocator;
    var first: ?RunResult = null;
    for (configs) |config| {
        const result = runHelloWorld(allocator, config.zolt, config.cpp_threads) catch |err| {
            std.debug.print("HelloWorld parity failed with Zolt {any} and C++ {d} threads\n", .{ config.zolt, config.cpp_threads });
            return err;
        };
        if (first) |f| {
            try std.testing.expectEqual(f.steps, result.steps);
            try std.testing.expectEqual(f.num_records, result.num_records);
        } else first = result;
    }

    // HelloWorld's run: the sphere falls on the floor, bounces off it (a contact is added, persisted and removed while
    // it bounces) and falls asleep after `hello_world_steps` steps
    const result = first.?;
    try std.testing.expectEqual(@as(u32, hello_world_steps), result.steps);
    try std.testing.expectEqual(@as(u32, 1), result.num_records[@intFromEnum(RecordKind.body_activated)]);
    try std.testing.expectEqual(@as(u32, 1), result.num_records[@intFromEnum(RecordKind.body_deactivated)]);
    try std.testing.expect(result.num_records[@intFromEnum(RecordKind.contact_added)] >= 1);
    try std.testing.expect(result.num_records[@intFromEnum(RecordKind.contact_persisted)] >= 1);
}
