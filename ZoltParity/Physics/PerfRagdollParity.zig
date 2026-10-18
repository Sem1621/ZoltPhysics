//! perf_ragdoll (SerializationArchitecture.md P10, milestone M9): ZoltPerformanceTest's RagdollScene against
//! PerformanceTest/RagdollScene.h, run by the C++ library (PerfRagdollReference.cpp). Both sides run the scene code
//! itself: Zolt's port is the module "perf_ragdoll_scene" (ZoltPerformanceTest/RagdollScene.zig, see build.zig).
//!
//! - Stage 1: both sides Load the scene from Assets/ (the three assets through ObjectStreamIn). Then these bytes must be
//!   equal: the SaveBinaryState of the RagdollSettings (with shapes and group filters) after CalculateParentJointIndices,
//!   Stabilize and the two Calculate* tables, the two tables, the parent joint indices, the SaveBinaryState of the
//!   animation, the sampled pose (root offset, joint rotations and translations) and the SaveBinaryState of the terrain
//!   after FixInvalidScales (with shapes and group filters).
//! - Stage 2: one PhysicsSystem per thread configuration, set up like PerformanceTest's main (-s=Ragdoll
//!   -q=LinearCast): StartTest, OptimizeBroadPhase, then the first steps (UpdateTest and Update). The SaveState(All)
//!   bytes must be equal after StartTest and after every step, the update errors too, and the hash of PerformanceTest
//!   at the end. A difference is decoded to (section, body, field) by SimulationParity.zig's decoder.
//!
//! Thread matrix: Zolt with JobSystemSingleThreaded and JobSystemThreadPool with 0, 1 and 3 workers, C++ with a
//! JobSystemThreadPool with 0, 3 and 1 workers. The scene has 160 ragdolls (3680 bodies), so the steps are few. The
//! parts of the ragdolls touch each other from the first step, and the lowest ragdolls land on the terrain from about
//! step 16, so the first configuration runs 40 steps and the others 20. When the Ragdoll hash no longer matches, this
//! test usually shows the first differing field. For a later divergence, raise the `num_updates` of `configs`.
//!
//! Proof that it bites: raising the friction of part 1 by 0.01 in StartTest is reported as "the state after update 23
//! differs ... bodies[1147] (id 0x0080047b) position.x zolt 0x40bf8567 jolt 0x40bf856e". Scaling the mass override of
//! part 1 by 1.001 after Stabilize fails stage 1 ("RagdollSettings::SaveBinaryState differs at byte 1290").

const std = @import("std");
const zolt = @import("zolt");
const perf = @import("perf_ragdoll_scene");
const sim = @import("SimulationParity.zig");

const Allocator = std.mem.Allocator;
const Real = zolt.Real;
const PhysicsSystem = zolt.PhysicsSystem;
const TempAllocatorImpl = zolt.TempAllocatorImpl;
const JobSystem = zolt.JobSystem;
const JobSystemSingleThreaded = zolt.JobSystemSingleThreaded;
const JobSystemThreadPool = zolt.JobSystemThreadPool;
const StateRecorderImpl = zolt.StateRecorderImpl;
const StreamOutWrapper = zolt.StreamOutWrapper;
const StreamOut = zolt.StreamOut;
const BodyIDVector = zolt.BodyIDVector;
const HashCombine = zolt.HashCombine;
const RagdollScene = perf.RagdollScene;
const Layers = perf.layers;

const io = std.testing.io;

extern fn jolt_pr_load(asset_path: [*:0]const u8, num_piles_per_axis: u32, pile_size: u32, vertical_separation: f32) u32;
extern fn jolt_pr_result(index: u32, size: *u64) [*]const u8;
extern fn jolt_pr_unload() void;
extern fn jolt_pr_world_create(num_threads: u32, motion_quality: u32) *anyopaque;
extern fn jolt_pr_world_num_ragdolls(world: *anyopaque) u32;
extern fn jolt_pr_world_update(world: *anyopaque) u32;
extern fn jolt_pr_world_state(world: *anyopaque, size: *u64) [*]const u8;
extern fn jolt_pr_world_hash(world: *anyopaque) u64;
extern fn jolt_pr_world_destroy(world: *anyopaque) void;
extern fn jolt_pr_body_has_motion_properties(world: *anyopaque, body_id: u32) u32;
extern fn jolt_pr_constraint_state_size(world: *anyopaque, index: u32) u32;
extern fn jolt_pr_constraint_sub_type(world: *anyopaque, index: u32) u32;

/// PerformanceTest's -s=Ragdoll: RagdollScene(4, 10, 0.6f)
const num_piles_per_axis = 4;
const pile_size = 10;
const vertical_separation: f32 = 0.6;

/// The asset path that PerformanceTest finds (the parity tests run in the repository root)
const asset_path = "Assets/";

/// Time step for physics (PerformanceTest.cpp)
const delta_time: f32 = 1.0 / 60.0;

fn cppBytes(index: u32) []const u8 {
    var size: u64 = 0;
    const ptr = jolt_pr_result(index, &size);
    return ptr[0..@intCast(size)];
}

/// The bytes that `write` writes to a StreamOut
fn saveBytes(allocator: Allocator, context: anytype, comptime write: fn (@TypeOf(context), StreamOut) anyerror!void) ![]u8 {
    var out: std.Io.Writer.Allocating = .init(allocator);
    errdefer out.deinit();
    var wrapper = StreamOutWrapper.init(&out.writer);
    try write(context, wrapper.streamOut());
    if (wrapper.isFailed()) return error.OutOfMemory;
    return out.toOwnedSlice();
}

fn expectSameBytes(what: []const u8, zolt_bytes: []const u8, jolt_bytes: []const u8) !void {
    if (std.mem.eql(u8, zolt_bytes, jolt_bytes)) return;
    const at = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes) orelse 0;
    std.debug.print("perf_ragdoll: {s} differs at byte {d} (zolt {d} bytes, jolt {d} bytes)\n", .{ what, at, zolt_bytes.len, jolt_bytes.len });
    const from = at -| 16;
    std.debug.print("  zolt: {x}\n  jolt: {x}\n", .{ zolt_bytes[from..@min(zolt_bytes.len, at + 32)], jolt_bytes[from..@min(jolt_bytes.len, at + 32)] });
    return error.ParityMismatch;
}

/// The Zolt scene, loaded like PerformanceTest's main does it
fn loadZoltScene(allocator: Allocator) !*RagdollScene {
    const scene = try RagdollScene.create(allocator, io, num_piles_per_axis, pile_size, vertical_separation);
    errdefer scene.destroy();
    if (!try scene.load(asset_path)) return error.HarnessError;
    return scene;
}

fn loadCppScene() !void {
    if (jolt_pr_load(asset_path, num_piles_per_axis, pile_size, vertical_separation) == 0) {
        std.debug.print("perf_ragdoll: the C++ RagdollScene cannot load {s}\n", .{asset_path});
        return error.HarnessError;
    }
}

// Stage 1 -----------------------------------------------------------------------------------------------------------

fn writeSettings(scene: *const RagdollScene, stream: StreamOut) anyerror!void {
    try scene.ragdoll_settings.get().?.saveBinaryState(stream, true, true);
}

fn writeTables(scene: *const RagdollScene, stream: StreamOut) anyerror!void {
    const settings = scene.ragdoll_settings.get().?;
    stream.write(@as(u32, @intCast(settings.getBodyIndexToConstraintIndex().len)));
    for (settings.getBodyIndexToConstraintIndex()) |i| stream.write(i);
    stream.write(@as(u32, @intCast(settings.getConstraintIndexToBodyIdxPair().len)));
    for (settings.getConstraintIndexToBodyIdxPair()) |p| {
        stream.write(p.first);
        stream.write(p.second);
    }
}

fn writeAnimation(scene: *const RagdollScene, stream: StreamOut) anyerror!void {
    scene.animation.get().?.saveBinaryState(stream);
}

fn writePose(scene: *const RagdollScene, stream: StreamOut) anyerror!void {
    stream.write(scene.pose.getRootOffset());
    for (scene.pose.getJoints()) |j| {
        stream.write(j.rotation);
        stream.write(j.translation);
    }
}

fn writeTerrain(scene: *const RagdollScene, stream: StreamOut) anyerror!void {
    try scene.background.get().?.saveBinaryState(stream, true, true);
}

fn writeParentJointIndices(scene: *const RagdollScene, stream: StreamOut) anyerror!void {
    for (scene.ragdoll_settings.get().?.getSkeleton().?.getJoints()) |j| stream.write(j.parent_joint_index);
}

test "perf_ragdoll stage 1: the loaded ragdoll settings, animation, pose and terrain of RagdollScene" {
    const allocator = std.testing.allocator;
    try loadCppScene();
    defer jolt_pr_unload();
    const scene = try loadZoltScene(allocator);
    defer scene.destroy();

    const Write = fn (*const RagdollScene, StreamOut) anyerror!void;
    const parts = [_]struct { name: []const u8, index: u32, write: *const Write }{
        .{ .name = "RagdollSettings::SaveBinaryState", .index = 0, .write = writeSettings },
        .{ .name = "the body / constraint tables of the RagdollSettings", .index = 1, .write = writeTables },
        .{ .name = "SkeletalAnimation::SaveBinaryState", .index = 2, .write = writeAnimation },
        .{ .name = "the sampled pose", .index = 3, .write = writePose },
        .{ .name = "PhysicsScene::SaveBinaryState of the terrain", .index = 4, .write = writeTerrain },
        .{ .name = "the parent joint indices of the skeleton", .index = 5, .write = writeParentJointIndices },
    };
    inline for (parts) |part| {
        const bytes = try saveBytes(allocator, @as(*const RagdollScene, scene), part.write.*);
        defer allocator.free(bytes);
        try expectSameBytes(part.name, bytes, cppBytes(part.index));
    }

    // The scene really has what the hashes depend on: a part and a pose joint for every joint, the terrain bodies
    const settings = scene.ragdoll_settings.get().?;
    try std.testing.expectEqual(settings.getSkeleton().?.getJointCount(), settings.parts.items.len);
    try std.testing.expectEqual(settings.parts.items.len, scene.pose.getJointCount());
    try std.testing.expect(settings.parts.items.len > 10);
    try std.testing.expect(scene.background.get().?.getBodies().len > 0);
}

// Stage 2 -----------------------------------------------------------------------------------------------------------

const JobKind = union(enum) {
    single_threaded,
    thread_pool: i32, // number of worker threads
};

const Config = struct {
    zolt: JobKind,
    /// The number of worker threads of the C++ JobSystemThreadPool
    cpp_threads: u32,
    /// The steps that are compared
    num_updates: u32,
};

/// Zolt with JobSystemSingleThreaded and thread pools with 0, 1 and 3 workers, C++ with 0, 3 and 1 workers
const configs = [_]Config{
    .{ .zolt = .single_threaded, .cpp_threads = 0, .num_updates = 40 },
    .{ .zolt = .{ .thread_pool = 0 }, .cpp_threads = 3, .num_updates = 20 },
    .{ .zolt = .{ .thread_pool = 1 }, .cpp_threads = 0, .num_updates = 20 },
    .{ .zolt = .{ .thread_pool = 3 }, .cpp_threads = 1, .num_updates = 20 },
};

/// One thread configuration of PerformanceTest's main on the Zolt side (heap allocated, the system must not move)
const ZoltWorld = struct {
    temp: TempAllocatorImpl,
    single: JobSystemSingleThreaded = .empty,
    pool: JobSystemThreadPool = .empty,
    is_pool: bool = false,
    broad_phase_layer_interface: Layers.BPLayerInterfaceImpl,
    object_vs_broad_phase_layer_filter: Layers.ObjectVsBroadPhaseLayerFilterImpl = .{},
    object_layer_pair_filter: Layers.ObjectLayerPairFilterImpl = .{},
    system: PhysicsSystem = .empty,

    fn create(allocator: Allocator, scene: *RagdollScene, kind: JobKind) !*ZoltWorld {
        const self = try allocator.create(ZoltWorld);
        errdefer allocator.destroy(self);
        self.* = .{ .temp = try .init(allocator, scene.scene().getTempAllocatorSizeMB() * 1024 * 1024), .broad_phase_layer_interface = .init() };
        errdefer self.temp.deinit();
        switch (kind) {
            .single_threaded => self.single = try .init(allocator, io, zolt.physics_settings.max_physics_jobs),
            .thread_pool => |n| {
                self.is_pool = true;
                try self.pool.init(allocator, io, zolt.physics_settings.max_physics_jobs, zolt.physics_settings.max_physics_barriers, .{ .num_threads = n });
            },
        }
        errdefer self.deinitJobs();
        const s = scene.scene();
        try self.system.init(allocator, io, s.getMaxBodies(), 0, s.getMaxBodyPairs(), s.getMaxContactConstraints(), &self.broad_phase_layer_interface.base, &self.object_vs_broad_phase_layer_filter.base, &self.object_layer_pair_filter.base, .{});
        errdefer self.system.deinit();
        try scene.startTest(&self.system, .linear_cast);
        errdefer scene.stopTest(&self.system) catch {};
        try self.system.optimizeBroadPhase();
        return self;
    }

    fn deinitJobs(self: *ZoltWorld) void {
        if (self.is_pool) self.pool.deinit() else self.single.deinit();
    }

    fn destroy(self: *ZoltWorld, allocator: Allocator, scene: *RagdollScene) !void {
        try scene.stopTest(&self.system);
        self.system.deinit();
        self.deinitJobs();
        self.temp.deinit();
        allocator.destroy(self);
    }

    fn jobSystem(self: *ZoltWorld) JobSystem {
        return if (self.is_pool) self.pool.jobSystem() else self.single.jobSystem();
    }

    fn state(self: *ZoltWorld, allocator: Allocator) ![]u8 {
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        try self.system.saveState(recorder.recorder(), .{});
        return allocator.dupe(u8, recorder.getData());
    }

    /// The hash of PerformanceTest's main
    fn hash(self: *ZoltWorld, allocator: Allocator, scene: *RagdollScene) !u64 {
        var h: u64 = HashCombine.hashBytes(&.{}); // Ensure we start with the proper seed
        const bi = self.system.getBodyInterfaceNoLock();
        var body_ids: BodyIDVector = .empty;
        defer body_ids.deinit(allocator);
        try self.system.getBodies(allocator, &body_ids);
        for (body_ids.items) |id| {
            const pos = bi.getPosition(id);
            h = HashCombine.hashBytesSeeded(std.mem.asBytes(&pos)[0 .. 3 * @sizeOf(Real)], h);
            const rot = bi.getRotation(id);
            h = HashCombine.hashBytesSeeded(std.mem.asBytes(&rot), h);
        }
        scene.scene().updateHash(&h);
        return h;
    }
};

/// The C++ world for SimulationParity's state decoder
const CppWorld = struct {
    handle: *anyopaque,

    pub fn hasMotionProperties(self: CppWorld, body_id: u32) bool {
        return jolt_pr_body_has_motion_properties(self.handle, body_id) != 0;
    }

    pub fn constraintStateSize(self: CppWorld, index: u32) u32 {
        return jolt_pr_constraint_state_size(self.handle, index);
    }

    pub fn constraintSubType(self: CppWorld, index: u32) ?zolt.ConstraintSubType {
        const sub_type = jolt_pr_constraint_sub_type(self.handle, index);
        return if (sub_type < @typeInfo(zolt.ConstraintSubType).@"enum".fields.len) @enumFromInt(sub_type) else null;
    }

    fn state(self: CppWorld) []const u8 {
        var size: u64 = 0;
        const ptr = jolt_pr_world_state(self.handle, &size);
        return ptr[0..@intCast(size)];
    }
};

fn compareStates(allocator: Allocator, zw: *ZoltWorld, cw: CppWorld, config: Config, update: ?u32) !void {
    const zolt_state = try zw.state(allocator);
    defer allocator.free(zolt_state);
    const jolt_state = cw.state();
    if (std.mem.eql(u8, zolt_state, jolt_state)) return;
    if (update) |u|
        std.debug.print("perf_ragdoll ({any}, C++ {d} threads): the state after update {d} differs\n", .{ config.zolt, config.cpp_threads, u })
    else
        std.debug.print("perf_ragdoll ({any}, C++ {d} threads): the state after StartTest differs\n", .{ config.zolt, config.cpp_threads });
    if (try sim.describeStateDifference(allocator, zolt_state, jolt_state, cw)) |text| {
        defer allocator.free(text);
        std.debug.print("  {s}\n", .{text});
    }
    return error.ParityMismatch;
}

test "perf_ragdoll stage 2: the state after StartTest and after each of the first updates, thread matrix" {
    const allocator = std.testing.allocator;
    try loadCppScene();
    defer jolt_pr_unload();
    const scene = try loadZoltScene(allocator);
    defer scene.destroy();

    for (configs) |config| {
        const zw = try ZoltWorld.create(allocator, scene, config.zolt);
        var zw_alive = true;
        defer if (zw_alive) zw.destroy(allocator, scene) catch {};
        const cw: CppWorld = .{ .handle = jolt_pr_world_create(config.cpp_threads, 1) };
        defer jolt_pr_world_destroy(cw.handle);

        // The ragdolls of StartTest: 4 x 4 piles of 10
        try std.testing.expectEqual(@as(usize, num_piles_per_axis * num_piles_per_axis * pile_size), scene.ragdolls.items.len);
        try std.testing.expectEqual(@as(u32, num_piles_per_axis * num_piles_per_axis * pile_size), jolt_pr_world_num_ragdolls(cw.handle));
        try compareStates(allocator, zw, cw, config, null);

        for (0..config.num_updates) |u| {
            // UpdateTest (RagdollScene has none) and Update
            try scene.scene().updateTest(&zw.system, zw.temp.tempAllocator(), delta_time);
            const zolt_errors = try zw.system.update(delta_time, 1, zw.temp.tempAllocator(), zw.jobSystem());
            const jolt_errors = jolt_pr_world_update(cw.handle);
            try std.testing.expectEqual(jolt_errors, @as(u32, @bitCast(zolt_errors)));
            try std.testing.expect(zw.temp.isEmpty());
            try compareStates(allocator, zw, cw, config, @intCast(u));
        }

        try std.testing.expectEqual(jolt_pr_world_hash(cw.handle), try zw.hash(allocator, scene));
        zw_alive = false;
        try zw.destroy(allocator, scene);
    }
}
