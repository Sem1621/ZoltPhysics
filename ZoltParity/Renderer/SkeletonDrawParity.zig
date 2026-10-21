//! Parity tests of the skeleton pose draws (Docs/Zolt/RemainingFeaturesArchitecture.md section 4, "Draw call log", group
//! D4). C ABI wrappers: `jolt_dr_run_skeleton_ops` in ZoltParity/Renderer/DebugRendererReference.cpp, and
//! `jolt_skel_draw_pose_create` / `jolt_skel_draw_pose_destroy` in ZoltParity/Physics/SkeletonReference.cpp. The op (128)
//! is in ParityRenderer.zig and ParityRenderer.h.
//!
//! Both sides build the same poses with the generator of SkeletonParity.zig: random trees of 0 to 40 joints named j0, j1,
//! ... (the draws write the names as texts), random joint states and root offsets (far from the origin too), then
//! CalculateJointMatrices. Every frame draws every pose with every combination of SkeletonPose::DrawSettings, once with
//! the default offset and once with a random offset (a rotation and a translation, far from the origin too).
//!
//! The layers (they exist only with -Ddebug_renderer=true, without the option they skip): the call log of a
//! ParityRenderer, the lines and texts of a DebugRendererSimple and the .jor bytes of a DebugRendererRecorder. The logs
//! must match call for call.

const std = @import("std");
const zolt = @import("zolt");
const pr = @import("ParityRenderer.zig");
const pw = @import("../Physics/PairwiseParity.zig");
const sp = @import("../Physics/SkeletonParity.zig");

const Real = zolt.Real;
const RVec3 = zolt.RVec3;
const SkeletonPose = zolt.SkeletonPose;
const JoltResult = pr.JoltResult;

const allocator = std.testing.allocator;
const io = std.testing.io;

const jolt = struct {
    extern fn jolt_dr_available() c_int;
    extern fn jolt_skel_draw_pose_create(skeleton: *const sp.SkelDesc, states: [*]const [7]f32, root_offset: *const [3]Real) *anyopaque;
    extern fn jolt_skel_draw_pose_destroy(pose: *anyopaque) void;
    extern fn jolt_dr_run_skeleton_ops(poses: [*]const *anyopaque, num_poses: u32, ops: [*]const u8, size: u64, kind: c_int) ?*anyopaque;
};

/// Skip without the option (the C++ reference has no DebugRenderer then)
fn requireDebugRenderer() !void {
    if (!zolt.Core.debug_renderer) return error.SkipZigTest;
    try std.testing.expectEqual(@as(c_int, 1), jolt.jolt_dr_available()); // Both sides follow -Ddebug_renderer
}

/// The renderer that runs the ops (the `kind` of jolt_dr_run_skeleton_ops)
const Kind = enum(c_int) { parity_renderer = 0, simple = 1, recorder = 2 };

/// The poses on both sides
const Poses = struct {
    zolt_poses: []SkeletonPose,
    pointers: []*const SkeletonPose,
    jolt_poses: []*anyopaque,

    /// `count` random poses: the first has no joints, the second one joint, the others up to max_joints
    fn create(seed: u32, count: usize) !Poses {
        var g: sp.Gen = .{ .rng = .{ .state = seed } };
        var offsets: pw.Gen = .{ .rng = .{ .state = seed ^ 0x0ff5e7 } };
        const zolt_poses = try allocator.alloc(SkeletonPose, count);
        errdefer allocator.free(zolt_poses);
        const pointers = try allocator.alloc(*const SkeletonPose, count);
        errdefer allocator.free(pointers);
        const jolt_poses = try allocator.alloc(*anyopaque, count);
        errdefer allocator.free(jolt_poses);

        for (zolt_poses, pointers, jolt_poses, 0..) |*pose, *pointer, *jolt_pose, i| {
            const num_joints: usize = switch (i) {
                0 => 0,
                1 => 1,
                else => 1 + offsets.index(sp.max_joints),
            };
            const desc = g.tree(num_joints);
            var states: [sp.max_joints][7]f32 = undefined;
            for (&states) |*s| s.* = g.state();
            const far: Real = if (offsets.chance(30)) 1.0e5 else 1.0;
            const root_offset = [3]Real{ far * @as(Real, offsets.float(-1, 1)), offsets.float(-2, 2), far * @as(Real, offsets.float(-1, 1)) };

            // Zolt
            var skeleton = try sp.makeSkeleton(allocator, &desc, null);
            defer skeleton.deinit();
            pose.* = .init(allocator);
            try pose.setSkeleton(skeleton.get().?);
            for (0..pose.getJointCount()) |j| sp.loadState(&states[j], pose.getJointMut(j));
            pose.setRootOffset(RVec3.init(root_offset[0], root_offset[1], root_offset[2]));
            pose.calculateJointMatrices();
            pointer.* = pose;

            // Jolt
            jolt_pose.* = jolt.jolt_skel_draw_pose_create(&desc, &states, &root_offset);
        }
        return .{ .zolt_poses = zolt_poses, .pointers = pointers, .jolt_poses = jolt_poses };
    }

    fn destroy(self: *Poses) void {
        for (self.zolt_poses) |*pose| pose.deinit();
        for (self.jolt_poses) |pose| jolt.jolt_skel_draw_pose_destroy(pose);
        allocator.free(self.zolt_poses);
        allocator.free(self.pointers);
        allocator.free(self.jolt_poses);
    }
};

/// A random offset: a rotation and a translation, now and then far from the origin
fn randomOffset(gen: *pw.Gen) pr.Affine {
    const q = gen.rotation();
    const translation = if (gen.chance(20))
        RVec3.init(@as(Real, gen.float(-1.0e5, 1.0e5)) + @as(Real, gen.float(0, 1)) / 7, gen.float(-1.0e3, 1.0e3), gen.float(-1.0e5, 1.0e5))
    else
        RVec3.init(gen.float(-10, 10), gen.float(-10, 10), gen.float(-10, 10));
    return .{ .columns = .{ q.rotateAxisX(), q.rotateAxisY(), q.rotateAxisZ() }, .translation = translation };
}

/// The ops: in every frame every pose with every combination of the draw settings, with the default and a random offset
fn makeOps(w: *pr.OpWriter, num_poses: usize, seed: u32, frames: u32, end_frames: bool) !void {
    var gen: pw.Gen = .{ .rng = .{ .state = seed } };
    for (0..frames) |_| {
        for (0..num_poses) |p| {
            for (0..pr.SkeletonDrawBits.all + 1) |bits| {
                try w.drawSkeletonPose(@intCast(p), @intCast(bits), null);
                try w.drawSkeletonPose(@intCast(p), @intCast(bits), randomOffset(&gen));
            }
        }
        if (end_frames) try w.endFrame();
    }
}

/// Compare the .jor bytes of the recorders (after the construction)
fn compareBytes(name: []const u8, zolt_bytes: []const u8, jolt_bytes: []const u8) !void {
    if (std.mem.eql(u8, zolt_bytes, jolt_bytes))
        return;
    const index = std.mem.indexOfDiff(u8, zolt_bytes, jolt_bytes).?;
    std.debug.print("{s}: the .jor bytes differ at byte {d} of {d} (zolt) / {d} (jolt)\n  zolt: {x}\n  jolt: {x}\n", .{ name, index, zolt_bytes.len, jolt_bytes.len, zolt_bytes[index..@min(index + 64, zolt_bytes.len)], jolt_bytes[index..@min(index + 64, jolt_bytes.len)] });
    return error.ParityMismatch;
}

/// One layer: `num_poses` poses, `frames` frames of makeOps on the renderers of `kind`
fn runLayer(name: []const u8, seed: u32, kind: Kind, num_poses: usize, frames: u32) !void {
    var poses = try Poses.create(seed, num_poses);
    defer poses.destroy();

    var w: pr.OpWriter = .{ .allocator = allocator };
    defer w.deinit();
    try makeOps(&w, num_poses, seed +% 1, frames, kind == .recorder);

    const result: JoltResult = .{ .handle = jolt.jolt_dr_run_skeleton_ops(poses.jolt_poses.ptr, @intCast(num_poses), w.bytes.items.ptr, w.bytes.items.len, @intFromEnum(kind)).? };
    defer result.deinit();

    var context: pr.PhysicsContext = .{ .allocator = allocator, .poses = poses.pointers };
    defer context.deinit();
    switch (kind) {
        .parity_renderer => {
            var renderer: pr.ParityRenderer = undefined;
            try renderer.init(allocator);
            defer renderer.deinit();
            renderer.log.bytes.clearRetainingCapacity();
            try pr.runOpsWithContext(allocator, &renderer.base, w.bytes.items, null, null, null, &context);
            try pr.compareLogs(name, renderer.log.bytes.items, result.part(0));
        },
        .simple => {
            var renderer: pr.SimpleParityRenderer = undefined;
            try renderer.init(allocator);
            defer renderer.deinit();
            renderer.log.bytes.clearRetainingCapacity();
            try pr.runOpsWithContext(allocator, renderer.base.debugRenderer(), w.bytes.items, &renderer.base, null, null, &context);
            try pr.compareLogs(name, renderer.log.bytes.items, result.part(0));
        },
        .recorder => {
            var out: std.Io.Writer.Allocating = .init(allocator);
            defer out.deinit();
            var wrapper: zolt.StreamOutWrapper = .init(&out.writer);
            var recorder: zolt.DebugRendererRecorder = undefined;
            try recorder.init(allocator, io, wrapper.streamOut());
            defer recorder.deinit();
            const construction_len = out.written().len;
            try pr.runOpsWithContext(allocator, recorder.debugRenderer(), w.bytes.items, null, &recorder, null, &context);
            try compareBytes(name, out.written()[construction_len..], result.part(0));
        },
    }
    try std.testing.expectEqual(@as(usize, 0), result.part(1).len); // The skeleton op computes nothing
}

test "Skeleton draw parity: SkeletonPose::Draw of random poses, every DrawSettings combination, offsets (call log)" {
    try requireDebugRenderer();
    try runLayer("skeleton pose draws", 0xd4000001, .parity_renderer, 40, 2);
}

test "Skeleton draw parity: DebugRendererSimple (call log)" {
    try requireDebugRenderer();
    try runLayer("skeleton pose draws: DebugRendererSimple", 0xd4000002, .simple, 12, 1);
}

test "Skeleton draw parity: DebugRendererRecorder (.jor bytes)" {
    try requireDebugRenderer();
    try runLayer("skeleton pose draws: recorder", 0xd4000003, .recorder, 12, 2);
}
