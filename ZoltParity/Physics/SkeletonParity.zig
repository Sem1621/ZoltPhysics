//! Parity tests of the Skeleton package (Jolt/Skeleton): Skeleton, SkeletalAnimation, SkeletonPose and SkeletonMapper,
//! compared bit for bit with the C++ library on random cases (fixed seeds). C ABI wrappers:
//! ZoltParity/Physics/SkeletonReference.cpp. Both sides build the same objects from the same descriptions and run the
//! same calls; every output is compared (the structs are zeroed on both sides, so entries that a side does not write
//! compare equal, and Map / MapReverse outputs start as the same sentinel matrix).
//!
//! - Skeleton: random AddJoint scripts (by parent name, with the default parent name, by parent index including
//!   negative indices other than -1; repeated, empty and unknown names, parents after their children), with and
//!   without CalculateParentJointIndices: the AddJoint results, the parent indices, GetJointCount,
//!   AreJointsCorrectlyOrdered, GetJointIndex of random names, the SaveBinaryState bytes, sRestoreFromBinaryState of the
//!   other side's bytes saved again, truncated streams (always "Failed to read skeleton from stream" on both sides).
//! - SkeletalAnimation: random animations of random skeletons (1 to 12 keyframes per joint, sorted times with repeated
//!   times, negative times, rotations that are equal, close (the LERP branch of SLERP) or in the other hemisphere,
//!   looping or not): Sample at times before the first keyframe, on keyframes, one ulp next to keyframes, between them,
//!   after the last one, on multiples of the duration and far beyond (the looping wrap around), GetDuration,
//!   ScaleJoints (Sample again), IsLooping, the SaveBinaryState bytes; sRestoreFromBinaryState of the other side's bytes
//!   and of truncated streams (Jolt never reports an error: what could be read, including partially read floats, is
//!   saved again and compared), animations with joints without keyframes.
//! - SkeletonPose: SetSkeleton (also to a second skeleton: the states that are kept and the default ones),
//!   CalculateJointMatrices, CalculateLocalSpaceJointMatrices, CalculateJointStates of random affine matrices (with
//!   scale), the root offset (Real, both precisions); JointState::FromMatrix (ToMatrix is CalculateLocalSpaceJointMatrices).
//! - SkeletonMapper: random skeleton pairs where skeleton 1 is a subset of skeleton 2 (chains of joints between mapped
//!   joints, extra joints at the root and the leaves, renamed joints that cannot be mapped, perturbed neutral poses),
//!   Initialize with the default argument and with a can map joint function (a table: no, if the names are equal, yes),
//!   LockTranslations, LockAllTranslations, both: the mappings with their transforms, the chains, the unmapped and
//!   locked joints, GetMappedJointIdx, IsJointTranslationLocked, Map and MapReverse. Skeletons in a random joint order
//!   (parents after their children) are compared for Initialize and MapReverse.
//! - Release only (Jolt asserts on the input, Zolt checks the same asserts in safe builds, so these run with
//!   `-Doptimize=ReleaseFast`): Sample at negative times and with unsorted keyframes, SkeletonPose with parents after
//!   their children, Map / LockTranslations / LockAllTranslations on skeletons in a random joint order.

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../ParityFramework.zig");

const Checker = fw.Checker;
const Allocator = std.mem.Allocator;
const Vec3 = zolt.Vec3;
const Vec4 = zolt.Vec4;
const Quat = zolt.Quat;
const Mat44 = zolt.Mat44;
const Real = zolt.Real;
const RVec3 = zolt.RVec3;
const Ref = zolt.Ref;
const Skeleton = zolt.Skeleton;
const SkeletalAnimation = zolt.SkeletalAnimation;
const SkeletonPose = zolt.SkeletonPose;
const SkeletonMapper = zolt.SkeletonMapper;
const StreamInWrapper = zolt.StreamInWrapper;
const StreamOutWrapper = zolt.StreamOutWrapper;

const max_joints = 40;
const max_name = 8;
const max_animated_joints = 12;
const max_keyframes = 12;
const max_times = 24;
const max_queries = 16;

/// Must match SkelJointDesc in SkeletonReference.cpp
const SkelJointDesc = extern struct {
    /// 0: AddJoint(name, parent name), 1: AddJoint(name) (default parent name), 2: AddJoint(name, parent index)
    kind: u32,
    /// Kind 2
    parent_index: i32,
    name_len: u32,
    parent_name_len: u32,
    name: [max_name]u8,
    parent_name: [max_name]u8,
};

/// Must match SkelDesc in SkeletonReference.cpp
const SkelDesc = extern struct {
    num_joints: u32,
    /// Call CalculateParentJointIndices after adding the joints
    calculate_parent_joint_indices: u32,
    joints: [max_joints]SkelJointDesc,
};

/// Must match SkelName in SkeletonReference.cpp
const SkelName = extern struct {
    len: u32,
    name: [max_name]u8,
};

/// Must match SkelResult in SkeletonReference.cpp
const SkelResult = extern struct {
    add_joint_results: [max_joints]u32,
    parent_joint_indices: [max_joints]i32,
    joint_count: u32,
    correctly_ordered: u32,
};

/// Must match SkelKeyframe in SkeletonReference.cpp
const SkelKeyframe = extern struct {
    time: f32,
    rotation: [4]f32,
    translation: [3]f32,
};

/// Must match SkelAnimatedJoint in SkeletonReference.cpp
const SkelAnimatedJoint = extern struct {
    name_len: u32,
    name: [max_name]u8,
    num_keyframes: u32,
    keyframes: [max_keyframes]SkelKeyframe,
};

/// Must match SkelAnimDesc in SkeletonReference.cpp
const SkelAnimDesc = extern struct {
    num_joints: u32,
    is_looping: u32,
    joints: [max_animated_joints]SkelAnimatedJoint,
};

/// Must match SkelPoseResult in SkeletonReference.cpp
const SkelPoseResult = extern struct {
    /// CalculateJointMatrices
    joint_matrices: [max_joints][16]f32,
    /// CalculateLocalSpaceJointMatrices
    local_matrices: [max_joints][16]f32,
    /// CalculateJointStates of the input matrices
    joint_states: [max_joints][7]f32,
    /// The joint states after SetSkeleton(skeleton 2)
    resized_states: [max_joints][7]f32,
    /// JointState::FromMatrix of every input matrix
    from_matrix_states: [max_joints][7]f32,
    root_offset: [3]Real,
    joint_count: u32,
    resized_joint_count: u32,
    resized_matrix_count: u32,
};

/// Must match SkelMapperDesc in SkeletonReference.cpp
const SkelMapperDesc = extern struct {
    skeleton1: SkelDesc,
    skeleton2: SkelDesc,
    neutral_pose1: [max_joints][16]f32,
    neutral_pose2: [max_joints][16]f32,
    /// 0: the default argument (sDefaultCanMapJoint), 1: canMapJoint with can_map_table
    can_map_mode: u32,
    /// Bit 0: LockTranslations(locked), bit 1: LockAllTranslations (when there are mappings), in that order
    lock_mode: u32,
    /// Call Map (no locked joint without a parent)
    map: u32,
    /// [j1 * n2 + j2]: 0 no, 1 if the names are equal, 2 yes
    can_map_table: [max_joints * max_joints]u8,
    locked: [max_joints]u8,
    pose1_model_space: [max_joints][16]f32,
    pose2_local_space: [max_joints][16]f32,
    pose2_model_space: [max_joints][16]f32,
    /// Written to the outputs of Map / MapReverse before the call
    sentinel: [16]f32,
};

/// Must match SkelMapperResult in SkeletonReference.cpp
const SkelMapperResult = extern struct {
    num_mappings: u32,
    num_chains: u32,
    num_unmapped: u32,
    num_locked: u32,
    mapping_joints: [max_joints][2]i32,
    mapping_matrices: [max_joints][2][16]f32,
    chain_lengths: [max_joints][2]u32,
    chain_joints1: [max_joints][max_joints]i32,
    chain_joints2: [max_joints][max_joints]i32,
    unmapped: [max_joints][2]i32,
    locked_joints: [2 * max_joints][2]i32,
    locked_translations: [2 * max_joints][3]f32,
    /// GetMappedJointIdx(j1) for j1 = -1 .. n1
    mapped_joint_idx: [max_joints + 2]i32,
    /// IsJointTranslationLocked(j2) for j2 = -1 .. n2
    is_locked: [max_joints + 2]u32,
    /// Map
    pose2_model_space: [max_joints][16]f32,
    /// MapReverse
    pose1_model_space: [max_joints][16]f32,
};

const jolt = struct {
    extern fn jolt_skel_struct_sizes(out_sizes: *[6]u32) void;
    extern fn jolt_skel_skeleton(desc: *const SkelDesc, queries: [*]const SkelName, num_queries: u32, out_joint_indices: [*]i32, out_result: *SkelResult, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_skel_skeleton_resave(bytes: [*]const u8, size: u32, out_bytes: [*]u8, capacity: u32, out_error: [*]u8, error_capacity: u32) u32;
    extern fn jolt_skel_animation(skeleton: *const SkelDesc, animation: *const SkelAnimDesc, initial_states: [*]const f32, times: [*]const f32, num_times: u32, scale: f32, out_states: [*]f32, out_durations: *[2]f32, out_is_looping: *u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_skel_animation_resave(bytes: [*]const u8, size: u32, out_bytes: [*]u8, capacity: u32) u32;
    extern fn jolt_skel_pose(skeleton1: *const SkelDesc, skeleton2: *const SkelDesc, states: [*]const f32, matrices: [*]const f32, root_offset: *const [3]Real, out_result: *SkelPoseResult) void;
    extern fn jolt_skel_mapper(desc: *const SkelMapperDesc, out_result: *SkelMapperResult) void;
};

// ---------------------------------------------------------------------------------------------------------------------
// Conversions

fn loadMat44(m: *const [16]f32) Mat44 {
    return Mat44.init(Vec4.init(m[0], m[1], m[2], m[3]), Vec4.init(m[4], m[5], m[6], m[7]), Vec4.init(m[8], m[9], m[10], m[11]), Vec4.init(m[12], m[13], m[14], m[15]));
}

fn storeMat44(m: Mat44, out: *[16]f32) void {
    for (0..4) |c| {
        const col = m.getColumn4(@intCast(c));
        out[4 * c ..][0..4].* = .{ col.getX(), col.getY(), col.getZ(), col.getW() };
    }
}

fn loadState(v: *const [7]f32, state: *SkeletalAnimation.JointState) void {
    state.rotation = Quat.init(v[0], v[1], v[2], v[3]);
    state.translation = Vec3.init(v[4], v[5], v[6]);
}

fn storeState(state: *const SkeletalAnimation.JointState, out: *[7]f32) void {
    out.* = .{ state.rotation.getX(), state.rotation.getY(), state.rotation.getZ(), state.rotation.getW(), state.translation.getX(), state.translation.getY(), state.translation.getZ() };
}

fn setName(out: *[max_name]u8, out_len: *u32, comptime fmt: []const u8, args: anytype) void {
    out_len.* = @intCast((std.fmt.bufPrint(out, fmt, args) catch unreachable).len);
}

// ---------------------------------------------------------------------------------------------------------------------
// The Zolt side of the C functions

/// The skeleton of a description (SkelMakeSkeleton), one reference owned by the caller
fn makeSkeleton(allocator: Allocator, desc: *const SkelDesc, out_add_joint_results: ?*[max_joints]u32) !Ref(Skeleton) {
    var skeleton = Ref(Skeleton).init(try Skeleton.create(allocator));
    errdefer skeleton.deinit();
    const s = skeleton.get().?;
    for (desc.joints[0..desc.num_joints], 0..) |*d, j| {
        const name = d.name[0..d.name_len];
        const result = switch (d.kind) {
            0 => try s.addJoint(name, .{ .parent_name = d.parent_name[0..d.parent_name_len] }),
            1 => try s.addJoint(name, .{}),
            else => try s.addJointWithParentIndex(name, d.parent_index),
        };
        if (out_add_joint_results) |r| r[j] = result;
    }
    if (desc.calculate_parent_joint_indices != 0)
        s.calculateParentJointIndices();
    return skeleton;
}

/// The animation of a description (SkelMakeAnimation), one reference owned by the caller
fn makeAnimation(allocator: Allocator, desc: *const SkelAnimDesc) !Ref(SkeletalAnimation) {
    var animation = Ref(SkeletalAnimation).init(try SkeletalAnimation.create(allocator));
    errdefer animation.deinit();
    const a = animation.get().?;
    a.setIsLooping(desc.is_looping != 0);
    for (desc.joints[0..desc.num_joints]) |*d| {
        var aj: SkeletalAnimation.AnimatedJoint = .{ .joint_name = try allocator.dupe(u8, d.name[0..d.name_len]) };
        errdefer aj.deinit(allocator);
        for (d.keyframes[0..d.num_keyframes]) |*kd| {
            try aj.keyframes.append(allocator, .{
                .base = .{ .rotation = Quat.init(kd.rotation[0], kd.rotation[1], kd.rotation[2], kd.rotation[3]), .translation = Vec3.init(kd.translation[0], kd.translation[1], kd.translation[2]) },
                .time = kd.time,
            });
        }
        try a.getAnimatedJointsMut().append(allocator, aj);
    }
    return animation;
}

/// SaveBinaryState into an owned slice
fn saveBytes(allocator: Allocator, object: anytype) ![]u8 {
    var data: std.Io.Writer.Allocating = .init(allocator);
    defer data.deinit();
    var w = StreamOutWrapper.init(&data.writer);
    object.saveBinaryState(w.streamOut());
    if (w.isFailed()) return error.OutOfMemory; // The allocating writer could not grow
    return data.toOwnedSlice();
}

/// jolt_skel_skeleton on the Zolt side
fn zoltSkeleton(allocator: Allocator, desc: *const SkelDesc, queries: []const SkelName, out_joint_indices: []i32, out: *SkelResult) ![]u8 {
    var skeleton = try makeSkeleton(allocator, desc, &out.add_joint_results);
    defer skeleton.deinit();
    const s = skeleton.get().?;
    for (s.getJoints(), 0..) |*j, i|
        out.parent_joint_indices[i] = j.parent_joint_index;
    out.joint_count = @intCast(s.getJointCount());
    out.correctly_ordered = @intFromBool(s.areJointsCorrectlyOrdered());
    for (queries, out_joint_indices) |*q, *r|
        r.* = s.getJointIndex(q.name[0..q.len]);
    return saveBytes(allocator, s);
}

/// A restore + save: the bytes or the error text
const Resaved = struct {
    bytes: std.ArrayList(u8) = .empty,
    err: ?[]const u8 = null,
    err_buffer: [128]u8 = undefined,

    fn deinit(self: *Resaved, allocator: Allocator) void {
        self.bytes.deinit(allocator);
    }

    fn setError(self: *Resaved, message: []const u8) void {
        const n = @min(message.len, self.err_buffer.len);
        @memcpy(self.err_buffer[0..n], message[0..n]);
        self.err = self.err_buffer[0..n];
    }

    fn eql(self: *const Resaved, other: *const Resaved) bool {
        if (self.err != null or other.err != null)
            return self.err != null and other.err != null and std.mem.eql(u8, self.err.?, other.err.?);
        return std.mem.eql(u8, self.bytes.items, other.bytes.items);
    }
};

/// Skeleton.restoreFromBinaryState of `bytes`, then saveBinaryState of the result
fn zoltSkeletonResave(allocator: Allocator, bytes: []const u8, out: *Resaved) !void {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    var result = try Skeleton.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    if (result.hasError()) {
        out.setError(result.getError());
        return;
    }
    const data = try saveBytes(allocator, result.getPtr().?);
    defer allocator.free(data);
    try out.bytes.appendSlice(allocator, data);
}

/// SkeletalAnimation.restoreFromBinaryState of `bytes` (never an error), then saveBinaryState of the result
fn zoltAnimationResave(allocator: Allocator, bytes: []const u8, out: *Resaved) !void {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    var result = try SkeletalAnimation.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    const data = try saveBytes(allocator, result.getPtr().?);
    defer allocator.free(data);
    try out.bytes.appendSlice(allocator, data);
}

var byte_buffer: [65536]u8 = undefined;

fn joltSkeletonResave(allocator: Allocator, bytes: []const u8, out: *Resaved) !void {
    var err_buffer: [128]u8 = undefined;
    const size = jolt.jolt_skel_skeleton_resave(bytes.ptr, @intCast(bytes.len), &byte_buffer, byte_buffer.len, &err_buffer, err_buffer.len);
    if (size == 0xFFFFFFFF) {
        out.setError(std.mem.sliceTo(&err_buffer, 0));
        return;
    }
    try std.testing.expect(size <= byte_buffer.len);
    try out.bytes.appendSlice(allocator, byte_buffer[0..size]);
}

fn joltAnimationResave(allocator: Allocator, bytes: []const u8, out: *Resaved) !void {
    const size = jolt.jolt_skel_animation_resave(bytes.ptr, @intCast(bytes.len), &byte_buffer, byte_buffer.len);
    try std.testing.expect(size <= byte_buffer.len);
    try out.bytes.appendSlice(allocator, byte_buffer[0..size]);
}

/// The results of jolt_skel_animation
const AnimationRun = struct {
    states: [2 * max_times * max_joints][7]f32,
    durations: [2]f32,
    is_looping: u32,
};

/// jolt_skel_animation on the Zolt side, returns the saved bytes
fn zoltAnimation(allocator: Allocator, skeleton_desc: *const SkelDesc, animation_desc: *const SkelAnimDesc, initial_states: []const [7]f32, times: []const f32, scale: f32, out: *AnimationRun) ![]u8 {
    var skeleton = try makeSkeleton(allocator, skeleton_desc, null);
    defer skeleton.deinit();
    var animation = try makeAnimation(allocator, animation_desc);
    defer animation.deinit();
    const a = animation.get().?;

    var pose: SkeletonPose = .init(allocator);
    defer pose.deinit();
    try pose.setSkeleton(skeleton.get().?);
    const n = pose.getJointCount();
    for (0..n) |j|
        loadState(&initial_states[j], pose.getJointMut(j));

    var o: usize = 0;
    for (0..2) |pass| {
        out.durations[pass] = a.getDuration();
        for (times) |t| {
            a.sample(t, &pose);
            for (0..n) |j| {
                storeState(pose.getJoint(j), &out.states[o]);
                o += 1;
            }
        }
        a.scaleJoints(scale);
    }

    out.is_looping = @intFromBool(a.isLooping());
    return saveBytes(allocator, a);
}

fn joltAnimation(allocator: Allocator, skeleton_desc: *const SkelDesc, animation_desc: *const SkelAnimDesc, initial_states: []const [7]f32, times: []const f32, scale: f32, out: *AnimationRun) ![]u8 {
    const size = jolt.jolt_skel_animation(skeleton_desc, animation_desc, @ptrCast(initial_states.ptr), times.ptr, @intCast(times.len), scale, @ptrCast(&out.states), &out.durations, &out.is_looping, &byte_buffer, byte_buffer.len);
    try std.testing.expect(size <= byte_buffer.len);
    return allocator.dupe(u8, byte_buffer[0..size]);
}

/// jolt_skel_pose on the Zolt side
fn zoltPose(allocator: Allocator, skeleton1_desc: *const SkelDesc, skeleton2_desc: *const SkelDesc, states: []const [7]f32, matrices: []const [16]f32, root_offset: *const [3]Real, out: *SkelPoseResult) !void {
    var skeleton1 = try makeSkeleton(allocator, skeleton1_desc, null);
    defer skeleton1.deinit();
    var skeleton2 = try makeSkeleton(allocator, skeleton2_desc, null);
    defer skeleton2.deinit();

    var pose: SkeletonPose = .init(allocator);
    defer pose.deinit();
    try pose.setSkeleton(skeleton1.get().?);
    const n = pose.getJointCount();
    out.joint_count = @intCast(n);
    for (0..n) |j| {
        loadState(&states[j], pose.getJointMut(j));
        pose.getJointMatrixMut(j).* = loadMat44(&matrices[j]);
    }
    pose.setRootOffset(RVec3.init(root_offset[0], root_offset[1], root_offset[2]));

    pose.calculateJointMatrices();
    for (0..n) |j|
        storeMat44(pose.getJointMatrix(j).*, &out.joint_matrices[j]);

    var local: [max_joints]Mat44 = undefined;
    pose.calculateLocalSpaceJointMatrices(&local);
    for (0..n) |j|
        storeMat44(local[j], &out.local_matrices[j]);

    for (pose.getJointMatricesMut().items, 0..) |*m, j|
        m.* = loadMat44(&matrices[j]);
    pose.calculateJointStates();
    for (0..n) |j|
        storeState(pose.getJoint(j), &out.joint_states[j]);

    const offset = pose.getRootOffset();
    out.root_offset = .{ offset.getX(), offset.getY(), offset.getZ() };

    try pose.setSkeleton(skeleton2.get().?);
    out.resized_joint_count = @intCast(pose.getJointCount());
    out.resized_matrix_count = @intCast(pose.getJointMatrices().len);
    for (0..pose.getJointCount()) |j|
        storeState(pose.getJoint(j), &out.resized_states[j]);

    for (matrices, &out.from_matrix_states) |*m, *o| {
        var state: SkeletalAnimation.JointState = .{};
        state.fromMatrix(loadMat44(m));
        storeState(&state, o);
    }
}

/// The can map joint function of mode 1 (SkelCanMapJoint): the table decides (0 no, 1 if the names are equal, 2 yes)
fn canMapJoint(desc: *const SkelMapperDesc, skeleton1: *const Skeleton, index1: usize, skeleton2: *const Skeleton, index2: usize) bool {
    const v = desc.can_map_table[index1 * skeleton2.getJointCount() + index2];
    return v == 2 or (v == 1 and std.mem.eql(u8, skeleton1.getJoint(index1).name, skeleton2.getJoint(index2).name));
}

/// jolt_skel_mapper on the Zolt side
fn zoltMapper(allocator: Allocator, desc: *const SkelMapperDesc, out: *SkelMapperResult) !void {
    var skeleton1 = try makeSkeleton(allocator, &desc.skeleton1, null);
    defer skeleton1.deinit();
    var skeleton2 = try makeSkeleton(allocator, &desc.skeleton2, null);
    defer skeleton2.deinit();
    const s1 = skeleton1.get().?;
    const s2 = skeleton2.get().?;
    const n1 = s1.getJointCount();
    const n2 = s2.getJointCount();

    var neutral1: [max_joints]Mat44 = undefined;
    var neutral2: [max_joints]Mat44 = undefined;
    for (0..n1) |j| neutral1[j] = loadMat44(&desc.neutral_pose1[j]);
    for (0..n2) |j| neutral2[j] = loadMat44(&desc.neutral_pose2[j]);

    var mapper = Ref(SkeletonMapper).init(try SkeletonMapper.create(allocator));
    defer mapper.deinit();
    const m = mapper.get().?;
    if (desc.can_map_mode == 0)
        try m.initialize(s1, neutral1[0..n1], s2, neutral2[0..n2], .{})
    else
        try m.initialize(s1, neutral1[0..n1], s2, neutral2[0..n2], .{ .can_map_joint = .init(desc, canMapJoint) });

    if ((desc.lock_mode & 1) != 0) {
        var locked: [max_joints]bool = undefined;
        for (0..n2) |j| locked[j] = desc.locked[j] != 0;
        try m.lockTranslations(s2, locked[0..n2], neutral2[0..n2]);
    }
    if ((desc.lock_mode & 2) != 0 and m.getMappings().len != 0)
        try m.lockAllTranslations(s2, neutral2[0..n2]);

    const mappings = m.getMappings();
    out.num_mappings = @intCast(mappings.len);
    for (mappings, 0..) |*mp, i| {
        out.mapping_joints[i] = .{ mp.joint_idx1, mp.joint_idx2 };
        storeMat44(mp.joint1_to2, &out.mapping_matrices[i][0]);
        storeMat44(mp.joint2_to1, &out.mapping_matrices[i][1]);
    }

    const chains = m.getChains();
    out.num_chains = @intCast(chains.len);
    for (chains, 0..) |*c, i| {
        out.chain_lengths[i] = .{ @intCast(c.joint_indices1.items.len), @intCast(c.joint_indices2.items.len) };
        @memcpy(out.chain_joints1[i][0..c.joint_indices1.items.len], c.joint_indices1.items);
        @memcpy(out.chain_joints2[i][0..c.joint_indices2.items.len], c.joint_indices2.items);
    }

    const unmapped = m.getUnmapped();
    out.num_unmapped = @intCast(unmapped.len);
    for (unmapped, 0..) |*u, i|
        out.unmapped[i] = .{ u.joint_idx, u.parent_joint_idx };

    const locked = m.getLockedTranslations();
    out.num_locked = @intCast(locked.len);
    for (locked, 0..) |*l, i| {
        out.locked_joints[i] = .{ l.joint_idx, l.parent_joint_idx };
        out.locked_translations[i] = .{ l.translation.getX(), l.translation.getY(), l.translation.getZ() };
    }

    for (0..n1 + 2) |i|
        out.mapped_joint_idx[i] = m.getMappedJointIdx(@as(i32, @intCast(i)) - 1);
    for (0..n2 + 2) |i|
        out.is_locked[i] = @intFromBool(m.isJointTranslationLocked(@as(i32, @intCast(i)) - 1));

    const sentinel = loadMat44(&desc.sentinel);
    var pose1: [max_joints]Mat44 = undefined;
    var pose2_local: [max_joints]Mat44 = undefined;
    var pose2_model: [max_joints]Mat44 = undefined;
    var out2: [max_joints]Mat44 = undefined;
    var out1: [max_joints]Mat44 = undefined;
    for (0..n1) |j| {
        pose1[j] = loadMat44(&desc.pose1_model_space[j]);
        out1[j] = sentinel;
    }
    for (0..n2) |j| {
        pose2_local[j] = loadMat44(&desc.pose2_local_space[j]);
        pose2_model[j] = loadMat44(&desc.pose2_model_space[j]);
        out2[j] = sentinel;
    }

    if (desc.map != 0)
        m.map(pose1[0..n1], pose2_local[0..n2], out2[0..n2]);
    m.mapReverse(pose2_model[0..n2], out1[0..n1]);

    for (0..n2) |j| storeMat44(out2[j], &out.pose2_model_space[j]);
    for (0..n1) |j| storeMat44(out1[j], &out.pose1_model_space[j]);
}

/// Compare two result structs field by field (fw.sameValue: bits, NaNs are equal), report the fields that differ
fn checkFields(checker: *Checker, iteration: usize, zolt_result: anytype, jolt_result: @TypeOf(zolt_result)) void {
    var equal = true;
    inline for (@typeInfo(@TypeOf(zolt_result)).@"struct".fields) |field| {
        if (!fw.sameValue(@field(zolt_result, field.name), @field(jolt_result, field.name))) {
            if (checker.mismatches < 5)
                std.debug.print("{s}: iteration {d}: {s} differs\n  zolt: {any}\n  jolt: {any}\n", .{ checker.name, iteration, field.name, @field(zolt_result, field.name), @field(jolt_result, field.name) });
            equal = false;
        }
    }
    checker.check(iteration, equal, true);
}

// ---------------------------------------------------------------------------------------------------------------------
// The input generator

const Gen = struct {
    rng: fw.Rng,

    fn index(self: *Gen, n: usize) usize {
        return self.rng.next() % n;
    }

    fn oneIn(self: *Gen, n: u32) bool {
        return self.rng.next() % n == 0;
    }

    fn float(self: *Gen, min: f32, max: f32) f32 {
        return self.rng.float(min, max);
    }

    fn vec3(self: *Gen, min: f32, max: f32) Vec3 {
        return Vec3.init(self.float(min, max), self.float(min, max), self.float(min, max));
    }

    fn unitVec3(self: *Gen) Vec3 {
        while (true) {
            const v = self.vec3(-1, 1);
            if (v.lengthSq() > 0.01) return v.normalized();
        }
    }

    /// A random unit quaternion, sometimes a special one (identity, -identity, a half turn with w = 0)
    fn quat(self: *Gen) Quat {
        return switch (self.index(10)) {
            0 => Quat.identity(),
            1 => Quat.init(0, 0, 0, -1),
            2 => blk: {
                const axis = self.unitVec3();
                break :blk Quat.init(axis.getX(), axis.getY(), axis.getZ(), 0);
            },
            else => blk: {
                while (true) {
                    const v = Vec4.init(self.float(-1, 1), self.float(-1, 1), self.float(-1, 1), self.float(-1, 1));
                    if (v.lengthSq() > 0.01) break :blk Quat.fromVec4(v.normalized());
                }
            },
        };
    }

    /// A rotation of at most `max_angle` radians
    fn smallQuat(self: *Gen, max_angle: f32) Quat {
        return Quat.rotation(self.unitVec3(), self.float(-max_angle, max_angle));
    }

    /// A random affine matrix: rotation and translation, sometimes scaled (not orthonormal)
    fn affine(self: *Gen) Mat44 {
        const m = Mat44.rotationTranslation(self.quat(), self.vec3(-5, 5));
        return switch (self.index(4)) {
            0 => m.mul(Mat44.scaleVec3(Vec3.init(self.float(0.5, 2), self.float(0.5, 2), self.float(0.5, 2)))),
            1 => m.mul(Mat44.scale(self.float(0.2, 3))),
            else => m,
        };
    }

    fn state(self: *Gen) [7]f32 {
        const q = self.quat();
        const t = self.vec3(-2, 2);
        return .{ q.getX(), q.getY(), q.getZ(), q.getW(), t.getX(), t.getY(), t.getZ() };
    }

    /// A name from a small pool, so that names repeat and parent names can be found; sometimes empty
    fn poolName(self: *Gen, out: *[max_name]u8, out_len: *u32) void {
        if (self.oneIn(20)) {
            out_len.* = 0;
            return;
        }
        setName(out, out_len, "j{d}", .{self.index(48)});
    }

    /// A random AddJoint script (see the file comment)
    fn script(self: *Gen) SkelDesc {
        var desc = std.mem.zeroes(SkelDesc);
        desc.num_joints = @intCast(self.index(max_joints + 1));
        desc.calculate_parent_joint_indices = @intFromBool(self.oneIn(2));
        for (desc.joints[0..desc.num_joints], 0..) |*d, j| {
            d.kind = @intCast(self.index(3));
            if (self.oneIn(3))
                self.poolName(&d.name, &d.name_len)
            else
                setName(&d.name, &d.name_len, "j{d}", .{j});
            switch (d.kind) {
                0 => switch (self.index(4)) {
                    0 => self.poolName(&d.parent_name, &d.parent_name_len),
                    1 => setName(&d.parent_name, &d.parent_name_len, "j{d}", .{j + 1 + self.index(4)}), // A later joint
                    2 => setName(&d.parent_name, &d.parent_name_len, "x{d}", .{self.index(10)}), // Not in the skeleton
                    else => setName(&d.parent_name, &d.parent_name_len, "j{d}", .{self.index(j + 1)}), // Probably earlier, maybe itself
                },
                1 => {},
                else => d.parent_index = if (j > 0 and !self.oneIn(5)) @intCast(self.index(j)) else -@as(i32, @intCast(1 + self.index(3))),
            }
        }
        return desc;
    }

    /// Parents of a tree with n joints in parent first order: deep chains, branches and a few extra roots
    fn treeParents(self: *Gen, n: usize, out: []i32) void {
        for (out[0..n], 0..) |*p, j|
            p.* = if (j == 0 or self.oneIn(15)) -1 else if (self.oneIn(2)) @intCast(j - 1) else @intCast(self.index(j));
    }

    /// A random permutation of 0 .. n - 1 (new index of each joint), the identity when `shuffle` is false
    fn permutation(self: *Gen, n: usize, shuffle: bool, out: []usize) void {
        for (out[0..n], 0..) |*p, i| p.* = i;
        if (shuffle) {
            var i = n;
            while (i > 1) {
                i -= 1;
                std.mem.swap(usize, &out[i], &out[self.index(i + 1)]);
            }
        }
    }

    /// The skeleton description of a tree: joint `order[j]` of the description is joint j of the tree, it is named
    /// `names[j]`. Parents after their children (shuffled order) can only be added by name.
    fn treeSkeleton(self: *Gen, n: usize, parents: []const i32, order: []const usize, names: []const [max_name]u8, name_lens: []const u32) SkelDesc {
        var desc = std.mem.zeroes(SkelDesc);
        desc.num_joints = @intCast(n);
        var ordered = true;
        for (0..n) |j| {
            if (parents[j] >= 0 and order[@intCast(parents[j])] > order[j]) ordered = false;
        }
        const by_name = !ordered or self.oneIn(2);
        desc.calculate_parent_joint_indices = @intFromBool(by_name);
        for (0..n) |j| {
            const d = &desc.joints[order[j]];
            d.name = names[j];
            d.name_len = name_lens[j];
            if (by_name) {
                d.kind = 0;
                if (parents[j] >= 0) {
                    d.parent_name = names[@intCast(parents[j])];
                    d.parent_name_len = name_lens[@intCast(parents[j])];
                }
            } else {
                d.kind = 2;
                d.parent_index = if (parents[j] >= 0) @intCast(order[@intCast(parents[j])]) else -1;
            }
        }
        return desc;
    }

    /// An ordered skeleton with n joints named j0, j1, ...
    fn tree(self: *Gen, n: usize) SkelDesc {
        var parents: [max_joints]i32 = undefined;
        self.treeParents(n, &parents);
        var order: [max_joints]usize = undefined;
        self.permutation(n, false, &order);
        var names: [max_joints][max_name]u8 = undefined;
        var name_lens: [max_joints]u32 = undefined;
        for (0..n) |j| setName(&names[j], &name_lens[j], "j{d}", .{j});
        return self.treeSkeleton(n, &parents, &order, &names, &name_lens);
    }

    /// A random animation of the joints of `skeleton` (see the file comment). For sampling every animated joint has a
    /// keyframe and is a joint of the skeleton, otherwise joints without keyframes and other names occur.
    fn animation(self: *Gen, skeleton: *const SkelDesc, for_sampling: bool, sorted: bool) SkelAnimDesc {
        var desc = std.mem.zeroes(SkelAnimDesc);
        desc.is_looping = @intFromBool(!self.oneIn(3));
        desc.num_joints = @intCast(if (for_sampling) 1 + self.index(max_animated_joints) else self.index(max_animated_joints + 1));
        const start: f32 = switch (self.index(8)) {
            0 => 0.0,
            1 => -1.5, // Negative keyframe times
            2 => self.float(-0.5, 0.0),
            else => self.float(0.0, 0.5),
        };
        const step: f32 = if (self.oneIn(4)) self.float(1, 5) else self.float(0.02, 0.5);
        for (desc.joints[0..desc.num_joints]) |*d| {
            if (skeleton.num_joints > 0 and (for_sampling or !self.oneIn(8))) {
                const j = &skeleton.joints[self.index(skeleton.num_joints)];
                d.name = j.name;
                d.name_len = j.name_len;
            } else setName(&d.name, &d.name_len, "y{d}", .{self.index(10)});
            d.num_keyframes = @intCast(if (for_sampling) 1 + self.index(max_keyframes) else self.index(max_keyframes + 1));
            var t = start + (if (self.oneIn(2)) 0.0 else self.float(0, step));
            var prev = self.quat();
            for (d.keyframes[0..d.num_keyframes]) |*k| {
                k.time = t;
                t += if (sorted and self.oneIn(8)) 0.0 else self.float(0.25 * step, step); // Unsorted: distinct times, so that fractions are finite

                const q = switch (self.index(8)) {
                    0 => prev, // Equal rotations
                    1 => prev.mul(self.smallQuat(0.01)).normalized(), // Close: the LERP branch of SLERP
                    2 => prev.negate(), // The other hemisphere: a negative dot product
                    else => self.quat(),
                };
                prev = q;
                k.rotation = .{ q.getX(), q.getY(), q.getZ(), q.getW() };
                const tr = self.vec3(-2, 2);
                k.translation = .{ tr.getX(), tr.getY(), tr.getZ() };
            }
            if (!sorted) {
                // Shuffle the (distinct) times of the keyframes
                var i: usize = d.num_keyframes;
                while (i > 1) {
                    i -= 1;
                    std.mem.swap(f32, &d.keyframes[i].time, &d.keyframes[self.index(i + 1)].time);
                }
            }
        }
        return desc;
    }

    /// Sample times (see the file comment): negative times only when `allow_negative`
    fn times(self: *Gen, desc: *const SkelAnimDesc, out: []f32, allow_negative: bool) void {
        const first = &desc.joints[0];
        const duration = if (desc.num_joints > 0 and first.num_keyframes > 0) first.keyframes[first.num_keyframes - 1].time else 0.0;
        const range: f32 = if (duration > 1.0) duration else 1.0;
        for (out) |*t| {
            const j = &desc.joints[self.index(desc.num_joints)];
            const key = if (j.num_keyframes > 0) j.keyframes[self.index(j.num_keyframes)].time else 0.0;
            var v: f32 = switch (self.index(10)) {
                0 => key, // On a keyframe
                1 => std.math.nextAfter(f32, key, std.math.inf(f32)), // One ulp after a keyframe
                2 => std.math.nextAfter(f32, key, -std.math.inf(f32)), // One ulp before a keyframe
                3 => 0.0,
                4 => duration * @as(f32, @floatFromInt(1 + self.index(4))), // A multiple of the duration: wraps to 0
                5 => key + duration * @as(f32, @floatFromInt(1 + self.index(3))), // A keyframe in a later loop
                6 => self.float(0, 4 * range), // Beyond the end
                7 => self.float(100, 10000), // Far beyond
                else => self.float(0, 1.2 * range),
            };
            if (allow_negative) {
                if (self.oneIn(3)) v = -self.float(0, 3 * range);
            } else if (!(v >= 0.0)) v = self.float(0, range); // Jolt asserts that the time is not negative
            t.* = v;
        }
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// The tests

test "Skeleton parity: struct layout" {
    var sizes: [6]u32 = undefined;
    jolt.jolt_skel_struct_sizes(&sizes);
    try std.testing.expectEqual([6]u32{ @sizeOf(SkelDesc), @sizeOf(SkelResult), @sizeOf(SkelAnimDesc), @sizeOf(SkelPoseResult), @sizeOf(SkelMapperDesc), @sizeOf(SkelMapperResult) }, sizes);
}

test "Skeleton parity: AddJoint, GetJointIndex, CalculateParentJointIndices, AreJointsCorrectlyOrdered, SaveBinaryState" {
    const allocator = std.testing.allocator;
    var result_checker: Checker = .{ .name = "Skeleton: AddJoint, parent indices, GetJointCount, AreJointsCorrectlyOrdered" };
    var index_checker: Checker = .{ .name = "Skeleton::GetJointIndex" };
    var save_checker: Checker = .{ .name = "Skeleton::SaveBinaryState" };
    var g: Gen = .{ .rng = .{ .state = 0x5E1E7001 } };
    var num_unordered: u32 = 0;
    var num_found: u32 = 0;
    var num_not_found: u32 = 0;
    for (0..4000) |iteration| {
        const desc = g.script();
        var queries: [max_queries]SkelName = undefined;
        for (&queries) |*q| {
            if (desc.num_joints > 0 and g.oneIn(2)) {
                const j = &desc.joints[g.index(desc.num_joints)];
                q.name = j.name;
                q.len = j.name_len;
            } else g.poolName(&q.name, &q.len);
        }

        var zolt_result = std.mem.zeroes(SkelResult);
        var jolt_result = std.mem.zeroes(SkelResult);
        var zolt_indices: [max_queries]i32 = undefined;
        var jolt_indices: [max_queries]i32 = undefined;
        const zolt_bytes = try zoltSkeleton(allocator, &desc, &queries, &zolt_indices, &zolt_result);
        defer allocator.free(zolt_bytes);
        const size = jolt.jolt_skel_skeleton(&desc, &queries, max_queries, &jolt_indices, &jolt_result, &byte_buffer, byte_buffer.len);
        try std.testing.expect(size <= byte_buffer.len);

        checkFields(&result_checker, iteration, zolt_result, jolt_result);
        index_checker.check(iteration, zolt_indices, jolt_indices);
        save_checker.check(iteration, std.mem.eql(u8, zolt_bytes, byte_buffer[0..size]), true);

        // What the inputs reach
        if (jolt_result.correctly_ordered == 0) num_unordered += 1;
        for (jolt_indices) |i| {
            if (i >= 0) num_found += 1 else num_not_found += 1;
        }
    }
    try fw.finishAll(&.{ &result_checker, &index_checker, &save_checker });
    try std.testing.expect(num_unordered > 1000);
    try std.testing.expect(num_found > 10000 and num_not_found > 5000);
}

test "Skeleton parity: sRestoreFromBinaryState" {
    const allocator = std.testing.allocator;
    var resave_checker: Checker = .{ .name = "Skeleton::sRestoreFromBinaryState of the other side's bytes" };
    var error_checker: Checker = .{ .name = "Skeleton::sRestoreFromBinaryState of truncated streams" };
    var g: Gen = .{ .rng = .{ .state = 0x5E1E7002 } };
    var queries: [1]SkelName = undefined;
    var indices: [1]i32 = undefined;
    for (0..2000) |iteration| {
        const desc = g.script();
        var zolt_result = std.mem.zeroes(SkelResult);
        var jolt_result = std.mem.zeroes(SkelResult);
        const zolt_bytes = try zoltSkeleton(allocator, &desc, queries[0..0], indices[0..0], &zolt_result);
        defer allocator.free(zolt_bytes);
        const size = jolt.jolt_skel_skeleton(&desc, &queries, 0, &indices, &jolt_result, &byte_buffer, byte_buffer.len);
        const jolt_bytes = try allocator.dupe(u8, byte_buffer[0..size]);
        defer allocator.free(jolt_bytes);

        // Each side restores the other side's bytes and saves them again
        {
            var z: Resaved = .{};
            defer z.deinit(allocator);
            try zoltSkeletonResave(allocator, jolt_bytes, &z);
            resave_checker.check(iteration, z.err == null and std.mem.eql(u8, z.bytes.items, jolt_bytes), true);
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltSkeletonResave(allocator, zolt_bytes, &j);
            resave_checker.check(iteration, j.err == null and std.mem.eql(u8, j.bytes.items, zolt_bytes), true);
        }

        // Truncated streams: the same error on both sides
        {
            const len = g.index(zolt_bytes.len);
            var z: Resaved = .{};
            defer z.deinit(allocator);
            try zoltSkeletonResave(allocator, zolt_bytes[0..len], &z);
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltSkeletonResave(allocator, zolt_bytes[0..len], &j);
            if (!z.eql(&j) and error_checker.mismatches < 5)
                std.debug.print("truncated at {d}: zolt '{s}' jolt '{s}'\n", .{ len, z.err orelse "(ok)", j.err orelse "(ok)" });
            error_checker.check(.{ iteration, len }, z.eql(&j), true);
            try std.testing.expectEqualStrings("Failed to read skeleton from stream", z.err.?);
        }
    }
    try fw.finishAll(&.{ &resave_checker, &error_checker });
}

/// Sample + ScaleJoints + SaveBinaryState of random animations on both sides
fn runAnimations(seed: u32, num_cases: usize, allow_negative_times: bool, sorted: bool) !void {
    const allocator = std.testing.allocator;
    var sample_checker: Checker = .{ .name = "SkeletalAnimation::Sample (and ScaleJoints)" };
    var duration_checker: Checker = .{ .name = "SkeletalAnimation::GetDuration, IsLooping" };
    var save_checker: Checker = .{ .name = "SkeletalAnimation::SaveBinaryState" };
    var g: Gen = .{ .rng = .{ .state = seed } };
    const zolt_run = try allocator.create(AnimationRun);
    defer allocator.destroy(zolt_run);
    const jolt_run = try allocator.create(AnimationRun);
    defer allocator.destroy(jolt_run);

    // What the inputs reach
    var num_before: u32 = 0;
    var num_after: u32 = 0;
    var num_between: u32 = 0;
    var num_wrapped: u32 = 0;
    var num_single: u32 = 0;
    for (0..num_cases) |iteration| {
        const skeleton = g.tree(1 + g.index(max_joints));
        const anim = g.animation(&skeleton, true, sorted);
        var initial: [max_joints][7]f32 = undefined;
        for (initial[0..skeleton.num_joints]) |*s| s.* = g.state();
        const num_times = 1 + g.index(max_times);
        var times: [max_times]f32 = undefined;
        g.times(&anim, times[0..num_times], allow_negative_times);
        const scale: f32 = switch (g.index(6)) {
            0 => 1.0,
            1 => 0.0,
            2 => -1.0,
            else => g.float(0.1, 3),
        };

        zolt_run.* = std.mem.zeroes(AnimationRun);
        jolt_run.* = std.mem.zeroes(AnimationRun);
        const zolt_bytes = try zoltAnimation(allocator, &skeleton, &anim, initial[0..skeleton.num_joints], times[0..num_times], scale, zolt_run);
        defer allocator.free(zolt_bytes);
        const jolt_bytes = try joltAnimation(allocator, &skeleton, &anim, initial[0..skeleton.num_joints], times[0..num_times], scale, jolt_run);
        defer allocator.free(jolt_bytes);

        // The states after every sample, the first difference is printed
        const num_states = 2 * num_times * skeleton.num_joints;
        var same_states = true;
        for (zolt_run.states[0..num_states], jolt_run.states[0..num_states], 0..) |z, j, i| {
            if (!fw.sameValue(z, j)) {
                if (sample_checker.mismatches < 5)
                    std.debug.print("{s}: iteration {d}: first difference in state {d} (time {d}, joint {d})\n  zolt: {any}\n  jolt: {any}\n", .{ sample_checker.name, iteration, i, times[(i / skeleton.num_joints) % num_times], i % skeleton.num_joints, z, j });
                same_states = false;
                break;
            }
        }
        sample_checker.check(iteration, same_states, true);
        duration_checker.check(iteration, zolt_run.durations, jolt_run.durations);
        duration_checker.check(iteration, zolt_run.is_looping, jolt_run.is_looping);
        save_checker.check(iteration, std.mem.eql(u8, zolt_bytes, jolt_bytes), true);

        // Classify the samples (the times as Sample sees them)
        const duration = jolt_run.durations[0];
        for (times[0..num_times]) |time| {
            const t = if (duration > 0.0 and anim.is_looping != 0) @rem(time, duration) else time;
            if (t != time) num_wrapped += 1;
            for (anim.joints[0..anim.num_joints]) |*aj| {
                if (aj.num_keyframes == 1) num_single += 1;
                if (!(aj.keyframes[0].time < t))
                    num_before += 1
                else if (aj.keyframes[aj.num_keyframes - 1].time < t)
                    num_after += 1
                else
                    num_between += 1;
            }
        }
    }
    try fw.finishAll(&.{ &sample_checker, &duration_checker, &save_checker });
    if (sorted and !allow_negative_times) {
        try std.testing.expect(num_before > num_cases and num_after > num_cases and num_between > num_cases);
        try std.testing.expect(num_wrapped > num_cases / 2 and num_single > num_cases / 2);
    }
}

test "SkeletalAnimation parity: Sample, GetDuration, ScaleJoints, SaveBinaryState" {
    try runAnimations(0x5E1E7003, 4000, false, true);
}

test "SkeletalAnimation parity: sRestoreFromBinaryState" {
    const allocator = std.testing.allocator;
    var resave_checker: Checker = .{ .name = "SkeletalAnimation::sRestoreFromBinaryState of the other side's bytes" };
    var truncated_checker: Checker = .{ .name = "SkeletalAnimation::sRestoreFromBinaryState of truncated streams" };
    var g: Gen = .{ .rng = .{ .state = 0x5E1E7004 } };
    const zolt_run = try allocator.create(AnimationRun);
    defer allocator.destroy(zolt_run);
    const jolt_run = try allocator.create(AnimationRun);
    defer allocator.destroy(jolt_run);
    var num_partial_floats: u32 = 0;
    for (0..2000) |iteration| {
        const skeleton = g.tree(1 + g.index(8));
        const anim = g.animation(&skeleton, false, true); // Joints without keyframes, names that are not in the skeleton
        var initial: [max_joints][7]f32 = undefined;
        for (initial[0..skeleton.num_joints]) |*s| s.* = g.state();
        const zolt_bytes = try zoltAnimation(allocator, &skeleton, &anim, initial[0..skeleton.num_joints], &.{}, 1.0, zolt_run); // No samples
        defer allocator.free(zolt_bytes);
        const jolt_bytes = try joltAnimation(allocator, &skeleton, &anim, initial[0..skeleton.num_joints], &.{}, 1.0, jolt_run);
        defer allocator.free(jolt_bytes);

        // Each side restores the other side's bytes and saves them again
        {
            var z: Resaved = .{};
            defer z.deinit(allocator);
            try zoltAnimationResave(allocator, jolt_bytes, &z);
            resave_checker.check(iteration, std.mem.eql(u8, z.bytes.items, jolt_bytes), true);
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltAnimationResave(allocator, zolt_bytes, &j);
            resave_checker.check(iteration, std.mem.eql(u8, j.bytes.items, zolt_bytes), true);
        }

        // Truncated streams: Jolt does not check the stream, both sides save what they read
        for (0..4) |_| {
            const len = g.index(zolt_bytes.len);
            var z: Resaved = .{};
            defer z.deinit(allocator);
            try zoltAnimationResave(allocator, zolt_bytes[0..len], &z);
            var j: Resaved = .{};
            defer j.deinit(allocator);
            try joltAnimationResave(allocator, zolt_bytes[0..len], &j);
            truncated_checker.check(.{ iteration, len }, z.eql(&j), true);
            if (len >= 4 and len + 1 < zolt_bytes.len and z.bytes.items.len > len) num_partial_floats += 1;
        }
    }
    try fw.finishAll(&.{ &resave_checker, &truncated_checker });
    try std.testing.expect(num_partial_floats > 1000);
}

/// SkeletonPose on both sides: `unordered` allows parents after their children (release only)
fn runPoses(seed: u32, num_cases: usize, unordered: bool) !void {
    const allocator = std.testing.allocator;
    var checker: Checker = .{ .name = "SkeletonPose: SetSkeleton, CalculateJointMatrices, CalculateLocalSpaceJointMatrices, CalculateJointStates, root offset" };
    var g: Gen = .{ .rng = .{ .state = seed } };
    var zolt_result: SkelPoseResult = undefined;
    var jolt_result: SkelPoseResult = undefined;
    var num_parents_after: u32 = 0;
    for (0..num_cases) |iteration| {
        const skeleton1 = if (unordered) g.script() else g.tree(g.index(max_joints + 1));
        const skeleton2 = if (g.oneIn(4)) skeleton1 else g.tree(g.index(max_joints + 1));
        var states: [max_joints][7]f32 = undefined;
        var matrices: [max_joints][16]f32 = undefined;
        for (0..max_joints) |j| {
            states[j] = g.state();
            storeMat44(g.affine(), &matrices[j]);
        }
        const far: Real = if (g.oneIn(3)) 1.0e7 else 1.0;
        const root_offset = [3]Real{ far * g.float(-1, 1), far * g.float(-1, 1), g.float(-10, 10) };

        zolt_result = std.mem.zeroes(SkelPoseResult);
        jolt_result = std.mem.zeroes(SkelPoseResult);
        try zoltPose(allocator, &skeleton1, &skeleton2, &states, &matrices, &root_offset, &zolt_result);
        jolt.jolt_skel_pose(&skeleton1, &skeleton2, @ptrCast(&states), @ptrCast(&matrices), &root_offset, &jolt_result);
        checkFields(&checker, iteration, zolt_result, jolt_result);

        if (unordered) {
            var s = try makeSkeleton(allocator, &skeleton1, null);
            defer s.deinit();
            if (!s.get().?.areJointsCorrectlyOrdered()) num_parents_after += 1;
        }
    }
    try checker.finish();
    if (unordered) try std.testing.expect(num_parents_after > num_cases / 4);
}

test "SkeletonPose parity" {
    try runPoses(0x5E1E7005, 3000, false);
}

/// A random mapper case (see the file comment). `unordered` shuffles the joint order of both skeletons (parents after
/// their children), `allow_unordered_calls` then still calls Map and the lock functions (release only).
fn mapperDesc(g: *Gen, desc: *SkelMapperDesc, unordered: bool, allow_unordered_calls: bool) void {
    desc.* = std.mem.zeroes(SkelMapperDesc);

    // The tree of skeleton 2 in parent first order, its neutral pose: random local transforms, model space through the
    // hierarchy
    const n2 = 2 + g.index(max_joints - 1);
    var parents2: [max_joints]i32 = undefined;
    g.treeParents(n2, &parents2);
    var local2: [max_joints]Mat44 = undefined;
    var model2: [max_joints]Mat44 = undefined;
    for (0..n2) |j| {
        local2[j] = Mat44.rotationTranslation(g.smallQuat(1.0), g.vec3(-1, 1));
        model2[j] = if (parents2[j] >= 0) model2[@intCast(parents2[j])].mul(local2[j]) else local2[j];
    }

    // Skeleton 1: a subset of skeleton 2 in the same order, the parent of a joint is its nearest ancestor in the subset
    const keep_percentage: usize = switch (g.index(5)) {
        0 => 15,
        1 => 30,
        2 => 50,
        3 => 75,
        else => 100,
    };
    var index1: [max_joints]i32 = @splat(-1); // The joint of skeleton 1 of each joint of skeleton 2
    var joints1: [max_joints]usize = undefined; // The joint of skeleton 2 of each joint of skeleton 1
    var n1: usize = 0;
    for (0..n2) |j| {
        if (g.index(100) < keep_percentage) {
            index1[j] = @intCast(n1);
            joints1[n1] = j;
            n1 += 1;
        }
    }
    if (n1 == 0) {
        index1[0] = 0;
        joints1[0] = 0;
        n1 = 1;
    }
    var parents1: [max_joints]i32 = undefined;
    for (0..n1) |k| {
        var a = parents2[joints1[k]];
        while (a >= 0 and index1[@intCast(a)] < 0) a = parents2[@intCast(a)];
        parents1[k] = if (a >= 0) index1[@intCast(a)] else -1;
    }

    // Names: the same in both skeletons, sometimes a ragdoll joint that skeleton 2 does not have. The neutral pose of
    // skeleton 1 is the one of skeleton 2, sometimes perturbed.
    var names2: [max_joints][max_name]u8 = undefined;
    var name_lens2: [max_joints]u32 = undefined;
    for (0..n2) |j| setName(&names2[j], &name_lens2[j], "j{d}", .{j});
    var names1: [max_joints][max_name]u8 = undefined;
    var name_lens1: [max_joints]u32 = undefined;
    var model1: [max_joints]Mat44 = undefined;
    const perturb = g.oneIn(2);
    for (0..n1) |k| {
        if (g.oneIn(15))
            setName(&names1[k], &name_lens1[k], "x{d}", .{k})
        else {
            names1[k] = names2[joints1[k]];
            name_lens1[k] = name_lens2[joints1[k]];
        }
        model1[k] = model2[joints1[k]];
        if (perturb and g.oneIn(2))
            model1[k] = model1[k].mul(Mat44.rotationTranslation(g.smallQuat(0.2), g.vec3(-0.05, 0.05)));
    }

    // The joint orders
    var order1: [max_joints]usize = undefined;
    var order2: [max_joints]usize = undefined;
    g.permutation(n1, unordered, &order1);
    g.permutation(n2, unordered, &order2);
    desc.skeleton1 = g.treeSkeleton(n1, &parents1, &order1, &names1, &name_lens1);
    desc.skeleton2 = g.treeSkeleton(n2, &parents2, &order2, &names2, &name_lens2);

    // The poses: perturbed neutral poses (all affine)
    for (0..n1) |k| {
        storeMat44(model1[k], &desc.neutral_pose1[order1[k]]);
        storeMat44(model1[k].mul(Mat44.rotationTranslation(g.smallQuat(0.5), g.vec3(-0.2, 0.2))), &desc.pose1_model_space[order1[k]]);
    }
    for (0..n2) |j| {
        storeMat44(model2[j], &desc.neutral_pose2[order2[j]]);
        storeMat44(local2[j].mul(Mat44.rotationTranslation(g.smallQuat(0.5), g.vec3(-0.2, 0.2))), &desc.pose2_local_space[order2[j]]);
        storeMat44(model2[j].mul(Mat44.rotationTranslation(g.smallQuat(0.5), g.vec3(-0.2, 0.2))), &desc.pose2_model_space[order2[j]]);
    }
    storeMat44(Mat44.rotationTranslation(Quat.rotation(Vec3.init(0.6, 0.8, 0), 0.3), Vec3.init(5, 6, 7)), &desc.sentinel);

    // Initialize with the default argument or with the table: mostly "if the names are equal", sometimes no or yes
    desc.can_map_mode = @intFromBool(g.oneIn(3));
    if (desc.can_map_mode != 0) {
        const no_one_in: u32 = if (g.oneIn(2)) 4 else 50;
        for (0..n1) |k| {
            for (0..n2) |j| {
                desc.can_map_table[k * n2 + j] = if (g.oneIn(no_one_in)) 0 else if (g.oneIn(60)) 2 else 1;
            }
        }
    }

    // Map and the locks (Jolt asserts that skeleton 2 is ordered, Map reads index -1 for a locked joint without a parent)
    const calls_allowed = !unordered or allow_unordered_calls;
    desc.map = @intFromBool(calls_allowed and !g.oneIn(8));
    desc.lock_mode = if (calls_allowed) @intCast(g.index(4)) else 0;
    for (0..n2) |j| {
        const has_parent = parents2[j] >= 0;
        desc.locked[order2[j]] = @intFromBool(g.oneIn(3) and (has_parent or desc.map == 0));
    }
}

fn runMappers(seed: u32, num_cases: usize, unordered: bool, allow_unordered_calls: bool) !void {
    const allocator = std.testing.allocator;
    var checker: Checker = .{ .name = "SkeletonMapper: Initialize, LockTranslations, LockAllTranslations, Map, MapReverse, GetMappedJointIdx, IsJointTranslationLocked" };
    var g: Gen = .{ .rng = .{ .state = seed } };
    const desc = try allocator.create(SkelMapperDesc);
    defer allocator.destroy(desc);
    const zolt_result = try allocator.create(SkelMapperResult);
    defer allocator.destroy(zolt_result);
    const jolt_result = try allocator.create(SkelMapperResult);
    defer allocator.destroy(jolt_result);

    // What the inputs reach
    var num_chains: u32 = 0;
    var num_long_chains: u32 = 0;
    var num_unmapped: u32 = 0;
    var num_locked: u32 = 0;
    var num_rotated_chains: u32 = 0;
    var num_non_identity: u32 = 0;
    var num_unmapped_ragdoll_joints: u32 = 0;
    for (0..num_cases) |iteration| {
        mapperDesc(&g, desc, unordered, allow_unordered_calls);
        zolt_result.* = std.mem.zeroes(SkelMapperResult);
        jolt_result.* = std.mem.zeroes(SkelMapperResult);
        try zoltMapper(allocator, desc, zolt_result);
        jolt.jolt_skel_mapper(desc, jolt_result);
        checkFields(&checker, iteration, zolt_result.*, jolt_result.*);

        if (jolt_result.num_chains > 0) num_chains += 1;
        for (jolt_result.chain_lengths[0..jolt_result.num_chains]) |l| {
            if (l[1] > 3) num_long_chains += 1;
            if (desc.map != 0) num_rotated_chains += 1;
        }
        if (jolt_result.num_unmapped > 0) num_unmapped += 1;
        if (jolt_result.num_locked > 0) num_locked += 1;
        for (jolt_result.mapping_matrices[0..jolt_result.num_mappings]) |m| {
            if (!loadMat44(&m[0]).eql(Mat44.identity())) num_non_identity += 1;
        }
        if (jolt_result.num_mappings < desc.skeleton1.num_joints) num_unmapped_ragdoll_joints += 1;
    }
    try checker.finish();
    const calls = !unordered or allow_unordered_calls;
    // Jolt only finds a chain from an earlier to a later mapping, so a shuffled skeleton 1 has about half the chains
    const chain_divisor: usize = if (unordered) 2 else 1;
    const reached = num_chains > num_cases / 3 / chain_divisor and num_long_chains > num_cases / 4 / chain_divisor and num_unmapped > num_cases / 2 and
        num_non_identity > num_cases and num_unmapped_ragdoll_joints > num_cases / 4 and
        (!calls or (num_locked > num_cases / 2 and num_rotated_chains > num_cases / 2));
    if (!reached) {
        std.debug.print("SkeletonMapper parity: the inputs reach too few cases: chains {d}, long chains {d}, unmapped {d}, non identity {d}, unmapped ragdoll joints {d}, locked {d}, rotated chains {d} (of {d})\n", .{ num_chains, num_long_chains, num_unmapped, num_non_identity, num_unmapped_ragdoll_joints, num_locked, num_rotated_chains, num_cases });
        return error.TestUnexpectedResult;
    }
}

test "SkeletonMapper parity" {
    try runMappers(0x5E1E7006, 3000, false, false);
}

test "SkeletonMapper parity: Initialize and MapReverse with parents after their children" {
    try runMappers(0x5E1E7007, 1000, true, false);
}

test "Skeleton parity (release only): Sample at negative times and of unsorted keyframes, SkeletonPose and SkeletonMapper with parents after their children" {
    // Jolt asserts on these inputs, Zolt checks the same asserts in safe builds; Jolt's release build continues
    if (zolt.Core.enable_asserts) return error.SkipZigTest;
    try runAnimations(0x5E1E7008, 1500, true, true);
    try runAnimations(0x5E1E7009, 1500, false, false);
    try runPoses(0x5E1E700A, 1500, true);
    try runMappers(0x5E1E700B, 1500, true, true);
}
