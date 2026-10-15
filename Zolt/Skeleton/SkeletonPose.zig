//! Port of: Jolt/Skeleton/SkeletonPose.h, Jolt/Skeleton/SkeletonPose.cpp
//! Status: complete
//!
//! JPH_DEBUG_RENDERER (DrawSettings, Draw): TODO(debug_renderer). JPH_OVERRIDE_NEW_DELETE: dropped.
//!
//! CharacterVehicleArchitecture E2: a value struct that owns its arrays. `init(allocator)` stores the allocator that
//! allocates the joint arrays, `deinit` releases the skeleton and frees them. Jolt's implicit copy constructor (a copy
//! that references the same skeleton, used by PerformanceTest's RagdollScene) is `clone`.
//! - `setSkeleton` resizes the arrays, so it returns `Allocator.Error`; nothing changes when it fails. Like Jolt's
//!   `resize`, new joint states are default constructed (identity) and new joint matrices are left uninitialized.
//! - The non-const GetJoints / GetJointMatrices are `getJointsMut` / `getJointMatricesMut` (the arrays themselves,
//!   allocated with `allocator`).
//! - Jolt's "Joints must be ordered: parents first" assert is checked in safe builds only (`if (Core.enable_asserts)`):
//!   Jolt's release build continues with a matrix that was computed earlier, and so do Zolt's optimized builds.
//! - Where Jolt has undefined behavior Zolt panics: calculateJointMatrices and calculateJointStates of a pose that has
//!   joint states but no skeleton (Jolt dereferences a null mSkeleton; reachable by adding joint states through
//!   getJointsMut before setSkeleton).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../Core/Core.zig");
const RefConst = @import("../Core/Reference.zig").RefConst;
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const RVec3 = @import("../Math/Real.zig").RVec3;
const Skeleton = @import("Skeleton.zig").Skeleton;
const SkeletalAnimation = @import("SkeletalAnimation.zig").SkeletalAnimation;

/// Instance of a skeleton, contains the pose the current skeleton is in
pub const SkeletonPose = struct {
    pub const JointState = SkeletalAnimation.JointState;
    pub const JointStateVector = std.ArrayList(JointState);
    pub const Mat44Vector = std.ArrayList(Mat44);

    /// Allocates the joint arrays (Zolt)
    allocator: Allocator,

    /// Skeleton definition (private in Jolt, use getSkeleton / setSkeleton)
    skeleton: RefConst(Skeleton) = .empty,
    /// Extra offset applied to the root (and therefore also to all of its children) (private in Jolt, use
    /// getRootOffset / setRootOffset)
    root_offset: RVec3 = RVec3.zero(),
    /// Local joint orientations (local to parent Joint) (private in Jolt, use getJoints / getJointsMut)
    joints: JointStateVector = .empty,
    /// Local joint matrices (local to world matrix) (private in Jolt, use getJointMatrices / getJointMatricesMut)
    joint_matrices: Mat44Vector = .empty,

    /// Default constructor: no skeleton, no joints
    pub fn init(allocator: Allocator) SkeletonPose {
        return .{ .allocator = allocator };
    }

    /// Destructor: releases the skeleton and frees the joint arrays
    pub fn deinit(self: *SkeletonPose) void {
        self.skeleton.deinit();
        self.joints.deinit(self.allocator);
        self.joint_matrices.deinit(self.allocator);
    }

    /// Copy constructor: the copy references the same skeleton and has copies of the joint arrays
    pub fn clone(self: *const SkeletonPose) Allocator.Error!SkeletonPose {
        var joints = try self.joints.clone(self.allocator);
        errdefer joints.deinit(self.allocator);
        const joint_matrices = try self.joint_matrices.clone(self.allocator);
        return .{
            .allocator = self.allocator,
            .skeleton = self.skeleton.clone(),
            .root_offset = self.root_offset,
            .joints = joints,
            .joint_matrices = joint_matrices,
        };
    }

    // Skeleton

    /// Set the skeleton and resize the joint arrays to its joint count (Zolt: Allocator.Error, nothing changes when it
    /// fails)
    pub fn setSkeleton(self: *SkeletonPose, skeleton: *const Skeleton) Allocator.Error!void {
        const num_joints = skeleton.getJointCount();

        // Zolt: reserve first, so that the resizes below cannot fail
        try self.joints.ensureTotalCapacity(self.allocator, num_joints);
        try self.joint_matrices.ensureTotalCapacity(self.allocator, num_joints);

        self.skeleton.set(skeleton);

        // mJoints.resize(): new joint states are default constructed
        if (num_joints > self.joints.items.len)
            self.joints.appendNTimesAssumeCapacity(.{}, num_joints - self.joints.items.len)
        else
            self.joints.shrinkRetainingCapacity(num_joints);

        // mJointMatrices.resize(): Mat44 is trivially constructible, new matrices are uninitialized
        if (num_joints > self.joint_matrices.items.len)
            _ = self.joint_matrices.addManyAsSliceAssumeCapacity(num_joints - self.joint_matrices.items.len)
        else
            self.joint_matrices.shrinkRetainingCapacity(num_joints);
    }

    /// The skeleton (null before setSkeleton)
    pub fn getSkeleton(self: *const SkeletonPose) ?*const Skeleton {
        return self.skeleton.get();
    }

    /// Extra offset applied to the root (and therefore also to all of its children)
    pub fn setRootOffset(self: *SkeletonPose, offset: RVec3) void {
        self.root_offset = offset;
    }

    /// Extra offset applied to the root (and therefore also to all of its children)
    pub fn getRootOffset(self: *const SkeletonPose) RVec3 {
        return self.root_offset;
    }

    // Properties of the joints

    /// The number of joint states
    pub fn getJointCount(self: *const SkeletonPose) usize {
        return self.joints.items.len;
    }

    /// The joint states (GetJoints() const)
    pub fn getJoints(self: *const SkeletonPose) []const JointState {
        return self.joints.items;
    }

    /// The joint states (the non-const GetJoints: the array itself, allocated with `allocator`)
    pub fn getJointsMut(self: *SkeletonPose) *JointStateVector {
        return &self.joints;
    }

    /// A joint state (GetJoint(int) const)
    pub fn getJoint(self: *const SkeletonPose, joint: usize) *const JointState {
        return &self.joints.items[joint];
    }

    /// A joint state (the non-const GetJoint)
    pub fn getJointMut(self: *SkeletonPose, joint: usize) *JointState {
        return &self.joints.items[joint];
    }

    // Joint matrices

    /// The joint matrices (GetJointMatrices() const)
    pub fn getJointMatrices(self: *const SkeletonPose) []const Mat44 {
        return self.joint_matrices.items;
    }

    /// The joint matrices (the non-const GetJointMatrices: the array itself, allocated with `allocator`)
    pub fn getJointMatricesMut(self: *SkeletonPose) *Mat44Vector {
        return &self.joint_matrices;
    }

    /// A joint matrix (GetJointMatrix(int) const)
    pub fn getJointMatrix(self: *const SkeletonPose, joint: usize) *const Mat44 {
        return &self.joint_matrices.items[joint];
    }

    /// A joint matrix (the non-const GetJointMatrix)
    pub fn getJointMatrixMut(self: *SkeletonPose, joint: usize) *Mat44 {
        return &self.joint_matrices.items[joint];
    }

    /// Convert the joint states to joint matrices
    pub fn calculateJointMatrices(self: *SkeletonPose) void {
        const matrices = self.joint_matrices.items;
        for (self.joints.items, 0..) |*joint, i| {
            matrices[i] = joint.toMatrix();

            const parent = self.skeletonOfJoints().getJoint(i).parent_joint_index;
            if (parent >= 0) {
                if (Core.enable_asserts) std.debug.assert(parent < @as(i32, @intCast(i))); // Joints must be ordered: parents first
                matrices[i] = matrices[@intCast(parent)].mul(matrices[i]);
            }
        }
    }

    /// Convert joint matrices to joint states
    pub fn calculateJointStates(self: *SkeletonPose) void {
        const matrices = self.joint_matrices.items;
        for (self.joints.items, 0..) |*joint, i| {
            var local_transform: Mat44 = undefined;
            const parent = self.skeletonOfJoints().getJoint(i).parent_joint_index;
            if (parent >= 0)
                local_transform = matrices[@intCast(parent)].inversed().mul(matrices[i])
            else
                local_transform = matrices[i];

            joint.translation = local_transform.getTranslation();
            joint.rotation = local_transform.getQuaternion();
        }
    }

    /// Outputs the joint matrices in local space (ensure that outMatrices has GetJointCount() elements, assumes that values in GetJoints() is up to date)
    pub fn calculateLocalSpaceJointMatrices(self: *const SkeletonPose, out_matrices: []Mat44) void {
        for (self.joints.items, 0..) |*joint, i|
            out_matrices[i] = joint.toMatrix();
    }

    // TODO(debug_renderer): DrawSettings (mDrawJoints, mDrawJointOrientations, mDrawJointNames) and Draw(inDrawSettings,
    // inRenderer, inOffset) (JPH_DEBUG_RENDERER)

    /// The skeleton of a pose that has joint states (Jolt dereferences mSkeleton without a check)
    fn skeletonOfJoints(self: *const SkeletonPose) *const Skeleton {
        return self.skeleton.get() orelse @panic("SkeletonPose: joints without a skeleton (Jolt dereferences a null pointer)");
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests

const testing = std.testing;
const Ref = @import("../Core/Reference.zig").Ref;
const Quat = @import("../Math/Quat.zig").Quat;
const Vec3 = @import("../Math/Vec3.zig").Vec3;

/// Root - Spine - Head, Spine - Arm
fn buildSkeleton(skeleton: *Skeleton) !void {
    _ = try skeleton.addJoint("Root", .{});
    _ = try skeleton.addJointWithParentIndex("Spine", 0);
    _ = try skeleton.addJointWithParentIndex("Head", 1);
    _ = try skeleton.addJointWithParentIndex("Arm", 1);
}

fn setStates(pose: *SkeletonPose) void {
    const states = pose.getJointsMut().items;
    states[0] = .{ .rotation = Quat.rotation(Vec3.axisY(), 0.5), .translation = Vec3.init(1, 2, 3) };
    states[1] = .{ .rotation = Quat.rotation(Vec3.axisX(), -0.25), .translation = Vec3.init(0, 1, 0) };
    states[2] = .{ .rotation = Quat.identity(), .translation = Vec3.init(0, 0.5, 0) };
    states[3] = .{ .rotation = Quat.rotation(Vec3.init(1, 1, 0).normalized(), 2.0), .translation = Vec3.init(0.3, 0.8, -0.1) };
}

test "SkeletonPose: setSkeleton, root offset and accessors" {
    const allocator = testing.allocator;
    var skeleton = Ref(Skeleton).init(try Skeleton.create(allocator));
    defer skeleton.deinit();
    try buildSkeleton(skeleton.get().?);

    var pose: SkeletonPose = .init(allocator);
    defer pose.deinit();
    try testing.expect(pose.getSkeleton() == null);
    try testing.expectEqual(@as(usize, 0), pose.getJointCount());
    try testing.expect(pose.getRootOffset().eql(RVec3.zero()));
    pose.setRootOffset(RVec3.init(1, -2, 1.0e6));
    try testing.expect(pose.getRootOffset().eql(RVec3.init(1, -2, 1.0e6)));

    // The pose references the skeleton and has one default joint state per joint
    try pose.setSkeleton(skeleton.get().?);
    try testing.expect(pose.getSkeleton() == skeleton.get().?);
    try testing.expectEqual(@as(u32, 2), skeleton.get().?.getRefCount());
    try testing.expectEqual(@as(usize, 4), pose.getJointCount());
    try testing.expectEqual(@as(usize, 4), pose.getJoints().len);
    try testing.expectEqual(@as(usize, 4), pose.getJointMatrices().len);
    for (pose.getJoints()) |*state| {
        try testing.expect(state.rotation.eql(Quat.identity()));
        try testing.expect(state.translation.eql(Vec3.zero()));
    }

    // A smaller skeleton keeps the first states, a larger one adds default states
    pose.getJointMut(0).translation = Vec3.init(5, 5, 5);
    pose.getJointMut(3).translation = Vec3.init(6, 6, 6);
    var small = Ref(Skeleton).init(try Skeleton.create(allocator));
    defer small.deinit();
    _ = try small.get().?.addJoint("Root", .{});
    try pose.setSkeleton(small.get().?);
    try testing.expectEqual(@as(u32, 1), skeleton.get().?.getRefCount());
    try testing.expectEqual(@as(usize, 1), pose.getJointCount());
    try testing.expectEqual(@as(usize, 1), pose.getJointMatrices().len);
    try pose.setSkeleton(skeleton.get().?);
    try testing.expect(pose.getJoint(0).translation.eql(Vec3.init(5, 5, 5)));
    try testing.expect(pose.getJoint(3).translation.eql(Vec3.zero()));
    try testing.expect(pose.getRootOffset().eql(RVec3.init(1, -2, 1.0e6)));

    // Mutable access
    pose.getJointMatrixMut(2).* = Mat44.identity();
    try testing.expect(pose.getJointMatrix(2).eql(Mat44.identity()));
    try testing.expect(pose.getJointMatricesMut().items[2].eql(Mat44.identity()));
}

test "SkeletonPose: calculateJointMatrices, calculateLocalSpaceJointMatrices, calculateJointStates" {
    const allocator = testing.allocator;
    var skeleton = Ref(Skeleton).init(try Skeleton.create(allocator));
    defer skeleton.deinit();
    try buildSkeleton(skeleton.get().?);
    var pose: SkeletonPose = .init(allocator);
    defer pose.deinit();
    try pose.setSkeleton(skeleton.get().?);
    setStates(&pose);
    pose.setRootOffset(RVec3.init(100, 0, 0)); // Not part of the joint matrices

    // Model space matrices: the parent's matrix times the local matrix, the root offset is not applied
    pose.calculateJointMatrices();
    const states = pose.getJoints();
    var local: [4]Mat44 = undefined;
    pose.calculateLocalSpaceJointMatrices(&local);
    for (states, local) |*state, m|
        try testing.expect(m.eql(Mat44.rotationTranslation(state.rotation, state.translation)));
    const m = pose.getJointMatrices();
    try testing.expect(m[0].eql(local[0]));
    try testing.expect(m[1].eql(local[0].mul(local[1])));
    try testing.expect(m[2].eql(local[0].mul(local[1]).mul(local[2])));
    try testing.expect(m[3].eql(m[1].mul(local[3])));
    try testing.expect(m[2].getTranslation().isClose(m[1].mulVec3(Vec3.init(0, 0.5, 0)), .{}));

    // Back to joint states: the same rotations (up to sign) and translations, within rounding
    var copy = try pose.clone();
    defer copy.deinit();
    try testing.expectEqual(@as(u32, 3), skeleton.get().?.getRefCount());
    copy.calculateJointStates();
    for (copy.getJoints(), states) |*a, *b| {
        try testing.expect(a.translation.isClose(b.translation, .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(a.rotation.isClose(b.rotation, .{ .max_dist_sq = 1.0e-10 }) or a.rotation.isClose(b.rotation.negate(), .{ .max_dist_sq = 1.0e-10 }));
    }

    // The clone is independent
    copy.getJointMut(0).translation = Vec3.zero();
    try testing.expect(pose.getJoint(0).translation.eql(Vec3.init(1, 2, 3)));
    try testing.expect(copy.getRootOffset().eql(RVec3.init(100, 0, 0)));

    // A root keeps its matrix as its state
    pose.getJointMatrixMut(0).* = Mat44.rotationTranslation(Quat.rotation(Vec3.axisZ(), 3.0), Vec3.init(-1, 0, 2));
    pose.calculateJointStates();
    try testing.expect(pose.getJoint(0).translation.eql(Vec3.init(-1, 0, 2)));
    try testing.expect(pose.getJoint(0).rotation.eql(pose.getJointMatrix(0).getQuaternion()));
}

fn setSkeletonOutOfMemory(allocator: Allocator) !void {
    var skeleton = Ref(Skeleton).init(try Skeleton.create(allocator));
    defer skeleton.deinit();
    try buildSkeleton(skeleton.get().?);
    var pose: SkeletonPose = .init(allocator);
    defer pose.deinit();
    pose.setSkeleton(skeleton.get().?) catch |err| {
        // Nothing changed
        try testing.expect(pose.getSkeleton() == null);
        try testing.expectEqual(@as(usize, 0), pose.getJointCount());
        try testing.expectEqual(@as(usize, 0), pose.getJointMatrices().len);
        return err;
    };
    setStates(&pose);
    pose.calculateJointMatrices();
    var copy = try pose.clone();
    defer copy.deinit();
    try testing.expect(copy.getJointMatrix(3).eql(pose.getJointMatrix(3).*));
}

test "SkeletonPose: out of memory in setSkeleton and clone" {
    try testing.checkAllAllocationFailures(testing.allocator, setSkeletonOutOfMemory, .{});
}
