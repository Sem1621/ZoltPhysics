//! Port of: Jolt/Skeleton/SkeletalAnimation.h, Jolt/Skeleton/SkeletalAnimation.cpp
//! Status: complete
//!
//! JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL (SkeletalAnimation, JointState,
//! Keyframe, AnimatedJoint): TODO(serialization) (ObjectStream is Phase 8, CharacterVehicleArchitecture E1; the binary
//! state is ported).
//!
//! CharacterVehicleArchitecture E2: a RefTarget without virtual functions, like PhysicsScene. The animation keeps the
//! allocator that owns it, its animated joints, their names and keyframes: `create(allocator)` is `new
//! SkeletalAnimation` (reference count 0, put it in a `Ref` / `RefConst`), `init(allocator)` / `deinit` an animation on
//! the stack or embedded.
//! - `Keyframe : JointState` embeds its parent as `base` (`static_cast<const JointState &>(keyframe)` is
//!   `keyframe.base`).
//! - `getAnimatedJointsMut` returns the array itself (Jolt's non-const GetAnimatedJoints): the joint names and keyframe
//!   arrays added through it must be allocated with the animation's allocator, the animation frees them.
//! - sRestoreFromBinaryState is `restoreFromBinaryState(allocator, stream) Allocator.Error!AnimationResult`. Like Jolt
//!   it never reports an error: the stream is not checked, so a truncated stream gives an animation with what could be
//!   read (default keyframes, values that were read partially). A count or name length that cannot be allocated (a
//!   corrupt stream, where Jolt's resize aborts) is `error.OutOfMemory`.
//! - Sample: Jolt's asserts on its input (`inTime >= 0`, the keyframe fraction in [0, 1], normalized rotations) are
//!   checked in safe builds only (`if (Core.enable_asserts)`), so that optimized builds compute what Jolt's release
//!   build computes. The binary search always ends on two keyframes that bracket the time, also when the keyframes are
//!   not sorted, so the fraction is only outside [0, 1] when it is NaN: times whose differences overflow (near the end
//!   of the float range, infinite). Where Jolt has undefined behavior Zolt panics: an animated joint that is not in the pose's skeleton
//!   (Jolt writes `ioPose.GetJoint(-1)`), an animated joint without keyframes (Jolt reads `front()` of an empty array)
//!   and a pose without a skeleton (Jolt dereferences null).
//! - `fmod(inTime, duration)` is `@rem(time, duration)`: both are the exact remainder with the sign of the time (see
//!   GearConstraint.zig).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../Core/Core.zig");
const ReferenceFile = @import("../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefCount = ReferenceFile.RefCount;
const Result = @import("../Core/Result.zig").Result;
const StreamIn = @import("../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../Core/StreamOut.zig").StreamOut;
const Mat44 = @import("../Math/Mat44.zig").Mat44;
const Quat = @import("../Math/Quat.zig").Quat;
const Vec3 = @import("../Math/Vec3.zig").Vec3;
const SkeletonPose = @import("SkeletonPose.zig").SkeletonPose;

/// Resource for a skinned animation
pub const SkeletalAnimation = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, SkeletalAnimation)

    /// Contains the current state of a joint, a local space transformation relative to its parent joint
    pub const JointState = struct {
        // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, JointState)

        /// Local space rotation of the joint
        rotation: Quat = Quat.identity(),
        /// Local space translation of the joint
        translation: Vec3 = Vec3.zero(),

        /// Convert from a local space matrix
        pub fn fromMatrix(self: *JointState, matrix: Mat44) void {
            self.rotation = matrix.getQuaternion();
            self.translation = matrix.getTranslation();
        }

        /// Convert to matrix representation
        pub fn toMatrix(self: *const JointState) Mat44 {
            return Mat44.rotationTranslation(self.rotation, self.translation);
        }
    };

    /// Contains the state of a single joint at a particular time
    pub const Keyframe = struct {
        // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, Keyframe)

        /// JointState (the C++ base class)
        base: JointState = .{},
        /// Time of keyframe in seconds
        time: f32 = 0.0,
    };

    pub const KeyframeVector = std.ArrayList(Keyframe);

    /// Contains the animation for a single joint
    pub const AnimatedJoint = struct {
        // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, AnimatedJoint)

        /// Name of the joint (owned, allocated with the animation's allocator)
        joint_name: []u8 = &.{},
        /// List of keyframes over time (allocated with the animation's allocator)
        keyframes: KeyframeVector = .empty,

        /// Destructor: frees the name and the keyframes
        pub fn deinit(self: *AnimatedJoint, allocator: Allocator) void {
            allocator.free(self.joint_name);
            self.keyframes.deinit(allocator);
            self.* = undefined;
        }
    };

    pub const AnimatedJointVector = std.ArrayList(AnimatedJoint);

    pub const AnimationResult = Result(Ref(SkeletalAnimation));

    /// RefTarget<SkeletalAnimation>
    ref_count: RefCount = .{},
    /// Allocates the animated joints, their names and keyframes, frees the animation
    allocator: Allocator,

    /// List of joints and keyframes (private in Jolt, use getAnimatedJoints / getAnimatedJointsMut)
    animated_joints: AnimatedJointVector = .empty,
    /// If this animation loops back to start (private in Jolt, use isLooping / setIsLooping)
    is_looping: bool = true,

    /// Constructor of an animation that is not on the heap (`deinit` it)
    pub fn init(allocator: Allocator) SkeletalAnimation {
        return .{ .allocator = allocator };
    }

    /// new SkeletalAnimation: reference count 0, the last `release` destroys it
    pub fn create(allocator: Allocator) Allocator.Error!*SkeletalAnimation {
        const self = try allocator.create(SkeletalAnimation);
        self.* = .init(allocator);
        return self;
    }

    /// Destructor: frees the animated joints
    pub fn deinit(self: *SkeletalAnimation) void {
        self.ref_count.assertUnreferenced();
        self.destruct();
    }

    fn destruct(self: *SkeletalAnimation) void {
        for (self.animated_joints.items) |*j| j.deinit(self.allocator);
        self.animated_joints.deinit(self.allocator);
    }

    /// Add a reference to this object
    pub fn addRef(self: *const SkeletalAnimation) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const SkeletalAnimation) void {
        if (self.ref_count.release()) {
            const mutable = @constCast(self); // The object is dead afterwards (Rule M exception)
            mutable.destruct();
            mutable.allocator.destroy(mutable);
        }
    }

    /// Mark this animation as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const SkeletalAnimation) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const SkeletalAnimation) u32 {
        return self.ref_count.get();
    }

    /// Get the length (in seconds) of this animation
    pub fn getDuration(self: *const SkeletalAnimation) f32 {
        if (self.animated_joints.items.len != 0 and self.animated_joints.items[0].keyframes.items.len != 0)
            return self.animated_joints.items[0].keyframes.getLast().time
        else
            return 0.0;
    }

    /// Scale the size of all joints by inScale
    pub fn scaleJoints(self: *SkeletalAnimation, scale: f32) void {
        for (self.animated_joints.items) |*j|
            for (j.keyframes.items) |*k| {
                k.base.translation = k.base.translation.mulScalar(scale);
            };
    }

    /// If the animation is looping or not. If an animation is looping, the animation will continue playing after completion
    pub fn setIsLooping(self: *SkeletalAnimation, is_looping: bool) void {
        self.is_looping = is_looping;
    }

    /// If the animation is looping or not. If an animation is looping, the animation will continue playing after completion
    pub fn isLooping(self: *const SkeletalAnimation) bool {
        return self.is_looping;
    }

    /// Get the (interpolated) joint transforms at time inTime
    pub fn sample(self: *const SkeletalAnimation, time: f32, pose: *SkeletonPose) void {
        // Correct time when animation is looping
        if (Core.enable_asserts) std.debug.assert(time >= 0.0); // Jolt's release build continues with a negative time
        const duration = self.getDuration();
        const t = if (duration > 0.0 and self.is_looping) @rem(time, duration) else time;

        for (self.animated_joints.items) |*aj| {
            const keyframes = aj.keyframes.items;

            // Do binary search for keyframe
            var high: i32 = @intCast(keyframes.len);
            var low: i32 = -1;
            while (high - low > 1) {
                const probe = @divTrunc(high + low, 2);
                if (keyframes[@intCast(probe)].time < t)
                    low = probe
                else
                    high = probe;
            }

            const skeleton = pose.getSkeleton() orelse @panic("SkeletalAnimation::Sample: the pose has no skeleton (Jolt dereferences a null pointer)");
            const joint_index = skeleton.getJointIndex(aj.joint_name);
            if (joint_index < 0)
                @panic("SkeletalAnimation::Sample: an animated joint is not in the skeleton of the pose (Jolt writes before the start of the pose's joints)");
            const state = pose.getJointMut(@intCast(joint_index));

            if (low == -1) {
                // Before first key, return first key
                if (keyframes.len == 0)
                    @panic("SkeletalAnimation::Sample: an animated joint without keyframes (Jolt reads the first element of an empty array)");
                state.* = keyframes[0].base;
            } else if (high == @as(i32, @intCast(keyframes.len))) {
                // Beyond last key, return last key
                state.* = keyframes[keyframes.len - 1].base;
            } else {
                // Interpolate
                const s1 = &keyframes[@intCast(low)];
                const s2 = &keyframes[@intCast(low + 1)];

                const fraction = (t - s1.time) / (s2.time - s1.time);
                if (Core.enable_asserts) std.debug.assert(fraction >= 0.0 and fraction <= 1.0); // Only a NaN fails it (times whose differences overflow), Jolt's release build continues

                state.translation = s1.base.translation.mulScalar(1.0 - fraction).add(s2.base.translation.mulScalar(fraction));
                if (Core.enable_asserts) std.debug.assert(s1.base.rotation.isNormalized(.{}));
                if (Core.enable_asserts) std.debug.assert(s2.base.rotation.isNormalized(.{}));
                state.rotation = s1.base.rotation.slerp(s2.base.rotation, fraction);
                if (Core.enable_asserts) std.debug.assert(state.rotation.isNormalized(.{}));
            }
        }
    }

    /// Get joint samples (GetAnimatedJoints() const)
    pub fn getAnimatedJoints(self: *const SkeletalAnimation) []const AnimatedJoint {
        return self.animated_joints.items;
    }

    /// Get joint samples (the non-const GetAnimatedJoints: the array itself, its names and keyframes are owned by
    /// `allocator`)
    pub fn getAnimatedJointsMut(self: *SkeletalAnimation) *AnimatedJointVector {
        return &self.animated_joints;
    }

    /// Saves the state of this animation in binary form to inStream.
    pub fn saveBinaryState(self: *const SkeletalAnimation, stream: StreamOut) void {
        stream.write(@as(u32, @intCast(self.animated_joints.items.len)));
        for (self.animated_joints.items) |*j| {
            // Write Joint name and number of keyframes
            stream.writeString(j.joint_name);
            stream.write(@as(u32, @intCast(j.keyframes.items.len)));
            for (j.keyframes.items) |*k| {
                stream.write(k.time);
                stream.write(k.base.rotation);
                stream.write(k.base.translation);
            }
        }

        // Save additional parameters
        stream.write(self.is_looping);
    }

    /// Restore a saved ragdoll from inStream (Jolt: sRestoreFromBinaryState, the result is always valid; it holds a
    /// reference to the new animation, `deinit` it)
    pub fn restoreFromBinaryState(allocator: Allocator, stream: StreamIn) Allocator.Error!AnimationResult {
        var result: AnimationResult = .empty;

        var animation = Ref(SkeletalAnimation).init(try create(allocator));
        defer animation.deinit();
        const a = animation.get().?;

        // Restore animated joints (Jolt resizes the arrays to the lengths that are read without checking the stream, the
        // new elements are default constructed)
        var len: u32 = 0;
        stream.read(&len);
        try a.animated_joints.ensureTotalCapacityPrecise(allocator, len);
        a.animated_joints.appendNTimesAssumeCapacity(.{}, len);
        for (a.animated_joints.items) |*j| {
            // Read joint name
            try stream.readString(allocator, &j.joint_name);

            // Read keyframes
            len = 0;
            stream.read(&len);
            try j.keyframes.ensureTotalCapacityPrecise(allocator, len);
            j.keyframes.appendNTimesAssumeCapacity(.{}, len);
            for (j.keyframes.items) |*k| {
                stream.read(&k.time);
                stream.read(&k.base.rotation);
                stream.read(&k.base.translation);
            }
        }

        // Read additional parameters
        stream.read(&a.is_looping);
        result.set(animation.clone());
        return result;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests

const testing = std.testing;
const StreamWrapper = @import("../Core/StreamWrapper.zig");
const Skeleton = @import("Skeleton.zig").Skeleton;

fn keyframe(time: f32, rotation: Quat, translation: Vec3) SkeletalAnimation.Keyframe {
    return .{ .base = .{ .rotation = rotation, .translation = translation }, .time = time };
}

/// Add an animated joint with a copy of `name` and `keyframes` to `animation`
fn addAnimatedJoint(animation: *SkeletalAnimation, name: []const u8, keyframes: []const SkeletalAnimation.Keyframe) !void {
    const allocator = animation.allocator;
    var aj: SkeletalAnimation.AnimatedJoint = .{ .joint_name = try allocator.dupe(u8, name) };
    errdefer aj.deinit(allocator);
    try aj.keyframes.appendSlice(allocator, keyframes);
    try animation.getAnimatedJointsMut().append(allocator, aj);
}

/// Skeleton Root - Arm - Hand
fn buildSkeleton(skeleton: *Skeleton) !void {
    _ = try skeleton.addJoint("Root", .{});
    _ = try skeleton.addJointWithParentIndex("Arm", 0);
    _ = try skeleton.addJointWithParentIndex("Hand", 1);
}

/// Arm: 3 keyframes from 0.5 to 2.5 seconds, Hand: 1 keyframe, Root: 2 keyframes (with a 180 degree turn)
fn buildAnimation(animation: *SkeletalAnimation) !void {
    try addAnimatedJoint(animation, "Arm", &.{
        keyframe(0.5, Quat.identity(), Vec3.init(1, 0, 0)),
        keyframe(1.5, Quat.rotation(Vec3.axisZ(), 0.5), Vec3.init(2, 0, 0)),
        keyframe(2.5, Quat.rotation(Vec3.axisZ(), 1.0), Vec3.init(2, 4, 0)),
    });
    try addAnimatedJoint(animation, "Hand", &.{keyframe(1.0, Quat.rotation(Vec3.axisX(), 0.3), Vec3.init(0, 1, 0))});
    try addAnimatedJoint(animation, "Root", &.{
        keyframe(0.0, Quat.identity(), Vec3.zero()),
        keyframe(2.0, Quat.rotation(Vec3.axisY(), 3.0), Vec3.init(0, 0, 8)),
    });
}

fn saveAnimation(animation: *const SkeletalAnimation, out: *std.Io.Writer.Allocating) ![]const u8 {
    var wrapper = StreamWrapper.StreamOutWrapper.init(&out.writer);
    animation.saveBinaryState(wrapper.streamOut());
    if (wrapper.isFailed()) return error.OutOfMemory; // The allocating writer could not grow
    return out.written();
}

fn restoreAnimation(allocator: Allocator, bytes: []const u8) Allocator.Error!SkeletalAnimation.AnimationResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return SkeletalAnimation.restoreFromBinaryState(allocator, in.streamIn());
}

test "SkeletalAnimation: JointState fromMatrix / toMatrix" {
    var state: SkeletalAnimation.JointState = .{};
    try testing.expect(state.rotation.eql(Quat.identity()));
    try testing.expect(state.translation.eql(Vec3.zero()));
    try testing.expect(state.toMatrix().eql(Mat44.identity()));

    const rotation = Quat.rotation(Vec3.init(1, 2, 3).normalized(), 0.7);
    const matrix = Mat44.rotationTranslation(rotation, Vec3.init(4, 5, 6));
    state.fromMatrix(matrix);
    try testing.expect(state.rotation.isClose(rotation, .{}));
    try testing.expect(state.translation.eql(Vec3.init(4, 5, 6)));
    try testing.expect(state.toMatrix().isClose(matrix, .{ .max_dist_sq = 1.0e-10 }));
    try testing.expect(state.toMatrix().eql(Mat44.rotationTranslation(state.rotation, state.translation)));
}

test "SkeletalAnimation: getDuration, scaleJoints, isLooping" {
    var animation: SkeletalAnimation = .init(testing.allocator);
    defer animation.deinit();
    try testing.expectEqual(@as(f32, 0.0), animation.getDuration());
    try testing.expect(animation.isLooping());
    animation.setIsLooping(false);
    try testing.expect(!animation.isLooping());

    // The duration is the time of the last keyframe of the first animated joint
    try addAnimatedJoint(&animation, "Empty", &.{});
    try testing.expectEqual(@as(f32, 0.0), animation.getDuration());
    animation.getAnimatedJointsMut().items[0].deinit(animation.allocator);
    _ = animation.getAnimatedJointsMut().orderedRemove(0);
    try buildAnimation(&animation);
    try testing.expectEqual(@as(f32, 2.5), animation.getDuration());
    try testing.expectEqual(@as(usize, 3), animation.getAnimatedJoints().len);

    animation.scaleJoints(0.5);
    const arm = animation.getAnimatedJoints()[0].keyframes.items;
    try testing.expect(arm[2].base.translation.eql(Vec3.init(1, 2, 0)));
    try testing.expect(arm[2].base.rotation.eql(Quat.rotation(Vec3.axisZ(), 1.0))); // Rotations are not scaled
    try testing.expectEqual(@as(f32, 2.5), arm[2].time);
}

test "SkeletalAnimation: sample" {
    const allocator = testing.allocator;
    var skeleton = Ref(Skeleton).init(try Skeleton.create(allocator));
    defer skeleton.deinit();
    try buildSkeleton(skeleton.get().?);
    var animation = Ref(SkeletalAnimation).init(try SkeletalAnimation.create(allocator));
    defer animation.deinit();
    const anim = animation.get().?;
    try buildAnimation(anim);
    anim.setIsLooping(false);
    var pose: SkeletonPose = .init(allocator);
    defer pose.deinit();
    try pose.setSkeleton(skeleton.get().?);

    const arm = anim.getAnimatedJoints()[0].keyframes.items;
    const hand = anim.getAnimatedJoints()[1].keyframes.items;
    const root = anim.getAnimatedJoints()[2].keyframes.items;

    // Before the first keyframe: the first keyframe (also exactly on it), a joint with one keyframe always has it
    anim.sample(0.25, &pose);
    try testing.expect(pose.getJoint(1).rotation.eql(arm[0].base.rotation));
    try testing.expect(pose.getJoint(1).translation.eql(arm[0].base.translation));
    try testing.expect(pose.getJoint(2).translation.eql(hand[0].base.translation));
    anim.sample(0.5, &pose);
    try testing.expect(pose.getJoint(1).translation.eql(arm[0].base.translation));

    // Between keyframes: linear translation, slerped rotation
    anim.sample(1.0, &pose);
    try testing.expect(pose.getJoint(1).translation.eql(Vec3.init(1.5, 0, 0)));
    try testing.expect(pose.getJoint(1).rotation.isClose(Quat.rotation(Vec3.axisZ(), 0.25), .{}));
    try testing.expect(pose.getJoint(1).rotation.eql(arm[0].base.rotation.slerp(arm[1].base.rotation, 0.5)));
    try testing.expect(pose.getJoint(0).translation.eql(Vec3.init(0, 0, 4)));
    try testing.expect(pose.getJoint(2).rotation.eql(hand[0].base.rotation));
    anim.sample(1.5, &pose); // Exactly on a keyframe: the interpolation with fraction 1
    try testing.expect(pose.getJoint(1).translation.eql(arm[1].base.translation));

    // After the last keyframe (not looping): the last keyframe
    anim.sample(2.75, &pose);
    try testing.expect(pose.getJoint(1).translation.eql(arm[2].base.translation));
    try testing.expect(pose.getJoint(0).translation.eql(root[1].base.translation));
    anim.sample(100.0, &pose);
    try testing.expect(pose.getJoint(1).rotation.eql(arm[2].base.rotation));

    // Looping: the time wraps around the duration (2.5 s)
    anim.setIsLooping(true);
    anim.sample(3.5, &pose); // = 1.0
    try testing.expect(pose.getJoint(1).translation.eql(Vec3.init(1.5, 0, 0)));
    try testing.expect(pose.getJoint(0).translation.eql(Vec3.init(0, 0, 4)));
    anim.sample(5.0, &pose); // = 0.0: before the first keyframe of the arm, the first of the root
    try testing.expect(pose.getJoint(1).translation.eql(arm[0].base.translation));
    try testing.expect(pose.getJoint(0).translation.eql(root[0].base.translation));
    anim.sample(2.5, &pose); // = 0.0 as well
    try testing.expect(pose.getJoint(0).translation.eql(root[0].base.translation));
    anim.sample(2.4, &pose); // Not wrapped, after the last keyframe of the root
    try testing.expect(pose.getJoint(0).translation.eql(root[1].base.translation));

    // A joint that is not animated keeps its state
    pose.getJointMut(2).translation = Vec3.init(7, 8, 9);
    anim.getAnimatedJointsMut().items[1].deinit(anim.allocator);
    _ = anim.getAnimatedJointsMut().orderedRemove(1);
    anim.sample(1.0, &pose);
    try testing.expect(pose.getJoint(2).translation.eql(Vec3.init(7, 8, 9)));

    // An animation without joints changes nothing, a zero duration does not wrap
    var empty: SkeletalAnimation = .init(allocator);
    defer empty.deinit();
    empty.sample(10.0, &pose);
    try testing.expect(pose.getJoint(2).translation.eql(Vec3.init(7, 8, 9)));
    var still: SkeletalAnimation = .init(allocator);
    defer still.deinit();
    try addAnimatedJoint(&still, "Hand", &.{ keyframe(0.0, Quat.identity(), Vec3.zero()), keyframe(0.0, Quat.identity(), Vec3.init(1, 1, 1)) });
    still.sample(0.0, &pose); // Before (not after) the first keyframe at time 0
    try testing.expect(pose.getJoint(2).translation.eql(Vec3.zero()));
    still.sample(1.0, &pose);
    try testing.expect(pose.getJoint(2).translation.eql(Vec3.init(1, 1, 1)));
}

test "SkeletalAnimation: saveBinaryState / restoreFromBinaryState" {
    const allocator = testing.allocator;
    var animation: SkeletalAnimation = .init(allocator);
    defer animation.deinit();
    try buildAnimation(&animation);
    animation.setIsLooping(false);

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const bytes = try saveAnimation(&animation, &out);

    // Jolt's layout: joint count, per joint the name, keyframe count and per keyframe time, rotation (16 bytes) and
    // translation (12 bytes), then the looping flag
    try testing.expectEqual(@as(usize, 4 + 3 * (4 + 4) + 3 + 4 + 4 + 6 * 32 + 1), bytes.len);
    try testing.expectEqual(@as(u32, 3), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqualStrings("Arm", bytes[8..11]);
    try testing.expectEqual(@as(u32, 3), std.mem.readInt(u32, bytes[11..15], .little));
    try testing.expectEqual(@as(u32, @bitCast(@as(f32, 0.5))), std.mem.readInt(u32, bytes[15..19], .little));
    try testing.expectEqual(@as(u8, 0), bytes[bytes.len - 1]);

    var result = try restoreAnimation(allocator, bytes);
    defer result.deinit();
    const restored = result.getPtr().?;
    try testing.expectEqual(@as(u32, 1), restored.getRefCount());
    try testing.expect(!restored.isLooping());
    try testing.expectEqual(@as(usize, 3), restored.getAnimatedJoints().len);
    for (animation.getAnimatedJoints(), restored.getAnimatedJoints()) |*a, *b| {
        try testing.expectEqualStrings(a.joint_name, b.joint_name);
        try testing.expectEqual(a.keyframes.items.len, b.keyframes.items.len);
        for (a.keyframes.items, b.keyframes.items) |ka, kb| {
            try testing.expectEqual(ka.time, kb.time);
            try testing.expect(ka.base.rotation.eql(kb.base.rotation));
            try testing.expect(ka.base.translation.eql(kb.base.translation));
        }
    }
    var out2: std.Io.Writer.Allocating = .init(allocator);
    defer out2.deinit();
    try testing.expectEqualSlices(u8, bytes, try saveAnimation(restored, &out2));

    // A truncated stream is not an error: what could be read, default keyframes and the default looping flag
    {
        var truncated = try restoreAnimation(allocator, bytes[0..38]); // Inside the X of the first translation of the arm
        defer truncated.deinit();
        const t = truncated.getPtr().?;
        try testing.expect(t.isLooping());
        try testing.expectEqual(@as(usize, 3), t.getAnimatedJoints().len);
        const arm = t.getAnimatedJoints()[0].keyframes.items;
        try testing.expectEqual(@as(usize, 3), arm.len);
        try testing.expectEqual(@as(f32, 0.5), arm[0].time);
        try testing.expect(arm[0].base.rotation.eql(Quat.identity()));
        // 3 of the 4 bytes of 1.0f (0x3f800000) were read over the default 0: 0x00800000
        try testing.expectEqual(@as(u32, 0x00800000), @as(u32, @bitCast(arm[0].base.translation.getX())));
        try testing.expectEqual(@as(f32, 0.0), arm[0].base.translation.getY());
        try testing.expectEqual(@as(f32, 0.0), arm[1].time);
        try testing.expect(arm[1].base.translation.eql(Vec3.zero()));
        try testing.expect(arm[2].base.rotation.eql(Quat.identity()));
        try testing.expectEqual(@as(usize, 0), t.getAnimatedJoints()[1].joint_name.len);
        try testing.expectEqual(@as(usize, 0), t.getAnimatedJoints()[2].keyframes.items.len);
    }
    for (0..bytes.len) |len| {
        var truncated = try restoreAnimation(allocator, bytes[0..len]);
        defer truncated.deinit();
        try testing.expect(truncated.isValid());
    }
}

fn buildAndSaveOutOfMemory(allocator: Allocator) !void {
    var animation = Ref(SkeletalAnimation).init(try SkeletalAnimation.create(allocator));
    defer animation.deinit();
    try buildAnimation(animation.get().?);
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const bytes = try saveAnimation(animation.get().?, &out);
    var result = try restoreAnimation(allocator, bytes);
    defer result.deinit();
    try testing.expectEqual(@as(usize, 3), result.getPtr().?.getAnimatedJoints().len);
}

test "SkeletalAnimation: out of memory in restoreFromBinaryState" {
    try testing.checkAllAllocationFailures(testing.allocator, buildAndSaveOutOfMemory, .{});
}

test "SkeletalAnimation: reference counting" {
    var animation = Ref(SkeletalAnimation).init(try SkeletalAnimation.create(testing.allocator));
    defer animation.deinit();
    try buildAnimation(animation.get().?);
    var second = animation.clone();
    try testing.expectEqual(@as(u32, 2), animation.get().?.getRefCount());
    second.deinit();
    try testing.expectEqual(@as(u32, 1), animation.get().?.getRefCount());
}
