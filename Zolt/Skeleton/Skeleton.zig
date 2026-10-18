//! Port of: Jolt/Skeleton/Skeleton.h, Jolt/Skeleton/Skeleton.cpp
//! Status: complete
//!
//! JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL (Skeleton, Skeleton::Joint):
//! TODO(serialization) (ObjectStream is Phase 8, CharacterVehicleArchitecture E1; the binary state is ported).
//!
//! CharacterVehicleArchitecture E2: a RefTarget without virtual functions, like PhysicsScene. The skeleton keeps the
//! allocator that owns it, its joint array and the joint names: `create(allocator)` is `new Skeleton` (reference count
//! 0, put it in a `Ref` / `RefConst`), `init(allocator)` / `deinit` a skeleton on the stack or embedded (`setEmbedded`
//! before references are taken).
//! - The joint names are owned strings (`[]u8`, allocated with the skeleton's allocator). `getJointsMut` returns the
//!   joint array itself (Jolt's non-const GetJoints): joints added through it must own their names with the skeleton's
//!   allocator (`Joint.init`), the skeleton frees them.
//! - The AddJoint overloads are `addJoint(name, .{ .parent_name })` and `addJointWithParentIndex(name, parent_index)`.
//!   They copy the names and return `Allocator.Error!u32`; nothing is added when they fail.
//! - sRestoreFromBinaryState is `restoreFromBinaryState(allocator, stream) Allocator.Error!SkeletonResult`: Jolt's
//!   "Failed to read skeleton from stream" stays a value in the result, only allocation failure is a Zig error. A joint
//!   count or name length that cannot be allocated (a corrupt stream, where Jolt's resize aborts) is
//!   `error.OutOfMemory`.
//! - Joint indices: Jolt's `int` stays `i32` where it can be -1 (`Joint.parent_joint_index`, `getJointIndex`); the joint
//!   count and indices that are always valid joints are `usize` (`getJointCount`, `getJoint`).

const std = @import("std");
const Allocator = std.mem.Allocator;
const ReferenceFile = @import("../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefCount = ReferenceFile.RefCount;
const Result = @import("../Core/Result.zig").Result;
const StreamIn = @import("../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../Core/StreamOut.zig").StreamOut;

/// Resource that contains the joint hierarchy for a skeleton
pub const Skeleton = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, Skeleton)

    pub const SkeletonResult = Result(Ref(Skeleton));

    /// Declare internal structure for a joint
    pub const Joint = struct {
        // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, Joint)

        /// Name of the joint (owned, allocated with the skeleton's allocator)
        name: []u8 = &.{},
        /// Name of parent joint (owned, allocated with the skeleton's allocator)
        parent_name: []u8 = &.{},
        /// Index of parent joint (in mJoints) or -1 if it has no parent
        parent_joint_index: i32 = -1,

        /// Constructor Joint(inName, inParentName, inParentJointIndex): copies the names with `allocator` (the
        /// skeleton's allocator when the joint is added to a skeleton)
        pub fn init(allocator: Allocator, name: []const u8, parent_name: []const u8, parent_joint_index: i32) Allocator.Error!Joint {
            const name_copy = try allocator.dupe(u8, name);
            errdefer allocator.free(name_copy);
            const parent_name_copy = try allocator.dupe(u8, parent_name);
            return .{ .name = name_copy, .parent_name = parent_name_copy, .parent_joint_index = parent_joint_index };
        }

        /// Destructor: frees the names
        pub fn deinit(self: *Joint, allocator: Allocator) void {
            allocator.free(self.name);
            allocator.free(self.parent_name);
            self.* = undefined;
        }
    };

    pub const JointVector = std.ArrayList(Joint);

    /// RefTarget<Skeleton>
    ref_count: RefCount = .{},
    /// Allocates the joint array and the joint names, frees the skeleton
    allocator: Allocator,

    /// Joints (private in Jolt, use getJoints / getJointsMut)
    joints: JointVector = .empty,

    /// Constructor of a skeleton that is not on the heap (`deinit` it)
    pub fn init(allocator: Allocator) Skeleton {
        return .{ .allocator = allocator };
    }

    /// new Skeleton: reference count 0, the last `release` destroys it
    pub fn create(allocator: Allocator) Allocator.Error!*Skeleton {
        const self = try allocator.create(Skeleton);
        self.* = .init(allocator);
        return self;
    }

    /// Destructor: frees the joints and their names
    pub fn deinit(self: *Skeleton) void {
        self.ref_count.assertUnreferenced();
        self.destruct();
    }

    fn destruct(self: *Skeleton) void {
        for (self.joints.items) |*j| j.deinit(self.allocator);
        self.joints.deinit(self.allocator);
    }

    /// Add a reference to this object
    pub fn addRef(self: *const Skeleton) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const Skeleton) void {
        if (self.ref_count.release()) {
            const mutable = @constCast(self); // The object is dead afterwards (Rule M exception)
            mutable.destruct();
            mutable.allocator.destroy(mutable);
        }
    }

    /// Mark this skeleton as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const Skeleton) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const Skeleton) u32 {
        return self.ref_count.get();
    }

    // Access to the joints

    /// The joints (GetJoints() const)
    pub fn getJoints(self: *const Skeleton) []const Joint {
        return self.joints.items;
    }

    /// The joints (the non-const GetJoints: the array itself, its elements and their names are owned by `allocator`)
    pub fn getJointsMut(self: *Skeleton) *JointVector {
        return &self.joints;
    }

    /// The number of joints
    pub fn getJointCount(self: *const Skeleton) usize {
        return self.joints.items.len;
    }

    /// A joint (GetJoint(int) const)
    pub fn getJoint(self: *const Skeleton, joint: usize) *const Joint {
        return &self.joints.items[joint];
    }

    /// A joint (the non-const GetJoint)
    pub fn getJointMut(self: *Skeleton, joint: usize) *Joint {
        return &self.joints.items[joint];
    }

    /// AddJoint(const string_view &inName, const string_view &inParentName = string_view()): adds a joint without a
    /// parent index (see calculateParentJointIndices), returns its index
    pub fn addJoint(self: *Skeleton, name: []const u8, opts: struct { parent_name: []const u8 = "" }) Allocator.Error!u32 {
        try self.joints.ensureUnusedCapacity(self.allocator, 1);
        self.joints.appendAssumeCapacity(try .init(self.allocator, name, opts.parent_name, -1));
        return @as(u32, @intCast(self.joints.items.len)) - 1;
    }

    /// AddJoint(const string_view &inName, int inParentIndex): adds a joint whose parent name is the name of joint
    /// `parent_index` (no parent name when it is negative), returns its index
    pub fn addJointWithParentIndex(self: *Skeleton, name: []const u8, parent_index: i32) Allocator.Error!u32 {
        try self.joints.ensureUnusedCapacity(self.allocator, 1);
        const parent_name: []const u8 = if (parent_index >= 0) self.joints.items[@intCast(parent_index)].name else "";
        self.joints.appendAssumeCapacity(try .init(self.allocator, name, parent_name, parent_index));
        return @as(u32, @intCast(self.joints.items.len)) - 1;
    }

    /// Find joint by name
    pub fn getJointIndex(self: *const Skeleton, name: []const u8) i32 {
        for (self.joints.items, 0..) |*j, i|
            if (std.mem.eql(u8, j.name, name))
                return @intCast(i);

        return -1;
    }

    /// Fill in parent joint indices based on name
    pub fn calculateParentJointIndices(self: *Skeleton) void {
        for (self.joints.items) |*j|
            j.parent_joint_index = self.getJointIndex(j.parent_name);
    }

    /// Many of the algorithms that use the Skeleton class require that parent joints are in the mJoints array before their children.
    /// This function returns true if this is the case, false if not.
    pub fn areJointsCorrectlyOrdered(self: *const Skeleton) bool {
        for (self.joints.items, 0..) |*j, i|
            if (j.parent_joint_index >= @as(i32, @intCast(i)))
                return false;

        return true;
    }

    /// Saves the state of this object in binary form to inStream.
    pub fn saveBinaryState(self: *const Skeleton, stream: StreamOut) void {
        stream.write(@as(u32, @intCast(self.joints.items.len)));
        for (self.joints.items) |*j| {
            stream.writeString(j.name);
            stream.write(j.parent_joint_index);
            stream.writeString(j.parent_name);
        }
    }

    /// Restore the state of this object from inStream (Jolt: sRestoreFromBinaryState; the result holds a reference to
    /// the new skeleton, `deinit` it)
    pub fn restoreFromBinaryState(allocator: Allocator, stream: StreamIn) Allocator.Error!SkeletonResult {
        var skeleton = Ref(Skeleton).init(try create(allocator));
        defer skeleton.deinit();
        const s = skeleton.get().?;

        // Jolt resizes the array to the length that is read without checking the stream (a short read gives the bytes
        // that were read), the new joints are default constructed
        var len: u32 = 0;
        stream.read(&len);
        try s.joints.ensureTotalCapacityPrecise(allocator, len);
        s.joints.appendNTimesAssumeCapacity(.{}, len);
        for (s.joints.items) |*j| {
            try stream.readString(allocator, &j.name);
            stream.read(&j.parent_joint_index);
            try stream.readString(allocator, &j.parent_name);
        }

        var result: SkeletonResult = .empty;
        if (stream.isEOF() or stream.isFailed())
            result.setError("Failed to read skeleton from stream")
        else
            result.set(skeleton.clone());
        return result;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests

const testing = std.testing;
const StreamWrapper = @import("../Core/StreamWrapper.zig");

/// A small humanoid: pelvis, spine, head, two arms and two legs (parents first)
fn buildHumanoid(skeleton: *Skeleton) !void {
    _ = try skeleton.addJoint("Pelvis", .{});
    _ = try skeleton.addJoint("Spine", .{ .parent_name = "Pelvis" });
    _ = try skeleton.addJoint("Head", .{ .parent_name = "Spine" });
    _ = try skeleton.addJointWithParentIndex("UpperArmL", 1);
    _ = try skeleton.addJointWithParentIndex("LowerArmL", 3);
    _ = try skeleton.addJointWithParentIndex("UpperArmR", 1);
    _ = try skeleton.addJointWithParentIndex("LowerArmR", 5);
    _ = try skeleton.addJoint("UpperLegL", .{ .parent_name = "Pelvis" });
    _ = try skeleton.addJoint("LowerLegL", .{ .parent_name = "UpperLegL" });
    _ = try skeleton.addJoint("UpperLegR", .{ .parent_name = "Pelvis" });
    _ = try skeleton.addJoint("LowerLegR", .{ .parent_name = "UpperLegR" });
    skeleton.calculateParentJointIndices();
}

fn saveSkeleton(skeleton: *const Skeleton, out: *std.Io.Writer.Allocating) ![]const u8 {
    var wrapper = StreamWrapper.StreamOutWrapper.init(&out.writer);
    skeleton.saveBinaryState(wrapper.streamOut());
    if (wrapper.isFailed()) return error.OutOfMemory; // The allocating writer could not grow
    return out.written();
}

fn restoreSkeleton(allocator: Allocator, bytes: []const u8) Allocator.Error!Skeleton.SkeletonResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return Skeleton.restoreFromBinaryState(allocator, in.streamIn());
}

test "Skeleton: addJoint, getJointIndex, calculateParentJointIndices" {
    var skeleton: Skeleton = .init(testing.allocator);
    defer skeleton.deinit();

    // Joints added by name have no parent index until calculateParentJointIndices
    try testing.expectEqual(@as(u32, 0), try skeleton.addJoint("Root", .{}));
    try testing.expectEqual(@as(u32, 1), try skeleton.addJoint("Child", .{ .parent_name = "Root" }));
    try testing.expectEqual(@as(i32, -1), skeleton.getJoint(1).parent_joint_index);
    try testing.expectEqualStrings("Root", skeleton.getJoint(1).parent_name);

    // A joint added by parent index takes the name of the parent, a negative index means no parent (and is kept)
    try testing.expectEqual(@as(u32, 2), try skeleton.addJointWithParentIndex("GrandChild", 1));
    try testing.expectEqualStrings("Child", skeleton.getJoint(2).parent_name);
    try testing.expectEqual(@as(i32, 1), skeleton.getJoint(2).parent_joint_index);
    try testing.expectEqual(@as(u32, 3), try skeleton.addJointWithParentIndex("Loose", -3));
    try testing.expectEqualStrings("", skeleton.getJoint(3).parent_name);
    try testing.expectEqual(@as(i32, -3), skeleton.getJoint(3).parent_joint_index);

    try testing.expectEqual(@as(usize, 4), skeleton.getJointCount());
    try testing.expectEqual(@as(usize, 4), skeleton.getJoints().len);
    try testing.expectEqual(@as(i32, 0), skeleton.getJointIndex("Root"));
    try testing.expectEqual(@as(i32, 2), skeleton.getJointIndex("GrandChild"));
    try testing.expectEqual(@as(i32, -1), skeleton.getJointIndex("Unknown"));
    try testing.expectEqual(@as(i32, -1), skeleton.getJointIndex(""));

    // Parent indices from the names: an unknown or empty parent name gives -1 (also for the joint that had -3)
    skeleton.calculateParentJointIndices();
    try testing.expectEqual(@as(i32, -1), skeleton.getJoint(0).parent_joint_index);
    try testing.expectEqual(@as(i32, 0), skeleton.getJoint(1).parent_joint_index);
    try testing.expectEqual(@as(i32, 1), skeleton.getJoint(2).parent_joint_index);
    try testing.expectEqual(@as(i32, -1), skeleton.getJoint(3).parent_joint_index);

    // The first joint with a name wins, an empty parent name finds a joint with an empty name
    _ = try skeleton.addJoint("Root", .{ .parent_name = "" });
    _ = try skeleton.addJoint("", .{ .parent_name = "Root" });
    skeleton.calculateParentJointIndices();
    try testing.expectEqual(@as(i32, 0), skeleton.getJointIndex("Root"));
    try testing.expectEqual(@as(i32, 5), skeleton.getJoint(4).parent_joint_index);
    try testing.expectEqual(@as(i32, 0), skeleton.getJoint(5).parent_joint_index);
    try testing.expectEqual(@as(i32, 5), skeleton.getJoint(3).parent_joint_index);

    // Mutable access
    skeleton.getJointMut(3).parent_joint_index = 1;
    try testing.expectEqual(@as(i32, 1), skeleton.getJoints()[3].parent_joint_index);
    try testing.expectEqual(@as(usize, 6), skeleton.getJointsMut().items.len);
}

test "Skeleton: areJointsCorrectlyOrdered" {
    var skeleton: Skeleton = .init(testing.allocator);
    defer skeleton.deinit();
    try testing.expect(skeleton.areJointsCorrectlyOrdered()); // Empty

    try buildHumanoid(&skeleton);
    try testing.expect(skeleton.areJointsCorrectlyOrdered());
    try testing.expectEqual(@as(i32, 3), skeleton.getJoint(4).parent_joint_index);
    try testing.expectEqual(@as(i32, 9), skeleton.getJoint(10).parent_joint_index);

    // A parent after its child, a joint that is its own parent
    _ = try skeleton.addJoint("Hand", .{ .parent_name = "Finger" });
    _ = try skeleton.addJoint("Finger", .{});
    skeleton.calculateParentJointIndices();
    try testing.expect(!skeleton.areJointsCorrectlyOrdered());
    skeleton.getJointMut(11).parent_joint_index = 11;
    try testing.expect(!skeleton.areJointsCorrectlyOrdered());
    skeleton.getJointMut(11).parent_joint_index = 4;
    try testing.expect(skeleton.areJointsCorrectlyOrdered());
}

test "Skeleton: saveBinaryState / restoreFromBinaryState" {
    const allocator = testing.allocator;
    var skeleton: Skeleton = .init(allocator);
    defer skeleton.deinit();
    try buildHumanoid(&skeleton);
    skeleton.getJointMut(7).parent_joint_index = -7; // Saved as is

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const bytes = try saveSkeleton(&skeleton, &out);

    // Jolt's layout: the joint count, then per joint the name, the parent index and the parent name
    try testing.expectEqual(@as(u32, 11), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(u32, 6), std.mem.readInt(u32, bytes[4..8], .little));
    try testing.expectEqualStrings("Pelvis", bytes[8..14]);
    try testing.expectEqual(@as(i32, -1), std.mem.readInt(i32, bytes[14..18], .little));
    try testing.expectEqual(@as(u32, 0), std.mem.readInt(u32, bytes[18..22], .little));

    var result = try restoreSkeleton(allocator, bytes);
    defer result.deinit();
    try testing.expect(result.isValid());
    const restored = result.getPtr().?;
    try testing.expectEqual(@as(u32, 1), restored.getRefCount());
    try testing.expectEqual(skeleton.getJointCount(), restored.getJointCount());
    for (skeleton.getJoints(), restored.getJoints()) |*a, *b| {
        try testing.expectEqualStrings(a.name, b.name);
        try testing.expectEqualStrings(a.parent_name, b.parent_name);
        try testing.expectEqual(a.parent_joint_index, b.parent_joint_index);
    }

    // Saving the restored skeleton gives the same bytes
    var out2: std.Io.Writer.Allocating = .init(allocator);
    defer out2.deinit();
    try testing.expectEqualSlices(u8, bytes, try saveSkeleton(restored, &out2));

    // An empty skeleton
    var empty: Skeleton = .init(allocator);
    defer empty.deinit();
    var out3: std.Io.Writer.Allocating = .init(allocator);
    defer out3.deinit();
    const empty_bytes = try saveSkeleton(&empty, &out3);
    try testing.expectEqualSlices(u8, &.{ 0, 0, 0, 0 }, empty_bytes);
    var empty_result = try restoreSkeleton(allocator, empty_bytes);
    defer empty_result.deinit();
    try testing.expectEqual(@as(usize, 0), empty_result.getPtr().?.getJointCount());

    // Every truncated stream is an error (the stream ends while it is read), nothing leaks
    for (0..bytes.len) |len| {
        var truncated = try restoreSkeleton(allocator, bytes[0..len]);
        defer truncated.deinit();
        try testing.expectEqualStrings("Failed to read skeleton from stream", truncated.getError());
    }

    // A joint count that cannot be allocated (a corrupt stream, Jolt's resize aborts) is error.OutOfMemory
    var corrupt = [_]u8{ 0xff, 0xff, 0xff, 0x7f };
    var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = 1 });
    try testing.expectError(error.OutOfMemory, restoreSkeleton(failing.allocator(), &corrupt));
}

fn buildAndSaveOutOfMemory(allocator: Allocator) !void {
    var skeleton = Ref(Skeleton).init(try Skeleton.create(allocator));
    defer skeleton.deinit();
    try buildHumanoid(skeleton.get().?);

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    const bytes = try saveSkeleton(skeleton.get().?, &out);
    var result = try restoreSkeleton(allocator, bytes);
    defer result.deinit();
    try testing.expect(result.isValid());
}

test "Skeleton: out of memory in addJoint and restoreFromBinaryState" {
    try testing.checkAllAllocationFailures(testing.allocator, buildAndSaveOutOfMemory, .{});

    // A failed addJoint adds nothing (the failing allocator forwards to the skeleton's allocator, so the skeleton can
    // use it for a while)
    for ([_]bool{ false, true }) |by_index| {
        for (0..3) |fail_index| {
            var skeleton: Skeleton = .init(testing.allocator);
            defer skeleton.deinit();
            _ = try skeleton.addJoint("Root", .{});
            var failing = std.testing.FailingAllocator.init(testing.allocator, .{ .fail_index = fail_index });
            skeleton.allocator = failing.allocator();
            const added = if (by_index) skeleton.addJointWithParentIndex("Child", 0) else skeleton.addJoint("Child", .{ .parent_name = "Root" });
            skeleton.allocator = testing.allocator;
            if (added) |index| {
                try testing.expectEqual(@as(u32, 1), index);
                try testing.expectEqualStrings("Root", skeleton.getJoint(1).parent_name);
            } else |err| {
                try testing.expectEqual(error.OutOfMemory, err);
                try testing.expectEqual(@as(usize, 1), skeleton.getJointCount());
            }
        }
    }
}

test "Skeleton: reference counting" {
    const allocator = testing.allocator;
    var skeleton = Ref(Skeleton).init(try Skeleton.create(allocator));
    defer skeleton.deinit();
    try buildHumanoid(skeleton.get().?);
    var second = skeleton.clone();
    try testing.expectEqual(@as(u32, 2), skeleton.get().?.getRefCount());
    second.deinit();
    try testing.expectEqual(@as(u32, 1), skeleton.get().?.getRefCount());

    // An embedded skeleton is not destroyed by its last release
    var embedded: Skeleton = .init(allocator);
    embedded.setEmbedded();
    {
        var ref = Ref(Skeleton).init(&embedded);
        ref.deinit();
    }
    _ = try embedded.addJoint("Root", .{});
    embedded.deinit();
}
