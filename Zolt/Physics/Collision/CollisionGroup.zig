//! Port of: Jolt/Physics/Collision/CollisionGroup.h, Jolt/Physics/Collision/CollisionGroup.cpp
//! Status: complete
//!
//! A value type that holds a reference to its group filter (`RefConst(GroupFilter)`, Jolt's `RefConst<GroupFilter>`):
//! - `CollisionGroup()` is `.{}` (no filter, invalid group and sub group), the member wise constructor is `init`;
//! - Zig has no copy constructor: `clone()` copies (adds a reference to the filter), `assign(&other)` is the copy
//!   assignment, `deinit()` the destructor (releases the filter) and a plain `=` moves;
//! - `operator ==` is `eql` (`!=` is `!a.eql(b)`);
//! - `sInvalid` is the compile time constant `invalid`.

const std = @import("std");
const RefConst = @import("../../Core/Reference.zig").RefConst;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const GroupFilter = @import("GroupFilter.zig").GroupFilter;

/// Two objects collide with each other if:
/// - Both don't have a group filter
/// - The first group filter says that the objects can collide
/// - Or if there's no filter for the first object, the second group filter says the objects can collide
pub const CollisionGroup = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, CollisionGroup)

    pub const GroupID = u32;
    pub const SubGroupID = u32;

    pub const invalid_group: GroupID = ~@as(GroupID, 0);
    pub const invalid_sub_group: SubGroupID = ~@as(SubGroupID, 0);

    group_filter: RefConst(GroupFilter) = .empty,
    group_id: GroupID = invalid_group,
    sub_group_id: SubGroupID = invalid_sub_group,

    /// An invalid collision group
    pub const invalid: CollisionGroup = .{};

    /// Construct with all properties
    pub fn init(filter: ?*const GroupFilter, group_id: GroupID, sub_group_id: SubGroupID) CollisionGroup {
        return .{ .group_filter = .init(filter), .group_id = group_id, .sub_group_id = sub_group_id };
    }

    /// Copy constructor (adds a reference to the group filter)
    pub fn clone(self: *const CollisionGroup) CollisionGroup {
        return .{ .group_filter = self.group_filter.clone(), .group_id = self.group_id, .sub_group_id = self.sub_group_id };
    }

    /// Copy assignment (operator =)
    pub fn assign(self: *CollisionGroup, other: *const CollisionGroup) void {
        self.group_filter.set(other.group_filter.get());
        self.group_id = other.group_id;
        self.sub_group_id = other.sub_group_id;
    }

    /// Destructor (releases the group filter)
    pub fn deinit(self: *CollisionGroup) void {
        self.group_filter.deinit();
    }

    /// Test if two collision groups are equal
    pub fn eql(self: *const CollisionGroup, other: *const CollisionGroup) bool {
        return self.group_filter.eql(other.group_filter) and self.group_id == other.group_id and self.sub_group_id == other.sub_group_id;
    }

    /// Set the collision group filter
    pub fn setGroupFilter(self: *CollisionGroup, filter: ?*const GroupFilter) void {
        self.group_filter.set(filter);
    }

    /// Get the collision group filter
    pub fn getGroupFilter(self: *const CollisionGroup) ?*const GroupFilter {
        return self.group_filter.get();
    }

    /// Set the main group id for this object
    pub fn setGroupID(self: *CollisionGroup, id: GroupID) void {
        self.group_id = id;
    }

    pub fn getGroupID(self: *const CollisionGroup) GroupID {
        return self.group_id;
    }

    /// Add this object to a sub group
    pub fn setSubGroupID(self: *CollisionGroup, id: SubGroupID) void {
        self.sub_group_id = id;
    }

    pub fn getSubGroupID(self: *const CollisionGroup) SubGroupID {
        return self.sub_group_id;
    }

    /// Check if this object collides with another object
    pub fn canCollide(self: *const CollisionGroup, other: *const CollisionGroup) bool {
        // Call the CanCollide function of the first group filter that's not null
        if (self.group_filter.get()) |filter|
            return filter.canCollide(self, other)
        else if (other.group_filter.get()) |filter|
            return filter.canCollide(other, self)
        else
            return true;
    }

    /// Saves the state of this object in binary form to inStream. Does not save group filter.
    pub fn saveBinaryState(self: *const CollisionGroup, stream: StreamOut) void {
        stream.write(self.group_id);
        stream.write(self.sub_group_id);
    }

    /// Restore the state of this object from inStream. Does not save group filter.
    pub fn restoreBinaryState(self: *CollisionGroup, stream: StreamIn) void {
        stream.read(&self.group_id);
        stream.read(&self.sub_group_id);
    }
};

const Ref = @import("../../Core/Reference.zig").Ref;
const GroupFilterTable = @import("GroupFilterTable.zig").GroupFilterTable;
const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;

test "CollisionGroup: defaults, accessors, copies, equality" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    // Default constructed / sInvalid
    var group: CollisionGroup = .{};
    defer group.deinit();
    try expect(group.getGroupFilter() == null);
    try std.testing.expectEqual(CollisionGroup.invalid_group, group.getGroupID());
    try std.testing.expectEqual(CollisionGroup.invalid_sub_group, group.getSubGroupID());
    try std.testing.expectEqual(@as(u32, 0xffffffff), CollisionGroup.invalid_group);
    try expect(group.eql(&CollisionGroup.invalid));
    try expect(CollisionGroup.invalid.canCollide(&CollisionGroup.invalid)); // No filters

    const filter = try GroupFilterTable.create(allocator, .{ .num_sub_groups = 2 });
    var filter_ref = Ref(GroupFilter).init(filter.asGroupFilterMut());
    defer filter_ref.deinit();

    // Setters
    group.setGroupFilter(filter.asGroupFilter());
    group.setGroupID(3);
    group.setSubGroupID(1);
    try expect(group.getGroupFilter() == filter.asGroupFilter());
    try std.testing.expectEqual(@as(u32, 3), group.getGroupID());
    try std.testing.expectEqual(@as(u32, 1), group.getSubGroupID());
    try std.testing.expectEqual(@as(u32, 2), filter.base.getRefCount());
    try expect(!group.eql(&CollisionGroup.invalid));

    // Copies reference the filter
    var copy = group.clone();
    try std.testing.expectEqual(@as(u32, 3), filter.base.getRefCount());
    try expect(copy.eql(&group));
    copy.setSubGroupID(0);
    try expect(!copy.eql(&group));
    try expect(copy.canCollide(&group)); // Same filter and group, different sub groups: the table decides
    filter.disableCollision(0, 1);
    try expect(!copy.canCollide(&group));
    filter.enableCollision(0, 1);
    try expect(copy.canCollide(&group));
    try expect(group.canCollide(&copy));

    var assigned = CollisionGroup.init(null, 5, 5);
    defer assigned.deinit();
    assigned.assign(&group);
    try expect(assigned.eql(&group));
    try std.testing.expectEqual(@as(u32, 4), filter.base.getRefCount());
    assigned.assign(&CollisionGroup.invalid);
    try std.testing.expectEqual(@as(u32, 3), filter.base.getRefCount());
    try expect(assigned.eql(&CollisionGroup.invalid));
    assigned.assign(&assigned); // Self assignment
    try expect(assigned.eql(&CollisionGroup.invalid));

    copy.deinit();
    try std.testing.expectEqual(@as(u32, 2), filter.base.getRefCount());

    // A group without a filter uses the filter of the other group (with the groups swapped)
    var no_filter = CollisionGroup.init(null, 3, 0);
    defer no_filter.deinit();
    try expect(!no_filter.canCollide(&group)); // Same group ID, different filters
    try expect(!group.canCollide(&no_filter));
    no_filter.setGroupID(4);
    try expect(no_filter.canCollide(&group)); // Different group IDs
    try expect(group.canCollide(&no_filter));
}

test "CollisionGroup: binary state" {
    const expect = std.testing.expect;

    const group = CollisionGroup.init(null, 0x01020304, 0x05060708);
    var buffer: [16]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamOutWrapper.init(&writer);
    group.saveBinaryState(out.streamOut());
    try std.testing.expectEqualSlices(u8, &.{ 4, 3, 2, 1, 8, 7, 6, 5 }, writer.buffered());

    var restored: CollisionGroup = .{};
    var reader: std.Io.Reader = .fixed(writer.buffered());
    var in = StreamInWrapper.init(&reader);
    restored.restoreBinaryState(in.streamIn());
    try expect(!in.isEOF() and !in.isFailed());
    try expect(restored.eql(&group));

    // A truncated stream: the available bytes are read
    var truncated: CollisionGroup = .{};
    reader = .fixed(writer.buffered()[0..6]);
    in = StreamInWrapper.init(&reader);
    truncated.restoreBinaryState(in.streamIn());
    try expect(in.isEOF() and in.isFailed());
    try std.testing.expectEqual(@as(u32, 0x01020304), truncated.getGroupID());
    try std.testing.expectEqual(@as(u32, 0xffff0708), truncated.getSubGroupID());
}
