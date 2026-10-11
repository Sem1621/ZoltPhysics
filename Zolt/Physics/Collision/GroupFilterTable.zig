//! Port of: Jolt/Physics/Collision/GroupFilterTable.h, Jolt/Physics/Collision/GroupFilterTable.cpp
//! Status: complete
//!
//! A final GroupFilter (Docs/Zolt/CollisionArchitecture.md, D2 and D7) that owns its bit table, allocated with the
//! allocator stored in the base class.
//!
//! Constructors: `GroupFilterTable(uint inNumSubGroups = 0)` is `init(allocator, .{ .num_sub_groups = n })` (on the
//! stack or as a member: `base.setEmbedded()` before references are taken, `deinit()` at the end) /
//! `create(allocator, .{ .num_sub_groups = n })` (`new`, reference count 0, put it in a `Ref(GroupFilter)` via
//! `asGroupFilterMut()`). The copy constructor is `clone(allocator)` (the copy gets reference count 0, like
//! RefTarget's copy constructor).
//!
//! The bit index arithmetic is done in `u32` with wrapping operations like Jolt's unsigned arithmetic (GetBit returns
//! an `int` in Jolt, every index inside the table is below 2^31 so the value is the same).

const std = @import("std");
const Allocator = std.mem.Allocator;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const GroupFilter = @import("GroupFilter.zig").GroupFilter;
const CollisionGroup = @import("CollisionGroup.zig").CollisionGroup;

/// Implementation of GroupFilter that stores a bit table with one bit per sub shape ID pair to determine if they collide or not
///
/// The collision rules:
/// - If one of the objects is in the cInvalidGroup the objects will collide.
/// - If the objects are in different groups they will collide.
/// - If they're in the same group but their collision filter is different they will not collide.
/// - If they're in the same group and their collision filters match, we'll use the SubGroupID and the table below.
///
/// For N = 6 sub groups the table will look like:
///
///                    sub group 1 --->
///        sub group 2 x.....
///             |      ox....
///             |      oox...
///             V      ooox..
///                    oooox.
///                    ooooox
///
/// * 'x' means sub group 1 == sub group 2 and we define this to never collide.
/// * 'o' is a bit that we have to store that defines if the sub groups collide or not.
/// * '.' is a bit we don't need to store because the table is symmetric, we take care that group 2 > group 1 by swapping sub group 1 and sub group 2 if needed.
///
/// The total number of bits we need to store is (N * (N - 1)) / 2
pub const GroupFilterTable = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, GroupFilterTable)

    pub const overrides = .{ .canCollide, .saveBinaryState, .restoreBinaryState };

    /// Name of the class for Jolt's RTTI hash (JPH_RTTI)
    pub const rtti_name = "GroupFilterTable";

    const GroupID = CollisionGroup.GroupID;
    const SubGroupID = CollisionGroup.SubGroupID;

    base: GroupFilter,
    /// The number of subgroups that this group filter supports
    num_sub_groups: u32 = 0,
    /// The table of bits that indicates which pairs collide (owned by base.allocator)
    table: std.ArrayList(u8) = .empty,

    /// Get which bit corresponds to the pair (inSubGroup1, inSubGroup2)
    fn getBit(self: *const GroupFilterTable, sub_group1_in: SubGroupID, sub_group2_in: SubGroupID) u32 {
        var sub_group1 = sub_group1_in;
        var sub_group2 = sub_group2_in;

        std.debug.assert(sub_group1 != sub_group2);

        // We store the lower left half only, so swap the inputs when trying to access the top right half
        if (sub_group1 > sub_group2)
            std.mem.swap(SubGroupID, &sub_group1, &sub_group2);

        std.debug.assert(sub_group2 < self.num_sub_groups);

        // Calculate at which bit the entry for this pair resides
        // We use the fact that a row always starts at inSubGroup2 * (inSubGroup2 - 1) / 2
        // (this is the amount of bits needed to store a table of inSubGroup2 entries)
        return (sub_group2 *% (sub_group2 -% 1)) / 2 +% sub_group1;
    }

    /// Constructs the table with inNumSubGroups subgroups, initially all collision pairs are enabled except when the sub group ID is the same
    pub fn init(allocator: Allocator, opts: struct { num_sub_groups: u32 = 0 }) Allocator.Error!GroupFilterTable {
        var self: GroupFilterTable = .{ .base = .init(GroupFilterTable, allocator), .num_sub_groups = opts.num_sub_groups };

        // By default everything collides
        const table_size: u32 = ((opts.num_sub_groups *% (opts.num_sub_groups -% 1)) / 2 + 7) / 8;
        try self.table.appendNTimes(allocator, 0xff, table_size);
        return self;
    }

    /// new GroupFilterTable(inNumSubGroups): reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator, opts: struct { num_sub_groups: u32 = 0 }) Allocator.Error!*GroupFilterTable {
        const self = try allocator.create(GroupFilterTable);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator, .{ .num_sub_groups = opts.num_sub_groups });
        return self;
    }

    /// new GroupFilterTable() (the default constructor, used by GroupFilter.restoreFromBinaryState)
    pub fn createDefault(allocator: Allocator) Allocator.Error!*GroupFilterTable {
        return create(allocator, .{});
    }

    /// Copy constructor: a new filter owned by `allocator` with the same table (reference count 0)
    pub fn clone(self: *const GroupFilterTable, allocator: Allocator) Allocator.Error!GroupFilterTable {
        return .{ .base = .init(GroupFilterTable, allocator), .num_sub_groups = self.num_sub_groups, .table = try self.table.clone(allocator) };
    }

    /// ~GroupFilterTable
    pub fn destruct(self: *GroupFilterTable) void {
        self.table.deinit(self.base.allocator);
    }

    /// Destructor of a filter that is not on the heap (embedded / stack), see GroupFilter.deinit
    pub fn deinit(self: *GroupFilterTable) void {
        self.base.deinit();
    }

    /// Implicit upcast to the base class
    pub fn asGroupFilter(self: *const GroupFilterTable) *const GroupFilter {
        return &self.base;
    }

    /// Implicit upcast to the base class (mutable)
    pub fn asGroupFilterMut(self: *GroupFilterTable) *GroupFilter {
        return &self.base;
    }

    /// Disable collision between two sub groups
    pub fn disableCollision(self: *GroupFilterTable, sub_group1: SubGroupID, sub_group2: SubGroupID) void {
        const bit = self.getBit(sub_group1, sub_group2);
        self.table.items[bit >> 3] &= (0xff ^ (@as(u8, 1) << @as(u3, @intCast(bit & 0b111))));
    }

    /// Enable collision between two sub groups
    pub fn enableCollision(self: *GroupFilterTable, sub_group1: SubGroupID, sub_group2: SubGroupID) void {
        const bit = self.getBit(sub_group1, sub_group2);
        self.table.items[bit >> 3] |= @as(u8, 1) << @as(u3, @intCast(bit & 0b111));
    }

    /// Check if the collision between two subgroups is enabled
    pub fn isCollisionEnabled(self: *const GroupFilterTable, sub_group1: SubGroupID, sub_group2: SubGroupID) bool {
        // Test if the bit is set for this group pair
        const bit = self.getBit(sub_group1, sub_group2);
        return (self.table.items[bit >> 3] & (@as(u8, 1) << @as(u3, @intCast(bit & 0b111)))) != 0;
    }

    /// Checks if two CollisionGroups collide
    pub fn canCollide(self: *const GroupFilterTable, group1: *const CollisionGroup, group2: *const CollisionGroup) bool {
        // If one of the groups is cInvalidGroup the objects will collide (note that the if following this if will ensure that group2 is not cInvalidGroup)
        if (group1.getGroupID() == CollisionGroup.invalid_group)
            return true;

        // If the objects are in different groups, they collide
        if (group1.getGroupID() != group2.getGroupID())
            return true;

        // If the collision filters do not match, but they're in the same group we ignore the collision
        if (group1.getGroupFilter() != group2.getGroupFilter())
            return false;

        // If they are in the same sub group, they don't collide
        if (group1.getSubGroupID() == group2.getSubGroupID())
            return false;

        // Check the bit table
        return self.isCollisionEnabled(group1.getSubGroupID(), group2.getSubGroupID());
    }

    // See: GroupFilter::SaveBinaryState
    pub fn saveBinaryState(self: *const GroupFilterTable, stream: StreamOut) void {
        GroupFilter.impl.saveBinaryState(&self.base, stream);

        stream.write(self.num_sub_groups);
        stream.writeArray(u8, self.table.items);
    }

    // See: GroupFilter::RestoreBinaryState
    pub fn restoreBinaryState(self: *GroupFilterTable, stream: StreamIn) Allocator.Error!void {
        try GroupFilter.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.num_sub_groups);
        try stream.readArray(u8, self.base.allocator, &self.table);
    }
};

const Ref = @import("../../Core/Reference.zig").Ref;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const RefCount = @import("../../Core/Reference.zig").RefCount;
const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;

/// Saves `filter` into `buffer`, returns the written bytes
fn saveToBuffer(filter: *const GroupFilter, buffer: []u8) []const u8 {
    var writer: std.Io.Writer = .fixed(buffer);
    var out = StreamOutWrapper.init(&writer);
    filter.saveBinaryState(out.streamOut());
    return writer.buffered();
}

/// Restores a group filter from `bytes`
fn restoreFromBuffer(allocator: Allocator, bytes: []const u8) Allocator.Error!GroupFilter.GroupFilterResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    return GroupFilter.restoreFromBinaryState(allocator, in.streamIn());
}

test "GroupFilterTable: table layout and bits" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    // Table sizes: (N * (N - 1)) / 2 bits, rounded up to bytes (N = 0 wraps around in unsigned arithmetic like Jolt)
    const sizes = [_][2]u32{ .{ 0, 0 }, .{ 1, 0 }, .{ 2, 1 }, .{ 4, 1 }, .{ 5, 2 }, .{ 6, 2 }, .{ 10, 6 }, .{ 100, 619 } };
    for (sizes) |s| {
        var table = try GroupFilterTable.init(allocator, .{ .num_sub_groups = s[0] });
        defer table.deinit();
        try std.testing.expectEqual(s[1], @as(u32, @intCast(table.table.items.len)));
        for (table.table.items) |b| try std.testing.expectEqual(@as(u8, 0xff), b);
    }

    var table = try GroupFilterTable.init(allocator, .{ .num_sub_groups = 6 });
    defer table.deinit();

    // The bit of every pair is unique and symmetric
    var seen: [15]bool = @splat(false);
    for (0..6) |i| for (0..6) |j| {
        if (i == j) continue;
        const bit = table.getBit(@intCast(i), @intCast(j));
        try std.testing.expectEqual(bit, table.getBit(@intCast(j), @intCast(i)));
        if (i < j) {
            try expect(!seen[bit]);
            seen[bit] = true;
        }
    };
    for (seen) |s| try expect(s);

    // Everything collides initially, disable / enable are symmetric
    try expect(table.isCollisionEnabled(1, 4));
    table.disableCollision(4, 1);
    try expect(!table.isCollisionEnabled(1, 4));
    try expect(!table.isCollisionEnabled(4, 1));
    try expect(table.isCollisionEnabled(1, 3));
    try expect(table.isCollisionEnabled(0, 4));
    table.enableCollision(1, 4);
    try expect(table.isCollisionEnabled(4, 1));
    table.disableCollision(5, 0);
    try expect(!table.isCollisionEnabled(0, 5));
}

test "GroupFilterTable: binary state, copy" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    const filter = try GroupFilterTable.create(allocator, .{ .num_sub_groups = 10 });
    var filter_ref = Ref(GroupFilter).init(filter.asGroupFilterMut());
    defer filter_ref.deinit();
    filter.disableCollision(1, 2);
    filter.disableCollision(9, 5);

    // RTTI hash, the number of sub groups, the table (length + bytes)
    var buffer: [64]u8 = undefined;
    const bytes = saveToBuffer(filter.asGroupFilter(), &buffer);
    try std.testing.expectEqual(@as(usize, 4 + 4 + 4 + 6), bytes.len);
    try std.testing.expectEqualSlices(u8, std.mem.asBytes(&GroupFilter.rttiHash("GroupFilterTable")), bytes[0..4]);
    try std.testing.expectEqual(GroupFilter.rttiHash("GroupFilterTable"), filter.asGroupFilter().getRTTIHash());

    var restored = try restoreFromBuffer(allocator, bytes);
    defer restored.deinit();
    try expect(restored.isValid());
    const restored_table: *const GroupFilterTable = @alignCast(@fieldParentPtr("base", restored.getPtr().?));
    try std.testing.expectEqual(@as(u32, 10), restored_table.num_sub_groups);
    try std.testing.expectEqualSlices(u8, filter.table.items, restored_table.table.items);
    try expect(!restored_table.isCollisionEnabled(2, 1));
    try expect(!restored_table.isCollisionEnabled(5, 9));
    try expect(restored_table.isCollisionEnabled(5, 8));

    // Truncated streams
    for (4..bytes.len) |len| {
        var truncated = try restoreFromBuffer(allocator, bytes[0..len]);
        defer truncated.deinit();
        try std.testing.expectEqualStrings("Failed to restore object", truncated.getError());
    }

    // Restoring into an existing table replaces its contents
    var other = try GroupFilterTable.init(allocator, .{ .num_sub_groups = 3 });
    defer other.deinit();
    var reader: std.Io.Reader = .fixed(bytes[4..]); // Skip the RTTI hash like restoreFromBinaryState
    var in = StreamInWrapper.init(&reader);
    try other.asGroupFilterMut().restoreBinaryState(in.streamIn());
    try std.testing.expectEqual(@as(u32, 10), other.num_sub_groups);
    try std.testing.expectEqualSlices(u8, filter.table.items, other.table.items);

    // Copy constructor
    var copy = try filter.clone(allocator);
    copy.base.setEmbedded();
    defer copy.deinit();
    try std.testing.expectEqual(RefCount.embedded, copy.base.getRefCount()); // The reference count is not copied
    var copy_bytes_buffer: [64]u8 = undefined;
    try std.testing.expectEqualSlices(u8, bytes, saveToBuffer(copy.asGroupFilter(), &copy_bytes_buffer));
    copy.enableCollision(1, 2);
    try expect(!filter.isCollisionEnabled(1, 2));

    // The default constructed table
    const default = try GroupFilterTable.createDefault(allocator);
    var default_ref = RefConst(GroupFilter).init(default.asGroupFilter());
    defer default_ref.deinit();
    try std.testing.expectEqual(@as(u32, 0), default.num_sub_groups);
    try std.testing.expectEqual(@as(usize, 0), default.table.items.len);
}

test "GroupFilterTable: CanCollide rules" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    var filter1 = try GroupFilterTable.init(allocator, .{ .num_sub_groups = 4 });
    filter1.base.setEmbedded();
    defer filter1.deinit();
    var filter2 = try GroupFilterTable.init(allocator, .{ .num_sub_groups = 4 });
    filter2.base.setEmbedded();
    defer filter2.deinit();
    filter1.disableCollision(0, 1);

    const invalid = CollisionGroup.invalid_group;
    var g_invalid = CollisionGroup.init(filter1.asGroupFilter(), invalid, 0);
    defer g_invalid.deinit();
    var g_a0 = CollisionGroup.init(filter1.asGroupFilter(), 7, 0);
    defer g_a0.deinit();
    var g_a1 = CollisionGroup.init(filter1.asGroupFilter(), 7, 1);
    defer g_a1.deinit();
    var g_a2 = CollisionGroup.init(filter1.asGroupFilter(), 7, 2);
    defer g_a2.deinit();
    var g_b1 = CollisionGroup.init(filter2.asGroupFilter(), 7, 1);
    defer g_b1.deinit();
    var g_other = CollisionGroup.init(filter1.asGroupFilter(), 8, 1);
    defer g_other.deinit();

    // Invalid group: always collides (also with itself)
    try expect(filter1.canCollide(&g_invalid, &g_a0));
    try expect(filter1.canCollide(&g_invalid, &g_invalid));
    // Different groups
    try expect(filter1.canCollide(&g_a0, &g_other));
    try expect(filter1.canCollide(&g_a0, &g_invalid));
    // Same group, different filters
    try expect(!filter1.canCollide(&g_a1, &g_b1));
    // Same group, same sub group
    try expect(!filter1.canCollide(&g_a1, &g_a1));
    // The table
    try expect(!filter1.canCollide(&g_a0, &g_a1));
    try expect(!filter1.canCollide(&g_a1, &g_a0));
    try expect(filter1.canCollide(&g_a0, &g_a2));
    try expect(filter1.canCollide(&g_a2, &g_a1));
}

test "GroupFilterTable: out of memory" {
    const allocator = std.testing.allocator;

    // create: every allocation failure is returned and nothing leaks
    var fail_index: usize = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        const filter = GroupFilterTable.create(failing.allocator(), .{ .num_sub_groups = 10 }) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        var ref = RefConst(GroupFilter).init(filter.asGroupFilter());
        ref.deinit();
        try std.testing.expectEqual(@as(usize, 2), fail_index); // The object and its table
        break;
    }

    // init and clone: the same
    var table = try GroupFilterTable.init(allocator, .{ .num_sub_groups = 10 });
    defer table.deinit();
    fail_index = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var copy = table.clone(failing.allocator()) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        copy.deinit();
        try std.testing.expectEqual(@as(usize, 1), fail_index); // The table
        break;
    }
    fail_index = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var stack = GroupFilterTable.init(failing.allocator(), .{ .num_sub_groups = 10 }) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        stack.deinit();
        try std.testing.expectEqual(@as(usize, 1), fail_index); // The table
        break;
    }

    // restoreFromBinaryState: the same
    var buffer: [64]u8 = undefined;
    const bytes = saveToBuffer(table.asGroupFilter(), &buffer);
    fail_index = 0;
    while (true) : (fail_index += 1) {
        var failing = std.testing.FailingAllocator.init(allocator, .{ .fail_index = fail_index });
        var result = restoreFromBuffer(failing.allocator(), bytes) catch |err| {
            try std.testing.expectEqual(error.OutOfMemory, err);
            continue;
        };
        defer result.deinit();
        try std.testing.expect(result.isValid());
        try std.testing.expectEqual(@as(usize, 2), fail_index); // The object (its default table is empty) and the restored table
        break;
    }
}
