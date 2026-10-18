//! Port of: Jolt/Physics/Collision/GroupFilter.h, Jolt/Physics/Collision/GroupFilter.cpp
//! Status: complete
//!
//! Pattern A RefTarget root like PhysicsMaterial (Docs/Zolt/CollisionArchitecture.md, D1, D2, D7 and D8): the vtable,
//! the atomic reference count and the allocator that frees the object (and the memory it owns).
//!
//! - GroupFilter is abstract (`CanCollide` is pure virtual), so it keeps the bodies of its other virtual functions in
//!   `impl`. A derived filter has a field `base: GroupFilter` and its constructor sets `.base = .init(@This(),
//!   allocator)` (the protected base class constructor). It lists its `overrides` and declares `pub const rtti`
//!   (JPH_RTTI, SerializationArchitecture.md S3). `new X(...)` is `X.create(allocator, ...)` (reference count 0, put
//!   it in a `Ref` / `RefConst`), the last `release()` destroys it through the vtable. A filter on the stack or
//!   embedded in another object calls `setEmbedded()` before references are taken and `deinit()` at the end.
//! - SaveBinaryState writes Jolt's RTTI hash of the most derived class (the `rtti` vtable data entry).
//!   sRestoreFromBinaryState (StreamUtils::RestoreObject) finds the class in `RegisterTypes.factory`. Restoring
//!   allocates, so it returns `Allocator.Error!GroupFilterResult`. Jolt's errors ("Failed to read type hash", ...) are
//!   values in the result. An unknown hash, the hash of the abstract GroupFilter itself and the hash of a registered
//!   class that is not a group filter (Jolt dereferences a null pointer or reinterprets an unrelated object) give
//!   "Failed to create instance of type" (D8).
//! - The copy constructor and copy assignment of the C++ base class (protected, they do not copy the reference
//!   count) are part of the derived classes' copy functions, e.g. `GroupFilterTable.clone`.
//! - User group filter classes register through `serializable_types` of the `zolt_user_types` module (Jolt: the
//!   Factory registration of the application).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const SerializableObject = @import("../../ObjectStream/SerializableObject.zig").SerializableObject;
const RefCount = @import("../../Core/Reference.zig").RefCount;
const Ref = @import("../../Core/Reference.zig").Ref;
const Result = @import("../../Core/Result.zig").Result;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const StreamUtils = @import("../../Core/StreamUtils.zig");
const virtual = @import("../../Core/Virtual.zig");
const CollisionGroup = @import("CollisionGroup.zig").CollisionGroup;

/// Abstract class that checks if two CollisionGroups collide
pub const GroupFilter = struct {
    /// JPH_DECLARE_SERIALIZABLE_ABSTRACT / JPH_IMPLEMENT_SERIALIZABLE_ABSTRACT_BASE(GroupFilter)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "GroupFilter",
        .kind = .abstract,
        .bases = &.{SerializableObject},
    });
    /// TODO(serialization): the attributes of Jolt's sCreateRTTI (SerializationArchitecture.md S18)
    pub const serialization_pending = true;

    pub const VTable = struct {
        /// GetRTTI(): JPH_RTTI of the most derived class (a data entry: every concrete class declares `rtti`)
        rtti: *const RTTI,
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *GroupFilter) void,
        /// delete this (generated)
        destroy: *const fn (self: *GroupFilter) void,
        /// Check if two groups collide
        canCollide: *const fn (self: *const GroupFilter, group1: *const CollisionGroup, group2: *const CollisionGroup) bool,
        /// Saves the contents of the group filter in binary form to inStream.
        saveBinaryState: *const fn (self: *const GroupFilter, stream: StreamOut) void,
        /// This function should not be called directly, it is used by sRestoreFromBinaryState (protected in Jolt).
        restoreBinaryState: *const fn (self: *GroupFilter, stream: StreamIn) Allocator.Error!void,
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<GroupFilter>)
    ref_count: RefCount = .{},
    /// Frees this filter (heap filters) and the memory it owns
    allocator: Allocator,

    pub const GroupFilterResult = Result(Ref(GroupFilter));

    /// Constructor (protected in Jolt), called by derived classes with their most derived type
    pub fn init(comptime T: type, allocator: Allocator) GroupFilter {
        return .{ .vtable = vtableFor(T), .allocator = allocator };
    }

    /// The vtable of group filter class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<GroupFilter>

    /// Add a reference to this object
    pub fn addRef(self: *const GroupFilter) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const GroupFilter) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark this filter as embedded (on the stack or a member), the last release will not destroy it
    pub fn setEmbedded(self: *const GroupFilter) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const GroupFilter) u32 {
        return self.ref_count.get();
    }

    /// Destructor of a filter that is not on the heap (embedded / stack)
    pub fn deinit(self: *GroupFilter) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// Check if two groups collide
    pub fn canCollide(self: *const GroupFilter, group1: *const CollisionGroup, group2: *const CollisionGroup) bool {
        return self.vtable.canCollide(self, group1, group2);
    }

    /// Saves the contents of the group filter in binary form to inStream.
    pub fn saveBinaryState(self: *const GroupFilter, stream: StreamOut) void {
        self.vtable.saveBinaryState(self, stream);
    }

    /// This function should not be called directly, it is used by sRestoreFromBinaryState (protected in Jolt).
    pub fn restoreBinaryState(self: *GroupFilter, stream: StreamIn) Allocator.Error!void {
        return self.vtable.restoreBinaryState(self, stream);
    }

    /// Creates a GroupFilter of the correct type and restores its contents from the binary stream inStream.
    pub fn restoreFromBinaryState(allocator: Allocator, stream: StreamIn) Allocator.Error!GroupFilterResult {
        return StreamUtils.restoreObject(GroupFilter, allocator, stream, GroupFilter.restoreBinaryState);
    }

    /// GetRTTI()->GetHash() of the most derived class
    pub fn getRTTIHash(self: *const GroupFilter) u32 {
        return self.vtable.rtti.hash;
    }

    /// Implementations of the virtual functions in GroupFilter (CanCollide is pure virtual)
    pub const impl = struct {
        pub fn saveBinaryState(self: *const GroupFilter, stream: StreamOut) void {
            stream.write(self.getRTTIHash());
        }

        pub fn restoreBinaryState(self: *GroupFilter, stream: StreamIn) Allocator.Error!void {
            // RTTI hash is read in sRestoreFromBinaryState
            _ = self;
            _ = stream;
        }
    };
};

const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;

/// Restores a group filter from `bytes`
fn restoreFromBuffer(allocator: Allocator, bytes: []const u8) Allocator.Error!GroupFilter.GroupFilterResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamInWrapper.init(&reader);
    return GroupFilter.restoreFromBinaryState(allocator, in.streamIn());
}

test "GroupFilter: a user group filter, the base class implementations" {
    const allocator = std.testing.allocator;
    const expect = std.testing.expect;

    // A user filter that only lets even group IDs collide, it keeps the base class SaveBinaryState
    const EvenFilter = struct {
        /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(EvenFilter)
        pub const rtti = RTTI.declare(@This(), .{ .name = "EvenFilter", .kind = .virtual, .bases = &.{GroupFilter} });
        pub const overrides = .{.canCollide};

        base: GroupFilter,
        destructed: ?*u32 = null,

        fn create(a: Allocator, destructed: ?*u32) Allocator.Error!*@This() {
            const self = try a.create(@This());
            self.* = .{ .base = .init(@This(), a), .destructed = destructed };
            return self;
        }

        /// new EvenFilter (RTTI create)
        pub fn createDefault(a: Allocator) Allocator.Error!*@This() {
            return create(a, null);
        }

        pub fn destruct(self: *@This()) void {
            if (self.destructed) |d| d.* += 1;
        }

        pub fn canCollide(self: *const @This(), group1: *const CollisionGroup, group2: *const CollisionGroup) bool {
            _ = self;
            return group1.getGroupID() % 2 == 0 and group2.getGroupID() % 2 == 0;
        }
    };

    var destructed: u32 = 0;
    const filter = try EvenFilter.create(allocator, &destructed);
    var filter_ref = Ref(GroupFilter).init(&filter.base);
    try std.testing.expectEqual(@as(u32, 1), filter.base.getRefCount());

    var a = CollisionGroup.init(&filter.base, 2, 0);
    defer a.deinit();
    var b = CollisionGroup.init(null, 4, 0);
    defer b.deinit();
    var c = CollisionGroup.init(null, 3, 0);
    defer c.deinit();
    try std.testing.expectEqual(@as(u32, 2), filter.base.getRefCount());
    try expect(a.canCollide(&b));
    try expect(b.canCollide(&a)); // b has no filter, the filter of a is used
    try expect(!a.canCollide(&c));
    try expect(!c.canCollide(&a));
    try expect(b.canCollide(&c)); // No filters

    // Base class SaveBinaryState: only the RTTI hash
    var buffer: [16]u8 = undefined;
    var writer: std.Io.Writer = .fixed(&buffer);
    var out = StreamOutWrapper.init(&writer);
    filter.base.saveBinaryState(out.streamOut());
    try std.testing.expectEqualSlices(u8, std.mem.asBytes(&RTTI.hashName("EvenFilter")), writer.buffered());
    try std.testing.expectEqual(RTTI.hashName("EvenFilter"), filter.base.getRTTIHash());

    // The class is not in RegisterTypes.factory, so it cannot be restored
    var restored = try restoreFromBuffer(allocator, writer.buffered());
    defer restored.deinit();
    try std.testing.expectEqualStrings("Failed to create instance of type", restored.getError());

    // The last release destroys the filter through the vtable (destructor chain, leak checked)
    filter_ref.deinit();
    try std.testing.expectEqual(@as(u32, 0), destructed); // a still references the filter
    a.deinit();
    try std.testing.expectEqual(@as(u32, 1), destructed);

    // A filter on the stack
    var embedded: EvenFilter = .{ .base = .init(EvenFilter, allocator), .destructed = &destructed };
    embedded.base.setEmbedded();
    var embedded_group = CollisionGroup.init(&embedded.base, 0, 0);
    embedded_group.deinit();
    embedded.base.deinit();
    try std.testing.expectEqual(@as(u32, 2), destructed);
}

test "GroupFilter: CollisionGroup.canCollide passes the group with the filter first" {
    const expect = std.testing.expect;

    // A user filter that is not symmetric, it records the order of the groups it gets
    const OrderFilter = struct {
        pub const rtti = GroupFilter.rtti; // No JPH_RTTI of its own
        pub const overrides = .{.canCollide};

        base: GroupFilter,
        last_call: *[2]?*const CollisionGroup,

        pub fn canCollide(self: *const @This(), group1: *const CollisionGroup, group2: *const CollisionGroup) bool {
            self.last_call.* = .{ group1, group2 };
            return group1.getSubGroupID() < group2.getSubGroupID();
        }
    };

    var last_call: [2]?*const CollisionGroup = .{ null, null };
    var filter: OrderFilter = .{ .base = .init(OrderFilter, std.testing.allocator), .last_call = &last_call };
    filter.base.setEmbedded();
    defer filter.base.deinit();

    var with_filter = CollisionGroup.init(&filter.base, 0, 1);
    defer with_filter.deinit();
    var other_with_filter = CollisionGroup.init(&filter.base, 0, 3);
    defer other_with_filter.deinit();
    var no_filter = CollisionGroup.init(null, 0, 2);
    defer no_filter.deinit();

    // The filter of this group: CanCollide(*this, inOther)
    try expect(with_filter.canCollide(&no_filter)); // 1 < 2
    try expect(last_call[0].? == &with_filter and last_call[1].? == &no_filter);
    try expect(!other_with_filter.canCollide(&with_filter)); // 3 < 1
    try expect(last_call[0].? == &other_with_filter and last_call[1].? == &with_filter);
    try expect(with_filter.canCollide(&other_with_filter)); // 1 < 3
    try expect(last_call[0].? == &with_filter and last_call[1].? == &other_with_filter);

    // This group has no filter, the filter of the other group: CanCollide(inOther, *this)
    try expect(no_filter.canCollide(&with_filter)); // 1 < 2
    try expect(last_call[0].? == &with_filter and last_call[1].? == &no_filter);
    try expect(!no_filter.canCollide(&other_with_filter)); // 3 < 2
    try expect(last_call[0].? == &other_with_filter and last_call[1].? == &no_filter);
}

test "GroupFilter: restore errors" {
    const allocator = std.testing.allocator;

    // Empty stream
    var r1 = try restoreFromBuffer(allocator, &.{});
    defer r1.deinit();
    try std.testing.expectEqualStrings("Failed to read type hash", r1.getError());

    // Truncated hash
    var r2 = try restoreFromBuffer(allocator, &.{ 1, 2, 3 });
    defer r2.deinit();
    try std.testing.expectEqualStrings("Failed to read type hash", r2.getError());

    // Unknown type, the abstract base class
    var r3 = try restoreFromBuffer(allocator, &.{ 1, 2, 3, 4 });
    defer r3.deinit();
    try std.testing.expectEqualStrings("Failed to create instance of type", r3.getError());
    const abstract_hash = RTTI.hashName("GroupFilter");
    var r4 = try restoreFromBuffer(allocator, std.mem.asBytes(&abstract_hash));
    defer r4.deinit();
    try std.testing.expectEqualStrings("Failed to create instance of type", r4.getError());

    // Truncated data of a GroupFilterTable (the object is created, restoring hits EOF, nothing leaks)
    const hash = RTTI.hashName("GroupFilterTable");
    var r5 = try restoreFromBuffer(allocator, std.mem.asBytes(&hash));
    defer r5.deinit();
    try std.testing.expectEqualStrings("Failed to restore object", r5.getError());
}
