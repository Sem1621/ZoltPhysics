//! Port of: UnitTests/LoggingContactListener.h
//! Status: complete
//!
//! Contact listener that just logs the calls made to it for later validation. Register it with
//! `system.setContactListener(.init(&listener))`.
//! - The callbacks cannot fail a test (Jolt's `CHECK` inside a callback): a violated check is remembered and
//!   `checkError` reports it as error.TestUnexpectedResult after the update. The log is an ArrayList with an explicit
//!   allocator, a failed append is remembered too and reported as error.OutOfMemory (Jolt's push_back cannot fail).
//! - Callbacks are made from worker threads, the log is protected by a mutex like in Jolt. `mExistingContacts`
//!   (`UnorderedSet<SubShapeIDPair>`, only used for validation) is a std hash set: its order is never observed.
//! - The two `Find` / `Contains` overloads are `find` / `contains` and `findSubShape` / `containsSubShape`.

const std = @import("std");
const zolt = @import("zolt");

const Allocator = std.mem.Allocator;
const RVec3 = zolt.RVec3;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const SubShapeID = zolt.SubShapeID;
const SubShapeIDPair = zolt.SubShapeIDPair;
const ContactManifold = zolt.ContactManifold;
const ContactSettings = zolt.ContactSettings;
const ValidateResult = zolt.ValidateResult;
const CollideShapeResult = zolt.CollideShapeResult;
const Mutex = zolt.Mutex;

const io = std.testing.io;

/// Contact listener that just logs the calls made to it for later validation
pub const LoggingContactListener = struct {
    /// Contact callback type
    pub const Type = enum {
        validate,
        add,
        persist,
        remove,
    };

    /// Entry written when a contact callback happens
    pub const LogEntry = struct {
        type: Type,
        body1: BodyID,
        body2: BodyID,
        manifold: ContactManifold,
    };

    /// The key of mExistingContacts (the bytes of a SubShapeIDPair)
    const PairKey = [4]u32;

    allocator: Allocator,
    /// Callbacks are made from a thread, make sure we don't corrupt the log
    log_mutex: Mutex = .{},
    log: std.ArrayList(LogEntry) = .empty,
    /// For validation purposes: the contacts that are currently active
    existing_contacts: std.AutoHashMapUnmanaged(PairKey, void) = .empty,
    /// Zolt: a CHECK inside a callback failed
    check_failed: bool = false,
    /// Zolt: an append failed (out of memory)
    out_of_memory: bool = false,

    pub fn init(allocator: Allocator) LoggingContactListener {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *LoggingContactListener) void {
        self.log.deinit(self.allocator);
        self.existing_contacts.deinit(self.allocator);
    }

    fn key(pair: SubShapeIDPair) PairKey {
        return @bitCast(pair);
    }

    /// Jolt's CHECK in a callback (called with the log mutex locked)
    fn check(self: *LoggingContactListener, ok: bool) void {
        if (!ok) self.check_failed = true;
    }

    fn append(self: *LoggingContactListener, entry: LogEntry) void {
        self.log.append(self.allocator, entry) catch {
            self.out_of_memory = true;
        };
    }

    pub fn onContactValidate(self: *LoggingContactListener, body1: *const Body, body2: *const Body, base_offset: RVec3, collision_result: *const CollideShapeResult) ValidateResult {
        _ = .{ base_offset, collision_result };

        // Check ordering contract between body 1 and body 2
        const contract = @intFromEnum(body1.getMotionType()) >= @intFromEnum(body2.getMotionType()) or
            (body1.getMotionType() == body2.getMotionType() and body1.getID().lessThan(body2.getID()));

        self.log_mutex.lock(io);
        defer self.log_mutex.unlock(io);
        self.check(contract);
        self.append(.{ .type = .validate, .body1 = body1.getID(), .body2 = body2.getID(), .manifold = .init() });
        return .accept_contact;
    }

    pub fn onContactAdded(self: *LoggingContactListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        _ = settings;

        self.log_mutex.lock(io);
        defer self.log_mutex.unlock(io);

        // Check contract that body 1 < body 2
        self.check(body1.getID().lessThan(body2.getID()));

        const pair = SubShapeIDPair.init(body1.getID(), manifold.sub_shape_id1, body2.getID(), manifold.sub_shape_id2);
        const result = self.existing_contacts.getOrPut(self.allocator, key(pair)) catch {
            self.out_of_memory = true;
            return;
        };
        self.check(!result.found_existing); // Validate that contact does not exist yet
        self.append(.{ .type = .add, .body1 = body1.getID(), .body2 = body2.getID(), .manifold = manifold.* });
    }

    pub fn onContactPersisted(self: *LoggingContactListener, body1: *const Body, body2: *const Body, manifold: *const ContactManifold, settings: *ContactSettings) void {
        _ = settings;

        self.log_mutex.lock(io);
        defer self.log_mutex.unlock(io);

        // Check contract that body 1 < body 2
        self.check(body1.getID().lessThan(body2.getID()));

        const pair = SubShapeIDPair.init(body1.getID(), manifold.sub_shape_id1, body2.getID(), manifold.sub_shape_id2);
        self.check(self.existing_contacts.contains(key(pair))); // Validate that OnContactAdded was called
        self.append(.{ .type = .persist, .body1 = body1.getID(), .body2 = body2.getID(), .manifold = manifold.* });
    }

    pub fn onContactRemoved(self: *LoggingContactListener, sub_shape_pair: *const SubShapeIDPair) void {
        self.log_mutex.lock(io);
        defer self.log_mutex.unlock(io);

        // Check contract that body 1 < body 2
        self.check(sub_shape_pair.getBody1ID().lessThan(sub_shape_pair.getBody2ID()));

        self.check(self.existing_contacts.remove(key(sub_shape_pair.*))); // Validate that OnContactAdded was called
        var manifold: ContactManifold = .init();
        manifold.sub_shape_id1 = sub_shape_pair.getSubShapeID1();
        manifold.sub_shape_id2 = sub_shape_pair.getSubShapeID2();
        self.append(.{ .type = .remove, .body1 = sub_shape_pair.getBody1ID(), .body2 = sub_shape_pair.getBody2ID(), .manifold = manifold });
    }

    /// Zolt: report a CHECK that failed inside a callback (error.TestUnexpectedResult) or a failed append
    /// (error.OutOfMemory)
    pub fn checkError(self: *const LoggingContactListener) !void {
        if (self.out_of_memory) return error.OutOfMemory;
        if (self.check_failed) return error.TestUnexpectedResult;
    }

    pub fn clear(self: *LoggingContactListener) void {
        self.log.clearRetainingCapacity();
    }

    pub fn getEntryCount(self: *const LoggingContactListener) usize {
        return self.log.items.len;
    }

    pub fn getEntry(self: *const LoggingContactListener, idx: usize) *const LogEntry {
        return &self.log.items[idx];
    }

    /// Find first event with a particular type and involving two particular bodies
    pub fn find(self: *const LoggingContactListener, entry_type: Type, body1: BodyID, body2: BodyID) ?usize {
        for (self.log.items, 0..) |e, i| {
            if (e.type == entry_type and ((e.body1.eql(body1) and e.body2.eql(body2)) or (e.body1.eql(body2) and e.body2.eql(body1))))
                return i;
        }

        return null;
    }

    /// Check if event with a particular type and involving two particular bodies exists
    pub fn contains(self: *const LoggingContactListener, entry_type: Type, body1: BodyID, body2: BodyID) bool {
        return self.find(entry_type, body1, body2) != null;
    }

    /// Find first event with a particular type and involving two particular bodies and sub shape IDs
    pub fn findSubShape(self: *const LoggingContactListener, entry_type: Type, body1: BodyID, sub_shape_id1: SubShapeID, body2: BodyID, sub_shape_id2: SubShapeID) ?usize {
        for (self.log.items, 0..) |e, i| {
            if (e.type == entry_type and
                ((e.body1.eql(body1) and e.manifold.sub_shape_id1.eql(sub_shape_id1) and e.body2.eql(body2) and e.manifold.sub_shape_id2.eql(sub_shape_id2)) or
                    (e.body1.eql(body2) and e.manifold.sub_shape_id1.eql(sub_shape_id2) and e.body2.eql(body1) and e.manifold.sub_shape_id2.eql(sub_shape_id1))))
                return i;
        }

        return null;
    }

    /// Check if event with a particular type and involving two particular bodies and sub shape IDs exists
    pub fn containsSubShape(self: *const LoggingContactListener, entry_type: Type, body1: BodyID, sub_shape_id1: SubShapeID, body2: BodyID, sub_shape_id2: SubShapeID) bool {
        return self.findSubShape(entry_type, body1, sub_shape_id1, body2, sub_shape_id2) != null;
    }
};
