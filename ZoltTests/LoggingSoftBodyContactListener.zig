//! Port of: UnitTests/LoggingSoftBodyContactListener.h
//! Status: complete
//!
//! Soft body contact listener that just logs the calls made to it for later validation. Register it with
//! `system.setSoftBodyContactListener(.init(&listener))`.
//! - Callbacks may be made from multiple threads, the log is protected by a `zolt.Mutex` (with `std.testing.io`), like
//!   in Jolt.
//! - The log is an ArrayList with an explicit allocator. A callback cannot fail a test: a failed allocation is
//!   remembered and `checkError` reports it as error.OutOfMemory after the update (Jolt's push_back cannot fail).
//! - `OnSoftBodyContactAdded` collects the bodies in contact in Jolt's `UnorderedSet<BodyID>` (`zolt.UnorderedSet`), so
//!   the entries are logged in Jolt's order.

const std = @import("std");
const zolt = @import("zolt");

const Allocator = std.mem.Allocator;
const Body = zolt.Body;
const BodyID = zolt.BodyID;
const Mutex = zolt.Mutex;
const SoftBodyContactSettings = zolt.SoftBodyContactSettings;
const SoftBodyManifold = zolt.SoftBodyManifold;
const SoftBodyValidateResult = zolt.SoftBodyValidateResult;

const io = std.testing.io;

/// Soft body contact listener that just logs the calls made to it for later validation
pub const LoggingSoftBodyContactListener = struct {
    /// Contact callback type
    pub const Type = enum {
        validate,
        add,
    };

    /// Entry written when a contact callback happens
    pub const LogEntry = struct {
        type: Type,
        /// The soft body involved in the contact
        soft_body_id: BodyID,
        /// The rigid body involved in the contact
        other_body_id: BodyID,
    };

    allocator: Allocator,
    /// Callbacks may be made from multiple threads, protect the log
    log_mutex: Mutex = .{},
    log: std.ArrayList(LogEntry) = .empty,
    validate_value_to_return: SoftBodyValidateResult = .accept_contact,
    /// Zolt: an allocation in a callback failed (out of memory)
    out_of_memory: bool = false,

    pub fn init(allocator: Allocator) LoggingSoftBodyContactListener {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *LoggingSoftBodyContactListener) void {
        self.log.deinit(self.allocator);
    }

    /// Value to return from the OnSoftBodyContactValidate callback
    pub fn setValidateValueToReturn(self: *LoggingSoftBodyContactListener, value: SoftBodyValidateResult) void {
        self.validate_value_to_return = value;
    }

    /// Append to the log (called with the log mutex locked)
    fn append(self: *LoggingSoftBodyContactListener, entry: LogEntry) void {
        self.log.append(self.allocator, entry) catch {
            self.out_of_memory = true;
        };
    }

    pub fn onSoftBodyContactValidate(self: *LoggingSoftBodyContactListener, soft_body: *const Body, other_body: *const Body, settings: *SoftBodyContactSettings) SoftBodyValidateResult {
        _ = settings;

        self.log_mutex.lock(io);
        defer self.log_mutex.unlock(io);
        self.append(.{ .type = .validate, .soft_body_id = soft_body.getID(), .other_body_id = other_body.getID() });
        return self.validate_value_to_return;
    }

    pub fn onSoftBodyContactAdded(self: *LoggingSoftBodyContactListener, soft_body: *const Body, manifold: *const SoftBodyManifold) void {
        // Count the number of vertices in contact per body
        var bodies_in_contact: zolt.UnorderedSet(BodyID, .{}) = .empty;
        defer bodies_in_contact.deinit(self.allocator);
        for (manifold.getVertices()) |*v| {
            if (manifold.hasContact(v)) {
                _ = bodies_in_contact.insert(self.allocator, manifold.getContactBodyID(v)) catch {
                    self.log_mutex.lock(io);
                    defer self.log_mutex.unlock(io);
                    self.out_of_memory = true;
                    return;
                };
            }
        }

        self.log_mutex.lock(io);
        defer self.log_mutex.unlock(io);
        var it = bodies_in_contact.constIterator();
        while (it.next()) |entry|
            self.append(.{ .type = .add, .soft_body_id = soft_body.getID(), .other_body_id = entry.* });
    }

    pub fn clear(self: *LoggingSoftBodyContactListener) void {
        self.log.clearRetainingCapacity();
    }

    pub fn getEntryCount(self: *const LoggingSoftBodyContactListener) usize {
        return self.log.items.len;
    }

    pub fn getEntry(self: *const LoggingSoftBodyContactListener, idx: usize) *const LogEntry {
        return &self.log.items[idx];
    }

    /// Find the first event of a given type involving the specified soft body and other body
    pub fn find(self: *const LoggingSoftBodyContactListener, entry_type: Type, soft_body_id: BodyID, other_body_id: BodyID) i32 {
        for (self.log.items, 0..) |*e, i| {
            if (e.type == entry_type and e.soft_body_id.eql(soft_body_id) and e.other_body_id.eql(other_body_id))
                return @intCast(i);
        }
        return -1;
    }

    /// Check if an event of a given type involving the specified soft body and other body exists
    pub fn contains(self: *const LoggingSoftBodyContactListener, entry_type: Type, soft_body_id: BodyID, other_body_id: BodyID) bool {
        return self.find(entry_type, soft_body_id, other_body_id) >= 0;
    }

    /// Zolt: report an allocation that failed inside a callback (error.OutOfMemory)
    pub fn checkError(self: *const LoggingSoftBodyContactListener) !void {
        if (self.out_of_memory) return error.OutOfMemory;
    }
};

test "LoggingSoftBodyContactListener: log, find and clear" {
    var listener: LoggingSoftBodyContactListener = .init(std.testing.allocator);
    defer listener.deinit();
    const soft_body: Body = .{ .id = .init(1) };
    const other_body: Body = .{ .id = .init(2) };

    // Validate returns the value that was set and logs the call
    var settings: SoftBodyContactSettings = .{ .is_sensor = false };
    const sb_listener = zolt.SoftBodyContactListener.init(&listener);
    try std.testing.expectEqual(SoftBodyValidateResult.accept_contact, sb_listener.onSoftBodyContactValidate(&soft_body, &other_body, &settings));
    listener.setValidateValueToReturn(.reject_contact);
    try std.testing.expectEqual(SoftBodyValidateResult.reject_contact, sb_listener.onSoftBodyContactValidate(&soft_body, &other_body, &settings));
    try std.testing.expectEqual(@as(usize, 2), listener.getEntryCount());
    try std.testing.expectEqual(@as(i32, 0), listener.find(.validate, soft_body.getID(), other_body.getID()));
    try std.testing.expect(!listener.contains(.add, soft_body.getID(), other_body.getID()));
    try std.testing.expect(!listener.contains(.validate, other_body.getID(), soft_body.getID()));
    try std.testing.expect(listener.getEntry(1).soft_body_id.eql(soft_body.getID()));

    // A manifold without vertices adds nothing
    const empty_manifold: SoftBodyManifold = .{ .vertices = &.{}, .colliding_shapes = &.{}, .colliding_sensors = &.{} };
    sb_listener.onSoftBodyContactAdded(&soft_body, &empty_manifold);
    try std.testing.expectEqual(@as(usize, 2), listener.getEntryCount());
    try listener.checkError();

    listener.clear();
    try std.testing.expectEqual(@as(usize, 0), listener.getEntryCount());
    try std.testing.expectEqual(@as(i32, -1), listener.find(.validate, soft_body.getID(), other_body.getID()));
}
