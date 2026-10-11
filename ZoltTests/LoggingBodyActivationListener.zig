//! Port of: UnitTests/LoggingBodyActivationListener.h
//! Status: complete
//!
//! Activation listener that just logs the activations/deactivations. Register it with
//! `system.setBodyActivationListener(.init(&listener))`. The log is an ArrayList with an explicit allocator: the
//! callbacks cannot return an error, so a failed append is remembered and reported by `checkError` (Jolt's push_back
//! cannot fail). Callbacks are made from worker threads, the log is protected by a mutex like in Jolt.

const std = @import("std");
const zolt = @import("zolt");

const Allocator = std.mem.Allocator;
const BodyID = zolt.BodyID;
const Mutex = zolt.Mutex;

const io = std.testing.io;

/// Activation listener that just logs the activations/deactivations
pub const LoggingBodyActivationListener = struct {
    /// Activation callback type
    pub const Type = enum {
        activated,
        deactivated,
    };

    /// Entry written when an activation callback happens
    pub const LogEntry = struct {
        type: Type,
        body_id: BodyID,
    };

    allocator: Allocator,
    /// Callbacks are made from a thread, make sure we don't corrupt the log
    log_mutex: Mutex = .{},
    log: std.ArrayList(LogEntry) = .empty,
    /// Zolt: an append failed (out of memory)
    out_of_memory: bool = false,

    pub fn init(allocator: Allocator) LoggingBodyActivationListener {
        return .{ .allocator = allocator };
    }

    pub fn deinit(self: *LoggingBodyActivationListener) void {
        self.log.deinit(self.allocator);
    }

    fn append(self: *LoggingBodyActivationListener, entry: LogEntry) void {
        self.log_mutex.lock(io);
        defer self.log_mutex.unlock(io);
        self.log.append(self.allocator, entry) catch {
            self.out_of_memory = true;
        };
    }

    pub fn onBodyActivated(self: *LoggingBodyActivationListener, body_id: BodyID, body_user_data: u64) void {
        _ = body_user_data;
        self.append(.{ .type = .activated, .body_id = body_id });
    }

    pub fn onBodyDeactivated(self: *LoggingBodyActivationListener, body_id: BodyID, body_user_data: u64) void {
        _ = body_user_data;
        self.append(.{ .type = .deactivated, .body_id = body_id });
    }

    /// Zolt: error.OutOfMemory when an entry could not be logged
    pub fn checkError(self: *const LoggingBodyActivationListener) Allocator.Error!void {
        if (self.out_of_memory) return error.OutOfMemory;
    }

    pub fn clear(self: *LoggingBodyActivationListener) void {
        self.log.clearRetainingCapacity();
    }

    pub fn getEntryCount(self: *const LoggingBodyActivationListener) usize {
        return self.log.items.len;
    }

    /// Check if we have logged an event with a particular type and involving a particular body
    pub fn contains(self: *const LoggingBodyActivationListener, entry_type: Type, body_id: BodyID) bool {
        for (self.log.items) |e| {
            if (e.type == entry_type and e.body_id.eql(body_id))
                return true;
        }

        return false;
    }
};
