//! Port of: Jolt/Physics/Body/BodyActivationListener.h
//! Status: complete
//!
//! An interface (porting guide pattern B, CollisionArchitecture D7, SimulationArchitecture D7): a type erased
//! `ptr: *anyopaque` (the callbacks are non-const in Jolt) and a vtable. Both callbacks are pure virtual in Jolt, so the
//! entries are non-null and an implementation must declare both:
//! ```zig
//! const MyListener = struct {
//!     pub fn onBodyActivated(self: *MyListener, body_id: BodyID, body_user_data: u64) void { ... }
//!     pub fn onBodyDeactivated(self: *MyListener, body_id: BodyID, body_user_data: u64) void { ... }
//! };
//! body_manager.setBodyActivationListener(.init(&my_listener));
//! ```
//! The listener is stored by value in the BodyManager and called with the active bodies mutex locked. The callbacks run
//! concurrently on the job system's worker threads during PhysicsSystem.update. The listener does not own the
//! implementation (the C++ virtual destructor has no counterpart).

const BodyID = @import("BodyID.zig").BodyID;

/// A listener class that receives events when a body activates or deactivates.
/// It can be registered with the BodyManager (or PhysicsSystem).
pub const BodyActivationListener = struct {
    ptr: *anyopaque,
    vtable: *const VTable,

    pub const VTable = struct {
        /// Called whenever a body activates, note this can be called from any thread so make sure your code is thread safe.
        /// At the time of the callback the body body_id will be locked and no bodies can be written/activated/deactivated from the callback.
        onBodyActivated: *const fn (ptr: *anyopaque, body_id: BodyID, body_user_data: u64) void,

        /// Called whenever a body deactivates, note this can be called from any thread so make sure your code is thread safe.
        /// At the time of the callback the body body_id will be locked and no bodies can be written/activated/deactivated from the callback.
        onBodyDeactivated: *const fn (ptr: *anyopaque, body_id: BodyID, body_user_data: u64) void,
    };

    /// Wrap any `*T` that declares `onBodyActivated` and `onBodyDeactivated`
    pub fn init(impl: anytype) BodyActivationListener {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            // Thunks need their own names: Zig forbids shadowing the functions of the outer struct
            fn onBodyActivatedThunk(ptr: *anyopaque, body_id: BodyID, body_user_data: u64) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.onBodyActivated(body_id, body_user_data);
            }

            fn onBodyDeactivatedThunk(ptr: *anyopaque, body_id: BodyID, body_user_data: u64) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.onBodyDeactivated(body_id, body_user_data);
            }

            const vtable: VTable = .{
                .onBodyActivated = onBodyActivatedThunk,
                .onBodyDeactivated = onBodyDeactivatedThunk,
            };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    /// Called whenever a body activates (see VTable)
    pub fn onBodyActivated(self: BodyActivationListener, body_id: BodyID, body_user_data: u64) void {
        self.vtable.onBodyActivated(self.ptr, body_id, body_user_data);
    }

    /// Called whenever a body deactivates (see VTable)
    pub fn onBodyDeactivated(self: BodyActivationListener, body_id: BodyID, body_user_data: u64) void {
        self.vtable.onBodyDeactivated(self.ptr, body_id, body_user_data);
    }
};

test "BodyActivationListener forwards both callbacks" {
    const std = @import("std");
    const Counter = struct {
        activated: u32 = 0,
        deactivated: u32 = 0,
        last_user_data: u64 = 0,

        pub fn onBodyActivated(self: *@This(), body_id: BodyID, body_user_data: u64) void {
            _ = body_id;
            self.activated += 1;
            self.last_user_data = body_user_data;
        }

        pub fn onBodyDeactivated(self: *@This(), body_id: BodyID, body_user_data: u64) void {
            _ = body_id;
            self.deactivated += 1;
            self.last_user_data = body_user_data;
        }
    };
    var counter: Counter = .{};
    const listener = BodyActivationListener.init(&counter);
    listener.onBodyActivated(.init(1), 5);
    listener.onBodyActivated(.init(2), 6);
    listener.onBodyDeactivated(.init(1), 7);
    try std.testing.expectEqual(@as(u32, 2), counter.activated);
    try std.testing.expectEqual(@as(u32, 1), counter.deactivated);
    try std.testing.expectEqual(@as(u64, 7), counter.last_user_data);
}
