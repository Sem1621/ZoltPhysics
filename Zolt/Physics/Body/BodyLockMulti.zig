//! Port of: Jolt/Physics/Body/BodyLockMulti.h
//! Status: complete
//!
//! SimulationArchitecture D3: `template <bool Write, class BodyType> class BodyLockMultiBase` is
//! `BodyLockMultiBase(comptime write: bool)`; `getBody` returns `?*const Body` for read locks and `?*Body` for write
//! locks. The pointer + count of body IDs is a slice that must outlive the lock (Jolt keeps the pointer). The RAII
//! classes are init / deinit (`var lock: BodyLockMultiWrite = .init(interface, ids); defer lock.deinit();`).

const std = @import("std");
const BodyID = @import("BodyID.zig").BodyID;
const Body = @import("Body.zig").Body;
const BodyLockInterface = @import("BodyLockInterface.zig").BodyLockInterface;

/// Base class for locking multiple bodies for the duration of the scope of this class (do not use directly)
pub fn BodyLockMultiBase(comptime write: bool) type {
    return struct {
        const Self = @This();

        /// Redefine MutexMask
        pub const MutexMask = BodyLockInterface.MutexMask;

        /// BodyType *: `*Body` for write locks, `*const Body` for read locks
        pub const BodyPtr = if (write) *Body else *const Body;

        body_lock_interface: *const BodyLockInterface,
        mutex_mask: MutexMask,
        body_ids: []const BodyID,

        /// Constructor will lock the bodies
        pub fn init(body_lock_interface: *const BodyLockInterface, body_ids: []const BodyID) Self {
            const self: Self = .{ .body_lock_interface = body_lock_interface, .mutex_mask = body_lock_interface.getMutexMask(body_ids), .body_ids = body_ids };
            if (self.mutex_mask != 0) {
                // Get mutex
                if (write)
                    body_lock_interface.lockWriteMask(self.mutex_mask)
                else
                    body_lock_interface.lockReadMask(self.mutex_mask);
            }
            return self;
        }

        /// Explicitly release the locks on all bodies (normally this is done in the destructor)
        pub fn releaseLocks(self: *Self) void {
            if (self.mutex_mask != 0) {
                if (write)
                    self.body_lock_interface.unlockWriteMask(self.mutex_mask)
                else
                    self.body_lock_interface.unlockReadMask(self.mutex_mask);

                self.mutex_mask = 0;
                self.body_ids = &.{};
            }
        }

        /// Destructor will unlock the bodies
        pub fn deinit(self: *Self) void {
            self.releaseLocks();
        }

        /// Returns the number of bodies that were locked
        pub fn getNumBodies(self: *const Self) u32 {
            return @intCast(self.body_ids.len);
        }

        /// Access the body (returns null if body was not properly locked)
        pub fn getBody(self: *const Self, body_index: u32) ?BodyPtr {
            // Range check
            std.debug.assert(body_index < self.body_ids.len);

            // Get body ID
            const body_id = self.body_ids[body_index];
            if (body_id.isInvalid())
                return null;

            // Get a reference to the body or nullptr when it is no longer valid
            return self.body_lock_interface.tryGetBody(body_id);
        }
    };
}

/// A multi body lock takes a number of body IDs and locks the underlying bodies so that other threads cannot access its members
///
/// The common usage pattern is:
///
///        const lock_interface = physics_system.getBodyLockInterface(); // Or non-locking interface if the lock is already taken
///        const body_ids: []const BodyID = ...; // Obtain IDs to bodies
///
///        // Scoped lock
///        {
///            var lock: BodyLockMultiRead = .init(lock_interface, body_ids);
///            defer lock.deinit();
///            for (0..body_ids.len) |i|
///            {
///                if (lock.getBody(@intCast(i))) |body|
///                {
///                    // Do something with body
///                    ...
///                }
///            }
///        }
pub const BodyLockMultiRead = BodyLockMultiBase(false);

/// Specialization that locks multiple bodies for writing to. @see BodyLockMultiRead for usage patterns.
pub const BodyLockMultiWrite = BodyLockMultiBase(true);
