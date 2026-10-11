//! Port of: Jolt/Physics/Body/BodyLock.h
//! Status: complete
//!
//! SimulationArchitecture D3: `template <bool Write, class BodyType> class BodyLockBase` is `BodyLockBase(comptime write:
//! bool)`; the body type follows from `write` (`*const Body` for read locks, `*Body` for write locks). The RAII classes
//! are init / deinit:
//! ```zig
//! var lock: BodyLockWrite = .init(body_lock_interface, body_id);
//! defer lock.deinit();
//! if (lock.succeeded()) lock.getBody().setUserData(42);
//! ```
//! A lock must not be copied while it holds the mutex (Jolt: NonCopyable).

const std = @import("std");
const SharedMutex = @import("../../Core/Mutex.zig").SharedMutex;
const BodyID = @import("BodyID.zig").BodyID;
const Body = @import("Body.zig").Body;
const BodyLockInterface = @import("BodyLockInterface.zig").BodyLockInterface;

/// Base class for locking bodies for the duration of the scope of this class (do not use directly)
pub fn BodyLockBase(comptime write: bool) type {
    return struct {
        const Self = @This();

        /// BodyType *: `*Body` for write locks, `*const Body` for read locks
        pub const BodyPtr = if (write) *Body else *const Body;

        body_lock_interface: *const BodyLockInterface,
        body_lock_mutex: ?*SharedMutex,
        body: ?BodyPtr,

        /// Constructor will lock the body
        pub fn init(body_lock_interface: *const BodyLockInterface, body_id: BodyID) Self {
            if (body_id.eql(BodyID.invalid)) {
                // Invalid body id
                return .{ .body_lock_interface = body_lock_interface, .body_lock_mutex = null, .body = null };
            } else {
                // Get mutex
                const mutex = if (write) body_lock_interface.lockWrite(body_id) else body_lock_interface.lockRead(body_id);

                // Get a reference to the body or nullptr when it is no longer valid
                return .{ .body_lock_interface = body_lock_interface, .body_lock_mutex = mutex, .body = body_lock_interface.tryGetBody(body_id) };
            }
        }

        /// Explicitly release the lock (normally this is done in the destructor)
        pub fn releaseLock(self: *Self) void {
            if (self.body_lock_mutex != null) {
                if (write)
                    self.body_lock_interface.unlockWrite(self.body_lock_mutex)
                else
                    self.body_lock_interface.unlockRead(self.body_lock_mutex);

                self.body_lock_mutex = null;
                self.body = null;
            }
        }

        /// Destructor will unlock the body
        pub fn deinit(self: *Self) void {
            self.releaseLock();
        }

        /// Test if the lock was successful (if the body ID was valid)
        pub fn succeeded(self: *const Self) bool {
            return self.body != null;
        }

        /// Test if the lock was successful (if the body ID was valid) and the body is still in the broad phase
        pub fn succeededAndIsInBroadPhase(self: *const Self) bool {
            return self.body != null and self.body.?.isInBroadPhase();
        }

        /// Access the body
        pub fn getBody(self: *const Self) BodyPtr {
            std.debug.assert(self.body != null); // Should check Succeeded() first
            return self.body.?;
        }
    };
}

/// A body lock takes a body ID and locks the underlying body so that other threads cannot access its members
///
/// The common usage pattern is:
///
///        const lock_interface = physics_system.getBodyLockInterface(); // Or non-locking interface if the lock is already taken
///        const body_id = ...; // Obtain ID to body
///
///        // Scoped lock
///        {
///            var lock: BodyLockRead = .init(lock_interface, body_id);
///            defer lock.deinit();
///            if (lock.succeeded()) // body_id may no longer be valid
///            {
///                const body = lock.getBody();
///
///                // Do something with body
///                ...
///            }
///        }
pub const BodyLockRead = BodyLockBase(false);

/// Specialization that locks a body for writing to. @see BodyLockRead for usage patterns.
pub const BodyLockWrite = BodyLockBase(true);
