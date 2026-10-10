//! Port of: Jolt/Physics/PhysicsLock.h
//! Status: complete
//!
//! SimulationArchitecture D3:
//! - Jolt passes the lock context and type only with JPH_ENABLE_ASSERTS (`JPH_IF_ENABLE_ASSERTS(, context, type)`).
//!   Zolt always passes them (`PhysicsLock.lock(mutex, io, context, lock_type)`); without asserts they are unused.
//!   `io` is passed per call, like the small sync primitives (`mutex.lock(io)`), the owners read `body_manager.io`.
//! - The lock order check is Jolt's `static thread_local` table, a `threadlocal var` typed `void` when asserts are off,
//!   so it provably cannot influence release results. `EPhysicsLockTypes` and `PhysicsLockContext` exist in every build.
//! - `UniqueLock(T)` / `SharedLock(T)` are Jolt's RAII classes: `var lock: UniqueLock(Mutex) = .init(&mutex, io,
//!   context, .bodies_list); defer lock.deinit();`. Their context and type members are `void` without asserts.
//! - `LockType` is any mutex with `lock(io)` / `unlock(io)` (and `lockShared(io)` / `unlockShared(io)` for the shared
//!   variants): Zolt's `Mutex` and `SharedMutex`.

const std = @import("std");
const Core = @import("../Core/Core.zig");
const BodyManager = @import("Body/BodyManager.zig").BodyManager;

/// This is the list of locks used by the physics engine, they need to be locked in a particular order (from top of the list to bottom of the list) in order to prevent deadlocks
pub const PhysicsLockTypes = enum(u32) {
    broad_phase_query = 1 << 0,
    per_body = 1 << 1,
    bodies_list = 1 << 2,
    broad_phase_update = 1 << 3,
    constraints_list = 1 << 4,
    active_bodies_list = 1 << 5,
};

/// A token that indicates the context of a lock (we use 1 per physics system and we use the body manager pointer because it's convenient)
pub const PhysicsLockContext = *const BodyManager;

/// Helpers to safely lock the different mutexes that are part of the physics system while preventing deadlock
/// Class that keeps track per thread which lock are taken and if the order of locking is correct
pub const PhysicsLock = struct {
    const LockData = struct {
        locked_mutexes: u32 = 0,
        context: ?PhysicsLockContext = null,
    };

    /// static thread_local LockData sLocks[4] (in sGetLockedMutexes): debug only per thread state, `void` without asserts
    threadlocal var locks: if (Core.enable_asserts) [4]LockData else void = if (Core.enable_asserts) @splat(.{}) else {};

    /// Call before taking the lock (only does something when asserts are enabled)
    pub fn checkLock(context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        if (Core.enable_asserts) {
            const mutexes = getLockedMutexes(context);
            std.debug.assert(@intFromEnum(lock_type) > mutexes.*); // A lock of same or higher priority was already taken, this can create a deadlock!
            mutexes.* = mutexes.* | @intFromEnum(lock_type);
        }
    }

    /// Call after releasing the lock (only does something when asserts are enabled)
    pub fn checkUnlock(context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        if (Core.enable_asserts) {
            const mutexes = getLockedMutexes(context);
            std.debug.assert((mutexes.* & @intFromEnum(lock_type)) != 0); // Mutex was not locked!
            mutexes.* = mutexes.* & ~@intFromEnum(lock_type);
        }
    }

    pub fn lock(mutex: anytype, io: std.Io, context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        checkLock(context, lock_type);
        mutex.lock(io);
    }

    pub fn unlock(mutex: anytype, io: std.Io, context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        checkUnlock(context, lock_type);
        mutex.unlock(io);
    }

    pub fn lockShared(mutex: anytype, io: std.Io, context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        checkLock(context, lock_type);
        mutex.lockShared(io);
    }

    pub fn unlockShared(mutex: anytype, io: std.Io, context: PhysicsLockContext, lock_type: PhysicsLockTypes) void {
        checkUnlock(context, lock_type);
        mutex.unlockShared(io);
    }

    /// Helper function to find the locked mutexes for a particular context (only exists with asserts)
    fn getLockedMutexes(context: PhysicsLockContext) *u32 {
        if (!Core.enable_asserts) @compileError("PhysicsLock.getLockedMutexes only exists with asserts");

        // If we find a matching context we can use it
        for (&locks) |*l| {
            if (l.context == context)
                return &l.locked_mutexes;
        }

        // Otherwise we look for an entry that is not in use
        for (&locks) |*l| {
            if (l.locked_mutexes == 0) {
                l.context = context;
                return &l.locked_mutexes;
            }
        }

        @panic("Too many physics systems locked at the same time!"); // Jolt: JPH_ASSERT(false), asserts are on here
    }

    /// The bit mask of locks that this thread holds for `context` (zero without asserts, used by the tests)
    pub fn getLockedMutexesForTest(context: PhysicsLockContext) u32 {
        if (Core.enable_asserts) {
            for (&locks) |*l| {
                if (l.context == context)
                    return l.locked_mutexes;
            }
        }
        return 0;
    }
};

/// Helper class that is similar to std::unique_lock
pub fn UniqueLock(comptime LockType: type) type {
    return struct {
        const Self = @This();

        lock: *LockType,
        io: std.Io,
        context: if (Core.enable_asserts) PhysicsLockContext else void,
        lock_type: if (Core.enable_asserts) PhysicsLockTypes else void,

        pub fn init(mutex: *LockType, io: std.Io, context: PhysicsLockContext, lock_type: PhysicsLockTypes) Self {
            const self: Self = .{
                .lock = mutex,
                .io = io,
                .context = if (Core.enable_asserts) context else {},
                .lock_type = if (Core.enable_asserts) lock_type else {},
            };
            PhysicsLock.lock(self.lock, io, context, lock_type);
            return self;
        }

        pub fn deinit(self: *Self) void {
            if (Core.enable_asserts)
                PhysicsLock.unlock(self.lock, self.io, self.context, self.lock_type)
            else
                self.lock.unlock(self.io);
        }
    };
}

/// Helper class that is similar to std::shared_lock
pub fn SharedLock(comptime LockType: type) type {
    return struct {
        const Self = @This();

        lock: *LockType,
        io: std.Io,
        context: if (Core.enable_asserts) PhysicsLockContext else void,
        lock_type: if (Core.enable_asserts) PhysicsLockTypes else void,

        pub fn init(mutex: *LockType, io: std.Io, context: PhysicsLockContext, lock_type: PhysicsLockTypes) Self {
            const self: Self = .{
                .lock = mutex,
                .io = io,
                .context = if (Core.enable_asserts) context else {},
                .lock_type = if (Core.enable_asserts) lock_type else {},
            };
            PhysicsLock.lockShared(self.lock, io, context, lock_type);
            return self;
        }

        pub fn deinit(self: *Self) void {
            if (Core.enable_asserts)
                PhysicsLock.unlockShared(self.lock, self.io, self.context, self.lock_type)
            else
                self.lock.unlockShared(self.io);
        }
    };
}

test "PhysicsLock: lock order bookkeeping, UniqueLock and SharedLock" {
    const Mutex = @import("../Core/Mutex.zig").Mutex;
    const SharedMutex = @import("../Core/Mutex.zig").SharedMutex;
    const io = std.testing.io;
    const expectEqual = std.testing.expectEqual;

    // Two distinct contexts (the lock functions never dereference them)
    const managers: [2]BodyManager = .{ .empty, .empty };
    const c1: PhysicsLockContext = &managers[0];
    const c2: PhysicsLockContext = &managers[1];

    var query_mutex: SharedMutex = .{};
    var body_mutex: SharedMutex = .{};
    var list_mutex: Mutex = .{};
    var active_mutex: Mutex = .{};

    // Locks taken in the order of PhysicsLockTypes
    PhysicsLock.lockShared(&query_mutex, io, c1, .broad_phase_query);
    PhysicsLock.lock(&body_mutex, io, c1, .per_body);
    {
        var lock: UniqueLock(Mutex) = .init(&list_mutex, io, c1, .bodies_list);
        defer lock.deinit();
        try std.testing.expect(list_mutex.isLocked());
        if (Core.enable_asserts) try expectEqual(@as(u32, 0b111), PhysicsLock.getLockedMutexesForTest(c1));

        // Another context has its own bookkeeping
        var lock2: UniqueLock(Mutex) = .init(&active_mutex, io, c2, .active_bodies_list);
        defer lock2.deinit();
        if (Core.enable_asserts) try expectEqual(@as(u32, 0b100000), PhysicsLock.getLockedMutexesForTest(c2));
    }
    try std.testing.expect(!list_mutex.isLocked());
    PhysicsLock.unlock(&body_mutex, io, c1, .per_body);
    {
        var shared: SharedLock(SharedMutex) = .init(&body_mutex, io, c1, .per_body);
        defer shared.deinit();
        try std.testing.expect(body_mutex.tryLockShared(io)); // Other readers may enter
        body_mutex.unlockShared(io);
    }
    PhysicsLock.unlockShared(&query_mutex, io, c1, .broad_phase_query);
    try expectEqual(@as(u32, 0), PhysicsLock.getLockedMutexesForTest(c1));
    try expectEqual(@as(u32, 0), PhysicsLock.getLockedMutexesForTest(c2));

    // The bookkeeping is per thread
    const Other = struct {
        fn run(context: PhysicsLockContext, mutex: *Mutex) void {
            var lock: UniqueLock(Mutex) = .init(mutex, io, context, .per_body);
            lock.deinit();
        }
    };
    PhysicsLock.lock(&list_mutex, io, c1, .active_bodies_list);
    const thread = try std.Thread.spawn(.{}, Other.run, .{ c1, &active_mutex }); // .per_body would be out of order on this thread
    thread.join();
    PhysicsLock.unlock(&list_mutex, io, c1, .active_bodies_list);
}
