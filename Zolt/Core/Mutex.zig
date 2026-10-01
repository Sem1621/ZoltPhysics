//! Port of: Jolt/Core/Mutex.h
//! Status: complete
//!
//! `Mutex` wraps `std.Io.Mutex` (std::mutex) and `SharedMutex` wraps `SharedMutexBase` (std::shared_mutex), a copy of
//! `std.Io.RwLock` with a fixed `tryLock` (see SharedMutexBase). Like all small synchronization primitives in Zolt
//! they take the `io: std.Io` per call instead of storing it, because they are embedded in many other structs (see
//! "Threading" in the porting guide). Physics code is not cancelable, so blocking calls use the `*Uncancelable`
//! variants of std.Io and keep Jolt's signatures (no error union).
//!
//! Zig has no RAII, so the lock helpers that Jolt imports from the STL become explicit calls plus `defer`:
//! - `lock_guard lock(mutex)` / `unique_lock lock(mutex)` -> `mutex.lock(io); defer mutex.unlock(io);`
//! - `shared_lock lock(shared_mutex)` -> `shared_mutex.lockShared(io); defer shared_mutex.unlockShared(io);`
//! - `std::thread` (`using std::thread`) -> `std.Thread`
//!
//! Jolt only uses its wrappers (which assert that lock and unlock happen on the same thread) when asserts or the
//! profiler are enabled, otherwise `Mutex` is `std::mutex`. Zolt always uses the wrapper; the lock tracking only
//! exists when `Core.enable_asserts` (JPH_ENABLE_ASSERTS) and JPH_PROFILE is dropped (see the porting guide).
//! The JPH_PLATFORM_BLUE implementations are not ported, std.Io provides the platform layer.
//!
//! API summary (all types are default initialized with `.{}`, need no deinit and must not be copied once used):
//! - `Mutex`: `lock(io)`, `tryLock() bool`, `unlock(io)`, `isLocked() bool` (for asserts).
//! - `SharedMutex`: exclusive `lock(io)`, `tryLock(io) bool`, `unlock(io)`; shared `lockShared(io)`,
//!   `tryLockShared(io) bool`, `unlockShared(io)`; `isLocked() bool` (exclusive lock only, for asserts).
//! - `MutexBase` (= `std.Io.Mutex`) / `SharedMutexBase`: the underlying locks without the lock tracking
//!   (std::mutex / std::shared_mutex); their blocking functions are `lockUncancelable` / `lockSharedUncancelable`.
//!   Use them only where Jolt uses MutexBase / SharedMutexBase directly, otherwise use Mutex / SharedMutex.

const std = @import("std");
const Core = @import("Core.zig");

/// ID of the thread that holds a lock, only tracked when asserts are enabled (JPH_IF_ENABLE_ASSERTS(thread::id mLockedThreadID))
const LockedThreadId = if (Core.enable_asserts) std.atomic.Value(std.Thread.Id) else void;

/// Value of LockedThreadId when no thread holds the lock (thread::id()). Operating systems never hand out 0 as a thread ID.
const no_thread: std.Thread.Id = 0;

const locked_thread_id_init: LockedThreadId = if (Core.enable_asserts) .init(no_thread) else {};

/// The mutex that Mutex wraps (MutexBase, std::mutex in Jolt)
pub const MutexBase = std.Io.Mutex;

/// Reader-writer lock that allows one writer or many readers (SharedMutexBase, std::shared_mutex in Jolt).
///
/// This is the algorithm of `std.Io.RwLock` from Zig 0.16, except for `tryLock`: std.Io.RwLock.tryLock checks that
/// there are no readers and then sets its writing flag in a separate step, so a reader that takes the fast path in
/// between ends up holding the lock at the same time as the writer. Here a compare and swap does both at once.
pub const SharedMutexBase = struct {
    /// Bit 0: a writer holds (or is acquiring) the lock, then the number of writers waiting for `mutex`, then the number of readers
    state: std.atomic.Value(usize) = .init(0),

    /// Held by the writer, also taken by readers when they can't take the fast path
    mutex: std.Io.Mutex = .init,

    /// Signals a writer that waits for the last reader to leave
    semaphore: std.Io.Semaphore = .{},

    const Count = @Int(.unsigned, @divFloor(@bitSizeOf(usize) - 1, 2));
    const is_writing: usize = 1;
    const writer: usize = 1 << 1;
    const reader: usize = 1 << (1 + @bitSizeOf(Count));
    const writer_mask: usize = std.math.maxInt(Count) << @ctz(writer);
    const reader_mask: usize = std.math.maxInt(Count) << @ctz(reader);

    /// Try to take the lock for writing without blocking
    pub fn tryLock(self: *SharedMutexBase, io: std.Io) bool {
        if (self.mutex.tryLock()) {
            // Only set the writing flag if there are no readers, atomically with the check
            var state = self.state.load(.seq_cst);
            while (state & reader_mask == 0)
                state = self.state.cmpxchgWeak(state, state | is_writing, .seq_cst, .seq_cst) orelse return true;

            self.mutex.unlock(io);
        }
        return false;
    }

    /// Take the lock for writing, blocks until all readers and writers are gone
    pub fn lockUncancelable(self: *SharedMutexBase, io: std.Io) void {
        _ = self.state.fetchAdd(writer, .seq_cst);
        self.mutex.lockUncancelable(io);

        const state = self.state.fetchAdd(is_writing -% writer, .seq_cst);
        if (state & reader_mask != 0)
            self.semaphore.waitUncancelable(io);
    }

    /// Release the lock for writing
    pub fn unlock(self: *SharedMutexBase, io: std.Io) void {
        _ = self.state.fetchAnd(~is_writing, .seq_cst);
        self.mutex.unlock(io);
    }

    /// Try to take the lock for reading without blocking
    pub fn tryLockShared(self: *SharedMutexBase, io: std.Io) bool {
        const state = self.state.load(.seq_cst);
        if (state & (is_writing | writer_mask) == 0) {
            _ = self.state.cmpxchgStrong(state, state + reader, .seq_cst, .seq_cst) orelse return true;
        }

        if (self.mutex.tryLock()) {
            _ = self.state.fetchAdd(reader, .seq_cst);
            self.mutex.unlock(io);
            return true;
        }

        return false;
    }

    /// Take the lock for reading, blocks while a writer holds or waits for the lock
    pub fn lockSharedUncancelable(self: *SharedMutexBase, io: std.Io) void {
        var state = self.state.load(.seq_cst);
        while (state & (is_writing | writer_mask) == 0)
            state = self.state.cmpxchgWeak(state, state + reader, .seq_cst, .seq_cst) orelse return;

        self.mutex.lockUncancelable(io);
        _ = self.state.fetchAdd(reader, .seq_cst);
        self.mutex.unlock(io);
    }

    /// Release the lock for reading
    pub fn unlockShared(self: *SharedMutexBase, io: std.Io) void {
        const state = self.state.fetchSub(reader, .seq_cst);

        if ((state & reader_mask == reader) and (state & is_writing != 0))
            self.semaphore.post(io);
    }

    /// True when a writer holds (or is acquiring) the lock
    pub fn isWriting(self: *const SharedMutexBase) bool {
        return self.state.load(.monotonic) & is_writing != 0;
    }
};

/// Very simple wrapper around MutexBase (std.Io.Mutex) which tracks lock contention in the profiler
/// and asserts that locks/unlocks take place on the same thread.
///
/// Usage: `var mutex: Mutex = .{};` then `mutex.lock(io); defer mutex.unlock(io);`. No deinit is needed.
pub const Mutex = struct {
    /// The underlying mutex
    base: MutexBase = .init,

    /// Thread that currently holds the lock, only when Core.enable_asserts (mLockedThreadID)
    locked_thread_id: LockedThreadId = locked_thread_id_init,

    /// Try to lock the mutex without blocking (try_lock), returns true when the lock was acquired.
    /// It is not allowed to lock a mutex that the calling thread already holds.
    pub fn tryLock(self: *Mutex) bool {
        if (Core.enable_asserts) std.debug.assert(self.locked_thread_id.load(.monotonic) != std.Thread.getCurrentId());
        if (self.base.tryLock()) {
            if (Core.enable_asserts) self.locked_thread_id.store(std.Thread.getCurrentId(), .monotonic);
            return true;
        }
        return false;
    }

    /// Lock the mutex, blocks until it is available (lock). Not cancelable.
    pub fn lock(self: *Mutex, io: std.Io) void {
        if (!self.tryLock()) {
            // JPH_PROFILE("Lock", 0xff00ffff) is not ported
            self.base.lockUncancelable(io);
            if (Core.enable_asserts) self.locked_thread_id.store(std.Thread.getCurrentId(), .monotonic);
        }
    }

    /// Unlock the mutex (unlock), must be called by the thread that locked it
    pub fn unlock(self: *Mutex, io: std.Io) void {
        if (Core.enable_asserts) {
            std.debug.assert(self.locked_thread_id.load(.monotonic) == std.Thread.getCurrentId());
            self.locked_thread_id.store(no_thread, .monotonic);
        }
        self.base.unlock(io);
    }

    /// Returns true when the mutex is locked by any thread (is_locked).
    /// Jolt only has this function when JPH_ENABLE_ASSERTS is defined (it is meant for asserts). Zolt answers from the
    /// tracked thread ID when Core.enable_asserts and from the state of the underlying mutex otherwise, so that
    /// `std.debug.assert(mutex.isLocked())` is correct in every build mode.
    pub fn isLocked(self: *const Mutex) bool {
        if (Core.enable_asserts)
            return self.locked_thread_id.load(.monotonic) != no_thread;
        return self.base.state.load(.monotonic) != .unlocked;
    }
};

/// Very simple wrapper around SharedMutexBase which tracks lock contention in the profiler
/// and asserts that locks/unlocks take place on the same thread.
///
/// Exclusive (write) access: `lock(io)` / `tryLock(io)` / `unlock(io)`.
/// Shared (read) access: `lockShared(io)` / `tryLockShared(io)` / `unlockShared(io)`.
/// Unlike `Mutex.tryLock`, `tryLock` takes `io` because a failed attempt has to release the internal mutex.
///
/// Usage: `var mutex: SharedMutex = .{};` then `mutex.lockShared(io); defer mutex.unlockShared(io);`. No deinit is needed.
pub const SharedMutex = struct {
    /// The underlying lock (SharedMutexBase)
    base: SharedMutexBase = .{},

    /// Thread that currently holds the exclusive lock, only when Core.enable_asserts (mLockedThreadID)
    locked_thread_id: LockedThreadId = locked_thread_id_init,

    /// Try to lock the mutex for exclusive access without blocking (try_lock), returns true when the lock was acquired
    pub fn tryLock(self: *SharedMutex, io: std.Io) bool {
        if (Core.enable_asserts) std.debug.assert(self.locked_thread_id.load(.monotonic) != std.Thread.getCurrentId());
        if (self.base.tryLock(io)) {
            if (Core.enable_asserts) self.locked_thread_id.store(std.Thread.getCurrentId(), .monotonic);
            return true;
        }
        return false;
    }

    /// Lock the mutex for exclusive access, blocks until it is available (lock). Not cancelable.
    pub fn lock(self: *SharedMutex, io: std.Io) void {
        if (!self.tryLock(io)) {
            // JPH_PROFILE("WLock", 0xff00ffff) is not ported
            self.base.lockUncancelable(io);
            if (Core.enable_asserts) self.locked_thread_id.store(std.Thread.getCurrentId(), .monotonic);
        }
    }

    /// Release exclusive access (unlock), must be called by the thread that locked it
    pub fn unlock(self: *SharedMutex, io: std.Io) void {
        if (Core.enable_asserts) {
            std.debug.assert(self.locked_thread_id.load(.monotonic) == std.Thread.getCurrentId());
            self.locked_thread_id.store(no_thread, .monotonic);
        }
        self.base.unlock(io);
    }

    /// Returns true when a thread holds the exclusive lock (is_locked), shared locks are not reported.
    /// Jolt only has this function when JPH_ENABLE_ASSERTS is defined, see `Mutex.isLocked`.
    pub fn isLocked(self: *const SharedMutex) bool {
        if (Core.enable_asserts)
            return self.locked_thread_id.load(.monotonic) != no_thread;
        return self.base.isWriting();
    }

    /// Try to lock the mutex for shared access without blocking (try_lock_shared, inherited from std::shared_mutex)
    pub fn tryLockShared(self: *SharedMutex, io: std.Io) bool {
        return self.base.tryLockShared(io);
    }

    /// Lock the mutex for shared access, blocks while a writer holds it (lock_shared). Not cancelable.
    pub fn lockShared(self: *SharedMutex, io: std.Io) void {
        if (!self.tryLockShared(io)) {
            // JPH_PROFILE("RLock", 0xff00ffff) is not ported
            self.base.lockSharedUncancelable(io);
        }
    }

    /// Release shared access (unlock_shared, inherited from std::shared_mutex)
    pub fn unlockShared(self: *SharedMutex, io: std.Io) void {
        self.base.unlockShared(io);
    }
};

test "Mutex lock / tryLock / unlock" {
    const io = std.testing.io;
    var mutex: Mutex = .{};
    try std.testing.expect(!mutex.isLocked());

    mutex.lock(io);
    try std.testing.expect(mutex.isLocked());
    mutex.unlock(io);
    try std.testing.expect(!mutex.isLocked());

    try std.testing.expect(mutex.tryLock());
    try std.testing.expect(mutex.isLocked());
    mutex.unlock(io);
    try std.testing.expect(!mutex.isLocked());
}

test "SharedMutex exclusive and shared locking" {
    const io = std.testing.io;
    var mutex: SharedMutex = .{};
    try std.testing.expect(!mutex.isLocked());

    // Exclusive lock blocks other exclusive and shared lockers
    mutex.lock(io);
    try std.testing.expect(mutex.isLocked());
    try std.testing.expect(!mutex.tryLockShared(io));
    mutex.unlock(io);
    try std.testing.expect(!mutex.isLocked());

    // Shared locks can be held multiple times and block exclusive lockers
    mutex.lockShared(io);
    try std.testing.expect(mutex.tryLockShared(io));
    try std.testing.expect(!mutex.isLocked());
    try std.testing.expect(!mutex.tryLock(io));
    mutex.unlockShared(io);
    mutex.unlockShared(io);

    try std.testing.expect(mutex.tryLock(io));
    try std.testing.expect(mutex.isLocked());
    mutex.unlock(io);
    try std.testing.expect(!mutex.isLocked());
}

test "SharedMutexBase.tryLock excludes readers" {
    // Regression test: std.Io.RwLock.tryLock (Zig 0.16) can succeed while a reader holds the lock
    if (@import("builtin").single_threaded) return error.SkipZigTest;
    const io = std.testing.io;

    const num_iterations = 20_000;

    const Context = struct {
        lock: SharedMutexBase = .{},
        num_readers: std.atomic.Value(u32) = .init(0),
        num_overlaps: std.atomic.Value(u32) = .init(0),

        fn write(self: *@This()) void {
            for (0..num_iterations) |_| {
                while (!self.lock.tryLock(io))
                    std.Thread.yield() catch {};
                if (self.num_readers.load(.seq_cst) != 0)
                    _ = self.num_overlaps.fetchAdd(1, .monotonic);
                self.lock.unlock(io);
            }
        }

        fn read(self: *@This()) void {
            for (0..num_iterations) |_| {
                self.lock.lockSharedUncancelable(io);
                _ = self.num_readers.fetchAdd(1, .seq_cst);
                _ = self.num_readers.fetchSub(1, .seq_cst);
                self.lock.unlockShared(io);
            }
        }
    };

    var context: Context = .{};
    var threads: [3]std.Thread = undefined;
    threads[0] = try std.Thread.spawn(.{}, Context.write, .{&context});
    for (threads[1..]) |*t| t.* = try std.Thread.spawn(.{}, Context.read, .{&context});
    for (threads) |t| t.join();

    try std.testing.expectEqual(@as(u32, 0), context.num_overlaps.load(.monotonic));
    try std.testing.expectEqual(@as(usize, 0), context.lock.state.load(.monotonic));
}

test "Mutex protects a counter between threads" {
    if (@import("builtin").single_threaded) return error.SkipZigTest;
    const io = std.testing.io;

    const num_threads = 4;
    const num_iterations = 20_000;

    const Context = struct {
        mutex: Mutex = .{},
        counter: u64 = 0, // Deliberately not atomic, protected by the mutex
        in_critical_section: std.atomic.Value(u32) = .init(0),
        failed: std.atomic.Value(bool) = .init(false),

        fn run(self: *@This()) void {
            for (0..num_iterations) |i| {
                // Alternate between blocking and non blocking locking
                if (i % 2 == 0) {
                    self.mutex.lock(io);
                } else {
                    while (!self.mutex.tryLock())
                        std.Thread.yield() catch {};
                }
                if (!self.mutex.isLocked() or self.in_critical_section.fetchAdd(1, .monotonic) != 0)
                    self.failed.store(true, .monotonic);
                self.counter += 1;
                _ = self.in_critical_section.fetchSub(1, .monotonic);
                self.mutex.unlock(io);
            }
        }
    };

    var context: Context = .{};
    var threads: [num_threads]std.Thread = undefined;
    for (&threads) |*t| t.* = try std.Thread.spawn(.{}, Context.run, .{&context});
    for (threads) |t| t.join();

    try std.testing.expect(!context.failed.load(.monotonic));
    try std.testing.expectEqual(@as(u64, num_threads * num_iterations), context.counter);
    try std.testing.expect(!context.mutex.isLocked());
}

test "SharedMutex readers and writers" {
    if (@import("builtin").single_threaded) return error.SkipZigTest;
    const io = std.testing.io;

    const num_writers = 2;
    const num_readers = 3;
    const num_iterations = 10_000;

    const Context = struct {
        mutex: SharedMutex = .{},
        // Writers keep both values equal, readers check that they never see a half finished update
        value1: u64 = 0,
        value2: u64 = 0,
        num_readers_inside: std.atomic.Value(u32) = .init(0),
        num_writers_inside: std.atomic.Value(u32) = .init(0),
        failed: std.atomic.Value(bool) = .init(false),

        fn write(self: *@This()) void {
            for (0..num_iterations) |i| {
                if (i % 2 == 0) {
                    self.mutex.lock(io);
                } else {
                    while (!self.mutex.tryLock(io))
                        std.Thread.yield() catch {};
                }
                if (self.num_writers_inside.fetchAdd(1, .monotonic) != 0 or self.num_readers_inside.load(.monotonic) != 0)
                    self.failed.store(true, .monotonic);
                if (!self.mutex.isLocked())
                    self.failed.store(true, .monotonic);
                self.value1 += 1;
                self.value2 += 1;
                _ = self.num_writers_inside.fetchSub(1, .monotonic);
                self.mutex.unlock(io);
            }
        }

        fn read(self: *@This()) void {
            for (0..num_iterations) |i| {
                if (i % 2 == 0) {
                    self.mutex.lockShared(io);
                } else {
                    while (!self.mutex.tryLockShared(io))
                        std.Thread.yield() catch {};
                }
                _ = self.num_readers_inside.fetchAdd(1, .monotonic);
                if (self.num_writers_inside.load(.monotonic) != 0 or self.value1 != self.value2)
                    self.failed.store(true, .monotonic);
                _ = self.num_readers_inside.fetchSub(1, .monotonic);
                self.mutex.unlockShared(io);
            }
        }
    };

    var context: Context = .{};
    var threads: [num_writers + num_readers]std.Thread = undefined;
    for (threads[0..num_writers]) |*t| t.* = try std.Thread.spawn(.{}, Context.write, .{&context});
    for (threads[num_writers..]) |*t| t.* = try std.Thread.spawn(.{}, Context.read, .{&context});
    for (threads) |t| t.join();

    try std.testing.expect(!context.failed.load(.monotonic));
    try std.testing.expectEqual(@as(u64, num_writers * num_iterations), context.value1);
    try std.testing.expectEqual(context.value1, context.value2);
    try std.testing.expect(!context.mutex.isLocked());
}
