//! Port of: Jolt/Core/Semaphore.h, Jolt/Core/Semaphore.cpp
//! Status: complete
//!
//! Jolt keeps an atomic counter and only touches the expensive OS semaphore (CreateSemaphore, sem_t,
//! dispatch_semaphore_t) when a thread actually has to wait. Zolt keeps that fast path and replaces the OS semaphore
//! with `FutexSemaphore`, a counting semaphore on top of the std.Io futex (`io.futexWait` / `io.futexWake`), which is
//! also how sem_t is implemented on Linux. Jolt's fallback for other platforms (std::mutex + std::condition_variable)
//! is not needed, std.Io provides the platform layer.
//!
//! Like the other small synchronization primitives, `Semaphore` takes `io: std.Io` per call (see "Threading" in the
//! porting guide). It needs no deinit, a futex owns no OS resources.

const std = @import("std");
const Core = @import("Core.zig");

/// Implements a semaphore
/// When we switch to C++20 we can use counting_semaphore to unify this
///
/// Usage: `var semaphore: Semaphore = .{};`, `semaphore.release(io, .{ .number = 3 })` adds 3 to the count and wakes
/// waiting threads, `semaphore.acquire(io, .{})` takes 1 from the count and blocks while it is not available.
pub const Semaphore = struct {
    /// We increment count for every release, to acquire we decrement the count. If the count is negative we know that we are waiting on the actual semaphore.
    count: std.atomic.Value(i32) align(Core.cache_line_size) = .init(0),

    /// The semaphore is an expensive construct so we only acquire/release it if we know that we need to wait/have waiting threads
    semaphore: FutexSemaphore = .{},

    /// Release the semaphore, signaling the thread waiting on the barrier that there may be work.
    /// `opts.number` (default 1) is the amount to add to the semaphore count.
    pub fn release(self: *Semaphore, io: std.Io, opts: struct { number: u32 = 1 }) void {
        std.debug.assert(opts.number > 0);

        const number: i32 = @bitCast(opts.number);
        const old_value = self.count.fetchAdd(number, .release);
        if (old_value < 0) {
            const new_value = old_value + number;
            const num_to_release = @min(new_value, 0) - old_value;
            self.semaphore.post(io, @intCast(num_to_release));
        }
    }

    /// Acquire the semaphore `opts.number` (default 1) times, blocks until the count allows it. Not cancelable.
    pub fn acquire(self: *Semaphore, io: std.Io, opts: struct { number: u32 = 1 }) void {
        std.debug.assert(opts.number > 0);

        const number: i32 = @bitCast(opts.number);
        const old_value = self.count.fetchSub(number, .acquire);
        const new_value = old_value - number;
        if (new_value < 0) {
            const num_to_acquire = @min(old_value, 0) - new_value;
            var i: i32 = 0;
            while (i < num_to_acquire) : (i += 1)
                self.semaphore.wait(io);
        }
    }

    /// Get the current value of the semaphore (negative when threads are waiting)
    pub fn getValue(self: *const Semaphore) i32 {
        return self.count.load(.monotonic);
    }
};

/// Counting semaphore on top of the std.Io futex, the replacement for the OS semaphore that Jolt uses
/// (sem_post / sem_wait, ReleaseSemaphore / WaitForSingleObject, ...).
const FutexSemaphore = struct {
    /// Number of times that `wait` can return without blocking
    permits: std.atomic.Value(u32) = .init(0),

    /// Add `number` permits and wake up to `number` waiting threads (sem_post called `number` times)
    fn post(self: *FutexSemaphore, io: std.Io, number: u32) void {
        _ = self.permits.fetchAdd(number, .release);
        io.futexWake(u32, &self.permits.raw, number);
    }

    /// Take one permit, blocks while there are none (sem_wait). Not cancelable.
    fn wait(self: *FutexSemaphore, io: std.Io) void {
        var permits = self.permits.load(.monotonic);
        while (true) {
            if (permits == 0) {
                // Sleep until post changes the value, spurious wake ups are handled by checking again
                io.futexWaitUncancelable(u32, &self.permits.raw, 0);
                permits = self.permits.load(.monotonic);
            } else {
                permits = self.permits.cmpxchgWeak(permits, permits - 1, .acquire, .monotonic) orelse return;
            }
        }
    }
};

test "Semaphore single threaded counting" {
    const io = std.testing.io;
    var semaphore: Semaphore = .{};
    try std.testing.expectEqual(@as(i32, 0), semaphore.getValue());

    semaphore.release(io, .{});
    try std.testing.expectEqual(@as(i32, 1), semaphore.getValue());
    semaphore.release(io, .{ .number = 5 });
    try std.testing.expectEqual(@as(i32, 6), semaphore.getValue());

    // These don't block because the count is high enough
    semaphore.acquire(io, .{ .number = 4 });
    try std.testing.expectEqual(@as(i32, 2), semaphore.getValue());
    semaphore.acquire(io, .{});
    semaphore.acquire(io, .{});
    try std.testing.expectEqual(@as(i32, 0), semaphore.getValue());

    // The OS semaphore was never needed
    try std.testing.expectEqual(@as(u32, 0), semaphore.semaphore.permits.load(.monotonic));
}

test "Semaphore acquire blocks until released" {
    if (@import("builtin").single_threaded) return error.SkipZigTest;
    const io = std.testing.io;

    const Context = struct {
        semaphore: Semaphore = .{},
        acquired: std.atomic.Value(bool) = .init(false),

        fn waiter(self: *@This()) void {
            self.semaphore.acquire(io, .{ .number = 3 });
            self.acquired.store(true, .release);
        }
    };

    var context: Context = .{};
    const thread = try std.Thread.spawn(.{}, Context.waiter, .{&context});

    // Wait until the waiter has decremented the count (it then blocks on the futex semaphore)
    while (context.semaphore.getValue() != -3)
        std.Thread.yield() catch {};
    try std.testing.expect(!context.acquired.load(.acquire));

    // Release one at a time, the waiter can only continue after the third release
    context.semaphore.release(io, .{});
    try std.testing.expectEqual(@as(i32, -2), context.semaphore.getValue());
    context.semaphore.release(io, .{});
    try std.testing.expectEqual(@as(i32, -1), context.semaphore.getValue());
    try std.testing.expect(!context.acquired.load(.acquire));
    context.semaphore.release(io, .{});

    thread.join();
    try std.testing.expect(context.acquired.load(.acquire));
    try std.testing.expectEqual(@as(i32, 0), context.semaphore.getValue());
    try std.testing.expectEqual(@as(u32, 0), context.semaphore.semaphore.permits.load(.monotonic));
}

test "Semaphore producers and consumers" {
    if (@import("builtin").single_threaded) return error.SkipZigTest;
    const io = std.testing.io;

    const num_producers = 2;
    const num_consumers = 3;
    // Total number of units that go through the semaphore, produced in chunks of 1..5 and consumed in chunks of 1..3
    const total = 2 * 3 * 4 * 2_000;

    const Context = struct {
        semaphore: Semaphore = .{},
        produced: std.atomic.Value(u32) = .init(0),
        consumed: std.atomic.Value(u32) = .init(0),
        failed: std.atomic.Value(bool) = .init(false),

        fn produce(self: *@This(), seed: u32) void {
            var rng: std.Random.DefaultPrng = .init(seed);
            var remaining: u32 = total / num_producers;
            while (remaining > 0) {
                const n = @min(remaining, rng.random().intRangeAtMost(u32, 1, 5));
                _ = self.produced.fetchAdd(n, .seq_cst);
                self.semaphore.release(io, .{ .number = n });
                remaining -= n;
            }
        }

        fn consume(self: *@This(), chunk: u32) void {
            var remaining: u32 = total / num_consumers;
            while (remaining > 0) {
                const n = @min(remaining, chunk);
                self.semaphore.acquire(io, .{ .number = n });
                // Can never consume more than was produced
                if (self.consumed.fetchAdd(n, .seq_cst) + n > self.produced.load(.seq_cst))
                    self.failed.store(true, .monotonic);
                remaining -= n;
            }
        }
    };

    var context: Context = .{};
    var threads: [num_producers + num_consumers]std.Thread = undefined;
    for (threads[0..num_consumers], 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Context.consume, .{ &context, @as(u32, @intCast(i + 1)) });
    for (threads[num_consumers..], 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Context.produce, .{ &context, @as(u32, @intCast(i + 1)) });
    for (threads) |t| t.join();

    try std.testing.expect(!context.failed.load(.monotonic));
    try std.testing.expectEqual(@as(u32, total), context.produced.load(.monotonic));
    try std.testing.expectEqual(@as(u32, total), context.consumed.load(.monotonic));
    try std.testing.expectEqual(@as(i32, 0), context.semaphore.getValue());
    try std.testing.expectEqual(@as(u32, 0), context.semaphore.semaphore.permits.load(.monotonic));
}

test "Semaphore worker wake ups" {
    // The pattern of JobSystemThreadPool: workers block in acquire, the producer wakes up to num_workers of them at a time
    if (@import("builtin").single_threaded) return error.SkipZigTest;
    const io = std.testing.io;

    const num_workers = 4;
    const num_rounds = 5_000;

    const Context = struct {
        semaphore: Semaphore = .{},
        quit: std.atomic.Value(bool) = .init(false),
        num_wake_ups: std.atomic.Value(u32) = .init(0),

        fn worker(self: *@This()) void {
            while (true) {
                self.semaphore.acquire(io, .{});
                if (self.quit.load(.acquire))
                    return;
                _ = self.num_wake_ups.fetchAdd(1, .monotonic);
            }
        }
    };

    var context: Context = .{};
    var threads: [num_workers]std.Thread = undefined;
    for (&threads) |*t| t.* = try std.Thread.spawn(.{}, Context.worker, .{&context});

    var total: u32 = 0;
    for (0..num_rounds) |round| {
        const number: u32 = @intCast(round % num_workers + 1);
        context.semaphore.release(io, .{ .number = number });
        total += number;
        // The count never exceeds what was released, it is negative while workers wait
        try std.testing.expect(context.semaphore.getValue() <= @as(i32, @intCast(total)));
    }

    // Every release wakes up exactly one acquire
    while (context.num_wake_ups.load(.monotonic) != total)
        std.Thread.yield() catch {};

    // Stop the workers
    context.quit.store(true, .release);
    context.semaphore.release(io, .{ .number = num_workers });
    for (threads) |t| t.join();

    try std.testing.expectEqual(total, context.num_wake_ups.load(.monotonic));
    try std.testing.expectEqual(@as(i32, 0), context.semaphore.getValue());
    try std.testing.expectEqual(@as(u32, 0), context.semaphore.semaphore.permits.load(.monotonic));
}
