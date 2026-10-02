//! Port of: Jolt/Core/MutexArray.h
//! Status: complete
//!
//! Unmanaged container: the allocator is passed to `init` and `deinit`. The C++ default constructor + `Init(n)` and
//! the constructor `MutexArray(n)` are both `init(allocator, n)`; `empty` is a default constructed array.
//! Locking takes `io: std.Io` per call like the mutexes themselves (see "Threading" in the porting guide).
//!
//! The accessors and `lockAll` / `unlockAll` take `*const Self`: they don't change the array, only the mutexes it
//! points to. Jolt's BodyManager stores its MutexArray as a `mutable` member and locks it from const functions.

const std = @import("std");
const Core = @import("Core.zig");
const HashCombine = @import("HashCombine.zig");
const math = @import("../Math/Math.zig");

/// A mutex array protects a number of resources with a limited amount of mutexes.
/// It uses hashing to find the mutex of a particular object.
/// The idea is that if the amount of threads is much smaller than the amount of mutexes
/// that there is a relatively small chance that two different objects map to the same mutex.
///
/// `MutexType` is `Mutex` or `SharedMutex` (Core/Mutex.zig): it needs a default value (`.{}`) and `lock(io)` / `unlock(io)`.
pub fn MutexArray(comptime MutexType: type) type {
    return struct {
        const Self = @This();

        /// Align the mutex to a cache line to ensure there is no false sharing (this is platform dependent, we do this to be safe)
        const MutexStorage = struct {
            mutex: MutexType align(Core.cache_line_size) = .{},
        };

        /// The mutexes, one per cache line
        mutex_storage: []MutexStorage = &.{},

        /// Number of mutexes in the array
        num_mutexes: u32 = 0,

        /// Constructs an empty mutex array that you need to initialize with init (MutexArray())
        pub const empty: Self = .{};

        /// Constructor, constructs an array with `num_mutexes` entries (MutexArray(uint) / Init(uint)).
        /// `num_mutexes` is the amount of mutexes to allocate, it must be a power of 2.
        pub fn init(allocator: std.mem.Allocator, num_mutexes: u32) error{OutOfMemory}!Self {
            std.debug.assert(num_mutexes > 0 and math.isPowerOf2(num_mutexes));

            const mutex_storage = try allocator.alloc(MutexStorage, num_mutexes);
            @memset(mutex_storage, .{});
            return .{ .mutex_storage = mutex_storage, .num_mutexes = num_mutexes };
        }

        /// Destructor, frees the mutexes. None of them may be locked.
        pub fn deinit(self: *Self, allocator: std.mem.Allocator) void {
            allocator.free(self.mutex_storage);
            self.* = .empty;
        }

        /// Get the number of mutexes that were allocated
        pub fn getNumMutexes(self: *const Self) u32 {
            return self.num_mutexes;
        }

        /// Convert an object index to a mutex index
        pub fn getMutexIndex(self: *const Self, object_index: u32) u32 {
            return @truncate(HashCombine.hash(object_index) & (self.num_mutexes - 1));
        }

        /// Get the mutex belonging to a certain object by index
        pub fn getMutexByObjectIndex(self: *const Self, object_index: u32) *MutexType {
            return &self.mutex_storage[self.getMutexIndex(object_index)].mutex;
        }

        /// Get a mutex by index in the array
        pub fn getMutexByIndex(self: *const Self, mutex_index: u32) *MutexType {
            return &self.mutex_storage[mutex_index].mutex;
        }

        /// Lock all mutexes (in index order)
        pub fn lockAll(self: *const Self, io: std.Io) void {
            for (self.mutex_storage) |*m|
                m.mutex.lock(io);
        }

        /// Unlock all mutexes
        pub fn unlockAll(self: *const Self, io: std.Io) void {
            for (self.mutex_storage) |*m|
                m.mutex.unlock(io);
        }
    };
}

const Mutex = @import("Mutex.zig").Mutex;
const SharedMutex = @import("Mutex.zig").SharedMutex;

test "MutexArray layout and indexing" {
    inline for (.{ Mutex, SharedMutex }) |MutexType| {
        const Array = MutexArray(MutexType);
        try std.testing.expectEqual(Core.cache_line_size, @alignOf(Array.MutexStorage));
        try std.testing.expectEqual(Core.cache_line_size, @sizeOf(Array.MutexStorage));

        var empty: Array = .empty;
        empty.deinit(std.testing.allocator);

        var array: Array = try .init(std.testing.allocator, 16);
        defer array.deinit(std.testing.allocator);
        try std.testing.expectEqual(@as(u32, 16), array.getNumMutexes());

        // Every object maps to a mutex in range, by hashing the object index
        var used: [16]bool = @splat(false);
        for (0..1000) |i| {
            const object_index: u32 = @intCast(i);
            const mutex_index = array.getMutexIndex(object_index);
            try std.testing.expect(mutex_index < 16);
            try std.testing.expectEqual(@as(u32, @truncate(HashCombine.hash(object_index) & 15)), mutex_index);
            try std.testing.expectEqual(array.getMutexByIndex(mutex_index), array.getMutexByObjectIndex(object_index));
            used[mutex_index] = true;
        }
        for (used) |u| try std.testing.expect(u);

        // Mutexes are on separate cache lines
        for (0..15) |i|
            try std.testing.expectEqual(@as(usize, Core.cache_line_size), @intFromPtr(array.getMutexByIndex(@intCast(i + 1))) - @intFromPtr(array.getMutexByIndex(@intCast(i))));

        const io = std.testing.io;
        array.lockAll(io);
        for (0..16) |i|
            try std.testing.expect(array.getMutexByIndex(@intCast(i)).isLocked());
        array.unlockAll(io);
        for (0..16) |i|
            try std.testing.expect(!array.getMutexByIndex(@intCast(i)).isLocked());
    }
}

test "MutexArray protects objects between threads" {
    if (@import("builtin").single_threaded) return error.SkipZigTest;
    const io = std.testing.io;

    const num_threads = 4;
    const num_iterations = 10_000;
    const num_objects = 256;

    inline for (.{ Mutex, SharedMutex }) |MutexType| {
        const Context = struct {
            mutexes: MutexArray(MutexType),
            counters: [num_objects]u64 = @splat(0), // Deliberately not atomic, protected by the mutex array
            lock_all_count: u64 = 0,
            failed: std.atomic.Value(bool) = .init(false),

            fn run(self: *@This(), seed: u64) void {
                var rng: std.Random.DefaultPrng = .init(seed);
                for (0..num_iterations) |i| {
                    if (i % 1000 == 999) {
                        // Occasionally lock everything and check the totals
                        self.mutexes.lockAll(io);
                        var sum: u64 = 0;
                        for (self.counters) |c| sum += c;
                        if (sum > num_threads * num_iterations)
                            self.failed.store(true, .monotonic);
                        self.lock_all_count += 1;
                        self.mutexes.unlockAll(io);
                    } else {
                        const object_index = rng.random().uintLessThan(u32, num_objects);
                        const mutex = self.mutexes.getMutexByObjectIndex(object_index);
                        mutex.lock(io);
                        if (!mutex.isLocked())
                            self.failed.store(true, .monotonic);
                        self.counters[object_index] += 1;
                        mutex.unlock(io);
                    }
                }
            }
        };

        var context: Context = .{ .mutexes = try .init(std.testing.allocator, 8) };
        defer context.mutexes.deinit(std.testing.allocator);

        var threads: [num_threads]std.Thread = undefined;
        for (&threads, 0..) |*t, i| t.* = try std.Thread.spawn(.{}, Context.run, .{ &context, @as(u64, i) });
        for (threads) |t| t.join();

        try std.testing.expect(!context.failed.load(.monotonic));
        var sum: u64 = 0;
        for (context.counters) |c| sum += c;
        const num_lock_all = num_threads * (num_iterations / 1000);
        try std.testing.expectEqual(@as(u64, num_threads * num_iterations - num_lock_all), sum);
        try std.testing.expectEqual(@as(u64, num_lock_all), context.lock_all_count);
    }
}
