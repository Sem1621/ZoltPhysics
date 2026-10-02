//! Port of: Jolt/Core/JobSystemWithBarrier.h, Jolt/Core/JobSystemWithBarrier.cpp
//! Status: complete
//!
//! Differences with the C++ version:
//! - In C++ a job system derives from JobSystemWithBarrier to inherit CreateBarrier / DestroyBarrier / WaitForJobs.
//!   In Zolt it embeds `base: JobSystemWithBarrier`, declares `createBarrier`, `destroyBarrier` and `waitForJobs`
//!   functions that forward to it, and creates the JobSystem interface from itself (see JobSystemThreadPool).
//! - `JobSystemWithBarrier()` + `Init(inMaxBarriers)` / `JobSystemWithBarrier(inMaxBarriers)` ->
//!   `init(allocator, io, max_barriers)`, the destructor -> `deinit()` (`empty` is the default constructed state).
//!   Every barrier stores the io for its semaphore, because Barrier.addJob has no io parameter.
//! - The "Barrier full, stalling!" assert panics when asserts are enabled (Core.enable_asserts, see the porting guide
//!   for JPH_ASSERT(false) on paths that release builds handle), release builds sleep and retry like Jolt.
//! - Jolt aligns mJobReadIndex and mJobWriteIndex to cache lines (mNumToAcquire shares the cache line of
//!   mJobWriteIndex). Zig reorders struct fields, so they are grouped in cache line aligned structs that occupy whole
//!   cache lines: `read_state.job_read_index`, `write_state.job_write_index` and `write_state.num_to_acquire`.
//!
//! Note (inherited from Jolt): a job that is added to a barrier while another thread waits on it must not be able to
//! finish before addJob / addJobs has returned, unless another job in the barrier (besides the one calling addJob)
//! stays pending until then. addJob sets the barrier on the job before it increments num_to_acquire, so a job that
//! finishes in between releases the semaphore before it is counted, and wait() can return while jobs are still
//! running. Create such jobs with a dependency and remove it after adding them (PhysicsSystem guarantees this through
//! jobs that depend on each other).

const std = @import("std");
const Core = @import("Core.zig");
const JobSystem = @import("JobSystem.zig").JobSystem;
const Semaphore = @import("Semaphore.zig").Semaphore;
const Color = @import("Color.zig").Color;
const math = @import("../Math/Math.zig");

const Job = JobSystem.Job;
const JobHandle = JobSystem.JobHandle;
const Barrier = JobSystem.Barrier;

/// Implementation of the Barrier class for a JobSystem
///
/// This class can be used to make it easier to create a new JobSystem implementation that integrates with your own job system.
/// It will implement all functionality relating to barriers, so the only functions that are left to be implemented are:
///
/// * JobSystem.getMaxConcurrency
/// * JobSystem.createJob
/// * JobSystem.freeJob
/// * JobSystem.queueJob/queueJobs
///
/// See instructions in JobSystem for more information on how to implement these.
///
/// Usage: embed it as `base: JobSystemWithBarrier` and forward `createBarrier`, `destroyBarrier` and `waitForJobs`
/// to it (see JobSystemThreadPool).
pub const JobSystemWithBarrier = struct {
    /// Allocator for the barriers
    allocator: std.mem.Allocator = undefined,

    /// Max amount of barriers
    max_barriers: u32 = 0,

    /// List of the actual barriers (we keep them constructed all the time since constructing a semaphore/mutex is not cheap)
    barriers: ?[*]BarrierImpl = null,

    /// Not initialized (default constructor), deinit does nothing
    pub const empty: JobSystemWithBarrier = .{};

    /// Constructs barriers
    /// max_barriers: Max number of barriers that can be allocated at any time
    pub fn init(allocator: std.mem.Allocator, io: std.Io, max_barriers: u32) error{OutOfMemory}!JobSystemWithBarrier {
        // Init freelist of barriers
        const barriers = try allocator.alloc(BarrierImpl, max_barriers);
        for (barriers) |*barrier|
            barrier.* = .init(io);
        return .{ .allocator = allocator, .max_barriers = max_barriers, .barriers = barriers.ptr };
    }

    /// Destructor, all barriers must have been destroyed
    pub fn deinit(self: *JobSystemWithBarrier) void {
        if (self.barriers) |barriers_ptr| {
            const barriers = barriers_ptr[0..self.max_barriers];

            // Ensure that none of the barriers are used
            if (Core.enable_asserts) {
                for (barriers) |*barrier|
                    std.debug.assert(!barrier.in_use.load(.seq_cst));
            }

            for (barriers) |*barrier|
                barrier.deinit();
            self.allocator.free(barriers);
        }
        self.* = .empty;
    }

    /// Create a new barrier, used to wait on jobs (null when all barriers are in use)
    pub fn createBarrier(self: *JobSystemWithBarrier) ?*Barrier {
        // Find the first unused barrier
        const barriers = self.barriers orelse return null;
        for (barriers[0..self.max_barriers]) |*barrier| {
            if (barrier.in_use.cmpxchgStrong(false, true, .seq_cst, .seq_cst) == null)
                return &barrier.base;
        }

        return null;
    }

    /// Destroy a barrier when it is no longer used. The barrier should be empty at this point.
    pub fn destroyBarrier(self: *JobSystemWithBarrier, barrier: *Barrier) void {
        _ = self;
        const barrier_impl = BarrierImpl.fromBarrier(barrier);

        // Check that no jobs are in the barrier
        std.debug.assert(barrier_impl.isEmpty());

        // Flag the barrier as unused
        const result = barrier_impl.in_use.cmpxchgStrong(true, false, .seq_cst, .seq_cst);
        std.debug.assert(result == null); // The barrier was in use
    }

    /// Wait for a set of jobs to be finished, note that only 1 thread can be waiting on a barrier at a time
    pub fn waitForJobs(self: *JobSystemWithBarrier, barrier: *Barrier) void {
        _ = self;

        // Let our barrier implementation wait for the jobs
        BarrierImpl.fromBarrier(barrier).wait();
    }

    const BarrierImpl = struct {
        /// The Barrier interface
        base: Barrier = .{ .vtable = &vtable },

        /// Flag to indicate if a barrier has been handed out
        in_use: std.atomic.Value(bool) = .init(false),

        /// Io used to release and acquire the semaphore
        io: std.Io,

        /// List of jobs that are part of this barrier, nullptrs for empty slots
        jobs: [max_jobs]std.atomic.Value(?*Job) = @splat(.init(null)),

        /// mJobReadIndex on its own cache line (see the top of this file)
        read_state: ReadState = .{},

        /// mJobWriteIndex and mNumToAcquire on their own cache line (see the top of this file)
        write_state: WriteState = .{},

        /// Semaphore used by finishing jobs to signal the barrier that they're done
        semaphore: Semaphore = .{},

        /// Jobs queue for the barrier
        const max_jobs: u32 = 2048;
        comptime {
            std.debug.assert(math.isPowerOf2(max_jobs)); // We do bit operations and require max jobs to be a power of 2
        }

        const ReadState = struct {
            /// First job that could be valid (modulo max_jobs), can be nullptr if other thread is still working on adding the job
            job_read_index: std.atomic.Value(u32) align(Core.cache_line_size) = .init(0),
        };

        const WriteState = struct {
            /// First job that can be written (modulo max_jobs)
            job_write_index: std.atomic.Value(u32) align(Core.cache_line_size) = .init(0),

            /// Number of times the semaphore has been released, the barrier should acquire the semaphore this many times (written at the same time as job_write_index so ok to put in same cache line)
            num_to_acquire: std.atomic.Value(i32) = .init(0),
        };

        comptime {
            // The read and write indices must not share a cache line with each other or with any other member
            for ([_]type{ ReadState, WriteState }) |State|
                std.debug.assert(@alignOf(State) == Core.cache_line_size and @sizeOf(State) % Core.cache_line_size == 0);
        }

        const vtable: Barrier.VTable = .{
            .addJob = addJobImpl,
            .addJobs = addJobsImpl,
            .onJobFinished = onJobFinishedImpl,
        };

        /// Constructor
        fn init(io: std.Io) BarrierImpl {
            return .{ .io = io };
        }

        /// Destructor
        fn deinit(self: *BarrierImpl) void {
            std.debug.assert(self.isEmpty());
        }

        /// Downcast (static_cast<BarrierImpl *>(inBarrier) in C++)
        fn fromBarrier(barrier: *Barrier) *BarrierImpl {
            return @alignCast(@fieldParentPtr("base", barrier));
        }

        fn addJobImpl(barrier: *Barrier, job: *const JobHandle) void {
            fromBarrier(barrier).addJob(job);
        }

        fn addJobsImpl(barrier: *Barrier, handles: []const JobHandle) void {
            fromBarrier(barrier).addJobs(handles);
        }

        fn onJobFinishedImpl(barrier: *Barrier, job: *Job) void {
            fromBarrier(barrier).onJobFinished(job);
        }

        /// Add a job to this barrier
        fn addJob(self: *BarrierImpl, job_handle: *const JobHandle) void {
            var release_semaphore = false;

            // Set the barrier on the job, this returns true if the barrier was successfully set (otherwise the job is already done and we don't need to add it to our list)
            const job = job_handle.getPtr().?;
            if (job.setBarrier(&self.base)) {
                // If the job can be executed we want to release the semaphore an extra time to allow the waiting thread to start executing it
                _ = self.write_state.num_to_acquire.fetchAdd(1, .seq_cst);
                if (job.canBeExecuted()) {
                    release_semaphore = true;
                    _ = self.write_state.num_to_acquire.fetchAdd(1, .seq_cst);
                }

                // Add the job to our job list
                self.appendJob(job);
            }

            // Notify waiting thread that a new executable job is available
            if (release_semaphore)
                self.semaphore.release(self.io, .{});
        }

        /// Add multiple jobs to this barrier
        fn addJobs(self: *BarrierImpl, handles: []const JobHandle) void {
            var release_semaphore = false;

            for (handles) |*handle| {
                // Set the barrier on the job, this returns true if the barrier was successfully set (otherwise the job is already done and we don't need to add it to our list)
                const job = handle.getPtr().?;
                if (job.setBarrier(&self.base)) {
                    // If the job can be executed we want to release the semaphore an extra time to allow the waiting thread to start executing it
                    _ = self.write_state.num_to_acquire.fetchAdd(1, .seq_cst);
                    if (!release_semaphore and job.canBeExecuted()) {
                        release_semaphore = true;
                        _ = self.write_state.num_to_acquire.fetchAdd(1, .seq_cst);
                    }

                    // Add the job to our job list
                    self.appendJob(job);
                }
            }

            // Notify waiting thread that a new executable job is available
            if (release_semaphore)
                self.semaphore.release(self.io, .{});
        }

        /// Add the job to our job list (shared by AddJob and AddJobs in Jolt)
        fn appendJob(self: *BarrierImpl, job: *Job) void {
            job.addRef();
            const write_index = self.write_state.job_write_index.fetchAdd(1, .seq_cst);
            while (write_index -% self.read_state.job_read_index.load(.seq_cst) >= max_jobs) {
                if (Core.enable_asserts) @panic("Barrier full, stalling!");
                sleepUncancelable(self.io, .fromMicroseconds(100));
            }
            self.jobs[write_index & (max_jobs - 1)].store(job, .seq_cst);
        }

        /// Check if there are any jobs in the job barrier
        fn isEmpty(self: *const BarrierImpl) bool {
            return self.read_state.job_read_index.load(.seq_cst) == self.write_state.job_write_index.load(.seq_cst);
        }

        /// Called by a Job to mark that it is finished
        fn onJobFinished(self: *BarrierImpl, job: *Job) void {
            _ = job;
            self.semaphore.release(self.io, .{});
        }

        /// Wait for all jobs in this job barrier, while waiting, execute jobs that are part of this barrier on the current thread
        fn wait(self: *BarrierImpl) void {
            while (self.write_state.num_to_acquire.load(.seq_cst) > 0) {
                {
                    // Go through all jobs
                    var has_executed: bool = undefined;
                    while (true) {
                        has_executed = false;

                        // Loop through the jobs and erase jobs from the beginning of the list that are done
                        while (self.read_state.job_read_index.load(.seq_cst) < self.write_state.job_write_index.load(.seq_cst)) {
                            const job = &self.jobs[self.read_state.job_read_index.load(.seq_cst) & (max_jobs - 1)];
                            const job_ptr = job.load(.seq_cst) orelse break;
                            if (!job_ptr.isDone())
                                break;

                            // Job is finished, release it
                            job_ptr.release();
                            job.store(null, .seq_cst);
                            _ = self.read_state.job_read_index.fetchAdd(1, .seq_cst);
                        }

                        // Loop through the jobs and execute the first executable job
                        var index = self.read_state.job_read_index.load(.seq_cst);
                        while (index < self.write_state.job_write_index.load(.seq_cst)) : (index += 1) {
                            const job = &self.jobs[index & (max_jobs - 1)];
                            const job_ptr = job.load(.seq_cst);
                            if (job_ptr != null and job_ptr.?.canBeExecuted()) {
                                // This will only execute the job if it has not already executed
                                _ = job_ptr.?.execute();
                                has_executed = true;
                                break;
                            }
                        }

                        if (!has_executed)
                            break;
                    }
                }

                // Wait for another thread to wake us when either there is more work to do or when all jobs have completed.
                // When there have been multiple releases, we acquire them all at the same time to avoid needlessly spinning on executing jobs.
                // Note that using GetValue is inherently unsafe since we can read a stale value, but this is not an issue here as this is the only
                // place where we acquire the semaphore. Other threads only release it, so we can only read a value that is lower or equal to the actual value.
                const num_to_acquire = @max(1, self.semaphore.getValue());
                self.semaphore.acquire(self.io, .{ .number = @intCast(num_to_acquire) });
                _ = self.write_state.num_to_acquire.fetchSub(num_to_acquire, .seq_cst);
            }

            // All jobs should be done now, release them
            while (self.read_state.job_read_index.load(.seq_cst) < self.write_state.job_write_index.load(.seq_cst)) {
                const job = &self.jobs[self.read_state.job_read_index.load(.seq_cst) & (max_jobs - 1)];
                const job_ptr = job.load(.seq_cst);
                std.debug.assert(job_ptr != null and job_ptr.?.isDone());
                job_ptr.?.release();
                job.store(null, .seq_cst);
                _ = self.read_state.job_read_index.fetchAdd(1, .seq_cst);
            }
        }
    };
};

/// std::this_thread::sleep_for, not cancelable (see "Threading" in the porting guide). Also used by JobSystemThreadPool.
pub fn sleepUncancelable(io: std.Io, duration: std.Io.Duration) void {
    const old_cancel_protection = io.swapCancelProtection(.blocked);
    defer _ = io.swapCancelProtection(old_cancel_protection);
    io.sleep(duration, .awake) catch |err| switch (err) {
        error.Canceled => unreachable, // Cancelation is blocked
    };
}

/// A job system for the tests below that has no threads: like JobSystemThreadPool without worker threads, queueJob
/// does nothing and the jobs are executed by the barrier while waiting.
const TestJobSystem = struct {
    base: JobSystemWithBarrier,
    jobs: FixedSizeFreeList(Job),

    const FixedSizeFreeList = @import("FixedSizeFreeList.zig").FixedSizeFreeList;

    fn init(allocator: std.mem.Allocator, io: std.Io, max_jobs: u32, max_barriers: u32) !TestJobSystem {
        var base: JobSystemWithBarrier = try .init(allocator, io, max_barriers);
        errdefer base.deinit();
        return .{ .base = base, .jobs = try .init(allocator, io, max_jobs, max_jobs) };
    }

    fn deinit(self: *TestJobSystem) void {
        self.jobs.deinit();
        self.base.deinit();
    }

    fn jobSystem(self: *TestJobSystem) JobSystem {
        return .init(self);
    }

    pub fn getMaxConcurrency(self: *const TestJobSystem) i32 {
        _ = self;
        return 1;
    }

    pub fn createJob(self: *TestJobSystem, job_name: []const u8, color: Color, job_function: JobSystem.JobFunction, opts: struct { num_dependencies: u32 = 0 }) error{OutOfMemory}!JobHandle {
        const index = try self.jobs.constructObject(.init(job_name, color, self.jobSystem(), job_function, opts.num_dependencies));
        std.debug.assert(index != FixedSizeFreeList(Job).invalid_object_index);
        const job = self.jobs.get(index);
        const handle: JobHandle = .init(job);
        if (opts.num_dependencies == 0)
            self.queueJob(job);
        return handle;
    }

    pub fn createBarrier(self: *TestJobSystem) ?*Barrier {
        return self.base.createBarrier();
    }

    pub fn destroyBarrier(self: *TestJobSystem, barrier: *Barrier) void {
        self.base.destroyBarrier(barrier);
    }

    pub fn waitForJobs(self: *TestJobSystem, barrier: *Barrier) void {
        self.base.waitForJobs(barrier);
    }

    pub fn queueJob(self: *TestJobSystem, job: *Job) void {
        _ = self;
        _ = job;
    }

    pub fn queueJobs(self: *TestJobSystem, jobs: []const *Job) void {
        _ = self;
        _ = jobs;
    }

    pub fn freeJob(self: *TestJobSystem, job: *Job) void {
        self.jobs.destructObjectPtr(job);
    }
};

test "JobSystemWithBarrier create and destroy barriers" {
    var test_system: TestJobSystem = try .init(std.testing.allocator, std.testing.io, 16, 3);
    defer test_system.deinit();
    const job_system = test_system.jobSystem();

    // Barriers are handed out in order until all are in use
    const barriers = test_system.base.barriers.?;
    const barrier1 = job_system.createBarrier().?;
    const barrier2 = job_system.createBarrier().?;
    const barrier3 = job_system.createBarrier().?;
    try std.testing.expectEqual(&barriers[0].base, barrier1);
    try std.testing.expectEqual(&barriers[1].base, barrier2);
    try std.testing.expectEqual(&barriers[2].base, barrier3);
    try std.testing.expectEqual(null, job_system.createBarrier());

    // A destroyed barrier is reused
    job_system.destroyBarrier(barrier2);
    try std.testing.expect(!barriers[1].in_use.load(.seq_cst));
    try std.testing.expectEqual(barrier2, job_system.createBarrier().?);

    // Waiting on an empty barrier returns immediately
    job_system.waitForJobs(barrier1);

    job_system.destroyBarrier(barrier3);
    job_system.destroyBarrier(barrier2);
    job_system.destroyBarrier(barrier1);

    // A job system without barriers
    var no_barriers: JobSystemWithBarrier = try .init(std.testing.allocator, std.testing.io, 0);
    defer no_barriers.deinit();
    try std.testing.expectEqual(null, no_barriers.createBarrier());
    var not_initialized: JobSystemWithBarrier = .empty;
    try std.testing.expectEqual(null, not_initialized.createBarrier());
    not_initialized.deinit();
}

test "JobSystemWithBarrier executes jobs while waiting" {
    const num_jobs = 64;
    var test_system: TestJobSystem = try .init(std.testing.allocator, std.testing.io, num_jobs, 1);
    defer test_system.deinit();
    const job_system = test_system.jobSystem();

    const Context = struct {
        counter: u32 = 1,
        values: [num_jobs]u32 = @splat(0),
        handles: [num_jobs]JobHandle = @splat(.empty),

        fn run(self: *@This(), i: usize) void {
            // Set sequence number
            self.values[i] = self.counter;
            self.counter += 1;

            // Start previous job
            if (i > 0)
                self.handles[i - 1].removeDependency(.{});
        }
    };
    var context: Context = .{};
    defer for (&context.handles) |*handle| handle.deinit();

    // A chain of jobs that is started from the last job, the jobs are executed by the waiting thread
    const barrier = job_system.createBarrier().?;
    for (&context.handles, 0..) |*handle, i| {
        handle.* = try job_system.createJob("Chain", Color.red, .init(Context.run, .{ &context, i }), .{ .num_dependencies = 1 });
        if (i % 2 == 0)
            barrier.addJob(handle)
        else
            barrier.addJobs(handle[0..1]);
    }
    const barrier_impl = JobSystemWithBarrier.BarrierImpl.fromBarrier(barrier);
    try std.testing.expectEqual(@as(i32, num_jobs), barrier_impl.write_state.num_to_acquire.load(.seq_cst));
    context.handles[num_jobs - 1].removeDependency(.{});
    job_system.waitForJobs(barrier);
    try std.testing.expect(barrier_impl.isEmpty());
    job_system.destroyBarrier(barrier);

    // Jobs were executed in reverse order
    for (context.values, 0..) |value, i|
        try std.testing.expectEqual(@as(u32, @intCast(num_jobs - i)), value);
    for (&context.handles) |*handle| try std.testing.expect(handle.isDone());
}

test "JobSystemWithBarrier jobs added while waiting" {
    var test_system: TestJobSystem = try .init(std.testing.allocator, std.testing.io, 32, 1);
    defer test_system.deinit();

    // Each job spawns two new jobs until the depth is reached (a binary tree of 2^5 - 1 jobs), the new jobs are added to the barrier while it is being waited on
    const Context = struct {
        job_system: JobSystem,
        barrier: *Barrier,
        num_executed: u32 = 0,
        max_depth: u32 = 5,

        fn run(self: *@This(), depth: u32) void {
            self.num_executed += 1;
            if (depth + 1 < self.max_depth) {
                for (0..2) |_| {
                    var handle = self.job_system.createJob("Child", Color.green, .init(run, .{ self, depth + 1 }), .{}) catch @panic("out of memory");
                    defer handle.deinit();
                    self.barrier.addJob(&handle);
                }
            }
        }
    };

    const job_system = test_system.jobSystem();
    const barrier = job_system.createBarrier().?;
    var context: Context = .{ .job_system = job_system, .barrier = barrier };
    {
        var root = try job_system.createJob("Root", Color.green, .init(Context.run, .{ &context, @as(u32, 0) }), .{});
        defer root.deinit();
        barrier.addJobs((&root)[0..1]);
    }
    job_system.waitForJobs(barrier);
    job_system.destroyBarrier(barrier);
    try std.testing.expectEqual(@as(u32, 31), context.num_executed);
}

test "JobSystemWithBarrier finished jobs and many jobs" {
    const max_jobs = 512;
    var test_system: TestJobSystem = try .init(std.testing.allocator, std.testing.io, max_jobs, 1);
    defer test_system.deinit();
    const job_system = test_system.jobSystem();

    const Context = struct {
        fn increment(value: *u32) void {
            value.* += 1;
        }
    };
    var value: u32 = 0;

    // A job that already finished is not added to the barrier
    const barrier = job_system.createBarrier().?;
    const barrier_impl = JobSystemWithBarrier.BarrierImpl.fromBarrier(barrier);
    {
        var handle = try job_system.createJob("Done", Color.blue, .init(Context.increment, .{&value}), .{});
        defer handle.deinit();
        try std.testing.expectEqual(Job.done_state, handle.getPtr().?.execute());
        barrier.addJob(&handle);
        try std.testing.expect(barrier_impl.isEmpty());
        try std.testing.expectEqual(@as(i32, 0), barrier_impl.write_state.num_to_acquire.load(.seq_cst));
    }
    try std.testing.expectEqual(@as(u32, 1), value);

    // Run more jobs through the barrier than fit in its job list (the read and write indices wrap around the list)
    var handles: [max_jobs]JobHandle = @splat(.empty);
    for (0..10) |round| {
        for (&handles) |*handle|
            handle.* = try job_system.createJob("Many", Color.blue, .init(Context.increment, .{&value}), .{});
        barrier.addJobs(&handles);
        for (&handles) |*handle| handle.deinit();
        job_system.waitForJobs(barrier);
        try std.testing.expect(barrier_impl.isEmpty());
        try std.testing.expectEqual(@as(u32, @intCast(1 + (round + 1) * max_jobs)), value);
    }
    try std.testing.expectEqual(@as(u32, 10 * max_jobs), barrier_impl.read_state.job_read_index.load(.seq_cst));
    job_system.destroyBarrier(barrier);
}
