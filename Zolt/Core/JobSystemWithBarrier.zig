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
//! - Fix of a race in Jolt (scheduling only, the jobs and the results are the same): Jolt's AddJob / AddJobs set the
//!   barrier on a job (SetBarrier) before they count it (mNumToAcquire++).
//!   - A job that finishes in between releases the semaphore before it is counted. Wait() can spend that release on
//!     the count of the job that is adding. If no other job of the barrier is pending, Wait() returns while the adding
//!     job is still in AddJobs, which then reads handles whose owner may already have freed them.
//!   - PhysicsSystem's soft body step hits this. JobSoftBodyPrepare creates the SoftBodyCollide jobs with no
//!     dependency, so they are queued when they are created and added to the barrier only afterwards. In the last
//!     collision step, no other job of the barrier waits for the prepare job. Jolt v5.6.1 and Jolt's master branch
//!     have this order.
//!   - Zolt counts the job before `setBarrier`, so a release never comes before its count. The count is a seq_cst
//!     read-modify-write, like the subtraction in wait(), so a waiter that acquired the release of the job also sees
//!     its count.
//!   - When `setBarrier` fails (the job is already done), addJob / addJobs release the semaphore for that count
//!     instead of taking it back. Suppose the adding thread is not a job of the barrier and the other jobs of the
//!     barrier finish in the meantime. A waiting thread can then already be blocked on that count, and taking it back
//!     would never wake it up. The note below rules this case out, and in it Jolt's Wait() can return early. wait()
//!     acquires the release together with the others. Until a thread waits, the barrier holds the count and its
//!     release.
//! - `test_hook` (test builds only) lets the tests at the end of this file finish a job in the middle of addJob /
//!   addJobs, before and after the barrier is set on it.
//!
//! Note (inherited from Jolt): jobs are added to the barrier while it is being waited on by jobs of that barrier.
//! A thread that is not a job of the barrier can only add jobs while another job of the barrier stays pending until
//! addJob / addJobs has returned. Otherwise wait() can return before the jobs are added.

const std = @import("std");
const builtin = @import("builtin");
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

        /// Zolt: test builds only, called by addJob / addJobs before and after they set the barrier on a job (see the
        /// top of this file)
        test_hook: if (builtin.is_test) ?TestHook else void = if (builtin.is_test) null else {},

        const TestHook = struct {
            context: *anyopaque,
            function: *const fn (context: *anyopaque, job: *Job, point: TestHookPoint) void,
        };

        const TestHookPoint = enum { before_set_barrier, after_set_barrier };

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
            var num_already_done: u32 = 0;

            // Zolt: count the job before setting the barrier on it, so that it cannot release the semaphore before it is counted (see the top of this file)
            const job = job_handle.getPtr().?;
            _ = self.write_state.num_to_acquire.fetchAdd(1, .seq_cst);
            self.callTestHook(job, .before_set_barrier);

            // Set the barrier on the job, this returns true if the barrier was successfully set (otherwise the job is already done and we don't need to add it to our list)
            if (job.setBarrier(&self.base)) {
                self.callTestHook(job, .after_set_barrier);

                // If the job can be executed we want to release the semaphore an extra time to allow the waiting thread to start executing it
                if (job.canBeExecuted()) {
                    release_semaphore = true;
                    _ = self.write_state.num_to_acquire.fetchAdd(1, .seq_cst);
                }

                // Add the job to our job list
                self.appendJob(job);
            } else {
                // Zolt: release the count of the job that is already done (see the top of this file)
                num_already_done += 1;
            }

            // Notify waiting thread that a new executable job is available
            self.releaseSemaphore(release_semaphore, num_already_done);
        }

        /// Add multiple jobs to this barrier
        fn addJobs(self: *BarrierImpl, handles: []const JobHandle) void {
            var release_semaphore = false;
            var num_already_done: u32 = 0;

            for (handles) |*handle| {
                // Zolt: count the job before setting the barrier on it, so that it cannot release the semaphore before it is counted (see the top of this file)
                const job = handle.getPtr().?;
                _ = self.write_state.num_to_acquire.fetchAdd(1, .seq_cst);
                self.callTestHook(job, .before_set_barrier);

                // Set the barrier on the job, this returns true if the barrier was successfully set (otherwise the job is already done and we don't need to add it to our list)
                if (job.setBarrier(&self.base)) {
                    self.callTestHook(job, .after_set_barrier);

                    // If the job can be executed we want to release the semaphore an extra time to allow the waiting thread to start executing it
                    if (!release_semaphore and job.canBeExecuted()) {
                        release_semaphore = true;
                        _ = self.write_state.num_to_acquire.fetchAdd(1, .seq_cst);
                    }

                    // Add the job to our job list
                    self.appendJob(job);
                } else {
                    // Zolt: release the count of the job that is already done (see the top of this file)
                    num_already_done += 1;
                }
            }

            // Notify waiting thread that a new executable job is available
            self.releaseSemaphore(release_semaphore, num_already_done);
        }

        /// Zolt: release the semaphore once for a new executable job (Jolt) and once per job that was already done
        fn releaseSemaphore(self: *BarrierImpl, release_semaphore: bool, num_already_done: u32) void {
            const num_to_release = @as(u32, @intFromBool(release_semaphore)) + num_already_done;
            if (num_to_release > 0)
                self.semaphore.release(self.io, .{ .number = num_to_release });
        }

        /// Zolt: call the test hook (test builds only)
        fn callTestHook(self: *BarrierImpl, job: *Job, point: TestHookPoint) void {
            if (builtin.is_test) {
                if (self.test_hook) |hook|
                    hook.function(hook.context, job, point);
            }
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

    // A job that already finished is not added to the barrier (Zolt: its count is released at once, the wait acquires it)
    const barrier = job_system.createBarrier().?;
    const barrier_impl = JobSystemWithBarrier.BarrierImpl.fromBarrier(barrier);
    {
        var handle = try job_system.createJob("Done", Color.blue, .init(Context.increment, .{&value}), .{});
        defer handle.deinit();
        try std.testing.expectEqual(Job.done_state, handle.getPtr().?.execute());
        barrier.addJob(&handle);
        try std.testing.expect(barrier_impl.isEmpty());
        try std.testing.expectEqual(@as(i32, 1), barrier_impl.write_state.num_to_acquire.load(.seq_cst));
        try std.testing.expectEqual(@as(i32, 1), barrier_impl.semaphore.getValue());
        barrier.addJobs((&handle)[0..1]);
        try std.testing.expect(barrier_impl.isEmpty());
        try std.testing.expectEqual(@as(i32, 2), barrier_impl.write_state.num_to_acquire.load(.seq_cst));
        try std.testing.expectEqual(@as(i32, 2), barrier_impl.semaphore.getValue());
        job_system.waitForJobs(barrier);
        try std.testing.expectEqual(@as(i32, 0), barrier_impl.write_state.num_to_acquire.load(.seq_cst));
        try std.testing.expectEqual(@as(i32, 0), barrier_impl.semaphore.getValue());
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

/// A thread that waits on a barrier for the tests below, `returned` is set when wait() has returned
const Waiter = struct {
    job_system: JobSystem,
    barrier: *Barrier,
    returned: std.atomic.Value(bool) = .init(false),

    fn run(self: *Waiter) void {
        self.job_system.waitForJobs(self.barrier);
        self.returned.store(true, .seq_cst);
    }

    /// Wait until the waiter is blocked on the semaphore of the barrier (only wait() acquires it, which makes its value
    /// negative until the next release) or until it has returned
    fn waitUntilBlockedOrReturned(self: *Waiter) void {
        const barrier_impl = JobSystemWithBarrier.BarrierImpl.fromBarrier(self.barrier);
        while (barrier_impl.semaphore.getValue() >= 0 and !self.returned.load(.seq_cst))
            std.Thread.yield() catch {};
    }
};

test "JobSystemWithBarrier a job that finishes while it is added cannot end the wait" {
    if (builtin.single_threaded) return error.SkipZigTest;

    // A prepare job of the barrier adds a collide job that has no dependency, like JobSoftBodyPrepare of PhysicsSystem.
    // The test hook finishes the collide job right after the barrier is set on it (where Jolt has not counted it yet,
    // see the top of this file) and lets the waiting thread run until it blocks again. In Jolt's order the waiter would
    // spend the release of the collide job on the count of the prepare job and return while the prepare job runs.
    for ([_]bool{ false, true }) |use_add_jobs| {
        var test_system: TestJobSystem = try .init(std.testing.allocator, std.testing.io, 16, 1);
        defer test_system.deinit();
        const job_system = test_system.jobSystem();
        const barrier = job_system.createBarrier().?;
        const barrier_impl = JobSystemWithBarrier.BarrierImpl.fromBarrier(barrier);
        var waiter: Waiter = .{ .job_system = job_system, .barrier = barrier };

        const Context = struct {
            job_system: JobSystem,
            barrier: *Barrier,
            waiter: *Waiter,
            use_add_jobs: bool,
            num_collided: u32 = 0,
            collide_state: u32 = 0,
            waiter_returned_early: bool = false,

            fn prepare(self: *@This()) void {
                var collide_job = self.job_system.createJob("SoftBodyCollide", Color.green, .init(collide, .{self}), .{}) catch @panic("out of memory"); // depends on: nothing
                defer collide_job.deinit();
                if (self.use_add_jobs)
                    self.barrier.addJobs((&collide_job)[0..1])
                else
                    self.barrier.addJob(&collide_job);
            }

            fn collide(self: *@This()) void {
                self.num_collided += 1;
            }

            fn hook(context: *anyopaque, job: *Job, point: JobSystemWithBarrier.BarrierImpl.TestHookPoint) void {
                const self: *@This() = @ptrCast(@alignCast(context));
                if (point != .after_set_barrier)
                    return;

                // Finish the job (like a worker thread would) and let the waiter acquire its release
                self.collide_state = job.execute();
                self.waiter.waitUntilBlockedOrReturned();
                self.waiter_returned_early = self.waiter.returned.load(.seq_cst);
            }
        };
        var context: Context = .{ .job_system = job_system, .barrier = barrier, .waiter = &waiter, .use_add_jobs = use_add_jobs };

        // The prepare job is in the barrier before the wait starts, its dependency keeps the waiter from executing it
        var prepare = try job_system.createJob("SoftBodyPrepare", Color.green, .init(Context.prepare, .{&context}), .{ .num_dependencies = 1 });
        defer prepare.deinit();
        barrier.addJob(&prepare);
        barrier_impl.test_hook = .{ .context = &context, .function = Context.hook };

        // Run the prepare job on this thread (like a worker thread) once the waiter is blocked
        const thread = try std.Thread.spawn(.{}, Waiter.run, .{&waiter});
        waiter.waitUntilBlockedOrReturned();
        const waiter_returned_before_prepare = waiter.returned.load(.seq_cst);
        prepare.removeDependency(.{}); // TestJobSystem does not queue it
        const prepare_state = prepare.getPtr().?.execute();
        thread.join();
        barrier_impl.test_hook = null;

        try std.testing.expect(!waiter_returned_before_prepare);
        try std.testing.expectEqual(Job.done_state, prepare_state);
        try std.testing.expectEqual(Job.done_state, context.collide_state);
        try std.testing.expectEqual(@as(u32, 1), context.num_collided);
        try std.testing.expect(!context.waiter_returned_early);
        try std.testing.expect(barrier_impl.isEmpty());
        try std.testing.expectEqual(@as(i32, 0), barrier_impl.write_state.num_to_acquire.load(.seq_cst));
        try std.testing.expectEqual(@as(i32, 0), barrier_impl.semaphore.getValue());
        job_system.destroyBarrier(barrier);
    }
}

test "JobSystemWithBarrier a job that is done before the barrier is set on it wakes the waiter" {
    if (builtin.single_threaded) return error.SkipZigTest;

    // The adding thread is not a job of the barrier. The test hook finishes the added job before the barrier is set on
    // it, then finishes the other job of the barrier and lets the waiting thread run until it blocks on the count of the
    // added job. setBarrier then fails: taking the count back would leave the waiter blocked, releasing it wakes the
    // waiter (see the top of this file).
    for ([_]bool{ false, true }) |use_add_jobs| {
        var test_system: TestJobSystem = try .init(std.testing.allocator, std.testing.io, 16, 1);
        defer test_system.deinit();
        const job_system = test_system.jobSystem();
        const barrier = job_system.createBarrier().?;
        const barrier_impl = JobSystemWithBarrier.BarrierImpl.fromBarrier(barrier);
        var waiter: Waiter = .{ .job_system = job_system, .barrier = barrier };

        const Context = struct {
            waiter: *Waiter,
            pending: JobHandle = .empty,
            num_executed: u32 = 0,
            added_state: u32 = 0,
            pending_state: u32 = 0,
            waiter_returned_early: bool = false,

            fn run(self: *@This()) void {
                self.num_executed += 1;
            }

            fn hook(context: *anyopaque, job: *Job, point: JobSystemWithBarrier.BarrierImpl.TestHookPoint) void {
                const self: *@This() = @ptrCast(@alignCast(context));
                if (point != .before_set_barrier)
                    return;

                // Finish the added job and the job of the barrier (like worker threads would), then let the waiter
                // acquire the release of the job of the barrier
                self.added_state = job.execute();
                self.pending.removeDependency(.{}); // TestJobSystem does not queue it
                self.pending_state = self.pending.getPtr().?.execute();
                self.waiter.waitUntilBlockedOrReturned();
                self.waiter_returned_early = self.waiter.returned.load(.seq_cst);
            }
        };
        var context: Context = .{ .waiter = &waiter };

        // The job of the barrier, its dependency keeps the waiter from executing it
        context.pending = try job_system.createJob("Pending", Color.red, .init(Context.run, .{&context}), .{ .num_dependencies = 1 });
        defer context.pending.deinit();
        barrier.addJob(&context.pending);
        var added = try job_system.createJob("Added", Color.red, .init(Context.run, .{&context}), .{});
        defer added.deinit();

        // Add the job from this thread once the waiter is blocked
        const thread = try std.Thread.spawn(.{}, Waiter.run, .{&waiter});
        waiter.waitUntilBlockedOrReturned();
        const waiter_returned_before_add = waiter.returned.load(.seq_cst);
        barrier_impl.test_hook = .{ .context = &context, .function = Context.hook };
        if (use_add_jobs)
            barrier.addJobs((&added)[0..1])
        else
            barrier.addJob(&added);
        barrier_impl.test_hook = null;

        // No job is left, so a negative semaphore value means that the waiter stays blocked: release it to end the test
        const waiter_left_blocked = barrier_impl.semaphore.getValue() < 0;
        if (waiter_left_blocked)
            barrier_impl.semaphore.release(std.testing.io, .{});
        thread.join();

        try std.testing.expect(!waiter_returned_before_add);
        try std.testing.expect(!waiter_left_blocked);
        try std.testing.expect(!context.waiter_returned_early);
        try std.testing.expectEqual(Job.done_state, context.added_state);
        try std.testing.expectEqual(Job.done_state, context.pending_state);
        try std.testing.expectEqual(@as(u32, 2), context.num_executed);
        try std.testing.expect(barrier_impl.isEmpty());
        try std.testing.expectEqual(@as(i32, 0), barrier_impl.write_state.num_to_acquire.load(.seq_cst));
        try std.testing.expectEqual(@as(i32, 0), barrier_impl.semaphore.getValue());
        job_system.destroyBarrier(barrier);
    }
}

test "JobSystemWithBarrier the soft body jobs of PhysicsSystem on a thread pool" {
    if (builtin.single_threaded) return error.SkipZigTest;

    const JobSystemThreadPool = @import("JobSystemThreadPool.zig").JobSystemThreadPool;

    // The job graph of the soft body step (PhysicsSystem.jobSoftBodyPrepare) on a thread pool. A prepare job of the
    // barrier runs on a worker thread and creates and adds the finalize, simulate and collide jobs. The collide jobs
    // have no dependency, so worker threads run them while they are being added. The test hook forces the
    // interleaving of the race (see the top of this file):
    // - a collide job is running on a worker before it is added (so that it does not take the extra count of an
    //   executable job) and finishes only after the barrier is set on it,
    // - after the barrier is set on the last collide job, every job except the prepare job finishes, then the waiting
    //   thread runs until it blocks. In Jolt's order that is the moment in which the waiter returns while the prepare
    //   job is still in addJobs.
    // The step lives in heap memory that is freed after the wait, like the temp memory of an update.
    const num_iterations = 200;
    const num_soft_body_jobs = 4;
    var pool: JobSystemThreadPool = .empty;
    try pool.init(std.testing.allocator, std.testing.io, 1024, 1, .{ .num_threads = num_soft_body_jobs - 1 });
    defer pool.deinit();
    const job_system = pool.jobSystem();
    try std.testing.expectEqual(@as(i32, num_soft_body_jobs), job_system.getMaxConcurrency());

    const Step = struct {
        job_system: JobSystem,
        barrier: *Barrier,
        prepare: JobHandle = .empty,
        finalize: JobHandle = .empty,
        simulate: [num_soft_body_jobs]JobHandle = @splat(.empty),
        collide: [num_soft_body_jobs]JobHandle = @splat(.empty),
        prepare_started: std.atomic.Value(bool) = .init(false),
        collide_barrier_set: [num_soft_body_jobs]std.atomic.Value(bool) = @splat(.init(false)),
        wait_returned: std.atomic.Value(bool) = .init(false),
        num_collided: std.atomic.Value(u32) = .init(0),
        num_simulated: std.atomic.Value(u32) = .init(0),
        num_finalized: std.atomic.Value(u32) = .init(0),

        fn deinit(self: *@This()) void {
            self.prepare.deinit();
            self.finalize.deinit();
            for (&self.simulate) |*h| h.deinit();
            for (&self.collide) |*h| h.deinit();
        }

        fn jobPrepare(self: *@This()) void {
            self.prepare_started.store(true, .seq_cst);

            // Create finalize job
            self.finalize = self.job_system.createJob("SoftBodyFinalize", Color.red, .init(jobFinalize, .{self}), .{ .num_dependencies = num_soft_body_jobs }) catch @panic("out of memory"); // depends on: soft body simulate
            self.barrier.addJob(&self.finalize);

            // Create simulate jobs
            for (&self.simulate) |*h|
                h.* = self.job_system.createJob("SoftBodySimulate", Color.green, .init(jobSimulate, .{self}), .{ .num_dependencies = num_soft_body_jobs }) catch @panic("out of memory"); // depends on: soft body collide
            self.barrier.addJobs(&self.simulate);

            // Create collision jobs
            for (&self.collide, 0..) |*h, i|
                h.* = self.job_system.createJob("SoftBodyCollide", Color.blue, .init(jobCollide, .{ self, i }), .{}) catch @panic("out of memory"); // depends on: nothing
            self.barrier.addJobs(&self.collide);
        }

        fn jobCollide(self: *@This(), index: usize) void {
            while (!self.collide_barrier_set[index].load(.seq_cst))
                std.Thread.yield() catch {};
            _ = self.num_collided.fetchAdd(1, .monotonic);
            for (&self.simulate) |*h|
                h.removeDependency(.{});
        }

        fn jobSimulate(self: *@This()) void {
            _ = self.num_simulated.fetchAdd(1, .monotonic);
            self.finalize.removeDependency(.{});
        }

        fn jobFinalize(self: *@This()) void {
            _ = self.num_finalized.fetchAdd(1, .monotonic);
        }

        fn hook(context: *anyopaque, job: *Job, point: JobSystemWithBarrier.BarrierImpl.TestHookPoint) void {
            const self: *@This() = @ptrCast(@alignCast(context));
            const index = for (&self.collide, 0..) |*h, i| {
                if (h.getPtr() == job) break i;
            } else return; // Not a collide job
            switch (point) {
                .before_set_barrier => {
                    // Wait until a worker thread runs the collide job
                    while (job.canBeExecuted())
                        std.Thread.yield() catch {};
                },
                .after_set_barrier => {
                    // Let the collide job finish
                    self.collide_barrier_set[index].store(true, .seq_cst);
                    if (index == num_soft_body_jobs - 1) {
                        // Wait until every job except the prepare job has finished and the waiter is blocked or has returned
                        while (!self.finalize.isDone())
                            std.Thread.yield() catch {};
                        const barrier_impl = JobSystemWithBarrier.BarrierImpl.fromBarrier(self.barrier);
                        while (barrier_impl.semaphore.getValue() >= 0 and !self.wait_returned.load(.seq_cst))
                            std.Thread.yield() catch {};
                    }
                },
            }
        }
    };

    for (0..num_iterations) |_| {
        const barrier = job_system.createBarrier().?;
        const barrier_impl = JobSystemWithBarrier.BarrierImpl.fromBarrier(barrier);
        const step = try std.testing.allocator.create(Step);
        defer std.testing.allocator.destroy(step);
        step.* = .{ .job_system = job_system, .barrier = barrier };
        defer step.deinit();
        barrier_impl.test_hook = .{ .context = step, .function = Step.hook };

        // The prepare job is in the barrier before its dependency (Jolt: solve position constraints) is removed. Wait
        // until a worker thread runs it, so that the waiting thread does not execute it.
        step.prepare = try job_system.createJob("SoftBodyPrepare", Color.yellow, .init(Step.jobPrepare, .{step}), .{ .num_dependencies = 1 });
        barrier.addJob(&step.prepare);
        step.prepare.removeDependency(.{});
        while (!step.prepare_started.load(.seq_cst))
            std.Thread.yield() catch {};
        job_system.waitForJobs(barrier);
        step.wait_returned.store(true, .seq_cst);
        barrier_impl.test_hook = null;
        job_system.destroyBarrier(barrier);

        try std.testing.expect(step.prepare.isDone());
        try std.testing.expect(step.finalize.isDone());
        try std.testing.expectEqual(@as(u32, num_soft_body_jobs), step.num_collided.load(.monotonic));
        try std.testing.expectEqual(@as(u32, num_soft_body_jobs), step.num_simulated.load(.monotonic));
        try std.testing.expectEqual(@as(u32, 1), step.num_finalized.load(.monotonic));
    }
}
