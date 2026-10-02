//! Port of: Jolt/Core/JobSystemSingleThreaded.h, Jolt/Core/JobSystemSingleThreaded.cpp
//! Status: complete
//!
//! Differences with the C++ version:
//! - `JobSystemSingleThreaded()` + `Init(inMaxJobs)` / `JobSystemSingleThreaded(inMaxJobs)` ->
//!   `init(allocator, io, max_jobs)` (`empty` is the default constructed state), the destructor -> `deinit()`.
//!   The io is needed by the job free list (it locks a mutex when it allocates its page). `max_jobs` is the page size
//!   of the job free list, so it must be a power of 2 (as in Jolt).
//! - The JobSystem interface is `job_system.jobSystem()`, the jobs keep a pointer to it so the job system must not move
//!   while jobs exist.
//! - `createJob` returns `error.OutOfMemory` when the job storage cannot be allocated (Jolt does not check).

const std = @import("std");
const Color = @import("Color.zig").Color;
const FixedSizeFreeList = @import("FixedSizeFreeList.zig").FixedSizeFreeList;
const JobSystem = @import("JobSystem.zig").JobSystem;

const Job = JobSystem.Job;
const JobFunction = JobSystem.JobFunction;
const JobHandle = JobSystem.JobHandle;
const Barrier = JobSystem.Barrier;

/// Implementation of a JobSystem without threads, runs jobs as soon as they are added
pub const JobSystemSingleThreaded = struct {
    /// Shared barrier since the barrier implementation does nothing
    dummy_barrier: BarrierImpl = .{},

    /// Array of jobs (fixed size)
    jobs: AvailableJobs = .empty,

    /// Array of jobs (fixed size)
    const AvailableJobs = FixedSizeFreeList(Job);

    /// Not initialized (default constructor), deinit does nothing
    pub const empty: JobSystemSingleThreaded = .{};

    /// Initialize the job system
    /// max_jobs: Max number of jobs that can be allocated at any time (a power of 2)
    pub fn init(allocator: std.mem.Allocator, io: std.Io, max_jobs: u32) error{OutOfMemory}!JobSystemSingleThreaded {
        return .{ .jobs = try .init(allocator, io, max_jobs, max_jobs) };
    }

    /// Destructor, all jobs must have been released
    pub fn deinit(self: *JobSystemSingleThreaded) void {
        self.jobs.deinit();
        self.* = .empty;
    }

    /// The JobSystem interface of this job system
    pub fn jobSystem(self: *JobSystemSingleThreaded) JobSystem {
        return .init(self);
    }

    /// Get maximum number of concurrently executing jobs (see JobSystem)
    pub fn getMaxConcurrency(self: *const JobSystemSingleThreaded) i32 {
        _ = self;
        return 1;
    }

    /// Create a new job, it is executed immediately when it has no dependencies (see JobSystem)
    pub fn createJob(self: *JobSystemSingleThreaded, job_name: []const u8, color: Color, job_function: JobFunction, opts: struct { num_dependencies: u32 = 0 }) error{OutOfMemory}!JobHandle {
        // Construct an object
        const index = try self.jobs.constructObject(.init(job_name, color, self.jobSystem(), job_function, opts.num_dependencies));
        std.debug.assert(index != AvailableJobs.invalid_object_index);
        const job = self.jobs.get(index);

        // Construct handle to keep a reference, the job is queued below and will immediately complete
        const handle: JobHandle = .init(job);

        // If there are no dependencies, queue the job now
        if (opts.num_dependencies == 0)
            self.queueJob(job);

        // Return the handle
        return handle;
    }

    /// Create a new barrier (all jobs are executed immediately, so this is a shared dummy barrier)
    pub fn createBarrier(self: *JobSystemSingleThreaded) ?*Barrier {
        return &self.dummy_barrier.base;
    }

    /// Destroy a barrier
    pub fn destroyBarrier(self: *JobSystemSingleThreaded, barrier: *Barrier) void {
        // There's nothing to do here, the barrier is just a dummy
        _ = self;
        _ = barrier;
    }

    /// Wait for a set of jobs to be finished
    pub fn waitForJobs(self: *JobSystemSingleThreaded, barrier: *Barrier) void {
        // There's nothing to do here, the barrier is just a dummy, we just execute the jobs immediately
        _ = self;
        _ = barrier;
    }

    /// Adds a job to the job queue, executes it immediately (protected in C++, see JobSystem)
    pub fn queueJob(self: *JobSystemSingleThreaded, job: *Job) void {
        _ = self;
        _ = job.execute();
    }

    /// Adds a number of jobs at once to the job queue, executes them immediately (protected in C++, see JobSystem)
    pub fn queueJobs(self: *JobSystemSingleThreaded, jobs: []const *Job) void {
        for (jobs) |job|
            self.queueJob(job);
    }

    /// Frees a job (protected in C++, see JobSystem)
    pub fn freeJob(self: *JobSystemSingleThreaded, job: *Job) void {
        self.jobs.destructObjectPtr(job);
    }

    /// Dummy implementation of Barrier, all jobs are executed immediately
    const BarrierImpl = struct {
        /// The Barrier interface
        base: Barrier = .{ .vtable = &vtable },

        const vtable: Barrier.VTable = .{
            .addJob = addJob,
            .addJobs = addJobs,
            .onJobFinished = onJobFinished,
        };

        fn addJob(barrier: *Barrier, job: *const JobHandle) void {
            // We don't need to track jobs
            _ = barrier;
            _ = job;
        }

        fn addJobs(barrier: *Barrier, handles: []const JobHandle) void {
            // We don't need to track jobs
            _ = barrier;
            _ = handles;
        }

        /// Called by a Job to mark that it is finished
        fn onJobFinished(barrier: *Barrier, job: *Job) void {
            // We don't need to track jobs
            _ = barrier;
            _ = job;
        }
    };
};

test "JobSystemSingleThreaded executes jobs immediately" {
    var single_threaded: JobSystemSingleThreaded = try .init(std.testing.allocator, std.testing.io, 16);
    defer single_threaded.deinit();
    const job_system = single_threaded.jobSystem();
    try std.testing.expectEqual(@as(i32, 1), job_system.getMaxConcurrency());

    const Context = struct {
        fn increment(value: *u32) void {
            value.* += 1;
        }
    };
    var value: u32 = 0;

    // A job without dependencies has executed when createJob returns
    var handle = try job_system.createJob("Immediate", Color.red, .init(Context.increment, .{&value}), .{});
    try std.testing.expect(handle.isDone());
    try std.testing.expectEqual(@as(u32, 1), value);
    handle.deinit();

    // A job with dependencies executes when the last dependency is removed
    var dependent = try job_system.createJob("Dependent", Color.green, .init(Context.increment, .{&value}), .{ .num_dependencies = 2 });
    defer dependent.deinit();
    dependent.removeDependency(.{});
    try std.testing.expect(!dependent.isDone());
    try std.testing.expectEqual(@as(u32, 1), value);
    dependent.removeDependency(.{});
    try std.testing.expect(dependent.isDone());
    try std.testing.expectEqual(@as(u32, 2), value);

    // The barrier is a dummy
    const barrier = job_system.createBarrier().?;
    try std.testing.expectEqual(barrier, job_system.createBarrier().?);
    barrier.addJob(&dependent);
    barrier.addJobs((&dependent)[0..1]);
    job_system.waitForJobs(barrier);
    job_system.destroyBarrier(barrier);
    try std.testing.expectEqual(@as(u32, 2), value);

    // Jobs are freed and reused, more jobs than max_jobs can be created over time
    for (0..100) |_| {
        var h = try job_system.createJob("Reuse", Color.blue, .init(Context.increment, .{&value}), .{});
        h.deinit();
    }
    try std.testing.expectEqual(@as(u32, 102), value);

    var not_initialized: JobSystemSingleThreaded = .empty;
    not_initialized.deinit();
}

test "JobSystemSingleThreaded job dependencies" {
    const num_jobs = 32;
    var single_threaded: JobSystemSingleThreaded = try .init(std.testing.allocator, std.testing.io, 2 * num_jobs);
    defer single_threaded.deinit();
    const job_system = single_threaded.jobSystem();

    // A chain that is started from the last job (like TestJobSystemRunChain), runs in reverse order on the calling thread
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

    const barrier = job_system.createBarrier().?;
    for (&context.handles, 0..) |*handle, i| {
        handle.* = try job_system.createJob("Chain", Color.red, .init(Context.run, .{ &context, i }), .{ .num_dependencies = 1 });
        barrier.addJob(handle);
    }
    context.handles[num_jobs - 1].removeDependency(.{});
    job_system.waitForJobs(barrier);
    job_system.destroyBarrier(barrier);
    for (context.values, 0..) |value, i|
        try std.testing.expectEqual(@as(u32, @intCast(num_jobs - i)), value);

    // Removing dependencies from a batch of jobs executes them in order
    var order: [4]u32 = @splat(0);
    var num_executed: u32 = 0;
    const Record = struct {
        fn run(o: *[4]u32, n: *u32, index: u32) void {
            o[n.*] = index;
            n.* += 1;
        }
    };
    var batch: [4]JobHandle = undefined;
    for (&batch, 0..) |*handle, i|
        handle.* = try job_system.createJob("Batch", Color.green, .init(Record.run, .{ &order, &num_executed, @as(u32, @intCast(i)) }), .{ .num_dependencies = 1 });
    defer for (&batch) |*handle| handle.deinit();
    JobHandle.removeDependencies(&batch, .{});
    try std.testing.expectEqual([4]u32{ 0, 1, 2, 3 }, order);
}
