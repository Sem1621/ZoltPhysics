//! Port of: Jolt/Core/JobSystemThreadPool.h, Jolt/Core/JobSystemThreadPool.cpp
//! Status: complete
//!
//! Differences with the C++ version:
//! - `JobSystemThreadPool()` + `Init(inMaxJobs, inMaxBarriers, inNumThreads)` (or the constructor with these arguments)
//!   -> `var pool: JobSystemThreadPool = .empty;` + `try pool.init(allocator, io, max_jobs, max_barriers,
//!   .{ .num_threads = n })`, the destructor -> `pool.deinit()`. `init` works in place because the worker threads and
//!   the jobs keep a pointer to the pool, so the pool must not be moved or copied after `init`. Like the C++ `Init`
//!   it expects a default constructed pool (`.empty`, optionally with thread init/exit functions set).
//!   `max_jobs` is the page size of the job free list, so it must be a power of 2 (as in Jolt).
//! - The JobSystem interface is `pool.jobSystem()`, the functions of the pool can also be called directly.
//! - Workers are `std.Thread`s. Blocking (the semaphore, sleeping) uses the `io` given to `init`. `init` and
//!   `setNumThreads` return the errors of allocating memory and spawning threads (Jolt doesn't check); when spawning a
//!   thread fails, the threads that were started are stopped again.
//! - `thread::hardware_concurrency()` -> `std.Thread.getCpuCount()`, which is taken as 1 when it fails, so that
//!   `num_threads = -1` then starts no worker threads (Jolt would compute a negative number of threads).
//! - Naming the worker threads ("Worker %d", SetThreadName), FPExceptionsEnable (JPH_FLOATING_POINT_EXCEPTIONS_ENABLED
//!   is not ported) and the profiler hooks (JPH_PROFILE_THREAD_START/END) are no-ops.
//! - `InitExitFunction` (std::function<void(int)>) is a closure like JobSystem.JobFunction:
//!   `InitExitFunction.init(function, .{ args... })` calls `function(args..., thread_index)`.
//! - "No jobs available!" (all jobs are in use) panics when asserts are enabled (Core.enable_asserts, see the porting
//!   guide for JPH_ASSERT(false) on paths that release builds handle), release builds sleep and retry like Jolt.
//! - Jolt aligns mTail to a cache line. Zig reorders struct fields, so it is in a cache line aligned struct that
//!   occupies whole cache lines: `tail_state.tail`.
//! - A single threaded build (builtin.single_threaded) starts no worker threads, the jobs are then executed by the
//!   barriers like with `num_threads = 0`.

const std = @import("std");
const builtin = @import("builtin");
const Core = @import("Core.zig");
const Color = @import("Color.zig").Color;
const FixedSizeFreeList = @import("FixedSizeFreeList.zig").FixedSizeFreeList;
const JobSystem = @import("JobSystem.zig").JobSystem;
const JobSystemWithBarrier = @import("JobSystemWithBarrier.zig").JobSystemWithBarrier;
const Semaphore = @import("Semaphore.zig").Semaphore;
const math = @import("../Math/Math.zig");

const Job = JobSystem.Job;
const JobFunction = JobSystem.JobFunction;
const JobHandle = JobSystem.JobHandle;
const Barrier = JobSystem.Barrier;

/// Implementation of a JobSystem using a thread pool
///
/// Note that this is considered an example implementation. It is expected that when you integrate
/// the physics engine into your own project that you'll provide your own implementation of the
/// JobSystem built on top of whatever job system your project uses.
pub const JobSystemThreadPool = struct {
    /// Base class
    base: JobSystemWithBarrier = .empty,

    /// Allocator for the jobs, the thread list and the heads
    allocator: std.mem.Allocator = undefined,

    /// Io used to wait for and wake up the worker threads
    io: std.Io = undefined,

    /// Functions to call when initializing or exiting a thread
    thread_init_function: InitExitFunction = .noop,
    thread_exit_function: InitExitFunction = .noop,

    /// Array of jobs (fixed size)
    jobs: AvailableJobs = .empty,

    /// Threads running jobs
    threads: std.ArrayList(std.Thread) = .empty,

    // The job queue
    queue: [queue_length]std.atomic.Value(?*Job) = @splat(.init(null)),

    // Head and tail of the queue, do this value modulo queue_length - 1 to get the element in the queue array

    /// Per executing thread the head of the current queue
    heads: []std.atomic.Value(u32) = &.{},

    /// Tail (write end) of the queue, on its own cache line (see the top of this file)
    tail_state: TailState = .{},

    /// Semaphore used to signal worker threads that there is new work
    semaphore: Semaphore = .{},

    /// Boolean to indicate that we want to stop the job system
    quit: std.atomic.Value(bool) = .init(false),

    /// Array of jobs (fixed size)
    const AvailableJobs = FixedSizeFreeList(Job);

    // The job queue
    const queue_length: u32 = 1024;
    comptime {
        std.debug.assert(math.isPowerOf2(queue_length)); // We do bit operations and require queue length to be a power of 2
    }

    const TailState = struct {
        /// Tail (write end) of the queue
        tail: std.atomic.Value(u32) align(Core.cache_line_size) = .init(0),
    };

    comptime {
        // The tail must not share a cache line with any other member
        std.debug.assert(@alignOf(TailState) == Core.cache_line_size and @sizeOf(TailState) % Core.cache_line_size == 0);
    }

    /// Errors of init and setNumThreads
    pub const Error = error{OutOfMemory} || std.Thread.SpawnError;

    /// Functions to call when a thread is initialized or exits, must be set before calling init() (function<void(int)>)
    ///
    /// A closure like JobSystem.JobFunction: `InitExitFunction.init(function, .{ args... })` calls
    /// `function(args..., thread_index)`, the arguments are copied into the closure.
    pub const InitExitFunction = struct {
        /// Maximum size of the captured arguments in bytes
        pub const max_capture_size = JobFunction.max_capture_size;

        /// Maximum alignment of the captured arguments
        pub const max_capture_alignment = JobFunction.max_capture_alignment;

        /// Storage for the captured arguments
        const Captures = [max_capture_size]u8;

        /// Calls the function with the arguments stored in `captures` and the thread index
        invoke: *const fn (captures: *const Captures, thread_index: i32) void,

        /// Copy of the argument tuple (the bytes after it are undefined)
        captures: Captures align(max_capture_alignment),

        /// The default function, does nothing ([](int) { })
        pub const noop: InitExitFunction = .init(noopFunction, .{});

        /// Create a function that calls `function` with the arguments in the tuple `args` followed by the thread index
        pub fn init(comptime function: anytype, args: anytype) InitExitFunction {
            const Args = @TypeOf(args);
            comptime {
                if (@sizeOf(Args) > max_capture_size)
                    @compileError("InitExitFunction: the arguments " ++ @typeName(Args) ++ " don't fit in max_capture_size, pass a pointer to a struct instead");
                if (@alignOf(Args) > max_capture_alignment)
                    @compileError("InitExitFunction: the arguments " ++ @typeName(Args) ++ " are aligned more than max_capture_alignment, pass them by pointer");
            }
            const gen = struct {
                fn invoke(captures: *const Captures, thread_index: i32) void {
                    const captured: *const Args = @ptrCast(@alignCast(captures));
                    @call(.auto, function, captured.* ++ .{thread_index});
                }
            };
            var result: InitExitFunction = .{ .invoke = gen.invoke, .captures = undefined };
            if (@sizeOf(Args) > 0) // Also allows creating a function without captures at comptime
                @memcpy(result.captures[0..@sizeOf(Args)], std.mem.asBytes(&args));
            return result;
        }

        /// Call the function (operator ())
        pub fn call(self: *const InitExitFunction, thread_index: i32) void {
            self.invoke(&self.captures, thread_index);
        }

        fn noopFunction(thread_index: i32) void {
            _ = thread_index;
        }
    };

    /// Default constructed thread pool, call init to start it
    pub const empty: JobSystemThreadPool = .{};

    /// Set the function to call when a thread is initialized, must be set before calling init()
    pub fn setThreadInitFunction(self: *JobSystemThreadPool, init_function: InitExitFunction) void {
        self.thread_init_function = init_function;
    }

    /// Set the function to call when a thread exits, must be set before calling init()
    pub fn setThreadExitFunction(self: *JobSystemThreadPool, exit_function: InitExitFunction) void {
        self.thread_exit_function = exit_function;
    }

    /// Initialize the thread pool, `self` must be default constructed (`.empty`) and must not move afterwards
    /// max_jobs: Max number of jobs that can be allocated at any time (a power of 2)
    /// max_barriers: Max number of barriers that can be allocated at any time
    /// opts.num_threads: Number of threads to start (the number of concurrent jobs is 1 more because the main thread will also run jobs while waiting for a barrier to complete). Use -1 to auto detect the amount of CPU's.
    pub fn init(self: *JobSystemThreadPool, allocator: std.mem.Allocator, io: std.Io, max_jobs: u32, max_barriers: u32, opts: struct { num_threads: i32 = -1 }) Error!void {
        std.debug.assert(self.base.barriers == null); // Already initialized?
        self.allocator = allocator;
        self.io = io;

        self.base = try .init(allocator, io, max_barriers);
        errdefer self.base.deinit();

        // Init freelist of jobs
        self.jobs = try .init(allocator, io, max_jobs, max_jobs);
        errdefer self.jobs.deinit();

        // Init queue
        for (&self.queue) |*j|
            j.store(null, .seq_cst);

        // Start the worker threads
        errdefer {
            self.threads.deinit(allocator);
            self.threads = .empty;
        }
        try self.startThreads(opts.num_threads);
    }

    /// Destructor, stops all worker threads. All jobs and barriers must have been released.
    pub fn deinit(self: *JobSystemThreadPool) void {
        // Stop all worker threads
        self.stopThreads();

        self.threads.deinit(self.allocator);
        self.jobs.deinit();
        self.base.deinit();
        self.* = .empty;
    }

    /// The JobSystem interface of this thread pool
    pub fn jobSystem(self: *JobSystemThreadPool) JobSystem {
        return .init(self);
    }

    /// Get maximum number of concurrently executing jobs (see JobSystem)
    pub fn getMaxConcurrency(self: *const JobSystemThreadPool) i32 {
        return @as(i32, @intCast(self.threads.items.len)) + 1;
    }

    /// Create a new job (see JobSystem)
    pub fn createJob(self: *JobSystemThreadPool, job_name: []const u8, color: Color, job_function: JobFunction, opts: struct { num_dependencies: u32 = 0 }) error{OutOfMemory}!JobHandle {
        // Loop until we can get a job from the free list
        var index: u32 = undefined;
        while (true) {
            index = try self.jobs.constructObject(.init(job_name, color, self.jobSystem(), job_function, opts.num_dependencies));
            if (index != AvailableJobs.invalid_object_index)
                break;
            if (Core.enable_asserts) @panic("No jobs available!");
            sleepUncancelable(self.io, .fromMicroseconds(100));
        }
        const job = self.jobs.get(index);

        // Construct handle to keep a reference, the job is queued below and may immediately complete
        const handle: JobHandle = .init(job);

        // If there are no dependencies, queue the job now
        if (opts.num_dependencies == 0)
            self.queueJob(job);

        // Return the handle
        return handle;
    }

    /// Create a new barrier (see JobSystemWithBarrier)
    pub fn createBarrier(self: *JobSystemThreadPool) ?*Barrier {
        return self.base.createBarrier();
    }

    /// Destroy a barrier (see JobSystemWithBarrier)
    pub fn destroyBarrier(self: *JobSystemThreadPool, barrier: *Barrier) void {
        self.base.destroyBarrier(barrier);
    }

    /// Wait for a set of jobs to be finished (see JobSystemWithBarrier)
    pub fn waitForJobs(self: *JobSystemThreadPool, barrier: *Barrier) void {
        self.base.waitForJobs(barrier);
    }

    /// Change the max concurrency after initialization
    pub fn setNumThreads(self: *JobSystemThreadPool, num_threads: i32) Error!void {
        self.stopThreads();
        try self.startThreads(num_threads);
    }

    /// Adds a job to the job queue (protected in C++, see JobSystem)
    pub fn queueJob(self: *JobSystemThreadPool, job: *Job) void {
        // If we have no worker threads, we can't queue the job either. We assume in this case that the job will be added to a barrier and that the barrier will execute the job when it's Wait() function is called.
        if (self.threads.items.len == 0)
            return;

        // Queue the job
        self.queueJobInternal(job);

        // Wake up thread
        self.semaphore.release(self.io, .{});
    }

    /// Adds a number of jobs at once to the job queue (protected in C++, see JobSystem)
    pub fn queueJobs(self: *JobSystemThreadPool, jobs: []const *Job) void {
        std.debug.assert(jobs.len > 0);

        // If we have no worker threads, we can't queue the job either. We assume in this case that the job will be added to a barrier and that the barrier will execute the job when it's Wait() function is called.
        if (self.threads.items.len == 0)
            return;

        // Queue all jobs
        for (jobs) |job|
            self.queueJobInternal(job);

        // Wake up threads
        self.semaphore.release(self.io, .{ .number = @intCast(@min(jobs.len, self.threads.items.len)) });
    }

    /// Frees a job (protected in C++, see JobSystem)
    pub fn freeJob(self: *JobSystemThreadPool, job: *Job) void {
        self.jobs.destructObjectPtr(job);
    }

    /// Start the worker threads
    fn startThreads(self: *JobSystemThreadPool, num_threads_in: i32) Error!void {
        var num_threads = num_threads_in;

        // Auto detect number of threads
        if (num_threads < 0)
            num_threads = @as(i32, @intCast(std.Thread.getCpuCount() catch 1)) - 1;

        // Zolt: a single threaded build can't start threads, the barriers execute the jobs
        if (builtin.single_threaded)
            return;

        // If no threads are requested we're done
        if (num_threads == 0)
            return;

        // Don't quit the threads
        self.quit.store(false, .seq_cst);

        // Allocate heads
        const thread_count: usize = @intCast(num_threads);
        self.heads = try self.allocator.alloc(std.atomic.Value(u32), thread_count);
        for (self.heads) |*head|
            head.* = .init(0);
        errdefer if (self.threads.items.len == 0) {
            self.allocator.free(self.heads);
            self.heads = &.{};
        };

        // Start running threads
        std.debug.assert(self.threads.items.len == 0);
        try self.threads.ensureTotalCapacity(self.allocator, thread_count);
        for (0..thread_count) |i| {
            const thread = std.Thread.spawn(.{}, threadMain, .{ self, @as(i32, @intCast(i)) }) catch |err| {
                // Zolt: stop the threads that were already started
                self.stopThreads();
                return err;
            };
            self.threads.appendAssumeCapacity(thread);
        }
    }

    /// Stop the worker threads
    fn stopThreads(self: *JobSystemThreadPool) void {
        if (self.threads.items.len == 0)
            return;

        // Signal threads that we want to stop and wake them up
        self.quit.store(true, .seq_cst);
        self.semaphore.release(self.io, .{ .number = @intCast(self.threads.items.len) });

        // Wait for all threads to finish
        for (self.threads.items) |thread|
            thread.join();

        // Delete all threads
        self.threads.clearRetainingCapacity();

        // Ensure that there are no lingering jobs in the queue
        var head: u32 = 0;
        while (head != self.tail_state.tail.load(.seq_cst)) : (head +%= 1) {
            // Fetch job
            if (self.queue[head & (queue_length - 1)].swap(null, .seq_cst)) |job_ptr| {
                // And execute it
                _ = job_ptr.execute();
                job_ptr.release();
            }
        }

        // Destroy heads and reset tail
        self.allocator.free(self.heads);
        self.heads = &.{};
        self.tail_state.tail.store(0, .seq_cst);
    }

    /// Entry point for a thread
    fn threadMain(self: *JobSystemThreadPool, thread_index: i32) void {
        // Naming the thread ("Worker %d"), enabling floating point exceptions and JPH_PROFILE_THREAD_START are not ported (see the top of this file)

        // Call the thread init function
        self.thread_init_function.call(thread_index);

        const head = &self.heads[@intCast(thread_index)];

        while (!self.quit.load(.seq_cst)) {
            // Wait for jobs
            self.semaphore.acquire(self.io, .{});

            // Loop over the queue
            while (head.load(.seq_cst) != self.tail_state.tail.load(.seq_cst)) {
                // Exchange any job pointer we find with a nullptr
                const job = &self.queue[head.load(.seq_cst) & (queue_length - 1)];
                if (job.load(.seq_cst) != null) {
                    if (job.swap(null, .seq_cst)) |job_ptr| {
                        // And execute it
                        _ = job_ptr.execute();
                        job_ptr.release();
                    }
                }
                _ = head.fetchAdd(1, .seq_cst);
            }
        }

        // Call the thread exit function
        self.thread_exit_function.call(thread_index);
    }

    /// Get the head of the thread that has processed the least amount of jobs
    fn getHead(self: *const JobSystemThreadPool) u32 {
        // Find the minimal value across all threads
        var head = self.tail_state.tail.load(.seq_cst);
        for (self.heads[0..self.threads.items.len]) |*thread_head|
            head = @min(head, thread_head.load(.seq_cst));
        return head;
    }

    /// Internal helper function to queue a job
    fn queueJobInternal(self: *JobSystemThreadPool, job: *Job) void {
        // Add reference to job because we're adding the job to the queue
        job.addRef();

        // Need to read head first because otherwise the tail can already have passed the head
        // We read the head outside of the loop since it involves iterating over all threads and we only need to update
        // it if there's not enough space in the queue.
        var head = self.getHead();

        while (true) {
            // Check if there's space in the queue
            var old_value = self.tail_state.tail.load(.seq_cst);
            if (old_value -% head >= queue_length) {
                // We calculated the head outside of the loop, update head (and we also need to update tail to prevent it from passing head)
                head = self.getHead();
                old_value = self.tail_state.tail.load(.seq_cst);

                // Second check if there's space in the queue
                if (old_value -% head >= queue_length) {
                    // Wake up all threads in order to ensure that they can clear any nullptrs they may not have processed yet
                    self.semaphore.release(self.io, .{ .number = @intCast(self.threads.items.len) });

                    // Sleep a little (we have to wait for other threads to update their head pointer in order for us to be able to continue)
                    sleepUncancelable(self.io, .fromMicroseconds(100));
                    continue;
                }
            }

            // Write the job pointer if the slot is empty
            const success = self.queue[old_value & (queue_length - 1)].cmpxchgStrong(null, job, .seq_cst, .seq_cst) == null;

            // Regardless of who wrote the slot, we will update the tail (if the successful thread got scheduled out
            // after writing the pointer we still want to be able to continue)
            _ = self.tail_state.tail.cmpxchgStrong(old_value, old_value +% 1, .seq_cst, .seq_cst);

            // If we successfully added our job we're done
            if (success)
                break;
        }
    }
};

/// std::this_thread::sleep_for, not cancelable (see "Threading" in the porting guide)
fn sleepUncancelable(io: std.Io, duration: std.Io.Duration) void {
    const old_cancel_protection = io.swapCancelProtection(.blocked);
    defer _ = io.swapCancelProtection(old_cancel_protection);
    io.sleep(duration, .awake) catch |err| switch (err) {
        error.Canceled => unreachable, // Cancelation is blocked
    };
}

fn incrementCounter(counter: *std.atomic.Value(u32)) void {
    _ = counter.fetchAdd(1, .monotonic);
}

/// Run `num_jobs` jobs through a barrier, half of them are started by removing a dependency after adding them to the barrier
fn runJobsThroughBarrier(job_system: JobSystem, num_jobs: u32) !void {
    var counter: std.atomic.Value(u32) = .init(0);
    var handles: [64]JobHandle = undefined;
    const barrier = job_system.createBarrier().?;
    var i: u32 = 0;
    while (i < num_jobs) : (i += handles.len) {
        const n = @min(handles.len, num_jobs - i);
        for (handles[0..n], 0..) |*handle, j|
            handle.* = try job_system.createJob("Barrier", Color.red, .init(incrementCounter, .{&counter}), .{ .num_dependencies = @intCast(j % 2) });
        barrier.addJobs(handles[0..n]);
        for (handles[0..n], 0..) |*handle, j| {
            if (j % 2 == 1)
                handle.removeDependency(.{});
            handle.deinit();
        }
    }
    job_system.waitForJobs(barrier);
    job_system.destroyBarrier(barrier);
    try std.testing.expectEqual(num_jobs, counter.load(.monotonic));
}

test "JobSystemThreadPool runs many jobs" {
    if (builtin.single_threaded) return error.SkipZigTest;

    // More jobs than fit in the queue. max_jobs is larger than num_jobs + queue_length because jobs that were executed by
    // the barrier can still be referenced by the queue until a worker thread reaches them.
    const max_jobs = 4096;
    const num_jobs = 2000;

    var pool: JobSystemThreadPool = .empty;
    try pool.init(std.testing.allocator, std.testing.io, max_jobs, 4, .{ .num_threads = 3 });
    defer pool.deinit();
    const job_system = pool.jobSystem();
    try std.testing.expectEqual(@as(i32, 4), job_system.getMaxConcurrency());

    const Context = struct {
        counts: [num_jobs]std.atomic.Value(u32) = @splat(.init(0)),
        total: std.atomic.Value(u32) = .init(0),

        fn run(self: *@This(), i: usize) void {
            _ = self.counts[i].fetchAdd(1, .monotonic);
            _ = self.total.fetchAdd(1, .monotonic);
        }
    };
    var context: Context = .{};

    for (0..3) |round| {
        const barrier = job_system.createBarrier().?;
        var handles: [16]JobHandle = undefined;
        var i: usize = 0;
        while (i < num_jobs) : (i += handles.len) {
            const n = @min(handles.len, num_jobs - i);
            for (handles[0..n], 0..) |*handle, j|
                handle.* = try job_system.createJob("Many", Color.red, .init(Context.run, .{ &context, i + j }), .{});
            barrier.addJobs(handles[0..n]);
            for (handles[0..n]) |*handle| handle.deinit();
        }
        job_system.waitForJobs(barrier);
        job_system.destroyBarrier(barrier);

        // Every job ran exactly once
        try std.testing.expectEqual(@as(u32, @intCast((round + 1) * num_jobs)), context.total.load(.monotonic));
        for (&context.counts) |*count|
            try std.testing.expectEqual(@as(u32, @intCast(round + 1)), count.load(.monotonic));
    }
}

test "JobSystemThreadPool job dependencies" {
    if (builtin.single_threaded) return error.SkipZigTest;

    const num_graphs = 100;
    var pool: JobSystemThreadPool = .empty;
    try pool.init(std.testing.allocator, std.testing.io, 512, 1, .{ .num_threads = 3 });
    defer pool.deinit();

    // A diamond: top -> (left, right) -> bottom, each job records when it ran
    const Graph = struct {
        handles: [4]JobHandle = @splat(.empty), // top, left, right, bottom
        order: [4]std.atomic.Value(u32) = @splat(.init(0)),
        counter: *std.atomic.Value(u32),

        fn run(self: *@This(), index: usize) void {
            self.order[index].store(self.counter.fetchAdd(1, .seq_cst), .seq_cst);
            switch (index) {
                0 => JobHandle.removeDependencies(self.handles[1..3], .{}),
                1, 2 => self.handles[3].removeDependency(.{}),
                else => {},
            }
        }
    };
    var counter: std.atomic.Value(u32) = .init(1);
    var graphs: [num_graphs]Graph = @splat(.{ .counter = &counter });
    defer for (&graphs) |*graph| for (&graph.handles) |*handle| handle.deinit();

    const barrier = pool.createBarrier().?;
    for (&graphs) |*graph| {
        // Create the jobs bottom up, the top job has a dependency so that all graphs start at the same time
        graph.handles[3] = try pool.createJob("Bottom", Color.red, .init(Graph.run, .{ graph, @as(usize, 3) }), .{ .num_dependencies = 2 });
        graph.handles[2] = try pool.createJob("Right", Color.green, .init(Graph.run, .{ graph, @as(usize, 2) }), .{ .num_dependencies = 1 });
        graph.handles[1] = try pool.createJob("Left", Color.green, .init(Graph.run, .{ graph, @as(usize, 1) }), .{ .num_dependencies = 1 });
        graph.handles[0] = try pool.createJob("Top", Color.blue, .init(Graph.run, .{ graph, @as(usize, 0) }), .{ .num_dependencies = 1 });
        barrier.addJobs(&graph.handles);
    }

    // Start all graphs at once
    var tops: [num_graphs]JobHandle = undefined;
    for (&tops, &graphs) |*top, *graph| top.* = graph.handles[0].clone();
    defer for (&tops) |*top| top.deinit();
    JobHandle.removeDependencies(&tops, .{});

    pool.waitForJobs(barrier);
    pool.destroyBarrier(barrier);

    // Every job ran once, after the jobs it depends on
    try std.testing.expectEqual(@as(u32, 4 * num_graphs + 1), counter.load(.seq_cst));
    for (&graphs) |*graph| {
        const top = graph.order[0].load(.seq_cst);
        const left = graph.order[1].load(.seq_cst);
        const right = graph.order[2].load(.seq_cst);
        const bottom = graph.order[3].load(.seq_cst);
        try std.testing.expect(top > 0 and top < left and top < right and left < bottom and right < bottom);
        for (&graph.handles) |*handle| try std.testing.expect(handle.isDone());
    }
}

test "JobSystemThreadPool jobs creating jobs" {
    if (builtin.single_threaded) return error.SkipZigTest;

    // max_jobs covers all jobs of one tree + the jobs of the previous tree that can still be referenced by the queue
    var pool: JobSystemThreadPool = .empty;
    try pool.init(std.testing.allocator, std.testing.io, 4096, 1, .{ .num_threads = 3 });
    defer pool.deinit();

    // Each job creates two jobs until max_depth is reached (a binary tree of 2^max_depth - 1 jobs), the new jobs are added
    // to the barrier while it is being waited on
    const Context = struct {
        job_system: JobSystem,
        barrier: *Barrier,
        num_executed: std.atomic.Value(u32) = .init(0),

        const max_depth = 10;

        fn run(self: *@This(), depth: u32) void {
            _ = self.num_executed.fetchAdd(1, .monotonic);
            if (depth + 1 < max_depth) {
                var children: [2]JobHandle = undefined;
                for (&children) |*child|
                    child.* = self.job_system.createJob("Child", Color.green, .init(run, .{ self, depth + 1 }), .{}) catch @panic("out of memory");
                self.barrier.addJobs(&children);
                for (&children) |*child| child.deinit();
            }
        }
    };

    const job_system = pool.jobSystem();
    for (0..3) |_| {
        const barrier = job_system.createBarrier().?;
        var context: Context = .{ .job_system = job_system, .barrier = barrier };
        {
            var root = try job_system.createJob("Root", Color.green, .init(Context.run, .{ &context, @as(u32, 0) }), .{});
            defer root.deinit();
            barrier.addJob(&root);
        }
        job_system.waitForJobs(barrier);
        job_system.destroyBarrier(barrier);
        try std.testing.expectEqual(@as(u32, (1 << Context.max_depth) - 1), context.num_executed.load(.monotonic));
    }
}

test "JobSystemThreadPool setNumThreads" {
    if (builtin.single_threaded) return error.SkipZigTest;

    const Counters = struct {
        num_init: std.atomic.Value(u32) = .init(0),
        num_exit: std.atomic.Value(u32) = .init(0),
        index_sum: std.atomic.Value(i32) = .init(0),

        fn onInit(self: *@This(), thread_index: i32) void {
            _ = self.num_init.fetchAdd(1, .seq_cst);
            _ = self.index_sum.fetchAdd(thread_index, .seq_cst);
        }

        fn onExit(self: *@This(), thread_index: i32) void {
            _ = self.num_exit.fetchAdd(1, .seq_cst);
            _ = self.index_sum.fetchSub(thread_index, .seq_cst);
        }
    };
    var counters: Counters = .{};

    var pool: JobSystemThreadPool = .empty;
    pool.setThreadInitFunction(.init(Counters.onInit, .{&counters}));
    pool.setThreadExitFunction(.init(Counters.onExit, .{&counters}));
    try pool.init(std.testing.allocator, std.testing.io, 256, 2, .{ .num_threads = 0 });
    defer pool.deinit();
    const job_system = pool.jobSystem();

    // Without worker threads the barrier executes the jobs
    try std.testing.expectEqual(@as(i32, 1), job_system.getMaxConcurrency());
    try runJobsThroughBarrier(job_system, 200);

    var expected_threads: u32 = 0;
    for ([_]i32{ 2, 4, 0, 1, 3 }) |num_threads| {
        try pool.setNumThreads(num_threads);
        expected_threads += @intCast(num_threads);
        try std.testing.expectEqual(num_threads + 1, job_system.getMaxConcurrency());
        try std.testing.expectEqual(@as(usize, @intCast(num_threads)), pool.heads.len);
        try runJobsThroughBarrier(job_system, 200);
    }

    // Stopping the threads calls the exit function of every thread that was started
    try pool.setNumThreads(0);
    try std.testing.expectEqual(@as(i32, 1), job_system.getMaxConcurrency());
    try std.testing.expectEqual(expected_threads, counters.num_init.load(.seq_cst));
    try std.testing.expectEqual(expected_threads, counters.num_exit.load(.seq_cst));
    try std.testing.expectEqual(@as(i32, 0), counters.index_sum.load(.seq_cst));
    try std.testing.expectEqual(@as(u32, 0), pool.tail_state.tail.load(.seq_cst));

    // Auto detect the number of threads
    try pool.setNumThreads(-1);
    const cpu_count: i32 = @intCast(std.Thread.getCpuCount() catch 1);
    try std.testing.expectEqual(cpu_count, job_system.getMaxConcurrency());
    try runJobsThroughBarrier(job_system, 100);
}

test "JobSystemThreadPool jobs without barrier" {
    if (builtin.single_threaded) return error.SkipZigTest;

    const num_jobs = 500;
    var pool: JobSystemThreadPool = .empty;
    try pool.init(std.testing.allocator, std.testing.io, 512, 1, .{ .num_threads = 2 });
    defer pool.deinit();

    // Worker threads execute jobs that are not part of a barrier
    var counter: std.atomic.Value(u32) = .init(0);
    {
        var handle = try pool.createJob("NoBarrier", Color.red, .init(incrementCounter, .{&counter}), .{});
        defer handle.deinit();
        while (!handle.isDone())
            std.Thread.yield() catch {};
        try std.testing.expectEqual(@as(u32, 1), counter.load(.monotonic));
    }

    // Stopping the threads executes the jobs that are still in the queue
    var handles: [num_jobs]JobHandle = undefined;
    for (&handles) |*handle|
        handle.* = try pool.createJob("Queued", Color.red, .init(incrementCounter, .{&counter}), .{});
    defer for (&handles) |*handle| handle.deinit();
    try pool.setNumThreads(0);
    for (&handles) |*handle| try std.testing.expect(handle.isDone());
    try std.testing.expectEqual(@as(u32, num_jobs + 1), counter.load(.monotonic));

    // Without threads a job only runs when a barrier executes it
    var handle = try pool.createJob("NoThreads", Color.red, .init(incrementCounter, .{&counter}), .{});
    defer handle.deinit();
    try std.testing.expect(!handle.isDone());
    const barrier = pool.createBarrier().?;
    barrier.addJob(&handle);
    pool.waitForJobs(barrier);
    pool.destroyBarrier(barrier);
    try std.testing.expect(handle.isDone());
    try std.testing.expectEqual(@as(u32, num_jobs + 2), counter.load(.monotonic));
}

test "JobSystemThreadPool queue full" {
    if (builtin.single_threaded) return error.SkipZigTest;

    // Fill the queue while the only worker thread is busy, the thread that queues jobs then has to wait for space in the queue
    const num_jobs = 1500;
    var pool: JobSystemThreadPool = .empty;
    try pool.init(std.testing.allocator, std.testing.io, 2048, 1, .{ .num_threads = 1 });
    defer pool.deinit();

    const Context = struct {
        pool: *JobSystemThreadPool,
        num_executed: std.atomic.Value(u32) = .init(0),

        // Blocks the worker thread until the queue is full
        fn block(self: *@This()) void {
            while (self.pool.tail_state.tail.load(.seq_cst) -% self.pool.heads[0].load(.seq_cst) < JobSystemThreadPool.queue_length)
                std.Thread.yield() catch {};
            _ = self.num_executed.fetchAdd(1, .monotonic);
        }

        fn run(self: *@This()) void {
            _ = self.num_executed.fetchAdd(1, .monotonic);
        }
    };
    var context: Context = .{ .pool = &pool };

    const barrier = pool.createBarrier().?;
    for (0..num_jobs) |i| {
        const function: JobFunction = if (i == 0) .init(Context.block, .{&context}) else .init(Context.run, .{&context});
        var handle = try pool.createJob("Queue", Color.red, function, .{});
        defer handle.deinit();
        barrier.addJob(&handle);
    }
    pool.waitForJobs(barrier);
    pool.destroyBarrier(barrier);
    try std.testing.expectEqual(@as(u32, num_jobs), context.num_executed.load(.monotonic));
}

test "JobSystemThreadPool barriers waited on by several threads" {
    if (builtin.single_threaded) return error.SkipZigTest;

    const num_waiters = 4;
    const num_jobs = 200;

    // max_jobs covers the jobs in the barriers + the jobs that can still be referenced by the queue
    var pool: JobSystemThreadPool = .empty;
    try pool.init(std.testing.allocator, std.testing.io, 2048, num_waiters, .{ .num_threads = 2 });
    defer pool.deinit();

    const Waiter = struct {
        job_system: JobSystem,
        values: [num_jobs]std.atomic.Value(u32) = @splat(.init(0)),

        fn run(self: *@This()) void {
            for (0..3) |_| {
                const barrier = self.job_system.createBarrier().?;
                for (&self.values) |*value| {
                    var handle = self.job_system.createJob("Waiter", Color.red, .init(incrementCounter, .{value}), .{}) catch @panic("out of memory");
                    defer handle.deinit();
                    barrier.addJob(&handle);
                }
                self.job_system.waitForJobs(barrier);
                self.job_system.destroyBarrier(barrier);
            }
        }
    };
    var waiters: [num_waiters]Waiter = @splat(.{ .job_system = pool.jobSystem() });
    var threads: [num_waiters]std.Thread = undefined;
    for (&threads, &waiters) |*thread, *waiter|
        thread.* = try std.Thread.spawn(.{}, Waiter.run, .{waiter});
    for (threads) |thread| thread.join();

    for (&waiters) |*waiter| {
        for (&waiter.values) |*value|
            try std.testing.expectEqual(@as(u32, 3), value.load(.monotonic));
    }
}

test "JobSystemThreadPool InitExitFunction" {
    const Context = struct {
        fn record(values: *[4]i32, offset: i32, thread_index: i32) void {
            values[@intCast(thread_index)] = offset + thread_index;
        }
    };
    var values: [4]i32 = @splat(0);
    var offset: i32 = 100;
    _ = &offset;
    const function: JobSystemThreadPool.InitExitFunction = .init(Context.record, .{ &values, offset });
    function.call(1);
    function.call(3);
    JobSystemThreadPool.InitExitFunction.noop.call(2);
    try std.testing.expectEqual([4]i32{ 0, 101, 0, 103 }, values);
}
