//! Port of: Jolt/Core/JobSystem.h, Jolt/Core/JobSystem.inl
//! Status: complete
//!
//! Differences with the C++ version:
//! - `JobSystem` is a type erased interface (pattern B of the porting guide, like std.mem.Allocator): `ptr` + `vtable`,
//!   made with `JobSystem.init(impl)` from any `*T` that declares the virtual functions of the C++ class. All of them
//!   are pure virtual, so `T` must declare all of them, including the protected `queueJob`, `queueJobs` and `freeJob`
//!   (they must be `pub` because the generated vtable calls them). Implementations hand out the interface with a
//!   `jobSystem()` method, e.g. `thread_pool.jobSystem()`, and must not move while jobs exist (a job stores it).
//! - `Barrier` uses pattern A (the implementation embeds `base: Barrier`, which holds the vtable) instead of a fat
//!   pointer: a barrier is identified by its address, `createBarrier` returns a `*Barrier` and a `Job` stores the
//!   barrier pointer in an atomic integer (mBarrier), which a fat pointer does not fit in.
//! - `JobFunction` (std::function<void()>) is a small closure: a comptime known function plus a copy of its arguments,
//!   stored inline in the job (no allocation, like the small buffer of std::function). A lambda capture becomes the
//!   argument tuple: capture by reference -> a pointer, capture by value -> a copy, e.g.
//!   `[&values, i] { values[i]++; }` -> `JobFunction.init(increment, .{ &values, i })`.
//! - `JobHandle` (which privately derives from Ref<Job>) holds a `Ref(Job)`. Zig has no copy constructors or
//!   destructors: copy a handle with `clone()`, release it with `deinit()`; a plain assignment moves it.
//! - Default arguments become options structs: `createJob(name, color, function, .{ .num_dependencies = 1 })`,
//!   `handle.addDependency(.{})`, `handle.removeDependency(.{ .count = 2 })`.
//! - `createJob` returns `error.OutOfMemory` when the storage for the job cannot be allocated (Jolt does not check).
//! - `Job.release` decrements the reference count with acq_rel instead of release + an acquire fence (Zig has no
//!   standalone fence), which is Jolt's JPH_TSAN_ENABLED path; the count is a `RefCount` of Core/Reference.zig.
//! - The profiler members of `Job` (mJobName, mColor and GetName, which Jolt only has with JPH_PROFILE_ENABLED or
//!   JPH_EXTERNAL_PROFILE) always exist. JPH_PROFILE scopes are dropped (see the porting guide).
//! - `JobHandle.removeDependencies` collects the jobs to queue in a fixed size buffer on the stack instead of
//!   JPH_STACK_ALLOC (alloca, sized by the number of handles). When more jobs become ready than the buffer holds, they
//!   are queued in batches of `JobHandle.max_jobs_per_batch` (in the same order); the physics code never removes
//!   dependencies from more than 32 handles at once (PhysicsUpdateContext::cMaxConcurrency).

const std = @import("std");
const Color = @import("Color.zig").Color;
const Reference = @import("Reference.zig");
const Ref = Reference.Ref;
const RefCount = Reference.RefCount;
const StaticArray = @import("StaticArray.zig").StaticArray;

/// A class that allows units of work (Jobs) to be scheduled across multiple threads.
/// It allows dependencies between the jobs so that the jobs form a graph.
///
/// The pattern for using this class is:
/// ```zig
/// // Create job system
/// var thread_pool: JobSystemThreadPool = .empty;
/// try thread_pool.init(allocator, io, max_jobs, max_barriers, .{});
/// const job_system = thread_pool.jobSystem();
///
/// // Create some jobs
/// var second_job = try job_system.createJob("SecondJob", Color.red, .init(secondJob, .{...}), .{ .num_dependencies = 1 }); // Create a job with 1 dependency
/// var first_job = try job_system.createJob("FirstJob", Color.green, .init(firstJob, .{&second_job}), .{}); // Job can start immediately, will start second job when it's done (firstJob calls second_job.removeDependency(.{}))
/// var third_job = try job_system.createJob("ThirdJob", Color.blue, .init(thirdJob, .{...}), .{}); // This job can run immediately as well and can run in parallel to job 1 and 2
///
/// // Add the jobs to the barrier so that we can execute them while we're waiting
/// const barrier = job_system.createBarrier().?;
/// barrier.addJob(&first_job);
/// barrier.addJob(&second_job);
/// barrier.addJob(&third_job);
/// job_system.waitForJobs(barrier);
///
/// // Clean up
/// job_system.destroyBarrier(barrier);
/// first_job.deinit();
/// second_job.deinit();
/// third_job.deinit();
/// thread_pool.deinit();
/// ```
///
/// Jobs are guaranteed to be started in the order that their dependency counter becomes zero (in case they're scheduled on a background thread)
/// or in the order they're added to the barrier (when dependency count is zero and when executing on the thread that calls WaitForJobs).
///
/// If you want to implement your own job system, implement the following functions and create the interface with `JobSystem.init(&your_job_system)`:
///
/// * getMaxConcurrency - This should return the maximum number of jobs that can run in parallel.
/// * createJob - This should create a Job object and return it to the caller.
/// * freeJob - This should free the memory associated with the job object. It is called when the job is release()-ed for the last time.
/// * queueJob/queueJobs - These should store the job pointer in an internal queue to run immediately (dependencies are tracked internally, this function is called when the job can run).
/// The Job objects are reference counted and are guaranteed to stay alive during the queueJob(s) call. If you store the job in your own data structure you need to call addRef() to take a reference.
/// After the job has been executed you need to call release() to release the reference. Make sure you no longer dereference the job pointer after calling release().
///
/// JobSystem.Barrier is used to track the completion of a set of jobs. Jobs will be created by other jobs and added to the barrier while it is being waited on. This means that you cannot
/// create a dependency graph beforehand as the graph changes while jobs are running. Implement the following functions:
///
/// * Barrier.addJob/addJobs - Add a job to the barrier, any call to waitForJobs will now also wait for this job to complete.
/// If you store the job in a data structure in the Barrier you need to call addRef() on the job to keep it alive and release() after you're done with it.
/// * Barrier.onJobFinished - This function is called when a job has finished executing, you can use this to track completion and remove the job from the list of jobs to wait on.
///
/// The functions on JobSystem that need to be implemented to support barriers are:
///
/// * createBarrier - Create a new barrier.
/// * destroyBarrier - Destroy a barrier.
/// * waitForJobs - This is the main function that is used to wait for all jobs that have been added to a Barrier. waitForJobs can execute jobs that have
/// been added to the barrier while waiting. It is not wise to execute other jobs that touch physics structures as this can cause race conditions and deadlocks. Please keep in mind that the barrier is
/// only intended to wait on the completion of the Jolt jobs added to it, if you scheduled any jobs in your engine's job system to execute the Jolt jobs as part of queueJob/queueJobs, you might still need
/// to wait for these in this function after the barrier is finished waiting.
///
/// An example implementation is JobSystemThreadPool. If you don't want to write the Barrier class you can also build on JobSystemWithBarrier.
pub const JobSystem = struct {
    /// The job system implementation
    ptr: *anyopaque,

    /// The virtual functions of the implementation
    vtable: *const VTable,

    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Get maximum number of concurrently executing jobs
        getMaxConcurrency: *const fn (ptr: *anyopaque) i32,

        /// Create a new job (see JobSystem.createJob)
        createJob: *const fn (ptr: *anyopaque, job_name: []const u8, color: Color, job_function: JobFunction, num_dependencies: u32) error{OutOfMemory}!JobHandle,

        /// Create a new barrier, used to wait on jobs
        createBarrier: *const fn (ptr: *anyopaque) ?*Barrier,

        /// Destroy a barrier when it is no longer used. The barrier should be empty at this point.
        destroyBarrier: *const fn (ptr: *anyopaque, barrier: *Barrier) void,

        /// Wait for a set of jobs to be finished, note that only 1 thread can be waiting on a barrier at a time
        waitForJobs: *const fn (ptr: *anyopaque, barrier: *Barrier) void,

        /// Adds a job to the job queue
        queueJob: *const fn (ptr: *anyopaque, job: *Job) void,

        /// Adds a number of jobs at once to the job queue
        queueJobs: *const fn (ptr: *anyopaque, jobs: []const *Job) void,

        /// Frees a job
        freeJob: *const fn (ptr: *anyopaque, job: *Job) void,
    };

    /// Wrap any `*T` that implements the job system functions:
    /// ```zig
    /// pub fn getMaxConcurrency(self: *const T) i32
    /// pub fn createJob(self: *T, job_name: []const u8, color: Color, job_function: JobFunction, opts: struct { num_dependencies: u32 = 0 }) error{OutOfMemory}!JobHandle
    /// pub fn createBarrier(self: *T) ?*Barrier
    /// pub fn destroyBarrier(self: *T, barrier: *Barrier) void
    /// pub fn waitForJobs(self: *T, barrier: *Barrier) void
    /// pub fn queueJob(self: *T, job: *Job) void
    /// pub fn queueJobs(self: *T, jobs: []const *Job) void
    /// pub fn freeJob(self: *T, job: *Job) void
    /// ```
    pub fn init(impl: anytype) JobSystem {
        const T = @typeInfo(@TypeOf(impl)).pointer.child;
        const gen = struct {
            fn getMaxConcurrencyThunk(ptr: *anyopaque) i32 {
                const self: *T = @ptrCast(@alignCast(ptr));
                return self.getMaxConcurrency();
            }
            fn createJobThunk(ptr: *anyopaque, job_name: []const u8, color: Color, job_function: JobFunction, num_dependencies: u32) error{OutOfMemory}!JobHandle {
                const self: *T = @ptrCast(@alignCast(ptr));
                return self.createJob(job_name, color, job_function, .{ .num_dependencies = num_dependencies });
            }
            fn createBarrierThunk(ptr: *anyopaque) ?*Barrier {
                const self: *T = @ptrCast(@alignCast(ptr));
                return self.createBarrier();
            }
            fn destroyBarrierThunk(ptr: *anyopaque, barrier: *Barrier) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.destroyBarrier(barrier);
            }
            fn waitForJobsThunk(ptr: *anyopaque, barrier: *Barrier) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.waitForJobs(barrier);
            }
            fn queueJobThunk(ptr: *anyopaque, job: *Job) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.queueJob(job);
            }
            fn queueJobsThunk(ptr: *anyopaque, jobs: []const *Job) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.queueJobs(jobs);
            }
            fn freeJobThunk(ptr: *anyopaque, job: *Job) void {
                const self: *T = @ptrCast(@alignCast(ptr));
                self.freeJob(job);
            }
            const vtable: VTable = .{
                .getMaxConcurrency = getMaxConcurrencyThunk,
                .createJob = createJobThunk,
                .createBarrier = createBarrierThunk,
                .destroyBarrier = destroyBarrierThunk,
                .waitForJobs = waitForJobsThunk,
                .queueJob = queueJobThunk,
                .queueJobs = queueJobsThunk,
                .freeJob = freeJobThunk,
            };
        };
        return .{ .ptr = impl, .vtable = &gen.vtable };
    }

    /// Get maximum number of concurrently executing jobs
    pub fn getMaxConcurrency(self: JobSystem) i32 {
        return self.vtable.getMaxConcurrency(self.ptr);
    }

    /// Create a new job, the job is started immediately if opts.num_dependencies == 0 otherwise it starts when
    /// RemoveDependency causes the dependency counter to reach 0.
    pub fn createJob(self: JobSystem, job_name: []const u8, color: Color, job_function: JobFunction, opts: struct { num_dependencies: u32 = 0 }) error{OutOfMemory}!JobHandle {
        return self.vtable.createJob(self.ptr, job_name, color, job_function, opts.num_dependencies);
    }

    /// Create a new barrier, used to wait on jobs (null when no barrier is available)
    pub fn createBarrier(self: JobSystem) ?*Barrier {
        return self.vtable.createBarrier(self.ptr);
    }

    /// Destroy a barrier when it is no longer used. The barrier should be empty at this point.
    pub fn destroyBarrier(self: JobSystem, barrier: *Barrier) void {
        self.vtable.destroyBarrier(self.ptr, barrier);
    }

    /// Wait for a set of jobs to be finished, note that only 1 thread can be waiting on a barrier at a time
    pub fn waitForJobs(self: JobSystem, barrier: *Barrier) void {
        self.vtable.waitForJobs(self.ptr, barrier);
    }

    /// Adds a job to the job queue (protected in C++)
    fn queueJob(self: JobSystem, job: *Job) void {
        self.vtable.queueJob(self.ptr, job);
    }

    /// Adds a number of jobs at once to the job queue (protected in C++)
    fn queueJobs(self: JobSystem, jobs: []const *Job) void {
        self.vtable.queueJobs(self.ptr, jobs);
    }

    /// Frees a job (protected in C++)
    fn freeJob(self: JobSystem, job: *Job) void {
        self.vtable.freeJob(self.ptr, job);
    }

    /// Main function of the job (function<void()>)
    ///
    /// A closure that stores a comptime known function and a copy of the arguments to call it with (the captures of
    /// the C++ lambda). The arguments are stored inline, so creating a job never allocates for its function.
    pub const JobFunction = struct {
        /// Maximum size of the captured arguments in bytes (4 pointers)
        pub const max_capture_size = 4 * @sizeOf(usize);

        /// Maximum alignment of the captured arguments, capture larger aligned values (e.g. vectors) by pointer
        pub const max_capture_alignment = @alignOf(usize);

        /// Storage for the captured arguments
        const Captures = [max_capture_size]u8;

        /// Calls the function with the arguments stored in `captures`
        invoke: *const fn (captures: *const Captures) void,

        /// Copy of the argument tuple (the bytes after it are undefined)
        captures: Captures align(max_capture_alignment),

        /// Create a job function that calls `function` with the arguments in the tuple `args`, which are copied:
        /// `JobFunction.init(foo, .{ &context, i })` calls `foo(&context, i)` when the job executes.
        pub fn init(comptime function: anytype, args: anytype) JobFunction {
            const Args = @TypeOf(args);
            comptime {
                if (@sizeOf(Args) > max_capture_size)
                    @compileError("JobFunction: the arguments " ++ @typeName(Args) ++ " don't fit in max_capture_size, pass a pointer to a struct instead");
                if (@alignOf(Args) > max_capture_alignment)
                    @compileError("JobFunction: the arguments " ++ @typeName(Args) ++ " are aligned more than max_capture_alignment, pass them by pointer");
            }
            const gen = struct {
                fn invoke(captures: *const Captures) void {
                    const captured: *const Args = @ptrCast(@alignCast(captures));
                    @call(.auto, function, captured.*);
                }
            };
            var result: JobFunction = .{ .invoke = gen.invoke, .captures = undefined };
            if (@sizeOf(Args) > 0) // Also allows creating a function without captures at comptime
                @memcpy(result.captures[0..@sizeOf(Args)], std.mem.asBytes(&args));
            return result;
        }

        /// Call the function (operator ())
        pub fn call(self: *const JobFunction) void {
            self.invoke(&self.captures);
        }
    };

    /// A class that contains information for a single unit of work (protected in C++, used by job system implementations)
    pub const Job = struct {
        /// Name of the job (only used by the profiler in Jolt, see the top of this file)
        job_name: []const u8,

        /// Color of the job in the profiler
        color: Color,

        /// The job system we belong to
        job_system: JobSystem,

        /// Barrier that this job is associated with (is a Barrier pointer)
        barrier: std.atomic.Value(usize) = .init(0),

        /// Main job function
        job_function: JobFunction,

        /// Amount of JobHandles pointing to this job (mReferenceCount)
        ref_count: RefCount = .{},

        /// Amount of jobs that need to complete before this job can run
        num_dependencies: std.atomic.Value(u32),

        /// Value of num_dependencies when job is executing
        pub const executing_state: u32 = 0xe0e0e0e0;

        /// Value of num_dependencies when job is done executing
        pub const done_state: u32 = 0xd0d0d0d0;

        /// Value to use when the barrier has been triggered
        pub const barrier_done_state: usize = ~@as(usize, 0);

        /// Constructor
        pub fn init(job_name: []const u8, color: Color, job_system: JobSystem, job_function: JobFunction, num_dependencies: u32) Job {
            return .{
                .job_name = job_name,
                .color = color,
                .job_system = job_system,
                .job_function = job_function,
                .num_dependencies = .init(num_dependencies),
            };
        }

        /// Get the jobs system to which this job belongs
        pub fn getJobSystem(self: *const Job) JobSystem {
            return self.job_system;
        }

        /// Add a reference to this object
        pub fn addRef(self: *Job) void {
            // Adding a reference can use relaxed memory ordering
            self.ref_count.addRef();
        }

        /// Release a reference to this object, frees the job through its job system when it was the last reference
        pub fn release(self: *Job) void {
            // Releasing a reference must use release semantics so that we can use acquire to ensure that we see any
            // updates from other threads that released a ref before freeing the job. Zig has no standalone fence, so
            // like Jolt's JPH_TSAN_ENABLED path RefCount.release uses an acq_rel operation unconditionally.
            if (self.ref_count.release())
                self.job_system.freeJob(self);
        }

        /// Add to the dependency counter.
        pub fn addDependency(self: *Job, count: i32) void {
            const old_value = self.num_dependencies.fetchAdd(@bitCast(count), .monotonic);
            std.debug.assert(old_value > 0 and old_value != executing_state and old_value != done_state); // Job is queued, running or done, it is not allowed to add a dependency to a running job
        }

        /// Remove from the dependency counter. Returns true whenever the dependency counter reaches zero
        /// and if it does it is no longer valid to call the AddDependency/RemoveDependency functions.
        pub fn removeDependency(self: *Job, count: i32) bool {
            const old_value = self.num_dependencies.fetchSub(@bitCast(count), .release);
            std.debug.assert(old_value != executing_state and old_value != done_state); // Job is running or done, it is not allowed to add a dependency to a running job
            const new_value = old_value -% @as(u32, @bitCast(count));
            std.debug.assert(old_value > new_value); // Test wrap around, this is a logic error
            return new_value == 0;
        }

        /// Remove from the dependency counter. Job will be queued whenever the dependency counter reaches zero
        /// and if it does it is no longer valid to call the AddDependency/RemoveDependency functions.
        pub fn removeDependencyAndQueue(self: *Job, count: i32) void {
            if (self.removeDependency(count))
                self.job_system.queueJob(self);
        }

        /// Set the job barrier that this job belongs to and returns false if this was not possible because the job already finished
        pub fn setBarrier(self: *Job, barrier: *Barrier) bool {
            if (self.barrier.cmpxchgStrong(0, @intFromPtr(barrier), .monotonic, .monotonic)) |old_barrier| {
                std.debug.assert(old_barrier == barrier_done_state); // A job can only belong to 1 barrier
                return false;
            }
            return true;
        }

        /// Run the job function, returns the number of dependencies that this job still has or executing_state or done_state
        pub fn execute(self: *Job) u32 {
            // Transition job to executing state
            // We can only start running with a dependency counter of 0
            if (self.num_dependencies.cmpxchgStrong(0, executing_state, .acquire, .acquire)) |state|
                return state; // state is the current value when the exchange fails

            // Run the job function
            self.job_function.call();

            // Fetch the barrier pointer and exchange it for the done state, so we're sure that no barrier gets set after we want to call the callback
            var barrier = self.barrier.load(.monotonic);
            while (true) {
                barrier = self.barrier.cmpxchgWeak(barrier, barrier_done_state, .monotonic, .monotonic) orelse break;
            }
            std.debug.assert(barrier != barrier_done_state);

            // Mark job as done
            const state = self.num_dependencies.cmpxchgStrong(executing_state, done_state, .monotonic, .monotonic);
            std.debug.assert(state == null); // The state was executing_state

            // Notify the barrier after we've changed the job to the done state so that any thread reading the state after receiving the callback will see that the job has finished
            if (barrier != 0)
                @as(*Barrier, @ptrFromInt(barrier)).onJobFinished(self);

            return done_state;
        }

        /// Test if the job can be executed
        pub fn canBeExecuted(self: *const Job) bool {
            return self.num_dependencies.load(.monotonic) == 0;
        }

        /// Test if the job finished executing
        pub fn isDone(self: *const Job) bool {
            return self.num_dependencies.load(.monotonic) == done_state;
        }

        /// Get the name of the job
        pub fn getName(self: *const Job) []const u8 {
            return self.job_name;
        }
    };

    /// A job handle contains a reference to a job. The job will be deleted as soon as there are no JobHandles.
    /// referring to the job and when it is not in the job queue / being processed.
    ///
    /// Zig has no copy constructor or destructor: copy a handle with `clone()` and release it with `deinit()`.
    pub const JobHandle = struct {
        /// The job (JobHandle privately derives from Ref<Job>)
        ref: Ref(Job) = .empty,

        /// Maximum number of jobs that `removeDependencies` queues with one queueJobs call (see the top of this file)
        pub const max_jobs_per_batch = 128;

        /// Handle that doesn't contain a job (default constructor)
        pub const empty: JobHandle = .{};

        /// Constructor, only to be used by JobSystem (adds a reference to the job)
        pub fn init(job: *Job) JobHandle {
            return .{ .ref = .init(job) };
        }

        /// Copy the handle (copy constructor), adds a reference to the job
        pub fn clone(self: *const JobHandle) JobHandle {
            return .{ .ref = self.ref.clone() };
        }

        /// Release the reference to the job (destructor)
        pub fn deinit(self: *JobHandle) void {
            self.ref.deinit();
        }

        /// Assignment (operator = (const JobHandle &)): references the job of `other` and releases the current job
        pub fn set(self: *JobHandle, other: *const JobHandle) void {
            self.ref.set(other.ref.ptr);
        }

        /// Check if this handle contains a job
        pub fn isValid(self: *const JobHandle) bool {
            return self.getPtr() != null;
        }

        /// Check if this job has finished executing
        pub fn isDone(self: *const JobHandle) bool {
            return self.getPtr() != null and self.getPtr().?.isDone();
        }

        /// Add to the dependency counter.
        pub fn addDependency(self: *const JobHandle, opts: struct { count: i32 = 1 }) void {
            self.getPtr().?.addDependency(opts.count);
        }

        /// Remove from the dependency counter. Job will start whenever the dependency counter reaches zero
        /// and if it does it is no longer valid to call the AddDependency/RemoveDependency functions.
        pub fn removeDependency(self: *const JobHandle, opts: struct { count: i32 = 1 }) void {
            self.getPtr().?.removeDependencyAndQueue(opts.count);
        }

        /// Remove a dependency from a batch of jobs at once, this can be more efficient than removing them one by one as it requires less locking
        pub fn removeDependencies(handles: []const JobHandle, opts: struct { count: i32 = 1 }) void {
            std.debug.assert(handles.len > 0);

            // Get the job system, all jobs should be part of the same job system
            const job_system = handles[0].getPtr().?.getJobSystem();

            // Buffer to store the jobs that need to be queued (JPH_STACK_ALLOC in Jolt, see the top of this file)
            var jobs_to_queue: [max_jobs_per_batch]*Job = undefined;
            var num_jobs_to_queue: usize = 0;

            // Remove the dependencies on all jobs
            for (handles) |*handle| {
                const job = handle.getPtr().?;
                std.debug.assert(job.getJobSystem().ptr == job_system.ptr); // All jobs should belong to the same job system
                if (job.removeDependency(opts.count)) {
                    jobs_to_queue[num_jobs_to_queue] = job;
                    num_jobs_to_queue += 1;

                    // Zolt: the buffer is full, queue the jobs collected so far
                    if (num_jobs_to_queue == jobs_to_queue.len) {
                        job_system.queueJobs(&jobs_to_queue);
                        num_jobs_to_queue = 0;
                    }
                }
            }

            // If any jobs need to be scheduled, schedule them as a batch
            if (num_jobs_to_queue != 0)
                job_system.queueJobs(jobs_to_queue[0..num_jobs_to_queue]);
        }

        /// Helper function to remove dependencies on a static array of job handles
        /// (`handles` is a `*StaticArray(JobHandle, N)` or `*const StaticArray(JobHandle, N)`)
        pub fn removeDependenciesStaticArray(handles: anytype, opts: struct { count: i32 = 1 }) void {
            const Array = @typeInfo(@TypeOf(handles)).pointer.child;
            comptime {
                if (Array != StaticArray(JobHandle, Array.capacity))
                    @compileError("removeDependenciesStaticArray expects a pointer to a StaticArray(JobHandle, N), got " ++ @typeName(@TypeOf(handles)));
            }
            removeDependencies(handles.constSlice(), .{ .count = opts.count });
        }

        /// Get the job, only to be used by the JobSystem (GetPtr)
        pub fn getPtr(self: *const JobHandle) ?*Job {
            return self.ref.get();
        }
    };

    /// A job barrier keeps track of a number of jobs and allows waiting until they are all completed.
    ///
    /// Implementations embed it as their field `base` (pattern A of the porting guide) and destroy it with
    /// JobSystem.destroyBarrier.
    pub const Barrier = struct {
        /// The virtual functions of the implementation (qualified because JobSystem.VTable makes `VTable` ambiguous here)
        vtable: *const Barrier.VTable,

        pub const VTable = struct {
            /// Add a job to this barrier
            addJob: *const fn (self: *Barrier, job: *const JobHandle) void,

            /// Add multiple jobs to this barrier
            addJobs: *const fn (self: *Barrier, handles: []const JobHandle) void,

            /// Called by a Job to mark that it is finished
            onJobFinished: *const fn (self: *Barrier, job: *Job) void,
        };

        /// Add a job to this barrier
        /// Note that jobs can keep being added to the barrier while waiting for the barrier
        pub fn addJob(self: *Barrier, job: *const JobHandle) void {
            self.vtable.addJob(self, job);
        }

        /// Add multiple jobs to this barrier
        /// Note that jobs can keep being added to the barrier while waiting for the barrier
        pub fn addJobs(self: *Barrier, handles: []const JobHandle) void {
            self.vtable.addJobs(self, handles);
        }

        /// Called by a Job to mark that it is finished (protected in C++, Job is a friend)
        fn onJobFinished(self: *Barrier, job: *Job) void {
            self.vtable.onJobFinished(self, job);
        }
    };
};

// `using JobHandle = JobSystem::JobHandle;` is the re-export `zolt.JobHandle` in zolt.zig: a file level alias would
// make the references to `JobHandle` inside `JobSystem` ambiguous.

/// A minimal job system for the tests below: queued jobs are stored in a list and executed by waitForJobs, jobs are
/// allocated with the allocator.
const TestJobSystem = struct {
    const Job = JobSystem.Job;
    const JobFunction = JobSystem.JobFunction;
    const JobHandle = JobSystem.JobHandle;
    const Barrier = JobSystem.Barrier;

    allocator: std.mem.Allocator,
    queue: std.ArrayList(*Job) = .empty,
    num_queue_jobs_calls: u32 = 0,
    num_created: u32 = 0,
    num_freed: u32 = 0,
    barrier: TestBarrier = .{},

    const TestBarrier = struct {
        base: Barrier = .{ .vtable = &vtable },
        num_added: u32 = 0,
        num_finished: u32 = 0,
        last_finished: ?*Job = null,

        const vtable: Barrier.VTable = .{ .addJob = addJob, .addJobs = addJobs, .onJobFinished = onJobFinished };

        fn fromBarrier(barrier: *Barrier) *TestBarrier {
            return @fieldParentPtr("base", barrier);
        }
        fn addJob(barrier: *Barrier, job: *const JobHandle) void {
            const self = fromBarrier(barrier);
            if (job.getPtr().?.setBarrier(barrier))
                self.num_added += 1;
        }
        fn addJobs(barrier: *Barrier, handles: []const JobHandle) void {
            for (handles) |*handle| addJob(barrier, handle);
        }
        fn onJobFinished(barrier: *Barrier, job: *Job) void {
            const self = fromBarrier(barrier);
            self.num_finished += 1;
            self.last_finished = job;
        }
    };

    fn deinit(self: *TestJobSystem) void {
        std.debug.assert(self.queue.items.len == 0);
        self.queue.deinit(self.allocator);
    }

    fn jobSystem(self: *TestJobSystem) JobSystem {
        return .init(self);
    }

    pub fn getMaxConcurrency(self: *const TestJobSystem) i32 {
        _ = self;
        return 1;
    }

    pub fn createJob(self: *TestJobSystem, job_name: []const u8, color: Color, job_function: JobFunction, opts: struct { num_dependencies: u32 = 0 }) error{OutOfMemory}!JobHandle {
        try self.queue.ensureUnusedCapacity(self.allocator, 1);
        const job = try self.allocator.create(Job);
        job.* = .init(job_name, color, self.jobSystem(), job_function, opts.num_dependencies);
        self.num_created += 1;
        const handle: JobHandle = .init(job);
        if (opts.num_dependencies == 0)
            self.queueJob(job);
        return handle;
    }

    pub fn createBarrier(self: *TestJobSystem) ?*Barrier {
        return &self.barrier.base;
    }

    pub fn destroyBarrier(self: *TestJobSystem, barrier: *Barrier) void {
        _ = self;
        _ = barrier;
    }

    pub fn waitForJobs(self: *TestJobSystem, barrier: *Barrier) void {
        _ = barrier;
        self.runQueuedJobs();
    }

    pub fn queueJob(self: *TestJobSystem, job: *Job) void {
        job.addRef();
        self.queue.append(self.allocator, job) catch @panic("TestJobSystem: out of memory");
    }

    pub fn queueJobs(self: *TestJobSystem, jobs: []const *Job) void {
        self.num_queue_jobs_calls += 1;
        for (jobs) |job| self.queueJob(job);
    }

    pub fn freeJob(self: *TestJobSystem, job: *Job) void {
        self.num_freed += 1;
        self.allocator.destroy(job);
    }

    /// Execute the queued jobs in order (including jobs that get queued while doing so)
    fn runQueuedJobs(self: *TestJobSystem) void {
        var i: usize = 0;
        while (i < self.queue.items.len) : (i += 1) {
            const job = self.queue.items[i];
            _ = job.execute();
            job.release();
        }
        self.queue.clearRetainingCapacity();
    }
};

test "JobFunction calls the function with the captured arguments" {
    const JobFunction = JobSystem.JobFunction;
    const Context = struct {
        fn noArguments() void {
            no_arguments_calls += 1;
        }
        fn increment(value: *u32) void {
            value.* += 1;
        }
        fn setIndex(values: *[4]u64, index: usize, value: u64) void {
            values[index] = value;
        }
        fn fourWords(a: *u64, b: u64, c: usize, d: *u64) void {
            a.* = b + c + d.*;
        }
        fn smallValues(sum: *i64, a: u8, b: i16, c: bool, d: i32) void {
            sum.* = @as(i64, a) + b + d + @intFromBool(c);
        }
        var no_arguments_calls: u32 = 0;
    };

    // No captures ([] { ... })
    const no_arguments: JobFunction = .init(Context.noArguments, .{});
    no_arguments.call();
    no_arguments.call();
    try std.testing.expectEqual(@as(u32, 2), Context.no_arguments_calls);

    // Capture by reference ([&value] { ... })
    var value: u32 = 5;
    const increment: JobFunction = .init(Context.increment, .{&value});
    increment.call();
    try std.testing.expectEqual(@as(u32, 6), value);

    // Captures by value are copied when the job function is created ([&values, i] { ... })
    var values: [4]u64 = @splat(0);
    var index: usize = 2;
    var new_value: u64 = 42;
    const set_index: JobFunction = .init(Context.setIndex, .{ &values, index, new_value });
    index = 3;
    new_value = 0;
    const copy = set_index; // Copying the job function copies the captures
    copy.call();
    try std.testing.expectEqual([4]u64{ 0, 0, 42, 0 }, values);

    // The maximum amount of captures
    var result: u64 = 0;
    var b: u64 = 20;
    var c: usize = 3;
    var d: u64 = 100;
    _ = .{ &b, &c, &d };
    const four_words: JobFunction = .init(Context.fourWords, .{ &result, b, c, &d });
    try std.testing.expectEqual(JobFunction.max_capture_size, @sizeOf(@TypeOf(.{ &result, b, c, &d })));
    four_words.call();
    try std.testing.expectEqual(@as(u64, 123), result);

    // Small values and comptime known arguments (which take no storage)
    var sum: i64 = 0;
    var small: u8 = 200;
    var negative: i16 = -50;
    _ = .{ &small, &negative };
    const small_values: JobFunction = .init(Context.smallValues, .{ &sum, small, negative, true, 7 });
    small_values.call();
    try std.testing.expectEqual(@as(i64, 158), sum);
}

test "Job dependencies and state" {
    const Job = JobSystem.Job;
    const JobHandle = JobSystem.JobHandle;
    var test_system: TestJobSystem = .{ .allocator = std.testing.allocator };
    defer test_system.deinit();
    const job_system = test_system.jobSystem();
    try std.testing.expectEqual(@as(i32, 1), job_system.getMaxConcurrency());

    var value: u32 = 0;
    const Context = struct {
        fn increment(v: *u32) void {
            v.* += 1;
        }
    };

    // A job with 2 dependencies is not queued
    var handle = try job_system.createJob("Test", Color.red, .init(Context.increment, .{&value}), .{ .num_dependencies = 2 });
    try std.testing.expect(handle.isValid());
    const job = handle.getPtr().?;
    try std.testing.expectEqualStrings("Test", job.getName());
    try std.testing.expect(job.color.eql(Color.red));
    try std.testing.expectEqual(job_system.ptr, job.getJobSystem().ptr);
    try std.testing.expectEqual(@as(u32, 1), job.ref_count.get());
    try std.testing.expectEqual(@as(usize, 0), test_system.queue.items.len);
    try std.testing.expect(!job.canBeExecuted());
    try std.testing.expect(!handle.isDone());

    // Executing a job that has dependencies does nothing and returns the number of dependencies
    try std.testing.expectEqual(@as(u32, 2), job.execute());
    try std.testing.expectEqual(@as(u32, 0), value);

    handle.addDependency(.{ .count = 3 });
    try std.testing.expectEqual(@as(u32, 5), job.num_dependencies.load(.monotonic));
    handle.removeDependency(.{});
    handle.removeDependency(.{ .count = 3 });
    try std.testing.expectEqual(@as(u32, 1), job.num_dependencies.load(.monotonic));
    try std.testing.expectEqual(@as(usize, 0), test_system.queue.items.len);

    // Removing the last dependency queues the job (which takes a reference)
    handle.removeDependency(.{});
    try std.testing.expect(job.canBeExecuted());
    try std.testing.expectEqual(@as(usize, 1), test_system.queue.items.len);
    try std.testing.expectEqual(@as(u32, 2), job.ref_count.get());

    // A copy of the handle keeps the job alive
    var copy = handle.clone();
    try std.testing.expectEqual(@as(u32, 3), job.ref_count.get());
    var assigned: JobHandle = .empty;
    try std.testing.expect(!assigned.isValid());
    try std.testing.expect(!assigned.isDone());
    assigned.set(&copy);
    assigned.set(&copy);
    try std.testing.expectEqual(@as(u32, 4), job.ref_count.get());
    assigned.deinit();
    try std.testing.expect(!assigned.isValid());

    // Execute the job
    const barrier = job_system.createBarrier().?;
    barrier.addJob(&handle);
    try std.testing.expectEqual(@as(u32, 1), test_system.barrier.num_added);
    job_system.waitForJobs(barrier);
    job_system.destroyBarrier(barrier);
    try std.testing.expectEqual(@as(u32, 1), value);
    try std.testing.expect(handle.isDone());
    try std.testing.expect(copy.isDone());
    try std.testing.expectEqual(Job.done_state, job.num_dependencies.load(.monotonic));
    try std.testing.expectEqual(Job.barrier_done_state, job.barrier.load(.monotonic));
    try std.testing.expectEqual(@as(u32, 1), test_system.barrier.num_finished);
    try std.testing.expectEqual(job, test_system.barrier.last_finished.?);

    // Executing it again does nothing
    try std.testing.expectEqual(Job.done_state, job.execute());
    try std.testing.expectEqual(@as(u32, 1), value);

    // A finished job can no longer be added to a barrier
    try std.testing.expect(!job.setBarrier(barrier));
    barrier.addJob(&handle);
    try std.testing.expectEqual(@as(u32, 1), test_system.barrier.num_added);

    // Releasing the last handle frees the job
    handle.deinit();
    try std.testing.expectEqual(@as(u32, 0), test_system.num_freed);
    copy.deinit();
    try std.testing.expectEqual(@as(u32, 1), test_system.num_freed);
}

test "JobHandle removeDependencies" {
    const JobHandle = JobSystem.JobHandle;
    var test_system: TestJobSystem = .{ .allocator = std.testing.allocator };
    defer test_system.deinit();
    const job_system = test_system.jobSystem();

    const num_jobs = 300;
    var order: std.ArrayList(u32) = .empty;
    defer order.deinit(std.testing.allocator);
    try order.ensureTotalCapacity(std.testing.allocator, num_jobs + 1);
    const Context = struct {
        fn record(o: *std.ArrayList(u32), index: u32) void {
            o.appendAssumeCapacity(index);
        }
    };

    var handles: [num_jobs]JobHandle = @splat(.empty);
    defer for (&handles) |*handle| handle.deinit();
    for (&handles, 0..) |*handle, i|
        handle.* = try job_system.createJob("Test", Color.green, .init(Context.record, .{ &order, @as(u32, @intCast(i)) }), .{ .num_dependencies = if (i % 3 == 0) 2 else 1 });

    // Only the jobs whose dependency counter reaches zero are queued, in batches of max_jobs_per_batch
    JobHandle.removeDependencies(&handles, .{});
    try std.testing.expectEqual(@as(usize, 200), test_system.queue.items.len);
    try std.testing.expectEqual(@as(u32, 2), test_system.num_queue_jobs_calls);
    for (test_system.queue.items, 0..) |job, i|
        try std.testing.expectEqual(handles[i + i / 2 + 1].getPtr().?, job);

    // Remove the second dependency of the other jobs
    var remaining: StaticArray(JobHandle, num_jobs / 3) = .empty;
    defer for (remaining.slice()) |*handle| handle.deinit();
    for (0..num_jobs / 3) |i| remaining.append(handles[3 * i].clone());
    JobHandle.removeDependenciesStaticArray(&remaining, .{});
    try std.testing.expectEqual(@as(u32, 3), test_system.num_queue_jobs_calls);
    try std.testing.expectEqual(@as(usize, num_jobs), test_system.queue.items.len);

    // Jobs execute in the order they were queued
    test_system.runQueuedJobs();
    for (order.items[0..200], 0..) |index, i|
        try std.testing.expectEqual(@as(u32, @intCast(i + i / 2 + 1)), index);
    for (order.items[200..], 0..) |index, i|
        try std.testing.expectEqual(@as(u32, @intCast(3 * i)), index);
    for (&handles) |*handle| try std.testing.expect(handle.isDone());

    // Removing more dependencies than 1 at a time
    var multi = try job_system.createJob("Test", Color.blue, .init(Context.record, .{ &order, @as(u32, 1000) }), .{ .num_dependencies = 4 });
    defer multi.deinit();
    JobHandle.removeDependencies((&multi)[0..1], .{ .count = 3 });
    try std.testing.expect(!multi.getPtr().?.canBeExecuted());
    JobHandle.removeDependencies((&multi)[0..1], .{ .count = 1 });
    try std.testing.expect(multi.getPtr().?.canBeExecuted());
    test_system.runQueuedJobs();
    try std.testing.expect(multi.isDone());
    try std.testing.expectEqual(@as(u32, 1000), order.getLast());
    try std.testing.expectEqual(@as(u32, 4), test_system.num_queue_jobs_calls);
}
