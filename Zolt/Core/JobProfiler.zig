//! Zolt addition, no Jolt file: a timeline of the jobs that a job system runs (build option `-Djob_profile=true`)
//! Status: complete
//!
//! Jolt's profiler (Jolt/Core/Profiler, JPH_PROFILE) is not ported. This smaller tool shows where the time of the
//! job threads goes. With `-Djob_profile=true` the job system records, per thread, when each job starts and ends and
//! when the thread waits for work (idle):
//! - `Job.execute` records the job body (Core/JobSystem.zig), under the name of the job (Jolt's job names, such as
//!   "FindCollisions" or "SolveVelocityConstraints").
//! - `JobSystemThreadPool` records the time a worker thread waits for jobs, `JobSystemWithBarrier` the time a thread
//!   waits for a barrier.
//!
//! Without the option `Core.job_profile` is false: the profiler fields of the job systems are `void` and the hooks
//! generate no code, like the debug renderer gates (Docs/Zolt/RemainingFeaturesArchitecture.md R2).
//!
//! Rules:
//! - The state lives in the job system (`JobSystemWithBarrier.profiler`), not in a global. A job system hands it to
//!   the jobs with `getJobProfiler` (an optional function of the JobSystem interface).
//! - Each thread writes only its own ring buffer, so recording needs no lock. A worker thread of JobSystemThreadPool
//!   takes the buffer of its index when it starts. Any other thread (e.g. the thread that waits for a barrier) takes a
//!   free buffer the first time it records. When no buffer is free, its events are only counted (`num_dropped`).
//! - The ring buffers have a fixed capacity, allocated by `init`: recording never allocates. When a buffer is full, the
//!   oldest events are overwritten. The totals (busy time, idle time, number of jobs) count every event, also the
//!   overwritten ones.
//! - Times are nanoseconds since `init` or the last `reset`, read from the monotonic clock of `std.Io`
//!   (`Clock.awake`). A wait that started before a `reset` is recorded from time 0.
//! - Call `reset` only while no job runs, e.g. between two PhysicsSystem.update calls. A waiting worker thread does
//!   not write until the next job wakes it up.
//! - `stop` ends the recording. Read the events (`Thread.events`, `writeChromeTrace`, `writeSummary`) after `stop`,
//!   once no thread can still be in the middle of recording: stop the worker threads first
//!   (`JobSystemThreadPool.setNumThreads(0)` keeps the profiler). A worker thread can wake up at any time to record
//!   the end of a wait.
//!
//! `ZoltPerformanceTest -p` writes the events of every run as a Chrome trace (open it in chrome://tracing or
//! https://ui.perfetto.dev) and prints the totals.

const std = @import("std");
const Core = @import("Core.zig");

/// A timeline of jobs and idle time, one ring buffer per thread
pub const JobProfiler = struct {
    /// Allocator of the ring buffers
    allocator: std.mem.Allocator = undefined,

    /// Io of the clock
    io: std.Io = undefined,

    /// One entry per thread that can record, the first `num_workers` are for the worker threads
    threads: []Thread = &.{},

    /// The events of all threads (each thread owns a slice of `capacity` events)
    event_storage: []Event = &.{},

    /// Number of entries of `threads` that are reserved for worker threads (`registerThread`)
    num_workers: u32 = 0,

    /// Number of entries after the worker threads that other threads have taken
    num_other_threads: std.atomic.Value(u32) = .init(0),

    /// Number of events that were not recorded because no ring buffer was free for their thread
    num_dropped: std.atomic.Value(u64) = .init(0),

    /// Clock value (nanoseconds) of time 0
    epoch_ns: u64 = 0,

    /// False after `stop`: the hooks record nothing
    recording: std.atomic.Value(bool) = .init(false),

    /// Time of `stop` (nanoseconds since time 0)
    stop_ns: u64 = 0,

    /// Not initialized: recording does nothing
    pub const empty: JobProfiler = .{};

    /// Default number of events per thread before the oldest ones are overwritten
    pub const default_capacity: u32 = 32 * 1024;

    /// Default number of ring buffers for threads that are not worker threads
    pub const default_num_other_threads: u32 = 4;

    /// The kind of a span
    pub const Kind = enum(u8) {
        /// A job ran (the job body, Job.execute)
        job,

        /// The thread waited for work
        idle,
    };

    /// A span of time on one thread
    pub const Event = struct {
        /// Name of the job, or what the thread waited for
        name: []const u8,

        /// Start time, nanoseconds since time 0
        start_ns: u64,

        /// End time, nanoseconds since time 0
        end_ns: u64,

        /// Index of the thread in `threads`
        thread_index: u32,

        /// Job or idle
        kind: Kind,
    };

    /// The ring buffer and the totals of one thread
    pub const Thread = struct {
        /// The thread that owns this entry (0: free, a thread id is never 0). Other threads read it to find their
        /// entry, so it does not share a cache line with the fields below, which the owner writes.
        owner: std.atomic.Value(std.Thread.Id) align(Core.cache_line_size) = .init(0),

        /// Ring buffer of events, `num_events % buffer.len` is the next one to write
        buffer: []Event align(Core.cache_line_size) = &.{},

        /// Number of events recorded, including the overwritten ones
        num_events: u64 = 0,

        /// Number of jobs and their total time
        num_jobs: u64 = 0,
        busy_ns: u64 = 0,

        /// Total time spent waiting for work
        idle_ns: u64 = 0,

        /// The events in the order they were recorded: `first` holds the older ones (only after the buffer wrapped
        /// around), `second` the newer ones
        pub fn events(self: *const Thread) struct { first: []const Event, second: []const Event } {
            const capacity = self.buffer.len;
            if (self.num_events <= capacity)
                return .{ .first = &.{}, .second = self.buffer[0..@intCast(self.num_events)] };
            const next: usize = @intCast(self.num_events % capacity);
            return .{ .first = self.buffer[next..], .second = self.buffer[0..next] };
        }

        /// Number of events that were overwritten
        pub fn numOverwritten(self: *const Thread) u64 {
            return self.num_events -| self.buffer.len;
        }

        fn record(self: *Thread, event: Event) void {
            const duration = event.end_ns -| event.start_ns;
            switch (event.kind) {
                .job => {
                    self.num_jobs += 1;
                    self.busy_ns += duration;
                },
                .idle => self.idle_ns += duration,
            }
            self.buffer[@intCast(self.num_events % self.buffer.len)] = event;
            self.num_events += 1;
        }

        fn clear(self: *Thread) void {
            self.num_events = 0;
            self.num_jobs = 0;
            self.busy_ns = 0;
            self.idle_ns = 0;
        }
    };

    /// Allocate the ring buffers, `self` must be `empty`. Time 0 is now.
    /// opts.num_workers: number of worker threads, which call `registerThread` with their index
    /// opts.num_other_threads: number of ring buffers for other threads (e.g. the threads that wait for barriers)
    /// opts.capacity: number of events per thread before the oldest ones are overwritten (at least 1)
    pub fn init(self: *JobProfiler, allocator: std.mem.Allocator, io: std.Io, opts: struct { num_workers: u32 = 0, num_other_threads: u32 = default_num_other_threads, capacity: u32 = default_capacity }) error{OutOfMemory}!void {
        std.debug.assert(self.threads.len == 0); // Already initialized?
        std.debug.assert(opts.capacity > 0);
        const num_threads = @as(usize, opts.num_workers) + opts.num_other_threads;
        const threads = try allocator.alloc(Thread, num_threads);
        errdefer allocator.free(threads);
        const event_storage = try allocator.alloc(Event, num_threads * opts.capacity);
        for (threads, 0..) |*thread, i|
            thread.* = .{ .buffer = event_storage[i * opts.capacity ..][0..opts.capacity] };
        self.* = .{
            .allocator = allocator,
            .io = io,
            .threads = threads,
            .event_storage = event_storage,
            .num_workers = opts.num_workers,
        };
        self.reset();
    }

    /// Free the ring buffers, the profiler is `empty` again
    pub fn deinit(self: *JobProfiler) void {
        if (self.threads.len != 0) {
            self.allocator.free(self.event_storage);
            self.allocator.free(self.threads);
        }
        self.* = .empty;
    }

    /// True after `init`
    pub fn isInitialized(self: *const JobProfiler) bool {
        return self.threads.len != 0;
    }

    /// Clear all events and totals and start the time at 0. The threads keep their ring buffers.
    pub fn reset(self: *JobProfiler) void {
        if (!self.isInitialized())
            return;
        for (self.threads) |*thread|
            thread.clear();
        self.num_dropped.store(0, .monotonic);
        self.epoch_ns = self.clock();
        self.stop_ns = 0;
        self.recording.store(true, .release);
    }

    /// End the recording: the hooks record nothing until the next `reset`. The time since time 0 is the wall time of
    /// `writeSummary`.
    pub fn stop(self: *JobProfiler) void {
        if (!self.isInitialized() or !self.recording.load(.acquire))
            return;
        self.stop_ns = self.now();
        self.recording.store(false, .release);
    }

    /// True between `reset` (or `init`) and `stop`
    pub fn isRecording(self: *const JobProfiler) bool {
        return self.recording.load(.acquire);
    }

    /// The time from time 0 to `stop` (to now while recording)
    pub fn wallTime(self: *const JobProfiler) u64 {
        return if (self.isRecording()) self.now() else self.stop_ns;
    }

    /// Nanoseconds since time 0 (0 when not initialized)
    pub fn now(self: *const JobProfiler) u64 {
        if (!self.isInitialized())
            return 0;
        return self.clock() -| self.epoch_ns;
    }

    /// The monotonic clock in nanoseconds
    fn clock(self: *const JobProfiler) u64 {
        return std.math.cast(u64, std.Io.Clock.awake.now(self.io).nanoseconds) orelse 0;
    }

    /// Called by worker thread `index` (0 <= index < num_workers) when it starts: its events go to entry `index`
    pub fn registerThread(self: *JobProfiler, index: u32) void {
        if (!self.isInitialized())
            return;
        std.debug.assert(index < self.num_workers);
        self.threads[index].owner.store(std.Thread.getCurrentId(), .release);
    }

    /// The entry of the calling thread. A thread that is not a registered worker thread takes a free entry the first
    /// time. Null when not initialized or when no entry is free.
    pub fn currentThread(self: *JobProfiler) ?*Thread {
        if (!self.isInitialized())
            return null;
        const id = std.Thread.getCurrentId();
        const num_used = self.num_workers + @min(self.num_other_threads.load(.acquire), self.threads.len - self.num_workers);
        for (self.threads[0..num_used]) |*thread| {
            if (thread.owner.load(.acquire) == id)
                return thread;
        }

        // Take a free entry (only this thread writes its owner, other threads only compare it with their own id)
        const other_index = self.num_other_threads.fetchAdd(1, .acq_rel);
        if (other_index >= self.threads.len - self.num_workers) {
            _ = self.num_other_threads.fetchSub(1, .acq_rel);
            return null;
        }
        const thread = &self.threads[self.num_workers + other_index];
        thread.owner.store(id, .release);
        return thread;
    }

    /// Start of a span: the clock value to pass to `end` (0 without a profiler)
    pub fn begin(profiler: ?*JobProfiler) u64 {
        const self = profiler orelse return 0;
        if (!self.isInitialized())
            return 0;
        return self.clock();
    }

    /// End of a span that started at `start_clock` (the result of `begin`): record it on the calling thread. A span
    /// that started before time 0 starts at 0.
    pub fn end(profiler: ?*JobProfiler, kind: Kind, name: []const u8, start_clock: u64) void {
        const self = profiler orelse return;
        if (!self.isInitialized() or !self.recording.load(.acquire))
            return;
        const end_clock = self.clock();
        const thread = self.currentThread() orelse {
            _ = self.num_dropped.fetchAdd(1, .monotonic);
            return;
        };
        thread.record(.{
            .name = name,
            .start_ns = start_clock -| self.epoch_ns,
            .end_ns = end_clock -| self.epoch_ns,
            .thread_index = @intCast((@intFromPtr(thread) - @intFromPtr(self.threads.ptr)) / @sizeOf(Thread)),
            .kind = kind,
        });
    }

    /// The entries that a thread has used: all worker threads, then the other threads that recorded
    pub fn usedThreads(self: *JobProfiler) []Thread {
        if (!self.isInitialized())
            return &.{};
        return self.threads[0 .. self.num_workers + @min(self.num_other_threads.load(.acquire), self.threads.len - self.num_workers)];
    }

    /// Name of entry `index` of `threads`: "Worker <n>" for a worker thread (Jolt's thread names), "Thread <n>" for
    /// another thread (n counts from 0)
    pub fn threadName(self: *const JobProfiler, index: usize, buffer: []u8) []const u8 {
        return if (index < self.num_workers)
            std.fmt.bufPrint(buffer, "Worker {d}", .{index}) catch "Worker"
        else
            std.fmt.bufPrint(buffer, "Thread {d}", .{index - self.num_workers}) catch "Thread";
    }

    /// Write the events in the Chrome trace event format (JSON, the "traceEvents" array of complete events "X", times
    /// in microseconds). `process_name` names the process in the viewer.
    pub fn writeChromeTrace(self: *JobProfiler, writer: *std.Io.Writer, process_name: []const u8) std.Io.Writer.Error!void {
        try writer.writeAll("{\"traceEvents\":[\n");
        try writer.writeAll("{\"name\":\"process_name\",\"ph\":\"M\",\"pid\":1,\"tid\":0,\"args\":{\"name\":");
        try std.json.Stringify.encodeJsonString(process_name, .{}, writer);
        try writer.writeAll("}}");
        var name_buffer: [32]u8 = undefined;
        for (self.usedThreads(), 0..) |*thread, index| {
            try writer.print(",\n{{\"name\":\"thread_name\",\"ph\":\"M\",\"pid\":1,\"tid\":{d},\"args\":{{\"name\":\"{s}\"}}}}", .{ index, self.threadName(index, &name_buffer) });
            try writer.print(",\n{{\"name\":\"thread_sort_index\",\"ph\":\"M\",\"pid\":1,\"tid\":{d},\"args\":{{\"sort_index\":{d}}}}}", .{ index, index });
            const parts = thread.events();
            for ([_][]const Event{ parts.first, parts.second }) |part| {
                for (part) |event| {
                    try writer.writeAll(",\n{\"name\":");
                    try std.json.Stringify.encodeJsonString(event.name, .{}, writer);
                    try writer.print(",\"cat\":\"{s}\",\"ph\":\"X\",\"pid\":1,\"tid\":{d},\"ts\":{f},\"dur\":{f}}}", .{
                        @tagName(event.kind),
                        event.thread_index,
                        Microseconds{ .ns = event.start_ns },
                        Microseconds{ .ns = event.end_ns -| event.start_ns },
                    });
                }
            }
        }
        try writer.writeAll("\n],\"displayTimeUnit\":\"ms\"}\n");
    }

    /// Write one line with the totals: the time from time 0 to `stop` (wall), then per thread the time in jobs (busy),
    /// the time waiting for work (idle) and the number of jobs
    pub fn writeSummary(self: *JobProfiler, writer: *std.Io.Writer) std.Io.Writer.Error!void {
        try writer.print("Job profile: wall {f} ms", .{Milliseconds{ .ns = self.wallTime() }});
        var name_buffer: [32]u8 = undefined;
        for (self.usedThreads(), 0..) |*thread, index| {
            try writer.print(", {s}: busy {f} ms idle {f} ms jobs {d}", .{ self.threadName(index, &name_buffer), Milliseconds{ .ns = thread.busy_ns }, Milliseconds{ .ns = thread.idle_ns }, thread.num_jobs });
        }
        var num_overwritten: u64 = 0;
        for (self.usedThreads()) |*thread|
            num_overwritten += thread.numOverwritten();
        if (num_overwritten != 0)
            try writer.print(", {d} oldest events not in the trace", .{num_overwritten});
        const num_dropped = self.num_dropped.load(.monotonic);
        if (num_dropped != 0)
            try writer.print(", {d} events of threads without a ring buffer", .{num_dropped});
    }

    /// Nanoseconds printed as microseconds with 3 decimals
    const Microseconds = struct {
        ns: u64,

        pub fn format(self: Microseconds, writer: *std.Io.Writer) std.Io.Writer.Error!void {
            try writer.print("{d}.{d:0>3}", .{ self.ns / 1000, self.ns % 1000 });
        }
    };

    /// Nanoseconds printed as milliseconds with 3 decimals
    const Milliseconds = struct {
        ns: u64,

        pub fn format(self: Milliseconds, writer: *std.Io.Writer) std.Io.Writer.Error!void {
            try writer.print("{d}.{d:0>3}", .{ self.ns / std.time.ns_per_ms, self.ns % std.time.ns_per_ms / std.time.ns_per_us });
        }
    };
};

test "JobProfiler: events of one thread, wraparound and totals" {
    var profiler: JobProfiler = .empty;
    try profiler.init(std.testing.allocator, std.testing.io, .{ .num_workers = 0, .num_other_threads = 1, .capacity = 4 });
    defer profiler.deinit();

    const thread = profiler.currentThread().?;
    try std.testing.expectEqual(@as(usize, 1), profiler.usedThreads().len);
    try std.testing.expectEqual(thread, profiler.currentThread().?); // The same entry on the next call

    // Record 6 spans with known times (record directly, the clock is not under test here)
    for (0..6) |i| {
        thread.record(.{ .name = if (i % 2 == 0) "Job" else "Idle", .start_ns = 10 * i, .end_ns = 10 * i + 3, .thread_index = 0, .kind = if (i % 2 == 0) .job else .idle });
    }
    try std.testing.expectEqual(@as(u64, 6), thread.num_events);
    try std.testing.expectEqual(@as(u64, 3), thread.num_jobs);
    try std.testing.expectEqual(@as(u64, 9), thread.busy_ns);
    try std.testing.expectEqual(@as(u64, 9), thread.idle_ns);
    try std.testing.expectEqual(@as(u64, 2), thread.numOverwritten());

    // The ring keeps the last 4 events, oldest first
    const parts = thread.events();
    try std.testing.expectEqual(@as(usize, 4), parts.first.len + parts.second.len);
    var expected_start: u64 = 20;
    for ([_][]const JobProfiler.Event{ parts.first, parts.second }) |part| {
        for (part) |event| {
            try std.testing.expectEqual(expected_start, event.start_ns);
            expected_start += 10;
        }
    }

    // Reset clears the events and the totals, the thread keeps its entry
    profiler.reset();
    try std.testing.expectEqual(@as(u64, 0), thread.num_events);
    try std.testing.expectEqual(@as(u64, 0), thread.busy_ns);
    try std.testing.expectEqual(@as(usize, 0), thread.events().second.len);
    try std.testing.expectEqual(thread, profiler.currentThread().?);

    // begin / end record a span on the calling thread
    const start = JobProfiler.begin(&profiler);
    JobProfiler.end(&profiler, .job, "Measured", start);
    try std.testing.expectEqual(@as(u64, 1), thread.num_jobs);
    const event = thread.events().second[0];
    try std.testing.expectEqualStrings("Measured", event.name);
    try std.testing.expect(event.end_ns >= event.start_ns);

    // After stop nothing is recorded and the wall time stays, until the next reset
    try std.testing.expect(profiler.isRecording());
    profiler.stop();
    try std.testing.expect(!profiler.isRecording());
    const wall = profiler.wallTime();
    JobProfiler.end(&profiler, .job, "Ignored", JobProfiler.begin(&profiler));
    try std.testing.expectEqual(@as(u64, 1), thread.num_events);
    try std.testing.expectEqual(wall, profiler.wallTime());
    try std.testing.expectEqual(@as(u64, 0), profiler.num_dropped.load(.monotonic));
    profiler.reset();
    try std.testing.expect(profiler.isRecording());
}

test "JobProfiler: one ring buffer per thread, workers by index" {
    var profiler: JobProfiler = .empty;
    try profiler.init(std.testing.allocator, std.testing.io, .{ .num_workers = 2, .num_other_threads = 2, .capacity = 64 });
    defer profiler.deinit();

    const Worker = struct {
        fn run(p: *JobProfiler, index: u32, num_jobs: u32) void {
            p.registerThread(index);
            for (0..num_jobs) |_| {
                const start = JobProfiler.begin(p);
                JobProfiler.end(p, .job, "Work", start);
            }
            const start = JobProfiler.begin(p);
            JobProfiler.end(p, .idle, "Wait", start);
        }
        fn other(p: *JobProfiler) void {
            const start = JobProfiler.begin(p);
            JobProfiler.end(p, .job, "Other", start);
        }
    };

    // Two workers and three other threads (one more than there are entries for)
    var threads: [5]std.Thread = undefined;
    threads[0] = try std.Thread.spawn(.{}, Worker.run, .{ &profiler, 0, 10 });
    threads[1] = try std.Thread.spawn(.{}, Worker.run, .{ &profiler, 1, 100 }); // Wraps around (capacity 64)
    for (threads[2..]) |*t| {
        t.* = try std.Thread.spawn(.{}, Worker.other, .{&profiler});
        t.join(); // One at a time: the third one finds no free entry
    }
    threads[0].join();
    threads[1].join();

    const used = profiler.usedThreads();
    try std.testing.expectEqual(@as(usize, 4), used.len);
    try std.testing.expectEqual(@as(u64, 10), used[0].num_jobs);
    try std.testing.expectEqual(@as(u64, 11), used[0].num_events);
    try std.testing.expectEqual(@as(u64, 100), used[1].num_jobs);
    try std.testing.expectEqual(@as(u64, 101 - 64), used[1].numOverwritten());
    for (used[2..]) |*thread|
        try std.testing.expectEqual(@as(u64, 1), thread.num_jobs);
    try std.testing.expectEqual(@as(u64, 1), profiler.num_dropped.load(.monotonic));

    // Every event carries the index of its thread
    for (used, 0..) |*thread, index| {
        const parts = thread.events();
        for ([_][]const JobProfiler.Event{ parts.first, parts.second }) |part| {
            for (part) |event|
                try std.testing.expectEqual(@as(u32, @intCast(index)), event.thread_index);
        }
    }

    // The newest event of worker 1 is its idle span
    const parts = used[1].events();
    try std.testing.expectEqualStrings("Wait", parts.second[parts.second.len - 1].name);
    try std.testing.expectEqual(JobProfiler.Kind.idle, parts.second[parts.second.len - 1].kind);

    var name_buffer: [32]u8 = undefined;
    try std.testing.expectEqualStrings("Worker 1", profiler.threadName(1, &name_buffer));
    try std.testing.expectEqualStrings("Thread 0", profiler.threadName(2, &name_buffer));
}

test "JobProfiler: Chrome trace is valid JSON, summary line" {
    var profiler: JobProfiler = .empty;
    try profiler.init(std.testing.allocator, std.testing.io, .{ .num_workers = 1, .num_other_threads = 1, .capacity = 2 });
    defer profiler.deinit();
    profiler.registerThread(0); // The test thread acts as worker 0
    const thread = profiler.currentThread().?;
    thread.record(.{ .name = "A \"quoted\" job\n", .start_ns = 1500, .end_ns = 2750, .thread_index = 0, .kind = .job });
    thread.record(.{ .name = "Idle", .start_ns = 3000, .end_ns = 4000, .thread_index = 0, .kind = .idle });
    thread.record(.{ .name = "Last", .start_ns = 5000, .end_ns = 1_234_567, .thread_index = 0, .kind = .job });

    var trace: std.Io.Writer.Allocating = .init(std.testing.allocator);
    defer trace.deinit();
    try profiler.writeChromeTrace(&trace.writer, "Test \"process\"");

    const parsed = try std.json.parseFromSlice(std.json.Value, std.testing.allocator, trace.written(), .{});
    defer parsed.deinit();
    const events = parsed.value.object.get("traceEvents").?.array.items;
    // process_name + thread_name and sort index for 1 thread + the 2 events left in the ring
    try std.testing.expectEqual(@as(usize, 5), events.len);
    try std.testing.expectEqualStrings("Test \"process\"", events[0].object.get("args").?.object.get("name").?.string);
    try std.testing.expectEqualStrings("Worker 0", events[1].object.get("args").?.object.get("name").?.string);
    try std.testing.expectEqualStrings("Idle", events[3].object.get("name").?.string);
    try std.testing.expectEqualStrings("idle", events[3].object.get("cat").?.string);
    const last = events[4].object;
    try std.testing.expectEqualStrings("Last", last.get("name").?.string);
    try std.testing.expectEqualStrings("X", last.get("ph").?.string);
    try std.testing.expectApproxEqAbs(@as(f64, 5.0), last.get("ts").?.float, 1e-9);
    try std.testing.expectApproxEqAbs(@as(f64, 1229.567), last.get("dur").?.float, 1e-9);

    var summary: std.Io.Writer.Allocating = .init(std.testing.allocator);
    defer summary.deinit();
    try profiler.writeSummary(&summary.writer);
    try std.testing.expect(std.mem.startsWith(u8, summary.written(), "Job profile: wall "));
    try std.testing.expect(std.mem.indexOf(u8, summary.written(), ", Worker 0: busy 1.230 ms idle 0.001 ms jobs 2, 1 oldest events not in the trace") != null);
}

test "JobProfiler: not initialized records nothing" {
    var profiler: JobProfiler = .empty;
    try std.testing.expectEqual(@as(u64, 0), JobProfiler.begin(&profiler));
    JobProfiler.end(&profiler, .job, "Nothing", 0);
    JobProfiler.end(null, .job, "Nothing", 0);
    try std.testing.expectEqual(@as(?*JobProfiler.Thread, null), profiler.currentThread());
    try std.testing.expectEqual(@as(usize, 0), profiler.usedThreads().len);
    try std.testing.expectEqual(@as(u64, 0), profiler.num_dropped.load(.monotonic));
    profiler.reset();
    profiler.deinit();
}

test "JobProfiler: allocation failures" {
    const Test = struct {
        fn run(allocator: std.mem.Allocator) !void {
            var profiler: JobProfiler = .empty;
            try profiler.init(allocator, std.testing.io, .{ .num_workers = 3 });
            profiler.deinit();
        }
    };
    try std.testing.checkAllAllocationFailures(std.testing.allocator, Test.run, .{});
}
