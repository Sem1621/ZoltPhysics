//! Port of: UnitTests/Core/JobSystemTest.cpp

const std = @import("std");
const zolt = @import("zolt");
const fw = @import("../UnitTestFramework.zig");

const Color = zolt.Color;
const JobHandle = zolt.JobHandle;
const JobSystem = zolt.JobSystem;
const JobSystemThreadPool = zolt.JobSystemThreadPool;

test "TestJobSystemRunJobs" {
    // Create job system
    const max_jobs = 128;
    const max_barriers = 10;
    const max_threads = 10;
    var system: JobSystemThreadPool = .empty;
    try system.init(std.testing.allocator, std.testing.io, max_jobs, max_barriers, .{ .num_threads = max_threads });
    defer system.deinit();

    // Create array of zeros
    var values: [max_jobs]std.atomic.Value(u32) = undefined;
    for (&values) |*value|
        value.* = .init(0);

    // Create a barrier
    const barrier: *JobSystem.Barrier = system.createBarrier().?;

    // Create jobs that will increment all values
    const Job = struct {
        fn run(v: *[max_jobs]std.atomic.Value(u32), i: usize) void {
            _ = v[i].fetchAdd(1, .seq_cst);
        }
    };
    for (0..max_jobs) |i| {
        var handle = try system.createJob("JobTest", Color.red, .init(Job.run, .{ &values, i }), .{});
        defer handle.deinit();
        barrier.addJob(&handle);
    }

    // Wait for the barrier to complete
    system.waitForJobs(barrier);

    // Destroy our barrier
    system.destroyBarrier(barrier);

    // Test all values are 1
    for (&values) |*value|
        try fw.expectEqual(@as(u32, 1), value.load(.seq_cst));
}

test "TestJobSystemRunChain" {
    // Create job system
    const max_jobs = 128;
    const max_barriers = 10;
    var system: JobSystemThreadPool = .empty;
    try system.init(std.testing.allocator, std.testing.io, max_jobs, max_barriers, .{});
    defer system.deinit();

    // Create a barrier
    const barrier: *JobSystem.Barrier = system.createBarrier().?;

    // Counter that keeps track of order in which jobs ran
    var counter: std.atomic.Value(u32) = .init(1);

    // Create array of zeros
    var values: [max_jobs]std.atomic.Value(u32) = undefined;
    for (&values) |*value|
        value.* = .init(0);

    // Create jobs that will set sequence number
    var handles: [max_jobs]JobHandle = @splat(.empty);
    defer for (&handles) |*handle| handle.deinit();
    const Job = struct {
        fn run(v: *[max_jobs]std.atomic.Value(u32), c: *std.atomic.Value(u32), h: *[max_jobs]JobHandle, i: usize) void {
            // Set sequence number
            v[i].store(c.fetchAdd(1, .seq_cst), .seq_cst);

            // Start previous job
            if (i > 0)
                h[i - 1].removeDependency(.{});
        }
    };
    for (0..max_jobs) |i| {
        handles[i] = try system.createJob("JobTestChain", Color.red, .init(Job.run, .{ &values, &counter, &handles, i }), .{ .num_dependencies = 1 });

        barrier.addJob(&handles[i]);
    }

    // Start the last job
    handles[max_jobs - 1].removeDependency(.{});

    // Wait for the barrier to complete
    system.waitForJobs(barrier);

    // Destroy our barrier
    system.destroyBarrier(barrier);

    // Test jobs were executed in reverse order
    var i: usize = max_jobs;
    while (i > 0) {
        i -= 1;
        try fw.expectEqual(@as(u32, @intCast(max_jobs - i)), values[i].load(.seq_cst));
    }
}
