//! Port of: Jolt/Core/TickCounter.h, Jolt/Core/TickCounter.cpp
//! Status: complete
//!
//! Jolt reads the CPU's cycle counter on x86 (__rdtsc), ARM64 (cntvct_el0) and LoongArch (rdtime), uses
//! QueryPerformanceCounter on Windows ARM / UWP and returns 0 on other CPUs. Zolt uses the same counters and
//! falls back to the OS's monotonic clock (in nanoseconds) on other CPUs, or 0 when there is no OS.

const std = @import("std");
const builtin = @import("builtin");

/// Functionality to get the processors cycle counter
pub fn getProcessorTickCount() u64 {
    switch (builtin.cpu.arch) {
        .x86, .x86_64 => {
            // __rdtsc()
            var low: u32 = undefined;
            var high: u32 = undefined;
            asm volatile ("rdtsc"
                : [low] "={eax}" (low),
                  [high] "={edx}" (high),
            );
            return (@as(u64, high) << 32) | low;
        },
        .aarch64, .aarch64_be => {
            return asm volatile ("mrs %[val], cntvct_el0"
                : [val] "=r" (-> u64),
            );
        },
        .loongarch64 => {
            // __rdtime_d()
            return asm volatile ("rdtime.d %[val], $zero"
                : [val] "=r" (-> u64),
            );
        },
        else => return monotonicClockTicks(),
    }
}

/// Fallback for CPUs without a cycle counter that is accessible from user mode: the monotonic clock of the OS in nanoseconds
fn monotonicClockTicks() u64 {
    switch (builtin.os.tag) {
        .windows => {
            // Like Jolt on Windows ARM / UWP
            var count: std.os.windows.LARGE_INTEGER = undefined;
            _ = std.os.windows.ntdll.RtlQueryPerformanceCounter(&count);
            return @bitCast(count);
        },
        .wasi => {
            var ns: std.os.wasi.timestamp_t = undefined;
            if (std.os.wasi.clock_time_get(.MONOTONIC, 1, &ns) != .SUCCESS)
                return 0;
            return ns;
        },
        else => {
            const posix = std.posix;
            if (!@hasDecl(posix.system, "clock_gettime"))
                return 0; // Not supported
            var ts: posix.timespec = undefined;
            if (posix.errno(posix.system.clock_gettime(posix.CLOCK.MONOTONIC, &ts)) != .SUCCESS)
                return 0;
            return @as(u64, @intCast(ts.sec)) * std.time.ns_per_s + @as(u64, @intCast(ts.nsec));
        },
    }
}

test "getProcessorTickCount does not go backwards" {
    const t1 = getProcessorTickCount();
    var sum: u64 = 0;
    for (0..1000) |i| sum +%= i * i;
    std.mem.doNotOptimizeAway(sum);
    const t2 = getProcessorTickCount();
    try std.testing.expect(t2 >= t1);
}
