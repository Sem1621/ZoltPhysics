//! Port of: Jolt/Core/IssueReporting.h, Jolt/Core/IssueReporting.cpp
//! Status: complete
//!
//! Docs/Zolt/RemainingFeaturesArchitecture.md R8.
//!
//! - `Trace` is `std.log.scoped(.zolt)` (porting guide section 5), JPH_ASSERT on an invariant is `std.debug.assert`.
//! - `AssertFailed` (Jolt's hook, a global function pointer that returns true to break) is ported for the asserts that
//!   a test expects to fail. They are release-reachable asserts (the porting guide's rows 2 and 3 of `JPH_ASSERT`), and
//!   their sites call `assertRecoverable` / `assertFailed` instead of `std.debug.assert` / `@panic`. With asserts
//!   enabled a failure asks the installed handler; without a handler, or when the handler returns true, it panics like
//!   before. A handler that returns false lets the code continue on Jolt's release path, which is what Jolt's
//!   `ExpectAssert` relies on.
//! - The handler pointer is the one global that Phase 9 adds to the library (an explicit exception to "no globals"):
//!   Jolt's `AssertFailed` is global, asserts fire on any thread (job threads included), and the handler must be
//!   found from code that has no system pointer (shapes, skeletons). It is `void` without asserts (ReleaseFast), so it
//!   cannot influence results, and it is an atomic pointer: install and remove handlers only while no physics update
//!   runs (`ExpectAssert` does this from the test thread).
//! - `ExpectAssert` (Jolt's UnitTests/UnitTestFramework.h) is a test helper here, like `FailingTempAllocator`, so that
//!   the inline tests of the library can use it too. ZoltTests/UnitTestFramework.zig re-exports it.
//! - `CountAsserts` (Zolt addition) counts the failed asserts instead of expecting a number. The parity tests of Jolt's
//!   release behavior compare its count with the count of the C++ reference (ZoltParity/ParityHooks.zig).
//! - Core.zig re-exports `assertRecoverable` and `assertFailed`, so a converted assert is one line at its site:
//!   `if (Core.enable_asserts) Core.assertRecoverable(ok, "message", @src());`.

const std = @import("std");
const Core = @import("Core.zig");

/// The receiver of a failed assert (Jolt: AssertFailedFunction). A pattern A interface whose identity is its address,
/// because it is stored in an atomic (porting guide section 6, exception to pattern B).
pub const AssertHandler = struct {
    /// Called when an assert fails. Return true to break (Zolt panics), false to continue like Jolt's release build.
    assertFailed: *const fn (self: *AssertHandler, message: []const u8, src: std.builtin.SourceLocation) bool,
};

/// AssertFailed: the installed handler, null for Jolt's default (DummyAssertFailed: trace and break)
var assert_failed: if (Core.enable_asserts) std.atomic.Value(?*AssertHandler) else void = if (Core.enable_asserts) .init(null) else {};

/// Install `handler` (null: the default, panic) and return the previous one. Without asserts there is nothing to
/// install and it returns null.
pub fn setAssertHandler(handler: ?*AssertHandler) ?*AssertHandler {
    if (!Core.enable_asserts) return null;
    return assert_failed.swap(handler, .acq_rel);
}

/// JPH_ASSERT(ok, message) at a site that Jolt's release build continues after. Guard the call with
/// `if (Core.enable_asserts)` like a `std.debug.assert` of such a site, so the condition is not evaluated in ReleaseFast.
/// `inline` like `std.debug.assert`: without asserts nothing remains at the call site (no call, no `@src()` data).
pub inline fn assertRecoverable(ok: bool, comptime message: []const u8, src: std.builtin.SourceLocation) void {
    if (!Core.enable_asserts) return;
    if (!ok) assertFailed(message, src);
}

/// JPH_ASSERT(false, message) on a path that Jolt's release build continues on: returns when the installed handler
/// accepts the failure, panics otherwise. `inline` for the same reason as `assertRecoverable` (the slow path is
/// `assertFailedSlow`).
pub inline fn assertFailed(comptime message: []const u8, src: std.builtin.SourceLocation) void {
    if (!Core.enable_asserts) return;
    assertFailedSlow(message, src);
}

fn assertFailedSlow(message: []const u8, src: std.builtin.SourceLocation) void {
    if (assert_failed.load(.acquire)) |handler| {
        if (!handler.assertFailed(handler, message, src))
            return; // Continue like Jolt's release build
    }
    std.debug.panic("{s}:{d}: {s}", .{ src.file, src.line, message });
}

/// Helper class that expects a number of asserts to fail (Jolt: ExpectAssert, a test helper). Initialize in place,
/// it must not move until `finish` / `restore`:
/// ```
/// var expect_assert: ExpectAssert = undefined;
/// expect_assert.init(1); // JPH_IF_ENABLE_ASSERTS(ExpectAssert expect_assert(1);)
/// errdefer expect_assert.restore();
/// errors = try c.simulateSingleStep();
/// try expect_assert.finish(); // ~ExpectAssert: CHECK(sCount == 0)
/// ```
/// Without asserts it does nothing and `finish` always succeeds, like Jolt's `JPH_IF_ENABLE_ASSERTS`.
pub const ExpectAssert = struct {
    handler: AssertHandler = .{ .assertFailed = onAssertFailed },
    /// Keeps track of number of asserts that are expected (Jolt: the static atomic sCount)
    count: std.atomic.Value(i32) = .init(0),
    /// Previous assert handler
    previous: ?*AssertHandler = null,
    installed: bool = false,

    /// Expect `count` asserts
    pub fn init(self: *ExpectAssert, count: i32) void {
        self.* = .{ .count = .init(count) };
        if (!Core.enable_asserts) return;
        self.previous = setAssertHandler(&self.handler);
        self.installed = true;
    }

    /// Put the previous handler back (idempotent, for `errdefer`)
    pub fn restore(self: *ExpectAssert) void {
        if (!self.installed) return;
        _ = setAssertHandler(self.previous);
        self.installed = false;
    }

    /// Put the previous handler back and verify that the expected number of asserts were triggered
    pub fn finish(self: *ExpectAssert) error{TestUnexpectedResult}!void {
        self.restore();
        if (Core.enable_asserts and self.count.load(.seq_cst) != 0)
            return error.TestUnexpectedResult;
    }

    fn onAssertFailed(handler: *AssertHandler, message: []const u8, src: std.builtin.SourceLocation) bool {
        _ = .{ message, src };
        const self: *ExpectAssert = @fieldParentPtr("handler", handler);
        _ = self.count.fetchSub(1, .seq_cst); // --sCount
        return false;
    }
};

/// A Zolt test helper next to ExpectAssert: it counts the failed asserts and lets the code continue, like Jolt's
/// release build. A test of Jolt's release behavior whose assert count is only known on the C++ side (a parity test)
/// compares this count with the count of the C++ reference (ZoltParity/ParityHooks.h). Initialize in place, it must not
/// move until `restore`. Without asserts it counts nothing.
pub const CountAsserts = struct {
    handler: AssertHandler = .{ .assertFailed = onAssertFailed },
    /// Number of asserts that failed since `init`
    count: std.atomic.Value(u32) = .init(0),
    /// Previous assert handler
    previous: ?*AssertHandler = null,
    installed: bool = false,

    /// Install the counting handler
    pub fn init(self: *CountAsserts) void {
        self.* = .{};
        if (!Core.enable_asserts) return;
        self.previous = setAssertHandler(&self.handler);
        self.installed = true;
    }

    /// Number of asserts that failed since `init`
    pub fn get(self: *const CountAsserts) u32 {
        return self.count.load(.seq_cst);
    }

    /// Put the previous handler back (idempotent)
    pub fn restore(self: *CountAsserts) void {
        if (!self.installed) return;
        _ = setAssertHandler(self.previous);
        self.installed = false;
    }

    fn onAssertFailed(handler: *AssertHandler, message: []const u8, src: std.builtin.SourceLocation) bool {
        _ = .{ message, src };
        const self: *CountAsserts = @fieldParentPtr("handler", handler);
        _ = self.count.fetchAdd(1, .seq_cst);
        return false;
    }
};

test "IssueReporting: ExpectAssert accepts the expected asserts and restores the handler" {
    var expect_assert: ExpectAssert = undefined;
    expect_assert.init(2);
    errdefer expect_assert.restore();
    if (Core.enable_asserts) {
        assertRecoverable(false, "first", @src());
        assertRecoverable(true, "not failed", @src());
        assertFailed("second", @src());
    }
    try expect_assert.finish();
    if (Core.enable_asserts) try std.testing.expect(assert_failed.load(.acquire) == null);

    // Too few asserts is a test failure
    var too_few: ExpectAssert = undefined;
    too_few.init(1);
    errdefer too_few.restore();
    if (Core.enable_asserts)
        try std.testing.expectError(error.TestUnexpectedResult, too_few.finish())
    else
        try too_few.finish();
}

test "IssueReporting: CountAsserts counts and restores the handler" {
    var expect_assert: ExpectAssert = undefined;
    expect_assert.init(1);
    errdefer expect_assert.restore();

    // A counter on top of another handler: it gets the asserts until it is restored
    var counter: CountAsserts = undefined;
    counter.init();
    errdefer counter.restore();
    if (Core.enable_asserts) {
        assertFailed("counted", @src());
        assertRecoverable(false, "counted", @src());
        assertRecoverable(true, "not failed", @src());
    }
    try std.testing.expectEqual(@as(u32, if (Core.enable_asserts) 2 else 0), counter.get());
    counter.restore();
    counter.restore();

    // The previous handler is back
    if (Core.enable_asserts) assertFailed("expected", @src());
    try expect_assert.finish();
}

test "IssueReporting: asserts from several threads reach the handler" {
    if (!Core.enable_asserts) return error.SkipZigTest;
    const num_threads = 4;
    const per_thread = 100;
    var expect_assert: ExpectAssert = undefined;
    expect_assert.init(num_threads * per_thread);
    errdefer expect_assert.restore();
    var threads: [num_threads]std.Thread = undefined;
    for (&threads) |*t| t.* = try std.Thread.spawn(.{}, struct {
        fn run() void {
            for (0..per_thread) |_| assertFailed("from a thread", @src());
        }
    }.run, .{});
    for (threads) |t| t.join();
    try expect_assert.finish();
}
