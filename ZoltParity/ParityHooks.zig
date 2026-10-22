//! The Zig side of ZoltParity/ParityHooks.h (Docs/Zolt/RemainingFeaturesArchitecture.md R1, R8).
//!
//! - The C++ reference counts the Jolt asserts that fail when it is built with `-Dreference_asserts=true`
//!   (ParityAssertFailed of ParityHooks.h). `assertCount` reads that count.
//! - `ReleaseAsserts` is for the parity tests of Jolt's release behavior: inputs that Jolt asserts on. Zolt's asserts
//!   at these sites are recoverable (R8). A `CountAsserts` handler counts them, and the code continues like Jolt's
//!   release build. With a reference that counts its asserts, every case must hit as many asserts as Jolt: this is
//!   ExpectAssert(n) with n the number of Jolt asserts. Without it, the test checks that its cases reach the asserts.
//! - Calling any function of ParityHooks.cpp links that file, so the hooks are installed before main even in a
//!   filtered parity run.

const std = @import("std");
const zolt = @import("zolt");

extern fn jolt_parity_assert_count() u32;
extern fn jolt_parity_reference_has_asserts() u32;
extern fn jolt_parity_hooks_installed_before_main() u32;
extern fn jolt_parity_reinstall_check() u32;
extern fn jolt_parity_test_assert_hook(out_before: *u32, out_after: *u32) void;
extern fn jolt_parity_zero_filling_check() u32;

/// The number of Jolt asserts that failed in this process (always 0 without `-Dreference_asserts=true`)
pub fn assertCount() u32 {
    return jolt_parity_assert_count();
}

/// True when the C++ reference is built with JPH_ENABLE_ASSERTS (`-Dreference_asserts=true`)
pub fn referenceHasAsserts() bool {
    return jolt_parity_reference_has_asserts() != 0;
}

/// Counts the asserts of both sides in a parity test of Jolt's release behavior. Initialize in place, it must not
/// move until `finish` / `deinit`:
/// ```
/// var asserts: ParityHooks.ReleaseAsserts = undefined;
/// asserts.init("VehicleDifferential release only");
/// defer asserts.deinit();
/// for (cases) |case| {
///     asserts.begin();
///     ... // the C++ call and the Zolt call of the case
///     asserts.end(case);
/// }
/// try asserts.finish();
/// ```
pub const ReleaseAsserts = struct {
    /// The handler that counts Zolt's asserts and lets the code continue
    zolt_asserts: zolt.IssueReporting.CountAsserts = undefined,
    name: []const u8,
    /// The counts of both sides at `begin`
    zolt_mark: u32 = 0,
    jolt_mark: u32 = 0,
    /// Number of Zolt asserts in all cases
    num_zolt_asserts: u32 = 0,
    /// Number of Jolt asserts in all cases (0 without `-Dreference_asserts=true`)
    num_jolt_asserts: u32 = 0,
    /// Number of cases in which Zolt and Jolt hit a different number of asserts
    mismatches: u32 = 0,

    /// Install the counting handler
    pub fn init(self: *ReleaseAsserts, name: []const u8) void {
        self.* = .{ .name = name };
        self.zolt_asserts.init();
    }

    /// Put the previous handler back (idempotent)
    pub fn deinit(self: *ReleaseAsserts) void {
        self.zolt_asserts.restore();
    }

    /// Start a case
    pub fn begin(self: *ReleaseAsserts) void {
        self.zolt_mark = self.zolt_asserts.get();
        self.jolt_mark = assertCount();
    }

    /// End a case: compare the asserts of both sides since `begin` when both sides count them
    pub fn end(self: *ReleaseAsserts, case: usize) void {
        const zolt_count = self.zolt_asserts.get() - self.zolt_mark;
        const jolt_count = assertCount() - self.jolt_mark;
        self.num_zolt_asserts += zolt_count;
        self.num_jolt_asserts += jolt_count;
        if (zolt.Core.enable_asserts and referenceHasAsserts() and zolt_count != jolt_count) {
            if (self.mismatches < 5)
                std.debug.print("{s}: case {d}: Zolt hit {d} asserts, Jolt {d}\n", .{ self.name, case, zolt_count, jolt_count });
            self.mismatches += 1;
        }
    }

    /// Fails when a case hit a different number of asserts than Jolt's, or when Zolt checks asserts and no case hit one
    /// (the inputs do not reach what the test is written for)
    pub fn finish(self: *ReleaseAsserts) !void {
        self.deinit();
        if (self.mismatches > 0) {
            std.debug.print("{s}: {d} cases hit a different number of asserts than Jolt's (Zolt {d}, Jolt {d} in all cases)\n", .{ self.name, self.mismatches, self.num_zolt_asserts, self.num_jolt_asserts });
            return error.TestUnexpectedResult;
        }
        if (zolt.Core.enable_asserts and self.num_zolt_asserts == 0) {
            std.debug.print("{s}: no case hit an assert\n", .{self.name});
            return error.TestUnexpectedResult;
        }
    }
};

test "ParityHooks: the hooks are installed before main" {
    try std.testing.expectEqual(@as(u32, 1), jolt_parity_hooks_installed_before_main());
}

test "ParityHooks: ParityInstallHooks installs the hooks again and is idempotent" {
    // The older reference files call RegisterDefaultAllocator(), which replaces the allocation hooks
    try std.testing.expectEqual(@as(u32, 1), jolt_parity_reinstall_check());
}

test "ParityHooks: the reference counts its asserts" {
    // With -Dreference_asserts=true the hook prints and counts a failed Jolt assert, without asserts nothing is counted
    var before: u32 = undefined;
    var after: u32 = undefined;
    jolt_parity_test_assert_hook(&before, &after);
    try std.testing.expectEqual(before + @intFromBool(referenceHasAsserts()), after);
    try std.testing.expectEqual(after, assertCount());
}

test "ParityHooks: the reference zero-fills its allocations" {
    try std.testing.expectEqual(@as(u32, 1), jolt_parity_zero_filling_check());
}

test "ParityHooks: ReleaseAsserts counts Zolt's asserts" {
    var asserts: ReleaseAsserts = undefined;
    asserts.init("ParityHooks test");
    defer asserts.deinit();
    asserts.begin();
    if (zolt.Core.enable_asserts) {
        zolt.Core.assertFailed("ParityHooks: the test of ReleaseAsserts (expected)", @src());

        // Jolt hits the same assert, when it checks its asserts
        var before: u32 = undefined;
        var after: u32 = undefined;
        jolt_parity_test_assert_hook(&before, &after);
    }
    asserts.end(0);
    try std.testing.expectEqual(@as(u32, @intFromBool(zolt.Core.enable_asserts)), asserts.num_zolt_asserts);
    try asserts.finish();
}
