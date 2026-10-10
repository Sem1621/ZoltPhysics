//! Port of: Jolt/Physics/Body/BodyAccess.h
//! Status: complete
//!
//! Jolt compiles this file only with JPH_ENABLE_ASSERTS: it is a debug system that detects race conditions during the
//! simulation. Zolt keeps the declarations in every build, but the per thread state is typed `void` when asserts are off
//! (SimulationArchitecture D3), so it provably cannot influence release results. Callers check the rights inside
//! `if (Core.enable_asserts)` (Jolt's `JPH_ASSERT(BodyAccess::sCheckRights(...))`), the Zolt helpers
//! `assertVelocityAccess` / `assertPositionAccess` do exactly that:
//! ```zig
//! if (Core.enable_asserts) std.debug.assert(BodyAccess.checkRights(BodyAccess.velocityAccess().*, .read));
//! BodyAccess.assertVelocityAccess(.read); // the same
//! ```
//! `Grant` is Jolt's RAII class: `var grant: BodyAccess.Grant = .init(.read_write, .read); defer grant.deinit();`.

const std = @import("std");
const Core = @import("../../Core/Core.zig");

pub const BodyAccess = struct {
    /// Access rules, used to detect race conditions during simulation (EAccess)
    pub const Access = enum(u8) {
        none = 0,
        read = 1,
        read_write = 3,
    };

    /// The type of the per thread state: `Access` with asserts, `void` without
    pub const AccessState = if (Core.enable_asserts) Access else void;

    /// static thread_local EAccess sAccess in sVelocityAccess()
    threadlocal var velocity_access_state: AccessState = if (Core.enable_asserts) .read_write else {};

    /// static thread_local EAccess sAccess in sPositionAccess()
    threadlocal var position_access_state: AccessState = if (Core.enable_asserts) .read_write else {};

    /// Grant a scope specific access rights on the current thread
    pub const Grant = struct {
        pub fn init(velocity: Access, position: Access) Grant {
            if (Core.enable_asserts) {
                const velocity_state = velocityAccess();
                const position_state = positionAccess();

                std.debug.assert(velocity_state.* == .read_write);
                std.debug.assert(position_state.* == .read_write);

                velocity_state.* = velocity;
                position_state.* = position;
            }
            return .{};
        }

        pub fn deinit(self: *Grant) void {
            _ = self;
            if (Core.enable_asserts) {
                velocityAccess().* = .read_write;
                positionAccess().* = .read_write;
            }
        }
    };

    /// Check if we have permission
    pub fn checkRights(rights: Access, desired_rights: Access) bool {
        return (@intFromEnum(rights) & @intFromEnum(desired_rights)) == @intFromEnum(desired_rights);
    }

    /// Access to read/write velocities (a pointer to `void` when asserts are off)
    pub fn velocityAccess() *AccessState {
        return &velocity_access_state;
    }

    /// Access to read/write positions (a pointer to `void` when asserts are off)
    pub fn positionAccess() *AccessState {
        return &position_access_state;
    }

    /// Zolt helper for Jolt's `JPH_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), inDesiredRights))`
    pub fn assertVelocityAccess(desired_rights: Access) void {
        if (Core.enable_asserts) std.debug.assert(checkRights(velocityAccess().*, desired_rights));
    }

    /// Zolt helper for Jolt's `JPH_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), inDesiredRights))`
    pub fn assertPositionAccess(desired_rights: Access) void {
        if (Core.enable_asserts) std.debug.assert(checkRights(positionAccess().*, desired_rights));
    }
};

test "BodyAccess rights and grants" {
    const expect = std.testing.expect;
    try expect(BodyAccess.checkRights(.read_write, .read));
    try expect(BodyAccess.checkRights(.read_write, .read_write));
    try expect(BodyAccess.checkRights(.read, .read));
    try expect(!BodyAccess.checkRights(.read, .read_write));
    try expect(!BodyAccess.checkRights(.none, .read));
    try expect(BodyAccess.checkRights(.none, .none));

    if (Core.enable_asserts) {
        try expect(BodyAccess.velocityAccess().* == .read_write);
        {
            var grant: BodyAccess.Grant = .init(.none, .read);
            defer grant.deinit();
            try expect(BodyAccess.velocityAccess().* == .none);
            try expect(BodyAccess.positionAccess().* == .read);

            // The state is per thread: another thread still has all rights
            const Other = struct {
                fn run(ok: *bool) void {
                    ok.* = BodyAccess.velocityAccess().* == .read_write and BodyAccess.positionAccess().* == .read_write;
                }
            };
            var ok = false;
            const thread = try std.Thread.spawn(.{}, Other.run, .{&ok});
            thread.join();
            try expect(ok);
        }
        try expect(BodyAccess.velocityAccess().* == .read_write);
        try expect(BodyAccess.positionAccess().* == .read_write);
    } else {
        try std.testing.expectEqual(void, BodyAccess.AccessState);
        var grant: BodyAccess.Grant = .init(.none, .none);
        grant.deinit();
    }
}
