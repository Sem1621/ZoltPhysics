//! Port of: Jolt/Physics/Vehicle/VehicleAntiRollBar.h, Jolt/Physics/Vehicle/VehicleAntiRollBar.cpp
//! Status: complete
//!
//! - JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleAntiRollBar):
//!   TODO(serialization) (ObjectStream is Phase 8, CharacterVehicleArchitecture E1; the binary state is ported).
//! - `VehicleAntiRollBars` (`Array<VehicleAntiRollBar>`) is `std.ArrayList(VehicleAntiRollBar)`.

const std = @import("std");
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;

/// An anti rollbar is a stiff spring that connects two wheels to reduce the amount of roll the vehicle makes in sharp corners
/// See: https://en.wikipedia.org/wiki/Anti-roll_bar
pub const VehicleAntiRollBar = extern struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleAntiRollBar)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "VehicleAntiRollBar",
        .kind = .non_virtual,
    });
    /// TODO(serialization): the attributes of Jolt's sCreateRTTI (SerializationArchitecture.md S18)
    pub const serialization_pending = true;

    /// Index (in mWheels) that represents the left wheel of this anti-rollbar
    left_wheel: i32 = 0,
    /// Index (in mWheels) that represents the right wheel of this anti-rollbar
    right_wheel: i32 = 1,
    /// Stiffness (spring constant in N/m) of anti rollbar, can be 0 to disable the anti-rollbar
    stiffness: f32 = 1000.0,

    /// Saves the contents in binary form to inStream.
    pub fn saveBinaryState(self: *const VehicleAntiRollBar, stream: StreamOut) void {
        stream.write(self.left_wheel);
        stream.write(self.right_wheel);
        stream.write(self.stiffness);
    }

    /// Restores the contents in binary form to inStream.
    pub fn restoreBinaryState(self: *VehicleAntiRollBar, stream: StreamIn) void {
        stream.read(&self.left_wheel);
        stream.read(&self.right_wheel);
        stream.read(&self.stiffness);
    }
};

/// The anti-roll bars of a vehicle (VehicleAntiRollBars)
pub const VehicleAntiRollBars = std.ArrayList(VehicleAntiRollBar);

comptime {
    std.debug.assert(@sizeOf(VehicleAntiRollBar) == 12);
}

test "VehicleAntiRollBar defaults and binary state" {
    const testing = std.testing;
    const allocator = testing.allocator;
    const StreamWrapper = @import("../../Core/StreamWrapper.zig");

    const defaults: VehicleAntiRollBar = .{};
    try testing.expectEqual(@as(i32, 0), defaults.left_wheel);
    try testing.expectEqual(@as(i32, 1), defaults.right_wheel);
    try testing.expectEqual(@as(f32, 1000.0), defaults.stiffness);

    const bar: VehicleAntiRollBar = .{ .left_wheel = 2, .right_wheel = 3, .stiffness = 500.0 };
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    bar.saveBinaryState(out_wrapper.streamOut());
    try testing.expectEqualSlices(u8, std.mem.asBytes(&bar), out.written());

    var restored: VehicleAntiRollBar = .{};
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
    restored.restoreBinaryState(in_wrapper.streamIn());
    try testing.expectEqual(bar, restored);
}
