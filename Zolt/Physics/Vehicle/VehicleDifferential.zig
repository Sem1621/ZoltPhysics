//! Port of: Jolt/Physics/Vehicle/VehicleDifferential.h, Jolt/Physics/Vehicle/VehicleDifferential.cpp
//! Status: complete
//!
//! - JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleDifferentialSettings):
//!   TODO(serialization) (ObjectStream is Phase 8, CharacterVehicleArchitecture E1; the binary state is ported).
//! - `CalculateTorqueRatio(left, right, outLeft, outRight)` returns `TorqueRatio{ .left, .right }` (out parameters).
//!   Jolt's asserts on the settings (`mLimitedSlipRatio > 1`, the fractions add up to 1) are user input checks: they are
//!   evaluated in safe builds only (`if (Core.enable_asserts)`), so that optimized builds compute what Jolt's release
//!   build computes.

const std = @import("std");
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const Core = @import("../../Core/Core.zig");
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");

pub const VehicleDifferentialSettings = extern struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleDifferentialSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "VehicleDifferentialSettings",
        .kind = .non_virtual,
    });
    /// TODO(serialization): the attributes of Jolt's sCreateRTTI (SerializationArchitecture.md S18)
    pub const serialization_pending = true;

    /// Index (in mWheels) that represents the left wheel of this differential (can be -1 to indicate no wheel)
    left_wheel: i32 = -1,
    /// Index (in mWheels) that represents the right wheel of this differential (can be -1 to indicate no wheel)
    right_wheel: i32 = -1,
    /// Ratio between rotation speed of gear box and wheels
    differential_ratio: f32 = 3.42,
    /// Defines how the engine torque is split across the left and right wheel (0 = left, 0.5 = center, 1 = right)
    left_right_split: f32 = 0.5,
    /// Ratio max / min wheel speed. When this ratio is exceeded, all torque gets distributed to the slowest moving wheel. This allows implementing a limited slip differential. Set to FLT_MAX for an open differential. Value should be > 1.
    limited_slip_ratio: f32 = 1.4,
    /// How much of the engines torque is applied to this differential (0 = none, 1 = full), make sure the sum of all differentials is 1.
    engine_torque_ratio: f32 = 1.0,

    /// The fractions of the torque that go to the left and the right wheel (the out parameters of CalculateTorqueRatio)
    pub const TorqueRatio = struct {
        /// Fraction of torque that should go to the left wheel
        left: f32,
        /// Fraction of torque that should go to the right wheel
        right: f32,
    };

    /// Saves the contents in binary form to inStream.
    pub fn saveBinaryState(self: *const VehicleDifferentialSettings, stream: StreamOut) void {
        stream.write(self.left_wheel);
        stream.write(self.right_wheel);
        stream.write(self.differential_ratio);
        stream.write(self.left_right_split);
        stream.write(self.limited_slip_ratio);
        stream.write(self.engine_torque_ratio);
    }

    /// Restores the contents in binary form to inStream.
    pub fn restoreBinaryState(self: *VehicleDifferentialSettings, stream: StreamIn) void {
        stream.read(&self.left_wheel);
        stream.read(&self.right_wheel);
        stream.read(&self.differential_ratio);
        stream.read(&self.left_right_split);
        stream.read(&self.limited_slip_ratio);
        stream.read(&self.engine_torque_ratio);
    }

    /// Calculate the torque ratio between left and right wheel
    /// @param inLeftAngularVelocity Angular velocity of left wheel (rad / s)
    /// @param inRightAngularVelocity Angular velocity of right wheel (rad / s)
    /// @return the fraction of torque that should go to the left wheel and to the right wheel
    pub fn calculateTorqueRatio(self: *const VehicleDifferentialSettings, left_angular_velocity: f32, right_angular_velocity: f32) TorqueRatio {
        // Start with the default torque ratio
        var left_torque_fraction = 1.0 - self.left_right_split;
        var right_torque_fraction = self.left_right_split;

        if (self.limited_slip_ratio < math.flt_max) {
            if (Core.enable_asserts) std.debug.assert(self.limited_slip_ratio > 1.0);

            // This is a limited slip differential, adjust torque ratios according to wheel speeds
            const omega_l = math.max(@as(f32, 1.0e-3), @abs(left_angular_velocity)); // prevent div by zero by setting a minimum velocity and ignoring that the wheels may be rotating in different directions
            const omega_r = math.max(@as(f32, 1.0e-3), @abs(right_angular_velocity));
            const omega_min = math.min(omega_l, omega_r);
            const omega_max = math.max(omega_l, omega_r);

            // Map into a value that is 0 when the wheels are turning at an equal rate and 1 when the wheels are turning at mLimitedSlipRotationRatio
            const alpha = math.min((omega_max / omega_min - 1.0) / (self.limited_slip_ratio - 1.0), @as(f32, 1.0));
            if (Core.enable_asserts) std.debug.assert(alpha >= 0.0);
            const one_min_alpha = 1.0 - alpha;

            if (omega_l < omega_r) {
                // Redirect more power to the left wheel
                left_torque_fraction = left_torque_fraction * one_min_alpha + alpha;
                right_torque_fraction = right_torque_fraction * one_min_alpha;
            } else {
                // Redirect more power to the right wheel
                left_torque_fraction = left_torque_fraction * one_min_alpha;
                right_torque_fraction = right_torque_fraction * one_min_alpha + alpha;
            }
        }

        // Assert the values add up to 1
        if (Core.enable_asserts) std.debug.assert(@abs(left_torque_fraction + right_torque_fraction - 1.0) < 1.0e-6);

        return .{ .left = left_torque_fraction, .right = right_torque_fraction };
    }
};

comptime {
    // Plain data, written as Jolt's members in the binary state
    std.debug.assert(@sizeOf(VehicleDifferentialSettings) == 24);
}

const testing = std.testing;

test "VehicleDifferentialSettings defaults and torque ratios" {
    const d: VehicleDifferentialSettings = .{};
    try testing.expectEqual(@as(i32, -1), d.left_wheel);
    try testing.expectEqual(@as(i32, -1), d.right_wheel);
    try testing.expectEqual(@as(f32, 3.42), d.differential_ratio);
    try testing.expectEqual(@as(f32, 0.5), d.left_right_split);
    try testing.expectEqual(@as(f32, 1.4), d.limited_slip_ratio);
    try testing.expectEqual(@as(f32, 1.0), d.engine_torque_ratio);

    // Equal speeds: the default split
    const equal = d.calculateTorqueRatio(10.0, 10.0);
    try testing.expectEqual(@as(f32, 0.5), equal.left);
    try testing.expectEqual(@as(f32, 0.5), equal.right);

    // Left wheel slower than the ratio: all torque to the left wheel
    const slow_left = d.calculateTorqueRatio(1.0, 10.0);
    try testing.expectEqual(@as(f32, 1.0), slow_left.left);
    try testing.expectEqual(@as(f32, 0.0), slow_left.right);

    // A small difference redirects part of the torque to the slower right wheel
    const slow_right = d.calculateTorqueRatio(-11.0, 10.0);
    try testing.expect(slow_right.right > 0.5 and slow_right.right < 1.0);
    try testing.expect(@abs(slow_right.left + slow_right.right - 1.0) < 1.0e-6);

    // An open differential keeps the split
    var open = d;
    open.limited_slip_ratio = math.flt_max;
    open.left_right_split = 0.25;
    const r = open.calculateTorqueRatio(1.0, 100.0);
    try testing.expectEqual(@as(f32, 0.75), r.left);
    try testing.expectEqual(@as(f32, 0.25), r.right);
}

test "VehicleDifferentialSettings binary state" {
    const allocator = testing.allocator;
    const StreamWrapper = @import("../../Core/StreamWrapper.zig");

    const d: VehicleDifferentialSettings = .{ .left_wheel = 2, .right_wheel = -1, .differential_ratio = 4.0, .left_right_split = 0.3, .limited_slip_ratio = 2.0, .engine_torque_ratio = 0.5 };
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    d.saveBinaryState(out_wrapper.streamOut());
    try testing.expectEqualSlices(u8, std.mem.asBytes(&d), out.written());

    var restored: VehicleDifferentialSettings = .{};
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
    restored.restoreBinaryState(in_wrapper.streamIn());
    try testing.expectEqual(d, restored);
}
