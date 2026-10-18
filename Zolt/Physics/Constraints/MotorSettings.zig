//! Port of: Jolt/Physics/Constraints/MotorSettings.h, Jolt/Physics/Constraints/MotorSettings.cpp
//! Status: complete
//!
//! - `EMotorState` is an `enum class` without an underlying type (`int`), so `MotorState` is `enum(i32)`: constraints
//!   write it with `inStream.Write(mMotorState)`, 4 bytes. The underlying type of an enum class is fixed, so every
//!   `int` is a valid EMotorState and a constraint's RestoreBinaryState stores whatever the stream holds: `MotorState` is
//!   non-exhaustive, a switch on it needs a `_` prong that does what Jolt's switch does when no case matches.
//! - The constructor overloads are `init(frequency, damping)` (Jolt's `MotorSettings(inFrequency, inDamping)`),
//!   `initMode(mode, frequency, damping)`, `initLimits(frequency, damping, force_limit, torque_limit)` and
//!   `initModeLimits(mode, frequency, damping, force_limit, torque_limit)`; the default constructor is `.{}`.
//! - The binary state is Jolt's bytes (the spring settings, then the four limits).
//! - JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL: `rtti`. The first three attributes are
//!   Jolt's aliases into the nested spring settings ("mFrequency" and "mDamping" are the old attribute names, which
//!   Assets/Human.tof still uses).

const std = @import("std");
const Core = @import("../../Core/Core.zig");
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const math = @import("../../Math/Math.zig");
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const SpringSettingsFile = @import("SpringSettings.zig");
const SpringMode = SpringSettingsFile.SpringMode;
const SpringSettings = SpringSettingsFile.SpringSettings;

/// EMotorState
pub const MotorState = enum(i32) {
    /// Motor is off
    off,
    /// Motor will drive to target velocity limited only by max force/torque that the motor can apply
    velocity,
    /// Motor will drive to target position using: force = stiffness * (target_position - current_position) - damping * current_velocity
    position,
    /// Motor will drive both to target position and velocity using: force = stiffness * (target_position - current_position) + damping * (target_velocity - current_velocity)
    position_and_velocity,
    /// Any other int value (Jolt's enum class has the fixed underlying type int, see the file comment)
    _,
};

// Ability to check if position/velocity is enabled for a motor state
pub fn isVelocityMotor(motor_state: MotorState) bool {
    return (@intFromEnum(motor_state) & @intFromEnum(MotorState.velocity)) != 0;
}

pub fn isPositionMotor(motor_state: MotorState) bool {
    return (@intFromEnum(motor_state) & @intFromEnum(MotorState.position)) != 0;
}

comptime {
    std.debug.assert(!isVelocityMotor(.off));
    std.debug.assert(isVelocityMotor(.velocity));
    std.debug.assert(!isVelocityMotor(.position));
    std.debug.assert(isVelocityMotor(.position_and_velocity));
    std.debug.assert(!isPositionMotor(.off));
    std.debug.assert(!isPositionMotor(.velocity));
    std.debug.assert(isPositionMotor(.position));
    std.debug.assert(isPositionMotor(.position_and_velocity));
}

/// Class that contains the settings for a constraint motor.
/// See the main page of the API documentation for more information on how to configure a motor.
pub const MotorSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(MotorSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "MotorSettings",
        .kind = .non_virtual,
        .attributes = &.{
            .enumAlias("mSpringMode", .{ .spring_settings, .mode }),
            .alias("mFrequency", .{ .spring_settings, .frequency }), // Renaming attributes to stay compatible with old versions of the library
            .alias("mDamping", .{ .spring_settings, .damping }),
            .attr("mMinForceLimit", .min_force_limit),
            .attr("mMaxForceLimit", .max_force_limit),
            .attr("mMinTorqueLimit", .min_torque_limit),
            .attr("mMaxTorqueLimit", .max_torque_limit),
        },
    });

    // Settings
    /// Settings for the spring that is used to drive to the position target (not used when motor is a velocity motor).
    spring_settings: SpringSettings = .init(.frequency_and_damping, 2.0, 1.0),
    /// Minimum force to apply in case of a linear constraint (N). Usually this is -mMaxForceLimit unless you want a motor that can e.g. push but not pull. Not used when motor is an angular motor.
    min_force_limit: f32 = -math.flt_max,
    /// Maximum force to apply in case of a linear constraint (N). Not used when motor is an angular motor.
    max_force_limit: f32 = math.flt_max,
    /// Minimum torque to apply in case of a angular constraint (N m). Usually this is -mMaxTorqueLimit unless you want a motor that can e.g. push but not pull. Not used when motor is a linear motor.
    min_torque_limit: f32 = -math.flt_max,
    /// Maximum torque to apply in case of a angular constraint (N m). Not used when motor is a linear motor.
    max_torque_limit: f32 = math.flt_max,

    /// Constructor MotorSettings(ESpringMode inMode, float inFrequency, float inDamping)
    pub fn initMode(mode: SpringMode, frequency: f32, damping: f32) MotorSettings {
        const settings: MotorSettings = .{ .spring_settings = .init(mode, frequency, damping) };
        if (Core.enable_asserts) std.debug.assert(settings.isValid());
        return settings;
    }

    /// Constructor MotorSettings(float inFrequency, float inDamping)
    pub fn init(frequency: f32, damping: f32) MotorSettings {
        const settings: MotorSettings = .{ .spring_settings = .init(.frequency_and_damping, frequency, damping) };
        if (Core.enable_asserts) std.debug.assert(settings.isValid());
        return settings;
    }

    /// Constructor MotorSettings(ESpringMode inMode, float inFrequency, float inDamping, float inForceLimit, float inTorqueLimit)
    pub fn initModeLimits(mode: SpringMode, frequency: f32, damping: f32, force_limit: f32, torque_limit: f32) MotorSettings {
        const settings: MotorSettings = .{ .spring_settings = .init(mode, frequency, damping), .min_force_limit = -force_limit, .max_force_limit = force_limit, .min_torque_limit = -torque_limit, .max_torque_limit = torque_limit };
        if (Core.enable_asserts) std.debug.assert(settings.isValid());
        return settings;
    }

    /// Constructor MotorSettings(float inFrequency, float inDamping, float inForceLimit, float inTorqueLimit)
    pub fn initLimits(frequency: f32, damping: f32, force_limit: f32, torque_limit: f32) MotorSettings {
        const settings: MotorSettings = .{ .spring_settings = .init(.frequency_and_damping, frequency, damping), .min_force_limit = -force_limit, .max_force_limit = force_limit, .min_torque_limit = -torque_limit, .max_torque_limit = torque_limit };
        if (Core.enable_asserts) std.debug.assert(settings.isValid());
        return settings;
    }

    /// Set asymmetric force limits
    pub fn setForceLimits(self: *MotorSettings, min: f32, max: f32) void {
        if (Core.enable_asserts) std.debug.assert(min <= max);
        self.min_force_limit = min;
        self.max_force_limit = max;
    }

    /// Set asymmetric torque limits
    pub fn setTorqueLimits(self: *MotorSettings, min: f32, max: f32) void {
        if (Core.enable_asserts) std.debug.assert(min <= max);
        self.min_torque_limit = min;
        self.max_torque_limit = max;
    }

    /// Set symmetric force limits
    pub fn setForceLimit(self: *MotorSettings, limit: f32) void {
        self.min_force_limit = -limit;
        self.max_force_limit = limit;
    }

    /// Set symmetric torque limits
    pub fn setTorqueLimit(self: *MotorSettings, limit: f32) void {
        self.min_torque_limit = -limit;
        self.max_torque_limit = limit;
    }

    /// Check if settings are valid
    pub fn isValid(self: *const MotorSettings) bool {
        return self.spring_settings.frequency >= 0.0 and self.spring_settings.damping >= 0.0 and self.min_force_limit <= self.max_force_limit and self.min_torque_limit <= self.max_torque_limit;
    }

    /// Saves the contents of the motor settings in binary form to inStream.
    pub fn saveBinaryState(self: *const MotorSettings, stream: StreamOut) void {
        self.spring_settings.saveBinaryState(stream);
        stream.write(self.min_force_limit);
        stream.write(self.max_force_limit);
        stream.write(self.min_torque_limit);
        stream.write(self.max_torque_limit);
    }

    /// Restores contents from the binary stream inStream.
    pub fn restoreBinaryState(self: *MotorSettings, stream: StreamIn) void {
        self.spring_settings.restoreBinaryState(stream);
        stream.read(&self.min_force_limit);
        stream.read(&self.max_force_limit);
        stream.read(&self.min_torque_limit);
        stream.read(&self.max_torque_limit);
    }
};

test "MotorState values and helpers" {
    const testing = std.testing;
    try testing.expectEqual(@as(i32, 0), @intFromEnum(MotorState.off));
    try testing.expectEqual(@as(i32, 1), @intFromEnum(MotorState.velocity));
    try testing.expectEqual(@as(i32, 2), @intFromEnum(MotorState.position));
    try testing.expectEqual(@as(i32, 3), @intFromEnum(MotorState.position_and_velocity));
    try testing.expectEqual(@as(usize, 4), @sizeOf(MotorState));
    var state: MotorState = .position_and_velocity;
    _ = &state;
    try testing.expect(isVelocityMotor(state) and isPositionMotor(state));

    // Values outside the named states (a restored int) test the bits like Jolt
    try testing.expect(isVelocityMotor(@enumFromInt(5)) and !isPositionMotor(@enumFromInt(5)));
    try testing.expect(!isVelocityMotor(@enumFromInt(4)) and !isPositionMotor(@enumFromInt(4)));
    try testing.expect(isVelocityMotor(@enumFromInt(-1)) and isPositionMotor(@enumFromInt(-1)));
}

test "MotorSettings constructors, limits and validity" {
    const testing = std.testing;

    const default_settings: MotorSettings = .{};
    try testing.expectEqual(SpringMode.frequency_and_damping, default_settings.spring_settings.mode);
    try testing.expectEqual(@as(f32, 2.0), default_settings.spring_settings.frequency);
    try testing.expectEqual(@as(f32, 1.0), default_settings.spring_settings.damping);
    try testing.expectEqual(-math.flt_max, default_settings.min_force_limit);
    try testing.expectEqual(math.flt_max, default_settings.max_force_limit);
    try testing.expectEqual(-math.flt_max, default_settings.min_torque_limit);
    try testing.expectEqual(math.flt_max, default_settings.max_torque_limit);
    try testing.expect(default_settings.isValid());

    const s1 = MotorSettings.init(3.0, 0.5);
    try testing.expectEqual(SpringMode.frequency_and_damping, s1.spring_settings.mode);
    try testing.expectEqual(@as(f32, 3.0), s1.spring_settings.frequency);
    try testing.expectEqual(@as(f32, 0.5), s1.spring_settings.damping);
    try testing.expectEqual(math.flt_max, s1.max_force_limit);

    const s2 = MotorSettings.initMode(.stiffness_and_damping, 100.0, 5.0);
    try testing.expectEqual(SpringMode.stiffness_and_damping, s2.spring_settings.mode);
    try testing.expectEqual(@as(f32, 100.0), s2.spring_settings.getStiffness());

    const s3 = MotorSettings.initLimits(3.0, 0.5, 10.0, 20.0);
    try testing.expectEqual(@as(f32, -10.0), s3.min_force_limit);
    try testing.expectEqual(@as(f32, 10.0), s3.max_force_limit);
    try testing.expectEqual(@as(f32, -20.0), s3.min_torque_limit);
    try testing.expectEqual(@as(f32, 20.0), s3.max_torque_limit);

    var s4 = MotorSettings.initModeLimits(.mass_normalized_stiffness_and_damping, 4.0, 0.0, 1.0, 2.0);
    try testing.expectEqual(SpringMode.mass_normalized_stiffness_and_damping, s4.spring_settings.mode);
    try testing.expectEqual(@as(f32, -1.0), s4.min_force_limit);
    try testing.expectEqual(@as(f32, 2.0), s4.max_torque_limit);

    s4.setForceLimits(-1.0, 5.0);
    try testing.expectEqual(@as(f32, -1.0), s4.min_force_limit);
    try testing.expectEqual(@as(f32, 5.0), s4.max_force_limit);
    s4.setTorqueLimits(0.0, 3.0);
    try testing.expectEqual(@as(f32, 0.0), s4.min_torque_limit);
    try testing.expectEqual(@as(f32, 3.0), s4.max_torque_limit);
    s4.setForceLimit(7.0);
    try testing.expectEqual(@as(f32, -7.0), s4.min_force_limit);
    try testing.expectEqual(@as(f32, 7.0), s4.max_force_limit);
    s4.setTorqueLimit(8.0);
    try testing.expectEqual(@as(f32, -8.0), s4.min_torque_limit);
    try testing.expectEqual(@as(f32, 8.0), s4.max_torque_limit);
    try testing.expect(s4.isValid());

    // Invalid settings (set through the fields, the constructors assert validity)
    var invalid = s4;
    invalid.spring_settings.frequency = -1.0;
    try testing.expect(!invalid.isValid());
    invalid = s4;
    invalid.spring_settings.damping = -1.0;
    try testing.expect(!invalid.isValid());
    invalid = s4;
    invalid.setForceLimit(-1.0);
    try testing.expect(!invalid.isValid());
    invalid = s4;
    invalid.setTorqueLimit(-1.0);
    try testing.expect(!invalid.isValid());
}

test "MotorSettings binary state is Jolt's bytes and round trips" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;

    var recorder = StateRecorderImpl.init(testing.allocator);
    defer recorder.deinit();

    var settings = MotorSettings.initModeLimits(.stiffness_and_damping, 50.0, 2.0, 3.0, 4.0);
    settings.setForceLimits(-1.0, 6.0);
    settings.saveBinaryState(recorder.recorder().streamOut());

    const data = recorder.getData();
    try testing.expectEqual(@as(usize, 9 + 4 * 4), data.len);
    try testing.expectEqual(@as(u8, 1), data[0]);
    try testing.expectEqual(@as(f32, 50.0), std.mem.bytesToValue(f32, data[1..5]));
    try testing.expectEqual(@as(f32, 2.0), std.mem.bytesToValue(f32, data[5..9]));
    try testing.expectEqual(@as(f32, -1.0), std.mem.bytesToValue(f32, data[9..13]));
    try testing.expectEqual(@as(f32, 6.0), std.mem.bytesToValue(f32, data[13..17]));
    try testing.expectEqual(@as(f32, -4.0), std.mem.bytesToValue(f32, data[17..21]));
    try testing.expectEqual(@as(f32, 4.0), std.mem.bytesToValue(f32, data[21..25]));

    var restored: MotorSettings = .{};
    restored.restoreBinaryState(recorder.recorder().streamIn());
    try testing.expect(!recorder.isFailed());
    try testing.expectEqual(settings.spring_settings.mode, restored.spring_settings.mode);
    try testing.expectEqual(settings.spring_settings.frequency, restored.spring_settings.frequency);
    try testing.expectEqual(settings.spring_settings.damping, restored.spring_settings.damping);
    try testing.expectEqual(settings.min_force_limit, restored.min_force_limit);
    try testing.expectEqual(settings.max_force_limit, restored.max_force_limit);
    try testing.expectEqual(settings.min_torque_limit, restored.min_torque_limit);
    try testing.expectEqual(settings.max_torque_limit, restored.max_torque_limit);
}
