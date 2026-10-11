//! Port of: Jolt/Physics/Constraints/SpringSettings.h, Jolt/Physics/Constraints/SpringSettings.cpp
//! Status: complete
//!
//! - Jolt's anonymous union `{ float mFrequency; float mStiffness; }` is the field `frequency`: the union member that
//!   Jolt initializes (`= 0.0f`), serializes (SaveBinaryState, the ObjectStream attribute "mFrequency") and reads most
//!   often. `mStiffness` is the same float, read and written with `getStiffness` / `setStiffness` (both members are
//!   `float`, so the union is one value with two names). The constructor `SpringSettings(inMode,
//!   inFrequencyOrStiffness, inDamping)` is `init(mode, frequency_or_stiffness, damping)`.
//! - The binary state is Jolt's bytes: the mode as one byte, then the frequency / stiffness and the damping.
//! - JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL: TODO(serialization).

const std = @import("std");
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;

/// Enum used by constraints to specify how the spring is defined (ESpringMode)
pub const SpringMode = enum(u8) {
    /// Frequency and damping are specified.
    frequency_and_damping,
    /// Stiffness and damping are specified.
    stiffness_and_damping,
    /// Stiffness and damping divided by mass / inertia are specified (also known as acceleration mode). This makes it easier to tune the spring and makes it mass independent.
    mass_normalized_stiffness_and_damping,
};

/// Settings for a linear or angular spring
pub const SpringSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, SpringSettings)

    /// Selects the way in which the spring is defined. See the descriptions of the mFrequency, mStiffness and mDamping properties.
    mode: SpringMode = .frequency_and_damping,

    /// Jolt's `union { float mFrequency; float mStiffness; }`, see the file comment and `getStiffness` / `setStiffness`.
    ///
    /// Valid when mMode = ESpringMode::FrequencyAndDamping.
    /// If > 0 the constraint will be soft and this specifies the oscillation frequency in Hz.
    /// If <= 0, mDamping is ignored and the constraint will have hard limits (as hard as the time step / the number of velocity / position solver steps allows).
    frequency: f32 = 0.0,

    /// When mMode = ESpringMode::FrequencyAndDamping this is the damping ratio (0 = no damping, 1 = critical damping).
    ///
    /// When mMode = ESpringMode::StiffnessAndDamping this is the damping (c) in the spring equation F = -k * x - c * v for a linear or T = -k * theta - c * w for an angular spring.
    /// Units are N s / m for a linear spring and N s m / rad for an angular spring.
    ///
    /// When mMode = ESpringMode::MassNormalizedStiffnessAndDamping this is the damping (c) in the spring equation F = m_eff * (-k * x - c * v) for a linear or T = i_eff * (-k * theta - c * w) for an angular spring.
    /// m_eff / i_eff is the effective mass / inertia of the constraint.
    /// Units are 1 / s for a linear spring and 1 / rad s for an angular spring.
    ///
    /// Note that if you set this to 0, you will not get an infinite oscillation. Because we integrate physics using an explicit Euler scheme, there is always energy loss.
    /// This is done to keep the simulation from exploding, because with a damping of 0 and even the slightest rounding error, the oscillation could become bigger and bigger until the simulation explodes.
    damping: f32 = 0.0,

    /// Constructor
    pub fn init(mode: SpringMode, frequency_or_stiffness: f32, damping: f32) SpringSettings {
        return .{ .mode = mode, .frequency = frequency_or_stiffness, .damping = damping };
    }

    /// mStiffness (the other member of the union with mFrequency):
    ///
    /// When mMode = ESpringMode::StiffnessAndDamping:
    /// Specifies the stiffness (k) in the spring equation F = -k * x - c * v for a linear or T = -k * theta - c * w for an angular spring.
    /// Units are N / m for a linear spring and N m / rad for an angular spring.
    ///
    /// Note that stiffness values are large numbers. To calculate a ballpark value for the needed stiffness you can use:
    /// force = stiffness * delta_spring_length = mass * gravity <=> stiffness = mass * gravity / delta_spring_length.
    /// So if your object weighs 1500 kg and the spring compresses by 2 meters, you need a stiffness in the order of 1500 * 9.81 / 2 ~ 7500 N/m.
    ///
    /// When mMode = ESpringMode::MassNormalizedStiffnessAndDamping:
    /// Specifies the stiffness (k) in the spring equation F = m_eff * (-k * x - c * v) for a linear or T = i_eff * (-k * theta - c * w) for an angular spring.
    /// m_eff / i_eff is the effective mass / inertia of the constraint.
    /// Units are 1 / s^2 for a linear spring and 1 / rad s^2 for an angular spring.
    ///
    /// Since the stiffness is multiplied by the effective mass / inertia of the constraint, you can use much smaller stiffness values and they will be mass independent.
    pub fn getStiffness(self: *const SpringSettings) f32 {
        return self.frequency;
    }

    /// Write mStiffness (the other member of the union with mFrequency), see getStiffness
    pub fn setStiffness(self: *SpringSettings, stiffness: f32) void {
        self.frequency = stiffness;
    }

    /// Saves the contents of the spring settings in binary form to inStream.
    pub fn saveBinaryState(self: *const SpringSettings, stream: StreamOut) void {
        stream.write(self.mode);
        stream.write(self.frequency);
        stream.write(self.damping);
    }

    /// Restores contents from the binary stream inStream.
    pub fn restoreBinaryState(self: *SpringSettings, stream: StreamIn) void {
        stream.read(&self.mode);
        stream.read(&self.frequency);
        stream.read(&self.damping);
    }

    /// Check if the spring has a valid frequency / stiffness, if not the spring will be hard
    pub fn hasStiffness(self: *const SpringSettings) bool {
        return self.frequency > 0.0;
    }

    /// Check if this spring has stiffness or damping (making it active), if not the constraint will be hard
    pub fn hasStiffnessOrDamping(self: *const SpringSettings) bool {
        return self.frequency > 0.0 or (self.mode != .frequency_and_damping and self.damping > 0.0);
    }
};

test "SpringSettings defaults and stiffness / damping checks" {
    const testing = std.testing;

    const default_settings: SpringSettings = .{};
    try testing.expectEqual(SpringMode.frequency_and_damping, default_settings.mode);
    try testing.expectEqual(@as(f32, 0.0), default_settings.frequency);
    try testing.expectEqual(@as(f32, 0.0), default_settings.getStiffness());
    try testing.expectEqual(@as(f32, 0.0), default_settings.damping);
    try testing.expect(!default_settings.hasStiffness());
    try testing.expect(!default_settings.hasStiffnessOrDamping());

    // Damping alone only activates the spring when the damping is not a damping ratio
    try testing.expect(!SpringSettings.init(.frequency_and_damping, 0.0, 1.0).hasStiffnessOrDamping());
    try testing.expect(SpringSettings.init(.stiffness_and_damping, 0.0, 1.0).hasStiffnessOrDamping());
    try testing.expect(!SpringSettings.init(.stiffness_and_damping, 0.0, 1.0).hasStiffness());
    try testing.expect(SpringSettings.init(.mass_normalized_stiffness_and_damping, 0.0, 0.5).hasStiffnessOrDamping());
    try testing.expect(!SpringSettings.init(.mass_normalized_stiffness_and_damping, -1.0, 0.0).hasStiffnessOrDamping());
    try testing.expect(SpringSettings.init(.frequency_and_damping, 2.0, 0.0).hasStiffness());
    try testing.expect(SpringSettings.init(.frequency_and_damping, 2.0, 0.0).hasStiffnessOrDamping());

    // mStiffness and mFrequency are the same union member
    var settings = SpringSettings.init(.stiffness_and_damping, 0.0, 3.0);
    settings.setStiffness(7500.0);
    try testing.expectEqual(@as(f32, 7500.0), settings.getStiffness());
    try testing.expectEqual(@as(f32, 7500.0), settings.frequency);
    try testing.expect(settings.hasStiffness());
}

test "SpringSettings binary state is Jolt's bytes and round trips" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;

    var recorder = StateRecorderImpl.init(testing.allocator);
    defer recorder.deinit();

    const settings = SpringSettings.init(.mass_normalized_stiffness_and_damping, 1.5, 0.25);
    settings.saveBinaryState(recorder.recorder().streamOut());

    var expected: [9]u8 = undefined;
    expected[0] = 2;
    @memcpy(expected[1..5], std.mem.asBytes(&@as(f32, 1.5)));
    @memcpy(expected[5..9], std.mem.asBytes(&@as(f32, 0.25)));
    try testing.expectEqualSlices(u8, &expected, recorder.getData());

    var restored: SpringSettings = .{};
    restored.restoreBinaryState(recorder.recorder().streamIn());
    try testing.expect(!recorder.isFailed());
    try testing.expectEqual(settings.mode, restored.mode);
    try testing.expectEqual(settings.frequency, restored.frequency);
    try testing.expectEqual(settings.damping, restored.damping);
}
