//! Port of: Jolt/Physics/Vehicle/VehicleEngine.h, Jolt/Physics/Vehicle/VehicleEngine.cpp
//! Status: complete
//!
//! - JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleEngineSettings):
//!   TODO(serialization) (ObjectStream is Phase 8, CharacterVehicleArchitecture E1; the binary state is ported).
//! - JPH_DEBUG_RENDERER (`ConvertRPMToAngle`, `DrawRPM`): TODO(debug renderer).
//! - `VehicleEngine : VehicleEngineSettings` (non-virtual inheritance) embeds its parent as `base`
//!   (CharacterVehicleArchitecture E2).
//! - The settings own a LinearCurve (`mNormalizedTorque`), so like LinearCurve the functions that allocate take the
//!   allocator: the default constructor is `init(allocator)` (it adds the 3 points of Jolt's default curve), the copy
//!   constructor / assignment are `clone` / `assign`, `deinit(allocator)` frees the curve. The owner (the wheeled
//!   controller and its settings) passes its own allocator.
//! - Jolt behavior reproduced on purpose (CharacterVehicleArchitecture, "Settled in the foundation review"):
//!   SaveBinaryState / RestoreBinaryState omit `mInertia` and `mAngularDamping`, so a restore keeps the values that the
//!   settings had before (the defaults for settings that a restore creates).

const std = @import("std");
const Allocator = std.mem.Allocator;
const LinearCurve = @import("../../Core/LinearCurve.zig").LinearCurve;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;

/// Generic properties for a vehicle engine
pub const VehicleEngineSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL(JPH_EXPORT, VehicleEngineSettings)

    /// Max amount of torque (Nm) that the engine can deliver
    max_torque: f32 = 500.0,
    /// Min amount of revolutions per minute (rpm) the engine can produce without stalling
    min_rpm: f32 = 1000.0,
    /// Max amount of revolutions per minute (rpm) the engine can generate
    max_rpm: f32 = 6000.0,
    /// Y-axis: Curve that describes a ratio of the max torque the engine can produce (0 = 0, 1 = mMaxTorque). X-axis: the fraction of the RPM of the engine (0 = mMinRPM, 1 = mMaxRPM)
    normalized_torque: LinearCurve = .{},
    /// Moment of inertia (kg m^2) of the engine
    inertia: f32 = 0.5,
    /// Angular damping factor of the wheel: dw/dt = -c * w. Value should be zero or positive and is usually close to 0.
    angular_damping: f32 = 0.2,

    /// Constructor (the default torque curve is allocated with `allocator`, free it with `deinit`)
    pub fn init(allocator: Allocator) Allocator.Error!VehicleEngineSettings {
        var self: VehicleEngineSettings = .{};
        errdefer self.deinit(allocator);
        try self.normalized_torque.reserve(allocator, 3);
        try self.normalized_torque.addPoint(allocator, 0.0, 0.8);
        try self.normalized_torque.addPoint(allocator, 0.66, 1.0);
        try self.normalized_torque.addPoint(allocator, 1.0, 0.8);
        return self;
    }

    /// Destructor: frees the torque curve
    pub fn deinit(self: *VehicleEngineSettings, allocator: Allocator) void {
        self.normalized_torque.deinit(allocator);
    }

    /// Copy of these settings (copy constructor)
    pub fn clone(self: *const VehicleEngineSettings, allocator: Allocator) Allocator.Error!VehicleEngineSettings {
        var result = self.*;
        result.normalized_torque = try self.normalized_torque.clone(allocator);
        return result;
    }

    /// Copy `other` into these settings (assignment operator): all or nothing
    pub fn assign(self: *VehicleEngineSettings, allocator: Allocator, other: *const VehicleEngineSettings) Allocator.Error!void {
        const copy = try other.clone(allocator);
        self.deinit(allocator);
        self.* = copy;
    }

    /// Saves the contents in binary form to inStream.
    pub fn saveBinaryState(self: *const VehicleEngineSettings, stream: StreamOut) void {
        stream.write(self.max_torque);
        stream.write(self.min_rpm);
        stream.write(self.max_rpm);
        self.normalized_torque.saveBinaryState(stream);
    }

    /// Restores the contents in binary form to inStream.
    pub fn restoreBinaryState(self: *VehicleEngineSettings, allocator: Allocator, stream: StreamIn) Allocator.Error!void {
        stream.read(&self.max_torque);
        stream.read(&self.min_rpm);
        stream.read(&self.max_rpm);
        try self.normalized_torque.restoreBinaryState(allocator, stream);
    }
};

/// Runtime data for engine
pub const VehicleEngine = struct {
    /// Multiply an angular velocity (rad/s) with this value to get rounds per minute (RPM)
    pub const angular_velocity_to_rpm: f32 = 60.0 / (2.0 * math.pi);

    base: VehicleEngineSettings = .{},

    /// Current rotation speed of engine in rounds per minute (Jolt: `= mMinRPM`, the default of the settings)
    current_rpm: f32 = (VehicleEngineSettings{}).min_rpm,

    /// Default constructor (the default settings, allocated with `allocator`; free with `deinit`)
    pub fn init(allocator: Allocator) Allocator.Error!VehicleEngine {
        return .{ .base = try .init(allocator) };
    }

    /// Destructor: frees the torque curve
    pub fn deinit(self: *VehicleEngine, allocator: Allocator) void {
        self.base.deinit(allocator);
    }

    /// Clamp the RPM between min and max RPM
    pub fn clampRPM(self: *VehicleEngine) void {
        self.current_rpm = math.clamp(self.current_rpm, self.base.min_rpm, self.base.max_rpm);
    }

    /// Current rotation speed of engine in rounds per minute
    pub fn getCurrentRPM(self: *const VehicleEngine) f32 {
        return self.current_rpm;
    }

    /// Update rotation speed of engine in rounds per minute
    pub fn setCurrentRPM(self: *VehicleEngine, rpm: f32) void {
        self.current_rpm = rpm;
        self.clampRPM();
    }

    /// Get current angular velocity of the engine in radians / second
    pub fn getAngularVelocity(self: *const VehicleEngine) f32 {
        return self.current_rpm / angular_velocity_to_rpm;
    }

    /// Get the amount of torque (N m) that the engine can supply
    /// @param inAcceleration How much the gas pedal is pressed [0, 1]
    pub fn getTorque(self: *const VehicleEngine, acceleration: f32) f32 {
        return acceleration * self.base.max_torque * self.base.normalized_torque.getValue(self.current_rpm / self.base.max_rpm);
    }

    /// Apply a torque to the engine rotation speed
    /// @param inTorque Torque in N m
    /// @param inDeltaTime Delta time in seconds
    pub fn applyTorque(self: *VehicleEngine, torque: f32, delta_time: f32) void {
        // Accelerate engine using torque
        self.current_rpm += angular_velocity_to_rpm * torque * delta_time / self.base.inertia;
        self.clampRPM();
    }

    /// Update the engine RPM for damping
    /// @param inDeltaTime Delta time in seconds
    pub fn applyDamping(self: *VehicleEngine, delta_time: f32) void {
        // Angular damping: dw/dt = -c * w
        // Solution: w(t) = w(0) * e^(-c * t) or w2 = w1 * e^(-c * dt)
        // Taylor expansion of e^(-c * dt) = 1 - c * dt + ...
        // Since dt is usually in the order of 1/60 and c is a low number too this approximation is good enough
        self.current_rpm *= math.max(@as(f32, 0.0), 1.0 - self.base.angular_damping * delta_time);
        self.clampRPM();
    }

    // TODO(debug renderer): ConvertRPMToAngle, DrawRPM (JPH_DEBUG_RENDERER)

    /// If the engine is idle we allow the vehicle to sleep
    pub fn allowSleep(self: *const VehicleEngine) bool {
        return self.current_rpm <= 1.01 * self.base.min_rpm;
    }

    /// Saving state for replay
    pub fn saveState(self: *const VehicleEngine, stream: *StateRecorder) void {
        stream.write(self.current_rpm);
    }

    pub fn restoreState(self: *VehicleEngine, stream: *StateRecorder) void {
        stream.read(&self.current_rpm);
    }
};

const testing = std.testing;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");

test "VehicleEngineSettings defaults, binary state without inertia and angular damping" {
    const allocator = testing.allocator;

    var settings = try VehicleEngineSettings.init(allocator);
    defer settings.deinit(allocator);
    try testing.expectEqual(@as(f32, 500.0), settings.max_torque);
    try testing.expectEqual(@as(f32, 1000.0), settings.min_rpm);
    try testing.expectEqual(@as(f32, 6000.0), settings.max_rpm);
    try testing.expectEqual(@as(f32, 0.5), settings.inertia);
    try testing.expectEqual(@as(f32, 0.2), settings.angular_damping);
    try testing.expectEqualSlices(LinearCurve.Point, &.{ .{ .x = 0.0, .y = 0.8 }, .{ .x = 0.66, .y = 1.0 }, .{ .x = 1.0, .y = 0.8 } }, settings.normalized_torque.points.items);

    settings.max_torque = 123.0;
    settings.inertia = 7.0;
    settings.angular_damping = 0.9;
    try settings.normalized_torque.addPoint(allocator, 2.0, 0.5);

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    settings.saveBinaryState(out_wrapper.streamOut());
    try testing.expectEqual(@as(usize, 3 * 4 + 4 + 4 * 8), out.written().len); // 3 floats, the curve, no inertia / damping

    var restored = try VehicleEngineSettings.init(allocator);
    defer restored.deinit(allocator);
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
    try restored.restoreBinaryState(allocator, in_wrapper.streamIn());
    try testing.expectEqual(@as(f32, 123.0), restored.max_torque);
    try testing.expectEqual(@as(f32, 0.5), restored.inertia); // Not in the stream (Jolt)
    try testing.expectEqual(@as(f32, 0.2), restored.angular_damping);
    try testing.expectEqualSlices(LinearCurve.Point, settings.normalized_torque.points.items, restored.normalized_torque.points.items);

    // Copy and assignment
    var copy = try settings.clone(allocator);
    defer copy.deinit(allocator);
    try testing.expectEqual(@as(f32, 7.0), copy.inertia);
    try testing.expectEqual(@as(usize, 4), copy.normalized_torque.points.items.len);
    try restored.assign(allocator, &copy);
    try testing.expectEqual(@as(f32, 0.9), restored.angular_damping);
}

test "VehicleEngine torque, damping and RPM clamping" {
    const allocator = testing.allocator;

    var engine = try VehicleEngine.init(allocator);
    defer engine.deinit(allocator);
    try testing.expectEqual(@as(f32, 1000.0), engine.getCurrentRPM());
    try testing.expect(engine.allowSleep());

    // Torque at the start of the curve: 0.8 * max torque
    try testing.expectEqual(@as(f32, 0.5 * 500.0 * engine.base.normalized_torque.getValue(1000.0 / 6000.0)), engine.getTorque(0.5));

    // Clamping
    engine.setCurrentRPM(10000.0);
    try testing.expectEqual(@as(f32, 6000.0), engine.getCurrentRPM());
    engine.setCurrentRPM(-5.0);
    try testing.expectEqual(@as(f32, 1000.0), engine.getCurrentRPM());

    // Applying torque increases the RPM: rpm += 60 / (2 pi) * torque * dt / inertia
    engine.applyTorque(100.0, 0.1);
    try testing.expectEqual(@as(f32, 1000.0) + VehicleEngine.angular_velocity_to_rpm * 100.0 * 0.1 / 0.5, engine.getCurrentRPM());
    try testing.expect(!engine.allowSleep());

    // Damping decreases it, but not below the minimum
    const rpm = engine.getCurrentRPM();
    engine.applyDamping(0.5);
    try testing.expectEqual(math.max(@as(f32, 1000.0), rpm * (1.0 - 0.2 * 0.5)), engine.getCurrentRPM());
    engine.applyDamping(100.0); // Factor clamped to 0, then to min RPM
    try testing.expectEqual(@as(f32, 1000.0), engine.getCurrentRPM());

    try testing.expectEqual(engine.getCurrentRPM() / VehicleEngine.angular_velocity_to_rpm, engine.getAngularVelocity());
}

test "VehicleEngine save / restore state" {
    const allocator = testing.allocator;
    const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;

    var engine = try VehicleEngine.init(allocator);
    defer engine.deinit(allocator);
    engine.setCurrentRPM(2500.0);

    var recorder: StateRecorderImpl = .init(allocator);
    defer recorder.deinit();
    engine.saveState(&recorder.base);
    try testing.expectEqualSlices(u8, &std.mem.toBytes(@as(f32, 2500.0)), recorder.getData());

    engine.setCurrentRPM(1500.0);
    engine.restoreState(&recorder.base);
    try testing.expectEqual(@as(f32, 2500.0), engine.getCurrentRPM());
}

test "VehicleEngineSettings out of memory" {
    try testing.checkAllAllocationFailures(testing.allocator, struct {
        fn run(allocator: Allocator) !void {
            var settings = try VehicleEngineSettings.init(allocator);
            defer settings.deinit(allocator);
            var copy = try settings.clone(allocator);
            defer copy.deinit(allocator);
            var other = try VehicleEngineSettings.init(allocator);
            defer other.deinit(allocator);
            try other.assign(allocator, &copy);
        }
    }.run, .{});
}
