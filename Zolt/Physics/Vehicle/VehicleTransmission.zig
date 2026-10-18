//! Port of: Jolt/Physics/Vehicle/VehicleTransmission.h, Jolt/Physics/Vehicle/VehicleTransmission.cpp
//! Status: complete
//!
//! - JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleTransmissionSettings):
//!   `rtti` with Jolt's attributes.
//! - `VehicleTransmission : VehicleTransmissionSettings` (non-virtual inheritance) embeds its parent as `base`
//!   (CharacterVehicleArchitecture E2).
//! - The gear ratio arrays are `std.ArrayList(f32)`: the default constructor `init(allocator)` allocates Jolt's default
//!   ratios, the copy constructor / assignment are `clone` / `assign`, `deinit(allocator)` frees them. The owner (the
//!   wheeled controller and its settings) passes its own allocator; arrays assigned through the writable getters must be
//!   allocated with it.
//! - `ETransmissionMode` has the fixed underlying type uint8 and the binary state restores any byte, so
//!   `TransmissionMode` is a non-exhaustive `enum(u8)` (porting guide, "Enums read from a stream"): every value other
//!   than `auto` behaves like Jolt's `mMode == ETransmissionMode::Auto` comparison (no automatic gear switching).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;

/// How gears are shifted
pub const TransmissionMode = enum(u8) {
    /// Automatically shift gear up and down
    auto,
    /// Manual gear shift (call SetTransmissionInput)
    manual,
    /// Any other uint8 value (Jolt's enum class has the fixed underlying type uint8)
    _,
};

/// Configuration for the transmission of a vehicle (gear box)
pub const VehicleTransmissionSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleTransmissionSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "VehicleTransmissionSettings",
        .kind = .non_virtual,
        .attributes = &.{
            .enumAttr("mMode", .mode),
            .attr("mGearRatios", .gear_ratios),
            .attr("mReverseGearRatios", .reverse_gear_ratios),
            .attr("mSwitchTime", .switch_time),
            .attr("mClutchReleaseTime", .clutch_release_time),
            .attr("mSwitchLatency", .switch_latency),
            .attr("mShiftUpRPM", .shift_up_rpm),
            .attr("mShiftDownRPM", .shift_down_rpm),
            .attr("mClutchStrength", .clutch_strength),
        },
    });

    /// Jolt's default gear ratios
    pub const default_gear_ratios = [_]f32{ 2.66, 1.78, 1.3, 1.0, 0.74 };
    /// Jolt's default reverse gear ratios
    pub const default_reverse_gear_ratios = [_]f32{-2.90};

    /// How to switch gears
    mode: TransmissionMode = .auto,
    /// Ratio in rotation rate between engine and gear box, first element is 1st gear, 2nd element 2nd gear etc.
    gear_ratios: std.ArrayList(f32) = .empty,
    /// Ratio in rotation rate between engine and gear box when driving in reverse
    reverse_gear_ratios: std.ArrayList(f32) = .empty,
    /// How long it takes to switch gears (s), only used in auto mode
    switch_time: f32 = 0.5,
    /// How long it takes to release the clutch (go to full friction), only used in auto mode
    clutch_release_time: f32 = 0.3,
    /// How long to wait after releasing the clutch before another switch is attempted (s), only used in auto mode
    switch_latency: f32 = 0.5,
    /// If RPM of engine is bigger then this we will shift a gear up, only used in auto mode
    shift_up_rpm: f32 = 4000.0,
    /// If RPM of engine is smaller then this we will shift a gear down, only used in auto mode
    shift_down_rpm: f32 = 2000.0,
    /// Strength of the clutch when fully engaged. Total torque a clutch applies is Torque = ClutchStrength * (Velocity Engine - Avg Velocity Wheels At Clutch) (units: k m^2 s^-1)
    clutch_strength: f32 = 10.0,

    /// The default constructor: Jolt's default gear ratios (allocated with `allocator`, free them with `deinit`)
    pub fn init(allocator: Allocator) Allocator.Error!VehicleTransmissionSettings {
        var self: VehicleTransmissionSettings = .{};
        errdefer self.deinit(allocator);
        try self.gear_ratios.appendSlice(allocator, &default_gear_ratios);
        try self.reverse_gear_ratios.appendSlice(allocator, &default_reverse_gear_ratios);
        return self;
    }

    /// Jolt's default constructor, used by RTTI create (Lifetime.defaultValue, SerializationArchitecture.md S7)
    pub const initDefault = init;

    /// Destructor: frees the gear ratio arrays
    pub fn deinit(self: *VehicleTransmissionSettings, allocator: Allocator) void {
        self.gear_ratios.deinit(allocator);
        self.reverse_gear_ratios.deinit(allocator);
    }

    /// Copy of these settings (copy constructor)
    pub fn clone(self: *const VehicleTransmissionSettings, allocator: Allocator) Allocator.Error!VehicleTransmissionSettings {
        var result = self.*;
        result.gear_ratios = try self.gear_ratios.clone(allocator);
        errdefer result.gear_ratios.deinit(allocator);
        result.reverse_gear_ratios = try self.reverse_gear_ratios.clone(allocator);
        return result;
    }

    /// Copy `other` into these settings (assignment operator): all or nothing
    pub fn assign(self: *VehicleTransmissionSettings, allocator: Allocator, other: *const VehicleTransmissionSettings) Allocator.Error!void {
        const copy = try other.clone(allocator);
        self.deinit(allocator);
        self.* = copy;
    }

    /// Saves the contents in binary form to inStream.
    pub fn saveBinaryState(self: *const VehicleTransmissionSettings, stream: StreamOut) void {
        stream.write(self.mode);
        stream.writeArray(f32, self.gear_ratios.items);
        stream.writeArray(f32, self.reverse_gear_ratios.items);
        stream.write(self.switch_time);
        stream.write(self.clutch_release_time);
        stream.write(self.switch_latency);
        stream.write(self.shift_up_rpm);
        stream.write(self.shift_down_rpm);
        stream.write(self.clutch_strength);
    }

    /// Restores the contents in binary form to inStream.
    pub fn restoreBinaryState(self: *VehicleTransmissionSettings, allocator: Allocator, stream: StreamIn) Allocator.Error!void {
        stream.read(&self.mode);
        try stream.readArray(f32, allocator, &self.gear_ratios);
        try stream.readArray(f32, allocator, &self.reverse_gear_ratios);
        stream.read(&self.switch_time);
        stream.read(&self.clutch_release_time);
        stream.read(&self.switch_latency);
        stream.read(&self.shift_up_rpm);
        stream.read(&self.shift_down_rpm);
        stream.read(&self.clutch_strength);
    }
};

/// Runtime data for transmission
pub const VehicleTransmission = struct {
    base: VehicleTransmissionSettings = .{},

    /// Current gear, -1 = reverse, 0 = neutral, 1 = 1st gear etc.
    current_gear: i32 = 0,
    /// Value between 0 and 1 indicating how much friction the clutch gives (0 = no friction, 1 = full friction)
    clutch_friction: f32 = 1.0,
    /// When switching gears this will be > 0 and will cause the engine to not provide any torque to the wheels for a short time (used for automatic gear switching only)
    gear_switch_time_left: f32 = 0.0,
    /// After switching gears this will be > 0 and will cause the clutch friction to go from 0 to 1 (used for automatic gear switching only)
    clutch_release_time_left: f32 = 0.0,
    /// After releasing the clutch this will be > 0 and will prevent another gear switch (used for automatic gear switching only)
    gear_switch_latency_time_left: f32 = 0.0,

    /// Default constructor (the default settings, allocated with `allocator`; free with `deinit`)
    pub fn init(allocator: Allocator) Allocator.Error!VehicleTransmission {
        return .{ .base = try .init(allocator) };
    }

    /// Destructor: frees the gear ratio arrays
    pub fn deinit(self: *VehicleTransmission, allocator: Allocator) void {
        self.base.deinit(allocator);
    }

    /// Set input from driver regarding the transmission (only relevant when transmission is set to manual mode)
    /// @param inCurrentGear Current gear, -1 = reverse, 0 = neutral, 1 = 1st gear etc.
    /// @param inClutchFriction Value between 0 and 1 indicating how much friction the clutch gives (0 = no friction, 1 = full friction)
    pub fn set(self: *VehicleTransmission, current_gear: i32, clutch_friction: f32) void {
        self.current_gear = current_gear;
        self.clutch_friction = clutch_friction;
    }

    /// Update the current gear and clutch friction if the transmission is in auto mode
    /// @param inDeltaTime Time step delta time in s
    /// @param inCurrentRPM Current RPM for engine
    /// @param inForwardInput Hint if the user wants to drive forward (> 0) or backwards (< 0)
    /// @param inCanShiftUp Indicates if we want to allow the transmission to shift up (e.g. pass false if wheels are slipping)
    pub fn update(self: *VehicleTransmission, delta_time: f32, current_rpm: f32, forward_input: f32, can_shift_up: bool) void {
        const s = &self.base;

        // Update current gear and calculate clutch friction
        if (s.mode == .auto) {
            // Switch gears based on rpm
            const old_gear = self.current_gear;
            if (self.current_gear == 0 // In neutral
            or forward_input * @as(f32, @floatFromInt(self.current_gear)) < 0.0) // Changing between forward / reverse
            {
                // Switch to first gear or reverse depending on input
                self.current_gear = if (forward_input > 0.0) 1 else (if (forward_input < 0.0) -1 else 0);
            } else if (self.gear_switch_latency_time_left == 0.0) // If not in the timout after switching gears
            {
                if (can_shift_up and current_rpm > s.shift_up_rpm) {
                    if (self.current_gear < 0) {
                        // Shift up, reverse
                        if (self.current_gear > -@as(i32, @intCast(s.reverse_gear_ratios.items.len)))
                            self.current_gear -= 1;
                    } else {
                        // Shift up, forward
                        if (self.current_gear < @as(i32, @intCast(s.gear_ratios.items.len)))
                            self.current_gear += 1;
                    }
                } else if (current_rpm < s.shift_down_rpm) {
                    if (self.current_gear < 0) {
                        // Shift down, reverse
                        const max_gear: i32 = if (forward_input != 0.0) -1 else 0;
                        if (self.current_gear < max_gear)
                            self.current_gear += 1;
                    } else {
                        // Shift down, forward
                        const min_gear: i32 = if (forward_input != 0.0) 1 else 0;
                        if (self.current_gear > min_gear)
                            self.current_gear -= 1;
                    }
                }
            }

            if (old_gear != self.current_gear) {
                // We've shifted gear, start switch countdown
                self.gear_switch_time_left = if (old_gear != 0) s.switch_time else 0.0;
                self.clutch_release_time_left = s.clutch_release_time;
                self.gear_switch_latency_time_left = s.switch_latency;
                self.clutch_friction = 0.0;
            } else if (self.gear_switch_time_left > 0.0) {
                // If still switching gears, count down
                self.gear_switch_time_left = math.max(@as(f32, 0.0), self.gear_switch_time_left - delta_time);
                self.clutch_friction = 0.0;
            } else if (self.clutch_release_time_left > 0.0) {
                // After switching the gears we slowly release the clutch
                self.clutch_release_time_left = math.max(@as(f32, 0.0), self.clutch_release_time_left - delta_time);
                self.clutch_friction = 1.0 - self.clutch_release_time_left / s.clutch_release_time;
            } else {
                // Clutch has full friction
                self.clutch_friction = 1.0;

                // Count down switch latency
                self.gear_switch_latency_time_left = math.max(@as(f32, 0.0), self.gear_switch_latency_time_left - delta_time);
            }
        }
    }

    /// Current gear, -1 = reverse, 0 = neutral, 1 = 1st gear etc.
    pub fn getCurrentGear(self: *const VehicleTransmission) i32 {
        return self.current_gear;
    }

    /// Value between 0 and 1 indicating how much friction the clutch gives (0 = no friction, 1 = full friction)
    pub fn getClutchFriction(self: *const VehicleTransmission) f32 {
        return self.clutch_friction;
    }

    /// If the auto box is currently switching gears
    pub fn isSwitchingGear(self: *const VehicleTransmission) bool {
        return self.gear_switch_time_left > 0.0;
    }

    /// Return the transmission ratio based on the current gear (ratio between engine and differential)
    pub fn getCurrentRatio(self: *const VehicleTransmission) f32 {
        if (self.current_gear < 0)
            return self.base.reverse_gear_ratios.items[@intCast(-self.current_gear - 1)]
        else if (self.current_gear == 0)
            return 0.0
        else
            return self.base.gear_ratios.items[@intCast(self.current_gear - 1)];
    }

    /// Only allow sleeping when the transmission is idle
    pub fn allowSleep(self: *const VehicleTransmission) bool {
        return self.gear_switch_time_left <= 0.0 and self.clutch_release_time_left <= 0.0 and self.gear_switch_latency_time_left <= 0.0;
    }

    /// Saving state for replay
    pub fn saveState(self: *const VehicleTransmission, stream: *StateRecorder) void {
        stream.write(self.current_gear);
        stream.write(self.clutch_friction);
        stream.write(self.gear_switch_time_left);
        stream.write(self.clutch_release_time_left);
        stream.write(self.gear_switch_latency_time_left);
    }

    pub fn restoreState(self: *VehicleTransmission, stream: *StateRecorder) void {
        stream.read(&self.current_gear);
        stream.read(&self.clutch_friction);
        stream.read(&self.gear_switch_time_left);
        stream.read(&self.clutch_release_time_left);
        stream.read(&self.gear_switch_latency_time_left);
    }
};

const testing = std.testing;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");

test "VehicleTransmissionSettings defaults and binary state" {
    const allocator = testing.allocator;

    var settings = try VehicleTransmissionSettings.init(allocator);
    defer settings.deinit(allocator);
    try testing.expectEqual(TransmissionMode.auto, settings.mode);
    try testing.expectEqualSlices(f32, &.{ 2.66, 1.78, 1.3, 1.0, 0.74 }, settings.gear_ratios.items);
    try testing.expectEqualSlices(f32, &.{-2.90}, settings.reverse_gear_ratios.items);
    try testing.expectEqual(@as(f32, 0.5), settings.switch_time);
    try testing.expectEqual(@as(f32, 0.3), settings.clutch_release_time);
    try testing.expectEqual(@as(f32, 0.5), settings.switch_latency);
    try testing.expectEqual(@as(f32, 4000.0), settings.shift_up_rpm);
    try testing.expectEqual(@as(f32, 2000.0), settings.shift_down_rpm);
    try testing.expectEqual(@as(f32, 10.0), settings.clutch_strength);

    settings.mode = .manual;
    settings.clutch_strength = 3.0;
    _ = settings.reverse_gear_ratios.pop();

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    settings.saveBinaryState(out_wrapper.streamOut());
    try testing.expectEqual(@as(usize, 1 + (4 + 5 * 4) + 4 + 6 * 4), out.written().len);

    var restored = try VehicleTransmissionSettings.init(allocator);
    defer restored.deinit(allocator);
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
    try restored.restoreBinaryState(allocator, in_wrapper.streamIn());
    try testing.expectEqual(TransmissionMode.manual, restored.mode);
    try testing.expectEqual(@as(f32, 3.0), restored.clutch_strength);
    try testing.expectEqual(@as(usize, 0), restored.reverse_gear_ratios.items.len);
    try testing.expectEqualSlices(f32, settings.gear_ratios.items, restored.gear_ratios.items);

    // An unknown mode byte is restored as is
    const modified = try allocator.dupe(u8, out.written());
    defer allocator.free(modified);
    modified[0] = 7;
    var reader2: std.Io.Reader = .fixed(modified);
    var in_wrapper2: StreamWrapper.StreamInWrapper = .init(&reader2);
    try restored.restoreBinaryState(allocator, in_wrapper2.streamIn());
    try testing.expectEqual(@as(u8, 7), @intFromEnum(restored.mode));
}

test "VehicleTransmission automatic gear switching" {
    const allocator = testing.allocator;

    var t = try VehicleTransmission.init(allocator);
    defer t.deinit(allocator);
    try testing.expect(t.allowSleep());
    try testing.expectEqual(@as(f32, 0.0), t.getCurrentRatio());

    // Neutral and forward input: first gear, clutch released over clutch_release_time, no switch time from neutral
    t.update(0.1, 1000.0, 1.0, true);
    try testing.expectEqual(@as(i32, 1), t.getCurrentGear());
    try testing.expectEqual(@as(f32, 0.0), t.getClutchFriction());
    try testing.expect(!t.isSwitchingGear());
    try testing.expect(!t.allowSleep());
    try testing.expectEqual(@as(f32, 2.66), t.getCurrentRatio());
    t.update(0.1, 1000.0, 1.0, true);
    try testing.expectEqual(@as(f32, 1.0 - (math.max(@as(f32, 0.0), @as(f32, 0.3) - 0.1)) / 0.3), t.getClutchFriction());

    // Let the clutch and the latency run out
    for (0..20) |_| t.update(0.1, 3000.0, 1.0, true);
    try testing.expectEqual(@as(f32, 1.0), t.getClutchFriction());
    try testing.expect(t.allowSleep());

    // High RPM shifts up (with a switch time), unless we may not shift up
    t.update(0.1, 4500.0, 1.0, false);
    try testing.expectEqual(@as(i32, 1), t.getCurrentGear());
    t.update(0.1, 4500.0, 1.0, true);
    try testing.expectEqual(@as(i32, 2), t.getCurrentGear());
    try testing.expect(t.isSwitchingGear());
    try testing.expectEqual(@as(f32, 0.5), t.gear_switch_time_left);

    // During the latency no switch happens
    for (0..5) |_| t.update(0.1, 4500.0, 1.0, true);
    try testing.expectEqual(@as(i32, 2), t.getCurrentGear());

    // Low RPM with input shifts down to first gear but not to neutral
    for (0..30) |_| t.update(0.1, 1000.0, 1.0, true);
    try testing.expectEqual(@as(i32, 1), t.getCurrentGear());

    // Reverse input: reverse gear, the ratio is negative
    t.update(0.1, 1000.0, -1.0, true);
    try testing.expectEqual(@as(i32, -1), t.getCurrentGear());
    try testing.expectEqual(@as(f32, -2.90), t.getCurrentRatio());

    // Only one reverse gear: shifting up in reverse stays in -1
    for (0..30) |_| t.update(0.1, 4500.0, -1.0, true);
    try testing.expectEqual(@as(i32, -1), t.getCurrentGear());

    // No input and low RPM: back to neutral
    for (0..30) |_| t.update(0.1, 1000.0, 0.0, true);
    try testing.expectEqual(@as(i32, 0), t.getCurrentGear());

    // Manual mode: Update does nothing, Set does
    t.base.mode = .manual;
    t.update(0.1, 1000.0, 1.0, true);
    try testing.expectEqual(@as(i32, 0), t.getCurrentGear());
    t.set(3, 0.25);
    try testing.expectEqual(@as(i32, 3), t.getCurrentGear());
    try testing.expectEqual(@as(f32, 0.25), t.getClutchFriction());
    try testing.expectEqual(@as(f32, 1.3), t.getCurrentRatio());
}

test "VehicleTransmission save / restore state" {
    const allocator = testing.allocator;
    const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;

    var t = try VehicleTransmission.init(allocator);
    defer t.deinit(allocator);
    t.update(0.1, 1000.0, -1.0, true);

    var recorder: StateRecorderImpl = .init(allocator);
    defer recorder.deinit();
    t.saveState(&recorder.base);
    try testing.expectEqual(@as(usize, 5 * 4), recorder.getDataSize());

    var t2 = try VehicleTransmission.init(allocator);
    defer t2.deinit(allocator);
    t2.restoreState(&recorder.base);
    try testing.expectEqual(@as(i32, -1), t2.getCurrentGear());
    try testing.expectEqual(t.clutch_release_time_left, t2.clutch_release_time_left);
    try testing.expectEqual(t.gear_switch_latency_time_left, t2.gear_switch_latency_time_left);
}

test "VehicleTransmissionSettings out of memory" {
    try testing.checkAllAllocationFailures(testing.allocator, struct {
        fn run(allocator: Allocator) !void {
            var settings = try VehicleTransmissionSettings.init(allocator);
            defer settings.deinit(allocator);
            var copy = try settings.clone(allocator);
            defer copy.deinit(allocator);
            var other = try VehicleTransmissionSettings.init(allocator);
            defer other.deinit(allocator);
            try other.assign(allocator, &copy);
        }
    }.run, .{});
}
