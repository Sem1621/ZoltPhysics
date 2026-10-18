//! Port of: Jolt/Physics/Vehicle/VehicleTrack.h, Jolt/Physics/Vehicle/VehicleTrack.cpp
//! Status: complete
//!
//! - JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleTrackSettings): `rtti` with
//!   Jolt's attributes.
//! - `VehicleTrack : VehicleTrackSettings` (non-virtual inheritance) embeds its parent as `base`
//!   (CharacterVehicleArchitecture E2). `VehicleTracks` (`VehicleTrack[ETrackSide::Num]`) is `[2]VehicleTrack`.
//! - `mWheels` is a `std.ArrayList(u32)`: the functions that allocate take the allocator (`deinit`, `clone`, `assign`,
//!   `restoreBinaryState`), the owner (the tracked controller and its settings) passes its own.
//! - Deviation (CharacterVehicleArchitecture, "Settled in the foundation review"): Jolt leaves `mDrivenWheel`
//!   uninitialized, Zolt defaults it to 0 (an undefined value has nothing to reproduce). An object stream therefore
//!   writes 0 for a default track (SerializationArchitecture.md D9).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;

/// On which side of the vehicle the track is located (for steering)
pub const TrackSide = enum(u32) {
    left = 0,
    right = 1,
    num = 2,
};

/// Generic properties for tank tracks
pub const VehicleTrackSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_NON_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleTrackSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "VehicleTrackSettings",
        .kind = .non_virtual,
        .attributes = &.{
            .attr("mDrivenWheel", .driven_wheel),
            .attr("mWheels", .wheels),
            .attr("mInertia", .inertia),
            .attr("mAngularDamping", .angular_damping),
            .attr("mMaxBrakeTorque", .max_brake_torque),
            .attr("mDifferentialRatio", .differential_ratio),
        },
    });

    /// Which wheel on the track is connected to the engine (Zolt: 0 by default, Jolt leaves it uninitialized)
    driven_wheel: u32 = 0,
    /// Indices of wheels that are inside this track, should include the driven wheel too
    wheels: std.ArrayList(u32) = .empty,
    /// Moment of inertia (kg m^2) of the track and its wheels as seen on the driven wheel
    inertia: f32 = 10.0,
    /// Damping factor of track and its wheels: dw/dt = -c * w as seen on the driven wheel
    angular_damping: f32 = 0.5,
    /// How much torque (Nm) the brakes can apply on the driven wheel
    max_brake_torque: f32 = 15000.0,
    /// Ratio between rotation speed of gear box and driven wheel of track
    differential_ratio: f32 = 6.0,

    /// Destructor: frees the wheel indices
    pub fn deinit(self: *VehicleTrackSettings, allocator: Allocator) void {
        self.wheels.deinit(allocator);
    }

    /// Copy of these settings (copy constructor)
    pub fn clone(self: *const VehicleTrackSettings, allocator: Allocator) Allocator.Error!VehicleTrackSettings {
        var result = self.*;
        result.wheels = try self.wheels.clone(allocator);
        return result;
    }

    /// Copy `other` into these settings (assignment operator): all or nothing
    pub fn assign(self: *VehicleTrackSettings, allocator: Allocator, other: *const VehicleTrackSettings) Allocator.Error!void {
        const copy = try other.clone(allocator);
        self.deinit(allocator);
        self.* = copy;
    }

    /// Saves the contents in binary form to inStream.
    pub fn saveBinaryState(self: *const VehicleTrackSettings, stream: StreamOut) void {
        stream.write(self.driven_wheel);
        stream.writeArray(u32, self.wheels.items);
        stream.write(self.inertia);
        stream.write(self.angular_damping);
        stream.write(self.max_brake_torque);
        stream.write(self.differential_ratio);
    }

    /// Restores the contents in binary form to inStream.
    pub fn restoreBinaryState(self: *VehicleTrackSettings, allocator: Allocator, stream: StreamIn) Allocator.Error!void {
        stream.read(&self.driven_wheel);
        try stream.readArray(u32, allocator, &self.wheels);
        stream.read(&self.inertia);
        stream.read(&self.angular_damping);
        stream.read(&self.max_brake_torque);
        stream.read(&self.differential_ratio);
    }
};

/// Runtime data for tank tracks
pub const VehicleTrack = struct {
    base: VehicleTrackSettings = .{},

    /// Angular velocity of the driven wheel, will determine the speed of the entire track
    angular_velocity: f32 = 0.0,

    /// Destructor: frees the wheel indices
    pub fn deinit(self: *VehicleTrack, allocator: Allocator) void {
        self.base.deinit(allocator);
    }

    /// Saving state for replay
    pub fn saveState(self: *const VehicleTrack, stream: *StateRecorder) void {
        stream.write(self.angular_velocity);
    }

    pub fn restoreState(self: *VehicleTrack, stream: *StateRecorder) void {
        stream.read(&self.angular_velocity);
    }
};

/// The tracks of a vehicle, indexed by TrackSide (VehicleTracks)
pub const VehicleTracks = [@intFromEnum(TrackSide.num)]VehicleTrack;

const testing = std.testing;

test "VehicleTrackSettings defaults, binary state, copies" {
    const allocator = testing.allocator;
    const StreamWrapper = @import("../../Core/StreamWrapper.zig");

    var settings: VehicleTrackSettings = .{};
    defer settings.deinit(allocator);
    try testing.expectEqual(@as(u32, 0), settings.driven_wheel);
    try testing.expectEqual(@as(usize, 0), settings.wheels.items.len);
    try testing.expectEqual(@as(f32, 10.0), settings.inertia);
    try testing.expectEqual(@as(f32, 0.5), settings.angular_damping);
    try testing.expectEqual(@as(f32, 15000.0), settings.max_brake_torque);
    try testing.expectEqual(@as(f32, 6.0), settings.differential_ratio);

    settings.driven_wheel = 2;
    try settings.wheels.appendSlice(allocator, &.{ 0, 1, 2, 3 });
    settings.max_brake_torque = 10.0;

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    settings.saveBinaryState(out_wrapper.streamOut());
    try testing.expectEqual(@as(usize, 4 + 4 + 4 * 4 + 4 * 4), out.written().len);

    var restored: VehicleTrackSettings = .{};
    defer restored.deinit(allocator);
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
    try restored.restoreBinaryState(allocator, in_wrapper.streamIn());
    try testing.expectEqual(@as(u32, 2), restored.driven_wheel);
    try testing.expectEqualSlices(u32, &.{ 0, 1, 2, 3 }, restored.wheels.items);
    try testing.expectEqual(@as(f32, 10.0), restored.max_brake_torque);

    var copy = try settings.clone(allocator);
    defer copy.deinit(allocator);
    try testing.expectEqualSlices(u32, settings.wheels.items, copy.wheels.items);
    var other: VehicleTrackSettings = .{};
    defer other.deinit(allocator);
    try other.assign(allocator, &copy);
    try testing.expectEqual(@as(u32, 2), other.driven_wheel);

    var tracks: VehicleTracks = .{ .{}, .{} };
    tracks[@intFromEnum(TrackSide.right)].angular_velocity = 3.0;
    try testing.expectEqual(@as(f32, 3.0), tracks[1].angular_velocity);
}
