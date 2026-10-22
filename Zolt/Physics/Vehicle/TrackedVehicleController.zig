//! Port of: Jolt/Physics/Vehicle/TrackedVehicleController.h, Jolt/Physics/Vehicle/TrackedVehicleController.cpp
//! Status: complete
//!
//! - JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(TrackedVehicleControllerSettings,
//!   WheelSettingsTV): `rtti` with Jolt's attributes. `mTracks` is the C array `[2]VehicleTrackSettings`: a stream must
//!   have exactly 2 tracks.
//! - JPH_DEBUG_RENDERER (`Draw`, `SetRPMMeter`): gated declarations (RemainingFeaturesArchitecture.md R2).
//!   `mRPMMeterPosition` and `mRPMMeterSize` are `void` without `-Ddebug_renderer`. Draw writes the per-track and
//!   per-wheel texts into its renderer, where Jolt uses `DebugRenderer::sInstance` (R4: there is one renderer, so it is
//!   the same).
//! - CharacterVehicleArchitecture E2:
//!   - `WheelSettingsTV` embeds `base: WheelSettings` and overrides its binary state; `WheelTV` embeds `base: Wheel`.
//!   - `TrackedVehicleControllerSettings` embeds `base: VehicleControllerSettings` (`rtti`, a class of
//!     `RegisterTypes.factory`, E3); `TrackedVehicleController` embeds `base:
//!     VehicleController`. Both are concrete classes that a user class may derive from (`initDerived(T, ...)`).
//!   - `ConstructWheel`'s `JPH_ASSERT(IsKindOf(&inWheel, JPH_RTTI(WheelSettingsTV)))` + static_cast is the checked
//!     `wheel.cast(WheelSettingsTV)`.
//!   - The protected functions are `pub` (VehicleConstraint is a friend in Jolt, `syncLeftRightTracks` is tested
//!     directly).
//! - The engine, transmission and track settings own arrays (the torque curve, the gear ratios, the wheel indices of a
//!   track): they use the allocator of the class that owns them (the settings' root, the controller's root); arrays
//!   assigned through the writable getters (`getEngineMut`, `getTransmissionMut`, `getTracksMut`) must be allocated with
//!   it.
//! - Allocation (E2): `create`, `getSettings` and `constructController` / `constructWheel` return `Allocator.Error` and
//!   leave nothing behind. PreCollide, PostCollide and SolveLongitudinalAndLateralConstraints run inside the physics
//!   update (step listener and solver jobs) and allocate nothing, as in Jolt.
//! - Jolt's asserts on the settings in the constructor and on the track ratios (`SetDriverInput`, `SetLeftRatio`,
//!   `SetRightRatio`: a ratio of 0 divides by zero in PostCollide) are user input checks, evaluated in safe builds only
//!   (`if (Core.enable_asserts)`).
//! - A wheel that is in no track keeps `mTrackIndex = -1` and Jolt indexes `mTracks[-1]` (undefined behavior); Zolt's
//!   safe builds panic in the index cast, ReleaseFast stays undefined as in C++. Wheel indices of a track and a driven
//!   wheel outside the wheels are out of bounds the same way.
//! - `VehicleTrackSettings::mDrivenWheel` is 0 by default in Zolt (uninitialized in Jolt, see VehicleTrack.zig).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const Ref = @import("../../Core/Reference.zig").Ref;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const DMat44 = @import("../../Math/DMat44.zig").DMat44;
const RMat44 = @import("../../Math/Real.zig").RMat44;
const Color = @import("../../Core/Color.zig").Color;
const StringTools = @import("../../Core/StringTools.zig");
const DebugRenderer = @import("../../Renderer/DebugRenderer.zig").DebugRenderer;
const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const VehicleConstraint = @import("VehicleConstraint.zig").VehicleConstraint;
const VehicleControllerFile = @import("VehicleController.zig");
const VehicleController = VehicleControllerFile.VehicleController;
const VehicleControllerSettings = VehicleControllerFile.VehicleControllerSettings;
const WheelFile = @import("Wheel.zig");
const Wheel = WheelFile.Wheel;
const WheelSettings = WheelFile.WheelSettings;
const VehicleEngineFile = @import("VehicleEngine.zig");
const VehicleEngine = VehicleEngineFile.VehicleEngine;
const VehicleEngineSettings = VehicleEngineFile.VehicleEngineSettings;
const VehicleTransmissionFile = @import("VehicleTransmission.zig");
const VehicleTransmission = VehicleTransmissionFile.VehicleTransmission;
const VehicleTransmissionSettings = VehicleTransmissionFile.VehicleTransmissionSettings;
const VehicleTrackFile = @import("VehicleTrack.zig");
const TrackSide = VehicleTrackFile.TrackSide;
const VehicleTrack = VehicleTrackFile.VehicleTrack;
const VehicleTracks = VehicleTrackFile.VehicleTracks;
const VehicleTrackSettings = VehicleTrackFile.VehicleTrackSettings;

/// The number of tracks (ETrackSide::Num)
const num_tracks = @intFromEnum(TrackSide.num);

/// WheelSettings object specifically for TrackedVehicleController
pub const WheelSettingsTV = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(WheelSettingsTV)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "WheelSettingsTV",
        .kind = .virtual,
        .bases = &.{WheelSettings},
        .attributes = &.{
            .attr("mLongitudinalFriction", .longitudinal_friction),
            .attr("mLateralFriction", .lateral_friction),
        },
    });

    pub const overrides = .{ .saveBinaryState, .restoreBinaryState };

    base: WheelSettings,

    /// Friction in forward direction of tire
    longitudinal_friction: f32 = 4.0,
    /// Friction in sideways direction of tire
    lateral_friction: f32 = 2.0,

    /// Constructor of the base part of a class T derived from WheelSettingsTV
    pub fn initDerived(comptime T: type, allocator: Allocator) WheelSettingsTV {
        return .{ .base = .init(T, allocator) };
    }

    /// Constructor of settings that are not on the heap (the owner calls `base.deinit()`)
    pub fn init(allocator: Allocator) WheelSettingsTV {
        return initDerived(WheelSettingsTV, allocator);
    }

    /// new WheelSettingsTV (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*WheelSettingsTV {
        const self = try allocator.create(WheelSettingsTV);
        self.* = .init(allocator);
        return self;
    }

    /// new WheelSettingsTV (RTTI create)
    pub const createDefault = create;

    /// Upcasts (implicit in C++)
    pub fn asWheelSettings(self: *WheelSettingsTV) *WheelSettings {
        return &self.base;
    }

    pub fn asWheelSettingsConst(self: *const WheelSettingsTV) *const WheelSettings {
        return &self.base;
    }

    // See: WheelSettings
    pub fn saveBinaryState(self: *const WheelSettingsTV, stream: StreamOut) void {
        WheelSettings.impl.saveBinaryState(&self.base, stream);

        stream.write(self.longitudinal_friction);
        stream.write(self.lateral_friction);
    }

    pub fn restoreBinaryState(self: *WheelSettingsTV, stream: StreamIn) Allocator.Error!void {
        try WheelSettings.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.longitudinal_friction);
        stream.read(&self.lateral_friction);
    }
};

/// Wheel object specifically for TrackedVehicleController
pub const WheelTV = struct {
    pub const overrides = .{};

    base: Wheel,

    /// Index in mTracks to which this wheel is attached (calculated on initialization)
    track_index: i32 = -1,
    /// Combined friction coefficient in longitudinal direction (combines terrain and track)
    combined_longitudinal_friction: f32 = 0.0,
    /// Combined friction coefficient in lateral direction (combines terrain and track)
    combined_lateral_friction: f32 = 0.0,
    /// Amount of impulse that the brakes can apply to the floor (excluding friction), spread out from brake impulse applied on track
    brake_impulse: f32 = 0.0,

    /// Constructor of the base part of a class T derived from WheelTV
    pub fn initDerived(comptime T: type, allocator: Allocator, settings: *const WheelSettingsTV) WheelTV {
        return .{ .base = .init(T, allocator, &settings.base) };
    }

    /// Constructor
    pub fn init(allocator: Allocator, settings: *const WheelSettingsTV) WheelTV {
        return initDerived(WheelTV, allocator, settings);
    }

    /// new WheelTV(inSettings) (VehicleConstraint destroys it)
    pub fn create(allocator: Allocator, settings: *const WheelSettingsTV) Allocator.Error!*WheelTV {
        const self = try allocator.create(WheelTV);
        self.* = .init(allocator, settings);
        return self;
    }

    /// Override GetSettings and cast to the correct class
    pub fn getSettings(self: *const WheelTV) *const WheelSettingsTV {
        return virtual.downcast(WheelSettingsTV, self.base.getSettings());
    }

    /// Update the angular velocity of the wheel based on the angular velocity of the track
    pub fn calculateAngularVelocity(self: *WheelTV, constraint: *const VehicleConstraint) void {
        const settings = self.getSettings();
        const wheels = constraint.getWheels();
        const track = &virtual.downcast(TrackedVehicleController, constraint.getController()).getTracks()[@intCast(self.track_index)];

        // Calculate angular velocity of this wheel
        self.base.angular_velocity = track.angular_velocity * wheels[track.base.driven_wheel].getSettings().radius / settings.base.radius;
    }

    /// Update the wheel rotation based on the current angular velocity
    pub fn update(self: *WheelTV, wheel_index: u32, delta_time: f32, constraint: *const VehicleConstraint) void {
        self.calculateAngularVelocity(constraint);

        // Update rotation of wheel
        self.base.angle = @rem(self.base.angle + self.base.angular_velocity * delta_time, 2.0 * math.pi); // fmod

        // Reset brake impulse, will be set during post collision again
        self.brake_impulse = 0.0;

        if (self.base.contact_body) |contact_body| {
            // Friction at the point of this wheel between track and floor
            const settings = self.getSettings();
            const combine_friction = constraint.getCombineFriction();
            self.combined_longitudinal_friction = settings.longitudinal_friction;
            self.combined_lateral_friction = settings.lateral_friction;
            combine_friction.call(wheel_index, &self.combined_longitudinal_friction, &self.combined_lateral_friction, contact_body, self.base.contact_sub_shape_id);
        } else {
            // No collision
            self.combined_lateral_friction = 0.0;
            self.combined_longitudinal_friction = 0.0;
        }
    }
};

/// Settings of a vehicle with tank tracks
///
/// Default settings are based around what I could find about the M1 Abrams tank.
/// Note to avoid issues with very heavy objects vs very light objects the mass of the tank should be a lot lower (say 10x) than that of a real tank. That means that the engine/brake torque is also 10x less.
pub const TrackedVehicleControllerSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(TrackedVehicleControllerSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "TrackedVehicleControllerSettings",
        .kind = .virtual,
        .bases = &.{VehicleControllerSettings},
        .attributes = &.{
            .attr("mEngine", .engine),
            .attr("mTransmission", .transmission),
            .attr("mTracks", .tracks),
        },
    });

    pub const overrides = .{ .constructController, .saveBinaryState, .restoreBinaryState };

    base: VehicleControllerSettings,

    /// The properties of the engine
    engine: VehicleEngineSettings = .{},
    /// The properties of the transmission (aka gear box)
    transmission: VehicleTransmissionSettings = .{},
    /// List of tracks and their properties
    tracks: [num_tracks]VehicleTrackSettings = .{ .{}, .{} },

    /// Constructor of the base part of a class T derived from TrackedVehicleControllerSettings (the engine curve and the
    /// gear ratios are allocated with `allocator`, the destructor frees them)
    pub fn initDerived(comptime T: type, allocator: Allocator) Allocator.Error!TrackedVehicleControllerSettings {
        var self: TrackedVehicleControllerSettings = .{ .base = .init(T, allocator) };
        errdefer self.destruct();
        self.engine = try .init(allocator);
        self.transmission = try .init(allocator);

        // Numbers guestimated from: https://en.wikipedia.org/wiki/M1_Abrams
        self.engine.min_rpm = 500.0;
        self.engine.max_rpm = 4000.0;
        self.engine.max_torque = 500.0; // Note actual torque for M1 is around 5000 but we need a reduced mass in order to keep the simulation sane

        self.transmission.shift_down_rpm = 1000.0;
        self.transmission.shift_up_rpm = 3500.0;
        self.transmission.gear_ratios.clearRetainingCapacity();
        try self.transmission.gear_ratios.appendSlice(allocator, &.{ 4.0, 3.0, 2.0, 1.0 });
        self.transmission.reverse_gear_ratios.clearRetainingCapacity();
        try self.transmission.reverse_gear_ratios.appendSlice(allocator, &.{ -4.0, -3.0 });
        return self;
    }

    /// Constructor of settings that are not on the heap (the owner calls `base.deinit()`)
    pub fn init(allocator: Allocator) Allocator.Error!TrackedVehicleControllerSettings {
        return initDerived(TrackedVehicleControllerSettings, allocator);
    }

    /// new TrackedVehicleControllerSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*TrackedVehicleControllerSettings {
        const self = try allocator.create(TrackedVehicleControllerSettings);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for VehicleConstraintSettings.restoreBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// ~TrackedVehicleControllerSettings: frees the engine curve, the gear ratios and the wheel indices of the tracks
    pub fn destruct(self: *TrackedVehicleControllerSettings) void {
        const allocator = self.base.allocator;
        self.engine.deinit(allocator);
        self.transmission.deinit(allocator);
        for (&self.tracks) |*t|
            t.deinit(allocator);
    }

    /// Upcasts (implicit in C++)
    pub fn asControllerSettings(self: *TrackedVehicleControllerSettings) *VehicleControllerSettings {
        return &self.base;
    }

    pub fn asControllerSettingsConst(self: *const TrackedVehicleControllerSettings) *const VehicleControllerSettings {
        return &self.base;
    }

    // See: VehicleControllerSettings
    pub fn constructController(self: *const TrackedVehicleControllerSettings, allocator: Allocator, constraint: *VehicleConstraint) Allocator.Error!*VehicleController {
        const controller = try TrackedVehicleController.create(allocator, self, constraint);
        return &controller.base;
    }

    pub fn saveBinaryState(self: *const TrackedVehicleControllerSettings, stream: StreamOut) void {
        self.engine.saveBinaryState(stream);

        self.transmission.saveBinaryState(stream);

        for (&self.tracks) |*t|
            t.saveBinaryState(stream);
    }

    pub fn restoreBinaryState(self: *TrackedVehicleControllerSettings, stream: StreamIn) Allocator.Error!void {
        const allocator = self.base.allocator;

        try self.engine.restoreBinaryState(allocator, stream);

        try self.transmission.restoreBinaryState(allocator, stream);

        for (&self.tracks) |*t|
            try t.restoreBinaryState(allocator, stream);
    }
};

/// Runtime controller class for vehicle with tank tracks
pub const TrackedVehicleController = struct {
    pub const overrides = .{ .getSettings, .constructWheel, .allowSleep, .preCollide, .postCollide, .solveLongitudinalAndLateralConstraints, .saveState, .restoreState, .draw };

    base: VehicleController,

    // Control information

    /// Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
    forward_input: f32 = 0.0,
    /// Value between -1 and 1 indicating an extra multiplier to the rotation rate of the left track (used for steering)
    left_ratio: f32 = 1.0,
    /// Value between -1 and 1 indicating an extra multiplier to the rotation rate of the right track (used for steering)
    right_ratio: f32 = 1.0,
    /// Value between 0 and 1 indicating how strong the brake pedal is pressed
    brake_input: f32 = 0.0,

    // Simulation information

    /// Engine state of the vehicle
    engine: VehicleEngine = .{},
    /// Transmission state of the vehicle
    transmission: VehicleTransmission = .{},
    /// Tracks of the vehicle
    tracks: VehicleTracks = .{ .{}, .{} },

    // Debug settings (JPH_DEBUG_RENDERER: `void` without -Ddebug_renderer)

    /// Position (in local space of the body) of the RPM meter when drawing the constraint
    rpm_meter_position: if (Core.debug_renderer) Vec3 else void = if (Core.debug_renderer) Vec3.init(0, 1, 0) else {},
    /// Size of the RPM meter when drawing the constraint
    rpm_meter_size: if (Core.debug_renderer) f32 else void = if (Core.debug_renderer) 0.5 else {},

    /// Constructor of the base part of a class T derived from TrackedVehicleController. On failure nothing is left
    /// allocated.
    pub fn initDerived(comptime T: type, allocator: Allocator, settings: *const TrackedVehicleControllerSettings, constraint: *VehicleConstraint) Allocator.Error!TrackedVehicleController {
        var self: TrackedVehicleController = .{ .base = .init(T, allocator, constraint) };
        errdefer self.destruct();

        // Copy engine settings
        self.engine.base = try settings.engine.clone(allocator);
        if (Core.enable_asserts) {
            std.debug.assert(settings.engine.min_rpm >= 0.0);
            std.debug.assert(settings.engine.min_rpm <= settings.engine.max_rpm);
        }
        self.engine.setCurrentRPM(self.engine.base.min_rpm);

        // Copy transmission settings
        self.transmission.base = try settings.transmission.clone(allocator);
        if (Core.enable_asserts) {
            for (settings.transmission.gear_ratios.items) |r|
                std.debug.assert(r > 0.0);
            for (settings.transmission.reverse_gear_ratios.items) |r|
                std.debug.assert(r < 0.0);
            std.debug.assert(settings.transmission.switch_time >= 0.0);
            std.debug.assert(settings.transmission.shift_down_rpm > 0.0);
            std.debug.assert(settings.transmission.mode != .auto or settings.transmission.shift_up_rpm < settings.engine.max_rpm);
            std.debug.assert(settings.transmission.shift_up_rpm > settings.transmission.shift_down_rpm);
        }

        // Copy track settings
        for (&self.tracks, &settings.tracks) |*t, *d| {
            t.base = try d.clone(allocator);
            if (Core.enable_asserts) {
                std.debug.assert(d.inertia >= 0.0);
                std.debug.assert(d.angular_damping >= 0.0);
                std.debug.assert(d.max_brake_torque >= 0.0);
                std.debug.assert(d.differential_ratio > 0.0);
            }
        }

        return self;
    }

    /// Constructor
    pub fn init(allocator: Allocator, settings: *const TrackedVehicleControllerSettings, constraint: *VehicleConstraint) Allocator.Error!TrackedVehicleController {
        return initDerived(TrackedVehicleController, allocator, settings, constraint);
    }

    /// new TrackedVehicleController(inSettings, inConstraint) (VehicleConstraint destroys it)
    pub fn create(allocator: Allocator, settings: *const TrackedVehicleControllerSettings, constraint: *VehicleConstraint) Allocator.Error!*TrackedVehicleController {
        const self = try allocator.create(TrackedVehicleController);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator, settings, constraint);
        return self;
    }

    /// ~TrackedVehicleController
    pub fn destruct(self: *TrackedVehicleController) void {
        const allocator = self.base.allocator;
        self.engine.deinit(allocator);
        self.transmission.deinit(allocator);
        for (&self.tracks) |*t|
            t.deinit(allocator);
    }

    /// Upcasts (implicit in C++)
    pub fn asController(self: *TrackedVehicleController) *VehicleController {
        return &self.base;
    }

    pub fn asControllerConst(self: *const TrackedVehicleController) *const VehicleController {
        return &self.base;
    }

    /// Set input from driver
    /// @param inForward Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
    /// @param inLeftRatio Value between -1 and 1 indicating an extra multiplier to the rotation rate of the left track (used for steering)
    /// @param inRightRatio Value between -1 and 1 indicating an extra multiplier to the rotation rate of the right track (used for steering)
    /// @param inBrake Value between 0 and 1 indicating how strong the brake pedal is pressed
    pub fn setDriverInput(self: *TrackedVehicleController, forward: f32, left_ratio: f32, right_ratio: f32, brake: f32) void {
        if (Core.enable_asserts) std.debug.assert(left_ratio != 0.0 and right_ratio != 0.0);
        self.forward_input = forward;
        self.left_ratio = left_ratio;
        self.right_ratio = right_ratio;
        self.brake_input = brake;
    }

    /// Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
    pub fn setForwardInput(self: *TrackedVehicleController, forward: f32) void {
        self.forward_input = forward;
    }

    pub fn getForwardInput(self: *const TrackedVehicleController) f32 {
        return self.forward_input;
    }

    /// Value between -1 and 1 indicating an extra multiplier to the rotation rate of the left track (used for steering)
    pub fn setLeftRatio(self: *TrackedVehicleController, left_ratio: f32) void {
        if (Core.enable_asserts) std.debug.assert(left_ratio != 0.0);
        self.left_ratio = left_ratio;
    }

    pub fn getLeftRatio(self: *const TrackedVehicleController) f32 {
        return self.left_ratio;
    }

    /// Value between -1 and 1 indicating an extra multiplier to the rotation rate of the right track (used for steering)
    pub fn setRightRatio(self: *TrackedVehicleController, right_ratio: f32) void {
        if (Core.enable_asserts) std.debug.assert(right_ratio != 0.0);
        self.right_ratio = right_ratio;
    }

    pub fn getRightRatio(self: *const TrackedVehicleController) f32 {
        return self.right_ratio;
    }

    /// Value between 0 and 1 indicating how strong the brake pedal is pressed
    pub fn setBrakeInput(self: *TrackedVehicleController, brake: f32) void {
        self.brake_input = brake;
    }

    pub fn getBrakeInput(self: *const TrackedVehicleController) f32 {
        return self.brake_input;
    }

    /// Get current engine state
    pub fn getEngine(self: *const TrackedVehicleController) *const VehicleEngine {
        return &self.engine;
    }

    /// Get current engine state (writable interface, allows you to make changes to the configuration which will take effect the next time step)
    pub fn getEngineMut(self: *TrackedVehicleController) *VehicleEngine {
        return &self.engine;
    }

    /// Get current transmission state
    pub fn getTransmission(self: *const TrackedVehicleController) *const VehicleTransmission {
        return &self.transmission;
    }

    /// Get current transmission state (writable interface, allows you to make changes to the configuration which will take effect the next time step)
    pub fn getTransmissionMut(self: *TrackedVehicleController) *VehicleTransmission {
        return &self.transmission;
    }

    /// Get the tracks this vehicle has
    pub fn getTracks(self: *const TrackedVehicleController) *const VehicleTracks {
        return &self.tracks;
    }

    /// Get the tracks this vehicle has (writable interface, allows you to make changes to the configuration which will take effect the next time step)
    pub fn getTracksMut(self: *TrackedVehicleController) *VehicleTracks {
        return &self.tracks;
    }

    /// Debug drawing of RPM meter (JPH_DEBUG_RENDERER)
    pub const setRPMMeter = if (Core.debug_renderer) setRPMMeterImpl else {};
    fn setRPMMeterImpl(self: *TrackedVehicleController, position: Vec3, size: f32) void {
        self.rpm_meter_position = position;
        self.rpm_meter_size = size;
    }

    // See: VehicleController
    pub fn getSettings(self: *const TrackedVehicleController) Allocator.Error!Ref(VehicleControllerSettings) {
        const allocator = self.base.allocator;
        const settings = try TrackedVehicleControllerSettings.create(allocator);
        var result = Ref(VehicleControllerSettings).init(&settings.base);
        errdefer result.deinit();
        try settings.engine.assign(allocator, &self.engine.base);
        try settings.transmission.assign(allocator, &self.transmission.base);
        for (&settings.tracks, &self.tracks) |*s, *t|
            try s.assign(allocator, &t.base);
        return result;
    }

    /// Synchronize angular velocities of left and right tracks according to their ratios (protected in Jolt)
    pub fn syncLeftRightTracks(self: *TrackedVehicleController) void {
        // Apply left to right ratio according to track inertias
        const tl = &self.tracks[@intFromEnum(TrackSide.left)];
        const tr = &self.tracks[@intFromEnum(TrackSide.right)];

        if (self.left_ratio * self.right_ratio > 0.0) {
            // Solve: (tl.mAngularVelocity + dl) / (tr.mAngularVelocity + dr) = mLeftRatio / mRightRatio and dl * tr.mInertia = -dr * tl.mInertia, where dl/dr are the delta angular velocities for left and right tracks
            const impulse = (self.left_ratio * tr.angular_velocity - self.right_ratio * tl.angular_velocity) / (self.left_ratio * tr.base.inertia + self.right_ratio * tl.base.inertia);
            tl.angular_velocity += impulse * tl.base.inertia;
            tr.angular_velocity -= impulse * tr.base.inertia;
        } else {
            // Solve: (tl.mAngularVelocity + dl) / (tr.mAngularVelocity + dr) = mLeftRatio / mRightRatio and dl * tr.mInertia = dr * tl.mInertia, where dl/dr are the delta angular velocities for left and right tracks
            const impulse = (self.left_ratio * tr.angular_velocity - self.right_ratio * tl.angular_velocity) / (self.right_ratio * tl.base.inertia - self.left_ratio * tr.base.inertia);
            tl.angular_velocity += impulse * tl.base.inertia;
            tr.angular_velocity += impulse * tr.base.inertia;
        }
    }

    pub fn constructWheel(self: *const TrackedVehicleController, allocator: Allocator, wheel: *const WheelSettings) Allocator.Error!*Wheel {
        _ = self;
        const w = try WheelTV.create(allocator, wheel.cast(WheelSettingsTV));
        return &w.base;
    }

    pub fn allowSleep(self: *const TrackedVehicleController) bool {
        return self.forward_input == 0.0 // No user input
        and self.transmission.allowSleep() // Transmission is not shifting
        and self.engine.allowSleep(); // Engine is idling
    }

    pub fn preCollide(self: *TrackedVehicleController, delta_time: f32, physics_system: *PhysicsSystem) void {
        _ = physics_system;

        const wheels = self.base.constraint.getWheelsMut();

        // Fill in track index
        for (&self.tracks, 0..) |*t, track_index| {
            for (t.base.wheels.items) |w|
                virtual.downcast(WheelTV, wheels[w]).track_index = @intCast(track_index);
        }

        // Angular damping: dw/dt = -c * w
        // Solution: w(t) = w(0) * e^(-c * t) or w2 = w1 * e^(-c * dt)
        // Taylor expansion of e^(-c * dt) = 1 - c * dt + ...
        // Since dt is usually in the order of 1/60 and c is a low number too this approximation is good enough
        for (&self.tracks) |*t|
            t.angular_velocity *= math.max(@as(f32, 0.0), 1.0 - t.base.angular_damping * delta_time);
    }

    pub fn postCollide(self: *TrackedVehicleController, delta_time: f32, physics_system: *PhysicsSystem) void {
        _ = physics_system;

        const constraint = self.base.constraint;
        const wheels = constraint.getWheelsMut();

        // Update wheel angle, do this before applying torque to the wheels (as friction will slow them down again)
        for (wheels, 0..) |w_base, wheel_index| {
            const w = virtual.downcast(WheelTV, w_base);
            w.update(@intCast(wheel_index), delta_time, constraint);
        }

        // First calculate engine speed based on speed of all wheels
        var can_engine_apply_torque = false;
        if (self.transmission.getCurrentGear() != 0 and self.transmission.getClutchFriction() > 1.0e-3) {
            const transmission_ratio = self.transmission.getCurrentRatio();
            const forward = transmission_ratio >= 0.0;
            var fastest_wheel_speed: f32 = if (forward) -math.flt_max else math.flt_max;
            for (&self.tracks) |*t| {
                if (forward)
                    fastest_wheel_speed = math.max(fastest_wheel_speed, t.angular_velocity * t.base.differential_ratio)
                else
                    fastest_wheel_speed = math.min(fastest_wheel_speed, t.angular_velocity * t.base.differential_ratio);
                for (t.base.wheels.items) |w| {
                    if (wheels[w].hasContact()) {
                        can_engine_apply_torque = true;
                        break;
                    }
                }
            }

            // Update RPM only if the tracks are connected to the engine
            if (fastest_wheel_speed > -math.flt_max and fastest_wheel_speed < math.flt_max)
                self.engine.setCurrentRPM(fastest_wheel_speed * self.transmission.getCurrentRatio() * VehicleEngine.angular_velocity_to_rpm);
        } else {
            // Update engine with damping
            self.engine.applyDamping(delta_time);

            // In auto transmission mode, don't accelerate the engine when switching gears
            const forward_input: f32 = if (self.transmission.base.mode == .manual) @abs(self.forward_input) else 0.0;

            // Engine not connected to wheels, update RPM based on engine inertia alone
            self.engine.applyTorque(self.engine.getTorque(forward_input), delta_time);
        }

        // Update transmission
        // Note: only allow switching gears up when the tracks are rolling in the same direction
        self.transmission.update(delta_time, self.engine.getCurrentRPM(), self.forward_input, self.left_ratio * self.right_ratio > 0.0 and can_engine_apply_torque);

        // Calculate the amount of torque the transmission gives to the differentials
        const transmission_ratio = self.transmission.getCurrentRatio();
        const transmission_torque = self.transmission.getClutchFriction() * transmission_ratio * self.engine.getTorque(@abs(self.forward_input));
        if (transmission_torque != 0.0) {
            // Apply the transmission torque to the wheels
            for (&self.tracks, 0..) |*t, i| {
                // Get wheel rotation ratio for this track
                const ratio = if (i == 0) self.left_ratio else self.right_ratio;

                // Calculate the max angular velocity of the driven wheel of the track given current engine RPM
                // Note this adds 0.1% slop to avoid numerical accuracy issues
                const track_max_angular_velocity = self.engine.getCurrentRPM() / (transmission_ratio * t.base.differential_ratio * ratio * VehicleEngine.angular_velocity_to_rpm) * 1.001;

                // Calculate torque on the driven wheel
                const differential_torque = t.base.differential_ratio * ratio * transmission_torque;

                // Apply torque to driven wheel
                if (t.angular_velocity * track_max_angular_velocity < 0.0 or @abs(t.angular_velocity) < @abs(track_max_angular_velocity))
                    t.angular_velocity += differential_torque * delta_time / t.base.inertia;
            }
        }

        // Ensure that we have the correct ratio between the two tracks
        self.syncLeftRightTracks();

        // Braking
        for (&self.tracks) |*t| {
            // Calculate brake torque
            var brake_torque = self.brake_input * t.base.max_brake_torque;
            if (brake_torque > 0.0) {
                // Calculate how much torque is needed to stop the track from rotating in this time step
                const brake_torque_to_lock_track = @abs(t.angular_velocity) * t.base.inertia / delta_time;
                if (brake_torque > brake_torque_to_lock_track) {
                    // Wheels are locked
                    t.angular_velocity = 0.0;
                    brake_torque -= brake_torque_to_lock_track;
                } else {
                    // Slow down the track
                    t.angular_velocity -= math.sign(t.angular_velocity) * brake_torque * delta_time / t.base.inertia;
                }
            }

            if (brake_torque > 0.0) {
                // Sum the radius of all wheels touching the floor
                var total_radius: f32 = 0.0;
                for (t.base.wheels.items) |wheel_index| {
                    const w = virtual.downcast(WheelTV, wheels[wheel_index]);

                    if (w.base.hasContact())
                        total_radius += w.getSettings().base.radius;
                }

                if (total_radius > 0.0) {
                    brake_torque /= total_radius;
                    for (t.base.wheels.items) |wheel_index| {
                        const w = virtual.downcast(WheelTV, wheels[wheel_index]);
                        if (w.base.hasContact()) {
                            // Impulse: p = F * dt = Torque / Wheel_Radius * dt, Torque = Total_Torque * Wheel_Radius / Summed_Radius => p = Total_Torque * dt / Summed_Radius
                            w.brake_impulse = brake_torque * delta_time;
                        }
                    }
                }
            }
        }

        // Update wheel angular velocity based on that of the track
        for (wheels) |w_base| {
            const w = virtual.downcast(WheelTV, w_base);
            w.calculateAngularVelocity(constraint);
        }
    }

    pub fn solveLongitudinalAndLateralConstraints(self: *TrackedVehicleController, delta_time: f32) bool {
        _ = delta_time;

        var impulse = false;

        const constraint = self.base.constraint;
        for (constraint.getWheelsMut()) |w_base| {
            if (w_base.hasContact()) {
                const w = virtual.downcast(WheelTV, w_base);
                const settings = w.getSettings();
                const track = &self.tracks[@intCast(w.track_index)];

                // Calculate max impulse that we can apply on the ground
                const max_longitudinal_friction_impulse = w.combined_longitudinal_friction * w.base.getSuspensionLambda();

                // Calculate relative velocity between wheel contact point and floor in longitudinal direction
                const relative_velocity = constraint.getVehicleBody().getPointVelocity(w.base.getContactPosition()).sub(w.base.getContactPointVelocity());
                const relative_longitudinal_velocity = relative_velocity.dot(w.base.getContactLongitudinal());

                // Calculate brake force to apply
                var min_longitudinal_impulse: f32 = undefined;
                var max_longitudinal_impulse: f32 = undefined;
                if (w.brake_impulse != 0.0) {
                    // Limit brake force by max tire friction
                    const brake_impulse = math.min(w.brake_impulse, max_longitudinal_friction_impulse);

                    // Check which direction the brakes should be applied (we don't want to apply an impulse that would accelerate the vehicle)
                    if (relative_longitudinal_velocity >= 0.0) {
                        min_longitudinal_impulse = -brake_impulse;
                        max_longitudinal_impulse = 0.0;
                    } else {
                        min_longitudinal_impulse = 0.0;
                        max_longitudinal_impulse = brake_impulse;
                    }

                    // Longitudinal impulse, note that we assume that once the wheels are locked that the brakes have more than enough torque to keep the wheels locked so we exclude any rotation deltas
                    if (w.base.solveLongitudinalConstraintPart(constraint, min_longitudinal_impulse, max_longitudinal_impulse))
                        impulse = true;
                } else {
                    // Assume we want to apply an angular impulse that makes the delta velocity between track and ground zero in one time step, calculate the amount of linear impulse needed to do that
                    const desired_angular_velocity = relative_longitudinal_velocity / settings.base.radius;
                    const linear_impulse = (track.angular_velocity - desired_angular_velocity) * track.base.inertia / settings.base.radius;

                    // Limit the impulse by max track friction
                    const prev_lambda = w.base.getLongitudinalLambda();
                    min_longitudinal_impulse = math.clamp(prev_lambda + linear_impulse, -max_longitudinal_friction_impulse, max_longitudinal_friction_impulse);
                    max_longitudinal_impulse = min_longitudinal_impulse;

                    // Longitudinal impulse
                    if (w.base.solveLongitudinalConstraintPart(constraint, min_longitudinal_impulse, max_longitudinal_impulse))
                        impulse = true;

                    // Update the angular velocity of the track according to the lambda that was applied
                    track.angular_velocity -= (w.base.getLongitudinalLambda() - prev_lambda) * settings.base.radius / track.base.inertia;
                    self.syncLeftRightTracks();
                }
            }
        }

        for (constraint.getWheelsMut()) |w_base| {
            if (w_base.hasContact()) {
                const w = virtual.downcast(WheelTV, w_base);

                // Update angular velocity of wheel for the next iteration
                w.calculateAngularVelocity(constraint);

                // Lateral friction
                const max_lateral_friction_impulse = w.combined_lateral_friction * w.base.getSuspensionLambda();
                if (w.base.solveLateralConstraintPart(constraint, -max_lateral_friction_impulse, max_lateral_friction_impulse))
                    impulse = true;
            }
        }

        return impulse;
    }

    // See: VehicleController (JPH_DEBUG_RENDERER)
    pub const draw = if (Core.debug_renderer) drawImpl else {};
    fn drawImpl(self: *const TrackedVehicleController, renderer: *DebugRenderer) void {
        const constraint = self.base.constraint;
        const constraint_size = constraint.base.getDrawConstraintSize();

        // Draw RPM
        const body = constraint.getVehicleBody();
        const rpm_meter_up = body.getRotation().mulVec3(constraint.getLocalUp());
        const rpm_meter_pos = body.getPosition().addVec3(body.getRotation().mulVec3(self.rpm_meter_position));
        const rpm_meter_fwd = body.getRotation().mulVec3(constraint.getLocalForward());
        self.engine.drawRPM(renderer, rpm_meter_pos, rpm_meter_fwd, rpm_meter_up, self.rpm_meter_size, self.transmission.base.shift_down_rpm, self.transmission.base.shift_up_rpm);

        // Draw current vehicle state
        // StringFormat("Forward: %.1f, LRatio: %.1f, RRatio: %.1f, Brake: %.1f\nGear: %d, Clutch: %.1f, EngineRPM: %.0f, V: %.1f km/h",
        // ...): its buffer holds 1023 characters
        var status_buffer: [1023]u8 = undefined;
        var status: std.Io.Writer = .fixed(&status_buffer);
        status.print("Forward: {f}, LRatio: {f}, RRatio: {f}, Brake: {f}\nGear: {d}, Clutch: {f}, EngineRPM: {f}, V: {f} km/h", .{
            StringTools.cppFixed(self.forward_input, 1),
            StringTools.cppFixed(self.left_ratio, 1),
            StringTools.cppFixed(self.right_ratio, 1),
            StringTools.cppFixed(self.brake_input, 1),
            self.transmission.getCurrentGear(),
            StringTools.cppFixed(self.transmission.getClutchFriction(), 1),
            StringTools.cppFixed(self.engine.getCurrentRPM(), 0),
            StringTools.cppFixed(@as(f64, body.getLinearVelocity().length()) * 3.6, 1),
        }) catch {};
        renderer.drawText3D(body.getPosition(), status.buffered(), .{ .color = Color.white, .height = constraint_size });

        for (&self.tracks) |*t| {
            const w = virtual.downcast(WheelTV, constraint.getWheels()[t.base.driven_wheel]);
            const settings = w.base.getSettings();

            // Calculate where the suspension attaches to the body in world space
            const ws_position = body.getCenterOfMassPosition().addVec3(body.getRotation().mulVec3(settings.position.sub(body.getShape().getCenterOfMass())));

            // Jolt draws this text into DebugRenderer::sInstance (R4: the renderer of this draw)
            var text_buffer: [1023]u8 = undefined;
            var text: std.Io.Writer = .fixed(&text_buffer);
            text.print("W: {f}", .{StringTools.cppFixed(t.angular_velocity, 1)}) catch {};
            renderer.drawText3D(ws_position, text.buffered(), .{ .color = Color.white, .height = constraint_size });
        }

        const body_transform = body.getWorldTransform();

        for (constraint.getWheels()) |w_base| {
            const w = virtual.downcast(WheelTV, w_base);
            const settings = w.base.getSettings();

            // Calculate where the suspension attaches to the body in world space
            const ws_position = body_transform.mulVec3(settings.position);
            const ws_direction = body_transform.multiply3x3(settings.suspension_direction);

            // Draw suspension
            const min_suspension_pos = ws_position.addVec3(ws_direction.mulScalar(settings.suspension_min_length));
            const max_suspension_pos = ws_position.addVec3(ws_direction.mulScalar(settings.suspension_max_length));
            renderer.drawLine(ws_position, min_suspension_pos, Color.red);
            renderer.drawLine(min_suspension_pos, max_suspension_pos, Color.green);

            // Draw current length
            const wheel_pos = ws_position.addVec3(ws_direction.mulScalar(w.base.getSuspensionLength()));
            renderer.drawMarker(wheel_pos, if (w.base.getSuspensionLength() < settings.suspension_min_length) Color.red else Color.green, constraint_size);

            // Draw wheel basis
            const basis = constraint.getWheelLocalBasis(&w.base);
            const wheel_forward = body_transform.multiply3x3(basis.forward);
            const wheel_up = body_transform.multiply3x3(basis.up);
            const wheel_right = body_transform.multiply3x3(basis.right);
            const steering_axis = body_transform.multiply3x3(settings.steering_axis);
            renderer.drawLine(wheel_pos, wheel_pos.addVec3(wheel_forward), Color.red);
            renderer.drawLine(wheel_pos, wheel_pos.addVec3(wheel_up), Color.green);
            renderer.drawLine(wheel_pos, wheel_pos.addVec3(wheel_right), Color.blue);
            renderer.drawLine(wheel_pos, wheel_pos.addVec3(steering_axis), Color.yellow);

            // Draw wheel
            var wheel_transform = rmat44FromColumns(Vec4.fromVec3W(wheel_up, 0.0), Vec4.fromVec3W(wheel_right, 0.0), Vec4.fromVec3W(wheel_forward, 0.0), wheel_pos);
            wheel_transform.setRotation(wheel_transform.getRotation().mul(Mat44.rotationY(-w.base.getRotationAngle())));
            renderer.drawCylinder(wheel_transform, settings.width * 0.5, settings.radius, if (w.base.getSuspensionLength() <= settings.suspension_min_length) Color.red else Color.green, .{ .cast_shadow = .off, .draw_mode = .wireframe });

            if (w.base.hasContact()) {
                // Draw contact
                renderer.drawLine(w.base.getContactPosition(), w.base.getContactPosition().addVec3(w.base.getContactNormal()), Color.yellow);
                renderer.drawLine(w.base.getContactPosition(), w.base.getContactPosition().addVec3(w.base.getContactLongitudinal()), Color.red);
                renderer.drawLine(w.base.getContactPosition(), w.base.getContactPosition().addVec3(w.base.getContactLateral()), Color.blue);

                // Jolt draws this text into DebugRenderer::sInstance (R4: the renderer of this draw)
                var text_buffer: [1023]u8 = undefined;
                var text: std.Io.Writer = .fixed(&text_buffer);
                text.print("S: {f}", .{StringTools.cppFixed(w.base.getSuspensionLength(), 2)}) catch {};
                renderer.drawText3D(w.base.getContactPosition(), text.buffered(), .{ .color = Color.white, .height = constraint_size });
            }
        }
    }

    pub fn saveState(self: *const TrackedVehicleController, stream: *StateRecorder) void {
        stream.write(self.forward_input);
        stream.write(self.left_ratio);
        stream.write(self.right_ratio);
        stream.write(self.brake_input);

        self.engine.saveState(stream);
        self.transmission.saveState(stream);

        for (&self.tracks) |*t|
            t.saveState(stream);
    }

    pub fn restoreState(self: *TrackedVehicleController, stream: *StateRecorder) void {
        stream.read(&self.forward_input);
        stream.read(&self.left_ratio);
        stream.read(&self.right_ratio);
        stream.read(&self.brake_input);

        self.engine.restoreState(stream);
        self.transmission.restoreState(stream);

        for (&self.tracks) |*t|
            t.restoreState(stream);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt has no unit tests of the tracked controller): the settings classes and their binary state, the wheel and
// controller functions on a tank driving over a floor (forward, steering with the track ratios, turning on the spot,
// braking, sleeping), the settings and state round trips and allocation failure. The bit exact comparison with Jolt is
// the simulation parity (ZoltParity/Physics/SimScenes.zig, the vehicle scenes) and VehiclePartsParity.zig.

/// RMat44(Vec4Arg inC1, Vec4Arg inC2, Vec4Arg inC3, RVec3Arg inC4) in both precisions (JPH_DEBUG_RENDERER)
fn rmat44FromColumns(c1: Vec4, c2: Vec4, c3: Vec4, c4: RVec3) RMat44 {
    return if (Core.double_precision) DMat44.init(c1, c2, c3, c4) else Mat44.fromColumnsTranslation(c1, c2, c3, c4);
}

const testing = std.testing;
const TempAllocatorImpl = @import("../../Core/TempAllocator.zig").TempAllocatorImpl;
const JobSystemSingleThreaded = @import("../../Core/JobSystemSingleThreaded.zig").JobSystemSingleThreaded;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");
const Quat = @import("../../Math/Quat.zig").Quat;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const Body = @import("../Body/Body.zig").Body;
const BodyCreationSettings = @import("../Body/BodyCreationSettings.zig").BodyCreationSettings;
const MotionType = @import("../Body/MotionType.zig").MotionType;
const ObjectLayer = @import("../Collision/ObjectLayer.zig").ObjectLayer;
const BroadPhaseLayerInterfaceTable = @import("../Collision/BroadPhase/BroadPhaseLayerInterfaceTable.zig").BroadPhaseLayerInterfaceTable;
const ObjectVsBroadPhaseLayerFilterTable = @import("../Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.zig").ObjectVsBroadPhaseLayerFilterTable;
const ObjectLayerPairFilterTable = @import("../Collision/ObjectLayerPairFilterTable.zig").ObjectLayerPairFilterTable;
const Shape = @import("../Collision/Shape/Shape.zig").Shape;
const BoxShape = @import("../Collision/Shape/BoxShape.zig").BoxShape;
const OffsetCenterOfMassShape = @import("../Collision/Shape/OffsetCenterOfMassShape.zig").OffsetCenterOfMassShape;
const ConstraintFile = @import("../Constraints/Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const ConstraintSettings = ConstraintFile.ConstraintSettings;
const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;
const VehicleConstraintSettings = @import("VehicleConstraint.zig").VehicleConstraintSettings;
const VehicleCollisionTesterFile = @import("VehicleCollisionTester.zig");
const VehicleCollisionTester = VehicleCollisionTesterFile.VehicleCollisionTester;
const VehicleCollisionTesterRay = VehicleCollisionTesterFile.VehicleCollisionTesterRay;
const WheelSettingsWV = @import("WheeledVehicleController.zig").WheelSettingsWV;

/// The object layers of the tests
const TestLayers = struct {
    const non_moving: ObjectLayer = 0;
    const moving: ObjectLayer = 1;
    const count = 2;
};

/// A physics system with a floor (friction 0.2), a temp allocator and a job system (in place: it must not move)
const TestWorld = struct {
    layer_interface: BroadPhaseLayerInterfaceTable,
    pair_filter: ObjectLayerPairFilterTable,
    ovb_filter: ObjectVsBroadPhaseLayerFilterTable,
    system: PhysicsSystem,
    temp: TempAllocatorImpl,
    jobs: JobSystemSingleThreaded,

    fn init(self: *TestWorld, allocator: Allocator) !void {
        self.layer_interface = try .init(allocator, TestLayers.count, 2);
        errdefer self.layer_interface.deinit();
        self.layer_interface.mapObjectToBroadPhaseLayer(TestLayers.non_moving, .init(0));
        self.layer_interface.mapObjectToBroadPhaseLayer(TestLayers.moving, .init(1));
        self.pair_filter = try .init(allocator, TestLayers.count);
        errdefer self.pair_filter.deinit();
        self.pair_filter.enableCollision(TestLayers.non_moving, TestLayers.moving);
        self.pair_filter.enableCollision(TestLayers.moving, TestLayers.moving);
        self.ovb_filter = try .init(allocator, &self.layer_interface.base, 2, &self.pair_filter.base, TestLayers.count);
        errdefer self.ovb_filter.deinit();
        self.system = .empty;
        try self.system.init(allocator, testing.io, 64, 0, 1024, 1024, &self.layer_interface.base, &self.ovb_filter.base, &self.pair_filter.base, .{});
        errdefer self.system.deinit();
        self.temp = try .init(allocator, 1024 * 1024);
        errdefer self.temp.deinit();
        self.jobs = try .init(allocator, testing.io, 256);
        errdefer self.jobs.deinit();

        // The floor
        const floor = try BoxShape.create(allocator, Vec3.init(100, 1, 100), .{ .convex_radius = 0.0 });
        _ = try self.addBody(floor.asShape(), RVec3.init(0, -1, 0), .static, 0);
    }

    fn deinit(self: *TestWorld) void {
        self.jobs.deinit();
        self.temp.deinit();
        self.system.deinit();
        self.ovb_filter.deinit();
        self.pair_filter.deinit();
        self.layer_interface.deinit();
    }

    /// `seconds` of updates of 1/60 s
    fn simulate(self: *TestWorld, seconds: f32) !void {
        const steps: u32 = @intFromFloat(@round(seconds * 60.0));
        for (0..steps) |_| {
            const errors = try self.system.update(1.0 / 60.0, 1, self.temp.tempAllocator(), self.jobs.jobSystem());
            try testing.expect(errors.isNone());
        }
    }

    /// A body added to the system (a static body in the non moving layer, a dynamic one in the moving layer with the
    /// mass `mass`)
    fn addBody(self: *TestWorld, shape: *const Shape, position: RVec3, motion_type: MotionType, mass: f32) !*Body {
        var settings: BodyCreationSettings = .initWithShape(shape, position, Quat.identity(), motion_type, if (motion_type == .static) TestLayers.non_moving else TestLayers.moving);
        defer settings.deinit();
        if (mass > 0.0) {
            settings.override_mass_properties = .calculate_inertia;
            settings.mass_properties_override.mass = mass;
        }
        const bi = self.system.getBodyInterfaceMut();
        const body = (try bi.createBody(&settings)) orelse return error.OutOfBodies;
        try bi.addBody(body.getID(), if (motion_type == .static) .dont_activate else .activate);
        return body;
    }
};

/// The radius of wheel 0 of the test tank (the other wheels have the default radius 0.3)
const small_wheel_radius = 0.25;

/// The tank of the tests (SimScenes' addTestTank): 8 wheels at y = -0.3, track 0 (left, x = 1.2) has wheels 0..3, track
/// 1 (right, x = -1.2) wheels 4..7, the driven wheel of a track is its last wheel. Wheel 0 is smaller. The caller calls
/// `asSettings().deinit()`.
fn makeTankSettings(allocator: Allocator) !VehicleConstraintSettings {
    var vehicle = VehicleConstraintSettings.init(allocator);
    errdefer vehicle.asSettings().deinit();
    vehicle.max_pitch_roll_angle = math.degreesToRadians(60.0);

    for (0..2) |t| {
        for (0..4) |i| {
            const w = try WheelSettingsTV.create(allocator);
            var w_ref = Ref(WheelSettings).init(&w.base);
            defer w_ref.deinit();
            w.base.position = Vec3.init(if (t == 0) 1.2 else -1.2, -0.3, 1.5 - @as(f32, @floatFromInt(i)));
            w.base.suspension_spring.frequency = 1.0;
            if (t == 0 and i == 0) w.base.radius = small_wheel_radius;
            w.longitudinal_friction = 4.0 - 0.5 * @as(f32, @floatFromInt(i));
            try vehicle.addWheel(&w.base);
        }
    }

    const controller = try TrackedVehicleControllerSettings.create(allocator);
    vehicle.setController(&controller.base);
    for (&controller.tracks, 0..) |*track, t| {
        const first: u32 = @intCast(4 * t);
        try track.wheels.appendSlice(allocator, &.{ first, first + 1, first + 2, first + 3 });
        track.driven_wheel = first + 3;
    }
    return vehicle;
}

/// The body of the test tank: a box with its center of mass lowered by its half height, 4000 kg
fn addTankBody(world: *TestWorld, allocator: Allocator, position: RVec3) !*Body {
    const box = try BoxShape.create(allocator, Vec3.init(1.2, 0.4, 2.0), .{});
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();
    const shape = try OffsetCenterOfMassShape.create(allocator, box.asShape(), Vec3.init(0, -0.4, 0));
    return world.addBody(shape.asShape(), position, .dynamic, 4000.0);
}

/// The test tank in the world (on the floor in the first update at y = 0.95), with a ray collision tester
fn addTank(world: *TestWorld, allocator: Allocator, position: RVec3) !*VehicleConstraint {
    const body = try addTankBody(world, allocator, position);
    var settings = try makeTankSettings(allocator);
    defer settings.asSettings().deinit();
    const constraint = try VehicleConstraint.create(allocator, body, &settings);
    var constraint_ref = Ref(Constraint).init(constraint.asConstraint());
    defer constraint_ref.deinit();
    var tester = RefConst(VehicleCollisionTester).init(&(try VehicleCollisionTesterRay.create(allocator, TestLayers.moving, .{})).base);
    defer tester.deinit();
    constraint.setVehicleCollisionTester(tester.ptr);
    try world.system.addConstraint(constraint.asConstraint());
    try world.system.addStepListener(constraint.stepListener());
    return constraint;
}

fn trackedController(constraint: *VehicleConstraint) *TrackedVehicleController {
    return virtual.downcast(TrackedVehicleController, constraint.getControllerMut());
}

/// SaveBinaryState of `object` into `out`
fn testSave(object: anytype, out: *std.Io.Writer.Allocating) void {
    var wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    object.saveBinaryState(wrapper.streamOut());
}

/// RestoreBinaryState of `object` from `bytes`, the stream must not hit its end
fn testRestore(object: anytype, bytes: []const u8) !void {
    var reader: std.Io.Reader = .fixed(bytes);
    var wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
    try object.restoreBinaryState(wrapper.streamIn());
    try testing.expect(!wrapper.isEOF() and !wrapper.isFailed());
}

test "WheelSettingsTV defaults, IsKindOf and binary state" {
    const allocator = testing.allocator;

    const settings = try WheelSettingsTV.create(allocator);
    var ref = Ref(WheelSettings).init(settings.asWheelSettings());
    defer ref.deinit();
    try testing.expectEqual(@as(f32, 4.0), settings.longitudinal_friction);
    try testing.expectEqual(@as(f32, 2.0), settings.lateral_friction);
    try testing.expectEqual(@as(f32, 0.3), settings.base.radius);
    try testing.expect(settings.base.isKindOf(WheelSettings) and settings.base.isKindOf(WheelSettingsTV));
    try testing.expect(!settings.base.isKindOf(WheelSettingsWV));
    try testing.expect(settings.base.cast(WheelSettingsTV) == settings);
    try testing.expect(settings.asWheelSettingsConst() == &settings.base);

    // The binary state: WheelSettings followed by the two frictions (through the base class)
    settings.longitudinal_friction = 3.5;
    settings.lateral_friction = 1.5;
    settings.base.width = 0.4;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    testSave(&settings.base, &out);
    try testing.expectEqual(@as(usize, 6 * 12 + 3 * 4 + 9 + 2 * 4 + 1 + 2 * 4), out.written().len);
    try testing.expectEqualSlices(u8, std.mem.asBytes(&[2]f32{ 3.5, 1.5 }), out.written()[out.written().len - 8 ..]);

    const restored = try WheelSettingsTV.create(allocator);
    var restored_ref = Ref(WheelSettings).init(&restored.base);
    defer restored_ref.deinit();
    try testRestore(&restored.base, out.written());
    try testing.expectEqual(@as(f32, 3.5), restored.longitudinal_friction);
    try testing.expectEqual(@as(f32, 1.5), restored.lateral_friction);
    try testing.expectEqual(@as(f32, 0.4), restored.base.width);

    // An embedded WheelSettingsTV
    var embedded = WheelSettingsTV.init(allocator);
    embedded.base.setEmbedded();
    defer embedded.base.deinit();
    try testing.expect(embedded.base.isKindOf(WheelSettingsTV));

    // Out of memory: creation and restore leave nothing behind
    const S = struct {
        fn run(a: Allocator, bytes: []const u8) !void {
            const s = try WheelSettingsTV.create(a);
            var s_ref = Ref(WheelSettings).init(&s.base);
            defer s_ref.deinit();
            try testRestore(&s.base, bytes);
        }
    };
    try testing.checkAllAllocationFailures(allocator, S.run, .{out.written()});
}

test "TrackedVehicleControllerSettings defaults, RTTI hash and binary state" {
    const allocator = testing.allocator;

    const settings = try TrackedVehicleControllerSettings.create(allocator);
    var ref = Ref(VehicleControllerSettings).init(settings.asControllerSettings());
    defer ref.deinit();
    try testing.expectEqual(@as(f32, 500.0), settings.engine.min_rpm);
    try testing.expectEqual(@as(f32, 4000.0), settings.engine.max_rpm);
    try testing.expectEqual(@as(f32, 500.0), settings.engine.max_torque);
    try testing.expectEqual(@as(usize, 3), settings.engine.normalized_torque.points.items.len);
    try testing.expectEqual(@as(f32, 1000.0), settings.transmission.shift_down_rpm);
    try testing.expectEqual(@as(f32, 3500.0), settings.transmission.shift_up_rpm);
    try testing.expectEqualSlices(f32, &.{ 4.0, 3.0, 2.0, 1.0 }, settings.transmission.gear_ratios.items);
    try testing.expectEqualSlices(f32, &.{ -4.0, -3.0 }, settings.transmission.reverse_gear_ratios.items);
    for (settings.tracks) |t| {
        try testing.expectEqual(@as(u32, 0), t.driven_wheel);
        try testing.expectEqual(@as(usize, 0), t.wheels.items.len);
        try testing.expectEqual(@as(f32, 10.0), t.inertia);
    }
    try testing.expectEqual(RTTI.hashName("TrackedVehicleControllerSettings"), settings.base.getRTTIHash());
    try testing.expect(settings.asControllerSettingsConst() == &settings.base);

    // The binary state (through the base class): the engine, the transmission and the two tracks
    try settings.tracks[1].wheels.appendSlice(allocator, &.{ 4, 5, 6 });
    settings.tracks[1].driven_wheel = 6;
    settings.tracks[0].max_brake_torque = 9000.0;
    settings.transmission.mode = .manual;
    settings.engine.max_torque = 650.0;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    testSave(&settings.base, &out);
    try testing.expectEqual(@as(usize, (3 * 4 + 4 + 3 * 8) + (1 + (4 + 4 * 4) + (4 + 2 * 4) + 6 * 4) + (4 + 4 + 4 * 4) + (4 + 4 + 3 * 4 + 4 * 4)), out.written().len);

    const restored = try TrackedVehicleControllerSettings.create(allocator);
    var restored_ref = Ref(VehicleControllerSettings).init(&restored.base);
    defer restored_ref.deinit();
    try testRestore(&restored.base, out.written());
    try testing.expectEqualSlices(u32, &.{ 4, 5, 6 }, restored.tracks[1].wheels.items);
    try testing.expectEqual(@as(u32, 6), restored.tracks[1].driven_wheel);
    try testing.expectEqual(@as(f32, 9000.0), restored.tracks[0].max_brake_torque);
    try testing.expectEqual(VehicleTransmissionFile.TransmissionMode.manual, restored.transmission.mode);
    try testing.expectEqual(@as(f32, 650.0), restored.engine.max_torque);
    var out2: std.Io.Writer.Allocating = .init(allocator);
    defer out2.deinit();
    testSave(&restored.base, &out2);
    try testing.expectEqualSlices(u8, out.written(), out2.written());

    // An embedded TrackedVehicleControllerSettings
    var embedded = try TrackedVehicleControllerSettings.init(allocator);
    embedded.base.setEmbedded();
    defer embedded.base.deinit();

    // Out of memory: creation and restore leave nothing behind
    const S = struct {
        fn run(a: Allocator, bytes: []const u8) !void {
            const s = try TrackedVehicleControllerSettings.create(a);
            var s_ref = Ref(VehicleControllerSettings).init(&s.base);
            defer s_ref.deinit();
            try testRestore(&s.base, bytes);
        }
    };
    try testing.checkAllAllocationFailures(allocator, S.run, .{out.written()});
}

test "TrackedVehicleControllerSettings restored by VehicleConstraintSettings from its RTTI hash" {
    const allocator = testing.allocator;

    // VehicleConstraintSettings without wheels (Jolt cannot restore wheels, see VehicleConstraint.zig) and a tracked
    // controller
    var settings = VehicleConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    const controller = try TrackedVehicleControllerSettings.create(allocator);
    settings.setController(&controller.base);
    try controller.tracks[0].wheels.appendSlice(allocator, &.{ 0, 1 });
    controller.tracks[0].differential_ratio = 5.0;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    testSave(settings.asSettingsConst(), &out);

    var reader: std.Io.Reader = .fixed(out.written());
    var in: StreamWrapper.StreamInWrapper = .init(&reader);
    var result = try ConstraintSettings.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    try testing.expect(!result.hasError());
    const restored = virtual.downcast(VehicleConstraintSettings, result.get().get().?);
    const restored_controller = restored.controller.ptr.?;
    try testing.expectEqualStrings("TrackedVehicleControllerSettings", restored_controller.vtable.rtti.name);
    const tracked = virtual.downcast(TrackedVehicleControllerSettings, restored_controller);
    try testing.expectEqualSlices(u32, &.{ 0, 1 }, tracked.tracks[0].wheels.items);
    try testing.expectEqual(@as(f32, 5.0), tracked.tracks[0].differential_ratio);
}

test "TrackedVehicleController syncLeftRightTracks" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    const constraint = try addTank(&world, allocator, RVec3.init(0, 0.95, 0));
    const c = trackedController(constraint);
    const tl = &c.tracks[0];
    const tr = &c.tracks[1];
    tl.base.inertia = 8.0;
    tr.base.inertia = 12.0;

    // Same direction: the ratio of the angular velocities becomes the ratio of the inputs, dl * Ir = -dr * Il
    c.setDriverInput(1.0, 1.0, 0.5, 0.0);
    tl.angular_velocity = 3.0;
    tr.angular_velocity = 4.0;
    c.syncLeftRightTracks();
    try testing.expectApproxEqAbs(@as(f32, 2.0), tl.angular_velocity / tr.angular_velocity, 1.0e-5);
    try testing.expectApproxEqAbs((tl.angular_velocity - 3.0) * 12.0, -(tr.angular_velocity - 4.0) * 8.0, 1.0e-3);

    // Opposite directions (turning on the spot): dl * Ir = dr * Il
    c.setDriverInput(1.0, -1.0, 1.0, 0.0);
    tl.angular_velocity = 3.0;
    tr.angular_velocity = 4.0;
    c.syncLeftRightTracks();
    try testing.expectApproxEqAbs(@as(f32, -1.0), tl.angular_velocity / tr.angular_velocity, 1.0e-5);
    try testing.expectApproxEqAbs((tl.angular_velocity - 3.0) * 12.0, (tr.angular_velocity - 4.0) * 8.0, 1.0e-3);
}

test "TrackedVehicleController drives, steers with the track ratios, turns on the spot, brakes and sleeps" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    const constraint = try addTank(&world, allocator, RVec3.init(0, 0.95, 0));
    const c = trackedController(constraint);
    const body = constraint.getVehicleBody();

    // The defaults of the controller
    try testing.expectEqual(@as(f32, 0.0), c.getForwardInput());
    try testing.expectEqual(@as(f32, 1.0), c.getLeftRatio());
    try testing.expectEqual(@as(f32, 1.0), c.getRightRatio());
    try testing.expectEqual(@as(f32, 0.0), c.getBrakeInput());
    try testing.expectEqual(@as(f32, 500.0), c.getEngine().getCurrentRPM()); // SetCurrentRPM(mMinRPM)
    try testing.expectEqual(@as(i32, 0), c.getTransmission().getCurrentGear());
    try testing.expect(c.asControllerConst() == constraint.getController());
    try testing.expect(c.allowSleep());
    for (constraint.getWheels()) |w|
        try testing.expectEqual(@as(i32, -1), virtual.downcast(WheelTV, w).track_index);

    // Landing: PreCollide fills in the track indices, every wheel touches the floor
    try world.simulate(1.0);
    for (constraint.getWheels(), 0..) |w, i| {
        const tv = virtual.downcast(WheelTV, w);
        try testing.expectEqual(@as(i32, if (i < 4) 0 else 1), tv.track_index);
        try testing.expect(w.hasContact());
        try testing.expect(tv.getSettings() == w.getSettings().cast(WheelSettingsTV));

        // Default combine function: sqrt(track friction * floor friction (0.2))
        try testing.expectEqual(@sqrt(tv.getSettings().longitudinal_friction * 0.2), tv.combined_longitudinal_friction);
        try testing.expectEqual(@sqrt(2.0 * @as(f32, 0.2)), tv.combined_lateral_friction);
    }
    const start = body.getPosition();
    try testing.expect(@abs(start.getZ()) < 0.01);

    // Drive forward: the tracks turn at the same rate, the engine connects, the transmission shifts up
    c.setDriverInput(1.0, 1.0, 1.0, 0.0);
    try testing.expect(!c.allowSleep());
    world.system.getBodyInterfaceMut().activateBody(body.getID());
    try world.simulate(3.0);
    const p1 = body.getPosition();
    try testing.expect(p1.getZ() > start.getZ() + 2.0);
    try testing.expect(@abs(p1.getX()) < 0.05);
    try testing.expect(c.getTransmission().getCurrentGear() > 0);
    try testing.expect(c.getEngine().getCurrentRPM() > 500.0);
    try testing.expect(c.getTracks()[0].angular_velocity > 0.0);
    try testing.expectApproxEqRel(c.getTracks()[0].angular_velocity, c.getTracks()[1].angular_velocity, 1.0e-4);

    // WheelTV::CalculateAngularVelocity: the angular velocity of the driven wheel scaled by the radii, Update: the angle
    for (constraint.getWheels(), 0..) |w, i| {
        const track = c.getTracks()[if (i < 4) 0 else 1];
        const expected = track.angular_velocity * constraint.getWheel(track.base.driven_wheel).getSettings().radius / w.getSettings().radius;
        try testing.expectEqual(expected, w.getAngularVelocity());
        try testing.expect(w.getRotationAngle() > -2.0 * math.pi and w.getRotationAngle() < 2.0 * math.pi);
    }
    try testing.expect(constraint.getWheel(0).getAngularVelocity() > constraint.getWheel(1).getAngularVelocity()); // The small wheel turns faster

    // Steer right: the left track turns faster than the right track, the tank yaws to the right (around -y)
    c.setRightRatio(0.4);
    try world.simulate(1.0);
    try testing.expect(body.getAngularVelocity().getY() < -0.1);
    try testing.expectApproxEqRel(@as(f32, 1.0 / 0.4), c.getTracks()[0].angular_velocity / c.getTracks()[1].angular_velocity, 1.0e-3);

    // Turn on the spot to the left: the tracks turn in opposite directions
    c.setDriverInput(1.0, -1.0, 1.0, 0.0);
    try world.simulate(2.0);
    try testing.expect(body.getAngularVelocity().getY() > 0.1);
    try testing.expect(c.getTracks()[0].angular_velocity < 0.0 and c.getTracks()[1].angular_velocity > 0.0);

    // Brake: the tracks lock, the brakes act on the wheels that touch the floor
    c.setForwardInput(0.0);
    c.setLeftRatio(1.0);
    c.setRightRatio(1.0);
    c.setBrakeInput(1.0);
    var braked = false;
    for (0..60) |_| {
        try world.simulate(1.0 / 60.0);
        for (constraint.getWheels()) |w| {
            if (virtual.downcast(WheelTV, w).brake_impulse > 0.0) braked = true;
        }
    }
    try testing.expect(braked);
    try testing.expectEqual(@as(f32, 0.0), c.getTracks()[0].angular_velocity);
    try testing.expectEqual(@as(f32, 0.0), c.getTracks()[1].angular_velocity);
    try testing.expect(body.getLinearVelocity().length() < 0.1);

    // Without input the engine idles and the vehicle falls asleep
    c.setBrakeInput(0.0);
    try world.simulate(5.0);
    try testing.expect(c.allowSleep());
    try testing.expect(!body.isActive());
}

test "TrackedVehicleController getters, setters, settings and state" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    const constraint = try addTank(&world, allocator, RVec3.init(0, 0.95, 0));
    const c = trackedController(constraint);

    c.setForwardInput(0.5);
    c.setLeftRatio(0.75);
    c.setRightRatio(-0.25);
    c.setBrakeInput(0.125);
    try testing.expectEqual(@as(f32, 0.5), c.getForwardInput());
    try testing.expectEqual(@as(f32, 0.75), c.getLeftRatio());
    try testing.expectEqual(@as(f32, -0.25), c.getRightRatio());
    try testing.expectEqual(@as(f32, 0.125), c.getBrakeInput());
    c.getEngineMut().base.max_torque = 333.0;
    c.getTransmissionMut().base.clutch_strength = 7.0;
    c.getTracksMut()[1].base.inertia = 11.0;
    c.getTracksMut()[0].angular_velocity = 2.5;
    c.getEngineMut().setCurrentRPM(1234.0);
    c.getTransmissionMut().set(2, 0.5);

    // GetSettings gives the settings of the engine, transmission and tracks
    var settings = try constraint.getController().getSettings();
    defer settings.deinit();
    const tracked = virtual.downcast(TrackedVehicleControllerSettings, settings.get().?);
    try testing.expectEqual(@as(f32, 333.0), tracked.engine.max_torque);
    try testing.expectEqual(@as(f32, 7.0), tracked.transmission.clutch_strength);
    try testing.expectEqual(@as(f32, 11.0), tracked.tracks[1].inertia);
    try testing.expectEqualSlices(u32, &.{ 4, 5, 6, 7 }, tracked.tracks[1].wheels.items);
    try testing.expectEqual(@as(u32, 3), tracked.tracks[0].driven_wheel);

    // GetConstraintSettings recreates the controller settings through GetSettings
    var copy = try constraint.asConstraint().getConstraintSettings();
    defer copy.deinit();
    const copy_controller = virtual.downcast(VehicleConstraintSettings, copy.get().?).controller.ptr.?;
    try testing.expectEqualStrings("TrackedVehicleControllerSettings", copy_controller.vtable.rtti.name);
    var settings_bytes: std.Io.Writer.Allocating = .init(allocator);
    defer settings_bytes.deinit();
    testSave(settings.get().?, &settings_bytes);
    var copy_bytes: std.Io.Writer.Allocating = .init(allocator);
    defer copy_bytes.deinit();
    testSave(copy_controller, &copy_bytes);
    try testing.expectEqualSlices(u8, settings_bytes.written(), copy_bytes.written());

    // SaveState / RestoreState: the inputs, the engine, the transmission and the angular velocities of the tracks
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    constraint.getController().saveState(recorder.recorder());
    try testing.expectEqual(@as(usize, 4 * 4 + 4 + 20 + 2 * 4), recorder.getData().len);
    c.setDriverInput(0.0, 1.0, 1.0, 0.0);
    c.getEngineMut().setCurrentRPM(700.0);
    c.getTransmissionMut().set(0, 1.0);
    c.getTracksMut()[0].angular_velocity = 0.0;
    recorder.rewind();
    constraint.getControllerMut().restoreState(recorder.recorder());
    try testing.expectEqual(@as(f32, 0.5), c.getForwardInput());
    try testing.expectEqual(@as(f32, -0.25), c.getRightRatio());
    try testing.expectEqual(@as(f32, 1234.0), c.getEngine().getCurrentRPM());
    try testing.expectEqual(@as(i32, 2), c.getTransmission().getCurrentGear());
    try testing.expectEqual(@as(f32, 2.5), c.getTracks()[0].angular_velocity);
}

test "TrackedVehicleController creation, settings and controller settings out of memory" {
    var world: TestWorld = undefined;
    try world.init(testing.allocator);
    defer world.deinit();
    const body = try addTankBody(&world, testing.allocator, RVec3.init(0, 0.95, 0));

    const S = struct {
        fn run(allocator: Allocator, tank_body: *Body) !void {
            var settings = try makeTankSettings(allocator);
            defer settings.asSettings().deinit();

            const constraint = try VehicleConstraint.create(allocator, tank_body, &settings);
            var constraint_ref = Ref(Constraint).init(constraint.asConstraint());
            defer constraint_ref.deinit();

            var copy = try constraint.asConstraint().getConstraintSettings();
            defer copy.deinit();
            var controller_settings = try constraint.getController().getSettings();
            defer controller_settings.deinit();
        }
    };
    try testing.checkAllAllocationFailures(testing.allocator, S.run, .{body});
}
