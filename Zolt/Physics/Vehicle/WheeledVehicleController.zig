//! Port of: Jolt/Physics/Vehicle/WheeledVehicleController.h, Jolt/Physics/Vehicle/WheeledVehicleController.cpp
//! Status: complete
//! Not ported: JPH_TRACE_VEHICLE_STATS (a developer switch that is commented out in Jolt)
//!
//! - JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(WheeledVehicleControllerSettings,
//!   WheelSettingsWV): TODO(serialization) (ObjectStream is Phase 8, CharacterVehicleArchitecture E1; the binary state is
//!   ported). JPH_DEBUG_RENDERER (`Draw`, `SetRPMMeter`, `mRPMMeterPosition`, `mRPMMeterSize`): TODO(debug renderer).
//! - CharacterVehicleArchitecture E2:
//!   - `WheelSettingsWV` embeds `base: WheelSettings` and overrides its binary state; `WheelWV` embeds `base: Wheel`.
//!   - `WheeledVehicleControllerSettings` embeds `base: VehicleControllerSettings` (`rtti_name`, the settings class of
//!     `RegisterTypes.vehicle_controller_settings_types`, E3); `WheeledVehicleController` embeds `base:
//!     VehicleController`. Both are concrete classes that MotorcycleController(Settings) derive from: `initDerived(T,
//!     ...)` builds them as the base of class T, their overrides are top-level functions (Jolt's
//!     `WheeledVehicleController::PreCollide(...)` from a derived class is `self.base.preCollide(...)`).
//!   - `ConstructWheel`'s `JPH_ASSERT(IsKindOf(&inWheel, JPH_RTTI(WheelSettingsWV)))` + static_cast is the checked
//!     `wheel.cast(WheelSettingsWV)`.
//! - The engine, transmission and settings arrays use the allocator of the class that owns them (the settings' root,
//!   the controller's root); arrays assigned through the writable getters must be allocated with it.
//! - Allocation (E2): `create`, `getSettings` and `constructController` / `constructWheel` return `Allocator.Error` and
//!   leave nothing behind. PostCollide and SolveLongitudinalAndLateralConstraints run inside the physics update (step
//!   listener and solver jobs, no error channel), so they never allocate:
//!   - PostCollide's `Array<DrivenDifferential>`, `Array<DrivenWheel>` and the two DynMatrix of the implicit Euler step
//!     use scratch buffers that the constructor allocates for the number of differentials (at most one driven
//!     differential per differential, at most two driven wheels per differential, a matrix of (2 * differentials + 1)^2
//!     plus a vector). Deviation: `getDifferentialsMut()` therefore gives the differentials as a slice (their values can
//!     be changed, their number is fixed after creation; Jolt's writable GetDifferentials returns the array). The
//!     matrix dimension is bounded by GaussianElimination's `max_dynamic_size` (256, Jolt's JPH_STACK_ALLOC is
//!     unbounded): at most 127 differentials.
//!   - SolveLongitudinalAndLateralConstraints' `JPH_STACK_ALLOC(mWheels.size() * sizeof(float))` array of lateral
//!     impulses is the Zolt field `WheelWV.max_lateral_friction_impulse` (one value per wheel, written in the first loop
//!     and read in the second, like the array entries).
//! - `TireMaxImpulseCallback` (std::function) is a function pointer + context with `init` and `call`.
//! - Jolt behavior reproduced (CharacterVehicleArchitecture, "Settled in the foundation review"):
//!   `GetWheelSpeedAtClutch` divides by the number of driven wheels without a guard (0 / 0 without driven wheels).
//! - Jolt's asserts on the settings in the constructors and the sum of the torque ratios in PostCollide are user input
//!   checks, evaluated in safe builds only (`if (Core.enable_asserts)`). "New engine/wheel speeds could not be
//!   calculated!" (a failed Gaussian elimination) panics in safe builds and continues like Jolt's release build
//!   otherwise.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const LinearCurve = @import("../../Core/LinearCurve.zig").LinearCurve;
const Ref = @import("../../Core/Reference.zig").Ref;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const Trigonometry = @import("../../Math/Trigonometry.zig");
const DynMatrix = @import("../../Math/DynMatrix.zig").DynMatrix;
const gaussianElimination = @import("../../Math/GaussianElimination.zig").gaussianElimination;
const Body = @import("../Body/Body.zig").Body;
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
const VehicleDifferentialSettings = @import("VehicleDifferential.zig").VehicleDifferentialSettings;

/// WheelSettings object specifically for WheeledVehicleController
pub const WheelSettingsWV = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, WheelSettingsWV)

    /// JPH_RTTI(WheelSettingsWV)
    pub const rtti: *const WheelSettings.RTTI = &.{ .name = rtti_name, .base = WheelSettings.rtti };
    /// Class name (JPH_RTTI)
    pub const rtti_name = "WheelSettingsWV";
    pub const overrides = .{ .saveBinaryState, .restoreBinaryState };

    base: WheelSettings,

    /// Moment of inertia (kg m^2), for a cylinder this would be 0.5 * M * R^2 which is 0.9 for a wheel with a mass of 20 kg and radius 0.3 m
    inertia: f32 = 0.9,
    /// Angular damping factor of the wheel: dw/dt = -c * w. Value should be zero or positive and is usually close to 0.
    angular_damping: f32 = 0.2,
    /// How much this wheel can steer (radians)
    max_steer_angle: f32 = math.degreesToRadians(70.0),
    /// On the Y-axis: friction in the forward direction of the tire. Friction is normally between 0 (no friction) and 1 (full friction) although friction can be a little bit higher than 1 because of the profile of a tire. On the X-axis: the slip ratio (fraction) defined as (omega_wheel * r_wheel - v_longitudinal) / |v_longitudinal|. You can see slip ratio as the amount the wheel is spinning relative to the floor: 0 means the wheel has full traction and is rolling perfectly in sync with the ground, 1 is for example when the wheel is locked and sliding over the ground.
    longitudinal_friction: LinearCurve = .{},
    /// On the Y-axis: friction in the sideways direction of the tire. Friction is normally between 0 (no friction) and 1 (full friction) although friction can be a little bit higher than 1 because of the profile of a tire. On the X-axis: the slip angle (degrees) defined as angle between relative contact velocity and tire direction.
    lateral_friction: LinearCurve = .{},
    /// How much torque (Nm) the brakes can apply to this wheel
    max_brake_torque: f32 = 1500.0,
    /// How much torque (Nm) the hand brake can apply to this wheel (usually only applied to the rear wheels)
    max_hand_brake_torque: f32 = 4000.0,

    /// Constructor of the base part of a class T derived from WheelSettingsWV (the friction curves are allocated with
    /// `allocator`, the destructor frees them)
    pub fn initDerived(comptime T: type, allocator: Allocator) Allocator.Error!WheelSettingsWV {
        var self: WheelSettingsWV = .{ .base = .init(T, allocator) };
        errdefer self.destruct();

        try self.longitudinal_friction.reserve(allocator, 3);
        try self.longitudinal_friction.addPoint(allocator, 0.0, 0.0);
        try self.longitudinal_friction.addPoint(allocator, 0.06, 1.2);
        try self.longitudinal_friction.addPoint(allocator, 0.2, 1.0);

        try self.lateral_friction.reserve(allocator, 3);
        try self.lateral_friction.addPoint(allocator, 0.0, 0.0);
        try self.lateral_friction.addPoint(allocator, 3.0, 1.2);
        try self.lateral_friction.addPoint(allocator, 20.0, 1.0);
        return self;
    }

    /// Constructor of settings that are not on the heap (the owner calls `base.deinit()`)
    pub fn init(allocator: Allocator) Allocator.Error!WheelSettingsWV {
        return initDerived(WheelSettingsWV, allocator);
    }

    /// new WheelSettingsWV (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*WheelSettingsWV {
        const self = try allocator.create(WheelSettingsWV);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator);
        return self;
    }

    /// ~WheelSettingsWV: frees the friction curves
    pub fn destruct(self: *WheelSettingsWV) void {
        self.longitudinal_friction.deinit(self.base.allocator);
        self.lateral_friction.deinit(self.base.allocator);
    }

    /// Upcasts (implicit in C++)
    pub fn asWheelSettings(self: *WheelSettingsWV) *WheelSettings {
        return &self.base;
    }

    pub fn asWheelSettingsConst(self: *const WheelSettingsWV) *const WheelSettings {
        return &self.base;
    }

    // See: WheelSettings
    pub fn saveBinaryState(self: *const WheelSettingsWV, stream: StreamOut) void {
        WheelSettings.impl.saveBinaryState(&self.base, stream);

        stream.write(self.inertia);
        stream.write(self.angular_damping);
        stream.write(self.max_steer_angle);
        self.longitudinal_friction.saveBinaryState(stream);
        self.lateral_friction.saveBinaryState(stream);
        stream.write(self.max_brake_torque);
        stream.write(self.max_hand_brake_torque);
    }

    pub fn restoreBinaryState(self: *WheelSettingsWV, stream: StreamIn) Allocator.Error!void {
        try WheelSettings.impl.restoreBinaryState(&self.base, stream);

        stream.read(&self.inertia);
        stream.read(&self.angular_damping);
        stream.read(&self.max_steer_angle);
        try self.longitudinal_friction.restoreBinaryState(self.base.allocator, stream);
        try self.lateral_friction.restoreBinaryState(self.base.allocator, stream);
        stream.read(&self.max_brake_torque);
        stream.read(&self.max_hand_brake_torque);
    }
};

/// Wheel object specifically for WheeledVehicleController
pub const WheelWV = struct {
    pub const overrides = .{};

    base: Wheel,

    /// Velocity difference between ground and wheel relative to ground velocity
    longitudinal_slip: f32 = 0.0,
    /// Angular difference (in radians) between ground and wheel relative to ground velocity
    lateral_slip: f32 = 0.0,
    /// Combined friction coefficient in longitudinal direction (combines terrain and tires)
    combined_longitudinal_friction: f32 = 0.0,
    /// Combined friction coefficient in lateral direction (combines terrain and tires)
    combined_lateral_friction: f32 = 0.0,
    /// Amount of impulse that the brakes can apply to the floor (excluding friction)
    brake_impulse: f32 = 0.0,
    /// Zolt: the entry of this wheel in SolveLongitudinalAndLateralConstraints' JPH_STACK_ALLOC array
    /// `max_lateral_friction_impulse` (see the file comment)
    max_lateral_friction_impulse: f32 = 0.0,

    /// Constructor of the base part of a class T derived from WheelWV
    pub fn initDerived(comptime T: type, allocator: Allocator, settings: *const WheelSettingsWV) WheelWV {
        if (Core.enable_asserts) {
            std.debug.assert(settings.inertia >= 0.0);
            std.debug.assert(settings.angular_damping >= 0.0);
            std.debug.assert(@abs(settings.max_steer_angle) <= 0.5 * math.pi);
            std.debug.assert(settings.max_brake_torque >= 0.0);
            std.debug.assert(settings.max_hand_brake_torque >= 0.0);
        }
        return .{ .base = .init(T, allocator, &settings.base) };
    }

    /// Constructor
    pub fn init(allocator: Allocator, settings: *const WheelSettingsWV) WheelWV {
        return initDerived(WheelWV, allocator, settings);
    }

    /// new WheelWV(inSettings) (VehicleConstraint destroys it)
    pub fn create(allocator: Allocator, settings: *const WheelSettingsWV) Allocator.Error!*WheelWV {
        const self = try allocator.create(WheelWV);
        self.* = .init(allocator, settings);
        return self;
    }

    /// Override GetSettings and cast to the correct class
    pub fn getSettings(self: *const WheelWV) *const WheelSettingsWV {
        return virtual.downcast(WheelSettingsWV, self.base.getSettings());
    }

    /// Apply a torque (N m) to the wheel for a particular delta time
    pub fn applyTorque(self: *WheelWV, torque: f32, delta_time: f32) void {
        self.base.angular_velocity += torque * delta_time / self.getSettings().inertia;
    }

    /// Update the wheel rotation based on the current angular velocity
    pub fn update(self: *WheelWV, wheel_index: u32, delta_time: f32, constraint: *const VehicleConstraint) void {
        const settings = self.getSettings();

        // Angular damping: dw/dt = -c * w
        // Solution: w(t) = w(0) * e^(-c * t) or w2 = w1 * e^(-c * dt)
        // Taylor expansion of e^(-c * dt) = 1 - c * dt + ...
        // Since dt is usually in the order of 1/60 and c is a low number too this approximation is good enough
        self.base.angular_velocity *= math.max(@as(f32, 0.0), 1.0 - settings.angular_damping * delta_time);

        // Update rotation of wheel
        self.base.angle = @rem(self.base.angle + self.base.angular_velocity * delta_time, 2.0 * math.pi); // fmod

        if (self.base.contact_body) |contact_body| {
            const body = constraint.getVehicleBody();

            // Calculate relative velocity between wheel contact point and floor
            var relative_velocity = body.getPointVelocity(self.base.contact_position).sub(self.base.contact_point_velocity);

            // Cancel relative velocity in the normal plane
            relative_velocity = relative_velocity.sub(self.base.contact_normal.mulScalar(self.base.contact_normal.dot(relative_velocity)));
            const relative_longitudinal_velocity = relative_velocity.dot(self.base.contact_longitudinal);

            // Calculate longitudinal friction based on difference between velocity of rolling wheel and drive surface
            const relative_longitudinal_velocity_denom = math.sign(relative_longitudinal_velocity) * math.max(@as(f32, 1.0e-3), @abs(relative_longitudinal_velocity)); // Ensure we don't divide by zero
            self.longitudinal_slip = @abs((self.base.angular_velocity * settings.base.radius - relative_longitudinal_velocity) / relative_longitudinal_velocity_denom);
            const longitudinal_slip_friction = settings.longitudinal_friction.getValue(self.longitudinal_slip);

            // Calculate lateral friction based on slip angle
            const relative_velocity_len = relative_velocity.length();
            self.lateral_slip = if (relative_velocity_len < 1.0e-3) 0.0 else Trigonometry.acos(@abs(relative_longitudinal_velocity) / relative_velocity_len);
            const lateral_slip_angle = math.radiansToDegrees(self.lateral_slip);
            const lateral_slip_friction = settings.lateral_friction.getValue(lateral_slip_angle);

            // Tire friction
            const combine_friction = constraint.getCombineFriction();
            self.combined_longitudinal_friction = longitudinal_slip_friction;
            self.combined_lateral_friction = lateral_slip_friction;
            combine_friction.call(wheel_index, &self.combined_longitudinal_friction, &self.combined_lateral_friction, contact_body, self.base.contact_sub_shape_id);
        } else {
            // No collision
            self.longitudinal_slip = 0.0;
            self.lateral_slip = 0.0;
            self.combined_lateral_friction = 0.0;
            self.combined_longitudinal_friction = 0.0;
        }
    }
};

/// Settings of a vehicle with regular wheels
///
/// The properties in this controller are largely based on "Car Physics for Games" by Marco Monster.
/// See: https://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html
pub const WheeledVehicleControllerSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, WheeledVehicleControllerSettings)

    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "WheeledVehicleControllerSettings";
    pub const overrides = .{ .constructController, .saveBinaryState, .restoreBinaryState };

    base: VehicleControllerSettings,

    /// The properties of the engine
    engine: VehicleEngineSettings = .{},
    /// The properties of the transmission (aka gear box)
    transmission: VehicleTransmissionSettings = .{},
    /// List of differentials and their properties
    differentials: std.ArrayList(VehicleDifferentialSettings) = .empty,
    /// Ratio max / min average wheel speed of each differential (measured at the clutch). When the ratio is exceeded all torque gets distributed to the differential with the minimal average velocity. This allows implementing a limited slip differential between differentials. Set to FLT_MAX for an open differential. Value should be > 1.
    differential_limited_slip_ratio: f32 = 1.4,

    /// Constructor of the base part of a class T derived from WheeledVehicleControllerSettings (the default engine curve
    /// and gear ratios are allocated with `allocator`, the destructor frees them)
    pub fn initDerived(comptime T: type, allocator: Allocator) Allocator.Error!WheeledVehicleControllerSettings {
        var self: WheeledVehicleControllerSettings = .{ .base = .init(T, allocator) };
        errdefer self.destruct();
        self.engine = try .init(allocator);
        self.transmission = try .init(allocator);
        return self;
    }

    /// Constructor of settings that are not on the heap (the owner calls `base.deinit()`)
    pub fn init(allocator: Allocator) Allocator.Error!WheeledVehicleControllerSettings {
        return initDerived(WheeledVehicleControllerSettings, allocator);
    }

    /// new WheeledVehicleControllerSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*WheeledVehicleControllerSettings {
        const self = try allocator.create(WheeledVehicleControllerSettings);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for VehicleConstraintSettings.restoreBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// ~WheeledVehicleControllerSettings: frees the engine curve, the gear ratios and the differentials
    pub fn destruct(self: *WheeledVehicleControllerSettings) void {
        const allocator = self.base.allocator;
        self.engine.deinit(allocator);
        self.transmission.deinit(allocator);
        self.differentials.deinit(allocator);
    }

    /// Upcasts (implicit in C++)
    pub fn asControllerSettings(self: *WheeledVehicleControllerSettings) *VehicleControllerSettings {
        return &self.base;
    }

    pub fn asControllerSettingsConst(self: *const WheeledVehicleControllerSettings) *const VehicleControllerSettings {
        return &self.base;
    }

    // See: VehicleControllerSettings
    pub fn constructController(self: *const WheeledVehicleControllerSettings, allocator: Allocator, constraint: *VehicleConstraint) Allocator.Error!*VehicleController {
        const controller = try WheeledVehicleController.create(allocator, self, constraint);
        return &controller.base;
    }

    pub fn saveBinaryState(self: *const WheeledVehicleControllerSettings, stream: StreamOut) void {
        self.engine.saveBinaryState(stream);

        self.transmission.saveBinaryState(stream);

        const num_differentials: u32 = @intCast(self.differentials.items.len);
        stream.write(num_differentials);
        for (self.differentials.items) |*d|
            d.saveBinaryState(stream);

        stream.write(self.differential_limited_slip_ratio);
    }

    pub fn restoreBinaryState(self: *WheeledVehicleControllerSettings, stream: StreamIn) Allocator.Error!void {
        const allocator = self.base.allocator;

        try self.engine.restoreBinaryState(allocator, stream);

        try self.transmission.restoreBinaryState(allocator, stream);

        var num_differentials: u32 = 0;
        stream.read(&num_differentials);
        const old_num_differentials = self.differentials.items.len;
        try self.differentials.resize(allocator, num_differentials);
        for (self.differentials.items[@min(old_num_differentials, num_differentials)..]) |*d|
            d.* = .{}; // Array::resize default constructs the new elements
        for (self.differentials.items) |*d|
            d.restoreBinaryState(stream);

        stream.read(&self.differential_limited_slip_ratio);
    }
};

/// Runtime controller class
pub const WheeledVehicleController = struct {
    pub const overrides = .{ .getSettings, .constructWheel, .allowSleep, .preCollide, .postCollide, .solveLongitudinalAndLateralConstraints, .saveState, .restoreState };

    /// Typedefs
    pub const Differentials = std.ArrayList(VehicleDifferentialSettings);

    /// Calculate max tire impulses by combining friction, slip, and suspension impulse. Note that the actual applied impulse may be lower (e.g. when the vehicle is stationary on a horizontal surface the actual impulse applied will be 0).
    /// (std::function: a function pointer + context)
    pub const TireMaxImpulseCallback = struct {
        /// The context that is passed to `function`
        context: ?*anyopaque = null,
        /// The function
        function: *const fn (context: ?*anyopaque, wheel_index: u32, longitudinal_impulse: *f32, lateral_impulse: *f32, suspension_impulse: f32, longitudinal_friction: f32, lateral_friction: f32, longitudinal_slip: f32, lateral_slip: f32, delta_time: f32) void,

        /// Jolt's default: the frictions times the suspension impulse
        pub const default: TireMaxImpulseCallback = .{ .function = defaultTireMaxImpulse };

        fn defaultTireMaxImpulse(context: ?*anyopaque, wheel_index: u32, longitudinal_impulse: *f32, lateral_impulse: *f32, suspension_impulse: f32, longitudinal_friction: f32, lateral_friction: f32, longitudinal_slip: f32, lateral_slip: f32, delta_time: f32) void {
            _ = .{ context, wheel_index, longitudinal_slip, lateral_slip, delta_time };
            longitudinal_impulse.* = longitudinal_friction * suspension_impulse;
            lateral_impulse.* = lateral_friction * suspension_impulse;
        }

        /// `function(context, wheel_index, longitudinal_impulse, lateral_impulse, suspension_impulse,
        /// longitudinal_friction, lateral_friction, longitudinal_slip, lateral_slip, delta_time)`, `context` is a
        /// pointer (it may be const)
        pub fn init(context: anytype, comptime function: fn (@TypeOf(context), u32, *f32, *f32, f32, f32, f32, f32, f32, f32) void) TireMaxImpulseCallback {
            const Context = @TypeOf(context);
            const gen = struct {
                fn call(ptr: ?*anyopaque, wheel_index: u32, longitudinal_impulse: *f32, lateral_impulse: *f32, suspension_impulse: f32, longitudinal_friction: f32, lateral_friction: f32, longitudinal_slip: f32, lateral_slip: f32, delta_time: f32) void {
                    const typed: Context = @ptrCast(@alignCast(ptr.?));
                    function(typed, wheel_index, longitudinal_impulse, lateral_impulse, suspension_impulse, longitudinal_friction, lateral_friction, longitudinal_slip, lateral_slip, delta_time);
                }
            };
            return .{ .context = @ptrCast(@constCast(context)), .function = gen.call };
        }

        /// Call the function
        pub fn call(self: TireMaxImpulseCallback, wheel_index: u32, longitudinal_impulse: *f32, lateral_impulse: *f32, suspension_impulse: f32, longitudinal_friction: f32, lateral_friction: f32, longitudinal_slip: f32, lateral_slip: f32, delta_time: f32) void {
            self.function(self.context, wheel_index, longitudinal_impulse, lateral_impulse, suspension_impulse, longitudinal_friction, lateral_friction, longitudinal_slip, lateral_slip, delta_time);
        }
    };

    /// Information about driven differentials (i.e. that have wheels connected), PostCollide's DrivenDifferential
    const DrivenDifferential = struct {
        differential: *const VehicleDifferentialSettings,
        angular_velocity: f32,
        clutch_to_differential_torque_ratio: f32,
        temp_torque_factor: f32,
    };

    /// Information about the wheels that connect to the engine, PostCollide's DrivenWheel
    const DrivenWheel = struct {
        wheel: *WheelWV,
        clutch_to_wheel_ratio: f32,
        clutch_to_wheel_torque_ratio: f32,
        estimated_angular_impulse: f32,
    };

    base: VehicleController,

    // Control information

    /// Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
    forward_input: f32 = 0.0,
    /// Value between -1 and 1 indicating desired steering angle
    right_input: f32 = 0.0,
    /// Value between 0 and 1 indicating how strong the brake pedal is pressed
    brake_input: f32 = 0.0,
    /// Value between 0 and 1 indicating how strong the hand brake is pulled
    hand_brake_input: f32 = 0.0,

    // Simulation information

    /// Engine state of the vehicle
    engine: VehicleEngine = .{},
    /// Transmission state of the vehicle
    transmission: VehicleTransmission = .{},
    /// Differential states of the vehicle
    differentials: Differentials = .empty,
    /// Ratio max / min average wheel speed of each differential (measured at the clutch).
    differential_limited_slip_ratio: f32 = undefined,
    /// Delta time of the last step
    previous_delta_time: f32 = 0.0,

    /// Callback that calculates the max impulse that the tire can apply to the ground
    tire_max_impulse_callback: TireMaxImpulseCallback = .default,

    /// Zolt: PostCollide's driven differentials, driven wheels and matrices, allocated for the number of differentials
    /// (see the file comment)
    driven_differentials_scratch: []DrivenDifferential = &.{},
    driven_wheels_scratch: []DrivenWheel = &.{},
    matrix_scratch: []f32 = &.{},

    /// Constructor of the base part of a class T derived from WheeledVehicleController (MotorcycleController). On
    /// failure nothing is left allocated.
    pub fn initDerived(comptime T: type, allocator: Allocator, settings: *const WheeledVehicleControllerSettings, constraint: *VehicleConstraint) Allocator.Error!WheeledVehicleController {
        var self: WheeledVehicleController = .{ .base = .init(T, allocator, constraint) };
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
            std.debug.assert(settings.transmission.clutch_strength > 0.0);
        }

        // Copy differential settings
        self.differentials = try settings.differentials.clone(allocator);
        if (Core.enable_asserts) {
            for (settings.differentials.items) |d| {
                std.debug.assert(d.differential_ratio > 0.0);
                std.debug.assert(d.left_right_split >= 0.0 and d.left_right_split <= 1.0);
                std.debug.assert(d.engine_torque_ratio >= 0.0);
                std.debug.assert(d.limited_slip_ratio > 1.0);
            }
        }

        self.differential_limited_slip_ratio = settings.differential_limited_slip_ratio;
        if (Core.enable_asserts) std.debug.assert(self.differential_limited_slip_ratio > 1.0);

        // Zolt: the scratch buffers of PostCollide
        const num_differentials = self.differentials.items.len;
        const max_n = 2 * num_differentials + 1;
        self.driven_differentials_scratch = try allocator.alloc(DrivenDifferential, num_differentials);
        self.driven_wheels_scratch = try allocator.alloc(DrivenWheel, 2 * num_differentials);
        self.matrix_scratch = try allocator.alloc(f32, max_n * max_n + max_n);

        return self;
    }

    /// Constructor
    pub fn init(allocator: Allocator, settings: *const WheeledVehicleControllerSettings, constraint: *VehicleConstraint) Allocator.Error!WheeledVehicleController {
        return initDerived(WheeledVehicleController, allocator, settings, constraint);
    }

    /// new WheeledVehicleController(inSettings, inConstraint) (VehicleConstraint destroys it)
    pub fn create(allocator: Allocator, settings: *const WheeledVehicleControllerSettings, constraint: *VehicleConstraint) Allocator.Error!*WheeledVehicleController {
        const self = try allocator.create(WheeledVehicleController);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator, settings, constraint);
        return self;
    }

    /// ~WheeledVehicleController
    pub fn destruct(self: *WheeledVehicleController) void {
        const allocator = self.base.allocator;
        self.engine.deinit(allocator);
        self.transmission.deinit(allocator);
        self.differentials.deinit(allocator);
        allocator.free(self.driven_differentials_scratch);
        allocator.free(self.driven_wheels_scratch);
        allocator.free(self.matrix_scratch);
    }

    /// Upcasts (implicit in C++)
    pub fn asController(self: *WheeledVehicleController) *VehicleController {
        return &self.base;
    }

    pub fn asControllerConst(self: *const WheeledVehicleController) *const VehicleController {
        return &self.base;
    }

    /// Set input from driver
    /// @param inForward Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
    /// @param inRight Value between -1 and 1 indicating desired steering angle (1 = right)
    /// @param inBrake Value between 0 and 1 indicating how strong the brake pedal is pressed
    /// @param inHandBrake Value between 0 and 1 indicating how strong the hand brake is pulled
    pub fn setDriverInput(self: *WheeledVehicleController, forward: f32, right: f32, brake: f32, hand_brake: f32) void {
        self.forward_input = forward;
        self.right_input = right;
        self.brake_input = brake;
        self.hand_brake_input = hand_brake;
    }

    /// Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
    pub fn setForwardInput(self: *WheeledVehicleController, forward: f32) void {
        self.forward_input = forward;
    }

    pub fn getForwardInput(self: *const WheeledVehicleController) f32 {
        return self.forward_input;
    }

    /// Value between -1 and 1 indicating desired steering angle (1 = right)
    pub fn setRightInput(self: *WheeledVehicleController, right: f32) void {
        self.right_input = right;
    }

    pub fn getRightInput(self: *const WheeledVehicleController) f32 {
        return self.right_input;
    }

    /// Value between 0 and 1 indicating how strong the brake pedal is pressed
    pub fn setBrakeInput(self: *WheeledVehicleController, brake: f32) void {
        self.brake_input = brake;
    }

    pub fn getBrakeInput(self: *const WheeledVehicleController) f32 {
        return self.brake_input;
    }

    /// Value between 0 and 1 indicating how strong the hand brake is pulled
    pub fn setHandBrakeInput(self: *WheeledVehicleController, hand_brake: f32) void {
        self.hand_brake_input = hand_brake;
    }

    pub fn getHandBrakeInput(self: *const WheeledVehicleController) f32 {
        return self.hand_brake_input;
    }

    /// Get current engine state
    pub fn getEngine(self: *const WheeledVehicleController) *const VehicleEngine {
        return &self.engine;
    }

    /// Get current engine state (writable interface, allows you to make changes to the configuration which will take effect the next time step)
    pub fn getEngineMut(self: *WheeledVehicleController) *VehicleEngine {
        return &self.engine;
    }

    /// Get current transmission state
    pub fn getTransmission(self: *const WheeledVehicleController) *const VehicleTransmission {
        return &self.transmission;
    }

    /// Get current transmission state (writable interface, allows you to make changes to the configuration which will take effect the next time step)
    pub fn getTransmissionMut(self: *WheeledVehicleController) *VehicleTransmission {
        return &self.transmission;
    }

    /// Get the differentials this vehicle has
    pub fn getDifferentials(self: *const WheeledVehicleController) []const VehicleDifferentialSettings {
        return self.differentials.items;
    }

    /// Get the differentials this vehicle has (writable interface, allows you to make changes to the configuration which will take effect the next time step)
    /// (Zolt: the differentials, not the array: their number is fixed, see the file comment)
    pub fn getDifferentialsMut(self: *WheeledVehicleController) []VehicleDifferentialSettings {
        return self.differentials.items;
    }

    /// Ratio max / min average wheel speed of each differential (measured at the clutch).
    pub fn getDifferentialLimitedSlipRatio(self: *const WheeledVehicleController) f32 {
        return self.differential_limited_slip_ratio;
    }

    pub fn setDifferentialLimitedSlipRatio(self: *WheeledVehicleController, v: f32) void {
        self.differential_limited_slip_ratio = v;
    }

    /// Get the average wheel speed of all driven wheels (measured at the clutch)
    pub fn getWheelSpeedAtClutch(self: *const WheeledVehicleController) f32 {
        var wheel_speed_at_clutch: f32 = 0.0;
        var num_driven_wheels: i32 = 0;
        for (self.differentials.items) |d| {
            const wheels = [_]i32{ d.left_wheel, d.right_wheel };
            for (wheels) |w| {
                if (w >= 0) {
                    wheel_speed_at_clutch += self.base.constraint.getWheel(@intCast(w)).getAngularVelocity() * d.differential_ratio;
                    num_driven_wheels += 1;
                }
            }
        }
        return wheel_speed_at_clutch / @as(f32, @floatFromInt(num_driven_wheels)) * VehicleEngine.angular_velocity_to_rpm * self.transmission.getCurrentRatio();
    }

    pub fn getTireMaxImpulseCallback(self: *const WheeledVehicleController) TireMaxImpulseCallback {
        return self.tire_max_impulse_callback;
    }

    pub fn setTireMaxImpulseCallback(self: *WheeledVehicleController, callback: TireMaxImpulseCallback) void {
        self.tire_max_impulse_callback = callback;
    }

    // TODO(debug renderer): SetRPMMeter

    // See: VehicleController
    pub fn getSettings(self: *const WheeledVehicleController) Allocator.Error!Ref(VehicleControllerSettings) {
        const settings = try WheeledVehicleControllerSettings.create(self.base.allocator);
        var result = Ref(VehicleControllerSettings).init(&settings.base);
        errdefer result.deinit();
        try self.toSettings(settings);
        return result;
    }

    /// Convert controller back to settings (protected in Jolt; the arrays are allocated with the settings' allocator)
    pub fn toSettings(self: *const WheeledVehicleController, out_settings: *WheeledVehicleControllerSettings) Allocator.Error!void {
        const allocator = out_settings.base.allocator;
        try out_settings.engine.assign(allocator, &self.engine.base);
        try out_settings.transmission.assign(allocator, &self.transmission.base);
        const differentials = try self.differentials.clone(allocator);
        out_settings.differentials.deinit(allocator);
        out_settings.differentials = differentials;
        out_settings.differential_limited_slip_ratio = self.differential_limited_slip_ratio;
    }

    pub fn constructWheel(self: *const WheeledVehicleController, allocator: Allocator, wheel: *const WheelSettings) Allocator.Error!*Wheel {
        _ = self;
        const w = try WheelWV.create(allocator, wheel.cast(WheelSettingsWV));
        return &w.base;
    }

    pub fn allowSleep(self: *const WheeledVehicleController) bool {
        return self.forward_input == 0.0 // No user input
        and self.transmission.allowSleep() // Transmission is not shifting
        and self.engine.allowSleep(); // Engine is idling
    }

    pub fn preCollide(self: *WheeledVehicleController, delta_time: f32, physics_system: *PhysicsSystem) void {
        _ = .{ delta_time, physics_system };

        for (self.base.constraint.getWheels()) |w_base| {
            const w = virtual.downcast(WheelWV, w_base);

            // Set steering angle
            w.base.setSteerAngle(-self.right_input * w.getSettings().max_steer_angle);
        }
    }

    pub fn postCollide(self: *WheeledVehicleController, delta_time: f32, physics_system: *PhysicsSystem) void {
        _ = physics_system;

        // Remember old RPM so we can detect if we're increasing or decreasing
        const old_engine_rpm = self.engine.getCurrentRPM();

        const wheels = self.base.constraint.getWheels();

        // Update wheel angle, do this before applying torque to the wheels (as friction will slow them down again)
        for (wheels, 0..) |w_base, wheel_index| {
            const w = virtual.downcast(WheelWV, w_base);
            w.update(@intCast(wheel_index), delta_time, self.base.constraint);
        }

        // In auto transmission mode, don't accelerate the engine when switching gears
        var forward_input = @abs(self.forward_input);
        if (self.transmission.base.mode == .auto)
            forward_input *= self.transmission.getClutchFriction();

        // Apply engine damping
        self.engine.applyDamping(delta_time);

        // Calculate engine torque
        const engine_torque = self.engine.getTorque(forward_input);

        // Collect driven differentials and their speeds (Array<DrivenDifferential> in Jolt: the scratch buffer)
        var num_driven_differentials: usize = 0;
        const driven_differentials = self.driven_differentials_scratch;
        var differential_omega_min: f32 = math.flt_max;
        var differential_omega_max: f32 = 0.0;
        for (self.differentials.items) |*d| {
            var avg_omega: f32 = 0.0;
            var avg_omega_denom: i32 = 0;
            const indices = [_]i32{ d.left_wheel, d.right_wheel };
            for (indices) |idx| {
                if (idx != -1) {
                    avg_omega += wheels[@intCast(idx)].getAngularVelocity();
                    avg_omega_denom += 1;
                }
            }

            if (avg_omega_denom > 0) {
                avg_omega = @abs(avg_omega * d.differential_ratio / @as(f32, @floatFromInt(avg_omega_denom))); // ignoring that the differentials may be rotating in different directions
                driven_differentials[num_driven_differentials] = .{ .differential = d, .angular_velocity = avg_omega, .clutch_to_differential_torque_ratio = d.engine_torque_ratio, .temp_torque_factor = 0 };
                num_driven_differentials += 1;

                // Remember min and max velocity
                differential_omega_min = math.min(differential_omega_min, avg_omega);
                differential_omega_max = math.max(differential_omega_max, avg_omega);
            }
        }

        if (self.differential_limited_slip_ratio < math.flt_max // Limited slip differential needs to be turned on
        and differential_omega_max > differential_omega_min) // There needs to be a velocity difference
        {
            // Calculate factor based on relative speed of a differential
            var sum_factor: f32 = 0.0;
            for (driven_differentials[0..num_driven_differentials]) |*d| {
                // Differential with max velocity gets factor 0, differential with min velocity 1
                d.temp_torque_factor = (differential_omega_max - d.angular_velocity) / (differential_omega_max - differential_omega_min);
                sum_factor += d.temp_torque_factor;
            }

            // Normalize the result
            for (driven_differentials[0..num_driven_differentials]) |*d|
                d.temp_torque_factor /= sum_factor;

            // Prevent div by zero
            differential_omega_min = math.max(@as(f32, 1.0e-3), differential_omega_min);
            differential_omega_max = math.max(@as(f32, 1.0e-3), differential_omega_max);

            // Map into a value that is 0 when the wheels are turning at an equal rate and 1 when the wheels are turning at mDifferentialLimitedSlipRatio
            const alpha = math.min((differential_omega_max / differential_omega_min - 1.0) / (self.differential_limited_slip_ratio - 1.0), @as(f32, 1.0));
            if (Core.enable_asserts) std.debug.assert(alpha >= 0.0);
            const one_min_alpha = 1.0 - alpha;

            // Update torque ratio for all differentials
            for (driven_differentials[0..num_driven_differentials]) |*d|
                d.clutch_to_differential_torque_ratio = one_min_alpha * d.clutch_to_differential_torque_ratio + alpha * d.temp_torque_factor;
        }

        if (Core.enable_asserts) {
            // Assert the values add up to 1
            var sum_torque_factors: f32 = 0.0;
            for (driven_differentials[0..num_driven_differentials]) |*d|
                sum_torque_factors += d.clutch_to_differential_torque_ratio;
            std.debug.assert(@abs(sum_torque_factors - 1.0) < 1.0e-6);
        }

        // Collect driven wheels (Array<DrivenWheel> in Jolt: the scratch buffer)
        const driven_wheels = self.driven_wheels_scratch;
        var num_driven_wheels: usize = 0;
        const transmission_ratio = self.transmission.getCurrentRatio();
        for (driven_differentials[0..num_driven_differentials]) |*dd| {
            const d = dd.differential.*;

            const wl: ?*WheelWV = if (d.left_wheel != -1) virtual.downcast(WheelWV, wheels[@intCast(d.left_wheel)]) else null;
            const wr: ?*WheelWV = if (d.right_wheel != -1) virtual.downcast(WheelWV, wheels[@intCast(d.right_wheel)]) else null;

            const clutch_to_wheel_ratio = transmission_ratio * d.differential_ratio;

            if (wl != null and wr != null) {
                // Calculate torque ratio
                const ratio = d.calculateTorqueRatio(wl.?.base.getAngularVelocity(), wr.?.base.getAngularVelocity());

                // Add both wheels
                driven_wheels[num_driven_wheels] = .{ .wheel = wl.?, .clutch_to_wheel_ratio = clutch_to_wheel_ratio, .clutch_to_wheel_torque_ratio = dd.clutch_to_differential_torque_ratio * ratio.left, .estimated_angular_impulse = 0.0 };
                driven_wheels[num_driven_wheels + 1] = .{ .wheel = wr.?, .clutch_to_wheel_ratio = clutch_to_wheel_ratio, .clutch_to_wheel_torque_ratio = dd.clutch_to_differential_torque_ratio * ratio.right, .estimated_angular_impulse = 0.0 };
                num_driven_wheels += 2;
            } else if (wl) |left| {
                // Only left wheel, all power to left
                driven_wheels[num_driven_wheels] = .{ .wheel = left, .clutch_to_wheel_ratio = clutch_to_wheel_ratio, .clutch_to_wheel_torque_ratio = dd.clutch_to_differential_torque_ratio, .estimated_angular_impulse = 0.0 };
                num_driven_wheels += 1;
            } else if (wr) |right| {
                // Only right wheel, all power to right
                driven_wheels[num_driven_wheels] = .{ .wheel = right, .clutch_to_wheel_ratio = clutch_to_wheel_ratio, .clutch_to_wheel_torque_ratio = dd.clutch_to_differential_torque_ratio, .estimated_angular_impulse = 0.0 };
                num_driven_wheels += 1;
            }
        }

        var solved = false;
        if (num_driven_wheels != 0) {
            // Define the torque at the clutch at time t as:
            //
            // tc(t):=S*(we(t)-sum(R(j)*ww(j,t),j,1,N)/N)
            //
            // Where:
            // S is the total strength of clutch (= friction * strength)
            // we(t) is the engine angular velocity at time t
            // R(j) is the total gear ratio of clutch to wheel for wheel j
            // ww(j,t) is the angular velocity of wheel j at time t
            // N is the amount of wheels
            //
            // The torque that increases the engine angular velocity at time t is:
            //
            // te(t):=TE-tc(t)
            //
            // Where:
            // TE is the torque delivered by the engine
            //
            // The torque that increases the wheel angular velocity for wheel i at time t is:
            //
            // tw(i,t):=TW(i)+R(i)*F(i)*tc(t)
            //
            // Where:
            // TW(i) is the torque applied to the wheel outside of the engine (brake + torque due to friction with the ground)
            // F(i) is the fraction of the engine torque applied from engine to wheel i
            //
            // Because the angular acceleration and torque are connected through: Torque = I * dw/dt
            //
            // We have the angular acceleration of the engine at time t:
            //
            // ddt_we(t):=te(t)/Ie
            //
            // Where:
            // Ie is the inertia of the engine
            //
            // We have the angular acceleration of wheel i at time t:
            //
            // ddt_ww(i,t):=tw(i,t)/Iw(i)
            //
            // Where:
            // Iw(i) is the inertia of wheel i
            //
            // We could take a simple Euler step to calculate the resulting accelerations but because the system is very stiff this turns out to be unstable, so we need to use implicit Euler instead:
            //
            // we(t+dt)=we(t)+dt*ddt_we(t+dt)
            //
            // and:
            //
            // ww(i,t+dt)=ww(i,t)+dt*ddt_ww(i,t+dt)
            //
            // Expanding both equations (the equations above are in wxMaxima format and this can easily be done by expand(%)):
            //
            // For wheel:
            //
            // ww(i,t+dt) + (S*dt*F(i)*R(i)*sum(R(j)*ww(j,t+dt),j,1,N))/(N*Iw(i)) - (S*dt*F(i)*R(i)*we(t+dt))/Iw(i) = ww(i,t)+(dt*TW(i))/Iw(i)
            //
            // For engine:
            //
            // we(t+dt) + (S*dt*we(t+dt))/Ie - (S*dt*sum(R(j)*ww(j,t+dt),j,1,N))/(Ie*N) = we(t)+(TE*dt)/Ie
            //
            // Defining a vector w(t) = (ww(1, t), ww(2, t), ..., ww(N, t), we(t)) we can write both equations as a matrix multiplication:
            //
            // a * w(t + dt) = b
            //
            // We then invert the matrix to get the new angular velocities.

            // Dimension of matrix is N + 1
            const n: u32 = @intCast(num_driven_wheels + 1);

            // Last column of w is for the engine angular velocity
            const engine = n - 1;

            // Define a and b (DynMatrix a(n, n), b(n, 1): views on the scratch buffer)
            var a: DynMatrix = .{ .rows = n, .cols = n, .elements = self.matrix_scratch[0 .. n * n], .allocator = self.base.allocator };
            var b: DynMatrix = .{ .rows = n, .cols = 1, .elements = self.matrix_scratch[n * n ..][0..n], .allocator = self.base.allocator };

            // Get number of driven wheels as a float
            const num_driven_wheels_float: f32 = @floatFromInt(num_driven_wheels);

            // Angular velocity of engine
            const w_engine = self.engine.getAngularVelocity();

            // Calculate the total strength of the clutch
            const clutch_strength = if (transmission_ratio != 0.0) self.transmission.getClutchFriction() * self.transmission.base.clutch_strength else 0.0;

            // dt / Ie
            const dt_div_ie = delta_time / self.engine.base.inertia;

            // Calculate scale factor for impulses based on previous delta time
            const impulse_scale = if (self.previous_delta_time > 0.0) delta_time / self.previous_delta_time else 0.0;

            // Iterate the rows for the wheels
            for (driven_wheels[0..num_driven_wheels], 0..) |*w_i, ii| {
                const i: u32 = @intCast(ii);
                const settings = w_i.wheel.getSettings();

                // Get wheel inertia
                const inertia = settings.inertia;

                // S * R(i)
                const s_r = clutch_strength * w_i.clutch_to_wheel_ratio;

                // dt * S * R(i) * F(i) / Iw
                const dt_s_r_f_div_iw = delta_time * s_r * w_i.clutch_to_wheel_torque_ratio / inertia;

                // Fill in the columns of a for wheel j
                for (driven_wheels[0..num_driven_wheels], 0..) |*w_j, j|
                    a.set(i, @intCast(j), dt_s_r_f_div_iw * w_j.clutch_to_wheel_ratio / num_driven_wheels_float);

                // Add ww(i, t+dt)
                a.set(i, i, a.get(i, i) + 1.0);

                // Add the column for the engine
                a.set(i, engine, -dt_s_r_f_div_iw);

                // Calculate external angular impulse operating on the wheel: TW(i) * dt
                var dt_tw: f32 = 0.0;

                // Combine brake with hand brake torque
                const brake_torque = self.brake_input * settings.max_brake_torque + self.hand_brake_input * settings.max_hand_brake_torque;
                if (brake_torque > 0.0) {
                    // We're braking
                    // Calculate brake angular impulse
                    var sign: f32 = undefined;
                    if (w_i.wheel.base.getAngularVelocity() != 0.0)
                        sign = math.sign(w_i.wheel.base.getAngularVelocity())
                    else
                        sign = math.sign(self.transmission.getCurrentRatio()); // When wheels have locked up use the transmission ratio to determine the sign
                    dt_tw = sign * delta_time * brake_torque;
                }

                if (w_i.wheel.base.hasContact()) {
                    // We have wheel contact with the floor
                    // Note that we don't know the torque due to the ground contact yet, so we use the impulse applied from the last frame to estimate it
                    // Wheel torque TW = force * radius = lambda / dt * radius
                    dt_tw += impulse_scale * w_i.wheel.base.getLongitudinalLambda() * settings.base.radius;
                }

                w_i.estimated_angular_impulse = dt_tw;

                // Fill in the constant b = ww(i,t)+(dt*TW(i))/Iw(i)
                b.set(i, 0, w_i.wheel.base.getAngularVelocity() - dt_tw / inertia);

                // To avoid looping over the wheels again, we also fill in the wheel columns of the engine row here
                a.set(engine, i, -dt_div_ie * s_r / num_driven_wheels_float);
            }

            // Finalize the engine row
            a.set(engine, engine, 1.0 + dt_div_ie * clutch_strength);
            b.set(engine, 0, w_engine + dt_div_ie * engine_torque);

            // Solve the linear equation
            if (gaussianElimination(&a, &b, .{})) {
                // Update the angular velocities for the wheels
                for (driven_wheels[0..num_driven_wheels], 0..) |*w_i, ii| {
                    const i: u32 = @intCast(ii);
                    const settings = w_i.wheel.getSettings();

                    // Get solved wheel angular velocity
                    var angular_velocity = b.get(i, 0);

                    // We estimated TW and applied it in the equation above, but we haven't actually applied this torque yet so we undo it here.
                    // It will be applied when we solve the actual braking / the constraints with the floor.
                    angular_velocity += w_i.estimated_angular_impulse / settings.inertia;

                    // Update angular velocity
                    w_i.wheel.base.setAngularVelocity(angular_velocity);
                }

                // Update the engine RPM
                self.engine.setCurrentRPM(b.get(engine, 0) * VehicleEngine.angular_velocity_to_rpm);

                // The speeds have been solved
                solved = true;
            } else {
                if (Core.enable_asserts) @panic("New engine/wheel speeds could not be calculated!");
            }
        }

        if (!solved) {
            // Engine not connected to wheels, apply all torque to engine rotation
            self.engine.applyTorque(engine_torque, delta_time);
        }

        // Calculate if any of the wheels are slipping, this is used to prevent gear switching
        var wheels_slipping = false;
        for (driven_wheels[0..num_driven_wheels]) |*w| {
            if (w.clutch_to_wheel_torque_ratio > 0.0 and (!w.wheel.base.hasContact() or w.wheel.longitudinal_slip > 0.1))
                wheels_slipping = true;
        }

        // Only allow shifting up when we're not slipping and we're increasing our RPM.
        // After a jump, we have a very high engine RPM but once we hit the ground the RPM should be decreasing and we don't want to shift up
        // during that time.
        const can_shift_up = !wheels_slipping and self.engine.getCurrentRPM() >= old_engine_rpm;

        // Update transmission
        self.transmission.update(delta_time, self.engine.getCurrentRPM(), self.forward_input, can_shift_up);

        // Braking
        for (wheels) |w_base| {
            const w = virtual.downcast(WheelWV, w_base);
            const settings = w.getSettings();

            // Combine brake with hand brake torque
            const brake_torque = self.brake_input * settings.max_brake_torque + self.hand_brake_input * settings.max_hand_brake_torque;
            if (brake_torque > 0.0) {
                // Calculate how much torque is needed to stop the wheels from rotating in this time step
                const brake_torque_to_lock_wheels = @abs(w.base.getAngularVelocity()) * settings.inertia / delta_time;
                if (brake_torque > brake_torque_to_lock_wheels) {
                    // Wheels are locked
                    w.base.setAngularVelocity(0.0);
                    w.brake_impulse = (brake_torque - brake_torque_to_lock_wheels) * delta_time / settings.base.radius;
                } else {
                    // Slow down the wheels
                    w.applyTorque(-math.sign(w.base.getAngularVelocity()) * brake_torque, delta_time);
                    w.brake_impulse = 0.0;
                }
            } else {
                // Not braking
                w.brake_impulse = 0.0;
            }
        }

        // Remember previous delta time so we can scale the impulses correctly
        self.previous_delta_time = delta_time;
    }

    pub fn solveLongitudinalAndLateralConstraints(self: *WheeledVehicleController, delta_time: f32) bool {
        var impulse = false;

        // max_lateral_friction_impulse (JPH_STACK_ALLOC in Jolt) is WheelWV.max_lateral_friction_impulse

        const constraint = self.base.constraint;
        for (constraint.getWheels(), 0..) |w_base, wheel_index| {
            if (w_base.hasContact()) {
                const w = virtual.downcast(WheelWV, w_base);
                const settings = w.getSettings();

                // Calculate max impulse that we can apply on the ground
                var max_longitudinal_friction_impulse: f32 = 0.0;
                self.tire_max_impulse_callback.call(@intCast(wheel_index), &max_longitudinal_friction_impulse, &w.max_lateral_friction_impulse, w.base.getSuspensionLambda(), w.combined_longitudinal_friction, w.combined_lateral_friction, w.longitudinal_slip, w.lateral_slip, delta_time);

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
                    // Assume we want to apply an angular impulse that makes the delta velocity between wheel and ground zero in one time step, calculate the amount of linear impulse needed to do that
                    const desired_angular_velocity = relative_longitudinal_velocity / settings.base.radius;
                    const linear_impulse = (w.base.getAngularVelocity() - desired_angular_velocity) * settings.inertia / settings.base.radius;

                    // Limit the impulse by max tire friction
                    const prev_lambda = w.base.getLongitudinalLambda();
                    min_longitudinal_impulse = math.clamp(prev_lambda + linear_impulse, -max_longitudinal_friction_impulse, max_longitudinal_friction_impulse);
                    max_longitudinal_impulse = min_longitudinal_impulse;

                    // Longitudinal impulse
                    if (w.base.solveLongitudinalConstraintPart(constraint, min_longitudinal_impulse, max_longitudinal_impulse))
                        impulse = true;

                    // Update the angular velocity of the wheels according to the lambda that was applied
                    w.base.setAngularVelocity(w.base.getAngularVelocity() - (w.base.getLongitudinalLambda() - prev_lambda) * settings.base.radius / settings.inertia);
                }
            }
        }

        for (constraint.getWheels()) |w_base| {
            if (w_base.hasContact()) {
                const w = virtual.downcast(WheelWV, w_base);

                // Lateral friction
                const max_lateral_impulse = w.max_lateral_friction_impulse;
                if (w.base.solveLateralConstraintPart(constraint, -max_lateral_impulse, max_lateral_impulse))
                    impulse = true;
            }
        }

        return impulse;
    }

    // TODO(debug renderer): Draw

    pub fn saveState(self: *const WheeledVehicleController, stream: *StateRecorder) void {
        stream.write(self.forward_input);
        stream.write(self.right_input);
        stream.write(self.brake_input);
        stream.write(self.hand_brake_input);
        stream.write(self.previous_delta_time);

        self.engine.saveState(stream);
        self.transmission.saveState(stream);
    }

    pub fn restoreState(self: *WheeledVehicleController, stream: *StateRecorder) void {
        stream.read(&self.forward_input);
        stream.read(&self.right_input);
        stream.read(&self.brake_input);
        stream.read(&self.hand_brake_input);
        stream.read(&self.previous_delta_time);

        self.engine.restoreState(stream);
        self.transmission.restoreState(stream);
    }
};

const testing = std.testing;

/// SaveBinaryState of `object` into `out`
fn testSave(object: anytype, out: *std.Io.Writer.Allocating) void {
    const StreamOutWrapper = @import("../../Core/StreamWrapper.zig").StreamOutWrapper;
    var wrapper: StreamOutWrapper = .init(&out.writer);
    object.saveBinaryState(wrapper.streamOut());
}

/// RestoreBinaryState of `object` from `bytes`, the stream must not hit its end
fn testRestore(object: anytype, bytes: []const u8) !void {
    const StreamInWrapper = @import("../../Core/StreamWrapper.zig").StreamInWrapper;
    var reader: std.Io.Reader = .fixed(bytes);
    var wrapper: StreamInWrapper = .init(&reader);
    try object.restoreBinaryState(wrapper.streamIn());
    try testing.expect(!wrapper.isEOF() and !wrapper.isFailed());
}

test "WheelSettingsWV defaults, IsKindOf and binary state" {
    const allocator = testing.allocator;

    const settings = try WheelSettingsWV.create(allocator);
    var ref = Ref(WheelSettings).init(&settings.base);
    defer ref.deinit();
    try testing.expectEqual(@as(f32, 0.9), settings.inertia);
    try testing.expectEqual(@as(f32, 0.2), settings.angular_damping);
    try testing.expectEqual(math.degreesToRadians(70.0), settings.max_steer_angle);
    try testing.expectEqual(@as(f32, 1500.0), settings.max_brake_torque);
    try testing.expectEqual(@as(f32, 4000.0), settings.max_hand_brake_torque);
    try testing.expectEqual(@as(usize, 3), settings.longitudinal_friction.points.items.len);
    try testing.expectEqual(@as(f32, 1.2), settings.longitudinal_friction.getValue(0.06));
    try testing.expectEqual(@as(f32, 1.2), settings.lateral_friction.getValue(3.0));
    try testing.expectEqual(@as(f32, 1.0), settings.lateral_friction.getValue(30.0));
    try testing.expect(settings.base.isKindOf(WheelSettings) and settings.base.isKindOf(WheelSettingsWV));
    try testing.expect(settings.base.cast(WheelSettingsWV) == settings);

    // The binary state: WheelSettings followed by the members of WheelSettingsWV (through the base class)
    settings.inertia = 2.0;
    settings.base.radius = 0.4;
    try settings.lateral_friction.addPoint(allocator, 40.0, 0.5);
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    testSave(&settings.base, &out);
    try testing.expectEqual(@as(usize, 6 * 12 + 3 * 4 + 9 + 2 * 4 + 1 + 3 * 4 + (4 + 3 * 8) + (4 + 4 * 8) + 2 * 4), out.written().len);

    const restored = try WheelSettingsWV.create(allocator);
    var restored_ref = Ref(WheelSettings).init(&restored.base);
    defer restored_ref.deinit();
    try testRestore(&restored.base, out.written());
    try testing.expectEqual(@as(f32, 2.0), restored.inertia);
    try testing.expectEqual(@as(f32, 0.4), restored.base.radius);
    try testing.expectEqual(@as(usize, 4), restored.lateral_friction.points.items.len);
    var out2: std.Io.Writer.Allocating = .init(allocator);
    defer out2.deinit();
    testSave(&restored.base, &out2);
    try testing.expectEqualSlices(u8, out.written(), out2.written());

    // Out of memory: creation and restore leave nothing behind
    const S = struct {
        fn run(a: Allocator, bytes: []const u8) !void {
            const s = try WheelSettingsWV.create(a);
            var s_ref = Ref(WheelSettings).init(&s.base);
            defer s_ref.deinit();
            try testRestore(&s.base, bytes);
        }
    };
    try testing.checkAllAllocationFailures(allocator, S.run, .{out.written()});
}

test "WheeledVehicleControllerSettings defaults, RTTI hash and binary state" {
    const allocator = testing.allocator;

    const settings = try WheeledVehicleControllerSettings.create(allocator);
    var ref = Ref(VehicleControllerSettings).init(&settings.base);
    defer ref.deinit();
    try testing.expectEqual(@as(usize, 3), settings.engine.normalized_torque.points.items.len);
    try testing.expectEqual(@as(usize, 5), settings.transmission.gear_ratios.items.len);
    try testing.expectEqual(@as(usize, 1), settings.transmission.reverse_gear_ratios.items.len);
    try testing.expectEqual(@as(usize, 0), settings.differentials.items.len);
    try testing.expectEqual(@as(f32, 1.4), settings.differential_limited_slip_ratio);
    try testing.expectEqual(VehicleControllerSettings.rttiHash("WheeledVehicleControllerSettings"), settings.base.getRTTIHash());

    // The binary state (through the base class), new differentials are default constructed by the restore
    try settings.differentials.append(allocator, .{ .left_wheel = 2, .right_wheel = 3, .engine_torque_ratio = 0.25 });
    try settings.differentials.append(allocator, .{ .limited_slip_ratio = math.flt_max });
    settings.differential_limited_slip_ratio = 2.0;
    settings.transmission.mode = .manual;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    testSave(&settings.base, &out);

    const restored = try WheeledVehicleControllerSettings.create(allocator);
    var restored_ref = Ref(VehicleControllerSettings).init(&restored.base);
    defer restored_ref.deinit();
    try testRestore(&restored.base, out.written());
    try testing.expectEqual(@as(usize, 2), restored.differentials.items.len);
    try testing.expectEqual(@as(i32, 3), restored.differentials.items[0].right_wheel);
    try testing.expectEqual(math.flt_max, restored.differentials.items[1].limited_slip_ratio);
    try testing.expectEqual(@as(f32, 2.0), restored.differential_limited_slip_ratio);
    try testing.expectEqual(VehicleTransmissionFile.TransmissionMode.manual, restored.transmission.mode);
    var out2: std.Io.Writer.Allocating = .init(allocator);
    defer out2.deinit();
    testSave(&restored.base, &out2);
    try testing.expectEqualSlices(u8, out.written(), out2.written());

    // Out of memory: creation and restore leave nothing behind
    const S = struct {
        fn run(a: Allocator, bytes: []const u8) !void {
            const s = try WheeledVehicleControllerSettings.create(a);
            var s_ref = Ref(VehicleControllerSettings).init(&s.base);
            defer s_ref.deinit();
            try testRestore(&s.base, bytes);
        }
    };
    try testing.checkAllAllocationFailures(allocator, S.run, .{out.written()});
}

test "WheeledVehicleController TireMaxImpulseCallback default and with a context" {
    const TireMaxImpulseCallback = WheeledVehicleController.TireMaxImpulseCallback;
    var longitudinal: f32 = 0.0;
    var lateral: f32 = 0.0;
    TireMaxImpulseCallback.default.call(0, &longitudinal, &lateral, 10.0, 0.5, 0.25, 0.1, 0.2, 1.0 / 60.0);
    try testing.expectEqual(@as(f32, 5.0), longitudinal);
    try testing.expectEqual(@as(f32, 2.5), lateral);

    const Grip = struct {
        factor: f32,
        fn apply(self: *const @This(), wheel_index: u32, longitudinal_impulse: *f32, lateral_impulse: *f32, suspension_impulse: f32, longitudinal_friction: f32, lateral_friction: f32, longitudinal_slip: f32, lateral_slip: f32, delta_time: f32) void {
            _ = .{ wheel_index, longitudinal_slip, lateral_slip, delta_time };
            longitudinal_impulse.* = self.factor * longitudinal_friction * suspension_impulse;
            lateral_impulse.* = self.factor * lateral_friction * suspension_impulse;
        }
    };
    const grip: Grip = .{ .factor = 2.0 };
    const callback: TireMaxImpulseCallback = .init(&grip, Grip.apply);
    callback.call(1, &longitudinal, &lateral, 10.0, 0.5, 0.25, 0.1, 0.2, 1.0 / 60.0);
    try testing.expectEqual(@as(f32, 10.0), longitudinal);
    try testing.expectEqual(@as(f32, 5.0), lateral);
}
