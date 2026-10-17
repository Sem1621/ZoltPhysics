//! Port of: Jolt/Physics/Vehicle/MotorcycleController.h, Jolt/Physics/Vehicle/MotorcycleController.cpp
//! Status: complete
//!
//! - JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(MotorcycleControllerSettings):
//!   TODO(serialization) (ObjectStream is Phase 8, CharacterVehicleArchitecture E1; the binary state is ported).
//!   JPH_DEBUG_RENDERER (`Draw`): TODO(debug renderer).
//! - CharacterVehicleArchitecture E2: `MotorcycleControllerSettings` embeds `base: WheeledVehicleControllerSettings`
//!   (`rtti_name`, the settings class of `RegisterTypes.vehicle_controller_settings_types`, E3), `MotorcycleController`
//!   embeds `base: WheeledVehicleController`: a level under the wheeled classes, built with their `initDerived`. Jolt's
//!   `WheeledVehicleController::PreCollide(...)` / `WheeledVehicleControllerSettings::SaveBinaryState(...)` are the
//!   static calls `self.base.preCollide(...)` / `self.base.saveBinaryState(...)`. The overrides that the motorcycle does
//!   not declare (ConstructWheel, AllowSleep, PostCollide) are the wheeled controller's.
//! - The protected virtual functions are `pub` (VehicleConstraint is a friend of the controller in Jolt).
//! - Allocation (E2): `create`, `getSettings` and `constructController` return `Allocator.Error` and leave nothing
//!   behind. PreCollide and SolveLongitudinalAndLateralConstraints run inside the physics update and allocate nothing.
//! - Jolt behavior reproduced on purpose:
//!   - `MotorcycleController::SaveState` / `RestoreState` save and restore `mTargetLean` only, not
//!     `mLeanSpringIntegratedDeltaAngle` (CharacterVehicleArchitecture, "Settled in the foundation review"): a restore
//!     keeps the integrated angle of the restoring controller, so a lean spring integration coefficient other than 0
//!     makes a restored simulation diverge, in Jolt too. Neither does it save `mAppliedImpulse` (PreCollide resets it
//!     before it is read) and the configuration (`mEnableLeanController`, the lean parameters).
//!   - `GetSettings` assigns `settings->mLeanSpringDamping = settings->mLeanSpringDamping` (a self assignment), so the
//!     returned settings have the default lean spring damping (1000), not the controller's.

const std = @import("std");
const Allocator = std.mem.Allocator;
const virtual = @import("../../Core/Virtual.zig");
const Ref = @import("../../Core/Reference.zig").Ref;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const Trigonometry = @import("../../Math/Trigonometry.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Quat = @import("../../Math/Quat.zig").Quat;
const DeterminismLog = @import("../DeterminismLog.zig");
const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const VehicleConstraint = @import("VehicleConstraint.zig").VehicleConstraint;
const VehicleControllerFile = @import("VehicleController.zig");
const VehicleController = VehicleControllerFile.VehicleController;
const VehicleControllerSettings = VehicleControllerFile.VehicleControllerSettings;
const WheeledVehicleControllerFile = @import("WheeledVehicleController.zig");
const WheeledVehicleController = WheeledVehicleControllerFile.WheeledVehicleController;
const WheeledVehicleControllerSettings = WheeledVehicleControllerFile.WheeledVehicleControllerSettings;
const WheelWV = WheeledVehicleControllerFile.WheelWV;

/// Settings of a two wheeled motorcycle (adds a spring to balance the motorcycle)
/// Note: The motor cycle controller is still in development and may need a lot of tweaks/hacks to work properly!
pub const MotorcycleControllerSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, MotorcycleControllerSettings)

    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "MotorcycleControllerSettings";
    pub const overrides = .{ .constructController, .saveBinaryState, .restoreBinaryState };

    base: WheeledVehicleControllerSettings,

    /// How far we're willing to make the bike lean over in turns (in radians)
    max_lean_angle: f32 = math.degreesToRadians(45.0),
    /// Spring constant for the lean spring
    lean_spring_constant: f32 = 5000.0,
    /// Spring damping constant for the lean spring
    lean_spring_damping: f32 = 1000.0,
    /// The lean spring applies an additional force equal to this coefficient * Integral(delta angle, 0, t), this effectively makes the lean spring a PID controller
    lean_spring_integration_coefficient: f32 = 0.0,
    /// How much to decay the angle integral when the wheels are not touching the floor: new_value = e^(-decay * t) * initial_value
    lean_spring_integration_coefficient_decay: f32 = 4.0,
    /// How much to smooth the lean angle (0 = no smoothing, 1 = lean angle never changes)
    /// Note that this is frame rate dependent because the formula is: smoothing_factor * previous + (1 - smoothing_factor) * current
    lean_smoothing_factor: f32 = 0.8,

    /// Constructor of the base part of a class T derived from MotorcycleControllerSettings (the wheeled settings allocate
    /// their engine curve and gear ratios with `allocator`, the destructor frees them)
    pub fn initDerived(comptime T: type, allocator: Allocator) Allocator.Error!MotorcycleControllerSettings {
        return .{ .base = try .initDerived(T, allocator) };
    }

    /// Constructor of settings that are not on the heap (the owner calls `base.base.deinit()`)
    pub fn init(allocator: Allocator) Allocator.Error!MotorcycleControllerSettings {
        return initDerived(MotorcycleControllerSettings, allocator);
    }

    /// new MotorcycleControllerSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*MotorcycleControllerSettings {
        const self = try allocator.create(MotorcycleControllerSettings);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for VehicleConstraintSettings.restoreBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// Upcasts (implicit in C++)
    pub fn asControllerSettings(self: *MotorcycleControllerSettings) *VehicleControllerSettings {
        return &self.base.base;
    }

    pub fn asControllerSettingsConst(self: *const MotorcycleControllerSettings) *const VehicleControllerSettings {
        return &self.base.base;
    }

    // See: VehicleControllerSettings
    pub fn constructController(self: *const MotorcycleControllerSettings, allocator: Allocator, constraint: *VehicleConstraint) Allocator.Error!*VehicleController {
        const controller = try MotorcycleController.create(allocator, self, constraint);
        return &controller.base.base;
    }

    pub fn saveBinaryState(self: *const MotorcycleControllerSettings, stream: StreamOut) void {
        self.base.saveBinaryState(stream);

        stream.write(self.max_lean_angle);
        stream.write(self.lean_spring_constant);
        stream.write(self.lean_spring_damping);
        stream.write(self.lean_spring_integration_coefficient);
        stream.write(self.lean_spring_integration_coefficient_decay);
        stream.write(self.lean_smoothing_factor);
    }

    pub fn restoreBinaryState(self: *MotorcycleControllerSettings, stream: StreamIn) Allocator.Error!void {
        try self.base.restoreBinaryState(stream);

        stream.read(&self.max_lean_angle);
        stream.read(&self.lean_spring_constant);
        stream.read(&self.lean_spring_damping);
        stream.read(&self.lean_spring_integration_coefficient);
        stream.read(&self.lean_spring_integration_coefficient_decay);
        stream.read(&self.lean_smoothing_factor);
    }
};

/// Runtime controller class
pub const MotorcycleController = struct {
    pub const overrides = .{ .getSettings, .preCollide, .solveLongitudinalAndLateralConstraints, .saveState, .restoreState };

    base: WheeledVehicleController,

    // Configuration properties
    enable_lean_controller: bool = true,
    enable_lean_steering_limit: bool = true,
    max_lean_angle: f32,
    lean_spring_constant: f32,
    lean_spring_damping: f32,
    lean_spring_integration_coefficient: f32,
    lean_spring_integration_coefficient_decay: f32,
    lean_smoothing_factor: f32,

    /// Run-time calculated target lean vector
    target_lean: Vec3 = Vec3.zero(),

    /// Integrated error for the lean spring
    lean_spring_integrated_delta_angle: f32 = 0.0,

    /// Run-time total angular impulse applied to turn the cycle towards the target lean angle
    applied_impulse: f32 = 0.0,

    /// Constructor of the base part of a class T derived from MotorcycleController. On failure nothing is left
    /// allocated.
    pub fn initDerived(comptime T: type, allocator: Allocator, settings: *const MotorcycleControllerSettings, constraint: *VehicleConstraint) Allocator.Error!MotorcycleController {
        return .{
            .base = try .initDerived(T, allocator, &settings.base, constraint),
            .max_lean_angle = settings.max_lean_angle,
            .lean_spring_constant = settings.lean_spring_constant,
            .lean_spring_damping = settings.lean_spring_damping,
            .lean_spring_integration_coefficient = settings.lean_spring_integration_coefficient,
            .lean_spring_integration_coefficient_decay = settings.lean_spring_integration_coefficient_decay,
            .lean_smoothing_factor = settings.lean_smoothing_factor,
        };
    }

    /// Constructor
    pub fn init(allocator: Allocator, settings: *const MotorcycleControllerSettings, constraint: *VehicleConstraint) Allocator.Error!MotorcycleController {
        return initDerived(MotorcycleController, allocator, settings, constraint);
    }

    /// new MotorcycleController(inSettings, inConstraint) (VehicleConstraint destroys it)
    pub fn create(allocator: Allocator, settings: *const MotorcycleControllerSettings, constraint: *VehicleConstraint) Allocator.Error!*MotorcycleController {
        const self = try allocator.create(MotorcycleController);
        errdefer allocator.destroy(self);
        self.* = try .init(allocator, settings, constraint);
        return self;
    }

    /// Upcasts (implicit in C++)
    pub fn asController(self: *MotorcycleController) *VehicleController {
        return &self.base.base;
    }

    pub fn asControllerConst(self: *const MotorcycleController) *const VehicleController {
        return &self.base.base;
    }

    /// The vehicle constraint (Jolt's mConstraint)
    fn vehicleConstraint(self: *const MotorcycleController) *VehicleConstraint {
        return self.base.base.constraint;
    }

    /// Get the distance between the front and back wheels
    pub fn getWheelBase(self: *const MotorcycleController) f32 {
        var low: f32 = math.flt_max;
        var high: f32 = -math.flt_max;

        const c = self.vehicleConstraint();
        for (c.getWheels()) |w| {
            const s = w.getSettings();

            // Measure distance along the forward axis by looking at the fully extended suspension.
            // If the suspension force point is active, use that instead.
            const force_point = if (s.enable_suspension_force_point) s.suspension_force_point else s.position.add(s.suspension_direction.mulScalar(s.suspension_max_length));
            const value = force_point.dot(c.getLocalForward());

            // Update min and max
            low = math.min(low, value);
            high = math.max(high, value);
        }

        return high - low;
    }

    /// Enable or disable the lean spring. This allows you to temporarily disable the lean spring to allow the motorcycle to fall over.
    pub fn enableLeanController(self: *MotorcycleController, enable: bool) void {
        self.enable_lean_controller = enable;
    }

    /// Check if the lean spring is enabled.
    pub fn isLeanControllerEnabled(self: *const MotorcycleController) bool {
        return self.enable_lean_controller;
    }

    /// Enable or disable the lean steering limit. When enabled (default) the steering angle is limited based on the vehicle speed to prevent steering that would cause an inertial force that causes the motorcycle to topple over.
    pub fn enableLeanSteeringLimit(self: *MotorcycleController, enable: bool) void {
        self.enable_lean_steering_limit = enable;
    }

    pub fn isLeanSteeringLimitEnabled(self: *const MotorcycleController) bool {
        return self.enable_lean_steering_limit;
    }

    /// Spring constant for the lean spring
    pub fn setLeanSpringConstant(self: *MotorcycleController, constant: f32) void {
        self.lean_spring_constant = constant;
    }

    pub fn getLeanSpringConstant(self: *const MotorcycleController) f32 {
        return self.lean_spring_constant;
    }

    /// Spring damping constant for the lean spring
    pub fn setLeanSpringDamping(self: *MotorcycleController, damping: f32) void {
        self.lean_spring_damping = damping;
    }

    pub fn getLeanSpringDamping(self: *const MotorcycleController) f32 {
        return self.lean_spring_damping;
    }

    /// The lean spring applies an additional force equal to this coefficient * Integral(delta angle, 0, t), this effectively makes the lean spring a PID controller
    pub fn setLeanSpringIntegrationCoefficient(self: *MotorcycleController, coefficient: f32) void {
        self.lean_spring_integration_coefficient = coefficient;
    }

    pub fn getLeanSpringIntegrationCoefficient(self: *const MotorcycleController) f32 {
        return self.lean_spring_integration_coefficient;
    }

    /// How much to decay the angle integral when the wheels are not touching the floor: new_value = e^(-decay * t) * initial_value
    pub fn setLeanSpringIntegrationCoefficientDecay(self: *MotorcycleController, decay: f32) void {
        self.lean_spring_integration_coefficient_decay = decay;
    }

    pub fn getLeanSpringIntegrationCoefficientDecay(self: *const MotorcycleController) f32 {
        return self.lean_spring_integration_coefficient_decay;
    }

    /// How much to smooth the lean angle (0 = no smoothing, 1 = lean angle never changes)
    /// Note that this is frame rate dependent because the formula is: smoothing_factor * previous + (1 - smoothing_factor) * current
    pub fn setLeanSmoothingFactor(self: *MotorcycleController, factor: f32) void {
        self.lean_smoothing_factor = factor;
    }

    pub fn getLeanSmoothingFactor(self: *const MotorcycleController) f32 {
        return self.lean_smoothing_factor;
    }

    // See: VehicleController
    pub fn getSettings(self: *const MotorcycleController) Allocator.Error!Ref(VehicleControllerSettings) {
        const settings = try MotorcycleControllerSettings.create(self.base.base.allocator);
        var result = Ref(VehicleControllerSettings).init(settings.asControllerSettings());
        errdefer result.deinit();
        try self.base.toSettings(&settings.base);
        settings.max_lean_angle = self.max_lean_angle;
        settings.lean_spring_constant = self.lean_spring_constant;
        settings.lean_spring_damping = settings.lean_spring_damping; // Jolt assigns the settings' own value (see the file comment)
        settings.lean_spring_integration_coefficient = self.lean_spring_integration_coefficient;
        settings.lean_spring_integration_coefficient_decay = self.lean_spring_integration_coefficient_decay;
        settings.lean_smoothing_factor = self.lean_smoothing_factor;
        return result;
    }

    pub fn preCollide(self: *MotorcycleController, delta_time: f32, physics_system: *PhysicsSystem) void {
        self.base.preCollide(delta_time, physics_system);

        const c = self.vehicleConstraint();
        const body = c.getVehicleBody();
        const forward = body.getRotation().mulVec3(c.getLocalForward());
        const wheel_base = self.getWheelBase();
        const world_up = c.getWorldUp();

        if (self.enable_lean_controller) {
            // Calculate the target lean vector, this is in the direction of the total applied impulse by the ground on the wheels
            var target_lean = Vec3.zero();
            for (c.getWheels()) |w| {
                if (w.hasContact())
                    target_lean = target_lean.add(w.getContactNormal().mulScalar(w.getSuspensionLambda()).add(w.getContactLateral().mulScalar(w.getLateralLambda())));
            }

            // Normalize the impulse
            target_lean = target_lean.normalizedOr(world_up);

            // Smooth the impulse to avoid jittery behavior
            self.target_lean = self.target_lean.mulScalar(self.lean_smoothing_factor).add(target_lean.mulScalar(1.0 - self.lean_smoothing_factor));

            // Remove forward component, we can only lean sideways
            self.target_lean = self.target_lean.sub(forward.mulScalar(self.target_lean.dot(forward)));
            self.target_lean = self.target_lean.normalizedOr(world_up);

            // Clamp the target lean against the max lean angle
            var adjusted_world_up = world_up.sub(forward.mulScalar(world_up.dot(forward)));
            adjusted_world_up = adjusted_world_up.normalizedOr(world_up);
            const w_angle = -math.sign(self.target_lean.cross(adjusted_world_up).dot(forward)) * Trigonometry.acos(self.target_lean.dot(adjusted_world_up));
            if (@abs(w_angle) > self.max_lean_angle)
                self.target_lean = Quat.rotation(forward, math.sign(w_angle) * self.max_lean_angle).mulVec3(adjusted_world_up);

            // Integrate the delta angle
            const up = body.getRotation().mulVec3(c.getLocalUp());
            const d_angle = -math.sign(self.target_lean.cross(up).dot(forward)) * Trigonometry.acos(self.target_lean.dot(up));
            self.lean_spring_integrated_delta_angle += d_angle * delta_time;
        } else {
            // Controller not enabled, reset target lean
            self.target_lean = world_up;

            // Reset integrated delta angle
            self.lean_spring_integrated_delta_angle = 0;
        }

        if (DeterminismLog.enabled) DeterminismLog.log(.{ "WheeledVehicleController::PreCollide: mTargetLean: ", self.target_lean });

        // Calculate max steering angle based on the max lean angle we're willing to take
        // See: https://en.wikipedia.org/wiki/Bicycle_and_motorcycle_dynamics#Leaning
        // LeanAngle = Atan(Velocity^2 / (Gravity * TurnRadius))
        // And: https://en.wikipedia.org/wiki/Turning_radius (we're ignoring the tire width)
        // The CasterAngle is the added according to https://en.wikipedia.org/wiki/Bicycle_and_motorcycle_dynamics#Turning (this is the same formula but without small angle approximation)
        // TurnRadius = WheelBase / (Sin(SteerAngle) * Cos(CasterAngle))
        // => SteerAngle = ASin(WheelBase * Tan(LeanAngle) * Gravity / (Velocity^2 * Cos(CasterAngle))
        // The caster angle is different for each wheel so we can only calculate part of the equation here
        const max_steer_angle_factor = wheel_base * Trigonometry.tan(self.max_lean_angle) * (if (c.isGravityOverridden()) c.getGravityOverride() else physics_system.getGravity()).length();

        // Calculate forward velocity
        const velocity = body.getLinearVelocity().dot(forward);
        const velocity_sq = math.square(velocity);

        // Decompose steering into sign and direction
        const steer_strength = @abs(self.base.right_input);
        const steer_sign = -math.sign(self.base.right_input);

        for (c.getWheelsMut()) |w_base| {
            const w = virtual.downcast(WheelWV, w_base);
            const s = w.getSettings();

            // Check if this wheel can steer
            if (s.max_steer_angle != 0.0) {
                // Calculate cos(caster angle), the angle between the steering axis and the up vector
                const cos_caster_angle = s.base.steering_axis.dot(c.getLocalUp());

                // Calculate steer angle
                var steer_angle = steer_strength * w.getSettings().max_steer_angle;

                // Clamp to max steering angle
                if (self.enable_lean_steering_limit and velocity_sq > 1.0e-6 and cos_caster_angle > 1.0e-6) {
                    const max_steer_angle = Trigonometry.asin(max_steer_angle_factor / (velocity_sq * cos_caster_angle));
                    steer_angle = math.min(steer_angle, max_steer_angle);
                }

                // Set steering angle
                w.base.setSteerAngle(steer_sign * steer_angle);
            }
        }

        // Reset applied impulse
        self.applied_impulse = 0;
    }

    pub fn solveLongitudinalAndLateralConstraints(self: *MotorcycleController, delta_time: f32) bool {
        var impulse = self.base.solveLongitudinalAndLateralConstraints(delta_time);

        if (self.enable_lean_controller) {
            const c = self.vehicleConstraint();

            // Only apply a lean impulse if all wheels are in contact, otherwise we can easily spin out
            var all_in_contact = true;
            for (c.getWheels()) |w| {
                if (!w.hasContact() or w.getSuspensionLambda() <= 0.0) {
                    all_in_contact = false;
                    break;
                }
            }

            if (all_in_contact) {
                const body = c.getVehicleBody();
                const mp = body.getMotionProperties();

                const forward = body.getRotation().mulVec3(c.getLocalForward());
                const up = body.getRotation().mulVec3(c.getLocalUp());

                // Calculate delta to target angle and derivative
                const d_angle = -math.sign(self.target_lean.cross(up).dot(forward)) * Trigonometry.acos(self.target_lean.dot(up));
                const ddt_angle = body.getAngularVelocity().dot(forward);

                // Calculate impulse to apply to get to target lean angle
                const total_impulse = (self.lean_spring_constant * d_angle - self.lean_spring_damping * ddt_angle + self.lean_spring_integration_coefficient * self.lean_spring_integrated_delta_angle) * delta_time;

                // Remember angular velocity pre angular impulse
                const old_w = mp.getAngularVelocity();

                // Apply impulse taking into account the impulse we've applied earlier
                const delta_impulse = total_impulse - self.applied_impulse;
                body.addAngularImpulse(forward.mulScalar(delta_impulse));
                self.applied_impulse = total_impulse;

                // Calculate delta angular velocity due to angular impulse
                const dw = mp.getAngularVelocity().sub(old_w);
                var linear_acceleration = Vec3.zero();
                var total_lambda: f32 = 0.0;
                for (c.getWheels()) |w_base| {
                    const w = virtual.downcast(WheelWV, w_base);

                    // We weigh the importance of each contact point according to the contact force
                    const lambda = w.base.getSuspensionLambda();
                    total_lambda += lambda;

                    // Linear acceleration of contact point is dw x com_to_contact
                    const r = w.base.getContactPosition().sub(body.getCenterOfMassPosition()).toVec3();
                    linear_acceleration = linear_acceleration.add(dw.cross(r).mulScalar(lambda));
                }

                // Apply linear impulse to COM to cancel the average velocity change on the wheels due to the angular impulse
                const linear_impulse = linear_acceleration.negate().divScalar(total_lambda * mp.getInverseMass());
                body.addImpulse(linear_impulse);

                // Return true if we applied an impulse
                if (delta_impulse != 0.0)
                    impulse = true;
            } else {
                // Decay the integrated angle because we won't be applying a torque this frame
                // Uses 1st order Taylor approximation of e^(-decay * dt) = 1 - decay * dt
                self.lean_spring_integrated_delta_angle *= math.max(@as(f32, 0.0), 1.0 - self.lean_spring_integration_coefficient_decay * delta_time);
            }
        }

        return impulse;
    }

    pub fn saveState(self: *const MotorcycleController, stream: *StateRecorder) void {
        self.base.saveState(stream);

        stream.write(self.target_lean);
    }

    pub fn restoreState(self: *MotorcycleController, stream: *StateRecorder) void {
        self.base.restoreState(stream);

        stream.read(&self.target_lean);
    }

    // TODO(debug renderer): Draw
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (Jolt has no unit tests of the motorcycle controller): the settings and their binary state, GetWheelBase, a
// motorcycle that drives, leans into turns (with and without the lean controller and the lean steering limit, with the
// lean spring integration), brakes, the settings and state round trips and allocation failure. The bit exact comparison
// with Jolt is the simulation parity (ZoltParity/Physics/SimScenes.zig, the vehicle scenes) and VehiclePartsParity.zig.

const testing = std.testing;
const TempAllocatorImpl = @import("../../Core/TempAllocator.zig").TempAllocatorImpl;
const JobSystemSingleThreaded = @import("../../Core/JobSystemSingleThreaded.zig").JobSystemSingleThreaded;
const RefConst = @import("../../Core/Reference.zig").RefConst;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");
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
const VehicleCollisionTesterCastSphere = VehicleCollisionTesterFile.VehicleCollisionTesterCastSphere;
const WheelFile = @import("Wheel.zig");
const WheelSettings = WheelFile.WheelSettings;
const WheelSettingsWV = WheeledVehicleControllerFile.WheelSettingsWV;

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
        const floor = try BoxShape.create(allocator, Vec3.init(200, 1, 200), .{ .convex_radius = 0.0 });
        _ = try self.addBody(floor.asShape(), RVec3.init(0, -1, 0), Quat.identity(), .static, 0);
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
    fn addBody(self: *TestWorld, shape: *const Shape, position: RVec3, rotation: Quat, motion_type: MotionType, mass: f32) !*Body {
        var settings: BodyCreationSettings = .initWithShape(shape, position, rotation, motion_type, if (motion_type == .static) TestLayers.non_moving else TestLayers.moving);
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

/// The caster angle of the front wheel of the test motorcycle
const caster_angle = math.degreesToRadians(30.0);

/// The suspension direction of the front wheel of the test motorcycle (tilted by the caster angle)
fn frontSuspensionDirection() Vec3 {
    return Vec3.init(0, -1, Trigonometry.tan(caster_angle)).normalized();
}

/// The motorcycle of the tests (Jolt's MotorcycleTest, SimScenes' addTestMotorcycle): a front wheel with a caster angle
/// and the suspension force point enabled, a back wheel driven by one differential, the given lean spring integration
/// coefficient. The caller calls `asSettings().deinit()`.
fn makeMotorcycleSettings(allocator: Allocator, integration_coefficient: f32) !VehicleConstraintSettings {
    var vehicle = VehicleConstraintSettings.init(allocator);
    errdefer vehicle.asSettings().deinit();
    vehicle.max_pitch_roll_angle = math.degreesToRadians(60.0);

    const direction = frontSuspensionDirection();
    const front = try WheelSettingsWV.create(allocator);
    var front_ref = Ref(WheelSettings).init(&front.base);
    defer front_ref.deinit();
    front.base.position = Vec3.init(0, -0.27, 0.75);
    front.base.suspension_direction = direction;
    front.base.steering_axis = direction.negate();
    front.base.enable_suspension_force_point = true;
    front.base.suspension_force_point = front.base.position.add(direction.mulScalar(0.3));
    front.base.radius = 0.31;
    front.base.width = 0.05;
    front.max_steer_angle = math.degreesToRadians(30.0);
    front.max_brake_torque = 500.0;
    try vehicle.addWheel(&front.base);

    const back = try WheelSettingsWV.create(allocator);
    var back_ref = Ref(WheelSettings).init(&back.base);
    defer back_ref.deinit();
    back.base.position = Vec3.init(0, -0.27, -0.75);
    back.base.radius = 0.31;
    back.base.width = 0.05;
    back.base.suspension_spring.frequency = 2.0;
    back.max_steer_angle = 0.0;
    back.max_brake_torque = 250.0;
    try vehicle.addWheel(&back.base);

    const controller = try MotorcycleControllerSettings.create(allocator);
    vehicle.setController(controller.asControllerSettings());
    controller.base.engine.max_torque = 150.0;
    controller.base.engine.max_rpm = 10000.0;
    controller.base.transmission.shift_down_rpm = 2000.0;
    controller.base.transmission.shift_up_rpm = 8000.0;
    controller.base.transmission.gear_ratios.clearRetainingCapacity();
    try controller.base.transmission.gear_ratios.appendSlice(allocator, &.{ 2.27, 1.63, 1.3, 1.09, 0.96, 0.88 });
    controller.base.transmission.reverse_gear_ratios.clearRetainingCapacity();
    try controller.base.transmission.reverse_gear_ratios.appendSlice(allocator, &.{-4.0});
    controller.base.transmission.clutch_strength = 2.0;
    try controller.base.differentials.append(allocator, .{ .left_wheel = -1, .right_wheel = 1, .differential_ratio = 1.93 * 40.0 / 16.0 });
    controller.lean_spring_integration_coefficient = integration_coefficient;
    return vehicle;
}

/// The body of the test motorcycle: a box with its center of mass lowered by its half height, 240 kg
fn addMotorcycleBody(world: *TestWorld, allocator: Allocator, position: RVec3, rotation: Quat) !*Body {
    const box = try BoxShape.create(allocator, Vec3.init(0.2, 0.3, 0.4), .{});
    var box_ref = RefConst(Shape).init(box.asShape());
    defer box_ref.deinit();
    const shape = try OffsetCenterOfMassShape.create(allocator, box.asShape(), Vec3.init(0, -0.3, 0));
    return world.addBody(shape.asShape(), position, rotation, .dynamic, 240.0);
}

/// The test motorcycle in the world (on the floor in the first update at y = 0.9 without rotation), with a cast sphere
/// collision tester
fn addMotorcycle(world: *TestWorld, allocator: Allocator, position: RVec3, rotation: Quat, integration_coefficient: f32) !*VehicleConstraint {
    const body = try addMotorcycleBody(world, allocator, position, rotation);
    var settings = try makeMotorcycleSettings(allocator, integration_coefficient);
    defer settings.asSettings().deinit();
    const constraint = try VehicleConstraint.create(allocator, body, &settings);
    var constraint_ref = Ref(Constraint).init(constraint.asConstraint());
    defer constraint_ref.deinit();
    var tester = RefConst(VehicleCollisionTester).init(&(try VehicleCollisionTesterCastSphere.create(allocator, TestLayers.moving, 0.025, .{})).base);
    defer tester.deinit();
    constraint.setVehicleCollisionTester(tester.ptr);
    try world.system.addConstraint(constraint.asConstraint());
    try world.system.addStepListener(constraint.stepListener());
    return constraint;
}

fn motorcycleController(constraint: *VehicleConstraint) *MotorcycleController {
    return virtual.downcast(MotorcycleController, constraint.getControllerMut());
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

/// The roll angle of the motorcycle (positive: the up axis of the body leans to the left, +x)
fn rollAngle(constraint: *const VehicleConstraint) f32 {
    const body = constraint.getVehicleBody();
    const up = body.getRotation().mulVec3(constraint.getLocalUp());
    return Trigonometry.asin(up.getX());
}

test "MotorcycleControllerSettings defaults, RTTI hash and binary state" {
    const allocator = testing.allocator;

    const settings = try MotorcycleControllerSettings.create(allocator);
    var ref = Ref(VehicleControllerSettings).init(settings.asControllerSettings());
    defer ref.deinit();
    try testing.expectEqual(math.degreesToRadians(45.0), settings.max_lean_angle);
    try testing.expectEqual(@as(f32, 5000.0), settings.lean_spring_constant);
    try testing.expectEqual(@as(f32, 1000.0), settings.lean_spring_damping);
    try testing.expectEqual(@as(f32, 0.0), settings.lean_spring_integration_coefficient);
    try testing.expectEqual(@as(f32, 4.0), settings.lean_spring_integration_coefficient_decay);
    try testing.expectEqual(@as(f32, 0.8), settings.lean_smoothing_factor);
    try testing.expectEqual(@as(usize, 5), settings.base.transmission.gear_ratios.items.len); // The wheeled defaults
    try testing.expectEqual(@as(f32, 1.4), settings.base.differential_limited_slip_ratio);
    try testing.expectEqual(VehicleControllerSettings.rttiHash("MotorcycleControllerSettings"), settings.base.base.getRTTIHash());
    try testing.expect(settings.asControllerSettingsConst() == &settings.base.base);

    // The binary state (through the base class): the wheeled settings, then the six lean members
    var wheeled = try WheeledVehicleControllerSettings.create(allocator);
    var wheeled_ref = Ref(VehicleControllerSettings).init(&wheeled.base);
    defer wheeled_ref.deinit();
    try settings.base.differentials.append(allocator, .{ .left_wheel = -1, .right_wheel = 1 });
    try wheeled.differentials.append(allocator, .{ .left_wheel = -1, .right_wheel = 1 });
    settings.max_lean_angle = 0.5;
    settings.lean_spring_damping = 1200.0;
    settings.lean_spring_integration_coefficient = 3.0;
    settings.lean_smoothing_factor = 0.25;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    testSave(settings.asControllerSettingsConst(), &out);
    var wheeled_out: std.Io.Writer.Allocating = .init(allocator);
    defer wheeled_out.deinit();
    testSave(&wheeled.base, &wheeled_out);
    try testing.expectEqual(wheeled_out.written().len + 6 * 4, out.written().len);
    try testing.expectEqualSlices(u8, wheeled_out.written(), out.written()[0..wheeled_out.written().len]);
    try testing.expectEqualSlices(u8, std.mem.asBytes(&[6]f32{ 0.5, 5000.0, 1200.0, 3.0, 4.0, 0.25 }), out.written()[wheeled_out.written().len..]);

    const restored = try MotorcycleControllerSettings.create(allocator);
    var restored_ref = Ref(VehicleControllerSettings).init(restored.asControllerSettings());
    defer restored_ref.deinit();
    try testRestore(restored.asControllerSettings(), out.written());
    try testing.expectEqual(@as(f32, 0.5), restored.max_lean_angle);
    try testing.expectEqual(@as(f32, 1200.0), restored.lean_spring_damping);
    try testing.expectEqual(@as(f32, 3.0), restored.lean_spring_integration_coefficient);
    try testing.expectEqual(@as(f32, 0.25), restored.lean_smoothing_factor);
    try testing.expectEqual(@as(usize, 1), restored.base.differentials.items.len);

    // An embedded MotorcycleControllerSettings
    var embedded = try MotorcycleControllerSettings.init(allocator);
    embedded.base.base.setEmbedded();
    defer embedded.base.base.deinit();

    // Out of memory: creation and restore leave nothing behind
    const S = struct {
        fn run(a: Allocator, bytes: []const u8) !void {
            const s = try MotorcycleControllerSettings.create(a);
            var s_ref = Ref(VehicleControllerSettings).init(s.asControllerSettings());
            defer s_ref.deinit();
            try testRestore(s.asControllerSettings(), bytes);
        }
    };
    try testing.checkAllAllocationFailures(allocator, S.run, .{out.written()});
}

test "MotorcycleControllerSettings restored by VehicleConstraintSettings from its RTTI hash" {
    const allocator = testing.allocator;

    var settings = VehicleConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    const controller = try MotorcycleControllerSettings.create(allocator);
    settings.setController(controller.asControllerSettings());
    controller.lean_spring_constant = 4321.0;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    testSave(settings.asSettingsConst(), &out);

    var reader: std.Io.Reader = .fixed(out.written());
    var in: StreamWrapper.StreamInWrapper = .init(&reader);
    var result = try ConstraintSettings.restoreFromBinaryState(allocator, in.streamIn());
    defer result.deinit();
    try testing.expect(!result.hasError());
    const restored = virtual.downcast(VehicleConstraintSettings, result.get().get().?).controller.ptr.?;
    try testing.expectEqualStrings("MotorcycleControllerSettings", restored.vtable.rtti_name);
    try testing.expectEqual(@as(f32, 4321.0), virtual.downcast(MotorcycleControllerSettings, restored).lean_spring_constant);
}

test "MotorcycleController GetWheelBase" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    const constraint = try addMotorcycle(&world, allocator, RVec3.init(0, 0.9, 0), Quat.identity(), 0.0);
    const c = motorcycleController(constraint);

    // The front wheel uses its suspension force point, the back wheel its fully extended suspension
    const front = constraint.getWheel(0).getSettings();
    const back = constraint.getWheel(1).getSettings();
    try testing.expectEqual(front.suspension_force_point.getZ() - back.position.add(back.suspension_direction.mulScalar(back.suspension_max_length)).getZ(), c.getWheelBase());

    // Without the force point (a second vehicle on the same body, not added to the system)
    var settings = try makeMotorcycleSettings(allocator, 0.0);
    defer settings.asSettings().deinit();
    settings.wheels.items[0].ptr.?.enable_suspension_force_point = false;
    const other = try VehicleConstraint.create(allocator, constraint.getVehicleBody(), &settings);
    var other_ref = Ref(Constraint).init(other.asConstraint());
    defer other_ref.deinit();
    try testing.expectEqual(front.position.add(front.suspension_direction.mulScalar(front.suspension_max_length)).getZ() - back.position.add(back.suspension_direction.mulScalar(back.suspension_max_length)).getZ(), motorcycleController(other).getWheelBase());
}

test "MotorcycleController drives, leans into turns, limits the steering and brakes" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    const constraint = try addMotorcycle(&world, allocator, RVec3.init(0, 0.9, 0), Quat.identity(), 0.0);
    const c = motorcycleController(constraint);
    const body = constraint.getVehicleBody();

    // The configuration of the settings
    try testing.expect(c.isLeanControllerEnabled());
    try testing.expect(c.isLeanSteeringLimitEnabled());
    try testing.expectEqual(math.degreesToRadians(45.0), c.max_lean_angle);
    try testing.expect(c.asControllerConst() == constraint.getController());
    try testing.expect(c.target_lean.eql(Vec3.zero()));

    // The lean spring keeps the motorcycle upright while it stands and drives straight
    try world.simulate(1.0);
    for (constraint.getWheels()) |w|
        try testing.expect(w.hasContact());
    try testing.expect(@abs(rollAngle(constraint)) < 0.02);
    try testing.expect(c.target_lean.getY() > 0.99); // Normalized, along the impulse of the ground
    c.base.setDriverInput(1.0, 0.0, 0.0, 0.0);
    world.system.getBodyInterfaceMut().activateBody(body.getID());
    try world.simulate(3.0);
    try testing.expect(body.getPosition().getZ() > 5.0);
    try testing.expect(@abs(rollAngle(constraint)) < 0.02);
    try testing.expect(body.getLinearVelocity().getZ() > 4.0);

    // Steering right with the lean steering limit: the front wheel steers less than its max angle at this speed
    c.base.setRightInput(1.0);
    try world.simulate(1.0 / 60.0);
    const max_steer = constraint.getWheel(0).getSettings().cast(WheelSettingsWV).max_steer_angle;
    try testing.expect(constraint.getWheel(0).getSteerAngle() < 0.0);
    try testing.expect(-constraint.getWheel(0).getSteerAngle() < max_steer);
    try testing.expectEqual(@as(f32, 0.0), constraint.getWheel(1).getSteerAngle()); // The back wheel cannot steer

    // Without the limit it steers the full angle
    c.enableLeanSteeringLimit(false);
    try testing.expect(!c.isLeanSteeringLimitEnabled());
    try world.simulate(1.0 / 60.0);
    try testing.expectEqual(-max_steer, constraint.getWheel(0).getSteerAngle());
    c.enableLeanSteeringLimit(true);

    // It leans into the right turn (the up axis of the body tilts to the right, -x) and turns right
    c.base.setRightInput(0.3);
    try world.simulate(1.5);
    try testing.expect(rollAngle(constraint) < -0.05);
    try testing.expect(body.getAngularVelocity().getY() < -0.05);
    try testing.expect(c.target_lean.getX() < 0.0);
    try testing.expect(c.applied_impulse != 0.0);

    // Without the lean controller the target lean is the world up vector and the integrated angle is reset
    c.enableLeanController(false);
    try testing.expect(!c.isLeanControllerEnabled());
    try world.simulate(1.0 / 60.0);
    try testing.expect(c.target_lean.eql(constraint.getWorldUp()));
    try testing.expectEqual(@as(f32, 0.0), c.lean_spring_integrated_delta_angle);
    c.enableLeanController(true);

    // Brake to a stop
    c.base.setDriverInput(0.0, 0.0, 1.0, 0.0);
    try world.simulate(8.0);
    try testing.expect(body.getLinearVelocity().length() < 0.1);
    try testing.expect(@abs(rollAngle(constraint)) < 0.15); // It stays on its wheels
}

test "MotorcycleController lean spring integration" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();

    // In the air rolled to the side: PreCollide integrates the angle to the target lean, SolveLongitudinalAndLateral-
    // Constraints decays it (the wheels do not touch the floor)
    const constraint = try addMotorcycle(&world, allocator, RVec3.init(0, 1.2, 0), Quat.rotation(Vec3.axisZ(), 0.15), 1000.0);
    const c = motorcycleController(constraint);
    try testing.expectEqual(@as(f32, 1000.0), c.getLeanSpringIntegrationCoefficient());
    try world.simulate(1.0 / 60.0);
    for (constraint.getWheels()) |w|
        try testing.expect(!w.hasContact());
    const in_air = c.lean_spring_integrated_delta_angle;
    try testing.expect(in_air != 0.0);
    try world.simulate(1.0 / 60.0);
    try testing.expect(math.sign(c.lean_spring_integrated_delta_angle) == math.sign(in_air)); // Integrated further

    // A faster decay
    c.setLeanSpringIntegrationCoefficientDecay(1000.0);
    try testing.expectEqual(@as(f32, 1000.0), c.getLeanSpringIntegrationCoefficientDecay());
    try world.simulate(1.0 / 60.0);
    try testing.expectEqual(@as(f32, 0.0), c.lean_spring_integrated_delta_angle); // max(0, 1 - decay * dt) = 0

    // On the floor the integrated angle pushes the motorcycle upright too
    c.setLeanSpringIntegrationCoefficientDecay(4.0);
    try world.simulate(2.0);
    for (constraint.getWheels()) |w|
        try testing.expect(w.hasContact());
    try testing.expect(@abs(rollAngle(constraint)) < 0.05);
}

test "MotorcycleController getters, setters, settings and state" {
    const allocator = testing.allocator;
    var world: TestWorld = undefined;
    try world.init(allocator);
    defer world.deinit();
    const constraint = try addMotorcycle(&world, allocator, RVec3.init(0, 0.9, 0), Quat.identity(), 0.0);
    const c = motorcycleController(constraint);

    c.setLeanSpringConstant(6000.0);
    c.setLeanSpringDamping(1200.0);
    c.setLeanSpringIntegrationCoefficient(2.0);
    c.setLeanSpringIntegrationCoefficientDecay(3.0);
    c.setLeanSmoothingFactor(0.7);
    try testing.expectEqual(@as(f32, 6000.0), c.getLeanSpringConstant());
    try testing.expectEqual(@as(f32, 1200.0), c.getLeanSpringDamping());
    try testing.expectEqual(@as(f32, 2.0), c.getLeanSpringIntegrationCoefficient());
    try testing.expectEqual(@as(f32, 3.0), c.getLeanSpringIntegrationCoefficientDecay());
    try testing.expectEqual(@as(f32, 0.7), c.getLeanSmoothingFactor());

    // GetSettings: the wheeled settings and the lean members, but the default lean spring damping (Jolt assigns the
    // settings' own value)
    c.base.setDifferentialLimitedSlipRatio(1.7);
    var settings = try constraint.getController().getSettings();
    defer settings.deinit();
    try testing.expectEqualStrings("MotorcycleControllerSettings", settings.get().?.vtable.rtti_name);
    const motorcycle = virtual.downcast(MotorcycleControllerSettings, settings.get().?);
    try testing.expectEqual(@as(f32, 6000.0), motorcycle.lean_spring_constant);
    try testing.expectEqual(@as(f32, 1000.0), motorcycle.lean_spring_damping);
    try testing.expectEqual(@as(f32, 2.0), motorcycle.lean_spring_integration_coefficient);
    try testing.expectEqual(@as(f32, 3.0), motorcycle.lean_spring_integration_coefficient_decay);
    try testing.expectEqual(@as(f32, 0.7), motorcycle.lean_smoothing_factor);
    try testing.expectEqual(c.max_lean_angle, motorcycle.max_lean_angle);
    try testing.expectEqual(@as(f32, 1.7), motorcycle.base.differential_limited_slip_ratio);
    try testing.expectEqual(@as(usize, 6), motorcycle.base.transmission.gear_ratios.items.len);

    // SaveState / RestoreState: the wheeled state and the target lean, not the integrated angle
    try world.simulate(0.5);
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    constraint.getController().saveState(recorder.recorder());
    try testing.expectEqual(@as(usize, 5 * 4 + 4 + 20 + 12), recorder.getData().len);
    const target_lean = c.target_lean;
    const integrated = c.lean_spring_integrated_delta_angle;
    c.target_lean = Vec3.axisX();
    c.lean_spring_integrated_delta_angle = integrated + 1.0;
    c.base.setForwardInput(0.5);
    recorder.rewind();
    constraint.getControllerMut().restoreState(recorder.recorder());
    try testing.expect(c.target_lean.eql(target_lean));
    try testing.expectEqual(integrated + 1.0, c.lean_spring_integrated_delta_angle);
    try testing.expectEqual(@as(f32, 0.0), c.base.getForwardInput());
}

test "MotorcycleController creation, settings and controller settings out of memory" {
    var world: TestWorld = undefined;
    try world.init(testing.allocator);
    defer world.deinit();
    const body = try addMotorcycleBody(&world, testing.allocator, RVec3.init(0, 0.9, 0), Quat.identity());

    const S = struct {
        fn run(allocator: Allocator, motorcycle_body: *Body) !void {
            var settings = try makeMotorcycleSettings(allocator, 0.0);
            defer settings.asSettings().deinit();

            const constraint = try VehicleConstraint.create(allocator, motorcycle_body, &settings);
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
