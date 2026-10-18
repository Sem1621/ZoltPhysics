//! Port of: Jolt/Physics/Vehicle/VehicleConstraint.h, Jolt/Physics/Vehicle/VehicleConstraint.cpp
//! Status: complete
//!
//! - JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(VehicleConstraintSettings):
//!   TODO(serialization) (ObjectStream is Phase 8, CharacterVehicleArchitecture E1; the binary state is ported).
//! - JPH_DEBUG_RENDERER (`DrawConstraint`, `DrawConstraintLimits`): TODO(debug renderer).
//! - CharacterVehicleArchitecture E2 and the porter template of FixedConstraint.zig:
//!   - `VehicleConstraintSettings` embeds `base: ConstraintSettings` (rtti_name, overrides, `create` / `createDefault`
//!     for `RegisterTypes.constraint_settings_types`, E3). Its wheels are `Ref(WheelSettings)`, its controller a
//!     `Ref(VehicleControllerSettings)`; `destruct` releases them.
//!   - `VehicleConstraint` is a pattern A level directly under `Constraint` (Jolt: `Constraint, PhysicsStepListener`): it
//!     adds no virtual functions and lists its Constraint overrides. Jolt's private PhysicsStepListener base is
//!     `stepListener()`, the pattern B `PhysicsStepListener` for `PhysicsSystem.addStepListener` /
//!     `removeStepListener` (the same `ptr` every time, so removing finds it). `onStep` is `pub` for that interface
//!     (private in Jolt).
//!   - The constraint owns its controller and its wheels (`destroy()` in `destruct`, Jolt's `delete`). The controller
//!     keeps a pointer to the constraint, so a VehicleConstraint is built in place: `create(allocator, body, settings)`
//!     (`new VehicleConstraint`, reference count 0) or `init(self, ...)` for an embedded constraint (`setEmbedded`,
//!     `deinit`), never moved afterwards.
//! - Allocation: `create` / `init` return `Allocator.Error` and leave nothing behind on failure (the anti-roll bars, the
//!   controller, every wheel and the scratch buffer are freed again, CharacterVehicleArchitecture E2).
//!   `getConstraintSettings` returns `Allocator.Error` (it creates the settings, their wheel array and the controller
//!   settings). The functions of the physics update cannot report an allocation failure (`onStep` runs in a step
//!   listener job, `buildIslands` in the BuildIslandsFromConstraints job), so:
//!   - BuildIslands' `JPH_STACK_ALLOC((mWheels.size() + 1) * sizeof(BodyID))` is a scratch buffer allocated with the
//!     constraint (`body_ids_scratch`).
//!   - Deviation: `getWheelsMut()` gives the wheels as a slice (Jolt's writable `GetWheels()` returns the array): the
//!     wheels are created from the settings by the controller and their number cannot change after creation, because
//!     `body_ids_scratch` (the number of wheels + 1) is allocated with the constraint. (The number of differentials of a
//!     WheeledVehicleController is a separate matter: its `setDifferentials` changes it, see WheeledVehicleController.zig.)
//! - `GetWheelLocalBasis(wheel, outForward, outUp, outRight)` returns `WheelLocalBasis{ .forward, .up, .right }`,
//!   `CalculateSuspensionForcePoint` a `SuspensionForcePoint{ .r1_plus_u, .r2 }` (out parameters).
//! - `CombineFunction` and `StepCallback` (std::function) are a function pointer + context with `init(context,
//!   function)` and `call` (porting guide, containers table); the step callbacks are optional (`?StepCallback`, Jolt's
//!   empty std::function).
//! - Jolt behavior reproduced: `IsActive` reads `mIsActive`, which OnStep computes and SaveState does not save (a restore
//!   keeps the value of the restoring constraint); OnStep reads the gravity of the physics system unless it is
//!   overridden and changes the gravity factor of the vehicle body while the override is active.
//! - Jolt bug found in the design review (CharacterVehicleArchitecture E6): `VehicleConstraintSettings::RestoreBinaryState`
//!   resizes `mWheels` with null `Ref`s and calls `RestoreBinaryState` on them, so every restore of settings with wheels
//!   dereferences a null pointer (the stream does not say whether a wheel is a WheelSettingsWV or WheelSettingsTV). Zolt
//!   panics there with a message. The same for a controller hash that is not a registered class
//!   (`Factory::sInstance->Find(hash)` returns null, Jolt dereferences it; a class that is still a Zolt stub, `is_stub`,
//!   counts as unknown): Zolt panics with a message. Settings without wheels restore and compare (parity).
//! - Where Jolt dereferences a null pointer that the user did not set (no controller settings, a null wheel settings
//!   Ref, OnStep without a collision tester) Zolt panics with a message.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const HashCombine = @import("../../Core/HashCombine.zig");
const ReferenceFile = @import("../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefConst = ReferenceFile.RefConst;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const Trigonometry = @import("../../Math/Trigonometry.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Quat = @import("../../Math/Quat.zig").Quat;
const RealFile = @import("../../Math/Real.zig");
const RVec3 = RealFile.RVec3;
const RMat44 = RealFile.RMat44;
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const BodyManager = @import("../Body/BodyManager.zig").BodyManager;
const SubShapeID = @import("../Collision/Shape/SubShapeID.zig").SubShapeID;
const IslandBuilder = @import("../IslandBuilder.zig").IslandBuilder;
const LargeIslandSplitter = @import("../LargeIslandSplitter.zig").LargeIslandSplitter;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const PhysicsStepListenerFile = @import("../PhysicsStepListener.zig");
const PhysicsStepListener = PhysicsStepListenerFile.PhysicsStepListener;
const PhysicsStepListenerContext = PhysicsStepListenerFile.PhysicsStepListenerContext;
const ConstraintFile = @import("../Constraints/Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const ConstraintSettings = ConstraintFile.ConstraintSettings;
const ConstraintSubType = ConstraintFile.ConstraintSubType;
const SpringMode = @import("../Constraints/SpringSettings.zig").SpringMode;
const AngleConstraintPart = @import("../Constraints/ConstraintPart/AngleConstraintPart.zig").AngleConstraintPart;
const VehicleCollisionTester = @import("VehicleCollisionTester.zig").VehicleCollisionTester;
const VehicleAntiRollBarFile = @import("VehicleAntiRollBar.zig");
const VehicleAntiRollBar = VehicleAntiRollBarFile.VehicleAntiRollBar;
const VehicleAntiRollBars = VehicleAntiRollBarFile.VehicleAntiRollBars;
const WheelFile = @import("Wheel.zig");
const Wheel = WheelFile.Wheel;
const Wheels = WheelFile.Wheels;
const WheelSettings = WheelFile.WheelSettings;
const VehicleControllerFile = @import("VehicleController.zig");
const VehicleController = VehicleControllerFile.VehicleController;
const VehicleControllerSettings = VehicleControllerFile.VehicleControllerSettings;
const RegisterTypes = @import("../../RegisterTypes.zig");

/// Configuration for constraint that simulates a wheeled vehicle.
///
/// The properties in this constraint are largely based on "Car Physics for Games" by Marco Monster.
/// See: https://www.asawicki.info/Mirror/Car%20Physics%20for%20Games/Car%20Physics%20for%20Games.html
pub const VehicleConstraintSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, VehicleConstraintSettings)

    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "VehicleConstraintSettings";
    pub const overrides = .{ .saveBinaryState, .restoreBinaryState };

    base: ConstraintSettings,

    /// Vector indicating the up direction of the vehicle (in local space to the body)
    up: Vec3 = Vec3.init(0, 1, 0),
    /// Vector indicating forward direction of the vehicle (in local space to the body)
    forward: Vec3 = Vec3.init(0, 0, 1),
    /// Defines the maximum pitch/roll angle (rad), can be used to avoid the car from getting upside down. The vehicle up direction will stay within a cone centered around the up axis with half top angle mMaxPitchRollAngle, set to pi to turn off.
    max_pitch_roll_angle: f32 = math.pi,
    /// List of wheels and their properties (allocated with the settings' allocator, see `addWheel`)
    wheels: std.ArrayList(Ref(WheelSettings)) = .empty,
    /// List of anti rollbars and their properties (allocated with the settings' allocator)
    anti_roll_bars: VehicleAntiRollBars = .empty,
    /// Defines how the vehicle can accelerate / decelerate
    controller: Ref(VehicleControllerSettings) = .empty,

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) VehicleConstraintSettings {
        return .{ .base = .init(VehicleConstraintSettings, allocator) };
    }

    /// new VehicleConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*VehicleConstraintSettings {
        const self = try allocator.create(VehicleConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other` (adds references to its wheels and controller),
    /// not its reference count. The arrays are allocated with `allocator`.
    pub fn initCopy(allocator: Allocator, other: *const VehicleConstraintSettings) Allocator.Error!VehicleConstraintSettings {
        var result: VehicleConstraintSettings = .{
            .base = .initCopy(other.base.vtable, allocator, &other.base),
            .up = other.up,
            .forward = other.forward,
            .max_pitch_roll_angle = other.max_pitch_roll_angle,
        };
        errdefer result.destruct();
        try result.wheels.ensureTotalCapacityPrecise(allocator, other.wheels.items.len);
        for (other.wheels.items) |w|
            result.wheels.appendAssumeCapacity(w.clone());
        result.anti_roll_bars = try other.anti_roll_bars.clone(allocator);
        result.controller = other.controller.clone();
        return result;
    }

    /// ~VehicleConstraintSettings: releases the wheels and the controller, frees the arrays
    pub fn destruct(self: *VehicleConstraintSettings) void {
        const allocator = self.base.allocator;
        for (self.wheels.items) |*w|
            w.deinit();
        self.wheels.deinit(allocator);
        self.anti_roll_bars.deinit(allocator);
        self.controller.deinit();
    }

    pub fn asSettings(self: *VehicleConstraintSettings) *ConstraintSettings {
        return &self.base;
    }

    pub fn asSettingsConst(self: *const VehicleConstraintSettings) *const ConstraintSettings {
        return &self.base;
    }

    /// Zolt helper: append a wheel (`mWheels.push_back(inWheel)`, adds a reference), all or nothing
    pub fn addWheel(self: *VehicleConstraintSettings, wheel: *WheelSettings) Allocator.Error!void {
        try self.wheels.ensureUnusedCapacity(self.base.allocator, 1);
        self.wheels.appendAssumeCapacity(.init(wheel));
    }

    /// Zolt helper: append an anti-roll bar (`mAntiRollBars.push_back(inBar)`)
    pub fn addAntiRollBar(self: *VehicleConstraintSettings, bar: VehicleAntiRollBar) Allocator.Error!void {
        try self.anti_roll_bars.append(self.base.allocator, bar);
    }

    /// Zolt helper: set the controller settings (`mController = inController`, adds a reference)
    pub fn setController(self: *VehicleConstraintSettings, controller: ?*VehicleControllerSettings) void {
        self.controller.set(controller);
    }

    /// Saves the contents of the constraint settings in binary form to inStream.
    pub fn saveBinaryState(self: *const VehicleConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.up);
        stream.write(self.forward);
        stream.write(self.max_pitch_roll_angle);

        const num_anti_rollbars: u32 = @intCast(self.anti_roll_bars.items.len);
        stream.write(num_anti_rollbars);
        for (self.anti_roll_bars.items) |*r|
            r.saveBinaryState(stream);

        const num_wheels: u32 = @intCast(self.wheels.items.len);
        stream.write(num_wheels);
        for (self.wheels.items) |w|
            (w.ptr orelse @panic("VehicleConstraintSettings.saveBinaryState: a wheel settings Ref is null (Jolt dereferences it)")).saveBinaryState(stream);

        const controller = self.controller.ptr orelse @panic("VehicleConstraintSettings.saveBinaryState: no controller settings (Jolt dereferences mController)");
        stream.write(controller.getRTTIHash());
        controller.saveBinaryState(stream);
    }

    /// This function should not be called directly, it is used by sRestoreFromBinaryState (protected in Jolt).
    pub fn restoreBinaryState(self: *VehicleConstraintSettings, stream: StreamIn) Allocator.Error!void {
        const allocator = self.base.allocator;

        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.up);
        stream.read(&self.forward);
        stream.read(&self.max_pitch_roll_angle);

        var num_anti_rollbars: u32 = 0;
        stream.read(&num_anti_rollbars);
        const old_num_anti_rollbars = self.anti_roll_bars.items.len;
        try self.anti_roll_bars.resize(allocator, num_anti_rollbars);
        for (self.anti_roll_bars.items[@min(old_num_anti_rollbars, num_anti_rollbars)..]) |*r|
            r.* = .{}; // Array::resize default constructs the new elements
        for (self.anti_roll_bars.items) |*r|
            r.restoreBinaryState(stream);

        var num_wheels: u32 = 0;
        stream.read(&num_wheels);
        if (num_wheels < self.wheels.items.len) {
            for (self.wheels.items[num_wheels..]) |*w|
                w.deinit();
            self.wheels.shrinkRetainingCapacity(num_wheels);
        } else {
            try self.wheels.appendNTimes(allocator, .empty, num_wheels - self.wheels.items.len); // New Refs are null
        }
        for (self.wheels.items) |w|
            try (w.ptr orelse @panic("VehicleConstraintSettings.restoreBinaryState: settings with wheels cannot be restored (Jolt calls RestoreBinaryState on the null wheel settings it just added, see CharacterVehicleArchitecture E6)")).restoreBinaryState(stream);

        var hash: u32 = 0;
        stream.read(&hash);
        const controller: *VehicleControllerSettings = inline for (RegisterTypes.vehicle_controller_settings_types) |T| {
            if (comptime @hasDecl(T, "is_stub")) continue;
            if (hash == comptime VehicleControllerSettings.rttiHash(T.rtti_name)) break virtual.upcast(VehicleControllerSettings, try T.createDefault(allocator));
        } else @panic("VehicleConstraintSettings.restoreBinaryState: unknown controller settings hash (Jolt dereferences the null RTTI that Factory::Find returns)");
        self.controller.set(controller);
        try controller.restoreBinaryState(stream);
    }
};

/// Constraint that simulates a vehicle
/// Note: Don't forget to register the constraint as a StepListener with the PhysicsSystem!
///
/// When the vehicle drives over very light objects (rubble) you may see the car body dip down. This is a known issue and is an artifact of the iterative solver that Jolt is using.
/// Basically if a light object is sandwiched between two heavy objects (the static floor and the car body), the light object is not able to transfer enough force from the ground to
/// the car body to keep the car body up. You can see this effect in the HeavyOnLightTest sample, the boxes on the right have a lot of penetration because they're on top of light objects.
///
/// There are a couple of ways to improve this:
///
/// 1. You can increase the number of velocity steps (global settings PhysicsSettings::mNumVelocitySteps or if you only want to increase it on
/// the vehicle you can use VehicleConstraintSettings::mNumVelocityStepsOverride). E.g. going from 10 to 30 steps in the HeavyOnLightTest sample makes the penetration a lot less.
/// The number of position steps can also be increased (the first prevents the body from going down, the second corrects it if the problem did
/// occur which inevitably happens due to numerical drift). This solution costs CPU cycles.
///
/// 2. You can reduce the mass difference between the vehicle body and the rubble on the floor (by making the rubble heavier or the car lighter).
///
/// 3. You could filter out collisions between the vehicle collision test and the rubble completely. This would make the wheels ignore the rubble but would cause the vehicle to drive
/// through it as if nothing happened. You could create fake wheels (keyframed bodies) that move along with the vehicle and that only collide with rubble (and not the vehicle or the ground).
/// This would cause the vehicle to push away the rubble without the rubble being able to affect the vehicle (unless it hits the main body of course).
///
/// Note that when driving over rubble, you may see the wheel jump up and down quite quickly because one frame a collision is found and the next frame not.
/// To alleviate this, it may be needed to smooth the motion of the visual mesh for the wheel.
pub const VehicleConstraint = struct {
    pub const overrides = .{ .getSubType, .isActive, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .buildIslands, .buildIslandSplits, .saveState, .restoreState, .getConstraintSettings };

    /// Callback function to combine the friction of a tire with the friction of the body it is colliding with.
    /// On input ioLongitudinalFriction and ioLateralFriction contain the friction of the tire, on output they should contain the combined friction with inBody2.
    /// (std::function: a function pointer + context)
    pub const CombineFunction = struct {
        /// The context that is passed to `function`
        context: ?*anyopaque = null,
        /// The function
        function: *const fn (context: ?*anyopaque, wheel_index: u32, longitudinal_friction: *f32, lateral_friction: *f32, body2: *const Body, sub_shape_id2: SubShapeID) void,

        /// Jolt's default: the geometric mean, sqrt(friction1 * friction2)
        pub const default: CombineFunction = .{ .function = defaultCombine };

        fn defaultCombine(context: ?*anyopaque, wheel_index: u32, longitudinal_friction: *f32, lateral_friction: *f32, body2: *const Body, sub_shape_id2: SubShapeID) void {
            _ = .{ context, wheel_index, sub_shape_id2 };
            const body_friction = body2.getFriction();

            longitudinal_friction.* = @sqrt(longitudinal_friction.* * body_friction);
            lateral_friction.* = @sqrt(lateral_friction.* * body_friction);
        }

        /// `function(context, wheel_index, longitudinal_friction, lateral_friction, body2, sub_shape_id2)`, `context` is a
        /// pointer (it may be const)
        pub fn init(context: anytype, comptime function: fn (@TypeOf(context), u32, *f32, *f32, *const Body, SubShapeID) void) CombineFunction {
            const Context = @TypeOf(context);
            const gen = struct {
                fn call(ptr: ?*anyopaque, wheel_index: u32, longitudinal_friction: *f32, lateral_friction: *f32, body2: *const Body, sub_shape_id2: SubShapeID) void {
                    const typed: Context = @ptrCast(@alignCast(ptr.?));
                    function(typed, wheel_index, longitudinal_friction, lateral_friction, body2, sub_shape_id2);
                }
            };
            return .{ .context = @ptrCast(@constCast(context)), .function = gen.call };
        }

        /// Call the function
        pub fn call(self: CombineFunction, wheel_index: u32, longitudinal_friction: *f32, lateral_friction: *f32, body2: *const Body, sub_shape_id2: SubShapeID) void {
            self.function(self.context, wheel_index, longitudinal_friction, lateral_friction, body2, sub_shape_id2);
        }
    };

    /// Callback function to notify of current stage in PhysicsStepListener::OnStep.
    /// (std::function: a function pointer + context)
    pub const StepCallback = struct {
        /// The context that is passed to `function`
        context: ?*anyopaque = null,
        /// The function
        function: *const fn (context: ?*anyopaque, vehicle: *VehicleConstraint, listener_context: *const PhysicsStepListenerContext) void,

        /// `function(context, vehicle, listener_context)`, `context` is a pointer (it may be const)
        pub fn init(context: anytype, comptime function: fn (@TypeOf(context), *VehicleConstraint, *const PhysicsStepListenerContext) void) StepCallback {
            const Context = @TypeOf(context);
            const gen = struct {
                fn call(ptr: ?*anyopaque, vehicle: *VehicleConstraint, listener_context: *const PhysicsStepListenerContext) void {
                    const typed: Context = @ptrCast(@alignCast(ptr.?));
                    function(typed, vehicle, listener_context);
                }
            };
            return .{ .context = @ptrCast(@constCast(context)), .function = gen.call };
        }

        /// Call the function
        pub fn call(self: StepCallback, vehicle: *VehicleConstraint, listener_context: *const PhysicsStepListenerContext) void {
            self.function(self.context, vehicle, listener_context);
        }
    };

    /// The basis vectors of a wheel (the out parameters of GetWheelLocalBasis)
    pub const WheelLocalBasis = struct {
        /// Forward vector for the wheel
        forward: Vec3,
        /// Up vector for the wheel
        up: Vec3,
        /// Right vector for the wheel
        right: Vec3,
    };

    /// The out parameters of CalculateSuspensionForcePoint
    const SuspensionForcePoint = struct {
        r1_plus_u: Vec3,
        r2: Vec3,
    };

    base: Constraint,

    // Gravity override

    /// If the gravity is currently overridden
    is_gravity_overridden: bool = false,
    /// Gravity override value, replaces PhysicsSystem::GetGravity() when mIsGravityOverridden is true
    gravity_override: Vec3 = Vec3.zero(),

    // Simulation information

    /// Body of the vehicle
    body: *Body,
    /// Local space forward vector for the vehicle
    forward: Vec3,
    /// Local space up vector for the vehicle
    up: Vec3,
    /// Vector indicating the world space up direction (used to limit vehicle pitch/roll)
    world_up: Vec3,
    /// Wheel states of the vehicle
    wheels: Wheels = .empty,
    /// Anti rollbars of the vehicle
    anti_roll_bars: VehicleAntiRollBars = .empty,
    /// Controls the acceleration / deceleration of the vehicle
    controller: *VehicleController = undefined,
    /// If this constraint is active
    is_active: bool = false,
    /// Number of simulation steps between wheel collision tests when the vehicle is active
    num_steps_between_collision_test_active: u32 = 1,
    /// Number of simulation steps between wheel collision tests when the vehicle is inactive
    num_steps_between_collision_test_inactive: u32 = 1,
    /// Current step number, used to determine when to test a wheel
    current_step: u32 = 0,

    // Prevent vehicle from toppling over

    /// Cos of the max pitch/roll angle
    cos_max_pitch_roll_angle: f32 = undefined,
    /// Cos of the current pitch/roll angle (written by calculatePitchRollConstraintProperties before it is read)
    cos_pitch_roll_angle: f32 = undefined,
    /// Current axis along which to apply torque to prevent the car from toppling over
    pitch_roll_rotation_axis: Vec3 = Vec3.init(0, 1, 0),
    /// Constraint part that prevents the car from toppling over
    pitch_roll_part: AngleConstraintPart = .{},

    // Interfaces

    /// Class that performs testing of collision for the wheels
    vehicle_collision_tester: RefConst(VehicleCollisionTester) = .empty,
    combine_friction: CombineFunction = .default,

    // Callbacks
    pre_step_callback: ?StepCallback = null,
    post_collide_callback: ?StepCallback = null,
    post_step_callback: ?StepCallback = null,

    /// Zolt: BuildIslands' JPH_STACK_ALLOC of mWheels.size() + 1 body IDs, allocated with the constraint (see the file
    /// comment)
    body_ids_scratch: []BodyID = &.{},

    /// Constructor (in place: the controller keeps a pointer to the constraint). On failure nothing is left allocated.
    pub fn init(self: *VehicleConstraint, allocator: Allocator, vehicle_body: *Body, settings: *const VehicleConstraintSettings) Allocator.Error!void {
        self.* = .{
            .base = .init(VehicleConstraint, allocator, &settings.base),
            .body = vehicle_body,
            .forward = settings.forward,
            .up = settings.up,
            .world_up = settings.up,
        };
        self.anti_roll_bars = try settings.anti_roll_bars.clone(allocator);
        errdefer self.anti_roll_bars.deinit(allocator);

        // Check sanity of incoming settings
        if (Core.enable_asserts) {
            std.debug.assert(settings.up.isNormalized(.{}));
            std.debug.assert(settings.forward.isNormalized(.{}));
            std.debug.assert(settings.wheels.items.len != 0);
        }

        // Store max pitch/roll angle
        self.setMaxPitchRollAngle(settings.max_pitch_roll_angle);

        // Construct our controller class
        const controller_settings = settings.controller.ptr orelse @panic("VehicleConstraint: the settings have no controller (Jolt dereferences mController)");
        self.controller = try controller_settings.constructController(allocator, self);
        errdefer self.controller.destroy();

        // Create wheels
        try self.wheels.ensureTotalCapacityPrecise(allocator, settings.wheels.items.len);
        errdefer {
            for (self.wheels.items) |w|
                w.destroy();
            self.wheels.deinit(allocator);
        }
        for (settings.wheels.items) |w| {
            const wheel_settings = w.ptr orelse @panic("VehicleConstraint: a wheel settings Ref is null (Jolt dereferences it)");
            self.wheels.appendAssumeCapacity(try self.controller.constructWheel(allocator, wheel_settings));
        }

        // Zolt: the body IDs of BuildIslands
        self.body_ids_scratch = try allocator.alloc(BodyID, self.wheels.items.len + 1);

        // Use the body ID as a seed for the step counter so that not all vehicles will update at the same time
        self.current_step = @truncate(HashCombine.hash64(vehicle_body.getID().getIndex()));
    }

    /// new VehicleConstraint(inVehicleBody, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, vehicle_body: *Body, settings: *const VehicleConstraintSettings) Allocator.Error!*VehicleConstraint {
        const self = try allocator.create(VehicleConstraint);
        errdefer allocator.destroy(self);
        try self.init(allocator, vehicle_body, settings);
        return self;
    }

    /// ~VehicleConstraint
    pub fn destruct(self: *VehicleConstraint) void {
        const allocator = self.base.allocator;

        // Destroy controller
        self.controller.destroy();

        // Destroy our wheels
        for (self.wheels.items) |w|
            w.destroy();
        self.wheels.deinit(allocator);

        self.anti_roll_bars.deinit(allocator);
        self.vehicle_collision_tester.deinit();
        allocator.free(self.body_ids_scratch);
    }

    pub fn asConstraint(self: *VehicleConstraint) *Constraint {
        return &self.base;
    }

    pub fn asConstraintConst(self: *const VehicleConstraint) *const Constraint {
        return &self.base;
    }

    /// The PhysicsStepListener of this constraint (Jolt's second base class): register it with
    /// PhysicsSystem.addStepListener, remove it with removeStepListener (it is the same listener every time)
    pub fn stepListener(self: *VehicleConstraint) PhysicsStepListener {
        return .init(self);
    }

    /// Get the type of a constraint
    pub fn getSubType(self: *const VehicleConstraint) ConstraintSubType {
        _ = self;
        return .vehicle;
    }

    /// Defines the maximum pitch/roll angle (rad), can be used to avoid the car from getting upside down. The vehicle up direction will stay within a cone centered around the up axis with half top angle mMaxPitchRollAngle, set to pi to turn off.
    pub fn setMaxPitchRollAngle(self: *VehicleConstraint, max_pitch_roll_angle: f32) void {
        self.cos_max_pitch_roll_angle = Trigonometry.cos(max_pitch_roll_angle);
    }

    pub fn getMaxPitchRollAngle(self: *const VehicleConstraint) f32 {
        return Trigonometry.acos(self.cos_max_pitch_roll_angle);
    }

    /// Set the interface that tests collision between wheel and ground
    pub fn setVehicleCollisionTester(self: *VehicleConstraint, tester: ?*const VehicleCollisionTester) void {
        self.vehicle_collision_tester.set(tester);
    }

    pub fn getVehicleCollisionTester(self: *const VehicleConstraint) ?*const VehicleCollisionTester {
        return self.vehicle_collision_tester.ptr;
    }

    /// Set the function that combines the friction of two bodies and returns it
    /// Default method is the geometric mean: sqrt(friction1 * friction2).
    pub fn setCombineFriction(self: *VehicleConstraint, combine_friction: CombineFunction) void {
        self.combine_friction = combine_friction;
    }

    pub fn getCombineFriction(self: *const VehicleConstraint) CombineFunction {
        return self.combine_friction;
    }

    /// Callback function to notify that PhysicsStepListener::OnStep has started for this vehicle. Default is to do nothing.
    /// Can be used to allow higher-level code to e.g. control steering. This is the last moment that the position/orientation of the vehicle can be changed.
    /// Wheel collision checks have not been performed yet.
    pub fn getPreStepCallback(self: *const VehicleConstraint) ?StepCallback {
        return self.pre_step_callback;
    }

    pub fn setPreStepCallback(self: *VehicleConstraint, callback: ?StepCallback) void {
        self.pre_step_callback = callback;
    }

    /// Callback function to notify that PhysicsStepListener::OnStep has just completed wheel collision checks. Default is to do nothing.
    /// Can be used to allow higher-level code to e.g. detect tire contact or to modify the velocity of the vehicle based on the wheel contacts.
    /// You should not change the position of the vehicle in this callback as the wheel collision checks have already been performed.
    pub fn getPostCollideCallback(self: *const VehicleConstraint) ?StepCallback {
        return self.post_collide_callback;
    }

    pub fn setPostCollideCallback(self: *VehicleConstraint, callback: ?StepCallback) void {
        self.post_collide_callback = callback;
    }

    /// Callback function to notify that PhysicsStepListener::OnStep has completed for this vehicle. Default is to do nothing.
    /// Can be used to allow higher-level code to e.g. control the vehicle in the air.
    /// You should not change the position of the vehicle in this callback as the wheel collision checks have already been performed.
    pub fn getPostStepCallback(self: *const VehicleConstraint) ?StepCallback {
        return self.post_step_callback;
    }

    pub fn setPostStepCallback(self: *VehicleConstraint, callback: ?StepCallback) void {
        self.post_step_callback = callback;
    }

    /// Override gravity for this vehicle. Note that overriding gravity will set the gravity factor of the vehicle body to 0 and apply gravity in the PhysicsStepListener instead.
    pub fn overrideGravity(self: *VehicleConstraint, gravity: Vec3) void {
        self.gravity_override = gravity;
        self.is_gravity_overridden = true;
    }

    pub fn isGravityOverridden(self: *const VehicleConstraint) bool {
        return self.is_gravity_overridden;
    }

    pub fn getGravityOverride(self: *const VehicleConstraint) Vec3 {
        return self.gravity_override;
    }

    /// Note that resetting the gravity override will restore the gravity factor of the vehicle body to 1.
    pub fn resetGravityOverride(self: *VehicleConstraint) void {
        self.is_gravity_overridden = false;
        self.body.getMotionPropertiesMut().setGravityFactor(1.0);
    }

    /// Get the local space forward vector of the vehicle
    pub fn getLocalForward(self: *const VehicleConstraint) Vec3 {
        return self.forward;
    }

    /// Get the local space up vector of the vehicle
    pub fn getLocalUp(self: *const VehicleConstraint) Vec3 {
        return self.up;
    }

    /// Vector indicating the world space up direction (used to limit vehicle pitch/roll), calculated every frame by inverting gravity
    pub fn getWorldUp(self: *const VehicleConstraint) Vec3 {
        return self.world_up;
    }

    /// Access to the vehicle body
    pub fn getVehicleBody(self: *const VehicleConstraint) *Body {
        return self.body;
    }

    /// Access to the vehicle controller interface (determines acceleration / deceleration)
    pub fn getController(self: *const VehicleConstraint) *const VehicleController {
        return self.controller;
    }

    /// Access to the vehicle controller interface (determines acceleration / deceleration)
    pub fn getControllerMut(self: *VehicleConstraint) *VehicleController {
        return self.controller;
    }

    /// Get the state of the wheels
    pub fn getWheels(self: *const VehicleConstraint) []const *Wheel {
        return self.wheels.items;
    }

    /// Get the state of a wheels (writable interface, allows you to make changes to the configuration which will take effect the next time step)
    /// (Zolt: the wheels, not the array: their number is fixed, see the file comment)
    pub fn getWheelsMut(self: *VehicleConstraint) []*Wheel {
        return self.wheels.items;
    }

    /// Get the state of a wheel
    pub fn getWheel(self: *const VehicleConstraint, index: u32) *const Wheel {
        return self.wheels.items[index];
    }

    pub fn getWheelMut(self: *VehicleConstraint, index: u32) *Wheel {
        return self.wheels.items[index];
    }

    /// Get the basis vectors for the wheel in local space to the vehicle body (note: basis does not rotate when the wheel rotates around its axis)
    /// @param inWheel Wheel to fetch basis for
    /// @return the forward, up and right vectors for the wheel
    pub fn getWheelLocalBasis(self: *const VehicleConstraint, wheel: *const Wheel) WheelLocalBasis {
        _ = self;
        const settings = wheel.getSettings();

        const steer_rotation = Quat.rotation(settings.steering_axis, wheel.steer_angle);
        const up = steer_rotation.mulVec3(settings.wheel_up);
        var forward = steer_rotation.mulVec3(settings.wheel_forward);
        const right = forward.cross(up).normalized();
        forward = up.cross(right).normalized();
        return .{ .forward = forward, .up = up, .right = right };
    }

    /// Get the transform of a wheel in local space to the vehicle body, returns a matrix that transforms a cylinder aligned with the Y axis in body space (not COM space)
    /// @param inWheelIndex Index of the wheel to fetch
    /// @param inWheelRight Unit vector that indicates right in model space of the wheel (so if you only have 1 wheel model, you probably want to specify the opposite direction for the left and right wheels)
    /// @param inWheelUp Unit vector that indicates up in model space of the wheel
    pub fn getWheelLocalTransform(self: *const VehicleConstraint, wheel_index: u32, wheel_right: Vec3, wheel_up: Vec3) Mat44 {
        std.debug.assert(wheel_index < self.wheels.items.len);

        const wheel = self.wheels.items[wheel_index];
        const settings = wheel.getSettings();

        // Use the two vectors provided to calculate a matrix that takes us from wheel model space to X = right, Y = up, Z = forward (the space where we will rotate the wheel)
        const wheel_to_rotational = Mat44.init(Vec4.fromVec3W(wheel_right, 0), Vec4.fromVec3W(wheel_up, 0), Vec4.fromVec3W(wheel_up.cross(wheel_right), 0), Vec4.init(0, 0, 0, 1)).transposed();

        // Calculate the matrix that takes us from the rotational space to vehicle local space
        const basis = self.getWheelLocalBasis(wheel);
        const local_wheel_pos = settings.position.add(settings.suspension_direction.mulScalar(wheel.suspension_length));
        const rotational_to_local = Mat44.init(Vec4.fromVec3W(basis.right, 0), Vec4.fromVec3W(basis.up, 0), Vec4.fromVec3W(basis.forward, 0), Vec4.fromVec3W(local_wheel_pos, 1));

        // Calculate transform of rotated wheel
        return rotational_to_local.mul(Mat44.rotationX(wheel.angle)).mul(wheel_to_rotational);
    }

    /// Get the transform of a wheel in world space, returns a matrix that transforms a cylinder aligned with the Y axis in world space
    /// @param inWheelIndex Index of the wheel to fetch
    /// @param inWheelRight Unit vector that indicates right in model space of the wheel (so if you only have 1 wheel model, you probably want to specify the opposite direction for the left and right wheels)
    /// @param inWheelUp Unit vector that indicates up in model space of the wheel
    pub fn getWheelWorldTransform(self: *const VehicleConstraint, wheel_index: u32, wheel_right: Vec3, wheel_up: Vec3) RMat44 {
        return mulRMat44Mat44(self.body.getWorldTransform(), self.getWheelLocalTransform(wheel_index, wheel_right, wheel_up));
    }

    /// Access to the vehicle's anti roll bars
    pub fn getAntiRollBars(self: *const VehicleConstraint) []const VehicleAntiRollBar {
        return self.anti_roll_bars.items;
    }

    /// The anti roll bars (the array itself: elements added must be allocated with the constraint's allocator)
    pub fn getAntiRollBarsMut(self: *VehicleConstraint) *VehicleAntiRollBars {
        return &self.anti_roll_bars;
    }

    /// Number of simulation steps between wheel collision tests when the vehicle is active. Default is 1. 0 = never, 1 = every step, 2 = every other step, etc.
    /// Note that if a vehicle has multiple wheels and the number of steps > 1, the wheels will be tested in a round robin fashion.
    /// If there are multiple vehicles, the tests will be spread out based on the BodyID of the vehicle.
    /// If you set this to test less than every step, you may see simulation artifacts. This setting can be used to reduce the cost of simulating vehicles in the distance.
    pub fn setNumStepsBetweenCollisionTestActive(self: *VehicleConstraint, steps: u32) void {
        self.num_steps_between_collision_test_active = steps;
    }

    pub fn getNumStepsBetweenCollisionTestActive(self: *const VehicleConstraint) u32 {
        return self.num_steps_between_collision_test_active;
    }

    /// Number of simulation steps between wheel collision tests when the vehicle is inactive. Default is 1. 0 = never, 1 = every step, 2 = every other step, etc.
    /// Note that if a vehicle has multiple wheels and the number of steps > 1, the wheels will be tested in a round robin fashion.
    /// If there are multiple vehicles, the tests will be spread out based on the BodyID of the vehicle.
    /// This number can be lower than the number of steps when the vehicle is active as the only purpose of this test is
    /// to allow the vehicle to wake up in response to bodies moving into the wheels but not touching the body of the vehicle.
    pub fn setNumStepsBetweenCollisionTestInactive(self: *VehicleConstraint, steps: u32) void {
        self.num_steps_between_collision_test_inactive = steps;
    }

    pub fn getNumStepsBetweenCollisionTestInactive(self: *const VehicleConstraint) u32 {
        return self.num_steps_between_collision_test_inactive;
    }

    // Generic interface of a constraint

    pub fn isActive(self: *const VehicleConstraint) bool {
        return self.is_active and self.body.isInBroadPhase() and Constraint.impl.isActive(&self.base);
    }

    pub fn notifyShapeChanged(self: *VehicleConstraint, body_id: BodyID, delta_com: Vec3) void {
        // Do nothing
        _ = .{ self, body_id, delta_com };
    }

    /// The collision tester (Jolt dereferences mVehicleCollisionTester without a check)
    fn collisionTester(self: *const VehicleConstraint) *const VehicleCollisionTester {
        return self.vehicle_collision_tester.ptr orelse @panic("VehicleConstraint: no collision tester (call setVehicleCollisionTester, Jolt dereferences null)");
    }

    // See: PhysicsStepListener (private in Jolt, pub for the PhysicsStepListener interface, see stepListener)
    pub fn onStep(self: *VehicleConstraint, context: *const PhysicsStepListenerContext) void {
        // Step only if we're in the broadphase
        if (!self.body.isInBroadPhase())
            return;

        // Callback to higher-level systems. We do it before PreCollide, in case steering changes.
        if (self.pre_step_callback) |callback|
            callback.call(self, context);

        if (self.is_gravity_overridden) {
            // If gravity is overridden, we replace the normal gravity calculations
            if (self.body.isActive()) {
                const mp = self.body.getMotionPropertiesMut();
                mp.setGravityFactor(0.0);
                self.body.addForce(self.gravity_override.divScalar(mp.getInverseMass()));
            }

            // And we calculate the world up using the custom gravity
            self.world_up = self.gravity_override.negate().normalizedOr(self.world_up);
        } else {
            // Calculate new world up vector by inverting gravity
            self.world_up = context.physics_system.getGravity().negate().normalizedOr(self.world_up);
        }

        // Callback on our controller
        self.controller.preCollide(context.delta_time, context.physics_system);

        // Calculate if this constraint is active by checking if our main vehicle body is active or any of the bodies we touch are active
        self.is_active = self.body.isActive();

        // Test how often we need to update the wheels
        const num_steps_between_collisions = if (self.is_active) self.num_steps_between_collision_test_active else self.num_steps_between_collision_test_inactive;

        const body_transform = self.body.getWorldTransform();

        // Test collision for wheels
        for (self.wheels.items, 0..) |w, i| {
            const wheel_index: u32 = @intCast(i);
            const settings = w.getSettings();

            // Calculate suspension origin and direction
            const ws_origin = body_transform.mulVec3(settings.position);
            const ws_direction = body_transform.multiply3x3(settings.suspension_direction);

            // Test if we need to update this wheel
            if (num_steps_between_collisions == 0 or (self.current_step +% wheel_index) % num_steps_between_collisions != 0) {
                // Simplified wheel contact test
                if (!w.contact_body_id.isInvalid()) {
                    // Test if the body is still valid
                    w.contact_body = context.physics_system.getBodyLockInterfaceNoLock().tryGetBody(w.contact_body_id);
                    if (w.contact_body == null) {
                        // It's not, forget the contact
                        w.contact_body_id = .invalid;
                        w.contact_sub_shape_id = .empty;
                        w.suspension_length = settings.suspension_max_length;
                    } else {
                        // Extrapolate the wheel contact properties
                        self.collisionTester().predictContactProperties(context.physics_system, self, wheel_index, ws_origin, ws_direction, self.body.getID(), &w.contact_body, &w.contact_sub_shape_id, &w.contact_position, &w.contact_normal, &w.suspension_length);
                    }
                }
            } else {
                // Full wheel contact test, start by resetting the contact data
                w.contact_body_id = .invalid;
                w.contact_body = null;
                w.contact_sub_shape_id = .empty;
                w.suspension_length = settings.suspension_max_length;

                // Test collision to find the floor
                if (self.collisionTester().collide(context.physics_system, self, wheel_index, ws_origin, ws_direction, self.body.getID(), &w.contact_body, &w.contact_sub_shape_id, &w.contact_position, &w.contact_normal, &w.suspension_length)) {
                    // Store ID (pointer is not valid outside of the simulation step)
                    w.contact_body_id = w.contact_body.?.getID();
                }
            }

            if (w.contact_body) |contact_body| {
                // Store contact velocity, cache this as the contact body may be removed
                w.contact_point_velocity = contact_body.getPointVelocity(w.contact_position);

                // Determine plane constant for axle contact plane
                w.axle_plane_constant = RVec3.fromVec3(w.contact_normal).dot(ws_origin.addVec3(ws_direction.mulScalar(w.suspension_length)));

                // Check if body is active, if so the entire vehicle should be active
                if (contact_body.isActive())
                    self.is_active = true;

                // Determine world space forward using steering angle and body rotation
                const basis = self.getWheelLocalBasis(w);
                const forward = body_transform.multiply3x3(basis.forward);
                const right = body_transform.multiply3x3(basis.right);

                // The longitudinal axis is in the up/forward plane
                w.contact_longitudinal = w.contact_normal.cross(right);

                // Make sure that the longitudinal axis is aligned with the forward axis
                if (w.contact_longitudinal.dot(forward) < 0.0)
                    w.contact_longitudinal = w.contact_longitudinal.negate();

                // Normalize it
                w.contact_longitudinal = w.contact_longitudinal.normalizedOr(w.contact_normal.getNormalizedPerpendicular());

                // The lateral axis is perpendicular to contact normal and longitudinal axis
                w.contact_lateral = w.contact_longitudinal.cross(w.contact_normal).normalized();
            }
        }

        // Callback to higher-level systems. We do it immediately after wheel collision.
        if (self.post_collide_callback) |callback|
            callback.call(self, context);

        // Calculate anti-rollbar impulses
        for (self.anti_roll_bars.items) |r| {
            if (Core.enable_asserts) std.debug.assert(r.stiffness >= 0.0);

            const lw = self.wheels.items[@intCast(r.left_wheel)];
            const rw = self.wheels.items[@intCast(r.right_wheel)];

            if (lw.contact_body != null and rw.contact_body != null) {
                // Calculate the impulse to apply based on the difference in suspension length
                const difference = rw.suspension_length - lw.suspension_length;
                const impulse = difference * r.stiffness * context.delta_time;
                lw.anti_roll_bar_impulse = -impulse;
                rw.anti_roll_bar_impulse = impulse;
            } else {
                // When one of the wheels is not on the ground we don't apply any impulses
                rw.anti_roll_bar_impulse = 0.0;
                lw.anti_roll_bar_impulse = 0.0;
            }
        }

        // Callback on our controller
        self.controller.postCollide(context.delta_time, context.physics_system);

        // Callback to higher-level systems. We do it before the sleep section, in case velocities change.
        if (self.post_step_callback) |callback|
            callback.call(self, context);

        // If the wheels are rotating, we don't want to go to sleep yet
        if (self.body.getAllowSleeping()) {
            var allow_sleep = self.controller.allowSleep();
            if (allow_sleep) {
                for (self.wheels.items) |w| {
                    if (@abs(w.angular_velocity) > math.degreesToRadians(10.0)) {
                        allow_sleep = false;
                        break;
                    }
                }
            }
            if (!allow_sleep)
                self.body.resetSleepTimer();
        }

        // Increment step counter
        self.current_step +%= 1;
    }

    pub fn buildIslands(self: *VehicleConstraint, constraint_index: u32, builder: *IslandBuilder, body_manager: *BodyManager) void {
        // Find dynamic bodies that our wheels are touching
        const body_ids = self.body_ids_scratch; // JPH_STACK_ALLOC((mWheels.size() + 1) * sizeof(BodyID)), allocated with the constraint
        var num_bodies: usize = 0;
        var needs_to_activate = false;
        for (self.wheels.items) |w| {
            if (w.contact_body) |contact_body| {
                // Avoid adding duplicates
                var duplicate = false;
                const id = contact_body.getID();
                for (body_ids[0..num_bodies]) |existing| {
                    if (existing.eql(id)) {
                        duplicate = true;
                        break;
                    }
                }
                if (duplicate)
                    continue;

                if (contact_body.isDynamic()) {
                    body_ids[num_bodies] = id;
                    num_bodies += 1;
                    if (!contact_body.isActive())
                        needs_to_activate = true;
                }
            }
        }

        // Activate bodies, note that if we get here we have already told the system that we're active so that means our main body needs to be active too
        if (!self.body.isActive()) {
            // Our main body is not active, activate it too
            body_ids[num_bodies] = self.body.getID();
            body_manager.activateBodies(body_ids[0 .. num_bodies + 1]);
        } else if (needs_to_activate) {
            // Only activate bodies the wheels are touching
            body_manager.activateBodies(body_ids[0..num_bodies]);
        }

        // Link dynamic bodies into the same island as the vehicle
        for (body_ids[0..num_bodies]) |id| {
            const body = body_manager.getBody(id);
            if (body.isDynamic())
                builder.linkBodies(self.body.getIndexInActiveBodiesInternal(), body.getIndexInActiveBodiesInternal());
        }

        // Link the vehicle constraint to the car body
        builder.linkConstraint(constraint_index, self.body.getIndexInActiveBodiesInternal());
    }

    pub fn buildIslandSplits(self: *const VehicleConstraint, splitter: *LargeIslandSplitter) u32 {
        return splitter.assignToNonParallelSplit(self.body);
    }

    // Calculate the position where the suspension and traction forces should be applied in world space, relative to the center of mass of both bodies
    fn calculateSuspensionForcePoint(self: *const VehicleConstraint, wheel: *const Wheel) SuspensionForcePoint {
        const settings = wheel.getSettings();

        // Determine point to apply force to
        const force_point: RVec3 = if (settings.enable_suspension_force_point) self.body.getWorldTransform().mulVec3(settings.suspension_force_point) else wheel.contact_position;

        // Calculate r1 + u and r2
        return .{
            .r1_plus_u = force_point.sub(self.body.getCenterOfMassPosition()).toVec3(),
            .r2 = force_point.sub(wheel.contact_body.?.getCenterOfMassPosition()).toVec3(),
        };
    }

    // Calculate the constraint properties for mPitchRollPart
    fn calculatePitchRollConstraintProperties(self: *VehicleConstraint, body_transform: RMat44) void {
        // Check if a limit was specified
        if (self.cos_max_pitch_roll_angle > -1.0) {
            // Calculate cos of angle between world up vector and vehicle up vector
            const vehicle_up = body_transform.multiply3x3(self.up);
            self.cos_pitch_roll_angle = self.world_up.dot(vehicle_up);
            if (self.cos_pitch_roll_angle < self.cos_max_pitch_roll_angle) {
                // Calculate rotation axis to rotate vehicle towards up
                const rotation_axis = self.world_up.cross(vehicle_up);
                const len = rotation_axis.length();
                if (len > 0.0)
                    self.pitch_roll_rotation_axis = rotation_axis.divScalar(len);

                self.pitch_roll_part.calculateConstraintProperties(self.body, Body.fixedToWorld(), self.pitch_roll_rotation_axis, .{});
            } else self.pitch_roll_part.deactivate();
        } else self.pitch_roll_part.deactivate();
    }

    pub fn setupVelocityConstraint(self: *VehicleConstraint, delta_time: f32) void {
        const body_transform = self.body.getWorldTransform();

        for (self.wheels.items) |w| {
            if (w.contact_body) |contact_body| {
                const settings = w.getSettings();

                const neg_contact_normal = w.contact_normal.negate();

                const force_point = self.calculateSuspensionForcePoint(w);
                const r1_plus_u = force_point.r1_plus_u;
                const r2 = force_point.r2;

                // Suspension spring
                if (settings.suspension_max_length > settings.suspension_min_length) {
                    var stiffness: f32 = undefined;
                    var damping: f32 = undefined;
                    const mode = settings.suspension_spring.mode;
                    if (mode == SpringMode.frequency_and_damping or mode == SpringMode.mass_normalized_stiffness_and_damping) {
                        // Calculate effective mass based on vehicle configuration (the stiffness of the spring should not be affected by the dynamics of the vehicle): K = 1 / (J M^-1 J^T)
                        // Note that if no suspension force point is supplied we don't know where the force is applied so we assume it is applied at average suspension length
                        const spring_force_point = if (settings.enable_suspension_force_point) settings.suspension_force_point else settings.position.add(settings.suspension_direction.mulScalar(0.5 * (settings.suspension_min_length + settings.suspension_max_length)));
                        const force_point_x_neg_up = spring_force_point.cross(self.up.negate());
                        const mp = self.body.getMotionProperties();
                        const effective_mass = 1.0 / (mp.getInverseMass() + force_point_x_neg_up.dot(mp.getLocalSpaceInverseInertia().multiply3x3(force_point_x_neg_up)));

                        if (mode == SpringMode.frequency_and_damping) {
                            // Convert frequency and damping to stiffness and damping
                            const omega = 2.0 * math.pi * settings.suspension_spring.frequency;
                            stiffness = effective_mass * math.square(omega);
                            damping = 2.0 * effective_mass * settings.suspension_spring.damping * omega;
                        } else {
                            // Multiply only by effective mass
                            stiffness = effective_mass * settings.suspension_spring.getStiffness();
                            damping = effective_mass * settings.suspension_spring.damping;
                        }
                    } else {
                        // In this case we can simply copy the properties
                        stiffness = settings.suspension_spring.getStiffness();
                        damping = settings.suspension_spring.damping;
                    }

                    // Calculate the damping and frequency of the suspension spring given the angle between the suspension direction and the contact normal
                    // If the angle between the suspension direction and the inverse of the contact normal is alpha then the force on the spring relates to the force along the contact normal as:
                    //
                    // Fspring = Fnormal * cos(alpha)
                    //
                    // The spring force is:
                    //
                    // Fspring = -k * x
                    //
                    // where k is the spring constant and x is the displacement of the spring. So we have:
                    //
                    // Fnormal * cos(alpha) = -k * x <=> Fnormal = -k / cos(alpha) * x
                    //
                    // So we can see this as a spring with spring constant:
                    //
                    // k' = k / cos(alpha)
                    //
                    // In the same way the velocity relates like:
                    //
                    // Vspring = Vnormal * cos(alpha)
                    //
                    // Which results in the modified damping constant c:
                    //
                    // c' = c / cos(alpha)
                    //
                    // Note that we clamp 1 / cos(alpha) to the range [0.1, 1] in order not to increase the stiffness / damping by too much.
                    const ws_direction = body_transform.multiply3x3(settings.suspension_direction);
                    const cos_angle = math.max(@as(f32, 0.1), ws_direction.dot(neg_contact_normal));
                    stiffness /= cos_angle;
                    damping /= cos_angle;

                    // Get the value of the constraint equation
                    const c = w.suspension_length - settings.suspension_max_length - settings.suspension_preload_length;

                    w.suspension_part.calculateConstraintPropertiesWithStiffnessAndDamping(delta_time, self.body, r1_plus_u, contact_body, r2, neg_contact_normal, w.anti_roll_bar_impulse, c, stiffness, damping);
                } else w.suspension_part.deactivate();

                // Check if we reached the 'max up' position and if so add a hard velocity constraint that stops any further movement in the normal direction
                if (w.suspension_length < settings.suspension_min_length)
                    w.suspension_max_up_part.calculateConstraintProperties(self.body, r1_plus_u, contact_body, r2, neg_contact_normal, .{})
                else
                    w.suspension_max_up_part.deactivate();

                // Friction and propulsion
                w.longitudinal_part.calculateConstraintProperties(self.body, r1_plus_u, contact_body, r2, w.contact_longitudinal.negate(), .{});
                w.lateral_part.calculateConstraintProperties(self.body, r1_plus_u, contact_body, r2, w.contact_lateral.negate(), .{});
            } else {
                // No contact -> disable everything
                w.suspension_part.deactivate();
                w.suspension_max_up_part.deactivate();
                w.longitudinal_part.deactivate();
                w.lateral_part.deactivate();
            }
        }

        self.calculatePitchRollConstraintProperties(body_transform);
    }

    pub fn resetWarmStart(self: *VehicleConstraint) void {
        for (self.wheels.items) |w| {
            w.suspension_part.deactivate();
            w.suspension_max_up_part.deactivate();
            w.longitudinal_part.deactivate();
            w.lateral_part.deactivate();
        }

        self.pitch_roll_part.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *VehicleConstraint, warm_start_impulse_ratio: f32) void {
        for (self.wheels.items) |w| {
            if (w.contact_body) |contact_body| {
                const neg_contact_normal = w.contact_normal.negate();

                w.suspension_part.warmStart(self.body, contact_body, neg_contact_normal, warm_start_impulse_ratio);
                w.suspension_max_up_part.warmStart(self.body, contact_body, neg_contact_normal, warm_start_impulse_ratio);
                w.longitudinal_part.warmStart(self.body, contact_body, w.contact_longitudinal.negate(), 0.0); // Don't warm start the longitudinal part (the engine/brake force, we don't want to preserve anything from the last frame)
                w.lateral_part.warmStart(self.body, contact_body, w.contact_lateral.negate(), warm_start_impulse_ratio);
            }
        }

        self.pitch_roll_part.warmStart(self.body, Body.fixedToWorld(), warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *VehicleConstraint, delta_time: f32) bool {
        var impulse = false;

        // Solve suspension
        for (self.wheels.items) |w| {
            if (w.contact_body) |contact_body| {
                const neg_contact_normal = w.contact_normal.negate();

                // Suspension spring, note that it can only push and not pull
                if (w.suspension_part.isActive()) {
                    if (w.suspension_part.solveVelocityConstraint(self.body, contact_body, neg_contact_normal, 0.0, math.flt_max))
                        impulse = true;
                }

                // When reaching the minimal suspension length only allow forces pushing the bodies away
                if (w.suspension_max_up_part.isActive()) {
                    if (w.suspension_max_up_part.solveVelocityConstraint(self.body, contact_body, neg_contact_normal, 0.0, math.flt_max))
                        impulse = true;
                }
            }
        }

        // Solve the horizontal movement of the vehicle
        if (self.controller.solveLongitudinalAndLateralConstraints(delta_time))
            impulse = true;

        // Apply the pitch / roll constraint to avoid the vehicle from toppling over
        if (self.pitch_roll_part.isActive()) {
            if (self.pitch_roll_part.solveVelocityConstraint(self.body, Body.fixedToWorld(), self.pitch_roll_rotation_axis, 0, math.flt_max))
                impulse = true;
        }

        return impulse;
    }

    pub fn solvePositionConstraint(self: *VehicleConstraint, delta_time: f32, baumgarte: f32) bool {
        _ = delta_time;

        var impulse = false;

        const body_transform = self.body.getWorldTransform();

        for (self.wheels.items) |w| {
            if (w.contact_body) |contact_body| {
                const settings = w.getSettings();

                // Check if we reached the 'max up' position now that the body has possibly moved
                // We do this by calculating the axle position at minimum suspension length and making sure it does not go through the
                // plane defined by the contact normal and the axle position when the contact happened
                // TODO: This assumes that only the vehicle moved and not the ground as we kept the axle contact plane in world space
                const ws_direction = body_transform.multiply3x3(settings.suspension_direction);
                const ws_position = body_transform.mulVec3(settings.position);
                const min_suspension_pos = ws_position.addVec3(ws_direction.mulScalar(settings.suspension_min_length));
                const max_up_error: f32 = @floatCast(RVec3.fromVec3(w.contact_normal).dot(min_suspension_pos) - w.axle_plane_constant);
                if (max_up_error < 0.0) {
                    const neg_contact_normal = w.contact_normal.negate();

                    // Recalculate constraint properties since the body may have moved
                    const force_point = self.calculateSuspensionForcePoint(w);
                    w.suspension_max_up_part.calculateConstraintProperties(self.body, force_point.r1_plus_u, contact_body, force_point.r2, neg_contact_normal, .{});

                    if (w.suspension_max_up_part.solvePositionConstraint(self.body, contact_body, neg_contact_normal, max_up_error, baumgarte))
                        impulse = true;
                }
            }
        }

        // Apply the pitch / roll constraint to avoid the vehicle from toppling over
        self.calculatePitchRollConstraintProperties(body_transform);
        if (self.pitch_roll_part.isActive()) {
            if (self.pitch_roll_part.solvePositionConstraint(self.body, Body.fixedToWorld(), self.cos_pitch_roll_angle - self.cos_max_pitch_roll_angle, baumgarte))
                impulse = true;
        }

        return impulse;
    }

    // TODO(debug renderer): DrawConstraint (mController->Draw), DrawConstraintLimits

    pub fn saveState(self: *const VehicleConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(&self.base, stream);

        self.controller.saveState(stream);

        for (self.wheels.items) |w| {
            stream.write(w.angular_velocity);
            stream.write(w.angle);
            stream.write(w.contact_body_id); // Used by MotorcycleController::PreCollide
            stream.write(w.contact_position); // Used by VehicleCollisionTester::PredictContactProperties
            stream.write(w.contact_normal); // Used by MotorcycleController::PreCollide
            stream.write(w.contact_lateral); // Used by MotorcycleController::PreCollide
            stream.write(w.suspension_length); // Used by VehicleCollisionTester::PredictContactProperties

            w.suspension_part.saveState(stream);
            w.suspension_max_up_part.saveState(stream);
            w.longitudinal_part.saveState(stream);
            w.lateral_part.saveState(stream);
        }

        stream.write(self.pitch_roll_rotation_axis); // When rotation is too small we use last frame so we need to store it
        self.pitch_roll_part.saveState(stream);
        stream.write(self.current_step);
    }

    pub fn restoreState(self: *VehicleConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(&self.base, stream);

        self.controller.restoreState(stream);

        for (self.wheels.items) |w| {
            stream.read(&w.angular_velocity);
            stream.read(&w.angle);
            stream.read(&w.contact_body_id);
            stream.read(&w.contact_position);
            stream.read(&w.contact_normal);
            stream.read(&w.contact_lateral);
            stream.read(&w.suspension_length);
            w.contact_body = null; // No longer valid

            w.suspension_part.restoreState(stream);
            w.suspension_max_up_part.restoreState(stream);
            w.longitudinal_part.restoreState(stream);
            w.lateral_part.restoreState(stream);
        }

        stream.read(&self.pitch_roll_rotation_axis);
        self.pitch_roll_part.restoreState(stream);
        stream.read(&self.current_step);
    }

    pub fn getConstraintSettings(self: *const VehicleConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const allocator = self.base.allocator;
        const settings = try VehicleConstraintSettings.create(allocator);
        var result = Ref(ConstraintSettings).init(settings.asSettings());
        errdefer result.deinit();

        self.base.toConstraintSettings(settings.asSettings());
        settings.up = self.up;
        settings.forward = self.forward;
        settings.max_pitch_roll_angle = Trigonometry.acos(self.cos_max_pitch_roll_angle);
        try settings.wheels.ensureTotalCapacityPrecise(allocator, self.wheels.items.len);
        for (self.wheels.items) |w|
            settings.wheels.appendAssumeCapacity(.init(@constCast(w.getSettings()))); // Jolt: const_cast<WheelSettings *> (only the reference count is written)
        settings.anti_roll_bars = try self.anti_roll_bars.clone(allocator);
        settings.controller = try self.controller.getSettings();
        return result;
    }
};

/// RMat44 * Mat44 in both precisions (DMat44::operator*(Mat44Arg) / Mat44::operator*)
fn mulRMat44Mat44(a: RMat44, b: Mat44) RMat44 {
    return if (Core.double_precision) a.mulMat44(b) else a.mul(b);
}

const testing = std.testing;

test "VehicleConstraintSettings defaults, copies and binary state without wheels" {
    const allocator = testing.allocator;
    const StreamWrapper = @import("../../Core/StreamWrapper.zig");
    const WheeledVehicleControllerFile = @import("WheeledVehicleController.zig");

    var settings = VehicleConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    try testing.expect(settings.up.eql(Vec3.init(0, 1, 0)));
    try testing.expect(settings.forward.eql(Vec3.init(0, 0, 1)));
    try testing.expectEqual(math.pi, settings.max_pitch_roll_angle);
    try testing.expect(settings.wheels.items.len == 0 and settings.anti_roll_bars.items.len == 0 and settings.controller.ptr == null);
    try testing.expectEqual(ConstraintSettings.rttiHash("VehicleConstraintSettings"), settings.asSettings().getRTTIHash());

    const wheel = try WheeledVehicleControllerFile.WheelSettingsWV.create(allocator);
    try settings.addWheel(&wheel.base);
    try settings.addAntiRollBar(.{ .left_wheel = 2, .right_wheel = 3, .stiffness = 500.0 });
    const controller = try WheeledVehicleControllerFile.WheeledVehicleControllerSettings.create(allocator);
    settings.setController(&controller.base);
    try testing.expectEqual(@as(u32, 1), wheel.base.getRefCount());
    try testing.expectEqual(@as(u32, 1), controller.base.getRefCount());

    // The copy shares the wheels and the controller
    {
        var copy = try VehicleConstraintSettings.initCopy(allocator, &settings);
        defer copy.asSettings().deinit();
        try testing.expect(copy.wheels.items[0].ptr.? == &wheel.base);
        try testing.expect(copy.controller.ptr.? == &controller.base);
        try testing.expectEqual(@as(u32, 2), wheel.base.getRefCount());
        try testing.expectEqual(@as(u32, 2), controller.base.getRefCount());
        try testing.expectEqual(@as(f32, 500.0), copy.anti_roll_bars.items[0].stiffness);
    }
    try testing.expectEqual(@as(u32, 1), wheel.base.getRefCount());
    const Copy = struct {
        fn run(a: Allocator, s: *const VehicleConstraintSettings) !void {
            var copy = try VehicleConstraintSettings.initCopy(a, s);
            copy.asSettings().deinit();
        }
    };
    try testing.checkAllAllocationFailures(allocator, Copy.run, .{&settings});

    // The binary state of settings without wheels (Jolt cannot restore settings with wheels, see the file comment):
    // the hash, ConstraintSettings, up, forward, the angle, the anti-roll bars, no wheels, the controller hash and the
    // controller
    for (settings.wheels.items) |*w| w.deinit();
    settings.wheels.clearRetainingCapacity();
    settings.max_pitch_roll_angle = 1.0;
    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    settings.asSettings().saveBinaryState(out_wrapper.streamOut());
    var controller_out: std.Io.Writer.Allocating = .init(allocator);
    defer controller_out.deinit();
    var controller_wrapper: StreamWrapper.StreamOutWrapper = .init(&controller_out.writer);
    controller.base.saveBinaryState(controller_wrapper.streamOut());
    try testing.expectEqual(@as(usize, 21 + 12 + 12 + 4 + 4 + 12 + 4 + 4) + controller_out.written().len, out.written().len);

    const Restore = struct {
        fn run(a: Allocator, bytes: []const u8) !void {
            var restored = VehicleConstraintSettings.init(a);
            defer restored.asSettings().deinit();
            var reader: std.Io.Reader = .fixed(bytes);
            var in_wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
            var hash: u32 = 0;
            in_wrapper.streamIn().read(&hash); // Read by sRestoreFromBinaryState
            try testing.expectEqual(ConstraintSettings.rttiHash("VehicleConstraintSettings"), hash);
            try restored.restoreBinaryState(in_wrapper.streamIn());
            try testing.expect(!in_wrapper.isEOF() and !in_wrapper.isFailed());
            try testing.expectEqual(@as(f32, 1.0), restored.max_pitch_roll_angle);
            try testing.expectEqual(@as(usize, 1), restored.anti_roll_bars.items.len);
            try testing.expectEqual(@as(i32, 3), restored.anti_roll_bars.items[0].right_wheel);
            try testing.expectEqual(VehicleControllerSettings.rttiHash("WheeledVehicleControllerSettings"), restored.controller.ptr.?.getRTTIHash());

            // Saved again: the same bytes
            var again: std.Io.Writer.Allocating = .init(a);
            defer again.deinit();
            var again_wrapper: StreamWrapper.StreamOutWrapper = .init(&again.writer);
            restored.asSettings().saveBinaryState(again_wrapper.streamOut());
            if (again_wrapper.isFailed()) return error.OutOfMemory; // The allocating writer could not grow
            try testing.expectEqualSlices(u8, bytes, again.written());
        }
    };
    try Restore.run(allocator, out.written());
    try testing.checkAllAllocationFailures(allocator, Restore.run, .{out.written()});
}
