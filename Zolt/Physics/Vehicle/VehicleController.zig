//! Port of: Jolt/Physics/Vehicle/VehicleController.h, Jolt/Physics/Vehicle/VehicleController.cpp
//! Status: complete
//!
//! - JPH_DECLARE_SERIALIZABLE_ABSTRACT / JPH_IMPLEMENT_SERIALIZABLE_ABSTRACT(VehicleControllerSettings): `rtti`. Jolt
//!   adds no attributes, only the base class SerializableObject.
//! - JPH_DEBUG_RENDERER (`VehicleController::Draw`, pure virtual): an option-gated vtable entry
//!   (`virtual.OptionalVirtual`, no slot without `-Ddebug_renderer`) with a gated dispatcher
//!   (RemainingFeaturesArchitecture.md R2). Every controller implements it, like in Jolt.
//! - CharacterVehicleArchitecture E2:
//!   - `VehicleControllerSettings` is an abstract pattern A root with `ref_count` (RefTarget<VehicleControllerSettings>):
//!     `saveBinaryState`, `restoreBinaryState` and `constructController` are pure virtual. The settings classes declare
//!     `pub const rtti` (the `rtti` vtable data entry, SerializationArchitecture.md S3): VehicleConstraintSettings
//!     writes the RTTI hash of the controller settings and finds the class in `RegisterTypes.factory` (E3).
//!   - `VehicleController` is a pattern A root without `ref_count`: VehicleConstraint owns its controller
//!     (`delete mController` is `controller.destroy()`, like BroadPhase). It keeps Jolt's `VehicleConstraint &mConstraint`
//!     as `constraint: *VehicleConstraint`, so a controller must not outlive (or move away from) its constraint.
//! - The functions that allocate take the allocator and return `Allocator.Error`: `constructController(allocator,
//!   constraint)` (`new XController(*this, inConstraint)`, the constraint passes its own allocator), `constructWheel(
//!   allocator, wheel_settings)` (`new XWheel(settings)`) and `getSettings()` (`new XSettings`, with the controller's
//!   allocator). The functions that run in the physics update (`preCollide`, `postCollide`,
//!   `solveLongitudinalAndLateralConstraints`) cannot report an allocation failure (step listener and solver jobs), so a
//!   controller allocates the memory they need when it is constructed (see WheeledVehicleController.zig).
//! - The protected virtual functions are `pub` (VehicleConstraint is a friend of the controller in Jolt).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const SerializableObject = @import("../../ObjectStream/SerializableObject.zig").SerializableObject;
const virtual = @import("../../Core/Virtual.zig");
const Core = @import("../../Core/Core.zig");
const DebugRenderer = @import("../../Renderer/DebugRenderer.zig").DebugRenderer;
const ReferenceFile = @import("../../Core/Reference.zig");
const Ref = ReferenceFile.Ref;
const RefCount = ReferenceFile.RefCount;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const PhysicsSystem = @import("../PhysicsSystem.zig").PhysicsSystem;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const VehicleConstraint = @import("VehicleConstraint.zig").VehicleConstraint;
const WheelFile = @import("Wheel.zig");
const Wheel = WheelFile.Wheel;
const WheelSettings = WheelFile.WheelSettings;

/// Basic settings object for interface that controls acceleration / deceleration of the vehicle
pub const VehicleControllerSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_ABSTRACT / JPH_IMPLEMENT_SERIALIZABLE_ABSTRACT(VehicleControllerSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "VehicleControllerSettings",
        .kind = .abstract,
        .bases = &.{SerializableObject},
    });

    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// GetRTTI(): JPH_RTTI of the most derived class (a data entry: every concrete class declares `rtti`)
        rtti: *const RTTI,
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *VehicleControllerSettings) void,
        /// delete this (generated)
        destroy: *const fn (self: *VehicleControllerSettings) void,

        /// Saves the contents of the controller settings in binary form to inStream.
        saveBinaryState: *const fn (self: *const VehicleControllerSettings, stream: StreamOut) void,

        /// Restore the contents of the controller settings in binary form from inStream.
        restoreBinaryState: *const fn (self: *VehicleControllerSettings, stream: StreamIn) Allocator.Error!void,

        /// Create an instance of the vehicle controller class (`allocator` frees the controller: its `destroy`)
        constructController: *const fn (self: *const VehicleControllerSettings, allocator: Allocator, constraint: *VehicleConstraint) Allocator.Error!*VehicleController,
    };

    vtable: *const VTable,
    /// Reference count (RefTarget<VehicleControllerSettings>)
    ref_count: RefCount = .{},
    /// Frees this object (heap settings) and the memory the derived classes own
    allocator: Allocator,

    /// Constructor, called by derived classes with their most derived type
    pub fn init(comptime T: type, allocator: Allocator) VehicleControllerSettings {
        return .{ .vtable = vtableFor(T), .allocator = allocator };
    }

    /// The vtable of controller settings class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<VehicleControllerSettings>

    /// Add a reference to this object
    pub fn addRef(self: *const VehicleControllerSettings) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const VehicleControllerSettings) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark these settings as embedded (on the stack or a member), the last release will not destroy them
    pub fn setEmbedded(self: *const VehicleControllerSettings) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const VehicleControllerSettings) u32 {
        return self.ref_count.get();
    }

    /// Destructor of settings that are not on the heap (embedded / stack)
    pub fn deinit(self: *VehicleControllerSettings) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// Saves the contents of the controller settings in binary form to inStream.
    pub fn saveBinaryState(self: *const VehicleControllerSettings, stream: StreamOut) void {
        self.vtable.saveBinaryState(self, stream);
    }

    /// Restore the contents of the controller settings in binary form from inStream.
    pub fn restoreBinaryState(self: *VehicleControllerSettings, stream: StreamIn) Allocator.Error!void {
        return self.vtable.restoreBinaryState(self, stream);
    }

    /// Create an instance of the vehicle controller class
    pub fn constructController(self: *const VehicleControllerSettings, allocator: Allocator, constraint: *VehicleConstraint) Allocator.Error!*VehicleController {
        return self.vtable.constructController(self, allocator, constraint);
    }

    /// GetRTTI()->GetHash() of the most derived class
    pub fn getRTTIHash(self: *const VehicleControllerSettings) u32 {
        return self.vtable.rtti.hash;
    }

    /// Abstract: every virtual function is pure virtual
    pub const impl = struct {};
};

/// Runtime data for interface that controls acceleration / deceleration of the vehicle
pub const VehicleController = struct {
    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *VehicleController) void,
        /// delete this (generated)
        destroy: *const fn (self: *VehicleController) void,

        /// Recreate the settings for this controller (allocated with the controller's allocator)
        getSettings: *const fn (self: *const VehicleController) Allocator.Error!Ref(VehicleControllerSettings),

        // The functions below are only for the VehicleConstraint (protected, VehicleConstraint is a friend)

        /// Create a new instance of wheel (`allocator` frees the wheel: its `destroy`)
        constructWheel: *const fn (self: *const VehicleController, allocator: Allocator, wheel: *const WheelSettings) Allocator.Error!*Wheel,

        /// If the vehicle is allowed to go to sleep
        allowSleep: *const fn (self: *const VehicleController) bool,

        /// Called before the wheel probes have been done
        preCollide: *const fn (self: *VehicleController, delta_time: f32, physics_system: *PhysicsSystem) void,

        /// Called after the wheel probes have been done
        postCollide: *const fn (self: *VehicleController, delta_time: f32, physics_system: *PhysicsSystem) void,

        /// Solve longitudinal and lateral constraint parts for all of the wheels
        solveLongitudinalAndLateralConstraints: *const fn (self: *VehicleController, delta_time: f32) bool,

        /// Saving state for replay
        saveState: *const fn (self: *const VehicleController, stream: *StateRecorder) void,
        restoreState: *const fn (self: *VehicleController, stream: *StateRecorder) void,

        // Drawing interface (JPH_DEBUG_RENDERER: no slot without -Ddebug_renderer)
        draw: virtual.OptionalVirtual(Core.debug_renderer, *const fn (self: *const VehicleController, renderer: *DebugRenderer) void),
    };

    vtable: *const VTable,
    /// Frees the most derived controller (`destroy`) and the memory it owns
    allocator: Allocator,

    /// The vehicle constraint we belong to
    constraint: *VehicleConstraint,

    /// Constructor (explicit VehicleController(VehicleConstraint &)), called by derived classes with their most
    /// derived type
    pub fn init(comptime T: type, allocator: Allocator, constraint: *VehicleConstraint) VehicleController {
        return .{ .vtable = vtableFor(T), .allocator = allocator, .constraint = constraint };
    }

    /// The vtable of controller class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// delete this (through the virtual destructor)
    pub fn destroy(self: *VehicleController) void {
        self.vtable.destroy(self);
    }

    /// Destructor of a controller that is not on the heap (embedded)
    pub fn deinit(self: *VehicleController) void {
        self.vtable.deinit(self);
    }

    /// Access the vehicle constraint that this controller is part of
    pub fn getConstraint(self: *const VehicleController) *VehicleConstraint {
        return self.constraint;
    }

    /// Recreate the settings for this controller
    pub fn getSettings(self: *const VehicleController) Allocator.Error!Ref(VehicleControllerSettings) {
        return self.vtable.getSettings(self);
    }

    /// Create a new instance of wheel
    pub fn constructWheel(self: *const VehicleController, allocator: Allocator, wheel: *const WheelSettings) Allocator.Error!*Wheel {
        return self.vtable.constructWheel(self, allocator, wheel);
    }

    /// If the vehicle is allowed to go to sleep
    pub fn allowSleep(self: *const VehicleController) bool {
        return self.vtable.allowSleep(self);
    }

    /// Called before the wheel probes have been done
    pub fn preCollide(self: *VehicleController, delta_time: f32, physics_system: *PhysicsSystem) void {
        self.vtable.preCollide(self, delta_time, physics_system);
    }

    /// Called after the wheel probes have been done
    pub fn postCollide(self: *VehicleController, delta_time: f32, physics_system: *PhysicsSystem) void {
        self.vtable.postCollide(self, delta_time, physics_system);
    }

    /// Solve longitudinal and lateral constraint parts for all of the wheels
    pub fn solveLongitudinalAndLateralConstraints(self: *VehicleController, delta_time: f32) bool {
        return self.vtable.solveLongitudinalAndLateralConstraints(self, delta_time);
    }

    /// Saving state for replay
    pub fn saveState(self: *const VehicleController, stream: *StateRecorder) void {
        self.vtable.saveState(self, stream);
    }

    pub fn restoreState(self: *VehicleController, stream: *StateRecorder) void {
        self.vtable.restoreState(self, stream);
    }

    // Drawing interface (JPH_DEBUG_RENDERER only: a gated dispatcher, `void` without -Ddebug_renderer)
    pub const draw = if (Core.debug_renderer) drawDispatch else {};
    fn drawDispatch(self: *const VehicleController, renderer: *DebugRenderer) void {
        self.vtable.draw(self, renderer);
    }

    /// Abstract: every virtual function is pure virtual
    pub const impl = struct {};
};

/// A user controller (settings, controller and wheel classes) for the tests of the virtual functions
const TestControllerSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(TestControllerSettings)
    pub const rtti = RTTI.declare(@This(), .{ .name = "TestControllerSettings", .kind = .virtual, .bases = &.{VehicleControllerSettings} });
    pub const overrides = .{ .saveBinaryState, .restoreBinaryState, .constructController };

    base: VehicleControllerSettings,
    value: f32 = 1.0,

    fn create(allocator: Allocator, value: f32) Allocator.Error!*TestControllerSettings {
        const self = try allocator.create(TestControllerSettings);
        self.* = .{ .base = .init(TestControllerSettings, allocator), .value = value };
        return self;
    }

    /// new TestControllerSettings (RTTI create)
    pub fn createDefault(allocator: Allocator) Allocator.Error!*TestControllerSettings {
        return create(allocator, 1.0);
    }

    pub fn saveBinaryState(self: *const TestControllerSettings, stream: StreamOut) void {
        stream.write(self.value);
    }

    pub fn restoreBinaryState(self: *TestControllerSettings, stream: StreamIn) Allocator.Error!void {
        stream.read(&self.value);
    }

    pub fn constructController(self: *const TestControllerSettings, allocator: Allocator, constraint: *VehicleConstraint) Allocator.Error!*VehicleController {
        const controller = try allocator.create(TestController);
        controller.* = .{ .base = .init(TestController, allocator, constraint), .value = self.value };
        return &controller.base;
    }
};

const TestController = struct {
    pub const overrides = .{ .getSettings, .constructWheel, .allowSleep, .preCollide, .postCollide, .solveLongitudinalAndLateralConstraints, .saveState, .restoreState, .draw };

    base: VehicleController,
    value: f32,

    pub fn getSettings(self: *const TestController) Allocator.Error!Ref(VehicleControllerSettings) {
        const settings = try TestControllerSettings.create(self.base.allocator, self.value);
        return .init(&settings.base);
    }

    pub fn constructWheel(self: *const TestController, allocator: Allocator, wheel: *const WheelSettings) Allocator.Error!*Wheel {
        _ = self;
        const result = try allocator.create(TestWheel);
        result.* = .{ .base = .init(TestWheel, allocator, wheel) };
        return &result.base;
    }

    pub fn allowSleep(self: *const TestController) bool {
        return self.value > 0.0;
    }

    pub fn preCollide(self: *TestController, delta_time: f32, physics_system: *PhysicsSystem) void {
        _ = physics_system;
        self.value += delta_time;
    }

    pub fn postCollide(self: *TestController, delta_time: f32, physics_system: *PhysicsSystem) void {
        _ = physics_system;
        self.value -= delta_time;
    }

    pub fn solveLongitudinalAndLateralConstraints(self: *TestController, delta_time: f32) bool {
        _ = .{ self, delta_time };
        return false;
    }

    pub fn saveState(self: *const TestController, stream: *StateRecorder) void {
        stream.write(self.value);
    }

    pub fn restoreState(self: *TestController, stream: *StateRecorder) void {
        stream.read(&self.value);
    }

    pub const draw = if (Core.debug_renderer) drawImpl else {};
    fn drawImpl(self: *const TestController, renderer: *DebugRenderer) void {
        renderer.drawMarker(self.base.constraint.getVehicleBody().getPosition(), @import("../../Core/Color.zig").Color.white, self.value);
    }
};

const TestWheel = struct {
    pub const overrides = .{};

    base: Wheel,
};

test "VehicleControllerSettings and VehicleController dispatch through their vtables" {
    const testing = std.testing;
    const allocator = testing.allocator;
    const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;

    const settings = try TestControllerSettings.create(allocator, 3.0);
    var settings_ref = Ref(VehicleControllerSettings).init(&settings.base);
    defer settings_ref.deinit();
    try testing.expectEqual(RTTI.hashName("TestControllerSettings"), settings.base.getRTTIHash());

    // The controller keeps a pointer to its constraint (not used by this controller)
    var constraint: VehicleConstraint = undefined;
    const controller = try settings.base.constructController(allocator, &constraint);
    defer controller.destroy(); // Frees the TestController through the vtable (leak checked)
    try testing.expect(controller.getConstraint() == &constraint);
    try testing.expect(controller.allowSleep());
    try testing.expect(!controller.solveLongitudinalAndLateralConstraints(1.0 / 60.0));

    // The settings are recreated by the controller
    var recreated = try controller.getSettings();
    defer recreated.deinit();
    try testing.expectEqual(@as(f32, 3.0), virtual.downcast(TestControllerSettings, recreated.get().?).value);

    // State
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    controller.saveState(recorder.recorder());
    const test_controller = virtual.downcast(TestController, controller);
    test_controller.value = -1.0;
    try testing.expect(!controller.allowSleep());
    recorder.rewind();
    controller.restoreState(recorder.recorder());
    try testing.expectEqual(@as(f32, 3.0), test_controller.value);

    // A wheel constructed by the controller references its settings
    const wheel_settings = try WheelSettings.create(allocator);
    var wheel_settings_ref = Ref(WheelSettings).init(wheel_settings);
    defer wheel_settings_ref.deinit();
    const wheel = try controller.constructWheel(allocator, wheel_settings);
    try testing.expectEqual(@as(u32, 2), wheel_settings.getRefCount());
    wheel.destroy();
    try testing.expectEqual(@as(u32, 1), wheel_settings.getRefCount());
}
