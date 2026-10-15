//! Port of: Jolt/Physics/Vehicle/VehicleController.h, Jolt/Physics/Vehicle/VehicleController.cpp
//! Status: complete
//!
//! - JPH_DECLARE_SERIALIZABLE_ABSTRACT / JPH_IMPLEMENT_SERIALIZABLE_ABSTRACT(VehicleControllerSettings):
//!   TODO(serialization) (ObjectStream is Phase 8, CharacterVehicleArchitecture E1; the binary state is ported).
//! - JPH_DEBUG_RENDERER (`VehicleController::Draw`, pure virtual): TODO(debug renderer).
//! - CharacterVehicleArchitecture E2:
//!   - `VehicleControllerSettings` is an abstract pattern A root with `ref_count` (RefTarget<VehicleControllerSettings>):
//!     `saveBinaryState`, `restoreBinaryState` and `constructController` are pure virtual. The settings classes declare
//!     `pub const rtti_name` (a vtable data entry): VehicleConstraintSettings writes the RTTI hash of the controller
//!     settings and restores the class through `RegisterTypes.vehicle_controller_settings_types` (E3).
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
const virtual = @import("../../Core/Virtual.zig");
const HashCombine = @import("../../Core/HashCombine.zig");
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
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_ABSTRACT(JPH_EXPORT, VehicleControllerSettings)

    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// Class name of the most derived class (JPH_RTTI, data entry: every concrete class declares `rtti_name`)
        rtti_name: []const u8,
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
        return rttiHash(self.vtable.rtti_name);
    }

    /// RTTI::GetHash: FNV-1a hash of the class name, folded from 64 to 32 bits
    pub fn rttiHash(name: []const u8) u32 {
        // Perform diffusion step to get from 64 to 32 bits (see https://en.wikipedia.org/wiki/Fowler%E2%80%93Noll%E2%80%93Vo_hash_function)
        const hash = HashCombine.hashString(name);
        return @truncate(hash ^ (hash >> 32));
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

        // TODO(debug renderer): Draw (pure virtual, JPH_DEBUG_RENDERER)
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

    /// Abstract: every virtual function is pure virtual
    pub const impl = struct {};
};
