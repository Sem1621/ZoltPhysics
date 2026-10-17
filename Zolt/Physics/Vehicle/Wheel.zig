//! Port of: Jolt/Physics/Vehicle/Wheel.h, Jolt/Physics/Vehicle/Wheel.cpp
//! Status: complete
//!
//! - JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(WheelSettings): TODO(serialization)
//!   (ObjectStream is Phase 8, CharacterVehicleArchitecture E1; the binary state is ported). The Factory registration of
//!   WheelSettingsWV / WheelSettingsTV is Phase 8 too (E3).
//! - CharacterVehicleArchitecture E2:
//!   - `WheelSettings` is a concrete pattern A root with `ref_count` (RefTarget<WheelSettings>): its virtual functions
//!     `saveBinaryState` / `restoreBinaryState` keep their bodies in `impl` (so that WheelSettings itself can be created,
//!     `create(allocator)` is `new WheelSettings`), derived classes (WheelSettingsWV, WheelSettingsTV, user classes)
//!     embed it as `base` and list their `overrides`. `restoreBinaryState` uses the allocator of the root (the derived
//!     classes restore LinearCurves).
//!   - `WheelSettings.RTTI` is the part of JPH_RTTI that `IsKindOf` needs: the class name and the base class (Jolt's
//!     `JPH_ADD_BASE_CLASS`). Every concrete class declares `pub const rtti: *const WheelSettings.RTTI`, a vtable data
//!     entry. `isKindOf(T)` / `cast(T)` are Jolt's `IsKindOf(&settings, JPH_RTTI(T))` and the checked static cast.
//!   - `Wheel` is a pattern A root without `ref_count`: VehicleConstraint owns its wheels (`delete w` is
//!     `w.destroy()`, the generated `destroy` frees the most derived wheel with the allocator of the root). Its only
//!     virtual function is the destructor, so `Wheel.VTable` has the generated entries only. WheelWV / WheelTV embed it
//!     as `base` (`init(T, allocator, settings)` builds the vtable of the most derived class T).
//! - Deviation (CharacterVehicleArchitecture, "Settled in the foundation review"): Jolt leaves `mContactPosition`,
//!   `mContactPointVelocity`, `mContactNormal`, `mContactLongitudinal`, `mContactLateral` and `mAxlePlaneConstant`
//!   uninitialized until the first contact, and VehicleConstraint::SaveState writes the contact position, normal and
//!   lateral direction. Zolt zero-initializes them (the parity reference library zero-fills its allocations, so a
//!   vehicle whose wheels never had a contact saves the same bytes on both sides).
//! - The getters that Jolt asserts on (`GetContactPosition` ... without a contact) and the asserts on the settings in
//!   the constructor are user input checks: evaluated in safe builds only (`if (Core.enable_asserts)`), optimized
//!   builds return what Jolt's release build returns.

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const virtual = @import("../../Core/Virtual.zig");
const ReferenceFile = @import("../../Core/Reference.zig");
const RefCount = ReferenceFile.RefCount;
const RefConst = ReferenceFile.RefConst;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const RealFile = @import("../../Math/Real.zig");
const Real = RealFile.Real;
const RVec3 = RealFile.RVec3;
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const SubShapeID = @import("../Collision/Shape/SubShapeID.zig").SubShapeID;
const SpringSettings = @import("../Constraints/SpringSettings.zig").SpringSettings;
const AxisConstraintPart = @import("../Constraints/ConstraintPart/AxisConstraintPart.zig").AxisConstraintPart;
const VehicleConstraint = @import("VehicleConstraint.zig").VehicleConstraint;

/// Base class for wheel settings, each VehicleController can implement a derived class of this
pub const WheelSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, WheelSettings)

    /// The part of JPH_RTTI that IsKindOf needs: the class name and its base class (JPH_ADD_BASE_CLASS)
    pub const RTTI = struct {
        /// Name of the class
        name: []const u8,
        /// RTTI of the base class (null for WheelSettings)
        base: ?*const RTTI = null,

        /// True when this class is `other` or derives from it
        pub fn isKindOf(self: *const RTTI, other: *const RTTI) bool {
            var r: ?*const RTTI = self;
            while (r) |x| : (r = x.base)
                if (std.mem.eql(u8, x.name, other.name)) return true;
            return false;
        }
    };

    /// One entry per C++ virtual function, in declaration order
    pub const VTable = struct {
        /// JPH_RTTI of the most derived class (data entry: every concrete class declares `pub const rtti`)
        rtti: *const RTTI,
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *WheelSettings) void,
        /// delete this (generated)
        destroy: *const fn (self: *WheelSettings) void,

        /// Saves the contents in binary form to inStream.
        saveBinaryState: *const fn (self: *const WheelSettings, stream: StreamOut) void,

        /// Restores the contents in binary form to inStream.
        restoreBinaryState: *const fn (self: *WheelSettings, stream: StreamIn) Allocator.Error!void,
    };

    /// JPH_RTTI(WheelSettings)
    pub const rtti: *const RTTI = &.{ .name = "WheelSettings" };

    vtable: *const VTable,
    /// Reference count (RefTarget<WheelSettings>)
    ref_count: RefCount = .{},
    /// Frees this object (heap settings) and the memory the derived classes own
    allocator: Allocator,

    /// Attachment point of wheel suspension in local space of the body
    position: Vec3 = Vec3.zero(),
    /// Where tire forces (suspension and traction) are applied, in local space of the body. A good default is the center of the wheel in its neutral pose. See mEnableSuspensionForcePoint.
    suspension_force_point: Vec3 = Vec3.zero(),
    /// Direction of the suspension in local space of the body, should point down
    suspension_direction: Vec3 = Vec3.init(0, -1, 0),
    /// Direction of the steering axis in local space of the body, should point up (e.g. for a bike would be -mSuspensionDirection)
    steering_axis: Vec3 = Vec3.init(0, 1, 0),
    /// Up direction when the wheel is in the neutral steering position (usually VehicleConstraintSettings::mUp but can be used to give the wheel camber or for a bike would be -mSuspensionDirection)
    wheel_up: Vec3 = Vec3.init(0, 1, 0),
    /// Forward direction when the wheel is in the neutral steering position (usually VehicleConstraintSettings::mForward but can be used to give the wheel toe, does not need to be perpendicular to mWheelUp)
    wheel_forward: Vec3 = Vec3.init(0, 0, 1),
    /// How long the suspension is in max raised position relative to the attachment point (m)
    suspension_min_length: f32 = 0.3,
    /// How long the suspension is in max droop position relative to the attachment point (m)
    suspension_max_length: f32 = 0.5,
    /// The natural length (m) of the suspension spring is defined as mSuspensionMaxLength + mSuspensionPreloadLength. Can be used to preload the suspension as the spring is compressed by mSuspensionPreloadLength when the suspension is in max droop position. Note that this means when the vehicle touches the ground there is a discontinuity so it will also make the vehicle more bouncy as we're updating with discrete time steps.
    suspension_preload_length: f32 = 0.0,
    /// Settings for the suspension spring
    suspension_spring: SpringSettings = .init(.frequency_and_damping, 1.5, 0.5),
    /// Radius of the wheel (m)
    radius: f32 = 0.3,
    /// Width of the wheel (m)
    width: f32 = 0.1,
    /// Enables mSuspensionForcePoint, if disabled, the forces are applied at the collision contact point. This leads to a more accurate simulation when interacting with dynamic objects but makes the vehicle less stable. When setting this to true, all forces will be applied to a fixed point on the vehicle body.
    enable_suspension_force_point: bool = false,

    /// Constructor, called by derived classes with their most derived type
    pub fn init(comptime T: type, allocator: Allocator) WheelSettings {
        return .{ .vtable = vtableFor(T), .allocator = allocator };
    }

    /// new WheelSettings: reference count 0, put it in a Ref / RefConst
    pub fn create(allocator: Allocator) Allocator.Error!*WheelSettings {
        const self = try allocator.create(WheelSettings);
        self.* = .init(WheelSettings, allocator);
        return self;
    }

    /// The vtable of wheel settings class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    // RefTarget<WheelSettings>

    /// Add a reference to this object
    pub fn addRef(self: *const WheelSettings) void {
        self.ref_count.addRef();
    }

    /// Release a reference to this object, destroys it after the last reference
    pub fn release(self: *const WheelSettings) void {
        if (self.ref_count.release()) self.vtable.destroy(@constCast(self)); // The object is dead afterwards (Rule M exception)
    }

    /// Mark these settings as embedded (on the stack or a member), the last release will not destroy them
    pub fn setEmbedded(self: *const WheelSettings) void {
        self.ref_count.setEmbedded();
    }

    /// Get current refcount of this object
    pub fn getRefCount(self: *const WheelSettings) u32 {
        return self.ref_count.get();
    }

    /// Destructor of settings that are not on the heap (embedded / stack)
    pub fn deinit(self: *WheelSettings) void {
        self.ref_count.assertUnreferenced();
        self.vtable.deinit(self);
    }

    /// IsKindOf(this, JPH_RTTI(T)): true when these settings are a T or a class derived from T
    pub fn isKindOf(self: *const WheelSettings, comptime T: type) bool {
        return self.vtable.rtti.isKindOf(T.rtti);
    }

    /// Checked downcast (IsKindOf + static_cast<const T *>): asserts the dynamic type in safe builds
    pub fn cast(self: *const WheelSettings, comptime T: type) *const T {
        if (Core.enable_asserts) std.debug.assert(self.isKindOf(T));
        return virtual.downcast(T, self);
    }

    /// Checked downcast of mutable settings
    pub fn castMut(self: *WheelSettings, comptime T: type) *T {
        if (Core.enable_asserts) std.debug.assert(self.isKindOf(T));
        return virtual.downcast(T, self);
    }

    /// Saves the contents in binary form to inStream.
    pub fn saveBinaryState(self: *const WheelSettings, stream: StreamOut) void {
        self.vtable.saveBinaryState(self, stream);
    }

    /// Restores the contents in binary form to inStream.
    pub fn restoreBinaryState(self: *WheelSettings, stream: StreamIn) Allocator.Error!void {
        return self.vtable.restoreBinaryState(self, stream);
    }

    /// The bodies of the virtual functions of WheelSettings
    pub const impl = struct {
        pub fn saveBinaryState(self: *const WheelSettings, stream: StreamOut) void {
            stream.write(self.suspension_force_point);
            stream.write(self.position);
            stream.write(self.suspension_direction);
            stream.write(self.steering_axis);
            stream.write(self.wheel_forward);
            stream.write(self.wheel_up);
            stream.write(self.suspension_min_length);
            stream.write(self.suspension_max_length);
            stream.write(self.suspension_preload_length);
            self.suspension_spring.saveBinaryState(stream);
            stream.write(self.radius);
            stream.write(self.width);
            stream.write(self.enable_suspension_force_point);
        }

        pub fn restoreBinaryState(self: *WheelSettings, stream: StreamIn) Allocator.Error!void {
            stream.read(&self.suspension_force_point);
            stream.read(&self.position);
            stream.read(&self.suspension_direction);
            stream.read(&self.steering_axis);
            stream.read(&self.wheel_forward);
            stream.read(&self.wheel_up);
            stream.read(&self.suspension_min_length);
            stream.read(&self.suspension_max_length);
            stream.read(&self.suspension_preload_length);
            self.suspension_spring.restoreBinaryState(stream);
            stream.read(&self.radius);
            stream.read(&self.width);
            stream.read(&self.enable_suspension_force_point);
        }
    };
};

/// Base class for runtime data for a wheel, each VehicleController can implement a derived class of this
pub const Wheel = struct {
    /// The virtual destructor (generated entries)
    pub const VTable = struct {
        /// Virtual destructor (generated destructor chain)
        deinit: *const fn (self: *Wheel) void,
        /// delete this (generated)
        destroy: *const fn (self: *Wheel) void,
    };

    vtable: *const VTable,
    /// Frees the most derived wheel (`destroy`)
    allocator: Allocator,

    /// Configuration settings for this wheel
    settings: RefConst(WheelSettings),
    /// ID of body for ground
    contact_body_id: BodyID = .invalid,
    /// Sub shape ID for ground
    contact_sub_shape_id: SubShapeID = .empty,
    /// Body for ground
    contact_body: ?*Body = null,
    /// Current length of the suspension
    suspension_length: f32,
    /// Position of the contact point between wheel and ground (Zolt: zero until the first contact, see the file comment)
    contact_position: RVec3 = RVec3.zero(),
    /// Velocity of the contact point (m / s, not relative to the wheel but in world space)
    contact_point_velocity: Vec3 = Vec3.zero(),
    /// Normal of the contact point between wheel and ground
    contact_normal: Vec3 = Vec3.zero(),
    /// Vector perpendicular to normal in the forward direction
    contact_longitudinal: Vec3 = Vec3.zero(),
    /// Vector perpendicular to normal and longitudinal direction in the right direction
    contact_lateral: Vec3 = Vec3.zero(),
    /// Constant for the contact plane of the axle, defined as ContactNormal . (WorldSpaceSuspensionPoint + SuspensionLength * WorldSpaceSuspensionDirection)
    axle_plane_constant: Real = 0.0,
    /// Amount of impulse applied to the suspension from the anti-rollbars
    anti_roll_bar_impulse: f32 = 0.0,

    /// Rotation around the suspension direction, positive is to the left
    steer_angle: f32 = 0.0,
    /// Rotation speed of wheel, positive when the wheels cause the vehicle to move forwards (rad/s)
    angular_velocity: f32 = 0.0,
    /// Current rotation of the wheel (rad, [0, 2 pi])
    angle: f32 = 0.0,

    /// Controls movement up/down along the contact normal
    suspension_part: AxisConstraintPart = .{},
    /// Adds a hard limit when reaching the minimal suspension length
    suspension_max_up_part: AxisConstraintPart = .{},
    /// Controls movement forward/backward
    longitudinal_part: AxisConstraintPart = .{},
    /// Controls movement sideways (slip)
    lateral_part: AxisConstraintPart = .{},

    /// Constructor (explicit Wheel(const WheelSettings &)), called by derived classes with their most derived type
    pub fn init(comptime T: type, allocator: Allocator, settings: *const WheelSettings) Wheel {
        if (Core.enable_asserts) {
            std.debug.assert(settings.suspension_direction.isNormalized(.{}));
            std.debug.assert(settings.steering_axis.isNormalized(.{}));
            std.debug.assert(settings.wheel_forward.isNormalized(.{}));
            std.debug.assert(settings.wheel_up.isNormalized(.{}));
            std.debug.assert(settings.suspension_min_length >= 0.0);
            std.debug.assert(settings.suspension_max_length >= settings.suspension_min_length);
            std.debug.assert(settings.suspension_preload_length >= 0.0);
            std.debug.assert(settings.suspension_spring.frequency > 0.0);
            std.debug.assert(settings.suspension_spring.damping >= 0.0);
            std.debug.assert(settings.radius > 0.0);
            std.debug.assert(settings.width >= 0.0);
        }
        return .{ .vtable = vtableFor(T), .allocator = allocator, .settings = .init(settings), .suspension_length = settings.suspension_max_length };
    }

    /// The vtable of wheel class T
    pub fn vtableFor(comptime T: type) *const VTable {
        return virtual.vtablePtr(VTable, T);
    }

    /// ~Wheel: releases the settings
    pub fn destruct(self: *Wheel) void {
        self.settings.deinit();
    }

    /// Destructor of a wheel that is not on the heap (embedded / stack)
    pub fn deinit(self: *Wheel) void {
        self.vtable.deinit(self);
    }

    /// delete this (through the virtual destructor)
    pub fn destroy(self: *Wheel) void {
        self.vtable.destroy(self);
    }

    /// Abstract in Zolt: a wheel is always a WheelWV, WheelTV or a user class (Jolt's Wheel has no pure virtuals)
    pub const impl = struct {};

    /// Get settings for the wheel
    pub fn getSettings(self: *const Wheel) *const WheelSettings {
        return self.settings.ptr.?;
    }

    /// Get the angular velocity (rad/s) for this wheel, note that positive means the wheel is rotating such that the car moves forward
    pub fn getAngularVelocity(self: *const Wheel) f32 {
        return self.angular_velocity;
    }

    /// Update the angular velocity (rad/s)
    pub fn setAngularVelocity(self: *Wheel, vel: f32) void {
        self.angular_velocity = vel;
    }

    /// Get the current rotation angle of the wheel in radians [0, 2 pi]
    pub fn getRotationAngle(self: *const Wheel) f32 {
        return self.angle;
    }

    /// Set the current rotation angle of the wheel in radians [0, 2 pi]
    pub fn setRotationAngle(self: *Wheel, angle: f32) void {
        self.angle = angle;
    }

    /// Get the current steer angle of the wheel in radians [-pi, pi], positive is to the left
    pub fn getSteerAngle(self: *const Wheel) f32 {
        return self.steer_angle;
    }

    /// Set the current steer angle of the wheel in radians [-pi, pi]
    pub fn setSteerAngle(self: *Wheel, angle: f32) void {
        self.steer_angle = angle;
    }

    /// Returns true if the wheel is touching an object
    pub fn hasContact(self: *const Wheel) bool {
        return !self.contact_body_id.isInvalid();
    }

    /// Returns the body ID of the body that this wheel is touching
    pub fn getContactBodyID(self: *const Wheel) BodyID {
        return self.contact_body_id;
    }

    /// Returns the sub shape ID where we're contacting the body
    pub fn getContactSubShapeID(self: *const Wheel) SubShapeID {
        return self.contact_sub_shape_id;
    }

    /// Returns the current contact position in world space (note by the time you call this the vehicle has moved)
    pub fn getContactPosition(self: *const Wheel) RVec3 {
        if (Core.enable_asserts) std.debug.assert(self.hasContact());
        return self.contact_position;
    }

    /// Velocity of the contact point (m / s, not relative to the wheel but in world space)
    pub fn getContactPointVelocity(self: *const Wheel) Vec3 {
        if (Core.enable_asserts) std.debug.assert(self.hasContact());
        return self.contact_point_velocity;
    }

    /// Returns the current contact normal in world space (note by the time you call this the vehicle has moved)
    pub fn getContactNormal(self: *const Wheel) Vec3 {
        if (Core.enable_asserts) std.debug.assert(self.hasContact());
        return self.contact_normal;
    }

    /// Returns longitudinal direction (direction along the wheel relative to floor) in world space (note by the time you call this the vehicle has moved)
    pub fn getContactLongitudinal(self: *const Wheel) Vec3 {
        if (Core.enable_asserts) std.debug.assert(self.hasContact());
        return self.contact_longitudinal;
    }

    /// Returns lateral direction (sideways direction) in world space (note by the time you call this the vehicle has moved)
    pub fn getContactLateral(self: *const Wheel) Vec3 {
        if (Core.enable_asserts) std.debug.assert(self.hasContact());
        return self.contact_lateral;
    }

    /// Get the length of the suspension for a wheel (m) relative to the suspension attachment point (hard point)
    pub fn getSuspensionLength(self: *const Wheel) f32 {
        return self.suspension_length;
    }

    /// Check if the suspension hit its upper limit
    pub fn hasHitHardPoint(self: *const Wheel) bool {
        return self.suspension_max_up_part.isActive();
    }

    /// Get the total impulse (N s) that was applied by the suspension
    pub fn getSuspensionLambda(self: *const Wheel) f32 {
        return self.suspension_part.getTotalLambda() + self.suspension_max_up_part.getTotalLambda();
    }

    /// Get total impulse (N s) applied along the forward direction of the wheel
    pub fn getLongitudinalLambda(self: *const Wheel) f32 {
        return self.longitudinal_part.getTotalLambda();
    }

    /// Get total impulse (N s) applied along the sideways direction of the wheel
    pub fn getLateralLambda(self: *const Wheel) f32 {
        return self.lateral_part.getTotalLambda();
    }

    /// Internal function that should only be called by the controller. Used to apply impulses in the forward direction of the vehicle.
    pub fn solveLongitudinalConstraintPart(self: *Wheel, constraint: *const VehicleConstraint, min_impulse: f32, max_impulse: f32) bool {
        return self.longitudinal_part.solveVelocityConstraint(constraint.getVehicleBody(), self.contact_body.?, self.contact_longitudinal.negate(), min_impulse, max_impulse);
    }

    /// Internal function that should only be called by the controller. Used to apply impulses in the sideways direction of the vehicle.
    pub fn solveLateralConstraintPart(self: *Wheel, constraint: *const VehicleConstraint, min_impulse: f32, max_impulse: f32) bool {
        return self.lateral_part.solveVelocityConstraint(constraint.getVehicleBody(), self.contact_body.?, self.contact_lateral.negate(), min_impulse, max_impulse);
    }
};

/// The wheels of a vehicle (Wheels: an array of owning pointers, VehicleConstraint destroys them)
pub const Wheels = std.ArrayList(*Wheel);

const testing = std.testing;

/// A wheel settings class derived from WheelSettings with a member and its own binary state (tests the virtual calls,
/// the destructor chain and IsKindOf)
const TestWheelSettings = struct {
    pub const overrides = .{ .saveBinaryState, .restoreBinaryState };
    pub const rtti: *const WheelSettings.RTTI = &.{ .name = "TestWheelSettings", .base = WheelSettings.rtti };

    base: WheelSettings,
    extra: f32 = 2.0,
    destructed: ?*bool = null,

    fn create(allocator: Allocator) !*TestWheelSettings {
        const self = try allocator.create(TestWheelSettings);
        self.* = .{ .base = .init(TestWheelSettings, allocator) };
        return self;
    }

    pub fn destruct(self: *TestWheelSettings) void {
        if (self.destructed) |d| d.* = true;
    }

    pub fn saveBinaryState(self: *const TestWheelSettings, stream: StreamOut) void {
        WheelSettings.impl.saveBinaryState(&self.base, stream);
        stream.write(self.extra);
    }

    pub fn restoreBinaryState(self: *TestWheelSettings, stream: StreamIn) Allocator.Error!void {
        try WheelSettings.impl.restoreBinaryState(&self.base, stream);
        stream.read(&self.extra);
    }
};

/// A wheel class for the tests
const TestWheel = struct {
    pub const overrides = .{};

    base: Wheel,
    destructed: ?*bool = null,

    pub fn destruct(self: *TestWheel) void {
        if (self.destructed) |d| d.* = true;
    }
};

test "WheelSettings defaults and binary state" {
    const allocator = testing.allocator;
    const StreamWrapper = @import("../../Core/StreamWrapper.zig");

    const settings = try WheelSettings.create(allocator);
    var ref = RefConst(WheelSettings).init(settings);
    defer ref.deinit();
    try testing.expect(settings.position.eql(Vec3.zero()));
    try testing.expect(settings.suspension_force_point.eql(Vec3.zero()));
    try testing.expect(settings.suspension_direction.eql(Vec3.init(0, -1, 0)));
    try testing.expect(settings.steering_axis.eql(Vec3.init(0, 1, 0)));
    try testing.expect(settings.wheel_up.eql(Vec3.init(0, 1, 0)));
    try testing.expect(settings.wheel_forward.eql(Vec3.init(0, 0, 1)));
    try testing.expectEqual(@as(f32, 0.3), settings.suspension_min_length);
    try testing.expectEqual(@as(f32, 0.5), settings.suspension_max_length);
    try testing.expectEqual(@as(f32, 0.0), settings.suspension_preload_length);
    try testing.expectEqual(@as(f32, 1.5), settings.suspension_spring.frequency);
    try testing.expectEqual(@as(f32, 0.5), settings.suspension_spring.damping);
    try testing.expectEqual(@as(f32, 0.3), settings.radius);
    try testing.expectEqual(@as(f32, 0.1), settings.width);
    try testing.expect(!settings.enable_suspension_force_point);
    try testing.expectEqual(@as(u32, 1), settings.getRefCount());

    settings.position = Vec3.init(1, 2, 3);
    settings.enable_suspension_force_point = true;
    settings.suspension_spring.mode = .stiffness_and_damping;

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    settings.saveBinaryState(out_wrapper.streamOut());
    try testing.expectEqual(@as(usize, 6 * 12 + 3 * 4 + 9 + 2 * 4 + 1), out.written().len);
    try testing.expectEqualSlices(u8, std.mem.asBytes(&[3]f32{ 1, 2, 3 }), out.written()[12..24]); // mSuspensionForcePoint comes first

    const restored = try WheelSettings.create(allocator);
    var restored_ref = RefConst(WheelSettings).init(restored);
    defer restored_ref.deinit();
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
    try restored.restoreBinaryState(in_wrapper.streamIn());
    try testing.expect(restored.position.eql(Vec3.init(1, 2, 3)));
    try testing.expect(restored.enable_suspension_force_point);
    try testing.expectEqual(settings.suspension_spring.mode, restored.suspension_spring.mode);
}

test "WheelSettings virtual binary state, IsKindOf and the destructor chain of a derived class" {
    const allocator = testing.allocator;
    const StreamWrapper = @import("../../Core/StreamWrapper.zig");

    var destructed = false;
    const derived = try TestWheelSettings.create(allocator);
    derived.destructed = &destructed;
    derived.extra = 5.0;
    const base: *WheelSettings = &derived.base;
    try testing.expect(base.isKindOf(WheelSettings));
    try testing.expect(base.isKindOf(TestWheelSettings));
    try testing.expect(base.cast(TestWheelSettings) == derived);

    var out: std.Io.Writer.Allocating = .init(allocator);
    defer out.deinit();
    var out_wrapper: StreamWrapper.StreamOutWrapper = .init(&out.writer);
    base.saveBinaryState(out_wrapper.streamOut()); // Virtual call
    try testing.expectEqual(@as(usize, 6 * 12 + 3 * 4 + 9 + 2 * 4 + 1 + 4), out.written().len);

    derived.extra = 0.0;
    var reader: std.Io.Reader = .fixed(out.written());
    var in_wrapper: StreamWrapper.StreamInWrapper = .init(&reader);
    try base.restoreBinaryState(in_wrapper.streamIn());
    try testing.expectEqual(@as(f32, 5.0), derived.extra);

    // A plain WheelSettings is not a TestWheelSettings
    const plain = try WheelSettings.create(allocator);
    try testing.expect(!plain.isKindOf(TestWheelSettings));
    plain.addRef();
    plain.release();

    base.addRef();
    base.release(); // Destroys through the vtable (leak checked)
    try testing.expect(destructed);
}

test "Wheel construction, getters and destruction" {
    const allocator = testing.allocator;

    const settings = try WheelSettings.create(allocator);
    settings.suspension_max_length = 0.7;
    var settings_ref = RefConst(WheelSettings).init(settings);
    defer settings_ref.deinit();

    var destructed = false;
    const wheel = try allocator.create(TestWheel);
    wheel.* = .{ .base = .init(TestWheel, allocator, settings), .destructed = &destructed };
    try testing.expectEqual(@as(u32, 2), settings.getRefCount());
    const w: *Wheel = &wheel.base;
    try testing.expect(w.getSettings() == settings);
    try testing.expectEqual(@as(f32, 0.7), w.getSuspensionLength());
    try testing.expect(!w.hasContact());
    try testing.expect(w.getContactBodyID().isInvalid());
    try testing.expect(w.getContactSubShapeID().isEmpty());
    try testing.expect(!w.hasHitHardPoint());
    try testing.expectEqual(@as(f32, 0.0), w.getSuspensionLambda());
    try testing.expectEqual(@as(f32, 0.0), w.getLongitudinalLambda());
    try testing.expectEqual(@as(f32, 0.0), w.getLateralLambda());

    // The contact members start at zero (Zolt, see the file comment)
    try testing.expect(w.contact_position.eql(RVec3.zero()));
    try testing.expect(w.contact_normal.eql(Vec3.zero()));
    try testing.expect(w.contact_lateral.eql(Vec3.zero()));

    w.setAngularVelocity(3.0);
    w.setRotationAngle(1.0);
    w.setSteerAngle(-0.5);
    try testing.expectEqual(@as(f32, 3.0), w.getAngularVelocity());
    try testing.expectEqual(@as(f32, 1.0), w.getRotationAngle());
    try testing.expectEqual(@as(f32, -0.5), w.getSteerAngle());

    w.contact_body_id = BodyID.init(5);
    w.contact_normal = Vec3.axisY();
    try testing.expect(w.hasContact());
    try testing.expect(w.getContactNormal().eql(Vec3.axisY()));

    w.destroy(); // Frees the TestWheel, releases the settings (leak checked)
    try testing.expect(destructed);
    try testing.expectEqual(@as(u32, 1), settings.getRefCount());
}
