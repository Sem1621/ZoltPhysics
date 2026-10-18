//! Port of: Jolt/Physics/Constraints/SixDOFConstraint.h, Jolt/Physics/Constraints/SixDOFConstraint.cpp
//! Status: complete
//!
//! Follows the porter template in FixedConstraint.zig. JPH_DECLARE_SERIALIZABLE_VIRTUAL /
//! JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(SixDOFConstraintSettings): TODO(serialization) (the binary state is ported).
//! JPH_DEBUG_RENDERER (DrawConstraint, DrawConstraintLimits): TODO(debug_renderer).
//!
//! - `SixDOFConstraintSettings::EAxis` is `SixDOFConstraintSettings.Axis` (also `SixDOFConstraint.Axis`, Jolt's `using
//!   EAxis`), an exhaustive enum of the six axes. Its extra enumerators `Num` and `NumTranslation` are the constants
//!   `Axis.num` and `Axis.num_translation` (Jolt indexes its arrays with every EAxis below `Num`; `Num` itself is no
//!   axis, an index of 6 is out of bounds in Jolt too).
//! - The const and non-const overloads of `GetMotorSettings(EAxis)` are `getMotorSettings` (`*const`) and
//!   `getMotorSettingsMut` (`*`, guide section 4).
//! - The private `GetPositionConstraintProperties(Vec3 &outR1PlusU, Vec3 &outR2, Vec3 &outU)` returns the three vectors
//!   as a struct (`PositionConstraintProperties`).
//! - `GetTranslationLimitsMin()` etc. load three consecutive floats of `mLimitMin` / `mLimitMax`
//!   (`Vec3::sLoadFloat3Unsafe` of a reinterpret_cast), Zolt builds the Vec3 from the three array elements.
//! - The asserts that user input can violate (a motor state other than Off with invalid motor settings) only run with
//!   `Core.enable_asserts`, so ReleaseFast keeps Jolt's release behavior. `GetLimitsSpringSettings` /
//!   `SetLimitsSpringSettings` of a rotation axis: Jolt asserts and then indexes past the array (undefined behavior),
//!   Zolt keeps the plain assert (and the bounds checked index of the safe builds).
//! - Like Jolt, `SetLimitsSpringSettings` only recaches mHasSpringLimits (no UpdateFixedFreeAxis): when it switches a
//!   fully fixed translation between the point part and the axis parts, the parts keep their warm start impulses.
//! - `EMotorState` accepts every int (MotorSettings.zig): a motor state outside the named ones (only reachable through
//!   RestoreState of foreign bytes) matches no case of Jolt's switches, the `_` prongs do nothing like Jolt.
//!   `ESwingType` (SwingTwistConstraintPart.zig) accepts every byte too: RestoreBinaryState stores whatever the stream
//!   holds, a value other than Cone behaves like Pyramid (Jolt only compares with Cone).
//! - Jolt's `GetConstraintSettings` does not copy `mLimitsSpringSettings` (the returned settings have default, hard
//!   limits); the port does the same on purpose (parity).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const Core = @import("../../Core/Core.zig");
const Ref = @import("../../Core/Reference.zig").Ref;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const RVec3 = @import("../../Math/Real.zig").RVec3;
const Body = @import("../Body/Body.zig").Body;
const BodyID = @import("../Body/BodyID.zig").BodyID;
const StateRecorder = @import("../StateRecorder.zig").StateRecorder;
const ConstraintFile = @import("Constraint.zig");
const Constraint = ConstraintFile.Constraint;
const ConstraintSettings = ConstraintFile.ConstraintSettings;
const ConstraintSpace = ConstraintFile.ConstraintSpace;
const ConstraintSubType = ConstraintFile.ConstraintSubType;
const TwoBodyConstraintFile = @import("TwoBodyConstraint.zig");
const TwoBodyConstraint = TwoBodyConstraintFile.TwoBodyConstraint;
const TwoBodyConstraintSettings = TwoBodyConstraintFile.TwoBodyConstraintSettings;
const MotorSettingsFile = @import("MotorSettings.zig");
const MotorSettings = MotorSettingsFile.MotorSettings;
const MotorState = MotorSettingsFile.MotorState;
const isPositionMotor = MotorSettingsFile.isPositionMotor;
const SpringSettings = @import("SpringSettings.zig").SpringSettings;
const PointConstraintPart = @import("ConstraintPart/PointConstraintPart.zig").PointConstraintPart;
const AxisConstraintPart = @import("ConstraintPart/AxisConstraintPart.zig").AxisConstraintPart;
const AngleConstraintPart = @import("ConstraintPart/AngleConstraintPart.zig").AngleConstraintPart;
const RotationEulerConstraintPart = @import("ConstraintPart/RotationEulerConstraintPart.zig").RotationEulerConstraintPart;
const SwingTwistConstraintPartFile = @import("ConstraintPart/SwingTwistConstraintPart.zig");
const SwingTwistConstraintPart = SwingTwistConstraintPartFile.SwingTwistConstraintPart;
const SwingType = SwingTwistConstraintPartFile.SwingType;

/// 6 Degree Of Freedom Constraint setup structure. Allows control over each of the 6 degrees of freedom.
pub const SixDOFConstraintSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(SixDOFConstraintSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "SixDOFConstraintSettings",
        .kind = .virtual,
        .bases = &.{TwoBodyConstraintSettings},
    });
    /// TODO(serialization): the attributes of Jolt's sCreateRTTI (SerializationArchitecture.md S18)
    pub const serialization_pending = true;

    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    /// Constraint is split up into translation/rotation around X, Y and Z axis.
    pub const Axis = enum {
        translation_x,
        translation_y,
        translation_z,

        rotation_x,
        rotation_y,
        rotation_z,

        /// EAxis::Num
        pub const num = 6;
        /// EAxis::NumTranslation = TranslationZ + 1
        pub const num_translation = @intFromEnum(Axis.translation_z) + 1;
    };

    base: TwoBodyConstraintSettings,

    /// This determines in which space the constraint is setup, all properties below should be in the specified space
    space: ConstraintSpace = .world_space,

    /// Body 1 constraint reference frame (space determined by mSpace)
    position1: RVec3 = RVec3.zero(),
    axis_x1: Vec3 = Vec3.axisX(),
    axis_y1: Vec3 = Vec3.axisY(),

    /// Body 2 constraint reference frame (space determined by mSpace)
    position2: RVec3 = RVec3.zero(),
    axis_x2: Vec3 = Vec3.axisX(),
    axis_y2: Vec3 = Vec3.axisY(),

    /// Friction settings.
    /// For translation: Max friction force in N. 0 = no friction.
    /// For rotation: Max friction torque in Nm. 0 = no friction.
    max_friction: [Axis.num]f32 = .{ 0, 0, 0, 0, 0, 0 },

    /// The type of swing constraint that we want to use.
    swing_type: SwingType = .cone,

    /// Limits.
    /// For translation: Min and max linear limits in m (0 is frame of body 1 and 2 coincide).
    /// For rotation: Min and max angular limits in rad (0 is frame of body 1 and 2 coincide). See comments at Axis enum for limit ranges.
    ///
    /// Remove degree of freedom by setting min = FLT_MAX and max = -FLT_MAX. The constraint will be driven to 0 for this axis.
    ///
    /// Free movement over an axis is allowed when min = -FLT_MAX and max = FLT_MAX.
    ///
    /// Rotation limit around X-Axis: When limited, should be \f$\in [-\pi, \pi]\f$. Can be asymmetric around zero.
    ///
    /// Rotation limit around Y-Z Axis: Forms a pyramid or cone shaped limit:
    /// * For pyramid, should be \f$\in [-\pi, \pi]\f$ and does not need to be symmetrical around zero.
    /// * For cone should be \f$\in [0, \pi]\f$ and needs to be symmetrical around zero (min limit is assumed to be -max limit).
    limit_min: [Axis.num]f32 = .{ -math.flt_max, -math.flt_max, -math.flt_max, -math.flt_max, -math.flt_max, -math.flt_max },
    limit_max: [Axis.num]f32 = .{ math.flt_max, math.flt_max, math.flt_max, math.flt_max, math.flt_max, math.flt_max },

    /// When enabled, this makes the limits soft. When the constraint exceeds the limits, a spring force will pull it back.
    /// Only soft translation limits are supported, soft rotation limits are not currently supported.
    limits_spring_settings: [Axis.num_translation]SpringSettings = @splat(.{}),

    /// Motor settings for each axis
    motor_settings: [Axis.num]MotorSettings = @splat(.{}),

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) SixDOFConstraintSettings {
        return .{ .base = .init(SixDOFConstraintSettings, allocator) };
    }

    /// new SixDOFConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*SixDOFConstraintSettings {
        const self = try allocator.create(SixDOFConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other`, not its reference count
    pub fn initCopy(allocator: Allocator, other: *const SixDOFConstraintSettings) SixDOFConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        return result;
    }

    pub fn asSettings(self: *SixDOFConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const SixDOFConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *SixDOFConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const SixDOFConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const SixDOFConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.space);
        stream.write(self.position1);
        stream.write(self.axis_x1);
        stream.write(self.axis_y1);
        stream.write(self.position2);
        stream.write(self.axis_x2);
        stream.write(self.axis_y2);
        stream.write(self.max_friction);
        stream.write(self.swing_type);
        stream.write(self.limit_min);
        stream.write(self.limit_max);
        for (&self.limits_spring_settings) |*s|
            s.saveBinaryState(stream);
        for (&self.motor_settings) |*m|
            m.saveBinaryState(stream);
    }

    /// Create an instance of this constraint
    pub fn createConstraint(self: *const SixDOFConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        const constraint = try SixDOFConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    /// Make axis free (unconstrained)
    pub fn makeFreeAxis(self: *SixDOFConstraintSettings, axis: Axis) void {
        self.limit_min[@intFromEnum(axis)] = -math.flt_max;
        self.limit_max[@intFromEnum(axis)] = math.flt_max;
    }

    pub fn isFreeAxis(self: *const SixDOFConstraintSettings, axis: Axis) bool {
        return self.limit_min[@intFromEnum(axis)] == -math.flt_max and self.limit_max[@intFromEnum(axis)] == math.flt_max;
    }

    /// Make axis fixed (fixed at value 0)
    pub fn makeFixedAxis(self: *SixDOFConstraintSettings, axis: Axis) void {
        self.limit_min[@intFromEnum(axis)] = math.flt_max;
        self.limit_max[@intFromEnum(axis)] = -math.flt_max;
    }

    pub fn isFixedAxis(self: *const SixDOFConstraintSettings, axis: Axis) bool {
        return self.limit_min[@intFromEnum(axis)] >= self.limit_max[@intFromEnum(axis)];
    }

    /// Set a valid range for the constraint (if inMax < inMin, the axis will become fixed)
    pub fn setLimitedAxis(self: *SixDOFConstraintSettings, axis: Axis, min: f32, max: f32) void {
        self.limit_min[@intFromEnum(axis)] = min;
        self.limit_max[@intFromEnum(axis)] = max;
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *SixDOFConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.space);
        stream.read(&self.position1);
        stream.read(&self.axis_x1);
        stream.read(&self.axis_y1);
        stream.read(&self.position2);
        stream.read(&self.axis_x2);
        stream.read(&self.axis_y2);
        stream.read(&self.max_friction);
        stream.read(&self.swing_type);
        stream.read(&self.limit_min);
        stream.read(&self.limit_max);
        for (&self.limits_spring_settings) |*s|
            s.restoreBinaryState(stream);
        for (&self.motor_settings) |*m|
            m.restoreBinaryState(stream);
    }
};

/// 6 Degree Of Freedom Constraint. Allows control over each of the 6 degrees of freedom.
pub const SixDOFConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    /// Get Axis from settings class
    pub const Axis = SixDOFConstraintSettings.Axis;

    base: TwoBodyConstraint,

    // CONFIGURATION PROPERTIES FOLLOW

    // Local space constraint positions
    local_space_position1: Vec3 = undefined,
    local_space_position2: Vec3 = undefined,

    // Transforms from constraint space to body space
    constraint_to_body1: Quat = undefined,
    constraint_to_body2: Quat = undefined,

    // Limits
    /// Bitmask of free axis (bit 0 = TranslationX)
    free_axis: u8 = 0,
    /// Bitmask of fixed axis (bit 0 = TranslationX)
    fixed_axis: u8 = 0,
    /// If any of the translational frictions / motors are active
    translation_motor_active: bool = false,
    /// If any of the rotational frictions / motors are active
    rotation_motor_active: bool = false,
    /// Bitmask of axis that have position motor active (bit 0 = RotationX)
    rotation_position_motor_active: u8 = 0,
    /// If any of the limit springs have a non-zero frequency/stiffness
    has_spring_limits: bool = false,
    limit_min: [Axis.num]f32 = undefined,
    limit_max: [Axis.num]f32 = undefined,
    limits_spring_settings: [Axis.num_translation]SpringSettings = @splat(.{}),

    // Motor settings for each axis
    motor_settings: [Axis.num]MotorSettings = @splat(.{}),

    // Friction settings for each axis
    max_friction: [Axis.num]f32 = undefined,

    // Motor controls
    motor_state: [Axis.num]MotorState = .{ .off, .off, .off, .off, .off, .off },
    target_velocity: Vec3 = Vec3.zero(),
    target_angular_velocity: Vec3 = Vec3.zero(),
    target_position: Vec3 = Vec3.zero(),
    target_orientation: Quat = Quat.identity(),

    // RUN TIME PROPERTIES FOLLOW

    // Constraint space axis in world space
    translation_axis: [3]Vec3 = undefined,
    rotation_axis: [3]Vec3 = undefined,

    // Translation displacement (valid when translation axis has a range limit)
    displacement: [3]f32 = undefined,

    // Individual constraint parts for translation, or a combined point constraint part if all axis are fixed
    translation_constraint_part: [3]AxisConstraintPart = @splat(.{}),
    point_constraint_part: PointConstraintPart = .{},

    // Individual constraint parts for rotation or a combined constraint part if rotation is fixed
    swing_twist_constraint_part: SwingTwistConstraintPart = .{},
    rotation_constraint_part: RotationEulerConstraintPart = .{},

    // Motor or friction constraints
    motor_translation_constraint_part: [3]AxisConstraintPart = @splat(.{}),
    motor_rotation_constraint_part: [3]AngleConstraintPart = @splat(.{}),

    /// Construct six DOF constraint
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const SixDOFConstraintSettings) SixDOFConstraint {
        var self: SixDOFConstraint = .{ .base = .init(SixDOFConstraint, allocator, body1, body2, &settings.base) };

        // Override swing type
        self.swing_twist_constraint_part.setSwingType(settings.swing_type);

        // Calculate rotation needed to go from constraint space to body1 local space
        const axis_z1 = settings.axis_x1.cross(settings.axis_y1);
        const c_to_b1 = Mat44.init(Vec4.fromVec3W(settings.axis_x1, 0), Vec4.fromVec3W(settings.axis_y1, 0), Vec4.fromVec3W(axis_z1, 0), Vec4.init(0, 0, 0, 1));
        self.constraint_to_body1 = c_to_b1.getQuaternion();

        // Calculate rotation needed to go from constraint space to body2 local space
        const axis_z2 = settings.axis_x2.cross(settings.axis_y2);
        const c_to_b2 = Mat44.init(Vec4.fromVec3W(settings.axis_x2, 0), Vec4.fromVec3W(settings.axis_y2, 0), Vec4.fromVec3W(axis_z2, 0), Vec4.init(0, 0, 0, 1));
        self.constraint_to_body2 = c_to_b2.getQuaternion();

        if (settings.space == .world_space) {
            // If all properties were specified in world space, take them to local space now
            self.local_space_position1 = body1.getInverseCenterOfMassTransform().mulRVec3(settings.position1).toVec3();
            self.constraint_to_body1 = body1.getRotation().conjugated().mul(self.constraint_to_body1);

            self.local_space_position2 = body2.getInverseCenterOfMassTransform().mulRVec3(settings.position2).toVec3();
            self.constraint_to_body2 = body2.getRotation().conjugated().mul(self.constraint_to_body2);
        } else {
            self.local_space_position1 = settings.position1.toVec3();
            self.local_space_position2 = settings.position2.toVec3();
        }

        // Copy translation and rotation limits
        self.limit_min = settings.limit_min;
        self.limit_max = settings.limit_max;
        self.limits_spring_settings = settings.limits_spring_settings;
        self.updateTranslationLimits();
        self.updateRotationLimits();
        self.updateFixedFreeAxis();
        self.cacheHasSpringLimits();

        // Store friction settings
        self.max_friction = settings.max_friction;

        // Store motor settings
        for (0..Axis.num) |i|
            self.motor_settings[i] = settings.motor_settings[i];

        // Cache if motors are active (motors are off initially, but we may have friction)
        self.cacheTranslationMotorActive();
        self.cacheRotationMotorActive();

        return self;
    }

    /// new SixDOFConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const SixDOFConstraintSettings) Allocator.Error!*SixDOFConstraint {
        const self = try allocator.create(SixDOFConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    pub fn asConstraint(self: *SixDOFConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const SixDOFConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *SixDOFConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const SixDOFConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    // Sanitize the translation limits
    fn updateTranslationLimits(self: *SixDOFConstraint) void {
        // Set to zero if the limits are inversed
        for (@intFromEnum(Axis.translation_x)..@intFromEnum(Axis.translation_z) + 1) |i| {
            if (self.limit_min[i] > self.limit_max[i]) {
                self.limit_max[i] = 0.0;
                self.limit_min[i] = 0.0;
            }
        }
    }

    // Propagate the rotation limits to the constraint part
    fn updateRotationLimits(self: *SixDOFConstraint) void {
        const rotation_y = @intFromEnum(Axis.rotation_y);
        const rotation_z = @intFromEnum(Axis.rotation_z);

        if (self.swing_twist_constraint_part.getSwingType() == .cone) {
            // Cone swing upper limit needs to be positive
            self.limit_max[rotation_y] = math.max(@as(f32, 0.0), self.limit_max[rotation_y]);
            self.limit_max[rotation_z] = math.max(@as(f32, 0.0), self.limit_max[rotation_z]);

            // Cone swing limits only support symmetric ranges
            self.limit_min[rotation_y] = -self.limit_max[rotation_y];
            self.limit_min[rotation_z] = -self.limit_max[rotation_z];
        }

        for (@intFromEnum(Axis.rotation_x)..@intFromEnum(Axis.rotation_z) + 1) |i| {
            // Clamp to [-PI, PI] range
            self.limit_min[i] = math.clamp(self.limit_min[i], -math.pi, math.pi);
            self.limit_max[i] = math.clamp(self.limit_max[i], -math.pi, math.pi);

            // Set to zero if the limits are inversed
            if (self.limit_min[i] > self.limit_max[i]) {
                self.limit_max[i] = 0.0;
                self.limit_min[i] = 0.0;
            }
        }

        // Pass limits on to constraint part
        const rotation_x = @intFromEnum(Axis.rotation_x);
        self.swing_twist_constraint_part.setLimits(self.limit_min[rotation_x], self.limit_max[rotation_x], self.limit_min[rotation_y], self.limit_max[rotation_y], self.limit_min[rotation_z], self.limit_max[rotation_z]);
    }

    // Update the cached state of which axis are free and which ones are fixed
    fn updateFixedFreeAxis(self: *SixDOFConstraint) void {
        const old_free_axis = self.free_axis;
        const old_fixed_axis = self.fixed_axis;

        // Cache which axis are fixed and which ones are free
        self.free_axis = 0;
        self.fixed_axis = 0;
        for (0..Axis.num) |a| {
            const limit: f32 = if (a >= @intFromEnum(Axis.rotation_x)) math.pi else math.flt_max;

            if (self.limit_min[a] >= self.limit_max[a])
                self.fixed_axis |= @as(u8, 1) << @intCast(a)
            else if (self.limit_min[a] <= -limit and self.limit_max[a] >= limit)
                self.free_axis |= @as(u8, 1) << @intCast(a);
        }

        // On change we deactivate all constraints to reset warm starting
        if (old_free_axis != self.free_axis or old_fixed_axis != self.fixed_axis) {
            for (&self.translation_constraint_part) |*c|
                c.deactivate();
            self.point_constraint_part.deactivate();
            self.swing_twist_constraint_part.deactivate();
            self.rotation_constraint_part.deactivate();
            for (&self.motor_translation_constraint_part) |*c|
                c.deactivate();
            for (&self.motor_rotation_constraint_part) |*c|
                c.deactivate();
        }
    }

    // Generic interface of a constraint
    pub fn getSubType(self: *const SixDOFConstraint) ConstraintSubType {
        _ = self;
        return .six_dof;
    }

    pub fn notifyShapeChanged(self: *SixDOFConstraint, body_id: BodyID, delta_com: Vec3) void {
        if (self.base.body1.getID().eql(body_id)) {
            self.local_space_position1 = self.local_space_position1.sub(delta_com);
        } else if (self.base.body2.getID().eql(body_id)) {
            self.local_space_position2 = self.local_space_position2.sub(delta_com);
        }
    }

    /// Update the translation limits for this constraint
    pub fn setTranslationLimits(self: *SixDOFConstraint, limit_min: Vec3, limit_max: Vec3) void {
        self.limit_min[@intFromEnum(Axis.translation_x)] = limit_min.getX();
        self.limit_min[@intFromEnum(Axis.translation_y)] = limit_min.getY();
        self.limit_min[@intFromEnum(Axis.translation_z)] = limit_min.getZ();
        self.limit_max[@intFromEnum(Axis.translation_x)] = limit_max.getX();
        self.limit_max[@intFromEnum(Axis.translation_y)] = limit_max.getY();
        self.limit_max[@intFromEnum(Axis.translation_z)] = limit_max.getZ();

        self.updateTranslationLimits();
        self.updateFixedFreeAxis();
    }

    /// Update the rotational limits for this constraint
    pub fn setRotationLimits(self: *SixDOFConstraint, limit_min: Vec3, limit_max: Vec3) void {
        self.limit_min[@intFromEnum(Axis.rotation_x)] = limit_min.getX();
        self.limit_min[@intFromEnum(Axis.rotation_y)] = limit_min.getY();
        self.limit_min[@intFromEnum(Axis.rotation_z)] = limit_min.getZ();
        self.limit_max[@intFromEnum(Axis.rotation_x)] = limit_max.getX();
        self.limit_max[@intFromEnum(Axis.rotation_y)] = limit_max.getY();
        self.limit_max[@intFromEnum(Axis.rotation_z)] = limit_max.getZ();

        self.updateRotationLimits();
        self.updateFixedFreeAxis();
    }

    /// Get constraint Limits
    pub fn getLimitsMin(self: *const SixDOFConstraint, axis: Axis) f32 {
        return self.limit_min[@intFromEnum(axis)];
    }

    pub fn getLimitsMax(self: *const SixDOFConstraint, axis: Axis) f32 {
        return self.limit_max[@intFromEnum(axis)];
    }

    pub fn getTranslationLimitsMin(self: *const SixDOFConstraint) Vec3 {
        const i = @intFromEnum(Axis.translation_x);
        return Vec3.init(self.limit_min[i], self.limit_min[i + 1], self.limit_min[i + 2]);
    }

    pub fn getTranslationLimitsMax(self: *const SixDOFConstraint) Vec3 {
        const i = @intFromEnum(Axis.translation_x);
        return Vec3.init(self.limit_max[i], self.limit_max[i + 1], self.limit_max[i + 2]);
    }

    pub fn getRotationLimitsMin(self: *const SixDOFConstraint) Vec3 {
        const i = @intFromEnum(Axis.rotation_x);
        return Vec3.init(self.limit_min[i], self.limit_min[i + 1], self.limit_min[i + 2]);
    }

    pub fn getRotationLimitsMax(self: *const SixDOFConstraint) Vec3 {
        const i = @intFromEnum(Axis.rotation_x);
        return Vec3.init(self.limit_max[i], self.limit_max[i + 1], self.limit_max[i + 2]);
    }

    /// Check which axis are fixed/free
    pub inline fn isFixedAxis(self: *const SixDOFConstraint, axis: Axis) bool {
        return (self.fixed_axis & (@as(u8, 1) << @intFromEnum(axis))) != 0;
    }

    pub inline fn isFreeAxis(self: *const SixDOFConstraint, axis: Axis) bool {
        return (self.free_axis & (@as(u8, 1) << @intFromEnum(axis))) != 0;
    }

    /// Update the limits spring settings
    pub fn getLimitsSpringSettings(self: *const SixDOFConstraint, axis: Axis) *const SpringSettings {
        std.debug.assert(@intFromEnum(axis) < Axis.num_translation);
        return &self.limits_spring_settings[@intFromEnum(axis)];
    }

    pub fn setLimitsSpringSettings(self: *SixDOFConstraint, axis: Axis, limits_spring_settings: SpringSettings) void {
        std.debug.assert(@intFromEnum(axis) < Axis.num_translation);
        self.limits_spring_settings[@intFromEnum(axis)] = limits_spring_settings;
        self.cacheHasSpringLimits();
    }

    /// Set the max friction for each axis
    pub fn setMaxFriction(self: *SixDOFConstraint, axis: Axis, friction: f32) void {
        self.max_friction[@intFromEnum(axis)] = friction;

        if (@intFromEnum(axis) >= @intFromEnum(Axis.translation_x) and @intFromEnum(axis) <= @intFromEnum(Axis.translation_z))
            self.cacheTranslationMotorActive()
        else
            self.cacheRotationMotorActive();
    }

    pub fn getMaxFriction(self: *const SixDOFConstraint, axis: Axis) f32 {
        return self.max_friction[@intFromEnum(axis)];
    }

    /// Calculate properties needed for the position constraint (GetPositionConstraintProperties' out parameters)
    const PositionConstraintProperties = struct { r1_plus_u: Vec3, r2: Vec3, u: Vec3 };

    // Calculate properties needed for the position constraint
    inline fn getPositionConstraintProperties(self: *const SixDOFConstraint) PositionConstraintProperties {
        const p1 = self.base.body1.getCenterOfMassTransform().mulVec3(self.local_space_position1);
        const p2 = self.base.body2.getCenterOfMassTransform().mulVec3(self.local_space_position2);
        return .{
            .r1_plus_u = p2.sub(self.base.body1.getCenterOfMassPosition()).toVec3(), // r1 + u = (p1 - x1) + (p2 - p1) = p2 - x1
            .r2 = p2.sub(self.base.body2.getCenterOfMassPosition()).toVec3(),
            .u = p2.sub(p1).toVec3(),
        };
    }

    /// Get rotation of constraint in constraint space
    pub fn getRotationInConstraintSpace(self: *const SixDOFConstraint) Quat {
        // Let b1, b2 be the center of mass transform of body1 and body2 (For body1 this is mBody1->GetCenterOfMassTransform())
        // Let c1, c2 be the transform that takes a vector from constraint space to local space of body1 and body2 (For body1 this is Mat44::sRotationTranslation(mConstraintToBody1, mLocalSpacePosition1))
        // Let q be the rotation of the constraint in constraint space
        // b2 takes a vector from the local space of body2 to world space
        // To express this in terms of b1: b2 = b1 * c1 * q * c2^-1
        // c2^-1 goes from local body 2 space to constraint space
        // q rotates the constraint
        // c1 goes from constraint space to body 1 local space
        // b1 goes from body 1 local space to world space
        // So when the body rotations are given, q = (b1 * c1)^-1 * b2 c2
        // Or: q = (q1 * c1)^-1 * (q2 * c2) if we're only interested in rotations
        return self.base.body1.getRotation().mul(self.constraint_to_body1).conjugated().mul(self.base.body2.getRotation()).mul(self.constraint_to_body2);
    }

    // Cache the state of mTranslationMotorActive
    fn cacheTranslationMotorActive(self: *SixDOFConstraint) void {
        self.translation_motor_active = self.motor_state[@intFromEnum(Axis.translation_x)] != .off or
            self.motor_state[@intFromEnum(Axis.translation_y)] != .off or
            self.motor_state[@intFromEnum(Axis.translation_z)] != .off or
            self.hasFriction(.translation_x) or
            self.hasFriction(.translation_y) or
            self.hasFriction(.translation_z);
    }

    // Cache the state of mRotationMotorActive
    fn cacheRotationMotorActive(self: *SixDOFConstraint) void {
        self.rotation_motor_active = self.motor_state[@intFromEnum(Axis.rotation_x)] != .off or
            self.motor_state[@intFromEnum(Axis.rotation_y)] != .off or
            self.motor_state[@intFromEnum(Axis.rotation_z)] != .off or
            self.hasFriction(.rotation_x) or
            self.hasFriction(.rotation_y) or
            self.hasFriction(.rotation_z);
    }

    // Cache the state of mRotationPositionMotorActive
    fn cacheRotationPositionMotorActive(self: *SixDOFConstraint) void {
        self.rotation_position_motor_active = 0;
        for (0..3) |i| {
            if (isPositionMotor(self.motor_state[@intFromEnum(Axis.rotation_x) + i]))
                self.rotation_position_motor_active |= @as(u8, 1) << @intCast(i);
        }
    }

    /// Cache the state of mHasSpringLimits
    fn cacheHasSpringLimits(self: *SixDOFConstraint) void {
        self.has_spring_limits = self.limits_spring_settings[@intFromEnum(Axis.translation_x)].frequency > 0.0 or
            self.limits_spring_settings[@intFromEnum(Axis.translation_y)].frequency > 0.0 or
            self.limits_spring_settings[@intFromEnum(Axis.translation_z)].frequency > 0.0;
    }

    // Constraint settings helper functions
    inline fn isTranslationConstrained(self: *const SixDOFConstraint) bool {
        return (self.free_axis & 0b111) != 0b111;
    }

    inline fn isTranslationFullyConstrained(self: *const SixDOFConstraint) bool {
        return (self.fixed_axis & 0b111) == 0b111 and !self.has_spring_limits;
    }

    inline fn isRotationConstrained(self: *const SixDOFConstraint) bool {
        return (self.free_axis & 0b111000) != 0b111000;
    }

    inline fn isRotationFullyConstrained(self: *const SixDOFConstraint) bool {
        return (self.fixed_axis & 0b111000) == 0b111000;
    }

    inline fn hasFriction(self: *const SixDOFConstraint, axis: Axis) bool {
        return !self.isFixedAxis(axis) and self.max_friction[@intFromEnum(axis)] > 0.0;
    }

    /// Motor settings
    pub fn getMotorSettingsMut(self: *SixDOFConstraint, axis: Axis) *MotorSettings {
        return &self.motor_settings[@intFromEnum(axis)];
    }

    pub fn getMotorSettings(self: *const SixDOFConstraint, axis: Axis) *const MotorSettings {
        return &self.motor_settings[@intFromEnum(axis)];
    }

    /// Motor controls.
    /// Translation motors work in constraint space of body 1.
    /// Rotation motors work in constraint space of body 2 (!).
    pub fn setMotorState(self: *SixDOFConstraint, axis: Axis, state: MotorState) void {
        const a = @intFromEnum(axis);
        if (Core.enable_asserts) std.debug.assert(state == .off or self.motor_settings[a].isValid());

        if (self.motor_state[a] != state) {
            self.motor_state[a] = state;

            // Ensure that warm starting next frame doesn't apply any impulses (motor parts are repurposed for different modes)
            if (a >= @intFromEnum(Axis.translation_x) and a <= @intFromEnum(Axis.translation_z)) {
                self.motor_translation_constraint_part[a - @intFromEnum(Axis.translation_x)].deactivate();

                self.cacheTranslationMotorActive();
            } else {
                std.debug.assert(a >= @intFromEnum(Axis.rotation_x) and a <= @intFromEnum(Axis.rotation_z));

                self.motor_rotation_constraint_part[a - @intFromEnum(Axis.rotation_x)].deactivate();

                self.cacheRotationMotorActive();
                self.cacheRotationPositionMotorActive();
            }
        }
    }

    pub fn getMotorState(self: *const SixDOFConstraint, axis: Axis) MotorState {
        return self.motor_state[@intFromEnum(axis)];
    }

    /// Set the target velocity in body 1 constraint space
    pub fn getTargetVelocityCS(self: *const SixDOFConstraint) Vec3 {
        return self.target_velocity;
    }

    pub fn setTargetVelocityCS(self: *SixDOFConstraint, velocity: Vec3) void {
        self.target_velocity = velocity;
    }

    /// Set the target angular velocity in body 2 constraint space (!)
    pub fn setTargetAngularVelocityCS(self: *SixDOFConstraint, angular_velocity: Vec3) void {
        self.target_angular_velocity = angular_velocity;
    }

    pub fn getTargetAngularVelocityCS(self: *const SixDOFConstraint) Vec3 {
        return self.target_angular_velocity;
    }

    /// Set the target position in body 1 constraint space
    pub fn getTargetPositionCS(self: *const SixDOFConstraint) Vec3 {
        return self.target_position;
    }

    pub fn setTargetPositionCS(self: *SixDOFConstraint, position: Vec3) void {
        self.target_position = position;
    }

    /// Set the target orientation in body 1 constraint space
    pub fn setTargetOrientationCS(self: *SixDOFConstraint, orientation: Quat) void {
        const swing_twist = orientation.getSwingTwist();
        var q_swing = swing_twist.swing;
        var q_twist = swing_twist.twist;

        const clamped_axis = self.swing_twist_constraint_part.clampSwingTwist(&q_swing, &q_twist);

        if (clamped_axis != 0)
            self.target_orientation = q_swing.mul(q_twist)
        else
            self.target_orientation = orientation;
    }

    pub fn getTargetOrientationCS(self: *const SixDOFConstraint) Quat {
        return self.target_orientation;
    }

    /// Set the target orientation in body space (R2 = R1 * inOrientation, where R1 and R2 are the world space rotations for body 1 and 2).
    /// Solve: R2 * ConstraintToBody2 = R1 * ConstraintToBody1 * q (see SwingTwistConstraint::GetSwingTwist) and R2 = R1 * inOrientation for q.
    pub fn setTargetOrientationBS(self: *SixDOFConstraint, orientation: Quat) void {
        self.setTargetOrientationCS(self.constraint_to_body1.conjugated().mul(orientation).mul(self.constraint_to_body2));
    }

    pub fn setupVelocityConstraint(self: *SixDOFConstraint, delta_time: f32) void {
        const body1 = self.base.body1;
        const body2 = self.base.body2;

        // Get body rotations
        const rotation1 = body1.getRotation();
        const rotation2 = body2.getRotation();

        // Quaternion that rotates from body1's constraint space to world space
        const constraint_body1_to_world = rotation1.mul(self.constraint_to_body1);

        // Store world space axis of constraint space
        const translation_axis_mat = Mat44.rotationQuat(constraint_body1_to_world);
        for (0..3) |i|
            self.translation_axis[i] = translation_axis_mat.getColumn3(@intCast(i));

        if (self.isTranslationFullyConstrained()) {
            // All translation locked: Setup point constraint
            self.point_constraint_part.calculateConstraintProperties(body1, Mat44.rotationQuat(rotation1), self.local_space_position1, body2, Mat44.rotationQuat(rotation2), self.local_space_position2);
        } else if (self.isTranslationConstrained() or self.translation_motor_active) {
            // Update world space positions (the bodies may have moved)
            const props = self.getPositionConstraintProperties();
            const r1_plus_u = props.r1_plus_u;
            const r2 = props.r2;
            const u = props.u;

            // Setup axis constraint parts
            for (0..3) |i| {
                const axis: Axis = @enumFromInt(@intFromEnum(Axis.translation_x) + i);

                const translation_axis = self.translation_axis[i];

                // Calculate displacement along this axis
                const d = translation_axis.dot(u);
                self.displacement[i] = d; // Store for SolveVelocityConstraint

                // Setup limit constraint
                var constraint_active = false;
                var constraint_value: f32 = 0.0;
                if (self.isFixedAxis(axis)) {
                    // When constraint is fixed it is always active
                    constraint_value = d - self.limit_min[i];
                    constraint_active = true;
                } else if (!self.isFreeAxis(axis)) {
                    // When constraint is limited, it is only active when outside of the allowed range
                    if (d <= self.limit_min[i]) {
                        constraint_value = d - self.limit_min[i];
                        constraint_active = true;
                    } else if (d >= self.limit_max[i]) {
                        constraint_value = d - self.limit_max[i];
                        constraint_active = true;
                    }
                }

                if (constraint_active)
                    self.translation_constraint_part[i].calculateConstraintPropertiesWithSettingsForLimit(delta_time, body1, r1_plus_u, body2, r2, translation_axis, 0.0, constraint_value, &self.limits_spring_settings[i])
                else
                    self.translation_constraint_part[i].deactivate();

                // Setup motor constraint
                switch (self.motor_state[i]) {
                    .off => {
                        if (self.hasFriction(axis))
                            self.motor_translation_constraint_part[i].calculateConstraintProperties(body1, r1_plus_u, body2, r2, translation_axis, .{})
                        else
                            self.motor_translation_constraint_part[i].deactivate();
                    },

                    .velocity => self.motor_translation_constraint_part[i].calculateConstraintProperties(body1, r1_plus_u, body2, r2, translation_axis, .{ .bias = -self.target_velocity.getComponent(@intCast(i)) }),

                    .position => {
                        const spring_settings = &self.motor_settings[i].spring_settings;
                        if (spring_settings.hasStiffness())
                            self.motor_translation_constraint_part[i].calculateConstraintPropertiesWithSettingsForMotor(delta_time, body1, r1_plus_u, body2, r2, translation_axis, 0.0, translation_axis.dot(u) - self.target_position.getComponent(@intCast(i)), spring_settings)
                        else
                            self.motor_translation_constraint_part[i].deactivate();
                    },

                    .position_and_velocity => {
                        const spring_settings = &self.motor_settings[i].spring_settings;
                        if (spring_settings.hasStiffnessOrDamping())
                            self.motor_translation_constraint_part[i].calculateConstraintPropertiesWithSettingsForMotor(delta_time, body1, r1_plus_u, body2, r2, translation_axis, -self.target_velocity.getComponent(@intCast(i)), translation_axis.dot(u) - self.target_position.getComponent(@intCast(i)), spring_settings)
                        else
                            self.motor_translation_constraint_part[i].deactivate();
                    },

                    _ => {}, // No case of Jolt's switch matches
                }
            }
        }

        // Setup rotation constraints
        if (self.isRotationFullyConstrained()) {
            // All rotation locked: Setup rotation constraint
            self.rotation_constraint_part.calculateConstraintProperties(body1, Mat44.rotationQuat(rotation1), body2, Mat44.rotationQuat(rotation2));
        } else if (self.isRotationConstrained() or self.rotation_motor_active) {
            // GetRotationInConstraintSpace without redoing the calculation of constraint_body1_to_world
            const constraint_body2_to_world = rotation2.mul(self.constraint_to_body2);
            const q = constraint_body1_to_world.conjugated().mul(constraint_body2_to_world);

            // Use swing twist constraint part
            if (self.isRotationConstrained())
                self.swing_twist_constraint_part.calculateConstraintProperties(body1, body2, q, constraint_body1_to_world)
            else
                self.swing_twist_constraint_part.deactivate();

            if (self.rotation_motor_active) {
                // Calculate rotation motor axis
                const ws_axis = Mat44.rotationQuat(constraint_body2_to_world);
                for (0..3) |i|
                    self.rotation_axis[i] = ws_axis.getColumn3(@intCast(i));

                // Get target orientation along the shortest path from q
                const target_orientation = if (q.dot(self.target_orientation) > 0.0) self.target_orientation else self.target_orientation.negate();

                // The definition of the constraint rotation q:
                // R2 * ConstraintToBody2 = R1 * ConstraintToBody1 * q (1)
                //
                // R2' is the rotation of body 2 when reaching the target_orientation:
                // R2' * ConstraintToBody2 = R1 * ConstraintToBody1 * target_orientation (2)
                //
                // The difference in body 2 space:
                // R2' = R2 * diff_body2 (3)
                //
                // We want to specify the difference in the constraint space of body 2:
                // diff_body2 = ConstraintToBody2 * diff * ConstraintToBody2^* (4)
                //
                // Extracting R2' from 2: R2' = R1 * ConstraintToBody1 * target_orientation * ConstraintToBody2^* (5)
                // Combining 3 & 4: R2' = R2 * ConstraintToBody2 * diff * ConstraintToBody2^* (6)
                // Combining 1 & 6: R2' = R1 * ConstraintToBody1 * q * diff * ConstraintToBody2^* (7)
                // Combining 5 & 7: R1 * ConstraintToBody1 * target_orientation * ConstraintToBody2^* = R1 * ConstraintToBody1 * q * diff * ConstraintToBody2^*
                // <=> target_orientation = q * diff
                // <=> diff = q^* * target_orientation
                const diff = q.conjugated().mul(target_orientation);

                // Project diff so that only rotation around axis that have a position motor are remaining
                const projected_diff = switch (self.rotation_position_motor_active) {
                    // Keep only rotation around X
                    0b001 => diff.getTwist(Vec3.axisX()),

                    // Keep only rotation around Y
                    0b010 => diff.getTwist(Vec3.axisY()),

                    // Keep only rotation around Z
                    0b100 => diff.getTwist(Vec3.axisZ()),

                    // Remove rotation around Z
                    // q = swing_xy * twist_z <=> swing_xy = q * twist_z^*
                    0b011 => diff.mul(diff.getTwist(Vec3.axisZ()).conjugated()),

                    // Remove rotation around Y
                    // q = swing_xz * twist_y <=> swing_xz = q * twist_y^*
                    0b101 => diff.mul(diff.getTwist(Vec3.axisY()).conjugated()),

                    // Remove rotation around X
                    // q = swing_yz * twist_x <=> swing_yz = q * twist_x^*
                    0b110 => diff.mul(diff.getTwist(Vec3.axisX()).conjugated()),

                    // 0b111 and default: All motors off is handled here but the results are unused
                    // Keep entire rotation
                    else => diff,
                };

                // Approximate error angles
                // The imaginary part of a quaternion is rotation_axis * sin(angle / 2)
                // If angle is small, sin(x) = x so angle[i] ~ 2.0f * rotation_axis[i]
                // We'll be making small time steps, so if the angle is not small at least the sign will be correct and we'll move in the right direction
                const rotation_error = projected_diff.getXYZ().mulScalar(-2.0);

                // Setup motors
                for (0..3) |i| {
                    const axis: Axis = @enumFromInt(@intFromEnum(Axis.rotation_x) + i);
                    const a = @intFromEnum(axis);

                    const rotation_axis = self.rotation_axis[i];

                    switch (self.motor_state[a]) {
                        .off => {
                            if (self.hasFriction(axis))
                                self.motor_rotation_constraint_part[i].calculateConstraintProperties(body1, body2, rotation_axis, .{})
                            else
                                self.motor_rotation_constraint_part[i].deactivate();
                        },

                        .velocity => self.motor_rotation_constraint_part[i].calculateConstraintProperties(body1, body2, rotation_axis, .{ .bias = -self.target_angular_velocity.getComponent(@intCast(i)) }),

                        .position => {
                            const spring_settings = &self.motor_settings[a].spring_settings;
                            if (spring_settings.hasStiffness())
                                self.motor_rotation_constraint_part[i].calculateConstraintPropertiesWithSettingsForMotor(delta_time, body1, body2, rotation_axis, 0.0, rotation_error.getComponent(@intCast(i)), spring_settings)
                            else
                                self.motor_rotation_constraint_part[i].deactivate();
                        },

                        .position_and_velocity => {
                            const spring_settings = &self.motor_settings[a].spring_settings;
                            if (spring_settings.hasStiffnessOrDamping())
                                self.motor_rotation_constraint_part[i].calculateConstraintPropertiesWithSettingsForMotor(delta_time, body1, body2, rotation_axis, -self.target_angular_velocity.getComponent(@intCast(i)), rotation_error.getComponent(@intCast(i)), spring_settings)
                            else
                                self.motor_rotation_constraint_part[i].deactivate();
                        },

                        _ => {}, // No case of Jolt's switch matches
                    }
                }
            }
        }
    }

    pub fn resetWarmStart(self: *SixDOFConstraint) void {
        for (&self.motor_translation_constraint_part) |*c|
            c.deactivate();
        for (&self.motor_rotation_constraint_part) |*c|
            c.deactivate();
        self.rotation_constraint_part.deactivate();
        self.swing_twist_constraint_part.deactivate();
        self.point_constraint_part.deactivate();
        for (&self.translation_constraint_part) |*c|
            c.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *SixDOFConstraint, warm_start_impulse_ratio: f32) void {
        const body1 = self.base.body1;
        const body2 = self.base.body2;

        // Warm start translation motors
        if (self.translation_motor_active) {
            for (0..3) |i| {
                if (self.motor_translation_constraint_part[i].isActive())
                    self.motor_translation_constraint_part[i].warmStart(body1, body2, self.translation_axis[i], warm_start_impulse_ratio);
            }
        }

        // Warm start rotation motors
        if (self.rotation_motor_active) {
            for (&self.motor_rotation_constraint_part) |*c| {
                if (c.isActive())
                    c.warmStart(body1, body2, warm_start_impulse_ratio);
            }
        }

        // Warm start rotation constraints
        if (self.isRotationFullyConstrained())
            self.rotation_constraint_part.warmStart(body1, body2, warm_start_impulse_ratio)
        else if (self.isRotationConstrained())
            self.swing_twist_constraint_part.warmStart(body1, body2, warm_start_impulse_ratio);

        // Warm start translation constraints
        if (self.isTranslationFullyConstrained()) {
            self.point_constraint_part.warmStart(body1, body2, warm_start_impulse_ratio);
        } else if (self.isTranslationConstrained()) {
            for (0..3) |i| {
                if (self.translation_constraint_part[i].isActive())
                    self.translation_constraint_part[i].warmStart(body1, body2, self.translation_axis[i], warm_start_impulse_ratio);
            }
        }
    }

    pub fn solveVelocityConstraint(self: *SixDOFConstraint, delta_time: f32) bool {
        const body1 = self.base.body1;
        const body2 = self.base.body2;

        var impulse = false;

        // Solve translation motor
        if (self.translation_motor_active) {
            for (0..3) |i| {
                if (self.motor_translation_constraint_part[i].isActive()) {
                    switch (self.motor_state[i]) {
                        .off => {
                            // Apply friction only
                            const max_lambda = self.max_friction[i] * delta_time;
                            if (self.motor_translation_constraint_part[i].solveVelocityConstraint(body1, body2, self.translation_axis[i], -max_lambda, max_lambda))
                                impulse = true;
                        },

                        .velocity, .position, .position_and_velocity => {
                            // Drive motor
                            if (self.motor_translation_constraint_part[i].solveVelocityConstraint(body1, body2, self.translation_axis[i], delta_time * self.motor_settings[i].min_force_limit, delta_time * self.motor_settings[i].max_force_limit))
                                impulse = true;
                        },

                        _ => {}, // No case of Jolt's switch matches
                    }
                }
            }
        }

        // Solve rotation motor
        if (self.rotation_motor_active) {
            for (0..3) |i| {
                const a = @intFromEnum(Axis.rotation_x) + i;
                if (self.motor_rotation_constraint_part[i].isActive()) {
                    switch (self.motor_state[a]) {
                        .off => {
                            // Apply friction only
                            const max_lambda = self.max_friction[a] * delta_time;
                            if (self.motor_rotation_constraint_part[i].solveVelocityConstraint(body1, body2, self.rotation_axis[i], -max_lambda, max_lambda))
                                impulse = true;
                        },

                        .velocity, .position, .position_and_velocity => {
                            // Drive motor
                            if (self.motor_rotation_constraint_part[i].solveVelocityConstraint(body1, body2, self.rotation_axis[i], delta_time * self.motor_settings[a].min_torque_limit, delta_time * self.motor_settings[a].max_torque_limit))
                                impulse = true;
                        },

                        _ => {}, // No case of Jolt's switch matches
                    }
                }
            }
        }

        // Solve rotation constraint
        if (self.isRotationFullyConstrained()) {
            if (self.rotation_constraint_part.solveVelocityConstraint(body1, body2))
                impulse = true;
        } else if (self.isRotationConstrained()) {
            if (self.swing_twist_constraint_part.solveVelocityConstraint(body1, body2))
                impulse = true;
        }

        // Solve position constraint
        if (self.isTranslationFullyConstrained()) {
            if (self.point_constraint_part.solveVelocityConstraint(body1, body2))
                impulse = true;
        } else if (self.isTranslationConstrained()) {
            for (0..3) |i| {
                if (self.translation_constraint_part[i].isActive()) {
                    // If the axis is not fixed it must be limited (or else the constraint would not be active)
                    // Calculate the min and max constraint force based on on which side we're limited
                    var limit_min: f32 = -math.flt_max;
                    var limit_max: f32 = math.flt_max;
                    const axis: Axis = @enumFromInt(@intFromEnum(Axis.translation_x) + i);
                    if (!self.isFixedAxis(axis)) {
                        std.debug.assert(!self.isFreeAxis(axis));
                        if (self.displacement[i] <= self.limit_min[i])
                            limit_min = 0
                        else if (self.displacement[i] >= self.limit_max[i])
                            limit_max = 0;
                    }

                    if (self.translation_constraint_part[i].solveVelocityConstraint(body1, body2, self.translation_axis[i], limit_min, limit_max))
                        impulse = true;
                }
            }
        }

        return impulse;
    }

    pub fn solvePositionConstraint(self: *SixDOFConstraint, delta_time: f32, baumgarte: f32) bool {
        _ = delta_time;
        const body1 = self.base.body1;
        const body2 = self.base.body2;

        var impulse = false;

        if (self.isRotationFullyConstrained()) {
            // Rotation locked: Solve rotation constraint

            // Inverse of initial rotation from body 1 to body 2 in body 1 space
            // Definition of initial orientation r0: q2 = q1 r0
            // Initial rotation (see: GetRotationInConstraintSpace): q2 = q1 c1 c2^-1
            // So: r0^-1 = (c1 c2^-1)^-1 = c2 * c1^-1
            const constraint_to_body1 = self.constraint_to_body1.mul(Quat.eulerAngles(self.getRotationLimitsMin()));
            const inv_initial_orientation = self.constraint_to_body2.mul(constraint_to_body1.conjugated());

            // Solve rotation violations
            self.rotation_constraint_part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), body2, Mat44.rotationQuat(body2.getRotation()));
            if (self.rotation_constraint_part.solvePositionConstraint(body1, body2, inv_initial_orientation, baumgarte))
                impulse = true;
        } else if (self.isRotationConstrained()) {
            // Rotation partially constraint

            // Solve rotation violations
            const q = self.getRotationInConstraintSpace();
            if (self.swing_twist_constraint_part.solvePositionConstraint(body1, body2, q, self.constraint_to_body1, self.constraint_to_body2, baumgarte))
                impulse = true;
        }

        // Solve position violations
        if (self.isTranslationFullyConstrained()) {
            // Translation locked: Solve point constraint
            const local_space_position1 = self.local_space_position1.add(self.constraint_to_body1.mulVec3(self.getTranslationLimitsMin()));
            self.point_constraint_part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), local_space_position1, body2, Mat44.rotationQuat(body2.getRotation()), self.local_space_position2);
            if (self.point_constraint_part.solvePositionConstraint(body1, body2, baumgarte))
                impulse = true;
        } else if (self.isTranslationConstrained()) {
            // Translation partially locked: Solve per axis
            for (0..3) |i| {
                if (self.limits_spring_settings[i].frequency <= 0.0) { // If not soft limit
                    // Update world space positions (the bodies may have moved)
                    const props = self.getPositionConstraintProperties();
                    const r1_plus_u = props.r1_plus_u;
                    const r2 = props.r2;
                    const u = props.u;

                    // Quaternion that rotates from body1's constraint space to world space
                    const constraint_body1_to_world = body1.getRotation().mul(self.constraint_to_body1);

                    // Calculate axis
                    const translation_axis = switch (i) {
                        0 => constraint_body1_to_world.rotateAxisX(),
                        1 => constraint_body1_to_world.rotateAxisY(),
                        else => blk: {
                            std.debug.assert(i == 2);
                            break :blk constraint_body1_to_world.rotateAxisZ();
                        },
                    };

                    // Determine position error
                    var err: f32 = 0.0;
                    const axis: Axis = @enumFromInt(@intFromEnum(Axis.translation_x) + i);
                    const a = @intFromEnum(axis);
                    if (self.isFixedAxis(axis)) {
                        err = u.dot(translation_axis) - self.limit_min[a];
                    } else if (!self.isFreeAxis(axis)) {
                        const displacement = u.dot(translation_axis);
                        if (displacement <= self.limit_min[a])
                            err = displacement - self.limit_min[a]
                        else if (displacement >= self.limit_max[a])
                            err = displacement - self.limit_max[a];
                    }

                    if (err != 0.0) {
                        // Setup axis constraint part and solve it
                        self.translation_constraint_part[i].calculateConstraintProperties(body1, r1_plus_u, body2, r2, translation_axis, .{});
                        if (self.translation_constraint_part[i].solvePositionConstraint(body1, body2, translation_axis, err, baumgarte))
                            impulse = true;
                    }
                }
            }
        }

        return impulse;
    }

    // TODO(debug_renderer): DrawConstraint, DrawConstraintLimits

    pub fn saveState(self: *const SixDOFConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        for (&self.translation_constraint_part) |*c|
            c.saveState(stream);
        self.point_constraint_part.saveState(stream);
        self.swing_twist_constraint_part.saveState(stream);
        self.rotation_constraint_part.saveState(stream);
        for (&self.motor_translation_constraint_part) |*c|
            c.saveState(stream);
        for (&self.motor_rotation_constraint_part) |*c|
            c.saveState(stream);

        stream.write(self.motor_state);
        stream.write(self.target_velocity);
        stream.write(self.target_angular_velocity);
        stream.write(self.target_position);
        stream.write(self.target_orientation);
    }

    pub fn restoreState(self: *SixDOFConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        for (&self.translation_constraint_part) |*c|
            c.restoreState(stream);
        self.point_constraint_part.restoreState(stream);
        self.swing_twist_constraint_part.restoreState(stream);
        self.rotation_constraint_part.restoreState(stream);
        for (&self.motor_translation_constraint_part) |*c|
            c.restoreState(stream);
        for (&self.motor_rotation_constraint_part) |*c|
            c.restoreState(stream);

        stream.read(&self.motor_state);
        stream.read(&self.target_velocity);
        stream.read(&self.target_angular_velocity);
        stream.read(&self.target_position);
        stream.read(&self.target_orientation);

        self.cacheTranslationMotorActive();
        self.cacheRotationMotorActive();
        self.cacheRotationPositionMotorActive();
    }

    pub fn getConstraintSettings(self: *const SixDOFConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try SixDOFConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(settings.asSettings());
        settings.space = .local_to_body_com;
        settings.position1 = RVec3.fromVec3(self.local_space_position1);
        settings.axis_x1 = self.constraint_to_body1.rotateAxisX();
        settings.axis_y1 = self.constraint_to_body1.rotateAxisY();
        settings.position2 = RVec3.fromVec3(self.local_space_position2);
        settings.axis_x2 = self.constraint_to_body2.rotateAxisX();
        settings.axis_y2 = self.constraint_to_body2.rotateAxisY();
        settings.swing_type = self.swing_twist_constraint_part.getSwingType();
        settings.limit_min = self.limit_min;
        settings.limit_max = self.limit_max;
        settings.max_friction = self.max_friction;
        for (0..Axis.num) |i|
            settings.motor_settings[i] = self.motor_settings[i];
        // Jolt does not copy mLimitsSpringSettings (see the file comment)
        return .init(settings.asSettings());
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const SixDOFConstraint) Mat44 {
        return Mat44.rotationTranslation(self.constraint_to_body1, self.local_space_position1);
    }

    pub fn getConstraintToBody2Matrix(self: *const SixDOFConstraint) Mat44 {
        return Mat44.rotationTranslation(self.constraint_to_body2, self.local_space_position2);
    }

    ///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
    pub inline fn getTotalLambdaPosition(self: *const SixDOFConstraint) Vec3 {
        return if (self.isTranslationFullyConstrained()) self.point_constraint_part.getTotalLambda() else Vec3.init(self.translation_constraint_part[0].getTotalLambda(), self.translation_constraint_part[1].getTotalLambda(), self.translation_constraint_part[2].getTotalLambda());
    }

    pub inline fn getTotalLambdaRotation(self: *const SixDOFConstraint) Vec3 {
        return if (self.isRotationFullyConstrained()) self.rotation_constraint_part.getTotalLambda() else Vec3.init(self.swing_twist_constraint_part.getTotalTwistLambda(), self.swing_twist_constraint_part.getTotalSwingYLambda(), self.swing_twist_constraint_part.getTotalSwingZLambda());
    }

    pub inline fn getTotalLambdaMotorTranslation(self: *const SixDOFConstraint) Vec3 {
        return Vec3.init(self.motor_translation_constraint_part[0].getTotalLambda(), self.motor_translation_constraint_part[1].getTotalLambda(), self.motor_translation_constraint_part[2].getTotalLambda());
    }

    pub inline fn getTotalLambdaMotorRotation(self: *const SixDOFConstraint) Vec3 {
        return Vec3.init(self.motor_rotation_constraint_part[0].getTotalLambda(), self.motor_rotation_constraint_part[1].getTotalLambda(), self.motor_rotation_constraint_part[2].getTotalLambda());
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig, simulations in
// ZoltTests/Physics/SixDOFConstraintTests.zig and the six_dof scenes of ZoltParity/Physics/SimulationParity.zig)

const testing = std.testing;
const Real = @import("../../Math/Real.zig").Real;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");
const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;
const TestContext = @import("ConstraintPart/ConstraintPartTestContext.zig");
const virtual = @import("../../Core/Virtual.zig");

/// The binary state of settings
fn testSaveSettings(allocator: Allocator, settings: *const ConstraintSettings) ![]u8 {
    var data: std.Io.Writer.Allocating = .init(allocator);
    defer data.deinit();
    var out = StreamWrapper.StreamOutWrapper.init(&data.writer);
    settings.saveBinaryState(out.streamOut());
    if (out.isFailed()) return error.OutOfMemory; // The writer could not grow
    return data.toOwnedSlice();
}

/// ConstraintSettings.restoreFromBinaryState of bytes
fn testRestoreSettings(allocator: Allocator, bytes: []const u8) Allocator.Error!ConstraintSettings.ConstraintResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return ConstraintSettings.restoreFromBinaryState(allocator, in.streamIn());
}

/// Settings with a non default value in every member
fn testSettings(allocator: Allocator) SixDOFConstraintSettings {
    var settings = SixDOFConstraintSettings.init(allocator);
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 7;
    settings.base.base.num_velocity_steps_override = 3;
    settings.base.base.num_position_steps_override = 5;
    settings.base.base.draw_constraint_size = 0.25;
    settings.space = .local_to_body_com;
    settings.position1 = RVec3.init(1, -2, 3);
    settings.axis_x1 = Vec3.axisZ();
    settings.axis_y1 = Vec3.axisX();
    settings.position2 = RVec3.init(-0.5, 0.25, 4);
    settings.axis_x2 = Vec3.axisY();
    settings.axis_y2 = Vec3.axisZ().negate();
    settings.max_friction = .{ 1, 2, 3, 4, 5, 6 };
    settings.swing_type = .pyramid;
    settings.limit_min = .{ -1, -0.5, 0, -0.25, -0.75, -1.5 };
    settings.limit_max = .{ 1, 0.5, 2, 0.25, 0.5, 1.25 };
    settings.limits_spring_settings = .{ .init(.stiffness_and_damping, 120, 3.5), .init(.frequency_and_damping, 2, 0.5), .init(.mass_normalized_stiffness_and_damping, 30, 1) };
    for (&settings.motor_settings, 0..) |*m, i|
        m.* = .initModeLimits(.mass_normalized_stiffness_and_damping, 40 + @as(f32, @floatFromInt(i)), 0.5, 100, 25);
    return settings;
}

test "SixDOFConstraintSettings: defaults, axis helpers, binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;

    // Jolt's defaults
    {
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        try testing.expectEqual(ConstraintSpace.world_space, settings.space);
        try testing.expect(settings.position1.eql(RVec3.zero()) and settings.position2.eql(RVec3.zero()));
        try testing.expect(settings.axis_x1.eql(Vec3.axisX()) and settings.axis_y1.eql(Vec3.axisY()));
        try testing.expect(settings.axis_x2.eql(Vec3.axisX()) and settings.axis_y2.eql(Vec3.axisY()));
        try testing.expectEqual([6]f32{ 0, 0, 0, 0, 0, 0 }, settings.max_friction);
        try testing.expectEqual(SwingType.cone, settings.swing_type);
        for (0..SixDOFConstraintSettings.Axis.num) |a| {
            const axis: SixDOFConstraintSettings.Axis = @enumFromInt(a);
            try testing.expectEqual(-math.flt_max, settings.limit_min[a]);
            try testing.expectEqual(math.flt_max, settings.limit_max[a]);
            try testing.expect(settings.isFreeAxis(axis) and !settings.isFixedAxis(axis));
            try testing.expectEqual(MotorSettings{}, settings.motor_settings[a]);
        }
        for (settings.limits_spring_settings) |s| try testing.expectEqual(SpringSettings{}, s);
        try testing.expectEqual(@as(usize, 6), SixDOFConstraintSettings.Axis.num);
        try testing.expectEqual(@as(usize, 3), SixDOFConstraintSettings.Axis.num_translation);
        try testing.expectEqualStrings("SixDOFConstraintSettings", settings.asSettings().vtable.rtti.name);

        // The axis helpers
        settings.makeFixedAxis(.translation_y);
        try testing.expectEqual(math.flt_max, settings.limit_min[1]);
        try testing.expectEqual(-math.flt_max, settings.limit_max[1]);
        try testing.expect(settings.isFixedAxis(.translation_y) and !settings.isFreeAxis(.translation_y));
        settings.setLimitedAxis(.rotation_z, -0.5, 0.75);
        try testing.expectEqual(@as(f32, -0.5), settings.limit_min[5]);
        try testing.expectEqual(@as(f32, 0.75), settings.limit_max[5]);
        try testing.expect(!settings.isFixedAxis(.rotation_z) and !settings.isFreeAxis(.rotation_z));
        settings.setLimitedAxis(.rotation_x, 0.25, 0.25); // min >= max: fixed
        try testing.expect(settings.isFixedAxis(.rotation_x) and !settings.isFreeAxis(.rotation_x));
        settings.setLimitedAxis(.rotation_y, 0.5, -0.5); // Inverted: fixed
        try testing.expect(settings.isFixedAxis(.rotation_y));
        settings.setLimitedAxis(.translation_z, -math.flt_max, 1.0e30); // Not exactly FLT_MAX: limited
        try testing.expect(!settings.isFreeAxis(.translation_z) and !settings.isFixedAxis(.translation_z));
        settings.makeFreeAxis(.translation_y);
        settings.makeFreeAxis(.rotation_x);
        try testing.expect(settings.isFreeAxis(.translation_y) and settings.isFreeAxis(.rotation_x));
        try testing.expect(!settings.isFixedAxis(.translation_y));
    }

    var settings = testSettings(allocator);
    defer settings.asSettings().deinit();
    settings.base.base.user_data = 99; // Not part of the binary state (Jolt)

    // ConstraintSettings (21 bytes), space (int), the positions (Real), the axes, the max friction (6 floats), the swing
    // type (byte), limit min / max (6 floats each), the limits springs (mode byte, 2 floats), the motors (their spring,
    // 4 limits)
    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    const after_axes = 25 + 2 * 3 * @sizeOf(Real) + 4 * 12;
    try testing.expectEqual(@as(usize, after_axes + 24 + 1 + 2 * 24 + 3 * 9 + 6 * (9 + 16)), bytes.len);
    try testing.expectEqual(RTTI.hashName("SixDOFConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(i32, 0), std.mem.readInt(i32, bytes[21..25], .little)); // LocalToBodyCOM
    try testing.expectEqual(@as(u32, @bitCast(@as(f32, 1))), std.mem.readInt(u32, bytes[after_axes..][0..4], .little)); // Max friction
    try testing.expectEqual(@as(u8, 1), bytes[after_axes + 24]); // Swing type: Pyramid
    try testing.expectEqual(@as(u32, @bitCast(@as(f32, -1))), std.mem.readInt(u32, bytes[after_axes + 25 ..][0..4], .little)); // Limit min
    try testing.expectEqual(@as(u32, @bitCast(@as(f32, 1.25))), std.mem.readInt(u32, bytes[after_axes + 25 + 44 ..][0..4], .little)); // Limit max of rotation Z
    try testing.expectEqual(@as(u8, 1), bytes[after_axes + 73]); // Limits spring 0: StiffnessAndDamping

    // Restore through RegisterTypes.factory
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(SixDOFConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(!restored.base.base.enabled);
        try testing.expectEqual(@as(u32, 7), restored.base.base.constraint_priority);
        try testing.expectEqual(@as(u32, 3), restored.base.base.num_velocity_steps_override);
        try testing.expectEqual(@as(u32, 5), restored.base.base.num_position_steps_override);
        try testing.expectEqual(@as(f32, 0.25), restored.base.base.draw_constraint_size);
        try testing.expectEqual(@as(u64, 0), restored.base.base.user_data);
        try testing.expectEqual(ConstraintSpace.local_to_body_com, restored.space);
        try testing.expect(restored.position1.eql(settings.position1) and restored.position2.eql(settings.position2));
        try testing.expect(restored.axis_x1.eql(settings.axis_x1) and restored.axis_y1.eql(settings.axis_y1));
        try testing.expect(restored.axis_x2.eql(settings.axis_x2) and restored.axis_y2.eql(settings.axis_y2));
        try testing.expectEqual(settings.max_friction, restored.max_friction);
        try testing.expectEqual(SwingType.pyramid, restored.swing_type);
        try testing.expectEqual(settings.limit_min, restored.limit_min);
        try testing.expectEqual(settings.limit_max, restored.limit_max);
        try testing.expectEqual(settings.limits_spring_settings, restored.limits_spring_settings);
        try testing.expectEqual(settings.motor_settings, restored.motor_settings);
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, bytes, resaved);
    }

    // A swing type byte outside the named values is restored and saved as it is (ESwingType accepts every uint8), and
    // a constraint made from it behaves like Pyramid (Jolt only compares with Cone)
    {
        const odd = try allocator.dupe(u8, bytes);
        defer allocator.free(odd);
        odd[after_axes + 24] = 7;
        var result = try testRestoreSettings(allocator, odd);
        defer result.deinit();
        const restored = virtual.downcast(SixDOFConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expectEqual(@as(u8, 7), @intFromEnum(restored.swing_type));
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, odd, resaved);

        var context: TestContext.ConstraintPartTestContext = undefined;
        try context.init(allocator, testing.io);
        defer context.deinit();
        const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
        defer context.destroyBodies(&bodies);
        restored.limit_min[4] = -0.2; // Asymmetric, Cone would make it symmetric
        restored.limit_max[4] = 0.6;
        var c = SixDOFConstraint.init(allocator, bodies[0], bodies[1], restored);
        defer c.asConstraint().deinit();
        try testing.expectEqual(@as(f32, -0.2), c.getLimitsMin(.rotation_y));
        try testing.expectEqual(@as(f32, 0.6), c.getLimitsMax(.rotation_y));
    }

    // Truncated anywhere: Jolt's error
    for ([_]usize{ 1, 9, 17, 30, bytes.len - after_axes - 20, bytes.len - 22 }) |cut| {
        var result = try testRestoreSettings(allocator, bytes[0 .. bytes.len - cut]);
        defer result.deinit();
        try testing.expectEqualStrings("Failed to restore object", result.getError());
    }

    // Out of memory while restoring: the error arrives, nothing leaks
    const Ctx = struct {
        fn run(a: Allocator, b: []const u8) !void {
            var result = try testRestoreSettings(a, b);
            result.deinit();
        }
    };
    try testing.checkAllAllocationFailures(allocator, Ctx.run, .{bytes});

    // The copy constructor copies the settings, not the reference count
    settings.asSettings().setEmbedded();
    settings.asSettings().addRef();
    defer settings.asSettings().release();
    var copy = SixDOFConstraintSettings.initCopy(allocator, &settings);
    defer copy.asSettings().deinit();
    try testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try testing.expect(copy.position2.eql(settings.position2) and copy.axis_y2.eql(settings.axis_y2));
    try testing.expectEqual(settings.limit_min, copy.limit_min);
    try testing.expectEqual(settings.motor_settings, copy.motor_settings);
    try testing.expectEqual(settings.limits_spring_settings, copy.limits_spring_settings);
    try testing.expectEqual(@as(u64, 99), copy.base.base.user_data);
}

/// The quaternion of the frame with the given X and Y axes (Z = X x Y), as the constructor builds it
fn testFrame(axis_x: Vec3, axis_y: Vec3) Quat {
    return Mat44.init(Vec4.fromVec3W(axis_x, 0), Vec4.fromVec3W(axis_y, 0), Vec4.fromVec3W(axis_x.cross(axis_y), 0), Vec4.init(0, 0, 0, 1)).getQuaternion();
}

/// q or -q (the same rotation)
fn sameRotation(a: Quat, b: Quat) bool {
    return a.isClose(b, .{ .max_dist_sq = 1.0e-10 }) or a.isClose(b.negate(), .{ .max_dist_sq = 1.0e-10 });
}

test "SixDOFConstraint: virtual dispatch, space conversions, settings and state" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);
    const body1 = bodies[0];
    const body2 = bodies[1];

    // World space
    var settings = SixDOFConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.position1 = RVec3.init(1.5, 2.0, 3.5);
    settings.position2 = RVec3.init(1.25, 2.0, 3.75);
    settings.axis_x1 = Vec3.axisY();
    settings.axis_y1 = Vec3.axisX();
    settings.axis_x2 = Vec3.axisZ();
    settings.axis_y2 = Vec3.axisY().negate();
    settings.makeFixedAxis(.translation_x);
    settings.setLimitedAxis(.translation_y, -0.5, 0.5);
    settings.setLimitedAxis(.rotation_x, -0.3, 0.6);
    settings.setLimitedAxis(.rotation_y, -0.2, 0.4); // Cone: made symmetric (-0.4, 0.4)
    settings.makeFixedAxis(.rotation_z);
    settings.max_friction = .{ 0, 0, 2, 0, 0, 0 };
    settings.limits_spring_settings[1] = .init(.frequency_and_damping, 3, 0.5);
    settings.motor_settings[3] = .initLimits(5, 0.5, 10, 20);
    settings.base.base.user_data = 77;

    // TwoBodyConstraintSettings::Create through the vtable
    var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
    defer constraint.deinit();
    const base = constraint.get().?;
    const two_body = virtual.downcast(TwoBodyConstraint, base);
    const c = virtual.downcast(SixDOFConstraint, two_body);
    try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
    try testing.expectEqual(ConstraintSubType.six_dof, base.getSubType());
    try testing.expectEqual(@as(u64, 77), base.getUserData());
    try testing.expect(two_body.getBody1() == body1 and two_body.getBody2() == body2);

    // The constructor took the positions and frames to the local space of the bodies
    try testing.expect(c.local_space_position1.eql(body1.getInverseCenterOfMassTransform().mulRVec3(settings.position1).toVec3()));
    try testing.expect(c.local_space_position2.eql(body2.getInverseCenterOfMassTransform().mulRVec3(settings.position2).toVec3()));
    try testing.expect(c.constraint_to_body1.eql(body1.getRotation().conjugated().mul(testFrame(settings.axis_x1, settings.axis_y1))));
    try testing.expect(c.constraint_to_body2.eql(body2.getRotation().conjugated().mul(testFrame(settings.axis_x2, settings.axis_y2))));
    try testing.expect(two_body.getConstraintToBody1Matrix().eql(Mat44.rotationTranslation(c.constraint_to_body1, c.local_space_position1)));
    try testing.expect(two_body.getConstraintToBody2Matrix().eql(Mat44.rotationTranslation(c.constraint_to_body2, c.local_space_position2)));

    // GetRotationInConstraintSpace: (q1 c1)^-1 (q2 c2); in world space both frames are the world frames of the settings
    const q = c.getRotationInConstraintSpace();
    try testing.expect(q.eql(body1.getRotation().mul(c.constraint_to_body1).conjugated().mul(body2.getRotation()).mul(c.constraint_to_body2)));
    try testing.expect(sameRotation(q, testFrame(settings.axis_x1, settings.axis_y1).conjugated().mul(testFrame(settings.axis_x2, settings.axis_y2))));

    // The settings values (sanitized limits, the fixed / free bookkeeping)
    try testing.expectEqual(@as(f32, 0.0), c.getLimitsMin(.translation_x)); // Fixed: inverted limits become 0
    try testing.expectEqual(@as(f32, 0.0), c.getLimitsMax(.translation_x));
    try testing.expect(c.getTranslationLimitsMin().eql(Vec3.init(0, -0.5, -math.flt_max)));
    try testing.expect(c.getTranslationLimitsMax().eql(Vec3.init(0, 0.5, math.flt_max)));
    try testing.expect(c.getRotationLimitsMin().eql(Vec3.init(-0.3, -0.4, 0)));
    try testing.expect(c.getRotationLimitsMax().eql(Vec3.init(0.6, 0.4, 0)));
    try testing.expect(c.isFixedAxis(.translation_x) and !c.isFreeAxis(.translation_x));
    try testing.expect(!c.isFixedAxis(.translation_y) and !c.isFreeAxis(.translation_y));
    try testing.expect(!c.isFixedAxis(.translation_z) and c.isFreeAxis(.translation_z));
    try testing.expect(!c.isFixedAxis(.rotation_x) and !c.isFreeAxis(.rotation_x));
    try testing.expect(c.isFixedAxis(.rotation_z));
    try testing.expectEqual(@as(u8, 0b100001), c.fixed_axis);
    try testing.expectEqual(@as(u8, 0b000100), c.free_axis);
    try testing.expect(c.has_spring_limits);
    try testing.expect(c.translation_motor_active); // Friction on translation Z
    try testing.expect(!c.rotation_motor_active);
    try testing.expectEqual(@as(f32, 2.0), c.getMaxFriction(.translation_z));
    try testing.expectEqual(settings.limits_spring_settings[1], c.getLimitsSpringSettings(.translation_y).*);
    try testing.expectEqual(settings.motor_settings[3], c.getMotorSettings(.rotation_x).*);
    for (0..6) |a| try testing.expectEqual(MotorState.off, c.getMotorState(@enumFromInt(a)));
    try testing.expect(c.getTargetVelocityCS().eql(Vec3.zero()) and c.getTargetAngularVelocityCS().eql(Vec3.zero()));
    try testing.expect(c.getTargetPositionCS().eql(Vec3.zero()) and c.getTargetOrientationCS().eql(Quat.identity()));

    // GetConstraintSettings: local space, the sanitized limits; Jolt does not copy the limits springs
    {
        c.getMotorSettingsMut(.rotation_y).* = .initLimits(7, 0.25, 30, 40);
        c.setMotorState(.rotation_y, .velocity); // Not part of the settings
        defer c.setMotorState(.rotation_y, .off);
        var copy = try base.getConstraintSettings();
        defer copy.deinit();
        const s = virtual.downcast(SixDOFConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, copy.get().?));
        try testing.expectEqual(ConstraintSpace.local_to_body_com, s.space);
        try testing.expectEqual(@as(u64, 77), s.base.base.user_data);
        try testing.expect(s.position1.eql(RVec3.fromVec3(c.local_space_position1)) and s.position2.eql(RVec3.fromVec3(c.local_space_position2)));
        try testing.expect(s.axis_x1.eql(c.constraint_to_body1.rotateAxisX()) and s.axis_y2.eql(c.constraint_to_body2.rotateAxisY()));
        try testing.expectEqual(SwingType.cone, s.swing_type);
        try testing.expectEqual(c.limit_min, s.limit_min);
        try testing.expectEqual(c.limit_max, s.limit_max);
        try testing.expectEqual(c.max_friction, s.max_friction);
        try testing.expectEqual(c.motor_settings, s.motor_settings);
        for (s.limits_spring_settings) |spring| try testing.expectEqual(SpringSettings{}, spring); // Jolt's omission
        var again = Ref(Constraint).init(&(try s.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer again.deinit();
        const c2 = virtual.downcast(SixDOFConstraint, virtual.downcast(TwoBodyConstraint, again.get().?));
        try testing.expect(c2.local_space_position1.eql(c.local_space_position1) and c2.local_space_position2.eql(c.local_space_position2));
        try testing.expect(sameRotation(c2.constraint_to_body1, c.constraint_to_body1) and sameRotation(c2.constraint_to_body2, c.constraint_to_body2));
        try testing.expectEqual(c.limit_min, c2.limit_min);
        try testing.expectEqual(c.fixed_axis, c2.fixed_axis);
        try testing.expectEqual(c.free_axis, c2.free_axis);
        try testing.expect(!c2.has_spring_limits);
        try testing.expectEqual(MotorState.off, c2.getMotorState(.rotation_y));
    }

    // SaveState / RestoreState: enabled, the lambdas of every part, the motor states and targets
    base.setupVelocityConstraint(1.0 / 60.0);
    base.warmStartVelocityConstraint(1.0);
    for (0..4) |_| _ = base.solveVelocityConstraint(1.0 / 60.0);
    c.setMotorState(.translation_y, .position);
    c.setMotorState(.rotation_x, .position_and_velocity);
    c.setTargetVelocityCS(Vec3.init(1, 2, 3));
    c.setTargetAngularVelocityCS(Vec3.init(-1, 0.5, 0.25));
    c.setTargetPositionCS(Vec3.init(0.1, 0.2, 0.3));
    c.setTargetOrientationCS(Quat.rotation(Vec3.axisX(), 0.2));
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    base.saveState(recorder.recorder());
    try testing.expectEqual(@as(usize, 1 + 3 * 4 + 12 + 3 * 4 + 12 + 3 * 4 + 3 * 4 + 6 * 4 + 3 * 12 + 16), recorder.getData().len);
    const lambda_position = c.getTotalLambdaPosition();
    const lambda_rotation = c.getTotalLambdaRotation();
    const lambda_motor_translation = c.getTotalLambdaMotorTranslation();
    try testing.expect(!lambda_position.eql(Vec3.zero()) and !lambda_rotation.eql(Vec3.zero()));
    try testing.expect(lambda_motor_translation.getZ() != 0.0); // Friction
    base.resetWarmStart();
    c.setMotorState(.translation_y, .off);
    c.setMotorState(.rotation_x, .off);
    c.setTargetVelocityCS(Vec3.zero());
    c.setTargetAngularVelocityCS(Vec3.zero());
    c.setTargetPositionCS(Vec3.zero());
    c.setTargetOrientationCS(Quat.identity());
    try testing.expect(c.getTotalLambdaPosition().eql(Vec3.zero()) and c.getTotalLambdaRotation().eql(Vec3.zero()));
    try testing.expect(c.getTotalLambdaMotorTranslation().eql(Vec3.zero()) and c.getTotalLambdaMotorRotation().eql(Vec3.zero()));
    try testing.expectEqual(@as(u8, 0), c.rotation_position_motor_active);
    base.restoreState(recorder.recorder());
    try testing.expect(c.getTotalLambdaPosition().eql(lambda_position));
    try testing.expect(c.getTotalLambdaRotation().eql(lambda_rotation));
    try testing.expect(c.getTotalLambdaMotorTranslation().eql(lambda_motor_translation));
    try testing.expectEqual(MotorState.position, c.getMotorState(.translation_y));
    try testing.expectEqual(MotorState.position_and_velocity, c.getMotorState(.rotation_x));
    try testing.expect(c.getTargetVelocityCS().eql(Vec3.init(1, 2, 3)));
    try testing.expect(c.getTargetAngularVelocityCS().eql(Vec3.init(-1, 0.5, 0.25)));
    try testing.expect(c.getTargetPositionCS().eql(Vec3.init(0.1, 0.2, 0.3)));
    try testing.expect(c.getTargetOrientationCS().eql(Quat.rotation(Vec3.axisX(), 0.2)));
    // The caches are rebuilt from the restored motor states
    try testing.expect(c.translation_motor_active and c.rotation_motor_active);
    try testing.expectEqual(@as(u8, 0b001), c.rotation_position_motor_active);

    // NotifyShapeChanged moves the attachment point of the body that changed
    const p1 = c.local_space_position1;
    const p2 = c.local_space_position2;
    base.notifyShapeChanged(body2.getID(), Vec3.init(0.5, 0, 0));
    try testing.expect(c.local_space_position1.eql(p1) and c.local_space_position2.eql(p2.sub(Vec3.init(0.5, 0, 0))));
    base.notifyShapeChanged(body1.getID(), Vec3.init(0, 1, 0));
    try testing.expect(c.local_space_position1.eql(p1.sub(Vec3.init(0, 1, 0))));
    base.notifyShapeChanged(BodyID.init(12345), Vec3.init(1, 1, 1));
    try testing.expect(c.local_space_position1.eql(p1.sub(Vec3.init(0, 1, 0))) and c.local_space_position2.eql(p2.sub(Vec3.init(0.5, 0, 0))));

    // A disabled constraint is not active (TwoBodyConstraint::IsActive)
    try testing.expect(base.isActive());
    base.setEnabled(false);
    try testing.expect(!base.isActive());
}

test "SixDOFConstraint: local space settings and the limits bookkeeping" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .kinematic, .dynamic });
    defer context.destroyBodies(&bodies);

    // Local space: the positions are used as they are, the frames are the settings' frames
    var settings = SixDOFConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.space = .local_to_body_com;
    settings.position1 = RVec3.init(0.25, 0.5, 0.75);
    settings.position2 = RVec3.init(-1, -2, -3);
    settings.axis_x1 = Vec3.axisZ();
    settings.axis_y1 = Vec3.axisX();
    // Inverted translation limits become 0 (fixed), rotation limits are clamped to [-pi, pi], the cone swing limits
    // are made symmetric with a positive max
    settings.limit_min = .{ 1, -0.5, -math.flt_max, -5, -0.25, 0.5 };
    settings.limit_max = .{ -1, 0.5, math.flt_max, 5, -0.75, 4 };
    var c = SixDOFConstraint.init(allocator, bodies[0], bodies[1], &settings);
    defer c.asConstraint().deinit();
    try testing.expect(c.local_space_position1.eql(Vec3.init(0.25, 0.5, 0.75)) and c.local_space_position2.eql(Vec3.init(-1, -2, -3)));
    try testing.expect(c.constraint_to_body1.eql(testFrame(Vec3.axisZ(), Vec3.axisX())));
    try testing.expect(c.constraint_to_body2.eql(Quat.identity()));
    try testing.expectEqual([6]f32{ 0, -0.5, -math.flt_max, -math.pi, -0.0, -math.pi }, c.limit_min);
    try testing.expectEqual([6]f32{ 0, 0.5, math.flt_max, math.pi, 0, math.pi }, c.limit_max);
    // Rotation X [-pi, pi] is free (limit pi), rotation Y [0, 0] is fixed, rotation Z [-pi, pi] is free
    try testing.expectEqual(@as(u8, 0b010001), c.fixed_axis);
    try testing.expectEqual(@as(u8, 0b101100), c.free_axis);
    try testing.expect(!c.has_spring_limits and !c.translation_motor_active and !c.rotation_motor_active);

    // Pyramid: the swing limits stay asymmetric
    settings.swing_type = .pyramid;
    var p = SixDOFConstraint.init(allocator, bodies[0], bodies[1], &settings);
    defer p.asConstraint().deinit();
    try testing.expectEqual(SwingType.pyramid, p.swing_twist_constraint_part.getSwingType());
    try testing.expectEqual(@as(f32, 0.0), p.getLimitsMin(.rotation_y)); // -0.25 > -0.75: inverted, both 0
    try testing.expectEqual(@as(f32, 0.0), p.getLimitsMax(.rotation_y));
    try testing.expectEqual(@as(f32, 0.5), p.getLimitsMin(.rotation_z));
    try testing.expectEqual(math.pi, p.getLimitsMax(.rotation_z));
    try testing.expect(!p.isFreeAxis(.rotation_z) and !p.isFixedAxis(.rotation_z));

    // SetTranslationLimits / SetRotationLimits sanitize the same way; a change of the fixed / free axes deactivates
    // every part (warm starting restarts)
    c.translation_constraint_part[1].total_lambda = 1.0;
    c.motor_rotation_constraint_part[2].total_lambda = 2.0;
    c.setTranslationLimits(Vec3.init(-1, -0.5, 2), Vec3.init(1, 0.5, 1)); // X becomes limited, Z fixed
    try testing.expect(c.getTranslationLimitsMin().eql(Vec3.init(-1, -0.5, 0)));
    try testing.expect(c.getTranslationLimitsMax().eql(Vec3.init(1, 0.5, 0)));
    try testing.expectEqual(@as(u8, 0b010100), c.fixed_axis);
    try testing.expectEqual(@as(u8, 0b101000), c.free_axis);
    try testing.expectEqual(@as(f32, 0.0), c.translation_constraint_part[1].total_lambda);
    try testing.expectEqual(@as(f32, 0.0), c.motor_rotation_constraint_part[2].total_lambda);

    // No change of the fixed / free axes: the parts keep their lambdas
    c.translation_constraint_part[1].total_lambda = 1.0;
    c.setTranslationLimits(Vec3.init(-2, -0.25, 3), Vec3.init(2, 0.25, 1));
    try testing.expectEqual(@as(f32, 1.0), c.translation_constraint_part[1].total_lambda);
    c.setRotationLimits(Vec3.init(-0.5, -0.3, -0.2), Vec3.init(0.5, 0.3, 0.2));
    try testing.expectEqual(@as(f32, 0.0), c.translation_constraint_part[1].total_lambda); // Rotation X / Z no longer free
    try testing.expect(c.getRotationLimitsMin().eql(Vec3.init(-0.5, -0.3, -0.2)));
    try testing.expect(c.getRotationLimitsMax().eql(Vec3.init(0.5, 0.3, 0.2)));
    try testing.expectEqual(@as(u8, 0b000100), c.fixed_axis);
    try testing.expectEqual(@as(u8, 0b000000), c.free_axis);

    // Cone: the min is the negated max (the min given is ignored), a negative max becomes 0 (fixed)
    c.setRotationLimits(Vec3.init(-4, 0.1, -0.9), Vec3.init(4, -0.3, 0.4));
    try testing.expect(c.getRotationLimitsMin().eql(Vec3.init(-math.pi, -0.0, -0.4)));
    try testing.expect(c.getRotationLimitsMax().eql(Vec3.init(math.pi, 0, 0.4)));
    try testing.expect(c.isFreeAxis(.rotation_x) and c.isFixedAxis(.rotation_y) and !c.isFixedAxis(.rotation_z));

    // The limits springs: only a positive frequency / stiffness makes a soft limit
    c.setLimitsSpringSettings(.translation_z, .init(.stiffness_and_damping, 0, 5));
    try testing.expect(!c.has_spring_limits);
    c.setLimitsSpringSettings(.translation_z, .init(.stiffness_and_damping, 10, 0));
    try testing.expect(c.has_spring_limits);
    try testing.expectEqual(@as(f32, 10), c.getLimitsSpringSettings(.translation_z).getStiffness());
    c.setLimitsSpringSettings(.translation_z, .{});
    try testing.expect(!c.has_spring_limits);

    // Friction: a fixed axis has no friction
    c.setMaxFriction(.rotation_y, 1.0); // Fixed
    try testing.expect(!c.rotation_motor_active);
    c.setMaxFriction(.rotation_z, 1.0);
    try testing.expect(c.rotation_motor_active and !c.translation_motor_active);
    c.setMaxFriction(.translation_z, 1.0); // Fixed
    try testing.expect(!c.translation_motor_active);
    c.setMaxFriction(.translation_y, 0.5);
    try testing.expect(c.translation_motor_active);
    try testing.expectEqual(@as(f32, 0.5), c.getMaxFriction(.translation_y));
    c.setMaxFriction(.translation_y, 0.0);
    c.setMaxFriction(.rotation_z, 0.0);
    try testing.expect(!c.translation_motor_active and !c.rotation_motor_active);
}

test "SixDOFConstraint: motor states and target orientation" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .static, .dynamic });
    defer context.destroyBodies(&bodies);

    var settings = SixDOFConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.setLimitedAxis(.rotation_x, -0.5, 0.5);
    settings.setLimitedAxis(.rotation_y, -0.4, 0.4);
    settings.setLimitedAxis(.rotation_z, -0.3, 0.3);
    var c = SixDOFConstraint.init(allocator, bodies[0], bodies[1], &settings);
    defer c.asConstraint().deinit();

    // SetMotorState: the motor parts of the axis are deactivated on a change, the caches follow
    c.motor_translation_constraint_part[1].total_lambda = 1.0;
    c.setMotorState(.translation_y, .velocity);
    try testing.expectEqual(@as(f32, 0.0), c.motor_translation_constraint_part[1].total_lambda);
    try testing.expect(c.translation_motor_active and !c.rotation_motor_active);
    c.motor_translation_constraint_part[1].total_lambda = 1.0;
    c.setMotorState(.translation_y, .velocity); // No change: nothing happens
    try testing.expectEqual(@as(f32, 1.0), c.motor_translation_constraint_part[1].total_lambda);
    c.setMotorState(.translation_y, .off);
    try testing.expect(!c.translation_motor_active);

    for ([_]MotorState{ .velocity, .position, .position_and_velocity }) |state| {
        for (0..3) |i| {
            const axis: SixDOFConstraint.Axis = @enumFromInt(3 + i);
            c.motor_rotation_constraint_part[i].total_lambda = 1.0;
            c.setMotorState(axis, state);
            try testing.expectEqual(@as(f32, 0.0), c.motor_rotation_constraint_part[i].total_lambda);
            try testing.expectEqual(state, c.getMotorState(axis));
            try testing.expect(c.rotation_motor_active);
            const expected_bits: u8 = if (state == .velocity) 0 else @as(u8, 1) << @intCast(i);
            try testing.expectEqual(expected_bits, c.rotation_position_motor_active);
            c.setMotorState(axis, .off);
            try testing.expect(!c.rotation_motor_active);
            try testing.expectEqual(@as(u8, 0), c.rotation_position_motor_active);
        }
    }
    c.setMotorState(.rotation_x, .position);
    c.setMotorState(.rotation_z, .position_and_velocity);
    try testing.expectEqual(@as(u8, 0b101), c.rotation_position_motor_active);
    c.setMotorState(.rotation_x, .off);
    c.setMotorState(.rotation_z, .off);

    // SetTargetOrientationCS: within the limits the orientation is kept as it is, outside it is clamped to the limits
    const inside = Quat.rotation(Vec3.axisX(), 0.25).mul(Quat.rotation(Vec3.axisY(), 0.1));
    c.setTargetOrientationCS(inside);
    try testing.expect(c.getTargetOrientationCS().eql(inside));
    c.setTargetOrientationCS(Quat.rotation(Vec3.axisX(), 1.2));
    try testing.expect(@abs(c.getTargetOrientationCS().getRotationAngle(Vec3.axisX()) - 0.5) < 1.0e-5);
    const outside = Quat.rotation(Vec3.axisZ(), 0.9);
    const swing_twist = outside.getSwingTwist();
    var q_swing = swing_twist.swing;
    var q_twist = swing_twist.twist;
    try testing.expect(c.swing_twist_constraint_part.clampSwingTwist(&q_swing, &q_twist) != 0);
    c.setTargetOrientationCS(outside);
    try testing.expect(c.getTargetOrientationCS().eql(q_swing.mul(q_twist)));

    // SetTargetOrientationBS: R2 = R1 * orientation in constraint space is c1^-1 * orientation * c2 (then clamped)
    const bs = Quat.rotation(Vec3.init(1, 1, 0).normalized(), 0.2);
    c.setTargetOrientationBS(bs);
    const via_bs = c.getTargetOrientationCS();
    c.setTargetOrientationCS(c.constraint_to_body1.conjugated().mul(bs).mul(c.constraint_to_body2));
    try testing.expect(c.getTargetOrientationCS().eql(via_bs));
    // The current relative orientation of the bodies is the identity in constraint space (the frames coincide)
    c.setTargetOrientationBS(bodies[0].getRotation().conjugated().mul(bodies[1].getRotation()));
    try testing.expect(sameRotation(c.getTargetOrientationCS(), Quat.identity()));
}

/// A dynamic box at the origin attached to the world (body 1 is Body::sFixedToWorld) with a SixDOF at its center of
/// mass, the frames of both bodies the world frame
const TestSixDOF = struct {
    body: *Body,
    c: SixDOFConstraint,

    fn init(self: *TestSixDOF, context: *TestContext.ConstraintPartTestContext, allocator: Allocator, settings: *const SixDOFConstraintSettings, linear_velocity: Vec3, angular_velocity: Vec3) !void {
        self.body = try context.createBody(.dynamic, .{ .linear_velocity = linear_velocity, .angular_velocity = angular_velocity, .half_extent = Vec3.init(0.5, 0.5, 0.5) });
        self.c = SixDOFConstraint.init(allocator, Body.fixedToWorld(), self.body, settings);
    }

    fn deinit(self: *TestSixDOF, context: *TestContext.ConstraintPartTestContext) void {
        self.c.asConstraint().deinit();
        context.destroyBodies(&.{self.body});
    }
};

test "SixDOFConstraint: solver with fixed, limited and free axes, springs, motors and friction" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const dt: f32 = 1.0 / 60.0;

    // Every axis free: nothing acts
    {
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        var t: TestSixDOF = undefined;
        try t.init(&context, allocator, &settings, Vec3.init(1, 2, 3), Vec3.init(0.5, -0.5, 0.25));
        defer t.deinit(&context);
        const base = t.c.asConstraint();
        base.setupVelocityConstraint(dt);
        base.warmStartVelocityConstraint(1.0);
        try testing.expect(!base.solveVelocityConstraint(dt));
        try testing.expect(!base.solvePositionConstraint(dt, 0.2));
        try testing.expect(t.body.getLinearVelocity().eql(Vec3.init(1, 2, 3)));
    }

    // Every axis fixed: the point and rotation (Euler) parts act, the body stops
    {
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        for (0..6) |a| settings.makeFixedAxis(@enumFromInt(a));
        var t: TestSixDOF = undefined;
        try t.init(&context, allocator, &settings, Vec3.init(1, 2, 3), Vec3.init(0.5, -0.5, 0.25));
        defer t.deinit(&context);
        const base = t.c.asConstraint();
        try testing.expect(t.c.isTranslationFullyConstrained() and t.c.isRotationFullyConstrained());
        base.setupVelocityConstraint(dt);
        try testing.expect(t.c.point_constraint_part.isActive() and t.c.rotation_constraint_part.isActive());
        base.warmStartVelocityConstraint(1.0);
        for (0..5) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(t.body.getLinearVelocity().isNearZero(.{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(t.body.getAngularVelocity().isNearZero(.{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(t.c.getTotalLambdaPosition().eql(t.c.point_constraint_part.getTotalLambda()));
        try testing.expect(t.c.getTotalLambdaRotation().eql(t.c.rotation_constraint_part.getTotalLambda()));

        // A fixed axis with a value (SetLimitedAxis(min = max)): the position solver drives towards it
        t.c.setTranslationLimits(Vec3.init(0.5, 0, 0), Vec3.init(0.5, 0, 0));
        try testing.expect(t.c.isTranslationFullyConstrained());
        for (0..20) |_| _ = base.solvePositionConstraint(dt, 0.5);
        try testing.expect(@abs(t.body.getCenterOfMassPosition().toVec3().getX() - 0.5) < 1.0e-3);

        // Rotation fixed at an angle: the position solver rotates there
        t.c.setRotationLimits(Vec3.init(0.2, 0, 0), Vec3.init(0.2, 0, 0));
        try testing.expect(t.c.isRotationFullyConstrained());
        for (0..30) |_| _ = base.solvePositionConstraint(dt, 0.5);
        try testing.expect(@abs(t.body.getRotation().getRotationAngle(Vec3.axisX()) - 0.2) < 1.0e-3);

        // A spring on a fixed translation axis: the axis parts are used instead of the point part
        t.c.setLimitsSpringSettings(.translation_x, .init(.frequency_and_damping, 2, 0.5));
        try testing.expect(!t.c.isTranslationFullyConstrained());
        base.setupVelocityConstraint(dt);
        for (0..3) |i| try testing.expect(t.c.translation_constraint_part[i].isActive());
    }

    // Limited translation: inactive inside the limits, pushes back outside (only on one side)
    for ([_]f32{ 0.8, -0.8 }) |offset| {
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.setLimitedAxis(.translation_y, -0.5, 0.5);
        var t: TestSixDOF = undefined;
        try t.init(&context, allocator, &settings, Vec3.init(0, offset, 0), Vec3.zero());
        defer t.deinit(&context);
        const base = t.c.asConstraint();
        base.setupVelocityConstraint(dt);
        try testing.expect(!t.c.translation_constraint_part[1].isActive());
        try testing.expect(!t.c.translation_constraint_part[0].isActive() and !t.c.translation_constraint_part[2].isActive());

        // Move it past the limit, moving away
        t.body.setPositionAndRotationInternal(RVec3.init(0, offset, 0), Quat.identity(), .{});
        base.setupVelocityConstraint(dt);
        try testing.expect(t.c.translation_constraint_part[1].isActive());
        try testing.expectEqual(offset, t.c.displacement[1]);
        base.warmStartVelocityConstraint(1.0);
        for (0..5) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(t.body.getLinearVelocity().getY() * offset <= 1.0e-5); // Not moving further out
        try testing.expect(t.c.getTotalLambdaPosition().getY() * offset < 0.0);

        // The position solver moves it back towards the limit
        try testing.expect(base.solvePositionConstraint(dt, 0.5));
        try testing.expect(@abs(t.body.getPosition().toVec3().getY()) < 0.8);

        // A soft limit is not corrected in the position solver
        t.c.setLimitsSpringSettings(.translation_y, .init(.frequency_and_damping, 2, 0.5));
        const before = t.body.getPosition();
        try testing.expect(!base.solvePositionConstraint(dt, 0.5));
        try testing.expect(t.body.getPosition().eql(before));
    }

    // Limited rotation: the swing twist part acts when outside the limits
    for ([_]SwingType{ .cone, .pyramid }) |swing_type| {
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.swing_type = swing_type;
        for (0..3) |a| settings.makeFixedAxis(@enumFromInt(a));
        settings.setLimitedAxis(.rotation_x, -0.2, 0.2);
        settings.setLimitedAxis(.rotation_y, -0.3, 0.3);
        settings.setLimitedAxis(.rotation_z, -0.3, 0.3);
        var t: TestSixDOF = undefined;
        try t.init(&context, allocator, &settings, Vec3.zero(), Vec3.init(2, 0, 0));
        defer t.deinit(&context);
        const base = t.c.asConstraint();
        t.body.setPositionAndRotationInternal(RVec3.zero(), Quat.rotation(Vec3.axisX(), 0.5), .{});
        base.setupVelocityConstraint(dt);
        try testing.expect(t.c.swing_twist_constraint_part.isActive());
        base.warmStartVelocityConstraint(1.0);
        for (0..5) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(t.body.getAngularVelocity().getX() <= 1.0e-5);
        try testing.expect(t.c.getTotalLambdaRotation().getX() != 0.0); // Twist
        try testing.expect(base.solvePositionConstraint(dt, 0.5));
        try testing.expect(t.body.getRotation().getRotationAngle(Vec3.axisX()) < 0.5);
    }

    // Translation velocity motor: drives the velocity along the axis of body 1's constraint space to the target, limited
    // by the force limits
    {
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        var t: TestSixDOF = undefined;
        try t.init(&context, allocator, &settings, Vec3.zero(), Vec3.zero());
        defer t.deinit(&context);
        const base = t.c.asConstraint();
        t.c.setMotorState(.translation_z, .velocity);
        t.c.setTargetVelocityCS(Vec3.init(0, 0, 1.5));
        base.setupVelocityConstraint(dt);
        try testing.expect(t.c.motor_translation_constraint_part[2].isActive());
        try testing.expect(!t.c.motor_translation_constraint_part[0].isActive());
        base.warmStartVelocityConstraint(1.0);
        for (0..5) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(@abs(t.body.getLinearVelocity().getZ() - 1.5) < 1.0e-4);
        try testing.expect(t.c.getTotalLambdaMotorTranslation().getZ() > 0.0);

        t.c.getMotorSettingsMut(.translation_z).setForceLimit(0.01);
        t.body.setLinearVelocity(Vec3.zero());
        base.resetWarmStart();
        base.setupVelocityConstraint(dt);
        for (0..5) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(@abs(t.c.getTotalLambdaMotorTranslation().getZ() - 0.01 * dt) < 1.0e-7);
    }

    // Translation position motors: drive towards the target position; without stiffness (position) or without
    // stiffness and damping (position and velocity) the motor part is off
    for ([_]MotorState{ .position, .position_and_velocity }) |state| {
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        var t: TestSixDOF = undefined;
        try t.init(&context, allocator, &settings, Vec3.zero(), Vec3.zero());
        defer t.deinit(&context);
        const base = t.c.asConstraint();
        t.c.setMotorState(.translation_x, state);
        t.c.setTargetPositionCS(Vec3.init(1, 0, 0));
        base.setupVelocityConstraint(dt);
        try testing.expect(t.c.motor_translation_constraint_part[0].isActive());
        base.warmStartVelocityConstraint(1.0);
        for (0..5) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(t.body.getLinearVelocity().getX() > 0.0);

        t.c.getMotorSettingsMut(.translation_x).spring_settings = .init(.stiffness_and_damping, 0.0, 0.0);
        base.setupVelocityConstraint(dt);
        try testing.expect(!t.c.motor_translation_constraint_part[0].isActive());
        t.c.getMotorSettingsMut(.translation_x).spring_settings = .init(.stiffness_and_damping, 0.0, 1.0);
        base.setupVelocityConstraint(dt);
        try testing.expectEqual(state == .position_and_velocity, t.c.motor_translation_constraint_part[0].isActive());
    }

    // Rotation velocity motor (body 2 constraint space) and torque limits
    {
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        var t: TestSixDOF = undefined;
        try t.init(&context, allocator, &settings, Vec3.zero(), Vec3.zero());
        defer t.deinit(&context);
        const base = t.c.asConstraint();
        t.c.setMotorState(.rotation_y, .velocity);
        t.c.setTargetAngularVelocityCS(Vec3.init(0, 1.5, 0));
        base.setupVelocityConstraint(dt);
        try testing.expect(t.c.motor_rotation_constraint_part[1].isActive() and !t.c.swing_twist_constraint_part.isActive());
        base.warmStartVelocityConstraint(1.0);
        for (0..5) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(@abs(t.body.getAngularVelocity().getY() - 1.5) < 1.0e-4);

        t.c.getMotorSettingsMut(.rotation_y).setTorqueLimit(0.01);
        t.body.setAngularVelocity(Vec3.zero());
        base.resetWarmStart();
        base.setupVelocityConstraint(dt);
        for (0..5) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(@abs(t.c.getTotalLambdaMotorRotation().getY() - 0.01 * dt) < 1.0e-7);
    }

    // Rotation position motors: every combination of axes with a position motor (the projection of the difference to
    // the target orientation) drives towards the target orientation around those axes
    for (1..8) |mask| {
        for ([_]MotorState{ .position, .position_and_velocity }) |state| {
            var settings = SixDOFConstraintSettings.init(allocator);
            defer settings.asSettings().deinit();
            var t: TestSixDOF = undefined;
            try t.init(&context, allocator, &settings, Vec3.zero(), Vec3.zero());
            defer t.deinit(&context);
            const base = t.c.asConstraint();
            for (0..3) |i| {
                if ((mask & (@as(usize, 1) << @intCast(i))) != 0)
                    t.c.setMotorState(@enumFromInt(3 + i), state);
            }
            try testing.expectEqual(@as(u8, @intCast(mask)), t.c.rotation_position_motor_active);
            const target = Quat.rotation(Vec3.init(1, 2, 3).normalized(), 0.3);
            t.c.setTargetOrientationCS(target);
            base.setupVelocityConstraint(dt);
            base.warmStartVelocityConstraint(1.0);
            for (0..5) |_| _ = base.solveVelocityConstraint(dt);
            const diff = target.getXYZ();
            for (0..3) |i| {
                const w = t.body.getAngularVelocity().getComponent(@intCast(i));
                if ((mask & (@as(usize, 1) << @intCast(i))) != 0) {
                    try testing.expect(t.c.motor_rotation_constraint_part[i].isActive());
                    try testing.expect(w * diff.getComponent(@intCast(i)) > 0.0);
                } else {
                    try testing.expect(!t.c.motor_rotation_constraint_part[i].isActive());
                }
            }
        }
    }

    // Friction: translation and rotation stop when the friction is high enough, weak friction only slows down
    {
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.max_friction = .{ 1.0e6, 0, 0, 0, 0, 1.0e6 };
        var t: TestSixDOF = undefined;
        try t.init(&context, allocator, &settings, Vec3.init(1, 1, 0), Vec3.init(0, 0, 2));
        defer t.deinit(&context);
        const base = t.c.asConstraint();
        try testing.expect(t.c.translation_motor_active and t.c.rotation_motor_active);
        base.setupVelocityConstraint(dt);
        try testing.expect(t.c.motor_translation_constraint_part[0].isActive() and !t.c.motor_translation_constraint_part[1].isActive());
        try testing.expect(t.c.motor_rotation_constraint_part[2].isActive() and !t.c.motor_rotation_constraint_part[0].isActive());
        base.warmStartVelocityConstraint(1.0);
        for (0..5) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(@abs(t.body.getLinearVelocity().getX()) < 1.0e-4);
        try testing.expectEqual(@as(f32, 1.0), t.body.getLinearVelocity().getY());
        try testing.expect(@abs(t.body.getAngularVelocity().getZ()) < 1.0e-4);

        t.c.setMaxFriction(.translation_x, 0.1);
        t.body.setLinearVelocity(Vec3.init(1, 1, 0));
        base.resetWarmStart();
        base.setupVelocityConstraint(dt);
        for (0..5) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(@abs(t.c.getTotalLambdaMotorTranslation().getX() + 0.1 * dt) < 1.0e-7);
        try testing.expect(t.body.getLinearVelocity().getX() > 0.5);
    }

    // A motor state outside the named values (RestoreState of foreign bytes) does nothing, like Jolt's switches
    {
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        var t: TestSixDOF = undefined;
        try t.init(&context, allocator, &settings, Vec3.init(1, 0, 0), Vec3.init(0, 1, 0));
        defer t.deinit(&context);
        const base = t.c.asConstraint();
        t.c.motor_state[0] = @enumFromInt(8);
        t.c.motor_state[4] = @enumFromInt(8);
        t.c.cacheTranslationMotorActive();
        t.c.cacheRotationMotorActive();
        t.c.cacheRotationPositionMotorActive();
        try testing.expect(t.c.translation_motor_active and t.c.rotation_motor_active);
        try testing.expectEqual(@as(u8, 0), t.c.rotation_position_motor_active); // 8 has no position bit
        base.setupVelocityConstraint(dt);
        try testing.expect(!t.c.motor_translation_constraint_part[0].isActive() and !t.c.motor_rotation_constraint_part[1].isActive());
        base.warmStartVelocityConstraint(1.0);
        try testing.expect(!base.solveVelocityConstraint(dt));
    }
}

test "SixDOFConstraint: body 1 fixed to the world, static and kinematic bodies" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        var settings = SixDOFConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.position1 = RVec3.init(1.5, 1.75, 3.5);
        settings.position2 = settings.position1;
        settings.makeFixedAxis(.translation_x);
        settings.setLimitedAxis(.translation_y, -0.1, 0.1);
        settings.setLimitedAxis(.rotation_x, -0.1, 0.1);
        settings.setLimitedAxis(.rotation_y, -0.1, 0.1);
        settings.max_friction = .{ 0, 0, 1, 0, 0, 1 };
        var c = SixDOFConstraint.init(allocator, bodies[0], bodies[1], &settings);
        defer c.asConstraint().deinit();
        const base = c.asConstraint();
        const any_dynamic = pair[0] == .dynamic or pair[1] == .dynamic;
        try testing.expectEqual(any_dynamic, base.isActive());

        // A static or kinematic body is never moved by the solver
        const before1 = TestContext.BodySnapshot.take(bodies[0]);
        const before2 = TestContext.BodySnapshot.take(bodies[1]);
        base.setupVelocityConstraint(1.0 / 60.0);
        base.warmStartVelocityConstraint(1.0);
        try testing.expectEqual(any_dynamic, base.solveVelocityConstraint(1.0 / 60.0));
        _ = base.solvePositionConstraint(1.0 / 60.0, 0.2);
        try TestContext.checkUnmoved(bodies[0], before1);
        try TestContext.checkUnmoved(bodies[1], before2);
    }

    // Body 1 is Body::sFixedToWorld
    const body = try context.createBody(.dynamic, .{ .position = RVec3.init(1, 2, 3), .linear_velocity = Vec3.init(1, 0, 0) });
    defer context.destroyBodies(&.{body});
    var settings = SixDOFConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.position1 = body.getCenterOfMassPosition();
    settings.position2 = settings.position1;
    for (0..3) |a| settings.makeFixedAxis(@enumFromInt(a));
    var c = SixDOFConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
    defer c.asConstraint().deinit();
    try testing.expect(c.local_space_position2.isNearZero(.{}));
    try testing.expect(c.local_space_position1.isClose(body.getCenterOfMassPosition().toVec3(), .{}));
    const base = c.asConstraint();
    base.setupVelocityConstraint(1.0 / 60.0);
    base.warmStartVelocityConstraint(1.0);
    try testing.expect(base.solveVelocityConstraint(1.0 / 60.0));
    try testing.expect(body.getLinearVelocity().isNearZero(.{ .max_dist_sq = 1.0e-10 }));
}

test "SixDOFConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const bodies = try context.createBodyPair(.{ .dynamic, .static });
            defer context.destroyBodies(&bodies);
            var settings = Ref(ConstraintSettings).init((try SixDOFConstraintSettings.create(allocator)).asSettings());
            defer settings.deinit();
            const two_body_settings = virtual.downcast(TwoBodyConstraintSettings, settings.get().?);
            var constraint = Ref(Constraint).init(&(try two_body_settings.createConstraint(allocator, bodies[0], bodies[1])).base);
            defer constraint.deinit();
            var copy = try constraint.get().?.getConstraintSettings();
            defer copy.deinit();
            const bytes = try testSaveSettings(allocator, copy.get().?);
            defer allocator.free(bytes);
            var restored = try testRestoreSettings(allocator, bytes);
            defer restored.deinit();
            if (!restored.isValid()) return error.TestUnexpectedResult;
        }
    };
    try testing.checkAllAllocationFailures(testing.allocator, S.run, .{});
}
