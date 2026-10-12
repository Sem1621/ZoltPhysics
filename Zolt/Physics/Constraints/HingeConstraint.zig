//! Port of: Jolt/Physics/Constraints/HingeConstraint.h, Jolt/Physics/Constraints/HingeConstraint.cpp
//! Status: complete
//!
//! Follows the porter template in FixedConstraint.zig. JPH_DECLARE_SERIALIZABLE_VIRTUAL /
//! JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(HingeConstraintSettings): TODO(serialization) (the binary state is ported).
//! JPH_DEBUG_RENDERER (DrawConstraint, DrawConstraintLimits): TODO(debug_renderer).
//!
//! - The const and non-const overloads of `GetMotorSettings` / `GetLimitsSpringSettings` are `getMotorSettings` /
//!   `getLimitsSpringSettings` (`*const`) and `getMotorSettingsMut` / `getLimitsSpringSettingsMut` (`*`, guide section
//!   4). `GetTotalLambdaRotation` returns the `Vector(2)` of the HingeRotationConstraintPart by value, like Jolt.
//! - The asserts that user input can violate (the limits of `SetLimits`, "better use a fixed constraint" in the
//!   constructor, a motor state other than Off with invalid motor settings) only run with `Core.enable_asserts`, so
//!   ReleaseFast keeps Jolt's release behavior.
//! - `EMotorState` accepts every int (MotorSettings.zig): a motor state outside the named ones (only reachable through
//!   RestoreState of foreign bytes) matches no case of Jolt's switches, the `_` prongs do nothing like Jolt.
//! - GearConstraint and RackAndPinionConstraint read `getCurrentAngle` and the local space hinge axes of a hinge
//!   (`virtual.downcast(HingeConstraint, two_body)` after checking `getSubType() == .hinge`).

const std = @import("std");
const Allocator = std.mem.Allocator;
const Core = @import("../../Core/Core.zig");
const Ref = @import("../../Core/Reference.zig").Ref;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Vector = @import("../../Math/Vector.zig").Vector;
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
const SpringSettings = @import("SpringSettings.zig").SpringSettings;
const PointConstraintPart = @import("ConstraintPart/PointConstraintPart.zig").PointConstraintPart;
const HingeRotationConstraintPart = @import("ConstraintPart/HingeRotationConstraintPart.zig").HingeRotationConstraintPart;
const AngleConstraintPart = @import("ConstraintPart/AngleConstraintPart.zig").AngleConstraintPart;
const RotationEulerConstraintPart = @import("ConstraintPart/RotationEulerConstraintPart.zig").RotationEulerConstraintPart;

/// Hinge constraint settings, used to create a hinge constraint
pub const HingeConstraintSettings = struct {
    // TODO(serialization): JPH_DECLARE_SERIALIZABLE_VIRTUAL(JPH_EXPORT, HingeConstraintSettings)

    /// Class name for the RTTI hash of the binary state (JPH_RTTI)
    pub const rtti_name = "HingeConstraintSettings";
    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// This determines in which space the constraint is setup, all properties below should be in the specified space
    space: ConstraintSpace = .world_space,

    /// Body 1 constraint reference frame (space determined by mSpace).
    /// Hinge axis is the axis where rotation is allowed.
    /// When the normal axis of both bodies align in world space, the hinge angle is defined to be 0.
    /// mHingeAxis1 and mNormalAxis1 should be perpendicular. mHingeAxis2 and mNormalAxis2 should also be perpendicular.
    /// If you configure the joint in world space and create both bodies with a relative rotation you want to be defined as zero,
    /// you can simply set mHingeAxis1 = mHingeAxis2 and mNormalAxis1 = mNormalAxis2.
    point1: RVec3 = RVec3.zero(),
    hinge_axis1: Vec3 = Vec3.axisY(),
    normal_axis1: Vec3 = Vec3.axisX(),

    /// Body 2 constraint reference frame (space determined by mSpace)
    point2: RVec3 = RVec3.zero(),
    hinge_axis2: Vec3 = Vec3.axisY(),
    normal_axis2: Vec3 = Vec3.axisX(),

    /// Rotation around the hinge axis will be limited between [mLimitsMin, mLimitsMax] where mLimitsMin e [-pi, 0] and mLimitsMax e [0, pi].
    /// Both angles are in radians.
    limits_min: f32 = -math.pi,
    limits_max: f32 = math.pi,

    /// When enabled, this makes the limits soft. When the constraint exceeds the limits, a spring force will pull it back.
    limits_spring_settings: SpringSettings = .{},

    /// Maximum amount of torque (N m) to apply as friction when the constraint is not powered by a motor
    max_friction_torque: f32 = 0.0,

    /// In case the constraint is powered, this determines the motor settings around the hinge axis
    motor_settings: MotorSettings = .{},

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) HingeConstraintSettings {
        return .{ .base = .init(HingeConstraintSettings, allocator) };
    }

    /// new HingeConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*HingeConstraintSettings {
        const self = try allocator.create(HingeConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other`, not its reference count
    pub fn initCopy(allocator: Allocator, other: *const HingeConstraintSettings) HingeConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        return result;
    }

    pub fn asSettings(self: *HingeConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const HingeConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *HingeConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const HingeConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const HingeConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.space);
        stream.write(self.point1);
        stream.write(self.hinge_axis1);
        stream.write(self.normal_axis1);
        stream.write(self.point2);
        stream.write(self.hinge_axis2);
        stream.write(self.normal_axis2);
        stream.write(self.limits_min);
        stream.write(self.limits_max);
        stream.write(self.max_friction_torque);
        self.limits_spring_settings.saveBinaryState(stream);
        self.motor_settings.saveBinaryState(stream);
    }

    /// Create an instance of this constraint
    pub fn createConstraint(self: *const HingeConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        const constraint = try HingeConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *HingeConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.space);
        stream.read(&self.point1);
        stream.read(&self.hinge_axis1);
        stream.read(&self.normal_axis1);
        stream.read(&self.point2);
        stream.read(&self.hinge_axis2);
        stream.read(&self.normal_axis2);
        stream.read(&self.limits_min);
        stream.read(&self.limits_max);
        stream.read(&self.max_friction_torque);
        self.limits_spring_settings.restoreBinaryState(stream);
        self.motor_settings.restoreBinaryState(stream);
    }
};

/// A hinge constraint constrains 2 bodies on a single point and allows only a single axis of rotation
pub const HingeConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,

    // CONFIGURATION PROPERTIES FOLLOW

    // Local space constraint positions
    local_space_position1: Vec3 = undefined,
    local_space_position2: Vec3 = undefined,

    // Local space hinge directions
    local_space_hinge_axis1: Vec3 = undefined,
    local_space_hinge_axis2: Vec3 = undefined,

    // Local space normal direction (direction relative to which to draw constraint limits)
    local_space_normal_axis1: Vec3 = undefined,
    local_space_normal_axis2: Vec3 = undefined,

    // Inverse of initial relative orientation between bodies (which defines hinge angle = 0)
    inv_initial_orientation: Quat = undefined,

    // Hinge limits
    has_limits: bool = undefined,
    limits_min: f32 = undefined,
    limits_max: f32 = undefined,

    // Soft constraint limits
    limits_spring_settings: SpringSettings = .{},

    // Friction
    max_friction_torque: f32 = undefined,

    // Motor controls
    motor_settings: MotorSettings = .{},
    motor_state: MotorState = .off,
    target_angular_velocity: f32 = 0.0,
    target_angle: f32 = 0.0,

    // RUN TIME PROPERTIES FOLLOW

    // Current rotation around the hinge axis
    theta: f32 = 0.0,

    // World space hinge axis for body 1
    a1: Vec3 = undefined,

    // The constraint parts
    point_constraint_part: PointConstraintPart = .{},
    rotation_constraint_part: HingeRotationConstraintPart = .{},
    rotation_limits_constraint_part: AngleConstraintPart = .{},
    motor_constraint_part: AngleConstraintPart = .{},

    /// Construct hinge constraint
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const HingeConstraintSettings) HingeConstraint {
        var self: HingeConstraint = .{
            .base = .init(HingeConstraint, allocator, body1, body2, &settings.base),
            .max_friction_torque = settings.max_friction_torque,
            .motor_settings = settings.motor_settings,
        };

        // Store limits
        if (Core.enable_asserts) std.debug.assert(settings.limits_min != settings.limits_max or settings.limits_spring_settings.frequency > 0.0); // Better use a fixed constraint in this case
        self.setLimits(settings.limits_min, settings.limits_max);

        // Store inverse of initial rotation from body 1 to body 2 in body 1 space
        self.inv_initial_orientation = RotationEulerConstraintPart.getInvInitialOrientationXZ(settings.normal_axis1, settings.hinge_axis1, settings.normal_axis2, settings.hinge_axis2);

        if (settings.space == .world_space) {
            // If all properties were specified in world space, take them to local space now
            const inv_transform1 = body1.getInverseCenterOfMassTransform();
            self.local_space_position1 = inv_transform1.mulRVec3(settings.point1).toVec3();
            self.local_space_hinge_axis1 = inv_transform1.multiply3x3(settings.hinge_axis1).normalized();
            self.local_space_normal_axis1 = inv_transform1.multiply3x3(settings.normal_axis1).normalized();

            const inv_transform2 = body2.getInverseCenterOfMassTransform();
            self.local_space_position2 = inv_transform2.mulRVec3(settings.point2).toVec3();
            self.local_space_hinge_axis2 = inv_transform2.multiply3x3(settings.hinge_axis2).normalized();
            self.local_space_normal_axis2 = inv_transform2.multiply3x3(settings.normal_axis2).normalized();

            // Constraints were specified in world space, so we should have replaced c1 with q10^-1 c1 and c2 with q20^-1 c2
            // => r0^-1 = (q20^-1 c2) (q10^-1 c1)^1 = q20^-1 (c2 c1^-1) q10
            self.inv_initial_orientation = body2.getRotation().conjugated().mul(self.inv_initial_orientation).mul(body1.getRotation());
        } else {
            self.local_space_position1 = settings.point1.toVec3();
            self.local_space_hinge_axis1 = settings.hinge_axis1;
            self.local_space_normal_axis1 = settings.normal_axis1;

            self.local_space_position2 = settings.point2.toVec3();
            self.local_space_hinge_axis2 = settings.hinge_axis2;
            self.local_space_normal_axis2 = settings.normal_axis2;
        }

        // Store spring settings
        self.setLimitsSpringSettings(settings.limits_spring_settings);

        return self;
    }

    /// new HingeConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const HingeConstraintSettings) Allocator.Error!*HingeConstraint {
        const self = try allocator.create(HingeConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    pub fn asConstraint(self: *HingeConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const HingeConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *HingeConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const HingeConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    // Generic interface of a constraint
    pub fn getSubType(self: *const HingeConstraint) ConstraintSubType {
        _ = self;
        return .hinge;
    }

    pub fn notifyShapeChanged(self: *HingeConstraint, body_id: BodyID, delta_com: Vec3) void {
        if (self.base.body1.getID().eql(body_id)) {
            self.local_space_position1 = self.local_space_position1.sub(delta_com);
        } else if (self.base.body2.getID().eql(body_id)) {
            self.local_space_position2 = self.local_space_position2.sub(delta_com);
        }
    }

    pub fn setupVelocityConstraint(self: *HingeConstraint, delta_time: f32) void {
        // Cache constraint values that are valid until the bodies move
        const rotation1 = Mat44.rotationQuat(self.base.body1.getRotation());
        const rotation2 = Mat44.rotationQuat(self.base.body2.getRotation());
        self.point_constraint_part.calculateConstraintProperties(self.base.body1, rotation1, self.local_space_position1, self.base.body2, rotation2, self.local_space_position2);
        self.rotation_constraint_part.calculateConstraintProperties(self.base.body1, rotation1, rotation1.multiply3x3(self.local_space_hinge_axis1), self.base.body2, rotation2, rotation2.multiply3x3(self.local_space_hinge_axis2));
        self.calculateA1AndTheta();
        self.calculateRotationLimitsConstraintProperties(delta_time);
        self.calculateMotorConstraintProperties(delta_time);
    }

    pub fn resetWarmStart(self: *HingeConstraint) void {
        self.motor_constraint_part.deactivate();
        self.point_constraint_part.deactivate();
        self.rotation_constraint_part.deactivate();
        self.rotation_limits_constraint_part.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *HingeConstraint, warm_start_impulse_ratio: f32) void {
        // Warm starting: Apply previous frame impulse
        self.motor_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
        self.point_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
        self.rotation_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
        self.rotation_limits_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *HingeConstraint, delta_time: f32) bool {
        // Solve motor
        var motor = false;
        if (self.motor_constraint_part.isActive()) {
            switch (self.motor_state) {
                .off => {
                    const max_lambda = self.max_friction_torque * delta_time;
                    motor = self.motor_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2, self.a1, -max_lambda, max_lambda);
                },

                .velocity, .position, .position_and_velocity => {
                    motor = self.motor_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2, self.a1, delta_time * self.motor_settings.min_torque_limit, delta_time * self.motor_settings.max_torque_limit);
                },

                // No case of Jolt's switch matches a value outside the named ones (see the file comment)
                _ => {},
            }
        }

        // Solve point constraint
        const pos = self.point_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2);

        // Solve rotation constraint
        const rot = self.rotation_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2);

        // Solve rotation limits
        var limit = false;
        if (self.rotation_limits_constraint_part.isActive()) {
            var min_lambda: f32 = undefined;
            var max_lambda: f32 = undefined;
            if (self.limits_min == self.limits_max) {
                min_lambda = -math.flt_max;
                max_lambda = math.flt_max;
            } else if (self.isMinLimitClosest()) {
                min_lambda = 0.0;
                max_lambda = math.flt_max;
            } else {
                min_lambda = -math.flt_max;
                max_lambda = 0.0;
            }
            limit = self.rotation_limits_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2, self.a1, min_lambda, max_lambda);
        }

        return motor or pos or rot or limit;
    }

    pub fn solvePositionConstraint(self: *HingeConstraint, delta_time: f32, baumgarte: f32) bool {
        // Motor operates on velocities only, don't call SolvePositionConstraint

        // Solve point constraint
        self.point_constraint_part.calculateConstraintProperties(self.base.body1, Mat44.rotationQuat(self.base.body1.getRotation()), self.local_space_position1, self.base.body2, Mat44.rotationQuat(self.base.body2.getRotation()), self.local_space_position2);
        const pos = self.point_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, baumgarte);

        // Solve rotation constraint
        const rotation1 = Mat44.rotationQuat(self.base.body1.getRotation()); // Note that previous call to GetRotation() is out of date since the rotation has changed
        const rotation2 = Mat44.rotationQuat(self.base.body2.getRotation());
        self.rotation_constraint_part.calculateConstraintProperties(self.base.body1, rotation1, rotation1.multiply3x3(self.local_space_hinge_axis1), self.base.body2, rotation2, rotation2.multiply3x3(self.local_space_hinge_axis2));
        const rot = self.rotation_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, baumgarte);

        // Solve rotation limits
        var limit = false;
        if (self.has_limits and self.limits_spring_settings.frequency <= 0.0) {
            self.calculateA1AndTheta();
            self.calculateRotationLimitsConstraintProperties(delta_time);
            if (self.rotation_limits_constraint_part.isActive())
                limit = self.rotation_limits_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, self.getSmallestAngleToLimit(), baumgarte);
        }

        return pos or rot or limit;
    }

    // TODO(debug_renderer): DrawConstraint, DrawConstraintLimits

    pub fn saveState(self: *const HingeConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        self.motor_constraint_part.saveState(stream);
        self.rotation_constraint_part.saveState(stream);
        self.point_constraint_part.saveState(stream);
        self.rotation_limits_constraint_part.saveState(stream);

        stream.write(self.motor_state);
        stream.write(self.target_angular_velocity);
        stream.write(self.target_angle);
    }

    pub fn restoreState(self: *HingeConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        self.motor_constraint_part.restoreState(stream);
        self.rotation_constraint_part.restoreState(stream);
        self.point_constraint_part.restoreState(stream);
        self.rotation_limits_constraint_part.restoreState(stream);

        stream.read(&self.motor_state);
        stream.read(&self.target_angular_velocity);
        stream.read(&self.target_angle);
    }

    pub fn getConstraintSettings(self: *const HingeConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try HingeConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(settings.asSettings());
        settings.space = .local_to_body_com;
        settings.point1 = RVec3.fromVec3(self.local_space_position1);
        settings.hinge_axis1 = self.local_space_hinge_axis1;
        settings.normal_axis1 = self.local_space_normal_axis1;
        settings.point2 = RVec3.fromVec3(self.local_space_position2);
        settings.hinge_axis2 = self.local_space_hinge_axis2;
        settings.normal_axis2 = self.local_space_normal_axis2;
        settings.limits_min = self.limits_min;
        settings.limits_max = self.limits_max;
        settings.limits_spring_settings = self.limits_spring_settings;
        settings.max_friction_torque = self.max_friction_torque;
        settings.motor_settings = self.motor_settings;
        return .init(settings.asSettings());
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const HingeConstraint) Mat44 {
        return Mat44.init(Vec4.fromVec3W(self.local_space_hinge_axis1, 0), Vec4.fromVec3W(self.local_space_normal_axis1, 0), Vec4.fromVec3W(self.local_space_hinge_axis1.cross(self.local_space_normal_axis1), 0), Vec4.fromVec3W(self.local_space_position1, 1));
    }

    pub fn getConstraintToBody2Matrix(self: *const HingeConstraint) Mat44 {
        return Mat44.init(Vec4.fromVec3W(self.local_space_hinge_axis2, 0), Vec4.fromVec3W(self.local_space_normal_axis2, 0), Vec4.fromVec3W(self.local_space_hinge_axis2.cross(self.local_space_normal_axis2), 0), Vec4.fromVec3W(self.local_space_position2, 1));
    }

    /// Get the attachment point for body 1 relative to body 1 COM (transform by Body::GetCenterOfMassTransform to take to world space)
    pub fn getLocalSpacePoint1(self: *const HingeConstraint) Vec3 {
        return self.local_space_position1;
    }

    /// Get the attachment point for body 2 relative to body 2 COM (transform by Body::GetCenterOfMassTransform to take to world space)
    pub fn getLocalSpacePoint2(self: *const HingeConstraint) Vec3 {
        return self.local_space_position2;
    }

    // Local space hinge directions (transform direction by Body::GetCenterOfMassTransform to take to world space)
    pub fn getLocalSpaceHingeAxis1(self: *const HingeConstraint) Vec3 {
        return self.local_space_hinge_axis1;
    }

    pub fn getLocalSpaceHingeAxis2(self: *const HingeConstraint) Vec3 {
        return self.local_space_hinge_axis2;
    }

    // Local space normal directions (transform direction by Body::GetCenterOfMassTransform to take to world space)
    pub fn getLocalSpaceNormalAxis1(self: *const HingeConstraint) Vec3 {
        return self.local_space_normal_axis1;
    }

    pub fn getLocalSpaceNormalAxis2(self: *const HingeConstraint) Vec3 {
        return self.local_space_normal_axis2;
    }

    /// Get the current rotation angle from the rest position
    pub fn getCurrentAngle(self: *const HingeConstraint) f32 {
        // See: CalculateA1AndTheta
        const rotation1 = self.base.body1.getRotation();
        const diff = self.base.body2.getRotation().mul(self.inv_initial_orientation).mul(rotation1.conjugated());
        return diff.getRotationAngle(rotation1.mulVec3(self.local_space_hinge_axis1));
    }

    // Friction control
    pub fn setMaxFrictionTorque(self: *HingeConstraint, friction_torque: f32) void {
        self.max_friction_torque = friction_torque;
    }

    pub fn getMaxFrictionTorque(self: *const HingeConstraint) f32 {
        return self.max_friction_torque;
    }

    // Motor settings
    pub fn getMotorSettingsMut(self: *HingeConstraint) *MotorSettings {
        return &self.motor_settings;
    }

    pub fn getMotorSettings(self: *const HingeConstraint) *const MotorSettings {
        return &self.motor_settings;
    }

    // Motor controls
    pub fn setMotorState(self: *HingeConstraint, state: MotorState) void {
        if (Core.enable_asserts) std.debug.assert(state == .off or self.motor_settings.isValid());
        self.motor_state = state;
    }

    pub fn getMotorState(self: *const HingeConstraint) MotorState {
        return self.motor_state;
    }

    /// rad/s
    pub fn setTargetAngularVelocity(self: *HingeConstraint, angular_velocity: f32) void {
        self.target_angular_velocity = angular_velocity;
    }

    pub fn getTargetAngularVelocity(self: *const HingeConstraint) f32 {
        return self.target_angular_velocity;
    }

    /// rad
    pub fn setTargetAngle(self: *HingeConstraint, angle: f32) void {
        self.target_angle = if (self.has_limits) math.clamp(angle, self.limits_min, self.limits_max) else angle;
    }

    pub fn getTargetAngle(self: *const HingeConstraint) f32 {
        return self.target_angle;
    }

    /// Set the target orientation in body space (R2 = R1 * inOrientation, where R1 and R2 are the world space rotations for body 1 and 2).
    /// Calculates the local space target angle and calls SetTargetAngle. Motor state must be EMotorState::Position or EMotorState::PositionAndVelocity for this to have any effect.
    /// May set the wrong angle if inOrientation contains large rotations around other axis than the hinge axis.
    pub fn setTargetOrientationBS(self: *HingeConstraint, orientation: Quat) void {
        // See: CalculateA1AndTheta
        //
        // The rotation between body 1 and 2 can be written as:
        //
        // q2 = q1 rh1 r0
        //
        // where rh1 is a rotation around local hinge axis 1, also:
        //
        // q2 = q1 inOrientation
        //
        // This means:
        //
        // rh1 r0 = inOrientation <=> rh1 = inOrientation * r0^-1
        const rh1 = orientation.mul(self.inv_initial_orientation);
        self.setTargetAngle(rh1.getRotationAngle(self.local_space_hinge_axis1));
    }

    /// Update the rotation limits of the hinge, value in radians (see HingeConstraintSettings)
    pub fn setLimits(self: *HingeConstraint, limits_min: f32, limits_max: f32) void {
        if (Core.enable_asserts) std.debug.assert(limits_min <= 0.0 and limits_min >= -math.pi);
        if (Core.enable_asserts) std.debug.assert(limits_max >= 0.0 and limits_max <= math.pi);
        self.limits_min = limits_min;
        self.limits_max = limits_max;
        self.has_limits = self.limits_min > -math.pi or self.limits_max < math.pi;
    }

    pub fn getLimitsMin(self: *const HingeConstraint) f32 {
        return self.limits_min;
    }

    pub fn getLimitsMax(self: *const HingeConstraint) f32 {
        return self.limits_max;
    }

    pub fn hasLimits(self: *const HingeConstraint) bool {
        return self.has_limits;
    }

    /// Update the limits spring settings
    pub fn getLimitsSpringSettings(self: *const HingeConstraint) *const SpringSettings {
        return &self.limits_spring_settings;
    }

    pub fn getLimitsSpringSettingsMut(self: *HingeConstraint) *SpringSettings {
        return &self.limits_spring_settings;
    }

    pub fn setLimitsSpringSettings(self: *HingeConstraint, limits_spring_settings: SpringSettings) void {
        self.limits_spring_settings = limits_spring_settings;
    }

    ///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
    pub fn getTotalLambdaPosition(self: *const HingeConstraint) Vec3 {
        return self.point_constraint_part.getTotalLambda();
    }

    pub fn getTotalLambdaRotation(self: *const HingeConstraint) Vector(2) {
        return self.rotation_constraint_part.getTotalLambda().*;
    }

    pub fn getTotalLambdaRotationLimits(self: *const HingeConstraint) f32 {
        return self.rotation_limits_constraint_part.getTotalLambda();
    }

    pub fn getTotalLambdaMotor(self: *const HingeConstraint) f32 {
        return self.motor_constraint_part.getTotalLambda();
    }

    // Internal helper function to calculate the values below
    fn calculateA1AndTheta(self: *HingeConstraint) void {
        if (self.has_limits or self.motor_state != .off or self.max_friction_torque > 0.0) {
            const rotation1 = self.base.body1.getRotation();

            // Calculate relative rotation in world space
            //
            // The rest rotation is:
            //
            // q2 = q1 r0
            //
            // But the actual rotation is
            //
            // q2 = diff q1 r0
            // <=> diff = q2 r0^-1 q1^-1
            //
            // Where:
            // q1 = current rotation of body 1
            // q2 = current rotation of body 2
            // diff = relative rotation in world space
            const diff = self.base.body2.getRotation().mul(self.inv_initial_orientation).mul(rotation1.conjugated());

            // Calculate hinge axis in world space
            self.a1 = rotation1.mulVec3(self.local_space_hinge_axis1);

            // Get rotation angle around the hinge axis
            self.theta = diff.getRotationAngle(self.a1);
        }
    }

    fn calculateRotationLimitsConstraintProperties(self: *HingeConstraint, delta_time: f32) void {
        // Apply constraint if outside of limits
        if (self.has_limits and (self.theta <= self.limits_min or self.theta >= self.limits_max))
            self.rotation_limits_constraint_part.calculateConstraintPropertiesWithSettingsForLimit(delta_time, self.base.body1, self.base.body2, self.a1, 0.0, self.getSmallestAngleToLimit(), &self.limits_spring_settings)
        else
            self.rotation_limits_constraint_part.deactivate();
    }

    fn calculateMotorConstraintProperties(self: *HingeConstraint, delta_time: f32) void {
        switch (self.motor_state) {
            .off => {
                if (self.max_friction_torque > 0.0)
                    self.motor_constraint_part.calculateConstraintProperties(self.base.body1, self.base.body2, self.a1, .{})
                else
                    self.motor_constraint_part.deactivate();
            },

            .velocity => {
                self.motor_constraint_part.calculateConstraintProperties(self.base.body1, self.base.body2, self.a1, .{ .bias = -self.target_angular_velocity });
            },

            .position => {
                if (self.motor_settings.spring_settings.hasStiffness())
                    self.motor_constraint_part.calculateConstraintPropertiesWithSettingsForMotor(delta_time, self.base.body1, self.base.body2, self.a1, 0.0, math.centerAngleAroundZero(self.theta - self.target_angle), &self.motor_settings.spring_settings)
                else
                    self.motor_constraint_part.deactivate();
            },

            .position_and_velocity => {
                if (self.motor_settings.spring_settings.hasStiffnessOrDamping())
                    self.motor_constraint_part.calculateConstraintPropertiesWithSettingsForMotor(delta_time, self.base.body1, self.base.body2, self.a1, -self.target_angular_velocity, math.centerAngleAroundZero(self.theta - self.target_angle), &self.motor_settings.spring_settings)
                else
                    self.motor_constraint_part.deactivate();
            },

            // No case of Jolt's switch matches a value outside the named ones (see the file comment)
            _ => {},
        }
    }

    fn getSmallestAngleToLimit(self: *const HingeConstraint) f32 {
        const dist_to_min = math.centerAngleAroundZero(self.theta - self.limits_min);
        const dist_to_max = math.centerAngleAroundZero(self.theta - self.limits_max);
        return if (@abs(dist_to_min) < @abs(dist_to_max)) dist_to_min else dist_to_max;
    }

    fn isMinLimitClosest(self: *const HingeConstraint) bool {
        const dist_to_min = math.centerAngleAroundZero(self.theta - self.limits_min);
        const dist_to_max = math.centerAngleAroundZero(self.theta - self.limits_max);
        return @abs(dist_to_min) < @abs(dist_to_max);
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig, simulations in
// ZoltTests/Physics/HingeConstraintTests.zig)

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
fn testSettings(allocator: Allocator) HingeConstraintSettings {
    var settings = HingeConstraintSettings.init(allocator);
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 7;
    settings.base.base.num_velocity_steps_override = 3;
    settings.base.base.num_position_steps_override = 5;
    settings.base.base.draw_constraint_size = 0.25;
    settings.space = .local_to_body_com;
    settings.point1 = RVec3.init(1, -2, 3);
    settings.hinge_axis1 = Vec3.axisZ();
    settings.normal_axis1 = Vec3.axisY();
    settings.point2 = RVec3.init(-0.5, 0.25, 4);
    settings.hinge_axis2 = Vec3.axisX();
    settings.normal_axis2 = Vec3.axisZ();
    settings.limits_min = -1.25;
    settings.limits_max = 0.75;
    settings.limits_spring_settings = .init(.stiffness_and_damping, 120, 3.5);
    settings.max_friction_torque = 2.5;
    settings.motor_settings = .initModeLimits(.mass_normalized_stiffness_and_damping, 40, 0.5, 100, 25);
    return settings;
}

test "HingeConstraintSettings: defaults, binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;

    // Jolt's defaults
    {
        var settings = HingeConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        try testing.expectEqual(ConstraintSpace.world_space, settings.space);
        try testing.expect(settings.point1.eql(RVec3.zero()) and settings.point2.eql(RVec3.zero()));
        try testing.expect(settings.hinge_axis1.eql(Vec3.axisY()) and settings.normal_axis1.eql(Vec3.axisX()));
        try testing.expect(settings.hinge_axis2.eql(Vec3.axisY()) and settings.normal_axis2.eql(Vec3.axisX()));
        try testing.expectEqual(-math.pi, settings.limits_min);
        try testing.expectEqual(math.pi, settings.limits_max);
        try testing.expectEqual(SpringSettings{}, settings.limits_spring_settings);
        try testing.expectEqual(@as(f32, 0.0), settings.max_friction_torque);
        try testing.expectEqual(MotorSettings{}, settings.motor_settings);
        try testing.expectEqualStrings("HingeConstraintSettings", settings.asSettings().vtable.rtti_name);
    }

    var settings = testSettings(allocator);
    defer settings.asSettings().deinit();
    settings.base.base.user_data = 99; // Not part of the binary state (Jolt)

    // ConstraintSettings (21 bytes), space (int), the points (Real), the axes, limits min / max, max friction torque,
    // the limits spring (mode byte, 2 floats), the motor (its spring, 4 limits)
    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    try testing.expectEqual(@as(usize, 21 + 4 + 2 * 3 * @sizeOf(Real) + 4 * 12 + 3 * 4 + 9 + 9 + 4 * 4), bytes.len);
    try testing.expectEqual(ConstraintSettings.rttiHash("HingeConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(i32, 0), std.mem.readInt(i32, bytes[21..25], .little)); // LocalToBodyCOM
    const after_axes = 25 + 2 * 3 * @sizeOf(Real) + 4 * 12;
    try testing.expectEqual(@as(u32, @bitCast(@as(f32, -1.25))), std.mem.readInt(u32, bytes[after_axes..][0..4], .little));
    try testing.expectEqual(@as(u32, @bitCast(@as(f32, 2.5))), std.mem.readInt(u32, bytes[after_axes + 8 ..][0..4], .little)); // Friction before the springs
    try testing.expectEqual(@as(u8, 1), bytes[after_axes + 12]); // Limits spring mode: StiffnessAndDamping

    // Restore through RegisterTypes.constraint_settings_types
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(HingeConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(!restored.base.base.enabled);
        try testing.expectEqual(@as(u32, 7), restored.base.base.constraint_priority);
        try testing.expectEqual(@as(u32, 3), restored.base.base.num_velocity_steps_override);
        try testing.expectEqual(@as(u32, 5), restored.base.base.num_position_steps_override);
        try testing.expectEqual(@as(f32, 0.25), restored.base.base.draw_constraint_size);
        try testing.expectEqual(@as(u64, 0), restored.base.base.user_data);
        try testing.expectEqual(ConstraintSpace.local_to_body_com, restored.space);
        try testing.expect(restored.point1.eql(settings.point1) and restored.point2.eql(settings.point2));
        try testing.expect(restored.hinge_axis1.eql(settings.hinge_axis1) and restored.normal_axis1.eql(settings.normal_axis1));
        try testing.expect(restored.hinge_axis2.eql(settings.hinge_axis2) and restored.normal_axis2.eql(settings.normal_axis2));
        try testing.expectEqual(settings.limits_min, restored.limits_min);
        try testing.expectEqual(settings.limits_max, restored.limits_max);
        try testing.expectEqual(settings.limits_spring_settings, restored.limits_spring_settings);
        try testing.expectEqual(settings.max_friction_torque, restored.max_friction_torque);
        try testing.expectEqual(settings.motor_settings, restored.motor_settings);
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, bytes, resaved);
    }

    // Truncated anywhere: Jolt's error
    for ([_]usize{ 1, 9, 26, bytes.len - 22 }) |cut| {
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
    var copy = HingeConstraintSettings.initCopy(allocator, &settings);
    defer copy.asSettings().deinit();
    try testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try testing.expect(copy.point2.eql(settings.point2) and copy.normal_axis2.eql(settings.normal_axis2));
    try testing.expectEqual(settings.motor_settings, copy.motor_settings);
    try testing.expectEqual(@as(u64, 99), copy.base.base.user_data);
}

/// The relative angular velocity of the bodies around the world space hinge axis of body 1
fn relativeAngularVelocity(c: *const HingeConstraint) f32 {
    const a1 = c.base.body1.getRotation().mulVec3(c.local_space_hinge_axis1);
    return a1.dot(c.base.body2.getAngularVelocity().sub(c.base.body1.getAngularVelocity()));
}

test "HingeConstraint: virtual dispatch, space conversions, settings and state" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    defer context.destroyBodies(&bodies);
    const body1 = bodies[0];
    const body2 = bodies[1];

    // World space
    var settings = HingeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = RVec3.init(1.5, 2.0, 3.5);
    settings.point2 = RVec3.init(1.25, 2.0, 3.75);
    settings.hinge_axis1 = Vec3.axisY();
    settings.normal_axis1 = Vec3.axisX();
    settings.hinge_axis2 = Vec3.axisZ();
    settings.normal_axis2 = Vec3.axisY().negate();
    settings.limits_min = -0.5;
    settings.limits_max = 1.0;
    settings.max_friction_torque = 3.0;
    settings.base.base.user_data = 77;

    // TwoBodyConstraintSettings::Create through the vtable
    var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
    defer constraint.deinit();
    const base = constraint.get().?;
    const two_body = virtual.downcast(TwoBodyConstraint, base);
    const c = virtual.downcast(HingeConstraint, two_body);
    try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
    try testing.expectEqual(ConstraintSubType.hinge, base.getSubType());
    try testing.expectEqual(@as(u64, 77), base.getUserData());
    try testing.expect(two_body.getBody1() == body1 and two_body.getBody2() == body2);

    // The constructor took the points and axes to the local space of the bodies
    const inv1 = body1.getInverseCenterOfMassTransform();
    const inv2 = body2.getInverseCenterOfMassTransform();
    try testing.expect(c.getLocalSpacePoint1().eql(inv1.mulRVec3(settings.point1).toVec3()));
    try testing.expect(c.getLocalSpacePoint2().eql(inv2.mulRVec3(settings.point2).toVec3()));
    try testing.expect(c.getLocalSpaceHingeAxis1().eql(inv1.multiply3x3(settings.hinge_axis1).normalized()));
    try testing.expect(c.getLocalSpaceNormalAxis1().eql(inv1.multiply3x3(settings.normal_axis1).normalized()));
    try testing.expect(c.getLocalSpaceHingeAxis2().eql(inv2.multiply3x3(settings.hinge_axis2).normalized()));
    try testing.expect(c.getLocalSpaceNormalAxis2().eql(inv2.multiply3x3(settings.normal_axis2).normalized()));
    try testing.expect(c.getLocalSpaceHingeAxis1().isNormalized(.{}) and c.getLocalSpaceNormalAxis2().isNormalized(.{}));
    const expected_orientation = body2.getRotation().conjugated().mul(RotationEulerConstraintPart.getInvInitialOrientationXZ(settings.normal_axis1, settings.hinge_axis1, settings.normal_axis2, settings.hinge_axis2)).mul(body1.getRotation());
    try testing.expect(c.inv_initial_orientation.eql(expected_orientation));

    // World space axes are normalized after the transform (the initial orientation uses them as they are, Jolt expects
    // unit axes there)
    {
        var scaled = HingeConstraintSettings.initCopy(allocator, &settings);
        defer scaled.asSettings().deinit();
        scaled.hinge_axis1 = Vec3.init(0, 2, 0);
        scaled.normal_axis1 = Vec3.init(3, 0, 0);
        scaled.hinge_axis2 = Vec3.init(0, 0, 0.5);
        scaled.normal_axis2 = Vec3.init(0, -1, 0);
        var c2 = HingeConstraint.init(allocator, body1, body2, &scaled);
        defer c2.asConstraint().deinit();
        try testing.expect(c2.getLocalSpaceHingeAxis1().eql(inv1.multiply3x3(scaled.hinge_axis1).normalized()));
        try testing.expect(c2.getLocalSpaceNormalAxis1().eql(inv1.multiply3x3(scaled.normal_axis1).normalized()));
        try testing.expect(c2.getLocalSpaceHingeAxis2().eql(inv2.multiply3x3(scaled.hinge_axis2).normalized()));
        try testing.expect(c2.getLocalSpaceNormalAxis2().eql(inv2.multiply3x3(scaled.normal_axis2).normalized()));
        try testing.expect(c2.getLocalSpaceHingeAxis1().isClose(c.getLocalSpaceHingeAxis1(), .{ .max_dist_sq = 1.0e-12 }));
        try testing.expect(c2.getLocalSpaceNormalAxis2().isClose(c.getLocalSpaceNormalAxis2(), .{ .max_dist_sq = 1.0e-12 }));
    }

    // The settings values
    try testing.expect(c.hasLimits());
    try testing.expectEqual(@as(f32, -0.5), c.getLimitsMin());
    try testing.expectEqual(@as(f32, 1.0), c.getLimitsMax());
    try testing.expectEqual(@as(f32, 3.0), c.getMaxFrictionTorque());
    try testing.expectEqual(MotorState.off, c.getMotorState());
    try testing.expectEqual(@as(f32, 0.0), c.getTargetAngularVelocity());
    try testing.expectEqual(@as(f32, 0.0), c.getTargetAngle());
    try testing.expectEqual(MotorSettings{}, c.getMotorSettings().*);
    try testing.expectEqual(SpringSettings{}, c.getLimitsSpringSettings().*);

    // The hinge angle (the frames of the bodies differ at the creation, so it is not 0: see the setters test for the
    // angle of a rotation around the hinge axis)
    try testing.expect(std.math.isFinite(c.getCurrentAngle()) and @abs(c.getCurrentAngle()) <= math.pi);

    // The matrices: hinge axis, normal axis, their cross product and the attachment point
    const m1 = two_body.getConstraintToBody1Matrix();
    try testing.expect(m1.getAxisX().eql(c.getLocalSpaceHingeAxis1()) and m1.getAxisY().eql(c.getLocalSpaceNormalAxis1()));
    try testing.expect(m1.getAxisZ().eql(c.getLocalSpaceHingeAxis1().cross(c.getLocalSpaceNormalAxis1())));
    try testing.expect(m1.getTranslation().eql(c.getLocalSpacePoint1()));
    try testing.expectEqual(@as(f32, 0.0), m1.get(3, 0));
    try testing.expectEqual(@as(f32, 1.0), m1.get(3, 3));
    const m2 = two_body.getConstraintToBody2Matrix();
    try testing.expect(m2.getAxisX().eql(c.getLocalSpaceHingeAxis2()) and m2.getAxisY().eql(c.getLocalSpaceNormalAxis2()));
    try testing.expect(m2.getAxisZ().eql(c.getLocalSpaceHingeAxis2().cross(c.getLocalSpaceNormalAxis2())));
    try testing.expect(m2.getTranslation().eql(c.getLocalSpacePoint2()));

    // GetConstraintSettings: local space, a constraint made from them is the same
    {
        c.setMotorState(.velocity); // Not part of the settings
        defer c.setMotorState(.off);
        c.getMotorSettingsMut().* = .initLimits(5, 0.5, 10, 20);
        c.setLimitsSpringSettings(.init(.frequency_and_damping, 3, 0.25));
        var copy = try base.getConstraintSettings();
        defer copy.deinit();
        const s = virtual.downcast(HingeConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, copy.get().?));
        try testing.expectEqual(ConstraintSpace.local_to_body_com, s.space);
        try testing.expectEqual(@as(u64, 77), s.base.base.user_data);
        try testing.expect(s.point1.eql(RVec3.fromVec3(c.getLocalSpacePoint1())) and s.hinge_axis2.eql(c.getLocalSpaceHingeAxis2()));
        try testing.expectEqual(@as(f32, -0.5), s.limits_min);
        try testing.expectEqual(@as(f32, 3.0), s.max_friction_torque);
        try testing.expectEqual(c.getMotorSettings().*, s.motor_settings);
        try testing.expectEqual(c.getLimitsSpringSettings().*, s.limits_spring_settings);
        var again = Ref(Constraint).init(&(try s.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer again.deinit();
        const c2 = virtual.downcast(HingeConstraint, virtual.downcast(TwoBodyConstraint, again.get().?));
        try testing.expect(c2.getLocalSpacePoint1().eql(c.getLocalSpacePoint1()) and c2.getLocalSpacePoint2().eql(c.getLocalSpacePoint2()));
        try testing.expect(c2.getLocalSpaceHingeAxis1().eql(c.getLocalSpaceHingeAxis1()) and c2.getLocalSpaceNormalAxis2().eql(c.getLocalSpaceNormalAxis2()));
        try testing.expect(@abs(c2.getCurrentAngle() - c.getCurrentAngle()) < 1.0e-5);
        try testing.expectEqual(MotorState.off, c2.getMotorState());
    }

    // SaveState / RestoreState: enabled, the lambdas of the motor, rotation, point and limits parts, the motor state
    // and targets
    base.setupVelocityConstraint(1.0 / 60.0);
    base.warmStartVelocityConstraint(1.0);
    for (0..4) |_| _ = base.solveVelocityConstraint(1.0 / 60.0);
    c.setMotorState(.position);
    c.setTargetAngularVelocity(1.5);
    c.setTargetAngle(0.25);
    var recorder = StateRecorderImpl.init(allocator);
    defer recorder.deinit();
    base.saveState(recorder.recorder());
    try testing.expectEqual(@as(usize, 1 + 4 + 8 + 12 + 4 + 4 + 4 + 4), recorder.getData().len);
    const lambda_position = c.getTotalLambdaPosition();
    const lambda_rotation = c.getTotalLambdaRotation();
    const lambda_motor = c.getTotalLambdaMotor();
    try testing.expect(!lambda_position.eql(Vec3.zero()) and !lambda_rotation.isZero());
    try testing.expect(lambda_motor != 0.0); // Friction
    base.resetWarmStart();
    c.setMotorState(.off);
    c.setTargetAngularVelocity(0);
    c.setTargetAngle(0);
    try testing.expect(c.getTotalLambdaPosition().eql(Vec3.zero()) and c.getTotalLambdaRotation().isZero());
    try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaMotor());
    try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaRotationLimits());
    base.restoreState(recorder.recorder());
    try testing.expect(c.getTotalLambdaPosition().eql(lambda_position));
    try testing.expectEqual(lambda_rotation.getComponent(0), c.getTotalLambdaRotation().getComponent(0));
    try testing.expectEqual(lambda_rotation.getComponent(1), c.getTotalLambdaRotation().getComponent(1));
    try testing.expectEqual(lambda_motor, c.getTotalLambdaMotor());
    try testing.expectEqual(MotorState.position, c.getMotorState());
    try testing.expectEqual(@as(f32, 1.5), c.getTargetAngularVelocity());
    try testing.expectEqual(@as(f32, 0.25), c.getTargetAngle());

    // NotifyShapeChanged moves the attachment point of the body that changed
    const p1 = c.getLocalSpacePoint1();
    const p2 = c.getLocalSpacePoint2();
    base.notifyShapeChanged(body2.getID(), Vec3.init(0.5, 0, 0));
    try testing.expect(c.getLocalSpacePoint1().eql(p1) and c.getLocalSpacePoint2().eql(p2.sub(Vec3.init(0.5, 0, 0))));
    base.notifyShapeChanged(body1.getID(), Vec3.init(0, 1, 0));
    try testing.expect(c.getLocalSpacePoint1().eql(p1.sub(Vec3.init(0, 1, 0))));
    base.notifyShapeChanged(BodyID.init(12345), Vec3.init(1, 1, 1));
    try testing.expect(c.getLocalSpacePoint1().eql(p1.sub(Vec3.init(0, 1, 0))) and c.getLocalSpacePoint2().eql(p2.sub(Vec3.init(0.5, 0, 0))));

    // A disabled constraint is not active (TwoBodyConstraint::IsActive)
    try testing.expect(base.isActive());
    base.setEnabled(false);
    try testing.expect(!base.isActive());
}

test "HingeConstraint: setters, limits and the current angle" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const body = try context.createBody(.dynamic, .{ .position = RVec3.init(1, 2, 3), .rotation = Quat.rotation(Vec3.init(1, 1, 0).normalized(), 0.4) });
    defer context.destroyBodies(&.{body});

    // A hinge around the world Z axis at the center of mass of the body
    var settings = HingeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = body.getCenterOfMassPosition();
    settings.point2 = settings.point1;
    settings.hinge_axis1 = Vec3.axisZ();
    settings.hinge_axis2 = Vec3.axisZ();
    settings.normal_axis1 = Vec3.axisX();
    settings.normal_axis2 = Vec3.axisX();
    var c = HingeConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
    defer c.asConstraint().deinit();

    // No limits: the target angle is not clamped
    try testing.expect(!c.hasLimits());
    c.setTargetAngle(3.0);
    try testing.expectEqual(@as(f32, 3.0), c.getTargetAngle());
    c.setTargetAngle(-3.1);
    try testing.expectEqual(@as(f32, -3.1), c.getTargetAngle());

    // Limits: the target angle is clamped to them; limits at -pi / pi are no limits
    c.setLimits(-0.5, 1.0);
    try testing.expect(c.hasLimits());
    c.setTargetAngle(3.0);
    try testing.expectEqual(@as(f32, 1.0), c.getTargetAngle());
    c.setTargetAngle(-3.0);
    try testing.expectEqual(@as(f32, -0.5), c.getTargetAngle());
    c.setTargetAngle(0.25);
    try testing.expectEqual(@as(f32, 0.25), c.getTargetAngle());
    c.setLimits(-math.pi, 0.0);
    try testing.expect(c.hasLimits());
    c.setLimits(0.0, math.pi);
    try testing.expect(c.hasLimits());
    c.setLimits(-math.pi, math.pi);
    try testing.expect(!c.hasLimits());
    try testing.expectEqual(-math.pi, c.getLimitsMin());
    try testing.expectEqual(math.pi, c.getLimitsMax());

    // SetTargetOrientationBS: the angle of the orientation around the hinge axis
    c.setTargetOrientationBS(Quat.rotation(Vec3.axisZ(), 0.6));
    try testing.expect(@abs(c.getTargetAngle() - 0.6) < 1.0e-5);
    c.setLimits(-0.2, 0.3);
    c.setTargetOrientationBS(Quat.rotation(Vec3.axisZ(), 0.6));
    try testing.expectEqual(@as(f32, 0.3), c.getTargetAngle());
    c.setTargetOrientationBS(Quat.rotation(Vec3.axisZ(), -0.6));
    try testing.expectEqual(@as(f32, -0.2), c.getTargetAngle());

    // Friction, motor state, target velocity and the springs
    c.setMaxFrictionTorque(4.5);
    try testing.expectEqual(@as(f32, 4.5), c.getMaxFrictionTorque());
    c.setTargetAngularVelocity(-2.0);
    try testing.expectEqual(@as(f32, -2.0), c.getTargetAngularVelocity());
    for ([_]MotorState{ .velocity, .position, .position_and_velocity, .off }) |state| {
        c.setMotorState(state);
        try testing.expectEqual(state, c.getMotorState());
    }
    c.getLimitsSpringSettingsMut().damping = 0.75;
    try testing.expectEqual(@as(f32, 0.75), c.getLimitsSpringSettings().damping);
    c.getMotorSettingsMut().max_torque_limit = 12.0;
    try testing.expectEqual(@as(f32, 12.0), c.getMotorSettings().max_torque_limit);

    // GetCurrentAngle: the rotation of body 2 around the (world) hinge axis since the creation, in [-pi, pi]
    const rotation = body.getRotation();
    for ([_]f32{ 0.0, 0.3, -1.2, 2.5, -3.0 }) |angle| {
        body.setPositionAndRotationInternal(body.getPosition(), Quat.rotation(Vec3.axisZ(), angle).mul(rotation), .{});
        try testing.expect(@abs(c.getCurrentAngle() - angle) < 1.0e-5);
    }
    body.setPositionAndRotationInternal(body.getPosition(), Quat.rotation(Vec3.axisZ(), 3.5).mul(rotation), .{});
    try testing.expect(@abs(c.getCurrentAngle() - (3.5 - 2.0 * math.pi)) < 1.0e-5);
}

test "HingeConstraint: local space settings" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .kinematic, .dynamic });
    defer context.destroyBodies(&bodies);

    // Local space: the points and axes are used as they are
    var settings = HingeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.space = .local_to_body_com;
    settings.point1 = RVec3.init(0.25, 0.5, 0.75);
    settings.point2 = RVec3.init(-1, -2, -3);
    settings.hinge_axis1 = Vec3.axisX();
    settings.normal_axis1 = Vec3.axisY();
    settings.hinge_axis2 = Vec3.axisZ();
    settings.normal_axis2 = Vec3.axisX();
    var c = HingeConstraint.init(allocator, bodies[0], bodies[1], &settings);
    defer c.asConstraint().deinit();
    try testing.expect(c.getLocalSpacePoint1().eql(Vec3.init(0.25, 0.5, 0.75)) and c.getLocalSpacePoint2().eql(Vec3.init(-1, -2, -3)));
    try testing.expect(c.getLocalSpaceHingeAxis1().eql(Vec3.axisX()) and c.getLocalSpaceNormalAxis1().eql(Vec3.axisY()));
    try testing.expect(c.getLocalSpaceHingeAxis2().eql(Vec3.axisZ()) and c.getLocalSpaceNormalAxis2().eql(Vec3.axisX()));
    try testing.expect(c.inv_initial_orientation.eql(RotationEulerConstraintPart.getInvInitialOrientationXZ(Vec3.axisY(), Vec3.axisX(), Vec3.axisX(), Vec3.axisZ())));

    // Same frames on both bodies: the initial orientation is the identity
    settings.hinge_axis2 = settings.hinge_axis1;
    settings.normal_axis2 = settings.normal_axis1;
    var c2 = HingeConstraint.init(allocator, bodies[0], bodies[1], &settings);
    defer c2.asConstraint().deinit();
    try testing.expect(c2.inv_initial_orientation.isClose(Quat.identity(), .{}));
}

/// A body spinning with `angular_velocity` at (0, 2, 0) hinged to the world around the Y axis at its center of mass
const TestHinge = struct {
    settings: HingeConstraintSettings,
    body: *Body,
    c: HingeConstraint,

    fn init(self: *TestHinge, context: *TestContext.ConstraintPartTestContext, allocator: Allocator, angular_velocity: Vec3) !void {
        self.body = try context.createBody(.dynamic, .{ .position = RVec3.init(0, 2, 0), .angular_velocity = angular_velocity });
        self.settings = HingeConstraintSettings.init(allocator);
        self.settings.point1 = self.body.getCenterOfMassPosition();
        self.settings.point2 = self.settings.point1;
        self.c = HingeConstraint.init(allocator, Body.fixedToWorld(), self.body, &self.settings);
    }

    fn deinit(self: *TestHinge, context: *TestContext.ConstraintPartTestContext) void {
        self.c.asConstraint().deinit();
        self.settings.asSettings().deinit();
        context.destroyBodies(&.{self.body});
    }
};

test "HingeConstraint: solver with limits, a motor in every state and friction" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const dt: f32 = 1.0 / 60.0;

    // No limits, no motor, no friction: only the point and rotation parts act, the rotation around the hinge axis stays
    {
        var h: TestHinge = undefined;
        try h.init(&context, allocator, Vec3.init(0.5, 2.0, -0.25));
        defer h.deinit(&context);
        const base = h.c.asConstraint();
        base.setupVelocityConstraint(dt);
        try testing.expect(!h.c.motor_constraint_part.isActive() and !h.c.rotation_limits_constraint_part.isActive());
        base.warmStartVelocityConstraint(1.0);
        try testing.expect(base.solveVelocityConstraint(dt));
        for (0..10) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(h.body.getAngularVelocity().isClose(Vec3.init(0, 2.0, 0), .{ .max_dist_sq = 1.0e-8 }));
        try testing.expectEqual(@as(f32, 0.0), h.c.getTotalLambdaMotor());
        try testing.expectEqual(@as(f32, 0.0), h.c.getTotalLambdaRotationLimits());
    }

    // Friction: the rotation around the hinge axis stops when the friction torque is high enough
    {
        var h: TestHinge = undefined;
        try h.init(&context, allocator, Vec3.init(0, 2.0, 0));
        defer h.deinit(&context);
        h.c.setMaxFrictionTorque(1.0e6);
        const base = h.c.asConstraint();
        base.setupVelocityConstraint(dt);
        try testing.expect(h.c.motor_constraint_part.isActive());
        base.warmStartVelocityConstraint(1.0);
        for (0..10) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(@abs(relativeAngularVelocity(&h.c)) < 1.0e-4);
        try testing.expect(h.c.getTotalLambdaMotor() < 0.0);

        // Weak friction only slows it down: the impulse is limited by the friction torque
        h.c.setMaxFrictionTorque(0.1);
        h.body.setAngularVelocity(Vec3.init(0, 2.0, 0));
        base.resetWarmStart();
        base.setupVelocityConstraint(dt);
        for (0..10) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(@abs(h.c.getTotalLambdaMotor() + 0.1 * dt) < 1.0e-7);
        try testing.expect(relativeAngularVelocity(&h.c) > 1.0);
    }

    // Velocity motor: the relative angular velocity around the hinge axis becomes the target
    {
        var h: TestHinge = undefined;
        try h.init(&context, allocator, Vec3.zero());
        defer h.deinit(&context);
        h.c.setMotorState(.velocity);
        h.c.setTargetAngularVelocity(1.5);
        const base = h.c.asConstraint();
        base.setupVelocityConstraint(dt);
        base.warmStartVelocityConstraint(1.0);
        for (0..10) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(@abs(relativeAngularVelocity(&h.c) - 1.5) < 1.0e-4);

        // The torque limits of the motor limit the impulse
        h.c.getMotorSettingsMut().setTorqueLimit(0.01);
        h.body.setAngularVelocity(Vec3.zero());
        base.resetWarmStart();
        base.setupVelocityConstraint(dt);
        for (0..10) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(@abs(h.c.getTotalLambdaMotor() - 0.01 * dt) < 1.0e-7);
    }

    // Position motor (and position and velocity motor): drives towards the target angle; without stiffness (position)
    // or without stiffness and damping (position and velocity) the motor part is off
    for ([_]MotorState{ .position, .position_and_velocity }) |state| {
        var h: TestHinge = undefined;
        try h.init(&context, allocator, Vec3.zero());
        defer h.deinit(&context);
        h.c.setMotorState(state);
        h.c.setTargetAngle(0.5);
        const base = h.c.asConstraint();
        base.setupVelocityConstraint(dt);
        try testing.expect(h.c.motor_constraint_part.isActive());
        base.warmStartVelocityConstraint(1.0);
        for (0..10) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(relativeAngularVelocity(&h.c) > 0.0);
        try testing.expect(h.c.getTotalLambdaMotor() > 0.0);

        h.c.getMotorSettingsMut().spring_settings = .init(.stiffness_and_damping, 0.0, 0.0);
        base.setupVelocityConstraint(dt);
        try testing.expect(!h.c.motor_constraint_part.isActive());
        h.c.getMotorSettingsMut().spring_settings = .init(.stiffness_and_damping, 0.0, 1.0);
        base.setupVelocityConstraint(dt);
        try testing.expectEqual(state == .position_and_velocity, h.c.motor_constraint_part.isActive());
    }

    // Limits: outside the limits the limit part pushes back (the closest limit decides the direction), inside it is off
    for ([_]f32{ 0.8, -0.8 }) |angle| {
        var h: TestHinge = undefined;
        try h.init(&context, allocator, Vec3.zero());
        defer h.deinit(&context);
        h.c.setLimits(-0.5, 0.5);
        const base = h.c.asConstraint();
        base.setupVelocityConstraint(dt);
        try testing.expect(!h.c.rotation_limits_constraint_part.isActive());

        // Rotate the body past the limit, moving away from it
        h.body.setPositionAndRotationInternal(h.body.getPosition(), Quat.rotation(Vec3.axisY(), angle), .{});
        h.body.setAngularVelocity(Vec3.init(0, angle, 0));
        base.setupVelocityConstraint(dt);
        try testing.expect(h.c.rotation_limits_constraint_part.isActive());
        try testing.expect(@abs(h.c.theta - angle) < 1.0e-5);
        try testing.expect(@abs(h.c.getSmallestAngleToLimit() - (angle - math.sign(angle) * 0.5)) < 1.0e-5);
        try testing.expectEqual(angle < 0.0, h.c.isMinLimitClosest());
        base.warmStartVelocityConstraint(1.0);
        for (0..10) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(h.c.getTotalLambdaRotationLimits() * angle < 0.0);
        try testing.expect(relativeAngularVelocity(&h.c) * angle <= 1.0e-5); // Not moving further out

        // The position solver rotates it back towards the limit
        const before = h.c.getCurrentAngle();
        try testing.expect(base.solvePositionConstraint(dt, 0.5));
        try testing.expect(@abs(h.c.getCurrentAngle()) < @abs(before));

        // A soft limit (spring) is not corrected in the position solver
        h.c.setLimitsSpringSettings(.init(.frequency_and_damping, 2.0, 0.5));
        base.setupVelocityConstraint(dt);
        try testing.expect(h.c.rotation_limits_constraint_part.isActive());
        const soft_before = h.c.getCurrentAngle();
        _ = base.solvePositionConstraint(dt, 0.5);
        try testing.expect(@abs(h.c.getCurrentAngle() - soft_before) < 1.0e-4);
    }

    // Equal limits (with a spring): both directions are allowed
    {
        var h: TestHinge = undefined;
        try h.init(&context, allocator, Vec3.init(0, 1, 0));
        defer h.deinit(&context);
        h.c.setLimits(0.0, 0.0);
        h.c.setLimitsSpringSettings(.init(.frequency_and_damping, 5.0, 1.0));
        const base = h.c.asConstraint();
        h.body.setPositionAndRotationInternal(h.body.getPosition(), Quat.rotation(Vec3.axisY(), -0.25), .{});
        base.setupVelocityConstraint(dt);
        try testing.expect(h.c.rotation_limits_constraint_part.isActive());
        for (0..10) |_| _ = base.solveVelocityConstraint(dt);
        try testing.expect(h.c.getTotalLambdaRotationLimits() != 0.0);
    }
}

test "HingeConstraint: body 1 fixed to the world, static and kinematic bodies" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        var settings = HingeConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.point1 = RVec3.init(1.5, 1.75, 3.5);
        settings.point2 = settings.point1;
        settings.limits_min = -0.1;
        settings.limits_max = 0.1;
        settings.max_friction_torque = 1.0;
        var c = HingeConstraint.init(allocator, bodies[0], bodies[1], &settings);
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
    var settings = HingeConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.point1 = body.getCenterOfMassPosition();
    settings.point2 = settings.point1;
    var c = HingeConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
    defer c.asConstraint().deinit();
    try testing.expect(c.getLocalSpacePoint2().isNearZero(.{}));
    try testing.expect(c.getLocalSpacePoint1().isClose(body.getCenterOfMassPosition().toVec3(), .{}));
    const base = c.asConstraint();
    base.setupVelocityConstraint(1.0 / 60.0);
    base.warmStartVelocityConstraint(1.0);
    try testing.expect(base.solveVelocityConstraint(1.0 / 60.0));
    try testing.expect(body.getLinearVelocity().isNearZero(.{ .max_dist_sq = 1.0e-10 }));
}

test "HingeConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const bodies = try context.createBodyPair(.{ .dynamic, .static });
            defer context.destroyBodies(&bodies);
            var settings = Ref(ConstraintSettings).init((try HingeConstraintSettings.create(allocator)).asSettings());
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
