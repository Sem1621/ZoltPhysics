//! Port of: Jolt/Physics/Constraints/SliderConstraint.h, Jolt/Physics/Constraints/SliderConstraint.cpp
//! Status: complete
//!
//! Follows the porter template in FixedConstraint.zig. JPH_DECLARE_SERIALIZABLE_VIRTUAL /
//! JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(SliderConstraintSettings): `rtti` with Jolt's attributes.
//! JPH_DEBUG_RENDERER (DrawConstraint, DrawConstraintLimits): gated declarations (RemainingFeaturesArchitecture.md R2).
//!
//! - The const and non-const overloads `GetMotorSettings()` / `GetLimitsSpringSettings()` are `getMotorSettings` /
//!   `getLimitsSpringSettings` (`*const`) and `getMotorSettingsMut` / `getLimitsSpringSettingsMut` (`*`).
//! - `GetTotalLambdaPosition` returns Jolt's `Vector<2>` by value (`Vector(2)`).
//! - `EMotorState` accepts every int (MotorSettings.zig): the switches on the motor state have a `_` prong that does
//!   what Jolt's switch does when no case matches (nothing).
//! - The asserts on user input (SetSliderAxis outside world space, the limits of SetLimits, the "Better use a fixed
//!   constraint" check of the constructor, a motor state with invalid motor settings) only run with
//!   `Core.enable_asserts`, so ReleaseFast keeps Jolt's release behavior.
//! - RackAndPinionConstraint reads the slider through `getCurrentPosition` (Jolt's `GetCurrentPosition`, called on a
//!   `*const SliderConstraint` after checking `getSubType() == .slider`).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const Core = @import("../../Core/Core.zig");
const Ref = @import("../../Core/Reference.zig").Ref;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
const math = @import("../../Math/Math.zig");
const Vec3 = @import("../../Math/Vec3.zig").Vec3;
const Color = @import("../../Core/Color.zig").Color;
const DebugRenderer = @import("../../Renderer/DebugRenderer.zig").DebugRenderer;
const isVelocityMotor = MotorSettingsFile.isVelocityMotor;
const isPositionMotor = MotorSettingsFile.isPositionMotor;
const Vec4 = @import("../../Math/Vec4.zig").Vec4;
const Quat = @import("../../Math/Quat.zig").Quat;
const Mat44 = @import("../../Math/Mat44.zig").Mat44;
const Vector = @import("../../Math/Vector.zig").Vector;
const RealFile = @import("../../Math/Real.zig");
const Real = RealFile.Real;
const RVec3 = RealFile.RVec3;
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
const DualAxisConstraintPart = @import("ConstraintPart/DualAxisConstraintPart.zig").DualAxisConstraintPart;
const RotationEulerConstraintPart = @import("ConstraintPart/RotationEulerConstraintPart.zig").RotationEulerConstraintPart;
const AxisConstraintPart = @import("ConstraintPart/AxisConstraintPart.zig").AxisConstraintPart;

/// Slider constraint settings, used to create a slider constraint
pub const SliderConstraintSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(SliderConstraintSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "SliderConstraintSettings",
        .kind = .virtual,
        .bases = &.{TwoBodyConstraintSettings},
        .attributes = &.{
            .enumAttr("mSpace", .space),
            .attr("mAutoDetectPoint", .auto_detect_point),
            .attr("mPoint1", .point1),
            .attr("mSliderAxis1", .slider_axis1),
            .attr("mNormalAxis1", .normal_axis1),
            .attr("mPoint2", .point2),
            .attr("mSliderAxis2", .slider_axis2),
            .attr("mNormalAxis2", .normal_axis2),
            .attr("mLimitsMin", .limits_min),
            .attr("mLimitsMax", .limits_max),
            .enumAlias("mSpringMode", .{ .limits_spring_settings, .mode }),
            .alias("mFrequency", .{ .limits_spring_settings, .frequency }), // Renaming attributes to stay compatible with old versions of the library
            .alias("mDamping", .{ .limits_spring_settings, .damping }),
            .attr("mMaxFrictionForce", .max_friction_force),
            .attr("mMotorSettings", .motor_settings),
        },
    });

    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// This determines in which space the constraint is setup, all properties below should be in the specified space
    space: ConstraintSpace = .world_space,

    /// When mSpace is WorldSpace mPoint1 and mPoint2 can be automatically calculated based on the positions of the bodies when the constraint is created (the current relative position/orientation is chosen as the '0' position). Set this to false if you want to supply the attachment points yourself.
    auto_detect_point: bool = false,

    /// Body 1 constraint reference frame (space determined by mSpace).
    /// Slider axis is the axis along which movement is possible (direction), normal axis is a perpendicular vector to define the frame.
    point1: RVec3 = RVec3.zero(),
    slider_axis1: Vec3 = Vec3.axisX(),
    normal_axis1: Vec3 = Vec3.axisY(),

    /// Body 2 constraint reference frame (space determined by mSpace)
    point2: RVec3 = RVec3.zero(),
    slider_axis2: Vec3 = Vec3.axisX(),
    normal_axis2: Vec3 = Vec3.axisY(),

    /// When the bodies move so that mPoint1 coincides with mPoint2 the slider position is defined to be 0, movement will be limited between [mLimitsMin, mLimitsMax] where mLimitsMin e [-inf, 0] and mLimitsMax e [0, inf]
    limits_min: f32 = -math.flt_max,
    limits_max: f32 = math.flt_max,

    /// When enabled, this makes the limits soft. When the constraint exceeds the limits, a spring force will pull it back.
    limits_spring_settings: SpringSettings = .{},

    /// Maximum amount of friction force to apply (N) when not driven by a motor.
    max_friction_force: f32 = 0.0,

    /// In case the constraint is powered, this determines the motor settings around the sliding axis
    motor_settings: MotorSettings = .{},

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) SliderConstraintSettings {
        return .{ .base = .init(SliderConstraintSettings, allocator) };
    }

    /// new SliderConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*SliderConstraintSettings {
        const self = try allocator.create(SliderConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other`, not its reference count
    pub fn initCopy(allocator: Allocator, other: *const SliderConstraintSettings) SliderConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        return result;
    }

    pub fn asSettings(self: *SliderConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const SliderConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *SliderConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const SliderConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const SliderConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.space);
        stream.write(self.auto_detect_point);
        stream.write(self.point1);
        stream.write(self.slider_axis1);
        stream.write(self.normal_axis1);
        stream.write(self.point2);
        stream.write(self.slider_axis2);
        stream.write(self.normal_axis2);
        stream.write(self.limits_min);
        stream.write(self.limits_max);
        stream.write(self.max_friction_force);
        self.limits_spring_settings.saveBinaryState(stream);
        self.motor_settings.saveBinaryState(stream);
    }

    /// Create an instance of this constraint.
    /// Note that the rotation constraint will be solved from body 1. This means that if body 1 and body 2 have different masses / inertias (kinematic body = infinite mass / inertia), body 1 should be the heaviest body.
    pub fn createConstraint(self: *const SliderConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        const constraint = try SliderConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    /// Simple way of setting the slider and normal axis in world space (assumes the bodies are already oriented correctly when the constraint is created)
    pub fn setSliderAxis(self: *SliderConstraintSettings, slider_axis: Vec3) void {
        // Settings of the user: Jolt's release build continues
        if (Core.enable_asserts) std.debug.assert(self.space == .world_space);

        self.slider_axis1 = slider_axis;
        self.slider_axis2 = slider_axis;
        const normal_axis = slider_axis.getNormalizedPerpendicular();
        self.normal_axis1 = normal_axis;
        self.normal_axis2 = normal_axis;
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *SliderConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.space);
        stream.read(&self.auto_detect_point);
        stream.read(&self.point1);
        stream.read(&self.slider_axis1);
        stream.read(&self.normal_axis1);
        stream.read(&self.point2);
        stream.read(&self.slider_axis2);
        stream.read(&self.normal_axis2);
        stream.read(&self.limits_min);
        stream.read(&self.limits_max);
        stream.read(&self.max_friction_force);
        self.limits_spring_settings.restoreBinaryState(stream);
        self.motor_settings.restoreBinaryState(stream);
    }
};

/// A slider constraint allows movement in only 1 axis (and no rotation). Also known as a prismatic constraint.
pub const SliderConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .drawConstraint, .drawConstraintLimits, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,

    // CONFIGURATION PROPERTIES FOLLOW

    // Local space constraint positions
    local_space_position1: Vec3 = undefined,
    local_space_position2: Vec3 = undefined,

    // Local space sliding direction
    local_space_slider_axis1: Vec3 = undefined,

    // Local space normals to the sliding direction (in body 1 space)
    local_space_normal1: Vec3 = undefined,
    local_space_normal2: Vec3 = undefined,

    // Inverse of initial rotation from body 1 to body 2 in body 1 space
    inv_initial_orientation: Quat = undefined,

    // Slider limits
    has_limits: bool = undefined,
    limits_min: f32 = undefined,
    limits_max: f32 = undefined,

    // Soft constraint limits
    limits_spring_settings: SpringSettings = undefined,

    // Friction
    max_friction_force: f32,

    // Motor controls
    motor_settings: MotorSettings,
    motor_state: MotorState = .off,
    target_velocity: f32 = 0.0,
    target_position: f32 = 0.0,

    // RUN TIME PROPERTIES FOLLOW

    // Positions where the point constraint acts on (middle point between center of masses)
    r1: Vec3 = undefined,
    r2: Vec3 = undefined,

    // X2 + R2 - X1 - R1
    u: Vec3 = undefined,

    // World space sliding direction
    world_space_slider_axis: Vec3 = undefined,

    // Normals to the slider axis
    n1: Vec3 = undefined,
    n2: Vec3 = undefined,

    // Distance along the slide axis
    d: f32 = 0.0,

    // The constraint parts
    position_constraint_part: DualAxisConstraintPart = .{},
    rotation_constraint_part: RotationEulerConstraintPart = .{},
    position_limits_constraint_part: AxisConstraintPart = .{},
    motor_constraint_part: AxisConstraintPart = .{},

    /// Construct slider constraint
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const SliderConstraintSettings) SliderConstraint {
        var self: SliderConstraint = .{
            .base = .init(SliderConstraint, allocator, body1, body2, &settings.base),
            .max_friction_force = settings.max_friction_force,
            .motor_settings = settings.motor_settings,
        };

        // Store inverse of initial rotation from body 1 to body 2 in body 1 space
        self.inv_initial_orientation = RotationEulerConstraintPart.getInvInitialOrientationXY(settings.slider_axis1, settings.normal_axis1, settings.slider_axis2, settings.normal_axis2);

        if (settings.space == .world_space) {
            const inv_transform1 = body1.getInverseCenterOfMassTransform();
            const inv_transform2 = body2.getInverseCenterOfMassTransform();

            if (settings.auto_detect_point) {
                // Determine anchor point: If any of the bodies can never be dynamic use the other body as anchor point
                var anchor: RVec3 = undefined;
                if (!body1.canBeKinematicOrDynamic()) {
                    anchor = body2.getCenterOfMassPosition();
                } else if (!body2.canBeKinematicOrDynamic()) {
                    anchor = body1.getCenterOfMassPosition();
                } else {
                    // Otherwise use weighted anchor point towards the lightest body
                    const inv_m1: Real = body1.getMotionPropertiesUnchecked().?.getInverseMassUnchecked();
                    const inv_m2: Real = body2.getMotionPropertiesUnchecked().?.getInverseMassUnchecked();
                    const total_inv_mass = inv_m1 + inv_m2;
                    if (total_inv_mass != 0.0)
                        anchor = body1.getCenterOfMassPosition().mulScalar(inv_m1).add(body2.getCenterOfMassPosition().mulScalar(inv_m2)).divScalar(total_inv_mass)
                    else
                        anchor = body1.getCenterOfMassPosition();
                }

                // Store local positions
                self.local_space_position1 = inv_transform1.mulRVec3(anchor).toVec3();
                self.local_space_position2 = inv_transform2.mulRVec3(anchor).toVec3();
            } else {
                // Store local positions
                self.local_space_position1 = inv_transform1.mulRVec3(settings.point1).toVec3();
                self.local_space_position2 = inv_transform2.mulRVec3(settings.point2).toVec3();
            }

            // If all properties were specified in world space, take them to local space now
            self.local_space_slider_axis1 = inv_transform1.multiply3x3(settings.slider_axis1).normalized();
            self.local_space_normal1 = inv_transform1.multiply3x3(settings.normal_axis1).normalized();

            // Constraints were specified in world space, so we should have replaced c1 with q10^-1 c1 and c2 with q20^-1 c2
            // => r0^-1 = (q20^-1 c2) (q10^-1 c1)^1 = q20^-1 (c2 c1^-1) q10
            self.inv_initial_orientation = body2.getRotation().conjugated().mul(self.inv_initial_orientation).mul(body1.getRotation());
        } else {
            // Store local positions
            self.local_space_position1 = settings.point1.toVec3();
            self.local_space_position2 = settings.point2.toVec3();

            // Store local space axis
            self.local_space_slider_axis1 = settings.slider_axis1;
            self.local_space_normal1 = settings.normal_axis1;
        }

        // Calculate 2nd local space normal
        self.local_space_normal2 = self.local_space_slider_axis1.cross(self.local_space_normal1);

        // Store limits
        // Settings of the user: Jolt's release build continues
        if (Core.enable_asserts) std.debug.assert(settings.limits_min != settings.limits_max or settings.limits_spring_settings.frequency > 0.0); // Better use a fixed constraint
        self.setLimits(settings.limits_min, settings.limits_max);

        // Store spring settings
        self.setLimitsSpringSettings(settings.limits_spring_settings);

        return self;
    }

    /// new SliderConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const SliderConstraintSettings) Allocator.Error!*SliderConstraint {
        const self = try allocator.create(SliderConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    pub fn asConstraint(self: *SliderConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const SliderConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *SliderConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const SliderConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    // Generic interface of a constraint
    pub fn getSubType(self: *const SliderConstraint) ConstraintSubType {
        _ = self;
        return .slider;
    }

    pub fn notifyShapeChanged(self: *SliderConstraint, body_id: BodyID, delta_com: Vec3) void {
        if (self.base.body1.getID().eql(body_id)) {
            self.local_space_position1 = self.local_space_position1.sub(delta_com);
        } else if (self.base.body2.getID().eql(body_id)) {
            self.local_space_position2 = self.local_space_position2.sub(delta_com);
        }
    }

    /// Get the current distance from the rest position
    pub fn getCurrentPosition(self: *const SliderConstraint) f32 {
        // See: CalculateR1R2U and CalculateSlidingAxisAndPosition
        const body1 = self.base.body1;
        const body2 = self.base.body2;
        const r1 = body1.getRotation().mulVec3(self.local_space_position1);
        const r2 = body2.getRotation().mulVec3(self.local_space_position2);
        const u = body2.getCenterOfMassPosition().sub(body1.getCenterOfMassPosition()).toVec3().add(r2).sub(r1);
        return u.dot(body1.getRotation().mulVec3(self.local_space_slider_axis1));
    }

    /// Friction control
    pub fn setMaxFrictionForce(self: *SliderConstraint, friction_force: f32) void {
        self.max_friction_force = friction_force;
    }

    pub fn getMaxFrictionForce(self: *const SliderConstraint) f32 {
        return self.max_friction_force;
    }

    /// Motor settings
    pub fn getMotorSettingsMut(self: *SliderConstraint) *MotorSettings {
        return &self.motor_settings;
    }

    pub fn getMotorSettings(self: *const SliderConstraint) *const MotorSettings {
        return &self.motor_settings;
    }

    // Motor controls
    pub fn setMotorState(self: *SliderConstraint, state: MotorState) void {
        // Settings of the user: Jolt's release build continues
        if (Core.enable_asserts) std.debug.assert(state == .off or self.motor_settings.isValid());
        self.motor_state = state;
    }

    pub fn getMotorState(self: *const SliderConstraint) MotorState {
        return self.motor_state;
    }

    pub fn setTargetVelocity(self: *SliderConstraint, velocity: f32) void {
        self.target_velocity = velocity;
    }

    pub fn getTargetVelocity(self: *const SliderConstraint) f32 {
        return self.target_velocity;
    }

    pub fn setTargetPosition(self: *SliderConstraint, position: f32) void {
        self.target_position = if (self.has_limits) math.clamp(position, self.limits_min, self.limits_max) else position;
    }

    pub fn getTargetPosition(self: *const SliderConstraint) f32 {
        return self.target_position;
    }

    /// Update the limits of the slider constraint (see SliderConstraintSettings)
    pub fn setLimits(self: *SliderConstraint, limits_min: f32, limits_max: f32) void {
        // Settings of the user: Jolt's release build continues
        if (Core.enable_asserts) std.debug.assert(limits_min <= 0.0);
        if (Core.enable_asserts) std.debug.assert(limits_max >= 0.0);
        self.limits_min = limits_min;
        self.limits_max = limits_max;
        self.has_limits = self.limits_min != -math.flt_max or self.limits_max != math.flt_max;
    }

    pub fn getLimitsMin(self: *const SliderConstraint) f32 {
        return self.limits_min;
    }

    pub fn getLimitsMax(self: *const SliderConstraint) f32 {
        return self.limits_max;
    }

    pub fn hasLimits(self: *const SliderConstraint) bool {
        return self.has_limits;
    }

    /// Update the limits spring settings
    pub fn getLimitsSpringSettings(self: *const SliderConstraint) *const SpringSettings {
        return &self.limits_spring_settings;
    }

    pub fn getLimitsSpringSettingsMut(self: *SliderConstraint) *SpringSettings {
        return &self.limits_spring_settings;
    }

    pub fn setLimitsSpringSettings(self: *SliderConstraint, limits_spring_settings: SpringSettings) void {
        self.limits_spring_settings = limits_spring_settings;
    }

    ///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
    pub fn getTotalLambdaPosition(self: *const SliderConstraint) Vector(2) {
        return self.position_constraint_part.getTotalLambda().*;
    }

    pub fn getTotalLambdaPositionLimits(self: *const SliderConstraint) f32 {
        return self.position_limits_constraint_part.getTotalLambda();
    }

    pub fn getTotalLambdaRotation(self: *const SliderConstraint) Vec3 {
        return self.rotation_constraint_part.getTotalLambda();
    }

    pub fn getTotalLambdaMotor(self: *const SliderConstraint) f32 {
        return self.motor_constraint_part.getTotalLambda();
    }

    // Internal helper function to calculate the values below
    fn calculateR1R2U(self: *SliderConstraint, rotation1: Mat44, rotation2: Mat44) void {
        // Calculate points relative to body
        self.r1 = rotation1.mulVec3(self.local_space_position1);
        self.r2 = rotation2.mulVec3(self.local_space_position2);

        // Calculate X2 + R2 - X1 - R1
        self.u = self.base.body2.getCenterOfMassPosition().sub(self.base.body1.getCenterOfMassPosition()).toVec3().add(self.r2).sub(self.r1);
    }

    fn calculatePositionConstraintProperties(self: *SliderConstraint, rotation1: Mat44, rotation2: Mat44) void {
        // Calculate world space normals
        self.n1 = rotation1.mulVec3(self.local_space_normal1);
        self.n2 = rotation1.mulVec3(self.local_space_normal2);

        self.position_constraint_part.calculateConstraintProperties(self.base.body1, rotation1, self.r1.add(self.u), self.base.body2, rotation2, self.r2, self.n1, self.n2);
    }

    fn calculateSlidingAxisAndPosition(self: *SliderConstraint, rotation1: Mat44) void {
        if (self.has_limits or self.motor_state != .off or self.max_friction_force > 0.0) {
            // Calculate world space slider axis
            self.world_space_slider_axis = rotation1.mulVec3(self.local_space_slider_axis1);

            // Calculate slide distance along axis
            self.d = self.u.dot(self.world_space_slider_axis);
        }
    }

    fn calculatePositionLimitsConstraintProperties(self: *SliderConstraint, delta_time: f32) void {
        // Check if distance is within limits
        const below_min = self.d <= self.limits_min;
        if (self.has_limits and (below_min or self.d >= self.limits_max))
            self.position_limits_constraint_part.calculateConstraintPropertiesWithSettingsForLimit(delta_time, self.base.body1, self.r1.add(self.u), self.base.body2, self.r2, self.world_space_slider_axis, 0.0, self.d - (if (below_min) self.limits_min else self.limits_max), &self.limits_spring_settings)
        else
            self.position_limits_constraint_part.deactivate();
    }

    fn calculateMotorConstraintProperties(self: *SliderConstraint, delta_time: f32) void {
        switch (self.motor_state) {
            .off => {
                if (self.max_friction_force > 0.0)
                    self.motor_constraint_part.calculateConstraintProperties(self.base.body1, self.r1.add(self.u), self.base.body2, self.r2, self.world_space_slider_axis, .{})
                else
                    self.motor_constraint_part.deactivate();
            },

            .velocity => {
                self.motor_constraint_part.calculateConstraintProperties(self.base.body1, self.r1.add(self.u), self.base.body2, self.r2, self.world_space_slider_axis, .{ .bias = -self.target_velocity });
            },

            .position => {
                if (self.motor_settings.spring_settings.hasStiffness())
                    self.motor_constraint_part.calculateConstraintPropertiesWithSettingsForMotor(delta_time, self.base.body1, self.r1.add(self.u), self.base.body2, self.r2, self.world_space_slider_axis, 0.0, self.d - self.target_position, &self.motor_settings.spring_settings)
                else
                    self.motor_constraint_part.deactivate();
            },

            .position_and_velocity => {
                if (self.motor_settings.spring_settings.hasStiffnessOrDamping())
                    self.motor_constraint_part.calculateConstraintPropertiesWithSettingsForMotor(delta_time, self.base.body1, self.r1.add(self.u), self.base.body2, self.r2, self.world_space_slider_axis, -self.target_velocity, self.d - self.target_position, &self.motor_settings.spring_settings)
                else
                    self.motor_constraint_part.deactivate();
            },

            // A restored int that is no EMotorState: no case of Jolt's switch matches
            _ => {},
        }
    }

    pub fn setupVelocityConstraint(self: *SliderConstraint, delta_time: f32) void {
        // Calculate constraint properties that are constant while bodies don't move
        const rotation1 = Mat44.rotationQuat(self.base.body1.getRotation());
        const rotation2 = Mat44.rotationQuat(self.base.body2.getRotation());
        self.calculateR1R2U(rotation1, rotation2);
        self.calculatePositionConstraintProperties(rotation1, rotation2);
        self.rotation_constraint_part.calculateConstraintProperties(self.base.body1, rotation1, self.base.body2, rotation2);
        self.calculateSlidingAxisAndPosition(rotation1);
        self.calculatePositionLimitsConstraintProperties(delta_time);
        self.calculateMotorConstraintProperties(delta_time);
    }

    pub fn resetWarmStart(self: *SliderConstraint) void {
        self.motor_constraint_part.deactivate();
        self.position_constraint_part.deactivate();
        self.rotation_constraint_part.deactivate();
        self.position_limits_constraint_part.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *SliderConstraint, warm_start_impulse_ratio: f32) void {
        // Warm starting: Apply previous frame impulse
        self.motor_constraint_part.warmStart(self.base.body1, self.base.body2, self.world_space_slider_axis, warm_start_impulse_ratio);
        self.position_constraint_part.warmStart(self.base.body1, self.base.body2, self.n1, self.n2, warm_start_impulse_ratio);
        self.rotation_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
        self.position_limits_constraint_part.warmStart(self.base.body1, self.base.body2, self.world_space_slider_axis, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *SliderConstraint, delta_time: f32) bool {
        // Solve motor
        var motor = false;
        if (self.motor_constraint_part.isActive()) {
            switch (self.motor_state) {
                .off => {
                    const max_lambda = self.max_friction_force * delta_time;
                    motor = self.motor_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2, self.world_space_slider_axis, -max_lambda, max_lambda);
                },

                .velocity, .position, .position_and_velocity => {
                    motor = self.motor_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2, self.world_space_slider_axis, delta_time * self.motor_settings.min_force_limit, delta_time * self.motor_settings.max_force_limit);
                },

                // A restored int that is no EMotorState: no case of Jolt's switch matches
                _ => {},
            }
        }

        // Solve position constraint along 2 axis
        const pos = self.position_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2, self.n1, self.n2);

        // Solve rotation constraint
        const rot = self.rotation_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2);

        // Solve limits along slider axis
        var limit = false;
        if (self.position_limits_constraint_part.isActive()) {
            var min_lambda: f32 = undefined;
            var max_lambda: f32 = undefined;
            if (self.limits_min == self.limits_max) {
                min_lambda = -math.flt_max;
                max_lambda = math.flt_max;
            } else if (self.d <= self.limits_min) {
                min_lambda = 0.0;
                max_lambda = math.flt_max;
            } else {
                min_lambda = -math.flt_max;
                max_lambda = 0.0;
            }
            limit = self.position_limits_constraint_part.solveVelocityConstraint(self.base.body1, self.base.body2, self.world_space_slider_axis, min_lambda, max_lambda);
        }

        return motor or pos or rot or limit;
    }

    pub fn solvePositionConstraint(self: *SliderConstraint, delta_time: f32, baumgarte: f32) bool {
        // Motor operates on velocities only, don't call SolvePositionConstraint

        // Solve position constraint along 2 axis
        var rotation1 = Mat44.rotationQuat(self.base.body1.getRotation());
        var rotation2 = Mat44.rotationQuat(self.base.body2.getRotation());
        self.calculateR1R2U(rotation1, rotation2);
        self.calculatePositionConstraintProperties(rotation1, rotation2);
        const pos = self.position_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, self.u, self.n1, self.n2, baumgarte);

        // Solve rotation constraint
        self.rotation_constraint_part.calculateConstraintProperties(self.base.body1, Mat44.rotationQuat(self.base.body1.getRotation()), self.base.body2, Mat44.rotationQuat(self.base.body2.getRotation()));
        const rot = self.rotation_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, self.inv_initial_orientation, baumgarte);

        // Solve limits along slider axis
        var limit = false;
        if (self.has_limits and self.limits_spring_settings.frequency <= 0.0) {
            rotation1 = Mat44.rotationQuat(self.base.body1.getRotation());
            rotation2 = Mat44.rotationQuat(self.base.body2.getRotation());
            self.calculateR1R2U(rotation1, rotation2);
            self.calculateSlidingAxisAndPosition(rotation1);
            self.calculatePositionLimitsConstraintProperties(delta_time);
            if (self.position_limits_constraint_part.isActive()) {
                if (self.d <= self.limits_min)
                    limit = self.position_limits_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, self.world_space_slider_axis, self.d - self.limits_min, baumgarte)
                else {
                    std.debug.assert(self.d >= self.limits_max);
                    limit = self.position_limits_constraint_part.solvePositionConstraint(self.base.body1, self.base.body2, self.world_space_slider_axis, self.d - self.limits_max, baumgarte);
                }
            }
        }

        return pos or rot or limit;
    }

    // See Constraint::DrawConstraint (JPH_DEBUG_RENDERER)
    pub const drawConstraint = if (Core.debug_renderer) drawConstraintImpl else {};
    fn drawConstraintImpl(self: *const SliderConstraint, renderer: *DebugRenderer) void {
        const transform1 = self.base.body1.getCenterOfMassTransform();
        const transform2 = self.base.body2.getCenterOfMassTransform();

        // Transform the local positions into world space
        const slider_axis = transform1.multiply3x3(self.local_space_slider_axis1);
        const position1 = transform1.mulVec3(self.local_space_position1);
        const position2 = transform2.mulVec3(self.local_space_position2);

        // Draw constraint
        renderer.drawMarker(position1, Color.red, 0.1);
        renderer.drawMarker(position2, Color.green, 0.1);
        renderer.drawLine(position1, position2, Color.green);

        // Draw motor
        if (isPositionMotor(self.motor_state))
            renderer.drawMarker(position1.addVec3(slider_axis.mulScalar(self.target_position)), Color.yellow, 1.0);

        if (isVelocityMotor(self.motor_state)) {
            const cur_vel = slider_axis.mulScalar(self.base.body2.getLinearVelocity().sub(self.base.body1.getLinearVelocity()).dot(slider_axis));
            renderer.drawLine(position2, position2.addVec3(cur_vel), Color.blue);
            renderer.drawArrow(position2.addVec3(cur_vel), position2.addVec3(slider_axis.mulScalar(self.target_velocity)), Color.red, 0.1);
        }
    }

    // See Constraint::DrawConstraintLimits (JPH_DEBUG_RENDERER)
    pub const drawConstraintLimits = if (Core.debug_renderer) drawConstraintLimitsImpl else {};
    fn drawConstraintLimitsImpl(self: *const SliderConstraint, renderer: *DebugRenderer) void {
        if (self.has_limits) {
            const transform1 = self.base.body1.getCenterOfMassTransform();
            const transform2 = self.base.body2.getCenterOfMassTransform();

            // Transform the local positions into world space
            const slider_axis = transform1.multiply3x3(self.local_space_slider_axis1);
            const position1 = transform1.mulVec3(self.local_space_position1);
            const position2 = transform2.mulVec3(self.local_space_position2);

            // Calculate the limits in world space
            const limits_min = position1.addVec3(slider_axis.mulScalar(self.limits_min));
            const limits_max = position1.addVec3(slider_axis.mulScalar(self.limits_max));

            renderer.drawLine(limits_min, position1, Color.white);
            renderer.drawLine(position2, limits_max, Color.white);

            renderer.drawMarker(limits_min, Color.white, 0.1);
            renderer.drawMarker(limits_max, Color.white, 0.1);
        }
    }

    pub fn saveState(self: *const SliderConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        self.motor_constraint_part.saveState(stream);
        self.position_constraint_part.saveState(stream);
        self.rotation_constraint_part.saveState(stream);
        self.position_limits_constraint_part.saveState(stream);

        stream.write(self.motor_state);
        stream.write(self.target_velocity);
        stream.write(self.target_position);
    }

    pub fn restoreState(self: *SliderConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        self.motor_constraint_part.restoreState(stream);
        self.position_constraint_part.restoreState(stream);
        self.rotation_constraint_part.restoreState(stream);
        self.position_limits_constraint_part.restoreState(stream);

        stream.read(&self.motor_state);
        stream.read(&self.target_velocity);
        stream.read(&self.target_position);
    }

    pub fn getConstraintSettings(self: *const SliderConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try SliderConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(settings.asSettings());
        settings.space = .local_to_body_com;
        settings.point1 = RVec3.fromVec3(self.local_space_position1);
        settings.slider_axis1 = self.local_space_slider_axis1;
        settings.normal_axis1 = self.local_space_normal1;
        settings.point2 = RVec3.fromVec3(self.local_space_position2);
        const inv_initial_rotation = Mat44.rotationQuat(self.inv_initial_orientation);
        settings.slider_axis2 = inv_initial_rotation.multiply3x3(self.local_space_slider_axis1);
        settings.normal_axis2 = inv_initial_rotation.multiply3x3(self.local_space_normal1);
        settings.limits_min = self.limits_min;
        settings.limits_max = self.limits_max;
        settings.limits_spring_settings = self.limits_spring_settings;
        settings.max_friction_force = self.max_friction_force;
        settings.motor_settings = self.motor_settings;
        return .init(settings.asSettings());
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const SliderConstraint) Mat44 {
        return Mat44.init(Vec4.fromVec3W(self.local_space_slider_axis1, 0), Vec4.fromVec3W(self.local_space_normal1, 0), Vec4.fromVec3W(self.local_space_normal2, 0), Vec4.fromVec3W(self.local_space_position1, 1));
    }

    pub fn getConstraintToBody2Matrix(self: *const SliderConstraint) Mat44 {
        var mat = Mat44.rotationQuat(self.inv_initial_orientation).multiply3x3Mat44(Mat44.init(Vec4.fromVec3W(self.local_space_slider_axis1, 0), Vec4.fromVec3W(self.local_space_normal1, 0), Vec4.fromVec3W(self.local_space_normal2, 0), Vec4.init(0, 0, 0, 1)));
        mat.setTranslation(self.local_space_position2);
        return mat;
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig, simulations in
// ZoltTests/Physics/SliderConstraintTests.zig)

const testing = std.testing;
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
    return data.toOwnedSlice();
}

/// ConstraintSettings.restoreFromBinaryState of bytes
fn testRestoreSettings(allocator: Allocator, bytes: []const u8) Allocator.Error!ConstraintSettings.ConstraintResult {
    var reader: std.Io.Reader = .fixed(bytes);
    var in = StreamWrapper.StreamInWrapper.init(&reader);
    return ConstraintSettings.restoreFromBinaryState(allocator, in.streamIn());
}

/// The world space position of a point in the local space of the center of mass of a body
fn testWorldPoint(body: *const Body, local_point: Vec3) RVec3 {
    return body.getCenterOfMassTransform().mulRVec3(RVec3.fromVec3(local_point));
}

/// SetupVelocityConstraint without the impulses of earlier steps (each check of the solver starts from zero lambdas)
fn testSetup(base: *Constraint, delta_time: f32) void {
    base.resetWarmStart();
    base.setupVelocityConstraint(delta_time);
}

/// Moves a body by `delta` (world space)
fn testMoveBody(body: *Body, delta: Vec3) void {
    body.setPositionAndRotationInternal(body.getPosition().add(RVec3.fromVec3(delta)), body.getRotation(), .{});
}

test "SliderConstraintSettings: defaults, binary state, restoreFromBinaryState through the registry, copy, setSliderAxis" {
    const allocator = testing.allocator;

    // Jolt's defaults
    var settings = SliderConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    try testing.expectEqual(ConstraintSpace.world_space, settings.space);
    try testing.expect(!settings.auto_detect_point);
    try testing.expect(settings.point1.eql(RVec3.zero()) and settings.point2.eql(RVec3.zero()));
    try testing.expect(settings.slider_axis1.eql(Vec3.axisX()) and settings.normal_axis1.eql(Vec3.axisY()));
    try testing.expect(settings.slider_axis2.eql(Vec3.axisX()) and settings.normal_axis2.eql(Vec3.axisY()));
    try testing.expectEqual(-math.flt_max, settings.limits_min);
    try testing.expectEqual(math.flt_max, settings.limits_max);
    try testing.expectEqual(SpringSettings{}, settings.limits_spring_settings);
    try testing.expectEqual(@as(f32, 0.0), settings.max_friction_force);
    try testing.expectEqual(MotorSettings{}, settings.motor_settings);
    try testing.expectEqualStrings("SliderConstraintSettings", settings.asSettings().vtable.rtti.name);

    // SetSliderAxis: both slider axes and a perpendicular normal axis on both bodies
    const axis = Vec3.init(1, 2, -2).normalized();
    settings.setSliderAxis(axis);
    try testing.expect(settings.slider_axis1.eql(axis) and settings.slider_axis2.eql(axis));
    try testing.expect(settings.normal_axis1.eql(axis.getNormalizedPerpendicular()) and settings.normal_axis2.eql(settings.normal_axis1));
    try testing.expect(@abs(settings.normal_axis1.dot(axis)) < 1.0e-6);

    // Non default values in every field
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 9;
    settings.base.base.num_velocity_steps_override = 3;
    settings.base.base.num_position_steps_override = 4;
    settings.base.base.draw_constraint_size = 0.5;
    settings.space = .local_to_body_com;
    settings.auto_detect_point = true;
    settings.point1 = RVec3.init(1, 2, 3);
    settings.slider_axis1 = Vec3.axisY();
    settings.normal_axis1 = Vec3.axisZ();
    settings.point2 = RVec3.init(-4, 5, -6);
    settings.slider_axis2 = Vec3.axisZ();
    settings.normal_axis2 = Vec3.axisX();
    settings.limits_min = -1.5;
    settings.limits_max = 2.5;
    settings.limits_spring_settings = .init(.stiffness_and_damping, 100.0, 3.0);
    settings.max_friction_force = 7.0;
    settings.motor_settings = .initModeLimits(.mass_normalized_stiffness_and_damping, 5.0, 0.5, 10.0, 20.0);

    // The bytes: ConstraintSettings (hash, enabled, draw size, priority, steps), space (int), auto detect (bool), the
    // points (Real), the axes (3 floats each), limits min / max, max friction force, the spring (mode byte, 2 floats),
    // the motor (a spring, 4 floats)
    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    try testing.expectEqual(@as(usize, 21 + 4 + 1 + 2 * 3 * @sizeOf(Real) + 4 * 12 + 3 * 4 + 9 + 25), bytes.len);
    try testing.expectEqual(RTTI.hashName("SliderConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(i32, 0), std.mem.readInt(i32, bytes[21..25], .little)); // LocalToBodyCOM
    const limits_offset = 21 + 4 + 1 + 2 * 3 * @sizeOf(Real) + 4 * 12;
    try testing.expectEqual(@as(f32, -1.5), @as(f32, @bitCast(std.mem.readInt(u32, bytes[limits_offset..][0..4], .little))));
    try testing.expectEqual(@as(f32, 7.0), @as(f32, @bitCast(std.mem.readInt(u32, bytes[limits_offset + 8 ..][0..4], .little))));
    try testing.expectEqual(@as(u8, 1), bytes[limits_offset + 12]); // StiffnessAndDamping

    // Restore through RegisterTypes.factory
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(SliderConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(!restored.base.base.enabled);
        try testing.expectEqual(@as(u32, 9), restored.base.base.constraint_priority);
        try testing.expectEqual(ConstraintSpace.local_to_body_com, restored.space);
        try testing.expect(restored.auto_detect_point);
        try testing.expect(restored.point1.eql(settings.point1) and restored.point2.eql(settings.point2));
        try testing.expect(restored.slider_axis1.eql(settings.slider_axis1) and restored.normal_axis1.eql(settings.normal_axis1));
        try testing.expect(restored.slider_axis2.eql(settings.slider_axis2) and restored.normal_axis2.eql(settings.normal_axis2));
        try testing.expectEqual(settings.limits_min, restored.limits_min);
        try testing.expectEqual(settings.limits_max, restored.limits_max);
        try testing.expectEqual(settings.limits_spring_settings, restored.limits_spring_settings);
        try testing.expectEqual(settings.max_friction_force, restored.max_friction_force);
        try testing.expectEqual(settings.motor_settings, restored.motor_settings);
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, bytes, resaved);
    }

    // Truncated: Jolt's error
    {
        var result = try testRestoreSettings(allocator, bytes[0 .. bytes.len - 1]);
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
    var copy = SliderConstraintSettings.initCopy(allocator, &settings);
    defer copy.asSettings().deinit();
    try testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try testing.expect(copy.point2.eql(settings.point2) and copy.normal_axis2.eql(settings.normal_axis2));
    try testing.expectEqual(settings.motor_settings, copy.motor_settings);
    try testing.expectEqual(@as(u32, 9), copy.base.base.constraint_priority);
}

test "SliderConstraint: virtual dispatch, space conversions, solver, settings and state" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];

        // World space with given points and axes (the frame of body 2 is rotated a quarter turn around Z)
        var settings = SliderConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.point1 = RVec3.init(1.5, 1.75, 3.5);
        settings.point2 = RVec3.init(1.75, 1.5, 3.75);
        settings.slider_axis1 = Vec3.init(1, 1, 0).normalized();
        settings.normal_axis1 = Vec3.init(-1, 1, 0).normalized();
        settings.slider_axis2 = Vec3.init(-1, 1, 0).normalized();
        settings.normal_axis2 = Vec3.init(-1, -1, 0).normalized();
        settings.base.base.user_data = 77;

        // TwoBodyConstraintSettings::Create through the vtable
        var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer constraint.deinit();
        const base = constraint.get().?;
        const two_body = virtual.downcast(TwoBodyConstraint, base);
        const c = virtual.downcast(SliderConstraint, two_body);
        try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
        try testing.expectEqual(ConstraintSubType.slider, base.getSubType());
        try testing.expectEqual(@as(u64, 77), base.getUserData());
        try testing.expectEqual(pair[0] == .dynamic or pair[1] == .dynamic, base.isActive());
        try testing.expect(two_body.getBody1() == body1 and two_body.getBody2() == body2);

        // The constructor took the points and axes to the local space of the bodies (and the orientation difference)
        const inv1 = body1.getInverseCenterOfMassTransform();
        try testing.expect(c.local_space_position1.eql(inv1.mulRVec3(settings.point1).toVec3()));
        try testing.expect(c.local_space_position2.eql(body2.getInverseCenterOfMassTransform().mulRVec3(settings.point2).toVec3()));
        try testing.expect(c.local_space_slider_axis1.eql(inv1.multiply3x3(settings.slider_axis1).normalized()));
        try testing.expect(c.local_space_normal1.eql(inv1.multiply3x3(settings.normal_axis1).normalized()));
        try testing.expect(c.local_space_normal2.eql(c.local_space_slider_axis1.cross(c.local_space_normal1)));
        const expected_orientation = body2.getRotation().conjugated().mul(RotationEulerConstraintPart.getInvInitialOrientationXY(settings.slider_axis1, settings.normal_axis1, settings.slider_axis2, settings.normal_axis2)).mul(body1.getRotation());
        try testing.expect(c.inv_initial_orientation.eql(expected_orientation));
        try testing.expect(!c.hasLimits());
        try testing.expectEqual(MotorState.off, c.getMotorState());

        // The constraint matrices: the slider axis is the first column, the normal axes the others; at creation they
        // are the frames of the settings in world space
        const m1 = two_body.getConstraintToBody1Matrix();
        try testing.expect(m1.getAxisX().eql(c.local_space_slider_axis1) and m1.getAxisY().eql(c.local_space_normal1) and m1.getAxisZ().eql(c.local_space_normal2));
        try testing.expect(m1.getTranslation().eql(c.local_space_position1));
        const m2 = two_body.getConstraintToBody2Matrix();
        try testing.expect(m2.getTranslation().eql(c.local_space_position2));
        const world1 = Mat44.rotationQuat(body1.getRotation()).multiply3x3Mat44(m1);
        const world2 = Mat44.rotationQuat(body2.getRotation()).multiply3x3Mat44(m2);
        try testing.expect(world1.getAxisX().isClose(settings.slider_axis1, .{ .max_dist_sq = 1.0e-10 }) and world1.getAxisY().isClose(settings.normal_axis1, .{ .max_dist_sq = 1.0e-10 }));
        try testing.expect(world2.getAxisX().isClose(settings.slider_axis2, .{ .max_dist_sq = 1.0e-10 }) and world2.getAxisY().isClose(settings.normal_axis2, .{ .max_dist_sq = 1.0e-10 }));

        // GetCurrentPosition: the distance of the attachment points along the slider axis (world space)
        const p1 = testWorldPoint(body1, c.local_space_position1);
        const p2 = testWorldPoint(body2, c.local_space_position2);
        const world_axis = body1.getRotation().mulVec3(c.local_space_slider_axis1);
        try testing.expectApproxEqAbs(p2.sub(p1).toVec3().dot(world_axis), c.getCurrentPosition(), 1.0e-5);

        // GetConstraintSettings: local space; a constraint made from them is the same
        {
            var copy = try base.getConstraintSettings();
            defer copy.deinit();
            const s = virtual.downcast(SliderConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, copy.get().?));
            try testing.expectEqual(ConstraintSpace.local_to_body_com, s.space);
            try testing.expectEqual(@as(u64, 77), s.base.base.user_data);
            try testing.expect(s.slider_axis1.eql(c.local_space_slider_axis1) and s.normal_axis1.eql(c.local_space_normal1));
            var again = SliderConstraint.init(allocator, body1, body2, s);
            defer again.asConstraint().deinit();
            try testing.expect(again.local_space_position1.eql(c.local_space_position1) and again.local_space_position2.eql(c.local_space_position2));
            try testing.expect(again.local_space_slider_axis1.eql(c.local_space_slider_axis1) and again.local_space_normal2.eql(c.local_space_normal2));
            const q = again.inv_initial_orientation;
            try testing.expect(q.isClose(c.inv_initial_orientation, .{ .max_dist_sq = 1.0e-10 }) or q.isClose(c.inv_initial_orientation.negate(), .{ .max_dist_sq = 1.0e-10 }));
        }

        // Solver: the attachment points move along the slider axis only and both bodies rotate together; a static or
        // kinematic body does not move
        const before1 = TestContext.BodySnapshot.take(body1);
        const before2 = TestContext.BodySnapshot.take(body2);
        base.setupVelocityConstraint(1.0 / 60.0);
        base.warmStartVelocityConstraint(1.0);
        const applied = base.solveVelocityConstraint(1.0 / 60.0);
        // (the rotation part also reports an impulse between two bodies that are not dynamic: Jolt sets the zero inverse
        // inertia to identity, the impulse is computed but no body is written)
        const any_dynamic = pair[0] == .dynamic or pair[1] == .dynamic;
        if (any_dynamic) try testing.expect(applied);
        try TestContext.checkUnmoved(body1, before1);
        try TestContext.checkUnmoved(body2, before2);
        if (any_dynamic) {
            for (0..30) |_| _ = base.solveVelocityConstraint(1.0 / 60.0);
            try testing.expect(body1.getAngularVelocity().isClose(body2.getAngularVelocity(), .{ .max_dist_sq = 1.0e-6 }));
            const r1 = body1.getRotation().mulVec3(c.local_space_position1).add(c.u);
            const r2 = body2.getRotation().mulVec3(c.local_space_position2);
            const v1 = body1.getLinearVelocity().add(body1.getAngularVelocity().cross(r1));
            const v2 = body2.getLinearVelocity().add(body2.getAngularVelocity().cross(r2));
            const dv = v2.sub(v1);
            try testing.expect(@abs(dv.dot(c.n1)) < 1.0e-5 and @abs(dv.dot(c.n2)) < 1.0e-5);
            try testing.expect(!c.getTotalLambdaPosition().isZero() and !c.getTotalLambdaRotation().eql(Vec3.zero()));
            try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaPositionLimits());
            try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaMotor());
            try testing.expect(base.solvePositionConstraint(1.0 / 60.0, 0.2));
        }

        // SaveState / RestoreState: enabled, the lambdas of the parts, the motor state and the targets
        c.setMotorState(.velocity);
        c.setTargetVelocity(1.25);
        c.setTargetPosition(-0.75);
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        base.saveState(recorder.recorder());
        try testing.expectEqual(@as(usize, 1 + 4 + 8 + 12 + 4 + 4 + 4 + 4), recorder.getData().len);
        const lambda_position = c.getTotalLambdaPosition();
        const lambda_rotation = c.getTotalLambdaRotation();
        base.resetWarmStart();
        try testing.expect(c.getTotalLambdaPosition().isZero() and c.getTotalLambdaRotation().eql(Vec3.zero()));
        c.setMotorState(.off);
        c.setTargetVelocity(0.0);
        c.setTargetPosition(0.0);
        base.restoreState(recorder.recorder());
        try testing.expect(c.getTotalLambdaPosition().eql(lambda_position) and c.getTotalLambdaRotation().eql(lambda_rotation));
        try testing.expectEqual(MotorState.velocity, c.getMotorState());
        try testing.expectEqual(@as(f32, 1.25), c.getTargetVelocity());
        try testing.expectEqual(@as(f32, -0.75), c.getTargetPosition());

        // NotifyShapeChanged moves the attachment point of the body that changed
        const lp1 = c.local_space_position1;
        const lp2 = c.local_space_position2;
        base.notifyShapeChanged(body2.getID(), Vec3.init(0.5, 0, 0));
        try testing.expect(c.local_space_position1.eql(lp1) and c.local_space_position2.eql(lp2.sub(Vec3.init(0.5, 0, 0))));
        base.notifyShapeChanged(body1.getID(), Vec3.init(0, 1, 0));
        try testing.expect(c.local_space_position1.eql(lp1.sub(Vec3.init(0, 1, 0))));
        base.notifyShapeChanged(BodyID.init(12345), Vec3.init(1, 1, 1));
        try testing.expect(c.local_space_position1.eql(lp1.sub(Vec3.init(0, 1, 0))) and c.local_space_position2.eql(lp2.sub(Vec3.init(0.5, 0, 0))));

        // A disabled constraint is not active (TwoBodyConstraint::IsActive)
        base.setEnabled(false);
        try testing.expect(!base.isActive());
    }
}

test "SliderConstraint: setters and getters" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .static, .dynamic });
    defer context.destroyBodies(&bodies);

    var settings = SliderConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.max_friction_force = 3.0;
    settings.limits_min = -2.0;
    settings.limits_max = 1.0;
    settings.limits_spring_settings = .init(.frequency_and_damping, 4.0, 0.5);
    settings.motor_settings = .initLimits(10.0, 1.0, 50.0, 0.0);
    var c = SliderConstraint.init(allocator, bodies[0], bodies[1], &settings);
    defer c.asConstraint().deinit();

    // The constructor takes the settings
    try testing.expectEqual(@as(f32, 3.0), c.getMaxFrictionForce());
    try testing.expectEqual(@as(f32, -2.0), c.getLimitsMin());
    try testing.expectEqual(@as(f32, 1.0), c.getLimitsMax());
    try testing.expect(c.hasLimits());
    try testing.expectEqual(settings.limits_spring_settings, c.getLimitsSpringSettings().*);
    try testing.expectEqual(settings.motor_settings, c.getMotorSettings().*);
    try testing.expectEqual(MotorState.off, c.getMotorState());
    try testing.expectEqual(@as(f32, 0.0), c.getTargetVelocity());
    try testing.expectEqual(@as(f32, 0.0), c.getTargetPosition());

    // Friction
    c.setMaxFrictionForce(5.5);
    try testing.expectEqual(@as(f32, 5.5), c.getMaxFrictionForce());

    // Motor settings through the mutable reference
    c.getMotorSettingsMut().setForceLimits(-1.0, 2.0);
    try testing.expectEqual(@as(f32, -1.0), c.getMotorSettings().min_force_limit);
    try testing.expectEqual(@as(f32, 2.0), c.getMotorSettings().max_force_limit);

    // Motor controls
    for ([_]MotorState{ .velocity, .position, .position_and_velocity, .off }) |state| {
        c.setMotorState(state);
        try testing.expectEqual(state, c.getMotorState());
    }
    c.setTargetVelocity(-4.5);
    try testing.expectEqual(@as(f32, -4.5), c.getTargetVelocity());

    // The target position is clamped to the limits when there are limits
    c.setTargetPosition(0.5);
    try testing.expectEqual(@as(f32, 0.5), c.getTargetPosition());
    c.setTargetPosition(3.0);
    try testing.expectEqual(@as(f32, 1.0), c.getTargetPosition());
    c.setTargetPosition(-3.0);
    try testing.expectEqual(@as(f32, -2.0), c.getTargetPosition());

    // SetLimits: a limit on one side only counts, the defaults are no limits
    c.setLimits(-math.flt_max, 0.5);
    try testing.expect(c.hasLimits());
    c.setLimits(0.0, math.flt_max);
    try testing.expect(c.hasLimits());
    c.setLimits(-math.flt_max, math.flt_max);
    try testing.expect(!c.hasLimits());
    try testing.expectEqual(-math.flt_max, c.getLimitsMin());
    try testing.expectEqual(math.flt_max, c.getLimitsMax());
    c.setTargetPosition(100.0);
    try testing.expectEqual(@as(f32, 100.0), c.getTargetPosition());

    // Limits spring settings
    c.setLimitsSpringSettings(.init(.stiffness_and_damping, 200.0, 2.0));
    try testing.expectEqual(SpringSettings.init(.stiffness_and_damping, 200.0, 2.0), c.getLimitsSpringSettings().*);
    c.getLimitsSpringSettingsMut().damping = 3.0;
    try testing.expectEqual(@as(f32, 3.0), c.getLimitsSpringSettings().damping);
}

test "SliderConstraint: local space, auto detect anchor and getCurrentPosition" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();

    // Local space: the points and axes are used as they are
    {
        const bodies = try context.createBodyPair(.{ .kinematic, .dynamic });
        defer context.destroyBodies(&bodies);
        var settings = SliderConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.space = .local_to_body_com;
        settings.point1 = RVec3.init(0.25, 0.5, 0.75);
        settings.point2 = RVec3.init(-1, -2, -3);
        settings.slider_axis1 = Vec3.axisZ();
        settings.normal_axis1 = Vec3.axisX();
        settings.slider_axis2 = Vec3.axisZ();
        settings.normal_axis2 = Vec3.axisX();
        var c = SliderConstraint.init(allocator, bodies[0], bodies[1], &settings);
        defer c.asConstraint().deinit();
        try testing.expect(c.local_space_position1.eql(Vec3.init(0.25, 0.5, 0.75)) and c.local_space_position2.eql(Vec3.init(-1, -2, -3)));
        try testing.expect(c.local_space_slider_axis1.eql(Vec3.axisZ()) and c.local_space_normal1.eql(Vec3.axisX()));
        try testing.expect(c.local_space_normal2.eql(Vec3.axisY()));
        try testing.expect(c.inv_initial_orientation.eql(Quat.identity())); // Same axes: identity
    }

    // Auto detect: the anchor is the center of mass of the body that can move when the other one is static, the
    // weighted center of mass otherwise (towards the lighter body); both points are the anchor in world space, so the
    // current position is zero
    const axis = Vec3.init(0, 1, 1).normalized();
    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        var settings = SliderConstraintSettings.init(allocator);
        defer settings.asSettings().deinit();
        settings.auto_detect_point = true;
        settings.point1 = RVec3.init(100, 100, 100); // Ignored
        settings.setSliderAxis(axis);
        var c = SliderConstraint.init(allocator, bodies[0], bodies[1], &settings);
        defer c.asConstraint().deinit();
        const anchor1 = testWorldPoint(bodies[0], c.local_space_position1);
        const anchor2 = testWorldPoint(bodies[1], c.local_space_position2);
        try testing.expect(anchor1.sub(anchor2).toVec3().length() < 1.0e-5);
        try testing.expect(@abs(c.getCurrentPosition()) < 1.0e-5);
        const p1 = bodies[0].getCenterOfMassPosition();
        const p2 = bodies[1].getCenterOfMassPosition();
        if (pair[0] == .static) {
            try testing.expect(anchor1.sub(p2).toVec3().length() < 1.0e-5);
        } else if (pair[1] == .static) {
            try testing.expect(anchor1.sub(p1).toVec3().length() < 1.0e-5);
        } else {
            // On the line between the centers of mass
            const t = anchor1.sub(p1).toVec3().length() / p2.sub(p1).toVec3().length();
            try testing.expect(t > 0.0 and t < 1.0);
        }

        // Moving body 2 along the slider axis changes the current position by that distance
        testMoveBody(bodies[1], axis.mulScalar(0.75));
        try testing.expectApproxEqAbs(@as(f32, 0.75), c.getCurrentPosition(), 1.0e-5);
    }
}

test "SliderConstraint: limits, motors and friction in the solver" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();

    // A dynamic box on a slider along X from a static box, moving along the axis at 2 m/s
    const body1 = try context.createBody(.static, .{});
    const body2 = try context.createBody(.dynamic, .{ .position = RVec3.init(1, 0, 0), .linear_velocity = Vec3.init(2, 0, 0) });
    defer context.destroyBodies(&.{ body1, body2 });
    var settings = SliderConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.auto_detect_point = true;
    settings.setSliderAxis(Vec3.axisX());
    var c = SliderConstraint.init(allocator, body1, body2, &settings);
    defer c.asConstraint().deinit();
    const base = c.asConstraint();
    const dt: f32 = 1.0 / 60.0;
    const mass = 1.0 / body2.getMotionProperties().getInverseMass();

    // No limits, no motor, no friction: only the position and rotation parts are active, the body keeps sliding
    testSetup(base, dt);
    try testing.expect(!c.motor_constraint_part.isActive() and !c.position_limits_constraint_part.isActive());
    _ = base.solveVelocityConstraint(dt);
    try testing.expectApproxEqAbs(@as(f32, 2.0), body2.getLinearVelocity().getX(), 1.0e-5);

    // Friction: the motor part (motor off) limits the impulse to max friction force * dt
    c.setMaxFrictionForce(mass * 6.0);
    testSetup(base, dt);
    try testing.expect(c.motor_constraint_part.isActive());
    try testing.expect(base.solveVelocityConstraint(dt));
    try testing.expectApproxEqAbs(-mass * 6.0 * dt, c.getTotalLambdaMotor(), 1.0e-3);
    try testing.expectApproxEqAbs(@as(f32, 2.0 - 6.0 * dt), body2.getLinearVelocity().getX(), 1.0e-4);
    c.setMaxFrictionForce(0.0);

    // Velocity motor: drives to the target velocity within the force limits
    c.getMotorSettingsMut().setForceLimit(1.0e8);
    c.setMotorState(.velocity);
    c.setTargetVelocity(-1.0);
    testSetup(base, dt);
    try testing.expect(c.motor_constraint_part.isActive());
    for (0..4) |_| _ = base.solveVelocityConstraint(dt);
    try testing.expectApproxEqAbs(@as(f32, -1.0), body2.getLinearVelocity().getX(), 1.0e-4);

    // Position motors without stiffness (and damping): deactivated; with stiffness: pull towards the target
    c.getMotorSettingsMut().spring_settings = .init(.frequency_and_damping, 0.0, 0.0);
    c.setMotorState(.position);
    testSetup(base, dt);
    try testing.expect(!c.motor_constraint_part.isActive());
    c.setMotorState(.position_and_velocity);
    testSetup(base, dt);
    try testing.expect(!c.motor_constraint_part.isActive());
    c.getMotorSettingsMut().spring_settings = .init(.frequency_and_damping, 2.0, 1.0);
    c.setMotorState(.position);
    c.setTargetPosition(5.0);
    body2.setLinearVelocity(Vec3.zero());
    testSetup(base, dt);
    try testing.expect(c.motor_constraint_part.isActive());
    _ = base.solveVelocityConstraint(dt);
    try testing.expect(body2.getLinearVelocity().getX() > 0.0); // Towards the target
    c.setMotorState(.position_and_velocity);
    testSetup(base, dt);
    try testing.expect(c.motor_constraint_part.isActive());
    c.setMotorState(.off);

    // Hard limits: the body is at position 0 and moves towards the max limit at 0: the limit stops it, the position
    // solver pushes it back towards the limit
    body2.setLinearVelocity(Vec3.init(3, 0, 0));
    c.setLimits(-1.0, 0.0);
    try testing.expect(c.hasLimits());
    testSetup(base, dt);
    try testing.expect(c.position_limits_constraint_part.isActive());
    _ = base.solveVelocityConstraint(dt);
    try testing.expect(c.getTotalLambdaPositionLimits() < 0.0); // Max limit: the impulse pushes back
    try testing.expectApproxEqAbs(@as(f32, 0.0), body2.getLinearVelocity().getX(), 1.0e-4);
    testMoveBody(body2, Vec3.init(0.1, 0, 0));
    try testing.expect(base.solvePositionConstraint(dt, 0.5));
    try testing.expectApproxEqAbs(@as(f32, 0.05), c.getCurrentPosition(), 1.0e-4);

    // Min limit: moving away from the min limit is free
    c.setLimits(0.0, 1.0);
    body2.setLinearVelocity(Vec3.init(1, 0, 0));
    testMoveBody(body2, Vec3.init(-0.1, 0, 0));
    testSetup(base, dt);
    try testing.expect(c.position_limits_constraint_part.isActive()); // At the min limit
    _ = base.solveVelocityConstraint(dt);
    try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaPositionLimits());
    try testing.expectApproxEqAbs(@as(f32, 1.0), body2.getLinearVelocity().getX(), 1.0e-5);

    // Soft limits: the position solver leaves them to the spring
    c.setLimitsSpringSettings(.init(.frequency_and_damping, 5.0, 0.5));
    testMoveBody(body2, Vec3.init(-0.2, 0, 0));
    testSetup(base, dt);
    try testing.expect(c.position_limits_constraint_part.isActive());
    const before = c.getCurrentPosition();
    _ = base.solvePositionConstraint(dt, 0.5);
    try testing.expectApproxEqAbs(before, c.getCurrentPosition(), 1.0e-5);

    // Equal limits with a soft spring (allowed by the constructor assert): the impulse is not clamped to one side
    c.setLimits(0.0, 0.0);
    testSetup(base, dt);
    try testing.expect(c.position_limits_constraint_part.isActive());
    body2.setLinearVelocity(Vec3.init(-1, 0, 0));
    _ = base.solveVelocityConstraint(dt);
    try testing.expect(c.getTotalLambdaPositionLimits() > 0.0);
}

test "SliderConstraint: body 1 fixed to the world" {
    // Body 1 is Body::sFixedToWorld: the constraint attaches body 2 to the world (the solver never writes body 1)
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const body = try context.createBody(.dynamic, .{ .position = RVec3.init(1, 2, 3), .linear_velocity = Vec3.init(1, 2, 0), .angular_velocity = Vec3.init(0, 1, 0) });
    defer context.destroyBodies(&.{body});
    var settings = SliderConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    settings.auto_detect_point = true;
    settings.setSliderAxis(Vec3.axisX());
    var c = SliderConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
    defer c.asConstraint().deinit();
    try testing.expect(c.local_space_position2.isNearZero(.{})); // The anchor is the center of mass of body 2
    try testing.expect(c.local_space_position1.isClose(body.getCenterOfMassPosition().toVec3(), .{}));
    const base = c.asConstraint();
    base.setupVelocityConstraint(1.0 / 60.0);
    base.warmStartVelocityConstraint(1.0);
    try testing.expect(base.solveVelocityConstraint(1.0 / 60.0));
    for (0..5) |_| _ = base.solveVelocityConstraint(1.0 / 60.0);

    // Only the velocity along the slider axis remains
    try testing.expect(body.getLinearVelocity().isClose(Vec3.init(1, 0, 0), .{ .max_dist_sq = 1.0e-8 }));
    try testing.expect(body.getAngularVelocity().isNearZero(.{ .max_dist_sq = 1.0e-8 }));
}

test "SliderConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const bodies = try context.createBodyPair(.{ .dynamic, .static });
            defer context.destroyBodies(&bodies);
            var settings = Ref(ConstraintSettings).init((try SliderConstraintSettings.create(allocator)).asSettings());
            defer settings.deinit();
            const two_body_settings = virtual.downcast(TwoBodyConstraintSettings, settings.get().?);
            var constraint = Ref(Constraint).init(&(try two_body_settings.createConstraint(allocator, bodies[0], bodies[1])).base);
            defer constraint.deinit();
            var copy = try constraint.get().?.getConstraintSettings();
            defer copy.deinit();
        }
    };
    try testing.checkAllAllocationFailures(testing.allocator, S.run, .{});
}
