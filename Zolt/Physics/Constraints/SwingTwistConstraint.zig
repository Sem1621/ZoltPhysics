//! Port of: Jolt/Physics/Constraints/SwingTwistConstraint.h, Jolt/Physics/Constraints/SwingTwistConstraint.cpp
//! Status: complete
//!
//! Follows the porter template in FixedConstraint.zig. JPH_DECLARE_SERIALIZABLE_VIRTUAL /
//! JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(SwingTwistConstraintSettings): `rtti` with Jolt's attributes.
//! JPH_DEBUG_RENDERER (DrawConstraint, DrawConstraintLimits): TODO(debug_renderer).
//!
//! - The non-const overloads `MotorSettings &GetSwingMotorSettings()` / `GetTwistMotorSettings()` are
//!   `getSwingMotorSettingsMut` / `getTwistMotorSettingsMut` (guide section 4).
//! - `impulse |= part.SolveVelocityConstraint(...)` always runs the solve (no short circuit).
//! - Asserts that user input can violate only run with `Core.enable_asserts`, ReleaseFast keeps Jolt's release
//!   behavior: a motor state other than Off with invalid motor settings (SetSwingMotorState / SetTwistMotorState), and
//!   the swing motor parts turning on / off together in SolveVelocityConstraint (with restricted allowed DOFs the
//!   inverse inertia along one swing motor axis can be zero while it is not along the other one, the part on that axis
//!   then deactivates itself).
//! - The switches on `EMotorState` have a `_` prong that does nothing, like Jolt's switches when no case matches (a
//!   restored state can hold any int, see MotorSettings.zig).

const std = @import("std");
const Allocator = std.mem.Allocator;
const RTTI = @import("../../Core/RTTI.zig").RTTI;
const Core = @import("../../Core/Core.zig");
const Ref = @import("../../Core/Reference.zig").Ref;
const StreamIn = @import("../../Core/StreamIn.zig").StreamIn;
const StreamOut = @import("../../Core/StreamOut.zig").StreamOut;
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
const PointConstraintPart = @import("ConstraintPart/PointConstraintPart.zig").PointConstraintPart;
const AngleConstraintPart = @import("ConstraintPart/AngleConstraintPart.zig").AngleConstraintPart;
const SwingTwistConstraintPartFile = @import("ConstraintPart/SwingTwistConstraintPart.zig");
const SwingTwistConstraintPart = SwingTwistConstraintPartFile.SwingTwistConstraintPart;
const SwingType = SwingTwistConstraintPartFile.SwingType;

/// Swing twist constraint settings, used to create a swing twist constraint
/// All values in this structure are copied to the swing twist constraint and the settings object is no longer needed afterwards.
///
/// This image describes the limit settings:
/// @image html Docs/SwingTwistConstraint.png
pub const SwingTwistConstraintSettings = struct {
    /// JPH_DECLARE_SERIALIZABLE_VIRTUAL / JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL(SwingTwistConstraintSettings)
    pub const rtti = RTTI.declare(@This(), .{
        .name = "SwingTwistConstraintSettings",
        .kind = .virtual,
        .bases = &.{TwoBodyConstraintSettings},
        .attributes = &.{
            .enumAttr("mSpace", .space),
            .attr("mPosition1", .position1),
            .attr("mTwistAxis1", .twist_axis1),
            .attr("mPlaneAxis1", .plane_axis1),
            .attr("mPosition2", .position2),
            .attr("mTwistAxis2", .twist_axis2),
            .attr("mPlaneAxis2", .plane_axis2),
            .enumAttr("mSwingType", .swing_type),
            .attr("mNormalHalfConeAngle", .normal_half_cone_angle),
            .attr("mPlaneHalfConeAngle", .plane_half_cone_angle),
            .attr("mTwistMinAngle", .twist_min_angle),
            .attr("mTwistMaxAngle", .twist_max_angle),
            .attr("mMaxFrictionTorque", .max_friction_torque),
            .attr("mSwingMotorSettings", .swing_motor_settings),
            .attr("mTwistMotorSettings", .twist_motor_settings),
        },
    });

    pub const overrides = .{ .saveBinaryState, .createConstraint, .restoreBinaryState };

    base: TwoBodyConstraintSettings,

    /// This determines in which space the constraint is setup, all properties below should be in the specified space
    space: ConstraintSpace = .world_space,

    ///@name Body 1 constraint reference frame (space determined by mSpace)
    position1: RVec3 = RVec3.zero(),
    twist_axis1: Vec3 = Vec3.axisX(),
    plane_axis1: Vec3 = Vec3.axisY(),

    ///@name Body 2 constraint reference frame (space determined by mSpace)
    position2: RVec3 = RVec3.zero(),
    twist_axis2: Vec3 = Vec3.axisX(),
    plane_axis2: Vec3 = Vec3.axisY(),

    /// The type of swing constraint that we want to use.
    swing_type: SwingType = .cone,

    ///@name Swing rotation limits
    /// See image at Detailed Description. Angle in radians.
    normal_half_cone_angle: f32 = 0.0,
    /// See image at Detailed Description. Angle in radians.
    plane_half_cone_angle: f32 = 0.0,

    ///@name Twist rotation limits
    /// See image at Detailed Description. Angle in radians. Should be \f$\in [-\pi, \pi]\f$.
    twist_min_angle: f32 = 0.0,
    /// See image at Detailed Description. Angle in radians. Should be \f$\in [-\pi, \pi]\f$.
    twist_max_angle: f32 = 0.0,

    ///@name Friction
    /// Maximum amount of torque (N m) to apply as friction when the constraint is not powered by a motor
    max_friction_torque: f32 = 0.0,

    ///@name In case the constraint is powered, this determines the motor settings around the swing and twist axis
    swing_motor_settings: MotorSettings = .{},
    twist_motor_settings: MotorSettings = .{},

    /// Default constructor of settings that are not on the heap (the owner calls `asSettings().deinit()`)
    pub fn init(allocator: Allocator) SwingTwistConstraintSettings {
        return .{ .base = .init(SwingTwistConstraintSettings, allocator) };
    }

    /// new SwingTwistConstraintSettings (reference count 0)
    pub fn create(allocator: Allocator) Allocator.Error!*SwingTwistConstraintSettings {
        const self = try allocator.create(SwingTwistConstraintSettings);
        self.* = .init(allocator);
        return self;
    }

    /// The default constructor on the heap, for ConstraintSettings.restoreFromBinaryState (RegisterTypes)
    pub const createDefault = create;

    /// The implicit copy constructor: copies the settings of `other`, not its reference count
    pub fn initCopy(allocator: Allocator, other: *const SwingTwistConstraintSettings) SwingTwistConstraintSettings {
        var result = other.*;
        result.base.base = .initCopy(other.base.base.vtable, allocator, &other.base.base);
        return result;
    }

    pub fn asSettings(self: *SwingTwistConstraintSettings) *ConstraintSettings {
        return &self.base.base;
    }

    pub fn asSettingsConst(self: *const SwingTwistConstraintSettings) *const ConstraintSettings {
        return &self.base.base;
    }

    pub fn asTwoBodySettings(self: *SwingTwistConstraintSettings) *TwoBodyConstraintSettings {
        return &self.base;
    }

    pub fn asTwoBodySettingsConst(self: *const SwingTwistConstraintSettings) *const TwoBodyConstraintSettings {
        return &self.base;
    }

    // See: ConstraintSettings::SaveBinaryState
    pub fn saveBinaryState(self: *const SwingTwistConstraintSettings, stream: StreamOut) void {
        ConstraintSettings.impl.saveBinaryState(self.asSettingsConst(), stream);

        stream.write(self.space);
        stream.write(self.position1);
        stream.write(self.twist_axis1);
        stream.write(self.plane_axis1);
        stream.write(self.position2);
        stream.write(self.twist_axis2);
        stream.write(self.plane_axis2);
        stream.write(self.swing_type);
        stream.write(self.normal_half_cone_angle);
        stream.write(self.plane_half_cone_angle);
        stream.write(self.twist_min_angle);
        stream.write(self.twist_max_angle);
        stream.write(self.max_friction_torque);
        self.swing_motor_settings.saveBinaryState(stream);
        self.twist_motor_settings.saveBinaryState(stream);
    }

    /// Create an instance of this constraint
    pub fn createConstraint(self: *const SwingTwistConstraintSettings, allocator: Allocator, body1: *Body, body2: *Body) Allocator.Error!*TwoBodyConstraint {
        const constraint = try SwingTwistConstraint.create(allocator, body1, body2, self);
        return constraint.asTwoBodyConstraint();
    }

    // See: ConstraintSettings::RestoreBinaryState
    pub fn restoreBinaryState(self: *SwingTwistConstraintSettings, stream: StreamIn) Allocator.Error!void {
        try ConstraintSettings.impl.restoreBinaryState(self.asSettings(), stream);

        stream.read(&self.space);
        stream.read(&self.position1);
        stream.read(&self.twist_axis1);
        stream.read(&self.plane_axis1);
        stream.read(&self.position2);
        stream.read(&self.twist_axis2);
        stream.read(&self.plane_axis2);
        stream.read(&self.swing_type);
        stream.read(&self.normal_half_cone_angle);
        stream.read(&self.plane_half_cone_angle);
        stream.read(&self.twist_min_angle);
        stream.read(&self.twist_max_angle);
        stream.read(&self.max_friction_torque);
        self.swing_motor_settings.restoreBinaryState(stream);
        self.twist_motor_settings.restoreBinaryState(stream);
    }
};

/// A swing twist constraint is a specialized constraint for humanoid ragdolls that allows limited rotation only
///
/// @see SwingTwistConstraintSettings for a description of the limits
pub const SwingTwistConstraint = struct {
    pub const overrides = .{ .getSubType, .notifyShapeChanged, .setupVelocityConstraint, .resetWarmStart, .warmStartVelocityConstraint, .solveVelocityConstraint, .solvePositionConstraint, .saveState, .restoreState, .getConstraintSettings, .getConstraintToBody1Matrix, .getConstraintToBody2Matrix };

    base: TwoBodyConstraint,

    // CONFIGURATION PROPERTIES FOLLOW

    // Local space constraint positions
    local_space_position1: Vec3 = undefined,
    local_space_position2: Vec3 = undefined,

    // Transforms from constraint space to body space
    constraint_to_body1: Quat = undefined,
    constraint_to_body2: Quat = undefined,

    // Limits
    normal_half_cone_angle: f32,
    plane_half_cone_angle: f32,
    twist_min_angle: f32,
    twist_max_angle: f32,

    // Friction
    max_friction_torque: f32,

    // Motor controls
    swing_motor_settings: MotorSettings,
    twist_motor_settings: MotorSettings,
    swing_motor_state: MotorState = .off,
    twist_motor_state: MotorState = .off,
    target_angular_velocity: Vec3 = Vec3.zero(),
    target_orientation: Quat = Quat.identity(),

    // RUN TIME PROPERTIES FOLLOW

    // Rotation axis for motor constraint parts
    world_space_motor_axis: [3]Vec3 = undefined,

    // The constraint parts
    point_constraint_part: PointConstraintPart = .{},
    swing_twist_constraint_part: SwingTwistConstraintPart = .{},
    motor_constraint_part: [3]AngleConstraintPart = @splat(.{}),

    // Update the limits in the swing twist constraint part
    fn updateLimits(self: *SwingTwistConstraint) void {
        // Pass limits on to swing twist constraint part
        self.swing_twist_constraint_part.setLimits(self.twist_min_angle, self.twist_max_angle, -self.plane_half_cone_angle, self.plane_half_cone_angle, -self.normal_half_cone_angle, self.normal_half_cone_angle);
    }

    /// Construct swing twist constraint
    pub fn init(allocator: Allocator, body1: *Body, body2: *Body, settings: *const SwingTwistConstraintSettings) SwingTwistConstraint {
        var self: SwingTwistConstraint = .{
            .base = .init(SwingTwistConstraint, allocator, body1, body2, &settings.base),
            .normal_half_cone_angle = settings.normal_half_cone_angle,
            .plane_half_cone_angle = settings.plane_half_cone_angle,
            .twist_min_angle = settings.twist_min_angle,
            .twist_max_angle = settings.twist_max_angle,
            .max_friction_torque = settings.max_friction_torque,
            .swing_motor_settings = settings.swing_motor_settings,
            .twist_motor_settings = settings.twist_motor_settings,
        };

        // Override swing type
        self.swing_twist_constraint_part.setSwingType(settings.swing_type);

        // Calculate rotation needed to go from constraint space to body1 local space
        const normal_axis1 = settings.plane_axis1.cross(settings.twist_axis1);
        const c_to_b1 = Mat44.init(Vec4.fromVec3W(settings.twist_axis1, 0), Vec4.fromVec3W(normal_axis1, 0), Vec4.fromVec3W(settings.plane_axis1, 0), Vec4.init(0, 0, 0, 1));
        self.constraint_to_body1 = c_to_b1.getQuaternion();

        // Calculate rotation needed to go from constraint space to body2 local space
        const normal_axis2 = settings.plane_axis2.cross(settings.twist_axis2);
        const c_to_b2 = Mat44.init(Vec4.fromVec3W(settings.twist_axis2, 0), Vec4.fromVec3W(normal_axis2, 0), Vec4.fromVec3W(settings.plane_axis2, 0), Vec4.init(0, 0, 0, 1));
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

        self.updateLimits();

        return self;
    }

    /// new SwingTwistConstraint(inBody1, inBody2, inSettings) (reference count 0)
    pub fn create(allocator: Allocator, body1: *Body, body2: *Body, settings: *const SwingTwistConstraintSettings) Allocator.Error!*SwingTwistConstraint {
        const self = try allocator.create(SwingTwistConstraint);
        self.* = .init(allocator, body1, body2, settings);
        return self;
    }

    pub fn asConstraint(self: *SwingTwistConstraint) *Constraint {
        return &self.base.base;
    }

    pub fn asConstraintConst(self: *const SwingTwistConstraint) *const Constraint {
        return &self.base.base;
    }

    pub fn asTwoBodyConstraint(self: *SwingTwistConstraint) *TwoBodyConstraint {
        return &self.base;
    }

    pub fn asTwoBodyConstraintConst(self: *const SwingTwistConstraint) *const TwoBodyConstraint {
        return &self.base;
    }

    ///@name Generic interface of a constraint
    pub fn getSubType(self: *const SwingTwistConstraint) ConstraintSubType {
        _ = self;
        return .swing_twist;
    }

    pub fn notifyShapeChanged(self: *SwingTwistConstraint, body_id: BodyID, delta_com: Vec3) void {
        if (self.base.body1.getID().eql(body_id)) {
            self.local_space_position1 = self.local_space_position1.sub(delta_com);
        } else if (self.base.body2.getID().eql(body_id)) {
            self.local_space_position2 = self.local_space_position2.sub(delta_com);
        }
    }

    /// Get current rotation of constraint in constraint space.
    /// Solve: R2 * ConstraintToBody2 = R1 * ConstraintToBody1 * q for q.
    pub fn getRotationInConstraintSpace(self: *const SwingTwistConstraint) Quat {
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
        const constraint_body1_to_world = self.base.body1.getRotation().mul(self.constraint_to_body1);
        const constraint_body2_to_world = self.base.body2.getRotation().mul(self.constraint_to_body2);
        return constraint_body1_to_world.conjugated().mul(constraint_body2_to_world);
    }

    /// Controls if the motors are on or off
    pub fn setSwingMotorState(self: *SwingTwistConstraint, state: MotorState) void {
        // User input: Jolt's release build accepts any state
        if (Core.enable_asserts) std.debug.assert(state == .off or self.swing_motor_settings.isValid());

        if (self.swing_motor_state != state) {
            self.swing_motor_state = state;

            // Ensure that warm starting next frame doesn't apply any impulses (motor parts are repurposed for different modes)
            for (&self.motor_constraint_part) |*c|
                c.deactivate();
        }
    }

    pub fn setTwistMotorState(self: *SwingTwistConstraint, state: MotorState) void {
        // User input: Jolt's release build accepts any state
        if (Core.enable_asserts) std.debug.assert(state == .off or self.twist_motor_settings.isValid());

        if (self.twist_motor_state != state) {
            self.twist_motor_state = state;

            // Ensure that warm starting next frame doesn't apply any impulses (motor parts are repurposed for different modes)
            self.motor_constraint_part[0].deactivate();
        }
    }

    /// Set the target orientation in constraint space (drives constraint to: GetRotationInConstraintSpace() == inOrientation)
    pub fn setTargetOrientationCS(self: *SwingTwistConstraint, orientation: Quat) void {
        const swing_twist = orientation.getSwingTwist();
        var q_swing = swing_twist.swing;
        var q_twist = swing_twist.twist;

        const clamped_axis = self.swing_twist_constraint_part.clampSwingTwist(&q_swing, &q_twist);

        if (clamped_axis != 0)
            self.target_orientation = q_swing.mul(q_twist)
        else
            self.target_orientation = orientation;
    }

    pub fn setupVelocityConstraint(self: *SwingTwistConstraint, delta_time: f32) void {
        const body1 = self.base.body1;
        const body2 = self.base.body2;

        // Setup point constraint
        const rotation1 = Mat44.rotationQuat(body1.getRotation());
        const rotation2 = Mat44.rotationQuat(body2.getRotation());
        self.point_constraint_part.calculateConstraintProperties(body1, rotation1, self.local_space_position1, body2, rotation2, self.local_space_position2);

        // GetRotationInConstraintSpace written out since we reuse the sub expressions
        const constraint_body1_to_world = body1.getRotation().mul(self.constraint_to_body1);
        const constraint_body2_to_world = body2.getRotation().mul(self.constraint_to_body2);
        const q = constraint_body1_to_world.conjugated().mul(constraint_body2_to_world);

        // Calculate constraint properties for the swing twist limit
        self.swing_twist_constraint_part.calculateConstraintProperties(body1, body2, q, constraint_body1_to_world);

        if (self.swing_motor_state != .off or self.twist_motor_state != .off or self.max_friction_torque > 0.0) {
            // Calculate rotation motor axis
            const ws_axis = Mat44.rotationQuat(constraint_body2_to_world);
            for (0..3) |i|
                self.world_space_motor_axis[i] = ws_axis.getColumn3(@intCast(i));

            var rotation_error: Vec3 = undefined;
            if (isPositionMotor(self.swing_motor_state) or isPositionMotor(self.twist_motor_state)) {
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

                // Approximate error angles
                // The imaginary part of a quaternion is rotation_axis * sin(angle / 2)
                // If angle is small, sin(x) = x so angle[i] ~ 2.0f * rotation_axis[i]
                // We'll be making small time steps, so if the angle is not small at least the sign will be correct and we'll move in the right direction
                rotation_error = diff.getXYZ().mulScalar(-2.0);
            }

            // Swing motor
            switch (self.swing_motor_state) {
                .off => {
                    if (self.max_friction_torque > 0.0) {
                        // Enable friction
                        for (1..3) |i|
                            self.motor_constraint_part[i].calculateConstraintProperties(body1, body2, self.world_space_motor_axis[i], .{ .bias = 0.0 });
                    } else {
                        // Disable friction
                        for (&self.motor_constraint_part) |*c|
                            c.deactivate();
                    }
                },

                .velocity => {
                    // Use motor to create angular velocity around desired axis
                    for (1..3) |i|
                        self.motor_constraint_part[i].calculateConstraintProperties(body1, body2, self.world_space_motor_axis[i], .{ .bias = -self.target_angular_velocity.getComponent(@intCast(i)) });
                },

                .position => {
                    // Use motor to drive rotation error to zero
                    if (self.swing_motor_settings.spring_settings.hasStiffness()) {
                        for (1..3) |i|
                            self.motor_constraint_part[i].calculateConstraintPropertiesWithSettingsForMotor(delta_time, body1, body2, self.world_space_motor_axis[i], 0.0, rotation_error.getComponent(@intCast(i)), &self.swing_motor_settings.spring_settings);
                    } else {
                        for (1..3) |i|
                            self.motor_constraint_part[i].deactivate();
                    }
                },

                .position_and_velocity => {
                    // Use motor to drive rotation error to zero
                    if (self.swing_motor_settings.spring_settings.hasStiffnessOrDamping()) {
                        for (1..3) |i|
                            self.motor_constraint_part[i].calculateConstraintPropertiesWithSettingsForMotor(delta_time, body1, body2, self.world_space_motor_axis[i], -self.target_angular_velocity.getComponent(@intCast(i)), rotation_error.getComponent(@intCast(i)), &self.swing_motor_settings.spring_settings);
                    } else {
                        for (1..3) |i|
                            self.motor_constraint_part[i].deactivate();
                    }
                },

                // Jolt's switch has no case for the other values (a restored int, see MotorSettings.zig)
                _ => {},
            }

            // Twist motor
            switch (self.twist_motor_state) {
                .off => {
                    if (self.max_friction_torque > 0.0) {
                        // Enable friction
                        self.motor_constraint_part[0].calculateConstraintProperties(body1, body2, self.world_space_motor_axis[0], .{ .bias = 0.0 });
                    } else {
                        // Disable friction
                        self.motor_constraint_part[0].deactivate();
                    }
                },

                .velocity => {
                    // Use motor to create angular velocity around desired axis
                    self.motor_constraint_part[0].calculateConstraintProperties(body1, body2, self.world_space_motor_axis[0], .{ .bias = -self.target_angular_velocity.getComponent(0) });
                },

                .position => {
                    // Use motor to drive rotation error to zero
                    if (self.twist_motor_settings.spring_settings.hasStiffness())
                        self.motor_constraint_part[0].calculateConstraintPropertiesWithSettingsForMotor(delta_time, body1, body2, self.world_space_motor_axis[0], 0.0, rotation_error.getComponent(0), &self.twist_motor_settings.spring_settings)
                    else
                        self.motor_constraint_part[0].deactivate();
                },

                .position_and_velocity => {
                    // Use motor to drive rotation error to zero
                    if (self.twist_motor_settings.spring_settings.hasStiffnessOrDamping())
                        self.motor_constraint_part[0].calculateConstraintPropertiesWithSettingsForMotor(delta_time, body1, body2, self.world_space_motor_axis[0], -self.target_angular_velocity.getComponent(0), rotation_error.getComponent(0), &self.twist_motor_settings.spring_settings)
                    else
                        self.motor_constraint_part[0].deactivate();
                },

                // Jolt's switch has no case for the other values (a restored int, see MotorSettings.zig)
                _ => {},
            }
        } else {
            // Disable rotation motor
            for (&self.motor_constraint_part) |*c|
                c.deactivate();
        }
    }

    pub fn resetWarmStart(self: *SwingTwistConstraint) void {
        for (&self.motor_constraint_part) |*c|
            c.deactivate();
        self.swing_twist_constraint_part.deactivate();
        self.point_constraint_part.deactivate();
    }

    pub fn warmStartVelocityConstraint(self: *SwingTwistConstraint, warm_start_impulse_ratio: f32) void {
        // Warm starting: Apply previous frame impulse
        for (&self.motor_constraint_part) |*c|
            c.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
        self.swing_twist_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
        self.point_constraint_part.warmStart(self.base.body1, self.base.body2, warm_start_impulse_ratio);
    }

    pub fn solveVelocityConstraint(self: *SwingTwistConstraint, delta_time: f32) bool {
        const body1 = self.base.body1;
        const body2 = self.base.body2;
        var impulse = false;

        // Solve twist rotation motor
        if (self.motor_constraint_part[0].isActive()) {
            // Twist limits
            var min_twist_limit: f32 = undefined;
            var max_twist_limit: f32 = undefined;
            if (self.twist_motor_state == .off) {
                max_twist_limit = delta_time * self.max_friction_torque;
                min_twist_limit = -max_twist_limit;
            } else {
                min_twist_limit = delta_time * self.twist_motor_settings.min_torque_limit;
                max_twist_limit = delta_time * self.twist_motor_settings.max_torque_limit;
            }

            if (self.motor_constraint_part[0].solveVelocityConstraint(body1, body2, self.world_space_motor_axis[0], min_twist_limit, max_twist_limit))
                impulse = true;
        }

        // Solve swing rotation motor
        if (self.motor_constraint_part[1].isActive()) {
            // Swing parts should turn on / off together (not with restricted allowed DOFs, see the file comment)
            if (Core.enable_asserts) std.debug.assert(self.motor_constraint_part[2].isActive());

            // Swing limits
            var min_swing_limit: f32 = undefined;
            var max_swing_limit: f32 = undefined;
            if (self.swing_motor_state == .off) {
                max_swing_limit = delta_time * self.max_friction_torque;
                min_swing_limit = -max_swing_limit;
            } else {
                min_swing_limit = delta_time * self.swing_motor_settings.min_torque_limit;
                max_swing_limit = delta_time * self.swing_motor_settings.max_torque_limit;
            }

            for (1..3) |i| {
                if (self.motor_constraint_part[i].solveVelocityConstraint(body1, body2, self.world_space_motor_axis[i], min_swing_limit, max_swing_limit))
                    impulse = true;
            }
        } else {
            // Swing parts should turn on / off together (not with restricted allowed DOFs, see the file comment)
            if (Core.enable_asserts) std.debug.assert(!self.motor_constraint_part[2].isActive());
        }

        // Solve rotation limits
        if (self.swing_twist_constraint_part.solveVelocityConstraint(body1, body2))
            impulse = true;

        // Solve position constraint
        if (self.point_constraint_part.solveVelocityConstraint(body1, body2))
            impulse = true;

        return impulse;
    }

    pub fn solvePositionConstraint(self: *SwingTwistConstraint, delta_time: f32, baumgarte: f32) bool {
        _ = delta_time;
        const body1 = self.base.body1;
        const body2 = self.base.body2;
        var impulse = false;

        // Solve rotation violations
        const q = self.getRotationInConstraintSpace();
        if (self.swing_twist_constraint_part.solvePositionConstraint(body1, body2, q, self.constraint_to_body1, self.constraint_to_body2, baumgarte))
            impulse = true;

        // Solve position violations
        self.point_constraint_part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), self.local_space_position1, body2, Mat44.rotationQuat(body2.getRotation()), self.local_space_position2);
        if (self.point_constraint_part.solvePositionConstraint(body1, body2, baumgarte))
            impulse = true;

        return impulse;
    }

    // TODO(debug_renderer): DrawConstraint, DrawConstraintLimits

    pub fn saveState(self: *const SwingTwistConstraint, stream: *StateRecorder) void {
        Constraint.impl.saveState(self.asConstraintConst(), stream); // TwoBodyConstraint::SaveState

        self.point_constraint_part.saveState(stream);
        self.swing_twist_constraint_part.saveState(stream);
        for (&self.motor_constraint_part) |*c|
            c.saveState(stream);

        stream.write(self.swing_motor_state);
        stream.write(self.twist_motor_state);
        stream.write(self.target_angular_velocity);
        stream.write(self.target_orientation);
    }

    pub fn restoreState(self: *SwingTwistConstraint, stream: *StateRecorder) void {
        Constraint.impl.restoreState(self.asConstraint(), stream); // TwoBodyConstraint::RestoreState

        self.point_constraint_part.restoreState(stream);
        self.swing_twist_constraint_part.restoreState(stream);
        for (&self.motor_constraint_part) |*c|
            c.restoreState(stream);

        stream.read(&self.swing_motor_state);
        stream.read(&self.twist_motor_state);
        stream.read(&self.target_angular_velocity);
        stream.read(&self.target_orientation);
    }

    pub fn getConstraintSettings(self: *const SwingTwistConstraint) Allocator.Error!Ref(ConstraintSettings) {
        const settings = try SwingTwistConstraintSettings.create(self.base.base.allocator);
        self.base.base.toConstraintSettings(settings.asSettings());
        settings.space = .local_to_body_com;
        settings.position1 = RVec3.fromVec3(self.local_space_position1);
        settings.twist_axis1 = self.constraint_to_body1.rotateAxisX();
        settings.plane_axis1 = self.constraint_to_body1.rotateAxisZ();
        settings.position2 = RVec3.fromVec3(self.local_space_position2);
        settings.twist_axis2 = self.constraint_to_body2.rotateAxisX();
        settings.plane_axis2 = self.constraint_to_body2.rotateAxisZ();
        settings.swing_type = self.swing_twist_constraint_part.getSwingType();
        settings.normal_half_cone_angle = self.normal_half_cone_angle;
        settings.plane_half_cone_angle = self.plane_half_cone_angle;
        settings.twist_min_angle = self.twist_min_angle;
        settings.twist_max_angle = self.twist_max_angle;
        settings.max_friction_torque = self.max_friction_torque;
        settings.swing_motor_settings = self.swing_motor_settings;
        settings.twist_motor_settings = self.twist_motor_settings;
        return .init(settings.asSettings());
    }

    // See: TwoBodyConstraint
    pub fn getConstraintToBody1Matrix(self: *const SwingTwistConstraint) Mat44 {
        return Mat44.rotationTranslation(self.constraint_to_body1, self.local_space_position1);
    }

    pub fn getConstraintToBody2Matrix(self: *const SwingTwistConstraint) Mat44 {
        return Mat44.rotationTranslation(self.constraint_to_body2, self.local_space_position2);
    }

    ///@name Constraint reference frame
    pub fn getLocalSpacePosition1(self: *const SwingTwistConstraint) Vec3 {
        return self.local_space_position1;
    }

    pub fn getLocalSpacePosition2(self: *const SwingTwistConstraint) Vec3 {
        return self.local_space_position2;
    }

    pub fn getConstraintToBody1(self: *const SwingTwistConstraint) Quat {
        return self.constraint_to_body1;
    }

    pub fn getConstraintToBody2(self: *const SwingTwistConstraint) Quat {
        return self.constraint_to_body2;
    }

    ///@name Constraint limits
    pub fn getNormalHalfConeAngle(self: *const SwingTwistConstraint) f32 {
        return self.normal_half_cone_angle;
    }

    pub fn setNormalHalfConeAngle(self: *SwingTwistConstraint, angle: f32) void {
        self.normal_half_cone_angle = angle;
        self.updateLimits();
    }

    pub fn getPlaneHalfConeAngle(self: *const SwingTwistConstraint) f32 {
        return self.plane_half_cone_angle;
    }

    pub fn setPlaneHalfConeAngle(self: *SwingTwistConstraint, angle: f32) void {
        self.plane_half_cone_angle = angle;
        self.updateLimits();
    }

    pub fn getTwistMinAngle(self: *const SwingTwistConstraint) f32 {
        return self.twist_min_angle;
    }

    pub fn setTwistMinAngle(self: *SwingTwistConstraint, angle: f32) void {
        self.twist_min_angle = angle;
        self.updateLimits();
    }

    pub fn getTwistMaxAngle(self: *const SwingTwistConstraint) f32 {
        return self.twist_max_angle;
    }

    pub fn setTwistMaxAngle(self: *SwingTwistConstraint, angle: f32) void {
        self.twist_max_angle = angle;
        self.updateLimits();
    }

    ///@name Motor settings
    pub fn getSwingMotorSettings(self: *const SwingTwistConstraint) *const MotorSettings {
        return &self.swing_motor_settings;
    }

    pub fn getSwingMotorSettingsMut(self: *SwingTwistConstraint) *MotorSettings {
        return &self.swing_motor_settings;
    }

    pub fn getTwistMotorSettings(self: *const SwingTwistConstraint) *const MotorSettings {
        return &self.twist_motor_settings;
    }

    pub fn getTwistMotorSettingsMut(self: *SwingTwistConstraint) *MotorSettings {
        return &self.twist_motor_settings;
    }

    ///@name Friction control
    pub fn setMaxFrictionTorque(self: *SwingTwistConstraint, friction_torque: f32) void {
        self.max_friction_torque = friction_torque;
    }

    pub fn getMaxFrictionTorque(self: *const SwingTwistConstraint) f32 {
        return self.max_friction_torque;
    }

    ///@name Motor controls
    pub fn getSwingMotorState(self: *const SwingTwistConstraint) MotorState {
        return self.swing_motor_state;
    }

    pub fn getTwistMotorState(self: *const SwingTwistConstraint) MotorState {
        return self.twist_motor_state;
    }

    /// Set the target angular velocity of body 2 in constraint space of body 2
    pub fn setTargetAngularVelocityCS(self: *SwingTwistConstraint, angular_velocity: Vec3) void {
        self.target_angular_velocity = angular_velocity;
    }

    pub fn getTargetAngularVelocityCS(self: *const SwingTwistConstraint) Vec3 {
        return self.target_angular_velocity;
    }

    /// Set the target angular velocity of body 2 in body 2 space
    pub fn setTargetAngularVelocityBS(self: *SwingTwistConstraint, angular_velocity: Vec3) void {
        self.target_angular_velocity = self.constraint_to_body2.inverseRotate(angular_velocity);
    }

    pub fn getTargetOrientationCS(self: *const SwingTwistConstraint) Quat {
        return self.target_orientation;
    }

    /// Set the target orientation in body space (R2 = R1 * inOrientation, where R1 and R2 are the world space rotations for body 1 and 2).
    /// Solve: R2 * ConstraintToBody2 = R1 * ConstraintToBody1 * q (see SwingTwistConstraint::GetSwingTwist) and R2 = R1 * inOrientation for q.
    pub fn setTargetOrientationBS(self: *SwingTwistConstraint, orientation: Quat) void {
        self.setTargetOrientationCS(self.constraint_to_body1.conjugated().mul(orientation).mul(self.constraint_to_body2));
    }

    ///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
    pub fn getTotalLambdaPosition(self: *const SwingTwistConstraint) Vec3 {
        return self.point_constraint_part.getTotalLambda();
    }

    pub fn getTotalLambdaTwist(self: *const SwingTwistConstraint) f32 {
        return self.swing_twist_constraint_part.getTotalTwistLambda();
    }

    pub fn getTotalLambdaSwingY(self: *const SwingTwistConstraint) f32 {
        return self.swing_twist_constraint_part.getTotalSwingYLambda();
    }

    pub fn getTotalLambdaSwingZ(self: *const SwingTwistConstraint) f32 {
        return self.swing_twist_constraint_part.getTotalSwingZLambda();
    }

    pub fn getTotalLambdaMotor(self: *const SwingTwistConstraint) Vec3 {
        return Vec3.init(self.motor_constraint_part[0].getTotalLambda(), self.motor_constraint_part[1].getTotalLambda(), self.motor_constraint_part[2].getTotalLambda());
    }
};

// ---------------------------------------------------------------------------------------------------------------------
// Tests (the parity tests are in ZoltParity/Physics/ConstraintsParity.zig, simulations in
// ZoltTests/Physics/SwingTwistConstraintTests.zig)

const testing = std.testing;
const math = @import("../../Math/Math.zig");
const trigonometry = @import("../../Math/Trigonometry.zig");
const Real = @import("../../Math/Real.zig").Real;
const StreamWrapper = @import("../../Core/StreamWrapper.zig");
const StateRecorderImpl = @import("../StateRecorderImpl.zig").StateRecorderImpl;
const SpringSettings = @import("SpringSettings.zig").SpringSettings;
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

/// The number of bytes of the binary state of SwingTwistConstraintSettings: ConstraintSettings (21 bytes), space (int),
/// position 1 (Real), twist axis 1, plane axis 1 (3 floats each), the same for body 2, the swing type (uint8), the four
/// angles and the friction torque, two motors (a spring of 9 bytes and four limits each)
const settings_size = 21 + 4 + 2 * (3 * @sizeOf(Real) + 2 * 12) + 1 + 5 * 4 + 2 * (9 + 16);

/// Offset of the swing type in the binary state
const swing_type_offset = 21 + 4 + 2 * (3 * @sizeOf(Real) + 2 * 12);

test "SwingTwistConstraintSettings: defaults, binary state, restoreFromBinaryState through the registry, copy" {
    const allocator = testing.allocator;

    // Jolt's defaults
    var settings = SwingTwistConstraintSettings.init(allocator);
    defer settings.asSettings().deinit();
    try testing.expectEqual(ConstraintSpace.world_space, settings.space);
    try testing.expect(settings.position1.eql(RVec3.zero()) and settings.position2.eql(RVec3.zero()));
    try testing.expect(settings.twist_axis1.eql(Vec3.axisX()) and settings.plane_axis1.eql(Vec3.axisY()));
    try testing.expect(settings.twist_axis2.eql(Vec3.axisX()) and settings.plane_axis2.eql(Vec3.axisY()));
    try testing.expectEqual(SwingType.cone, settings.swing_type);
    try testing.expectEqual(@as(f32, 0.0), settings.normal_half_cone_angle);
    try testing.expectEqual(@as(f32, 0.0), settings.plane_half_cone_angle);
    try testing.expectEqual(@as(f32, 0.0), settings.twist_min_angle);
    try testing.expectEqual(@as(f32, 0.0), settings.twist_max_angle);
    try testing.expectEqual(@as(f32, 0.0), settings.max_friction_torque);
    const default_motor: MotorSettings = .{};
    try testing.expectEqual(default_motor, settings.swing_motor_settings);
    try testing.expectEqual(default_motor, settings.twist_motor_settings);
    try testing.expectEqualStrings("SwingTwistConstraintSettings", settings.asSettings().vtable.rtti.name);

    // Non default values in every field
    settings.base.base.enabled = false;
    settings.base.base.constraint_priority = 3;
    settings.base.base.num_velocity_steps_override = 9;
    settings.base.base.num_position_steps_override = 1;
    settings.base.base.draw_constraint_size = 2.5;
    settings.base.base.user_data = 0x1234_5678_9ABC; // Not part of the binary state (Jolt)
    settings.space = .local_to_body_com;
    settings.position1 = RVec3.init(1, 2, 3);
    settings.twist_axis1 = Vec3.axisY();
    settings.plane_axis1 = Vec3.axisZ();
    settings.position2 = RVec3.init(-1, 0.5, 0.25);
    settings.twist_axis2 = Vec3.axisZ();
    settings.plane_axis2 = Vec3.axisX();
    settings.swing_type = .pyramid;
    settings.normal_half_cone_angle = 0.5;
    settings.plane_half_cone_angle = 0.25;
    settings.twist_min_angle = -0.75;
    settings.twist_max_angle = 1.5;
    settings.max_friction_torque = 12.0;
    settings.swing_motor_settings = .initModeLimits(.stiffness_and_damping, 100, 2, 50, 30);
    settings.twist_motor_settings = .init(3, 0.5);
    settings.twist_motor_settings.setTorqueLimits(-4, 5);

    const bytes = try testSaveSettings(allocator, settings.asSettingsConst());
    defer allocator.free(bytes);
    try testing.expectEqual(@as(usize, settings_size), bytes.len);
    try testing.expectEqual(RTTI.hashName("SwingTwistConstraintSettings"), std.mem.readInt(u32, bytes[0..4], .little));
    try testing.expectEqual(@as(i32, 0), std.mem.readInt(i32, bytes[21..25], .little)); // LocalToBodyCOM
    try testing.expectEqual(@as(u8, 1), bytes[swing_type_offset]); // ESwingType::Pyramid
    try testing.expectEqual(@as(u32, @bitCast(@as(f32, 12.0))), std.mem.readInt(u32, bytes[swing_type_offset + 17 ..][0..4], .little));
    try testing.expectEqual(@as(u8, 1), bytes[swing_type_offset + 21]); // The spring mode of the swing motor

    // Restore through RegisterTypes.factory
    {
        var result = try testRestoreSettings(allocator, bytes);
        defer result.deinit();
        try testing.expect(result.isValid());
        const restored = virtual.downcast(SwingTwistConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expect(!restored.base.base.enabled);
        try testing.expectEqual(@as(u32, 3), restored.base.base.constraint_priority);
        try testing.expectEqual(@as(u32, 9), restored.base.base.num_velocity_steps_override);
        try testing.expectEqual(@as(u32, 1), restored.base.base.num_position_steps_override);
        try testing.expectEqual(@as(f32, 2.5), restored.base.base.draw_constraint_size);
        try testing.expectEqual(@as(u64, 0), restored.base.base.user_data);
        try testing.expectEqual(ConstraintSpace.local_to_body_com, restored.space);
        try testing.expect(restored.position1.eql(settings.position1) and restored.position2.eql(settings.position2));
        try testing.expect(restored.twist_axis1.eql(settings.twist_axis1) and restored.plane_axis1.eql(settings.plane_axis1));
        try testing.expect(restored.twist_axis2.eql(settings.twist_axis2) and restored.plane_axis2.eql(settings.plane_axis2));
        try testing.expectEqual(SwingType.pyramid, restored.swing_type);
        try testing.expectEqual(settings.normal_half_cone_angle, restored.normal_half_cone_angle);
        try testing.expectEqual(settings.plane_half_cone_angle, restored.plane_half_cone_angle);
        try testing.expectEqual(settings.twist_min_angle, restored.twist_min_angle);
        try testing.expectEqual(settings.twist_max_angle, restored.twist_max_angle);
        try testing.expectEqual(settings.max_friction_torque, restored.max_friction_torque);
        try testing.expectEqual(settings.swing_motor_settings, restored.swing_motor_settings);
        try testing.expectEqual(settings.twist_motor_settings, restored.twist_motor_settings);
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, bytes, resaved);
    }

    // A swing type byte outside the named values is restored as it is (Jolt's enum class with underlying type uint8)
    {
        var odd = try allocator.dupe(u8, bytes);
        defer allocator.free(odd);
        odd[swing_type_offset] = 7;
        var result = try testRestoreSettings(allocator, odd);
        defer result.deinit();
        const restored = virtual.downcast(SwingTwistConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, result.getPtr().?));
        try testing.expectEqual(@as(u8, 7), @intFromEnum(restored.swing_type));
        const resaved = try testSaveSettings(allocator, restored.asSettingsConst());
        defer allocator.free(resaved);
        try testing.expectEqualSlices(u8, odd, resaved);
    }

    // Truncated anywhere: Jolt's error
    for ([_]usize{ 4, 21, swing_type_offset, swing_type_offset + 1, bytes.len - 1 }) |len| {
        var result = try testRestoreSettings(allocator, bytes[0..len]);
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
    var copy = SwingTwistConstraintSettings.initCopy(allocator, &settings);
    defer copy.asSettings().deinit();
    try testing.expectEqual(@as(u32, 0), copy.asSettings().getRefCount());
    try testing.expect(copy.plane_axis2.eql(settings.plane_axis2) and copy.swing_type == .pyramid);
    try testing.expectEqual(settings.twist_motor_settings, copy.twist_motor_settings);
    try testing.expectEqual(@as(u64, 0x1234_5678_9ABC), copy.base.base.user_data);
}

/// Settings in world space at `position` with the twist axis along X and the plane axis along Y on both bodies
fn testSettings(allocator: Allocator, position: RVec3) SwingTwistConstraintSettings {
    var settings = SwingTwistConstraintSettings.init(allocator);
    settings.position1 = position;
    settings.position2 = position;
    settings.normal_half_cone_angle = 0.6;
    settings.plane_half_cone_angle = 0.4;
    settings.twist_min_angle = -0.5;
    settings.twist_max_angle = 0.7;
    return settings;
}

test "SwingTwistConstraint: virtual dispatch, space conversions, getters and setters, settings and state" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();

    for (TestContext.motion_type_pairs) |pair| {
        const bodies = try context.createBodyPair(pair);
        defer context.destroyBodies(&bodies);
        const body1 = bodies[0];
        const body2 = bodies[1];

        // World space: twist axis Y and plane axis Z for body 1, twist axis Z and plane axis X for body 2
        var settings = testSettings(allocator, RVec3.init(1.5, 1.75, 3.5));
        defer settings.asSettings().deinit();
        settings.twist_axis1 = Vec3.axisY();
        settings.plane_axis1 = Vec3.axisZ();
        settings.twist_axis2 = Vec3.axisZ();
        settings.plane_axis2 = Vec3.axisX();
        settings.swing_type = .pyramid;
        settings.max_friction_torque = 2.0;
        settings.swing_motor_settings = .init(4, 1);
        settings.twist_motor_settings = .initLimits(6, 0.5, 10, 20);
        settings.base.base.user_data = 11;

        // TwoBodyConstraintSettings::Create through the vtable
        var constraint = Ref(Constraint).init(&(try settings.asTwoBodySettingsConst().createConstraint(allocator, body1, body2)).base);
        defer constraint.deinit();
        const base = constraint.get().?;
        const two_body = virtual.downcast(TwoBodyConstraint, base);
        const c = virtual.downcast(SwingTwistConstraint, two_body);
        try testing.expectEqual(ConstraintFile.ConstraintType.two_body_constraint, base.getType());
        try testing.expectEqual(ConstraintSubType.swing_twist, base.getSubType());
        try testing.expectEqual(@as(u64, 11), base.getUserData());
        try testing.expectEqual(pair[0] == .dynamic or pair[1] == .dynamic, base.isActive());

        // The constraint space of body 1 maps X to the twist axis, Z to the plane axis and Y to plane x twist (in the local
        // space of the body)
        try testing.expect(c.getLocalSpacePosition1().eql(body1.getInverseCenterOfMassTransform().mulRVec3(settings.position1).toVec3()));
        try testing.expect(c.getLocalSpacePosition2().eql(body2.getInverseCenterOfMassTransform().mulRVec3(settings.position2).toVec3()));
        const close: f32 = 1.0e-10;
        try testing.expect(body1.getRotation().mulVec3(c.getConstraintToBody1().rotateAxisX()).isClose(Vec3.axisY(), .{ .max_dist_sq = close }));
        try testing.expect(body1.getRotation().mulVec3(c.getConstraintToBody1().rotateAxisY()).isClose(Vec3.axisX().negate(), .{ .max_dist_sq = close }));
        try testing.expect(body1.getRotation().mulVec3(c.getConstraintToBody1().rotateAxisZ()).isClose(Vec3.axisZ(), .{ .max_dist_sq = close }));
        try testing.expect(body2.getRotation().mulVec3(c.getConstraintToBody2().rotateAxisX()).isClose(Vec3.axisZ(), .{ .max_dist_sq = close }));
        try testing.expect(body2.getRotation().mulVec3(c.getConstraintToBody2().rotateAxisZ()).isClose(Vec3.axisX(), .{ .max_dist_sq = close }));
        try testing.expect(two_body.getConstraintToBody1Matrix().eql(Mat44.rotationTranslation(c.getConstraintToBody1(), c.getLocalSpacePosition1())));
        try testing.expect(two_body.getConstraintToBody2Matrix().eql(Mat44.rotationTranslation(c.getConstraintToBody2(), c.getLocalSpacePosition2())));

        // The rotation in constraint space: (q1 c1)^-1 (q2 c2), here a rotation of the twist axis Y onto Z
        const q = c.getRotationInConstraintSpace();
        try testing.expect(q.isClose(body1.getRotation().mul(c.getConstraintToBody1()).conjugated().mul(body2.getRotation().mul(c.getConstraintToBody2())), .{}));
        try testing.expect(q.isNormalized(.{}));

        // The values of the settings and the limits of the part (plane half cone angle = swing Y, normal = swing Z)
        try testing.expectEqual(@as(f32, 0.6), c.getNormalHalfConeAngle());
        try testing.expectEqual(@as(f32, 0.4), c.getPlaneHalfConeAngle());
        try testing.expectEqual(@as(f32, -0.5), c.getTwistMinAngle());
        try testing.expectEqual(@as(f32, 0.7), c.getTwistMaxAngle());
        try testing.expectEqual(@as(f32, 2.0), c.getMaxFrictionTorque());
        try testing.expectEqual(settings.swing_motor_settings, c.getSwingMotorSettings().*);
        try testing.expectEqual(settings.twist_motor_settings, c.getTwistMotorSettings().*);
        try testing.expectEqual(SwingType.pyramid, c.swing_twist_constraint_part.getSwingType());
        try testing.expectEqual(@as(f32, -0.2), c.swing_twist_constraint_part.swing_y_half_min_angle);
        try testing.expectEqual(@as(f32, 0.3), c.swing_twist_constraint_part.swing_z_half_max_angle);
        try testing.expectEqual(MotorState.off, c.getSwingMotorState());
        try testing.expectEqual(MotorState.off, c.getTwistMotorState());
        try testing.expect(c.getTargetAngularVelocityCS().eql(Vec3.zero()));
        try testing.expect(c.getTargetOrientationCS().eql(Quat.identity()));

        // The limit setters update the part
        c.setNormalHalfConeAngle(0.8);
        try testing.expectEqual(@as(f32, 0.8), c.getNormalHalfConeAngle());
        try testing.expectEqual(@as(f32, -0.4), c.swing_twist_constraint_part.swing_z_half_min_angle);
        c.setPlaneHalfConeAngle(0.2);
        try testing.expectEqual(@as(f32, 0.2), c.getPlaneHalfConeAngle());
        try testing.expectEqual(@as(f32, 0.1), c.swing_twist_constraint_part.swing_y_half_max_angle);
        c.setTwistMinAngle(-0.3);
        try testing.expectEqual(@as(f32, -0.3), c.getTwistMinAngle());
        try testing.expectEqual(trigonometry.sin(-0.15), c.swing_twist_constraint_part.sin_twist_half_min_angle);
        c.setTwistMaxAngle(0.9);
        try testing.expectEqual(@as(f32, 0.9), c.getTwistMaxAngle());
        try testing.expectEqual(trigonometry.cos(0.45), c.swing_twist_constraint_part.cos_twist_half_max_angle);
        c.setMaxFrictionTorque(1.5);
        try testing.expectEqual(@as(f32, 1.5), c.getMaxFrictionTorque());
        c.getSwingMotorSettingsMut().setTorqueLimit(7);
        try testing.expectEqual(@as(f32, -7), c.getSwingMotorSettings().min_torque_limit);
        c.getTwistMotorSettingsMut().spring_settings = .init(.stiffness_and_damping, 50, 3);
        try testing.expectEqual(@as(f32, 50), c.getTwistMotorSettings().spring_settings.getStiffness());

        // Target angular velocity in constraint space and in body space (taken to the constraint space of body 2)
        c.setTargetAngularVelocityCS(Vec3.init(1, 2, 3));
        try testing.expect(c.getTargetAngularVelocityCS().eql(Vec3.init(1, 2, 3)));
        c.setTargetAngularVelocityBS(Vec3.init(-1, 0.5, 2));
        try testing.expect(c.getTargetAngularVelocityCS().eql(c.getConstraintToBody2().inverseRotate(Vec3.init(-1, 0.5, 2))));

        // Target orientation: kept when inside the limits, clamped (swing * twist) when outside
        const inside = Quat.rotation(Vec3.axisX(), 0.2);
        c.setTargetOrientationCS(inside);
        try testing.expect(c.getTargetOrientationCS().eql(inside));
        const outside = Quat.rotation(Vec3.axisX(), 1.5).mul(Quat.rotation(Vec3.axisY(), 0.05));
        c.setTargetOrientationCS(outside);
        {
            const st = outside.getSwingTwist();
            var swing = st.swing;
            var twist = st.twist;
            try testing.expect(c.swing_twist_constraint_part.clampSwingTwist(&swing, &twist) != 0);
            try testing.expect(c.getTargetOrientationCS().eql(swing.mul(twist)));
            try testing.expect(!c.getTargetOrientationCS().isClose(outside, .{}));
        }
        const body_space = Quat.rotation(Vec3.axisZ(), 0.1);
        c.setTargetOrientationBS(body_space);
        {
            const cs = c.getConstraintToBody1().conjugated().mul(body_space).mul(c.getConstraintToBody2());
            const st = cs.getSwingTwist();
            var swing = st.swing;
            var twist = st.twist;
            const expected = if (c.swing_twist_constraint_part.clampSwingTwist(&swing, &twist) != 0) swing.mul(twist) else cs;
            try testing.expect(c.getTargetOrientationCS().eql(expected));
        }

        // Motor states: changing the swing state deactivates all motor parts, the twist state only the twist part, the same
        // state changes nothing
        c.motor_constraint_part = @splat(.{ .effective_mass = 1.0, .total_lambda = 2.0 });
        c.setSwingMotorState(.off);
        try testing.expectEqual(@as(f32, 2.0), c.getTotalLambdaMotor().getX());
        c.setTwistMotorState(.velocity);
        try testing.expectEqual(MotorState.velocity, c.getTwistMotorState());
        try testing.expect(c.getTotalLambdaMotor().eql(Vec3.init(0, 2, 2)));
        try testing.expect(!c.motor_constraint_part[0].isActive() and c.motor_constraint_part[1].isActive());
        c.setSwingMotorState(.position);
        try testing.expectEqual(MotorState.position, c.getSwingMotorState());
        try testing.expect(c.getTotalLambdaMotor().eql(Vec3.zero()));
        try testing.expect(!c.motor_constraint_part[1].isActive() and !c.motor_constraint_part[2].isActive());

        // GetConstraintSettings: local space, the axes from the constraint to body rotations, every value
        {
            var copy = try base.getConstraintSettings();
            defer copy.deinit();
            const s = virtual.downcast(SwingTwistConstraintSettings, virtual.downcast(TwoBodyConstraintSettings, copy.get().?));
            try testing.expectEqual(ConstraintSpace.local_to_body_com, s.space);
            try testing.expectEqual(@as(u64, 11), s.base.base.user_data);
            try testing.expect(s.position1.eql(RVec3.fromVec3(c.getLocalSpacePosition1())) and s.position2.eql(RVec3.fromVec3(c.getLocalSpacePosition2())));
            try testing.expect(s.twist_axis1.eql(c.getConstraintToBody1().rotateAxisX()) and s.plane_axis1.eql(c.getConstraintToBody1().rotateAxisZ()));
            try testing.expect(s.twist_axis2.eql(c.getConstraintToBody2().rotateAxisX()) and s.plane_axis2.eql(c.getConstraintToBody2().rotateAxisZ()));
            try testing.expectEqual(SwingType.pyramid, s.swing_type);
            try testing.expectEqual(@as(f32, 0.8), s.normal_half_cone_angle);
            try testing.expectEqual(@as(f32, 0.2), s.plane_half_cone_angle);
            try testing.expectEqual(@as(f32, -0.3), s.twist_min_angle);
            try testing.expectEqual(@as(f32, 0.9), s.twist_max_angle);
            try testing.expectEqual(@as(f32, 1.5), s.max_friction_torque);
            try testing.expectEqual(c.getSwingMotorSettings().*, s.swing_motor_settings);
            try testing.expectEqual(c.getTwistMotorSettings().*, s.twist_motor_settings);

            // A constraint made from them has the same frames
            var again = SwingTwistConstraint.init(allocator, body1, body2, s);
            defer again.asConstraint().deinit();
            try testing.expect(again.getLocalSpacePosition1().eql(c.getLocalSpacePosition1()) and again.getLocalSpacePosition2().eql(c.getLocalSpacePosition2()));
            try testing.expect(again.getConstraintToBody1().isClose(c.getConstraintToBody1(), .{ .max_dist_sq = close }));
            try testing.expect(again.getConstraintToBody2().isClose(c.getConstraintToBody2(), .{ .max_dist_sq = close }));
        }

        // SaveState / RestoreState: enabled, the lambdas of the point part (3), the swing twist part (3) and the motors (3),
        // the motor states, the target angular velocity and the target orientation
        var recorder = StateRecorderImpl.init(allocator);
        defer recorder.deinit();
        c.motor_constraint_part[2].total_lambda = 0.5;
        c.swing_twist_constraint_part.twist_limit_constraint_part.total_lambda = -1.5;
        c.point_constraint_part.total_lambda = Vec3.init(1, 2, 3);
        base.saveState(recorder.recorder());
        try testing.expectEqual(@as(usize, 1 + 12 + 3 * 4 + 3 * 4 + 2 * 4 + 12 + 16), recorder.getData().len);
        const target_velocity = c.getTargetAngularVelocityCS();
        const target_orientation = c.getTargetOrientationCS();
        base.resetWarmStart();
        try testing.expect(c.getTotalLambdaMotor().eql(Vec3.zero()) and c.getTotalLambdaTwist() == 0.0 and c.getTotalLambdaPosition().eql(Vec3.zero()));
        c.setSwingMotorState(.off);
        c.setTwistMotorState(.position_and_velocity);
        c.setTargetAngularVelocityCS(Vec3.zero());
        c.setTargetOrientationCS(Quat.identity());
        base.restoreState(recorder.recorder());
        try testing.expect(c.getTotalLambdaMotor().eql(Vec3.init(0, 0, 0.5)));
        try testing.expectEqual(@as(f32, -1.5), c.getTotalLambdaTwist());
        try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaSwingY());
        try testing.expectEqual(@as(f32, 0.0), c.getTotalLambdaSwingZ());
        try testing.expect(c.getTotalLambdaPosition().eql(Vec3.init(1, 2, 3)));
        try testing.expectEqual(MotorState.position, c.getSwingMotorState());
        try testing.expectEqual(MotorState.velocity, c.getTwistMotorState());
        try testing.expect(c.getTargetAngularVelocityCS().eql(target_velocity) and c.getTargetOrientationCS().eql(target_orientation));

        // NotifyShapeChanged moves the attachment point of the body that changed
        const p1 = c.getLocalSpacePosition1();
        const p2 = c.getLocalSpacePosition2();
        base.notifyShapeChanged(body2.getID(), Vec3.init(0.5, 0, 0));
        try testing.expect(c.getLocalSpacePosition1().eql(p1) and c.getLocalSpacePosition2().eql(p2.sub(Vec3.init(0.5, 0, 0))));
        base.notifyShapeChanged(body1.getID(), Vec3.init(0, 1, 0));
        try testing.expect(c.getLocalSpacePosition1().eql(p1.sub(Vec3.init(0, 1, 0))));
        base.notifyShapeChanged(BodyID.init(12345), Vec3.init(1, 1, 1));
        try testing.expect(c.getLocalSpacePosition1().eql(p1.sub(Vec3.init(0, 1, 0))) and c.getLocalSpacePosition2().eql(p2.sub(Vec3.init(0.5, 0, 0))));

        // A disabled constraint is not active (TwoBodyConstraint::IsActive)
        base.setEnabled(false);
        try testing.expect(!base.isActive());
    }
}

test "SwingTwistConstraint: local space" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const bodies = try context.createBodyPair(.{ .kinematic, .dynamic });
    defer context.destroyBodies(&bodies);

    var settings = testSettings(allocator, RVec3.init(0.25, 0.5, 0.75));
    defer settings.asSettings().deinit();
    settings.space = .local_to_body_com;
    settings.position2 = RVec3.init(-1, -2, -3);
    var c = SwingTwistConstraint.init(allocator, bodies[0], bodies[1], &settings);
    defer c.asConstraint().deinit();
    try testing.expect(c.getLocalSpacePosition1().eql(Vec3.init(0.25, 0.5, 0.75)) and c.getLocalSpacePosition2().eql(Vec3.init(-1, -2, -3)));
    // Twist X, plane Y: the normal axis is Y x X = -Z, the matrix (X, -Z, Y) is a rotation of -90 degrees around X
    try testing.expect(c.getConstraintToBody1().eql(c.getConstraintToBody2()));
    try testing.expect(c.getConstraintToBody1().isClose(Quat.rotation(Vec3.axisX(), -0.5 * math.pi), .{}));
    try testing.expect(c.getConstraintToBody1().rotateAxisZ().isClose(Vec3.axisY(), .{}));
}

/// The bodies of a solver test: body 1 static, body 2 a dynamic box at `position` without velocities
fn createSolverBodies(context: *TestContext.ConstraintPartTestContext, position: RVec3) ![2]*Body {
    const body1 = try context.createBody(.static, .{ .position = position.add(RVec3.init(0, 2, 0)) });
    const body2 = try context.createBody(.dynamic, .{ .position = position, .half_extent = Vec3.init(0.5, 0.5, 0.5) });
    return .{ body1, body2 };
}

/// One frame of the solver: setup, warm start, `n` velocity iterations, returns if any of them applied an impulse
fn solveFrame(base: *Constraint, n: usize) bool {
    base.setupVelocityConstraint(1.0 / 60.0);
    base.warmStartVelocityConstraint(1.0);
    var applied = false;
    for (0..n) |_| {
        if (base.solveVelocityConstraint(1.0 / 60.0))
            applied = true;
    }
    return applied;
}

test "SwingTwistConstraint: motors in every state, friction" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const position = RVec3.init(1, 2, 3);
    const bodies = try createSolverBodies(&context, position);
    defer context.destroyBodies(&bodies);
    const body2 = bodies[1];

    // The joint at the center of mass of body 2: the point constraint does not rotate it
    var settings = testSettings(allocator, position);
    defer settings.asSettings().deinit();
    settings.twist_min_angle = -math.pi;
    settings.twist_max_angle = math.pi;
    var c = SwingTwistConstraint.init(allocator, bodies[0], body2, &settings);
    defer c.asConstraint().deinit();
    const base = c.asConstraint();

    // No motor, no friction: the motor parts are off
    _ = solveFrame(base, 1);
    for (c.motor_constraint_part) |p| try testing.expect(!p.isActive());

    // Friction: all motor parts on, the impulse of each is limited by dt * max friction torque
    c.setMaxFrictionTorque(0.5);
    body2.setAngularVelocity(Vec3.init(3, 2, -1));
    _ = solveFrame(base, 4);
    for (c.motor_constraint_part) |p| try testing.expect(p.isActive());
    const lambda = c.getTotalLambdaMotor();
    for (0..3) |i| try testing.expect(@abs(lambda.getComponent(@intCast(i))) <= 0.5 / 60.0 + 1.0e-7);
    try testing.expect(!lambda.isNearZero(.{}));
    c.setMaxFrictionTorque(0.0);

    // Velocity motors: body 2 turns with the target angular velocity in its constraint space (body 2 is not rotated, the
    // constraint space is X = twist, Y = normal = -Z, Z = plane = Y)
    c.setTwistMotorState(.velocity);
    c.setSwingMotorState(.velocity);
    c.setTargetAngularVelocityCS(Vec3.init(0.5, -0.25, 0.125));
    body2.setAngularVelocity(Vec3.zero());
    _ = solveFrame(base, 10);
    try testing.expect(body2.getAngularVelocity().isClose(Vec3.init(0.5, 0.125, 0.25), .{ .max_dist_sq = 1.0e-8 }));
    try testing.expect(c.getConstraintToBody2().inverseRotate(body2.getAngularVelocity()).isClose(c.getTargetAngularVelocityCS(), .{ .max_dist_sq = 1.0e-8 }));

    // A twist velocity motor alone, swing friction off: only the twist part is on
    c.setSwingMotorState(.off);
    _ = solveFrame(base, 2);
    try testing.expect(c.motor_constraint_part[0].isActive() and !c.motor_constraint_part[1].isActive() and !c.motor_constraint_part[2].isActive());

    // Torque limits of the motors
    c.getTwistMotorSettingsMut().setTorqueLimit(0.001);
    c.setTargetAngularVelocityCS(Vec3.init(-5, 0, 0));
    _ = solveFrame(base, 5);
    try testing.expect(@abs(c.getTotalLambdaMotor().getX()) <= 0.001 / 60.0 + 1.0e-9);
    c.getTwistMotorSettingsMut().setTorqueLimit(math.flt_max);

    // Position motors: body 2 turns towards the target orientation (inside the limits: kept as it is)
    body2.setAngularVelocity(Vec3.zero());
    c.setTwistMotorState(.position);
    c.setSwingMotorState(.position_and_velocity);
    c.setTargetAngularVelocityCS(Vec3.zero());
    const target = Quat.rotation(Vec3.axisX(), 0.3).mul(Quat.rotation(Vec3.axisZ(), -0.2));
    c.setTargetOrientationCS(target);
    try testing.expect(c.getTargetOrientationCS().eql(target));
    _ = solveFrame(base, 5);
    for (c.motor_constraint_part) |p| try testing.expect(p.isActive() and p.spring_part.isActive());
    const w_cs = c.getConstraintToBody2().inverseRotate(body2.getAngularVelocity());
    try testing.expect(w_cs.getX() > 0.0 and w_cs.getZ() < 0.0);

    // A position motor without stiffness deactivates its parts, with only damping the position and velocity motor works
    c.getSwingMotorSettingsMut().spring_settings = .init(.stiffness_and_damping, 0, 2);
    c.getTwistMotorSettingsMut().spring_settings = .init(.stiffness_and_damping, 0, 2);
    _ = solveFrame(base, 1);
    try testing.expect(!c.motor_constraint_part[0].isActive()); // Twist: position, no stiffness
    try testing.expect(c.motor_constraint_part[1].isActive() and c.motor_constraint_part[2].isActive()); // Swing: damping
    c.getSwingMotorSettingsMut().spring_settings = .init(.frequency_and_damping, 0, 2);
    _ = solveFrame(base, 1);
    try testing.expect(!c.motor_constraint_part[1].isActive() and !c.motor_constraint_part[2].isActive()); // No spring

    // A motor state that Jolt's switch does not handle (a restored int) leaves the parts as they are
    c.setTwistMotorState(.velocity);
    c.getTwistMotorSettingsMut().spring_settings = .init(.frequency_and_damping, 2, 1);
    _ = solveFrame(base, 1);
    try testing.expect(c.motor_constraint_part[0].isActive());
    c.twist_motor_state = @enumFromInt(7);
    c.swing_motor_state = @enumFromInt(-3);
    c.motor_constraint_part[0].total_lambda = 0.25;
    base.setupVelocityConstraint(1.0 / 60.0);
    try testing.expect(c.motor_constraint_part[0].isActive());
    try testing.expectEqual(@as(f32, 0.25), c.getTotalLambdaMotor().getX());
}

test "SwingTwistConstraint: the limits" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const position = RVec3.init(-2, 1, 0.5);

    for ([_]SwingType{ .cone, .pyramid }) |swing_type| {
        const bodies = try createSolverBodies(&context, position);
        defer context.destroyBodies(&bodies);
        const body2 = bodies[1];

        var settings = testSettings(allocator, position);
        defer settings.asSettings().deinit();
        settings.swing_type = swing_type;
        var c = SwingTwistConstraint.init(allocator, bodies[0], body2, &settings);
        defer c.asConstraint().deinit();
        const base = c.asConstraint();

        // Inside the limits nothing is active
        _ = solveFrame(base, 1);
        try testing.expect(!c.swing_twist_constraint_part.isActive());

        // Twisted beyond the max twist angle and spinning further: the twist limit pushes back
        body2.setPositionAndRotationInternal(body2.getPosition(), Quat.rotation(Vec3.axisX(), 0.9), .{});
        body2.setAngularVelocity(Vec3.init(2, 0, 0));
        try testing.expect(c.getRotationInConstraintSpace().isClose(Quat.rotation(Vec3.axisX(), 0.9), .{}));
        try testing.expect(solveFrame(base, 5));
        try testing.expect(c.swing_twist_constraint_part.twist_limit_constraint_part.isActive());
        try testing.expect(body2.getAngularVelocity().getX() <= 1.0e-5);
        try testing.expect(c.getTotalLambdaTwist() != 0.0);

        // The position solver rotates it back towards the limit
        try testing.expect(base.solvePositionConstraint(1.0 / 60.0, 0.5));
        try testing.expect(c.getRotationInConstraintSpace().getSwingTwist().twist.getX() < trigonometry.sin(0.45));

        // Swung beyond the cone / pyramid
        body2.setPositionAndRotationInternal(body2.getPosition(), Quat.rotation(Vec3.axisZ(), 1.0), .{});
        body2.setAngularVelocity(Vec3.init(0, 0, 1));
        base.resetWarmStart();
        try testing.expect(solveFrame(base, 5));
        try testing.expect(c.swing_twist_constraint_part.isActive());
        try testing.expect(body2.getAngularVelocity().getZ() <= 1.0e-5);
        try testing.expect(c.getTotalLambdaSwingY() != 0.0 or c.getTotalLambdaSwingZ() != 0.0);
        try testing.expect(base.solvePositionConstraint(1.0 / 60.0, 0.5));
    }
}

test "SwingTwistConstraint: body 1 fixed to the world" {
    const allocator = testing.allocator;
    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(allocator, testing.io);
    defer context.deinit();
    const body = try context.createBody(.dynamic, .{ .position = RVec3.init(1, 2, 3), .linear_velocity = Vec3.init(1, 0, 0), .angular_velocity = Vec3.init(0, 0.5, 0) });
    defer context.destroyBodies(&.{body});
    var settings = testSettings(allocator, RVec3.init(1, 3, 3));
    defer settings.asSettings().deinit();
    settings.twist_axis1 = Vec3.axisY().negate();
    settings.plane_axis1 = Vec3.axisX();
    settings.twist_axis2 = Vec3.axisY().negate();
    settings.plane_axis2 = Vec3.axisX();
    var c = SwingTwistConstraint.init(allocator, Body.fixedToWorld(), body, &settings);
    defer c.asConstraint().deinit();
    try testing.expect(c.getLocalSpacePosition1().eql(Vec3.init(1, 3, 3))); // The world body has an identity transform
    try testing.expect(c.getRotationInConstraintSpace().isClose(Quat.identity(), .{}));
    c.setTwistMotorState(.velocity);
    c.setTargetAngularVelocityCS(Vec3.init(-1, 0, 0)); // Around -(-Y) = Y in the world
    const base = c.asConstraint();
    try testing.expect(solveFrame(base, 10));
    try testing.expect(body.getAngularVelocity().getY() > 0.9);
    _ = base.solvePositionConstraint(1.0 / 60.0, 0.2);
}

test "SwingTwistConstraint: out of memory" {
    const S = struct {
        fn run(allocator: Allocator) !void {
            var context: TestContext.ConstraintPartTestContext = undefined;
            try context.init(allocator, testing.io);
            defer context.deinit();
            const bodies = try context.createBodyPair(.{ .dynamic, .kinematic });
            defer context.destroyBodies(&bodies);
            var settings = Ref(ConstraintSettings).init((try SwingTwistConstraintSettings.create(allocator)).asSettings());
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
