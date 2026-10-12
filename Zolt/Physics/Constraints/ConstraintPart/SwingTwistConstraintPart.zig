//! Port of: Jolt/Physics/Constraints/ConstraintPart/SwingTwistConstraintPart.h
//! Status: complete
//!
//! - `ClampSwingTwist(Quat &ioSwing, Quat &ioTwist, uint &outClampedAxis)` is `clampSwingTwist(&swing, &twist) u32`
//!   (the in/out quaternions stay pointers, the out parameter is the return value).
//! - `sDistanceToMinShorter` is `distanceToMinShorter`, the `cClamped*` constants are `clamped_*` (`u32`).
//! - The private `ERotationFlags` enum is the namespace `RotationFlags` of `u8` constants on `rotation_flags`.
//! - `impulse |= part.SolveVelocityConstraint(...)` always runs the solve (no short circuit).

const std = @import("std");
const Core = @import("../../../Core/Core.zig");
const math = @import("../../../Math/Math.zig");
const Vec3 = @import("../../../Math/Vec3.zig").Vec3;
const Vec4 = @import("../../../Math/Vec4.zig").Vec4;
const Quat = @import("../../../Math/Quat.zig").Quat;
const Mat44 = @import("../../../Math/Mat44.zig").Mat44;
const Float2 = @import("../../../Math/Float2.zig").Float2;
const Ellipse = @import("../../../Geometry/Ellipse.zig").Ellipse;
const Body = @import("../../Body/Body.zig").Body;
const StateRecorder = @import("../../StateRecorder.zig").StateRecorder;
const AngleConstraintPart = @import("AngleConstraintPart.zig").AngleConstraintPart;
const RotationEulerConstraintPart = @import("RotationEulerConstraintPart.zig").RotationEulerConstraintPart;

/// How the swing limit behaves (ESwingType)
/// (Jolt's enum class has the fixed underlying type uint8, which the binary state of the SixDOF / SwingTwist constraint
/// settings writes; like SpringMode and MotorState every byte is a valid ESwingType, so a restored value outside the
/// named ones is not illegal behavior: it behaves like Pyramid, Jolt only compares with Cone)
pub const SwingType = enum(u8) {
    /// Swing is limited by a cone shape, note that this cone starts to deform for larger swing angles. Cone limits only support limits that are symmetric around 0.
    cone,
    /// Swing is limited by a pyramid shape, note that this pyramid starts to deform for larger swing angles.
    pyramid,
    /// Any other uint8 value
    _,
};

/// Quaternion based constraint that decomposes the rotation in constraint space in swing and twist: q = q_swing * q_twist
/// where q_swing.x = 0 and where q_twist.y = q_twist.z = 0
///
/// - Rotation around the twist (x-axis) is within [inTwistMinAngle, inTwistMaxAngle].
/// - Rotation around the swing axis (y and z axis) are limited to an ellipsoid in quaternion space formed by the equation:
///
/// (q_swing.y / sin(inSwingYHalfAngle / 2))^2 + (q_swing.z / sin(inSwingZHalfAngle / 2))^2 <= 1
///
/// Which roughly corresponds to an elliptic cone shape with major axis (inSwingYHalfAngle, inSwingZHalfAngle).
///
/// In case inSwingYHalfAngle = 0, the rotation around Y will be constrained to 0 and the rotation around Z
/// will be constrained between [-inSwingZHalfAngle, inSwingZHalfAngle]. Vice versa if inSwingZHalfAngle = 0.
pub const SwingTwistConstraintPart = struct {
    /// Flags to indicate which axis got clamped by ClampSwingTwist
    pub const clamped_twist_min: u32 = 1 << 0;
    pub const clamped_twist_max: u32 = 1 << 1;
    pub const clamped_swing_y_min: u32 = 1 << 2;
    pub const clamped_swing_y_max: u32 = 1 << 3;
    pub const clamped_swing_z_min: u32 = 1 << 4;
    pub const clamped_swing_z_max: u32 = 1 << 5;

    // CONFIGURATION PROPERTIES FOLLOW

    /// ERotationFlags
    const RotationFlags = struct {
        /// Indicates that axis is completely locked (cannot rotate around this axis)
        const twist_x_locked: u8 = 1 << 0;
        const swing_y_locked: u8 = 1 << 1;
        const swing_z_locked: u8 = 1 << 2;

        /// Indicates that axis is completely free (can rotate around without limits)
        const twist_x_free: u8 = 1 << 3;
        const swing_y_free: u8 = 1 << 4;
        const swing_z_free: u8 = 1 << 5;
        const swing_yz_free: u8 = swing_y_free | swing_z_free;
    };

    rotation_flags: u8 = undefined,

    // Constants
    swing_type: SwingType = .cone,
    sin_twist_half_min_angle: f32 = undefined,
    sin_twist_half_max_angle: f32 = undefined,
    cos_twist_half_min_angle: f32 = undefined,
    cos_twist_half_max_angle: f32 = undefined,
    swing_y_half_min_angle: f32 = undefined,
    swing_y_half_max_angle: f32 = undefined,
    swing_z_half_min_angle: f32 = undefined,
    swing_z_half_max_angle: f32 = undefined,
    sin_swing_y_half_min_angle: f32 = undefined,
    sin_swing_y_half_max_angle: f32 = undefined,
    sin_swing_z_half_min_angle: f32 = undefined,
    sin_swing_z_half_max_angle: f32 = undefined,
    cos_swing_y_half_min_angle: f32 = undefined,
    cos_swing_y_half_max_angle: f32 = undefined,
    cos_swing_z_half_min_angle: f32 = undefined,
    cos_swing_z_half_max_angle: f32 = undefined,

    // RUN TIME PROPERTIES FOLLOW

    /// Rotation axis for the angle constraint parts
    world_space_swing_limit_y_rotation_axis: Vec3 = undefined,
    world_space_swing_limit_z_rotation_axis: Vec3 = undefined,
    world_space_twist_limit_rotation_axis: Vec3 = undefined,

    /// The constraint parts
    swing_limit_y_constraint_part: AngleConstraintPart = .{},
    swing_limit_z_constraint_part: AngleConstraintPart = .{},
    twist_limit_constraint_part: AngleConstraintPart = .{},

    /// Override the swing type
    pub fn setSwingType(self: *SwingTwistConstraintPart, swing_type: SwingType) void {
        self.swing_type = swing_type;
    }

    /// Get the swing type for this part
    pub fn getSwingType(self: *const SwingTwistConstraintPart) SwingType {
        return self.swing_type;
    }

    /// Set limits for this constraint (see description above for parameters)
    pub fn setLimits(self: *SwingTwistConstraintPart, twist_min_angle: f32, twist_max_angle: f32, swing_y_min_angle: f32, swing_y_max_angle: f32, swing_z_min_angle: f32, swing_z_max_angle: f32) void {
        const locked_angle = comptime math.degreesToRadians(0.5);
        const free_angle = comptime math.degreesToRadians(179.5);

        // Assume sane input
        if (Core.enable_asserts) std.debug.assert(twist_min_angle <= twist_max_angle);
        if (Core.enable_asserts) std.debug.assert(swing_y_min_angle <= swing_y_max_angle);
        if (Core.enable_asserts) std.debug.assert(swing_z_min_angle <= swing_z_max_angle);
        if (Core.enable_asserts) std.debug.assert(swing_y_min_angle >= -math.pi and swing_y_max_angle <= math.pi);
        if (Core.enable_asserts) std.debug.assert(swing_z_min_angle >= -math.pi and swing_z_max_angle <= math.pi);

        // Calculate the sine and cosine of the half angles
        const half_twist = Vec4.init(twist_min_angle, twist_max_angle, 0, 0).mulScalar(0.5);
        const twist = half_twist.sinCos();
        const half_swing = Vec4.init(swing_y_min_angle, swing_y_max_angle, swing_z_min_angle, swing_z_max_angle).mulScalar(0.5);
        const swing = half_swing.sinCos();

        // Store half angles for pyramid limit
        self.swing_y_half_min_angle = half_swing.getX();
        self.swing_y_half_max_angle = half_swing.getY();
        self.swing_z_half_min_angle = half_swing.getZ();
        self.swing_z_half_max_angle = half_swing.getW();

        // Store axis flags which are used at runtime to quickly decided which constraints to apply
        self.rotation_flags = 0;
        if (twist_min_angle > -locked_angle and twist_max_angle < locked_angle) {
            self.rotation_flags |= RotationFlags.twist_x_locked;
            self.sin_twist_half_min_angle = 0.0;
            self.sin_twist_half_max_angle = 0.0;
            self.cos_twist_half_min_angle = 1.0;
            self.cos_twist_half_max_angle = 1.0;
        } else if (twist_min_angle < -free_angle and twist_max_angle > free_angle) {
            self.rotation_flags |= RotationFlags.twist_x_free;
            self.sin_twist_half_min_angle = -1.0;
            self.sin_twist_half_max_angle = 1.0;
            self.cos_twist_half_min_angle = 0.0;
            self.cos_twist_half_max_angle = 0.0;
        } else {
            self.sin_twist_half_min_angle = twist.sin.getX();
            self.sin_twist_half_max_angle = twist.sin.getY();
            self.cos_twist_half_min_angle = twist.cos.getX();
            self.cos_twist_half_max_angle = twist.cos.getY();
        }

        if (swing_y_min_angle > -locked_angle and swing_y_max_angle < locked_angle) {
            self.rotation_flags |= RotationFlags.swing_y_locked;
            self.sin_swing_y_half_min_angle = 0.0;
            self.sin_swing_y_half_max_angle = 0.0;
            self.cos_swing_y_half_min_angle = 1.0;
            self.cos_swing_y_half_max_angle = 1.0;
        } else if (swing_y_min_angle < -free_angle and swing_y_max_angle > free_angle) {
            self.rotation_flags |= RotationFlags.swing_y_free;
            self.sin_swing_y_half_min_angle = -1.0;
            self.sin_swing_y_half_max_angle = 1.0;
            self.cos_swing_y_half_min_angle = 0.0;
            self.cos_swing_y_half_max_angle = 0.0;
        } else {
            self.sin_swing_y_half_min_angle = swing.sin.getX();
            self.sin_swing_y_half_max_angle = swing.sin.getY();
            self.cos_swing_y_half_min_angle = swing.cos.getX();
            self.cos_swing_y_half_max_angle = swing.cos.getY();
            if (Core.enable_asserts) std.debug.assert(self.sin_swing_y_half_min_angle <= self.sin_swing_y_half_max_angle);
        }

        if (swing_z_min_angle > -locked_angle and swing_z_max_angle < locked_angle) {
            self.rotation_flags |= RotationFlags.swing_z_locked;
            self.sin_swing_z_half_min_angle = 0.0;
            self.sin_swing_z_half_max_angle = 0.0;
            self.cos_swing_z_half_min_angle = 1.0;
            self.cos_swing_z_half_max_angle = 1.0;
        } else if (swing_z_min_angle < -free_angle and swing_z_max_angle > free_angle) {
            self.rotation_flags |= RotationFlags.swing_z_free;
            self.sin_swing_z_half_min_angle = -1.0;
            self.sin_swing_z_half_max_angle = 1.0;
            self.cos_swing_z_half_min_angle = 0.0;
            self.cos_swing_z_half_max_angle = 0.0;
        } else {
            self.sin_swing_z_half_min_angle = swing.sin.getZ();
            self.sin_swing_z_half_max_angle = swing.sin.getW();
            self.cos_swing_z_half_min_angle = swing.cos.getZ();
            self.cos_swing_z_half_max_angle = swing.cos.getW();
            if (Core.enable_asserts) std.debug.assert(self.sin_swing_z_half_min_angle <= self.sin_swing_z_half_max_angle);
        }
    }

    /// Helper function to determine if we're clamped against the min or max limit
    pub fn distanceToMinShorter(delta_min_in: f32, delta_max_in: f32) bool {
        // We're outside of the limits, get actual delta to min/max range
        // Note that a swing/twist of -1 and 1 represent the same angle, so if the difference is bigger than 1, the shortest angle is the other way around (2 - difference)
        // We should actually be working with angles rather than sin(angle / 2). When the difference is small the approximation is accurate, but
        // when working with extreme values the calculation is off and e.g. when the limit is between 0 and 180 a value of approx -60 will clamp
        // to 180 rather than 0 (you'd expect anything > -90 to go to 0).
        var delta_min = @abs(delta_min_in);
        if (delta_min > 1.0) delta_min = 2.0 - delta_min;
        var delta_max = @abs(delta_max_in);
        if (delta_max > 1.0) delta_max = 2.0 - delta_max;
        return delta_min < delta_max;
    }

    /// Clamp twist and swing against the constraint limits, returns which parts were clamped (everything assumed in constraint space)
    pub fn clampSwingTwist(self: *const SwingTwistConstraintPart, swing: *Quat, twist: *Quat) u32 {
        // Start with not clamped
        var clamped_axis: u32 = 0;

        // Check that swing and twist quaternions don't contain rotations around the wrong axis
        if (Core.enable_asserts) std.debug.assert(swing.getX() == 0.0);
        if (Core.enable_asserts) std.debug.assert(twist.getY() == 0.0);
        if (Core.enable_asserts) std.debug.assert(twist.getZ() == 0.0);

        // Ensure quaternions have w > 0
        const negate_swing = swing.getW() < 0.0;
        if (negate_swing)
            swing.* = swing.negate();
        const negate_twist = twist.getW() < 0.0;
        if (negate_twist)
            twist.* = twist.negate();

        if ((self.rotation_flags & RotationFlags.twist_x_locked) != 0) {
            // Twist axis is locked, clamp whenever twist is not identity
            clamped_axis |= if (twist.getX() != 0.0) (clamped_twist_min | clamped_twist_max) else 0;
            twist.* = Quat.identity();
        } else if ((self.rotation_flags & RotationFlags.twist_x_free) == 0) {
            // Twist axis has limit, clamp whenever out of range
            const delta_min = self.sin_twist_half_min_angle - twist.getX();
            const delta_max = twist.getX() - self.sin_twist_half_max_angle;
            if (delta_min > 0.0 or delta_max > 0.0) {
                // Pick the twist that corresponds to the smallest delta
                if (distanceToMinShorter(delta_min, delta_max)) {
                    twist.* = Quat.init(self.sin_twist_half_min_angle, 0, 0, self.cos_twist_half_min_angle);
                    clamped_axis |= clamped_twist_min;
                } else {
                    twist.* = Quat.init(self.sin_twist_half_max_angle, 0, 0, self.cos_twist_half_max_angle);
                    clamped_axis |= clamped_twist_max;
                }
            }
        }

        // Clamp swing
        if ((self.rotation_flags & RotationFlags.swing_y_locked) != 0) {
            if ((self.rotation_flags & RotationFlags.swing_z_locked) != 0) {
                // Both swing Y and Z are disabled, no degrees of freedom in swing
                clamped_axis |= if (swing.getY() != 0.0) (clamped_swing_y_min | clamped_swing_y_max) else 0;
                clamped_axis |= if (swing.getZ() != 0.0) (clamped_swing_z_min | clamped_swing_z_max) else 0;
                swing.* = Quat.identity();
            } else {
                // Swing Y angle disabled, only 1 degree of freedom in swing
                clamped_axis |= if (swing.getY() != 0.0) (clamped_swing_y_min | clamped_swing_y_max) else 0;
                const delta_min = self.sin_swing_z_half_min_angle - swing.getZ();
                const delta_max = swing.getZ() - self.sin_swing_z_half_max_angle;
                if (delta_min > 0.0 or delta_max > 0.0) {
                    // Pick the swing that corresponds to the smallest delta
                    if (distanceToMinShorter(delta_min, delta_max)) {
                        swing.* = Quat.init(0, 0, self.sin_swing_z_half_min_angle, self.cos_swing_z_half_min_angle);
                        clamped_axis |= clamped_swing_z_min;
                    } else {
                        swing.* = Quat.init(0, 0, self.sin_swing_z_half_max_angle, self.cos_swing_z_half_max_angle);
                        clamped_axis |= clamped_swing_z_max;
                    }
                } else if ((clamped_axis & clamped_swing_y_min) != 0) {
                    const z = swing.getZ();
                    swing.* = Quat.init(0, 0, z, math.sqrt(1.0 - math.square(z)));
                }
            }
        } else if ((self.rotation_flags & RotationFlags.swing_z_locked) != 0) {
            // Swing Z angle disabled, only 1 degree of freedom in swing
            clamped_axis |= if (swing.getZ() != 0.0) (clamped_swing_z_min | clamped_swing_z_max) else 0;
            const delta_min = self.sin_swing_y_half_min_angle - swing.getY();
            const delta_max = swing.getY() - self.sin_swing_y_half_max_angle;
            if (delta_min > 0.0 or delta_max > 0.0) {
                // Pick the swing that corresponds to the smallest delta
                if (distanceToMinShorter(delta_min, delta_max)) {
                    swing.* = Quat.init(0, self.sin_swing_y_half_min_angle, 0, self.cos_swing_y_half_min_angle);
                    clamped_axis |= clamped_swing_y_min;
                } else {
                    swing.* = Quat.init(0, self.sin_swing_y_half_max_angle, 0, self.cos_swing_y_half_max_angle);
                    clamped_axis |= clamped_swing_y_max;
                }
            } else if ((clamped_axis & clamped_swing_z_min) != 0) {
                const y = swing.getY();
                swing.* = Quat.init(0, y, 0, math.sqrt(1.0 - math.square(y)));
            }
        } else {
            // Two degrees of freedom
            if (self.swing_type == .cone) {
                // Use ellipse to solve limits
                const ellipse = Ellipse.init(self.sin_swing_y_half_max_angle, self.sin_swing_z_half_max_angle);
                const point = Float2.init(swing.getY(), swing.getZ());
                if (!ellipse.isInside(point)) {
                    const closest = ellipse.getClosestPoint(point);
                    swing.* = Quat.init(0, closest.x, closest.y, math.sqrt(math.max(@as(f32, 0.0), 1.0 - math.square(closest.x) - math.square(closest.y))));
                    clamped_axis |= clamped_swing_y_min | clamped_swing_y_max | clamped_swing_z_min | clamped_swing_z_max; // We're not using the flags on which side we got clamped here
                }
            } else {
                // Use pyramid to solve limits
                // The quaternion rotating by angle y around the Y axis then rotating by angle z around the Z axis is:
                // q = Quat::sRotation(Vec3::sAxisZ(), z) * Quat::sRotation(Vec3::sAxisY(), y)
                // [q.x, q.y, q.z, q.w] = [-sin(y / 2) * sin(z / 2), sin(y / 2) * cos(z / 2), cos(y / 2) * sin(z / 2), cos(y / 2) * cos(z / 2)]
                // So we can calculate y / 2 = atan2(q.y, q.w) and z / 2 = atan2(q.z, q.w)
                const half_angle = Vec4.atan2(swing.getXYZW().swizzle(.y, .y, .z, .z), swing.getXYZW().splatW());
                const min_half_angle = Vec4.init(self.swing_y_half_min_angle, self.swing_y_half_min_angle, self.swing_z_half_min_angle, self.swing_z_half_min_angle);
                const max_half_angle = Vec4.init(self.swing_y_half_max_angle, self.swing_y_half_max_angle, self.swing_z_half_max_angle, self.swing_z_half_max_angle);
                const clamped_half_angle = Vec4.min(Vec4.max(half_angle, min_half_angle), max_half_angle);
                const unclamped = Vec4.equals(half_angle, clamped_half_angle);
                if (!unclamped.testAllTrue()) {
                    // We now calculate the quaternion again using the formula for q above,
                    // but we leave out the x component in order to not introduce twist
                    const sc = clamped_half_angle.sinCos();
                    const s = sc.sin;
                    const c = sc.cos;
                    swing.* = Quat.init(0, s.getY() * c.getZ(), c.getY() * s.getZ(), c.getY() * c.getZ()).normalized();
                    clamped_axis |= clamped_swing_y_min | clamped_swing_y_max | clamped_swing_z_min | clamped_swing_z_max; // We're not using the flags on which side we got clamped here
                }
            }
        }

        // Flip sign back
        if (negate_swing)
            swing.* = swing.negate();
        if (negate_twist)
            twist.* = twist.negate();

        if (Core.enable_asserts) std.debug.assert(swing.isNormalized(.{}));
        if (Core.enable_asserts) std.debug.assert(twist.isNormalized(.{}));

        return clamped_axis;
    }

    /// Calculate properties used during the functions below
    /// @param inBody1 The first body that this constraint is attached to
    /// @param inBody2 The second body that this constraint is attached to
    /// @param inConstraintRotation The current rotation of the constraint in constraint space
    /// @param inConstraintToWorld Rotates from constraint space into world space
    pub fn calculateConstraintProperties(self: *SwingTwistConstraintPart, body1: *const Body, body2: *const Body, constraint_rotation: Quat, constraint_to_world: Quat) void {
        // Decompose into swing and twist
        const swing_twist = constraint_rotation.getSwingTwist();
        const q_swing = swing_twist.swing;
        const q_twist = swing_twist.twist;

        // Clamp against joint limits
        var q_clamped_swing = q_swing;
        var q_clamped_twist = q_twist;
        const clamped_axis = self.clampSwingTwist(&q_clamped_swing, &q_clamped_twist);

        if ((self.rotation_flags & RotationFlags.swing_y_locked) != 0) {
            const twist_to_world = constraint_to_world.mul(q_swing);
            self.world_space_swing_limit_y_rotation_axis = twist_to_world.rotateAxisY();
            self.world_space_swing_limit_z_rotation_axis = twist_to_world.rotateAxisZ();

            if ((self.rotation_flags & RotationFlags.swing_z_locked) != 0) {
                // Swing fully locked
                self.swing_limit_y_constraint_part.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_y_rotation_axis, .{});
                self.swing_limit_z_constraint_part.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_z_rotation_axis, .{});
            } else {
                // Swing only locked around Y
                self.swing_limit_y_constraint_part.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_y_rotation_axis, .{});
                if ((clamped_axis & (clamped_swing_z_min | clamped_swing_z_max)) != 0) {
                    if ((clamped_axis & clamped_swing_z_min) != 0)
                        self.world_space_swing_limit_z_rotation_axis = self.world_space_swing_limit_z_rotation_axis.negate(); // Flip axis if hitting min limit because the impulse limit is going to be between [-FLT_MAX, 0]
                    self.swing_limit_z_constraint_part.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_z_rotation_axis, .{});
                } else self.swing_limit_z_constraint_part.deactivate();
            }
        } else if ((self.rotation_flags & RotationFlags.swing_z_locked) != 0) {
            // Swing only locked around Z
            const twist_to_world = constraint_to_world.mul(q_swing);
            self.world_space_swing_limit_y_rotation_axis = twist_to_world.rotateAxisY();
            self.world_space_swing_limit_z_rotation_axis = twist_to_world.rotateAxisZ();

            if ((clamped_axis & (clamped_swing_y_min | clamped_swing_y_max)) != 0) {
                if ((clamped_axis & clamped_swing_y_min) != 0)
                    self.world_space_swing_limit_y_rotation_axis = self.world_space_swing_limit_y_rotation_axis.negate(); // Flip axis if hitting min limit because the impulse limit is going to be between [-FLT_MAX, 0]
                self.swing_limit_y_constraint_part.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_y_rotation_axis, .{});
            } else self.swing_limit_y_constraint_part.deactivate();
            self.swing_limit_z_constraint_part.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_z_rotation_axis, .{});
        } else if ((self.rotation_flags & RotationFlags.swing_yz_free) != RotationFlags.swing_yz_free) {
            // Swing has limits around Y and Z
            if ((clamped_axis & (clamped_swing_y_min | clamped_swing_y_max | clamped_swing_z_min | clamped_swing_z_max)) != 0) {
                // Calculate axis of rotation from clamped swing to swing
                const current = constraint_to_world.mul(q_swing).rotateAxisX();
                const desired = constraint_to_world.mul(q_clamped_swing).rotateAxisX();
                self.world_space_swing_limit_y_rotation_axis = desired.cross(current);
                const len = self.world_space_swing_limit_y_rotation_axis.length();
                if (len != 0.0) {
                    self.world_space_swing_limit_y_rotation_axis = self.world_space_swing_limit_y_rotation_axis.divScalar(len);
                    self.swing_limit_y_constraint_part.calculateConstraintProperties(body1, body2, self.world_space_swing_limit_y_rotation_axis, .{});
                } else self.swing_limit_y_constraint_part.deactivate();
            } else self.swing_limit_y_constraint_part.deactivate();
            self.swing_limit_z_constraint_part.deactivate();
        } else {
            // No swing limits
            self.swing_limit_y_constraint_part.deactivate();
            self.swing_limit_z_constraint_part.deactivate();
        }

        if ((self.rotation_flags & RotationFlags.twist_x_locked) != 0) {
            // Twist locked, always activate constraint
            self.world_space_twist_limit_rotation_axis = constraint_to_world.mul(q_swing).rotateAxisX();
            self.twist_limit_constraint_part.calculateConstraintProperties(body1, body2, self.world_space_twist_limit_rotation_axis, .{});
        } else if ((self.rotation_flags & RotationFlags.twist_x_free) == 0) {
            // Twist has limits
            if ((clamped_axis & (clamped_twist_min | clamped_twist_max)) != 0) {
                self.world_space_twist_limit_rotation_axis = constraint_to_world.mul(q_swing).rotateAxisX();
                if ((clamped_axis & clamped_twist_min) != 0)
                    self.world_space_twist_limit_rotation_axis = self.world_space_twist_limit_rotation_axis.negate(); // Flip axis if hitting min limit because the impulse limit is going to be between [-FLT_MAX, 0]
                self.twist_limit_constraint_part.calculateConstraintProperties(body1, body2, self.world_space_twist_limit_rotation_axis, .{});
            } else self.twist_limit_constraint_part.deactivate();
        } else {
            // No twist limits
            self.twist_limit_constraint_part.deactivate();
        }
    }

    /// Deactivate this constraint
    pub fn deactivate(self: *SwingTwistConstraintPart) void {
        self.swing_limit_y_constraint_part.deactivate();
        self.swing_limit_z_constraint_part.deactivate();
        self.twist_limit_constraint_part.deactivate();
    }

    /// Check if constraint is active
    pub fn isActive(self: *const SwingTwistConstraintPart) bool {
        return self.swing_limit_y_constraint_part.isActive() or self.swing_limit_z_constraint_part.isActive() or self.twist_limit_constraint_part.isActive();
    }

    /// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
    pub fn warmStart(self: *SwingTwistConstraintPart, body1: *Body, body2: *Body, warm_start_impulse_ratio: f32) void {
        self.swing_limit_y_constraint_part.warmStart(body1, body2, warm_start_impulse_ratio);
        self.swing_limit_z_constraint_part.warmStart(body1, body2, warm_start_impulse_ratio);
        self.twist_limit_constraint_part.warmStart(body1, body2, warm_start_impulse_ratio);
    }

    /// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
    pub fn solveVelocityConstraint(self: *SwingTwistConstraintPart, body1: *Body, body2: *Body) bool {
        var impulse = false;

        // Solve swing constraint
        if (self.swing_limit_y_constraint_part.isActive()) {
            if (self.swing_limit_y_constraint_part.solveVelocityConstraint(body1, body2, self.world_space_swing_limit_y_rotation_axis, -math.flt_max, if (self.sin_swing_y_half_min_angle == self.sin_swing_y_half_max_angle) math.flt_max else 0.0))
                impulse = true;
        }

        if (self.swing_limit_z_constraint_part.isActive()) {
            if (self.swing_limit_z_constraint_part.solveVelocityConstraint(body1, body2, self.world_space_swing_limit_z_rotation_axis, -math.flt_max, if (self.sin_swing_z_half_min_angle == self.sin_swing_z_half_max_angle) math.flt_max else 0.0))
                impulse = true;
        }

        // Solve twist constraint
        if (self.twist_limit_constraint_part.isActive()) {
            if (self.twist_limit_constraint_part.solveVelocityConstraint(body1, body2, self.world_space_twist_limit_rotation_axis, -math.flt_max, if (self.sin_twist_half_min_angle == self.sin_twist_half_max_angle) math.flt_max else 0.0))
                impulse = true;
        }

        return impulse;
    }

    /// Iteratively update the position constraint. Makes sure C(...) = 0.
    /// @param ioBody1 The first body that this constraint is attached to
    /// @param ioBody2 The second body that this constraint is attached to
    /// @param inConstraintRotation The current rotation of the constraint in constraint space
    /// @param inConstraintToBody1 , inConstraintToBody2 Rotates from constraint space to body 1/2 space
    /// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
    pub fn solvePositionConstraint(self: *const SwingTwistConstraintPart, body1: *Body, body2: *Body, constraint_rotation: Quat, constraint_to_body1: Quat, constraint_to_body2: Quat, baumgarte: f32) bool {
        const swing_twist = constraint_rotation.getSwingTwist();
        var q_swing = swing_twist.swing;
        var q_twist = swing_twist.twist;

        const clamped_axis = self.clampSwingTwist(&q_swing, &q_twist);

        // Solve rotation violations
        if (clamped_axis != 0) {
            var part: RotationEulerConstraintPart = .{};
            const inv_initial_orientation = constraint_to_body2.mul(constraint_to_body1.mul(q_swing).mul(q_twist).conjugated());
            part.calculateConstraintProperties(body1, Mat44.rotationQuat(body1.getRotation()), body2, Mat44.rotationQuat(body2.getRotation()));
            return part.solvePositionConstraint(body1, body2, inv_initial_orientation, baumgarte);
        }

        return false;
    }

    /// Return lagrange multiplier for swing
    pub fn getTotalSwingYLambda(self: *const SwingTwistConstraintPart) f32 {
        return self.swing_limit_y_constraint_part.getTotalLambda();
    }

    pub fn getTotalSwingZLambda(self: *const SwingTwistConstraintPart) f32 {
        return self.swing_limit_z_constraint_part.getTotalLambda();
    }

    /// Return lagrange multiplier for twist
    pub fn getTotalTwistLambda(self: *const SwingTwistConstraintPart) f32 {
        return self.twist_limit_constraint_part.getTotalLambda();
    }

    /// Save state of this constraint part
    pub fn saveState(self: *const SwingTwistConstraintPart, stream: *StateRecorder) void {
        self.swing_limit_y_constraint_part.saveState(stream);
        self.swing_limit_z_constraint_part.saveState(stream);
        self.twist_limit_constraint_part.saveState(stream);
    }

    /// Restore state of this constraint part
    pub fn restoreState(self: *SwingTwistConstraintPart, stream: *StateRecorder) void {
        self.swing_limit_y_constraint_part.restoreState(stream);
        self.swing_limit_z_constraint_part.restoreState(stream);
        self.twist_limit_constraint_part.restoreState(stream);
    }
};

const TestContext = @import("ConstraintPartTestContext.zig");

test "SwingTwistConstraintPart limits and clamping" {
    const testing = std.testing;
    const Part = SwingTwistConstraintPart;

    var part: Part = .{};
    try testing.expectEqual(SwingType.cone, part.getSwingType());
    part.setSwingType(.pyramid);
    try testing.expectEqual(SwingType.pyramid, part.getSwingType());
    part.setSwingType(.cone);

    // Locked, free and limited axis
    part.setLimits(-0.001, 0.001, -math.pi, math.pi, -0.5, 0.25);
    try testing.expectEqual(Part.RotationFlags.twist_x_locked | Part.RotationFlags.swing_y_free, part.rotation_flags);
    try testing.expectEqual(@as(f32, -1.0), part.sin_swing_y_half_min_angle);
    try testing.expectEqual(@as(f32, 0.0), part.cos_swing_y_half_max_angle);
    try testing.expectEqual(@as(f32, 0.0), part.sin_twist_half_max_angle);
    try testing.expectApproxEqAbs(@sin(@as(f32, -0.25)), part.sin_swing_z_half_min_angle, 1.0e-6);
    try testing.expectApproxEqAbs(@cos(@as(f32, 0.125)), part.cos_swing_z_half_max_angle, 1.0e-6);
    try testing.expectEqual(@as(f32, -0.25), part.swing_z_half_min_angle);

    part.setLimits(-math.pi, math.pi, -0.001, 0.0, 0.0, 0.001);
    try testing.expectEqual(Part.RotationFlags.twist_x_free | Part.RotationFlags.swing_y_locked | Part.RotationFlags.swing_z_locked, part.rotation_flags);

    // Distance helper: -1 and 1 are the same angle
    try testing.expect(Part.distanceToMinShorter(0.1, 0.2));
    try testing.expect(!Part.distanceToMinShorter(0.3, -0.2));
    try testing.expect(Part.distanceToMinShorter(1.9, 0.2)); // 2 - 1.9 = 0.1 < 0.2
    try testing.expect(!Part.distanceToMinShorter(0.2, -1.9));
    try testing.expect(Part.distanceToMinShorter(1.95, 0.1));

    // Twist limited to [-0.4, 0.4]: a twist of 0.6 is clamped to the max
    part.setLimits(-0.4, 0.4, -0.5, 0.5, -0.5, 0.5);
    var swing = Quat.identity();
    var twist = Quat.rotation(Vec3.axisX(), 0.6);
    var clamped = part.clampSwingTwist(&swing, &twist);
    try testing.expectEqual(Part.clamped_twist_max, clamped);
    try testing.expect(twist.isClose(Quat.rotation(Vec3.axisX(), 0.4), .{ .max_dist_sq = 1.0e-12 }));

    // A negative w is handled by negating and flipping back
    twist = Quat.rotation(Vec3.axisX(), -0.6).negate();
    swing = Quat.identity();
    clamped = part.clampSwingTwist(&swing, &twist);
    try testing.expectEqual(Part.clamped_twist_min, clamped);
    try testing.expect(twist.getW() < 0.0);

    // Cone: outside the ellipse
    swing = Quat.rotation(Vec3.init(0, 1, 1).normalized(), 1.0);
    twist = Quat.identity();
    clamped = part.clampSwingTwist(&swing, &twist);
    try testing.expectEqual(Part.clamped_swing_y_min | Part.clamped_swing_y_max | Part.clamped_swing_z_min | Part.clamped_swing_z_max, clamped);
    try testing.expectEqual(@as(f32, 0.0), swing.getX());

    // Pyramid: outside the box
    part.setSwingType(.pyramid);
    swing = Quat.rotation(Vec3.init(0, 1, 1).normalized(), 1.0);
    clamped = part.clampSwingTwist(&swing, &twist);
    try testing.expectEqual(Part.clamped_swing_y_min | Part.clamped_swing_y_max | Part.clamped_swing_z_min | Part.clamped_swing_z_max, clamped);

    // Inside: nothing clamped, nothing changed
    swing = Quat.rotation(Vec3.axisY(), 0.1);
    twist = Quat.rotation(Vec3.axisX(), 0.1);
    const swing_before = swing;
    clamped = part.clampSwingTwist(&swing, &twist);
    try testing.expectEqual(@as(u32, 0), clamped);
    try testing.expect(swing.eql(swing_before));

    // Swing Y locked, Z limited: the Y rotation is removed
    part.setLimits(-0.4, 0.4, 0.0, 0.0, -0.5, 0.5);
    swing = Quat.init(0, 0.1, 0.1, 1).normalized();
    clamped = part.clampSwingTwist(&swing, &twist);
    try testing.expectEqual(Part.clamped_swing_y_min | Part.clamped_swing_y_max, clamped);
    try testing.expectEqual(@as(f32, 0.0), swing.getY());

    // Swing Z locked, Y limited and exceeded
    part.setLimits(-0.4, 0.4, -0.5, 0.5, 0.0, 0.0);
    swing = Quat.rotation(Vec3.axisY(), -1.5);
    clamped = part.clampSwingTwist(&swing, &twist);
    try testing.expectEqual(Part.clamped_swing_y_min, clamped);
}

test "SwingTwistConstraintPart for every combination of motion types" {
    const testing = std.testing;
    const StateRecorderImpl = @import("../../StateRecorderImpl.zig").StateRecorderImpl;

    var context: TestContext.ConstraintPartTestContext = undefined;
    try context.init(testing.allocator, testing.io);
    defer context.deinit();

    const LimitCase = struct { limits: [6]f32, swing_type: SwingType };
    const cases = [_]LimitCase{
        .{ .limits = .{ -0.2, 0.2, -0.3, 0.3, -0.3, 0.3 }, .swing_type = .cone },
        .{ .limits = .{ -0.2, 0.2, -0.3, 0.3, -0.4, 0.2 }, .swing_type = .pyramid },
        .{ .limits = .{ 0.0, 0.0, 0.0, 0.0, -0.3, 0.3 }, .swing_type = .cone },
        .{ .limits = .{ -0.2, 0.2, -0.3, 0.3, 0.0, 0.0 }, .swing_type = .cone },
        .{ .limits = .{ -0.2, 0.2, 0.0, 0.0, 0.0, 0.0 }, .swing_type = .cone },
        .{ .limits = .{ -math.pi, math.pi, -math.pi, math.pi, -math.pi, math.pi }, .swing_type = .cone },
    };

    // A constraint rotation outside of most of the limits
    const constraint_rotation = Quat.rotation(Vec3.init(0.6, 1, -0.8).normalized(), 0.9);
    const constraint_to_body1 = Quat.rotation(Vec3.axisZ(), 0.3);
    const constraint_to_body2 = Quat.rotation(Vec3.axisX(), -0.2);

    for (TestContext.motion_type_pairs) |pair| {
        for (cases) |case| {
            const bodies = try context.createBodyPair(pair);
            defer context.destroyBodies(&bodies);
            const body1 = bodies[0];
            const body2 = bodies[1];
            const before1 = TestContext.BodySnapshot.take(body1);
            const before2 = TestContext.BodySnapshot.take(body2);

            var part: SwingTwistConstraintPart = .{};
            part.setSwingType(case.swing_type);
            part.setLimits(case.limits[0], case.limits[1], case.limits[2], case.limits[3], case.limits[4], case.limits[5]);
            part.calculateConstraintProperties(body1, body2, constraint_rotation, body1.getRotation().mul(constraint_to_body1));

            const any_dynamic = body1.isDynamic() or body2.isDynamic();
            const free = case.limits[0] == -math.pi;
            try testing.expectEqual(any_dynamic and !free, part.isActive());
            if (!part.isActive()) {
                try testing.expect(!part.solveVelocityConstraint(body1, body2));
                continue;
            }

            _ = part.solveVelocityConstraint(body1, body2); // A one sided limit may clamp the impulse to zero
            part.warmStart(body1, body2, 1.0);
            try testing.expect(part.solvePositionConstraint(body1, body2, constraint_rotation, constraint_to_body1, constraint_to_body2, 0.2));
            try TestContext.checkUnmoved(body1, before1);
            try TestContext.checkUnmoved(body2, before2);

            // The impulses respect the one sided limits
            if (case.limits[2] != case.limits[3]) try testing.expect(part.getTotalSwingYLambda() <= 0.0);
            if (case.limits[4] != case.limits[5]) try testing.expect(part.getTotalSwingZLambda() <= 0.0);
            if (case.limits[0] != case.limits[1]) try testing.expect(part.getTotalTwistLambda() <= 0.0);

            // State round trip
            var recorder = StateRecorderImpl.init(testing.allocator);
            defer recorder.deinit();
            part.saveState(recorder.recorder());
            try testing.expectEqual(@as(usize, 12), recorder.getData().len);
            var restored: SwingTwistConstraintPart = .{};
            restored.restoreState(recorder.recorder());
            try testing.expectEqual(part.getTotalSwingYLambda(), restored.getTotalSwingYLambda());
            try testing.expectEqual(part.getTotalSwingZLambda(), restored.getTotalSwingZLambda());
            try testing.expectEqual(part.getTotalTwistLambda(), restored.getTotalTwistLambda());

            part.deactivate();
            try testing.expect(!part.isActive());
        }
    }

    // Inside the limits: no position correction
    const bodies = try context.createBodyPair(.{ .dynamic, .dynamic });
    var part: SwingTwistConstraintPart = .{};
    part.setLimits(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5);
    try testing.expect(!part.solvePositionConstraint(bodies[0], bodies[1], Quat.rotation(Vec3.axisY(), 0.1), Quat.identity(), Quat.identity(), 0.2));
}
